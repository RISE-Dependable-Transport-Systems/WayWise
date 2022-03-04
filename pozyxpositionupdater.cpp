#include "pozyxpositionupdater.h"
#include <QDebug>
#include <QtEndian>

PozyxPositionUpdater::PozyxPositionUpdater(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;

    connect(&mSerialPort, &QSerialPort::readyRead, [&]() {
        QString receivedData;
        while (mSerialPort.bytesAvailable())
            receivedData.append(mSerialPort.readAll());

        // get raw data from result string (format: D,%data%\r)
        QString result = receivedData.split(',').last().trimmed();

        static double heading = 0;
        static double xyz[3] = {0,0,0};
        if (mLastPolled == POZYX_EUL_HEADING)
             heading = qFromBigEndian((int16_t) result.toUInt(nullptr, 16)) / 16.0;
        else if (mLastPolled == POZYX_POS_XYZ) {
            xyz[0] = qFromBigEndian((int32_t) result.mid( 0,8).toUInt(nullptr, 16)) / 1000.0;
            xyz[1] = qFromBigEndian((int32_t) result.mid( 8,8).toUInt(nullptr, 16)) / 1000.0;
            xyz[2] = qFromBigEndian((int32_t) result.mid(16,8).toUInt(nullptr, 16)) / 1000.0;
        }
//        qDebug() /*<< result*/ << heading << xyz[0] << xyz[1] << xyz[2];

        PosPoint currUWBpos = mVehicleState->getPosition(PosType::UWB);
        currUWBpos.setX(xyz[0]);
        currUWBpos.setY(xyz[1]);
        currUWBpos.setHeight(xyz[2]);
        currUWBpos.setYaw(heading);
        currUWBpos.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
        mVehicleState->setPosition(currUWBpos);

        emit updatedUWBPositionAndYaw(mVehicleState);
    });

    connect(&mPollPositionTimer, &QTimer::timeout, this, [this](){
        static int i = 0;
        QString pollPozyxStr;

        // alternatingly trigger positioning or poll yaw/position
        if (i == 0) {
            pollPozyxStr.sprintf("F,%.2x,,1\r", POZYX_DO_POSITIONING); // TODO: move to separate timer
            mLastPolled = POZYX_DO_POSITIONING;
        } else if (i == 1) {
            pollPozyxStr.sprintf("R,%.2x,%i\r", POZYX_EUL_HEADING, POZYX_EUL_HEADING_size);
            mLastPolled = POZYX_EUL_HEADING;
        } else if (i == 2) {
            pollPozyxStr.sprintf("R,%.2x,%i\r", POZYX_POS_XYZ, POZYX_POS_XYZ_size);
            mLastPolled = POZYX_POS_XYZ;
        }
        mSerialPort.write(pollPozyxStr.toLatin1());
        mSerialPort.flush();

        i = (i + 1) % 3;
    });
}

bool PozyxPositionUpdater::connectSerial(const QSerialPortInfo &serialPortInfo)
{
    if(mSerialPort.isOpen()) {
        mSerialPort.close();
    }

    mSerialPort.setPort(serialPortInfo);
    mSerialPort.open(QIODevice::ReadWrite);

    if(!mSerialPort.isOpen()) {
        return false;
    }

    mSerialPort.setBaudRate(115200);
    mSerialPort.setDataBits(QSerialPort::DataBits::Data8);
    mSerialPort.setParity(QSerialPort::Parity::NoParity);
    mSerialPort.setStopBits(QSerialPort::StopBits::OneStop);

    mPollPositionTimer.start(30);

    return true;
}

bool PozyxPositionUpdater::isSerialConnected()
{
    return mSerialPort.isOpen() && mSerialPort.isWritable();
}
