#include "pozyxpositionupdater.h"
#include <QDebug>
#include <QtEndian>

PozyxPositionUpdater::PozyxPositionUpdater(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;

    connect(&mSerialPort, &QSerialPort::readyRead, [&]() {
        while (mSerialPort.canReadLine()) {
            QString receivedString(mSerialPort.readLine());

            // get raw data from result string (format: D,%data%\r)
            QString receivedData = receivedString.split(',').last().trimmed();
//            qDebug() << receivedData;

            static double heading = 0;
            static double xyz[3] = {0,0,0};
            if (receivedData.length() == 4) { // POZYX_EUL_HEADING
                heading = qFromBigEndian((int16_t) receivedData.toUInt(nullptr, 16)) / 16.0;
            } else if (receivedData.length() == 24) { // POZYX_POS_XYZ
                xyz[0] = qFromBigEndian((int32_t) receivedData.mid( 0,8).toUInt(nullptr, 16)) / 1000.0;
                xyz[1] = qFromBigEndian((int32_t) receivedData.mid( 8,8).toUInt(nullptr, 16)) / 1000.0;
                xyz[2] = qFromBigEndian((int32_t) receivedData.mid(16,8).toUInt(nullptr, 16)) / 1000.0;

                // Trigger new positioning
                QString pollPozyxStr;
                pollPozyxStr.sprintf("F,%.2x,,1\r", POZYX_DO_POSITIONING);
                mSerialPort.write(pollPozyxStr.toLatin1());
                mSerialPort.flush();
            } else if (receivedData.length() == 2) { // POZYX_DO_POSITIONING
                if (receivedData.compare("01") != 0)
                    qDebug() << "Warning: PozyxPositionUpdater could not trigger position update.";
            } else
                qDebug() << "Warning: PozyxPositionUpdater could not parse incoming data.";

//            qDebug() /*<< result*/ << heading << xyz[0] << xyz[1] << xyz[2];

            PosPoint currUWBpos = mVehicleState->getPosition(PosType::UWB);
            currUWBpos.setX(xyz[0]);
            currUWBpos.setY(xyz[1]);
            currUWBpos.setHeight(xyz[2]);
            currUWBpos.setYaw(heading);
            currUWBpos.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
            mVehicleState->setPosition(currUWBpos);
        }

        emit updatedUWBPositionAndYaw(mVehicleState);
    });

    connect(&mPollPositionTimer, &QTimer::timeout, this, [this](){
        QString pollPozyxStr;
        static int i = 0;

        pollPozyxStr.sprintf("R,%.2x,%i\r", POZYX_EUL_HEADING, POZYX_EUL_HEADING_size);
        mSerialPort.write(pollPozyxStr.toLatin1());
        mSerialPort.flush();

        // poll position every POLL_HEADINGS_PER_POSITIONth time
        if (i == 0) {
            pollPozyxStr.sprintf("R,%.2x,%i\r", POZYX_POS_XYZ, POZYX_POS_XYZ_size);
            mSerialPort.write(pollPozyxStr.toLatin1());
            mSerialPort.flush();
        }
        i = (i + 1) % POLL_HEADINGS_PER_POSITION;
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

    mPollPositionTimer.start(25);

    return true;
}

bool PozyxPositionUpdater::isSerialConnected()
{
    return mSerialPort.isOpen() && mSerialPort.isWritable();
}
