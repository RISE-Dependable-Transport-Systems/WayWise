#include "ubloxrover.h"
#include <QDebug>

UbloxRover::UbloxRover(QObject *parent) : QObject(parent)
{
    connect(&mUblox, &Ublox::rxEsfMeas, [](const ubx_esf_meas &meas){
        for (int i = 0; i < meas.num_meas; i++)
            switch(meas.data_type[i]) {
            case GYRO_Z:
                qDebug() << "gz" << esfMeas2Dbl(meas.data_field[i], 12);
                break;
            case GYRO_Y:
                qDebug() << "gy" << esfMeas2Dbl(meas.data_field[i], 12);
                break;
            case GYRO_X:
                qDebug() << "gx" << esfMeas2Dbl(meas.data_field[i], 12);
                break;
            case ACC_X:
                qDebug() << "ax" << esfMeas2Dbl(meas.data_field[i], 10);
                break;
            case ACC_Y:
                qDebug() << "ay" << esfMeas2Dbl(meas.data_field[i], 10);
                break;
            case ACC_Z:
                qDebug() << "az" << esfMeas2Dbl(meas.data_field[i], 10);
                break;
            }


        qDebug() << "";
    });
}

bool UbloxRover::connectSerial(const QSerialPortInfo &serialPortInfo)
{
    if (mUblox.connectSerial(serialPortInfo))
        if (configureUblox())
            return true;
        else {
            mUblox.disconnectSerial();
            return false;
        }
    else
        return false;
}

bool UbloxRover::configureUblox()
{
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_MEAS, 1);
    return true;
}
