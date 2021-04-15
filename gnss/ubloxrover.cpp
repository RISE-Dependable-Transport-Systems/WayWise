#include "ubloxrover.h"
#include <QDebug>

UbloxRover::UbloxRover(QObject *parent) : QObject(parent)
{
    connect(&mUblox, &Ublox::rxEsfMeas, [](const ubx_esf_meas &meas){qDebug() << meas.data_field[0];});
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
