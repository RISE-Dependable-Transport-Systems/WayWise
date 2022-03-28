#ifndef POZYXPOSITIONUPDATER_H
#define POZYXPOSITIONUPDATER_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QSharedPointer>
#include "vehiclestate.h"

class PozyxPositionUpdater : public QObject
{
    Q_OBJECT
public:
    explicit PozyxPositionUpdater(QSharedPointer<VehicleState> vehicleState);


    bool connectSerial(const QSerialPortInfo &serialPortInfo);
    bool isSerialConnected();

signals:
    void updatedUWBPositionAndYaw(QSharedPointer<VehicleState> vehicleState);

private:
    const int POZYX_POS_XYZ = 0x30;         // x,y,z-coordinates [mm]
    const int POZYX_POS_XYZ_size = 12;      //  reply size in bytes
    const int POZYX_EUL_HEADING = 0x66;     // Euler angles of yaw
    const int POZYX_EUL_HEADING_size = 2;   // reply size in bytes
    const int POZYX_DO_POSITIONING = 0xB6;   // Initiate positioning process

    QSerialPort mSerialPort;
    QTimer mPollPositionTimer;
    const int POLL_HEADINGS_PER_POSITION = 2;
    QSharedPointer<VehicleState> mVehicleState;

};

#endif // POZYXPOSITIONUPDATER_H