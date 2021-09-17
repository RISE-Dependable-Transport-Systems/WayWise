#ifndef CANOPENMOTORCONTROLLER_H
#define CANOPENMOTORCONTROLLER_H

#include <QObject>
#include "sdvp_qtcommon/motorcontroller.h"

class CANopenMotorController : public MotorController
{
    Q_OBJECT

public:
    CANopenMotorController();
    ~CANopenMotorController();

    virtual void pollFirmwareVersion();
    virtual void requestRPM(int32_t rpm);

    void setCommandSpeed(qint8 speed);
    void setCommandSteering(qint16 steering);

    qint16 getActualSteering();
    qint8 getActualSpeed();
    quint8 getStatus();
    quint8 getBatterySOC();
    quint16 getBatteryVoltage();

public slots:
    void actualSpeedReceived(qint8 speed);
    void actualSteeringReceived(qint16 steering);
    void statusReceived(quint8 status);
    void batterySOCReceived(quint8 batterysoc);
    void batteryVoltageReceived(quint16 batteryvoltage);

signals:
    void sendCommandSpeed(qint8 speed);
    void sendCommandSteering(qint16 steering);

private:
    qint8 mActualSpeed;
    qint16 mActualSteering;
    quint8 mStatus;
    quint8 mBatterySOC;
    quint16 mBatteryVoltage;
};

#endif // CANOPENMOTORCONTROLLER_H
