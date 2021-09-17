#ifndef CANOPENMOVEMENTCONTROLLER_H
#define CANOPENMOVEMENTCONTROLLER_H

#include <QObject>
#include "sdvp_qtcommon/movementcontroller.h"
#include "sdvp_qtcommon/diffdrivevehiclestate.h"
#include <QSharedPointer>

class CANopenMovementController : public MovementController
{
    Q_OBJECT
public:
    CANopenMovementController(QSharedPointer<DiffDriveVehicleState> vehicleState);

    // MovementController interface
    virtual void setDesiredSpeed(double desiredSpeed) override;
    virtual void setDesiredSteeringCurvature(double desiredSteeringAngle) override;
    virtual void setDesiredSteering(double desiredSteering) override;
    virtual void setDesiredAttributes(quint32 desiredAttributes) override;

    double getActualSteeringCurvature();
    double getActualSpeed();
    quint8 getStatus();
    double getBatterySOC();
    double getBatteryVoltage();

public slots:
    void actualSpeedReceived(double speed);
    void actualSteeringReceived(double steering);
    void commandStatusReceived(quint8 status);
    void batterySOCReceived(double batterysoc);
    void batteryVoltageReceived(double batteryvoltage);

signals:
    void sendCommandSpeed(double speed);
    void sendCommandSteering(double steering);
    void sendCommandAttributes(quint32 attributes);
    void sendActualStatus(quint8 status);

private:
    double mActualSpeed = 0;
    double mActualSteeringAngle = 0;
    quint8 mStatus = 0; // Start with emergency stop activated
    double mBatterySOC = 0;
    double mBatteryVoltage = 0;

    QSharedPointer<DiffDriveVehicleState> mDiffDriveVehicleState;

};
#endif // CANOPENMOVEMENTCONTROLLER_H
