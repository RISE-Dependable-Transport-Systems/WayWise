#ifndef CANOPENMOVEMENTCONTROLLER_H
#define CANOPENMOVEMENTCONTROLLER_H

#include <QObject>
#include "sdvp_qtcommon/movementcontroller.h"
#include "canopencontrollerinterface.h"
#include <QSharedPointer>

struct CANOpenAutopilotControlState {
    bool emergencyStop = true;
    bool autoPilot = false;
    bool followMe = false;
    bool pause = false;
    bool resume = false;
};

class CANopenMovementController : public MovementController
{
    Q_OBJECT
public:
    CANopenMovementController(QSharedPointer<VehicleState> vehicleState);

    // MovementController interface
    virtual void setDesiredSpeed(double desiredSpeed) override;
    virtual void setDesiredSteering(double desiredSteering) override;
    virtual void setDesiredSteeringCurvature(double desiredSteeringCurvature) override;
    virtual void setDesiredAttributes(quint32 desiredAttributes) override;

    bool isMovementSimulated() const;

signals:
    void sendCommandSpeed(double speed);
    void sendCommandSteeringCurvature(double steering);
    void sendCommandAttributes(quint32 attributes);
    void sendActualStatus(quint8 status);

    void CANOpenAutopilotControlStateChanged(CANOpenAutopilotControlState controlState);

private slots:
    void actualSpeedReceived(double speed);
    void actualSteeringCurvatureReceived(double steering);
    void commandStatusReceived(quint8 status);
    void batterySOCReceived(double batterysoc);
    void batteryVoltageReceived(double batteryvoltage);

private:
    bool mSimulateMovement = false;
    CANOpenAutopilotControlState mCANOpenAutopilotControlState;
    double mBatterySOC = 0;
    double mBatteryVoltage = 0;

    QSharedPointer<QThread> mCanopenThread;
    QSharedPointer<CANopenControllerInterface> mCANopenControllerInterface;

};
#endif // CANOPENMOVEMENTCONTROLLER_H
