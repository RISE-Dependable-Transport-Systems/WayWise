/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of MovementController for controll over CANopen.
 */

#ifndef CANOPENMOVEMENTCONTROLLER_H
#define CANOPENMOVEMENTCONTROLLER_H

#include <QObject>
#include "vehicles/controller/movementcontroller.h"
#include "canopencontrollerinterface.h"
#include <QSharedPointer>
#include "sensors/gnss/ublox.h"

struct CANOpenAutopilotControlState {
    bool emergencyStop = true;
    bool autoPilot = false;
    bool followMe = false;
    bool pause = false;
    bool resume = false;
    int waypointfollowerID = 0;
};

class CANopenMovementController : public MovementController
{
    Q_OBJECT
public:
    CANopenMovementController(QSharedPointer<VehicleState> vehicleState);
    ~CANopenMovementController();

    // MovementController interface
    virtual void setDesiredSpeed(double desiredSpeed) override;
    virtual void setDesiredSteering(double desiredSteering) override;
    virtual void setDesiredSteeringCurvature(double desiredSteeringCurvature) override;
    virtual void setDesiredAttributes(quint32 desiredAttributes) override;

    bool isMovementSimulated() const;

signals:
    void sendCommandSpeed(const double& speed);
    void sendCommandSteeringCurvature(const double& steering);
    void sendCommandAttributes(const quint32& attributes);
    void sendActualStatus(const quint8& status);
    void sendGNSSDataToCAN(const QVariant&);
    void distanceOfRouteLeft(double dist);

    void CANOpenAutopilotControlStateChanged(CANOpenAutopilotControlState controlState);

private slots:
    void actualSpeedReceived(double speed);
    void actualSteeringCurvatureReceived(double steering);
    void commandStatusReceived(quint8 status);
    void batterySOCReceived(double batterysoc);
    void batteryVoltageReceived(double batteryvoltage);

public slots:
    void rxNavPvt(const ubx_nav_pvt &pvt);
    void receiveDistanceOfRouteLeft(double dist);

private:
    bool mSimulateMovement = false;
    CANOpenAutopilotControlState mCANOpenAutopilotControlState;
    double mBatterySOC = 0;
    double mBatteryVoltage = 0;

    QSharedPointer<QThread> mCanopenThread;
    QSharedPointer<CANopenControllerInterface> mCANopenControllerInterface;
};
#endif // CANOPENMOVEMENTCONTROLLER_H
