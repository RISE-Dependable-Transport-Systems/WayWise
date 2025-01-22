/*
 *     Copyright 2024 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Follow a person or other vehicle when the point to follow is continously updated.
 */

#ifndef FOLLOWPOINT_H
#define FOLLOWPOINT_H

#include <QObject>
#include <QTimer>
#include <QSharedPointer>
#include <QPointF>
#include <QLineF>
#include "core/pospoint.h"
#include "vehicles/controller/movementcontroller.h"
#include "communication/vehicleconnections/vehicleconnection.h"

enum class FollowPointSTMstates {NONE, FOLLOWING, WAITING};

struct FollowPointState {
    FollowPointSTMstates stmState = FollowPointSTMstates::NONE;
    PosPoint currentPointToFollow;
    double distanceToPointIn2D = 0;
    QLineF lineFromVehicleToPoint;
    double followPointDistance = 2.0;
    int followPointMaximumDistance = 100;
    // For ground vehicles
    double followPointSpeed = 1.0;
    double autopilotRadius = 1;
    // For airbourne vehicles
    double followPointHeight = 3.0;
    double followPointAngleInDeg = 180; // [-180,180];
};

class VehicleConnection;

class FollowPoint : public QObject
{
    Q_OBJECT
public:
    // FollowPoint either works locally on a MovementController on the vehicle, or remotely on a VehicleConnection
    FollowPoint(QSharedPointer<MovementController> movementController);
    FollowPoint(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed);

    inline bool isOnVehicle() {return !mMovementController.isNull();}

    void startFollowPoint();
    void stopFollowPoint();

    bool isActive();

    void setFollowPointDistance(double distance);
    double getFollowPointDistance() const;
    void setFollowPointMaximumDistance(int distance);
    int getFollowPointMaximumDistance() const;
    void setFollowPointSpeed(double speed);
    double getFollowPointSpeed() const;
    void setAutopilotRadius(double radius);
    double getAutopilotRadius() const;
    void setFollowPointHeight(double height);
    double getFollowPointHeight() const;
    void setFollowPointAngleInDeg(double angle);
    double getFollowPointAngleInDeg() const;

    void provideParametersToParameterServer();

signals:
    void deactivateEmergencyBrake();
    void activateEmergencyBrake();

public slots:
    void updatePointToFollowInVehicleFrame(const PosPoint &point);
    void updatePointToFollowInEnuFrame(const PosPoint &point);

private:
    unsigned mFollowPointTimeout_ms = 1000;
    bool mFollowPointTimedOut = true;
    unsigned mUpdateStatePeriod_ms = 50;
    QTimer mFollowPointHeartbeatTimer;
    QTimer mUpdateStateTimer;

    PosType mPosTypeUsed = PosType::fused; // The type of position (Odom, GNSS, UWB, ...)

    FollowPointState mCurrentState;

    QSharedPointer<MovementController> mMovementController;
    QSharedPointer<VehicleConnection> mVehicleConnection;
    QSharedPointer<VehicleState> mVehicleState;

    void updateState();
    void holdPosition();
    bool thePointIsNewResetTheTimer(const PosPoint &point);
    void initializeTimers();
};

#endif // FOLLOWPOINT_H
