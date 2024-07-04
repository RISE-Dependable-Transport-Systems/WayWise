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

signals:
    void deactivateEmergencyBrake();
    void activateEmergencyBrake();

public slots:
    void pointToFollowInVehicleFrame(const PosPoint &point);
    void pointToFollowInEnuFrame(const PosPoint &point);

private:
    unsigned mFollowPointTimeout_ms = 1000;
    unsigned mUpdateStatePeriod_ms = 50;
    QTimer mFollowPointHeartbeatTimer;
    QTimer mUpdateStateTimer;

    PosPoint mCurrentGoal;
    FollowPointSTMstates mStmState = FollowPointSTMstates::NONE;

    PosPoint mCurrentPointToFollowInEnuFrame;
    double mDistanceToPointIn2D;
    QLineF mLineFromVehicleToPoint;
    double mFollowPointHeight = 3.0;
    double mFollowPointSpeed = 1.0;
    double mFollowPointDistance = 1.0;
    double mFollowPointAngleInDeg = 0;
    bool mFollowPointTimedOut = true;
    double mPurepursuitRadius = 1;
    int mMaximumFollowPointDistance = 100;

    PosType mPosTypeUsed = PosType::fused; // The type of position (Odom, GNSS, UWB, ...)

    QSharedPointer<MovementController> mMovementController;
    QSharedPointer<VehicleConnection> mVehicleConnection;
    QSharedPointer<VehicleState> mVehicleState;

    void updateState();
    void holdPosition();
    PosPoint getCurrentVehiclePosition();
    double getCurvatureToPointInVehicleFrame(const QPointF &point);
    bool pointIsNew(const PosPoint &point);
    void initializeTimers();
};

#endif // FOLLOWPOINT_H
