/*
 *     Copyright 2024 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "followpoint.h"
#include "core/geometry.h"
#include <QDebug>
#include <QLineF>

FollowPoint::FollowPoint(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &FollowPoint::updateState);

    // ToDo: Provide system parameters to ControlTower

    // Follow point requires continuous updates of the point to follow
    mFollowPointTimedOut = true;
    FollowPoint::mFollowPointHeartbeatTimer.setSingleShot(true);
    connect(&mFollowPointHeartbeatTimer, &QTimer::timeout, this, [&](){
        if ((mStmState == FollowPointSTMstates::FOLLOWING || mStmState == FollowPointSTMstates::WAITING) && this->isActive()) {
            qDebug() << "WARNING: Follow point timed out. Stopping WaypointFollower.";
            this->stopFollowPoint();
        }
        mFollowPointTimedOut = true;
    });
}

FollowPoint::FollowPoint(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed)
{
    mVehicleConnection = vehicleConnection;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &FollowPoint::updateState);
    setPosTypeUsed(posTypeUsed);

    throw std::logic_error("FollowPoint::FollowPoint(..): follow point over remote connection not supported for now.");
}

bool FollowPoint::isActive()
{
    return mUpdateStateTimer.isActive();
}

void FollowPoint::startFollowPoint()
{
    // Deactivate emergency brake in order to use follow point
    emit deactivateEmergencyBrake();
    if (isOnVehicle()) {
        mStmState = FollowPointSTMstates::FOLLOWING;
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    } else {
        qDebug() << "WARNING: Follow Point not implemented for remote connections. Exiting.";
        mUpdateStateTimer.stop();
        mStmState = FollowPointSTMstates::NONE;
    }
}

void FollowPoint::stopFollowPoint()
{
    mUpdateStateTimer.stop();
    (isOnVehicle() ? mMovementController->getVehicleState() : mVehicleConnection->getVehicleState())->setAutopilotRadius(0);
    holdPosition();
    mStmState = FollowPointSTMstates::NONE;
    emit activateEmergencyBrake();
}

void FollowPoint::holdPosition()
{
    if (isOnVehicle()) {
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
    } else {
        mVehicleConnection->requestVelocityAndYaw({}, mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getYaw());
    }
}

void FollowPoint::setPosTypeUsed(const PosType &posTypeUsed)
{
    mPosTypeUsed = posTypeUsed;
}

void FollowPoint::updateFollowPointInVehicleFrame(const PosPoint &point)
{
    if ((mStmState == FollowPointSTMstates::FOLLOWING || mStmState == FollowPointSTMstates::WAITING) &&
         (point.getTime() > mCurrentFollowPointInVehicleFrame.getTime())) {
        if (mFollowPointTimedOut) {
            qDebug() << "Follow Point: timeout reset.";
        }

        mCurrentFollowPointInVehicleFrame = point;
        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mFollowPointTimedOut = false;
    }
}

PosPoint FollowPoint::getCurrentVehiclePosition()
{
    if (isOnVehicle())
        return mMovementController->getVehicleState()->getPosition(mPosTypeUsed);
    else
        return mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed);
}

void FollowPoint::updateState()
{
    QLineF egoVehicleToFollowPointLine(QPointF(0,0), mCurrentFollowPointInVehicleFrame.getPoint());

    switch (mStmState) {
    case FollowPointSTMstates::NONE:
        qDebug() << "WARNING: FollowPoint running uninitialized statemachine.";
        break;

    // FOLLOW_POINT: we follow a point that is moving "follow me", works on vehicle frame to be independent of positioning
    case FollowPointSTMstates::FOLLOWING:
        if (egoVehicleToFollowPointLine.length() > mMaximumFollowPointDistance) {
            qDebug() << "WARNING: Follow point is over" << mMaximumFollowPointDistance << "meters away. Exiting.";
            stopFollowPoint();
        }
        if (egoVehicleToFollowPointLine.length() < mFollowPointDistance)
            mStmState = FollowPointSTMstates::WAITING;
        else {
            QVector<QPointF> intersections = geometry::findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(QPointF(0,0), mPurepursuitRadius), egoVehicleToFollowPointLine);
            mMovementController->setDesiredSteeringCurvature(getCurvatureToPointInVehicleFrame(QPointF(intersections[0].x(), intersections[0].y())));
            mMovementController->setDesiredSpeed(mFollowPointSpeed);
         }
        break;
    case FollowPointSTMstates::WAITING:
        holdPosition();

        if (egoVehicleToFollowPointLine.length() > mFollowPointDistance)
            mStmState = FollowPointSTMstates::FOLLOWING;
        break;
    }
}

double FollowPoint::getCurvatureToPointInVehicleFrame(const QPointF &point)
{
    // ToDo: move to a general place?
    // calc steering angle (pure pursuit)
    double distanceSquared = pow(point.x(), 2) + pow(point.y(), 2);
    double steeringAngleProportional = (2*point.y()) / distanceSquared;

    return -steeringAngleProportional;
}
