/*
 *     Copyright 2024 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "followpoint.h"
#include "core/geometry.h"
#include <QDebug>

FollowPoint::FollowPoint(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    mVehicleState = mMovementController->getVehicleState();

    initializeTimers();

    // ToDo: Provide system parameters to ControlTower
}

FollowPoint::FollowPoint(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed)
{
    mVehicleConnection = vehicleConnection;
    mPosTypeUsed = posTypeUsed;
    mVehicleState = mVehicleConnection->getVehicleState();

    // Lower frequency for remote connection
    mFollowPointTimeout_ms = 3000;
    mUpdateStatePeriod_ms = 1000;

    initializeTimers();

    // ToDo: Provide system parameters to ControlTower
}

void FollowPoint::initializeTimers()
{
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &FollowPoint::updateState);

    // Follow point requires continuous updates of the point to follow
    mFollowPointTimedOut = true;
    FollowPoint::mFollowPointHeartbeatTimer.setSingleShot(true);
    connect(&mFollowPointHeartbeatTimer, &QTimer::timeout, this, [&](){
        if ((mStmState == FollowPointSTMstates::FOLLOWING || mStmState == FollowPointSTMstates::WAITING) && this->isActive()) {
            qDebug() << "WARNING: Follow point timed out. Stopping FollowPoint.";
            this->stopFollowPoint();
        }
        mFollowPointTimedOut = true;
    });
}

bool FollowPoint::isActive()
{
    return mUpdateStateTimer.isActive();
}

void FollowPoint::startFollowPoint()
{
    emit deactivateEmergencyBrake();
    mStmState = FollowPointSTMstates::FOLLOWING;
    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
}

void FollowPoint::stopFollowPoint()
{
    mUpdateStateTimer.stop();
    mVehicleState->setAutopilotRadius(0);
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
        mVehicleConnection->requestGotoENU(getCurrentVehiclePosition().getXYZ(), true);
    }
}

void FollowPoint::pointToFollowInVehicleFrame(const PosPoint &point)
{
    if (pointIsNew(point)) {
        // ToDo: Apply follow point parameters here
        mLineFromVehicleToPoint.setP1(QPointF(0,0));
        mLineFromVehicleToPoint.setP2(point.getPoint());
        mDistanceToPointIn2D = mLineFromVehicleToPoint.length();
    }
}

void FollowPoint::pointToFollowInEnuFrame(const PosPoint &point)
{
    if (pointIsNew(point)) {
        // ToDo: Apply follow point parameters here
        mCurrentPointToFollowInEnuFrame = point;
        mCurrentPointToFollowInEnuFrame.setHeight(point.getHeight() + mFollowPointHeight);

        mDistanceToPointIn2D = getCurrentVehiclePosition().getDistanceTo(mCurrentPointToFollowInEnuFrame);
    }
}

bool FollowPoint::pointIsNew(const PosPoint &point)
{
    static QTime oldPointTime = QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc());

    if ((mStmState == FollowPointSTMstates::FOLLOWING || mStmState == FollowPointSTMstates::WAITING) &&
            (point.getTime() > oldPointTime)) {
        if (mFollowPointTimedOut)
            qDebug() << "Follow Point: timeout reset.";

        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mFollowPointTimedOut = false;
        return true;
    }
    return false;
}

PosPoint FollowPoint::getCurrentVehiclePosition()
{
    return mVehicleState->getPosition(mPosTypeUsed);
}

void FollowPoint::updateState()
{
    switch (mStmState) {
    case FollowPointSTMstates::NONE:
        qDebug() << "WARNING: FollowPoint running uninitialized statemachine.";
        break;

    // FOLLOW_POINT: we follow a point that is moving "follow me", works on vehicle frame to be independent of positioning
    case FollowPointSTMstates::FOLLOWING:
        if (mDistanceToPointIn2D > mFollowPointMaximumDistance) {
            qDebug() << "WARNING: Follow point is over" << mFollowPointMaximumDistance << "meters away. Exiting.";
            stopFollowPoint();
        }
        if (mDistanceToPointIn2D < mFollowPointDistance)
            mStmState = FollowPointSTMstates::WAITING;
        else {
            if (isOnVehicle()) {
                QVector<QPointF> intersections = geometry::findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(QPointF(0,0), mVehicleState->getAutopilotRadius()), mLineFromVehicleToPoint);
                mMovementController->setDesiredSteeringCurvature(getCurvatureToPointInVehicleFrame(QPointF(intersections[0].x(), intersections[0].y())));
                mMovementController->setDesiredSpeed(mFollowPointSpeed);
            } else {
                mVehicleConnection->requestGotoENU(mCurrentPointToFollowInEnuFrame.getXYZ(), true);
            }
         }
        break;
    case FollowPointSTMstates::WAITING:
        holdPosition();

        if (mDistanceToPointIn2D > mFollowPointDistance)
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
