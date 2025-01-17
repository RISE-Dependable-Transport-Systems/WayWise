/*
 *     Copyright 2024 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "followpoint.h"
#include "core/geometry.h"
#include "communication/parameterserver.h"
#include <QDebug>

FollowPoint::FollowPoint(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    mVehicleState = mMovementController->getVehicleState();

    initializeTimers();
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
}

void FollowPoint::provideParametersToParameterServer()
{
    // Provide parameters to ControlTower
    std::string id = std::to_string(mVehicleState->getId());
    if (ParameterServer::getInstance()) {
        ParameterServer::getInstance()->provideFloatParameter("FP"+id+"_DIST", std::bind(&FollowPoint::setFollowPointDistance, this, std::placeholders::_1), std::bind(&FollowPoint::getFollowPointDistance, this));
        ParameterServer::getInstance()->provideFloatParameter("FP"+id+"_MAX_DIST", std::bind(&FollowPoint::setFollowPointMaximumDistance, this, std::placeholders::_1), std::bind(&FollowPoint::getFollowPointMaximumDistance, this));
        ParameterServer::getInstance()->provideFloatParameter("FP"+id+"_HEIGHT", std::bind(&FollowPoint::setFollowPointHeight, this, std::placeholders::_1), std::bind(&FollowPoint::getFollowPointHeight, this));
        ParameterServer::getInstance()->provideFloatParameter("FP"+id+"_ANGLE_DEG", std::bind(&FollowPoint::setFollowPointAngleInDeg, this, std::placeholders::_1), std::bind(&FollowPoint::getFollowPointAngleInDeg, this));
    }
}

void FollowPoint::initializeTimers()
{
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &FollowPoint::updateState);

    // Follow point requires continuous updates of the point to follow
    mFollowPointTimedOut = true;
    FollowPoint::mFollowPointHeartbeatTimer.setSingleShot(true);
    connect(&mFollowPointHeartbeatTimer, &QTimer::timeout, this, [&](){
        if ((mCurrentState.stmState == FollowPointSTMstates::FOLLOWING || mCurrentState.stmState == FollowPointSTMstates::WAITING) && this->isActive()) {
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
    mVehicleState->setAutopilotRadius(mCurrentState.autopilotRadius);
    mCurrentState.stmState = FollowPointSTMstates::FOLLOWING;
    mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
}

void FollowPoint::stopFollowPoint()
{
    mUpdateStateTimer.stop();
    mFollowPointHeartbeatTimer.stop();
    mVehicleState->setAutopilotRadius(0);
    holdPosition();
    mCurrentState.stmState = FollowPointSTMstates::NONE;
    emit activateEmergencyBrake();
}

void FollowPoint::holdPosition()
{
    if (isOnVehicle()) {
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
    } else {
        mVehicleConnection->requestGotoENU(mVehicleState->getPosition(mPosTypeUsed).getXYZ(), true);
    }
}

void FollowPoint::updatePointToFollowInVehicleFrame(const PosPoint &point)
{
    if (thePointIsNewResetTheTimer(point)) {
        mCurrentState.currentPointToFollow = point;
        mCurrentState.currentPointToFollow.setRadius(mCurrentState.followPointDistance);
        mCurrentState.lineFromVehicleToPoint.setP1(QPointF(0,0));
        mCurrentState.lineFromVehicleToPoint.setP2(point.getPoint());
        mCurrentState.distanceToPointIn2D = mCurrentState.lineFromVehicleToPoint.length();
        QVector<QPointF> intersections = geometry::findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(QPointF(0,0), mVehicleState->getAutopilotRadius()), mCurrentState.lineFromVehicleToPoint);
        (intersections.size() != 0) ? mCurrentState.currentPointToFollow.setXY(intersections[0].x(), intersections[0].y()) : mCurrentState.currentPointToFollow.setXY(0, 0);
    }
}

void FollowPoint::updatePointToFollowInEnuFrame(const PosPoint &point)
{
    if (thePointIsNewResetTheTimer(point)) {
        mCurrentState.currentPointToFollow = point;
        mCurrentState.currentPointToFollow.setRadius(mCurrentState.followPointDistance/10);
        mCurrentState.currentPointToFollow.setHeight(point.getHeight() + mCurrentState.followPointHeight);
        mCurrentState.currentPointToFollow.setXY(point.getX() + mCurrentState.followPointDistance*cos((point.getYaw() + mCurrentState.followPointAngleInDeg)* M_PI / 180.0), point.getY() + mCurrentState.followPointDistance*sin((point.getYaw() + mCurrentState.followPointAngleInDeg)* M_PI / 180.0));

        mCurrentState.distanceToPointIn2D = mVehicleState->getPosition(mPosTypeUsed).getDistanceTo(mCurrentState.currentPointToFollow);
    }
}

bool FollowPoint::thePointIsNewResetTheTimer(const PosPoint &point)
{
    static QTime oldPointTime = QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc());

    if ((mCurrentState.stmState == FollowPointSTMstates::FOLLOWING || mCurrentState.stmState == FollowPointSTMstates::WAITING) &&
            (point.getTime() > oldPointTime)) {
        if (mFollowPointTimedOut)
            qDebug() << "Follow Point: timeout reset.";

        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mFollowPointTimedOut = false;
        return true;
    }
    return false;
}

void FollowPoint::updateState()
{
    switch (mCurrentState.stmState) {
    case FollowPointSTMstates::NONE:
        qDebug() << "WARNING: FollowPoint running uninitialized statemachine.";
        break;

    // FOLLOW_POINT: we follow a point that is moving "follow me", works on vehicle frame to be independent of positioning
    case FollowPointSTMstates::FOLLOWING:
        if (mCurrentState.distanceToPointIn2D > mCurrentState.followPointMaximumDistance) {
            qDebug() << "WARNING: Follow point is over" << mCurrentState.followPointMaximumDistance << "meters away. Exiting.";
            stopFollowPoint();
            mCurrentState.stmState = FollowPointSTMstates::NONE;
            break;
        }
        if (mCurrentState.distanceToPointIn2D < mCurrentState.currentPointToFollow.getRadius())
            mCurrentState.stmState = FollowPointSTMstates::WAITING;
        else {
            if (isOnVehicle()) {
                mMovementController->setDesiredSteeringCurvature(mVehicleState->getCurvatureToPointInVehicleFrame(QPointF(mCurrentState.currentPointToFollow.getX(), mCurrentState.currentPointToFollow.getY())));
                mMovementController->setDesiredSpeed(mCurrentState.followPointSpeed);
            } else {
                mVehicleConnection->requestGotoENU(mCurrentState.currentPointToFollow.getXYZ(), true);
            }
         }
        break;
    case FollowPointSTMstates::WAITING:
        holdPosition();

        if (mCurrentState.distanceToPointIn2D > mCurrentState.currentPointToFollow.getRadius())
            mCurrentState.stmState = FollowPointSTMstates::FOLLOWING;
        break;
    }
}

void FollowPoint::setFollowPointDistance(double distance)
{
    mCurrentState.followPointDistance = distance;
}

double FollowPoint::getFollowPointDistance() const
{
    return mCurrentState.followPointDistance;
}

void FollowPoint::setFollowPointMaximumDistance(int distance)
{
    mCurrentState.followPointMaximumDistance = distance;
}

int FollowPoint::getFollowPointMaximumDistance() const
{
    return mCurrentState.followPointMaximumDistance;
}

void FollowPoint::setFollowPointSpeed(double speed)
{
    mCurrentState.followPointSpeed = speed;
}

double FollowPoint::getFollowPointSpeed() const
{
    return mCurrentState.followPointSpeed;
}

void FollowPoint::setAutopilotRadius(double radius)
{
    mCurrentState.autopilotRadius = radius;
}

double FollowPoint::getAutopilotRadius() const
{
    return mCurrentState.autopilotRadius;
}

void FollowPoint::setFollowPointHeight(double height)
{
    mCurrentState.followPointHeight = height;
}

double FollowPoint::getFollowPointHeight() const
{
    return mCurrentState.followPointHeight;
}

void FollowPoint::setFollowPointAngleInDeg(double angle)
{
    mCurrentState.followPointAngleInDeg = angle;
}

double FollowPoint::getFollowPointAngleInDeg() const
{
    return mCurrentState.followPointAngleInDeg;
}
