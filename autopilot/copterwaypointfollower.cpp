/*
 *     Copyright 2026 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "copterwaypointfollower.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QDebug>

namespace {
constexpr double kDegToRad = M_PI / 180.0;
}

CopterWaypointFollower::CopterWaypointFollower(
    QSharedPointer<MovementController> movementController,
    PosType posTypeUsed)
{
    mMovementController = movementController;
    mVehicleState = mMovementController->getVehicleState();
    mPosTypeUsed = posTypeUsed;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &CopterWaypointFollower::updateState);
}

bool CopterWaypointFollower::getRepeatRoute() const
{
    return mCurrentState.repeatRoute;
}

void CopterWaypointFollower::setRepeatRoute(bool value)
{
    mCurrentState.repeatRoute = value;
}

const PosPoint CopterWaypointFollower::getCurrentGoal()
{
    return mCurrentState.currentGoal;
}

void CopterWaypointFollower::clearRoute()
{
    stop();
    mWaypointList.clear();
}

void CopterWaypointFollower::addWaypoint(const PosPoint &point)
{
    mWaypointList.append(point);
}

void CopterWaypointFollower::addRoute(const QList<PosPoint> &route)
{
    if (route.isEmpty()) {
        return;
    }
    mWaypointList.append(route);
}

QList<PosPoint> CopterWaypointFollower::getCurrentRoute()
{
    return mWaypointList;
}

void CopterWaypointFollower::startFollowingRoute(bool fromBeginning)
{
    if (mWaypointList.isEmpty()) {
        qDebug() << "Note: CopterWaypointFollower does not have any waypoints to follow.";
        return;
    }

    emit activateEmergencyBrake();

    if (fromBeginning) {
        mCurrentState.currentWaypointIndex = 0;
        mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;
    } else {
        mCurrentState.currentWaypointIndex = findClosestWaypointIndex();
        mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);
        mCurrentState.stmState =
            mCurrentState.currentWaypointIndex == mWaypointList.size() - 1 ?
            WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL :
            WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
    }

    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
}

bool CopterWaypointFollower::isActive()
{
    return mUpdateStateTimer.isActive();
}

void CopterWaypointFollower::stop()
{
    if (mUpdateStateTimer.isActive()) {
        mUpdateStateTimer.stop();
    }
    holdPosition();
    emit deactivateEmergencyBrake();
}

void CopterWaypointFollower::resetState()
{
    mUpdateStateTimer.stop();
    mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    mCurrentState.currentWaypointIndex = mWaypointList.size();
    mCurrentState.currentGoal = PosPoint();
    mPrevDistanceToGoal = std::numeric_limits<double>::max();
    holdPosition();
}

PosType CopterWaypointFollower::getPosTypeUsed() const
{
    return mPosTypeUsed;
}

void CopterWaypointFollower::setPosTypeUsed(const PosType &posTypeUsed)
{
    mPosTypeUsed = posTypeUsed;
}

void CopterWaypointFollower::updateState()
{
    switch (mCurrentState.stmState) {
    case WayPointFollowerSTMstates::NONE:
        qDebug() << "WARNING: CopterWaypointFollower running uninitialized statemachine.";
        break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT:
        if (mWaypointList.isEmpty()) {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
            break;
        }
        mCurrentState.currentWaypointIndex = 0;
        mCurrentState.currentGoal = mWaypointList.at(0);
        mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN;
        break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN:
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING: {
        if (mCurrentState.currentWaypointIndex >= mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = mWaypointList.size() - 1;
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
            mPrevDistanceToGoal = std::numeric_limits<double>::max();
            break;
        }

        mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);
        const double distanceToGoal =
            getCurrentVehiclePosition().getDistanceTo3d(mCurrentState.currentGoal);

        // Advance if within proximity, OR if the drone overshot — it was within
        // 2× proximity at closest approach but is now moving away from the waypoint.
        const bool reachedByProximity = distanceToGoal <= mWaypointProximity;
        const bool reachedByOvershoot = mPrevDistanceToGoal <= mWaypointProximity * 2.0
                                        && distanceToGoal > mPrevDistanceToGoal;
        if (reachedByProximity || reachedByOvershoot) {
            mPrevDistanceToGoal = std::numeric_limits<double>::max();
            mCurrentState.currentWaypointIndex++;
            if (mCurrentState.currentWaypointIndex >= mWaypointList.size()) {
                mCurrentState.currentWaypointIndex = mWaypointList.size() - 1;
                mCurrentState.currentGoal = mWaypointList.last();
                mCurrentState.stmState =
                    WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
            } else {
                mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
            }
        } else {
            mPrevDistanceToGoal = distanceToGoal;
        }

        updateControl(mCurrentState.currentGoal);
    } break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL: {
        if (mWaypointList.isEmpty()) {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
            break;
        }

        mCurrentState.currentWaypointIndex = mWaypointList.size() - 1;
        mCurrentState.currentGoal = mWaypointList.last();
        const double distanceToGoal =
            getCurrentVehiclePosition().getDistanceTo3d(mCurrentState.currentGoal);
        if (distanceToGoal <= mEndGoalAlignmentThreshold) {
            if (mCurrentState.repeatRoute) {
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;
            } else {
                qDebug() << "Copter route goal reached with accuracy:" << distanceToGoal << "m.";
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
            }
        } else {
            updateControl(mCurrentState.currentGoal);
        }
    } break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED:
        stop();
        break;

    default:
        break;
    }
}

void CopterWaypointFollower::updateControl(const PosPoint &goal)
{
    const PosPoint currentPos = getCurrentVehiclePosition();
    const double dx = goal.getX() - currentPos.getX();
    const double dy = goal.getY() - currentPos.getY();
    const double dz = goal.getHeight() - currentPos.getHeight();
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    mDesiredVelocityCommand = {};
    if (distance > std::numeric_limits<double>::epsilon()) {
        const double speed = speedForGoal(goal, distance);
        const double vxEnu = dx / distance * speed;
        const double vyEnu = dy / distance * speed;
        const double vzEnu = dz / distance * speed;
        const double yawRad = currentPos.getYaw() * kDegToRad;
        const double cosYaw = std::cos(yawRad);
        const double sinYaw = std::sin(yawRad);

        mDesiredVelocityCommand.forward = vxEnu * cosYaw + vyEnu * sinYaw;
        mDesiredVelocityCommand.left = -vxEnu * sinYaw + vyEnu * cosYaw;
        mDesiredVelocityCommand.up = vzEnu;

        const double targetYawRad = mFaceTravelDirection ?
            std::atan2(dy, dx) :
            goal.getYaw() * kDegToRad;
        const double yawError = normalizeAngleRad(targetYawRad - yawRad);
        mDesiredVelocityCommand.yawRate =
            std::clamp(mYawGain * yawError, -mMaxYawRate, mMaxYawRate);
    }

    mMovementController->setDesiredSpeed(mDesiredVelocityCommand.forward);
    mMovementController->setDesiredSteering(mDesiredVelocityCommand.yawRate);
    mMovementController->setDesiredAttributes(goal.getAttributes());

    // Place the visual target on the approach-slowdown-radius circle facing the goal,
    // mirroring what PurePursuit does with its lookahead circle.
    const double dist2d = std::sqrt(dx * dx + dy * dy);
    const QPointF circleTarget = (dist2d > mApproachSlowdownRadius)
        ? QPointF(currentPos.getX() + dx / dist2d * mApproachSlowdownRadius,
                  currentPos.getY() + dy / dist2d * mApproachSlowdownRadius)
        : goal.getPoint();
    mVehicleState->setAutopilotTargetPoint(circleTarget);
}

void CopterWaypointFollower::holdPosition()
{
    mDesiredVelocityCommand = {};
    if (mMovementController) {
        mMovementController->setDesiredSpeed(0.0);
        mMovementController->setDesiredSteering(0.0);
    }
}

PosPoint CopterWaypointFollower::getCurrentVehiclePosition() const
{
    return mVehicleState->getPosition(mPosTypeUsed);
}

int CopterWaypointFollower::findClosestWaypointIndex() const
{
    const PosPoint currentPos = getCurrentVehiclePosition();
    int closestIndex = 0;
    double closestDistance = std::numeric_limits<double>::max();
    for (int i = 0; i < mWaypointList.size(); i++) {
        const double distance = currentPos.getDistanceTo3d(mWaypointList.at(i));
        if (distance < closestDistance) {
            closestDistance = distance;
            closestIndex = i;
        }
    }
    return closestIndex;
}

double CopterWaypointFollower::speedForGoal(const PosPoint &goal, double distanceToGoal) const
{
    double speed = std::abs(goal.getSpeed());
    if (speed <= std::numeric_limits<double>::epsilon()) {
        speed = mCruiseSpeed;
    }
    speed = std::min(speed, mMaxSpeed);

    if (mCurrentState.stmState == WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL &&
        mApproachSlowdownRadius > 0.0 && distanceToGoal < mApproachSlowdownRadius)
    {
        const double scaledSpeed = speed * distanceToGoal / mApproachSlowdownRadius;
        speed = std::max(mMinApproachSpeed, scaledSpeed);
    }

    return speed;
}

double CopterWaypointFollower::normalizeAngleRad(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}
