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
constexpr double kVerticalLegHorizontalSpeedThreshold = 0.15;

double controlledVerticalSpeed(
    double heightError,
    double verticalVelocity,
    double maxVerticalSpeed,
    double dt,
    double heightTolerance,
    double proportionalGain,
    double integralGain,
    double derivativeGain,
    double integralLimit,
    double &heightErrorIntegral)
{
    if (maxVerticalSpeed <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }

    if (std::abs(heightError) <= heightTolerance) {
        heightErrorIntegral = 0.0;
        return 0.0;
    }

    heightErrorIntegral = std::clamp(
        heightErrorIntegral + heightError * dt,
        -integralLimit,
        integralLimit);

    const double verticalSpeed =
        proportionalGain * heightError +
        integralGain * heightErrorIntegral -
        derivativeGain * verticalVelocity;
    return std::clamp(verticalSpeed, -maxVerticalSpeed, maxVerticalSpeed);
}

double verticalSpeedForHeightError(
    double heightError,
    double verticalVelocity,
    double verticalSpeedLimit,
    double dt,
    double heightTolerance,
    double proportionalGain,
    double integralGain,
    double derivativeGain,
    double integralLimit,
    double &heightErrorIntegral)
{
    if (heightError < -heightTolerance) {
        heightErrorIntegral = 0.0;
        return -verticalSpeedLimit;
    }

    return controlledVerticalSpeed(
        heightError,
        verticalVelocity,
        verticalSpeedLimit,
        dt,
        heightTolerance,
        proportionalGain,
        integralGain,
        derivativeGain,
        integralLimit,
        heightErrorIntegral);
}
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

    if (fromBeginning || mWaypointList.size() < 2) {
        mCurrentState.currentWaypointIndex = 0;
    } else {
        int closestIdx = findClosestSegmentStartIndex();
        const PosPoint segmentStart = mWaypointList.at(closestIdx);
        const PosPoint segmentEnd = mWaypointList.at(closestIdx + 1);
        const double px = getCurrentVehiclePosition().getX();
        const double py = getCurrentVehiclePosition().getY();
        const double sx = segmentStart.getX();
        const double sy = segmentStart.getY();
        const double vx = segmentEnd.getX() - sx;
        const double vy = segmentEnd.getY() - sy;
        const double segmentLengthSquared = vx * vx + vy * vy;
        const double projection = segmentLengthSquared > std::numeric_limits<double>::epsilon() ?
            ((px - sx) * vx + (py - sy) * vy) / segmentLengthSquared : 0.0;
        mCurrentState.currentWaypointIndex = projection > 0.0 ? closestIdx + 1 : closestIdx;
    }
    mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;
    mSkipStartWaypointAfterClimb = false;
    mClimbingToStartWaypointHeight = !fromBeginning || mFinishRouteAfterInitialClimb;
    if (fromBeginning) {
        const PosPoint currentPos = getCurrentVehiclePosition();
        const PosPoint startWaypoint = mWaypointList.first();
        const double dx = startWaypoint.getX() - currentPos.getX();
        const double dy = startWaypoint.getY() - currentPos.getY();
        const double distance2d = std::sqrt(dx * dx + dy * dy);
        mClimbingToStartWaypointHeight = distance2d <= mWaypointProximityXY &&
            std::abs(startWaypoint.getHeight() - currentPos.getHeight()) > mVerticalHeightTolerance;
        mSkipStartWaypointAfterClimb = mClimbingToStartWaypointHeight;
    }
    mPrevDistanceToGoal = std::numeric_limits<double>::max();
    mVerticalHeightErrorIntegral = 0.0;

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
    mClimbingToStartWaypointHeight = false;
    mSkipStartWaypointAfterClimb = false;
    mFinishRouteAfterInitialClimb = false;
    mPrevDistanceToGoal = std::numeric_limits<double>::max();
    mVerticalHeightErrorIntegral = 0.0;
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

        if (mClimbingToStartWaypointHeight) {
            if (mCurrentState.currentWaypointIndex >= mWaypointList.size()) {
                mCurrentState.currentWaypointIndex = mWaypointList.size() - 1;
            }
            mCurrentState.currentGoal = waypointHeightAtCurrentPosition(
                mCurrentState.currentWaypointIndex);
            const double heightError =
                mCurrentState.currentGoal.getHeight() - getCurrentVehiclePosition().getHeight();
            if (std::abs(heightError) > mWaypointProximityZ) {
                updateClimbControl(mCurrentState.currentGoal);
                break;
            }

            mClimbingToStartWaypointHeight = false;
            mDesiredVelocityCommand = {};
            mPrevDistanceToGoal = std::numeric_limits<double>::max();
            mVerticalHeightErrorIntegral = 0.0;
            if (mFinishRouteAfterInitialClimb) {
                mFinishRouteAfterInitialClimb = false;
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
                qInfo() << "Copter route goal reached with vertical accuracy:"
                        << std::abs(heightError) << "m.";
                break;
            }
            if (mSkipStartWaypointAfterClimb &&
                mCurrentState.currentWaypointIndex + 1 < mWaypointList.size()) {
                mCurrentState.currentWaypointIndex++;
            }
            mSkipStartWaypointAfterClimb = false;
            mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);
            const PosPoint currentPos = getCurrentVehiclePosition();
            const double dx = mCurrentState.currentGoal.getX() - currentPos.getX();
            const double dy = mCurrentState.currentGoal.getY() - currentPos.getY();
            const double distance2d = std::sqrt(dx * dx + dy * dy);
            if (distance2d <= mWaypointProximityXY &&
                mCurrentState.currentWaypointIndex + 1 < mWaypointList.size()) {
                mCurrentState.currentWaypointIndex++;
                mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);
            }
            mCurrentState.stmState =
                mCurrentState.currentWaypointIndex == mWaypointList.size() - 1 ?
                WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL :
                WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
            updateControl(mCurrentState.currentGoal);
            break;
        }

        mCurrentState.currentWaypointIndex = 0;
        mCurrentState.currentGoal = mWaypointList.at(0);
        mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN;
        break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN: {
        if (mCurrentState.currentWaypointIndex >= mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = mWaypointList.size() - 1;
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
            mPrevDistanceToGoal = std::numeric_limits<double>::max();
            break;
        }

        mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);
        const double distanceToGoal =
            getCurrentVehiclePosition().getDistanceTo3d(mCurrentState.currentGoal);
        const PosPoint currentPos = getCurrentVehiclePosition();
        const double dx = mCurrentState.currentGoal.getX() - currentPos.getX();
        const double dy = mCurrentState.currentGoal.getY() - currentPos.getY();
        const double dz = mCurrentState.currentGoal.getHeight() - currentPos.getHeight();
        const double distance2d = std::sqrt(dx * dx + dy * dy);

        const bool reachedByProximity = distance2d <= mWaypointProximityXY && std::abs(dz) <= mWaypointProximityZ;
        const bool reachedByOvershoot = distance2d <= mWaypointProximityXY * 2.0 &&
                                        distance2d > mPrevDistanceToGoal;
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
        const PosPoint currentPos = getCurrentVehiclePosition();
        const double dx = mCurrentState.currentGoal.getX() - currentPos.getX();
        const double dy = mCurrentState.currentGoal.getY() - currentPos.getY();
        const double dz = mCurrentState.currentGoal.getHeight() - currentPos.getHeight();
        const double distance2d = std::sqrt(dx * dx + dy * dy);

        // Advance if within proximity, OR if the drone overshot — it was within
        // 2× proximity at closest approach but is now moving away from the waypoint.
        const bool reachedByProximity = distance2d <= mWaypointProximityXY && std::abs(dz) <= mWaypointProximityZ;
        const bool reachedByOvershoot = distance2d <= mWaypointProximityXY * 2.0 &&
                                        distance2d > mPrevDistanceToGoal;
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
        const PosPoint currentPos = getCurrentVehiclePosition();
        const double dx = mCurrentState.currentGoal.getX() - currentPos.getX();
        const double dy = mCurrentState.currentGoal.getY() - currentPos.getY();
        const double distance2d = std::sqrt(dx * dx + dy * dy);

        const auto vel = mVehicleState->getVelocity();
        const double speed2d = std::sqrt(vel.x * vel.x + vel.y * vel.y);

        // Require the horizontal speed to be relatively low to ensure we are actually
        // settling at the goal, not just blowing past it at high speed (overshoot).
        if (distance2d <= mWaypointProximityXY && speed2d < mStopSpeedThreshold) {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL_Z;
            qInfo() << "Copter route end goal XY tolerance reached. Adjusting Z. (2D accuracy:" << distance2d << "m)";
        } else {
            updateControl(mCurrentState.currentGoal);
        }
    } break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL_Z: {
        if (mWaypointList.isEmpty()) {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
            break;
        }

        mCurrentState.currentWaypointIndex = mWaypointList.size() - 1;
        mCurrentState.currentGoal = mWaypointList.last();

        const PosPoint currentPos = getCurrentVehiclePosition();
        const double dx = mCurrentState.currentGoal.getX() - currentPos.getX();
        const double dy = mCurrentState.currentGoal.getY() - currentPos.getY();
        const double distance2d = std::sqrt(dx * dx + dy * dy);

        // If a strong wind or overshoot pushes the drone significantly out of the XY tolerance, 
        // fallback to the previous state to re-acquire the horizontal position.
        if (distance2d > mWaypointProximityXY * 2.0) {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
            updateControl(mCurrentState.currentGoal);
            break;
        }

        const double dz = mCurrentState.currentGoal.getHeight() - currentPos.getHeight();

        if (std::abs(dz) <= mEndGoalAlignmentThresholdZ && distance2d <= mEndGoalAlignmentThresholdXY) {
            const auto vel = mVehicleState->getVelocity();
            const double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
            if (speed < mStopSpeedThreshold) {
                if (mCurrentState.repeatRoute) {
                    mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;
                } else {
                    mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
                    qInfo() << "Copter route goal reached with 2D accuracy:" << distance2d << "m and Z accuracy:" << std::abs(dz) << "m.";
                }
            } else {
                holdPosition();
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
    const double distance2d = std::sqrt(dx * dx + dy * dy);

    if (isVerticalLeg(mCurrentState.currentWaypointIndex)) {
        const auto velocity = mVehicleState->getVelocity();
        const double horizontalSpeed = std::hypot(velocity.x, velocity.y);
        const double horizontalSpeedTolerance = std::max(
            kVerticalLegHorizontalSpeedThreshold,
            mMinApproachSpeed);
        if (distance2d > mWaypointProximityXY ||
            horizontalSpeed > horizontalSpeedTolerance) {
            PosPoint levelGoal = goal;
            levelGoal.setHeight(verticalLegReferenceHeight(mCurrentState.currentWaypointIndex));
            updateTrackingControl(levelGoal);
        } else {
            updateClimbControl(goal);
        }
        return;
    }

    updateTrackingControl(goal);
}

void CopterWaypointFollower::updateTrackingControl(const PosPoint &goal)
{
    const PosPoint currentPos = getCurrentVehiclePosition();
    const double dx = goal.getX() - currentPos.getX();
    const double dy = goal.getY() - currentPos.getY();
    const double dz = goal.getHeight() - currentPos.getHeight();
    const double distance2d = std::sqrt(dx * dx + dy * dy);
    const double distance = std::sqrt(distance2d * distance2d + dz * dz);

    if (distance2d <= mWaypointProximityXY && std::abs(dz) > mWaypointProximityZ) {
        updateClimbControl(goal);
        return;
    }

    mDesiredVelocityCommand = {};
    if (distance > std::numeric_limits<double>::epsilon()) {
        const double speed = speedForGoal(goal, distance2d);
        const double verticalSpeedLimit = dz < -mVerticalHeightTolerance ?
            descentSpeedForGoal(goal) : speed;
        const double vzEnu = verticalSpeedForHeightError(
            dz,
            mVehicleState->getVelocity().z,
            verticalSpeedLimit,
            mUpdateStatePeriod_ms / 1000.0,
            mVerticalHeightTolerance,
            mVerticalProportionalGain,
            mVerticalIntegralGain,
            mVerticalDerivativeGain,
            mVerticalIntegralLimit,
            mVerticalHeightErrorIntegral);
        const double horizontalSpeed = distance2d > std::numeric_limits<double>::epsilon() ?
            speed : 0.0;
        const double vxEnu = distance2d > std::numeric_limits<double>::epsilon() ?
            dx / distance2d * horizontalSpeed : 0.0;
        const double vyEnu = distance2d > std::numeric_limits<double>::epsilon() ?
            dy / distance2d * horizontalSpeed : 0.0;
        const double yawRad = currentPos.getYaw() * kDegToRad;
        const double cosYaw = std::cos(yawRad);
        const double sinYaw = std::sin(yawRad);

        mDesiredVelocityCommand.forward = vxEnu * cosYaw + vyEnu * sinYaw;
        mDesiredVelocityCommand.left = -vxEnu * sinYaw + vyEnu * cosYaw;
        mDesiredVelocityCommand.up = vzEnu;

        const double targetYawRad = mFaceTravelDirection &&
            distance2d > std::numeric_limits<double>::epsilon() ?
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
    const QPointF circleTarget = (distance2d > mApproachSlowdownRadius)
        ? QPointF(currentPos.getX() + dx / distance2d * mApproachSlowdownRadius,
                  currentPos.getY() + dy / distance2d * mApproachSlowdownRadius)
        : goal.getPoint();
    mVehicleState->setAutopilotTargetPoint(circleTarget);
}

void CopterWaypointFollower::updateClimbControl(const PosPoint &goal)
{
    const PosPoint currentPos = getCurrentVehiclePosition();
    const double heightError = goal.getHeight() - currentPos.getHeight();
    const double verticalSpeedLimit = heightError < -mVerticalHeightTolerance ?
        descentSpeedForGoal(goal) : climbSpeedForGoal(goal);

    mDesiredVelocityCommand = {};
    mDesiredVelocityCommand.up = verticalSpeedForHeightError(
        heightError,
        mVehicleState->getVelocity().z,
        verticalSpeedLimit,
        mUpdateStatePeriod_ms / 1000.0,
        mVerticalHeightTolerance,
        mVerticalProportionalGain,
        mVerticalIntegralGain,
        mVerticalDerivativeGain,
        mVerticalIntegralLimit,
        mVerticalHeightErrorIntegral);

    mMovementController->setDesiredSpeed(0.0);
    mMovementController->setDesiredSteering(0.0);
    mMovementController->setDesiredAttributes(goal.getAttributes());
    mVehicleState->setAutopilotTargetPoint(currentPos.getPoint());
}

void CopterWaypointFollower::holdPosition()
{
    mDesiredVelocityCommand = {};
    mVerticalHeightErrorIntegral = 0.0;
    if (mMovementController) {
        mMovementController->setDesiredSpeed(0.0);
        mMovementController->setDesiredSteering(0.0);
    }
}

bool CopterWaypointFollower::isVerticalLeg(int waypointIndex) const
{
    if (waypointIndex <= 0 || waypointIndex >= mWaypointList.size()) {
        return false;
    }

    const PosPoint previousGoal = mWaypointList.at(waypointIndex - 1);
    const PosPoint goal = mWaypointList.at(waypointIndex);
    const double dx = goal.getX() - previousGoal.getX();
    const double dy = goal.getY() - previousGoal.getY();
    const double dz = goal.getHeight() - previousGoal.getHeight();
    return std::sqrt(dx * dx + dy * dy) <= mWaypointProximityXY &&
        std::abs(dz) > mWaypointProximityZ;
}

PosPoint CopterWaypointFollower::waypointHeightAtCurrentPosition(int waypointIndex) const
{
    PosPoint climbGoal = mWaypointList.at(waypointIndex);
    const PosPoint currentPos = getCurrentVehiclePosition();
    climbGoal.setX(currentPos.getX());
    climbGoal.setY(currentPos.getY());
    return climbGoal;
}

double CopterWaypointFollower::verticalLegReferenceHeight(int waypointIndex) const
{
    if (waypointIndex <= 0 || waypointIndex >= mWaypointList.size()) {
        return getCurrentVehiclePosition().getHeight();
    }
    return mWaypointList.at(waypointIndex - 1).getHeight();
}

double CopterWaypointFollower::descentSpeedForGoal(const PosPoint &) const
{
    return std::min(mDescentSpeed, mMaxSpeed);
}

double CopterWaypointFollower::climbSpeedForGoal(const PosPoint &goal) const
{
    double speed = std::abs(goal.getSpeed());
    if (speed <= std::numeric_limits<double>::epsilon()) {
        speed = mCruiseSpeed;
    }
    return std::min(speed, mMaxSpeed);
}

PosPoint CopterWaypointFollower::getCurrentVehiclePosition() const
{
    return mVehicleState->getPosition(mPosTypeUsed);
}

int CopterWaypointFollower::findClosestSegmentStartIndex() const
{
    if (mWaypointList.size() < 2) {
        return 0;
    }

    const PosPoint currentPos = getCurrentVehiclePosition();
    const double px = currentPos.getX();
    const double py = currentPos.getY();
    int closestIndex = 0;
    double closestDistanceSquared = std::numeric_limits<double>::max();

    for (int i = 0; i < mWaypointList.size() - 1; i++) {
        const PosPoint segmentStart = mWaypointList.at(i);
        const PosPoint segmentEnd = mWaypointList.at(i + 1);
        const double sx = segmentStart.getX();
        const double sy = segmentStart.getY();
        const double vx = segmentEnd.getX() - sx;
        const double vy = segmentEnd.getY() - sy;
        const double segmentLengthSquared = vx * vx + vy * vy;
        const double projection = segmentLengthSquared > std::numeric_limits<double>::epsilon() ?
            std::clamp(((px - sx) * vx + (py - sy) * vy) / segmentLengthSquared, 0.0, 1.0) :
            0.0;
        const double closestX = sx + projection * vx;
        const double closestY = sy + projection * vy;
        const double dx = px - closestX;
        const double dy = py - closestY;
        const double distanceSquared = dx * dx + dy * dy;

        if (distanceSquared < closestDistanceSquared) {
            closestDistanceSquared = distanceSquared;
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
