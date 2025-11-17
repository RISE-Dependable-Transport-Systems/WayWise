/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include <cmath>
#include <QDebug>
#include <QLineF>
#include "purepursuitwaypointfollower.h"
#include "communication/parameterserver.h"
#include "core/geometry.h"
#include "routeplanning/zigzagroutegenerator.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDir>
#include <QStandardPaths>

PurepursuitWaypointFollower::PurepursuitWaypointFollower(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    mVehicleState = mMovementController->getVehicleState();
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &PurepursuitWaypointFollower::updateState);

    loadSpeedLimitRegionsFile();    // uses the default ENU reference (for simulation)
    connect(mVehicleState.get(), &VehicleState::updatedEnuReference, this, [this]() {
        qInfo() << "New ENU reference received, reloading speed limit regions.";

        clearSpeedLimitRegions();
        loadSpeedLimitRegionsFile();
    });
}

PurepursuitWaypointFollower::PurepursuitWaypointFollower(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed)
{
    mVehicleConnection = vehicleConnection;
    mVehicleState = mVehicleConnection->getVehicleState();
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &PurepursuitWaypointFollower::updateState);
    setPosTypeUsed(posTypeUsed);

    loadSpeedLimitRegionsFile();    // uses the default ENU reference (for simulation)
    connect(mVehicleState.get(), &VehicleState::updatedEnuReference, this, [this]() {
        qInfo() << "New ENU reference received, reloading speed limit regions.";

        clearSpeedLimitRegions();
        loadSpeedLimitRegionsFile();
    });
}

void PurepursuitWaypointFollower::provideParametersToParameterServer()
{
    if (ParameterServer::getInstance()) {
        ParameterServer::getInstance()->provideFloatParameter("PP_RADIUS", std::bind(&PurepursuitWaypointFollower::setPurePursuitRadius, this, std::placeholders::_1), std::bind(&PurepursuitWaypointFollower::getPurePursuitRadius, this));
        ParameterServer::getInstance()->provideFloatParameter("PP_ARC", std::bind(&PurepursuitWaypointFollower::setAdaptivePurePursuitRadiusCoefficient, this, std::placeholders::_1), std::bind(&PurepursuitWaypointFollower::getAdaptivePurePursuitRadiusCoefficient, this));
    }
}

void PurepursuitWaypointFollower::clearRoute()
{
    stop();
    mWaypointList.clear();
}

void PurepursuitWaypointFollower::addWaypoint(const PosPoint &point)
{
    mWaypointList.append(point);
}

QPointF PurepursuitWaypointFollower::getVehicleReferencePosition(const QList<PosPoint>& waypoints)
{
    // Default to the vehicle position.
    QPointF positionXY = mVehicleState->getPosition(mPosTypeUsed).getPoint();
    // If backing (speed < 0) and a trailer exists, use its position.
    if (!waypoints.isEmpty() && mVehicleState->hasTrailingVehicle() && waypoints.first().getSpeed() < 0) {
        positionXY = mVehicleState->getTrailingVehicle()->getPosition(mPosTypeUsed).getPoint();
    }
    return positionXY;
}

int PurepursuitWaypointFollower::findClosestWaypointIndex(const QList<PosPoint>& route, const QPointF &currentVehiclePositionXY)
{
    // Calculate closest point on a route to current vehicle position
    int closestPointIndex = 0;
    double minDistance = std::numeric_limits<double>::max();

    for (int i = 0; i < route.size(); i++) {
        double distance = QLineF(currentVehiclePositionXY, route[i].getPoint()).length();
        if (distance < minDistance) {
            minDistance = distance;
            closestPointIndex = i;
        }
    }

    while (closestPointIndex < route.size()) {
        if (QLineF(currentVehiclePositionXY, route.at(closestPointIndex).getPoint()).length() >= purePursuitRadius()) {
            break;
        }
        closestPointIndex++;
    }
    if (closestPointIndex == route.size()) {
        closestPointIndex--;
    }

    return closestPointIndex;
}

void PurepursuitWaypointFollower::addRoute(const QList<PosPoint> &route)
{
    if (route.isEmpty()) {
        return;
    }

    if (!isActive()) {
        mWaypointList.append(route);
    } else {
        // Calculate closest point on new route to current vehicle position
        QPointF currentVehiclePositionXY = getVehicleReferencePosition(route);
        int closestPointIndex = findClosestWaypointIndex(route, currentVehiclePositionXY);

        // Truncate the current route and append the new route from closest point onwards
        mWaypointList = mWaypointList.mid(0, mCurrentState.currentWaypointIndex);
        mWaypointList.append(route.mid(closestPointIndex));
        if (mCurrentState.currentWaypointIndex == mWaypointList.size()) {
            mCurrentState.currentWaypointIndex--;
        }
        mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);

        // Update stmState
        if (mCurrentState.currentWaypointIndex == mWaypointList.size() - 1) {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
        } else {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
        }
    }
}

void PurepursuitWaypointFollower::startFollowingRoute(bool fromBeginning)
{
    if (mWaypointList.size() == 0) {
        qDebug() << "Note: WaypointFollower does not have any waypoints to follow.";
        return;
    }

    // Activate emergency brake for follow route
    emit activateEmergencyBrake();
    mCurrentState.overrideAltitude = mVehicleState->getPosition(mPosTypeUsed).getHeight();
    qDebug() << "Note: WaypointFollower starts following route. Height info from route is ignored (staying at" << QString::number(mCurrentState.overrideAltitude, 'g', 2) << "m).";

    if (fromBeginning)
        mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;
    else {
        QPointF currentVehiclePositionXY = getVehicleReferencePosition(mWaypointList);
        int closestPointIndex = findClosestWaypointIndex(mWaypointList, currentVehiclePositionXY);
        mCurrentState.currentWaypointIndex = closestPointIndex;

        // Update stmState
        if (mCurrentState.currentWaypointIndex == mWaypointList.size() - 1) {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
        } else {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
        }
        mCurrentState.currentGoal = mWaypointList.at(mCurrentState.currentWaypointIndex);
        mCurrentState.startPointXY = currentVehiclePositionXY;
    }

    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
}

bool PurepursuitWaypointFollower::isActive()
{
    return mUpdateStateTimer.isActive();
}

void PurepursuitWaypointFollower::holdPosition()
{
    if (isOnVehicle()) {
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
    } else {
        mVehicleConnection->requestVelocityAndYaw({}, mVehicleState->getPosition(mPosTypeUsed).getYaw());
    }
}

void PurepursuitWaypointFollower::stop()
{
    if (mUpdateStateTimer.isActive()) {
        mUpdateStateTimer.stop();
    }
    mVehicleState->setAutopilotRadius(0);
    holdPosition();
    emit deactivateEmergencyBrake();
}

void PurepursuitWaypointFollower::resetState()
{
    mUpdateStateTimer.stop();
    mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    mCurrentState.currentWaypointIndex = mWaypointList.size();
    mCurrentState.startPointXY = QPointF();
}

void PurepursuitWaypointFollower::updateState()
{
    QPointF currentVehiclePositionXY = getVehicleReferencePosition(mWaypointList);

    switch (mCurrentState.stmState) {
    case WayPointFollowerSTMstates::NONE:
        qDebug() << "WARNING: WayPointFollower running uninitialized statemachine.";
        break;

    // FOLLOW_ROUTE: waypoints describe a route to be followed waypoint by waypoint
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT:
        if (mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = 0;
            mCurrentState.currentGoal = mWaypointList.at(0);
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN;
            mCurrentState.startPointXY = currentVehiclePositionXY;
        } else
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
        break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN: {
        calculateDistanceOfRouteLeft(currentVehiclePositionXY);

        // draw straight line to first point and apply purePursuitRadius to find intersection
        QLineF carToStartLine(currentVehiclePositionXY, mWaypointList.at(0).getPoint());
        QVector<QPointF> intersections = geometry::findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePositionXY, purePursuitRadius()), carToStartLine);

        if (intersections.size()) {
            mCurrentState.currentGoal.setXY(intersections[0].x(), intersections[0].y());
            trol(mCurrentState.currentGoal);
        } else { // first waypoint within circle -> start route
            if (mWaypointList.size() > 1) {
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
            } else {
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
            }
        }
    } break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING: {
        calculateDistanceOfRouteLeft(currentVehiclePositionXY);

        QPointF currentWaypointPoint = mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint();
        if (QLineF(currentVehiclePositionXY, currentWaypointPoint).length() < purePursuitRadius()) // consider previous waypoint as reached
            mCurrentState.currentWaypointIndex++;

        if (mCurrentState.currentWaypointIndex == mWaypointList.size()) {
            mCurrentState.currentWaypointIndex--;
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL;
        } else {
            // --- Calculate current goal on route (which lies between two waypoints)
            // 1. Find intersection between circle around vehicle and route
            // look a number of points ahead and jump forward on route, if applicable
            // and take care of index wrap in case route is repeated
            QList<PosPoint> lookAheadWaypoints;
            if (mCurrentState.repeatRoute) {
                lookAheadWaypoints = mWaypointList.mid(mCurrentState.currentWaypointIndex - 1, mCurrentState.numWaypointsLookahead);

                const int lookaheadWaypointEndIndex = mCurrentState.currentWaypointIndex + mCurrentState.numWaypointsLookahead - 1;
                if (lookaheadWaypointEndIndex > mWaypointList.size()) // index wraparound
                    lookAheadWaypoints.append(mWaypointList.mid(0, lookaheadWaypointEndIndex % mWaypointList.size()));
                else if (mCurrentState.currentWaypointIndex == 0) // restarting from end to beginning
                    lookAheadWaypoints.prepend(mWaypointList.last());
            } else
                lookAheadWaypoints = mWaypointList.mid(mCurrentState.currentWaypointIndex - 1,  mCurrentState.numWaypointsLookahead);

            QVector<QPointF> intersections;
            for (int i = lookAheadWaypoints.size() - 1; i > 0; i--) { // step backwards through lookahead window until intersection is found
                QPointF iWaypoint = lookAheadWaypoints.at(i).getPoint();
                QLineF iLineSegment(lookAheadWaypoints.at(i-1).getPoint(), iWaypoint);

                intersections = geometry::findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePositionXY, purePursuitRadius()), iLineSegment);
                if (intersections.size() > 0) {
                    mCurrentState.currentWaypointIndex = (i + mCurrentState.currentWaypointIndex - 1) % mWaypointList.size();
                    currentWaypointPoint = iWaypoint;
                    break;
                }
            }

            // 2. Set Goal depending on number of intersections found
            int previousWaypointIndex = mCurrentState.currentWaypointIndex - 1 >= 0 ? mCurrentState.currentWaypointIndex - 1 : mWaypointList.size() - 1;
            switch (intersections.size()) {
            case 0:
                // We seem to have left the route (e.g., because of high speed), reuse previous goal to  get back to route
                break;
            case 1:
                mCurrentState.currentGoal.setX(intersections[0].x());
                mCurrentState.currentGoal.setY(intersections[0].y());
                mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(previousWaypointIndex), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                break;
            case 2:
                // Take intersection closest to current waypoint (most progress)
                if (QLineF(intersections[0], currentWaypointPoint).length()
                        < QLineF(intersections[1], currentWaypointPoint).length()) {
                    mCurrentState.currentGoal.setX(intersections[0].x());
                    mCurrentState.currentGoal.setY(intersections[0].y());
                    mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(previousWaypointIndex), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                }
                else {
                    mCurrentState.currentGoal.setX(intersections[1].x());
                    mCurrentState.currentGoal.setY(intersections[1].y());
                    mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(previousWaypointIndex), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                }
                break;
            default:
                break;
            }

            // 3. Determine closest waypoint to vehicle, it determines attributes
            PosPoint closestWaypoint;
            if (QLineF(currentVehiclePositionXY, mWaypointList.at(previousWaypointIndex).getPoint()).length()
                    < QLineF(currentVehiclePositionXY, mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint()).length())
                closestWaypoint = mWaypointList.at(previousWaypointIndex);
            else
                closestWaypoint = mWaypointList.at(mCurrentState.currentWaypointIndex);
            mCurrentState.currentGoal.setAttributes(closestWaypoint.getAttributes());

            // 4. Update control for current goal
            trol(mCurrentState.currentGoal);
        }
    } break;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_APPROACHING_END_GOAL: {
        QPointF vehicleAlignmentReferencePointXY = getVehicleAlignmentReferencePosPoint().getPoint();

        const PosPoint& endGoalPosPoint = mWaypointList.at(mCurrentState.currentWaypointIndex);
        QPointF endGoalPointXY = endGoalPosPoint.getPoint();
        QLineF endGoalToreferencePointLine(endGoalPointXY, vehicleAlignmentReferencePointXY);
        double endGoalToreferencePointDistance = endGoalToreferencePointLine.length();
        bool isAlignedWithEndGoal = endGoalToreferencePointDistance < mEndGoalAlignmentThreshold;

        if (isAlignedWithEndGoal) {
            if (!mCurrentState.repeatRoute) {
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
                qDebug() << "Goal reached with accuracy:" << endGoalToreferencePointDistance << "m.";
            } else {
                qDebug() << "Goal reached with accuracy:" << endGoalToreferencePointDistance << "m. Repeating the route...";
            }
        } else {
            QPointF previousWayPointPointXY;
            if (mCurrentState.currentWaypointIndex > 0) {
                previousWayPointPointXY = mWaypointList[mCurrentState.currentWaypointIndex - 1].getPoint();
            } else {
                previousWayPointPointXY = mCurrentState.startPointXY;
            }
            QLineF endGoalToPreviousWayPointLine(endGoalPointXY, previousWayPointPointXY);

            double angleBetweenLines = endGoalToreferencePointLine.angleTo(endGoalToPreviousWayPointLine);
            if (angleBetweenLines > 180) { // Convert to -180 to 180 range
                angleBetweenLines -= 360;
            }

            bool hasOvershotEndGoal = false;
            if (fabs(angleBetweenLines) > 90) { // check if we are still approaching end goal or overshot it
                hasOvershotEndGoal = true;
            }
            if (hasOvershotEndGoal && !mRetryAfterEndGoalOvershot) {
                stop();
                qDebug() << "Goal overshot! Stopped waypoint follower with distance to goal:" << endGoalToreferencePointDistance << "m.";
            } else {
                double purePursuitRadius_ = purePursuitRadius();
                QSharedPointer<VehicleState> referenceVehicleState = getReferenceVehicleState();
                double endGoalToVehicleDistance = QLineF(endGoalPointXY, referenceVehicleState->getPosition(mPosTypeUsed).getPoint()).length();
                double extensionDistance = std::max(purePursuitRadius_ - endGoalToVehicleDistance, 0.0);
                double extensionRatio = (-extensionDistance) / endGoalToPreviousWayPointLine.length();
                QPointF extendedGoalPointXY = endGoalToPreviousWayPointLine.pointAt(extensionRatio);
                mCurrentState.currentGoal.setXY(extendedGoalPointXY.x(), extendedGoalPointXY.y());

                if (mAdaptiveApproachSpeedEnabled && mMinApproachSpeed > 0.0) {
                    double adaptiveSpeed = (purePursuitRadius_ - extensionDistance) / purePursuitRadius_ * endGoalPosPoint.getSpeed();
                    if (endGoalPosPoint.getSpeed() > 0) {
                        adaptiveSpeed = std::max(adaptiveSpeed, mMinApproachSpeed);
                    } else {
                        adaptiveSpeed = std::min(adaptiveSpeed, -mMinApproachSpeed);
                    }
                    mCurrentState.currentGoal.setSpeed(adaptiveSpeed);
                } else {
                    mCurrentState.currentGoal.setSpeed(endGoalPosPoint.getSpeed());
                }
                trol(mCurrentState.currentGoal);
            }
        }
    } break;
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED:
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        mCurrentState.startPointXY = QPointF();
        stop();
        break;

    default:
        break;
    }
}

void PurepursuitWaypointFollower::updateControl(const PosPoint &goal)
{
    PosPoint currentPos = mVehicleState->getPosition(mPosTypeUsed);   // ENU position
    double cappedSpeed = goal.getSpeed();   // the goal's desired speed

    // Check each speed limit region (if inside overlapping regions, the most restrictive limit will be applied)
    for (const SpeedLimitRegion &region : mSpeedLimitRegions) {
        if (ZigZagRouteGenerator::isPointWithin(currentPos, region.boundary)) {
            cappedSpeed = std::min(cappedSpeed, region.maxSpeed);
        }
    }

    if (isOnVehicle()) {
        mMovementController->setDesiredSteeringCurvature(mVehicleState->getCurvatureToPointInENU(goal.getPoint(), mPosTypeUsed));
        mMovementController->setDesiredSpeed(cappedSpeed);
        mMovementController->setDesiredAttributes(goal.getAttributes());

        mVehicleState->setAutopilotTargetPoint(goal.getPoint());
    } else {
        // NOTE: we calculate in ENU coordinates
        xyz_t positionDifference = {goal.getX() - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getX(),
                                    goal.getY() - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getY(),
                                    mCurrentState.overrideAltitude - mVehicleState->getPosition(mPosTypeUsed).getHeight()};
        double positionDiffDistance = sqrtf(positionDifference.x*positionDifference.x + positionDifference.y*positionDifference.y + positionDifference.z*positionDifference.z);
        double velocityFactor = cappedSpeed / positionDiffDistance;

        double yawDeg = atan2(goal.getY() - mVehicleState->getPosition(mPosTypeUsed).getY(),
                              goal.getX() - mVehicleState->getPosition(mPosTypeUsed).getX()) * 180.0 / M_PI;

        mVehicleConnection->requestVelocityAndYaw({positionDifference.x*velocityFactor, positionDifference.y*velocityFactor, positionDifference.z*velocityFactor}, yawDeg);
    }
}

PosType PurepursuitWaypointFollower::getPosTypeUsed() const
{
    return mPosTypeUsed;
}

void PurepursuitWaypointFollower::setPosTypeUsed(const PosType &posTypeUsed)
{
    mPosTypeUsed = posTypeUsed;
}

double PurepursuitWaypointFollower::getPurePursuitRadius() const
{
    return mCurrentState.purePursuitRadius;
}

void PurepursuitWaypointFollower::setPurePursuitRadius(double value)
{
    mCurrentState.purePursuitRadius = value;
}

void PurepursuitWaypointFollower::setAdaptivePurePursuitRadiusCoefficient(double coefficient)
{
    mCurrentState.adaptivePurePursuitRadiusCoefficient = coefficient;
}

double PurepursuitWaypointFollower::getAdaptivePurePursuitRadiusCoefficient()
{
    return mCurrentState.adaptivePurePursuitRadiusCoefficient;
}

bool PurepursuitWaypointFollower::getRepeatRoute() const
{
    return mCurrentState.repeatRoute;
}

void PurepursuitWaypointFollower::setRepeatRoute(bool value)
{
    mCurrentState.repeatRoute = value;
}

const PosPoint PurepursuitWaypointFollower::getCurrentGoal()
{
    return mCurrentState.currentGoal;
}

double PurepursuitWaypointFollower::getInterpolatedSpeed(const PosPoint &currentGoal, const PosPoint &lastWaypoint, const PosPoint &nextWaypoint)
{
    // Linear interpolation
    double distanceToNextWaypoint = currentGoal.getDistanceTo(nextWaypoint);
    double distanceBetweenWaypoints = lastWaypoint.getDistanceTo(nextWaypoint);
    double x = distanceBetweenWaypoints - distanceToNextWaypoint;

    return lastWaypoint.getSpeed() + (nextWaypoint.getSpeed()-lastWaypoint.getSpeed())*(x/distanceBetweenWaypoints);
}

void PurepursuitWaypointFollower::calculateDistanceOfRouteLeft(const QPointF currentVehiclePositionXY)
{
    double distance = QLineF(currentVehiclePositionXY, mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint()).length();

    for (int index = mCurrentState.currentWaypointIndex; index < mWaypointList.size()-1; index++) {
        distance += QLineF(mWaypointList.at(index).getPoint(), mWaypointList.at(index+1).getPoint()).length();
    }
    emit distanceOfRouteLeft(distance);
}

double PurepursuitWaypointFollower::purePursuitRadius()
{
    if (mCurrentState.adaptivePurePursuitRadius) {
        double dynamicRadius =  mCurrentState.adaptivePurePursuitRadiusCoefficient * mVehicleState->getSpeed();

        if (dynamicRadius > mCurrentState.purePursuitRadius)
            mVehicleState->setAutopilotRadius(dynamicRadius);
        else
            mVehicleState->setAutopilotRadius(mCurrentState.purePursuitRadius);
    } else
        mVehicleState->setAutopilotRadius(mCurrentState.purePursuitRadius);

    return mVehicleState->getAutopilotRadius();
}

void PurepursuitWaypointFollower::setAdaptivePurePursuitRadiusActive(bool adaptive)
{
    mCurrentState.adaptivePurePursuitRadius = adaptive;
}

QList<PosPoint> PurepursuitWaypointFollower::getCurrentRoute()
{
    return mWaypointList;
}

PosPoint PurepursuitWaypointFollower::getVehicleAlignmentReferencePosPoint()
{
    PosPoint vehicleAlignmentReferencePosPoint;
    QSharedPointer<VehicleState> referenceVehicleState = getReferenceVehicleState();

    xyz_t rearAxleToReferencePointOffset;
    switch (mVehicleState->getEndGoalAlignmentType()) {
        case AutopilotEndGoalAlignmentType::CENTER: {
            rearAxleToReferencePointOffset = referenceVehicleState->getRearAxleToCenterOffset();
            vehicleAlignmentReferencePosPoint = referenceVehicleState->posInVehicleFrameToPosPointENU(rearAxleToReferencePointOffset, mPosTypeUsed);
        } break;
        case AutopilotEndGoalAlignmentType::FRONT_REAR_END: {
            rearAxleToReferencePointOffset = referenceVehicleState->getRearAxleToRearEndOffset();
            if(mWaypointList.at(mCurrentState.currentWaypointIndex).getSpeed() >= 0) {
                rearAxleToReferencePointOffset.x += referenceVehicleState->getLength();
            }
            vehicleAlignmentReferencePosPoint = referenceVehicleState->posInVehicleFrameToPosPointENU(rearAxleToReferencePointOffset, mPosTypeUsed);
        } break;
        case AutopilotEndGoalAlignmentType::REAR_AXLE:
        default:{
            vehicleAlignmentReferencePosPoint = referenceVehicleState->getPosition(mPosTypeUsed);
        } break;
    }

    return vehicleAlignmentReferencePosPoint;
}

QSharedPointer<VehicleState> PurepursuitWaypointFollower::getReferenceVehicleState()
{
    if (mVehicleState->hasTrailingVehicle() &&
        mCurrentState.currentWaypointIndex < mWaypointList.size() &&
        mWaypointList.at(mCurrentState.currentWaypointIndex).getSpeed() < 0) // position defined by trailer when backing (if exists)
    {
        return mVehicleState->getTrailingVehicle();
    } else {
        return mVehicleState;
    }
}

void PurepursuitWaypointFollower::addSpeedLimitRegion(const QList<PosPoint>& boundary, double maxSpeed_kmph)
{
    if (boundary.size() < 3) {
        qDebug() << "WARNING: Speed limit region requires at least 3 points to define a polygon.";
        return;
    }

    double maxSpeed_ms = maxSpeed_kmph / 3.6;  // convert km/h to m/s

    SpeedLimitRegion region;
    region.boundary = boundary;
    region.maxSpeed = maxSpeed_ms;
    mSpeedLimitRegions.append(region);

    qInfo() << "Added speed limit region with" << boundary.size() << "vertices and max speed" << maxSpeed_kmph << "km/h. Keep in mind, any potential inner regions or holes are ignored";
}

void PurepursuitWaypointFollower::clearSpeedLimitRegions()
{
    mSpeedLimitRegions.clear();
    qDebug() << "Cleared all speed limit regions";
}

QList<SpeedLimitRegion> PurepursuitWaypointFollower::getSpeedLimitRegions() const
{
    return mSpeedLimitRegions;
}

void PurepursuitWaypointFollower::loadSpeedLimitRegionsFile()
{
    QDir documentsDirectory(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation));   // OS agnostic

    QString folderName = "WayWise Speed Limits";
    QString folderPath = documentsDirectory.filePath(folderName);

    if (!documentsDirectory.exists(folderPath)) {
        if (documentsDirectory.mkpath(folderPath)) {
            qInfo() << "Speed limits folder created";
        } else {
            qWarning() << "Failed to create speed limits folder";
            return;
        }
    }

    QString fileName = "speedLimitRegions.json";
    QString filePath = documentsDirectory.filePath(folderName + "/" + fileName);

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qInfo() << "Could not open speed limit regions file:" << filePath << Qt::endl
                << "To create speed limit regions:" << Qt::endl
                << "1. Go to https://geojson.io/#map=2/0/20 to easily define Polygons geometry" << Qt::endl
                << "2. Copy the JSON source (make sure type is 'FeatureCollection')" << Qt::endl
                << "3. Store as \"speedLimitRegions.json\" in the Documents/WayWise Speed Limits folder"  << Qt::endl
                << "4. Add 'maxSpeed' in the 'properties' field (for example 'maxSpeed: 30.0') to set the speed limit (km/h) for a region";
        return;
    }

    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(file.readAll(), &parseError);
    file.close();

    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "JSON parse error:" << parseError.errorString();
        return;
    }

    if (!jsonDoc.isObject()) {
        qWarning() << "Invalid JSON: root must be an object";
        return;
    }

    parseSpeedLimitRegionsDocument(jsonDoc);
}

void PurepursuitWaypointFollower::parseSpeedLimitRegionsDocument(const QJsonDocument& jsonDoc)
{
    QJsonObject root = jsonDoc.object();

    // Validate file is a WGS‑84 GeoJSON (standard per RFC 7946)
    if (root["type"].toString() != "FeatureCollection") {
        qWarning() << "Invalid GeoJSON: type must be FeatureCollection";
        return;
    }

    QJsonArray features = root["features"].toArray();

    for (const QJsonValue& featureValue : features) {

        QJsonObject feature = featureValue.toObject();

        // --- properties ---
        double maxSpeed_kmph = feature["properties"].toObject()["maxSpeed"].toDouble(std::numeric_limits<double>::quiet_NaN()); // returns NaN if conversion fails
        if (!std::isfinite(maxSpeed_kmph) || maxSpeed_kmph < 0.0) {
            qWarning() << "Skipping feature: invalid 'maxSpeed' value";
            continue;
        }

        // --- geometry ---
        QJsonObject geometry = feature["geometry"].toObject();
        if (geometry["type"].toString() != "Polygon") {
            qWarning() << "Skipping non-Polygon geometry (only polygons are supported, not MultiPolygon, LineString, etc.)";
            continue;
        }

        // GeoJSON Polygon coordinates are [[[lon, lat], [lon, lat], ...]]
        QJsonArray coordinates = geometry["coordinates"].toArray();
        if (coordinates.isEmpty() || !coordinates[0].isArray()) {
            qWarning() << "Skipping empty coordinates";
            continue;
        }

        // Get the outer ring (first array)
        QJsonArray outerRing = coordinates[0].toArray();

        llh_t enuRef = mVehicleState->getEnuRef();
        QList<PosPoint> boundary;
        for (const QJsonValue& coordValue : outerRing) {
            QJsonArray coord = coordValue.toArray();
            if (coord.size() < 2) {
                qWarning() << "Skipping invalid coordinates";
                continue;
            }

            // GeoJSON uses [longitude, latitude] order
            double longitude = coord[0].toDouble();
            double latitude = coord[1].toDouble();
            double altitude = coord.size() > 2 ? coord[2].toDouble() : 0.0; // use 0 if altitude is absent

            // latitude/longitude range checks
            if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) continue;

            // Convert lat/lon to ENU coordinates
            llh_t llh{latitude, longitude, altitude};
            xyz_t enu = coordinateTransforms::llhToEnu(enuRef, llh);

            PosPoint point;
            point.setX(enu.x);
            point.setY(enu.y);
            point.setHeight(enu.z);
            boundary.append(point);
        }

        if (!boundary.isEmpty()) {
            addSpeedLimitRegion(boundary, maxSpeed_kmph);
        }
    }
}
