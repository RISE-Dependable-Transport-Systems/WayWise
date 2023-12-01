/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include <cmath>
#include <QDebug>
#include <QLineF>
#include "purepursuitwaypointfollower.h"
#include "WayWise/communication/parameterserver.h"

PurepursuitWaypointFollower::PurepursuitWaypointFollower(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &PurepursuitWaypointFollower::updateState);

    // Provide system parameters to ControlTower
    ParameterServer::getInstance()->provideParameter("PPRadius", std::bind(&PurepursuitWaypointFollower::setPurePursuitRadius, this, std::placeholders::_1), std::bind(&PurepursuitWaypointFollower::getPurePursuitRadius, this));
    ParameterServer::getInstance()->provideParameter("APPRC", std::bind(&PurepursuitWaypointFollower::setAdaptivePurePursuitRadiusCoefficient, this, std::placeholders::_1), std::bind(&PurepursuitWaypointFollower::getAdaptivePurePursuitRadiusCoefficient, this));


    // Follow point requires continuous updates of the point to follow
    mCurrentState.followPointTimedOut = true;
    PurepursuitWaypointFollower::mFollowPointHeartbeatTimer.setSingleShot(true);
    connect(&mFollowPointHeartbeatTimer, &QTimer::timeout, this, [&](){
        if ((mCurrentState.stmState == WayPointFollowerSTMstates::FOLLOW_POINT_FOLLOWING || mCurrentState.stmState == WayPointFollowerSTMstates::FOLLOW_POINT_WAITING) && this->isActive()) {
            qDebug() << "WARNING: Follow point timed out. Stopping WaypointFollower.";
            this->stop();
        }
        mCurrentState.followPointTimedOut = true;
    });
}

PurepursuitWaypointFollower::PurepursuitWaypointFollower(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed)
{
    mVehicleConnection = vehicleConnection;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &PurepursuitWaypointFollower::updateState);
    setPosTypeUsed(posTypeUsed);

    // TODO: follow point not supported for now
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

void PurepursuitWaypointFollower::addRoute(const QList<PosPoint> &route)
{
    mWaypointList.append(route);
}

void PurepursuitWaypointFollower::startFollowingRoute(bool fromBeginning)
{
    // Activate emergency brake for follow route
    emit activateEmergencyBrake();
    const auto &vehicleState = (isOnVehicle() ? mMovementController->getVehicleState() : mVehicleConnection->getVehicleState());
    mCurrentState.overrideAltitude = vehicleState->getPosition(mPosTypeUsed).getHeight();
    qDebug() << "Note: WaypointFollower starts following route. Height info from route is ignored (staying at" << QString::number(mCurrentState.overrideAltitude, 'g', 2) << "m).";

    if (fromBeginning || mCurrentState.stmState == WayPointFollowerSTMstates::NONE)
        mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT;

    if (mCurrentState.stmState == WayPointFollowerSTMstates::FOLLOW_POINT_FOLLOWING || mCurrentState.stmState == WayPointFollowerSTMstates::FOLLOW_POINT_WAITING) {
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
        qDebug() << "WARNING: trying to follow route while follow point is active. Stopping WaypointFollower.";
    } else {
        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    }
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
        mVehicleConnection->requestVelocityAndYaw({}, mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getYaw());
    }
}

void PurepursuitWaypointFollower::stop()
{
    mUpdateStateTimer.stop();
    (isOnVehicle() ? mMovementController->getVehicleState() : mVehicleConnection->getVehicleState())->setAutopilotRadius(0);
    holdPosition();
    emit deactivateEmergencyBrake();
}

void PurepursuitWaypointFollower::startFollowPoint()
{
    // Deactivate emergency brake in order to use follow point
    emit deactivateEmergencyBrake();
    // Check that we got a recent point to follow
    if (isOnVehicle() && mCurrentState.currentFollowPointInVehicleFrame.getTime() > mCurrentState.currentGoal.getTime()) {
        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_POINT_FOLLOWING;
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    } else {
        if (isOnVehicle())
            qDebug() << "WARNING: Follow Point did not get a recent point to follow. Exiting.";
        else
            qDebug() << "WARNING: Follow Point not implemented for remote connections. Exiting.";
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    }
}

void PurepursuitWaypointFollower::resetState()
{
    mUpdateStateTimer.stop();
    mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    mCurrentState.currentWaypointIndex = mWaypointList.size();
}

double PurepursuitWaypointFollower::getCurvatureToPointInENU(QSharedPointer<VehicleState> vehicleState, const QPointF &point, PosType vehiclePosType)
{
    // vehicleState and point assumed in ENU frame
    const PosPoint vehiclePos = vehicleState->getPosition(vehiclePosType);

    // 1. transform point to vehicle frame, TODO: general transform in vehicleState?
    QPointF pointInVehicleFrame;
    // translate
    pointInVehicleFrame.setX(point.x()-vehiclePos.getX());
    pointInVehicleFrame.setY(point.y()-vehiclePos.getY());
    // rotate
    double currYaw_rad = vehiclePos.getYaw() * M_PI / 180.0;
    const double newX = cos(-currYaw_rad)*pointInVehicleFrame.x() - sin(-currYaw_rad)*pointInVehicleFrame.y();
    const double newY = sin(-currYaw_rad)*pointInVehicleFrame.x() + cos(-currYaw_rad)*pointInVehicleFrame.y();
    pointInVehicleFrame.setX(newX);
    pointInVehicleFrame.setY(newY);

    return getCurvatureToPointInVehicleFrame(pointInVehicleFrame);
}

double PurepursuitWaypointFollower::getCurvatureToPointInENU(const QPointF &point)
{
    return getCurvatureToPointInENU(isOnVehicle() ? mMovementController->getVehicleState() : mVehicleConnection->getVehicleState(), point, mPosTypeUsed);
}

double PurepursuitWaypointFollower::getCurvatureToPointInVehicleFrame(const QPointF &point)
{
    // calc steering angle (pure pursuit)
    double distanceSquared = pow(point.x(), 2) + pow(point.y(), 2);
    double steeringAngleProportional = (2*point.y()) / distanceSquared;

    return -steeringAngleProportional;
}

// TODO: utility function, move to a more central place
QVector<QPointF> findIntersectionsBetweenCircleAndLine(QPair<QPointF,double> circle, QLineF line) {
    QVector<QPointF> intersections;

    double maxX = line.x1();
    double minX = line.x2();
    double maxY = line.y1();
    double minY = line.y2();
    if (maxX < minX) {
        maxX = line.x2();
        minX = line.x1();
    }
    if (maxY < minY) {
        maxY = line.y2();
        minY = line.y1();
    }

    double a = line.dx() * line.dx() + line.dy() * line.dy();
    double b = 2 * (line.dx() * (line.x1() - circle.first.x()) + line.dy() * (line.y1() - circle.first.y()));
    double c = (line.x1() - circle.first.x()) * (line.x1() - circle.first.x()) + (line.y1() - circle.first.y()) * (line.y1() - circle.first.y()) - circle.second * circle.second;

    double det = b * b - 4 * a * c;

    if ((a <= 1e-6) || (det < 0.0)) {
//         qDebug() << "No real solutions.";
    } else if (det == 0) {
//         qDebug() << "One solution.";
        double t = -b / (2 * a);
        double x = line.x1() + t * line.dx();
        double y = line.y1() + t * line.dy();

        if (x >= minX && x <= maxX &&
                y >= minY && y <= maxY)
            intersections.append(QPointF(x, y));
    } else {
//         qDebug() << "Two solutions.";
        double t = (-b + sqrtf(det)) / (2 * a);
        double x = line.x1() + t * line.dx();
        double y = line.y1() + t * line.dy();

        if (x >= minX && x <= maxX &&
                y >= minY && y <= maxY)
            intersections.append(QPointF(x, y));

        t = (-b - sqrtf(det)) / (2 * a);
        x = line.x1() + t * line.dx();
        y = line.y1() + t * line.dy();

        if (x >= minX && x <= maxX &&
                y >= minY && y <= maxY)
            intersections.append(QPointF(x, y));
    }

    return intersections;
}

void PurepursuitWaypointFollower::updateState()
{
    QPointF currentVehiclePositionXY = getCurrentVehiclePosition().getPoint();

    switch (mCurrentState.stmState) {
    case WayPointFollowerSTMstates::NONE:
        qDebug() << "WARNING: WayPointFollower running uninitialized statemachine.";
        break;

    // FOLLOW_POINT: we follow a point that is moving "follow me", works on vehicle frame to be independent of positioning
    case WayPointFollowerSTMstates::FOLLOW_POINT_FOLLOWING: {
        // draw straight line to follow point and apply purePursuitRadius to find intersection
        QLineF carToFollowPointLine(QPointF(0,0), mCurrentState.currentFollowPointInVehicleFrame.getPoint());
        QVector<QPointF> intersections = findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(QPointF(0,0), purePursuitRadius()), carToFollowPointLine);

        if (intersections.size()) {
            // Translate to ENU for correct representation of currentGoal (when positioning is working), TODO: general transform in vehicleState?
            PosPoint carPosition = getCurrentVehiclePosition();

            // clockwise rotation
            double currYaw_rad = carPosition.getYaw() * (M_PI / 180.0);
            double newX =  cos(-currYaw_rad)*intersections[0].x() + sin(-currYaw_rad)*intersections[0].y();
            double newY = -sin(-currYaw_rad)*intersections[0].x() + cos(-currYaw_rad)*intersections[0].y();

            // translation
            newX += carPosition.getX();
            newY += carPosition.getY();

            mCurrentState.currentGoal.setXY(newX, newY);
            // Timestamp currentGoal for timeout in case currentFollowPoint is not updated anymore
            mCurrentState.currentGoal.setTime(mCurrentState.currentFollowPointInVehicleFrame.getTime());

            mMovementController->setDesiredSteeringCurvature(getCurvatureToPointInVehicleFrame(QPointF(intersections[0].x(), intersections[0].y())));
            mMovementController->setDesiredSpeed(mCurrentState.followPointSpeed);
        } else // FollowPoint within circle -> wait
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_POINT_WAITING;
    } break;

    case WayPointFollowerSTMstates::FOLLOW_POINT_WAITING:
        holdPosition();

        if (QLineF(QPointF(0,0), mCurrentState.currentFollowPointInVehicleFrame.getPoint()).length() > mCurrentState.followPointDistance)
        {
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_POINT_FOLLOWING;
        }
        break;

    // FOLLOW_ROUTE: waypoints describe a route to be followed waypoint by waypoint
    case WayPointFollowerSTMstates::FOLLOW_ROUTE_INIT:
        currentVehiclePositionXY = getCurrentVehiclePosition().getPoint();
        if (mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = 0;
            mCurrentState.currentGoal = mWaypointList.at(0);
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN;
        } else
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
        break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_GOTO_BEGIN: {
        calculateDistanceOfRouteLeft();
        currentVehiclePositionXY = getCurrentVehiclePosition().getPoint();

        // draw straight line to first point and apply purePursuitRadius to find intersection
        QLineF carToStartLine(currentVehiclePositionXY, mWaypointList.at(0).getPoint());
        QVector<QPointF> intersections = findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePositionXY, purePursuitRadius()), carToStartLine);

        if (intersections.size()) {
            mCurrentState.currentGoal.setXY(intersections[0].x(), intersections[0].y());
            updateControl(mCurrentState.currentGoal);
        } else // first waypoint within circle -> start route
            mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING;
    } break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FOLLOWING: {
        calculateDistanceOfRouteLeft();
        currentVehiclePositionXY = getCurrentVehiclePosition().getPoint();
        QPointF currentWaypointPoint = mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint();

        if (QLineF(currentVehiclePositionXY, currentWaypointPoint).length() < purePursuitRadius()) // consider previous waypoint as reached
            mCurrentState.currentWaypointIndex++;

        if (mCurrentState.currentWaypointIndex == mWaypointList.size() && !mCurrentState.repeatRoute)
                mCurrentState.stmState = WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED;
        else {
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

                intersections = findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePositionXY, purePursuitRadius()), iLineSegment);
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
            updateControl(mCurrentState.currentGoal);
        }
    } break;

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED:
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        stop();
        break;

    default:
        break;
    }
}

void PurepursuitWaypointFollower::updateControl(const PosPoint &goal)
{
    if (isOnVehicle()) {
        mMovementController->setDesiredSteeringCurvature(getCurvatureToPointInENU(goal.getPoint()));
        mMovementController->setDesiredSpeed(goal.getSpeed());
        mMovementController->setDesiredAttributes(goal.getAttributes());
    } else {
        // NOTE: we calculate in ENU coordinates
        xyz_t positionDifference = {goal.getX() - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getX(),
                                    goal.getY() - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getY(),
                                    mCurrentState.overrideAltitude - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getHeight()};
        double positionDiffDistance = sqrtf(positionDifference.x*positionDifference.x + positionDifference.y*positionDifference.y + positionDifference.z*positionDifference.z);
        double velocityFactor = goal.getSpeed() / positionDiffDistance;

        double yawDeg = atan2(goal.getY() - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getY(),
                              goal.getX() - mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed).getX()) * 180.0 / M_PI;

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

void PurepursuitWaypointFollower::updateFollowPointInVehicleFrame(const PosPoint &point)
{
    mCurrentState.currentFollowPointInVehicleFrame = point;

    if ((mCurrentState.stmState == WayPointFollowerSTMstates::FOLLOW_POINT_FOLLOWING || mCurrentState.stmState == WayPointFollowerSTMstates::FOLLOW_POINT_WAITING) &&
         (mCurrentState.currentFollowPointInVehicleFrame.getTime() > mCurrentState.currentGoal.getTime())) {
        if (mCurrentState.followPointTimedOut) {
            qDebug() << "Follow Point: timeout reset.";
        }

        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mCurrentState.followPointTimedOut = false;
    }
}

PosPoint PurepursuitWaypointFollower::getCurrentVehiclePosition()
{
    if (isOnVehicle())
        return mMovementController->getVehicleState()->getPosition(mPosTypeUsed);
    else
        return mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed);
}

double PurepursuitWaypointFollower::getFollowPointSpeed() const
{
    return mCurrentState.followPointSpeed;
}

void PurepursuitWaypointFollower::setFollowPointSpeed(double value)
{
    mCurrentState.followPointSpeed = value;
}

void PurepursuitWaypointFollower::calculateDistanceOfRouteLeft()
{
    double distance = QLineF(getCurrentVehiclePosition().getPoint(), mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint()).length();

    for (int index = mCurrentState.currentWaypointIndex; index < mWaypointList.size()-1; index++) {
        distance += QLineF(mWaypointList.at(index).getPoint(), mWaypointList.at(index+1).getPoint()).length();
    }
    emit distanceOfRouteLeft(distance);
}

double PurepursuitWaypointFollower::purePursuitRadius()
{
    const auto& vehicleState = (isOnVehicle() ? mMovementController->getVehicleState() : mVehicleConnection->getVehicleState());

    if (mCurrentState.adaptivePurePursuitRadius) {
        double dynamicRadius =  mCurrentState.adaptivePurePursuitRadiusCoefficient * vehicleState->getSpeed();

        if (dynamicRadius > mCurrentState.purePursuitRadius)
            vehicleState->setAutopilotRadius(dynamicRadius);
        else
            vehicleState->setAutopilotRadius(mCurrentState.purePursuitRadius);
    } else
        vehicleState->setAutopilotRadius(mCurrentState.purePursuitRadius);

    return vehicleState->getAutopilotRadius();
}

void PurepursuitWaypointFollower::setAdaptivePurePursuitRadiusActive(bool adaptive)
{
    mCurrentState.adaptivePurePursuitRadius = adaptive;
}

QList<PosPoint> PurepursuitWaypointFollower::getRoute()
{
    return mWaypointList;
}
