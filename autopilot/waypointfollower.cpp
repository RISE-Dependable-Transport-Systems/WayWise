/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "waypointfollower.h"
#include <cmath>
#include <QDebug>
#include <QLineF>

WaypointFollower::WaypointFollower(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &WaypointFollower::updateState);

    // Follow point requires continuous updates of the point to follow
    mCurrentState.followPointTimedOut = true;
    WaypointFollower::mFollowPointHeartbeatTimer.setSingleShot(true);
    connect(&mFollowPointHeartbeatTimer, &QTimer::timeout, this, [&](){
        if ((mCurrentState.stmState == FOLLOW_POINT_FOLLOWING || mCurrentState.stmState == FOLLOW_POINT_WAITING) && this->isActive()) {
            qDebug() << "WARNING: Follow point timed out. Stopping WaypointFollower.";
            this->stop();
        }
        mCurrentState.followPointTimedOut = true;
    });
}

WaypointFollower::WaypointFollower(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed)
{
    mVehicleConnection = vehicleConnection;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &WaypointFollower::updateState);
    setPosTypeUsed(posTypeUsed);

    // TODO: follow point not supported for now
}

void WaypointFollower::clearRoute()
{
    mWaypointList.clear();
}

void WaypointFollower::addWaypoint(const PosPoint &point)
{
    mWaypointList.append(point);
}

void WaypointFollower::addRoute(const QList<PosPoint> &route)
{
    mWaypointList.append(route);
}

void WaypointFollower::startFollowingRoute(bool fromBeginning)
{
    if (fromBeginning || mCurrentState.stmState == NONE)
        mCurrentState.stmState = FOLLOW_ROUTE_INIT;

    if (mCurrentState.stmState == FOLLOW_POINT_FOLLOWING || mCurrentState.stmState == FOLLOW_POINT_WAITING) {
        mCurrentState.stmState = NONE;
        qDebug() << "WARNING: trying to follow route while follow point is active. Stopping WaypointFollower.";
    } else {
        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    }
}

bool WaypointFollower::isActive()
{
    return mUpdateStateTimer.isActive();
}

void WaypointFollower::stop()
{
    mUpdateStateTimer.stop();

    if (isOnVehicle()) {
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
    } else {
        // TODO: support for updating target position continuously

    }
}

void WaypointFollower::startFollowPoint()
{
    // Check that we got a recent point to follow
    if (isOnVehicle() && mCurrentState.currentFollowPointInVehicleFrame.getTime() > mCurrentState.currentGoal.getTime()) {
        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mCurrentState.stmState = FOLLOW_POINT_FOLLOWING;
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    } else {
        if (isOnVehicle())
            qDebug() << "WARNING: Follow Point did not get a recent point to follow. Exiting.";
        else
            qDebug() << "WARNING: Follow Point not implemented for remote connections. Exiting.";
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    }
}

void WaypointFollower::resetState()
{
    mUpdateStateTimer.stop();

    mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    mCurrentState.currentWaypointIndex = mWaypointList.size();
}

double WaypointFollower::getCurvatureToPointInENU(QSharedPointer<VehicleState> vehicleState, const QPointF &point, PosType vehiclePosType)
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
    const double newX = cos(currYaw_rad)*pointInVehicleFrame.x() - sin(currYaw_rad)*pointInVehicleFrame.y();
    const double newY = sin(currYaw_rad)*pointInVehicleFrame.x() + cos(currYaw_rad)*pointInVehicleFrame.y();
    pointInVehicleFrame.setX(newX);
    pointInVehicleFrame.setY(newY);

    return getCurvatureToPointInVehicleFrame(pointInVehicleFrame);
}

double WaypointFollower::getCurvatureToPointInENU(const QPointF &point)
{
    return getCurvatureToPointInENU(isOnVehicle() ? mMovementController->getVehicleState() : mVehicleConnection->getVehicleState(), point, mPosTypeUsed);
}

double WaypointFollower::getCurvatureToPointInVehicleFrame(const QPointF &point)
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

void WaypointFollower::updateState()
{
    QPointF currentVehiclePositionXY = getCurrentVehiclePosition().getPoint();

    switch (mCurrentState.stmState) {
    case NONE:
        qDebug() << "WARNING: WayPointFollower running uninitialized statemachine.";
        break;

    // FOLLOW_POINT: we follow a point that is moving "follow me", works on vehicle frame to be independent of positioning
    case FOLLOW_POINT_FOLLOWING: {
        // draw straight line to follow point and apply purePursuitRadius to find intersection
        QLineF carToFollowPointLine(QPointF(0,0), mCurrentState.currentFollowPointInVehicleFrame.getPoint());
        QVector<QPointF> intersections = findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(QPointF(0,0), mCurrentState.purePursuitRadius), carToFollowPointLine);

        if (intersections.size()) {
            // Translate to ENU for correct representation of currentGoal (when positioning is working), TODO: general transform in vehicleState?
            PosPoint carPosition = getCurrentVehiclePosition();

            // clockwise rotation
            double currYaw_rad = carPosition.getYaw() * (M_PI / 180.0);
            double newX =  cos(currYaw_rad)*intersections[0].x() + sin(currYaw_rad)*intersections[0].y();
            double newY = -sin(currYaw_rad)*intersections[0].x() + cos(currYaw_rad)*intersections[0].y();

            // translation
            newX += carPosition.getX();
            newY += carPosition.getY();

            mCurrentState.currentGoal.setXY(newX, newY);
            // Timestamp currentGoal for timeout in case currentFollowPoint is not updated anymore
            mCurrentState.currentGoal.setTime(mCurrentState.currentFollowPointInVehicleFrame.getTime());

            mMovementController->setDesiredSteeringCurvature(getCurvatureToPointInVehicleFrame(QPointF(intersections[0].x(), intersections[0].y())));
            mMovementController->setDesiredSpeed(mCurrentState.followPointSpeed);
        } else // FollowPoint within circle -> wait
            mCurrentState.stmState = FOLLOW_POINT_WAITING;
    } break;

    case FOLLOW_POINT_WAITING:
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);

        if (QLineF(QPointF(0,0), mCurrentState.currentFollowPointInVehicleFrame.getPoint()).length() > mCurrentState.purePursuitRadius)
        {
            mCurrentState.stmState = FOLLOW_POINT_FOLLOWING;
        }
        break;

    // FOLLOW_ROUTE: waypoints describe a route to be followed waypoint by waypoint
    case FOLLOW_ROUTE_INIT:
        currentVehiclePositionXY = getCurrentVehiclePosition().getPoint();

        if (mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = 0;
            mCurrentState.currentGoal = mWaypointList.at(0);
            mCurrentState.stmState = FOLLOW_ROUTE_GOTO_BEGIN;
        } else
            mCurrentState.stmState = FOLLOW_ROUTE_FINISHED;
        break;

    case FOLLOW_ROUTE_GOTO_BEGIN: {
        currentVehiclePositionXY = getCurrentVehiclePosition().getPoint();

        // draw straight line to first point and apply purePursuitRadius to find intersection
        QLineF carToStartLine(currentVehiclePositionXY, mWaypointList.at(0).getPoint());
        QVector<QPointF> intersections = findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePositionXY, mCurrentState.purePursuitRadius), carToStartLine);

        if (intersections.size()) {
            mCurrentState.currentGoal.setXY(intersections[0].x(), intersections[0].y());
            updateControl(mCurrentState.currentGoal);
        } else // first waypoint within circle -> start route
            mCurrentState.stmState = FOLLOW_ROUTE_FOLLOWING;
    } break;

    case FOLLOW_ROUTE_FOLLOWING: {
        currentVehiclePositionXY = getCurrentVehiclePosition().getPoint();
        QPointF currentWaypointPoint = mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint();

        if (QLineF(currentVehiclePositionXY, currentWaypointPoint).length() < mCurrentState.purePursuitRadius) // consider previous waypoint as reached
            mCurrentState.currentWaypointIndex++;

        if (mCurrentState.currentWaypointIndex == mWaypointList.size() && !mCurrentState.repeatRoute)
                mCurrentState.stmState = FOLLOW_ROUTE_FINISHED;
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

                intersections = findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePositionXY, mCurrentState.purePursuitRadius), iLineSegment);
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

    case FOLLOW_ROUTE_FINISHED:
        mCurrentState.stmState = NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        stop();
        break;

    default:
        break;
    }
}

void WaypointFollower::updateControl(const PosPoint &goal)
{
    if (isOnVehicle()) {
        mMovementController->setDesiredSteeringCurvature(getCurvatureToPointInENU(goal.getPoint()));
        mMovementController->setDesiredSpeed(goal.getSpeed());
        mMovementController->setDesiredAttributes(goal.getAttributes());
    } else {
        mVehicleConnection->requestGotoENU(xyz_t {goal.getX(), goal.getY(), mCurrentState.overrideAltitude});
    }
}

PosType WaypointFollower::getPosTypeUsed() const
{
    return mPosTypeUsed;
}

void WaypointFollower::setPosTypeUsed(const PosType &posTypeUsed)
{
    mPosTypeUsed = posTypeUsed;
}

double WaypointFollower::getPurePursuitRadius() const
{
    return mCurrentState.purePursuitRadius;
}

void WaypointFollower::setPurePursuitRadius(double value)
{
    mCurrentState.purePursuitRadius = value;
}

bool WaypointFollower::getRepeatRoute() const
{
    return mCurrentState.repeatRoute;
}

void WaypointFollower::setRepeatRoute(bool value)
{
    mCurrentState.repeatRoute = value;
}

const PosPoint WaypointFollower::getCurrentGoal()
{
    return mCurrentState.currentGoal;
}

double WaypointFollower::getInterpolatedSpeed(const PosPoint &currentGoal, const PosPoint &lastWaypoint, const PosPoint &nextWaypoint)
{
    // Linear interpolation
    double distanceToNextWaypoint = currentGoal.getDistanceTo(nextWaypoint);
    double distanceBetweenWaypoints = lastWaypoint.getDistanceTo(nextWaypoint);
    double x = distanceBetweenWaypoints - distanceToNextWaypoint;

    return lastWaypoint.getSpeed() + (nextWaypoint.getSpeed()-lastWaypoint.getSpeed())*(x/distanceBetweenWaypoints);
}

void WaypointFollower::updateFollowPointInVehicleFrame(const PosPoint &point)
{
    mCurrentState.currentFollowPointInVehicleFrame = point;

    if ((mCurrentState.stmState == FOLLOW_POINT_FOLLOWING || mCurrentState.stmState == FOLLOW_POINT_WAITING) &&
         (mCurrentState.currentFollowPointInVehicleFrame.getTime() > mCurrentState.currentGoal.getTime())) {
        if (mCurrentState.followPointTimedOut) {
            qDebug() << "Follow Point: timeout reset.";
        }

        mFollowPointHeartbeatTimer.start(mFollowPointTimeout_ms);
        mCurrentState.followPointTimedOut = false;
    }
}

PosPoint WaypointFollower::getCurrentVehiclePosition()
{
    if (isOnVehicle())
        return mMovementController->getVehicleState()->getPosition(mPosTypeUsed);
    else
        return mVehicleConnection->getVehicleState()->getPosition(mPosTypeUsed);
}

double WaypointFollower::getFollowPointSpeed() const
{
    return mCurrentState.followPointSpeed;
}

void WaypointFollower::setFollowPointSpeed(double value)
{
    mCurrentState.followPointSpeed = value;
}
