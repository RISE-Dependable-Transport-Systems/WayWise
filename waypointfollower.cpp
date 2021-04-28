#include "waypointfollower.h"
#include <cmath>
#include <QDebug>
#include <QLineF>

WaypointFollower::WaypointFollower(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &WaypointFollower::updateState);
}

void WaypointFollower::clearRoute()
{
    mWaypointList.clear();
}

void WaypointFollower::addWaypoint(const PosPoint &point)
{
    mWaypointList.append(point);
}

void WaypointFollower::startFollowingRoute(bool fromBeginning)
{
    if (fromBeginning)
        resetState();

    mCurrentState.stmState = FOLLOW_ROUTE_INIT;
    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
}

void WaypointFollower::stopFollowingRoute()
{
    mUpdateStateTimer.stop();
    // TODO: brake
}

void WaypointFollower::resetState()
{
    mUpdateStateTimer.stop();

    mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    mCurrentState.currentWaypointIndex = mWaypointList.size();
}

double WaypointFollower::getCurvatureToPoint(QSharedPointer<VehicleState> vehicleState, const QPointF &point)
{
    // vehicleState and point in ENU frame
    // 1. transform point to vehicle frame, TODO: general transform in vehicleState?
    QPointF pointInVehicleFrame;
    // translate
    pointInVehicleFrame.setX(point.x()-vehicleState->getPosition().getX());
    pointInVehicleFrame.setY(point.y()-vehicleState->getPosition().getY());
    // rotate
    double currYaw = vehicleState->getPosition().getYaw();
    const double newX = cos(currYaw)*pointInVehicleFrame.x() - sin(currYaw)*pointInVehicleFrame.y();
    const double newY = sin(currYaw)*pointInVehicleFrame.x() + cos(currYaw)*pointInVehicleFrame.y();
    pointInVehicleFrame.setX(newX);
    pointInVehicleFrame.setY(newY);

    // 2. calc steering angle (pure pursuit)
    double distanceSquared = pow(pointInVehicleFrame.x(), 2) + pow(pointInVehicleFrame.y(), 2);
    double steeringAngleProportional = (2*pointInVehicleFrame.y()) / distanceSquared;
    return -steeringAngleProportional;
}

double WaypointFollower::getCurvatureToPoint(const QPointF &point)
{
    return getCurvatureToPoint(mMovementController->getVehicleState(), point);
}

WayPointFollowerState WaypointFollower::getCurrentState() const
{
    return mCurrentState;
}

void WaypointFollower::setCurrentState(const WayPointFollowerState&currentState)
{
    mCurrentState = currentState;
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
    switch (mCurrentState.stmState) {
    case NONE:
        qDebug() << "WARNING: WayPointFollower running uninitialized statemachine.";
        break;

    // FOLLOW_POINT: we follow a point that is moving "follow me"
    case FOLLOW_POINT_FOLLOWING:
        // TODO
        break;

    case FOLLOW_POINT_WAITING:
        // TODO
        break;

    // FOLLOW_ROUTE: waypoints describe a route to be followed waypoint by waypoint
    case FOLLOW_ROUTE_INIT:
        if (mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = 0;
            mCurrentState.currentGoal = mWaypointList.at(0);
            mCurrentState.stmState = FOLLOW_ROUTE_GOTO_BEGIN;
        } else
            mCurrentState.stmState = FOLLOW_ROUTE_FINISHED;
        break;

    case FOLLOW_ROUTE_GOTO_BEGIN:
        mMovementController->setDesiredSteering(getCurvatureToPoint(mCurrentState.currentGoal.getPoint())); // TODO: steering should be proportional to curvature (but not necessarily equal)
        mMovementController->setDesiredSpeed(mCurrentState.currentGoal.getSpeed());

        if (QLineF(mMovementController->getVehicleState()->getPosition().getPoint(), mCurrentState.currentGoal.getPoint()).length() < mCurrentState.purePursuitRadius) // TODO: initially bigger distance (might be coming from bad angle)?
            mCurrentState.stmState = FOLLOW_ROUTE_FOLLOWING;
        break;

    case FOLLOW_ROUTE_FOLLOWING: {
        QPointF currentVehiclePosition = mMovementController->getVehicleState()->getPosition().getPoint();
        QPointF currentWaypoint = mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint();

        if (QLineF(currentVehiclePosition, currentWaypoint).length() < mCurrentState.purePursuitRadius) // consider previous waypoint as reached
            mCurrentState.currentWaypointIndex++;

        if (mCurrentState.currentWaypointIndex == mWaypointList.size())
            mCurrentState.stmState = FOLLOW_ROUTE_FINISHED;
        else {
            // --- Calculate current goal on route (which lies between two waypoints)
            // 1. Find intersection between circle around vehicle and route (TODO: look a number of points ahead)
            QLineF currentLineSegment(mWaypointList.at(mCurrentState.currentWaypointIndex-1).getPoint(), currentWaypoint);
            QVector<QPointF> intersections = findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePosition, mCurrentState.purePursuitRadius), currentLineSegment);

            // 2. Set Goal depending on number of intersections found
            switch (intersections.size()) {
            case 0:
                // We seem to have left the route (e.g., because of high speed), reuse previous goal to get back to route
                break;
            case 1:
                mCurrentState.currentGoal = PosPoint(intersections[0].x(), intersections[0].y());
                break;
            case 2:
                if (QLineF(intersections[0], currentWaypoint).length()
                        < QLineF(intersections[1], currentWaypoint).length())
                    mCurrentState.currentGoal = PosPoint(intersections[0].x(), intersections[0].y());
                else
                    mCurrentState.currentGoal = PosPoint(intersections[1].x(), intersections[1].y());
                break;
            }

            // 3. Update control for current goal
            mMovementController->setDesiredSteering(getCurvatureToPoint(mCurrentState.currentGoal.getPoint())); // TODO: steering should be proportional to curvature (but not necessarily equal)
            mMovementController->setDesiredSpeed(mCurrentState.currentGoal.getSpeed()); // TODO interpolate speed between to points
        }
    } break;

    case FOLLOW_ROUTE_FINISHED:
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
        mUpdateStateTimer.stop();
        mCurrentState.stmState = NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        break;

    }
}

double WaypointFollower::getPurePursuitRadius() const
{
    return mCurrentState.purePursuitRadius;
}

void WaypointFollower::setPurePursuitRadius(double value)
{
    mCurrentState.purePursuitRadius = value;
}
