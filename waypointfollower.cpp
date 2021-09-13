#include "waypointfollower.h"
#include <cmath>
#include <QDebug>
#include <QLineF>

WaypointFollower::WaypointFollower(QSharedPointer<MovementController> movementController)
{
    mMovementController = movementController;
    connect(&mUpdateStateTimer, &QTimer::timeout, this, &WaypointFollower::updateState);

    // --- Handle heartbeat from sensor
    mSensorHeartbeat = false;
    WaypointFollower::mSensorHeartbeatTimer.setSingleShot(true);
    connect(&mSensorHeartbeatTimer, &QTimer::timeout, this, [&](){
        if ((mCurrentState.stmState == FOLLOW_POINT_FOLLOWING || mCurrentState.stmState == FOLLOW_POINT_WAITING) && this->isActive()) {
            qDebug() << "WARNING: Follow point sensor heartbeat missing. Exiting follow me!";
            mSensorHeartbeat =false;
            this->stop();
        } else if ((mCurrentState.stmState != NONE) && this->isActive()){
            qDebug() << "WARNING: Visual sensor heartbeat missing. Emergency brake deactivated!";
            mSensorHeartbeat = false;
        } else
            mSensorHeartbeat = false;
    });
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
        mCurrentState.stmState = FOLLOW_ROUTE_INIT;

    if (mCurrentState.stmState == FOLLOW_POINT_FOLLOWING || mCurrentState.stmState == FOLLOW_POINT_WAITING)
    {
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
        qDebug() << "Command not allowed! Press reset button to start following route";
    }

    mSensorHeartbeatTimer.start(mCountdown_ms);
    mUpdateStateTimer.start(mUpdateStatePeriod_ms);
}

bool WaypointFollower::isActive()
{
    return mUpdateStateTimer.isActive();
}

void WaypointFollower::stop()
{
    mUpdateStateTimer.stop();
    mMovementController->setDesiredSteering(0.0);
    mMovementController->setDesiredSpeed(0.0);
}

void WaypointFollower::startFollowMe()
{
    mCurrentState.currentGoal = mFollowMePoint;
    if (mCurrentState.currentGoal.getTime() > mFollowMeTimeStamp) {
        mSensorHeartbeatTimer.start(mCountdown_ms);
        mCurrentState.stmState = FOLLOW_POINT_FOLLOWING;
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    } else {
        qDebug() << "WARNING: Follow point sensor heartbeat missing. Exiting follow point!";
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    }
}

void WaypointFollower::resetState()
{
    mUpdateStateTimer.stop();

    mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
    mCurrentState.currentWaypointIndex = mWaypointList.size();
}

double WaypointFollower::getCurvatureToPoint(QSharedPointer<VehicleState> vehicleState, const QPointF &point, PosType vehiclePosType)
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

    // 2. calc steering angle (pure pursuit)
    double distanceSquared = pow(pointInVehicleFrame.x(), 2) + pow(pointInVehicleFrame.y(), 2);
    double steeringAngleProportional = (2*pointInVehicleFrame.y()) / distanceSquared;
    return -steeringAngleProportional;
}

double WaypointFollower::getCurvatureToPoint(const QPointF &point)
{
    return getCurvatureToPoint(mMovementController->getVehicleState(), point, mPosTypeUsed);
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
    QPointF currentVehiclePosition = mMovementController->getVehicleState()->getPosition(mPosTypeUsed).getPoint();

    switch (mCurrentState.stmState) {
    case NONE:
        qDebug() << "WARNING: WayPointFollower running uninitialized statemachine.";
        break;

    // FOLLOW_POINT: we follow a point that is moving "follow me"
    case FOLLOW_POINT_FOLLOWING:
        currentVehiclePosition = mMovementController->getVehicleState()->getPosition(mPosTypeUsed).getPoint();
        mCurrentState.currentGoal = mFollowMePoint;

        if (mCurrentState.currentGoal.getTime() > mFollowMeTimeStamp) {
            if (!mSensorHeartbeat) {
                qDebug() << "Follow point sensor heartbeat reset";
            }
            mFollowMeTimeStamp = mCurrentState.currentGoal.getTime();
            mSensorHeartbeatTimer.start(mCountdown_ms);
            mSensorHeartbeat = true;
        }

        if (QLineF(currentVehiclePosition, mCurrentState.currentGoal.getPoint()).length() < mCurrentState.purePursuitRadius)
            mCurrentState.stmState = FOLLOW_POINT_WAITING;
        else {
            mMovementController->setDesiredSteeringCurvature(getCurvatureToPoint(mCurrentState.currentGoal.getPoint()));
            mMovementController->setDesiredSpeed(mCurrentState.followPointSpeed);
        }
        break;

    case FOLLOW_POINT_WAITING:
        currentVehiclePosition = mMovementController->getVehicleState()->getPosition(mPosTypeUsed).getPoint();
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
        mCurrentState.currentGoal = mFollowMePoint;

        if (mCurrentState.currentGoal.getTime() > mFollowMeTimeStamp) {
            if (!mSensorHeartbeat) {
                qDebug() << "Follow point sensor heartbeat reset";
            }
            mFollowMeTimeStamp = mCurrentState.currentGoal.getTime();
            mSensorHeartbeatTimer.start(mCountdown_ms);
            mSensorHeartbeat = true;
        }

        if (QLineF(currentVehiclePosition, mCurrentState.currentGoal.getPoint()).length() > mCurrentState.purePursuitRadius)
        {
            mCurrentState.stmState = FOLLOW_POINT_FOLLOWING;
        }
        break;

    // FOLLOW_ROUTE: waypoints describe a route to be followed waypoint by waypoint
    case FOLLOW_ROUTE_INIT:
        currentVehiclePosition = mMovementController->getVehicleState()->getPosition(mPosTypeUsed).getPoint();

        // Check heartbeat from visual sensor
        if (mFollowMePoint.getTime() > mFollowMeTimeStamp) {
            if (!mSensorHeartbeat) {
                qDebug() << "Emergency brake activated!";
            }
            mFollowMeTimeStamp = mFollowMePoint.getTime();
            mSensorHeartbeatTimer.start(mCountdown_ms);
            mSensorHeartbeat = true;

            // Emergency brake if object detected
            if (QLineF(currentVehiclePosition, mFollowMePoint.getPoint()).length() > 1 && QLineF(currentVehiclePosition, mFollowMePoint.getPoint()).length() < 10) { // TODO: decide how close the objects need to be
                mMovementController->setDesiredSteering(0.0);
                mMovementController->setDesiredSpeed(0.0);
                mUpdateStateTimer.stop();
                break;
            }
        }

        if (mWaypointList.size()) {
            mCurrentState.currentWaypointIndex = 0;
            mCurrentState.currentGoal = mWaypointList.at(0);
            mCurrentState.stmState = FOLLOW_ROUTE_GOTO_BEGIN;
        } else
            mCurrentState.stmState = FOLLOW_ROUTE_FINISHED;
        break;

    case FOLLOW_ROUTE_GOTO_BEGIN:
        currentVehiclePosition = mMovementController->getVehicleState()->getPosition(mPosTypeUsed).getPoint();

        // Check heartbeat from visual sensor
        if (mFollowMePoint.getTime() > mFollowMeTimeStamp) {
            if (!mSensorHeartbeat) {
                qDebug() << "Emergency brake activated!";
            }
            mFollowMeTimeStamp = mFollowMePoint.getTime();
            mSensorHeartbeatTimer.start(mCountdown_ms);
            mSensorHeartbeat = true;

            // Emergency brake if object detected
            if (QLineF(currentVehiclePosition, mFollowMePoint.getPoint()).length() > 1 && QLineF(currentVehiclePosition, mFollowMePoint.getPoint()).length() < 10) { // TODO: decide how close the objects need to be
                mMovementController->setDesiredSteering(0.0);
                mMovementController->setDesiredSpeed(0.0);
                mUpdateStateTimer.stop();
                break;
            }
        }

        mMovementController->setDesiredSteeringCurvature(getCurvatureToPoint(mCurrentState.currentGoal.getPoint()));
        mMovementController->setDesiredSpeed(mCurrentState.currentGoal.getSpeed());

        if (QLineF(currentVehiclePosition, mCurrentState.currentGoal.getPoint()).length() < mCurrentState.purePursuitRadius) // TODO: initially bigger distance (might be coming from bad angle)?
            mCurrentState.stmState = FOLLOW_ROUTE_FOLLOWING;
        break;

    case FOLLOW_ROUTE_FOLLOWING: {
        currentVehiclePosition = mMovementController->getVehicleState()->getPosition(mPosTypeUsed).getPoint();
        QPointF currentWaypoint = mWaypointList.at(mCurrentState.currentWaypointIndex).getPoint();

        // Check heartbeat from visual sensor
        if (mFollowMePoint.getTime() > mFollowMeTimeStamp) {
            if (!mSensorHeartbeat) {
                qDebug() << "Emergency brake activated!";
            }
            mFollowMeTimeStamp = mFollowMePoint.getTime();
            mSensorHeartbeatTimer.start(mCountdown_ms);
            mSensorHeartbeat = true;

            // Emergency brake if object detected
            if (QLineF(currentVehiclePosition, mFollowMePoint.getPoint()).length() > 1 && QLineF(currentVehiclePosition, mFollowMePoint.getPoint()).length() < 10) { // TODO: decide how close the objects need to be
                mMovementController->setDesiredSteering(0.0);
                mMovementController->setDesiredSpeed(0.0);
                mUpdateStateTimer.stop();
                break;
            }
        }

        if (QLineF(currentVehiclePosition, currentWaypoint).length() < mCurrentState.purePursuitRadius) // consider previous waypoint as reached
            mCurrentState.currentWaypointIndex++;

        if (mCurrentState.currentWaypointIndex == mWaypointList.size())
            mCurrentState.stmState = FOLLOW_ROUTE_FINISHED;
        else {
            // --- Calculate current goal on route (which lies between two waypoints)
            // 1. Find intersection between circle around vehicle and route
            // look a number of points ahead and jump forward on route if applicable
            QVector<QPointF> intersections;
            const int lookaheadWaypointStartIndex = (mCurrentState.currentWaypointIndex + mCurrentState.numWaypointsLookahead - 1  < mWaypointList.size()) ?
                        (mCurrentState.currentWaypointIndex + mCurrentState.numWaypointsLookahead - 1) : mWaypointList.size() - 1;
            for (int i = lookaheadWaypointStartIndex; i >= mCurrentState.currentWaypointIndex; i--) { // step backwards through lookahead window until intersection is found
                QPointF iWaypoint = mWaypointList.at(i).getPoint();
                QLineF iLineSegment(mWaypointList.at(i-1).getPoint(), iWaypoint);

                intersections = findIntersectionsBetweenCircleAndLine(QPair<QPointF, double>(currentVehiclePosition, mCurrentState.purePursuitRadius), iLineSegment);
                if (intersections.size() > 0) {
                    mCurrentState.currentWaypointIndex = i;
                    currentWaypoint = iWaypoint;
                    break;
                }
            }

            // 2. Set Goal depending on number of intersections found
            switch (intersections.size()) {
            case 0:
                // We seem to have left the route (e.g., because of high speed), reuse previous goal to  get back to route
                break;
            case 1:
                mCurrentState.currentGoal.setX(intersections[0].x());
                mCurrentState.currentGoal.setY(intersections[0].y());
                mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(mCurrentState.currentWaypointIndex-1), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                break;
            case 2:
                // Take intersection closest to current waypoint (most progress)
                if (QLineF(intersections[0], currentWaypoint).length()
                        < QLineF(intersections[1], currentWaypoint).length()) {
                    mCurrentState.currentGoal.setX(intersections[0].x());
                    mCurrentState.currentGoal.setY(intersections[0].y());
                    mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(mCurrentState.currentWaypointIndex-1), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                }
                else {
                    mCurrentState.currentGoal.setX(intersections[1].x());
                    mCurrentState.currentGoal.setY(intersections[1].y());
                    mCurrentState.currentGoal.setSpeed(getInterpolatedSpeed(mCurrentState.currentGoal, mWaypointList.at(mCurrentState.currentWaypointIndex-1), mWaypointList.at(mCurrentState.currentWaypointIndex)));

                }
                break;
            default:
                break;
            }

            // 3. Update control for current goal
            mMovementController->setDesiredSteeringCurvature(getCurvatureToPoint(mCurrentState.currentGoal.getPoint()));
            mMovementController->setDesiredSpeed(mCurrentState.currentGoal.getSpeed());
        }
    } break;

    case FOLLOW_ROUTE_FINISHED:
        mMovementController->setDesiredSteering(0.0);
        mMovementController->setDesiredSpeed(0.0);
        mUpdateStateTimer.stop();
        mCurrentState.stmState = NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        break;

    default:
        break;
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

int WaypointFollower::getCurrentWaypointindex()
{
    return mCurrentState.currentWaypointIndex;
}

void WaypointFollower::setCurrentWaypointindex(int value)
{
    mCurrentState.currentWaypointIndex = value;
}

PosPoint WaypointFollower::getCurrentGoal()
{
    return mCurrentState.currentGoal;
}

void WaypointFollower::setCurrentGoal(PosPoint &point)
{
    mCurrentState.currentGoal = point;
}

WayPointFollowerSTMstates WaypointFollower::getSTMState()
{
    return mCurrentState.stmState;
}

void WaypointFollower::setSTMState(WayPointFollowerSTMstates state)
{
    mCurrentState.stmState = state;
}

double WaypointFollower::getInterpolatedSpeed(const PosPoint &currentGoal, const PosPoint &lastWaypoint, const PosPoint &nextWaypoint)
{
    // Linear interpolation
    double distanceToNextWaypoint = currentGoal.getDistanceTo(nextWaypoint);
    double distanceBetweenWaypoints = lastWaypoint.getDistanceTo(nextWaypoint);
    double x = distanceBetweenWaypoints - distanceToNextWaypoint;

    return lastWaypoint.getSpeed() + (nextWaypoint.getSpeed()-lastWaypoint.getSpeed())*(x/distanceBetweenWaypoints);
}

void WaypointFollower::updateFollowPoint(const PosPoint &point)
{
    mFollowMePoint = point;
}

double WaypointFollower::getFollowPointSpeed() const
{
    return mCurrentState.followPointSpeed;
}

void WaypointFollower::setFollowPointSpeed(double value)
{
    mCurrentState.followPointSpeed = value;
}
