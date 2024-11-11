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
#include "WayWise/vehicles/truckstate.h"

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

    // rotate
    double currYaw_rad = vehiclePos.getYaw() * M_PI / 180.0;
    auto truckState = qSharedPointerDynamicCast<TruckState>(vehicleState);
    
    double measuredTrailerAngle = truckState->getTrailerAngleRadians() ; // value from the angle sensor

    
    if (vehicleState->getSpeed()> 0 ){
        // 1. transform point to vehicle frame, TODO: general transform in vehicleState?
        QPointF pointInVehicleFrame;
        // // translate
        pointInVehicleFrame.setX(point.x()-vehiclePos.getX());
        pointInVehicleFrame.setY(point.y()-vehiclePos.getY());

        // double l1=truckState->getAxisDistance(); // in meters - tailer distance join to rear axle of trailer
        double l2 = 0.715;
       
        double theta_err =  atan2(pointInVehicleFrame.y(), pointInVehicleFrame.x()) - currYaw_rad; // the theta error between the target point and curect position 
        double desired_hitch_angle = atan(2*l2*sin(theta_err)); // desired trailer/hitch angle
        double k=1; // K is a try and error value, higher value -> more responsive controler 
        double curve = k *( measuredTrailerAngle - desired_hitch_angle ) - ( sin(measuredTrailerAngle)/ (l2));
        return curve/cos(measuredTrailerAngle);

    } else {
        // double l1=truckState->getAxisDistance(); // truck wheelbase in meters 
        double l2 = 0.715; // trailer wheelbase in meters (from middle point of trucks's wheel/ also happens to be the joint)

        double trailerYaw = currYaw_rad - measuredTrailerAngle ; // in radians θ = θ_vehicle - θ_e (hitch-angle)
        trailerYaw = fmod(trailerYaw + M_PI, 2 * M_PI) - M_PI;

        double delta_x_ = (l2) * cos( trailerYaw); // trailer x difference from x of the truck wheelbase
        double delta_y = (l2) * sin( trailerYaw); // trailer y difference from y of the truck wheelbase
        double trailerPositionx = vehiclePos.getX() - delta_x_;
        double trailerPositiony = vehiclePos.getY() - delta_y; 
        
        QPointF pointInTrailerFrame; // sometimes called virtual vehicle in pure pursuit
        // target point (from pure pursuit) minus current trailer point x,y 
        pointInTrailerFrame.setX(point.x()-trailerPositionx); 
        pointInTrailerFrame.setY(point.y()-trailerPositiony); 
        
        // step 1, found the target angle based on PurePursuit using trailer x,y as reference 
        double theta_err =  atan2(pointInTrailerFrame.y(), pointInTrailerFrame.x()) - trailerYaw;        
      
        // step 2, based on x,y of trailer and target point find α from desired hitch-angle
        double desired_hitch_angle = atan(2*l2*sin(theta_err) / truckState->getAutopilotRadius() ); //<-- desired trailerAngle

        // step 3, return the curvature of measured vs desired trailer hitch angle -> which will be transle to steering angle 
         double k=-1.3; // K is a try and error value, higher value -> more responsive controler 
         double curve = k *( measuredTrailerAngle - desired_hitch_angle ) - ( sin(measuredTrailerAngle)/ (l2));
        return curve/cos(measuredTrailerAngle);

    }

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
    // qDebug() << "steeringAngleProportional " << -steeringAngleProportional;
    
    return -steeringAngleProportional;
}

// TODO: utility function, move to a more central place
QVector<QPointF> PurepursuitWaypointFollower::findIntersectionsBetweenCircleAndLine(QPair<QPointF,double> circle, QLineF line) {
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
        qDebug()<< "Start logging" ;
        auto truckState = qSharedPointerDynamicCast<TruckState>(mMovementController->getVehicleState());

        truckState->initLogFile("truckposition");
        truckState->initLogFile("trailerposition");
        truckState->initLogFile("trailerAngle");
        truckState->initLogFile("distance");
        truckState->initLogFile("steering");
        truckState->initLogFile("tofback");

        truckState->initLogFile("accelerometer");
        truckState->initLogFile("gyroscope");
        // truckState->initLogFile("tofrear");
        
        truckState->logData("truckposition", "time,x,y,yaw");
        truckState->logData("trailerposition", "time,x,y,yaw");
        truckState->logData("trailerAngle", "time,degrees,radians");
        truckState->logData("distance", "time,m");
        truckState->logData("steering", "time,angle");
        truckState->logData("tofback", "time,m");

        

                // Start the timer for logging
        connect(&mLogTimer, &QTimer::timeout, [this,truckState]() {
            // logging data for different categories


            std::array<float, 3> gyrodata = truckState-> getGyroscopeXYZ() ;
            QString GyrologMessage = QString("%1,%2,%3")
                         .arg(gyrodata[0])
                         .arg(gyrodata[1])
                         .arg(gyrodata[2]);

            std::array<float, 3> accelerdata = truckState-> getAccelerometerXYZ();
            QString AccelogMessage = QString("%1,%2, %3")
                         .arg(accelerdata[0])
                         .arg(accelerdata[1])
                         .arg(accelerdata[2]);


            PosPoint truckPosition = getCurrentVehiclePosition();
            // QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
            QString PositionlogMessage = QString("%1,%2,%3,%4")
                        .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
                        .arg(truckPosition.getX())
                        .arg(truckPosition.getY())
                        .arg(truckPosition.getYaw());


            double trailerAngle = truckState->getTrailerAngleRadians();
            double currYaw_rad = getCurrentVehiclePosition().getYaw() * M_PI / 180.0;
            double trailerAxis = 0.715;
            double delta_x_ = trailerAxis * cos(currYaw_rad- trailerAngle);
            double delta_y = trailerAxis * sin(currYaw_rad - trailerAngle);
            double trailerPositionx = truckPosition.getX() - delta_x_;
            double trailerPositiony = truckPosition.getY() - delta_y;

            double trailerYaw = currYaw_rad - trailerAngle ; 
            trailerYaw = fmod(trailerYaw + M_PI, 2 * M_PI) - M_PI;

            QString TrailerPositionlogMessage = QString("%1,%2,%3,%4")
                    .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
                    .arg(trailerPositionx)
                    .arg(trailerPositiony)
                    .arg(trailerYaw);


            QString AnglelogMessage = QString("%1,%2,%3")
                         .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
                         .arg(truckState->getTrailerAngleDegrees())
                         .arg(truckState->getTrailerAngleRadians());

            QString tofbackMessage = QString("%1,%2")
                        .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
                         .arg(truckState->getTrailerDistanceToF());

            QString DistanceMessage = QString("%1,%2")
                        .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
                        .arg(truckState->GetTotalDistance());
            
            QString SteeringMessage = QString("%1,%2")
                        .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
                        .arg( truckState->getMaxSteeringAngle() * mMovementController->getDesiredSteering() );



            truckState->logData("truckposition", PositionlogMessage);
            truckState->logData("trailerposition", TrailerPositionlogMessage);
            truckState->logData("trailerAngle", AnglelogMessage);
            truckState->logData("distance", DistanceMessage);
            truckState->logData("steering", SteeringMessage);
            truckState->logData("tofback", tofbackMessage);
            // truckState->logData("tofrear", tofbackMessage);
            truckState->logData("accelerometer", AccelogMessage);
            truckState->logData("gyroscope", GyrologMessage);

            //vehicleState->startLogging(); // Replace with your logging logic
        });
        mLogTimer.start(mLogFrequency);




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
        

        if(mMovementController->getVehicleState()->getSpeed() < 0){
            auto truckState = qSharedPointerDynamicCast<TruckState>(mMovementController->getVehicleState());
            double trailerAngle = truckState->getTrailerAngleRadians();
            double currYaw_rad = getCurrentVehiclePosition().getYaw() * M_PI / 180.0;
            double trailerAxis = 0.715; // <----- TODO set as a parameter  
            double trailerYaw = currYaw_rad - trailerAngle ; // in radians θ = θ_vehicle - θ_e (hitch-angle)
            trailerYaw = fmod(trailerYaw + M_PI, 2 * M_PI) - M_PI;
            double delta_x_ = (trailerAxis) * cos( trailerYaw); // trailer x difference from x of the truck wheelbase
            double delta_y = (trailerAxis) * sin( trailerYaw); // trailer y difference from y of the truck wheelbase

            // double delta_x_ = trailerAxis * cos(currYaw_rad- trailerAngle);
            // double delta_y = trailerAxis * sin(currYaw_rad - trailerAngle);
            currentVehiclePositionXY.setX( currentVehiclePositionXY.x() - delta_x_ );
            currentVehiclePositionXY.setY( currentVehiclePositionXY.y() - delta_y );
        }

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

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED:{
        qDebug()<< "Stop logging" ;
        
        auto truckState = qSharedPointerDynamicCast<TruckState>(mMovementController->getVehicleState());
         // Stop the logging timer
        mLogTimer.stop();  // Stop the timer
        truckState->closeLogFiles();

        // Disconnect the timeout signal to avoid calling the lambda again
        disconnect(&mLogTimer, &QTimer::timeout, nullptr, nullptr);


        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        stop();
    }break;

    default:
        break;
    }
}

void PurepursuitWaypointFollower::updateControl(const PosPoint &goal)
{
    if (isOnVehicle()) {
        mMovementController->setDesiredSteeringCurvature(getCurvatureToPointInENU(goal.getPoint()));
        //  qDebug()<<"turning by: "<< mMovementController->getDesiredSteering();
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

QList<PosPoint> PurepursuitWaypointFollower::getCurrentRoute()
{
    return mWaypointList;
}
