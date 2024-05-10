
#include <QLineF>
#include "purepursuitwaypointfollowerwithtrailer.h"
#include "WayWise/vehicles/truckstate.h"



double PurepursuitWaypointFollowerWithTrailer::getCurvatureToPointInENU(QSharedPointer<VehicleState> vehicleState, const QPointF &point, PosType vehiclePosType)
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
        return  k*( measuredTrailerAngle - desired_hitch_angle ) - ( sin(measuredTrailerAngle)/ (l2)) ;

    } else {
        // double l1=truckState->getAxisDistance(); // truck wheelbase in meters 
        double l2 = 0.715; // trailer wheelbase in meters (from middle point of trucks's wheel/ also happens to be the joint)

        double trailerYaw = currYaw_rad - measuredTrailerAngle ; // in radians θ = θ_vehicle - θ_e (hitch-angle)

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
         double k=-2.5; // K is a try and error value, higher value -> more responsive controler 
        return  k*( measuredTrailerAngle - desired_hitch_angle ) - ( sin(measuredTrailerAngle)/ (l2)) ;

    }

}


void PurepursuitWaypointFollowerWithTrailer::updateState()
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
        
        if(mMovementController->getVehicleState()->getSpeed() < 0){
            auto truckState = qSharedPointerDynamicCast<TruckState>(mMovementController->getVehicleState());
            double trailerAngle = truckState->getTrailerAngleRadians();
            double currYaw_rad = getCurrentVehiclePosition().getYaw() * M_PI / 180.0;
            double trailerAxis = 0.715; // <----- TODO set as a parameter  
            double delta_x_ = trailerAxis * cos(currYaw_rad- trailerAngle);
            double delta_y = trailerAxis * sin(currYaw_rad - trailerAngle);
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

    case WayPointFollowerSTMstates::FOLLOW_ROUTE_FINISHED:
        mCurrentState.stmState = WayPointFollowerSTMstates::NONE;
        mCurrentState.currentWaypointIndex = mWaypointList.size();
        stop();
        break;

    default:
        break;
    }
}



