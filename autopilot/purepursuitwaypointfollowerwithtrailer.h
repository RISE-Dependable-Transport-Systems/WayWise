#ifndef PUREPURSUITWAYPOINTFOLLOWERWITHTRAILER_H
#define PUREPURSUITWAYPOINTFOLLOWERWITHTRAILER_H

#include "purepursuitwaypointfollower.h"

class PurepursuitWaypointFollowerWithTrailer : public PurepursuitWaypointFollower{
    public:
        PurepursuitWaypointFollowerWithTrailer(QSharedPointer<MovementController> movementController)
            : PurepursuitWaypointFollower(movementController) {
                
        }

        PurepursuitWaypointFollowerWithTrailer(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed)
            : PurepursuitWaypointFollower(vehicleConnection,posTypeUsed ) {

        }


        static double getCurvatureToPointInENU(QSharedPointer<VehicleState> vehicleState, const QPointF& point, PosType vehiclePosType = PosType::simulated) ;
    private:
        void updateState() ;
};

#endif // PUREPURSUITWAYPOINTFOLLOWERWITHTRAILER_H