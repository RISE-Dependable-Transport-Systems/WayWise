/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of pure pursuit for following a list of waypoints ("Follow Route").
 */

#ifndef PUREPURSUITWAYPOINTFOLLOWER_H
#define PUREPURSUITWAYPOINTFOLLOWER_H

#include <QObject>
#include <QSharedPointer>
#include <QPointF>
#include <QTimer>
#include "vehicles/vehiclestate.h"
#include "vehicles/controller/movementcontroller.h"
#include "communication/vehicleconnections/vehicleconnection.h"
#include "autopilot/waypointfollower.h"
#include "routeplanning/routeutils.h"

enum class WayPointFollowerSTMstates {NONE, FOLLOW_ROUTE_INIT, FOLLOW_ROUTE_GOTO_BEGIN, FOLLOW_ROUTE_FOLLOWING, FOLLOW_ROUTE_APPROACHING_END_GOAL, FOLLOW_ROUTE_FINISHED};
struct WayPointFollowerState {
    WayPointFollowerSTMstates stmState = WayPointFollowerSTMstates::NONE;
    PosPoint currentGoal;
    QPointF startPointXY;
    int currentWaypointIndex;
    bool adaptivePurePursuitRadius = false;
    double purePursuitRadius = 1.0;
    double adaptivePurePursuitRadiusCoefficient = 1.0;
    // Follow Route
    int numWaypointsLookahead = 8;
    bool repeatRoute = false;
    // -- for flying vehicles
    double overrideAltitude = 0.0;
};

struct SpeedLimitRegion {
    QList<PosPoint> boundary;  // 2D-Polygon vertices (altitude not considered). If first point != last point, the polygon will automatically be closed by connecting the last vertex back to the first vertex
    double maxSpeed;           // Speed limit [m/s]
};

class PurepursuitWaypointFollower : public WaypointFollower
{
    Q_OBJECT
public:
    // WaypointFollower either works locally on a MovementController on the vehicle, or remotely on a VehicleConnection
    PurepursuitWaypointFollower(QSharedPointer<MovementController> movementController);
    PurepursuitWaypointFollower(QSharedPointer<VehicleConnection> vehicleConnection, PosType posTypeUsed);

    inline bool isOnVehicle() {return !mMovementController.isNull();}

    double getPurePursuitRadius() const;
    void setPurePursuitRadius(double value);
    void setAdaptivePurePursuitRadiusActive(bool adaptive);
    double getAdaptivePurePursuitRadiusCoefficient();
    void setAdaptivePurePursuitRadiusCoefficient(double coefficient);

    bool getRetryAfterEndGoalOvershot() const {return mRetryAfterEndGoalOvershot;};
    void setRetryAfterEndGoalOvershot(bool value) { mRetryAfterEndGoalOvershot = value; };
    double getEndGoalAlignmentThreshold() const {return mEndGoalAlignmentThreshold;};
    void setEndGoalAlignmentThreshold(double value) { mEndGoalAlignmentThreshold = value; };

    virtual bool getRepeatRoute() const override;
    virtual void setRepeatRoute(bool value) override;

    virtual const PosPoint getCurrentGoal() override;

    virtual void clearRoute() override;
    virtual void addWaypoint(const PosPoint &point) override;
    virtual void addRoute(const QList<PosPoint>& route) override;

    virtual void startFollowingRoute(bool fromBeginning) override;
    virtual bool isActive() override;
    virtual void stop() override;
    virtual void resetState() override;

    virtual QList<PosPoint> getCurrentRoute() override;

    double getInterpolatedSpeed(const PosPoint &currentGoal, const PosPoint &lastWaypoint, const PosPoint &nextWaypoint);

    PosType getPosTypeUsed() const;
    void setPosTypeUsed(const PosType &posTypeUsed);

    void provideParametersToParameterServer();
    PosPoint getVehicleAlignmentReferencePosPoint();
    QSharedPointer<VehicleState> getReferenceVehicleState();

    WayPointFollowerState getCurrentState() const {return mCurrentState;}

    void setAdaptiveApproachSpeedEnabled(bool adaptive) {mAdaptiveApproachSpeedEnabled = adaptive;}
    void setMinApproachSpeed(double minApproachSpeed) {mMinApproachSpeed = minApproachSpeed;}

    void loadSpeedLimitRegionsFile(const QString& filePath);
    void parseSpeedLimitRegionsDocument(const QJsonDocument& jsonDoc);
    void addSpeedLimitRegion(const QList<PosPoint>& boundary, double maxSpeed_kmph);
    void clearSpeedLimitRegions();
    QList<SpeedLimitRegion> getSpeedLimitRegions() const;

signals:
    void distanceOfRouteLeft(double meters);

private:
    void updateState();
    void updateControl(const PosPoint& goal);
    int findClosestWaypointIndex(const QList<PosPoint>& waypoints, const QPointF &currentVehiclePositionXY);
    QPointF getVehicleReferencePosition(const QList<PosPoint>& waypoints);

    WayPointFollowerState mCurrentState;
    QList<SpeedLimitRegion> mSpeedLimitRegions;
    PosType mPosTypeUsed = PosType::fused; // The type of position (Odom, GNSS, UWB, ...) that should be used for planning
    QSharedPointer<MovementController> mMovementController;
    QSharedPointer<VehicleConnection> mVehicleConnection;
    QSharedPointer<VehicleState> mVehicleState;
    QList <PosPoint> mWaypointList;
    unsigned mUpdateStatePeriod_ms = 50;
    QTimer mUpdateStateTimer;

    void holdPosition();
    void calculateDistanceOfRouteLeft(QPointF currentVehiclePositionXY);
    double purePursuitRadius();

    bool mRetryAfterEndGoalOvershot = false;
    double mEndGoalAlignmentThreshold = 0.1; //[m]

    bool mAdaptiveApproachSpeedEnabled = false;
    double mMinApproachSpeed = 0.0;
};

#endif // PUREPURSUITWAYPOINTFOLLOWER_H
