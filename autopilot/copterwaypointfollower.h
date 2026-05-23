/*
 *     Copyright 2026 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Waypoint follower for multicopters. It follows ENU waypoints and produces body-frame
 * velocity plus yaw-rate commands suitable for a velocity offboard bridge.
 */

#ifndef COPTERWAYPOINTFOLLOWER_H
#define COPTERWAYPOINTFOLLOWER_H

#include <QSharedPointer>
#include <QTimer>

#include "autopilot/purepursuitwaypointfollower.h"
#include "vehicles/controller/movementcontroller.h"

struct CopterVelocityCommand {
    double forward = 0.0;  // body x, [m/s]
    double left = 0.0;     // body y, [m/s]
    double up = 0.0;       // body z, [m/s]
    double yawRate = 0.0;  // ENU yaw rate, [rad/s]
};

class CopterWaypointFollower : public WaypointFollower
{
    Q_OBJECT
public:
    explicit CopterWaypointFollower(QSharedPointer<MovementController> movementController,
                                    PosType posTypeUsed = PosType::odom);

    virtual bool getRepeatRoute() const override;
    virtual void setRepeatRoute(bool value) override;

    virtual const PosPoint getCurrentGoal() override;

    virtual void clearRoute() override;
    virtual void addWaypoint(const PosPoint &point) override;
    virtual void addRoute(const QList<PosPoint>& route) override;
    virtual QList<PosPoint> getCurrentRoute() override;

    virtual void startFollowingRoute(bool fromBeginning) override;
    virtual bool isActive() override;
    virtual void stop() override;
    virtual void resetState() override;

    WayPointFollowerState getCurrentState() const {return mCurrentState;}
    CopterVelocityCommand getDesiredVelocityCommand() const {return mDesiredVelocityCommand;}

    PosType getPosTypeUsed() const;
    void setPosTypeUsed(const PosType &posTypeUsed);

    double getWaypointProximity() const {return mWaypointProximity;}
    void setWaypointProximity(double value) {mWaypointProximity = value;}

    double getEndGoalAlignmentThreshold() const {return mEndGoalAlignmentThreshold;}
    void setEndGoalAlignmentThreshold(double value) {mEndGoalAlignmentThreshold = value;}

    double getCruiseSpeed() const {return mCruiseSpeed;}
    void setCruiseSpeed(double value) {mCruiseSpeed = value;}

    double getMaxSpeed() const {return mMaxSpeed;}
    void setMaxSpeed(double value) {mMaxSpeed = value;}

    double getMinApproachSpeed() const {return mMinApproachSpeed;}
    void setMinApproachSpeed(double value) {mMinApproachSpeed = value;}

    double getApproachSlowdownRadius() const {return mApproachSlowdownRadius;}
    void setApproachSlowdownRadius(double value) {mApproachSlowdownRadius = value;}

    bool getFaceTravelDirection() const {return mFaceTravelDirection;}
    void setFaceTravelDirection(bool value) {mFaceTravelDirection = value;}

    double getYawGain() const {return mYawGain;}
    void setYawGain(double value) {mYawGain = value;}

    double getMaxYawRate() const {return mMaxYawRate;}
    void setMaxYawRate(double value) {mMaxYawRate = value;}

private:
    void updateState();
    void updateControl(const PosPoint &goal);
    void holdPosition();
    PosPoint getCurrentVehiclePosition() const;
    int findClosestWaypointIndex() const;
    double speedForGoal(const PosPoint &goal, double distanceToGoal) const;
    static double normalizeAngleRad(double angle);

    WayPointFollowerState mCurrentState;
    PosType mPosTypeUsed = PosType::odom;
    QSharedPointer<MovementController> mMovementController;
    QSharedPointer<VehicleState> mVehicleState;
    QList<PosPoint> mWaypointList;
    QTimer mUpdateStateTimer;
    unsigned mUpdateStatePeriod_ms = 50;

    CopterVelocityCommand mDesiredVelocityCommand;
    double mPrevDistanceToGoal = std::numeric_limits<double>::max(); // overshoot detection
    double mWaypointProximity = 0.5;            // [m]
    double mEndGoalAlignmentThreshold = 0.25;   // [m]
    double mCruiseSpeed = 1.0;                  // [m/s]
    double mMaxSpeed = 2.0;                     // [m/s]
    double mMinApproachSpeed = 0.1;             // [m/s]
    double mApproachSlowdownRadius = 1.5;       // [m]
    bool mFaceTravelDirection = true;
    double mYawGain = 1.5;                      // [1/s]
    double mMaxYawRate = 1.0;                   // [rad/s]
};

#endif // COPTERWAYPOINTFOLLOWER_H
