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
    bool isClimbingToStartWaypointHeight() const {return mClimbingToStartWaypointHeight;}

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

    double getDescentSpeed() const {return mDescentSpeed;}
    void setDescentSpeed(double value) {mDescentSpeed = value;}

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

    double getVerticalHeightTolerance() const {return mVerticalHeightTolerance;}
    void setVerticalHeightTolerance(double value) {mVerticalHeightTolerance = value;}

    double getVerticalProportionalGain() const {return mVerticalProportionalGain;}
    void setVerticalProportionalGain(double value) {mVerticalProportionalGain = value;}

    double getVerticalIntegralGain() const {return mVerticalIntegralGain;}
    void setVerticalIntegralGain(double value) {mVerticalIntegralGain = value;}

    double getVerticalDerivativeGain() const {return mVerticalDerivativeGain;}
    void setVerticalDerivativeGain(double value) {mVerticalDerivativeGain = value;}

    double getVerticalIntegralLimit() const {return mVerticalIntegralLimit;}
    void setVerticalIntegralLimit(double value) {mVerticalIntegralLimit = value;}

private:
    void updateState();
    void updateControl(const PosPoint &goal);
    void updateTrackingControl(const PosPoint &goal);
    void updateClimbControl(const PosPoint &goal);
    void holdPosition();
    bool isVerticalLeg(int waypointIndex) const;
    PosPoint waypointHeightAtCurrentPosition(int waypointIndex) const;
    double verticalLegReferenceHeight(int waypointIndex) const;
    PosPoint getCurrentVehiclePosition() const;
    int findClosestSegmentStartIndex() const;
    double descentSpeedForGoal(const PosPoint &goal) const;
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
    bool mClimbingToStartWaypointHeight = false;
    bool mSkipStartWaypointAfterClimb = false;
    double mVerticalHeightErrorIntegral = 0.0;
    double mPrevDistanceToGoal = std::numeric_limits<double>::max(); // overshoot detection
    double mWaypointProximity = 0.5;            // [m]
    double mEndGoalAlignmentThreshold = 0.25;   // [m]
    double mCruiseSpeed = 1.0;                  // [m/s]
    double mMaxSpeed = 2.0;                     // [m/s]
    double mDescentSpeed = 0.3;                 // [m/s]
    double mMinApproachSpeed = 0.1;             // [m/s]
    double mApproachSlowdownRadius = 1.5;       // [m]
    bool mFaceTravelDirection = true;
    double mYawGain = 1.5;                      // [1/s]
    double mMaxYawRate = 1.0;                   // [rad/s]
    double mVerticalHeightTolerance = 0.10;      // [m]
    double mVerticalProportionalGain = 0.8;      // [1/s]
    double mVerticalIntegralGain = 0.04;         // [1/s^2]
    double mVerticalDerivativeGain = 0.5;        // dimensionless damping on vertical speed
    double mVerticalIntegralLimit = 2.0;         // [m*s]
};

#endif // COPTERWAYPOINTFOLLOWER_H
