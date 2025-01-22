/*
 *     Copyright 2022 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Holds multiple waypointfollowers to enable switching between routes.
 */

#ifndef MULTIWAYPOINTFOLLOWER_H
#define MULTIWAYPOINTFOLLOWER_H

#include <QObject>
#include <QList>
#include <QSharedPointer>
#include "autopilot/gotowaypointfollower.h"
#include "autopilot/purepursuitwaypointfollower.h"

class MultiWaypointFollower : public WaypointFollower
{
    Q_OBJECT
public:
    MultiWaypointFollower(QSharedPointer<WaypointFollower> waypointfollower);

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

    // PurepursuitWaypointFollower parameters
    double getPurePursuitRadius() const;
    void setPurePursuitRadius(double value);
    void setAdaptivePurePursuitRadiusActive(bool adaptive);
    double getAdaptivePurePursuitRadiusCoefficient();
    void setAdaptivePurePursuitRadiusCoefficient(double coefficient);

    int addWaypointFollower(QSharedPointer<WaypointFollower> waypointfollower);
    void setActiveWaypointFollower(int waypointfollowerID);
    QSharedPointer<WaypointFollower> getActiveWaypointFollower();
    int getNumberOfWaypointFollowers();

    void provideParametersToParameterServer();
    void provideParametersToParameterServer(int waypointfollowerID);

public slots:
    void receiveDistanceOfRouteLeft(double meters);

signals:
    void distanceOfRouteLeft(double meters);

private:
    QList<QSharedPointer<WaypointFollower>> mWaypointFollowerList;

    int mActiveWaypointFollowerID = 0;
};

#endif // MULTIWAYPOINTFOLLOWER_H
