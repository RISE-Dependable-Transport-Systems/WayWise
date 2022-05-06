/*
 *     Copyright 2012 Benjamin Vedder   benjamin@vedder.se
 *               2020 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Specific implementation of VehicleState for multicopter drones/UAVs, storing all (dynamic and static) state
 */

#ifndef COPTERSTATE_H
#define COPTERSTATE_H

#include "vehicles/vehiclestate.h"
#include <QObject>
#include <QPainter>
#include <cmath>

class CopterState : public VehicleState
{
    Q_OBJECT
public:
    enum class CopterFrameType {
        X,
        PLUS
    };

    // like MAVSDK LandedState
    enum class LandedState {
        Unknown,
        OnGround,
        InAir,
        TakingOff,
        Landing
    };

    // like MAVSDK FlightMode
    enum class FlightMode {
        Unknown,
        Ready,
        Takeoff,
        Hold,
        Mission,
        ReturnToLaunch,
        Land,
        Offboard,
        FollowMe,
        Manual,
        Altctl,
        Posctl,
        Acro,
        Stabilized,
        Rattitude
    };

    CopterState(int id = 0, Qt::GlobalColor color = Qt::red);

    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true);
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType) override;
    virtual double steeringCurvatureToSteering(double steeringCurvature) override;

    LandedState getLandedState() const;
    void setLandedState(const LandedState &landedState);

    FlightMode getFlightMode() const;
    void setFlightMode(const FlightMode &flightMode);

private:
    CopterFrameType mFrameType;
    int mPropellerSize; // [mm]
    LandedState mLandedState = LandedState::Unknown;
    FlightMode mFlightMode = FlightMode::Unknown;
};

#endif // COPTERSTATE_H
