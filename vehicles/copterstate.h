/*
 *     Copyright 2020 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
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

    CopterState(ObjectID_t id = 1, Qt::GlobalColor color = Qt::red);

    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true);
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType) override;
    virtual double steeringCurvatureToSteering(double steeringCurvature) override;

    LandedState getLandedState() const;
    void setLandedState(const LandedState &landedState);

private:
    CopterFrameType mFrameType;
    int mPropellerSize; // [mm]
    LandedState mLandedState = LandedState::Unknown;
};

#endif // COPTERSTATE_H
