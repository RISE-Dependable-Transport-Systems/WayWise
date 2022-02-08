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

    CopterState(int id = 0, Qt::GlobalColor color = Qt::red);

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
