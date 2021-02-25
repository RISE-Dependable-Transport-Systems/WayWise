#ifndef COPTERSTATE_H
#define COPTERSTATE_H

#include "vehiclestate.h"
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

    LandedState getLandedState() const;
    void setLandedState(const LandedState &landedState);

    PosPoint getHomePosition() const;
    void setHomePosition(const PosPoint &homePosition);

private:
    CopterFrameType mFrameType;
    int mPropellerSize; // [mm]
    LandedState mLandedState = LandedState::Unknown;

    PosPoint mHomePosition;
};

#endif // COPTERSTATE_H
