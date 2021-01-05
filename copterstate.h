#ifndef COPTERSTATE_H
#define COPTERSTATE_H

#include "vehiclestate.h"
#include <QObject>
#include <QPainter>
#include <cmath>

enum class CopterFrameType {
    X,
    PLUS
};

class CopterState : public VehicleState
{
    Q_OBJECT
public:
    CopterState(int id = 0, Qt::GlobalColor color = Qt::red);

    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true);

private:
    CopterFrameType mFrameType;
    int mPropellerSize; // [mm]
};

#endif // COPTERSTATE_H
