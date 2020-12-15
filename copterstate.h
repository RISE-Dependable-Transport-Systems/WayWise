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
    CopterState(int id = 0, Qt::GlobalColor color = Qt::red);

    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true);
};

#endif // COPTERSTATE_H
