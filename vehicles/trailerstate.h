/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Specialisation of ObjectState for tailer state
 */

#ifndef TRAILERSTATE_H
#define TRAILERSTATE_H


#include <QObject>
#include "vehicles/vehiclestate.h"

#ifdef QT_GUI_LIB
#include <QPainter>
#endif

class TrailerState : public VehicleState
{
    Q_OBJECT
public:

    TrailerState(ObjectID_t id = 25, Qt::GlobalColor color = Qt::white);
    virtual void provideParameters() override;

    double getLength() const { return mLength; }
    void setLength(double length) { mLength = length; }
    double getWidth() const { return mWidth; }
    void setWidth(double width) { mWidth = width; }
    double getWheelBase() const{ return mWheelBase;}
    void setWheelBase(double value){ mWheelBase = value;}

    // VehicleState interface
    void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType);
    // TODO
    double steeringCurvatureToSteering(double steeringCurvature) { Q_UNUSED(steeringCurvature); return 0;};

#ifdef QT_GUI_LIB
    // drawing functions for trailer (to draw a trailer)
    void drawTrailer(QPainter &painter,const QTransform &drawTrans, const PosPoint &carPos, double angle); // this would called from truck
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) override;// if trailer is stand alone (not attached to truck)
#endif
    double mAngle;

private:
    double mLength; // [m]
    double mWidth; // [m]
    double mWheelBase; //[m]
};

#endif // TRAILERSTATE_H
