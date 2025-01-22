/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Specialisation of VehicleState for tailer state
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
    virtual void provideParametersToParameterServer() override;

    double getWheelBase() const{ return mWheelBase;}
    void setWheelBase(double value){ mWheelBase = value;}
    virtual void setLength(double length) override;

    // VehicleState interface
    void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType);
    // TODO
    double steeringCurvatureToSteering(double steeringCurvature) { Q_UNUSED(steeringCurvature); return 0;};

#ifdef QT_GUI_LIB
    // drawing functions for trailer (to draw a trailer)
    void drawTrailer(QPainter &painter,const QTransform &drawTrans); // this would called from truck
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) override;// if trailer is stand alone (not attached to truck)

    void setStateInitialized(bool value){ mStateInitialized = value;}
    bool isStateInitialized(){ return mStateInitialized;}
#endif
    double mAngle;

private:
    double mWheelBase;

#ifdef QT_GUI_LIB
    bool mStateInitialized = false; // whether parameters are setup
#endif
};

#endif // TRAILERSTATE_H
