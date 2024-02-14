/*
 *     Copyright 2024 RISE Sweden
 * Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
 */

#ifndef TRUCKSTATE_H
#define TRUCKSTATE_H

#include "carstate.h"
#include "trailerstate.h"
#include <QSharedPointer>

class TruckState : public CarState
{
    Q_OBJECT
public:
    TruckState(int id = 0, Qt::GlobalColor color = Qt::blue);

    // Additional set/get state for angle sensor
    uint16_t getTrailerAngleRaw() const { return mTrailerRawAngle; }
    double getTrailerAngleRadians() const { return mTrailerAngleRadians; }
    double getTrailerAngleDegrees() const { return mTrailerAngleDegress; }
    void setTrailerAngle(uint16_t raw_angle , double angle_in_radians, double agnle_in_degrees) { 
        mTrailerRawAngle = raw_angle;
        mTrailerAngleRadians = angle_in_radians;
        mTrailerAngleDegress = agnle_in_degrees;
    }

    // Override the updateOdomPositionAndYaw function
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) override;


    QSharedPointer<TrailerState> getTrailerState() const { return mTrailerState; }
    QSharedPointer<TrailerState> setTrailerState(QSharedPointer<TrailerState> newTrailerState) { mTrailerState=newTrailerState; }


#ifdef QT_GUI_LIB
    // Override or add drawing functions if needed (to draw a truck)
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) override;
#endif

private:
    uint16_t mTrailerRawAngle; // Raw angle value from sensor
    double mTrailerAngleRadians; // Angle in Radians
    double mTrailerAngleDegress; // Angle in Degrees
    QSharedPointer<TrailerState> mTrailerState; // we assume the trailer happens dynamically, 
    //trailer can change during run time , also the trailer can exist if truck dies :)
};

#endif // TRUCKSTATE_H
