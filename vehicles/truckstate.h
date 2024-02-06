/*
 *     Copyright 2024 RISE Sweden
 * Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
 */

#ifndef TRUCKSTATE_H
#define TRUCKSTATE_H

#include "carstate.h" // Include the base class


class TruckState : public CarState
{
    Q_OBJECT
public:
    TruckState(int id = 0, Qt::GlobalColor color = Qt::blue);

    // Additional set/get state for angle sensor
    uint16_t getTrailerAngleRaw() const { return mTrailerRawAngle; }
    double getTrailerAngleRadians() const { return mTrailerAngleRadians; }
    void setTrailerAngle(uint16_t raw_angle , double angle_in_radians) { 
        mTrailerRawAngle = raw_angle;
        mTrailerAngleRadians= angle_in_radians;
    }

    // Override the updateOdomPositionAndYaw function
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) override;

#ifdef QT_GUI_LIB
    // Override or add drawing functions if needed (to draw a truck trailer)
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) override;
#endif

private:
    uint16_t mTrailerRawAngle; // Raw angle value from sensor
    double mTrailerAngleRadians; // Agnle in Radians

};

#endif // TRUCKSTATE_H
