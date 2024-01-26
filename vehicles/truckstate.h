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
    TruckState(int id = 0, Qt::GlobalColor color = Qt::red);

    // Additional state for angle sensor
    double getTrailerAngle() const { return mTrailerAngle; }
    void setTrailerAngle(double value) { mTrailerAngle = value; }

    // Override the updateOdomPositionAndYaw function
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) override;

// #ifdef QT_GUI_LIB
//     // Override or add drawing functions if needed (to draw a truck trailer)
//     virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) override;
//     virtual QPainterPath getBoundingBox() const override;
// #endif

private:
    double mTrailerAngle; // Extra state for angle sensor
};

#endif // TRUCKSTATE_H
