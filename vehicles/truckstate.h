/*
 *     Copyright 2024 RISE Sweden
 *
 *   Specific implementation of VehicleState for car-type (ackermann) vehicles, storing all (dynamic and static) state
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
    TruckState(ObjectID_t id = 1, Qt::GlobalColor color = Qt::blue);

    // Additional set/get state for angle sensor
    uint16_t getTrailerAngleRaw() const { return mTrailerRawAngle; }
    double getTrailerAngleRadians() const { return mTrailerAngleRadians; }
    double getTrailerAngleDegrees() const { return mTrailerAngleDegress; }
    double getTrailerWheelBase() const { return mtrailerwheelbase; }

    void setTrailerAngle(uint16_t raw_angle , double angle_in_radians, double agnle_in_degrees);

    // Override the updateOdomPositionAndYaw function to consider the angle of the trailer
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) override;

    double getCurvatureToPointInVehicleFrame(const QPointF &point) override;

    QSharedPointer<TrailerState> getTrailerState() const;
    void setTrailerState(QSharedPointer<TrailerState> newTrailerState);

    bool getHasTrailer() const ;
    void setHasTrailer(bool mHasTrailer);

    double getPurePursuitForwardGain() const{ return mPurePursuitForwardGain;}
    void setPurePursuitForwardGain(double value){ mPurePursuitForwardGain = value;}

    double getPurePursuitReverseGain() const{ return mPurePursuitReverseGain;}
    void setPurePursuitReverseGain(double value){ mPurePursuitReverseGain = value;}

#ifdef QT_GUI_LIB
    // Override or add drawing functions if needed (to draw a truck)
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) override;
#endif

private:
    uint16_t mTrailerRawAngle; // Raw angle value from sensor
    double mTrailerAngleRadians; // Angle in Radians
    double mTrailerAngleDegress; // Angle in Degrees
    double mtrailerwheelbase; // trailer wheelbase
    QSharedPointer<TrailerState> mTrailerState; // trailer created dynamically
    bool mHasTrailer = false;

    double mPurePursuitForwardGain;
    double mPurePursuitReverseGain;

    double getCurvatureWithTrailer(const QPointF &point);
};

#endif // TRUCKSTATE_H
