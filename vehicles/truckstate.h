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
    double getTrailerAngleRadians() const { return mTrailerAngle_deg * (M_PI / 180.0); }
    double getTrailerAngleDegrees() const { return mTrailerAngle_deg; }

    void setTrailerAngle(double angle_deg);

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
    double mTrailerAngle_deg; // Angle in Degrees
    QSharedPointer<TrailerState> mTrailerState; // trailer created dynamically
    bool mHasTrailer = false;

    double mPurePursuitForwardGain;
    double mPurePursuitReverseGain;

    double getCurvatureWithTrailer(const QPointF &point);
};

#endif // TRUCKSTATE_H
