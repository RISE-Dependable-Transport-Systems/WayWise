#ifndef CARSTATE_H
#define CARSTATE_H

#include "vehiclestate.h"

#include <QObject>
#include <QString>
#include <QPainter>
#include <cmath>

class CarState : public VehicleState
{
    Q_OBJECT
public:
    CarState(int id = 0, Qt::GlobalColor color = Qt::red);

    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true);

    // Static state
    double getAxisDistance() const;
    void setAxisDistance(double axisDistance);
    inline double getMaxSteeringAngle() const { return M_PI/4.0; }; // = 45°, fixed for now

    // Dynamic state
    double getSteering() const;
    void setSteering(double value);
    inline double getTurnRadiusRear() const { return mAxisDistance / -mSteering; } // steering in [-1.0:1.0] as a simple approximation of tan(steering angle)
    inline double getTurnRadiusFront() const { return sqrt(pow(mAxisDistance,2) + pow(getTurnRadiusRear(),2)); }
    inline double getTotalReactionTime() const { return 0.8; } // TODO: placeholder, needs to be calculated
    inline double getBrakingDistance() const;
    const QPointF getStoppingPointForTurnRadiusAndBrakeDistance(const double turnRadius, const double brakeDistance) const;
    const QPointF getStoppingPointForTurnRadius(const double turnRadius) const;
    inline double getMinTurnRadiusRear() const { return qMax(mAxisDistance / tanf(getMaxSteeringAngle()), pow(getSpeed(), 2)/(0.21*9.81)); }

private:
    double mAxisDistance; // [m]
    double mSteering = 0.0; // [-1.0:1.0]
};

#endif // CARSTATE_H
