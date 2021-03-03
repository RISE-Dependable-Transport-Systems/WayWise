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
	inline double getMaxSteeringAngle() const { return mMaxSteeringAngle < M_PI/180.0 ? M_PI/4.0 : mMaxSteeringAngle; } // 45° assumed if unset
	void setMaxSteeringAngle(double steeringAngle_rad);
	void setMinTurnRadiusRear(double minTurnRadius_m);

    // Dynamic state
    double getSteering() const;
    void setSteering(double value);
    inline double getTurnRadiusRear() const { return mAxisDistance / -mSteering; } // steering in [-1.0:1.0] as a simple approximation of tan(steering angle)
    inline double getTurnRadiusFront() const { return sqrt(pow(mAxisDistance,2) + pow(getTurnRadiusRear(),2)); }
    inline double getTotalReactionTime() const { return 0.8; } // TODO: placeholder, needs to be calculated
    double getBrakingDistance() const;
    double getThreeSecondsDistance() const; // Distance the vehicle can move within 3 seconds at current speed, Swedish "Tresekundersregeln"
    const QPointF getStoppingPointForTurnRadiusAndBrakingDistance(const double turnRadius, const double brakeDistance) const;
    const QPointF getStoppingPointForTurnRadius(const double turnRadius) const;
	inline double getMinTurnRadiusRear() const { return qMin(qMax(mAxisDistance / tanf(getMaxSteeringAngle()), pow(getSpeed(), 2)/(0.21*9.81)), mMinTurnRadiusRear); }

private:
    double mAxisDistance; // [m]
    double mSteering = 0.0; // [-1.0:1.0]
	double mMaxSteeringAngle = 0.0; // [rad]
	double mMinTurnRadiusRear = std::numeric_limits<double>::infinity(); // [m]
};

#endif // CARSTATE_H
