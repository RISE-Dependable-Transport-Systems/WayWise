#ifndef CARSTATE_H
#define CARSTATE_H

#include "vehiclestate.h"

#include <QObject>
#include <QString>
#ifdef QT_GUI_LIB
#include <QPainter>
#endif
#include <cmath>

class CarState : public VehicleState
{
    Q_OBJECT
public:
    CarState(int id = 0, Qt::GlobalColor color = Qt::red);
#ifdef QT_GUI_LIB
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true);
#endif
    virtual void simulationStep(double dt_ms, PosType usePosType = PosType::simulated);

    // Static state
    double getAxisDistance() const;
    void setAxisDistance(double axisDistance);
    double getRearOverhang() const;
    void setRearOverhang(double rearOverhang);
    double getFrontOverhang() const;
    void setFrontOverhang(double frontOverhang);

    inline double getMaxSteeringAngle() const { return mMaxSteeringAngle < M_PI/180.0 ? M_PI/4.0 : mMaxSteeringAngle; } // 45° assumed if unset
    void setMaxSteeringAngle(double steeringAngle_rad);
    void setMinTurnRadiusRear(double minTurnRadius_m);

    // Dynamic state
    double getSteering() const;
    void setSteering(double value);
    inline double getTurnRadiusRear() const { return getAxisDistance() / -mSteering; } // steering in [-1.0:1.0] as a simple approximation of tan(steering angle)
    inline double getTurnRadiusFront() const { return sqrt(pow(getAxisDistance(),2) + pow(getTurnRadiusRear(),2)); }
    inline double getTotalReactionTime() const { return 0.3; } // TODO: needs to be calculated/estimated
    double getBrakingDistance() const;
    double getBrakingDistance(double deceleration) const;
    double getThreeSecondsDistance() const; // Distance the vehicle can move within 3 seconds at current speed, Swedish "Tresekundersregeln"
    const QPointF getStoppingPointForTurnRadiusAndBrakingDistance(const double turnRadius, const double brakeDistance) const;
    const QPointF getStoppingPointForTurnRadius(const double turnRadius) const;
    inline double getMinTurnRadiusRear() const { return qMax(qMin(getAxisDistance() / tanf(getMaxSteeringAngle()), mMinTurnRadiusRear), pow(getSpeed(), 2)/(0.21*9.81)); }

private:
    double mRearOverhang = 0.0; //[m]
    double mFrontOverhang = 0.0; //[m]
    double mAxisDistance; // [m]
    double mSteering = 0.0; // [-1.0:1.0]
    double mMaxSteeringAngle = 0.0; // [rad]
    double mMinTurnRadiusRear = std::numeric_limits<double>::infinity(); // [m]

};

#endif // CARSTATE_H
