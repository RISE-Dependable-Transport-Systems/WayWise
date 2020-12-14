/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se
              2020 Marvin Damschen  marvin.damschen@ri.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef CARSTATE_H
#define CARSTATE_H

#include <QObject>
#include <QVector>
#include <QString>
#include <QPainter>
#include "pospoint.h"
#include <math.h>

class CarState : public QObject
{
    Q_OBJECT
public:
    CarState(int id = 0, Qt::GlobalColor color = Qt::red);

    void draw(QPainter &painter, const QTransform &drawTrans, bool isSelected = true);

    // Static state
    int getId() const;
    void setId(int id, bool changeName = false);
    QString getName() const;
    void setName(QString name);
    Qt::GlobalColor getColor() const;
    void setColor(Qt::GlobalColor color);
    double getLength() const;
    void setLength(double length);
    double getWidth() const;
    void setWidth(double width);
    double getAxisDistance() const;
    void setAxisDistance(double axisDistance);
    double getMinAcceleration() const;
    void setMinAcceleration(double minAcceleration);
    double getMaxAcceleration() const;
    void setMaxAcceleration(double maxAcceleration);
    inline double getMaxSteeringAngle() const { return M_PI/4.0; }; // = 45°, fixed for now

    // Dynamic state
    PosPoint getPosition(PosType type = PosType::fused) const;
    void setPosition(PosPoint &point);
    PosPoint &getApGoal();
    void setApGoal(const PosPoint &apGoal);
    qint32 getTime() const;
    void setTime(const qint32 &time);
    double getSteering() const;
    void setSteering(double value);
    double getSpeed() const;
    void setSpeed(double value);
    inline double getTurnRadiusRear() const { return mAxisDistance / -mSteering; } // steering in [-1.0:1.0] as a simple approximation of tan(steering angle)
    inline double getTurnRadiusFront() const { return sqrt(pow(mAxisDistance,2) + pow(getTurnRadiusRear(),2)); }
    inline double getTotalReactionTime() const { return 0.8; } // TODO: placeholder, needs to be calculated
    inline double getBrakingDistance() const { return getSpeed() * getTotalReactionTime() - 0.5 * pow(getSpeed() + getMaxAcceleration() * getTotalReactionTime(), 2.0) / getMinAcceleration(); }
    const QPointF getStoppingPointForTurnRadiusAndBrakeDistance(const double turnRadius, const double brakeDistance) const;
    const QPointF getStoppingPointForTurnRadius(const double turnRadius) const;
    inline double getMinTurnRadiusRear() const { return qMax(mAxisDistance / tanf(getMaxSteeringAngle()), pow(getSpeed(), 2)/(0.21*9.81)); }

signals:
    void carPositionUpdated(const CarState& updatedCarState);

private:
    // Static state
    int mId;
    QString mName;
    double mLength; // [m]
    double mWidth; // [m]
    double mAxisDistance; // [m]
    // TODO: reasonable default values? set here or move?
    double mMinAcceleration = -5.0; // [m/s²]
    double mMaxAcceleration = 3.0; // [m/s²]
    Qt::GlobalColor mColor;

    // Dynamic state
    PosPoint mPosition;
    PosPoint mPositionGNSS;
    PosPoint mPositionUWB;
    PosPoint mApGoal;
    qint32 mTime;
    double mSteering = 0.0; // [-1.0:1.0]
    double mSpeed = 0.0; // [m/s]
};

#endif // CARSTATE_H
