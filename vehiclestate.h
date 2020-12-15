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

#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H

#include <QObject>
#include <QVector>
#include <QString>
#include <QPainter>
#include "pospoint.h"
#include <math.h>

class VehicleState : public QObject
{
    Q_OBJECT
public:
    VehicleState(int id = 0, Qt::GlobalColor color = Qt::red);

    virtual void draw(QPainter &painter, const QTransform &drawTrans, bool isSelected = true) = 0;

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
    double getMinAcceleration() const;
    void setMinAcceleration(double minAcceleration);
    double getMaxAcceleration() const;
    void setMaxAcceleration(double maxAcceleration);

    // Dynamic state
    PosPoint getPosition(PosType type = PosType::fused) const;
    void setPosition(PosPoint &point);
    qint32 getTime() const;
    void setTime(const qint32 &time);
    double getSpeed() const;
    void setSpeed(double value);

signals:
    void positionUpdated(const VehicleState& updatedVehicleState);

private:
    // Static state
    int mId;
    QString mName;
    double mLength; // [m]
    double mWidth; // [m]
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
    double mSpeed = 0.0; // [m/s]
};

#endif // VEHICLESTATE_H
