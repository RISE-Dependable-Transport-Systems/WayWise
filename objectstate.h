#ifndef OBJECTSTATE_H
#define OBJECTSTATE_H
/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se
              2020 Marvin Damschen  marvin.damschen@ri.se
              2021 Lukas Wikander	lukas.wikander@astazero.com

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


#include <QObject>
#include <QVector>
#include <QTime>
#include <QString>
#ifdef QT_GUI_LIB
#include <QPainter>
#endif

#include "pospoint.h"
#include <math.h>

class ObjectState : public QObject
{
    Q_OBJECT
public:
    typedef int ObjectID_t;
    typedef struct {double x, y, z;} Velocity;
    typedef struct {double x, y, z;} Acceleration;
    ObjectState(ObjectID_t id = 0, Qt::GlobalColor color = Qt::red);
#ifdef QT_GUI_LIB
    virtual void draw(QPainter &painter, const QTransform &drawTrans, const QTransform &txtTrans, bool isSelected = true) = 0;
#endif
    // Static state
    ObjectID_t getId() const { return mId; }
    void setId(ObjectID_t id, bool changeName = false);
    QString getName() const { return mName; }
    void setName(const QString& name) { mName = name; }
    Qt::GlobalColor getColor() const { return mColor; }
    void setColor(const Qt::GlobalColor color) { mColor = color; }

    // Dynamic state
    virtual PosPoint getPosition() const { return mPosition; }
    virtual void setPosition(PosPoint &point);
    virtual QTime getTime() const { return mPosition.getTime(); }
    virtual void setTime(const QTime &time) { mPosition.setTime(time); }
    virtual double getSpeed() const;
    virtual void setSpeed(double value);
    virtual Velocity getVelocity() const { return mVelocity; }
    virtual void setVelocity(const Velocity &velocity) { mVelocity = velocity; }
    virtual Acceleration getAcceleration() const { return mAcceleration; }
    virtual void setAcceleration(const Acceleration &acceleration) { mAcceleration = acceleration; }

    void setDrawStatusText(bool drawStatusText);
    bool getDrawStatusText() const;

    virtual void simulationStep(double dt_ms, PosType usePosType = PosType::simulated) = 0; // Take current state and simulate step forward for dt_ms milliseconds, update state accordingly

signals:
    void positionUpdated();

private:
    // Static state
    ObjectID_t mId;
    QString mName;
    Qt::GlobalColor mColor;
    bool mDrawStatusText = true;

protected:
    // Dynamic state
    PosPoint mPosition;
    Velocity mVelocity = {0.0, 0.0, 0.0}; // [m/s]
    Acceleration mAcceleration = {0.0, 0.0, 0.0}; // [m/s²]
};


#endif // OBJECTSTATE_H
