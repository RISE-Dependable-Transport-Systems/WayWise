/*
 *     Copyright 2012 Benjamin Vedder   benjamin@vedder.se
 *               2020 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Lukas Wikander    lukas.wikander@astazero.com
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Core class for storing all state (static and dynamic) of a moveable or immovable object (e.g., vehicle, camera, ...)
 */

#ifndef OBJECTSTATE_H
#define OBJECTSTATE_H

#include <QObject>
#include <QVector>
#include <QTime>
#include <QString>
#ifdef QT_GUI_LIB
#include <QPainter>
#include <QPainterPath>
#endif

#include "core/pospoint.h"
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
    virtual QPainterPath getBoundingBox() const { return QPainterPath(); }
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
    virtual double getSpeed() const { return mSpeed; }
    virtual void setSpeed(double value) { mSpeed = value; }
    virtual Velocity getVelocity() const { return mVelocity; }
    virtual void setVelocity(const Velocity &velocity) { mVelocity = velocity; }
    virtual Acceleration getAcceleration() const { return mAcceleration; }
    virtual void setAcceleration(const Acceleration &acceleration) { mAcceleration = acceleration; }

    void setDrawStatusText(bool drawStatusText);
    bool getDrawStatusText() const;

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
    double mSpeed = 0.0; // [m/s]
    Velocity mVelocity = {0.0, 0.0, 0.0}; // [m/s]
    Acceleration mAcceleration = {0.0, 0.0, 0.0}; // [m/s²]
};


#endif // OBJECTSTATE_H
