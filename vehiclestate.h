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
#ifdef QT_GUI_LIB
#include <QPainter>
#endif

#include "pospoint.h"
#include "objectstate.h"
#include <math.h>

class VehicleState : public ObjectState
{
    Q_OBJECT
public:
    VehicleState(ObjectID_t id = 0, Qt::GlobalColor color = Qt::red);


    // Static state
    double getLength() const { return mLength; }
    void setLength(double length) { mLength = length; }
    double getWidth() const { return mWidth; }
    void setWidth(double width) { mWidth = width; }
    double getMinAcceleration() const { return mMinAcceleration; }
    void setMinAcceleration(double minAcceleration) { mMinAcceleration = minAcceleration; }
    double getMaxAcceleration() const { return mMaxAcceleration; }
    void setMaxAcceleration(double maxAcceleration) { mMaxAcceleration = maxAcceleration; }

    // Dynamic state
    virtual PosPoint getPosition(PosType type) const;
    virtual PosPoint getPosition() const override { return getPosition(PosType::simulated); }
    virtual void setPosition(PosPoint &point) override;
    virtual QTime getTime() const override { return mTime; }
    virtual void setTime(const QTime &time) override { mTime = time; }

    void simulationStep(double dt_ms, PosType usePosType = PosType::simulated); // Take current state and simulate step forward for dt_ms milliseconds, update state accordingly
    virtual void updateOdomPositionAndYaw(double drivenDistance, PosType usePosType = PosType::odom) = 0;
    virtual double steeringCurvatureToSteering(double steeringCurvature) = 0;

    // For debugging and logging
    std::array<float, 3> getGyroscopeXYZ() const;
    void setGyroscopeXYZ(const std::array<float, 3> &gyroscopeXYZ);
    std::array<float, 3> getAccelerometerXYZ() const;
    void setAccelerometerXYZ(const std::array<float, 3> &accelerometerXYZ);

    double getSteering() const;
    virtual void setSteering(double steering);

private:
    // Static state
    double mLength; // [m]
    double mWidth; // [m]
    // TODO: reasonable default values? set here or move?
    double mMinAcceleration = -5.0; // [m/s²]
    double mMaxAcceleration = 3.0; // [m/s²]

    // Dynamic state
    double mSteering = 0.0; // [-1.0:1.0]
    PosPoint mPositionBySource[(int)PosType::_LAST_];
    PosPoint mApGoal;
    QTime mTime;

    std::array<float,3> mGyroscopeXYZ = std::array<float,3>({0.0, 0.0, 0.0}); // [deg/s]
    std::array<float,3> mAccelerometerXYZ = std::array<float,3>({0.0, 0.0, 0.0}); // [g]
};

#endif // VEHICLESTATE_H
