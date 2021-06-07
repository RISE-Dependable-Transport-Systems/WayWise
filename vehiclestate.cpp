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

#include "vehiclestate.h"
#include <QDebug>

VehicleState::VehicleState(int id, Qt::GlobalColor color)
{
    mId = id;
    mColor = color;
    mName = "";
    mName.sprintf("Vehicle %d", mId);
    mTime = 0;
    mLength = 0.8;
    mWidth = 0.335;


    for (int i = 0; i < (int)PosType::_LAST_; i++)
        switch((PosType) i) {
            case PosType::simulated: mPosition[i].setType(PosType::simulated); break;
            case PosType::fused: mPosition[i].setType(PosType::fused); break;
            case PosType::odom: mPosition[i].setType(PosType::odom); break;
            case PosType::IMU: mPosition[i].setType(PosType::IMU); break;
            case PosType::GNSS: mPosition[i].setType(PosType::GNSS); break;
            case PosType::UWB: mPosition[i].setType(PosType::UWB); break;
        case PosType::_LAST_: qDebug() << "This should not have happended."; break;

        }
}

int VehicleState::getId() const
{
    return mId;
}

void VehicleState::setId(int id, bool changeName)
{
    mId = id;

    if (changeName) {
        mName = "";
        mName.sprintf("Vehicle %d", mId);
    }
}

QString VehicleState::getName() const
{
    return mName;
}

void VehicleState::setName(QString name)
{
    mName = name;
}

void VehicleState::setPosition(PosPoint &point)
{
    mPosition[(int)point.getType()] = point;

    emit positionUpdated();
}

Qt::GlobalColor VehicleState::getColor() const
{
    return mColor;
}

void VehicleState::setColor(Qt::GlobalColor color)
{
    mColor = color;
}

qint32 VehicleState::getTime() const
{
    return mTime;
}

void VehicleState::setTime(const qint32 &time)
{
    mTime = time;
}

double VehicleState::getSpeed() const
{
    return mSpeed;
}

void VehicleState::setSpeed(double value)
{
    mSpeed = value;
}

void VehicleState::setDrawStatusText(bool drawStatusText)
{
    mDrawStatusText = drawStatusText;
}

bool VehicleState::getDrawStatusText() const
{
    return mDrawStatusText;
}

std::array<float, 3> VehicleState::getGyroscopeXYZ() const
{
    return mGyroscopeXYZ;
}

void VehicleState::setGyroscopeXYZ(const std::array<float, 3> &gyroscopeXYZ)
{
    mGyroscopeXYZ = gyroscopeXYZ;
}

std::array<float, 3> VehicleState::getAccelerometerXYZ() const
{
    return mAccelerometerXYZ;
}

void VehicleState::setAccelerometerXYZ(const std::array<float, 3> &accelerometerXYZ)
{
    mAccelerometerXYZ = accelerometerXYZ;
}

VehicleState::Velocity VehicleState::getVelocity() const
{
    return mVelocity;
}

void VehicleState::setVelocity(const VehicleState::Velocity &velocity)
{
    mVelocity = velocity;
}

double VehicleState::getMinAcceleration() const
{
    return mMinAcceleration;
}

void VehicleState::setMinAcceleration(double minAcceleration)
{
    mMinAcceleration = minAcceleration;
}

double VehicleState::getMaxAcceleration() const
{
    return mMaxAcceleration;
}

void VehicleState::setMaxAcceleration(double maxAcceleration)
{
    mMaxAcceleration = maxAcceleration;
}

double VehicleState::getLength() const
{
    return mLength;
}

void VehicleState::setLength(double length)
{
    mLength = length;
}

double VehicleState::getWidth() const
{
    return mWidth;
}

void VehicleState::setWidth(double width)
{
    mWidth = width;
}

PosPoint VehicleState::getPosition(PosType type) const
{
    return mPosition[(int)type];
}
