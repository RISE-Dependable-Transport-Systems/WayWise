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
    mName.sprintf("Car %d", mId);
    mTime = 0;
    mLength = 0.8;
    mWidth = 0.335;

    mPosition.setType(PosType::fused);
    mPositionGNSS.setType(PosType::GNSS);
    mPositionUWB.setType(PosType::UWB);
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
        mName.sprintf("Car %d", mId);
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
    switch (point.getType()) {
        case PosType::fused:    mPosition = point; break;
        case PosType::GNSS:     mPositionGNSS = point; break;
        case PosType::UWB:      mPositionUWB = point; break;
    }
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
    switch (type) {
        case PosType::fused:    return mPosition;
        case PosType::GNSS:     return mPositionGNSS;
        case PosType::UWB:      return mPositionUWB;
    }

    return mPosition;
}
