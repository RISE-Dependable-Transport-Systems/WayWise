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

VehicleState::VehicleState(ObjectState::ObjectID_t id, Qt::GlobalColor color)
	: ObjectState (id, color)
{
	mTime = QTime();
    mLength = 0.8;
    mWidth = 0.335;

    for (int i = 0; i < (int)PosType::_LAST_; i++)
        switch((PosType) i) {
			case PosType::simulated: mPositionBySource[i].setType(PosType::simulated); break;
			case PosType::fused: mPositionBySource[i].setType(PosType::fused); break;
			case PosType::odom: mPositionBySource[i].setType(PosType::odom); break;
			case PosType::IMU: mPositionBySource[i].setType(PosType::IMU); break;
			case PosType::GNSS: mPositionBySource[i].setType(PosType::GNSS); break;
			case PosType::UWB: mPositionBySource[i].setType(PosType::UWB); break;
        case PosType::_LAST_: qDebug() << "This should not have happended."; break;

        }
}


void VehicleState::setPosition(PosPoint &point)
{
	mPositionBySource[(int)point.getType()] = point;

    emit positionUpdated();
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

PosPoint VehicleState::getPosition(PosType type) const
{
	return mPositionBySource[(int)type];
}

