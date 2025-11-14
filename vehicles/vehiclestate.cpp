/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "vehiclestate.h"
#include <QDebug>

VehicleState::VehicleState(ObjectID_t id, Qt::GlobalColor color)
    : ObjectState (id, color)
{
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

double VehicleState::getSteering() const
{
    return mSteering;
}

void VehicleState::setSteering(double steering)
{
    if (abs(steering) > 1.0)
        steering = steering / abs(steering);

    mSteering = steering;
}

PosPoint VehicleState::getHomePosition() const
{
    return mHomePosition;
}

void VehicleState::setHomePosition(const PosPoint &homePosition)
{
    mHomePosition = homePosition;
}

bool VehicleState::getIsArmed() const
{
    return mIsArmed;
}

void VehicleState::setIsArmed(bool isArmed)
{
    mIsArmed = isArmed;
}

PosPoint VehicleState::posInVehicleFrameToPosPointENU(xyz_t offset, PosType type) const
{
    PosPoint offsetPosition = getPosition(type);
    offsetPosition.updateWithOffsetAndYawRotation(offset, getPosition(type).getYaw() * M_PI / 180.0);
    return offsetPosition;
}

VehicleState::FlightMode VehicleState::getFlightMode() const
{
    return mFlightMode;
}

void VehicleState::setFlightMode(const FlightMode &flightMode)
{
    mFlightMode = flightMode;
}

void VehicleState::setAutopilotRadius(double radius)
{
    mAutopilotRadius = radius;
}

double VehicleState::getAutopilotRadius()
{
    return mAutopilotRadius;
}

double VehicleState::getCurvatureToPointInVehicleFrame(const QPointF &point)
{
    // calc steering angle (pure pursuit)
    double distanceSquared = pow(point.x(), 2) + pow(point.y(), 2);
    double steeringAngleProportional = (2*point.y()) / distanceSquared;

    return -steeringAngleProportional;
}

double VehicleState::getCurvatureToPointInENU(const QPointF &point, PosType type)
{
    PosPoint vehiclePosition = mPositionBySource[(int)type];

    return getCurvatureToPointInVehicleFrame(coordinateTransforms::ENUToVehicleFrame(point, vehiclePosition.getXYZ(), vehiclePosition.getYaw()));
}

QSharedPointer<VehicleState> VehicleState::getTrailingVehicle() const
{
    return mTrailingVehicle;
}

void VehicleState::setTrailingVehicle(QSharedPointer<VehicleState> trailer)
{
    mTrailingVehicle = trailer;
}

bool VehicleState::hasTrailingVehicle() const
{
    return !mTrailingVehicle.isNull();
}
