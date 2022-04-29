/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract class to provide interface towards, e.g., autopilot, and to be implemented for specific vehicle types.
 * Gets desired steering, throttle/speed as input and translates it for specific vehicle respecting kinematic model and, e.g., motor controller as well as servo setup.
 * Reports actual steering, throttle/speed reported from sensors/lower-level controllers back to VehicleState.
 */

#ifndef MOVEMENTCONTROLLER_H
#define MOVEMENTCONTROLLER_H

#include <QObject>
#include <QSharedPointer>
#include "vehicles/vehiclestate.h"

class MovementController : public QObject
{
    Q_OBJECT
public:
    MovementController(QSharedPointer<VehicleState> vehicleState);
    virtual double getDesiredSteering() const;
    virtual void setDesiredSteering(double desiredSteering);
    virtual void setDesiredSteeringCurvature(double desiredSteeringCurvature);

    virtual double getDesiredSpeed() const;
    virtual void setDesiredSpeed(double desiredSpeed);

    virtual void setDesiredAttributes(quint32 desiredAttributes);

    QSharedPointer<VehicleState> getVehicleState() const;

signals:
    void updatedOdomPositionAndYaw(QSharedPointer<VehicleState> vehicleState, double distanceMoved);

private:
    QSharedPointer<VehicleState> mVehicleState;
    double mDesiredSteering = 0.0; // [-1.0:1.0]
    double mDesiredSpeed = 0.0; // [m/s]
    quint32 mDesiredAttributes = 0;
};

#endif // MOVEMENTCONTROLLER_H
