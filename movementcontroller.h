#ifndef MOVEMENTCONTROLLER_H
#define MOVEMENTCONTROLLER_H

#include <QObject>
#include "vehiclestate.h"

// Gets desired steering, throttle/speed as input and translates it for specific vehicle respecting kinematic model and, e.g., motor controller as well as servo setup
// Reports actual steering, throttle/speed reported from sensors/lower-level controllers back to VehicleState
class MovementController : public QObject
{
    Q_OBJECT
public:
    virtual double getDesiredSteering() const;
    virtual void setDesiredSteering(double desiredSteering);

    virtual double getDesiredSpeed() const;
    virtual void setDesiredSpeed(double desiredSpeed);

signals:

private:
    double mDesiredSteering = 0.0; // [-1.0:1.0]
    double mDesiredSpeed = 0.0; // [m/s]

};

#endif // MOVEMENTCONTROLLER_H
