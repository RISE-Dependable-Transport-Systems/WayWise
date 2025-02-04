/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of MovementController for car-type (ackermann) vehicles.
 * Translates desired driving, e.g., from an autopilot to servo- and motorcontroller calls and updates CarState with feedback.
 */

#ifndef CARMOVEMENTCONTROLLER_H
#define CARMOVEMENTCONTROLLER_H

#include "vehicles/controller/movementcontroller.h"
#include "motorcontroller.h"
#include "servocontroller.h"
#include "vehicles/carstate.h"
#include <QObject>
#include <QSharedPointer>

class CarMovementController : public MovementController
{
    Q_OBJECT
public:
    CarMovementController(QSharedPointer<CarState> vehicleState);
    // MovementController interface
    virtual void setDesiredSteering(double desiredSteering) override;
    virtual void setDesiredSpeed(double desiredSpeed) override;

    void setMotorController(const QSharedPointer<MotorController> motorController);
    void setServoController(const QSharedPointer<ServoController> servoController);

    double getSpeedToRPMFactor() const;
    void setSpeedToRPMFactor(double getSpeedToRPMFactor);


private:
    void updateVehicleState(double rpm, int tachometer, int tachometer_abs, double voltageInput, double temperature, int errorID);

    QSharedPointer<CarState> mCarState;
    QSharedPointer<MotorController> mMotorController;
    QSharedPointer<ServoController> mServoController;
    double mSpeedToRPMFactor = 4123.3; // default for Traxxas Slash VXL
};

#endif // CARMOVEMENTCONTROLLER_H
