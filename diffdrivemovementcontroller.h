#ifndef DIFFDRIVEMOVEMENTCONTROLLER_H
#define DIFFDRIVEMOVEMENTCONTROLLER_H

#include <QObject>
#include "movementcontroller.h"
#include "diffdrivevehiclestate.h"
#include <QSharedPointer>

class DiffDriveMovementController : public MovementController
{
    Q_OBJECT
public:
    DiffDriveMovementController(QSharedPointer<DiffDriveVehicleState> vehicleState);

private:

    // MovementController interface
public:
    virtual void setDesiredSteering(double desiredSteering) override;
    virtual void setDesiredSpeed(double desiredSpeed) override;
};

#endif // DIFFDRIVEMOVEMENTCONTROLLER_H
