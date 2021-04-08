#ifndef DIFFDRIVEVEHICLESTATE_H
#define DIFFDRIVEVEHICLESTATE_H

#include "vehiclestate.h"

class DiffDriveVehicleState : public VehicleState
{
public:
    DiffDriveVehicleState();

    virtual void simulationStep(double dt_ms);
};

#endif // DIFFDRIVEVEHICLESTATE_H
