/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 * 
 * Implementation of tof sensor specific for VL53L0X
 */

#ifndef VL53L0XUPDATER_H
#define VL53L0XUPDATER_H

#include "sensors/tof/tofsensorupdater.h"
#include "vehicles/truckstate.h"

#include <QTimer>

// connects to AS5600 via i2c bus and polls orientation data (angle) periodically
class VL53L0XUpdater : public ToFSensorUpdater
{
public:
    VL53L0XUpdater(QSharedPointer<VehicleState> vehicleState);
    //void printSensorInfo();
    virtual bool setUpdateIntervall(int pollIntervall_ms) override;

private:
    int mPollIntervall_ms = 900; // interval (ms) to read from AS5600,
    QTimer mPollTimer;
};

#endif // VL53L0XUPDATER_H
