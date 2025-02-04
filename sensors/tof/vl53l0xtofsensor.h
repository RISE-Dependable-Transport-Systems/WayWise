/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Implementation of tof sensor specific for VL53L0X
 */

#ifndef VL53L0XTOFSENSOR_H
#define VL53L0XTOFSENSOR_H

#include "sensors/tof/tofsensor.h"
#include <QTimer>

// connects to VL53L0X via i2c bus and polls distance values periodically
class VL53L0XToFSensor : public ToFSensor
{
public:
    VL53L0XToFSensor();
    virtual bool setUpdateIntervall(int pollIntervall_ms) override;

private:
    int mPollIntervall_ms = 500;
    QTimer mPollTimer;
};

#endif // VL53L0XTOFSENSOR_H
