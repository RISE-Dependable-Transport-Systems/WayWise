/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "vl53l0xtofsensor.h"
#include <cmath>
#include <QDebug>

extern "C" {
#include "external/VL53L0X/tof.h" // time of flight sensor library
}

VL53L0XToFSensor::VL53L0XToFSensor()
{
    int res = tofInit(1, 0x29, 1); // init using i2c bus 1 address 0x29, and longrange active (1)
    if (res != 1)
        qDebug() << "Error: Failed to initilize VL53L0XToFSensor.";
    else {
        int model, revision;
        tofGetModel(&model, &revision);
        qDebug() << "VL53L0X" << "Model ID - "<< model<< "Revision ID -" << revision << "successfully opened.";

        connect(&mPollTimer, &QTimer::timeout, [this]() {
            int iDistance_mm = tofReadDistance();
            if (iDistance_mm < 4096) { // valid range?
                //qDebug() <<  "Object Distance to trailer = " <<  iDistance << "mm";
                mLastDistance = iDistance_mm / 1000.0;
            } else
                mLastDistance = -1.0;

            emit updatedDistance(mLastDistance);
        });
        mPollTimer.start(mPollIntervall_ms); // call back (poll) every mPollIntervall_ms e.g., 50 ms
    }
}

bool VL53L0XToFSensor::setUpdateIntervall(int pollIntervall_ms)
{
   mPollIntervall_ms = pollIntervall_ms;
   mPollTimer.start(mPollIntervall_ms);

   return true;
}
