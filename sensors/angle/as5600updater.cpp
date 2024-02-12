/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "as5600updater.h"
#include <cmath>
#include <QDebug>

AS5600Updater::AS5600Updater(QSharedPointer<VehicleState> vehicleState) : AngleSensorUpdater(vehicleState)
{
   int res{};
   res = as5600_basic_init(); // basic init for reading angle using i2c
   
   if (res == 0) {
      printSensorInfo(); // print AS5600 information
      connect(&mPollTimer, &QTimer::timeout, [this, vehicleState]() {
         uint16_t angle_raw{};    
         uint16_t scaled_angle{};    
         float angleInDegrees{};
         int res{};
         // read 1) degree angle (converted from raw value), 2) raw angle (0x0C | 0x0D), 3) scaled angle (0x0E | 0x0F)
         res = as5600_basic_read(&angleInDegrees, &angle_raw, &scaled_angle);
         if (res == 0) {
            //qDebug() << "as5600: scaled angle: " << scaled_angle << "| in Radians : "  <<"| in degrees: " << deg ;
            angleInDegrees = angleInDegrees -94.043;
            scaled_angle = scaled_angle - 1070;
             double angleRadians = angleInDegrees * (M_PI / 180.0);
            
            // only TruckState has a trailer angle
            QSharedPointer<TruckState> truckState = qSharedPointerDynamicCast<TruckState>(vehicleState);
            if (truckState) {
                  truckState->setTrailerAngle(scaled_angle, angleRadians, angleInDegrees);
            } else {
               qDebug() << "Error: Failed to cast VehicleState to TruckState.";
            }
         } else {
            qDebug() << "ERROR: as5600 Read failed";
         }
      });   
      mPollTimer.start(mPollIntervall_ms); // call back (poll) every 50 ms
   } else {
      qDebug() << "ERROR: Unable to open i2c bus to AS5600";
   }
}

bool AS5600Updater::setUpdateIntervall(int pollIntervall_ms)
{
   mPollIntervall_ms = pollIntervall_ms;
   mPollTimer.start(mPollIntervall_ms);

   return true;
}

void AS5600Updater::printSensorInfo()
{
   int res{};
   as5600_info_t info;
   // get chip information 
   res = as5600_info(&info);
   if (res != 0) {
      qDebug() << "as5600: get info failed";
   }
   else {
      // print chip information 
      qDebug() << "as5600: chip is " << info.chip_name;
      qDebug() << "as5600: manufacturer is " << info.manufacturer_name;
      qDebug() << "as5600: interface is " << info.interface;
      qDebug() << "as5600: min supply voltage is " << info.supply_voltage_min_v;
      qDebug() << "as5600: max supply voltage is " << info.supply_voltage_max_v;
      qDebug() << "as5600: max current is " << info.max_current_ma;
      qDebug() << "as5600: max temperature is " << info.temperature_max;
      qDebug() << "as5600: min temperature is " << info.temperature_min;
   }
}