/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "as5600updater.h"
#include <QDebug>

AS5600Updater::AS5600Updater(QSharedPointer<VehicleState> vehicleState) : AngleSensorUpdater(vehicleState)
{
   int res{};

   res = as5600_basic_init();
   printSensorInfo();

   if (res == 0)
   {

      connect(&mPollTimer, &QTimer::timeout, [this, vehicleState]()
              {
               uint16_t angle_raw{};    
               uint16_t scaled_angle{};    
               float deg{};
               int res{};
               /* read data */
               res = as5600_basic_read(&deg, &angle_raw, &scaled_angle);
               if (res == 0){
                  // QSharedPointer<VehicleState> vehicleState = getVehicleState();
                  QSharedPointer<TruckState> truckState = qSharedPointerDynamicCast<TruckState>(vehicleState);
                  if (truckState) {
                        truckState->setTrailerAngle(scaled_angle);
                        //qDebug() << "as5600: scaled angle: " << scaled_angle << "| raw angle: " << angle_raw <<"| in degrees: " << deg ;
                  } else {
                        qDebug() << "Error: Failed to cast VehicleState to TruckState.";
                  }
               
               }
               else {
                  qDebug() << "ERROR: as5600 Read failed";
               }
               
               });
               
      mPollTimer.start(mPollIntervall_ms);
   }
   else
   {
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
   if (res != 0)
   {
      qDebug() << "as5600: get info failed";
   }
   else
   {
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