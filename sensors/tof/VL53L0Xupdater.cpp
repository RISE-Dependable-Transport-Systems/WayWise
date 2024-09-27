/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "VL53L0Xupdater.h"
#include <cmath>
#include <QDebug>

extern "C" {
#include "tof.h" // time of flight sensor library
}
VL53L0XUpdater::VL53L0XUpdater(QSharedPointer<VehicleState> vehicleState) : ToFSensorUpdater(vehicleState)
{
   int res{};
   res = tofInit(1, 0x29, 1); // init using i2c bus 1 address 0x29, and longrange active (1)
   int model, revision;

   if (res != 1){
       qDebug() << "Error: Failed to initilize ";
   }
   else{
       qDebug() << "VL53L0X device successfully opened." ;
      res = tofGetModel(&model, &revision);
      qDebug() << "Model ID - "<< model;
      qDebug() << "Revision ID -"<< revision;

      connect(&mPollTimer, &QTimer::timeout, [this, vehicleState]() {
         int iDistance = tofReadDistance();
         if (iDistance < 4096){ // valid range?
            qDebug() <<  "Distance = " <<  iDistance << "dmm";
         }
         QSharedPointer<TruckState> truckState = qSharedPointerDynamicCast<TruckState>(vehicleState);
         if (truckState) {  
            truckState->setTrailerDistanceToF(iDistance);
         } else {
            qDebug() << "Error: Failed to cast VehicleState to TruckState.";
         }

      });   
      mPollTimer.start(mPollIntervall_ms); // call back (poll) every mPollIntervall_ms e.g., 50 ms

   }

}

bool VL53L0XUpdater::setUpdateIntervall(int pollIntervall_ms)
{
   mPollIntervall_ms = pollIntervall_ms;
   mPollTimer.start(mPollIntervall_ms);

   return true;
}
