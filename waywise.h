/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef WAYWISE_H
#define WAYWISE_H

#include <cstdint>

constexpr uint16_t WAYWISE_MAVLINK_VENDOR_ID = 12321;
constexpr uint8_t WAYWISE_MAVLINK_AUTOPILOT_ID = 121;


typedef enum WAYWISE_VEHICLE_TYPE
{
   VEHICLE_TYPE_ROVER=0,    /* Generic ground rover vehicle | */
   VEHICLE_TYPE_TRUCK=1,    /* Truck vehicle | */
   VEHICLE_TYPE_TRAILER=2,  /* Trailer passive vehicle | */
    // ADD YOUR VEHICLE TYPE HERE
} WAYWISE_VEHICLE_TYPE;


#endif // WAYWISE_H
