/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Rewritten and restructured version of SDVP's sensor "fusion" algorithm for obtaining position and orientation 
 * (see: https://github.com/RISE-Dependable-Transport-Systems/rise_sdvp/blob/master/Embedded/RC_Controller/pos.c).
 * Inputs are GNSS position, Odometry feedback (e.g., from motor controller) and IMU orientation.
 * The algorithm tries to tackle two main problems: 1. GNSS position is exact but old and arrives at low frequency 2. IMU orientation is not absolute and drifts
 * The general approach is as follows:
 *  - GNSS is the ground truth for the position few hundred ms ago, two consecutive GNSS positions are used to obtain yaw
 *  - IMU measurements arrive more frequently and are applied to the "fused" position's yaw directly with the yaw offset obtained from GNSS.
 *      The yaw offset from GNSS allows to calculate the absolute yaw. The offset is fixed at standstill to counter IMU drift.
 *  - Odom feedback also arrives more frequently than GNSS. The driven distance received and the "fused" yaw are used to update the "fused" position inbetween input from GNSS.
 *      The resulting "fused" position and yaw are sampled in a history buffer.
 *  - When a new GNSS position arrives, the time-wise closest "fused" position in the history buffer is taken to calculate the position error towards the new GNSS position at the GNSS position's time.
 *      The resulting error is applied to the current "fused" position with weights (static and dynamic, based on distance moved). Yaw is updated similarly.
 */

#ifndef SDVPVEHICLEPOSITIONFUSER_H
#define SDVPVEHICLEPOSITIONFUSER_H

#include <QObject>
#include <QSharedPointer>
#include "vehicles/vehiclestate.h"

class SDVPVehiclePositionFuser : public QObject
{
    Q_OBJECT
public:
    explicit SDVPVehiclePositionFuser(QObject *parent = nullptr);
    void correctPositionAndYawGNSS(QSharedPointer<VehicleState> vehicleState, double distanceMoved, bool fused);
    void correctPositionAndYawOdom(QSharedPointer<VehicleState> vehicleState, double distanceDriven);
    void correctPositionAndYawIMU(QSharedPointer<VehicleState> vehicleState);

    void setPosGNSSxyStaticGain(double posGNSSxyStaticGain);
    void setPosGNSSyawGain(double posGNSSyawGain);

    double getPosGNSSxyDynamicGain() const;
    void setPosGNSSxyDynamicGain(double posGNSSxyDynamicGain);

signals:

private:
    struct PosSample {
        QPointF posXY;
        double yaw;
        QTime timestamp;
    };

    double getMaxSignedStepFromValueTowardsGoal(double value, double goal, double maxStepSize);

    void samplePosFused(const PosPoint &posFused);
    PosSample getClosestPosFusedSampleInTime(QTime timeUTC);

    double mPosIMUyawOffset = 0.0;
    bool mPosGNSSisFused = false; // use GNSS pos as "fused" pos when true, e.g., F9R
    double mPosGNSSxyStaticGain = 0.05;
    double mPosGNSSxyDynamicGain = 0.1;
    double mPosGNSSyawGain = 1.0;
    double mPosOdomDistanceDrivenSinceGNSSupdate = std::numeric_limits<double>::min();
    static constexpr double BIG_DISTANCE_ERROR_m = 50.0;

    static constexpr int POSFUSED_HISTORY_SIZE = 128;
    PosSample mPosFusedHistory[POSFUSED_HISTORY_SIZE];
    int mPosFusedHistoryCurrentIdx = 0;
};

#endif // SDVPVEHICLEPOSITIONFUSER_H
