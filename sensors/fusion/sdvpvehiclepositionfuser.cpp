/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "sdvpvehiclepositionfuser.h"
#include <QDebug>

SDVPVehiclePositionFuser::SDVPVehiclePositionFuser(QObject *parent) : QObject(parent)
{

}

void SDVPVehiclePositionFuser::samplePosFused(const PosPoint &posFused)
{
    mPosFusedHistory[mPosFusedHistoryCurrentIdx++] = PosSample {posFused.getPoint(), posFused.getYaw(), posFused.getTime()};
    if (mPosFusedHistoryCurrentIdx == POSFUSED_HISTORY_SIZE)
        mPosFusedHistoryCurrentIdx = 0;
}

SDVPVehiclePositionFuser::PosSample SDVPVehiclePositionFuser::getClosestPosFusedSampleInTime(QTime timeUTC)
{
    int lastSavedHistoryIdx = mPosFusedHistoryCurrentIdx - 1 >= 0 ? mPosFusedHistoryCurrentIdx - 1 : POSFUSED_HISTORY_SIZE - 1;
    int smallestDiffFound = abs(timeUTC.msecsSinceStartOfDay() - mPosFusedHistory[lastSavedHistoryIdx].timestamp.msecsSinceStartOfDay());
    int posFusedHistoryClosestIdx = -1;

    for (int i = lastSavedHistoryIdx; i != mPosFusedHistoryCurrentIdx; i--) {
        if (i < 0)
            i = POSFUSED_HISTORY_SIZE-1;

        int currentDiff = abs(timeUTC.msecsSinceStartOfDay() - mPosFusedHistory[i].timestamp.msecsSinceStartOfDay());
        if (currentDiff <= smallestDiffFound) {
            smallestDiffFound = currentDiff;
            posFusedHistoryClosestIdx = i;
        } else
            break;
    }

    return mPosFusedHistory[posFusedHistoryClosestIdx];
}

double SDVPVehiclePositionFuser::getPosGNSSxyDynamicGain() const
{
    return mPosGNSSxyDynamicGain;
}

void SDVPVehiclePositionFuser::setPosGNSSxyDynamicGain(double posGNSSxyDynamicGain)
{
    mPosGNSSxyDynamicGain = posGNSSxyDynamicGain;
}

double SDVPVehiclePositionFuser::getMaxSignedStepFromValueTowardsGoal(double value, double goal, double maxStepSize) {
    maxStepSize = abs(maxStepSize);

    if ((value < goal) && (value + maxStepSize) < goal)
        return maxStepSize;

    if ((value > goal) && (value - maxStepSize) > goal)
        return -maxStepSize;

    return goal - value;
}

void SDVPVehiclePositionFuser::correctPositionAndYawGNSS(QSharedPointer<VehicleState> vehicleState, double distanceMoved, bool fused)
{
    mPosGNSSisFused = fused;
    PosPoint posGNSS = vehicleState->getPosition(PosType::GNSS);
    PosPoint posIMU = vehicleState->getPosition(PosType::IMU);
    PosPoint posFused = vehicleState->getPosition(PosType::fused);

    if (mPosGNSSisFused) {// use GNSS position directly if that was already fused (e.g., F9R).
        posFused.setYaw(posGNSS.getYaw());
        posFused.setXY(posGNSS.getX(), posGNSS.getY());
    } else {
        
        posFused.setXY(posGNSS.getX(), posGNSS.getY());
            posFused.setYaw(posGNSS.getYaw());
         //qDebug() << " forward yaw " << posGNSS.getYaw();
        // if( vehicleState->getSpeed() >= 0) {
        //     posFused.setYaw(posGNSS.getYaw());
        //     // qDebug() << " forward yaw " << posGNSS.getYaw();
        // }
        if ( vehicleState->getSpeed() < 0){
        //         // When moving in reverse, adjust the yaw accordingly
            float reverseYaw = posGNSS.getYaw() + 180.0; // Invert yaw by adding 180 degrees
            if (reverseYaw > 180.0) {
                reverseYaw -= 360.0; // Ensure yaw stays within the range of [-180, 180] degrees
            }
            posFused.setYaw(reverseYaw);
             //qDebug() << " reverse yaw " << reverseYaw;
        }
//         // 1. GNSS position is precise, but old. Find sampled position at matching time to calculate error
//         PosSample closestPosFusedSample = getClosestPosFusedSampleInTime(posGNSS.getTime());

//         // 2. Update yaw offset, limit max change depending on last driven distance reported by odometry (if available)
//         //    GNSS yaw represents direction of motion and needs to be reversed when driving backwards
//         double yawErrorCmpToGNSS = (((mPosOdomDistanceDrivenSinceGNSSupdate < 0.0) ? 180.0 : 0.0) + posGNSS.getYaw()) - closestPosFusedSample.yaw;
//         while (yawErrorCmpToGNSS < -180.0) yawErrorCmpToGNSS += 360.0;
//         while (yawErrorCmpToGNSS > 180.0) yawErrorCmpToGNSS -= 360.0;
//         double yawMaxChangeDistanceInput = mPosOdomDistanceDrivenSinceGNSSupdate == 0.0 ? distanceMoved : abs(mPosOdomDistanceDrivenSinceGNSSupdate);
//         double yawStepSize = yawMaxChangeDistanceInput * mPosGNSSyawGain;
//         double stepYaw = getMaxSignedStepFromValueTowardsGoal(mPosIMUyawOffset, mPosIMUyawOffset + yawErrorCmpToGNSS, yawStepSize);

//         mPosIMUyawOffset += stepYaw;

//         // 3. Update position, limit max change depending on last driven distance reported by odometry (if available) but jump to GNSS position if error is big
//         double posMaxChangeDistanceInput = yawMaxChangeDistanceInput;
//         double xyStepSize = mPosGNSSxyStaticGain + posMaxChangeDistanceInput * mPosGNSSxyDynamicGain;
//         double stepX = getMaxSignedStepFromValueTowardsGoal(closestPosFusedSample.posXY.x(), posGNSS.getX(), xyStepSize);
//         double stepY = getMaxSignedStepFromValueTowardsGoal(closestPosFusedSample.posXY.y(), posGNSS.getY(), xyStepSize);
//         QPointF posErrorCmpToGNSS = QPointF(posGNSS.getX() - closestPosFusedSample.posXY.x(), posGNSS.getY() - closestPosFusedSample.posXY.y());

//         if (abs(posErrorCmpToGNSS.x()) > BIG_DISTANCE_ERROR_m || abs(posErrorCmpToGNSS.y()) > BIG_DISTANCE_ERROR_m) {
//             posFused.setXY(posGNSS.getX(), posGNSS.getY());
// //            qDebug() << "BIG_DISTANCE_ERROR_m";
//         } else {
//             posFused.setX(posFused.getX() + stepX);
//             posFused.setY(posFused.getY() + stepY);
//         }
    }

    posFused.setHeight(posGNSS.getHeight());
    posFused.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
    vehicleState->setPosition(posFused);
    mPosOdomDistanceDrivenSinceGNSSupdate = 0.0;
}

void SDVPVehiclePositionFuser::correctPositionAndYawOdom(QSharedPointer<VehicleState> vehicleState, double distanceDriven)
{
    // if (!mPosGNSSisFused) {
    //     PosPoint posFused = vehicleState->getPosition(PosType::fused);

    //     // use Odom input from motorcontroller for IMU-based dead reckoning
    //     double yawRad = posFused.getYaw() / (180.0/M_PI);
    //     posFused.setXY(posFused.getX() + cos(yawRad) * distanceDriven,
    //                    posFused.getY() + sin(yawRad) * distanceDriven);

    //     posFused.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
    //     vehicleState->setPosition(posFused);

    //     samplePosFused(posFused);
    // }

    mPosOdomDistanceDrivenSinceGNSSupdate += distanceDriven;
}

void SDVPVehiclePositionFuser::correctPositionAndYawIMU(QSharedPointer<VehicleState> vehicleState)
{
    // if (!mPosGNSSisFused) {
    //     static bool standstillAtLastCall = false;
    //     static double yawWhenStopping = 0.0;
    //     static double yawDriftSinceStandstill = 0.0;

    //     // --- correct relative/raw IMU yaw with external offset
    //     PosPoint posIMU = vehicleState->getPosition(PosType::IMU);
    //     PosPoint posFused = vehicleState->getPosition(PosType::fused);

    //     // 1. handle drift at standstill
    //     if (fabs(vehicleState->getSpeed()) < 0.05) {
    //         if (!standstillAtLastCall)
    //             yawWhenStopping = posIMU.getYaw();

    //         standstillAtLastCall = true;
    //         yawDriftSinceStandstill = yawWhenStopping - posIMU.getYaw();
    //         posIMU.setYaw(yawWhenStopping); // fix yaw during standstill
    //     } else {
    //         if (standstillAtLastCall)
    //             mPosIMUyawOffset += yawDriftSinceStandstill;

    //         standstillAtLastCall = false;
    //     }

    //     // 2. apply offset & normalize
    //     double yawResult = posIMU.getYaw() + mPosIMUyawOffset;
    //     while (yawResult < -180.0)
    //         yawResult += 360.0;
    //     while (yawResult >= 180.0)
    //         yawResult -= 360.0;
    //     posFused.setYaw(yawResult);

    //     posFused.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
    //     vehicleState->setPosition(posFused);
    // }
}

void SDVPVehiclePositionFuser::setPosGNSSxyStaticGain(double posGNSSxyStaticGain)
{
    mPosGNSSxyStaticGain = posGNSSxyStaticGain;
}


void SDVPVehiclePositionFuser::setPosGNSSyawGain(double posGNSSyawGain)
{
    mPosGNSSyawGain = posGNSSyawGain;
}
