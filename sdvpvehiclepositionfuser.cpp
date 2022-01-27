#include "sdvpvehiclepositionfuser.h"

SDVPVehiclePositionFuser::SDVPVehiclePositionFuser(QObject *parent) : QObject(parent)
{

}


SDVPVehiclePositionFuser::PosSample SDVPVehiclePositionFuser::getClosestPosFusedSampleInTime(int msecsSinceStartOfDay)
{
    int lastSavedHistoryIdx = mPosFusedHistoryCurrentIdx - 1 >= 0 ? mPosFusedHistoryCurrentIdx - 1 : POSFUSED_HISTORY_SIZE - 1;
    int smallestDiffFound = abs(msecsSinceStartOfDay - mPosFusedHistory[lastSavedHistoryIdx].timestamp.msecsSinceStartOfDay());
    int posFusedHistoryClosestIdx = -1;

    for (int i = lastSavedHistoryIdx; i != mPosFusedHistoryCurrentIdx; i--) {
        if (i < 0)
            i = POSFUSED_HISTORY_SIZE-1;

        int currentDiff = abs(msecsSinceStartOfDay - mPosFusedHistory[i].timestamp.msecsSinceStartOfDay());
        if (currentDiff < smallestDiffFound) {
            smallestDiffFound = currentDiff;
            posFusedHistoryClosestIdx = i;
        } else
            break;
    }

    return mPosFusedHistory[posFusedHistoryClosestIdx];
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
        // 1. GNSS position is precise, but old. Find sampled position at matching time to calculate error
        PosSample closestPosFusedSample = getClosestPosFusedSampleInTime(posGNSS.getTime().msecsSinceStartOfDay());

        // 2. Update yaw offset, limit max change depending on last driven distance reported by odometry (if available)
        double yawErrorCmpToGNSS = posGNSS.getYaw() - closestPosFusedSample.yaw;
        double yawMaxChangeDistanceInput = mPosOdomLastDistanceDriven == 0.0 ? distanceMoved : abs(mPosOdomLastDistanceDriven);
        double yawMaxChange = (yawMaxChangeDistanceInput * mPosGNSSyawGain) * (yawErrorCmpToGNSS / abs(yawErrorCmpToGNSS));

        mPosIMUyawOffset += abs(yawErrorCmpToGNSS) > abs(yawMaxChange) ? yawMaxChange : yawErrorCmpToGNSS;

        // 3. Update position, limit max change depending on last driven distance reported by odometry (if available) but jump to GNSS position if error is big
        QPointF posErrorCmpToGNSS = QPointF(posGNSS.getX() - closestPosFusedSample.posXY.x(), posGNSS.getY() - closestPosFusedSample.posXY.y());
        double posMaxChangeDistanceInput = yawMaxChangeDistanceInput;
        QPointF posMaxChange = QPointF((mPosGNSSxyGain + posMaxChangeDistanceInput * mPosGNSSxyGain) * (posErrorCmpToGNSS.x() / abs(posErrorCmpToGNSS.x())),
                                       (mPosGNSSxyGain + posMaxChangeDistanceInput * mPosGNSSxyGain) * (posErrorCmpToGNSS.y() / abs(posErrorCmpToGNSS.y())));

        if (abs(posErrorCmpToGNSS.x()) > BIG_DISTANCE_ERROR_m || abs(posErrorCmpToGNSS.y()) > BIG_DISTANCE_ERROR_m)
            posFused.setXY(posGNSS.getX(), posGNSS.getY());
        else {
            posFused.setX(posFused.getX() + abs(posErrorCmpToGNSS.x()) > abs(posMaxChange.x()) ? posMaxChange.x() : posErrorCmpToGNSS.x());
            posFused.setY(posFused.getY() + abs(posErrorCmpToGNSS.y()) > abs(posMaxChange.y()) ? posMaxChange.y() : posErrorCmpToGNSS.y());
        }
    }

    posFused.setHeight(posGNSS.getHeight());
    vehicleState->setPosition(posFused);
}

void SDVPVehiclePositionFuser::correctPositionAndYawOdom(QSharedPointer<VehicleState> vehicleState, double distanceDriven)
{
    if (!mPosGNSSisFused) {
        PosPoint posFused = vehicleState->getPosition(PosType::fused);

        // use Odom input from motorcontroller for IMU-based dead reckoning
        double yawRad = posFused.getYaw() / (180.0/M_PI);
        posFused.setXY(posFused.getX() + cos(-yawRad) * distanceDriven,
                       posFused.getY() + sin(-yawRad) * distanceDriven);

        vehicleState->setPosition(posFused);

        // sample position to history
        mPosFusedHistory[mPosFusedHistoryCurrentIdx++] = PosSample {posFused.getPoint(), posFused.getYaw(), posFused.getTime()};
        if (mPosFusedHistoryCurrentIdx == POSFUSED_HISTORY_SIZE)
            mPosFusedHistoryCurrentIdx = 0;
    }

    mPosOdomLastDistanceDriven = distanceDriven;
}

void SDVPVehiclePositionFuser::correctPositionAndYawIMU(QSharedPointer<VehicleState> vehicleState)
{
    if (!mPosGNSSisFused) {
        static bool standstillAtLastCall = false;
        static double yawWhenStopping = 0.0;
        static double yawDriftSinceStandstill = 0.0;

        // --- correct relative/raw IMU yaw with external offset
        PosPoint posIMU = vehicleState->getPosition(PosType::IMU);
        PosPoint posFused = vehicleState->getPosition(PosType::fused);

        // 1. handle drift at standstill
        if (fabs(vehicleState->getSpeed()) < 0.05) {
            if (!standstillAtLastCall)
                yawWhenStopping = posIMU.getYaw();

            standstillAtLastCall = true;
            yawDriftSinceStandstill = yawWhenStopping - posIMU.getYaw();
            posIMU.setYaw(yawWhenStopping); // fix yaw during standstill
        } else {
            if (standstillAtLastCall)
                mPosIMUyawOffset += yawDriftSinceStandstill;

            standstillAtLastCall = false;
        }

        // 2. apply offset & normalize
        double yawResult = posIMU.getYaw() + mPosIMUyawOffset;
        while (yawResult < 0.0)
            yawResult += 360.0;
        while (yawResult > 360.0)
            yawResult -= 360.0;
        posFused.setYaw(yawResult);

        vehicleState->setPosition(posFused);
    }
}

void SDVPVehiclePositionFuser::setPosGNSSxyGain(double posGNSSxyGain)
{
    mPosGNSSxyGain = posGNSSxyGain;
}


void SDVPVehiclePositionFuser::setPosGNSSyawGain(double posGNSSyawGain)
{
    mPosGNSSyawGain = posGNSSyawGain;
}
