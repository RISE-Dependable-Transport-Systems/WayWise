/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract base class for GNSS receivers, supporting both real and simulated GNSS data.
 */

#include "gnssreceiver.h"

GNSSReceiver::GNSSReceiver(QSharedPointer<ObjectState> objectState)
{
    mObjectState = objectState;
}

void GNSSReceiver::simulationStep(const std::function<GnssFixStatus(QTime, QSharedPointer<ObjectState>)> &simulationFn)
{
    static xyz_t lastGNSS_xyz;
    PosPoint gnssPosPoint = mObjectState->getPosition(PosType::GNSS);
    PosPoint odomPosPoint = mObjectState->getPosition(PosType::odom);

    GnssFixStatus gnssFixStatus;
    if (simulationFn) {
        gnssFixStatus = simulationFn(odomPosPoint.getTime(), mObjectState);
        gnssPosPoint = mObjectState->getPosition(PosType::GNSS);
    } else {
        static PosPoint lastOdomPosPoint = PosPoint();

        double deltaX = odomPosPoint.getX() - lastOdomPosPoint.getX();
        double deltaY = odomPosPoint.getY() - lastOdomPosPoint.getY();
        double deltaYaw = odomPosPoint.getYaw() - lastOdomPosPoint.getYaw();
        gnssPosPoint.setX(gnssPosPoint.getX() + deltaX);
        gnssPosPoint.setY(gnssPosPoint.getY() + deltaY);
        double yawResult = gnssPosPoint.getYaw() + deltaYaw;

        while (yawResult < -180.0)
            yawResult += 360.0;
        while (yawResult >= 180.0)
            yawResult -= 360.0;

        gnssPosPoint.setYaw(yawResult);
        gnssPosPoint.setTime(odomPosPoint.getTime());
        mObjectState->setPosition(gnssPosPoint);

        gnssFixStatus.isFusedOnChip = true;
        gnssFixStatus.fixType = GNSS_FIX_TYPE::FIX_3D;
        gnssFixStatus.horizontalAccuracy = 0.0;
        gnssFixStatus.verticalAccuracy = 0.0;
        gnssFixStatus.headingAccuracy = 0.0;
        gnssFixStatus.lastRtcmCorrectionAge = 0;
        gnssFixStatus.numSatellites = 0;

        lastOdomPosPoint = odomPosPoint;
    }

    emit updatedGNSSPositionAndOrientation(mObjectState, lastGNSS_xyz.dist(gnssPosPoint.getXYZ()), gnssFixStatus);
    lastGNSS_xyz = gnssPosPoint.getXYZ();
}

void GNSSReceiver::updateGNSSPositionAndOrientation(llh_t llh, rpy_t rpy_degNED, bool isFusedOnChip)
{

    PosPoint gnssPos = mObjectState->getPosition(PosType::GNSS);
    xyz_t xyz = {0.0, 0.0, 0.0};

    if (!mObjectState->isEnuReferenceSet()) {
        mObjectState->setEnuRef(llh);
        qDebug() << "GNSSReceiver: ENU reference point set to" << llh.latitude << llh.longitude << llh.height;
    } else
        xyz = coordinateTransforms::llhToEnu(mObjectState->getEnuRef(), llh);

    // Position
    gnssPos.setXYZ(xyz);

    if (isFusedOnChip) {
        double yaw_degENU = coordinateTransforms::yawNEDtoENU(rpy_degNED.yaw) + mAChipOrientationOffset.yawOffset_deg;

        // normalize to [-180.0:180.0]
        while (yaw_degENU < -180.0)
            yaw_degENU += 360.0;
        while (yaw_degENU >= 180.0)
            yaw_degENU -= 360.0;

        gnssPos.setYaw(yaw_degENU);

        // Apply Chip to rear axle offset if set.
        if (mChipToBaseOffset.x != 0.0 || mChipToBaseOffset.y != 0.0) {
            gnssPos.updateWithOffsetAndYawRotation(mChipToBaseOffset, yaw_degENU * M_PI / 180.0);
        }
        gnssPos.setRoll(rpy_degNED.roll);
        gnssPos.setPitch(-rpy_degNED.pitch); // NED to ENU
    } else { // Assumes fused yaw is updated.
        PosPoint fusedPos = mObjectState->getPosition(PosType::fused);
        rpy_t fusedPos_rpy_degENU = fusedPos.getRPY();

        // Apply antenna to rear axle offset if set.
        xyz_t mAntennaToRearAxleOffset = mAntennaToChipOffset + mChipToBaseOffset;
        if (mAntennaToRearAxleOffset.x != 0.0 || mAntennaToRearAxleOffset.y != 0.0) {
            gnssPos.updateWithOffsetAndYawRotation(mAntennaToRearAxleOffset, fusedPos_rpy_degENU.yaw * M_PI / 180.0);
        }
        gnssPos.setRoll(fusedPos_rpy_degENU.roll);
        gnssPos.setPitch(fusedPos_rpy_degENU.pitch);
    }
    mObjectState->setPosition(gnssPos);
}

void GNSSReceiver::updateGNSSPositionAndOrientation(llh_t llh, double heading_degNED, bool isFusedOnChip)
{
    rpy_t rpy_degNED = {0.0, 0.0, heading_degNED};
    updateGNSSPositionAndOrientation(llh, rpy_degNED, isFusedOnChip);
}