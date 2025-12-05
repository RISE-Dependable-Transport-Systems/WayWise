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
    static QPointF lastGNSSPoint = QPointF();
    static PosPoint lastOdomPosPoint = PosPoint();
    PosPoint gnssPosPoint = mObjectState->getPosition(PosType::GNSS);
    PosPoint odomPosPoint = mObjectState->getPosition(PosType::odom);

    GnssFixStatus gnssFixStatus;
    if (simulationFn) {
        gnssFixStatus = simulationFn(odomPosPoint.getTime(), mObjectState);
        gnssPosPoint = mObjectState->getPosition(PosType::GNSS);
    } else {
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

        gnssFixStatus.isFused = true;
        gnssFixStatus.fixType = GNSS_FIX_TYPE::FIX_3D;
        gnssFixStatus.horizontalAccuracy = 0.0;
        gnssFixStatus.verticalAccuracy = 0.0;
        gnssFixStatus.headingAccuracy = 0.0;
        gnssFixStatus.lastRtcmCorrectionAge = 0;
        gnssFixStatus.numSatellites = 5;
    }

    emit updatedGNSSPositionAndYaw(mObjectState, QLineF(lastGNSSPoint, gnssPosPoint.getPoint()).length(), gnssFixStatus);
    lastGNSSPoint = gnssPosPoint.getPoint();
    lastOdomPosPoint = odomPosPoint;
}

void GNSSReceiver::updateGNSSPositionAndYaw(llh_t llh, double heading, bool isFused)
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

    double vehYaw_radENU = 0.0;
    if (isFused) {
        double yaw_degENU = coordinateTransforms::yawNEDtoENU(heading) + mAChipOrientationOffset.yawOffset_deg;

        // normalize to [-180.0:180.0]
        while (yaw_degENU < -180.0)
            yaw_degENU += 360.0;
        while (yaw_degENU >= 180.0)
            yaw_degENU -= 360.0;

        gnssPos.setYaw(yaw_degENU);

        vehYaw_radENU = yaw_degENU * M_PI / 180.0;

        // Apply Chip to rear axle offset if set.
        if (mChipToBaseOffset.x != 0.0 || mChipToBaseOffset.y != 0.0) {
            gnssPos.updateWithOffsetAndYawRotation(mChipToBaseOffset, vehYaw_radENU);
        }
    } else { // Assumes fused yaw is updated.
        PosPoint fusedPos = mObjectState->getPosition(PosType::fused);
        vehYaw_radENU = fusedPos.getYaw() * M_PI / 180.0;

        // Apply antenna to rear axle offset if set.
        xyz_t mAntennaToRearAxleOffset = mAntennaToChipOffset + mChipToBaseOffset;
        if (mAntennaToRearAxleOffset.x != 0.0 || mAntennaToRearAxleOffset.y != 0.0) {
            gnssPos.updateWithOffsetAndYawRotation(mAntennaToRearAxleOffset, vehYaw_radENU);
        }
    }
    mObjectState->setPosition(gnssPos);
}