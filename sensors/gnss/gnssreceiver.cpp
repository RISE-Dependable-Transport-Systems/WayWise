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

void GNSSReceiver::simulationStep(const std::function<GnssFixStatus(QTime, QSharedPointer<ObjectState>)> &perturbationFn)
{
    static QPointF lastGNSSPoint = QPointF();
    static PosPoint lastOdomPosPoint = PosPoint();
    PosPoint gnssPosPoint = mObjectState->getPosition(PosType::GNSS);
    PosPoint odomPosPoint = mObjectState->getPosition(PosType::odom);

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

    GnssFixStatus gnssFixStatus;
    if (perturbationFn) {
        gnssFixStatus = perturbationFn(odomPosPoint.getTime(), mObjectState);
        gnssPosPoint = mObjectState->getPosition(PosType::GNSS);
    } else {
        gnssFixStatus.isFusedOnChip = true;
        gnssFixStatus.fixType = GNSS_FIX_TYPE::FIX_3D;
        gnssFixStatus.horizontalAccuracy = 0.0;
        gnssFixStatus.verticalAccuracy = 0.0;
        gnssFixStatus.headingAccuracy = 0.0;
        gnssFixStatus.lastRtcmCorrectionAge = 0;
        gnssFixStatus.numSatellites = 0;
    }

    emit updatedGNSSPositionAndYaw(mObjectState, QLineF(lastGNSSPoint, gnssPosPoint.getPoint()).length(), gnssFixStatus);
    lastGNSSPoint = gnssPosPoint.getPoint();
    lastOdomPosPoint = odomPosPoint;
}
