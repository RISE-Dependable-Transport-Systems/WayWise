/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "imuorientationupdater.h"

IMUOrientationUpdater::IMUOrientationUpdater(QSharedPointer<ObjectState> objectState)
{
    mObjectState = objectState;
}

QSharedPointer<ObjectState> IMUOrientationUpdater::getObjectState() const
{
    return mObjectState;
}

void IMUOrientationUpdater::simulationStep(const std::function<void(QTime, QSharedPointer<ObjectState>)> &simulationFn)
{
    static PosPoint lastOdomPosPoint = PosPoint();
    PosPoint odomPosPoint = mObjectState->getPosition(PosType::odom);

    if (simulationFn) {
        simulationFn(odomPosPoint.getTime(), mObjectState);
    } else {
        double deltaX = odomPosPoint.getX() - lastOdomPosPoint.getX();
        double deltaY = odomPosPoint.getY() - lastOdomPosPoint.getY();
        double deltaYaw = odomPosPoint.getYaw() - lastOdomPosPoint.getYaw();

        PosPoint imuPosPoint = mObjectState->getPosition(PosType::IMU);
        imuPosPoint.setX(imuPosPoint.getX() + deltaX);
        imuPosPoint.setY(imuPosPoint.getY() + deltaY);
        double yawResult = imuPosPoint.getYaw() + deltaYaw;

        while (yawResult < -180.0)
            yawResult += 360.0;
        while (yawResult >= 180.0)
            yawResult -= 360.0;

        imuPosPoint.setYaw(yawResult);
        imuPosPoint.setTime(odomPosPoint.getTime());
        mObjectState->setPosition(imuPosPoint);
    }

    emit updatedIMUOrientation(mObjectState);
    lastOdomPosPoint = odomPosPoint;
}
