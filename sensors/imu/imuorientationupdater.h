/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract interface for receiving orientation data from IMUs
 */

#ifndef IMUORIENTATIONUPDATER_H
#define IMUORIENTATIONUPDATER_H

#include <QObject>
#include <QSharedPointer>
#include "vehicles/objectstate.h"

class IMUOrientationUpdater : public QObject
{
    Q_OBJECT
public:
    IMUOrientationUpdater(QSharedPointer<ObjectState> objectState);
    QSharedPointer<ObjectState> getObjectState() const;
    virtual bool setUpdateIntervall(int intervall_ms) = 0;
    void simulationStep(const std::function<void(QTime, QSharedPointer<ObjectState>)> &simulationFn = nullptr);


signals:
    void updatedIMUOrientation(QSharedPointer<ObjectState> objectState);

private:
    QSharedPointer<ObjectState> mObjectState; // object whose PosType::IMU is periodically updated

};

#endif // IMUORIENTATIONUPDATER_H
