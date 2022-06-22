/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#ifndef MAVSDKGIMBAL_H
#define MAVSDKGIMBAL_H

#include <QObject>
#include <QSharedPointer>
#include "sensors/camera/gimbal.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/gimbal/gimbal.h>

class MavsdkGimbal : public Gimbal
{
    Q_OBJECT
public:
    explicit MavsdkGimbal(std::shared_ptr<mavsdk::System> system);
private:
    QSharedPointer<mavsdk::Gimbal> mMavsdkGimbal;

    // Gimbal interface
public:
    virtual void setRegionOfInterest(const llh_t &roiLlh) override;
    virtual void setPitchAndYaw(double pitch_deg, double yaw_deg) override;
    virtual void setYawLocked(bool setLocked) override;
};

#endif // MAVSDKGIMBAL_H
