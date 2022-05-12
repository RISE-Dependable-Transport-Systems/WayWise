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
};

#endif // MAVSDKGIMBAL_H
