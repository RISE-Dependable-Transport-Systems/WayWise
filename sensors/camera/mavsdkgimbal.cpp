/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "mavsdkgimbal.h"
#include <QtDebug>

MavsdkGimbal::MavsdkGimbal(std::shared_ptr<mavsdk::System> system)
{
    mMavsdkGimbal = QSharedPointer<mavsdk::Gimbal>::create(system);

//    mMavsdkGimbal->subscribe_control([this](mavsdk::Gimbal::ControlStatus controlStatus){
//       qDebug() << (int)controlStatus.control_mode << controlStatus.sysid_primary_control << controlStatus.compid_primary_control
//                << controlStatus.sysid_secondary_control << controlStatus.compid_secondary_control;
//    });

    mMavsdkGimbal->take_control_async(mavsdk::Gimbal::ControlMode::Primary, [](mavsdk::Gimbal::Result result) {
        if (result != mavsdk::Gimbal::Result::Success)
            qDebug() << "Warning: MavsdkGimbal failed to take control over gimbal.";
    });
}


void MavsdkGimbal::setRegionOfInterest(const llh_t &roiLlh)
{
    mMavsdkGimbal->set_roi_location_async(roiLlh.latitude, roiLlh.longitude, roiLlh.height, [](mavsdk::Gimbal::Result result) {
        if (result != mavsdk::Gimbal::Result::Success)
            qDebug() << "Warning: MavsdkGimbal failed to set ROI.";
    });
}

void MavsdkGimbal::setPitchAndYaw(double pitch_deg, double yaw_deg)
{
    mMavsdkGimbal->set_pitch_and_yaw_async(pitch_deg, yaw_deg, [](mavsdk::Gimbal::Result result) {
        if (result != mavsdk::Gimbal::Result::Success)
            qDebug() << "Warning: MavsdkGimbal failed to set pitch & yaw.";
    });
}

void MavsdkGimbal::setYawLocked(bool setLocked)
{
    mMavsdkGimbal->set_mode_async((setLocked ? mavsdk::Gimbal::GimbalMode::YawLock : mavsdk::Gimbal::GimbalMode::YawFollow), [](mavsdk::Gimbal::Result result) {
        if (result != mavsdk::Gimbal::Result::Success)
            qDebug() << "Warning: MavsdkGimbal failed to lock/unlock yaw.";
    });
}
