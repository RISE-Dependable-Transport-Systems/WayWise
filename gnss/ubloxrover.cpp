#include "ubloxrover.h"
#include "sdvp_qtcommon/coordinatetransforms.h"
#include <QDebug>

UbloxRover::UbloxRover(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;
    mEnuReference = {57.6828, 11.9637, 0}; // AztaZero {57.7810, 12.7692, 0}, Klätterlabbet {57.6876, 11.9807, 0};


    // Initialize AHRS algorithm from Fusion library.
    // This provides orientation data if the u-blox device contains an IMU (e.g., F9R), even without GNSS reception
    FusionBiasInitialise(&mFusionBias, 0.5f, mIMUSamplePeriod); // stationary threshold = 0.5 degrees per second, expected period 20 ms
    FusionAhrsInitialise(&mFusionAhrs, 0.5f); // gain = 0.5
    connect(&mUblox, &Ublox::rxEsfMeas, this, &UbloxRover::updateAHRS);

    // Use GNSS reception to update location
    connect(&mUblox, &Ublox::rxNavPvt, this, &UbloxRover::updateGNSS);

    // Print nav-pvt message
    // Search for nav-pvt in following document:
    // https://www.u-blox.com/sites/default/files/ZED-F9R_Interfacedescription_UBX-19056845.pdf
//    connect(&mUblox, &Ublox::rxNavPvt, this, [](const ubx_nav_pvt &pvt){
//        qDebug() << "NAV-PVT data:\n"
//                 << "\nDate: " << pvt.year << pvt.month << pvt.day
//                 << "\nTime: " << pvt.hour << pvt.min << pvt.second
//                 << "\nFix Type: " << pvt.fix_type
//                 << "\nGNSS valid fix: " << pvt.gnss_fix_ok
//                 << "\nHeading valid: " << pvt.head_veh_valid
//                 << "\nNumber of satelites used: " << pvt.num_sv
//                 << "\nLongitude:" << pvt.lon << "Latitude:" << pvt.lat << "Height:" << pvt.height
//                 << "\nGround speed " << pvt.g_speed << "m/s"
//                 << "\nHeading of motion: " << pvt.head_mot
//                 << "\nHeading of vehicle: " << pvt.head_veh
//                << "\nDifferential corrections applied:" << pvt.diffsoln
//                 << "\n";});

    // Print esf-status message
    // Search for sensor data type in following document for explanation:
    // https://www.u-blox.com/sites/default/files/ZED-F9R_Integrationmanual_UBX-20039643.pdf
//    connect(&mUblox, &Ublox::rxEsfStatus, this, [](const ubx_esf_status &status){
//        qDebug() << "ESF-STATUS data:\n"
//                 << "\nVersion: " << status.version
//                 << "\nFusion mode: " << status.fusion_mode
//                 << "\nNumber of sensors: " << status.num_sens
//                 << "\n";
//        for (int i = 0;i < status.num_sens;i++) {
//            qDebug() << "Sensor data type: " << status.sensors[i].type
//                     << "\nSensor data used: " << status.sensors[i].used
//                     << "\nSensor data ready: " << status.sensors[i].ready
//                     << "\nSensor calibration status: " << status.sensors[i].calib_status
//                     << "\nSensor time status: " << status.sensors[i].time_status
//                     << "\nSensor observation freq: " << status.sensors[i].freq
//                     << "\n";

//        }});
}

bool UbloxRover::connectSerial(const QSerialPortInfo &serialPortInfo)
{
    if (mUblox.connectSerial(serialPortInfo))
        if (configureUblox())
            return true;
        else {
            mUblox.disconnectSerial();
            return false;
        }
    else
        return false;
}

void UbloxRover::setEnuRef(llh_t enuRef)
{
    mEnuReference = enuRef;
}

void UbloxRover::writeRtcmToUblox(QByteArray data)
{
    mUblox.writeRaw(data);
}

bool UbloxRover::configureUblox()
{
    // The rate of NMEA and UBX protocol output messages are configurable
    // and it is possible to enable or disable single NMEA or UBX messages individually.
    // If the rate configuration value is zero, then the corresponding message will not be output.
    // Values greater than zero indicate how often the message is output.

    //mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_MEAS, 1);
    mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_PVT, 1); // Choose update rate
    mUblox.ubxCfgRate(100, 1, 0); // Navigation solution every 100 ms (10 Hz)
    //mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_STATUS, 1); // Choose update rate
    return true;
}

void UbloxRover::updateAHRS(const ubx_esf_meas &meas)
{
    const float G = 9.82f;
    static unsigned lastGyroTtag = 0;
    static float gyro_xyz[3] = {0.0, 0.0, 0.0};
    static unsigned lastAccTtag = 0;
    static float acc_xyz[3] = {0.0, 0.0, 0.0};
    static bool gotGyro = false, gotAcc = false;

    // Decode gyroscope/accelerometer data from UBX-ESF-MEAS message
    for (int i = 0; i < meas.num_meas; i++) {
        if (lastGyroTtag < meas.time_tag) { // Messages might arrive out of order -> discard old ones
            switch(meas.data_type[i]) {
            case GYRO_Z: {
                gyro_xyz[2] = esfMeas2Float(meas.data_field[i], 12);
                gotGyro = true;
            } break;
            case GYRO_Y: {
                gyro_xyz[1] = esfMeas2Float(meas.data_field[i], 12);
            } break;
            case GYRO_X: {
                gyro_xyz[0] = esfMeas2Float(meas.data_field[i], 12);
            } break;
            default:
                break;
            }
        }

        if (lastAccTtag < meas.time_tag) { // Messages might arrive out of order -> discard old ones
            switch(meas.data_type[i]) {
            case ACC_X: {
                acc_xyz[0] = esfMeas2Float(meas.data_field[i], 10)/G;
                gotAcc = true;
            } break;
            case ACC_Y: {
                acc_xyz[1] = esfMeas2Float(meas.data_field[i], 10)/G;
            } break;
            case ACC_Z: {
                acc_xyz[2] = esfMeas2Float(meas.data_field[i], 10)/G;
            } break;
            default:
                break;
            }
        }
    }

    if (gotGyro) {
        lastGyroTtag = meas.time_tag;
        mVehicleState->setGyroscopeXYZ({gyro_xyz[0], gyro_xyz[1], gyro_xyz[2]});
    }

    if (gotAcc) {
        lastAccTtag = meas.time_tag;
        mVehicleState->setAccelerometerXYZ({acc_xyz[0], acc_xyz[1], acc_xyz[2]});
    }

    if (gotGyro && gotAcc) { // Update AHRS only when a new pair of measurements has arrived
        gotGyro = false;
        gotAcc = false;

        // Calibrate gyroscope
        FusionVector3 uncalibratedGyroscope = { {gyro_xyz[0], gyro_xyz[1], gyro_xyz[2]},};
        FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate accelerometer
        FusionVector3 uncalibratedAccelerometer = { {acc_xyz[0], acc_xyz[1], acc_xyz[2]},};
        FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

        // Update gyroscope bias correction algorithm
        calibratedGyroscope = FusionBiasUpdate(&mFusionBias, calibratedGyroscope);

        // Update AHRS algorithm
        FusionAhrsUpdateWithoutMagnetometer(&mFusionAhrs, calibratedGyroscope, calibratedAccelerometer, mIMUSamplePeriod);

        // Print Euler angles
        FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&mFusionAhrs));
        PosPoint tmppos = mVehicleState->getPosition(PosType::GNSS);
        tmppos.setRoll(eulerAngles.angle.roll);
        tmppos.setPitch(-eulerAngles.angle.pitch);
        float newYaw = -eulerAngles.angle.yaw+180.0f;
        while (newYaw < -180.0) // normalize
            newYaw += 360.0;
        while (newYaw >  180.0)
            newYaw -= 360.0;
        tmppos.setYaw(newYaw);
        mVehicleState->setPosition(tmppos);
    }
}

void UbloxRover::updateGNSS(const ubx_nav_pvt &pvt)
{
    //qDebug() << "Current enuRef:" << mEnuReference.latitude << mEnuReference.longitude << mEnuReference.height;
    llh_t llh = {pvt.lat, pvt.lon, pvt.height};
    xyz_t xyz = coordinateTransforms::llhToEnu(mEnuReference, llh);
    PosPoint gnssPos = mVehicleState->getPosition(PosType::GNSS);
    gnssPos.setX(xyz.x);
    gnssPos.setY(xyz.y);
    gnssPos.setHeight(xyz.z);

    // -- Only set if the receiver is in sensor fusion mode
    if(pvt.head_veh_valid) {
        gnssPos.setYaw(pvt.head_veh);
    }

    mVehicleState->setPosition(gnssPos);
}
