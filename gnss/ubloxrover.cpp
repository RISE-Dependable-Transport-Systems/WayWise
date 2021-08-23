#include "ubloxrover.h"
#include "sdvp_qtcommon/coordinatetransforms.h"
#include <QDebug>
#include <QDateTime>
#include <cmath>

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
    connect(&mUblox, &Ublox::rxNavPvt, this, &UbloxRover::updateGNSSPositionAndYaw);

    // Save-on-shutdown feature
    connect(&mUblox, &Ublox::rxUpdSos, this, &UbloxRover::updSosResponse);

    // Automatic IMU orientation alignment feature
    connect(&mUblox, &Ublox::rxEsfAlg, this, [this](const ubx_esf_alg &alg){
        if (alg.autoMntAlgOn)
            setIMUOrientationOffset(alg.roll, alg.pitch, alg.yaw);

    //        static int count = 0;
    //        if (count++%5)
    //            return;
    //        qDebug() << "ESF-ALG data:"
    //                 << "\nStatus:" << alg.status
    //                 << "\nAuto mount alignmend enabled:" << alg.autoMntAlgOn
    //                 << "\nRoll, Pitch, Yaw:" << alg.roll << alg.pitch << alg.yaw;
    });

    // Print nav-pvt message
    // Search for nav-pvt in following document:
    // https://www.u-blox.com/sites/default/files/ZED-F9R_Interfacedescription_UBX-19056845.pdf
//    connect(&mUblox, &Ublox::rxNavPvt, this, [](const ubx_nav_pvt &pvt){
//        static int count = 0;
//        if (count++%5)
//            return;
//        qDebug() << "NAV-PVT data:"
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
//                << "\nDifferential corrections applied:" << pvt.diffsoln;
//    });

    // Print esf-meas
//    connect(&mUblox, &Ublox::rxEsfMeas, this, [](const ubx_esf_meas &meas){
//        static int count = 0;
//        if (count++%100)
//            return;
//        qDebug() << "ESF-MEAS data:"
//                 << "\ntimeTag: " << meas.time_tag
//                 << "\ntimeMarkSent: " << meas.time_mark_sent
//                 << "\ntimeMarkEdge: " << meas.time_mark_edge
//                 << "\ncalibTtagValid:" << meas.calib_t_tag_valid
//                 << "\nnumMeas:" << meas.num_meas
//                 << "\nid:" << meas.id;
//        for (int i = 0;i < meas.num_meas;i++) {
//            qDebug() << "\ndataType:" << meas.data_type[i]
//                     << "dataField:" << bin << meas.data_field[i];
//        }
//        if (meas.calib_t_tag_valid)
//            qDebug() << "\ncalibTtag:" << meas.calib_t_tag;
//    });

    // Print esf-status message
    // Search for sensor data type in following document for explanation:
    // https://www.u-blox.com/sites/default/files/ZED-F9R_Integrationmanual_UBX-20039643.pdf
//    connect(&mUblox, &Ublox::rxEsfStatus, this, [](const ubx_esf_status &status){
//        static int count = 0;
//        if (count++%5)
//            return;
//        qDebug() << "ESF-STATUS data:"
//                 << "\nVersion: " << status.version
//                 << "\nFusion mode: " << status.fusion_mode
//                 << "\nNumber of sensors: " << status.num_sens;
//                    ;
//        for (int i = 0;i < status.num_sens;i++) {
//            qDebug() << "Sensor data type: " << status.sensors[i].type
//                     << "\nSensor data used: " << status.sensors[i].used
//                     << "\nSensor data ready: " << status.sensors[i].ready
//                     << "\nSensor calibration status: " << status.sensors[i].calib_status
//                     << "\nSensor time status: " << status.sensors[i].time_status
//                     << "\nSensor observation freq: " << status.sensors[i].freq
//                     ;
//        }});

    // Print config recevied from valget
//    connect(&mUblox, &Ublox::rxCfgValget, this, [](const ubx_cfg_valget &valget){
//        qDebug() << "\n--- Polled UBX-CFG-VALGET ---"
//                 << "\nVersion:" << valget.version
//                 << "\nLayer:" << valget.layer
//                 << "\nPosition:" << valget.position;

//        for (int i=0; i<int(sizeof(valget.cfgData)/sizeof(*valget.cfgData)); i++) {
//            if (valget.key[i] != 0) {
//                qDebug() << "Key:" << hex << valget.key[i]
//                            << "Config:" << hex << valget.cfgData[i];
//            }
//        }
//        qDebug() << "--- End of poll ---\n";
//    });
}

bool UbloxRover::connectSerial(const QSerialPortInfo &serialPortInfo)
{
    if (mUblox.connectSerial(serialPortInfo)) {
        if (configureUblox())
            return true;
        else {
            mUblox.disconnectSerial();
            return false;
        }
    } else
        return false;
}

bool UbloxRover::isSerialConnected()
{
    return mUblox.isSerialConnected();
}

void UbloxRover::setEnuRef(llh_t enuRef)
{
    mEnuReference = enuRef;
    mEnuReferenceSet = true;
}

void UbloxRover::writeRtcmToUblox(QByteArray data)
{
    mUblox.writeRaw(data);
}

void UbloxRover::writeOdoToUblox(ubx_esf_datatype_enum dataType, uint32_t dataField)
{
    mUblox.ubloxOdometerInput(dataType, dataField);
}

void UbloxRover::setEnableIMUOrientationUpdate(bool enabled)
{
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_MEAS, (enabled ? 1 : 0)); // TODO: disable in fusion mode and use UBX-NAV-ATT instead
}

void UbloxRover::setIMUOrientationOffset(double roll_deg, double pitch_deg, double yaw_deg)
{
    mIMUOrientationOffset = {roll_deg, pitch_deg, yaw_deg};

    float sin_roll = sinf(roll_deg * M_PI / 180.0f);
    float cos_roll = cosf(roll_deg * M_PI / 180.0f);
    float sin_pitch = sinf(pitch_deg * M_PI / 180.0f);
    float cos_pitch = cosf(pitch_deg * M_PI / 180.0f);
    float sin_yaw = sinf(yaw_deg * M_PI / 180.0f);
    float cos_yaw = cosf(yaw_deg * M_PI / 180.0f);

    mIMUOrientationOffsetMatrix = {.array = {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - cos_roll * sin_yaw, sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch,
                                             cos_pitch * sin_yaw, cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll, cos_roll * sin_yaw * sin_pitch - cos_yaw * sin_roll,
                                             -sin_pitch,          cos_pitch * sin_roll,                                cos_pitch * cos_roll                               }};
}

bool UbloxRover::configureUblox()
{
    // The u-blox receiver detects the previously stored data in the flash.
    // It restores the corresponding memory and reports the success of the operation
    mUblox.ubloxUpdSos(7);

    // The rate of NMEA and UBX protocol output messages are configurable
    // and it is possible to enable or disable single NMEA or UBX messages individually.
    // If the rate configuration value is zero, then the corresponding message will not be output.
    // Values greater than zero indicate how often the message is output.
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_MEAS, 1);
    mUblox.ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_PVT, 1);
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_STATUS, 1);
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_ALG, 1);

    unsigned char buffer[512];
    int ind = 0;
    mUblox.ubloxCfgAppendMntalg(buffer, &ind, true); // enable auto mount alignment
    mUblox.ubloxCfgAppendRate(buffer, &ind, 100, 1, 0, 30); // nav prio mode
    mUblox.ubloxCfgValset(buffer, ind, true, true, true);

    //mUblox.ubloxCfgValset(unsigned char *values, int len, bool ram, bool bbr, bool flash);
//        unsigned char buffer[512];
//        int ind = 0;
//        mUblox.ubloxCfgAppendEnableGps(buffer, &ind, true, true, true);
//        mUblox.ubloxCfgAppendEnableGal(buffer, &ind, true, true, true);
//        mUblox.ubloxCfgAppendEnableBds(buffer, &ind, true, true, true);
//        mUblox.ubloxCfgAppendEnableGlo(buffer, &ind, true, true, true);
//        mUblox.ubloxCfgValset(buffer, ind, true, true, true);

    // Poll request UBX-CFG-VALGET
    // This message is limited to containing a maximum of 64 key-value pairs
//    unsigned char buffer[64];
//    int ind = 0;
//    uint8_t layer = 0;
//    mUblox.ubloxCfgAppendKey(buffer, &ind, CFG_SFIMU_AUTO_MNTALG_ENA);
//    mUblox.ubloxCfgAppendKey(buffer, &ind, CFG_SIGNAL_GPS_ENA);
//    mUblox.ubloxCfgAppendKey(buffer, &ind, CFG_SIGNAL_GAL_ENA);
//    mUblox.ubloxCfgValget(buffer, ind, layer);

//    qDebug() << "--- Poll request UBX-CFG-VALGET ---"
//             << "\nCFG_SFIMU_AUTO_MNTALG_ENA:" << hex <<  CFG_SFIMU_AUTO_MNTALG_ENA
//             << "\nCFG_SIGNAL_GPS_ENA:" << hex << CFG_SIGNAL_GPS_ENA
//             << "\nCFG_SIGNAL_GAL_ENA:" << hex << CFG_SIGNAL_GAL_ENA
//             << "\nLayer:" << layer
//             << "\n--- End of poll request ---\n";
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

    if (gotGyro && gotAcc) { // Update AHRS only when a new pair of measurements has arrived. TODO: how to handle faster updates in one compared to the other?
        gotGyro = false;
        gotAcc = false;

        // Calibrate gyroscope
        FusionVector3 uncalibratedGyroscope = { {gyro_xyz[0], gyro_xyz[1], gyro_xyz[2]},};
        FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, mIMUOrientationOffsetMatrix, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate accelerometer
        FusionVector3 uncalibratedAccelerometer = { {acc_xyz[0], acc_xyz[1], acc_xyz[2]},};
        FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, mIMUOrientationOffsetMatrix, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

        // Update gyroscope bias correction algorithm
        calibratedGyroscope = FusionBiasUpdate(&mFusionBias, calibratedGyroscope);

        // Update AHRS algorithm
        FusionAhrsUpdateWithoutMagnetometer(&mFusionAhrs, calibratedGyroscope, calibratedAccelerometer, mIMUSamplePeriod);

        // Get Euler angles
        FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&mFusionAhrs));
        PosPoint tmppos = mVehicleState->getPosition(PosType::IMU);
        tmppos.setRoll(eulerAngles.angle.roll);
        tmppos.setPitch(-eulerAngles.angle.pitch);
        float newYaw = -eulerAngles.angle.yaw+180.0f;
        while (newYaw < -180.0) // normalize
            newYaw += 360.0;
        while (newYaw >  180.0)
            newYaw -= 360.0;
        tmppos.setYaw(newYaw);

        tmppos.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()).msecsSinceStartOfDay()); // TODO: can meas.time_tag be mapped to UTC/iTOW?
        mVehicleState->setPosition(tmppos);

        emit updatedIMUOrientation(mVehicleState);
    }
}

void UbloxRover::updateGNSSPositionAndYaw(const ubx_nav_pvt &pvt)
{
    static bool initializationDone = false;
    static int leapSeconds_ms = 0;

    if (!initializationDone && pvt.valid_time) {
        leapSeconds_ms =  (pvt.i_tow % ms_per_day) - ((pvt.hour * 60 * 60 + pvt.min * 60 + pvt.second) * 1000);
        qDebug() << "UbloxRover: assuming" << leapSeconds_ms / 1000 << "seconds difference between GNSS time and UTC.";
        initializationDone = true;
    } else {
        PosPoint gnssPos = mVehicleState->getPosition(PosType::GNSS);

        llh_t llh = {pvt.lat, pvt.lon, pvt.height};
        xyz_t xyz = {0.0, 0.0, 0.0};

        if (!mEnuReferenceSet) {
            mEnuReference = llh;
            mEnuReferenceSet = true;
        } else
            xyz = coordinateTransforms::llhToEnu(mEnuReference, llh);

        // Position
        gnssPos.setX(xyz.x);
        gnssPos.setY(xyz.y);
        gnssPos.setHeight(xyz.z);

        // Yaw --- based on last GNSS position if fusion (F9R) unavailable
        static xyz_t lastXyz;
        if(pvt.head_veh_valid)
            gnssPos.setYaw(pvt.head_veh + mIMUOrientationOffset.yawOffset_deg);
        else
            gnssPos.setYaw(-atan2(xyz.y - lastXyz.y, xyz.x - lastXyz.x) * 180.0 / M_PI);

        // Time and speed
        gnssPos.setTime((pvt.i_tow % ms_per_day) - leapSeconds_ms);
//        qDebug() << "UbloxRover, gnssPos.getTime() - msSinceTodayUTC:" << QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()).msecsSinceStartOfDay() - gnssPos.getTime();
        gnssPos.setSpeed(pvt.g_speed);

        mVehicleState->setPosition(gnssPos);
        lastXyz = xyz;

        emit updatedGNSSPositionAndYaw(mVehicleState, pvt.head_veh_valid);
    }
}

void UbloxRover::updSosResponse(const ubx_upd_sos &sos)
{
    if(sos.cmd == 2){
        qDebug() << "\n--- Response from UBX-UPD-SOS ---"
                 << "\nBackup creation acknowledge"
                 << "\n0 = Not acknowledged"
                 << "\n1 = Acknowledged"
                 << "\nResponse:" << sos.response
                 << "\n--- End of response --\n";
    } else if(sos.cmd == 3) {
        qDebug() << "\n--- Response from UBX-UPD-SOS ---"
                 << "\nSystem restored from backup"
                 << "\n0 = Unknown"
                 << "\n1 = Failed restoring from backup"
                 << "\n2 = Restored from backup"
                 << "\n3 = Not restored (no backup)"
                 << "\nResponse:" << sos.response
                 << "\n--- End of response --\n";
        if(sos.response == 2) {
            // Once the u-blox receiver has started up it is recommended to delete the stored data
            mUblox.ubloxUpdSos(1);
        }
    }
}

void UbloxRover::saveOnShutdown()
{
    // With the UBX-CFG-RST message, the host commands the u-blox receiver to stop, specifying
    // a BBR mask 0 ("Hotstart") and a reset mode of 0x08 ("Controlled GNSS stop")
    mUblox.ubxCfgRst(8, 0);

    // The host commands the saving of the contents of BBR to the flash memory using the UBX-
    // UPD-SOS-BACKUP message
    mUblox.ubloxUpdSos(0);
}
