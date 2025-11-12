/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "ubloxrover.h"
#include <QDebug>
#include <QDateTime>
#include <QLineF>
#include <cmath>

UbloxRover::UbloxRover(QSharedPointer<VehicleState> vehicleState)
    : GNSSReceiver(vehicleState)
{
    // Use GNSS reception to update location
    connect(&mUblox, &Ublox::rxNavPvt, this, [this](const ubx_nav_pvt &pvt){
        if (mReceiverVariant == RECEIVER_VARIANT::UBLX_ZED_F9P && mReceiverState != RECEIVER_STATE::READY) {
            setReceiverState(RECEIVER_STATE::READY);
        }
        updateGNSSPositionAndYaw(pvt);
    });

    // Fordward received NMEA GGA messages
    connect(&mUblox, &Ublox::rxNmeaGga, this, &UbloxRover::gotNmeaGga);

    // Automatic IMU orientation alignment feature
    connect(&mUblox, &Ublox::rxEsfAlg, this, [this](const ubx_esf_alg &alg){
        if (alg.autoMntAlgOn)
            setChipOrientationOffset(alg.roll, alg.pitch, alg.yaw);
        if (mPrintVerbose){
            qDebug() << "---------------------------------";
            qDebug() << "ESF-ALG data:"
                     << "\nAuto mount alignmend enabled:" << alg.autoMntAlgOn
                     << "\nStatus:" << Ublox::getAutoMntAlgStatusText(alg.status)
                     << "\n roll/pitch angle error:" << alg.tiltAlgError
                     << "\n yaw angle error:" << alg.yawAlgError
                     << "\n gimbal-lock error:" << alg.angleError
                     << "\nRoll, Pitch, Yaw:" << alg.roll << alg.pitch << alg.yaw;
        }
    });

    // Print esf-status message
    // Search for sensor data type in following document for explanation:
    // https://www.u-blox.com/sites/default/files/ZED-F9R_Integrationmanual_UBX-20039643.pdf
    connect(&mUblox, &Ublox::rxEsfStatus, this, [this](const ubx_esf_status &status) {
        switch (mReceiverState)
        {
            case RECEIVER_STATE::CONFIGURED:
            {
                if (status.fusion_mode == 0) {
                    setReceiverState(RECEIVER_STATE::CALIBRATING);
                    qDebug() << "UbloxRover: Sensors are calibrating";
                } else if (status.fusion_mode == 1) {
                    setReceiverState(RECEIVER_STATE::READY);
                    qDebug() << "UbloxRover: Sensors are precalibratied. Fusion mode is enabled";
                    switchNavPrioMode(true);
                }
            } break;
            case RECEIVER_STATE::CALIBRATING:
            {
                if (status.fusion_mode == 1) {
                    setReceiverState(RECEIVER_STATE::READY);
                    qDebug() << "UbloxRover: Sensors are calibrated. Fusion mode is enabled";
                    switchNavPrioMode(true);
                    mCreateBackupWithSoS = true;

                    // Once the u-blox receiver has calibrated clear the stored backup data in flash
                    mUblox.ubloxUpdSos(1);
                }
            } break;
            case RECEIVER_STATE::READY:
            {
                if (status.fusion_mode == 0) { // Recalibrating sensors
                    setReceiverState(RECEIVER_STATE::CALIBRATING);
                    qDebug() << "UbloxRover: Recalibrating sensors";
                }
            } break;
            default:
                break;
        }

        if (mPrintVerbose) {
            qDebug() << "---------------------------------";
            qDebug() << "ESF-STATUS data:"
                    // << "\nVersion:" << status.version
                    << "\nFusion mode:" << Ublox::getFusionModeText(status.fusion_mode)
                    // << "\nNumber of sensors:" << status.num_sens
                    << "\nIMU initialization status:" << Ublox::getInitStatusText(status.imuInitStatus)
                    << "\nMount alignment status:" << Ublox::getInitStatusText(status.mntAlgStatus)
                    << "\nWheel tick status:" << Ublox::getInitStatusText(status.wtInitStatus)
                    << "\nINS initialization status:" << Ublox::getInitStatusText(status.insInitStatus);

            qDebug() << "\nSensor Data:";
            for (int i = 0; i < status.num_sens; i++) {
                const auto &sensor = status.sensors[i];
                if (sensor.calib_status != 2 && sensor.calib_status != 3) {
                    QString sensorStr = QString("Sensor #%1: Type: %2 | Used in fusion: %3 | Ready: %4 | Calibration: %5 | Time Status: %6 | Frequency: %7 Hz | Faults: %8")
                        .arg(i + 1)
                        .arg(Ublox::getSensorTypeText(sensor.type).leftJustified(30, ' '))
                        .arg(QString(sensor.used ? "Yes" : "No").leftJustified(3, ' '))
                        .arg(QString(sensor.ready ? "Yes" : "No").leftJustified(3, ' '))
                        .arg(Ublox::getCalibStatusText(sensor.calib_status).leftJustified(15, ' '))
                        .arg(Ublox::getTimeStatusText(sensor.time_status).leftJustified(15, ' '))
                        .arg(sensor.freq, 4)  // pads the number to 4 characters
                        .arg(Ublox::getFaultsText(sensor.bad_meas, sensor.bad_t_tag, sensor.missing_meas, sensor.noisy_meas).leftJustified(25, ' '));

                    qDebug() << sensorStr;
                }
            }
            qDebug() << "---------------------------------";
        }
    });

    // Print nav-status message
    connect(&mUblox, &Ublox::rxNavStatus, this, [this](const ubx_nav_status &status) {
        mFixType = static_cast<GNSS_FIX_TYPE>(status.gps_fix);
        if (mPrintVerbose && mReceiverState == RECEIVER_STATE::CALIBRATING) {
            qDebug() << "---------------------------------";
            qDebug() << "NAV-STATUS data:"
                    //  << "\nGPS Time of Week (iTOW):" << status.i_tow
                    << "\nGPS Fix Type:" << Ublox::getGpsFixTypeText(status.gps_fix)
                    << "\nGNSS Fix OK:" << (status.gnss_fix_ok ? "Yes" : "No")
                    << "\nDifferential Solution Applied:" << (status.diffSoln ? "Yes" : "No")
                    //  << "\nWeek Number Valid:" << (status.wknSet ? "Yes" : "No")
                    //  << "\nTime of Week Valid:" << (status.towSet ? "Yes" : "No")
                    << "\nDifferential Corrections Available:" << (status.diffCorr ? "Yes" : "No")
                    //  << "\nCarrier Phase Solution Valid:" << (status.carrSolnValid ? "Yes" : "No")
                    //  << "\nMap Matching Status:" << Ublox::getMapMatchingStatusText(status.mapMatching)
                    //  << "\nPower Save Mode State:" << Ublox::getPsmStateText(status.psmState)
                    //  << "\nSpoofing Detection State:" << Ublox::getSpoofDetStateText(status.spoofDetState)
                    //  << "\nCarrier Phase Solution:" << Ublox::getCarrSolnText(status.carrSoln)
                    //  << "\nTime to First Fix (TTFF):" << status.ttff << "ms"
                    //  << "\nMilliseconds Since Startup/Reset (MSSS):" << status.msss << "ms"
                    << "\n---------------------------------";
        }
    });

    // Print nav-pvt message
    // Search for nav-pvt in following document:
    // https://www.u-blox.com/sites/default/files/ZED-F9R_Interfacedescription_UBX-19056845.pdf
    // connect(&mUblox, &Ublox::rxNavPvt, this, [](const ubx_nav_pvt &pvt){
    //     static int count = 0;
    //     if (count++%5)
    //         return;
    //     qDebug() << "\nNAV-PVT data:"
    //             << "\nDate: " << pvt.year << pvt.month << pvt.day
    //             << "\nTime: " << pvt.hour << pvt.min << pvt.second
    //             << "\nFix Type: " << pvt.fix_type
    //             << "\nGNSS valid fix: " << pvt.gnss_fix_ok
    //             << "\nHeading valid: " << pvt.head_veh_valid
    //             << "\nNumber of satelites used: " << pvt.num_sv
    //             << "\nLongitude:" << pvt.lon << "Latitude:" << pvt.lat << "Height:" << pvt.height
    //             << "\nGround speed: " << pvt.g_speed << "m/s"
    //             << "\nHeading of motion: " << pvt.head_mot
    //             << "\nHeading of vehicle: " << pvt.head_veh
    //             << "\nDifferential corrections applied:" << pvt.diffsoln
    //             << "\nCarrier phase range solution status:" << pvt.carr_soln;
    // });

    // // Print esf-meas
    // connect(&mUblox, &Ublox::rxEsfMeas, this, [](const ubx_esf_meas &meas){
    //     static int count = 0;
    //     if (count++%10)
    //         return;
    //     qDebug() << "ESF-MEAS data:"
    //                 << "\ntimeTag: " << meas.time_tag
    //                 << "\ntimeMarkSent: " << meas.time_mark_sent
    //                 << "\ntimeMarkEdge: " << meas.time_mark_edge
    //                 << "\ncalibTtagValid:" << meas.calib_t_tag_valid
    //                 << "\nnumMeas:" << meas.num_meas
    //                 << "\nid:" << meas.id;
    //     for (int i = 0;i < meas.num_meas;i++) {
    //         qDebug() << "\ndataType:" << meas.data_type[i]
    //                     << "dataField:" << bin << meas.data_field[i];
    //     }
    //     if (meas.calib_t_tag_valid)
    //         qDebug() << "\ncalibTtag:" << meas.calib_t_tag;
    // });

    // Print config recevied from valget
    // connect(&mUblox, &Ublox::rxCfgValget, this, [](const ubx_cfg_valget &valget){
    //     qDebug() << "\n--- Polled UBX-CFG-VALGET ---"
    //                 << "\nVersion:" << valget.version
    //                 << "\nLayer:" << valget.layer
    //                 << "\nPosition:" << valget.position;

    //     for (int i=0; i<int(sizeof(valget.cfgData)/sizeof(*valget.cfgData)); i++) {
    //         if (valget.key[i] != 0) {
    //             qDebug() << "Key:" << hex << valget.key[i]
    //                         << "Config:" << hex << valget.cfgData[i];
    //         }
    //     }
    //     qDebug() << "--- End of poll ---\n";
    // });
}

bool UbloxRover::connectSerial(const QSerialPortInfo &serialPortInfo)
{
    if (!mUblox.connectSerial(serialPortInfo)) {
        setReceiverState(RECEIVER_STATE::DISCONNECTED);
        return false; // Failed to connect to the serial port
    }

    setReceiverState(RECEIVER_STATE::CONNECTED);

    // Poll the receiver to restore the backup configuration with calibration parameters (blocking call)
    restoreBackedupConfiguration();

    if (mReceiverState == RECEIVER_STATE::UNKNOWN || !configureUblox()) {
        shutdownGNSSReceiver();
        return false;
    }

    return true; // Successfully connected and configured
}

bool UbloxRover::isSerialConnected()
{
    return mUblox.isSerialConnected();
}

void UbloxRover::writeRtcmToUblox(QByteArray data)
{
    mUblox.writeRaw(data);
}

void UbloxRover::writeOdomToUblox(ubx_esf_datatype_enum dataType, uint32_t dataField, uint32_t timeTag)
{
    mUblox.ubloxOdometerInput(dataType, dataField, timeTag);
}

void UbloxRover::readVehicleSpeedForPositionFusion()
{
    int32_t speed = static_cast<int32_t>(mVehicleState->getSpeed() * 1000.0);  // m/s to mm/s
    speed &= 0x00FFFFFF;  // Bits 31..23 are set to zero
    uint32_t timeTag = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    writeOdomToUblox(SPEED, speed, timeTag);
}

bool UbloxRover::configureUblox()
{
    bool result = true;
    unsigned char buffer[4096];
    int ind = 0;

    mUblox.ubloxCfgAppendEnableGps(buffer, &ind, true, true, true);
    mUblox.ubloxCfgAppendEnableGal(buffer, &ind, true, true, true);
    mUblox.ubloxCfgAppendEnableBds(buffer, &ind, true, true, true);
    mUblox.ubloxCfgAppendEnableGlo(buffer, &ind, true, true, true);

    mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_NAVSPG_DYNMODEL, mDynamicModel); // Set Dynmaic model
    // Set the output rates for the messages
    mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_NMEA_ID_GGA_USB, 1); // NMEA-GGA rate

    if (mReceiverVariant == RECEIVER_VARIANT::UBLX_ZED_F9R) {
        // F9R specific configuration

        if (mCalibrateEsfSensors) { // TODO: do a valget to compare the key values and change them if required
            mUblox.ubloxCfgAppendEnableSf(buffer, &ind, true); // enable/disable sensor fusion

            // Set the odometer input configuration for sensor fusion
            mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_SFODO_USE_SPEED, true); // Use speed data for fusion, speed should be sent as mm/s, signed int.
            mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_SFODO_USE_WT_PIN, false); // Disable wheel tick input
            mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_SFODO_FREQUENCY, mSpeedDataInputRate); // Speed data frequency (Hz), should be accurate upto 10%, set it to 0 for automatic estimation
            // mUblox.ubloxCfgAppendU2Key(buffer, &ind, CFG_SFODO_LATENCY, 5); // Wheel tick/speed data latency (in ms), if not provided, it is assumed to be 0.
            mUblox.ubloxCfgAppendU4Key(buffer, &ind, CFG_SFODO_QUANT_ERROR, 1e3); // Wheel tick/speed data quantization error (units: 1e-6), set it to 0 for automatic estimation, 1e3 = 1mm/s

            // Automatic IMU mount alignment is not supported for MOWER and E-Scooter models
            uint32_t yaw_deg_scaled = static_cast<uint32_t>(mAChipOrientationOffset.yawOffset_deg * 100);
            uint16_t pitch_deg_scaled = static_cast<uint16_t>(mAChipOrientationOffset.pitchOffset_deg * 100);
            uint16_t roll_deg_scaled = static_cast<uint16_t>(mAChipOrientationOffset.rollOffset_deg * 100);
            mUblox.ubloxCfgAppendMntalg(buffer, &ind, mESFAlgAutoMntAlgOn, yaw_deg_scaled, pitch_deg_scaled, roll_deg_scaled); // IMU mount alignment, in degrees with a scale of 1/1e-2

            // Set the offset between the IMU and the antenna
            uint16_t imu2ant_la_x_cm = static_cast<uint16_t>(-mAntennaToChipOffset.x * 100);
            uint16_t imu2ant_la_y_cm = static_cast<uint16_t>(-mAntennaToChipOffset.y * 100);
            uint16_t imu2ant_la_z_cm = static_cast<uint16_t>(-mAntennaToChipOffset.z * 100);
            mUblox.ubloxCfgAppendI2Key(buffer, &ind, CFG_SFIMU_IMU2ANT_LA_X, imu2ant_la_x_cm); // IMU to antenna lever arm X (cm)
            mUblox.ubloxCfgAppendI2Key(buffer, &ind, CFG_SFIMU_IMU2ANT_LA_Y, imu2ant_la_y_cm); // IMU to antenna lever arm Y (cm)
            mUblox.ubloxCfgAppendI2Key(buffer, &ind, CFG_SFIMU_IMU2ANT_LA_Z, imu2ant_la_z_cm); // IMU to antenna lever arm Z (cm)

            // Set the offset between the IMU and the vehicle reference point
            uint16_t imu2vrp_la_x_cm = static_cast<uint16_t>(mChipToRearAxleOffset.x * 100);
            uint16_t imu2vrp_la_y_cm = static_cast<uint16_t>(mChipToRearAxleOffset.y * 100);
            uint16_t imu2vrp_la_z_cm = static_cast<uint16_t>(mChipToRearAxleOffset.z * 100);
            mUblox.ubloxCfgAppendI2Key(buffer, &ind, CFG_SFODO_IMU2VRP_LA_X, imu2vrp_la_x_cm); // IMU to vehicle reference point lever arm X (cm)
            mUblox.ubloxCfgAppendI2Key(buffer, &ind, CFG_SFODO_IMU2VRP_LA_Y, imu2vrp_la_y_cm); // IMU to vehicle reference point lever arm Y (cm)
            mUblox.ubloxCfgAppendI2Key(buffer, &ind, CFG_SFODO_IMU2VRP_LA_Z, imu2vrp_la_z_cm); // IMU to vehicle reference point lever arm Z (cm)
        }

        // Set the output rates for the messages
        mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_UBX_ESF_STATUS_USB, 1); // UBX-ESF-STATUS rate

        // Disable nav prio mode by default at startup, it will be enabled when the ESF sensors are calibrated
        mUblox.ubloxCfgAppendRate(buffer, &ind, 1000, 1, 0, 0); // Disable nav prio mode
        mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_UBX_NAV_PVT_USB, 0); // UBX-NAV-PVT rate

        // if (mPrintVerbose) { // TODO: handle this after configuration is done by writing to just ram and bbr
        //     // mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_UBX_ESF_MEAS_USB, 1); // UBX-ESF-MEAS rate
        //     mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_UBX_NAV_STATUS_USB, 1); // UBX-NAV-STATUS rate
        // }

        if (mESFAlgAutoMntAlgOn) {
            mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_UBX_ESF_ALG_USB, 1); // UBX-ESF-ALG rate
        }

        result &= mUblox.ubloxCfgValset(buffer, ind, true, true, mCalibrateEsfSensors, 0); // timeout set to 0 as ack is not being received

        if (result)
            qDebug() << "UbloxRover: F9R was successfully configured with " << Ublox::getDynamicModelName(mDynamicModel) << " dynamic model";
        else
            qDebug() << "UbloxRover: F9R configuration reported an error";

    } else if (mReceiverVariant == RECEIVER_VARIANT::UBLX_ZED_F9P) {
        // F9P specific configuration

        int gNSSMeasurementPeriod_ms = 1000 / mGNSSMeasurementRate; // convert Hz to ms
        mUblox.ubloxCfgAppendRate(buffer, &ind, gNSSMeasurementPeriod_ms, 1, 0); // Set rate
        mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_UBX_NAV_PVT_USB, 1); // Enable UBX-NAV-PVT

        result &= mUblox.ubloxCfgValset(buffer, ind, true, true, mCalibrateEsfSensors, 0); // timeout set to 0 as ack is not being received

        qDebug() << "UbloxRover: F9P configuration" << (result ? "was successful" : "reported an error");
        mCreateBackupWithSoS = true;
    }

    setReceiverState(RECEIVER_STATE::CONFIGURED);
    return true;
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
            setEnuRef(llh);
            qDebug() << "UbloxRover: ENU reference point set to" << pvt.lat << pvt.lon << pvt.height;
        } else
            xyz = coordinateTransforms::llhToEnu(mEnuReference, llh);

        // Position
        gnssPos.setXYZ(xyz);

        double vehYaw_radENU = 0.0;
        if (mReceiverVariant == RECEIVER_VARIANT::UBLX_ZED_F9R) {
            double yaw_degENU = coordinateTransforms::yawNEDtoENU(pvt.head_veh) + mAChipOrientationOffset.yawOffset_deg;

            // normalize to [-180.0:180.0]
            while (yaw_degENU < -180.0)
                yaw_degENU += 360.0;
            while (yaw_degENU >= 180.0)
                yaw_degENU -= 360.0;

            gnssPos.setYaw(yaw_degENU);

            vehYaw_radENU = yaw_degENU * M_PI / 180.0;

            // Apply Chip to rear axle offset if set.
            if (mChipToRearAxleOffset.x != 0.0 || mChipToRearAxleOffset.y != 0.0) {
                gnssPos.updateWithOffsetAndYawRotation(mChipToRearAxleOffset, vehYaw_radENU);
            }
        } else { // Assumes fused yaw is updated.
            PosPoint fusedPos = mVehicleState->getPosition(PosType::fused);
            vehYaw_radENU = fusedPos.getYaw() * M_PI / 180.0;

            // Apply antenna to rear axle offset if set.
            xyz_t mAntennaToRearAxleOffset = mAntennaToChipOffset + mChipToRearAxleOffset;
            if (mAntennaToRearAxleOffset.x != 0.0 || mAntennaToRearAxleOffset.y != 0.0) {
                gnssPos.updateWithOffsetAndYawRotation(mAntennaToRearAxleOffset, vehYaw_radENU);
            }
        }

        // Time and speed
        gnssPos.setTime(QTime::fromMSecsSinceStartOfDay((pvt.i_tow % ms_per_day) - leapSeconds_ms));
//        qDebug() << "UbloxRover, gnssPos.getTime() - msSinceTodayUTC:" << QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()).msecsSinceStartOfDay() - gnssPos.getTime();
        gnssPos.setSpeed(pvt.g_speed);

        mVehicleState->setPosition(gnssPos);

        // Accuracy
        mGnssFixAccuracy.horizontal = pvt.h_acc;
        mGnssFixAccuracy.vertical = pvt.v_acc;
        mGnssFixAccuracy.heading = pvt.head_acc;

        static xyz_t lastXyz;
        emit updatedGNSSPositionAndYaw(mVehicleState, QLineF(QPointF(lastXyz.x, lastXyz.y), gnssPos.getPoint()).length(), pvt.head_veh_valid);
        emit txNavPvt(pvt);

        lastXyz = xyz;
    }
}

void UbloxRover::restoreBackedupConfiguration(int pollIntervalms, int maxPolls)
{
    QTimer pollTimer;
    pollTimer.setInterval(pollIntervalms);

    QEventLoop loop;
    int polls = 0;

    int8_t restore_bkup_response = -1;

    // Connect the rxMonVer signal to process the version information
    auto conn = QObject::connect(&mUblox, &Ublox::rxUpdSos, [this, &loop, &restore_bkup_response]
        (const ubx_upd_sos &sos) {
            if(sos.cmd == 3) {
                qDebug() << "\n--- Response from UBX-UPD-SOS ---"
                        << "\nSystem restored from backup"
                        << "\n0 = Unknown"
                        << "\n1 = Failed restoring from backup"
                        << "\n2 = Restored from backup"
                        << "\n3 = Not restored (no backup)"
                        << "\nResponse:" << sos.response
                        << "\n--- End of response --\n";
                restore_bkup_response = sos.response;
                loop.quit();
            }
    });

    // The u-blox receiver detects the previously stored data in the flash.
    // It restores the corresponding memory and reports the success of the operation
    bool result = mUblox.ubloxUpdSos(7);
    qDebug() << "UbloxRover: Backup restore request" << (result ? "was sent" : "reported an error");

    QObject::connect(&pollTimer, &QTimer::timeout, [&]() {
        if (restore_bkup_response != -1 || polls++ >= maxPolls) {
            pollTimer.stop();
            loop.quit();  // Exit the event loop
        } else {
            mUblox.ubxPoll(UBX_CLASS_UPD, UBX_UPD_SOS);
            qDebug() << "UbloxRover: Polling UBX-UPD-SOS status";
        }
    });

    pollTimer.start();
    loop.exec();
    disconnect(conn);

    if (restore_bkup_response == 2) {
        setReceiverState(RECEIVER_STATE::BACKUP_RESTORED);
    } else if (restore_bkup_response == 3) {
        setReceiverState(RECEIVER_STATE::BACKUP_NOT_FOUND);
        if (mReceiverVariant == RECEIVER_VARIANT::UBLX_ZED_F9R) {
            mCalibrateEsfSensors = true;
        }
    } else if (restore_bkup_response == -1) {
        setReceiverState(RECEIVER_STATE::UNKNOWN);
        qDebug() << "UbloxRover: Non responsive while waiting for backup restore";
    }
}

void UbloxRover::createConfigurationBackup(int pollIntervalms, int maxPolls)
{
    QTimer pollTimer;
    pollTimer.setInterval(pollIntervalms);

    QEventLoop loop;
    int polls = 0;

    int8_t create_bkup_response = -1;

    // Connect the rxMonVer signal to process the version information
    auto conn = QObject::connect(&mUblox, &Ublox::rxUpdSos, [this, &loop, &create_bkup_response]
        (const ubx_upd_sos &sos) {
            if(sos.cmd == 2){
                qDebug() << "\n--- Response from UBX-UPD-SOS ---"
                         << "\nBackup creation acknowledge"
                         << "\n0 = Not acknowledged"
                         << "\n1 = Acknowledged"
                         << "\nResponse:" << sos.response
                         << "\n--- End of response --\n";
                create_bkup_response = sos.response;
                loop.quit();
            }
    });

    // The host commands the saving of the contents of BBR to the flash memory using the UBX-
    // UPD-SOS-BACKUP message
    bool result = mUblox.ubloxUpdSos(0);
    qDebug() << "UbloxRover: Backup creation request" << (result ? "was sent" : "failed");

    QObject::connect(&pollTimer, &QTimer::timeout, [&]() {
        if (create_bkup_response != -1 || polls++ >= maxPolls) {
            pollTimer.stop();
            loop.quit();  // Exit the event loop
        } else {
            qDebug() << "UbloxRover: Waiting for backup to complete...";
        }
    });

    pollTimer.start();
    loop.exec();
    disconnect(conn);

    if (create_bkup_response == 2) {
        setReceiverState(RECEIVER_STATE::BACKUP_CREATED);
    }
}

bool UbloxRover::switchNavPrioMode(bool navPrioMode)
{

    bool result = true;
    unsigned char buffer[4096];
    int ind = 0;

    if (navPrioMode) {
        int gNSSMeasurementPeriod_ms = 1000 / mGNSSMeasurementRate; // convert Hz to ms
        mUblox.ubloxCfgAppendRate(buffer, &ind, gNSSMeasurementPeriod_ms, 1, 0, mNavPrioMessageRate); // Enable/disable nav prio mode
        mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_UBX_NAV_PVT_USB, 1); // UBX-NAV-PVT rate
    } else {
        mUblox.ubloxCfgAppendRate(buffer, &ind, 1000, 1, 0, 0); // Disable nav prio mode
        mUblox.ubloxCfgAppendE1U1Key(buffer, &ind, CFG_MSGOUT_UBX_NAV_PVT_USB, 0); // UBX-NAV-PVT rate
    }

    result &= mUblox.ubloxCfgValset(buffer, ind, true, true, false, 0); // timeout set to 0 as ack is not being received

    qDebug() << "UbloxRover: Nav prio mode" << (result ? "was turned" : "could not be turned") << (navPrioMode ? "on" : "off");

    return result;
}

void UbloxRover::aboutToShutdown()
{
    if (mUblox.isSerialConnected()) {
        if (mReceiverVariant == RECEIVER_VARIANT::UBLX_ZED_F9R) {
            switchNavPrioMode(false);
        }
        switch (mReceiverState)
        {
            case RECEIVER_STATE::READY:
            {
                if (mCreateBackupWithSoS) {
                    setReceiverState(RECEIVER_STATE::BACKUP_ONGOING);

                    // With the UBX-CFG-RST message, the host commands the u-blox receiver to stop, specifying
                    // a BBR mask 0 ("Hotstart") and a reset mode of 0x08 ("Controlled GNSS stop")
                    bool result = mUblox.ubxCfgRst(0x0000, 0x08); // request CONTROLLED GNSS stop with HOT start
                    qDebug() << "UbloxRover: Receiver stop request" << (result ? "was sent" : "reported an error");

                    createConfigurationBackup();    // this is a blocking call

                    if (mReceiverState == RECEIVER_STATE::BACKUP_CREATED) {
                        qDebug() << "UbloxRover: Backup created, please power cycle the receiver to find the created backup on next run";

                        result = mUblox.ubxCfgRst(0x0000, 0x09); // request CONTROLLED GNSS start with HOT start
                        qDebug() << "UbloxRover: Receiver start request" << (result ? "was sent" : "reported an error");
                    }
                }
            } break;
            default:
                break;
        }

        shutdownGNSSReceiver();
    }
}

void UbloxRover::shutdownGNSSReceiver()
{
    if (mUblox.isSerialConnected()) {
        // bool result = mUblox.ubxCfgRst(0x0000, 0x01); // request CONTROLLED SOFTWARE reset with HOT start
        // qDebug() << "UbloxRover: Software reset request" << (result ? "was sent" : "reported an error");
        setReceiverState(RECEIVER_STATE::DISCONNECTED);
        mUblox.disconnectSerial();
    }
}
