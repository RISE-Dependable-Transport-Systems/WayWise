#include "ubloxrover.h"
#include <QDebug>

UbloxRover::UbloxRover(QSharedPointer<VehicleState> vehicleState)
{
    mVehicleState = vehicleState;

    // Initialize AHRS algorithm from Fusion library.
    // This provides orientation data if the u-blox device contains an IMU (e.g., F9R), even without GNSS reception
    FusionBiasInitialise(&mFusionBias, 0.5f, mIMUSamplePeriod); // stationary threshold = 0.5 degrees per second, expected period 20 ms
    FusionAhrsInitialise(&mFusionAhrs, 0.5f); // gain = 0.5
    connect(&mUblox, &Ublox::rxEsfMeas, this, &UbloxRover::updateAHRS);
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

bool UbloxRover::configureUblox()
{
    mUblox.ubxCfgMsg(UBX_CLASS_ESF, UBX_ESF_MEAS, 1);
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
