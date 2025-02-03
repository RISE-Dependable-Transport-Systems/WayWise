/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "bno055orientationupdater.h"
#include <QDebug>

BNO055OrientationUpdater::BNO055OrientationUpdater(QSharedPointer<VehicleState> vehicleState, QString i2cBus) : IMUOrientationUpdater(vehicleState)
{
    int res = get_i2cbus(i2cBus.toLocal8Bit().data(), mBNO055I2CAddress.toLocal8Bit().data());

    if (res == 0) {
        if(0) // for debugging
            printBNO055Info();

        //bno_reset();

        // IMU fusion mode: euler angles without magnetometer
        opmode_t newmode = imu;
        res = set_mode(newmode);

        connect(&mPollTimer, &QTimer::timeout, [this]() {
            static struct bnoeul bnod;
            int res = get_eul(&bnod);
            if (res != 0) {
                qDebug() << "WARNING: BNO055OrientationUpdater cannot read orientation data from i2c bus.";
            } else {
                // qDebug() << "Roll:" << bnod.eul_roll << "Pitch:" << bnod.eul_pitc << "Yaw:" << bnod.eul_head;
                QSharedPointer<VehicleState> vehicleState = getVehicleState();
                PosPoint currIMUPos = vehicleState->getPosition(PosType::IMU);

                currIMUPos.setRollPitchYaw(bnod.eul_roll, bnod.eul_pitc, coordinateTransforms::yawNEDtoENU(bnod.eul_head));
                currIMUPos.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
                vehicleState->setPosition(currIMUPos);

                emit updatedIMUOrientation(vehicleState);
            }
        });
        mPollTimer.start(mPollIntervall_ms);
    } else
        qDebug() << "WARNING: BNO055OrientationUpdater is unable to open i2c bus" << i2cBus << ". Disabled.";
}

void BNO055OrientationUpdater::printBNO055Info()
{
    struct bnoinf bnoi;
    if(get_inf(&bnoi) != 0) {
       printf("Error: Cannot read sensor version data.\n");
       exit(-1);
    }

    /* ----------------------------------------------------------- *
     * print the formatted output strings to stdout                *
     * ----------------------------------------------------------- */
    printf("\nBN0055 Information\n");
    printf("----------------------------------------------\n");
    printf("   Chip Version ID = 0x%02X\n", bnoi.chip_id);
    printf("  Accelerometer ID = 0x%02X\n", bnoi.acc_id);
    printf("      Gyroscope ID = 0x%02X\n", bnoi.gyr_id);
    printf("   Magnetoscope ID = 0x%02X\n", bnoi.mag_id);
    printf("  Software Version = %d.%d\n", bnoi.sw_msb, bnoi.sw_lsb);
    printf("   Operations Mode = "); print_mode(bnoi.opr_mode);
    printf("        Power Mode = "); print_power(bnoi.pwr_mode);
    printf("Axis Configuration = "); print_remap_conf(bnoi.axr_conf);
    printf("   Axis Remap Sign = "); print_remap_sign(bnoi.axr_sign);
    printf("System Status Code = "); print_sstat(bnoi.sys_stat);
    printf("System Clocksource = "); print_clksrc();

    printf("Accelerometer Test = ");
    if((bnoi.selftest >> 0) & 0x01) printf("OK\n");
    else printf("FAIL\n");

    printf(" Magnetometer Test = ");
    if((bnoi.selftest >> 1) & 0x01) printf("OK\n");
    else printf("FAIL\n");

    printf("    Gyroscope Test = ");
    if((bnoi.selftest >> 2) & 0x01) printf("OK\n");
    else printf("FAIL\n");

    printf("MCU Cortex M0 Test = ");
    if((bnoi.selftest >> 3) & 0x01) printf("OK\n");
    else printf("FAIL\n");

    printf(" System Error Code = ");
    switch(bnoi.sys_err) {
       case 0x00:
          printf("No Error\n");
          break;
       case 0x01:
          printf("Peripheral initialization error\n");
          break;
       case 0x02:
          printf("System initializion error\n");
          break;
       case 0x03:
          printf("Selftest result failed\n");
          break;
       case 0x04:
          printf("Register map value out of range\n");
          break;
       case 0x05:
          printf("Register map address out of range\n");
          break;
       case 0x06:
          printf("Register map write error\n");
          break;
       case 0x07:
          printf("BNO low power mode not available\n");
          break;
       case 0x08:
          printf("Accelerometer power mode not available\n");
          break;
       case 0x09:
          printf("Fusion algorithm configuration error\n");
          break;
       case 0x0A:
          printf("Sensor configuration error\n");
          break;
    }

    print_unit(bnoi.unitsel);

    printf("Sensor Temperature = ");
    if(bnoi.opr_mode > 0) {
       if((bnoi.unitsel >> 4) & 0x01) printf("%d°F\n", bnoi.temp_val);
       else printf("%d°C\n",bnoi.temp_val);
    }
    else  printf("no data in CONFIG mode\n");

    printf("\n----------------------------------------------\n");
    struct bnoaconf bnoac;
    if(get_acc_conf(&bnoac) == 0) print_acc_conf(&bnoac);
}

bool BNO055OrientationUpdater::setUpdateIntervall(int pollIntervall_ms)
{
    mPollIntervall_ms = pollIntervall_ms;
    mPollTimer.start(mPollIntervall_ms);

    return true;
}
