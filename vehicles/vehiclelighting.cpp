/*
 *     Copyright 2023 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 *     Code adapted from example:
 *     https://www.ics.com/blog/gpio-programming-exploring-libgpiod-library
 */

#include "vehiclelighting.h"

VehicleLighting::VehicleLighting(QObject *parent)
    : QObject{parent}
{
    // Open GPIO chip
    if (!(mChip = gpiod_chip_open_by_name(mChipName)))
        throw QString("Could not open gpio chip with name: ")+=QString::fromLocal8Bit(mChipName);
    else {
        // Open GPIO lines
        mBackupLight = gpiod_chip_get_line(mChip, 22);
        mBrakeLight = gpiod_chip_get_line(mChip, 23);
        mRightTurnSignal = gpiod_chip_get_line(mChip, 24);
        mLeftTurnSignal = gpiod_chip_get_line(mChip, 25);

        // Open LED lines for output
        gpiod_line_request_output(mBackupLight, "lights", 0);
        gpiod_line_request_output(mBrakeLight, "lights", 0);
        gpiod_line_request_output(mRightTurnSignal, "lights", 0);
        gpiod_line_request_output(mLeftTurnSignal, "lights", 0);
    }
}

VehicleLighting::~VehicleLighting()
{
    // Release lines and chip
    gpiod_line_release(mBackupLight);
    gpiod_line_release(mBrakeLight);
    gpiod_line_release(mRightTurnSignal);
    gpiod_line_release(mLeftTurnSignal);
    gpiod_chip_close(mChip);
}

void VehicleLighting::turnSignal(double steering)
{
    if (steering < -0.5)
        turnSignalRelay(true);
    else if (steering > 0.5)
        turnSignalRelay(false);
    else
        stopSignaling();
}

void VehicleLighting::rearLights(double speed)
{
    if (speed < -0.01) {
        backupLight(true);
        brakeLight(false);
    } else if (speed > 0.01) {
        backupLight(false);
        brakeLight(false);
    } else {
        backupLight(false);
        brakeLight(true);
    }
}

void VehicleLighting::stopSignaling()
{
    gpiod_line_set_value(mLeftTurnSignal, 0);
    gpiod_line_set_value(mRightTurnSignal, 0);
}

void VehicleLighting::turnSignalRelay(bool left)
{
    static int x, counter = 1;
    if (counter == 10) {
        x = 1 - x;
        if (left)
            gpiod_line_set_value(mLeftTurnSignal, x);
        else
            gpiod_line_set_value(mRightTurnSignal, x);
        counter = 0;
    }
    counter++;
}

void VehicleLighting::backupLight(bool On)
{
    if (On)
        gpiod_line_set_value(mBackupLight, 1);
    else
        gpiod_line_set_value(mBackupLight, 0);
}

void VehicleLighting::brakeLight(bool On)
{
    if (On)
        gpiod_line_set_value(mBrakeLight, 1);
    else
        gpiod_line_set_value(mBrakeLight, 0);
}
