/*
 *     Copyright 2023 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 *     Code adapted from example:
 *     https://www.ics.com/blog/gpio-programming-exploring-libgpiod-library
 */

#include "vehiclelighting.h"
#include <QDebug>

VehicleLighting::VehicleLighting(QSharedPointer<VehicleState> vehicleState, QString chipName)
{
    mVehicleState = vehicleState;

    try {
        // Open GPIO chip
        mChip.open(chipName.toStdString(), 3);

        // Open GPIO lines
        mBackupLight = mChip.get_line(22);
        mBrakeLight = mChip.get_line(23);
        mRightTurnSignal = mChip.get_line(24);
        mLeftTurnSignal = mChip.get_line(25);

        // Open LED lines for output
        mBackupLight.request({"lights", gpiod::line_request::DIRECTION_OUTPUT, 0});
        mBrakeLight.request({"lights", gpiod::line_request::DIRECTION_OUTPUT, 0});
        mRightTurnSignal.request({"lights", gpiod::line_request::DIRECTION_OUTPUT, 0});
        mLeftTurnSignal.request({"lights", gpiod::line_request::DIRECTION_OUTPUT, 0});

        connect(&mUpdateStateTimer, &QTimer::timeout, this, &VehicleLighting::updateState);
        mUpdateStateTimer.start(mUpdateStatePeriod_ms);
    
    } catch (const std::exception &e) {
        qDebug() << "Error: " << e.what();
    }
}

VehicleLighting::~VehicleLighting()
{
    // Release lines
    if (mBackupLight)
        mBackupLight.release();
    if (mBrakeLight)
        mBrakeLight.release();
    if (mRightTurnSignal)
        mRightTurnSignal.release();
    if (mLeftTurnSignal)
        mLeftTurnSignal.release();
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
    mLeftTurnSignal.set_value(0);
    mRightTurnSignal.set_value(0);
}

void VehicleLighting::turnSignalRelay(bool left)
{
    static int x = 1;
    x = 1 - x;
    if (left)
        mLeftTurnSignal.set_value(x);
    else
        mRightTurnSignal.set_value(x);
}

void VehicleLighting::backupLight(bool On)
{
    if (On)
        mBackupLight.set_value(1);
    else
        mBackupLight.set_value(0);
}

void VehicleLighting::brakeLight(bool On)
{
    if (On)
        mBrakeLight.set_value(1);
    else
        mBrakeLight.set_value(0);
}

void VehicleLighting::updateState()
{
    turnSignal(mVehicleState->getSteering());
    rearLights(mVehicleState->getSpeed());
}
