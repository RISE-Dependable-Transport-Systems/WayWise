/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 *     Code adapted from example:
 *     https://www.ics.com/blog/gpio-programming-exploring-libgpiod-library
 */

#ifndef VEHICLELIGHTING_H
#define VEHICLELIGHTING_H

#include <QObject>
#include <gpiod.hpp>
#include <stdio.h>
#include <unistd.h>
#include <QSharedPointer>
#include <QTimer>
#include "vehiclestate.h"

class VehicleLighting : public QObject
{
    Q_OBJECT
public:
    explicit VehicleLighting(QSharedPointer<VehicleState> vehicleState, QString mChipName = "gpiochip0");
    ~VehicleLighting();

    void turnSignal(double steering);
    void rearLights(double speed);

private:
    QSharedPointer<VehicleState> mVehicleState;
    QTimer mUpdateStateTimer;
    unsigned mUpdateStatePeriod_ms = 500;

    void stopSignaling();
    void turnSignalRelay(bool Left = true);
    void backupLight(bool On = false);
    void brakeLight(bool On = false);
    void updateState();

    gpiod::chip mChip;
    gpiod::line mBackupLight;  // White LED
    gpiod::line mBrakeLight;  // Red LED
    gpiod::line mRightTurnSignal;  // Orange LED
    gpiod::line mLeftTurnSignal; // Orange LED
};

#endif // VEHICLELIGHTING_H
