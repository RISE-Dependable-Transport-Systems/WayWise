/*
 *     Copyright 2023 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 *     Code adapted from example:
 *     https://www.ics.com/blog/gpio-programming-exploring-libgpiod-library
 */

#ifndef VEHICLELIGHTING_H
#define VEHICLELIGHTING_H

#include <QObject>
#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <QSharedPointer>
#include <QTimer>
#include "vehiclestate.h"

class VehicleLighting : public QObject
{
    Q_OBJECT
public:
    explicit VehicleLighting(QSharedPointer<VehicleState> vehicleState);
    ~VehicleLighting();

    void turnSignal(double steering);
    void rearLights(double speed);

signals:

private:
    QSharedPointer<VehicleState> mVehicleState;
    QTimer mUpdateStateTimer;
    unsigned mUpdateStatePeriod_ms = 1000;

    void leftTurnSignal();
    void rightTurnSignal();
    void stopSignaling();
    void turnSignalRelay(bool Left = true);
    void backupLight(bool On = false);
    void brakeLight(bool On = false);
    void updateState();

    const char *mChipName = "gpiochip0";
    gpiod_chip *mChip;
    gpiod_line *mBackupLight;  // White LED
    gpiod_line *mBrakeLight;  // Red LED
    gpiod_line *mRightTurnSignal;  // Orange LED
    gpiod_line *mLeftTurnSignal; // Orange LED
};

#endif // VEHICLELIGHTING_H
