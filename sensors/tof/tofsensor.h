/*
 *     Copyright 2024 RISE Sweden
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract interface for receiving distance values from a Time of Flight (ToF) sensor
 */

#ifndef TOFSENSOR_H
#define TOFSENSOR_H

#include <QObject>

class ToFSensor : public QObject
{
    Q_OBJECT
public:
    virtual bool setUpdateIntervall(int intervall_ms) = 0;
    double getLastDistance() {return mLastDistance;};


signals:
    void updatedDistance(double distance_m);

protected:
    double mLastDistance = 0;

private:

};

#endif // TOFSENSOR_H
