/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Abstract class that provides an interface to talk to servos, e.g., to be used within a MovementController
 */

#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <QObject>

class ServoController : public QObject
{
    Q_OBJECT
public:
    bool getInvertOutput() const;
    void setInvertOutput(bool getInvertOutput);

    double getServoRange() const;
    void setServoRange(double servoRange);

    double getServoCenter() const;
    void setServoCenter(double servoCenter);

    void requestSteering(float steering);

signals:

private:
    virtual void requestServoPosition(float servoPosition) = 0;

    bool mInvertOutput = false;
    double mServoRange = 2.0; // servo goes from -1.0 to 1.0 by default -> range 2.0
    double mServoCenter = 0.0; // default center = 0.0

};

#endif // SERVOCONTROLLER_H
