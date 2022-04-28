#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <QObject>

class ServoController : public QObject
{
    Q_OBJECT
public:
    virtual void requestSteering(float steering) = 0; // TODO: respect servo range, center in abstract class?

    bool getInvertOutput() const;
    void setInvertOutput(bool getInvertOutput);

    double getServoRange() const;
    void setServoRange(double servoRange);

    double getServoCenter() const;
    void setServoCenter(double servoCenter);

signals:

private:
    bool mInvertOutput = false;
    double mServoRange = 2.0; // servo goes from -1.0 to 1.0 by default -> range 2.0
    double mServoCenter = 0.0; // default center = 0.0

};

#endif // SERVOCONTROLLER_H