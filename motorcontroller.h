#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <QObject>

class MotorController : public QObject
{
    Q_OBJECT
public:
    virtual void pollFirmwareVersion() = 0;
    virtual void requestRPM(int32_t rpm) = 0;

signals:
    void firmwareVersionReceived(QPair<int,int>); // Major and minor firmware version
    void gotStatusValues(double rpm, int tachometer, int tachometer_abs, double voltageInput, double temperature, int errorID);

};

#endif // MOTORCONTROLLER_H
