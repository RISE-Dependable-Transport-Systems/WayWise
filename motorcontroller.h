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
    void statusValuesReceived(double rpm, int tachometer, double voltageInput, double temperature, int errorID);

};

#endif // MOTORCONTROLLER_H
