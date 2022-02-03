#ifndef CANOPENCONTROLLERINTERFACE_H
#define CANOPENCONTROLLERINTERFACE_H

#include <QObject>

class CANopenControllerInterface : public QObject
{
    Q_OBJECT

public slots:
    void startDevice();
    void actualSpeedReceived(double speed);
    void actualSteeringReceived(double steering);
    void commandSpeedReceived(double speed);
    void commandSteeringReceived(double steering);
    void commandStatusReceived(quint8 status);
    void actualStatusReceived(quint8 status);
    void batterySOCReceived(double batterysoc);
    void batteryVoltageReceived(double batteryvoltage);
    void commandAttributesReceived(quint32 attributes);

signals:
    void finished();
    void error(QString err);
    void sendActualSpeed(double speed);
    void sendActualSteeringCurvature(double steeringCurvature);
    void sendCommandSpeed(double speed);
    void sendCommandSteering(double steering);
    void sendCommandStatus(quint8 status);
    void sendActualStatus(quint8 status);
    void sendBatterySOC(double batterysoc);
    void sendBatteryVoltage(double batteryvoltage);
    void sendCommandAttributes(quint32 attributes);
    void activateSimulation();
};

#endif // CANOPENCONTROLLERINTERFACE_H
