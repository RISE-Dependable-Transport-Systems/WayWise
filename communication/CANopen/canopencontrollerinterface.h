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
    void commandSpeedReceived(const double& speed);
    void commandSteeringReceived(const double& steering);
    void commandStatusReceived(quint8 status);
    void actualStatusReceived(const quint8& status);
    void batterySOCReceived(double batterysoc);
    void batteryVoltageReceived(double batteryvoltage);
    void commandAttributesReceived(const quint32& attributes);
    void GNSSDataToCANReceived(const QVariant& gnssData);

signals:
    void finished();
    void error(QString err);
    void sendActualSpeed(double speed);
    void sendActualSteeringCurvature(double steeringCurvature);
    void sendCommandSpeed(const double& speed);
    void sendCommandSteering(const double& steering);
    void sendCommandStatus(quint8 status);
    void sendActualStatus(const quint8& status);
    void sendBatterySOC(double batterysoc);
    void sendBatteryVoltage(double batteryvoltage);
    void sendCommandAttributes(const quint32& attributes);
    void activateSimulation();
    void sendGNSSDataToCAN(const QVariant&);
};

#endif // CANOPENCONTROLLERINTERFACE_H
