#ifndef SLAVE_H
#define SLAVE_H

#include <QObject>
#include <lely/coapp/slave.hpp>
#include <QDebug>

using namespace lely;

class MySlave : public QObject, public canopen::BasicSlave{
    Q_OBJECT
public:
    using BasicSlave::BasicSlave;

public slots:
    void commandSpeedReceived(const double& speed);
    void commandSteeringReceived(const double& steering);
    void statusReceived(const quint8& status);
    void commandAttributesReceived(const quint32& attributes);
    void GNSSDataToCANReceived(const QVariant& gnssData);

signals:
    void sendActualSpeed(double speed);
    void sendActualSteering(double steering);
    void sendStatus(quint8 status);
    void sendBatterySOC(double batterysoc);
    void sendBatteryVoltage(double batteryvoltage);

protected:
 // This function gets called every time a value is written to the local object
 // dictionary by an SDO or RPDO.
 void OnWrite(uint16_t idx, uint8_t subidx) noexcept override;
};

#endif // SLAVE_H
