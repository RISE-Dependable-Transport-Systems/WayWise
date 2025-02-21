/*
 *     Copyright 2024 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * "Control station" that accepts vehicle connections via MAVLINK using MAVSDK
 */

#ifndef MAVSDKSTATION_H
#define MAVSDKSTATION_H

#include <QObject>
#include <QMap>
#include <QSharedPointer>
#include <QSerialPortInfo>
#include <QTimer>
#include <QVector>
#include <QPair>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include "mavsdkvehicleconnection.h"

class MavsdkStation : public QObject
{
    Q_OBJECT
public:
    explicit MavsdkStation(QObject *parent = nullptr);
    bool startListeningUDP(uint16_t port = mavsdk::Mavsdk::DEFAULT_UDP_PORT);
    bool startListeningSerial(const QSerialPortInfo& portInfo = QSerialPortInfo("ttyUSB0"), int baudrate = mavsdk::Mavsdk::DEFAULT_SERIAL_BAUDRATE);

    // broadcasts to all vehicles
    void forwardRtcmData(const QByteArray& data, const int &type);
    void setEnuReference(const llh_t &enuReference);

    QList<QSharedPointer<MavsdkVehicleConnection>> getVehicleConnectionList() const;
    QSharedPointer<MavsdkVehicleConnection> getVehicleConnection(const quint8 systemId) const;

private slots:
    void on_gotHeartbeat(quint8 systemId);
    void on_timeout();

signals:
    void gotNewVehicleConnection(QSharedPointer<MavsdkVehicleConnection>);
    void gotNewMavsdkSystem();
    void disconnectOfVehicleConnection(int systemId);

private:
    std::shared_ptr<mavsdk::Mavsdk> mMavsdk;
    QMap<int, QSharedPointer<MavsdkVehicleConnection>> mVehicleConnectionMap;

    QTimer mHeartbeatTimer;
    const int HEARTBEATTIMER_TIMEOUT_SECONDS = 5;
    QVector<QPair<quint8, int>> mVehicleHeartbeatTimeoutCounters;
    void handleNewMavsdkSystem();
};

#endif // MAVSDKSTATION_H
