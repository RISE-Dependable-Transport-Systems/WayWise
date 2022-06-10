/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
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
#include <mavsdk/mavsdk.h>
#include "mavsdkvehicleconnection.h"

class MavsdkStation : public QObject
{
    Q_OBJECT
public:
    explicit MavsdkStation(QObject *parent = nullptr);
    bool startListeningUDP(uint16_t port = mavsdk::Mavsdk::DEFAULT_UDP_PORT);
    bool startListeningSerial(const QSerialPortInfo& portInfo, int baudrate);

    // broadcasts to all vehicles
    void forwardRtcmData(const QByteArray& data, const int &type);
    void setEnuReference(const llh_t &enuReference);

    QList<QSharedPointer<MavsdkVehicleConnection>> getVehicleConnectionList() const;

signals:
    void gotNewVehicleConnection(QSharedPointer<MavsdkVehicleConnection> vehicleConnection);

private:
    mavsdk::Mavsdk mMavsdk;
    QMap<int, QSharedPointer<MavsdkVehicleConnection>> mVehicleConnectionMap;
};

#endif // MAVSDKSTATION_H