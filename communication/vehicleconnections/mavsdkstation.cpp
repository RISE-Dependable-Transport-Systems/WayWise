/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Copyright 2024 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "mavsdkstation.h"
#include <QtDebug>
#include <QThread>
#include <QHostAddress>
#include <QNetworkInterface>

MavsdkStation::MavsdkStation(QObject *parent) : QObject(parent)
{
    connect(&mHeartbeatTimer, &QTimer::timeout, this, &MavsdkStation::on_timeout);
    mHeartbeatTimer.start(1000);

    mavsdk::Mavsdk::Configuration config = mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation};
    config.set_always_send_heartbeats(true);
    mMavsdk.reset(new mavsdk::Mavsdk{config});

    mMavsdk->subscribe_on_new_system([this](){ emit gotNewMavsdkSystem(); });

    connect(this, &MavsdkStation::gotNewMavsdkSystem, this, &MavsdkStation::handleNewMavsdkSystem, Qt::QueuedConnection);
}

bool MavsdkStation::startListeningUDP(uint16_t port)
{
    QString connection_url = "udp://:" + QString::number(port);
    mavsdk::ConnectionResult connection_result = mMavsdk->add_any_connection(connection_url.toStdString());
    if (connection_result == mavsdk::ConnectionResult::Success) {
        const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);
        for (const QHostAddress &address: QNetworkInterface::allAddresses()) {
            if (address.protocol() == QAbstractSocket::IPv4Protocol && address != localhost)
                qDebug() << "MavsdkStation listening on localhost and external IP address: " + address.toString();;
        }
        qDebug() << "MavsdkStation: Waiting to discover vehicles on " + connection_url + "...";
        return true;
    } else {
        qDebug() << "MavsdkStation: Failed to open connection on " + connection_url;
        return false;
    }
}

bool MavsdkStation::startListeningSerial(const QSerialPortInfo &portInfo, int baudrate)
{
    mavsdk::ConnectionResult connection_result = mMavsdk->add_serial_connection(portInfo.systemLocation().toStdString(), baudrate);
    if (connection_result == mavsdk::ConnectionResult::Success) {
        qDebug() << "MavsdkStation: Waiting to discover vehicles on " + portInfo.systemLocation() + "...";
        return true;
    } else {
        qDebug() << "MavsdkStation: Failed to open connection on " + portInfo.systemLocation();
        return false;
    }
}

void MavsdkStation::forwardRtcmData(const QByteArray &data, const int &type)
{
    Q_UNUSED(type)
    for (const auto &vehicleConnection : getVehicleConnectionList())
        vehicleConnection->inputRtcmData(data);
}

void MavsdkStation::setEnuReference(const llh_t &enuReference)
{
    for (const auto &vehicleConnection : getVehicleConnectionList())
        vehicleConnection->setEnuReference(enuReference);
}

QList<QSharedPointer<MavsdkVehicleConnection>> MavsdkStation::getVehicleConnectionList() const
{
    QList<QSharedPointer<MavsdkVehicleConnection>> vehicleConnectionList = mVehicleConnectionMap.values();

    vehicleConnectionList.removeAll(nullptr);

    return vehicleConnectionList;
}

void MavsdkStation::on_timeout()
{
    for(auto& vehicleTimeoutCounter : mVehicleHeartbeatTimeoutCounters) {
        vehicleTimeoutCounter.second++;
        qDebug() << "verbose " << "Heartbeat timeout count: " << vehicleTimeoutCounter.second;


        if(vehicleTimeoutCounter.second == HEARTBEATTIMER_TIMEOUT_SECONDS) {
            mVehicleConnectionMap.remove(vehicleTimeoutCounter.first);
            emit disconnectOfVehicleConnection(vehicleTimeoutCounter.first);

            qDebug() << "System" << vehicleTimeoutCounter.first << "disconnected. ";

            mVehicleHeartbeatTimeoutCounters.removeOne(vehicleTimeoutCounter);
        }
    }
}

void MavsdkStation::on_gotHeartbeat(const quint8 systemId)
{
    qDebug() << "verbose " << "Got heartbeat with systemId: " << systemId;
    for(auto& vehicleTimeoutCounter : mVehicleHeartbeatTimeoutCounters)
        if(vehicleTimeoutCounter.first == systemId)
            vehicleTimeoutCounter.second = 0;
}

QSharedPointer<MavsdkVehicleConnection> MavsdkStation::getVehicleConnection(const quint8 systemId) const
{
    return mVehicleConnectionMap.find(systemId).value();
}

void MavsdkStation::handleNewMavsdkSystem()
{
    for (const auto &system : mMavsdk->systems()) {
        if (!mVehicleConnectionMap.contains(system->get_system_id())) {
            if (system->has_autopilot()) {
                qDebug() << "MavsdkStation: detected system" << system->get_system_id() << "waiting for another heartbeat for initializing MavsdkVehicleConnection...";
                mVehicleConnectionMap.insert(system->get_system_id(), nullptr); // Register system_id, but wait for heartbeat to initialize VehicleConnection

                // Wait for heartbeat using passthrough to instantiate vehicleConnection (mainly needed to get MAV_TYPE)
                auto mavlinkPassthrough = new mavsdk::MavlinkPassthrough(system);
                mavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_HEARTBEAT, [this, system, mavlinkPassthrough](const mavlink_message_t &message) {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&message, &heartbeat);

                    // Only use heartbeat from autopilot component to instantiate vehicleConnection once
                    if ((MAV_AUTOPILOT)heartbeat.autopilot != MAV_AUTOPILOT_INVALID) {
                        // unsubscribe from further heartbeats by deleting passthrough
                        delete mavlinkPassthrough;

                        QSharedPointer<MavsdkVehicleConnection> vehicleConnection = QSharedPointer<MavsdkVehicleConnection>::create(system, (MAV_TYPE) heartbeat.type);
                        mVehicleConnectionMap[system->get_system_id()] = vehicleConnection;

                        connect(vehicleConnection.get(), &MavsdkVehicleConnection::gotHeartbeat, this, &MavsdkStation::on_gotHeartbeat);

                        mVehicleHeartbeatTimeoutCounters.append(qMakePair(system->get_system_id(), 0));    // Timer initialised to zero

                        emit gotNewVehicleConnection(vehicleConnection);
                    }
                });
            } else
                qDebug() << "Note: MavsdkStation ignored system" << system->get_system_id(); // ToDo: create connection to systems that doesn't have an autopilot
        }
    }
}
