/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "mavsdkstation.h"
#include <QtDebug>
#include <QThread>

MavsdkStation::MavsdkStation(QObject *parent) : QObject(parent)
{
    connect(&mHeartbeatTimer, &QTimer::timeout, this, &MavsdkStation::on_timeout);
    mHeartbeatTimer.start(1000);

    mMavsdk.subscribe_on_new_system([this](){
        for (const auto &system : mMavsdk.systems()) {
            if (!mVehicleConnectionMap.contains(system->get_system_id())) {
                if (system->has_autopilot()) {
                    qDebug() << "MavsdkStation: detected system" << system->get_system_id() << "waiting for another heartbeat for initializing MavsdkVehicleConnection...";

                    // Wait for heartbeat using passthrough to instantiate vehicleConnection (mainly needed to get MAV_TYPE)
                    auto mavlinkPassthrough = new mavsdk::MavlinkPassthrough(system);
                    mavlinkPassthrough->subscribe_message(MAVLINK_MSG_ID_HEARTBEAT, [this, system, mavlinkPassthrough](const mavlink_message_t &message){
                        // unsubscribe from further heartbeats by deleting passthrough
                        delete mavlinkPassthrough;

                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&message, &heartbeat);

                        QSharedPointer<MavsdkVehicleConnection> vehicleConnection = QSharedPointer<MavsdkVehicleConnection>::create(system, (MAV_TYPE) heartbeat.type);
                        mVehicleConnectionMap.insert(system->get_system_id(), vehicleConnection);

                        // move to same QThread MavsdkStation lives in
                        vehicleConnection->moveToThread(thread());

                        connect(vehicleConnection.get(), &MavsdkVehicleConnection::gotHeartbeat, this, &MavsdkStation::on_gotHeartbeat);

                        mVehicleTimers.append(qMakePair(system->get_system_id(), 0));    // Timer initialised to zero

                        emit gotNewVehicleConnection(vehicleConnection);
                    });
                } else
                    qDebug() << "Note: MavsdkStation ignored system" << system->get_system_id();
            }
        }
    });
}

bool MavsdkStation::startListeningUDP(uint16_t port)
{
    QString connection_url = "udp://:" + QString::number(port);
    mavsdk::ConnectionResult connection_result = mMavsdk.add_any_connection(connection_url.toStdString());
    if (connection_result == mavsdk::ConnectionResult::Success) {
        qDebug() << "MavsdkStation: Waiting to discover vehicles on " + connection_url + "...";
        return true;
    } else {
        qDebug() << "MavsdkStation: Failed to open connection on " + connection_url;
        return false;
    }
}

bool MavsdkStation::startListeningSerial(const QSerialPortInfo &portInfo, int baudrate)
{
    mavsdk::ConnectionResult connection_result = mMavsdk.add_serial_connection(portInfo.systemLocation().toStdString(), baudrate);
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
    for (const auto &vehicleConnection : mVehicleConnectionMap)
        vehicleConnection->inputRtcmData(data);
}

void MavsdkStation::setEnuReference(const llh_t &enuReference)
{
    for (const auto &vehicleConnection : mVehicleConnectionMap)
        vehicleConnection->setEnuReference(enuReference);
}

QList<QSharedPointer<MavsdkVehicleConnection>> MavsdkStation::getVehicleConnectionList() const
{
    return mVehicleConnectionMap.values();
}

void MavsdkStation::on_timeout()
{
    for(auto& vehicleTimer : mVehicleTimers) {
        vehicleTimer.second++;

        if(vehicleTimer.second == 5) {    // disconnect criteria: 5 heartbeats missed
            mVehicleConnectionMap.remove(vehicleTimer.first);
            mVehicleTimers.removeOne(vehicleTimer);

            qDebug() << "System " << vehicleTimer.first << " disconnected.";
        }
    }
}

void MavsdkStation::on_gotHeartbeat(const quint8 systemId)
{
    for(auto& vehicleTimer : mVehicleTimers)
        if(vehicleTimer.first == systemId)
            vehicleTimer.second = 0;
}
