/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "mavsdkstation.h"
#include <QtDebug>
#include <QThread>

MavsdkStation::MavsdkStation(QObject *parent) : QObject(parent)
{
    mMavsdk.subscribe_on_new_system([this](){
        for (const auto &system : mMavsdk.systems()) {
            if (!mVehicleConnectionMap.contains(system->get_system_id())) {
                if (system->has_autopilot()) {
                    QSharedPointer<MavsdkVehicleConnection> vehicleConnection = QSharedPointer<MavsdkVehicleConnection>::create(system);
                    mVehicleConnectionMap.insert(system->get_system_id(), vehicleConnection);

                    // move to same QThread MavsdkStation lives in
                    vehicleConnection->moveToThread(thread());

                    qDebug() << "Note: MavsdkStation added system" << system->get_system_id();
                    emit gotNewVehicleConnection(vehicleConnection);
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
