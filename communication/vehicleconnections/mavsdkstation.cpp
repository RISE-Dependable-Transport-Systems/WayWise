#include "mavsdkstation.h"
#include <QtDebug>

MavsdkStation::MavsdkStation(QObject *parent) : QObject(parent)
{
    mMavsdk.subscribe_on_new_system([this](){
        int vehicleID = mMavsdk.systems().back()->get_system_id();
        QSharedPointer<MavsdkVehicleConnection> vehicleConnection = QSharedPointer<MavsdkVehicleConnection>::create(mMavsdk.systems().back());

        mVehicleConnectionMap.insert(vehicleID, vehicleConnection);
        // move to same QThread MavsdkStation lives in
        vehicleConnection->moveToThread(thread());

        emit gotNewVehicleConnection(vehicleConnection);
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
