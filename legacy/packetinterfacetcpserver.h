#ifndef PACKETINTERFACETCPSERVER_H
#define PACKETINTERFACETCPSERVER_H

#include <QObject>
#include <sdvp_qtcommon/legacy/tcpserversimple.h>
#include <sdvp_qtcommon/legacy/packetinterface.h>
#include <sdvp_qtcommon/legacy/utility.h>
#include <sdvp_qtcommon/legacy/vbytearray.h>
#include <sdvp_qtcommon/vehiclestate.h>

class PacketInterfaceTCPServer : public QObject
{
    Q_OBJECT
public:
    explicit PacketInterfaceTCPServer(QObject *parent = nullptr);
    bool listen(quint16 port = 8300);

    QSharedPointer<VehicleState> vehicleState() const;
    void setVehicleState(const QSharedPointer<VehicleState> &vehicleState);

signals:

private:
    TcpServerSimple mTcpServer;
    PacketInterface mPacketInterface;
    QSharedPointer<VehicleState> mVehicleState;

};

#endif // PACKETINTERFACETCPSERVER_H
