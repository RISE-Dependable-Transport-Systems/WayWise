#ifndef PACKETINTERFACETCPSERVER_H
#define PACKETINTERFACETCPSERVER_H

#include <QObject>
#include <sdvp_qtcommon/legacy/tcpserversimple.h>
#include <sdvp_qtcommon/legacy/packetinterface.h>
#include <sdvp_qtcommon/legacy/utility.h>
#include <sdvp_qtcommon/legacy/vbytearray.h>
#include <sdvp_qtcommon/vehiclestate.h>
#include <sdvp_qtcommon/movementcontroller.h>

class PacketInterfaceTCPServer : public QObject
{
    Q_OBJECT
public:
    explicit PacketInterfaceTCPServer(QObject *parent = nullptr);
    bool listen(quint16 port = 8300);

    QSharedPointer<VehicleState> getVehicleState() const;
    void setVehicleState(const QSharedPointer<VehicleState> &getVehicleState);

    QSharedPointer<MovementController> getMovementController() const;
    void setMovementController(const QSharedPointer<MovementController> &movementController);

signals:

private:
    const int firmware_version_major = 12;
    const int firmware_version_minor = 3;

    TcpServerSimple mTcpServer;
    PacketInterface mPacketInterface;
    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<MovementController> mMovementController;

};

#endif // PACKETINTERFACETCPSERVER_H
