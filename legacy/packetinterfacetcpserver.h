#ifndef PACKETINTERFACETCPSERVER_H
#define PACKETINTERFACETCPSERVER_H

#include <QObject>
#include <sdvp_qtcommon/legacy/tcpserversimple.h>
#include <sdvp_qtcommon/legacy/packetinterface.h>
#include <sdvp_qtcommon/legacy/utility.h>
#include <sdvp_qtcommon/legacy/vbytearray.h>

class PacketInterfaceTCPServer : public QObject
{
    Q_OBJECT
public:
    explicit PacketInterfaceTCPServer(QObject *parent = nullptr);
    bool listen(quint16 port = 8300);

signals:

private:
    TcpServerSimple mTcpServer;
    PacketInterface mPacketInterface;

};

#endif // PACKETINTERFACETCPSERVER_H
