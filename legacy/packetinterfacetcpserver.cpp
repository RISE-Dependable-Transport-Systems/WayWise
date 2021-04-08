#include "packetinterfacetcpserver.h"
#include "datatypes.h"

PacketInterfaceTCPServer::PacketInterfaceTCPServer(QObject *parent) : QObject(parent)
{
    mTcpServer.setUsePacket(true);

    QObject::connect(mTcpServer.packet(), &Packet::packetReceived, [this](QByteArray& data){mPacketInterface.sendPacket(data);}); // feed incoming tcp data into "PacketInterface"

    QObject::connect(&mPacketInterface, &PacketInterface::dataToSend, [this](QByteArray& data){  // data from controlstation (-> TCP -> PacketInterface) to rover
        QByteArray packetData;
        // drop crc (3), metainfo (data.at(0) + id & cmd) and copy actual data
        packetData.resize(data.size()-3-data.at(0)-2);
        quint8 recipientID = data.at(data.at(0));
        CMD_PACKET commandID = (CMD_PACKET)(quint8)data.at(data.at(0)+1);
        memcpy(packetData.data(), data.data()+data.at(0)+2, packetData.size());
        //qDebug() << data.size() << (quint8)data.at(0) << packetData.size();

        qDebug() << "Got packet for id:" << recipientID << "cmd:" << commandID;

        switch(commandID) {
        case CMD_PACKET::CMD_GET_STATE: {
            // TODO testing reply, double check message format!
            VByteArray ret;
            ret.vbAppendUint8(recipientID);
            ret.vbAppendUint8(commandID);
            ret.vbAppendUint8(12);
            ret.vbAppendUint8(3);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6);
            ret.vbAppendDouble32(0.0, 1e6); // Mag x
            ret.vbAppendDouble32(0.0, 1e6); // Mag y
            ret.vbAppendDouble32(0.0, 1e6); // Mag z
            ret.vbAppendDouble32(5.0, 1e4);
            ret.vbAppendDouble32(8.0, 1e4);
            ret.vbAppendDouble32(10.0, 1e6);
            ret.vbAppendDouble32(12.1, 1e6); // v_in
            ret.vbAppendDouble32(24.0, 1e6); // temp mos
            ret.vbAppendUint8(0); // MC Fault code
            ret.vbAppendDouble32(0.0, 1e4); // PX GPS
            ret.vbAppendDouble32(0.0, 1e4); // PY GPS
            ret.vbAppendDouble32(0.0, 1e4);
            ret.vbAppendDouble32(0.0, 1e4);
            ret.vbAppendDouble32(5, 1e6);
            ret.vbAppendInt32(utility::getTimeUtcToday());
            ret.vbAppendInt16(0);
            ret.vbAppendDouble32(0.0, 1e4); // UWB px
            ret.vbAppendDouble32(0.0, 1e4); // UWB PY
            mTcpServer.packet()->sendPacket(ret);
            break;
        }
        default:
            qDebug() << "WARNING: unhandled packet with command id" << commandID;
            break;
        }
    });
}

bool PacketInterfaceTCPServer::listen(quint16 port)
{
    return mTcpServer.startServer(port);
}

QSharedPointer<VehicleState> PacketInterfaceTCPServer::vehicleState() const
{
    return mVehicleState;
}

void PacketInterfaceTCPServer::setVehicleState(const QSharedPointer<VehicleState> &vehicleState)
{
    mVehicleState = vehicleState;
}
