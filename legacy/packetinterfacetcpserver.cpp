#include "packetinterfacetcpserver.h"

PacketInterfaceTCPServer::PacketInterfaceTCPServer(QObject *parent) : QObject(parent)
{
    mTcpServer.setUsePacket(true);

    QObject::connect(mTcpServer.packet(), &Packet::packetReceived, [this](QByteArray& data){mPacketInterface.sendPacket(data);}); // feed incoming tcp data into "PacketInterface"

    QObject::connect(&mPacketInterface, &PacketInterface::dataToSend, [this](QByteArray& data){  // data from controlstation (-> TCP -> PacketInterface) to rover
        QByteArray packetData;
        // drop crc and copy actual data
        packetData.resize(data.size()-data.at(0)-3);
        memcpy(packetData.data(), data.data()+data.at(0), packetData.size());

        qDebug() << "Got packet for id:" << (quint8)packetData.at(0) << "cmd:" << (quint8)packetData.at(1);

        // TODO testing reply
        VByteArray ret;
        ret.vbAppendUint8(packetData.at(0));
        ret.vbAppendUint8(packetData.at(1));
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
    });
}

bool PacketInterfaceTCPServer::listen(quint16 port)
{
    return mTcpServer.startServer(port);
}
