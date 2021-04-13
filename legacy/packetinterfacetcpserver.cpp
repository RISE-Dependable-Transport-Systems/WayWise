#include "packetinterfacetcpserver.h"
#include "datatypes.h"

PacketInterfaceTCPServer::PacketInterfaceTCPServer(QObject *parent) : QObject(parent)
{
    mTcpServer.setUsePacket(true);

    QObject::connect(mTcpServer.packet(), &Packet::packetReceived, [this](QByteArray& data){mPacketInterface.sendPacket(data);}); // feed incoming tcp data into "PacketInterface"

    QObject::connect(&mPacketInterface, &PacketInterface::dataToSend, [this](QByteArray& data){  // data from controlstation (-> TCP -> PacketInterface) to rover
        VByteArray packetData;
        // drop crc (3), metainfo (data.at(0) + id & cmd) and copy actual data
        packetData.resize(data.size()-3-data.at(0)-2);
        quint8 recipientID = data.at(data.at(0));
        CMD_PACKET commandID = (CMD_PACKET)(quint8)data.at(data.at(0)+1);
        memcpy(packetData.data(), data.data()+data.at(0)+2, packetData.size());
        //qDebug() << data.size() << (quint8)data.at(0) << packetData.size();

        if (commandID != CMD_GET_STATE)
            qDebug() << "Got packet for id:" << recipientID << "cmd:" << commandID << "length:" << packetData.size();

        switch(commandID) {
        case CMD_GET_STATE: {
            if (mVehicleState && mVehicleState->getId() == recipientID) {
                VByteArray ret;
                ret.vbAppendUint8(mVehicleState->getId());
                ret.vbAppendUint8(commandID);
                ret.vbAppendUint8(firmware_version_major);
                ret.vbAppendUint8(firmware_version_minor);
                ret.vbAppendDouble32(mVehicleState->getPosition().getRoll(), 1e6);
                ret.vbAppendDouble32(mVehicleState->getPosition().getPitch(), 1e6);
                ret.vbAppendDouble32(mVehicleState->getPosition().getYaw(), 1e6);
                ret.vbAppendDouble32(0.0, 1e6); // accel_x
                ret.vbAppendDouble32(0.0, 1e6); // accel_y
                ret.vbAppendDouble32(0.0, 1e6); // accel_z
                ret.vbAppendDouble32(0.0, 1e6); // roll_rate
                ret.vbAppendDouble32(0.0, 1e6); // pitch_rate
                ret.vbAppendDouble32(0.0, 1e6); // yaw_rate
                ret.vbAppendDouble32(0.0, 1e6); // magnet_x
                ret.vbAppendDouble32(0.0, 1e6); // magnet_y
                ret.vbAppendDouble32(0.0, 1e6); // magnet_z
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::fused).getX(), 1e4);
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::fused).getY(), 1e4);
                ret.vbAppendDouble32(mVehicleState->getSpeed(), 1e6);
                ret.vbAppendDouble32(-1.0, 1e6); // v_in
                ret.vbAppendDouble32(-1.0, 1e6); // temp mos
                ret.vbAppendUint8(0); // MC Fault code
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getX(), 1e4);
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getY(), 1e4);
                ret.vbAppendDouble32(0.0, 1e4); // autopilot_goal_x
                ret.vbAppendDouble32(0.0, 1e4); // autopilot_goal_y
                ret.vbAppendDouble32(5, 1e6); // autopilot_pp_radius
                ret.vbAppendInt32(utility::getTimeUtcToday());
                ret.vbAppendInt16(0); // autopilot_route_left
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::UWB).getX(), 1e4); // UWB px
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::UWB).getY(), 1e4); // UWB PY
                mTcpServer.packet()->sendPacket(ret);
            }
        } break;
        case CMD_SET_POS:
        case CMD_SET_POS_ACK: {
            PosPoint tmpPos = mVehicleState->getPosition();
            tmpPos.setX(packetData.vbPopFrontDouble32(1e4));
            tmpPos.setY(packetData.vbPopFrontDouble32(1e4));
            tmpPos.setYaw(packetData.vbPopFrontDouble32(1e6));
            mVehicleState->setPosition(tmpPos);

            if (commandID == CMD_SET_POS_ACK) {
                VByteArray ack;
                ack.vbAppendUint8(mVehicleState->getId());
                ack.vbAppendUint8(commandID);
                mTcpServer.packet()->sendPacket(ack);
            }
        } break;
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
