#include "packetinterfacetcpserver.h"
#include <QTime>
#include <QDateTime>
#include <QCoreApplication>

PacketInterfaceTCPServer::PacketInterfaceTCPServer(QObject *parent) : QObject(parent)
{
    mTcpServer.setUsePacket(true);

    // --- Handle heartbeat from base station
    mHeartbeat = false;
    PacketInterfaceTCPServer::mHeartbeatTimer.setSingleShot(true);
    connect(&mHeartbeatTimer, &QTimer::timeout, this, &PacketInterfaceTCPServer::heartbeatTimeout);

    QObject::connect(mTcpServer.packet(), &Packet::packetReceived, [this](QByteArray& data){ // data from controlstation to rover
        VByteArray packetData;
        // drop id & cmd and copy actual data, TODO: unnecessary to copy
        packetData.resize(data.size()-2);
        quint8 recipientID = data.at(0);
        CMD_PACKET commandID = (CMD_PACKET)(quint8)data.at(1);
        memcpy(packetData.data(), data.data()+2, packetData.size());

//        if (commandID != CMD_GET_STATE)
//            qDebug() << "Got packet for id:" << recipientID << "cmd:" << commandID << "length:" << packetData.size();

        if (!mVehicleState) { // TODO: might make sense to set VehicleState in constructor as PacketInterfaceTCPServer does not make sense without it.
            qDebug() << "WARNING: VehicleState unset in PacketInterfaceTCPServer, unable to handle messages.";
            return;
        }


        switch(commandID) {
        // --- Track base station heartbeat
        case CMD_HEARTBEAT: {
            if (!mHeartbeat) {
                qDebug() << "Heartbeat reset";
            }
            mHeartbeatTimer.start(mCountdown_ms);
            mHeartbeat = true;

            // Send ack
            VByteArray ack;
            ack.vbAppendUint8(mVehicleState->getId());
            ack.vbAppendUint8(commandID);
            mTcpServer.packet()->sendPacket(ack);
        } break;

        if(mHeartbeat) {
            // --- Use differential corrections data from control station
            case CMD_SEND_RTCM_USB: {
                if (mUbloxRover)
                    mUbloxRover->writeRtcmToUblox(packetData);

                emit rtcmData(packetData);
            } break;

            // --- Get state from vehicle
            case CMD_GET_STATE: {
                if (mVehicleState->getId() == recipientID) {
                    VByteArray ret;
                    ret.vbAppendUint8(mVehicleState->getId());
                    ret.vbAppendUint8(commandID);
                    ret.vbAppendUint8(firmware_version_major);
                    ret.vbAppendUint8(firmware_version_minor);
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::fused).getRoll(), 1e6);
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::fused).getPitch(), 1e6);
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::fused).getYaw(), 1e6); // yaw in degrees
                    ret.vbAppendDouble32(mVehicleState->getAccelerometerXYZ()[0], 1e6); // accel_x in g
                    ret.vbAppendDouble32(mVehicleState->getAccelerometerXYZ()[1], 1e6); // accel_y in g
                    ret.vbAppendDouble32(mVehicleState->getAccelerometerXYZ()[2], 1e6); // accel_z in g
                    ret.vbAppendDouble32(mVehicleState->getGyroscopeXYZ()[0]*M_PI/180.0, 1e6); // roll_rate in radians per second
                    ret.vbAppendDouble32(mVehicleState->getGyroscopeXYZ()[1]*M_PI/180.0, 1e6); // pitch_rate in radians per second
                    ret.vbAppendDouble32(mVehicleState->getGyroscopeXYZ()[2]*M_PI/180.0, 1e6); // yaw_rate in radians per second
                    ret.vbAppendDouble32(0.0, 1e6); // magnet_x
                    ret.vbAppendDouble32(0.0, 1e6); // magnet_y
                    ret.vbAppendDouble32(0.0, 1e6); // magnet_z
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::fused).getX(), 1e4);
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::fused).getY(), 1e4);
                    ret.vbAppendDouble32(mVehicleState->getSpeed(), 1e6);
                    ret.vbAppendDouble32(mMotorControllerStatus.voltageInput, 1e6);
                    ret.vbAppendDouble32(mMotorControllerStatus.temperature, 1e6);
                    ret.vbAppendUint8(mMotorControllerStatus.errorID);
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getX(), 1e4);
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getY(), 1e4);
                    ret.vbAppendDouble32(mWaypointFollower ? mWaypointFollower->getCurrentGoal().getX() : 0.0, 1e4); // autopilot_goal_x
                    ret.vbAppendDouble32(mWaypointFollower ? mWaypointFollower->getCurrentGoal().getY() : 0.0, 1e4); // autopilot_goal_y
                    ret.vbAppendDouble32(mWaypointFollower ? mWaypointFollower->getPurePursuitRadius() : 0.0, 1e6); // autopilot_pp_radius
                    ret.vbAppendInt32(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()).msecsSinceStartOfDay());
                    ret.vbAppendInt16(0); // autopilot_route_left
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::UWB).getX(), 1e4); // UWB px
                    ret.vbAppendDouble32(mVehicleState->getPosition(PosType::UWB).getY(), 1e4); // UWB PY
                    mTcpServer.packet()->sendPacket(ret);
                }
            } break;
            // --- Set state on vehicle
            case CMD_SET_POS:
            case CMD_SET_POS_ACK: {
                PosPoint tmpPos = mVehicleState->getPosition(PosType::fused);
                tmpPos.setX(packetData.vbPopFrontDouble32(1e4));
                tmpPos.setY(packetData.vbPopFrontDouble32(1e4));
                tmpPos.setYaw(packetData.vbPopFrontDouble32(1e6) * M_PI/180.0);
                mVehicleState->setPosition(tmpPos);

                // Send ack
                if (commandID == CMD_SET_POS_ACK) {
                    VByteArray ack;
                    ack.vbAppendUint8(mVehicleState->getId());
                    ack.vbAppendUint8(commandID);
                    mTcpServer.packet()->sendPacket(ack);
                }
            } break;
            // --- Set base station reference point on rover
            case CMD_SET_ENU_REF: {
                llh_t enuRef = {packetData.vbPopFrontDouble64(1e16), packetData.vbPopFrontDouble64(1e16), packetData.vbPopFrontDouble32(1e3)};
                //qDebug() << "EnuRef received:" << enuRef.latitude << enuRef.longitude << enuRef.height;

                if (mUbloxRover) {
                    mUbloxRover->setEnuRef(enuRef);

                    // Send ack
                    VByteArray ack;
                    ack.vbAppendUint8(mVehicleState->getId());
                    ack.vbAppendUint8(commandID);
                    mTcpServer.packet()->sendPacket(ack);
                } else
                    qDebug() << "WARNING: Unhandled CMD_SET_ENU_REF";
            } break;
            // --- Get base station reference point from rover
            case CMD_GET_ENU_REF: {
                if (mUbloxRover && mVehicleState->getId() == recipientID) {
                    llh_t enuRef = mUbloxRover->getEnuRef();

                    VByteArray ret;
                    ret.vbAppendUint8(mVehicleState->getId());
                    ret.vbAppendUint8(commandID);
                    ret.vbAppendDouble64(enuRef.latitude, 1e16);
                    ret.vbAppendDouble64(enuRef.longitude, 1e16);
                    ret.vbAppendDouble32(enuRef.height, 1e3);
                    mTcpServer.packet()->sendPacket(ret);
                } else
                    qDebug() << "WARNING: Unhandled CMD_GET_ENU_REF";
            } break;
            // --- Remote control vehicle
            case CMD_RC_CONTROL: {
                if (!mMovementController) // we have nowhere to send the control input to
                    break;

                RC_MODE mode;
                double throttle, steering;
                mode = (RC_MODE)packetData.vbPopFrontUint8();
                throttle = packetData.vbPopFrontDouble32(1e4);
                steering = packetData.vbPopFrontDouble32(1e6);

                if (mWaypointFollower && mWaypointFollower->isActive())
                    mWaypointFollower->stop();

                // NOTE: mode is ignored. Instead, throttle [0.0:1.0] is multiplied by 10 m/s.
                // Handling mode would allow interact more closely with motor controller using
                // CMD_RC_CONTROL, but not currently not relevant for us.
                Q_UNUSED(mode)
                mMovementController->setDesiredSteering(steering);
                mMovementController->setDesiredSpeed(throttle * 10.0);
            } break;
            // --- Autopilot
            case CMD_AP_CLEAR_POINTS: {
                if (mWaypointFollower)  {
                    mWaypointFollower->clearRoute();

                    // Send ack
                    VByteArray ack;
                    ack.vbAppendUint8(mVehicleState->getId());
                    ack.vbAppendUint8(commandID);
                    mTcpServer.packet()->sendPacket(ack);
                } else
                    qDebug() << "WARNING: unhandled CMD_AP_CLEAR_POINTS";
            } break;
            case CMD_AP_ADD_POINTS: {
                if (mWaypointFollower)  {
                    PosPoint newPoint;
                    while (!packetData.isEmpty()) {
                        newPoint.setX(packetData.vbPopFrontDouble32(1e4));
                        newPoint.setY(packetData.vbPopFrontDouble32(1e4));
                        newPoint.setHeight(packetData.vbPopFrontDouble32(1e4));
                        newPoint.setSpeed(packetData.vbPopFrontDouble32(1e6));
                        newPoint.setTime(QTime::fromMSecsSinceStartOfDay(packetData.vbPopFrontInt32()));
                        newPoint.setAttributes(packetData.vbPopFrontUint32());
                        mWaypointFollower->addWaypoint(newPoint);
                    }

                    // Send ack
                    VByteArray ack;
                    ack.vbAppendUint8(mVehicleState->getId());
                    ack.vbAppendUint8(commandID);
                    mTcpServer.packet()->sendPacket(ack);
                } else
                    qDebug() << "WARNING: unhandled CMD_AP_CLEAR_POINTS";
            } break;
            case CMD_SET_AP_MODE: {
                mode = (AP_MODE)packetData.vbPopFrontUint8();
            } break;
            case CMD_AP_SET_ACTIVE: {
                if (mWaypointFollower)  {
                    bool activateAutopilot = packetData.vbPopFrontInt8();
                    bool resetAutopilotState = packetData.vbPopFrontInt8();

                    if (activateAutopilot) {
                        switch (mode) {
                        case AP_MODE_FOLLOW_ROUTE:
                            mWaypointFollower->startFollowingRoute(resetAutopilotState);
                            break;
                        case AP_MODE_FOLLOW_ME:
                            mWaypointFollower->startFollowPoint();
                            break;
                        default:
                            break;
                        }
                    } else {
                            mWaypointFollower->stop();
                            if (resetAutopilotState)
                                mWaypointFollower->resetState();
                        }

                    // Send ack
                    VByteArray ack;
                    ack.vbAppendUint8(mVehicleState->getId());
                    ack.vbAppendUint8(commandID);
                    mTcpServer.packet()->sendPacket(ack);
                } else
                    qDebug() << "WARNING: unhandled CMD_AP_SET_ACTIVE";
            } break;

            case CMD_REBOOT_SYSTEM: {
                if (mUbloxRover) {
                    if (packetData.vbPopFrontUint8()) {
                        qDebug() << "\nSystem shutdown...";
                        mUbloxRover->saveOnShutdown();
                        QTimer::singleShot(3000, &QCoreApplication::quit);
                        // TODO: implement hardware shutdown
                    } else {
                        qDebug() << "\nSystem reboot...";
                        mUbloxRover->saveOnShutdown();
                        // TODO: implement hardware reboot
                    }

                    // Send ack
                    VByteArray ack;
                    ack.vbAppendUint8(mVehicleState->getId());
                    ack.vbAppendUint8(CMD_REBOOT_SYSTEM_ACK);
                    mTcpServer.packet()->sendPacket(ack);
                } else
                    qDebug() << "WARNING: unhandled CMD_REBOOT_SYSTEM";
                break;
            }

            default:
                qDebug() << "WARNING: unhandled packet with command id" << commandID;
                break;
            }
        }
    });
}

bool PacketInterfaceTCPServer::listen(quint16 port)
{
    return mTcpServer.startServer(port);
}

QSharedPointer<VehicleState> PacketInterfaceTCPServer::getVehicleState() const
{
    return mVehicleState;
}

void PacketInterfaceTCPServer::setVehicleState(const QSharedPointer<VehicleState> &vehicleState)
{
    mVehicleState = vehicleState;
}

QSharedPointer<MovementController> PacketInterfaceTCPServer::getMovementController() const
{
    return mMovementController;
}

void PacketInterfaceTCPServer::setMovementController(const QSharedPointer<MovementController> &movementController)
{
    mMovementController = movementController;
}

QSharedPointer<WaypointFollower> PacketInterfaceTCPServer::getWaypointFollower() const
{
    return mWaypointFollower;
}

void PacketInterfaceTCPServer::setWaypointFollower(const QSharedPointer<WaypointFollower> &waypointFollower)
{
    mWaypointFollower = waypointFollower;
}

void PacketInterfaceTCPServer::setUbloxRover(const QSharedPointer<UbloxRover> &uBloxRover)
{
    mUbloxRover = uBloxRover;
}

void PacketInterfaceTCPServer::heartbeatTimeout()
{
    mHeartbeat = false;
    qDebug() << "Heartbeat timeout";

    if (mWaypointFollower) {
        mWaypointFollower->stop();
     }
     if (mMovementController) {
              mMovementController->setDesiredSteering(0.0);
              mMovementController->setDesiredSpeed(0.0);
     }

    // ToDo: Set action for CMD_EMERGENCY_STOP in base station.
    // When connection is reestablished, a CMD_AP_SET_ACTIVE needs to be sent from RControlStation to
    // reactivate the autopilot.
}

void PacketInterfaceTCPServer::updateMotorControllerStatus(double rpm, int tachometer, int tachometer_abs, double voltageInput, double temperature, int errorID)
{
    mMotorControllerStatus = {rpm, tachometer, tachometer_abs, voltageInput, temperature, errorID};
}
