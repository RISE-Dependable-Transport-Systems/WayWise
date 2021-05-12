#include "packetinterfacetcpserver.h"
#include "datatypes.h"
#include <QTime>
#include <QDateTime>

PacketInterfaceTCPServer::PacketInterfaceTCPServer(QObject *parent) : QObject(parent)
{
    mTcpServer.setUsePacket(true);

    QObject::connect(mTcpServer.packet(), &Packet::packetReceived, [this](QByteArray& data){ // data from controlstation to rover
        VByteArray packetData;
        // drop id & cmd and copy actual data, TODO: unnecessary to copy
        packetData.resize(data.size()-2);
        quint8 recipientID = data.at(0);
        CMD_PACKET commandID = (CMD_PACKET)(quint8)data.at(1);
        memcpy(packetData.data(), data.data()+2, packetData.size());

//        if (commandID != CMD_GET_STATE)
//            qDebug() << "Got packet for id:" << recipientID << "cmd:" << commandID << "length:" << packetData.size();

        switch(commandID) {
        case CMD_HEARTBEAT: {
            //qDebug() << "Received heartbeat";
        }break;

        // --- Get state from vehicle
        case CMD_GET_STATE: {
            if (mVehicleState && mVehicleState->getId() == recipientID) {

                WayPointFollowerState wpFollowerState;
                if (mWaypointFollower)
                    wpFollowerState = mWaypointFollower->getCurrentState();

                VByteArray ret;
                ret.vbAppendUint8(mVehicleState->getId());
                ret.vbAppendUint8(commandID);
                ret.vbAppendUint8(firmware_version_major);
                ret.vbAppendUint8(firmware_version_minor);
                // TODO: conf GNSS/Odo orientation fuse from PacketInterface
//                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getRoll(), 1e6);
//                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getPitch(), 1e6);
//                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getYaw(), 1e6); // yaw in degrees
                ret.vbAppendDouble32(mVehicleState->getPosition().getRoll(), 1e6);
                ret.vbAppendDouble32(mVehicleState->getPosition().getPitch(), 1e6);
                ret.vbAppendDouble32(mVehicleState->getPosition().getYaw()*180.0/M_PI, 1e6); // yaw in degrees
                ret.vbAppendDouble32(mVehicleState->getAccelerometerXYZ()[0], 1e6); // accel_x in g
                ret.vbAppendDouble32(mVehicleState->getAccelerometerXYZ()[1], 1e6); // accel_y in g
                ret.vbAppendDouble32(mVehicleState->getAccelerometerXYZ()[2], 1e6); // accel_z in g
                ret.vbAppendDouble32(mVehicleState->getGyroscopeXYZ()[0]*M_PI/180.0, 1e6); // roll_rate in radians per second
                ret.vbAppendDouble32(mVehicleState->getGyroscopeXYZ()[1]*M_PI/180.0, 1e6); // pitch_rate in radians per second
                ret.vbAppendDouble32(mVehicleState->getGyroscopeXYZ()[2]*M_PI/180.0, 1e6); // yaw_rate in radians per second
                ret.vbAppendDouble32(0.0, 1e6); // magnet_x
                ret.vbAppendDouble32(0.0, 1e6); // magnet_y
                ret.vbAppendDouble32(0.0, 1e6); // magnet_z
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getX(), 1e4); // Should be PosType::fused
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getY(), 1e4); // Should be PosType::fused
                ret.vbAppendDouble32(mVehicleState->getSpeed(), 1e6);
                ret.vbAppendDouble32(-1.0, 1e6); // v_in
                ret.vbAppendDouble32(-1.0, 1e6); // temp mos
                ret.vbAppendUint8(0); // MC Fault code
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getX(), 1e4);
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::GNSS).getY(), 1e4);
                ret.vbAppendDouble32(wpFollowerState.currentGoal.getX(), 1e4); // autopilot_goal_x
                ret.vbAppendDouble32(wpFollowerState.currentGoal.getY(), 1e4); // autopilot_goal_y
                ret.vbAppendDouble32(wpFollowerState.purePursuitRadius, 1e6); // autopilot_pp_radius
                ret.vbAppendInt32(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()).msecsSinceStartOfDay());
                ret.vbAppendInt16(0); // autopilot_route_left
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::UWB).getX(), 1e4); // UWB px
                ret.vbAppendDouble32(mVehicleState->getPosition(PosType::UWB).getY(), 1e4); // UWB PY
                mTcpServer.packet()->sendPacket(ret);
            }
        } break;
        // --- Set state on vehicle
        case CMD_SET_POS:
        case CMD_SET_ENU_REF: {
            llh_t enuRef = {packetData.vbPopFrontDouble64(1e16), packetData.vbPopFrontDouble64(1e16), packetData.vbPopFrontDouble32(1e3)};
            //qDebug() << "EnuRef received:" << enuRef.latitude << enuRef.longitude << enuRef.height;
            mUbloxRover->setEnuRef(enuRef);

            if (commandID == CMD_SET_ENU_REF) {
                VByteArray ack;
                ack.vbAppendUint8(mVehicleState->getId());
                ack.vbAppendUint8(commandID);
                mTcpServer.packet()->sendPacket(ack);
            }
        } break;
        case CMD_SET_POS_ACK: {
            PosPoint tmpPos = mVehicleState->getPosition();
            tmpPos.setX(packetData.vbPopFrontDouble32(1e4));
            tmpPos.setY(packetData.vbPopFrontDouble32(1e4));
            tmpPos.setYaw(packetData.vbPopFrontDouble32(1e6) * M_PI/180.0);
            mVehicleState->setPosition(tmpPos);

            if (commandID == CMD_SET_POS_ACK) {
                VByteArray ack;
                ack.vbAppendUint8(mVehicleState->getId());
                ack.vbAppendUint8(commandID);
                mTcpServer.packet()->sendPacket(ack);
            }
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

//            steering *= mAutoPilot->autopilot_get_steering_scale();
//            mAutoPilot->autopilot_set_active(false);
            mMovementController->setDesiredSteering(steering);
            mMovementController->setDesiredSpeed(throttle*3.0); // TODO!

            // TODO:
//            switch (mode) {
//            case RC_MODE_CURRENT:
//                mMotor->setControl(MotorSim::MOTOR_CONTROL_CURRENT, throttle);
//                break;

//            case RC_MODE_DUTY:
//                utility::truncateNumber(&throttle, -1.0, 1.0);
//                mMotor->setControl(MotorSim::MOTOR_CONTROL_DUTY, throttle);
//                break;

//            case RC_MODE_PID: // In m/s
//                setMotorSpeed(throttle);
//                break;

//            case RC_MODE_CURRENT_BRAKE:
//                mMotor->setControl(MotorSim::MOTOR_CONTROL_CURRENT_BRAKE, throttle);
//                break;

//            default:
//                break;
//            }
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
                    newPoint.setTime(packetData.vbPopFrontInt32());
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
        case CMD_AP_SET_ACTIVE: {
            if (mWaypointFollower)  {
                bool activateAutopilot = packetData.vbPopFrontInt8();
                bool resetAutopilotState = packetData.vbPopFrontInt8();
                if (activateAutopilot)
                    mWaypointFollower->startFollowingRoute(resetAutopilotState);
                else {
                    mWaypointFollower->stopFollowingRoute();
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
