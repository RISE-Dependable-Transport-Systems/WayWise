/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Legacy class for implementing "PacketInterface" in TCP/IP, i.e., the communication protocol between RControlStation and vehicle.
 */

#ifndef PACKETINTERFACETCPSERVER_H
#define PACKETINTERFACETCPSERVER_H

#include <QObject>
#include "legacy/tcpserversimple.h"
#include "core/vbytearray.h"
#include "vehicles/vehiclestate.h"
#include "vehicles/controller/movementcontroller.h"
#include "autopilot/waypointfollower.h"
#include "sensors/gnss/ubloxrover.h"
#include "legacy/datatypes.h"
#include <QTimer>

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

    QSharedPointer<WaypointFollower> getWaypointFollower() const;
    void setWaypointFollower(const QSharedPointer<WaypointFollower> &waypointFollower);

    QSharedPointer<UbloxRover> getUbloxRover() const;
    void setUbloxRover(const QSharedPointer<UbloxRover> &uBloxRover);

    void heartbeatTimeout();

    void updateMotorControllerStatus(double rpm, int tachometer, int tachometer_abs, double voltageInput, double temperature, int errorID);

signals:
    void rtcmData(const QByteArray &data);

private:
    const int firmware_version_major = 20;
    const int firmware_version_minor = 1;

    TcpServerSimple mTcpServer;
    QSharedPointer<VehicleState> mVehicleState;
    QSharedPointer<MovementController> mMovementController;
    QSharedPointer<WaypointFollower> mWaypointFollower;
    QSharedPointer<UbloxRover> mUbloxRover;

    const unsigned mCountdown_ms = 1000;
    QTimer mHeartbeatTimer;
    bool mHeartbeat;
    AP_MODE mode;

    // NOTE: quite VESC specific, but OK in legacy parts
    struct {
        double rpm;
        int tachometer;
        int tachometer_abs;
        double voltageInput;
        double temperature;
        int errorID;
    } mMotorControllerStatus = {0.0, 0, 0, 0.0, 0.0, 0};

};

#endif // PACKETINTERFACETCPSERVER_H
