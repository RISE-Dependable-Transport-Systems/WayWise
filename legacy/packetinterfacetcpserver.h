#ifndef PACKETINTERFACETCPSERVER_H
#define PACKETINTERFACETCPSERVER_H

#include <QObject>
#include <sdvp_qtcommon/legacy/tcpserversimple.h>
//#include <sdvp_qtcommon/legacy/utility.h>
#include <sdvp_qtcommon/vbytearray.h>
#include <sdvp_qtcommon/vehiclestate.h>
#include <sdvp_qtcommon/movementcontroller.h>
#include <sdvp_qtcommon/waypointfollower.h>
#include <sdvp_qtcommon/gnss/ubloxrover.h>
#include <sdvp_qtcommon/legacy/datatypes.h>
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
    } mMotorControllerStatus;

};

#endif // PACKETINTERFACETCPSERVER_H
