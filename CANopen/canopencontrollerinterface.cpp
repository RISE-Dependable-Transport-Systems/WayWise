#include <QAbstractEventDispatcher>
#include <QDebug>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <iostream>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include "canopencontrollerinterface.h"
#include "sdvp_qtcommon/CANopen/slave.h"

using namespace lely;

void CANopenControllerInterface::actualSpeedReceived(double speed) {
    emit sendActualSpeed(speed);
}

void CANopenControllerInterface::actualSteeringReceived(double steering) {
    emit sendActualSteeringCurvature(steering);
}

void CANopenControllerInterface::commandSpeedReceived(const double &speed) {
    emit sendCommandSpeed(speed);
}

void CANopenControllerInterface::commandSteeringReceived(const double &steering) {
    emit sendCommandSteering(steering);
}

void CANopenControllerInterface::commandStatusReceived(quint8 status) {
    emit sendCommandStatus(status);
}

void CANopenControllerInterface::actualStatusReceived(const quint8 &status) {
    emit sendActualStatus(status);
}

void CANopenControllerInterface::batterySOCReceived(double batterysoc) {
    emit sendBatterySOC(batterysoc);
}

void CANopenControllerInterface::batteryVoltageReceived(double batteryvoltage) {
    emit sendBatteryVoltage(batteryvoltage);
}

void CANopenControllerInterface::commandAttributesReceived(const quint32 &attributes) {
    emit sendCommandAttributes(attributes);
}

void CANopenControllerInterface::GNSSDataToCANReceived(const QVariant& gnssData) {
    emit sendGNSSDataToCAN(gnssData);
}

// --- PROCESS ---
// Start processing data.
void CANopenControllerInterface::startDevice() {
    // Create an I/O context to synchronize I/O services during shutdown.
    io::Context ctx;
    // Create an platform-specific I/O polling instance to monitor the CAN bus, as
    // well as timers and signals.
    io::Poll poll(ctx);
    // Create a polling event loop and pass it the platform-independent polling
    // interface. If no tasks are pending, the event loop will poll for I/O
    // events.
    ev::Loop loop(poll.get_poll());
    // I/O devices only need access to the executor interface of the event loop.
    auto exec = loop.get_executor();
    // Create a timer using a monotonic clock, i.e., a clock that is not affected
    // by discontinuous jumps in the system time.
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);

    try {
        // Create a virtual SocketCAN CAN controller and channel, and do not modify
        // the current CAN bus state or bitrate.
        //io::CanController ctrl("vcan0");
        io::CanController ctrl("can0");
        io::CanChannel chan(poll, exec);
        chan.open(ctrl);
        // Create a CANopen slave with node-ID 2.
        MySlave mSlave(timer, chan, "../sdvp_qtcommon/CANopen/cpp-slave.eds", "", 2);
        QObject::connect(&mSlave, SIGNAL(sendActualSpeed(double)), this, SLOT(actualSpeedReceived(double)));
        QObject::connect(this, SIGNAL(sendCommandSpeed(double)), &mSlave, SLOT(commandSpeedReceived(double)));
        QObject::connect(&mSlave, SIGNAL(sendActualSteering(double)), this, SLOT(actualSteeringReceived(double)));
        QObject::connect(this, SIGNAL(sendCommandSteering(double)), &mSlave, SLOT(commandSteeringReceived(double)));
        QObject::connect(&mSlave, SIGNAL(sendStatus(quint8)), this, SLOT(commandStatusReceived(quint8)));
        QObject::connect(this, SIGNAL(sendActualStatus(quint8)), &mSlave, SLOT(statusReceived(quint8)));
        QObject::connect(&mSlave, SIGNAL(sendBatterySOC(double)), this, SLOT(batterySOCReceived(double)));
        QObject::connect(&mSlave, SIGNAL(sendBatteryVoltage(double)), this, SLOT(batteryVoltageReceived(double)));
        QObject::connect(this, SIGNAL(sendCommandAttributes(quint32)), &mSlave, SLOT(commandAttributesReceived(quint32)));
        QObject::connect(this, SIGNAL(sendGNSSDataToCAN(QVariant)), &mSlave, SLOT(GNSSDataToCANReceived(QVariant)));
        // Create a signal handler.
        io::SignalSet sigset(poll, exec);
        // Watch for Ctrl+C or process termination.
        sigset.insert(SIGHUP);
        sigset.insert(SIGINT);
        sigset.insert(SIGTERM);
        // Submit a task to be executed when a signal is raised. We don't care which.
        sigset.submit_wait([&](int /*signo*/) {
          // If the signal is raised again, terminate immediately.
          sigset.clear();
          // Perform a clean shutdown.
          qDebug() << "Shutting down CAN";
          ctx.shutdown();
          emit finished();
        });
        // Start the NMT service of the slave by pretending to receive a 'reset node'
        // command.
        mSlave.Reset();
        // Run the event loop once, then check for Qt events
        while (true) {
            loop.run_one();
            thread()->eventDispatcher()->processEvents(QEventLoop::ProcessEventsFlag::AllEvents);
        }
    } catch (...) { // TODO: handle specific exception
        qDebug() << "WARNING: CANopenControllerInterface could not open CAN device, trying to activate simulation.";
        emit activateSimulation();
    }
}
