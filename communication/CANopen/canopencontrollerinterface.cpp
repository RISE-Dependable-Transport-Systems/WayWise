/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *               2021 Rickard Häll      rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 */

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
#include "slave.h"

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
        MySlave mSlave(timer, chan, "../sdvp_qtcommon/communication/CANopen/cpp-slave.eds", "", 2);

        QObject::connect(&mSlave, &MySlave::sendActualSpeed, this, &CANopenControllerInterface::actualSpeedReceived);
        QObject::connect(&mSlave, &MySlave::sendActualSteering, this, &CANopenControllerInterface::actualSteeringReceived);
        QObject::connect(&mSlave, &MySlave::sendStatus, this, &CANopenControllerInterface::commandStatusReceived);
        QObject::connect(&mSlave, &MySlave::sendBatterySOC, this, &CANopenControllerInterface::batterySOCReceived);
        QObject::connect(&mSlave, &MySlave::sendBatteryVoltage, this, &CANopenControllerInterface::batteryVoltageReceived);
        QObject::connect(this, &CANopenControllerInterface::sendCommandSpeed, &mSlave, &MySlave::commandSpeedReceived);
        QObject::connect(this, &CANopenControllerInterface::sendCommandSteering, &mSlave, &MySlave::commandSteeringReceived);
        QObject::connect(this, &CANopenControllerInterface::sendActualStatus, &mSlave, &MySlave::statusReceived);
        QObject::connect(this, &CANopenControllerInterface::sendCommandAttributes, &mSlave, &MySlave::commandAttributesReceived);
        QObject::connect(this, &CANopenControllerInterface::sendGNSSDataToCAN, &mSlave, &MySlave::GNSSDataToCANReceived);

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
