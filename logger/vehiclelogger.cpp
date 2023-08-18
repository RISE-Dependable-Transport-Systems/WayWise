/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#include <mavsdk/mavsdk.h>
#include <QVector>
#include "vehiclelogger.h"

bool VehicleLogger::isInit = false;

struct logQueueItem {
    QString log;
    uint8_t severity;
};

static QVector<logQueueItem> logQueue;

VehicleLogger::VehicleLogger(QObject *parent)
    : QObject{parent}
{

}

void VehicleLogger::init()
{
    if(isInit)  return;

    qInstallMessageHandler(messageOutput);

    VehicleLogger::isInit = true;
}

void VehicleLogger::shutDown()
{
    qInstallMessageHandler(0);  // detach qInstallMessageHandler
}

void VehicleLogger::messageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    uint8_t msgSeverity;

    switch (type) {
    case QtDebugMsg:
        msgSeverity = MAV_SEVERITY_DEBUG;
        fprintf(stderr, "Debug: %s\n", qPrintable(msg));
        break;
    case QtInfoMsg:
        msgSeverity = MAV_SEVERITY_INFO;
        fprintf(stderr, "Info: %s\n", qPrintable(msg));
        break;
    case QtWarningMsg:
        msgSeverity = MAV_SEVERITY_WARNING;
        fprintf(stderr, "Warning: %s\n", qPrintable(msg));
        break;
    case QtCriticalMsg:
        msgSeverity = MAV_SEVERITY_CRITICAL;
        fprintf(stderr, "Critical: %s\n", qPrintable(msg));
        break;
    case QtFatalMsg:
        msgSeverity = MAV_SEVERITY_EMERGENCY;
        fprintf(stderr, "Fatal: %s\n", qPrintable(msg));
        break;
    }

    if(VehicleLogSignalRelay::getInstance().isConnected()) {
        if(!logQueue.isEmpty())
            for(const logQueueItem &item : logQueue)
                VehicleLogSignalRelay::getInstance().emit logSent(item.log, item.severity);

        logQueue.clear();

        VehicleLogSignalRelay::getInstance().emit logSent(msg, msgSeverity);
    } else {
        logQueueItem item;

        item.log = msg;
        item.severity = msgSeverity;

        logQueue.push_back(item);
    }
}
