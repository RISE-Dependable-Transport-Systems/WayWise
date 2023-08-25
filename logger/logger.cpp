/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <mavsdk/mavsdk.h>
#include <QStandardPaths>
#include "logger.h"

QFile* Logger::logFile = Q_NULLPTR;
bool Logger::isInit = false;

Logger::Logger(QObject *parent)
    : QObject{parent}
{

}

Logger::~Logger()
{
    if(logFile != Q_NULLPTR) {
        logFile->close();
        delete logFile;
    }

    qInstallMessageHandler(0);  // detach qInstallMessageHandler
}

void Logger::initGroundStation()
{
    if(isInit)  return;

    QDir documentsDirectory(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation));   // Documents directory (OS agnostic)

    QString folderName = "ControlTower Logs";
    QString folderPath = documentsDirectory.filePath(folderName);

    if (!documentsDirectory.exists(folderPath))
    {
        if (documentsDirectory.mkdir(folderPath))   qDebug() << "Logs folder created";
        else                                        qDebug() << "Failed to create Logs folder.";
    }

    logFile = new QFile;

    QString fileName = QString("%1/Log %2.log").arg(folderPath).arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss"));

    logFile->setFileName(fileName);
    logFile->open(QIODevice::Append | QIODevice::Text);

    qInstallMessageHandler(Logger::messageOutputGroundStation);

    Logger::isInit = true;
}

void Logger::initVehicle()
{
    if(isInit)  return;

    qInstallMessageHandler(Logger::messageOutputVehicle);

    Logger::isInit = true;
}

void Logger::messageOutputGroundStation(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QString log;

    switch (type) {
    case QtDebugMsg:
        log = QObject::tr("%1 | Debug: %3").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        break;
    case QtInfoMsg:
        log = QObject::tr("%1 | Info: %3").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        break;
    case QtWarningMsg:
        log = QObject::tr("%1 | Warning: %3").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        break;
    case QtCriticalMsg:
        log = QObject::tr("%1 | Critical: %3").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        break;
    case QtFatalMsg:
        log = QObject::tr("%1 | Fatal: %3").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        break;
    }

    fprintf(stderr, "%s\n", qPrintable(log));

    static QStringList logQueue;

    if(Logger::getInstance().isConnectedGroundStation()) {
        if(!logQueue.empty())
            for(const QString &log : logQueue)
                Logger::getInstance().emit logSentGroundStation(log);

        logQueue.clear();

        Logger::getInstance().emit logSentGroundStation(log);
    } else
        logQueue.append(log);

    log.append("\n");
    logFile->write(log.toLocal8Bit());
    logFile->flush();
}

void Logger::messageOutputVehicle(QtMsgType type, const QMessageLogContext &context, const QString &msg)
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

    struct logQueueItem {
        QString log;
        uint8_t severity;
    };

    static QVector<logQueueItem> logQueue;

    if(Logger::getInstance().isConnectedVehicle()) {
        if(!logQueue.isEmpty())
            for(const logQueueItem &item : logQueue)
                Logger::getInstance().emit logSentVehicle(item.log, item.severity);

        logQueue.clear();

        Logger::getInstance().emit logSentVehicle(msg, msgSeverity);
    } else {
        logQueueItem item;

        item.log = msg;
        item.severity = msgSeverity;

        logQueue.push_back(item);
    }
}
