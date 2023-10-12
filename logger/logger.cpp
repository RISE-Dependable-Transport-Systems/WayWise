/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <mavsdk/mavsdk.h>
#include <QStandardPaths>
#include "logger.h"

QFile* Logger::logFile = nullptr;
bool Logger::isInit = false;

Logger::Logger(QObject *parent)
    : QObject{parent}
{

}

Logger::~Logger()
{
    if(logFile) {
        logFile->close();
        delete logFile;
    }

    qInstallMessageHandler(0);  // detach qInstallMessageHandler
}

Logger& Logger::getInstance()
{
    static Logger instance;
    return instance;
}

bool Logger::isConnected()
{
    static const QMetaMethod logSentSignal = QMetaMethod::fromSignal(&Logger::logSent);
    return isSignalConnected(logSentSignal);
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

    QString fileName = QString("%1/Log %2.log").arg(folderPath).arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh-mm-ss"));

    logFile->setFileName(fileName);
    logFile->open(QIODevice::Append | QIODevice::Text);

    qInstallMessageHandler(Logger::messageOutput);

    Logger::isInit = true;
}

void Logger::initVehicle()
{
    if(isInit)  return;

    qInstallMessageHandler(Logger::messageOutput);

    Logger::isInit = true;
}

void Logger::messageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    struct logItem {
        QString log;
        quint8 severity;
    };

    logItem newItem;

    switch (type) {
    case QtDebugMsg:
        newItem.log = QObject::tr("%1 | Debug: %2").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        newItem.severity = MAV_SEVERITY_DEBUG;
        break;
    case QtInfoMsg:
        newItem.log = QObject::tr("%1 | Info: %2").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        newItem.severity = MAV_SEVERITY_INFO;
        break;
    case QtWarningMsg:
        newItem.log = QObject::tr("%1 | Warning: %2").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        newItem.severity = MAV_SEVERITY_WARNING;
        break;
    case QtCriticalMsg:
        newItem.log = QObject::tr("%1 | Critical: %2").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        newItem.severity = MAV_SEVERITY_CRITICAL;
        break;
    case QtFatalMsg:
        newItem.log = QObject::tr("%1 | Fatal: %2").arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss")).arg(qPrintable(msg));
        newItem.severity = MAV_SEVERITY_EMERGENCY;
        break;
    }

    fprintf(stderr, "%s\n", qPrintable(newItem.log));

    static QVector<logItem> logQueue;

    if(Logger::getInstance().isConnected()) {
        if(!logQueue.isEmpty())
            for(const logItem &item : logQueue)
                Logger::getInstance().emit logSent(item.log, item.severity);

        logQueue.clear();

        Logger::getInstance().emit logSent(newItem.log, newItem.severity);
    } else
        logQueue.push_back(newItem);

    if(logFile) {
        newItem.log.append("\n");
        logFile->write(newItem.log.toLocal8Bit());
        logFile->flush();
    }
}
