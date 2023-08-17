/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#include <QDateTime>
#include <QDir>
#include <QStandardPaths>
#include <QStringList>
#include "groundstationlogger.h"

QFile* GroundStationLogger::logFile = Q_NULLPTR;
bool GroundStationLogger::isInit = false;

GroundStationLogger::GroundStationLogger(QObject *parent) : QObject{parent}
{

}

void GroundStationLogger::init()
{
    if(isInit)  return;

    QDir documentsDirectory(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation));   // Documents directory (OS agnostic)

    QString folderName = "ControlTower Logs";
    QString folderPath = documentsDirectory.filePath(folderName);

    if (!documentsDirectory.exists(folderPath)) {
        if (documentsDirectory.mkdir(folderPath))   qDebug() << "Logs folder created";
        else                                        qDebug() << "Failed to create Logs folder.";
    }

    logFile = new QFile;

    QString fileName = QString("%1/Log %2.log").arg(folderPath).arg(QDateTime::currentDateTime().toString("dd-MM-yyyy hh:mm:ss"));

    logFile->setFileName(fileName);
    logFile->open(QIODevice::Append | QIODevice::Text);

    qInstallMessageHandler(GroundStationLogger::messageOutput);

    GroundStationLogger::isInit = true;
}

void GroundStationLogger::shutDown()
{
    if(logFile != Q_NULLPTR) {
        logFile->close();
        delete logFile;
    }

    qInstallMessageHandler(0);  // detach qInstallMessageHandler
}

void GroundStationLogger::messageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
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

    if(GroundStationLogSignalRelay::getInstance().isConnected()) {
        if(!logQueue.empty())
            for(const QString &log : logQueue)
                GroundStationLogSignalRelay::getInstance().emit logSent(log);

        logQueue.clear();

        GroundStationLogSignalRelay::getInstance().emit logSent(log);
    } else
        logQueue.append(log);

    log.append("\n");
    logFile->write(log.toLocal8Bit());
    logFile->flush();
}
