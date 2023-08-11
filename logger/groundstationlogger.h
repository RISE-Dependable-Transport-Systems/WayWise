/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#ifndef GROUNDSTATIONLOGGER_H
#define GROUNDSTATIONLOGGER_H

#include <QObject>
#include <QDebug>
#include <QFile>
#include "groundstationlogsignalrelay.h"

class GroundStationLogger : public QObject
{
    Q_OBJECT

public:
    explicit GroundStationLogger(QObject *parent = nullptr);

    static void init();

    static void shutDown();

    static void messageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg);

private:
    static QFile* logFile;

    static bool isInit;
};

#endif // GROUNDSTATIONLOGGER_H
