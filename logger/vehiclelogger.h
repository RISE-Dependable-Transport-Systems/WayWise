/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#ifndef VEHICLELOGGER_H
#define VEHICLELOGGER_H

#include <QObject>
#include <QDebug>
#include "vehiclelogsignalrelay.h"

class VehicleLogger : public QObject
{
    Q_OBJECT
public:
    explicit VehicleLogger(QObject *parent = nullptr);

    static void init();

    static void shutDown();

    static void messageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg);

private:
    static bool isInit;

};

#endif // VEHICLELOGGER_H
