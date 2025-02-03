/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QMetaMethod>
#include <QFile>

class Logger : public QObject {
    Q_OBJECT
public:
    static Logger& getInstance();

    bool isConnected();

    static void initGroundStation();

    static void initVehicle();

    static void messageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg);

private:
    explicit Logger(QObject *parent = nullptr);

    ~Logger();

    static QFile* logFile;

    static bool isInit;

signals:
    void logSent(const QString& message, const quint8& severity);
};

#endif // LOGGER_H
