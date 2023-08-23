/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QMetaMethod>
#include <QFile>

class Logger : public QObject {
    Q_OBJECT
public:
    static Logger& getInstance()
    {
        static Logger instance;
        return instance;
    }

    bool isConnectedGroundStation() {
        static const QMetaMethod logSentSignal = QMetaMethod::fromSignal(&Logger::logSentGroundStation);
        return isSignalConnected(logSentSignal);
    }

    bool isConnectedVehicle() {
        static const QMetaMethod logSentSignal = QMetaMethod::fromSignal(&Logger::logSentVehicle);
        return isSignalConnected(logSentSignal);
    }

    static void initGroundStation();

    static void initVehicle();

    static void messageOutputGroundStation(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static void messageOutputVehicle(QtMsgType type, const QMessageLogContext& context, const QString& msg);

private:
    explicit Logger(QObject *parent = nullptr);

    ~Logger();

    static QFile* logFile;

    static bool isInit;

signals:
    void logSentGroundStation(const QString& message);

    void logSentVehicle(const QString& message, const uint8_t& severity);
};

#endif // LOGGER_H
