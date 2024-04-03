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
    static Logger& getInstance();

    bool isConnected();

    static void setVerbose(bool v);

    static bool getVerbose();

    static void initGroundStation();

    static void initVehicle();

    static void messageOutput(QtMsgType type, const QMessageLogContext& context, const QString& msg);

private:
    explicit Logger(QObject *parent = nullptr);

    ~Logger();

    static QFile* logFile;

    static bool isInit;

    static bool verbose;

signals:
    void logSent(const QString& message, const quint8& severity);
};

#endif // LOGGER_H
