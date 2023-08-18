/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#ifndef VEHICLELOGSIGNALRELAY_H
#define VEHICLELOGSIGNALRELAY_H

#include <QObject>
#include <QMetaMethod>

class VehicleLogSignalRelay : public QObject {
    Q_OBJECT

public:
    static VehicleLogSignalRelay& getInstance() {
        static VehicleLogSignalRelay instance;
        return instance;
    }

    bool isConnected() {
        static const QMetaMethod logSentSignal = QMetaMethod::fromSignal(&VehicleLogSignalRelay::logSent);
        return isSignalConnected(logSentSignal);
    }

signals:
    void logSent(const QString& message, const uint8_t& severity);
};

#endif // VEHICLELOGSIGNALRELAY_H
