/*
 *  Author: Aria Mirzai, aria.mirzai@ri.se (2023)
 */

#ifndef GROUNDSTATIONLOGSIGNALRELAY_H
#define GROUNDSTATIONLOGSIGNALRELAY_H

#include <QObject>
#include <QMetaMethod>

class GroundStationLogSignalRelay : public QObject {
    Q_OBJECT

public:
    static GroundStationLogSignalRelay& getInstance() {
        static GroundStationLogSignalRelay instance;
        return instance;
    }

    bool isConnected() {
        static const QMetaMethod logSentSignal = QMetaMethod::fromSignal(&GroundStationLogSignalRelay::logSent);
        return isSignalConnected(logSentSignal);
    }

signals:
    void logSent(const QString& message);
};

#endif // GROUNDSTATIONLOGSIGNALRELAY_H
