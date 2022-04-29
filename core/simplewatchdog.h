/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * A simple class to detect when Qt's event loop takes longer than expected
 */

#ifndef SIMPLEWATCHDOG_H
#define SIMPLEWATCHDOG_H

#include <QObject>
#include <QTimer>

class SimpleWatchdog : public QObject
{
    Q_OBJECT
public:
    explicit SimpleWatchdog(QObject *parent = nullptr);

    int getTimeout() const;
    void setTimeout(const int &value_ms);

    int getTimeoutTolerance() const;
    void setTimeoutTolerance(const int &value_ms);

signals:
    void timeout(int timeTaken_ms);

private:
    QTimer mWatchdogTimer;
    int timeout_ms = 20;
    int timeout_tolerance_ms = 5;

};

#endif // SIMPLEWATCHDOG_H
