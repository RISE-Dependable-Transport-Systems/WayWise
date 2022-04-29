/*
 *     Copyright 2021 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "simplewatchdog.h"
#include <QTime>
#include <QDebug>

SimpleWatchdog::SimpleWatchdog(QObject *parent) : QObject(parent)
{
    connect(&mWatchdogTimer, &QTimer::timeout, [this](){
        static QTime last = QTime::currentTime();
        int timeTaken_ms = QTime::currentTime().msecsSinceStartOfDay() - last.msecsSinceStartOfDay();

        if (timeTaken_ms > timeout_ms + timeout_tolerance_ms) {
            qDebug() << "WARNING: SimpleWatchdog timed out, EventLoop slowed down? Time taken:" << timeTaken_ms << "ms (time out:" << timeout_ms << "ms, tolerance:" << timeout_tolerance_ms << "ms).";
            emit timeout(timeTaken_ms);
        }
        last = QTime::currentTime();
    });
    mWatchdogTimer.start(timeout_ms);
}

int SimpleWatchdog::getTimeout() const
{
    return timeout_ms;
}

void SimpleWatchdog::setTimeout(const int &value_ms)
{
    timeout_ms = value_ms;
    mWatchdogTimer.start(timeout_ms);
}

int SimpleWatchdog::getTimeoutTolerance() const
{
    return timeout_tolerance_ms;
}

void SimpleWatchdog::setTimeoutTolerance(const int &value)
{
    timeout_tolerance_ms = value;
}
