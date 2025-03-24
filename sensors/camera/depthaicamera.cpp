/*
 *     Copyright 2022 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Class to parse a JSON stream with object detections (incl. depth information) from DepthAI
 */
#include "depthaicamera.h"
#include <QDebug>

DepthAiCamera::DepthAiCamera()
{
    // Connect to camera stream
    attemptConnection();
    QObject::connect(&mJsonParser, &JsonStreamParserTcp::gotJsonArray, this, &DepthAiCamera::cameraInput);
    QObject::connect(&mJsonParser, &JsonStreamParserTcp::connectionError, this, &DepthAiCamera::handleConnectionError);

    connect(&mConnectionTimer, &QTimer::timeout, this, &DepthAiCamera::ConnectionIssue);
    mConnectionTimer.start(3000); // [ms]

    connect(&mReconnectTimer, &QTimer::timeout, this, &DepthAiCamera::attemptConnection);
}

void DepthAiCamera::attemptConnection()
{
    mJsonParser.connectToHost(QHostAddress::LocalHost, 8070);
}

void DepthAiCamera::cameraInput(const QString& tcpMsg)
{
    mConnectionTimer.start(3000); // Restart
    emit brakeSignal(tcpMsg);
}

void DepthAiCamera::ConnectionIssue()
{
    qDebug() << "DepthAICamera connection issue. No message received for 3 seconds.";
    emit brakeSignal("0");  // Emergency break
}

void DepthAiCamera::handleConnectionError(QTcpSocket::SocketError error)
{
    qDebug() << "Info: DepthAiCamera not connected, got" << error;
    mConnectionTimer.stop();
    mReconnectTimer.start(5000); // Try to reconnect every 5 seconds
}
