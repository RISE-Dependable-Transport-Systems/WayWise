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
    mJsonParser.connectToHost(QHostAddress::LocalHost, 8070);
    QObject::connect(&mJsonParser, &JsonStreamParserTcp::gotJsonArray, this, &DepthAiCamera::cameraInput);
    QObject::connect(&mJsonParser, &JsonStreamParserTcp::connectionError, [](QTcpSocket::SocketError error){
        qDebug() << "Info: DepthAiCamera not connected, got" << error;
    });

    connect(&mTimer, &QTimer::timeout, this, &DepthAiCamera::checkConnection);
    mTimer.start(3000); // 3 sec interval
}

void DepthAiCamera::cameraInput(const QString& tcpMsg)
{
    mTimer.start(3000); // Reset
    emit brakeSignal(tcpMsg);
}

void DepthAiCamera::checkConnection()
{
    qDebug() << "DepthAICamera connection issue. No message received for 3 seconds.";
    emit brakeSignal("0");  // Emergency break
}
