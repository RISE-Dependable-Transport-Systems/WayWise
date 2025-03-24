/*
 *     Copyright 2021 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */
#include "jsonstreamparsertcp.h"

JsonStreamParserTcp::JsonStreamParserTcp(QObject *parent) : QObject(parent)
{
    connect(&mTcpSocket, &QTcpSocket::readyRead, this, &JsonStreamParserTcp::parseJson);
    connect(&mTcpSocket, QOverload<QTcpSocket::SocketError>::of(&QTcpSocket::errorOccurred), this, &JsonStreamParserTcp::tcpError);
}

void JsonStreamParserTcp::connectToHost(QHostAddress address, qint16 port)
{
    mTcpSocket.connectToHost(address, port, QTcpSocket::ReadOnly);
}

void JsonStreamParserTcp::parseJson()
{
    for (const auto& elem : mTcpSocket.readAll().split('\n')) {
        if (elem.isEmpty())
            continue;

        QString msg = QString::fromUtf8(elem).trimmed();    // remove trailing whitespace including \r and \n

        emit gotJsonArray(msg);
    }
}

void JsonStreamParserTcp::tcpError(QTcpSocket::SocketError error)
{
    emit connectionError(error);
}
