#include "jsonstreamparsertcp.h"

JsonStreamParserTcp::JsonStreamParserTcp(QObject *parent) : QObject(parent)
{
    connect(&mTcpSocket, &QTcpSocket::readyRead, this, &JsonStreamParserTcp::parseJson);
    connect(&mTcpSocket, QOverload<QTcpSocket::SocketError>::of(&QTcpSocket::error), this, &JsonStreamParserTcp::tcpError);
}

void JsonStreamParserTcp::connectToHost(QHostAddress address, qint16 port)
{
    mTcpSocket.connectToHost(address, port, QTcpSocket::ReadOnly);
}

void JsonStreamParserTcp::parseJson()
{
    QJsonParseError err;
    for (const auto& elem : mTcpSocket.readAll().split('\n')) {
        if (elem.isEmpty())
            continue;

        QJsonDocument jsonDoc = QJsonDocument::fromJson(elem, &err);

        if (err.error != QJsonParseError::NoError) {
            qDebug() << "Skipped input due to error while parsing JSON:" << err.errorString();
        } else {
            if (jsonDoc.isArray())
                emit gotJsonArray(jsonDoc.array());

            if (jsonDoc.isObject())
                emit gotJsonObject(jsonDoc.object());
        }
    }
}

void JsonStreamParserTcp::tcpError(QTcpSocket::SocketError error)
{
    qDebug() << "TCP error (" << error << ") on " << mTcpSocket.peerAddress() << ", " << mTcpSocket.peerPort() << ".";
}
