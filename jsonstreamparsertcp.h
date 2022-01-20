#ifndef JSONSTREAMPARSERTCP_H
#define JSONSTREAMPARSERTCP_H

#include <QObject>
#include <QHostAddress>
#include <QTcpSocket>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonArray>
#include <QJsonObject>

// connects to tcp port which is assumed to stream newline-delimited json objects and/or arrays, parses stream and makes data available using singals
class JsonStreamParserTcp : public QObject
{
    Q_OBJECT
public:
    explicit JsonStreamParserTcp(QObject *parent = nullptr);
    void connectToHost(QHostAddress address, qint16 port);

signals:
    void gotJsonArray(const QJsonArray& jsonArray);
    void gotJsonObject(const QJsonObject& jsonObject);
    void connectionError(QTcpSocket::SocketError);
private:
    void parseJson();
    void tcpError(QTcpSocket::SocketError error);

    QTcpSocket mTcpSocket;
};

#endif // JSONSTREAMPARSERTCP_H
