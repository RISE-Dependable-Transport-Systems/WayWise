#include "rtcmclient.h"
#include <QFile>

RtcmClient::RtcmClient(QObject *parent) : QObject(parent)
{
    connect(&mTcpSocket, &QTcpSocket::readyRead, [this]{
        QByteArray data =  mTcpSocket.readAll();
//        qDebug() << data;
        emit rtcmData(data);
    });

    connect(&mTcpSocket, &QTcpSocket::connected, [this]{
        // If a stream is selected, we connected to an NTRIP server and potentially need to authenticate.
        if (mCurrentNtripConnectionInfo.stream.size() > 0) {
            QString msg;
            msg += "GET /" + mCurrentNtripConnectionInfo.stream + " HTTP/1.1\r\n";
            msg += "User-Agent: NTRIP " + mCurrentHost + "\r\n";

            if (mCurrentNtripConnectionInfo.user.size() > 0 || mCurrentNtripConnectionInfo.password.size() > 0) {
                QString authStr = mCurrentNtripConnectionInfo.user + ":" + mCurrentNtripConnectionInfo.password;
                QByteArray auth;
                auth.append(authStr);
                msg += "Authorization: Basic " + auth.toBase64() + "\r\n";
            }

            msg += "Accept: */*\r\nConnection: close\r\n";
            msg += "\r\n";

            mTcpSocket.write(msg.toLocal8Bit());
        }
    });
}

void RtcmClient::connectTcp(QString host, qint16 port)
{
    mCurrentHost = host;
    mCurrentPort = port;
    mCurrentNtripConnectionInfo = {"", "", ""};
    mTcpSocket.connectToHost(mCurrentHost, mCurrentPort);
}

void RtcmClient::connectNtrip(QString host, qint16 port, NtripConnectionInfo ntripConnectionInfo)
{
    mCurrentHost = host;
    mCurrentPort = port;
    mCurrentNtripConnectionInfo = ntripConnectionInfo;
    mTcpSocket.connectToHost(mCurrentHost, mCurrentPort);
}

bool RtcmClient::connectWithInfoFromFile(QString filePath)
{
    QFile rtcmServerInfoFile(filePath);
    if (!rtcmServerInfoFile.open(QIODevice::ReadOnly)) {
        qDebug() << "Warning: RtcmClient was unable to open" << filePath;
        return false;
    } else {
        QString serverName = QString(rtcmServerInfoFile.readLine()).trimmed();
        qint16 port = rtcmServerInfoFile.readLine().toShort();
        NtripConnectionInfo ntripConnectionInfo = {QString(rtcmServerInfoFile.readLine()).trimmed(),  // username
                                                   QString(rtcmServerInfoFile.readLine()).trimmed(),  // password
                                                   QString(rtcmServerInfoFile.readLine()).trimmed()}; // stream
//        qDebug() << serverName << port << ntripConnectionInfo.user << ntripConnectionInfo.password << ntripConnectionInfo.stream;

        connectNtrip(serverName, port, ntripConnectionInfo);
    }

    return isConnected();
}

bool RtcmClient::isConnected()
{
    return mTcpSocket.isOpen() && mTcpSocket.isReadable();
}

void RtcmClient::disconnect()
{
    if (isConnected())
        mTcpSocket.disconnectFromHost();
}

QString RtcmClient::getCurrentHost() const
{
    return mCurrentHost;
}

qint16 RtcmClient::getCurrentPort() const
{
    return mCurrentPort;
}
