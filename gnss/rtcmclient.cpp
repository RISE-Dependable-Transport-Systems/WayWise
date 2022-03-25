#include "rtcmclient.h"

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
    mCurrentNtripConnectionInfo = {"", "", ""};
    mTcpSocket.connectToHost(host, port);
}

void RtcmClient::connectNtrip(QString host, qint16 port, NtripConnectionInfo ntripConnectionInfo)
{
    mCurrentNtripConnectionInfo = ntripConnectionInfo;
    mTcpSocket.connectToHost(host, port);
}

bool RtcmClient::isConnected()
{
    return mTcpSocket.isOpen() && mTcpSocket.isReadable();
}
