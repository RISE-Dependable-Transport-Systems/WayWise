#ifndef RTCMCLIENT_H
#define RTCMCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>

struct NtripConnectionInfo {
    QString user;
    QString password;
    QString stream;
};

class RtcmClient : public QObject
{
    Q_OBJECT
public:
    explicit RtcmClient(QObject *parent = nullptr);
    void connectTcp(QString host, qint16 port);
    void connectNtrip(QString host, qint16 port, NtripConnectionInfo ntripInfo);
    bool isConnected();

signals:
    void rtcmData(const QByteArray &data);

private:
    QTcpSocket mTcpSocket;
    QString mCurrentHost;
    NtripConnectionInfo mCurrentNtripConnectionInfo;
};

#endif // RTCMCLIENT_H
