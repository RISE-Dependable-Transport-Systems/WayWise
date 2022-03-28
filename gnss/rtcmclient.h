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
    bool connectWithInfoFromFile(QString filePath);
    bool isConnected();
    void disconnect();

    QString getCurrentHost() const;

    qint16 getCurrentPort() const;

signals:
    void rtcmData(const QByteArray &data);

private:
    QTcpSocket mTcpSocket;
    QString mCurrentHost;
    qint16 mCurrentPort;
    NtripConnectionInfo mCurrentNtripConnectionInfo;
};

#endif // RTCMCLIENT_H
