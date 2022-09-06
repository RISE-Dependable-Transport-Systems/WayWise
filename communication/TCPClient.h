#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <QTcpSocket>

class TCPClient : public QObject
{
    Q_OBJECT

public:
    TCPClient();
    ~TCPClient();
    void ConnectToHost(QString ip, QString port);
signals:
    void setLandingTarget(double,double,double);
public slots:
    void onReadyRead();
    void errorOccurred(QAbstractSocket::SocketError error);

private:
    QTcpSocket _socket;
};

#endif // TCPCLIENT_H
