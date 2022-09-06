

#include <QDebug>
#include <QObject>

#include <QHostAddress>
#include "TCPClient.h"

TCPClient::TCPClient() :
    _socket(this)
{


        
;
}

TCPClient::~TCPClient()
{
    qDebug() << "killing TCPCLient";

}
void TCPClient::ConnectToHost(QString ip, QString port)
{
    _socket.connectToHost(QHostAddress::LocalHost, port.toInt());
    _socket.waitForConnected();
    _socket.waitForReadyRead();

    qDebug() << _socket.state();
    connect(&_socket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
   // connect(&_socket, SIGNAL(errorOccurred(QAbstractSocket::SocketError)),
   //     this, SLOT(TCPClient::errorOccurred(QAbstractSocket::SocketError)));
}
void TCPClient::errorOccurred(QAbstractSocket::SocketError error) {
    qDebug() << "error in connection: " << error;
}
void TCPClient::onReadyRead()
{
    QByteArray datas = _socket.readAll();
QVector<double> data;
   // qDebug() << datas;
    foreach (QByteArray a, datas.split(',')) {
        data.append(a.split(':').last().toDouble());
    }
   // _socket.write(QByteArray("ok !\n"));

   // qDebug()<< data[0] << data[1] << data[2];
    emit setLandingTarget(data[0],data[1],data[2]);
}
