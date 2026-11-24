#ifndef CORE_QT_GAMELINK_H
#define CORE_QT_GAMELINK_H

#include <QByteArray>
#include <QList>
#include <QObject>
#include <QString>

class QTcpServer;
class QTcpSocket;

// 웹 게이트웨이(web/server.js)가 붙는 줄 단위 TCP 서버. 이 컴퓨터(127.0.0.1)에서 오는 연결만 받는다.
//   받은 줄은 lineReceived 로, Broadcast 한 줄은 연결된 모두에게 보낸다. 연결이 끊겨도 서버는 계속 기다린다.
class GameLink : public QObject {
    Q_OBJECT
public:
    explicit GameLink(QObject* parent = nullptr);
    bool Listen(quint16 port, QString* error = nullptr);
    quint16 Port() const;
    int Connections() const { return static_cast<int>(clients_.size()); }
    void Broadcast(const QByteArray& line);   // 끝에 줄바꿈을 붙여 보낸다

signals:
    void lineReceived(const QString& line);
    void connectionsChanged(int count);

private:
    void onNewConnection();
    void onReadyRead(QTcpSocket* s);
    void onDisconnected(QTcpSocket* s);

    QTcpServer* server_;
    QList<QTcpSocket*> clients_;
    QList<QByteArray> buffers_;
};

#endif  // CORE_QT_GAMELINK_H
