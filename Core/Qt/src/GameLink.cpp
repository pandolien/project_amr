#include "CoreQt/GameLink.h"

#include <QHostAddress>
#include <QTcpServer>
#include <QTcpSocket>

GameLink::GameLink(QObject* parent) : QObject(parent), server_(new QTcpServer(this)) {
    QObject::connect(server_, &QTcpServer::newConnection, this, [this]() { onNewConnection(); });
}

bool GameLink::Listen(quint16 port, QString* error) {
    if (server_->listen(QHostAddress::LocalHost, port)) return true;
    if (error) *error = server_->errorString();
    return false;
}

quint16 GameLink::Port() const { return server_->serverPort(); }

void GameLink::onNewConnection() {
    while (QTcpSocket* s = server_->nextPendingConnection()) {
        clients_.append(s);
        buffers_.append(QByteArray());
        QObject::connect(s, &QTcpSocket::readyRead, this, [this, s]() { onReadyRead(s); });
        QObject::connect(s, &QTcpSocket::disconnected, this, [this, s]() { onDisconnected(s); });
        emit connectionsChanged(static_cast<int>(clients_.size()));
    }
}

void GameLink::onReadyRead(QTcpSocket* s) {
    const int i = clients_.indexOf(s);
    if (i < 0) return;
    buffers_[i].append(s->readAll());
    if (buffers_[i].size() > (1 << 20)) { buffers_[i].clear(); s->disconnectFromHost(); return; }   // 줄바꿈 없이 계속 오는 입력은 끊는다
    for (;;) {
        const int nl = buffers_[i].indexOf('\n');
        if (nl < 0) break;
        const QString line = QString::fromUtf8(buffers_[i].left(nl)).trimmed();
        buffers_[i].remove(0, nl + 1);
        if (!line.isEmpty()) emit lineReceived(line);
        if (clients_.indexOf(s) < 0) return;   // 처리 중에 끊겼다
    }
}

void GameLink::onDisconnected(QTcpSocket* s) {
    const int i = clients_.indexOf(s);
    if (i < 0) return;
    clients_.removeAt(i);
    buffers_.removeAt(i);
    s->deleteLater();
    emit connectionsChanged(static_cast<int>(clients_.size()));
}

void GameLink::Broadcast(const QByteArray& line) {
    QByteArray out = line;
    out.append('\n');
    for (int i = 0; i < clients_.size(); i++) clients_[i]->write(out);
}
