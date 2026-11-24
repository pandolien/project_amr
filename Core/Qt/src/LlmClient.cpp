#include "CoreQt/LlmClient.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPointer>
#include <QTimer>
#include <QUrl>

#include "CSimul/Chat.h"
#include "CoreQt/QtString.h"

LlmClient::LlmClient(QObject* parent)
    : QObject(parent), net_(new QNetworkAccessManager(this)), current_(nullptr), currentId_(0), url_("http://127.0.0.1:11434"), model_("qwen2.5:1.5b"), timeoutSec_(35) {}

void LlmClient::SetServer(const QString& url, const QString& model) {
    url_ = url;
    while (url_.endsWith('/')) url_.chop(1);
    model_ = model;
}

void LlmClient::Ask(int id, const QString& prompt) {
    Item it;
    it.id = id;
    const simul::String body = simul::sim::Chat::BuildOllamaBody(simulqt::FromQ(model_), simulqt::FromQ(prompt));
    it.body = QByteArray(body.c_str(), static_cast<int>(body.size()));
    queue_.push_back(it);
    while (queue_.size() > 4) {   // 줄이 길면 오래된 것부터 포기한다 (이미 늦은 대사다)
        const int dropped = queue_.front().id;
        queue_.pop_front();
        emit failed(dropped, QString::fromUtf8("LLM 이 바빠서 건너뜀"));
    }
    if (!current_) next();
}

void LlmClient::next() {
    if (current_ || queue_.isEmpty()) return;
    const Item it = queue_.front();
    queue_.pop_front();
    QNetworkRequest req(QUrl(url_ + "/api/generate"));
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    current_ = net_->post(req, it.body);
    currentId_ = it.id;
    connect(current_, &QNetworkReply::finished, this, &LlmClient::onFinished);
    QPointer<QNetworkReply> guard(current_);
    QTimer::singleShot(timeoutSec_ * 1000, this, [guard]() {
        if (guard) guard->abort();   // finished -> onFinished 에서 시간 초과로 처리된다
    });
}

void LlmClient::onFinished() {
    QNetworkReply* reply = current_;
    const int id = currentId_;
    current_ = nullptr;
    if (!reply) return;
    const QByteArray data = reply->readAll();
    const QNetworkReply::NetworkError err = reply->error();
    const QString errText = reply->errorString();
    reply->deleteLater();
    if (err == QNetworkReply::OperationCanceledError) {
        emit failed(id, QString::fromUtf8("시간 초과 (%1초)").arg(timeoutSec_));
    } else if (err != QNetworkReply::NoError) {
        // 모델이 없으면 Ollama 가 404 와 {"error":"model ... not found"} 를 준다
        QString detail = errText;
        const int at = data.indexOf("\"error\"");
        if (at >= 0) detail += " / " + QString::fromUtf8(data.mid(at, 160));
        emit failed(id, detail);
    } else {
        simul::String text;
        if (simul::sim::Chat::ExtractResponse(simul::String(data.constData(), static_cast<size_t>(data.size())), &text)) emit answered(id, simulqt::ToQ(text));
        else emit failed(id, QString::fromUtf8("응답을 읽을 수 없음"));
    }
    next();
}
