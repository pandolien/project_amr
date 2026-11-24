#ifndef CORE_QT_LLMCLIENT_H
#define CORE_QT_LLMCLIENT_H

#include <QList>
#include <QObject>
#include <QString>

class QNetworkAccessManager;
class QNetworkReply;

// 로컬 LLM 서버(Ollama 의 /api/generate)에 한 마디씩 물어본다. 무료 모델을 PC 에서 직접 돌리는 용도라 키도 요금도 없다.
//   한 번에 하나만 보내고 나머지는 줄을 세운다 (작은 모델은 동시에 여러 개를 처리하지 못한다). 줄이 길어지면 오래된 것부터 버리고 실패로 알린다.
//   답을 못 받으면 (연결 실패, 모델 없음, 시간 초과, 이상한 응답) failed 를 낸다. 부르는 쪽이 정해 둔 대사로 대신하면 된다.
class LlmClient : public QObject {
    Q_OBJECT
public:
    explicit LlmClient(QObject* parent = nullptr);
    void SetServer(const QString& url, const QString& model);   // 예: "http://127.0.0.1:11434", "qwen2.5:1.5b"
    const QString& Url() const { return url_; }
    const QString& Model() const { return model_; }
    void SetTimeoutSeconds(int seconds) { timeoutSec_ = seconds; }   // 기본 35 초
    void Ask(int id, const QString& prompt);
    int Waiting() const { return static_cast<int>(queue_.size()) + (current_ ? 1 : 0); }

signals:
    void answered(int id, const QString& text);
    void failed(int id, const QString& why);

private:
    struct Item {
        int id;
        QByteArray body;
    };
    void next();
    void onFinished();

    QNetworkAccessManager* net_;
    QNetworkReply* current_;
    int currentId_;
    QList<Item> queue_;
    QString url_, model_;
    int timeoutSec_;
};

#endif  // CORE_QT_LLMCLIENT_H
