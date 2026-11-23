#ifndef SIMUL_EDITOR_PYCONSOLE_H
#define SIMUL_EDITOR_PYCONSOLE_H

#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProcess>
#include <QStringList>
#include <QWidget>

// Python CMD: 프로젝트 폴더에서 `python -i` 로 sim 과 오브젝트를 만들어 두고 한 줄씩 보낸다.
// (python 은 오브젝트를 움직일 때만 쓰고, 이 창은 그 python 을 손으로 부리는 곳이다)
class PyConsole : public QWidget {
    Q_OBJECT
public:
    explicit PyConsole(QWidget* parent = nullptr);
    void Start(const QString& projectDir);   // 이미 돌고 있으면 다시 시작
    void Stop();
    bool IsRunning() const { return proc_ && proc_->state() != QProcess::NotRunning; }
signals:
    void feed(const QString& json);          // python 이 보낸 오브젝트 상태 한 줄 (@@SIMUL ...)
private slots:
    void submit();
    void readOutput();
private:
    void append(const QString& text);
    QPlainTextEdit* out_;
    QLineEdit* in_;
    QProcess* proc_;
    QStringList history_;
    int hpos_;
    QByteArray pending_;
protected:
    bool eventFilter(QObject* obj, QEvent* e) override;
};

#endif
