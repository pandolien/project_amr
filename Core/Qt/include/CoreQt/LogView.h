#ifndef CORECQT_LOGVIEW_H
#define CORECQT_LOGVIEW_H

#include <QComboBox>
#include <QDateTime>
#include <QPlainTextEdit>
#include <QString>
#include <QVector>
#include <QWidget>

// 프로그램이 한 일과 경고/오류를 시간순으로 쌓아 보여주는 로그 창.
// 수준(정보/경고/오류)마다 색이 다르고, 위쪽 콤보로 걸러 볼 수 있다. 줄 수가 많아지면 오래된 것부터 버린다.
class LogView : public QWidget {
    Q_OBJECT
public:
    enum Level { kInfo = 0, kWarning = 1, kError = 2 };

    explicit LogView(QWidget* parent = nullptr);

    void Append(Level level, const QString& text);
    void Info(const QString& text) { Append(kInfo, text); }
    void Warning(const QString& text) { Append(kWarning, text); }
    void Error(const QString& text) { Append(kError, text); }
    void Clear();
    int Count() const { return entries_.size(); }
    int CountOf(Level level) const;
    QString ToPlainText() const;   // 걸러지지 않은 전체 (시간 수준 내용)

private slots:
    void onFilterChanged();

private:
    struct Entry {
        QDateTime time;
        Level level;
        QString text;
    };
    static const int kMaxEntries = 5000;
    bool shown(const Entry& e) const;
    QString html(const Entry& e) const;
    void rebuild();

    QVector<Entry> entries_;
    QPlainTextEdit* edit_;
    QComboBox* filter_;
};

#endif  // CORECQT_LOGVIEW_H
