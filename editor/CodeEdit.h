#ifndef SIMUL_EDITOR_CODEEDIT_H
#define SIMUL_EDITOR_CODEEDIT_H

#include <QPlainTextEdit>
#include <QString>
#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <QVector>
#include <QRegularExpression>

// python 문법 색
class PyHighlighter : public QSyntaxHighlighter {
public:
    explicit PyHighlighter(QTextDocument* doc);
protected:
    void highlightBlock(const QString& text) override;
private:
    struct Rule { QRegularExpression rx; QTextCharFormat fmt; };
    QVector<Rule> rules_;
};

// 파일 하나를 고치는 python 편집기 (자동 들여쓰기, Tab = 공백 4 칸)
class CodeEdit : public QPlainTextEdit {
    Q_OBJECT
public:
    explicit CodeEdit(const QString& path, QWidget* parent = nullptr);
    const QString& path() const { return path_; }
    bool Save(QString* error = nullptr);
protected:
    void keyPressEvent(QKeyEvent* e) override;
private:
    QString path_;
};

#endif
