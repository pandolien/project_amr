#include "CodeEdit.h"

#include <QFile>
#include <QFont>
#include <QFontDatabase>
#include <QKeyEvent>
#include <QTextBlock>

PyHighlighter::PyHighlighter(QTextDocument* doc) : QSyntaxHighlighter(doc) {
    auto add = [this](const QString& pattern, const QColor& c, bool bold) {
        Rule r;
        r.rx = QRegularExpression(pattern);
        r.fmt.setForeground(c);
        if (bold) r.fmt.setFontWeight(QFont::Bold);
        rules_.push_back(r);
    };
    add(QStringLiteral("\\b(and|as|assert|break|class|continue|def|del|elif|else|except|finally|for|from|global|if|import|in|is|lambda|nonlocal|not|or|pass|raise|return|try|while|with|yield|None|True|False|self)\\b"), QColor("#0000c0"), true);
    add(QStringLiteral("\\b[0-9]+(\\.[0-9]+)?\\b"), QColor("#a05000"), false);
    add(QStringLiteral("\"[^\"\\n]*\"|'[^'\\n]*'"), QColor("#007000"), false);
    add(QStringLiteral("#[^\\n]*"), QColor("#808080"), false);
}

void PyHighlighter::highlightBlock(const QString& text) {
    for (int i = 0; i < rules_.size(); i++) {
        QRegularExpressionMatchIterator it = rules_[i].rx.globalMatch(text);
        while (it.hasNext()) {
            const QRegularExpressionMatch m = it.next();
            setFormat(m.capturedStart(), m.capturedLength(), rules_[i].fmt);
        }
    }
}

CodeEdit::CodeEdit(const QString& path, QWidget* parent) : QPlainTextEdit(parent), path_(path) {
    setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    setTabStopDistance(4 * fontMetrics().horizontalAdvance(QLatin1Char(' ')));
    setLineWrapMode(QPlainTextEdit::NoWrap);
    QFile f(path);
    if (f.open(QIODevice::ReadOnly)) setPlainText(QString::fromUtf8(f.readAll()));
    document()->setModified(false);
    new PyHighlighter(document());
}

bool CodeEdit::Save(QString* error) {
    QFile f(path_);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) { if (error) *error = QString::fromUtf8("저장할 수 없습니다: ") + path_; return false; }
    f.write(toPlainText().toUtf8());
    document()->setModified(false);
    return true;
}

void CodeEdit::keyPressEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter) {
        const QString line = textCursor().block().text();
        QString indent;
        for (int i = 0; i < line.size() && (line[i] == QLatin1Char(' ') || line[i] == QLatin1Char('\t')); i++) indent += line[i];
        if (line.trimmed().endsWith(QLatin1Char(':'))) indent += QStringLiteral("    ");
        QPlainTextEdit::keyPressEvent(e);
        insertPlainText(indent);
        return;
    }
    if (e->key() == Qt::Key_Tab) { insertPlainText(QStringLiteral("    ")); return; }
    QPlainTextEdit::keyPressEvent(e);
}
