#include "CoreQt/PythonPanel.h"

#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QTextBlock>
#include <QTextStream>
#include <QVBoxLayout>

namespace {

QFont CodeFont() {
    QFont f = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    f.setPointSize(10);
    return f;
}

class NumberArea : public QWidget {
public:
    explicit NumberArea(PythonEditor* e) : QWidget(e), ed_(e) {}
    QSize sizeHint() const override { return QSize(ed_->LineNumberWidth(), 0); }

protected:
    void paintEvent(QPaintEvent* e) override { ed_->PaintLineNumbers(e); }

private:
    PythonEditor* ed_;
};

}  // namespace

// ---- 편집기 ----

PythonEditor::PythonEditor(QWidget* parent) : QPlainTextEdit(parent), numbers_(new NumberArea(this)) {
    setFont(CodeFont());
    setLineWrapMode(QPlainTextEdit::NoWrap);
    setTabStopDistance(fontMetrics().horizontalAdvance(QLatin1Char(' ')) * 4);
    connect(this, &QPlainTextEdit::blockCountChanged, this, &PythonEditor::updateMargin);
    connect(this, &QPlainTextEdit::updateRequest, this, &PythonEditor::updateNumbers);
    connect(this, &QPlainTextEdit::cursorPositionChanged, this, &PythonEditor::highlightLine);
    updateMargin();
    highlightLine();
}

int PythonEditor::LineNumberWidth() const {
    int digits = 1;
    for (int max = qMax(1, blockCount()); max >= 10; max /= 10) digits++;
    return 12 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * qMax(digits, 3);
}

void PythonEditor::updateMargin() { setViewportMargins(LineNumberWidth(), 0, 0, 0); }

void PythonEditor::updateNumbers(const QRect& rect, int dy) {
    if (dy) numbers_->scroll(0, dy);
    else numbers_->update(0, rect.y(), numbers_->width(), rect.height());
    if (rect.contains(viewport()->rect())) updateMargin();
}

void PythonEditor::resizeEvent(QResizeEvent* e) {
    QPlainTextEdit::resizeEvent(e);
    const QRect cr = contentsRect();
    numbers_->setGeometry(QRect(cr.left(), cr.top(), LineNumberWidth(), cr.height()));
}

void PythonEditor::highlightLine() {
    QTextEdit::ExtraSelection sel;
    sel.format.setBackground(palette().alternateBase().color().lighter(105));
    sel.format.setProperty(QTextFormat::FullWidthSelection, true);
    sel.cursor = textCursor();
    sel.cursor.clearSelection();
    setExtraSelections(QList<QTextEdit::ExtraSelection>() << sel);
}

void PythonEditor::PaintLineNumbers(QPaintEvent* e) {
    QPainter p(numbers_);
    p.fillRect(e->rect(), palette().window());
    QTextBlock block = firstVisibleBlock();
    int n = block.blockNumber();
    int top = qRound(blockBoundingGeometry(block).translated(contentOffset()).top());
    int bottom = top + qRound(blockBoundingRect(block).height());
    p.setPen(palette().color(QPalette::Disabled, QPalette::Text));
    while (block.isValid() && top <= e->rect().bottom()) {
        if (block.isVisible() && bottom >= e->rect().top()) p.drawText(0, top, numbers_->width() - 6, fontMetrics().height(), Qt::AlignRight, QString::number(n + 1));
        block = block.next();
        top = bottom;
        bottom = top + qRound(blockBoundingRect(block).height());
        n++;
    }
}

void PythonEditor::keyPressEvent(QKeyEvent* e) {
    QTextCursor c = textCursor();
    if (e->key() == Qt::Key_Tab && !(e->modifiers() & Qt::ControlModifier)) {
        if (c.hasSelection()) {   // 선택한 줄 전체를 들여쓴다
            const int a = qMin(c.anchor(), c.position()), b = qMax(c.anchor(), c.position());
            QTextCursor s(document());
            s.setPosition(a);
            const int first = s.blockNumber();
            s.setPosition(b);
            const int last = s.blockNumber();
            c.beginEditBlock();
            for (int i = first; i <= last; i++) {
                QTextCursor l(document()->findBlockByNumber(i));
                l.insertText(QStringLiteral("    "));
            }
            c.endEditBlock();
        } else {
            c.insertText(QString(4 - c.positionInBlock() % 4, QLatin1Char(' ')));
        }
        return;
    }
    if (e->key() == Qt::Key_Backtab) {   // Shift+Tab: 내어쓰기
        const int a = qMin(c.anchor(), c.position()), b = qMax(c.anchor(), c.position());
        QTextCursor s(document());
        s.setPosition(a);
        const int first = s.blockNumber();
        s.setPosition(b);
        const int last = s.blockNumber();
        c.beginEditBlock();
        for (int i = first; i <= last; i++) {
            QTextBlock blk = document()->findBlockByNumber(i);
            int k = 0;
            while (k < 4 && k < blk.text().size() && blk.text()[k] == QLatin1Char(' ')) k++;
            QTextCursor l(blk);
            l.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, k);
            l.removeSelectedText();
        }
        c.endEditBlock();
        return;
    }
    if (e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter) {
        const QString line = c.block().text().left(c.positionInBlock());
        int indent = 0;
        while (indent < line.size() && line[indent] == QLatin1Char(' ')) indent++;
        QString t = line.trimmed();
        const int hash = t.indexOf(QLatin1Char('#'));
        if (hash >= 0) t = t.left(hash).trimmed();
        if (t.endsWith(QLatin1Char(':'))) indent += 4;
        QPlainTextEdit::keyPressEvent(e);
        textCursor().insertText(QString(indent, QLatin1Char(' ')));
        return;
    }
    if (e->key() == Qt::Key_Backspace && !c.hasSelection()) {   // 공백만 있는 들여쓰기에서는 한 단계씩 지운다
        const QString before = c.block().text().left(c.positionInBlock());
        if (!before.isEmpty() && before.trimmed().isEmpty()) {
            const int n = before.size() % 4 == 0 ? 4 : before.size() % 4;
            c.movePosition(QTextCursor::Left, QTextCursor::KeepAnchor, n);
            c.removeSelectedText();
            return;
        }
    }
    QPlainTextEdit::keyPressEvent(e);
}

// ---- 문법 색 ----

PythonHighlighter::PythonHighlighter(QTextDocument* doc) : QSyntaxHighlighter(doc) {
    auto fmt = [](const char* color, bool bold, bool italic) {
        QTextCharFormat f;
        f.setForeground(QColor(color));
        if (bold) f.setFontWeight(QFont::Bold);
        f.setFontItalic(italic);
        return f;
    };
    const QString kw = QStringLiteral("\\b(and|as|assert|break|class|continue|def|del|elif|else|except|finally|for|from|global|if|import|in|is|lambda|nonlocal|not|or|pass|raise|return|try|while|with|yield|None|True|False)\\b");
    rules_ << Rule{QRegularExpression(kw), fmt("#0000c0", true, false)};
    rules_ << Rule{QRegularExpression(QStringLiteral("\\b(self|print|len|range|sorted|list|dict|set|tuple|str|int|float|bool|abs|min|max|sum|round|zip|enumerate|isinstance)\\b")), fmt("#7a3e9d", false, false)};
    rules_ << Rule{QRegularExpression(QStringLiteral("\\b(Behavior|register|simul|Main)\\b")), fmt("#007070", true, false)};
    rules_ << Rule{QRegularExpression(QStringLiteral("\\b(on_start|on_tick|on_event|on_stop)\\b")), fmt("#a05000", true, false)};
    rules_ << Rule{QRegularExpression(QStringLiteral("\\b[0-9]+(\\.[0-9]+)?([eE][-+]?[0-9]+)?\\b")), fmt("#c04000", false, false)};
    rules_ << Rule{QRegularExpression(QStringLiteral("^\\s*@\\w+")), fmt("#a05000", false, false)};
    string_ = fmt("#008000", false, false);
    comment_ = fmt("#808080", false, true);
}

void PythonHighlighter::highlightBlock(const QString& text) {
    for (int i = 0; i < rules_.size(); i++) {
        QRegularExpressionMatchIterator it = rules_[i].re.globalMatch(text);
        while (it.hasNext()) {
            const QRegularExpressionMatch m = it.next();
            setFormat(m.capturedStart(), m.capturedLength(), rules_[i].fmt);
        }
    }
    // 문자열과 주석은 위의 규칙을 덮어쓴다. 앞에서부터 훑으면서 따옴표 안과 # 뒤를 칠한다 (여러 줄 """ 는 블록 상태로 이어간다)
    int i = 0;
    const int n = text.size();
    int state = previousBlockState();   // 0: 없음, 1: """ 안, 2: ''' 안
    auto tail = [&](const QString& q) {   // 문자열 끝을 찾아 칠한다. 끝이 없으면 -1
        const int end = text.indexOf(q, i);
        const int stop = end < 0 ? n : end + 3;
        setFormat(i, stop - i, string_);
        i = stop;
        return end >= 0;
    };
    if (state == 1 || state == 2) {
        if (!tail(state == 1 ? QStringLiteral("\"\"\"") : QStringLiteral("'''"))) {
            setCurrentBlockState(state);
            return;
        }
    }
    setCurrentBlockState(0);
    while (i < n) {
        const QChar c = text[i];
        if (c == QLatin1Char('#')) {
            setFormat(i, n - i, comment_);
            break;
        }
        if (c == QLatin1Char('"') || c == QLatin1Char('\'')) {
            const QString triple(3, c);
            if (text.mid(i, 3) == triple) {
                const int start = i;
                i += 3;
                const int s = i;
                Q_UNUSED(s);
                const int end = text.indexOf(triple, i);
                if (end < 0) {
                    setFormat(start, n - start, string_);
                    setCurrentBlockState(c == QLatin1Char('"') ? 1 : 2);
                    return;
                }
                setFormat(start, end + 3 - start, string_);
                i = end + 3;
                continue;
            }
            int j = i + 1;
            while (j < n && text[j] != c) j += (text[j] == QLatin1Char('\\')) ? 2 : 1;
            const int stop = qMin(n, j + 1);
            setFormat(i, stop - i, string_);
            i = stop;
            continue;
        }
        i++;
    }
}

// ---- 패널 ----

QString PythonPanel::Template() {
    return QString::fromUtf8(
        "# -*- coding: utf-8 -*-\n"
        "from Main import Behavior, register\n"
        "\n"
        "\n"
        "class MyBehavior(Behavior):\n"
        "    objects = []            # 움직일 수 있는 오브젝트 이름 (예: [\"arm\"]). \"*\" 는 전부. 등록할 때 아래 입력칸으로 덮어쓸 수도 있다\n"
        "    period = 0.0            # on_tick 을 부르는 최소 간격 (s). 0 이면 매 틱\n"
        "\n"
        "    def on_start(self):\n"
        "        # self.objects[\"이름\"] : 움직일 수 있는 오브젝트,  self.world[\"이름\"] : 모든 오브젝트 (읽기 전용)\n"
        "        print(\"시작\", sorted(self.world))\n"
        "\n"
        "    def on_tick(self, t, dt):\n"
        "        pass\n"
        "\n"
        "    def on_event(self, ev):\n"
        "        # ev.type: arrived, joint_reached, added, removed, command_failed\n"
        "        pass\n"
        "\n"
        "    def on_stop(self):\n"
        "        pass\n"
        "\n"
        "\n"
        "register(MyBehavior)\n");
}

PythonPanel::PythonPanel(QWidget* parent) : QWidget(parent) {
    QVBoxLayout* v = new QVBoxLayout(this);
    v->setContentsMargins(4, 4, 4, 4);
    QHBoxLayout* bar = new QHBoxLayout;
    QPushButton* nw = new QPushButton(QString::fromUtf8("새 파일"));
    QPushButton* op = new QPushButton(QString::fromUtf8("열기"));
    QPushButton* sv = new QPushButton(QString::fromUtf8("저장"));
    QPushButton* sa = new QPushButton(QString::fromUtf8("다른 이름"));
    bar->addWidget(nw);
    bar->addWidget(op);
    bar->addWidget(sv);
    bar->addWidget(sa);
    bar->addStretch();
    v->addLayout(bar);
    title_ = new QLabel;
    v->addWidget(title_);
    editor_ = new PythonEditor;
    highlighter_ = new PythonHighlighter(editor_->document());
    v->addWidget(editor_, 1);
    QHBoxLayout* reg = new QHBoxLayout;
    reg->addWidget(new QLabel(QString::fromUtf8("움직일 오브젝트")));
    objects_ = new QLineEdit;
    objects_->setPlaceholderText(QString::fromUtf8("arm, box  (비우면 파일의 objects, * 는 전부)"));
    reg->addWidget(objects_, 1);
    QPushButton* rb = new QPushButton(QString::fromUtf8("저장 후 등록"));
    rb->setToolTip(QString::fromUtf8("저장하고 Main.py 에 등록한다. 이미 등록된 같은 파일은 새로 읽는다 (시뮬레이션이 돌고 있어도 바로 바뀐다). Ctrl+Enter"));
    reg->addWidget(rb);
    v->addLayout(reg);
    line_ = new QLineEdit;
    line_->setFont(CodeFont());
    line_->setPlaceholderText(QString::fromUtf8(">>> 한 줄 실행 (예: simul.find(\"arm\").joints)  Enter"));
    v->addWidget(line_);

    connect(nw, &QPushButton::clicked, this, [this]() { if (maybeDiscard()) NewFromTemplate(); });
    connect(op, &QPushButton::clicked, this, &PythonPanel::onOpen);
    connect(sv, &QPushButton::clicked, this, [this]() { Save(); });
    connect(sa, &QPushButton::clicked, this, &PythonPanel::onSaveAs);
    connect(rb, &QPushButton::clicked, this, &PythonPanel::onRegister);
    connect(line_, &QLineEdit::returnPressed, this, &PythonPanel::onRunLine);
    connect(editor_->document(), &QTextDocument::modificationChanged, this, &PythonPanel::updateTitle);
    NewFromTemplate();
}

void PythonPanel::updateTitle() {
    title_->setText((path_.isEmpty() ? QString::fromUtf8("(저장 안 한 새 파일)") : QFileInfo(path_).fileName()) + (Modified() ? QStringLiteral(" *") : QString()));
    title_->setToolTip(path_);
}

void PythonPanel::NewFromTemplate() {
    path_.clear();
    editor_->setPlainText(Template());
    editor_->document()->setModified(false);
    updateTitle();
}

bool PythonPanel::maybeDiscard() {
    if (!Modified()) return true;
    const QMessageBox::StandardButton b = QMessageBox::question(this, QString::fromUtf8("저장"), QString::fromUtf8("바뀐 내용을 저장할까요?"), QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    if (b == QMessageBox::Cancel) return false;
    if (b == QMessageBox::Save) return Save();
    return true;
}

bool PythonPanel::Open(const QString& path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, QString::fromUtf8("열 수 없음"), path);
        return false;
    }
    editor_->setPlainText(QString::fromUtf8(f.readAll()));
    editor_->document()->setModified(false);
    path_ = path;
    updateTitle();
    emit fileChanged(path_);
    return true;
}

bool PythonPanel::saveTo(const QString& path) {
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        QMessageBox::warning(this, QString::fromUtf8("저장 실패"), path);
        return false;
    }
    f.write(editor_->toPlainText().toUtf8());
    f.close();
    path_ = path;
    editor_->document()->setModified(false);
    updateTitle();
    emit fileChanged(path_);
    return true;
}

bool PythonPanel::Save() {
    if (path_.isEmpty()) {
        const QString p = QFileDialog::getSaveFileName(this, QString::fromUtf8("python 파일로 저장"), defaultDir_, QString::fromUtf8("Python (*.py)"));
        if (p.isEmpty()) return false;
        return saveTo(p.endsWith(QStringLiteral(".py"), Qt::CaseInsensitive) ? p : p + QStringLiteral(".py"));
    }
    return saveTo(path_);
}

void PythonPanel::onSaveAs() {
    const QString p = QFileDialog::getSaveFileName(this, QString::fromUtf8("다른 이름으로 저장"), path_.isEmpty() ? defaultDir_ : path_, QString::fromUtf8("Python (*.py)"));
    if (p.isEmpty()) return;
    saveTo(p.endsWith(QStringLiteral(".py"), Qt::CaseInsensitive) ? p : p + QStringLiteral(".py"));
}

void PythonPanel::onOpen() {
    if (!maybeDiscard()) return;
    const QString p = QFileDialog::getOpenFileName(this, QString::fromUtf8("python 파일 열기"), path_.isEmpty() ? defaultDir_ : QFileInfo(path_).absolutePath(), QString::fromUtf8("Python (*.py)"));
    if (!p.isEmpty()) Open(p);
}

void PythonPanel::onRegister() {
    if (!Save()) return;
    emit registerRequested(path_, objects_->text().trimmed());
}

void PythonPanel::onRunLine() {
    const QString code = line_->text();
    if (code.trimmed().isEmpty()) return;
    line_->clear();
    emit snippetRequested(code);
}
