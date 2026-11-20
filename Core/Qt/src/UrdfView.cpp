#include "CoreQt/UrdfView.h"

#include <QEvent>
#include <QFontDatabase>
#include <QMouseEvent>
#include <QRegularExpression>
#include <QScrollBar>
#include <QSyntaxHighlighter>
#include <QTextBlock>
#include <QTextDocument>
#include <QVBoxLayout>

#include "CURDF/Urdf.h"
#include "CoreQt/ModelViewer.h"
#include "CoreQt/QtString.h"

namespace {

// XML 색칠: 주석, 태그 이름, 속성 이름, 문자열 값
class XmlHighlighter : public QSyntaxHighlighter {
public:
    explicit XmlHighlighter(QTextDocument* doc, bool dark) : QSyntaxHighlighter(doc) {
        tag_.setForeground(dark ? QColor(86, 156, 214) : QColor(0, 70, 160));
        attr_.setForeground(dark ? QColor(156, 220, 254) : QColor(120, 60, 0));
        str_.setForeground(dark ? QColor(206, 145, 120) : QColor(160, 30, 30));
        comment_.setForeground(dark ? QColor(106, 153, 85) : QColor(0, 120, 0));
    }

protected:
    void highlightBlock(const QString& text) override {
        static const QRegularExpression tag("</?[A-Za-z_?][\\w:.\\-?]*|/?>|\\?>");
        static const QRegularExpression attr("\\b[A-Za-z_][\\w:.\\-]*(?=\\s*=)");
        static const QRegularExpression str("\"[^\"]*\"");
        static const QRegularExpression comment("<!--.*?(-->|$)");
        for (QRegularExpressionMatchIterator it = tag.globalMatch(text); it.hasNext();) {
            const QRegularExpressionMatch m = it.next();
            setFormat(m.capturedStart(), m.capturedLength(), tag_);
        }
        for (QRegularExpressionMatchIterator it = attr.globalMatch(text); it.hasNext();) {
            const QRegularExpressionMatch m = it.next();
            setFormat(m.capturedStart(), m.capturedLength(), attr_);
        }
        for (QRegularExpressionMatchIterator it = str.globalMatch(text); it.hasNext();) {
            const QRegularExpressionMatch m = it.next();
            setFormat(m.capturedStart(), m.capturedLength(), str_);
        }
        for (QRegularExpressionMatchIterator it = comment.globalMatch(text); it.hasNext();) {
            const QRegularExpressionMatch m = it.next();
            setFormat(m.capturedStart(), m.capturedLength(), comment_);
        }
    }

private:
    QTextCharFormat tag_, attr_, str_, comment_;
};

QString Escape(const QString& s) {
    QString o = s;
    o.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("\"", "&quot;");
    return o;
}

QString Unescape(const QString& s) {
    QString o = s;
    o.replace("&quot;", "\"").replace("&gt;", ">").replace("&lt;", "<").replace("&amp;", "&");
    return o;
}

// 한 줄에서 attr="..." 의 값 (없으면 빈 문자열)
QString AttrOf(const QString& line, const QString& attr) {
    const QString key = attr + "=\"";
    const int a = line.indexOf(key);
    if (a < 0) return QString();
    const int b = line.indexOf('"', a + key.size());
    return b < 0 ? QString() : Unescape(line.mid(a + key.size(), b - a - key.size()));
}

}  // namespace

UrdfView::UrdfView(ModelViewer* viewer, QWidget* parent) : QWidget(parent), viewer_(viewer), shownLink_(-1), settingSelection_(false) {
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->setContentsMargins(4, 4, 4, 4);
    lay->setSpacing(4);
    header_ = new QLabel(this);
    lay->addWidget(header_);
    edit_ = new QPlainTextEdit(this);
    edit_->setReadOnly(true);
    edit_->setLineWrapMode(QPlainTextEdit::NoWrap);
    edit_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    edit_->setUndoRedoEnabled(false);
    edit_->viewport()->installEventFilter(this);
    lay->addWidget(edit_, 1);
    new XmlHighlighter(edit_->document(), palette().base().color().lightness() < 128);

    timer_.setSingleShot(true);
    timer_.setInterval(150);   // 드래그 중에는 신호가 계속 오므로 모아서 한 번만 만든다
    connect(&timer_, &QTimer::timeout, this, &UrdfView::RefreshNow);
    connect(viewer_, &ModelViewer::modelChanged, this, &UrdfView::scheduleRefresh);
    connect(viewer_, &ModelViewer::robotChanged, this, &UrdfView::scheduleRefresh);
    connect(viewer_, &ModelViewer::structureChanged, this, &UrdfView::scheduleRefresh);
    connect(viewer_, &ModelViewer::selectionChanged, this, [this](int link) { ShowLink(link); });
    connect(viewer_, &ModelViewer::selectionPicked, this, [this](int link, bool) { ShowLink(link); });
    RefreshNow();
}

void UrdfView::scheduleRefresh() { timer_.start(); }

void UrdfView::RefreshNow() {
    timer_.stop();
    const QString text = viewer_->UrdfText();
    if (!viewer_->HasRobot()) {
        header_->setText(QString::fromUtf8("모델이 없습니다 (파일 > 열기, 또는 도형 추가)"));
    } else {
        header_->setText(QString::fromUtf8("URDF (읽기 전용) · 모델 %1개 · 링크 %2 · 관절 %3 · %4줄")
                             .arg(viewer_->ModelCount()).arg(viewer_->Robot().links.size()).arg(viewer_->Robot().joints.size()).arg(text.count('\n')));
    }
    if (text == edit_->toPlainText()) return;   // 같으면 건드리지 않는다 (깜박임, 스크롤 유지)
    const int v = edit_->verticalScrollBar()->value();
    const int h = edit_->horizontalScrollBar()->value();
    edit_->setPlainText(text);
    edit_->verticalScrollBar()->setValue(v);
    edit_->horizontalScrollBar()->setValue(h);
    const int keep = shownLink_;
    shownLink_ = -1;
    ShowLink(keep);
}

bool UrdfView::LinkRange(const QString& name, int* firstLine, int* lastLine) const {
    const QString open = "<link name=\"" + Escape(name) + "\"";
    QTextBlock b = edit_->document()->begin();
    for (; b.isValid(); b = b.next()) {
        const QString t = b.text().trimmed();
        if (!t.startsWith(open)) continue;
        // 이름이 접두사만 같은 다른 링크를 거른다 (바로 뒤가 " 이므로 open 이 따옴표까지 포함해 정확히 일치)
        *firstLine = b.blockNumber();
        if (t.endsWith("/>")) {
            *lastLine = b.blockNumber();
            return true;
        }
        for (QTextBlock e = b.next(); e.isValid(); e = e.next())
            if (e.text().trimmed() == "</link>") {
                *lastLine = e.blockNumber();
                return true;
            }
        *lastLine = b.blockNumber();
        return true;
    }
    return false;
}

void UrdfView::highlightRange(int first, int last) {
    QList<QTextEdit::ExtraSelection> sel;
    if (first >= 0) {
        QTextEdit::ExtraSelection s;
        QColor c = palette().highlight().color();
        c.setAlpha(70);
        s.format.setBackground(c);
        s.format.setProperty(QTextFormat::FullWidthSelection, true);
        QTextBlock a = edit_->document()->findBlockByNumber(first);
        QTextBlock b = edit_->document()->findBlockByNumber(last);
        if (a.isValid() && b.isValid()) {
            s.cursor = QTextCursor(a);
            s.cursor.setPosition(b.position() + b.length() - 1, QTextCursor::KeepAnchor);
            sel.push_back(s);
        }
    }
    edit_->setExtraSelections(sel);
}

void UrdfView::ShowLink(int link) {
    if (settingSelection_) return;
    shownLink_ = link;
    const simul::urdf::Robot& r = viewer_->Robot();
    int a = 0, b = 0;
    if (link < 0 || static_cast<size_t>(link) >= r.links.size() || !LinkRange(simulqt::ToQ(r.links[static_cast<size_t>(link)].name), &a, &b)) {
        highlightRange(-1, -1);
        return;
    }
    highlightRange(a, b);
    QTextCursor c(edit_->document()->findBlockByNumber(a));
    const QTextCursor keep = edit_->textCursor();
    edit_->setTextCursor(c);
    edit_->centerCursor();   // 블록 첫 줄이 화면 가운데에 오게 한다
    c.clearSelection();
    edit_->setTextCursor(c);
    (void)keep;
}

int UrdfView::LinkAtLine(int line) const {
    QTextDocument* d = edit_->document();
    for (QTextBlock b = d->findBlockByNumber(line); b.isValid(); b = b.previous()) {
        const QString t = b.text().trimmed();
        if (b.blockNumber() != line && (t == "</link>" || t == "</joint>")) return -1;   // 위에서 블록이 이미 끝났다
        QString name;
        if (t.startsWith("<link ")) {
            name = AttrOf(t, "name");
        } else if (t.startsWith("<joint ")) {
            for (QTextBlock e = b; e.isValid(); e = e.next()) {   // 아래로 내려가며 <child link="..."/>
                const QString u = e.text().trimmed();
                if (u.startsWith("<child ")) {
                    name = AttrOf(u, "link");
                    break;
                }
                if (u == "</joint>") break;
            }
        } else {
            if (t == "</link>" && b.blockNumber() == line) return -1;
            continue;
        }
        if (name.isEmpty()) return -1;
        return viewer_->Robot().FindLink(simulqt::FromQ(name));
    }
    return -1;
}

bool UrdfView::eventFilter(QObject* obj, QEvent* e) {
    if (obj == edit_->viewport() && e->type() == QEvent::MouseButtonRelease) {
        QMouseEvent* m = static_cast<QMouseEvent*>(e);
        if (m->button() == Qt::LeftButton && !edit_->textCursor().hasSelection()) {   // 드래그로 글을 고르는 중이 아닐 때만
            const int line = edit_->cursorForPosition(m->pos()).blockNumber();
            const int link = LinkAtLine(line);
            if (link >= 0 && link != viewer_->SelectedLink()) {
                settingSelection_ = true;   // 우리가 건 선택이 다시 우리를 스크롤하지 않게 한다
                viewer_->SetSelectedLink(link);
                settingSelection_ = false;
                shownLink_ = link;
                int a = 0, b = 0;
                if (LinkRange(simulqt::ToQ(viewer_->Robot().links[static_cast<size_t>(link)].name), &a, &b)) highlightRange(a, b);
            }
        }
    }
    return QWidget::eventFilter(obj, e);
}
