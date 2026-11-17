#include "CoreQt/LogView.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QScrollBar>
#include <QVBoxLayout>

LogView::LogView(QWidget* parent) : QWidget(parent), edit_(nullptr), filter_(nullptr) {
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->setContentsMargins(4, 4, 4, 4);
    QHBoxLayout* bar = new QHBoxLayout;
    filter_ = new QComboBox(this);
    filter_->addItem(QString::fromUtf8("모두"), kInfo);
    filter_->addItem(QString::fromUtf8("경고 이상"), kWarning);
    filter_->addItem(QString::fromUtf8("오류만"), kError);
    QPushButton* clear = new QPushButton(QString::fromUtf8("지우기"), this);
    bar->addWidget(filter_);
    bar->addStretch(1);
    bar->addWidget(clear);
    lay->addLayout(bar);
    edit_ = new QPlainTextEdit(this);
    edit_->setReadOnly(true);
    edit_->setMaximumBlockCount(kMaxEntries);
    edit_->setLineWrapMode(QPlainTextEdit::NoWrap);
    QFont f = edit_->font();
    f.setStyleHint(QFont::Monospace);
    f.setFamily("monospace");
    edit_->setFont(f);
    lay->addWidget(edit_, 1);
    connect(filter_, SIGNAL(currentIndexChanged(int)), this, SLOT(onFilterChanged()));
    connect(clear, &QPushButton::clicked, this, [this]() { Clear(); });
}

bool LogView::shown(const Entry& e) const {
    return static_cast<int>(e.level) >= filter_->currentData().toInt();
}

QString LogView::html(const Entry& e) const {
    const char* color = e.level == kError ? "#d33" : (e.level == kWarning ? "#b8860b" : "#333");
    const char* tag = e.level == kError ? "오류" : (e.level == kWarning ? "경고" : "정보");
    QString t = e.text.toHtmlEscaped();
    t.replace("\n", "<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;");
    return QString("<span style=\"color:#888\">%1</span> <span style=\"color:%2;font-weight:%3\">[%4]</span> <span style=\"color:%2\">%5</span>")
        .arg(e.time.toString("HH:mm:ss"))
        .arg(color)
        .arg(e.level == kInfo ? "normal" : "bold")
        .arg(QString::fromUtf8(tag))
        .arg(t);
}

void LogView::Append(Level level, const QString& text) {
    Entry e;
    e.time = QDateTime::currentDateTime();
    e.level = level;
    e.text = text;
    entries_.push_back(e);
    if (entries_.size() > kMaxEntries) entries_.remove(0, entries_.size() - kMaxEntries);
    if (!shown(e)) return;
    QScrollBar* sb = edit_->verticalScrollBar();
    const bool atEnd = sb->value() >= sb->maximum() - 2;   // 맨 아래를 보고 있을 때만 따라 내려간다
    edit_->appendHtml(html(e));
    if (atEnd) sb->setValue(sb->maximum());
}

void LogView::Clear() {
    entries_.clear();
    edit_->clear();
}

int LogView::CountOf(Level level) const {
    int n = 0;
    for (int i = 0; i < entries_.size(); i++) if (entries_[i].level == level) n++;
    return n;
}

QString LogView::ToPlainText() const {
    QString s;
    for (int i = 0; i < entries_.size(); i++) {
        const char* tag = entries_[i].level == kError ? "ERROR" : (entries_[i].level == kWarning ? "WARN" : "INFO");
        s += entries_[i].time.toString("HH:mm:ss") + " [" + tag + "] " + entries_[i].text + "\n";
    }
    return s;
}

void LogView::onFilterChanged() {
    rebuild();
}

void LogView::rebuild() {
    edit_->clear();
    for (int i = 0; i < entries_.size(); i++) if (shown(entries_[i])) edit_->appendHtml(html(entries_[i]));
    edit_->verticalScrollBar()->setValue(edit_->verticalScrollBar()->maximum());
}
