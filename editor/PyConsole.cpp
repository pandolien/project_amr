#include "PyConsole.h"

#include <QEvent>
#include <QFileInfo>
#include <QFont>
#include <QFontDatabase>
#include <QKeyEvent>
#include <QProcessEnvironment>
#include <QVBoxLayout>

#include "ProjectTools.h"

PyConsole::PyConsole(QWidget* parent) : QWidget(parent), proc_(nullptr), hpos_(0) {
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    out_ = new QPlainTextEdit(this);
    out_->setReadOnly(true);
    out_->setMaximumBlockCount(5000);
    const QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    out_->setFont(mono);
    in_ = new QLineEdit(this);
    in_->setFont(mono);
    in_->setPlaceholderText(QString::fromUtf8("예)  arm.set_joint(\"base_yaw\", 1.0)      sim.step(0.5)      arm.joints[\"base_yaw\"].value"));
    in_->installEventFilter(this);
    connect(in_, &QLineEdit::returnPressed, this, &PyConsole::submit);
    lay->addWidget(out_, 1);
    lay->addWidget(in_);
}

void PyConsole::append(const QString& text) {
    out_->moveCursor(QTextCursor::End);
    out_->insertPlainText(text);
    out_->moveCursor(QTextCursor::End);
}

void PyConsole::Start(const QString& projectDir) {
    Stop();
    QString program;
    QStringList args;
    projecttools::FindPython(&program, &args);
    if (program.isEmpty()) { append(QString::fromUtf8("python 을 찾지 못했습니다. python 3 를 설치하거나 환경 변수 SIMUL_PYTHON 에 실행 파일을 지정하세요.\n")); return; }
    if (!QFileInfo(projectDir + QStringLiteral("/Simul/Simul.py")).isFile()) { append(QString::fromUtf8("Simul/Simul.py 가 없는 프로젝트입니다.\n")); return; }
    proc_ = new QProcess(this);
    proc_->setWorkingDirectory(projectDir);
    proc_->setProcessChannelMode(QProcess::MergedChannels);
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert(QStringLiteral("PYTHONUNBUFFERED"), QStringLiteral("1"));
    env.insert(QStringLiteral("PYTHONIOENCODING"), QStringLiteral("utf-8"));
    env.insert(QStringLiteral("SIMUL_LIB"), projectDir + QStringLiteral("/Simul"));
    proc_->setProcessEnvironment(env);
    connect(proc_, &QProcess::readyRead, this, &PyConsole::readOutput);
    connect(proc_, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this,
            [this](int code, QProcess::ExitStatus) { append(QString::fromUtf8("\n--- python 끝 (종료 코드 %1) ---\n").arg(code)); });
    out_->clear();
    // sim 을 만들고 OBJECT/ 의 URDF 를 올린 상태로 시작한다. 오브젝트 이름(arm 등)이 그대로 변수가 된다.
    const QString init = QString::fromUtf8(
        "import sys, os, glob\n"
        "sys.path.insert(0, 'Simul')\n"
        "import Simul\n"
        "sim = Simul.Simulation()\n"
        "for _f in sorted(glob.glob('OBJECT/*.urdf')): sim.add_model(_f)\n"
        "for _n in sim.objects:\n"
        "    if _n.isidentifier(): globals()[_n] = sim.objects[_n]\n"
        "print('sim, 오브젝트: ' + ', '.join(sorted(sim.objects)) + '   (sim.show() 로 3D 창)')\n");
    args << QStringLiteral("-i") << QStringLiteral("-u") << QStringLiteral("-c") << init;
    proc_->start(program, args);
    if (!proc_->waitForStarted(3000)) append(QString::fromUtf8("python 을 시작하지 못했습니다: ") + program + QLatin1Char('\n'));
}

void PyConsole::Stop() {
    if (!proc_) return;
    disconnect(proc_, nullptr, this, nullptr);
    if (proc_->state() != QProcess::NotRunning) { proc_->kill(); proc_->waitForFinished(2000); }
    proc_->deleteLater();
    proc_ = nullptr;
}

void PyConsole::readOutput() {
    if (proc_) append(QString::fromUtf8(proc_->readAll()));
}

void PyConsole::submit() {
    const QString line = in_->text();
    in_->clear();
    if (!line.trimmed().isEmpty()) history_ << line;
    hpos_ = history_.size();
    if (!IsRunning()) { append(QString::fromUtf8("(python 이 돌고 있지 않습니다: 프로젝트를 열거나 실행 > Python CMD 다시 시작)\n")); return; }
    append(line + QLatin1Char('\n'));
    proc_->write((line + QLatin1Char('\n')).toUtf8());
}

bool PyConsole::eventFilter(QObject* obj, QEvent* e) {
    if (obj == in_ && e->type() == QEvent::KeyPress && !history_.isEmpty()) {
        const int k = static_cast<QKeyEvent*>(e)->key();
        if (k == Qt::Key_Up || k == Qt::Key_Down) {
            hpos_ += (k == Qt::Key_Up) ? -1 : 1;
            if (hpos_ < 0) hpos_ = 0;
            if (hpos_ > history_.size()) hpos_ = history_.size();
            in_->setText(hpos_ < history_.size() ? history_[hpos_] : QString());
            return true;
        }
    }
    return QWidget::eventFilter(obj, e);
}
