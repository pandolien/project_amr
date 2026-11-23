#include "EditorWindow.h"

#include <QAction>
#include <QCloseEvent>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFileSystemModel>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPointer>
#include <QProcessEnvironment>
#include <QTimer>
#include <QSplitter>
#include <QStatusBar>
#include <QTreeView>
#include <QVBoxLayout>

#include <cstring>

#include "3d/dhPose.h"
#include "CodeEdit.h"
#include "ProjectTools.h"
#include "PyConsole.h"

namespace {
QString K(const char* s) { return QString::fromUtf8(s); }

QWidget* Labeled(const QString& title, QWidget* w) {
    QWidget* box = new QWidget;
    QVBoxLayout* lay = new QVBoxLayout(box);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(0);
    QLabel* head = new QLabel(QStringLiteral(" ") + title);
    head->setStyleSheet(QStringLiteral("background:#dde3ea; padding:2px;"));
    lay->addWidget(head);
    lay->addWidget(w, 1);
    return box;
}
}  // namespace

EditorWindow::EditorWindow(const QString& projectDir) : QMainWindow(), proc_(nullptr) {
    setWindowTitle(QStringLiteral("Simul Editor"));
    resize(1400, 820);

    // 솔루션 View
    fs_ = new QFileSystemModel(this);
    fs_->setNameFilters(QStringList() << QStringLiteral("*.py") << QStringLiteral("*.urdf") << QStringLiteral("*.txt") << QStringLiteral("*.md") << QStringLiteral("*.obj") << QStringLiteral("*.stl") << QStringLiteral("*.glb") << QStringLiteral("*.dll") << QStringLiteral("*.so"));
    fs_->setNameFilterDisables(false);
    tree_ = new QTreeView;
    tree_->setModel(fs_);
    for (int c = 1; c < 4; c++) tree_->hideColumn(c);
    tree_->setHeaderHidden(true);
    connect(tree_, &QTreeView::doubleClicked, this, [this](const QModelIndex& i) { OpenFile(fs_->filePath(i)); });

    // 3D View
    view_ = new SimulView(nullptr, SimulView::kSelectionPanel | SimulView::kMouseEdit | SimulView::kDropFiles);

    // Python CMD / 실행 결과
    console_ = new PyConsole;
    connect(console_, &PyConsole::feed, this, &EditorWindow::applyFeed);
    log_ = new QPlainTextEdit;
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(5000);
    bottom_ = new QTabWidget;
    bottom_->addTab(console_, K("Python CMD"));
    bottom_->addTab(log_, K("실행 결과"));

    QSplitter* mid = new QSplitter(Qt::Vertical);
    mid->addWidget(view_);
    mid->addWidget(bottom_);
    mid->setSizes(QList<int>() << 440 << 340);

    // Python Editor
    editors_ = new QTabWidget;
    editors_->setTabsClosable(true);
    connect(editors_, &QTabWidget::tabCloseRequested, this, &EditorWindow::onCloseTab);

    QSplitter* main = new QSplitter(Qt::Horizontal);
    main->addWidget(Labeled(K("솔루션"), tree_));
    main->addWidget(mid);
    main->addWidget(Labeled(K("Python Editor"), editors_));
    main->setSizes(QList<int>() << 250 << 560 << 590);
    setCentralWidget(main);
    statusBar()->showMessage(K("프로젝트를 열거나 파일 > 새 프로젝트로 시작하세요"));
    buildMenus();
    if (!projectDir.isEmpty()) OpenProject(projectDir);
}

EditorWindow::~EditorWindow() {
    StopMain();
    console_->Stop();
}

void EditorWindow::buildMenus() {
    QMenu* file = menuBar()->addMenu(K("파일"));
    QAction* a = file->addAction(K("새 프로젝트..."));
    a->setShortcut(QKeySequence::New);
    connect(a, &QAction::triggered, this, &EditorWindow::onNewProject);
    a = file->addAction(K("OBJECT 에 URDF 추가..."));
    connect(a, &QAction::triggered, this, &EditorWindow::onAddUrdf);
    a = file->addAction(K("프로젝트 열기..."));
    a->setShortcut(QKeySequence::Open);
    connect(a, &QAction::triggered, this, &EditorWindow::onOpenProject);
    file->addSeparator();
    a = file->addAction(K("저장"));
    a->setShortcut(QKeySequence::Save);
    connect(a, &QAction::triggered, this, &EditorWindow::onSave);
    a = file->addAction(K("모두 저장"));
    a->setShortcut(QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_S));
    connect(a, &QAction::triggered, this, [this]() { SaveAll(); });
    file->addSeparator();
    a = file->addAction(K("끝내기"));
    connect(a, &QAction::triggered, this, &QWidget::close);

    QMenu* run = menuBar()->addMenu(K("실행"));
    a = run->addAction(K("main.py 실행 (3D View 에서 보기)"));
    a->setShortcut(Qt::Key_F5);
    connect(a, &QAction::triggered, this, [this]() { RunMain(InEditor); });
    a = run->addAction(K("main.py 실행 (python 의 별도 3D 창)"));
    a->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_F5));
    connect(a, &QAction::triggered, this, [this]() { RunMain(Window); });
    a = run->addAction(K("main.py 실행 (창 없이, 5 초)"));
    connect(a, &QAction::triggered, this, [this]() { RunMain(Headless); });
    a = run->addAction(K("정지"));
    a->setShortcut(QKeySequence(Qt::SHIFT + Qt::Key_F5));
    connect(a, &QAction::triggered, this, [this]() { StopMain(); });
    run->addSeparator();
    a = run->addAction(K("venv 만들기 (프로젝트의 python 환경)"));
    connect(a, &QAction::triggered, this, &EditorWindow::onCreateVenv);
    a = run->addAction(K("Python CMD 다시 시작"));
    connect(a, &QAction::triggered, this, [this]() { if (!project_.isEmpty()) console_->Start(project_); });

    QMenu* viewMenu = menuBar()->addMenu(K("보기"));
    a = viewMenu->addAction(K("모델 다시 읽기"));
    a->setShortcut(Qt::Key_F6);
    connect(a, &QAction::triggered, this, &EditorWindow::reloadModels);
    a = viewMenu->addAction(K("구조 트리"));
    a->setCheckable(true);
    a->setChecked(false);
    connect(a, &QAction::toggled, this, [this](bool on) { view_->SetTreeVisible(on); });
    a = viewMenu->addAction(K("선택 패널 (오브젝트 / 관절)"));
    a->setCheckable(true);
    a->setChecked(true);
    connect(a, &QAction::toggled, this, [this](bool on) { view_->SetSelectionPanelVisible(on); });
}

void EditorWindow::log(const QString& text) {
    log_->appendPlainText(text);
}

CodeEdit* EditorWindow::CurrentEditor() const { return qobject_cast<CodeEdit*>(editors_->currentWidget()); }

void EditorWindow::onNewProject() {
    const QString parent = QFileDialog::getExistingDirectory(this, K("새 프로젝트를 만들 위치(폴더)"));
    if (parent.isEmpty()) return;
    bool ok = false;
    const QString name = QInputDialog::getText(this, K("새 프로젝트"), K("프로젝트 이름 (이 이름의 폴더가 만들어집니다)"), QLineEdit::Normal, QStringLiteral("my_project"), &ok);
    if (!ok || name.trimmed().isEmpty()) return;
    const QString dir = parent + QLatin1Char('/') + name.trimmed();
    if (QFileInfo(dir).exists() && !QDir(dir).entryList(QDir::NoDotAndDotDot | QDir::AllEntries).isEmpty()) {
        QMessageBox::warning(this, K("새 프로젝트"), K("비어 있지 않은 폴더입니다: ") + dir);
        return;
    }
    NewProject(dir, QStringList());
}

void EditorWindow::onAddUrdf() {
    if (project_.isEmpty()) { QMessageBox::information(this, K("URDF 추가"), K("먼저 프로젝트를 만들거나 여세요")); return; }
    const QStringList urdfs = QFileDialog::getOpenFileNames(this, K("OBJECT 에 넣을 URDF (여러 개 가능)"), QString::fromUtf8(SIMUL_SOURCE_DIR) + QStringLiteral("/Data/Object"), K("URDF (*.urdf)"));
    if (urdfs.isEmpty()) return;
    QString err;
    QStringList lines, meshDirs;
    int missing = 0;
    for (;;) {
        lines.clear();
        if (!projecttools::AddModels(project_, urdfs, &err, &lines, meshDirs, &missing)) { log(err); QMessageBox::warning(this, K("URDF 추가"), err); return; }
        if (missing == 0) break;
        QStringList warn;
        for (int i = 0; i < lines.size(); i++) if (lines[i].startsWith(K("[경고]"))) warn << lines[i];
        if (warn.size() > 8) { const int n = warn.size(); warn = warn.mid(0, 8); warn << QStringLiteral("... (") + QString::number(n) + K("개)"); }
        const QString dirStart = QFileInfo(urdfs[0]).absolutePath();
        if (QMessageBox::question(this, K("mesh 를 찾지 못했습니다"), warn.join(QLatin1Char('\n')) + K("\n\nmesh 파일이 들어 있는 폴더를 직접 고르시겠습니까?")) != QMessageBox::Yes) break;
        const QString d = QFileDialog::getExistingDirectory(this, K("mesh 가 있는 폴더 (이 폴더 아래를 모두 찾습니다)"), dirStart);
        if (d.isEmpty()) break;
        meshDirs << d;
    }
    bottom_->setCurrentWidget(log_);
    log(lines.join(QLatin1Char('\n')));
    reloadModels();
    console_->Start(project_);
}

bool EditorWindow::NewProject(const QString& dir, const QStringList& urdfs) {
    QString err;
    QStringList lines;
    if (!projecttools::CreateProject(dir, urdfs, &err, &lines)) {
        log(err);
        QMessageBox::warning(this, K("새 프로젝트"), err);
        return false;
    }
    bottom_->setCurrentWidget(log_);
    log(lines.join(QLatin1Char('\n')));
    return OpenProject(dir);
}

void EditorWindow::onOpenProject() {
    const QString dir = QFileDialog::getExistingDirectory(this, K("프로젝트 폴더 (main.py 가 있는 폴더)"));
    if (!dir.isEmpty()) OpenProject(dir);
}

bool EditorWindow::OpenProject(const QString& dirIn) {
    const QString dir = QDir::cleanPath(QFileInfo(dirIn).absoluteFilePath());
    if (!projecttools::IsProject(dir)) {
        QMessageBox::warning(this, K("프로젝트 열기"), K("main.py 가 없는 폴더입니다: ") + dir);
        return false;
    }
    if (!closeAllTabs()) return false;
    StopMain();
    offerUpgrade(dir);
    project_ = dir;
    setWindowTitle(QStringLiteral("Simul Editor - ") + dir);
    fs_->setRootPath(dir);
    tree_->setRootIndex(fs_->index(dir));
    reloadModels();
    OpenFile(dir + QStringLiteral("/main.py"));
    console_->Start(dir);
    statusBar()->showMessage(K("프로젝트: ") + dir);
    return true;
}

// 예전 구조이거나 Simul/ 이 비었거나 에디터 연동 이전 Simul.py 이면 갱신을 권한다. 갱신했거나 필요 없으면 true.
bool EditorWindow::offerUpgrade(const QString& dir) {
    if (!projecttools::NeedsUpgrade(dir)) return true;
    if (QMessageBox::question(this, K("프로젝트 갱신"), K("이 프로젝트는 예전 구조이거나 Simul/ (DLL, Simul.py) 이 최신이 아닙니다.\n(3D View 에서 움직임을 보려면 갱신이 필요합니다)\nSimul/ 을 채우고, main.py 와 틀 그대로인 Code/*.py 도 class 구조로 바꿀까요?\n(바꾸는 파일은 .bak 으로 남기고, 직접 쓴 코드는 그대로 둡니다)")) != QMessageBox::Yes) return false;
    QString uerr;
    QStringList ulog;
    const bool ok = projecttools::UpgradeProject(dir, &uerr, &ulog);
    if (!ok) QMessageBox::warning(this, K("프로젝트 갱신"), uerr);
    bottom_->setCurrentWidget(log_);
    log(ulog.join(QLatin1Char('\n')));
    return ok;
}

void EditorWindow::reloadModels() {
    if (project_.isEmpty()) return;
    view_->viewer()->CloseAll();
    modelNames_.clear();
    const QStringList urdfs = projecttools::ProjectModels(project_);
    for (int i = 0; i < urdfs.size(); i++) {
        QString err;
        if (view_->viewer()->AddFile(urdfs[i], &err)) modelNames_ << QFileInfo(urdfs[i]).completeBaseName();
        else log(QStringLiteral("[") + QFileInfo(urdfs[i]).fileName() + QStringLiteral("] ") + err);
    }
}

// python(시뮬레이션)이 보낸 오브젝트 상태 {"o": {이름: {"p": [x,y,z,qx,qy,qz,qw], "j": {관절: 값}}}} 를 3D View 에 그대로 옮긴다
void EditorWindow::applyFeed(const QString& json) {
    feedSeen_ = true;
    const QJsonObject objs = QJsonDocument::fromJson(json.toUtf8()).object().value(QStringLiteral("o")).toObject();
    simul::urdf::JointValues q = view_->viewer()->CurrentJointValues();
    for (QJsonObject::const_iterator it = objs.begin(); it != objs.end(); ++it) {
        const int index = modelNames_.indexOf(it.key());
        if (index < 0) continue;
        const QJsonObject o = it.value().toObject();
        const QJsonArray p = o.value(QStringLiteral("p")).toArray();
        if (p.size() == 7) {
            dhPose pose;
            pose.position = dhVector(static_cast<float>(p[0].toDouble()), static_cast<float>(p[1].toDouble()), static_cast<float>(p[2].toDouble()));
            pose.orientation.x = static_cast<float>(p[3].toDouble());
            pose.orientation.y = static_cast<float>(p[4].toDouble());
            pose.orientation.z = static_cast<float>(p[5].toDouble());
            pose.orientation.w = static_cast<float>(p[6].toDouble());
            const dhMat m = pose.Normalized().ToMat();
            const dhMat cur = view_->viewer()->ModelPose(index);
            if (std::memcmp(cur.v, m.v, sizeof(m.v)) != 0) view_->viewer()->SetModelPose(index, m);
        }
        const QJsonObject js = o.value(QStringLiteral("j")).toObject();
        for (QJsonObject::const_iterator j = js.begin(); j != js.end(); ++j) q[simul::String(j.key().toUtf8().constData())] = static_cast<float>(j.value().toDouble());
    }
    view_->viewer()->SetJointValues(q);
}

void EditorWindow::OpenFile(const QString& path) {
    const QFileInfo fi(path);
    const QString ext = fi.suffix().toLower();
    if (!fi.isFile() || (ext != QLatin1String("py") && ext != QLatin1String("urdf") && ext != QLatin1String("txt") && ext != QLatin1String("md"))) return;
    for (int i = 0; i < editors_->count(); i++) {
        CodeEdit* e = qobject_cast<CodeEdit*>(editors_->widget(i));
        if (e && e->path() == fi.absoluteFilePath()) { editors_->setCurrentIndex(i); return; }
    }
    CodeEdit* ed = new CodeEdit(fi.absoluteFilePath());
    connect(ed->document(), &QTextDocument::modificationChanged, this, [this, ed]() { markTab(ed); });
    const int idx = editors_->addTab(ed, fi.fileName());
    editors_->setTabToolTip(idx, fi.absoluteFilePath());
    editors_->setCurrentIndex(idx);
}

void EditorWindow::markTab(CodeEdit* ed) {
    const int i = editors_->indexOf(ed);
    if (i >= 0) editors_->setTabText(i, QFileInfo(ed->path()).fileName() + (ed->document()->isModified() ? QStringLiteral(" *") : QString()));
}

void EditorWindow::onSave() {
    CodeEdit* e = CurrentEditor();
    QString err;
    if (e && !e->Save(&err)) QMessageBox::warning(this, K("저장"), err);
}

bool EditorWindow::SaveAll() {
    bool ok = true;
    for (int i = 0; i < editors_->count(); i++) {
        CodeEdit* e = qobject_cast<CodeEdit*>(editors_->widget(i));
        QString err;
        if (e && e->document()->isModified() && !e->Save(&err)) { log(err); ok = false; }
    }
    return ok;
}

void EditorWindow::onCloseTab(int index) {
    CodeEdit* e = qobject_cast<CodeEdit*>(editors_->widget(index));
    if (e && e->document()->isModified()) {
        const QMessageBox::StandardButton r = QMessageBox::question(this, K("저장"), QFileInfo(e->path()).fileName() + K(" 을(를) 저장할까요?"), QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
        if (r == QMessageBox::Cancel) return;
        if (r == QMessageBox::Yes) e->Save();
    }
    editors_->removeTab(index);
    delete e;
}

bool EditorWindow::closeAllTabs() {
    while (editors_->count() > 0) {
        const int before = editors_->count();
        onCloseTab(0);
        if (editors_->count() == before) return false;   // 취소했다
    }
    return true;
}

void EditorWindow::closeEvent(QCloseEvent* e) {
    if (!closeAllTabs()) { e->ignore(); return; }
    e->accept();
}

void EditorWindow::RunMain(RunMode mode) {
    if (project_.isEmpty()) return;
    StopMain();
    SaveAll();
    if (mode == InEditor && !offerUpgrade(project_)) { log(K("갱신하지 않아 3D View 로 실행하지 않습니다 (실행 > 별도 3D 창은 됩니다)")); bottom_->setCurrentWidget(log_); return; }
    if (mode == InEditor) reloadModels();   // 처음 자세에서 시작
    QString program;
    QStringList args;
    projecttools::FindPython(&program, &args, project_);
    if (program.isEmpty()) { log(K("python 을 찾지 못했습니다. python 3 를 설치하세요 (python.org, 설치할 때 \"Add python.exe to PATH\" 선택. Windows 의 Microsoft Store 가짜 python 은 쓸 수 없습니다). 이미 설치했다면 환경 변수 SIMUL_PYTHON 에 python 실행 파일을 지정하세요.")); bottom_->setCurrentWidget(log_); return; }
    proc_ = new QProcess(this);
    proc_->setWorkingDirectory(project_);
    proc_->setProcessChannelMode(QProcess::MergedChannels);
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert(QStringLiteral("PYTHONUNBUFFERED"), QStringLiteral("1"));
    env.insert(QStringLiteral("PYTHONIOENCODING"), QStringLiteral("utf-8"));
    env.insert(QStringLiteral("SIMUL_LIB"), project_ + QStringLiteral("/Simul"));
    if (mode == InEditor) env.insert(QStringLiteral("SIMUL_FEED"), QStringLiteral("1"));   // 오브젝트 상태를 한 줄씩 에디터로 보낸다
    proc_->setProcessEnvironment(env);
    runBuf_.clear();
    connect(proc_, &QProcess::readyRead, this, [this]() {
        if (!proc_) return;
        runBuf_ += proc_->readAll();
        for (;;) {
            const int nl = runBuf_.indexOf('\n');
            if (nl < 0) break;
            const QByteArray line = runBuf_.left(nl).trimmed();
            runBuf_.remove(0, nl + 1);
            if (line.startsWith("@@SIMUL ")) applyFeed(QString::fromUtf8(line.mid(8)));
            else if (!line.isEmpty()) log_->appendPlainText(QString::fromUtf8(line));
        }
    });
    connect(proc_, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this,
            [this](int code, QProcess::ExitStatus) {
                log(K("--- main.py 끝 (종료 코드 %1) ---").arg(code));
                if (code == 9009) log(K("[알림] 종료 코드 9009 는 python 이 진짜가 아니라는 뜻입니다 (Microsoft Store 의 가짜 python). python.org 에서 python 3 를 설치하거나 환경 변수 SIMUL_PYTHON 을 지정하세요."));
            });
    args << QStringLiteral("main.py");
    if (mode == InEditor) args << QStringLiteral("--editor");
    else if (mode == Headless) args << QStringLiteral("--no-window");
    bottom_->setCurrentWidget(log_);
    log(K("--- main.py 실행: ") + program + QLatin1Char(' ') + args.join(QLatin1Char(' ')) + QStringLiteral(" ---"));
    feedSeen_ = false;
    proc_->start(program, args);
    if (mode == InEditor) {
        QPointer<QProcess> guard(proc_);
        QTimer::singleShot(5000, this, [this, guard]() {
            if (guard && guard == proc_ && guard->state() != QProcess::NotRunning && !feedSeen_)
                log(K("[알림] 5초가 지나도 python 에서 오브젝트 상태가 오지 않습니다. 위 로그의 오류를 확인하세요. 오류가 없으면 main.py 에 --editor 처리(Main class)가 있는지, Simul/Simul.py 가 최신인지 확인하세요 (프로젝트를 다시 열어 갱신)."));
        });
    }
}

// 프로젝트 폴더에 venv 를 만들고(python -m venv), requirements.txt 에 패키지가 있으면 설치한다. 만들면 F5 와 Python CMD 가 그 python 을 쓴다.
void EditorWindow::onCreateVenv() {
    if (project_.isEmpty()) { QMessageBox::information(this, K("venv"), K("먼저 프로젝트를 만들거나 여세요")); return; }
    if (!projecttools::VenvPython(project_).isEmpty()) {
        if (QMessageBox::question(this, K("venv"), K("이미 venv 가 있습니다. requirements.txt 만 다시 설치할까요?")) != QMessageBox::Yes) return;
    }
    QString program;
    QStringList args;
    projecttools::FindPython(&program, &args);   // 시스템 python
    if (program.isEmpty()) { log(K("python 을 찾지 못했습니다. python 3 를 설치하세요 (python.org, 설치할 때 \"Add python.exe to PATH\" 선택. Windows 의 Microsoft Store 가짜 python 은 쓸 수 없습니다). 이미 설치했다면 환경 변수 SIMUL_PYTHON 에 python 실행 파일을 지정하세요.")); bottom_->setCurrentWidget(log_); return; }
    bottom_->setCurrentWidget(log_);
    const bool had = !projecttools::VenvPython(project_).isEmpty();
    QProcess* p = new QProcess(this);
    p->setWorkingDirectory(project_);
    p->setProcessChannelMode(QProcess::MergedChannels);
    connect(p, &QProcess::readyRead, this, [this, p]() { log_->appendPlainText(QString::fromUtf8(p->readAll()).trimmed()); });
    auto finish = [this, p](int code) {
        log(K("--- venv 끝 (종료 코드 %1) ---").arg(code));
        p->deleteLater();
        if (code == 0 && !project_.isEmpty()) console_->Start(project_);   // 새 python 으로 다시 시작
    };
    auto install = [this, p, finish]() {
        const QString vpy = projecttools::VenvPython(project_);
        QFile rf(project_ + QStringLiteral("/requirements.txt"));
        bool any = false;
        if (rf.open(QIODevice::ReadOnly)) {
            const QStringList lines = QString::fromUtf8(rf.readAll()).split(QLatin1Char('\n'));
            for (int i = 0; i < lines.size(); i++) if (!lines[i].trimmed().isEmpty() && !lines[i].trimmed().startsWith(QLatin1Char('#'))) any = true;
        }
        if (vpy.isEmpty()) { finish(1); return; }
        if (!any) { finish(0); return; }
        log(K("pip install -r requirements.txt"));
        disconnect(p, nullptr, this, nullptr);
        connect(p, &QProcess::readyRead, this, [this, p]() { log_->appendPlainText(QString::fromUtf8(p->readAll()).trimmed()); });
        connect(p, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this, [finish](int code, QProcess::ExitStatus) { finish(code); });
        p->start(vpy, QStringList() << QStringLiteral("-m") << QStringLiteral("pip") << QStringLiteral("install") << QStringLiteral("-r") << QStringLiteral("requirements.txt"));
    };
    if (had) { install(); return; }
    log(K("--- venv 만들기: ") + program + QLatin1Char(' ') + (args + QStringList() << QStringLiteral("-m") << QStringLiteral("venv") << QStringLiteral("venv")).join(QLatin1Char(' ')) + QStringLiteral(" ---"));
    connect(p, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this, [install, finish](int code, QProcess::ExitStatus) { if (code != 0) finish(code); else install(); });
    p->start(program, args + QStringList() << QStringLiteral("-m") << QStringLiteral("venv") << QStringLiteral("venv"));
}

void EditorWindow::StopMain() {
    if (!proc_) return;
    disconnect(proc_, nullptr, this, nullptr);
    if (proc_->state() != QProcess::NotRunning) { proc_->kill(); proc_->waitForFinished(2000); }
    proc_->deleteLater();
    proc_ = nullptr;
}
