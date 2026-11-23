#include "EditorWindow.h"

#include <QAction>
#include <QCloseEvent>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFileSystemModel>
#include <QInputDialog>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <QSplitter>
#include <QStatusBar>
#include <QTreeView>
#include <QVBoxLayout>

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
    a = run->addAction(K("main.py 실행 (python 의 3D 창)"));
    a->setShortcut(Qt::Key_F5);
    connect(a, &QAction::triggered, this, [this]() { RunMain(true); });
    a = run->addAction(K("main.py 실행 (창 없이)"));
    a->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_F5));
    connect(a, &QAction::triggered, this, [this]() { RunMain(false); });
    a = run->addAction(K("정지"));
    a->setShortcut(QKeySequence(Qt::SHIFT + Qt::Key_F5));
    connect(a, &QAction::triggered, this, [this]() { StopMain(); });
    run->addSeparator();
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
    const QStringList urdfs = QFileDialog::getOpenFileNames(this, K("프로젝트에 넣을 URDF (여러 개 가능)"), QString::fromUtf8(SIMUL_SOURCE_DIR) + QStringLiteral("/Data/Object"), K("URDF (*.urdf)"));
    if (urdfs.isEmpty()) return;
    const QString parent = QFileDialog::getExistingDirectory(this, K("프로젝트를 만들 위치(폴더)"));
    if (parent.isEmpty()) return;
    bool ok = false;
    const QString name = QInputDialog::getText(this, K("새 프로젝트"), K("프로젝트 이름"), QLineEdit::Normal, QStringLiteral("my_project"), &ok);
    if (!ok || name.trimmed().isEmpty()) return;
    const QString dir = parent + QLatin1Char('/') + name.trimmed();
    if (QFileInfo(dir).exists() && !QDir(dir).entryList(QDir::NoDotAndDotDot | QDir::AllEntries).isEmpty()) {
        QMessageBox::warning(this, K("새 프로젝트"), K("비어 있지 않은 폴더입니다: ") + dir);
        return;
    }
    NewProject(dir, urdfs);
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

void EditorWindow::reloadModels() {
    if (project_.isEmpty()) return;
    view_->viewer()->CloseAll();
    const QStringList urdfs = projecttools::ProjectModels(project_);
    for (int i = 0; i < urdfs.size(); i++) {
        QString err;
        if (!view_->viewer()->AddFile(urdfs[i], &err)) log(QStringLiteral("[") + QFileInfo(urdfs[i]).fileName() + QStringLiteral("] ") + err);
    }
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

void EditorWindow::RunMain(bool window) {
    if (project_.isEmpty()) return;
    StopMain();
    SaveAll();
    QString program;
    QStringList args;
    projecttools::FindPython(&program, &args);
    if (program.isEmpty()) { log(K("python 을 찾지 못했습니다. python 3 를 설치하거나 환경 변수 SIMUL_PYTHON 에 실행 파일을 지정하세요.")); bottom_->setCurrentWidget(log_); return; }
    proc_ = new QProcess(this);
    proc_->setWorkingDirectory(project_);
    proc_->setProcessChannelMode(QProcess::MergedChannels);
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert(QStringLiteral("PYTHONUNBUFFERED"), QStringLiteral("1"));
    env.insert(QStringLiteral("PYTHONIOENCODING"), QStringLiteral("utf-8"));
    env.insert(QStringLiteral("SIMUL_LIB"), project_ + QStringLiteral("/Simul"));
    proc_->setProcessEnvironment(env);
    connect(proc_, &QProcess::readyRead, this, [this]() { if (proc_) log_->appendPlainText(QString::fromUtf8(proc_->readAll()).trimmed()); });
    connect(proc_, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this,
            [this](int code, QProcess::ExitStatus) { log(K("--- main.py 끝 (종료 코드 %1) ---").arg(code)); });
    args << QStringLiteral("main.py");
    if (!window) args << QStringLiteral("--no-window");
    bottom_->setCurrentWidget(log_);
    log(K("--- main.py 실행: ") + program + QLatin1Char(' ') + args.join(QLatin1Char(' ')) + QStringLiteral(" ---"));
    proc_->start(program, args);
}

void EditorWindow::StopMain() {
    if (!proc_) return;
    disconnect(proc_, nullptr, this, nullptr);
    if (proc_->state() != QProcess::NotRunning) { proc_->kill(); proc_->waitForFinished(2000); }
    proc_->deleteLater();
    proc_ = nullptr;
}
