#include "CoreQt/FolderView.h"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QToolButton>
#include <QVBoxLayout>

FolderView::FolderView(QWidget* parent) : QWidget(parent), model_(new QFileSystemModel(this)), tree_(new QTreeView(this)), path_(new QLineEdit(this)) {
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->setContentsMargins(4, 4, 4, 4);
    lay->setSpacing(4);
    QHBoxLayout* top = new QHBoxLayout;
    top->setSpacing(4);
    QToolButton* up = new QToolButton(this);
    up->setText(QString::fromUtf8("위로"));
    up->setToolTip(QString::fromUtf8("위 폴더로"));
    QToolButton* pick = new QToolButton(this);
    pick->setText(QString::fromUtf8("폴더..."));
    pick->setToolTip(QString::fromUtf8("보여 줄 폴더 고르기"));
    path_->setPlaceholderText(QString::fromUtf8("폴더 경로 (Enter)"));
    top->addWidget(up);
    top->addWidget(path_, 1);
    top->addWidget(pick);
    lay->addLayout(top);
    lay->addWidget(tree_, 1);

    model_->setFilter(QDir::AllDirs | QDir::Files | QDir::NoDotAndDotDot);   // 폴더는 모두, 파일은 아래 이름 필터에 맞는 것만
    model_->setNameFilters(QStringList() << "*.urdf" << "*.stl" << "*.obj" << "*.mtl" << "*.dae" << "*.glb" << "*.gltf"
                                         << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp" << "*.tga");
    model_->setNameFilterDisables(false);   // 맞지 않는 파일은 흐리게가 아니라 숨긴다
    tree_->setModel(model_);
    tree_->setHeaderHidden(false);
    tree_->setUniformRowHeights(true);
    tree_->setSortingEnabled(true);
    tree_->sortByColumn(0, Qt::AscendingOrder);
    tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    tree_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tree_->setColumnHidden(2, true);   // 종류
    tree_->setColumnHidden(3, true);   // 수정한 날짜
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);

    connect(tree_, &QTreeView::doubleClicked, this, &FolderView::onActivated);
    connect(tree_, &QTreeView::customContextMenuRequested, this, &FolderView::onContextMenu);
    connect(model_, &QFileSystemModel::directoryLoaded, this, &FolderView::onLoaded);
    connect(path_, &QLineEdit::returnPressed, this, &FolderView::onPathEdited);
    connect(up, &QToolButton::clicked, this, &FolderView::goUp);
    connect(pick, &QToolButton::clicked, this, &FolderView::chooseRoot);

    const QString cwd = QDir::currentPath();
    SetRootPath(cwd == "/" || cwd.isEmpty() ? QDir::homePath() : cwd);
}

bool FolderView::IsOpenableFile(const QString& path) {
    const QString e = QFileInfo(path).suffix().toLower();
    return e == "urdf" || e == "glb" || e == "gltf" || e == "stl" || e == "obj" || e == "dae";
}

bool FolderView::FolderUrdf(const QString& dir, QString* urdfPath) {
    const QDir d(dir);
    const QString p = d.filePath(d.dirName() + ".urdf");
    if (!QFileInfo(p).isFile()) return false;
    if (urdfPath) *urdfPath = p;
    return true;
}

void FolderView::SetRootPath(const QString& dirIn) {
    const QString dir = QDir::cleanPath(QFileInfo(dirIn).absoluteFilePath());
    if (!QFileInfo(dir).isDir()) return;
    root_ = dir;
    path_->setText(QDir::toNativeSeparators(dir));
    model_->setRootPath(dir);
    tree_->setRootIndex(model_->index(dir));
}

QString FolderView::CurrentPath() const {
    const QModelIndex i = tree_->currentIndex();
    return i.isValid() ? model_->filePath(i) : QString();
}

void FolderView::Reveal(const QString& pathIn) {
    const QString path = QDir::cleanPath(QFileInfo(pathIn).absoluteFilePath());
    if (!QFileInfo(path).exists()) return;
    if (!(path + "/").startsWith(root_ + "/") || path == root_) SetRootPath(path == root_ ? path : QFileInfo(path).absolutePath());
    pending_ = path;
    trySelectPending();
}

void FolderView::trySelectPending() {
    if (pending_.isEmpty()) return;
    const QModelIndex i = model_->index(pending_);
    if (!i.isValid()) return;   // 아직 읽히지 않았다. directoryLoaded 에서 다시 시도한다
    tree_->setCurrentIndex(i);
    if (model_->isDir(i)) tree_->expand(i);
    tree_->scrollTo(i);
    pending_.clear();
}

void FolderView::onLoaded(const QString&) { trySelectPending(); }

void FolderView::onActivated(const QModelIndex& idx) {
    const QString p = model_->filePath(idx);
    if (model_->isDir(idx)) return;   // 폴더는 펼치기/접기만 (열기는 오른쪽 클릭 메뉴)
    if (IsOpenableFile(p)) emit openRequested(p);
}

void FolderView::onContextMenu(const QPoint& pos) {
    const QModelIndex idx = tree_->indexAt(pos);
    QMenu menu(this);
    if (idx.isValid()) {
        const QString p = model_->filePath(idx);
        if (model_->isDir(idx)) {
            if (FolderUrdf(p)) menu.addAction(QString::fromUtf8("폴더 열기 (장면에 추가)"), this, [this, p]() { emit openRequested(p); });
            menu.addAction(QString::fromUtf8("이 폴더를 맨 위로"), this, [this, p]() { SetRootPath(p); });
        } else if (IsOpenableFile(p)) {
            menu.addAction(QString::fromUtf8("열기 (장면에 추가)"), this, [this, p]() { emit openRequested(p); });
        }
    }
    menu.addAction(QString::fromUtf8("위 폴더로"), this, &FolderView::goUp);
    menu.exec(tree_->viewport()->mapToGlobal(pos));
}

void FolderView::onPathEdited() {
    const QString p = QDir::fromNativeSeparators(path_->text().trimmed());
    if (QFileInfo(p).isDir()) SetRootPath(p);
    else path_->setText(QDir::toNativeSeparators(root_));   // 없는 경로는 되돌린다
}

void FolderView::goUp() {
    QDir d(root_);
    if (d.cdUp()) SetRootPath(d.absolutePath());
}

void FolderView::chooseRoot() {
    const QString d = QFileDialog::getExistingDirectory(this, QString::fromUtf8("보여 줄 폴더"), root_);
    if (!d.isEmpty()) SetRootPath(d);
}
