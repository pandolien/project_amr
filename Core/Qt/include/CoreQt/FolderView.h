#ifndef CORECQT_FOLDERVIEW_H
#define CORECQT_FOLDERVIEW_H

#include <QFileSystemModel>
#include <QLineEdit>
#include <QTreeView>
#include <QWidget>

// 디스크 탐색기 (엔진 밖의 독립 위젯). 폴더로 저장한 URDF 구성(내폴더/ 내폴더.urdf, obj/ ...)을 포함해 디스크의 폴더를 탐색한다.
// URDF 와 메시/재질/텍스처 파일만 보이고, 열 수 있는 파일을 더블클릭하면 openRequested 가 나온다. 이 위젯은 파일을 직접 열지 않는다
// (호스트가 신호를 받아 ModelViewer::AddFile 등을 부른다).
class FolderView : public QWidget {
    Q_OBJECT
public:
    explicit FolderView(QWidget* parent = nullptr);

    QString RootPath() const { return root_; }
    void SetRootPath(const QString& dir);       // 이 폴더를 맨 위로 보여 준다
    void Reveal(const QString& path);           // 경로가 보이게 한다 (루트 밖이면 그 부모를 루트로 바꾼다). 선택하고 펼친다
    QString CurrentPath() const;                // 선택한 파일/폴더 (없으면 빈 문자열)
    QTreeView* Tree() const { return tree_; }
    QFileSystemModel* Model() const { return model_; }

    // 열 수 있는 파일인가 (.urdf .glb .gltf .stl .obj .dae)
    static bool IsOpenableFile(const QString& path);
    // 폴더로 저장한 구성인가 (<폴더이름>.urdf 가 있다). 있으면 그 URDF 경로를 돌려준다.
    static bool FolderUrdf(const QString& dir, QString* urdfPath = nullptr);

signals:
    void openRequested(const QString& path);    // 파일 또는 URDF 폴더를 장면에 추가해 달라는 요청

private slots:
    void onActivated(const QModelIndex& idx);
    void onContextMenu(const QPoint& pos);
    void onLoaded(const QString& dir);
    void onPathEdited();
    void goUp();
    void chooseRoot();

private:
    void trySelectPending();
    QFileSystemModel* model_;
    QTreeView* tree_;
    QLineEdit* path_;
    QString root_;
    QString pending_;
};

#endif  // CORECQT_FOLDERVIEW_H
