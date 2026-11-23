#ifndef SIMUL_EDITOR_EDITORWINDOW_H
#define SIMUL_EDITOR_EDITORWINDOW_H

#include <QMainWindow>
#include <QPlainTextEdit>
#include <QProcess>
#include <QTabWidget>

#include "CoreQt/SimulView.h"

class CodeEdit;
class PyConsole;
class QFileSystemModel;
class QTreeView;

// 화면: 메뉴 / 솔루션 View(왼쪽) / 3D View + Python CMD(가운데) / Python Editor(오른쪽)
class EditorWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit EditorWindow(const QString& projectDir = QString());
    ~EditorWindow() override;
    bool OpenProject(const QString& dir);
    bool NewProject(const QString& dir, const QStringList& urdfs);   // 대화상자 없이 (시험용)
    QString ProjectDir() const { return project_; }
    SimulView* View() const { return view_; }
    CodeEdit* CurrentEditor() const;
    void OpenFile(const QString& path);
    bool SaveAll();
    enum RunMode { InEditor, Window, Headless };   // 에디터의 3D View 에 보인다 / python 의 별도 3D 창 / 창 없이 (5 초)
    void RunMain(RunMode mode);
    void StopMain();
    PyConsole* Console() const { return console_; }
    QPlainTextEdit* Log() const { return log_; }
    QProcess* MainProcess() const { return proc_; }

protected:
    void closeEvent(QCloseEvent* e) override;

private slots:
    void onNewProject();
    void onOpenProject();
    void onAddUrdf();
    void onCreateVenv();
    void onSave();
    void onCloseTab(int index);
    void reloadModels();
    void applyFeed(const QString& json);

private:
    void buildMenus();
    bool closeAllTabs();
    void markTab(CodeEdit* ed);
    bool offerUpgrade(const QString& dir);
    void log(const QString& text);

    SimulView* view_;
    QFileSystemModel* fs_;
    QTreeView* tree_;
    QTabWidget* editors_;
    QTabWidget* bottom_;
    PyConsole* console_;
    QPlainTextEdit* log_;
    QProcess* proc_;
    QString project_;
    QStringList modelNames_;   // 3D View 의 모델 순서 (오브젝트 이름 = URDF 파일 이름)
    QByteArray runBuf_;
    bool feedSeen_ = false;
};

#endif
