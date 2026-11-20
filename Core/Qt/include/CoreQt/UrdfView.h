#ifndef CORECQT_URDFVIEW_H
#define CORECQT_URDFVIEW_H

#include <QLabel>
#include <QPlainTextEdit>
#include <QTimer>
#include <QWidget>

class ModelViewer;

// 엔진(ModelViewer) 밖에 두는 URDF 파일 보기 (읽기 전용). 지금 장면이 URDF 로 저장되면 어떤 내용이 되는지를 그대로 보여 준다.
// 엔진의 공개 함수(UrdfText, Robot, SetSelectedLink)와 신호만 쓴다.
//  - 장면이 바뀌면(모델 추가/제거, 이동, 크기, 도형 추가) 잠깐 모았다가 한 번 갱신한다. 보던 위치는 유지한다.
//  - 3D 화면에서 링크를 선택하면 그 <link> 블록을 강조하고 그 위치로 이동한다.
//  - 텍스트에서 <link> 나 <joint> 블록 안을 클릭하면 3D 에서 그 링크(관절이면 자식 링크)가 선택된다.
class UrdfView : public QWidget {
    Q_OBJECT
public:
    explicit UrdfView(ModelViewer* viewer, QWidget* parent = nullptr);

    QString Text() const { return edit_->toPlainText(); }
    QPlainTextEdit* Edit() const { return edit_; }
    // 지연 없이 바로 갱신 (테스트/호스트용)
    void RefreshNow();
    // 링크 번호(Robot::links 인덱스)의 블록을 강조하고 그리로 이동한다. -1 이면 강조를 지운다.
    void ShowLink(int link);
    // 텍스트의 한 줄(0부터)이 속한 <link>/<joint> 가 가리키는 링크 번호. 없으면 -1.
    int LinkAtLine(int line) const;
    // 이름이 name 인 <link> 블록의 줄 범위. 없으면 false.
    bool LinkRange(const QString& name, int* firstLine, int* lastLine) const;

protected:
    bool eventFilter(QObject* obj, QEvent* e) override;

private slots:
    void scheduleRefresh();

private:
    void highlightRange(int first, int last);
    ModelViewer* viewer_;
    QLabel* header_;
    QPlainTextEdit* edit_;
    QTimer timer_;
    int shownLink_;
    bool settingSelection_;
};

#endif  // CORECQT_URDFVIEW_H
