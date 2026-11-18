#ifndef CORECQT_FLOATINGPANEL_H
#define CORECQT_FLOATINGPANEL_H

#include <QEvent>
#include <QFrame>
#include <QLabel>
#include <QPoint>
#include <QString>

// 3D 화면 위에 떠 있는 작은 창: 제목줄을 끌어 옮기고, 오른쪽 아래 모서리로 크기를 바꾸고, X 로 닫는다.
// 부모(3D 화면) 밖으로 나가지 않는다. 안에 넣은 위젯(예: 구조 트리)은 이 패널이 소유한다.
class FloatingPanel : public QFrame {
    Q_OBJECT
public:
    FloatingPanel(const QString& title, QWidget* content, QWidget* parent);

    void SetTopInset(int px);            // 위쪽에 이만큼은 비워 둔다 (메뉴줄 아래로 들어가지 않게)
    void PlaceDefault(const QPoint& pos, const QSize& size);   // 처음 놓을 자리와 크기 (사용자가 옮기기 전까지)
    void Clamp();                        // 부모 안으로 밀어 넣는다 (부모 크기가 바뀐 뒤에 부른다)

signals:
    void closeRequested();               // X 를 눌렀다 (숨기는 것은 받는 쪽이 정한다)

protected:
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    bool eventFilter(QObject* obj, QEvent* e) override;   // 오른쪽 아래 크기 조절 손잡이

private:
    QLabel* title_;
    int topInset_;
    bool dragging_;
    QWidget* grip_;
    bool resizing_;
    QPoint resizeStart_;
    QSize sizeStart_;
    QPoint grab_;
};

#endif  // CORECQT_FLOATINGPANEL_H
