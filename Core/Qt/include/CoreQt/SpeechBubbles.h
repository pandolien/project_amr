#ifndef CORE_QT_SPEECHBUBBLES_H
#define CORE_QT_SPEECHBUBBLES_H

#include <QMap>
#include <QObject>
#include <QString>

#include "CSimul/Chat.h"

class ModelViewer;
class QLabel;

// 주민 머리 위에 말풍선을 띄운다 (3D 화면 위에 겹치는 QLabel). 주민을 이름이 같은 모델에서 찾고, 카메라 뒤나 화면 밖이면 숨긴다.
// 마우스는 그냥 통과시켜서 풍선 밑의 주민도 클릭할 수 있다.
class SpeechBubbles : public QObject {
public:
    explicit SpeechBubbles(ModelViewer* viewer, QObject* parent = nullptr);
    void Update(const simul::Array<simul::sim::Bubble>& bubbles);   // 매 틱 불러도 된다 (위치만 갱신)

private:
    ModelViewer* viewer_;
    QMap<QString, QLabel*> labels_;
};

#endif  // CORE_QT_SPEECHBUBBLES_H
