#include "CoreQt/SpeechBubbles.h"

#include <QFontMetrics>
#include <QLabel>
#include <QSet>

#include "3d/dhPose.h"
#include "CoreQt/ModelViewer.h"
#include "CoreQt/QtString.h"

SpeechBubbles::SpeechBubbles(ModelViewer* viewer, QObject* parent) : QObject(parent), viewer_(viewer) {}

void SpeechBubbles::Update(const simul::Array<simul::sim::Bubble>& bubbles) {
    QSet<QString> shown;
    for (size_t i = 0; i < bubbles.size(); i++) {
        const QString id = simulqt::ToQ(bubbles[i].id);
        int model = -1;
        for (int m = 0; m < viewer_->ModelCount(); m++)
            if (viewer_->ModelName(m) == id) model = m;
        if (model < 0) continue;
        QLabel* label = labels_.value(id, nullptr);
        if (!label) {
            label = new QLabel(viewer_);
            label->setAttribute(Qt::WA_TransparentForMouseEvents);
            label->setWordWrap(true);
            label->setAlignment(Qt::AlignCenter);
            label->setStyleSheet("QLabel { background: rgba(255,255,255,238); color: #222222; border: 2px solid #7a5cff; border-radius: 10px; padding: 5px 9px; font-size: 13px; }");
            labels_.insert(id, label);
        }
        const QString text = simulqt::ToQ(bubbles[i].text);
        if (label->text() != text) {
            label->setText(text);
            const int w = qMin(210, label->fontMetrics().horizontalAdvance(text) + 30);
            label->setFixedWidth(qMax(70, w));
            label->adjustSize();
        }
        const dhPose pose = viewer_->ModelPlacementPose(model);
        QPointF p;
        if (!viewer_->WorldToScreen(dhVector(pose.position.x, pose.position.y, pose.position.z + 1.5f), &p)) continue;   // 머리 위
        label->move(static_cast<int>(p.x()) - label->width() / 2, static_cast<int>(p.y()) - label->height() - 6);
        label->show();
        label->raise();
        shown.insert(id);
    }
    for (QMap<QString, QLabel*>::iterator it = labels_.begin(); it != labels_.end(); ++it)
        if (!shown.contains(it.key())) it.value()->hide();
}
