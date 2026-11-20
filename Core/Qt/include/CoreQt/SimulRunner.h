#ifndef CORECQT_SIMULRUNNER_H
#define CORECQT_SIMULRUNNER_H

#include <QObject>
#include <QTimer>

#include "CSimul/Engine.h"

class ModelViewer;

// ModelViewer 의 모델들을 sim::Engine 의 오브젝트로 보여 주는 Backend.
//   오브젝트 = 장면에 올린 모델 하나 (이름은 장면 이름). 움직이는 관절이 있으면 kind "robot", 없으면 "model".
//   pose 는 모델의 월드 자세, 관절은 그 모델에 속한 움직이는 관절 (값은 rad 또는 m).
class ViewerBackend : public simul::sim::Backend {
public:
    explicit ViewerBackend(ModelViewer* viewer) : viewer_(viewer) {}
    int ObjectCount() const override;
    bool GetObject(int index, simul::sim::ObjectInfo* out) const override;
    bool SetPose(int index, const dhPose& pose, simul::String* error) override;
    bool SetJoint(int index, const simul::String& joint, double value, simul::String* error) override;

private:
    ModelViewer* viewer_;
};

// 엔진을 실시간으로 돌리는 틱 타이머. 틱 간격은 고정 dt (기본 1/60 s)이고, 화면이나 스크립트가 느려서 밀리면 한 번에 최대 maxCatchUp 틱까지만 따라잡는다.
class SimulRunner : public QObject {
    Q_OBJECT
public:
    SimulRunner(simul::sim::Engine* engine, QObject* parent = nullptr);

    void SetRate(double hz);                 // 초당 틱 수 (1 ~ 1000). 기본 60
    double Rate() const { return 1.0 / dt_; }
    double Dt() const { return dt_; }
    bool Running() const { return engine_->Running(); }

public slots:
    void Start();     // 엔진 Start 후 타이머 시작
    void Stop();
    void StepOnce();  // 한 틱만 (멈춘 상태에서도 된다)

signals:
    void ticked(double time);
    void started();
    void stopped();
    void stepFailed(const QString& error);

private slots:
    void onTimer();

private:
    simul::sim::Engine* engine_;
    QTimer* timer_;
    double dt_;
    qint64 lastMs_;
    double carry_;
};

#endif  // CORECQT_SIMULRUNNER_H
