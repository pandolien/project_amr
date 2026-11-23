#include "simul_gui.h"

#include <QApplication>
#include <QCoreApplication>
#include <QImage>
#include <QString>
#include <QVector>
#include <map>
#include <string>
#include <vector>

#include "3d/dhPose.h"
#include "CoreQt/ModelViewer.h"
#include "CoreQt/QtString.h"

namespace {

struct Window {
    ModelViewer* viewer;
    std::vector<std::string> paths;     // 지금 viewer 에 올라 있는 모델 (simul_core 의 오브젝트 순서)
    std::vector<std::string> names;
    std::vector<double> lastPose;       // 오브젝트마다 7 개: 마지막으로 viewer 에 준 자세
    Window() : viewer(nullptr) {}
};

std::map<simul_sim*, Window*>& Windows() {
    static std::map<simul_sim*, Window*> w;
    return w;
}

std::string g_error;
QApplication* g_app = nullptr;
int g_argc = 1;
char g_arg0[] = "simul";
char* g_argv[] = {g_arg0, nullptr};

bool Fail(const std::string& m) {
    g_error = m;
    return false;
}

bool EnsureApp() {
    if (QCoreApplication::instance()) return true;
    g_app = new QApplication(g_argc, g_argv);   // 프로세스가 끝날 때까지 둔다
    return g_app != nullptr;
}

Window* Find(simul_sim* sim) {
    std::map<simul_sim*, Window*>::iterator it = Windows().find(sim);
    return it == Windows().end() ? nullptr : it->second;
}

dhMat PoseMat(const double p[7]) {
    dhPose pose;
    pose.position = dhVector(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
    pose.orientation.x = static_cast<float>(p[3]);
    pose.orientation.y = static_cast<float>(p[4]);
    pose.orientation.z = static_cast<float>(p[5]);
    pose.orientation.w = static_cast<float>(p[6]);
    return pose.Normalized().ToMat();
}

// simul_core 의 오브젝트 목록이 viewer 에 올린 것과 다르면 viewer 를 비우고 다시 올린다
bool SyncModels(simul_sim* sim, Window* w) {
    const int n = simul_object_count(sim);
    std::vector<std::string> paths, names;
    for (int i = 0; i < n; i++) {
        paths.push_back(simul_object_path(sim, i));
        names.push_back(simul_object_name(sim, i));
    }
    if (paths == w->paths && names == w->names) return true;
    w->viewer->CloseAll();
    w->viewer->LoadOptions().searchDirs.clear();
    for (int i = 0; i < simul_search_dir_count(sim); i++) w->viewer->LoadOptions().searchDirs.push_back(simul::String(simul_search_dir(sim, i)));
    w->lastPose.assign(static_cast<size_t>(n) * 7, 1e300);
    for (int i = 0; i < n; i++) {
        QString err;
        if (!w->viewer->AddFile(QString::fromUtf8(paths[static_cast<size_t>(i)].c_str()), &err))
            return Fail("창에 모델을 올리지 못했습니다: " + paths[static_cast<size_t>(i)] + " (" + err.toUtf8().constData() + ")");
    }
    w->paths = paths;
    w->names = names;
    if (n > 0) w->viewer->FitView();
    return true;
}

bool SameValues(const simul::urdf::JointValues& a, const simul::urdf::JointValues& b) {
    if (a.size() != b.size()) return false;
    for (simul::urdf::JointValues::const_iterator it = a.begin(); it != a.end(); ++it) {
        simul::urdf::JointValues::const_iterator f = b.find(it->first);
        if (f == b.end() || f->second != it->second) return false;
    }
    return true;
}

void SyncState(simul_sim* sim, Window* w) {
    const int n = simul_object_count(sim);
    simul::urdf::JointValues q;
    for (int i = 0; i < n; i++) {
        double pose[7];
        if (!simul_object_pose(sim, i, pose)) continue;
        bool changed = false;
        for (int k = 0; k < 7; k++) changed = changed || pose[k] != w->lastPose[static_cast<size_t>(i) * 7 + static_cast<size_t>(k)];
        if (changed) {
            w->viewer->SetModelPose(i, PoseMat(pose));
            for (int k = 0; k < 7; k++) w->lastPose[static_cast<size_t>(i) * 7 + static_cast<size_t>(k)] = pose[k];
        }
        for (int j = 0; j < simul_joint_count(sim, i); j++) {
            double info[6];
            if (!simul_joint_info(sim, i, j, info)) continue;
            q[simul::String(simul_joint_name(sim, i, j))] = static_cast<float>(info[0]);
        }
    }
    if (n > 0 && !SameValues(q, w->viewer->CurrentJointValues())) w->viewer->SetJointValues(q);
}

}  // namespace

extern "C" {

int simul_gui_api_version(void) { return SIMUL_API_VERSION; }
const char* simul_gui_last_error(void) { return g_error.c_str(); }

int simul_window_open(simul_sim* sim, const char* title, int width, int height) {
    if (!sim) return Fail("simul_sim 이 NULL 입니다") ? 1 : 0;
    if (Find(sim)) return 1;
    if (!EnsureApp()) return Fail("Qt 응용 프로그램을 만들 수 없습니다") ? 1 : 0;
    Window* w = new Window();
    w->viewer = new ModelViewer();
    w->viewer->setWindowTitle(QString::fromUtf8(title && *title ? title : "Project Simul"));
    w->viewer->resize(width > 0 ? width : 960, height > 0 ? height : 640);
    Windows()[sim] = w;
    if (!SyncModels(sim, w)) {
        delete w->viewer;
        delete w;
        Windows().erase(sim);
        return 0;
    }
    SyncState(sim, w);
    w->viewer->show();
    QCoreApplication::processEvents();
    return 1;
}

int simul_window_update(simul_sim* sim) {
    Window* w = Find(sim);
    if (!w) return 0;
    if (!w->viewer->isVisible()) return 0;
    if (!SyncModels(sim, w)) return 1;   // 못 올린 이유는 simul_gui_last_error
    SyncState(sim, w);
    w->viewer->update();
    QCoreApplication::processEvents();
    return w->viewer->isVisible() ? 1 : 0;
}

int simul_window_is_open(simul_sim* sim) {
    Window* w = Find(sim);
    return w && w->viewer->isVisible() ? 1 : 0;
}

void simul_window_close(simul_sim* sim) {
    Window* w = Find(sim);
    if (!w) return;
    Windows().erase(sim);
    delete w->viewer;
    delete w;
    if (QCoreApplication::instance()) QCoreApplication::processEvents();
}

int simul_window_save_image(simul_sim* sim, const char* path) {
    Window* w = Find(sim);
    if (!w || !path) return Fail("열린 창이 없거나 path 가 NULL 입니다") ? 1 : 0;
    SyncModels(sim, w);
    SyncState(sim, w);
    w->viewer->update();
    QCoreApplication::processEvents();
    const QImage img = w->viewer->grabFramebuffer();
    if (img.isNull() || !img.save(QString::fromUtf8(path))) return Fail(std::string("이미지를 저장하지 못했습니다: ") + path) ? 1 : 0;
    return 1;
}

int simul_window_fit_view(simul_sim* sim) {
    Window* w = Find(sim);
    if (!w) return 0;
    w->viewer->FitView();
    return 1;
}

}  // extern "C"
