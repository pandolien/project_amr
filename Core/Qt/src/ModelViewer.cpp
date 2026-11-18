#include "CKinematics/IK.h"
#include "CoreQt/QtString.h"
#include "CoreQt/ModelViewer.h"

#include <QFileInfo>
#include <QMouseEvent>
#include <QWheelEvent>
#include "CBase/Util.h"
#include "CBase/Math.h"

#include "CCollision/MeshRaycast.h"
#include "CDynamics/FreeFall.h"
#include "CMesh/MeshLoader.h"

namespace {

const char* kVertexShader =
    "#version 120\n"
    "attribute vec3 aPos;\n"
    "attribute vec3 aNormal;\n"
    "attribute vec2 aUV;\n"        // glTF TEXCOORD_0 (텍스처가 있는 모델만 켠다)
    "attribute vec4 aVColor;\n"    // glTF COLOR_0
    "uniform mat4 uMVP;\n"
    "varying vec2 vUV;\n"
    "varying vec4 vVC;\n"
    "uniform mat4 uModelView;\n"
    "varying vec3 vN;\n"
    "varying vec3 vP;\n"
    "void main() {\n"
    "    vUV = aUV;\n"
    "    vVC = aVColor;\n"
    "    vN = mat3(uModelView) * aNormal;\n"        // 법선을 카메라 좌표로
    "    vP = (uModelView * vec4(aPos, 1.0)).xyz;\n"  // 위치를 카메라 좌표로 (시선 방향 V 를 구하려고)
    "    gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "}\n";

// 조명 방식을 uShading 으로 고른다: 0 = Lambert, 1 = Blinn-Phong, 2 = PBR (Cook-Torrance GGX).
// 모든 계산은 카메라 좌표에서 한다 (카메라 위치가 원점, 카메라가 -z 쪽을 본다). 빛 두 개는 카메라에 고정이라 화면을 돌려도 입체감이 같다.
// 약어: N 법선, V 표면에서 카메라로 가는 방향, L 표면에서 빛으로 가는 방향, H = normalize(L + V) 하프벡터.
const char* kFragmentShader =
    "#version 120\n"
    "uniform vec4 uColor;\n"      // 재질 색 (알파 포함)
    "uniform float uLit;\n"       // 0 이면 조명 없이 uColor 그대로 (광선, 좌표축 같은 것)
    "uniform int uShading;\n"     // 0 Lambert, 1 Blinn-Phong, 2 PBR
    "uniform float uMetal;\n"     // 금속성 0..1 (PBR)
    "uniform float uRough;\n"     // 거칠기 0..1 (Blinn-Phong 은 광택도로 바꿔 쓴다)
    "uniform vec3 uUp;\n"         // 월드 +z 를 카메라 좌표로 (주변 환경의 위/아래)
    "uniform sampler2D uTex;\n"
    "uniform float uUseTex;\n"   // 1 이면 기본 색 텍스처를 곱한다
    "uniform float uUseVC;\n"    // 1 이면 정점 색을 곱한다
    "varying vec3 vN;\n"
    "varying vec3 vP;\n"
    "varying vec2 vUV;\n"
    "varying vec4 vVC;\n"
    "const float PI = 3.14159265;\n"
    "\n"
    // ---- PBR 의 세 항 ----
    // D: 표면의 미세한 면들이 H 쪽을 향한 비율 (GGX). 거칠수록 넓게 퍼지고, 매끈할수록 좁고 밝은 점이 된다.
    "float D_GGX(float NdH, float a2) {\n"
    "    float d = NdH * NdH * (a2 - 1.0) + 1.0;\n"
    "    return a2 / (PI * d * d);\n"
    "}\n"
    // G: 미세한 면끼리 서로 빛/시선을 가리는 정도 (Schlick-GGX 를 빛 쪽과 시선 쪽에 한 번씩 곱한 Smith 방식)
    "float G_Smith(float NdL, float NdV, float rough) {\n"
    "    float k = (rough + 1.0) * (rough + 1.0) / 8.0;\n"
    "    return (NdL / (NdL * (1.0 - k) + k)) * (NdV / (NdV * (1.0 - k) + k));\n"
    "}\n"
    // F: 보는 각도가 비스듬할수록 반사가 강해지는 Fresnel (Schlick 근사). F0 은 정면에서 본 반사율.
    "vec3 F_Schlick(float cosT, vec3 F0) {\n"
    "    return F0 + (1.0 - F0) * pow(1.0 - cosT, 5.0);\n"
    "}\n"
    // 주변 환경: 아래는 어둡고 위는 밝은 하늘. 월드 위쪽(uUp)을 기준으로 방향마다 색을 돌려준다.
    "vec3 envColor(vec3 dir) {\n"
    "    float t = dot(dir, uUp) * 0.5 + 0.5;\n"
    "    vec3 ground = vec3(0.10, 0.10, 0.11);\n"
    "    vec3 horizon = vec3(0.55, 0.57, 0.60);\n"
    "    vec3 sky = vec3(0.80, 0.88, 1.00);\n"
    "    return t < 0.5 ? mix(ground, horizon, t * 2.0) : mix(horizon, sky, (t - 0.5) * 2.0);\n"
    "}\n"
    // 빛 하나가 만드는 색: (난반사 + 정반사) x 빛 세기 x cos(N, L)
    "vec3 pbrLight(vec3 L, vec3 radiance, vec3 N, vec3 V, vec3 albedo, vec3 F0, float rough, float metal) {\n"
    "    vec3 H = normalize(L + V);\n"
    "    float NdL = max(dot(N, L), 0.0);\n"
    "    float NdV = max(dot(N, V), 1e-3);\n"
    "    float NdH = max(dot(N, H), 0.0);\n"
    "    float VdH = max(dot(V, H), 0.0);\n"
    "    float a = rough * rough;\n"
    "    vec3 F = F_Schlick(VdH, F0);\n"
    "    vec3 spec = D_GGX(NdH, a * a) * G_Smith(NdL, NdV, rough) * F / (4.0 * NdL * NdV + 1e-4);\n"
    "    vec3 diff = (1.0 - F) * (1.0 - metal) * albedo / PI;\n"   // 반사되고 남은 빛만 난반사, 금속은 난반사가 없다
    "    return (diff + spec) * radiance * NdL;\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    if (uLit < 0.5) { gl_FragColor = uColor; return; }\n"
    "    vec4 base = uColor;\n"                                     // 재질 색 x 정점 색 x 텍스처 (glTF 의 baseColor 규칙)
    "    if (uUseVC > 0.5) base *= vVC;\n"
    "    if (uUseTex > 0.5) base *= texture2D(uTex, vUV);\n"
    "    vec3 N = normalize(vN);\n"
    "    if (!gl_FrontFacing) N = -N;\n"                     // 양면 렌더링: 뒷면은 법선을 뒤집는다
    "    vec3 V = normalize(-vP);\n"
    "    vec3 col;\n"
    "    if (uShading == 0) {\n"
    // Lambert: 주변광 + cos(N, 빛) 두 개 (카메라 방향 빛, 보조광)
    "        float l = 0.30 + 0.55 * max(dot(N, vec3(0.0, 0.0, 1.0)), 0.0)\n"
    "                       + 0.30 * max(dot(N, normalize(vec3(-0.5, 0.7, 0.5))), 0.0);\n"
    "        col = base.rgb * min(l, 1.2);\n"
    "    } else if (uShading == 1) {\n"
    // Blinn-Phong: Lambert + 하이라이트. 거칠기를 광택도로 바꿔 쓴다 (매끈할수록 작고 밝은 점)
    "        vec3 L1 = normalize(vec3(-0.3, 0.6, 0.75));\n"
    "        vec3 L2 = normalize(vec3(0.6, -0.2, 0.5));\n"
    "        float shin = clamp(2.0 / (uRough * uRough + 0.001) - 1.0, 4.0, 400.0);\n"
    "        float d = 0.25 + 0.65 * max(dot(N, L1), 0.0) + 0.25 * max(dot(N, L2), 0.0);\n"
    "        float s = pow(max(dot(N, normalize(L1 + V)), 0.0), shin) * 0.45 * step(0.0, dot(N, L1));\n"
    "        col = base.rgb * min(d, 1.2) + vec3(s);\n"
    "    } else {\n"
    // PBR. 재질 색은 sRGB 이므로 먼저 선형으로 바꾸고, 계산한 뒤 톤매핑과 감마로 화면 값으로 돌려놓는다.
    "        vec3 albedo = pow(base.rgb, vec3(2.2));\n"
    "        float rough = clamp(uRough, 0.05, 1.0);\n"
    "        float metal = clamp(uMetal, 0.0, 1.0);\n"
    "        vec3 F0 = mix(vec3(0.04), albedo, metal);\n"      // 비금속 반사율은 4%, 금속은 자기 색이 곧 반사색
    "        float NdV = max(dot(N, V), 1e-3);\n"
    "        vec3 c = pbrLight(normalize(vec3(-0.3, 0.6, 0.75)), vec3(2.4), N, V, albedo, F0, rough, metal);\n"   // 주광
    "        c += pbrLight(normalize(vec3(0.6, -0.2, 0.5)), vec3(0.9, 0.95, 1.1), N, V, albedo, F0, rough, metal);\n"  // 보조광
    // 주변광: 난반사는 법선 쪽 환경색, 정반사는 반사 방향의 환경색 (거칠수록 흐려져 법선 쪽 색과 섞는다)
    "        vec3 R = reflect(-V, N);\n"
    "        vec3 envSpec = mix(envColor(R), envColor(N), rough);\n"
    "        vec3 Fe = F0 + (max(vec3(1.0 - rough), F0) - F0) * pow(1.0 - NdV, 5.0);\n"
    "        c += (1.0 - metal) * albedo * (1.0 - Fe) * envColor(N) * 0.30 + Fe * envSpec * 0.55;\n"
    "        c = c * (1.0 + c / 16.0) / (1.0 + c);\n"        // 밝은 부분이 하얗게 날아가지 않게 누르는 톤매핑 (Reinhard)
    "        col = pow(c, vec3(1.0 / 2.2));\n"
    "    }\n"
    "    gl_FragColor = vec4(col, base.a);\n"
    "}\n";

QMatrix4x4 ToQt(const dhMat& m) {
    return QMatrix4x4(m.v).transposed();  // QMatrix4x4(float*) 는 행 우선, dhMat 은 열 우선
}

void Transform(const dhMat& m, const float* p, float* o) {
    o[0] = m.v[0] * p[0] + m.v[4] * p[1] + m.v[8] * p[2] + m.v[12];
    o[1] = m.v[1] * p[0] + m.v[5] * p[1] + m.v[9] * p[2] + m.v[13];
    o[2] = m.v[2] * p[0] + m.v[6] * p[1] + m.v[10] * p[2] + m.v[14];
}

const float kDefaultColor[4] = {0.72f, 0.74f, 0.78f, 1.0f};
const float kDefaultMetal = 0.0f;   // glTF 재질이 없는 모델(URDF 기본 도형, STL 등)의 기본 PBR 값
const float kDefaultRough = 0.45f;

}  // namespace

ModelViewer::ModelViewer(QWidget* parent)
    : QOpenGLWidget(parent), buffersDirty_(false), hasRobot_(false), triangleCount_(0), radius_(1.0f),
      dist_(3.0f), yaw_(-60.0f), pitch_(25.0f), showBoxes_(false), showObbs_(false), showGrid_(true), gridCell_(1.0f) {
    setMouseTracking(true);  // 버튼을 누르지 않아도 링 위에 있으면 밝게 보인다
    selectedLink_ = -1;
    selectedModel_ = false;
    showFrame_ = true;
    gizmoDirty_ = false;
    dragged_ = false;
    lidarOn_ = false;
    lidarDraw_ = true;
    shading_ = 2;
    lidarLink_ = -1;
    lidarHeading_ = 0.0f;
    dragAxis_ = -1;
    dragIk_ = false;
    dragModel_ = -1;
    dragRing_ = -1;
    dragRingDir_ = QPointF(1, 0);
    dragRingPx_ = 100;
    ringLocal_ = false;
    hoverRing_ = -1;
    mouseEdit_ = true;
    modelMinZValid_ = false;
    dragU0_ = 0;
    fallEnabled_ = true;
    gravityZ_ = -9.80665;
    fallVel_ = 0;
    minZ_ = 0;
    minZValid_ = false; modelMinZValid_ = false;
    fallTimer_ = new QTimer(this);
    fallTimer_->setInterval(16);
    fallClock_.start();
    QObject::connect(fallTimer_, &QTimer::timeout, this, [this]() {
        const double dt = static_cast<double>(fallClock_.restart()) / 1000.0;
        StepFall(dt);
    });
    fallTimer_->start();
    hasScene_ = false;
    extent_[0] = extent_[1] = extent_[2] = 0.0f;
    setMinimumSize(320, 240);
    setFocusPolicy(Qt::StrongFocus);
    rebuildGrid();
}

ModelViewer::~ModelViewer() {
    makeCurrent();
    releaseBuffers();
    releaseItems(gizmo_);
    prog_.reset();
    doneCurrent();
}

void ModelViewer::releaseItems(ItemList& list) {
    for (size_t i = 0; i < list.size(); i++) {
        if (list[i]->vbo.isCreated()) list[i]->vbo.destroy();
        if (list[i]->ibo.isCreated()) list[i]->ibo.destroy();
        if (list[i]->evbo.isCreated()) list[i]->evbo.destroy();
        list[i]->tex.reset();
    }
}

void ModelViewer::releaseBuffers() { releaseItems(items_); }

void ModelViewer::Clear() {
    // GL 자원은 컨텍스트가 있을 때만 지울 수 있다. 없으면 다음 paintGL 에서 정리된다.
    if (isValid()) {
        makeCurrent();
        releaseBuffers();
        doneCurrent();
    }
    items_.clear();
    hasRobot_ = false;
    robot_ = simul::urdf::Robot();
    linkT_.clear();
    q_.clear();
    colModel_ = simul::RobotCollisionModel();
    report_ = simul::SelfCollisionReport();
    selectedLink_ = -1;
    selectedModel_ = false;
    objectPos_ = dhVector(0, 0, 0);
    fallVel_ = 0;
    minZValid_ = false; modelMinZValid_ = false;
    scene_ = simul::Scene();
    hasScene_ = false;
    warnings_.clear();
    triangleCount_ = 0;
    buffersDirty_ = false;
    updateBounds();
    update();
}

void ModelViewer::addScene(const simul::Scene& scene, int link, const dhMat& local, const float* colorOverride, ItemList* dst) {
    ItemList& out = dst ? *dst : items_;
    const simul::Array<dhMat> world = scene.WorldTransforms();
    for (size_t n = 0; n < scene.nodes.size(); n++) {
        const simul::SceneNode& node = scene.nodes[n];
        if (node.mesh < 0 || static_cast<size_t>(node.mesh) >= scene.meshes.size()) continue;
        const simul::Mesh& mesh = scene.meshes[static_cast<size_t>(node.mesh)];
        for (size_t p = 0; p < mesh.primitives.size(); p++) {
            const simul::Primitive& prim = mesh.primitives[p];
            if (prim.indices.empty() || prim.normals.size() != prim.positions.size()) continue;
            simul::UniquePtr<Item> it(new Item);
            const size_t nv = prim.VertexCount();
            it->data.resize(nv * 6);
            for (size_t v = 0; v < nv; v++) {
                for (size_t k = 0; k < 3; k++) {
                    it->data[v * 6 + k] = prim.positions[v * 3 + k];
                    it->data[v * 6 + 3 + k] = prim.normals[v * 3 + k];
                }
            }
            it->indices = prim.indices;
            const bool hasUv = prim.uvs.size() == nv * 2, hasVc = prim.colors.size() == nv * 4;
            const simul::Material* mat = (!colorOverride && prim.material >= 0 && static_cast<size_t>(prim.material) < scene.materials.size()) ? &scene.materials[static_cast<size_t>(prim.material)] : nullptr;
            if (mat) {   // 텍스처 이미지: 파일 안(GLB/data URI)의 바이트, 아니면 파일 경로
                if (!mat->baseColorImage.empty()) it->image.loadFromData(mat->baseColorImage.data(), static_cast<uint>(mat->baseColorImage.size()));
                else if (!mat->baseColorTexture.empty()) it->image.load(simulqt::ToQ(mat->baseColorTexture));
                if (it->image.isNull() && (!mat->baseColorImage.empty() || !mat->baseColorTexture.empty()))
                    if (&out == &items_) warnings_ << QString::fromUtf8("텍스처 이미지를 읽을 수 없습니다: ") + (mat->baseColorTexture.empty() ? QString::fromUtf8("(파일 안의 이미지)") : simulqt::ToQ(mat->baseColorTexture));
                if (!hasUv) it->image = QImage();   // uv 가 없으면 입힐 수 없다
            }
            it->hasVertexColor = hasVc && !colorOverride;
            if (!it->image.isNull() || it->hasVertexColor) {
                it->extra.assign(nv * 6, 1.0f);
                for (size_t v = 0; v < nv; v++) {
                    it->extra[v * 6] = hasUv ? prim.uvs[v * 2] : 0.0f;
                    it->extra[v * 6 + 1] = hasUv ? prim.uvs[v * 2 + 1] : 0.0f;
                    for (size_t k = 0; k < 4; k++) it->extra[v * 6 + 2 + k] = it->hasVertexColor ? prim.colors[v * 4 + k] : 1.0f;
                }
            }
            it->base = local * world[n];
            it->link = link;
            it->metallic = kDefaultMetal;
            it->roughness = kDefaultRough;
            if (colorOverride) {
                for (int k = 0; k < 4; k++) it->color[k] = colorOverride[k];
            } else if (prim.material >= 0 && static_cast<size_t>(prim.material) < scene.materials.size()) {
                const simul::Material& m = scene.materials[static_cast<size_t>(prim.material)];
                it->color[0] = m.baseColor.r; it->color[1] = m.baseColor.g; it->color[2] = m.baseColor.b; it->color[3] = m.baseColor.a;
                it->metallic = m.metallic;   // glTF 파일의 PBR 재질 값을 그대로 쓴다
                it->roughness = m.roughness;
            } else {
                for (int k = 0; k < 4; k++) it->color[k] = kDefaultColor[k];
            }
            if (&out == &items_) triangleCount_ += prim.TriangleCount();
            out.push_back(simul::Move(it));
        }
    }
    if (&out == &items_)
        for (size_t i = 0; i < scene.warnings.size(); i++) warnings_ << simulqt::ToQ(scene.warnings[i]);
}

void ModelViewer::SetScene(const simul::Scene& scene) {
    Clear();
    scene_ = scene;
    hasScene_ = true;
    addScene(scene, -1, dhMat(), nullptr);
    buffersDirty_ = true;
    updateBounds();
    FitView();
    emit modelChanged();
    emit robotChanged();   // 정적 모델도 구성이 바뀐 것이다
}

void ModelViewer::SetRobot(const simul::urdf::Robot& robot, const simul::urdf::RobotMeshes& meshes) {
    Clear();
    robot_ = robot;
    hasRobot_ = true;
    linkT_ = robot_.LinkTransforms(simul::urdf::JointValues());
    for (size_t i = 0; i < meshes.visuals.size(); i++) {
        const simul::urdf::LinkVisual& lv = meshes.visuals[i];
        float color[4];
        if (lv.hasColor) { color[0] = lv.color.r; color[1] = lv.color.g; color[2] = lv.color.b; color[3] = lv.color.a; }
        addScene(lv.scene, lv.link, lv.local, lv.hasColor ? color : nullptr);
    }
    for (size_t i = 0; i < meshes.warnings.size(); i++) warnings_ << simulqt::ToQ(meshes.warnings[i]);
    for (size_t i = 0; i < robot_.warnings.size(); i++) warnings_ << simulqt::ToQ(robot_.warnings[i]);
    simul::CollisionModelOptions copt;
    copt.load = options_;
    copt.fallbackToVisual = true;  // <collision> 이 없는 URDF 도 박스를 볼 수 있게 한다
    simul::BuildCollisionModel(robot_, copt, &colModel_);
    for (size_t i = 0; i < colModel_.warnings.size(); i++) warnings_ << simulqt::ToQ(colModel_.warnings[i]);
    recomputeCollisions();
    buffersDirty_ = true;
    updateBounds();
    FitView();
    emit modelChanged();
    emit robotChanged();
}

void ModelViewer::SetJointValues(const simul::urdf::JointValues& q) {
    if (!hasRobot_) return;
    q_ = q;
    linkT_ = robot_.LinkTransforms(q);
    minZValid_ = false; modelMinZValid_ = false;
    recomputeCollisions();
    update();
    emit jointValuesChanged();
}

void ModelViewer::recomputeCollisions() {
    const simul::Array<simul::CollisionPair> before = report_.collisions;
    const simul::Array<simul::CollisionPair> beforeCand = report_.aabbCandidates;
    report_ = simul::CheckSelfCollisions(robot_, colModel_, q_);
    if (!(before == report_.collisions) || !(beforeCand == report_.aabbCandidates)) emit collisionsChanged();
}

void ModelViewer::SetShowCollisionBoxes(bool show) {
    showBoxes_ = show;
    update();
}

void ModelViewer::SetShowObbs(bool show) {
    showObbs_ = show;
    update();
}

void ModelViewer::SetSelectedLink(int link) {
    selectedLink_ = link;
    if (link >= 0) selectedModel_ = false;
    update();
    emit selectionChanged(link);
}

void ModelViewer::SelectModel(bool selected) {
    selectedModel_ = selected && hasScene_;
    if (selectedModel_) selectedLink_ = -1;
    update();
}

void ModelViewer::SetShowSelectionFrame(bool show) {
    showFrame_ = show;
    update();
}

dhMat ModelViewer::itemWorld(const Item& it) const {
    dhMat m = it.base;
    if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
    return dhMat::Trans(objectPos_.x, objectPos_.y, objectPos_.z) * m;
}

bool ModelViewer::SelectedFrame(dhMat* out) const {
    const dhMat t = dhMat::Trans(objectPos_.x, objectPos_.y, objectPos_.z);
    if (selectedLink_ >= 0 && static_cast<size_t>(selectedLink_) < linkT_.size()) {
        if (out) *out = t * linkT_[static_cast<size_t>(selectedLink_)];
        return true;
    }
    if (selectedModel_) {
        if (out) *out = t;
        return true;
    }
    return false;
}

bool ModelViewer::CanMoveSelection() const {
    if (world_.Composite()) return selectedLink_ >= 0 && ModelOfRoot(selectedLink_) >= 0;  // 둘 이상이면 각 모델의 루트 링크가 이동 손잡이
    if (selectedModel_) return true;
    return hasRobot_ && selectedLink_ >= 0 && selectedLink_ == robot_.root;
}

bool ModelViewer::CanIkSelection() const {
    if (!hasRobot_ || selectedLink_ < 0 || static_cast<size_t>(selectedLink_) >= robot_.links.size() || selectedLink_ == robot_.root) return false;
    int cur = selectedLink_;
    for (size_t guard = 0; guard <= robot_.joints.size(); guard++) {
        int parentJoint = -1;
        for (size_t j = 0; j < robot_.joints.size(); j++)
            if (robot_.FindLink(robot_.joints[j].child) == cur) {
                parentJoint = static_cast<int>(j);
                break;
            }
        if (parentJoint < 0) return false;
        if (robot_.joints[static_cast<size_t>(parentJoint)].IsMovable()) return true;
        cur = robot_.FindLink(robot_.joints[static_cast<size_t>(parentJoint)].parent);
        if (cur < 0) return false;
    }
    return false;
}

void ModelViewer::SetObjectPosition(float x, float y, float z) {
    objectPos_ = dhVector(x, y, z);
    fallVel_ = 0;
    update();
    emit objectMoved();
}

dhVector ModelViewer::ObjectPosition() const { return objectPos_; }

float ModelViewer::objectMinZ() const {
    if (minZValid_) return minZ_;
    float lo = simul::kFloatMax;
    for (size_t i = 0; i < items_.size(); i++) {
        const Item& it = *items_[i];
        dhMat m = it.base;  // 이동량은 빼고 계산한다
        if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
        for (size_t v = 0; v + 5 < it.data.size(); v += 6) {
            float p[3];
            Transform(m, &it.data[v], p);
            lo = simul::Min(lo, p[2]);
        }
    }
    minZ_ = lo == simul::kFloatMax ? 0.0f : lo;
    minZValid_ = true;
    return minZ_;
}

float ModelViewer::ObjectBottom() const { return objectMinZ() + objectPos_.z; }

void ModelViewer::SetFallEnabled(bool enabled) {
    fallEnabled_ = enabled;
    fallVel_ = 0;
}

void ModelViewer::SetGravity(double, double, double gz) { gravityZ_ = gz; }

bool ModelViewer::StepFall(double dt) {
    // world 아래에 모델을 올려 둔 장면(둘 이상이거나 회전한 모델)은 모델마다 자세를 attach 관절이 가지므로 모델 하나하나가 떨어진다
    if (world_.Composite()) return stepFallModels(dt);
    if (!fallEnabled_ || items_.empty() || dragAxis_ >= 0) {  // 끄는 동안에는 손이 붙잡고 있다
        fallVel_ = 0;
        return false;
    }
    simul::dynamics::FallBody b;
    b.height = static_cast<double>(ObjectBottom());
    b.velocity = fallVel_;
    if (b.height >= 0.0 && b.height < 1e-5 && b.velocity <= 0.0) {  // 바닥에 놓여 있다 (부동소수점 오차 안쪽)
        fallVel_ = 0;
        return false;
    }
    const double before = b.height;
    const bool moving = b.Step(dt, gravityZ_);
    fallVel_ = b.velocity;
    if (b.height != before) {
        objectPos_.z += static_cast<float>(b.height - before);
        update();
        emit objectMoved();
    }
    return moving;
}

// 모델에 속한 모든 꼭짓점의 월드 z 가 가장 낮은 값 (자세와 관절 값이 반영된다)
float ModelViewer::modelBottom(int model) const {
    if (!modelMinZValid_) {
        modelMinZ_.clear();
        for (size_t k = 0; k < world_.size(); k++) modelMinZ_.push_back(simul::kFloatMax);
        for (size_t i = 0; i < items_.size(); i++) {
            const Item& it = *items_[i];
            const int k = world_.EntryOfLink(robot_, it.link);
            if (k < 0) continue;
            dhMat m = it.base;
            if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
            float lo = modelMinZ_[static_cast<size_t>(k)];
            for (size_t v = 0; v + 5 < it.data.size(); v += 6) {
                float p[3];
                Transform(m, &it.data[v], p);
                lo = simul::Min(lo, p[2]);
            }
            modelMinZ_[static_cast<size_t>(k)] = lo;
        }
        modelMinZValid_ = true;
    }
    if (model < 0 || static_cast<size_t>(model) >= modelMinZ_.size() || modelMinZ_[static_cast<size_t>(model)] == simul::kFloatMax) return 0.0f;
    return modelMinZ_[static_cast<size_t>(model)];
}

bool ModelViewer::stepFallModels(double dt) {
    const size_t n = world_.size();
    while (modelFallVel_.size() < n) modelFallVel_.push_back(0.0);
    if (!fallEnabled_ || items_.empty()) {
        for (size_t k = 0; k < modelFallVel_.size(); k++) modelFallVel_[k] = 0.0;
        return false;
    }
    bool any = false;
    for (size_t k = 0; k < n; k++) {
        if (dragModel_ == static_cast<int>(k) && (dragAxis_ >= 0 || dragRing_ >= 0)) {  // 끄는 동안에는 손이 붙잡고 있다
            modelFallVel_[k] = 0.0;
            continue;
        }
        simul::dynamics::FallBody b;
        b.height = static_cast<double>(modelBottom(static_cast<int>(k)));
        b.velocity = modelFallVel_[k];
        if (b.height >= 0.0 && b.height < 1e-5 && b.velocity <= 0.0) {  // 바닥에 놓여 있다
            modelFallVel_[k] = 0.0;
            continue;
        }
        const double before = b.height;
        const bool moving = b.Step(dt, gravityZ_);
        modelFallVel_[k] = b.velocity;
        if (b.height != before) {
            dhMat pose = ModelPose(static_cast<int>(k));
            pose.v[14] += static_cast<float>(b.height - before);
            SetModelPose(static_cast<int>(k), pose);
        }
        any = any || moving;
    }
    return any;
}

double ModelViewer::SimulateFall(double maxSeconds) {
    double t = 0.0;
    const double dt = 1.0 / 240.0;
    while (t < maxSeconds && StepFall(dt)) t += dt;
    return t;
}

bool ModelViewer::LoadGizmo(const QString& objPath, QString* error) {
    simul::Scene scene;
    simul::String err;
    if (!simul::LoadObj(simulqt::FromQ(objPath), &scene, &err)) {
        if (error) *error = simulqt::ToQ(err);
        return false;
    }
    if (isValid()) {
        makeCurrent();
        releaseItems(gizmo_);
        doneCurrent();
    }
    gizmo_.clear();
    addScene(scene, -1, dhMat(), nullptr, &gizmo_);
    gizmoDirty_ = true;
    if (gizmo_.empty()) {
        if (error) *error = QStringLiteral("좌표계 OBJ 에 그릴 면이 없습니다: ") + objPath;
        return false;
    }
    update();
    return true;
}

simul::Array<int> ModelViewer::CollidingLinks() const {
    simul::Array<int> out;
    for (size_t i = 0; i < report_.collisions.size(); i++) {
        out.push_back(report_.collisions[i].a);
        out.push_back(report_.collisions[i].b);
    }
    simul::Sort(out.begin(), out.end());
    out.erase(simul::Unique(out.begin(), out.end()), out.end());
    return out;
}

QStringList ModelViewer::CollisionDescriptions() const {
    QStringList out;
    for (size_t i = 0; i < report_.collisions.size(); i++) {
        const simul::CollisionPair& p = report_.collisions[i];
        out << simulqt::ToQ(robot_.links[static_cast<size_t>(p.a)].name) + QString::fromUtf8(" <-> ") +
                   simulqt::ToQ(robot_.links[static_cast<size_t>(p.b)].name);
    }
    return out;
}

bool ModelViewer::AddFile(const QString& path, QString* error) {
    simul::String err;
    const bool hadOne = world_.size() == 1 && !world_.Composite();
    if (!world_.Add(simulqt::FromQLocal8Bit(path), &err)) {
        if (error) *error = simulqt::ToQ(err);
        return false;
    }
    if (hadOne) {  // 하나일 때 오브젝트 위치로 옮겨 둔 것을 첫 모델의 배치로 넘긴다 (둘 이상이면 배치는 attach 관절이 가진다)
        world_.SetPlacement(0, dhMat::Trans(objectPos_.x, objectPos_.y, objectPos_.z));
        objectPos_ = dhVector(0, 0, 0);
    }
    if (!rebuildWorld(error)) {
        world_.Remove(world_.size() - 1);  // 합치지 못하면 추가를 취소한다
        if (hadOne) {
            const dhMat p = world_.entry(0).placement;
            world_.SetPlacement(0, dhMat());
            rebuildWorld(nullptr);
            objectPos_ = dhVector(p.v[12], p.v[13], p.v[14]);
        }
        return false;
    }
    return true;
}

bool ModelViewer::LoadFile(const QString& path, QString* error) {
    const simul::urdf::World keep = world_;
    const dhVector keepPos = objectPos_;
    world_.Clear();
    objectPos_ = dhVector(0, 0, 0);
    if (AddFile(path, error)) return true;
    world_ = keep;  // 열지 못하면 이전 장면을 그대로 둔다
    objectPos_ = keepPos;
    return false;
}

void ModelViewer::CloseAll() {
    world_.Clear();
    Clear();
    emit modelChanged();
    emit robotChanged();
}

void ModelViewer::RemoveModel(int index) {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return;
    const bool wasComposite = world_.Composite();
    // 둘 중 하나를 지워 하나만 남으면 그 모델의 자세를 가져간다 (회전이 있으면 world 아래에 그대로 둔다)
    const dhMat last = world_.size() == 2 ? world_.entry(static_cast<size_t>(1 - index)).placement : dhMat();
    world_.Remove(static_cast<size_t>(index));
    if (world_.size() == 0) {
        Clear();
        emit modelChanged();
        emit robotChanged();
        return;
    }
    bool rotated = false;
    for (int k = 0; k < 3; k++)
        for (int r = 0; r < 3; r++)
            if (simul::Abs(last.v[k * 4 + r] - (k == r ? 1.0f : 0.0f)) > 1e-6f) rotated = true;
    const bool stay = world_.Composite() || (wasComposite && world_.size() == 1 && rotated);
    const bool toObjectPos = world_.size() == 1 && wasComposite && !stay;
    if (toObjectPos) world_.SetPlacement(0, dhMat());
    if (stay && world_.size() == 1) world_.SetForceComposite(true);
    rebuildWorld(nullptr);
    if (toObjectPos) objectPos_ = dhVector(last.v[12], last.v[13], last.v[14]);  // 남은 하나는 오브젝트 위치로 (SetRobot 이 지운 뒤에 넣는다)
    update();
}

QString ModelViewer::ModelName(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return QString();
    return simulqt::ToQ(world_.entry(static_cast<size_t>(index)).name);
}

bool ModelViewer::ensureComposite() {
    if (world_.Composite() || world_.size() != 1) return world_.Composite();
    world_.SetPlacement(0, dhMat::Trans(objectPos_.x, objectPos_.y, objectPos_.z));
    objectPos_ = dhVector(0, 0, 0);
    world_.SetForceComposite(true);
    if (!rebuildWorld(nullptr, true)) {
        world_.SetForceComposite(false);
        const dhMat p = world_.entry(0).placement;
        world_.SetPlacement(0, dhMat());
        rebuildWorld(nullptr, true);
        objectPos_ = dhVector(p.v[12], p.v[13], p.v[14]);  // SetRobot 이 위치를 지우므로 다시 넣는다
        return false;
    }
    emit structureChanged();
    return true;
}

bool ModelViewer::SelectionScreenPos(QPoint* out) const {
    dhMat f;
    if (!SelectedFrame(&f) || width() <= 0 || height() <= 0) return false;
    const QVector4D c = projMatrix() * viewMatrix() * QVector4D(ToQt(f).column(3).toVector3D(), 1.0f);
    if (c.w() <= 0.0f) return false;  // 카메라 뒤
    if (out) *out = QPoint(static_cast<int>((c.x() / c.w() * 0.5f + 0.5f) * static_cast<float>(width())), static_cast<int>((1.0f - (c.y() / c.w() * 0.5f + 0.5f)) * static_cast<float>(height())));
    return true;
}

bool ModelViewer::CanRotateSelection() const {
    return showFrame_ && hasRobot_ && selectedLink_ >= 0 && ModelOfRoot(selectedLink_) >= 0;
}

void ModelViewer::SetModelPose(int index, const dhMat& pose) {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return;
    if (!world_.Composite() && !ensureComposite()) return;
    world_.SetPlacement(static_cast<size_t>(index), pose);
    const simul::String& root = world_.entry(static_cast<size_t>(index)).rootName;
    for (size_t j = 0; j < robot_.joints.size(); j++) {
        if (robot_.joints[j].child == root && robot_.joints[j].parent == "world") robot_.joints[j].origin = pose;
    }
    linkT_ = robot_.LinkTransforms(q_);
    minZValid_ = false; modelMinZValid_ = false;
    recomputeCollisions();
    updateBounds();
    update();
    emit modelChanged();
    emit objectMoved();
}

dhMat ModelViewer::ModelPose(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return dhMat();
    if (!world_.Composite()) return dhMat::Trans(objectPos_.x, objectPos_.y, objectPos_.z);
    return world_.entry(static_cast<size_t>(index)).placement;
}

void ModelViewer::SetModelPlacement6(int index, const float v[6]) {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return;
    const bool rotates = v[3] != 0.0f || v[4] != 0.0f || v[5] != 0.0f;
    if (!world_.Composite() && !rotates) {
        SetObjectPosition(v[0], v[1], v[2]);
        return;
    }
    const dhMat m = dhMat::Trans(v[0], v[1], v[2]) * dhMat::RotZ(static_cast<float>(dhRad(v[5]))) *
                    dhMat::RotY(static_cast<float>(dhRad(v[4]))) * dhMat::RotX(static_cast<float>(dhRad(v[3])));
    SetModelPose(index, m);
}

void ModelViewer::SetModelPlacement(int index, float x, float y, float z, float yawDeg) {
    const float v[6] = {x, y, z, 0.0f, 0.0f, yawDeg};
    SetModelPlacement6(index, v);
}

void ModelViewer::ModelPlacement6(int index, float v[6]) const {
    for (int i = 0; i < 6; i++) v[i] = 0;
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return;
    const dhMat m = ModelPose(index);
    v[0] = m.v[12]; v[1] = m.v[13]; v[2] = m.v[14];
    v[3] = static_cast<float>(dhDeg(simul::Atan2(m.v[6], m.v[10])));                          // roll
    v[4] = static_cast<float>(dhDeg(asinf(simul::Max(-1.0f, simul::Min(1.0f, -m.v[2])))));    // pitch
    v[5] = static_cast<float>(dhDeg(simul::Atan2(m.v[1], m.v[0])));                           // yaw
}

void ModelViewer::ModelPlacement(int index, float* x, float* y, float* z, float* yawDeg) const {
    float v[6];
    ModelPlacement6(index, v);
    if (x) *x = v[0];
    if (y) *y = v[1];
    if (z) *z = v[2];
    if (yawDeg) *yawDeg = v[5];
}

// 올려 둔 모델을 하나의 Robot 으로 합쳐 화면에 새로 올린다. 이름이 남아 있는 관절 값은 유지한다.
bool ModelViewer::rebuildWorld(QString* error, bool keepView) {
    simul::urdf::Robot built;
    simul::String err;
    if (!world_.Build(options_, &built, &err)) {
        if (error) *error = simulqt::ToQ(err);
        return false;
    }
    modelFallVel_.clear();
    const simul::urdf::JointValues oldQ = q_;
    const QString selName = (selectedLink_ >= 0 && static_cast<size_t>(selectedLink_) < robot_.links.size()) ? simulqt::ToQ(robot_.links[static_cast<size_t>(selectedLink_)].name) : QString();
    const float vYaw = yaw_, vPitch = pitch_, vDist = dist_;
    const QVector3D vTarget = target_;
    simul::urdf::RobotMeshes meshes;
    simul::urdf::LoadRobotMeshes(built, options_, &meshes);  // 일부 실패는 경고로 남는다
    SetRobot(built, meshes);
    if (keepView) {  // 이미 보고 있던 장면을 다시 합치는 것이면 카메라를 그대로 둔다
        yaw_ = vYaw; pitch_ = vPitch; dist_ = vDist; target_ = vTarget;
    }
    simul::urdf::JointValues q;
    for (simul::urdf::JointValues::const_iterator it = oldQ.begin(); it != oldQ.end(); ++it)
        if (built.FindJoint(it->first) >= 0) q[it->first] = it->second;
    if (!q.empty()) SetJointValues(q);
    if (!selName.isEmpty()) {  // 합치기 전에 골라 둔 링크는 이름으로 다시 찾아 선택을 유지한다
        const int l = robot_.FindLink(simulqt::FromQ(selName));
        if (l >= 0) selectedLink_ = l;
    }
    emit selectionChanged(selectedLink_);
    return true;
}

void ModelViewer::updateBounds() {
    float mn[3] = {simul::kFloatMax, simul::kFloatMax, simul::kFloatMax};
    float mx[3] = {-mn[0], -mn[1], -mn[2]};
    bool any = false;
    for (size_t i = 0; i < items_.size(); i++) {
        const Item& it = *items_[i];
        dhMat m = it.base;
        if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
        for (size_t v = 0; v + 5 < it.data.size(); v += 6) {
            float o[3];
            Transform(m, &it.data[v], o);
            for (int k = 0; k < 3; k++) { mn[k] = simul::Min(mn[k], o[k]); mx[k] = simul::Max(mx[k], o[k]); }
            any = true;
        }
    }
    if (!any) {
        center_ = QVector3D(0, 0, 0);
        radius_ = 1.0f;
        extent_[0] = extent_[1] = extent_[2] = 0.0f;
    } else {
        center_ = QVector3D((mn[0] + mx[0]) * 0.5f, (mn[1] + mx[1]) * 0.5f, (mn[2] + mx[2]) * 0.5f);
        for (int k = 0; k < 3; k++) extent_[k] = mx[k] - mn[k];
        const float d = simul::Sqrt(extent_[0] * extent_[0] + extent_[1] * extent_[1] + extent_[2] * extent_[2]);
        radius_ = simul::Max(d * 0.5f, 1e-4f);
    }
    rebuildGrid();
}

void ModelViewer::rebuildGrid() {
    // 격자 칸은 10의 거듭제곱 (모델 크기의 1/5 이하 중 가장 큰 값)
    gridCell_ = simul::Pow(10.0f, simul::Floor(simul::Log10(simul::Max(radius_, 1e-4f) * 0.4f)));
    const int n = 10;
    const float h = gridCell_ * static_cast<float>(n);
    gridData_.clear();
    for (int i = -n; i <= n; i++) {
        const float t = gridCell_ * static_cast<float>(i);
        const float a[12] = {t, -h, 0, t, h, 0, -h, t, 0, h, t, 0};
        gridData_.insert(gridData_.end(), a, a + 12);
    }
}

void ModelViewer::FitView() {
    target_ = center_;
    dist_ = radius_ * 2.8f;
    yaw_ = -60.0f;
    pitch_ = 25.0f;
    update();
}

void ModelViewer::SetShowGrid(bool show) {
    showGrid_ = show;
    update();
}

QString ModelViewer::Summary() const {
    if (items_.empty()) return QString();
    return QString("삼각형 %1개, 크기 %2 x %3 x %4")
        .arg(triangleCount_)
        .arg(static_cast<double>(extent_[0]), 0, 'g', 4)
        .arg(static_cast<double>(extent_[1]), 0, 'g', 4)
        .arg(static_cast<double>(extent_[2]), 0, 'g', 4);
}

void ModelViewer::initializeGL() {
    initializeOpenGLFunctions();
    prog_.reset(new QOpenGLShaderProgram);
    prog_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader);
    prog_->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader);
    prog_->bindAttributeLocation("aPos", 0);
    prog_->bindAttributeLocation("aNormal", 1);
    prog_->bindAttributeLocation("aUV", 2);
    prog_->bindAttributeLocation("aVColor", 3);
    prog_->link();
    buffersDirty_ = !items_.empty();
}

void ModelViewer::resizeGL(int, int) {}

void ModelViewer::uploadBuffers() {
    uploadItems(items_);
    buffersDirty_ = false;
}

void ModelViewer::uploadItems(ItemList& list) {
    for (size_t i = 0; i < list.size(); i++) {
        Item& it = *list[i];
        if (it.vbo.isCreated()) it.vbo.destroy();
        if (it.ibo.isCreated()) it.ibo.destroy();
        it.vbo.create();
        it.vbo.bind();
        it.vbo.allocate(it.data.data(), static_cast<int>(it.data.size() * sizeof(float)));
        it.vbo.release();
        it.ibo.create();
        it.ibo.bind();
        it.ibo.allocate(it.indices.data(), static_cast<int>(it.indices.size() * sizeof(uint32_t)));
        it.ibo.release();
        if (it.evbo.isCreated()) it.evbo.destroy();
        it.tex.reset();
        if (!it.extra.empty()) {
            it.evbo.create();
            it.evbo.bind();
            it.evbo.allocate(it.extra.data(), static_cast<int>(it.extra.size() * sizeof(float)));
            it.evbo.release();
        }
        if (!it.image.isNull()) {
            it.tex.reset(new QOpenGLTexture(it.image.mirrored(), QOpenGLTexture::GenerateMipMaps));   // glTF 의 uv 원점은 왼쪽 위라서 위아래를 뒤집어 올린다
            it.tex->setWrapMode(QOpenGLTexture::Repeat);
            it.tex->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::Linear);
        }
    }
}

QMatrix4x4 ModelViewer::viewMatrix() const {
    const float cy = simul::Cos(yaw_ * 3.14159265f / 180.0f), sy = simul::Sin(yaw_ * 3.14159265f / 180.0f);
    const float cp = simul::Cos(pitch_ * 3.14159265f / 180.0f), sp = simul::Sin(pitch_ * 3.14159265f / 180.0f);
    const QVector3D eye = target_ + dist_ * QVector3D(cp * cy, cp * sy, sp);
    QMatrix4x4 v;
    v.lookAt(eye, target_, QVector3D(0, 0, 1));  // Z-up
    return v;
}

QMatrix4x4 ModelViewer::projMatrix() const {
    const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    QMatrix4x4 proj;
    proj.perspective(45.0f, aspect, simul::Max(dist_ * 0.01f, radius_ * 0.01f * 0.1f), (dist_ + radius_) * 20.0f);
    return proj;
}

// 꼭짓점 8개(비트 0,1,2 = x,y,z 의 +/-)로 이루어진 상자의 모서리 12개를 그린다
void ModelViewer::drawBoxEdges(const dhVector c[8], const QVector4D& color) {
    static const int kEdges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    float v[24][3];
    for (int e = 0; e < 12; e++)
        for (int k = 0; k < 2; k++) {
            const dhVector& p = c[kEdges[e][k]];
            v[e * 2 + k][0] = p.x + objectPos_.x;  // 박스는 로봇 좌표계에서 계산되므로 이동량을 더해서 그린다
            v[e * 2 + k][1] = p.y + objectPos_.y;
            v[e * 2 + k][2] = p.z + objectPos_.z;
        }
    prog_->setUniformValue("uColor", color);
    prog_->setAttributeArray(0, GL_FLOAT, v, 3);
    glDrawArrays(GL_LINES, 0, 24);
}

void ModelViewer::SetLidar(int link, const simul::sensor::LidarSettings& settings, const dhMat& mount) {
    lidarOn_ = true;
    lidarDraw_ = true;
    lidarLink_ = link;
    lidarMount_ = mount;
    lidar_.SetSettings(settings);
    update();
}

void ModelViewer::ClearLidar() {
    lidarOn_ = false;
    lidarScan_ = simul::sensor::LidarScan();
    update();
}

const simul::sensor::LidarScan& ModelViewer::ScanLidar() {
    // 센서 자세(월드) = 오브젝트 이동 * 링크 변환 * 마운트
    dhMat pose = dhMat::Trans(objectPos_.x, objectPos_.y, objectPos_.z);
    if (lidarLink_ >= 0 && static_cast<size_t>(lidarLink_) < linkT_.size()) pose = pose * linkT_[static_cast<size_t>(lidarLink_)];
    pose = pose * lidarMount_;
    simul::Array<simul::sensor::LidarTarget> targets;
    for (size_t i = 0; i < items_.size(); i++) {
        Item& it = *items_[i];
        if (lidarLink_ >= 0 && it.link == lidarLink_) continue;  // 센서가 붙은 링크 자신은 보지 않는다
        if (it.link < 0 && !hasScene_) continue;                  // (로봇이 열려 있으면 gizmo 같은 보조 물체는 items_ 에 없다)
        if (it.data.empty() || it.indices.empty()) continue;
        simul::sensor::LidarTarget t;
        t.positions = it.data.data();
        t.vertexCount = it.data.size() / 6;
        t.indices = it.indices.data();
        t.indexCount = it.indices.size();
        t.stride = 6;
        t.world = itemWorld(it);
        t.id = it.link < 0 ? 0 : it.link;
        if (!it.bvhOk) {  // 처음 한 번만 (메시 좌표에서 만들어 두면 링크가 움직여도 그대로 쓴다)
            it.bvh.Build(it.data.data(), it.data.size() / 6, it.indices.data(), it.indices.size(), 6);
            it.bvhOk = true;
        }
        t.bvh = &it.bvh;
        targets.push_back(t);
    }
    lidar_.Scan(pose, targets.data(), targets.size(), &lidarScan_);
    lidarOrigin_ = pose.O();
    const dhVector ex = pose * dhVector(1.0f, 0.0f, 0.0f);
    lidarHeading_ = atan2f(ex.y - lidarOrigin_.y, ex.x - lidarOrigin_.x);
    return lidarScan_;
}

void ModelViewer::drawLidar() {
    const simul::sensor::LidarScan& scan = ScanLidar();
    emit lidarScanned(scan, lidarOrigin_, lidarHeading_);
    simul::Array<float> beams, pts;
    for (size_t i = 0; i < scan.size(); i++) {
        if (!scan.IsHit(i)) continue;
        beams.push_back(lidarOrigin_.x);
        beams.push_back(lidarOrigin_.y);
        beams.push_back(lidarOrigin_.z);
        beams.push_back(scan.points[i].x);
        beams.push_back(scan.points[i].y);
        beams.push_back(scan.points[i].z);
        // 맞은 점은 작은 십자(축 방향 선 3개, 한 변 3 cm)로 표시한다 (glPointSize 는 QOpenGLFunctions 에 없다)
        const float h = 0.015f;
        const dhVector& p = scan.points[i];
        const float c[6][3] = {{p.x - h, p.y, p.z}, {p.x + h, p.y, p.z}, {p.x, p.y - h, p.z}, {p.x, p.y + h, p.z}, {p.x, p.y, p.z - h}, {p.x, p.y, p.z + h}};
        for (int k = 0; k < 6; k++)
            for (int m = 0; m < 3; m++) pts.push_back(c[k][m]);
    }
    if (pts.empty()) return;
    glDisable(GL_DEPTH_TEST);  // 물체 안쪽에서 시작하는 광선도 보이게
    glLineWidth(1.0f);
    prog_->setUniformValue("uColor", QVector4D(0.3f, 0.9f, 0.95f, 0.30f));
    prog_->setAttributeArray(0, GL_FLOAT, beams.data(), 3);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(beams.size() / 3));
    glLineWidth(2.0f);
    prog_->setUniformValue("uColor", QVector4D(1.0f, 0.15f, 0.15f, 1.0f));
    prog_->setAttributeArray(0, GL_FLOAT, pts.data(), 3);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(pts.size() / 3));
    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
}

void ModelViewer::paintGL() {
    glClearColor(0.16f, 0.17f, 0.20f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!prog_ || !prog_->isLinked()) return;
    if (buffersDirty_) uploadBuffers();
    if (gizmoDirty_) {
        uploadItems(gizmo_);
        gizmoDirty_ = false;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const QMatrix4x4 proj = projMatrix();
    const QMatrix4x4 view = viewMatrix();

    prog_->bind();
    prog_->setUniformValue("uLit", 0.0f);
    prog_->setUniformValue("uUseTex", 0.0f);
    prog_->setUniformValue("uUseVC", 0.0f);
    prog_->setUniformValue("uShading", shading_);
    prog_->setUniformValue("uMetal", kDefaultMetal);
    prog_->setUniformValue("uRough", kDefaultRough);
    {   // 월드 위쪽(+z)이 카메라 좌표에서 어느 쪽인지 (PBR 의 하늘/바닥 환경)
        const QVector3D up = view.mapVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();
        prog_->setUniformValue("uUp", up);
    }

    // 격자와 축 (조명 없음)
    {
        const QMatrix4x4 mvp = proj * view;
        prog_->setUniformValue("uMVP", mvp);
        prog_->setUniformValue("uModelView", view);
        prog_->enableAttributeArray(0);
        prog_->disableAttributeArray(1);
        if (showGrid_ && !gridData_.empty()) {
            prog_->setUniformValue("uColor", QVector4D(0.35f, 0.37f, 0.42f, 1.0f));
            prog_->setAttributeArray(0, GL_FLOAT, gridData_.data(), 3);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(gridData_.size() / 3));
        }
        const float L = gridCell_ * 2.0f;
        const float axes[3][6] = {{0, 0, 0, L, 0, 0}, {0, 0, 0, 0, L, 0}, {0, 0, 0, 0, 0, L}};
        const QVector4D colors[3] = {QVector4D(0.95f, 0.25f, 0.25f, 1), QVector4D(0.3f, 0.85f, 0.3f, 1), QVector4D(0.3f, 0.5f, 1.0f, 1)};
        glLineWidth(2.0f);
        for (int a = 0; a < 3; a++) {
            prog_->setUniformValue("uColor", colors[a]);
            prog_->setAttributeArray(0, GL_FLOAT, axes[a], 3);
            glDrawArrays(GL_LINES, 0, 2);
        }
        if (hasRobot_ && (showBoxes_ || showObbs_)) {
            const size_t nl = robot_.links.size();
            simul::Array<bool> hit(nl, false), cand(nl, false);
            for (size_t i = 0; i < report_.collisions.size(); i++) {
                hit[static_cast<size_t>(report_.collisions[i].a)] = true;
                hit[static_cast<size_t>(report_.collisions[i].b)] = true;
            }
            for (size_t i = 0; i < report_.aabbCandidates.size(); i++) {
                cand[static_cast<size_t>(report_.aabbCandidates[i].a)] = true;
                cand[static_cast<size_t>(report_.aabbCandidates[i].b)] = true;
            }
            glLineWidth(2.0f);
            // AABB: 링크마다 하나. 노랑, AABB 후보이면 주황 (OBB 까지 확인되면 아래 OBB 가 빨갛게 나온다)
            if (showBoxes_) {
                for (size_t i = 0; i < report_.linkAabbs.size() && i < nl; i++) {
                    const simul::Aabb& b = report_.linkAabbs[i];
                    if (b.IsEmpty()) continue;
                    dhVector c[8];
                    for (int k = 0; k < 8; k++) c[k] = dhVector((k & 1) ? b.hi.x : b.lo.x, (k & 2) ? b.hi.y : b.lo.y, (k & 4) ? b.hi.z : b.lo.z);
                    drawBoxEdges(c, cand[i] ? QVector4D(1.0f, 0.55f, 0.1f, 1.0f) : QVector4D(0.95f, 0.8f, 0.2f, 1.0f));
                }
            }
            // OBB: 도형마다 하나. 하늘색, 충돌한 링크의 도형이면 빨강
            if (showObbs_) {
                for (size_t i = 0; i < report_.shapeObbs.size() && i < colModel_.shapes.size(); i++) {
                    const simul::Obb& o = report_.shapeObbs[i];
                    if (o.IsEmpty()) continue;
                    const int l = colModel_.shapes[i].link;
                    const bool red = l >= 0 && static_cast<size_t>(l) < nl && hit[static_cast<size_t>(l)];
                    dhVector c[8];
                    o.Corners(c);
                    drawBoxEdges(c, red ? QVector4D(1.0f, 0.2f, 0.2f, 1.0f) : QVector4D(0.3f, 0.85f, 0.95f, 1.0f));
                }
            }
        }
        glLineWidth(1.0f);
        prog_->disableAttributeArray(0);
    }

    // 모델 (불투명 먼저, 반투명은 나중에)
    prog_->setUniformValue("uLit", 1.0f);
    for (int pass = 0; pass < 2; pass++) {
        for (size_t i = 0; i < items_.size(); i++) {
            Item& it = *items_[i];
            const bool translucent = it.color[3] < 0.99f;
            if ((pass == 0) == translucent) continue;
            if (!it.vbo.isCreated() || !it.ibo.isCreated()) continue;
            const QMatrix4x4 mv = view * ToQt(itemWorld(it));
            prog_->setUniformValue("uMVP", proj * mv);
            prog_->setUniformValue("uModelView", mv);
            QVector4D col(it.color[0], it.color[1], it.color[2], it.color[3]);
            if ((selectedLink_ >= 0 && it.link == selectedLink_) || (selectedModel_ && it.link < 0)) {  // 선택한 링크: 밝은 노랑 쪽으로 섞는다
                const QVector4D hi(1.0f, 0.95f, 0.45f, col.w());
                col = col * 0.45f + hi * 0.55f;
                col.setW(it.color[3]);
            }
            prog_->setUniformValue("uColor", col);
            prog_->setUniformValue("uMetal", it.metallic);
            prog_->setUniformValue("uRough", it.roughness);
            const bool useTex = it.tex && it.evbo.isCreated(), useVc = it.hasVertexColor && it.evbo.isCreated();
            prog_->setUniformValue("uUseTex", useTex ? 1.0f : 0.0f);
            prog_->setUniformValue("uUseVC", useVc ? 1.0f : 0.0f);
            if (useTex) {
                glActiveTexture(GL_TEXTURE0);
                it.tex->bind();
                prog_->setUniformValue("uTex", 0);
            }
            it.vbo.bind();
            it.ibo.bind();
            prog_->enableAttributeArray(0);
            prog_->enableAttributeArray(1);
            prog_->setAttributeBuffer(0, GL_FLOAT, 0, 3, 6 * static_cast<int>(sizeof(float)));
            prog_->setAttributeBuffer(1, GL_FLOAT, 3 * static_cast<int>(sizeof(float)), 3, 6 * static_cast<int>(sizeof(float)));
            if (useTex || useVc) {
                it.evbo.bind();
                prog_->enableAttributeArray(2);
                prog_->enableAttributeArray(3);
                prog_->setAttributeBuffer(2, GL_FLOAT, 0, 2, 6 * static_cast<int>(sizeof(float)));
                prog_->setAttributeBuffer(3, GL_FLOAT, 2 * static_cast<int>(sizeof(float)), 4, 6 * static_cast<int>(sizeof(float)));
                it.evbo.release();
            }
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(it.indices.size()), GL_UNSIGNED_INT, nullptr);
            prog_->disableAttributeArray(0);
            prog_->disableAttributeArray(1);
            if (useTex || useVc) {
                prog_->disableAttributeArray(2);
                prog_->disableAttributeArray(3);
            }
            if (useTex) it.tex->release();
            it.vbo.release();
            it.ibo.release();
        }
    }
    prog_->setUniformValue("uUseTex", 0.0f);
    prog_->setUniformValue("uUseVC", 0.0f);
    if (lidarOn_ && lidarDraw_) {
        prog_->setUniformValue("uLit", 0.0f);
        prog_->setUniformValue("uMVP", proj * view);
        prog_->setUniformValue("uModelView", view);
        prog_->enableAttributeArray(0);
        prog_->disableAttributeArray(1);
        drawLidar();
    }
    if (showFrame_ && HasSelection()) drawSelectionFrame(proj, view);
    prog_->release();
}

// 선택한 것의 좌표계. 다른 물체에 가려지지 않게 깊이 버퍼를 지우고 그리며, 화면에서 보이는 크기가 일정하도록 거리에 비례해 키운다.
void ModelViewer::drawSelectionFrame(const QMatrix4x4& proj, const QMatrix4x4& view) {
    dhMat f;
    if (!SelectedFrame(&f)) return;
    QMatrix4x4 frame = ToQt(f);
    const QVector3D origin = frame.column(3).toVector3D();
    if (worldAxes()) {  // 위치만 쓰고 자세는 월드에 맞춘다
        frame.setToIdentity();
        frame.translate(origin);
    }
    const QVector3D eye = view.inverted().column(3).toVector3D();
    const float s = simul::Max((eye - origin).length() * 0.14f, 1e-6f);
    glClear(GL_DEPTH_BUFFER_BIT);
    if (!gizmo_.empty()) {
        QMatrix4x4 scaled = frame;
        scaled.scale(s);
        prog_->setUniformValue("uLit", 1.0f);
        for (size_t i = 0; i < gizmo_.size(); i++) {
            Item& it = *gizmo_[i];
            if (!it.vbo.isCreated() || !it.ibo.isCreated()) continue;
            const QMatrix4x4 mv = view * scaled * ToQt(it.base);
            prog_->setUniformValue("uMVP", proj * mv);
            prog_->setUniformValue("uModelView", mv);
            prog_->setUniformValue("uColor", QVector4D(it.color[0], it.color[1], it.color[2], 1.0f));
            it.vbo.bind();
            it.ibo.bind();
            prog_->enableAttributeArray(0);
            prog_->enableAttributeArray(1);
            prog_->setAttributeBuffer(0, GL_FLOAT, 0, 3, 6 * static_cast<int>(sizeof(float)));
            prog_->setAttributeBuffer(1, GL_FLOAT, 3 * static_cast<int>(sizeof(float)), 3, 6 * static_cast<int>(sizeof(float)));
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(it.indices.size()), GL_UNSIGNED_INT, nullptr);
            prog_->disableAttributeArray(0);
            prog_->disableAttributeArray(1);
            it.vbo.release();
            it.ibo.release();
        }
    } else {  // 좌표계 OBJ 가 없으면 선 3개
        prog_->setUniformValue("uLit", 0.0f);
        prog_->setUniformValue("uMVP", proj * view * frame);
        prog_->setUniformValue("uModelView", view * frame);
        const float axes[3][6] = {{0, 0, 0, s, 0, 0}, {0, 0, 0, 0, s, 0}, {0, 0, 0, 0, 0, s}};
        const QVector4D colors[3] = {QVector4D(0.92f, 0.2f, 0.2f, 1), QVector4D(0.25f, 0.8f, 0.25f, 1), QVector4D(0.25f, 0.45f, 0.95f, 1)};
        prog_->enableAttributeArray(0);
        prog_->disableAttributeArray(1);
        glLineWidth(3.0f);
        for (int a = 0; a < 3; a++) {
            prog_->setUniformValue("uColor", colors[a]);
            prog_->setAttributeArray(0, GL_FLOAT, axes[a], 3);
            glDrawArrays(GL_LINES, 0, 2);
        }
        glLineWidth(1.0f);
        prog_->disableAttributeArray(0);
    }
    drawRings(proj, view);
}

// 광선이 원점 o 를 지나고 법선이 n 인 평면과 만나는 곳의 점
static bool RayPlane(const QVector3D& ro, const QVector3D& rd, const QVector3D& o, const QVector3D& n, QVector3D* p) {
    const float den = QVector3D::dotProduct(rd, n);
    if (simul::Abs(den) < 1e-9f) return false;
    const float t = QVector3D::dotProduct(o - ro, n) / den;
    *p = ro + rd * t;
    return true;
}

void ModelViewer::mousePressEvent(QMouseEvent* e) {
    lastPos_ = e->pos();
    pressPos_ = e->pos();
    dragged_ = false;
    dragAxis_ = -1;
    dragRing_ = -1;
    dragModel_ = -1;
    if (e->button() == Qt::LeftButton && mouseEdit_) {  // 좌표계 화살표 위에서 누르면 카메라가 아니라 물체를 움직인다
        float u = 0, ringAng = 0;
        const int ringHit = hitRing(e->pos(), &ringAng);   // 링이 화살표와 겹치는 곳에서는 링을 먼저 잡는다
        const int axis = ringHit >= 0 ? -1 : hitGizmoAxis(e->pos(), &u);
        if (axis >= 0) {
            dragAxis_ = axis;
            dragU0_ = u;
            dragStart_ = objectPos_;
            dragIk_ = !CanMoveSelection();
            if (!dragIk_ && world_.Composite()) {  // 모델 하나를 움직인다: 시작 순간의 좌표계와 자세를 기억한다
                dragModel_ = ModelOfRoot(selectedLink_);
                float s = 0;
                selectionGizmo(&dragIkOrigin_, dragIkAxes_, &s);
                dragPose_ = ModelPose(dragModel_);
            }
            if (dragIk_) {  // 시작 순간의 좌표계를 기억해 둔다 (관절이 움직여도 축이 흔들리지 않게)
                float s = 0;
                selectionGizmo(&dragIkOrigin_, dragIkAxes_, &s);
            }
        } else {  // 화살표가 아니면 회전 링인지 본다
            const int ring = ringHit;
            if (ring >= 0) {
                const int model = ModelOfRoot(selectedLink_);
                if (!world_.Composite()) ensureComposite();  // 모델 하나도 자세를 attach 관절로 표현해야 돌릴 수 있다
                if (world_.Composite()) {
                    dragModel_ = model;
                    dragRing_ = ring;
                    // ensureComposite 로 다시 합쳐졌으니 링 기준을 새로 구한다
                    QVector3D a;
                    float s = 0;
                    QVector3D uu, vv;
                    if (ringBasis(ring, &dragIkOrigin_, &a, &uu, &vv, &s)) {
                        dragIkAxes_[0] = a;
                        dragPose_ = ModelPose(dragModel_);
                        QVector3D ro, rd, p;
                        if (mouseRay(e->pos(), &ro, &rd) && RayPlane(ro, rd, dragIkOrigin_, a, &p)) {
                            // 링 위에서 잡은 점의 접선을 화면으로 옮겨, 마우스가 그 방향으로 간 만큼 돌린다.
                            // (평면과 광선의 교점으로 각도를 구하면 링을 비스듬히 볼 때 값이 크게 튄다)
                            const QVector3D rad = p - dragIkOrigin_;
                            const float R = simul::Max(rad.length(), 1e-6f);
                            const QVector3D tan = QVector3D::crossProduct(a, rad).normalized();
                            const QMatrix4x4 pv = projMatrix() * viewMatrix();
                            auto scr = [&](const QVector3D& w) {
                                const QVector4D c = pv * QVector4D(w, 1.0f);
                                return QPointF((c.x() / c.w() * 0.5f + 0.5f) * width(), (1.0f - (c.y() / c.w() * 0.5f + 0.5f)) * height());
                            };
                            const QPointF s0 = scr(p), s1 = scr(p + tan * (R * 0.1f));
                            const QPointF st = (s1 - s0) / (R * 0.1f);          // 접선 방향 1 m 당 화면 픽셀
                            const float stLen = sqrtf(static_cast<float>(st.x() * st.x() + st.y() * st.y()));
                            const QPointF c0 = scr(dragIkOrigin_), c1 = scr(dragIkOrigin_ + (p - dragIkOrigin_).normalized() * R);
                            const QPointF rr = c1 - c0;
                            const float radiusPx = simul::Max(sqrtf(static_cast<float>(rr.x() * rr.x() + rr.y() * rr.y())), 20.0f);  // 링 반지름의 화면 크기
                            dragRingDir_ = stLen > 1e-3f ? QPointF(st.x() / stLen, st.y() / stLen) : QPointF(1, 0);
                            dragRingPx_ = simul::Max(stLen * R, radiusPx * 0.25f);  // 비스듬히 볼 때도 너무 예민해지지 않게
                        } else {
                            dragRing_ = -1;
                        }
                    } else {
                        dragRing_ = -1;
                    }
                }
            }
        }
    }
}

void ModelViewer::mouseReleaseEvent(QMouseEvent* e) {
    // 거의 움직이지 않고 뗀 왼쪽 버튼 = 클릭: 그 자리의 물체를 선택한다
    if (e->button() != Qt::LeftButton) return;
    if (dragAxis_ >= 0 || dragRing_ >= 0) {  // 화살표/링을 끌던 중이었다: 선택은 그대로 두고 끝낸다
        const bool rotated = dragRing_ >= 0;
        const int movedModel = (!dragIk_ && dragRing_ < 0 && dragModel_ < 0) ? ModelOfRoot(selectedLink_) : dragModel_;
        if (movedModel >= 0 && dragged_) emit modelDragFinished(movedModel, rotated);
        dragRing_ = -1;
        dragAxis_ = -1;
        dragIk_ = false;
        dragModel_ = -1;
        dragRing_ = -1;
        update();
        return;
    }
    if (dragged_) return;
    bool model = false;
    const int link = PickAt(e->pos(), &model);
    selectedLink_ = link;
    selectedModel_ = model;
    update();
    emit selectionPicked(link, model);
}

bool ModelViewer::mouseRay(const QPoint& pos, QVector3D* origin, QVector3D* dir) const {
    if (width() <= 0 || height() <= 0) return false;
    bool ok = false;
    const QMatrix4x4 inv = (projMatrix() * viewMatrix()).inverted(&ok);
    if (!ok) return false;
    const float x = 2.0f * (static_cast<float>(pos.x()) + 0.5f) / static_cast<float>(width()) - 1.0f;
    const float y = 1.0f - 2.0f * (static_cast<float>(pos.y()) + 0.5f) / static_cast<float>(height());
    const QVector4D n4 = inv * QVector4D(x, y, -1.0f, 1.0f), f4 = inv * QVector4D(x, y, 1.0f, 1.0f);
    if (n4.w() == 0.0f || f4.w() == 0.0f) return false;
    *origin = n4.toVector3D() / n4.w();
    *dir = f4.toVector3D() / f4.w() - *origin;  // 원점에서 먼 점까지가 t = 0..1
    return true;
}

bool ModelViewer::selectionGizmo(QVector3D* origin, QVector3D axis[3], float* scale) const {
    dhMat f;
    if (!SelectedFrame(&f)) return false;
    const QMatrix4x4 frame = ToQt(f);
    *origin = frame.column(3).toVector3D();
    for (int i = 0; i < 3; i++) axis[i] = frame.column(i).toVector3D().normalized();
    if (worldAxes()) {  // 모델을 돌려도 화살표가 따라 돌지 않게 월드 축에 고정한다 (오브젝트 축을 고르면 따라 돈다)
        axis[0] = QVector3D(1, 0, 0); axis[1] = QVector3D(0, 1, 0); axis[2] = QVector3D(0, 0, 1);
    }
    const QVector3D eye = viewMatrix().inverted().column(3).toVector3D();
    *scale = simul::Max((eye - *origin).length() * 0.14f, 1e-6f);  // drawSelectionFrame 과 같은 크기
    return true;
}

// 광선과 축 직선(점 p, 단위 방향 a) 사이의 가장 가까운 점의 축 위 위치 u. 평행이면 false
static bool RayAxisParam(const QVector3D& o, const QVector3D& d, const QVector3D& p, const QVector3D& a, float* u) {
    const QVector3D w0 = o - p;
    const float A = QVector3D::dotProduct(d, d), B = QVector3D::dotProduct(d, a);
    const float D = QVector3D::dotProduct(d, w0), E = QVector3D::dotProduct(a, w0);
    const float denom = A - B * B;
    if (!(simul::Abs(denom) > 1e-9f * simul::Max(A, 1e-12f))) return false;
    *u = (A * E - B * D) / denom;
    return true;
}

int ModelViewer::hitGizmoAxis(const QPoint& pos, float* uOut) const {
    QVector3D o, d, p, ax[3];
    float s = 0;
    if (!showFrame_ || !(CanMoveSelection() || CanIkSelection()) || !mouseRay(pos, &o, &d) || !selectionGizmo(&p, ax, &s)) return -1;
    int best = -1;
    float bestDist = 0.10f * s;  // 화살촉 반지름(0.075) 보다 조금 넉넉하게
    for (int i = 0; i < 3; i++) {
        float u = 0;
        if (!RayAxisParam(o, d, p, ax[i], &u)) continue;
        const float uc = simul::Max(0.0f, simul::Min(s, u));
        const QVector3D q = p + ax[i] * uc;
        const float dist = QVector3D::crossProduct(d, q - o).length() / simul::Max(d.length(), 1e-12f);
        if (dist < bestDist) {
            bestDist = dist;
            best = i;
            if (uOut) *uOut = u;
        }
    }
    return best;
}

// 회전 링의 기준: 링 평면의 법선(축, 월드), 평면 안의 직교 기준 u,v (u x v = 축), 화면 크기
bool ModelViewer::ringBasis(int axis, QVector3D* origin, QVector3D* a, QVector3D* u, QVector3D* v, float* scale) const {
    QVector3D ax[3];
    if (axis < 0 || axis > 2 || !selectionGizmo(origin, ax, scale)) return false;
    // 링은 모델이 얼마나 돌아가 있든 월드 X/Y/Z 에 고정한다 (모델 좌표축을 따라가면 한 번 돌린 뒤 링이 비스듬해진다)
    const QVector3D w[3] = {QVector3D(1, 0, 0), QVector3D(0, 1, 0), QVector3D(0, 0, 1)};
    if (ringLocal_) {  // 오브젝트 축에 붙인다
        *a = ax[axis];
        *u = ax[(axis + 1) % 3];
        *v = ax[(axis + 2) % 3];
        return true;
    }
    *a = w[axis];
    *u = w[(axis + 1) % 3];
    *v = w[(axis + 2) % 3];
    return true;
}

static const float kRingRadius = 0.72f;  // 링 반지름 (좌표계 화살표 길이 1 기준)

int ModelViewer::hitRing(const QPoint& pos, float* angle) const {
    if (!CanRotateSelection()) return -1;
    QVector3D ro, rd;
    if (!mouseRay(pos, &ro, &rd)) return -1;
    int best = -1;
    float bestErr = 1e30f;
    for (int i = 0; i < 3; i++) {
        QVector3D o, a, u, v, p;
        float s = 0;
        if (!ringBasis(i, &o, &a, &u, &v, &s) || !RayPlane(ro, rd, o, a, &p)) continue;
        const QVector3D r = p - o;
        const float err = simul::Abs(r.length() - kRingRadius * s);
        if (err < 0.05f * s && err < bestErr) {  // 링 두께보다 조금 넉넉하게
            bestErr = err;
            best = i;
            if (angle) *angle = simul::Atan2(QVector3D::dotProduct(r, v), QVector3D::dotProduct(r, u));
        }
    }
    return best;
}

// 선택한 모델의 회전 링 3개 (축마다 같은 색). 좌표계처럼 화면에서 일정한 크기로 그린다.
void ModelViewer::drawRings(const QMatrix4x4& proj, const QMatrix4x4& view) {
    if (!mouseEdit_ || !CanRotateSelection()) return;
    dhMat f;
    if (!SelectedFrame(&f)) return;
    const QVector3D origin = ToQt(f).column(3).toVector3D();
    const QVector3D eye = view.inverted().column(3).toVector3D();
    const float s = simul::Max((eye - origin).length() * 0.14f, 1e-6f);
    QMatrix4x4 scaled;  // 회전 없이 모델 원점에 놓는다 (월드 축 고정)
    scaled.translate(origin);
    scaled.scale(s * kRingRadius);
    prog_->setUniformValue("uLit", 0.0f);
    prog_->setUniformValue("uMVP", proj * view * scaled);
    prog_->setUniformValue("uModelView", view * scaled);
    const QVector4D colors[3] = {QVector4D(0.92f, 0.2f, 0.2f, 1), QVector4D(0.25f, 0.8f, 0.25f, 1), QVector4D(0.25f, 0.45f, 0.95f, 1)};
    const int N = 64;
    float pts[N + 1][3];
    prog_->enableAttributeArray(0);
    prog_->disableAttributeArray(1);
    glLineWidth(3.0f);
    for (int axis = 0; axis < 3; axis++) {
        const int i1 = (axis + 1) % 3, i2 = (axis + 2) % 3;
        for (int k = 0; k <= N; k++) {
            const float t = 6.2831853f * static_cast<float>(k) / static_cast<float>(N);
            pts[k][0] = pts[k][1] = pts[k][2] = 0.0f;
            pts[k][i1] = simul::Cos(t);
            pts[k][i2] = simul::Sin(t);
        }
        prog_->setUniformValue("uColor", (dragRing_ == axis || (dragRing_ < 0 && hoverRing_ == axis)) ? QVector4D(1.0f, 0.95f, 0.3f, 1) : colors[axis]);
        prog_->setAttributeArray(0, GL_FLOAT, pts, 3);
        glDrawArrays(GL_LINE_STRIP, 0, N + 1);
    }
    glLineWidth(1.0f);
    prog_->disableAttributeArray(0);
}

int ModelViewer::PickAt(const QPoint& pos, bool* model) const {
    if (model) *model = false;
    if (items_.empty()) return -1;
    QVector3D origin, dir;
    if (!mouseRay(pos, &origin, &dir)) return -1;
    float bestT = 2.0f;
    int bestLink = -1;
    bool bestModel = false;
    for (size_t i = 0; i < items_.size(); i++) {
        const Item& it = *items_[i];
        if (it.link < 0 && !hasScene_) continue;
        bool inverted = false;
        const QMatrix4x4 local = ToQt(itemWorld(it)).inverted(&inverted);  // 광선을 물체 좌표로 옮겨서 검사한다
        if (!inverted) continue;
        const QVector3D o = local.map(origin), d = local.mapVector(dir);
        float t = 0.0f;
        if (simul::RaycastTriangles(it.data.data(), it.data.size() / 6, it.indices.data(), it.indices.size(), dhVector(o.x(), o.y(), o.z()),
                                  dhVector(d.x(), d.y(), d.z()), 1.0f, &t, nullptr, 6) && t < bestT) {
            bestT = t;
            bestLink = it.link;
            bestModel = it.link < 0;
        }
    }
    if (model) *model = bestModel;
    return bestLink;
}

void ModelViewer::mouseMoveEvent(QMouseEvent* e) {
    if (e->buttons() == Qt::NoButton) {
        const int hr = mouseEdit_ ? hitRing(e->pos(), nullptr) : -1;
        if (hr != hoverRing_) { hoverRing_ = hr; update(); }
        return;
    }
    const QPoint d = e->pos() - lastPos_;
    lastPos_ = e->pos();
    if ((e->pos() - pressPos_).manhattanLength() > 4) dragged_ = true;
    if (dragAxis_ >= 0 && dragIk_ && (e->buttons() & Qt::LeftButton)) {
        QVector3D o, dd;
        float u = 0;
        if (mouseRay(e->pos(), &o, &dd) && RayAxisParam(o, dd, dragIkOrigin_, dragIkAxes_[dragAxis_], &u) &&
            selectedLink_ >= 0 && static_cast<size_t>(selectedLink_) < linkT_.size()) {
            const QVector3D w = dragIkOrigin_ + dragIkAxes_[dragAxis_] * (u - dragU0_);  // 월드 목표
            dhMat target = linkT_[static_cast<size_t>(selectedLink_)];                     // 자세는 그대로, 위치만 옮긴다
            target.v[12] = w.x() - objectPos_.x;  // 로봇 자신의 좌표계(루트 기준)로
            target.v[13] = w.y() - objectPos_.y;
            target.v[14] = w.z() - objectPos_.z;
            simul::kinematics::IkOptions opt;
            opt.maxIterations = 40;
            const simul::kinematics::IkResult r = simul::kinematics::SolveIk(robot_, q_, selectedLink_, target, opt);
            SetJointValues(r.q);
            emit jointValuesDragged(r.q);
        }
    } else if (dragRing_ >= 0 && dragModel_ >= 0 && (e->buttons() & Qt::LeftButton)) {
        const QPointF dm(e->pos().x() - pressPos_.x(), e->pos().y() - pressPos_.y());
        const float a = static_cast<float>((dm.x() * dragRingDir_.x() + dm.y() * dragRingDir_.y())) / dragRingPx_;  // 접선 방향으로 간 거리 / 반지름 = 각도
        // 링의 축(월드) 둘레로 모델 원점을 지나게 돌린다: M = T(o) R(axis, a) T(-o) M0 (쿼터니언 회전)
        const dhVector ax(dragIkAxes_[0].x(), dragIkAxes_[0].y(), dragIkAxes_[0].z());
        const dhMat rot = dhMat::Trans(dragIkOrigin_.x(), dragIkOrigin_.y(), dragIkOrigin_.z()) * dhMat::Quat(a, ax) *
                          dhMat::Trans(-dragIkOrigin_.x(), -dragIkOrigin_.y(), -dragIkOrigin_.z());
        SetModelPose(dragModel_, rot * dragPose_);
    } else if (dragAxis_ >= 0 && dragModel_ >= 0 && (e->buttons() & Qt::LeftButton)) {
        QVector3D o, dd;
        float u = 0;
        if (mouseRay(e->pos(), &o, &dd) && RayAxisParam(o, dd, dragIkOrigin_, dragIkAxes_[dragAxis_], &u)) {
            const QVector3D delta = dragIkAxes_[dragAxis_] * (u - dragU0_);
            dhMat m = dragPose_;  // 자세는 그대로 두고 위치만 옮긴다
            m.v[12] += delta.x();
            m.v[13] += delta.y();
            m.v[14] += delta.z();
            SetModelPose(dragModel_, m);
        }
    } else if (dragAxis_ >= 0 && (e->buttons() & Qt::LeftButton)) {
        QVector3D o, dd, p, ax[3];
        float s = 0, u = 0;
        // 이동 중에도 좌표계는 시작 위치를 기준으로 계산해야 흔들리지 않는다: 시작 위치로 되돌려 놓고 축을 구한다
        const dhVector cur = objectPos_;
        objectPos_ = dragStart_;
        const bool ok = mouseRay(e->pos(), &o, &dd) && selectionGizmo(&p, ax, &s) && RayAxisParam(o, dd, p, ax[dragAxis_], &u);
        objectPos_ = cur;
        if (ok) {
            const QVector3D delta = ax[dragAxis_] * (u - dragU0_);
            objectPos_ = dhVector(dragStart_.x + delta.x(), dragStart_.y + delta.y(), dragStart_.z + delta.z());
            update();
            emit objectMoved();
        }
    } else if (e->buttons() & Qt::LeftButton) {
        yaw_ -= static_cast<float>(d.x()) * 0.4f;
        pitch_ = simul::Max(-89.0f, simul::Min(89.0f, pitch_ + static_cast<float>(d.y()) * 0.4f));
        update();
    } else if (e->buttons() & (Qt::RightButton | Qt::MiddleButton)) {
        // 화면 기준 오른쪽/위 방향으로 이동
        const QMatrix4x4 v = viewMatrix();
        const QVector3D right(v(0, 0), v(0, 1), v(0, 2)), up(v(1, 0), v(1, 1), v(1, 2));
        const float s = dist_ * 0.0016f;
        target_ += (-static_cast<float>(d.x()) * right + static_cast<float>(d.y()) * up) * s;
        update();
    }
}

void ModelViewer::wheelEvent(QWheelEvent* e) {
    dist_ = simul::Max(radius_ * 0.02f, dist_ * simul::Pow(0.9985f, static_cast<float>(e->angleDelta().y())));
    update();
}

void ModelViewer::mouseDoubleClickEvent(QMouseEvent*) { FitView(); }
