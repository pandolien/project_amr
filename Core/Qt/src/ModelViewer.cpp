#include "CoreQt/ModelViewer.h"

#include <QFileInfo>
#include <QMouseEvent>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <limits>

#include "CMesh/MeshLoader.h"

namespace {

const char* kVertexShader =
    "#version 120\n"
    "attribute vec3 aPos;\n"
    "attribute vec3 aNormal;\n"
    "uniform mat4 uMVP;\n"
    "uniform mat4 uModelView;\n"
    "varying vec3 vN;\n"
    "void main() {\n"
    "    vN = mat3(uModelView) * aNormal;\n"
    "    gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "}\n";

// 카메라 방향 조명 + 보조광. 양면 렌더링을 위해 뒷면은 법선을 뒤집는다.
const char* kFragmentShader =
    "#version 120\n"
    "uniform vec4 uColor;\n"
    "uniform float uLit;\n"
    "varying vec3 vN;\n"
    "void main() {\n"
    "    vec3 n = normalize(vN);\n"
    "    if (!gl_FrontFacing) n = -n;\n"
    "    float l = 0.30 + 0.55 * max(dot(n, vec3(0.0, 0.0, 1.0)), 0.0)\n"
    "                   + 0.30 * max(dot(n, normalize(vec3(-0.5, 0.7, 0.5))), 0.0);\n"
    "    gl_FragColor = vec4(uColor.rgb * mix(1.0, min(l, 1.2), uLit), uColor.a);\n"
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

}  // namespace

ModelViewer::ModelViewer(QWidget* parent)
    : QOpenGLWidget(parent), buffersDirty_(false), hasRobot_(false), triangleCount_(0), radius_(1.0f),
      dist_(3.0f), yaw_(-60.0f), pitch_(25.0f), showBoxes_(false), showObbs_(false), showGrid_(true), gridCell_(1.0f) {
    selectedLink_ = -1;
    hasScene_ = false;
    extent_[0] = extent_[1] = extent_[2] = 0.0f;
    setMinimumSize(320, 240);
    setFocusPolicy(Qt::StrongFocus);
    rebuildGrid();
}

ModelViewer::~ModelViewer() {
    makeCurrent();
    releaseBuffers();
    prog_.reset();
    doneCurrent();
}

void ModelViewer::releaseBuffers() {
    for (size_t i = 0; i < items_.size(); i++) {
        if (items_[i]->vbo.isCreated()) items_[i]->vbo.destroy();
        if (items_[i]->ibo.isCreated()) items_[i]->ibo.destroy();
    }
}

void ModelViewer::Clear() {
    // GL 자원은 컨텍스트가 있을 때만 지울 수 있다. 없으면 다음 paintGL 에서 정리된다.
    if (isValid()) {
        makeCurrent();
        releaseBuffers();
        doneCurrent();
    }
    items_.clear();
    hasRobot_ = false;
    robot_ = amr::urdf::Robot();
    linkT_.clear();
    q_.clear();
    colModel_ = amr::RobotCollisionModel();
    report_ = amr::SelfCollisionReport();
    selectedLink_ = -1;
    scene_ = amr::Scene();
    hasScene_ = false;
    warnings_.clear();
    triangleCount_ = 0;
    buffersDirty_ = false;
    updateBounds();
    update();
}

void ModelViewer::addScene(const amr::Scene& scene, int link, const dhMat& local, const float* colorOverride) {
    const std::vector<dhMat> world = scene.WorldTransforms();
    for (size_t n = 0; n < scene.nodes.size(); n++) {
        const amr::SceneNode& node = scene.nodes[n];
        if (node.mesh < 0 || static_cast<size_t>(node.mesh) >= scene.meshes.size()) continue;
        const amr::Mesh& mesh = scene.meshes[static_cast<size_t>(node.mesh)];
        for (size_t p = 0; p < mesh.primitives.size(); p++) {
            const amr::Primitive& prim = mesh.primitives[p];
            if (prim.indices.empty() || prim.normals.size() != prim.positions.size()) continue;
            std::unique_ptr<Item> it(new Item);
            const size_t nv = prim.VertexCount();
            it->data.resize(nv * 6);
            for (size_t v = 0; v < nv; v++) {
                for (size_t k = 0; k < 3; k++) {
                    it->data[v * 6 + k] = prim.positions[v * 3 + k];
                    it->data[v * 6 + 3 + k] = prim.normals[v * 3 + k];
                }
            }
            it->indices = prim.indices;
            it->base = local * world[n];
            it->link = link;
            if (colorOverride) {
                for (int k = 0; k < 4; k++) it->color[k] = colorOverride[k];
            } else if (prim.material >= 0 && static_cast<size_t>(prim.material) < scene.materials.size()) {
                const amr::Color& c = scene.materials[static_cast<size_t>(prim.material)].baseColor;
                it->color[0] = c.r; it->color[1] = c.g; it->color[2] = c.b; it->color[3] = c.a;
            } else {
                for (int k = 0; k < 4; k++) it->color[k] = kDefaultColor[k];
            }
            triangleCount_ += prim.TriangleCount();
            items_.push_back(std::move(it));
        }
    }
    for (size_t i = 0; i < scene.warnings.size(); i++) warnings_ << QString::fromStdString(scene.warnings[i]);
}

void ModelViewer::SetScene(const amr::Scene& scene) {
    Clear();
    scene_ = scene;
    hasScene_ = true;
    addScene(scene, -1, dhMat(), nullptr);
    buffersDirty_ = true;
    updateBounds();
    FitView();
    emit modelChanged();
}

void ModelViewer::SetRobot(const amr::urdf::Robot& robot, const amr::urdf::RobotMeshes& meshes) {
    Clear();
    robot_ = robot;
    hasRobot_ = true;
    linkT_ = robot_.LinkTransforms(amr::urdf::JointValues());
    for (size_t i = 0; i < meshes.visuals.size(); i++) {
        const amr::urdf::LinkVisual& lv = meshes.visuals[i];
        float color[4];
        if (lv.hasColor) { color[0] = lv.color.r; color[1] = lv.color.g; color[2] = lv.color.b; color[3] = lv.color.a; }
        addScene(lv.scene, lv.link, lv.local, lv.hasColor ? color : nullptr);
    }
    for (size_t i = 0; i < meshes.warnings.size(); i++) warnings_ << QString::fromStdString(meshes.warnings[i]);
    for (size_t i = 0; i < robot_.warnings.size(); i++) warnings_ << QString::fromStdString(robot_.warnings[i]);
    amr::CollisionModelOptions copt;
    copt.load = options_;
    copt.fallbackToVisual = true;  // <collision> 이 없는 URDF 도 박스를 볼 수 있게 한다
    amr::BuildCollisionModel(robot_, copt, &colModel_);
    for (size_t i = 0; i < colModel_.warnings.size(); i++) warnings_ << QString::fromStdString(colModel_.warnings[i]);
    recomputeCollisions();
    buffersDirty_ = true;
    updateBounds();
    FitView();
    emit modelChanged();
}

void ModelViewer::SetJointValues(const amr::urdf::JointValues& q) {
    if (!hasRobot_) return;
    q_ = q;
    linkT_ = robot_.LinkTransforms(q);
    recomputeCollisions();
    update();
}

void ModelViewer::recomputeCollisions() {
    const std::vector<amr::CollisionPair> before = report_.collisions;
    const std::vector<amr::CollisionPair> beforeCand = report_.aabbCandidates;
    report_ = amr::CheckSelfCollisions(robot_, colModel_, q_);
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
    update();
}

std::vector<int> ModelViewer::CollidingLinks() const {
    std::vector<int> out;
    for (size_t i = 0; i < report_.collisions.size(); i++) {
        out.push_back(report_.collisions[i].a);
        out.push_back(report_.collisions[i].b);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

QStringList ModelViewer::CollisionDescriptions() const {
    QStringList out;
    for (size_t i = 0; i < report_.collisions.size(); i++) {
        const amr::CollisionPair& p = report_.collisions[i];
        out << QString::fromStdString(robot_.links[static_cast<size_t>(p.a)].name) + QString::fromUtf8(" <-> ") +
                   QString::fromStdString(robot_.links[static_cast<size_t>(p.b)].name);
    }
    return out;
}

bool ModelViewer::LoadFile(const QString& path, QString* error) {
    const std::string p = path.toLocal8Bit().toStdString();
    const QString ext = QFileInfo(path).suffix().toLower();
    std::string err;
    if (ext == "urdf") {
        amr::urdf::Robot robot;
        if (!amr::urdf::LoadUrdf(p, &robot, &err)) {
            if (error) *error = QString::fromStdString(err);
            return false;
        }
        amr::urdf::RobotMeshes meshes;
        amr::urdf::LoadRobotMeshes(robot, options_, &meshes);  // 일부 실패는 경고로 남는다
        SetRobot(robot, meshes);
        return true;
    }
    amr::Scene scene;
    if (!amr::LoadMesh(p, &scene, &err)) {
        if (error) *error = QString::fromStdString(err);
        return false;
    }
    if (ext == "glb" || ext == "gltf") scene.ApplyRootTransform(dhMat::RotX(static_cast<float>(dhRad(90.0))));  // Y-up -> Z-up
    SetScene(scene);
    return true;
}

void ModelViewer::updateBounds() {
    float mn[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    float mx[3] = {-mn[0], -mn[1], -mn[2]};
    bool any = false;
    for (size_t i = 0; i < items_.size(); i++) {
        const Item& it = *items_[i];
        dhMat m = it.base;
        if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
        for (size_t v = 0; v + 5 < it.data.size(); v += 6) {
            float o[3];
            Transform(m, &it.data[v], o);
            for (int k = 0; k < 3; k++) { mn[k] = std::min(mn[k], o[k]); mx[k] = std::max(mx[k], o[k]); }
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
        const float d = std::sqrt(extent_[0] * extent_[0] + extent_[1] * extent_[1] + extent_[2] * extent_[2]);
        radius_ = std::max(d * 0.5f, 1e-4f);
    }
    rebuildGrid();
}

void ModelViewer::rebuildGrid() {
    // 격자 칸은 10의 거듭제곱 (모델 크기의 1/5 이하 중 가장 큰 값)
    gridCell_ = std::pow(10.0f, std::floor(std::log10(std::max(radius_, 1e-4f) * 0.4f)));
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
    prog_->link();
    buffersDirty_ = !items_.empty();
}

void ModelViewer::resizeGL(int, int) {}

void ModelViewer::uploadBuffers() {
    for (size_t i = 0; i < items_.size(); i++) {
        Item& it = *items_[i];
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
    }
    buffersDirty_ = false;
}

QMatrix4x4 ModelViewer::viewMatrix() const {
    const float cy = std::cos(yaw_ * 3.14159265f / 180.0f), sy = std::sin(yaw_ * 3.14159265f / 180.0f);
    const float cp = std::cos(pitch_ * 3.14159265f / 180.0f), sp = std::sin(pitch_ * 3.14159265f / 180.0f);
    const QVector3D eye = target_ + dist_ * QVector3D(cp * cy, cp * sy, sp);
    QMatrix4x4 v;
    v.lookAt(eye, target_, QVector3D(0, 0, 1));  // Z-up
    return v;
}

// 꼭짓점 8개(비트 0,1,2 = x,y,z 의 +/-)로 이루어진 상자의 모서리 12개를 그린다
void ModelViewer::drawBoxEdges(const dhVector c[8], const QVector4D& color) {
    static const int kEdges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    float v[24][3];
    for (int e = 0; e < 12; e++)
        for (int k = 0; k < 2; k++) {
            const dhVector& p = c[kEdges[e][k]];
            v[e * 2 + k][0] = p.x;
            v[e * 2 + k][1] = p.y;
            v[e * 2 + k][2] = p.z;
        }
    prog_->setUniformValue("uColor", color);
    prog_->setAttributeArray(0, GL_FLOAT, v, 3);
    glDrawArrays(GL_LINES, 0, 24);
}

void ModelViewer::paintGL() {
    glClearColor(0.16f, 0.17f, 0.20f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!prog_ || !prog_->isLinked()) return;
    if (buffersDirty_) uploadBuffers();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    QMatrix4x4 proj;
    proj.perspective(45.0f, aspect, std::max(dist_ * 0.01f, radius_ * 0.01f * 0.1f), (dist_ + radius_) * 20.0f);
    const QMatrix4x4 view = viewMatrix();

    prog_->bind();
    prog_->setUniformValue("uLit", 0.0f);

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
            std::vector<bool> hit(nl, false), cand(nl, false);
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
                    const amr::Aabb& b = report_.linkAabbs[i];
                    if (b.IsEmpty()) continue;
                    dhVector c[8];
                    for (int k = 0; k < 8; k++) c[k] = dhVector((k & 1) ? b.hi.x : b.lo.x, (k & 2) ? b.hi.y : b.lo.y, (k & 4) ? b.hi.z : b.lo.z);
                    drawBoxEdges(c, cand[i] ? QVector4D(1.0f, 0.55f, 0.1f, 1.0f) : QVector4D(0.95f, 0.8f, 0.2f, 1.0f));
                }
            }
            // OBB: 도형마다 하나. 하늘색, 충돌한 링크의 도형이면 빨강
            if (showObbs_) {
                for (size_t i = 0; i < report_.shapeObbs.size() && i < colModel_.shapes.size(); i++) {
                    const amr::Obb& o = report_.shapeObbs[i];
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
            dhMat m = it.base;
            if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
            const QMatrix4x4 mv = view * ToQt(m);
            prog_->setUniformValue("uMVP", proj * mv);
            prog_->setUniformValue("uModelView", mv);
            QVector4D col(it.color[0], it.color[1], it.color[2], it.color[3]);
            if (selectedLink_ >= 0 && it.link == selectedLink_) {  // 선택한 링크: 밝은 노랑 쪽으로 섞는다
                const QVector4D hi(1.0f, 0.95f, 0.45f, col.w());
                col = col * 0.45f + hi * 0.55f;
                col.setW(it.color[3]);
            }
            prog_->setUniformValue("uColor", col);
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
    }
    prog_->release();
}

void ModelViewer::mousePressEvent(QMouseEvent* e) { lastPos_ = e->pos(); }

void ModelViewer::mouseMoveEvent(QMouseEvent* e) {
    const QPoint d = e->pos() - lastPos_;
    lastPos_ = e->pos();
    if (e->buttons() & Qt::LeftButton) {
        yaw_ -= static_cast<float>(d.x()) * 0.4f;
        pitch_ = std::max(-89.0f, std::min(89.0f, pitch_ + static_cast<float>(d.y()) * 0.4f));
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
    dist_ = std::max(radius_ * 0.02f, dist_ * std::pow(0.9985f, static_cast<float>(e->angleDelta().y())));
    update();
}

void ModelViewer::mouseDoubleClickEvent(QMouseEvent*) { FitView(); }
