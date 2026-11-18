#include "CURDF/Scale.h"

namespace simul {
namespace urdf {

namespace {

void ScaleTranslation(dhMat* m, float s) {
    m->v[12] *= s;
    m->v[13] *= s;
    m->v[14] *= s;
}

void ScaleGeometry(Geometry* g, float s) {
    for (int k = 0; k < 3; k++) {
        g->size[k] *= s;
        g->scale[k] *= s;   // 메시: 파일 크기 위에 s 배
    }
    g->radius *= s;
    g->length *= s;
}

}  // namespace

bool ScaleRobot(const Robot& in, double sd, bool massScales, Robot* out) {
    if (!out || !(sd > 0.0) || !(sd < 1e9)) return false;
    *out = in;
    const float s = static_cast<float>(sd);
    const float s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    for (size_t l = 0; l < out->links.size(); l++) {
        Link& link = out->links[l];
        for (size_t v = 0; v < link.visuals.size(); v++) {
            ScaleTranslation(&link.visuals[v].origin, s);
            ScaleGeometry(&link.visuals[v].geometry, s);
        }
        for (size_t c = 0; c < link.collisions.size(); c++) {
            ScaleTranslation(&link.collisions[c].origin, s);
            ScaleGeometry(&link.collisions[c].geometry, s);
        }
        ScaleTranslation(&link.inertial.origin, s);
        if (massScales) {
            Inertial& i = link.inertial;
            i.mass *= s3;
            i.ixx *= s5; i.ixy *= s5; i.ixz *= s5; i.iyy *= s5; i.iyz *= s5; i.izz *= s5;
        }
    }
    for (size_t j = 0; j < out->joints.size(); j++) {
        Joint& jt = out->joints[j];
        ScaleTranslation(&jt.origin, s);
        const bool prismatic = jt.type == kJointPrismatic;
        if (prismatic) {
            jt.lower *= s;
            jt.upper *= s;
            jt.velocity *= s;
            if (jt.hasMimic) jt.mimicOffset *= s;
        }
        if (massScales) {
            const float f = prismatic ? s3 : s4;   // 힘은 질량(s^3), 토크는 질량 x 길이(s^4)
            jt.effort *= f;
            jt.damping *= prismatic ? s2 : s4;     // 점성은 힘/속도: 이동 관절은 s^3 / s = s^2, 회전 관절은 토크 / 각속도 = s^4
            jt.friction *= f;
        }
    }
    for (size_t i = 0; i < out->lidars.size(); i++) ScaleTranslation(&out->lidars[i].pose, s);
    return true;
}

}  // namespace urdf
}  // namespace simul
