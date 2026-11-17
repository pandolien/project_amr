// URDF 파서, 구조 검증, 순기구학
#include "CBase/Text.h"
#include "CURDF/Urdf.h"

#include "CBase/Math.h"
#include "CBase/Map.h"

#include "3d/dhEuler.h"
#include "CMesh/Primitives.h"
#include "FileUtil.h"
#include "tinyxml2.h"

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace simul {
namespace urdf {

void Robot::Clear() {
    name.clear();
    links.clear();
    joints.clear();
    materials.clear();
    lidars.clear();
    root = -1;
    order.clear();
    baseDir.clear();
    warnings.clear();
}

int Robot::FindLink(const simul::String& n) const {
    for (size_t i = 0; i < links.size(); i++) if (links[i].name == n) return static_cast<int>(i);
    return -1;
}

int Robot::FindJoint(const simul::String& n) const {
    for (size_t i = 0; i < joints.size(); i++) if (joints[i].name == n) return static_cast<int>(i);
    return -1;
}

simul::Array<int> Robot::ActuatedJoints() const {
    simul::Array<int> r;
    for (size_t i = 0; i < joints.size(); i++) if (joints[i].IsMovable() && !joints[i].hasMimic) r.push_back(static_cast<int>(i));
    return r;
}

namespace {

float Clamp(const Joint& j, float v) {
    if (j.type == kJointContinuous || !j.hasLimit) return v;
    if (v < j.lower) return j.lower;
    if (v > j.upper) return j.upper;
    return v;
}

float ValueRec(const Robot& r, const JointValues& q, int idx, bool clampToLimits, size_t depth) {
    const Joint& j = r.joints[static_cast<size_t>(idx)];
    float v = 0.0f;
    if (j.hasMimic && depth < r.joints.size()) {
        const int m = r.FindJoint(j.mimicJoint);
        if (m >= 0) v = j.mimicMultiplier * ValueRec(r, q, m, clampToLimits, depth + 1) + j.mimicOffset;
    } else {
        JointValues::const_iterator it = q.find(j.name);
        if (it != q.end()) v = it->second;
    }
    if (!j.IsMovable()) return 0.0f;
    return clampToLimits ? Clamp(j, v) : v;
}

}  // namespace

float Robot::EffectiveJointValue(const JointValues& q, int jointIndex, bool clampToLimits) const {
    if (jointIndex < 0 || static_cast<size_t>(jointIndex) >= joints.size()) return 0.0f;
    return ValueRec(*this, q, jointIndex, clampToLimits, 0);
}

simul::Array<dhMat> Robot::LinkTransforms(const JointValues& q, bool clampToLimits) const {
    simul::Array<dhMat> T(links.size());  // 기본 단위행렬
    for (size_t k = 0; k < order.size(); k++) {
        const int ji = order[k];
        const Joint& j = joints[static_cast<size_t>(ji)];
        const int p = FindLink(j.parent), c = FindLink(j.child);
        if (p < 0 || c < 0) continue;
        dhMat motion;
        const float v = EffectiveJointValue(q, ji, clampToLimits);
        if (j.type == kJointRevolute || j.type == kJointContinuous) motion = dhMat::Quat(v, j.axis);
        else if (j.type == kJointPrismatic) motion = dhMat::Trans(j.axis.x * v, j.axis.y * v, j.axis.z * v);
        T[static_cast<size_t>(c)] = T[static_cast<size_t>(p)] * j.origin * motion;
    }
    return T;
}

namespace {

struct Parser {
    Robot* robot;
    simul::String error;

    bool Fail(const simul::String& msg) {
        if (error.empty()) error = msg;
        return false;
    }

    // "1 2 3" 형태의 실수 목록. 개수가 정확히 n 이고 유한해야 한다.
    bool ParseFloats(const char* text, size_t n, float* out) {
        const char* p = text ? text : "";
        for (size_t i = 0; i < n; i++) {
            double v;
            if (!simul::ParseDouble(p, &v, &p) || !simul::IsFinite(v)) return false;
            out[i] = static_cast<float>(v);
            if (!simul::IsFinite(out[i])) return false;  // double 로는 유한해도 float 로 넘치는 값
        }
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        return *p == '\0';  // 남은 것이 있으면 개수가 맞지 않는다
    }

    bool Float(XMLElement* e, const char* attr, float def, float* out, const char* ctx) {
        const char* s = e->Attribute(attr);
        *out = def;
        if (!s) return true;
        if (!ParseFloats(s, 1, out)) return Fail(simul::String(ctx) + ": '" + attr + "' 값이 올바른 숫자가 아닙니다 (" + s + ")");
        return true;
    }

    bool Origin(XMLElement* parent, dhMat* out, const char* ctx) {
        *out = dhMat();
        XMLElement* o = parent->FirstChildElement("origin");
        if (!o) return true;
        float xyz[3] = {0, 0, 0}, rpy[3] = {0, 0, 0};
        if (o->Attribute("xyz") && !ParseFloats(o->Attribute("xyz"), 3, xyz)) return Fail(simul::String(ctx) + ": origin xyz 가 올바르지 않습니다");
        if (o->Attribute("rpy") && !ParseFloats(o->Attribute("rpy"), 3, rpy)) return Fail(simul::String(ctx) + ": origin rpy 가 올바르지 않습니다");
        *out = dhEuler(rpy[2], rpy[1], rpy[0]).GetMat();  // Rz(yaw) Ry(pitch) Rx(roll)
        out->v[12] = xyz[0];
        out->v[13] = xyz[1];
        out->v[14] = xyz[2];
        return true;
    }

    bool ParseColor(XMLElement* colorEl, Color* c, const char* ctx) {
        float v[4];
        if (!colorEl->Attribute("rgba") || !ParseFloats(colorEl->Attribute("rgba"), 4, v))
            return Fail(simul::String(ctx) + ": color rgba 는 숫자 4개여야 합니다");
        *c = Color(v[0], v[1], v[2], v[3]);
        return true;
    }

    bool ParseMaterial(XMLElement* m, MaterialDef* out, const char* ctx) {
        out->name = m->Attribute("name") ? m->Attribute("name") : "";
        XMLElement* col = m->FirstChildElement("color");
        if (col) {
            if (!ParseColor(col, &out->color, ctx)) return false;
            out->hasColor = true;
        }
        XMLElement* tex = m->FirstChildElement("texture");
        if (tex && tex->Attribute("filename")) out->texture = tex->Attribute("filename");
        return true;
    }

    bool ParseGeometry(XMLElement* g, Geometry* out, const simul::String& ctx) {
        XMLElement* e = g->FirstChildElement();
        if (!e) return Fail(ctx + ": geometry 가 비어 있습니다");
        const simul::String n = e->Name();
        if (n == "box") {
            out->type = kGeomBox;
            if (!e->Attribute("size") || !ParseFloats(e->Attribute("size"), 3, out->size)) return Fail(ctx + ": box size 는 숫자 3개여야 합니다");
            if (out->size[0] < 0 || out->size[1] < 0 || out->size[2] < 0) return Fail(ctx + ": box size 가 음수입니다");
        } else if (n == "cylinder") {
            out->type = kGeomCylinder;
            if (!Float(e, "radius", -1, &out->radius, ctx.c_str()) || !Float(e, "length", -1, &out->length, ctx.c_str())) return false;
            if (out->radius < 0 || out->length < 0) return Fail(ctx + ": cylinder 의 radius/length 가 없거나 음수입니다");
        } else if (n == "sphere") {
            out->type = kGeomSphere;
            if (!Float(e, "radius", -1, &out->radius, ctx.c_str())) return false;
            if (out->radius < 0) return Fail(ctx + ": sphere radius 가 없거나 음수입니다");
        } else if (n == "mesh") {
            out->type = kGeomMesh;
            if (!e->Attribute("filename") || !*e->Attribute("filename")) return Fail(ctx + ": mesh filename 이 없습니다");
            out->filename = e->Attribute("filename");
            if (e->Attribute("scale")) {
                float s3[3];
                if (ParseFloats(e->Attribute("scale"), 3, s3)) {
                    for (int i = 0; i < 3; i++) out->scale[i] = s3[i];
                } else {
                    float s1;
                    if (!ParseFloats(e->Attribute("scale"), 1, &s1)) return Fail(ctx + ": mesh scale 이 올바르지 않습니다");
                    out->scale[0] = out->scale[1] = out->scale[2] = s1;  // 일부 파일은 숫자 하나만 쓴다
                }
            }
        } else {
            return Fail(ctx + ": 지원하지 않는 geometry (" + n + ")");
        }
        return true;
    }

    bool ParseLink(XMLElement* e) {
        Link l;
        if (!e->Attribute("name") || !*e->Attribute("name")) return Fail("name 이 없는 link 가 있습니다");
        l.name = e->Attribute("name");
        const simul::String ctx = "link '" + l.name + "'";

        for (XMLElement* v = e->FirstChildElement("visual"); v; v = v->NextSiblingElement("visual")) {
            Visual vis;
            if (v->Attribute("name")) vis.name = v->Attribute("name");
            if (!Origin(v, &vis.origin, ctx.c_str())) return false;
            XMLElement* g = v->FirstChildElement("geometry");
            if (!g) return Fail(ctx + ": visual 에 geometry 가 없습니다");
            if (!ParseGeometry(g, &vis.geometry, ctx + " visual")) return false;
            XMLElement* m = v->FirstChildElement("material");
            if (m) {
                vis.hasMaterial = true;
                if (!ParseMaterial(m, &vis.material, ctx.c_str())) return false;
            }
            l.visuals.push_back(vis);
        }
        for (XMLElement* v = e->FirstChildElement("collision"); v; v = v->NextSiblingElement("collision")) {
            Collision col;
            if (v->Attribute("name")) col.name = v->Attribute("name");
            if (!Origin(v, &col.origin, ctx.c_str())) return false;
            XMLElement* g = v->FirstChildElement("geometry");
            if (!g) return Fail(ctx + ": collision 에 geometry 가 없습니다");
            if (!ParseGeometry(g, &col.geometry, ctx + " collision")) return false;
            l.collisions.push_back(col);
        }
        XMLElement* in = e->FirstChildElement("inertial");
        if (in) {
            l.hasInertial = true;
            if (!Origin(in, &l.inertial.origin, ctx.c_str())) return false;
            XMLElement* mass = in->FirstChildElement("mass");
            if (mass && !Float(mass, "value", 0, &l.inertial.mass, ctx.c_str())) return false;
            XMLElement* it = in->FirstChildElement("inertia");
            if (it) {
                if (!Float(it, "ixx", 0, &l.inertial.ixx, ctx.c_str()) || !Float(it, "ixy", 0, &l.inertial.ixy, ctx.c_str()) ||
                    !Float(it, "ixz", 0, &l.inertial.ixz, ctx.c_str()) || !Float(it, "iyy", 0, &l.inertial.iyy, ctx.c_str()) ||
                    !Float(it, "iyz", 0, &l.inertial.iyz, ctx.c_str()) || !Float(it, "izz", 0, &l.inertial.izz, ctx.c_str())) return false;
            }
        }
        robot->links.push_back(l);
        return true;
    }

    bool ParseJoint(XMLElement* e) {
        Joint j;
        if (!e->Attribute("name") || !*e->Attribute("name")) return Fail("name 이 없는 joint 가 있습니다");
        j.name = e->Attribute("name");
        const simul::String ctx = "joint '" + j.name + "'";
        const char* t = e->Attribute("type");
        if (!t) return Fail(ctx + ": type 이 없습니다");
        const simul::String ts = t;
        if (ts == "revolute") j.type = kJointRevolute;
        else if (ts == "continuous") j.type = kJointContinuous;
        else if (ts == "prismatic") j.type = kJointPrismatic;
        else if (ts == "fixed") j.type = kJointFixed;
        else if (ts == "floating") j.type = kJointFloating;
        else if (ts == "planar") j.type = kJointPlanar;
        else return Fail(ctx + ": 알 수 없는 type (" + ts + ")");

        XMLElement* p = e->FirstChildElement("parent");
        XMLElement* c = e->FirstChildElement("child");
        if (!p || !p->Attribute("link") || !c || !c->Attribute("link")) return Fail(ctx + ": parent/child link 가 필요합니다");
        j.parent = p->Attribute("link");
        j.child = c->Attribute("link");
        if (!Origin(e, &j.origin, ctx.c_str())) return false;

        XMLElement* ax = e->FirstChildElement("axis");
        if (ax && ax->Attribute("xyz")) {
            float a[3];
            if (!ParseFloats(ax->Attribute("xyz"), 3, a)) return Fail(ctx + ": axis xyz 가 올바르지 않습니다");
            const float len = simul::Sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            if (len < 1e-6f) {
                if (j.IsMovable()) return Fail(ctx + ": axis 의 길이가 0 입니다");
            } else {
                j.axis = dhVector(a[0] / len, a[1] / len, a[2] / len);
            }
        }
        else if (j.IsMovable())
            robot->warnings.push_back(ctx + ": axis 가 없어 z축 (0 0 1) 을 씁니다 (URDF 규격의 기본값은 x축)");
        XMLElement* lim = e->FirstChildElement("limit");
        if (lim) {
            if (!Float(lim, "lower", 0, &j.lower, ctx.c_str()) || !Float(lim, "upper", 0, &j.upper, ctx.c_str()) ||
                !Float(lim, "effort", 0, &j.effort, ctx.c_str()) || !Float(lim, "velocity", 0, &j.velocity, ctx.c_str())) return false;
            j.hasLimit = true;
            if ((j.type == kJointRevolute || j.type == kJointPrismatic) && j.lower > j.upper)
                return Fail(ctx + ": limit 의 lower 가 upper 보다 큽니다");
        } else if (j.type == kJointRevolute || j.type == kJointPrismatic) {
            robot->warnings.push_back(ctx + ": limit 이 없어 제한 없이 움직입니다 (URDF 규격상 필수)");
        }
        XMLElement* dyn = e->FirstChildElement("dynamics");
        if (dyn) {
            if (!Float(dyn, "damping", 0, &j.damping, ctx.c_str()) || !Float(dyn, "friction", 0, &j.friction, ctx.c_str())) return false;
            if (j.damping < 0 || j.friction < 0) return Fail(ctx + ": dynamics 의 damping/friction 은 음수일 수 없습니다");
            j.hasDynamics = true;
        }
        XMLElement* mi = e->FirstChildElement("mimic");
        if (mi) {
            if (!mi->Attribute("joint")) return Fail(ctx + ": mimic 에 joint 가 없습니다");
            j.hasMimic = true;
            j.mimicJoint = mi->Attribute("joint");
            if (!Float(mi, "multiplier", 1, &j.mimicMultiplier, ctx.c_str()) || !Float(mi, "offset", 0, &j.mimicOffset, ctx.c_str())) return false;
        }
        robot->joints.push_back(j);
        return true;
    }

    bool Validate() {
        Robot& r = *robot;
        if (r.links.empty()) return Fail("link 가 하나도 없습니다");
        simul::Set<simul::String> names;
        for (size_t i = 0; i < r.links.size(); i++)
            if (!names.insert(r.links[i].name).second) return Fail("link 이름이 중복됩니다: " + r.links[i].name);
        names.clear();
        for (size_t i = 0; i < r.joints.size(); i++)
            if (!names.insert(r.joints[i].name).second) return Fail("joint 이름이 중복됩니다: " + r.joints[i].name);

        simul::Array<int> parentJoint(r.links.size(), -1);
        simul::Array<simul::Array<int> > childJoints(r.links.size());
        for (size_t i = 0; i < r.joints.size(); i++) {
            const Joint& j = r.joints[i];
            const int p = r.FindLink(j.parent), c = r.FindLink(j.child);
            if (p < 0) return Fail("joint '" + j.name + "' 의 parent link 가 없습니다: " + j.parent);
            if (c < 0) return Fail("joint '" + j.name + "' 의 child link 가 없습니다: " + j.child);
            if (p == c) return Fail("joint '" + j.name + "' 의 parent 와 child 가 같습니다");
            if (parentJoint[static_cast<size_t>(c)] >= 0) return Fail("link '" + j.child + "' 의 부모 joint 가 둘 이상입니다");
            parentJoint[static_cast<size_t>(c)] = static_cast<int>(i);
            childJoints[static_cast<size_t>(p)].push_back(static_cast<int>(i));
            if (j.hasMimic && r.FindJoint(j.mimicJoint) < 0) return Fail("joint '" + j.name + "' 이 mimic 하는 joint 가 없습니다: " + j.mimicJoint);
        }
        simul::Array<int> roots;
        for (size_t i = 0; i < r.links.size(); i++) if (parentJoint[i] < 0) roots.push_back(static_cast<int>(i));
        if (roots.empty()) return Fail("루트 link 가 없습니다 (순환 구조)");
        if (roots.size() > 1) return Fail("루트 link 가 둘 이상입니다 (연결되지 않은 link: " + r.links[static_cast<size_t>(roots[1])].name + ")");
        r.root = roots[0];

        // 루트에서 너비 우선으로 훑어 순서를 정한다. 방문하지 못한 joint 가 있으면 순환이다.
        r.order.clear();
        simul::Array<int> queue(1, r.root);
        for (size_t qi = 0; qi < queue.size(); qi++) {
            const simul::Array<int>& cj = childJoints[static_cast<size_t>(queue[qi])];
            for (size_t k = 0; k < cj.size(); k++) {
                r.order.push_back(cj[k]);
                queue.push_back(r.FindLink(r.joints[static_cast<size_t>(cj[k])].child));
            }
        }
        if (r.order.size() != r.joints.size()) return Fail("joint 구조에 순환이 있어 루트와 연결되지 않은 link 가 있습니다");
        return true;
    }

    // <tag>숫자</tag> 한 개. 없으면 def.
    bool ChildFloat(XMLElement* parent, const char* tag, float def, float* out, const simul::String& ctx) {
        *out = def;
        XMLElement* e = parent ? parent->FirstChildElement(tag) : nullptr;
        if (!e) return true;
        if (!e->GetText() || !ParseFloats(e->GetText(), 1, out)) return Fail(ctx + ": <" + tag + "> 가 올바른 숫자가 아닙니다");
        return true;
    }

    // <scan><horizontal|vertical><samples/min_angle/max_angle>. 개수 samples 는 정수여야 한다.
    bool ScanAxis(XMLElement* axis, int defSamples, int* samples, float* mn, float* mx, const simul::String& ctx) {
        float n = static_cast<float>(defSamples);
        if (!ChildFloat(axis, "samples", n, &n, ctx) || !ChildFloat(axis, "min_angle", *mn, mn, ctx) || !ChildFloat(axis, "max_angle", *mx, mx, ctx)) return false;
        if (!(n >= 1.0f) || n > 100000.0f || n != static_cast<float>(static_cast<int>(n))) return Fail(ctx + ": samples 는 1 이상의 정수여야 합니다");
        *samples = static_cast<int>(n);
        if (*mx < *mn) return Fail(ctx + ": max_angle 이 min_angle 보다 작습니다");
        return true;
    }

    bool ParseGazebo(XMLElement* g) {
        const char* ref = g->Attribute("reference");
        for (XMLElement* s = g->FirstChildElement("sensor"); s; s = s->NextSiblingElement("sensor")) {
            const simul::String type = s->Attribute("type") ? s->Attribute("type") : "";
            if (type != "ray" && type != "gpu_ray" && type != "lidar" && type != "gpu_lidar") continue;  // 카메라 등은 아직 읽지 않는다
            LidarDef d;
            d.name = s->Attribute("name") ? s->Attribute("name") : "";
            const simul::String ctx = "라이다 '" + (d.name.empty() ? simul::String(ref ? ref : "") : d.name) + "'";
            if (!ref || !*ref) return Fail(ctx + ": <gazebo reference=\"링크\"> 가 필요합니다");
            d.link = ref;
            if (d.name.empty()) d.name = d.link;
            XMLElement* pose = s->FirstChildElement("pose");
            if (pose) {
                float v[6];
                if (!pose->GetText() || !ParseFloats(pose->GetText(), 6, v)) return Fail(ctx + ": <pose> 는 숫자 6개 (x y z roll pitch yaw) 여야 합니다");
                d.pose = dhEuler(v[5], v[4], v[3]).GetMat();
                d.pose.v[12] = v[0];
                d.pose.v[13] = v[1];
                d.pose.v[14] = v[2];
            }
            XMLElement* ray = s->FirstChildElement("ray");
            if (!ray) return Fail(ctx + ": <ray> 가 없습니다");
            XMLElement* scan = ray->FirstChildElement("scan");
            if (scan) {
                if (XMLElement* h = scan->FirstChildElement("horizontal"))
                    if (!ScanAxis(h, d.hSamples, &d.hSamples, &d.hMin, &d.hMax, ctx + " horizontal")) return false;
                if (XMLElement* v = scan->FirstChildElement("vertical"))
                    if (!ScanAxis(v, 1, &d.vSamples, &d.vMin, &d.vMax, ctx + " vertical")) return false;
            }
            if (d.hSamples < 2 && d.hMax > d.hMin) return Fail(ctx + ": horizontal samples 가 1 이면 min_angle 과 max_angle 이 같아야 합니다");
            XMLElement* range = ray->FirstChildElement("range");
            if (range && (!ChildFloat(range, "min", d.rangeMin, &d.rangeMin, ctx + " range") || !ChildFloat(range, "max", d.rangeMax, &d.rangeMax, ctx + " range"))) return false;
            if (!(d.rangeMin >= 0.0f) || !(d.rangeMax > d.rangeMin)) return Fail(ctx + ": range 는 0 <= min < max 여야 합니다");
            XMLElement* noise = ray->FirstChildElement("noise");
            if (noise && !ChildFloat(noise, "stddev", 0.0f, &d.noiseStdDev, ctx + " noise")) return false;
            if (d.noiseStdDev < 0.0f) return Fail(ctx + ": noise stddev 가 음수입니다");
            robot->lidars.push_back(d);
        }
        return true;
    }

    bool Parse(const simul::String& xml) {
        if (xml.find("<xacro:") != simul::String::npos || xml.find("${") != simul::String::npos)
            return Fail("xacro 파일은 지원하지 않습니다. 'xacro robot.urdf.xacro > robot.urdf' 로 URDF 로 변환해 주세요");
        XMLDocument doc;
        if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS)
            return Fail(simul::String("XML 파싱 실패: ") + (doc.ErrorStr() ? doc.ErrorStr() : ""));
        XMLElement* root = doc.RootElement();
        if (!root || simul::String(root->Name()) != "robot") return Fail("URDF 파일이 아닙니다 (루트 요소가 robot 이 아님)");
        robot->name = root->Attribute("name") ? root->Attribute("name") : "";

        for (XMLElement* m = root->FirstChildElement("material"); m; m = m->NextSiblingElement("material")) {
            MaterialDef md;
            if (!ParseMaterial(m, &md, "robot material")) return false;
            if (!md.name.empty()) robot->materials.push_back(md);
        }
        for (XMLElement* l = root->FirstChildElement("link"); l; l = l->NextSiblingElement("link"))
            if (!ParseLink(l)) return false;
        for (XMLElement* j = root->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint"))
            if (!ParseJoint(j)) return false;
        for (XMLElement* g = root->FirstChildElement("gazebo"); g; g = g->NextSiblingElement("gazebo"))
            if (!ParseGazebo(g)) return false;

        // 이름만 참조한 visual 재질에 robot 재질의 색을 채운다
        for (size_t li = 0; li < robot->links.size(); li++) {
            for (size_t vi = 0; vi < robot->links[li].visuals.size(); vi++) {
                Visual& v = robot->links[li].visuals[vi];
                if (!v.hasMaterial || v.material.hasColor || v.material.name.empty()) continue;
                for (size_t mi = 0; mi < robot->materials.size(); mi++) {
                    if (robot->materials[mi].name == v.material.name && robot->materials[mi].hasColor) {
                        v.material.color = robot->materials[mi].color;
                        v.material.hasColor = true;
                    }
                }
            }
        }
        if (!Validate()) return false;
        for (size_t i = 0; i < robot->lidars.size(); i++)
            if (robot->FindLink(robot->lidars[i].link) < 0) return Fail("라이다 '" + robot->lidars[i].name + "' 가 붙은 링크가 없습니다: " + robot->lidars[i].link);
        return true;
    }
};

}  // namespace

bool LoadUrdfFromString(const simul::String& xml, Robot* out, simul::String* error, const simul::String& baseDir) {
    if (!out) return false;
    out->Clear();
    if (xml.empty()) {
        detail::SetError(error, "데이터가 비어 있습니다");
        return false;
    }
    Parser p;
    p.robot = out;
    if (!p.Parse(xml)) {
        detail::SetError(error, p.error);
        out->Clear();
        return false;
    }
    out->baseDir = baseDir;
    return true;
}

bool LoadUrdf(const simul::String& path, Robot* out, simul::String* error) {
    if (!out) return false;
    out->Clear();
    simul::Array<char> bytes;
    if (!detail::ReadFileBytes(path, &bytes, error)) return false;
    return LoadUrdfFromString(simul::String(bytes.begin(), bytes.end()), out, error, detail::DirName(path));
}

}  // namespace urdf
}  // namespace simul
