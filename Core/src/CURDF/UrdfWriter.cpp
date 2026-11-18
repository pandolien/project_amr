#include "CURDF/UrdfWriter.h"

#include <math.h>
#include <stdio.h>

#include "CBase/File.h"

namespace simul {
namespace urdf {

namespace {

// 숫자 하나: 유효숫자 7자리, 의미 없는 0 과 "-0" 은 없앤다
simul::String Num(double v) {
    if (fabs(v) < 1e-9) return "0";
    char buf[48];
    snprintf(buf, sizeof(buf), "%.7g", v);
    return buf;
}

simul::String Escape(const simul::String& s) {
    simul::String o;
    for (size_t i = 0; i < s.size(); i++) {
        const char c = s[i];
        if (c == '&') o += "&amp;";
        else if (c == '<') o += "&lt;";
        else if (c == '>') o += "&gt;";
        else if (c == '"') o += "&quot;";
        else o += c;
    }
    return o;
}

bool IsIdentity(const dhMat& m) {
    for (int i = 0; i < 16; i++) {
        const double want = (i % 5 == 0) ? 1.0 : 0.0;   // 열 우선 4x4 의 대각선은 0, 5, 10, 15
        if (fabs(static_cast<double>(m.v[i]) - want) > 1e-7) return false;
    }
    return true;
}

// <origin xyz=".." rpy=".."/> (단위 변환이면 생략)
simul::String Origin(const dhMat& m) {
    if (IsIdentity(m)) return simul::String();
    double rpy[3];
    RpyFromMat(m, rpy);
    simul::String s = "<origin xyz=\"" + Num(m.v[12]) + " " + Num(m.v[13]) + " " + Num(m.v[14]) + "\"";
    if (fabs(rpy[0]) > 1e-9 || fabs(rpy[1]) > 1e-9 || fabs(rpy[2]) > 1e-9) s += " rpy=\"" + Num(rpy[0]) + " " + Num(rpy[1]) + " " + Num(rpy[2]) + "\"";
    return s + "/>";
}

bool IsAbsolute(const simul::String& p) {
    if (p.empty()) return false;
    if (p[0] == '/') return true;
    return p.size() > 2 && p[1] == ':' && (p[2] == '/' || p[2] == '\\');
}

bool HasScheme(const simul::String& p) { return p.find("://") != simul::String::npos; }

simul::String Slashes(simul::String p) {
    for (size_t i = 0; i < p.size(); i++)
        if (p[i] == '\\') p[i] = '/';
    return p;
}

simul::Array<simul::String> Split(const simul::String& p) {
    simul::Array<simul::String> out;
    simul::String cur;
    for (size_t i = 0; i <= p.size(); i++) {
        if (i == p.size() || p[i] == '/') {
            if (cur == "..") {
                if (!out.empty() && out.back() != "..") out.pop_back();
                else out.push_back(cur);
            } else if (!cur.empty() && cur != ".") {
                out.push_back(cur);
            }
            cur.clear();
        } else {
            cur += p[i];
        }
    }
    return out;
}

// abs 를 dir 기준 상대 경로로. 드라이브가 다르면 false.
bool Relative(const simul::String& abs, const simul::String& dir, simul::String* out) {
    const simul::String a = Slashes(abs), d = Slashes(dir);
    const bool drive = a.size() > 1 && a[1] == ':';
    if (drive != (d.size() > 1 && d[1] == ':')) return false;
    if (drive && a[0] != d[0]) return false;
    const simul::Array<simul::String> pa = Split(drive ? a.substr(2) : a), pd = Split(drive ? d.substr(2) : d);
    size_t k = 0;
    while (k < pa.size() && k < pd.size() && pa[k] == pd[k]) k++;
    simul::String r;
    for (size_t i = k; i < pd.size(); i++) r += "../";
    for (size_t i = k; i < pa.size(); i++) {
        r += pa[i];
        if (i + 1 < pa.size()) r += "/";
    }
    *out = r;
    return !r.empty();
}

// 메시 경로: package:// 같은 것은 그대로, 상대 경로는 URDF 폴더 기준 절대 경로로 바꾼 뒤(저장 위치가 다르므로) 다시 상대 경로로 쓴다
simul::String MeshPath(const simul::String& file, const Robot& robot, const simul::String& outDir, const WriteOptions& opt) {
    if (file.empty() || HasScheme(file)) return file;
    simul::String abs = Slashes(file);
    if (!IsAbsolute(abs)) {
        if (robot.baseDir.empty()) return abs;
        abs = Slashes(robot.baseDir) + "/" + abs;
    }
    simul::String rel;
    if (opt.relativeMeshes && !outDir.empty() && Relative(abs, outDir, &rel)) return rel;
    return abs;
}

simul::String Geom(const Geometry& g, const Robot& robot, const simul::String& outDir, const WriteOptions& opt) {
    switch (g.type) {
        case kGeomBox: return "<box size=\"" + Num(g.size[0]) + " " + Num(g.size[1]) + " " + Num(g.size[2]) + "\"/>";
        case kGeomCylinder: return "<cylinder radius=\"" + Num(g.radius) + "\" length=\"" + Num(g.length) + "\"/>";
        case kGeomSphere: return "<sphere radius=\"" + Num(g.radius) + "\"/>";
        case kGeomMesh: {
            simul::String s = "<mesh filename=\"" + Escape(MeshPath(g.filename, robot, outDir, opt)) + "\"";
            if (g.scale[0] != 1.0f || g.scale[1] != 1.0f || g.scale[2] != 1.0f) s += " scale=\"" + Num(g.scale[0]) + " " + Num(g.scale[1]) + " " + Num(g.scale[2]) + "\"";
            return s + "/>";
        }
        default: return simul::String();
    }
}

simul::String ColorText(const Color& c) { return "<color rgba=\"" + Num(c.r) + " " + Num(c.g) + " " + Num(c.b) + " " + Num(c.a) + "\"/>"; }

const char* JointTypeName(JointType t) {
    switch (t) {
        case kJointRevolute: return "revolute";
        case kJointContinuous: return "continuous";
        case kJointPrismatic: return "prismatic";
        case kJointFloating: return "floating";
        case kJointPlanar: return "planar";
        default: return "fixed";
    }
}

}  // namespace

void RpyFromMat(const dhMat& m, double rpy[3]) {
    // 열 우선: m.v[col * 4 + row]. R = Rz(yaw) Ry(pitch) Rx(roll)
    const double r20 = m.v[2];
    if (fabs(r20) < 1.0 - 1e-9) {
        rpy[0] = atan2(static_cast<double>(m.v[6]), static_cast<double>(m.v[10]));
        rpy[1] = asin(-r20);
        rpy[2] = atan2(static_cast<double>(m.v[1]), static_cast<double>(m.v[0]));
    } else {   // pitch = ±90° (짐벌 락): yaw 를 0 으로 두고 roll 로 표현
        const double sp = r20 < 0 ? 1.0 : -1.0;
        rpy[1] = sp * 1.5707963267948966;
        rpy[2] = 0.0;
        rpy[0] = atan2(static_cast<double>(m.v[4]) * sp, static_cast<double>(m.v[5]));
    }
}

simul::String UrdfToString(const Robot& robot, const simul::String& outDir, const WriteOptions& opt) {
    simul::String out;
    simul::String name = !opt.robotName.empty() ? opt.robotName : (!robot.name.empty() ? robot.name : simul::String("robot"));
    out += "<?xml version=\"1.0\"?>\n<robot name=\"" + Escape(name) + "\">\n";

    // 재질: 이름이 있고 색이 있는 것은 한 번만 정의하고 이름으로 부른다 (같은 이름에 색이 다르면 처음 것만 쓰고 나머지는 인라인)
    simul::Array<MaterialDef> defs;
    for (size_t i = 0; i < robot.links.size(); i++)
        for (size_t k = 0; k < robot.links[i].visuals.size(); k++) {
            const Visual& v = robot.links[i].visuals[k];
            if (!v.hasMaterial || v.material.name.empty() || !v.material.hasColor) continue;
            bool have = false;
            for (size_t d = 0; d < defs.size(); d++) have = have || defs[d].name == v.material.name;
            if (!have) defs.push_back(v.material);
        }
    for (size_t d = 0; d < defs.size(); d++) out += "  <material name=\"" + Escape(defs[d].name) + "\">" + ColorText(defs[d].color) + "</material>\n";

    for (size_t i = 0; i < robot.links.size(); i++) {
        const Link& l = robot.links[i];
        out += "\n  <link name=\"" + Escape(l.name) + "\"";
        if (!l.hasInertial && l.visuals.empty() && l.collisions.empty()) {
            out += "/>\n";
            continue;
        }
        out += ">\n";
        if (l.hasInertial) {
            const Inertial& n = l.inertial;
            out += "    <inertial>\n";
            const simul::String o = Origin(n.origin);
            if (!o.empty()) out += "      " + o + "\n";
            out += "      <mass value=\"" + Num(n.mass) + "\"/>\n";
            out += "      <inertia ixx=\"" + Num(n.ixx) + "\" ixy=\"" + Num(n.ixy) + "\" ixz=\"" + Num(n.ixz) + "\" iyy=\"" + Num(n.iyy) + "\" iyz=\"" + Num(n.iyz) + "\" izz=\"" + Num(n.izz) + "\"/>\n";
            out += "    </inertial>\n";
        }
        for (size_t k = 0; k < l.visuals.size(); k++) {
            const Visual& v = l.visuals[k];
            const simul::String g = Geom(v.geometry, robot, outDir, opt);
            if (g.empty()) continue;
            out += "    <visual" + (v.name.empty() ? simul::String() : " name=\"" + Escape(v.name) + "\"") + ">\n";
            const simul::String o = Origin(v.origin);
            if (!o.empty()) out += "      " + o + "\n";
            out += "      <geometry>" + g + "</geometry>\n";
            if (v.hasMaterial) {
                bool defined = false;
                for (size_t d = 0; d < defs.size(); d++) defined = defined || (defs[d].name == v.material.name && v.material.hasColor && defs[d].color.r == v.material.color.r && defs[d].color.g == v.material.color.g && defs[d].color.b == v.material.color.b && defs[d].color.a == v.material.color.a);
                if (defined) out += "      <material name=\"" + Escape(v.material.name) + "\"/>\n";
                else if (v.material.hasColor || !v.material.texture.empty()) {
                    out += "      <material" + (v.material.name.empty() ? simul::String() : " name=\"" + Escape(v.material.name) + "\"") + ">";
                    if (v.material.hasColor) out += ColorText(v.material.color);
                    if (!v.material.texture.empty()) out += "<texture filename=\"" + Escape(v.material.texture) + "\"/>";
                    out += "</material>\n";
                } else if (!v.material.name.empty()) {
                    out += "      <material name=\"" + Escape(v.material.name) + "\"/>\n";
                }
            }
            out += "    </visual>\n";
        }
        for (size_t k = 0; k < l.collisions.size(); k++) {
            const Collision& c = l.collisions[k];
            const simul::String g = Geom(c.geometry, robot, outDir, opt);
            if (g.empty()) continue;
            out += "    <collision" + (c.name.empty() ? simul::String() : " name=\"" + Escape(c.name) + "\"") + ">\n";
            const simul::String o = Origin(c.origin);
            if (!o.empty()) out += "      " + o + "\n";
            out += "      <geometry>" + g + "</geometry>\n";
            out += "    </collision>\n";
        }
        out += "  </link>\n";
    }

    for (size_t i = 0; i < robot.joints.size(); i++) {
        const Joint& j = robot.joints[i];
        out += "\n  <joint name=\"" + Escape(j.name) + "\" type=\"" + JointTypeName(j.type) + "\">\n";
        out += "    <parent link=\"" + Escape(j.parent) + "\"/>\n    <child link=\"" + Escape(j.child) + "\"/>\n";
        const simul::String o = Origin(j.origin);
        if (!o.empty()) out += "    " + o + "\n";
        if (j.type != kJointFixed)   // 축은 항상 명시한다 (축 기본값이 URDF 규격(x)과 이 프로젝트(z)에서 다르다)
            out += "    <axis xyz=\"" + Num(j.axis.x) + " " + Num(j.axis.y) + " " + Num(j.axis.z) + "\"/>\n";
        if (j.hasLimit) out += "    <limit lower=\"" + Num(j.lower) + "\" upper=\"" + Num(j.upper) + "\" effort=\"" + Num(j.effort) + "\" velocity=\"" + Num(j.velocity) + "\"/>\n";
        if (j.hasDynamics || j.damping != 0.0f || j.friction != 0.0f) out += "    <dynamics damping=\"" + Num(j.damping) + "\" friction=\"" + Num(j.friction) + "\"/>\n";
        if (j.hasMimic) {
            out += "    <mimic joint=\"" + Escape(j.mimicJoint) + "\"";
            if (j.mimicMultiplier != 1.0f) out += " multiplier=\"" + Num(j.mimicMultiplier) + "\"";
            if (j.mimicOffset != 0.0f) out += " offset=\"" + Num(j.mimicOffset) + "\"";
            out += "/>\n";
        }
        out += "  </joint>\n";
    }

    for (size_t i = 0; i < robot.lidars.size(); i++) {
        const LidarDef& d = robot.lidars[i];
        out += "\n  <gazebo reference=\"" + Escape(d.link) + "\">\n    <sensor type=\"ray\" name=\"" + Escape(d.name) + "\">\n";
        double rpy[3];
        RpyFromMat(d.pose, rpy);
        out += "      <pose>" + Num(d.pose.v[12]) + " " + Num(d.pose.v[13]) + " " + Num(d.pose.v[14]) + " " + Num(rpy[0]) + " " + Num(rpy[1]) + " " + Num(rpy[2]) + "</pose>\n";
        out += "      <ray>\n        <scan>\n";
        out += "          <horizontal><samples>" + Num(d.hSamples) + "</samples><min_angle>" + Num(d.hMin) + "</min_angle><max_angle>" + Num(d.hMax) + "</max_angle></horizontal>\n";
        if (d.vSamples > 1) out += "          <vertical><samples>" + Num(d.vSamples) + "</samples><min_angle>" + Num(d.vMin) + "</min_angle><max_angle>" + Num(d.vMax) + "</max_angle></vertical>\n";
        out += "        </scan>\n        <range><min>" + Num(d.rangeMin) + "</min><max>" + Num(d.rangeMax) + "</max></range>\n";
        if (d.noiseStdDev > 0.0f) out += "        <noise><stddev>" + Num(d.noiseStdDev) + "</stddev></noise>\n";
        out += "      </ray>\n    </sensor>\n  </gazebo>\n";
    }
    out += "</robot>\n";
    return out;
}

bool SaveUrdf(const Robot& robot, const simul::String& path, simul::String* error, const WriteOptions& opt) {
    simul::String dir = Slashes(path);
    const size_t cut = dir.rfind('/');
    dir = cut == simul::String::npos ? simul::String() : dir.substr(0, cut);
    if (!IsAbsolute(dir) && !dir.empty()) {   // 상대 경로로 저장해도 메시 경로 계산은 절대 경로로 한다 (현재 폴더 모름 -> 그대로 둔다)
        dir.clear();
    }
    const simul::String text = UrdfToString(robot, dir, opt);
    if (!WriteFileText(path, text)) {
        if (error) *error = "파일을 쓸 수 없습니다: " + path;
        return false;
    }
    return true;
}

}  // namespace urdf
}  // namespace simul
