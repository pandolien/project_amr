#include "CProject/Project.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CBase/File.h"
#include "tinyxml2.h"

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace simul {
namespace project {

namespace {

String Esc(const String& s) {
    String o;
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

String Num(double v) {
    char b[64];
    snprintf(b, sizeof(b), "%.9g", v);
    return String(b);
}

bool Attr(const XMLElement* e, const char* name, String* out) {
    const char* v = e->Attribute(name);
    if (!v) return false;
    *out = v;
    return true;
}

bool NumAttr(const XMLElement* e, const char* name, double* out, String* error) {
    const char* v = e->Attribute(name);
    if (!v) return true;   // 생략하면 그대로
    char* end = nullptr;
    const double d = strtod(v, &end);
    if (!end || end == v || *end != 0 || !(d - d == 0.0)) {
        if (error) *error = String("속성 ") + name + " 가 숫자가 아닙니다: " + v;
        return false;
    }
    *out = d;
    return true;
}

bool BoolAttr(const XMLElement* e, const char* name, bool* out) {
    const char* v = e->Attribute(name);
    if (!v) return true;
    const String s = v;
    if (s == "true" || s == "1") *out = true;
    else if (s == "false" || s == "0") *out = false;
    else return false;
    return true;
}

}  // namespace

bool ParseProject(const String& xml, Project* out, String* error) {
    *out = Project();
    XMLDocument doc;
    if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS) {
        if (error) *error = String("project.simul XML 을 읽지 못했습니다: ") + doc.ErrorStr();
        return false;
    }
    const XMLElement* root = doc.RootElement();
    if (!root || String(root->Name()) != "simul_project") {
        if (error) *error = "루트 요소가 <simul_project> 가 아닙니다";
        return false;
    }
    Attr(root, "name", &out->name);
    for (const XMLElement* e = root->FirstChildElement(); e; e = e->NextSiblingElement()) {
        const String tag = e->Name();
        if (tag == "model") {
            ModelEntry m;
            if (!Attr(e, "file", &m.file) || !SafeRelativePath(m.file)) {
                if (error) *error = "<model file=...> 은 프로젝트 폴더 안의 상대 경로여야 합니다: " + m.file;
                return false;
            }
            if (!NumAttr(e, "x", &m.position[0], error) || !NumAttr(e, "y", &m.position[1], error) || !NumAttr(e, "z", &m.position[2], error) ||
                !NumAttr(e, "roll", &m.rpy[0], error) || !NumAttr(e, "pitch", &m.rpy[1], error) || !NumAttr(e, "yaw", &m.rpy[2], error))
                return false;
            out->models.push_back(m);
        } else if (tag == "script") {
            ScriptEntry s;
            if (!Attr(e, "file", &s.file) || !SafeRelativePath(s.file)) {
                if (error) *error = "<script file=...> 은 프로젝트 폴더 안의 상대 경로여야 합니다: " + s.file;
                return false;
            }
            Attr(e, "objects", &s.objects);
            out->scripts.push_back(s);
        } else if (tag == "python") {
            if (!NumAttr(e, "hz", &out->hz, error)) return false;
            if (!(out->hz >= 1.0 && out->hz <= 1000.0)) {
                if (error) *error = "<python hz> 는 1 ~ 1000 입니다";
                return false;
            }
            if (!BoolAttr(e, "autostart", &out->autostart) || !BoolAttr(e, "editor", &out->showEditor)) {
                if (error) *error = "<python autostart/editor> 는 true 또는 false 입니다";
                return false;
            }
            if (Attr(e, "main", &out->mainPy) && !out->mainPy.empty() && !SafeRelativePath(out->mainPy)) {
                if (error) *error = "<python main> 은 프로젝트 폴더 안의 상대 경로여야 합니다";
                return false;
            }
        }
        // 모르는 요소는 무시한다 (앞으로 항목이 늘어도 옛 프로그램이 열 수 있게)
    }
    return true;
}

String WriteProjectText(const Project& p) {
    String s = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    s += "<simul_project version=\"1\" name=\"" + Esc(p.name) + "\">\n";
    for (size_t i = 0; i < p.models.size(); i++) {
        const ModelEntry& m = p.models[i];
        s += "  <model file=\"" + Esc(m.file) + "\"";
        if (m.Placed()) {
            s += " x=\"" + Num(m.position[0]) + "\" y=\"" + Num(m.position[1]) + "\" z=\"" + Num(m.position[2]) + "\"";
            s += " roll=\"" + Num(m.rpy[0]) + "\" pitch=\"" + Num(m.rpy[1]) + "\" yaw=\"" + Num(m.rpy[2]) + "\"";
        }
        s += "/>\n";
    }
    for (size_t i = 0; i < p.scripts.size(); i++) s += "  <script file=\"" + Esc(p.scripts[i].file) + "\" objects=\"" + Esc(p.scripts[i].objects) + "\"/>\n";
    s += "  <python hz=\"" + Num(p.hz) + "\" autostart=\"" + (p.autostart ? "true" : "false") + "\" editor=\"" + (p.showEditor ? "true" : "false") + "\" main=\"" + Esc(p.mainPy) + "\"/>\n";
    s += "</simul_project>\n";
    return s;
}

bool LoadProject(const String& path, Project* out, String* error) {
    Array<unsigned char> bytes;
    String err;
    if (!ReadAllBytes(path, &bytes, &err)) {
        if (error) *error = "프로젝트 파일을 읽을 수 없습니다: " + path;
        return false;
    }
    return ParseProject(String(reinterpret_cast<const char*>(bytes.data()), bytes.size()), out, error);
}

bool SaveProject(const String& path, const Project& p, String* error) {
    if (!WriteFileText(path, WriteProjectText(p))) {
        if (error) *error = "프로젝트 파일을 쓸 수 없습니다: " + path;
        return false;
    }
    return true;
}

// ---- 패키지 ----

bool SafeRelativePath(const String& path) {
    if (path.empty() || path[0] == '/' || path[0] == '\\') return false;
    if (path.size() >= 2 && path[1] == ':') return false;   // 드라이브 문자
    if (path.find('\\') != String::npos) return false;
    // 각 요소가 비어 있거나 ".." 이면 안 된다
    size_t start = 0;
    for (size_t i = 0; i <= path.size(); i++) {
        if (i == path.size() || path[i] == '/') {
            const String part = path.substr(start, i - start);
            if (part.empty() || part == "..") return false;
            start = i + 1;
        }
    }
    return true;
}

namespace {

void PutU32(Array<unsigned char>* o, unsigned v) {
    for (int i = 0; i < 4; i++) o->push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}
void PutU64(Array<unsigned char>* o, unsigned long long v) {
    for (int i = 0; i < 8; i++) o->push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}
unsigned long long GetU(const unsigned char* p, int n) {
    unsigned long long v = 0;
    for (int i = 0; i < n; i++) v |= static_cast<unsigned long long>(p[i]) << (8 * i);
    return v;
}

}  // namespace

void SerializePackage(const Array<PackedFile>& files, Array<unsigned char>* out) {
    out->clear();
    const char magic[4] = {'S', 'P', 'K', '1'};
    for (int i = 0; i < 4; i++) out->push_back(static_cast<unsigned char>(magic[i]));
    PutU32(out, static_cast<unsigned>(files.size()));
    for (size_t i = 0; i < files.size(); i++) {
        PutU32(out, static_cast<unsigned>(files[i].path.size()));
        for (size_t k = 0; k < files[i].path.size(); k++) out->push_back(static_cast<unsigned char>(files[i].path[k]));
        PutU64(out, static_cast<unsigned long long>(files[i].data.size()));
        out->insert(out->end(), files[i].data.begin(), files[i].data.end());
    }
}

bool ParsePackage(const unsigned char* d, size_t size, Array<PackedFile>* out, String* error) {
    out->clear();
    auto fail = [&](const char* why) {
        if (error) *error = why;
        return false;
    };
    if (size < 8 || memcmp(d, "SPK1", 4) != 0) return fail("패키지 형식이 아닙니다");
    const unsigned long long count = GetU(d + 4, 4);
    size_t pos = 8;
    for (unsigned long long i = 0; i < count; i++) {
        if (pos + 4 > size) return fail("패키지가 잘렸습니다");
        const size_t plen = static_cast<size_t>(GetU(d + pos, 4));
        pos += 4;
        if (plen > size - pos || size - pos - plen < 8) return fail("패키지가 잘렸습니다");
        PackedFile f;
        f.path = String(reinterpret_cast<const char*>(d + pos), plen);
        pos += plen;
        const unsigned long long dlen = GetU(d + pos, 8);
        pos += 8;
        if (dlen > size - pos) return fail("패키지가 잘렸습니다");
        if (!SafeRelativePath(f.path)) return fail("패키지에 안전하지 않은 경로가 있습니다");
        f.data.assign(d + pos, d + pos + static_cast<size_t>(dlen));
        pos += static_cast<size_t>(dlen);
        out->push_back(f);
    }
    if (pos != size) return fail("패키지 끝에 남는 데이터가 있습니다");
    return true;
}

void MakeFooter(size_t payloadSize, unsigned long long checksum, unsigned char out[kFooterSize]) {
    Array<unsigned char> b;
    PutU64(&b, static_cast<unsigned long long>(payloadSize));
    PutU64(&b, checksum);
    const char magic[8] = {'S', 'I', 'M', 'U', 'L', 'P', 'K', '1'};
    for (int i = 0; i < 8; i++) b.push_back(static_cast<unsigned char>(magic[i]));
    memcpy(out, b.data(), kFooterSize);
}

bool ParseFooter(const unsigned char in[kFooterSize], unsigned long long* payloadSize, unsigned long long* checksum) {
    if (memcmp(in + 16, "SIMULPK1", 8) != 0) return false;
    *payloadSize = GetU(in, 8);
    *checksum = GetU(in + 8, 8);
    return true;
}

unsigned long long Checksum(const unsigned char* data, size_t size) {
    unsigned long long h = 1469598103934665603ULL;
    for (size_t i = 0; i < size; i++) {
        h ^= data[i];
        h *= 1099511628211ULL;
    }
    return h;
}

}  // namespace project
}  // namespace simul
