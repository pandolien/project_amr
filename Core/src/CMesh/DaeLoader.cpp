// COLLADA(.dae) 로더 (tinyxml2). 지원: geometry(triangles/polylist/polygons), visual_scene 노드 계층,
// matrix/translate/rotate/scale, instance_geometry/instance_node/instance_controller(형상만),
// 재질의 diffuse 색상, asset 의 unit / up_axis.
#include "VertexKey.h"
#include "CBase/HashMap.h"
#include "CBase/Text.h"
#include "CBase/Util.h"
#include "CBase/Math.h"
#include "CBase/Map.h"

#include "CMesh/MeshLoader.h"
#include "FileUtil.h"
#include "tinyxml2.h"

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace simul {

namespace {

const int kMaxDepth = 64;

struct Ctx {
    simul::Map<simul::String, XMLElement*> ids;       // id -> 요소
    simul::Map<simul::String, int> materialIndex;     // collada material id -> Scene::materials
    simul::Map<simul::String, int> meshCache;         // geometry + 재질 바인딩 -> Scene::meshes
    Scene* scene;
    simul::String error;
};

typedef simul::Map<simul::String, simul::String> Binding;  // symbol -> material id

void CollectIds(XMLElement* e, simul::Map<simul::String, XMLElement*>* ids, int depth) {
    if (depth > 512) return;
    const char* id = e->Attribute("id");
    if (id && ids->find(id) == ids->end()) (*ids)[id] = e;
    for (XMLElement* c = e->FirstChildElement(); c; c = c->NextSiblingElement()) CollectIds(c, ids, depth + 1);
}

XMLElement* Child(XMLElement* e, const char* name) { return e ? e->FirstChildElement(name) : NULL; }

simul::String UrlId(const char* url) {
    if (!url) return simul::String();
    return url[0] == '#' ? simul::String(url + 1) : simul::String(url);
}

XMLElement* Lookup(Ctx* c, const char* url, const char* expectName) {
    simul::Map<simul::String, XMLElement*>::iterator it = c->ids.find(UrlId(url));
    if (it == c->ids.end()) return NULL;
    if (expectName && simul::String(it->second->Name()) != expectName) return NULL;
    return it->second;
}

bool ReadFloats(const char* text, size_t expected, simul::Array<float>* out) {
    out->clear();
    if (!text) return expected == 0;
    const char* p = text;
    double v;
    while (out->size() < expected && ParseDouble(p, &v, &p)) out->push_back(static_cast<float>(v));
    return out->size() == expected;
}

bool ReadInts(const char* text, simul::Array<int>* out) {
    out->clear();
    if (!text) return true;
    const char* p = text;
    long v;
    while (ParseInt(p, &v, &p)) {
        if (v < 0 || v > 0x7FFFFFFF) return false;
        out->push_back(static_cast<int>(v));
    }
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return *p == '\0';  // 숫자가 아닌 것이 남아 있으면 실패
}

struct Source {
    simul::Array<float> data;
    int stride;
    int offset;
    int count;
    Source() : stride(0), offset(0), count(0) {}
};

bool LoadSource(Ctx* c, XMLElement* src, Source* out) {
    if (!src) return false;
    XMLElement* arr = Child(src, "float_array");
    XMLElement* acc = Child(Child(src, "technique_common"), "accessor");
    if (!arr || !acc) return false;
    const int n = arr->IntAttribute("count", -1);
    out->stride = acc->IntAttribute("stride", 1);
    out->offset = acc->IntAttribute("offset", 0);
    out->count = acc->IntAttribute("count", -1);
    if (n < 0 || out->stride < 1 || out->offset < 0 || out->count < 0 || n > 100000000) return false;
    if (!ReadFloats(arr->GetText(), static_cast<size_t>(n), &out->data)) return false;
    // 접근자가 배열 밖을 가리키지 않는지
    if (out->count > 0 &&
        static_cast<long long>(out->count - 1) * out->stride + out->offset + 2 >= static_cast<long long>(n) &&
        out->stride >= 3) return false;
    (void)c;
    return true;
}

struct InputRef {
    int offset;
    Source src;
    bool present;
    InputRef() : offset(0), present(false) {}
};

typedef detail::VertexKey Key;

// <triangles>/<polylist>/<polygons> 하나를 Primitive 로 만든다. 반환: 1 성공, 0 건너뜀, -1 오류
int BuildPrimitive(Ctx* c, XMLElement* prim, Primitive* out) {
    const simul::String kind = prim->Name();
    InputRef vertex, normal, texcoord;
    int stride = 1;
    int bestSet = 1 << 30;
    for (XMLElement* in = prim->FirstChildElement("input"); in; in = in->NextSiblingElement("input")) {
        const char* sem = in->Attribute("semantic");
        if (!sem) continue;
        const int off = in->IntAttribute("offset", 0);
        if (off < 0 || off > 64) { c->error = "COLLADA input offset 이 올바르지 않습니다"; return -1; }
        if (off + 1 > stride) stride = off + 1;
        const simul::String s = sem;
        XMLElement* src = Lookup(c, in->Attribute("source"), NULL);
        if (s == "VERTEX") {
            XMLElement* verts = src;
            if (!verts || simul::String(verts->Name()) != "vertices") { c->error = "VERTEX input 이 vertices 를 가리키지 않습니다"; return -1; }
            for (XMLElement* vi = verts->FirstChildElement("input"); vi; vi = vi->NextSiblingElement("input")) {
                const char* vs = vi->Attribute("semantic");
                if (vs && simul::String(vs) == "POSITION") {
                    if (!LoadSource(c, Lookup(c, vi->Attribute("source"), "source"), &vertex.src) || vertex.src.stride < 3) {
                        c->error = "POSITION 소스를 읽을 수 없습니다";
                        return -1;
                    }
                    vertex.present = true;
                    vertex.offset = off;
                }
            }
        } else if (s == "NORMAL") {
            if (LoadSource(c, src, &normal.src) && normal.src.stride >= 3) { normal.present = true; normal.offset = off; }
        } else if (s == "TEXCOORD") {
            const int set = in->IntAttribute("set", 0);
            if (set < bestSet && LoadSource(c, src, &texcoord.src) && texcoord.src.stride >= 2) {
                texcoord.present = true; texcoord.offset = off; bestSet = set;
            }
        }
    }
    if (!vertex.present) { c->error = "COLLADA primitive 에 VERTEX/POSITION 이 없습니다"; return -1; }

    // 다각형 목록 (각각 정점 튜플 순서열)
    simul::Array<int> flat;
    simul::Array<int> vcount;  // 다각형별 정점 수
    if (kind == "triangles") {
        if (!ReadInts(Child(prim, "p") ? Child(prim, "p")->GetText() : NULL, &flat)) { c->error = "p 배열이 올바르지 않습니다"; return -1; }
        const int cnt = prim->IntAttribute("count", 0);
        if (cnt < 0 || static_cast<long long>(cnt) * 3 * stride > static_cast<long long>(flat.size())) { c->error = "triangles 의 count 가 p 데이터보다 큽니다"; return -1; }
        vcount.assign(static_cast<size_t>(cnt), 3);
    } else if (kind == "polylist") {
        if (!ReadInts(Child(prim, "p") ? Child(prim, "p")->GetText() : NULL, &flat) ||
            !ReadInts(Child(prim, "vcount") ? Child(prim, "vcount")->GetText() : NULL, &vcount)) {
            c->error = "polylist 데이터가 올바르지 않습니다";
            return -1;
        }
    } else {  // polygons: p 가 여러 개, 각각 다각형 하나
        for (XMLElement* p = prim->FirstChildElement("p"); p; p = p->NextSiblingElement("p")) {
            simul::Array<int> one;
            if (!ReadInts(p->GetText(), &one) || one.size() % static_cast<size_t>(stride) != 0) { c->error = "polygons 의 p 가 올바르지 않습니다"; return -1; }
            vcount.push_back(static_cast<int>(one.size() / static_cast<size_t>(stride)));
            flat.insert(flat.end(), one.begin(), one.end());
        }
    }

    simul::HashMap<Key, uint32_t> dedup;
    size_t cursor = 0;
    for (size_t poly = 0; poly < vcount.size(); poly++) {
        const size_t nv = static_cast<size_t>(vcount[poly]);
        if (cursor + nv * static_cast<size_t>(stride) > flat.size()) { c->error = "vcount 합이 p 데이터보다 큽니다"; return -1; }
        simul::Array<uint32_t> ids;
        for (size_t i = 0; i < nv; i++, cursor += static_cast<size_t>(stride)) {
            Key k;
            k[0] = flat[cursor + static_cast<size_t>(vertex.offset)];
            k[1] = normal.present ? flat[cursor + static_cast<size_t>(normal.offset)] : -1;
            k[2] = texcoord.present ? flat[cursor + static_cast<size_t>(texcoord.offset)] : -1;
            if (k[0] >= vertex.src.count || (normal.present && k[1] >= normal.src.count) ||
                (texcoord.present && k[2] >= texcoord.src.count)) {
                c->error = "COLLADA 인덱스가 소스 범위를 벗어났습니다";
                return -1;
            }
            const uint32_t* found = dedup.Find(k);
            if (found) { ids.push_back(*found); continue; }
            const uint32_t id = static_cast<uint32_t>(out->VertexCount());
            dedup.Insert(k, id);
            ids.push_back(id);
            const float* pv = &vertex.src.data[static_cast<size_t>(k[0]) * static_cast<size_t>(vertex.src.stride) + static_cast<size_t>(vertex.src.offset)];
            out->positions.push_back(pv[0]); out->positions.push_back(pv[1]); out->positions.push_back(pv[2]);
            if (normal.present) {
                const float* pn = &normal.src.data[static_cast<size_t>(k[1]) * static_cast<size_t>(normal.src.stride) + static_cast<size_t>(normal.src.offset)];
                out->normals.push_back(pn[0]); out->normals.push_back(pn[1]); out->normals.push_back(pn[2]);
            }
            if (texcoord.present) {
                const float* pt = &texcoord.src.data[static_cast<size_t>(k[2]) * static_cast<size_t>(texcoord.src.stride) + static_cast<size_t>(texcoord.src.offset)];
                out->uvs.push_back(pt[0]); out->uvs.push_back(pt[1]);
            }
        }
        for (size_t i = 1; i + 1 < ids.size(); i++) {  // 볼록 다각형 가정 팬 삼각화
            out->indices.push_back(ids[0]);
            out->indices.push_back(ids[i]);
            out->indices.push_back(ids[i + 1]);
        }
    }
    if (out->indices.empty()) return 0;
    if (out->normals.empty()) out->ComputeNormals();
    return 1;
}

int MaterialFromId(Ctx* c, const simul::String& matId) {
    simul::Map<simul::String, int>::iterator it = c->materialIndex.find(matId);
    if (it != c->materialIndex.end()) return it->second;
    Material m;
    m.name = matId;
    XMLElement* mat = Lookup(c, matId.c_str(), "material");
    if (mat) {
        XMLElement* eff = Lookup(c, Child(mat, "instance_effect") ? Child(mat, "instance_effect")->Attribute("url") : NULL, "effect");
        XMLElement* tech = Child(Child(eff, "profile_COMMON"), "technique");
        const char* shaders[] = {"phong", "lambert", "blinn", "constant"};
        for (size_t s = 0; s < 4 && tech; s++) {
            XMLElement* sh = Child(tech, shaders[s]);
            if (!sh) continue;
            XMLElement* col = Child(Child(sh, "diffuse"), "color");
            if (!col) col = Child(Child(sh, "emission"), "color");
            simul::Array<float> v;
            if (col && ReadFloats(col->GetText(), 4, &v)) m.baseColor = Color(v[0], v[1], v[2], v[3]);
            XMLElement* tr = Child(Child(sh, "transparency"), "float");
            float t;
            if (tr && tr->QueryFloatText(&t) == tinyxml2::XML_SUCCESS && t >= 0.0f && t <= 1.0f) m.baseColor.a *= t;
            break;
        }
    }
    c->scene->materials.push_back(m);
    const int idx = static_cast<int>(c->scene->materials.size() - 1);
    c->materialIndex[matId] = idx;
    return idx;
}

// geometry + 재질 바인딩으로 Scene::meshes 항목을 만든다. 반환: 인덱스, 실패 -1
int MeshForGeometry(Ctx* c, XMLElement* geom, const Binding& binding) {
    simul::String key = geom->Attribute("id") ? geom->Attribute("id") : "";
    for (Binding::const_iterator it = binding.begin(); it != binding.end(); ++it) key += "|" + it->first + "=" + it->second;
    simul::Map<simul::String, int>::iterator cached = c->meshCache.find(key);
    if (cached != c->meshCache.end()) return cached->second;

    XMLElement* meshEl = Child(geom, "mesh");
    if (!meshEl) {
        c->scene->warnings.push_back("mesh 가 아닌 geometry(spline 등)를 건너뛰었습니다");
        return -1;
    }
    Mesh mesh;
    mesh.name = geom->Attribute("name") ? geom->Attribute("name") : (geom->Attribute("id") ? geom->Attribute("id") : "geometry");
    const char* kinds[] = {"triangles", "polylist", "polygons"};
    for (size_t k = 0; k < 3; k++) {
        for (XMLElement* p = meshEl->FirstChildElement(kinds[k]); p; p = p->NextSiblingElement(kinds[k])) {
            Primitive prim;
            const int r = BuildPrimitive(c, p, &prim);
            if (r < 0) return -2;
            if (r == 0) continue;
            const char* sym = p->Attribute("material");
            if (sym) {
                Binding::const_iterator b = binding.find(sym);
                if (b != binding.end()) prim.material = MaterialFromId(c, b->second);
            }
            mesh.primitives.push_back(prim);
        }
    }
    static const char* const kSkipped[4] = {"lines", "linestrips", "trifans", "tristrips"};
    for (int si = 0; si < 4; si++) {
        const char* skip = kSkipped[si];
        if (meshEl->FirstChildElement(skip)) c->scene->warnings.push_back(simul::String("지원하지 않는 요소를 건너뛰었습니다: ") + skip);
    }
    if (mesh.primitives.empty()) return -1;
    c->scene->meshes.push_back(mesh);
    const int idx = static_cast<int>(c->scene->meshes.size() - 1);
    c->meshCache[key] = idx;
    return idx;
}

dhMat Compose(XMLElement* node, Scene* scene) {
    dhMat m;
    for (XMLElement* e = node->FirstChildElement(); e; e = e->NextSiblingElement()) {
        const simul::String n = e->Name();
        simul::Array<float> v;
        if (n == "matrix") {
            if (ReadFloats(e->GetText(), 16, &v)) {
                dhMat t;  // COLLADA 는 행 우선 -> dhMat 은 열 우선
                for (int r = 0; r < 4; r++)
                    for (int col = 0; col < 4; col++) t.v[col * 4 + r] = v[static_cast<size_t>(r * 4 + col)];
                m = m * t;
            }
        } else if (n == "translate") {
            if (ReadFloats(e->GetText(), 3, &v)) m = m * dhMat::Trans(v[0], v[1], v[2]);
        } else if (n == "rotate") {
            if (ReadFloats(e->GetText(), 4, &v)) {
                dhVector axis(v[0], v[1], v[2]);
                const float len = simul::Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                if (len > 1e-8f) {
                    axis = dhVector(v[0] / len, v[1] / len, v[2] / len);
                    m = m * dhMat::Quat(static_cast<float>(dhRad(v[3])), axis);
                }
            }
        } else if (n == "scale") {
            if (ReadFloats(e->GetText(), 3, &v)) m = m * dhMat::Scale(v[0], v[1], v[2]);
        } else if (n == "skew" || n == "lookat") {
            scene->warnings.push_back("지원하지 않는 변환(" + n + ")을 무시했습니다");
        }
    }
    return m;
}

Binding ReadBinding(XMLElement* inst) {
    Binding b;
    XMLElement* tc = Child(Child(inst, "bind_material"), "technique_common");
    for (XMLElement* im = tc ? tc->FirstChildElement("instance_material") : NULL; im; im = im->NextSiblingElement("instance_material")) {
        const char* sym = im->Attribute("symbol");
        const char* target = im->Attribute("target");
        if (sym && target) b[sym] = UrlId(target);
    }
    return b;
}

bool BuildNode(Ctx* c, XMLElement* node, int parent, int depth) {
    if (depth > kMaxDepth) { c->error = "COLLADA 노드 중첩이 너무 깊습니다 (순환 참조?)"; return false; }
    Scene* s = c->scene;
    SceneNode sn;
    sn.name = node->Attribute("name") ? node->Attribute("name") : (node->Attribute("id") ? node->Attribute("id") : "node");
    sn.local = Compose(node, s);
    sn.parent = parent;
    s->nodes.push_back(sn);
    const int self = static_cast<int>(s->nodes.size() - 1);
    if (parent >= 0) s->nodes[static_cast<size_t>(parent)].children.push_back(self);
    else s->roots.push_back(self);

    // 형상 인스턴스: 첫 번째는 이 노드에, 나머지는 단위 변환 자식 노드에 붙인다
    const char* instKinds[] = {"instance_geometry", "instance_controller"};
    for (size_t k = 0; k < 2; k++) {
        for (XMLElement* inst = node->FirstChildElement(instKinds[k]); inst; inst = inst->NextSiblingElement(instKinds[k])) {
            XMLElement* geom = NULL;
            if (k == 0) {
                geom = Lookup(c, inst->Attribute("url"), "geometry");
            } else {
                XMLElement* ctrl = Lookup(c, inst->Attribute("url"), "controller");
                XMLElement* skin = Child(ctrl, "skin");
                geom = skin ? Lookup(c, skin->Attribute("source"), "geometry") : NULL;
                s->warnings.push_back("스킨(controller)은 무시하고 기본 형상만 읽었습니다");
            }
            if (!geom) { s->warnings.push_back("참조한 geometry 를 찾을 수 없어 건너뛰었습니다"); continue; }
            const int mi = MeshForGeometry(c, geom, ReadBinding(inst));
            if (mi == -2) return false;
            if (mi < 0) continue;
            int target = self;
            if (s->nodes[static_cast<size_t>(self)].mesh >= 0) {
                SceneNode extra;
                extra.name = s->nodes[static_cast<size_t>(self)].name + "_geom";
                extra.parent = self;
                s->nodes.push_back(extra);
                target = static_cast<int>(s->nodes.size() - 1);
                s->nodes[static_cast<size_t>(self)].children.push_back(target);
            }
            s->nodes[static_cast<size_t>(target)].mesh = mi;
        }
    }
    for (XMLElement* ch = node->FirstChildElement("node"); ch; ch = ch->NextSiblingElement("node")) {
        if (!BuildNode(c, ch, self, depth + 1)) return false;
    }
    for (XMLElement* in = node->FirstChildElement("instance_node"); in; in = in->NextSiblingElement("instance_node")) {
        XMLElement* target = Lookup(c, in->Attribute("url"), "node");
        if (!target) { s->warnings.push_back("참조한 node 를 찾을 수 없어 건너뛰었습니다"); continue; }
        if (!BuildNode(c, target, self, depth + 1)) return false;
    }
    return true;
}

bool ParseDae(const char* xml, size_t size, Scene* out, simul::String* error) {
    XMLDocument doc;
    if (doc.Parse(xml, size) != tinyxml2::XML_SUCCESS) {
        detail::SetError(error, simul::String("XML 파싱 실패: ") + (doc.ErrorStr() ? doc.ErrorStr() : ""));
        return false;
    }
    XMLElement* root = doc.RootElement();
    if (!root || simul::String(root->Name()) != "COLLADA") {
        detail::SetError(error, "COLLADA 파일이 아닙니다");
        return false;
    }

    Ctx c;
    c.scene = out;
    CollectIds(root, &c.ids, 0);

    float unit = 1.0f;
    simul::String up = "Y_UP";  // COLLADA 기본값
    XMLElement* asset = Child(root, "asset");
    if (asset) {
        XMLElement* u = Child(asset, "unit");
        if (u) {
            const double meter = u->DoubleAttribute("meter", 1.0);
            if (meter > 0.0 && meter < 1e6) unit = static_cast<float>(meter);
        }
        XMLElement* ua = Child(asset, "up_axis");
        if (ua && ua->GetText()) up = ua->GetText();
    }

    XMLElement* vs = NULL;
    XMLElement* scn = Child(root, "scene");
    XMLElement* ivs = Child(scn, "instance_visual_scene");
    if (ivs) vs = Lookup(&c, ivs->Attribute("url"), "visual_scene");
    if (!vs) vs = Child(Child(root, "library_visual_scenes"), "visual_scene");

    if (vs) {
        out->name = vs->Attribute("name") ? vs->Attribute("name") : "dae";
        for (XMLElement* n = vs->FirstChildElement("node"); n; n = n->NextSiblingElement("node")) {
            if (!BuildNode(&c, n, -1, 0)) { detail::SetError(error, c.error); return false; }
        }
    } else {
        // visual_scene 이 없으면 geometry 마다 루트 노드를 만든다
        XMLElement* lib = Child(root, "library_geometries");
        for (XMLElement* g = lib ? lib->FirstChildElement("geometry") : NULL; g; g = g->NextSiblingElement("geometry")) {
            const int mi = MeshForGeometry(&c, g, Binding());
            if (mi == -2) { detail::SetError(error, c.error); return false; }
            if (mi < 0) continue;
            SceneNode sn;
            sn.name = out->meshes[static_cast<size_t>(mi)].name;
            sn.mesh = mi;
            out->nodes.push_back(sn);
            out->roots.push_back(static_cast<int>(out->nodes.size() - 1));
        }
    }
    if (out->TriangleCount() == 0) {
        detail::SetError(error, "COLLADA 에서 삼각형을 찾지 못했습니다");
        return false;
    }

    // 단위(미터) 와 up_axis(Z-up) 정규화
    dhMat fix = dhMat::Scale(unit, unit, unit);
    if (up == "Y_UP") fix = dhMat::RotX(static_cast<float>(dhRad(90.0))) * fix;
    else if (up == "X_UP") fix = dhMat::RotY(static_cast<float>(dhRad(-90.0))) * fix;
    out->ApplyRootTransform(fix);

    if (!out->IsValid()) {
        detail::SetError(error, "변환된 장면의 구조가 올바르지 않습니다");
        return false;
    }
    return true;
}

}  // namespace

bool LoadDaeFromString(const simul::String& xml, Scene* out, simul::String* error) {
    if (!out) return false;
    out->Clear();
    if (xml.empty()) {
        detail::SetError(error, "데이터가 비어 있습니다");
        return false;
    }
    const bool ok = ParseDae(xml.c_str(), xml.size(), out, error);
    if (!ok) out->Clear();
    return ok;
}

bool LoadDae(const simul::String& path, Scene* out, simul::String* error) {
    if (!out) return false;
    out->Clear();
    simul::Array<char> bytes;
    if (!detail::ReadFileBytes(path, &bytes, error)) return false;
    if (bytes.empty()) {
        detail::SetError(error, "빈 파일입니다: " + path);
        return false;
    }
    return LoadDaeFromString(simul::String(bytes.begin(), bytes.end()), out, error);
}

}  // namespace simul
