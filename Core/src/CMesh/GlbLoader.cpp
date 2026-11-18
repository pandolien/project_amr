// GLB / glTF 2.0 로더 (cgltf 사용). 렌더링과 무관한 Scene 구조로 변환한다.
#include <string.h>


#include "CMesh/MeshLoader.h"
#include "FileUtil.h"
#include "cgltf.h"

namespace simul {

namespace {

simul::String ResultName(cgltf_result r) {
    switch (r) {
        case cgltf_result_success: return "성공";
        case cgltf_result_data_too_short: return "데이터가 너무 짧습니다 (잘린 파일)";
        case cgltf_result_unknown_format: return "glTF/GLB 형식이 아닙니다";
        case cgltf_result_invalid_json: return "JSON 이 올바르지 않습니다";
        case cgltf_result_invalid_gltf: return "glTF 내용이 올바르지 않습니다";
        case cgltf_result_invalid_options: return "외부 파일 경로를 알 수 없습니다";
        case cgltf_result_file_not_found: return "파일(또는 외부 버퍼)을 찾을 수 없습니다";
        case cgltf_result_io_error: return "읽기 오류";
        case cgltf_result_out_of_memory: return "메모리가 부족합니다";
        case cgltf_result_legacy_gltf: return "glTF 1.0 은 지원하지 않습니다";
        default: return "알 수 없는 오류";
    }
}

simul::String IndexName(const char* prefix, size_t i) {
    return simul::String(prefix) + simul::ToString(i);
}

bool CheckRequiredExtensions(const cgltf_data* d, simul::String* error) {
    for (cgltf_size i = 0; i < d->extensions_required_count; i++) {
        const char* e = d->extensions_required[i];
        if (!e) continue;
        if (strcmp(e, "KHR_draco_mesh_compression") == 0 || strcmp(e, "EXT_meshopt_compression") == 0) {
            detail::SetError(error, simul::String("지원하지 않는 압축 확장입니다: ") + e);
            return false;
        }
    }
    return true;
}

// base64 (data URI 용). 잘못된 글자는 건너뛴다
void DecodeBase64(const char* s, simul::Array<unsigned char>* out) {
    int val = 0, bits = -8;
    for (; *s; s++) {
        const char c = *s;
        int d;
        if (c >= 'A' && c <= 'Z') d = c - 'A';
        else if (c >= 'a' && c <= 'z') d = c - 'a' + 26;
        else if (c >= '0' && c <= '9') d = c - '0' + 52;
        else if (c == '+' || c == '-') d = 62;
        else if (c == '/' || c == '_') d = 63;
        else continue;
        val = (val << 6) | d;
        bits += 6;
        if (bits >= 0) {
            out->push_back(static_cast<unsigned char>((val >> bits) & 0xFF));
            bits -= 8;
        }
    }
}

// 기본 색 텍스처를 재질에 옮긴다: 외부 파일이면 경로(파일 기준으로 풀어서), GLB 안이나 data URI 면 원본 바이트
void ConvertTexture(const cgltf_texture_view& view, const simul::String& baseDir, Material* out) {
    const cgltf_texture* tex = view.texture;
    if (!tex || !tex->image) return;
    const cgltf_image* img = tex->image;
    if (img->buffer_view && img->buffer_view->buffer && img->buffer_view->buffer->data) {
        const unsigned char* p = static_cast<const unsigned char*>(img->buffer_view->buffer->data) + img->buffer_view->offset;
        out->baseColorImage.assign(p, p + img->buffer_view->size);
    } else if (img->uri) {
        if (strncmp(img->uri, "data:", 5) == 0) {
            const char* comma = strchr(img->uri, ',');
            if (comma) DecodeBase64(comma + 1, &out->baseColorImage);
        } else {
            simul::String uri = img->uri;
            simul::Array<char> tmp;
            for (size_t i = 0; i < uri.size(); i++) tmp.push_back(uri[i]);
            tmp.push_back('\0');
            cgltf_decode_uri(&tmp[0]);   // %20 같은 것을 풀어 준다
            uri = &tmp[0];
            const bool absolute = !uri.empty() && (uri[0] == '/' || (uri.size() > 1 && uri[1] == ':'));
            out->baseColorTexture = (absolute || baseDir.empty()) ? uri : baseDir + uri;
        }
    }
}

void ConvertMaterial(const cgltf_material* m, const simul::String& baseDir, Material* out) {
    out->baseColor = Color(1.0f, 1.0f, 1.0f, 1.0f);  // glTF 기본값
    if (m->name) out->name = m->name;
    out->doubleSided = m->double_sided != 0;
    if (m->has_pbr_metallic_roughness) {
        const cgltf_pbr_metallic_roughness& pbr = m->pbr_metallic_roughness;
        out->baseColor = Color(pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2],
                               pbr.base_color_factor[3]);
        out->metallic = pbr.metallic_factor;
        out->roughness = pbr.roughness_factor;
        ConvertTexture(pbr.base_color_texture, baseDir, out);
    } else if (m->has_pbr_specular_glossiness) {
        const cgltf_pbr_specular_glossiness& sg = m->pbr_specular_glossiness;
        out->baseColor = Color(sg.diffuse_factor[0], sg.diffuse_factor[1], sg.diffuse_factor[2], sg.diffuse_factor[3]);
        ConvertTexture(sg.diffuse_texture, baseDir, out);
    }
}

// 반환: 1 = 성공, 0 = 건너뜀, -1 = 오류
int ConvertPrimitive(const cgltf_data* data, const cgltf_primitive* prim, Primitive* out,
                     simul::Array<simul::String>* warnings, simul::String* error) {
    if (prim->type != cgltf_primitive_type_triangles && prim->type != cgltf_primitive_type_triangle_strip &&
        prim->type != cgltf_primitive_type_triangle_fan) {
        warnings->push_back("삼각형이 아닌 primitive(점/선)를 건너뛰었습니다");
        return 0;
    }

    const cgltf_accessor* pos = NULL;
    const cgltf_accessor* nor = NULL;
    const cgltf_accessor* uv = NULL;
    const cgltf_accessor* col = NULL;
    for (cgltf_size i = 0; i < prim->attributes_count; i++) {
        const cgltf_attribute& a = prim->attributes[i];
        if (a.index != 0 || !a.data) continue;
        if (a.type == cgltf_attribute_type_position) pos = a.data;
        else if (a.type == cgltf_attribute_type_normal) nor = a.data;
        else if (a.type == cgltf_attribute_type_texcoord) uv = a.data;
        else if (a.type == cgltf_attribute_type_color) col = a.data;
    }
    if (!pos || pos->count == 0) {
        warnings->push_back("POSITION 이 없거나 비어 있는 primitive 를 건너뛰었습니다");
        return 0;
    }
    if (pos->type != cgltf_type_vec3) {
        detail::SetError(error, "POSITION 접근자의 형식이 VEC3 가 아닙니다");
        return -1;
    }
    if (pos->count > 0xFFFFFFFFu) {
        detail::SetError(error, "정점 수가 너무 많습니다");
        return -1;
    }

    const size_t n = static_cast<size_t>(pos->count);
    out->positions.assign(n * 3, 0.0f);
    if (cgltf_accessor_unpack_floats(pos, &out->positions[0], n * 3) != static_cast<cgltf_size>(n * 3)) {
        detail::SetError(error, "POSITION 데이터를 읽을 수 없습니다");
        return -1;
    }

    if (nor) {
        if (nor->type == cgltf_type_vec3 && nor->count == pos->count) {
            out->normals.assign(n * 3, 0.0f);
            if (cgltf_accessor_unpack_floats(nor, &out->normals[0], n * 3) != static_cast<cgltf_size>(n * 3)) {
                out->normals.clear();
                warnings->push_back("NORMAL 데이터를 읽을 수 없어 법선을 계산했습니다");
            }
        } else {
            warnings->push_back("NORMAL 접근자가 POSITION 과 맞지 않아 법선을 계산했습니다");
        }
    }
    if (uv) {
        if (uv->type == cgltf_type_vec2 && uv->count == pos->count) {
            out->uvs.assign(n * 2, 0.0f);
            if (cgltf_accessor_unpack_floats(uv, &out->uvs[0], n * 2) != static_cast<cgltf_size>(n * 2)) {
                out->uvs.clear();
                warnings->push_back("TEXCOORD_0 데이터를 읽을 수 없어 무시했습니다");
            }
        } else {
            warnings->push_back("TEXCOORD_0 접근자가 POSITION 과 맞지 않아 무시했습니다");
        }
    }

    if (col) {   // 정점 색: vec3 또는 vec4 (정수 형식은 cgltf 가 0..1 로 바꿔 준다)
        if ((col->type == cgltf_type_vec3 || col->type == cgltf_type_vec4) && col->count == pos->count) {
            const cgltf_size comp = col->type == cgltf_type_vec3 ? 3 : 4;
            out->colors.assign(n * 4, 1.0f);
            for (size_t v = 0; v < n; v++) {
                cgltf_float c[4] = {1, 1, 1, 1};
                if (!cgltf_accessor_read_float(col, v, c, comp)) { out->colors.clear(); break; }
                for (int k = 0; k < 4; k++) out->colors[v * 4 + static_cast<size_t>(k)] = c[k];
            }
            if (out->colors.empty()) warnings->push_back("COLOR_0 데이터를 읽을 수 없어 무시했습니다");
        } else {
            warnings->push_back("COLOR_0 접근자 형식이 맞지 않아 무시했습니다");
        }
    }

    // 인덱스 (8/16/32비트 모두 cgltf_accessor_read_index 가 처리한다)
    simul::Array<uint32_t> idx;
    if (prim->indices) {
        const size_t c = static_cast<size_t>(prim->indices->count);
        idx.resize(c);
        for (size_t i = 0; i < c; i++) {
            const size_t v = static_cast<size_t>(cgltf_accessor_read_index(prim->indices, i));
            if (v >= n) {
                detail::SetError(error, "인덱스가 정점 범위를 벗어났습니다");
                return -1;
            }
            idx[i] = static_cast<uint32_t>(v);
        }
    } else {
        idx.resize(n);
        for (size_t i = 0; i < n; i++) idx[i] = static_cast<uint32_t>(i);
    }

    // 스트립/팬을 삼각형 목록으로 바꾼다
    if (prim->type == cgltf_primitive_type_triangles) {
        const size_t usable = idx.size() - idx.size() % 3;
        if (usable != idx.size()) warnings->push_back("삼각형 인덱스 수가 3의 배수가 아니어서 나머지를 버렸습니다");
        out->indices.assign(idx.begin(), idx.begin() + static_cast<ptrdiff_t>(usable));
    } else if (prim->type == cgltf_primitive_type_triangle_strip) {
        for (size_t i = 0; i + 2 < idx.size(); i++) {
            if (i % 2 == 0) {
                out->indices.push_back(idx[i]);
                out->indices.push_back(idx[i + 1]);
            } else {
                out->indices.push_back(idx[i + 1]);
                out->indices.push_back(idx[i]);
            }
            out->indices.push_back(idx[i + 2]);
        }
    } else {  // 팬
        for (size_t i = 1; i + 1 < idx.size(); i++) {
            out->indices.push_back(idx[0]);
            out->indices.push_back(idx[i]);
            out->indices.push_back(idx[i + 1]);
        }
    }
    if (out->indices.empty()) {
        warnings->push_back("삼각형이 없는 primitive 를 건너뛰었습니다");
        return 0;
    }

    if (out->normals.empty()) out->ComputeNormals();
    out->material = prim->material ? static_cast<int>(cgltf_material_index(data, prim->material)) : -1;
    return 1;
}

bool ConvertScene(const cgltf_data* d, const simul::String& baseDir, Scene* out, simul::String* error) {
    out->Clear();

    out->materials.resize(d->materials_count);
    for (size_t i = 0; i < d->materials_count; i++) ConvertMaterial(&d->materials[i], baseDir, &out->materials[i]);

    out->meshes.resize(d->meshes_count);
    for (size_t i = 0; i < d->meshes_count; i++) {
        const cgltf_mesh& cm = d->meshes[i];
        Mesh& mesh = out->meshes[i];
        mesh.name = cm.name ? cm.name : IndexName("mesh", i);
        for (size_t p = 0; p < cm.primitives_count; p++) {
            Primitive prim;
            const int r = ConvertPrimitive(d, &cm.primitives[p], &prim, &out->warnings, error);
            if (r < 0) return false;
            if (r > 0) mesh.primitives.push_back(prim);
        }
    }

    const size_t nn = d->nodes_count;
    out->nodes.resize(nn);
    for (size_t i = 0; i < nn; i++) {
        const cgltf_node& cn = d->nodes[i];
        SceneNode& sn = out->nodes[i];
        sn.name = cn.name ? cn.name : IndexName("node", i);
        cgltf_float m[16];
        cgltf_node_transform_local(&cn, m);  // matrix 또는 TRS 를 열 우선 4x4 로 만든다
        memcpy(sn.local.v, m, sizeof(float) * 16);
        sn.mesh = cn.mesh ? static_cast<int>(cgltf_mesh_index(d, cn.mesh)) : -1;
        sn.parent = cn.parent ? static_cast<int>(cgltf_node_index(d, cn.parent)) : -1;
        for (size_t c = 0; c < cn.children_count; c++) {
            sn.children.push_back(static_cast<int>(cgltf_node_index(d, cn.children[c])));
        }
    }

    // 노드가 없고 메시만 있는 파일: 메시마다 루트 노드를 하나씩 만든다
    if (nn == 0) {
        for (size_t i = 0; i < out->meshes.size(); i++) {
            SceneNode sn;
            sn.name = out->meshes[i].name;
            sn.mesh = static_cast<int>(i);
            out->nodes.push_back(sn);
            out->roots.push_back(static_cast<int>(i));
        }
    } else {
        const cgltf_scene* sc = d->scene ? d->scene : (d->scenes_count > 0 ? &d->scenes[0] : NULL);
        if (sc) {
            if (sc->name) out->name = sc->name;
            for (size_t i = 0; i < sc->nodes_count; i++) {
                out->roots.push_back(static_cast<int>(cgltf_node_index(d, sc->nodes[i])));
            }
        } else {
            out->warnings.push_back("scene 이 없어 부모가 없는 노드를 루트로 사용했습니다");
            for (size_t i = 0; i < nn; i++) {
                if (out->nodes[i].parent == -1) out->roots.push_back(static_cast<int>(i));
            }
        }
    }

    if (!out->IsValid()) {
        detail::SetError(error, "변환된 장면의 구조가 올바르지 않습니다");
        return false;
    }
    return true;
}

// 파싱된 cgltf_data 를 검증하고 변환한다 (data 는 호출한 쪽에서 해제)
bool Finish(cgltf_data* data, const simul::String& baseDir, Scene* out, simul::String* error) {
    const cgltf_result v = cgltf_validate(data);
    if (v != cgltf_result_success) {
        detail::SetError(error, "glTF 검증 실패: " + ResultName(v));
        return false;
    }
    if (!CheckRequiredExtensions(data, error)) return false;
    return ConvertScene(data, baseDir, out, error);
}

}  // namespace

bool LoadGlbFromMemory(const void* bytes, size_t size, Scene* out, simul::String* error) {
    if (!out) return false;
    out->Clear();
    if (!bytes || size == 0) {
        detail::SetError(error, "데이터가 비어 있습니다");
        return false;
    }
    cgltf_options options;
    memset(&options, 0, sizeof(options));
    cgltf_data* data = NULL;
    cgltf_result r = cgltf_parse(&options, bytes, size, &data);
    if (r != cgltf_result_success) {
        detail::SetError(error, "GLB 파싱 실패: " + ResultName(r));
        return false;
    }
    r = cgltf_load_buffers(&options, data, NULL);  // 메모리 입력은 GLB 에 포함된 버퍼와 data URI 만 쓴다
    bool ok = false;
    if (r != cgltf_result_success) detail::SetError(error, "버퍼 로드 실패: " + ResultName(r));
    else ok = Finish(data, simul::String(), out, error);
    cgltf_free(data);
    if (!ok) out->Clear();
    return ok;
}

bool LoadGlb(const simul::String& path, Scene* out, simul::String* error) {
    if (!out) return false;
    out->Clear();
    cgltf_options options;
    memset(&options, 0, sizeof(options));
    cgltf_data* data = NULL;
    cgltf_result r = cgltf_parse_file(&options, path.c_str(), &data);
    if (r != cgltf_result_success) {
        detail::SetError(error, "GLB 읽기 실패 (" + path + "): " + ResultName(r));
        return false;
    }
    r = cgltf_load_buffers(&options, data, path.c_str());  // 외부 .bin 은 이 파일 기준 상대경로
    bool ok = false;
    if (r != cgltf_result_success) detail::SetError(error, "버퍼 로드 실패 (" + path + "): " + ResultName(r));
    else {
        const size_t slash = path.find_last_of("/\\");
        ok = Finish(data, slash == simul::String::npos ? simul::String() : path.substr(0, slash + 1), out, error);
    }
    cgltf_free(data);
    if (!ok) out->Clear();
    return ok;
}

}  // namespace simul
