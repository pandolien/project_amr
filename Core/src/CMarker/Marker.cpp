#include "CMarker/Marker.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CBase/File.h"
#include "FileUtil.h"

namespace simul {
namespace marker {

namespace {

#include "ArucoTables.inc"

int Dim(ArucoDict d) { return ArucoBitSize(d); }

String Fmt(const char* f, double a = 0, double b = 0, double c = 0, double d = 0) {
    char buf[256];
    snprintf(buf, sizeof(buf), f, a, b, c, d);
    return String(buf);
}

String Fail(String* error, const String& why) {
    if (error) *error = why;
    return why;
}

// ---- 메시 만들기 ----

void AddQuad(MarkerMesh* m, double x0, double y0, double x1, double y1, double z, bool faceUp) {
    const unsigned base = static_cast<unsigned>(m->positions.size() / 3);
    const double p[4][2] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
    for (int i = 0; i < 4; i++) {
        m->positions.push_back(static_cast<float>(p[i][0]));
        m->positions.push_back(static_cast<float>(p[i][1]));
        m->positions.push_back(static_cast<float>(z));
    }
    // x0 < x1, y0 < y1 일 때 (0,1,2) 는 +z 를 보는 반시계. 아래를 보려면 뒤집는다
    const unsigned t[6] = {0, 1, 2, 0, 2, 3};
    for (int i = 0; i < 6; i++) {
        unsigned k = t[i];
        if (!faceUp) {
            // 순서를 뒤집어 반시계 방향이 -z 쪽이 되게 한다
            static const unsigned flip[6] = {0, 2, 1, 0, 3, 2};
            k = flip[i];
        }
        m->indices.push_back(base + k);
    }
}

// 상자: x [x0, x1], y [y0, y1], z [z0, z1]
void AddBox(MarkerMesh* m, double x0, double y0, double z0, double x1, double y1, double z1) {
    const unsigned base = static_cast<unsigned>(m->positions.size() / 3);
    for (int i = 0; i < 8; i++) {
        m->positions.push_back(static_cast<float>((i & 1) ? x1 : x0));
        m->positions.push_back(static_cast<float>((i & 2) ? y1 : y0));
        m->positions.push_back(static_cast<float>((i & 4) ? z1 : z0));
    }
    // 면 6개 (바깥을 보는 반시계 순서): 정점 번호는 (x 비트) + 2 (y 비트) + 4 (z 비트)
    static const unsigned f[6][4] = {{0, 4, 6, 2},   // -x
                                     {1, 3, 7, 5},   // +x
                                     {0, 1, 5, 4},   // -y
                                     {2, 6, 7, 3},   // +y
                                     {0, 2, 3, 1},   // -z
                                     {4, 5, 7, 6}};  // +z
    for (int s = 0; s < 6; s++) {
        m->indices.push_back(base + f[s][0]);
        m->indices.push_back(base + f[s][1]);
        m->indices.push_back(base + f[s][2]);
        m->indices.push_back(base + f[s][0]);
        m->indices.push_back(base + f[s][2]);
        m->indices.push_back(base + f[s][3]);
    }
}

// 마커 하나(검은 테두리 포함 (bits+2) x (bits+2) 칸)의 검은 칸을 행마다 이어서 사각형으로 만든다.
// 왼쪽 위 모서리 (x0, y0), 칸 크기 cell. yDown 이면 행이 아래(+y)로, 아니면 위(-y)로 가는 것이 아니라 "행 0 이 위쪽 = y 가 큰 쪽" 이다.
void AddMarkerInk(MarkerMesh* ink, ArucoDict dict, int id, double x0, double y0, double cell, bool yDown, double z, bool faceUp) {
    const int n = Dim(dict), cells = n + 2;
    unsigned char bits[49];
    ArucoBits(dict, id, bits);
    for (int r = 0; r < cells; r++) {
        int c = 0;
        while (c < cells) {
            const bool border = r == 0 || r == cells - 1 || c == 0 || c == cells - 1;
            const bool black = border || bits[(r - 1) * n + (c - 1)] == 0;
            if (!black) {
                c++;
                continue;
            }
            int e = c;
            for (;;) {
                if (e + 1 >= cells) break;
                const int cc = e + 1;
                const bool bd = r == 0 || r == cells - 1 || cc == 0 || cc == cells - 1;
                const bool bk = bd || bits[(r - 1) * n + (cc - 1)] == 0;
                if (!bk) break;
                e++;
            }
            const double xa = x0 + c * cell, xb = x0 + (e + 1) * cell;
            double ya, yb;
            if (yDown) {
                ya = y0 + r * cell;
                yb = y0 + (r + 1) * cell;
            } else {
                yb = y0 - r * cell;        // 행 0 이 위(큰 y)
                ya = y0 - (r + 1) * cell;
            }
            AddQuad(ink, xa, ya, xb, yb, z, faceUp);
            c = e + 1;
        }
    }
}

}  // namespace

int ArucoBitSize(ArucoDict dict) { return 4 + static_cast<int>(dict); }

String ArucoDictName(ArucoDict dict) {
    const int n = ArucoBitSize(dict);
    return Fmt("%gx%g", n, n);
}

bool ParseArucoDict(const String& name, ArucoDict* dict) {
    // 첫 "숫자 x 숫자" 를 찾는다 ("4x4", "DICT_5X5_250")
    const char* s = name.c_str();
    for (; *s; s++) {
        if (*s >= '4' && *s <= '7' && (s[1] == 'x' || s[1] == 'X') && s[2] == s[0]) {
            *dict = static_cast<ArucoDict>(*s - '4');
            return true;
        }
    }
    return false;
}

bool ArucoBits(ArucoDict dict, int id, unsigned char* bits) {
    if (id < 0 || id > kMaxMarkerId) return false;
    const int n = Dim(dict), total = n * n, nb = (total + 7) / 8;
    const unsigned char* table = dict == kDict4x4 ? kTable4x4 : dict == kDict5x5 ? kTable5x5 : dict == kDict6x6 ? kTable6x6 : kTable7x7;
    const unsigned char* m = table + id * nb;
    for (int k = 0; k < total; k++) {
        const int byte = k / 8, off = k % 8;
        const int validBits = byte == nb - 1 ? total - 8 * (nb - 1) : 8;   // 마지막 바이트는 모자랄 수 있고 낮은 자리에 채워져 있다
        bits[k] = static_cast<unsigned char>((m[byte] >> (validBits - 1 - off)) & 1);
    }
    return true;
}

bool BuildAruco(ArucoDict dict, int id, double sideLength, const MarkerBuildOptions& options, MarkerParts* out, String* error) {
    *out = MarkerParts();
    if (id < 0 || id > kMaxMarkerId) {
        Fail(error, Fmt("마커 id 는 0 ~ %g 입니다 (받은 값 %g)", kMaxMarkerId, id));
        return false;
    }
    if (!(sideLength > 0.0) || !(options.thickness > 0.0) || !(options.inkLift >= 0.0)) {
        Fail(error, "마커 한 변과 판 두께는 0 보다 커야 합니다");
        return false;
    }
    const int cells = Dim(dict) + 2;
    const double cell = sideLength / cells;
    const double margin = options.margin >= 0.0 ? options.margin : cell;
    const double half = sideLength * 0.5 + margin;
    // 판: z [-두께, 0]. 무늬 면이 z = 0 이고 +z (보는 사람) 쪽을 본다
    AddBox(&out->plate, -half, -half, -options.thickness, half, half, 0.0);
    AddMarkerInk(&out->ink, dict, id, -sideLength * 0.5, sideLength * 0.5, cell, false, options.inkLift, true);
    out->ids.push_back(id);
    out->width = out->height = 2.0 * half;
    return true;
}

bool BuildCharuco(ArucoDict dict, int squaresX, int squaresY, double squareLength, double markerLength, int firstId, const MarkerBuildOptions& options, MarkerParts* out, String* error) {
    *out = MarkerParts();
    if (squaresX < 2 || squaresY < 2 || squaresX > 64 || squaresY > 64) {
        Fail(error, "ChArUco 칸 수는 가로 세로 2 ~ 64 입니다");
        return false;
    }
    if (!(squareLength > 0.0) || !(markerLength > 0.0) || !(markerLength < squareLength)) {
        Fail(error, "ChArUco 는 0 < 마커 한 변 < 칸 길이 여야 합니다");
        return false;
    }
    if (!(options.thickness > 0.0) || !(options.inkLift >= 0.0)) {
        Fail(error, "판 두께는 0 보다 커야 합니다");
        return false;
    }
    const int markers = squaresX * squaresY / 2;
    if (firstId < 0 || firstId + markers - 1 > kMaxMarkerId) {
        Fail(error, Fmt("마커 id 가 0 ~ %g 를 벗어납니다 (마커 %g 개, 첫 id %g)", kMaxMarkerId, markers, firstId));
        return false;
    }
    const double margin = options.margin >= 0.0 ? options.margin : squareLength * 0.5;
    const double W = squaresX * squareLength, H = squaresY * squareLength;
    // 판: z [0, 두께] (보드 안쪽), 무늬 면은 z = 0 이고 -z (보는 사람) 쪽을 본다
    AddBox(&out->plate, -margin, -margin, 0.0, W + margin, H + margin, options.thickness);
    const double lift = -options.inkLift;
    const int cells = Dim(dict) + 2;
    const double cell = markerLength / cells, pad = (squareLength - markerLength) * 0.5;
    int id = firstId;
    for (int y = 0; y < squaresY; y++)
        for (int x = 0; x < squaresX; x++) {
            const double sx = x * squareLength, sy = y * squareLength;
            if ((x + y) % 2 == 0) {
                AddQuad(&out->ink, sx, sy, sx + squareLength, sy + squareLength, lift, false);   // 검은 칸
            } else {
                AddMarkerInk(&out->ink, dict, id, sx + pad, sy + pad, cell, true, lift, false);   // 흰 칸 안의 마커
                out->ids.push_back(id);
                id++;
            }
        }
    out->width = W + 2.0 * margin;
    out->height = H + 2.0 * margin;
    return true;
}

bool WriteMarkerStl(const MarkerMesh& mesh, const String& path, String* error) {
    const size_t tris = mesh.indices.size() / 3;
    Array<unsigned char> buf;
    buf.resize(84 + tris * 50, 0);
    const char* head = "simul marker";
    memcpy(buf.data(), head, strlen(head));
    const unsigned count = static_cast<unsigned>(tris);
    memcpy(buf.data() + 80, &count, 4);   // 리틀 엔디언 (x86/ARM 공통)
    for (size_t t = 0; t < tris; t++) {
        float v[3][3];
        for (int k = 0; k < 3; k++) {
            const unsigned i = mesh.indices[t * 3 + static_cast<size_t>(k)];
            for (int c = 0; c < 3; c++) v[k][c] = mesh.positions[static_cast<size_t>(i) * 3 + static_cast<size_t>(c)];
        }
        float e1[3], e2[3], n[3];
        for (int c = 0; c < 3; c++) {
            e1[c] = v[1][c] - v[0][c];
            e2[c] = v[2][c] - v[0][c];
        }
        n[0] = e1[1] * e2[2] - e1[2] * e2[1];
        n[1] = e1[2] * e2[0] - e1[0] * e2[2];
        n[2] = e1[0] * e2[1] - e1[1] * e2[0];
        float len = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
        if (len > 0.0f) {
            len = 1.0f / static_cast<float>(sqrt(static_cast<double>(len)));
            for (int c = 0; c < 3; c++) n[c] *= len;
        }
        unsigned char* p = buf.data() + 84 + t * 50;
        memcpy(p, n, 12);
        memcpy(p + 12, v, 36);
    }
    if (!WriteFileBytes(path, buf.data(), buf.size())) {
        Fail(error, "파일을 쓸 수 없습니다: " + path);
        return false;
    }
    return true;
}

bool BuildMarkerFromSpec(const String& spec, const MarkerBuildOptions& options, MarkerParts* out, String* name, String* error) {
    // 쉼표로 나눈다
    Array<String> f;
    {
        String cur;
        for (size_t i = 0; i <= spec.size(); i++) {
            if (i == spec.size() || spec[i] == ',') {
                f.push_back(cur);
                cur = String();
            } else if (spec[i] != ' ') {
                cur += String(1, spec[i]);
            }
        }
    }
    auto num = [&](size_t i, double* v) {
        if (i >= f.size() || f[i].empty()) return false;
        char* end = nullptr;
        *v = strtod(f[i].c_str(), &end);
        return end && *end == 0;
    };
    ArucoDict dict;
    if (f.size() < 2 || !ParseArucoDict(f[1], &dict)) {
        Fail(error, "마커 명세는 \"aruco,사전,id,한변(m)\" 또는 \"charuco,사전,가로칸,세로칸,칸길이(m),마커한변(m)[,첫id]\" 입니다 (사전: 4x4 5x5 6x6 7x7)");
        return false;
    }
    double a, b, c, d, e;
    if (f[0] == "aruco") {
        if (f.size() != 4 || !num(2, &a) || !num(3, &b)) {
            Fail(error, "aruco 명세는 \"aruco,사전,id,한변(m)\" 입니다 (예: aruco,4x4,0,0.15)");
            return false;
        }
        if (!BuildAruco(dict, static_cast<int>(a), b, options, out, error)) return false;
        if (name) *name = "aruco_" + ArucoDictName(dict) + Fmt("_id%g", a);
        return true;
    }
    if (f[0] == "charuco") {
        e = 0;
        if ((f.size() != 6 && f.size() != 7) || !num(2, &a) || !num(3, &b) || !num(4, &c) || !num(5, &d) || (f.size() == 7 && !num(6, &e))) {
            Fail(error, "charuco 명세는 \"charuco,사전,가로칸,세로칸,칸길이(m),마커한변(m)[,첫id]\" 입니다 (예: charuco,4x4,5,7,0.04,0.03)");
            return false;
        }
        if (!BuildCharuco(dict, static_cast<int>(a), static_cast<int>(b), c, d, static_cast<int>(e), options, out, error)) return false;
        if (name) *name = "charuco_" + ArucoDictName(dict) + Fmt("_%gx%g", a, b);
        return true;
    }
    Fail(error, "마커 종류는 aruco 또는 charuco 입니다");
    return false;
}

bool WriteMarkerFiles(const MarkerParts& parts, const String& dir, const String& baseName, String* urdfPath, String* error) {
    const String plate = baseName + "_plate.stl", ink = baseName + "_ink.stl";
    if (!WriteMarkerStl(parts.plate, detail::JoinPath(dir, plate), error) || !WriteMarkerStl(parts.ink, detail::JoinPath(dir, ink), error)) return false;
    String urdf;
    urdf += "<?xml version=\"1.0\"?>\n";
    urdf += "<!-- 자동 생성: ArUco/ChArUco 마커 (" + baseName + "). 흰 판과 검은 칸 두 메시. 좌표계는 Core/include/CMarker/Marker.h 참고. 단위 m -->\n";
    urdf += "<robot name=\"" + baseName + "\">\n";
    urdf += "  <material name=\"marker_paper\"><color rgba=\"0.93 0.93 0.91 1\"/></material>\n";
    urdf += "  <material name=\"marker_ink\"><color rgba=\"0.03 0.03 0.03 1\"/></material>\n";
    urdf += "  <link name=\"marker\">\n";
    urdf += "    <visual><geometry><mesh filename=\"" + plate + "\"/></geometry><material name=\"marker_paper\"/></visual>\n";
    urdf += "    <visual><geometry><mesh filename=\"" + ink + "\"/></geometry><material name=\"marker_ink\"/></visual>\n";
    urdf += "  </link>\n";
    urdf += "</robot>\n";
    const String path = detail::JoinPath(dir, baseName + ".urdf");
    if (!WriteFileText(path, urdf)) {
        Fail(error, "파일을 쓸 수 없습니다: " + path);
        return false;
    }
    if (urdfPath) *urdfPath = path;
    return true;
}

}  // namespace marker
}  // namespace simul
