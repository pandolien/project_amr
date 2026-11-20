#include "CKinematics/IK.h"
#include "CoreQt/QtString.h"
#include "CoreQt/ModelViewer.h"

#include "CURDF/UrdfWriter.h"
#include <string.h>

#include "3d/dhEuler.h"

#include <QFileInfo>
#include <QPainter>
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
    "uniform float uSunOn;\n"     // 1 이면 주광을 태양(uSunDir)으로 쓰고 보조광은 끈다. 0 이면 카메라에 고정된 두 빛 (기본)
    "uniform vec3 uSunDir;\n"     // 표면에서 태양으로 가는 방향 (카메라 좌표)
    "uniform vec3 uSunColor;\n"   // 태양 빛의 색 (낮게 뜨면 붉다)
    "uniform float uSunK;\n"      // 주광 세기 배율 (기본 1, 해가 지면 0)
    "uniform float uFillK;\n"     // 보조광 세기 배율 (기본 1, 태양 모드는 0)
    "uniform float uAmb;\n"       // 주변광/하늘 밝기 배율 (기본 1, 밤에는 작다)
    "uniform sampler2D uTex;\n"
    "uniform sampler2D uShadowMap;\n"   // 태양 쪽에서 본 깊이 (RGBA8 에 나눠 담은 값)
    "uniform float uShadowOn;\n"        // 1 이면 그림자를 쓴다
    "uniform mat4 uLightFromView;\n"    // 카메라 좌표 -> 태양 쪽 클립 좌표
    "uniform float uShadowTexel;\n"     // 그림자 맵 한 칸의 월드 크기 (m)
    "uniform float uShadowInv;\n"       // 1 / 그림자 맵 크기
    "uniform float uCatcher;\n"         // 1 이면 바닥 그림자 받이: 그림자인 곳만 반투명 검정으로 그린다 (바닥 모델이 없어도 그림자가 보이게)
    "uniform float uUseTex;\n"   // 1 이면 기본 색 텍스처를 곱한다
    "uniform float uUseVC;\n"    // 1 이면 정점 색을 곱한다
    "uniform int uAlphaMode;\n"  // 0 불투명 (텍스처/정점 색의 알파는 무시), 1 마스크, 2 블렌드
    "uniform float uCutoff;\n"
    "varying vec3 vN;\n"
    "varying vec3 vP;\n"
    "varying vec2 vUV;\n"
    "varying vec4 vVC;\n"
    "const float PI = 3.14159265;\n"
    "\n"
    "float unpackDepth(vec4 c) {\n"
    "    return dot(c, vec4(1.0, 1.0 / 255.0, 1.0 / 65025.0, 1.0 / 16581375.0));\n"
    "}\n"
    // 그림자: 1 = 태양빛을 다 받음, 0 = 완전히 가려짐. 표면을 법선 쪽으로 조금 밀고(비스듬할수록 더), 3x3 칸을 평균해 가장자리를 부드럽게 한다.
    "float shadowFactor(vec3 N, vec3 L) {\n"
    "    if (uShadowOn < 0.5) return 1.0;\n"
    "    float ndl = clamp(dot(N, L), 0.0, 1.0);\n"
    "    vec3 p = vP + N * (uShadowTexel * (0.7 + 2.0 * (1.0 - ndl)));\n"
    "    vec4 q = uLightFromView * vec4(p, 1.0);\n"
    "    vec3 c = q.xyz / q.w * 0.5 + 0.5;\n"
    "    if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;\n"
    "    float bias = 0.0004 + 0.0015 * (1.0 - ndl);\n"
    "    float lit = 0.0;\n"
    "    for (int i = -1; i <= 1; i++) {\n"
    "        for (int j = -1; j <= 1; j++) {\n"
    "            float d = unpackDepth(texture2D(uShadowMap, c.xy + vec2(float(i), float(j)) * uShadowInv));\n"
    "            lit += (c.z - bias <= d) ? 1.0 : 0.0;\n"
    "        }\n"
    "    }\n"
    "    return lit / 9.0;\n"
    "}\n"
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
    "    if (uCatcher > 0.5) {\n"
    "        float s = uShadowOn > 0.5 ? shadowFactor(normalize(uUp), uSunDir) : 1.0;\n"
    "        if (s > 0.999) discard;\n"
    "        gl_FragColor = vec4(0.0, 0.0, 0.0, (1.0 - s) * 0.30);\n"
    "        return;\n"
    "    }\n"
    "    if (uLit < 0.5) { gl_FragColor = uColor; return; }\n"
    "    vec4 base = uColor;\n"                                     // 재질 색 x 정점 색 x 텍스처 (glTF 의 baseColor 규칙)
    "    float baseA = uColor.a;\n"
    "    if (uUseVC > 0.5) base *= vVC;\n"
    "    if (uUseTex > 0.5) base *= texture2D(uTex, vUV);\n"
    "    if (uAlphaMode == 0) base.a = baseA;\n"                    // 불투명: 이미지의 알파는 무시 (안 그러면 알파가 0 인 자리가 구멍으로 보인다)
    "    else if (uAlphaMode == 1) { if (base.a < uCutoff) discard; base.a = 1.0; }\n"   // 마스크: 기준보다 옅은 곳만 뚫는다
    "    vec3 N = normalize(vN);\n"
    "    if (!gl_FrontFacing) N = -N;\n"                     // 양면 렌더링: 뒷면은 법선을 뒤집는다
    "    vec3 V = normalize(-vP);\n"
    "    float sh = uSunOn > 0.5 ? shadowFactor(N, uSunDir) : 1.0;\n"
    "    vec3 col;\n"
    "    if (uShading == 0) {\n"
    // Lambert: 주변광 + cos(N, 빛) 두 개 (카메라 방향 빛, 보조광)
    "        vec3 A = uSunOn > 0.5 ? uSunDir : vec3(0.0, 0.0, 1.0);\n"
    "        float l = 0.30 * uAmb + 0.55 * max(dot(N, A), 0.0) * uSunK * sh\n"
    "                       + 0.30 * max(dot(N, normalize(vec3(-0.5, 0.7, 0.5))), 0.0) * uFillK;\n"
    "        col = base.rgb * min(l, 1.2) * (uSunOn > 0.5 ? mix(vec3(1.0), uSunColor, 0.6) : vec3(1.0));\n"
    "    } else if (uShading == 1) {\n"
    // Blinn-Phong: Lambert + 하이라이트. 거칠기를 광택도로 바꿔 쓴다 (매끈할수록 작고 밝은 점)
    "        vec3 L1 = uSunOn > 0.5 ? uSunDir : normalize(vec3(-0.3, 0.6, 0.75));\n"
    "        vec3 L2 = normalize(vec3(0.6, -0.2, 0.5));\n"
    "        float shin = clamp(2.0 / (uRough * uRough + 0.001) - 1.0, 4.0, 400.0);\n"
    "        float d = 0.25 * uAmb + 0.65 * max(dot(N, L1), 0.0) * uSunK * sh + 0.25 * max(dot(N, L2), 0.0) * uFillK;\n"
    "        float s = pow(max(dot(N, normalize(L1 + V)), 0.0), shin) * 0.45 * step(0.0, dot(N, L1)) * uSunK * sh;\n"
    "        col = base.rgb * min(d, 1.2) * (uSunOn > 0.5 ? mix(vec3(1.0), uSunColor, 0.6) : vec3(1.0)) + vec3(s) * uSunColor;\n"
    "    } else {\n"
    // PBR. 재질 색은 sRGB 이므로 먼저 선형으로 바꾸고, 계산한 뒤 톤매핑과 감마로 화면 값으로 돌려놓는다.
    "        vec3 albedo = pow(base.rgb, vec3(2.2));\n"
    "        float rough = clamp(uRough, 0.05, 1.0);\n"
    "        float metal = clamp(uMetal, 0.0, 1.0);\n"
    "        vec3 F0 = mix(vec3(0.04), albedo, metal);\n"      // 비금속 반사율은 4%, 금속은 자기 색이 곧 반사색
    "        float NdV = max(dot(N, V), 1e-3);\n"
    "        vec3 L1 = uSunOn > 0.5 ? uSunDir : normalize(vec3(-0.3, 0.6, 0.75));\n"
    "        vec3 c = pbrLight(L1, vec3(2.4) * uSunColor * uSunK * sh, N, V, albedo, F0, rough, metal);\n"   // 주광 (태양 모드에서는 태양)
    "        c += pbrLight(normalize(vec3(0.6, -0.2, 0.5)), vec3(0.9, 0.95, 1.1) * uFillK, N, V, albedo, F0, rough, metal);\n"  // 보조광
    // 주변광: 난반사는 법선 쪽 환경색, 정반사는 반사 방향의 환경색 (거칠수록 흐려져 법선 쪽 색과 섞는다)
    "        vec3 R = reflect(-V, N);\n"
    "        vec3 envSpec = mix(envColor(R), envColor(N), rough);\n"
    "        vec3 Fe = F0 + (max(vec3(1.0 - rough), F0) - F0) * pow(1.0 - NdV, 5.0);\n"
    "        c += ((1.0 - metal) * albedo * (1.0 - Fe) * envColor(N) * 0.30 + Fe * envSpec * 0.55) * uAmb;\n"
    "        c = c * (1.0 + c / 16.0) / (1.0 + c);\n"        // 밝은 부분이 하얗게 날아가지 않게 누르는 톤매핑 (Reinhard)
    "        col = pow(c, vec3(1.0 / 2.2));\n"
    "    }\n"
    "    gl_FragColor = vec4(col, base.a);\n"
    "}\n";

// 그림자 맵: 태양 쪽에서 본 깊이를 RGBA8 에 나눠 담는다 (깊이 텍스처 확장 없이 어디서나 된다)
const char* kShadowVertexShader =
    "#version 120\n"
    "attribute vec3 aPos;\n"
    "uniform mat4 uLightMVP;\n"
    "void main() {\n"
    "    gl_Position = uLightMVP * vec4(aPos, 1.0);\n"
    "}\n";

const char* kShadowFragmentShader =
    "#version 120\n"
    "void main() {\n"
    "    float d = clamp(gl_FragCoord.z, 0.0, 0.999999);\n"
    "    vec4 e = fract(d * vec4(1.0, 255.0, 65025.0, 16581375.0));\n"
    "    e -= e.yzww * vec4(1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0, 0.0);\n"
    "    gl_FragColor = e;\n"
    "}\n";

// 하늘: 화면 전체를 덮는 삼각형 하나에 시선 방향별로 색을 계산한다 (하늘색 그라데이션 + 프랙탈 값 잡음 구름 + 태양 빛무리).
// 구름은 높은 곳의 평면에 투영한 잡음이라 수평선 쪽으로 갈수록 촘촘해진다.
const char* kSkyVertexShader =
    "#version 120\n"
    "attribute vec2 aPos;\n"
    "varying vec2 vNdc;\n"
    "void main() {\n"
    "    vNdc = aPos;\n"
    "    gl_Position = vec4(aPos, 1.0, 1.0);\n"
    "}\n";

const char* kSkyFragmentShader =
    "#version 120\n"
    "uniform mat4 uInvVP;\n"       // 클립 -> 월드
    "uniform vec3 uEye;\n"         // 카메라 위치 (월드)
    "uniform vec3 uSkyTop;\n"      // 천정 색
    "uniform vec3 uSkyHorizon;\n"  // 수평선 색
    "uniform vec3 uSunDirW;\n"     // 표면 -> 태양 (월드)
    "uniform vec3 uSunCol;\n"
    "uniform float uSunStrength;\n" // 0..1 해 세기
    "uniform float uDay;\n"         // 0 밤 .. 1 낮
    "varying vec2 vNdc;\n"
    "float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n"
    "float vnoise(vec2 p) {\n"
    "    vec2 i = floor(p);\n"
    "    vec2 f = fract(p);\n"
    "    f = f * f * (3.0 - 2.0 * f);\n"
    "    float a = hash(i), b = hash(i + vec2(1.0, 0.0)), c = hash(i + vec2(0.0, 1.0)), d = hash(i + vec2(1.0, 1.0));\n"
    "    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);\n"
    "}\n"
    "float fbm(vec2 p) {\n"
    "    float v = 0.0;\n"
    "    float a = 0.5;\n"
    "    for (int i = 0; i < 5; i++) {\n"
    "        v += a * vnoise(p);\n"
    "        p = p * 2.03 + vec2(17.0, 9.0);\n"
    "        a *= 0.5;\n"
    "    }\n"
    "    return v;\n"
    "}\n"
    "void main() {\n"
    "    vec4 w = uInvVP * vec4(vNdc, 1.0, 1.0);\n"
    "    vec3 d = normalize(w.xyz / w.w - uEye);\n"
    "    float up = max(d.z, 0.0);\n"
    "    vec3 col = mix(uSkyHorizon, uSkyTop, pow(up, 0.5));\n"
    "    if (d.z > 0.0) {\n"
    "        vec2 p = d.xy / (d.z + 0.12) * 1.1 + vec2(3.0, 7.0);\n"
    "        float f = fbm(p);\n"
    "        float cover = smoothstep(0.38, 0.66, f) * smoothstep(0.0, 0.22, d.z);\n"
    "        float body = smoothstep(0.45, 0.85, fbm(p * 1.7 + vec2(5.0, 2.0)));\n"   // 구름 안쪽이 밝고 가장자리가 어둡다
    "        vec3 cloud = mix(vec3(0.62, 0.66, 0.74), vec3(1.0), body) * (0.30 + 0.70 * uDay);\n"
    "        cloud *= mix(vec3(1.0), uSunCol, 0.65);\n"
    "        col = mix(col, cloud, cover * 0.92);\n"
    "        float s = max(dot(d, uSunDirW), 0.0);\n"
    "        col += uSunCol * (pow(s, 600.0) * 2.0 + pow(s, 24.0) * 0.18) * uSunStrength * (1.0 - cover * 0.8);\n"   // 태양 원반과 빛무리 (구름에 가려진다)
    "    }\n"
    "    gl_FragColor = vec4(col, 1.0);\n"
    "}\n";

const int kShadowMapSize = 2048;

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
    : QOpenGLWidget(parent), shadows_(true), shadowActive_(false), skyOn_(true), sunGuide_(false), sunHover_(false), dragSun_(false), buffersDirty_(false), hasRobot_(false), triangleCount_(0), radius_(1.0f),
      dist_(3.0f), yaw_(-1.0471976f), pitch_(0.43633232f), scaleInfo_(nullptr), scaleInfoVisible_(true), frameStatsVisible_(false), fps_(0.0), paintMs_(0.0), statFrames_(0), statPaintSum_(0.0), showBoxes_(false), showObbs_(false), showGrid_(true), gridCell_(1.0f) {
    setMouseTracking(true);  // 버튼을 누르지 않아도 링 위에 있으면 밝게 보인다
    {   // 기본으로 태양 조명을 켠다 (오늘 날짜 10:00, 서울): 그림자와 하늘이 보이는 모습이 기본이다. 끄려면 SetSunEnabled(false) 또는 --no-sun
        SunSettings s;
        s.enabled = true;
        SetSun(s, nullptr);
    }
    selectedLink_ = -1;
    selectedModel_ = false;
    showFrame_ = true;
    gizmoDirty_ = false;
    dragged_ = false;
    lidarOn_ = false;
    lidarDraw_ = true;
    cameraOn_ = false;
    cameraDraw_ = true;
    cameraLink_ = -1;
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
    modelMinZValid_ = false; floorValid_ = false;
    dragU0_ = 0;
    fallEnabled_ = true;
    gravityZ_ = -9.80665;
    fallVel_ = 0;
    modelSigDirty_ = true;
    floorModel_ = -1;
    floorNx_ = floorNy_ = 0;
    floorMinX_ = floorMinY_ = 0.0f;
    floorCell_ = 1.0f;
    minZ_ = 0;
    minZValid_ = false; modelMinZValid_ = false; floorValid_ = false;
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
    shadowProg_.reset();
    shadowFbo_.reset();
    skyProg_.reset();
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
    minZValid_ = false; modelMinZValid_ = false; floorValid_ = false;
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
    QHash<int, QPair<QString, QImage> > matImage;   // 재질 번호 -> (이미지 이름, 디코딩한 이미지)
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
            if (mat && hasUv) {   // 텍스처 이미지: 재질마다 한 번만 풀고, 같은 이미지는 다음에 다시 합칠 때도 다시 풀지 않는다
                const int mi = prim.material;
                if (!matImage.contains(mi)) {
                    QString key;
                    QImage img;
                    if (!mat->baseColorImage.empty()) {
                        key = QString("mem:%1:%2").arg(mat->baseColorImage.size()).arg(qHashBits(mat->baseColorImage.data(), mat->baseColorImage.size()));
                        if (!imageCache_.contains(key)) {
                            QImage d;
                            d.loadFromData(mat->baseColorImage.data(), static_cast<uint>(mat->baseColorImage.size()));
                            imageCache_.insert(key, d);
                        }
                    } else if (!mat->baseColorTexture.empty()) {
                        key = "file:" + simulqt::ToQ(mat->baseColorTexture);
                        if (!imageCache_.contains(key)) imageCache_.insert(key, QImage(simulqt::ToQ(mat->baseColorTexture)));
                    }
                    if (!key.isEmpty()) {
                        img = imageCache_.value(key);
                        if (img.isNull() && &out == &items_)
                            warnings_ << QString::fromUtf8("텍스처 이미지를 읽을 수 없습니다: ") + (mat->baseColorTexture.empty() ? QString::fromUtf8("(파일 안의 이미지)") : simulqt::ToQ(mat->baseColorTexture));
                    }
                    matImage.insert(mi, qMakePair(key, img));
                }
                it->imageKey = matImage.value(mi).first;
                it->image = matImage.value(mi).second;
            }
            if (mat) { it->alphaMode = mat->alphaMode; it->alphaCutoff = mat->alphaCutoff; }
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

bool ModelViewer::SaveUrdf(const QString& path, QString* error) const {
    if (!hasRobot_) {
        if (error) *error = QString::fromUtf8("저장할 모델이 없습니다");
        return false;
    }
    simul::urdf::WriteOptions opt;
    opt.robotName = robot_.name.empty() || robot_.name == "world" ? simul::String("scene") : robot_.name;
    simul::String err;
    const simul::String p = simulqt::FromQ(path);
    if (!simul::urdf::SaveUrdf(robot_, p, &err, opt)) {
        if (error) *error = simulqt::ToQ(err);
        return false;
    }
    return true;
}

QString ModelViewer::UrdfText() const {
    if (!hasRobot_) return QString();
    simul::urdf::WriteOptions opt;
    opt.robotName = robot_.name.empty() || robot_.name == "world" ? simul::String("scene") : robot_.name;
    return simulqt::ToQ(simul::urdf::UrdfToString(robot_, simul::String(), opt));
}

bool ModelViewer::SaveUrdfFolder(const QString& dir, QString* error, QStringList* warnings, int meshFormat) const {
    if (!hasRobot_) {
        if (error) *error = QString::fromUtf8("저장할 모델이 없습니다");
        return false;
    }
    simul::urdf::FolderSaveResult res;
    simul::urdf::FolderOptions fo;
    fo.glbToStl = meshFormat == 1;
    fo.keepGlb = meshFormat == 2;
    simul::String err;
    if (!simul::urdf::SaveUrdfFolder(robot_, simulqt::FromQ(dir), options_, &err, &res, simul::urdf::WriteOptions(), fo)) {
        if (error) *error = simulqt::ToQ(err);
        return false;
    }
    if (warnings) {
        warnings->clear();
        for (size_t i = 0; i < res.warnings.size(); i++) *warnings << simulqt::ToQ(res.warnings[i]);
    }
    return true;
}

void ModelViewer::SetJointValues(const simul::urdf::JointValues& q) {
    if (!hasRobot_) return;
    q_ = q;
    linkT_ = robot_.LinkTransforms(q);
    minZValid_ = false; modelSigDirty_ = true;   // 모델마다 자세가 바뀌었는지는 syncModelCaches 가 따로 본다
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
    if (world_.Composite()) return selectedLink_ >= 0 && ModelOfRoot(selectedLink_) >= 0 && !world_.IsConnected(static_cast<size_t>(ModelOfRoot(selectedLink_)));  // 둘 이상이면 각 모델의 루트 링크가 이동 손잡이 (관절로 연결된 모델은 부모를 따라가므로 제외)
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

// 모델의 링크마다 월드 변환을 이어 붙인 값. 이 값이 그대로면 모델의 모양과 자리가 그대로다.
void ModelViewer::modelSignature(size_t model, simul::Array<float>* sig) const {
    sig->clear();
    for (size_t i = 0; i < items_.size(); i++) {
        const Item& it = *items_[i];
        if (groupOfLink(it.link) != static_cast<int>(model)) continue;
        dhMat m = it.base;
        if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
        for (int k = 0; k < 16; k++) sig->push_back(m.v[k]);
    }
}

// 바닥/낙하 계산에 쓰는 모델별 캐시를 맞춘다. 자세나 관절이 바뀌어도 바뀐 모델만 다시 계산한다
// (큰 배경 모델을 매 프레임 꼭짓점까지 훑지 않으려는 것이다).
void ModelViewer::syncModelCaches() const {
    const size_t n = world_.size();
    if (!modelMinZValid_) {
        modelMinZ_.assign(n, simul::kFloatMax);
        modelClear_.assign(n, simul::kFloatMax);
        minZOk_.assign(n, false);
        clearOk_.assign(n, false);
        modelSig_.assign(n, simul::Array<float>());
        floorValid_ = false;
        modelMinZValid_ = true;
        modelSigDirty_ = true;
    }
    if (!modelSigDirty_) return;
    modelSigDirty_ = false;
    simul::Array<float> sig;
    bool floorMoved = false;
    for (size_t k = 0; k < n; k++) {
        modelSignature(k, &sig);
        if (sig.size() == modelSig_[k].size() && (sig.empty() || memcmp(&sig[0], &modelSig_[k][0], sig.size() * sizeof(float)) == 0)) continue;
        modelSig_[k] = sig;
        minZOk_[k] = false;
        clearOk_[k] = false;
        if (static_cast<int>(k) == floorModel_) floorMoved = true;
    }
    if (floorMoved) {   // 바닥이 움직였으면 모든 모델의 바닥까지 거리를 다시 구한다
        floorValid_ = false;
        for (size_t k = 0; k < n; k++) clearOk_[k] = false;
    }
}

// 모델에 속한 모든 꼭짓점의 월드 z 가 가장 낮은 값 (자세와 관절 값이 반영된다)
float ModelViewer::modelBottom(int model) const {
    syncModelCaches();
    if (model < 0 || static_cast<size_t>(model) >= modelMinZ_.size()) return 0.0f;
    const size_t k = static_cast<size_t>(model);
    if (!minZOk_[k]) {
        float lo = simul::kFloatMax;
        for (size_t i = 0; i < items_.size(); i++) {
            const Item& it = *items_[i];
            if (groupOfLink(it.link) != model) continue;
            dhMat m = it.base;
            if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
            for (size_t v = 0; v + 5 < it.data.size(); v += 6) {
                float p[3];
                Transform(m, &it.data[v], p);
                lo = simul::Min(lo, p[2]);
            }
        }
        modelMinZ_[k] = lo;
        minZOk_[k] = true;
    }
    return modelMinZ_[k] == simul::kFloatMax ? 0.0f : modelMinZ_[k];
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
        if (world_.IsConnected(k) || (dragModel_ == static_cast<int>(k) && (dragAxis_ >= 0 || dragRing_ >= 0))) {  // 연결된 모델은 부모를 따라간다. 끄는 동안에는 손이 붙잡고 있다
            modelFallVel_[k] = 0.0;
            continue;
        }
        simul::dynamics::FallBody b;
        const bool onFloorModel = floorModel_ >= 0 && floorModel_ != static_cast<int>(k);
        b.height = static_cast<double>(onFloorModel ? modelClearance(static_cast<int>(k)) : modelBottom(static_cast<int>(k)));
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

// ---- 바닥 모델 ----

void ModelViewer::SetFloorModel(int model) {
    if (model < 0 || static_cast<size_t>(model) >= world_.size()) model = -1;
    if (model >= 0) model = world_.RootOf(static_cast<size_t>(model));   // 연결된 모델이면 한 덩어리의 맨 위 모델이 바닥이다
    if (model == floorModel_) return;
    floorModel_ = model;
    floorValid_ = false;
    modelMinZValid_ = false;   // 캐시를 모두 새로 만든다
    for (size_t k = 0; k < modelFallVel_.size(); k++) modelFallVel_[k] = 0.0;
    update();
}

bool ModelViewer::SetFloorByLink(const QString& linkName, QString* error) {
    const int link = robot_.FindLink(simulqt::FromQ(linkName));
    const int model = link >= 0 ? ModelOfLink(link) : -1;
    if (model < 0) {
        if (error) *error = QStringLiteral("바닥으로 지정할 모델을 찾지 못했습니다: ") + linkName;
        return false;
    }
    SetFloorModel(model);
    return true;
}

// 바닥 모델의 위를 향한 삼각형을 월드 좌표로 모으고 xy 격자에 나눠 담는다
void ModelViewer::buildFloor() const {
    floorTris_.clear();
    floorGrid_.clear();
    floorNx_ = floorNy_ = 0;
    floorValid_ = true;
    if (floorModel_ < 0) return;
    float lo[2] = {simul::kFloatMax, simul::kFloatMax}, hi[2] = {-simul::kFloatMax, -simul::kFloatMax};
    for (size_t i = 0; i < items_.size(); i++) {
        const Item& it = *items_[i];
        if (groupOfLink(it.link) != floorModel_) continue;
        dhMat m = it.base;
        if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
        for (size_t t = 0; t + 2 < it.indices.size(); t += 3) {
            float p[3][3];
            for (int k = 0; k < 3; k++) Transform(m, &it.data[static_cast<size_t>(it.indices[t + static_cast<size_t>(k)]) * 6], p[k]);
            const float ex = p[1][0] - p[0][0], ey = p[1][1] - p[0][1], fx = p[2][0] - p[0][0], fy = p[2][1] - p[0][1];
            if (ex * fy - ey * fx <= 1e-12f) continue;   // 위에서 봤을 때 반시계가 아니면(아래를 향하거나 선이면) 윗면이 아니다
            for (int k = 0; k < 3; k++) {
                floorTris_.push_back(p[k][0]);
                floorTris_.push_back(p[k][1]);
                floorTris_.push_back(p[k][2]);
                lo[0] = simul::Min(lo[0], p[k][0]);
                lo[1] = simul::Min(lo[1], p[k][1]);
                hi[0] = simul::Max(hi[0], p[k][0]);
                hi[1] = simul::Max(hi[1], p[k][1]);
            }
        }
    }
    const size_t n = floorTris_.size() / 9;
    if (n == 0) return;
    // 삼각형 수에 맞춰 칸 수를 정한다 (칸마다 삼각형 몇 개)
    const float w = simul::Max(hi[0] - lo[0], 1e-4f), h = simul::Max(hi[1] - lo[1], 1e-4f);
    int cells = static_cast<int>(sqrtf(static_cast<float>(n))) + 1;
    if (cells > 128) cells = 128;
    floorCell_ = simul::Max(w, h) / static_cast<float>(cells);
    floorMinX_ = lo[0];
    floorMinY_ = lo[1];
    floorNx_ = static_cast<int>(w / floorCell_) + 1;
    floorNy_ = static_cast<int>(h / floorCell_) + 1;
    floorGrid_.assign(static_cast<size_t>(floorNx_ * floorNy_), simul::Array<int>());
    for (size_t t = 0; t < n; t++) {
        const float* v = &floorTris_[t * 9];
        const float x0 = simul::Min(v[0], simul::Min(v[3], v[6])), x1 = simul::Max(v[0], simul::Max(v[3], v[6]));
        const float y0 = simul::Min(v[1], simul::Min(v[4], v[7])), y1 = simul::Max(v[1], simul::Max(v[4], v[7]));
        int ix0 = static_cast<int>((x0 - floorMinX_) / floorCell_), ix1 = static_cast<int>((x1 - floorMinX_) / floorCell_);
        int iy0 = static_cast<int>((y0 - floorMinY_) / floorCell_), iy1 = static_cast<int>((y1 - floorMinY_) / floorCell_);
        ix0 = simul::Max(ix0, 0); iy0 = simul::Max(iy0, 0);
        ix1 = simul::Min(ix1, floorNx_ - 1); iy1 = simul::Min(iy1, floorNy_ - 1);
        for (int iy = iy0; iy <= iy1; iy++)
            for (int ix = ix0; ix <= ix1; ix++) floorGrid_[static_cast<size_t>(iy * floorNx_ + ix)].push_back(static_cast<int>(t));
    }
}

bool ModelViewer::FloorHeightAt(double x, double y, double* z) const {
    if (floorModel_ < 0) return false;
    syncModelCaches();
    if (!floorValid_) buildFloor();
    if (floorNx_ <= 0) return false;
    const float fx = static_cast<float>(x), fy = static_cast<float>(y);
    const int ix = static_cast<int>(floorf((fx - floorMinX_) / floorCell_)), iy = static_cast<int>(floorf((fy - floorMinY_) / floorCell_));
    if (ix < 0 || iy < 0 || ix >= floorNx_ || iy >= floorNy_) return false;
    const simul::Array<int>& cell = floorGrid_[static_cast<size_t>(iy * floorNx_ + ix)];
    bool hit = false;
    float best = -simul::kFloatMax;
    for (size_t c = 0; c < cell.size(); c++) {
        const float* v = &floorTris_[static_cast<size_t>(cell[c]) * 9];
        const float d = (v[3] - v[0]) * (v[7] - v[1]) - (v[4] - v[1]) * (v[6] - v[0]);   // 삼각형 xy 넓이의 2배 (윗면만 모았으니 > 0)
        const float a = ((v[3] - fx) * (v[7] - fy) - (v[4] - fy) * (v[6] - fx)) / d;       // 무게중심 좌표
        const float b = ((v[6] - fx) * (v[1] - fy) - (v[7] - fy) * (v[0] - fx)) / d;
        const float g = 1.0f - a - b;
        const float eps = -1e-5f;   // 이웃 삼각형 사이 틈에 빠지지 않게 조금 너그럽게
        if (a < eps || b < eps || g < eps) continue;
        const float h = a * v[2] + b * v[5] + g * v[8];
        if (h > best) best = h;
        hit = true;
    }
    if (hit && z) *z = static_cast<double>(best);
    return hit;
}

// 모델의 꼭짓점마다 (꼭짓점 높이 - 그 아래 바닥 높이) 를 구해 가장 작은 값이 더 내려갈 수 있는 거리다.
// 바닥 모델이 없는 자리는 z=0 평면이 바닥이다.
float ModelViewer::modelClearance(int model) const {
    syncModelCaches();
    if (model < 0 || static_cast<size_t>(model) >= modelClear_.size()) return 0.0f;
    const size_t k = static_cast<size_t>(model);
    if (!clearOk_[k]) {
        if (!floorValid_) buildFloor();
        float lo = simul::kFloatMax;
        for (size_t i = 0; i < items_.size(); i++) {
            const Item& it = *items_[i];
            if (groupOfLink(it.link) != model) continue;
            dhMat m = it.base;
            if (it.link >= 0 && static_cast<size_t>(it.link) < linkT_.size()) m = linkT_[static_cast<size_t>(it.link)] * it.base;
            const size_t verts = it.data.size() / 6;
            const size_t stride = verts > 4000 ? verts / 4000 : 1;   // 아주 큰 메시는 건너뛰며 본다
            for (size_t v = 0; v < verts; v += stride) {
                float p[3];
                Transform(m, &it.data[v * 6], p);
                double fz = 0.0;
                if (!FloorHeightAt(static_cast<double>(p[0]), static_cast<double>(p[1]), &fz)) fz = 0.0;
                lo = simul::Min(lo, p[2] - static_cast<float>(fz));
            }
        }
        modelClear_[k] = lo;
        clearOk_[k] = true;
    }
    return modelClear_[k] == simul::kFloatMax ? 0.0f : modelClear_[k];
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
    return commitAdd(hadOne, error);
}

bool ModelViewer::commitAdd(bool hadOne, QString* error) {
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

int ModelViewer::addPrimitive(const simul::urdf::PrimitiveSpec& spec, const QString& name, const float* pose6, QString* error) {
    simul::urdf::Robot robot;
    simul::String err;
    const simul::String base = name.isEmpty() ? simul::String("shape") : simulqt::FromQ(name);
    if (!simul::urdf::MakePrimitiveRobot(base, spec, &robot, &err)) {
        if (error) *error = simulqt::ToQ(err);
        return -1;
    }
    const bool hadOne = world_.size() == 1 && !world_.Composite();
    if (!world_.AddRobot(base, robot, &err)) {
        if (error) *error = simulqt::ToQ(err);
        return -1;
    }
    if (!commitAdd(hadOne, error)) return -1;
    const int index = static_cast<int>(world_.size()) - 1;
    if (pose6) SetModelPlacement6(index, pose6);
    return index;
}

int ModelViewer::AddBox(const QString& name, double sx, double sy, double sz, const float* pose6, const QColor& color, double mass, QString* error) {
    simul::urdf::PrimitiveSpec s;
    s.type = simul::urdf::kGeomBox;
    s.size[0] = static_cast<float>(sx); s.size[1] = static_cast<float>(sy); s.size[2] = static_cast<float>(sz);
    s.mass = static_cast<float>(mass);
    s.color = simul::Color(static_cast<float>(color.redF()), static_cast<float>(color.greenF()), static_cast<float>(color.blueF()), static_cast<float>(color.alphaF()));
    return addPrimitive(s, name, pose6, error);
}

int ModelViewer::AddCylinder(const QString& name, double radius, double length, const float* pose6, const QColor& color, double mass, QString* error) {
    simul::urdf::PrimitiveSpec s;
    s.type = simul::urdf::kGeomCylinder;
    s.radius = static_cast<float>(radius); s.length = static_cast<float>(length);
    s.mass = static_cast<float>(mass);
    s.color = simul::Color(static_cast<float>(color.redF()), static_cast<float>(color.greenF()), static_cast<float>(color.blueF()), static_cast<float>(color.alphaF()));
    return addPrimitive(s, name, pose6, error);
}

int ModelViewer::AddSphere(const QString& name, double radius, const float* pose6, const QColor& color, double mass, QString* error) {
    simul::urdf::PrimitiveSpec s;
    s.type = simul::urdf::kGeomSphere;
    s.radius = static_cast<float>(radius);
    s.mass = static_cast<float>(mass);
    s.color = simul::Color(static_cast<float>(color.redF()), static_cast<float>(color.greenF()), static_cast<float>(color.blueF()), static_cast<float>(color.alphaF()));
    return addPrimitive(s, name, pose6, error);
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
    imageCache_.clear();
    simul::ClearMeshCache();
    emit modelChanged();
    emit robotChanged();
}

void ModelViewer::RemoveModel(int index) {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return;
    const bool wasComposite = world_.Composite();
    for (size_t k = 0; k < world_.size(); k++) {   // 지우는 모델에 붙어 있던 모델은 제자리에서 world 로 풀어 준다
        if (world_.ParentOf(k) != index) continue;
        const dhMat keep = ModelWorldPose(static_cast<int>(k));
        world_.ClearAttach(k);
        world_.SetPlacement(k, keep);
    }
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
    return showFrame_ && hasRobot_ && selectedLink_ >= 0 && ModelOfRoot(selectedLink_) >= 0 && !world_.IsConnected(static_cast<size_t>(ModelOfRoot(selectedLink_)));
}

void ModelViewer::SetModelPose(int index, const dhMat& pose) {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return;
    if (!world_.Composite() && !ensureComposite()) return;
    world_.SetPlacement(static_cast<size_t>(index), pose);
    const simul::String& root = world_.entry(static_cast<size_t>(index)).rootName;
    for (size_t j = 0; j < robot_.joints.size(); j++) {
        if (robot_.joints[j].child == root) robot_.joints[j].origin = pose;   // world 에 매달린 모델은 월드 자세, 연결된 모델은 부모 링크 기준 origin
    }
    linkT_ = robot_.LinkTransforms(q_);
    minZValid_ = false; modelSigDirty_ = true;   // 모델마다 자세가 바뀌었는지는 syncModelCaches 가 따로 본다
    recomputeCollisions();
    updateBounds();
    update();
    emit modelChanged();
    emit objectMoved();
}

int ModelViewer::groupOfLink(int link) const {
    const int m = world_.EntryOfLink(robot_, link);
    return m < 0 ? -1 : world_.RootOf(static_cast<size_t>(m));
}

dhMat ModelViewer::ModelWorldPose(int model) const {
    if (model < 0 || static_cast<size_t>(model) >= world_.size()) return dhMat();
    if (!world_.Composite()) return dhMat::Trans(objectPos_.x, objectPos_.y, objectPos_.z);
    const int root = robot_.FindLink(world_.entry(static_cast<size_t>(model)).rootName);
    if (root < 0 || static_cast<size_t>(root) >= linkT_.size()) return world_.entry(static_cast<size_t>(model)).placement;
    return linkT_[static_cast<size_t>(root)];
}

bool ModelViewer::ConnectModel(int child, int parentLink, const simul::urdf::AttachSpec& spec, const dhMat* origin, QString* error) {
    auto fail = [&](const QString& m) {
        if (error) *error = m;
        return false;
    };
    if (child < 0 || static_cast<size_t>(child) >= world_.size()) return fail(QStringLiteral("연결할 모델이 올바르지 않습니다"));
    if (parentLink < 0 || static_cast<size_t>(parentLink) >= robot_.links.size()) return fail(QStringLiteral("부모 링크가 올바르지 않습니다"));
    const int pm = ModelOfLink(parentLink);
    if (pm < 0) return fail(QStringLiteral("가상 링크(world)에는 연결할 수 없습니다. 연결을 풀면 world 에 고정됩니다"));
    if (pm == child) return fail(QStringLiteral("같은 모델 안의 링크에는 연결할 수 없습니다"));
    const simul::urdf::WorldEntry& pe = world_.entry(static_cast<size_t>(pm));
    int local = -1;
    for (size_t i = 0; i < pe.linkNames.size(); i++)
        if (pe.linkNames[i] == robot_.links[static_cast<size_t>(parentLink)].name) local = static_cast<int>(i);
    if (local < 0) return fail(QStringLiteral("부모 링크를 모델에서 찾지 못했습니다"));
    simul::urdf::AttachSpec a = spec;
    a.parentModel = pe.name;
    a.parentLink = pe.robot.links[static_cast<size_t>(local)].name;

    // 연결해도 제자리에 있게: 부모 링크 기준으로 본 지금의 루트 링크 자세가 관절 origin 이다
    dhMat org;
    if (origin) {
        org = *origin;
    } else {
        const int root = robot_.FindLink(world_.entry(static_cast<size_t>(child)).rootName);
        if (root < 0 || static_cast<size_t>(root) >= linkT_.size() || static_cast<size_t>(parentLink) >= linkT_.size()) return fail(QStringLiteral("모델의 현재 자세를 알 수 없습니다"));
        org = linkT_[static_cast<size_t>(parentLink)].Inv() * linkT_[static_cast<size_t>(root)];
    }
    simul::String err;
    const simul::urdf::AttachSpec before = world_.entry(static_cast<size_t>(child)).attach;
    const dhMat beforePlacement = world_.entry(static_cast<size_t>(child)).placement;
    if (!world_.SetAttach(static_cast<size_t>(child), a, &err)) return fail(simulqt::ToQ(err));
    world_.SetPlacement(static_cast<size_t>(child), org);
    q_.erase(world_.AttachJointName(static_cast<size_t>(child)));   // 새 관절은 0 에서 시작한다 (같은 이름의 옛 값이 남아 자세가 틀어지지 않게)
    q_.erase(world_.entry(static_cast<size_t>(child)).name + "/" + world_.AttachJointName(static_cast<size_t>(child)));
    QString rerr;
    if (!rebuildWorld(&rerr, true)) {   // 합치지 못하면 되돌린다
        world_.SetAttach(static_cast<size_t>(child), before, nullptr);
        if (!before.connected) world_.ClearAttach(static_cast<size_t>(child));
        world_.SetPlacement(static_cast<size_t>(child), beforePlacement);
        rebuildWorld(nullptr, true);
        return fail(rerr);
    }
    emit structureChanged();
    update();
    return true;
}

bool ModelViewer::DisconnectModel(int child) {
    if (child < 0 || static_cast<size_t>(child) >= world_.size() || !world_.IsConnected(static_cast<size_t>(child))) return false;
    const dhMat keep = ModelWorldPose(child);
    q_.erase(world_.AttachJointName(static_cast<size_t>(child)));
    q_.erase(world_.entry(static_cast<size_t>(child)).name + "/" + world_.AttachJointName(static_cast<size_t>(child)));
    world_.ClearAttach(static_cast<size_t>(child));
    world_.SetPlacement(static_cast<size_t>(child), keep);
    rebuildWorld(nullptr, true);
    emit structureChanged();
    update();
    return true;
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
    refreshScaleInfo();
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
    // 모델을 감싸는 구가 세로 시야각(45 도)의 약 70% 를 차지하도록 거리를 잡는다. (예전에는 거의 꽉 차서 작은 물체도 화면을 가득 채웠고,
    // 가까워서 모든 것이 크게 느껴졌다.) 화면 왼쪽 아래의 길이 표시가 실제 크기를 알려 준다.
    target_ = center_ + QVector3D(objectPos_.x, objectPos_.y, objectPos_.z);   // 모델 하나를 오브젝트 위치로 옮겨 둔 경우도 그 자리를 본다
    dist_ = radius_ * 3.6f;
    yaw_ = -1.0471976f;     // -60 도
    pitch_ = 0.43633232f;   // 25 도
    refreshScaleInfo();
    update();
}

void ModelViewer::SetCamera(const dhVector& target, double distance, double yaw, double pitch) {
    const float kMaxPitch = 89.0f * 3.14159265f / 180.0f;
    target_ = QVector3D(target.x, target.y, target.z);
    dist_ = simul::Max(static_cast<float>(distance), 1e-4f);
    yaw_ = static_cast<float>(yaw);
    pitch_ = simul::Max(-kMaxPitch, simul::Min(kMaxPitch, static_cast<float>(pitch)));
    refreshScaleInfo();
    update();
}

dhPose ModelViewer::CameraPose() const {
    const float cy = simul::Cos(yaw_), sy = simul::Sin(yaw_), cp = simul::Cos(pitch_), sp = simul::Sin(pitch_);
    const QVector3D eye = target_ + dist_ * QVector3D(cp * cy, cp * sy, sp);
    // 앞(x) 이 눈에서 target 쪽이 되도록: 방위각 yaw + 180 도, 고도각 pitch (아래를 보면 +)
    const dhEuler e(yaw_ + 3.14159265f, pitch_, 0.0f);
    return dhPose(dhVector(eye.x(), eye.y(), eye.z()), e.GetQuat());
}

// 길이를 사람이 읽기 좋은 단위(mm, cm, m, km)로
static QString LengthText(double m) {
    const double a = m < 0 ? -m : m;
    if (a < 0.01) return QString::number(m * 1000.0, 'g', 3) + " mm";
    if (a < 1.0) return QString::number(m * 100.0, 'g', 3) + " cm";
    if (a < 1000.0) return QString::number(m, 'g', 3) + " m";
    return QString::number(m / 1000.0, 'g', 3) + " km";
}

void ModelViewer::SetFrameStatsVisible(bool on) {
    frameStatsVisible_ = on;
    refreshScaleInfo();
    update();
}

void ModelViewer::SetScaleInfoVisible(bool on) {
    scaleInfoVisible_ = on;
    refreshScaleInfo();
}

void ModelViewer::refreshScaleInfo() {
    if (!scaleInfo_) {
        scaleInfo_ = new QLabel(this);
        scaleInfo_->setAttribute(Qt::WA_TransparentForMouseEvents);
        scaleInfo_->setStyleSheet("QLabel { color: rgba(230,232,240,200); background: rgba(20,22,28,120); padding: 2px 8px; border-radius: 3px; font-size: 11px; }");
    }
    const bool show = scaleInfoVisible_ && !items_.empty();
    scaleInfo_->setVisible(show);
    if (!show) return;
    QString text = QString::fromUtf8("격자 한 칸 %1  ·  카메라 거리 %2").arg(LengthText(static_cast<double>(gridCell_)), LengthText(static_cast<double>(dist_)));
    if (frameStatsVisible_) text += QString::fromUtf8("  ·  %1 FPS, 그리기 %2 ms").arg(fps_, 0, 'f', 0).arg(paintMs_, 0, 'f', 1);
    if (scaleInfo_->text() != text) scaleInfo_->setText(text);
    scaleInfo_->adjustSize();
    scaleInfo_->move(8, height() - scaleInfo_->height() - 8);
}

bool ModelViewer::SetModelUnitScale(int index, double s) {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return false;
    if (!world_.SetMeshScale(static_cast<size_t>(index), s)) return false;
    QString err;
    const dhMat keep = ModelPose(index);
    if (!rebuildWorld(&err, true)) return false;
    SetModelPose(index, keep);
    return true;
}

bool ModelViewer::SetModelScale(int index, double s, bool massScales) {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return false;
    if (!world_.SetModelScale(static_cast<size_t>(index), s, massScales)) return false;
    QString err;
    const dhMat keep = ModelPose(index);
    if (!rebuildWorld(&err, true)) return false;
    SetModelPose(index, keep);
    return true;
}

double ModelViewer::ModelScale(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return 1.0;
    return world_.ModelScale(static_cast<size_t>(index));
}

double ModelViewer::ModelUnitScale(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return 1.0;
    return world_.MeshScale(static_cast<size_t>(index));
}

void ModelViewer::SetShowGrid(bool show) {
    showGrid_ = show;
    update();
}

QString ModelViewer::Summary() const {
    if (items_.empty()) return QString();
    QString text = QString::fromUtf8("삼각형 %1개, 크기 %2 x %3 x %4 m")
                       .arg(triangleCount_)
                       .arg(static_cast<double>(extent_[0]), 0, 'g', 4)
                       .arg(static_cast<double>(extent_[1]), 0, 'g', 4)
                       .arg(static_cast<double>(extent_[2]), 0, 'g', 4);
    const double big = static_cast<double>(simul::Max(extent_[0], simul::Max(extent_[1], extent_[2])));
    if (big > 200.0) text += QString::fromUtf8(" (200 m 보다 큽니다. 파일이 mm 단위라면 모델 단위 변환으로 0.001 배 하세요)");
    else if (big > 0.0 && big < 0.005) text += QString::fromUtf8(" (5 mm 보다 작습니다. 파일이 m 단위가 아닐 수 있습니다)");
    return text;
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
    const bool linked = prog_->link();
    shaderLog_ = prog_->log();   // 컴파일/링크 경고와 오류 (드라이버마다 문법을 다르게 받아들여서 화면이 비는 원인을 찾는 데 쓴다)
    if (!linked) {
        const QString glInfo = QString::fromUtf8("OpenGL: %1 / %2 / %3").arg(QString::fromLatin1(reinterpret_cast<const char*>(glGetString(GL_VENDOR))))
                                   .arg(QString::fromLatin1(reinterpret_cast<const char*>(glGetString(GL_RENDERER)))).arg(QString::fromLatin1(reinterpret_cast<const char*>(glGetString(GL_VERSION))));
        shaderLog_ = QString::fromUtf8("셰이더를 만들 수 없습니다 (화면이 그려지지 않습니다)\n%1\n%2").arg(glInfo).arg(shaderLog_);
        fprintf(stderr, "%s\n", shaderLog_.toUtf8().constData());
        emit shaderFailed(shaderLog_);
    }
    shadowProg_.reset(new QOpenGLShaderProgram);
    shadowProg_->addShaderFromSourceCode(QOpenGLShader::Vertex, kShadowVertexShader);
    shadowProg_->addShaderFromSourceCode(QOpenGLShader::Fragment, kShadowFragmentShader);
    shadowProg_->bindAttributeLocation("aPos", 0);
    if (!shadowProg_->link()) {   // 그림자만 안 나오고 나머지는 그대로 그린다
        shaderLog_ += QString::fromUtf8("\n그림자 셰이더를 만들 수 없습니다 (그림자 없이 그립니다)\n") + shadowProg_->log();
        fprintf(stderr, "%s\n", shadowProg_->log().toUtf8().constData());
        shadowProg_.reset();
    }
    skyProg_.reset(new QOpenGLShaderProgram);
    skyProg_->addShaderFromSourceCode(QOpenGLShader::Vertex, kSkyVertexShader);
    skyProg_->addShaderFromSourceCode(QOpenGLShader::Fragment, kSkyFragmentShader);
    skyProg_->bindAttributeLocation("aPos", 0);
    if (!skyProg_->link()) {   // 하늘만 단색으로 그린다
        shaderLog_ += QString::fromUtf8("\n하늘 셰이더를 만들 수 없습니다 (단색 하늘)\n") + skyProg_->log();
        fprintf(stderr, "%s\n", skyProg_->log().toUtf8().constData());
        skyProg_.reset();
    }
    buffersDirty_ = !items_.empty();
}

void ModelViewer::resizeGL(int, int) { refreshScaleInfo(); }

void ModelViewer::uploadBuffers() {
    uploadItems(items_);
    buffersDirty_ = false;
}

void ModelViewer::uploadItems(ItemList& list) {
    QHash<QString, QSharedPointer<QOpenGLTexture> > texByKey;
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
        if (!it.image.isNull()) {   // 같은 이미지를 쓰는 primitive 는 GL 텍스처 하나를 나눠 쓴다
            if (!texByKey.contains(it.imageKey)) {
                QSharedPointer<QOpenGLTexture> t(new QOpenGLTexture(it.image.mirrored(), QOpenGLTexture::GenerateMipMaps));   // glTF 의 uv 원점은 왼쪽 위라서 위아래를 뒤집어 올린다
                t->setWrapMode(QOpenGLTexture::Repeat);
                t->setMinMagFilters(QOpenGLTexture::LinearMipMapLinear, QOpenGLTexture::Linear);
                texByKey.insert(it.imageKey, t);
            }
            it.tex = texByKey.value(it.imageKey);
        }
    }
}

QMatrix4x4 ModelViewer::viewMatrix() const {
    const float cy = simul::Cos(yaw_), sy = simul::Sin(yaw_);
    const float cp = simul::Cos(pitch_), sp = simul::Sin(pitch_);
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

bool ModelViewer::SetSun(const SunSettings& settings, QString* error) {
    simul::String err;
    simul::env::SunState st;
    if (!simul::env::ComputeSun(settings.time, settings.site, &st, &err)) {
        if (error) *error = QString::fromUtf8(err.c_str());
        return false;
    }
    sunSettings_ = settings;
    sunState_ = st;
    emit sunChanged(sunState_);
    update();
    return true;
}

bool ModelViewer::SetSunTime(const simul::env::SunTime& time, QString* error) {
    SunSettings s = sunSettings_;
    s.time = time;
    return SetSun(s, error);
}

void ModelViewer::SetSunEnabled(bool on) {
    if (sunSettings_.enabled == on) return;
    SunSettings s = sunSettings_;
    s.enabled = on;
    QString err;
    if (!SetSun(s, &err)) return;   // 시각/위치가 올바르지 않으면 켜지 않는다 (기본값은 항상 올바르다)
}

void ModelViewer::SetSunGuideVisible(bool on) {
    if (sunGuide_ == on) return;
    sunGuide_ = on;
    update();
}

// 표시 기준: 장면 중심의 바닥(z = 0) 위 점, 선 길이 (장면 크기에 맞춘다)
void ModelViewer::sunGuideGeometry(QVector3D* origin, float* length) const {
    *origin = QVector3D(center_.x(), center_.y(), 0.0f);
    *length = simul::Max(radius_ * 1.6f, gridCell_ * 3.0f);
}

// 월드 안의 방위 bearingDeg (북에서 시계 방향) 의 수평 단위 벡터. 월드 +y 의 방위가 northBearingDeg 이다.
static QVector3D BearingDir(double bearingDeg, double northBearingDeg) {
    const double phi = (bearingDeg - northBearingDeg) * 3.14159265358979323846 / 180.0;
    return QVector3D(static_cast<float>(sin(phi)), static_cast<float>(cos(phi)), 0.0f);
}

// 그날의 태양 경로: 같은 날짜/위치에서 10분마다의 방향 (센터에서 길이 L). up[i] = 해가 떠 있는가
void ModelViewer::sunPath(simul::Array<QVector3D>* points, simul::Array<int>* minutes, simul::Array<bool>* up) const {
    QVector3D o;
    float L = 0.0f;
    sunGuideGeometry(&o, &L);
    for (int m = 0; m < 1440; m += 10) {
        simul::env::SunTime t = sunSettings_.time;
        t.hour = m / 60;
        t.minute = m % 60;
        t.second = 0.0;
        simul::env::SunState st;
        if (!simul::env::ComputeSun(t, sunSettings_.site, &st)) continue;
        points->push_back(o + QVector3D(st.direction.x, st.direction.y, st.direction.z) * L);
        minutes->push_back(m);
        up->push_back(st.elevationDeg >= 0.0);
    }
}

bool ModelViewer::sunHit(const QPoint& pos) const {
    if (!sunGuide_ || !sunSettings_.enabled) return false;
    QVector3D o;
    float L = 0.0f;
    sunGuideGeometry(&o, &L);
    const QVector3D tip = o + QVector3D(sunState_.direction.x, sunState_.direction.y, sunState_.direction.z) * L;
    const QVector4D c = (projMatrix() * viewMatrix()) * QVector4D(tip, 1.0f);
    if (c.w() <= 1e-6f) return false;
    const float sx = (c.x() / c.w() * 0.5f + 0.5f) * static_cast<float>(width()), sy = (1.0f - (c.y() / c.w() * 0.5f + 0.5f)) * static_cast<float>(height());
    const float dx = sx - static_cast<float>(pos.x()), dy = sy - static_cast<float>(pos.y());
    return dx * dx + dy * dy <= 18.0f * 18.0f;
}

void ModelViewer::setSunFromScreen(const QPoint& pos) {
    QVector3D o;
    float L = 0.0f;
    sunGuideGeometry(&o, &L);
    const QMatrix4x4 pv = projMatrix() * viewMatrix();
    double bestD = 1e30;
    int best = -1;
    for (int m = 0; m < 1440; m++) {
        simul::env::SunTime t = sunSettings_.time;
        t.hour = m / 60;
        t.minute = m % 60;
        t.second = 0.0;
        simul::env::SunState st;
        if (!simul::env::ComputeSun(t, sunSettings_.site, &st) || st.elevationDeg < -1.0) continue;
        const QVector4D c = pv * QVector4D(o + QVector3D(st.direction.x, st.direction.y, st.direction.z) * L, 1.0f);
        if (c.w() <= 1e-6f) continue;
        const double dx = (c.x() / c.w() * 0.5 + 0.5) * width() - pos.x(), dy = (1.0 - (c.y() / c.w() * 0.5 + 0.5)) * height() - pos.y();
        const double d = dx * dx + dy * dy;
        if (d < bestD) {
            bestD = d;
            best = m;
        }
    }
    if (best < 0) return;
    simul::env::SunTime t = sunSettings_.time;
    t.hour = best / 60;
    t.minute = best % 60;
    t.second = 0.0;
    if (t.hour == sunSettings_.time.hour && t.minute == sunSettings_.time.minute) return;
    SetSunTime(t, nullptr);
}

void ModelViewer::drawSunGuide() {
    if (!sunGuide_ || !sunSettings_.enabled) return;
    QVector3D o;
    float L = 0.0f;
    sunGuideGeometry(&o, &L);
    const QVector3D sd(sunState_.direction.x, sunState_.direction.y, sunState_.direction.z);
    const double nb = sunSettings_.site.northBearingDeg;
    glLineWidth(1.5f);
    // 나침반: 북(붉은색)/동/남/서 눈금 (원은 그리지 않는다)
    {
        for (int k = 0; k < 4; k++) {
            const QVector3D d = BearingDir(90.0 * k, nb);
            const float t[6] = {o.x() + d.x() * L * 0.9f, o.y() + d.y() * L * 0.9f, 0.0f, o.x() + d.x() * L * 1.08f, o.y() + d.y() * L * 1.08f, 0.0f};
            prog_->setUniformValue("uColor", k == 0 ? QVector4D(1.0f, 0.3f, 0.3f, 1.0f) : QVector4D(0.30f, 0.36f, 0.50f, 1.0f));
            prog_->setAttributeArray(0, GL_FLOAT, t, 3);
            glDrawArrays(GL_LINES, 0, 2);
        }
    }
    // 그날의 태양 경로: 뜬 구간을 곡선으로, 정시마다 작은 점(십자). 평소에는 숨기고 태양 표시에 마우스를 올리거나 끄는 동안에만 보인다.
    if (dragSun_ || sunHover_) {
        simul::Array<QVector3D> pts;
        simul::Array<int> mins;
        simul::Array<bool> up;
        sunPath(&pts, &mins, &up);
        simul::Array<float> seg, ticks;
        for (size_t i = 0; i + 1 < pts.size(); i++) {
            if (!up[i] || !up[i + 1]) continue;
            const float s[6] = {pts[i].x(), pts[i].y(), pts[i].z(), pts[i + 1].x(), pts[i + 1].y(), pts[i + 1].z()};
            seg.insert(seg.end(), s, s + 6);
        }
        const float ts = L * 0.018f;
        for (size_t i = 0; i < pts.size(); i++) {
            if (!up[i] || mins[i] % 60 != 0) continue;
            const QVector3D& q = pts[i];
            const float c[18] = {q.x() - ts, q.y(), q.z(), q.x() + ts, q.y(), q.z(), q.x(), q.y() - ts, q.z(), q.x(), q.y() + ts, q.z(), q.x(), q.y(), q.z() - ts, q.x(), q.y(), q.z() + ts};
            ticks.insert(ticks.end(), c, c + 18);
        }
        glLineWidth(2.0f);
        prog_->setUniformValue("uColor", QVector4D(0.95f, 0.82f, 0.35f, 1.0f));
        if (!seg.empty()) {
            prog_->setAttributeArray(0, GL_FLOAT, seg.data(), 3);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(seg.size() / 3));
        }
        if (!ticks.empty()) {
            prog_->setAttributeArray(0, GL_FLOAT, ticks.data(), 3);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(ticks.size() / 3));
        }
    }
    // 태양: 중심 -> 태양 선(노랑), 바닥 투영선과 수직 보조선(주황), 끝에 십자
    {
        const bool up = sunState_.aboveHorizon;
        const QVector4D yellow = up ? QVector4D(1.0f, 0.88f, 0.2f, 1.0f) : QVector4D(0.5f, 0.5f, 0.45f, 1.0f);
        const QVector4D orange = up ? QVector4D(1.0f, 0.55f, 0.15f, 1.0f) : QVector4D(0.45f, 0.4f, 0.35f, 1.0f);
        const QVector3D tip = o + sd * L;
        const QVector3D foot(tip.x(), tip.y(), 0.0f);
        const float ray[6] = {o.x(), o.y(), o.z(), tip.x(), tip.y(), tip.z()};
        const float ground[6] = {o.x(), o.y(), 0.0f, foot.x(), foot.y(), 0.0f};
        const float drop[6] = {foot.x(), foot.y(), 0.0f, tip.x(), tip.y(), tip.z()};
        glLineWidth(3.0f);
        prog_->setUniformValue("uColor", yellow);
        prog_->setAttributeArray(0, GL_FLOAT, ray, 3);
        glDrawArrays(GL_LINES, 0, 2);
        glLineWidth(1.5f);
        prog_->setUniformValue("uColor", orange);
        prog_->setAttributeArray(0, GL_FLOAT, ground, 3);
        glDrawArrays(GL_LINES, 0, 2);
        prog_->setAttributeArray(0, GL_FLOAT, drop, 3);
        glDrawArrays(GL_LINES, 0, 2);
        const float s = L * 0.05f;
        const float cross[18] = {tip.x() - s, tip.y(), tip.z(), tip.x() + s, tip.y(), tip.z(), tip.x(), tip.y() - s, tip.z(), tip.x(), tip.y() + s, tip.z(), tip.x(), tip.y(), tip.z() - s, tip.x(), tip.y(), tip.z() + s};
        glLineWidth(3.0f);
        prog_->setUniformValue("uColor", yellow);
        prog_->setAttributeArray(0, GL_FLOAT, cross, 3);
        glDrawArrays(GL_LINES, 0, 6);
    }
    glLineWidth(1.0f);
}

void ModelViewer::paintSunOverlay(const QMatrix4x4& proj, const QMatrix4x4& view) {
    if (!sunGuide_ || !sunSettings_.enabled) return;
    for (int i = 0; i < 4; i++) glDisableVertexAttribArray(static_cast<GLuint>(i));   // QPainter 가 쓰기 전에 남은 정점 배열을 꺼 둔다
    QVector3D o;
    float L = 0.0f;
    sunGuideGeometry(&o, &L);
    const QMatrix4x4 pv = proj * view;
    auto toScreen = [&](const QVector3D& w, QPointF* out) {
        const QVector4D c = pv * QVector4D(w, 1.0f);
        if (c.w() <= 1e-6f) return false;
        *out = QPointF((c.x() / c.w() * 0.5f + 0.5f) * width(), (1.0f - (c.y() / c.w() * 0.5f + 0.5f)) * height());
        return true;
    };
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    QFont f = p.font();
    f.setBold(true);
    f.setPointSizeF(f.pointSizeF() > 0 ? f.pointSizeF() * 1.15 : 10.0);
    p.setFont(f);
    // 글자는 어두운 반투명 바탕 위에 그려서 밝은 하늘 위에서도 읽힌다
    auto label = [&](const QRectF& r, const QString& text, const QColor& color) {
        const QFontMetrics m(p.font());
        const QRectF box(r.center().x() - (m.horizontalAdvance(text) + 8) * 0.5, r.center().y() - (m.height() + 2) * 0.5, m.horizontalAdvance(text) + 8, m.height() + 2);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0, 0, 0, 120));
        p.drawRoundedRect(box, 4, 4);
        p.setPen(color);
        p.drawText(box, Qt::AlignCenter, text);
    };
    const double nb = sunSettings_.site.northBearingDeg;
    const char* names[4] = {"북", "동", "남", "서"};
    for (int k = 0; k < 4; k++) {
        const QVector3D d = BearingDir(90.0 * k, nb);
        QPointF s;
        if (!toScreen(QVector3D(o.x() + d.x() * L * 1.2f, o.y() + d.y() * L * 1.2f, 0.0f), &s)) continue;
        label(QRectF(s.x() - 20, s.y() - 12, 40, 24), QString::fromUtf8(names[k]), k == 0 ? QColor(255, 120, 120) : QColor(225, 230, 245));
    }
    QPointF sp;
    if (toScreen(o + QVector3D(sunState_.direction.x, sunState_.direction.y, sunState_.direction.z) * L * 1.12f, &sp)) {
        label(QRectF(sp.x() - 30, sp.y() - 26, 60, 22), QString::fromUtf8("태양"), sunState_.aboveHorizon ? QColor(255, 225, 90) : QColor(170, 170, 160));
    }
    if (dragSun_ || sunHover_) {   // 정시 글자 (경로 위)
        simul::Array<QVector3D> pts;
        simul::Array<int> mins;
        simul::Array<bool> up;
        sunPath(&pts, &mins, &up);
        QFont smallFont = p.font();
        smallFont.setPointSizeF(smallFont.pointSizeF() * 0.8);
        smallFont.setBold(false);
        p.setFont(smallFont);
        for (size_t i = 0; i < pts.size(); i++) {
            if (!up[i] || mins[i] % 120 != 0) continue;   // 2시간마다 글자
            QPointF s;
            if (toScreen(pts[i], &s)) label(QRectF(s.x() - 20, s.y() + 10, 40, 16), QString::fromUtf8("%1시").arg(mins[i] / 60), QColor(240, 220, 150));
        }
        p.setFont(f);
    }
    // 위쪽에 글자: 시각, 방위각(8방위 이름), 고도
    static const char* dir8[8] = {"북", "북동", "동", "남동", "남", "남서", "서", "북서"};
    const int idx = static_cast<int>((sunState_.azimuthDeg + 22.5) / 45.0) % 8;
    const simul::env::SunTime& t = sunSettings_.time;
    const QString text = QString::fromUtf8("태양  %1-%2-%3 %4:%5   방위각 %6° (%7)   고도 %8° %9")
                             .arg(t.year, 4, 10, QChar('0')).arg(t.month, 2, 10, QChar('0')).arg(t.day, 2, 10, QChar('0')).arg(t.hour, 2, 10, QChar('0')).arg(t.minute, 2, 10, QChar('0'))
                             .arg(sunState_.azimuthDeg, 0, 'f', 1).arg(QString::fromUtf8(dir8[idx < 0 ? 0 : idx])).arg(sunState_.elevationDeg, 0, 'f', 1)
                             .arg(sunState_.aboveHorizon ? QString::fromUtf8("(떠 있음)") : QString::fromUtf8("(해 진 상태)"));
    const QFontMetrics fm(p.font());
    const QRect box(10, 8, fm.horizontalAdvance(text) + 18, fm.height() + 10);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 120));
    p.drawRoundedRect(box, 5, 5);
    p.setPen(sunState_.aboveHorizon ? QColor(255, 230, 120) : QColor(190, 190, 180));
    p.drawText(box, Qt::AlignCenter, text);
}

void ModelViewer::SetSkyEnabled(bool on) {
    if (skyOn_ == on) return;
    skyOn_ = on;
    update();
}

void ModelViewer::drawSky(const QMatrix4x4& proj, const QMatrix4x4& view, const SunLighting& sun) {
    if (!skyOn_ || !skyProg_ || !skyProg_->isLinked()) return;
    bool ok = false;
    const QMatrix4x4 inv = (proj * view).inverted(&ok);
    if (!ok) return;
    const QVector3D eye = view.inverted().column(3).toVector3D();
    // 태양 조명이 꺼져 있어도 하늘은 낮 하늘(구름 포함)로 그린다. 태양 원반은 없다 (세기 0).
    SunLighting L = sun;
    QVector3D sunDir(0.3f, 0.4f, 0.85f);
    if (sun.on) {
        sunDir = QVector3D(sunState_.direction.x, sunState_.direction.y, sunState_.direction.z);
    } else {
        L.sunK = 0.0f;
        L.ambient = 1.0f;
        L.sky[0] = 0.42f; L.sky[1] = 0.60f; L.sky[2] = 0.84f;
        L.color[0] = L.color[1] = L.color[2] = 1.0f;
    }
    const float day = L.ambient > 0.1f ? (L.ambient - 0.1f) / 0.9f : 0.0f;
    // 천정은 지평 쪽(L.sky)보다 짙고, 낮에는 수평선이 하얗게 흐리다
    const QVector3D horizon(L.sky[0] + (1.0f - L.sky[0]) * 0.40f * day, L.sky[1] + (1.0f - L.sky[1]) * 0.40f * day, L.sky[2] + (1.0f - L.sky[2]) * 0.30f * day);
    const QVector3D top(L.sky[0] * 0.55f, L.sky[1] * 0.72f, simul::Min(L.sky[2] * 1.05f, 1.0f));
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    skyProg_->bind();
    skyProg_->setUniformValue("uInvVP", inv);
    skyProg_->setUniformValue("uEye", eye);
    skyProg_->setUniformValue("uSkyTop", top);
    skyProg_->setUniformValue("uSkyHorizon", horizon);
    skyProg_->setUniformValue("uSunDirW", sunDir.normalized());
    skyProg_->setUniformValue("uSunCol", QVector3D(L.color[0], L.color[1], L.color[2]));
    skyProg_->setUniformValue("uSunStrength", L.sunK);
    skyProg_->setUniformValue("uDay", day);
    static const float tri[6] = {-1.0f, -1.0f, 3.0f, -1.0f, -1.0f, 3.0f};
    skyProg_->enableAttributeArray(0);
    skyProg_->setAttributeArray(0, GL_FLOAT, tri, 2);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    skyProg_->disableAttributeArray(0);
    skyProg_->release();
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

void ModelViewer::SetShadowsEnabled(bool on) {
    if (shadows_ == on) return;
    shadows_ = on;
    update();
}

void ModelViewer::shadowBounds(QVector3D* center, float* radius) const {
    float mn[3] = {simul::kFloatMax, simul::kFloatMax, simul::kFloatMax};
    float mx[3] = {-mn[0], -mn[1], -mn[2]};
    bool any = false;
    for (size_t i = 0; i < items_.size(); i++) {
        const Item& it = *items_[i];
        if (it.color[3] < 0.99f || !it.vbo.isCreated() || !it.ibo.isCreated() || it.data.size() < 6) continue;
        if (!it.boxOk) {
            for (int k = 0; k < 3; k++) { it.boxLo[k] = simul::kFloatMax; it.boxHi[k] = -simul::kFloatMax; }
            for (size_t v = 0; v + 5 < it.data.size(); v += 6)
                for (int k = 0; k < 3; k++) { it.boxLo[k] = simul::Min(it.boxLo[k], it.data[v + static_cast<size_t>(k)]); it.boxHi[k] = simul::Max(it.boxHi[k], it.data[v + static_cast<size_t>(k)]); }
            it.boxOk = true;
        }
        const dhMat m = itemWorld(it);
        for (int c = 0; c < 8; c++) {
            const float p[3] = {(c & 1) ? it.boxHi[0] : it.boxLo[0], (c & 2) ? it.boxHi[1] : it.boxLo[1], (c & 4) ? it.boxHi[2] : it.boxLo[2]};
            float o[3];
            Transform(m, p, o);
            for (int k = 0; k < 3; k++) { mn[k] = simul::Min(mn[k], o[k]); mx[k] = simul::Max(mx[k], o[k]); }
            any = true;
        }
    }
    if (!any) {
        *center = QVector3D(0, 0, 0);
        *radius = 0.0f;
        return;
    }
    *center = QVector3D((mn[0] + mx[0]) * 0.5f, (mn[1] + mx[1]) * 0.5f, (mn[2] + mx[2]) * 0.5f);
    const float dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
    *radius = simul::Sqrt(dx * dx + dy * dy + dz * dz) * 0.5f;
}

// 태양 쪽에서 본 깊이를 그려 둔다. 그린 뒤에는 그림자 맵을 텍스처 1번에 걸고 메인 셰이더의 uniform 을 채운다.
bool ModelViewer::renderShadowMap(const QMatrix4x4& view, const SunLighting& sun) {
    if (!shadows_ || !sun.on || sun.sunK <= 0.001f || !shadowProg_ || !shadowProg_->isLinked()) return false;
    QVector3D c;
    float R = 0.0f;
    shadowBounds(&c, &R);
    if (!(R > 1e-5f)) return false;
    R *= 1.02f;
    QVector3D dir(sunState_.direction.x, sunState_.direction.y, sunState_.direction.z);
    if (dir.length() < 1e-6f) return false;
    dir.normalize();
    if (dir.z() <= 0.0f) return false;   // 태양이 수평선 아래
    if (!shadowFbo_ || shadowFbo_->width() != kShadowMapSize) {
        shadowFbo_.reset(new QOpenGLFramebufferObject(kShadowMapSize, kShadowMapSize, QOpenGLFramebufferObject::Depth));
        if (!shadowFbo_->isValid()) {
            shadowFbo_.reset();
            return false;
        }
        glBindTexture(GL_TEXTURE_2D, shadowFbo_->texture());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);   // 깊이를 나눠 담았으니 섞이면 안 된다
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    QMatrix4x4 lv;
    lv.lookAt(c + dir * R, c, (dir.z() > 0.99f || dir.z() < -0.99f) ? QVector3D(0.0f, 1.0f, 0.0f) : QVector3D(0.0f, 0.0f, 1.0f));
    QMatrix4x4 lp;
    lp.ortho(-R, R, -R, R, 0.0f, 2.0f * R);
    const QMatrix4x4 lightVP = lp * lv;

    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    shadowFbo_->bind();
    glViewport(0, 0, kShadowMapSize, kShadowMapSize);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);   // 깊이 1 (아주 멀다)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_BLEND);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0f, 4.0f);
    shadowProg_->bind();
    shadowProg_->enableAttributeArray(0);
    for (size_t i = 0; i < items_.size(); i++) {
        Item& it = *items_[i];
        if (it.color[3] < 0.99f || !it.vbo.isCreated() || !it.ibo.isCreated()) continue;
        if (it.alphaMode == 2 && (it.tex || it.hasVertexColor)) continue;   // 반투명은 그림자를 드리우지 않는다
        shadowProg_->setUniformValue("uLightMVP", lightVP * ToQt(itemWorld(it)));
        it.vbo.bind();
        it.ibo.bind();
        shadowProg_->setAttributeBuffer(0, GL_FLOAT, 0, 3, 6 * static_cast<int>(sizeof(float)));
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(it.indices.size()), GL_UNSIGNED_INT, nullptr);
        it.vbo.release();
        it.ibo.release();
    }
    shadowProg_->disableAttributeArray(0);
    shadowProg_->release();
    glDisable(GL_POLYGON_OFFSET_FILL);
    glEnable(GL_BLEND);
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
    glViewport(vp[0], vp[1], vp[2], vp[3]);

    // 메인 셰이더: 그림자 맵은 텍스처 1번 (0번은 재질 텍스처)
    prog_->bind();   // shadowProg_->release() 가 프로그램을 0 으로 돌려놓았다
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, shadowFbo_->texture());
    glActiveTexture(GL_TEXTURE0);
    prog_->setUniformValue("uShadowMap", 1);
    prog_->setUniformValue("uShadowOn", 1.0f);
    prog_->setUniformValue("uLightFromView", lightVP * view.inverted());
    prog_->setUniformValue("uShadowTexel", 2.0f * R / static_cast<float>(kShadowMapSize));
    prog_->setUniformValue("uShadowInv", 1.0f / static_cast<float>(kShadowMapSize));
    return true;
}

void ModelViewer::SetSensorCamera(int link, const simul::sensor::CameraSettings& settings, const dhMat& mount) {
    cameraOn_ = true;
    cameraDraw_ = true;
    cameraLink_ = link;
    cameraMount_ = mount;
    camera_.SetSettings(settings);
    update();
}

void ModelViewer::ClearSensorCamera() {
    cameraOn_ = false;
    cameraImage_ = simul::sensor::CameraImage();
    update();
}

dhMat ModelViewer::SensorCameraPose() const {
    dhMat pose = dhMat::Trans(objectPos_.x, objectPos_.y, objectPos_.z);
    if (cameraLink_ >= 0 && static_cast<size_t>(cameraLink_) < linkT_.size()) pose = pose * linkT_[static_cast<size_t>(cameraLink_)];
    return pose * cameraMount_;
}

// 카메라가 볼 메시들 (skipLink 의 링크는 뺀다). 색은 primitive 의 기본 색이다.
void ModelViewer::collectTargets(int skipLink, simul::Array<simul::sensor::CameraTarget>* out) {
    for (size_t i = 0; i < items_.size(); i++) {
        Item& it = *items_[i];
        if (skipLink >= 0 && it.link == skipLink) continue;
        if (it.link < 0 && !hasScene_) continue;
        if (it.data.empty() || it.indices.empty()) continue;
        simul::sensor::CameraTarget c;
        c.mesh.positions = it.data.data();
        c.mesh.vertexCount = it.data.size() / 6;
        c.mesh.indices = it.indices.data();
        c.mesh.indexCount = it.indices.size();
        c.mesh.stride = 6;
        c.mesh.world = itemWorld(it);
        c.mesh.id = it.link < 0 ? 0 : it.link;
        if (!it.bvhOk) {
            it.bvh.Build(it.data.data(), it.data.size() / 6, it.indices.data(), it.indices.size(), 6);
            it.bvhOk = true;
        }
        c.mesh.bvh = &it.bvh;
        c.color[0] = it.color[0];
        c.color[1] = it.color[1];
        c.color[2] = it.color[2];
        out->push_back(c);
    }
}

const simul::sensor::CameraImage& ModelViewer::RenderSensorCamera() {
    if (!cameraOn_) {
        cameraImage_ = simul::sensor::CameraImage();
        return cameraImage_;
    }
    simul::Array<simul::sensor::CameraTarget> targets;
    collectTargets(cameraLink_, &targets);
    {   // 태양 모드면 카메라 영상도 같은 태양으로 비춘다 (그림자 포함)
        simul::sensor::SunLight sl;
        const SunLighting L = sunLighting();
        if (L.on) {
            sl.enabled = true;
            sl.direction[0] = sunState_.direction.x;
            sl.direction[1] = sunState_.direction.y;
            sl.direction[2] = sunState_.direction.z;
            sl.intensity = L.sunK;
            sl.ambient = L.ambient;
            for (int i = 0; i < 3; i++) {
                sl.color[i] = L.color[i];
                sl.sky[i] = L.sky[i];
            }
        }
        camera_.SetSun(sl);
    }
    camera_.Render(SensorCameraPose(), targets.data(), targets.size(), &cameraImage_);
    emit sensorCameraRendered(cameraImage_);
    return cameraImage_;
}

// 시야 절두체: 센서 원점에서 먼 면의 네 모서리까지 선 4개와 그 사각형. 먼 면은 far 또는 0.5 m 중 가까운 쪽에 둔다.
void ModelViewer::drawSensorCameraFrustum() {
    if (!camera_.Valid()) return;
    const simul::sensor::CameraSettings& cs = camera_.Settings();
    const float d = simul::Min(cs.farClip, 0.5f);
    const dhMat pose = SensorCameraPose();
    const float w = static_cast<float>(cs.width), h = static_cast<float>(cs.height);
    dhVector c[4];
    const float us[4] = {0.0f, w, w, 0.0f}, vs[4] = {0.0f, 0.0f, h, h};
    for (int i = 0; i < 4; i++) c[i] = pose * (camera_.PixelDirection(us[i], vs[i]) * d);
    const dhVector o = pose.O();
    simul::Array<float> v;
    const dhVector* pts[8][2] = {{&o, &c[0]}, {&o, &c[1]}, {&o, &c[2]}, {&o, &c[3]}, {&c[0], &c[1]}, {&c[1], &c[2]}, {&c[2], &c[3]}, {&c[3], &c[0]}};
    for (int e = 0; e < 8; e++)
        for (int k = 0; k < 2; k++) {
            v.push_back(pts[e][k]->x);
            v.push_back(pts[e][k]->y);
            v.push_back(pts[e][k]->z);
        }
    glLineWidth(1.5f);
    prog_->setUniformValue("uColor", QVector4D(1.0f, 0.85f, 0.1f, 0.9f));
    prog_->setAttributeArray(0, GL_FLOAT, v.data(), 3);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(v.size() / 3));
    // 위쪽 변 위에 작은 삼각형을 겹쳐 영상의 위가 어느 쪽인지 알려준다
    const dhVector tipBase0 = (c[0] + c[1]) * 0.5f;
    const dhVector tip = tipBase0 + (tipBase0 - (c[2] + c[3]) * 0.5f) * 0.25f;
    simul::Array<float> t;
    const dhVector* tv[6] = {&c[0], &tip, &tip, &c[1], &c[1], &c[0]};
    for (int i = 0; i < 6; i++) {
        t.push_back(tv[i]->x);
        t.push_back(tv[i]->y);
        t.push_back(tv[i]->z);
    }
    prog_->setAttributeArray(0, GL_FLOAT, t.data(), 3);
    glDrawArrays(GL_LINES, 0, 6);
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
    QElapsedTimer paintTimer;
    paintTimer.start();
    paintScene();
    // 프레임 정보: 0.5 초마다 평균을 낸다
    if (!statClock_.isValid()) statClock_.start();
    statFrames_++;
    statPaintSum_ += static_cast<double>(paintTimer.nsecsElapsed()) / 1.0e6;
    const double elapsed = static_cast<double>(statClock_.elapsed());
    if (elapsed >= 500.0) {
        fps_ = static_cast<double>(statFrames_) * 1000.0 / elapsed;
        paintMs_ = statPaintSum_ / statFrames_;
        statFrames_ = 0;
        statPaintSum_ = 0.0;
        statClock_.start();
        if (frameStatsVisible_) refreshScaleInfo();
    }
}

// 태양 모드의 빛: 해 고도에 따라 세기, 색, 하늘 밝기, 배경색을 정한다 (해 아래 -8도 아래는 완전한 밤, 10도 위는 완전한 낮)
ModelViewer::SunLighting ModelViewer::sunLighting() const {
    SunLighting L;
    if (!sunSettings_.enabled) return L;
    const double el = sunState_.elevationDeg;
    auto smooth = [](double a, double b, double x) {
        double t = (x - a) / (b - a);
        t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
        return t * t * (3.0 - 2.0 * t);
    };
    L.on = true;
    L.sunK = static_cast<float>(smooth(-1.5, 8.0, el));
    L.ambient = static_cast<float>(0.10 + 0.90 * smooth(-8.0, 10.0, el));
    const double warm = 1.0 - smooth(0.0, 25.0, el);   // 낮게 뜰수록 붉다
    L.color[0] = 1.0f;
    L.color[1] = static_cast<float>(0.97 - 0.35 * warm);
    L.color[2] = static_cast<float>(0.92 - 0.57 * warm);
    const double day = smooth(-8.0, 10.0, el);
    L.sky[0] = static_cast<float>(0.015 + (0.42 + 0.30 * warm) * day);
    L.sky[1] = static_cast<float>(0.02 + (0.60 - 0.18 * warm) * day);
    L.sky[2] = static_cast<float>(0.045 + (0.84 - 0.34 * warm) * day);
    return L;
}

void ModelViewer::paintScene() {
    const SunLighting sun = sunLighting();
    if (sun.on) glClearColor(sun.sky[0], sun.sky[1], sun.sky[2], 1.0f);
    else glClearColor(0.16f, 0.17f, 0.20f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!prog_ || !prog_->isLinked()) {   // 셰이더가 없으면 그릴 수 없다: 빈 화면 대신 이유를 화면에 쓴다
        QPainter p(this);
        p.setPen(QColor(255, 200, 120));
        p.drawText(rect().adjusted(12, 12, -12, -12), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap, shaderLog_.isEmpty() ? QString::fromUtf8("OpenGL 셰이더가 준비되지 않았습니다") : shaderLog_);
        return;
    }
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

    drawSky(proj, view, sun);

    prog_->bind();
    prog_->setUniformValue("uLit", 0.0f);
    prog_->setUniformValue("uUseTex", 0.0f);
    prog_->setUniformValue("uUseVC", 0.0f);
    prog_->setUniformValue("uAlphaMode", 0);
    prog_->setUniformValue("uShading", shading_);
    prog_->setUniformValue("uMetal", kDefaultMetal);
    prog_->setUniformValue("uRough", kDefaultRough);
    {   // 월드 위쪽(+z)이 카메라 좌표에서 어느 쪽인지 (PBR 의 하늘/바닥 환경)
        const QVector3D up = view.mapVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();
        prog_->setUniformValue("uUp", up);
    }
    {   // 태양: 월드 방향을 카메라 좌표로 (화면을 돌려도 태양은 월드에 고정이다)
        prog_->setUniformValue("uSunOn", sun.on ? 1.0f : 0.0f);
        const QVector3D d = view.mapVector(QVector3D(sunState_.direction.x, sunState_.direction.y, sunState_.direction.z)).normalized();
        prog_->setUniformValue("uSunDir", sun.on ? d : QVector3D(0.0f, 0.0f, 1.0f));
        prog_->setUniformValue("uSunColor", sun.on ? QVector3D(sun.color[0], sun.color[1], sun.color[2]) : QVector3D(1.0f, 1.0f, 1.0f));
        prog_->setUniformValue("uSunK", sun.on ? sun.sunK * 1.25f : 1.0f);
        // 태양 모드: 그림자와 태양 반대쪽 면이 너무 어둡지 않게 낮에는 보조광(카메라 쪽 약한 빛)을 조금 켜고 주변광을 키운다. 밤에는 같이 줄어든다.
        // (센서 카메라 영상의 밝기는 SunLighting::ambient 를 그대로 써서 바뀌지 않는다)
        const float day = sun.ambient > 0.1f ? (sun.ambient - 0.1f) / 0.9f : 0.0f;
        prog_->setUniformValue("uFillK", sun.on ? 0.8f * day : 1.0f);
        prog_->setUniformValue("uAmb", sun.on ? sun.ambient * 2.0f : 1.0f);
    }

    prog_->setUniformValue("uShadowOn", 0.0f);
    prog_->setUniformValue("uCatcher", 0.0f);
    prog_->setUniformValue("uShadowMap", 1);
    shadowActive_ = renderShadowMap(view, sun);

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
        drawSunGuide();
        prog_->disableAttributeArray(0);
    }

    // 모델 (불투명 먼저, 반투명은 나중에)
    prog_->setUniformValue("uLit", 1.0f);
    for (int pass = 0; pass < 2; pass++) {
        for (size_t i = 0; i < items_.size(); i++) {
            Item& it = *items_[i];
            const bool translucent = it.color[3] < 0.99f || (it.alphaMode == 2 && (it.tex || it.hasVertexColor));
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
            prog_->setUniformValue("uAlphaMode", (useTex || useVc) ? it.alphaMode : 0);
            prog_->setUniformValue("uCutoff", it.alphaCutoff);
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
    prog_->setUniformValue("uAlphaMode", 0);
    if (shadowActive_) {   // 바닥 그림자 받이: 바닥(z = 0) 바로 아래의 투명한 면에 그림자만 그린다. 바닥 모델이 있으면 그 아래에 가려진다.
        const float h = gridCell_ * 10.0f, z = -simul::Max(radius_ * 1e-3f, 1e-4f);
        const float quad[18] = {-h, -h, z, h, -h, z, h, h, z, -h, -h, z, h, h, z, -h, h, z};
        prog_->setUniformValue("uCatcher", 1.0f);
        prog_->setUniformValue("uMVP", proj * view);
        prog_->setUniformValue("uModelView", view);
        prog_->enableAttributeArray(0);
        prog_->disableAttributeArray(1);
        prog_->setAttributeArray(0, GL_FLOAT, quad, 3);
        glDepthMask(GL_FALSE);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glDepthMask(GL_TRUE);
        prog_->disableAttributeArray(0);
        prog_->setUniformValue("uCatcher", 0.0f);
    }
    if (lidarOn_ && lidarDraw_) {
        prog_->setUniformValue("uLit", 0.0f);
        prog_->setUniformValue("uMVP", proj * view);
        prog_->setUniformValue("uModelView", view);
        prog_->enableAttributeArray(0);
        prog_->disableAttributeArray(1);
        drawLidar();
    }
    if (cameraOn_ && cameraDraw_) {
        prog_->setUniformValue("uLit", 0.0f);
        prog_->setUniformValue("uMVP", proj * view);
        prog_->setUniformValue("uModelView", view);
        prog_->enableAttributeArray(0);
        prog_->disableAttributeArray(1);
        glDisable(GL_DEPTH_TEST);
        drawSensorCameraFrustum();
        glEnable(GL_DEPTH_TEST);
    }
    if (showFrame_ && HasSelection()) drawSelectionFrame(proj, view);
    prog_->release();
    paintSunOverlay(proj, view);
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
    if (e->button() == Qt::LeftButton && sunHit(e->pos())) {   // 태양 표시를 잡았다: 끌어서 시각을 바꾼다
        dragSun_ = true;
        dragged_ = true;
        setCursor(Qt::ClosedHandCursor);
        return;
    }
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
    if (dragSun_) {
        dragSun_ = false;
        sunHover_ = sunHit(e->pos());
        setCursor(sunHover_ ? Qt::OpenHandCursor : Qt::ArrowCursor);
        update();
        return;
    }
    // 거의 움직이지 않고 뗀 오른쪽 버튼 = 메뉴 요청 (끌었다면 카메라 이동이었다)
    if (e->button() == Qt::RightButton) {
        if (!dragged_) {
            QVector3D ro, rd, p(0, 0, 0);
            if (mouseRay(e->pos(), &ro, &rd) && RayPlane(ro, rd, QVector3D(0, 0, 0), QVector3D(0, 0, 1), &p)) {
                if (!(p.length() < 1000.0f)) p = QVector3D(0, 0, 0);   // 지평선 쪽을 눌러 터무니없이 먼 점이 나온 경우
            }
            emit viewContextMenuRequested(e->globalPos(), p.x(), p.y());
        }
        return;
    }
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
    if (dragSun_) {
        setSunFromScreen(e->pos());
        return;
    }
    if (e->buttons() == Qt::NoButton) {
        const bool sunOver = sunHit(e->pos());
        setCursor(sunOver ? Qt::OpenHandCursor : Qt::ArrowCursor);
        if (sunOver != sunHover_) { sunHover_ = sunOver; update(); }
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
        const float kPerPixel = 0.4f * 3.14159265f / 180.0f;   // 한 픽셀에 0.4 도
        const float kMaxPitch = 89.0f * 3.14159265f / 180.0f;
        yaw_ -= static_cast<float>(d.x()) * kPerPixel;
        pitch_ = simul::Max(-kMaxPitch, simul::Min(kMaxPitch, pitch_ + static_cast<float>(d.y()) * kPerPixel));
        refreshScaleInfo();
        update();
    } else if (e->buttons() & (Qt::RightButton | Qt::MiddleButton)) {
        // 화면 기준 오른쪽/위 방향으로 이동
        const QMatrix4x4 v = viewMatrix();
        const QVector3D right(v(0, 0), v(0, 1), v(0, 2)), up(v(1, 0), v(1, 1), v(1, 2));
        const float s = dist_ * 0.0016f;
        target_ += (-static_cast<float>(d.x()) * right + static_cast<float>(d.y()) * up) * s;
        refreshScaleInfo();
        update();
    }
}

void ModelViewer::wheelEvent(QWheelEvent* e) {
    dist_ = simul::Max(radius_ * 0.02f, dist_ * simul::Pow(0.9985f, static_cast<float>(e->angleDelta().y())));
    refreshScaleInfo();
    update();
}

void ModelViewer::mouseDoubleClickEvent(QMouseEvent*) { FitView(); }
