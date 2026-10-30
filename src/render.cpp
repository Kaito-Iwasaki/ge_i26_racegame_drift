#include "render.hpp"

#include <SDL.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#ifdef __EMSCRIPTEN__
#include <GLES3/gl3.h>
#else
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#endif

namespace {

const char* kVersion =
#ifdef __EMSCRIPTEN__
    "#version 300 es\nprecision highp float;\n";
#else
    "#version 330 core\n";
#endif

const char* kWorldVs = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
uniform mat4 uViewProj;
uniform mat4 uModel;
out vec3 vNormal;
out vec3 vColor;
out vec3 vWorld;
void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    vColor = aColor;
    gl_Position = uViewProj * world;
}
)GLSL";

const char* kWorldFs = R"GLSL(
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorld;
uniform vec3 uCamPos;
uniform float uNight;
uniform vec3 uLampLeft, uLampRight, uLampDir, uRearLight;
uniform float uBrakeOn, uReverseOn;
uniform float uEmissive;
out vec4 fragColor;
float beam(vec3 position, vec3 lamp) {
    vec3 offset = position - lamp;
    vec3 horizontal = normalize(vec3(uLampDir.x, 0.0, uLampDir.z));
    float along = dot(offset.xz, horizontal.xz);
    float across = abs(offset.x * horizontal.z - offset.z * horizontal.x);
    float width = 2.5 + max(along, 0.0) * 0.75;
    float cone = 1.0 - smoothstep(width * 0.70, width, across);
    float range = smoothstep(0.2, 2.5, along) * (1.0 - smoothstep(40.0, 95.0, along));
    float expectedY = lamp.y + along * (uLampDir.y - 0.025);
    float height = 1.0 - smoothstep(2.0, 7.0 + max(along, 0.0) * 0.10, abs(position.y - expectedY));
    return cone * range * height;
}
void main() {
    vec3 n = normalize(vNormal);
    if (!gl_FrontFacing) n = -n;
    vec3 sun = normalize(vec3(0.35, 0.86, 0.28));
    float light = 0.36 + 0.64 * max(dot(n, sun), 0.0);
    vec3 color = vColor * light;
    if (uNight > 0.5) {
        float lamps = max(beam(vWorld, uLampLeft), beam(vWorld, uLampRight));
        vec3 moon = vec3(0.12, 0.16, 0.24);
        color = vColor * (moon + vec3(1.45, 1.32, 1.05) * lamps);
    }
    if (uBrakeOn > 0.5 || uReverseOn > 0.5) {
        vec3 offset = vWorld - uRearLight;
        vec2 fwd = normalize(uLampDir.xz);
        float behind = -dot(offset.xz, fwd);
        float spread = length(offset.xz);
        float behindFade = smoothstep(-0.5, 1.0, behind);
        float brakeReach = (1.0 - smoothstep(0.5, 4.0, spread)) * behindFade;
        float reverseReach = (1.0 - smoothstep(1.0, 8.0, spread)) * behindFade;
        float vertical = 1.0 - smoothstep(1.5, 4.0, abs(offset.y));
        float intensity = vertical * mix(0.30, 1.0, uNight);
        color += vColor * intensity * (uBrakeOn * brakeReach * vec3(1.6, 0.035, 0.02) + uReverseOn * reverseReach * vec3(0.60, 0.58, 0.48));
    }
    if (uEmissive > 0.5) color = vColor;
    float dist = distance(vWorld, uCamPos);
    float fog = clamp((dist - 60.0) / 900.0, 0.0, 1.0);
    vec3 haze = mix(vec3(0.62, 0.74, 0.84), vec3(0.025, 0.035, 0.07), uNight);
    if (uNight > 0.5) fog = clamp((dist - 120.0) / 650.0, 0.0, 1.0);
    if (uEmissive > 0.5) fog = 0.0;
    fragColor = vec4(mix(color, haze, fog), 1.0);
}
)GLSL";

const char* kSkyVs = R"GLSL(
out float vY;
void main() {
    vec2 pos = vec2((gl_VertexID == 2) ? 3.0 : -1.0, (gl_VertexID == 1) ? 3.0 : -1.0);
    vY = pos.y;
    gl_Position = vec4(pos, 1.0, 1.0);
}
)GLSL";

const char* kSkyFs = R"GLSL(
in float vY;
uniform float uNight;
out vec4 fragColor;
void main() {
    float t = clamp(vY * 0.5 + 0.5, 0.0, 1.0);
    vec3 horizon = vec3(0.62, 0.74, 0.84);
    vec3 zenith = vec3(0.28, 0.48, 0.78);
    horizon = mix(horizon, vec3(0.025, 0.035, 0.07), uNight);
    zenith = mix(zenith, vec3(0.006, 0.010, 0.035), uNight);
    fragColor = vec4(mix(horizon, zenith, smoothstep(0.0, 1.0, t)), 1.0);
}
)GLSL";

const char* kSmokeVs = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUv;
layout(location = 2) in float aAlpha;
uniform mat4 uViewProj;
out vec2 vUv;
out float vAlpha;
void main() {
    vUv = aUv;
    vAlpha = aAlpha;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

const char* kSmokeFs = R"GLSL(
in vec2 vUv;
in float vAlpha;
out vec4 fragColor;
void main() {
    // 中心からの距離で円形に柔らかく落とす（煙の粒の見た目）。
    float d = length(vUv - vec2(0.5)) * 2.0;
    float fall = smoothstep(1.0, 0.2, d);
    fall *= fall;
    vec3 color = vec3(0.90, 0.90, 0.92);
    fragColor = vec4(color, vAlpha * fall);
}
)GLSL";

const char* kGlowVs = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
layout(location = 2) in float aAlpha;
uniform mat4 uViewProj;
out vec3 vColor;
out float vAlpha;
void main() {
    vColor = aColor;
    vAlpha = aAlpha;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

const char* kGlowFs = R"GLSL(
in vec3 vColor;
in float vAlpha;
out vec4 fragColor;
void main() {
    fragColor = vec4(vColor, vAlpha);
}
)GLSL";

const char* kHudVs = R"GLSL(
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uOrtho;
out vec4 vColor;
void main() {
    vColor = aColor;
    gl_Position = uOrtho * vec4(aPos, 0.0, 1.0);
}
)GLSL";

const char* kHudFs = R"GLSL(
in vec4 vColor;
out vec4 fragColor;
void main() {
    fragColor = vColor;
}
)GLSL";

// 5x7。bit4 が左端。並びは 0-9 A-Z . : - / %
constexpr char kGlyphOrder[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.:-/%";
const std::uint8_t kFont[][7] = {
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E}, {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}, {0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E},
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}, {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}, {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11},
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}, {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    {0x0E, 0x11, 0x10, 0x0E, 0x01, 0x11, 0x0E}, {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}, {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}, {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00},
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}, {0x01, 0x02, 0x04, 0x04, 0x08, 0x10, 0x10},
    {0x19, 0x1A, 0x02, 0x04, 0x08, 0x0B, 0x13},
};
static_assert(sizeof(kFont) / sizeof(kFont[0]) == sizeof(kGlyphOrder) - 1, "glyph count");

GLuint compileShader(GLenum type, const char* source) {
    const char* parts[] = {kVersion, source};
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 2, parts, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        SDL_Log("shader compile failed: %s", log);
        return 0;
    }
    return shader;
}

GLuint linkProgram(const char* vs, const char* fs) {
    GLuint vertex = compileShader(GL_VERTEX_SHADER, vs);
    GLuint fragment = compileShader(GL_FRAGMENT_SHADER, fs);
    if (!vertex || !fragment) {
        return 0;
    }
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        SDL_Log("shader link failed: %s", log);
        return 0;
    }
    return program;
}

int glyphIndex(char c) {
    if (c >= 'a' && c <= 'z') {
        c = static_cast<char>(c - 'a' + 'A');
    }
    const char* found = std::strchr(kGlyphOrder, c);
    if (!found) {
        return -1;
    }
    return static_cast<int>(found - kGlyphOrder);
}

}  // namespace

bool Renderer::init() {
    glGenVertexArrays(1, &dummyVao);
    worldProgram = linkProgram(kWorldVs, kWorldFs);
    skyProgram = linkProgram(kSkyVs, kSkyFs);
    hudProgram = linkProgram(kHudVs, kHudFs);
    smokeProgram = linkProgram(kSmokeVs, kSmokeFs);
    glowProgram = linkProgram(kGlowVs, kGlowFs);
    if (!worldProgram || !skyProgram || !hudProgram || !smokeProgram || !glowProgram) {
        return false;
    }
    uViewProj = glGetUniformLocation(worldProgram, "uViewProj");
    uModel = glGetUniformLocation(worldProgram, "uModel");
    uCamPos = glGetUniformLocation(worldProgram, "uCamPos");
    uNight = glGetUniformLocation(worldProgram, "uNight");
    uLampLeft = glGetUniformLocation(worldProgram, "uLampLeft");
    uLampRight = glGetUniformLocation(worldProgram, "uLampRight");
    uLampDir = glGetUniformLocation(worldProgram, "uLampDir");
    uRearLight = glGetUniformLocation(worldProgram, "uRearLight");
    uBrakeOn = glGetUniformLocation(worldProgram, "uBrakeOn");
    uReverseOn = glGetUniformLocation(worldProgram, "uReverseOn");
    uEmissive = glGetUniformLocation(worldProgram, "uEmissive");
    uSkyNight = glGetUniformLocation(skyProgram, "uNight");
    uOrtho = glGetUniformLocation(hudProgram, "uOrtho");
    uSmokeViewProj = glGetUniformLocation(smokeProgram, "uViewProj");
    uGlowViewProj = glGetUniformLocation(glowProgram, "uViewProj");

    glGenVertexArrays(1, &hudVao);
    glGenBuffers(1, &hudVbo);
    glGenBuffers(1, &hudIbo);
    glBindVertexArray(hudVao);
    glBindBuffer(GL_ARRAY_BUFFER, hudVbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hudIbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(2 * sizeof(float)));

    glGenVertexArrays(1, &smokeVao);
    glGenBuffers(1, &smokeVbo);
    glGenBuffers(1, &smokeIbo);
    glBindVertexArray(smokeVao);
    glBindBuffer(GL_ARRAY_BUFFER, smokeVbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, smokeIbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(5 * sizeof(float)));

    glGenVertexArrays(1, &glowVao);
    glGenBuffers(1, &glowVbo);
    glGenBuffers(1, &glowIbo);
    glBindVertexArray(glowVao);
    glBindBuffer(GL_ARRAY_BUFFER, glowVbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, glowIbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 7 * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));
    glBindVertexArray(dummyVao);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    // lookAt は左右を直すために鏡映になっている。巻き方向が反転するので、表の判定も合わせる。
    glFrontFace(GL_CW);
    resize(width, height);
    return true;
}

void Renderer::clearFrame() {
    glClearColor(0.62f, 0.74f, 0.84f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::resize(int newWidth, int newHeight) {
    width = newWidth > 0 ? newWidth : 1;
    height = newHeight > 0 ? newHeight : 1;
    glViewport(0, 0, width, height);
}

void Renderer::beginMirror(int x, int y, int w, int h) {
    glEnable(GL_SCISSOR_TEST);
    glScissor(x,y,w,h);
    glViewport(x,y,w,h);
    glClear(GL_DEPTH_BUFFER_BIT);
    // Mirror projection reverses winding; keep the same visible face lighting.
    glFrontFace(GL_CCW);
}
void Renderer::endMirror() {
    glFrontFace(GL_CW);
    glDisable(GL_SCISSOR_TEST);
    glViewport(0,0,width,height);
}

void Renderer::drawSky(bool night) {
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glUseProgram(skyProgram);
    glUniform1f(uSkyNight, night ? 1.0f : 0.0f);
    glBindVertexArray(dummyVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void Renderer::beginWorld(const Mat4& viewProj, Vec3 cameraPos, bool night, Vec3 lampLeft, Vec3 lampRight, Vec3 lampDirection, Vec3 rearLight, bool braking, bool reversing) {
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(worldProgram);
    glUniformMatrix4fv(uViewProj, 1, GL_FALSE, viewProj.m);
    glUniform3f(uCamPos, cameraPos.x, cameraPos.y, cameraPos.z);
    glUniform1f(uNight, night ? 1.0f : 0.0f);
    glUniform3f(uLampLeft, lampLeft.x, lampLeft.y, lampLeft.z);
    glUniform3f(uLampRight, lampRight.x, lampRight.y, lampRight.z);
    glUniform3f(uLampDir, lampDirection.x, lampDirection.y, lampDirection.z);
    glUniform3f(uRearLight, rearLight.x, rearLight.y, rearLight.z);
    glUniform1f(uBrakeOn, braking ? 1.0f : 0.0f);
    glUniform1f(uReverseOn, reversing ? 1.0f : 0.0f);
    glUniform1f(uEmissive, 0.0f);
}

void Renderer::draw(const GpuMesh& mesh, const Mat4& model) {
    if (mesh.indexCount <= 0) {
        return;
    }
    glBindVertexArray(mesh.vao);
    glUniformMatrix4fv(uModel, 1, GL_FALSE, model.m);
    glDrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, nullptr);
}

void Renderer::drawEmissive(const GpuMesh& mesh, const Mat4& model) {
    glUniform1f(uEmissive, 1.0f);
    draw(mesh, model);
    glUniform1f(uEmissive, 0.0f);
}

void Renderer::drawCourse(const GpuMesh& mesh, const Mat4& viewProj) {
    if (mesh.indexCount <= 0) return;
    glBindVertexArray(mesh.vao);
    Mat4 identity;
    glUniformMatrix4fv(uModel, 1, GL_FALSE, identity.m);
    for (const CourseChunk& chunk : mesh.chunks) {
        if (!courseChunkVisible(chunk, viewProj)) continue;
        glDrawElements(GL_TRIANGLES, chunk.count, GL_UNSIGNED_INT,
                       reinterpret_cast<const void*>(size_t(chunk.first) * sizeof(std::uint32_t)));
    }
}

void Renderer::drawSmoke(const float* verts, int vertCount, const unsigned* indices,
                         int indexCount, const Mat4& viewProj) {
    if (indexCount <= 0 || vertCount <= 0 || !smokeProgram) {
        return;
    }
    // 半透明：色は重ねて描き、深度は読むだけ（書き込まない）ので煙同士や
    // 車体と自然に重なる。不透明物の後に呼ぶこと。
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glUseProgram(smokeProgram);
    glUniformMatrix4fv(uSmokeViewProj, 1, GL_FALSE, viewProj.m);
    glBindVertexArray(smokeVao);
    glBindBuffer(GL_ARRAY_BUFFER, smokeVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertCount) * 6 * sizeof(float), verts,
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, smokeIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indexCount) * sizeof(unsigned),
                 indices, GL_DYNAMIC_DRAW);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(dummyVao);
}

void Renderer::drawGlow(const float* verts, int vertCount, const unsigned* indices,
                        int indexCount, const Mat4& viewProj, bool additive) {
    if (indexCount <= 0 || vertCount <= 0 || !glowProgram) {
        return;
    }
    // 加算合成で光の筋に見せる。深度は読むだけ（書き込まない）。
    // 不透明物と煙の後に呼ぶこと。
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, additive ? GL_ONE : GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glUseProgram(glowProgram);
    glUniformMatrix4fv(uGlowViewProj, 1, GL_FALSE, viewProj.m);
    glBindVertexArray(glowVao);
    glBindBuffer(GL_ARRAY_BUFFER, glowVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertCount) * 7 * sizeof(float), verts,
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, glowIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indexCount) * sizeof(unsigned),
                 indices, GL_DYNAMIC_DRAW);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(dummyVao);
}

void Renderer::beginHud() {
    hudVerts.clear();
    hudIndices.clear();
}

void Renderer::addQuad(float x0, float y0, float x1, float y1, float r, float g, float b, float a) {
    unsigned base = static_cast<unsigned>(hudVerts.size() / 6);
    float verts[] = {
        x0, y0, r, g, b, a, x1, y0, r, g, b, a, x1, y1, r, g, b, a, x0, y1, r, g, b, a,
    };
    hudVerts.insert(hudVerts.end(), verts, verts + 24);
    unsigned idx[] = {base, base + 1, base + 2, base, base + 2, base + 3};
    hudIndices.insert(hudIndices.end(), idx, idx + 6);
}

void Renderer::drawLine(float x0, float y0, float x1, float y1, float thickness, float r, float g, float b, float a) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) {
        return;
    }
    dx /= len;
    dy /= len;
    float px = -dy * thickness * 0.5f;
    float py = dx * thickness * 0.5f;
    unsigned base = static_cast<unsigned>(hudVerts.size() / 6);
    float verts[] = {
        x0 + px, y0 + py, r, g, b, a, x0 - px, y0 - py, r, g, b, a,
        x1 - px, y1 - py, r, g, b, a, x1 + px, y1 + py, r, g, b, a,
    };
    hudVerts.insert(hudVerts.end(), verts, verts + 24);
    unsigned idx[] = {base, base + 1, base + 2, base, base + 2, base + 3};
    hudIndices.insert(hudIndices.end(), idx, idx + 6);
}

void Renderer::drawText(float x, float y, float scale, const char* text, float r, float g, float b, float a) {
    float pen = x;
    for (const char* cursor = text; *cursor; ++cursor) {
        char c = *cursor;
        if (c >= 'a' && c <= 'z') {
            c = static_cast<char>(c - 'a' + 'A');
        }
        if (c == ' ') {
            pen += 4.0f * scale;
            continue;
        }
        int index = glyphIndex(c);
        if (index < 0) {
            pen += 6.0f * scale;
            continue;
        }
        const std::uint8_t* rows = kFont[index];
        for (int row = 0; row < 7; ++row) {
            for (int col = 0; col < 5; ++col) {
                if ((rows[row] & (0x10 >> col)) == 0) {
                    continue;
                }
                float x0 = pen + float(col) * scale;
                float y0 = y + float(row) * scale;
                addQuad(x0, y0, x0 + scale, y0 + scale, r, g, b, a);
            }
        }
        pen += 6.0f * scale;
    }
}

void Renderer::flushHud() {
    if (hudIndices.empty()) {
        return;
    }
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(hudProgram);
    Mat4 ortho = orthoTopLeft(float(width), float(height));
    glUniformMatrix4fv(uOrtho, 1, GL_FALSE, ortho.m);
    glBindVertexArray(hudVao);
    glBindBuffer(GL_ARRAY_BUFFER, hudVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(hudVerts.size() * sizeof(float)), hudVerts.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hudIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(hudIndices.size() * sizeof(unsigned)), hudIndices.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(hudIndices.size()), GL_UNSIGNED_INT, nullptr);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

GpuMesh Renderer::upload(const CpuMesh& mesh, bool chunked) {
    GpuMesh gpu;
    if (mesh.indices.empty()) {
        return gpu;
    }
    glGenVertexArrays(1, &gpu.vao);
    glGenBuffers(1, &gpu.vbo);
    glGenBuffers(1, &gpu.ibo);
    glBindVertexArray(gpu.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(float)), mesh.vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ibo);
    ChunkedCourse course;
    if (chunked) course = chunkCourse(mesh);
    const auto& indices = chunked ? course.indices : mesh.indices;
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(std::uint32_t)), indices.data(), GL_STATIC_DRAW);
    gpu.chunks = std::move(course.chunks);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));
    gpu.indexCount = static_cast<int>(mesh.indices.size());
    glBindVertexArray(dummyVao);
    return gpu;
}
