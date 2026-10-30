#pragma once

#include "geom.hpp"
#include "meshbuild.hpp"
#include "course_chunks.hpp"

#include <vector>

struct GpuMesh {
    unsigned vao = 0;
    unsigned vbo = 0;
    unsigned ibo = 0;
    int indexCount = 0;
    std::vector<CourseChunk> chunks;
};

struct Renderer {
    int width = 1280;
    int height = 720;

    bool init();
    void resize(int width, int height);
    void clearFrame();
    void drawSky(bool night);
    void beginMirror(int x, int y, int w, int h);
    void endMirror();
    void beginWorld(const Mat4& viewProj, Vec3 cameraPos, bool night, Vec3 lampLeft, Vec3 lampRight, Vec3 lampDirection, Vec3 rearLight, bool braking, bool reversing);
    void drawEmissive(const GpuMesh& mesh, const Mat4& model);
    void draw(const GpuMesh& mesh, const Mat4& model);
    void drawCourse(const GpuMesh& mesh, const Mat4& viewProj);
    // タイヤ煙などの半透明ビルボード用。頂点は [xyz, uv, alpha] の 6 float。
    void drawSmoke(const float* verts, int vertCount, const unsigned* indices, int indexCount,
                   const Mat4& viewProj);
    // 光の軌跡などの加算グロー用。頂点は [xyz, r, g, b, alpha] の 7 float。
    void drawGlow(const float* verts, int vertCount, const unsigned* indices, int indexCount,
                  const Mat4& viewProj, bool additive = true);
    void beginHud();
    void addQuad(float x0, float y0, float x1, float y1, float r, float g, float b, float a);
    void drawLine(float x0, float y0, float x1, float y1, float thickness, float r, float g, float b, float a);
    void drawText(float x, float y, float scale, const char* text, float r, float g, float b, float a);
    void flushHud();
    GpuMesh upload(const CpuMesh& mesh, bool chunked = false);

    unsigned worldProgram = 0;
    unsigned skyProgram = 0;
    unsigned hudProgram = 0;
    unsigned smokeProgram = 0;
    unsigned glowProgram = 0;
    unsigned dummyVao = 0;
    unsigned hudVao = 0;
    unsigned hudVbo = 0;
    unsigned hudIbo = 0;
    unsigned smokeVao = 0;
    unsigned smokeVbo = 0;
    unsigned smokeIbo = 0;
    unsigned glowVao = 0;
    unsigned glowVbo = 0;
    unsigned glowIbo = 0;
    int uViewProj = -1;
    int uModel = -1;
    int uCamPos = -1;
    int uNight = -1, uSkyNight = -1, uLampLeft = -1, uLampRight = -1, uLampDir = -1, uEmissive = -1, uRearLight = -1, uBrakeOn = -1, uReverseOn = -1;
    int uOrtho = -1;
    int uSmokeViewProj = -1;
    int uGlowViewProj = -1;
    std::vector<float> hudVerts;
    std::vector<unsigned> hudIndices;
};
