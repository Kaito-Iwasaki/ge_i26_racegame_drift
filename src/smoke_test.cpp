// タイヤ煙（smoke.hpp）の単体テスト。SDL / OpenGL には依存しない。
// ビルド: g++ -std=c++17 -I.. smoke_test.cpp -o smoke_test
#include "smoke.hpp"

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void check(bool ok, const char* name, float value) {
    std::printf("[%s] %-46s value=%.3f\n", ok ? "PASS" : "FAIL", name, value);
    if (!ok) {
        ++failures;
    }
}

bool finite(const SmokeSystem& smoke) {
    for (const auto& p : smoke.particles) {
        if (!std::isfinite(p.pos.x) || !std::isfinite(p.pos.y) || !std::isfinite(p.pos.z) ||
            !std::isfinite(p.vel.x) || !std::isfinite(p.vel.y) || !std::isfinite(p.vel.z)) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    const float dt = 1.0f / 60.0f;
    Vec3 left{0, 0.1f, 0};
    Vec3 right{1.6f, 0.1f, 0};
    Vec3 carVel{0, 0, 20};

    // 発生：強さ 1 で 0.5 秒出し続ければ、粒がたまる。
    {
        SmokeSystem smoke;
        for (int i = 0; i < 30; ++i) {
            smoke.emit(left, right, carVel, 1.0f, dt);
            smoke.update(dt);
        }
        check(smoke.particles.size() > 20, "smoke: particles accumulate",
              float(smoke.particles.size()));
        check(finite(smoke), "smoke: state stays finite", float(smoke.particles.size()));

        // 頂点化：1 粒 = 4 頂点（6 float）+ 6 インデックス。alpha は 0..0.5。
        std::vector<float> verts;
        std::vector<unsigned> indices;
        smoke.buildQuads({1, 0, 0}, {0, 1, 0}, verts, indices);
        check(verts.size() == smoke.particles.size() * 24, "smoke: quad vertex count",
              float(verts.size()));
        check(indices.size() == smoke.particles.size() * 6, "smoke: quad index count",
              float(indices.size()));
        float maxAlpha = 0;
        bool vertsFinite = true;
        for (size_t i = 5; i < verts.size(); i += 6) {
            maxAlpha = std::max(maxAlpha, verts[i]);
        }
        for (float v : verts) {
            vertsFinite = vertsFinite && std::isfinite(v);
        }
        check(maxAlpha > 0.0f && maxAlpha <= 0.5f, "smoke: alpha within range", maxAlpha);
        check(vertsFinite, "smoke: quad verts finite", maxAlpha);

        // 寿命：最大寿命（1.7 秒）を超えて進めれば全部消える。
        smoke.update(3.0f);
        check(smoke.particles.empty(), "smoke: particles die out",
              float(smoke.particles.size()));
        smoke.buildQuads({1, 0, 0}, {0, 1, 0}, verts, indices);
        check(verts.empty() && indices.empty(), "smoke: no quads when empty", 0.0f);
    }

    // 強さ 0 では出ない。
    {
        SmokeSystem smoke;
        smoke.emit(left, right, carVel, 0.0f, dt);
        smoke.update(dt);
        check(smoke.particles.empty(), "smoke: no emission at strength 0",
              float(smoke.particles.size()));
    }

    // 上限：長く出し続けても上限を超えない。
    {
        SmokeSystem smoke;
        for (int i = 0; i < 60 * 10; ++i) {
            smoke.emit(left, right, carVel, 1.0f, dt);
            smoke.update(dt);
        }
        check(int(smoke.particles.size()) <= SmokeSystem::kMaxParticles, "smoke: particle cap",
              float(smoke.particles.size()));
        check(finite(smoke), "smoke: capped state stays finite", float(smoke.particles.size()));
    }

    std::printf(failures == 0 ? "\nALL PASS\n" : "\n%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
