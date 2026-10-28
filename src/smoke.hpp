#pragma once

#include "geom.hpp"

#include <cstdint>
#include <vector>

// ドリフト中のタイヤ煙。パーティクルのシミュレーションだけを担当する
// （SDL / OpenGL 非依存なので単体テストできる）。
// 描画は buildQuads でカメラ向きに広げた頂点を Renderer::drawSmoke に渡す。
// 頂点フォーマットは [x, y, z, u, v, alpha] の 6 float。
struct SmokeSystem {
    struct Particle {
        Vec3 pos{0, 0, 0};
        Vec3 vel{0, 0, 0};
        float age = 0;
        float life = 1;
        float seed = 0;  // 大きさの個体差
    };

    std::vector<Particle> particles;
    float spawnAcc = 0;
    std::uint32_t randState = 0x9E3779B9u;

    static constexpr int kMaxParticles = 420;

    float nextRand() {  // 0..1 の一様乱数（再現性のある簡易 LCG）
        randState = randState * 1664525u + 1013904223u;
        return float((randState >> 9) & 0x7FFFFFu) / float(0x800000u);
    }

    void clear() {
        particles.clear();
        spawnAcc = 0;
    }

    // 後輪の接地点（左右）から煙を出す。strength は 0..1（滑りの深さ・速さ）。
    void emit(Vec3 left, Vec3 right, Vec3 carVel, float strength, float dt) {
        if (strength <= 0.01f || dt <= 0.0f) {
            return;
        }
        spawnAcc += dt * (30.0f + 90.0f * strength);
        while (spawnAcc >= 1.0f) {
            spawnAcc -= 1.0f;
            if (static_cast<int>(particles.size()) >= kMaxParticles) {
                break;
            }
            Particle p;
            Vec3 base = nextRand() < 0.5f ? left : right;
            p.pos = base + Vec3{(nextRand() - 0.5f) * 0.5f, nextRand() * 0.10f,
                                (nextRand() - 0.5f) * 0.5f};
            p.vel = carVel * 0.30f + Vec3{(nextRand() - 0.5f) * 1.2f,
                                          0.7f + nextRand() * 0.9f,
                                          (nextRand() - 0.5f) * 1.2f};
            p.life = 0.9f + nextRand() * 0.8f;
            p.seed = nextRand();
            particles.push_back(p);
        }
    }

    void update(float dt) {
        if (dt < 0.0f) {
            dt = 0.0f;
        }
        float drag = std::exp(-0.7f * dt);
        for (Particle& p : particles) {
            p.age += dt;
            p.vel = p.vel * drag;
            p.vel.y += 0.55f * dt;  // 熱を持った煙はゆっくり浮き上がる
            p.pos = p.pos + p.vel * dt;
        }
        // 寿命が尽きた粒を消す（順序は保たなくてよいので末尾と入れ替える）。
        for (size_t i = 0; i < particles.size();) {
            if (particles[i].age >= particles[i].life) {
                particles[i] = particles.back();
                particles.pop_back();
            } else {
                ++i;
            }
        }
    }

    float sizeOf(const Particle& p) const {
        float t = p.life > 0.0f ? p.age / p.life : 1.0f;
        return (0.55f + 2.3f * t) * (0.8f + 0.4f * p.seed);
    }

    float alphaOf(const Particle& p) const {
        float t = p.life > 0.0f ? p.age / p.life : 1.0f;
        float fadeIn = clampf(t / 0.10f, 0.0f, 1.0f);
        return 0.50f * fadeIn * (1.0f - t);
    }

    // カメラの right / up 方向に各粒を四角形へ広げ、頂点とインデックスを詰める。
    void buildQuads(Vec3 camRight, Vec3 camUp, std::vector<float>& verts,
                    std::vector<unsigned>& indices) const {
        verts.clear();
        indices.clear();
        const float uvs[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        for (const Particle& p : particles) {
            float half = sizeOf(p) * 0.5f;
            float alpha = alphaOf(p);
            Vec3 r = camRight * half;
            Vec3 u = camUp * half;
            Vec3 corners[4] = {p.pos - r - u, p.pos + r - u, p.pos + r + u, p.pos - r + u};
            unsigned base = static_cast<unsigned>(verts.size() / 6);
            for (int i = 0; i < 4; ++i) {
                verts.insert(verts.end(), {corners[i].x, corners[i].y, corners[i].z,
                                           uvs[i][0], uvs[i][1], alpha});
            }
            indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        }
    }
};
