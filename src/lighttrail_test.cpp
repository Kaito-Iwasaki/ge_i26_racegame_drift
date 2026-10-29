// ブレーキランプ光跡（lighttrail.hpp）の検証ハーネス。SDL/GL を使わず、
// 記録・間引き・減衰・リボン生成だけを数値で見る。
//
//   g++ -std=c++17 -I. lighttrail_test.cpp -o lighttrail_test
//   ./lighttrail_test
#include "lighttrail.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* name, float value) {
    std::printf("[%s] %-46s value=%.3f\n", ok ? "PASS" : "FAIL", name, value);
    if (!ok) {
        ++failures;
    }
}

bool finite(const std::vector<float>& v) {
    for (float f : v) {
        if (!std::isfinite(f)) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    // 1. 記録とリボン生成
    {
        LightTrail trail;
        std::vector<float> verts;
        std::vector<unsigned> idx;
        trail.buildQuads(verts, idx);
        check(verts.empty() && idx.empty(), "empty trail builds nothing", 0);
        for (int i = 0; i < 10; ++i) {
            float z = i * 1.0f;
            trail.record({-0.5f, 0.7f, z}, {0.5f, 0.7f, z});
            trail.update(1.0f / 60.0f);
        }
        check(trail.nodes.size() == 10, "records kept (1 m spacing)", float(trail.nodes.size()));
        trail.buildQuads(verts, idx);
        // 9 区間 × 2 リボン（左右別々）× 4 頂点 = 72 頂点、9×2×6 = 108 インデックス
        check(verts.size() == 72 * 7, "quad vertex count (7 floats each)", float(verts.size()));
        check(idx.size() == 108, "quad index count", float(idx.size()));
        check(finite(verts), "all vertices finite", 1);
        bool idxOk = true;
        for (unsigned i : idx) {
            if (i >= verts.size() / 7) {
                idxOk = false;
            }
        }
        check(idxOk, "indices within vertex range", 1);
    }

    // 2. 近すぎる記録は間引く（停車中の踏みっぱなしで団子にならない）
    {
        LightTrail trail;
        for (int i = 0; i < 30; ++i) {
            trail.record({-0.5f, 0.7f, 0.0f}, {0.5f, 0.7f, 0.0f});
            trail.update(1.0f / 60.0f);
        }
        check(trail.nodes.size() == 1, "stationary records collapse to one node",
              float(trail.nodes.size()));
    }

    // 3. 時間で消える
    {
        LightTrail trail;
        for (int i = 0; i < 5; ++i) {
            trail.record({-0.5f, 0.7f, i * 1.0f}, {0.5f, 0.7f, i * 1.0f});
        }
        trail.update(LightTrail::kLife + 0.01f);
        check(trail.nodes.empty(), "nodes expire after kLife", float(trail.nodes.size()));
        std::vector<float> verts;
        std::vector<unsigned> idx;
        trail.buildQuads(verts, idx);
        check(verts.empty(), "expired trail builds nothing", 0);
    }

    // 4. アルファは古いほど小さく、範囲内
    {
        LightTrail trail;
        for (int i = 0; i < 6; ++i) {
            trail.record({-0.5f, 0.7f, i * 1.0f}, {0.5f, 0.7f, i * 1.0f});
            trail.update(0.08f);
        }
        std::vector<float> verts;
        std::vector<unsigned> idx;
        trail.buildQuads(verts, idx);
        float firstAlpha = verts[6];   // 最も古い頂点の alpha
        float lastAlpha = verts[verts.size() - 1];  // 最新の頂点の alpha
        check(firstAlpha < lastAlpha, "alpha fades with age", lastAlpha - firstAlpha);
        check(lastAlpha > 0.0f && lastAlpha <= 0.55f, "alpha within (0, 0.55]", lastAlpha);
    }

    // 5. clear で消える
    {
        LightTrail trail;
        trail.record({-0.5f, 0.7f, 0.0f}, {0.5f, 0.7f, 0.0f});
        trail.record({-0.5f, 0.7f, 1.0f}, {0.5f, 0.7f, 1.0f});
        trail.clear();
        check(trail.nodes.empty(), "clear empties the trail", 0);
    }

    // 6. 上限を超えない
    {
        LightTrail trail;
        for (int i = 0; i < 300; ++i) {
            trail.record({-0.5f, 0.7f, i * 1.0f}, {0.5f, 0.7f, i * 1.0f});
        }
        check(int(trail.nodes.size()) <= LightTrail::kMaxNodes, "node cap respected",
              float(trail.nodes.size()));
    }

    if (failures == 0) {
        std::printf("\nALL PASS\n");
        return 0;
    }
    std::printf("\n%d FAILURES\n", failures);
    return 1;
}
