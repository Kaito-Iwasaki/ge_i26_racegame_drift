#pragma once

#include "geom.hpp"

#include <cstddef>
#include <vector>

// ブレーキランプの光の軌跡。点灯中に左右テールランプの世界座標を
// 履歴として記録し、時間で減衰するリボン（面）として描くための
// データを作る。SDL/GL 非依存（smoke.hpp と同じ流儀）。
// 頂点フォーマットは Renderer::drawGlow 用: [xyz, r, g, b, alpha] の 7 float。
struct LightTrail {
    struct Node {
        Vec3 left;
        Vec3 right;
        float age = 0;
    };

    static constexpr float kLife = 0.6f;      // 軌跡が残る時間 (s)
    static constexpr int kMaxNodes = 110;     // 履歴の上限（約2秒ぶん）
    static constexpr float kMinSpacing = 0.35f;  // これより近い記録は間引く (m)
    static constexpr float kRibbonHalf = 0.15f;  // 縦リボンの半分の高さ (m)

    std::vector<Node> nodes;  // 古い順 → 新しい順

    void clear() { nodes.clear(); }

    // 点灯中のフレームごとに呼ぶ。直前の記録点からほぼ動いていない
    // （停車中に踏みっぱなし等）場合は間引いて、軌跡が団子にならないようにする。
    void record(Vec3 left, Vec3 right) {
        if (!nodes.empty()) {
            const Node& last = nodes.back();
            Vec3 mid = (left + right) * 0.5f;
            Vec3 lastMid = (last.left + last.right) * 0.5f;
            if (length(mid - lastMid) < kMinSpacing) {
                // 位置は最新に寄せておく（ランプから軌跡が離れないように）。
                nodes.back().left = left;
                nodes.back().right = right;
                return;
            }
        }
        nodes.push_back(Node{left, right, 0.0f});
        if (static_cast<int>(nodes.size()) > kMaxNodes) {
            nodes.erase(nodes.begin());
        }
    }

    void update(float dt) {
        for (Node& n : nodes) {
            n.age += dt;
        }
        while (!nodes.empty() && nodes.front().age > kLife) {
            nodes.erase(nodes.begin());
        }
    }

    float nodeAlpha(const Node& n) const {
        float t = 1.0f - n.age / kLife;
        if (t < 0.0f) {
            t = 0.0f;
        }
        return t * t * 0.55f;
    }

    // 連続する記録点の間を面で繋ぐ。左右ランプ間を結ぶ水平シートに加え、
    // 各ランプ位置に縦リボン（上下 kRibbonHalf）を立てて、後方からも
    // 横からも光の筋が見えるようにする。
    void buildQuads(std::vector<float>& verts, std::vector<unsigned>& indices) const {
        verts.clear();
        indices.clear();
        if (nodes.size() < 2) {
            return;
        }
        const Vec3 color{1.0f, 0.10f, 0.06f};
        auto pushVert = [&](Vec3 p, float alpha) {
            verts.insert(verts.end(), {p.x, p.y, p.z, color.x, color.y, color.z, alpha});
        };
        auto quad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, float alphaA, float alphaB) {
            unsigned base = static_cast<unsigned>(verts.size() / 7);
            pushVert(a, alphaA);
            pushVert(b, alphaA);
            pushVert(c, alphaB);
            pushVert(d, alphaB);
            indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        };
        const Vec3 down{0, -kRibbonHalf, 0};
        const Vec3 up{0, kRibbonHalf, 0};
        for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
            const Node& a = nodes[i];
            const Node& b = nodes[i + 1];
            float aa = nodeAlpha(a);
            float ab = nodeAlpha(b);
            // 水平シート（左右ランプ間）
            quad(a.left, a.right, b.right, b.left, aa, ab);
            // 左ランプの縦リボン
            quad(a.left + down, a.left + up, b.left + up, b.left + down, aa, ab);
            // 右ランプの縦リボン
            quad(a.right + down, a.right + up, b.right + up, b.right + down, aa, ab);
        }
    }
};
