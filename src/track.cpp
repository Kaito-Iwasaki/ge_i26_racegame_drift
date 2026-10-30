#include "track.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace {

// オープンコースでは端の向こうを巻き戻さず、端点でクランプする。
int prevIndex(const Track& track, int index) {
    const int n = static_cast<int>(track.samples.size());
    if (track.closed) {
        return (index + n - 1) % n;
    }
    return index > 0 ? index - 1 : 0;
}

int nextIndex(const Track& track, int index) {
    const int n = static_cast<int>(track.samples.size());
    if (track.closed) {
        return (index + 1) % n;
    }
    return index + 1 < n ? index + 1 : n - 1;
}

// 実際に描く／判定する区間の数。周回は n 本、オープンは n-1 本。
int segmentCount(const Track& track) {
    const int n = static_cast<int>(track.samples.size());
    return track.closed ? n : n - 1;
}

Vec3 horizontalRight(const Track& track, int index) {
    const TrackSample& prev = track.samples[prevIndex(track, index)];
    const TrackSample& next = track.samples[nextIndex(track, index)];
    float tx = next.x - prev.x;
    float tz = next.z - prev.z;
    float len = std::sqrt(tx * tx + tz * tz);
    if (len < 1e-4f) {
        return {1, 0, 0};
    }
    tx /= len;
    tz /= len;
    return {tz, 0, -tx};
}

Vec3 roadNormal(const Track& track, int index) {
    const TrackSample& prev = track.samples[prevIndex(track, index)];
    const TrackSample& next = track.samples[nextIndex(track, index)];
    Vec3 tangent = normalize(Vec3{next.x - prev.x, next.y - prev.y, next.z - prev.z});
    Vec3 right = horizontalRight(track, index);
    right.y = track.samples[index].cross;
    right = normalize(right);
    Vec3 normal = normalize(cross(tangent, right));
    if (normal.y < 0) {
        normal = normal * -1.0f;
    }
    return normal;
}

Vec3 onRoad(const Track& track, int index, float lateral) {
    const TrackSample& sample = track.samples[index];
    Vec3 right = horizontalRight(track, index);
    return {
        sample.x + right.x * lateral,
        sample.y + lateral * sample.cross,
        sample.z + right.z * lateral,
    };
}

void addRibbon(CpuMesh& mesh, const Track& track, float lat0, float lat1, Vec3 color, bool flatOuter) {
    const int segs = segmentCount(track);
    for (int i = 0; i < segs; ++i) {
        int j = nextIndex(track, i);
        Vec3 a = onRoad(track, i, lat0);
        Vec3 b = onRoad(track, i, lat1);
        Vec3 c = onRoad(track, j, lat1);
        Vec3 d = onRoad(track, j, lat0);
        if (flatOuter) {
            // 芝は縁石の外側の高さで水平にする。
            float edge = (std::fabs(lat0) > std::fabs(lat1) ? lat1 : lat0);
            float sign = edge >= 0 ? 1.0f : -1.0f;
            float edgeLat = sign * (track.halfWidth + track.curbWidth);
            float y0 = track.samples[i].y + edgeLat * track.samples[i].cross;
            float y1 = track.samples[j].y + edgeLat * track.samples[j].cross;
            a.y = y0;
            b.y = y0;
            c.y = y1;
            d.y = y1;
        }
        Vec3 normal = normalize(roadNormal(track, i) + roadNormal(track, j));
        Vec3 tint = color;
        if (!flatOuter) {
            float shade = 0.94f + 0.08f * float((i * 13) % 5) / 5.0f;
            tint = color * shade;
        }
        mesh.addQuad(a, b, c, d, normal, tint);
    }
}

void addWalls(CpuMesh& mesh, const Track& track) {
    const int segs = segmentCount(track);
    const float inner = track.wallLateral();
    const float outer = inner + 0.4f;
    const float height = 0.9f;
    const Vec3 concrete{0.62f, 0.63f, 0.66f};
    for (int side = -1; side <= 1; side += 2) {
        float sign = float(side);
        for (int i = 0; i < segs; ++i) {
            int j = nextIndex(track, i);
            Vec3 a = onRoad(track, i, sign * inner);
            Vec3 b = onRoad(track, j, sign * inner);
            Vec3 c = onRoad(track, j, sign * outer);
            Vec3 d = onRoad(track, i, sign * outer);
            // 芝の高さに合わせる
            float edge = sign * (track.halfWidth + track.curbWidth);
            a.y = track.samples[i].y + edge * track.samples[i].cross;
            b.y = track.samples[j].y + edge * track.samples[j].cross;
            c.y = b.y;
            d.y = a.y;
            Vec3 topA = a + Vec3{0, height, 0};
            Vec3 topB = b + Vec3{0, height, 0};
            Vec3 topC = c + Vec3{0, height, 0};
            Vec3 topD = d + Vec3{0, height, 0};
            Vec3 inward{-sign * horizontalRight(track, i).x, 0, -sign * horizontalRight(track, i).z};
            mesh.addQuad(a, b, topB, topA, inward, concrete);
            mesh.addQuad(d, topD, topC, c, inward * -1.0f, concrete * 0.85f);
            mesh.addQuad(topA, topB, topC, topD, {0, 1, 0}, concrete * 1.05f);
        }
    }
}

// 路面から少し浮かせた帯（白線などのマーキング用。通常のリボンと
// 重なるとチラつくので 0.03 m 上げる）。dashLen > 0 のときは破線。
void addMarkingRibbon(CpuMesh& mesh, const Track& track, float lat0, float lat1, Vec3 color, float dashLen) {
    const int segs = segmentCount(track);
    for (int i = 0; i < segs; ++i) {
        if (dashLen > 0.0f) {
            float phase = std::fmod(track.samples[i].s, dashLen * 2.0f);
            if (phase > dashLen) {
                continue;
            }
        }
        int j = nextIndex(track, i);
        Vec3 a = onRoad(track, i, lat0);
        Vec3 b = onRoad(track, i, lat1);
        Vec3 c = onRoad(track, j, lat1);
        Vec3 d = onRoad(track, j, lat0);
        a.y += 0.03f;
        b.y += 0.03f;
        c.y += 0.03f;
        d.y += 0.03f;
        Vec3 normal = normalize(roadNormal(track, i) + roadNormal(track, j));
        mesh.addQuad(a, b, c, d, normal, color);
    }
}

// 峠のセンターライン（破線）と両側の白い路側帯。
void addRoadMarkings(CpuMesh& mesh, const Track& track) {
    const Vec3 lineWhite{0.84f, 0.84f, 0.80f};
    const float edgeIn = track.halfWidth - 0.50f;
    const float edgeOut = track.halfWidth - 0.20f;
    addMarkingRibbon(mesh, track, edgeIn, edgeOut, lineWhite, 0.0f);
    addMarkingRibbon(mesh, track, -edgeOut, -edgeIn, lineWhite, 0.0f);
    addMarkingRibbon(mesh, track, -0.13f, 0.13f, lineWhite, 5.0f);
}

// 日本の峠のガードレール: 支柱＋銀色のビーム1段。物理上の壁
// （wallLateral）の内側に面を合わせる。支柱の上にはオレンジの
// 視線誘導標（デリネーター）を付ける。
void addGuardrails(CpuMesh& mesh, const Track& track) {
    const int segs = segmentCount(track);
    const float face = track.wallLateral() - 0.05f;
    const Vec3 beam{0.76f, 0.78f, 0.80f};
    const Vec3 post{0.52f, 0.54f, 0.56f};
    const Vec3 reflector{0.98f, 0.48f, 0.07f};
    for (int side = -1; side <= 1; side += 2) {
        float sign = float(side);
        for (int i = 0; i < segs; ++i) {
            int j = nextIndex(track, i);
            auto baseAt = [&](int idx) {
                Vec3 p = onRoad(track, idx, sign * face);
                float edge = sign * (track.halfWidth + track.curbWidth);
                p.y = track.samples[idx].y + edge * track.samples[idx].cross;
                return p;
            };
            Vec3 a = baseAt(i);
            Vec3 b = baseAt(j);
            Vec3 inward{-sign * horizontalRight(track, i).x, 0, -sign * horizontalRight(track, i).z};
            float shade = 0.94f + 0.08f * float((i * 11) % 4) / 4.0f;
            mesh.addQuad(a + Vec3{0, 0.38f, 0}, b + Vec3{0, 0.38f, 0}, b + Vec3{0, 0.82f, 0},
                         a + Vec3{0, 0.82f, 0}, inward, beam * shade);
            if (i % 2 == 0) {
                // 支柱（路面側から見える板）とデリネーター
                Vec3 right = horizontalRight(track, i);
                Vec3 c = onRoad(track, i, sign * (face + 0.16f));
                c.y = a.y;
                Vec3 r = right * 0.09f;
                mesh.addQuad(c - r, c + r, c + r + Vec3{0, 0.80f, 0}, c - r + Vec3{0, 0.80f, 0},
                             inward, post);
                Vec3 d = c + Vec3{0, 0.52f, 0} - inward * 0.18f;
                Vec3 rr = right * 0.055f;
                mesh.addQuad(d - rr, d + rr, d + rr + Vec3{0, 0.20f, 0}, d - rr + Vec3{0, 0.20f, 0},
                             inward, reflector);
            }
        }
    }
}

// 決定的なハッシュ乱数（コースを読み直しても同じ森になるように）。
float treeRand(int k) {
    std::uint32_t h = std::uint32_t(k + 1) * 2654435761u;
    h ^= h >> 15;
    h *= 2246822519u;
    h ^= h >> 13;
    return float(h & 0xffffffu) / float(0x1000000u);
}

void addTree(CpuMesh& mesh, float cx, float cz, float baseY, int seed, float scale = 1.0f) {
    const float trunkH = (1.9f + treeRand(seed * 7 + 1) * 0.7f) * scale;
    const Vec3 bark{0.30f, 0.22f, 0.15f};
    mesh.addBox({cx - 0.16f * scale, baseY - 0.3f, cz - 0.16f * scale},
                {cx + 0.16f * scale, baseY + trunkH, cz + 0.16f * scale}, bark);
    float g = 0.26f + treeRand(seed * 7 + 2) * 0.13f;
    Vec3 leaf{0.09f + treeRand(seed * 7 + 3) * 0.05f, g, 0.10f + treeRand(seed * 7 + 4) * 0.05f};
    const float radius = (1.7f + treeRand(seed * 7 + 5) * 1.1f) * scale;
    const float height = (5.4f + treeRand(seed * 7 + 6) * 2.4f) * scale;
    // 杉のような円錐を2段重ねる
    const int segN = 7;
    for (int tier = 0; tier < 2; ++tier) {
        float t0 = tier == 0 ? 0.0f : 0.42f;
        float baseR = radius * (tier == 0 ? 1.0f : 0.62f);
        float yBase = baseY + trunkH * 0.55f + height * t0;
        float yTop = baseY + trunkH * 0.55f + height * (tier == 0 ? 0.72f : 1.0f);
        Vec3 apex{cx, yTop, cz};
        float rot = treeRand(seed * 13 + tier) * 6.2831853f;
        for (int k = 0; k < segN; ++k) {
            float a0 = rot + 6.2831853f * float(k) / float(segN);
            float a1 = rot + 6.2831853f * float(k + 1) / float(segN);
            Vec3 p0{cx + std::cos(a0) * baseR, yBase, cz + std::sin(a0) * baseR};
            Vec3 p1{cx + std::cos(a1) * baseR, yBase, cz + std::sin(a1) * baseR};
            Vec3 mid = (p0 + p1) * 0.5f - Vec3{cx, 0, cz};
            Vec3 n = normalize(Vec3{mid.x, 0.45f, mid.z});
            mesh.addTriangle(p0, p1, apex, n, leaf * (tier == 0 ? 1.0f : 1.12f));
        }
    }
}

// 路肩の外の斜面に沿って森を作る。近くの別の区間（折り返しの反対側など）に
// かぶる候補は捨てる。近い帯（密な林）と遠い帯（大きな木で奥行きを出す）の
// 2層にして、山の斜面が続いているように見せる。
void addTrees(CpuMesh& mesh, const Track& track) {
    const int n = static_cast<int>(track.samples.size());
    const float wall = track.wallLateral();
    const int segs = segmentCount(track);
    int seed = 0;
    auto tryPlant = [&](int i, float sign, float lat, float scale, int sd) {
        Vec3 right = horizontalRight(track, i);
        float cx = track.samples[i].x + right.x * sign * lat;
        float cz = track.samples[i].z + right.z * sign * lat;
        float edge = sign * (track.halfWidth + track.curbWidth);
        float baseY = track.samples[i].y + edge * track.samples[i].cross
                      - 2.2f - (lat - wall) * 0.136f - 0.25f;
        for (int j = 0; j < n; ++j) {
            float dx = track.samples[j].x - cx;
            float dz = track.samples[j].z - cz;
            float d2 = dx * dx + dz * dz;
            if (d2 < (wall + 2.5f) * (wall + 2.5f)) {
                return;  // どこかの路面・レールの内側には植えない
            }
            if (d2 < 24.0f * 24.0f && track.samples[j].y > baseY + 5.0f) {
                return;  // 自分より高い路面が近い＝その下敷きになる
            }
        }
        addTree(mesh, cx, cz, baseY, sd, scale);
    };
    for (int i = 0; i < segs; i += 8) {
        for (int side = -1; side <= 1; side += 2) {
            float sign = float(side);
            ++seed;
            if (treeRand(seed * 31 + 7) >= 0.70f) {
                float lat = wall + 3.5f + treeRand(seed * 31 + 8) * 30.0f;
                tryPlant(i, sign, lat, 0.9f + treeRand(seed * 31 + 9) * 0.3f, seed);
            }
            ++seed;
            if (treeRand(seed * 31 + 7) >= 0.85f) {
                float lat = wall + 32.0f + treeRand(seed * 31 + 8) * 45.0f;
                tryPlant(i, sign, lat, 1.6f + treeRand(seed * 31 + 9) * 0.8f, seed);
            }
        }
    }
    (void)n;
}

// 遠景の山並み: コースの周囲に大きな稜線シルエットを置いて「山の中」に
// いる感じを出す。コースから十分遠く、地形（路面・斜面）には一切
// 干渉しないので、折り返し区間でも破綻しない。
void addBackdropRidges(CpuMesh& mesh, const Track& track) {
    if (track.samples.empty()) {
        return;
    }
    float minX = track.samples[0].x, maxX = minX;
    float minZ = track.samples[0].z, maxZ = minZ;
    float minY = track.samples[0].y;
    for (const TrackSample& s : track.samples) {
        minX = std::min(minX, s.x); maxX = std::max(maxX, s.x);
        minZ = std::min(minZ, s.z); maxZ = std::max(maxZ, s.z);
        minY = std::min(minY, s.y);
    }
    const float cx = (minX + maxX) * 0.5f;
    const float cz = (minZ + maxZ) * 0.5f;
    const float baseR = std::max(maxX - minX, maxZ - minZ) * 0.5f;
    const float baseY = minY - 6.0f;
    const int peaks = 16;
    for (int k = 0; k < peaks; ++k) {
        float ang = 6.2831853f * float(k) / float(peaks);
        float dist = baseR + 260.0f + treeRand(k * 17 + 1) * 260.0f;
        float px = cx + std::cos(ang) * dist;
        float pz = cz + std::sin(ang) * dist;
        float h = 130.0f + treeRand(k * 17 + 2) * 130.0f;
        float r = 200.0f + treeRand(k * 17 + 3) * 160.0f;
        float shade = 0.85f + treeRand(k * 17 + 4) * 0.3f;
        Vec3 col{0.30f * shade, 0.42f * shade, 0.34f * shade};
        Vec3 apex{px, baseY + h, pz};
        const int segN = 9;
        float rot = treeRand(k * 17 + 5) * 6.2831853f;
        for (int sgm = 0; sgm < segN; ++sgm) {
            float a0 = rot + 6.2831853f * float(sgm) / float(segN);
            float a1 = rot + 6.2831853f * float(sgm + 1) / float(segN);
            Vec3 p0{px + std::cos(a0) * r, baseY, pz + std::sin(a0) * r};
            Vec3 p1{px + std::cos(a1) * r, baseY, pz + std::sin(a1) * r};
            Vec3 mid = (p0 + p1) * 0.5f - Vec3{px, 0, pz};
            Vec3 nrm = normalize(Vec3{mid.x, 0.5f, mid.z});
            mesh.addTriangle(p0, p1, apex, nrm, col);
        }
    }
}

void addCurbs(CpuMesh& mesh, const Track& track) {
    const int segs = segmentCount(track);
    const Vec3 red{0.75f, 0.09f, 0.08f};
    const Vec3 white{0.90f, 0.90f, 0.92f};
    for (int side = -1; side <= 1; side += 2) {
        float sign = float(side);
        float inner = sign * track.halfWidth;
        float outer = sign * (track.halfWidth + track.curbWidth);
        for (int i = 0; i < segs; ++i) {
            int j = nextIndex(track, i);
            bool paint = (static_cast<int>(track.samples[i].s / 3.0f) % 2) == 0;
            Vec3 color = paint ? red : white;
            Vec3 a = onRoad(track, i, inner);
            Vec3 b = onRoad(track, i, outer);
            Vec3 c = onRoad(track, j, outer);
            Vec3 d = onRoad(track, j, inner);
            Vec3 normal = normalize(roadNormal(track, i) + roadNormal(track, j));
            mesh.addQuad(a, b, c, d, normal, color);
        }
    }
}

// チェッカーのラインと門を、サンプル ia（接線は ia→ib）に出す。
// スタートは (0, 1)、オープンコースのゴールは (最後, 最後-1) で使う。
void addStartLine(CpuMesh& mesh, const Track& track, int ia, int ib) {
    if (track.samples.size() < 2) {
        return;
    }
    Vec3 right = horizontalRight(track, ia);
    float yaw = track.spawnYaw;
    const TrackSample& a = track.samples[ia];
    const TrackSample& b = track.samples[ib];
    float tx = b.x - a.x;
    float tz = b.z - a.z;
    float len = std::sqrt(tx * tx + tz * tz);
    if (len > 1e-4f) {
        tx /= len;
        tz /= len;
        yaw = std::atan2(tx, tz);
    }
    Vec3 forward{std::sin(yaw), 0, std::cos(yaw)};
    Vec3 center{a.x, a.y, a.z};
    const int cells = 8;
    const float depth = 0.8f;
    for (int i = 0; i < cells; ++i) {
        float lat0 = -track.halfWidth + track.halfWidth * 2.0f * (float(i) / cells);
        float lat1 = -track.halfWidth + track.halfWidth * 2.0f * (float(i + 1) / cells);
        Vec3 color = (i % 2 == 0) ? Vec3{0.92f, 0.92f, 0.94f} : Vec3{0.05f, 0.05f, 0.06f};
        Vec3 p0 = center + right * lat0;
        Vec3 p1 = center + right * lat1;
        Vec3 p2 = p1 + forward * depth;
        Vec3 p3 = p0 + forward * depth;
        p0.y = a.y + lat0 * a.cross + 0.04f;
        p1.y = a.y + lat1 * a.cross + 0.04f;
        p2.y = p1.y;
        p3.y = p0.y;
        mesh.addQuad(p0, p1, p2, p3, {0, 1, 0}, color);
    }

    // スタートの門。位置が分かるようにする。
    const float pillar = track.halfWidth + 1.1f;
    const Vec3 steel{0.22f, 0.24f, 0.28f};
    for (int side = -1; side <= 1; side += 2) {
        Vec3 base = center + right * (float(side) * pillar);
        base.y = a.y;
        Vec3 axisX = right * 0.28f;
        Vec3 axisZ = forward * 0.28f;
        Vec3 y0 = base;
        Vec3 y1 = base + Vec3{0, 6.4f, 0};
        mesh.addQuad(y0 - axisX - axisZ, y0 + axisX - axisZ, y1 + axisX - axisZ, y1 - axisX - axisZ, forward * -1.0f, steel);
        mesh.addQuad(y0 - axisX + axisZ, y1 - axisX + axisZ, y1 + axisX + axisZ, y0 + axisX + axisZ, forward, steel);
        mesh.addQuad(y0 - axisX - axisZ, y1 - axisX - axisZ, y1 - axisX + axisZ, y0 - axisX + axisZ, right * -1.0f, steel);
        mesh.addQuad(y0 + axisX - axisZ, y0 + axisX + axisZ, y1 + axisX + axisZ, y1 + axisX - axisZ, right, steel);
    }
    Vec3 left = center + right * -pillar + Vec3{0, 6.2f, 0};
    Vec3 rightTop = center + right * pillar + Vec3{0, 6.2f, 0};
    mesh.addQuad(left, rightTop, rightTop + Vec3{0, 0.4f, 0}, left + Vec3{0, 0.4f, 0}, forward, steel);
}

void addPit(CpuMesh& mesh, const std::vector<Vec3>& pit) {
    if (pit.size() < 2) {
        return;
    }
    const float half = 4.0f;
    const Vec3 color{0.24f, 0.24f, 0.26f};
    for (size_t i = 0; i + 1 < pit.size(); ++i) {
        Vec3 a = pit[i];
        Vec3 b = pit[i + 1];
        float tx = b.x - a.x;
        float tz = b.z - a.z;
        float len = std::sqrt(tx * tx + tz * tz);
        if (len < 0.05f) {
            continue;
        }
        tx /= len;
        tz /= len;
        Vec3 right{tz, 0, -tx};
        Vec3 a0 = a + right * -half;
        Vec3 a1 = a + right * half;
        Vec3 b1 = b + right * half;
        Vec3 b0 = b + right * -half;
        a0.y = a.y + 0.03f;
        a1.y = a.y + 0.03f;
        b0.y = b.y + 0.03f;
        b1.y = b.y + 0.03f;
        mesh.addQuad(a0, a1, b1, b0, {0, 1, 0}, color);
    }
}

void addGround(CpuMesh& mesh, const Track& track, Vec3 color) {
    float minX = track.samples[0].x;
    float maxX = minX;
    float minZ = track.samples[0].z;
    float maxZ = minZ;
    float minY = track.samples[0].y;
    for (const TrackSample& sample : track.samples) {
        minX = std::min(minX, sample.x);
        maxX = std::max(maxX, sample.x);
        minZ = std::min(minZ, sample.z);
        maxZ = std::max(maxZ, sample.z);
        minY = std::min(minY, sample.y);
    }
    const float pad = 120.0f;
    float y = minY - 4.0f;
    Vec3 a{minX - pad, y, minZ - pad};
    Vec3 b{maxX + pad, y, minZ - pad};
    Vec3 c{maxX + pad, y, maxZ + pad};
    Vec3 d{minX - pad, y, maxZ + pad};
    mesh.addQuad(a, d, c, b, {0, 1, 0}, color);
}

// オープンコース（山道）用: 路面が谷底の平面から浮いて見えないよう、
// 路肩の外側に沿って斜面を付ける。外側へ緩く下げる。
// ヘアピンでは別の脚の路面が斜面の幅（最大 55 m）の中に入る。その路面より
// 斜面が高いと路面を覆い隠してしまうので、サンプルごとに斜面の外端を
// 「別区間の路面に届く手前」までに縮める（高さの関係で路面の下を通る場合は縮めない）。
void addMountain(CpuMesh& mesh, const Track& track) {
    const int n = static_cast<int>(track.samples.size());
    const int segs = segmentCount(track);
    const float inner = track.halfWidth + track.curbWidth + track.runoff;
    const float fullWidth = 55.0f;
    const float fullOuter = inner + fullWidth;
    const float startDrop = 2.2f;
    const float gradient = (9.7f - 2.2f) / fullWidth;  // 外側へ進む 1 m あたりの下がり
    const float footprint = inner;                      // 別区間の路面＋路肩の半幅
    const float ownSkip = 25.0f;                        // 自分の近傍（同じ脚）は判定しない (m)
    const Vec3 base{0.13f, 0.30f, 0.12f};

    for (int side = -1; side <= 1; side += 2) {
        float sign = float(side);
        const float edge = sign * (track.halfWidth + track.curbWidth);
        std::vector<float> outerLat(n, fullOuter);
        for (int i = 0; i < n; ++i) {
            const TrackSample& si = track.samples[i];
            Vec3 r = horizontalRight(track, i) * sign;
            float edgeY = si.y + edge * si.cross;
            float limit = fullOuter;
            for (int j = 0; j < n; ++j) {
                const TrackSample& sj = track.samples[j];
                if (std::fabs(sj.s - si.s) < ownSkip) {
                    continue;
                }
                float dx = sj.x - si.x;
                float dz = sj.z - si.z;
                float a = dx * r.x + dz * r.z;  // 斜面の向き（外側）への距離
                if (a <= 0.0f || a > fullOuter + footprint) {
                    continue;
                }
                float b2 = dx * dx + dz * dz - a * a;  // 向きに直交する距離の二乗
                if (b2 >= footprint * footprint) {
                    continue;
                }
                // 相手の路面（半径 footprint の円で近似）に斜面が入り始める位置
                float stop = a - std::sqrt(footprint * footprint - b2);
                float slopeY = edgeY - startDrop - std::max(0.0f, stop - inner) * gradient;
                // 相手の路面と同じか高いなら覆ってしまう。1.5 m の余裕を見て縮める。
                if (slopeY > sj.y - 1.5f) {
                    limit = std::min(limit, std::max(stop, inner));
                }
            }
            outerLat[i] = limit;
        }
        // 区間ごとに外端がギザギザしないよう、前後 3 サンプルの最小に揃える。
        std::vector<float> smooth(n);
        for (int i = 0; i < n; ++i) {
            float m = outerLat[i];
            for (int k = std::max(0, i - 3); k <= std::min(n - 1, i + 3); ++k) {
                m = std::min(m, outerLat[k]);
            }
            smooth[i] = m;
        }
        for (int i = 0; i < segs; ++i) {
            int j = nextIndex(track, i);
            auto pointAt = [&](int idx, float lat, float drop) {
                Vec3 p = onRoad(track, idx, lat);
                const TrackSample& smp = track.samples[idx];
                p.y = smp.y + edge * smp.cross - drop;
                return p;
            };
            auto dropAt = [&](int idx) { return startDrop + (smooth[idx] - inner) * gradient; };
            Vec3 a = pointAt(i, sign * inner, startDrop);
            Vec3 b = pointAt(i, sign * smooth[i], dropAt(i));
            Vec3 c = pointAt(j, sign * smooth[j], dropAt(j));
            Vec3 d = pointAt(j, sign * inner, startDrop);
            float shade = 0.92f + 0.10f * float((i * 7) % 5) / 5.0f;
            mesh.addQuad(a, b, c, d, {0, 1, 0}, base * shade);
        }
    }
}


// ---------------------------------------------------------------------------
// 山岳地形と森（オープンの峠コース用）
//
// コースの標高に沿った高さ場を作る。路面の近くは路面より必ず下に置き、
// 離れるほど斜面が立ち上がり、さらに遠くは稜線が視界を塞ぐ高さまで上がる。
// 木はこの高さ場に植えるので、斜面にも自然に立つ。
// ---------------------------------------------------------------------------

float hash2(int x, int z) {
    std::uint32_t h = std::uint32_t(x) * 374761393u + std::uint32_t(z) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return float(h & 0xffffffu) / float(0x1000000u);
}

float valueNoise(float x, float z) {
    int xi = int(std::floor(x));
    int zi = int(std::floor(z));
    float fx = x - float(xi);
    float fz = z - float(zi);
    float ux = fx * fx * (3.0f - 2.0f * fx);
    float uz = fz * fz * (3.0f - 2.0f * fz);
    float a = hash2(xi, zi), b = hash2(xi + 1, zi);
    float c = hash2(xi, zi + 1), d = hash2(xi + 1, zi + 1);
    return (a + (b - a) * ux) * (1.0f - uz) + (c + (d - c) * ux) * uz;
}

float fbm(float x, float z, int octaves) {
    float sum = 0, amp = 0.5f, norm = 0;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * valueNoise(x, z);
        norm += amp;
        x = x * 2.03f + 17.0f;
        z = z * 2.03f + 31.0f;
        amp *= 0.5f;
    }
    return sum / norm;
}

float smooth01(float a, float b, float v) {
    float t = clampf((v - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// 路面サンプルの空間ハッシュ（最寄りの路面を速く引く）。
struct SampleGrid {
    float minX = 0, minZ = 0, cell = 48.0f;
    int nx = 1, nz = 1;
    std::vector<std::vector<int>> bins;
    void build(const Track& track) {
        float maxX = track.samples[0].x, maxZ = track.samples[0].z;
        minX = maxX;
        minZ = maxZ;
        for (const TrackSample& s : track.samples) {
            minX = std::min(minX, s.x); maxX = std::max(maxX, s.x);
            minZ = std::min(minZ, s.z); maxZ = std::max(maxZ, s.z);
        }
        nx = int((maxX - minX) / cell) + 1;
        nz = int((maxZ - minZ) / cell) + 1;
        bins.assign(size_t(nx) * size_t(nz), {});
        for (int i = 0; i < int(track.samples.size()); ++i) {
            int ix = clampi(int((track.samples[i].x - minX) / cell), 0, nx - 1);
            int iz = clampi(int((track.samples[i].z - minZ) / cell), 0, nz - 1);
            bins[size_t(iz) * size_t(nx) + size_t(ix)].push_back(i);
        }
    }
    static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
    // Find only points within the scenery radius, without scanning the whole track.
    int nearestWithin(const Track& track, float x, float z, float radius, float& d2Out) const {
        const int x0 = std::max(0, int(std::floor((x - radius - minX) / cell)));
        const int x1 = std::min(nx - 1, int(std::floor((x + radius - minX) / cell)));
        const int z0 = std::max(0, int(std::floor((z - radius - minZ) / cell)));
        const int z1 = std::min(nz - 1, int(std::floor((z + radius - minZ) / cell)));
        int best = -1;
        float bestD2 = radius * radius;
        for (int iz = z0; iz <= z1; ++iz) {
            for (int ix = x0; ix <= x1; ++ix) {
                for (int idx : bins[size_t(iz) * size_t(nx) + size_t(ix)]) {
                    const float dx = track.samples[idx].x - x, dz = track.samples[idx].z - z;
                    const float d2 = dx * dx + dz * dz;
                    if (d2 < bestD2) { bestD2 = d2; best = idx; }
                }
            }
        }
        d2Out = bestD2;
        return best;
    }
    // 最寄りの路面サンプル番号と、その距離の二乗。遠い点は全体を総当たりで探す。
    int nearest(const Track& track, float x, float z, float& d2Out) const {
        int cx = clampi(int(std::floor((x - minX) / cell)), -1000000, 1000000);
        int cz = clampi(int(std::floor((z - minZ) / cell)), -1000000, 1000000);
        int best = -1;
        float bestD2 = 1.0e30f;
        for (int ring = 0; ring < 6; ++ring) {
            for (int dz = -ring; dz <= ring; ++dz) {
                for (int dx = -ring; dx <= ring; ++dx) {
                    if (std::max(std::abs(dx), std::abs(dz)) != ring) continue;
                    int ix = cx + dx, iz = cz + dz;
                    if (ix < 0 || iz < 0 || ix >= nx || iz >= nz) continue;
                    for (int idx : bins[size_t(iz) * size_t(nx) + size_t(ix)]) {
                        float ex = track.samples[idx].x - x, ez = track.samples[idx].z - z;
                        float d2 = ex * ex + ez * ez;
                        if (d2 < bestD2) { bestD2 = d2; best = idx; }
                    }
                }
            }
            // 次のリングは最低でも ring*cell 以上離れている。見つかっていて十分近ければ打ち切る。
            if (best >= 0 && bestD2 < (float(ring) * cell) * (float(ring) * cell)) break;
        }
        if (best < 0) {
            for (int idx = 0; idx < int(track.samples.size()); ++idx) {
                float ex = track.samples[idx].x - x, ez = track.samples[idx].z - z;
                float d2 = ex * ex + ez * ez;
                if (d2 < bestD2) { bestD2 = d2; best = idx; }
            }
        }
        d2Out = bestD2;
        return best;
    }
};

struct Terrain {
    float minX = 0, minZ = 0;
    float cell = 10.0f;
    int nx = 0, nz = 0;
    std::vector<float> h;
    std::vector<float> roadDistance2;
    float at(int ix, int iz) const {
        ix = ix < 0 ? 0 : (ix >= nx ? nx - 1 : ix);
        iz = iz < 0 ? 0 : (iz >= nz ? nz - 1 : iz);
        return h[size_t(iz) * size_t(nx) + size_t(ix)];
    }
    // メッシュと同じ三角形分割で高さを求める。
    float sample(float x, float z) const {
        float gx = (x - minX) / cell, gz = (z - minZ) / cell;
        int ix = int(std::floor(gx)), iz = int(std::floor(gz));
        float fx = gx - float(ix), fz = gz - float(iz);
        float a = at(ix, iz), b = at(ix + 1, iz), c = at(ix + 1, iz + 1), d = at(ix, iz + 1);
        // 分割は a-c の対角線。
        if (fx >= fz) return a + (b - a) * fx + (c - b) * fz;
        return a + (d - a) * fz + (c - d) * fx;
    }
};

// 路面の端の高さ（横断勾配込み。路面リボンの外端の高さと同じ）。
float roadSurfaceAt(const Track& track, int idx, float x, float z) {
    Vec3 r = horizontalRight(track, idx);
    float lat = (x - track.samples[idx].x) * r.x + (z - track.samples[idx].z) * r.z;
    float edge = track.halfWidth + track.curbWidth;
    return track.samples[idx].y + clampf(lat, -edge, edge) * track.samples[idx].cross;
}

constexpr float kTerrainCell = 10.0f;
constexpr float kTerrainRadius = 180.0f;
constexpr float kTerrainMargin = kTerrainRadius + 2.0f * kTerrainCell;

Terrain buildTerrain(const Track& track, const SampleGrid& grid) {
    Terrain t;
    float minX = track.samples[0].x, maxX = minX, minZ = track.samples[0].z, maxZ = minZ;
    for (const TrackSample& s : track.samples) {
        minX = std::min(minX, s.x); maxX = std::max(maxX, s.x);
        minZ = std::min(minZ, s.z); maxZ = std::max(maxZ, s.z);
    }
    t.cell = kTerrainCell;
    t.minX = minX - kTerrainMargin;
    t.minZ = minZ - kTerrainMargin;
    t.nx = int((maxX - minX + 2.0f * kTerrainMargin) / t.cell) + 1;
    t.nz = int((maxZ - minZ + 2.0f * kTerrainMargin) / t.cell) + 1;
    t.h.assign(size_t(t.nx) * size_t(t.nz), 0.0f);
    t.roadDistance2.assign(t.h.size(), 1.0e30f);

    // 遠くの高さを滑らかにつなぐための間引き（約 24 m 間隔）。
    std::vector<int> coarse;
    for (int i = 0; i < int(track.samples.size()); i += 6) coarse.push_back(i);
    const float wall = track.wallLateral();

    for (int iz = 0; iz < t.nz; ++iz) {
        for (int ix = 0; ix < t.nx; ++ix) {
            float x = t.minX + float(ix) * t.cell;
            float z = t.minZ + float(iz) * t.cell;
            float d2;
            // A one-cell halo is retained for normals and interpolation at the edge.
            int near = grid.nearestWithin(track, x, z, kTerrainRadius + 2.0f * t.cell, d2);
            if (near < 0) continue;
            t.roadDistance2[size_t(iz) * size_t(t.nx) + size_t(ix)] = d2;
            float dm = std::sqrt(d2);
            float hStar = roadSurfaceAt(track, near, x, z);  // 最寄りの路面の高さ

            // 路面の高さを距離で重みづけして滑らかに補間（尾根・谷の骨格）。
            double wsum = 0, hsum = 0;
            for (int idx : coarse) {
                float ex = track.samples[idx].x - x, ez = track.samples[idx].z - z;
                float q = ex * ex + ez * ez + 25.0f;
                double w = 1.0 / (double(q) * std::sqrt(std::sqrt(double(q))));
                wsum += w;
                hsum += w * track.samples[idx].y;
            }
            float cs = float(hsum / wsum);

            // 近い起伏: 路肩の外から斜面が立ち上がる（谷側は少し下がる場所もある）。
            float nNear = fbm(x / 170.0f, z / 170.0f, 3);
            float amp = -16.0f + 78.0f * smooth01(0.18f, 0.78f, nNear);
            float u = std::max(dm - 14.0f, 0.0f);
            float near_rise = amp * (1.0f - std::exp(-u / 85.0f)) * smooth01(0.0f, 16.0f, u);
            // 遠い起伏: 外へ行くほど稜線が高くなり、遠くまで見通せないようにする。
            float nFar = fbm(x / 650.0f + 5.0f, z / 650.0f + 9.0f, 3);
            float far_rise = smooth01(120.0f, 820.0f, dm) * (190.0f + 300.0f * nFar);
            // 細かい凹凸（路面の近くは出さない）。
            float bump = (fbm(x / 28.0f, z / 28.0f, 2) - 0.5f) * 9.0f * smooth01(20.0f, 70.0f, dm);

            // 路面の近くは最寄りの路面の高さそのもの、離れるほど滑らかな骨格 cs へ移る。
            float base = hStar + (cs - hStar) * smooth01(10.0f, 40.0f, dm);
            float height = base - 0.9f + near_rise + far_rise + bump;
            // 路面を覆わないための安全弁: 最寄りの路面より少し下から、最大でも約 42° の勾配まで。
            if (dm < 60.0f) {
                height = std::min(height, hStar - 0.9f + std::max(dm - 10.0f, 0.0f) * 0.9f);
            }
            (void)wall;
            t.h[size_t(iz) * size_t(t.nx) + size_t(ix)] = height;
        }
    }
    return t;
}

void addTerrainMesh(CpuMesh& mesh, const Terrain& t) {
    const float radius2 = kTerrainRadius * kTerrainRadius;
    const size_t count = size_t(t.nx) * size_t(t.nz);
    std::vector<unsigned char> used(count, 0);
    auto index = [&](int x, int z) { return size_t(z) * size_t(t.nx) + size_t(x); };
    auto includeCell = [&](int x, int z) {
        const size_t a = index(x,z), b = index(x+1,z), c = index(x+1,z+1), d = index(x,z+1);
        const float missing = 1.0e29f;
        if (t.roadDistance2[a] >= missing || t.roadDistance2[b] >= missing ||
            t.roadDistance2[c] >= missing || t.roadDistance2[d] >= missing) return false;
        return std::min(std::min(t.roadDistance2[a], t.roadDistance2[b]),
                        std::min(t.roadDistance2[c], t.roadDistance2[d])) <= radius2;
    };
    for (int z = 0; z + 1 < t.nz; ++z) for (int x = 0; x + 1 < t.nx; ++x) {
        if (!includeCell(x,z)) continue;
        used[index(x,z)] = used[index(x+1,z)] = used[index(x+1,z+1)] = used[index(x,z+1)] = 1;
    }
    std::vector<std::uint32_t> ids(count, 0);
    auto heightNear = [&](int x, int z, float fallback) {
        x = std::max(0, std::min(x,t.nx-1)); z = std::max(0, std::min(z,t.nz-1));
        const size_t k = index(x,z);
        return t.roadDistance2[k] < 1.0e29f ? t.h[k] : fallback;
    };
    for (int z = 0; z < t.nz; ++z) for (int x = 0; x < t.nx; ++x) {
        const size_t k = index(x,z);
        if (!used[k]) continue;
        const float wx = t.minX + float(x)*t.cell, wz = t.minZ + float(z)*t.cell, h = t.h[k];
        const float dhx = (heightNear(x+1,z,h) - heightNear(x-1,z,h)) / (2.0f*t.cell);
        const float dhz = (heightNear(x,z+1,h) - heightNear(x,z-1,h)) / (2.0f*t.cell);
        const Vec3 n = normalize(Vec3{-dhx,1.0f,-dhz});
        const float slope = std::sqrt(dhx*dhx+dhz*dhz);
        const float v1 = fbm(wx/60.0f,wz/60.0f,3);
        const float v2 = fbm(wx/9.0f+3.0f,wz/9.0f+7.0f,2);
        const Vec3 forest{0.07f+0.05f*v1,0.17f+0.11f*v1+0.03f*v2,0.06f+0.03f*v1};
        const Vec3 rock{0.30f+0.06f*v2,0.27f+0.05f*v2,0.22f+0.04f*v2};
        const float blend = smooth01(0.85f,1.25f,slope);
        ids[k] = static_cast<std::uint32_t>(mesh.vertices.size()/9);
        mesh.addVertex({wx,h,wz},n,forest*(1.0f-blend)+rock*blend);
    }
    for (int z = 0; z + 1 < t.nz; ++z) for (int x = 0; x + 1 < t.nx; ++x) {
        if (!includeCell(x,z)) continue;
        const auto a=ids[index(x,z)], b=ids[index(x+1,z)], c=ids[index(x+1,z+1)], d=ids[index(x,z+1)];
        // The grid increases towards +x,+z: clockwise in xz gives an upward normal.
        mesh.addRawTriangle(a,c,b);
        mesh.addRawTriangle(a,d,c);
    }
}

// 幹（先細りの四角柱）。頂点 8・三角形 8 で済ませる。
void addTrunk(CpuMesh& mesh, float cx, float cz, float yBot, float yTop, float rBot, float rTop,
              float rot, Vec3 bark) {
    std::uint32_t base = static_cast<std::uint32_t>(mesh.vertices.size() / 9);
    for (int ring = 0; ring < 2; ++ring) {
        float r = ring == 0 ? rBot : rTop;
        float y = ring == 0 ? yBot : yTop;
        for (int k = 0; k < 4; ++k) {
            float a = rot + 1.5707963f * float(k);
            mesh.addVertex({cx + std::cos(a) * r, y, cz + std::sin(a) * r},
                           normalize(Vec3{std::cos(a), 0.1f, std::sin(a)}), bark * (ring == 0 ? 0.8f : 1.0f));
        }
    }
    for (int k = 0; k < 4; ++k) {
        int k2 = (k + 1) % 4;
        std::uint32_t a = base + std::uint32_t(k), b = base + std::uint32_t(k2);
        std::uint32_t c = base + 4 + std::uint32_t(k2), d = base + 4 + std::uint32_t(k);
        Vec3 pa{mesh.vertices[a * 9], mesh.vertices[a * 9 + 1], mesh.vertices[a * 9 + 2]};
        Vec3 pb{mesh.vertices[b * 9], mesh.vertices[b * 9 + 1], mesh.vertices[b * 9 + 2]};
        Vec3 pd{mesh.vertices[d * 9], mesh.vertices[d * 9 + 1], mesh.vertices[d * 9 + 2]};
        Vec3 outward = (pa + pb) * 0.5f - Vec3{cx, 0, cz};
        if (dot(cross(pb - pa, pd - pa), outward) < 0.0f) {
            mesh.addRawTriangle(a, d, b); mesh.addRawTriangle(b, d, c);
        } else {
            mesh.addRawTriangle(a, b, d); mesh.addRawTriangle(b, c, d);
        }
    }
}

// 針葉樹（杉）。高さは 17〜27 m。lod 0 は近景（3 段）、1 は中景（2 段）、2 は遠景（1 段）。
void addConifer(CpuMesh& mesh, float cx, float cz, float baseY, int seed, float height, int lod) {
    float radius = height * (0.185f + 0.045f * treeRand(seed * 5 + 1));
    float g = treeRand(seed * 5 + 2);
    Vec3 leaf{0.05f + 0.06f * g, 0.17f + 0.13f * g, 0.07f + 0.05f * treeRand(seed * 5 + 3)};
    const int segN = 6;
    float rot = treeRand(seed * 5 + 4) * 6.2831853f;
    int tiers = lod == 0 ? 3 : (lod == 1 ? 2 : 1);
    if (lod == 0) {
        // 下枝の下に見える幹
        float tr = 0.30f + height * 0.006f;
        addTrunk(mesh, cx, cz, baseY - 0.6f, baseY + height * 0.32f, tr, tr * 0.6f, rot,
                 Vec3{0.26f, 0.19f, 0.13f});
    }
    for (int tier = 0; tier < tiers; ++tier) {
        float t0 = tiers == 1 ? 0.12f : (0.14f + 0.26f * float(tier));
        float t1 = tiers == 1 ? 1.0f : std::min(1.0f, t0 + (tier == tiers - 1 ? (1.0f - t0) : 0.46f));
        float baseR = radius * (tiers == 1 ? 1.0f : (1.0f - 0.26f * float(tier)));
        float yBase = baseY + height * t0;
        float yTop = baseY + height * t1;
        std::uint32_t apexId = static_cast<std::uint32_t>(mesh.vertices.size() / 9);
        mesh.addVertex({cx, yTop, cz}, {0, 1, 0}, leaf * 1.18f);
        for (int k = 0; k < segN; ++k) {
            float a = rot + 6.2831853f * float(k) / float(segN) + (tier & 1) * 0.5f;
            Vec3 p{cx + std::cos(a) * baseR, yBase, cz + std::sin(a) * baseR};
            Vec3 n = normalize(Vec3{std::cos(a), 0.55f, std::sin(a)});
            mesh.addVertex(p, n, leaf * 0.82f);
        }
        for (int k = 0; k < segN; ++k) {
            std::uint32_t p0 = apexId + 1 + std::uint32_t(k);
            std::uint32_t p1 = apexId + 1 + std::uint32_t((k + 1) % segN);
            Vec3 A{mesh.vertices[apexId * 9], mesh.vertices[apexId * 9 + 1], mesh.vertices[apexId * 9 + 2]};
            Vec3 B{mesh.vertices[p0 * 9], mesh.vertices[p0 * 9 + 1], mesh.vertices[p0 * 9 + 2]};
            Vec3 C{mesh.vertices[p1 * 9], mesh.vertices[p1 * 9 + 1], mesh.vertices[p1 * 9 + 2]};
            Vec3 mid = (B + C) * 0.5f - Vec3{cx, 0, cz};
            Vec3 nrm = normalize(Vec3{mid.x, 0.55f, mid.z});
            if (dot(cross(B - A, C - A), nrm) < 0.0f) {
                mesh.addRawTriangle(apexId, p1, p0);
            } else {
                mesh.addRawTriangle(apexId, p0, p1);
            }
        }
    }
}

// 広葉樹（明るめの緑）。丸い樹冠を 4 つの輪を結んだ膨らみで表す。
void addBroadleaf(CpuMesh& mesh, float cx, float cz, float baseY, int seed, float height) {
    float radius = height * (0.33f + 0.07f * treeRand(seed * 5 + 1));
    float g = treeRand(seed * 5 + 2);
    Vec3 leaf{0.13f + 0.12f * g, 0.29f + 0.16f * g, 0.08f + 0.05f * treeRand(seed * 5 + 3)};
    const int segN = 7;
    float rot = treeRand(seed * 5 + 4) * 6.2831853f;
    float tr = 0.28f + height * 0.007f;
    addTrunk(mesh, cx, cz, baseY - 0.6f, baseY + height * 0.50f, tr, tr * 0.65f, rot,
             Vec3{0.28f, 0.21f, 0.15f});
    // 下の頂点 → 4 つの輪 → 上の頂点
    const float ringH[4] = {0.40f, 0.52f, 0.68f, 0.88f};
    const float ringR[4] = {0.55f, 0.97f, 1.00f, 0.60f};
    const float shade[4] = {0.70f, 0.90f, 1.05f, 1.18f};
    std::uint32_t botId = static_cast<std::uint32_t>(mesh.vertices.size() / 9);
    mesh.addVertex({cx, baseY + height * 0.34f, cz}, {0, -1, 0}, leaf * 0.6f);
    std::uint32_t ringId = botId + 1;
    for (int r = 0; r < 4; ++r) {
        for (int k = 0; k < segN; ++k) {
            float a = rot + 6.2831853f * float(k) / float(segN) + 0.4f * float(r);
            Vec3 p{cx + std::cos(a) * radius * ringR[r], baseY + height * ringH[r],
                   cz + std::sin(a) * radius * ringR[r]};
            float ny = r == 0 ? -0.4f : (r == 3 ? 0.7f : 0.2f);
            mesh.addVertex(p, normalize(Vec3{std::cos(a), ny, std::sin(a)}), leaf * shade[r]);
        }
    }
    std::uint32_t topId = ringId + 4 * segN;
    mesh.addVertex({cx, baseY + height, cz}, {0, 1, 0}, leaf * 1.25f);
    auto V = [&](std::uint32_t id) {
        return Vec3{mesh.vertices[id * 9], mesh.vertices[id * 9 + 1], mesh.vertices[id * 9 + 2]};
    };
    auto tri = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        Vec3 pa = V(a), pb = V(b), pc = V(c);
        Vec3 out = (pa + pb + pc) * (1.0f / 3.0f) - Vec3{cx, baseY + height * 0.62f, cz};
        if (dot(cross(pb - pa, pc - pa), out) < 0.0f) std::swap(b, c);
        mesh.addRawTriangle(a, b, c);
    };
    for (int k = 0; k < segN; ++k) {
        int k2 = (k + 1) % segN;
        tri(botId, ringId + std::uint32_t(k), ringId + std::uint32_t(k2));
        for (int r = 0; r < 3; ++r) {
            std::uint32_t a = ringId + std::uint32_t(r * segN + k), b = ringId + std::uint32_t(r * segN + k2);
            std::uint32_t c = ringId + std::uint32_t((r + 1) * segN + k2), d = ringId + std::uint32_t((r + 1) * segN + k);
            tri(a, b, c);
            tri(a, c, d);
        }
        tri(topId, ringId + std::uint32_t(3 * segN + k), ringId + std::uint32_t(3 * segN + k2));
    }
}

// Small low-poly shrubs fill the near verge without another full tree mesh.
void addShrub(CpuMesh& mesh, float x, float z, float y, float radius, int seed) {
    const float shade = treeRand(seed * 11 + 3);
    const Vec3 color{0.10f + 0.04f * shade, 0.28f + 0.13f * shade, 0.08f};
    const Vec3 top{x, y + radius * 0.85f, z};
    Vec3 p[4] = {{x-radius,y,z-radius},{x+radius,y,z-radius},
                 {x+radius,y,z+radius},{x-radius,y,z+radius}};
    for (int k=0; k<4; ++k) {
        const Vec3 a=p[k], b=p[(k+1)%4];
        const Vec3 outward = normalize(Vec3{(a.x+b.x)*0.5f-x, radius*0.7f, (a.z+b.z)*0.5f-z});
        mesh.addTriangle(a,b,top,outward,color*(0.85f+0.05f*k));
    }
}

// Grow a sparse, oversized canopy in two road-facing rows.  Trees behind a
// ridge or far from all road samples do not contribute and are not generated.
void addForest(CpuMesh& mesh, const Track& track, const Terrain& terrain, const SampleGrid& grid) {
    const float wall = track.wallLateral();
    const int n = static_cast<int>(track.samples.size());
    auto exposed = [&](int road, float x, float z, float ground, float height) {
        const auto& s = track.samples[road];
        // Check several points along the line of sight to the upper canopy.
        for (float f : {0.25f, 0.50f, 0.75f}) {
            const float px = s.x + (x-s.x)*f, pz = s.z + (z-s.z)*f;
            const float sightY = (s.y + 3.0f)*(1.0f-f) + (ground + height*0.75f)*f;
            if (terrain.sample(px,pz) > sightY + 1.0f) return false;
        }
        return true;
    };
    auto place = [&](int i, int side, int row) {
        const int seed = i * 7 + (side > 0 ? 1 : 4) + row * 100003;
        const float height = 25.0f + 11.0f * treeRand(seed * 3 + 1);
        const bool broad = treeRand(seed * 3 + 2) < 0.24f;
        const float treeHeight = height * (broad ? 0.90f : 1.0f);
        // Maximum canopy radius plus a margin keeps all branches off the playable road.
        const float crown = treeHeight * (broad ? 0.40f : 0.23f);
        const float offset = wall + crown + (row == 0 ? 2.0f : 15.0f) + 5.0f * treeRand(seed * 3 + 3);
        Vec3 right = horizontalRight(track,i);
        const float x = track.samples[i].x + right.x * float(side) * offset;
        const float z = track.samples[i].z + right.z * float(side) * offset;
        float d2;
        const int near = grid.nearestWithin(track,x,z,65.0f,d2);
        if (near < 0 || d2 < (wall+crown+1.0f)*(wall+crown+1.0f)) return;
        const float ground = terrain.sample(x,z);
        const float roadY = roadSurfaceAt(track,near,x,z);
        if (ground > roadY + 28.0f || !exposed(near,x,z,ground,treeHeight)) return;
        if (broad) addBroadleaf(mesh,x,z,ground-0.3f,seed,treeHeight);
        else addConifer(mesh,x,z,ground-0.3f,seed,treeHeight,row == 0 ? 0 : 1);
    };
    float lastNear = -1.0e6f, lastBack = -1.0e6f, lastShrub = -1.0e6f;
    for (int i=0; i<n; ++i) {
        const float s = track.samples[i].s;
        if (s - lastNear >= 15.0f) {
            lastNear = s;
            for (int side : {-1,1}) place(i,side,0);
        }
        if (s - lastBack >= 36.0f) {
            lastBack = s;
            for (int side : {-1,1}) place(i,side,1);
        }
        if (s - lastShrub >= 24.0f) {
            lastShrub = s;
            for (int side : {-1,1}) {
                const int seed = i*17 + (side > 0 ? 7 : 11);
                if (treeRand(seed*3+1) > 0.70f) continue;
                const float radius = 2.4f + 1.6f * treeRand(seed*3+2);
                const float crown = radius * 1.42f;
                const float offset = wall + crown + 2.5f;
                Vec3 right = horizontalRight(track,i);
                const float x = track.samples[i].x + right.x * float(side) * offset;
                const float z = track.samples[i].z + right.z * float(side) * offset;
                float d2; int near=grid.nearestWithin(track,x,z,35.0f,d2);
                if (near < 0 || d2 < (wall+crown+1.5f)*(wall+crown+1.5f)) continue;
                const float ground = terrain.sample(x,z);
                if (ground > roadSurfaceAt(track,near,x,z)+10.0f) continue;
                if (!exposed(near,x,z,ground,radius*0.85f)) continue;
                addShrub(mesh,x,z,ground-0.1f,radius,seed);
            }
        }
    }
}

}  // namespace

bool Track::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return false;
    }

    enum class Mode { Top, Names, Samples, Pit };
    Mode mode = Mode::Top;
    int expectNames = 0;
    int expectSamples = 0;
    int expectPit = 0;
    std::vector<Vec3> pit;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        if (mode == Mode::Names) {
            int id = 0;
            ss >> id;
            std::string name;
            std::getline(ss, name);
            if (!name.empty() && name[0] == ' ') {
                name.erase(0, 1);
            }
            names.push_back(name);
            if (static_cast<int>(names.size()) >= expectNames) {
                mode = Mode::Top;
            }
            continue;
        }
        if (mode == Mode::Samples) {
            TrackSample sample;
            int bridge = 0;
            ss >> sample.x >> sample.z >> sample.y >> sample.cross >> bridge >> sample.corner >> sample.nameId;
            if (ss.fail()) {
                return false;
            }
            sample.bridge = bridge != 0;
            samples.push_back(sample);
            if (static_cast<int>(samples.size()) >= expectSamples) {
                mode = Mode::Top;
            }
            continue;
        }
        if (mode == Mode::Pit) {
            Vec3 point;
            ss >> point.x >> point.z >> point.y;
            if (ss.fail()) {
                return false;
            }
            pit.push_back(point);
            if (static_cast<int>(pit.size()) >= expectPit) {
                mode = Mode::Top;
            }
            continue;
        }

        std::string key;
        ss >> key;
        if (key == "origin") {
            ss >> originLat >> originLon;
        } else if (key == "closed") {
            int v = 1;
            ss >> v;
            closed = v != 0;
        } else if (key == "title") {
            std::string rest;
            std::getline(ss, rest);
            if (!rest.empty() && rest[0] == ' ') {
                rest.erase(0, 1);
            }
            title = rest;
        } else if (key == "style") {
            std::string v;
            ss >> v;
            touge = v == "touge";
        } else if (key == "runoff") {
            ss >> runoff;
        } else if (key == "half_width") {
            ss >> halfWidth;
        } else if (key == "curb_width") {
            ss >> curbWidth;
        } else if (key == "length") {
            ss >> length;
        } else if (key == "spawn") {
            ss >> spawnX >> spawnY >> spawnZ >> spawnYaw;
        } else if (key == "names") {
            ss >> expectNames;
            mode = Mode::Names;
        } else if (key == "samples") {
            ss >> expectSamples;
            mode = Mode::Samples;
        } else if (key == "pit") {
            ss >> expectPit;
            mode = expectPit > 0 ? Mode::Pit : Mode::Top;
        }
    }

    if (samples.size() < 16) {
        return false;
    }

    downhillSpawnX=spawnX; downhillSpawnY=spawnY; downhillSpawnZ=spawnZ; downhillSpawnYaw=spawnYaw;
    startGate=0; finishGate=int(samples.size())-1; uphill=false;
    if (!closed && touge) {
        // Extend both ends so either direction has 100 m after its finish line.
        constexpr int count=25;
        constexpr float step=4.0f;
        auto extension = [&](const TrackSample& endpoint, const TrackSample& neighbour, float distance) {
            TrackSample v=endpoint;
            float dx=endpoint.x-neighbour.x, dz=endpoint.z-neighbour.z;
            float len=std::sqrt(dx*dx+dz*dz);
            if (len > 0.001f) {
                v.x += dx/len*distance; v.z += dz/len*distance;
                v.y += clampf((endpoint.y-neighbour.y)/len, -0.12f, 0.12f)*distance;
            }
            v.nameId=-1; v.corner=0; v.bridge=false;
            return v;
        };
        const TrackSample first=samples.front(), second=samples[1];
        const TrackSample last=samples.back(), before=samples[samples.size()-2];
        std::vector<TrackSample> extended; extended.reserve(samples.size()+2*count);
        for(int i=count;i>0;--i) extended.push_back(extension(first,second,float(i)*step));
        extended.insert(extended.end(),samples.begin(),samples.end());
        for(int i=1;i<=count;++i) extended.push_back(extension(last,before,float(i)*step));
        samples.swap(extended);
        startGate=count; finishGate=int(samples.size())-count-1;
    }
    length = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        samples[i].s = length;
        if (!closed && i + 1 >= samples.size()) {
            break;  // オープン: ゴールからスタートへは繋がない
        }
        const TrackSample& a = samples[i];
        const TrackSample& b = samples[(i + 1) % samples.size()];
        length += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));
    }

    raceStartS=samples[startGate].s;
    raceFinishS=closed ? length : samples[finishGate].s;
    visual = CpuMesh{};
    addGround(visual, *this, touge ? Vec3{0.08f, 0.20f, 0.08f} : Vec3{0.12f, 0.27f, 0.11f});
    SampleGrid sampleGrid;
    Terrain terrain;
    const bool mountainForest = !closed && touge;
    if (mountainForest) {
        // 山岳地形（高さ場）。近くは路面の下、離れるほど立ち上がり、遠くは稜線で視界を塞ぐ。
        sampleGrid.build(*this);
        terrain = buildTerrain(*this, sampleGrid);
        addTerrainMesh(visual, terrain);
    } else if (!closed) {
        addMountain(visual, *this);
    }
    const float curbOuter = halfWidth + curbWidth;
    const float grassOuter = curbOuter + runoff;
    if (touge) {
        // 峠仕様: 赤白の縁石とコンクリート壁は使わない。路肩は土、
        // その外は林床の色にして、ガードレールと杉林で囲う。
        addRibbon(visual, *this, -grassOuter, -curbOuter, {0.13f, 0.30f, 0.12f}, true);
        addRibbon(visual, *this, curbOuter, grassOuter, {0.13f, 0.30f, 0.12f}, true);
        addRibbon(visual, *this, -curbOuter, -halfWidth, {0.40f, 0.36f, 0.28f}, true);
        addRibbon(visual, *this, halfWidth, curbOuter, {0.40f, 0.36f, 0.28f}, true);
        addRibbon(visual, *this, -halfWidth, halfWidth, {0.15f, 0.15f, 0.16f}, false);
        addRoadMarkings(visual, *this);
        addGuardrails(visual, *this);
        if (mountainForest) {
            addForest(visual, *this, terrain, sampleGrid);
        } else {
            addTrees(visual, *this);
            addBackdropRidges(visual, *this);
        }
    } else {
        addRibbon(visual, *this, -grassOuter, -curbOuter, {0.15f, 0.36f, 0.14f}, true);
        addRibbon(visual, *this, curbOuter, grassOuter, {0.15f, 0.36f, 0.14f}, true);
        addRibbon(visual, *this, -halfWidth, halfWidth, {0.17f, 0.17f, 0.19f}, false);
        addCurbs(visual, *this);
        addWalls(visual, *this);
    }
    addPit(visual, pit);
    addStartLine(visual, *this, startGate, startGate + 1);
    if (!closed) {
        addStartLine(visual, *this, finishGate, finishGate - 1);  // ゴールラインと門
    }
    return true;
}

bool Track::setUphill(bool enabled) {
    if (closed || !touge) return !enabled;
    if (enabled == uphill) return true;
    std::reverse(samples.begin(), samples.end());
    for (TrackSample& sample : samples) sample.cross = -sample.cross;
    length=0;
    for (size_t i=0; i<samples.size(); ++i) {
        samples[i].s=length;
        if (i+1<samples.size()) {
            float dx=samples[i+1].x-samples[i].x, dz=samples[i+1].z-samples[i].z;
            length+=std::sqrt(dx*dx+dz*dz);
        }
    }
    raceStartS=samples[startGate].s; raceFinishS=samples[finishGate].s;
    uphill=enabled;
    if (uphill) {
        const int i=std::min(startGate+2,finishGate-1);
        const TrackSample& a=samples[i]; const TrackSample& b=samples[i+1];
        spawnX=a.x; spawnY=a.y; spawnZ=a.z; spawnYaw=std::atan2(b.x-a.x,b.z-a.z);
    } else {
        spawnX=downhillSpawnX; spawnY=downhillSpawnY; spawnZ=downhillSpawnZ; spawnYaw=downhillSpawnYaw;
    }
    return true;
}

TrackPose Track::query(float x, float z, float yHint, float preferS) const {
    TrackPose best;
    float bestScore = 1.0e30f;
    const int segs = segmentCount(*this);
    for (int i = 0; i < segs; ++i) {
        const TrackSample& a = samples[i];
        const TrackSample& b = samples[nextIndex(*this, i)];
        float abx = b.x - a.x;
        float abz = b.z - a.z;
        float len2 = abx * abx + abz * abz;
        if (len2 < 1e-4f) {
            continue;
        }
        float t = ((x - a.x) * abx + (z - a.z) * abz) / len2;
        t = clampf(t, 0.0f, 1.0f);
        float cx = a.x + abx * t;
        float cz = a.z + abz * t;
        float cy = a.y + (b.y - a.y) * t;
        float dx = x - cx;
        float dz = z - cz;
        float dy = yHint - cy;
        // 立体交差では平面距離より高さを優先する。6m 差なら、壁際にいても下の路面へ落ちない。
        float score = dx * dx + dz * dz + dy * dy * 12.0f;
        if (preferS >= 0.0f && length > 1.0f) {
            float along = a.s + std::sqrt(len2) * t;
            float ds = std::fabs(along - preferS);
            if (closed && ds > length * 0.5f) {
                ds = length - ds;
            }
            // 今走っている周回位置から遠い路面は、交差の真下でも選ばない。
            score += ds * 0.2f;
        }
        if (score >= bestScore) {
            continue;
        }
        bestScore = score;
        float invLen = 1.0f / std::sqrt(len2);
        float tx = abx * invLen;
        float tz = abz * invLen;
        best.found = true;
        best.s = a.s + std::sqrt(len2) * t;
        if (closed && best.s >= length) {
            best.s -= length;
        }
        best.rightX = tz;
        best.rightZ = -tx;
        best.lateral = dx * best.rightX + dz * best.rightZ;
        best.centerY = cy;
        best.cross = a.cross + (b.cross - a.cross) * t;
        best.yaw = std::atan2(tx, tz);
        best.xzDist = std::sqrt(dx * dx + dz * dz);
        best.corner = t < 0.5f ? a.corner : b.corner;
        best.nameId = t < 0.5f ? a.nameId : b.nameId;
    }
    return best;
}

const char* Track::nameOf(int nameId) const {
    if (nameId < 0 || nameId >= static_cast<int>(names.size())) {
        return "";
    }
    return names[nameId].c_str();
}
