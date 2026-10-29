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
    for (int i = 0; i < segs; i += 2) {
        for (int side = -1; side <= 1; side += 2) {
            float sign = float(side);
            ++seed;
            if (treeRand(seed * 31 + 7) >= 0.30f) {
                float lat = wall + 3.5f + treeRand(seed * 31 + 8) * 30.0f;
                tryPlant(i, sign, lat, 0.9f + treeRand(seed * 31 + 9) * 0.3f, seed);
            }
            ++seed;
            if (treeRand(seed * 31 + 7) >= 0.65f) {
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

    visual = CpuMesh{};
    addGround(visual, *this, touge ? Vec3{0.08f, 0.20f, 0.08f} : Vec3{0.12f, 0.27f, 0.11f});
    if (!closed) {
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
        addTrees(visual, *this);
        addBackdropRidges(visual, *this);
    } else {
        addRibbon(visual, *this, -grassOuter, -curbOuter, {0.15f, 0.36f, 0.14f}, true);
        addRibbon(visual, *this, curbOuter, grassOuter, {0.15f, 0.36f, 0.14f}, true);
        addRibbon(visual, *this, -halfWidth, halfWidth, {0.17f, 0.17f, 0.19f}, false);
        addCurbs(visual, *this);
        addWalls(visual, *this);
    }
    addPit(visual, pit);
    addStartLine(visual, *this, 0, 1);
    if (!closed) {
        const int n = static_cast<int>(samples.size());
        addStartLine(visual, *this, n - 1, n - 2);  // ゴールラインと門
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
