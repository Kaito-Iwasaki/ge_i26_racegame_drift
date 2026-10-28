// 秋名（オープンコース）の検証ハーネス。SDL/OpenGL は使わず、
// car.cpp / track.cpp だけを直接叩く（drift_test.cpp と同じ流儀）。
//
//   g++ -std=c++17 -I<game dir> akina_test.cpp <game dir>/car.cpp <game dir>/track.cpp -o akina_test
//   ./akina_test akina.txt
//
// チェック項目:
//   1. ロードできる／closed 0／全長が 7,552 m 級／サンプル間隔が均一
//   2. 標高が単調に下る（下りコース）／総落差 500 m 以上
//   3. nameId / コーナー番号が正規の範囲、横断勾配が上限内
//   4. 本編 self-test と同じオートパイロットで 50 秒走って進む・有限
//   5. 曲率ベース速度制御のオートパイロットでゴールまで完走できる
#include "car.hpp"
#include "track.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* name, float value) {
    std::printf("[%s] %-52s value=%.3f\n", ok ? "PASS" : "FAIL", name, value);
    if (!ok) ++failures;
}

// 標本の進行方向（yaw）を s から引くための簡易テーブル
struct CourseInfo {
    std::vector<float> kappa;  // サンプルごとの |曲率| (1/m), 平滑化済み
};

CourseInfo analyze(const Track& track) {
    const int n = static_cast<int>(track.samples.size());
    std::vector<float> yaw(n), raw(n, 0.0f);
    for (int i = 0; i < n; ++i) {
        int p = i > 0 ? i - 1 : 0;
        int q = i + 1 < n ? i + 1 : n - 1;
        yaw[i] = std::atan2(track.samples[q].x - track.samples[p].x,
                            track.samples[q].z - track.samples[p].z);
    }
    for (int i = 1; i + 1 < n; ++i) {
        float d = yaw[i + 1] - yaw[i - 1];
        while (d > 3.14159265f) d -= 2.0f * 3.14159265f;
        while (d < -3.14159265f) d += 2.0f * 3.14159265f;
        float ds = track.samples[i + 1].s - track.samples[i - 1].s;
        raw[i] = ds > 0.1f ? std::fabs(d) / ds : 0.0f;
    }
    CourseInfo info;
    info.kappa.assign(n, 0.0f);
    for (int i = 0; i < n; ++i) {
        float sum = 0;
        int cnt = 0;
        for (int k = -3; k <= 3; ++k) {
            int j = i + k;
            if (0 <= j && j < n) {
                sum += raw[j];
                ++cnt;
            }
        }
        info.kappa[i] = sum / float(cnt);
    }
    return info;
}

int nearestIndex(const Track& track, float s) {
    int best = 0;
    float gap = 1e9f;
    for (int i = 0; i < static_cast<int>(track.samples.size()); ++i) {
        float g = std::fabs(track.samples[i].s - s);
        if (g < gap) {
            gap = g;
            best = i;
        }
    }
    return best;
}

}  // namespace

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "akina.txt";
    Track track;
    if (!track.load(path)) {
        std::printf("FAIL: cannot load %s\n", path);
        return 1;
    }
    const int n = static_cast<int>(track.samples.size());
    std::printf("loaded %s: samples=%d length=%.1f closed=%d title=%s\n",
                path, n, track.length, track.closed ? 1 : 0, track.title.c_str());

    check(!track.closed, "open course (closed == 0)", track.closed ? 1 : 0);
    check(track.length > 7000.0f && track.length < 8100.0f, "length ~7552 m", track.length);
    check(n >= 1800, "sample count >= 1800", float(n));

    float minGap = 1e9f, maxGap = 0;
    for (int i = 0; i + 1 < n; ++i) {
        float d = std::hypot(track.samples[i + 1].x - track.samples[i].x,
                             track.samples[i + 1].z - track.samples[i].z);
        minGap = std::min(minGap, d);
        maxGap = std::max(maxGap, d);
    }
    check(minGap > 3.0f && maxGap < 5.0f, "sample spacing 3..5 m (min/max)", maxGap);

    // 標高: 全体として単調に下る（局所の逆行は 0.5 m まで許容）
    float worstRise = 0;
    for (int i = 0; i + 1 < n; ++i) {
        worstRise = std::max(worstRise, track.samples[i + 1].y - track.samples[i].y);
    }
    float drop = track.samples.front().y - track.samples.back().y;
    check(worstRise <= 0.5f, "elevation never rises > 0.5 m per sample", worstRise);
    check(drop > 500.0f, "total elevation drop > 500 m", drop);

    int maxCorner = 0;
    bool idsOk = true;
    float maxCross = 0;
    for (const TrackSample& s : track.samples) {
        maxCorner = std::max(maxCorner, s.corner);
        if (s.nameId < -1 || s.nameId >= static_cast<int>(track.names.size())) idsOk = false;
        maxCross = std::max(maxCross, std::fabs(s.cross));
    }
    check(idsOk, "all nameId within names table", float(track.names.size()));
    check(maxCorner >= 30, "corner count >= 30", float(maxCorner));
    check(maxCross <= 0.0701f, "|cross slope| <= 0.07", maxCross);

    TrackPose spawnPose = track.query(track.spawnX, track.spawnZ, track.spawnY);
    check(spawnPose.found && spawnPose.xzDist < 2.0f, "spawn is on the road", spawnPose.xzDist);
    TrackPose goalPose = track.query(track.samples.back().x, track.samples.back().z,
                                     track.samples.back().y);
    check(goalPose.found && goalPose.s > track.length - 5.0f, "goal sample s ~ length", goalPose.s);

    CourseInfo info = analyze(track);
    float maxKappa = 0;
    for (float k : info.kappa) maxKappa = std::max(maxKappa, k);
    std::printf("max curvature %.4f 1/m (min radius %.1f m)\n", maxKappa, 1.0f / maxKappa);
    check(maxKappa < 0.075f, "tightest radius >= ~13 m", 1.0f / maxKappa);

    // --- 4. 本編 self-test と同じオートパイロット (50 s) -------------------
    {
        Car car;
        car.reset(track);
        float travelled = 0, prevS = -1, maxLat = 0;
        bool finite = true;
        for (int step = 0; step < 60 * 50; ++step) {
            TrackPose pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
            float lookS = pose.s + 70.0f;
            if (lookS > track.length) lookS = track.length;
            int nearest = nearestIndex(track, lookS);
            int next = std::min(nearest + 1, n - 1);
            float targetYaw = std::atan2(track.samples[next].x - track.samples[nearest].x,
                                         track.samples[next].z - track.samples[nearest].z);
            float errNow = wrapPi(pose.yaw - car.yaw);
            float errAhead = wrapPi(targetYaw - car.yaw);
            float bend = std::fabs(wrapPi(targetYaw - pose.yaw));
            DriveInput input;
            input.steer = clampf(errNow * 2.6f + errAhead * 0.5f, -1.0f, 1.0f);
            input.steer = clampf(input.steer - pose.lateral * 0.12f, -1.0f, 1.0f);
            input.throttle = bend < 0.15f ? 1.0f : 0.45f;
            if (bend > 0.28f && car.speed > 22.0f) {
                input.throttle = 0;
                input.brake = 1.0f;
            } else if (bend > 0.12f && car.speed > 34.0f) {
                input.throttle = 0;
                input.brake = 0.55f;
            }
            if (car.speed < 10.0f) {
                input.brake = 0;
                input.throttle = std::max(input.throttle, 0.5f);
            }
            car.update(1.0f / 60.0f, input, track);
            pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
            if (prevS >= 0 && pose.s - prevS > 0) travelled += pose.s - prevS;
            prevS = pose.s;
            maxLat = std::max(maxLat, pose.xzDist);
            if (!std::isfinite(car.x) || !std::isfinite(car.speed)) finite = false;
        }
        std::printf("self-test autopilot: travelled %.0f m, maxLat %.1f m\n", travelled, maxLat);
        check(finite, "self-test autopilot: finite state", finite ? 1 : 0);
        check(travelled >= 1200.0f, "self-test autopilot: travelled >= 1200 m", travelled);
        check(maxLat <= 30.0f, "self-test autopilot: stays near course", maxLat);
    }

    // --- 5. 完走テスト: 曲率から速度を決めるオートパイロット ---------------
    {
        Car car;
        car.reset(track);
        float simTime = 0;
        float prevS = -1;
        bool finite = true;
        int nextReport = 1000;
        while (simTime < 900.0f) {
            TrackPose pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
            if (pose.s >= track.length - 4.0f) break;
            // この先 90 m の最小許容速度（横加速度 22 m/s^2 相当で見積もる）
            int here = nearestIndex(track, pose.s);
            float vAllow = 55.0f;
            for (int j = here; j < n && track.samples[j].s < pose.s + 90.0f; ++j) {
                if (info.kappa[j] > 0.004f) {
                    vAllow = std::min(vAllow, std::sqrt(22.0f / info.kappa[j]));
                }
            }
            vAllow = clampf(vAllow, 9.0f, 55.0f);
            float look = clampf(car.groundSpeed() * 0.9f, 12.0f, 45.0f);
            int target = nearestIndex(track, std::min(pose.s + look, track.length));
            int tnext = std::min(target + 1, n - 1);
            float targetYaw = std::atan2(track.samples[tnext].x - track.samples[target].x,
                                         track.samples[tnext].z - track.samples[target].z);
            DriveInput input;
            float errNow = wrapPi(pose.yaw - car.yaw);
            float errAhead = wrapPi(targetYaw - car.yaw);
            input.steer = clampf(errNow * 2.2f + errAhead * 0.9f - pose.lateral * 0.10f, -1.0f, 1.0f);
            float v = car.groundSpeed();
            if (v < vAllow - 1.5f) {
                input.throttle = 1.0f;
            } else if (v > vAllow + 1.0f) {
                input.brake = clampf((v - vAllow) / 8.0f, 0.2f, 1.0f);
            } else {
                input.throttle = 0.35f;
            }
            if (v < 6.0f) {
                input.brake = 0;
                input.throttle = 1.0f;
            }
            car.update(1.0f / 60.0f, input, track);
            simTime += 1.0f / 60.0f;
            if (!std::isfinite(car.x) || !std::isfinite(car.speed)) {
                finite = false;
                break;
            }
            if (pose.s > float(nextReport)) {
                std::printf("  ... s=%.0f m at %.0f s (%.0f km/h)\n", pose.s, simTime, v * 3.6f);
                nextReport += 1000;
            }
            prevS = pose.s;
        }
        TrackPose end = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
        std::printf("full run: reached s=%.0f / %.0f m in %.1f s\n", end.s, track.length, simTime);
        check(finite, "full run: finite state", finite ? 1 : 0);
        check(end.s >= track.length - 4.0f, "full run: reached the goal", end.s);
        check(simTime < 900.0f, "full run: within 15 min", simTime);
        (void)prevS;
    }

    std::printf(failures == 0 ? "\nALL PASS\n" : "\n%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
