// ドリフト物理の検証ハーネス。SDL/OpenGL は使わず、円形コースを
// プログラムで組み立てて car.cpp / track.cpp だけを直接叩く。
//
//   g++ -std=c++17 -I.. drift_test.cpp ../car.cpp ../track.cpp -o drift_test
//   ./drift_test
//
// チェック項目:
//   1. グリップ走行: 直進と通常コーナリングで滑らない・勝手にドリフトしない
//   2. ドリフト突入: サイドブレーキ + ステアで大きく滑り、判定とスコアが付く
//   3. 収束: 手を離せば滑りが収まってグリップ走行に戻る（発散・NaN なし）
//   4. 自走: 簡易オートパイロットで円形コースを周回できる（本編 self-test 相当）
//   5. 後退と据え切りが従来通り使える
#include "car.hpp"

#include <cmath>
#include <cstdio>

namespace {

constexpr float kDt = 1.0f / 60.0f;
int failures = 0;

void check(bool ok, const char* name, float value) {
    std::printf("[%s] %-46s value=%.3f\n", ok ? "PASS" : "FAIL", name, value);
    if (!ok) ++failures;
}

Track makeCircle(float radius, float runoff) {
    Track track;
    track.halfWidth = 6.5f;
    track.curbWidth = 0.8f;
    track.runoff = runoff;
    const int n = 720;
    const float pi = 3.14159265f;
    track.samples.resize(n);
    for (int i = 0; i < n; ++i) {
        float phi = 2.0f * pi * float(i) / float(n);
        track.samples[i].x = radius * std::sin(phi);
        track.samples[i].z = radius * std::cos(phi);
        track.samples[i].y = 0;
        track.samples[i].cross = 0;
        track.samples[i].nameId = -1;
    }
    track.length = 0;
    for (int i = 0; i < n; ++i) {
        const TrackSample& a = track.samples[i];
        const TrackSample& b = track.samples[(i + 1) % n];
        track.samples[i].s = track.length;
        track.length += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));
    }
    track.spawnX = track.samples[0].x;
    track.spawnY = 0;
    track.spawnZ = track.samples[0].z;
    track.spawnYaw = std::atan2(track.samples[1].x - track.samples[0].x,
                                track.samples[1].z - track.samples[0].z);
    return track;
}

// まっすぐなコース（高速域の測定用）。+Z 方向へ伸びる。
Track makeStraight(float length) {
    Track track;
    track.halfWidth = 6.5f;
    track.curbWidth = 0.8f;
    track.runoff = 60.0f;
    const float step = 10.0f;
    const int n = static_cast<int>(length / step) + 1;
    track.samples.resize(n);
    for (int i = 0; i < n; ++i) {
        track.samples[i].x = 0;
        track.samples[i].z = i * step;
        track.samples[i].y = 0;
        track.samples[i].cross = 0;
        track.samples[i].nameId = -1;
        track.samples[i].s = i * step;
    }
    track.length = length;
    track.spawnX = 0;
    track.spawnY = 0;
    track.spawnZ = 0;
    track.spawnYaw = 0;
    return track;
}

bool finite(const Car& car) {
    return std::isfinite(car.x) && std::isfinite(car.z) && std::isfinite(car.yaw) &&
           std::isfinite(car.speed) && std::isfinite(car.velX) && std::isfinite(car.velZ) &&
           std::isfinite(car.slipAngle);
}

void run(Car& car, const Track& track, const DriveInput& input, float seconds) {
    int steps = static_cast<int>(seconds / kDt);
    for (int i = 0; i < steps; ++i) car.update(kDt, input, track);
}

// 目標速度まで加速する。円コースでは中心線を追いながら加速する
// （2026-10-29: 加速を緩めた結果、無操舵だと助走距離が伸びて壁に当たって
// から計測が始まるようになったため。直線コースでは従来と実質同じ）。
void accelerateTo(Car& car, const Track& track, float targetSpeed) {
    DriveInput input;
    input.throttle = 1;
    for (int i = 0; i < 60 * 20 && car.speed < targetSpeed; ++i) {
        float lookS = car.ground.s + 20.0f;
        if (lookS > track.length) {
            lookS = track.length;  // 円コースでは最終サンプル≒始点なのでこれで繋がる
        }
        int idx = 0;
        float best = 1.0e30f;
        for (int j = 0; j < static_cast<int>(track.samples.size()); ++j) {
            float d = std::fabs(track.samples[j].s - lookS);
            if (d < best) {
                best = d;
                idx = j;
            }
        }
        float want = std::atan2(track.samples[idx].x - car.x, track.samples[idx].z - car.z);
        input.steer = clampf(wrapPi(want - car.yaw) * 1.4f, -1.0f, 1.0f);
        car.update(kDt, input, track);
    }
}

}  // namespace

int main() {
    // ---- 1. グリップ走行 ----------------------------------------------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        Car car;
        car.reset(track);
        accelerateTo(car, track, 30.0f);
        check(car.speed > 25.0f, "grip: accelerates to speed", car.speed);
        check(std::fabs(car.slipAngle) < 0.05f, "grip: no slip going straight", car.slipAngle);

        DriveInput turn;
        turn.steer = 0.6f;
        turn.throttle = 0.5f;
        float yawBefore = car.yaw;
        float maxSlip = 0;
        bool everDrift = false;
        for (int i = 0; i < 60 * 2; ++i) {
            car.update(kDt, turn, track);
            maxSlip = std::max(maxSlip, std::fabs(car.slipAngle));
            everDrift = everDrift || car.drifting;
        }
        check(maxSlip < 0.20f, "grip: cornering slip stays small (rad)", maxSlip);
        check(!everDrift, "grip: cornering does not trigger drift", everDrift ? 1.0f : 0.0f);
        check(std::fabs(wrapPi(car.yaw - yawBefore)) > 0.5f, "grip: car actually turns (rad)",
              std::fabs(wrapPi(car.yaw - yawBefore)));
        check(finite(car), "grip: state stays finite", car.speed);
    }

    // ---- 2. ドリフト突入と維持、3. 収束（同じ車両で続けて行う） -------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        Car car;
        car.reset(track);
        accelerateTo(car, track, 27.0f);

        DriveInput pull;
        pull.steer = 1.0f;
        pull.handbrake = true;
        float maxSlip = 0;
        float minGrip = 99.0f;
        bool wasDrifting = false;
        for (int i = 0; i < static_cast<int>(0.55f / kDt); ++i) {
            car.update(kDt, pull, track);
            maxSlip = std::max(maxSlip, std::fabs(car.slipAngle));
            minGrip = std::min(minGrip, car.gripLat);
            wasDrifting = wasDrifting || car.drifting;
        }
        check(minGrip < 3.0f, "drift: handbrake drops lateral grip", minGrip);
        check(maxSlip > 0.35f, "drift: slip angle grows past 20deg (rad)", maxSlip);
        check(wasDrifting, "drift: drift state turns on", wasDrifting ? 1.0f : 0.0f);

        // カウンターステア + アクセルで維持する。
        DriveInput hold;
        hold.steer = -0.65f;
        hold.throttle = 0.85f;
        float driftSeconds = 0;
        for (int i = 0; i < static_cast<int>(1.4f / kDt); ++i) {
            car.update(kDt, hold, track);
            if (car.drifting) driftSeconds += kDt;
            maxSlip = std::max(maxSlip, std::fabs(car.slipAngle));
        }
        check(driftSeconds > 0.5f, "drift: counter-steer + throttle sustains it (s)", driftSeconds);
        check(car.driftScore > 0.0f, "drift: score accumulates", car.driftScore);
        check(car.groundSpeed() > 9.0f, "drift: car keeps moving (m/s)", car.groundSpeed());
        check(maxSlip < 1.20f, "drift: does not spin out of control (rad)", maxSlip);

        // 3a. 粘り: 少し手を離したくらいではドリフトは終わらない。
        DriveInput coast;
        run(car, track, coast, 0.4f);
        check(car.drifting, "sticky: brief lift-off keeps the drift state",
              car.drifting ? 1.0f : 0.0f);

        // 3b. 収束: 全部離して惰性で走り続ければ、時間をかけて滑りが収まって戻る。
        run(car, track, coast, 5.5f);
        check(std::fabs(car.slipAngle) < 0.08f, "recover: slip decays (rad)", car.slipAngle);
        check(!car.drifting, "recover: drift state turns off", car.drifting ? 1.0f : 0.0f);
        check(std::fabs(car.yawRate) < 0.25f, "recover: yaw settles (rad/s)", car.yawRate);
        check(finite(car), "recover: state stays finite", car.speed);
    }

    // ---- 4. 自走で周回できる -------------------------------------------------
    {
        Track track = makeCircle(150.0f, 8.0f);
        Car car;
        car.reset(track);
        float travelled = 0;
        float prevS = -1;
        float maxLat = 0;
        for (int step = 0; step < 60 * 50; ++step) {
            TrackPose pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
            float lookS = pose.s + 55.0f;
            if (lookS >= track.length) lookS -= track.length;
            int nearest = 0;
            float nearestGap = 1.0e9f;
            for (int i = 0; i < static_cast<int>(track.samples.size()); i += 2) {
                float gap = std::fabs(track.samples[i].s - lookS);
                if (gap > track.length * 0.5f) gap = track.length - gap;
                if (gap < nearestGap) {
                    nearestGap = gap;
                    nearest = i;
                }
            }
            int next = (nearest + 1) % static_cast<int>(track.samples.size());
            float targetYaw = std::atan2(track.samples[next].x - track.samples[nearest].x,
                                         track.samples[next].z - track.samples[nearest].z);
            DriveInput input;
            input.steer = clampf(wrapPi(targetYaw - car.yaw) * 2.4f - pose.lateral * 0.10f, -1.0f, 1.0f);
            input.throttle = car.speed < 26.0f ? 0.9f : 0.25f;
            if (car.speed > 34.0f) {
                input.throttle = 0;
                input.brake = 0.6f;
            }
            car.update(kDt, input, track);
            pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
            if (prevS >= 0) {
                float ds = pose.s - prevS;
                if (ds < -track.length * 0.5f) ds += track.length;
                if (ds > track.length * 0.5f) ds -= track.length;
                if (ds > 0) travelled += ds;
            }
            prevS = pose.s;
            maxLat = std::max(maxLat, std::fabs(pose.lateral));
        }
        check(travelled > 1000.0f, "autopilot: travels far in 50s (m)", travelled);
        check(maxLat < 14.0f, "autopilot: stays near the line (m)", maxLat);
        check(finite(car), "autopilot: state stays finite", car.speed);
    }

    // ---- 5. 後退 ----------------------------------------------------------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        Car car;
        car.reset(track);
        DriveInput back;
        back.brake = 1;
        run(car, track, back, 1.6f);
        check(car.speed < -2.0f, "reverse: brake-hold reverses (m/s)", car.speed);
        check(finite(car), "reverse: state stays finite", car.speed);
    }

    // ---- 6. 高速では穏やかに、低速では小回りが利く --------------------------------
    {
        // 高速の測定は直線コースで（円コースでは加速中にコースアウトして
        // 低速ピボットの値を拾ってしまうため）。
        Track track = makeStraight(4000.0f);
        DriveInput turn;
        turn.steer = 1.0f;
        turn.throttle = 0.55f;

        Car fast;
        fast.reset(track);
        accelerateTo(fast, track, 55.0f);
        DriveInput firm;
        firm.steer = 0.5f;
        firm.throttle = 0.55f;
        float maxYawFast = 0;
        for (int i = 0; i < 90; ++i) {
            fast.update(kDt, firm, track);
            if (i >= 45) maxYawFast = std::max(maxYawFast, std::fabs(fast.yawRate));
        }
        // 55 m/s (約200 km/h) での通常の切り角では、回頭は穏やかでグリップのまま。
        check(maxYawFast < 0.55f, "speed: yaw stays manageable at 55 m/s (rad/s)", maxYawFast);
        check(!fast.drifting, "speed: firm steer at 55 stays in grip", fast.drifting ? 1.0f : 0.0f);
        check(finite(fast), "speed: state stays finite at speed", fast.speed);

        // 一方、フルロックを保持すれば高速でもドリフトへ入れる（ただし暴れない）。
        Car flick;
        flick.reset(track);
        accelerateTo(flick, track, 55.0f);
        float maxYawFlick = 0;
        for (int i = 0; i < 90; ++i) {
            flick.update(kDt, turn, track);
            maxYawFlick = std::max(maxYawFlick, std::fabs(flick.yawRate));
        }
        check(flick.drifting, "speed: full lock at 55 spills into drift",
              flick.drifting ? 1.0f : 0.0f);
        check(maxYawFlick < 2.00f, "speed: no snap spin at 55 (rad/s)", maxYawFlick);

        Car slow;
        slow.reset(track);
        accelerateTo(slow, track, 18.0f);
        float maxYawSlow = 0;
        for (int i = 0; i < 60; ++i) {
            slow.update(kDt, turn, track);
            maxYawSlow = std::max(maxYawSlow, std::fabs(slow.yawRate));
        }
        check(maxYawSlow > 0.55f, "speed: still agile at 18 m/s (rad/s)", maxYawSlow);
    }

    // ---- 7. 軽いブレーキでも少し滑る ----------------------------------------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        auto cornerSlip = [&](float brake, float throttle) {
            Car car;
            car.reset(track);
            accelerateTo(car, track, 25.0f);
            DriveInput in;
            in.steer = 0.55f;  // フリック突入 (0.65) 未満にして、ブレーキの効果だけを見る
            in.brake = brake;
            in.throttle = throttle;
            float maxSlip = 0;
            for (int i = 0; i < 60; ++i) {
                car.update(kDt, in, track);
                maxSlip = std::max(maxSlip, std::fabs(car.slipAngle));
            }
            return maxSlip;
        };
        float plain = cornerSlip(0.0f, 0.45f);
        float light = cornerSlip(0.30f, 0.0f);
        check(light > plain + 0.05f, "light brake: slips more than plain cornering (rad)",
              light - plain);
        check(light < 1.00f, "light brake: held brake drifts deep but never spins (rad)", light);
    }

    // ---- 8. ドリフト中に速度が乗る ---------------------------------------------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        Car car;
        car.reset(track);
        accelerateTo(car, track, 27.0f);
        DriveInput pull;
        pull.steer = 1.0f;
        pull.handbrake = true;
        run(car, track, pull, 0.55f);
        float startSpeed = car.groundSpeed();
        DriveInput hold;
        hold.steer = -0.65f;
        hold.throttle = 0.9f;
        run(car, track, hold, 2.2f);
        float endSpeed = car.groundSpeed();
        check(endSpeed > startSpeed * 1.15f, "drift speed: pace builds while drifting (ratio)",
              endSpeed / startSpeed);
        check(endSpeed > 24.0f, "drift speed: fast in absolute terms (m/s)", endSpeed);
        check(finite(car), "drift speed: state stays finite", car.speed);
    }

    // ---- 9. ステアリングだけでもドリフトに入る --------------------------------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        Car car;
        car.reset(track);
        accelerateTo(car, track, 28.0f);
        DriveInput flick;
        flick.steer = 1.0f;
        flick.throttle = 0.55f;  // パワースライド条件 (0.85) 未満。ブレーキもサイドもなし
        float maxSlip = 0;
        bool wasDrifting = false;
        for (int i = 0; i < 60 * 2; ++i) {
            car.update(kDt, flick, track);
            maxSlip = std::max(maxSlip, std::fabs(car.slipAngle));
            wasDrifting = wasDrifting || car.drifting;
        }
        check(maxSlip > 0.28f, "steer only: hard steering breaks the rear loose (rad)", maxSlip);
        check(wasDrifting, "steer only: drift state turns on", wasDrifting ? 1.0f : 0.0f);
        check(finite(car), "steer only: state stays finite", car.speed);
    }

    // ---- 10. サイドブレーキは即ドリフト状態、ステアで向きを傾けられる --------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        Car car;
        car.reset(track);
        accelerateTo(car, track, 27.0f);

        // まっすぐ走っていても、サイドを引いた瞬間にドリフト状態へ移行する。
        DriveInput side;
        side.handbrake = true;
        run(car, track, side, 0.1f);
        check(car.drifting, "handbrake: switches to drift state at once",
              car.drifting ? 1.0f : 0.0f);

        // ドリフト中にステアを切ると、車の向きを大きく傾けられる。
        DriveInput pull;
        pull.steer = 1.0f;
        pull.handbrake = true;
        run(car, track, pull, 0.45f);
        float yawBefore = car.yaw;
        DriveInput counter;
        counter.steer = -1.0f;
        counter.throttle = 0.7f;
        run(car, track, counter, 0.8f);
        float yawSwing = std::fabs(wrapPi(car.yaw - yawBefore));
        check(yawSwing > 0.75f, "drift steer: steering swings the nose (rad)", yawSwing);
        check(finite(car), "drift steer: state stays finite", car.speed);
    }

    // ---- 11. ステア＋サイドの組み合わせは即ドリフト --------------------------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        Car car;
        car.reset(track);
        accelerateTo(car, track, 22.0f);
        DriveInput combo;
        combo.steer = 1.0f;
        combo.handbrake = true;
        float maxSlip = 0;
        for (int i = 0; i < static_cast<int>(0.35f / kDt); ++i) {
            car.update(kDt, combo, track);
            maxSlip = std::max(maxSlip, std::fabs(car.slipAngle));
        }
        check(car.drifting, "combo: steer + handbrake is drifting", car.drifting ? 1.0f : 0.0f);
        check(maxSlip > 0.45f, "combo: rear steps out at once (rad)", maxSlip);
        check(finite(car), "combo: state stays finite", car.speed);
    }

    // ---- 12. ドリフトの終わり方：無操作では維持、カウンターで復帰 -------------------
    {
        Track track = makeCircle(150.0f, 60.0f);
        // withCounter=true: カウンターで姿勢を保つ入り方（復帰操作の検証用）。
        // false: ステア中立のまま滑らせた深いドリフト（手を離す検証用）。
        auto enterDrift = [&](bool withCounter) {
            Car car;
            car.reset(track);
            accelerateTo(car, track, 27.0f);
            DriveInput pull;
            pull.steer = 1.0f;
            pull.handbrake = true;
            run(car, track, pull, 0.55f);
            DriveInput hold;
            hold.steer = withCounter ? -0.65f : 0.0f;
            hold.throttle = 0.85f;
            run(car, track, hold, 0.4f);
            return car;
        };

        // 無操作：深いドリフトの最中に全部離しても、ドリフト状態は維持される
        // （すぐには戻らない）。ただし、さらに長く離せば時間をかけて戻る。
        {
            Car car = enterDrift(false);
            check(car.drifting && std::fabs(car.slipAngle) > 0.30f,
                  "exit: starts from a deep drift (rad)", std::fabs(car.slipAngle));
            DriveInput none;
            run(car, track, none, 1.6f);
            check(car.drifting, "exit: no input keeps the drift state (1.6s)",
                  car.drifting ? 1.0f : 0.0f);
            run(car, track, none, 9.0f);
            check(!car.drifting, "exit: no input still recovers eventually",
                  car.drifting ? 1.0f : 0.0f);
        }

        // カウンターステア：滑っている向きへ、滑りの量に応じて切ると
        // グリップ状態へ戻れる（実際の運転と同じく、当て続けるのではなく
        // 滑りが小さくなるにつれて戻していく操作を再現する）。
        {
            Car car = enterDrift(true);
            float exitTime = -1.0f;
            for (int i = 0; i < 60 * 2 && exitTime < 0.0f; ++i) {
                DriveInput counter;
                counter.steer = clampf(car.slipAngle * 2.2f, -0.9f, 0.9f);
                counter.throttle = 0.3f;
                car.update(kDt, counter, track);
                if (!car.drifting) exitTime = (i + 1) * kDt;
            }
            check(exitTime > 0.0f && exitTime < 1.6f, "exit: counter-steer recovers to grip (s)",
                  exitTime);
            check(finite(car), "exit: state stays finite", car.speed);
        }
    }

    std::printf(failures == 0 ? "\nALL PASS\n" : "\n%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
