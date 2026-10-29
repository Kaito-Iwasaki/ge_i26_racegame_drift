#include "car.hpp"

#include <cmath>

namespace {

constexpr float kMaxSpeed = 160.0f / 3.6f;
constexpr float kMaxReverse = 9.0f;
constexpr float kAccel = 15.0f;
constexpr float kBrake = 30.0f;
constexpr float kReverseAccel = 8.0f;
constexpr float kDrag = 0.00028f;
constexpr float kRolling = 0.9f;
constexpr float kWheelbase = 3.05f;
constexpr float kSteerRate = 4.6f;
constexpr float kSteerRateDrift = 5.0f;
constexpr float kWheelRadius = 0.34f;
constexpr float kGripAsphalt = 9.0f;
constexpr float kGripGrass = 4.5f;
constexpr float kHandbrakeDecel = 5.5f;
constexpr float kLateralLimit = 14.0f; // Base lateral acceleration (m/s^2).
constexpr float kDriftCornerAssist = 16.0f; // Arcade allowance for tight hairpins.

float maxSteerAt(float speed) {
    // 切れ角は速度で絞る。分母に速度の2乗の項を入れるのが肝心で、1次だけだと
    // (v / L) * tan(steer) が高速でもほぼ一定になってしまい、200 km/h 超でも
    // クイックに回ってしまう。2乗項で「高速では穏やかに」と「中低速では
    // しっかり曲がる」を両立させる。曲がりすぎた分はドリフト移行が受け止める。
    const float s = std::fabs(speed);
    return 0.85f / (1.0f + s * 0.025f + s * s * 0.0025f);
}

// 値 current を rate (1/s) で target へ指数的に近づける。
float approachExp(float current, float target, float rate, float dt) {
    return current + (target - current) * (1.0f - std::exp(-rate * dt));
}

}  // namespace

float Car::groundSpeed() const {
    return std::sqrt(velX * velX + velZ * velZ);
}

void Car::reset(const Track& track) {
    x = track.spawnX;
    y = track.spawnY;
    z = track.spawnZ;
    yaw = track.spawnYaw;
    speed = 0;
    steerHold = 0;
    steerInput = 0;
    steerAngle = 0;
    wheelSpin = 0;
    brakeHold = 0;
    onAsphalt = true;
    velX = 0;
    velZ = 0;
    velForward = 0;
    velLateral = 0;
    yawRate = 0;
    slipAngle = 0;
    drifting = false;
    driftTime = 0;
    driftScore = 0;
    gripLat = kGripAsphalt;
    driftAmount = 0;
    slideEntryTime = 0;
    brakeDriftMix = 0;
    ground = track.query(x, z, y);
}

void Car::stepPhysics(float dt, const DriveInput& input) {
    if (dt <= 0.0f) return;
    const float mag = groundSpeed();
    const float fx = std::sin(yaw), fz = std::cos(yaw);
    const float vF = velX * fx + velZ * fz;
    const float vL = velX * fz - velZ * fx;
    const float slip = mag > 0.1f ? std::atan2(vL, std::fabs(vF) + 0.001f) : 0.0f;
    const float slipTarget = clampf((std::fabs(slip) - 0.06f) / 0.30f, 0.0f, 1.0f);
    driftAmount = approachExp(driftAmount, slipTarget, 5.0f, dt);

    // A single continuous load criterion, with no upper speed gate.
    // Arcade cornering support grows with slip; initiation uses the base grip limit.
    const float baseLimit = kLateralLimit * (onAsphalt ? 1.0f : 0.55f);
    const float slideMaturity = driftAmount * clampf((std::fabs(slip) - 0.12f) / 0.10f, 0.0f, 1.0f);
    slideEntryTime = slideMaturity > 0.15f ? std::min(slideEntryTime + dt, 2.0f) : 0.0f;
    const bool braking = input.handbrake || input.brake > 0.05f;
    brakeDriftMix = braking ? 1.0f : approachExp(brakeDriftMix, 0.0f, 2.0f, dt);
    const float limit = baseLimit + (onAsphalt ? kDriftCornerAssist : 12.0f) * slideMaturity;
    const float demand = vF * vF / kWheelbase * std::tan(steerAngle);
    const float overload = std::tanh(std::max(0.0f, std::fabs(demand) / baseLimit - 1.0f)) *
        clampf((std::fabs(steerInput) - 0.70f) / 0.30f, 0.0f, 1.0f) *
        clampf((steerHold - 0.30f) / 0.60f, 0.0f, 1.0f);
    const float slideHold = slideMaturity * clampf((input.throttle - 0.2f) / 0.4f, 0.0f, 1.0f);
    const float release = std::max(std::max(overload, input.handbrake ? 1.0f : 0.0f), slideHold);
    const float handbrakeGrip = input.handbrake ? 0.40f : 0.0f;
    const float gripTarget = (onAsphalt ? kGripAsphalt : kGripGrass) * (1.0f - 0.30f * release - handbrakeGrip);
    gripLat = approachExp(gripLat, gripTarget, 5.0f, dt);

    // Body attitude grows towards a slip angle; velocity is never rotated by yaw.
    const float moving = clampf(mag / 5.0f, 0.0f, 1.0f);
    const float steeringSlip = (0.65f * release + (input.handbrake ? 0.15f : 0.0f)) * steerInput * moving;
    const bool counter = steerInput * slip > 0;
    const float maintainedSlip = -slip * (1.08f - (counter ? 0.70f * std::fabs(steerInput) : 0.0f));
    const float holding = slideHold * (counter ? 1.0f : 1.0f - std::fabs(steerInput));
    const float desiredSlip = steeringSlip * (1.0f - holding) + maintainedSlip * holding;
    // Releasing or countersteering reduces how hard the slide pulls the path inward.
    const float aligned = clampf(-steerInput * std::tanh(slip / 0.10f), 0.0f, 1.0f);
    const float entryFade = 1.0f - clampf((slideEntryTime - 0.4f) / 0.6f, 0.0f, 1.0f);
    const float straightEntry = 0.25f * slideMaturity * entryFade * (1.0f - brakeDriftMix);
    const float cornerScale = (1.0f - 0.65f * slideMaturity * (1.0f - aligned)) * (1.0f - straightEntry);
    const float tyreAccel = -limit * std::tanh(gripLat * vL / limit) * cornerScale;
    const float pathRate = tyreAccel * vF / std::max(mag * mag, 1.0f);
    const float bicycle = vF / kWheelbase * std::tan(steerAngle);
    float yawTarget = bicycle * (1.0f - release) +
        (pathRate + 3.5f * (desiredSlip + slip)) * release;
    const float yawLimit = (limit / std::max(mag, 5.0f)) + 0.8f * release;
    yawTarget = clampf(yawTarget, -yawLimit, yawLimit);
    const float response = 10.0f - 6.0f * driftAmount;
    const float nextYawRate = approachExp(yawRate, yawTarget, response, dt);
    yawRate += clampf(nextYawRate - yawRate, -5.5f * dt, 5.5f * dt);
    yaw = wrapPi(yaw + yawRate * dt);

    const float nfx = std::sin(yaw), nfz = std::cos(yaw);
    float forward = velX * nfx + velZ * nfz;
    float lateral = velX * nfz - velZ * nfx;
    const float lateralAccel = -limit * std::tanh(gripLat * lateral / limit) * cornerScale;
    const float lateralBefore = lateral;
    lateral += clampf(lateralAccel * dt, -std::fabs(lateral), std::fabs(lateral));
    // Arcade drift drive offsets part of tyre scrub under throttle.
    // It returns only dissipated energy, never adds a minimum speed or cancels braking.
    if (forward > 3.0f && input.brake <= 0.05f) {
        const float assist = (input.handbrake ? 0.65f : 0.90f) * driftAmount * clampf(input.throttle, 0.0f, 1.0f);
        const float lostEnergy = std::max(0.0f, lateralBefore * lateralBefore - lateral * lateral);
        forward = std::sqrt(forward * forward + assist * lostEnergy);
    }
    // Extra scrub grows continuously above the sustained drift speed.
    const float powerScrub = 6.0f - 2.0f * clampf(input.throttle, 0.0f, 1.0f) *
        ((input.handbrake || input.brake > 0.05f) ? 0.0f : 1.0f);
    const float scrub = driftAmount * powerScrub * std::max(0.0f, mag - 30.0f);
    forward *= std::exp(-scrub / std::max(mag, 1.0f) * dt);
    velX = nfx * forward + nfz * lateral;
    velZ = nfz * forward - nfx * lateral;
    velForward = speed = forward;
    velLateral = lateral;
    slipAngle = groundSpeed() > 1.0f ? std::atan2(lateral, std::fabs(forward) + 0.001f) : 0.0f;

    // Hysteresis affects scoring and presentation only.
    if (!drifting && std::fabs(slipAngle) > 0.19f && groundSpeed() > 9.0f) drifting = true;
    if (drifting && (std::fabs(slipAngle) < 0.12f || groundSpeed() < 2.5f)) drifting = false;
    if (drifting) {
        driftTime += dt;
        driftScore += groundSpeed() * std::fabs(slipAngle) * dt * 10.0f;
    } else {
        driftTime = 0;
    }
    x += velX * dt;
    z += velZ * dt;
}

void Car::update(float dt, const DriveInput& input, const Track& track) {
    // 速度ベクトルが正。前方成分を speed に同期してから縦の力を計算する。
    {
        const float fx = std::sin(yaw), fz = std::cos(yaw);
        const float rx = std::cos(yaw), rz = -std::sin(yaw);
        velForward = velX * fx + velZ * fz;
        velLateral = velX * rx + velZ * rz;
        speed = velForward;
    }

    if (dt <= 0.0f) return;
    const float restoreMix = driftAmount;
    float steerLimit = maxSteerAt(groundSpeed()) * (1.0f + 0.35f * restoreMix);
    // Smooth the same steering command for wheels and drift attitude; keyboard keys are binary.
    const bool holdingSameDirection = input.steer * steerInput >= 0.0f;
    steerHold = (std::fabs(input.steer) > 0.85f && holdingSameDirection)
        ? std::min(steerHold + dt, 1.0f) : 0.0f;
    steerInput = approachExp(steerInput, clampf(input.steer, -1.0f, 1.0f), 8.0f - 4.0f * restoreMix, dt);
    const float steer = steerInput;
    // Fine corrections retain grip; full lock requests the tight drift line.
    const float fastSteer = clampf(groundSpeed() / 20.0f, 0.0f, 1.0f);
    float target = steer * (1.0f + 0.65f * (steer * steer - 1.0f) * fastSteer) * steerLimit;
    float step = (kSteerRate + (kSteerRateDrift - kSteerRate) * restoreMix) * dt;
    steerAngle += clampf(target - steerAngle, -step, step);
    steerAngle = clampf(steerAngle, -0.6f, 0.6f);

    // 一度後退に入ったら、ブレーキを離すまで後退を維持する。
    // 速度の絶対値だけで判定すると、後退し始めた瞬間に brakeHold が消えて
    // 「少し下がって止まる」を繰り返してしまうため（従来挙動の不具合）。
    if ((std::fabs(speed) < 0.45f || brakeHold > 0.28f) && input.brake > 0.5f && input.throttle < 0.1f) {
        brakeHold += dt;
    } else {
        brakeHold = 0;
    }
    bool reversing = brakeHold > 0.28f;

    float force = 0;
    float speedBefore = speed;
    if (input.handbrake && speed > 0.2f) {
        // サイドブレーキ中は駆動力を弱め、後輪ロックで減速する。
        // 通常ブレーキほどは止まらないので、滑りながら速度を保てる。
        // 突入時はしっかり落とし、いったんドリフトに入ったら減速を弱めて
        // 「引きっぱなしでも失速しすぎない」ようにする。
        force -= kHandbrakeDecel - (kHandbrakeDecel - 2.2f) * restoreMix;
        // Holding throttle while pulling the handbrake keeps some engine drive.
        const float ratio = clampf(groundSpeed() / kMaxSpeed, 0.0f, 1.0f);
        force += input.throttle * kAccel * 0.35f * (1.0f - ratio * ratio);
    } else if (input.brake > 0.05f && !reversing) {
        force -= input.brake * kBrake * (speed >= 0 ? 1.0f : -1.0f);
    } else if (reversing) {
        force -= input.brake * kReverseAccel;
    } else if (input.throttle > 0.0f && speed > -0.4f) {
        float ratio = clampf(groundSpeed() / kMaxSpeed, 0.0f, 1.0f);
        const float boost = 1.0f + 0.3f * driftAmount;
        const float traction = 1.0f - 0.15f * driftAmount;
        float drive = kAccel * (1.0f - ratio * ratio) * boost * traction;
        force += input.throttle * drive;

    }
    force -= kDrag * speed * std::fabs(speed);
    if (std::fabs(speed) > 0.05f) {
        force -= kRolling * (speed > 0 ? 1.0f : -1.0f);
    }
    // 芝は加速が鈍く、速度も伸びない。横グリップも落ちる（stepPhysics 側）。
    if (!onAsphalt) {
        if (force > 0) {
            force *= 0.55f;
        }
        if (std::fabs(speed) > 0.2f) {
            force -= 3.2f * (speed > 0 ? 1.0f : -1.0f);
        }
    }

    speed += force * dt;
    if (!reversing && input.brake > 0.05f) {
        if (speedBefore >= 0 && speed < 0) speed = 0;
        if (speedBefore <= 0 && speed > 0) speed = 0;
    }
    if (input.handbrake && speedBefore > 0 && speed < 0) speed = 0;
    // Do not snap externally supplied overspeed down in a single frame.
    speed = clampf(speed, -kMaxReverse, std::max(kMaxSpeed, speedBefore));

    // 縦方向の力は車体の前方へ速度ベクトルを増減させる。横成分は触らない。
    {
        const float fx = std::sin(yaw), fz = std::cos(yaw);
        const float delta = speed - velForward;
        velX += fx * delta;
        velZ += fz * delta;
        velForward = speed;
    }

    stepPhysics(dt, input);
    wheelSpin += speed / kWheelRadius * dt;

    float preferS = ground.found ? ground.s : -1.0f;
    ground = track.query(x, z, y, preferS);
    if (!ground.found) {
        return;
    }

    const float wall = track.wallLateral();
    if (std::fabs(ground.lateral) > wall) {
        float sign = ground.lateral > 0 ? 1.0f : -1.0f;
        float overflow = std::fabs(ground.lateral) - wall;
        x -= ground.rightX * sign * overflow;
        z -= ground.rightZ * sign * overflow;
        // 壁の外へ向かう速度成分は落とす。ドリフトで横向きに当たっても
        // 壁に張り付かず、コースに沿う成分だけ残して擦りながら進める。
        float outVel = (velX * ground.rightX + velZ * ground.rightZ) * sign;
        if (outVel > 0) {
            velX -= ground.rightX * sign * outVel * 0.9f;
            velZ -= ground.rightZ * sign * outVel * 0.9f;
        }
        // 壁処理のあとはベクトルとスカラーを一致させる。
        const float fx = std::sin(yaw), fz = std::cos(yaw);
        const float rx = std::cos(yaw), rz = -std::sin(yaw);
        velForward = velX * fx + velZ * fz;
        velLateral = velX * rx + velZ * rz;
        speed = velForward;
        ground.lateral = sign * wall;
    }

    float bankLimit = track.halfWidth + track.curbWidth;
    float bankLat = clampf(ground.lateral, -bankLimit, bankLimit);
    y = ground.centerY + bankLat * ground.cross;
    onAsphalt = std::fabs(ground.lateral) <= track.halfWidth + 0.15f;
}

void buildCarBody(CpuMesh& mesh) {
    // AE86（スプリンタートレノ）風の3ドアハッチバック。
    // 白×黒のパンダツートン、角ばったウェッジシェイプ、寝たハッチガラス、
    // 黒いリアガーニッシュとバンパーが要点。前方が +z。
    const Vec3 white{0.92f, 0.92f, 0.90f};
    const Vec3 black{0.05f, 0.05f, 0.06f};
    const Vec3 glass{0.07f, 0.09f, 0.12f};
    const Vec3 lidGray{0.78f, 0.78f, 0.76f};
    const Vec3 tailRed{0.70f, 0.05f, 0.06f};
    const Vec3 amber{0.85f, 0.45f, 0.08f};
    const Vec3 steel{0.25f, 0.25f, 0.27f};

    // 下半分は黒帯（パンダ塗装）: バンパーと一体で車体を一周する。
    mesh.addBox({-0.845f, 0.24f, -2.02f}, {0.845f, 0.46f, 2.06f}, black);
    // 白い上半分（ボンネットは平ら、ベルトライン 0.66）。
    mesh.addBox({-0.82f, 0.44f, -1.98f}, {0.82f, 0.66f, 2.04f}, white);
    // 前後バンパー（黒、少し張り出し）。
    mesh.addBox({-0.85f, 0.28f, 2.02f}, {0.85f, 0.50f, 2.16f}, black);
    mesh.addBox({-0.85f, 0.26f, -2.16f}, {0.85f, 0.48f, -2.00f}, black);
    // フロントウインカー。
    mesh.addBox({0.52f, 0.36f, 2.10f}, {0.78f, 0.44f, 2.17f}, amber);
    mesh.addBox({-0.78f, 0.36f, 2.10f}, {-0.52f, 0.44f, 2.17f}, amber);
    // リトラクタブルライト（閉じた状態）: 蓋の輪郭を黒い溝＋灰色の蓋で描く。
    mesh.addBox({0.26f, 0.645f, 1.76f}, {0.78f, 0.665f, 2.03f}, black);
    mesh.addBox({-0.78f, 0.645f, 1.76f}, {-0.26f, 0.665f, 2.03f}, black);
    mesh.addBox({0.28f, 0.66f, 1.78f}, {0.76f, 0.685f, 2.01f}, lidGray);
    mesh.addBox({-0.76f, 0.66f, 1.78f}, {-0.28f, 0.685f, 2.01f}, lidGray);

    // グリーンハウス: 傾斜したガラス面＋白いルーフ。
    // フロントガラス
    mesh.addQuad({-0.63f, 0.65f, 1.02f}, {0.63f, 0.65f, 1.02f},
                 {0.57f, 1.04f, 0.42f}, {-0.57f, 1.04f, 0.42f},
                 {0, 0.84f, 0.55f}, glass);
    // ルーフ
    mesh.addBox({-0.615f, 1.03f, -0.62f}, {0.615f, 1.10f, 0.47f}, white);
    // ハッチガラス（急角度で寝る）
    mesh.addQuad({-0.57f, 1.04f, -0.60f}, {0.57f, 1.04f, -0.60f},
                 {0.63f, 0.66f, -1.60f}, {-0.63f, 0.66f, -1.60f},
                 {0, 0.93f, -0.36f}, glass);
    // サイドガラス（左右）
    mesh.addQuad({0.645f, 0.65f, 1.00f}, {0.645f, 0.65f, -1.56f},
                 {0.585f, 1.035f, -0.57f}, {0.585f, 1.035f, 0.40f},
                 {1, 0.1f, 0}, glass);
    mesh.addQuad({-0.645f, 0.65f, -1.56f}, {-0.645f, 0.65f, 1.00f},
                 {-0.585f, 1.035f, 0.40f}, {-0.585f, 1.035f, -0.57f},
                 {-1, 0.1f, 0}, glass);
    // ドアミラー（黒い角型）。
    mesh.addBox({0.80f, 0.82f, 0.90f}, {0.95f, 0.93f, 1.02f}, black);
    mesh.addBox({-0.95f, 0.82f, 0.90f}, {-0.80f, 0.93f, 1.02f}, black);

    // リア: 黒ガーニッシュ＋赤テールランプ＋マフラー。
    mesh.addBox({-0.79f, 0.58f, -2.00f}, {0.79f, 0.82f, -1.93f}, black);
    mesh.addBox({0.30f, 0.63f, -2.03f}, {0.77f, 0.77f, -1.96f}, tailRed);
    mesh.addBox({-0.77f, 0.63f, -2.03f}, {-0.30f, 0.77f, -1.96f}, tailRed);
    mesh.addBox({0.48f, 0.14f, -2.20f}, {0.68f, 0.24f, -2.02f}, steel);
}

void buildCarBrakeLights(CpuMesh& mesh) {
    // 色は 1 を超えてよい（シェーダーの陰影 0.36〜 を掛けても明るい赤になり、
    // 通常の暗いテール（0.70, 0.05, 0.06）との差がはっきり出る）。
    const Vec3 lit{3.0f, 0.18f, 0.14f};
    mesh.addBox({0.295f, 0.625f, -2.045f}, {0.775f, 0.775f, -1.955f}, lit);
    mesh.addBox({-0.775f, 0.625f, -2.045f}, {-0.295f, 0.775f, -1.955f}, lit);
}

void buildCarWheel(CpuMesh& mesh) {
    const int seg = 14;
    const float radius = 0.34f;
    const float halfW = 0.15f;
    const Vec3 tire{0.07f, 0.07f, 0.08f};
    const Vec3 rim{0.13f, 0.13f, 0.14f};  // 黒いホイール（ワタナベ風）
    for (int i = 0; i < seg; ++i) {
        float a0 = float(i) / float(seg) * 6.2831853f;
        float a1 = float(i + 1) / float(seg) * 6.2831853f;
        float c0 = std::cos(a0);
        float s0 = std::sin(a0);
        float c1 = std::cos(a1);
        float s1 = std::sin(a1);
        Vec3 normal = normalize(Vec3{0, c0 + c1, s0 + s1});
        Vec3 a{-halfW, c0 * radius, s0 * radius};
        Vec3 b{halfW, c0 * radius, s0 * radius};
        Vec3 c{halfW, c1 * radius, s1 * radius};
        Vec3 d{-halfW, c1 * radius, s1 * radius};
        mesh.addQuad(a, b, c, d, normal, tire);
        for (int side = -1; side <= 1; side += 2) {
            float x = float(side) * halfW;
            Vec3 nrm{float(side), 0, 0};
            Vec3 center{x, 0, 0};
            Vec3 p0{x, c0 * radius * 0.62f, s0 * radius * 0.62f};
            Vec3 p1{x, c1 * radius * 0.62f, s1 * radius * 0.62f};
            Vec3 o0{x, c0 * radius, s0 * radius};
            Vec3 o1{x, c1 * radius, s1 * radius};
            mesh.addQuad(o0, o1, p1, p0, nrm, tire);
            mesh.addTriangle(center, p0, p1, nrm, rim);
        }
    }
}

void buildCarShadow(CpuMesh& mesh) {
    const Vec3 ink{0.02f, 0.02f, 0.025f};
    mesh.addQuad({-1.02f, 0, -2.28f}, {1.02f, 0, -2.28f}, {1.02f, 0, 2.24f}, {-1.02f, 0, 2.24f}, {0, 1, 0}, ink);
}

Mat4 carModelMatrix(const Car& car, float pitch, float roll) {
    Vec3 forward = normalize(Vec3{
        std::sin(car.yaw) * std::cos(pitch),
        std::sin(pitch),
        std::cos(car.yaw) * std::cos(pitch),
    });
    Vec3 right0{std::cos(car.yaw), 0, -std::sin(car.yaw)};
    Vec3 up0 = normalize(cross(forward, right0));
    Vec3 right = normalize(right0 * std::cos(roll) + up0 * std::sin(roll));
    Vec3 up = normalize(cross(forward, right));
    return basis({car.x, car.y, car.z}, right, up, forward);
}
