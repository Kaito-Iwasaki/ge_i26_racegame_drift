#include "car.hpp"

#include <cmath>

namespace {

constexpr float kMaxSpeed = 88.0f;      // 約 317 km/h
constexpr float kMaxReverse = 9.0f;
constexpr float kAccel = 15.0f;
constexpr float kBrake = 30.0f;
constexpr float kReverseAccel = 8.0f;
constexpr float kDrag = 0.00045f;
constexpr float kRolling = 0.9f;
constexpr float kWheelbase = 3.05f;
constexpr float kSteerRate = 4.6f;       // グリップ走行の切り込みの速さ（応答性）
constexpr float kSteerRateDrift = 5.0f;  // カウンターステアを素早く当てるため
constexpr float kWheelRadius = 0.34f;

// --- ドリフト調整値 ---------------------------------------------------------
// 横グリップ係数 grip (1/s): 横速度が exp(-grip*dt) で減衰する速さ。
// 大きいほど「レールに乗った」走り、小さいほど滑る。
constexpr float kGripAsphalt = 9.0f;    // 通常走行（アスファルト）
constexpr float kGripGrass = 4.5f;      // 通常走行（芝）。元から滑りやすい
constexpr float kGripHandbrake = 1.5f;  // サイドブレーキ中。後輪がロックした状態
constexpr float kGripDrift = 1.45f;     // ドリフト状態の間ずっと保つ横グリップ（よく滑る）
constexpr float kGripBrakeTurn = 3.1f;  // ブレーキを踏みながら大きく切ったとき（最大）
constexpr float kGripInitiate = 3.2f;   // フルロック＋フルアクセルが続いたとき
constexpr float kGripSteerSlide = 1.9f;   // ステアだけで大きく切ったとき（フリック）
constexpr float kGripCombo = 1.0f;      // ステア＋サイドの組み合わせ（後輪はほぼフリー）
constexpr float kYawResponseGrip = 15.5f;   // ヨーレートが目標へ追従する速さ (1/s)
constexpr float kYawResponseDrift = 12.5f;  // ドリフト中もステアで向きを動かせる追従
constexpr float kDriftEnterSlip = 0.19f;    // ドリフト開始のスリップ角 (rad, 約11°)
constexpr float kDriftExitSlip = 0.12f;     // ここまで滑りが収まったらグリップへ戻れる (rad)
constexpr float kDriftRecoverBase = 0.075f;  // 何もしなくても溜まる回復の速さ (/s)。とてもゆっくり
constexpr float kDriftRecoverCounter = 1.7f;  // カウンターステア時の上乗せ (/s)
constexpr float kDriftRecoverDeep = 0.45f;    // 深い滑りでのカウンターの上乗せ (/s)
constexpr float kDriftDeepSlip = 0.25f;     // これより深い滑りではカウンターはバランス取り扱い
constexpr float kHandbrakeDecel = 5.5f;     // サイドブレーキの減速度 (m/s^2)

float maxSteerAt(float speed) {
    // 切れ角は速度で絞る。分母に速度の2乗の項を入れるのが肝心で、1次だけだと
    // (v / L) * tan(steer) が高速でもほぼ一定になってしまい、200 km/h 超でも
    // クイックに回ってしまう。2乗項で「高速では穏やかに」と「中低速では
    // しっかり曲がる」を両立させる。曲がりすぎた分はドリフト移行が受け止める。
    const float s = std::fabs(speed);
    return 0.70f / (1.0f + s * 0.041f + s * s * 0.0037f);
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
    powerSlideHold = 0;
    steerSlideHold = 0;
    driftRecovery = 0;
    driftCounterAge = 10.0f;
    slipEntryHold = 0;
    postureHold = 0;
    settledHold = 0;
    ground = track.query(x, z, y);
}

void Car::stepPhysics(float dt, const DriveInput& input) {
    // ------------------------------------------------------------------------
    // グリップ走行とドリフトを一つのモデルで扱う。
    //
    // 考え方:
    //   速度は世界座標のベクトル (velX, velZ) で持ち、慣性でまっすぐ進もうとする。
    //   車体がヨーで回っても速度ベクトルはすぐには付いてこない。その差を車体座標に
    //   分解した横成分 velLateral が「横滑り」で、横グリップがそれを毎ステップ
    //   exp(-grip*dt) だけ消していく。グリップが高い通常時は横成分が即座に消えて
    //   従来のグリップ走行とほぼ同じ挙動になり、サイドブレーキなどでグリップを
    //   落とすと横成分が残ってドリフトになる。
    //
    // ヨー:
    //   目標ヨーレートは滑らない自転車モデル (speed / wheelbase) * tan(steer)。
    //   ただし即座にその値にはせず、yawRate を慣性付きで追従させる。ドリフト中は
    //   追従を遅くし、切れ角の上限も広げてカウンターステアで姿勢を戻せるようにする。
    //   低速の据え切り（壁際で止まっても向きを変えられる）は従来通り残す。
    // ------------------------------------------------------------------------

    // 車体座標への分解。forward = (sin yaw, cos yaw), right = (cos yaw, -sin yaw)
    const float fx = std::sin(yaw), fz = std::cos(yaw);
    const float rx = std::cos(yaw), rz = -std::sin(yaw);
    float vF = velX * fx + velZ * fz;
    float vL = velX * rx + velZ * rz;
    const float speedNow = std::sqrt(vF * vF + vL * vL);
    const float slipNow = (speedNow > 1.0f) ? std::atan2(vL, std::fabs(vF) + 0.001f) : 0.0f;

    // --- ドリフト突入 -------------------------------------------------------
    // サイドブレーキは押した瞬間にドリフト状態へ入る（滑りが育つのを待たない）。
    // それ以外は、スリップ角が育った時点で入る（フリック／ブレーキ／パワースライド）。
    if (!drifting) {
        // スリップ角での突入は「少しの間キープされた滑り」だけを拾う。
        // 軽いブレーキなどで一瞬跳ねただけの滑りでは状態に入らない
        // （入った途端に横グリップが落ち、滑りが暴走するのを防ぐ）。
        if (std::fabs(slipNow) > kDriftEnterSlip && speedNow > 9.0f) {
            slipEntryHold += dt;
        } else {
            slipEntryHold = 0.0f;
        }
        if ((input.handbrake && speedNow > 3.5f) || slipEntryHold > 0.09f) {
            drifting = true;
            driftRecovery = 0;
            driftCounterAge = 10.0f;
            slipEntryHold = 0.0f;
            settledHold = 0.0f;
            gripRestore = 0.0f;
        }
    }

    // --- 横グリップの目標値を決める --------------------------------------
    // 条件が重なったら一番小さい（滑る）ものを採用する。
    float gripTarget = onAsphalt ? kGripAsphalt : kGripGrass;

    // 大きく切ってアクセルを踏み続けると後輪が空転して滑り出す。
    // 一瞬のステア修正で誤発しないよう継続時間で判定する。
    const float steerLimit = maxSteerAt(vF);
    bool powerCondition = input.throttle > 0.80f && steerLimit > 0.01f &&
                          std::fabs(steerAngle) > steerLimit * 0.75f &&
                          vF > 10.0f && vF < 42.0f && onAsphalt;
    powerSlideHold = powerCondition ? powerSlideHold + dt : 0.0f;
    if (powerSlideHold > 0.20f) {
        gripTarget = std::min(gripTarget, kGripInitiate);
    }

    // ステアリングだけでも滑り出しを作る（フリック）。
    // 速度が乗った状態で大きく切ると、荷重移動で後ろが軽くなってお尻が出る。
    // アクセルもブレーキも要らない。直進からでも、曲がりすぎたらそのまま
    // ドリフトへ、が狙い。一瞬の当て舵では誤発しないよう、0.07 秒だけ
    // 継続を要求する（体感では即時）。通常コーナリングの切り角（〜0.6）では
    // 発動しないよう、閾値は 0.72 に置く。
    bool steerCondition = std::fabs(input.steer) > 0.72f && vF > 8.0f && vF < 70.0f &&
                          onAsphalt;
    steerSlideHold = steerCondition ? steerSlideHold + dt : 0.0f;
    if (steerSlideHold > 0.07f) {
        gripTarget = std::min(gripTarget, kGripSteerSlide);
    }

    // ブレーキを踏みながら切ると荷重が前へ移って後ろが軽くなり、滑り出す。
    // 以前は「強ブレーキ＋大切り」の二値判定だったので、それ以外では滑らなかった。
    // 踏力・切り角・速度に比例して連続的にグリップを落とす：軽く踏んだだけでも
    // 少し滑り、強く踏めばしっかりドリフトに入る。
    if (input.brake > 0.05f && vF > 8.0f && onAsphalt) {
        float press = clampf((input.brake - 0.05f) / 0.34f, 0.0f, 1.0f);
        float cut = clampf((std::fabs(input.steer) - 0.10f) / 0.45f, 0.0f, 1.0f);
        float fast = clampf((vF - 8.0f) / 10.0f, 0.0f, 1.0f);
        float amount = press * cut * fast;
        if (amount > 0.0f) {
            float g = kGripAsphalt + (kGripBrakeTurn - kGripAsphalt) * amount;
            gripTarget = std::min(gripTarget, g);
        }
    }

    // サイドブレーキが一番強い。後輪ロックで横グリップがほぼ無くなる。
    // ステアと組み合わせると最強で、後輪はほぼフリーになって即座に流れる。
    if (input.handbrake && vF > 4.0f) {
        gripTarget = std::min(gripTarget, kGripHandbrake);
        if (std::fabs(input.steer) > 0.3f) {
            gripTarget = std::min(gripTarget, kGripCombo);
        }
    }
    if (!onAsphalt) {
        gripTarget = std::min(gripTarget, kGripGrass);
    }

    // ドリフト状態の間は、原因によらず横グリップを低く保ち続ける。
    // 「ドリフト状態＝滑りながら進む」「グリップ状態＝向いた方向に進む」の
    // 二状態をはっきり分けるのが狙い。抜けるまでは滑り続ける。
    if (drifting) {
        gripTarget = std::min(gripTarget, kGripDrift);
    }

    // ドリフト終了直後は、横グリップの上限をドリフト側から通常へ
    // 約1.4秒かけて滑らかに戻す（クロスフェード、序盤は緩やかな
    // イーズイン）。状態が切り替わった
    // 瞬間にグリップが跳ね上がると、残っていた横滑りが一気に消えて
    // 進行方向が「ガクッ」と跳ぶため。特に、失速して速度の下限で
    // 終了したときは滑りが深いまま残っているので効きが大きい。
    if (!drifting && gripRestore > 0.0f) {
        float fade = 1.0f - gripRestore;
        float cap = kGripDrift + (kGripAsphalt - kGripDrift) * fade * fade;
        if (gripTarget > cap) gripTarget = cap;
        gripRestore = std::max(0.0f, gripRestore - dt / 1.4f);
    }

    // グリップは落とすとき素早く、戻すとき少し遅く。戻しを即座にすると
    // 姿勢が跳ね返る（スナップオーバーステア）ので非対称にする。
    const float gripRate = (gripTarget < gripLat) ? 16.0f : 5.0f;
    gripLat = approachExp(gripLat, gripTarget, gripRate, dt);

    // --- ヨー --------------------------------------------------------------
    float yawTarget = 0;
    if (std::fabs(vF) > 0.05f) {
        yawTarget = (vF / kWheelbase) * std::tan(steerAngle);
    }
    float slow = clampf(1.0f - std::fabs(vF) / 8.0f, 0.0f, 1.0f);
    yawTarget += steerAngle * 0.9f * slow;

    // サイドブレーキ中は切った方向へ車体が回り込むのを助ける。
    // 自転車モデルだけだと突入が鈍いため。ステアと組み合わせたときは
    // 特に強く効かせて、即ドリフトに入れる。
    if (input.handbrake && vF > 4.0f) {
        float assist = std::fabs(input.steer) > 0.3f ? 1.45f : 0.75f;
        yawTarget += clampf(input.steer, -1.0f, 1.0f) * assist *
                     clampf(vF / 25.0f, 0.0f, 1.0f);
    }
    // 姿勢ホールド：ドリフト中にステアを離し、アクセルを踏んでいる間は
    // 「いまのドリフト姿勢を保つ」モードに入る（postureHold が 0→1 に
    // 滑らかに寄る）。中立だとヨーがゼロへ落ちて姿勢がほどけ、まっすぐ
    // 滑るだけだったのを直すための仕組みで、3つの働きをまとめて行う：
    //   1. 滑りに応じた自己ヨーで車が回り続ける（手を離しても曲がる）
    //   2. ヨーで回った分だけ速度ベクトルも一緒に回す（角度がほどけない）
    //   3. 滑りを消す向きの減衰（横グリップと向き寄せ）を弱める
    // ステアを入れるかアクセルを離せば即座に通常へ戻る。
    {
        bool holdActive = drifting && std::fabs(input.steer) < 0.08f && input.throttle > 0.35f;
        // ホールドは「保つべき姿勢（滑り）」があるときだけ効かせる。滑りが
        // 小さいのに効かせると、減衰を止める作用のせいで小さな残留スリップが
        // 消えず、まっすぐ走っているのにドリフト状態が解けない。滑り
        // 0.10 rad 以下では無効、0.25 rad 以上でフルに効く。
        float holdTarget = holdActive
            ? clampf((std::fabs(slipNow) - 0.10f) / 0.15f, 0.0f, 1.0f) : 0.0f;
        postureHold = approachExp(postureHold, holdTarget, 6.0f, dt);
    }
    if (drifting && postureHold > 0.0f) {
        yawTarget += -0.50f * std::tanh(slipNow / 0.40f) * postureHold;
    }
    // ドリフト中はステアリングで車の向きを大きく傾けられるよう、回頭の上限も広げる。
    // ただし上限は速度で絞る：切れ角（×3 倍）自体が低速ほど大きいので、一定の
    // 上限だと中高速の進入で鼻が暴れてスピンに近くなる。低速の振り回しは
    // そのままに、速いほど穏やかに頭を振る。
    const float driftYawLimit = clampf(3.5f - 0.045f * std::fabs(vF), 1.45f, 3.05f);
    const float yawLimit = drifting ? driftYawLimit : 1.9f;
    yawTarget = clampf(yawTarget, -yawLimit, yawLimit);

    // サイドブレーキ中は回頭の追従も速くする（切ったらすぐ回り込む）。
    const float yawResponse =
        input.handbrake ? kYawResponseGrip : (drifting ? kYawResponseDrift : kYawResponseGrip);
    yawRate = approachExp(yawRate, yawTarget, yawResponse, dt);
    if (speedNow < 0.05f && std::fabs(steerAngle) < 0.01f) {
        yawRate = 0;
    }
    yaw += yawRate * dt;
    yaw = wrapPi(yaw);

    // --- 速度ベクトルを新しい車体向きで組み直す ----------------------------
    // ヨーが進んだ分だけ速度ベクトルは車体に対して遅れる（＝スリップ角が付く）。
    // そこへ横グリップによる減衰と、滑っている間のタイヤの削れ（減速）を入れる。
    const float vFPre = vF, vLPre = vL;  // ヨー更新前の分解（姿勢ホールド用）
    const float nfx = std::sin(yaw), nfz = std::cos(yaw);
    const float nrx = std::cos(yaw), nrz = -std::sin(yaw);
    vF = velX * nfx + velZ * nfz;
    vL = velX * nrx + velZ * nrz;
    if (postureHold > 0.0f) {
        // ホールド中は、ヨーで回った分だけ速度ベクトルも一緒に回っていた
        // ことにする：ヨーによるスリップ角の増減を打ち消し、姿勢を保つ。
        vF += (vFPre - vF) * postureHold;
        vL += (vLPre - vL) * postureHold;
    }

    vL *= std::exp(-gripLat * (1.0f - 0.93f * postureHold) * dt);

    // ドリフト中、アクセルを踏んでいる間は速度ベクトルの向きを車体の
    // 向きへ少しずつ寄せる。横滑りを「消してしまう」のではなく進む向きへ
    // 振り替えるので、滑りながらも車の向きに進んでいくし、速さの大きさも
    // 落ちない（パワーが向いている方向へ引っ張る、という表現）。
    // 踏んでいないときは寄せないので、突入直後の滑り出しは妨げない。
    if (drifting && vF > 1.0f && input.throttle > 0.15f) {
        float mag = std::sqrt(vF * vF + vL * vL);
        float ang = std::atan2(vL, vF);
        ang *= std::exp(-1.40f * (1.0f - 0.95f * postureHold) * dt);
        vF = mag * std::cos(ang);
        vL = mag * std::sin(ang);
    }

    // 横滑りは速度を削る。ただしドリフトが「決まっている」間は削れを弱める。
    // ここが強いと、滑らせるたびに失速して爽快感が出ない。ゲームとしては
    // アクセルで姿勢を保ちながら速度も乗せ続けられるバランスにする。
    const float slipAbs = std::fabs(std::atan2(vL, std::fabs(vF) + 0.001f));
    if (speedNow > 1.0f) {
        float scrubRate = drifting ? 0.07f : (0.55f - 0.48f * gripRestore);
        float scrubGain = drifting ? 0.03f : (0.35f - 0.32f * gripRestore);
        const float scrub = 1.0f - std::exp(-scrubRate * slipAbs * dt * 4.0f);
        vF -= vF * scrub * scrubGain;
    }
    // 芝の抵抗は update() 側の縦の力で既に入っているので、ここでは足さない。
    // 二重にかけると芝に少しはみ出しただけで失速しすぎる。

    velX = nfx * vF + nrx * vL;
    velZ = nfz * vF + nrz * vL;
    velForward = vF;
    velLateral = vL;
    speed = vF;

    slipAngle = (speedNow > 1.0f) ? std::atan2(vL, std::fabs(vF) + 0.001f) : 0.0f;

    // --- ドリフト終了とスコア ------------------------------------------------
    // グリップ状態へ戻るのは「回復ゲージ」(driftRecovery) が満ちてから。
    //   ・何もしない：ゆっくり溜まるだけ（ドリフト状態はしばらく維持される）
    //   ・アクセルを踏んでいる間：基本レートは止まる。踏む＝ドリフトを
    //     続ける意思、として扱う（踏んでいるのに勝手に戻らない）
    //   ・カウンターステア：速く溜まる（立て直し操作で戻るのが本筋）
    //     ただし深い滑りの最中のカウンターは姿勢のバランス取りなので溜めない
    //   ・ドリフト方向へ切り続ける：ゲージが戻り、ドリフトが続く
    // ゲージが満ちても、滑りが収まっていなければ（kDriftExitSlip 以下）
    // まだ戻らない。ほぼ停止したら即終了。また、深い滑りの最中は
    // ゲージが満ち切らない（上限 0.6）：アクセルで滑りが一瞬浅くなった
    // だけで「満ちていたゲージ＋滑り条件」で即終了する事故を防ぐ。
    // 満ち切れるのは、滑りが浅くなって立て直しに入ってからだけ。
    // 例外：滑り・ヨーとも小さい状態が少し続けば「明らかにグリップして
    // いる」として、ゲージを待たずに即終了する。真っすぐ走っているのに
    // ドリフト判定が残るのを防ぐため（切り返しの途中はヨーが大きい
    // ので、ここでは終了しない）。
    if (drifting) {
        if (std::fabs(slipAngle) < 0.07f && std::fabs(yawRate) < 0.30f) {
            settledHold += dt;
            if (settledHold > 0.35f) {
                drifting = false;
                driftRecovery = 0.0f;
                settledHold = 0.0f;
                gripRestore = 1.0f;  // 物理量はクロスフェードで戻す（下記）
            }
        } else {
            settledHold = 0.0f;
        }
    }
    if (drifting) {
        float rate = (input.throttle > 0.35f) ? 0.0f : kDriftRecoverBase;
        driftCounterAge += dt;
        if (std::fabs(slipAngle) > 0.03f && std::fabs(input.steer) > 0.05f) {
            bool counter = (input.steer > 0.0f) == (slipAngle > 0.0f);
            float amount = clampf(std::fabs(input.steer), 0.0f, 1.0f);
            if (counter && std::fabs(slipAngle) > 0.05f) {
                driftCounterAge = 0.0f;  // いま立て直し操作をしている
            }
            if (counter) {
                // 深い滑りの最中はバランス取りの側面が強いので上乗せは控えめに、
                // 浅くなってきたら一気に溜まる（立て直しのクライマックス）。
                // 浅い zone では、滑りに合わせて切り量を絞っても「立て直し操作の
                // 継続」として最低限の進みを保証する（終わり際が間延びしない）。
                float credit = amount;
                if (std::fabs(slipAngle) < kDriftDeepSlip && credit < 0.45f) credit = 0.45f;
                rate += (std::fabs(slipAngle) < kDriftDeepSlip ? kDriftRecoverCounter
                                                               : kDriftRecoverDeep) *
                        credit;
            } else {
                rate -= 1.1f * amount;
            }
        }
        // カウンターを当てている最中／当てた直後で、実際に車が収まっている
        // （滑り小さくヨーも calm）なら、仕上げとして速く溜まる。ステアリングを
        // 離して時間が経つと仕上げは切れ、ゆっくりした基本レートだけに戻る。
        if (driftCounterAge < 0.6f && std::fabs(slipAngle) < 0.10f &&
            std::fabs(yawRate) < 0.4f) {
            rate += 0.9f;
        }
        driftRecovery += rate * dt;
        if (driftRecovery < 0.0f) driftRecovery = 0.0f;
        if (std::fabs(slipAngle) > kDriftDeepSlip && driftRecovery > 0.6f) {
            driftRecovery = 0.6f;  // 深い滑りの最中は満ち切らない
        }
        if ((driftRecovery >= 1.0f && std::fabs(slipAngle) < kDriftExitSlip) ||
            speedNow < 2.5f) {
            drifting = false;
            driftRecovery = 0.0f;
            gripRestore = 1.0f;  // 物理量はクロスフェードで戻す（下記）
        }
    }
    if (drifting) {
        driftTime += dt;
        driftScore += speedNow * std::fabs(slipAngle) * dt * 10.0f;
    } else {
        driftTime = 0;
    }

    // --- 位置 ---------------------------------------------------------------
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

    // ドリフト中は切れ角を大きく広げ、戻しも速くする。ステアリングで
    // 車の向きをぐっと傾けたり、カウンターで立て直したりするための幅。
    // 終了直後は gripRestore の分だけドリフト側の値を残し、切れ角や
    // 駆動ブーストが一段で切り替わって「ガクッ」とならないようにする。
    const float restoreMix = drifting ? 1.0f : gripRestore;
    float steerLimit = maxSteerAt(speed) * (1.0f + 2.9f * restoreMix);
    float target = clampf(input.steer, -1.0f, 1.0f) * steerLimit;
    float step = (restoreMix > 0.0f ? kSteerRateDrift : kSteerRate) * dt;
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
        // サイドブレーキ中はエンジンが効かず、後輪ロックで穏やかに減速する。
        // 通常ブレーキほどは止まらないので、滑りながら速度を保てる。
        // 突入時はしっかり落とし、いったんドリフトに入ったら減速を弱めて
        // 「引きっぱなしでも失速しすぎない」ようにする。
        force -= kHandbrakeDecel - (kHandbrakeDecel - 2.2f) * restoreMix;
    } else if (input.brake > 0.05f && !reversing) {
        force -= input.brake * kBrake * (speed >= 0 ? 1.0f : -1.0f);
    } else if (reversing) {
        force -= input.brake * kReverseAccel;
    } else if (input.throttle > 0.0f && speed > -0.4f) {
        float ratio = clampf(speed / kMaxSpeed, 0.0f, 1.0f);
        // ドリフト中は駆動力を大きく上乗せする。車が向いている方向へ
        // ぐっと引っ張る力を強めにして、横を向いたままでもスピードが
        // 伸びていく感覚にする。深い角度では駆動力の速度方向への効きが
        // cos(角度) の分だけ落ちるので、深さに応じてさらに上乗せして
        // 打ち消す（深くても速い、を両立させる）。
        float boost = 1.0f;
        if (restoreMix > 0.0f) {
            float depth = clampf((std::fabs(slipAngle) - 0.35f) / 0.65f, 0.0f, 1.0f);
            boost = 1.0f + (2.15f + 1.35f * depth - 1.0f) * restoreMix;
        }
        force += input.throttle * kAccel * (1.0f - ratio * ratio) * boost;
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
    speed = clampf(speed, -kMaxReverse, kMaxSpeed);

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
        // 壁の外へ向いている分だけ速度を落とす。コースに沿っているときは壁を擦るだけ。
        float facingOut = sign * (std::sin(yaw) * ground.rightX + std::cos(yaw) * ground.rightZ);
        if (facingOut > 0.15f) {
            float cap = 2.0f + 8.0f * (1.0f - facingOut);
            if (speed > cap) {
                const float fx = std::sin(yaw), fz = std::cos(yaw);
                velX += fx * (cap - speed);
                velZ += fz * (cap - speed);
                speed = cap;
            }
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
    const Vec3 red{0.72f, 0.08f, 0.09f};
    const Vec3 dark{0.06f, 0.06f, 0.07f};
    const Vec3 carbon{0.12f, 0.12f, 0.13f};
    mesh.addBox({-0.88f, 0.32f, -2.05f}, {0.88f, 0.62f, 2.05f}, red);
    mesh.addBox({-0.72f, 0.62f, -0.55f}, {0.72f, 1.05f, 1.05f}, red);
    mesh.addBox({-0.62f, 0.78f, -0.35f}, {0.62f, 1.02f, 0.9f}, dark);
    mesh.addBox({-0.7f, 0.32f, 1.7f}, {0.7f, 0.5f, 2.25f}, red);
    mesh.addBox({-0.95f, 0.26f, 1.95f}, {0.95f, 0.36f, 2.35f}, carbon);
    mesh.addBox({-0.86f, 0.85f, -2.2f}, {0.86f, 1.0f, -1.85f}, carbon);
    mesh.addBox({-0.15f, 0.95f, -2.05f}, {0.15f, 1.15f, -1.9f}, carbon);
}

void buildCarWheel(CpuMesh& mesh) {
    const int seg = 14;
    const float radius = 0.34f;
    const float halfW = 0.15f;
    const Vec3 tire{0.07f, 0.07f, 0.08f};
    const Vec3 rim{0.72f, 0.74f, 0.76f};
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
    mesh.addQuad({-1.05f, 0, -2.1f}, {1.05f, 0, -2.1f}, {1.05f, 0, 2.15f}, {-1.05f, 0, 2.15f}, {0, 1, 0}, ink);
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
