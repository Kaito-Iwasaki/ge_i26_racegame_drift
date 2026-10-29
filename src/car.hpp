#pragma once

#include "meshbuild.hpp"
#include "track.hpp"

struct DriveInput {
    float throttle = 0;      // 0..1
    float brake = 0;         // 0..1  停止中は後退
    float steer = 0;         // -1..1  左が負、右が正
    bool handbrake = false;  // サイドブレーキ。後輪のグリップを落として滑らせる
};

struct Car {
    float x = 0;
    float y = 0;
    float z = 0;
    float yaw = 0;
    float speed = 0;        // m/s。前方成分。正が前進（velForward と常に一致）
    float steerAngle = 0;   // 前輪の切れ角（ラジアン、右が正）
    float wheelSpin = 0;
    float brakeHold = 0;    // 止まってから後退に入るまでの待ち時間
    bool onAsphalt = true;
    TrackPose ground;

    // --- ドリフト用の状態 ---------------------------------------------------
    // 速度はスカラーではなく世界座標のベクトルで持つ。車体の向き（yaw）と
    // 進行方向がずれることで横滑りが生まれる。
    float velX = 0;           // 世界座標の速度 x 成分 (m/s)
    float velZ = 0;           // 世界座標の速度 z 成分 (m/s)
    float velForward = 0;     // 車体座標の前方成分（speed と同値）
    float velLateral = 0;     // 車体座標の横成分。右が正。これが横滑りの量
    float yawRate = 0;        // ヨーレート (rad/s)。慣性を持たせて滑らかに追従
    float slipAngle = 0;      // 進行方向と車体の向きのずれ (rad)。速度が車体より右を向くと正
    bool drifting = false;    // スリップ角と速度から判定（ヒステリシス付き）
    float driftTime = 0;      // 現在継続中のドリフト時間 (s)
    float driftScore = 0;     // 累計スコア。速く・大きく滑るほど増える
    float gripLat = 9.0f;     // 現在の横グリップ係数 (1/s)。目標値へ滑らかに変化する
    float powerSlideHold = 0; // パワースライド条件が続いた時間（誤発防止）
    float steerSlideHold = 0; // ステアだけで滑らせる条件が続いた時間（誤発防止）
    float driftRecovery = 0;  // ドリフトからグリップへ戻る進み具合 0..1（カウンターで速く溜まる）
    float driftCounterAge = 10.0f;  // 最後にカウンターを当ててからの時間（立て直しの新鮮さ）
    float slipEntryHold = 0;  // スリップ角が突入しきい値を超え続けている時間（瞬間の跳ねでは入らない）
    float postureHold = 0;  // 無ステア＋アクセル時の姿勢ホールドの強さ 0..1（滑らかに出入りする）
    float settledHold = 0;  // 滑り・ヨーとも小さい状態が続いている時間（明らかにグリップしている判定）
    float gripRestore = 0;  // ドリフト終了直後の物理量クロスフェード 1→0（約1.4秒で通常へ戻す）

    void reset(const Track& track);
    void update(float dt, const DriveInput& input, const Track& track);

    // 地面に対する速さの大きさ |v| (m/s)。HUD 表示用。
    float groundSpeed() const;

    // 車体運動を 1 ステップ進める。グリップ走行とドリフトをここで一括に扱う。
    void stepPhysics(float dt, const DriveInput& input);
};

void buildCarBody(CpuMesh& mesh);
// ブレーキ時だけ重ねて描く明るい赤のテールランプ（車体メッシュのテールより少し外側）。
void buildCarBrakeLights(CpuMesh& mesh);
void buildCarWheel(CpuMesh& mesh);
void buildCarShadow(CpuMesh& mesh);
Mat4 carModelMatrix(const Car& car, float pitch, float roll);
