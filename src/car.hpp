#pragma once

#include "meshbuild.hpp"
#include "track.hpp"

struct DriveInput {
    float throttle = 0;      // 0..1
    float brake = 0;         // 0..1  停止中は後退
    float steer = 0;         // -1..1  左が負、右が正
    bool handbrake = false;  // サイドブレーキ。後輪のグリップを落として滑らせる
};

inline bool brakeLightsActive(const DriveInput& input, float forwardSpeed) {
    return input.handbrake || (input.brake > 0.05f && forwardSpeed > -0.5f);
}

inline bool reverseLightsActive(float forwardSpeed, float brakeHold) {
    return forwardSpeed < -0.15f || brakeHold > 0.28f;
}

struct Car {
    float x = 0;
    float y = 0;
    float z = 0;
    float yaw = 0;
    float speed = 0;        // m/s。前方成分。正が前進（velForward と常に一致）
    float steerHold = 0;    // Strong steering hold duration for gradual slide initiation
    float steerInput = 0;   // Smoothed steering command, -1..1
    float steerAngle = 0;   // 前輪の切れ角（ラジアン、右が正）
    float wheelSpin = 0;
    float brakeHold = 0;    // 止まってから後退に入るまでの待ち時間
    bool onAsphalt = true;
    bool wallContact = false;  // Current physics step is touching the guardrail.
    bool wallNewContact = false; // First step of a contact, for the initial spark burst.
    float wallImpact = 0;      // Inward impact speed (m/s).
    float wallAlongSpeed = 0;  // Tangential speed before wall friction (m/s).
    float wallContactX = 0, wallContactZ = 0; // Contact point on the rail.
    float wallRightX = 1, wallRightZ = 0; // Road normal at the contact.
    float wallSide = 0;        // -1 left, +1 right relative to the track.

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
    float slideEntryTime = 0; // Seconds since a meaningful slide began
    float brakeDriftMix = 0; // Preserves brake-initiated behavior after release
    float driftAmount = 0;  // Continuous physical slip blend, 0..1
    float suspension[4] = {};     // Visual spring displacement: front left/right, rear left/right (m).
    float suspensionVel[4] = {};  // Spring velocity (m/s).
    float bodyHeave = 0; // Vertical body travel in metres.
    float bodyRoll = 0;     // 車体の視覚ロール (rad)。サスペンションの左右差から算出。
    float bodyRollVel = 0;  // ロールの角速度。バネ・ダンパーで重心の揺れを出す
    float bodyPitch = 0;    // 車体の視覚ピッチ (rad)。加減速の荷重移動。正＝加速でノーズが上がる
    float longG = 0;        // 平滑化した前後G (m/s^2)。ピッチの駆動に使う
    float prevVF = 0;       // 前ステップの前進速度（前後Gの算出用）

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
void buildCarHeadlights(CpuMesh& mesh);
void buildCarReverseLights(CpuMesh& mesh);
void buildCarWheel(CpuMesh& mesh);
void buildCarShadow(CpuMesh& mesh);
Mat4 carModelMatrix(const Car& car, float pitch, float roll);
// 車体ローカル座標の点を、carModelMatrix と同じ姿勢で世界座標へ変換する
// （ブレーキランプ位置の取得などに使う）。
Vec3 carPointWorld(const Car& car, float pitch, float roll, Vec3 local);
