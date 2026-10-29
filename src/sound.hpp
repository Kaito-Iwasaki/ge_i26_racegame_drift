#pragma once

// ゲーム内サウンド。基本は録音サンプル（assets/sound/ の WAV）をループ再生し、
// 読み込めなかった場合だけリアルタイム合成にフォールバックする。
//
//   エンジン音 : 録音を 2 層（アイドル／全開）クロスフェード＋再生速度でピッチ追従。
//                サンプルが無い場合は倍音＋点火パルスの合成音。
//   タイヤ／ブレーキ音: ドリフト音の録音ループ。ドリフト・サイド・強ブレーキで鳴る。
//                サンプルが無い場合は帯域ノイズ＋金属音の合成。
//   BGM          : assets/sound/bgm.ogg をデコードしたものをシームレスに
//                ループ再生する（sound.cpp が stb_vorbis で読み込む）。
//                ポーズ中はフェードアウトして再生位置も止まる。
//
// このヘッダは SDL に依存しない純粋な DSP とパラメータ変換だけを持つ
// （単体テストがそのままコンパイルできる）。SDL のデバイス出力と WAV の
// 読み込みは sound.cpp 側。

#include <cmath>
#include <cstddef>
#include <vector>

namespace sounddetail {
inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
// 係数 rate (1/s) で指数的に近づけるための 1 サンプル分係数。
inline float smoothCoeff(float rate, float sampleRate) {
    return 1.0f - std::exp(-rate / sampleRate);
}
}  // namespace sounddetail

// 音源へ毎フレーム渡すパラメータ。すべて 0..1 に正規化済み。
struct SoundState {
    bool running = false;  // エンジンがかかっているか
    float rpm01 = 0;       // 回転数。0 がアイドル付近、1 がレッドゾーン
    float load = 0;        // アクセル開度。音量と音の荒さに効く
    float tire = 0;        // タイヤ音（ブレーキ／ドリフト）の強さ
    float tirePitch01 = 0; // タイヤ音の高さ。速度で変わる
    float squeal = 0;      // ブレーキ鳴き（金属音）の強さ
    float bgm = 0;         // BGM の目標音量 0..1（0 でフェードアウト＋停止）
};

// 速度から「ギアつきの回転数」を作る。ギアが上がるたびに回転数が落ちる、
// あの山型の動きがないとエンジン音に聞こえない。
inline float engineRpm01(float speedAbs, float throttle) {
    using namespace sounddetail;
    if (speedAbs < 0.6f) {
        // 停止中はアイドル。アクセルで空ぶかしできる。
        return clamp01(0.16f + throttle * 0.30f);
    }
    // 各ギアの上限速度 (m/s)。6 速。最終段は車両側の最高速に合わせる。
    const float top[6] = {13.5f, 25.0f, 38.0f, 52.0f, 66.0f, 88.0f};
    float lo = 0.0f;
    for (int gear = 0; gear < 6; ++gear) {
        if (speedAbs <= top[gear] || gear == 5) {
            float local = clamp01((speedAbs - lo) / (top[gear] - lo));
            return 0.30f + 0.70f * local;
        }
        lo = top[gear];
    }
    return 1.0f;
}

// エンジンの「ギアと回転数」の状態を持つ版。
// engineRpm01 のような速度への単純な写像だと、ギア境界の前後で回転数が
// 一瞬で跳ねるし、境界付近で速度が少し揺れるだけでピッチが行ったり来たりする
// （「変な位置でリピートする」聞こえ方の原因）。
// 実車と同じくギアを状態として持ち：
//   ・シフトアップは上限を超えたら行うが、回転数の変化は滑らかに追わせる
//   ・シフトダウンは速度が十分に落ちるまで遅らせる（ヒステリシス）
//   ・ドリフト中は駆動輪が空転するので回転が跳ね上がる（ホイールスピン）
struct EngineVoice {
    int gear = 0;         // 0 = 1速
    float rpm01 = 0.16f;  // 現在の回転数 0..1

    // dt 秒ぶん進めて、現在の回転数を返す。毎フレーム呼ぶ。
    float update(float speedAbs, float throttle, bool drifting, bool paused, float dt) {
        using namespace sounddetail;
        const float top[6] = {13.5f, 25.0f, 38.0f, 52.0f, 66.0f, 88.0f};
        if (dt <= 0.0f) return rpm01;

        float target;
        if (paused) {
            target = 0.16f;  // ポーズ中はアイドル
        } else {
            // シフトアップ：上限を超えたら上げる。
            while (gear < 5 && speedAbs > top[gear]) ++gear;
            // シフトダウン：下のギアの上限の 68% を切るまで落とさない。
            // この幅が境界付近でのギアの往復（ハンチング）を止める。
            while (gear > 0 && speedAbs < top[gear - 1] * 0.68f) --gear;

            if (speedAbs < 0.6f && gear == 0) {
                target = clamp01(0.16f + throttle * 0.30f);  // 停止中は空ぶかし
            } else {
                float lo = gear == 0 ? 0.0f : top[gear - 1];
                float local = clamp01((speedAbs - lo) / (top[gear] - lo));
                target = 0.30f + 0.70f * local;
                // ドリフト中は後輪が空転して回転が跳ねる。
                if (drifting && throttle > 0.3f) {
                    target = std::fmax(target, clamp01(0.55f + 0.35f * throttle));
                }
            }
        }
        // 回転数の変化は一気ではなく滑らかに。上げる方向は速く（吹け上がり）、
        // 下げる方向は少しゆっくり（シフトアップ時の落ちに余韻を持たせる）。
        float rate = (target > rpm01) ? 7.0f : 5.0f;
        rpm01 += (target - rpm01) * (1.0f - std::exp(-rate * dt));
        return rpm01;
    }
};

// 車両の状態 → 音パラメータ。main 側はこれを呼んで渡すだけ。
// rpm01 はここでも埋めるが、main 側は EngineVoice の値で上書きする
// （ギアの状態を持てるのは EngineVoice だけなので）。
inline SoundState makeSoundState(float speed, float groundSpeed, float throttle, float brake,
                                 bool handbrake, bool drifting, float slipAbs, bool paused) {
    using namespace sounddetail;
    SoundState state;
    state.running = true;
    if (paused) {
        state.rpm01 = 0.16f;
        return state;
    }
    state.rpm01 = engineRpm01(std::fabs(speed), throttle);
    state.load = clamp01(throttle);
    state.bgm = 1.0f;  // 走行中は BGM を流す（ポーズ時は上の早期リターンで 0）

    // タイヤ音：ブレーキ、サイドブレーキ、ドリフトのうち一番強いものを採用。
    float tire = 0.0f;
    if (brake > 0.05f && groundSpeed > 4.0f) {
        tire = std::fmax(tire, brake * clamp01(0.30f + groundSpeed / 30.0f));
    }
    if (handbrake && groundSpeed > 4.0f) {
        tire = std::fmax(tire, 0.75f);
    }
    if (drifting) {
        tire = std::fmax(tire, clamp01(slipAbs * 1.6f) * clamp01(0.25f + groundSpeed / 25.0f));
    }
    state.tire = clamp01(tire);
    state.tirePitch01 = clamp01(groundSpeed / 45.0f);

    // ブレーキ鳴きは「速いのに強く踏んでいる」ときに、はっきり鳴らす。
    // 踏力が増すほど・速いほど強くする。
    if (brake > 0.55f && groundSpeed > 12.0f) {
        float press = clamp01((brake - 0.55f) / 0.45f);
        state.squeal = clamp01((groundSpeed - 12.0f) / 45.0f) * press;
    }
    return state;
}

// ループ再生用サンプル（モノラル float、デバイスのサンプルレートに変換済み）。
struct SampleLoop {
    std::vector<float> data;

    bool empty() const { return data.empty(); }

    // 位置 pos（サンプル単位）で線形補間して読む。範囲外は循環させる。
    float read(double pos) const {
        if (data.empty()) {
            return 0.0f;
        }
        double n = double(data.size());
        double p = std::fmod(pos, n);
        if (p < 0.0) p += n;
        std::size_t i0 = std::size_t(p);
        float frac = float(p - double(i0));
        float a = data[i0];
        float b = data[(i0 + 1) % data.size()];
        return a + (b - a) * frac;
    }
};

// サンプル 1 本分の再生ヘッド。位置とゲイン（平滑化済み）を持つ。
struct SampleVoice {
    double pos = 0;
    float gainSm = 0;

    // 1 サンプル取り出す。rate は再生速度（1.0 で等速、ピッチもその分だけ動く）。
    float next(const SampleLoop& s, float targetGain, double rate, float gainCoeff) {
        if (s.empty()) {
            return 0.0f;
        }
        gainSm += (targetGain - gainSm) * gainCoeff;
        float v = s.read(pos) * gainSm;
        pos += rate;
        double n = double(s.data.size());
        if (pos >= n) pos -= n;  // rate は n よりずっと小さい前提
        return v;
    }
};

// サンプル生成器。オーディオスレッドから render() を呼ぶ。
// パラメータは内部で滑らかに追従させるので、フレーム単位で雑に渡してよい。
struct SoundDsp {
    float sampleRate = 44100.0f;

    // 位相とフィルタの状態（オーディオスレッド内でのみ触る）
    float phase = 0;       // エンジン基本波
    float phaseDet = 0;    // わずかにデチューンした相方（うねりの元）
    float pulseEnv = 0;    // 点火パルスの包絡線（1 周期ごとに立つ）
    float jitter = 1.0f;   // 周期ごとのピッチのムラ
    float exhaustLp = 0;   // 排気ノイズ用ローパス
    float squealPhase = 0; // ブレーキ鳴き
    float squealVib = 0;   // ブレーキ鳴きのビブラート位相
    float lpState = 0;     // エンジン用ローパス
    float svfLow = 0;      // タイヤ音用バンドパス (state variable filter)
    float svfBand = 0;
    unsigned noiseSeed = 22222222u;

    // 平滑化済みパラメータ
    float runSm = 0;
    float rpmSm = 0;
    float loadSm = 0;
    float tireSm = 0;
    float squealSm = 0;

    // 録音サンプル（sound.cpp が読み込む。空なら合成にフォールバック）。
    SampleLoop engineIdleSample;
    SampleLoop engineMainSample;
    SampleLoop driftSample;
    SampleVoice engineIdleVoice;
    SampleVoice engineMainVoice;
    SampleVoice driftVoice;

    // BGM（sound.cpp が bgm.ogg をデコードして入れる。モノラル化済み）。
    SampleLoop bgmSample;
    SampleVoice bgmVoice;

    float nextNoise() {
        noiseSeed = noiseSeed * 1664525u + 1013904223u;
        return float((noiseSeed >> 9) & 0x7FFFu) / 16384.0f - 1.0f;
    }

    // out に n サンプルのモノラル float (-1..1) を書く。
    void render(float* out, int n, const SoundState& target) {
        using namespace sounddetail;
        const float pi2 = 6.2831853f;
        const float cRun = smoothCoeff(6.0f, sampleRate);
        const float cRpm = smoothCoeff(14.0f, sampleRate);
        const float cLoad = smoothCoeff(18.0f, sampleRate);
        const float cTireOn = smoothCoeff(30.0f, sampleRate);   // 鳴り始めは速く
        const float cTireOff = smoothCoeff(9.0f, sampleRate);   // 止まり際は少し残す
        const float cSquealOn = smoothCoeff(40.0f, sampleRate);  // 鳴き始めは「キッ」と速く
        const float cSquealOff = smoothCoeff(14.0f, sampleRate);
        const float cSample = smoothCoeff(22.0f, sampleRate);  // サンプル再生のゲイン追従

        for (int i = 0; i < n; ++i) {
            runSm += ((target.running ? 1.0f : 0.0f) - runSm) * cRun;
            rpmSm += (target.rpm01 - rpmSm) * cRpm;
            loadSm += (target.load - loadSm) * cLoad;
            tireSm += (target.tire - tireSm) * (target.tire > tireSm ? cTireOn : cTireOff);
            squealSm += (target.squeal - squealSm) *
                        (target.squeal > squealSm ? cSquealOn : cSquealOff);

            // --- エンジン ---------------------------------------------------
            float engine = 0.0f;
            if (!engineMainSample.empty()) {
                // 録音サンプル版：再生速度でピッチを追い、低回転ではアイドル録音の
                // 層へクロスフェードする（吹かし切った録音を遅回しする違和感の回避）。
                double rate = 0.80 + 0.65 * double(rpmSm);
                float idleMix = clamp01((0.50f - rpmSm) / 0.28f);
                float engGain = (0.40f + 0.36f * loadSm + 0.12f * rpmSm) * runSm;
                float gMain = engGain * (1.0f - 0.85f * idleMix);
                float gIdle = engGain * idleMix * 1.7f;
                engine = engineMainVoice.next(engineMainSample, gMain, rate, cSample) +
                         engineIdleVoice.next(engineIdleSample, gIdle, rate, cSample);
            } else {
            // 基本周波数 45..235 Hz。2乗カーブで低回転の伸びをゆっくりに。
            // 荒さの作り方（滑らかなサイン波の重ね合わせだけだと電気モーターになる）：
            //   ・1 周期 = 1 回の爆発とみなし、周期の頭で音量を持ち上げる（点火パルス）。
            //     アイドルでは「バラ、バラ」と 1 発ずつ聞こえる。
            //   ・パルスの強さとピッチに毎回ムラを付ける（機械の不揃い感）。
            //   ・わずかにデチューンした相方を重ねてうなりを作る。
            //   ・波形をソフトクリップで歪ませて倍音を増やす（唸り）。
            //   ・パルスに同期した排気ノイズ「バフッ」を重ねる。
            float f0 = 45.0f + 190.0f * rpmSm * rpmSm;
            phase += (f0 * jitter) / sampleRate;
            if (phase >= 1.0f) {
                phase -= 1.0f;
                float r = nextNoise() * 0.5f + 0.5f;  // 0..1
                pulseEnv = 0.82f + 0.18f * r;         // 次の 1 発の強さ
                jitter = 1.0f + 0.018f * nextNoise(); // 次の 1 発のタイミングのムラ
            }
            phaseDet += (f0 * 1.004f) / sampleRate;
            if (phaseDet >= 1.0f) phaseDet -= 1.0f;
            // パルス包絡は周期の 1/3 ほどで減衰（減衰率を周期に比例させる）。
            pulseEnv *= std::exp(-3.0f * f0 / sampleRate);
            float bright = 0.45f + 0.55f * loadSm;
            float raw = std::sin(pi2 * phase) * 0.60f +
                        std::sin(pi2 * phaseDet) * 0.22f +
                        std::sin(pi2 * phase * 2.0f) * 0.30f * bright +
                        std::sin(pi2 * phase * 3.0f) * 0.14f * bright +
                        std::sin(pi2 * phase * 0.5f) * 0.36f;
            // 点火パルスで振幅変調。低回転ほど深くして 1 発ずつ目立たせる。
            float amDepth = lerp(0.62f, 0.34f, rpmSm);
            raw *= (1.0f - amDepth) + amDepth * pulseEnv;
            // ソフトクリップで歪ませる。アクセルを開けるほど強く。
            float drive = 0.8f + 2.4f * loadSm;
            raw = raw * (1.0f + drive) / (1.0f + drive * std::fabs(raw));
            // ローパスで角を落とす。開度と回転で少し開く。
            float cutoff = 320.0f + 2600.0f * (0.35f * rpmSm + 0.65f * loadSm);
            float lpCoeff = 1.0f - std::exp(-pi2 * cutoff / sampleRate);
            lpState += (raw - lpState) * lpCoeff;
            // 排気の「バフッ」：パルスの頭だけ顔を出す低めのノイズ。
            exhaustLp += (nextNoise() - exhaustLp) * 0.10f;
            float chuff = exhaustLp * pulseEnv * pulseEnv * (0.35f + 0.45f * loadSm);
            engine = (lpState + chuff) * (0.36f + 0.30f * loadSm + 0.10f * rpmSm) * runSm;
            }

            // --- タイヤ音／ブレーキ鳴き ---------------------------------------
            float tire = 0.0f;
            float squeal = 0.0f;
            if (!driftSample.empty()) {
                // 録音サンプル版：ドリフト・サイド・強ブレーキで同じ素材を鳴らす。
                // 鳴き条件（squeal）の方が強めに、速度で再生速度が少し上がる。
                float amount = std::fmax(tireSm, squealSm * 1.2f);
                float cGain = amount > driftVoice.gainSm ? cSquealOn : cTireOff;
                double rate = 0.90 + 0.30 * double(target.tirePitch01);
                tire = driftVoice.next(driftSample, amount * 0.80f, rate, cGain);
            } else {
            // 合成版タイヤ音（帯域ノイズ）
            float tireHz = 700.0f + 950.0f * target.tirePitch01;
            float svfF = 2.0f * std::sin(3.14159265f * tireHz / sampleRate);
            float noise = nextNoise();
            svfLow += svfF * svfBand;
            float high = noise - svfLow - 0.9f * svfBand;  // 減衰係数で Q を決める
            svfBand += svfF * high;
            tire = svfBand * tireSm * 0.55f;

            // --- ブレーキ鳴き（甲高い金属音） --------------------------------
            // 「キキーッ」の正体はパッドとローターの擦れによる自励振動。
            // 純音に近いが、高調波と細かい震え（ビブラート）が耳に刺さる。
            squealVib += pi2 * 8.5f / sampleRate;
            if (squealVib >= pi2) squealVib -= pi2;
            float squealHz = 2750.0f + 280.0f * std::sin(squealVib);
            squealPhase += squealHz / sampleRate;
            if (squealPhase >= 1.0f) squealPhase -= 1.0f;
            float sq = std::sin(pi2 * squealPhase) +
                       std::sin(pi2 * squealPhase * 2.0f) * 0.38f +
                       std::sin(pi2 * squealPhase * 3.0f) * 0.16f +
                       std::sin(pi2 * squealPhase * 4.0f) * 0.06f;
            squeal = sq * squealSm * 0.16f;
            }

            // --- BGM ------------------------------------------------------
            // 等速再生のループ。フェードはゆっくり（ポーズ開閉で音量が
            // 暴れないように）。フェードアウトし切ったら再生位置も止める
            // （ポーズを解いたときに曲が先に進んでいる事故を防ぐ）。
            float bgm = 0.0f;
            if (!bgmSample.empty()) {
                const float cBgm = smoothCoeff(2.5f, sampleRate);
                if (target.bgm > 0.0f || bgmVoice.gainSm > 0.002f) {
                    bgm = bgmVoice.next(bgmSample, target.bgm * 0.55f, 1.0, cBgm);
                }
            }

            float mix = (engine + tire + squeal) * 0.8f + bgm;
            // 簡易リミッタ。割れる前に止める。
            if (mix > 0.95f) mix = 0.95f;
            if (mix < -0.95f) mix = -0.95f;
            out[i] = mix;
        }
    }
};

// SDL デバイス出力の担当。使い方は sound.cpp 側の実装を参照。
// init に失敗してもゲームは無音で続行できる（戻り値 false）。
class GameSound {
public:
    bool init();
    void shutdown();
    void update(const SoundState& state);
    bool ready() const { return ready_; }

    // オーディオコールバックから使う内部状態（直接触らない）
    SoundDsp dsp;
    SoundState target;

private:
    unsigned int deviceId_ = 0;  // SDL_AudioDeviceID
    bool ready_ = false;
};
