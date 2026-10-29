// サウンド合成の検証ハーネス。SDL を使わず、sound.hpp の純粋な DSP と
// パラメータ変換だけを叩く（実際に鳴る音ではなく、波形の性質を数値で見る）。
//
//   g++ -std=c++17 -I.. sound_test.cpp -o sound_test
//   ./sound_test
#include "sound.hpp"

#include <cmath>
#include <cstdio>
#include <vector>
#include <tuple>

namespace {

int failures = 0;

void check(bool ok, const char* name, float value) {
    std::printf("[%s] %-46s value=%.3f\n", ok ? "PASS" : "FAIL", name, value);
    if (!ok) ++failures;
}

struct Stats {
    float rms = 0;
    float peak = 0;
    float zeroCrossPerSec = 0;
    float roughness = 0;  // 隣接サンプル差の平均絶対値。高域が多いほど大きい
    bool finite = true;
};

Stats renderStats(const SoundState& state, float seconds, bool keepDsp = false) {
    static SoundDsp persistent;
    SoundDsp local;
    SoundDsp& dsp = keepDsp ? persistent : local;
    if (!keepDsp) dsp = SoundDsp{};
    const int sr = 44100;
    dsp.sampleRate = float(sr);
    int total = static_cast<int>(seconds * sr);
    std::vector<float> buf(total);
    dsp.render(buf.data(), total, state);
    Stats st;
    double sumSq = 0;
    double sumDiff = 0;
    int crosses = 0;
    for (int i = 0; i < total; ++i) {
        float v = buf[i];
        if (!std::isfinite(v)) st.finite = false;
        sumSq += double(v) * v;
        st.peak = std::fmax(st.peak, std::fabs(v));
        if (i > 0) {
            sumDiff += std::fabs(v - buf[i - 1]);
            if ((buf[i - 1] < 0 && v >= 0) || (buf[i - 1] >= 0 && v < 0)) ++crosses;
        }
    }
    st.rms = std::sqrt(float(sumSq / total));
    st.zeroCrossPerSec = crosses / seconds;
    st.roughness = float(sumDiff / total);
    return st;
}

SoundState engineOnly(float rpm, float load) {
    SoundState s;
    s.running = true;
    s.rpm01 = rpm;
    s.load = load;
    return s;
}

// 約5msごとの小ブロック RMS のばらつき（変動係数）。
// 点火パルスで音量がうねる音ほど大きくなる＝「ガタガタ」感の物差し。
float blockRmsCov(const SoundState& state, float seconds) {
    SoundDsp dsp;
    dsp.sampleRate = 44100;
    int total = static_cast<int>(seconds * 44100);
    std::vector<float> buf(total);
    dsp.render(buf.data(), total, state);
    const int block = 220;
    int nb = total / block;
    std::vector<float> rms(nb);
    double mean = 0;
    for (int b = 0; b < nb; ++b) {
        double s = 0;
        for (int i = 0; i < block; ++i) {
            float v = buf[b * block + i];
            s += double(v) * v;
        }
        rms[b] = std::sqrt(float(s / block));
        mean += rms[b];
    }
    mean /= nb;
    double var = 0;
    for (int b = 0; b < nb; ++b) var += (rms[b] - mean) * (rms[b] - mean);
    return float(std::sqrt(var / nb) / mean);
}

}  // namespace

int main() {
    // ---- ギア変換 ------------------------------------------------------------
    check(engineRpm01(0, 0) > 0.10f && engineRpm01(0, 0) < 0.25f, "rpm: idle at standstill",
          engineRpm01(0, 0));
    check(engineRpm01(0, 1) > engineRpm01(0, 0) + 0.15f, "rpm: throttle revs at standstill",
          engineRpm01(0, 1));
    check(engineRpm01(13.0f, 0.5f) > 0.90f, "rpm: top of 1st gear", engineRpm01(13.0f, 0.5f));
    check(engineRpm01(14.0f, 0.5f) < 0.45f, "rpm: drops after upshift", engineRpm01(14.0f, 0.5f));
    check(engineRpm01(86.0f, 0.5f) > 0.90f, "rpm: top of top gear", engineRpm01(86.0f, 0.5f));

    // ---- エンジン音の波形 ------------------------------------------------------
    Stats low = renderStats(engineOnly(0.15f, 0.0f), 1.0f);
    Stats high = renderStats(engineOnly(0.85f, 0.8f), 1.0f);
    check(low.finite && high.finite, "engine: samples stay finite", low.rms);
    check(low.rms > 0.02f && low.rms < 0.45f, "engine: audible but not clipping (rms)", low.rms);
    check(low.peak <= 0.951f && high.peak <= 0.951f, "engine: limiter holds peak", high.peak);
    check(high.zeroCrossPerSec > low.zeroCrossPerSec * 1.8f,
          "engine: pitch rises with rpm (crossings/s)", high.zeroCrossPerSec / low.zeroCrossPerSec);
    check(high.rms > low.rms, "engine: louder under load", high.rms - low.rms);
    // 荒さ：アイドルでは点火パルスが 1 発ずつ聞こえる（音量のうねりが大きい）。
    // 回すとパルスが詰まってうねりは均される。その対比が「機械っぽさ」。
    float covIdle = blockRmsCov(engineOnly(0.15f, 0.0f), 1.0f);
    float covHigh = blockRmsCov(engineOnly(0.85f, 0.8f), 1.0f);
    check(covIdle > 0.25f, "engine: strong pulse throb at idle (block rms cov)", covIdle);
    check(covIdle > covHigh * 1.5f, "engine: throb blurs as revs rise (cov ratio)",
          covIdle / covHigh);

    // ---- タイヤ音 ---------------------------------------------------------------
    SoundState tireState = engineOnly(0.5f, 0.2f);
    tireState.tire = 1.0f;
    tireState.tirePitch01 = 0.6f;
    Stats tire = renderStats(tireState, 1.0f);
    Stats plain = renderStats(engineOnly(0.5f, 0.2f), 1.0f);
    check(tire.finite, "tire: samples stay finite", tire.rms);
    check(tire.rms > plain.rms * 1.10f, "tire: adds energy over engine (rms ratio)",
          tire.rms / plain.rms);
    check(tire.roughness > plain.roughness * 1.5f, "tire: adds high-frequency hiss (roughness ratio)",
          tire.roughness / plain.roughness);
    check(tire.peak <= 0.951f, "tire: limiter holds peak", tire.peak);

    // ---- ブレーキ鳴き -------------------------------------------------------------
    {
        SoundState squealOnly;  // running=false でエンジンは無音、鳴きだけ測る
        squealOnly.squeal = 1.0f;
        Stats sq = renderStats(squealOnly, 1.0f);
        check(sq.finite, "squeal: samples stay finite", sq.rms);
        // 旧実装は振幅 0.045 の純音で rms 0.025 ほどしかなかった。「キキーッ」と
        // 聞こえるにはエンジンの上に突き抜ける音量が要る。
        check(sq.rms > 0.07f, "squeal: loud enough to cut through (rms)", sq.rms);
        check(sq.zeroCrossPerSec > 4200.0f && sq.zeroCrossPerSec < 6800.0f,
              "squeal: pitch sits in the ear-piercing band (crossings/s)",
              sq.zeroCrossPerSec);
        check(sq.peak <= 0.951f, "squeal: limiter holds peak", sq.peak);
    }

    // ---- 状態マッピング -----------------------------------------------------------
    SoundState braking = makeSoundState(30, 30, 0, 1.0f, false, false, 0, false);
    check(braking.tire > 0.5f, "map: hard braking at speed makes tire noise", braking.tire);
    check(braking.squeal > 0.0f, "map: hard braking adds squeal", braking.squeal);
    SoundState firm = makeSoundState(25, 25, 0, 0.8f, false, false, 0, false);
    check(firm.squeal > 0.10f, "map: firm braking already squeals clearly", firm.squeal);
    SoundState stopped = makeSoundState(0, 0, 0, 1.0f, false, false, 0, false);
    check(stopped.tire == 0.0f, "map: braking at standstill is silent", stopped.tire);
    SoundState drift = makeSoundState(20, 22, 0.8f, 0, false, true, 0.5f, false);
    check(drift.tire > 0.4f, "map: drifting makes tire noise", drift.tire);
    SoundState hand = makeSoundState(20, 20, 0, 0, true, false, 0, false);
    check(hand.tire > 0.5f, "map: handbrake makes tire noise", hand.tire);
    SoundState cruise = makeSoundState(20, 20, 0.5f, 0, false, false, 0.02f, false);
    check(cruise.tire == 0.0f, "map: plain cruising has no tire noise", cruise.tire);
    SoundState paused = makeSoundState(30, 30, 1, 1, true, true, 0.6f, true);
    check(paused.tire == 0.0f && paused.load == 0.0f, "map: paused is idle and quiet", paused.rpm01);

    // ---- エンジンのギアと回転数（EngineVoice） ----------------------------------
    {
        const float dt = 1.0f / 60.0f;
        // 加速でシフトアップしても、回転数が 1 フレームで跳ねない（滑らかに落ちる）。
        EngineVoice voice;
        float maxStep = 0;
        float prev = voice.rpm01;
        for (int i = 0; i < 60 * 4; ++i) {
            float speed = 30.0f * (i * dt / 4.0f);  // 0 → 30 m/s を 4 秒で
            float r = voice.update(speed, 0.8f, false, false, dt);
            maxStep = std::fmax(maxStep, std::fabs(r - prev));
            prev = r;
        }
        check(maxStep < 0.09f, "voice: rpm never jumps in one frame", maxStep);
        check(voice.gear >= 1, "voice: upshifts while accelerating", float(voice.gear));

        // ギア境界付近で速度が揺れても、ギアもピッチも往復しない（ハンチング防止）。
        // まず境界の速度で 2 速のまま落ち着かせてから揺らす（遷移そのものは測らない）。
        EngineVoice hunt;
        for (int i = 0; i < 60 * 2; ++i) hunt.update(20.0f, 0.5f, false, false, dt);
        for (int i = 0; i < 60 * 2; ++i) hunt.update(13.5f, 0.5f, false, false, dt);
        int gearSettled = hunt.gear;
        float rMin = 1.0f, rMax = 0.0f;
        bool gearStable = true;
        for (int i = 0; i < 60 * 3; ++i) {
            float speed = 13.5f + 0.8f * std::sin(i * 0.11f);  // 1速上限の前後で揺らす
            float r = hunt.update(speed, 0.5f, false, false, dt);
            gearStable = gearStable && (hunt.gear == gearSettled);
            rMin = std::fmin(rMin, r);
            rMax = std::fmax(rMax, r);
        }
        check(gearStable, "voice: no gear hunting near the boundary", float(gearSettled));
        check(rMax - rMin < 0.25f, "voice: pitch stays put near the boundary (range)",
              rMax - rMin);

        // 十分に減速すればシフトダウンする。
        for (int i = 0; i < 60 * 2; ++i) hunt.update(6.0f, 0.2f, false, false, dt);
        check(hunt.gear == 0, "voice: downshifts when properly slow", float(hunt.gear));

        // ドリフト中はホイールスピンで回転が跳ね上がる。
        EngineVoice spin;
        for (int i = 0; i < 60 * 2; ++i) spin.update(20.0f, 0.5f, false, false, dt);
        float cruiseRpm = spin.rpm01;
        for (int i = 0; i < 60; ++i) spin.update(20.0f, 1.0f, true, false, dt);
        check(spin.rpm01 > cruiseRpm + 0.10f, "voice: wheelspin flare while drifting",
              spin.rpm01 - cruiseRpm);
    }

    // ---- エンジン停止で消える ------------------------------------------------------
    {
        SoundDsp dsp;
        dsp.sampleRate = 44100;
        std::vector<float> buf(44100);
        SoundState on = engineOnly(0.5f, 0.5f);
        dsp.render(buf.data(), 44100, on);
        SoundState off;  // running=false
        dsp.render(buf.data(), 44100, off);  // 最初の1秒はフェード区間として捨てる
        dsp.render(buf.data(), 22050, off);  // フェード後の定常区間を測る
        double sumSq = 0;
        for (int i = 0; i < 22050; ++i) sumSq += double(buf[i]) * buf[i];
        float rmsOff = std::sqrt(float(sumSq / 22050));
        check(rmsOff < 0.01f, "engine: fades out when stopped (rms)", rmsOff);
    }

    // ---- 録音サンプル再生（SampleLoop / SampleVoice） ------------------------------
    {
        // read() の補間と循環。
        SampleLoop loop;
        loop.data = {0.0f, 1.0f, 0.0f, -1.0f};
        check(std::fabs(loop.read(0.0) - 0.0f) < 1e-6f, "sample: read at 0", float(loop.read(0.0)));
        check(std::fabs(loop.read(0.5) - 0.5f) < 1e-6f, "sample: read interpolates",
              float(loop.read(0.5)));
        check(std::fabs(loop.read(4.0) - loop.read(0.0)) < 1e-6f, "sample: read wraps",
              float(loop.read(4.0)));
        check(std::fabs(loop.read(-0.5) - loop.read(3.5)) < 1e-6f, "sample: negative wraps",
              float(loop.read(-0.5)));

        // 正弦ループを等速再生：クリック無く、RMS が安定する。2 倍速では交差が倍。
        SampleLoop sine;
        const int period = 441;  // 100 Hz
        sine.data.resize(period);
        for (int i = 0; i < period; ++i) {
            sine.data[i] = std::sin(6.2831853f * i / period);
        }
        auto voiceStats = [&](double rate) {
            SampleVoice voice;
            std::vector<float> buf(44100);
            for (int i = 0; i < 44100; ++i) buf[i] = voice.next(sine, 1.0f, rate, 1.0f);
            double sumSq = 0;
            float maxStep = 0;
            int crosses = 0;
            for (int i = 0; i < 44100; ++i) {
                sumSq += double(buf[i]) * buf[i];
                if (i > 0) {
                    maxStep = std::fmax(maxStep, std::fabs(buf[i] - buf[i - 1]));
                    if ((buf[i - 1] < 0 && buf[i] >= 0) || (buf[i - 1] >= 0 && buf[i] < 0)) ++crosses;
                }
            }
            return std::make_tuple(std::sqrt(float(sumSq / 44100)), maxStep, float(crosses));
        };
        auto [rms1, step1, zc1] = voiceStats(1.0);
        auto [rms2, step2, zc2] = voiceStats(2.0);
        check(rms1 > 0.65f && rms1 < 0.75f, "sample: loop RMS stable", rms1);
        check(step1 < 0.05f, "sample: no clicks at loop seam", step1);
        check(zc2 > zc1 * 1.8f && zc2 < zc1 * 2.2f, "sample: rate doubles crossings", zc2 / zc1);
    }

    // ---- サンプル版エンジン／ドリフト音（ダミーのループ素材で配線を検証） ------------
    {
        auto makeSineLoop = [](int period) {
            SampleLoop s;
            s.data.resize(period);
            for (int i = 0; i < period; ++i) s.data[i] = std::sin(6.2831853f * i / period);
            return s;
        };
        SoundDsp dsp;
        dsp.sampleRate = 44100;
        dsp.engineIdleSample = makeSineLoop(882);  // 50 Hz
        dsp.engineMainSample = makeSineLoop(441);  // 100 Hz
        dsp.driftSample = makeSineLoop(49);        // 900 Hz
        auto renderRmsZc = [&](const SoundState& st, float seconds) {
            int total = static_cast<int>(seconds * 44100);
            std::vector<float> buf(total);
            dsp.render(buf.data(), total, st);
            double sumSq = 0;
            int crosses = 0;
            for (int i = 0; i < total; ++i) {
                sumSq += double(buf[i]) * buf[i];
                if (i > 0 && ((buf[i - 1] < 0 && buf[i] >= 0) || (buf[i - 1] >= 0 && buf[i] < 0))) {
                    ++crosses;
                }
            }
            return std::make_pair(std::sqrt(float(sumSq / total)), crosses / seconds);
        };
        // 低回転（アイドル層が主）→ 高回転（主層＋高速）で交差数が増える。
        dsp.render(nullptr, 0, engineOnly(0.16f, 0.0f));
        std::vector<float> warm(22050);
        dsp.render(warm.data(), 22050, engineOnly(0.16f, 0.0f));  // 平滑化を落ち着かせる
        auto [rmsIdle, zcIdle] = renderRmsZc(engineOnly(0.16f, 0.0f), 1.0f);
        auto [rmsTop, zcTop] = renderRmsZc(engineOnly(0.95f, 1.0f), 1.0f);
        check(rmsIdle > 0.02f, "sample engine: audible at idle (rms)", rmsIdle);
        check(zcTop > zcIdle * 1.6f, "sample engine: pitch rises with rpm", zcTop / zcIdle);
        check(rmsTop > rmsIdle, "sample engine: louder at top (rms)", rmsTop);

        // ドリフト素材：tire でも squeal でも鳴り、無ければエンジンだけになる。
        SoundState driftState = engineOnly(0.5f, 0.5f);
        driftState.tire = 1.0f;
        driftState.tirePitch01 = 0.5f;
        auto [rmsDrift, zcDrift] = renderRmsZc(driftState, 1.0f);
        SoundState squealState = engineOnly(0.5f, 0.5f);
        squealState.squeal = 1.0f;
        auto [rmsSqueal, zcSqueal] = renderRmsZc(squealState, 1.0f);
        auto [rmsPlain, zcPlain] = renderRmsZc(engineOnly(0.5f, 0.5f), 1.0f);
        check(rmsDrift > rmsPlain * 1.3f, "sample drift: plays while drifting (rms)", rmsDrift);
        check(rmsSqueal > rmsPlain * 1.3f, "sample drift: plays on brake squeal (rms)", rmsSqueal);
    }

    // ---- BGM ループ ---------------------------------------------------
    {
        SoundDsp dsp;
        dsp.sampleRate = 44100;
        // 100 Hz の正弦波 1 周期（441 サンプル）を BGM 素材にする。
        dsp.bgmSample.data.resize(441);
        for (int i = 0; i < 441; ++i) {
            dsp.bgmSample.data[i] = std::sin(6.2831853f * i / 441);
        }
        auto renderBuf = [&](const SoundState& st, float seconds) {
            int total = static_cast<int>(seconds * 44100);
            std::vector<float> buf(total);
            dsp.render(buf.data(), total, st);
            double sumSq = 0;
            bool finite = true;
            for (float v : buf) {
                if (!std::isfinite(v)) finite = false;
                sumSq += double(v) * v;
            }
            return std::make_pair(std::sqrt(float(sumSq / total)), finite);
        };
        SoundState bgmOn;  // running=false でエンジン無音、BGM だけを測る
        bgmOn.bgm = 1.0f;
        renderBuf(bgmOn, 1.0f);  // フェードインを落ち着かせる
        auto [rmsBgm, finBgm] = renderBuf(bgmOn, 1.0f);
        check(finBgm, "bgm: finite", finBgm ? 1.0f : 0.0f);
        check(rmsBgm > 0.2f, "bgm: audible when on (rms)", rmsBgm);
        // 素材の長さ（0.01 秒）を何周も越えて鳴り続ける = ループしている。
        auto [rmsLoop, finLoop] = renderBuf(bgmOn, 2.0f);
        check(finLoop && rmsLoop > 0.2f, "bgm: keeps looping (rms)", rmsLoop);
        // bgm=0 ではフェードアウトして無音になり、再生位置も止まる。
        SoundState bgmOff;
        renderBuf(bgmOff, 2.0f);
        auto [rmsOff, finOff] = renderBuf(bgmOff, 1.0f);
        check(rmsOff < 0.02f, "bgm: silent when off (rms)", rmsOff);
        double posBefore = dsp.bgmVoice.pos;
        renderBuf(bgmOff, 1.0f);
        check(dsp.bgmVoice.pos == posBefore, "bgm: position frozen while off",
              float(dsp.bgmVoice.pos - posBefore));
        // Countdown is silent and does not advance the BGM voice; GO enables it.
        dsp.resetBgm();
        SoundState countdown = makeSoundState(0, 0, 0.7f, 0, false, false, 0, false, false);
        check(countdown.bgm == 0 && countdown.running && countdown.load > 0,
              "bgm: countdown mutes music but keeps engine", countdown.bgm);
        countdown.running = false; // Isolate music in this DSP test.
        auto [rmsCountdown, finCountdown] = renderBuf(countdown, 1.0f);
        check(finCountdown && rmsCountdown < .02f && dsp.bgmVoice.pos == 0,
              "bgm: countdown silent at the start position", rmsCountdown);
        SoundState go = makeSoundState(0, 0, 0, 0, false, false, 0, false, true);
        check(go.bgm == 1, "bgm: starts at GO", go.bgm);
        go.running = false;
        renderBuf(go, 0.503f);
        check(dsp.bgmVoice.gainSm > 0 && dsp.bgmVoice.pos != 0,
              "bgm: advances after GO", dsp.bgmVoice.gainSm);
        dsp.resetBgm();
        check(dsp.bgmVoice.pos == 0 && dsp.bgmVoice.gainSm == 0,
              "bgm: reset rewinds and silences", dsp.bgmVoice.gainSm);
        check(makeSoundState(0,0,0,0,false,false,0,true,true).bgm == 0,
              "bgm: pause after GO mutes", 0);
        // makeSoundState 側の配線：走行中は流れ、ポーズでは止まる。
        check(makeSoundState(20, 20, 0.5f, 0, false, false, 0.02f, false).bgm == 1.0f,
              "bgm: on while driving", 1.0f);
        check(makeSoundState(20, 20, 0.5f, 0, false, false, 0.02f, true).bgm == 0.0f,
              "bgm: off while paused", 0.0f);
    }

    std::printf(failures == 0 ? "\nALL PASS\n" : "\n%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
