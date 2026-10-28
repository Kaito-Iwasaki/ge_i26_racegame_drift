// 録音サンプル版サウンドの試聴プレビュー生成。
// assets/sound/ の実ループ素材を SoundDsp に読み込み、運転シナリオをレンダリングする。
//   g++ -std=c++17 -I.. sound_preview_samples.cpp -o /tmp/preview_samples
//   /tmp/preview_samples sound_preview_samples.wav ../assets/sound
#include "sound.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

bool loadWavS16Mono(const std::string& path, SampleLoop& out) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::vector<unsigned char> bytes;
    unsigned char chunk[8192];
    size_t got;
    while ((got = std::fread(chunk, 1, sizeof(chunk), f)) > 0) {
        bytes.insert(bytes.end(), chunk, chunk + got);
    }
    std::fclose(f);
    // "data" チャンクを探す（自作アセットは標準 44 バイトヘッダだが一応走査する）。
    for (size_t i = 12; i + 8 <= bytes.size();) {
        std::uint32_t size;
        std::memcpy(&size, &bytes[i + 4], 4);
        if (std::memcmp(&bytes[i], "data", 4) == 0) {
            int frames = static_cast<int>(size / 2);
            out.data.resize(frames);
            for (int k = 0; k < frames; ++k) {
                std::int16_t v;
                std::memcpy(&v, &bytes[i + 8 + k * 2], 2);
                out.data[k] = v / 32768.0f;
            }
            return frames > 0;
        }
        i += 8 + size + (size & 1);
    }
    return false;
}

void writeWav(const char* path, const std::vector<float>& samples) {
    FILE* f = std::fopen(path, "wb");
    if (!f) return;
    auto w32 = [&](std::uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto w16 = [&](std::uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::uint32_t dataBytes = static_cast<std::uint32_t>(samples.size()) * 2;
    std::fwrite("RIFF", 1, 4, f);
    w32(36 + dataBytes);
    std::fwrite("WAVEfmt ", 1, 8, f);
    w32(16);
    w16(1);
    w16(1);
    w32(44100);
    w32(44100 * 2);
    w16(2);
    w16(16);
    std::fwrite("data", 1, 4, f);
    w32(dataBytes);
    for (float v : samples) {
        float c = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
        std::int16_t s = static_cast<std::int16_t>(c * 32767.0f);
        std::fwrite(&s, 2, 1, f);
    }
    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    const char* outPath = argc > 1 ? argv[1] : "sound_preview_samples.wav";
    std::string dir = argc > 2 ? argv[2] : "../assets/sound";

    SoundDsp dsp;
    dsp.sampleRate = 44100;
    if (!loadWavS16Mono(dir + "/engine_main.wav", dsp.engineMainSample) ||
        !loadWavS16Mono(dir + "/engine_idle.wav", dsp.engineIdleSample) ||
        !loadWavS16Mono(dir + "/drift.wav", dsp.driftSample)) {
        std::printf("sample load failed\n");
        return 1;
    }
    std::printf("loaded: main %zu, idle %zu, drift %zu samples\n",
                dsp.engineMainSample.data.size(), dsp.engineIdleSample.data.size(),
                dsp.driftSample.data.size());

    // シナリオ（8 秒）: アイドル → 加速 → ドリフト → 強ブレーキ。
    struct Phase {
        float dur, speed, throttle, brake;
        bool handbrake, drifting;
        float slip;
    };
    const Phase phases[] = {
        {1.2f, 0.0f, 0.0f, 0.0f, false, false, 0.0f},     // アイドル
        {3.5f, 62.0f, 1.0f, 0.0f, false, false, 0.0f},    // 加速（速度は下で補間）
        {2.0f, 45.0f, 0.8f, 0.0f, false, true, 0.6f},     // ドリフト
        {1.3f, 30.0f, 0.0f, 1.0f, false, false, 0.0f},    // 強ブレーキ（鳴き）
    };
    EngineVoice voice;
    std::vector<float> all;
    float prevSpeed = 0;
    for (const Phase& ph : phases) {
        int steps = static_cast<int>(ph.dur * 60.0f);
        for (int sIdx = 0; sIdx < steps; ++sIdx) {
            float t = steps > 1 ? float(sIdx) / float(steps - 1) : 1.0f;
            float speed = prevSpeed + (ph.speed - prevSpeed) * t;
            SoundState st = makeSoundState(speed, std::fabs(speed), ph.throttle, ph.brake,
                                           ph.handbrake, ph.drifting, ph.slip, false);
            st.rpm01 = voice.update(std::fabs(speed), ph.throttle, ph.drifting, false, 1.0f / 60.0f);
            float buf[735];
            dsp.render(buf, 735, st);
            all.insert(all.end(), buf, buf + 735);
        }
        prevSpeed = ph.speed;
    }
    writeWav(outPath, all);
    std::printf("wrote %s (%.1fs)\n", outPath, double(all.size()) / 44100.0);
    return 0;
}
