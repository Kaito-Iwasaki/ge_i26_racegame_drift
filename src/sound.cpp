#include "sound.hpp"

#include <SDL.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

// WAV を読み、デバイスのサンプルレートのモノラル float に変換する。
// SDL 本体の機能だけで足りるので SDL_mixer は使わない。
bool loadWavMono(const std::string& path, SampleLoop& out, int targetRate) {
    SDL_AudioSpec spec{};
    Uint8* buf = nullptr;
    Uint32 len = 0;
    if (SDL_LoadWAV(path.c_str(), &spec, &buf, &len) == nullptr) {
        return false;
    }
    SDL_AudioCVT cvt{};
    int build = SDL_BuildAudioCVT(&cvt, spec.format, spec.channels, spec.freq, AUDIO_F32SYS, 1,
                                  targetRate);
    if (build < 0) {
        SDL_FreeWAV(buf);
        return false;
    }
    if (!cvt.needed) {
        // 既に float モノラル・同レート（まず無いが、そのまま取り込む）。
        int frames = static_cast<int>(len / sizeof(float));
        out.data.assign(reinterpret_cast<const float*>(buf),
                        reinterpret_cast<const float*>(buf) + frames);
        SDL_FreeWAV(buf);
        return frames > 0;
    }
    cvt.len = static_cast<int>(len);
    cvt.buf = static_cast<Uint8*>(SDL_malloc(static_cast<size_t>(len) * cvt.len_mult));
    if (!cvt.buf) {
        SDL_FreeWAV(buf);
        return false;
    }
    std::memcpy(cvt.buf, buf, len);
    SDL_FreeWAV(buf);
    if (SDL_ConvertAudio(&cvt) != 0) {
        SDL_free(cvt.buf);
        return false;
    }
    int frames = cvt.len_cvt / static_cast<int>(sizeof(float));
    out.data.assign(reinterpret_cast<const float*>(cvt.buf),
                    reinterpret_cast<const float*>(cvt.buf) + frames);
    SDL_free(cvt.buf);
    return frames > 0;
}

// assets/sound/ からゲーム用サンプルを読む。見つからなければ合成のまま。
void loadSamples(GameSound& sound, int targetRate) {
    // 探す場所：相対パスに加えて、実行ファイルの隣（CMake が assets/ を
    // コピーする場所）を最優先で見る。ゲームを起動したフォルダによって
    // 「ファイルが見つからず合成音のまま」になる事故を防ぐため。
    std::vector<std::string> dirs = {"assets/sound/", "../assets/sound/", "/assets/sound/"};
    if (const char* basePath = SDL_GetBasePath()) {
        dirs.insert(dirs.begin(), std::string(basePath) + "assets/sound/");
        SDL_free(const_cast<char*>(basePath));
    }
    for (const std::string& dir : dirs) {
        const std::string& base = dir;
        if (!loadWavMono(base + "engine_main.wav", sound.dsp.engineMainSample, targetRate)) {
            continue;
        }
        // アイドル層とドリフト音は任意（無ければ主層／合成で代替される）。
        loadWavMono(base + "engine_idle.wav", sound.dsp.engineIdleSample, targetRate);
        loadWavMono(base + "drift.wav", sound.dsp.driftSample, targetRate);
        SDL_Log("sound: loaded samples from %s (engine %d + idle %d, drift %d samples)", dir.c_str(),
                static_cast<int>(sound.dsp.engineMainSample.data.size()),
                static_cast<int>(sound.dsp.engineIdleSample.data.size()),
                static_cast<int>(sound.dsp.driftSample.data.size()));
        return;
    }
    SDL_Log("sound: sample files not found, using synthesized sounds");
}

void SDLCALL audioCallback(void* userdata, Uint8* stream, int len) {
    GameSound& sound = *static_cast<GameSound*>(userdata);
    Sint16* out = reinterpret_cast<Sint16*>(stream);
    const int samples = len / static_cast<int>(sizeof(Sint16));
    float buffer[2048];
    int done = 0;
    while (done < samples) {
        int chunk = samples - done;
        if (chunk > 2048) chunk = 2048;
        sound.dsp.render(buffer, chunk, sound.target);
        for (int i = 0; i < chunk; ++i) {
            float v = buffer[i];
            if (v > 1.0f) v = 1.0f;
            if (v < -1.0f) v = -1.0f;
            out[done + i] = static_cast<Sint16>(v * 32767.0f);
        }
        done += chunk;
    }
}

}  // namespace

bool GameSound::init() {
    SDL_AudioSpec want{};
    want.freq = 44100;
    want.format = AUDIO_S16SYS;
    want.channels = 1;
    want.samples = 1024;
    want.callback = &audioCallback;
    want.userdata = this;

    SDL_AudioSpec got{};
    SDL_AudioDeviceID device = SDL_OpenAudioDevice(nullptr, 0, &want, &got, 0);
    if (device == 0) {
        SDL_Log("sound: cannot open audio device: %s", SDL_GetError());
        return false;
    }
    deviceId_ = device;
    dsp.sampleRate = static_cast<float>(got.freq > 0 ? got.freq : 44100);
    loadSamples(*this, got.freq > 0 ? got.freq : 44100);
    target = SoundState{};
    target.running = true;
    target.rpm01 = 0.16f;
    ready_ = true;
    SDL_PauseAudioDevice(device, 0);
    return true;
}

void GameSound::update(const SoundState& state) {
    if (!ready_) {
        return;
    }
    // コールバック実行中はデバイスがロックされる。target の差し替えも合わせる。
    SDL_LockAudioDevice(static_cast<SDL_AudioDeviceID>(deviceId_));
    target = state;
    SDL_UnlockAudioDevice(static_cast<SDL_AudioDeviceID>(deviceId_));
}

void GameSound::shutdown() {
    if (!ready_) {
        return;
    }
    SDL_PauseAudioDevice(static_cast<SDL_AudioDeviceID>(deviceId_), 1);
    SDL_CloseAudioDevice(static_cast<SDL_AudioDeviceID>(deviceId_));
    deviceId_ = 0;
    ready_ = false;
}
