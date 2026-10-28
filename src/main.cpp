#include "car.hpp"
#include "render.hpp"
#include "smoke.hpp"
#include "sound.hpp"
#include "track.hpp"

#include <SDL.h>
#include <SDL_image.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <utility>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/html5.h>

#include <cstring>
#endif

namespace {

constexpr double kFixedStepSec = 1.0 / 60.0;
constexpr double kMaxFrameSec = 0.25;

struct Race {
    int completed = 0;
    bool timing = false;
    float lap = 0;
    float best = -1;
    float prevS = -1;
    bool finished = false;  // オープンコース（下り）でゴールしたかどうか
};

struct Orbit {
    Vec3 eye{0, 8, -12};
    Vec3 look{0, 0, 0};
    bool ready = false;
    int mode = 0;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_GLContext gl = nullptr;
    SDL_GameController* pad = nullptr;
    Renderer renderer;
    GameSound sound;
    EngineVoice engineVoice;
    Track track;
    Car car;
    GpuMesh course;
    GpuMesh body;
    GpuMesh wheel;
    GpuMesh shadow;
    Race race;
    Orbit camera;
    SmokeSystem smoke;
    std::vector<float> smokeVerts;
    std::vector<unsigned> smokeIndices;
    bool running = true;
    bool paused = false;
    bool showHelp = true;
    double accumulator = 0;
    Uint64 prevCounter = 0;
    float fps = 0;  // 表示用 FPS（フレーム間隔の指数移動平均）
};

// コースの選択: 引数に `akina` / `suzuka`、または .txt のパスを渡せる。
// 引数が無ければ秋名（ダウンヒル）を読み、秋名ファイルが無い環境では
// 鈴鹿にフォールバックする。以前は無指定＝鈴鹿優先だったため、秋名を
// 入れても通常起動では鈴鹿のままだった（「コースが変わらない」原因）。
std::string findTrackPath(int argc, char** argv) {
    std::string wanted = "akina";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "akina" || arg == "suzuka") {
            wanted = arg;
        } else if (arg.size() > 4 && arg.substr(arg.size() - 4) == ".txt") {
            return arg;
        }
    }
    const std::string names[2] = {wanted, wanted == "akina" ? "suzuka" : "akina"};
    for (const std::string& name : names) {
        const std::string paths[3] = {
            "assets/track/" + name + ".txt",
            "../assets/track/" + name + ".txt",
            "/assets/track/" + name + ".txt",
        };
        for (const std::string& path : paths) {
            std::ifstream in(path);
            if (in) {
                return path;
            }
        }
    }
    return "assets/track/akina.txt";
}

void formatTime(char* buf, int size, float seconds) {
    if (seconds < 0) {
        std::snprintf(buf, size, "--:--.---");
        return;
    }
    int minutes = static_cast<int>(seconds / 60.0f);
    float rest = seconds - float(minutes) * 60.0f;
    std::snprintf(buf, size, "%d:%06.3f", minutes, rest);
}

void openPad(App& app) {
#ifdef __EMSCRIPTEN__
    // ブラウザのパッドは Gamepad API 側で読む。SDL で開くと入力が二重になる。
    (void)app;
    return;
#else
    if (app.pad) {
        return;
    }
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            app.pad = SDL_GameControllerOpen(i);
            if (app.pad) {
                SDL_Log("controller: %s", SDL_GameControllerName(app.pad));
                break;
            }
        }
    }
#endif
}

struct PadPulse {
    bool reset = false;
    bool pause = false;
    bool camera = false;
};

float stickDead(float value) {
    const float dead = 0.14f;
    if (std::fabs(value) < dead) {
        return 0;
    }
    float sign = value > 0 ? 1.0f : -1.0f;
    return (value - sign * dead) / (1.0f - dead);
}

#ifndef __EMSCRIPTEN__
float stickAxis(SDL_GameController* pad, SDL_GameControllerAxis axis) {
    return stickDead(SDL_GameControllerGetAxis(pad, axis) / 32767.0f);
}
#endif

#ifdef __EMSCRIPTEN__
// ブラウザの Gamepad API は 0 が離した状態、1 が押し込み。古いパッドは -1..1 で、離すと -1。
float triggerLevel(float value) {
    if (value < 0) {
        value = (value + 1.0f) * 0.5f;
    }
    return value < 0.04f ? 0.0f : clampf(value, 0.0f, 1.0f);
}

PadPulse readBrowserPad(DriveInput& input) {
    PadPulse pulse;
    static bool prevCamera = false;
    static bool prevReset = false;
    static bool prevPause = false;
    bool camera = false;
    bool reset = false;
    bool pause = false;
    if (emscripten_sample_gamepad_data() != EMSCRIPTEN_RESULT_SUCCESS) {
        prevCamera = prevReset = prevPause = false;
        return pulse;
    }
    const int count = emscripten_get_num_gamepads();
    for (int i = 0; i < count; ++i) {
        EmscriptenGamepadEvent pad;
        if (emscripten_get_gamepad_status(i, &pad) != EMSCRIPTEN_RESULT_SUCCESS || !pad.connected) {
            continue;
        }
        auto axis = [&](int index) {
            return index < pad.numAxes ? static_cast<float>(pad.axis[index]) : 0.0f;
        };
        auto analog = [&](int index) {
            return index < pad.numButtons ? static_cast<float>(pad.analogButton[index]) : 0.0f;
        };
        auto held = [&](int index) {
            return index < pad.numButtons && pad.digitalButton[index];
        };
        const bool standard = std::strcmp(pad.mapping, "standard") == 0 || (pad.numButtons >= 16 && pad.numAxes >= 2);
        input.steer += stickDead(axis(0));
        if (standard) {
            input.steer += stickDead(axis(2));
            if (held(14)) input.steer -= 1;
            if (held(15)) input.steer += 1;
            if (held(12)) input.throttle = 1;
            if (held(13)) input.brake = 1;
            float throttle = analog(7);
            float brake = analog(6);
            if (throttle < 0.01f && pad.numAxes > 5) throttle = triggerLevel(axis(5));
            if (brake < 0.01f && pad.numAxes > 4) brake = triggerLevel(axis(4));
            input.throttle = std::max(input.throttle, throttle);
            input.brake = std::max(input.brake, brake);
            camera = camera || held(3);
            reset = reset || held(8);
            pause = pause || held(9);
            // A(0) または RB(5) がサイドブレーキ
            input.handbrake = input.handbrake || held(0) || held(5);
        } else if (pad.numAxes >= 6) {
            input.throttle = std::max(input.throttle, triggerLevel(axis(5)));
            input.brake = std::max(input.brake, triggerLevel(axis(4)));
        }
    }
    pulse.camera = camera && !prevCamera;
    pulse.reset = reset && !prevReset;
    pulse.pause = pause && !prevPause;
    prevCamera = camera;
    prevReset = reset;
    prevPause = pause;
    return pulse;
}
#endif

DriveInput readDrive(SDL_GameController* pad, PadPulse& pulse) {
    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    DriveInput input;
    if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) input.throttle = 1;
    if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) input.brake = 1;
    if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) input.steer -= 1;
    if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) input.steer += 1;
    if (keys[SDL_SCANCODE_SPACE]) input.handbrake = true;
#ifndef __EMSCRIPTEN__
    if (pad) {
        float throttle = stickAxis(pad, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        float brake = stickAxis(pad, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        if (throttle < 0) throttle = 0;
        if (brake < 0) brake = 0;
        input.throttle = std::max(input.throttle, throttle);
        input.brake = std::max(input.brake, brake);
        input.steer += stickAxis(pad, SDL_CONTROLLER_AXIS_LEFTX);
        input.steer += stickAxis(pad, SDL_CONTROLLER_AXIS_RIGHTX);
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_DPAD_UP)) input.throttle = 1;
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) input.brake = 1;
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_DPAD_LEFT)) input.steer -= 1;
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) input.steer += 1;
        if (SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_A) ||
            SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) input.handbrake = true;
    }
#else
    (void)pad;
#endif
#ifdef __EMSCRIPTEN__
    // SDL はブラウザのパッドをゲームコントローラと認識しないことが多い。
    // 標準マッピングを Gamepad API から直接読む。
    PadPulse browser = readBrowserPad(input);
    pulse.camera = pulse.camera || browser.camera;
    pulse.reset = pulse.reset || browser.reset;
    pulse.pause = pulse.pause || browser.pause;
#else
    (void)pulse;
#endif
    input.throttle = clampf(input.throttle, 0, 1);
    input.brake = clampf(input.brake, 0, 1);
    input.steer = clampf(input.steer, -1, 1);
    return input;
}

void resetRace(App& app) {
    app.car.reset(app.track);
    app.race = Race{};
    app.camera.ready = false;
    app.smoke.clear();
}

void tickRace(Race& race, float s, float length, float dt, bool closed = true) {
    if (!closed) {
        // 一方向の下り: スタート直後から計時し、ゴールライン到達で確定する。
        if (!race.finished) {
            if (s > 5.0f) {
                race.timing = true;
            }
            if (race.prevS > length * 0.5f && s >= length - 3.0f) {
                race.finished = true;
                race.timing = false;
                race.completed = 1;
                race.best = race.lap;
            }
            race.lap += dt;
        }
        race.prevS = s;
        return;
    }
    if (race.prevS >= 0) {
        float gap = (length - race.prevS) + s;
        if (race.prevS > length * 0.5f && s < length * 0.5f && gap < 160.0f) {
            if (race.timing) {
                if (race.best < 0 || race.lap < race.best) {
                    race.best = race.lap;
                }
                race.completed += 1;
            }
            race.timing = true;
            race.lap = 0;
        }
    }
    race.prevS = s;
    race.lap += dt;
}

void updateCamera(App& app, float dt) {
    const float back[3] = {8.6f, 4.1f, 16.0f};
    const float up[3] = {3.3f, 1.55f, 7.2f};
    const float ahead[3] = {9.0f, 6.0f, 14.0f};
    int mode = app.camera.mode;
    // ドリフト中は車体の向きと進行方向がずれる。カメラはその中間を向いて、
    // ノーズの先ではなく「これから進む先」が画面に入るようにする。
    float camYaw = app.car.yaw + app.car.slipAngle * 0.45f;
    Vec3 forward{std::sin(camYaw), 0, std::cos(camYaw)};
    Vec3 eye = Vec3{app.car.x, app.car.y, app.car.z} - forward * back[mode] + Vec3{0, up[mode], 0};
    Vec3 look = Vec3{app.car.x, app.car.y, app.car.z} + forward * ahead[mode] + Vec3{0, 1.15f, 0};
    TrackPose under = app.track.query(eye.x, eye.z, app.car.y, app.car.ground.s);
    if (under.found && under.xzDist < 30.0f && eye.y < under.centerY + 0.8f) {
        eye.y = under.centerY + 0.8f;
    }
    if (!app.camera.ready) {
        app.camera.eye = eye;
        app.camera.look = look;
        app.camera.ready = true;
        return;
    }
    float blend = 1.0f - std::exp(-dt * 7.0f);
    app.camera.eye = app.camera.eye + (eye - app.camera.eye) * blend;
    app.camera.look = app.camera.look + (look - app.camera.look) * blend;
}

void drawMinimap(App& app) {
    // 自車中心のズーム表示（全体図だと峠のヘアピンが潰れて読めないため）。
    // 北が上、表示範囲は約 640 m 四方。全体の進行度は下のバーで示す。
    float size = 190.0f;
    float x0 = 24.0f;
    // 下の操作ヘルプ（height-78 / -54 の2行）と重ならないよう、その上に置く。
    float y0 = float(app.renderer.height) - size - 118.0f;
    app.renderer.addQuad(x0 - 2, y0 - 2, x0 + size + 2, y0 + size + 2, 0.55f, 0.58f, 0.62f, 0.9f);
    app.renderer.addQuad(x0, y0, x0 + size, y0 + size, 0.02f, 0.03f, 0.05f, 0.82f);
    const float span = 640.0f;  // 表示する一辺の長さ (m)
    const float px = size / span;
    auto mapX = [&](float x) { return x0 + size * 0.5f + (x - app.car.x) * px; };
    auto mapY = [&](float z) { return y0 + size * 0.5f - (z - app.car.z) * px; };
    // 線分をミニマップの枠でクリップして描く (Liang-Barsky)。
    auto drawClipped = [&](float ax, float ay, float bx, float by, float r, float g, float b) {
        float dx = bx - ax, dy = by - ay;
        float t0 = 0.0f, t1 = 1.0f;
        const float p[4] = {-dx, dx, -dy, dy};
        const float q[4] = {ax - x0, x0 + size - ax, ay - y0, y0 + size - ay};
        for (int i = 0; i < 4; ++i) {
            if (p[i] == 0.0f) {
                if (q[i] < 0.0f) return;
            } else {
                float t = q[i] / p[i];
                if (p[i] < 0.0f) { if (t > t1) return; if (t > t0) t0 = t; }
                else { if (t < t0) return; if (t < t1) t1 = t; }
            }
        }
        app.renderer.drawLine(ax + dx * t0, ay + dy * t0, ax + dx * t1, ay + dy * t1,
                              2.5f, r, g, b, 1);
    };
    const int n = static_cast<int>(app.track.samples.size());
    const int segs = app.track.closed ? n : n - 1;
    const float viewR = span * 0.75f;
    for (int i = 0; i < segs; ++i) {
        const TrackSample& a = app.track.samples[i];
        const TrackSample& b = app.track.samples[(i + 1) % n];
        if (std::fabs(a.x - app.car.x) > viewR && std::fabs(b.x - app.car.x) > viewR) continue;
        if (std::fabs(a.z - app.car.z) > viewR && std::fabs(b.z - app.car.z) > viewR) continue;
        float r = 0.80f, g = 0.83f, bl = 0.87f;
        if (a.bridge || b.bridge) {
            r = 0.95f; g = 0.72f; bl = 0.28f;
        }
        drawClipped(mapX(a.x), mapY(a.z), mapX(b.x), mapY(b.z), r, g, bl);
    }
    // スタート（緑）とゴール（赤）のマーカー（表示範囲内なら）。
    auto marker = [&](float wx, float wz, float r, float g, float b) {
        float mx = mapX(wx), my = mapY(wz);
        if (mx < x0 || mx > x0 + size || my < y0 || my > y0 + size) return;
        app.renderer.addQuad(mx - 4, my - 4, mx + 4, my + 4, r, g, b, 1);
    };
    if (n > 0) {
        marker(app.track.samples[0].x, app.track.samples[0].z, 0.25f, 0.85f, 0.35f);
        marker(app.track.samples[n - 1].x, app.track.samples[n - 1].z, 0.90f, 0.25f, 0.20f);
    }
    // 自車マーカー（赤い三角、従来より少し大きめ）。
    float mx = x0 + size * 0.5f;
    float my = y0 + size * 0.5f;
    Vec3 nose{std::sin(app.car.yaw), 0, std::cos(app.car.yaw)};
    Vec3 side{std::cos(app.car.yaw), 0, -std::sin(app.car.yaw)};
    float nx = mx + nose.x * 11.0f;
    float ny = my - nose.z * 11.0f;
    float lx = mx - nose.x * 6.0f + side.x * 6.0f;
    float ly = my + nose.z * 6.0f - side.z * 6.0f;
    float rx = mx - nose.x * 6.0f - side.x * 6.0f;
    float ry = my + nose.z * 6.0f + side.z * 6.0f;
    app.renderer.drawLine(lx, ly, nx, ny, 2.5f, 0.95f, 0.25f, 0.20f, 1);
    app.renderer.drawLine(rx, ry, nx, ny, 2.5f, 0.95f, 0.25f, 0.20f, 1);
    app.renderer.drawLine(lx, ly, rx, ry, 2.5f, 0.95f, 0.25f, 0.20f, 1);
    // 北の印。
    app.renderer.drawText(x0 + size - 16, y0 + 4, 1.6f, "N", 0.9f, 0.9f, 0.9f, 1);
    // 全体の進行度バー（ミニマップの下）: チェックポイントに目盛り。
    float barY = y0 + size + 6.0f;
    app.renderer.addQuad(x0, barY, x0 + size, barY + 7.0f, 0.10f, 0.11f, 0.13f, 0.85f);
    float frac = app.track.length > 0.0f ? app.car.ground.s / app.track.length : 0.0f;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    app.renderer.addQuad(x0, barY, x0 + size * frac, barY + 7.0f, 0.30f, 0.75f, 0.95f, 0.95f);
    for (int i = 0; i < n; ++i) {
        const TrackSample& smp = app.track.samples[i];
        if (smp.nameId < 0 || smp.nameId >= static_cast<int>(app.track.names.size())) continue;
        const std::string& nm = app.track.names[smp.nameId];
        if (nm.rfind("Check Point", 0) != 0) continue;
        if (i > 0 && app.track.samples[i - 1].nameId == smp.nameId) continue;
        float tx = x0 + size * (smp.s / app.track.length);
        app.renderer.addQuad(tx - 1, barY - 2.0f, tx + 1, barY + 9.0f, 0.9f, 0.9f, 0.9f, 0.9f);
    }
}

void drawHud(App& app) {
    Renderer& hud = app.renderer;
    hud.beginHud();
    char line[96];
    // ドリフト中は前方成分だけでなく、地面に対する速さそのものを出す。
    int kmh = static_cast<int>(std::lround(app.car.groundSpeed() * 3.6f));
    std::snprintf(line, sizeof(line), "%d KM/H", kmh);
    hud.drawText(24, 22, 4.0f, line, 0.95f, 0.96f, 0.98f, 1);

    if (app.car.drifting) {
        std::snprintf(line, sizeof(line), "DRIFT %d", static_cast<int>(app.car.driftScore));
        hud.drawText(24, 140, 2.6f, line, 1.0f, 0.72f, 0.25f, 1);
    } else if (app.car.driftScore > 0) {
        std::snprintf(line, sizeof(line), "DRIFT SCORE %d", static_cast<int>(app.car.driftScore));
        hud.drawText(24, 140, 2.0f, line, 0.8f, 0.72f, 0.55f, 1);
    }

    if (!app.track.closed && app.race.finished) {
        std::snprintf(line, sizeof(line), "FINISH");
    } else if (!app.track.closed) {
        std::snprintf(line, sizeof(line), "DOWNHILL");
    } else if (!app.race.timing) {
        std::snprintf(line, sizeof(line), "OUT LAP");
    } else {
        std::snprintf(line, sizeof(line), "LAP %d", app.race.completed + 1);
    }
    hud.drawText(24, 64, 2.2f, line, 0.85f, 0.88f, 0.92f, 1);
    formatTime(line, sizeof(line), app.race.finished ? app.race.best : app.race.lap);
    hud.drawText(150, 64, 2.2f, line, 0.95f, 0.96f, 0.98f, 1);

    char best[32];
    formatTime(best, sizeof(best), app.race.best);
    std::snprintf(line, sizeof(line), "%s %s", app.track.closed ? "BEST" : "GOAL", best);
    hud.drawText(24, 90, 2.0f, line, 0.75f, 0.8f, 0.7f, 1);

    int percent = static_cast<int>(app.car.ground.s / app.track.length * 100.0f);
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    const char* corner = app.track.nameOf(app.car.ground.nameId);
    if (app.car.ground.corner > 0 && corner[0]) {
        std::snprintf(line, sizeof(line), "T%d %s  %d%%", app.car.ground.corner, corner, percent);
    } else if (corner[0]) {
        std::snprintf(line, sizeof(line), "%s  %d%%", corner, percent);
    } else {
        std::snprintf(line, sizeof(line), "%d%%", percent);
    }
    hud.drawText(24, 116, 2.0f, line, 0.9f, 0.9f, 0.92f, 1);

    const char* camName = "CHASE";
    if (app.camera.mode == 1) camName = "CLOSE";
    if (app.camera.mode == 2) camName = "FAR";
    hud.drawText(float(hud.width) - 140, 22, 2.0f, camName, 0.8f, 0.84f, 0.88f, 1);
    std::snprintf(line, sizeof(line), "%d FPS", static_cast<int>(std::lround(app.fps)));
    hud.drawText(float(hud.width) - 140, 52, 2.0f, line, 0.8f, 0.84f, 0.88f, 1);

    if (app.showHelp) {
        hud.drawText(24, float(hud.height) - 78, 1.7f,
                     "W/UP ACCEL  S/DOWN BRAKE  A/D STEER  SPACE DRIFT  R RESET  C CAMERA  P PAUSE",
                     0.9f, 0.92f, 0.94f, 1);
        hud.drawText(24, float(hud.height) - 54, 1.7f,
                     "PAD: RT ACCEL  LT BRAKE  STICK STEER  A DRIFT  Y CAMERA  BACK RESET  START PAUSE",
                     0.75f, 0.8f, 0.84f, 1);
    }
    hud.drawText(float(hud.width) - 250, float(hud.height) - 28, 1.6f, "(C) OSM / GSI", 0.85f, 0.88f, 0.9f, 1);
    if (app.paused) {
        hud.drawText(float(hud.width) * 0.5f - 70, float(hud.height) * 0.45f, 5.0f, "PAUSE", 1, 0.95f, 0.9f, 1);
    }
    if (app.race.finished) {
        hud.drawText(float(hud.width) * 0.5f - 105, float(hud.height) * 0.45f, 5.0f, "FINISH", 1, 0.95f, 0.9f, 1);
    }
    drawMinimap(app);
    hud.flushHud();
}

float carPitch(const App& app) {
    Vec3 forward{std::sin(app.car.yaw), 0, std::cos(app.car.yaw)};
    float preferS = app.car.ground.found ? app.car.ground.s : -1.0f;
    TrackPose ahead = app.track.query(app.car.x + forward.x * 2.4f, app.car.z + forward.z * 2.4f, app.car.y, preferS);
    TrackPose behind = app.track.query(app.car.x - forward.x * 2.4f, app.car.z - forward.z * 2.4f, app.car.y, preferS);
    if (!ahead.found || !behind.found) {
        return 0;
    }
    return std::atan2(ahead.centerY - behind.centerY, 4.8f);
}

void frame(void* userdata) {
    App& app = *static_cast<App*>(userdata);
    SDL_Event event;
    bool reset = false;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            app.running = false;
        } else if (event.type == SDL_KEYDOWN && !event.key.repeat) {
            SDL_Scancode code = event.key.keysym.scancode;
            if (code == SDL_SCANCODE_ESCAPE) app.running = false;
            else if (code == SDL_SCANCODE_R) reset = true;
            else if (code == SDL_SCANCODE_C) app.camera.mode = (app.camera.mode + 1) % 3;
            else if (code == SDL_SCANCODE_P) app.paused = !app.paused;
            else if (code == SDL_SCANCODE_H) app.showHelp = !app.showHelp;
        } else if (event.type == SDL_CONTROLLERDEVICEADDED) {
            openPad(app);
        } else if (event.type == SDL_CONTROLLERDEVICEREMOVED && app.pad &&
                   event.cdevice.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(app.pad))) {
            SDL_GameControllerClose(app.pad);
            app.pad = nullptr;
        } else if (event.type == SDL_CONTROLLERBUTTONDOWN) {
            if (event.cbutton.button == SDL_CONTROLLER_BUTTON_BACK) reset = true;
            else if (event.cbutton.button == SDL_CONTROLLER_BUTTON_START) app.paused = !app.paused;
            else if (event.cbutton.button == SDL_CONTROLLER_BUTTON_Y) app.camera.mode = (app.camera.mode + 1) % 3;
        }
    }
    PadPulse padPulse;
    DriveInput input = readDrive(app.pad, padPulse);
    if (padPulse.reset) reset = true;
    if (padPulse.pause) app.paused = !app.paused;
    if (padPulse.camera) app.camera.mode = (app.camera.mode + 1) % 3;
    if (reset) {
        resetRace(app);
    }

    Uint64 now = SDL_GetPerformanceCounter();
    double elapsed = double(now - app.prevCounter) / double(SDL_GetPerformanceFrequency());
    app.prevCounter = now;
    if (elapsed > 0.0001) {
        float inst = static_cast<float>(1.0 / elapsed);
        app.fps = (app.fps <= 0) ? inst : app.fps + (inst - app.fps) * 0.05f;
    }
    if (elapsed > kMaxFrameSec) elapsed = kMaxFrameSec;
    if (elapsed < 0) elapsed = 0;

    if (app.paused) {
        app.accumulator = 0;
    } else {
        app.accumulator += elapsed;
        int guard = 0;
        while (app.accumulator >= kFixedStepSec && guard < 8) {
            app.car.update(float(kFixedStepSec), input, app.track);
            tickRace(app.race, app.car.ground.s, app.track.length, float(kFixedStepSec), app.track.closed);
            app.accumulator -= kFixedStepSec;
            ++guard;
        }
    }
    // 音はフレーム単位でよい。ポーズ中はアイドル＋無音になる。
    // 回転数だけは EngineVoice（ギア状態つき）で作って上書きする。速度への単純な
    // 写像だとギア境界でピッチが跳ねたり往復したりするため。
    SoundState soundState = makeSoundState(
        app.car.speed, app.car.groundSpeed(), input.throttle, input.brake,
        input.handbrake, app.car.drifting, std::fabs(app.car.slipAngle), app.paused);
    soundState.rpm01 = app.engineVoice.update(std::fabs(app.car.speed), input.throttle,
                                              app.car.drifting, app.paused, float(elapsed));
    app.sound.update(soundState);
    updateCamera(app, float(elapsed));

    // ドリフト中は後輪の接地点からタイヤ煙を出す。物理ステップとは独立に
    // フレーム単位で進める（ポーズ中は凍結）。
    if (!app.paused) {
        if (app.car.drifting && app.car.groundSpeed() > 4.0f) {
            Vec3 fwd{std::sin(app.car.yaw), 0, std::cos(app.car.yaw)};
            Vec3 right{fwd.z, 0, -fwd.x};
            Vec3 base{app.car.x, app.car.y + 0.12f, app.car.z};
            base = base - fwd * 1.38f;  // 後輪の位置
            Vec3 carVel{app.car.velX, 0, app.car.velZ};
            float strength = clampf(std::fabs(app.car.slipAngle) * 1.6f, 0.0f, 1.0f) *
                             clampf(app.car.groundSpeed() / 20.0f, 0.25f, 1.0f);
            app.smoke.emit(base - right * 0.82f, base + right * 0.82f, carVel, strength,
                           float(elapsed));
        }
        app.smoke.update(float(elapsed));
    }

    int drawableW = 0;
    int drawableH = 0;
    SDL_GL_GetDrawableSize(app.window, &drawableW, &drawableH);
    app.renderer.resize(drawableW, drawableH);
    app.renderer.clearFrame();
    app.renderer.drawSky();

    float aspect = float(drawableW) / float(drawableH > 0 ? drawableH : 1);
    // スピード感：速いほど視野角を少し広げて、景色の流れを強調する。
    float fovDeg = 60.0f + 9.0f * clampf(app.car.groundSpeed() / 88.0f, 0.0f, 1.0f);
    Mat4 proj = perspective(fovDeg * 3.14159265f / 180.0f, aspect, 0.15f, 4000.0f);
    Mat4 view = lookAt(app.camera.eye, app.camera.look, {0, 1, 0});
    Mat4 viewProj = mul(proj, view);
    app.renderer.beginWorld(viewProj, app.camera.eye);

    Mat4 identity;
    app.renderer.draw(app.course, identity);
    Mat4 shadow = mul(translation(app.car.x, app.car.y + 0.08f, app.car.z), rotationY(app.car.yaw));
    app.renderer.draw(app.shadow, shadow);

    float pitch = carPitch(app);
    float roll = std::atan(app.car.ground.cross);
    Mat4 model = carModelMatrix(app.car, pitch, roll);
    app.renderer.draw(app.body, model);
    const float wheelX = 0.82f;
    const float wheelZ[2] = {1.42f, -1.38f};
    const float steerForWheel[2] = {app.car.steerAngle, 0};
    for (int i = 0; i < 2; ++i) {
        for (int side = -1; side <= 1; side += 2) {
            Mat4 place = mul(translation(float(side) * wheelX, 0.34f, wheelZ[i]),
                             mul(rotationY(steerForWheel[i]), rotationX(app.car.wheelSpin)));
            app.renderer.draw(app.wheel, mul(model, place));
        }
    }
    // タイヤ煙は半透明なので、不透明な車体の後にビルボードで重ねる。
    {
        Vec3 camFwd = normalize(app.camera.look - app.camera.eye);
        Vec3 camRight = normalize(cross(camFwd, Vec3{0, 1, 0}));
        Vec3 camUp = cross(camRight, camFwd);
        app.smoke.buildQuads(camRight, camUp, app.smokeVerts, app.smokeIndices);
        app.renderer.drawSmoke(app.smokeVerts.data(),
                               static_cast<int>(app.smokeVerts.size() / 6),
                               app.smokeIndices.data(), static_cast<int>(app.smokeIndices.size()),
                               viewProj);
    }
    drawHud(app);
    SDL_GL_SwapWindow(app.window);

#ifndef __EMSCRIPTEN__
    if (!app.running) {
        // ネイティブの while が抜ける。
    }
#endif
}

int selfTest(const Track& track) {
    Car car;
    car.reset(track);
    float travelled = 0;
    float prevS = -1;
    float maxLat = 0;
    for (int step = 0; step < 60 * 50; ++step) {
        TrackPose pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
        float lookS = pose.s + 70.0f;
        if (track.closed && lookS >= track.length) {
            lookS -= track.length;
        }
        if (!track.closed && lookS > track.length) {
            lookS = track.length;
        }
        int nearest = 0;
        float nearestGap = 1.0e9f;
        for (int i = 0; i < static_cast<int>(track.samples.size()); i += 2) {
            float gap = std::fabs(track.samples[i].s - lookS);
            if (track.closed && gap > track.length * 0.5f) {
                gap = track.length - gap;
            }
            if (gap < nearestGap) {
                nearestGap = gap;
                nearest = i;
            }
        }
        int next = track.closed ? (nearest + 1) % static_cast<int>(track.samples.size())
                                 : std::min(nearest + 1, static_cast<int>(track.samples.size()) - 1);
        float tx = track.samples[next].x - track.samples[nearest].x;
        float tz = track.samples[next].z - track.samples[nearest].z;
        float targetYaw = std::atan2(tx, tz);
        float errNow = wrapPi(pose.yaw - car.yaw);
        float errAhead = wrapPi(targetYaw - car.yaw);
        float bend = std::fabs(wrapPi(targetYaw - pose.yaw));
        DriveInput input;
        input.steer = clampf(errNow * 2.6f + errAhead * 0.5f, -1.0f, 1.0f);
        input.steer = clampf(input.steer - pose.lateral * 0.12f, -1.0f, 1.0f);
        input.throttle = bend < 0.15f ? 1.0f : 0.45f;
        input.brake = 0;
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
        if (prevS >= 0) {
            float ds = pose.s - prevS;
            if (track.closed && ds < -track.length * 0.5f) ds += track.length;
            if (track.closed && ds > track.length * 0.5f) ds -= track.length;
            if (ds > 0) travelled += ds;
        }
        prevS = pose.s;
        maxLat = std::max(maxLat, pose.xzDist);
        if (!std::isfinite(car.x) || !std::isfinite(car.speed) || !std::isfinite(car.yaw)) {
            std::printf("self-test non-finite\n");
            return 1;
        }
    }
    std::printf("self-test length %.0f travelled %.0f maxLat %.1f speed %.1f km/h\n",
                track.length, travelled, maxLat, car.speed * 3.6f);
    if (track.closed) {
        if (track.length < 5200.0f || track.length > 6500.0f) return 2;  // 鈴鹿
    } else {
        if (track.length < 7000.0f || track.length > 8100.0f) return 2;  // 秋名 7,552 m
    }
    if (travelled < 1200.0f) return 3;
    if (maxLat > 30.0f) return 4;
    return 0;
}

bool initGl(App& app) {
#ifdef __EMSCRIPTEN__
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#endif
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    const char* windowTitle = app.track.title.empty() ? "Suzuka Circuit" : app.track.title.c_str();
    app.window = SDL_CreateWindow(windowTitle, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720,
                                  SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN);
    if (!app.window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        return false;
    }
    app.gl = SDL_GL_CreateContext(app.window);
    if (!app.gl) {
        SDL_Log("SDL_GL_CreateContext failed: %s", SDL_GetError());
        return false;
    }
    SDL_GL_SetSwapInterval(1);
    if (!app.renderer.init()) {
        SDL_Log("GL init failed");
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    bool test = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--self-test") == 0) {
            test = true;
        }
    }
    Track track;
    std::string path = findTrackPath(argc, argv);
    if (!track.load(path)) {
        std::fprintf(stderr, "コースを読めません: %s\n", path.c_str());
        return 1;
    }
    // どのコースが実際に読まれたかを必ず出す（「変わらない」の切り分け用）。
    std::fprintf(stderr, "コース: %s%s%s (length %.0f m, %s)\n", path.c_str(),
                 track.title.empty() ? "" : " - ", track.title.c_str(),
                 track.length, track.closed ? "周回" : "一方向・下り");
    if (test) {
        return selfTest(track);
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    bool imageOk = (IMG_Init(IMG_INIT_PNG) & IMG_INIT_PNG) != 0;
    if (!imageOk) {
        SDL_Log("IMG_Init failed: %s", IMG_GetError());
    }

    App app;
    app.track = std::move(track);
    if (!initGl(app)) {
        SDL_Quit();
        return 1;
    }
    openPad(app);
    // 音は全てリアルタイム合成（sound.cpp）。開けなくても無音で続行する。
    // 以前置いてあった SDL_mixer は初期化だけで何も鳴らしていなかったので置き換えた。
    app.sound.init();
    app.course = app.renderer.upload(app.track.visual);
    CpuMesh body;
    CpuMesh wheel;
    CpuMesh shadow;
    buildCarBody(body);
    buildCarWheel(wheel);
    buildCarShadow(shadow);
    app.body = app.renderer.upload(body);
    app.wheel = app.renderer.upload(wheel);
    app.shadow = app.renderer.upload(shadow);
    app.car.reset(app.track);
    app.prevCounter = SDL_GetPerformanceCounter();

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(frame, &app, 0, 1);
#else
    while (app.running) {
        Uint64 frameStart = SDL_GetPerformanceCounter();
        frame(&app);
        double frameMs = double(SDL_GetPerformanceCounter() - frameStart) / double(SDL_GetPerformanceFrequency()) * 1000.0;
        if (frameMs < 14.0) {
            SDL_Delay(static_cast<Uint32>(14.0 - frameMs));
        }
    }
#endif

    if (app.pad) SDL_GameControllerClose(app.pad);
    app.sound.shutdown();
    if (imageOk) IMG_Quit();
    SDL_GL_DeleteContext(app.gl);
    SDL_DestroyWindow(app.window);
    SDL_Quit();
    return 0;
}
