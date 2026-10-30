#include "car.hpp"
#include "camera.hpp"
#include "menu.hpp"
#include "rival.hpp"
#include "vehicle_collision.hpp"
#include "flare.hpp"
#include "wheel_pose.hpp"
#include "suspension_visual.hpp"
#include "countdown.hpp"
#include "render.hpp"
#include "lighttrail.hpp"
#include "smoke.hpp"
#include "effects.hpp"
#include "skidmarks.hpp"
#include "rumble.hpp"
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

#ifdef __EMSCRIPTEN__
// Gamepad haptics are optional: unsupported devices, browser policies and
// rejected promises must never interrupt driving.
EM_JS(void, browserGamepadRumble, (float low, float high, int stop), {
    try {
        if (!navigator.getGamepads) return;
        const pads = Array.from(navigator.getGamepads()).filter(p => p && p.connected);
        if (!pads.length) { Module.rumblePadIndex = undefined; return; }
        let pad = pads.find(p => p.index === Module.rumblePadIndex);
        if (!stop) {
            const active = pads.find(p => p.buttons.some(b => b.pressed || b.value > 0.15) ||
                                           p.axes.some(a => Math.abs(a) > 0.20));
            if (active) pad = active;
        }
        pad = pad || pads[0];
        if (Module.rumblePadIndex !== undefined && Module.rumblePadIndex !== pad.index) {
            const old = pads.find(p => p.index === Module.rumblePadIndex);
            if (old && old.vibrationActuator && old.vibrationActuator.reset) {
                const result = old.vibrationActuator.reset();
                if (result && result.catch) result.catch(() => {});
            }
        }
        Module.rumblePadIndex = pad.index;
        const motor = pad.vibrationActuator || (pad.hapticActuators && pad.hapticActuators[0]);
        if (!motor) return;
        let result;
        if (stop || document.hidden) {
            if (motor.reset) result = motor.reset();
            else if (motor.pulse) result = motor.pulse(0, 1);
        } else if (motor.playEffect) {
            result = motor.playEffect('dual-rumble', {duration: 140, startDelay: 0,
                strongMagnitude: Math.max(0, Math.min(1, low)),
                weakMagnitude: Math.max(0, Math.min(1, high))});
        } else if (motor.pulse) {
            result = motor.pulse(Math.max(low, high), 140);
        }
        if (result && result.catch) result.catch(() => {});
    } catch (_) { /* Haptics are not available; driving continues normally. */ }
});
#endif

namespace {

constexpr double kFixedStepSec = 1.0 / 60.0;
constexpr double kMaxFrameSec = 0.25;
// Arcade pace: HUD uses simulation km/h; world motion runs 25% faster.
// Scale the complete car step to preserve the drift path and steering balance.
constexpr float kDrivingPace = 1.25f;

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
    Vec3 up{0, 1, 0};
    int mode = 0;
    CameraBlend blend;
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
    Rival rival;
    SmokeSystem rivalSmoke;
    SuspensionVisual rivalSuspension;
    SkidMarks rivalSkids;
    LightTrail rivalTrail;
    float rivalWheelRoad[4] = {};
    float rivalPitch=0, rivalRoll=0;
    std::vector<float> flareVerts;
    std::vector<unsigned> flareIndices;
    bool rivalContact = false;
    int battleResult = 0; // 0 ongoing, 1 player wins, -1 rival wins
    bool battle = false;
    int difficulty = 1;
    GpuMesh course;
    GpuMesh body;
    GpuMesh rivalBody;
    GpuMesh brakeLights;
    GpuMesh headlights;
    GpuMesh reverseLights;
    GpuMesh wheel;
    GpuMesh shadow;
    bool braking = false;  // 今フレームのブレーキ入力（ブレーキランプ点灯用）
    Race race;
    StartCountdown countdown;
    SuspensionVisual suspensionVisual;
    Orbit camera;
    SmokeSystem smoke;
    SurfaceEffects surfaceEffects;
    SkidMarks skidMarks;
    std::vector<float> skidVerts;
    std::vector<unsigned> skidIndices;
    RumbleState rumble;
    std::vector<float> surfaceVerts;
    std::vector<unsigned> surfaceIndices;
    std::vector<float> smokeVerts;
    std::vector<unsigned> smokeIndices;
    LightTrail brakeTrail;
    std::vector<float> trailVerts;
    std::vector<unsigned> trailIndices;
    bool running = true;
    bool paused = false;
    bool night = false;
    bool uphill = false;
    bool titleScreen = true;
    bool showHelp = true;
    int menuSelection = 9;
    bool menuPadUp = false, menuPadDown = false, menuPadLeft = false, menuPadRight = false;
    bool menuPadConfirm = false;
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

void sendRumble(SDL_GameController* pad, const RumbleCommand& command) {
    if (!command.send && !command.stop) return;
#ifdef __EMSCRIPTEN__
    (void)pad;
    browserGamepadRumble(command.low, command.high, command.stop ? 1 : 0);
#else
    if (!pad) return;
#if SDL_VERSION_ATLEAST(2, 0, 9)
    SDL_GameControllerRumble(pad,
        command.stop ? 0 : static_cast<Uint16>(RumbleState::clamp01(command.low) * 65535.0f),
        command.stop ? 0 : static_cast<Uint16>(RumbleState::clamp01(command.high) * 65535.0f),
        command.stop ? 0 : 140);
#endif
#endif
}

struct PadPulse {
    bool reset = false;
    bool pause = false;
    bool camera = false;
    bool start = false;
    bool night = false;
    bool menuUp = false, menuDown = false, menuLeft = false, menuRight = false;
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
    static bool prevStart = false;
    static bool prevNight = false;
    bool camera = false;
    bool reset = false;
    bool pause = false;
    bool start = false;
    bool night = false;
    bool up=false, down=false, left=false, right=false;
    if (emscripten_sample_gamepad_data() != EMSCRIPTEN_RESULT_SUCCESS) {
        prevCamera = prevReset = prevPause = prevStart = prevNight = false;
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
        up = up || axis(1) < -0.55f || held(12);
        down = down || axis(1) > 0.55f || held(13);
        left = left || axis(0) < -0.55f || held(14);
        right = right || axis(0) > 0.55f || held(15);
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
            start = start || held(0);
            night = night || held(1);
            // A(0) または RB(5) がサイドブレーキ
            input.handbrake = input.handbrake || held(0) || held(5);
        } else if (pad.numAxes >= 6) {
            input.throttle = std::max(input.throttle, triggerLevel(axis(5)));
            input.brake = std::max(input.brake, triggerLevel(axis(4)));
        }
    }
    pulse.menuUp=up; pulse.menuDown=down; pulse.menuLeft=left; pulse.menuRight=right;
    pulse.night = night && !prevNight;
    prevNight = night;
    pulse.start = start && !prevStart;
    prevStart = start;
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
    pulse.menuUp=browser.menuUp; pulse.menuDown=browser.menuDown;
    pulse.menuLeft=browser.menuLeft; pulse.menuRight=browser.menuRight;
    pulse.night = pulse.night || browser.night;
    pulse.start = pulse.start || browser.start;
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
    if (app.battle && app.car.ground.found) {
        // Put the player and rival on separate sides of the same starting line.
        const float lane=2.10f;
        app.car.x += app.car.ground.rightX*lane;
        app.car.z += app.car.ground.rightZ*lane;
        app.car.ground=app.track.query(app.car.x,app.car.z,app.car.y,app.car.ground.s);
        if (app.car.ground.found) app.car.y=app.car.ground.centerY+app.car.ground.lateral*app.car.ground.cross;
    }
    app.rival.reset(app.track,app.car.ground.s,app.difficulty);
    app.rivalSmoke.clear();
    app.rivalSuspension.reset();
    app.rivalSkids.clear();
    app.rivalSkids.minSpacing=1.0f;
    app.rivalTrail.clear();
    app.battleResult=0;
    app.rivalContact=false;
    app.race = Race{};
    if (app.battle && app.track.closed) { app.race.timing=true; app.race.prevS=app.car.ground.s; }
    app.countdown.reset();
    app.suspensionVisual.reset();
    app.sound.resetBgm();
    app.accumulator = 0;
    app.smoke.clear();
    app.surfaceEffects.clear();
    app.skidMarks.clear();
    sendRumble(app.pad, RumbleCommand{0, 0, false, true});
    app.rumble.clear();
    app.brakeTrail.clear();
}

// Distant rivals continue racing, but their nearby visual effects can be skipped.
bool rivalNear(const App& app,float radius) {
    const float dx=app.rival.car.x-app.car.x, dz=app.rival.car.z-app.car.z;
    return dx*dx+dz*dz<=radius*radius && std::fabs(app.rival.car.y-app.car.y)<100.0f;
}

// Project each rear tyre onto the nearest road surface, including camber.
void recordTyreMarks(App& app, const Car& car, SkidMarks& marks, bool braking, float brakeInput) {
    const bool sliding = car.drifting && std::fabs(car.slipAngle) > 0.12f;
    const bool active = (braking || sliding) && car.groundSpeed() > 2.0f && car.ground.found;
    if (!active) {
        for (int i=0;i<2;++i) marks.sample(i, false, {}, 0);
        return;
    }
    const Vec3 fwd{std::sin(car.yaw),0,std::cos(car.yaw)};
    const Vec3 right{fwd.z,0,-fwd.x};
    const float brakeStrength = braking ? clampf(brakeInput,0.0f,1.0f) : 0.0f;
    const float driftStrength = sliding ? clampf((std::fabs(car.slipAngle) - 0.12f) / 0.45f, 0.15f, 1.0f) : 0.0f;
    const float strength = std::max(brakeStrength, driftStrength);
    for (int wheel=0; wheel<2; ++wheel) {
        const float side=wheel==0 ? -1.0f : 1.0f;
        const Vec3 center=Vec3{car.x,car.y,car.z}-fwd*1.38f+right*(side*0.82f);
        if (marks.minSpacing > SkidMarks::kMinSpacing && marks.connected[wheel]) {
            const Vec3 previous=(marks.previous[wheel].left+marks.previous[wheel].right)*0.5f;
            const float dx=center.x-previous.x,dz=center.z-previous.z;
            if(dx*dx+dz*dz<marks.minSpacing*marks.minSpacing) continue;
        }
        const TrackPose p=app.track.query(center.x,center.z,car.y,car.ground.s);
        const float across = right.x*p.rightX+right.z*p.rightZ;
        const float half=.14f;
        if (!p.found || std::fabs(p.lateral)+std::fabs(across)*half > app.track.halfWidth+0.15f) {
            marks.sample(wheel,false,{},0);
            continue;
        }
        const float y=p.centerY+p.lateral*p.cross+0.035f;
        const Vec3 width=right*half;
        SkidMarks::Edge edge{center-width,center+width};
        edge.left.y=y-p.cross*across*half;
        edge.right.y=y+p.cross*across*half;
        marks.sample(wheel,true,edge,strength);
    }
}

void tickRace(Race& race, float s, float length, float dt, bool closed = true, float startS = 0.0f, float finishS = -1.0f) {
    if (!closed) {
        // 一方向の下り: スタート直後から計時し、ゴールライン到達で確定する。
        if (!race.finished) {
            if (s > startS + 5.0f) {
                race.timing = true;
            }
            if (finishS < 0.0f) finishS = length;
            if (race.prevS > (startS + finishS) * 0.5f && s >= finishS) {
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

void updateCamera(App& app, float pitch, float roll, float dt) {
    Orbit& cam = app.camera;
    CameraPose chase = carCameraPose(app.car, 0, pitch, roll);
    // Apply road clearance to the chase endpoint before blending.
    TrackPose under = app.track.query(chase.eye.x, chase.eye.z, app.car.y, app.car.ground.s);
    if (under.found && under.xzDist < 30.0f && chase.eye.y < under.centerY + 0.8f)
        chase.eye.y = under.centerY + 0.8f;
    const CameraPose hood = carCameraPose(app.car, kHoodCameraMode, pitch, roll);
    cam.blend.update(cam.mode, dt);
    const CameraPose pose = cam.blend.apply(chase, hood);
    cam.eye = pose.eye; cam.look = pose.look; cam.up = pose.up;
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
        marker(app.track.samples[app.track.startGate].x, app.track.samples[app.track.startGate].z, 0.25f, 0.85f, 0.35f);
        marker(app.track.samples[app.track.finishGate].x, app.track.samples[app.track.finishGate].z, 0.90f, 0.25f, 0.20f);
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
    const float raceLength=app.track.raceFinishS-app.track.raceStartS;
    float frac = raceLength > 0.0f ? (app.car.ground.s-app.track.raceStartS) / raceLength : 0.0f;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    app.renderer.addQuad(x0, barY, x0 + size * frac, barY + 7.0f, 0.30f, 0.75f, 0.95f, 0.95f);
    for (int i = 0; i < n; ++i) {
        const TrackSample& smp = app.track.samples[i];
        if (smp.nameId < 0 || smp.nameId >= static_cast<int>(app.track.names.size())) continue;
        const std::string& nm = app.track.names[smp.nameId];
        if (nm.rfind("Check Point", 0) != 0) continue;
        if (i > 0 && app.track.samples[i - 1].nameId == smp.nameId) continue;
        float tx = x0 + size * clampf((smp.s-app.track.raceStartS) / std::max(1.0f, raceLength),0.0f,1.0f);
        app.renderer.addQuad(tx - 1, barY - 2.0f, tx + 1, barY + 9.0f, 0.9f, 0.9f, 0.9f, 0.9f);
    }
}

void drawMenuButton(Renderer& hud, MenuRect r, const char* label, bool selected, bool active) {
    const float border=selected?3.0f:1.0f;
    hud.addQuad(r.x-border,r.y-border,r.x+r.w+border,r.y+r.h+border,
                selected?1.0f:0.25f,selected?0.65f:0.32f,selected?0.25f:0.40f,1);
    hud.addQuad(r.x,r.y,r.x+r.w,r.y+r.h, active?0.43f:0.07f,active?0.23f:0.10f,active?0.08f:0.15f,1);
    const float scale=std::min(r.h/18.0f, r.w/(float(std::strlen(label))*6.0f+8.0f));
    const float width=float(std::strlen(label))*6.0f*scale;
    hud.drawText(r.x+(r.w-width)*0.5f,r.y+(r.h-7.0f*scale)*0.5f,scale,label,1,0.95f,0.88f,1);
}

void drawTitle(App& app) {
    Renderer& hud=app.renderer; hud.beginHud();
    MenuLayout m(float(hud.width),float(hud.height));
    const float l=m.left,t=m.top,s=m.scale;
    hud.addQuad(0,0,float(hud.width),float(hud.height),0.01f,0.02f,0.04f,0.80f);
    hud.addQuad(l,t,l+810*s,t+540*s,0.025f,0.04f,0.06f,0.96f);
    hud.addQuad(l,t,l+810*s,t+5*s,1,0.55f,0.18f,1);
    auto text=[&](float x,float y,float size,const char* label,float r,float g,float b){hud.drawText(l+x*s,t+y*s,size*s,label,r,g,b,1);};
    text(44,22,4.2f,"RACE GAME DRIFT",1,.95f,.85f);
    text(44,72,1.6f,"KEYS: W/S GAS/BRAKE  A/D STEER  SPACE DRIFT",.9f,.92f,.95f);
    text(44,96,1.6f,"PAD: RT/LT GAS/BRAKE  STICK STEER  A DRIFT",.9f,.92f,.95f);
    text(44,120,1.6f,"C/Y CAMERA   P/START PAUSE",.75f,.8f,.85f);
    text(44,211,1.5f,"TIME OF DAY",.95f,.59f,.25f);
    drawMenuButton(hud,m.title(0),"DAY",app.menuSelection==0,!app.night);
    drawMenuButton(hud,m.title(1),"NIGHT",app.menuSelection==1,app.night);
    text(44,269,1.5f,"COURSE DIRECTION",.95f,.59f,.25f);
    drawMenuButton(hud,m.title(2),"DOWNHILL",app.menuSelection==2,!app.uphill);
    drawMenuButton(hud,m.title(3),app.track.closed || !app.track.touge?"UPHILL N/A":"UPHILL",app.menuSelection==3,app.uphill);
    text(44,327,1.5f,"GAME MODE",.95f,.59f,.25f);
    drawMenuButton(hud,m.title(4),"TIME ATTACK",app.menuSelection==4,!app.battle);
    drawMenuButton(hud,m.title(5),"BATTLE",app.menuSelection==5,app.battle);
    text(44,385,1.5f,app.battle?"RIVAL DIFFICULTY":"RIVAL DIFFICULTY - BATTLE ONLY",.95f,.59f,.25f);
    const char* levels[3]={"EASY","NORMAL","HARD"};
    for(int i=0;i<3;i++)drawMenuButton(hud,m.title(i+6),levels[i],app.menuSelection==i+6,app.battle && app.difficulty==i);
    drawMenuButton(hud,m.title(9),"START RACE",app.menuSelection==9,false);
    text(44,520,1.2f,"MOUSE / ARROWS / STICK   ENTER / A: SELECT",.7f,.77f,.82f);
    hud.flushHud();
}

void drawPauseMenu(App& app) {
    Renderer& hud=app.renderer; hud.beginHud(); MenuLayout m(float(hud.width),float(hud.height));
    hud.addQuad(0,0,float(hud.width),float(hud.height),0.01f,0.02f,0.04f,0.75f);
    hud.addQuad(m.left+165*m.scale,m.top+115*m.scale,m.left+645*m.scale,m.top+465*m.scale,.025f,.04f,.06f,.96f);
    hud.drawText(m.left+330*m.scale,m.top+145*m.scale,4*m.scale,"PAUSE",1,.95f,.85f,1);
    const char* labels[3]={"RESUME","RESTART","TITLE"};
    for(int i=0;i<3;i++) drawMenuButton(hud,m.pause(i),labels[i],app.menuSelection==i,false);
    hud.drawText(m.left+235*m.scale,m.top+445*m.scale,1.2f*m.scale,"ARROWS / STICK   ENTER / A: SELECT",.7f,.77f,.82f,1);
    hud.flushHud();
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
        std::snprintf(line, sizeof(line), app.track.uphill ? "UPHILL" : "DOWNHILL");
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

    const float raceLength=app.track.raceFinishS-app.track.raceStartS;
    int percent = static_cast<int>((app.car.ground.s-app.track.raceStartS) / std::max(1.0f,raceLength) * 100.0f);
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
    if (app.camera.mode == kHoodCameraMode) camName = "HOOD";
    hud.drawText(float(hud.width) - 140, 22, 2.0f, camName, 0.8f, 0.84f, 0.88f, 1);
    std::snprintf(line, sizeof(line), "%d FPS", static_cast<int>(std::lround(app.fps)));
    hud.drawText(float(hud.width) - 140, 52, 2.0f, line, 0.8f, 0.84f, 0.88f, 1);

    if (app.showHelp) {
        const float y=float(hud.height)-92;
        hud.addQuad(12,y-8,std::min(float(hud.width)-12,620.f),y+65,.02f,.03f,.05f,.72f);
        hud.drawText(20,y,1.5f,"KEYS  W/S: GAS/BRAKE  A/D: STEER  SPACE: DRIFT",.95f,.95f,.95f,1);
        hud.drawText(20,y+20,1.5f,"PAD   RT/LT: GAS/BRAKE  STICK: STEER  A: DRIFT",.80f,.85f,.90f,1);
        hud.drawText(20,y+40,1.5f,"C / Y: CAMERA    P / START: PAUSE    H: HIDE HELP",.80f,.85f,.90f,1);
    }
    hud.drawText(float(hud.width) - 250, float(hud.height) - 28, 1.6f, "(C) OSM / GSI", 0.85f, 0.88f, 0.9f, 1);
    if (!app.paused && (app.countdown.number() > 0 || app.countdown.showGo())) {
        const bool go = app.countdown.showGo();
        char countdownText[4];
        if (go) std::snprintf(countdownText, sizeof(countdownText), "GO");
        else std::snprintf(countdownText, sizeof(countdownText), "%d", app.countdown.number());
        const float scale = 9.0f;
        const float centerX = float(hud.width) * 0.5f;
        const float centerY = float(hud.height) * 0.34f;
        const float width = float(std::strlen(countdownText)) * 6.0f * scale;
        hud.addQuad(centerX-width*.5f-18, centerY-12, centerX+width*.5f+18,
                    centerY+7.0f*scale+12, 0.02f, 0.03f, 0.05f, 0.70f);
        hud.drawText(centerX-width*.5f, centerY, scale, countdownText,
                     go ? 0.35f : 1.0f, go ? 1.0f : 0.83f, go ? 0.38f : 0.25f, 1.0f);
    }

    if (app.battle) {
        const float gap=app.car.ground.s-app.rival.car.ground.s;
        std::snprintf(line,sizeof(line),"RIVAL %+.0f M",gap);
        hud.drawText(float(hud.width)-225,82,1.8f,line,0.95f,0.85f,0.5f,1);
        if (app.battleResult!=0) hud.drawText(float(hud.width)*.5f-100,float(hud.height)*.45f,4.0f,
                                              app.battleResult>0?"YOU WIN":"YOU LOSE",1,.95f,.9f,1);
    } else if (app.race.finished) {
        hud.drawText(float(hud.width)*.5f-105,float(hud.height)*.45f,5.0f,"FINISH",1,.95f,.9f,1);
    }
    drawMinimap(app);
    hud.flushHud();
}

// Four road samples follow the vehicle yaw and use the current road layer at crossings.
void sampleWheelRoad(const App& app, const Car& car, float road[4]) {
    const float sn = std::sin(car.yaw), cs = std::cos(car.yaw);
    const float axleZ[2] = {1.42f, -1.38f};
    const float preferS = car.ground.found ? car.ground.s : -1.0f;
    const float edge = app.track.halfWidth + app.track.curbWidth;
    for (int axle=0; axle<2; ++axle) for (int side=0; side<2; ++side) {
        const float x = (side == 0 ? -0.82f : 0.82f);
        TrackPose p = app.track.query(car.x + cs*x + sn*axleZ[axle],
                                      car.z - sn*x + cs*axleZ[axle], car.y, preferS);
        road[axle*2+side] = p.found ? p.centerY + clampf(p.lateral,-edge,edge)*p.cross : car.y;
    }
}

void drawRival(App& app) {
    if (app.battle && !app.titleScreen && rivalNear(app,300.0f)) {
        const Car& rival=app.rival.car;
        const float rivalPitch=app.rivalPitch, rivalRoll=app.rivalRoll;
        const Mat4 rivalModel=carModelMatrix(rival,rivalPitch,rivalRoll);
        app.renderer.draw(app.rivalBody,rivalModel);
        if (app.rival.braking) app.renderer.drawEmissive(app.brakeLights,rivalModel);
        if (app.night) app.renderer.drawEmissive(app.headlights,rivalModel);

        for(int axle=0;axle<2;axle++)for(int side=0;side<2;side++) {
            Mat4 wheel=roadWheelMatrix(rival.x,rival.z,rival.yaw,app.rivalWheelRoad,axle,side,
                                       axle==0?rival.steerInput*.50f:0.0f,rival.wheelSpin);
            app.renderer.draw(app.wheel,wheel);
        }
    }
}

void setRivalLighting(App& app) {
    const Car& r=app.rival.car;
    const float p=app.rivalPitch, roll=app.rivalRoll;
    const Vec3 o=carPointWorld(r,p,roll,{0,0,0});
    const Vec3 f=normalize(carPointWorld(r,p,roll,{0,0,1})-o);
    app.renderer.setRivalLighting(app.battle && !app.titleScreen && rivalNear(app,300.0f),app.rival.braking,
        carPointWorld(r,p,roll,{-.52f,.60f,2.18f}),carPointWorld(r,p,roll,{.52f,.60f,2.18f}),
        f,carPointWorld(r,p,roll,{0,.70f,-2.05f}));
}

void drawFlares(App& app,Vec3 camera,Vec3 look,Vec3 up,const Mat4& vp,float playerPitch,float playerRoll) {
    app.flareVerts.clear();app.flareIndices.clear();
    const Vec3 forward=normalize(look-camera), right=normalize(cross(forward,up)), viewUp=cross(right,forward);
    auto vehicle=[&](const Car& c,float p,float r,bool headlights,bool brake) {
        Vec3 origin=carPointWorld(c,p,r,{0,0,0});
        Vec3 dir=normalize(carPointWorld(c,p,r,{0,0,1})-origin);
        for(float side : {-1.0f,1.0f}) {
            if(headlights) {
                Vec3 point=carPointWorld(c,p,r,{side*.52f,.57f,2.22f});
                if(length(camera-point)>1.2f) appendFlare(app.flareVerts,app.flareIndices,point,dir,camera,right,viewUp,{1,.90f,.62f},.9f,.6f);
            }
            if(brake) {
                Vec3 point=carPointWorld(c,p,r,{side*.535f,.70f,-2.08f});
                if(length(camera-point)>1.2f) appendFlare(app.flareVerts,app.flareIndices,point,dir*-1.0f,camera,right,viewUp,{1,.10f,.04f},.50f,.55f);
            }
        }
    };
    vehicle(app.car,playerPitch,playerRoll,app.night,app.braking);
    if(app.battle && !app.titleScreen && rivalNear(app,300.0f)) vehicle(app.rival.car,app.rivalPitch,app.rivalRoll,app.night,app.rival.braking);
    app.renderer.drawGlow(app.flareVerts.data(),int(app.flareVerts.size()/7),app.flareIndices.data(),int(app.flareIndices.size()),vp);
}

void frame(void* userdata) {
    App& app = *static_cast<App*>(userdata);
    SDL_Event event;
    bool reset = false, activate = false, start = false, togglePause = false;
    int dx = 0, dy = 0;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) app.running = false;
        else if (event.type == SDL_KEYDOWN && !event.key.repeat) {
            SDL_Scancode code = event.key.keysym.scancode;
            if (app.titleScreen || app.paused) {
                if (code == SDL_SCANCODE_UP || code == SDL_SCANCODE_W) dy=-1;
                else if (code == SDL_SCANCODE_DOWN || code == SDL_SCANCODE_S) dy=1;
                else if (code == SDL_SCANCODE_LEFT || code == SDL_SCANCODE_A) dx=-1;
                else if (code == SDL_SCANCODE_RIGHT || code == SDL_SCANCODE_D) dx=1;
                else if (code == SDL_SCANCODE_RETURN || code == SDL_SCANCODE_KP_ENTER) activate=true;
                else if ((code == SDL_SCANCODE_ESCAPE || code == SDL_SCANCODE_P) && app.paused) togglePause=true;
            } else {
                if (code == SDL_SCANCODE_ESCAPE || code == SDL_SCANCODE_P) togglePause=true;
                else if (code == SDL_SCANCODE_R) reset=true;
                else if (code == SDL_SCANCODE_C) app.camera.mode=(app.camera.mode+1)%kCameraModeCount;
                else if (code == SDL_SCANCODE_H) app.showHelp=!app.showHelp;
            }
        } else if ((event.type == SDL_MOUSEMOTION || event.type == SDL_MOUSEBUTTONDOWN) && (app.titleScreen || app.paused)) {
            int windowW=0,windowH=0; SDL_GetWindowSize(app.window,&windowW,&windowH);
            const int mouseX=event.type==SDL_MOUSEMOTION?event.motion.x:event.button.x;
            const int mouseY=event.type==SDL_MOUSEMOTION?event.motion.y:event.button.y;
            MenuLayout layout(float(app.renderer.width),float(app.renderer.height));
            const float px=float(mouseX)*float(app.renderer.width)/float(std::max(1,windowW));
            const float py=float(mouseY)*float(app.renderer.height)/float(std::max(1,windowH));
            int hit=layout.hit(app.titleScreen,px,py);
            if(hit>=0) {
                app.menuSelection=hit;
                if(event.type==SDL_MOUSEBUTTONDOWN && event.button.button==SDL_BUTTON_LEFT) activate=true;
            }
        } else if (event.type == SDL_CONTROLLERDEVICEADDED) openPad(app);
        else if (event.type == SDL_CONTROLLERDEVICEREMOVED && app.pad &&
                 event.cdevice.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(app.pad))) {
            SDL_GameControllerClose(app.pad); app.pad=nullptr;
        }
#ifndef __EMSCRIPTEN__
        else if (event.type == SDL_CONTROLLERBUTTONDOWN && app.titleScreen && event.cbutton.button==SDL_CONTROLLER_BUTTON_START) start=true;
        else if (event.type == SDL_CONTROLLERBUTTONDOWN && !app.titleScreen && !app.paused) {
            if(event.cbutton.button==SDL_CONTROLLER_BUTTON_BACK) reset=true;
            else if(event.cbutton.button==SDL_CONTROLLER_BUTTON_START) togglePause=true;
            else if(event.cbutton.button==SDL_CONTROLLER_BUTTON_Y) app.camera.mode=(app.camera.mode+1)%kCameraModeCount;
        } else if (event.type == SDL_CONTROLLERBUTTONDOWN && app.paused && event.cbutton.button==SDL_CONTROLLER_BUTTON_START) togglePause=true;
#endif
    }
    PadPulse padPulse;
    DriveInput input = readDrive(app.pad, padPulse);
#ifndef __EMSCRIPTEN__
    if(app.pad) {
        float ax=stickAxis(app.pad,SDL_CONTROLLER_AXIS_LEFTX), ay=stickAxis(app.pad,SDL_CONTROLLER_AXIS_LEFTY);
        padPulse.menuUp=ay<-.55f || SDL_GameControllerGetButton(app.pad,SDL_CONTROLLER_BUTTON_DPAD_UP);
        padPulse.menuDown=ay>.55f || SDL_GameControllerGetButton(app.pad,SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        padPulse.menuLeft=ax<-.55f || SDL_GameControllerGetButton(app.pad,SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        padPulse.menuRight=ax>.55f || SDL_GameControllerGetButton(app.pad,SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
        bool confirm=SDL_GameControllerGetButton(app.pad,SDL_CONTROLLER_BUTTON_A);
        padPulse.start=confirm && !app.menuPadConfirm; app.menuPadConfirm=confirm;
    } else app.menuPadConfirm=false;
#endif
    if (app.titleScreen || app.paused) {
        if(padPulse.menuUp && !app.menuPadUp) dy=-1;
        if(padPulse.menuDown && !app.menuPadDown) dy=1;
        if(padPulse.menuLeft && !app.menuPadLeft) dx=-1;
        if(padPulse.menuRight && !app.menuPadRight) dx=1;
        activate=activate || padPulse.start;
        if(padPulse.pause && app.paused) togglePause=true;
        if(padPulse.pause && app.titleScreen) start=true;
        app.menuSelection=menuMove(app.titleScreen,app.menuSelection,dx,dy,app.night,app.uphill,app.battle,app.difficulty);
    } else {
        if(padPulse.reset) reset=true;
        if(padPulse.pause) togglePause=true;
        if(padPulse.camera) app.camera.mode=(app.camera.mode+1)%kCameraModeCount;
    }
    app.menuPadUp=padPulse.menuUp; app.menuPadDown=padPulse.menuDown;
    app.menuPadLeft=padPulse.menuLeft; app.menuPadRight=padPulse.menuRight;
    if (togglePause && !app.titleScreen) {
        app.paused=!app.paused;
        if(app.paused) app.menuSelection=0;
        activate=false;
    }
    if (activate && app.titleScreen) {
        if(app.menuSelection==0) app.night=false;
        else if(app.menuSelection==1) app.night=true;
        else if(app.menuSelection==2) app.uphill=false;
        else if(app.menuSelection==3) { if(!app.track.closed && app.track.touge) app.uphill=true; }
        else if(app.menuSelection==4) app.battle=false;
        else if(app.menuSelection==5) app.battle=true;
        else if(app.menuSelection>=6 && app.menuSelection<=8) app.difficulty=app.menuSelection-6;
        else start=true;
    } else if (activate && app.paused) {
        if(app.menuSelection==0) app.paused=false;
        else if(app.menuSelection==1) { app.paused=false; reset=true; }
        else { app.paused=false; app.titleScreen=true; app.menuSelection=9; reset=true; }
    }
    if (start && app.titleScreen) {
        app.track.setUphill(app.uphill);
        app.titleScreen=false; reset=true;
    }
    if (reset) resetRace(app);

    Uint64 now = SDL_GetPerformanceCounter();
    double elapsed = double(now - app.prevCounter) / double(SDL_GetPerformanceFrequency());
    app.prevCounter = now;
    if (elapsed > 0.0001) {
        float inst = static_cast<float>(1.0 / elapsed);
        app.fps = (app.fps <= 0) ? inst : app.fps + (inst - app.fps) * 0.05f;
    }
    if (elapsed > kMaxFrameSec) elapsed = kMaxFrameSec;
    if (elapsed < 0) elapsed = 0;
    if (reset) elapsed = 0; // Do not consume time from before the reset.

    if (app.paused || app.titleScreen) {
        app.accumulator = 0;
    } else {
        app.accumulator += elapsed;
        int guard = 0;
        while (app.accumulator >= kFixedStepSec && guard < 8) {
            if (!app.countdown.step(kFixedStepSec)) {
                app.accumulator -= kFixedStepSec;
                ++guard;
                continue;
            }
            app.car.update(float(kFixedStepSec) * kDrivingPace, input, app.track);
            recordTyreMarks(app,app.car,app.skidMarks,brakeLightsActive(input,app.car.speed),
                            input.handbrake?1.0f:input.brake);
            // Emit on every physics step so brief collisions are not lost between frames.
            const float effectDt = float(kFixedStepSec);
            Vec3 carVel{app.car.velX, 0, app.car.velZ};
            if (app.car.wallContact) {
                if (app.car.wallNewContact) {
                    const float direction = RumbleState::wallDirection(app.car.yaw,
                        app.car.wallRightX, app.car.wallRightZ, app.car.wallSide);
                    app.rumble.collision(app.car.wallImpact, direction);
                }
                Vec3 inward{-app.car.wallRightX * app.car.wallSide, 0,
                            -app.car.wallRightZ * app.car.wallSide};
                Vec3 point{app.car.wallContactX, app.car.y + 0.55f, app.car.wallContactZ};
                app.surfaceEffects.emitSparks(point, carVel, inward, app.car.wallImpact,
                                              app.car.wallAlongSpeed, effectDt, app.car.wallNewContact);
            }
            if (app.car.ground.found && !app.car.onAsphalt && app.car.groundSpeed() > 2.0f) {
                Vec3 fwd{std::sin(app.car.yaw), 0, std::cos(app.car.yaw)};
                Vec3 right{fwd.z, 0, -fwd.x};
                Vec3 rear{app.car.x, app.car.y + 0.12f, app.car.z};
                rear = rear - fwd * 1.38f;
                app.surfaceEffects.emitMud(rear - right * 0.82f, rear + right * 0.82f,
                                           carVel, app.car.groundSpeed(), effectDt);
            }
            if (app.battle) {
                app.rival.update(app.track, float(kFixedStepSec)*kDrivingPace, app.car.ground.s, app.car.groundSpeed());
                const VehicleContact contact=resolveVehicleContact(app.car,app.rival.car);
                if (contact.hit) {
                    app.rival.acceptContact(app.track);
                    TrackPose p=app.track.query(app.car.x,app.car.z,app.car.y,app.car.ground.s);
                    if(p.found) {
                        app.car.ground=p;
                        const float edge=app.track.halfWidth+app.track.curbWidth;
                        app.car.y=p.centerY+clampf(p.lateral,-edge,edge)*p.cross;
                    }
                    const float cs=std::cos(app.car.yaw), sn=std::sin(app.car.yaw);
                    app.car.velForward=app.car.velX*sn+app.car.velZ*cs;
                    app.car.velLateral=app.car.velX*cs-app.car.velZ*sn;
                    app.car.speed=app.car.velForward;
                    if (!app.rivalContact) {
                        app.rumble.collision(contact.impact,contact.normal.x*cs-contact.normal.z*sn);
                        app.surfaceEffects.emitSparks(contact.point,{app.car.velX,0,app.car.velZ},
                                                      contact.normal*-1.0f,contact.impact,0.0f,
                                                      float(kFixedStepSec),true);
                    }
                }
                app.rivalContact=contact.hit;
                if(rivalNear(app,250.0f))
                    recordTyreMarks(app,app.rival.car,app.rivalSkids,app.rival.braking,app.rival.braking?1.0f:0.0f);
                else for(int i=0;i<2;++i) app.rivalSkids.sample(i,false,{},0);
            }
            tickRace(app.race, app.car.ground.s, app.track.length, float(kFixedStepSec), app.track.closed,
                     app.track.raceStartS, app.track.raceFinishS);
            if (app.battle && app.battleResult==0) {
                if (app.race.finished || (app.track.closed && app.race.completed>0)) app.battleResult=1;
                else if (app.rival.finished) app.battleResult=-1;
            }
            app.accumulator -= kFixedStepSec;
            ++guard;
        }
    }
    const float wallVibrationDirection = app.car.wallContact
        ? RumbleState::wallDirection(app.car.yaw, app.car.wallRightX,
                                     app.car.wallRightZ, app.car.wallSide) : 0.0f;
    const RumbleCommand vibration = app.rumble.update(float(elapsed), app.paused || app.titleScreen || !app.running,
        app.car.ground.found, app.car.onAsphalt, app.car.drifting, app.car.slipAngle,
        app.car.groundSpeed(), app.car.wallContact, wallVibrationDirection);
    sendRumble(app.pad, vibration);
    // 音はフレーム単位でよい。ポーズ中はアイドル＋無音になる。
    // 回転数だけは EngineVoice（ギア状態つき）で作って上書きする。速度への単純な
    // 写像だとギア境界でピッチが跳ねたり往復したりするため。
    SoundState soundState = makeSoundState(
        app.car.speed, app.car.groundSpeed(), input.throttle, input.brake,
        input.handbrake, app.car.drifting, std::fabs(app.car.slipAngle), app.paused || app.titleScreen,
        app.countdown.remaining <= 0.0);
    soundState.rpm01 = app.engineVoice.update(std::fabs(app.car.speed), input.throttle,
                                              app.car.drifting, app.paused, float(elapsed));
    app.sound.update(soundState);
    // 前進中（または停止中）にブレーキを踏んでいる間だけランプを点灯する。
    // 後退中の通常ブレーキは後退加速。サイドブレーキは速度に関係なく点灯。
    app.braking = brakeLightsActive(input, app.car.speed);
    // The hood camera and vehicle mesh use the same pitch and roll.
    float wheelRoad[4];
    sampleWheelRoad(app, app.car, wheelRoad);
    app.suspensionVisual.update(wheelRoad, (app.paused || app.titleScreen) ? 0.0f : float(elapsed));
    app.car.bodyHeave = app.suspensionVisual.heave(app.car.y);
    float pitch = app.suspensionVisual.pitch() + app.car.bodyPitch;
    float roll = app.suspensionVisual.roll() + app.car.bodyRoll;
    if (app.battle && !app.titleScreen && rivalNear(app,300.0f)) {
        sampleWheelRoad(app,app.rival.car,app.rivalWheelRoad);
        app.rivalSuspension.update(app.rivalWheelRoad,app.paused?0.0f:float(elapsed));
        app.rival.car.bodyHeave=app.rivalSuspension.heave(app.rival.car.y);
        app.rivalPitch=app.rivalSuspension.pitch()+app.rival.car.bodyPitch;
        app.rivalRoll=app.rivalSuspension.roll()+app.rival.car.bodyRoll;
    } else app.rivalSuspension.reset();
    updateCamera(app, pitch, roll, (app.paused || app.titleScreen) ? 0.0f : float(elapsed));

    // ブレーキランプ点灯中は、左右ランプの位置を履歴に残して光の軌跡にする。
    if (!app.paused) {
        if (app.braking && app.car.groundSpeed() > 2.5f) {
            Vec3 lampL = carPointWorld(app.car, pitch, roll, {-0.535f, 0.70f, -2.0f});
            Vec3 lampR = carPointWorld(app.car, pitch, roll, {0.535f, 0.70f, -2.0f});
            app.brakeTrail.record(lampL, lampR);
        }
        app.brakeTrail.update(float(elapsed));
        if (app.battle && !app.titleScreen && rivalNear(app,250.0f) && app.rival.braking && app.rival.car.groundSpeed()>2.5f) {
            const Car& r=app.rival.car;
            app.rivalTrail.record(carPointWorld(r,app.rivalPitch,app.rivalRoll,{-.535f,.70f,-2.0f}),
                                  carPointWorld(r,app.rivalPitch,app.rivalRoll,{ .535f,.70f,-2.0f}));
        }
        app.rivalTrail.update(float(elapsed));
    }

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
                           float(elapsed), 0.80f, 160);
        }
        app.smoke.update(float(elapsed));
        if (app.battle && !app.titleScreen && rivalNear(app,200.0f) && app.rival.car.drifting) {
            const Car& r=app.rival.car;
            Vec3 f{std::sin(r.yaw),0,std::cos(r.yaw)}, right{f.z,0,-f.x};
            Vec3 base{r.x,r.y+0.12f,r.z}; base=base-f*1.38f;
            app.rivalSmoke.emit(base-right*.82f,base+right*.82f,{r.velX,0,r.velZ},
                                clampf(std::fabs(r.slipAngle)*1.6f,0,1),float(elapsed),0.45f,110);
        }
        app.rivalSmoke.update(float(elapsed));
        app.surfaceEffects.update(float(elapsed));
        app.skidMarks.update(float(elapsed));
        app.rivalSkids.update(float(elapsed));
    }

    int drawableW = 0;
    int drawableH = 0;
    SDL_GL_GetDrawableSize(app.window, &drawableW, &drawableH);
    app.renderer.resize(drawableW, drawableH);
    app.renderer.clearFrame();
    app.renderer.drawSky(app.night);

    float aspect = float(drawableW) / float(drawableH > 0 ? drawableH : 1);
    // スピード感：速いほど視野角を少し広げて、景色の流れを強調する。
    float fovDeg = 62.0f + 12.0f * clampf(app.car.groundSpeed() / (150.0f / 3.6f), 0.0f, 1.0f);
    Mat4 proj = perspective(fovDeg * 3.14159265f / 180.0f, aspect, 0.15f, 4000.0f);
    Mat4 view = lookAt(app.camera.eye, app.camera.look, app.camera.up);
    Mat4 viewProj = mul(proj, view);
    const ViewFrustum effectView(viewProj);
    const Vec3 lampLeft = carPointWorld(app.car, pitch, roll, {-0.52f, 0.60f, 2.18f});
    const Vec3 lampRight = carPointWorld(app.car, pitch, roll, {0.52f, 0.60f, 2.18f});
    const Vec3 lampDirection = normalize(carPointWorld(app.car, pitch, roll, {0,0,1}) - carPointWorld(app.car, pitch, roll, {0,0,0}));
    const Vec3 rearLight = carPointWorld(app.car, pitch, roll, {0, 0.70f, -2.05f});
    const bool reversing = reverseLightsActive(app.car.speed, app.car.brakeHold);
    app.renderer.beginWorld(viewProj, app.camera.eye, app.night, lampLeft, lampRight, lampDirection, rearLight, app.braking, reversing);
    setRivalLighting(app);

    app.renderer.drawCourse(app.course, viewProj);
    // Marks lie just above the road and use ordinary translucent blending.
    app.skidVerts.clear();app.skidIndices.clear();
    app.skidMarks.append(app.camera.eye,app.skidVerts,app.skidIndices,&effectView);
    if (app.battle) app.rivalSkids.append(app.camera.eye,app.skidVerts,app.skidIndices,&effectView);
    app.renderer.drawGlow(app.skidVerts.data(),int(app.skidVerts.size()/7),
                          app.skidIndices.data(),int(app.skidIndices.size()),viewProj,false);
    app.renderer.beginWorld(viewProj, app.camera.eye, app.night, lampLeft, lampRight, lampDirection, rearLight, app.braking, reversing);
    setRivalLighting(app);
    Mat4 shadow = mul(translation(app.car.x, app.car.y + 0.08f, app.car.z), rotationY(app.car.yaw));
    app.renderer.draw(app.shadow, shadow);

    Mat4 model = carModelMatrix(app.car, pitch, roll);
    app.renderer.draw(app.body, model);
    if (app.braking) {
        app.renderer.drawEmissive(app.brakeLights, model);
    }
    if (app.night) app.renderer.drawEmissive(app.headlights, model);
    if (reversing) app.renderer.drawEmissive(app.reverseLights, model);
    // The wheels track the road separately from the sprung body.
    for (int axle = 0; axle < 2; ++axle) {
        for (int side = 0; side < 2; ++side) {
            const float steering = axle == 0 ? app.car.steerInput * 0.50f : 0.0f;
            Mat4 wheel = roadWheelMatrix(app.car.x, app.car.z, app.car.yaw, wheelRoad,
                                         axle, side, steering, app.car.wheelSpin);
            app.renderer.draw(app.wheel, wheel);
        }
    }
    drawRival(app);
    // タイヤ煙は半透明なので、不透明な車体の後にビルボードで重ねる。
    {
        Vec3 camFwd = normalize(app.camera.look - app.camera.eye);
        Vec3 camRight = normalize(cross(camFwd, app.camera.up));
        Vec3 camUp = cross(camRight, camFwd);
        app.smokeVerts.clear();app.smokeIndices.clear();
        app.smoke.appendQuads(camRight,camUp,app.smokeVerts,app.smokeIndices,&effectView);
        if (app.battle && !app.titleScreen)
            app.rivalSmoke.appendQuads(camRight,camUp,app.smokeVerts,app.smokeIndices,&effectView);
        app.renderer.drawSmoke(app.smokeVerts.data(),int(app.smokeVerts.size()/6),
                               app.smokeIndices.data(),int(app.smokeIndices.size()),viewProj);
    }
    // Mud uses normal alpha blending; sparks use the existing additive glow shader.
    {
        Vec3 camFwd = normalize(app.camera.look - app.camera.eye);
        Vec3 camRight = normalize(cross(camFwd, app.camera.up));
        Vec3 camUp = cross(camRight, camFwd);
        for (bool sparks : {false, true}) {
            app.surfaceEffects.buildQuads(camRight, camUp, sparks, app.surfaceVerts, app.surfaceIndices);
            app.renderer.drawGlow(app.surfaceVerts.data(), int(app.surfaceVerts.size() / 7),
                                  app.surfaceIndices.data(), int(app.surfaceIndices.size()), viewProj, sparks);
        }
    }
    // ブレーキランプの光の軌跡は加算グローで煙の後に重ねる。
    {
        app.trailVerts.clear();app.trailIndices.clear();
        app.brakeTrail.appendQuads(app.trailVerts,app.trailIndices);
        if (app.battle && !app.titleScreen) app.rivalTrail.appendQuads(app.trailVerts,app.trailIndices);
        app.renderer.drawGlow(app.trailVerts.data(),int(app.trailVerts.size()/7),
                              app.trailIndices.data(),int(app.trailIndices.size()),viewProj);
    }
    if (!app.titleScreen) drawFlares(app,app.camera.eye,app.camera.look,app.camera.up,viewProj,pitch,roll);
    if (!app.titleScreen) {
        // Render a bounded, horizontally mirrored rear view after the main scene.
        const int mw=std::max(1,std::min(480,int(float(drawableW)*.36f)));
        const int mh=std::max(1,std::min(120,int(float(drawableH)*.15f)));
        const int mx=(drawableW-mw)/2, top=12, my=std::max(0,drawableH-top-mh);
        const Vec3 mirrorEye=carPointWorld(app.car,pitch,roll,{0,1.35f,-2.38f});
        const Vec3 rearPoint=carPointWorld(app.car,pitch,roll,{0,1.35f,-22.38f});
        const Vec3 mirrorUp=normalize(carPointWorld(app.car,pitch,roll,{0,2.35f,-2.38f})-mirrorEye);
        Mat4 mirrorProj=perspective(68.0f*3.14159265f/180.0f,float(mw)/float(mh),.15f,700.0f);
        mirrorProj.m[0]*=-1.0f;
        const Mat4 mirrorVP=mul(mirrorProj,lookAt(mirrorEye,rearPoint,mirrorUp));
        app.renderer.beginMirror(mx,my,mw,mh);
        app.renderer.drawSky(app.night);
        app.renderer.beginWorld(mirrorVP,mirrorEye,app.night,lampLeft,lampRight,lampDirection,rearLight,app.braking,reversing,true);
        setRivalLighting(app);
        app.renderer.drawCourse(app.course,mirrorVP,500.0f);
        drawRival(app);
        drawFlares(app,mirrorEye,rearPoint,mirrorUp,mirrorVP,pitch,roll);
        app.renderer.endMirror();
        Renderer& h=app.renderer; h.beginHud();
        const float x=float(mx), y=float(drawableH-my-mh), w=float(mw), hh=float(mh);
        h.addQuad(x-3,y-3,x+w+3,y,.16f,.18f,.20f,1);
        h.addQuad(x-3,y+hh,x+w+3,y+hh+3,.16f,.18f,.20f,1);
        h.addQuad(x-3,y,x,y+hh,.16f,.18f,.20f,1);
        h.addQuad(x+w,y,x+w+3,y+hh,.16f,.18f,.20f,1);
        h.flushHud();
    }
    if (app.titleScreen) drawTitle(app);
    else { drawHud(app); if (app.paused) drawPauseMenu(app); }
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
    app.course = app.renderer.upload(app.track.visual, true);
    app.track.visual = CpuMesh{}; // GPU owns the uploaded course geometry.
    CpuMesh body;
    CpuMesh brakeLights;
    CpuMesh headlights;
    CpuMesh reverseLights;
    CpuMesh wheel;
    CpuMesh shadow;
    buildCarBody(body);
    buildCarBrakeLights(brakeLights);
    buildCarHeadlights(headlights);
    buildCarReverseLights(reverseLights);
    buildCarWheel(wheel);
    buildCarShadow(shadow);
    app.body = app.renderer.upload(body);
    CpuMesh rivalBody=body;
    for(size_t i=0;i+8<rivalBody.vertices.size();i+=9) {
        if(rivalBody.vertices[i+6]>.7f && rivalBody.vertices[i+7]>.7f && rivalBody.vertices[i+8]>.7f) {
            rivalBody.vertices[i+6]*=.28f; rivalBody.vertices[i+7]*=.52f; rivalBody.vertices[i+8]*=.95f;
        }
    }
    app.rivalBody=app.renderer.upload(rivalBody);
    app.brakeLights = app.renderer.upload(brakeLights);
    app.headlights = app.renderer.upload(headlights);
    app.reverseLights = app.renderer.upload(reverseLights);
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

    sendRumble(app.pad, RumbleCommand{0, 0, false, true});
    if (app.pad) SDL_GameControllerClose(app.pad);
    app.sound.shutdown();
    if (imageOk) IMG_Quit();
    SDL_GL_DeleteContext(app.gl);
    SDL_DestroyWindow(app.window);
    SDL_Quit();
    return 0;
}
