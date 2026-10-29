#pragma once
#include <algorithm>
#include <cmath>

struct RumbleCommand {
    float low = 0;   // Strong/low-frequency motor, 0..1
    float high = 0;  // Weak/high-frequency motor, 0..1
    bool send = false;
    bool stop = false;
};

// Pure feedback policy; platform-specific code sends the short motor pulses.
struct RumbleState {
    float impact = 0;
    float impactDirection = 0; // -1 left, +1 right in car coordinates
    float timeSinceSend = 1;
    bool wasPlaying = false;
    bool newImpact = false;
    static float clamp01(float v) { return std::max(0.0f, std::min(v, 1.0f)); }
    static float clampDirection(float v) { return std::max(-1.0f, std::min(v, 1.0f)); }
    // Track-side normal projected onto the car's right vector. A head-on hit
    // has no reliable left/right side and therefore stays centred.
    static float wallDirection(float yaw, float trackRightX, float trackRightZ, float wallSide) {
        return clampDirection(wallSide * (trackRightX * std::cos(yaw) - trackRightZ * std::sin(yaw)));
    }
    static float leftGain(float direction) { return 1.0f - 0.65f * std::max(0.0f, clampDirection(direction)); }
    static float rightGain(float direction) { return 1.0f - 0.65f * std::max(0.0f, -clampDirection(direction)); }
    void collision(float speed, float direction = 0) {
        if (speed < 1.0f) return;
        const float amount = clamp01(0.25f + speed / 18.0f);
        if (amount >= impact) { impact = amount; impactDirection = clampDirection(direction); }
        newImpact = true;
    }
    void clear() { impact = 0; impactDirection = 0; timeSinceSend = 1; newImpact = false; }
    RumbleCommand update(float dt, bool paused, bool grounded, bool asphalt,
                         bool drifting, float slip, float speed, bool touchingWall, float wallSide = 0) {
        RumbleCommand out;
        dt = std::max(0.0f, std::min(dt, 0.25f));
        if (paused) {
            out.stop = wasPlaying;
            wasPlaying = false;
            clear();
            return out;
        }
        const float moving = clamp01((speed - 2.0f) / 18.0f);
        const float drift = drifting ? clamp01((std::fabs(slip) - 0.12f) / 0.50f) * moving : 0.0f;
        const float grass = grounded && !asphalt ? moving : 0.0f;
        const float wall = touchingWall ? moving : 0.0f;
        // Rightward turns have negative slip; use the car's direction, not world axes.
        const float driftDirection = clampDirection(-slip / 0.30f);
        // SDL's low motor is usually on the left and its high motor on the right.
        // Web Gamepad exposes low/high frequencies, so physical location can vary.
        out.low = clamp01(std::max({impact * 0.80f * leftGain(impactDirection), grass * 0.22f,
                                    wall * 0.26f * leftGain(wallSide), drift * 0.12f * leftGain(driftDirection)}));
        out.high = clamp01(std::max({impact * rightGain(impactDirection), grass * 0.30f,
                                     wall * 0.38f * rightGain(wallSide), drift * 0.19f * rightGain(driftDirection)}));
        timeSinceSend += dt;
        if (out.low > 0.01f || out.high > 0.01f) {
            out.send = newImpact || timeSinceSend >= 0.09f;
            if (out.send) { timeSinceSend = 0; wasPlaying = true; }
        } else if (wasPlaying) {
            out.stop = true;
            wasPlaying = false;
        }
        newImpact = false;
        impact = std::max(0.0f, impact - dt * 3.5f);
        return out;
    }
};
