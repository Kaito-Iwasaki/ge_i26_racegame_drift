#pragma once
#include "car.hpp"
#include <algorithm>

constexpr int kCameraModeCount = 2;
constexpr int kHoodCameraMode = 1;

struct CameraPose {
    Vec3 eye;
    Vec3 look;
    Vec3 up;
};

inline CameraPose carCameraPose(const Car& car, int mode, float pitch, float roll) {
    if (mode == kHoodCameraMode) {
        // Mounted just ahead of the front bumper so the bonnet stays out of view.
        const Vec3 localEye{0.0f, 1.28f, 2.50f};
        const Vec3 eye = carPointWorld(car, pitch, roll, localEye);
        const Vec3 look = carPointWorld(car, pitch, roll, {0.0f, 1.00f, 26.78f});
        const Vec3 upPoint = carPointWorld(car, pitch, roll, {0.0f, 2.28f, 2.50f});
        return {eye, look, normalize(upPoint-eye)};
    }
    const Vec3 pos{car.x,car.y,car.z};
    const Vec3 forward{std::sin(car.yaw),0.0f,std::cos(car.yaw)};
    return {pos-forward*8.6f+Vec3{0,3.3f,0},
            pos+forward*9.0f+Vec3{0,1.15f,0}, {0,1,0}};
}

// Blend in car-relative camera space, so both endpoints follow a moving/turning car.
struct CameraBlend {
    int targetMode = 0;
    float start = 0.0f;
    float weight = 0.0f;
    float progress = 1.0f;
    void update(int mode, float dt) {
        if (mode != targetMode) {
            targetMode = mode;
            start = weight;
            progress = 0.0f;
        }
        progress = clampf(progress + std::max(0.0f, dt), 0.0f, 1.0f);
        const float ease = progress*progress*(3.0f-2.0f*progress);
        weight = start + ((targetMode == kHoodCameraMode ? 1.0f : 0.0f) - start)*ease;
    }
    CameraPose apply(const CameraPose& chase, const CameraPose& hood) const {
        const float t=weight;
        return {chase.eye*(1-t)+hood.eye*t,
                chase.look*(1-t)+hood.look*t,
                normalize(chase.up*(1-t)+hood.up*t)};
    }
};
