#pragma once
#include "car.hpp"

constexpr int kCameraModeCount = 4;
constexpr int kHoodCameraMode = 3;

struct CameraPose {
    Vec3 eye;
    Vec3 look;
    Vec3 up;
};

inline CameraPose carCameraPose(const Car& car, int mode, float pitch, float roll) {
    if (mode == kHoodCameraMode) {
        // Mounted just above the base of the windshield: the bonnet remains visible.
        const Vec3 localEye{0.0f, 1.28f, 0.72f};
        const Vec3 eye = carPointWorld(car, pitch, roll, localEye);
        const Vec3 look = carPointWorld(car, pitch, roll, {0.0f, 1.00f, 25.0f});
        const Vec3 upPoint = carPointWorld(car, pitch, roll, {0.0f, 2.28f, 0.72f});
        return {eye, look, normalize(upPoint-eye)};
    }
    const float back[3] = {8.6f, 4.1f, 16.0f};
    const float height[3] = {3.3f, 1.55f, 7.2f};
    const float ahead[3] = {9.0f, 6.0f, 14.0f};
    const int index = mode >= 0 && mode < 3 ? mode : 0;
    const Vec3 pos{car.x,car.y,car.z};
    const Vec3 forward{std::sin(car.yaw),0.0f,std::cos(car.yaw)};
    return {pos-forward*back[index]+Vec3{0,height[index],0},
            pos+forward*ahead[index]+Vec3{0,1.15f,0}, {0,1,0}};
}
