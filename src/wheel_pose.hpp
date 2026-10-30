#pragma once
#include "geom.hpp"

// Wheel orientation follows the road, independent of sprung body roll and pitch.
inline Mat4 roadWheelMatrix(float carX, float carZ, float yaw, const float road[4],
                            int axle, int side, float steering, float spin) {
    const float wheelX = (side == 0 ? -0.82f : 0.82f);
    const float wheelZ = axle == 0 ? 1.42f : -1.38f;
    const float sinYaw = std::sin(yaw), cosYaw = std::cos(yaw);
    const float roadPitch = std::atan2((road[0]+road[1]-road[2]-road[3])*0.5f, 2.80f);
    const float roadRoll = std::atan2((road[1]+road[3]-road[0]-road[2])*0.5f, 1.64f);
    const Vec3 forward = normalize(Vec3{sinYaw*std::cos(roadPitch), std::sin(roadPitch), cosYaw*std::cos(roadPitch)});
    const Vec3 right0{cosYaw, 0, -sinYaw};
    const Vec3 up0 = normalize(cross(forward, right0));
    const Vec3 right = normalize(right0*std::cos(roadRoll) + up0*std::sin(roadRoll));
    const Vec3 up = normalize(cross(forward, right));
    const Vec3 center{carX + cosYaw*wheelX + sinYaw*wheelZ,
                      road[axle*2+side] + 0.34f*up.y,
                      carZ - sinYaw*wheelX + cosYaw*wheelZ};
    return mul(basis(center,right,up,forward),mul(rotationY(steering),rotationX(spin)));
}
