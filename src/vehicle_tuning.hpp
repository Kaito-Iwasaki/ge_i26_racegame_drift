#pragma once
#include <algorithm>
namespace VehicleTuning {
constexpr float maxSpeed=160.0f/3.6f;
constexpr float acceleration=15.0f;
constexpr float brake=12.0f;
constexpr float drag=0.00028f;
constexpr float rolling=0.9f;
inline float engineAcceleration(float speed,float drift) {
 const float ratio=std::max(0.0f,std::min(1.0f,speed/maxSpeed));
 return acceleration*(1.0f-ratio*ratio)*(1.0f+0.3f*drift)*(1.0f-0.15f*drift);
}
}
