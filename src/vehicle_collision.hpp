#pragma once
#include "car.hpp"
#include <algorithm>
#include <cmath>
struct VehicleContact {
 bool hit=false; Vec3 normal{}, point{}; float impact=0;
};
// Oriented rectangles in the horizontal plane, including car length and width.
inline VehicleContact resolveVehicleContact(Car& a,Car& b) {
 VehicleContact result;
 if(std::fabs(a.y-b.y)>2.0f) return result;
 Vec3 ar{std::cos(a.yaw),0,-std::sin(a.yaw)},af{std::sin(a.yaw),0,std::cos(a.yaw)};
 Vec3 br{std::cos(b.yaw),0,-std::sin(b.yaw)},bf{std::sin(b.yaw),0,std::cos(b.yaw)};
 Vec3 delta{b.x-a.x,0,b.z-a.z};float overlap=1e9f;Vec3 normal{};
 for(Vec3 axis:{ar,af,br,bf}) {
  float ra=.95f*std::fabs(dot(ar,axis))+2.2f*std::fabs(dot(af,axis));
  float rb=.95f*std::fabs(dot(br,axis))+2.2f*std::fabs(dot(bf,axis));
  float d=dot(delta,axis), depth=ra+rb-std::fabs(d);
  if(depth<=0)return result;
  if(depth<overlap){overlap=depth;normal=axis*(d>=0?1.0f:-1.0f);}
 }
 result.hit=true;result.normal=normal;
 result.point={(a.x+b.x)*.5f,(a.y+b.y)*.5f+.55f,(a.z+b.z)*.5f};
 const Vec3 relative{b.velX-a.velX,0,b.velZ-a.velZ};
 const float closing=dot(relative,normal); result.impact=std::max(0.0f,-closing);
 const float move=(overlap+.002f)*.5f;
 a.x-=normal.x*move;a.z-=normal.z*move;
 b.x+=normal.x*move;b.z+=normal.z*move;
 if(closing<0) {
  const float impulse=-closing*.55f;
  a.velX-=normal.x*impulse;a.velZ-=normal.z*impulse;
  b.velX+=normal.x*impulse;b.velZ+=normal.z*impulse;
 }
 return result;
}
