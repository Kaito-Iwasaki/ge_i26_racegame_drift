#pragma once
#include "geom.hpp"
#include <vector>
#include <cmath>
// Add an occlusion-tested camera-facing glow with a soft radial falloff.
inline void appendFlare(std::vector<float>& v,std::vector<unsigned>& ids,Vec3 pos,Vec3 normal,
                        Vec3 camera,Vec3 right,Vec3 up,Vec3 color,float radius,float strength) {
    Vec3 delta=camera-pos; float dist=length(delta);
    if(dist<0.05f || dot(normal,delta)*(1.0f/dist)<=0.08f) return;
    const float facing=clampf((dot(normal,delta)/dist-.08f)/.45f,0,1);
    const float fade=clampf((180.0f-dist)/100.0f,0,1)*facing*strength;
    if(fade<=0)return;
    auto push=[&](Vec3 p,float a){v.insert(v.end(),{p.x,p.y,p.z,color.x,color.y,color.z,a});};
    const unsigned base=unsigned(v.size()/7);push(pos,fade);
    constexpr int segments=12;
    for(int i=0;i<segments;i++){float a=6.2831853f*float(i)/segments;push(pos+right*(std::cos(a)*radius)+up*(std::sin(a)*radius),0);}
    for(int i=0;i<segments;i++)ids.insert(ids.end(),{base,base+1+unsigned(i),base+1+unsigned((i+1)%segments)});
}
