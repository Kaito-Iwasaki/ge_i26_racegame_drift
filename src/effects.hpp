#pragma once
#include "geom.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

// Bounded colored particles, independent of SDL and OpenGL.
struct SurfaceEffects {
    struct Particle { Vec3 pos, vel, color; float age, life, size; bool spark; };
    std::vector<Particle> particles;
    float sparkAccumulator=0, mudAccumulator=0;
    std::uint32_t randomState=0x67a194d3u;
    static constexpr size_t kMaxParticles=320;
    float random() { randomState=randomState*1664525u+1013904223u; return float(randomState>>8)/16777216.0f; }
    void clear() { particles.clear(); sparkAccumulator=0; mudAccumulator=0; }
    void emitSparks(Vec3 point, Vec3 velocity, Vec3 inward, float impact, float alongSpeed, float dt, bool newContact) {
        if(dt<=0 || (impact<0.4f && alongSpeed<2.0f)) return;
        float intensity=clampf(impact/8.0f+alongSpeed/30.0f,0.0f,1.0f);
        sparkAccumulator+=dt*(22.0f+85.0f*intensity);
        if (newContact && impact > 1.0f) sparkAccumulator += 4.0f + 5.0f * intensity;
        while(sparkAccumulator>=1.0f) {
            sparkAccumulator-=1.0f;
            if(particles.size()>=kMaxParticles) break;
            Particle p; p.pos=point+Vec3{(random()-.5f)*.35f,random()*.25f,(random()-.5f)*.35f};
            p.vel=velocity*.15f+inward*(1.0f+random()*3.0f)+Vec3{(random()-.5f)*3,1.5f+random()*3,(random()-.5f)*3};
            p.color={1.0f,0.40f+random()*.45f,0.05f}; p.age=0; p.life=.18f+random()*.30f; p.size=.055f+random()*.05f; p.spark=true; particles.push_back(p);
        }
    }
    void emitMud(Vec3 left, Vec3 right, Vec3 velocity, float speed, float dt) {
        if(dt<=0 || speed<2.0f) return;
        mudAccumulator+=dt*(12.0f+32.0f*clampf(speed/25.0f,0.0f,1.0f));
        while(mudAccumulator>=1.0f) {
            mudAccumulator-=1.0f;
            if(particles.size()>=kMaxParticles) break;
            Particle p; p.pos=(random()<.5f?left:right)+Vec3{(random()-.5f)*.3f,0,(random()-.5f)*.3f};
            p.vel=velocity*.24f+Vec3{(random()-.5f)*2.4f,1.3f+random()*2.8f,(random()-.5f)*2.4f};
            float shade=.75f+random()*.3f;p.color={.30f*shade,.19f*shade,.09f*shade};p.age=0;p.life=.45f+random()*.5f;p.size=.11f+random()*.13f;p.spark=false;particles.push_back(p);
        }
    }
    void update(float dt) {
        dt=std::max(dt,0.0f);
        for(auto& p:particles){p.age+=dt;p.vel.y-=(p.spark?7.0f:9.8f)*dt;p.pos=p.pos+p.vel*dt;}
        for(size_t i=0;i<particles.size();) { if(particles[i].age>=particles[i].life){particles[i]=particles.back();particles.pop_back();}else ++i; }
    }
    // Same [position, rgb, alpha] vertex format as the glow renderer.
    void buildQuads(Vec3 cameraRight,Vec3 cameraUp,bool sparks,std::vector<float>& verts,std::vector<unsigned>& ids) const {
        verts.clear();ids.clear();
        for(const auto& p:particles){if(p.spark!=sparks)continue;
            float alpha=clampf(1.0f-p.age/p.life,0.0f,1.0f)*(p.spark?1.0f:.85f);
            Vec3 r=cameraRight*p.size;Vec3 u=cameraUp*(p.size*(p.spark?2.4f:1.0f));
            Vec3 q[4]={p.pos-r-u,p.pos+r-u,p.pos+r+u,p.pos-r+u};
            unsigned base=static_cast<unsigned>(verts.size()/7);
            for(const auto& v:q)verts.insert(verts.end(),{v.x,v.y,v.z,p.color.x,p.color.y,p.color.z,alpha});
            ids.insert(ids.end(),{base,base+1,base+2,base,base+2,base+3});
        }
    }
};
