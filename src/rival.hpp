#pragma once
#include "car.hpp"
#include <algorithm>
#include <cmath>

// A route-following rival: progress and drift attitude are animated independently
// of the player's handling model. Road samples are looked up by distance.
struct Rival {
    Car car;
    float s=0, speed=0, drift=0, distanceTravelled=0;
    bool finished=false;
    bool braking=false;
    int level=1;
    struct RoutePose {float x,y,z,cross,yaw;};
    static float normalizedS(const Track& t,float s) {
        if(t.closed && t.length>0) { s=std::fmod(s,t.length); if(s<0)s+=t.length; return s; }
        return clampf(s,0,t.length);
    }
    static RoutePose pose(const Track& t,float distance) {
        const float s=normalizedS(t,distance);
        auto it=std::upper_bound(t.samples.begin(),t.samples.end(),s,
            [](float v,const TrackSample& p){return v<p.s;});
        size_t i=it==t.samples.begin()?0:size_t(it-t.samples.begin()-1);
        size_t j=i+1<t.samples.size()?i+1:(t.closed?0:i);
        const auto& a=t.samples[i]; const auto& b=t.samples[j];
        float dx=b.x-a.x,dz=b.z-a.z, seg=std::sqrt(dx*dx+dz*dz);
        float f=seg>0.0001f?clampf((s-a.s)/seg,0,1):0;
        return {a.x+dx*f,a.y+(b.y-a.y)*f,a.z+dz*f,
                a.cross+(b.cross-a.cross)*f,std::atan2(dx,dz)};
    }
    void sync(const Track& t, float dt) {
        const auto p=pose(t,s);
        const float rx=std::cos(p.yaw),rz=-std::sin(p.yaw);
        const float lane=-1.25f;
        car.x=p.x+rx*lane;car.z=p.z+rz*lane;car.y=p.y+p.cross*lane;
        car.yaw=wrapPi(p.yaw+drift);
        car.velX=std::sin(p.yaw)*speed;car.velZ=std::cos(p.yaw)*speed;
        car.speed=speed*std::cos(drift);car.velForward=car.speed;
        car.velLateral=car.velX*std::cos(car.yaw)-car.velZ*std::sin(car.yaw);
        car.slipAngle=std::atan2(car.velLateral,std::fabs(car.velForward)+.001f);
        car.drifting=std::fabs(car.slipAngle)>.14f && speed>5;
        car.steerInput=clampf(-drift*2.2f,-1,1);
        car.wheelSpin+=speed/.34f*dt;
        car.ground.found=true;car.ground.s=normalizedS(t,s);car.ground.lateral=lane;
        car.ground.centerY=p.y;car.ground.cross=p.cross;car.ground.yaw=p.yaw;
        car.ground.rightX=rx;car.ground.rightZ=rz;
    }
    void reset(const Track& t,float playerS,int difficulty) {
        car=Car{};level=std::max(0,std::min(2,difficulty));speed=drift=distanceTravelled=0;finished=braking=false;
        s=normalizedS(t,playerS);sync(t,0);
    }
    void update(const Track& t,float dt, float playerS=-1.0f, float playerSpeed=0.0f) {
        if(dt<=0 || t.samples.size()<2)return;
        const auto near=pose(t,s+4), ahead=pose(t,s+45);
        const float bend=wrapPi(ahead.yaw-near.yaw);
        const float absBend=std::fabs(bend);
        const float top[3]={36.0f,38.75f,41.4f};
        const float corner[3]={20.25f,22.05f,23.16f};
        const float curve=clampf((absBend-.10f)/.75f,0,1);
        float target=finished?0.0f:top[level]+(corner[level]-top[level])*curve;
        const float progress=t.closed ? distanceTravelled/std::max(1.0f,t.length) :
            (s-t.raceStartS)/std::max(1.0f,t.raceFinishS-t.raceStartS);
        const bool follow=playerS>=0.0f && !finished && (level==0 || (level==1 && progress<0.5f));
        if (follow) {
            float gap=playerS-s;
            if(t.closed && t.length>0) {
                if(gap>t.length*.5f) gap-=t.length;
                if(gap<-t.length*.5f) gap+=t.length;
            }
            // Follow the player's pace within a small lead; allow a modest catch-up boost.
            const float desired=std::max(0.0f, playerSpeed + (gap+10.0f)*0.085f);
            const float adaptive=std::min(desired,target*1.06f);
            // Fade NORMAL's assistance near halfway so there is no sudden acceleration.
            const float weight=level==0?1.0f:clampf((0.50f-progress)/0.10f,0,1);
            target += (adaptive-target)*weight;
        }
        braking=speed>target+.6f;
        speed+=clampf(target-speed,-9.0f*dt,(7.0f+float(level)*.6f)*dt);
        speed=std::max(0.0f,speed);
        const float driftTarget=finished?0.0f:clampf(bend*.85f,-.38f,.38f)*clampf((speed-7.0f)/10.0f,0,1);
        drift+=(driftTarget-drift)*(1.0f-std::exp(-3.8f*dt));
        const float step=speed*dt;
        distanceTravelled+=step;
        s+=step;
        if(t.closed) { if(distanceTravelled>=t.length) finished=true; s=normalizedS(t,s); }
        else { if(s>=t.raceFinishS) finished=true; s=std::min(s,t.length-1.0f); }
        sync(t,dt);
    }
};
