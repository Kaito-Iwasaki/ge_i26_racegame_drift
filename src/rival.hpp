#pragma once
#include "car.hpp"
#include "vehicle_tuning.hpp"
#include <algorithm>
#include <cmath>

// A route-following rival: progress and drift attitude are animated independently
// of the player's handling model. Road samples are looked up by distance.
struct Rival {
    Car car;
    float s=0, speed=0, drift=0, distanceTravelled=0;
    float lane=-2.10f, laneVelocity=0;
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
        car.x=p.x+rx*lane;car.z=p.z+rz*lane;car.y=p.y+p.cross*lane;
        car.yaw=wrapPi(p.yaw+drift);
        car.velX=std::sin(p.yaw)*speed+rx*laneVelocity;car.velZ=std::cos(p.yaw)*speed+rz*laneVelocity;
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
        car=Car{};level=std::max(0,std::min(2,difficulty));speed=drift=distanceTravelled=0;lane=-2.10f;laneVelocity=0;finished=braking=false;
        s=normalizedS(t,playerS);sync(t,0);
    }
    // Carry the resolved displacement and momentum into the next AI step.
    void acceptContact(const Track& t) {
        TrackPose p=t.query(car.x,car.z,car.y,normalizedS(t,s));
        if(!p.found)return;
        s=p.s; lane=p.lateral;
        laneVelocity=car.velX*p.rightX+car.velZ*p.rightZ;
        const float fx=std::sin(p.yaw),fz=std::cos(p.yaw);
        speed=clampf(car.velX*fx+car.velZ*fz,0.0f,VehicleTuning::maxSpeed);
        car.ground=p;
        const float c=std::cos(car.yaw), sn=std::sin(car.yaw);
        car.velForward=car.velX*sn+car.velZ*c;
        car.velLateral=car.velX*c-car.velZ*sn;
        car.speed=car.velForward;
        car.y=p.centerY+clampf(p.lateral,-t.halfWidth-t.curbWidth,t.halfWidth+t.curbWidth)*p.cross;
    }
    void update(const Track& t,float dt, float playerS=-1.0f, float playerSpeed=0.0f) {
        if(dt<=0 || t.samples.size()<2)return;
        const auto near=pose(t,s+4), ahead=pose(t,s+45);
        const float bend=wrapPi(ahead.yaw-near.yaw);
        const float absBend=std::fabs(bend);
        const float corner[3]={21.0f,24.3f,30.0f};
        const float curve=clampf((absBend-.10f)/.75f,0,1);
        float target=finished?0.0f:VehicleTuning::maxSpeed+(corner[level]-VehicleTuning::maxSpeed)*curve;
        const float progress=t.closed ? distanceTravelled/std::max(1.0f,t.length) :
            (s-t.raceStartS)/std::max(1.0f,t.raceFinishS-t.raceStartS);
        if (playerS>=0.0f && !finished && level<2) {
            float gap=playerS-s; // positive: the player is ahead
            if(t.closed && t.length>0) {
                if(gap>t.length*.5f) gap-=t.length;
                if(gap<-t.length*.5f) gap+=t.length;
            }
            // Ease the rival's lead without stopping it or copying the player's speed.
            // NORMAL's lead adjustment fades out by the middle of the course.
            const float leadAllowance=level==0?60.0f:80.0f;
            const float lead=clampf((-gap-leadAllowance)/180.0f,0.0f,1.0f);
            const float leadWeight=level==0?1.0f:clampf((0.50f-progress)/0.10f,0,1);
            const float reduction=level==0?.28f:.22f;
            target *= 1.0f-reduction*lead*leadWeight;
            // A player far ahead may be matched through corners, without exceeding
            // the same speed or acceleration limits as the player's car.
            const float catchUp=clampf((gap-200.0f)/100.0f,0.0f,1.0f);
            const float match=clampf(playerSpeed+2.0f,0.0f,VehicleTuning::maxSpeed);
            target += (std::max(target,match)-target)*catchUp;
        }
        braking=speed>target+.6f;
        const float previousSpeed=speed;
        const float previousYaw=pose(t,s).yaw;
        // The same engine, drag and braking limits as the player's car.
        const float driftMix=clampf((std::fabs(drift)-.06f)/.30f,0.0f,1.0f);
        const float resistance=VehicleTuning::drag*speed*speed+(speed>.05f?VehicleTuning::rolling:0.0f);
        const float accel=std::max(0.0f,VehicleTuning::engineAcceleration(speed,driftMix)-resistance);
        const float decel=VehicleTuning::brake+resistance;
        speed=clampf(speed+clampf(target-speed,-decel*dt,accel*dt),0.0f,VehicleTuning::maxSpeed);
        const float skillDrift[3]={.85f,1.02f,1.20f};
        const float driftTarget=finished?0.0f:clampf(bend*skillDrift[level],-.48f,.48f)*clampf((speed-7.0f)/10.0f,0,1);
        drift+=(driftTarget-drift)*(1.0f-std::exp(-3.8f*dt));
        // Recover gradually from collision deflection instead of snapping to the racing line.
        laneVelocity+=((-2.10f-lane)*1.4f-laneVelocity*2.8f)*dt;
        laneVelocity=clampf(laneVelocity,-6.0f,6.0f);
        lane+=laneVelocity*dt;
        const float laneLimit=std::max(0.0f,t.wallLateral()-2.4f);
        if(lane>laneLimit){lane=laneLimit;laneVelocity=std::min(0.0f,laneVelocity);}
        if(lane<-laneLimit){lane=-laneLimit;laneVelocity=std::max(0.0f,laneVelocity);}
        const float step=speed*dt;
        distanceTravelled+=step;
        s+=step;
        if(t.closed) { if(distanceTravelled>=t.length) finished=true; s=normalizedS(t,s); }
        else { if(s>=t.raceFinishS) finished=true; s=std::min(s,t.length-1.0f); }
        sync(t,dt);
        // The sprung body reacts to acceleration and lateral load, independently
        // of the visual road-height suspension used by the renderer.
        const float pathYaw=pose(t,s).yaw;
        const float latAccel=clampf(wrapPi(pathYaw-previousYaw)*speed/dt,-25.0f,25.0f);
        const float rollTarget=clampf(latAccel*.0058f,-.08f,.08f);
        car.bodyRollVel+=((rollTarget-car.bodyRoll)*32.0f-car.bodyRollVel*8.0f)*dt;
        car.bodyRoll=clampf(car.bodyRoll+car.bodyRollVel*dt,-.10f,.10f);
        const float longAccel=(speed-previousSpeed)/dt;
        const float pitchTarget=clampf(longAccel*.0038f,-.04f,.04f);
        car.bodyPitch+=(pitchTarget-car.bodyPitch)*(1.0f-std::exp(-6.0f*dt));
    }
};
