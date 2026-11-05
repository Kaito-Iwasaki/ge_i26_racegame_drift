#pragma once
#include "car.hpp"
#include "vehicle_tuning.hpp"
#include <algorithm>
#include <cmath>

// The AI produces the same controls as a player; Car::update owns all movement.
struct Rival {
    Car car;
    DriveInput input;
    float s=0, speed=0, drift=0, distanceTravelled=0;
    float lane=-2.10f, laneVelocity=0;
    float preferredLane=-0.6f;
    float driftExitTime=0;
    bool finished=false, braking=false;
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
    void reset(const Track& t,float playerS,int difficulty) {
        car.reset(t); preferredLane=-0.6f; level=std::max(0,std::min(2,difficulty));
        s=normalizedS(t,playerS); const auto p=pose(t,s); lane=-2.10f;
        car.x=p.x+std::cos(p.yaw)*lane; car.z=p.z-std::sin(p.yaw)*lane;
        car.y=p.y+p.cross*lane; car.yaw=p.yaw;
        car.ground=t.query(car.x,car.z,car.y,s);
        input={}; speed=drift=distanceTravelled=laneVelocity=driftExitTime=0; finished=braking=false;
    }
    void acceptContact(const Track& t) {
        TrackPose p=t.query(car.x,car.z,car.y,normalizedS(t,s));
        if (!p.found) return;
        car.ground=p; s=p.s; lane=p.lateral;
        car.y=p.centerY+clampf(p.lateral,-t.halfWidth-t.curbWidth,t.halfWidth+t.curbWidth)*p.cross;
        const float c=std::cos(car.yaw), sn=std::sin(car.yaw);
        car.velForward=car.velX*sn+car.velZ*c; car.velLateral=car.velX*c-car.velZ*sn;
        car.speed=car.velForward; speed=car.groundSpeed();
    }
    void update(const Track& t,float dt,float playerS=-1.0f,float playerSpeed=0.0f) {
        if(dt<=0 || t.samples.size()<2)return;

        const float oldS=s;
        TrackPose ground=t.query(car.x,car.z,car.y,s);
        if(ground.found){car.ground=ground;s=ground.s;lane=ground.lateral;}
        speed=car.groundSpeed();
        const float look=clampf(10.0f+speed*1.0f,12.0f,40.0f);
        const auto aim=pose(t,s+look), near=pose(t,s+4), far=pose(t,s+50);
        const float targetLane=preferredLane + .4f;
        const float tx=aim.x+std::cos(aim.yaw)*targetLane-car.x;
        const float tz=aim.z-std::sin(aim.yaw)*targetLane-car.z;
        const float heading=std::atan2(tx,tz);
        const float error=wrapPi(heading-car.yaw);
        const float bend=std::fabs(wrapPi(far.yaw-near.yaw));
        const float corner[3]={26.0f,26.0f,26.0f};
        const float desiredTop=level==0?39.0f:(level==1?41.0f:VehicleTuning::maxSpeed);
        float target=finished?0.0f:desiredTop+(corner[level]-desiredTop)*clampf((bend-.1f)/1.2f,0,1);
        const float progress=t.closed ? distanceTravelled/std::max(1.0f,t.length) :
            (s-t.raceStartS)/std::max(1.0f,t.raceFinishS-t.raceStartS);
        float leadAdjustment=0.0f;
        if (playerS>=0.0f && !finished && level<2) {
            float gap=playerS-s;
            if(t.closed && t.length>0) {
                if(gap>t.length*.5f) gap-=t.length;
                if(gap<-t.length*.5f) gap+=t.length;
            }
            const float leadAllowance=level==0?60.0f:80.0f;
            const float lead=clampf((-gap-leadAllowance)/180.0f,0,1);
            const float weight=level==0?1.0f:clampf((.50f-progress)/.10f,0,1);
            leadAdjustment=lead*weight;
            target*=1.0f-(level==0?.28f:.22f)*leadAdjustment;
            const float catchUp=clampf((gap-200.0f)/100.0f,0,1);
            const float match=clampf(playerSpeed+2.0f,0,VehicleTuning::maxSpeed);
            // Catch up on straights; keep the corner-speed limit where braking is needed.
            const float straightWeight=1.0f-clampf((bend-.06f)/.35f,0.0f,1.0f);
            target+=(std::max(target,match)-target)*catchUp*straightWeight;
        }
        // Ease braking in a slide and let the driver accelerate promptly once grip returns.
        driftExitTime=car.drifting ? 1.5f : std::max(0.0f,driftExitTime-dt);
        const float leadSteerBlend=clampf((leadAdjustment-.15f)*20.0f,0.0f,1.0f);
        input.steer=clampf(error*(9.0f-3.5f*leadSteerBlend)-car.yawRate*(.3f+.15f*leadSteerBlend)-2.0f*(1.0f-leadSteerBlend)*car.slipAngle,-1.0f,1.0f);
        input.throttle=(!finished && (speed<target || (driftExitTime>0.0f && speed<target+5.0f)) && !car.drifting && std::fabs(car.slipAngle)<.20f)?1.0f:0.0f;
        // Apply the same full throttle available to the player while the slide is controllable.
        if(!finished && leadSteerBlend<.5f && car.drifting && speed<target && std::fabs(car.slipAngle)<.55f)
            input.throttle=1.0f;
        input.brake=(speed>target+4.0f) ? clampf((speed-target)*.05f,0,1) : 0.0f;
        if(car.drifting) input.brake=std::min(input.brake,.08f);
        if(input.throttle>0.0f) input.brake=0.0f;
        // Predict outward drift in road coordinates and lift before reaching the rail.
        if (car.ground.found) {
            const float lateralSpeed=car.velX*car.ground.rightX+car.velZ*car.ground.rightZ;
            const float projected=lane+lateralSpeed*(level==2 ? .6f : .75f);
            const float safeLateral=std::max(1.0f,t.halfWidth-(level==2 ? .1f : .4f));
            if (std::fabs(projected)>safeLateral && projected*lateralSpeed>0.0f) {
                input.throttle=0.0f;
                input.brake=std::max(input.brake,.08f);
            }
        }
        input.handbrake=false;
        braking=brakeLightsActive(input,car.speed);
        car.update(dt,input,t);
        if(car.ground.found){s=car.ground.s;lane=car.ground.lateral;}
        float step=s-oldS;
        if(t.closed && step < -t.length*.5f) step+=t.length;
        if(t.closed && step > t.length*.5f) step-=t.length;
        distanceTravelled+=std::max(0.0f,step);
        speed=car.groundSpeed(); drift=car.slipAngle;
        laneVelocity=car.velX*car.ground.rightX+car.velZ*car.ground.rightZ;
        if(t.closed ? distanceTravelled>=t.length : s>=t.raceFinishS) finished=true;
    }
};
