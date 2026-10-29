#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const { return {}; }
int main() {
    Track t; const float dt=1.25f/60.f;
    for(float side:{-1.f,1.f}) {
        Car grip; grip.velZ=120.f/3.6f;
        DriveInput in; in.steer=side*.5f; in.throttle=1;
        for(int i=0;i<120;++i) { grip.update(dt,in,t); assert(!grip.drifting); }
        assert(std::fabs(grip.slipAngle)<.12f);
        assert(grip.yawRate*side>0);
        in.steer=0;
        for(int i=0;i<90;++i) { grip.update(dt,in,t); assert(grip.yawRate*side > -.01f); }
        assert(std::fabs(grip.yawRate)<.02f);

        Car keys; keys.velZ=120.f/3.6f;
        DriveInput tap; tap.throttle=1; tap.steer=side;
        for(int i=0;i<18;++i) { keys.update(dt,tap,t); assert(!keys.drifting); }
        tap.steer=0;
        for(int i=0;i<72;++i) { keys.update(dt,tap,t); assert(!keys.drifting); assert(keys.yawRate*side > -.01f); }
        assert(std::fabs(keys.yawRate)<.02f);

        Car slide; slide.velZ=120.f/3.6f;
        in=DriveInput{}; in.throttle=1; in.steer=side*.5f; in.handbrake=true;
        for(int i=0;i<30;++i) slide.update(dt,in,t);
        assert(slide.drifting && std::fabs(slide.slipAngle)>.19f);
        assert(slide.groundSpeed()*3.6f>90);
        assert(brakeLightsActive(in,slide.speed));
    }
    DriveInput in; assert(!brakeLightsActive(in,0));
    in.brake=1; assert(brakeLightsActive(in,0)); assert(!brakeLightsActive(in,-1));
    in.brake=0; in.handbrake=true; assert(brakeLightsActive(in,0)); assert(brakeLightsActive(in,-1));
    std::puts("PASS: grip steering/release, space drift initiation, speed and brake lights");
}
