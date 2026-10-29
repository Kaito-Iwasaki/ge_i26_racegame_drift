#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const { return {}; }
int main() {
    Track track; constexpr float dt=1.25f/60.f;
    for(float side:{-1.f,1.f}) {
        Car drift; drift.velZ=120.f/3.6f;
        DriveInput input; input.steer=side; input.throttle=1;
        for(int i=0;i<90;++i) drift.update(dt,input,track);
        assert(drift.drifting);
        float start=drift.groundSpeed();
        input.throttle=0;
        for(int i=0;i<60;++i) drift.update(dt,input,track);
        float coast=drift.groundSpeed();
        assert(coast*3.6f>70 && coast<start);
        assert(drift.drifting);

        Car braking; braking.velZ=120.f/3.6f;
        input=DriveInput{}; input.steer=side; input.brake=1;
        float maxSlip=0;
        for(int i=0;i<30;++i) {
            braking.update(dt,input,track);
            maxSlip=std::max(maxSlip,std::fabs(braking.slipAngle));
        }
        assert(braking.drifting && maxSlip>.19f);
        assert(braking.groundSpeed()*3.6f>80 && braking.groundSpeed()*3.6f<95);
        assert(brakeLightsActive(input,braking.speed));
        std::printf("side %.0f coast %.1f km/h; brake+steer %.1f km/h, slip %.1f deg\n",side,coast*3.6f,braking.groundSpeed()*3.6f,maxSlip*57.2958f);
    }
    Car straight; straight.velZ=120.f/3.6f; DriveInput brake; brake.brake=1;
    for(int i=0;i<30;++i) straight.update(dt,brake,track);
    assert(!straight.drifting && straight.groundSpeed()*3.6f>85 && straight.groundSpeed()*3.6f<95);
    for(int i=0;i<180;++i) straight.update(dt,brake,track);
    assert(straight.speed < -1 && straight.speed >= -9.0f);
    std::puts("PASS: coasting, braking, brake drift, brake lamp, reversal and symmetry");
}
