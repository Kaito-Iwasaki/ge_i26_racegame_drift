#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const { return {}; }
int main() {
    Track track;
    for(float dt:{1.f/60,1.f/48,1.f/120}) for(float side:{-1.f,1.f}) {
        Car entry; entry.velZ=40;
        DriveInput input; input.steer=side; input.throttle=1;
        for(int i=0;i<int(1.2f/dt+0.5f);++i) entry.update(dt,input,track);
        assert(entry.drifting && std::fabs(entry.slipAngle)>.35f);
        Car neutral=entry, counter=entry;
        for(int i=0;i<int(1.f/dt+0.5f);++i) {
            input.steer=0; neutral.update(dt,input,track);
            input.steer=-side; counter.update(dt,input,track);
            assert(std::isfinite(counter.slipAngle));
        }
        assert(std::fabs(neutral.slipAngle)>.35f);
        assert(std::fabs(counter.slipAngle)<.20f);
        assert(counter.slipAngle*side<0); // Not yet an opposite-direction drift.
        assert(counter.groundSpeed()*3.6f>100);
        std::printf("dt %.4f side %.0f neutral %.1f deg counter %.1f deg speed %.1f km/h\n",dt,side,std::fabs(neutral.slipAngle)*57.2958f,std::fabs(counter.slipAngle)*57.2958f,counter.groundSpeed()*3.6f);
    }
    std::puts("PASS: countersteer recovery, neutral hold, speed and symmetry");
}
