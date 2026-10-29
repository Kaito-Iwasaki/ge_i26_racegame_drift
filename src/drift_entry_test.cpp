#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const { return {}; }

int main() {
    Track track;
    for (float side : {-1.f, 1.f}) {
        Car car; car.velZ=120.f/3.6f;
        DriveInput in; in.steer=side; in.throttle=1;
        const float dt=1.25f/60.f;
        float rate=0;
        for (int i=0;i<120;++i) {
            const float heading=std::atan2(car.velX,car.velZ);
            car.update(dt,in,track);
            rate=std::fabs(wrapPi(std::atan2(car.velX,car.velZ)-heading))*60.f*57.2958f;
            assert(std::isfinite(rate) && std::isfinite(car.speed));
            if (i==59) {
                assert(car.drifting && std::fabs(car.slipAngle)>.35f);
                assert(rate>30 && rate<52);
                assert(car.groundSpeed()*3.6f>105);
            }
        }
        assert(rate>55); // Steering gradually regains cornering authority.
        std::printf("direction %.0f: entry retains slip; later rate %.1f deg/s\n",side,rate);

        Car hand; hand.velZ=120.f/3.6f; in.handbrake=true;
        for (int i=0;i<60;++i) {
            const float heading=std::atan2(hand.velX,hand.velZ);
            hand.update(dt,in,track);
            rate=std::fabs(wrapPi(std::atan2(hand.velX,hand.velZ)-heading))*60.f*57.2958f;
        }
        assert(hand.drifting && rate>80 && rate<93); // Existing brake entry.
        assert(hand.groundSpeed()*3.6f>88 && hand.groundSpeed()*3.6f<95);
        in.handbrake=false;
        const float mix=hand.brakeDriftMix;
        hand.update(dt,in,track);
        assert(hand.brakeDriftMix < mix && hand.brakeDriftMix > .9f);
        hand.reset(track);
        assert(hand.slideEntryTime==0 && hand.brakeDriftMix==0);
    }
    std::puts("PASS: straight slide entry, gradual cornering, unchanged brake entry, release fade and reset");
}
