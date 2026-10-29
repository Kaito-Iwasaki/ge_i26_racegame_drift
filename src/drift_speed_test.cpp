#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const { return {}; }
int main() {
    Track track;
    for(float side:{-1.f,1.f}) {
        Car car; car.velZ=120.f/3.6f;
        DriveInput input; input.steer=side; input.throttle=1;
        for(int i=0;i<120;++i) {
            car.update(1.25f/60.f,input,track);
            assert(std::isfinite(car.groundSpeed()));
            if(i==59) { assert(car.drifting); assert(car.groundSpeed()*3.6f>118 && car.groundSpeed()*3.6f<125); }
        }
        assert(car.groundSpeed()*3.6f>110 && car.groundSpeed()*3.6f<120);
        const float held=car.groundSpeed();
        input.steer=0; input.throttle=0; input.brake=1;
        for(int i=0;i<30;++i) car.update(1.25f/60.f,input,track);
        assert(car.groundSpeed() < held-5);
        std::printf("side %.0f held %.1f km/h, braking %.1f km/h\n",side,held*3.6f,car.groundSpeed()*3.6f);
    }
    std::puts("PASS: throttle drift speed, no runaway, braking, symmetry");
}
