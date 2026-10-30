#include "car.hpp"
#include <cassert>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const { return {}; }
int main() {
    Track track; Car car; DriveInput input; constexpr float dt=1.0f/120;
    input.brake=1;
    for(int i=0;i<180;++i) car.update(dt,input,track);
    assert(car.speed < -5);
    float reverseSpeed=car.speed;
    input.brake=0; input.throttle=1;
    car.update(dt,input,track);
    assert(car.speed > reverseSpeed);
    for(int i=0;i<180;++i) car.update(dt,input,track);
    assert(car.speed > 5);
    // Simultaneous brake and throttle still give the brake priority.
    car.velX=0; car.velZ=8; car.yaw=0; input.brake=1;
    car.update(dt,input,track); assert(car.speed < 8);
    puts("PASS: reverse, accelerate out of reverse, forward acceleration and brake priority");
}
