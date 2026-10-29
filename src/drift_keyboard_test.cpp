#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

static bool wallEnabled = false;
TrackPose Track::query(float x, float, float, float) const {
    TrackPose p;
    if (wallEnabled) { p.found = true; p.lateral = x; p.rightX = 1; p.rightZ = 0; }
    return p;
}

static void run(Car& car, const DriveInput& in, int frames, float dt = 1.25f/60.0f) {
    Track track;
    for (int i = 0; i < frames; ++i) {
        const float prevYaw = car.yawRate;
        const float prevSteer = car.steerInput;
        car.update(dt, in, track);
        assert(std::isfinite(car.velX) && std::isfinite(car.velZ));
        assert(std::fabs(car.yawRate - prevYaw) <= 5.5f*dt + 1e-5f);
        assert(std::fabs(car.steerInput - prevSteer) < 0.32f);
    }
}
int main() {
    for (float dir : {-1.f, 1.f}) {
        Car c; c.velZ = 40;
        DriveInput in; in.throttle=1; in.steer=dir;
        run(c, in, 60);
        assert(c.drifting && std::fabs(c.slipAngle) > 0.35f);
        in.steer=0; run(c, in, 30);
        assert(c.drifting && std::fabs(c.slipAngle) > 0.35f);
        assert(c.groundSpeed()*3.6f > 105 && c.groundSpeed()*3.6f < 120);
        float heading = std::atan2(c.velX,c.velZ);
        run(c,in,1);
        float rate = std::fabs(wrapPi(std::atan2(c.velX,c.velZ)-heading))*60*57.2958f;
        assert(rate < 50);
        in.steer=-dir; run(c,in,30);
        assert(c.slipAngle*dir < -0.15f); // Countersteer doesn't immediately reverse the slide.
        assert(c.groundSpeed()*3.6f > 100);
        in.throttle=0; in.steer=0; run(c,in,240);
        assert(!c.drifting && std::fabs(c.slipAngle)<0.10f);
        std::printf("dir %.0f: neutral path rate %.1f deg/s, slide %.1f deg; counter stays controlled\n",dir,rate,c.slipAngle*57.2958f);
    }
    Car start; start.x = 15.29f; start.yaw = 0.4f; start.velX = 5; start.velZ = 25;
    Track track; DriveInput in; Car free = start;
    wallEnabled = false; free.update(1.f/60,in,track);
    wallEnabled = true; start.update(1.f/60,in,track);
    assert(start.ground.lateral == track.wallLateral());
    assert(std::fabs(start.velZ - free.velZ) < 0.01f); // preserve tangent velocity
    assert(start.velX < free.velX);
    std::puts("PASS: keyboard drift entry/hold/counter/exit, 1.25x real-time pacing, wall tangential velocity");
}
