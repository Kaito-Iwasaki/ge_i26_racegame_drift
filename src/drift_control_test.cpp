// Standalone, wall-free handling regression: g++ -std=c++17 -Isrc drift_control_test.cpp src/car.cpp
#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
TrackPose Track::query(float, float, float, float) const { return {}; }

void run(Car& c, const DriveInput& input, float dt, float duration) {
    Track track;
    for (int i = 0; i < int(duration / dt + 0.5f); ++i) {
        const float prev = c.yawRate;
        c.update(dt, input, track);
        assert(std::isfinite(c.velX) && std::isfinite(c.velZ));
        assert(std::fabs(c.yawRate - prev) <= 5.5f * dt + 0.0001f);
    }
}
int main() {
    for (float dt : {1.f / 30, 1.f / 48, 1.f / 60, 1.f / 120}) {
        for (float side : {-1.f, 1.f}) {
            Car entry; entry.velZ = 40;
            DriveInput steer; steer.steer = side; steer.throttle = 1;
            run(entry, steer, dt, 1.2f);
            assert(entry.drifting && std::fabs(entry.slipAngle) > 0.40f);
            assert(entry.groundSpeed()*3.6f > 100 && entry.groundSpeed()*3.6f < 132);
            Car neutral = entry; DriveInput hold; hold.throttle = 1;
            run(neutral, hold, dt, 0.5f);
            assert(neutral.drifting && std::fabs(neutral.slipAngle) > 0.25f);
            Car counter = entry; hold.steer = -0.25f * side;
            run(counter, hold, dt, 0.5f);
            assert(counter.drifting && std::fabs(counter.slipAngle) > 0.20f);
            run(counter, hold, dt, 1.5f);
            run(neutral, DriveInput{1, 0, 0, false}, dt, 1.5f);
            assert(std::fabs(counter.slipAngle) < std::fabs(neutral.slipAngle));
            hold.steer = -side;
            run(counter, hold, dt, 0.7f);
            assert(std::fabs(counter.slipAngle) < 0.20f);
            Car coast = entry; DriveInput noInput;
            run(coast, noInput, dt, 3.0f);
            assert(!coast.drifting && std::fabs(coast.slipAngle) < 0.08f);
            Car brake = entry; DriveInput braking; braking.brake = 1;
            run(brake, braking, dt, 0.5f);
            assert(brake.groundSpeed() < entry.groundSpeed() - 5);
            Car hand; hand.velZ = 30;
            DriveInput hb; hb.steer = side; hb.handbrake = true; hb.throttle = 1;
            run(hand, hb, dt, 1.0f);
            assert(hand.drifting && std::fabs(hand.slipAngle) > 0.35f);
            assert(hand.groundSpeed()*3.6f > 70 && hand.groundSpeed()*3.6f < 108);
        }
    }
    std::puts("PASS: entry, neutral and counter hold, recovery, brake, handbrake, symmetry and 30-120 Hz");
}
