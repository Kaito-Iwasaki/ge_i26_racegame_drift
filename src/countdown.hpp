#pragma once
#include <algorithm>
#include <cmath>

// Uses real simulation-step time, independent of the car's arcade pace.
struct StartCountdown {
    static constexpr double kDuration=3.0;
    static constexpr double kGoDuration=0.75;
    double remaining=kDuration;
    double goRemaining=0;
    void reset() { remaining=kDuration; goRemaining=0; }
    // Returns whether this step may advance the car and race timer.
    bool step(double dt, bool paused=false) {
        if (paused || dt<=0) return false;
        if (remaining>0) {
            remaining=std::max(0.0,remaining-dt);
            if (remaining<1.0e-9) { remaining=0; goRemaining=kGoDuration; }
            return false;
        }
        goRemaining=std::max(0.0,goRemaining-dt);
        return true;
    }
    int number() const { return remaining>0 ? static_cast<int>(std::ceil(remaining - 1.0e-9)) : 0; }
    bool showGo() const { return remaining<=0 && goRemaining>0; }
};
