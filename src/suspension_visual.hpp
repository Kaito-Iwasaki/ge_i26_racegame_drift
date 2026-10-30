#pragma once
#include <algorithm>
#include <cmath>

// Visual suspension: four damped support points follow the road independently.
// It is deliberately separate from tyre forces so the driving model is unchanged.
struct SuspensionVisual {
    float height[4] = {};
    float velocity[4] = {};
    bool initialized = false;
    void reset() { initialized = false; }
    void update(const float road[4], float dt) {
        if (!initialized) {
            for (int i=0;i<4;++i) { height[i]=road[i]; velocity[i]=0; }
            initialized=true;
            return;
        }
        // Substeps keep the damped spring stable even on a long rendered frame.
        dt = std::max(0.0f, std::min(dt, 0.10f));
        while (dt > 0.0f) {
            const float h=std::min(dt, 1.0f/120.0f);
            for (int i=0;i<4;++i) {
                const float error=std::max(-0.25f,std::min(0.25f,road[i]-height[i]));
                velocity[i] += (error * 90.0f - velocity[i] * 15.0f) * h;
                height[i] += velocity[i] * h;
            }
            dt-=h;
        }
    }
    float heave(float base) const {
        const float avg=(height[0]+height[1]+height[2]+height[3])*0.25f;
        return std::max(-0.16f,std::min(0.16f,avg-base));
    }
    float pitch() const { return std::atan2((height[0]+height[1]-height[2]-height[3])*0.5f, 2.80f); }
    float roll() const { return std::atan2((height[1]+height[3]-height[0]-height[2])*0.5f, 1.64f); }
};
