#include "car.hpp"
#include <cassert>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const { return {}; }
int main() {
 assert(!reverseLightsActive(0,0));
 assert(!reverseLightsActive(10,0));
 assert(reverseLightsActive(-1,0));
 assert(reverseLightsActive(0,.30f));
 DriveInput input; input.brake=1;
 assert(brakeLightsActive(input,10));
 assert(!brakeLightsActive(input,-1));
 input.handbrake=true; assert(brakeLightsActive(input,-1));
 CpuMesh lamps; buildCarReverseLights(lamps); assert(!lamps.indices.empty());
 for(size_t i=0;i<lamps.vertices.size();i+=9) assert(lamps.vertices[i+2]<-2.0f);
 puts("PASS: reverse and brake light states, rear lamp geometry");
}
