#include "suspension_visual.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main() {
 SuspensionVisual s; float flat[4]={10,10,10,10}; s.update(flat,0.016f);
 assert(std::fabs(s.heave(10))<1e-5 && std::fabs(s.pitch())<1e-5 && std::fabs(s.roll())<1e-5);
 float step[4]={10.1f,10,10,10}; s.update(step,1.f/60); assert(s.height[0]>10 && s.height[0]<10.1f && s.roll()<0 && s.pitch()>0);
 for(int i=0;i<600;i++) s.update(step,1.f/60);
 assert(std::fabs(s.height[0]-10.1f)<0.001f && std::fabs(s.height[1]-10)<0.001f);
 float before=s.height[0]; s.update(flat,0); assert(s.height[0]==before);
 s.reset(); float hill[4]={12,12,11.72f,11.72f}; s.update(hill,0); assert(s.pitch()>0.09f && std::fabs(s.roll())<1e-5);
 puts("PASS: flat, independent bump, damping, pause, reset and slope");
}
