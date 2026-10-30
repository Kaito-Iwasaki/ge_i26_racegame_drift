#include "smoke.hpp"
#include "skidmarks.hpp"
#include "course_chunks.hpp"
#include <cassert>
#include <cstdio>
int main(){SmokeSystem full,small;for(int i=0;i<300;i++){float dt=1.f/60;full.emit({0,0,0},{1,0,0},{0,0,0},1,dt);small.emit({0,0,0},{1,0,0},{0,0,0},1,dt,.6f,180);full.update(dt);small.update(dt);}assert(small.particles.size()<full.particles.size() && small.particles.size()<=180);
 SkidMarks marks;marks.minSpacing=1;auto edge=[](float x){return SkidMarks::Edge{{x,0,0},{x,0,.28f}};};marks.sample(0,true,edge(0),1);marks.sample(0,true,edge(.5f),1);assert(marks.segments.empty());marks.sample(0,true,edge(1.1f),1);assert(marks.segments.size()==1);marks.update(30);assert(marks.segments.empty());
 CourseChunk near{{-2,-1,-2},{2,1,2},0,3}, far{{171,0,0},{180,1,1},0,3};assert(courseChunkWithinDistance(near,{0,0,0},170));assert(!courseChunkWithinDistance(far,{0,0,0},170));
 printf("PASS: rival smoke %zu vs %zu, spaced tyre marks and lighting bounds\n",small.particles.size(),full.particles.size());}
