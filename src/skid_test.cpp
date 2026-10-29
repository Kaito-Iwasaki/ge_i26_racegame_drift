#include "skidmarks.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
static SkidMarks::Edge edge(float x,float z,float y=0){return {{x-.14f,y,z},{x+.14f,y,z}};}
int main(){
    SkidMarks marks;std::vector<float> vertices;std::vector<unsigned> indices;
    marks.sample(0,true,edge(-.8f,0),1);marks.sample(1,true,edge(.8f,0),1);
    marks.sample(0,true,edge(-.8f,1),1);marks.sample(1,true,edge(.8f,1),1);
    assert(marks.segments.size()==2);
    marks.build({0,2,0},vertices,indices);assert(vertices.size()==8*7 && indices.size()==12);
    for(auto id:indices)assert(id<vertices.size()/7);
    assert(vertices[3]<.05f && vertices[6]>.5f);
    marks.sample(0,false,{},0);marks.sample(0,true,edge(-.8f,3),1);
    assert(marks.segments.size()==2); // No mark across a brake-release gap.
    marks.sample(0,true,edge(-.8f,30),1);assert(marks.segments.size()==2); // No teleport streak.
    marks.build({1000,2,0},vertices,indices);assert(vertices.empty()&&indices.empty());
    marks.update(25);marks.build({0,2,0},vertices,indices);assert(vertices[6]<.5f);
    marks.update(5);assert(marks.segments.empty());
    marks.clear();
    for(int i=0;i<2105;++i)marks.sample(0,true,edge(0,float(i)),1);
    assert(marks.segments.size()==SkidMarks::kMaxSegments);
    marks.clear();assert(marks.segments.empty()&&!marks.connected[0]);
    std::puts("PASS: paired skid ribbons, release gaps, teleports, fade, culling and bounded storage");
}
