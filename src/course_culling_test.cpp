#include "course_chunks.hpp"
#include "track.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <vector>
int main(int argc,char**argv){assert(argc>1);Track t;assert(t.load(argv[1]));auto chunks=chunkCourse(t.visual);
 std::vector<std::array<unsigned,3>> a,b;
 for(size_t i=0;i<t.visual.indices.size();i+=3)a.push_back({t.visual.indices[i],t.visual.indices[i+1],t.visual.indices[i+2]});
 for(size_t i=0;i<chunks.indices.size();i+=3)b.push_back({chunks.indices[i],chunks.indices[i+1],chunks.indices[i+2]});
 std::sort(a.begin(),a.end());std::sort(b.begin(),b.end());assert(a==b);
 CourseChunk near{{-2,-2,-2},{2,2,2},0,3}, far{{1200,0,0},{1205,2,3},0,3}, edge{{1099,-1,-1},{1200,1,1},0,3};
 assert(courseChunkWithinDistance(near,{0,0,0},1100));assert(!courseChunkWithinDistance(far,{0,0,0},1100));assert(courseChunkWithinDistance(edge,{0,0,0},1100));
 for(auto &c:chunks.chunks)if(c.max.x-c.min.x>1000 || c.max.z-c.min.z>1000)assert(c.count==3);
 printf("PASS: %zu triangles retained, oversized ground isolated, conservative distance bounds\n",a.size());
}
