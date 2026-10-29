#include "track.cpp"
#include <cassert>
#include <cstdio>
int main(int argc,char**argv){
    assert(argc>1);Track t;assert(t.load(argv[1]));SampleGrid g;g.build(t);Terrain terrain=buildTerrain(t,g);CpuMesh forest;addForest(forest,t,terrain,g);
    assert(!forest.indices.empty());float minDist=100000;int nearGuard=0;
    for(size_t i=0;i<forest.vertices.size();i+=9){
        const float x=forest.vertices[i],y=forest.vertices[i+1],z=forest.vertices[i+2];
        assert(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z));float d2;int near=g.nearestWithin(t,x,z,90,d2);
        assert(near>=0);const float d=std::sqrt(d2);minDist=std::min(minDist,d);if(d<t.wallLateral())++nearGuard;
    }
    for(auto id:forest.indices)assert(id<forest.vertices.size()/9);
    std::printf("vertices %zu triangles %zu min-road-distance %.2fm road-intruding vertices %d\n",forest.vertices.size()/9,forest.indices.size()/3,minDist,nearGuard);
    assert(nearGuard==0);
    assert(forest.vertices.size()/9 < 80000);
}
