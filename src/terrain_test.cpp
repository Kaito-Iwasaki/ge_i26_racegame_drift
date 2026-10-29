#include "track.cpp"
#include <cassert>
#include <cstdio>
#include <chrono>
int main(int argc,char** argv) {
    assert(argc>1);
    Track t; assert(t.load(argv[1]));
    SampleGrid g; g.build(t);
    Terrain terrain=buildTerrain(t,g);
    CpuMesh ground; addTerrainMesh(ground,terrain);
    CpuMesh forest; addForest(forest,t,terrain,g);
    assert(ground.indices.size()%3==0 && !ground.indices.empty());
    size_t terrainVertices=ground.vertices.size()/9;
    for(auto id:ground.indices) assert(id<terrainVertices);
    for(size_t i=0;i<ground.indices.size();i+=3) {
        auto vertex=[&](unsigned id){return Vec3{ground.vertices[id*9],ground.vertices[id*9+1],ground.vertices[id*9+2]};};
        auto a=vertex(ground.indices[i]),b=vertex(ground.indices[i+1]),c=vertex(ground.indices[i+2]);
        assert(cross(b-a,c-a).y >= 0);
    }
    for(size_t i=0;i<t.samples.size();i+=5) {
        const auto &s=t.samples[i];
        int ix=int(std::floor((s.x-terrain.minX)/terrain.cell));
        int iz=int(std::floor((s.z-terrain.minZ)/terrain.cell));
        assert(ix>=0 && iz>=0 && ix+1<terrain.nx && iz+1<terrain.nz);
        for(int dz=0;dz<=1;++dz) for(int dx=0;dx<=1;++dx)
            assert(terrain.roadDistance2[size_t(iz+dz)*terrain.nx+ix+dx]<1.e29f);
        assert(std::isfinite(terrain.sample(s.x,s.z)));
    }
    for(size_t i=0;i<ground.vertices.size();i+=9) {
        float x=ground.vertices[i], z=ground.vertices[i+2], d2;
        assert(g.nearestWithin(t,x,z,210.f,d2)>=0);
        assert(d2<201.f*201.f);
    }
    std::printf("terrain vertices %zu, triangles %zu; forest vertices %zu, triangles %zu; road sample coverage and indices PASS\n",terrainVertices,ground.indices.size()/3,forest.vertices.size()/9,forest.indices.size()/3);
}
