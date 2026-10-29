#include "effects.hpp"
#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
static bool wall=false, grass=false;
TrackPose Track::query(float x,float,float,float) const {TrackPose p;p.found=wall||grass;p.rightX=1;p.rightZ=0;p.lateral=x;return p;}
int main(){
    SurfaceEffects fx; Vec3 zero{0,0,0}, right{1,0,0}, up{0,1,0};
    fx.emitMud(zero,zero,zero,0,1);assert(fx.particles.empty());
    fx.emitSparks(zero,zero,right,5,20,1.f/60,true);assert(fx.particles.size()>=4);
    const size_t sparks=fx.particles.size();
    fx.emitMud(zero,right,Vec3{0,0,20},20,.5f);assert(fx.particles.size()>sparks);
    std::vector<float> verts;std::vector<unsigned> ids;
    fx.buildQuads(right,up,false,verts,ids);assert(!verts.empty() && verts.size()%28==0 && ids.size()%6==0);
    for(unsigned id:ids) assert(id<verts.size()/7);
    fx.buildQuads(right,up,true,verts,ids);assert(!verts.empty());
    for(int i=0;i<100;++i)fx.emitSparks(zero,zero,right,15,30,.1f,true);
    assert(fx.particles.size()<=SurfaceEffects::kMaxParticles);
    fx.update(3);assert(fx.particles.empty());fx.clear();
    Track t;Car c;c.x=t.wallLateral()-.01f;c.velX=5;c.velZ=25;
    DriveInput none;wall=true;c.update(1.f/60,none,t);
    assert(c.wallContact&&c.wallNewContact&&c.wallImpact>0&&c.wallAlongSpeed>20);
    const float z=c.velZ;c.velX=0;c.update(1.f/60,none,t);
    assert(c.wallContact&&!c.wallNewContact&&c.velZ<z);
    c.velX=-5;c.update(1.f/60,none,t);assert(!c.wallContact);
    wall=false;grass=true;c.x=t.halfWidth+1;c.update(1.f/60,none,t);assert(!c.onAsphalt);
    c.reset(t);assert(!c.wallContact&&!c.wallNewContact);
    std::puts("PASS: spark burst and lifetime, dirt emission, limits, wall contact/scrub/release and grass state");
}
