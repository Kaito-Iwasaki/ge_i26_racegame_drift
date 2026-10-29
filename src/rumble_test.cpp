#include "rumble.hpp"
#include <cassert>
#include <cstdio>
int main(){
    RumbleState r;
    auto idle=r.update(.016f,false,true,true,false,0,0,false);
    assert(!idle.send&&!idle.stop);
    auto grass=r.update(.016f,false,true,false,false,0,25,false);
    assert(grass.send&&grass.high>.2f&&grass.low>0);
    auto pause=r.update(.016f,true,true,false,false,0,25,false);
    assert(pause.stop&&!pause.send);
    assert(!r.update(.016f,true,true,false,false,0,25,false).stop);
    auto drift=r.update(.016f,false,true,true,true,.4f,30,false);
    assert(drift.send&&drift.high>0&&drift.high<.25f);
    auto wall=r.update(.1f,false,true,true,false,0,30,true);
    assert(wall.send&&wall.high>.3f);
    r.collision(12);auto hit=r.update(.016f,false,true,true,false,0,30,false);
    assert(hit.send&&hit.high>.8f&&hit.high<=1&&hit.low<=1);
    for(int i=0;i<120;++i)r.update(.016f,false,true,true,false,0,30,false);
    assert(r.impact==0);
    r.collision(12);auto stop=r.update(.016f,true,true,true,false,0,30,false);
    assert(r.impact==0&&!stop.send);
    // Direction is relative to the vehicle, not the course or world axes.
    constexpr float pi=3.14159265f;
    assert(RumbleState::wallDirection(0,1,0,1)>.99f);
    assert(RumbleState::wallDirection(pi,1,0,1)<-.99f);
    assert(std::fabs(RumbleState::wallDirection(pi/2,1,0,1))<.001f);
    RumbleState l, rr;
    l.collision(12,-1); rr.collision(12,1);
    auto left=l.update(.016f,false,true,true,false,0,30,false);
    auto right=rr.update(.016f,false,true,true,false,0,30,false);
    assert(left.low>left.high && right.high>right.low);
    RumbleState dl,dr;
    auto driftLeft=dl.update(.016f,false,true,true,true,.4f,30,false);
    auto driftRight=dr.update(.016f,false,true,true,true,-.4f,30,false);
    assert(driftLeft.low>driftLeft.high && driftRight.high>driftRight.low);
    RumbleState wl,wr;
    auto wallLeft=wl.update(.016f,false,true,true,false,0,30,true,-1);
    auto wallRight=wr.update(.016f,false,true,true,false,0,30,true,1);
    assert(wallLeft.low>wallLeft.high && wallRight.high>wallRight.low);
    // Grass alone is independent of the last wall side.
    RumbleState gl,gr;
    auto grassLeft=gl.update(.016f,false,true,false,false,0,30,false,-1);
    auto grassRight=gr.update(.016f,false,true,false,false,0,30,false,1);
    assert(grassLeft.low==grassRight.low && grassLeft.high==grassRight.high);
    std::puts("PASS: idle, grass, drift, wall, impact, decay, pause and bounds");
}
