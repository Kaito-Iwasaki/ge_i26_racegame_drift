#include "smoke.hpp"
#include "skidmarks.hpp"
#include "lighttrail.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(){Mat4 m=mul(perspective(70*3.14159265f/180,16.f/9,.15f,1000),lookAt({0,0,0},{0,0,1},{0,1,0}));ViewFrustum fr(m);
 assert(fr.intersectsSphere({0,0,10},1));assert(!fr.intersectsSphere({0,0,-10},1));assert(!fr.intersectsSphere({200,0,10},1));assert(fr.intersectsSphere({0,0,.1f},1));
 SmokeSystem p,r;SmokeSystem::Particle x;x.age=.5f;x.life=1;x.pos={0,0,10};p.particles.push_back(x);x.pos={0,0,-10};p.particles.push_back(x);x.pos={1,0,12};r.particles.push_back(x);x.pos={200,0,10};r.particles.push_back(x);
 std::vector<float> v;std::vector<unsigned> ids;p.appendQuads({1,0,0},{0,1,0},v,ids,&fr);r.appendQuads({1,0,0},{0,1,0},v,ids,&fr);assert(v.size()==48&&ids.size()==12);for(auto id:ids)assert(id<v.size()/6);
 SkidMarks a,b;auto edge=[](float z){return SkidMarks::Edge{{-.1f,0,z},{.1f,0,z}};};a.sample(0,true,edge(10),1);a.sample(0,true,edge(11),1);b.sample(0,true,edge(-10),1);b.sample(0,true,edge(-9),1);
 std::vector<float> q;std::vector<unsigned> qi;a.append({0,0,0},q,qi,&fr);b.append({0,0,0},q,qi,&fr);assert(q.size()==28&&qi.size()==6);
 LightTrail l1,l2; l1.record({-1,0,1},{1,0,1}); l1.record({-1,0,2},{1,0,2}); l2.record({-1,0,3},{1,0,3}); l2.record({-1,0,4},{1,0,4});
 std::vector<float> t; std::vector<unsigned> ti; l1.appendQuads(t,ti); l2.appendQuads(t,ti); assert(t.size()==112&&ti.size()==24); for(auto id:ti) assert(id<t.size()/7);
 puts("PASS: conservative view culling and batched effect indices");}
