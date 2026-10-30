#include "track.hpp"
#include <cassert>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <random>
int main(int argc,char**argv){assert(argc>1);Track t;assert(t.load(argv[1]));t.visual=CpuMesh{};
 for(bool up:{false,true}){if(up && t.closed)break;assert(t.setUphill(up));Track ref=t;ref.queryNodes.clear();ref.querySegments.clear();ref.queryLengths.clear();std::mt19937 rng(149);std::uniform_real_distribution<float> pos(-35,35), h(-30,30);
  double fast=0,slow=0;for(int i=0;i<2500;i++) {const auto& a=t.samples[rng()%t.samples.size()];float x=a.x+pos(rng),z=a.z+pos(rng),y=a.y+h(rng),prefer=(i%5==0)?-1:a.s;
   auto begin=std::chrono::steady_clock::now();auto p=t.query(x,z,y,prefer);auto mid=std::chrono::steady_clock::now();auto q=ref.query(x,z,y,prefer);auto end=std::chrono::steady_clock::now();
   fast+=std::chrono::duration<double,std::micro>(mid-begin).count();slow+=std::chrono::duration<double,std::micro>(end-mid).count();
   assert(p.found==q.found && std::fabs(p.s-q.s)<.002f && std::fabs(p.lateral-q.lateral)<.002f && std::fabs(p.centerY-q.centerY)<.002f);
  }
  printf("%s: indexed %.2f us/query, exhaustive %.2f us/query; 2500 matching queries\n",up?"up":"down",fast/2500,slow/2500);
 }
 puts("PASS: indexed road queries match exhaustive search in both directions");}
