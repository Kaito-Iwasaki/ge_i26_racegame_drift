#include "track.hpp"
#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(int argc,char** argv) {
 assert(argc>1); Track t; assert(t.load(argv[1])); assert(!t.closed && t.touge);
 assert(t.startGate>0 && t.finishGate<int(t.samples.size())-1);
 assert(t.raceStartS>99 && t.length-t.raceFinishS>99);
 auto start=t.samples[t.startGate],finish=t.samples[t.finishGate];
 auto p=t.query(t.spawnX,t.spawnZ,t.spawnY); assert(p.found && p.s>t.raceStartS && p.s<t.raceFinishS);
 auto ext=t.samples[t.finishGate+10]; auto after=t.query(ext.x,ext.z,ext.y,ext.s);
 assert(after.found && after.s>t.raceFinishS+35);
 float spawnX=t.spawnX,spawnZ=t.spawnZ,spawnYaw=t.spawnYaw, oldLength=t.length;
 float c=t.samples[t.startGate+10].cross;
 auto original=t.samples[t.startGate+10];
 assert(t.setUphill(true) && t.uphill);
 assert(std::fabs(t.samples[t.startGate].x-finish.x)<.001f && std::fabs(t.samples[t.finishGate].x-start.x)<.001f);
 assert(t.samples[t.startGate].y<t.samples[t.finishGate].y);
 auto rev=t.samples[t.samples.size()-1-(t.finishGate+10)];
 assert(std::fabs(t.length-oldLength)<.1f && t.raceStartS>99 && t.length-t.raceFinishS>99);
 p=t.query(t.spawnX,t.spawnZ,t.spawnY); assert(p.found && p.s<t.raceStartS-19 && p.s>t.raceStartS-21);
 float dx=t.samples[t.startGate+3].x-t.samples[t.startGate+2].x,dz=t.samples[t.startGate+3].z-t.samples[t.startGate+2].z;
 assert(std::cos(t.spawnYaw)*dz+std::sin(t.spawnYaw)*dx>0);
 auto afterUp=t.samples[t.finishGate+10];p=t.query(afterUp.x,afterUp.z,afterUp.y,afterUp.s);assert(p.found && p.s>t.raceFinishS+35);
 (void)rev;
 assert(t.setUphill(false) && !t.uphill);
 assert(std::fabs(t.spawnX-spawnX)<.001f && std::fabs(t.spawnZ-spawnZ)<.001f && std::fabs(t.spawnYaw-spawnYaw)<.001f);
 assert(std::fabs(t.samples[t.startGate+10].cross-c)<.0001f && std::fabs(t.samples[t.startGate+10].x-original.x)<.001f);
 Track loop; loop.closed=true;assert(!loop.setUphill(true));
 std::printf("PASS: %.0fm track, 100m runout both ends, uphill spawn, finish and reversible banking\n",t.length);
}
