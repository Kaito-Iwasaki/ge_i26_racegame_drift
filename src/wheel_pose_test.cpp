#include "wheel_pose.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(){
 float flat[4]={10,10,10,10}; Mat4 w=roadWheelMatrix(1,2,0,flat,0,0,0,0);
 assert(std::fabs(w.m[12]-.18f)<.0001f && std::fabs(w.m[13]-10.34f)<.0001f && std::fabs(w.m[14]-3.42f)<.0001f);
 assert(std::fabs(w.m[1])<.0001f && std::fabs(w.m[5]-1)<.0001f);
 Mat4 steered=roadWheelMatrix(1,2,0,flat,0,0,.3f,0); assert(steered.m[8]>.29f);
 Mat4 spun=roadWheelMatrix(1,2,0,flat,0,0,0,.3f); assert(std::fabs(spun.m[6])>.29f);
 float uneven[4]={10,10.10f,10,10.10f}; Mat4 left=roadWheelMatrix(0,0,0,uneven,0,0,0,0), right=roadWheelMatrix(0,0,0,uneven,0,1,0,0);
 assert(left.m[1]>.05f && std::fabs((right.m[13]-left.m[13])-.10f)<.0001f);
 puts("PASS: independent wheel position, steering, rotation and road bank");
}

