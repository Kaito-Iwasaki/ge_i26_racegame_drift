#include "countdown.hpp"
#include "car.hpp"
#include <cassert>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const{return {};}
int main(){
    StartCountdown c;Track track;Car car;DriveInput input;input.throttle=1;
    const double dt=1.0/60.0;double raceTime=0;
    assert(c.number()==3&&!c.showGo());
    for(int i=0;i<180;++i){
        if(c.step(dt)){car.update(float(dt)*1.25f,input,track);raceTime+=dt;}
        assert(car.groundSpeed()==0 && raceTime==0);
        if(i==59)assert(c.number()==2);
        if(i==119)assert(c.number()==1);
    }
    assert(c.number()==0 && c.showGo());
    for(int i=0;i<100;++i)assert(!c.step(dt,true));
    assert(c.showGo() && c.goRemaining==StartCountdown::kGoDuration);
    assert(c.step(dt));car.update(float(dt)*1.25f,input,track);raceTime+=dt;
    assert(car.groundSpeed()>0&&raceTime>0);
    for(int i=0;i<60;++i)assert(c.step(dt));
    assert(!c.showGo()&&c.number()==0);
    c.reset();assert(c.number()==3&&!c.showGo());
    for(int i=0;i<30;++i)c.step(dt);
    const double before=c.remaining;
    for(int i=0;i<60;++i)assert(!c.step(dt,true));
    assert(c.remaining==before);
    c.reset();assert(c.remaining==3.0);
    std::puts("PASS: 3-2-1-GO timing, movement lock, race-time lock, pause, resume and reset");
}
