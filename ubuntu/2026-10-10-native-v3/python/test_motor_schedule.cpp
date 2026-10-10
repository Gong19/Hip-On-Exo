#include "hipexo_motor_schedule.h"
#include <cassert>
int main(){
 MotorSchedule s(1000000,1000);
 assert(s.due()==1000000);assert(s.sent(1200000,1200000)==0);
 assert(s.due()==2000000);assert(s.sent(2600000,2600000)==0);
 assert(s.due()==3350000);assert(s.sent(3350000,3350000)==0);
 assert(s.due()==4100000);assert(s.sent(4100000,4100000)==0);
 assert(s.due()==5000000);
 assert(s.sent(8400000,8400000)==3);assert(s.due()==9150000);
 for(unsigned hz: {950u,995u,1000u}){
  MotorSchedule t(1000000,hz);uint64_t last=0;unsigned count=0;
  for(;;){uint64_t now=t.due()+(count%7==0?400000:0);if(now>=1001000000)break;
   if(last)assert(now-last>=750000);assert(t.sent(now,now)==0);last=now;++count;
  }
  assert(count>=hz && count<=hz+1); // Integer-nanosecond rounding at the window boundary.
 }
}
