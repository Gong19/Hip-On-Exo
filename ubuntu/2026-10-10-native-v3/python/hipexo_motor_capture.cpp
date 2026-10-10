// GO-M8010-6 zero-output monitor. Uses the locally installed vendor codec only.
// Parent owns the serial lease; no active/position/torque commands are accepted.
#include "unitreeMotor/unitreeMotor.h"
#include <sys/socket.h>
#include <sys/prctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cstdio>
static uint64_t stamp(clockid_t c=CLOCK_MONOTONIC){timespec t;clock_gettime(c,&t);return uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;}
#pragma pack(push,1)
struct Row{uint64_t mono,wall;float q,dq,temp;int32_t error;};
struct Stats{uint64_t sent,valid,bad_bytes,skipped,tx_errors,queue_peak;};
#pragma pack(pop)
static_assert(sizeof(Row)==32,"IPC row layout");
static void append(std::vector<char>&b,const void*p,size_t n){const char*q=(const char*)p;b.insert(b.end(),q,q+n);}
int main(int argc,char**argv){
 if(argc!=4&&argc!=5)return 2;int ipc=atoi(argv[1]),id=atoi(argv[3]);
 int hz=argc==5?atoi(argv[4]):995;if(id<0||id>14||hz<100||hz>1000)return 2;
 int fd=-1;termios original{};bool restore=false;int result=0;
 try{
  // No transmission before the parent has applied scheduling and sent GO.
  fd_set ready;FD_ZERO(&ready);FD_SET(ipc,&ready);timeval limit{5,0};char go;
  if(select(ipc+1,&ready,0,0,&limit)<=0||recv(ipc,&go,1,0)!=1||go!='G')return 3;
  fd=open(argv[2],O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)throw std::runtime_error("open serial failed");
  if(tcgetattr(fd,&original))throw std::runtime_error("tcgetattr failed");restore=true;termios t=original;cfmakeraw(&t);
  cfsetispeed(&t,B4000000);cfsetospeed(&t,B4000000);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~(CSTOPB|CRTSCTS);t.c_cc[VMIN]=0;t.c_cc[VTIME]=0;
  if(tcsetattr(fd,TCSANOW,&t)||tcflush(fd,TCIOFLUSH))throw std::runtime_error("configure serial failed");
  const char*disabled=getenv("HIPEXO_DISABLE_TUNING");
  if(!disabled||strcmp(disabled,"1"))prctl(PR_SET_TIMERSLACK,1);MotorCmd cmd;cmd.motorType=MotorType::GO_M8010_6;cmd.id=id;cmd.mode=queryMotorMode(cmd.motorType,MotorMode::FOC);
  cmd.q=cmd.dq=cmd.kp=cmd.kd=cmd.tau=0;cmd.modify_data(&cmd);
  MotorData data;data.motorType=MotorType::GO_M8010_6;data.hex_len=16;
  Stats stats{};std::vector<Row> rows;rows.reserve(32);std::vector<char> pending;pending.reserve(65536);size_t consumed=0;
  char rx[4096];size_t used=0;uint64_t next=stamp(),last_flush=next,stop_at=0,send_until=UINT64_MAX;const uint64_t period=1000000000ULL/hz;uint64_t last_tx=0;
  auto flush=[&](){uint32_t size=sizeof(Stats)+rows.size()*sizeof(Row);append(pending,&size,4);append(pending,&stats,sizeof(stats));if(!rows.empty())append(pending,rows.data(),rows.size()*sizeof(Row));rows.clear();stats.queue_peak=std::max(stats.queue_peak,uint64_t(pending.size()-consumed));if(pending.size()-consumed>262144)throw std::runtime_error("motor IPC bounded queue overflow");};
  for(;;){auto now=stamp();
   char control;ssize_t r=recv(ipc,&control,1,MSG_DONTWAIT);
   if(r==0)break; // Parent gone: cease all TX immediately.
   if(r>0&&control=='S'&&!stop_at){send_until=now;stop_at=now+100000000;}
   if(r<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)break;
   if(now>=next&&now<send_until){
    if(last_tx && now-last_tx<750000){next=last_tx+750000;continue;}
    // Never burst overdue requests: it can collide with half-duplex responses.
    if(now-next>period){stats.skipped+=(now-next)/period;next=now;}
    auto n=write(fd,cmd.get_motor_send_data(),cmd.hex_len);
    if(n!=cmd.hex_len){stats.tx_errors++;throw std::runtime_error("partial motor command write");}
    stats.sent++;last_tx=stamp();next+=period;
   }
   if(now-last_flush>=20000000||rows.size()>=32){flush();last_flush=now;}
   if(consumed<pending.size()){
    ssize_t n=send(ipc,pending.data()+consumed,pending.size()-consumed,MSG_DONTWAIT|MSG_NOSIGNAL);
    if(n>0)consumed+=n;else if(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)break;
    if(consumed==pending.size()){pending.clear();consumed=0;}else if(consumed>65536){pending.erase(pending.begin(),pending.begin()+consumed);consumed=0;}
   }
   if(stop_at&&now>=stop_at){flush();break;}
   auto wait_now=stamp();uint64_t delay=now<send_until?(next>wait_now?next-wait_now:0):1000000;
   timespec timeout{0,(long)std::min(delay,uint64_t(1000000))};fd_set f;FD_ZERO(&f);FD_SET(fd,&f);FD_SET(ipc,&f);
   int count=pselect(std::max(fd,ipc)+1,&f,0,0,&timeout,0);
   if(count>0&&FD_ISSET(fd,&f)){
    ssize_t n=read(fd,rx+used,sizeof(rx)-used);if(n>0)used+=n;else if(n<0&&errno!=EAGAIN&&errno!=EINTR)throw std::runtime_error("serial read failed");
    size_t p=0;while(used-p>=16){
     if((uint8_t)rx[p]!=0xfd||(uint8_t)rx[p+1]!=0xee){p++;stats.bad_bytes++;continue;}
     memcpy(data.get_motor_recv_data(),rx+p,16);
     if(!data.extract_data(&data)||!data.correct||data.motor_id!=id){p++;stats.bad_bytes++;continue;}
     p+=16;stats.valid++;rows.push_back({stamp(),stamp(CLOCK_REALTIME),data.q,data.dq,float(data.temp),data.merror});
    }memmove(rx,rx+p,used-p);used-=p;
   }
  }
  // Bounded final drain; no commands are sent here.
  auto deadline=stamp()+1000000000ULL;
  while(consumed<pending.size()&&stamp()<deadline){ssize_t n=send(ipc,pending.data()+consumed,pending.size()-consumed,MSG_DONTWAIT|MSG_NOSIGNAL);if(n>0)consumed+=n;else if(errno==EAGAIN||errno==EWOULDBLOCK){timespec d{0,1000000};nanosleep(&d,0);}else break;}
  if(consumed<pending.size())throw std::runtime_error("final motor IPC drain incomplete");
 }catch(const std::exception&e){fprintf(stderr,"%s\n",e.what());result=1;}
 if(fd>=0){if(restore)tcsetattr(fd,TCSANOW,&original);close(fd);}close(ipc);return result;
}
