#include <libobsensor/ObSensor.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
int main(int argc,char**argv){
 if(argc!=2)return 2;
 int lease=open("/home/ubuntu/liujing/live.lock",O_RDWR|O_CREAT,0644);if(lease<0||flock(lease,LOCK_EX|LOCK_NB))return 3;
 ob::Pipeline pipe;auto cfg=std::make_shared<ob::Config>();cfg->enableStream(pipe.getStreamProfileList(OB_SENSOR_COLOR)->getVideoStreamProfile(640,480,OB_FORMAT_MJPG,30));cfg->enableStream(pipe.getStreamProfileList(OB_SENSOR_DEPTH)->getVideoStreamProfile(640,400,OB_FORMAT_Y11,30));cfg->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
 bool global=pipe.getDevice()->isGlobalTimestampSupported();std::ofstream out(argv[1]);out<<std::setprecision(17);pipe.start(cfg);auto start=std::chrono::steady_clock::now();bool delayed=false;
 while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<20){
  double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();if(!delayed&&elapsed>8){std::this_thread::sleep_for(std::chrono::milliseconds(180));delayed=true;}
  auto fs=pipe.waitForFrames(200);if(!fs)continue;double mono=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),wall=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  auto write=[&](const char*name,std::shared_ptr<ob::Frame>frame){if(!frame)return;out<<"{\"sensor\":\""<<name<<"\",\"id\":"<<frame->index()<<",\"received\":"<<mono<<",\"wall\":"<<wall<<",\"device\":"<<frame->timeStampUs()<<",\"system_ms\":"<<frame->systemTimeStamp()<<",\"global_supported\":"<<(global?"true":"false")<<",\"global_us\":"<<(global?frame->globalTimeStampUs():0)<<",\"delay_injected\":"<<(delayed?"true":"false")<<"}\n";};write("rgb",fs->colorFrame());write("depth",fs->depthFrame());
 }pipe.stop();close(lease);return out?0:4;
}
