// Standalone read-only sensor capture: no chassis interfaces and no inferred pose.
#include <libobsensor/ObSensor.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <cmath>
static void intr(std::ostream&o,const OBCameraIntrinsic&i,const OBCameraDistortion&d){
 o<<"{\"width\":"<<i.width<<",\"height\":"<<i.height<<",\"fx\":"<<i.fx<<",\"fy\":"<<i.fy<<",\"cx\":"<<i.cx<<",\"cy\":"<<i.cy<<",\"near_z\":0.3,\"far_z\":5.0,\"distortion\":["<<d.k1<<","<<d.k2<<","<<d.p1<<","<<d.p2<<","<<d.k3<<","<<d.k4<<","<<d.k5<<","<<d.k6<<"]}";
}
int main(int argc,char**argv){
 if(argc!=3){std::cerr<<"camera_capture NEW_OUTPUT_DIRECTORY FRAME_COUNT\n";return 2;}
 std::filesystem::path root(argv[1]);int wanted=std::stoi(argv[2]);if(wanted<1||wanted>600)return 2;
 if(std::filesystem::exists(root)){std::cerr<<"Output exists; refusing overwrite\n";return 2;}std::filesystem::create_directories(root);
 try{
  ob::Pipeline pipe;auto cfg=std::make_shared<ob::Config>();
  cfg->enableStream(pipe.getStreamProfileList(OB_SENSOR_COLOR)->getVideoStreamProfile(640,480,OB_FORMAT_MJPG,30));
  cfg->enableStream(pipe.getStreamProfileList(OB_SENSOR_DEPTH)->getVideoStreamProfile(640,400,OB_FORMAT_Y11,30));
  cfg->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);pipe.start(cfg);
  auto p=pipe.getCameraParam();auto device=pipe.getDevice()->getDeviceInfo();
  std::ofstream cal(root/"factory_calibration.json");cal<<std::setprecision(10)<<"{\"source\":\"Orbbec SDK getCameraParam; factory calibration, not extrinsic acceptance\",\"serial\":\""<<device->serialNumber()<<"\",\"mirrored\":"<<(p.isMirrored?"true":"false")<<",\"depth\":";intr(cal,p.depthIntrinsic,p.depthDistortion);cal<<",\"color\":";intr(cal,p.rgbIntrinsic,p.rgbDistortion);cal<<",\"rotation_raw\":[";
  for(int i=0;i<9;i++){if(i)cal<<",";cal<<p.transform.rot[i];}cal<<"],\"translation_raw\":["<<p.transform.trans[0]<<","<<p.transform.trans[1]<<","<<p.transform.trans[2]<<"]}";cal.close();
  std::ofstream meta(root/"frames.jsonl");int count=0,nc=0,nd=0;auto start=std::chrono::steady_clock::now();uint64_t prevc=~uint64_t(0),prevd=~uint64_t(0);
  std::shared_ptr<ob::ColorFrame> c;std::shared_ptr<ob::DepthFrame>d;long long cm=0,dm=0;
  while(count<wanted&&std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<60){
   auto fs=pipe.waitForFrames(1000);if(!fs)continue;
   auto received=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
   if(fs->colorFrame()){c=fs->colorFrame();cm=received;nc++;}if(fs->depthFrame()){d=fs->depthFrame();dm=received;nd++;}
   if(!c||!d||c->index()==prevc||d->index()==prevd)continue;
   auto mono=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
   auto ct=c->timeStampUs(),dt=d->timeStampUs();if(std::llabs(cm-dm)>30000000)continue;
   cv::Mat encoded(1,c->dataSize(),CV_8UC1,c->data());auto rgb=cv::imdecode(encoded,cv::IMREAD_COLOR);if(rgb.empty())continue;
   cv::Mat depth(d->height(),d->width(),CV_16UC1);auto src=static_cast<uint16_t*>(d->data());if(d->dataSize()!=depth.total()*2)throw std::runtime_error("depth payload size");
   float scale=d->getValueScale();for(size_t j=0;j<depth.total();j++)depth.ptr<uint16_t>()[j]=(uint16_t)std::min(65535.f,src[j]*scale+.5f);
   std::string stem=std::to_string(count);cv::imwrite((root/(stem+"_rgb.png")).string(),rgb);cv::imwrite((root/(stem+"_depth.png")).string(),depth);
   meta<<"{\"index\":"<<count<<",\"pairing_basis\":\"host arrival within 30ms; device clock domains not accepted\",\"color_host_ns\":"<<cm<<",\"depth_host_ns\":"<<dm<<",\"color_device_us\":"<<ct<<",\"depth_device_us\":"<<dt<<",\"color_sequence\":"<<c->index()<<",\"depth_sequence\":"<<d->index()<<",\"host_monotonic_ns\":"<<mono<<",\"host_wall_ns\":"<<wall<<",\"depth_scale_to_mm\":"<<scale<<"}\n";meta.flush();prevc=c->index();prevd=d->index();count++;
  }
  pipe.stop();std::cout<<"captured="<<count<<" requested="<<wanted<<" color_observations="<<nc<<" depth_observations="<<nd<<"\n";return count==wanted?0:3;
 }catch(const ob::Error&e){std::cerr<<e.getMessage()<<"\n";return 1;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
