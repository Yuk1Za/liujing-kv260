// Adapted from frozen sensor_bringup_20261021 camera driver. Original untouched.
#include <libobsensor/ObSensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <chrono>
#include <cmath>
#include <cstring>

sensor_msgs::msg::CameraInfo info(const OBCameraIntrinsic &i, const OBCameraDistortion &d) {
  sensor_msgs::msg::CameraInfo m;
  m.width=i.width; m.height=i.height; m.distortion_model="rational_polynomial";
  m.d={d.k1,d.k2,d.p1,d.p2,d.k3,d.k4,d.k5,d.k6};
  m.k={i.fx,0,i.cx,0,i.fy,i.cy,0,0,1}; m.r={1,0,0,0,1,0,0,0,1};
  m.p={i.fx,0,i.cx,0,0,i.fy,i.cy,0,0,0,1,0}; return m;
}
int main(int argc,char**argv) {
 rclcpp::init(argc,argv); auto node=std::make_shared<rclcpp::Node>("liujing_camera");
 auto qos=rclcpp::SensorDataQoS();
 auto metadata=node->create_publisher<std_msgs::msg::String>("/camera/frame_metadata",qos);
 uint64_t last_color=~uint64_t(0),last_depth=~uint64_t(0);
 auto rgb=node->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw",qos);
 auto dep=node->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw",qos);
 auto jpg=node->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/color/image_raw/compressed",qos);
 auto ci=node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info",qos);
 auto di=node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info",qos);
 try {
  ob::Pipeline pipe; auto device=pipe.getDevice()->getDeviceInfo();
  std::cout<<"DEVICE "<<device->name()<<" serial="<<device->serialNumber()<<std::endl;
  auto cfg=std::make_shared<ob::Config>();
  cfg->enableStream(pipe.getStreamProfileList(OB_SENSOR_COLOR)->getVideoStreamProfile(640,480,OB_FORMAT_MJPG,30));
  cfg->enableStream(pipe.getStreamProfileList(OB_SENSOR_DEPTH)->getVideoStreamProfile(640,400,OB_FORMAT_Y11,30));
  cfg->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
  pipe.start(cfg);
  auto p=pipe.getCameraParam(); auto cinfo=info(p.rgbIntrinsic,p.rgbDistortion), dinfo=info(p.depthIntrinsic,p.depthDistortion);
  uint64_t nc=0,nd=0; auto start=std::chrono::steady_clock::now(),report=start;
  while(rclcpp::ok()) {
   auto fs=pipe.waitForFrames(1000); if(!rclcpp::ok()) break; if(!fs) continue;
   auto received=node->now();
   auto received_mono=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
   auto c=fs->colorFrame(); auto d=fs->depthFrame();
   auto publish_meta=[&](const char* stream, auto frame){
    std::ostringstream out;out<<"{\"stream\":\""<<stream<<"\",\"sequence\":"<<frame->index()<<",\"device_timestamp_us\":"<<frame->timeStampUs()<<",\"sdk_system_timestamp_ms\":"<<frame->systemTimeStamp()<<",\"host_wall_ns\":"<<received.nanoseconds()<<",\"host_monotonic_ns\":"<<received_mono<<",\"ros_stamp_basis\":\"first host observation, not hardware synchronization\"}";
    std_msgs::msg::String msg;msg.data=out.str();metadata->publish(msg);
   };
   if(c && c->index()!=last_color) {
    last_color=c->index(); publish_meta("color",c);
    cv::Mat encoded(1,c->dataSize(),CV_8UC1,c->data()); auto image=cv::imdecode(encoded,cv::IMREAD_COLOR);
    if(!image.empty()) {
     sensor_msgs::msg::Image m; m.header.stamp=received; m.header.frame_id="camera_color_optical_frame";
     m.width=image.cols;m.height=image.rows;m.encoding="bgr8";m.step=image.cols*3;
     m.data.assign(image.data,image.data+image.total()*image.elemSize());rgb->publish(m);
     cinfo.header=m.header;ci->publish(cinfo);
     sensor_msgs::msg::CompressedImage j;j.header=m.header;j.format="bgr8; jpeg compressed bgr8";
     auto b=static_cast<uint8_t*>(c->data());j.data.assign(b,b+c->dataSize());jpg->publish(j);nc++;
    }
   }
   if(d && d->index()!=last_depth) {
    last_depth=d->index(); publish_meta("depth",d);
    sensor_msgs::msg::Image m;m.header.stamp=received;m.header.frame_id="camera_depth_optical_frame";
    m.width=d->width();m.height=d->height();m.encoding="16UC1";m.step=m.width*2;
    size_t n=m.width*m.height;
    if(d->dataSize()!=n*2) throw std::runtime_error("Unexpected decoded depth size");
    m.data.resize(n*2);auto src=static_cast<uint16_t*>(d->data());auto dst=reinterpret_cast<uint16_t*>(m.data.data());
    const float scale=d->getValueScale();
    if(scale==1.0f) std::memcpy(dst,src,n*2);
    else for(size_t k=0;k<n;k++) dst[k]=static_cast<uint16_t>(std::min(65535.0f,src[k]*scale+0.5f));
    dep->publish(m);dinfo.header=m.header;di->publish(dinfo);nd++;
   }
   if(rclcpp::ok()) rclcpp::spin_some(node);
   auto now=std::chrono::steady_clock::now();
   if(std::chrono::duration<double>(now-report).count()>10) {
    double secs=std::chrono::duration<double>(now-start).count();
    std::cout<<"color="<<nc<<" depth="<<nd<<" color_hz="<<nc/secs<<" depth_hz="<<nd/secs<<std::endl;report=now;
   }
  }
  pipe.stop();
 } catch(const ob::Error &e) {std::cerr<<e.getMessage()<<std::endl;return 1;}
 catch(const std::exception &e) {std::cerr<<e.what()<<std::endl;return 1;}
 rclcpp::shutdown();return 0;
}
