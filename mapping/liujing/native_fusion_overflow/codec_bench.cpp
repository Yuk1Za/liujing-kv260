#include "encode.hpp"
#include <chrono>
#include <ctime>
#include <iostream>
using namespace lj2;
int main(){
 cv::setNumThreads(1);Image image;image.width=640;image.height=480;image.rgb.resize(640*480);Camera camera;std::deque<std::array<float,3>>path;for(int i=0;i<6000;i++)path.push_back({float(i%100)*.01f,float(i/100)*.01f,2});
 for(int pattern=0;pattern<2;pattern++){
  uint32_t random=12345;for(size_t i=0;i<image.rgb.size();i++){random^=random<<13;random^=random>>17;random^=random<<5;image.rgb[i]=pattern?(random&0xffffff):uint32_t((i%640)/3)|uint32_t((i/640)/2)<<8|0x880000;}
  for(int iteration=0;iteration<20;iteration++){
   auto start=std::chrono::steady_clock::now();auto cpu=std::clock();auto encoded=encode_image(image,camera,identity(),path,false);
   double wall=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count(),process=1000.*(std::clock()-cpu)/CLOCKS_PER_SEC;
   if(iteration==0){auto decoded=cv::imdecode(encoded.bytes,cv::IMREAD_COLOR);if(decoded.cols!=640||decoded.rows!=480)return 1;auto pixel=encoded.bgr.at<cv::Vec3b>(0,0);auto rgb=image.rgb[0];if(pixel[0]!=((rgb>>16)&255)||pixel[1]!=((rgb>>8)&255)||pixel[2]!=(rgb&255))return 2;}
   std::cout<<"{\"scope\":\"synthetic_memory_only_same_runtime_encoder\",\"pattern\":\""<<(pattern?"noise":"gradient")<<"\",\"iteration\":"<<iteration<<",\"overlay_and_encode_ms\":"<<wall<<",\"process_ms\":"<<process<<",\"convert_ms\":"<<encoded.convert_ms<<",\"overlay_ms\":"<<encoded.overlay_ms<<",\"jpeg_ms\":"<<encoded.jpeg_ms<<",\"jpeg_bytes\":"<<encoded.bytes.size()<<",\"trajectory_points\":"<<path.size()<<"}\n";
  }
 }
}
