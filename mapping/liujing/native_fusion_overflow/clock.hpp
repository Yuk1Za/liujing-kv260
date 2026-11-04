#pragma once
#include <cmath>
#include <cstdint>
#include <deque>
#include <algorithm>
namespace lj2 {
// Guarded normalization for the observed legacy UVC 32-bit counter rollover,
// expressed by this SDK in units multiplied by 1000. Accept a rollover only
// when its increment agrees with the preceding per-frame increments; all
// other regressions remain faults. Host-receipt checks still run afterwards.
class LegacyRgbTicks {
 uint64_t previous_=0,id_=0,offset_=0,wraps_=0;bool seen_=false,fault_=false;std::deque<double>steps_;
public:
 bool observe(uint64_t raw,uint64_t id,uint64_t&mapped){
  constexpr uint64_t modulus=4294967296000ULL;
  if(fault_)return false;
  if(seen_){
   if(id<=id_){fault_=true;return false;}auto gap=id-id_;double step=0;
   if(raw<previous_){auto sorted=steps_;std::sort(sorted.begin(),sorted.end());double expected=sorted.empty()?0:sorted[sorted.size()/2]*gap;
    if(raw>UINT64_MAX-modulus||raw+modulus<previous_||gap>30||expected<=0||std::fabs(double(raw+modulus-previous_)-expected)>std::max(expected*.02,2000000.)){fault_=true;return false;}
    if(offset_>UINT64_MAX-modulus){fault_=true;return false;}step=double(raw+modulus-previous_)/gap;offset_+=modulus;++wraps_;
   }else step=double(raw-previous_)/gap;
   if(step>0){steps_.push_back(step);if(steps_.size()>16)steps_.pop_front();}
  }if(raw>UINT64_MAX-offset_){fault_=true;return false;}seen_=true;previous_=raw;id_=id;mapped=raw+offset_;return true;
 }
 bool fault()const{return fault_;}uint64_t wraps()const{return wraps_;}
};
// Device units are learned from growth against host monotonic reception time.
// This estimates transport timing; it does NOT certify exposure synchronization.
class ClockMapper {
 struct Sample{double device,host;};std::deque<Sample> samples_;bool fault_=false;
 double scale_=0,intercept_=0,residual_=0,origin_device_=0,last_device_=0,last_host_=0,last_good_host_=0;
 bool seen_=false;uint64_t rejected_=0,reacquired_=0;
public:
 bool observe(double device,double host,double&mapped){
  if(fault_||!std::isfinite(device)||!std::isfinite(host))return false;
  // Duplicate timestamps do not supply a new timing sample. Reject that frame,
  // while retaining the mapping for the next strictly increasing observation.
  if(seen_&&device==last_device_)return false;
  if(seen_&&(device<=last_device_||host<last_host_)){fault_=true;return false;}
  last_device_=device;last_host_=host;seen_=true;
  // Once calibrated, reject a delayed arrival before it contaminates the
  // four-second regression window. This frame is NOT eligible for fusion.
  // A persistent offset is not accepted using stale calibration: reacquire.
  if(scale_&&std::fabs(host-(intercept_+scale_*(device-origin_device_)))>.02){
   ++rejected_;if(host-last_good_host_<=.5)return false;
   samples_.clear();scale_=0;++reacquired_;
  }
  samples_.push_back({device,host});while(samples_.size()>120)samples_.pop_front();
  if(samples_.size()<60||host-samples_.front().host<1.5)return false;
  auto origin=samples_.front();double x=0,y=0,xx=0,xy=0,n=samples_.size();
  for(auto s:samples_){double a=s.device-origin.device,b=s.host-origin.host;x+=a;y+=b;xx+=a*a;xy+=a*b;}
  double denominator=xx-x*x/n;if(denominator<=0)return false;
  double scale=(xy-x*y/n)/denominator,offset=(y-scale*x)/n;
  residual_=0;for(auto s:samples_){double e=(s.host-origin.host)-(offset+scale*(s.device-origin.device));residual_+=e*e;}residual_=std::sqrt(residual_/n);
  if(!(scale>0)||residual_>.02)return false;
  if(scale_&&std::fabs(scale/scale_-1)>.05){scale_=0;return false;}
  scale_=scale;origin_device_=origin.device;intercept_=origin.host+offset;mapped=intercept_+scale*(device-origin.device);
  if(std::fabs(mapped-host)>.02){++rejected_;return false;}last_good_host_=host;return true;
 }
 double scale()const{return scale_;}double residual()const{return residual_;}bool fault()const{return fault_;}double last_device()const{return last_device_;}
 uint64_t rejected_arrivals()const{return rejected_;}uint64_t reacquisitions()const{return reacquired_;}
};
}
