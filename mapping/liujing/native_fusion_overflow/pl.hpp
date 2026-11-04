#pragma once
#include "scene.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <cstring>
#include <fstream>
#include <atomic>
#include <signal.h>
namespace lj2 {
// CMA is allocated and pinned by the single overlay manager. Native hot path
// maps inherited DMA-BUF descriptors (strict /dev/mem excludes CMA); no Python calls, array conversions or cacheable aliases.
class Pl {
 struct Region{void*base=MAP_FAILED;size_t bytes=0;uint8_t*data=nullptr;uint64_t physical=0,capacity=0;int shared_fd=-1;};
 int fd_=-1,lease_=-1;std::map<std::string,Region>buffers_;Region control_;std::map<std::string,uint32_t>regs_;
 Region region(uint64_t address,size_t capacity,int shared_fd=-1){size_t page=sysconf(_SC_PAGESIZE),offset=shared_fd<0?address%page:0;Region r;r.physical=address;r.capacity=capacity;r.shared_fd=shared_fd;r.bytes=(capacity+offset+page-1)/page*page;r.base=mmap(nullptr,r.bytes,PROT_READ|PROT_WRITE,MAP_SHARED,shared_fd<0?fd_:shared_fd,shared_fd<0?address-offset:0);if(r.base==MAP_FAILED)throw std::runtime_error("PL mmap failed");r.data=(uint8_t*)r.base+offset;return r;}
 void reg(const std::string&name,uint32_t value){auto i=regs_.find(name);if(i==regs_.end()||i->second+4>control_.capacity)throw std::runtime_error("PL register contract");*reinterpret_cast<volatile uint32_t*>(control_.data+i->second)=value;}
 static uint32_t bits(float f){uint32_t u;std::memcpy(&u,&f,4);return u;}
 void timeout(){int f=open("/run/lock/liujing_raster_fault.json",O_WRONLY|O_CREAT|O_EXCL,0644);if(f>=0){const char*t="{\"reason\":\"opaque_v1 AP_DONE timeout\",\"recovery\":\"reboot required; CMA must remain pinned\"}\n";write(f,t,std::strlen(t));fsync(f);close(f);}throw std::runtime_error("PL timeout latched; reboot required");}
 void sync(Region&b,bool begin,bool write){dma_buf_sync request{};request.flags=(begin?DMA_BUF_SYNC_START:DMA_BUF_SYNC_END)|(write?DMA_BUF_SYNC_WRITE:DMA_BUF_SYNC_READ);if(ioctl(b.shared_fd,DMA_BUF_IOCTL_SYNC,&request))throw std::runtime_error("DMA-BUF synchronization failed");}
 static void barrier(){
#if defined(__aarch64__)
  asm volatile("dsb sy" ::: "memory");
#else
  std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
 }
 template<class T>void copy(const char*name,const std::vector<T>&v){auto&b=buffers_.at(name);if(v.size()*sizeof(T)>b.capacity)throw std::runtime_error("PL buffer capacity");sync(b,true,true);if(!v.empty())std::memcpy(b.data,v.data(),v.size()*sizeof(T));sync(b,false,true);}
public:
 uint64_t completed=0,host_write_bytes=0,host_read_bytes=0;double last_execute_ms=0;
 std::array<uint32_t,12> metrics{};
 explicit Pl(const std::string&config){
  if(access("/run/lock/liujing_raster_fault.json",F_OK)==0)throw std::runtime_error("PL fault latch: reboot required");
  lease_=open("/run/lock/liujing_native_renderer.lock",O_RDWR|O_CREAT,0644);if(lease_<0||flock(lease_,LOCK_EX|LOCK_NB))throw std::runtime_error("renderer already owned");
  boost::property_tree::ptree p;boost::property_tree::read_json(config,p);if(p.get<std::string>("contract")!="opaque_v1"||p.get<std::string>("transport")!="inherited_dma_buf_v1"||p.get<bool>("cacheable"))throw std::runtime_error("PL ABI/cache contract mismatch");std::string boot;std::ifstream("/proc/sys/kernel/random/boot_id")>>boot;if(p.get<std::string>("boot_id")!=boot)throw std::runtime_error("stale PL allocation manifest");if(kill(p.get<int>("owner_pid"),0))throw std::runtime_error("PL allocator owner not alive");
  fd_=open("/dev/mem",O_RDWR|O_SYNC);if(fd_<0)throw std::runtime_error("PL /dev/mem open failed");control_=region(p.get<uint64_t>("control.address"),p.get<size_t>("control.nbytes"));for(auto&r:p.get_child("registers"))regs_[r.first]=r.second.get_value<uint32_t>();for(auto&b:p.get_child("buffers"))buffers_[b.first]=region(b.second.get<uint64_t>("address"),b.second.get<size_t>("nbytes"),b.second.get<int>("fd"));
  if(p.get<std::string>("metrics_contract")!="passive_axi_metrics_v1"||*reinterpret_cast<volatile uint32_t*>(control_.data+regs_.at("metric_signature"))!=0x4c4a0101)throw std::runtime_error("PL metric signature mismatch");
 }
 Image render(const Prepared&p,bool debug=false){
  if(!p.budget_ok||p.surfaces.size()>40000||p.indices.size()>100000||p.candidate_pixels>4000000)throw std::runtime_error("PL work budget");
  copy("camera_surfels",p.surfaces);copy("offsets",p.offsets);copy("indices",p.indices);host_write_bytes=p.surfaces.size()*64+p.offsets.size()*4+p.indices.size()*4;
  for(auto&b:buffers_){reg(b.first,uint32_t(b.second.physical));reg(b.first+"_high",uint32_t(b.second.physical>>32));}
  auto&c=p.camera;reg("width",c.width);reg("height",c.height);reg("fx",bits(c.fx));reg("fy",bits(c.fy));reg("cx",bits(c.cx));reg("cy",bits(c.cy));reg("near_z",bits(c.near_z));reg("far_z",bits(c.far_z));reg("debug",debug);
  barrier();auto start=std::chrono::steady_clock::now();auto*ctrl=reinterpret_cast<volatile uint32_t*>(control_.data);*ctrl=1;
  while(!(*ctrl&2)){if(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>1)timeout();std::this_thread::sleep_for(std::chrono::microseconds(25));}
  barrier();last_execute_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();++completed;
  for(size_t i=0;i<metrics.size();i++)metrics[i]=*reinterpret_cast<volatile uint32_t*>(control_.data+regs_.at("metric_signature")+i*4);
  Image out;out.width=c.width;out.height=c.height;size_t n=c.width*c.height;out.rgb.resize(n);sync(buffers_.at("rgb"),true,false);std::memcpy(out.rgb.data(),buffers_.at("rgb").data,n*4);sync(buffers_.at("rgb"),false,false);host_read_bytes=n*4;
  if(debug){out.depth.resize(n);out.ids.resize(n);sync(buffers_.at("depth"),true,false);sync(buffers_.at("ids"),true,false);std::memcpy(out.depth.data(),buffers_.at("depth").data,n*4);std::memcpy(out.ids.data(),buffers_.at("ids").data,n*4);sync(buffers_.at("depth"),false,false);sync(buffers_.at("ids"),false,false);host_read_bytes+=n*8;}return out;
 }
 ~Pl(){for(auto&b:buffers_){if(b.second.base!=MAP_FAILED)munmap(b.second.base,b.second.bytes);if(b.second.shared_fd>=0)close(b.second.shared_fd);}if(control_.base!=MAP_FAILED)munmap(control_.base,control_.bytes);if(fd_>=0)close(fd_);if(lease_>=0)close(lease_);}
};
}
