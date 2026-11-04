#pragma once
#include "fusion_association.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <linux/dma-buf.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <fstream>
#include <thread>
namespace lj2 {
class AssociationPl final:public AssociationBackend {
 struct Buffer {uint8_t*data=nullptr;size_t bytes=0;uint64_t physical=0;int fd=-1;};
 std::map<std::string,Buffer>buffers_;std::map<std::string,uint32_t>regs_;
 int mem_=-1,lease_=-1;volatile uint32_t*control_=nullptr;size_t control_size_=0;
 std::vector<AssociationCell>mirror_;uint64_t version_=0;uint32_t epoch_=0,surface_count_=0;
 static double now(){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();}
 static void barrier(){
#if defined(__aarch64__)
  asm volatile("dsb sy":::"memory");
#else
  std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
 }
 static void sync(Buffer&b,bool begin,bool write){dma_buf_sync s{};s.flags=(begin?DMA_BUF_SYNC_START:DMA_BUF_SYNC_END)|(write?DMA_BUF_SYNC_WRITE:DMA_BUF_SYNC_READ);if(ioctl(b.fd,DMA_BUF_IOCTL_SYNC,&s))throw std::runtime_error("association DMA synchronization failed");}
 void reg(const std::string&name,uint32_t value){auto offset=regs_.at(name);if(offset%4||offset+4>control_size_)throw std::runtime_error("association register bounds");control_[offset/4]=value;}
 void fault(const char*reason){int f=open("/run/lock/liujing_raster_fault.json",O_CREAT|O_EXCL|O_WRONLY,0644);if(f>=0){std::string text="{\"reason\":\""+std::string(reason)+"\",\"recovery\":\"reboot required; keep CMA pinned\"}\n";auto written=write(f,text.data(),text.size());(void)written;fsync(f);close(f);}throw std::runtime_error(reason);}
 template<class T>void upload(const char*name,const std::vector<T>&v){auto&b=buffers_.at(name);size_t n=v.size()*sizeof(T);if(n>b.bytes)throw std::runtime_error("association buffer capacity");sync(b,true,true);if(n)std::memcpy(b.data,v.data(),n);sync(b,false,true);host_write_bytes+=n;}
public:
 uint64_t completed=0,host_write_bytes=0,dirty_cells=0;double execute_ms=0,packing_ms=0,sync_ms=0;std::array<uint32_t,8>stats{};
 // Explicit diagnostic only: distinguish writes from an earlier successful
 // job from partial output produced by a subsequently stalled transaction.
 void diagnostic_reset_matches(){auto&b=buffers_.at("matches");sync(b,true,true);std::memset(b.data,0xa5,b.bytes);sync(b,false,true);barrier();}
 explicit AssociationPl(const std::string&manifest){
  if(access("/run/lock/liujing_raster_fault.json",F_OK)==0)throw std::runtime_error("hardware fault latch requires reboot");
  lease_=open("/run/lock/liujing_native_renderer.lock",O_CREAT|O_RDWR,0644);if(lease_<0||flock(lease_,LOCK_EX|LOCK_NB))throw std::runtime_error("native accelerator already owned");
  boost::property_tree::ptree p;boost::property_tree::read_json(manifest,p);
  if(p.get<std::string>("contract")!="fusion_association_v1"||p.get<std::string>("transport")!="inherited_dma_buf_v1"||p.get<bool>("cacheable"))throw std::runtime_error("association ABI mismatch");
  std::string boot;std::ifstream("/proc/sys/kernel/random/boot_id")>>boot;if(p.get<std::string>("boot_id")!=boot||kill(p.get<int>("owner_pid"),0))throw std::runtime_error("stale association allocation");
  mem_=open("/dev/mem",O_RDWR|O_SYNC);if(mem_<0)throw std::runtime_error("association control open");control_size_=p.get<size_t>("control.nbytes");auto address=p.get<uint64_t>("control.address");if(address%sysconf(_SC_PAGESIZE))throw std::runtime_error("unaligned control mapping");
  void*base=mmap(nullptr,control_size_,PROT_READ|PROT_WRITE,MAP_SHARED,mem_,address);if(base==MAP_FAILED)throw std::runtime_error("association control mmap");control_=static_cast<volatile uint32_t*>(base);
  for(auto&r:p.get_child("registers"))regs_[r.first]=r.second.get_value<uint32_t>();
  for(auto&e:p.get_child("buffers")){Buffer b;b.bytes=e.second.get<size_t>("nbytes");b.physical=e.second.get<uint64_t>("address");b.fd=e.second.get<int>("fd");void*m=mmap(nullptr,b.bytes,PROT_READ|PROT_WRITE,MAP_SHARED,b.fd,0);if(m==MAP_FAILED)throw std::runtime_error("association DMA mmap");b.data=static_cast<uint8_t*>(m);buffers_.emplace(e.first,b);reg(e.first,uint32_t(b.physical));reg(e.first+"_high",uint32_t(b.physical>>32));}
  if(buffers_.at("table").bytes!=2097152*sizeof(AssociationCell))throw std::runtime_error("association table contract");
  // The second search lane reads the same immutable allocation through its
  // independent AXI master. No duplicated table or per-frame map transfer.
  if(regs_.count("table_b")){auto physical=buffers_.at("table").physical;reg("table_b",uint32_t(physical));reg("table_b_high",uint32_t(physical>>32));}
  mirror_.resize(2097152);auto&b=buffers_.at("table");sync(b,true,true);std::memset(b.data,0,b.bytes);sync(b,false,true);barrier();
 }
 void associate(std::vector<AssociationQuery>&q,const std::vector<AssociationGroup>&groups,uint64_t version)override{
  if(version!=version_||epoch_==UINT32_MAX||q.size()>64000||groups.size()>64000)throw std::runtime_error("association version or capacity");auto start=now();host_write_bytes=0;
  std::vector<AssociationGroupWord>g(groups.size());std::vector<AssociationQueryWord>queries(q.size());std::vector<uint32_t>cursor(groups.size()),scatter(q.size());
  for(auto&p:q)++g.at(p.group).word[4];uint32_t offset=0;
  for(size_t i=0;i<groups.size();i++){auto&w=g[i].word;w[0]=uint32_t(groups[i].key.x);w[1]=uint32_t(groups[i].key.y);w[2]=uint32_t(groups[i].key.z);w[3]=offset;cursor[i]=offset;offset+=w[4];}
  for(uint32_t i=0;i<q.size();i++){auto at=cursor[q[i].group]++;scatter[at]=i;auto&p=q[i].value;float values[6]={p.x,p.y,p.z,p.nx,p.ny,p.nz};std::memcpy(queries[at].word,values,24);}
  upload("groups",g);upload("queries",queries);reg("group_count",g.size());reg("query_count",q.size());reg("table_mask",mirror_.size()-1);reg("epoch",++epoch_);packing_ms=now()-start;
  auto&sb=buffers_.at("stats");sync(sb,true,true);std::memset(sb.data,0xff,32);sync(sb,false,true);barrier();auto begin=now();control_[0]=1;
  while(!(control_[0]&2)){if(now()-begin>1000)fault("association AP_DONE timeout");std::this_thread::sleep_for(std::chrono::microseconds(200));}barrier();execute_ms=now()-begin;
  sync(sb,true,false);std::memcpy(stats.data(),sb.data,32);sync(sb,false,false);
  if(stats[0]||stats[3]!=q.size()||stats[4]!=epoch_)fault("association status or version mismatch");
  auto&matches=buffers_.at("matches");sync(matches,true,false);auto*ids=reinterpret_cast<const int32_t*>(matches.data);bool invalid_id=false;for(size_t i=0;i<q.size();i++){auto id=ids[i];invalid_id|=id< -1||(id>=0&&uint32_t(id)>=surface_count_);q[scatter[i]].match=id;}sync(matches,false,false);if(invalid_id)fault("association returned invalid surface ID");++completed;
 }
 void sync(const std::vector<AssociationUpdate>&updates,uint64_t version)override{
  if(version!=version_+1)throw std::runtime_error("association commit version");auto start=now();std::vector<uint32_t>dirty;dirty.reserve(updates.size());
  for(auto&u:updates){if(u.cell.word[3]>4)throw std::runtime_error("association cell capacity");for(uint32_t i=0;i<u.cell.word[3];i++){auto id=u.cell.word[10+i*7];if(id>=1000000)throw std::runtime_error("association surface capacity");surface_count_=std::max(surface_count_,id+1);}uint32_t slot=association_hash(u.key)&(mirror_.size()-1),tomb=UINT32_MAX;bool done=false;
   for(uint32_t probe=0;probe<64;probe++,slot=(slot+1)&(mirror_.size()-1)){auto&c=mirror_[slot];auto n=c.word[3];if(n==association_tombstone){if(tomb==UINT32_MAX)tomb=slot;continue;}
    if(!n){if(u.cell.word[3]){auto at=tomb==UINT32_MAX?slot:tomb;mirror_[at]=u.cell;dirty.push_back(at);}done=true;break;}
    if(int32_t(c.word[0])==u.key.x&&int32_t(c.word[1])==u.key.y&&int32_t(c.word[2])==u.key.z){if(u.cell.word[3])c=u.cell;else c.word[3]=association_tombstone;dirty.push_back(slot);done=true;break;}
   }if(!done)throw std::runtime_error("association probe bound; rehash required");
  }
  std::sort(dirty.begin(),dirty.end());dirty.erase(std::unique(dirty.begin(),dirty.end()),dirty.end());auto&b=buffers_.at("table");sync(b,true,true);for(auto slot:dirty)std::memcpy(b.data+slot*sizeof(AssociationCell),&mirror_[slot],sizeof(AssociationCell));sync(b,false,true);barrier();version_=version;dirty_cells=dirty.size();sync_ms=now()-start;host_write_bytes+=dirty.size()*sizeof(AssociationCell);
 }
 ~AssociationPl(){for(auto&e:buffers_){munmap(e.second.data,e.second.bytes);close(e.second.fd);}if(control_)munmap(const_cast<uint32_t*>(control_),control_size_);if(mem_>=0)close(mem_);if(lease_>=0)close(lease_);}
};
}
