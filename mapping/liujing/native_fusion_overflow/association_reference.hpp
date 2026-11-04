#pragma once
#include "fusion_association.hpp"
#include <fstream>
#include <cstring>
#include <array>
namespace lj2 {
// Executable DDR ABI reference and real-address cache model. Used by replay,
// never silently substituted for a requested hardware backend.
class AssociationReference:public AssociationBackend {
 std::vector<AssociationCell>table_;uint64_t version_=0;std::string prefix_;
 static float fp(uint32_t x){float f;std::memcpy(&f,&x,4);return f;}
 static bool equal(const AssociationCell&c,Key k){return int32_t(c.word[0])==k.x&&int32_t(c.word[1])==k.y&&int32_t(c.word[2])==k.z;}
public:
 explicit AssociationReference(std::string prefix,size_t slots=2097152):table_(slots),prefix_(std::move(prefix)){if(!slots||(slots&(slots-1)))throw std::invalid_argument("association table size");}
 void sync(const std::vector<AssociationUpdate>&updates,uint64_t version)override{
  if(version!=version_+1)throw std::runtime_error("association sync version");
  for(const auto&u:updates){uint32_t slot=association_hash(u.key)&(table_.size()-1),tomb=UINT32_MAX;bool done=false;
   for(uint32_t probe=0;probe<64;probe++,slot=(slot+1)&(table_.size()-1)){
    auto&c=table_[slot];auto n=c.word[3];if(n==association_tombstone){if(tomb==UINT32_MAX)tomb=slot;continue;}
    if(!n){if(u.cell.word[3])table_[tomb==UINT32_MAX?slot:tomb]=u.cell;done=true;break;}
    if(equal(c,u.key)){if(u.cell.word[3])c=u.cell;else c.word[3]=association_tombstone;done=true;break;}
   }if(!done)throw std::runtime_error("association table probe bound; rehash required");
  }version_=version;
 }
 void associate(std::vector<AssociationQuery>&q,const std::vector<AssociationGroup>&groups,uint64_t version)override{
  if(version!=version_)throw std::runtime_error("association read version");
  std::vector<std::vector<uint32_t>>members(groups.size());for(uint32_t i=0;i<q.size();i++)members.at(q[i].group).push_back(i);
  std::vector<AssociationGroupWord>packed_groups;std::vector<AssociationQueryWord>packed_queries;std::vector<int32_t>expected;
  std::array<uint32_t,4096>tags;tags.fill(UINT32_MAX);uint64_t probes=0,misses=0,candidates=0;uint32_t max_probe=0;
  for(size_t g=0;g<groups.size();g++){
   AssociationGroupWord desc{};Key key=groups[g].key;desc.word[0]=uint32_t(key.x);desc.word[1]=uint32_t(key.y);desc.word[2]=uint32_t(key.z);desc.word[3]=packed_queries.size();desc.word[4]=members[g].size();packed_groups.push_back(desc);
   std::vector<const AssociationCell*>neighbors;
   for(int z=-1;z<=1;z++)for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){
    Key k{key.x+x,key.y+y,key.z+z};uint32_t slot=association_hash(k)&(table_.size()-1);bool found=false;
    for(uint32_t probe=0;probe<64;probe++,slot=(slot+1)&(table_.size()-1)){
     ++probes;max_probe=std::max(max_probe,probe+1);auto line=slot&4095;if(tags[line]!=slot){++misses;tags[line]=slot;}
     const auto&c=table_[slot];auto n=c.word[3];if(!n){found=true;break;}if(n<=4&&equal(c,k)){neighbors.push_back(&c);found=true;break;}
    }if(!found)throw std::runtime_error("association lookup probe bound");
   }
   for(auto qi:members[g]){auto&p=q[qi].value;float distance=.025f*.025f;int32_t match=-1;
    for(auto*c:neighbors)for(uint32_t lane=0;lane<c->word[3];lane++){
     const auto*w=c->word+4+lane*7;float dx=p.x-fp(w[0]),dy=p.y-fp(w[1]),dz=p.z-fp(w[2]);float nx=fp(w[3]),ny=fp(w[4]),nz=fp(w[5]);float d=(dx*dx+dy*dy)+dz*dz;++candidates;
     if(d<distance&&((nx*p.nx+ny*p.ny)+nz*p.nz)>.94f&&std::fabs((dx*nx+dy*ny)+dz*nz)<.012f){distance=d;match=int32_t(w[6]);}
    }q[qi].match=match;expected.push_back(match);AssociationQueryWord query{};float values[6]={p.x,p.y,p.z,p.nx,p.ny,p.nz};std::memcpy(query.word,values,24);packed_queries.push_back(query);
   }
  }
  std::ofstream report(prefix_+"_access.jsonl",std::ios::app);report<<"{\"version\":"<<version<<",\"groups\":"<<groups.size()<<",\"queries\":"<<q.size()<<",\"probes\":"<<probes<<",\"max_probe\":"<<max_probe<<",\"cache_misses\":"<<misses<<",\"ddr_bytes\":"<<misses*128<<",\"candidates\":"<<candidates<<"}\n";
  if(version==30||version==90){
   std::ofstream f(prefix_+"_trace_"+std::to_string(version)+".bin",std::ios::binary);auto write=[&](const auto&v){f.write(reinterpret_cast<const char*>(&v),sizeof(v));};f.write("LJAT0001",8);uint32_t slots=table_.size(),occupied=0;for(auto&c:table_)if(c.word[3])++occupied;write(slots);write(occupied);uint32_t ng=groups.size(),nq=q.size();write(ng);write(nq);write(version);
   for(uint32_t i=0;i<table_.size();i++)if(table_[i].word[3]){write(i);write(table_[i]);}
   for(auto&g:packed_groups)write(g);for(auto&p:packed_queries)write(p);for(auto id:expected)write(id);if(!f)throw std::runtime_error("association trace write");
  }
 }
};
}
