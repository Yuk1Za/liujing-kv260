#include "association_pl.hpp"
#include <filesystem>
#include <iostream>
using namespace lj2;
template<class T>std::vector<T>read(const std::filesystem::path&p){
 auto size=std::filesystem::file_size(p);if(size%sizeof(T)||size/sizeof(T)>64000)throw std::runtime_error("probe file bound");
 std::vector<T>v(size/sizeof(T));std::ifstream f(p,std::ios::binary);if(!f.read(reinterpret_cast<char*>(v.data()),size))throw std::runtime_error("probe input");return v;
}
int main(int argc,char**argv){
 if(argc!=3||!getenv("LIUJING_FUSION_PL"))return 2;
 auto packed=read<AssociationGroupWord>(std::filesystem::path(argv[1])/"fault_groups.bin");
 auto points=read<AssociationQueryWord>(std::filesystem::path(argv[1])/"fault_queries.bin");
 AssociationPl pl(getenv("LIUJING_FUSION_PL"));std::ofstream log(std::string(argv[2])+".jsonl");if(!log)throw std::runtime_error("probe log");
 // No map writes or versions are submitted. Each job uses a new cache epoch
 // over the same empty committed table, so every valid answer must be -1.
 std::vector<uint32_t>counts{0u,1u,4u,64u,6608u};
 if(const char*one=getenv("LJ3_PROBE_GROUP_COUNT")){auto n=std::stoul(one);if(n>64000)throw std::runtime_error("probe count bound");counts={uint32_t(n)};}
 for(uint32_t ng:counts){
  if(ng>packed.size())throw std::runtime_error("probe group bound");
  std::vector<AssociationGroup>groups;std::vector<AssociationQuery>queries;
  for(uint32_t g=0;g<ng;g++){
   auto*w=packed[g].word;groups.push_back({{int32_t(w[0]),int32_t(w[1]),int32_t(w[2])},0,0});
   if(w[3]!=queries.size()||w[3]>points.size()||w[4]>points.size()-w[3])throw std::runtime_error("probe descriptor");
   for(uint32_t i=0;i<w[4];i++){FusionSurfel s{};std::memcpy(&s,points[w[3]+i].word,24);s.weight=1;queries.push_back({s,-1,g,-1});}
  }
  std::cout<<"Starting empty-table probe groups="<<ng<<" queries="<<queries.size()<<std::endl;
  pl.diagnostic_reset_matches();pl.associate(queries,groups,0);
  for(auto&q:queries)if(q.match!=-1)throw std::runtime_error("nonempty match from empty map");
  log<<"{\"groups\":"<<ng<<",\"queries\":"<<queries.size()<<",\"execute_ms\":"<<pl.execute_ms<<",\"packing_ms\":"<<pl.packing_ms<<",\"mismatches\":0}\n";log.flush();
  if(!log)throw std::runtime_error("probe log write");
  std::cout<<"PASS empty-table probe groups="<<ng<<" execute_ms="<<pl.execute_ms<<std::endl;
 }
}
