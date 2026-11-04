#include "fusion_store.hpp"
#include "association_reference.hpp"
#include "association_pl.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace lj2;
// Validation-only wrapper: both backends read the exact same version and receive
// the exact same incremental updates. Its CPU cost is excluded from benchmarks
// by selecting plain "pl" for timed runs.
class CheckedAssociation final:public AssociationBackend {
 std::shared_ptr<AssociationPl>pl_;AssociationReference reference_;
public:
 uint64_t checked_queries=0,checked_versions=0;
 CheckedAssociation(std::shared_ptr<AssociationPl>pl,std::string prefix):pl_(std::move(pl)),reference_(std::move(prefix)){}
 void associate(std::vector<AssociationQuery>&q,const std::vector<AssociationGroup>&g,uint64_t v)override{
  auto expected=q;pl_->associate(q,g,v);reference_.associate(expected,g,v);
  for(size_t i=0;i<q.size();i++)if(q[i].match!=expected[i].match){
   std::ofstream fault("/run/lock/liujing_raster_fault.json");fault<<"{\"reason\":\"association CPU/PL mismatch\",\"version\":"<<v<<",\"query\":"<<i<<"}\n";fault.close();
   throw std::runtime_error("CPU/PL association mismatch at version "+std::to_string(v)+" query "+std::to_string(i));
  }
  checked_queries+=q.size();++checked_versions;
 }
 void sync(const std::vector<AssociationUpdate>&u,uint64_t v)override{pl_->sync(u,v);reference_.sync(u,v);}
};
int main(int argc,char**argv){
 if(argc<4){std::cerr<<"fusion_replay baseline|fast|abi|pl|pl-check capture-dir output-prefix\n";return 2;}
 const std::string mode=argv[1];const bool overflow=getenv("LJ3_CELL_OVERFLOW")&&std::string(getenv("LJ3_CELL_OVERFLOW"))=="1";if(overflow&&mode!="fast")throw std::invalid_argument("overflow experiment supports only explicit CPU fast backend");if(mode!="baseline"&&mode!="fast"&&mode!="abi"&&mode!="pl"&&mode!="pl-check")throw std::invalid_argument("replay backend");const bool baseline=std::string(argv[1])=="baseline";Scene old;std::shared_ptr<AssociationBackend>backend;if(std::string(argv[1])=="abi")backend=std::make_shared<AssociationReference>(argv[3]);if(mode=="pl"||mode=="pl-check"){const char*config=getenv("LIUJING_FUSION_PL");if(!config)throw std::runtime_error("explicit PL manifest required");backend=std::make_shared<AssociationPl>(config);}auto hardware=std::dynamic_pointer_cast<AssociationPl>(backend);std::shared_ptr<CheckedAssociation>checked;if(mode=="pl-check"){checked=std::make_shared<CheckedAssociation>(hardware,std::string(argv[3])+"_reference");backend=checked;}FusionStore fast(1000000,backend,getenv("LJ3_QUERY_CACHE_CAPACITY")?std::stoul(getenv("LJ3_QUERY_CACHE_CAPACITY")):0,getenv("LJ3_TILED_INDEX")&&std::string(getenv("LJ3_TILED_INDEX"))=="1");if(const char*threads=getenv("LJ3_ASSOC_THREADS"))fast.set_cpu_threads(std::stoi(threads));fast.set_coalesced_writes(getenv("LJ3_COALESCED_WRITES")&&std::string(getenv("LJ3_COALESCED_WRITES"))=="1");fast.enable_voxel_overflow(overflow);if(getenv("LJ3_FAST_TILES"))fast.set_fast_tiles(std::string(getenv("LJ3_FAST_TILES"))=="1");fast.set_query_simd(getenv("LJ3_QUERY_SIMD")&&std::string(getenv("LJ3_QUERY_SIMD"))=="1");std::vector<std::filesystem::path>paths;
 for(auto&p:std::filesystem::directory_iterator(argv[2]))if(p.path().extension()==".points")paths.push_back(p.path());
 if(paths.empty())throw std::runtime_error("no captured inputs");
 auto index=[](const auto&p){auto s=p.stem().string();return std::stoi(s.substr(s.rfind('_')+1));};std::sort(paths.begin(),paths.end(),[&](const auto&a,const auto&b){return index(a)<index(b);});
 auto now=[](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();};
 std::ofstream metrics(std::string(argv[3])+".jsonl");if(!metrics)throw std::runtime_error("cannot open replay metrics");int frame=0;
 for(auto&path:paths){std::ifstream f(path,std::ios::binary);uint32_t n=0;if(!f.read((char*)&n,4)||n>64000||std::filesystem::file_size(path)!=4+uint64_t(n)*sizeof(Point))throw std::runtime_error("capture size or point bound");std::vector<Point>p(n);f.read((char*)p.data(),n*sizeof(Point));if(!f)throw std::runtime_error("capture truncated");double start=now();if(baseline)old.integrate(p,frame+1);else fast.integrate(p,frame+1);double ms=now()-start;auto t=fast.timing();
 metrics<<"{\"frame\":"<<frame++<<",\"ms\":"<<ms<<",\"grouping_ms\":"<<t.grouping_ms<<",\"associate_ms\":"<<t.association_ms<<",\"commit_ms\":"<<t.commit_ms<<",\"points\":"<<n<<",\"count\":"<<(baseline?old.snapshot()->count:fast.snapshot()->count);auto stats=baseline?old.stats():fast.stats();metrics<<",\"capacity_drops\":"<<stats.capacity_drops<<",\"orientation_drops\":"<<stats.orientation_drops<<",\"overflow_surfels\":"<<t.overflow_surfels<<",\"max_cell_occupancy\":"<<t.max_cell_occupancy;metrics<<",\"order_ms\":"<<t.order_ms<<",\"matched_ms\":"<<t.matched_ms<<",\"unmatched_ms\":"<<t.unmatched_ms<<",\"cache_update_ms\":"<<t.cache_update_ms<<",\"backend_sync_ms\":"<<t.backend_sync_ms<<",\"unmatched_queries\":"<<t.unmatched_queries<<",\"query_cache_hits\":"<<t.query_cache_hits<<",\"query_cache_rebuilds\":"<<t.query_cache_rebuilds<<",\"query_cache_invalidations\":"<<t.query_cache_invalidations;if(hardware)metrics<<",\"pl_execute_ms\":"<<hardware->execute_ms<<",\"pl_pack_ms\":"<<hardware->packing_ms<<",\"pl_map_sync_ms\":"<<hardware->sync_ms<<",\"host_write_bytes\":"<<hardware->host_write_bytes<<",\"cache_misses\":"<<hardware->stats[1]<<",\"probes\":"<<hardware->stats[2];metrics<<"}\n";metrics.flush();}
 std::ofstream out(std::string(argv[3])+".bin",std::ios::binary);if(!out)throw std::runtime_error("cannot open replay map");if(baseline){for(auto&b:old.snapshot()->blocks)for(auto&s:b.second->lod[0])out.write((char*)&s,sizeof s);}else{auto snap=fast.snapshot();for(uint32_t id=0;id<snap->count;id++){auto legacy=legacy_surface(snap->at(id),id);out.write((char*)&legacy,sizeof(legacy));}}
 out.flush();if(!out||!metrics)throw std::runtime_error("replay evidence write failed");if(checked)std::cout<<"{\"checked_versions\":"<<checked->checked_versions<<",\"checked_queries\":"<<checked->checked_queries<<",\"mismatches\":0}\n";std::cout<<"replayed "<<paths.size()<<" identical captured inputs\n";
}
