#pragma once
#include "contract.hpp"
#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <stdexcept>

namespace lj2 {
using Pose=std::array<float,16>;
Pose identity(); Pose multiply(const Pose&,const Pose&); Pose inverse(const Pose&);
void check_pose(const Pose&);
struct Camera { int width=640,height=480;float fx=477.6f,fy=477.6f,cx=319.5f,cy=239.5f,near_z=.3f,far_z=8.f; };
struct Point {float x,y,z,nx,ny,nz;uint32_t rgb;};
struct Key {int x,y,z;bool operator<(const Key&b)const{return std::array<int,3>{x,y,z}<std::array<int,3>{b.x,b.y,b.z};}bool operator==(const Key&b)const{return x==b.x&&y==b.y&&z==b.z;}};
struct Block {std::map<Key,std::vector<Surface>> cells;std::array<std::vector<Surface>,4> lod;};
struct Snapshot {uint64_t version=0;double source_stamp=0;size_t count=0;std::map<Key,std::shared_ptr<const Block>> blocks;};
struct MapStats {uint64_t added=0,merged=0,capacity_drops=0,orientation_drops=0,cloned_blocks=0;double index_ms=0,integrate_ms=0,lod_ms=0;};
class Scene {
 std::shared_ptr<const Snapshot> current_;std::mutex writer_;uint32_t next_id_=0;size_t capacity_;MapStats stats_;
public:
 explicit Scene(size_t capacity=1000000);
 std::shared_ptr<const Snapshot> snapshot()const{return std::atomic_load(&current_);}
 MapStats integrate(const std::vector<Point>&,double stamp);
 MapStats stats(){std::lock_guard<std::mutex>l(writer_);return stats_;}
};
struct Budget {uint32_t surfaces=40000,references=100000;uint64_t pixels=4000000;};
struct WorkCounts {uint64_t surfaces=0,references=0,pixels=0;};
struct Prepared {uint64_t map_version=0;double map_stamp=0;Pose view;Camera camera;std::vector<Surface> surfaces;std::vector<uint32_t> offsets,indices;uint64_t candidate_pixels=0;uint32_t coarsened_blocks=0;bool budget_ok=true;WorkCounts before,after;uint32_t exceeded=0,lod_changes=0;};
Prepared prepare(const Snapshot&,const Camera&,const Pose&,Budget={});
struct Image {int width=0,height=0;std::vector<uint32_t>rgb;std::vector<float>depth;std::vector<int32_t>ids;};
Image render_cpu(const Prepared&,bool debug=true);
std::vector<Point> unproject(const uint16_t*depth,const uint8_t*rgb,const Camera&,const Pose&,float scale=.001f);
std::vector<Point> unproject_cached(const uint16_t*depth,const uint8_t*rgb,const Camera&,const Pose&,float scale=.001f);
void save_map(const Snapshot&,const char*path);

// Single latest value. Moving a shared_ptr transfers ownership; a producer can
// replace only the waiting slot, never the object currently held by a consumer.
template<class T> class Latest {
 std::mutex mutex_;std::condition_variable cv_;std::shared_ptr<T> slot_;bool closed_=false;uint64_t dropped_=0;
public:
 bool put(std::shared_ptr<T> v){std::lock_guard<std::mutex>l(mutex_);if(closed_)return false;if(slot_)++dropped_;slot_=std::move(v);cv_.notify_all();return true;}
 std::shared_ptr<T> take(){std::lock_guard<std::mutex>l(mutex_);auto v=std::move(slot_);cv_.notify_all();return v;}
 std::shared_ptr<T> wait(){std::unique_lock<std::mutex>l(mutex_);cv_.wait(l,[&]{return closed_||slot_;});auto v=std::move(slot_);cv_.notify_all();return v;}
 // Single producer only: postpone expensive preparation until the consumer
 // owns the previous value. The two stages still execute concurrently.
 bool wait_for_space(){std::unique_lock<std::mutex>l(mutex_);cv_.wait(l,[&]{return closed_||!slot_;});return !closed_;}
 void close(){std::lock_guard<std::mutex>l(mutex_);closed_=true;cv_.notify_all();}
 uint64_t dropped(){std::lock_guard<std::mutex>l(mutex_);return dropped_;}
};
struct TimedPose {double stamp;Pose pose;};
class Poses {
 std::vector<TimedPose>raw_,mapped_;std::mutex mutex_;bool failed_=false;
 static bool sample(const std::vector<TimedPose>&,double,Pose&);
public:
 bool add(bool mapped,double stamp,const Pose&);
 bool corrected(double stamp,Pose&,double&anchor_stamp);
 bool failed(){std::lock_guard<std::mutex>l(mutex_);return failed_;}
};
}
