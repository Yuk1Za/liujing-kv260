#pragma once
#include "scene.hpp"
#include <deque>
#include <functional>

namespace lj2 {
inline bool input_fresh(double now,double rgb,double depth,bool clock_ready){return clock_ready&&rgb>0&&depth>0&&now-rgb<=.5&&now-depth<=.5&&now>=rgb&&now>=depth;}
inline bool render_pose_fresh(double now,double stamp,double maximum_age){return stamp>0&&now>=stamp&&now-stamp<=maximum_age;}
// Independent pipeline stages have 100 ms at the 10 Hz display target.
// Probe only with stage headroom; every larger task still faces hard budgets.
inline bool upshift_headroom(const WorkCounts&cost,double prepare_ms,double render_ms){
 return cost.surfaces<38000&&cost.references<85000&&cost.pixels<3000000&&prepare_ms<80&&render_ms<60;
}
inline Camera resized(Camera c,int width,int height){
 const float sx=float(width)/c.width,sy=float(height)/c.height;
 c.fx*=sx;c.fy*=sy;c.cx=(c.cx+.5f)*sx-.5f;c.cy=(c.cy+.5f)*sy-.5f;
 c.width=width;c.height=height;return c;
}

// Single preparation-thread owner. Rendering feedback is supplied by the caller.
class ResolutionPolicy {
 int preferred_,current_;double changed_=0,headroom_since_=-1;
public:
 static constexpr int widths[4]={640,320,256,160};
 static constexpr int heights[4]={480,240,192,120};
 explicit ResolutionPolicy(int preferred=1):preferred_(preferred),current_(preferred){if(preferred<0||preferred>3)throw std::invalid_argument("render size");}
 int current()const{return current_;}
 void observe(double now,bool headroom){if(!headroom)headroom_since_=-1;else if(headroom_since_<0)headroom_since_=now;}
 int attempt(double now)const{return current_>preferred_&&now-changed_>=5&&headroom_since_>=0&&now-headroom_since_>=5?current_-1:current_;}
 void accept(int level,double now){if(level!=current_){current_=level;changed_=now;headroom_since_=-1;}}
 void failed_probe(double now){headroom_since_=-1;changed_=now;}
};

// SDK frame references stay in RAM, with both a time and a count bound.
template<class T>class PendingFrames {
 struct Entry{std::shared_ptr<T> value;double received;};
 std::deque<Entry> queue_;mutable std::mutex mutex_;bool closed_=false;
 uint64_t expired_=0,evicted_=0,selected_out_=0;
 void prune(double now){while(!queue_.empty()&&now-queue_.front().received>.8){queue_.pop_front();++expired_;}}
public:
 bool put(std::shared_ptr<T> value,double received){std::lock_guard<std::mutex>l(mutex_);if(closed_)return false;prune(received);queue_.push_back({std::move(value),received});while(queue_.size()>24){queue_.pop_front();++evicted_;}return true;}
 template<class Predicate>std::shared_ptr<T> take(double now,Predicate eligible){
  std::lock_guard<std::mutex>l(mutex_);prune(now);
  for(size_t i=queue_.size();i>0;--i)if(eligible(*queue_[i-1].value)){
   auto value=queue_[i-1].value;selected_out_+=i-1;queue_.erase(queue_.begin(),queue_.begin()+i);return value;
  }return {};
 }
 template<class Predicate>std::shared_ptr<T> take_first(double now,Predicate eligible){
  std::lock_guard<std::mutex>l(mutex_);prune(now);
  for(size_t i=0;i<queue_.size();i++)if(eligible(*queue_[i].value)){auto value=queue_[i].value;selected_out_+=i;queue_.erase(queue_.begin(),queue_.begin()+i+1);return value;}return {};
 }
 size_t size()const{std::lock_guard<std::mutex>l(mutex_);return queue_.size();}
 uint64_t dropped()const{std::lock_guard<std::mutex>l(mutex_);return expired_+evicted_;}
 uint64_t selected_out()const{std::lock_guard<std::mutex>l(mutex_);return selected_out_;}
 void discard(){std::lock_guard<std::mutex>l(mutex_);evicted_+=queue_.size();queue_.clear();}
 void close(){std::lock_guard<std::mutex>l(mutex_);closed_=true;queue_.clear();}
};
}
