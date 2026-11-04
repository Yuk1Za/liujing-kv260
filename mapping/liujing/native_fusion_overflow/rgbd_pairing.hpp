#pragma once
#include <deque>
#include <memory>
#include <optional>
#include <cstdint>
#include <cmath>
namespace lj2 {
template<class R,class D>class RgbdPairer {
 template<class T>struct Frame {std::shared_ptr<T>value;double stamp,received;uint64_t id;};
 std::deque<Frame<R>>rgb_;std::deque<Frame<D>>depth_;
 double last_rgb_=-1,last_depth_=-1;
 template<class T>void put(std::deque<Frame<T>>&q,double&last,std::shared_ptr<T>value,double stamp,double received,uint64_t id){
  if(!std::isfinite(stamp)||!std::isfinite(received)||stamp<=last){++stamp_rejected;return;}last=stamp;
  q.push_back({std::move(value),stamp,received,id});if(q.size()>8){q.pop_front();++capacity_replaced;}
 }
public:
 struct Pair {std::shared_ptr<R>rgb;std::shared_ptr<D>depth;double rgb_stamp,depth_stamp,depth_received;uint64_t rgb_id,depth_id;};
 uint64_t time_rejected=0,age_rejected=0,stamp_rejected=0,capacity_replaced=0;
 void put_rgb(std::shared_ptr<R>p,double stamp,double received,uint64_t id){put(rgb_,last_rgb_,std::move(p),stamp,received,id);}
 void put_depth(std::shared_ptr<D>p,double stamp,double received,uint64_t id){put(depth_,last_depth_,std::move(p),stamp,received,id);}
 std::optional<Pair>take(double now){
  while(!rgb_.empty()&&now-rgb_.front().received>.3){rgb_.pop_front();++age_rejected;}
  while(!depth_.empty()&&now-depth_.front().received>.3){depth_.pop_front();++age_rejected;}
  while(!rgb_.empty()&&!depth_.empty()){
   const auto&r=rgb_.front();const auto&d=depth_.front();double delta=r.stamp-d.stamp;
   if(std::fabs(delta)<=.025){Pair p{r.value,d.value,r.stamp,d.stamp,d.received,r.id,d.id};rgb_.pop_front();depth_.pop_front();return p;}
   if(delta<0)rgb_.pop_front();else depth_.pop_front();++time_rejected;
  }return {};
 }
 void clear(){rgb_.clear();depth_.clear();last_rgb_=last_depth_=-1;}
};
}
