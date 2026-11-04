#include "rgbd_pairing.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
struct Rate {uint64_t count=0;double last=0,max_gap=0;void add(double t){if(count)max_gap=std::max(max_gap,t-last);last=t;++count;}};
int main(int argc,char**argv){
 if(argc!=2)return 2;std::ifstream f(argv[1]);std::string line;lj2::RgbdPairer<int,int>queue;auto frame=std::make_shared<int>(1);uint64_t last_rgb=~uint64_t(0),last_depth=~uint64_t(0);bool have_rgb=false,have_depth=false;double rgb=0,depth=0;Rate old,newer;
 while(std::getline(f,line)){boost::property_tree::ptree r;std::istringstream input(line);boost::property_tree::read_json(input,r);double now=r.get<double>("received_mono");auto ri=r.get<uint64_t>("rgb_id"),di=r.get<uint64_t>("depth_id");
  if(ri!=last_rgb){last_rgb=ri;have_rgb=r.get<bool>("rgb_valid");if(have_rgb){rgb=r.get<double>("rgb_mapped");queue.put_rgb(frame,rgb,r.get<double>("rgb_sdk_receipt_mono",now),ri);}}
  if(di!=last_depth){last_depth=di;have_depth=r.get<bool>("depth_valid");if(have_depth){depth=r.get<double>("depth_mapped");queue.put_depth(frame,depth,r.get<double>("depth_sdk_receipt_mono",now),di);}}
  if(have_rgb&&have_depth){if(std::fabs(rgb-depth)<=.025&&now-depth<.5){old.add(now);have_rgb=have_depth=false;}else{if(rgb<depth)have_rgb=false;else have_depth=false;}}
  while(auto pair=queue.take(now)){if(std::fabs(pair->rgb_stamp-pair->depth_stamp)>.025)throw std::runtime_error("pair time bound");newer.add(now);}
 }
 std::cout<<"{\"scope\":\"same recorded arrival and validity trace; clock faults remain in replay\",\"old_pairs\":"<<old.count<<",\"old_max_gap_ms\":"<<old.max_gap*1000<<",\"bounded_queue_pairs\":"<<newer.count<<",\"bounded_queue_max_gap_ms\":"<<newer.max_gap*1000<<",\"queue_capacity_replaced\":"<<queue.capacity_replaced<<",\"queue_age_rejected\":"<<queue.age_rejected<<",\"queue_stamp_rejected\":"<<queue.stamp_rejected<<"}\n";
}
