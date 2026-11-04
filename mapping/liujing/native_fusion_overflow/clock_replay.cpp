#include "clock.hpp"
#define ClockMapper BaselineClockMapper
#include "../native_v2/clock.hpp"
#undef ClockMapper
#include <boost/property_tree/json_parser.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
struct Stream {
 lj2::ClockMapper current;lj2::BaselineClockMapper baseline;
 uint64_t last_id=~uint64_t(0),accepted=0,rejected=0;double last=0,max_gap=0;
};
int main(int argc,char**argv){
 if(argc!=2)return 2;std::ifstream f(argv[1]);std::map<std::string,Stream>streams;std::string line;
 while(std::getline(f,line)){boost::property_tree::ptree row;std::istringstream input(line);boost::property_tree::read_json(input,row);
  for(bool sdk:{false,true})for(bool legacy:{false,true}){
   std::string key=row.get<std::string>("sensor")+(sdk?"_sdk":"_callback")+(legacy?"_old":"_new");auto&s=streams[key];auto id=row.get<uint64_t>("id");if(id==s.last_id)continue;s.last_id=id;
   double received=row.get<double>("received"),host=sdk?received+row.get<double>("system_ms")*.001-row.get<double>("wall"):received,mapped=0;
   bool valid=legacy?s.baseline.observe(row.get<double>("device"),host,mapped):s.current.observe(row.get<double>("device"),host,mapped);
   if(valid){++s.accepted;if(s.last)s.max_gap=std::max(s.max_gap,received-s.last);s.last=received;}else ++s.rejected;
  }
 }
 for(auto&entry:streams){auto&s=entry.second;std::cout<<"{\"path\":\""<<entry.first<<"\",\"accepted\":"<<s.accepted<<",\"rejected\":"<<s.rejected<<",\"max_accepted_arrival_gap_ms\":"<<s.max_gap*1000<<"}\n";}
}
