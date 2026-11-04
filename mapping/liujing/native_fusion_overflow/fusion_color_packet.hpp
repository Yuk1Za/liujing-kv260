#pragma once
#include "fusion_store.hpp"
#include <boost/property_tree/json_parser.hpp>
namespace lj2 {
// Encode only an immutable committed version. Transport scheduling is outside
// this function so raw replay and the live HTTP path use the same wire format.
inline std::string fusion_color_packet(const FusionDelta&d,const Pose&map_source,
 const std::string&session,double wall,double send_mono){
 const auto&snap=d.snapshot;boost::property_tree::ptree h;
 h.put("session",session);h.put("version",snap->version);h.put("base_version",d.base);h.put("full",d.full);h.put("frame","camera_init");h.put("source","committed_fusion");h.put("count",d.ids.size());h.put("map_count",snap->count);h.put("stride",20);h.put("deleted_count",0);h.put("source_stamp",snap->source_stamp);h.put("rgb_id",snap->rgb_id);h.put("depth_id",snap->depth_id);h.put("committed_mono",snap->committed_mono);h.put("send_mono",send_mono);h.put("source_age_s",wall-snap->source_stamp);
 std::ostringstream stream;boost::property_tree::write_json(stream,h,false);auto head=stream.str();uint32_t size=head.size();std::string packet="LJCD";packet.append(reinterpret_cast<const char*>(&size),4);packet+=head;packet.reserve(packet.size()+d.ids.size()*20);auto t=inverse(map_source);
 for(auto id:d.ids){const auto&p=snap->at(id);float xyz[]={(t[0]*p.x+t[1]*p.y)+t[2]*p.z+t[3],(t[4]*p.x+t[5]*p.y)+t[6]*p.z+t[7],(t[8]*p.x+t[9]*p.y)+t[10]*p.z+t[11]};packet.append(reinterpret_cast<const char*>(&id),4);packet.append(reinterpret_cast<const char*>(xyz),12);packet.append(reinterpret_cast<const char*>(&p.rgb),4);}return packet;
}
}
