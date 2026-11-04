#pragma once
#include "scene.hpp"
#include <fstream>
namespace lj2 {
// Explicit diagnostic export only: immutable map geometry, never raw camera data.
// Versioned board-native binary; reject incompatible sizes/counts on import.
inline void save_diagnostic(const Snapshot&s,const Camera&c,const Pose&v,const std::string&path){
 std::ofstream f(path,std::ios::binary);auto write=[&](const auto&x){f.write(reinterpret_cast<const char*>(&x),sizeof(x));};
 uint64_t magic=0x31474e4149444a4cULL,n=s.blocks.size();write(magic);write(c);write(v);write(s.version);write(s.source_stamp);write(s.count);write(n);
 for(const auto&b:s.blocks){write(b.first);for(const auto&level:b.second->lod){uint64_t count=level.size();write(count);for(const auto&surface:level)write(surface);}}
 if(!f)throw std::runtime_error("diagnostic write failed");
}
inline Snapshot load_diagnostic(const std::string&path,Camera&c,Pose&v){
 std::ifstream f(path,std::ios::binary);auto read=[&](auto&x){f.read(reinterpret_cast<char*>(&x),sizeof(x));if(!f)throw std::runtime_error("truncated diagnostic");};
 uint64_t magic,n;Snapshot s;read(magic);if(magic!=0x31474e4149444a4cULL)throw std::runtime_error("diagnostic version");read(c);read(v);read(s.version);read(s.source_stamp);read(s.count);read(n);
 if(n>1000000||s.count>1000000)throw std::runtime_error("diagnostic capacity");uint64_t total=0;
 for(uint64_t i=0;i<n;++i){Key key;read(key);auto b=std::make_shared<Block>();for(auto&level:b->lod){uint64_t count;read(count);total+=count;if(total>4000000)throw std::runtime_error("diagnostic LOD capacity");level.resize(count);for(auto&surface:level)read(surface);}s.blocks.emplace(key,b);}check_pose(v);return s;
}
}
