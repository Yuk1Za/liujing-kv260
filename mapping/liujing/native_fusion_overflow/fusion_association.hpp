#pragma once
#include "scene.hpp"
#include <vector>
#include <cstdint>
#include <cstring>
namespace lj2 {
// Only fusion state lives in immutable pages. Raster bounds stay out of the
// committed map and are reconstructed solely for the frozen replay file ABI.
// ID is the immutable page slot; radius is the unchanged .016 m constant.
// Neither needs duplication in every map copy and temporary query.
struct FusionSurfel {float x,y,z,nx,ny,nz;uint32_t rgb;float weight;};
static_assert(sizeof(FusionSurfel)==32);
inline Surface legacy_surface(const FusionSurfel&v,uint32_t id){Surface s{};s.x=v.x;s.y=v.y;s.z=v.z;s.nx=v.nx;s.ny=v.ny;s.nz=v.nz;s.radius=.016f;s.rgb=v.rgb;s.id=id;s.weight=v.weight;return s;}
// ABI uses explicit 32-bit words. No pointer or compiler-specific struct crosses AXI.
struct AssociationCell {uint32_t word[32]{};};
struct AssociationGroupWord {uint32_t word[8]{};};
struct AssociationQueryWord {uint32_t word[8]{};};
static_assert(sizeof(AssociationCell)==128 && sizeof(AssociationQueryWord)==32);
constexpr uint32_t association_tombstone=0xfffffffeu;
inline uint32_t association_hash(const Key&k){return uint32_t(k.x)*73856093u^uint32_t(k.y)*19349663u^uint32_t(k.z)*83492791u;}
struct AssociationQuery {FusionSurfel value;int32_t match=-1;uint32_t group=0;int32_t next=-1;};
struct AssociationGroup {Key key;uint32_t offset=0,count=0;};
struct AssociationUpdate {Key key;AssociationCell cell;};
class AssociationBackend {
public:
 virtual ~AssociationBackend()=default;
 // Called only by the ordered writer. associate reads version; sync ends before
 // that version is published and before the next hardware query can begin.
 virtual void associate(std::vector<AssociationQuery>&,const std::vector<AssociationGroup>&,uint64_t version)=0;
 virtual void sync(const std::vector<AssociationUpdate>&,uint64_t version)=0;
};
}
