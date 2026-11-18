#pragma once
#include <cstdint>
struct SurfaceWord {
    float x,y,z,nx,ny,nz;
    uint32_t rgb;
    float weight;
};
struct MatchUpdate {
    uint32_t id;
    SurfaceWord observation;
};
struct UpdateResult { uint32_t id; SurfaceWord value; uint32_t pad[7]; };
extern "C" void matched_commit(const MatchUpdate* ordered, SurfaceWord* resident,
    UpdateResult* dirty, unsigned count, unsigned map_count, unsigned& changed, unsigned& error);
