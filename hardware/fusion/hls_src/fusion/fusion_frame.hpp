#pragma once
#include "matched_commit.hpp"
struct Observation { float x,y,z,nx,ny,nz; unsigned rgb,pad; };
static constexpr unsigned valid_unit_normal=0x4e524d31u;
static constexpr unsigned invalid_unit_normal=0x4e524d30u;
struct ResidentNode {
    SurfaceWord surface;
    int x,y,z;
    unsigned next,primary,dirty_job;
    uint64_t insertion;
};
struct FusionResult {
    unsigned job,count,dirty,added,merged,dropped;
    uint64_t visits;
};
#ifdef NATIONAL_NARROW
struct FrameBeat { uint32_t words[4]; };
using ObservationPort=FrameBeat;
using NodePort=FrameBeat;
using DeltaPort=FrameBeat;
using ResultPort=FrameBeat;
#else
using ObservationPort=Observation;
using NodePort=ResidentNode;
using DeltaPort=UpdateResult;
using ResultPort=FusionResult;
#endif
extern "C" void fusion_frame(const ObservationPort* observations, unsigned* buckets,
    NodePort* nodes, unsigned* touched, DeltaPort* delta, int* matches,
    unsigned size, unsigned initial_count, unsigned capacity, unsigned job, ResultPort* result
#ifdef NATIONAL_CYCLE_COUNTERS
    ,const volatile uint32_t& fabric_cycle
#endif
    );
