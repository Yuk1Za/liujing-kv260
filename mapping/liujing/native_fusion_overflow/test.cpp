#include "scene.hpp"
#include "clock.hpp"
#include "live_policy.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <thread>
#include <future>
using namespace lj2;
int main(){
 Scene s(100);Point p{.01f,.01f,1,0,0,1,0x332211};s.integrate({p},1);auto before=s.snapshot();
 for(int i=0;i<100;i++)s.integrate({p},i+2);
 assert(s.snapshot()->count==1&&before->version==1&&s.snapshot()->version==101);
 assert(before->blocks.begin()->second->lod[0][0].weight==1);
 Point wall=p;wall.nx=1;wall.nz=0;s.integrate({wall},102);assert(s.snapshot()->count==2);
 assert(s.stats().merged==100);
 Scene colors;Point red{.01f,.01f,1,0,0,1,0x0000ff},blue=red;blue.rgb=0xff0000;colors.integrate({red},1);colors.integrate({blue},2);assert(colors.snapshot()->blocks.begin()->second->lod[0][0].rgb==0x800080);
 Scene boundary;auto a=red;a.x=.999f;boundary.integrate({a},1);auto retained=boundary.snapshot();a.x=1.01f;boundary.integrate({a},2);assert(boundary.snapshot()->count==1&&boundary.snapshot()->blocks.size()==1&&boundary.snapshot()->blocks.begin()->first.x==1&&retained->blocks.begin()->first.x==0);
 Scene cap(1);cap.integrate({p},1);p.x=50;cap.integrate({p},2);assert(cap.snapshot()->version==1&&cap.snapshot()->count==1&&cap.snapshot()->blocks.size()==1&&cap.stats().capacity_drops==1);
 // Coarse LOD must retain distinct normal groups rather than silently clip at four.
 Scene corners;std::vector<Point>faces;const float axes[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
 for(int i=0;i<6;i++)faces.push_back({.01f+i*.02f,.01f,1.01f,axes[i][0],axes[i][1],axes[i][2],0xffffff});corners.integrate(faces,1);assert(corners.snapshot()->blocks.begin()->second->lod[3].size()==6);
 Camera c;c.width=c.height=32;c.fx=c.fy=32;c.cx=c.cy=15.5;
 Scene empty;auto blank=render_cpu(prepare(*empty.snapshot(),c,identity()));for(int id:blank.ids)assert(id==-1);
 auto prepared=prepare(*s.snapshot(),c,identity());assert(prepared.budget_ok);auto im=render_cpu(prepared);auto color_only=render_cpu(prepared,false);assert(color_only.rgb==im.rgb&&color_only.depth.empty()&&color_only.ids.empty());bool visible=false;for(int id:im.ids)visible|=id>=0;assert(visible);
 auto rejected=prepare(*s.snapshot(),c,identity(),Budget{0,0,0});assert(!rejected.budget_ok&&rejected.surfaces.empty());
 // Fewer coarse disks can cost more pixels; do not accept that substitution
 // when it worsens the maximum normalized budget pressure.
 Snapshot difficult;auto block=std::make_shared<Block>();Surface disk{};disk.x=disk.y=.03125f;disk.z=2;disk.nz=1;disk.radius=.016f;disk.id=1;disk.rgb=0xabcdef;
 auto other=disk;other.x=.09375f;other.id=2;block->lod[0]={disk,other};for(int level=1;level<4;level++){disk.radius=.2f*level;block->lod[level]={disk};}
 difficult.blocks[{0,0,2}]=block;difficult.count=2;auto rejected_coarse=prepare(difficult,c,identity(),Budget{1,100,20});
 assert(!rejected_coarse.budget_ok&&rejected_coarse.after.surfaces==2&&rejected_coarse.after.pixels<=20&&rejected_coarse.lod_changes>0);
 auto reduced=prepare(difficult,c,identity(),Budget{2,100,20});assert(reduced.budget_ok&&reduced.after.pixels<=20);
 auto again=prepare(difficult,c,identity(),Budget{2,100,20});assert(again.indices==reduced.indices&&again.candidate_pixels==reduced.candidate_pixels&&again.surfaces.size()==reduced.surfaces.size());
 auto impossible=prepare(difficult,c,identity(),Budget{0,0,0});assert(!impossible.budget_ok&&impossible.exceeded==7&&impossible.after.surfaces==2);
 // A subpixel disk between integer centers has no covered samples.
 Snapshot between;auto tiny=std::make_shared<Block>();disk.x=disk.y=0;disk.radius=.016f;for(auto&level:tiny->lod)level={disk};between.blocks[{0,0,2}]=tiny;between.count=1;auto no_centers=prepare(between,c,identity(),Budget{0,0,0});assert(no_centers.budget_ok&&no_centers.surfaces.empty());
 Camera full;auto half=resized(full,320,240);assert(half.fx==full.fx*.5f&&half.cx==(full.cx+.5f)*.5f-.5f);
 ResolutionPolicy policy;assert(policy.current()==1);policy.accept(2,1);policy.observe(2,true);assert(policy.attempt(6)==2&&policy.attempt(7)==1);policy.failed_probe(7);assert(policy.attempt(20)==2);policy.observe(20,true);assert(policy.attempt(25)==1);policy.accept(1,25);assert(policy.current()==1);
 assert(upshift_headroom({30000,75000,2500000},55,40));assert(!upshift_headroom({30000,75000,2500000},85,40));assert(!upshift_headroom({30000,75000,3100000},55,40));assert(!upshift_headroom({30000,75000,2500000},55,65));
 PendingFrames<int> pending;for(int i=0;i<30;i++)pending.put(std::make_shared<int>(i),i*.01);assert(pending.size()==24&&pending.dropped()==6);
 assert(!pending.take(.3,[](int){return false;})&&pending.size()==24);auto selected=pending.take(.3,[](int value){return value<=20;});assert(selected&&*selected==20&&pending.size()==9);
 assert(!pending.take(2,[](int){return true;})&&pending.size()==0);pending.close();assert(!pending.put(std::make_shared<int>(1),2));
 assert(render_pose_fresh(2,1.6,.5)&&!render_pose_fresh(2,1.4,.5)&&!render_pose_fresh(2,2.1,.5));
 assert(input_fresh(2,1.9,1.8,true)&&!input_fresh(2,1,1.8,true)&&!input_fresh(2,1.9,1.8,false));
 PendingFrames<int> resetq;resetq.put(std::make_shared<int>(1),1);resetq.discard();assert(resetq.size()==0&&resetq.dropped()==1);assert(resetq.put(std::make_shared<int>(2),2));
 // A wider normal baseline must not resample measured depth positions or bridge steps.
 Camera nc{40,30,239,239,19.5f,14.5f,.3f,5};std::vector<uint16_t> noisy(1200),step(1200);std::vector<uint8_t> plain(3600,128);
 for(int y=0;y<30;++y)for(int x=0;x<40;++x){noisy[y*40+x]=1000+((x*17+y*13)%9)-4;step[y*40+x]=x<20?1000:1500;}
 auto normal_points=unproject(noisy.data(),plain.data(),nc,identity());assert(normal_points.size()==39*29);double error=0;size_t ni=0;
 for(int y=0;y<29;++y)for(int x=0;x<39;++x){const auto&p=normal_points[ni++];float z=noisy[y*40+x]*.001f;assert(p.z==z&&p.x==(x-nc.cx)*z/nc.fx&&p.y==(y-nc.cy)*z/nc.fy);if(x>=2&&x<38&&y>=2&&y<28)error+=1-p.nz;}
 assert(error/(36*26)<.08);auto step_points=unproject(step.data(),plain.data(),nc,identity());for(const auto&p:step_points)assert(std::fabs(p.nx)<1e-6&&std::fabs(p.ny)<1e-6&&p.nz>.999f);
 // Exact depth tie always selects smaller stable ID, regardless of descriptor order.
 Prepared tie=prepared;tie.surfaces={prepared.surfaces[0],prepared.surfaces[0]};tie.surfaces[0].id=7;tie.surfaces[0].rgb=0xaaaaaa;tie.surfaces[1].id=3;tie.surfaces[1].rgb=0x123456;tie.offsets={0,2,4,6,8};tie.indices={0,1,0,1,0,1,0,1};auto tied=render_cpu(tie);for(size_t i=0;i<tied.ids.size();i++)if(tied.ids[i]>=0)assert(tied.ids[i]==3&&tied.rgb[i]==0x123456);
 Poses poses;auto raw=identity(),mapped=identity();raw[3]=10;mapped[3]=1;assert(poses.add(false,1,raw));assert(poses.add(true,1,mapped));raw[3]=10.1f;assert(poses.add(false,1.1,raw));Pose corrected;double anchor;assert(poses.corrected(1.05,corrected,anchor));assert(std::fabs(corrected[3]-1.05f)<1e-5&&anchor==1);assert(!poses.corrected(1.2,corrected,anchor));assert(!poses.add(false,1.0,raw)&&poses.failed());
 Poses anchor_gap;auto origin=identity();for(double t:{1.,1.2,1.6,1.8}){auto v=origin;v[3]=float(t*.1);assert(anchor_gap.add(false,t,v));}auto older=origin;older[3]=.1f;assert(anchor_gap.add(true,1,older));auto unsupported=origin;unsupported[3]=.14f;assert(anchor_gap.add(true,1.4,unsupported));assert(anchor_gap.corrected(1.7,corrected,anchor)&&anchor==1&&std::fabs(corrected[3]-.17f)<1e-5);assert(!anchor_gap.corrected(1.4,corrected,anchor));
 Latest<int> q;q.put(std::make_shared<int>(1));auto held=q.take();q.put(std::make_shared<int>(2));q.put(std::make_shared<int>(3));assert(*held==1&&*q.take()==3&&q.dropped()==1);q.close();assert(!q.put(std::make_shared<int>(4)));
 Latest<int> demand;std::thread producer([&]{for(int i=0;i<500;i++){assert(demand.wait_for_space());assert(demand.put(std::make_shared<int>(i)));}});
 for(int i=0;i<500;i++){auto item=demand.wait();assert(item&&*item==i);}producer.join();assert(demand.dropped()==0);
 demand.put(std::make_shared<int>(501));std::promise<void>waiting;auto space=std::async(std::launch::async,[&]{waiting.set_value();return demand.wait_for_space();});waiting.get_future().wait();assert(space.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout);demand.close();assert(!space.get());
 // Readers retain immutable blocks while the writer commits replacements.
 Scene concurrent;concurrent.integrate({wall},1);std::thread writer([&]{for(int i=2;i<100;i++)concurrent.integrate({wall},i);});for(int i=0;i<1000;i++){auto snap=concurrent.snapshot();assert(snap->count==1&&snap->blocks.begin()->second->lod[0].size()==1);}writer.join();
 bool regression=false;try{s.integrate({},1);}catch(const std::invalid_argument&){regression=true;}assert(regression);
 ClockMapper clock;double mapped_time=0;bool ready=false;for(int i=0;i<120;i++)ready=clock.observe(5e12+i*5e8,10+i/30.,mapped_time);assert(ready&&std::fabs(mapped_time-(10+119/30.))<1e-6);assert(!clock.observe(1,20,mapped_time)&&clock.fault());
 LegacyRgbTicks ticks;uint64_t normalized=0;for(int i=0;i<4;i++)assert(ticks.observe(4294493455000ULL-(3-i)*498218000ULL,i,normalized));assert(ticks.observe(24375000ULL,4,normalized));assert(normalized==4294991671000ULL&&ticks.wraps()==1);assert(!ticks.observe(1,5,normalized)&&ticks.fault());
 ClockMapper duplicate;for(int i=0;i<100;++i)duplicate.observe(i*1000,10+i/30.,mapped_time);assert(!duplicate.observe(99000,14,mapped_time)&&!duplicate.fault());
 ClockMapper delayed;for(int i=0;i<120;i++)assert(i<59? !delayed.observe(i*1000,10+i/30.,mapped_time):delayed.observe(i*1000,10+i/30.,mapped_time));
 for(int i=120;i<124;i++)assert(!delayed.observe(i*1000,14.12+(i-120)*.001,mapped_time));
 assert(delayed.observe(124000,10+124/30.,mapped_time));assert(delayed.rejected_arrivals()==4);assert(std::fabs(mapped_time-(10+124/30.))<.001);
 ClockMapper shifted;for(int i=0;i<120;i++)shifted.observe(i*1000,10+i/30.,mapped_time);
 for(int i=120;i<135;i++)assert(!shifted.observe(i*1000,10+i/30.+.1,mapped_time));
 assert(shifted.reacquisitions()==1);for(int i=135;i<220;i++)ready=shifted.observe(i*1000,10+i/30.+.1,mapped_time);assert(ready&&!shifted.fault());
 std::cout<<"PASS bounded map, immutable snapshots, orientation separation, deterministic raster, budgets, corrected pose, latest ownership\n";
}
