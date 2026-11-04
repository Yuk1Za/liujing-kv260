// Synthetic RAM-only scheduling experiment. No sensors, ROS poses or mobile acceptance.
#include "pl.hpp"
#include "encode.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <cmath>
#include <exception>
using namespace lj2;
static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Task {Prepared p;uint64_t request;double requested,prepare_ms;};
struct Frame {std::shared_ptr<Task> task;Image image;double render_ms;uint32_t cycles;};
int main(int argc,char**argv){
 if(argc!=3)return 2;
 try{
  const double seconds=std::stod(argv[2]);if(seconds<10||seconds>120)throw std::runtime_error("duration must be 10..120 seconds");
  const bool cpu=std::string(argv[1])=="cpu";std::unique_ptr<Pl>pl;if(!cpu)pl=std::make_unique<Pl>(argv[1]);
  cv::setNumThreads(1);Scene scene;Camera camera,depth{320,200,238.8f,238.8f,160.5f,99.8f,.3f,5};
  std::vector<uint16_t>d(320*200,2000);std::vector<uint8_t>rgb(320*200*3);
  for(size_t i=0;i<rgb.size();i++)rgb[i]=uint8_t(i*37+(i/960)*17);
  const auto points=unproject(d.data(),rgb.data(),depth,identity());
  std::deque<std::array<float,3>>path;for(int i=0;i<6000;i++)path.push_back({float(std::sin(i*.002)*.5),float(std::cos(i*.002)*.3),1.f});
  Latest<Task>ready;Latest<Frame>finished;std::atomic<bool>stop{false};std::mutex error_mutex;std::exception_ptr error;
  auto fail=[&](){std::lock_guard<std::mutex>lock(error_mutex);if(!error)error=std::current_exception();stop=true;ready.close();finished.close();};
  const double start=now(),end=start+seconds;std::vector<std::pair<double,double>>updates;uint64_t rejected=0,frames=0,unique=0,last_hash=0;
  std::cout<<std::setprecision(12)<<"{\"scope\":\"synthetic_RAM_pipeline_not_mobile_acceptance\",\"backend\":\""<<(cpu?"cpu":"pl")<<"\",\"seconds\":"<<seconds<<",\"resolution\":[640,480],\"jpeg_quality\":85}"<<std::endl;
  std::thread mapper([&](){try{
   for(double due=start;!stop&&due<end;due+=.5){std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(due))));if(stop)break;double a=now();scene.integrate(points,a);updates.emplace_back(now()-start,(now()-a)*1000);}
  }catch(...){fail();}});
  std::thread prepare_worker([&](){try{
   uint64_t request=0;
   for(double due=start;!stop&&due<end;due+=.05){std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(due))));if(stop)break;
    double a=now();auto snapshot=scene.snapshot();if(!snapshot->version)continue;auto t=identity();t[3]=float(.1*std::sin((a-start)*1.3));t[7]=float(.05*std::cos((a-start)*.7));
    auto job=std::make_shared<Task>();job->requested=a;job->request=++request;job->p=prepare(*snapshot,camera,t);job->prepare_ms=(now()-a)*1000;if(job->p.budget_ok)ready.put(job);else ++rejected;
   }
   ready.close();
  }catch(...){fail();}});
  std::thread encoder([&](){try{while(auto f=finished.wait()){
   auto encoded=encode_image(f->image,camera,f->task->p.view,path,false);double done=now();uint64_t hash=14695981039346656037ull;for(auto b:encoded.bytes){hash^=b;hash*=1099511628211ull;}
   ++frames;bool changed=frames==1||hash!=last_hash;if(changed)++unique;last_hash=hash;
   std::cout<<"{\"frame\":"<<frames<<",\"request\":"<<f->task->request<<",\"completed_s\":"<<done-start<<",\"new_pixels\":"<<(changed?"true":"false")<<",\"map_version\":"<<f->task->p.map_version<<",\"map_age_ms\":"<<(done-f->task->p.map_stamp)*1000<<",\"latency_ms\":"<<(done-f->task->requested)*1000<<",\"prepare_ms\":"<<f->task->prepare_ms<<",\"render_ms\":"<<f->render_ms<<",\"pl_cycles\":"<<f->cycles<<",\"encode_ms\":"<<encoded.convert_ms+encoded.overlay_ms+encoded.jpeg_ms<<",\"visible\":"<<f->task->p.surfaces.size()<<",\"candidates\":"<<f->task->p.candidate_pixels<<",\"jpeg_bytes\":"<<encoded.bytes.size()<<"}"<<std::endl;
  }}catch(...){fail();}});
  try{while(auto job=ready.wait()){
   auto f=std::make_shared<Frame>();f->task=job;double a=now();f->image=cpu?render_cpu(job->p,false):pl->render(job->p,false);f->render_ms=(now()-a)*1000;f->cycles=cpu?0:pl->metrics[1];finished.put(f);
  }}catch(...){fail();}
  finished.close();prepare_worker.join();mapper.join();encoder.join();if(error)std::rethrow_exception(error);
  for(auto u:updates)std::cout<<"{\"map_commit_s\":"<<u.first<<",\"fusion_ms\":"<<u.second<<"}"<<std::endl;
  std::cout<<"{\"summary\":true,\"frames\":"<<frames<<",\"changed_images\":"<<unique<<",\"map_updates\":"<<updates.size()<<",\"map_surfels\":"<<scene.snapshot()->count<<",\"prepare_replaced\":"<<ready.dropped()<<",\"encode_replaced\":"<<finished.dropped()<<",\"budget_rejected\":"<<rejected<<",\"pl_completions\":"<<(cpu?0:pl->completed)<<"}"<<std::endl;
  return frames&&updates.size()>1?0:1;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
