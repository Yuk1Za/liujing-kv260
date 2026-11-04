#include "core.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <limits>
#include <stdexcept>
#include <cstring>
#if defined(__aarch64__) && !defined(LIUJING_SCALAR_REFERENCE)
#include <arm_neon.h>
#define LIUJING_NEON 1
#endif
#ifdef _OPENMP
#include <omp.h>
#endif
namespace {
struct Map { std::vector<Surfel>s;int capacity,stride;uint64_t version=0,drops=0;double last=-1,origin=-1;std::mutex mu; };
bool camera_ok(const Camera&c){return c.width>0&&c.height>0&&c.width<=640&&c.height<=480&&std::isfinite(c.fx)&&std::isfinite(c.fy)&&std::isfinite(c.cx)&&std::isfinite(c.cy)&&c.fx>0&&c.fy>0&&c.near_z>0&&c.far_z>c.near_z&&std::isfinite(c.far_z);}
bool transform_ok(const float*T){
 for(int i=0;i<16;i++)if(!std::isfinite(T[i]))return false;
 if(std::fabs(T[12])+std::fabs(T[13])+std::fabs(T[14])+std::fabs(T[15]-1)>1e-4)return false;
 for(int i=0;i<3;i++)for(int j=0;j<3;j++){float v=0;for(int k=0;k<3;k++)v+=T[k*4+i]*T[k*4+j];if(std::fabs(v-(i==j?1.f:0.f))>1e-3)return false;}
 float det=T[0]*(T[5]*T[10]-T[6]*T[9])-T[1]*(T[4]*T[10]-T[6]*T[8])+T[2]*(T[4]*T[9]-T[5]*T[8]);return std::fabs(det-1)<1e-3;
}
void forward(const float*T,float x,float y,float z,float&o0,float&o1,float&o2,bool point){o0=T[0]*x+T[1]*y+T[2]*z+(point?T[3]:0);o1=T[4]*x+T[5]*y+T[6]*z+(point?T[7]:0);o2=T[8]*x+T[9]*y+T[10]*z+(point?T[11]:0);}
Surfel camera_surfel(const Surfel&s,const float*T){Surfel q=s;float x=s.x-T[3],y=s.y-T[7],z=s.z-T[11];q.x=T[0]*x+T[4]*y+T[8]*z;q.y=T[1]*x+T[5]*y+T[9]*z;q.z=T[2]*x+T[6]*y+T[10]*z;q.nx=T[0]*s.nx+T[4]*s.ny+T[8]*s.nz;q.ny=T[1]*s.nx+T[5]*s.ny+T[9]*s.nz;q.nz=T[2]*s.nx+T[6]*s.ny+T[10]*s.nz;return q;}
bool bounds(const Surfel&s,const Camera&c,int&x0,int&x1,int&y0,int&y1){
 if(!(s.radius>0)||!std::isfinite(s.x)||!std::isfinite(s.y)||!std::isfinite(s.z)||!std::isfinite(s.radius)||s.z+s.radius<c.near_z||s.z-s.radius>c.far_z)return false;
 // Bounding sphere box projection is conservative, including near-plane intersections.
 float zn=std::max(c.near_z,s.z-s.radius),zf=std::max(zn,s.z+s.radius);
 float ux[4]={(s.x-s.radius)/zn,(s.x-s.radius)/zf,(s.x+s.radius)/zn,(s.x+s.radius)/zf};
 float vy[4]={(s.y-s.radius)/zn,(s.y-s.radius)/zf,(s.y+s.radius)/zn,(s.y+s.radius)/zf};
 float l=c.fx**std::min_element(ux,ux+4)+c.cx,r=c.fx**std::max_element(ux,ux+4)+c.cx;
 float t=c.fy**std::min_element(vy,vy+4)+c.cy,b=c.fy**std::max_element(vy,vy+4)+c.cy;
 if(r<0||b<0||l>c.width-1||t>c.height-1)return false;
 x0=(int)std::floor(std::max(0.f,l));x1=(int)std::ceil(std::min((float)c.width-1,r));y0=(int)std::floor(std::max(0.f,t));y1=(int)std::ceil(std::min((float)c.height-1,b));return true;
}
struct Prepared {std::vector<Surfel>s;std::vector<uint32_t>off,idx;};
uint32_t project_count(const std::vector<Surfel>&s,const Camera&c,const float*T,Surfel*out,uint32_t*off){
 int tx=(c.width+15)/16,nt=tx*((c.height+15)/16);std::fill(off,off+nt+1,0);
 // Independent transforms/bounds use the same configured CPU threads in both
 // backends. Keep counting/scattering ordered so lists and stable IDs do not
 // depend on worker scheduling. The arithmetic expression tree is unchanged.
#pragma omp parallel for schedule(static) if(s.size()>=1024)
 for(size_t i=0;i<s.size();i++){auto&v=out[i];v=camera_surfel(s[i],T);int a,b,d,e;if(!bounds(v,c,a,b,d,e)){v.submap=0;v.pad0=-1;continue;}v.submap=(float)a;v.pad0=(float)b;v.pad1=(float)d;v.pad2=(float)e;}
 for(size_t i=0;i<s.size();i++){const auto&v=out[i];int a=(int)v.submap,b=(int)v.pad0,d=(int)v.pad1,e=(int)v.pad2;if(b<a)continue;for(int y=d/16;y<=e/16;y++)for(int x=a/16;x<=b/16;x++)off[y*tx+x+1]++;}
 for(int i=1;i<=nt;i++)off[i]+=off[i-1];return off[nt];
}
void scatter(const Surfel*s,size_t n,const Camera&c,const uint32_t*off,uint32_t*idx){
 int tx=(c.width+15)/16,nt=tx*((c.height+15)/16);std::vector<uint32_t>cursor(off,off+nt+1);
 for(uint32_t i=0;i<n;i++){const auto&v=s[i];int a=(int)v.submap,b=(int)v.pad0,d=(int)v.pad1,e=(int)v.pad2;if(b<a)continue;for(int y=d/16;y<=e/16;y++)for(int x=a/16;x<=b/16;x++)idx[cursor[y*tx+x]++]=i;}
}
Prepared prepare(const std::vector<Surfel>&s,const Camera&c,const float*T){
 Prepared p;int nt=((c.width+15)/16)*((c.height+15)/16);p.s.resize(s.size());p.off.resize(nt+1);auto refs=project_count(s,c,T,p.s.data(),p.off.data());
 if(refs>4000000)throw std::runtime_error("tile index capacity exceeded");
 p.idx.resize(refs);scatter(p.s.data(),p.s.size(),c,p.off.data(),p.idx.data());
 return p;
}
// Independent CPU raster: clipped bounding rectangles, tile-owned output, no races.
template<int mode> void render_cpu_mode(const Prepared&p,const Camera&c,float*depth,int32_t*ids,uint32_t*rgb){
 constexpr bool mixed=mode==4||mode==5;
 int txs=(c.width+15)/16,tys=(c.height+15)/16;
 float rayx[640],rayy[480];for(int x=0;x<c.width;x++)rayx[x]=(x-c.cx)/c.fx;for(int y=0;y<c.height;y++)rayy[y]=(y-c.cy)/c.fy;
#pragma omp parallel for schedule(dynamic,4)
 for(int tile=0;tile<txs*tys;tile++){
  int tx=tile%txs,ty=tile/txs;float zbuf[256],sum[256]={},col[256][3]={};int owner[256];uint64_t iw[256]={},ic[256][3]={};
  std::fill(zbuf,zbuf+256,c.far_z+1);std::fill(owner,owner+256,-1);
  for(int pass=0;pass<(mode==3?1:2);pass++)for(uint32_t j=p.off[tile];j<p.off[tile+1];j++){
   uint32_t id=p.idx[j];const Surfel&s=p.s[id];int x0=(int)s.submap,x1=(int)s.pad0,y0=(int)s.pad1,y1=(int)s.pad2;
   x0=std::max(x0,tx*16);x1=std::min(x1,tx*16+15);y0=std::max(y0,ty*16);y1=std::min(y1,ty*16+15);
   const float numerator=(s.nx*s.x+s.ny*s.y)+s.nz*s.z,radius2=s.radius*s.radius,inverse_radius2=mode==5?1.f/radius2:0.f;
   for(int y=y0;y<=y1;y++)for(int block=x0;block<=x1;block+=4){
    float zs[4],qs[4];uint32_t valid[4];int lanes=std::min(4,x1-block+1);
#ifdef LIUJING_NEON
    if(lanes==4){
     float32x4_t rx=vld1q_f32(rayx+block),ry=vdupq_n_f32(rayy[y]);
     // Explicit vmul/vadd, never vfmaq or reciprocal approximations. The same
     // expression tree as scalar/HLS is evaluated in four independent pixels.
     auto den=vaddq_f32(vaddq_f32(vmulq_n_f32(rx,s.nx),vmulq_n_f32(ry,s.ny)),vdupq_n_f32(s.nz));
     auto z=vdivq_f32(vdupq_n_f32(numerator),den);
     auto dx=vsubq_f32(vmulq_f32(rx,z),vdupq_n_f32(s.x)),dy=vsubq_f32(vmulq_f32(ry,z),vdupq_n_f32(s.y)),dz=vsubq_f32(z,vdupq_n_f32(s.z));
     auto distance2=vaddq_f32(vaddq_f32(vmulq_f32(dx,dx),vmulq_f32(dy,dy)),vmulq_f32(dz,dz));
     auto q=mode==5?vmulq_n_f32(distance2,inverse_radius2):vdivq_f32(distance2,vdupq_n_f32(radius2));
     auto covered=mode==5?vcltq_f32(distance2,vdupq_n_f32(radius2)):vcltq_f32(q,vdupq_n_f32(1.f));
     auto keep=vandq_u32(vandq_u32(vcgeq_f32(vabsq_f32(den),vdupq_n_f32(1e-6f)),vcgeq_f32(z,vdupq_n_f32(c.near_z))),vandq_u32(vcleq_f32(z,vdupq_n_f32(c.far_z)),covered));
     vst1q_f32(zs,z);vst1q_f32(qs,q);vst1q_u32(valid,keep);
    }else
#endif
    for(int lane=0;lane<lanes;lane++){
     float rx=rayx[block+lane],ry=rayy[y],den=(s.nx*rx+s.ny*ry)+s.nz;
     float z=numerator/den,dx=rx*z-s.x,dy=ry*z-s.y,dz=z-s.z,distance2=(dx*dx+dy*dy)+dz*dz,q=mode==5?distance2*inverse_radius2:distance2/radius2;
     zs[lane]=z;qs[lane]=q;valid[lane]=std::fabs(den)>=1e-6f&&z>=c.near_z&&z<=c.far_z&&(mode==5?distance2<radius2:q<1);
    }
    for(int lane=0;lane<lanes;lane++){
    if(!valid[lane])continue;int x=block+lane;float z=zs[lane],q=qs[lane];
    int k=(y-ty*16)*16+x-tx*16;
    if(pass==0){if(z<zbuf[k]||(z==zbuf[k]&&(owner[k]<0||id<(uint32_t)owner[k]))){zbuf[k]=z;owner[k]=id;}}
    else if(std::fabs(z-zbuf[k])<=.01f){float w=(1-q)*std::min(32.f,std::max(1.f,s.weight));if(!mixed)sum[k]+=w;float r=s.r,g=s.g,b=s.b;
     if(mode==1){r=127.5f*(s.nx+1);g=127.5f*(s.ny+1);b=127.5f*(s.nz+1);}else if(mode==2){float conf=std::min(1.f,s.weight/8);r=255*(1-conf);g=255*conf;b=32;}
     if(mixed){uint32_t wq=(uint32_t)(w*262144.f+.5f);iw[k]+=wq;ic[k][0]+=(uint64_t)wq*(uint32_t)(r*4096.f+.5f);ic[k][1]+=(uint64_t)wq*(uint32_t)(g*4096.f+.5f);ic[k][2]+=(uint64_t)wq*(uint32_t)(b*4096.f+.5f);}
     else {col[k][0]+=w*r;col[k][1]+=w*g;col[k][2]+=w*b;}
    }
    }
   }
  }
  for(int y=0;y<16;y++)for(int x=0;x<16;x++){int xx=tx*16+x,yy=ty*16+y;if(xx>=c.width||yy>=c.height)continue;int k=y*16+x,o=yy*c.width+xx;ids[o]=owner[k];depth[o]=owner[k]>=0?zbuf[k]:0;rgb[o]=0;if(mixed){if(iw[k]){float den=float(iw[k])*4096.f,inverse=mode==5?1.f/den:0.f;for(int ch=0;ch<3;ch++){float color=mode==5?float(ic[k][ch])*inverse:float(ic[k][ch])/den;rgb[o]|=(uint32_t)std::min(255.f,color+.5f)<<(8*ch);}}}else if(sum[k]>0)for(int ch=0;ch<3;ch++)rgb[o]|=(uint32_t)std::clamp(col[k][ch]/sum[k]+.5f,0.f,255.f)<<(8*ch);}
 }
}
void render_cpu(const Prepared&p,const Camera&c,float*d,int32_t*i,uint32_t*r,int mode){
 switch(mode){
  case 0:render_cpu_mode<0>(p,c,d,i,r);break;case 1:render_cpu_mode<1>(p,c,d,i,r);break;
  case 2:render_cpu_mode<2>(p,c,d,i,r);break;case 3:render_cpu_mode<3>(p,c,d,i,r);break;
  case 4:render_cpu_mode<4>(p,c,d,i,r);break;case 5:render_cpu_mode<5>(p,c,d,i,r);break;default:throw std::invalid_argument("render mode");
 }
}
}
extern "C" {
void lj_unpack_color(const uint32_t*in,uint8_t*out,int pixels){for(int k=0;k<pixels;k++){uint32_t p=in[k];out[3*k]=p;out[3*k+1]=p>>8;out[3*k+2]=p>>16;}}
int lj_register_color(const float*depth,const uint8_t*rgb,float*outdepth,uint8_t*outcolor,int w,int h,int cw,int ch,const double*dc,const double*cc,const double*dist,const double*T){
 if(w<1||h<1||cw<1||ch<1||w>640||h>480||cw>640||ch>480||!(dc[0]>0)||!(dc[1]>0)||!(cc[0]>0)||!(cc[1]>0))return -1;
 const int n=w*h;std::vector<int>pixel(n,-1);std::vector<double>z(n),nearest(cw*ch,std::numeric_limits<double>::infinity());
#pragma omp parallel for schedule(static)
 for(int k=0;k<n;k++){
  double d=depth[k];if(!(d>0)||!std::isfinite(d))continue;double x=((k%w)-dc[2])*d/dc[0],y=((k/w)-dc[3])*d/dc[1];
  double px=T[0]*x+T[1]*y+T[2]*d+T[3],py=T[4]*x+T[5]*y+T[6]*d+T[7],pz=T[8]*x+T[9]*y+T[10]*d+T[11];if(!(pz>0))continue;
  double xx=px/pz,yy=py/pz,r2=xx*xx+yy*yy,r4=r2*r2,r6=r4*r2,radial=(1+dist[0]*r2+dist[1]*r4+dist[4]*r6)/(1+dist[5]*r2+dist[6]*r4+dist[7]*r6);
  double u=cc[0]*(xx*radial+2*dist[2]*xx*yy+dist[3]*(r2+2*xx*xx))+cc[2],v=cc[1]*(yy*radial+dist[2]*(r2+2*yy*yy)+2*dist[3]*xx*yy)+cc[3];
  if(!std::isfinite(u)||!std::isfinite(v)||u<-.5||u>cw-.5||v<-.5||v>ch-.5)continue;int ix=(int)std::lrint(u),iy=(int)std::lrint(v);if(ix<0||ix>=cw||iy<0||iy>=ch)continue;pixel[k]=iy*cw+ix;z[k]=pz;
 }
 // Serial scatter is deterministic and avoids a per-pixel atomic or 4 full
 // private RGB-depth buffers. Projection and gather are independently parallel.
 for(int k=0;k<n;k++)if(pixel[k]>=0)nearest[pixel[k]]=std::min(nearest[pixel[k]],z[k]);
#pragma omp parallel for schedule(static)
 for(int k=0;k<n;k++){int p=pixel[k];bool keep=p>=0&&z[k]<=nearest[p]+.02;outdepth[k]=keep?depth[k]:0;for(int c=0;c<3;c++)outcolor[k*3+c]=keep?rgb[p*3+c]:0;}
 return 0;
}
int lj_threads(int n){
#ifdef _OPENMP
 if(n>0&&n<=4)omp_set_num_threads(n);return omp_get_max_threads();
#else
 (void)n;return 1;
#endif
}
void*lj_create(int capacity,int stride){if(capacity<=0||capacity>1000000||stride<1||stride>8)return nullptr;auto*m=new Map;m->capacity=capacity;m->stride=stride;m->s.reserve(capacity);return m;}
void lj_destroy(void*p){delete static_cast<Map*>(p);}
uint32_t lj_count(void*p){auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);return m.s.size();}
uint64_t lj_version(void*p){auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);return m.version;}
uint64_t lj_capacity_drops(void*p){auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);return m.drops;}
int lj_copy(void*p,Surfel*out,int cap){auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);if(cap<(int)m.s.size())return -1;std::copy(m.s.begin(),m.s.end(),out);return m.s.size();}
int lj_set(void*p,const Surfel*in,int n){auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);if(n<0||n>m.capacity)return -1;for(int i=0;i<n;i++){const float*f=&in[i].x;for(int j=0;j<16;j++)if(!std::isfinite(f[j]))return -2;if(!(in[i].radius>0)||in[i].weight<1)return -2;for(int j=7;j<10;j++)if(f[j]<0||f[j]>255)return -2;}m.s.assign(in,in+n);m.version++;return n;}
static int prepare_export(void*p,const Camera*c,const float*T,Surfel*out,uint32_t*off,uint32_t*idx,int cap,bool packed,int outcap=-1,uint64_t expected_version=0,bool checked=false){
 if(!camera_ok(*c)||!transform_ok(T))return -1;
 auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);
 if(checked){if(m.version!=expected_version)return -5;if(outcap<0||m.s.size()>(size_t)outcap)return -8;}
 try{auto refs=project_count(m.s,*c,T,out,off);if(refs>4000000||cap<(int)refs)return -2;scatter(out,m.s.size(),*c,off,idx);
  if(packed){
#pragma omp parallel for schedule(static) if(m.s.size()>=1024)
   for(size_t i=0;i<m.s.size();i++){
    // The world map and CPU reference retain ordinary float bounds. Only the
    // exported DMA descriptor uses exact int32 words, saving four float-to-int
    // converters per fragment lane. Packing is inside the map-version lock.
    int32_t box[4]={(int32_t)out[i].submap,(int32_t)out[i].pad0,0,0};
    if(box[1]>=box[0]){box[2]=(int32_t)out[i].pad1;box[3]=(int32_t)out[i].pad2;}
    std::memcpy(&out[i].submap,box,sizeof(box));
   }
  }
  return refs;}catch(...){return -3;}
}
int lj_prepare(void*p,const Camera*c,const float*T,Surfel*out,uint32_t*off,uint32_t*idx,int cap){return prepare_export(p,c,T,out,off,idx,cap,false);}
int lj_prepare_packed(void*p,const Camera*c,const float*T,Surfel*out,uint32_t*off,uint32_t*idx,int cap){return prepare_export(p,c,T,out,off,idx,cap,true);}
int lj_prepare_checked(void*p,const Camera*c,const float*T,Surfel*out,uint32_t*off,uint32_t*idx,int cap,int outcap,uint64_t expected,int packed){if(packed<0||packed>1)return -1;return prepare_export(p,c,T,out,off,idx,cap,packed!=0,outcap,expected,true);}
int lj_validate_qrcp(void*p){auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);for(const auto&s:m.s){float r2=s.radius*s.radius;if(!(r2>=std::numeric_limits<float>::min())||!(r2<=0x1p126f))return -7;}return 0;}
int lj_validate_qrcp_normal_reciprocal(void*p){return lj_validate_qrcp(p);}
int lj_render(void*p,const Camera*c,const float*T,float*d,int32_t*i,uint32_t*r,int mode){
 if(!camera_ok(*c)||!transform_ok(T)||mode<0||mode>5)return -1;
 auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);
 // Both r^2 and its reciprocal must be normal FP32 values. A normal r^2
 // alone does not exclude a subnormal reciprocal (and hardware FTZ behavior).
 if(mode==5)for(const auto&s:m.s){float r2=s.radius*s.radius;if(!(r2>=std::numeric_limits<float>::min())||!(r2<=0x1p126f))return -7;}
 try{auto q=prepare(m.s,*c,T);render_cpu(q,*c,d,i,r,mode);return 0;}catch(...){return -3;}
}
static int integrate_impl(void*p,const float*d,const uint8_t*rgb,const Camera*c,const float*T,double stamp,const float*external_depth,const int32_t*external_ids,uint64_t expected_version){
 if(!camera_ok(*c)||!transform_ok(T)||!std::isfinite(stamp))return -1;
 auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);if(stamp<=m.last)return -2;
 const int n=c->width*c->height,old=m.s.size(),step=m.stride;int observations=0;double origin=m.origin<0?stamp:m.origin;std::vector<float>rd(n);std::vector<int32_t>ri(n);std::vector<uint32_t>rc(n);std::vector<uint8_t>seen(old,0);
 if(external_depth&&external_ids){
  if(expected_version!=m.version)return -5;
  for(int k=0;k<n;k++){int id=external_ids[k];float z=external_depth[k];if(id< -1||id>=old||(id>=0&&(!std::isfinite(z)||z<c->near_z||z>c->far_z)))return -6;rd[k]=z;ri[k]=id;}
 }else {try{auto q=prepare(m.s,*c,T);render_cpu(q,*c,rd.data(),ri.data(),rc.data(),3);}catch(...){return -3;}}
 auto valid=[&](float z){return std::isfinite(z)&&z>=c->near_z&&z<=c->far_z;};
 for(int y=1;y<c->height-1;y+=step)for(int x=1;x<c->width-1;x+=step){
  int k=y*c->width+x;float z=d[k],zr=d[k+1],zd=d[k+c->width];
  if(!valid(z)||!valid(zr)||!valid(zd)||std::fabs(z-zr)>.03f||std::fabs(z-zd)>.03f)continue;
  float px=(x-c->cx)*z/c->fx,py=(y-c->cy)*z/c->fy;
  float ax=(x+1-c->cx)*zr/c->fx-px,ay=(y-c->cy)*zr/c->fy-py,az=zr-z;
  float bx=(x-c->cx)*zd/c->fx-px,by=(y+1-c->cy)*zd/c->fy-py,bz=zd-z;
  float nx=ay*bz-az*by,ny=az*bx-ax*bz,nz=ax*by-ay*bx,len=std::sqrt(nx*nx+ny*ny+nz*nz);if(len<1e-10)continue;
  nx/=len;ny/=len;nz/=len;if(nx*px+ny*py+nz*z>0){nx=-nx;ny=-ny;nz=-nz;}observations++;
  Surfel s={};forward(T,px,py,z,s.x,s.y,s.z,true);forward(T,nx,ny,nz,s.nx,s.ny,s.nz,false);
  // Radius follows the measured tangent-plane footprint, not a fronto-parallel
  // assumption. Neighbours crossing a depth edge were rejected above.
  s.radius=std::min(.03f,.78f*step*std::sqrt(ax*ax+ay*ay+az*az+bx*bx+by*by+bz*bz));s.r=rgb[k*3];s.g=rgb[k*3+1];s.b=rgb[k*3+2];s.weight=1;s.last_seen=(float)(stamp-origin);s.submap=(float)((m.version)/100);
  int match=-1;float best=.03f*.03f;
  for(int oy=-1;oy<=1;oy++)for(int ox=-1;ox<=1;ox++){int id=ri[k+oy*c->width+ox];if(id<0||id>=old||std::fabs(rd[k+oy*c->width+ox]-z)>.03f)continue;const auto&a=m.s[id];float dx=a.x-s.x,dy=a.y-s.y,dz=a.z-s.z,dist=dx*dx+dy*dy+dz*dz;if(dist<best&&a.nx*s.nx+a.ny*s.ny+a.nz*s.nz>.9f){match=id;best=dist;}}
  if(match>=0){if(seen[match])continue;seen[match]=1;auto&a=m.s[match];float w=std::min(a.weight,31.f),v=1/(w+1);a.x=(a.x*w+s.x)*v;a.y=(a.y*w+s.y)*v;a.z=(a.z*w+s.z)*v;a.nx=(a.nx*w+s.nx)*v;a.ny=(a.ny*w+s.ny)*v;a.nz=(a.nz*w+s.nz)*v;float ln=std::sqrt(a.nx*a.nx+a.ny*a.ny+a.nz*a.nz);a.nx/=ln;a.ny/=ln;a.nz/=ln;a.radius=std::min(a.radius,s.radius);a.r=(a.r*w+s.r)*v;a.g=(a.g*w+s.g)*v;a.b=(a.b*w+s.b)*v;a.weight=std::min(32.f,a.weight+1);a.last_seen=s.last_seen;}
  else if((int)m.s.size()<m.capacity)m.s.push_back(s);else m.drops++;
 }
 if(!observations)return -4;
 m.origin=origin;m.last=stamp;m.version++;return (int)m.s.size()-old;
}
int lj_integrate(void*p,const float*d,const uint8_t*rgb,const Camera*c,const float*T,double stamp){return integrate_impl(p,d,rgb,c,T,stamp,nullptr,nullptr,0);}
int lj_integrate_associated(void*p,const float*d,const uint8_t*rgb,const Camera*c,const float*T,double stamp,const float*rd,const int32_t*ri,uint64_t version){if(!rd||!ri)return -6;return integrate_impl(p,d,rgb,c,T,stamp,rd,ri,version);}
}
