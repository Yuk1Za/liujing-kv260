#include "core.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <limits>
#include <stdexcept>
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
Prepared prepare(const std::vector<Surfel>&s,const Camera&c,const float*T){
 Prepared p;p.s.reserve(s.size());int tx=(c.width+15)/16,nt=tx*((c.height+15)/16);p.off.assign(nt+1,0);
 for(const auto&v:s)p.s.push_back(camera_surfel(v,T));
 for(auto&v:p.s){int a,b,d,e;if(!bounds(v,c,a,b,d,e))continue;v.submap=(float)a;v.pad0=(float)b;v.pad1=(float)d;v.pad2=(float)e;for(int y=d/16;y<=e/16;y++)for(int x=a/16;x<=b/16;x++)p.off[y*tx+x+1]++;}
 for(int i=1;i<=nt;i++)p.off[i]+=p.off[i-1];
 if(p.off.back()>4000000)throw std::runtime_error("tile index capacity exceeded");
 p.idx.resize(p.off.back());auto cursor=p.off;
 for(uint32_t i=0;i<p.s.size();i++){int a,b,d,e;if(!bounds(p.s[i],c,a,b,d,e))continue;for(int y=d/16;y<=e/16;y++)for(int x=a/16;x<=b/16;x++)p.idx[cursor[y*tx+x]++]=i;}
 return p;
}
// Independent CPU raster: clipped bounding rectangles, tile-owned output, no races.
void render_cpu(const Prepared&p,const Camera&c,float*depth,int32_t*ids,uint32_t*rgb,int mode){
 int txs=(c.width+15)/16,tys=(c.height+15)/16;
#pragma omp parallel for schedule(dynamic,4)
 for(int tile=0;tile<txs*tys;tile++){
  int tx=tile%txs,ty=tile/txs;float zbuf[256],sum[256]={},col[256][3]={};int owner[256];
  std::fill(zbuf,zbuf+256,c.far_z+1);std::fill(owner,owner+256,-1);
  for(int pass=0;pass<(mode==3?1:2);pass++)for(uint32_t j=p.off[tile];j<p.off[tile+1];j++){
   uint32_t id=p.idx[j];const Surfel&s=p.s[id];int x0,x1,y0,y1;bounds(s,c,x0,x1,y0,y1);
   x0=std::max(x0,tx*16);x1=std::min(x1,tx*16+15);y0=std::max(y0,ty*16);y1=std::min(y1,ty*16+15);
   for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++){
    float rx=(x-c.cx)/c.fx,ry=(y-c.cy)/c.fy,den=s.nx*rx+s.ny*ry+s.nz;
    if(std::fabs(den)<1e-6f)continue;
    float z=(s.nx*s.x+s.ny*s.y+s.nz*s.z)/den;
    if(!(z>=c.near_z&&z<=c.far_z))continue;
    float dx=rx*z-s.x,dy=ry*z-s.y,dz=z-s.z,q=(dx*dx+dy*dy+dz*dz)/(s.radius*s.radius);
    if(!(q<1))continue;
    int k=(y-ty*16)*16+x-tx*16;
    if(pass==0){if(z<zbuf[k]||(z==zbuf[k]&&(owner[k]<0||id<(uint32_t)owner[k]))){zbuf[k]=z;owner[k]=id;}}
    else if(std::fabs(z-zbuf[k])<=.01f){float w=(1-q)*std::min(32.f,std::max(1.f,s.weight));sum[k]+=w;float r=s.r,g=s.g,b=s.b;
     if(mode==1){r=127.5f*(s.nx+1);g=127.5f*(s.ny+1);b=127.5f*(s.nz+1);}else if(mode==2){float conf=std::min(1.f,s.weight/8);r=255*(1-conf);g=255*conf;b=32;}
     col[k][0]+=w*r;col[k][1]+=w*g;col[k][2]+=w*b;
    }
   }
  }
  for(int y=0;y<16;y++)for(int x=0;x<16;x++){int xx=tx*16+x,yy=ty*16+y;if(xx>=c.width||yy>=c.height)continue;int k=y*16+x,o=yy*c.width+xx;ids[o]=owner[k];depth[o]=owner[k]>=0?zbuf[k]:0;rgb[o]=0;if(sum[k]>0)for(int ch=0;ch<3;ch++)rgb[o]|=(uint32_t)std::clamp(col[k][ch]/sum[k]+.5f,0.f,255.f)<<(8*ch);}
 }
}
}
extern "C" {
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
int lj_set(void*p,const Surfel*in,int n){auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);if(n<0||n>m.capacity)return -1;for(int i=0;i<n;i++){const float*f=&in[i].x;for(int j=0;j<16;j++)if(!std::isfinite(f[j]))return -2;if(!(in[i].radius>0)||in[i].weight<1)return -2;}m.s.assign(in,in+n);m.version++;return n;}
int lj_prepare(void*p,const Camera*c,const float*T,Surfel*out,uint32_t*off,uint32_t*idx,int cap){
 if(!camera_ok(*c)||!transform_ok(T))return -1;
 auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);try{auto q=prepare(m.s,*c,T);if(cap<(int)q.idx.size())return -2;std::copy(q.s.begin(),q.s.end(),out);std::copy(q.off.begin(),q.off.end(),off);std::copy(q.idx.begin(),q.idx.end(),idx);return q.idx.size();}catch(...){return -3;}
}
int lj_render(void*p,const Camera*c,const float*T,float*d,int32_t*i,uint32_t*r,int mode){
 if(!camera_ok(*c)||!transform_ok(T)||mode<0||mode>2)return -1;
 auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);try{auto q=prepare(m.s,*c,T);render_cpu(q,*c,d,i,r,mode);return 0;}catch(...){return -3;}
}
int lj_integrate(void*p,const float*d,const uint8_t*rgb,const Camera*c,const float*T,double stamp){
 if(!camera_ok(*c)||!transform_ok(T)||!std::isfinite(stamp))return -1;
 auto&m=*static_cast<Map*>(p);std::lock_guard<std::mutex>l(m.mu);if(stamp<=m.last)return -2;
 const int n=c->width*c->height,old=m.s.size(),step=m.stride;int observations=0;double origin=m.origin<0?stamp:m.origin;std::vector<float>rd(n);std::vector<int32_t>ri(n);std::vector<uint32_t>rc(n);std::vector<uint8_t>seen(old,0);
 try{auto q=prepare(m.s,*c,T);render_cpu(q,*c,rd.data(),ri.data(),rc.data(),3);}catch(...){return -3;}
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
}
