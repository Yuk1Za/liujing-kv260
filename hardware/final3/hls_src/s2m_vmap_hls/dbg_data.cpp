// 只看数据本身：查询点落在哪、27 邻域有几个有效格、最近格的残差是多少
#include <cstdio>
#include <cmath>
#include <vector>
#include "s2m_vmap.hpp"
static unsigned rs=12345u;
static float rnd(){ rs=rs*1664525u+1013904223u; return (float)((rs>>8)&0xFFFFFF)/(float)0x1000000; }
static const signed char DI[27]={-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1};
static const signed char DJ[27]={-1,-1,-1,0,0,0,1,1,1,-1,-1,-1,0,0,0,1,1,1,-1,-1,-1,0,0,0,1,1,1};
static const signed char DK[27]={-1,0,1,-1,0,1,-1,0,1,-1,0,1,-1,0,1,-1,0,1,-1,0,1,-1,0,1,-1,0,1};
int main(){
    const float L=1.6f, invL=1.f/L, rmax=0.4f;
    const float orgx=-25.f,orgy=-25.f,orgz=-3.f;
    const int WXv=32,WYv=32,WZv=8,W=WXv*WYv*WZv;
    std::vector<ap_uint<8>> vb(W/8,0); std::vector<float> tab((size_t)W*6,0.f);
    for(int i=0;i<W;++i){
        int k=i%WZv,j=(i/WZv)%WYv,ii=i/(WYv*WZv);
        float nx=rnd()-0.5f,ny=rnd()-0.5f,nz=rnd()-0.5f,nn=sqrtf(nx*nx+ny*ny+nz*nz);
        if(nn<1e-3f){nx=0;ny=0;nz=1;nn=1;} nx/=nn;ny/=nn;nz/=nn;
        float cx=orgx+(ii+0.5f)*L, cy=orgy+(j+0.5f)*L, cz=orgz+(k+0.5f)*L;
        float off=(rnd()-0.5f)*3.f*rmax;
        tab[6*i+0]=nx;tab[6*i+1]=ny;tab[6*i+2]=nz;tab[6*i+3]=-(nx*cx+ny*cy+nz*cz)+off;
        if(rnd()<0.55f) vb[i>>3]|=(ap_uint<8>)(1u<<(i&7));
    }
    // 200 个点在某一有效格的平面上
    int stat_nofind=0, stat_big=0;
    for(int t=0;t<200;++t){
        int c=-1; while(c<0){ int x=(int)(rnd()*(float)W)%W; if(((unsigned)vb[x>>3]>>(x&7))&1u) c=x; }
        int kk=c%WZv,jj=(c/WZv)%WYv,ii=c/(WYv*WZv);
        float gx=orgx+(ii+rnd())*L, gy=orgy+(jj+rnd())*L, gz=orgz+(kk+rnd())*L;
        float n0=tab[6*c],n1=tab[6*c+1],n2=tab[6*c+2],d0=tab[6*c+3];
        float rr0=n0*gx+n1*gy+n2*gz+d0;
        float back=-rr0+(rnd()-0.5f)*0.3f*rmax;
        if(t<4) printf("  [gen] c=%d n=(%.3f,%.3f,%.3f) d=%.3f g=(%.3f,%.3f,%.3f) rr0=%.3f back=%.3f\n",
                       c,n0,n1,n2,d0,gx,gy,gz,rr0,back);
        float px=gx+n0*back, py=gy+n1*back, pz=gz+n2*back;
        int ci=(int)floorf(px*invL),cj=(int)floorf(py*invL),ck=(int)floorf(pz*invL);
        int nval=0,npass=0; float bestd=1e30f, bestr=0; int bestn=-1;
        for(int n=0;n<27;++n){
            int i=ci+DI[n],j=cj+DJ[n],k=ck+DK[n];
            if(i<0||j<0||k<0||i>=WXv||j>=WYv||k>=WZv) continue;
            int idx=(i*WYv+j)*WZv+k; if(!(((unsigned)vb[idx>>3]>>(idx&7))&1u)) continue;
            ++nval;
            float ex=orgx+(i+0.5f)*L-px, ey=orgy+(j+0.5f)*L-py, ez=orgz+(k+0.5f)*L-pz;
            float e2=ex*ex+ey*ey+ez*ez;
            if(e2<bestd){bestd=e2;bestn=idx;}
        }
        if(bestn<0){++stat_nofind; if(t<6) printf("t=%d 无有效格 ci=%d cj=%d ck=%d\\n",t,ci,cj,ck); continue;}
        bestr=tab[6*bestn]*px+tab[6*bestn+1]*py+tab[6*bestn+2]*pz+tab[6*bestn+3];
        if(fabsf(bestr)>rmax) ++stat_big;
        if(t<6) printf("t=%d 有效邻格=%d 最近格残差=%+.4f  own=(%d,%d,%d) c=(%d,%d,%d)\\n",
                       t,nval,bestr,ci,cj,ck,ii,jj,kk);
    }
    printf("200 点里：无有效格 %d，最近格超 rmax %d\\n",stat_nofind,stat_big);
    return 0;
}
