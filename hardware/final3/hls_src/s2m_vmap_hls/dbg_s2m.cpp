// 最小用例：1 个有效格、平面过格心、查询点就在格心 ⇒ r=0，必然命中
#include <cstdio>
#include <cmath>
#include <vector>
#include "s2m_vmap.hpp"
int main(){
    const float L=1.6f, invL=1.f/L, rmax=0.4f;
    const float orgx=-25.f, orgy=-25.f, orgz=-3.f;
    const int WXv=32, WYv=32, WZv=8, WMAXv=WXv*WYv*WZv;
    std::vector<float> q(4,0.f), parq(12,0.f), parm(27,0.f), Hb(27,0.f);
    std::vector<ap_uint<8>> vb(WMAXv/8, 0);
    std::vector<float> tab((size_t)WMAXv*6, 0.f);
    // 只让 (16,16,4) 有效，平面 z = const（法向 (0,0,1)）过格心
    const int ii=16,j=16,k=4;
    const int idx=(ii*WYv+j)*WZv+k;
    vb[idx>>3] |= (ap_uint<8>)(1u<<(idx&7));
    const float cz = orgz+((float)k+0.5f)*L;
    tab[6*idx+0]=0; tab[6*idx+1]=0; tab[6*idx+2]=1; tab[6*idx+3]=-cz;
    // 查询点 = 格心（世界系），R=I，t=0
    q[0]=orgx+((float)ii+0.5f)*L; q[1]=orgy+((float)j+0.5f)*L; q[2]=cz; q[3]=1.f;
    parq[0]=1;parq[4]=1;parq[8]=1;
    parm[0]=1;parm[4]=1;parm[8]=1;   // M[0..8]=I，其余 0
    s2m_vmap(q.data(),vb.data(),tab.data(),parq.data(),parm.data(),Hb.data(),
             1,0,L,invL,rmax,orgx,orgy,orgz,WXv,WYv,WZv,1);
    printf("H00=%.6e b0=%.6e b3=%.6e  sum=%.6e\n",Hb[0],Hb[21],Hb[24],Hb[0]+Hb[1]);
    double s=0; for(int i=0;i<27;++i) s+=Hb[i];
    printf("sum27=%.6e\n",s);
    return 0;
}
