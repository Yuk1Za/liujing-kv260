import sys, numpy as np, cv2
sys.path.insert(0,'/home/ubuntu/lvi_board')
from pl_match import PLMatcher, BIT
from vo_pnp_node import grab_img
m = PLMatcher(bit=BIT, max_cnt=60, min_dist=20, levels=2, max_iter=8)
def st_pts(g, q):
    m.detect(g, det='st')
    resp = np.array(m.rbuf).view(np.uint8).reshape(480,640)
    resp[:10,:]=0; resp[-10:,:]=0; resp[:,:10]=0; resp[:,-10:]=0
    thr = max(1,int(round(q*int(resp.max()))))
    r2 = np.where(resp>=thr, resp, 0).astype(np.uint8)
    from pl_match import select_min_dist_k
    p,_,_ = select_min_dist_k(r2, 60, 20, 600)
    return p.astype(np.float64), thr, int((r2>0).sum())
def rep(a,b,tol=2.0):
    d=np.linalg.norm(a[:,None,:]-b[None,:,:],axis=2); return (d.min(1)<=tol).mean()
print('frame  GFTT01  GFTT05  STq01(thr,n)  STq05(thr,n)  ov(ST01,CV05)  ov(ST05,CV05)  ov(CV01,CV05)')
for f in (0,60,300,600):
    g = cv2.cvtColor(grab_img(f,640,480), cv2.COLOR_BGR2GRAY)
    c1 = cv2.goodFeaturesToTrack(g,60,0.01,20); c1 = np.zeros((0,2)) if c1 is None else c1.reshape(-1,2)
    c5 = cv2.goodFeaturesToTrack(g,60,0.05,20); c5 = np.zeros((0,2)) if c5 is None else c5.reshape(-1,2)
    s1,t1,n1 = st_pts(g,0.01)
    s5,t5,n5 = st_pts(g,0.05)
    print('%5d %6d %7d   (%d,%d)      (%d,%d)      %5.0f%%        %5.0f%%        %5.0f%%'%(
        f,len(c1),len(c5),t1,n1,t5,n5,
        rep(s1,c5)*100 if len(c1) else 0, rep(s5,c5)*100 if len(c5) else 0, rep(c1,c5)*100))
