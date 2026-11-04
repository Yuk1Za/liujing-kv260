"""启动 CPU 融合 200 万点实车演示与浏览器。"""
import argparse,datetime,os,sys
from pathlib import Path
p=argparse.ArgumentParser(description='Start the KV260 CPU two-million-point demo and browser')
p.add_argument('--seconds',type=int,default=600)
p.add_argument('--host',default='192.168.1.102',help='KV260 address on the field router')
a=p.parse_args()
if not 30<=a.seconds<=3600:p.error('seconds must be between 30 and 3600')
root=Path(__file__).resolve().parents[1]
out=root/'evidence'/('capacity2m_demo_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
print('请先接好雷达和相机；开始后保持设备静止约 10 秒，再缓慢移动。',flush=True)
print('输入 KV260 密码后将自动打开浏览器。按 Ctrl+C 可提前停止。',flush=True)
print('版本：CPU 融合 · 200 万点；请使用已部署的扩容版板端程序。',flush=True)
print('KV260：'+a.host,flush=True)
print('演示时长：'+str(a.seconds)+' 秒；输出目录：'+str(out),flush=True)
args=[sys.executable,str(root/'demo_fusion/launch.py'),'--seconds',str(a.seconds),
      '--out',str(out),'--host',a.host,'--hz','15','--fusion-reference','overflow',
      '--fusion-backend','cpu','--cpu-association','query','--association-threads','2',
      '--fast-tiles','--preprocess-demand','--cpu-layout','shared','--input-max-age','.3',
      '--artifact','/home/ubuntu/liujing/artifacts/opaque_v1','--open-browser']
os.execv(sys.executable,args)
