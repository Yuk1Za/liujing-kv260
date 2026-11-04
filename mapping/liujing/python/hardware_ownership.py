"""Reject reconfiguration while known localization/accelerator clients are active."""
import os
from pathlib import Path
_reconfiguration_lease=None

def claim_renderer(path=None):
 """Exclusive renderer access, independent of the concurrently running C16 IP.

 Reconfiguration callers must first hold the original C16 pipeline.lock.
 """
 import fcntl
 path=Path(path) if path is not None else Path(__file__).resolve().parents[1]/'runs/renderer.lock'
 path.parent.mkdir(parents=True,exist_ok=True)
 lease=os.fdopen(os.open(path,os.O_RDONLY|os.O_CREAT,0o666),'r')
 try:fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except Exception:
  lease.close();raise RuntimeError('Renderer or reconfiguration already active; no hardware access')
 return lease

def require_no_localization_clients():
 from hardware_fault import require_healthy
 require_healthy()
 global _reconfiguration_lease
 if _reconfiguration_lease is None:_reconfiguration_lease=claim_renderer()
 ancestors=set();parent=os.getpid()
 while parent>1 and parent not in ancestors:
  ancestors.add(parent)
  try:parent=int(Path(f'/proc/{parent}/stat').read_text().rsplit(')',1)[1].split()[1])
  except FileNotFoundError:break
 names={'laserOdometry','map_optimization','feature_extract','pldataprocess','pynq_feature_extraction.py','live_frontend.py','c16_attached.py','liujing_native','lj2_pl_check','native_pl_smoke.py'}
 for proc in Path('/proc').glob('[0-9]*/cmdline'):
  if proc.parent.name==str(os.getpid()):continue
  try:
   args=[Path(os.fsdecode(x)).name for x in proc.read_bytes().split(b'\0') if x]
   # sudo/env wrappers repeat this child's script argument; they do not own IP.
   if int(proc.parent.name) in ancestors and args and args[0] in {'sudo','bash','sh','env','timeout','nohup'}:continue
   if names.intersection(args):raise RuntimeError('Existing localization/PL client PID '+proc.parent.name+'; do not reconfigure its FPGA')
  except (FileNotFoundError,PermissionError):continue
