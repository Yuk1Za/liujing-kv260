"""Read-only bounded C16 wire audit; exclusive sockets, no device commands."""
import argparse,fcntl,json,select,socket,statistics,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
a.output.mkdir(parents=True,exist_ok=False)
sockets=[];counts={2368:0,2369:0};wraps=[];previous=None;difop=None;invalid=0
try:
 for port in counts:
  s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024);s.bind(('0.0.0.0',port));sockets.append(s)
 start=time.monotonic()
 while time.monotonic()-start<6:
  ready,_,_=select.select(sockets,[],[],.2)
  for s in ready:
   data,address=s.recvfrom(65535);now=time.monotonic();port=s.getsockname()[1]
   if address[0]!='192.168.1.200':continue
   counts[port]+=1
   if counts[port]==1:(a.output/f'first_{port}.bin').write_bytes(data)
   if port==2369:
    if data[:4]==bytes.fromhex('a5ff005a') and len(data)>=1206:difop=data
   elif len(data)>=1206 and data[:2]==bytes.fromhex('ffee'):
    azimuth=int.from_bytes(data[2:4],'little')
    if azimuth>=36000:invalid+=1;continue
    if previous is not None and previous>30000 and azimuth<5000:wraps.append(now)
    previous=azimuth
   else:invalid+=1
 elapsed=time.monotonic()-start;gaps=[b-a for a,b in zip(wraps,wraps[1:])]
 result={'scope':'exclusive read-only UDP sockets; first-block azimuth wrap intervals, not ROS throughput','seconds':elapsed,'packets':counts,'msop_packets_per_second':counts[2368]/elapsed,'rotation_hz_from_wraps':(len(wraps)-1)/(wraps[-1]-wraps[0]) if len(wraps)>1 else None,'rotation_gap_median_s':statistics.median(gaps) if gaps else None,'rotation_gap_max_s':max(gaps) if gaps else None,'wrap_count':len(wraps),'invalid_msop':invalid,'difop_motor_rpm_field':int.from_bytes(difop[8:10],'big') if difop else None,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'unix_time':time.time()}
 (a.output/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
finally:
 for s in sockets:s.close()
