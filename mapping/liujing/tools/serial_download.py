"""Exclusive-console download with per-chunk ACK and verified atomic publication.

Leaves a .part file on failure, never overwrites an existing destination, and
restores the console baud. Do not run alongside another COM14 command.
"""
import argparse,base64,hashlib,json,os,shlex,time
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('--dest',required=True);p.add_argument('--port',default='COM14');p.add_argument('--baud',type=int,default=460800);a=p.parse_args()
dest=Path(a.dest);partial=dest.with_name(dest.name+'.part')
if dest.exists() or partial.exists():raise FileExistsError('download destination/partial already exists')
s=serial.Serial(a.port,115200,timeout=.2,write_timeout=10)
def drain(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:s.read(s.in_waiting or 1)
def line():
 end=time.monotonic()+15;data=b''
 while time.monotonic()<end:
  data+=s.read_until(b'\n')
  if data.endswith(b'\n'):return data.strip()
 raise TimeoutError('serial line timeout')
sender="""import sys,base64,hashlib,os,json
f=open(SOURCE,'rb')
sha=hashlib.file_digest(f,'sha256').hexdigest() if hasattr(hashlib,'file_digest') else hashlib.sha256(f.read()).hexdigest()
f.seek(0)
print('READY '+json.dumps(dict(size=os.fstat(f.fileno()).st_size,sha256=sha)),flush=True)
if sys.stdin.readline().strip()!='GO':raise RuntimeError('missing start')
while True:
 data=f.read(1536)
 if not data:break
 print(base64.b64encode(data).decode(),flush=True)
 if sys.stdin.readline().strip()!='ACK':raise RuntimeError('missing ACK')
f.close()
print('DONE',flush=True)
""".replace('SOURCE',repr(a.source))
try:
 s.write(('stty -echo '+str(a.baud)+'\r').encode());s.flush();time.sleep(.3);s.baudrate=a.baud;drain(.3)
 s.write(('python3 -c '+shlex.quote(sender)+'\r').encode());s.flush()
 header=line()
 marker=header.rfind(b'READY ')
 if marker<0:raise RuntimeError('unexpected sender response: '+repr(header[-200:]))
 info=json.loads(header[marker+6:]);s.write(b'GO\n');digest=hashlib.sha256();received=0;next_progress=0
 with partial.open('xb') as f:
  while received<info['size']:
   data=base64.b64decode(line(),validate=True)
   if not data or len(data)>1536 or received+len(data)>info['size']:raise ValueError('invalid chunk length')
   f.write(data);digest.update(data);received+=len(data);s.write(b'ACK\n')
   if received>=next_progress:print(received,'/',info['size'],flush=True);next_progress=received+262144
  f.flush();os.fsync(f.fileno())
 if line()!=b'DONE' or digest.hexdigest()!=info['sha256']:raise ValueError('download checksum/termination mismatch')
 # Path.rename on Windows fails if the destination appeared concurrently.
 partial.rename(dest);print('VERIFIED',received,digest.hexdigest(),flush=True)
finally:
 s.write(b'\x03');time.sleep(.1);s.write(b'stty echo 115200\r');s.flush();time.sleep(.3);s.baudrate=115200;drain(.3);s.close()
