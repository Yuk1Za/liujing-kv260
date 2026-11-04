"""Exclusive-console upload: sequence numbers, per-block SHA256, retry, final SHA.

Publish the final remote filename only after verification. Never overwrite a
destination or partial file. Keep failed partial files for diagnosis.
"""
import argparse,base64,hashlib,json,shlex,time
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('file');p.add_argument('--dest',required=True);p.add_argument('--port',default='COM14');p.add_argument('--baud',type=int,default=230400);p.add_argument('--report',required=True);p.add_argument('--inject-corrupt-block',type=int);a=p.parse_args()
data=Path(a.file).read_bytes();sha=hashlib.sha256(data).hexdigest();partial=a.dest+'.part'
report=dict(source=a.file,destination=a.dest,bytes=len(data),sha256=sha,baud=a.baud,retries=0,verified=False,injected_corruption=a.inject_corrupt_block)
if Path(a.report).exists():raise FileExistsError(a.report)
s=serial.Serial(a.port,115200,timeout=.1,write_timeout=10)
def drain(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:s.read(s.in_waiting or 1)
def response(prefix,timeout=15):
 end=time.monotonic()+timeout;buf=b''
 while time.monotonic()<end:
  buf+=s.read_until(b'\n')
  while b'\n' in buf:
   line,buf=buf.split(b'\n',1)
   if prefix==b'ACK ' and b'NACK ' in line:return line[line.find(b'NACK '):].strip()
   pos=line.find(prefix)
   if pos>=0:return line[pos:].strip()
   if b'Traceback' in line or b'ERROR' in line:raise RuntimeError(repr(line[-200:]))
 raise TimeoutError('missing response '+repr(prefix))
sender_config=json.dumps(dict(destination=a.dest,partial=partial,size=len(data),sha256=sha))
receiver="""import sys,os,json,hashlib,base64
c=json.loads(CONFIG)
if os.path.exists(c['destination']):raise FileExistsError(c['destination'])
f=open(c['partial'],'xb');digest=hashlib.sha256();expected=0;previous=None
print('READY',flush=True)
while True:
 line=sys.stdin.readline().strip()
 if line=='END':
  if f.tell()!=c['size'] or digest.hexdigest()!=c['sha256']:raise RuntimeError('ERROR final checksum')
  f.flush();os.fsync(f.fileno());f.close()
  os.link(c['partial'],c['destination']);os.unlink(c['partial'])
  print('VERIFIED '+c['sha256'],flush=True);break
 try:
  seq_text,block_sha,payload=line.split(' ');seq=int(seq_text);block=base64.b64decode(payload,validate=True)
  if not 0<len(block)<=768 or hashlib.sha256(block).hexdigest()!=block_sha:raise ValueError('block checksum')
  if seq==expected-1 and previous==(seq,block_sha):print('ACK '+str(seq),flush=True);continue
  if seq!=expected or f.tell()+len(block)>c['size']:raise ValueError('sequence/size')
 except Exception:
  print('NACK '+str(expected),flush=True);continue
 f.write(block);digest.update(block);previous=(seq,block_sha);expected+=1
 print('ACK '+str(seq),flush=True)
""".replace('CONFIG',repr(sender_config))
try:
 s.write(('stty -echo '+str(a.baud)+'\r').encode());s.flush();time.sleep(.3);s.baudrate=a.baud;drain(.3)
 # Keep the shell command on one physical line to avoid secondary prompt noise.
 command='python3 -c '+shlex.quote('exec('+repr(receiver)+')')
 if len(command.encode())>3500:raise ValueError('receiver command exceeds conservative console line limit')
 s.write((command+'\r').encode());s.flush();response(b'READY')
 for seq,offset in enumerate(range(0,len(data),768)):
  block=data[offset:offset+768];frame=str(seq).encode()+b' '+hashlib.sha256(block).hexdigest().encode()+b' '+base64.b64encode(block)+b'\n'
  for attempt in range(6):
   sent=frame
   if seq==a.inject_corrupt_block and attempt==0:
    corrupted=bytes([block[0]^1])+block[1:]
    sent=str(seq).encode()+b' '+hashlib.sha256(block).hexdigest().encode()+b' '+base64.b64encode(corrupted)+b'\n'
   s.write(sent);s.flush()
   try:
    # NACK contains ACK too: distinguish the complete response explicitly.
    reply=response(b'ACK ')
    if reply==('ACK '+str(seq)).encode():break
   except TimeoutError:pass
   report['retries']+=1
  else:raise RuntimeError('retry budget exceeded at block '+str(seq))
  if seq%200==0:print(offset,'/',len(data),'retries',report['retries'],flush=True)
 s.write(b'END\n');s.flush()
 if response(b'VERIFIED ')!=('VERIFIED '+sha).encode():raise RuntimeError('final verification mismatch')
 report['verified']=True;print(json.dumps(report),flush=True)
except Exception as e:
 report['error']=str(e);raise
finally:
 Path(a.report).write_text(json.dumps(report,indent=2))
 s.write(b'\x03');time.sleep(.1);s.write(b'stty echo 115200\r');s.flush();time.sleep(.3);s.baudrate=115200;drain(.3);s.close()
