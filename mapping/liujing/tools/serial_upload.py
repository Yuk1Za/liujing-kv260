"""Bounded-line serial upload with SHA256 verification and baud restoration.

Only run while the board console is idle. Uses a new destination, never replaces
an existing file. Linux canonical input lines remain below 4096 bytes.
"""
import argparse,base64,hashlib,shlex,time
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('file');p.add_argument('--dest',required=True);p.add_argument('--port',default='COM14');p.add_argument('--baud',type=int,default=460800);a=p.parse_args()
data=Path(a.file).read_bytes();sha=hashlib.sha256(data).hexdigest();s=serial.Serial(a.port,115200,timeout=.05,write_timeout=10)
def drain(seconds):
 end=time.monotonic()+seconds;b=b''
 while time.monotonic()<end:b+=s.read(s.in_waiting or 1)
 return b
def until(token):
 end=time.monotonic()+10;b=b''
 while token not in b and time.monotonic()<end:b+=s.read(s.in_waiting or 1)
 if token not in b:raise RuntimeError('Missing '+repr(token)+'; '+repr(b[-300:]))
 return b
receiver="import sys,base64,hashlib; f=open("+repr(a.dest)+",'xb'); print('READY',flush=True)\nwhile True:\n line=sys.stdin.readline().strip()\n if line=='END':break\n f.write(base64.b64decode(line,validate=True));print('ACK',flush=True)\nf.close();print('SHA256='+hashlib.sha256(open("+repr(a.dest)+",'rb').read()).hexdigest(),flush=True)"
try:
 s.write(('stty -echo '+str(a.baud)+'\r').encode());s.flush();time.sleep(.3);s.baudrate=a.baud;drain(.3)
 s.write(('python3 -c '+shlex.quote(receiver)+'\r').encode());s.flush();until(b'READY')
 for offset in range(0,len(data),1536):
  s.write(base64.b64encode(data[offset:offset+1536])+b'\n');until(b'ACK')
  if offset%153600==0:print(offset,'/',len(data),flush=True)
 s.write(b'END\n');until(('SHA256='+sha).encode());print('VERIFIED',len(data),sha,flush=True)
finally:
 s.write(b'\x03');time.sleep(.1);s.write(b'stty echo 115200\r');s.flush();time.sleep(.3);s.baudrate=115200;drain(.3);s.close()
