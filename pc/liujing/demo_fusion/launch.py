"""Bounded isolated fusion session; authenticate via stdin/getpass, never persist."""
import argparse,getpass,json,hashlib,pathlib,select,socketserver,subprocess,sys,threading,time,shlex
import paramiko
p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=180);p.add_argument('--hz',type=float,default=15);p.add_argument('--capture',type=int,default=0);p.add_argument('--password-stdin',action='store_true');p.add_argument('--no-start',action='store_true');p.add_argument('--out',type=pathlib.Path,required=True);p.add_argument('--fusion-backend',choices=['cpu','pl'],default='cpu');p.add_argument('--artifact');p.add_argument('--cpu-layout',choices=['shared','fusion-core'],default='shared');p.add_argument('--input-max-age',type=float,default=.3);p.add_argument('--input-policy',choices=['source_cadence','latest_eligible']);p.add_argument('--association-threads',type=int,choices=[1,2],default=2);p.add_argument('--preprocess-demand',action='store_true');p.add_argument('--fusion-reference',choices=['committed','overflow','variable','cached','pipeline','dataflow'],default='committed');p.add_argument('--cpu-association',choices=['candidate','query'],default='candidate');p.add_argument('--fast-tiles',action='store_true');p.add_argument('--fast-prepare',action='store_true');p.add_argument('--host',default='10.201.164.173');p.add_argument('--frozen-baseline',action='store_true',help='Verify the frozen live archive files before starting');p.add_argument('--open-browser',action='store_true');a=p.parse_args()
if not 30<=a.seconds<=3600 or not 1<=a.hz<=30:raise ValueError('duration/rate')
if a.input_policy and a.fusion_reference not in ('pipeline','dataflow'):raise ValueError('explicit input policy requires the pipeline runtime')
if a.fast_prepare and a.fusion_reference not in ('cached','pipeline','dataflow'):raise ValueError('fast preparation requires the cached runtime build')
if a.cpu_association=='query' and (a.fusion_reference not in ('overflow','variable','cached','pipeline','dataflow') or a.fusion_backend!='cpu'):raise ValueError('query SIMD requires CPU overflow reference')
if a.fast_tiles and (a.fusion_reference not in ('overflow','variable','cached','pipeline','dataflow') or (a.fusion_backend!='cpu' and a.fusion_reference not in ('variable','cached','pipeline','dataflow'))):raise ValueError('fast tile index requires overflow or variable backend')
if a.fusion_reference=='overflow' and a.fusion_backend!='cpu':raise ValueError('overflow reference is CPU only; no fallback')
password=sys.stdin.readline().rstrip('\r\n') if a.password_stdin else getpass.getpass('KV260 password: ')
c=paramiko.SSHClient();c.load_system_host_keys();c.connect(a.host,username='ubuntu',password=password,look_for_keys=False,allow_agent=False,timeout=10,auth_timeout=12,banner_timeout=12);c.get_transport().set_keepalive(10)
unit='liujing-fusion-'+time.strftime('%Y%m%d-%H%M%S');a.out.mkdir(parents=True,exist_ok=True);servers=[];child=None;owned=False
def command(cmd,sudo=False):
 i,o,e=c.exec_command(('sudo -S -p "" ' if sudo else '')+cmd,timeout=20)
 if sudo:i.write(password+'\n');i.flush()
 out=o.read().decode();err=e.read().decode();code=o.channel.recv_exit_status()
 if code:raise RuntimeError(err or out)
 return out
try:
 if a.frozen_baseline:
  if a.no_start or a.fusion_reference!='overflow' or a.fusion_backend!='cpu' or a.cpu_association!='query' or not a.fast_tiles or not a.preprocess_demand or a.association_threads!=2 or a.cpu_layout!='shared' or a.input_max_age!=.3 or a.hz!=15 or a.capture or a.fast_prepare or a.input_policy or a.artifact not in (None,'/home/ubuntu/liujing/artifacts/opaque_v1'):
   raise ValueError('frozen live baseline requires its exact CPU overflow settings')
  import tarfile
  archive=pathlib.Path(__file__).resolve().parents[1]/'evidence/rollback_baseline_20261028/rollback.tar.gz'
  if hashlib.sha256(archive.read_bytes()).hexdigest()!='1b5fc3eed2bb31774b9aad229e8d630e7e7aebc73b7a58e4b270f70abe462c04':raise RuntimeError('frozen archive changed')
  with tarfile.open(archive) as frozen:
   expected={}
   for member in frozen.getmembers():
    if member.isfile():
     if pathlib.PurePosixPath(member.name).is_absolute() or '..' in pathlib.PurePosixPath(member.name).parts:raise ValueError('baseline member path')
     expected['/home/ubuntu/liujing/'+member.name]=hashlib.sha256(frozen.extractfile(member).read()).hexdigest()
  result=command('sha256sum '+' '.join(shlex.quote(name) for name in expected))
  actual={line.split(maxsplit=1)[1].strip():line.split(maxsplit=1)[0] for line in result.splitlines()}
  if actual!=expected:raise RuntimeError('board differs from frozen baseline; stopped before starting sensors')
  (a.out/'baseline_verified.json').write_text(json.dumps({'status':'PASSED','files':len(expected),'sha256':expected},indent=2))
  print('Frozen live baseline verified: '+str(len(expected))+' files',flush=True)
 if not a.no_start:
  command('test ! -f /run/lock/liujing_raster_fault.json')
  artifact=a.artifact or ('/home/ubuntu/liujing/artifacts/fusion_resident_v4' if a.fusion_backend=='pl' and a.fusion_reference=='dataflow' else '/home/ubuntu/liujing/artifacts/fusion_cached_protocol_v1' if a.fusion_backend=='pl' and a.fusion_reference=='pipeline' else '/home/ubuntu/liujing/artifacts/fusion_cached_v2' if a.fusion_backend=='pl' and a.fusion_reference=='cached' else '/home/ubuntu/liujing/artifacts/fusion_variable_v1' if a.fusion_backend=='pl' and a.fusion_reference=='variable' else '/home/ubuntu/liujing/artifacts/fusion_assoc_v1' if a.fusion_backend=='pl' else '/home/ubuntu/liujing/artifacts/opaque_v1')
  if not artifact.startswith('/home/ubuntu/liujing/artifacts/') or '..' in artifact:raise ValueError('artifact path')
  command('systemd-run --collect --unit='+unit+' --setenv='+shlex.quote('LIUJING_FUSION_ARTIFACT='+artifact)+' /bin/bash /home/ubuntu/liujing/tools/'+('fusion_dataflow.sh' if a.fusion_reference=='dataflow' else 'fusion_pipeline.sh' if a.fusion_reference=='pipeline' else 'fusion_cached.sh' if a.fusion_reference=='cached' else 'fusion_variable.sh' if a.fusion_reference=='variable' else 'fusion_overflow.sh' if a.fusion_reference=='overflow' else 'fusion.sh')+' run --seconds '+str(a.seconds)+' --fusion-hz '+str(a.hz)+' --capture-frames '+str(a.capture)+' --fusion-backend '+a.fusion_backend+' --cpu-layout '+a.cpu_layout+' --input-max-age '+str(a.input_max_age)+' --association-threads '+str(a.association_threads)+(' --preprocess-demand' if a.preprocess_demand else '')+' --cpu-association '+a.cpu_association+(' --fast-tiles' if a.fast_tiles else '')+(' --fast-prepare' if a.fast_prepare else '')+(' --input-policy '+a.input_policy if a.input_policy else ''),True);owned=True
  time.sleep(1);command("systemctl is-active "+unit)
 for local,remote in ((18090,8090),(18092,8092)):
  def factory(port):
   class Forward(socketserver.BaseRequestHandler):
    def handle(self):
     ch=None
     try:
      ch=c.get_transport().open_channel('direct-tcpip',('127.0.0.1',port),self.request.getpeername(),timeout=4)
      while True:
       ready,_,_=select.select([self.request,ch],[],[],10)
       for src in ready:
        data=src.recv(65536)
        if not data:return
        (ch if src is self.request else self.request).sendall(data)
     except Exception:pass
     finally:
      if ch:ch.close()
   return Forward
  class Server(socketserver.ThreadingTCPServer):allow_reuse_address=True;daemon_threads=True
  server=Server(('127.0.0.1',local),factory(remote));threading.Thread(target=server.serve_forever,daemon=True).start();servers.append(server)
 with (a.out/'server.log').open('w') as log:
  child=subprocess.Popen([sys.executable,str(pathlib.Path(__file__).with_name('server.py')),'--out',str(a.out)],stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
  (a.out/'launch.json').write_text(json.dumps({'unit':unit,'seconds':a.seconds,'fusion_hz':a.hz,'fusion_backend':a.fusion_backend,'fusion_reference':a.fusion_reference,'association_threads':a.association_threads,'preprocess_demand':a.preprocess_demand,'cpu_association':a.cpu_association,'fast_tiles':a.fast_tiles,'cpu_layout':a.cpu_layout,'input_max_age':a.input_max_age,'artifact':a.artifact,'url':'http://127.0.0.1:8097','local_server_pid':child.pid,'web_source_sha256':{x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in pathlib.Path(__file__).with_name('web').glob('*') if x.is_file()}},indent=2))
  print('FUSION http://127.0.0.1:8097',flush=True)
  if a.open_browser:
   import urllib.request,webbrowser
   for _ in range(30):
    try:
     with urllib.request.urlopen('http://127.0.0.1:8097/',timeout=1) as response:
      if response.status==200:break
    except OSError:time.sleep(.1)
   webbrowser.open('http://127.0.0.1:8097',new=2)
  until=time.monotonic()+a.seconds+60
  while time.monotonic()<until and c.get_transport().is_active() and child.poll() is None:time.sleep(1)
finally:
 if owned and c.get_transport().is_active():
  try:command('systemctl kill --kill-who=main --signal=SIGINT '+unit,True)
  except Exception:pass
 for server in servers:server.shutdown();server.server_close()
 if child:child.terminate();child.wait(timeout=5)
 c.close()
