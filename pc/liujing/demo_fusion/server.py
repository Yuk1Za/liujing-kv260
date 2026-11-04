"""Local-only v3 UI, two native endpoints, and append-only client evidence."""
import argparse,json,time,urllib.request,urllib.error,urllib.parse,threading,struct,socket
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--port',type=int,default=8097);p.add_argument('--snapshot',type=Path,help='Read-only complete LJCD snapshot; never presented as live');p.add_argument('--snapshot-label',default='已保存的 RGB-D 融合地图');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
ROOT=Path(__file__).resolve().parent;lock=threading.Lock()
# Loopback forwards must not be routed through a system HTTP proxy.
local_http=urllib.request.build_opener(urllib.request.ProxyHandler({}))
snapshot=None;meta=None
if a.snapshot:
 snapshot=a.snapshot.read_bytes()
 if snapshot[:4]!=b'LJCD' or len(snapshot)<8:raise ValueError('LJCD snapshot required')
 length=struct.unpack_from('<I',snapshot,4)[0];meta=json.loads(snapshot[8:8+length])
 if meta.get('source')!='committed_fusion' or meta.get('full') not in (True,'true') or int(meta.get('base_version',-1))!=0 or int(meta['count'])!=int(meta['map_count']) or len(snapshot)!=8+length+int(meta['count'])*20:raise ValueError('complete committed snapshot required')
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*_):pass
 def reply(self,code,kind,data):
  try:
   self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers()
   self.wfile.write(data)
  except (ConnectionError,TimeoutError):
   self.close_connection=True
 def do_GET(self):
  if self.path.startswith('/api/'):
   if snapshot is not None:
    route=urllib.parse.urlsplit(self.path)
    if route.path=='/api/color':
     try:after=int(urllib.parse.parse_qs(route.query).get('after',['0'])[0])
     except ValueError:self.reply(400,'text/plain',b'bad version');return
     self.reply(204,'application/octet-stream',b'') if after==int(meta['version']) else self.reply(200,'application/octet-stream',snapshot)
    elif route.path=='/api/status':self.reply(200,'application/json',json.dumps({'session':meta['session'],'mono':time.monotonic(),'display_mode':'snapshot','snapshot_label':a.snapshot_label,'backend':'离线地图快照','map_version':meta['version'],'map_count':meta['map_count']},ensure_ascii=False).encode())
    else:self.reply(404,'text/plain',b'not available in snapshot mode')
    return
   suffix=self.path[4:];port=18092 if suffix.startswith('/color?') else 18090
   try:
    with local_http.open('http://127.0.0.1:'+str(port)+suffix,timeout=3) as r:
     code,kind,data=r.status,r.headers.get('Content-Type','application/octet-stream'),r.read(48_000_001)
   except (urllib.error.URLError,OSError):
    self.reply(503,'application/json',json.dumps({'error':'board disconnected','forward_port':port,'hint':'Check SSH forwarding and board service'}).encode());return
   if len(data)>48_000_000:
    self.reply(502,'application/json',b'{"error":"upstream payload exceeds 48 MB"}');return
   self.reply(code,kind,data)
   return
  name=self.path.split('?')[0];base=ROOT/'web'
  if name.startswith('/vendor/'):base=ROOT.parent/'demo'/'web';file=(base/name.lstrip('/')).resolve()
  else:file=(base/('index.html' if name=='/' else name.lstrip('/'))).resolve()
  if not file.is_relative_to(base.resolve()) or not file.is_file():self.reply(404,'text/plain',b'not found');return
  kind={'.js':'text/javascript','.html':'text/html','.css':'text/css'}.get(file.suffix,'application/octet-stream');self.reply(200,kind+'; charset=utf-8',file.read_bytes())
 def do_POST(self):
  n=int(self.headers.get('Content-Length','0'))
  if self.path!='/client-metrics' or not 0<n<1000000:self.reply(400,'text/plain',b'bad request');return
  try:
   data=json.loads(self.rfile.read(n));data['client_wall']=time.time()
   with lock:
    with (a.out/'browser_metrics.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(data)+'\n')
   self.reply(200,'application/json',b'{}')
  except ValueError:self.reply(400,'text/plain',b'invalid JSON')
server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler)
print(f'Viewer ready: http://127.0.0.1:{a.port}',flush=True)
if snapshot is None:
 for forward_port in (18090,18092):
  try:
   with socket.create_connection(('127.0.0.1',forward_port),timeout=.5):pass
   print(f'SSH local port {forward_port}: listening (board service not yet verified)',flush=True)
  except OSError:print(f'WARNING: SSH local port {forward_port} is not reachable. Start or reconnect the forwarding terminal.',flush=True)
try:server.serve_forever()
except KeyboardInterrupt:pass
finally:server.server_close()
