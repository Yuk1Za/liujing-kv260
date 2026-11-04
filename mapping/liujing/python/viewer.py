"""Two-dimensional transport only. No scene geometry or WebGL on the client."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json,threading,io
from PIL import Image,ImageDraw
import numpy as np

def compose_trajectory(rgb,poses,T,cam,label=None):
    canvas=Image.fromarray(rgb);draw=ImageDraw.Draw(canvas);points=[]
    for v in poses:
        p=T[:3,:3].T@(v-T[:3,3])
        if p[2]>cam.near_z:
            x=int(cam.fx*p[0]/p[2]+cam.cx);y=int(cam.fy*p[1]/p[2]+cam.cy)
            if 0<=x<cam.width and 0<=y<cam.height:points.append((x,y))
    if len(points)>1:draw.line(points,fill=(255,170,40),width=2)
    if points:
        x,y=points[-1];draw.ellipse((x-4,y-4,x+4,y+4),fill=(255,245,130),outline=(25,25,25))
    if label:draw.text((8,8),label,fill=(255,170,40))
    return canvas
def encode_frame(rgb):
    buf=io.BytesIO();im=rgb if isinstance(rgb,Image.Image) else Image.fromarray(rgb);im.save(buf,format='JPEG',quality=88,subsampling=0);return buf.getvalue()
HTML='''<!doctype html><meta charset="utf-8"><title>留境 · 在线空间记录</title>
<style>body{background:#101b25;color:#e4eef3;font:16px system-ui;max-width:1050px;margin:32px auto}img{width:100%;background:#050b10;image-rendering:auto}button,input{margin:8px;padding:8px}pre{white-space:pre-wrap;color:#9ed6bb}</style>
<h1>留境 · 在线空间记录</h1><p>画面由 KV260 生成 · 人工遥控采集</p><img id="view"><div>
<button onclick="send({mode:'follow'})">跟随</button><button onclick="send({mode:'top'})">俯视</button>
<button onclick="send({mode:'free',dx:-.1})">左移</button><button onclick="send({mode:'free',dx:.1})">右移</button>
<button onclick="send({mode:'free',dz:.1})">前移</button><button onclick="send({mode:'free',dz:-.1})">后移</button>
<button onclick="send({yaw:-.1})">左看</button><button onclick="send({yaw:.1})">右看</button>
<button onclick="send({render_mode:0})">彩色</button><button onclick="send({render_mode:1})">几何</button><button onclick="send({render_mode:2})">可信度</button></div><pre id="status"></pre>
<script>let version=0,url=null;async function send(v){await fetch('/view',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(v)})}async function tick(){try{let s=await(await fetch('/status')).json();document.getElementById('status').textContent=JSON.stringify(s,null,2);if(s.frame_version&&s.frame_version!==version){const blob=await(await fetch('/frame.jpg')).blob();const old=url;url=URL.createObjectURL(blob);document.getElementById('view').src=url;if(old)URL.revokeObjectURL(old);version=s.frame_version}}catch(e){document.getElementById('status').textContent='连接已中断。保留的是上一帧，并非实时画面；请检查板端运行状态。'}setTimeout(tick,100)}tick()</script>'''
class FrameServer:
    def __init__(self,host='127.0.0.1',port=8090,on_view=None):
        self.lock=threading.Lock();self.jpeg=b'';self.state={'phase':'waiting_for_frames'};self.on_view=on_view;self.version=0
        parent=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*a):pass
            def do_GET(self):
                with parent.lock:
                    if self.path.startswith('/frame.jpg'):body=parent.jpeg;kind='image/jpeg'
                    elif self.path=='/status':body=json.dumps(parent.state,allow_nan=False).encode();kind='application/json'
                    elif self.path=='/':body=HTML.encode();kind='text/html; charset=utf-8'
                    else:self.send_error(404);return
                self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers()
                try:self.wfile.write(body)
                except (BrokenPipeError,ConnectionResetError):pass
            def do_POST(self):
                try:
                    if self.path!='/view':self.send_error(404);return
                    n=int(self.headers.get('Content-Length',0))
                    if not 0<n<=4096:raise ValueError('request length')
                    value=json.loads(self.rfile.read(n))
                    if not isinstance(value,dict):raise ValueError('object required')
                    if parent.on_view:parent.on_view(value)
                    self.send_response(204);self.end_headers()
                except (ValueError,TypeError,KeyError):self.send_error(400)
        self.http=ThreadingHTTPServer((host,port),Handler);self.thread=threading.Thread(target=self.http.serve_forever,daemon=True);self.thread.start()
    def update(self,rgb,status):
        encoded=encode_frame(rgb)
        with self.lock:self.jpeg=encoded;self.version+=1;self.state={**status,'frame_version':self.version,'jpeg_quality':88,'jpeg_subsampling':0}
    def update_status(self,status):
        with self.lock:self.state={**status,'frame_version':self.version}
    def close(self):self.http.shutdown();self.http.server_close();self.thread.join(timeout=2)
