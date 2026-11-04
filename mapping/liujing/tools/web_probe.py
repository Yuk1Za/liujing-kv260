"""Check the board's own HTTP image and bounded latest-view request handling."""
import argparse,json,time,urllib.request
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args()
    root=Path(a.out);root.mkdir(parents=True,exist_ok=False)
    request=urllib.request.build_opener(urllib.request.ProxyHandler({})).open
    base='http://127.0.0.1:8090'
    def status():
        with request(base+'/status',timeout=3) as r:return json.load(r)
    before=status();responses=[];start=time.monotonic()
    for i in range(40):
        data=json.dumps({'mode':'follow'} if i==39 else {'mode':'free','yaw':.05 if i%2==0 else -.05}).encode()
        with request(urllib.request.Request(base+'/view',data=data,headers={'Content-Type':'application/json'}),timeout=3) as r:responses.append(r.status)
    time.sleep(3);after=status()
    with request(base+'/frame.jpg',timeout=3) as r:jpeg=r.read()
    if not jpeg.startswith(b'\xff\xd8'):raise RuntimeError('no board-generated JPEG')
    (root/'frame.jpg').write_bytes(jpeg)
    (root/'report.json').write_text(json.dumps({'scope':'board localhost HTTP; PC browser network not tested','before':before,'after':after,'request_count':len(responses),'all_204':all(x==204 for x in responses),'elapsed_wall_s':time.monotonic()-start,'jpeg_bytes':len(jpeg)},indent=2))
    print(json.dumps({'requests':len(responses),'all_204':all(x==204 for x in responses),'jpeg_bytes':len(jpeg)}))


if __name__=='__main__':main()
