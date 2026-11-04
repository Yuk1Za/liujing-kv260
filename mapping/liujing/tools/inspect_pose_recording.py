"""Read actual ROS Odometry frame identifiers from a recorded CDR header."""
import argparse,json,sqlite3,struct
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('bag');a=p.parse_args()
result={}
for db in sorted(Path(a.bag).glob('*.db3')):
    with sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True) as c:
        for tid,name in c.execute("select id,name from topics where type='nav_msgs/msg/Odometry'"):
            if name in result:continue
            row=c.execute('select data from messages where topic_id=? order by timestamp limit 1',(tid,)).fetchone()
            if not row:continue
            b=row[0];endian='<' if b[1]==1 else '>';n=struct.unpack_from(endian+'I',b,12)[0];frame=b[16:15+n].decode();offset=(16+n+3)//4*4
            m=struct.unpack_from(endian+'I',b,offset)[0];child=b[offset+4:offset+3+m].decode()
            result[name]={'frame':frame,'child':child,'stamp':struct.unpack_from(endian+'iI',b,4)}
print(json.dumps(result,indent=2))
