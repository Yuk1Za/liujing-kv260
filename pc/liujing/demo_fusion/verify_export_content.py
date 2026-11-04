"""Independent PLY parser: compare every XYZ byte and RGB channel to Surface64."""
import hashlib,json,struct
from pathlib import Path
root=Path(__file__).resolve().parent.parent
out=root/'evidence/viewer_redesign_20261028'
source=root/'evidence/fusion_variable_20261027/abi_board.bin'
export=out/'liujing-map-v120.ply'
raw=source.read_bytes();ply=export.read_bytes()
offset=ply.index(b'end_header\n')+len(b'end_header\n')
header=ply[:offset].decode('ascii').splitlines()
assert header[:2]==['ply','format binary_little_endian 1.0']
assert header[4:-1]==['property float x','property float y','property float z','property uchar red','property uchar green','property uchar blue']
count=int(next(line.split()[-1] for line in header if line.startswith('element vertex ')))
assert count==54876==len(raw)//64 and len(raw)%64==0
assert len(ply)==offset+count*15
xyz_mismatches=rgb_mismatches=0
for i in range(count):
    vertex=ply[offset+i*15:offset+(i+1)*15]
    xyz_mismatches+=vertex[:12]!=raw[i*64:i*64+12]
    rgb_mismatches+=vertex[12:]!=raw[i*64+28:i*64+31]
assert xyz_mismatches==rgb_mismatches==0
result={'scope':'Actual file from the shared browser serializer, independently parsed; browser download location not asserted','source':str(source),'source_sha256':hashlib.sha256(raw).hexdigest(),'export':str(export),'export_sha256':hashlib.sha256(ply).hexdigest(),'header':header,'vertices':count,'file_bytes':len(ply),'xyz_bitwise_mismatches':xyz_mismatches,'rgb_byte_mismatches':rgb_mismatches,'passed':True}
(out/'ply_content_validation.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
