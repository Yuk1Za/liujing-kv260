"""Restore the verified live baseline, preserving all replaced files first.

Run with the board's PYNQ Python and exclusive hardware ownership. Does not
start sensors. The frozen archive and explicit hashes are the restore source.
"""
import fcntl,hashlib,json,os,sys,tarfile,time
from pathlib import Path
ROOT=Path('/home/ubuntu/liujing')
sys.path.insert(0,str(ROOT/'python'))
from hardware_fault import require_healthy
from hardware_ownership import require_no_localization_clients
require_healthy()
leases=[]
for path in ('/home/ubuntu/losam_live_20261021/pipeline.lock',str(ROOT/'live.lock')):
    f=open(path,'a');fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);leases.append(f)
require_no_localization_clients()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
baseline=ROOT/'baselines/live_cpu_20261027'
archive=baseline/'rollback.tar.gz'
if sha(archive)!='1b5fc3eed2bb31774b9aad229e8d630e7e7aebc73b7a58e4b270f70abe462c04':
    raise RuntimeError('frozen archive digest mismatch')
out=ROOT/'baselines'/('before_restore_'+time.strftime('%Y%m%d_%H%M%S'))
out.mkdir(exist_ok=False)
with tarfile.open(archive) as source:
    members=source.getmembers()
    for member in members:
        path=ROOT/member.name
        if Path(member.name).is_absolute() or not path.resolve().is_relative_to(ROOT.resolve()) or not (member.isfile() or member.isdir()):
            raise RuntimeError('unsafe archive entry')
    with tarfile.open(out/'replaced_files.tar.gz','w:gz') as backup:
        for member in members:
            path=ROOT/member.name
            if path.exists():backup.add(path,arcname=member.name,recursive=False)
    expected={}
    for member in members:
        path=ROOT/member.name
        if member.isdir():path.mkdir(parents=True,exist_ok=True);continue
        data=source.extractfile(member).read();path.parent.mkdir(parents=True,exist_ok=True)
        temp=path.with_name(path.name+'.frozen_restore_pending')
        with temp.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
        os.chmod(temp,member.mode);os.chown(temp,member.uid,member.gid);os.replace(temp,path)
        expected[member.name]=hashlib.sha256(data).hexdigest()
    if any(sha(ROOT/name)!=digest for name,digest in expected.items()):raise RuntimeError('restored file mismatch')
required={
 'native_fusion_overflow_build/liujing_native':'fe96fd944aeb7ce34508c2cce298aca217a54bffaa5856c11d5d2c1dea20a7a9',
 'artifacts/opaque_v1/liujing.bit':'4d0e36c6e8b82949680b1e8ca41e93119bbf773c5479f7f46281ee382f6463a5',
 'artifacts/opaque_v1/liujing.hwh':'fd299b4b79c52af8a813e701c67aa6d4e43b74dd071580f45ca089e6c3e3ffbb',
 'configs/live_candidate_20261026.json':'464d98f452b28e732bd2b5ae5f84ada98a307a9b2b5106c130682fffa41ab1e0'}
if any(sha(ROOT/name)!=digest for name,digest in required.items()):raise RuntimeError('verified live identity mismatch')
import pynq.pl_server.embedded_device as ed
ed.EmbeddedDevice._xrt_download=lambda self,data:None
from pynq import Overlay,Clocks
from pynq.pl_server.global_state import GlobalState,save_global_state
bit=ROOT/'artifacts/opaque_v1/liujing.bit'
overlay=Overlay(str(bit))
if Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()!='operating':raise RuntimeError('FPGA configuration failed')
if abs(Clocks.fclk0_mhz-100)>.1 or 'fpga_feature_extract_0' not in overlay.ip_dict or 'lj_associate_0' in overlay.ip_dict:raise RuntimeError('wrong frozen platform')
state=GlobalState(bitfile_name=str(bit),active_name=overlay.device.name,timestamp=overlay.timestamp,psddr=overlay.mem_dict.get('PSDDR',{}))
for name,ip in overlay.ip_dict.items():
    if ip.get('type') in ('xilinx.com:ip:pr_axi_shutdown_manager:1.0','xilinx.com:ip:dfx_axi_shutdown_manager:1.0'):state.add(name,ip['phys_addr'])
save_global_state(state)
proof={'status':'RESTORED_AND_PL_LOADED','files_verified':len(expected),'sha256':required,'restored_file_sha256':expected,'backup':str(out/'replaced_files.tar.gz'),'backup_sha256':sha(out/'replaced_files.tar.gz'),'bitfile':str(bit),'clock_mhz':Clocks.fclk0_mhz,'ip_names':sorted(overlay.ip_dict),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'sensors_started':False,'fusion_backend':'cpu_overflow_reference','field_demo_validation':'NOT_RUN_SENSORS_DISCONNECTED'}
(out/'restore_result.json').write_text(json.dumps(proof,indent=2))
print(json.dumps({k:v for k,v in proof.items() if k!='restored_file_sha256'},indent=2))
