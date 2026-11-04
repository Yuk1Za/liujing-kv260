"""Re-run accepted C16 A/B/A contract on the new integrated bitstream.

Writes all outputs into a new LiuJing run. Original C16 evidence is not changed.
"""
import argparse,importlib.util,shutil,fcntl,os,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
def main():
 p=argparse.ArgumentParser();p.add_argument('--bit',required=True);p.add_argument('--out',required=True);p.add_argument('--c16-root',default='/home/ubuntu/losam_live_20261021');p.add_argument('--restore-bit',required=True);a=p.parse_args()
 original=Path(a.c16_root);out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
 guard=open(original/'pipeline.lock','a');fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
 from hardware_ownership import require_no_localization_clients
 require_no_localization_clients()
 shutil.copyfile(original/'captured_c16.npz',out/'captured_c16.npz')
 # Preserve all original assertions, but retain CMA buffers until reconfiguration
 # in case its AP_DONE timeout fires while an AXI master is still accessing DDR.
 source=(original/'verify_c16.py').read_text();release='for b in buffers.values():b.freebuffer()'
 if source.count(release)!=1:raise RuntimeError('Original verifier cleanup changed; inspect before adaptation')
 snapshot=out/'verify_c16_snapshot.py';snapshot.write_text(source.replace(release,"globals()['_held_buffers']=buffers"))
 spec=importlib.util.spec_from_file_location('liujing_c16_verifier',snapshot);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.ROOT=out;mod.BIT=Path(a.bit)
 try:
  mod.main()
  old=json.loads((original/'c16_acceptance.json').read_text());new=json.loads((out/'c16_acceptance.json').read_text())
  match=[x['hashes'] for x in old['trials']]==[x['hashes'] for x in new['trials']]
  (out/'baseline_comparison.json').write_text(json.dumps({'all_output_hashes_equal_to_original':match},indent=2))
  if not match:raise RuntimeError('C16 integrated output differs from accepted baseline')
 finally:
  os.environ['PATH']='/usr/local/share/pynq-venv/bin:'+os.environ.get('PATH','');mod.Overlay(a.restore_bit)
  for b in getattr(mod,'_held_buffers',{}).values():b.freebuffer()
  guard.close()
if __name__=='__main__':main()
