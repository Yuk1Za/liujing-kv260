"""Isolated same-input CPU/PL check. No sensors, recording or CPU fallback."""
import argparse,fcntl,hashlib,json,os,signal,subprocess,sys,time,threading
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
def main():
    p=argparse.ArgumentParser();p.add_argument('--artifact',required=True);p.add_argument('--out',required=True);p.add_argument('--joint-rounds',type=int,default=0);p.add_argument('--stress',action='store_true');p.add_argument('--pipeline-seconds',type=int,default=0);p.add_argument('--build-dir',default=str(ROOT/'native_v2_build'));p.add_argument('--live-sizes',action='store_true');a=p.parse_args()
    if not 0<=a.joint_rounds<=20:raise ValueError('joint rounds must be zero to 20')
    if a.pipeline_seconds and not 10<=a.pipeline_seconds<=120:raise ValueError('pipeline seconds must be 10..120')
    artifact=Path(a.artifact);out=Path(a.out);manifest=json.loads((artifact/'manifest.json').read_text())
    if manifest.get('contract')!='opaque_v1' or manifest.get('metrics_contract')!='passive_axi_metrics_v1' or not manifest.get('routed_timing_passed') or not manifest.get('rtl_cosim_passed'):raise ValueError('unaccepted hardware contract')
    for name,key in [('liujing.bit','bit_sha256'),('liujing.hwh','hwh_sha256')]:
        if hashlib.sha256((artifact/name).read_bytes()).hexdigest()!=manifest[key]:raise ValueError('artifact hash mismatch')
    if manifest.get('xclbin_sha256') and hashlib.sha256((artifact/'liujing.xclbin').read_bytes()).hexdigest()!=manifest['xclbin_sha256']:raise ValueError('XCLBIN metadata hash mismatch')
    native_env=os.environ.copy()
    native_env.update(OMP_NUM_THREADS='2',OMP_THREAD_LIMIT='2',OMP_WAIT_POLICY='PASSIVE',GOMP_SPINCOUNT='0',OPENBLAS_NUM_THREADS='1')
    locks=[]
    for path in (Path('/home/ubuntu/losam_live_20261021/pipeline.lock'),ROOT/'live.lock'):
        f=open(path,'a');fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(f)
    import hardware_ownership
    from hardware_fault import require_healthy
    require_healthy()
    hardware_ownership.require_no_localization_clients()
    import pynq.pl_server.embedded_device as ed
    ed.EmbeddedDevice._xrt_download=lambda self,data:None
    from pynq import Overlay,Clocks
    overlay=Overlay(str(artifact/'liujing.bit'))
    if abs(Clocks.fclk0_mhz-100)>.1:raise RuntimeError('wrong PL clock')
    out.mkdir(parents=True,exist_ok=False)
    from native_allocation import allocate_native
    buffers,allocation=allocate_native(overlay,out,artifact/'liujing.bit')
    hardware_ownership._reconfiguration_lease.close();hardware_ownership._reconfiguration_lease=None
    probe=None;failure=None;stop=threading.Event();worker=None;joint=[];preflight=[]
    try:
        from native_c16_probe import C16Probe
        probe=C16Probe(overlay)
        preflight=[probe.run(False),probe.run(True),probe.run(False)]
        if preflight[0]['hash']!=preflight[2]['hash']:raise RuntimeError('C16 A/B/A mismatch')
        def run_c16():
            while not stop.is_set():
                begin=time.monotonic()
                try:joint.append(dict(mono=begin,**probe.run(False)))
                except Exception as exc:
                    joint.append({'error':repr(exc)});stop.set();return
                stop.wait(max(0,.05-(time.monotonic()-begin)))
        if a.joint_rounds:
            worker=threading.Thread(target=run_c16);worker.start()
        with (out/'comparison.jsonl').open('w') as log:
            for _ in range(max(1,a.joint_rounds)):
                result=subprocess.run([str(Path(a.build_dir)/('lj2_live_check' if a.live_sizes else 'lj2_pl_check')),str(allocation)]+(['--stress'] if a.stress and not a.live_sizes else []),stdout=log,stderr=subprocess.STDOUT,env=native_env,pass_fds=tuple(b.native_export_fd for b in buffers))
                if result.returncode:raise RuntimeError('CPU/PL comparison failed')
                if stop.is_set():raise RuntimeError('C16 concurrent check failed')
        if a.pipeline_seconds:
            with (out/'pipeline.jsonl').open('w') as log:
                result=subprocess.run([str(Path(a.build_dir)/'lj2_stream_bench'),str(allocation),str(a.pipeline_seconds)],stdout=log,stderr=subprocess.STDOUT,env=native_env,pass_fds=tuple(b.native_export_fd for b in buffers))
                if result.returncode or stop.is_set():raise RuntimeError('Synthetic pipeline check failed')
    except Exception as exc:failure=repr(exc)
    finally:
        stop.set()
        if worker:worker.join()
        (out/'c16_check.json').write_text(json.dumps({'scope':'analytic IP integration check; not live localization','preflight':preflight,'joint':joint,'failure':failure},indent=2))
    dma_active=not (overlay.lj_raster_0.read(0)&4) or (probe is not None and not (overlay.fpga_feature_extract_0.read(0)&4))
    if dma_active:
        from hardware_fault import latch_timeout
        try:latch_timeout({'run':str(out),'reason':'test process exited while DMA active'})
        except OSError as exc:print('Fault latch write failed:',repr(exc),flush=True)
    if dma_active or Path('/run/lock/liujing_raster_fault.json').exists():
        # Keep allocator alive even after an operator's stop request. Reboot is
        # the recovery action; releasing physical memory under active DMA is unsafe.
        signal.signal(signal.SIGINT,lambda *_:None);signal.signal(signal.SIGTERM,lambda *_:None)
        print('PL fault: buffers pinned until reboot',flush=True)
        while True:time.sleep(1)
    if failure:raise RuntimeError(failure+'; inspect comparison.jsonl and c16_check.json')
    print((out/'comparison.jsonl').read_text())
if __name__=='__main__':main()
