"""Own one integrated overlay, sensor/ROS process groups (native v2 / legacy v5, no recording).

No chassis. Original C16 launcher and installed files remain untouched.
"""
import argparse, fcntl, hashlib, json, os, signal, subprocess, sys, time, uuid, shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOSAM = Path('/home/ubuntu/losam_live_20261021')
PYTHON = '/usr/local/share/pynq-venv/bin/python3'
STATE = ROOT/'runs/online_session.json'
TOPICS = ['/point_cloud_raw','/camera/color/image_raw/compressed','/camera/depth/image_raw',
          '/camera/frame_metadata','/camera/color/camera_info','/camera/depth/camera_info',
          '/odom_init/laser_odom','/mapping/odometry','/liujing/view_pose','/liujing/status']
INPUT_TOPICS = TOPICS[:6]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic(path, value):
    temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(value,indent=2));temp.replace(path)


def identity(pid):
    # PID reuse must not allow a stale stop request to signal another process.
    return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]


def current(state=None):
    state=STATE if state is None else state
    if not state.exists():return {'active':False,'status':'never_started'}
    data=json.loads(state.read_text())
    if data.get('boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        data['active']=False;data['previous_boot']=True;return data
    try: data['active']=identity(data['pid'])==data['start_ticks']
    except FileNotFoundError: data['active']=False
    return data


def main():
    p=argparse.ArgumentParser()
    p.add_argument('action',choices=['run','status','stop'])
    p.add_argument('--engine',choices=['legacy-v5','native-v2'],default='legacy-v5')
    p.add_argument('--bit-manifest')
    p.add_argument('--fusion-hz',type=float,default=15)
    p.add_argument('--fusion-backend',choices=['cpu','pl'],default='cpu')
    p.add_argument('--fusion-reference',choices=['committed','overflow','variable','cached','pipeline','dataflow','direct'],default='committed')
    p.add_argument('--association-threads',type=int,choices=[1,2],default=2)
    p.add_argument('--preprocess-demand',action='store_true')
    p.add_argument('--cpu-association',choices=['candidate','query'],default='candidate')
    p.add_argument('--fast-tiles',action='store_true');p.add_argument('--fast-prepare',action='store_true')
    p.add_argument('--input-max-age',type=float,default=.3)
    p.add_argument('--input-policy',choices=['source_cadence','latest_eligible'])
    p.add_argument('--cpu-layout',choices=['shared','fusion-core'],default='shared')
    p.add_argument('--capture-frames',type=int,default=0)
    p.add_argument('--render-size',choices=['640x480','320x240','256x192','160x120'],default='320x240')
    p.add_argument('--native-build',default=str(ROOT/'native_v2_live_build'))
    p.add_argument('--mode',choices=['calibrated','static_diagnostic'],default='calibrated')
    p.add_argument('--calibration',default=str(ROOT/'configs/jetson_candidates/initialization.json'))
    p.add_argument('--bit',default=str(ROOT/'artifacts/stream_v5/liujing.bit'))
    p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--replay',help='raw rosbag input; recorded odometry is never replayed')
    p.add_argument('--rate',type=float,default=1)
    p.add_argument('--lidar-only',action='store_true')
    p.add_argument('--backend',choices=['cpu','pl'],default='pl')
    p.add_argument('--port',type=int,default=8090)
    p.add_argument('--domain',type=int,default=42)
    p.add_argument('--demo-bridge',action='store_true')
    p.add_argument('--render-hz',type=float,default=15)
    p.add_argument('--pose-source',choices=['odometry','mapping'],default='odometry')
    p.add_argument('--static-warmup-seconds',type=float,default=10)
    p.add_argument('--observe-static-drift',action='store_true',help='only for operator-confirmed stationary diagnostics; drift remains a quality failure')
    a=p.parse_args()
    state=ROOT/'runs/edge_pc_session.json'
    if a.action!='run':
        data=current(state)
        data.setdefault('engine',a.engine)
        if a.action=='stop' and data['active']:os.kill(data['pid'],signal.SIGINT)
        print(json.dumps(data,indent=2));return
    if not 1<=a.fusion_hz<=30 or not 0<=a.capture_frames<=300:raise ValueError('fusion rate/capture bound')
    if not .1<=a.input_max_age<=.45:raise ValueError('fusion input age budget')
    if a.cpu_layout=='fusion-core' and a.fusion_backend!='pl':raise ValueError('dedicated fusion core requires PL association')
    if a.demo_bridge:raise ValueError('v3 serves committed color directly')
    if not .5<=a.render_hz<=15:raise ValueError('render rate outside .5..15')
    if a.demo_bridge and (a.engine!='native-v2' or a.lidar_only):raise ValueError('demo bridge requires native live camera')
    if not 0<=a.seconds<=86400 or not 0<a.rate<=1:raise ValueError('bounded duration/rate required')
    if not 0<=a.static_warmup_seconds<=30:raise ValueError('static warmup outside zero to 30 seconds')
    if a.observe_static_drift and a.mode!='static_diagnostic':raise ValueError('drift observation requires static diagnostic mode')
    if a.cpu_association=='query' and (a.fusion_reference not in ('overflow','variable','cached','pipeline','dataflow','direct') or a.fusion_backend!='cpu'):raise ValueError('query SIMD requires CPU overflow reference')
    if a.fast_tiles and (a.fusion_reference not in ('overflow','variable','cached','pipeline','dataflow','direct') or (a.fusion_backend!='cpu' and a.fusion_reference not in ('variable','cached','pipeline','dataflow','direct'))):raise ValueError('fast tile index requires overflow or variable backend')
    if a.input_policy and a.fusion_reference not in ('pipeline','dataflow','direct'):raise ValueError('explicit input policy requires the pipeline runtime')
    if a.fast_prepare and a.fusion_reference not in ('cached','pipeline','dataflow','direct'):raise ValueError('fast preparation requires the cached runtime build')
    native_source=ROOT/('native_fusion_direct' if a.fusion_reference=='direct' else 'native_fusion_dataflow' if a.fusion_reference=='dataflow' else 'native_pipeline' if a.fusion_reference=='pipeline' else 'native_fusion_cached' if a.fusion_reference=='cached' else 'native_fusion_variable' if a.fusion_reference=='variable' else 'native_edge_pc' if a.fusion_reference=='overflow' else 'native_fusion_v3')
    if a.fusion_reference=='overflow' and (a.fusion_backend!='cpu' or a.engine!='native-v2'):raise ValueError('overflow reference supports only explicit native CPU fusion; current PL ABI has four slots')
    if a.engine=='native-v2' and not a.lidar_only:
        cache=(Path(a.native_build)/'CMakeCache.txt').read_text()
        homes=[line.split('=',1)[1] for line in cache.splitlines() if line.startswith('CMAKE_HOME_DIRECTORY:INTERNAL=')]
        if homes!=[str(native_source)]:raise ValueError('native build/source selection mismatch')
    if a.replay and not (Path(a.replay)/'metadata.yaml').is_file():raise ValueError('rosbag metadata missing')
    from online_contract import validate_session
    if not a.lidar_only:validate_session(json.loads(Path(a.calibration).read_text()),a.mode)
    if a.engine=='native-v2' and not a.lidar_only and a.mode=='calibrated' and not json.loads(Path(a.calibration).read_text()).get('verified',{}).get('corrected_pose',False):raise ValueError('corrected pose convention not yet verified')
    if a.engine=='native-v2' and a.replay:raise ValueError('native product entry is live only')
    from hardware_fault import require_healthy
    require_healthy()
    ROOT.joinpath('runs').mkdir(exist_ok=True)
    lock=open(LOSAM/'pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    own=open(ROOT/'live.lock','a');fcntl.flock(own,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from hardware_ownership import require_no_localization_clients
    require_no_localization_clients()
    # Also refuse unmanaged sensor clients instead of killing them.
    for proc in Path('/proc').glob('[0-9]*/cmdline'):
        if proc.parent.name==str(os.getpid()):continue
        try: names={Path(os.fsdecode(x)).name for x in proc.read_bytes().split(b'\0') if x}
        except (PermissionError,FileNotFoundError):continue
        if names.intersection({'liujing_camera','liujing_native','lslidar_driver_node','camera_ros2'}):
            raise RuntimeError('Existing sensor process '+proc.parent.name)
    if shutil.disk_usage(ROOT).free<128*1024**2:raise RuntimeError('less than 128 MiB free for metrics')
    bit=Path(a.bit);hwh=bit.with_suffix('.hwh')
    if a.engine=='native-v2' and a.backend=='pl':
        if not a.bit_manifest:raise ValueError('opaque_v1 integrated manifest required')
        manifest=json.loads(Path(a.bit_manifest).read_text())
        if str(manifest.get('board_validation','')).startswith('FAILED'):raise ValueError('failed hardware artifact cannot start a live session')
        if a.fusion_backend=='pl' and not str(manifest.get('board_validation','')).startswith('PASSED_SAME_INPUT_'):
            raise ValueError('PL live session requires successful same-input board validation first')
        if manifest.get('required_binary_sha256') and digest(Path(a.native_build)/'liujing_native')!=manifest.get('required_live_binary_sha256'):
            raise ValueError('cache protocol artifact requires a separately identified live runtime binary')
        expected_contract=('fusion_resident_v4' if a.fusion_reference in ('dataflow','direct') else 'fusion_cached_v2' if a.fusion_reference in ('cached','pipeline','dataflow','direct') else 'fusion_variable_v1' if a.fusion_reference=='variable' else 'fusion_association_v1') if a.fusion_backend=='pl' else 'opaque_v1'
        if manifest.get('contract')!=expected_contract or manifest.get('routed_timing_passed') is not True or manifest.get('rtl_cosim_passed') is not True:raise ValueError('unverified integrated artifact')
        if digest(bit)!=manifest['bit_sha256'] or digest(hwh)!=manifest['hwh_sha256']:raise ValueError('opaque_v1 hash mismatch')
        if manifest.get('xclbin_sha256') and digest(bit.with_suffix('.xclbin'))!=manifest['xclbin_sha256']:raise ValueError('XCLBIN metadata hash mismatch')
    else:
        if digest(bit)!='396c59081b9291c6b6b9471536f5a852a4312adfdc1d3f9b645521d6ba639d02':raise ValueError('verified stream_v5 required for baseline/C16-only overlay')
        if digest(hwh)!='a65d438e839360b67ea516045ef7ce8952479f992628ede793aba5ea7a3def6f':raise ValueError('v5 HWH mismatch')
    run=ROOT/'runs'/('edge_pc_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);run.mkdir()
    calibration=Path(a.calibration)
    pose_topic='/odom_init/laser_odom'
    if a.pose_source=='mapping' and a.engine=='legacy-v5':
        cal=json.loads(calibration.read_text());cal.update(source_frame='camera_init',source_child_frame='odom_mapping',pose_source_topic='/mapping/odometry')
        cal['description']=cal.get('description','')+'; LO-SAM corrected mapping pose adapter; verification flags unchanged'
        calibration=run/'mapping_calibration.json';atomic(calibration,cal);pose_topic='/mapping/odometry'
    record={'pid':os.getpid(),'start_ticks':identity(os.getpid()),'run':str(run),'status':'starting',
            'bit_sha256':digest(bit),'hwh_sha256':digest(hwh),'backend':a.backend,'engine':a.engine,'mode':a.mode,
            'replay':a.replay,'rate':a.rate,'lidar_only':a.lidar_only,'domain':a.domain,
            'calibration_sha256':digest(calibration),'pose_source':'mapping_corrected_raw_increment' if a.engine=='native-v2' else a.pose_source,'raw_recording':bool(a.capture_frames),'source':{str(x.relative_to(ROOT)):digest(x) for x in (ROOT/'python/live.py',ROOT/'tools/online_session.py',ROOT/'python/c16_attached.py')},
            'static_warmup_seconds':a.static_warmup_seconds,'observe_static_drift':a.observe_static_drift,
            'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
    if a.engine=='native-v2':
        record['fusion_control_source_sha256']={str(x.relative_to(ROOT)):digest(x) for x in (ROOT/'tools/edge_pc_session.py',ROOT/('tools/fusion_direct.sh' if a.fusion_reference=='direct' else 'tools/fusion_dataflow.sh' if a.fusion_reference=='dataflow' else 'tools/fusion_pipeline.sh' if a.fusion_reference=='pipeline' else 'tools/fusion_cached.sh' if a.fusion_reference=='cached' else 'tools/fusion_variable.sh' if a.fusion_reference=='variable' else 'tools/edge_pc.sh' if a.fusion_reference=='overflow' else 'tools/fusion.sh'),ROOT/'tools/fusion_allocation.py',ROOT/'python/hardware_fault.py',ROOT/'python/hardware_ownership.py')}
        record['native_source_sha256']={str(x.relative_to(ROOT)):digest(x) for x in sorted(native_source.glob('*')) if x.is_file() and (x.suffix in ('.hpp','.cpp','.txt') or x.name=='CMakeLists.txt')}
        record['native_binary_sha256']=digest(Path(a.native_build)/'liujing_native') if not a.lidar_only else None
    record['render_size']=a.render_size
    record['requested_seconds']=a.seconds
    record['render_hz']=a.render_hz
    record['demo_bridge']=a.demo_bridge
    record['native_build']=str(Path(a.native_build).resolve())
    record['engine']=a.engine
    atomic(state,record);atomic(run/'manifest.json',record)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1',
        LOSAM_N_SCANS='16',LOSAM_BIT=str(bit),LOSAM_ATTACH_ONLY='1',LOSAM_RUN=str(run/'losam'),
        LIUJING_READY_DIR=str(run),
        LOSAM_SENSOR_MONITOR='driver',LOSAM_MODE='recorded_joint_replay' if a.replay else 'live_joint',
        LIUJING_RENDER_SIZE=a.render_size,LIUJING_LIBRARY=str(ROOT/'build_guarded/libliujing.so'),OMP_NUM_THREADS='2',
        OMP_THREAD_LIMIT='2',OMP_WAIT_POLICY='PASSIVE',GOMP_SPINCOUNT='0',OPENBLAS_NUM_THREADS='1')
    record['architecture']='pc_persistent_fusion_edge_local_window_v1'
    record['localization_sha256']={name:digest(ROOT/'edge_localization_build'/name) for name in ('map_optimization','laserOdometry')}
    record['input_max_age']=a.input_max_age;env['LIUJING_FUSION_MAX_INPUT_AGE']=str(a.input_max_age)
    record['cpu_layout']=a.cpu_layout
    if a.cpu_layout=='fusion-core':env['LIUJING_FUSION_CORE']='3'
    record['fusion_backend']=a.fusion_backend;record['fusion_hz']=a.fusion_hz;record['capture_frames']=a.capture_frames
    input_policy=a.input_policy or ('latest_eligible' if a.fusion_reference in ('pipeline','dataflow','direct') else 'source_cadence')
    env['LIUJING_FUSION_INPUT_POLICY']=input_policy;record['input_policy']=input_policy
    record['fusion_reference']=a.fusion_reference;env['LIUJING_FUSION_CELL_OVERFLOW']='1' if a.fusion_reference in ('overflow','variable','cached','pipeline','dataflow','direct') else '0'
    record['association_threads']=a.association_threads;env['LIUJING_FUSION_CPU_THREADS']=str(a.association_threads)
    record['cpu_association']=a.cpu_association;env['LIUJING_FUSION_QUERY_SIMD']='1' if a.cpu_association=='query' else '0'
    record['fast_prepare']=a.fast_prepare;env['LIUJING_FUSION_FAST_PREPARE']='1' if a.fast_prepare else '0'
    record['fast_tiles']=a.fast_tiles;env['LIUJING_FUSION_FAST_TILES']='1' if a.fast_tiles else '0'
    record['preprocess_demand']=a.preprocess_demand;env['LIUJING_FUSION_PREPROCESS_DEMAND']='1' if a.preprocess_demand else '0'
    env['LIUJING_FUSION_HZ']=str(a.fusion_hz)
    env['LIUJING_CAPTURE_FRAMES']=str(a.capture_frames)
    env['LIUJING_RENDER_HZ']=str(a.render_hz)
    env['ROS_LOG_DIR']=str(run/'ros_logs')
    if a.demo_bridge:
        export_dir=Path('/dev/shm')/('liujing_demo_'+run.name);export_dir.mkdir()
        env['LIUJING_DISPLAY_EXPORT']=str(export_dir)
    children=[];running=True;failure=None;dma_buffers=[]
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def start(name,cmd):
        f=open(run/(name+'.log'),'a');cmd=(['taskset','-c','0-2']+cmd) if a.cpu_layout=='fusion-core' else cmd;child=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,pass_fds=tuple(b.native_export_fd for b in dma_buffers) if name=='visual' else ())
        children.append((name,child,f));return child
    try:
        import pynq.pl_server.embedded_device as ed
        ed.EmbeddedDevice._xrt_download=lambda self,data:None
        from pynq import Overlay
        overlay=Overlay(str(bit))
        # PYNQ skips global state generation when parsing cached HWH metadata.
        # After a cold boot that file can be absent even after a real download.
        # Persist this manager's just-loaded overlay so attach-only children can
        # verify it; never use this path to adopt an unknown existing bitstream.
        from pynq.pl_server.global_state import GlobalState,save_global_state
        if Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()!='operating':
            raise RuntimeError('FPGA download did not reach operating state')
        loaded_state=GlobalState(bitfile_name=str(bit),active_name=overlay.device.name,
            timestamp=overlay.timestamp,psddr=overlay.mem_dict.get('PSDDR',{}))
        for ip_name,ip in overlay.ip_dict.items():
            if ip.get('type') in ('xilinx.com:ip:pr_axi_shutdown_manager:1.0','xilinx.com:ip:dfx_axi_shutdown_manager:1.0'):
                loaded_state.add(ip_name,ip['phys_addr'])
        save_global_state(loaded_state)
        from pynq import Clocks
        if abs(Clocks.fclk0_mhz-100)>.1:raise RuntimeError('PL clock differs from 100 MHz contract')
        if 'fpga_feature_extract_0' not in overlay.ip_dict:raise RuntimeError('C16 IP missing')
        if a.fusion_backend=='pl' and 'lj_associate_0' not in overlay.ip_dict:raise RuntimeError('fusion association IP missing')
        if a.fusion_backend=='pl':
            from fusion_allocation import allocate_fusion
            dma_buffers,allocation=allocate_fusion(overlay,run,bit,expected_contract)
            env['LIUJING_FUSION_PL']=str(allocation)
        # Manager has finished configuration; renderer takes its own exclusive lease.
        import hardware_ownership
        hardware_ownership._reconfiguration_lease.close();hardware_ownership._reconfiguration_lease=None
        # The legacy monitor saves trajectories/maps automatically; not used by this product entry.
        cfg=str(LOSAM/'c16.yaml');exe=LOSAM/'runtime/software/install/lib/lo_sam'
        for name in ('feature_extract','pldataprocess','laserOdometry','map_optimization'):
            cmd=[str(ROOT/'edge_localization_build'/name) if name in ('laserOdometry','map_optimization') else str(exe/name),'--ros-args','-r','__node:='+name,'--params-file',cfg]
            if name=='map_optimization':cmd+=['-p','savePCD:=false']
            if a.replay:cmd+=['-p','use_sim_time:=true']
            start(name,cmd)
        cmd=[PYTHON,str(ROOT/'python/c16_attached.py')]
        if a.replay:cmd+=['--ros-args','-p','use_sim_time:=true']
        start('c16_pl',cmd)
        if not a.lidar_only:
            cmd=[PYTHON,str(ROOT/'python/live.py'),'--calibration',str(calibration),'--out',str(run/'visual'),
                 '--session-mode',a.mode,'--backend',a.backend,'--numeric','qrcp_v1','--bit',str(bit),'--port',str(a.port),
                 '--pose-topic',pose_topic,'--pose-wait-seconds','2' if a.pose_source=='mapping' else '0.5',
                 '--static-warmup-seconds',str(a.static_warmup_seconds)]
            if a.replay:cmd+=['--use-sim-time']
            if a.observe_static_drift:cmd+=['--observe-static-drift']
            if a.engine=='native-v2':cmd=[str(Path(a.native_build)/'liujing_native'),str(calibration),str(run/'visual'),a.mode,str(a.port)]
            start('visual',cmd)
        if a.demo_bridge:
            start('demo_bridge',[PYTHON,str(ROOT/'demo/board_bridge.py'),'--export-dir',str(export_dir),'--calibration',str(calibration),'--out',str(run/'demo')])
        qos=str(ROOT/'configs/record_qos.yaml')
        # Allow subscribers and both hardware clients to initialise before input starts.
        until=time.monotonic()+90
        ready_files=[run/'c16.ready']+([] if a.lidar_only else [run/'visual.ready'])
        while running and time.monotonic()<until:
            if any(c.poll() is not None for _,c,_ in children):raise RuntimeError('startup child exited')
            if all(path.exists() for path in ready_files):break
            time.sleep(.2)
        if not running:return
        if not all(path.exists() for path in ready_files):raise RuntimeError('hardware client readiness timeout')
        if a.replay:
            import sqlite3
            topics=set()
            for db in Path(a.replay).glob('*.db3'):
                with sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True) as connection:topics.update(x[0] for x in connection.execute('select name from topics'))
            inputs=list(INPUT_TOPICS)
            if '/camera/color/image_raw' in topics:inputs[1]='/camera/color/image_raw'
            elif not a.lidar_only:
                if inputs[1] not in topics:raise ValueError('no recorded color input')
                start('color_decoder',[PYTHON,str(ROOT/'python/replay_color.py'),'--ros-args','-p','use_sim_time:=true']);time.sleep(2)
            start('replay',['ros2','bag','play',a.replay,'--clock','--rate',str(a.rate),'--qos-profile-overrides-path',qos,'--topics',*inputs])
        else:
            start('lidar',['/home/ubuntu/sensor_bringup_20261021/lidar_ws/install/lslidar_driver/lib/lslidar_driver/lslidar_driver_node','--ros-args','--params-file','/home/ubuntu/sensor_bringup_20261021/c16.yaml'])
            if not a.lidar_only and a.engine=='legacy-v5':start('camera',[str(ROOT/'sensors_build/liujing_camera')])
        begin=time.monotonic();record['status']='running';atomic(state,record)
        with open(run/'resources.jsonl','w',buffering=1) as resources:
            while running and (a.seconds==0 or time.monotonic()-begin<a.seconds):
                if Path('/run/lock/liujing_raster_fault.json').exists():
                    failure='PL fault latched; reboot required'
                    # Let the native fault status reach the browser, then stop
                    # this session's producers before retaining its CMA buffers.
                    time.sleep(2)
                    break
                ended=[(n,c.returncode) for n,c,_ in children if c.poll() is not None]
                if ended:
                    if ended==[('replay',0)]:time.sleep(2);break
                    raise RuntimeError('required process exited: '+repr(ended))
                free=shutil.disk_usage(ROOT).free
                if free<128*1024**2:raise RuntimeError('less than 128 MiB free for metrics')
                process_ticks={}
                for name,child,_ in children:
                    try:
                        fields=Path(f'/proc/{child.pid}/stat').read_text().rsplit(')',1)[1].split()
                        process_ticks[name]={'pid':child.pid,'user_ticks':int(fields[11]),'system_ticks':int(fields[12]),'rss_pages':int(fields[21])}
                    except FileNotFoundError:pass
                resources.seek(0);resources.truncate();resources.write(json.dumps({'process_ticks':process_ticks,'clock_ticks_per_second':os.sysconf('SC_CLK_TCK'),'mono':time.monotonic(),'free_bytes':free,'load':os.getloadavg(),
                    'cpu':Path('/proc/stat').read_text().splitlines()[0],'meminfo':Path('/proc/meminfo').read_text(),
                    'temperature':[x.read_text().strip() for x in Path('/sys/class/hwmon').glob('hwmon*/temp*_input')],
                    'temperature_paths':[str(x) for x in Path('/sys/class/hwmon').glob('hwmon*/temp*_input')]})+'\n')
                for _,_,logfile in children:
                    if os.fstat(logfile.fileno()).st_size>8*1024**2:logfile.truncate(0)
                time.sleep(1)
    except Exception as exc:
        failure=repr(exc)
    finally:
        # Stop producers first; only signal process groups owned by this manager.
        order={'demo_bridge':-1,'lidar':0,'camera':0,'replay':0,'visual':1,'c16_pl':2,'monitor':4}
        for name,child,f in sorted(children,key=lambda row:order.get(row[0],3)):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            record[name+'_exit']=child.returncode;f.close()
        if a.demo_bridge:
            for filename in ('map.ljmp','map.ljmp.tmp','camera.jpg','camera.jpg.tmp'):
                (export_dir/filename).unlink(missing_ok=True)
            try:export_dir.rmdir()
            except OSError:pass
        dma_active=False
        if dma_buffers:
            # An abnormal native process exit can leave an outstanding DMA job.
            # Never free the manager-owned physical buffers in that state.
            dma_active=not (overlay.lj_associate_0.read(0)&4)
            if dma_active:
                from hardware_fault import latch_timeout
                try:latch_timeout({'reason':'native association exited while AP_IDLE was false','run':str(run)})
                except OSError as exc:record['fault_latch_error']=repr(exc)
                failure=failure or 'Association DMA still active; reboot required'
        completion=run/'visual/completion.json'
        if a.engine=='native-v2' and not a.lidar_only:
            metrics=run/'visual/metrics.jsonl'
            if metrics.exists():
                final=json.loads(metrics.read_text().splitlines()[-1]);record['native_final']=final
                if final.get('failure'):failure=failure or final['failure']
                if int(final.get('edge_batches',0))==0:failure=failure or 'No edge observation batch'
            else:failure=failure or 'native metrics missing'
        elif completion.exists():
            record['visual_completion']=json.loads(completion.read_text())
            if record['visual_completion'].get('failure'):failure=failure or record['visual_completion']['failure']
            if not record['visual_completion']['counts'].get('fused') or not record['visual_completion']['counts'].get('rendered'):
                failure=failure or 'No fused and rendered online frame; data chain not completed'
        elif not a.lidar_only:failure=failure or 'visual completion missing'
        record.update(status='failed' if failure else 'stopped',error=failure)
        atomic(run/'manifest.json',record);atomic(state,record);print(json.dumps(record,indent=2),flush=True)
        if dma_buffers and (dma_active or Path('/run/lock/liujing_raster_fault.json').exists()):
            record['status']='fault_pinned_until_reboot';atomic(state,record)
            print('PL fault: allocator retains CMA until reboot; no reload or CPU fallback',flush=True)
            while True:time.sleep(1)
    if failure:raise SystemExit(1)


if __name__=='__main__':main()
