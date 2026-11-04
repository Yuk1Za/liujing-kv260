#!/usr/bin/env python3
"""统一管理 final3 硬件、雷达视觉定位与 200 万点地图进程。"""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument("--seconds",type=int,default=600)
p.add_argument("--out",type=Path,default=ROOT/"output/session")
p.add_argument("--calibration",type=Path,default=ROOT/"mapping/liujing/configs/live_candidate_20261026.json")
p.add_argument("--pynq-python",default="/usr/local/share/pynq-venv/bin/python3")
a=p.parse_args()
a.out.mkdir(parents=True,exist_ok=True)
bit=ROOT/"hardware/final3/kv260_final3.bit"
env=os.environ.copy()
env.update(ROS_DOMAIN_ID="42",LOSAM_BIT=str(bit),PL_DOWNLOAD="0",PL_CLEAR_CACHE="0",VIS_FACTOR="1",LIUJING_PUBLISH_COLOR="1",LIUJING_FUSION_CELL_OVERFLOW="1",LIUJING_FUSION_QUERY_SIMD="1",LIUJING_FUSION_FAST_TILES="1",LIUJING_FUSION_CPU_THREADS="2",LIUJING_FUSION_HZ="15")
env.pop("LIUJING_FUSION_PL",None)
children=[]
running=True

def stop(*_):
    global running
    running=False
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)
def start(name,command):
    log=(a.out/(name+".log")).open("wb")
    child=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append((name,child,log))

with open("/run/lock/liujing-final3.lock","a") as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    subprocess.run([a.pynq_python,"-c","from pynq import Overlay; import sys; Overlay(sys.argv[1])",str(bit)],check=True,env=env)
    try:
        start("feature",["ros2","run","lo_sam","feature_extract","--ros-args","--params-file",str(ROOT/"localization/final3/config/VLP16.yaml"),"-p","lidar_topic:=/velodyne_points"])
        start("radar_pl",["ros2","run","lo_sam","pynq_feature_extraction.py"])
        start("feature_unpack",["ros2","run","lo_sam","pldataprocess"])
        start("odometry",["ros2","run","lo_sam","laserOdometry"])
        start("backend",["ros2","run","lo_sam","map_optimization"])
        start("vision",[a.pynq_python,str(ROOT/"integration/visual_node.py"),"--bit",str(bit),"--calibration",str(a.calibration)])
        start("fusion",[str(ROOT/"mapping/liujing/native_fusion_overflow_build/liujing_native"),str(a.calibration),str(a.out/"map"),"static_diagnostic","8090"])
        start("lidar_adapter",["python3",str(ROOT/"localization/final3/scripts/c16_to_velodyne.py"),"--in","/point_cloud_raw","--out","/velodyne_points"])
        start("lidar",["ros2","run","lslidar_driver","lslidar_driver_node","--ros-args","--params-file",str(ROOT/"sensors/lidar/c16.yaml")])
        end=time.monotonic()+a.seconds
        while running and time.monotonic()<end:
            failed=[(name,c.returncode) for name,c,_ in children if c.poll() is not None]
            if failed:raise RuntimeError("module exited: "+repr(failed))
            time.sleep(.5)
    finally:
        for name,child,log in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            log.close()
