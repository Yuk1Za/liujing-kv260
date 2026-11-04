import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

KITTI_VELO = ('/home/ubuntu/FPT_Competition/LO-SAM-ROS2/'
              '2011_09_30_drive_0027_sync/2011_09_30/'
              '2011_09_30_drive_0027_sync/velodyne_points')

# ---- 视觉分支参数 ----
# 复用合并 bit 里的 fast_accel_0 / slk_accel_0；与雷达 PL 节点用【同一份】bit/hwh 文件，
# 保证 PYNQ 解析出的寄存器映射一致（视觉节点以 download=False attach，不重配 PL）。
PL_BIT = ('/home/ubuntu/ROS-PYNQ-LIO-SAM/software/install/share/lo_sam/'
          'bitstreams/feature_extraction.bit')
PYNQ_VENV_PY = '/usr/local/share/pynq-venv/bin/python3'
VISION_PY = '/home/ubuntu/pl_vision/pl_vision_node.py'
BRANCH_PY = '/home/ubuntu/pl_vision/pl_vision_branch.py'
IMG_DIR = '/home/ubuntu/kitti_cam/image_02/data'
IMG_TS = '/home/ubuntu/kitti_cam/image_02/timestamps.txt'
VELO_TS = KITTI_VELO + '/timestamps.txt'
VISION_READY_DELAY = 14.0   # 秒：等雷达 PL 节点先把 bit 加载好，视觉再 attach
BRANCH_READY_DELAY = 13.0   # 秒：下游是纯订阅者，比视觉早一点起即可


def generate_launch_description():
    lo_sam_share = get_package_share_directory('lo_sam')
    lidar_config = os.path.join(lo_sam_share, 'config', 'VLP16.yaml')

    # 视觉节拍（vision_sync 默认 /velodyne_points）：
    #   非空 -> 订阅该 topic，每收到一条雷达点云就取 header.stamp（KITTI 原始时间戳），
    #           据此定位【同一帧】的相机图再跑视觉；视觉自己不产生节拍。
    #   留空 -> 退回旧的定时器模式（视觉自走 rate Hz，与雷达各跑各的）。
    vision_sync = LaunchConfiguration('vision_sync')
    align = LaunchConfiguration('vision_align')
    fx = LaunchConfiguration('vision_fx')
    yaw_sign = LaunchConfiguration('vision_yaw_sign')

    # 用 ExecuteProcess 而非 Node：视觉节点需要 pynq-venv 的 python
    # （系统 python3 没有 pynq），且不在 colcon 包里。
    vision_branch = ExecuteProcess(
        name='pl_vision',
        output='screen',
        cmd=[
            PYNQ_VENV_PY, VISION_PY,
            '--ros-args',
            '-p', 'bit:=' + PL_BIT,
            '-p', 'image_dir:=' + IMG_DIR,
            '-p', 'timestamps:=' + IMG_TS,
            '-p', 'velo_timestamps:=' + VELO_TS,
            '-p', 'download:=false',
            '-p', 'detector:=st',        # 阶段 5：响应图语义（阈值→排序→MIN_DIST→前 N）
            '-p', 'threshold:=20',        # ST 的 λ_min 绝对阈值（不是 cv2 的 qualityLevel 比例）
            '-p', 'min_dist:=20',        # 同 cv2 goodFeaturesToTrack 的 minDistance
            '-p', 'max_corners:=100',
            '-p', 'lk_prior:=affine',   # 稀疏 LK 迭代初值：上一帧流场仿射拟合（急转弯 3.13->0.51 px）
            # --- 时间戳对齐 ---
            '-p', ['sync_topic:=', vision_sync],
            '-p', ['align:=', align],
            '-p', ['rate:=', LaunchConfiguration('rate')],   # 仅定时器模式使用
        ],
    )

    # 视觉分支的下游消费者：把光流估成 yaw、发布 pl_vision/vo，
    # 并与 radar 的 odom_init/laser_odom 逐帧配对发布 pl_vision/vo_vs_lidar。
    # 不需要 pynq -> 直接用系统 python3（有 rclpy）。
    vision_consumer = ExecuteProcess(
        name='pl_vision_branch',
        output='screen',
        cmd=[
            'python3', BRANCH_PY,
            '--ros-args',
            '-p', 'flow_topic:=pl_vision/flow',
            '-p', 'sync_topic:=pl_vision/sync',
            '-p', 'lidar_odom_topic:=odom_init/laser_odom',
            '-p', ['fx:=', fx],
            '-p', ['yaw_sign:=', yaw_sign],
            '-p', 'min_valid:=20',
            '-p', 'sat_px:=7.9',      # 截断值实测 8.0000，取 7.9 把紧贴硬顶的 band 也算进去
            '-p', ['sat_gate:=', LaunchConfiguration('vision_sat_gate')],  # 饱和占比超它判废；0=关闭
            '-p', 'med_gate:=0.90',   # |median(u)| > 0.9×sat_px 也判废（未饱和但贴边界）
            '-p', 'log_every:=25',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='KITTI replay rate in Hz (雷达节拍频率)'),
        DeclareLaunchArgument('kitti_dir', default_value=KITTI_VELO + '/data'),
        DeclareLaunchArgument('timestamps', default_value=KITTI_VELO + '/timestamps.txt'),
        DeclareLaunchArgument('with_vision', default_value='true',
                              description='是否启用视觉分支'),
        DeclareLaunchArgument('vision_sync', default_value='/velodyne_points',
                              description='视觉节拍来源 topic；留空则退回定时器模式'),
        DeclareLaunchArgument('vision_align', default_value='stamp',
                              description='stamp = 按时间戳最近邻取同帧；index = 按消息序号取同序号'),
        DeclareLaunchArgument('vision_fx', default_value='371.8',
                              description='640 宽图像下的等效焦距（像素）'),
        DeclareLaunchArgument('vision_yaw_sign', default_value='-1.0',
                              description='光流->yaw 的符号（由实测与雷达航向比对确定）'),
        DeclareLaunchArgument('vision_sat_gate', default_value='0.40',
                              description='饱和点占比超过它就判废该帧、不出 yaw；0 = 关闭门控'),

        # ===== 雷达管线（与原 lo_sam_demo.launch.py 一致）=====
        Node(package='lo_sam', executable='feature_extract', name='feature_extract',
             output='screen', parameters=[lidar_config]),

        Node(package='lo_sam', executable='pynq_feature_extraction.py',
             name='pynq_feature_extraction', output='screen'),

        Node(package='lo_sam', executable='pldataprocess', name='pldataprocess',
             output='screen'),

        Node(package='lo_sam', executable='laserOdometry', name='laserOdometry',
             output='screen'),

        Node(package='lo_sam', executable='map_optimization', name='map_optimization',
             output='screen',
             parameters=[{
                 'loopClosureEnableFlag': False,
                 'mappingSurfLeafSize': 0.8,
                 'mappingCornerLeafSize': 0.4,
                 'surroundingKeyframeSize': 50,
                 'surroundingKeyframeSearchRadius': 50,
                 'mappingProcessInterval': 0.15,
                 'savePCD': False,
             }]),

        Node(package='lo_sam', executable='kitti_pointcloud_publisher.py',
             name='kitti_pointcloud_publisher', output='screen',
             arguments=[LaunchConfiguration('kitti_dir'),
                        LaunchConfiguration('timestamps'),
                        LaunchConfiguration('rate')]),

        # ===== 视觉分支：下游消费者 + PL 视觉节点（都延迟启动）=====
        TimerAction(period=BRANCH_READY_DELAY, actions=[vision_consumer]),
        TimerAction(period=VISION_READY_DELAY, actions=[vision_branch]),
    ])
