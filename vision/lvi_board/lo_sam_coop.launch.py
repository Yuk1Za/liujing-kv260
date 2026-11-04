import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

KITTI_VELO = ('/home/ubuntu/FPT_Competition/LO-SAM-ROS2/'
              '2011_09_30_drive_0027_sync/2011_09_30/'
              '2011_09_30_drive_0027_sync/velodyne_points')


def generate_launch_description():
    lo_sam_share = get_package_share_directory('lo_sam')
    lidar_config = os.path.join(lo_sam_share, 'config', 'VLP16.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='KITTI replay rate in Hz'),
        DeclareLaunchArgument('kitti_dir', default_value=KITTI_VELO + '/data'),
        DeclareLaunchArgument('timestamps', default_value=KITTI_VELO + '/timestamps.txt'),

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

        # ★ 去掉自带 publisher：共存测试要由 lvi_board/kitti_pub.py 同时发
        #   /velodyne_points（全量，给 lo_sam）与 /velodyne_points_vis（抽稀，给视觉），
        #   并且它把帧号写进 header.frame_id='velodyne:<idx>'（视觉侧靠这个定位图像）。
    ])
