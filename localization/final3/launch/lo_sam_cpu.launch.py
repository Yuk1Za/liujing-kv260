import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """CPU 干跑模式：用 cpu_feature_extract 替代 pynq_feature_extraction.py +
    PLdataprocess（直接发布最终特征话题）。用于 x86 验证与板端 PS-only 基线。

    KITTI 输入单独起（发布 /velodyne_points）：
      python3 kitti_pointcloud_publisher.py <velodyne_data_dir> <timestamps.txt> 10
    """
    lo_sam_share = get_package_share_directory('lo_sam')

    lidar_config = os.path.join(lo_sam_share, 'config', 'VLP16.yaml')
    rviz_config = os.path.join(lo_sam_share, 'rviz_cfg', 'LO-SAM.rviz')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
                                     description='Whether to launch RViz')

    nodes = [
        # 输入：重排节点（与 FPGA 模式共用）
        Node(package='lo_sam', executable='feature_extract', name='feature_extract',
             output='screen', parameters=[lidar_config]),
        # CPU 特征提取（本次的核心替换）
        Node(package='lo_sam', executable='cpu_feature_extract', name='cpu_feature_extract',
             output='screen', parameters=[lidar_config, {
                 'lidar_height': 2.0,          # KITTI 值；C16 实测后修改
                 'scan_lower_bound_deg': -15.0,
                 'scan_resolution_deg': 2.0,
             }]),
        Node(package='lo_sam', executable='laserOdometry', name='laserOdometry',
             output='screen'),
        Node(package='lo_sam', executable='map_optimization', name='map_optimization',
             output='screen'),
    ]

    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz',
                     arguments=['-d', rviz_config], output='screen')

    return LaunchDescription([
        rviz_arg,
        *nodes,
        GroupAction([rviz_node], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
