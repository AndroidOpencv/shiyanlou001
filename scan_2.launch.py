#!/usr/bin/env python
# coding=utf-8

from launch import LaunchDescription
from launch_ros.actions import Node

# ros2 launch hins_le_ros2 hins_le_launch.py
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hins_le_ros2",
            executable="hins_le_ros2_node",
            name="scan_2_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "laser_2",
                 "topic_name": "scan_2",
                 "change_param":True,
                 "laser_ip": "192.168.10.66",			# 传感器ip地址		
                 "laser_port": 8080,				# 传感器端口（固定为8080）
                 "measure_frequency_kHz": "200",		# 雷达测量频率
                 "motor_speed": "20",				# 雷达转速
                 "point_sampling": "1",				# 重复采样点数
                 "filter_level": "1",				# 噪声过滤等级
                 "shadows_filter_level": 1,			# 防拖尾过滤等级
                 "shadows_filter_max_angle": 175.0,		# 见readme
                 "shadows_filter_min_angle": 5.0,		# 见readme
                 "shadows_filter_neighbors": 1,		        # 见readme
                 "shadows_filter_window": 5,			# 见readme
                 "shadows_traverse_step": 1,			# 见readme
                 "min_angle": 0.0,				# 传感器测量角度的最小值（角度过滤-135.0）
                 "max_angle": 360.0,				# 传感器测量角度的最大值（角度过滤135.0）
                 "range_min": 0.70,
                 "use_udp": True
                 }				
            ]
        )
    ])
