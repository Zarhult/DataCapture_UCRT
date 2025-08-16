#!/bin/bash
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.1.21 timestamp_mode:=TIME_FROM_SYNC_PULSE_IN lidar_port:=7502 imu_port:=7503 udp_dest_auto:=true metadata:=/tmp/metadata.json use_system_default_qos:=true

