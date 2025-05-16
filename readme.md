Overlay package needed for Segway RMP220 control.

Part of the CPS Segway RMP220 ROS2 meta-package:
- ros2_rmp
- ros2_rmp_middleware
- ros2_rmp_teleop
- ros2_rmp_support
 
####
copy from docker build to local:

```bash
docker buildx build --output type=local,dest=. .
```

rosservice call /ros_set_chassis_enable_cmd_srv "ros_set_chassis_enable_cmd: false"

rosrun gmapping slam_gmapping scan:=scan base_frame:=base_link odom_frame:=odom
