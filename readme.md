copy from docker build to local:

```bash
docker buildx build --output type=local,dest=. .
```

rosservice call /ros_set_chassis_enable_cmd_srv "ros_set_chassis_enable_cmd: false"

rosrun gmapping slam_gmapping scan:=scan base_frame:=base_link odom_frame:=odom

eduroam ip
wlx8416f916926c: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 84:16:f9:16:92:6c brd ff:ff:ff:ff:ff:ff
    inet 172.31.14.255/16