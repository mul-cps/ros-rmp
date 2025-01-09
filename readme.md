copy from docker build to local:

```bash
docker buildx build --output type=local,dest=. .
```

rosservice call /ros_set_chassis_enable_cmd_srv "ros_set_chassis_enable_cmd: false"