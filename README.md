# 导航无人机控制程序


## 仿真程序启动顺序
启动Micro XRCE Agent服务
```bash
# To run the Micro XRCE Agent for UDP communication
MicroXRCEAgent udp4 -p 8888
```
启动Gazebo仿真环境
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

测试通信
```bash
cd ~/Nav_Drones
source install/local_setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

启动Nav Drone Control
```bash
cd ~/Nav_Drones
source install/local_setup.bash
ros2 launch nav_drone_control nav_drone_control_sim.launch.py
```

## 实机启动顺序

启动Micro XRCE Agent Serial服务
```bash
# To run the Micro XRCE Agent for UDP communication
sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```

测试通信
```bash
cd ~/Nav_Drones
source install/local_setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

启动Nav Drone Control
```bash
cd ~/Nav_Drones
source install/local_setup.bash
ros2 launch nav_drone_control nav_drone_control_real.launch.py
```
