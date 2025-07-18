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

启动Nav Drone Control
```bash
cd ~/Nav_Drones
source install/local_setup.bash
ros2 launch nav_drone_control nav_drone_control_sim.launch.py
```

## 实机启动顺序
```bash
cd ~/Nav_Drones
source install/local_setup.bash
ros2 launch nav_drone_control nav_drone_control_real.launch.py

启动Micro XRCE Agent服务
```bash
# To run the Micro XRCE Agent for UDP communication
sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```
启动Nav Drone Control
```bash
cd ~/Nav_Drones/src/nav_drone_control
source install/local_setup.bash
