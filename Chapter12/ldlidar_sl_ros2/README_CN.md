# 操作指南

>此SDK仅适用于深圳乐动机器人有限公司销售的激光雷达产品，产品型号为:
> - LDROBOT LiDAR LD14
> - LDROBOT LiDAR LD14P

## 0. 获取激光雷达的ROS2功能包
```bash
cd ~

mkdir -p ldlidar_ros2_ws/src

cd ldlidar_ros2_ws/src

git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
```
## 1. 系统设置
- 第一步，通过板载串口或者USB转串口模块(例如,cp2102模块)的方式使雷达连接到你的系统主板.
- 第二步，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyUSB0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
cd ~/ldlidar_ros2_ws

sudo chmod 777 /dev/ttyUSB0
```
- 第三步，修改`launch/`目录下雷达产品型号对应的lanuch文件中的`port_name`值，以ld14.launch.py为例，如下所示.

```py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar_sl_ros2',
      executable='ldlidar_sl_ros2_node',
      name='ldlidar_publisher_ld14',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate' : 115200},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld14',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld
```
## 2. 编译方法

使用colcon编译.

```bash
cd ~/ldlidar_ros2_ws

colcon build
```
## 3. 运行方法
### 3.1. 设置功能包环境变量

- 编译完成后需要将编译生成的相关文件加入环境变量，便于 ROS 环境可以识别， 执行命令如下所示， 该命令是临时给终端加入环境变量，意味着您如果重新打开新的终端，也需要重新执行如下命令.

  ```bash
  cd ~/ldlidar_ros2_ws
  
  source install/local_setup.bash
  ```
- 为了重新打开终端后，永久不用执行上述添加环境变量的命令，可以进行如下操作.

  ```bash
  echo "source ~/ldlidar_ros2_ws/install/local_setup.bash" >> ~/.bashrc
  
  source ~/.bashrc
  ```
### 3.2. 启动激光雷达节点

- 产品型号为 LDROBOT LiDAR LD14
  - 启动ld14 lidar node:
  ``` bash
  ros2 launch ldlidar_sl_ros2 ld14.launch.py
  ```
  - 启动ld14 lidar node并显示激光数据在Rviz2上:
  ``` bash
  ros2 launch ldlidar_sl_ros2 viewer_ld14.launch.py
  ```

- 产品型号为 LDROBOT LiDAR LD14P
  - 启动ld14p lidar node:
  ``` bash
  ros2 launch ldlidar_sl_ros2 ld14p.launch.py
  ```
  - 启动ld14p lidar node并显示激光数据在Rviz2上:
  ``` bash
  ros2 launch ldlidar_sl_ros2 viewer_ld14p.launch.py
  ```
##   4. 测试

> 代码支持ubuntu20.04 ROS2 foxy版本及以上测试，使用rviz2可视化。
- 新打开一个终端 (Ctrl + Alt + T),运行命令:`rviz2`,并通过Rviz2工具打开readme文件所在目录的rviz2文件夹下面的ldlidar.rviz文件
```bash
rviz2
```