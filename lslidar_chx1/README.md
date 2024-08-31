### LSLIDAR_CHX1_ROS驱动说明



### 原始版本：

* LSLIDAR_CHX1_V1.1.9_221107_ROS


### 适用范围：

* 适用于镭神ch128x1、ch128s1、cx128s2、cx126s3、ch16x1、ch64w、cb64s1_a、cx6s3、ch256、cx1s3的雷达



## 依赖：

1.ros

**Ubuntu 18.04**: ros-melodic-desktop-full

**Ubuntu 20.04**: ros-noetic-desktop-full

**Installation**: please refer to [http://wiki.ros.org](http://wiki.ros.org/)

2.ros 依赖

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.其他依赖

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #Select the appropriate version
~~~

## 

### 编译与运行：

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
把驱动压缩包拷贝到src目录下，并解压
cd ~/lslidar_ws
catkin_make
source devel/setup.bash
roslaunch lslidar_driver lslidar_ch128x1.launch   
~~~



### launch 文件参数说明：

ch128x1

~~~xml
<launch>
  <arg name="device_ip" default="192.168.1.200" />  //雷达ip
  <arg name="msop_port" default="2368"/>   //数据包目的端口
  <arg name="difop_port" default="2369"/>   //设备包目的端口
   <arg name="lidar_type" default="ch128x1"/>  //  ch16x1,ch128x1,ch128s1，cx126s3,ch64w 等可选
  <arg name="pcl_type" default="false" />   //点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
 <arg name="use_time_service" default="false" />  //雷达是否使用gps或ptp授时，使用改为true
    
   
   <node pkg="lslidar_driver" type="lslidar_ch_driver_node" name="lslidar_driver_node" output="screen">
    <!-- param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/-->//取消注释（删除!-- --），启用离线pcap模式
    <param name="lidar_ip" value="$(arg device_ip)"/>
     <param name="lidar_type" value="$(arg lidar_type)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="add_multicast" value="false"/> // 是否开启组播模式。
    <param name="group_ip" value="224.1.1.2"/> //组播ip地址
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="frame_id" value="laser_link"/> //坐标系id
    <param name="pcl_type" value="$(arg pcl_type)"/>    //点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
    <param name="min_range" value="0.15"/> //单位，米。雷达盲区最小值，小于此值的点被过滤
    <param name="max_range" value="500.0"/> //单位，米。雷达盲区最大值 ，大于此值的点被过滤
    <param name="angle_disable_min" value="0"/> //雷达裁剪角度开始值 ，单位0.01°(填整数)
    <param name="angle_disable_max" value="0"/>  //雷达裁剪角度结束值，单位0.01°
    <Param name="pointcloud_topic" value="lslidar_point_cloud"/>  //点云话题名称，可修改
    <param name="horizontal_angle_resolution" value="0.12"/>  //雷达水平角度分辨率
    <param name="packet_rate" value="11228.0"/>  //数据包每秒发包数
    <param name="channel_num" value="16"/> // laserscan线号
    <param name="echo_num" value="0"/>  // 仅双回波模式下有效，0表示发布所有点云，1表示发布第一次回波点云，2表示发布第二次回波点云
    <param name="publish_laserscan" value="false"/> //是否发布laserscan话题，发布改为true
  </node>
 
 <!--node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 1 0 0 0 world laser_link 100" /-->  //取消注释（删除!-- --）,静态坐标系转换 


~~~

### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~xml
      <param name="add_multicast" value="true"/> 
      <param name="group_ip" value="224.1.1.2"/>  //上位机设置的组播ip地址
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~





### 离线pcap模式：

- 把录制好的pcap文件，拷贝到lslidar_driver/pcap文件夹下

- 修改launch文件以下参数

  ~~~xml
     //取消注释
   	 <param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap">  // xxx.pcap改为拷贝的pcap文件名
  ~~~



###  pcl点云类型：

- 修改launch文件以下参数

  ~~~xml
    <arg name="pcl_type" default="false" />   //点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
  ~~~

  

- 默认false为自定义点云类型，定义参考lslidar_driver/include/lslidar_ch_driver/lslidar_ch_driver.h头文件

- 改为true,为pcl自带类型 :

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~



## FAQ

Bug Report

version :LSLIDAR_CHX1_V1.1.9_221107_ROS

Modify:  original version

Date    : 2022-11-07

----



update version:LSLIDAR_CHX1_V1.2.1_230316_ROS

Modify:  

降低cpu占用；点的时间改为相对时间；新增兼容cx128s2 雷达。

Date    : 2023-03-16

----



update version:LSLIDAR_CHX1_V1.2.2_230414_ROS

Modify:  

新增兼容cx126s3 雷达。

Date    : 2023-04-14

----

update version:LSLIDAR_CHX1_V1.2.3_230704_ROS

Modify:  

新增兼容cb64s1_a雷达。

Date    : 2023-07-04



----

update version:LSLIDAR_CHX1_V1.2.4_230901_ROS

Modify:  

新增兼容cx6s3、ch256、cx1s3雷达。

Date    : 2023-09-01
