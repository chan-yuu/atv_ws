## LSLIDAR_CHX1_ROS

## 1.Introduction

​		LSLIDAR_CHX1_ROS is the lidar ros driver in linux environment, which is suitable for ch128x1、ch128s1、cx128s2、ch16x1、ch64w 、cb64s1_a 、cx6s3、cx128s2 、ch256、cx1s3 lidar. The program has  tested under ubuntu18.04 ros melodic , ubuntu 20.04 ros noetic.

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 18.04**: ros-melodic-desktop-full

**Ubuntu 20.04**: ros-noetic-desktop-full

**Installation**: please refer to [http://wiki.ros.org](http://wiki.ros.org/)

2.ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.other dependencies

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #Select the appropriate version
~~~

## 3.Compile & Run

### 3.1 Compile

~~~bash
mkdir -p ~/lidar_ws/src
~~~

Copy the whole lidar ROS driver directory into ROS workspace, i.e "~/lidar_ws/src".

~~~bash
cd ~/lidar_ws
catkin_make
source devel/setup.bash
~~~

### 3.2 Run

run with single  lidar:

~~~bash
roslaunch lslidar_driver lslidar_ch128x1.launch
~~~

run with double  lidar:

~~~bash
roslaunch lslidar_driver lslidar_double.launch
~~~



## 4. Introduction to parameters

~~~bash
<launch>
    <arg name="device_ip" default="192.168.1.200" />    #lidar ip
    <arg name="msop_port" default="2368" />   #Main data Stream Output Protocol packet port
    <arg name="difop_port" default="2369" />  #Device Information Output Protocol packet port
    <arg name="lidar_type" default="ch128x1"/>       # lidar type
    <arg name="pcl_type" default="false"/>        #pointcloud type，false: xyzirt,true:xyzi
    <arg name="use_time_service" default="false" />     # Whether gps time synchronization

  <node pkg="lslidar_driver" type="lslidar_ch_driver_node" name="lslidar_driver_node" output="screen">
    <!--param name="pcap" value="$(find lslidar_driver)/pcap/tt.pcap"/-->     #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
    <param name="lidar_ip" value="$(arg device_ip)"/>
    <param name="lidar_type" value="$(arg lidar_type)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="pcl_type" value="$(arg pcl_type)"/>
    <param name="add_multicast" value="false"/>    # Whether to add multicast
    <param name="group_ip" value="224.1.1.2"/>       #multicast ip
    <param name="frame_id" value="laser_link"/>     # lidar point cloud coordinate system name
    <param name="min_range" value="0.15"/>         #Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    <param name="max_range" value="500.0"/>       #Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
     <param name="packet_rate" value="11228.0"/>      #Number of packets played per second when playing pcap
    <param name="angle_disable_min" value="0"/>      #lidar clipping angle start value ，range [0,18000]
    <param name="angle_disable_max" value="0"/>      #lidar clipping angle end value ，range [0,18000]
    <Param name="pointcloud_topic" value="lslidar_point_cloud"/>     #point cloud topic name, can be modified
    <param name="horizontal_angle_resolution" value="0.18"/>    #40Hz:0.09  80Hz:0.18 120Hz: 0.36
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="publish_laserscan" value="false"/>      #Whether to publish the scan
    <param name="each_num" value="0"/>      #Only valid in double echo mode, 0 means release of all point clouds, 1 means release of the first echo point cloud, and 2 means release of the second echo point cloud
  </node>
</launch>
~~~

### Multicast mode:

- The host computer sets the lidar to enable multicast mode

- Modify the following parameters of the launch file

  ~~~xml
  add_multicast: true
  group_ip: 224.1.1.2    //The multicast ip address set by the host computer
  ~~~

- Run the following command to add the computer to the group (replace enp2s0 in the command with the network card name of the user's computer, you can use ifconfig to view the network card name)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### Offline pcap mode:

- Modify the following parameters of the launch file

  ~~~xml
  // uncomment
      <param name="pcap" value="$(find lslidar_driver)/pcap/tt.pcap"/>   
  #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar                 
  ~~~



###  pcl point cloud type:

- Modify the following parameters of the launch file

  ~~~xml
  pcl_type: false      # pointcloud type，false: xyzirt,true:xyzi
  ~~~

- The default false is the custom point cloud type, which references the definition in the file of

  lslidar_driver/include/lslidar_driver.h

  Change it to true, which is the own type of pcl:

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~

## FAQ

Bug Report

version :LSLIDAR_CHX1_V1.1.9_221107_ROS

Modify:  original version

Date    : 2022-11-04

-----



update version:LSLIDAR_CHX1_V1.2.1_230316_ROS

Modify:  

Reduce CPU consumption,

Change the time of point to relative time,

New compatible cx128s2 radar..

Date    : 2023--03-16

----



update version:LSLIDAR_CHX1_V1.2.2_230414_ROS

Modify:  

New compatible cx126s3 radar.

Date    : 2023-04-14

---



update version:LSLIDAR_CHX1_V1.2.3_230704_ROS

Modify:  

New compatible cb64s1_a radar.

Date    : 2023-07-04

---



update version:LSLIDAR_CHX1_V1.2.4_230901_ROS

Modify:  

New compatible cx6s3 and ch256 、cx1s3 radar.

Date    : 2023-09-01

---

