/******************************************************************************
 * This file is part of lslidar driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <lslidar_ch_driver/lslidar_ch_driver.h>

namespace lslidar_ch_driver {

    LslidarChDriver::LslidarChDriver(
            ros::NodeHandle &n, ros::NodeHandle &pn) :
            nh(n),
            pnh(pn),
            socket_id(-1),
            point_cloud_xyzirt_(new pcl::PointCloud<VPoint>),
            point_cloud_xyzi_(new pcl::PointCloud<pcl::PointXYZI>),
            point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>),
            point_cloud_xyzi_bak_(new pcl::PointCloud<pcl::PointXYZI>),
            scan_msg(new sensor_msgs::LaserScan),
            scan_msg_bak(new sensor_msgs::LaserScan),
            gain_prism_angle(true) {
        prism_offset = 0.0;
        last_packet_timestamp = 0.0;
        is_update_difop_packet = false;
        for (int j = 0; j < 36000; ++j) {
            sin_list[j] = sin(j * 0.01 * DEG_TO_RAD);
            cos_list[j] = cos(j * 0.01 * DEG_TO_RAD);
        }
        ROS_INFO("***********************CHX1 ROS1 DRIVER VERSION: %s ***********************", lslidar_driver_VERSION);
        return;
    }

    LslidarChDriver::~LslidarChDriver() {

        if (nullptr != difop_thread_) {
            difop_thread_->join();
        }
        return;
    }

    bool LslidarChDriver::loadParameters() {

        pnh.param("frame_id", frame_id, std::string("laser_link"));
        pnh.param("lidar_ip", lidar_ip_string, std::string("192.168.1.200"));
        pnh.param<bool>("add_multicast", add_multicast, false);
        pnh.param<bool>("pcl_type", pcl_type, false);
        pnh.param<std::string>("lidar_type", lidar_type, "ch128x1");
        pnh.param("group_ip", group_ip_string, std::string("224.1.1.2"));
        pnh.param("msop_port", msop_udp_port, 2368);
        pnh.param("difop_port", difop_udp_port, 2369);
        pnh.param("min_range", min_range, 0.5);
        pnh.param("max_range", max_range, 150.0);
        pnh.param<double>("angle_disable_min", angle_disable_min, 0.0);
        pnh.param<double>("angle_disable_max", angle_disable_max, 0.0);
        pnh.param<double>("horizontal_angle_resolution", horizontal_angle_resolution, 0.2);
        pnh.param<std::string>("pointcloud_topic", pointcloud_topic, "lslidar_point_cloud");
        pnh.param<bool>("publish_laserscan", publish_laserscan, false);
        pnh.param<int>("channel_num", channel_num, 8);
        pnh.param<int>("echo_num", echo_num, 0);
        pnh.param("pcap", dump_file, std::string(""));
        pnh.param("packet_rate", packet_rate, 6737.0);
        pnh.param("use_time_service", use_time_service, false);

        ROS_INFO("using time service or not: %d", use_time_service);
        ROS_INFO("lidar type: %s", lidar_type.c_str());
        inet_aton(lidar_ip_string.c_str(), &lidar_ip);


        if (add_multicast) ROS_INFO_STREAM("Opening UDP socket: group_address " << group_ip_string);


        if (publish_laserscan) {
            if (channel_num < 0) {
                channel_num = 0;
                ROS_WARN("channel_num outside of the index, select channel 0 instead!");
            } else if (channel_num > 255) {
                channel_num = 255;
                ROS_WARN("channel_num outside of the index, select channel 127 instead!");
            }
            ROS_INFO("select channel num: %d", channel_num);
        }

/*        switch (frequency) {
            case 5:
                horizontal_angle_resolution = DEG2RAD(0.1);
                break;
            case 20:
                horizontal_angle_resolution = DEG2RAD(0.4);
                break;
            default:
                horizontal_angle_resolution = DEG2RAD(0.2);
        }*/


        return true;
    }

    bool LslidarChDriver::createRosIO() {

        // ROS diagnostics

        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);
        laserscan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
        lslidar_control = nh.advertiseService("lslidarcontrol", &LslidarChDriver::lslidarControl, this);
        if (dump_file != "") {
            if (lidar_type == "ch128s1" || lidar_type == "ch16x1" || lidar_type == "cx128s2" ||
                lidar_type == "cx126s3" || lidar_type == "cx6s3" || lidar_type == "ch256" || lidar_type == "cx1s3") {
                msop_input_.reset(new lslidar_ch_driver::InputPCAP(pnh, msop_udp_port, 1212, packet_rate, dump_file));
                difop_input_.reset(new lslidar_ch_driver::InputPCAP(pnh, difop_udp_port, 1206, 1, dump_file));
            } else {
                msop_input_.reset(new lslidar_ch_driver::InputPCAP(pnh, msop_udp_port, 1206, packet_rate, dump_file));
                difop_input_.reset(new lslidar_ch_driver::InputPCAP(pnh, difop_udp_port, 1206, 1, dump_file));
            }

        } else {
            if (lidar_type == "ch128s1" || lidar_type == "ch16x1" || lidar_type == "cx128s2" ||
                lidar_type == "cx126s3" || lidar_type == "cx6s3" || lidar_type == "ch256" || lidar_type == "cx1s3") {
                msop_input_.reset(new lslidar_ch_driver::InputSocket(pnh, msop_udp_port, 1212));
                difop_input_.reset(new lslidar_ch_driver::InputSocket(pnh, difop_udp_port, 1206));
            } else {
                msop_input_.reset(new lslidar_ch_driver::InputSocket(pnh, msop_udp_port, 1206));
                difop_input_.reset(new lslidar_ch_driver::InputSocket(pnh, difop_udp_port, 1206));
            }
        }

        difop_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&LslidarChDriver::difopPoll, this)));


        return true;
    }

    bool LslidarChDriver::initialize() {

        this->initTimeStamp();

        if (!loadParameters()) {
            ROS_ERROR("Cannot load all required ROS parameters...");
            return false;
        }
        for (double &j : prism_angle) {
            j = 0.0f;
        }
        //ch256
        for (int k1 = 0; k1 < 256; ++k1) {
            ch256_sin_theta_1[k1] = sin((-20.0 + floor(k1 / 4) * 0.625) * DEG_TO_RAD);
            ch256_cos_theta_1[k1] = cos((-20.0 + floor(k1 / 4) * 0.625) * DEG_TO_RAD);
            ch256_sin_theta_2[k1] = sin((k1 % 4) * 0.11 * DEG_TO_RAD);
            ch256_cos_theta_2[k1] = cos((k1 % 4) * 0.11 * DEG_TO_RAD);
        }
        // ch64w
        for (int m = 0; m < 128; ++m) {
            //右边
            if (m / 4 % 2 == 0) {
                ch64w_sin_theta_1[m] = sin((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                ch64w_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                ch64w_cos_theta_1[m] = cos((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                ch64w_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
            } else { //左边
                ch64w_sin_theta_1[m] = sin((-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                ch64w_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                ch64w_cos_theta_1[m] = cos((-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                ch64w_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
            }
        }

        //CB64S1_A
        for (int m = 0; m < 64; ++m) {
            //右边
            cb64s1_A_sin_theta_1[m] = sin((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
            cb64s1_A_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
            cb64s1_A_cos_theta_1[m] = cos((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
            cb64s1_A_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
        }


        for (int i = 0; i < 128; i++) {
            // 左边
            if (i / 4 % 2 == 0) {
                sin_theta_1[i] = sin((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);
                cos_theta_1[i] = cos((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);

            } else {
                sin_theta_1[i] = sin(big_angle[i / 4] * DEG_TO_RAD);
                cos_theta_1[i] = cos(big_angle[i / 4] * DEG_TO_RAD);
            }

            if (lidar_type == "ch128s1" || lidar_type == "cx128s2") {
                sin_theta_1[i] = sin(big_angle_ch128s1[i / 4] * DEG_TO_RAD);
                cos_theta_1[i] = cos(big_angle_ch128s1[i / 4] * DEG_TO_RAD);
            }
            sin_theta_2[i] = sin((i % 4) * (-0.17) * DEG_TO_RAD);
            cos_theta_2[i] = cos((i % 4) * (-0.17) * DEG_TO_RAD);
        }
        for (int l = 0; l < 126; ++l) {
            cx126s3_sin_theta_1[l] = sin(big_angle_cx126s3[l / 3] * DEG_TO_RAD);
            cx126s3_cos_theta_1[l] = cos(big_angle_cx126s3[l / 3] * DEG_TO_RAD);

            cx126s3_sin_theta_2[l] = sin((l % 3) * (-0.14) * DEG_TO_RAD);
            cx126s3_cos_theta_2[l] = cos((l % 3) * (-0.14) * DEG_TO_RAD);
        }

        for (int i1 = 0; i1 < 6; ++i1) {
            cx6s3_sin_theta_1[i1] = sin(big_angle_cx6s3[i1 / 3] * DEG_TO_RAD);
            cx6s3_cos_theta_1[i1] = cos(big_angle_cx6s3[i1 / 3] * DEG_TO_RAD);

            cx6s3_sin_theta_2[i1] = sin((i1 % 3) * (-0.14) * DEG_TO_RAD);
            cx6s3_cos_theta_2[i1] = cos((i1 % 3) * (-0.14) * DEG_TO_RAD);
        }

        for (int k = 0; k < 16; ++k) {
            // 左边  prism_offset = 0.0
            if (k / 4 % 2 == 0) {
                ch16x1_sin_theta_1[k] = sin((big_angle_ch16x1[k / 4] + prism_offset) * DEG_TO_RAD);
                ch16x1_cos_theta_1[k] = cos((big_angle_ch16x1[k / 4] + prism_offset) * DEG_TO_RAD);
            } else {
                ch16x1_sin_theta_1[k] = sin(big_angle_ch16x1[k / 4] * DEG_TO_RAD);
                ch16x1_cos_theta_1[k] = cos(big_angle_ch16x1[k / 4] * DEG_TO_RAD);
            }
            ch16x1_sin_theta_2[k] = sin((k % 4) * (-0.17) * DEG_TO_RAD);
            ch16x1_cos_theta_2[k] = cos((k % 4) * (-0.17) * DEG_TO_RAD);
        }

        if (!createRosIO()) {
            ROS_ERROR("Cannot create all ROS IO...");
            return false;
        }
        point_cloud_xyzirt_->header.frame_id = frame_id;
        point_cloud_xyzirt_->height = 1;

        point_cloud_xyzi_->header.frame_id = frame_id;
        point_cloud_xyzi_->height = 1;
        if (publish_laserscan) {
            scan_msg->angle_min = DEG2RAD(0);
            scan_msg->angle_max = DEG2RAD(180);
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;

            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
            if (lidar_type == "ch64w") { point_size *= 2; }
            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        }


        return true;
    }

    bool LslidarChDriver::lslidarControl(lslidar_msgs::lslidar_control::Request &req,
                                         lslidar_msgs::lslidar_control::Response &res) {
        ROS_WARN("--------------------------");
        // sleep(1);
        lslidar_msgs::LslidarChPacketPtr packet0(new lslidar_msgs::LslidarChPacket);
        bool ret_val = false;

        packet0->data[0] = 0x00;
        packet0->data[1] = 0x00;
        int rc_msop = -1;

        if (req.LaserControl == 1) {
            if ((rc_msop = msop_input_->getPacket(packet0)) == 0) {
                res.status = 1;
                ROS_WARN("receive cmd: %d,already power on status", req.LaserControl);
                return true;
            }
            ROS_WARN("receive cmd: %d,power on", req.LaserControl);
            ret_val = SendPacketTolidar(true);
            double time1 = ros::Time::now().toSec();
            do {
                rc_msop = msop_input_->getPacket(packet0);
                double time2 = ros::Time::now().toSec();
                if (time2 - time1 > 20) {
                    res.status = 0;
                    ROS_WARN("lidar connect error");
                    return ret_val;
                }
            } while (rc_msop != 0);
            sleep(5);
            res.status = 1;
        } else if (req.LaserControl == 0) {
            ROS_WARN("receive cmd: %d,power off", req.LaserControl);
            ret_val = SendPacketTolidar(false);
            res.status = 1;
        } else {
            ROS_WARN("cmd error");
            res.status = 0;
        }
        return ret_val;

    }

    bool LslidarChDriver::SendPacketTolidar(bool power_switch) {
        int socketid;
        unsigned char config_data[1206];
        //int data_port = difop_data[24] * 256 + difop_data[25];
        if (difop_data[0] != 0xa5 || difop_data[1] != 0xff || difop_data[2] != 0x00 || difop_data[3] != 0x5a) {
            ROS_WARN("----------");
            ROS_WARN("Failed to obtain lidar difop packet");
            ROS_WARN("----------");
            return false;
        }
        mempcpy(config_data, difop_data, 1206);
        config_data[0] = 0xAA;
        config_data[1] = 0x00;
        config_data[2] = 0xFF;
        config_data[3] = 0x11;
        config_data[4] = 0x22;
        config_data[5] = 0x22;
        config_data[6] = 0xAA;
        config_data[7] = 0xAA;
        config_data[8] = 0x02;
        config_data[9] = 0x58;
        if (power_switch) {
            config_data[45] = 0x00;
        } else {
            config_data[45] = 0x01;
        }

        sockaddr_in addrSrv;
        socketid = socket(2, 2, 0);
        addrSrv.sin_addr.s_addr = inet_addr(lidar_ip_string.c_str());
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(2368);
        sendto(socketid, (const char *) config_data, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
        return true;

    }

    void LslidarChDriver::publishLaserScan() {
        if (!is_update_difop_packet) { return; }
        scan_msg_bak->header.frame_id = frame_id;
        scan_msg_bak->header.stamp = ros::Time(point_cloud_timestamp);
        laserscan_pub.publish(scan_msg_bak);

#if 0
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        if (lidar_type == "ch64w") {
            sensor_msgs::LaserScan::Ptr scan_msg(new sensor_msgs::LaserScan);
            scan_msg->header.frame_id = frame_id;
            scan_msg->angle_min = DEG2RAD(0);
            scan_msg->angle_max = DEG2RAD(180);
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
            uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment) * 2;
            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());

            scan_msg->header.stamp = pcl_conversions::fromPCL(point_cloud_timestamp);

            if (channel_num / 4 % 2 == 0) {
                channel_num1 = channel_num;
                channel_num2 = channel_num + 4;
            } else {
                channel_num1 = channel_num;
                channel_num2 = channel_num - 4;
            }
            if (sweep_data_bac->points.size() > 0) {
                for (size_t j = 0; j < sweep_data_bac->points.size() - 1; ++j) {
                    if (channel_num1 == sweep_data_bac->points[j].line ||
                        channel_num2 == sweep_data_bac->points[j].line) {
                        float horizontal_angle = sweep_data_bac->points[j].azimuth;
                        uint point_idx = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                        point_idx = (point_idx < point_size) ? point_idx : (point_idx % point_size);
                        scan_msg->ranges[point_idx] = sweep_data_bac->points[j].distance;
                        scan_msg->intensities[point_idx] = sweep_data_bac->points[j].intensity;
                    }
                }
                laserscan_pub.publish(scan_msg);
            }

        } else {
            sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
            scan_msg->header.frame_id = frame_id;

            //scan_msg->header.stamp = pcl_conversions::fromPCL(point_cloud_timestamp);
            scan_msg->header.stamp = ros::Time(point_cloud_timestamp);

            scan_msg->angle_min = DEG2RAD(0);
            scan_msg->angle_max = DEG2RAD(180);
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
            uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            if (sweep_data_bac->points.size() > 0) {
                for (size_t j = 0; j < sweep_data_bac->points.size(); ++j) {
                    if (channel_num == sweep_data_bac->points[j].line) {
                        float horizontal_angle = sweep_data_bac->points[j].azimuth;
                        uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                        point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                        scan_msg->ranges[point_index] = sweep_data_bac->points[j].distance;
                        scan_msg->intensities[point_index] = sweep_data_bac->points[j].intensity;
                    }
                }
                laserscan_pub.publish(scan_msg);
            }

        }
#endif
    }

    void LslidarChDriver::publishPointCloud() {
        if (!is_update_difop_packet) { return; }
        std::unique_lock<std::mutex> lock(pointcloud_lock);

        if (pcl_type) {
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud_xyzi_bak_, pc_msg);
            pc_msg.header.stamp = ros::Time(point_cloud_timestamp);
            pointcloud_pub.publish(pc_msg);
        } else {
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud_xyzirt_bak_, pc_msg);
            pc_msg.header.stamp = ros::Time(point_cloud_timestamp);
            pointcloud_pub.publish(pc_msg);
        }
    }


    int LslidarChDriver::convertCoordinate(struct Firing &lidardata) {
/*        if (!isPointInRange(lidardata.distance) || lidardata.azimuth > 18000) {
            return -1;
        }*/
        if ((lidardata.azimuth > angle_disable_min) && (lidardata.azimuth < angle_disable_max)) {
            return -1;
        }

        double x = 0.0, y = 0.0, z = 0.0;
        double sin_theat = 0.0;
        double cos_theat = 0.0;
        double add_distance = 0.0;
        double _R_ = 0.0;
        //ch64w
        double cos_xita;
        double sin_xita;
        double cos_H_xita;
        double sin_H_xita;
        double cos_xita_F;
        double sin_xita_F;
        if (lidar_type == "ch128x1" || lidar_type == "ch128s1" || lidar_type == "cx128s2") {
            _R_ = cos_theta_2[lidardata.vertical_line] * cos_theta_1[lidardata.vertical_line] *
                  cos_list[int(lidardata.azimuth * 0.5)] -
                  sin_theta_2[lidardata.vertical_line] * sin_theta_1[lidardata.vertical_line];

            sin_theat = sin_theta_1[lidardata.vertical_line] + 2 * _R_ * sin_theta_2[lidardata.vertical_line];
            cos_theat = sqrt(1 - pow(sin_theat, 2));
            x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
            y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
            z = lidardata.distance * sin_theat;
        } else if (lidar_type == "cx126s3") {
            _R_ = cx126s3_cos_theta_2[lidardata.vertical_line] * cx126s3_cos_theta_1[lidardata.vertical_line] *
                  cos_list[int(lidardata.azimuth * 0.5)] -
                  cx126s3_sin_theta_2[lidardata.vertical_line] * cx126s3_sin_theta_1[lidardata.vertical_line];

            sin_theat = cx126s3_sin_theta_1[lidardata.vertical_line] +
                        2 * _R_ * cx126s3_sin_theta_2[lidardata.vertical_line];
            cos_theat = sqrt(1 - pow(sin_theat, 2));
            x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
            y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
            z = lidardata.distance * sin_theat;
        } else if (lidar_type == "cx6s3") {
            _R_ = cx6s3_cos_theta_2[lidardata.vertical_line] * cx6s3_cos_theta_1[lidardata.vertical_line] *
                  cos_list[int(lidardata.azimuth * 0.5)] -
                  cx6s3_sin_theta_2[lidardata.vertical_line] * cx6s3_sin_theta_1[lidardata.vertical_line];

            sin_theat = cx6s3_sin_theta_1[lidardata.vertical_line] +
                        2 * _R_ * cx6s3_sin_theta_2[lidardata.vertical_line];
            cos_theat = sqrt(1 - pow(sin_theat, 2));
            x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
            y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
            z = lidardata.distance * sin_theat;
        }else if(lidar_type =="cx1s3"){
            x = lidardata.distance  * cos_list[lidardata.azimuth];
            y = lidardata.distance  * sin_list[lidardata.azimuth];
            z = 0;
        }else if (lidar_type == "ch16x1") {
            //中间变量
            _R_ = ch16x1_cos_theta_2[lidardata.vertical_line] * ch16x1_cos_theta_1[lidardata.vertical_line] *
                  cos_list[int(lidardata.azimuth * 0.5)] -
                  ch16x1_sin_theta_2[lidardata.vertical_line] * ch16x1_sin_theta_1[lidardata.vertical_line];

            sin_theat =
                    ch16x1_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * ch16x1_sin_theta_2[lidardata.vertical_line];
            cos_theat = sqrt(1 - pow(sin_theat, 2));
            x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
            y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
            z = lidardata.distance * sin_theat;
        } else if (lidar_type == "ch64w") {
            int line_num = lidardata.vertical_line;
            if (line_num / 4 % 2 == 0) {
                cos_xita = cos_list[int(lidardata.azimuth * 0.5 + 2250)];
                sin_xita = sin_list[int(lidardata.azimuth * 0.5 + 2250)];
            } else {
                int angle_tmp =
                        int(11250 - lidardata.azimuth * 0.5) < 0 ? int(11250 - lidardata.azimuth * 0.5) + 36000 : int(
                                11250 - lidardata.azimuth * 0.5);
                cos_xita = cos_list[angle_tmp];
                sin_xita = sin_list[angle_tmp];
            }
            _R_ = ch64w_cos_theta_2[line_num] * ch64w_cos_theta_1[line_num] * cos_xita -
                  ch64w_sin_theta_2[line_num] * ch64w_sin_theta_1[line_num];

            sin_theat = ch64w_sin_theta_1[line_num] + 2 * _R_ * ch64w_sin_theta_2[line_num];
            cos_theat = sqrt(1 - pow(sin_theat, 2));

            cos_H_xita = (2 * _R_ * ch64w_cos_theta_2[line_num] * cos_xita - ch64w_cos_theta_1[line_num]) / cos_theat;
            sin_H_xita = (2 * _R_ * ch64w_cos_theta_2[line_num] * sin_xita) / cos_theat;

            if (line_num / 4 % 2 == 0) {
                cos_xita_F = (cos_H_xita + sin_H_xita) * sqrt_0_5;
                //sin_xita_F = sqrt(1 - pow(cos_xita_F, 2));
                // sin_xita_F = (sin_H_xita - cos_H_xita) * sqrt(0.5);
                if (cos_xita_F > 1.0) {
                    cos_xita_F = 1.0;
                }
                double xita_hangle = acos(cos_xita_F) * RAD_TO_DEG;
                double xita_hangle_new = pow1 * pow(xita_hangle, 3)
                                         + pow2 * pow(xita_hangle, 2)
                                         + 0.9885 * pow(xita_hangle, 1)
                                         + 0.5894;
                while (xita_hangle_new < 0.0) {
                    xita_hangle_new += 360.0;
                }
                int xita_hangle_new_index = int(xita_hangle_new * 100) % 36000;
                cos_xita_F = cos_list[xita_hangle_new_index];
                sin_xita_F = sin_list[xita_hangle_new_index];
                // cos_xita_F = cos(xita_hangle_new * DEG_TO_RAD);
                // sin_xita_F = sin(xita_hangle_new * DEG_TO_RAD);
                add_distance = 0.017;
            } else {
                cos_xita_F = (cos_H_xita + sin_H_xita) * (-sqrt_0_5);
                //sin_xita_F = sqrt(1 - pow(cos_xita_F, 2));
                // sin_xita_F = (sin_H_xita - cos_H_xita) * sqrt(0.5);
                if (cos_xita_F < -1.0) {
                    cos_xita_F = -1.0;
                }
                double xita_hangle = acos(cos_xita_F) * RAD_TO_DEG;
                double xita_hangle_new = pow1 * pow(xita_hangle, 3)
                                         + pow3 * pow(xita_hangle, 2)
                                         + 0.9719 * pow(xita_hangle, 1)
                                         + 1.9003;
                while (xita_hangle_new < 0.0) {
                    xita_hangle_new += 360.0;
                }
                int xita_hangle_new_index = int(xita_hangle_new * 100) % 36000;
                cos_xita_F = cos_list[xita_hangle_new_index];
                sin_xita_F = sin_list[xita_hangle_new_index];
                //  cos_xita_F = cos(xita_hangle_new * DEG_TO_RAD);
                //  sin_xita_F = sin(xita_hangle_new * DEG_TO_RAD);
                add_distance = -0.017;
            }

            x = lidardata.distance * cos_theat * cos_xita_F + add_distance;
            y = lidardata.distance * cos_theat * sin_xita_F;
            z = lidardata.distance * sin_theat;
        } else if (lidar_type == "cb64s1_a") {
            int line_num = lidardata.vertical_line;

            {
                cos_xita = cos_list[int(lidardata.azimuth * 0.5)];
                sin_xita = sin_list[int(lidardata.azimuth * 0.5)];
            }

            _R_ = cb64s1_A_cos_theta_2[line_num] * cb64s1_A_cos_theta_1[line_num] * cos_xita -
                  cb64s1_A_sin_theta_2[line_num] * cb64s1_A_sin_theta_1[line_num];

            sin_theat = cb64s1_A_sin_theta_1[line_num] + 2 * _R_ * cb64s1_A_sin_theta_2[line_num];
            cos_theat = sqrt(1 - pow(sin_theat, 2));

            x = lidardata.distance * cos_theat * cos_list[int(lidardata.azimuth)];
            y = lidardata.distance * cos_theat * sin_list[int(lidardata.azimuth)];
            z = lidardata.distance * sin_theat;
        } else if (lidar_type == "ch256") {
            _R_ = ch256_cos_theta_2[lidardata.vertical_line] * ch256_cos_theta_1[lidardata.vertical_line] *
                  cos_list[int(lidardata.azimuth * 0.5)] -
                  ch256_sin_theta_2[lidardata.vertical_line] * ch256_sin_theta_1[lidardata.vertical_line];

            sin_theat = ch256_sin_theta_1[lidardata.vertical_line] +
                        2 * _R_ * ch256_sin_theta_2[lidardata.vertical_line];
            cos_theat = sqrt(1 - pow(sin_theat, 2));
            x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
            y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
            z = lidardata.distance * sin_theat;
        } else {
            ROS_WARN_ONCE("lidar type error,please reset param in launch file");
        }
        //add point
        if (pcl_type) {
            pcl::PointXYZI point_xyzi;
            point_xyzi.x = x;
            point_xyzi.y = y;
            point_xyzi.z = z;
            point_xyzi.intensity = lidardata.intensity;
            point_cloud_xyzi_->points.push_back(point_xyzi);
            ++point_cloud_xyzi_->width;
        } else {
            //pcl::PointXYZI point;
            VPoint point;
            point.time = lidardata.time;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = lidardata.intensity;
            point.ring = lidardata.vertical_line;
            if(lidar_type == "cx1s3"){
                point.ring = 0;
            }
            point_cloud_xyzirt_->points.push_back(point);
            ++point_cloud_xyzirt_->width;
        }
        // laserscan
        if (publish_laserscan) {
            if (lidar_type == "ch64w") {
                if (channel_num / 4 % 2 == 0) {
                    channel_num1 = channel_num;
                    channel_num2 = channel_num + 4;
                } else {
                    channel_num1 = channel_num;
                    channel_num2 = channel_num - 4;
                }

                if (channel_num1 == lidardata.vertical_line ||
                    channel_num2 == lidardata.vertical_line) {
                    float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;
                    uint point_idx = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_idx = (point_idx < point_size) ? point_idx : (point_idx % point_size);
                    scan_msg->ranges[point_idx] = lidardata.distance;
                    scan_msg->intensities[point_idx] = lidardata.intensity;
                }
            } else {
                if (channel_num == lidardata.vertical_line) {
                    float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_index] = lidardata.distance;
                    scan_msg->intensities[point_index] = lidardata.intensity;
                }
            }

        }
        return 0;
    }


    bool LslidarChDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::LslidarChPacketPtr packet(
                new lslidar_msgs::LslidarChPacket());

        int rc = -1;
        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            // keep reading until full packet received

            rc = msop_input_->getPacket(packet);

            if (rc == 0) break;       // got a full packet?
            if (rc < 0) return false; // end of file reached?
        }

        if (use_time_service) {
            if ("ch128x1" == lidar_type || "ch64w" == lidar_type || "cb64s1_a" == lidar_type) {
                if (packet->data[1205] == 0x01) {
                    this->packetTimeStamp[4] = packet->data[1199];
                    this->packetTimeStamp[5] = packet->data[1198];
                    this->packetTimeStamp[6] = packet->data[1197];
                } else if (packet->data[1205] == 0x02) {
                    this->packetTimeStamp[4] = packet->data[1199];
                }

                //struct tm cur_time{};
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec = this->packetTimeStamp[4];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;

                packet_timestamp_s = timegm(&cur_time);
                if (time_service_mode == "gps") {
                    packet_timestamp_ns = (packet->data[1203] +
                                           (packet->data[1202] << 8) +
                                           (packet->data[1201] << 16) +
                                           (packet->data[1200] << 24)) * 1e3; //ns
                } else {
                    packet_timestamp_ns = packet->data[1203] +
                                          (packet->data[1202] << 8) +
                                          (packet->data[1201] << 16) +
                                          (packet->data[1200] << 24); //ns
                }
                packet_timestamp = packet_timestamp_s + packet_timestamp_ns * 1e-9;
                // ch128x1_packet->timestamp = packet_timestamp;
            } else {
                if (packet->data[1200] == 0xff) {
                    packet_timestamp_s = packet->data[1205] +
                                         (packet->data[1204] << 8) +
                                         (packet->data[1203] << 16) +
                                         (packet->data[1202] << 24);

                } else {
                    // struct tm cur_time{};
                    memset(&cur_time, 0, sizeof(cur_time));
                    cur_time.tm_sec = packet->data[1205];
                    cur_time.tm_min = packet->data[1204];
                    cur_time.tm_hour = packet->data[1203];
                    cur_time.tm_mday = packet->data[1202];
                    cur_time.tm_mon = packet->data[1201] - 1;
                    cur_time.tm_year = packet->data[1200] + 2000 - 1900;
                    packet_timestamp_s = timegm(&cur_time);
                }
                packet_timestamp_ns = packet->data[1209] +
                                      (packet->data[1208] << 8) +
                                      (packet->data[1207] << 16) +
                                      (packet->data[1206] << 24); //ns
                packet_timestamp = packet_timestamp_s + packet_timestamp_ns * 1e-9;
                // packet->timestamp = packet_timestamp;
            }
            // it is already the msop msg

        } else {
            packet_timestamp = ros::Time::now().toSec();
        }

        struct Firing lidardata{};


        packet_interval_time = packet_timestamp - last_packet_timestamp;
        last_packet_timestamp = packet_timestamp;

        bool packetType = false;
        // Decode the packet
        if ("ch128x1" == lidar_type || "ch64w" == lidar_type || "cb64s1_a" == lidar_type) {
            if (packet->data[1205] == 0x01) {
                for (size_t point_idx = 0; point_idx < 1197; point_idx += 7) {
                    if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                        (packet->data[point_idx + 2] == 0xbb)) {
                        packetType = true;
                        point_cloud_timestamp =
                                packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1197.0;
                    }


                    if (packet->data[point_idx] < 128) {
                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.distance =
                                ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                 packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                        lidardata.azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                        if (!isPointInRange(lidardata.distance) || lidardata.azimuth >18000) {continue; }
                       // if (!isPointInRange(lidardata.distance)) { continue; }
                        // Compute the time of the point
                        point_time = packet_timestamp - packet_interval_time +
                                     ((int) point_idx - 7) * packet_interval_time / (1197 * 1.0) -
                                     point_cloud_timestamp;

                        lidardata.vertical_line = packet->data[point_idx];
                        lidardata.intensity = packet->data[point_idx + 6];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);
                    }
                    if (packetType) {
                        //("---------------onesweep--------------------------\n");
                        {
                            std::unique_lock<std::mutex> lock(pointcloud_lock);
                            point_cloud_xyzirt_bak_ = point_cloud_xyzirt_;
                            point_cloud_xyzi_bak_ = point_cloud_xyzi_;
                            scan_msg_bak = scan_msg;
                        }
                        std::thread ppc_thread(&LslidarChDriver::publishPointCloud, this);
                        ppc_thread.detach();
                        //publishPointCloud();

                        if (publish_laserscan) {
                            std::thread pls_thread(&LslidarChDriver::publishLaserScan, this);
                            pls_thread.detach();
                            //publishLaserScan();
                        }
                        packetType = false;
                        point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                        point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        point_cloud_xyzirt_->header.frame_id = frame_id;
                        point_cloud_xyzirt_->height = 1;

                        point_cloud_xyzi_->header.frame_id = frame_id;
                        point_cloud_xyzi_->height = 1;
                        if (publish_laserscan) {
                            scan_msg.reset(new sensor_msgs::LaserScan);
                            scan_msg->angle_min = DEG2RAD(0);
                            scan_msg->angle_max = DEG2RAD(180);
                            scan_msg->range_min = min_range;
                            scan_msg->range_max = max_range;
                            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
                            if (lidar_type == "ch64w") { point_size *= 2; }
                            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());

                        }
                    }
                }
            } else if (packet->data[1205] == 0x02) {
                ROS_INFO_ONCE(
                        "lidar is double echo model,and the selected echo is: %d [0 mean double echo; 1 mean first echo; 2 mean second echo]",
                        echo_num);
                for (size_t point_idx = 0; point_idx < 1199; point_idx += 11) {
                    if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                        (packet->data[point_idx + 2] == 0xbb)) {
                        packetType = true;
                        point_cloud_timestamp =
                                packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1199.0;

                    }


                    if (packet->data[point_idx] < 128) {
                        // Compute the time of the point

                        point_time = packet_timestamp - packet_interval_time +
                                     ((int) point_idx - 11) * packet_interval_time / (1199 * 1.0) -
                                     point_cloud_timestamp;
                        if (echo_num == 0) {
                            memset(&lidardata, 0, sizeof(lidardata));
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];

                            lidardata.distance =
                                    ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                     packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                            if (!isPointInRange(lidardata.distance) || lidardata.azimuth >18000) {continue; }
                            lidardata.vertical_line = packet->data[point_idx];

                            lidardata.intensity = packet->data[point_idx + 6];
                            lidardata.time = point_time;
                            convertCoordinate(lidardata);

                            memset(&lidardata, 0, sizeof(lidardata));
                            lidardata.vertical_line = packet->data[point_idx];
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                            lidardata.distance =
                                    ((packet->data[point_idx + 7] << 16) + (packet->data[point_idx + 8] << 8) +
                                     packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                            lidardata.intensity = packet->data[point_idx + 10];
                            lidardata.time = point_time;
                            convertCoordinate(lidardata);
                        } else if (echo_num == 1) {
                            memset(&lidardata, 0, sizeof(lidardata));
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                            lidardata.distance =
                                    ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                     packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                            if (!isPointInRange(lidardata.distance) || lidardata.azimuth >18000) {continue; }
                            lidardata.vertical_line = packet->data[point_idx];

                            lidardata.intensity = packet->data[point_idx + 6];
                            lidardata.time = point_time;
                            convertCoordinate(lidardata);

                        } else if (echo_num == 2) {
                            memset(&lidardata, 0, sizeof(lidardata));
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                            lidardata.distance =
                                    ((packet->data[point_idx + 7] << 16) + (packet->data[point_idx + 8] << 8) +
                                     packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                            if (!isPointInRange(lidardata.distance) || lidardata.azimuth >18000) {continue; }
                            lidardata.vertical_line = packet->data[point_idx];
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                            lidardata.distance =
                                    ((packet->data[point_idx + 7] << 16) + (packet->data[point_idx + 8] << 8) +
                                     packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                            lidardata.intensity = packet->data[point_idx + 10];
                            lidardata.time = point_time;
                            convertCoordinate(lidardata);
                        }

                    }
                    if (packetType) {
                        //("---------------onesweep--------------------------\n");
                        {
                            std::unique_lock<std::mutex> lock(pointcloud_lock);
                            point_cloud_xyzirt_bak_ = point_cloud_xyzirt_;
                            point_cloud_xyzi_bak_ = point_cloud_xyzi_;
                            scan_msg_bak = scan_msg;
                        }
                        std::thread ppc_thread(&LslidarChDriver::publishPointCloud, this);
                        ppc_thread.detach();
                        //publishPointCloud();

                        if (publish_laserscan) {
                            std::thread pls_thread(&LslidarChDriver::publishLaserScan, this);
                            pls_thread.detach();
                            //publishLaserScan();
                        }
                        packetType = false;
                        point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                        point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        point_cloud_xyzirt_->header.frame_id = frame_id;
                        point_cloud_xyzirt_->height = 1;

                        point_cloud_xyzi_->header.frame_id = frame_id;
                        point_cloud_xyzi_->height = 1;
                        if (publish_laserscan) {
                            scan_msg.reset(new sensor_msgs::LaserScan);
                            scan_msg->angle_min = DEG2RAD(0);
                            scan_msg->angle_max = DEG2RAD(180);
                            scan_msg->range_min = min_range;
                            scan_msg->range_max = max_range;
                            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
                            if (lidar_type == "ch64w") { point_size *= 2; }
                            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());

                        }
                    }
                }
            }

        } else {
            if (packet->data[1211] == 0x01) {
                for (size_t point_idx = 0; point_idx < 1197; point_idx += 7) {
                    if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                        (packet->data[point_idx + 2] == 0xbb)) {
                        packetType = true;
                        point_cloud_timestamp =
                                packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1197.0;

                    } else {
                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                        lidardata.distance =
                                ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                 packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                        if (!isPointInRange(lidardata.distance) || lidardata.azimuth >18000) {continue; }
                        // Compute the time of the point
                        point_time = packet_timestamp - packet_interval_time +
                                     ((int) point_idx - 7) * packet_interval_time / (1197 * 1.0) -
                                     point_cloud_timestamp;

                        lidardata.vertical_line = packet->data[point_idx];

                        lidardata.intensity = packet->data[point_idx + 6];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);
                    }
                    if (packetType) {
                        //("---------------onesweep--------------------------\n");
                        {
                            std::unique_lock<std::mutex> lock(pointcloud_lock);
                            point_cloud_xyzirt_bak_ = point_cloud_xyzirt_;
                            point_cloud_xyzi_bak_ = point_cloud_xyzi_;
                            scan_msg_bak = scan_msg;
                        }
                        std::thread ppc_thread(&LslidarChDriver::publishPointCloud, this);
                        ppc_thread.detach();
                        //publishPointCloud();

                        if (publish_laserscan) {
                            std::thread pls_thread(&LslidarChDriver::publishLaserScan, this);
                            pls_thread.detach();
                            //publishLaserScan();
                        }
                        packetType = false;
                        point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                        point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        point_cloud_xyzirt_->header.frame_id = frame_id;
                        point_cloud_xyzirt_->height = 1;

                        point_cloud_xyzi_->header.frame_id = frame_id;
                        point_cloud_xyzi_->height = 1;

                        if (publish_laserscan) {
                            scan_msg.reset(new sensor_msgs::LaserScan);
                            scan_msg->angle_min = DEG2RAD(0);
                            scan_msg->angle_max = DEG2RAD(180);
                            scan_msg->range_min = min_range;
                            scan_msg->range_max = max_range;
                            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
                            if (lidar_type == "ch64w") { point_size *= 2; }
                            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());

                        }
                    }
                }
            } else if (packet->data[1211] == 0x02) {
                for (size_t point_idx = 0; point_idx < 1199; point_idx += 11) {
                    if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                        (packet->data[point_idx + 2] == 0xbb)) {
                        packetType = true;
                        point_cloud_timestamp =
                                packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1199.0;

                    } else {
                        // Compute the time of the point
                        point_time = packet_timestamp - packet_interval_time +
                                     ((int) point_idx - 11) * packet_interval_time / (1199 * 1.0) -
                                     point_cloud_timestamp;

                        if (echo_num == 0) {
                            memset(&lidardata, 0, sizeof(lidardata));
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];

                            lidardata.distance =
                                    ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                     packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                            if (!isPointInRange(lidardata.distance) || lidardata.azimuth >18000) {continue; }
                            lidardata.vertical_line = packet->data[point_idx];

                            lidardata.intensity = packet->data[point_idx + 6];
                            lidardata.time = point_time;
                            convertCoordinate(lidardata);

                            memset(&lidardata, 0, sizeof(lidardata));
                            lidardata.vertical_line = packet->data[point_idx];
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                            lidardata.distance =
                                    ((packet->data[point_idx + 7] << 16) + (packet->data[point_idx + 8] << 8) +
                                     packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                            lidardata.intensity = packet->data[point_idx + 10];
                            lidardata.time = point_time;
                            convertCoordinate(lidardata);
                        } else if (echo_num == 1) {
                            memset(&lidardata, 0, sizeof(lidardata));
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];

                            lidardata.distance =
                                    ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                     packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                            if (!isPointInRange(lidardata.distance) || lidardata.azimuth >18000) {continue; }
                            lidardata.vertical_line = packet->data[point_idx];

                            lidardata.intensity = packet->data[point_idx + 6];
                            lidardata.time = point_time;
                            convertCoordinate(lidardata);

                        } else if (echo_num == 2) {
                            memset(&lidardata, 0, sizeof(lidardata));
                            lidardata.azimuth =
                                    (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                            lidardata.distance =
                                    ((packet->data[point_idx + 7] << 16) + (packet->data[point_idx + 8] << 8) +
                                     packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                            if (!isPointInRange(lidardata.distance) || lidardata.azimuth >18000) {continue; }
                            lidardata.vertical_line = packet->data[point_idx];
                            lidardata.intensity = packet->data[point_idx + 10];
                            lidardata.time = point_time;
                            convertCoordinate(lidardata);
                        }

                        //---------------

                    }
                    if (packetType) {
                        //("---------------onesweep--------------------------\n");
                        {
                            std::unique_lock<std::mutex> lock(pointcloud_lock);
                            point_cloud_xyzirt_bak_ = point_cloud_xyzirt_;
                            point_cloud_xyzi_bak_ = point_cloud_xyzi_;
                            scan_msg_bak = scan_msg;
                        }
                        std::thread ppc_thread(&LslidarChDriver::publishPointCloud, this);
                        ppc_thread.detach();
                        //publishPointCloud();

                        if (publish_laserscan) {
                            std::thread pls_thread(&LslidarChDriver::publishLaserScan, this);
                            pls_thread.detach();
                            //publishLaserScan();
                        }
                        packetType = false;
                        point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                        point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        point_cloud_xyzirt_->header.frame_id = frame_id;
                        point_cloud_xyzirt_->height = 1;

                        point_cloud_xyzi_->header.frame_id = frame_id;
                        point_cloud_xyzi_->height = 1;
                        if (publish_laserscan) {
                            scan_msg.reset(new sensor_msgs::LaserScan);
                            scan_msg->angle_min = DEG2RAD(0);
                            scan_msg->angle_max = DEG2RAD(180);
                            scan_msg->range_min = min_range;
                            scan_msg->range_max = max_range;
                            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
                            if (lidar_type == "ch64w") { point_size *= 2; }
                            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());

                        }
                    }
                }
            }

        }

        return true;
    }

    void LslidarChDriver::initTimeStamp(void) {

        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
    }

    void LslidarChDriver::difopPoll(void) {
        lslidar_msgs::LslidarChPacketPtr difop_packet(
                new lslidar_msgs::LslidarChPacket());
        // reading and publishing scans as fast as possible.
        while (ros::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                // getFPGA_GPSTimeStamp(difop_packet);
                if (difop_packet->data[0] == 0xa5 && difop_packet->data[1] == 0xff &&
                    difop_packet->data[2] == 0x00 &&
                    difop_packet->data[3] == 0x5a) {
                    is_update_difop_packet = true;
                    if (difop_packet->data[44] == 0x00) {
                        //gps授时
                        time_service_mode = "gps";
                    } else if (difop_packet->data[44] == 0x01) {
                        //ptp授时
                        time_service_mode = "gptp";
                    }
                    if (use_time_service) {
                        ROS_INFO("time service mode: %s", time_service_mode.c_str());
                    }

                    for (int j = 0; j < 1206; ++j) {
                        difop_data[j] = difop_packet->data[j];
                    }

                    if (gain_prism_angle) {
                        // 240 241   左边 增加角度
                        int prism_offset_difop = difop_packet->data[240] * 256 + difop_packet->data[241];

                        prism_offset_difop =
                                prism_offset_difop > 32767 ? prism_offset_difop - 65536 : prism_offset_difop;
                        this->prism_offset = prism_offset_difop * 0.01;
                        //ROS_INFO("prism=%f",prism_offset);

                        int angle0 = difop_packet->data[242] * 256 + difop_packet->data[243];
                        angle0 = angle0 > 32767 ? (angle0 - 65536) : angle0;
                        this->prism_angle[0] = angle0 * 0.01;
                        //ROS_INFO("0aa=%f",prism_angle[0]);

                        int angle1 = difop_packet->data[244] * 256 + difop_packet->data[245];
                        angle1 = angle1 > 32767 ? (angle1 - 65536) : angle1;
                        this->prism_angle[1] = angle1 * 0.01;
                        //ROS_INFO("1aa=%f",prism_angle[1]);

                        int angle2 = difop_packet->data[246] * 256 + difop_packet->data[247];
                        angle2 = angle2 > 32767 ? (angle2 - 65536) : angle2;
                        this->prism_angle[2] = angle2 * 0.01;
                        //ROS_INFO("2aa=%f",prism_angle[2]);

                        int angle3 = difop_packet->data[248] * 256 + difop_packet->data[249];
                        angle3 = angle3 > 32767 ? (angle3 - 65536) : angle3;
                        this->prism_angle[3] = angle3 * 0.01;
                        //ROS_INFO("3aa=%f",prism_angle[3]);
                        if (lidar_type == "ch128x1" || lidar_type == "ch128s1" || lidar_type == "cx128s2") {
                            for (int i = 0; i < 128; i++) {
                                // sinTheta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 &&
                                    fabs(this->prism_angle[3]) < 1e-6) {

                                    sin_theta_2[i] = sin((i % 4) * (-0.17) * DEG_TO_RAD);
                                    cos_theta_2[i] = cos((i % 4) * (-0.17) * DEG_TO_RAD);
                                } else {
                                    sin_theta_2[i] = sin(this->prism_angle[i % 4] * DEG_TO_RAD);
                                    cos_theta_2[i] = cos(this->prism_angle[i % 4] * DEG_TO_RAD);
                                }

                                // 左边
                                if (i / 4 % 2 == 0) {
                                    sin_theta_1[i] = sin((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);
                                    cos_theta_1[i] = cos((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);

                                } else {
                                    sin_theta_1[i] = sin(big_angle[i / 4] * DEG_TO_RAD);
                                    cos_theta_1[i] = cos(big_angle[i / 4] * DEG_TO_RAD);
                                }

                                if (lidar_type == "ch128s1" || lidar_type == "cx128s2") {
                                    // 左边
                                    if (i / 4 % 2 == 0) {
                                        sin_theta_1[i] = sin(
                                                (big_angle_ch128s1[i / 4] + prism_offset) * DEG_TO_RAD);
                                        cos_theta_1[i] = cos(
                                                (big_angle_ch128s1[i / 4] + prism_offset) * DEG_TO_RAD);
                                    } else {
                                        sin_theta_1[i] = sin(big_angle_ch128s1[i / 4] * DEG_TO_RAD);
                                        cos_theta_1[i] = cos(big_angle_ch128s1[i / 4] * DEG_TO_RAD);
                                    }

                                }
                            }

                        } else if (lidar_type == "ch256") {
                            for (int i = 0; i < 256; i++) {
                                // sinTheta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 &&
                                    fabs(this->prism_angle[3]) < 1e-6) {
                                    ch256_sin_theta_2[i] = sin((i % 4) * 0.11 * DEG_TO_RAD);
                                    ch256_cos_theta_2[i] = cos((i % 4) * 0.11 * DEG_TO_RAD);
                                } else {
                                    ch256_sin_theta_2[i] = sin(this->prism_angle[i % 4] * DEG_TO_RAD);
                                    ch256_cos_theta_2[i] = cos(this->prism_angle[i % 4] * DEG_TO_RAD);
                                }
                                // 左边
                                if (i / 4 % 2 == 0) {
                                    ch256_sin_theta_1[i] = sin(
                                            (-20.0 + floor(i / 4) * 0.625 + prism_offset) * DEG_TO_RAD);
                                    ch256_cos_theta_1[i] = cos(
                                            (-20.0 + floor(i / 4) * 0.625 + prism_offset) * DEG_TO_RAD);
                                } else {
                                    ch256_sin_theta_1[i] = sin((-20.0 + floor(i / 4) * 0.625) * DEG_TO_RAD);
                                    ch256_cos_theta_1[i] = cos((-20.0 + floor(i / 4) * 0.625) * DEG_TO_RAD);
                                }
                            }
                        } else if (lidar_type == "cx126s3") {
                            for (int i = 0; i < 126; i++) {
                                // sinTheta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6) {
                                    cx126s3_sin_theta_2[i] = sin((i % 3) * (-0.14) * DEG_TO_RAD);
                                    cx126s3_cos_theta_2[i] = cos((i % 3) * (-0.14) * DEG_TO_RAD);
                                } else {
                                    cx126s3_sin_theta_2[i] = sin(this->prism_angle[i % 3] * DEG_TO_RAD);
                                    cx126s3_cos_theta_2[i] = cos(this->prism_angle[i % 3] * DEG_TO_RAD);
                                }

                                // 左边
                                if (i / 3 % 2 == 0) {
                                    sin_theta_1[i] = sin((big_angle_cx126s3[i / 3] + prism_offset) * DEG_TO_RAD);
                                    cos_theta_1[i] = cos((big_angle_cx126s3[i / 3] + prism_offset) * DEG_TO_RAD);

                                } else {
                                    sin_theta_1[i] = sin(big_angle_cx126s3[i / 3] * DEG_TO_RAD);
                                    cos_theta_1[i] = cos(big_angle_cx126s3[i / 3] * DEG_TO_RAD);
                                }
                            }
                        } else if (lidar_type == "cx6s3") {
                            for (int i = 0; i < 6; i++) {
                                // sinTheta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6) {
                                    cx6s3_sin_theta_2[i] = sin((i % 3) * (-0.14) * DEG_TO_RAD);
                                    cx6s3_cos_theta_2[i] = cos((i % 3) * (-0.14) * DEG_TO_RAD);
                                } else {
                                    cx6s3_sin_theta_2[i] = sin(this->prism_angle[i % 3] * DEG_TO_RAD);
                                    cx6s3_cos_theta_2[i] = cos(this->prism_angle[i % 3] * DEG_TO_RAD);
                                }

                                // 左边
                                if (i / 3 % 2 == 0) {
                                    sin_theta_1[i] = sin((big_angle_cx6s3[i / 3] + prism_offset) * DEG_TO_RAD);
                                    cos_theta_1[i] = cos((big_angle_cx6s3[i / 3] + prism_offset) * DEG_TO_RAD);

                                } else {
                                    sin_theta_1[i] = sin(big_angle_cx6s3[i / 3] * DEG_TO_RAD);
                                    cos_theta_1[i] = cos(big_angle_cx6s3[i / 3] * DEG_TO_RAD);
                                }
                            }
                        } else if (lidar_type == "ch16x1") {
                            for (int k = 0; k < 16; ++k) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 &&
                                    fabs(this->prism_angle[3]) < 1e-6) {
                                    ch16x1_sin_theta_2[k] = sin((k % 4) * (-0.17) * M_PI / 180);
                                    ch16x1_cos_theta_2[k] = cos((k % 4) * (-0.17) * M_PI / 180);
                                } else {
                                    ch16x1_sin_theta_2[k] = sin(this->prism_angle[k % 4] * M_PI / 180);
                                    ch16x1_cos_theta_2[k] = cos(this->prism_angle[k % 4] * M_PI / 180);
                                }
                                // 左边
                                if (k / 4 % 2 == 0) {
                                    ch16x1_sin_theta_1[k] = sin(
                                            (big_angle_ch16x1[k / 4] + prism_offset) * DEG_TO_RAD);
                                    ch16x1_cos_theta_1[k] = cos(
                                            (big_angle_ch16x1[k / 4] + prism_offset) * DEG_TO_RAD);
                                } else {
                                    ch16x1_sin_theta_1[k] = sin(big_angle_ch16x1[k / 4] * DEG_TO_RAD);
                                    ch16x1_cos_theta_1[k] = cos(big_angle_ch16x1[k / 4] * DEG_TO_RAD);
                                }
                                ch16x1_sin_theta_2[k] = sin((k % 4) * (-0.17) * DEG_TO_RAD);
                                ch16x1_cos_theta_2[k] = cos((k % 4) * (-0.17) * DEG_TO_RAD);
                            }

                        } else if (lidar_type == "ch64w") {
                            for (int m = 0; m < 128; ++m) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 &&
                                    fabs(this->prism_angle[3]) < 1e-6) {
                                    //右边
                                    if (m / 4 % 2 == 0) {
                                        ch64w_sin_theta_1[m] = sin((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                                        ch64w_cos_theta_1[m] = cos((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
                                    } else { //左边
                                        ch64w_sin_theta_1[m] = sin(
                                                (-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                                        ch64w_cos_theta_1[m] = cos(
                                                (-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
                                    }

                                } else {
                                    //右边
                                    if (m / 4 % 2 == 0) {
                                        ch64w_sin_theta_1[m] = sin((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);
                                        ch64w_cos_theta_1[m] = cos((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                    } else { //左边
                                        ch64w_sin_theta_1[m] = sin(
                                                (-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);
                                        ch64w_cos_theta_1[m] = cos(
                                                (-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                    }

                                }

                            }

                        } else if (lidar_type == "cb64s1_a") {
                            for (int m = 0; m < 64; ++m) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 &&
                                    fabs(this->prism_angle[3]) < 1e-6) {
                                    //右边
                                    // if (m / 4 % 2 == 0)
                                    {
                                        cb64s1_A_sin_theta_1[m] = sin((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
                                        cb64s1_A_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                                        cb64s1_A_cos_theta_1[m] = cos((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
                                        cb64s1_A_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
                                    }

                                } else {
                                    //右边
                                    //if (m / 4 % 2 == 0)
                                    {
                                        cb64s1_A_sin_theta_1[m] = sin((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
                                        cb64s1_A_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);
                                        cb64s1_A_cos_theta_1[m] = cos((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
                                        cb64s1_A_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                    }

                                }

                            }

                        }

                        gain_prism_angle = false;
                    }
                    if ("ch128x1" == lidar_type || "ch64w" == lidar_type || "cb64s1_a" == lidar_type) {
                        if (difop_packet->data[176] == 0x00) {
                            this->packetTimeStamp[7] = difop_packet->data[54];
                            this->packetTimeStamp[8] = difop_packet->data[53];
                            this->packetTimeStamp[9] = difop_packet->data[52];
                        } else if (difop_packet->data[176] == 0x01) {
                            this->packetTimeStamp[5] = difop_packet->data[56];
                            this->packetTimeStamp[6] = difop_packet->data[55];
                            this->packetTimeStamp[7] = difop_packet->data[54];
                            this->packetTimeStamp[8] = difop_packet->data[53];
                            this->packetTimeStamp[9] = difop_packet->data[52];
                        }

                    }

                }

            } else if (rc < 0) {
                return;
            }
        }
    }


/*    void LslidarChDriver::getFPGA_GPSTimeStamp(lslidar_msgs::LslidarChPacketPtr &packet) {
        unsigned char head2[] = {packet->data[0], packet->data[1], packet->data[2], packet->data[3]};

        if (head2[0] == 0xA5 && head2[1] == 0xFF) {
            if (head2[2] == 0x00 && head2[3] == 0x5A) {
                this->packetTimeStamp[7] = packet->data[54];
                this->packetTimeStamp[8] = packet->data[53];
                this->packetTimeStamp[9] = packet->data[52];
            }
        }
    }*/

} // namespace lslidar_driver
