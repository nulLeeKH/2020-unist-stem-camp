/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

/*
MIT License

Copyright (c) 2020 Lee Kyung-ha <i_am@nulleekh.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "CYdLidar.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

using namespace ydlidar;

#define ROSVerision "1.4.1"


std::vector<float> split(const std::string &s, char delim) {
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "ydlidar_node");
    printf(" _________________________________________________________________  \n"); 
    printf("|  _____________________________________________________________  | \n");
    printf("| |   _     _      ___     _      _     _______     _________   | |\n");
    printf("| |  | |   | |    |   \\   | |    | |   |  _____|   |___   ___|  | |\n");
    printf("| |  | |   | |    | |\\ \\  | |    | |   | |_____        | |      | |\n");
    printf("| |  | |   | |    | | \\ \\ | |    | |   |_____  |       | |      | |\n");
    printf("| |  | |___| |    | |  \\ \\| |    | |    _____| |       | |      | |\n");
    printf("| |  |_______|    |_|   \\___|    |_|   |_______|       |_|      | |\n");
    printf("| |_____________________________________________________________| |\n");
    printf("|_________________________________________________________________|  \n"); 

    printf("\n\n\n");

    printf("db   dD    d88b d8888b.                  db       .d88b.  db    db d88888b         d8b        d8b    \n");
    printf("88 ,8P'    `8P' 88  `8D                  88      .8P  Y8. 88    88 88'           `8  '8b   d8'   8`  \n");
    printf("88,8P       88  88oooY'                  88      88    88 Y8    8P 88ooooo        `Y   'd8b`    P`   \n");
    printf("88`8b       88  88~~~b.      C8888D      88      88    88 `8b  d8' 88~~~~~         `8b        d8'    \n");
    printf("88 `88. db. 88  88   8D                  88booo. `8b  d8'  `8bd8'  88.               `8b    d8'      \n");
    printf("YP   YD Y8888P  Y8888P'                  Y88888P  `Y88P'     YP    Y88888P             `8bd8'        \n");

    printf("\n\n\n");

    printf(" _     _   ___   _   _       ___ _____   _   ___   ____  ___  _   _______ _____ \n");
    printf("| |   | | / | | | | | |     |_  |  ___| | | / | | / |  \\/  | | | / | ___ |  __ \\\n");
    printf("| |   | |/ /| |_| | | |       | | |__   | |/ /| |/ /| .  . | | |/ /| |_/ | |  \\/\n");
    printf("| |   |    \\|  _  | | |       | |  __|  |    \\|    \\| |\\/| | |    \\| ___ | | __ \n");
    printf("| |___| |\\  | | | | | |___/\\__/ | |___  | |\\  | |\\  | |  | | | |\\  | |_/ | |_\\ \\\n");
    printf("\\_____\\_| \\_\\_| |_/ \\_____\\____/\\____/  \\_| \\_\\_| \\_\\_|  |_/ \\_| \\_\\____/ \\____/\n");    

    printf("            ");

    printf("\n");
    fflush(stdout);
  
    std::string port;
    int baudrate=115200;
    std::string frame_id;
    bool reversion, resolution_fixed;
    bool auto_reconnect;
    double angle_max,angle_min;
    result_t op_result;
    std::string list;
    std::vector<float> ignore_array;  
    double max_range, min_range;
    bool sun_noise, glass_noise;
    int max_abnormal_check_count;
    double OffsetTime = 0.0;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ydlidar"); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("resolution_fixed", resolution_fixed, "true");
    nh_private.param<bool>("auto_reconnect", auto_reconnect, "true");
    nh_private.param<bool>("sun_noise", sun_noise, "true");
    nh_private.param<bool>("glass_noise", glass_noise, "true");
    nh_private.param<bool>("reversion", reversion, "false");
    nh_private.param<double>("angle_max", angle_max , 180);
    nh_private.param<double>("angle_min", angle_min , -180);
    nh_private.param<double>("range_max", max_range , 12.0);
    nh_private.param<double>("range_min", min_range , 0.08);
    nh_private.param<double>("OffsetTime", OffsetTime , 0.0);
    nh_private.param<int>("max_abnormal_check_count", max_abnormal_check_count , 2);
    nh_private.param<std::string>("ignore_array",list,"");

    ignore_array = split(list ,',');
    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            ROS_ERROR_STREAM("ignore array should be between -180 and 180");
        }
    }

    CYdLidar laser;
    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }
    if (max_abnormal_check_count < 2) {
        max_abnormal_check_count = 2;
    }

    ROS_INFO("[YDLIDAR INFO] Now YDLIDAR ROS SDK VERSION:%s .......", ROSVerision);
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setMaxRange(max_range);
    laser.setMinRange(min_range);
    laser.setMaxAngle(angle_max);
    laser.setMinAngle(angle_min);
    laser.setReversion(reversion);
    laser.setAutoReconnect(auto_reconnect);
    laser.setSunNoise(sun_noise);
    laser.setGlassNoise(glass_noise);
    laser.setAbnormalCheckCount(max_abnormal_check_count);
    laser.setIgnoreArray(ignore_array);
    laser.setOffsetTime(OffsetTime);
    bool ret = laser.initialize();
    if (ret) {
        ret = laser.turnOn();
        if (!ret) {
            ROS_ERROR("Failed to start scan mode!!!");
        }
    } else {
        ROS_ERROR("Error initializing YDLIDAR Comms and Status!!!");
    }
    ros::Rate rate(20);

    while (ret&&ros::ok()) {
        bool hardError;
        LaserScan scan;//
        if(laser.doProcessSimple(scan, hardError )){
            sensor_msgs::LaserScan scan_msg;
            ros::Time start_scan_time;
            start_scan_time.sec = scan.system_time_stamp/1000000000ul;
            start_scan_time.nsec = scan.system_time_stamp%1000000000ul;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frame_id;
            scan_msg.angle_min =(scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);
            int fixed_size = scan.data.size();
            if(resolution_fixed) {
                fixed_size = laser.getFixedSize();
            }
            if(scan.config.max_angle - scan.config.min_angle == 2*M_PI) {
                scan_msg.angle_increment = (scan.config.max_angle - scan.config.min_angle) / (fixed_size);
            } else {
                scan_msg.angle_increment = (scan.config.max_angle - scan.config.min_angle) / (fixed_size - 1);
            }
            int index = 0;
            scan_msg.ranges.resize(fixed_size, std::numeric_limits<float>::infinity());
            scan_msg.intensities.resize(fixed_size, 0);

             for(int i = 0; i < scan.data.size(); i++) {
                LaserPoint point = scan.data[i];
                index = (point.angle - scan.config.min_angle ) / scan_msg.angle_increment + 0.5;
                if(index >=0 && index < fixed_size) {
                    if(point.range == 0.0) {
                        //scan_msg.ranges[index] = std::numeric_limits<float>::infinity();
                        scan_msg.ranges[index] = point.range;
                        scan_msg.intensities[index] = point.intensity;
                    } else {
                        scan_msg.ranges[index] = point.range;
                        scan_msg.intensities[index] = point.intensity;
                    }
                }

            }       
            scan_pub.publish(scan_msg);
        }  
        rate.sleep();
        ros::spinOnce();
    }

    laser.turnOff();
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
    laser.disconnecting();
    return 0;
}