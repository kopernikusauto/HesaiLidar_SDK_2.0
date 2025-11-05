/************************************************************************************************
Copyright (C) 2023 Hesai Technology Co., Ltd.
Copyright (C) 2023 Original Authors
All rights reserved.

All code in this repository is released under the terms of the following Modified BSD License. 
Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and 
  the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
  the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************************/

#pragma once
#include <string>
#include "logger.h"  
namespace hesai
{
namespace lidar
{

#define NULL_TOPIC  "your topic name"

enum SourceType
{
  DATA_FROM_LIDAR = 1,
  DATA_FROM_PCAP = 2,
  DATA_FROM_ROS_PACKET = 3,
  DATA_FROM_SERIAL = 4,
};

enum UseTimestampType
{
  point_cloud_timestamp = 0,
  sdk_recv_timestamp = 1,
};

///< The Point transform parameter
typedef struct TransformParam  
{
  bool use_flag = false;
  ///< unit, m
  float x = 0.0f; 
  ///< unit, m     
  float y = 0.0f;
  ///< unit, m      
  float z = 0.0f;  
  ///< unit, radian    
  float roll = 0.0f;  
  ///< unit, radian 
  float pitch = 0.0f;  
  ///< unit, radian
  float yaw = 0.0f;    
} TransformParam;

///< LiDAR decoder parameter
typedef struct DecoderParam  
{
  ///< Used to transform points
  TransformParam transform_param;    
  int thread_num = 1;
  bool enable_udp_thread = true;
  bool enable_parser_thread = true;
  bool pcap_play_synchronization = true;
  //start a new frame when lidar azimuth greater than frame_start_azimuth
  //range:[0-360), set frame_start_azimuth less than 0 if you do want to use it
  float frame_start_azimuth = 0;
  // 0 use point cloud timestamp
  // 1 use sdk receive timestamp
  uint16_t use_timestamp_type = point_cloud_timestamp;
  int fov_start = -1;
  int fov_end = -1;
  bool distance_correction_flag = false;
  uint32_t socket_buffer_size = 0;
} DecoderParam;

///< The LiDAR input parameter
typedef struct InputParam  
{
  SourceType source_type = DATA_FROM_PCAP;
  // Ip of Lidar
  std::string device_ip_address = "";   
  ///< Address of multicast
  std::string multicast_ip_address = "";  
  uint16_t udp_port = 2368;   
  uint16_t ptc_port = 9347;
  bool is_use_ptc = false;
  ///< serial port and baudrate
  std::string pcap_path = "Your pcap file path";  ///< Absolute path of pcap file
  std::string correction_file_path = "Your correction file path";   ///< Path of angle calibration files(angle.csv).Only used for internal debugging.
  std::string firetimes_path = "Your firetime file path";  ///< Path of firetime files(angle.csv).

  bool send_packet_ros;
  bool send_point_cloud_ros;
  std::string frame_id;

  std::string ros_send_packet_topic = NULL_TOPIC;
  std::string ros_send_point_topic = NULL_TOPIC;
  std::string ros_send_correction_topic = NULL_TOPIC;
  std::string ros_send_imu_topic = NULL_TOPIC;

  std::string ros_recv_correction_topic = NULL_TOPIC;
  std::string ros_recv_packet_topic = NULL_TOPIC;


} InputParam;

///< The LiDAR driver parameter
typedef struct DriverParam  
{
  ///< Input parameter
  InputParam input_param;  
  ///< Decoder parameter        
  DecoderParam decoder_param;  
  ///< The frame id of LiDAR message    
  std::string frame_id = "hesai";  
  bool use_gpu = false;
  uint8_t log_level = LOG_INFO | LOG_WARNING | LOG_ERROR | LOG_FATAL; //
  uint8_t log_Target = LOG_TARGET_CONSOLE | LOG_TARGET_FILE;
  std::string log_path = "./log.log";
} DriverParam;
}  // namespace lidar
}  // namespace hesai