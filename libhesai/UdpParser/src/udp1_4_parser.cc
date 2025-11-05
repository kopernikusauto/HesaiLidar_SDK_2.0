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

/*
 * File:       udp1_4_parser.cc
 * Author:     Chang XingShuo <changxingshuo@hesaitech.com>
 * Description: Implemente Udp1_4Parser class
*/
#ifndef UDP1_4_PARSER_CC_
#define UDP1_4_PARSER_CC_
#include "udp1_4_parser.h"
using namespace hesai::lidar;

template<typename T_Point>
Udp1_4Parser<T_Point>::Udp1_4Parser() {
  optical_center.setNoFlag(LidarOpticalCenter{-0.0076, 0.01363, 0.01271});
  LogInfo("init JT128 parser");
  last_host_timestamp_ = 0;
  last_sensor_timestamp_ = 0;
  packet_count_ = 0;
  source_type_ = -1;
  first_packet_ = true;
  printErrorBool = true;
  frame_start_azimuth_uint16_ = 0;
  last_azimuth_ = 0;
  last_last_azimuth_ = 0;
  crc_initialized = false;
  get_correction_file_ = false;
  for (int i = 0; i < CIRCLE; ++i) {
    sin_all_angle_[i] = std::sin(i * 2 * M_PI / CIRCLE);
    cos_all_angle_[i] = std::cos(i * 2 * M_PI / CIRCLE);
  }
}

template<typename T_Point>
Udp1_4Parser<T_Point>::~Udp1_4Parser() { LogInfo("release JT128 parser"); }

template <typename T_Point>
void Udp1_4Parser<T_Point>::LoadCorrectionFile(const std::string lidar_correction_file) {
  std::ifstream fin(lidar_correction_file, std::ios::in);
  if (fin.is_open()) {
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = static_cast<int>(fin.tellg());
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    int ret = LoadCorrectionString(buffer, length);
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local correction file Error");
    } else {
      LogInfo("Parser correction file success!");
    }
  } else {
    LogError("Open correction file failed");
    return;
  }
}

template <typename T_Point>
int Udp1_4Parser<T_Point>::LoadCorrectionString(const char *correction_content, const int len) {
  try {
    std::string correction_content_str = correction_content;
    std::istringstream ifs(correction_content_str);
    std::string line;
    std::string hash = "";
    int is_hav_eeff = 0;
    // skip first line "Laser id,Elevation,Azimuth" or "eeff"
    std::getline(ifs, line);  
    float elevation_list[DEFAULT_MAX_LASER_NUM], azimuth_list[DEFAULT_MAX_LASER_NUM];
    std::vector<std::string> vfirstLine;
    split_string(vfirstLine, line, ',');
    if (vfirstLine[0] == "EEFF" || vfirstLine[0] == "eeff") {
      // skip second line
      std::getline(ifs, line);  
      is_hav_eeff = 1;
    }

    int lineCount = 0;
    while (std::getline(ifs, line)) {
      std::vector<std::string> vLineSplit;
      split_string(vLineSplit, line, ',');
      // skip error line or hash value line

      if (vLineSplit.size() > 0 && vLineSplit[0].size() == 64) {
        hash = vLineSplit[0];
        SHA256_USE sha256;
        // 按行读取内容  
        int readCount = lineCount + 1 + is_hav_eeff;
        int i = 0;
        for (i = 0; i < len; i++) {
          if (correction_content[i] == '\n') {
            readCount--;
          }
          if (readCount <= 0) break;
        }
        sha256.update(correction_content, i + 1);
        uint8_t u8Hash[32];
        sha256.hexdigest(u8Hash);
        std::ostringstream oss;  
        for (size_t i = 0; i < sizeof(u8Hash); ++i) {  
            oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(u8Hash[i]);  
        }  
        std::string hashString = oss.str(); 
        if (hashString != hash) {
          LogWarning("correction file is invalid, hash is error, lineCount: %d", lineCount);
        }
        continue;
      }
      if (vLineSplit.size() != 3) {  
        continue;
      } else {
        lineCount++;
        if (lineCount > DEFAULT_MAX_LASER_NUM) {
          throw std::invalid_argument("invalid correction input file!(count)");
        }
      }
      float elevation, azimuth;
      int laserId = 0;
      laserId = std::stoi(vLineSplit[0]);
      elevation = std::stof(vLineSplit[1]);
      azimuth = std::stof(vLineSplit[2]);

      if (laserId > DEFAULT_MAX_LASER_NUM || laserId <= 0) {
        throw std::invalid_argument("laser id is wrong in correction file. laser Id: "   
                                      + std::to_string(laserId) + ", line: " + std::to_string(lineCount));
      }
      if (laserId != lineCount) {
        LogWarning("laser id is wrong in correction file. laser Id: %d, line: %d.  continue", laserId, lineCount);
        lineCount--;
        continue;
      }
      elevation_list[laserId - 1] = elevation;
      azimuth_list[laserId - 1] = azimuth;
    }

    for (int i = 0; i < lineCount; ++i) {
      this->correction.elevation[i] = elevation_list[i];
      this->correction.azimuth[i] = azimuth_list[i];
      correction.int_azimuth[i] = doubleToInt(correction.azimuth[i] * kAllFineResolutionFloat);
      correction.int_elevation[i] = doubleToInt(correction.elevation[i] * kAllFineResolutionFloat);
    }
    this->correction.hash = hash;
    this->get_correction_file_ = true;
  } catch (const std::exception &e) {
    LogFatal("load correction error: %s", e.what());
    this->get_correction_file_ = false;
    return -1;
  }
  return 0;
}

template <typename T_Point>
void Udp1_4Parser<T_Point>::LoadFiretimesFile(const std::string firetimes_path) {
  try {
    std::ifstream inFile(firetimes_path, std::ios::in);
    if (inFile.is_open()) {
      int uselessLine = 0;
      int count = 0;
      bool is_OT = false;
      std::string lineStr;

      //skip first line
      std::getline(inFile, lineStr); 
      int lineCount = 0;
      while (getline(inFile, lineStr)) {
        std::vector<std::string> vLineSplit;
        split_string(vLineSplit, lineStr, ',');
        // skip error line or hash value line
        if (vLineSplit.size() < 2) {  
          continue;
        } else {
          lineCount++;
        }
        float deltTime = 0.f;
        int laserId = 0;
        laserId = std::stoi(vLineSplit[0]);
        deltTime = std::stof(vLineSplit[1]);
        if (laserId > DEFAULT_MAX_LASER_NUM || laserId <= 0) {
          throw std::invalid_argument("laser id is wrong in firetimes file. laser Id: "   
                                      + std::to_string(laserId) + ", line: " + std::to_string(lineCount));
        }
        firetime_correction_[laserId - 1] = deltTime;
      }
    } else {
      throw std::invalid_argument("Open firetime file failed");
    }
    this->get_firetime_file_ = true;
    LogInfo("Open firetime file success!");
  } catch (const std::exception &e) {
    LogError("load firetimes error: %s", e.what());
    this->get_firetime_file_ = false;
  }
  return;
}

template <typename T_Point>
void* Udp1_4Parser<T_Point>::getStruct(const int type) {
  if (type == CORRECTION_STRUCT)
    return (void*)&(correction);
  else if (type == FIRETIME_STRUCT)
    return (void*)&firetime_correction_;
  return nullptr;
}

template<typename T_Point>
int Udp1_4Parser<T_Point>::DecodePacket_true(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) 
{
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ||
      udpPacket.buffer[2] != 1 || udpPacket.buffer[3] != 4) {
    LogDebug("Invalid point cloud");
    return -1;
  }
  const HS_LIDAR_HEADER_ME_V4 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_ME_V4 *>(
          udpPacket.buffer + sizeof(HS_LIDAR_PRE_HEADER));
  int unitSize = pHeader->unitSize();

  const auto *pTail = reinterpret_cast<const HS_LIDAR_TAIL_ME_V4 *>(
      (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4) +
      (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + unitSize * pHeader->GetLaserNum()) *
      pHeader->GetBlockNum() + sizeof(HS_LIDAR_BODY_CRC_ME_V4) +
      (hasFunctionSafety(pHeader->m_u8Status) ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4) : 0));
  // if (hasSeqNum(pHeader->m_u8Status)) {
  //   const HS_LIDAR_TAIL_SEQ_NUM_ME_V4 *pTailSeqNum =
  //       reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ME_V4 *>(
  //           (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4) +
  //           (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + unitSize * pHeader->GetLaserNum()) *
  //           pHeader->GetBlockNum() + sizeof(HS_LIDAR_BODY_CRC_ME_V4) +
  //           (hasFunctionSafety(pHeader->m_u8Status) ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4) : 0) +
  //           sizeof(HS_LIDAR_TAIL_ME_V4));
  // }
  const HS_LIDAR_TAIL_IMU_ME_V4 *pTailImu = 
    reinterpret_cast<const HS_LIDAR_TAIL_IMU_ME_V4 *>(
          (const unsigned char *)pTail + sizeof(HS_LIDAR_TAIL_ME_V4) + 
          (hasSeqNum(pHeader->m_u8Status) ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_ME_V4) : 0));

  frame.scan_complete = false;
  frame.block_num = pHeader->GetBlockNum();
  frame.laser_num = pHeader->GetLaserNum();
  if(frame.block_num > 2 || frame.laser_num > DEFAULT_MAX_LASER_NUM) {
    LogError("Invalid block number or laser number");
    return -1;
  }
  frame.per_points_num = frame.block_num * frame.laser_num;
  if (frame.fParam.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64(last_utc_time_);
  } else {
    frame.sensor_timestamp[frame.packet_num] = GetMicroTimeU64();
  }   
  if (frame.fParam.pcap_time_synchronization) frame.host_timestamp = GetMicroTickCountU64();

  if (frame.fParam.update_imu_flag) {
    frame.imu_config.timestamp = frame.sensor_timestamp[frame.packet_num] * 1.0 / kMicrosecondToSecond;
    frame.imu_config.imu_accel_x = pTailImu->GetIMUXAccel();  // g/s
    frame.imu_config.imu_accel_y = pTailImu->GetIMUYAccel();
    frame.imu_config.imu_accel_z = pTailImu->GetIMUZAccel();
    frame.imu_config.imu_ang_vel_x = pTailImu->GetIMUXAngVel(); // deg/s
    frame.imu_config.imu_ang_vel_y = pTailImu->GetIMUYAngVel();
    frame.imu_config.imu_ang_vel_z = pTailImu->GetIMUZAngVel();
  }

  uint16_t u16Azimuth = 0; 
  const HS_LIDAR_BODY_AZIMUTH_ME_V4 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4));
  u16Azimuth = pAzimuth->GetAzimuth();
  if (this->IsNeedFrameSplit(u16Azimuth, frame.fParam)) {
    frame.scan_complete = true;
  }
  if (u16Azimuth != last_azimuth_) {
    last_last_azimuth_ = last_azimuth_;
    last_azimuth_ = u16Azimuth;
  }
  if (frame.scan_complete) return 0;
  if (frame.fParam.use_cuda) {
    memcpy(frame.jt128_buffer[frame.packet_num].data, udpPacket.buffer, 1100);
    frame.packet_num++;
    return 0;
  }
  float dist_unit = pHeader->GetDistUnit();
  float speed = pTail->GetMotorSpeed() * 0.1;
  for (int i = 0; i < frame.block_num; i++) {
    pAzimuth =
        reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4) + 
            (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + unitSize * pHeader->GetLaserNum()) * i);
    u16Azimuth = pAzimuth->GetAzimuth();
    for (int j = 0; j < frame.laser_num; j++) {
      const HS_LIDAR_BODY_CHN_UNIT_ME_V4* pChnUnit = 
          reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_ME_V4 *>(
              (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) + 
              unitSize * j);
      int elevation = 0;
      int azimuth = u16Azimuth * kFineResolutionInt;
      float distance = static_cast<float>(pChnUnit->GetDistance() * dist_unit);
      if (get_firetime_file_) {
        azimuth += (frame.fParam.rotation_flag > 0 ? 1 : -1) * firetime_correction_[j] * speed * 6E-6 * kAllFineResolutionInt;
      }
      if (get_correction_file_) {
        int azimuth_coll = correction.int_azimuth[j];
        int elevation_corr = correction.int_elevation[j];
        if (frame.fParam.distance_correction_flag) {
          GetDistanceCorrection(optical_center, azimuth_coll, elevation_corr, distance, OpticalCenter);
        }
        elevation = elevation_corr;
        azimuth += azimuth_coll;
      } 
      CircleRevise(azimuth);
      CircleRevise(elevation);
      if (frame.fParam.config.fov_start != -1 && frame.fParam.config.fov_end != -1) {
        int fov_transfer = azimuth / kAllFineResolutionInt;
        if (fov_transfer < frame.fParam.config.fov_start || fov_transfer > frame.fParam.config.fov_end) { //不在fov范围continue
          continue;
        }
      }
      float xyDistance = distance * cos_all_angle_[(elevation)];
      float y = xyDistance * cos_all_angle_[(azimuth)];
      float x = xyDistance * sin_all_angle_[(azimuth)];
      float z = distance * sin_all_angle_[(elevation)];
      TransformPoint(x, y, z, frame.fParam.transform);

      auto& ptinfo = frame.points[frame.points_num];
      set_x(ptinfo, x);
      set_y(ptinfo, y);
      set_z(ptinfo, z);
      set_intensity(ptinfo, pChnUnit->GetReflectivity());
      set_timestamp(ptinfo, double(frame.sensor_timestamp[frame.packet_num]) / kMicrosecondToSecond);
      set_ring(ptinfo, j);
      if (hasWeightFactor(pHeader->m_u8Status)) set_weightFactor(ptinfo, pChnUnit->reserved[0]);
      frame.points_num++;
    }
  }
  frame.packet_num++;
  return 0;
} 

//-----------------------------------
template <typename T_Point>
void Udp1_4Parser<T_Point>::SetPcapPlay(int source_type) {
  source_type_ = source_type;
}
template <typename T_Point>
void Udp1_4Parser<T_Point>::SetFrameAzimuth(float frame_start_azimuth) {
  // Determine frame_start_azimuth [0,360)
  if (frame_start_azimuth < 0.0f || frame_start_azimuth >= 360.0f) {
    frame_start_azimuth = 0.0f;
  }
  frame_start_azimuth_uint16_ = frame_start_azimuth * kResolutionInt;
}
template <typename T_Point>
void Udp1_4Parser<T_Point>::CircleRevise(int &angle) {
  while (angle < 0) {
    angle += CIRCLE;
  }
  while (angle >= CIRCLE) {
    angle -= CIRCLE;
  }
}
template <typename T_Point>
uint32_t Udp1_4Parser<T_Point>::CRCCalc(const uint8_t *bytes, int len, int zeros_num) {
  CRCInit();
  uint32_t i_crc = 0xffffffff;
  for (int i = 0; i < len; i++)
      i_crc = (i_crc << 8) ^ m_CRCTable[((i_crc >> 24) ^ bytes[i]) & 0xff];
  for (int i = 0; i < zeros_num; i++)
      i_crc = (i_crc << 8) ^ m_CRCTable[((i_crc >> 24) ^ 0) & 0xff];
  return i_crc;
}

template <typename T_Point>
void Udp1_4Parser<T_Point>::CRCInit() {
  if (crc_initialized) {
      return;
  }
  crc_initialized = true;

  uint32_t i, j, k;
  for (i = 0; i < 256; i++) {
      k = 0;
      for (j = (i << 24) | 0x800000; j != 0x80000000; j <<= 1)
          k = (k << 1) ^ (((k ^ j) & 0x80000000) ? 0x04c11db7 : 0);
      m_CRCTable[i] = k;
  }
}
template<typename T_Point>
int Udp1_4Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) {
  if (udpPacket.packet_len < 6) {   // sizeof(HS_LIDAR_PRE_HEADER)
    frame.scan_complete = false;
    return -1;
  }
  int res = 0;
  do {
    if (!get_correction_file_) {
      if (printErrorBool) {
        LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
        printErrorBool = false;
      }
      res = -1;
      break;
    }
    if (frame.packet_num >= frame.maxPackerPerFrame) {
      LogFatal("frame.packet_num(%u) out of %d", frame.packet_num, frame.maxPackerPerFrame);
      res = -1;
      break;
    }
    res = DecodePacket_true(frame, udpPacket);
  } while(0);
  //data from pcap and play rate synchronize with the host time
  if (source_type_ == 2 && frame.fParam.pcap_time_synchronization == true && res == 0) {
    if(first_packet_ == true) {
      last_host_timestamp_ = frame.host_timestamp;
      last_sensor_timestamp_ = frame.sensor_timestamp[frame.packet_num - 1];
      packet_count_ = 1;
      first_packet_ = false;
    } else {
      packet_count_ += 1;
      if (packet_count_ >= kPcapPlaySynchronizationCount) {
        int reset_time = static_cast<int>((frame.sensor_timestamp[frame.packet_num - 1] - last_sensor_timestamp_) - (frame.host_timestamp - last_host_timestamp_));
        last_host_timestamp_ = frame.host_timestamp;
        last_sensor_timestamp_ = frame.sensor_timestamp[frame.packet_num - 1];
        packet_count_ = 0;
        if (reset_time > 0) {
          std::this_thread::sleep_for(std::chrono::microseconds(reset_time));
        }
      }
    }
  }
  return res;
}
template <typename T_Point>
void Udp1_4Parser<T_Point>::TransformPoint(float& x, float& y, float& z, const TransformParam& transform)
{
  if (transform.use_flag == false) return;

  float cosa = std::cos(transform.roll);
  float sina = std::sin(transform.roll);
  float cosb = std::cos(transform.pitch);
  float sinb = std::sin(transform.pitch);
  float cosc = std::cos(transform.yaw);
  float sinc = std::sin(transform.yaw);

  float x_ = cosb * cosc * x + (sina * sinb * cosc - cosa * sinc) * y +
              (sina * sinc + cosa * sinb * cosc) * z + transform.x;
  float y_ = cosb * sinc * x + (cosa * cosc + sina * sinb * sinc) * y +
              (cosa * sinb * sinc - sina * cosc) * z + transform.y;
  float z_ = -sinb * x + sina * cosb * y + cosa * cosb * z + transform.z;
  x = x_;
  y = y_;
  z = z_; 
}

template<typename T_Point>
bool Udp1_4Parser<T_Point>::IsNeedFrameSplit(uint16_t azimuth, FrameDecodeParam &param) {
  // The first two packet dont have the information of last_azimuth_  and last_last_azimuth, so do not need split frame
  // The initial value of last_azimuth_ is -1
  // Determine the rotation direction and division
  
  int32_t division = 0;
  // If last_last_azimuth_ != -1，the packet is the third, so we can determine whether the current packet requires framing
  if (this->last_last_azimuth_ != -1) 
  {
    // Get the division
    int32_t division1 = abs(this->last_azimuth_ - this->last_last_azimuth_);
    int32_t division2 = abs(this->last_azimuth_ - azimuth);
    division = division1 > division2 ? division2 : division1 ;
    // Prevent two consecutive packets from having the same angle when causing an error in framing
    if ( division == 0) return false;
    // In the three consecutive angle values, if the angle values appear by the division of the decreasing situation,it must be reversed
    // The same is true for FOV
    if( this->last_last_azimuth_ - this->last_azimuth_ == division || this->last_azimuth_ - azimuth == division)
    {
      param.UpdateRotation(-1);
    } else {
      param.UpdateRotation(1);
    }
  } else {
    // The first  and second packet do not need split frame
    return false;
  }
  if (param.rotation_flag > 0) {
    if (this->last_azimuth_- azimuth > division)
    {
      if (frame_start_azimuth_uint16_ > this->last_azimuth_ || frame_start_azimuth_uint16_ <= azimuth) {
        return true;
      } 
      return false;
    }  
    if (this->last_azimuth_ < azimuth && this->last_azimuth_ < frame_start_azimuth_uint16_ 
        && azimuth >= frame_start_azimuth_uint16_) {
      return true;
    }
    return false;
  } else {
    if (azimuth - this->last_azimuth_ > division)
    {
      if (frame_start_azimuth_uint16_ <= this->last_azimuth_ || frame_start_azimuth_uint16_ > azimuth) {
        return true;
      } 
      return false;
    }  
    if (this->last_azimuth_ > azimuth && this->last_azimuth_ > frame_start_azimuth_uint16_ 
        && azimuth <= frame_start_azimuth_uint16_) {
      return true;
    }
    return false;
  }
}

template <typename T_Point>
void Udp1_4Parser<T_Point>::GetDistanceCorrection(LidarOpticalCenter optical_center, int &azimuth, int &elevation, float &distance, DistanceCorrectionType type) {
  if (distance <= 0.09) return;
  CircleRevise(azimuth);
  CircleRevise(elevation);
  float tx = this->cos_all_angle_[elevation] * this->sin_all_angle_[azimuth];
  float ty = this->cos_all_angle_[elevation] * this->cos_all_angle_[azimuth];
  float tz = this->sin_all_angle_[elevation];
  float d  = distance;
  if (type == GeometricCenter) {
    float B = 2 * tx * optical_center.x + 2 * ty * optical_center.y + 2 * tz * optical_center.z;
    float C = optical_center.x * optical_center.x + optical_center.y * optical_center.y + optical_center.z * optical_center.z - d * d;
    float d_opitcal = std::sqrt(B * B / 4 - C) - B / 2;
    float x = d_opitcal * tx + optical_center.x;
    float y = d_opitcal * ty + optical_center.y;
    float z = d_opitcal * tz + optical_center.z;
    azimuth = static_cast<int>(std::atan(x / y) * kHalfCircleFloat * kFineResolutionFloat / M_PI);
    elevation = static_cast<int>(std::asin(z / d) * kHalfCircleFloat * kFineResolutionFloat / M_PI);
    distance = d;
  } else if (type == OpticalCenter) {
    float x = d * tx + optical_center.x;
    float y = d * ty + optical_center.y;
    float z = d * tz + optical_center.z;
    float d_geometric_center = std::sqrt(x * x + y * y + z * z);
    azimuth = static_cast<int>(std::atan(x / y) * kHalfCircleFloat * kFineResolutionFloat / M_PI);
    elevation = static_cast<int>(std::asin(z / d_geometric_center) * kHalfCircleFloat * kFineResolutionFloat / M_PI);
    distance = d_geometric_center;
  } else {
    // It should never have been executed here.
  }
  CircleRevise(azimuth);
  CircleRevise(elevation);
}

#endif