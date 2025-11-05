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
 * File:       lidar_type.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: 
 */

#ifndef LIDAR_TYPES_H_
#define LIDAR_TYPES_H_
#include <stdint.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <algorithm>
#include <functional>
#include <memory>
#include <atomic>
#include <array>
#include <cmath>  
#include <fstream>  
#include "driver_param.h"
namespace hesai
{
namespace lidar
{
//max packet num of one frame, it means the capacity of frame buffer
static constexpr uint16_t kMaxPacketNumPerFrame = 5000;
//half of the max value in degrees, 1 LSB represents 0.01 degree, float type
static constexpr float kHalfCircleFloat = 18000.0f;
//half of the max value in degrees, 1 LSB represents 0.01 degree, int type
static constexpr int kHalfCircleInt = 18000;
//max value in degrees
static constexpr float kCircle = 360.f;
//laser azimuth resolution, 100 units represents 1 degree, int type
static constexpr int kResolutionInt = 100;
//laser azimuth resolution, 100 units represents 1 degree, float type
static constexpr float kResolutionFloat = 100.0f;
//conversion factor between second and micorsecond
static constexpr float kMicrosecondToSecond = 1000000.0f;
static constexpr int kMicrosecondToSecondInt = 1000000;
static constexpr int kMicrosecondToNanosecondInt = 1000;
//laser fine azimuth resolution, 1 LSB represents 0.01 / 256 degree, float type
static constexpr float kFineResolutionFloat = 1.0f;
static constexpr float kAllFineResolutionFloat = kResolutionFloat * kFineResolutionFloat;
//laser fine azimuth resolution, 1 LSB represents 0.01 / 256 degree, int type
static constexpr int kFineResolutionInt = 1;
static constexpr int kAllFineResolutionInt = kResolutionInt * kFineResolutionInt;
//synchronize host time with sensor time per kPcapPlaySynchronizationCount packets
static constexpr int kPcapPlaySynchronizationCount = 1;
//length of fault message packet
static constexpr int kFaultMessageLength = 99;
//default udp data max lenth
static const uint16_t kBufSize = 1500;
static const uint16_t kPacketBufferSize = 36000;
//min points of one frame for displaying frame message
static constexpr int kMinPointsOfOneFrame = 1000;
//max time interval between two frame
static constexpr int kMaxTimeInterval = 250000;

static constexpr int CIRCLE = (360 * kAllFineResolutionFloat);

#pragma pack(push, 1)
struct LidarPointXYZI
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;     
};

struct LidarPointXYZIRT
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;  
    uint16_t ring;
    double timestamp;  
};

struct LidarPointXYZICRT
{
    float x; 
    float y;             
    float z;             
    uint8_t intensity;  
    uint8_t confidence;  
    uint16_t ring;
    double timestamp;  
};

struct LidarPointXYZAIW
{
    float x; 
    float y;             
    float z;             
    float azimuthCalib;
    uint8_t intensity;     
    uint8_t weightFactor;
};

struct LidarDecodeConfig {
    int fov_start;
    int fov_end;

    LidarDecodeConfig() {
      fov_start = -1;
      fov_end = -1;
    }
};

struct LidarImuData {
  double timestamp; 
  double imu_accel_x;
  double imu_accel_y;
  double imu_accel_z;
  double imu_ang_vel_x;
  double imu_ang_vel_y;
  double imu_ang_vel_z;

  LidarImuData() {
    timestamp = 0;
    imu_accel_x = -1;
    imu_accel_y = -1;
    imu_accel_z = -1;
    imu_ang_vel_x = -1;
    imu_ang_vel_y = -1;
    imu_ang_vel_z = -1;
  }
  bool isSameImuValue(const LidarImuData& other) const {
    return imu_accel_x == other.imu_accel_x &&
           imu_accel_y == other.imu_accel_y &&
           imu_accel_z == other.imu_accel_z &&
           imu_ang_vel_x == other.imu_ang_vel_x &&
           imu_ang_vel_y == other.imu_ang_vel_y &&
           imu_ang_vel_z == other.imu_ang_vel_z;
  }
};

struct MonitorValue {
    uint16_t value;
    bool valid;
    MonitorValue() {
      value = 0;
      valid = false;
    }
    MonitorValue(uint16_t _value, bool _valid) {
      value = _value;
      valid = _valid;
    }
  };
struct LidarMonitorInfo {
  bool isUpdate;
  uint8_t TDM_version;
  MonitorValue monitor_info1_[256];
  MonitorValue monitor_info2_[256];
  MonitorValue monitor_info3_[256];
  LidarMonitorInfo() {
    isUpdate = false;
  }
};

struct LidarOpticalCenter {
  float x;
  float y;
  float z;
  LidarOpticalCenter() {
    x = 0;
    y = 0;
    z = 0;
  }
  LidarOpticalCenter(float _x, float _y, float _z) {
    x = _x;
    y = _y;
    z = _z;
  }
  void setNoFlag(LidarOpticalCenter other) {
    x = other.x;
    y = other.y;
    z = other.z;
  }
  LidarOpticalCenter& operator=(LidarOpticalCenter&) = delete;  
};

struct JT128buffer {
  uint8_t data[1100];
};

struct FrameDecodeParam {
  uint8_t use_timestamp_type;  // 0: point cloud time, other: local time
  bool pcap_time_synchronization; 
  bool distance_correction_flag;
  bool xt_spot_correction;
  bool update_imu_flag;  // update imu flag
  uint8_t echo_mode_filter;
  LidarDecodeConfig config;
  TransformParam transform;
  int rotation_flag;
  bool enable_packet_loss_tool_;
  bool enable_packet_timeloss_tool_;
  bool packet_timeloss_tool_continue_;
  bool use_cuda;
  FrameDecodeParam() {
    use_timestamp_type = 0;
    pcap_time_synchronization = false;
    distance_correction_flag = false;
    xt_spot_correction = false;
    echo_mode_filter = 0;
    rotation_flag = 0;
    enable_packet_loss_tool_ = false;
    enable_packet_timeloss_tool_ = false;
    packet_timeloss_tool_continue_ = false;
    use_cuda = false;
    update_imu_flag = true;
  }
  void UpdateRotation(int rotation) {
    if (abs(rotation_flag) == 10240) return;
    if (abs(rotation_flag) > 128 * 8) rotation_flag /= 128;
    rotation_flag += rotation;
  }
  void Init(const DriverParam& param) {
    use_timestamp_type = param.decoder_param.use_timestamp_type;
    pcap_time_synchronization = param.decoder_param.pcap_play_synchronization;
    distance_correction_flag = param.decoder_param.distance_correction_flag;
    config.fov_start = param.decoder_param.fov_start;
    config.fov_end = param.decoder_param.fov_end;
    transform = param.decoder_param.transform_param;
    use_cuda = param.use_gpu;
  }
};
#pragma pack(pop)

template <typename PointT>
class LidarDecodedFrame
{
    public:
    LidarDecodedFrame(uint16_t maxPacketNum = 5000, uint16_t maxNumPerPacket = 512) {
        resetMalloc(maxPacketNum, maxNumPerPacket);
        lidar_state = -1;
        work_mode = -1;
        packet_num = 0;
        frame_index = 0;
        host_timestamp = 0;
        block_num = 0;
        laser_num = 0; 
        return_mode = 0;
        points_num = 0;
        per_points_num = 0;
        scan_complete = false;
    };
    ~LidarDecodedFrame() {
        if (total_memory) {
            delete[] total_memory;
            total_memory = nullptr;
            points = nullptr;
            jt128_buffer = nullptr;
            sensor_timestamp = nullptr;
        }
    }
    void resetMalloc(uint16_t maxPacketNum, uint16_t maxNumPerPacket) {
      maxPackerPerFrame = maxPacketNum;
      maxPointPerPacket = maxNumPerPacket;
      if (total_memory) {
          delete[] total_memory;
          total_memory = nullptr;
          points = nullptr;
          jt128_buffer = nullptr;
          sensor_timestamp = nullptr;
      }
      total_memory = new uint8_t[sizeof(PointT) * maxPackerPerFrame * maxPointPerPacket +
                                sizeof(JT128buffer) * maxPackerPerFrame + 
                                sizeof(uint64_t) * maxPackerPerFrame];
      uint32_t offset = 0;
      points = reinterpret_cast <PointT* >(total_memory + offset);
      offset += (sizeof(PointT) * maxPackerPerFrame * maxPointPerPacket);
      jt128_buffer = reinterpret_cast <JT128buffer* >(total_memory + offset);
      offset += (sizeof(JT128buffer) * maxPackerPerFrame);
      sensor_timestamp = reinterpret_cast <uint64_t* >(total_memory + offset);
      offset += (sizeof(uint64_t) * maxPackerPerFrame);
    }
    LidarDecodedFrame(const LidarDecodedFrame&) = delete;
    LidarDecodedFrame& operator=(const LidarDecodedFrame&) = delete;
    void Update() {
        packet_num = 0;
        block_num = 0;
        laser_num = 0; 
        points_num = 0;
        per_points_num = 0;
        scan_complete = false;
        frame_index++;
    }
    uint32_t getPointSize() {
      return sizeof(PointT);
    }
    uint8_t* total_memory = nullptr; 
    uint16_t maxPackerPerFrame;
    uint16_t maxPointPerPacket;
    // configure
    FrameDecodeParam fParam;

    // frame parameter
    int16_t lidar_state;
    int16_t work_mode;
    uint16_t return_mode;
    uint32_t packet_num; 
    int frame_index;
    uint32_t points_num;
    JT128buffer* jt128_buffer;
    uint64_t* sensor_timestamp;
    // package parameter
    uint64_t host_timestamp;
    LidarMonitorInfo monitor_info;
    uint16_t block_num;
    uint16_t laser_num;
    uint16_t per_points_num;
    bool scan_complete;
    // point parameter
    PointT* points = nullptr;
    //special output
    LidarImuData imu_config;
};

struct UdpPacket {
  uint8_t buffer[1500];
  uint16_t packet_len;
  bool is_timeout = false;
  uint64_t recv_timestamp;
  uint32_t ip;
  uint16_t port;
  UdpPacket(const uint8_t* data = nullptr, uint16_t sz = 0, uint64_t tm = 0, 
            uint32_t i_ip = 0, uint16_t i_port = 0)
  : packet_len(sz), recv_timestamp(tm), ip(i_ip), port(i_port)
  {
    memset(buffer, 0, 1500);
    if(data != nullptr)
      memcpy(buffer, data, packet_len);
  }
};

typedef std::vector<uint8_t> u8Array_t;
typedef std::vector<uint16_t> u16Array_t;
typedef std::vector<uint32_t> u32Array_t;
typedef std::vector<double> doubleArray_t;
typedef std::vector<u8Array_t> u8ArrayArray_t;
typedef std::vector<doubleArray_t> doubleArrayArray_t;
typedef std::vector<u16Array_t> u16ArrayArray_t;
typedef std::vector<std::string> stringArray_t;
typedef std::vector<UdpPacket> UdpFrame_t;
typedef std::vector<UdpFrame_t> UdpFrameArray_t;

class SHA256_USE {  
public:  
    SHA256_USE() {  
        // Initialize the hash values  
        state = {{  
            0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,  
            0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19  
        }};  
        bufferIndex = 0;  
        bitcount = 0;  
    }  

    void update(const char* data, size_t length) {  
        for (size_t i = 0; i < length; i++) {  
            buffer[bufferIndex++] = data[i];  
            if (bufferIndex == 64) {  
                transform(buffer.data());  
                bufferIndex = 0;  
            }  
            bitcount += 8;  
        }  
    }  

    void hexdigest(uint8_t *hash) {  
        pad();  
        for (auto data : state) {
          *hash = (data >> 24) & 0xFF;
          *(hash + 1) = (data >> 16) & 0xFF;
          *(hash + 2) = (data >> 8) & 0xFF;
          *(hash + 3) = (data >> 0) & 0xFF;
          hash += sizeof(uint32_t);
        }
    }  

private:  
    void transform(const uint8_t* data) {  
        // Prepare the message schedule  
        std::array<uint32_t, 64> w = {};  
        for (size_t i = 0; i < 16; i++) {  
            w[i] = ((uint32_t)data[i * 4] << 24) | ((uint32_t)data[i * 4 + 1] << 16) |  
                   ((uint32_t)data[i * 4 + 2] << 8) | (uint32_t)data[i * 4 + 3];  
        }  

        for (size_t i = 16; i < 64; i++) {  
            uint32_t s0 = (w[i - 15] >> 7) | (w[i - 15] << (32 - 7));  
            s0 ^= (w[i - 15] >> 18) | (w[i - 15] << (32 - 18));  
            s0 ^= (w[i - 15] >> 3);  
            uint32_t s1 = (w[i - 2] >> 17) | (w[i - 2] << (32 - 17));  
            s1 ^= (w[i - 2] >> 19) | (w[i - 2] << (32 - 19));  
            s1 ^= (w[i - 2] >> 10);  
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;  
        }  

        // Initialize working variables to current hash value  
        uint32_t a = state[0];  
        uint32_t b = state[1];  
        uint32_t c = state[2];  
        uint32_t d = state[3];  
        uint32_t e = state[4];  
        uint32_t f = state[5];  
        uint32_t g = state[6];  
        uint32_t h = state[7];  

        // Compression function main loop  
        for (size_t i = 0; i < 64; i++) {  
            uint32_t S1 = (e >> 6) | (e << (32 - 6));  
            S1 ^= (e >> 11) | (e << (32 - 11));  
            S1 ^= (e >> 25) | (e << (32 - 25));  
            uint32_t ch = (e & f) ^ (~e & g);  
            uint32_t temp1 = h + S1 + ch + k[i] + w[i];  
            uint32_t S0 = (a >> 2) | (a << (32 - 2));  
            S0 ^= (a >> 13) | (a << (32 - 13));  
            S0 ^= (a >> 22) | (a << (32 - 22));  
            uint32_t maj = (a & b) ^ (a & c) ^ (b & c);  
            uint32_t temp2 = S0 + maj;  

            h = g;  
            g = f;  
            f = e;  
            e = d + temp1;  
            d = c;  
            c = b;  
            b = a;  
            a = temp1 + temp2;  
        }  

        // Add the compressed chunk to the current hash value  
        state[0] += a;  
        state[1] += b;  
        state[2] += c;  
        state[3] += d;  
        state[4] += e;  
        state[5] += f;  
        state[6] += g;  
        state[7] += h;  
    }  

    void pad() {  
        buffer[bufferIndex++] = 0x80;  
        if (bufferIndex > 56) {  
            while (bufferIndex < 64) {  
                buffer[bufferIndex++] = 0x00;  
            }  
            transform(buffer.data());  
            bufferIndex = 0;  
        }  
        while (bufferIndex < 56) {  
            buffer[bufferIndex++] = 0x00;  
        }  
        // Append the bits count  
        for (int i = 0; i < 8; i++) {  
            buffer[56 + i] = (bitcount >> (56 - i * 8)) & 0xFF;  
        }  
        transform(buffer.data());  
    }  

    std::array<uint32_t, 8> state;  
    std::array<uint8_t, 64> buffer;  
    uint64_t bitcount;  
    size_t bufferIndex;  

    const std::array<uint32_t, 64> k = {  
      0x428a2f98UL, 0x71374491UL, 0xb5c0fbcfUL, 0xe9b5dba5UL, 0x3956c25bUL,
      0x59f111f1UL, 0x923f82a4UL, 0xab1c5ed5UL, 0xd807aa98UL, 0x12835b01UL,
      0x243185beUL, 0x550c7dc3UL, 0x72be5d74UL, 0x80deb1feUL, 0x9bdc06a7UL,
      0xc19bf174UL, 0xe49b69c1UL, 0xefbe4786UL, 0x0fc19dc6UL, 0x240ca1ccUL,
      0x2de92c6fUL, 0x4a7484aaUL, 0x5cb0a9dcUL, 0x76f988daUL, 0x983e5152UL,
      0xa831c66dUL, 0xb00327c8UL, 0xbf597fc7UL, 0xc6e00bf3UL, 0xd5a79147UL,
      0x06ca6351UL, 0x14292967UL, 0x27b70a85UL, 0x2e1b2138UL, 0x4d2c6dfcUL,
      0x53380d13UL, 0x650a7354UL, 0x766a0abbUL, 0x81c2c92eUL, 0x92722c85UL,
      0xa2bfe8a1UL, 0xa81a664bUL, 0xc24b8b70UL, 0xc76c51a3UL, 0xd192e819UL,
      0xd6990624UL, 0xf40e3585UL, 0x106aa070UL, 0x19a4c116UL, 0x1e376c08UL,
      0x2748774cUL, 0x34b0bcb5UL, 0x391c0cb3UL, 0x4ed8aa4aUL, 0x5b9cca4fUL,
      0x682e6ff3UL, 0x748f82eeUL, 0x78a5636fUL, 0x84c87814UL, 0x8cc70208UL,
      0x90befffaUL, 0xa4506cebUL, 0xbef9a3f7UL, 0xc67178f2UL 
    };   
};  

}  // namespace lidar
}  // namespace hesai

 
#endif // LIDAR_TYPES_H_
