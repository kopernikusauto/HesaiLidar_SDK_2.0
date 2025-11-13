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
 * File:       udp1_4_parser.h
 * Author:     Chang XingShuo <changxingshuo@hesaitech.com>
 * Description: Declare Udp1_4Parser class
*/

#ifndef UDP1_4_PARSER_H_
#define UDP1_4_PARSER_H_
#include "lidar_types.h"
#include <iostream>
#include <iomanip>
#include "udp_protocol_v1_4.h"
#include "general_parser.h"
namespace hesai
{
namespace lidar
{
#define DEFAULT_MAX_LASER_NUM (128)

#ifndef DEFINE_SET_GET
#define DEFINE_SET_GET(member, Type)                                                                                   \
  template <typename T_Point>                                                                                          \
  inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, member)>::type set_##member(T_Point& point, const Type& value) \
  {                                                                                                                    \
  }                                                                                                                    \
  template <typename T_Point>                                                                                          \
  inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, member)>::type set_##member(T_Point& point, const Type& value) \
  {                                                                                                                    \
      point.member = value;                                                                                            \
  }                                                                                                                    \
  template <typename T_Point>                                                                                          \
  inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, member)>::type get_##member(T_Point& point, Type& value)  \
  {                                                                                                                    \
  }                                                                                                                    \
  template <typename T_Point>                                                                                          \
  inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, member)>::type get_##member(T_Point& point, Type& value)  \
  {                                                                                                                    \
      value = point.member;                                                                                            \
  }
#endif

DEFINE_MEMBER_CHECKER(weightFactor)
DEFINE_SET_GET(x, float)  
DEFINE_SET_GET(y, float)  
DEFINE_SET_GET(z, float)  
DEFINE_SET_GET(intensity, float)  
DEFINE_SET_GET(ring, uint16_t)  
DEFINE_SET_GET(timestamp, double)  
DEFINE_SET_GET(weightFactor, uint8_t)


enum StructType {
  CORRECTION_STRUCT = 1,
  FIRETIME_STRUCT = 2
};

enum DistanceCorrectionType {
  OpticalCenter,
  GeometricCenter,
};

struct CorrectionData {
  double elevation[DEFAULT_MAX_LASER_NUM];
  double azimuth[DEFAULT_MAX_LASER_NUM];
  int int_elevation[DEFAULT_MAX_LASER_NUM];
  int int_azimuth[DEFAULT_MAX_LASER_NUM];
  bool display[DEFAULT_MAX_LASER_NUM];
  std::string hash;
  CorrectionData() {
    memset(elevation, 0, sizeof(double) * DEFAULT_MAX_LASER_NUM);
    memset(azimuth, 0, sizeof(double) * DEFAULT_MAX_LASER_NUM);
    for (int i = 0; i < DEFAULT_MAX_LASER_NUM; ++i) {  
        display[i] = true;
    } 
    hash = "";
  }
};
// class Udp1_4Parser
// parsers packets and computes points for JT16
template<typename T_Point>
class Udp1_4Parser {
 public:
  Udp1_4Parser();
  virtual ~Udp1_4Parser();

  int DecodePacket_true(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);    
  int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket);    

  void LoadFiretimesFile(const std::string firetimes_path);
  void LoadCorrectionFile(const std::string lidar_correction_file);
  int LoadCorrectionString(const char *correction_string, const int len);
  // get the pointer to the struct of the parsed correction file or firetimes file
  void* getStruct(const int type);
  

  // set the parsing type
  void SetPcapPlay(int source_type);
  // set frame azimuth
  void SetFrameAzimuth(float frame_start_azimuth);
  void CircleRevise(int &angle);
  void TransformPoint(float& x, float& y, float& z, const TransformParam& transform);
  // crc
  void CRCInit();
  uint32_t CRCCalc(const uint8_t *bytes, int len, int zeros_num);
  // compute optical center correction
  void GetDistanceCorrection(LidarOpticalCenter optical_center, int &azimuth, int &elevation, float &distance, DistanceCorrectionType type);
  // determine whether to frame based on azimuth
  bool IsNeedFrameSplit(uint16_t azimuth, FrameDecodeParam &param);
  bool isSetCorrectionSucc() { return get_correction_file_; }
  bool isSetFiretimesSucc() { return get_firetime_file_; }
  
 private:
  int source_type_;
  uint16_t frame_start_azimuth_uint16_;
  LidarOpticalCenter optical_center;
  float firetime_correction_[DEFAULT_MAX_LASER_NUM] = {95.18,23.24,98.22,20.2,101.26,17.16,104.3,14.12,77.28,92.14,74.24,89.1,71.2,86.06,68.16,83.02,50.26,11.08,47.22,8.04,44.18,5,41.14,1.96,65.12,105.82,62.08,102.78,59.04,99.74,56,96.7,38.1,24.76,35.06,21.72,32.02,18.68,28.98,15.64,78.8,93.66,75.76,90.62,72.72,87.58,69.68,84.54,51.78,12.6,48.74,9.56,45.7,6.52,42.66,3.48,66.64,103.54,63.6,100.5,60.56,97.46,57.52,94.42,39.62,22.48,36.58,19.44,33.54,16.4,30.5,13.36,76.52,91.38,73.48,88.34,70.44,85.3,67.4,82.26,49.5,10.32,46.46,7.28,43.42,4.24,40.38,1.2,64.36,105.06,61.32,102.02,58.28,98.98,55.24,95.94,37.34,24,34.3,20.96,31.26,17.92,28.22,14.88,78.04,92.9,75,89.86,71.96,86.82,68.92,83.78,51.02,11.84,47.98,8.8,44.94,5.76,41.9,2.72,65.88,62.84,59.8,56.76,38.86,35.82,32.78,29.74};
  bool get_firetime_file_{true};
  CorrectionData correction;
  bool get_correction_file_;
  int32_t last_azimuth_;
  int32_t last_last_azimuth_;
  float cos_all_angle_[CIRCLE];
  float sin_all_angle_[CIRCLE];
  // synchronization
  bool first_packet_;
  uint64_t last_host_timestamp_;
  uint64_t last_sensor_timestamp_;
  uint8_t packet_count_;
  bool printErrorBool;
  uint32_t m_CRCTable[256];
  bool crc_initialized;
  LastUtcTime last_utc_time_;
};
}  // namespace lidar
}  // namespace hesai

#include "udp1_4_parser.cc"

#endif  // UDP1_4_PARSER_H_
