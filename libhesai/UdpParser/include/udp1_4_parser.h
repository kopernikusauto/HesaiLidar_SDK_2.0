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
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Declare Udp1_4Parser class
*/

#ifndef UDP1_4_PARSER_H_
#define UDP1_4_PARSER_H_

#include "general_parser.h"
#include "udp_protocol_v1_4.h"
namespace hesai
{
namespace lidar
{

// class Udp1_4Parser
// parsers packets and computes points for PandarN E3X、OT128
template<typename T_Point>
class Udp1_4Parser : public GeneralParser<T_Point> {
 public:
  Udp1_4Parser(std::string);
  virtual ~Udp1_4Parser();

  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket, const int packet_index = -1);    
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, uint32_t packet_index);

  virtual void LoadFiretimesFile(const std::string& firetimes_path);
  virtual int LoadCorrectionString(const char *correction_string, int len);
  // compute lidar firetime correciton
  double GetFiretimesCorrection(int laserId, double speed, uint8_t optMode, uint8_t angleState, float dist);
  // get the pointer to the struct of the parsed correction file or firetimes file
  virtual void* getStruct(const int type);
  virtual void setFrameRightMemorySpace(LidarDecodedFrame<T_Point> &frame);
 private:
  int GetFiretimes(int laserId, uint8_t optMode, uint8_t angleState, float dist);
  pandarN::FiretimesPandarN firetimes;
  // firetime corrections for JT128
  float firetime_correction_[128] = {95.18,23.24,98.22,20.2,101.26,17.16,104.3,14.12,77.28,92.14,74.24,89.1,71.2,86.06,68.16,83.02,50.26,11.08,47.22,8.04,44.18,5,41.14,1.96,65.12,105.82,62.08,102.78,59.04,99.74,56,96.7,38.1,24.76,35.06,21.72,32.02,18.68,28.98,15.64,78.8,93.66,75.76,90.62,72.72,87.58,69.68,84.54,51.78,12.6,48.74,9.56,45.7,6.52,42.66,3.48,66.64,103.54,63.6,100.5,60.56,97.46,57.52,94.42,39.62,22.48,36.58,19.44,33.54,16.4,30.5,13.36,76.52,91.38,73.48,88.34,70.44,85.3,67.4,82.26,49.5,10.32,46.46,7.28,43.42,4.24,40.38,1.2,64.36,105.06,61.32,102.02,58.28,98.98,55.24,95.94,37.34,24,34.3,20.96,31.26,17.92,28.22,14.88,78.04,92.9,75,89.86,71.96,86.82,68.92,83.78,51.02,11.84,47.98,8.8,44.94,5.76,41.9,2.72,65.88,62.84,59.8,56.76,38.86,35.82,32.78,29.74};
};
}  // namespace lidar
}  // namespace hesai

#include "udp1_4_parser.cc"

#endif  // UDP1_4_PARSER_H_
