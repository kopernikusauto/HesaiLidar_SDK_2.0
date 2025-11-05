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
 * File:       lidar.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Define Lidar class 
 */

#ifndef Lidar_H
#define Lidar_H
#include <time.h>
#include "lidar_types.h"
#include "udp1_4_parser.h"
#include "pcap_source.h"
#include "blocking_ring.h"
#include "blocking_ptr_ring.h"
#include "ring.h"
#include "driver_param.h"
#include "socket_source.h"
#include "ptc_client.h"
#include "pcap_saver.h"
#ifndef _MSC_VER
#include <endian.h>
#include <semaphore.h>
#endif
#define AT128E2X_PACKET_LEN (1180)
#define GPS_PACKET_LEN (512)
#define CMD_SET_STANDBY_MODE (0x1c)
#define CMD_SET_SPIN_SPEED (0x17)

namespace hesai
{
namespace lidar
{

enum LidarInitFinishStatus {
  FaultMessParse = 0,
  PtcInitFinish = 1,
  PointCloudParse = 2,
  AllInitFinish = 3,
  FailInit = 4,

  TotalStatus
};

// class Lidar
// the Lidar class will start a udp data recieve thread and parser thread when init()
// udp packet data will be recived in udp data recieve thread and put into origin_packets_buffer_
template <typename T_Point>
class Lidar
{
public:
  ~Lidar();
  Lidar();
  Lidar(const Lidar &orig) = delete;
  Lidar& operator=(const Lidar&) = delete;
  // init lidar with param. init logger, udp parser, source, ptc client, start receive/parser thread
  int Init(const DriverParam& param);
  // get one packet from origin_packets_buffer_, 
  int GetOnePacket(UdpPacket **packet);
  void PopOnePacket();
  // start to record pcap file with name record_path,the record source is udp data recieve from source
  int StartRecordPcap(std::string record_path);
  // stop record pcap
  int StopRecordPcap();
  // control record pcap
  void EnableRecordPcap(bool bRecord);
  // start to record pcap file with name record_path,the record source is the parameter UdpFrame_t
  // port is the udp port recorded in pcap
  int SaveUdpPacket(const std::string &record_path, const UdpFrame_t &packets,
                    int port = 2368);
  // start to record pcap file with name record_path,the record source is the parameter UdpFrameArray_t
  // port is the udp port recorded in pcap                   
  int SaveUdpPacket(const std::string &record_path,
                    const UdpFrameArray_t &packets, int port = 2368);
  // covert a origin udp packet to decoded packet, the decode function is in UdpParser module
  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udp_packet);
  // get lidar correction file from ptc,and pass to udp parser
  int LoadCorrectionForUdpParser();
  // load correction file from Ros bag 
  int LoadCorrectionFromROSbag();
  // get lidar correction file from local file,and pass to udp parser 
  void LoadCorrectionFile(const std::string correction_path); 
  int LoadCorrectionString(const char *correction_string, const int len);
  // set the parser thread number
  void SetThreadNum(int thread_num);
  // get pcap status
  bool IsPlayEnded();
  
  void SetSource(Source **source);
  Udp1_4Parser<T_Point> *GetUdpParser();

  std::shared_ptr<PtcClient> ptc_client_;
  LidarDecodedFrame<T_Point> frame_;
  u8Array_t correction_string_;
  bool init_finish_[TotalStatus];           // 0: 基本初始化完成， 1：ptc初始化完成， 2：角度校准文件加载完成， 3：全部初始化完成
  BlockingPtrRing<UdpPacket, kPacketBufferSize> origin_packets_buffer_;
  std::shared_ptr<Source> source_;
private:
  std::shared_ptr<Udp1_4Parser<T_Point>> udp1_4parser_;
  std::shared_ptr<PcapSaver> pcap_saver_;
  std::shared_ptr<Source> source_rs232_;
  uint16_t udp_port_;
  // recieve udp data thread, this function will recieve udp data in while(),exit when variable running_ is false
  void RecieveUdpThread();
  // this variable is the exit condition of udp/parser thread
  bool running_;
  // this variable decide whether recieve_packet_thread will run
  bool udp_thread_running_;
  // this variable decide whether parser_thread will run
  bool parser_thread_running_;
  bool init_running;
  std::shared_ptr<std::thread> recieve_packet_thread_ptr_;
  uint32_t handle_buffer_size_;
  bool is_record_pcap_;
  bool is_timeout_ = false;

};
}  // namespace lidar
}  // namespace hesai

#include "lidar.cc"
#endif /* Lidar_H */


