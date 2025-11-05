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
#ifndef Lidar_CC
#define Lidar_CC
#include "lidar.h"
#include <inttypes.h>
#include <stdio.h>
#include <chrono>
using namespace hesai::lidar;

template <typename T_Point>
Lidar<T_Point>::Lidar() {
  udp1_4parser_ = std::make_shared<Udp1_4Parser<T_Point>>();
  pcap_saver_ = std::make_shared<PcapSaver>();
  frame_.resetMalloc(kMaxPacketNumPerFrame, 256);
  is_record_pcap_ = false;
  running_ = true;
  init_running = true;
  udp_thread_running_ = true;
  parser_thread_running_ = true;
  handle_buffer_size_ = kPacketBufferSize;
  std::fill(init_finish_, init_finish_ + TotalStatus, false);
}

template <typename T_Point>
Lidar<T_Point>::~Lidar() {
  running_ = false;
  init_running = false;
  udp_thread_running_ = false;
  parser_thread_running_ = false;
  if (recieve_packet_thread_ptr_) {
    recieve_packet_thread_ptr_->join();
    recieve_packet_thread_ptr_.reset();
  }

  Logger::GetInstance().Stop();
}

template <typename T_Point>
int Lidar<T_Point>::Init(const DriverParam& param) {
    int res = -1;
    /*******************************Init log*********************************************/
    Logger::GetInstance().SetFileName(param.log_path.c_str());
    Logger::GetInstance().setLogTargetRule(param.log_Target);
    Logger::GetInstance().setLogLevelRule(param.log_level);
    // Logger::GetInstance().bindLogCallback(logCallback);
    Logger::GetInstance().Start(); 
    /**********************************************************************************/
    if (param.input_param.source_type == 1) {
      source_ = std::make_shared<SocketSource>(param.input_param.udp_port, param.input_param.multicast_ip_address);
      source_->Open();
    }
    /***************************Init source****************************************/
    udp_port_ = param.input_param.udp_port;
    if (param.input_param.source_type == 2) {
      int packet_interval = 10;
      source_ = std::make_shared<PcapSource>(param.input_param.pcap_path, packet_interval);
      if(source_->Open() == false) {
        init_finish_[FailInit] = true;
        return -1;
      }
    }
    else if(param.input_param.source_type == 4){
      LogFatal("Serial source not supported");
      init_finish_[FailInit] = true;
      return -1;
    }
    parser_thread_running_ = param.decoder_param.enable_parser_thread;
    udp_thread_running_ = param.decoder_param.enable_udp_thread;
    if (param.decoder_param.socket_buffer_size > 0) {
      source_->SetSocketBufferSize(param.decoder_param.socket_buffer_size);
    }

    frame_.fParam.Init(param);
    SetThreadNum(param.decoder_param.thread_num);
    /********************************************************************************/
    if (param.input_param.source_type == 1 && param.input_param.is_use_ptc) {
      ptc_client_ = std::make_shared<PtcClient>(param.input_param.device_ip_address
                                                  , param.input_param.ptc_port
                                                  , false
                                                  , 1
                                                  , 2000
                                                  , 2000);
    }
    init_finish_[FaultMessParse] = true;
    LogDebug("finish 0: The basic initialisation is complete");
    
    /***************************Init decoder****************************************/   
    udp1_4parser_->SetPcapPlay(param.input_param.source_type);
    udp1_4parser_->SetFrameAzimuth(param.decoder_param.frame_start_azimuth);
    switch (param.input_param.source_type)
    {
    case 1:
      if (param.input_param.is_use_ptc) {
        while (ptc_client_ != nullptr && (!ptc_client_->IsOpen()) && init_running) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (LoadCorrectionForUdpParser() == -1) {
          LogError("---Failed to obtain correction file from lidar!---");
          LoadCorrectionFile(param.input_param.correction_file_path);
        }
      }
      else {
        LoadCorrectionFile(param.input_param.correction_file_path);
      }
      break;
    case 2:
      LoadCorrectionFile(param.input_param.correction_file_path);
      break;
    case 3:
      LoadCorrectionFile(param.input_param.correction_file_path);
      break;
    case 4:
      break;
    default:
      break;
    }
    init_finish_[PointCloudParse] = true;
    LogDebug("finish 2: The angle calibration file is finished loading");
    /********************************************************************************/
    res = 0;
    return res;
}

template <typename T_Point>
int Lidar<T_Point>::GetOnePacket(UdpPacket **packet) {
  if (origin_packets_buffer_.try_get_front_ptr(packet))  return 0;
  else  return -1;
}

template <typename T_Point>
void Lidar<T_Point>::PopOnePacket() {
  origin_packets_buffer_.pop_front_ptr();
}

template <typename T_Point>
int Lidar<T_Point>::StartRecordPcap(std::string record_path) {
  pcap_saver_->SetPcapPath(record_path);
  EnableRecordPcap(true);
  pcap_saver_->Save();
  return 0;
}

template <typename T_Point>
int Lidar<T_Point>::StopRecordPcap() {
  EnableRecordPcap(false);
  pcap_saver_->close();
  return 0;
}

template <typename T_Point>
int Lidar<T_Point>::LoadCorrectionForUdpParser() {
  if (ptc_client_ == nullptr) return -1;
  u8Array_t sData;
  if (ptc_client_->GetCorrectionInfo(sData) != 0) {
    LogWarning("LoadCorrectionForUdpParser get correction info fail");
    return -1;
  }
  correction_string_ = sData;
  if (udp1_4parser_) {
    return udp1_4parser_->LoadCorrectionString(
        (char *)sData.data(), sData.size());
  } else {
    LogError("udp_parser_ nullptr");
    return -1;
  }
  return 0;
}

template <typename T_Point>
void Lidar<T_Point>::EnableRecordPcap(bool bRecord) {
  is_record_pcap_ = bRecord;
}

template <typename T_Point>
int Lidar<T_Point>::SaveUdpPacket(const std::string &record_path,
                              const UdpFrame_t &packets, int port) {
  return pcap_saver_->Save(record_path, packets, port);
}

template <typename T_Point>
int Lidar<T_Point>::SaveUdpPacket(const std::string &record_path,
                              const UdpFrameArray_t &packets, int port) {
  return pcap_saver_->Save(record_path, packets, port);
}

template <typename T_Point>
int Lidar<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udp_packet) {
  return udp1_4parser_->DecodePacket(frame, udp_packet);
} 

template <typename T_Point>
int Lidar<T_Point>::LoadCorrectionFromROSbag() {

  return udp1_4parser_->LoadCorrectionString(
        (char *)correction_string_.data(), correction_string_.size());
  return 0;
}

template <typename T_Point>
void Lidar<T_Point>::LoadCorrectionFile(const std::string correction_path) {
  udp1_4parser_->LoadCorrectionFile(correction_path);
  return;
}

template <typename T_Point>
int Lidar<T_Point>::LoadCorrectionString(const char *correction_string, const int len) {
  return udp1_4parser_->LoadCorrectionString(correction_string, len);
}

template <typename T_Point>
void Lidar<T_Point>::RecieveUdpThread() {
  if(!udp_thread_running_) return;
  // uint32_t u32StartTime = GetMicroTickCount();
  LogInfo("Lidar::Recieve Udp Thread start to run");
#ifdef _MSC_VER
  SetThreadPriorityWin(THREAD_PRIORITY_TIME_CRITICAL);
#else
  SetThreadPriority(SCHED_FIFO, SHED_FIFO_PRIORITY_MEDIUM);
#endif
  while (running_) {
    if (!source_) {
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
      continue;
    }
    while(origin_packets_buffer_.full() && running_) std::this_thread::sleep_for(std::chrono::microseconds(1000));
    if(running_ == false) break;

    UdpPacket* udp_packet = origin_packets_buffer_.get_back_next_ptr();
    int len = source_->Receive(*udp_packet, kBufSize);
    if (len == -1) {
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
      continue;
    }
    switch (len) {
      case GPS_PACKET_LEN:
        break;
      default :
        if (len > 0) {
          udp_packet->packet_len = static_cast<uint16_t>(len);
          origin_packets_buffer_.push_back_ptr();
        }
        break;
    }
    if (udp_packet->packet_len > 0 && is_record_pcap_) {
        pcap_saver_->Dump(udp_packet->buffer, udp_packet->packet_len, udp_port_);
    }
  }
  return;
}

template <typename T_Point>
void Lidar<T_Point>::SetThreadNum(int nThreadNum) {
  running_ = false;

  if (recieve_packet_thread_ptr_ != nullptr) {
    recieve_packet_thread_ptr_->join();
    recieve_packet_thread_ptr_.reset();
  }

  running_ = true;

  recieve_packet_thread_ptr_ =
      std::make_shared<std::thread>(std::bind(&Lidar::RecieveUdpThread, this)); 
}

template <typename T_Point>
bool Lidar<T_Point>::IsPlayEnded()
{
  if (!source_)
  {
    return false;
  }
  return source_->is_pcap_end;
}

template <typename T_Point>
void Lidar<T_Point>::SetSource(Source **source) {
  source_.reset(*source);
}

template <typename T_Point>
Udp1_4Parser<T_Point> *Lidar<T_Point>::GetUdpParser() { return udp1_4parser_.get(); }

#endif