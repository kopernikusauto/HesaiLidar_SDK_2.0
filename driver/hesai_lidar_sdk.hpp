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
#include "lidar.h"
#ifdef USE_CUDA
#include "udp1_4_parser_gpu.h"
#endif
namespace hesai
{
namespace lidar
{
template <typename T_Point>
class HesaiLidarSdk 
{
private:
  std::thread *runing_thread_ptr_;
  std::function<void(const UdpFrame_t&, double)> pkt_cb_;
  std::function<void(const LidarDecodedFrame<T_Point>&)> point_cloud_cb_;
  std::function<void(const u8Array_t&)> correction_cb_;
  std::function<void(const LidarImuData&)> imu_cb_;
  bool is_thread_runing_;
  LidarImuData last_imu_data_;
public:
  Lidar<T_Point> *lidar_ptr_;
#ifdef USE_CUDA
  Udp1_4ParserGpu<T_Point> *gpu_parser_ptr_;
#endif
  DriverParam param_;
  std::thread *init_thread_ptr_;
  SourceType source_type_;
  HesaiLidarSdk() {
#ifdef USE_CUDA
    std::cout << "-------- Hesai Lidar SDK Gpu V" << 2 << "." << 2 << "." << 2 << " --------" << std::endl;
#else
    std::cout << "-------- Hesai Lidar SDK V" << 2 << "." << 2 << "." << 2 << " --------" << std::endl;
#endif
    runing_thread_ptr_ = nullptr;
    lidar_ptr_ = nullptr;
    init_thread_ptr_ = nullptr;
    is_thread_runing_ = false;
    source_type_ = DATA_FROM_PCAP;
  }

  ~HesaiLidarSdk() {
    Stop();
  }

  void Init_thread()
  {
    //init lidar with param
    if (nullptr != lidar_ptr_) {
      lidar_ptr_->Init(param_);
      if (lidar_ptr_->init_finish_[FailInit]) return;
#ifdef USE_CUDA
      if (lidar_ptr_->frame_.fParam.use_cuda == false) {
      LogWarning("-------- param setup not to use GPU !!! --------");
      }
      gpu_parser_ptr_ = new Udp1_4ParserGpu<T_Point>(lidar_ptr_->frame_.maxPackerPerFrame, lidar_ptr_->frame_.maxPointPerPacket);
      if (lidar_ptr_->GetUdpParser()->isSetCorrectionSucc()) {
        gpu_parser_ptr_->LoadCorrectionStruct(lidar_ptr_->GetUdpParser()->getStruct(CORRECTION_STRUCT));
      }
      if (lidar_ptr_->GetUdpParser()->isSetFiretimesSucc()) {
        gpu_parser_ptr_->LoadFiretimesStruct(lidar_ptr_->GetUdpParser()->getStruct(FIRETIME_STRUCT));
      }
#else 
      lidar_ptr_->frame_.fParam.use_cuda = false;
      LogWarning("cpu version, not support for gpu");
#endif
      LogDebug("finish 3: Initialisation all complete !!!");
      lidar_ptr_->init_finish_[AllInitFinish] = true;
    }
  }

  //init lidar with param. init logger, udp parser, source, ptc client, start receive/parser thread
  bool Init(const DriverParam& param) 
  {
    /*****************************Init decoder******************************************************/ 
    lidar_ptr_ = new Lidar<T_Point>;
    if (nullptr == lidar_ptr_) {
      printf("create Lidar fail\n");
      return false;
    }
    source_type_ = param.input_param.source_type;
    param_ = param;
    init_thread_ptr_ = new std::thread(std::bind(&HesaiLidarSdk::Init_thread, this));
    /***********************************************************************************/ 
    return true;
  }

  // stop process thread
  void Stop() {
    if (nullptr != runing_thread_ptr_) {
      is_thread_runing_ = false;
      runing_thread_ptr_->join();
      delete runing_thread_ptr_;
      runing_thread_ptr_ = nullptr;
    }

    if (nullptr != lidar_ptr_) {
      delete lidar_ptr_;
      lidar_ptr_ = nullptr;
    } 

    if (nullptr != init_thread_ptr_) {
      init_thread_ptr_->join();
      delete init_thread_ptr_;
      init_thread_ptr_ = nullptr;
    } 
  }

  void onRelease() { is_thread_runing_ = false; }

  // start process thread
  void Start() {
    while(1) {
      if ((source_type_ == DATA_FROM_LIDAR && lidar_ptr_->init_finish_[FaultMessParse]) || lidar_ptr_->init_finish_[AllInitFinish]) {
        runing_thread_ptr_ = new std::thread(std::bind(&HesaiLidarSdk::Run, this));
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (lidar_ptr_->init_finish_[FailInit]) break;
    }
  }

  // process thread
  void Run()
  {
    LogInfo("--------begin to parse udp package--------");
    is_thread_runing_ = true;
    UdpFrame_t udp_packet_frame;
    uint32_t packet_index = 0;
    // auto start_time = std::chrono::high_resolution_clock::now();
    UdpPacket* packet;

    while (is_thread_runing_)
    {
      // auto end_time = std::chrono::high_resolution_clock::now();    
      //get one packte from origin_packets_buffer_, which receive data from upd or pcap thread
      int ret = lidar_ptr_->GetOnePacket(&packet);
      if (ret == -1) continue;

      // Wait for initialization to complete before starting to parse the point cloud
      if(!lidar_ptr_->init_finish_[AllInitFinish]) {
        lidar_ptr_->PopOnePacket();
        continue;
      }
      //get distance azimuth reflection, etc.and put them into decode_packet
      ret = lidar_ptr_->DecodePacket(lidar_ptr_->frame_, *packet);
      if (lidar_ptr_->frame_.fParam.update_imu_flag) {
        if (imu_cb_ && !last_imu_data_.isSameImuValue(lidar_ptr_->frame_.imu_config)) {
          imu_cb_(lidar_ptr_->frame_.imu_config);
          last_imu_data_ = lidar_ptr_->frame_.imu_config;
        }
      }
      if(ret != 0) {
        lidar_ptr_->PopOnePacket();
        continue;
      }
      // start_time = std::chrono::high_resolution_clock::now();

      //one frame is receive completely, split frame
      if(lidar_ptr_->frame_.scan_complete) {
        if (lidar_ptr_->frame_.fParam.use_cuda) {
#ifdef USE_CUDA
          ret = gpu_parser_ptr_->ComputeXYZI(lidar_ptr_->frame_);  
#endif
        }
        //log info, display frame message
        if (ret == 0 && lidar_ptr_->frame_.points_num > kMinPointsOfOneFrame) {
          double beg_timestamp = 0;
          double end_timestamp = 0;
          get_timestamp(lidar_ptr_->frame_.points[0], beg_timestamp);
          get_timestamp(lidar_ptr_->frame_.points[lidar_ptr_->frame_.points_num - 1], end_timestamp);
          LogInfo("frame:%d points:%u packet:%u start time:%lf end time:%lf", lidar_ptr_->frame_.frame_index, lidar_ptr_->frame_.points_num, 
                        lidar_ptr_->frame_.packet_num, beg_timestamp, end_timestamp) ;
          //publish point cloud topic
          if(point_cloud_cb_) point_cloud_cb_(lidar_ptr_->frame_);

          //publish upd packet topic
          if(pkt_cb_) {
            double timestamp = 0;
            get_timestamp(lidar_ptr_->frame_.points[0], timestamp);
            pkt_cb_(udp_packet_frame, timestamp);
          }

          if (correction_cb_ && lidar_ptr_->frame_.frame_index % 1000 == 1)
          {
            correction_cb_(lidar_ptr_->correction_string_);
          }
        }
        //reset frame variable
        lidar_ptr_->frame_.Update();
        //clear udp packet vector
        if (pkt_cb_) udp_packet_frame.clear();
        packet_index = 0;

        //if the packet which contains split frame msgs is valid, it will be the first packet of new frame
        lidar_ptr_->DecodePacket(lidar_ptr_->frame_, *packet);
        if (pkt_cb_) udp_packet_frame.emplace_back(*packet);
        packet_index++;
      }
      else {
        //new decoded packet of one frame, put it into decoded_packets_buffer_ and compute xyzi of points
        if (pkt_cb_) udp_packet_frame.emplace_back(*packet);
        packet_index++;

        //update status manually if start a new frame failedly
        if (packet_index >= lidar_ptr_->frame_.maxPackerPerFrame) {
          packet_index = 0;
          if (pkt_cb_) udp_packet_frame.clear();
          std::this_thread::sleep_for(std::chrono::microseconds(100));
          lidar_ptr_->frame_.Update();
          LogError("fail to start a new frame");
        }
      }
      lidar_ptr_->PopOnePacket();
    }
  } 
  // assign callback fuction
  void RegRecvCallback(const std::function<void(const LidarDecodedFrame<T_Point>&)>& callback) {
    point_cloud_cb_ = callback;
  }

  // assign callback fuction
  void RegRecvCallback(const std::function<void(const UdpFrame_t&, double)>& callback) {
    pkt_cb_ = callback;
  }

  // assign callback fuction
  void RegRecvCallback(const std::function<void (const u8Array_t&)>& callback) {
    correction_cb_ = callback;
  }
  void RegRecvCallback(const std::function<void (const LidarImuData&)>& callback) {
    imu_cb_ = callback;
  }
};

}  // namespace lidar
}  // namespace hesai
