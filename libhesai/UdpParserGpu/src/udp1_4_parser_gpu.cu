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

#ifndef Udp1_4_PARSER_GPU_CU_
#define Udp1_4_PARSER_GPU_CU_
#include "udp1_4_parser_gpu.h"

using namespace hesai::lidar;
template <typename T_Point>
Udp1_4ParserGpu<T_Point>::Udp1_4ParserGpu(uint16_t maxPacket, uint16_t maxPoint) {
  this->optical_center.setNoFlag(LidarOpticalCenter{-0.0076, 0.01363, 0.01271});
  cudaSafeMalloc(correction_azi_cu_, sizeof(double) * DEFAULT_MAX_LASER_NUM);
  cudaSafeMalloc(correction_ele_cu_, sizeof(double) * DEFAULT_MAX_LASER_NUM);
  cudaSafeMalloc(firetime_correction_cu_, sizeof(float) * DEFAULT_MAX_LASER_NUM);
  if (maxPacket > 0 && maxPoint > 0) {
    cudaSafeMalloc(point_could_cu_, sizeof(JT128buffer) * maxPacket);
    cudaSafeMalloc(points_cu_, sizeof(LidarPointXYZAIW) * maxPacket * maxPoint);
    points_ = new LidarPointXYZAIW[maxPacket * maxPoint];
  } else {
    point_could_cu_ = nullptr;
    points_cu_ = nullptr;
    points_ = nullptr;
  }
}
template <typename T_Point>
Udp1_4ParserGpu<T_Point>::~Udp1_4ParserGpu() {
  if (point_could_cu_ != nullptr) cudaSafeFree(point_could_cu_);
  if (points_cu_ != nullptr) cudaSafeFree(points_cu_);
  if (points_ != nullptr) delete[] points_;
}

template <typename T_Point>
int Udp1_4ParserGpu<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame) {
  if (!this->get_correction_file_) return int(ReturnCode::CorrectionsUnloaded);       
  cudaSafeCall(cudaMemcpy(this->point_could_cu_, frame.jt128_buffer,
                          frame.packet_num * sizeof(JT128buffer), 
                          cudaMemcpyHostToDevice), ReturnCode::CudaMemcpyHostToDeviceError);
  FrameDecodeParam cuda_Param = frame.fParam;
  int ret = compute_1_4_cuda(this->points_cu_, this->correction_azi_cu_, this->correction_ele_cu_, 
    this->point_could_cu_, this->optical_center, cuda_Param, frame.packet_num, frame.per_points_num, 
    get_firetime_file_, firetime_correction_cu_);
  if (ret != 0) return ret;
  cudaSafeCall(cudaMemcpy(this->points_, this->points_cu_,
                          frame.per_points_num * frame.packet_num * sizeof(LidarPointXYZAIW), 
                          cudaMemcpyDeviceToHost), ReturnCode::CudaMemcpyDeviceToHostError);
  for (uint32_t i = 0; i < frame.packet_num; i++) {
    for (uint32_t j = 0; j < frame.per_points_num; j++) {
      if (correction_ptr->display[j] == false) continue;
      if (frame.fParam.config.fov_start != -1 && frame.fParam.config.fov_end != -1) {
        int fov_transfer = this->points_[j].azimuthCalib;
        if (fov_transfer < frame.fParam.config.fov_start || fov_transfer > frame.fParam.config.fov_end) { //不在fov范围continue
          continue;
        }
      }
      int offset = i * frame.per_points_num + j;
      auto& ptinfo = frame.points[frame.points_num];
      set_x(ptinfo, points_[offset].x);
      set_y(ptinfo, points_[offset].y);
      set_z(ptinfo, points_[offset].z);
      set_intensity(ptinfo, points_[offset].intensity);
      set_timestamp(ptinfo, double(frame.sensor_timestamp[i]) / kMicrosecondToSecond);
      set_ring(ptinfo, j);
      set_weightFactor(ptinfo, points_[offset].weightFactor);
      frame.points_num++;
    }
  }
  return 0;
}

template <typename T_Point>
void Udp1_4ParserGpu<T_Point>::LoadFiretimesStruct(void * _firetime) {
  firetime_ptr = (float*)_firetime;
  CUDACheck(cudaMemcpy(firetime_correction_cu_, firetime_ptr, sizeof(float) * DEFAULT_MAX_LASER_NUM, cudaMemcpyHostToDevice));
  get_firetime_file_ = true;
}
template <typename T_Point>
void Udp1_4ParserGpu<T_Point>::LoadCorrectionStruct(void* _correction) {
  correction_ptr = (CorrectionData*)_correction;
  CUDACheck(cudaMemcpy(correction_azi_cu_, correction_ptr->azimuth, sizeof(double) * DEFAULT_MAX_LASER_NUM, cudaMemcpyHostToDevice));
  CUDACheck(cudaMemcpy(correction_ele_cu_, correction_ptr->elevation, sizeof(double) * DEFAULT_MAX_LASER_NUM, cudaMemcpyHostToDevice));
  get_correction_file_ = true;
}


#endif // Udp1_4_PARSER_GPU_CU_