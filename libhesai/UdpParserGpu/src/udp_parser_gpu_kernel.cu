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

#include "udp_parser_gpu_kernel.h"

namespace hesai
{
namespace lidar
{

__global__ void compute_xyzs_1_4_impl(LidarPointXYZAIW *xyzs, const double* correction_azi, const double* correction_ele, 
    const JT128buffer* point_data, const LidarOpticalCenter optical_center, const FrameDecodeParam fParam, const uint32_t per_points_num, 
    bool get_firetime_file, float* firetime_correction) {
  auto packet_index = blockIdx.x;
  auto channel = threadIdx.x;
  if (channel >= per_points_num) return;
  int point_index = packet_index * per_points_num + channel;
  const uint8_t* p_data = point_data[packet_index].data;
  p_data += 6;
  int lidar_num = *p_data;
  p_data += 1;
  int block_num = *p_data;
  p_data += 1;
  p_data += 1;
  float dist_unit = (*p_data) * 0.001;
  p_data += 1;
  p_data += 1;
  uint8_t flags = *p_data;
  p_data += 1;
  int unit_size = 3;
  if (flags & 0x20) unit_size += 1;
  auto tail = p_data + (2 + unit_size * lidar_num) * block_num + 4;
  int azimuth_num = (int)(channel / lidar_num);
  int channel_num = (int)(channel % lidar_num);
  p_data += (2 + unit_size * lidar_num) * azimuth_num;
  float azimuth = (p_data[0] + p_data[1] * 256) / 100.0;
  p_data += 2 + channel_num * unit_size;
  float rho = (p_data[0] + p_data[1] * 256) * dist_unit;
  p_data += 2;
  uint8_t intensity = *p_data;
  p_data += 1;
  float weight_factor = 0.0;
  if (flags & 0x20) {
    weight_factor = *p_data;
    p_data += 1;
  }

  azimuth = azimuth / HALF_CIRCLE * M_PI;
  if (get_firetime_file) {
    tail += 13;
    int speed = tail[0] + tail[1] * 256;
    azimuth += (fParam.rotation_flag > 0 ? 1 : -1) * firetime_correction[channel_num] * speed * 0.1 * 6E-6 / HALF_CIRCLE * M_PI;
  }
  float theta = correction_azi[channel_num] / HALF_CIRCLE * M_PI;
  float phi = correction_ele[channel_num] / HALF_CIRCLE * M_PI;

  if(rho > 0.09 && fParam.distance_correction_flag) {
    float tx = cos(phi) * sin(theta);
    float ty = cos(phi) * cos(theta);
    float tz = sin(phi);
    float d = rho;
    float x = d * tx + optical_center.x;
    float y = d * ty + optical_center.y;
    float z = d * tz + optical_center.z;
    float d_geometric_center = sqrt(x * x + y * y + z * z);
    theta = azimuth + atan(x / y);
    phi = asin(z / d_geometric_center);
    rho = d_geometric_center;
  } else {
    theta += azimuth;
  }

  float z = rho * sin(phi);
  auto r = rho * cosf(phi);
  float x = r * sin(theta);
  float y = r * cos(theta);
  
  if (fParam.transform.use_flag) {
    auto& transform = fParam.transform;
    float cosa = cos(transform.roll);
    float sina = sin(transform.roll);
    float cosb = cos(transform.pitch);
    float sinb = sin(transform.pitch);
    float cosc = cos(transform.yaw);
    float sinc = sin(transform.yaw);

    float x_ = cosb * cosc * x + (sina * sinb * cosc - cosa * sinc) * y +
                (sina * sinc + cosa * sinb * cosc) * z + transform.x;
    float y_ = cosb * sinc * x + (cosa * cosc + sina * sinb * sinc) * y +
                (cosa * sinb * sinc - sina * cosc) * z + transform.y;
    float z_ = -sinb * x + sina * cosb * y + cosa * cosb * z + transform.z;

    x = x_;
    y = y_;
    z = z_;
  }

  xyzs[point_index].x = x;
  xyzs[point_index].y = y;
  xyzs[point_index].z = z;
  xyzs[point_index].intensity = intensity;
  xyzs[point_index].azimuthCalib = theta * HALF_CIRCLE / M_PI;
  xyzs[point_index].weightFactor = weight_factor;
}

int compute_1_4_cuda(LidarPointXYZAIW* points, const double* correction_azi, const double* correction_ele,
  const JT128buffer* point_data, const LidarOpticalCenter optical_center, const FrameDecodeParam fParam, 
  const uint32_t packet_num, const uint32_t per_points_num, bool get_firetime_file, float* firetime_correction) {
    compute_xyzs_1_4_impl<<<packet_num, per_points_num>>>(points, correction_azi, correction_ele,
      point_data, optical_center, fParam, per_points_num, get_firetime_file, firetime_correction);
    cudaDeviceSynchronize();
    cudaSafeCall(cudaGetLastError(), ReturnCode::CudaXYZComputingError);
  return 0;
}


}
}