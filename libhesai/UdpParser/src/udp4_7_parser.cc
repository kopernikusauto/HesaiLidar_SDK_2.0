/*
This file is mainly a concrete implementation of the UDP4 parser.
*/

#include "udp4_7_parser.h"
#include "udp_protocol_v4_7.h"
#include "udp_protocol_header.h"
#include "udp_parser.h"

using namespace hesai::lidar;
template<typename T_Point>
Udp4_7Parser<T_Point>::Udp4_7Parser() {
  this->get_correction_file_ = false;
  for (int i = 0; i < 512; i++) {
    even_firetime_correction_[i] = 0;
    odd_firetime_correction_[i] = 0;
  }
}

template<typename T_Point>
Udp4_7Parser<T_Point>::~Udp4_7Parser() { LogInfo("release Udp4_7Parser"); }

template<typename T_Point>
int16_t Udp4_7Parser<T_Point>::GetVecticalAngle(int channel) {
  if (this->get_correction_file_ == false) {
    LogError("GetVecticalAngle: no correction file get, Error");
    return -1;
  }
  return m_ATX_corrections.elevation[channel];
}

template<typename T_Point>
bool Udp4_7Parser<T_Point>::IsNeedFrameSplit(uint16_t frame_id) {
  // printf("%d %d\n",frame_id,  this->last_frameid_);
  if ( frame_id != this->last_frameid_  && this->last_frameid_ >= 0) {
      return true;
  }
  return false;
}

template<typename T_Point>
void Udp4_7Parser<T_Point>::LoadCorrectionFile(std::string lidar_correction_file) {
  int ret = 0;
  LogInfo("load correction file from local correction.dat now!");
  std::ifstream fin(lidar_correction_file);
  if (fin.is_open()) {
    LogDebug("Open correction file success");
    int length = 0;
    std::string str_lidar_calibration;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    str_lidar_calibration = buffer;
    ret = LoadCorrectionString(buffer);
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local Correction file Error");
    } else {
      LogInfo("Parse local Correction file Success!!!");
    }
  } else {
    LogError("Open correction file failed");
    return;
  }
}

template<typename T_Point>
int Udp4_7Parser<T_Point>::LoadCorrectionString(char *data) {
  try {
    char *p = data;
    ATXCorrectionsHeader header = *(ATXCorrectionsHeader *)p;
    if (0xee == header.delimiter[0] && 0xff == header.delimiter[1]) {
      switch (header.version[1]) {
        case 1: {
          m_ATX_corrections.header = header;
          auto channel_num = m_ATX_corrections.header.channel_number;
          uint16_t division = m_ATX_corrections.header.angle_division;
          p += sizeof(ATXCorrectionsHeader);
          if (channel_num > ATX_LASER_NUM || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&m_ATX_corrections.raw_azimuths, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ATX_corrections.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;

          for (int i = 0; i < channel_num; i++) {
            m_ATX_corrections.azimuth[i] = ((float)(m_ATX_corrections.raw_azimuths[i])) / (float)division;
            m_ATX_corrections.elevation[i] = ((float)(m_ATX_corrections.raw_elevations[i])) / (float)division;
            // printf("%d %f %f %d\n", i, m_ATX_corrections.azimuth[i], m_ATX_corrections.elevation[i], division);
          } 
          memcpy((void*)&m_ATX_corrections.SHA_value, p, 32);
          this->get_correction_file_ = true;
          return 0;
        } break;
        case 2: {
          m_ATX_corrections.header = header;
          auto channel_num = m_ATX_corrections.header.channel_number;
          uint16_t division = m_ATX_corrections.header.angle_division;
          p += sizeof(ATXCorrectionsHeader);
          if (channel_num > ATX_LASER_NUM || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&m_ATX_corrections.raw_azimuths_even, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;       
          memcpy((void *)&m_ATX_corrections.raw_azimuths_odd, p,
                 sizeof(int16_t) * channel_num);       
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ATX_corrections.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;

          for (int i = 0; i < channel_num; i++) {
            m_ATX_corrections.azimuth_even[i] = ((float)(m_ATX_corrections.raw_azimuths_even[i])) / (float)division;
            m_ATX_corrections.azimuth_odd[i] = ((float)(m_ATX_corrections.raw_azimuths_odd[i])) / (float)division;
            m_ATX_corrections.elevation[i] = ((float)(m_ATX_corrections.raw_elevations[i])) / (float)division;
            // printf("%d %f %f %f %d\n", i, m_ATX_corrections.azimuth_even[i], m_ATX_corrections.azimuth_odd[i],  m_ATX_corrections.elevation[i], division);
          } 
          memcpy((void*)&m_ATX_corrections.SHA_value, p, 32);
          this->get_correction_file_ = true;
          return 0;
        } break;
        case 3: {
          m_ATX_corrections.header = header;
          auto channel_num = m_ATX_corrections.header.channel_number;
          uint16_t division = m_ATX_corrections.header.angle_division;
          p += sizeof(ATXCorrectionsHeader);
          if (channel_num > ATX_LASER_NUM || division == 0) {
            LogError("data error: channel_num is %u, division is %u", channel_num, division);
            return -1;
          }
          memcpy((void *)&m_ATX_corrections.raw_azimuths_even, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;       
          memcpy((void *)&m_ATX_corrections.raw_azimuths_odd, p,
                 sizeof(int16_t) * channel_num);       
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ATX_corrections.raw_elevations, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_ATX_corrections.raw_elevations_adjust, p,
                 sizeof(int16_t) * m_ATX_corrections.kLenElevationAdjust);
          p += sizeof(int16_t) * m_ATX_corrections.kLenElevationAdjust;

          for (int i = 0; i < channel_num; i++) {
            m_ATX_corrections.azimuth_even[i] = ((float)(m_ATX_corrections.raw_azimuths_even[i])) / (float)division;
            m_ATX_corrections.azimuth_odd[i] = ((float)(m_ATX_corrections.raw_azimuths_odd[i])) / (float)division;
            m_ATX_corrections.elevation[i] = ((float)(m_ATX_corrections.raw_elevations[i])) / (float)division;
            // printf("%d %f %f %f %d\n", i, m_ATX_corrections.azimuth_even[i], m_ATX_corrections.azimuth_odd[i],  m_ATX_corrections.elevation[i], division);
          } 
          for (uint32_t i = 0; i < m_ATX_corrections.kLenElevationAdjust; i++) {
            m_ATX_corrections.elevation_adjust[i] = ((float)(m_ATX_corrections.raw_elevations_adjust[i])) / (float)division;
          }
          memcpy((void*)&m_ATX_corrections.SHA_value, p, 32);
          this->get_correction_file_ = true;
          return 0;
        }
        default:
          break;
      }
    }
    return -1;
  } catch (const std::exception &e) {
    LogFatal("load correction error: %s", e.what());
    return -1;
  }
  return -1;
}

template <typename T_Point>
void Udp4_7Parser<T_Point>::LoadFiretimesFile(std::string firetimes_path) {
  std::ifstream inFile(firetimes_path, std::ios::in);
  if (inFile.is_open()) {
    std::string lineStr;
    //skip first line
    std::getline(inFile, lineStr); 
    while (getline(inFile, lineStr)) {
      std::stringstream ss(lineStr);
      std::string index, deltTime1, deltTime2;
      std::getline(ss, index, ',');
      std::getline(ss, deltTime1, ',');
      std::getline(ss, deltTime2, ',');
      // std::cout << index << "  "  << deltTime1 << " " << deltTime2 << std::endl;
      even_firetime_correction_[std::stoi(index) - 1] = std::stod(deltTime1);
      odd_firetime_correction_[std::stoi(index) - 1] = std::stod(deltTime2);
      
    }
    this->get_firetime_file_ = true;
    LogInfo("Open firetime file success!");
    inFile.close();
    return;
  } else {
    LogWarning("Open firetime file failed");
    this->get_firetime_file_ = false;
    return;
  }
}

template<typename T_Point>
int Udp4_7Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  for (int blockid = 0; blockid < frame.block_num; blockid++) {
    // T_Point point;
    int Azimuth = 0;
    auto elevation = 0;
    int azimuth = 0;

    for (int i = 0; i < frame.laser_num; i++) {
      int point_index = packet_index * frame.per_points_num + blockid * frame.laser_num + i;  
      float distance = frame.distances[point_index] * frame.distance_unit;
      Azimuth = frame.azimuth[point_index];
      if (this->get_correction_file_) {
        azimuth = (CIRCLE + Azimuth) % CIRCLE;
        if(m_ATX_corrections.header.version[1] == 1 || m_ATX_corrections.header.version[1] == 2) {
          elevation = m_ATX_corrections.elevation[i] * kAllFineResolutionInt;
        } 
        else if (m_ATX_corrections.header.version[1] == 3) {
          elevation = m_ATX_corrections.elevation[i] * kAllFineResolutionInt + m_ATX_corrections.ElevationAdjust(azimuth) * kAllFineResolutionInt;
        }
        elevation = (CIRCLE + elevation) % CIRCLE;
      }      
      if (frame.config.fov_start != -1 && frame.config.fov_end != -1)
      {
        int fov_transfer = azimuth / 256 / 100;
        if (fov_transfer < frame.config.fov_start || fov_transfer > frame.config.fov_end){//不在fov范围continue
          continue;
        }
      }
      float xyDistance = distance * this->cos_all_angle_[(elevation)];
      float x = xyDistance * this->sin_all_angle_[(azimuth)];
      float y = xyDistance * this->cos_all_angle_[(azimuth)];
      float z = distance * this->sin_all_angle_[(elevation)];
      this->TransformPoint(x, y, z);
      setX(frame.points[point_index], x);
      setY(frame.points[point_index], y);
      setZ(frame.points[point_index], z);
      setIntensity(frame.points[point_index], frame.reflectivities[point_index]);
      setConfidence(frame.points[point_index], frame.confidence[point_index]);
      setTimestamp(frame.points[point_index], double(frame.sensor_timestamp[packet_index]) / kMicrosecondToSecond);
      setRing(frame.points[point_index], i);
    }
  }
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
int Udp4_7Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket) {
  if (!this->get_correction_file_) {
    static bool printErrorBool = true;
    if (printErrorBool) {
      LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
      printErrorBool = false;
    }
    return -1;
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }

  const HS_LIDAR_HEADER_ST_V7 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_ST_V7 *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));
  
  const HS_LIDAR_TAIL_ST_V7 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_ST_V7 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V7) +
          (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7) +
           sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7) +
           sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V7) * pHeader->GetLaserNum()) *
              pHeader->GetBlockNum() +
          sizeof(HS_LIDAR_BODY_CRC_ST_V7));

  if (pHeader->HasSeqNum()) {
    const HS_LIDAR_TAIL_SEQ_NUM_ST_V7 *pTailSeqNum =
        reinterpret_cast<const HS_LIDAR_TAIL_SEQ_NUM_ST_V7 *>(
            (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V7) +
            (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7) +
             sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7) +
             sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V7) * pHeader->GetLaserNum()) *
                pHeader->GetBlockNum() +
            sizeof(HS_LIDAR_BODY_CRC_ST_V7) + sizeof(HS_LIDAR_TAIL_ST_V7));
    uint32_t packet_seqnum = pTailSeqNum->m_u32SeqNum;
    this->CalPktLoss(packet_seqnum);
  }
  uint64_t packet_timestamp = pTail->GetMicroLidarTimeU64();
  this->CalPktTimeLoss(packet_timestamp);
  frame.host_timestamp = GetMicroTickCountU64();
  // const HS_LIDAR_E2E_HEADER_ST_V7 *pE2EHeader = 
  //     reinterpret_cast<const HS_LIDAR_E2E_HEADER_ST_V7 *>(
  //           (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V7) +
  //           (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7) +
  //            sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7) +
  //            sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V7) * pHeader->GetLaserNum()) *
  //               pHeader->GetBlockNum() +
  //           sizeof(HS_LIDAR_BODY_CRC_ST_V7) + sizeof(HS_LIDAR_TAIL_ST_V7)
  //           + (pHeader->HasSeqNum() ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_ST_V7) : 0));
                            
  if (frame.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
  } else {
    frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
  }
  this->spin_speed_ = pTail->GetMotorSpeed();
  this->is_dual_return_= pTail->IsDualReturn();
  this->return_mode_ = pTail->GetReturnMode();
  frame.spin_speed = pTail->GetMotorSpeed();
  frame.return_mode = pTail->GetReturnMode();
  frame.per_points_num = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  frame.scan_complete = false;
  frame.distance_unit = pHeader->GetDistUnit();
  int index = frame.packet_num * pHeader->GetBlockNum() * pHeader->GetLaserNum();
  frame.block_num = pHeader->GetBlockNum();
  frame.laser_num = pHeader->GetLaserNum();
  const HS_LIDAR_BODY_AZIMUTH_ST_V7 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ST_V7 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V7));
  /* The following comment fields are not used */
  // const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7 *pFineAzimuth =
  //     reinterpret_cast<const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7 *>(
  //         (const unsigned char *)pAzimuth +
  //         sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7));
  const HS_LIDAR_BODY_CHN_NNIT_ST_V7 *pChnUnit =
      reinterpret_cast<const HS_LIDAR_BODY_CHN_NNIT_ST_V7 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7) +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7));



  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    int32_t u32Azimuth = pAzimuth->GetAzimuth();
    // uint8_t u8FineAzimuth = pFineAzimuth->GetFineAzimuth();
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_NNIT_ST_V7 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7) +
        sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7));

    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ST_V7 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7) +
        sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7) +
        sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V7) * pHeader->GetLaserNum());
    // pFineAzimuth = reinterpret_cast<const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V7 *>(
    //     (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V7));

    int azimuth = (u32Azimuth) * (kAllFineResolutionFloat  / m_ATX_corrections.header.angle_division);
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      if (m_ATX_corrections.header.version[1] == 1) {
        frame.azimuth[index] =  azimuth + m_ATX_corrections.azimuth[i] * kAllFineResolutionFloat;
      }
      else if (m_ATX_corrections.header.version[1] == 2 || m_ATX_corrections.header.version[1] == 3) {
        frame.azimuth[index] =  azimuth + ((pTail->GetFrameID() % 2 == 0) ? m_ATX_corrections.raw_azimuths_even[i] : m_ATX_corrections.raw_azimuths_odd[i] ) * kAllFineResolutionFloat / m_ATX_corrections.header.angle_division;
      }
      // printf("%d  %f\n", m_ATX_corrections.header.version[1], output.azimuth[index] / 256);
      if (this->get_firetime_file_) {
        frame.azimuth[index] =  frame.azimuth[index] + ((pTail->GetFrameID() % 2 == 0) ? even_firetime_correction_[i] : -odd_firetime_correction_[i] ) * (abs(pTail->GetMotorSpeed()) * 1E-9 / 8) * kAllFineResolutionFloat;
      }
      frame.reflectivities[index] = pChnUnit->GetReflectivity();  
      frame.distances[index] = pChnUnit->GetDistance();
      frame.confidence[index] = pChnUnit->GetConfidenceLevel();
      pChnUnit = pChnUnit + 1;
      index++;
    }
    // 分帧标记位为该字节的最后一个bit
    uint8_t frameID = pTail->GetFrameID() % 2;
    if (this->use_angle_ && IsNeedFrameSplit(frameID)) {
      frame.scan_complete = true;
    }
    this->use_angle_ = true;
    this->last_frameid_ = frameID;
    this->last_azimuth_ = u32Azimuth;
  }
  frame.packet_num++;
  return 0;
}

template<typename T_Point>
void Udp4_7Parser<T_Point>::ParserFaultMessage(UdpPacket& udp_packet, FaultMessageInfo &fault_message_info) {
  FaultMessageVersion4_7 *fault_message_ptr =  
      reinterpret_cast< FaultMessageVersion4_7*> (&(udp_packet.buffer[0]));
  fault_message_ptr->ParserFaultMessage(fault_message_info);
  return;
}