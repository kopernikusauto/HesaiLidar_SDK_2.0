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
 * File:   serial_source.cc
 * Author: chang xingshuo<changxingshuo@hesaitech.com>
 *
 * Created on Sep 10, 2024, 19:56 PM  
 */

#include "serial_source.h"
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <chrono>

using namespace hesai::lidar;
SerialSource::SerialSource(const std::string dev, int baudrate, int point_cloud_baudrate) {
  m_iFd = -1;
  dev_ = dev;
  baudrate_ = baudrate;
  serialData = new uint8_t[kDataMaxLength];
  serialData_backup = new uint8_t[kDataMaxLength];
  dataIndex = 0;
  dataLength = 0;
  point_cloud_baudrate_ = point_cloud_baudrate;
  running = false;
  runningRecvThreadPtr = nullptr;
}

SerialSource::~SerialSource() {
  Close();
  delete[] serialData;
  delete[] serialData_backup;
}

bool SerialSource::Open() {
  int baudrate = 0;
  if (receiveStype == SERIAL_POINT_CLOUD_RECV) 
    baudrate = point_cloud_baudrate_;
  else if (receiveStype == SERIAL_COMMAND_RECV)
    baudrate = baudrate_;
  else {
    LogError("not support receiveStype, open error");
    return false;
  }
  if (Open(dev_.c_str(), baudrate) == 0) {
    if (is_need_recv_) {
      running = true;
      runningRecvThreadPtr = new std::thread(std::bind(&SerialSource::ReceivedThread, this));
    }
    return true;
  }
  return false;
}

bool SerialSource::IsOpened() {
  if (m_iFd >= 0) {
    return true;
  }
  return false;
}

int SerialSource::Open(const char *dev, int baudrate) {
  Close();
  /* Open SerialSource port */
  if ((m_iFd = open(dev, O_RDWR | O_NOCTTY)) < 0) {
    // perror("open");
    return -1;
  }

  struct termios2 tio { };
  if (0 != ioctl(m_iFd, TCGETS2, &tio)) {
    // perror("ioctl");
    close(m_iFd);
    m_iFd= -1;
    return -1;
  }
  /**
  * Setting serial port,8E2, non-blocking.100Kbps
  */
  tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
          | IXON);
  tio.c_iflag |= (INPCK | IGNPAR);
  tio.c_oflag &= ~OPOST;
  tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tio.c_cflag &= ~(CSIZE | CBAUD);
  /**
  * use BOTHER to specify speed directly in c_[io]speed member
  */
  tio.c_cflag |= (CS8 | CLOCAL | BOTHER | CREAD);
  tio.c_cflag &= ~(CSTOPB | PARENB);
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;
  if (0 != ioctl(m_iFd, TCSETS2, &tio)) {
    // perror("ioctl");
    close(m_iFd);
    m_iFd= -1;
    return -1;
  }
  return 0;
}

void SerialSource::Close() {
  if (runningRecvThreadPtr != nullptr) {
    running = false;
    runningRecvThreadPtr->join();
    delete runningRecvThreadPtr;
    runningRecvThreadPtr = nullptr;
    dataIndex = 0;
    dataLength = 0;
  }
  if (m_iFd >= 0) {
    close(m_iFd);
    m_iFd = -1;
  }
}

int SerialSource::WaitRead(int32_t timeout) {
  int ret;
  fd_set rfds;
  struct timeval tv_timeout;

  FD_ZERO(&rfds);
  FD_SET(m_iFd, &rfds);

  if (timeout >= 0) {
    tv_timeout.tv_sec = timeout / 1000;
    tv_timeout.tv_usec = (timeout % 1000) * 1000;

    ret = select(m_iFd + 1, &rfds, NULL, NULL, &tv_timeout);
  } else {
    ret = select(m_iFd + 1, &rfds, NULL, NULL, NULL);
  }

  if (ret < 0) {
    return ret;
  } else if (ret == 0) ///< Timeout / nothing more to read
  {
    return 0;
  }

  return 1;
}

int SerialSource::Receive(UdpPacket& udpPacket, uint16_t u16Len, int flags, int timeout) {
  bool ret = false;
  while (ret == false && timeout >= 0) {
    if (flags == SERIAL_POINT_CLOUD_RECV) {
      ret = pointCloudRecvBuf.try_pop_front(udpPacket);
    } else if (flags == SERIAL_COMMAND_RECV) {
      ret = cmdAckRecvBuf.try_pop_front(udpPacket);
    }
    timeout -= 100;
  }
  if (ret) return udpPacket.packet_len;
  return -1;
}

int SerialSource::Send(uint8_t *u8Buf, uint16_t u16Len, int flags) {
  if (m_iFd < 0)
    return -EINVAL;

  return write(m_iFd, u8Buf, u16Len);
}

int SerialSource::Flush() {
  if (m_iFd >= 0)
    return tcflush(m_iFd, TCIOFLUSH);
  return 0;
}

int SerialSource::FlushInput() {
  if (m_iFd >= 0)
    return tcflush(m_iFd, TCIFLUSH);
  return 0;
}

int SerialSource::FlushOutput() {
  if (m_iFd >= 0)
    return tcflush(m_iFd, TCOFLUSH);
  return 0;
}

void SerialSource::SetReceiveStype(int type) {
  if (type == SERIAL_CLEAR_RECV_BUF) {
    cmdAckRecvBuf.eff_clear();
    return;
  }
  Close();
  cmdAckRecvBuf.eff_clear();
  pointCloudRecvBuf.eff_clear();
  FlushInput();
  receiveStype = type;
}

void SerialSource::ReceivedThread() {
  while (running) {
    int ret = 0;
    if (m_iFd < 0)
      return;

    /* Wait until data coming in*/
    ret = WaitRead(1);
    if (ret > 0) {
      if (dataLength + kOneRecvLength > kDataMaxLength) {
        memcpy(serialData_backup, serialData + dataIndex, dataLength - dataIndex);
        memcpy(serialData, serialData_backup, dataLength - dataIndex);
        dataLength = dataLength - dataIndex;
        dataIndex = 0;
      }
      if (dataLength + kOneRecvLength <= kDataMaxLength) {
        ret = read(m_iFd, serialData + dataLength, kOneRecvLength);
        if (ret > 0) {
          dataLength += ret;
        }
      }
    }
    while (dataLength - dataIndex >= 7) {
      if (serialData[dataIndex] == 0x24 && serialData[dataIndex + 1] == 0x4C && serialData[dataIndex + 2] == 0x44
          && serialData[dataIndex + 3] == 0x41 && serialData[dataIndex + 4] == 0x43 && serialData[dataIndex + 5] == 0x4B
          && serialData[dataIndex + 6] == 0x2C) 
      {
        if (cmdAckRecvBuf.full()) cmdAckRecvBuf.eff_pop_front();
        int i = 7;
        while (dataLength - dataIndex - i >= 2) {
          if (serialData[dataIndex + i] == 0xEE && serialData[dataIndex + i + 1] == 0xFF) {
            cmdAckRecvBuf.emplace_back(serialData + dataIndex, i + 2);
            dataIndex += i + 2;
            break;
          }
          i++;
        }
        break;
      }
      if (serialData[dataIndex] == 0xee && serialData[dataIndex + 1] == 0xff) 
      {
        if (cmdAckRecvBuf.full()) cmdAckRecvBuf.eff_pop_front();
        if (serialData[dataIndex + 5] == 0x00) {
          if (dataLength - dataIndex < 80) break;
          pointCloudRecvBuf.emplace_back(serialData + dataIndex, 80);
          dataIndex += 80;
          break;
        } else if (serialData[dataIndex + 5] == 0x01) {
          if (dataLength - dataIndex < 34) break;
          pointCloudRecvBuf.emplace_back(serialData + dataIndex, 34);
          dataIndex += 34;
          break;
        } else {
          dataIndex += 6;
          break;
        }
      }
      dataIndex++;
    }
  }
}