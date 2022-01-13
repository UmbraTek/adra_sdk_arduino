// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#include "crc16.h"
uint16_t crc_modbus(uint8_t *data, int length) {
  int leng = length;
  uint8_t init_crch = 0xFF;
  uint8_t init_crcl = 0xFF;
  int i = 0;
  while (leng > 0) {
    uint8_t index = init_crch ^ data[i];
    i++;
    init_crch = init_crcl ^ CRC_TABLE_H[index];
    init_crcl = CRC_TABLE_L[index];
    leng--;
  }
  uint16_t crc16 = init_crch << 8 | init_crcl;
  return crc16;
}