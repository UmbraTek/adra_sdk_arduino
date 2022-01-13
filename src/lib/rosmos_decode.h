// Copyright 2020 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#ifndef ROSMOS_DECODE_H
#define ROSMOS_DECODE_H
#include <Arduino.h>
#include "crc16.h"
class UX2HEX_RXSTART
{
public:
    static const uint8_t FROMID = 0;
    static const uint8_t TOID = 1;
    static const uint8_t LEN = 2;
    static const uint8_t DATA = 3;
    static const uint8_t CRC1 = 4;
    static const uint8_t CRC2 = 5;
    static const uint8_t RXLEN_MAX = 64;
};

class ROSMOSDecode
{
public:
    ROSMOSDecode(uint8_t fromid, uint8_t toid);
    ~ROSMOSDecode();
    void flush(uint8_t fromid, uint8_t toid);
    int8_t put(uint8_t *tem_buf, int length, uint8_t *buf);

private:
    uint8_t rxstate;
    uint8_t data_idx;
    uint8_t len;
    uint8_t fromid;
    uint8_t toid;
    uint8_t rxbuf[128];
};

#endif