// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#include "rosmos_decode.h"
ROSMOSDecode::ROSMOSDecode(uint8_t fromid, uint8_t toid) {
  this->rxstate = UX2HEX_RXSTART::FROMID;
  this->data_idx = 0;
  this->len = 0;
  this->fromid = fromid;
  this->toid = toid;
  memset(this->rxbuf, 0, sizeof(this->rxbuf));
}

ROSMOSDecode::~ROSMOSDecode() {}

void ROSMOSDecode::flush(uint8_t fromid, uint8_t toid) {
  this->rxstate = UX2HEX_RXSTART::FROMID;
  this->data_idx = 0;
  this->data_idx = 0;
  this->len = 0;
  this->fromid = fromid;
  this->toid = toid;
  memset(this->rxbuf, 0, sizeof(this->rxbuf));
}

int8_t ROSMOSDecode::put(uint8_t *tem_buf, int length, uint8_t *buf) {
  for (int i = 0; i < length; i++) {
    uint8_t rxch = tem_buf[i];
    if (UX2HEX_RXSTART::FROMID == this->rxstate && this->fromid == rxch) {
      this->rxbuf[0] = rxch;
      this->rxstate = UX2HEX_RXSTART::TOID;
    } else if (UX2HEX_RXSTART::TOID == this->rxstate) {
      if (this->toid == rxch) {
        this->rxbuf[1] = rxch;
        this->rxstate = UX2HEX_RXSTART::LEN;
      } else {
        this->rxstate = UX2HEX_RXSTART::FROMID;
      }
    } else if (UX2HEX_RXSTART::LEN == this->rxstate) {
      if ((rxch & 0x7F) < UX2HEX_RXSTART::RXLEN_MAX) {
        this->rxbuf[2] = rxch;
        this->len = rxch & 0x7F;
        this->data_idx = 0;
        this->rxstate = UX2HEX_RXSTART::DATA;
      } else {
        this->rxstate = UX2HEX_RXSTART::FROMID;
      }
    } else if (UX2HEX_RXSTART::DATA == this->rxstate) {
      if (this->data_idx < this->len) {
        this->rxbuf[this->data_idx + 3] = rxch;
        this->data_idx += 1;
        if (this->data_idx == this->len) {
          this->rxstate = UX2HEX_RXSTART::CRC1;
        }
      } else {
        this->rxstate = UX2HEX_RXSTART::FROMID;
      }
    } else if (UX2HEX_RXSTART::CRC1 == this->rxstate) {
      this->rxbuf[this->data_idx + 3] = rxch;
      this->rxstate = UX2HEX_RXSTART::CRC2;
    } else if (UX2HEX_RXSTART::CRC2 == this->rxstate) {
      this->rxbuf[this->data_idx + 4] = rxch;
      this->rxstate = UX2HEX_RXSTART::FROMID;
      uint16_t crc16 = crc_modbus(this->rxbuf, this->len + 3);
      uint8_t crc_1 = (crc16 / 256) % 256;
      uint8_t crc_2 = crc16 % 256;
      if (crc_1 == this->rxbuf[this->len + 3] && crc_2 == this->rxbuf[this->len + 4]) {
        memcpy(buf, this->rxbuf, 128 * sizeof(uint8_t));
        return this->data_idx + 5;  // return one protocol pack data length
      }
    }
  }
  return -1;
}

int8_t ROSMOSDecode::put(uint8_t rxch, uint8_t *buf) {
  // char buffer[5];
  // sprintf(buffer, "%02x", rxch);
  // Serial.println(buffer);
  if (UX2HEX_RXSTART::FROMID == this->rxstate && (this->fromid == rxch || this->fromid == 0x55)) {
    this->rxbuf[0] = rxch;
    this->rxstate = UX2HEX_RXSTART::TOID;
  } else if (UX2HEX_RXSTART::TOID == this->rxstate) {
    if (this->toid == rxch) {
      this->rxbuf[1] = rxch;
      this->rxstate = UX2HEX_RXSTART::LEN;
    } else {
      this->rxstate = UX2HEX_RXSTART::FROMID;
    }
  } else if (UX2HEX_RXSTART::LEN == this->rxstate) {
    if ((rxch & 0x7F) < UX2HEX_RXSTART::RXLEN_MAX) {
      this->rxbuf[2] = rxch;
      this->len = rxch & 0x7F;
      this->data_idx = 0;
      this->rxstate = UX2HEX_RXSTART::DATA;
    } else {
      this->rxstate = UX2HEX_RXSTART::FROMID;
    }
  } else if (UX2HEX_RXSTART::DATA == this->rxstate) {
    if (this->data_idx < this->len) {
      this->rxbuf[this->data_idx + 3] = rxch;
      this->data_idx += 1;
      if (this->data_idx == this->len) {
        this->rxstate = UX2HEX_RXSTART::CRC1;
      }
    } else {
      this->rxstate = UX2HEX_RXSTART::FROMID;
    }
  } else if (UX2HEX_RXSTART::CRC1 == this->rxstate) {
    this->rxbuf[this->data_idx + 3] = rxch;
    this->rxstate = UX2HEX_RXSTART::CRC2;
  } else if (UX2HEX_RXSTART::CRC2 == this->rxstate) {
    this->rxbuf[this->data_idx + 4] = rxch;
    // for(int i = 0;i<10; i++){
    //    char buffer[5];
    //    sprintf(buffer, "%02x", this->rxbuf[i]);
    //    Serial.println(buffer);
    // }
    this->rxstate = UX2HEX_RXSTART::FROMID;
    uint16_t crc16 = crc_modbus(this->rxbuf, this->len + 3);
    uint8_t crc_1 = (crc16 / 256) % 256;
    uint8_t crc_2 = crc16 % 256;
    if (crc_1 == this->rxbuf[this->len + 3] && crc_2 == this->rxbuf[this->len + 4]) {
      memcpy(buf, this->rxbuf, 128 * sizeof(uint8_t));
      return this->data_idx + 5;  // return one protocol pack data length
    }
  }
  return -1;
}