// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#include "rosmos.h"

ROSMOSType::ROSMOSType(uint8_t master_id, uint8_t slave_id, uint8_t state, uint8_t len, uint8_t rw, uint8_t cmd) {
  this->master_id = master_id;
  this->slave_id = slave_id;
  this->state = state;
  this->len = len;
  this->rw = rw;
  this->cmd = cmd;
  this->crc = 0;

  memset(this->data, 0, sizeof(this->data));
}

ROSMOSType::ROSMOSType(uint8_t master_id, uint8_t slave_id) {
  this->master_id = master_id;
  this->slave_id = slave_id;
  this->state = 0;
  this->len = 0;
  this->rw = 0;
  this->cmd = 0;
  this->crc = 0;
  memset(this->data, 0, sizeof(this->data));
}

ROSMOSType::ROSMOSType() {
  this->master_id = 0;
  this->slave_id = 0;
  this->state = 0;
  this->len = 0;
  this->rw = 0;
  this->cmd = 0;
  this->crc = 0;
  memset(this->data, 0, sizeof(this->data));
}

ROSMOSType::~ROSMOSType() {}

uint8_t ROSMOSType::pack(uint8_t *buf) {
  buf[0] = this->master_id;
  buf[1] = this->slave_id;
  buf[2] = ((this->state & 0x01) << 7) + (this->len & 0x7F);
  buf[3] = ((this->rw & 0x01) << 7) + (this->cmd & 0x7F);
  for (int index = 0; index < this->len - 1; index++) {
    buf[index + 4] = this->data[index];
  }
  uint16_t crc16 = crc_modbus(buf, this->len + 3);
  uint8_t crc_1 = (crc16 / 256) % 256;
  uint8_t crc_2 = crc16 % 256;
  buf[this->len + 3] = crc_1;
  buf[this->len + 4] = crc_2;
  return this->len + 5;
}

int8_t ROSMOSType::unpack(uint8_t *buf, uint8_t length) {
  if (length < 6) {
    return ROSMOS_RX_ERROR::LEN;
  }
  this->len = buf[2] & 0x7F;

  if (this->len + 5 != length) {
    return ROSMOS_RX_ERROR::LEN;
  }
  
  this->master_id = buf[0];
  this->slave_id = buf[1];
  this->state = (buf[2] & 0x80) >> 7;
  this->rw = (buf[3] & 0x80) >> 7;
  this->cmd = buf[3] & 0x7F;

  for (int index = 0; index < this->len - 1; index++) {
    this->data[index] = buf[4 + index];
  }
  this->crc = buf[this->len + 3] * 256 + buf[this->len + 4];

  return 0;
}

ROSMOSClient::ROSMOSClient(uint8_t rxPin, uint8_t txPin, uint8_t rts, uint8_t master_id, uint8_t slave_id) {
  this->port = new SocketSerial(rxPin, txPin, rts);
  this->master_id = master_id;
  this->slave_id = slave_id;
}

void ROSMOSClient::connect(long speed) { port->begin(speed); }

void ROSMOSClient::close() { port->end(); }

/*
    ROSMOSType *tx_ROSMOS is send data package

    return 0 is success
    return -1 is false
*/
int8_t ROSMOSClient::send(ROSMOSType *tx_ROSMOS) {
  uint8_t buf[128] = {0};
  uint8_t len = tx_ROSMOS->pack(buf);
  port->flush(this->slave_id, this->master_id);
  int8_t ret = port->write(buf, len);
  return ret;
}

/*
    ROSMOSType *tx_ROSMOS is send data package
    ROSMOSType *rx_ROSMOS is receive data package for return

    return:
        0 is success
        M_ID = -1;
        S_ID = -2;
        TIMEOUT = -3;
        STATE = -4;
        LEN = -5;
        RW = -6;
        CMD = -7;
        CRC = -8;
        CONNECT = -9;
        LEN_MIN = -10;
*/
int8_t ROSMOSClient::pend(ROSMOSType *tx_ROSMOS, ROSMOSType *rx_ROSMOS, uint8_t r_len, uint8_t timeout) {
  uint8_t readbuf[128];
  int8_t len = port->read(readbuf, timeout);
  int8_t ret = ROSMOS_RX_ERROR::TIMEOUT;
  if (len < 6) {
    return ret;
  }
  ret = rx_ROSMOS->unpack(readbuf, len);
  if (ret != 0) {
    return ret;
  } else if (rx_ROSMOS->master_id != tx_ROSMOS->slave_id && tx_ROSMOS->slave_id != 0x55) {
    ret = ROSMOS_RX_ERROR::M_ID;
  } else if (rx_ROSMOS->slave_id != tx_ROSMOS->master_id) {
    ret = ROSMOS_RX_ERROR::S_ID;
  } else if (rx_ROSMOS->state != 0) {
    ret = ROSMOS_RX_ERROR::STATE;
  } else if (rx_ROSMOS->len != r_len + 1) {
    ret = ROSMOS_RX_ERROR::LEN;
  } else if (rx_ROSMOS->rw != tx_ROSMOS->rw) {
    ret = ROSMOS_RX_ERROR::RW;
  } else if (rx_ROSMOS->cmd != tx_ROSMOS->cmd) {
    ret = ROSMOS_RX_ERROR::CMD;
  }
  return ret;
}