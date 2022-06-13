// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#ifndef ROSMOS_H
#define ROSMOS_H
#include <Arduino.h>

#include "crc16.h"
#include "socket_serial.h"

class ROSMOS_RX_ERROR {
 public:
  static const int8_t M_ID = -1;
  static const int8_t S_ID = -2;
  static const int8_t TIMEOUT = -3;
  static const int8_t STATE = -4;
  static const int8_t LEN = -5;
  static const int8_t RW = -6;
  static const int8_t CMD = -7;
  static const int8_t CRC = -8;
  static const int8_t CONNECT = -9;
  static const int8_t LEN_MIN = -10;
};

class ROSMOS_RW {
 public:
  const uint8_t R = 0;
  const uint8_t W = 1;
};

class ROSMOSType {
 public:
  ROSMOSType(uint8_t master_id, uint8_t slave_id, uint8_t state, uint8_t len, uint8_t rw, uint8_t cmd);
  ROSMOSType(uint8_t master_id, uint8_t slave_id);
  ROSMOSType();
  ~ROSMOSType();
  uint8_t pack(uint8_t *buf);
  int8_t unpack(uint8_t *buf, uint8_t length);
  void init();
  uint8_t master_id;
  uint8_t slave_id;
  uint8_t state;
  uint8_t len;
  uint8_t rw;
  uint8_t cmd;
  uint16_t crc;
  uint8_t data[128];
};

class ROSMOSClient {
 public:
  ROSMOSClient(uint8_t rxPin, uint8_t txPin, uint8_t rts, uint8_t master_id,
               uint8_t slave_id);  // option is array [ rxPin, txPin, rts] for serial connect

  ROSMOSClient(uint8_t serial_port, uint8_t rts, uint8_t master_id,
               uint8_t slave_id);  // option is array [ rxPin, txPin, rts] for serial connect
  // ~ROSMOSClient();
  uint8_t slave_id;

  void connect(long speed);
  void close();

  int8_t send(ROSMOSType *tx_ROSMOS);
  int8_t pend(ROSMOSType *tx_ROSMOS, ROSMOSType *rx_ROSMOS, uint8_t r_len, uint8_t timeout);

  SocketSerial *port;

 private:
  uint8_t master_id;
};

#endif
