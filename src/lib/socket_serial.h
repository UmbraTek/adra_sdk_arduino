// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#ifndef SOCKET_SERIAL_H
#define SOCKET_SERIAL_H
#include <Arduino.h>
#include <Boards.h>
// #include <Firmata.h>
#if (ARDUINO > 10605) && (defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ARC32))
#include <SoftwareSerial.h>
#endif


#include "rosmos_decode.h"

class SocketSerial {
 public:
  SocketSerial(uint8_t rxPin, uint8_t txPin, uint8_t rts);
  SocketSerial(uint8_t serial_port, uint8_t rts);
  ~SocketSerial();
  void begin(long baud);
  void end();
  void flush(uint8_t master_id, uint8_t slave_id);
  int is_error();
  int8_t write(uint8_t *buf, uint8_t len);
  int8_t read(uint8_t *buf, int timeout);

 private:
  Stream *serialPort;
  uint8_t rtsPin;
  long baud;
  long countTime;
  bool isSoftwareSerial = false;
  ROSMOSDecode *rx_decoder;
};
#endif
