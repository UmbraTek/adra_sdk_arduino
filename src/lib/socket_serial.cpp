// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#include "socket_serial.h"

SocketSerial::SocketSerial(uint8_t rxPin, uint8_t txPin, uint8_t rts) {
  softwareSerial = new SoftwareSerial(rxPin, txPin);
  this->rtsPin = rts;
  this->rx_decoder = new ROSMOSDecode(0, 0);
}
SocketSerial::~SocketSerial() { delete softwareSerial; }

void SocketSerial::begin(long speed) {
  softwareSerial->begin(speed);
  pinMode(this->rtsPin, OUTPUT);
}

void SocketSerial::end() { softwareSerial->end(); }

void SocketSerial::flush(uint8_t master_id, uint8_t slave_id) {
  softwareSerial->flush();
  rx_decoder->flush(master_id, slave_id);
}

int SocketSerial::is_error() { return 0; }

int8_t SocketSerial::write(uint8_t *buf, uint8_t len) {
  digitalWrite(this->rtsPin, HIGH);  // RTS high is send
  int8_t num = softwareSerial->write(buf, len);
  // softwareSerial->flush();
  if (num == len) {
    return 0;
  } else {
    return -1;
  }
  // digitalWrite(this->rtsPin, LOW);
}

/*
    return receive uint8_ts length is success
    return -1 is false
*/
int8_t SocketSerial::read(uint8_t *buf, int timeout) {
  memset(buf, 0, sizeof(buf));
  softwareSerial->flush();
  digitalWrite(this->rtsPin, LOW);  // RTS low is recevie
  countTime = millis();
  while (true) {
    if (softwareSerial->available() > 0) {
      uint8_t byte = softwareSerial->read();
      int8_t ret = rx_decoder->put(byte, buf);
      if (ret > 0) {
        return ret;
      }
    }
    if ( millis() - countTime > timeout) {
      return -1;
    }
  }
  return 0;
}
