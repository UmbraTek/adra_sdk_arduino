// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#include "socket_serial.h"

SocketSerial::SocketSerial(uint8_t rxPin, uint8_t txPin, uint8_t rts) {
  this->rtsPin = rts;
  this->rx_decoder = new ROSMOSDecode(0, 0);
  this->isSoftwareSerial = true;
#if defined(SoftwareSerial_h)
  serialPort = new SoftwareSerial(rxPin, txPin);
#endif
}

SocketSerial::SocketSerial(uint8_t serial_port, uint8_t rts) {
  this->rtsPin = rts;
  this->rx_decoder = new ROSMOSDecode(0, 0);
  this->isSoftwareSerial = false;
  switch (serial_port) {
#if defined(PIN_SERIAL1_RX)
    case 1:
      serialPort = &Serial1;
      return;
#endif
#if defined(PIN_SERIAL2_RX)
    case 2:
      serialPort = &Serial2;
      return;
#endif
#if defined(PIN_SERIAL3_RX)
    case 3:
      serialPort = &Serial3;
      return;
#endif
#if defined(PIN_SERIAL4_RX)
    case 4:
      serialPort = &Serial4;
      return;
#endif
#if defined(PIN_SERIAL5_RX)
    case 5:
      serialPort = &Serial5;
      return;
#endif
#if defined(PIN_SERIAL6_RX)
    case 6:
      serialPort = &Serial6;
      return;
#endif
  }
}

SocketSerial::~SocketSerial() { delete serialPort; }

void SocketSerial::begin(long baud) {
  if (this->isSoftwareSerial) {
#if defined(SoftwareSerial_h)
    ((SoftwareSerial *)serialPort)->begin(baud);
#endif
  } else {
    ((HardwareSerial *)serialPort)->begin(baud);
  }
  pinMode(this->rtsPin, OUTPUT);
}

void SocketSerial::end() {
  if (this->isSoftwareSerial) {
#if defined(SoftwareSerial_h)
    ((SoftwareSerial *)serialPort)->end();
#endif
  } else {
    ((HardwareSerial *)serialPort)->end();
  }
}

void SocketSerial::flush(uint8_t master_id, uint8_t slave_id) {
  serialPort->flush();
  rx_decoder->flush(master_id, slave_id);
}

int SocketSerial::is_error() { return 0; }

int8_t SocketSerial::write(uint8_t *buf, uint8_t len) {
  digitalWrite(this->rtsPin, HIGH);  // RTS high is send
  delay(5);
  int8_t num = 0;
    if (this->isSoftwareSerial) {
  #if defined(SoftwareSerial_h)
      num = ((SoftwareSerial *)serialPort)->write(buf, len);
      ((SoftwareSerial *)serialPort)->flush();
  #endif
    } else {
      num = ((HardwareSerial *)serialPort)->write(buf, len);
      ((HardwareSerial *)serialPort)->flush();
    }

    digitalWrite(this->rtsPin, LOW);
    if (num == len) {
      return 0;
    } else {
      return -1;
    }
  return -1;
}

/*
    return receive uint8_ts length is success
    return -1 is false
*/
int8_t SocketSerial::read(uint8_t *buf, int timeout) {
  // digitalWrite(this->rtsPin, LOW);
  memset(buf, 0, sizeof(buf));
  countTime = millis();
  while (true) {
    if (serialPort->available() > 0) {
      uint8_t byte = serialPort->read();
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
