// Copyright 2020 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#ifndef SOCKET_SERIAL_H
#define SOCKET_SERIAL_H
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "rosmos_decode.h"
class SocketSerial
{
public:
    SocketSerial(uint8_t rxPin, uint8_t txPin, uint8_t rts);
    ~SocketSerial();
    void begin(long speed);
    void end();
    void flush(uint8_t master_id, uint8_t slave_id);
    int is_error();
    int8_t write(uint8_t *buf, uint8_t len);
    int8_t read(uint8_t *buf, int timeout);

private:
    SoftwareSerial *softwareSerial;
    uint8_t rtsPin;
    int speed;
    ROSMOSDecode *rx_decoder;
};
#endif
