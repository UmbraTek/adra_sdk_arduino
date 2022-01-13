// Copyright 2020 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#include "socket_serial.h"

SocketSerial::SocketSerial(uint8_t rxPin, uint8_t txPin, uint8_t rts)
{
    softwareSerial = new SoftwareSerial(rxPin, txPin);
    this->rtsPin = rts;
    this->rx_decoder = new ROSMOSDecode(0, 0);
}
SocketSerial::~SocketSerial()
{
    delete softwareSerial;
}

void SocketSerial::begin(long speed)
{
    softwareSerial->begin(speed);
    pinMode(this->rtsPin, OUTPUT);
}

void SocketSerial::end()
{
    softwareSerial->end();
}

void SocketSerial::flush(uint8_t master_id, uint8_t slave_id)
{
    softwareSerial->flush();
    rx_decoder->flush(master_id, slave_id);
}

int SocketSerial::is_error()
{
    return 0;
}

int8_t SocketSerial::write(uint8_t *buf, uint8_t len)
{
    digitalWrite(this->rtsPin, HIGH); //RTS high is send
    int8_t num = softwareSerial->write(buf, len);
    if (num == len)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

/*
    return receive uint8_ts length is success
    return -1 is false
*/
int8_t SocketSerial::read(uint8_t *buf, int timeout)
{
    memset(buf, 0, sizeof(buf));
    digitalWrite(this->rtsPin, LOW); //RTS low is recevie
    int count = 0;
    while (true)
    {
        int len = softwareSerial->available();
        if (len > 0)
        {
            uint8_t tem_buf[len] = {0};
            int length = softwareSerial->readBytes(tem_buf, len);

            int8_t ret = rx_decoder->put(tem_buf, length, buf); //ret is the one protocol data pack length; ret > 0 is say that receive one pack

            if (ret > 0)
            {
                return ret;
            }
        }
        count = count + 1;
        if (count > timeout)
        {
            return -1;
        }
    }
    return 0;
}
