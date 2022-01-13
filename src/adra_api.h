// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#ifndef UTSERVO_API_H
#define UTSERVO_API_H
#include <Arduino.h>

#include "lib/adra_reg.h"
#include "lib/rosmos.h"

const uint8_t ROSMOS_TIMEOUT = 100;
const uint8_t R_LEN = 64;

class AdraApi {
 public:
  AdraApi(uint8_t rxPin, uint8_t txPin, uint8_t rts);
  ~AdraApi();

  void connect(long speed);
  void close();

  int get_uuid(int id, char uuid[24]);
  int get_sw_version(int id, char version[12]);
  int get_hw_version(int id, char type[24]);
  int get_multi_version(int id, char version[12]);

  int get_mech_ratio(int id, float *ratio);

  int set_com_id(int id, uint8_t set_id);
  int set_com_baud(int id, uint32_t baud);
  int reset_err(uint8_t id);
  int restart_driver(uint8_t id);

  int erase_parm(uint8_t id);
  int saved_parm(uint8_t id);

  int get_elec_ratio(int id, float *ratio);
  int set_elec_ratio(int id, float ratio);
  int get_motion_dir(int id, uint8_t *dir);
  int set_motion_dir(int id, uint8_t dir);
  int get_temp_limit(int id, int8_t *min, int8_t *max);
  int set_temp_limit(int id, int8_t *min, int8_t *max);
  int get_volt_limit(int id, uint8_t *min, uint8_t *max);
  int set_volt_limit(int id, uint8_t min, uint8_t max);
  int get_curr_limit(int id, float *buf);
  int set_curr_limit(int id, float buf);

  int get_motion_mode(int id, uint8_t *mode);
  int get_motion_enable(int id, uint8_t *able);
  int get_brake_enable(int id, uint8_t *able);

  int into_motion_mode_pos(int id);
  int into_motion_mode_vel(int id);
  int into_motion_mode_tau(int id);
  int into_motion_enable(int id);
  int into_motion_disable(int id);
  int into_brake_enable(int id);
  int into_brake_disable(int id);

  int get_temp_driver(int id, float *buf);
  int get_temp_motor(int id, float *buf);
  int get_bus_volt(int id, float *buf);
  int get_bus_curr(int id, float *buf);
  int get_multi_volt(int id, float *buf);
  int get_error_code(int id, uint8_t *code);

  int get_pos_target(int id, float *target);
  int set_pos_target(int id, float target);
  int get_pos_current(int id, float *pos);
  int get_pos_limit_min(int id, float *buf);
  int set_pos_limit_min(int id, float buf);
  int get_pos_limit_max(int id, float *buf);
  int set_pos_limit_max(int id, float buf);
  int get_pos_limit_diff(int id, float *max);
  int set_pos_limit_diff(int id, float max);
  int get_pos_pidp(int id, float *p);
  int set_pos_pidp(int id, float p);
  int get_pos_smooth_cyc(int id, uint8_t *cyc);
  int set_pos_smooth_cyc(int id, uint8_t cyc);
  int get_pos_adrc_param(int id, uint8_t i, float *param);
  int set_pos_adrc_param(int id, uint8_t i, float param);
  int pos_cal_zero(uint8_t id);

  int get_vel_target(int id, float *target);
  int set_vel_target(int id, float target);
  int get_vel_current(int id, float *current);
  int get_vel_limit_min(int id, float *buf);
  int set_vel_limit_min(int id, float buf);
  int get_vel_limit_max(int id, float *buf);
  int set_vel_limit_max(int id, float buf);
  int get_vel_limit_diff(int id, float *max);
  int set_vel_limit_diff(int id, float max);
  int get_vel_pidp(int id, float *p);
  int set_vel_pidp(int id, float p);
  int get_vel_pidi(int id, float *i);
  int set_vel_pidi(int id, float i);
  int get_vel_smooth_cyc(int id, uint8_t *cyc);
  int set_vel_smooth_cyc(int id, uint8_t cyc);
  int get_vel_adrc_param(int id, uint8_t i, float *param);
  int set_vel_adrc_param(int id, uint8_t i, float param);

  int get_tau_target(int id, float *target);
  int set_tau_target(int id, float target);
  int get_tau_current(int id, float *current);
  int get_tau_limit_min(int id, float *buf);
  int set_tau_limit_min(int id, float buf);
  int get_tau_limit_max(int id, float *buf);
  int set_tau_limit_max(int id, float buf);
  int get_tau_limit_diff(int id, float *max);
  int set_tau_limit_diff(int id, float max);
  int get_tau_pidp(int id, float *pidp);
  int set_tau_pidp(int id, float pidp);
  int get_tau_pidi(int id, float *pidi);
  int set_tau_pidi(int id, float pidi);
  int get_tau_smooth_cyc(int id, uint8_t *cyc);
  int set_tau_smooth_cyc(int id, uint8_t cyc);
  int get_tau_adrc_param(int id, uint8_t i, float *param);
  int set_tau_adrc_param(int id, uint8_t i, float param);

 private:
  uint8_t master_id;
  uint8_t slave_id;
  ROSMOSClient *port;
  ROSMOSType *tx_data;

  int _set_motion_mode(int id, uint8_t mode);
  int _set_motion_enable(int id, uint8_t able);
  int _set_brake_enable(int id, uint8_t able);

  int _send(uint8_t rw, uint8_t *cmd, uint8_t *cmd_data, uint8_t len_tx);
  int _pend(ROSMOSType *rx_data, uint8_t rw, uint8_t *cmd, uint8_t timeout);
  void _id(uint8_t id);

  int16_t hex_to_int16_big(uint8_t *a, uint8_t i) {
    unsigned long i1 = (unsigned long)(a[i]);
    unsigned long i2 = (unsigned long)(a[i + 1]);
    i1 = i1 << 8;
    return (int32_t)(i1 + i2);
  }
  int32_t hex_to_int24_big(uint8_t *a, uint8_t i) {
    unsigned long i1 = (unsigned long)(a[i]);
    unsigned long i2 = (unsigned long)(a[i + 1]);
    unsigned long i3 = (unsigned long)(a[i + 2]);
    i1 = i1 << 16;
    i2 = i2 << 8;
    return (int32_t)(i1 + i2 + i3);
  }

  int32_t hex_to_int32_big(uint8_t *a, uint8_t i) {
    unsigned long i1 = (unsigned long)(a[i]);
    unsigned long i2 = (unsigned long)(a[i + 1]);
    unsigned long i3 = (unsigned long)(a[i + 2]);
    unsigned long i4 = (unsigned long)(a[i + 3]);
    i1 = i1 << 24;
    i2 = i2 << 16;
    i3 = i3 << 8;
    return (int32_t)(i1 + i2 + i3 + i4);
  }

  void int16_to_hex_big(uint16_t a, uint8_t *b) {
    b[0] = (uint8_t)(a >> 8);
    b[1] = (uint8_t)a;
  }

  void int32_to_hex_big(uint32_t a, uint8_t *b) {
    unsigned long tem = (unsigned long)a;
    unsigned long i0 = tem >> 24;
    unsigned long i1 = tem >> 16;
    unsigned long i2 = tem >> 8;
    b[0] = (uint8_t)i0;
    b[1] = (uint8_t)i1;
    b[2] = (uint8_t)i2;
    b[3] = (uint8_t)tem;
  }
  float int_to_rad(uint32_t value) { return (float)value * 0.00001; }
  uint32_t rad_to_int(float value) { return (uint32_t)(value * 100000); }
  int32_t hex_to_int32_big(uint8_t *a) { return ((a[0] << 24) + (a[1] << 16) + (a[2] << 8) + a[3]); }
  float hex_to_fp32_big(uint8_t a[4]) {
    union _fp32hex {
      float dataf;
      uint32_t datai;
    } fp32hex;
    fp32hex.datai = hex_to_int32_big(a, 0);
    return (float)fp32hex.dataf;
  }
  void hex_to_fp32_big(uint8_t *a, float *b, int n) {
    for (int i = 0; i < n; ++i) b[i] = hex_to_fp32_big(&a[i * 4]);
  }

  void fp32_to_hex_big(float dataf, uint8_t datahex[4]) {
    union _fp32hex {
      float dataf;
      uint32_t datai;
    } fp32hex;
    fp32hex.dataf = dataf;
    int32_to_hex_big(fp32hex.datai, datahex);
  }
  void fp32_to_hex_big(float *dataf, uint8_t *datahex, int n) {
    for (int i = 0; i < n; ++i) fp32_to_hex_big(dataf[i], &datahex[i * 4]);
  }
};

class MOTION_MODEL {
 public:
  const static uint8_t POS = 1;
  const static uint8_t VEL = 2;
  const static uint8_t TAU = 3;
};

#endif