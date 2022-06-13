// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#include "adra_api.h"
// AdraApi::AdraApi(uint8_t rxPin, uint8_t txPin, uint8_t rts) {
//   this->port = new ROSMOSClient(rxPin, txPin, rts, 0xaa, 0);
//   this->master_id = 0xaa;
//   this->slave_id = 0;
//   this->tx_data = new ROSMOSType(0xaa, 0);
//   this->rx_data = new ROSMOSType();
//   uint8_t ret = RW_R;
// }

AdraApi::AdraApi(uint8_t serial_port, uint8_t rts) {
  this->port = new ROSMOSClient(serial_port, rts, 0xaa, 0);
  this->master_id = 0xaa;
  this->slave_id = 0;
  this->tx_data = new ROSMOSType(0xaa, 0);
  this->rx_data = new ROSMOSType();
  uint8_t ret = RW_R;
}

AdraApi::~AdraApi() {}

void AdraApi::connect(long speed) { port->connect(speed); }
void AdraApi::close() { port->close(); }

void AdraApi::_id(uint8_t id) {
  this->slave_id = id;
  this->tx_data->slave_id = id;
  this->port->slave_id = id;
}

/*
    return 0 is success
    return -1 is false
*/
int AdraApi::_send(uint8_t rw, uint8_t *cmd, uint8_t *cmd_data, uint8_t len_tx) {
  uint8_t data_wlen = 0;
  if (rw == RW_R) {
    data_wlen = cmd[1];
  } else {
    data_wlen = cmd[3];
  }

  if (len_tx != NULL) {
    data_wlen = len_tx;
  }

  tx_data->rw = rw;
  tx_data->cmd = cmd[0];
  tx_data->len = data_wlen + 1;
  for (int i = 0; i < data_wlen; i++) {
    tx_data->data[i] = cmd_data[i];
  }
  int ret = port->send(tx_data);
  return ret;
}

int AdraApi::_pend(ROSMOSType *rx_data, uint8_t rw, uint8_t *cmd, uint8_t timeout) {
  uint8_t data_rlen = 0;
  if (rw == RW_R) {
    data_rlen = cmd[2];
  } else {
    data_rlen = cmd[4];
  }
  int ret = port->pend(this->tx_data, rx_data, data_rlen, timeout);
  return ret;
}

int AdraApi::get_uuid(int id, char uuid[24]) {
  this->_id(id);
  uint8_t temp[12] = {-1};
  this->_send(RW_R, reg_.UUID, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.UUID, ROSMOS_TIMEOUT);
  memcpy(temp, rx_data->data, 12 * sizeof(uint8_t));
  for (int i = 0; i < 12; ++i) sprintf(&uuid[i * 2], "%02x", temp[i]);
  return 0;
}

int AdraApi::get_sw_version(int id, char version[12]) {
  this->_id(id);
  uint8_t temp[12];
  this->_send(RW_R, reg_.SW_VERSION, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.SW_VERSION, ROSMOS_TIMEOUT);

  memcpy(temp, rx_data->data, 12 * sizeof(uint8_t));
  for (int i = 0; i < 12; ++i) sprintf(&version[i], "%c", temp[i]);
  return ret;
}

int AdraApi::get_hw_version(int id, char version[24]) {
  this->_id(id);
  uint8_t temp[12] = {-1};
  this->_send(RW_R, reg_.HW_VERSION, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.HW_VERSION, ROSMOS_TIMEOUT);

  memcpy(temp, rx_data->data, 12 * sizeof(uint8_t));
  for (int i = 0; i < 12; ++i) sprintf(&version[i * 2], "%02x", temp[i]);
  
  return ret;
}

int AdraApi::get_multi_version(int id, char buf[12]) {
  this->_id(id);
  this->_send(RW_R, reg_.MULTI_VERSION, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.MULTI_VERSION, ROSMOS_TIMEOUT);

  int32_t temp2 = this->hex_to_int32_big(rx_data->data, 0);
  buf[0] = '0';
  buf[1] = '0';
  buf[2] = '0';
  sprintf(&buf[3], "%lu", temp2);
  
  return ret;
}

int AdraApi::get_mech_ratio(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.MECH_RATIO, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.MECH_RATIO, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_com_id(int id, uint8_t set_id) {
  this->_id(id);
  uint8_t txdata[1] = {set_id};
  this->_send(RW_W, reg_.COM_ID, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.COM_ID, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::set_com_baud(int id, uint32_t baud) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->int32_to_hex_big(baud, txdata);
  this->_send(RW_W, reg_.COM_BAUD, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.COM_BAUD, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::reset_err(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {reg_.RESET_ERR[0]};
  this->_send(RW_W, reg_.RESET_ERR, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.RESET_ERR, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::restart_driver(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {reg_.RESET_DRIVER[0]};
  this->_send(RW_W, reg_.RESET_DRIVER, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.RESET_DRIVER, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::erase_parm(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {reg_.ERASE_PARM[0]};
  this->_send(RW_W, reg_.ERASE_PARM, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.ERASE_PARM, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::saved_parm(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {reg_.SAVED_PARM[0]};
  this->_send(RW_W, reg_.SAVED_PARM, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.SAVED_PARM, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_elec_ratio(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.ELEC_RATIO, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.ELEC_RATIO, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_elec_ratio(int id, float ratio) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(ratio, txdata);
  this->_send(RW_W, reg_.ELEC_RATIO, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.ELEC_RATIO, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_motion_dir(int id, uint8_t *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.MOTION_DIR, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.MOTION_DIR, ROSMOS_TIMEOUT);
  buf[0] = rx_data->data[0];
  
  return ret;
}

int AdraApi::set_motion_dir(int id, uint8_t dir) {
  this->_id(id);
  uint8_t txdata[1] = {dir};
  this->_send(RW_W, reg_.MOTION_DIR, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.MOTION_DIR, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_iwdg_cyc(int id, int32_t *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.IWDG_CYC, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.IWDG_CYC, ROSMOS_TIMEOUT);

  *buf = this->hex_to_int32_big(rx_data->data, 0);
  
  return ret;
}

int AdraApi::set_iwdg_cyc(int id, uint32_t buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->int32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.IWDG_CYC, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.IWDG_CYC, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_temp_limit(int id, int8_t *min, int8_t *max) {
  this->_id(id);
  this->_send(RW_R, reg_.TEMP_LIMIT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TEMP_LIMIT, ROSMOS_TIMEOUT);
  min[0] = rx_data->data[0];
  max[0] = rx_data->data[1];
  
  return ret;
}

int AdraApi::set_temp_limit(int id, int8_t min, int8_t max) {
  this->_id(id);
  uint8_t txdata[2] = {min, max};
  this->_send(RW_W, reg_.TEMP_LIMIT, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TEMP_LIMIT, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_volt_limit(int id, uint8_t *min, uint8_t *max) {
  this->_id(id);
  this->_send(RW_R, reg_.VOLT_LIMIT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VOLT_LIMIT, ROSMOS_TIMEOUT);
  min[0] = rx_data->data[0];
  max[0] = rx_data->data[1];
  
  return ret;
}

int AdraApi::set_volt_limit(int id, uint8_t min, uint8_t max) {
  this->_id(id);
  uint8_t txdata[2] = {min, max};
  this->_send(RW_W, reg_.VOLT_LIMIT, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VOLT_LIMIT, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_curr_limit(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.CURR_LIMIT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.CURR_LIMIT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}
int AdraApi::set_curr_limit(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.CURR_LIMIT, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.CURR_LIMIT, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_motion_mode(int id, uint8_t *mode) {
  this->_id(id);
  this->_send(RW_R, reg_.MOTION_MODE, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.MOTION_MODE, ROSMOS_TIMEOUT);
  mode[0] = rx_data->data[0];
  
  return ret;
}

int AdraApi::_set_motion_mode(int id, uint8_t mode) {
  this->_id(id);
  uint8_t txdata[1] = {mode};
  this->_send(RW_W, reg_.MOTION_MODE, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.MOTION_MODE, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_motion_enable(int id, uint8_t *able) {
  this->_id(id);
  this->_send(RW_R, reg_.MOTION_ENABLE, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.MOTION_ENABLE, ROSMOS_TIMEOUT);
  able[0] = rx_data->data[0];
  
  return ret;
}

int AdraApi::_set_motion_enable(int id, uint8_t able) {
  this->_id(id);
  uint8_t txdata[1] = {able};
  this->_send(RW_W, reg_.MOTION_ENABLE, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.MOTION_ENABLE, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_brake_enable(int id, uint8_t *able) {
  this->_id(id);
  this->_send(RW_R, reg_.BRAKE_ENABLE, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.BRAKE_ENABLE, ROSMOS_TIMEOUT);
  able[0] = rx_data->data[0];
  
  return ret;
}

int AdraApi::_set_brake_enable(int id, uint8_t able) {
  this->_id(id);
  uint8_t txdata[1] = {able};
  this->_send(RW_W, reg_.BRAKE_ENABLE, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.BRAKE_ENABLE, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::into_motion_mode_pos(int id) { return this->_set_motion_mode(id, MOTION_MODEL::POS); }
int AdraApi::into_motion_mode_vel(int id) { return this->_set_motion_mode(id, MOTION_MODEL::VEL); }
int AdraApi::into_motion_mode_tau(int id) { return this->_set_motion_mode(id, MOTION_MODEL::TAU); }
int AdraApi::into_motion_enable(int id) { return this->_set_motion_enable(id, 1); }
int AdraApi::into_motion_disable(int id) { return this->_set_motion_enable(id, 0); }
int AdraApi::into_brake_enable(int id) { return this->_set_brake_enable(id, 1); }
int AdraApi::into_brake_disable(int id) { return this->_set_brake_enable(id, 0); }

int AdraApi::get_temp_driver(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TEMP_DRIVER, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TEMP_DRIVER, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::get_temp_motor(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TEMP_MOTOR, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TEMP_MOTOR, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::get_bus_volt(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.BUS_VOLT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.BUS_VOLT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::get_bus_curr(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.BUS_CURR, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.BUS_CURR, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::get_multi_volt(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.MULTI_VOLT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.MULTI_VOLT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::get_error_code(int id, uint8_t *code) {
  this->_id(id);
  this->_send(RW_R, reg_.ERROR_CODE, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.ERROR_CODE, ROSMOS_TIMEOUT);
  code[0] = rx_data->data[0];
  
  return ret;
}

int AdraApi::get_pos_target(int id, float *target) {
  this->_id(id);
  this->_send(RW_R, reg_.POS_TARGET, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.POS_TARGET, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], target, 1);
  
  return ret;
}

int AdraApi::set_pos_target(int id, float target) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(target, txdata);
  this->_send(RW_W, reg_.POS_TARGET, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.POS_TARGET, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_pos_current(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.POS_CURRENT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.POS_CURRENT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::get_pos_limit_min(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.POS_LIMIT_MIN, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.POS_LIMIT_MIN, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_pos_limit_min(int id, float pos) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(pos, txdata);
  this->_send(RW_W, reg_.POS_LIMIT_MIN, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.POS_LIMIT_MIN, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_pos_limit_max(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.POS_LIMIT_MAX, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.POS_LIMIT_MAX, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_pos_limit_max(int id, float pos) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(pos, txdata);
  this->_send(RW_W, reg_.POS_LIMIT_MAX, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.POS_LIMIT_MAX, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_pos_limit_diff(int id, float *max) {
  this->_id(id);
  this->_send(RW_R, reg_.POS_LIMIT_DIFF, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.POS_LIMIT_DIFF, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], max, 1);
  
  return ret;
}

int AdraApi::set_pos_limit_diff(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.POS_LIMIT_DIFF, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.POS_LIMIT_DIFF, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_pos_pidp(int id, float *p) {
  this->_id(id);
  this->_send(RW_R, reg_.POS_PIDP, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.POS_PIDP, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], p, 1);
  return ret;
}

int AdraApi::set_pos_pidp(int id, float p) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(p, txdata);
  this->_send(RW_W, reg_.POS_PIDP, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.POS_PIDP, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_pos_smooth_cyc(int id, uint8_t *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.POS_SMOOTH_CYC, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.POS_SMOOTH_CYC, ROSMOS_TIMEOUT);
  buf[0] = rx_data->data[0];
  
  return ret;
}

int AdraApi::set_pos_smooth_cyc(int id, uint8_t buf) {
  this->_id(id);
  uint8_t txdata[1] = {buf};
  this->_send(RW_W, reg_.POS_SMOOTH_CYC, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.POS_SMOOTH_CYC, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_pos_adrc_param(int id, uint8_t i, float *param) {
  this->_id(id);
  uint8_t txdata[1] = {i};
  this->_send(RW_R, reg_.POS_ADRC_PARAM, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.POS_ADRC_PARAM, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], param, 1);
  
  return ret;
}

int AdraApi::set_pos_adrc_param(int id, uint8_t i, float param) {
  this->_id(id);
  uint8_t data[4] = {0};
  this->fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  this->_send(RW_W, reg_.POS_ADRC_PARAM, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.POS_ADRC_PARAM, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::pos_cal_zero(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {reg_.POS_CAL_ZERO[0]};
  this->_send(RW_W, reg_.POS_CAL_ZERO, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.POS_CAL_ZERO, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_vel_target(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.VEL_TARGET, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_TARGET, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_vel_target(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.VEL_TARGET, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VEL_TARGET, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_vel_current(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.VEL_CURRENT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_CURRENT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::get_vel_limit_min(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.VEL_LIMIT_MIN, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_LIMIT_MIN, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_vel_limit_min(int id, float vel) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(vel, txdata);
  this->_send(RW_W, reg_.VEL_LIMIT_MIN, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VEL_LIMIT_MIN, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_vel_limit_max(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.VEL_LIMIT_MAX, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_LIMIT_MAX, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_vel_limit_max(int id, float vel) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(vel, txdata);
  this->_send(RW_W, reg_.VEL_LIMIT_MAX, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VEL_LIMIT_MAX, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_vel_limit_diff(int id, float *max) {
  this->_id(id);
  this->_send(RW_R, reg_.VEL_LIMIT_DIFF, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_LIMIT_DIFF, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], max, 1);
  
  return ret;
}

int AdraApi::set_vel_limit_diff(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.VEL_LIMIT_DIFF, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VEL_LIMIT_DIFF, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_vel_pidp(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.VEL_PIDP, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_PIDP, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_vel_pidp(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.VEL_PIDP, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VEL_PIDP, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_vel_pidi(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.VEL_PIDI, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_PIDI, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_vel_pidi(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.VEL_PIDI, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VEL_PIDI, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_vel_smooth_cyc(int id, uint8_t *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.VEL_SMOOTH_CYC, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_SMOOTH_CYC, ROSMOS_TIMEOUT);
  buf[0] = rx_data->data[0];
  
  return ret;
}

int AdraApi::set_vel_smooth_cyc(int id, uint8_t buf) {
  this->_id(id);
  uint8_t txdata[1] = {buf};
  this->_send(RW_W, reg_.VEL_SMOOTH_CYC, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VEL_SMOOTH_CYC, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_vel_adrc_param(int id, uint8_t i, float *param) {
  this->_id(id);
  uint8_t txdata[1] = {i};
  this->_send(RW_R, reg_.VEL_ADRC_PARAM, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.VEL_ADRC_PARAM, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], param, 1);
  
  return ret;
}

int AdraApi::set_vel_adrc_param(int id, uint8_t i, float param) {
  this->_id(id);
  uint8_t data[4] = {0};
  this->fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  this->_send(RW_W, reg_.VEL_ADRC_PARAM, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.VEL_ADRC_PARAM, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_tau_target(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TAU_TARGET, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_TARGET, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_tau_target(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.TAU_TARGET, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TAU_TARGET, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_tau_current(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TAU_CURRENT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_CURRENT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::get_tau_limit_min(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TAU_LIMIT_MIN, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_LIMIT_MIN, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_tau_limit_min(int id, float tau) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(tau, txdata);
  this->_send(RW_W, reg_.TAU_LIMIT_MIN, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TAU_LIMIT_MIN, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_tau_limit_max(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TAU_LIMIT_MAX, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_LIMIT_MAX, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_tau_limit_max(int id, float tau) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(tau, txdata);
  this->_send(RW_W, reg_.TAU_LIMIT_MAX, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TAU_LIMIT_MAX, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_tau_limit_diff(int id, float *max) {
  this->_id(id);
  this->_send(RW_R, reg_.TAU_LIMIT_DIFF, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_LIMIT_DIFF, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], max, 1);
  
  return ret;
}

int AdraApi::set_tau_limit_diff(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.TAU_LIMIT_DIFF, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TAU_LIMIT_DIFF, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_tau_pidp(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TAU_PIDP, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_PIDP, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_tau_pidp(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.TAU_PIDP, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TAU_PIDP, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_tau_pidi(int id, float *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TAU_PIDI, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_PIDI, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  
  return ret;
}

int AdraApi::set_tau_pidi(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(RW_W, reg_.TAU_PIDI, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TAU_PIDI, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_tau_smooth_cyc(int id, uint8_t *buf) {
  this->_id(id);
  this->_send(RW_R, reg_.TAU_SMOOTH_CYC, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_SMOOTH_CYC, ROSMOS_TIMEOUT);
  buf[0] = rx_data->data[0];
  
  return ret;
}

int AdraApi::set_tau_smooth_cyc(int id, uint8_t buf) {
  this->_id(id);
  uint8_t txdata[1] = {buf};
  this->_send(RW_W, reg_.TAU_SMOOTH_CYC, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TAU_SMOOTH_CYC, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::get_tau_adrc_param(int id, uint8_t i, float *param) {
  this->_id(id);
  uint8_t txdata[1] = {i};
  this->_send(RW_R, reg_.TAU_ADRC_PARAM, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.TAU_ADRC_PARAM, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], param, 1);
  
  return ret;
}

int AdraApi::set_tau_adrc_param(int id, uint8_t i, float param) {
  this->_id(id);
  uint8_t data[4] = {0};
  this->fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  this->_send(RW_W, reg_.TAU_ADRC_PARAM, txdata, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_W, reg_.TAU_ADRC_PARAM, ROSMOS_TIMEOUT);
  
  return ret;
}

int AdraApi::set_cpos_target(uint8_t sid, uint8_t eid, float *pos) {
  this->_id(0x55);
  int num = eid - sid + 1;
  uint8_t data[4 * num + 2];
  data[0] = sid;
  data[1] = eid;
  this->fp32_to_hex_big(pos, &data[2], num);
  reg_.CPOS_TARGET[3] = 2 + 4 * num;
  this->_send(RW_W, reg_.CPOS_TARGET, data, num * 4 + 2);
  return 0;
}

int AdraApi::set_ctau_target(uint8_t sid, uint8_t eid, float *tau) {
  this->_id(0x55);
  int num = eid - sid + 1;
  uint8_t data[4 * num + 2];
  data[0] = sid;
  data[1] = eid;
  this->fp32_to_hex_big(tau, &data[2], num);
  reg_.CTAU_TARGET[3] = 2 + 4 * num;
  this->_send(RW_W, reg_.CTAU_TARGET, data, num * 4 + 2);
  return 0;
}

int AdraApi::set_cpostau_target(uint8_t sid, uint8_t eid, float *pos, float *tau) {
  this->_id(0x55);
  int num = eid - sid + 1;
  float postau[num * 2];
  for (int i = 0; i < num; i++) {
    postau[i * 2] = pos[i];
    postau[i * 2 + 1] = tau[i];
  }

  uint8_t data[4 * num * 2 + 2];
  data[0] = sid;
  data[1] = eid;
  this->fp32_to_hex_big(postau, &data[2], num * 2);
  reg_.CPOSTAU_TARGET[3] = 4 * num * 2 + 2;
  this->_send(RW_W, reg_.CPOSTAU_TARGET, data, 4 * num * 2 + 2);
  return 0;
}

int AdraApi::get_spostau_current(int id, int *num, float *pos, float *tau) {
  this->_id(id);
  this->_send(RW_R, reg_.SPOSTAU_CURRENT, NULL, NULL);
  rx_data->init();
  int ret = this->_pend(rx_data, RW_R, reg_.SPOSTAU_CURRENT, ROSMOS_TIMEOUT);
  if (ret == ROSMOS_RX_ERROR::TIMEOUT) {
    *num = 0;
    *pos = 0;
    *tau = 0;
  } else {
    *num = rx_data->data[0];
    *pos = this->hex_to_fp32_big(&rx_data->data[1]);
    *tau = this->hex_to_fp32_big(&rx_data->data[5]);
  }
  
  return ret;
}

int AdraApi::get_cpostau_current(uint8_t sid, uint8_t eid, int *num, float *pos, float *tau, int *ret) {
  int temp = 0;
  this->_id(0x55);
  const uint8_t *cmd = reg_.CPOSTAU_CURRENT;
  uint8_t tx_data[2] = {sid, eid};
  float timeout_s = 0.5;
  int tx_len = cmd[1];
  int rx_len = cmd[2];
  this->_send(RW_R, reg_.CPOSTAU_CURRENT, tx_data, tx_len);
  for (int i = 0; i < eid - sid + 1; i++) {
    rx_data->init();
    ret[i] = this->_pend(rx_data, RW_R, reg_.CPOSTAU_CURRENT, ROSMOS_TIMEOUT);
    if (rx_data->master_id != sid + i) ret[i] = ROSMOS_RX_ERROR::TIMEOUT;
    if (ret[i] != ROSMOS_RX_ERROR::TIMEOUT) {
      num[i] = rx_data->data[0];
      pos[i] = this->hex_to_fp32_big(&rx_data->data[1]);
      tau[i] = this->hex_to_fp32_big(&rx_data->data[5]);
    } else {
      num[i] = 0;
      pos[i] = 0;
      tau[i] = 0;
    }

    temp += ret[i];
    
  }
  return temp;
}