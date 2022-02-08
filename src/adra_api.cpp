// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#include "adra_api.h"
AdraApi::AdraApi(uint8_t rxPin, uint8_t txPin, uint8_t rts) {
  this->port = new ROSMOSClient(rxPin, txPin, rts, 0xaa, 0);
  this->master_id = 0xaa;
  this->slave_id = 0;
  this->tx_data = new ROSMOSType(master_id, 0);
  uint8_t ret = ROSMOS_RW::R;
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
  if (rw == ROSMOS_RW::R) {
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
  if (rw == ROSMOS_RW::R) {
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
  this->_send(ROSMOS_RW::R, ADRA_REG_UUID, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_UUID, ROSMOS_TIMEOUT);

  memcpy(temp, rx_data->data, 12 * sizeof(uint8_t));
  for (int i = 0; i < 12; ++i) sprintf(&uuid[i * 2], "%02x", temp[i]);
  delete rx_data;
  return ret;
}

int AdraApi::get_sw_version(int id, char version[12]) {
  this->_id(id);
  uint8_t temp[12];
  this->_send(ROSMOS_RW::R, ADRA_REG_SW_VERSION, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_SW_VERSION, ROSMOS_TIMEOUT);

  memcpy(temp, rx_data->data, 12 * sizeof(uint8_t));
  for (int i = 0; i < 12; ++i) sprintf(&version[i], "%c", temp[i]);
  delete rx_data;
  return ret;
}

int AdraApi::get_hw_version(int id, char version[24]) {
  this->_id(id);
  uint8_t temp[12] = {-1};
  this->_send(ROSMOS_RW::R, ADRA_REG_HW_VERSION, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_HW_VERSION, ROSMOS_TIMEOUT);

  memcpy(temp, rx_data->data, 12 * sizeof(uint8_t));
  for (int i = 0; i < 12; ++i) sprintf(&version[i * 2], "%02x", temp[i]);
  delete rx_data;
  return ret;
}

int AdraApi::get_multi_version(int id, char buf[12]) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_MULTI_VERSION, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_MULTI_VERSION, ROSMOS_TIMEOUT);

  int32_t temp2 = this->hex_to_int32_big(rx_data->data, 0);
  buf[0] = '0';
  buf[1] = '0';
  buf[2] = '0';
  sprintf(&buf[3], "%lu", temp2);
  delete rx_data;
  return ret;
}

int AdraApi::get_mech_ratio(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_MECH_RATIO, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_MECH_RATIO, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_com_id(int id, uint8_t set_id) {
  this->_id(id);
  uint8_t txdata[1] = {set_id};
  this->_send(ROSMOS_RW::W, ADRA_REG_COM_ID, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_COM_ID, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::set_com_baud(int id, uint32_t baud) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->int32_to_hex_big(baud, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_COM_BAUD, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_COM_BAUD, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::reset_err(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {ADRA_REG_RESET_ERR[0]};
  this->_send(ROSMOS_RW::W, ADRA_REG_RESET_ERR, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_RESET_ERR, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::restart_driver(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {ADRA_REG_RESET_DRIVER[0]};
  this->_send(ROSMOS_RW::W, ADRA_REG_RESET_DRIVER, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_RESET_DRIVER, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::erase_parm(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {ADRA_REG_ERASE_PARM[0]};
  this->_send(ROSMOS_RW::W, ADRA_REG_ERASE_PARM, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_ERASE_PARM, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::saved_parm(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {ADRA_REG_SAVED_PARM[0]};
  this->_send(ROSMOS_RW::W, ADRA_REG_SAVED_PARM, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_SAVED_PARM, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_elec_ratio(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_ELEC_RATIO, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_ELEC_RATIO, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_elec_ratio(int id, float ratio) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(ratio, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_ELEC_RATIO, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_ELEC_RATIO, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_motion_dir(int id, uint8_t *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_MOTION_DIR, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_MOTION_DIR, ROSMOS_TIMEOUT);
  buf[0] = rx_data->data[0];
  delete rx_data;
  return ret;
}

int AdraApi::set_motion_dir(int id, uint8_t dir) {
  this->_id(id);
  uint8_t txdata[1] = {dir};
  this->_send(ROSMOS_RW::W, ADRA_REG_MOTION_DIR, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_MOTION_DIR, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_temp_limit(int id, int8_t *min, int8_t *max) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TEMP_LIMIT, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TEMP_LIMIT, ROSMOS_TIMEOUT);
  min[0] = rx_data->data[0];
  max[0] = rx_data->data[1];
  delete rx_data;
  return ret;
}

int AdraApi::set_temp_limit(int id, int8_t min, int8_t max) {
  this->_id(id);
  uint8_t txdata[2] = {min, max};
  this->_send(ROSMOS_RW::W, ADRA_REG_TEMP_LIMIT, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TEMP_LIMIT, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_volt_limit(int id, uint8_t *min, uint8_t *max) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VOLT_LIMIT, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VOLT_LIMIT, ROSMOS_TIMEOUT);
  min[0] = rx_data->data[0];
  max[0] = rx_data->data[1];
  delete rx_data;
  return ret;
}

int AdraApi::set_volt_limit(int id, uint8_t min, uint8_t max) {
  this->_id(id);
  uint8_t txdata[2] = {min, max};
  this->_send(ROSMOS_RW::W, ADRA_REG_VOLT_LIMIT, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VOLT_LIMIT, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_curr_limit(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_CURR_LIMIT, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_CURR_LIMIT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}
int AdraApi::set_curr_limit(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_CURR_LIMIT, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_CURR_LIMIT, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_motion_mode(int id, uint8_t *mode) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_MOTION_MDOE, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_MOTION_MDOE, ROSMOS_TIMEOUT);
  mode[0] = rx_data->data[0];
  delete rx_data;
  return ret;
}

int AdraApi::_set_motion_mode(int id, uint8_t mode) {
  this->_id(id);
  uint8_t txdata[1] = {mode};
  this->_send(ROSMOS_RW::W, ADRA_REG_MOTION_MDOE, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_MOTION_MDOE, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_motion_enable(int id, uint8_t *able) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_MOTION_ENABLE, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_MOTION_ENABLE, ROSMOS_TIMEOUT);
  able[0] = rx_data->data[0];
  delete rx_data;
  return ret;
}

int AdraApi::_set_motion_enable(int id, uint8_t able) {
  this->_id(id);
  uint8_t txdata[1] = {able};
  this->_send(ROSMOS_RW::W, ADRA_REG_MOTION_ENABLE, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_MOTION_ENABLE, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_brake_enable(int id, uint8_t *able) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_BRAKE_ENABLE, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_BRAKE_ENABLE, ROSMOS_TIMEOUT);
  able[0] = rx_data->data[0];
  delete rx_data;
  return ret;
}

int AdraApi::_set_brake_enable(int id, uint8_t able) {
  this->_id(id);
  uint8_t txdata[1] = {able};
  this->_send(ROSMOS_RW::W, ADRA_REG_BRAKE_ENABLE, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_BRAKE_ENABLE, ROSMOS_TIMEOUT);
  delete rx_data;
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
  this->_send(ROSMOS_RW::R, ADRA_REG_TEMP_DRIVER, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TEMP_DRIVER, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::get_temp_motor(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TEMP_MOTO, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TEMP_MOTO, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::get_bus_volt(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_BUS_VOLT, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_BUS_VOLT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::get_bus_curr(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_BUS_CURR, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_BUS_CURR, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::get_multi_volt(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_MULTI_VOLT, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_MULTI_VOLT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::get_error_code(int id, uint8_t *code) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_ERR_CODE, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_ERR_CODE, ROSMOS_TIMEOUT);
  code[0] = rx_data->data[0];
  delete rx_data;
  return ret;
}

int AdraApi::get_pos_target(int id, float *target) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_POS_TARGET, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_POS_TARGET, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], target, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_pos_target(int id, float target) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(target, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_POS_TARGET, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_POS_TARGET, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_pos_current(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_POS_CURRENT, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_POS_CURRENT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::get_pos_limit_min(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_POS_LIMIT_MIN, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_POS_LIMIT_MIN, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_pos_limit_min(int id, float pos) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(pos, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_POS_LIMIT_MIN, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_POS_LIMIT_MIN, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_pos_limit_max(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_POS_LIMIT_MAX, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_POS_LIMIT_MAX, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_pos_limit_max(int id, float pos) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(pos, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_POS_LIMIT_MAX, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_POS_LIMIT_MAX, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_pos_limit_diff(int id, float *max) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_POS_LIMIT_DIFF, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_POS_LIMIT_DIFF, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], max, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_pos_limit_diff(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_POS_LIMIT_DIFF, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_POS_LIMIT_DIFF, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_pos_pidp(int id, float *p) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_POS_PIDP, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_POS_PIDP, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], p, 1);
  return ret;
}

int AdraApi::set_pos_pidp(int id, float p) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(p, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_POS_PIDP, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_POS_PIDP, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_pos_smooth_cyc(int id, uint8_t *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_POS_SMOOTH_CYC, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_POS_SMOOTH_CYC, ROSMOS_TIMEOUT);
  buf[0] = rx_data->data[0];
  delete rx_data;
  return ret;
}

int AdraApi::set_pos_smooth_cyc(int id, uint8_t buf) {
  this->_id(id);
  uint8_t txdata[1] = {buf};
  this->_send(ROSMOS_RW::W, ADRA_REG_POS_SMOOTH_CYC, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_POS_SMOOTH_CYC, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_pos_adrc_param(int id, uint8_t i, float *param) {
  this->_id(id);
  uint8_t txdata[1] = {i};
  this->_send(ROSMOS_RW::R, ADRA_REG_POS_ADRC_PARAM, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_POS_ADRC_PARAM, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], param, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_pos_adrc_param(int id, uint8_t i, float param) {
  this->_id(id);
  uint8_t data[4] = {0};
  this->fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  this->_send(ROSMOS_RW::W, ADRA_REG_POS_ADRC_PARAM, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_POS_ADRC_PARAM, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::pos_cal_zero(uint8_t id) {
  this->_id(id);
  uint8_t txdata[1] = {ADRA_REG_POS_CAL_ZERO[0]};
  this->_send(ROSMOS_RW::W, ADRA_REG_POS_CAL_ZERO, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_POS_CAL_ZERO, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_target(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_TARGET, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_TARGET, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_vel_target(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_VEL_TARGET, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VEL_TARGET, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_current(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_CURRENT, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_CURRENT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_limit_min(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_LIMIT_MIN, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_LIMIT_MIN, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_vel_limit_min(int id, float vel) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(vel, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_VEL_LIMIT_MIN, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VEL_LIMIT_MIN, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_limit_max(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_LIMIT_MAX, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_LIMIT_MAX, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_vel_limit_max(int id, float vel) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(vel, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_VEL_LIMIT_MAX, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VEL_LIMIT_MAX, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_limit_diff(int id, float *max) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_LIMIT_DIFF, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_LIMIT_DIFF, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], max, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_vel_limit_diff(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_VEL_LIMIT_DIFF, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VEL_LIMIT_DIFF, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_pidp(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_PIDP, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_PIDP, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_vel_pidp(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_VEL_PIDP, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VEL_PIDP, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_pidi(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_PIDI, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_PIDI, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_vel_pidi(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_VEL_PIDI, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VEL_PIDI, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_smooth_cyc(int id, uint8_t *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_SMOOTH_CYC, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_SMOOTH_CYC, ROSMOS_TIMEOUT);
  buf[0] = rx_data->data[0];
  delete rx_data;
  return ret;
}

int AdraApi::set_vel_smooth_cyc(int id, uint8_t buf) {
  this->_id(id);
  uint8_t txdata[1] = {buf};
  this->_send(ROSMOS_RW::W, ADRA_REG_VEL_SMOOTH_CYC, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VEL_SMOOTH_CYC, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_vel_adrc_param(int id, uint8_t i, float *param) {
  this->_id(id);
  uint8_t txdata[1] = {i};
  this->_send(ROSMOS_RW::R, ADRA_REG_VEL_ADRC_PARAM, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_VEL_ADRC_PARAM, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], param, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_vel_adrc_param(int id, uint8_t i, float param) {
  this->_id(id);
  uint8_t data[4] = {0};
  this->fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  this->_send(ROSMOS_RW::W, ADRA_REG_VEL_ADRC_PARAM, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_VEL_ADRC_PARAM, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_target(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_TARGET, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_TARGET, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_tau_target(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_TAU_TARGET, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TAU_TARGET, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_current(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_CURRENT, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_CURRENT, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_limit_min(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_LIMIT_MIN, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_LIMIT_MIN, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_tau_limit_min(int id, float tau) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(tau, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_TAU_LIMIT_MIN, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TAU_LIMIT_MIN, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_limit_max(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_LIMIT_MAX, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_LIMIT_MAX, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_tau_limit_max(int id, float tau) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(tau, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_TAU_LIMIT_MAX, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TAU_LIMIT_MAX, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_limit_diff(int id, float *max) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_LIMIT_DIFF, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_LIMIT_DIFF, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], max, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_tau_limit_diff(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_TAU_LIMIT_DIFF, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TAU_LIMIT_DIFF, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_pidp(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_PIDP, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_PIDP, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_tau_pidp(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_TAU_PIDP, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TAU_PIDP, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_pidi(int id, float *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_PIDI, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_PIDI, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], buf, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_tau_pidi(int id, float buf) {
  this->_id(id);
  uint8_t txdata[4] = {0};
  this->fp32_to_hex_big(buf, txdata);
  this->_send(ROSMOS_RW::W, ADRA_REG_TAU_PIDI, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TAU_PIDI, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_smooth_cyc(int id, uint8_t *buf) {
  this->_id(id);
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_SMOOTH_CYC, NULL, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_SMOOTH_CYC, ROSMOS_TIMEOUT);
  buf[0] = rx_data->data[0];
  delete rx_data;
  return ret;
}

int AdraApi::set_tau_smooth_cyc(int id, uint8_t buf) {
  this->_id(id);
  uint8_t txdata[1] = {buf};
  this->_send(ROSMOS_RW::W, ADRA_REG_TAU_SMOOTH_CYC, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TAU_SMOOTH_CYC, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}

int AdraApi::get_tau_adrc_param(int id, uint8_t i, float *param) {
  this->_id(id);
  uint8_t txdata[1] = {i};
  this->_send(ROSMOS_RW::R, ADRA_REG_TAU_ADRC_PARAM, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::R, ADRA_REG_TAU_ADRC_PARAM, ROSMOS_TIMEOUT);

  this->hex_to_fp32_big(&rx_data->data[0], param, 1);
  delete rx_data;
  return ret;
}

int AdraApi::set_tau_adrc_param(int id, uint8_t i, float param) {
  this->_id(id);
  uint8_t data[4] = {0};
  this->fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  this->_send(ROSMOS_RW::W, ADRA_REG_TAU_ADRC_PARAM, txdata, NULL);
  ROSMOSType *rx_data = new ROSMOSType();
  int ret = this->_pend(rx_data, ROSMOS_RW::W, ADRA_REG_TAU_ADRC_PARAM, ROSMOS_TIMEOUT);
  delete rx_data;
  return ret;
}
