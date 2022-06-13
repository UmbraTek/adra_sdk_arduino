// Copyright 2022 The UmbraTek Inc. All Rights Reserved.
//
// Software License Agreement (BSD License)
//
// Author: johnson Huang <johnson@umbratek.com>
// =============================================================================
#ifndef ADRA_REG_H
#define ADRA_REG_H
#include <Arduino.h>

static const uint8_t ADRA_REG_UUID[5] = {0x01, 0, 12, NULL, NULL};
static const uint8_t ADRA_REG_SW_VERSION[5] = {0x02, 0, 12, NULL, NULL};
static const uint8_t ADRA_REG_HW_VERSION[5] = {0x03, 0, 12, NULL, NULL};
static const uint8_t ADRA_REG_MULTI_VERSION[5] = {0x04, 0, 12, NULL, NULL};
static const uint8_t ADRA_REG_MECH_RATIO[5] = {0x05, 0, 4, 4, 0};
static const uint8_t ADRA_REG_COM_ID[5] = {0x08, NULL, NULL, 1, 0};
static const uint8_t ADRA_REG_COM_BAUD[5] = {0x09, NULL, NULL, 4, 0};
static const uint8_t ADRA_REG_RESET_ERR[5] = {0x0C, NULL, NULL, 1, 0};
static const uint8_t ADRA_REG_RESET_DRIVER[5] = {0x0D, NULL, NULL, 1, 0};
static const uint8_t ADRA_REG_ERASE_PARM[5] = {0x0E, NULL, NULL, 1, 0};
static const uint8_t ADRA_REG_SAVED_PARM[5] = {0x0F, NULL, NULL, 1, 0};

static const uint8_t ADRA_REG_ELEC_RATIO[5] = {0x10, 0, 4, 4, 0};
static const uint8_t ADRA_REG_MOTION_DIR[5] = {0x11, 0, 1, 1, 0};
static const uint8_t ADRA_REG_IWDG_CYC[5] = {0x12, 0, 4, 4, 0};
static const uint8_t ADRA_REG_TEMP_LIMIT[5] = {0x18, 0, 2, 2, 0};
static const uint8_t ADRA_REG_VOLT_LIMIT[5] = {0x19, 0, 2, 2, 0};
static const uint8_t ADRA_REG_CURR_LIMIT[5] = {0x1A, 0, 4, 4, 0};
static const uint8_t ADRA_REG_BRAKE_PWM[5] = {0x1F, 0, 1, 1, 0};

static const uint8_t ADRA_REG_MOTION_MDOE[5] = {0x20, 0, 1, 1, 0};
static const uint8_t ADRA_REG_MOTION_ENABLE[5] = {0x21, 0, 1, 1, 0};
static const uint8_t ADRA_REG_BRAKE_ENABLE[5] = {0x22, 0, 1, 1, 0};
static const uint8_t ADRA_REG_TEMP_DRIVER[5] = {0x28, 0, 4, NULL, NULL};
static const uint8_t ADRA_REG_TEMP_MOTO[5] = {0x29, 0, 4, NULL, NULL};
static const uint8_t ADRA_REG_BUS_VOLT[5] = {0x2A, 0, 4, NULL, NULL};
static const uint8_t ADRA_REG_BUS_CURR[5] = {0x2B, 0, 4, NULL, NULL};
static const uint8_t ADRA_REG_MULTI_VOLT[5] = {0x2C, 0, 4, NULL, NULL};
static const uint8_t ADRA_REG_ERR_CODE[5] = {0x2F, 0, 1, NULL, NULL};

static const uint8_t ADRA_REG_POS_TARGET[5] = {0x30, 0, 4, 4, 0};
static const uint8_t ADRA_REG_POS_CURRENT[5] = {0x31, 0, 4, NULL, NULL};
static const uint8_t ADRA_REG_POS_LIMIT_MIN[5] = {0x32, 0, 4, 4, 0};
static const uint8_t ADRA_REG_POS_LIMIT_MAX[5] = {0x33, 0, 4, 4, 0};
static const uint8_t ADRA_REG_POS_LIMIT_DIFF[5] = {0x34, 0, 4, 4, 0};
static const uint8_t ADRA_REG_POS_PIDP[5] = {0x35, 0, 4, 4, 0};
static const uint8_t ADRA_REG_POS_SMOOTH_CYC[5] = {0x36, 0, 1, 1, 0};
static const uint8_t ADRA_REG_POS_ADRC_PARAM[5] = {0x39, 1, 4, 5, 0};
static const uint8_t ADRA_REG_POS_CAL_ZERO[5] = {0x3F, NULL, NULL, 1, 0};

static const uint8_t ADRA_REG_VEL_TARGET[5] = {0x40, 0, 4, 4, 0};
static const uint8_t ADRA_REG_VEL_CURRENT[5] = {0x41, 0, 4, NULL, NULL};
static const uint8_t ADRA_REG_VEL_LIMIT_MIN[5] = {0x42, 0, 4, 4, 0};
static const uint8_t ADRA_REG_VEL_LIMIT_MAX[5] = {0x43, 0, 4, 4, 0};
static const uint8_t ADRA_REG_VEL_LIMIT_DIFF[5] = {0x44, 0, 4, 4, 0};
static const uint8_t ADRA_REG_VEL_PIDP[5] = {0x45, 0, 4, 4, 0};
static const uint8_t ADRA_REG_VEL_PIDI[5] = {0x46, 0, 4, 4, 0};
static const uint8_t ADRA_REG_VEL_SMOOTH_CYC[5] = {0x47, 0, 1, 1, 0};
static const uint8_t ADRA_REG_VEL_ADRC_PARAM[5] = {0x49, 1, 4, 5, 0};

static const uint8_t ADRA_REG_TAU_TARGET[5] = {0x50, 0, 4, 4, 0};
static const uint8_t ADRA_REG_TAU_CURRENT[5] = {0x51, 0, 4, NULL, NULL};
static const uint8_t ADRA_REG_TAU_LIMIT_MIN[5] = {0x52, 0, 4, 4, 0};
static const uint8_t ADRA_REG_TAU_LIMIT_MAX[5] = {0x53, 0, 4, 4, 0};
static const uint8_t ADRA_REG_TAU_LIMIT_DIFF[5] = {0x54, 0, 4, 4, 0};
static const uint8_t ADRA_REG_TAU_PIDP[5] = {0x55, 0, 4, 4, 0};
static const uint8_t ADRA_REG_TAU_PIDI[5] = {0x56, 0, 4, 4, 0};
static const uint8_t ADRA_REG_TAU_SMOOTH_CYC[5] = {0x57, 0, 1, 1, 0};
static const uint8_t ADRA_REG_TAU_ADRC_PARAM[5] = {0x59, 1, 4, 5, 0};

// uint8_t ADRA_REG_CPOS_TARGET[5] = {0x60, ' ', ' ', 0, ' '};        // startId endId pos*Axis
// uint8_t ADRA_REG_CTAU_TARGET[5] = {0x61, ' ', ' ', 0, ' '};        // startId endId tau*Axis
// uint8_t ADRA_REG_CPOSTAU_TARGET[5] = {0x62, ' ', ' ', 0, ' '};     // startId endId (pos+tau)*Axis
// uint8_t ADRA_REG_SPOSTAU_CURRENT[5] = {0x68, 0, 8 + 1, ' ', ' '};  // Gets the current position and torque of an actuator
// uint8_t ADRA_REG_CPOSTAU_CURRENT[5] = {0x69, 2, 8 + 1, ' ', ' '};  // startId endId

#endif