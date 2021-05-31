/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_nimble_FD_PCB.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"
#include "math.h"
#include "configblock.h"

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

static struct {
  float roll;
  float pitch;
  float yaw;
} servoTrims;

static uint16_t act_max = 32767;

void powerDistributionInit(void)

{
  
  #ifndef MOTOR_M1_NEUTRAL
    #define MOTOR_M1_NEUTRAL 0
  #endif
  #ifndef MOTOR_M2_NEUTRAL
    #define MOTOR_M2_NEUTRAL 0
  #endif
  #ifndef MOTOR_M3_NEUTRAL
    #define MOTOR_M3_NEUTRAL 0
  #endif
  #ifndef MOTOR_M4_NEUTRAL
    #define MOTOR_M4_NEUTRAL 0
  #endif

  motorsInit(platformConfigGetMotorMapping());
  DEBUG_PRINT("Using Flapper Drone power distribution\n");
  
  servoTrims.roll = 0.0;
  servoTrims.pitch = 0.0;
  servoTrims.yaw = 0.0;

  // // values used for MAVLab order: #10, #14, #18
  // servoTrims.roll = 0.0;
  // servoTrims.pitch = 0.0;
  // servoTrims.yaw = 0.05;

  // // values used for MAVLab order: #09
  // servoTrims.roll = 0.0;
  // servoTrims.pitch = -0.15;
  // servoTrims.yaw = 0.0;

  // // values used for MAVLab order: #11
  // servoTrims.roll = 0.0;
  // servoTrims.pitch = 0.2;
  // servoTrims.yaw = 0.05;

  // // values used for MAVLab order: #12
  // servoTrims.roll = 0.0;
  // servoTrims.pitch = -0.2;
  // servoTrims.yaw = 0.1;

  // // values used for MAVLab order: #15
  // servoTrims.roll = 0.0;
  // servoTrims.pitch = 0.1;
  // servoTrims.yaw = 0.15;

  // // values used for MAVLab order: #16
  // servoTrims.roll = 0.0;
  // servoTrims.pitch = 0.08;
  // servoTrims.yaw = 0.0;

// // values used for MAVLab order: #17
//   servoTrims.roll = 0.0;
//   servoTrims.pitch = -0.07;
//   servoTrims.yaw = 0.15;

  // // values used for MAVLab order: #19
  // servoTrims.roll = 0.0;
  // servoTrims.pitch = -0.22;
  // servoTrims.yaw = 0.1;

  // servoTrims.pitch = configblockGetCalibPitch()/10;

}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, MOTOR_M1_NEUTRAL);
  motorsSetRatio(MOTOR_M2, MOTOR_M2_NEUTRAL + servoTrims.pitch*act_max);
  motorsSetRatio(MOTOR_M3, MOTOR_M3_NEUTRAL + servoTrims.yaw*act_max);
  motorsSetRatio(MOTOR_M4, MOTOR_M4_NEUTRAL);
}

void powerDistribution(const control_t *control)
{
  static float pitch_ampl = 0.4; // 1 = full servo stroke
  
  motorPower.m2 = limitThrust(MOTOR_M2_NEUTRAL + servoTrims.pitch*act_max + pitch_ampl*control->pitch); // pitch servo
  motorPower.m3 = limitThrust(MOTOR_M3_NEUTRAL + servoTrims.yaw*act_max - control->yaw); // yaw servo
  
  motorPower.m1 = limitThrust( 0.5f * control->roll + control->thrust * (1 + servoTrims.roll) ); // left motor
  motorPower.m4 = limitThrust(-0.5f * control->roll + control->thrust * (1 - servoTrims.roll) ); // right motor

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(_servoTrims)
PARAM_ADD(PARAM_FLOAT, rollTrim, &servoTrims.roll)
PARAM_ADD(PARAM_FLOAT, pitchTrim, &servoTrims.pitch)
PARAM_ADD(PARAM_FLOAT, yawTrim, &servoTrims.yaw)
PARAM_GROUP_STOP(servoTrims)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
