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
  uint8_t pitchServoNeutral;
  uint8_t yawServoNeutral;
  int8_t rollBias;
} flapperConfig;

static uint16_t act_max = 65535;

#ifdef ENABLE_PWM_EXTENDED
  static uint16_t motor_zero = 9362; // for extended PWM (stroke = 1.4 ms): motor_min = (act_max - act_max/1.4)/2
  static float pwm_extended_ratio = 1.4;
  static float pitch_ampl = 0.4f/1.4f; // 1 = full servo stroke
#else
  static uint16_t motor_zero = 0;
  static float pwm_extended_ratio = 1.0;
  static float pitch_ampl = 0.4f; // 1 = full servo stroke
#endif


void powerDistributionInit(void)

{
  motorsInit(platformConfigGetMotorMapping());
  DEBUG_PRINT("Using Flapper Drone power distribution\n");
  
  // Reading out servo trims stored in EEPROM
  flapperConfig.pitchServoNeutral = configblockGetServoNeutralPitch();
  flapperConfig.yawServoNeutral = configblockGetServoNeutralYaw();
  flapperConfig.rollBias = configblockGetMotorBiasRoll();
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
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, limitThrust(flapperConfig.pitchServoNeutral*act_max/100.0f));
  motorsSetRatio(MOTOR_M3, limitThrust(flapperConfig.yawServoNeutral*act_max/100.0f));
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
  motorPower.m2 = limitThrust(flapperConfig.pitchServoNeutral*act_max/100.0f + pitch_ampl*control->pitch); // pitch servo
  motorPower.m3 = limitThrust(flapperConfig.yawServoNeutral*act_max/100.0f - control->yaw); // yaw servo
  motorPower.m1 = motor_zero + 1.0f/pwm_extended_ratio * limitThrust( 0.5f * control->roll + control->thrust * (1.0f + flapperConfig.rollBias/100.0f) ); // left motor
  motorPower.m4 = motor_zero + 1.0f/pwm_extended_ratio * limitThrust(-0.5f * control->roll + control->thrust * (1.0f - flapperConfig.rollBias/100.0f) ); // right motor

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

PARAM_GROUP_START(_flapper)
PARAM_ADD(PARAM_INT8, motBiasRoll, &flapperConfig.rollBias)
PARAM_ADD(PARAM_UINT8, servPitchNeutr, &flapperConfig.pitchServoNeutral)
PARAM_ADD(PARAM_UINT8, servYawNeutr, &flapperConfig.yawServoNeutral)
PARAM_GROUP_STOP(_flapper)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
