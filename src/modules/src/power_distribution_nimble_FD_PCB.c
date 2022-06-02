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

struct flapperConfig_s {
  uint8_t pitchServoNeutral;
  uint8_t yawServoNeutral;
  int8_t rollBias;
};

struct flapperConfig_s flapperConfig = {
  .pitchServoNeutral = 50,
  .yawServoNeutral = 50,
  .rollBias = 0,
};

static float thrust;

static uint16_t act_max = 65535;

#ifndef NIMBLE_MAX_THRUST
  #define NIMBLE_MAX_THRUST 60000.0f
#endif


#ifdef ENABLE_PWM_EXTENDED
  static uint16_t motor_zero = 9362; // for extended PWM (stroke = 1.4 ms): motor_min = (act_max - act_max/1.4)/2
  static float pwm_extended_ratio = 1.4;
  static float pitch_ampl = 0.4f/1.4f; // 1 = full servo stroke
#else
  static uint16_t motor_zero = 0;
  static float pwm_extended_ratio = 1.0;
  static float pitch_ampl = 0.4f; // 1 = full servo stroke
#endif

uint8_t limitServoNeutral(uint8_t value)
{
  if(value > 75)
  {
    value = 75;
  }
  else if(value < 25)
  {
    value = 25;
  }

  return (uint8_t)value;
}

int8_t limitRollBias(uint8_t value)
{
  if(value > 25)
  {
    value = 25;
  }
  else if(value < -25)
  {
    value = -25;
  }

  return (uint8_t)value;
}

uint8_t flapperConfigPitchNeutral(void)
{
  return limitServoNeutral(flapperConfig.pitchServoNeutral);
}

uint8_t flapperConfigYawNeutral(void)
{
  return limitServoNeutral(flapperConfig.yawServoNeutral);
}

int8_t flapperConfigRollBias(void)
{
  return limitServoNeutral(flapperConfig.rollBias);
}

void powerDistributionInit(void)

{
  #ifdef NIMBLE_USE_CF2
  motorsInit(motorMapDefaltConBrushless);
  DEBUG_PRINT("Using Flapper Drone power distribution | CF2.1\n");
  #else
  motorsInit(platformConfigGetMotorMapping());
  DEBUG_PRINT("Using Flapper Drone power distribution | CF Bolt\n");
  #endif
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

void powerDistribution(motors_thrust_t* motorPower, const control_t *control)
{
  thrust = fmin(control->thrust, NIMBLE_MAX_THRUST);
  
  flapperConfig.pitchServoNeutral=limitServoNeutral(flapperConfig.pitchServoNeutral);
  flapperConfig.yawServoNeutral=limitServoNeutral(flapperConfig.yawServoNeutral);

  motorPower->m2 = limitThrust(flapperConfig.pitchServoNeutral*act_max/100.0f + pitch_ampl*control->pitch); // pitch servo
  motorPower->m3 = limitThrust(flapperConfig.yawServoNeutral*act_max/100.0f - control->yaw); // yaw servo
  motorPower->m1 = motor_zero + 1.0f/pwm_extended_ratio * limitThrust( 0.5f * control->roll + thrust * (1.0f + flapperConfig.rollBias/100.0f) ); // left motor
  motorPower->m4 = motor_zero + 1.0f/pwm_extended_ratio * limitThrust(-0.5f * control->roll + thrust * (1.0f - flapperConfig.rollBias/100.0f) ); // right motor

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    motorsSetRatio(MOTOR_M1, motorPower->m1);
    motorsSetRatio(MOTOR_M2, motorPower->m2);
    motorsSetRatio(MOTOR_M3, motorPower->m3);
    motorsSetRatio(MOTOR_M4, motorPower->m4);
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

/**
 *
 * Flapper Drone configration parameters
 */
PARAM_GROUP_START(_flapper)
/**
 * @brief Roll bias <-25%; 25%> (default 0%)
 *
 * This parameter can be used if uneven performance of the left and right flapping mechanaisms and/or wings
 * is observed, which in flight results in a drift in roll/sideways flight. Positive values make the drone roll 
 * more to the right, negative values to the left.
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, motBiasRoll, &flapperConfig.rollBias)
/**
 * @brief Pitch servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the pitch servo, such that the left and right wing-pairs are
 * aligned when observed from the side. If in flight you observe too much drift forward (nose down) increase the value
 * and vice versa if the drift is backward (nose up).
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servPitchNeutr, &flapperConfig.pitchServoNeutral)
/**
 * @brief Yaw servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the yaw servo, such that the yaw control arm is pointed spanwise. If in flight 
 * you observe drift in the clock-wise direction, increase this parameter and vice-versa if the drift is counter-clock-wise.
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servYawNeutr, &flapperConfig.yawServoNeutral)

PARAM_GROUP_STOP(_flapper)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
