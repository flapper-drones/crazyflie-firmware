/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB & Flapper Drones (https:\\flapper-drones.com)
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
 * power_distribution_flapper.c - Power distribution code for the Flapper Nimble+
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"

#include "debug.h"
#include "math.h"
#include "cfassert.h"

#include "filter.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

struct flapperConfig_s {
  uint8_t pitchServoNeutral;
  uint8_t yawServoNeutral;
  uint16_t maxThrust;
  float rollCutoff;
  float pitchCutoff;
  float yawCutoff;
};

struct flapperConfig_s flapperConfig = {
  .pitchServoNeutral = 50,
  .yawServoNeutral = 50,
  .maxThrust = 60000,
  .rollCutoff = 20.0,
  .pitchCutoff = 20.0,
  .yawCutoff = 5.0,
};

static float thrust;
static uint16_t act_max = 65535;
static uint16_t cmd_max = 32767;

static float pitch_ampl = 0.4; // 1 is full servo stroke, negative to flip the servo direction
static float yaw_ampl = 1.0;

static lpf2pData rollLpf;
static lpf2pData pitchLpf;
static lpf2pData yawLpf;

#if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
  uint32_t idPitch = 1;
  uint32_t idYaw = 2;
#else
  uint32_t idPitch = 0;
  uint32_t idYaw = 2;
#endif

static uint8_t limitServoNeutral(uint8_t value)
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

int powerDistributionMotorType(uint32_t id)
{
  int type = 1;
  if (id == idPitch || id == idYaw)
  {
    type = 0;
  }

  return type;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  uint16_t stopRatio = 0;
  if (id == idPitch)
  {
    stopRatio = flapperConfig.pitchServoNeutral*act_max/100.0f;
  }
  else if (id == idYaw)
  {
    stopRatio = flapperConfig.yawServoNeutral*act_max/100.0f;
  }

  return stopRatio;
}

void powerDistributionInit(void)
{
  #if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    DEBUG_PRINT("Using Flapper power distribution | PCB revB (2021)\n");
  #else
    DEBUG_PRINT("Using Flapper power distribution | PCB revD (2022) or newer\n");
  #endif

}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

uint16_t limitThrust(int32_t value, int32_t min, int32_t max, bool* isCapped)
{
  if (value < min) {
    return min;
  }

  if (value > max) {
    *isCapped = true;
    return max;
  }

  return value;
}

float limitCmd(float value)
{
  if (value < -cmd_max) {
    return -cmd_max;
  }

  if (value > cmd_max) {
    return cmd_max;
  }

  return value;
}

static int16_t cmdRoll = 0;
static int16_t cmdPitch = 0;
static int16_t cmdYaw = 0;

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  // Only legacy mode is currently supported
  ASSERT(control->controlMode == controlModeLegacy);

  thrust = fmin(control->thrust, flapperConfig.maxThrust);

  flapperConfig.pitchServoNeutral=limitServoNeutral(flapperConfig.pitchServoNeutral);
  flapperConfig.yawServoNeutral=limitServoNeutral(flapperConfig.yawServoNeutral);

  if (thrust > 0)
  {
    cmdRoll = limitCmd(lpf2pApply(&rollLpf, control->roll));
    cmdPitch = limitCmd(lpf2pApply(&pitchLpf, control->pitch));
    cmdYaw = limitCmd(lpf2pApply(&yawLpf, control->yaw));
  }
  else
  {
    cmdRoll = 0;
    cmdPitch = 0;
    cmdYaw = 0;
    
    lpf2pInit(&rollLpf, RATE_MAIN_LOOP, flapperConfig.rollCutoff);
    lpf2pInit(&pitchLpf, RATE_MAIN_LOOP, flapperConfig.pitchCutoff);
    lpf2pInit(&yawLpf, RATE_MAIN_LOOP, flapperConfig.yawCutoff);
  }
  
  #if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    motorThrustUncapped->motors.m2 = flapperConfig.pitchServoNeutral*act_max / 100.0f + 1.0f * pitch_ampl * cmdPitch; // pitch servo
    motorThrustUncapped->motors.m3 = flapperConfig.yawServoNeutral*act_max / 100.0f - 1.0f * yaw_ampl * cmdYaw; // yaw servo
    motorThrustUncapped->motors.m1 =  0.5f * cmdRoll + 1.0f * thrust; // left motor
    motorThrustUncapped->motors.m4 = -0.5f * cmdRoll + 1.0f * thrust; // right motor
  #else
    motorThrustUncapped->motors.m1 = flapperConfig.pitchServoNeutral*act_max / 100.0f + 1.0f * pitch_ampl * cmdPitch; // pitch servo
    motorThrustUncapped->motors.m3 = flapperConfig.yawServoNeutral*act_max / 100.0f - 1.0f * yaw_ampl * cmdYaw; // yaw servo
    motorThrustUncapped->motors.m2 =  0.5f * cmdRoll + 1.0f * thrust; // left motor
    motorThrustUncapped->motors.m4 = -0.5f * cmdRoll + 1.0f * thrust; // right motor
  #endif
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  bool isCapped = false;

  #if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    motorPwm->motors.m2 = limitThrust(motorThrustBatCompUncapped->motors.m2, 0, UINT16_MAX, &isCapped); // pitch servo
    motorPwm->motors.m3 = limitThrust(motorThrustBatCompUncapped->motors.m3, 0, UINT16_MAX, &isCapped); // yaw servo
    motorPwm->motors.m1 = limitThrust(motorThrustBatCompUncapped->motors.m1, idleThrust, UINT16_MAX, &isCapped); // left motor
    motorPwm->motors.m4 = limitThrust(motorThrustBatCompUncapped->motors.m4, idleThrust, UINT16_MAX, &isCapped); // right motor
  #else
    motorPwm->motors.m1 = limitThrust(motorThrustBatCompUncapped->motors.m1, 0, UINT16_MAX, &isCapped); // pitch servo
    motorPwm->motors.m3 = limitThrust(motorThrustBatCompUncapped->motors.m3, 0, UINT16_MAX, &isCapped); // yaw servo
    motorPwm->motors.m2 = limitThrust(motorThrustBatCompUncapped->motors.m2, idleThrust, UINT16_MAX, &isCapped); // left motor
    motorPwm->motors.m4 = limitThrust(motorThrustBatCompUncapped->motors.m4, idleThrust, UINT16_MAX, &isCapped); // right motor
  #endif

  return isCapped;
}

uint32_t powerDistributionGetIdleThrust() {
  return idleThrust;
}

float powerDistributionGetMaxThrust() {
  // Unknown
  ASSERT(false);
  return 0.0f;
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

/**
 *
 * Flapper Drone configration parameters
 */
PARAM_GROUP_START(flapper)
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
/**
 * @brief Maximal thrust (default 60000)
 *
 * The parameter sets the maximal thrust, lowering the value will leave higher margin for roll control
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, flapperMaxThrust, &flapperConfig.maxThrust)
/**
 * @brief Roll filter cutoff frequency in Hz (default 20)
 *
 * The parameter sets the cutoff frequency of roll command filter.
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, rollCut, &flapperConfig.rollCutoff)
/**
 * @brief Pitch filter cutoff frequency in Hz (default 20)
 *
 * The parameter sets the cutoff frequency of pitch command filter.
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitchCut, &flapperConfig.pitchCutoff)
/**
 * @brief Yaw filter cutoff frequency in Hz (default 5)
 *
 * The parameter sets the cutoff frequency of yaw command filter.
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yawCut, &flapperConfig.yawCutoff)

PARAM_GROUP_STOP(flapper)
