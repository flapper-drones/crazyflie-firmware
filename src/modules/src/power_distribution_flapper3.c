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
 * power_distribution_flapper3.c - Power distribution code for the Flapper Nimble Triple
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

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

struct flapperConfig_s {
  uint8_t yawServoNeutral;
  uint16_t maxThrust;
};

struct flapperConfig_s flapperConfig = {
  .yawServoNeutral = 50,
  .maxThrust = 60000,
};

static float thrust;
static uint16_t act_max = 65535;

uint32_t idYaw = 1;

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
  if (id == idYaw)
  {
    type = 0;
  }

  return type;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  uint16_t stopRatio = 0;
  if (id == idYaw)
  {
    stopRatio = flapperConfig.yawServoNeutral*act_max/100.0f;
  }

  return stopRatio;
}

void powerDistributionInit(void)
{
  DEBUG_PRINT("Using Flapper Triple power distribution\n");
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

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  // Only legacy mode is currently supported
  ASSERT(control->controlMode == controlModeLegacy);

  thrust = fmin(control->thrust, flapperConfig.maxThrust);

  flapperConfig.yawServoNeutral=limitServoNeutral(flapperConfig.yawServoNeutral);

  static float sqrtOf3=1.73205;
  motorThrustUncapped->motors.m2 = flapperConfig.yawServoNeutral*act_max/100.0f + 0.7f*control->yaw; // yaw servo

  motorThrustUncapped->motors.m4 = - sqrtOf3/3.0f * control->pitch + thrust; // rear motor
  motorThrustUncapped->motors.m1 = 0.5f * control->roll + sqrtOf3/6.0f * control->pitch + thrust; // left motor
  motorThrustUncapped->motors.m3 = -0.5f * control->roll + sqrtOf3/6.0f * control->pitch + thrust; // right motor

}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  bool isCapped = false;

  motorPwm->motors.m2 = limitThrust(motorThrustBatCompUncapped->motors.m2, 0, UINT16_MAX, &isCapped); // yaw servo
  
  motorPwm->motors.m4 = limitThrust(motorThrustBatCompUncapped->motors.m4, idleThrust, UINT16_MAX, &isCapped); // rear motor
  motorPwm->motors.m1 = limitThrust(motorThrustBatCompUncapped->motors.m1, idleThrust, UINT16_MAX, &isCapped); // left motor
  motorPwm->motors.m3 = limitThrust(motorThrustBatCompUncapped->motors.m3, idleThrust, UINT16_MAX, &isCapped); // right motor
  
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
PARAM_GROUP_START(flapper3)
/**
 * @brief Yaw servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the yaw servo, such that the yaw control arm is pointed spanwise. If in flight
 * you observe drift in the clock-wise direction, increase this parameter and vice-versa if the drift is counter-clock-wise.
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servYawNeutr, &flapperConfig.yawServoNeutral)
/**
 * @brief Yaw servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the yaw servo, such that the yaw control arm is pointed spanwise. If in flight
 * you observe drift in the clock-wise direction, increase this parameter and vice-versa if the drift is counter-clock-wise.
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, flapperMaxThrust, &flapperConfig.maxThrust)

PARAM_GROUP_STOP(flapper3)
