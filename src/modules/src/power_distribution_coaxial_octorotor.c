/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 * power_distribution_coaxial_octorotor.c - Coaxial octorotor power distribution code
 */

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "motors.h"
#include "platform_defaults.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

// Motors IDs define
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3
#define MOTOR_M5  4
#define MOTOR_M6  5
#define MOTOR_M7  6
#define MOTOR_M8  7

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

uint16_t * powerDistribution(const control_t *control)
{
  static uint16_t motorPower[NBR_OF_MOTORS];
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;
  motorPower[MOTOR_M1] = limitThrust(control->thrust - r + p + control->yaw);
  motorPower[MOTOR_M2] = limitThrust(control->thrust - r - p - control->yaw);
  motorPower[MOTOR_M3] =  limitThrust(control->thrust + r - p + control->yaw);
  motorPower[MOTOR_M4] =  limitThrust(control->thrust + r + p - control->yaw);
  motorPower[MOTOR_M5] = limitThrust(control->thrust - r + p - control->yaw);
  motorPower[MOTOR_M6] = limitThrust(control->thrust - r - p + control->yaw);
  motorPower[MOTOR_M7] =  limitThrust(control->thrust + r - p - control->yaw);
  motorPower[MOTOR_M8] =  limitThrust(control->thrust + r + p + control->yaw);

  for (int i = 0; i < NBR_OF_MOTORS; i++)
    {
      if (motorPower[i] < idleThrust) motorPower[i] = idleThrust;
    }
  return motorPower;
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
