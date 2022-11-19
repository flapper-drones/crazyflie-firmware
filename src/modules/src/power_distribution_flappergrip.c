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
 * power_distribution_flappergrip.c - Power distribution code for the Flapper Nimble+ with a gripper
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "platform_defaults.h"
#include "debug.h"
#include "math.h"
#include "extrx.h"
#include "crtp_commander.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

// Motors IDs define
#ifdef CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
  #define MOTOR_LEFT  0
  #define MOTOR_RIGHT  3
  #define SERVO_PITCH  1
  #define SERVO_YAW  2
  #define SERVO_GRIPPER 4
#else
  #define MOTOR_LEFT  1
  #define MOTOR_RIGHT  3
  #define SERVO_PITCH  0
  #define SERVO_YAW  2
  #define SERVO_GRIPPER 4
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

struct flapperConfig_s {
  uint8_t pitchServoNeutral;
  uint8_t yawServoNeutral;
  uint8_t gripServoOpen;
  uint8_t gripServoClosed;
  uint8_t gripServoState;
  int8_t rollBias;
  uint16_t maxThrust;
};

struct flapperConfig_s flapperConfig = {
  .pitchServoNeutral = 50,
  .yawServoNeutral = 50,
  .gripServoOpen = 75,
  .gripServoClosed = 25,
  .gripServoState = 0,
  .rollBias = 0,
  .maxThrust = 60000,
};

static float thrust;
static uint16_t act_max = 65535;

static float pitch_ampl = 0.4f; // 1 = full servo stroke

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

uint8_t flapperConfigGripClosed(void)
{
  return flapperConfig.gripServoClosed;
}

uint8_t flapperConfigGripOpen(void)
{
  return flapperConfig.gripServoOpen;
}

int8_t flapperConfigRollBias(void)
{
  return limitServoNeutral(flapperConfig.rollBias);
}

void flapperSetGripServoState(uint8_t value)
{
  flapperConfig.gripServoState = value;
}

void powerDistributionInit(void)
{
    DEBUG_PRINT("Using Flapper power distribution | PCB revD (2022) or newer, with gripper\n");
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
  static uint8_t gripServoPosition;
  static uint8_t gripServoState;
  thrust = fmin(control->thrust, flapperConfig.maxThrust);

  if (getCPPMAux1Value() || getExtRxAux1Value() || (flapperConfig.gripServoState > 0)) gripServoState = 1;
  else gripServoState = 0;
    
  flapperConfig.pitchServoNeutral = limitServoNeutral(flapperConfig.pitchServoNeutral);
  flapperConfig.yawServoNeutral = limitServoNeutral(flapperConfig.yawServoNeutral);
  gripServoPosition = flapperConfig.gripServoOpen * gripServoState + flapperConfig.gripServoClosed * (1 - gripServoState);

  motorPower[SERVO_PITCH] = limitThrust(flapperConfig.pitchServoNeutral*act_max/100.0f + pitch_ampl*control->pitch); // pitch servo
  motorPower[SERVO_YAW] = limitThrust(flapperConfig.yawServoNeutral*act_max/100.0f - control->yaw); // yaw servo
  motorPower[MOTOR_LEFT] = limitThrust( 0.5f * control->roll + thrust * (1.0f + flapperConfig.rollBias/100.0f) ); // left motor
  motorPower[MOTOR_RIGHT] = limitThrust(-0.5f * control->roll + thrust * (1.0f - flapperConfig.rollBias/100.0f) ); // right motor
  motorPower[SERVO_GRIPPER] = limitThrust(gripServoPosition*act_max/100.0f); // gripper servo
    
  if (motorPower[MOTOR_LEFT] < idleThrust) {
    motorPower[MOTOR_LEFT] = idleThrust;
  }
  if (motorPower[MOTOR_RIGHT] < idleThrust) {
    motorPower[MOTOR_RIGHT] = idleThrust;
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

/**
 *
 * Flapper Drone configration parameters
 */
PARAM_GROUP_START(flapper)
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
/**
 * @brief Gripper servo ON position <0%; 100%> (default 75%)
 *
 * The parameter sets the ON position of the gripper servo
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servGripOpen, &flapperConfig.gripServoOpen)
/**
 * @brief Gripper servo OFF position <0%; 100%> (default 25%)
 *
 * The parameter sets the OFF position of the gripper servo
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servGripClose, &flapperConfig.gripServoClosed)
/**
 * @brief Gripper servo state (default 0)
 *
 * The parameter sets the state of the gripper servo
 */
PARAM_ADD(PARAM_UINT8, servGripState, &flapperConfig.gripServoState)
/**
 * @brief Max Thrust <0; 65535> (default 60000)
 *
 * The parameter sets the max thrust limit
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, flapperMaxThrust, &flapperConfig.maxThrust)

PARAM_GROUP_STOP(flapper)
