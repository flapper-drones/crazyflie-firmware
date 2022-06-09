/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 *
 * platform_defaults_flapper.h - platform specific default values for the Flapper platform
 */

#pragma once

#ifndef __INCLUDED_FROM_PLATFORM_DEFAULTS__
    #pragma GCC error "Do not include this file directly, include platform_defaults.h instead."
#endif

// Defines for default values in the flapper platform

// Default values for battery limits
#define DEFAULT_BAT_LOW_VOLTAGE                   6.4f
#define DEFAULT_BAT_CRITICAL_LOW_VOLTAGE          6.0f
#define DEFAULT_BAT_LOW_DURATION_TO_TRIGGER_SEC   5

// Default value for system shutdown in minutes after radio silence.
// Requires kbuild config ENABLE_AUTO_SHUTDOWN to be activated.
#define DEFAULT_SYSTEM_SHUTDOWN_TIMEOUT_MIN       5

// Default values for Nimble+ 2022
//////////////////////////////////
#define IMU_PHI                   0.0f
#define IMU_THETA                -90.0f
#define IMU_PSI                   180.0f

// External receiver settings
/////////////////////////////
// #define EXTRX_ALT_HOLD // enable altitude hold with external Rx
// #define EXTRX_ARMING // enable arming with external Rx (setup via "Brushless handling" compile flags)
// #define EXTRX_TAER // use TAER channel mapping instead of the default AETR - Aileron(Roll), Elevator(Pitch), Thrust, Rudder(Yaw)

// RGB LED settings //
//////////////////////
// #define TURN_OFF_LED
// #define LEDRING_DEFAULT_EFFECT 7
// #define LED_RING_NBR_LEDS 12

// Better baro hold with no additional sensors //
/////////////////////////////////////////////////
// #define IMPROVED_BARO_Z_HOLD

// Limit maximal thrust (100% = 65535) //
/////////////////////////////////////////
// #define NIMBLE_MAX_THRUST 53000.0f

// Disable tumble check //
//////////////////////////
#define SUPERVISOR_TUMBLE_CHECK_DISABLE

// Do not change the settings below, unless if you know what you are doing //
/////////////////////////////////////////////////////////////////////////////

// Applicable when using CF2 as flight controller
// #define MEASURE_VBAT_ON_PA3 // requires modification of the PCB: 10k / 1k voltage divider on PA3 and R44 removed

// #define YAW_MAX_DELTA 30.0f // Keep the yaw setpoint within +/- YAW_MAX_DELTA from the current heading
// #define PID_FILTER_ALL
// #define CPPM_USE_PA3 // CPPM pin, other alternatives:  PA7 (default), PA2(TX2), PA3(RX2), PB4(IO_3), PB5(IO_2) or PB8(IO_1)