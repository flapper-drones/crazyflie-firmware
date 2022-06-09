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
 * platform_defaults_bolt.h - platform specific default values for the bolt platform
 */

#pragma once

#ifndef __INCLUDED_FROM_PLATFORM_DEFAULTS__
    #pragma GCC error "Do not include this file directly, include platform_defaults.h instead."
#endif

// Defines for default values in the bolt platform

// Default values for battery limits
#define DEFAULT_BAT_LOW_VOLTAGE                   6.4f
#define DEFAULT_BAT_CRITICAL_LOW_VOLTAGE          6.0f
#define DEFAULT_BAT_LOW_DURATION_TO_TRIGGER_SEC   5

// Default value for system shutdown in minutes after radio silence.
// Requires kbuild config ENABLE_AUTO_SHUTDOWN to be activated.
#define DEFAULT_SYSTEM_SHUTDOWN_TIMEOUT_MIN       5

// Default Euler angles defining IMU orientation on the airframe
#define IMU_PHI 0.0f
#define IMU_THETA 0.0f
#define IMU_PSI 0.0f
