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
 * platform_defaults.h - platform specific default values
 */

#pragma once

#define __INCLUDED_FROM_PLATFORM_DEFAULTS__

#ifdef CONFIG_PLATFORM_CF2
    #include "platform_defaults_cf2.h"
#endif
#ifdef CONFIG_PLATFORM_BOLT
    #include "platform_defaults_bolt.h"
#endif
#ifdef CONFIG_PLATFORM_TAG
    #include "platform_defaults_tag.h"
#endif
#ifdef CONFIG_PLATFORM_FLAPPER
    #include "platform_defaults_flapper.h"
#endif

// Default IMU alignment angles
#ifndef IMU_PHI
    #define IMU_PHI     0.0f
#endif
#ifndef IMU_THETA
    #define IMU_THETA   0.0f
#endif
#ifndef IMU_PSI
    #define IMU_PSI     0.0f
#endif

// Keep the yaw setpoint within +/- YAW_MAX_DELTA from the current heading if nonzero
#ifndef YAW_MAX_DELTA
    #define YAW_MAX_DELTA     0.0f
#endif