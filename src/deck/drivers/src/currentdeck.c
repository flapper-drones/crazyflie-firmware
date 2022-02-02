/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 BitCraze AB
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
 * currentdeck.c: Current sensor driver
 */

#define DEBUG_MODULE "CURRENT"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
// #include "range.h"
// #include "static_mem.h"

#include "currentdeck.h"

// #include "cf_math.h"

static float reading_last = 0.0;
static float current_last = 0.0;
static float current = 0.0;

static float cursense_gain = 100.0; // voltage amplifier gain
static float cursense_resistance = 0.004; // current sense resistor resistance in Ohms
static float cursense_offset = 0.0; // voltage offset

static bool isInit;

void currentDeckInit(DeckInfo* info)
{
  if (isInit)
    return;

  xTaskCreate(currentDeckTask, CURRENTDECK_TASK_NAME, CURRENTDECK_TASK_STACKSIZE, NULL, CURRENTDECK_TASK_PRI, NULL);

  isInit = true;
}

bool currentDeckTest(void)
{
  bool testStatus;
  testStatus = true;

  if (!isInit)
    return false;

  return testStatus;
}

void currentDeckTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(1));

    reading_last = analogReadVoltage(DECK_GPIO_TX2);
    // current_last = 36.7f*reading_last/3.0f-18.3f;

    current_last = reading_last/(cursense_gain*cursense_resistance) - cursense_offset;

    // simple low pass filter
    static float alpha = 0.975;
    current = alpha*current + (1.0f - alpha)*current_last;
  }
}

static const DeckDriver current_deck = {
  .vid = 0xBC,
  .pid = 0x09,
  .name = "bcCurrentDeck",
  .usedGpio = DECK_USING_PA2,

  .init = currentDeckInit,
  .test = currentDeckTest,
};

DECK_DRIVER(current_deck);



PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcCurrentDeck, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(current)
LOG_ADD(LOG_FLOAT, v_raw, &reading_last)
LOG_ADD(LOG_FLOAT, i_raw, &current_last)
LOG_ADD(LOG_FLOAT, current, &current)
LOG_GROUP_STOP(current)