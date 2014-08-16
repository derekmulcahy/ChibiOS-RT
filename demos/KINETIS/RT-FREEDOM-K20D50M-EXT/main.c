/*
    ChibiOS/RT - Copyright (C) 2006-2014 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "test.h"

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("RedBlinker");
  while (TRUE) {
    palTogglePad(IOPORT3, 3);
    chThdSleepMilliseconds(300);
  }

  return 0;
}

static THD_WORKING_AREA(waThread2, 64);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  chRegSetThreadName("GreenBlinker");
  while (TRUE) {
    palTogglePad(IOPORT4, 4);
    chThdSleepMilliseconds(600);
  }

  return 0;
}

/* Triggered when the button is pressed. The blue led is toggled. */
static void extcb1(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  palTogglePad(IOPORT1, 2);
}

static const EXTConfig extcfg = {
  {
   {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART, extcb1, PORTA, 1}
  }
};

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates serial 1 (UART0) using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  /*
   * Activates the EXT driver 1.
   */
  palSetPadMode(IOPORT1, 1, PAL_MODE_INPUT_PULLUP);
  extStart(&EXTD1, &extcfg);

  TestThread(&SD1);
  while (1) {
    chThdSleepMilliseconds(500);
  }
}
