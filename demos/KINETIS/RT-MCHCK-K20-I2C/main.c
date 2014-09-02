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
#include "chprintf.h"
#include "tmp101na.h"

static I2CConfig i2ccfg1 = {
  100000
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
   * Serial on D7 (TX) and D6 (RX).
   */
  sdStart(&SD1, NULL);
  palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATIVE_3);
  palSetPadMode(GPIOD, 7, PAL_MODE_ALTERNATIVE_3);

  /*
   * I2C on B0 (SCL) and B1 (SDA).
   */
  i2cStart(&I2CD1, &i2ccfg1);
  palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATIVE_2);
  palSetPadMode(GPIOB, 1, PAL_MODE_ALTERNATIVE_2);

  tmp101naSetConfiguration(TMP101NA_CR_RESOLUTION_12BITS);
  i2cStop(&I2CD1);

  while (1) {
    i2cStart(&I2CD1, &i2ccfg1);
    int t = tmp101naReadTemperature();
    chprintf((BaseSequentialStream *)&SD1, "%d\n", t);
    chThdSleepMilliseconds(1000);
    i2cStop(&I2CD1);
  }
}
