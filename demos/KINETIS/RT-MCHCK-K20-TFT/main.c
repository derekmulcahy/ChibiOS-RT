/*
    ChibiOS/RT - Copyright (C) 2014 Derek Mulcahy

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

#define USE_UGFX    FALSE
#if USE_UGFX
#include "gfx.h"
#else
#include "ili9341.h"
#endif

void spicb(SPIDriver *spip) {

  (void)spip;

  palClearPad(GPIOB, GPIOB_LED);
}

/*
 * SPI1 configuration structure.
 * The slave select line is the PCS4 pin also assigned to GPIOC pin 0.
 */
static const SPIConfig spi1cfg = {
  spicb,
  /* HW dependent part.*/
  KINETIS_SPI_PCS4,
  GPIOC,
  0,
  KINETIS_SPI_TAR_8BIT_FAST
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
   * Activates SPID1. Slave select is configured on GPIOC pin 0. This is
   * PCS4 for the KINETIS DSPI managed slave select.
   */
  palSetPadMode(GPIOC, 5, PAL_MODE_ALTERNATIVE_2);      /* SCK  */
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATIVE_2);      /* MOSI */
  palSetPadMode(GPIOD, 3, PAL_MODE_ALTERNATIVE_2);      /* MISO */
  palSetPadMode(GPIOC, 0, PAL_MODE_ALTERNATIVE_2);      /* SS   */
  palSetPadMode(GPIOD, 0, PAL_MODE_OUTPUT_PUSHPULL);    /* RESET   */
  palSetPadMode(GPIOD, 1, PAL_MODE_OUTPUT_PUSHPULL);    /* D/C   */

  /*
   *  Initializes the SPI driver 1.
   */
  spiStart(&SPID1, &spi1cfg);

#if USE_UGFX
  gfxInit();
#else
  ILI9341_init(&SPID1);
#endif

  volatile systime_t start = chVTGetSystemTime();

  while (1) {
#if USE_UGFX
    gdispClear(Red);
//    chThdSleepMilliseconds(500);
    gdispClear(Green);
//    chThdSleepMilliseconds(500);
    gdispClear(Blue);
//    chThdSleepMilliseconds(500);
#else

    volatile systime_t delta = chVTTimeElapsedSinceX(start);
    (void)delta;
    start = chVTGetSystemTime();

    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
    clearDisplay(&SPID1, 0x07C0);
    clearDisplay(&SPID1, 0x003F);
    clearDisplay(&SPID1, 0xF800);
#endif
  }
}
