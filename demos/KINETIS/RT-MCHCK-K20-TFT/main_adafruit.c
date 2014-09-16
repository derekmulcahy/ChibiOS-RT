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
#include "gfx.h"

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

*/

// Color definitions
#define ILI9341_BLACK   0x0000
#define ILI9341_BLUE    0x001F
#define ILI9341_RED     0xF800
#define ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW  0xFFE0
#define ILI9341_WHITE   0xFFFF

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
  KINETIS_SPI_TAR_8BIT_SLOW
};

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {
  static uint8_t txbuf[5];
  static uint8_t rxbuf[5];

  (void)arg;
  chRegSetThreadName("Blinker");
  while (TRUE) {
    palSetPad(GPIOB, GPIOB_LED);

    /* Send the Manufacturer and Device ID Read command */
    txbuf[0] = 0x9F;

    spiSelect(&SPID1);
    spiExchange(&SPID1, sizeof(txbuf), txbuf, rxbuf);
    spiUnselect(&SPID1);

    chThdSleepMilliseconds(1000);
  }

  return 0;
}

void writecommand(uint8_t c) {
  palClearPad(GPIOD, 1);

  spi_lld_polled_exchange(&SPID1,c);
}

void writedata(uint8_t c) {
  palSetPad(GPIOD, 1);

  spi_lld_polled_exchange(&SPID1,c);
}

int16_t _width = 320;
int16_t _height = 240;

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writecommand(ILI9341_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9341_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ILI9341_RAMWR); // write to RAM
}

// fill a rectangle
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  palSetPad(GPIOD, 1);

  //digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  //digitalWrite(_cs, HIGH);
}

void displayReset(void) {
  palSetPad(GPIOD, 0);
  chThdSleepMilliseconds(5);
  palClearPad(GPIOD, 0);
  chThdSleepMilliseconds(20);
  palSetPad(GPIOD, 0);
  chThdSleepMilliseconds(150);
}

void displayInit(void) {
  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);
  writedata(0x00);
  writedata(0XC1);
  writedata(0X30);

  writecommand(0xED);
  writedata(0x64);
  writedata(0x03);
  writedata(0X12);
  writedata(0X81);

  writecommand(0xE8);
  writedata(0x85);
  writedata(0x00);
  writedata(0x78);

  writecommand(0xCB);
  writedata(0x39);
  writedata(0x2C);
  writedata(0x00);
  writedata(0x34);
  writedata(0x02);

  writecommand(0xF7);
  writedata(0x20);

  writecommand(0xEA);
  writedata(0x00);
  writedata(0x00);

  writecommand(ILI9341_PWCTR1);    //Power control
  writedata(0x23);   //VRH[5:0]

  writecommand(ILI9341_PWCTR2);    //Power control
  writedata(0x10);   //SAP[2:0];BT[3:0]

  writecommand(ILI9341_VMCTR1);    //VCM control
  writedata(0x3e); //¶Ô±È¶Èµ÷½Ú
  writedata(0x28);

  writecommand(ILI9341_VMCTR2);    //VCM control2
  writedata(0x86);  //--

  writecommand(ILI9341_MADCTL);    // Memory Access Control
  writedata(0x48);

  writecommand(ILI9341_PIXFMT);
  writedata(0x55);

  writecommand(ILI9341_FRMCTR1);
  writedata(0x00);
  writedata(0x18);

  writecommand(ILI9341_DFUNCTR);    // Display Function Control
  writedata(0x08);
  writedata(0x82);
  writedata(0x27);

  writecommand(0xF2);    // 3Gamma Function Disable
  writedata(0x00);

  writecommand(ILI9341_GAMMASET);    //Gamma curve selected
  writedata(0x01);

  writecommand(ILI9341_GMCTRP1);    //Set Gamma
  writedata(0x0F);
  writedata(0x31);
  writedata(0x2B);
  writedata(0x0C);
  writedata(0x0E);
  writedata(0x08);
  writedata(0x4E);
  writedata(0xF1);
  writedata(0x37);
  writedata(0x07);
  writedata(0x10);
  writedata(0x03);
  writedata(0x0E);
  writedata(0x09);
  writedata(0x00);

  writecommand(ILI9341_GMCTRN1);    //Set Gamma
  writedata(0x00);
  writedata(0x0E);
  writedata(0x14);
  writedata(0x03);
  writedata(0x11);
  writedata(0x07);
  writedata(0x31);
  writedata(0xC1);
  writedata(0x48);
  writedata(0x08);
  writedata(0x0F);
  writedata(0x0C);
  writedata(0x31);
  writedata(0x36);
  writedata(0x0F);

  writecommand(ILI9341_SLPOUT);    //Exit Sleep
//    if (hwSPI) spi_end();
  chThdSleepMilliseconds(120);
//    if (hwSPI) spi_begin();
  writecommand(ILI9341_DISPOFF);    //Display off
//    if (hwSPI) spi_end();
  chThdSleepMilliseconds(120);
  writecommand(ILI9341_DISPON);    //Display on

}


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

//  displayReset();

  gfxInit();

  gdispClear(Red);
  /*
   * Creates the blinker threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (1) {
//    displayInit();
    chThdSleepMilliseconds(500);
  }
}
