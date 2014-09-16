#include "ili9341.h"

#define ILI9341_TFTWIDTH    240
#define ILI9341_TFTHEIGHT   320

#define ILI9341_NOP         0x00
#define ILI9341_SWRESET     0x01
#define ILI9341_RDDID       0x04
#define ILI9341_RDDST       0x09

#define ILI9341_SLPIN       0x10
#define ILI9341_SLPOUT      0x11
#define ILI9341_PTLON       0x12
#define ILI9341_NORON       0x13

#define ILI9341_RDMODE      0x0A
#define ILI9341_RDMADCTL    0x0B
#define ILI9341_RDPIXFMT    0x0C
#define ILI9341_RDIMGFMT    0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF      0x20
#define ILI9341_INVON       0x21
#define ILI9341_GAMMASET    0x26
#define ILI9341_DISPOFF     0x28
#define ILI9341_DISPON      0x29

#define ILI9341_CASET       0x2A
#define ILI9341_PASET       0x2B
#define ILI9341_RAMWR       0x2C
#define ILI9341_RAMRD       0x2E

#define ILI9341_PTLAR       0x30
#define ILI9341_MADCTL      0x36
#define ILI9341_PIXFMT      0x3A

#define ILI9341_FRMCTR1     0xB1
#define ILI9341_FRMCTR2     0xB2
#define ILI9341_FRMCTR3     0xB3
#define ILI9341_INVCTR      0xB4
#define ILI9341_DFUNCTR     0xB6

#define ILI9341_PWCTR1      0xC0
#define ILI9341_PWCTR2      0xC1
#define ILI9341_PWCTR3      0xC2
#define ILI9341_PWCTR4      0xC3
#define ILI9341_PWCTR5      0xC4
#define ILI9341_VMCTR1      0xC5
#define ILI9341_VMCTR2      0xC7

#define ILI9341_RDID1       0xDA
#define ILI9341_RDID2       0xDB
#define ILI9341_RDID3       0xDC
#define ILI9341_RDID4       0xDD

#define ILI9341_GMCTRP1     0xE0
#define ILI9341_GMCTRN1     0xE1
/*
#define ILI9341_PWCTR6      0xFC
*/

// Color definitions
#define ILI9341_BLACK       0x0000
#define ILI9341_BLUE        0x001F
#define ILI9341_RED         0xF800
#define ILI9341_GREEN       0x07E0
#define ILI9341_CYAN        0x07FF
#define ILI9341_MAGENTA     0xF81F
#define ILI9341_YELLOW      0xFFE0
#define ILI9341_WHITE       0xFFFF

static const uint8_t init_commands[] = {
    4, 0xEF, 0x03, 0x80, 0x02,
    4, 0xCF, 0x00, 0XC1, 0X30,
    5, 0xED, 0x64, 0x03, 0X12, 0X81,
    4, 0xE8, 0x85, 0x00, 0x78,
    6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
    2, 0xF7, 0x20,
    3, 0xEA, 0x00, 0x00,
    2, ILI9341_PWCTR1, 0x23,                // Power control
    2, ILI9341_PWCTR2, 0x10,                // Power control
    3, ILI9341_VMCTR1, 0x3e, 0x28,          // VCM control
    2, ILI9341_VMCTR2, 0x86,                // VCM control2
    2, ILI9341_MADCTL, 0x48,                // Memory Access Control
    2, ILI9341_PIXFMT, 0x55,
    3, ILI9341_FRMCTR1, 0x00, 0x18,
    4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27,   // Display Function Control
    2, 0xF2, 0x00,                          // Gamma Function Disable
    2, ILI9341_GAMMASET, 0x01,              // Gamma curve selected
    16, ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E,
        0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,     // Set Gamma
    16, ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31,
        0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,     // Set Gamma
    0
};

inline static void  command_mode(void) {
  palClearPad(GPIOD, 1);
}

inline static void data_mode(void) {
  palSetPad(GPIOD, 1);
}

inline static void send_command(SPIDriver *spip, uint8_t byte) {
  command_mode();
  spiSend(spip, 1, &byte);
  data_mode();
}

void clearDisplay(SPIDriver *spip, uint16_t color) {
  send_command(spip, 0x2A);
  int8_t vx[4] = { 0, 0, 0, 239 };
  spiSend(spip, 4, vx);

  send_command(spip, 0x2B);
  int8_t vy[4] = { 0, 0, (320 - 1) >> 8, (320 - 1) & 0xFF };
  spiSend(spip, 4, vy);

  send_command(spip, ILI9341_RAMWR);
  uint16_t c[] = {
    color, color, color, color, color, color, color, color,
    color, color, color, color, color, color, color, color,
    color, color, color, color, color, color, color, color,
    color, color, color, color, color, color, color, color,
    color, color, color, color, color, color, color, color,
    color, color, color, color, color, color, color, color,
    color, color, color, color, color, color, color, color,
    color, color, color, color, color, color, color, color,
  };
  for (uint32_t i = 0; i < ((240 * 320) / (sizeof(c) / 2)) ; i++) {
//    spi_lld_polled_exchange(spip, color >> 8);
//    spi_lld_polled_exchange(spip, color & 0xFF);
    spiSend(spip, sizeof(c), c);
  }
}

void ILI9341_init(SPIDriver *spip) {

  palSetPad(GPIOD, 0);
  chThdSleepMilliseconds(5);
  palClearPad(GPIOD, 0);
  chThdSleepMilliseconds(20);
  palSetPad(GPIOD, 0);
  chThdSleepMilliseconds(150);

  const uint8_t *addr = init_commands;
  while (*addr) {
      uint8_t count = *addr++ - 1;
      send_command(spip, *addr++);
      spiSend(spip, count, addr);
      addr += count;
  }
  send_command(spip, ILI9341_SLPOUT);    // Exit Sleep
  chThdSleepMilliseconds(120);
  send_command(spip, ILI9341_DISPON);    // Display on
}
