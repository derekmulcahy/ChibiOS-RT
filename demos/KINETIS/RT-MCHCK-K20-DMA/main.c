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

#define DMAMUX_SOURCE_UART0_RECIEVE     2
#define DMAMUX_SOURCE_UART0_TRANSMIT    3
#define DMAMUX_SOURCE_UART1_RECIEVE     4
#define DMAMUX_SOURCE_UART1_TRANSMIT    5
#define DMAMUX_SOURCE_UART2_RECIEVE     6
#define DMAMUX_SOURCE_UART2_TRANSMIT    7
#define DMAMUX_SOURCE_I2S0_RECEIVE      14
#define DMAMUX_SOURCE_I2S0_TRANSMIT     15
#define DMAMUX_SOURCE_SPI0_RECEIVE      16
#define DMAMUX_SOURCE_SPI0_TRANSMIT     17
#define DMAMUX_SOURCE_I2C0              22
#define DMAMUX_SOURCE_FTM0_CHANNEL0     24
#define DMAMUX_SOURCE_FTM0_CHANNEL1     25
#define DMAMUX_SOURCE_FTM0_CHANNEL2     26
#define DMAMUX_SOURCE_FTM0_CHANNEL3     27
#define DMAMUX_SOURCE_FTM0_CHANNEL4     28
#define DMAMUX_SOURCE_FTM0_CHANNEL5     29
#define DMAMUX_SOURCE_FTM0_CHANNEL6     30
#define DMAMUX_SOURCE_FTM0_CHANNEL7     31
#define DMAMUX_SOURCE_FTM1_CHANNEL0     32
#define DMAMUX_SOURCE_FTM1_CHANNEL1     33
#define DMAMUX_SOURCE_ADC0              40
#define DMAMUX_SOURCE_CMP0              42
#define DMAMUX_SOURCE_CMP1              43
#define DMAMUX_SOURCE_CMT               47
#define DMAMUX_SOURCE_PDB               48
#define DMAMUX_SOURCE_PORTA             49
#define DMAMUX_SOURCE_PORTB             50
#define DMAMUX_SOURCE_PORTC             51
#define DMAMUX_SOURCE_PORTD             52
#define DMAMUX_SOURCE_PORTE             53
#define DMAMUX_SOURCE_DMAMUX_CHANNEL0   54
#define DMAMUX_SOURCE_DMAMUX_CHANNEL1   55
#define DMAMUX_SOURCE_DMAMUX_CHANNEL2   56
#define DMAMUX_SOURCE_DMAMUX_CHANNEL3   57
#define DMAMUX_SOURCE_DMAMUX_CHANNEL4   58
#define DMAMUX_SOURCE_DMAMUX_CHANNEL5   59
#define DMAMUX_SOURCE_DMAMUX_CHANNEL6   60
#define DMAMUX_SOURCE_DMAMUX_CHANNEL7   61
#define DMAMUX_SOURCE_DMAMUX_CHANNEL8   62
#define DMAMUX_SOURCE_DMAMUX_CHANNEL9   63

static vuint8_t src[5] = { 'H', 'e', 'l', 'l', 'o' };
static vuint8_t dst[5] = { 'a', 'b', 'c', 'd', 'e' };

/* DMA channel 0 transfer complete */
OSAL_IRQ_HANDLER(Vector40) {
  OSAL_IRQ_PROLOGUE();
  vuint8_t a = dst[0];
  (void)a;
  vuint8_t b = dst[1];
  (void)b;
  vuint8_t c = dst[2];
  (void)c;
  vuint8_t d = dst[3];
  (void)d;
  vuint8_t e = dst[4];
  (void)e;
  DMA->INT = 0b0001;
  dst[0] = dst[1] = dst[2] = dst[3] = dst[4] = 0;
  OSAL_IRQ_EPILOGUE();
}

/* PIT Channel0  interrupt vector */
OSAL_IRQ_HANDLER(VectorB8) {
  OSAL_IRQ_PROLOGUE();
  /* Clear the Timer Interrupt Flag, TIF = 1 */
  PIT->CHANNEL[0].TFLG |= 1;
  OSAL_IRQ_EPILOGUE();
}

void dmaInit(void) {
  // Enable DMA clocks
  SIM->SCGC6 |= SIM_SCGC6_DMAMUX;
  SIM->SCGC7 |= SIM_SCGC7_DMA;

  /* Disable the DMA multiplexer for channel 0 */
  DMAMUX->CHCFG[0] &= ~(DMAMUX_CHCFGn_ENBL);

  /* Reset DMA engine to 'Normal' operation */
  DMA->CR = 0;

  nvicEnableVector(DMA0_IRQn, 3);
  nvicEnableVector(PITChannel0_IRQn, 8);
}

void dmaStart(void) {
  /* Enable DMA MUX channel 0 for PIT triggers and FTM0 channel 0 source */
  DMAMUX->CHCFG[0] = 0;
  DMAMUX->CHCFG[0] = DMAMUX_CHCFGn_ENBL |
      DMAMUX_CHCFGn_TRIG |
      DMAMUX_CHCFGn_SOURCE(DMAMUX_SOURCE_FTM0_CHANNEL0);

  /* Enable DMA request channel 0 */
  DMA->ERQ |= DMA_ERQ_ERQ0_MASK;

  /* Source data address */
  DMA->TCD[0].SADDR = (uint32_t)&src;

  /* Source signed increment between transfers */
  DMA->TCD[0].SOFF = 1;

  /* Destination data address */
  DMA->TCD[0].DADDR = (uint32_t)&dst;

  /* Current Major iteration count */
  DMA->TCD[0].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(1);

  /* Beginning Major iteration count */
  DMA->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(1);

  /* Minor transfer byte count */
  DMA->TCD[0].NBYTES_MLNO = sizeof(src);

  /* Destination signed increment between transfers */
  DMA->TCD[0].DOFF = 1;

  /* Set source and destination modulo to zero and transfer size to 8-bit */
  DMA->TCD[0].ATTR = DMA_ATTR_SMOD(0) | DMA_ATTR_SSIZE(0) |
      DMA_ATTR_DMOD(0) | DMA_ATTR_DSIZE(0);

  /* Adjustment source address back to beginning at end of transfer */
  DMA->TCD[0].SLAST = DMA_SLAST_SLAST(-sizeof(src));

  /* Adjustment destination address back to beginning at end of transfer */
  DMA->TCD[0].DLAST_SGA = DMA_SLAST_SLAST(-sizeof(dst));

  /* Generate an interrupt at the end of the major loop */
  DMA->TCD[0].CSR = DMA_CSR_INTMAJOR_MASK;
}

void ftm0Init(void) {
  // Enable DMA clock
  SIM->SCGC6 |= SIM_SCGC6_FTM0;

  /* Set PTC1 as TPM0_CH0 output pin */
  PORTC->PCR[1] = PORTx_PCRn_MUX(4);

  /*
   * Select output compare, toggle output on match,
   * enables channel DMA transfer
   * */
  /* CHIE | MSA |  ELSA | DMA */
  FTM0->C0SC = 0x55;

  /* channel value for compare, channel flag is toggled when FTM0_CNT value */
  /* Determines when in the cycle that the flag is toggled */
  /* Must be less than or equal to MOD */
  FTM0->C0V = 1;

  /* modulo value - determines rate at which timer repeats */
  /* For a 48MHz clock the slowest this can go is 48M/128/65535 = 5.7Hz */
  FTM0->MOD = 65535;

  /*
   * Selects system clock as FTM0 clock source, starts FTM0 counter
   * Prescaler of 128 therefore FTM counts in SYSCLK / 128 units.
   */
  FTM0->SC = (0b01 << 3) | 0b111;

  SIM->SCGC6 |= SIM_SCGC6_PIT;

  /* Enable the PIT, MDIS = 0 */
  PIT->MCR &= ~2;

  /* PIT trigger measured in system clock ticks */
  PIT->CHANNEL[0].LDVAL = 48000000;

  /* Enable PIT timer interrupt, TIE = 1 */
  PIT->CHANNEL[0].TCTRL |= 2;

  /* Enable timer0, TEN = 1 */
  PIT->CHANNEL[0].TCTRL |= 1;
}

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

  dmaInit();
  dmaStart();
  ftm0Init();

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
  palSetPadMode(GPIOC, 5, PAL_MODE_ALTERNATIVE_2);  /* SCK  */
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATIVE_2);  /* MOSI */
  palSetPadMode(GPIOD, 3, PAL_MODE_ALTERNATIVE_2);  /* MISO */
  palSetPadMode(GPIOC, 0, PAL_MODE_ALTERNATIVE_2);  /* SS   */

  /*
   *  Initializes the SPI driver 1.
   */
  spiStart(&SPID1, &spi1cfg);

  /*
   * Creates the blinker threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (1) {
    chThdSleepMilliseconds(500);
  }
}
