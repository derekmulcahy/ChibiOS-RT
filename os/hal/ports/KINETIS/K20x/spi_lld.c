/*
    ChibiOS/HAL - Copyright (C) 2014 Derek Mulcahy

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

/*
 * TODO: 16 bit data
 *       Allow selection of top bits of PUSHR (PCS, TAR) per transaction.
 *       DMA for receive.
 *       DMA for transmit.
 */

/**
 * @file    KINETIS/spi_lld.c
 * @brief   KINETIS SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*
 * Enable a mode which enables receive interrupts when we have no
 * receive buffer. The alternative is to ignore received bytes
 * when we do not have a buffer to store them.
 */
#define TEST_ALWAYS_RECEIVE     TRUE

/*
 * Transmit just one byte per interrupt. The alternative is to fill the
 * transmit fifo to the limit. This may result in receive overflows.
 */
#define TEST_TRANSMIT_ONE       FALSE

/*
 * Receive using DMA.
 */
#define TEST_SPI_RX_DMA         TRUE
#define SPI0_DMAMUX_CHANNEL     0
#define SPI0_DMA_RX_CHANNEL     0

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SPI0 driver identifier.*/
#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

volatile uint8_t rxDummy;

static void spi_start_xfer(SPIDriver *spip, bool polling)
{
  volatile DMA_TypeDef *dma = DMA;

  /*
   * Enable the DSPI peripheral in master mode.
   * On receive overflow, new data will overwrite old data.
   * Set slave selects to be active low.
   * Clear the TX and RX FIFOs.
   * */
  spip->spi->MCR = SPIx_MCR_MSTR |
      SPIx_MCR_ROOE |
      SPIx_MCR_PCSIS(KINETIS_SPI_PCS_ALL) |
      SPIx_MCR_CLR_TXF | SPIx_MCR_CLR_RXF;

  /* If we are not polling then enable interrupts */
  if (!polling) {

    /* Enable transmit fill, receive, receive and end of queue interrupts */
    spip->spi->RSER = SPIx_RSER_TFFF_RE | SPIx_RSER_RFDF_RE |
        SPIx_RSER_EOQF_RE;

#if TEST_SPI_RX_DMA
    /* Select DMA interrupt request instead of SPI0 interrupt request */
    spip->spi->RSER |= SPIx_RSER_RFDF_DIRS;

    rxDummy = 42;

    DMA->TCD[SPI0_DMA_RX_CHANNEL].DADDR = (uint32_t)(spip->rxbuf ? spip->rxbuf : &rxDummy);
    DMA->TCD[SPI0_DMA_RX_CHANNEL].DOFF = spip->rxbuf ? 1 : 0;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].NBYTES_MLNO = spip->nbytes;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].SLAST = -spip->nbytes;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].BITER_ELINKNO = 1;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].CITER_ELINKNO = 1;

    /* Set bit 0 in Enable Request Register (ERQ) by writing 0 to SERQ */
    DMA->SERQ = SPI0_DMA_RX_CHANNEL;
#endif /* TEST_SPI_RX_DMA */
  }
}

static void spi_stop_xfer(SPIDriver *spip)
{
  /* Halt the DSPI peripheral */
  spip->spi->MCR = SPIx_MCR_MSTR |
      SPIx_MCR_ROOE |
      SPIx_MCR_PCSIS(KINETIS_SPI_PCS_ALL) |
      SPIx_MCR_CLR_TXF | SPIx_MCR_CLR_RXF |
      SPIx_MCR_HALT;

  /* Clear all interrupt enables */
  spip->spi->RSER = 0;

  /* Clear all the flags which are currently set. */
  spip->spi->SR |= spip->spi->SR;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

OSAL_IRQ_HANDLER(Vector70) {
  OSAL_IRQ_PROLOGUE();

  volatile DMA_TypeDef *dma = DMA;
  SPIDriver *spip = &SPID1;
  vuint32_t sr = spip->spi->SR;

  /* Handle Receive FIFO Drain interrupt */
#if TEST_ALWAYS_RECEIVE
  while (spip->rxidx < spip->nbytes && (sr & SPIx_SR_RFDF)) {
#else
  if (spip->rxbuf && (sr & SPIx_SR_RFDF)) {
#endif /* TEST_ALWAYS_RECEIVE */

    /* Pop the data out of the fifo */
    uint8_t data = spip->spi->POPR;

    /* If we have an rxbuf and it has space then store the data */
    if (spip->rxbuf && spip->rxidx < spip->nbytes) {
      spip->rxbuf[spip->rxidx++] = data;
    } else {
      spip->rxidx++;
    }

    /* Clear the Receive FIFO Drain Flag */
    spip->spi->SR = SPIx_SR_RFDF;

#if TEST_ALWAYS_RECEIVE
    sr = spip->spi->SR;
#endif
  }

  /* Handle End of Queue interrupt */
  if (sr & SPIx_SR_EOQF) {

    spip->spi->SR = SPIx_SR_EOQF;

    spi_stop_xfer(spip);

    _spi_isr_code(spip);
  }

  /* Handle Transmit FIFO Fill interrupt */
#if TEST_TRANSMIT_ONE
  if (sr & SPIx_SR_TFFF) {
#else
    while ((spip->nbytes != spip->txidx) && (sr & SPIx_SR_TFFF)) {
#endif
    /* The data to send is either the next tx byte or a filler byte */
    uint8_t data = spip->txbuf ? spip->txbuf[spip->txidx] : 0x00;

    /* Calculate the number of bytes remaining and increment the index */
    size_t remaining = spip->nbytes - spip->txidx++;

    /* Push the data into the FIFO, CONT if more bytes, EOQ if last byte */
    spip->spi->PUSHR = (remaining > 1 ? SPIx_PUSHR_CONT : SPIx_PUSHR_EOQ) |
        SPIx_PUSHR_PCS(spip->config->pcs) |
        SPIx_PUSHR_TXDATA(data);

    /* Clear the Transmit FIFO Fill Flag */
    spip->spi->SR = SPIx_SR_TFFF;

#if !TEST_TRANSMIT_ONE
    /* Reload the Status Register */
    sr = spip->spi->SR;
#endif
  }

  OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(Vector40) {
  OSAL_IRQ_PROLOGUE();

  volatile DMA_TypeDef *dma = DMA;
  volatile SPIDriver *spip = &SPID1;

  /* Clear bit 0 in Interrupt Request Register (INT) by writing 0 to CINT */
  DMA->CINT = SPI0_DMA_RX_CHANNEL;

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {
#if KINETIS_SPI_USE_SPI0
  spiObjectInit(&SPID1);
#endif

}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_STOP) {

#if KINETIS_SPI_USE_SPI0
    if (&SPID1 == spip) {

      nvicEnableVector(SPI0_IRQn, KINETIS_SPI_SPI0_IRQ_PRIORITY);

      /* Enable the clock for SPI0 */
      SIM->SCGC6 |= SIM_SCGC6_SPI0;

      SPID1.spi = SPI0;

      if (spip->config->tar0) {
        spip->spi->CTAR[0] = spip->config->tar0;
      } else {
        spip->spi->CTAR[0] = KINETIS_SPI_TAR0_DEFAULT;
      }

      spip->spi->CTAR[1] = KINETIS_SPI_TAR1_DEFAULT;
    }
#endif

#if TEST_SPI_RX_DMA
    nvicEnableVector(DMA0_IRQn, KINETIS_SPI_DMA0_IRQ_PRIORITY);

    SIM->SCGC6 |= SIM_SCGC6_DMAMUX;
    SIM->SCGC7 |= SIM_SCGC7_DMA;

    volatile DMA_TypeDef *dma = DMA;

    /* Clear DMA error flags and DMAMUX channel configurations. */
    DMA->ERR = 0x0F;

    // enable requests
    // Rx, select SPI Rx FIFO
    DMAMUX->CHCFG[SPI0_DMAMUX_CHANNEL] = DMAMUX_CHCFGn_ENBL | DMAMUX_CHCFGn_SOURCE(16);

    // configure DMA mux, RX
    DMA->TCD[SPI0_DMA_RX_CHANNEL].SADDR = (uint32_t)&SPI0->POPR;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].SOFF = 0;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0);
    DMA->TCD[SPI0_DMA_RX_CHANNEL].BITER_ELINKNO = 1;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].CITER_ELINKNO = 1;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].SLAST = 0;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].DLASTSGA = 0;
    DMA->TCD[SPI0_DMA_RX_CHANNEL].CSR = DMA_CSR_DREQ_MASK | DMA_CSR_INTMAJOR_MASK;
#endif

  }

  /* SPI setup and enable.*/
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  /* If in ready state then disables the SPI clock.*/
  if (spip->state == SPI_READY) {

#if TEST_SPI_RX_DMA
    SIM->SCGC6 &= ~SIM_SCGC6_DMAMUX;
    SIM->SCGC7 &= ~SIM_SCGC7_DMA;

    nvicDisableVector(DMA0_IRQn);
#endif

#if KINETIS_SPI_USE_SPI0
    if (&SPID1 == spip) {
      /* SPI halt.*/
      spip->spi->MCR |= SPIx_MCR_HALT;
    }
#endif

    nvicDisableVector(SPI0_IRQn);

    /* Disable the clock for SPI0 */
    SIM->SCGC6 &= ~SIM_SCGC6_SPI0;
  }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  /* If we are not using the DSPI managed chip select then assert the SS */
  if (!spip->config->pcs) {
    palClearPad(spip->config->ssport, spip->config->sspad);
  }
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *          This has no effect if we are using the KINETIS PCS mode to
 *          manage slave select.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

  /* If we are not using the DSPI managed chip select then deassert the SS */
  if (!spip->config->pcs) {
    palSetPad(spip->config->ssport, spip->config->sspad);
  }
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {

  spip->nbytes = n;
  spip->rxbuf = NULL;
  spip->rxidx = 0;
  spip->txbuf = NULL;
  spip->txidx = 0;

  spi_start_xfer(spip, false);
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  spip->nbytes = n;
  spip->rxbuf = rxbuf;
  spip->rxidx = 0;
  spip->txbuf = txbuf;
  spip->txidx = 0;

  spi_start_xfer(spip, false);
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  spip->nbytes = n;
  spip->rxbuf = NULL;
  spip->rxidx = 0;
  spip->txbuf = (void *)txbuf;
  spip->txidx = 0;

  spi_start_xfer(spip, false);
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

  spip->nbytes = n;
  spip->rxbuf = rxbuf;
  spip->rxidx = 0;
  spip->txbuf = NULL;
  spip->txidx = 0;

  spi_start_xfer(spip, false);
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  spi_start_xfer(spip, true);

  spip->spi->PUSHR = SPIx_PUSHR_PCS(spip->config->pcs) | SPIx_PUSHR_TXDATA(frame);

  while ((spip->spi->SR & SPIx_SR_RFDF) == 0)
    ;

  frame = spip->spi->POPR;

  spi_stop_xfer(spip);

  return frame;
}

#endif /* HAL_USE_SPI */

/** @} */
