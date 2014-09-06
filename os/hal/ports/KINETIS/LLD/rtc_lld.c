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

/**
 * @file    KINETIS/LLD/rtc_lld.c
 * @brief   KINETIS RTC subsystem low level driver header.
 *
 * @addtogroup RTC
 * @{
 */

#include "hal.h"

#if HAL_USE_RTC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define SIM_SOPT1_OSC32KSEL_OSC32KCLK   0
#define SIM_SOPT1_OSC32KSEL_RTC_CLKIN   2
#define SIM_SOPT1_OSC32KSEL_LPO_1KHZ    3

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief RTC driver identifier.
 */
RTCDriver RTCD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*
 * Derived from https://community.freescale.com/docs/DOC-94734
 */

void rtc_kl2x_init() {
  /* Enable the internal reference clock. MCGIRCLK is active */
//  MCG->C1 |= MCG_C1_IRCLKEN;

  /* Select the slow internal reference clock source. */
//  MCG->C2 &= ~(MCG_C2_IRCS);

  /* Set PCT1 as RTC_CLKIN and select 32 KHz clock source for the RTC module */
  PORTC->PCR[1] |= PORTx_PCRn_MUX(0x1);
  SIM->SOPT1 |= SIM_SOPT1_OSC32KSEL(SIM_SOPT1_OSC32KSEL_RTC_CLKIN);

  /*
   * Set PTC3 as CLKOUT pin and selects the MCGIRCLK clock to output on the
   * CLKOUT pin.
   */
//  SIM->SOPT2 |= SIM_SOPT2_CLKOUTSEL(0b100);
//  PORTC->PCR[3] |= PORTx_PCRn_MUX(0x5);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   RTC Alarm interrupt handler.
 * @note    Interrupt handler for Alarm, Invalid and Overflow flags.
 *
 * @isr
 */
CH_IRQ_HANDLER(KINETIS_RTC_ALARM_IRQ_VECTOR) {

  CH_IRQ_PROLOGUE();

  vuint32_t sr = RTC->SR;

  if (sr & RTC_SR_TAF) {
    /*
     * Clear the Time Alarm Flag by writing to the Time Alarm Register
     * and disable the Alarm by setting it in the past
     */
    RTC->TAR = RTC->TSR - 1;
    if (RTCD1.callback)
      RTCD1.callback(&RTCD1, RTC_EVENT_ALARM);
  } else if (sr & RTC_SR_TIF) {
    /* Clear the Time Invalid Flag by writing to the Time Seconds Register */
    RTC->TSR = 0;
    if (RTCD1.callback)
      RTCD1.callback(&RTCD1, RTC_EVENT_OVERFLOW);
  } else if (sr & RTC_SR_TOF) {
    /* Clear the Time Overflow Flag by writing to the Time Seconds Register */
    RTC->TSR = 0;
    if (RTCD1.callback)
      RTCD1.callback(&RTCD1, RTC_EVENT_OVERFLOW);
  }

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   RTC Seconds interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(KINETIS_RTC_SECONDS_IRQ_VECTOR) {

  CH_IRQ_PROLOGUE();

  if (RTCD1.callback)
    RTCD1.callback(&RTCD1, RTC_EVENT_SECOND);

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initialize RTC.
 *
 * @notapi
 */
void rtc_lld_init(void){
  /* IRQ vector permanently assigned to this driver.*/
  nvicEnableVector(RTCAlarm_IRQn, KINETIS_RTC_ALARM_IRQ_PRIORITY);
  nvicEnableVector(RTCSeconds_IRQn, KINETIS_RTC_SECONDS_IRQ_PRIORITY);

  rtc_kl2x_init();

  /* Enable software access and interrupts to the RTC module. */
  SIM->SCGC6 |= SIM_SCGC6_RTC;

  /* Clear all RTC registers. */
  RTC->CR = RTC_CR_SWR;
  RTC->CR &= ~RTC_CR_SWR;

  if (RTC->SR & RTC_SR_TIF)
    RTC->TSR = 0x00000000;

  /* Disable the Alarm by setting it in the past */
  RTC->TAR = RTC->TSR - 1;

  /* Set time compensation parameters. */
  RTC->TCR = RTC_TCR_CIR(1) | RTC_TCR_TCR(0xFF);

  /* Enable Alarm, Invalid, Overflow and Seconds interrupts. */
  RTC->IER = RTC_IER_TAIE | RTC_IER_TIIE | RTC_IER_TOIE | RTC_IER_TSIE;

  /* Enable time counter */
  RTC->SR |= RTC_SR_TCE;
}

/**
 * @brief   Set current time.
 * @note    Fractional part will be silently ignored. There is no possibility
 *          to change it on STM32F1xx platform.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCTime structure
 *
 * @notapi
 */
void rtc_lld_set_time(RTCDriver *rtcp, const RTCDateTime *timespec) {

  (void)rtcp;

  /* FIXME: The time has to be calculated from the timespec fields */
  RTC->TSR = timespec->millisecond;
}

/**
 * @brief   Get current time.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCTime structure
 *
 * @notapi
 */
void rtc_lld_get_time(RTCDriver *rtcp, RTCDateTime *timespec) {

  (void)rtcp;

  /* FIXME: The time has to be stored into the timespec fields */
  timespec->millisecond = RTC->TSR;
}

/**
 * @brief   Set alarm time.
 *
 * @note      Default value after BKP domain reset is 0xFFFFFFFF
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[in] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_set_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm,
                       const RTCAlarm *alarmspec) {

  (void)rtcp;
  (void)alarm;

  if (alarmspec != NULL) {
    RTC->TAR = alarmspec->tv_sec;
    /* Enable Time Alarm Interrupt */
    RTC->IER |= RTC_IER_TAIE;
  }
  else {
    /* Set the Alarm time to one second ago */
    RTC->TAR = RTC->TSR - 1;
    /* Disable Time Alarm Interrupt */
    RTC->IER &= ~RTC_IER_TAIE;
  }
}

/**
 * @brief   Get current alarm.
 * @note    If an alarm has not been set then the returned alarm specification
 *          is not meaningful.
 *
 * @note    Default value after BKP domain reset is 0xFFFFFFFF.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[out] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_get_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm,
                       RTCAlarm *alarmspec) {

  (void)rtcp;
  (void)alarm;

  alarmspec->tv_sec = RTC->TAR;
}

/**
 * @brief   Enables or disables RTC callbacks.
 * @details This function enables or disables callbacks, use a @p NULL pointer
 *          in order to disable a callback.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] callback  callback function pointer or @p NULL
 *
 * @notapi
 */
void rtc_lld_set_callback(RTCDriver *rtcp, rtccb_t callback) {

  (void)rtcp;

  if (callback != NULL) {

    rtcp->callback = callback;
    /* IRQ sources enabled only after setting up the callback.*/
    RTC->IER |= RTC_IER_TSIE;
  }
  else {
    RTC->IER &= ~RTC_IER_TSIE;
    /* Callback set to NULL only after disabling the IRQ sources.*/
    rtcp->callback = NULL;
  }
}

#include "chrtclib.h"

/**
 * @brief   Get current time in format suitable for usage in FatFS.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @return              FAT time value.
 *
 * @api
 */
uint32_t rtc_lld_get_time_fat(RTCDriver *rtcp) {
  uint32_t fattime;
  struct tm timp;

  rtcGetTimeTm(rtcp, &timp);

  fattime  = (timp.tm_sec)       >> 1;
  fattime |= (timp.tm_min)       << 5;
  fattime |= (timp.tm_hour)      << 11;
  fattime |= (timp.tm_mday)      << 16;
  fattime |= (timp.tm_mon + 1)   << 21;
  fattime |= (timp.tm_year - 80) << 25;

  return fattime;
}
#endif /* HAL_USE_RTC */

/** @} */
