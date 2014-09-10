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

#include <time.h>

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

static time_t RTCDateTime2TimeT(const RTCDateTime *dtp) {

  struct tm tm;

  tm.tm_year = dtp->year + 80;
  tm.tm_mon = dtp->month - 1;
  tm.tm_isdst = dtp->dstflag;
  /* tm_wday is calculated by mktime from other fields */
  tm.tm_mday = dtp->day;

  /* Truncate the milliseconds when converting to seconds.*/
  uint32_t seconds = dtp->millisecond / 1000;
  tm.tm_hour = seconds / 3600;
  tm.tm_min = seconds / 60 % 60;
  tm.tm_sec = seconds % 60;

  return mktime(&tm);
}

static void TimeT2RTCDateTime(const time_t tt, RTCDateTime *dtp) {

  struct tm tm;
  localtime_r(&tt, &tm);

  dtp->year = tm.tm_year - 80;
  dtp->month = tm.tm_mon + 1;
  dtp->dstflag = tm.tm_isdst;
  dtp->dayofweek = tm.tm_wday == 0 ? RTC_DAY_SUNDAY : tm.tm_wday;
  dtp->day = tm.tm_mday;
  dtp->millisecond = ((tm.tm_hour * 60 + tm.tm_min) * 60 + tm.tm_sec) * 1000;
}

static void rtc_clock_init(void) {
#if defined(KL2x_MCUCONF)
#if  KINETIS_RTC_HAS_32KHZ_CRYSTAL
#else
  /*
   * Derived from https://community.freescale.com/docs/DOC-94734
   */

  /* Set PCT1 as RTC_CLKIN and select 32 KHz clock source for the RTC module */
  PORTC->PCR[1] |= PORTx_PCRn_MUX(0x1);
  SIM->SOPT1 |= SIM_SOPT1_OSC32KSEL(SIM_SOPT1_OSC32KSEL_RTC_CLKIN);

  /*
   * Set PTC3 as CLKOUT pin and selects the MCGIRCLK clock to output on the
   * CLKOUT pin.
   */
  /* Enable the internal reference clock. MCGIRCLK is active */
  //  MCG->C1 |= MCG_C1_IRCLKEN;

  /* Select the slow internal reference clock source. */
  //  MCG->C2 &= ~(MCG_C2_IRCS);

  //  SIM->SOPT2 |= SIM_SOPT2_CLKOUTSEL(0b100);
  //  PORTC->PCR[3] |= PORTx_PCRn_MUX(0x5);
#endif
#endif /* defined(KL2x_MCUCONF) */

#if defined(K20x_MCUCONF)
#if  KINETIS_RTC_HAS_32KHZ_CRYSTAL
#else
#endif /* KINETIS_RTC_HAS_32KHZ_CRYSTAL */
#endif /* defined(K20x_MCUCONF) */
}

void rtc_lld_start(RTCDriver *rtcp) {

  /* IRQ vector permanently assigned to this driver.*/
  nvicEnableVector(RTCAlarm_IRQn, KINETIS_RTC_ALARM_IRQ_PRIORITY);
  nvicEnableVector(RTCSeconds_IRQn, KINETIS_RTC_SECONDS_IRQ_PRIORITY);

  rtc_clock_init();

  /* Enable software access and interrupts to the RTC module. */
  SIM->SCGC6 |= SIM_SCGC6_RTC;

  /* Clear all RTC registers. */
  RTC->CR = RTC_CR_SWR;
  RTC->CR &= ~RTC_CR_SWR;

  /* If Invalid then set the Time Seconds Register */
  if (RTC->SR & RTC_SR_TIF)
    RTC->TSR = 0x00000000;

  /* Disable the Alarm by setting it in the past */
  RTC->TAR = RTC->TSR - 1;

  /* Set time compensation parameters. */
  /* FIXME: Configure values for correction */
  RTC->TCR = RTC_TCR_CIR(1) | RTC_TCR_TCR(0xFF);

  /* Enable Invalid, Overflow and Seconds interrupts. */
  RTC->IER = RTC_IER_TIIE | RTC_IER_TOIE | RTC_IER_TSIE;

  /* Enable time counter */
  RTC->SR |= RTC_SR_TCE;

  rtcp->state = RTC_STARTED;
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
  RTCD1.state = RTC_STOPPED;
}

/**
 * @brief   Set current time.
 * @note    Fractional part will be silently ignored.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCTime structure
 *
 * @notapi
 */
void rtc_lld_set_time(RTCDriver *rtcp, const RTCDateTime *dtp) {

  if (rtcp->state == RTC_STOPPED) {
    rtc_lld_start(rtcp);
  }

  /* Disable the clock */
  RTC->SR &= ~RTC_SR_TCE;

  RTC->TSR = RTCDateTime2TimeT(dtp);

  /* Enable the clock */
  RTC->SR |= RTC_SR_TCE;
}

/**
 * @brief   Get current time.
 *
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] timespec  pointer to a @p RTCTime structure
 *
 * @notapi
 */
void rtc_lld_get_time(RTCDriver *rtcp, RTCDateTime *dtp) {

  if (rtcp->state == RTC_STOPPED) {
    rtc_lld_start(rtcp);
  }

  TimeT2RTCDateTime(RTC->TSR, dtp);
}

/**
 * @brief   Set alarm time.
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

  (void)alarm;

  if (rtcp->state == RTC_STOPPED) {
    rtc_lld_start(rtcp);
  }
  if (alarmspec != NULL) {
    if (alarmspec->type == RTC_ALARM_DELTA) {
      RTC->TAR = RTC->TSR + alarmspec->u.delta;
    } else if (alarmspec->type == RTC_ALARM_ABSOLUTE) {
      RTC->TAR = RTCDateTime2TimeT(&alarmspec->u.absolute);
    }
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
 * @param[in] rtcp      pointer to RTC driver structure
 * @param[in] alarm     alarm identifier
 * @param[out] alarmspec pointer to a @p RTCAlarm structure
 *
 * @notapi
 */
void rtc_lld_get_alarm(RTCDriver *rtcp,
                       rtcalarm_t alarm,
                       RTCAlarm *alarmspec) {

  (void)alarm;

  if (rtcp->state == RTC_STOPPED) {
    rtc_lld_start(rtcp);
  }

  if (alarmspec->type == RTC_ALARM_DELTA)
    alarmspec->u.delta = RTC->TAR - RTC->TSR;
  else if (alarmspec->type == RTC_ALARM_ABSOLUTE)
    TimeT2RTCDateTime(RTC->TSR, &alarmspec->u.absolute);
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
