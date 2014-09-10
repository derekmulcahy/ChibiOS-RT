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
#include "chrtclib.h"

static const RTCAlarm alarmspec1[1] = {
  /* 10 seconds hence */
  { RTC_ALARM_DELTA, .u.delta = 10 }
};

static const RTCAlarm alarmspec2[1] = {
  /* December 31st 2014, 11:59:59PM EDT, Happy New Year! */
  { RTC_ALARM_ABSOLUTE, .u.absolute = { 34, 12, 1, 3, 31, 86399000 } }
};

static rtcalarm_t alarm_id = 1;

void rtc_cb(RTCDriver *rtcp, rtcevent_t event) {

  (void)rtcp;

  switch(event) {
  case RTC_EVENT_SECOND:
    palTogglePad(GPIOB, 19);    // Green LED
    break;
  case RTC_EVENT_ALARM:
    palTogglePad(GPIOB, 18);    // Red LED
    rtcSetAlarm(&RTCD1, alarm_id, alarmspec2);
    break;
  case RTC_EVENT_OVERFLOW:
    palTogglePad(GPIOD, 1);     // Blue LED
    break;
  }
}

/*
 * Application entry point.
 */
int main(void) {
  palSetPad(GPIOB, 18);       // Red LED
  palSetPad(GPIOB, 19);       // Green LED
  palSetPad(GPIOD, 1);        // Blue LED

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Enable RTC_CLOCKOUT on PTE0 */
  palSetPadMode(GPIOE, 0, PAL_MODE_ALTERNATIVE_4);

  /* The RTC is output on the RTC_CLKOUT pin. */
  SIM->SOPT2 &= ~SIM_SOPT2_RTCCLKOUTSEL;

  /* 10th September 2014, 11:44:37AM EST */
  RTCDateTime then = { 34, 9, 0, 3, 10, 4227700 };
  rtcSetTime(&RTCD1, &then);

  chThdSleepMilliseconds(3000);

  RTCDateTime now;
  rtcGetTime(&RTCD1, &now);

  uint32_t delta = now.millisecond -  then.millisecond;
  if (delta != 3000) {
    palTogglePad(GPIOD, 1);     // Blue LED
  }

  rtcSetAlarm(&RTCD1, alarm_id, alarmspec1);
  rtcSetCallback(&RTCD1, rtc_cb);

  while (1) {
    chThdSleepMilliseconds(3000);
  }
}
