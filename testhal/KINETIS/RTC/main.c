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

void rtc_cb(RTCDriver *rtcp, rtcevent_t event) {

  (void)rtcp;

  switch(event) {
  case RTC_EVENT_SECOND:
    palTogglePad(GPIOB, 19);    // Green LED
    break;
  case RTC_EVENT_ALARM:
    palTogglePad(GPIOB, 18);    // Red LED
    break;
  case RTC_EVENT_OVERFLOW:
    palTogglePad(GPIOD, 1);     // Blue LED
    break;
  }
}

static const RTCAlarm alarmspec[1] = {
  /* 12/31/2014 @ 23:59:59 UTC - Happy New Year! */
  { 1420070399 }
};

static rtcalarm_t alarm = 1;

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

  rtcSetAlarm(&RTCD1, alarm, alarmspec);
  rtcSetCallback(&RTCD1, rtc_cb);

  while (1) {
    chThdSleepMilliseconds(3000);
  }
}
