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

#define TMP101NA_BASE_ADDRESS           0x48

#define TMP101NA_TEMPERATURE_REGISTER   0x00
#define TMP101NA_CONFIGURATION_REGISTER 0x01
#define TMP101NA_TLOW_REGISTER          0x02
#define TMP101NA_THIGH_REGISTER         0x03

#define TMP101NA_CR_ALERT               0x80
#define TMP101NA_CR_RESOLUTION(x)       ((x & 0x3) << 5)
#define TMP101NA_CR_RESOLUTION_9BITS    TMP101NA_CR_RESOLUTION(0)
#define TMP101NA_CR_RESOLUTION_10BITS   TMP101NA_CR_RESOLUTION(1)
#define TMP101NA_CR_RESOLUTION_11BITS   TMP101NA_CR_RESOLUTION(2)
#define TMP101NA_CR_RESOLUTION_12BITS   TMP101NA_CR_RESOLUTION(3)
#define TMP101NA_CR_FAULTS(x)           ((x & 0x3) << 3)
#define TMP101NA_CR_FAULTS_1            TMP101NA_CR_FAULTS(0)
#define TMP101NA_CR_FAULTS_2            TMP101NA_CR_FAULTS(1)
#define TMP101NA_CR_FAULTS_4            TMP101NA_CR_FAULTS(2)
#define TMP101NA_CR_FAULTS_6            TMP101NA_CR_FAULTS(3)
#define TMP101NA_CR_ALERT_POLARITY      0x04
#define TMP101NA_CR_THERMOSTAT_MODE     0x02
#define TMP101NA_CR_SHUTDOWN            0x01

#define TMP101NA_NO_READING             0x8000

msg_t tmp101naSetConfiguration(uint8_t cr);
int tmp101naReadTemperature(void);
