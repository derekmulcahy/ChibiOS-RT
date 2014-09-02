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

#include "hal.h"
#include "tmp101na.h"

msg_t tmp101naSetConfiguration(uint8_t cr) {
  const uint8_t config_cmd[] = {TMP101NA_CONFIGURATION_REGISTER, cr};
  i2cAcquireBus(&I2CD1);
  msg_t status = i2cMasterTransmitTimeout(&I2CD1, TMP101NA_BASE_ADDRESS,
                                          config_cmd, sizeof(config_cmd),
                                          NULL,
                                          0,
                                          TIME_INFINITE);
  i2cReleaseBus(&I2CD1);

  return status;
}

int tmp101naReadTemperature(void) {
  const uint8_t read_cmd[1] = {TMP101NA_TEMPERATURE_REGISTER};
  uint8_t rx[2];
  i2cAcquireBus(&I2CD1);
  msg_t status = i2cMasterTransmitTimeout(&I2CD1, TMP101NA_BASE_ADDRESS,
                                          read_cmd, sizeof(read_cmd), rx,
                                          sizeof(rx),
                                          TIME_INFINITE);
  if (status != MSG_OK)
    return TMP101NA_NO_READING;

  int th = rx[0];
  int tl = (rx[1] >> 4) * 100 / 16;
  i2cReleaseBus(&I2CD1);
  return th * 100 + tl;
}
