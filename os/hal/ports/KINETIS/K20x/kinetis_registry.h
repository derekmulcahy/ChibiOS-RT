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
 * @file    K20x/kinetis_registry.h
 * @brief   K20x capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef _KINETIS_REGISTRY_H_
#define _KINETIS_REGISTRY_H_

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    K20x capabilities
 * @{
 */
/* EXT attributes.*/

#define KINETIS_PORTA_IRQ_VECTOR    VectorE0
#define KINETIS_PORTB_IRQ_VECTOR    VectorE4
#define KINETIS_PORTC_IRQ_VECTOR    VectorE8
#define KINETIS_PORTD_IRQ_VECTOR    VectorEC
#define KINETIS_PORTE_IRQ_VECTOR    VectorF0

/* ADC attributes.*/
#define KINETIS_HAS_ADC0            TRUE
#define KINETIS_ADC0_IRC_VECTOR     Vector98

/* RTC attributes.*/
#define KINETIS_HAS_RTC                 TRUE
#define KINETIS_RTC_ALARM_IRQ_VECTOR    VectorB0
#define KINETIS_RTC_SECONDS_IRQ_VECTOR  VectorB4

/** @} */

#endif /* _KINETIS_REGISTRY_H_ */

/** @} */
