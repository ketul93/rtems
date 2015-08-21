/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBone Black BSP definitions.
 */

/**
 * Copyright (c) 2015 Ketul Shah <ketulshah1993 at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_BEAGLE_ADC_H
#define LIBBSP_ARM_BEAGLE_ADC_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief ADC intialization.
 */
extern void rtems_adc_init();
/**
 * @brief Reads the ADC value for input from pin.
 */
extern int rtems_adc_read(unsigned int);
