/**
 * @file rtems/gpio.h
 *
 * @brief Global GPIO definitions.
 *
 * This include the generalized definitions for GPIO 
 */

/**
 * Copyright (c) 2015 Ketul Shah <ketulshah1993 at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef _RTEMS_GPIO_H
#define _RTEMS_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Returned Error Codes by function */
#define GPIO_SUCCESSFUL        0 /* operation is OK */
#define GPIO_UNKNOWN_PIN       1 /* pin not known by bsp */
#define GPIO_UNCONFIGURED_PIN  2 /* pin unable to configure */
#define GPIO_MISCONFIGURED_PIN 3 /* pin configuration can't match operation */

/* Possible GPIO Pin States */
#define GPIO_PIN_STATE_UNCONFIGURED 0
#define GPIO_PIN_STATE_DIGITAL_OUT  1
#define GPIO_PIN_STATE_DIGITAL_IN   2

/**
 * @brief Structure contains all the required members for GPIO access.
 */
typedef struct
{
  int   pin_number;/* The pin number. */
  void* platform;  /* Opaque hardware specific set up details. */
} gpio_pin_handle;

/**
 * @brief Initializes the GPIO API.
 */
extern void rtems_gpio_initialize(void);
/**
 * @brief Selects a GPIO pin for a digital output.
 */
extern int rtems_gpio_configure_pin_digital_out(
	gpio_pin_handle *, unsigned int );
/**
 * @brief Turns on the given pin.
 */
extern int rtems_gpio_digital_set(gpio_pin_handle *);
/**
 * @brief Turns off the given pin.
 */
extern int rtems_gpio_digital_clear(gpio_pin_handle *);
/**
 * @brief currently configured pin is released and made UNCONFIGURED.
 */
extern int rtems_gpio_release_pin(gpio_pin_handle *);