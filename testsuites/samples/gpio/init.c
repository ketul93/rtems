/*
 *  Copyright (c) 2015 Ketul Shah <ketulshah1993 at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/test.h>
#include <bsp/beagleboneblack.h> /* Calls the BBB specific library */
#include <rtems/gpio.h> /* Calls the BSP gpio library */
#include <stdio.h>
#include <stdlib.h>

static void inline delay_sec(int sec)
{
  rtems_task_wake_after(sec*rtems_clock_get_ticks_per_second());
}
/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument argument);

const char rtems_test_name[] = "Gpio Test";

rtems_task Init(
  rtems_task_argument ignored
)
{
  rtems_test_begin();
  printf("Starting Gpio Testing\n");
 
  /* Intializing GPIO API */
  rtems_gpio_initialize();
  static gpio_pin_handle led_usr0_handle, led_usr1_handle, led_usr2_handle;
  static gpio_pin_handle led_usr3_handle;
  static unsigned int status;
 
  status = rtems_gpio_configure_pin_digital_out(&led_usr0_handle, BBB_LED_USR0);
  if (status == GPIO_SUCCESSFUL) {
    rtems_gpio_digital_set(&led_usr0_handle);
    delay_sec(1);
    rtems_gpio_digital_clear(&led_usr0_handle);
    delay_sec(1);
    rtems_gpio_release_pin(&led_usr0_handle);
  }
  status = rtems_gpio_configure_pin_digital_out(&led_usr1_handle, BBB_LED_USR1);
  if (status == GPIO_SUCCESSFUL) {
    rtems_gpio_digital_set(&led_usr1_handle);
    delay_sec(1);
    rtems_gpio_digital_clear(&led_usr1_handle);
    delay_sec(1);
    rtems_gpio_release_pin(&led_usr1_handle);
  }
  status = rtems_gpio_configure_pin_digital_out(&led_usr2_handle, BBB_LED_USR2);
  if (status == GPIO_SUCCESSFUL) {
    rtems_gpio_digital_set(&led_usr2_handle);
    delay_sec(1);
    rtems_gpio_digital_clear(&led_usr2_handle);
    delay_sec(1);
    rtems_gpio_release_pin(&led_usr2_handle);
  }
  status = rtems_gpio_configure_pin_digital_out(&led_usr3_handle, BBB_LED_USR3);
  if (status == GPIO_SUCCESSFUL) {
    rtems_gpio_digital_set(&led_usr3_handle);
    delay_sec(1);
    rtems_gpio_digital_clear(&led_usr3_handle);
    delay_sec(1);
    rtems_gpio_release_pin(&led_usr3_handle);
  }
  printf("Gpio Test Completed\n");
  rtems_test_end();
  exit( 0 );
}

/* NOTICE: the clock driver is enabled */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
 
#define CONFIGURE_MAXIMUM_TASKS            1
#define CONFIGURE_USE_DEVFS_AS_BASE_FILESYSTEM

#define CONFIGURE_MAXIMUM_SEMAPHORES    1
 
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE 

#define CONFIGURE_EXTRA_TASK_STACKS         (2 * RTEMS_MINIMUM_STACK_SIZE)
 
#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION
 
#define CONFIGURE_INIT
#include <rtems/confdefs.h>