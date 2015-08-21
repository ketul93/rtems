#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/test.h>
#include <bsp.h> /* for device driver prototypes */
#include <bsp/gpio.h>

#include <assert.h>
#include <stdlib.h>


/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument ignored);

rtems_gpio_irq_state edge_test_1(void * arg);
rtems_gpio_irq_state edge_test_2(void * arg);


uint32_t LED1, LED2;
uint32_t SW1, SW2;

const char rtems_test_name[] = "LIBGPIO_TEST";

rtems_task Init(rtems_task_argument ignored)
{
  rtems_status_code sc;

  /* Pin numbers. */
  LED1 = BBB_LED_USR0;
  LED2 = BBB_LED_USR1;
  SW1 = BBB_P8_15;
  SW2 = BBB_P8_18;

  rtems_test_begin();

  /* Initializes the GPIO API */
  sc = rtems_gpio_initialize();
  assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_gpio_request_pin(LED1, DIGITAL_OUTPUT, false, false, NULL);
  assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_gpio_request_pin(LED2, DIGITAL_OUTPUT, false, false,  NULL);
  assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_gpio_request_pin(SW1, DIGITAL_INPUT, false, false, NULL);
  assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_gpio_request_pin(SW2, DIGITAL_INPUT, false, false, NULL);
  assert(sc == RTEMS_SUCCESSFUL);

  printk("GPIO request pin configured\n");

  sc = rtems_gpio_resistor_mode(SW1, PULL_DOWN);
  assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_gpio_resistor_mode(SW2, PULL_UP);
  assert(sc == RTEMS_SUCCESSFUL);

  rtems_gpio_set(LED1);
 
  /* Keeps the program running, so interrupts can be tested. */
  while (1);

  rtems_test_end();
  exit(0);
}
/* NOTICE: the clock driver is enabled */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
 
#define CONFIGURE_MAXIMUM_TASKS            4
#define CONFIGURE_USE_DEVFS_AS_BASE_FILESYSTEM

#define CONFIGURE_MAXIMUM_SEMAPHORES    5
 
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE 
#define CONFIGURE_INIT_TASK_PRIORITY 10

#define CONFIGURE_EXTRA_TASK_STACKS         (4 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_INIT_TASK_STACK_SIZE    (2 * RTEMS_MINIMUM_STACK_SIZE)
 
#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION
#define CONFIGURE_INIT_TASK_INITIAL_MODES RTEMS_DEFAULT_MODES
 
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
