/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief Global BSP definitions.
 */
 
/**
 * Copyright (c) 2015 Ketul Shah <ketulshah1993 at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */
 
#include <libcpu/am335x.h>
#include <rtems.h>
#include <rtems/gpio.h>
#include <bsp/irq.h>
#include <bsp/beagleboneblack.h>
#include <bsp.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

/**
 * @brief GPIO API mutex atributes.
 */
#define MUTEX_ATRIBUTES                         \
  ( RTEMS_LOCAL                                 \
    | RTEMS_PRIORITY                            \
    | RTEMS_BINARY_SEMAPHORE                    \
    | RTEMS_INHERIT_PRIORITY                    \
    | RTEMS_NO_PRIORITY_CEILING                 \
    )

#define OBTAIN_LOCK(s)  if(rtems_semaphore_obtain(s,                     \
                                                  RTEMS_WAIT,            \
                                                  RTEMS_NO_TIMEOUT       \ 
                                                  ) != RTEMS_SUCCESSFUL) \
                          printf("Semaphore not obtained\n");

#define RELEASE_LOCK(s) if(rtems_semaphore_release(s) != RTEMS_SUCCESSFUL) \
                          printf("Semaphore not released\n");

/* GPIO bank pin number as per TRM of AM335X */
static unsigned int gpio_bank_pin[GPIO_PIN_COUNT]; 
/* GPIO bank determines register of AM335X */
static unsigned int gpio_bank[GPIO_PIN_COUNT];
/* Pin states for all GPIO pins*/
static unsigned int gpio_pin_state[GPIO_PIN_COUNT];
/* Variable for gpio initialization */
static bool is_initialized = false;
/* Total number of gpio banks */
static int gpio_bank_count = GPIO_PIN_COUNT / GPIO_PINS_PER_BANK;
/* Semaphore for avoiding race condition */
static rtems_id bank_lock;

static const uint32_t gpio_bank_addrs[] = 
  { AM335X_GPIO0, AM335X_GPIO1, AM335X_GPIO2, AM335X_GPIO3 };

static uint32_t inline get_pin_mask(unsigned int pin_number)
{
  return (1UL << gpio_bank_pin[pin_number]);
}

static void inline reg_update_set(unsigned int pin_number,uint32_t reg)
{
  uint32_t gpioreg=gpio_bank[pin_number]+reg;
  uint32_t gpioreg_val=mmio_read(gpioreg);
  gpioreg_val |= get_pin_mask(pin_number);
  mmio_write(gpioreg, gpioreg_val);
}

static void inline reg_update_unset(unsigned int pin_number,uint32_t reg)
{
  uint32_t gpioreg=gpio_bank[pin_number]+reg;
  uint32_t gpioreg_val=mmio_read(gpioreg);
  gpioreg_val &= ~get_pin_mask(pin_number);
  mmio_write(gpioreg, gpioreg_val);
}

/**
 * @brief Initializes the GPIO API. 
 *        Allocates space to gpio_pin_state and sets pin state as UNCONFIGURED.
 *        Creates Semaphore for avoiding any race condition.
 *        If the API has already been initialized silently exits.
 */
void rtems_gpio_initialize(void)
{ 
  int i;
  rtems_status_code status;
  if ( is_initialized )
    return;
 
  is_initialized = true;
  for ( i = 0; i < GPIO_PIN_COUNT; ++i ) {
    gpio_pin_state[i] = GPIO_PIN_STATE_UNCONFIGURED;
  }
  /* Create GPIO bank Semaphores */
  status = rtems_semaphore_create(
    rtems_build_name('G', 'L', 'C', 'K'), 
    1, 
    MUTEX_ATRIBUTES, 
    0, 
    &bank_lock
  );
  if (status != RTEMS_SUCCESSFUL){
    printf("Semaphore not created\n");
  }
}

/**
 * @brief Configures a GPIO pin to perform a digital output.
 *
 * @retval GPIO_SUCCESSFUL Pin was configured successfully as output.
 * @retval GPIO_UNKNOWN_PIN Pin is invalid or unknown.
 * @retval GPIO_MISCONFIGURED_PIN Pin is already configured for another state.
 */
int rtems_gpio_configure_pin_digital_out(
  gpio_pin_handle *gpio_pin_assign,unsigned int pin_number){
  
  OBTAIN_LOCK(bank_lock);
  if (pin_number >= GPIO_PIN_COUNT || pin_number < 0){
    RELEASE_LOCK(bank_lock);
    return GPIO_UNKNOWN_PIN;
  }
  if (
  gpio_pin_state[gpio_pin_assign->pin_number] != GPIO_PIN_STATE_UNCONFIGURED &&
  gpio_pin_state[gpio_pin_assign->pin_number] != GPIO_PIN_STATE_DIGITAL_OUT){
    RELEASE_LOCK(bank_lock);
    return GPIO_MISCONFIGURED_PIN;
  }
 
  gpio_pin_state[gpio_pin_assign->pin_number] = GPIO_PIN_STATE_DIGITAL_OUT;
  gpio_pin_assign->pin_number = pin_number;
  gpio_bank_pin[pin_number] = pin_number % GPIO_PINS_PER_BANK;
  gpio_bank[pin_number] = gpio_bank_addrs[pin_number/GPIO_PINS_PER_BANK];

  reg_update_unset(gpio_pin_assign->pin_number,AM335X_GPIO_OE);

  RELEASE_LOCK(bank_lock);
  return GPIO_SUCCESSFUL;
}

/**
 * @brief Gives an output GPIO pin the logical value of 1.
 * @retval GPIO_SUCCESSFUL Pin was set successfully.
 * @retval GPIO_MISCONFIGURED_PIN The received pin is not configured 
 *         for digital output.
 */
int rtems_gpio_digital_set(gpio_pin_handle *gpio_pin_assign){

  OBTAIN_LOCK(bank_lock);
  if (
  gpio_pin_state[gpio_pin_assign->pin_number] != GPIO_PIN_STATE_DIGITAL_OUT){
    RELEASE_LOCK(bank_lock);
    return GPIO_MISCONFIGURED_PIN;
  }

  reg_update_set(gpio_pin_assign->pin_number,AM335X_GPIO_DATAOUT);
 
  RELEASE_LOCK(bank_lock);
  return GPIO_SUCCESSFUL;
}

/**
 * @brief Gives an output GPIO pin the logical value of 0.
 * @retval GPIO_SUCCESSFUL Pin was cleared successfully.
 * @retval GPIO_MISCONFIGURED_PIN The received pin is not configured 
 *         for digital output.
 */
int rtems_gpio_digital_clear(gpio_pin_handle *gpio_pin_assign){

  OBTAIN_LOCK(bank_lock);
  if (
  gpio_pin_state[gpio_pin_assign->pin_number] == GPIO_PIN_STATE_DIGITAL_OUT){
    RELEASE_LOCK(bank_lock);
    return GPIO_MISCONFIGURED_PIN ;
  }

  reg_update_unset(gpio_pin_assign->pin_number,AM335X_GPIO_DATAOUT);
 
  RELEASE_LOCK(bank_lock);
  return GPIO_SUCCESSFUL;
}
/**
 * @brief Releases currently configured pin and makes unused for repurposing.
 * @retval GPIO_SUCCESSFUL Pin was released successfully or it is already 
 *         UNCONFIGURED state.
 *
 */
int rtems_gpio_release_pin(gpio_pin_handle *gpio_pin_assign){
  
  OBTAIN_LOCK(bank_lock);
  if (
  gpio_pin_state[gpio_pin_assign->pin_number] == GPIO_PIN_STATE_UNCONFIGURED){
    RELEASE_LOCK(bank_lock);
    return GPIO_SUCCESSFUL;
  }
 
  OBTAIN_LOCK(bank_lock);
 
  gpio_pin_state[gpio_pin_assign->pin_number] = GPIO_PIN_STATE_UNCONFIGURED;
  reg_update_set(gpio_pin_assign->pin_number,AM335X_GPIO_OE);

  RELEASE_LOCK(bank_lock);
  return GPIO_SUCCESSFUL;
}