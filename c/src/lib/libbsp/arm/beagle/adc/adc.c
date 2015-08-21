/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief ADC definitions.
 */
 
/**
 * Copyright (c) 2015 Ketul Shah <ketulshah1993 at gmail.com>

 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */
 
#include <libcpu/am335x.h>
#include <rtems.h>
#include <bsp/adc.h>
#include <bsp/irq.h>
#include <bsp/beagleboneblack.h>
#include <bsp.h>
#include <stdlib.h>
#include <stdint.h>

static void inline reg_update_or(uint32_t reg, uint32_t val)
{
  uint32_t reg_val=mmio_read(reg);
  reg_val |= val;
  mmio_write(reg, reg_val);
}

/* Configure STEPCONFIG register with average of 16 */
static void inline stepc_update_16(uint32_t reg, uint32_t val)
{
  mmio_write(reg,(val | AM335X_ADC_AVG16));
}

/* Configure STEPDELAY register */
static void inline stepd_update(uint32_t reg)
{
  mmio_write(reg,(0x0F)<<24);
}

static uint32_t inline get_val(uint32_t reg, uint32_t val)
{
  return (mmio_read(reg) & val);
}

void rtems_adc_init(void)
{
  /* Enable the CM_WKUP_ADC_CLKCTRL with CM_WKUP_MODUELEMODE_ENABLE */
  reg_update_or(AM335X_CM_WKUP_ADC_CLKCTRL, AM335X_CM_WKUP_MODULEMODE_ENABLE);
 
  while(!(
  get_val(AM335X_CM_WKUP_ADC_CLKCTRL,AM335X_CM_WKUP_MODULEMODE_ENABLE))){
    /* Here waiting period for intialization of adc clock module */
  }

  /* Make sure STEPCONFIG write protect is off */
  reg_update_or(AM335X_ADC_CTRL, AM335X_ADC_STEPCONFIG_WRITE_PROTECT_OFF);
 
  /* ADC_STEPCONFIG for each AIN pin */
  stepc_update_16(AM335X_ADC_STEPCONFIG1,0x00<<19);
  stepd_update(AM335X_ADC_STEPDELAY1);
  stepc_update_16(AM335X_ADC_STEPCONFIG2,0x01<<19);
  stepd_update(AM335X_ADC_STEPDELAY1);
  stepc_update_16(AM335X_ADC_STEPCONFIG3,0x02<<19);
  stepd_update(AM335X_ADC_STEPDELAY1);
  stepc_update_16(AM335X_ADC_STEPCONFIG4,0x03<<19);
  stepd_update(AM335X_ADC_STEPDELAY1);
  stepc_update_16(AM335X_ADC_STEPCONFIG5,0x04<<19);
  stepd_update(AM335X_ADC_STEPDELAY1);
  stepc_update_16(AM335X_ADC_STEPCONFIG6,0x05<<19);
  stepd_update(AM335X_ADC_STEPDELAY1);
  stepc_update_16(AM335X_ADC_STEPCONFIG7,0x06<<19);
  stepd_update(AM335X_ADC_STEPDELAY1);
  stepc_update_16(AM335X_ADC_STEPCONFIG8,0x07<<19);
  stepd_update(AM335X_ADC_STEPDELAY1);

  /* ADC_CTRL is enabled */
  reg_update_or(AM335X_ADC_CTRL,0x01);
}

int rtems_adc_read(unsigned int pin_number) 
{
	
  /* The clock module is not enabled */
  if(get_val(AM335X_CM_WKUP_ADC_CLKCTRL,AM335X_CM_WKUP_IDLEST_DISABLED))
	rtems_adc_init();
	
  /* Enable the step sequencer for the given pin */
  reg_update_or(AM335X_ADC_STEPENABLE,(0x01<<(pin_number+1)));

  /* Return the value of data register FIFO0 */
  return (get_val(AM335X_ADC_FIFO0DATA,AM335X_ADC_FIFO_MASK));
}
