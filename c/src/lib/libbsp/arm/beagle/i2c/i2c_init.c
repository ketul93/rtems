/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBoard I2C bus initialization.
 */

/*
 * Copyright (c) 2015 Ketul Shah <ketulshah1993 at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <bsp/i2c.h>

static rtems_libi2c_bus_ops_t beagle_i2c_ops = {
  init:             beagle_i2c_init,
  send_start:       beagle_i2c_send_start,
  send_stop:        beagle_i2c_stop,
  send_addr:        beagle_i2c_send_addr,
  read_bytes:       beagle_i2c_read_bytes,
  write_bytes:      beagle_i2c_write_bytes,
  ioctl:            beagle_i2c_ioctl
};

static const i2c_base_addrs[] = {
	AM335X_I2C0_BASE, AM335X_I2C1_BASE, AM335X_I2C0_BASE
};

static const i2c_irq_num[] = {
	BEAGLE_I2C0_IRQ , BEAGLE_I2C1_IRQ , BEAGLE_I2C2_IRQ
};

static beagle_i2c_desc_t beagle_i2c_bus_desc = {
  {
    ops:			&beagle_i2c_ops,
    size:			sizeof(beagle_i2c_bus_desc)
  },
  {
	is_initialized:	false
  },
  &am335x_i2c_regs,
  i2c_base_addrs[0], /* Default I2C[0] bus selected */
  i2c_irq_num[0], /* Default I2C[0] irq */
  0 /* Default bus id */
};

/* Drivers registration for all the devices 
 * which want access to the I2C bus.
 *
 * This function returns 0 on success. */

int BSP_i2c_register_drivers(int i2c_bus_number)
{
  int rv = 0;

  if (i2c_bus_number < I2C_BUS_MAX && i2c_bus_number >= 0 ) {
  	beagle_i2c_bus_desc->i2c_base_addrs = i2c_base_addrs[i2c_bus_number];
  	beagle_i2c_bus_desc->i2c_bus_id = i2c_bus_number;
  	beagle_i2c_bus_desc->i2c_irq = i2c_irq_num[i2c_bus_number];
  }

  else {
  	rv = -1;
  	printf("Invalid bus number. Must be from 0 to 2\n");
  }

  return rv;
}

int BSP_i2c_init(void)
{
  int rv;

  /* Initialize the libi2c API. */
  rtems_libi2c_initialize ();

  /* Register the I2C bus. */
  rv = rtems_libi2c_register_bus("/dev/i2c", &(beagle_i2c_bus_desc.bus_desc));

  if ( rv < 0 ) {
    return -rv;
  }
  
  return 0;
}