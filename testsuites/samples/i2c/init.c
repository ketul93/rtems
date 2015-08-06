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

#include <bsp.h> /* for device driver prototypes */

#include <bsp/i2c.h>
#include <rtems/status-checks.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <libchip/adxl345.h>

/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument argument);

const char rtems_test_name[] = "I2C_ADXL345_TEST";

rtems_task Init(
  rtems_task_argument ignored
)
{
  int rv = 0;
  int fd;

  rtems_test_begin ();
  
  /* Open the adxl345 device file */
  fd = open("/dev/i2c.adxl345", O_RDWR);
  RTEMS_CHECK_RV(rv, "Open /dev/i2c.adxl345");
  
  /* Set the adxl345 BW Rate */
  rv = ioctl(fd, ADXL345_CONF_BWRATE);
  RTEMS_CHECK_RV(rv, "adxl345 BW rate is set");

  /* Powering up the module for measurement */
  rv = ioctl(fd, ADXL345_CONF_MEASUREMENT);
  RTEMS_CHECK_RV(rv, "adxl345 is ready for measurement");

  /* Set the adxl345 Range for measurement */
  rv = ioctl(fd, ADXL345_CONF_RANGE);
  RTEMS_CHECK_RV(rv, "adxl345 Range configuration");

  /* Now getting data from accelerometer from the all axis */
  while(1)
  {
  rv = ioctl(fd, ADXL345_READ_XAXIS);
  rv = ioctl(fd, ADXL345_READ_YAXIS);
  rv = ioctl(fd, ADXL345_READ_ZAXIS);
  RTEMS_CHECK_RV(rv, "All axis data fetched successfilly");

  rv = close(fd);
  RTEMS_CHECK_RV(rv, "Close /dev/i2c.adxl345");

  rtems_test_end ();
  exit ( 0 );
  }
}

#define CONFIGURE_MAXIMUM_DRIVERS 10
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_USE_IMFS_AS_BASE_FILESYSTEM

#define CONFIGURE_MAXIMUM_SEMAPHORES 3

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 30

#define CONFIGURE_MAXIMUM_TASKS 20

 #define CONFIGURE_INIT_TASK_STACK_SIZE (32 * 1024)

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_INIT
#include <rtems/confdefs.h>