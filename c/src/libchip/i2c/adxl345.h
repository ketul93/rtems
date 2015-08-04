include <rtems.h>
#include <rtems/libi2c.h>

#include <rtems/libio.h>

#ifndef LIBI2C_ADXL345_H
#define LIBI2C_ADXL345_H

#ifdef __cplusplus
extern "C" {
#endif

#define ADXL345_ADDR 0x53

typedef enum
{
  ADXL345_CONF_MEASUREMENT,
  ADXL345_CONF_BWRATE,
  ADXL345_CONF_RANGE,
  ADXL345_READ_XAXIS,
  ADXL345_READ_YAXIS,
  ADXL345_READ_ZAXIS,
} adxl345_cmd;


rtems_status_code i2c_adxl345_init(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
rtems_status_code i2c_adxl345_open(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
rtems_status_code i2c_adxl345_close(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
rtems_status_code i2c_adxl345_read_entry(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
rtems_status_code i2c_adxl345_write_entry(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
rtems_status_code i2c_adxl345_ioctl(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);

extern rtems_driver_address_table i2c_adxl345_ops;

extern rtems_libi2c_drv_t i2c_adxl345_drv_t;

#ifdef __cplusplus
}
#endif

#endif /* LIBI2C_ADXL345_H */
