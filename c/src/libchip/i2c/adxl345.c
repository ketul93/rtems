#define ADXL345_EARTH_GRAVITY_MS2 9.80665
#define ADXL345_SCALE_MULTIPLIER  0.004

#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D

#define ADXL345_BW_RATE_1600HZ 0x0F
#define ADXL345_BW_RATE_800HZ  0x0E
#define ADXL345_BW_RATE_400HZ  0x0D
#define ADXL345_BW_RATE_200HZ  0x0C
#define ADXL345_BW_RATE_100HZ  0x0B
#define ADXL345_BW_RATE_50HZ   0x0A
#define ADXL345_BW_RATE_25HZ   0x09

#define ADXL345_RANGE_2G  0x00
#define ADXL345_RANGE_4G  0x01
#define ADXL345_RANGE_8G  0x02
#define ADXL345_RANGE_16G 0x03

#define ADXL345_MEASURE   0x08
#define ADXL345_RA_DATAX0 0x32
#define ADXL345_RA_DATAX1 0x33
#define ADXL345_RA_DATAY0 0x34
#define ADXL345_RA_DATAY1 0x35
#define ADXL345_RA_DATAZ0 0x36
#define ADXL345_RA_DATAZ1 0x37
#define ADXL345_RA_FIFO_CTL 0x38
#define ADXL345_RA_FIFO_STATUS 0x39

#include <libchip/adxl345.h>

static rtems_libi2c_tfr_mode_t tfr_mode = 
{
  /* Set a baudrate of 100kHz */
  .baudrate = 100000
};

/* Wait ms miliseconds */
static rtems_status_code i2c_adxl345_wait_ms(int ms)
{
  rtems_interval ticks_per_second;

  ticks_per_second = rtems_clock_get_ticks_per_second();

  return rtems_task_wake_after((ticks_per_second * ms / 1000));
}

static rtems_status_code i2c_adxl345_write_register(rtems_device_minor_number minor, int reg, int val)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  unsigned char cmd_buf[2];
  int rv;

  /* Start the bus */
  sc = rtems_libi2c_send_start(minor);
   
  if ( sc != RTEMS_SUCCESSFUL )
    return sc;

  /* Set transfer mode */
  sc = rtems_libi2c_ioctl(minor, RTEMS_LIBI2C_IOCTL_SET_TFRMODE, &tfr_mode);

  if ( sc != RTEMS_SUCCESSFUL )
    return sc;
    
  /* Address device */
  sc = rtems_libi2c_send_addr(minor, FALSE);

  if ( sc != RTEMS_SUCCESSFUL )
    return sc;

  /* Wait 1 milisecond */
  sc = i2c_adxl345_wait_ms(1);

  if ( sc != RTEMS_SUCCESSFUL )
    return sc;

  cmd_buf[0] = reg;

  cmd_buf[1] = val;
    
  rv = rtems_libi2c_write_bytes(minor, cmd_buf, 2);

  if ( rv < 0 ) 
    return RTEMS_IO_ERROR;
      
  /* Terminate transfer */
  sc = rtems_libi2c_send_stop(minor);

  if ( sc != RTEMS_SUCCESSFUL )
    return sc;

  /* Wait 1 milisecond */
  sc = i2c_adxl345_wait_ms(1);

  if ( sc != RTEMS_SUCCESSFUL )
    return sc;

  return sc;
}

static rtems_status_code i2c_adxl345_read_register(rtems_device_minor_number minor, int reg, uint8_t* reg_content)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  unsigned char cmd_buf[1];
  int rv;
   
  /* Start the bus */
  sc = rtems_libi2c_send_start(minor);

  if ( sc != RTEMS_SUCCESSFUL )
    return sc;

  /* Set transfer mode */
   sc = rtems_libi2c_ioctl(minor, RTEMS_LIBI2C_IOCTL_SET_TFRMODE, &tfr_mode);

  if ( sc != RTEMS_SUCCESSFUL )
      return sc;
    
  /* Address device */
  sc = rtems_libi2c_send_addr(minor, FALSE);

  if ( sc != RTEMS_SUCCESSFUL )
      return sc;
  
  /* Wait 1 milisecond */
  sc = i2c_adxl345_wait_ms(1);

  if ( sc != RTEMS_SUCCESSFUL )
   return sc;

  cmd_buf[0] = reg;

  rv = rtems_libi2c_write_bytes(minor, cmd_buf, 1);

  if ( rv < 0 ) 
     return RTEMS_IO_ERROR;

  /* Address device */
  sc = rtems_libi2c_send_addr(minor, TRUE);

  if ( sc != RTEMS_SUCCESSFUL )
      return sc;
  
  /* Wait 1 milisecond */
  sc = i2c_adxl345_wait_ms(1);

  if ( sc != RTEMS_SUCCESSFUL )
   return sc;

  /* Fetch data */
  rv = rtems_libi2c_read_bytes(minor, reg_content, 1);
   
  if ( rv < 0 ) 
      return RTEMS_IO_ERROR;
  
  /* Terminate transfer */
  sc = rtems_libi2c_send_stop(minor);
  
  if ( sc != RTEMS_SUCCESSFUL )
   return sc;

  /* Wait 1 milisecond */
  sc = i2c_adxl345_wait_ms(1);

  if ( sc != RTEMS_SUCCESSFUL )
   return sc;

  return sc;
}

static rtems_status_code i2c_adxl345_set_register(rtems_device_minor_number minor, int reg,uint8_t val)
{
  rtems_status_code sc;
  uint8_t reg_content;

  sc = i2c_adxl345_read_register(minor, reg, &reg_content);

  if ( sc != RTEMS_SUCCESSFUL )
    return RTEMS_IO_ERROR;
	
  reg_content = val;

 sc = i2c_adxl345_write_register(minor, reg, reg_content);

  return sc;
}

static rtems_status_code i2c_adxl345_set_range(rtems_device_minor_number minor, uint8_t range_flag)
{
  rtems_status_code sc;
  uint8_t reg_content;

  sc = i2c_adxl345_read_register(minor, ADXL345_DATA_FORMAT, &reg_content);

  if ( sc != RTEMS_SUCCESSFUL )
    return RTEMS_IO_ERROR;
	
  reg_content &= ~0x0F;
  reg_content |= (range_flag | 0x08);

  sc = i2c_adxl345_write_register(minor, ADXL345_DATA_FORMAT, reg_content);

  return sc;
}

static rtems_status_code i2c_adxl345_getAxes(rtems_device_minor_number minor, const char axis)
{
  rtems_status_code sc;
  int byte0,byte1;
  float axis_val;

  switch (axis)
  {
  	case 'X':

  	  sc = i2c_adxl345_read_register(minor, ADXL345_RA_DATAX0, &byte0);
  	  if ( sc != RTEMS_SUCCESSFUL )
	    return RTEMS_IO_ERROR;
	  sc = i2c_adxl345_read_register(minor, ADXL345_RA_DATAX1, &byte1);
	  break;

	case 'Y':

  	  sc = i2c_adxl345_read_register(minor, ADXL345_RA_DATAY0, &byte0);
  	  if ( sc != RTEMS_SUCCESSFUL )
	    return RTEMS_IO_ERROR;
	  sc = i2c_adxl345_read_register(minor, ADXL345_RA_DATAY1, &byte1);
	  break;

	case 'Z':

  	  sc = i2c_adxl345_read_register(minor, ADXL345_RA_DATAZ0, &byte0);
  	  if ( sc != RTEMS_SUCCESSFUL )
	    return RTEMS_IO_ERROR;
	  sc = i2c_adxl345_read_register(minor, ADXL345_RA_DATAZ1, &byte1);
	  break;

	default :

	  sc = RTEMS_INVALID_NUMBER;
  }

  axis_val = byte0 | (byte1 << 8)
  if (axis_val & (1 << 16 - 1))
  {
  	axis_val = axis_val - (1 << 16);
  }

  axis_val = axis_val * ADXL345_SCALE_MULTIPLIER ;

  printf("  %c = %.3f\n",axis,axis_val);

  return sc;
} 

rtems_status_code i2c_adxl345_ioctl(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{  
  rtems_libio_ioctl_args_t *args = arg;
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  int pin, cmd;

  int rv = 0;
   
  uint8_t reg_content;

  cmd = (int)(args->command);

  switch ( cmd ) 
  {
    case ADXL345_CONF_BWRATE:

      sc = i2c_adxl345_set_register(minor, ADXL345_BW_RATE , ADXL345_BW_RATE_100HZ );
      break;

    case ADXL345_CONF_MEASUREMENT:

      sc = i2c_adxl345_set_register(minor, ADXL345_POWER_CTL, ADXL345_MEASURE);
      break;

    case ADXL345_CONF_RANGE:

      sc = i2c_adxl345_set_range(minor, ADXL345_RANGE_2G);
      break;

    case ADXL345_READ_XAXIS:

      sc = i2c_adxl345_getAxes(minor, X);
      break;
      
    case ADXL345_READ_YAXIS:

      sc = i2c_adxl345_getAxes(minor, Y);
      break;

    case ADXL345_READ_ZAXIS:

      sc = i2c_adxl345_getAxes(minor, Z);
      break;
    
    default:

    sc = RTEMS_INVALID_NUMBER;
      
  }

  args->ioctl_return = rv;

  return sc;
}

rtems_status_code i2c_adxl345_init(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
  return RTEMS_SUCCESSFUL;
}

rtems_status_code i2c_adxl345_open(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
  return RTEMS_SUCCESSFUL;
}

rtems_status_code i2c_adxl345_close(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
  return RTEMS_SUCCESSFUL;
}

rtems_status_code i2c_adxl345_read_entry(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
  return RTEMS_SUCCESSFUL;
}

rtems_status_code i2c_adxl345_write_entry(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
  return RTEMS_SUCCESSFUL;
}

rtems_driver_address_table i2c_adxl345_ops = 
{
  .initialization_entry = i2c_adxl345_init,
  .open_entry           = i2c_adxl345_open,
  .close_entry          = i2c_adxl345_close,
  .read_entry           = i2c_adxl345_read_entry,
  .write_entry          = i2c_adxl345_write_entry,
  .control_entry        = i2c_adxl345_ioctl
};

rtems_libi2c_drv_t i2c_adxl345_drv_t =
{
    .ops  = &i2c_adxl345_ops, 
    .size = sizeof (i2c_adxl345_drv_t),
};