/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBoard I2C support implementation.
 */

/*
 * Copyright (c) 2015 Ketul Shah <ketulshah1993 at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <bsp/i2c.h>
#include "i2c_init.c"

static const uint32_t i2c_clkcntrl_reg[]= {
	AM335X_CM_WKUP_I2C0_CLKCTRL, AM335X_CM_PER_I2C1_CLKCTRL,
	AM335X_CM_PER_I2C2_CLKCTRL
}; 

static inline uint32_t get_reg_addr(uint32_t offset)
{
  return ((beagle_i2c_bus_desc->i2c_base_addrs) + offset);
}

static inline uint16_t beagle_i2c_read_status(void){
  
  return (read16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_IRQSTATUS_RAW)));
}

static inline void beagle_i2c_write_status(uint16_t mask){

  /* Writing a 1 to IRQSTATUS will clear it to 0, that is, clear the IRQ */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_IRQSTATUS), mask);
}

static uint16_t beagle_i2c_poll(uint16_t mask){

  uint16_t status;
  unsigned int msec = 0;

  /* poll for up to 1 s */
  
  do {
    status = beagle_i2c_read_status();
    if ((status & mask) != 0) {
      return status;
    }
    usleep(1000);
    msec++;
 
    } while (msec < 1000);
 
  /* timeout reached, abort */
  return status;      
}

static void intr_handler(void *arg){

  uint16_t intmask = 0;

  /* Interrupt masking using following active events
   * Receive overrun
   * Access Error
   * Transmit data read
   * Receive data ready
   * Register access ready
   * No acknowledgment
   * Arbitration lost
   */

  intmask |= (
  	I2C_ROVR | I2C_AERR | I2C_XRDY | I2C_RRDY | I2C_ARDY | I2C_NACK | I2C_AL);

  /* Update Interrupt Enable Set Register */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_IRQENABLE_SET), intmask);
  
  if ( I2C_IO_MODE == 1 ) {
    /* Release the irq semaphore. */
    beagle_i2c_softc_t *softc_ptr = (beagle_i2c_softc_t *) arg;
    rtems_semaphore_release(softc_ptr->irq_sema_id);
  }
}

static bool beagle_i2c_bus_is_free(void){

  uint16_t status;
  unsigned int msec = 0;
  /* Wait to beacome bus free up to 1s */
  do {
    status = beagle_i2c_read_status();
    if ((status & I2C_BB) == 0) {
      /* Bus is free now */
      return true;
    }
    usleep(1000);
    msec++;
 
    } while (msec < 1000);
  /* Timeout Expired */
  return false;  
}

static inline void beagle_i2c_clkconfg(int i2c_bus_number){

  set32(i2c_clkcntrl_reg[i2c_bus_number], BIT(1), 0xFFFFFFFF);
}

static void beagle_i2c_flush(void){
  
  int checkpoints;
  uint16_t status;

  for (checkpoints = 0; checkpoints < 1000; checkpoints++){
  	status = beagle_i2c_poll(I2C_RRDY);
  	if ((status & I2C_RRDY) != 0){
  	  /* Bytes available for reading
  	   * Consume it and throw it away
  	   */
  	  (void) read16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_DATA));

  	  /* Now clear the Read Ready flag */
  	  beagle_i2c_write_status(I2C_RRDY);

  	}
  	else{
  	  /* Buffer Drained already */
  	  break;
  	}

  }
}

/**
 * @brief Reads/writes to/from the I2C bus.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] rd_buf Read buffer. If not NULL the function will read from 
 *                   the bus and store the read on this buffer.
 * @param[in] wr_buf Write buffer. If not NULL the function will write the 
 *                   contents of this buffer to the bus.
 * @param[in] buffer_size Size of the non-NULL buffer.
 *
 * @retval -1 Could not send/receive data to/from the bus.
 * @retval >=0 The number of bytes read/written.
 */
static int beagle_i2c_read_write(
rtems_libi2c_bus_t * bushdl, 
unsigned char *rd_buf, 
const unsigned char *wr_buf, 
int buffer_size
) {

  beagle_i2c_softc_t *softc_ptr = &(((beagle_i2c_desc_t *)(bushdl))->softc);

  uint16_t con_opts = 0, poll_mask = 0 , err_mask = 0;
  int i,status;

  err_mask |= (I2C_ROVR | I2C_AERR | I2C_NACK | I2C_AL);

  /* If writing. */
  if ( rd_buf == NULL ) {
  	poll_mask |= (I2C_XRDY);
  	con_opts |= I2C_CON_TRX;
  	beagle_i2c_write_status(0x7FFF);
  }
  else{
  	poll_mask |= (I2C_RRDY);
  }
   
  /* Set Bytes to Read/Write */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_CNT), buffer_size);
  
  con_opts |= (I2C_CON_EN | I2C_CON_TRX | I2C_CON_STT);
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_CON), con_opts);

  for (i = 0; i < buffer_size; i++){
  	status = beagle_i2c_poll(poll_mask | err_mask);

  	if ((status & err_mask) != 0){
  		printf("I/O Error\n");
  		return -1;
  	} else if ((status & poll_mask) == 0){
  		printf("Not Ready for I/O operation\n");
  		return -1;
  	}
  	if ( rd_buf == NULL ) {

  	  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_DATA),
  	  *(uint8_t *)wr_buf);
  	} 
  	else {

  	  (*(uint8_t *)rd_buf) = read16(
  	  	get_reg_addr(beagle_i2c_bus_desc->regs->I2C_DATA)) & 0xff;	

  	}

  	/* clear the read/write ready flag */
	beagle_i2c_write_status(poll_mask);

  	}

  status = beagle_i2c_read_status();
  if ((status & I2C_NACK) != 0){

  	printf("No acknowledgment\n");
  	return -1;
  }

  /* Wait for operation to complete(polling access ready bit) */
  poll_mask = I2C_ARDY;
  status = beagle_i2c_poll(poll_mask);
  if ((status & poll_mask) == 0){
  	printf("I/O operation never finished\n");
  	return -1;
  }

  beagle_i2c_write_status(0x7fff);

  return 0;
}

/**
 * @brief Low level function to initialize the I2C bus. 
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 *
 * @retval RTEMS_SUCCESSFUL I2C bus successfully initialized.
 * @retval Any other status code @see rtems_semaphore_create() and 
 *         @see rtems_interrupt_handler_install().
 */
rtems_status_code beagle_i2c_init(rtems_libi2c_bus_t * bushdl)
{
  beagle_i2c_softc_t *softc_ptr = &(((beagle_i2c_desc_t *)(bushdl))->softc);
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  if ( softc_ptr->is_initialized) {
    return sc;
  }

  softc_ptr->is_initialized = true;

  /* Ensure i2c module is disabled before setting prescalar & bus speed */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_CON), 0);
  udelay(50000);

  /* Disable autoidle mechanism (Value after reset is high) */
  set16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_SYSC), 
  	I2C_SYSC_AUTOIDLE, 0);

  /* Set prescalar to obtain 12 MHz i2c module clock */
  /* The core logic is sampled at the clock rate of the system clock for
		the module divided by (PSC + 1) */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_PSC),
  	((I2C_IP_CLK / BEAGLE_MODULE_CLOCK) - 1));

  /* Set the SCLL and SCLH for bus speed(100 KHz) configuration */

  /* tLOW = (SCLL + 7) * ICLK time period */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_SCLL),
  	((BEAGLE_MODULE_CLOCK / (2 * CONFIG_SYS_I2C_SPEED )) - 7));
  /* tHIGH = (SCLH + 5) * ICLK time period */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_SCLH),
  	((BEAGLE_MODULE_CLOCK / (2 * CONFIG_SYS_I2C_SPEED )) - 5));

  /* Set own I2C address */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_OA), I2C_OWN_ADDRESS);

  /* Configuring Buffer Configuration register 
   * Receive and Transmit DMA channel disabled
   * Receive and Transmit Threshold value = 1
   */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_BUF), 0x0000);

  /* Now bring I2C module out of reset */
  set16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_CON),
    I2C_CON_EN, I2C_CON_EN);
  udelay(50000);
  
  /* If the access to the bus is configured to be interrupt-driven. */
  if ( I2C_IO_MODE == 1 ) {
  	sc = rtems_semaphore_create(rtems_build_name('i','2','c','s'), 
                              0, 
                              RTEMS_FIFO | RTEMS_SIMPLE_BINARY_SEMAPHORE, 
                              0, 
                              &softc_ptr->irq_sema_id
                             );

  	sc = rtems_interrupt_handler_install(beagle_i2c_bus_desc->i2c_irq, 
                                       NULL, 
                                       RTEMS_INTERRUPT_UNIQUE, 
                                       (rtems_interrupt_handler) intr_handler,
                                       softc_ptr
                                      );

  }
  else {
  /* According to u-boot, these are needed even if just using by
   * polling method (i.e. non-interrupt driver programming).
   */
  intr_handler(bushdl);
  }
  return sc;

}

/**
 * @brief Low level function that would send a start condition over the I2C bus.
 *        Because of the way the BSC controller implements the I2C protocol, the
 *        start sequence is sent whenever beagle_i2c_read() function is called.
 *        Instead this function clears the garbage that may be used in FIFOS 
 *        before each new data transfer.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 *
 * @retval RTEMS_SUCCESSFUL
 */
rtems_status_code beagle_i2c_send_start(rtems_libi2c_bus_t * bushdl)
{
  /* Clear FIFOs. */
  beagle_i2c_flush();

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Low level function that would send a stop condition over the I2C bus,
 *        however the BSC controller send this condition automatically when the
 *        DLEN (data length - the number of bytes to be transferred) register
 *        value reaches 0.
 *        For that reason, it is here just to satisfy, the libi2c API,
 *        which requires this function.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 *
 * @retval RTEMS_SUCCESSFUL
 */
rtems_status_code beagle_i2c_stop(rtems_libi2c_bus_t * bushdl)
{
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Low level function which addresses a I2C device.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] addr Address of a connected I2C device
 * @param[in] rw Defines the nature of the transfer which will take place with 
 *               the addressed device - 0 to write and 1 to read.
 *
 * @retval RTEMS_SUCCESSFUL The device has been successfully addressed.
 */
rtems_status_code 
beagle_i2c_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw)
{
  /* Address slave device. */
  write16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_SA), addr);

  /* Set read/write bit. 
   * If writing. (Transmitter Mode) */
  if ( rw == 0 ) {
    set16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_CON),
      I2C_CON_TRX, I2C_CON_TRX);
  }
  /* If reading. (Receiver Mode) */
  else {
    set16(get_reg_addr(beagle_i2c_bus_desc->regs->I2C_CON), I2C_CON_TRX, 0);
  }

  return RTEMS_SUCCESSFUL;
}


/**
 * @brief Low level function that reads a number of bytes from the I2C bus 
 *        on to a buffer.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] bytes Buffer where the data read from the bus will be stored.
 * @param[in] nbytes Number of bytes to be read from the bus 
 *                   to the bytes buffer.
 *
 * @retval @see beagle_i2c_read_write().
 */
int beagle_i2c_read_bytes(
rtems_libi2c_bus_t * bushdl, 
unsigned char *bytes, 
int nbytes
)
{
  return beagle_i2c_read_write(bushdl, bytes, NULL, nbytes);
}

/**
 * @brief Low level function that writes a number of bytes from a buffer
 *        to the I2C bus.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] bytes Buffer with data to send through the bus.
 * @param[in] nbytes Number of bytes to be written from the bytes buffer 
                     to the bus.
 *
 * @retval @see beagle_i2c_read_write().
 */
int beagle_i2c_write_bytes(
rtems_libi2c_bus_t * bushdl, 
unsigned char *bytes, 
int nbytes
)
{
  return beagle_i2c_read_write(bushdl, NULL, bytes, nbytes);
}

/**
 * @brief Low level function that is used to perform ioctl 
 *        operations on the bus. Currently only setups
 *        the bus transfer mode, namely the bus clock divider.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] cmd IOCTL request command.
 * @param[in] arg Arguments needed to fulfill the requested IOCTL command.
 *
 * @retval 1 . This function would be in to do. 
 */
int beagle_i2c_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg)
{
	return 0;
}