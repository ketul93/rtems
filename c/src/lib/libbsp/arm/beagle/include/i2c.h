/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief I2C support API.
 */

/*
 * Copyright (c) 2012 Claas Ziemke. All rights reserved.
 * Copyright (c) 2015 Ketul Shah. All rights reserved.
 *
 *  Claas Ziemke
 *  Kernerstrasse 11
 *  70182 Stuttgart
 *  Germany
 *  <claas.ziemke@gmx.net>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 *
 */

#ifndef LIBBSP_ARM_BEAGLE_I2C_H
#define LIBBSP_ARM_BEAGLE_I2C_H

#include <rtems.h>

#include <bsp.h>
#include <bsp/irq.h>
#include <rtems/libi2c.h>
#include <libcpu/am335x.h>

#include <stdio.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Same offsets for both dm37xx and am335x SoCs */

/* I2C Configuration Register (I2C_CON): */

#define I2C_CON_EN  (1 << 15)  /* I2C module enable */
#define I2C_CON_BE  (1 << 14)  /* Big endian mode */
#define I2C_CON_STB (1 << 11)  /* Start byte mode (master mode only) */
#define I2C_CON_MST (1 << 10)  /* Master/slave mode */
#define I2C_CON_TRX (1 << 9)   /* Transmitter/receiver mode */
           /* (master mode only) */
#define I2C_CON_XA  (1 << 8)   /* Expand address */
#define I2C_CON_STP (1 << 1)   /* Stop condition (master mode only) */
#define I2C_CON_STT (1 << 0)   /* Start condition (master mode only) */

/* I2C Status Register (I2C_STAT): */

#define I2C_STAT_SBD  (1 << 15) /* Single byte data */
#define I2C_STAT_BB (1 << 12) /* Bus busy */
#define I2C_STAT_ROVR (1 << 11) /* Receive overrun */
#define I2C_STAT_XUDF (1 << 10) /* Transmit underflow */
#define I2C_STAT_AAS  (1 << 9)  /* Address as slave */
#define I2C_STAT_GC (1 << 5) /* General call address detected */
#define I2C_STAT_XRDY (1 << 4)  /* Transmit data ready */
#define I2C_STAT_RRDY (1 << 3)  /* Receive data ready */
#define I2C_STAT_ARDY (1 << 2)  /* Register access ready */
#define I2C_STAT_NACK (1 << 1)  /* No acknowledgment interrupt enable */
#define I2C_STAT_AL (1 << 0)  /* Arbitration lost interrupt enable */

/* I2C System Configuration Register (I2C_SYSC): */

#define I2C_SYSC_CLKACTIVITY_S (1 << 9) /* Only System clock active */
#define I2C_SYSC_CLKACTIVITY_I (1 << 8) /* Only Interface/OCP clock active */
#define I2C_SYSC_SMART_IDLE_MODE (1 << 4) /* Smart-idle Mode */
#define I2C_SYSC_NO_IDLE_MODE (1 << 3) /* No Idle mode */
#define I2C_SYSC_SRST (1 << 1) /*  Reset Entire Module (Hardware reset) */
#define I2C_SYSC_AUTOIDLE (1 << 0) /* Module activates its own Idle mode */

/* I2C Buffer Configuration Register */

#define I2C_BUF_RDMA_EN (1 << 15) /* Receive DMA channel enable */
#define I2C_BUF_RXFIFO_CLR (1 << 14) /* Receive FIFO clear */
#define I2C_BUF_XDMA_EN (1 << 7) /* Transmit DMA channel enable */
#define I2C_BUF_TXFIFO_CLR (1 << 6) /* Transmit FIFO clear */

/* I2C Interrupt Enable Register (I2C_IE): */
#define I2C_IE_GC_IE  (1 << 5)
#define I2C_IE_XRDY_IE  (1 << 4) /* Transmit data ready interrupt enable */
#define I2C_IE_RRDY_IE  (1 << 3) /* Receive data ready interrupt enable */
#define I2C_IE_ARDY_IE  (1 << 2) /* Register access ready interrupt enable */
#define I2C_IE_NACK_IE  (1 << 1) /* No acknowledgment interrupt enable */
#define I2C_IE_AL_IE  (1 << 0) /* Arbitration lost interrupt enable */

/* General(I2C_IRQSTATUS / I2C_STAT / I2C_IRQENABLE_SET / I2C_IE) 
 * offsets for active events
 */

#define I2C_BB   (1 << 12) /* Bus busy */
#define I2C_ROVR (1 << 11) /* Receive overrun */
#define I2C_AERR (1 << 7)  /* Access Error */
#define I2C_XRDY (1 << 4) /* Transmit data ready */
#define I2C_RRDY (1 << 3) /* Receive data ready */
#define I2C_ARDY (1 << 2) /* Register access ready */
#define I2C_NACK (1 << 1) /* No acknowledgment */
#define I2C_AL   (1 << 0) /* Arbitration lost */

/*
 * The equation for the low and high time is
 * tlow = scll + scll_trim = (sampling clock * tlow_duty) / speed
 * thigh = sclh + sclh_trim = (sampling clock * (1 - tlow_duty)) / speed
 *
 * If the duty cycle is 50%
 *
 * tlow = scll + scll_trim = sampling clock / (2 * speed)
 * thigh = sclh + sclh_trim = sampling clock / (2 * speed)
 *
 * In TRM
 * scll_trim = 7
 * sclh_trim = 5
 *
 * The linux 2.6.30 kernel uses
 * scll_trim = 6
 * sclh_trim = 6
 *
 * These are the trim values for standard and fast speed
 */
#ifndef I2C_FASTSPEED_SCLL_TRIM
#define I2C_FASTSPEED_SCLL_TRIM   6
#endif
#ifndef I2C_FASTSPEED_SCLH_TRIM
#define I2C_FASTSPEED_SCLH_TRIM   6
#endif

/* These are the trim values for high speed */
#ifndef I2C_HIGHSPEED_PHASE_ONE_SCLL_TRIM
#define I2C_HIGHSPEED_PHASE_ONE_SCLL_TRIM I2C_FASTSPEED_SCLL_TRIM
#endif
#ifndef I2C_HIGHSPEED_PHASE_ONE_SCLH_TRIM
#define I2C_HIGHSPEED_PHASE_ONE_SCLH_TRIM I2C_FASTSPEED_SCLH_TRIM
#endif
#ifndef I2C_HIGHSPEED_PHASE_TWO_SCLL_TRIM
#define I2C_HIGHSPEED_PHASE_TWO_SCLL_TRIM I2C_FASTSPEED_SCLL_TRIM
#endif
#ifndef I2C_HIGHSPEED_PHASE_TWO_SCLH_TRIM
#define I2C_HIGHSPEED_PHASE_TWO_SCLH_TRIM I2C_FASTSPEED_SCLH_TRIM
#endif

#define OMAP_I2C_STANDARD 100000 /* 100 KHz */
#define OMAP_I2C_FAST_MODE  400000 /* 400 KHz */
#define OMAP_I2C_HIGH_SPEED 3400000 /* 3.4 MHz */


/* Use the reference value of 96MHz if not explicitly set by the board */
#ifndef I2C_IP_CLK
#define I2C_IP_CLK    SYSTEM_CLOCK_96
#endif

/*
 * The reference minimum clock for high speed is 19.2MHz.
 * The linux 2.6.30 kernel uses this value.
 * The reference minimum clock for fast mode is 9.6MHz
 * The reference minimum clock for standard mode is 4MHz
 * In TRM, the value of 12MHz is used.
 */
#ifndef I2C_INTERNAL_SAMPLING_CLK
#define I2C_INTERNAL_SAMPLING_CLK 19200000
#endif

#define I2C_PSC_MAX   0x0f
#define I2C_PSC_MIN   0x00

#define I2C_OWN_ADDRESS 0x01 /* I2C own Address (to be written in I2C_OA) */

#define DISP_LINE_LEN 128
#define I2C_TIMEOUT 1000

#define I2C_BUS_MAX 3

#define I2C_BASE1         (OMAP34XX_CORE_L4_IO_BASE + 0x070000)

#define I2C_DEFAULT_BASE      I2C_BASE1

#define I2C_SYSS_RDONE            (1 << 0)  /* Internel reset monitoring */

#define CONFIG_SYS_I2C_SPEED    100000
#define CONFIG_SYS_I2C_SLAVE    1

typedef struct i2c_regs 
{
  unsigned short I2C_REVNB_LO;	/* AM335X Only */
  unsigned short I2C_REVNB_HI;	/* AM335X Only */
  unsigned short I2C_REV;	/* DM37XX Only */
  unsigned short I2C_IE;	/* DM37XX Only */
  unsigned short I2C_STAT;	/* DM37XX Only */
  unsigned short I2C_SYSC;
  unsigned short I2C_IRQSTATUS_RAW;	/* AM335X Only */
  unsigned short I2C_IRQSTATUS;	/* AM335X Only */
  unsigned short I2C_IRQENABLE_SET;	/* AM335X Only */
  unsigned short I2C_IRQENABLE_CLR;	/* AM335X Only */
  unsigned short I2C_WE;
  unsigned short I2C_DMARXENABLE_SET;	/* AM335X Only */
  unsigned short I2C_DMATXENABLE_SET;	/* AM335X Only */
  unsigned short I2C_DMARXENABLE_CLR;	/* AM335X Only */
  unsigned short I2C_DMATXENABLE_CLR;	/* AM335X Only */
  unsigned short I2C_DMARXWAKE_EN;	/* AM335X Only */
  unsigned short I2C_DMATXWAKE_EN;	/* AM335X Only */
  unsigned short I2C_SYSS;
  unsigned short I2C_BUF;
  unsigned short I2C_CNT;
  unsigned short I2C_DATA;
  unsigned short I2C_CON;
  unsigned short I2C_OA;	/* AM335X Only */
  unsigned short I2C_OA0;	/* DM37XX Only */
  unsigned short I2C_SA;
  unsigned short I2C_PSC;
  unsigned short I2C_SCLL;
  unsigned short I2C_SCLH;
  unsigned short I2C_SYSTEST;
  unsigned short I2C_BUFSTAT;
  unsigned short I2C_OA1;
  unsigned short I2C_OA2;
  unsigned short I2C_OA3;
  unsigned short I2C_ACTOA;
  unsigned short I2C_SBLOCK;
} beagle_i2c_regs;

/**
 * @name  I2C data structures.
 *
 * @{
 */

typedef struct {
  bool           is_initialized;
  rtems_id       irq_sema_id;
} beagle_i2c_softc_t;

typedef struct {
  rtems_libi2c_bus_t bus_desc;
  beagle_i2c_softc_t softc;
  beagle_i2c_regs *regs;
  uint32_t i2c_base_addrs;
  int i2c_irq;
  int i2c_bus_id;
} beagle_i2c_desc_t;

/** @} */

/* Registers defination for each chip */

static beagle_i2c_regs am335x_i2c_regs = {
	.I2C_REVNB_LO = AM335X_I2C_REVNB_LO,
	.I2C_REVNB_HI = AM335X_I2C_REVNB_HI,
	.I2C_SYSC = AM335X_I2C_SYSC,
	.I2C_IRQSTATUS_RAW = AM335X_I2C_IRQSTATUS_RAW,
	.I2C_IRQSTATUS = AM335X_I2C_IRQSTATUS,
	.I2C_IRQENABLE_SET = AM335X_I2C_IRQENABLE_SET,
	.I2C_IRQENABLE_CLR = AM335X_I2C_IRQENABLE_CLR,
	.I2C_WE = AM335X_I2C_WE,
	.I2C_DMARXENABLE_SET = AM335X_I2C_DMARXENABLE_SET,
	.I2C_DMATXENABLE_SET = AM335X_I2C_DMATXENABLE_SET,
	.I2C_DMARXENABLE_CLR = AM335X_I2C_DMARXENABLE_CLR,
	.I2C_DMATXENABLE_CLR = AM335X_I2C_DMATXENABLE_CLR,
	.I2C_DMARXWAKE_EN = AM335X_I2C_DMARXWAKE_EN,
	.I2C_DMATXWAKE_EN = AM335X_I2C_DMATXWAKE_EN,
	.I2C_SYSS = AM335X_I2C_SYSS,
	.I2C_BUF = AM335X_I2C_BUF,
	.I2C_CNT = AM335X_I2C_CNT,
	.I2C_DATA = AM335X_I2C_DATA,
	.I2C_CON = AM335X_I2C_CON,
	.I2C_OA = AM335X_I2C_OA,
	.I2C_SA = AM335X_I2C_SA,
	.I2C_PSC = AM335X_I2C_PSC,
	.I2C_SCLL = AM335X_I2C_SCLL,
	.I2C_SCLH = AM335X_I2C_SCLH,
	.I2C_SYSTEST = AM335X_I2C_SYSTEST,
	.I2C_BUFSTAT = AM335X_I2C_BUFSTAT,
	.I2C_OA1 = AM335X_I2C_OA1,
	.I2C_OA2 = AM335X_I2C_OA2,
	.I2C_OA3 = AM335X_I2C_OA3,
	.I2C_ACTOA = AM335X_I2C_ACTOA,
	.I2C_SBLOCK = AM335X_I2C_SBLOCK
};

/**
 * @name  I2C directives.
 *
 * @{
 * 
 * For more details read cpukit/libi2c/README_libi2c
 */

rtems_status_code beagle_i2c_init(rtems_libi2c_bus_t * bushdl);

rtems_status_code beagle_i2c_send_start(rtems_libi2c_bus_t * bushdl);

rtems_status_code beagle_i2c_stop(rtems_libi2c_bus_t * bushdl);

rtems_status_code 
beagle_i2c_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw);

int beagle_i2c_read_bytes(
rtems_libi2c_bus_t * bushdl, 
unsigned char *bytes, 
int nbytes
);

int beagle_i2c_write_bytes(
rtems_libi2c_bus_t * bushdl, 
unsigned char *bytes, 
int nbytes
);

int beagle_i2c_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *buffer);

int BSP_i2c_register_drivers(int i2c_bus_number);

int BSP_i2c_init(void);

/** @} */

static rtems_libi2c_bus_ops_t beagle_i2c_ops = {
  init:             beagle_i2c_init,
  send_start:       beagle_i2c_send_start,
  send_stop:        beagle_i2c_stop,
  send_addr:        beagle_i2c_send_addr,
  read_bytes:       beagle_i2c_read_bytes,
  write_bytes:      beagle_i2c_write_bytes,
  ioctl:            beagle_i2c_ioctl
};

static const uint32_t i2c_base_addrs[] = {
	AM335X_I2C0_BASE, AM335X_I2C1_BASE, AM335X_I2C0_BASE
};

static const int i2c_irq_num[] = {
	BEAGLE_I2C0_IRQ , BEAGLE_I2C1_IRQ , BEAGLE_I2C2_IRQ
};

static beagle_i2c_desc_t beagle_i2c_bus_desc_t = {
  {
    ops:			&beagle_i2c_ops,
    size:			sizeof(beagle_i2c_bus_desc_t)
  },
  {
	is_initialized:	false
  },
  &am335x_i2c_regs,
  AM335X_I2C0_BASE, /* Default I2C[0] bus selected */
  BEAGLE_I2C0_IRQ, /* Default I2C[0] irq */
  0 /* Default bus id */
};

static beagle_i2c_desc_t *beagle_i2c_bus_desc = &beagle_i2c_bus_desc_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_BEAGLE_I2C_H */