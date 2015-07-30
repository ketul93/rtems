/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief Basic BSP IRQ info.
 */

#ifndef LIBBSP_ARM_BEAGLE_IRQ_H
#define LIBBSP_ARM_BEAGLE_IRQ_H

#ifndef ASM

#include <rtems.h>
#include <rtems/irq.h>
#include <rtems/irq-extension.h>

#define BSP_INTERRUPT_VECTOR_MIN 0
#define BSP_INTERRUPT_VECTOR_MAX 127

#if IS_AM335X
#define BEAGLE_I2C0_IRQ 70
#define BEAGLE_I2C1_IRQ 71
#define BEAGLE_I2C2_IRQ 30
#endif

#if IS_DM3730
#define BEAGLE_I2C0_IRQ 56
#define BEAGLE_I2C1_IRQ 57
#define BEAGLE_I2C2_IRQ 61
#endif

#endif /* ASM */

#endif /* LIBBSP_ARM_BEAGLE_IRQ_H */
