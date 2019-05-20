/*!Software License Agreement
 * ************************************************************************************************
 *
 * Software License Agreement
 *
 * Copyright © 2012 Microchip Technology Inc.  All rights reserved. Microchip licenses to you the
 * right to use, modify, copy and distribute Software only when embedded on a Microchip 
 * microcontroller or digital signal controller, which is integrated into your product or third 
 * party product (pursuant to the sublicense terms in the accompanying license agreement).
 *
 * You should refer to the license agreement accompanying this Software for additional information 
 * regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR 
 * IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT 
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR 
 * OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT  
 * LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS  
 * OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY  
 * THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *
 * ***********************************************************************************************/
/*!irq.h
 * ************************************************************************************************
 * Summary:
 * Generic Interrupt Configuration Driver Module (header file)
 *
 * Description:
 * This additional header file contains defines for all required bit-settings of all related registers.
 * This file is an additional header file on top of the generic device header file.
 * 
 * See Also:
 *	p33EGS_irq.c
 * ***********************************************************************************************/

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef _P33GS_INTERRUPT_CONFIG_DEF_H_
#define	_P33GS_INTERRUPT_CONFIG_DEF_H_

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

#include "../p33SMPS_devices.h"

extern inline uint16_t gsirq_init_irq(uint16_t regINTCON1, uint16_t regINTCON2, uint16_t regINTCON3);
extern inline uint16_t gsirq_get_current_irq_priority_level(void);
extern inline uint16_t gsirq_init_soft_traps(unsigned int accumulator_a_overflow_trap_enable, unsigned int accumulator_b_overflow_trap_enable, 
                    unsigned int accumulator_catastrophic_overflow_trap_enable);

#if defined (__P33SMPS_EP__) || defined (__P33SMPS_CH__) || defined (__P33SMPS_CK__)

// Defines for gsirq_init_soft_traps(...)
#define ACCA_OVERFLOW_TRAP_ENABLE                   1
#define ACCA_OVERFLOW_TRAP_DISABLE                  0
#define ACCB_OVERFLOW_TRAP_ENABLE                   1
#define ACCB_OVERFLOW_TRAP_DISABLE                  0
#define ACCx_CATASTROPHIC_OVERFLOW_TRAP_ENABLE      1
#define ACCx_CATASTROPHIC_OVERFLOW_TRAP_DISABLE     0


// INTCON1 Configuration Bits
#define REG_INTCON1_WRITE_BIT_MSK           0b1000011100000000      // First interrupt configuration register for write operations (no status bits)
#define REG_INTCON1_VALID_BIT_MSK           0b1111111111011110      // First interrupt configuration register for read operations (incl. status bits)

#define REG_INTCON1_NSTDIS_ENABLED          0b0000000000000000      // Nested Interrupts enabled
#define REG_INTCON1_NSTDIS_DISABLED         0b1000000000000000      // Nested Interrupts disabled

#define REG_INTCON1_OVATE_ENABLE            0b0000010000000000      // Accumulator A overflow trap enable
#define REG_INTCON1_OVATE_DISABLE           0b0000000000000000      // Accumulator A overflow trap disable

#define REG_INTCON1_OVBTE_ENABLE            0b0000001000000000      // Accumulator B overflow trap enable
#define REG_INTCON1_OVBTE_DISABLE           0b0000000000000000      // Accumulator B overflow trap disable

#define REG_INTCON1_COVTE_ENABLE            0b0000000100000000      // Accumulator catastrophic overflow trap enable
#define REG_INTCON1_COVTE_DISABLE           0b0000000000000000      // Accumulator catastrophic overflow trap disable

// INTCON1 Flag Bits
#define REG_INTCON1_OVAERR_FLAG_SET         0b0100000000000000      // Accumulator A overflow trap flag bit is set
#define REG_INTCON1_OVAERR_FLAG_CLEAR       0b0000000000000000      // Accumulator A overflow trap flag bit is cleared

#define REG_INTCON1_OVBERR_FLAG_SET         0b0010000000000000      // Accumulator B overflow trap flag bit is set
#define REG_INTCON1_OVBERR_FLAG_CLEAR       0b0000000000000000      // Accumulator B overflow trap flag bit is cleared

#define REG_INTCON1_COVAERR_FLAG_SET        0b0001000000000000      // Accumulator A catastrophic overflow trap flag bit is set
#define REG_INTCON1_COVAERR_FLAG_CLEAR      0b0000000000000000      // Accumulator A catastrophic overflow trap flag bit is cleared

#define REG_INTCON1_COVBERR_FLAG_SET        0b0000100000000000      // Accumulator B catastrophic overflow trap flag bit is set
#define REG_INTCON1_COVBERR_FLAG_CLEAR      0b0000000000000000      // Accumulator B catastrophic overflow trap flag bit is cleared

// INTCON1 Status Bits
#define REG_INTCON1_STATUS_READ_MSK         0b0000000011011110

#define REG_INTCON1_STAT_SFTACERR_SET       0b0000000010000000
#define REG_INTCON1_STAT_SFTACERR_CLEAR     0b0000000000000000

#define REG_INTCON1_STAT_MATHDIV0ERR_SET    0b0000000001010000

#define REG_INTCON1_STAT_DIV0ERR_SET        0b0000000001000000
#define REG_INTCON1_STAT_DIV0ERR_CLEAR      0b0000000000000000

#define REG_INTCON1_STAT_MATHERR_SET        0b0000000000010000
#define REG_INTCON1_STAT_MATHERR_CLEAR      0b0000000000000000

#define REG_INTCON1_STAT_ADDRERR_SET        0b0000000000001000
#define REG_INTCON1_STAT_ADDRERR_CLEAR      0b0000000000000000

#define REG_INTCON1_STAT_STKERR_SET         0b0000000000000100
#define REG_INTCON1_STAT_STKERR_CLEAR       0b0000000000000000

#define REG_INTCON1_STAT_OSCFAIL_SET        0b0000000000000010
#define REG_INTCON1_STAT_OSCFAIL_CLEAR      0b0000000000000000


// INTCON2 Configuration Bits
#define REG_INTCON2_WRITE_BIT_MSK           0b1110000100010111      // Second interrupt configuration register for write operations (no status bits)
#define REG_INTCON2_VALID_BIT_MSK           0b1110000100010111      // Second interrupt configuration register for read operations (incl. status bits)

#define REG_INTCON2_GIE_ENABLE              0b1000000000000000
#define REG_INTCON2_GIE_DISABLED            0b0000000000000000

#define REG_INTCON2_SWTRAP_ENABLE           0b0010000000000000
#define REG_INTCON2_SWTRAP_DISABLE          0b0000000000000000

#define REG_INTCON2_AIVTEN_ENABLE           0b0000000100000000
#define REG_INTCON2_AIVTEN_DISABLE          0b0000000000000000

#define REG_INTCON2_INT4EP_FALLING          0b0000000000010000
#define REG_INTCON2_INT4EP_RISING           0b0000000000000000

#define REG_INTCON2_INT2EP_FALLING          0b0000000000000100
#define REG_INTCON2_INT2EP_RISING           0b0000000000000000

#define REG_INTCON2_INT1EP_FALLING          0b0000000000000010
#define REG_INTCON2_INT1EP_RISING           0b0000000000000000

#define REG_INTCON2_INT0EP_FALLING          0b0000000000000001
#define REG_INTCON2_INT0EP_RISING           0b0000000000000000

// INTCON2 Status Bits
#define REG_INTCON2_STAT_DISI_SET           0b0100000000000000
#define REG_INTCON2_STAT_DISI_CLEAR         0b0000000000000000

#define REG_INTCON2_STAT_SWTRAP_SET         0b0010000000000000
#define REG_INTCON2_STAT_SWTRAP_CLEAR       0b0000000000000000


// INTCON3 Configuration Bits
#define REG_INTCON3_WRITE_BIT_MSK           0b0000000000000000      // Third interrupt configuration register for write operations (no status bits)
#define REG_INTCON3_VALID_BIT_MSK           0b0000001101010001      // Third interrupt configuration register for read operations (incl. status bits)

#define REG_INTCON3_STAT_CAN_SET            0b0000001000000000
#define REG_INTCON3_STAT_CAN_CLEAR          0b0000000000000000

#define REG_INTCON3_STAT_NAE_SET            0b0000000100000000
#define REG_INTCON3_STAT_NAE_CLEAR          0b0000000000000000

#define REG_INTCON3_STAT_CAN2_SET           0b0000000001000000
#define REG_INTCON3_STAT_CAN2_CLEAR         0b0000000000000000

#define REG_INTCON3_STAT_DOOVR_SET          0b0000000000010000
#define REG_INTCON3_STAT_DOOVR_CLEAR        0b0000000000000000

#define REG_INTCON3_STAT_APLL_SET           0b0000000000000001
#define REG_INTCON3_STAT_APLL_CLEAR         0b0000000000000000


// INTCON4 Configuration Bits

#define REG_INTCON4_WRITE_BIT_MSK           0b0000000000000000      // Fourth interrupt configuration register for write operations (no status bits)
#define REG_INTCON4_VALID_BIT_MSK           0b0000000000000001      // Fourth interrupt configuration register for read operations (incl. status bits)

#define REG_INTCON4_STAT_SGHT_SET           0b0000000000000001
#define REG_INTCON4_STAT_SGHT_CLEAR         0b0000000000000000

#if defined (__P33SMPS_CH__) || defined (__P33SMPS_CK__)
// ECCDBE: ECC Double-Bit Error Trap bit
  #define REG_INTCON4_STAT_ECCDBE_SET       0b0000000000000010
  #define REG_INTCON4_STAT_ECCDBE_CLEAR     0b0000000000000000
#endif

// INTTREG: INTERRUPT CONTROL AND STATUS REGISTER
#define REG_INTTREG_VALID_BIT_MSK           0b0000111111111111      // Fourth interrupt configuration register for read operations (incl. status bits)

#define REG_INTTREG_ILR_15                  0b0000111100000000
#define REG_INTTREG_ILR_14                  0b0000111000000000
#define REG_INTTREG_ILR_13                  0b0000110100000000
#define REG_INTTREG_ILR_12                  0b0000110000000000
#define REG_INTTREG_ILR_11                  0b0000101100000000
#define REG_INTTREG_ILR_10                  0b0000101000000000
#define REG_INTTREG_ILR_9                   0b0000100100000000
#define REG_INTTREG_ILR_8                   0b0000100000000000
#define REG_INTTREG_ILR_7                   0b0000011100000000
#define REG_INTTREG_ILR_6                   0b0000011000000000
#define REG_INTTREG_ILR_5                   0b0000010100000000
#define REG_INTTREG_ILR_4                   0b0000010000000000
#define REG_INTTREG_ILR_3                   0b0000001100000000
#define REG_INTTREG_ILR_2                   0b0000001000000000
#define REG_INTTREG_ILR_1                   0b0000000100000000
#define REG_INTTREG_ILR_0                   0b0000000000000000

#define REG_INTTREG_ILR_MSK                 0b0000111100000000
#define REG_INTTREG_ILR(x)                  ((x << 8) & REG_INTTREG_ILR_MSK)

#define REG_INTTREG_VECNUM_MSK              0b0000000011111111      // Fourth interrupt configuration register for read operations (incl. status bits)
#define REG_INTTREG_VECNUM(x)               (x & REG_INTTREG_VECNUM_MSK)

#else
#error === selected device not supported ===
#endif


#endif	/* _P33GS_INTERRUPT_CONFIG_DEF_H_ */

