/*LICENSE *****************************************************************************************
 *
 * Software License Agreement
 *
 * Copyright (R) 2012 Microchip Technology Inc.  All rights reserved. Microchip licenses to you the
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

// Device header file
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "p33SMPS_hspwm_c.h"


/*@@p33MP_hspwm.c
 * ************************************************************************************************
 * Summary:
 * Driver file for the dsPIC33CxxxMPxxx-PWM SFRs
 *
 * Description:
 * The SMPS PWM module offers a large number of registers and configuration options. This additional
 * driver file contains initialization routines for all required settings.
 * ***********************************************************************************************/

/*@@hspwm_power_enable
 * ************************************************************************************************
 * Summary:
 * Enables the bias power to the high resolution PWM module
 *
 * Parameters:
 *	(none)
 * 
 * Description:
 * PICmicro controllers offer so-called PERIPHERAL MODULE DISABLE (PMD) registers turning on/off
 * the bias power to on-chip peripheral blocks. This routine enables the bias power to the high-
 * resolution PWM module.
 * ***********************************************************************************************/
uint16_t hspwm_power_enable(void) {
// enable power to peripheral
    _HSPWM_POWER_ENABLE;
    return(1 - _PWMMD);
}

/*@@hspwm_power_disable
 * ************************************************************************************************
 * Summary:
 * Disables the bias power to the high resolution PWM module
 *
 * Parameters:
 *	(none)
 * 
 * Description:
 * PICmicro controllers offer so-called PERIPHERAL MODULE DISABLE (PMD) registers turning on/off
 * the bias power to on-chip peripheral blocks. This routine disables the bias power to the high-
 * resolution PWM module.
 * ***********************************************************************************************/
uint16_t hspwm_power_disable(void) {
// disable power to peripheral
    _HSPWM_POWER_DISABLE;
    return(1 - _PWMMD);
}

/*@@hspwm_init_independent_pwm
 * ************************************************************************************************
 * Summary:
 * Sets the basic configuration of a PWM generator 
 *
 * Parameters:
 *	(none)
 * 
 * Description:
 * This PWM channel configuration includes PWMxH/PWMxL outputs, selection of clock source and dividers,
 * dead times and events. Each channel configuration disables the channel by default. the PWM channel 
 * needs to be enabled by using the function call "hspwm_enable_pwm".
 * ***********************************************************************************************/

uint16_t hspwm_init_independent_pwm(
            uint16_t channel, 
            REGBLK_PCLK_CONFIG_t regPCLK, 
            REGBLK_PGxCH_CONFIG_t regPGxCON, 
            REGBLK_PGxEVT_CONFIG_t regPGxEVT,
            REGBLK_PGxIO_CONFIG_t regPGxIOCON, 
            REGBLK_PGxDEAD_TIME_t regPGxDT
    )
{
    
    volatile uint16_t fres=1;
    volatile uint16_t *regptr16;
    volatile uint32_t *regptr32;
    volatile uint16_t reg_offset;

    // write clock configuration
    regptr16 = (volatile uint16_t*) &PCLKCON;
    *regptr16 = (regPCLK.value & REG_PCLKCON_VALID_DATA_WRITE_MASK);

    // Test if written value matches parameter
    fres &= ((*regptr16 & REG_PCLKCON_VALID_DATA_READ_MASK) == (regPCLK.value & REG_PCLKCON_VALID_DATA_WRITE_MASK));
    
    // write PWM channel configuration
    reg_offset = (channel-1) * ((volatile uint16_t)&PG2CONL - (volatile uint16_t)&PG1CONL);
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&PG1CONL + reg_offset);
    *regptr32 = (regPGxCON.value & REG_PGxCON_VALID_DATA_WRITE_MASK);

    // write PWM channel event configuration
    reg_offset = (channel-1) * ((volatile uint16_t)&PG2EVTL - (volatile uint16_t)&PG1EVTL);
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&PG1EVTL + reg_offset);
    *regptr32 = (regPGxEVT.value & REG_PGxEVT_VALID_DATA_WRITE_MASK);

    // write IO configuration
    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONL - (volatile uint16_t)&PG1IOCONL);
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&PG1IOCONL + reg_offset);
    *regptr32 = (regPGxIOCON.value & REG_PGxIOCON_VALID_DATA_WRITE_MASK);

    // write dead time
    reg_offset = (channel-1) * ((volatile uint16_t)&PG2DTL - (volatile uint16_t)&PG1DTL);
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&PG1DTL + reg_offset);
    *regptr32 = (regPGxDT.value & REG_PGxDT_VALID_DATA_WRITE_MASK);
    
    return(fres);

}

/*@@hspwm_init_pwm_timing
 * ************************************************************************************************
 * Summary:
 * Sets the basic signal timing configuration of a PWM generator 
 *
 * Parameters:
 *	uint16_t channel:     Index of the PWM generator addressed (e.g. 1 for PWM1, 2 for PWM2, etc.)
 *  uint16_t regPGxPER:   PWM generator period register setting the switching period 
 *  uint16_t regPGxDC:    PWM generator duty cycle register setting the switching signal on-time 
 *  uint16_t regPGxPHASE: PWM generator phase register setting the switching signal phase shift
 * 
 * Description:
 * This function defines the PWM channel switching frequency signal timing, such as frequency/period,
 * duty cycle/on-time and phase shift.
 * ***********************************************************************************************/

uint16_t hspwm_init_pwm_timing(uint16_t channel, uint16_t regPGxPER, uint16_t regPGxDC, uint16_t regPGxPHASE)
{
    
    volatile uint16_t fres=1;
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;

    // write PWM channel configuration
    reg_offset = (channel-1) * ((volatile uint16_t)&PG2PER - (volatile uint16_t)&PG1PER);
    regptr16 = (volatile uint16_t*)((volatile uint8_t*)&PG1PER + reg_offset);
    *regptr16 = (regPGxPER & REG_PGxPER_VALID_DATA_WRITE_MASK);

    // Test if written value matches parameter
    fres &= ((*regptr16 & REG_PGxPER_VALID_DATA_READ_MASK) == (regPGxPER & REG_PGxPER_VALID_DATA_WRITE_MASK));
    
    // write IO configuration
    reg_offset = (channel-1) * ((volatile uint16_t)&PG2DC - (volatile uint16_t)&PG1DC);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1DC + reg_offset);
    *regptr16 = regPGxDC;

    // Test if written value matches parameter
    fres &= ((*regptr16 & REG_PGxDC_VALID_DATA_READ_MASK) == (regPGxDC & REG_PGxDC_VALID_DATA_WRITE_MASK));
    
    // write dead time
    reg_offset = (channel-1) * ((volatile uint16_t)&PG2PHASE - (volatile uint16_t)&PG1PHASE);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1PHASE + reg_offset);
    *regptr16 = regPGxPHASE;
    
    // Test if written value matches parameter
    fres &= ((*regptr16 & REG_PGxPHASE_VALID_DATA_READ_MASK) == (regPGxPHASE & REG_PGxPHASE_VALID_DATA_WRITE_MASK));

    return(fres);

}

/*@@hspwm_enable_pwm
 * ************************************************************************************************
 * Summary:
 * Enables a PWM generator 
 *
 * Parameters:
 *	uint16_t channel:     Index of the PWM generator addressed (e.g. 1 for PWM1, 2 for PWM2, etc.)
 *  bool wait_for_hres:   PWM generator period register setting the switching period 
 *
 * Returns:
 *  uint16_t            a 16-bit unsigned integer number indicating if the write process was 
 *                      executed successfully (0: failure, 1: success)
 * 
 * Description:
 * This function disables a PWM generator defined by parameter CHANNEL. If high resolution mode is 
 * enabled, this function waits until the high resolution ready bit is set before enabling the PWM
 * generator.
 * ***********************************************************************************************/

uint16_t hspwm_enable_pwm(uint16_t channel, bool wait_for_hres)
{

    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;
    volatile uint16_t timeout=0;
    
    if(wait_for_hres)
    {
        while((PCLKCONbits.HRRDY == PCLKCON_HRRDY_WAIT) && 
          (PCLKCONbits.HRERR == PCLKCON_HRERR_NO_ERROR) && 
            (timeout++ < 5000))
            {
                Nop();
                Nop();
                Nop();
            }

        if(timeout >= 5000) // if High Resolution Ready Bit has not been set
        { return(0); }      // skip ENABLE and return failure code

    }

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2CONL - (volatile uint16_t)&PG1CONL);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1CONL + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf | REG_PGCON_ON_PWM_ENABLED;
    
    return((volatile uint16_t)(volatile bool)(*regptr16 & REG_PGCON_ON_PWM_ENABLED));

}

/*@@hspwm_disable_pwm
 * ************************************************************************************************
 * Summary:
 * Disables a PWM generator 
 *
 * Parameters:
 *	uint16_t channel:     Index of the PWM generator addressed (e.g. 1 for PWM1, 2 for PWM2, etc.)
 *
 * Returns:
 *  uint16_t            a 16-bit unsigned integer number indicating if the write process was 
 *                      executed successfully (0: failure, 1: success)
 *  
 * Description:
 * This function disables a PWM generator defined by parameter CHANNEL.
 * ***********************************************************************************************/

uint16_t hspwm_disable_pwm(uint16_t channel)
{

    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2CONL - (volatile uint16_t)&PG1CONL);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1CONL + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf & REG_PGCON_ON_PWM_RESET;
    
    return((volatile uint16_t)(volatile bool)(*regptr16 & REG_PGCON_ON_PWM_ENABLED));
}

uint16_t hspwm_ovr_hold(uint16_t channel)
{
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONL - (volatile uint16_t)&PG1IOCONL);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1IOCONL + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf | REG_IOCON_OVREN_COMP_SET;
    
    return((volatile uint16_t)(volatile bool)(*regptr16 & REG_IOCON_OVREN_COMP_SET));
}


uint16_t hspwm_ovr_release(uint16_t channel)
{
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONL - (volatile uint16_t)&PG1IOCONL);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1IOCONL + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf & REG_IOCON_OVREN_COMP_RESET;
    
    return(1 - (volatile uint16_t)(volatile bool)(*regptr16 & REG_IOCON_OVREN_COMP_SET));
}

uint16_t hspwm_ovr_release_high_side(uint16_t channel)
{
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONL - (volatile uint16_t)&PG1IOCONL);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1IOCONL + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf & REG_IOCON_OVRENH_RESET;
    
    return(1 - (volatile uint16_t)(volatile bool)(*regptr16 & REG_IOCON_OVRENH_ENABLED));
}

uint16_t hspwm_ovr_release_low_side(uint16_t channel)
{
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONL - (volatile uint16_t)&PG1IOCONL);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1IOCONL + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf & REG_IOCON_OVRENL_RESET;
    
    return(1 - (volatile uint16_t)(volatile bool)(*regptr16 & REG_IOCON_OVRENL_ENABLED));
}

uint16_t hspwm_set_gpio_high_side(uint16_t channel)
{
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONH - (volatile uint16_t)&PG1IOCONH);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1IOCONH + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf & REG_IOCON_PENH_GPIO_ENABLE;
    
    return((volatile uint16_t)(volatile bool)(*regptr16 & REG_IOCON_PENH_GPIO_DISABLE));
}

uint16_t hspwm_reset_gpio_high_side(uint16_t channel)
{
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONH - (volatile uint16_t)&PG1IOCONH);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1IOCONH + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf | REG_IOCON_PENH_GPIO_DISABLE;
    
    return(1 - (volatile uint16_t)(volatile bool)(*regptr16 & REG_IOCON_PENH_GPIO_DISABLE));
}

uint16_t hspwm_set_gpio_low_side(uint16_t channel)
{
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONH - (volatile uint16_t)&PG1IOCONH);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1IOCONH + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf & REG_IOCON_PENL_GPIO_ENABLE;
    
    return((volatile uint16_t)(volatile bool)(*regptr16 & REG_IOCON_PENL_GPIO_DISABLE));
}


uint16_t hspwm_reset_gpio_low_side(uint16_t channel)
{
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    reg_offset = (channel-1) * ((volatile uint16_t)&PG2IOCONH - (volatile uint16_t)&PG1IOCONH);
    regptr16 = (volatile uint16_t*) ((volatile uint8_t*)&PG1IOCONH + reg_offset);
    reg_buf = *regptr16;
    *regptr16 = reg_buf | REG_IOCON_PENL_GPIO_DISABLE;
    
    return(1 - (volatile uint16_t)(volatile bool)(*regptr16 & REG_IOCON_PENL_GPIO_DISABLE));
}
/* *****************************************************************************************************
 * *****************************************************************************************************/

