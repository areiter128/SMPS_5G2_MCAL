/*LICENSE ********************************************************************
 * Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 * ***************************************************************************/
/*!p33MP_timer.c
 * ***************************************************************************
 * Summary:
 * Driver file for the dsPIC33xxxGS Timer SFRs
 *
 * Description:
 * The timer module offers a number of registers and configuration options. This additional
 * driver file contains initialization routines for all required settings.
 *
 * File:   p33SMPS_timer.c
 * Author: M91406
 *
 * Created on October 25, 2017, 4:18 PM
 * 
 * Revision:
 * 
 * ***************************************************************************/

// Device header file
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "p33SMPS_timer.h"


/*!p33SMPS_timer.h
 * ************************************************************************************************
 * Summary:
 * Driver file for the dsPIC33CxxMP Timer SFRs
 *
 * Description:
 * The timer module offers a number of registers and configuration options. This additional
 * driver file contains initialization routines for all required settings.
 * ***********************************************************************************************/


volatile uint16_t smpsTimer_PowerOn(uint16_t instance) 
{

    volatile uint16_t fres=0;
    
    #ifdef PMDCON
    _PMDLOCK = 1; // Peripheral Module Disable-bits (1=can be set, 0=cannot be set) in software
    #endif
    switch (instance) {
        #if defined (TMR1)
        case 1:
            _T1MD = 0;    // Turn on power to Timer1 Module
            fres = (1-_T1MD);
            break;
        #endif
        #if defined (TMR2)
        case 2:
            _T2MD = 0;    // Turn on power to Timer2 Module
            fres = (1-_T2MD);
            break;
        #endif
        #if defined (TMR3)
        case 3:
            _T3MD = 0;    // Turn on power to Timer3 Module
            fres = (1-_T3MD);
            break;
        #endif
        #if defined (TMR4)
        case 4:
            _T4MD = 0;    // Turn on power to Timer4 Module
            fres = (1-_T4MD);
            break;
        #endif
        #if defined (TMR5)
        case 5:
            _T5MD = 0;    // Turn on power to Timer5 Module
            fres = (1-_T5MD);
            break;
        #endif
        default:
            fres = 0;
            break;
    }
    #ifdef PMDCON
    _PMDLOCK = 0; // Peripheral Module Disable-bits (1=can be set, 0=cannot be set) in software
    #endif

    return(fres);

}

volatile uint16_t smpsTimer_PowerOff(uint16_t instance) 
{

    volatile uint16_t fres=0;
    
    #ifdef PMDCON
    _PMDLOCK = 1; // Peripheral Module Disable-bits (1=can be set, 0=cannot be set) in software
    #endif
    switch (instance) {
        #if defined (TMR1)
        case 1:
            _T1MD = 1;    // Turn off power to Timer1 Module
            fres = _T1MD;
            break;
        #endif
        #if defined (TMR2)
        case 2:
            _T2MD = 1;    // Turn off power to Timer2 Module
            fres = _T2MD;
            break;
        #endif
        #if defined (TMR3)
        case 3:
            _T3MD = 1;    // Turn off power to Timer3 Module
            fres = _T3MD;
            break;
        #endif
        #if defined (TMR4)
        case 4:
            _T4MD = 1;    // Turn off power to Timer4 Module
            fres = _T4MD;
            break;
        #endif
        #if defined (TMR5)
        case 5:
            _T5MD = 1;    // Turn off power to Timer5 Module
            fres = _T5MD;
            break;
        #endif
        default:
            fres = 0;
            break;
    }
    #ifdef PMDCON
    _PMDLOCK = 0; // Peripheral Module Disable-bits (1=can be set, 0=cannot be set) in software
    #endif

    return(fres);

}

/*!smpsTimer_Initialize16b
 * ************************************************************************************************
 * Summary:
 * Initializes a specific timer unit in 16-bit mode
 *
 * Parameters:
 *	index		= selects the register address range of the target timer unit
 *	regTCON		= configures basic features of the selected timer unit
 *	period		= holds the period as 32bit number. The init routine will also take care about 
 *				  the correct pre-scaler setting to achieve the given period, even if the
 *				  timer is configured in 16-bit mode
 *
 * Description:
 * This routine is setting the timer pre-scaler in accordance to apply the
 * desired period using a 16-bit time base. Therefore the 16bit mode is enforced
 * and any pre-scaler settings will be ignored. The timer will remain disabled after
 * initialization and has to be enabled by the user.
 * ***********************************************************************************************/


volatile uint16_t smpsTimer_Initialize16b(uint16_t instance, TxCON_CONTROL_REGISTER_t regTCON, 
                    uint32_t period, TIMER_ISR_PRIORITY_e isr_priority)
{

    volatile uint32_t per_buf=0;
    volatile uint16_t reg_buf=0, tmr_reg_msk=0;

    if (!smpsTimer_PowerOn(instance)) return(0);
	if (instance > GSTMR_TIMER_COUNT) return(0); // Skip if index is out of range

	// Set register bit masks to prevent setting of unsupported features
	if (instance == 1)							tmr_reg_msk = TIMER1_TCON_REG_WRITE_MASK;
	else if ((instance == 2) || (instance == 4))	tmr_reg_msk = TIMER2_4_TCON_REG_WRITE_MASK;
	else									tmr_reg_msk = TIMERx_TCON_REG_WRITE_MASK;

	// Select required Pre-Scaler to achieve requested period
	per_buf = period;					// copy period parameter

	if (per_buf <= 0xFFFF)				// Length of calculated period fits into 16bit
	{
		reg_buf = REG_TCKPS_1_to_1;		// Set Pre-Scaler 1:1
	}
	else if (per_buf < 0x07FFF9)		// If length of calculated period fits into 19bit
	{
		per_buf >>= 3;					// Divide timer period by 8
		reg_buf = REG_TCKPS_1_to_8;		// and set Pre-Scaler 1:8
	}
	else if (per_buf < 0x3FFFC1)		// If length of calculated period fits into 22bit
	{
		per_buf >>= 6;					// Divide timer period by 64
		reg_buf = REG_TCKPS_1_to_64;	// and set Pre-Scaler 1:64
	}
	else if (per_buf < 0xFFFF01)		// If length of calculated period fits into 24bit
	{
		per_buf >>= 8;					// Divide timer period by 256
		reg_buf = REG_TCKPS_1_to_256;	// and set Pre-Scaler 1:256
	}
	else
	{
		return(0);						// Uuups, really ??? What super-number was that ???
	}

	// Initialize timer configuration register.
	//
	// Please note:
	//  The timer will remain disabled and requires to be enabled by the user in software.
	//  Further, this routine is setting the timer pre-scaler in accordance to apply the
	//  desired period using a 16-bit time base. Therefore the 16bit mode is enforced
	//  and any pre-scaler settings will be ignored.

	switch (instance)
	{
	
	case 1:

        TIMER1_ISR_ENABLE	= 0;			// disable interrupt
        TIMER1_ISR_PRIORITY	= isr_priority;	// set interrupt priority
        PR1		= (unsigned int)per_buf;	// Timer Period
        T1CON	= ((reg_buf | regTCON.value) &	// Pre-Scaler 1:1, 1:8, 1:64 or 1:256
                    tmr_reg_msk);	// apply user settings

        break;

	#if (GSTMR_TIMER_COUNT >= 2)
	case 2:

        TIMER2_ISR_ENABLE	= 0;			// disable interrupt
        TIMER2_ISR_PRIORITY	= isr_priority;	// set interrupt priority
        PR2		= (unsigned int)per_buf;	// Timer Period
        T2CON	= reg_buf 			|		// Pre-Scaler 1:1, 1:8, 1:64 or 1:256
                  (regTCON.value & tmr_reg_msk);	// apply user settings

        break;

	#endif

	#if (GSTMR_TIMER_COUNT >= 3)
	case 3:

        TIMER3_ISR_ENABLE	= 0;			// disable interrupt
        TIMER3_ISR_PRIORITY	= isr_priority;	// set interrupt priority
        PR3		= (unsigned int)per_buf;	// Timer Period
        T3CON	= reg_buf 			|		// Pre-Scaler 1:1, 1:8, 1:64 or 1:256
                  (regTCON.value & tmr_reg_msk);	// apply user settings

        break;

	#endif

	#if (GSTMR_TIMER_COUNT >= 4)
	case 4:

        TIMER4_ISR_ENABLE	= 0;			// disable interrupt
        TIMER4_ISR_PRIORITY	= isr_priority;	// set interrupt priority
        PR4		= (unsigned int)per_buf;	// Timer Period
        T4CON	= reg_buf 			|		// Pre-Scaler 1:1, 1:8, 1:64 or 1:256
                  (regTCON.value & tmr_reg_msk);	// apply user settings

        break;

	#endif

	#if (GSTMR_TIMER_COUNT >= 5)
	case 5:

        TIMER5_ISR_ENABLE	= 0;			// disable interrupt
        TIMER5_ISR_PRIORITY	= isr_priority;	// set interrupt priority
        PR5		= (unsigned int)per_buf;	// Timer Period
        T5CON	= reg_buf 			|		// Pre-Scaler 1:1, 1:8, 1:64 or 1:256
                  (regTCON.value & tmr_reg_msk);	// apply user settings

        break;
	
	#endif
	
	default:
        return(0);		// error: timer index out of range

	}
	
    /* ToDo: Add write/read-comparison to verify configuration was set correctly */
	return(1);

}

volatile uint16_t smpsTimer_GetTimerConfig(uint16_t instance, TxCON_CONTROL_REGISTER_t *regTCON, uint32_t period, TIMER_ISR_PRIORITY_e *isr_priority)
{
    
    volatile uint32_t reg_buf=0;

	if (instance > GSTMR_TIMER_COUNT) return(0); // Skip if index is out of range

	// Read period register
    switch (instance)
    {
        #if (GSTMR_TIMER_COUNT >= 1)
        case 1: 
            *regTCON = (volatile TxCON_CONTROL_REGISTER_t)(T1CON & TIMER1_TCON_REG_READ_MASK);
            reg_buf = (volatile uint32_t)PR1;
            *isr_priority = (volatile uint16_t)TIMER1_ISR_PRIORITY;
            break;
        #endif
        #if (GSTMR_TIMER_COUNT >= 2)
        case 2: 
            *regTCON = (volatile TxCON_CONTROL_REGISTER_t)(T2CON & TIMER2_4_TCON_REG_READ_MASK);
            reg_buf  = (volatile uint32_t)PR2;
            *isr_priority = (volatile uint16_t)TIMER2_ISR_PRIORITY;
            break;
        #endif
        #if (GSTMR_TIMER_COUNT >= 3)
        case 3:
            *regTCON = (volatile TxCON_CONTROL_REGISTER_t)(T3CON & TIMERx_TCON_REG_READ_MASK);
            reg_buf = (volatile uint32_t)PR3;
            *isr_priority = (volatile uint16_t)TIMER3_ISR_PRIORITY;
            break;
        #endif
        #if (GSTMR_TIMER_COUNT >= 4)
        case 4:
            *regTCON = (volatile TxCON_CONTROL_REGISTER_t)(T4CON & TIMER2_4_TCON_REG_READ_MASK);
            reg_buf  = (volatile uint32_t)PR4;
            *isr_priority = (volatile uint16_t)TIMER4_ISR_PRIORITY;
            break;
        #endif
        #if (GSTMR_TIMER_COUNT >= 5)
        case 5:
            *regTCON = (volatile TxCON_CONTROL_REGISTER_t)(T5CON & TIMERx_TCON_REG_READ_MASK);
            reg_buf = (volatile uint32_t)PR5;
            *isr_priority = (volatile uint16_t)TIMER5_ISR_PRIORITY;
            break;
        #endif
        default: return(0);
    }
    
    // decode period to full length
    switch(regTCON->bits.tckps)
    {
        case TCKPS_DIV_1_to_1:      period  = (volatile uint32_t)reg_buf; break;
        case TCKPS_DIV_1_to_8:      period  = (volatile uint32_t)(reg_buf<<3); break;
        case TCKPS_DIV_1_to_64:     period  = (volatile uint32_t)(reg_buf<<5); break;
        case TCKPS_DIV_1_to_256:    period  = (volatile uint32_t)(reg_buf<<8); break;
        default: return(0);
    }
    
    
    return(1);

}


/*!smpsTimer_Enable
 * ************************************************************************************************
 * Summary:
 * Enables a specific timer unit in 16-bit mode
 *
 * Parameters:
 *	index		= selects the register address range of the target timer unit
 *
 * Description:
 * This routine is enabling the selected timer as it was configured previously.
 * ***********************************************************************************************/


volatile uint16_t smpsTimer_Enable(uint16_t instance, TIMER_ISR_ENABLE_STATE_e isr_enable)
{
	
    if (!smpsTimer_PowerOn(instance)) return(0);
	if (instance > GSTMR_TIMER_COUNT) return(0); // Skip if index is out of range
	
	switch (instance)
	{
	
	case 1: 
        TMR1				= 0;			// Reset timer counter
        TIMER1_ISR_FLAG		= 0;			// Clear the timer interrupt flag
        TIMER1_ISR_ENABLE	= isr_enable;	// Enable/disable ISR
        T1CON 			   |= 0x8000;		// enable timer
        break;

	#if (GSTMR_TIMER_COUNT >= 2)
	case 2:
        TMR2				= 0;			// Reset timer counter
        TIMER2_ISR_FLAG		= 0;			// Clear the timer interrupt flag
        TIMER2_ISR_ENABLE	= isr_enable;	// Enable/disable ISR
        T2CON 			   |= 0x8000;		// enable timer
        break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 3)
	case 3:
        TMR3				= 0;			// Reset timer counter
        TIMER3_ISR_FLAG		= 0;			// Clear the timer interrupt flag
        TIMER3_ISR_ENABLE	= isr_enable;	// Enable/disable ISR
        T3CON 			   |= 0x8000;		// enable timer
        break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 4)
	case 4:
        TMR4				= 0;			// Reset timer counter
        TIMER4_ISR_FLAG		= 0;			// Clear the timer interrupt flag
        TIMER4_ISR_ENABLE	= isr_enable;	// Enable/disable ISR
        T4CON 			   |= 0x8000;		// enable timer
        break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 5)
	case 5:
        TMR5				= 0;			// Reset timer counter
        TIMER5_ISR_FLAG		= 0;			// Clear the timer interrupt flag
        TIMER5_ISR_ENABLE	= isr_enable;	// Enable/disable ISR
        T5CON 			   |= 0x8000;		// enable timer
        break;
	#endif
	
	default:
        return(0);		// error: timer index out of range

	}
    /* ToDo: add write/read comparison to verify configuration was set correctly */
	return(1);

}


/*!smpsTimer_Disable
 * ************************************************************************************************
 * Summary:
 * Disables a specific timer unit in 16-bit mode
 *
 * Parameters:
 *	index		= selects the register address range of the target timer unit
 *
 * Description:
 * This routine is disabling the selected timer.
 * ***********************************************************************************************/


volatile uint16_t smpsTimer_Disable(uint16_t instance)
{
	
	if (instance > GSTMR_TIMER_COUNT) return(0); // Skip if index is out of range
	
	switch (instance)
	{
	
	case 1: 
			T1CON &= 0x7FFF;
			break;

	#if (GSTMR_TIMER_COUNT >= 2)
	case 2:
			T2CON &= 0x7FFF;
			break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 3)
	case 3:
			T3CON &= 0x7FFF;
			break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 4)
	case 4:
			T4CON &= 0x7FFF;
			break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 5)
	case 5:
			T5CON &= 0x7FFF;
			break;
	#endif
	
	default:
			return(0);		// error: timer index out of range

	}

	return(1);

}

/*!smpsTimer_Reset
 * ************************************************************************************************
 * Summary:
 * Resets a specific timer unit in 16-bit mode
 *
 * Parameters:
 *	index		= selects the register address range of the target timer unit
 *
 * Description:
 * This routine is disabling the selected timer and resets its entire configuration.
 * ***********************************************************************************************/

volatile uint16_t smpsTimer_Reset(uint16_t instance)
{
	
	if (instance > GSTMR_TIMER_COUNT) return(0); // Skip if index is out of range
	
	switch (instance)
	{
	
	case 1: 
			T1CON = 0x0000;
			break;

	#if (GSTMR_TIMER_COUNT >= 2)
	case 2:
			T2CON = 0x0000;
			break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 3)
	case 3:
			T3CON = 0x0000;
			break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 4)
	case 4:
			T4CON = 0x0000;
			break;
	#endif

	#if (GSTMR_TIMER_COUNT >= 5)
	case 5:
			T5CON = 0x0000;
			break;
	#endif
	
	default:
			return(0);		// error: timer index out of range

	}

	return(1);

}

/*!smpsTimer_Dispose
 * ************************************************************************************************
 * Summary:
 * Resets a specific timer and turns off the power
 *
 * Parameters:
 *	index		= selects the register address range of the target timer unit
 *
 * Description:
 * This routine is disabling the selected timer and resets its entire configuration.
 * ***********************************************************************************************/

volatile uint16_t smpsTimer_Dispose(uint16_t instance)
{
    volatile uint16_t fres=0;
    
    fres  = smpsTimer_Disable(instance);
    fres |= smpsTimer_Reset(instance);
    fres |= smpsTimer_PowerOff(instance);
    
    return(fres);
}

// EOF
