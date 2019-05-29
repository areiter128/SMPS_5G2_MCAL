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
/* @@p33SMPS_oscillator.c
 * ***************************************************************************
 *
 * File:   p33SMPS_oscillator.c
 * Author: M91406
 *
 * Created on October 27, 2017, 11:24 AM
 * ***************************************************************************/

/* ************************************************************************************************
 * PRIVATE DEFINES
 * ************************************************************************************************/

#include <xc.h>
#include <stdint.h>

#include "../p33SMPS_devices.h"
#include "p33SMPS_oscillator.h"

/* ************************************************************************************************
 * PRIVATE DEFINES
 * ************************************************************************************************/
#define OSC_CLKSW_TIMEOUT	5000	// value to set the timeout for clock switching operations


/*!init_FRC_Defaults(OSCCON_xOSC_e osc_type)
 * ************************************************************************************************
 * Summary:
 * Initializes the major oscillator and the PLL module step by step by using clock switching
 * in software. Each step is tested and verified. PLL settings here are pulled from pre-defined 
 * settings set for default CPU speeds from 40 MIPS to 100 MIPS
 *
 * Parameters:
 *	OSCCON_xOSC_e osc_type: allows selection of internal and/or external clocks
 *  CLKDIV_FRCDIVN_e frc_div: allows using the FCR divider
 *  OSCTUN_TUN_e frc_tun: allow tuning of the FRC
 *
 * Returns:
 *  0 = unspecified clock failure detected
 *  1 = clock switch successful
 *  2 = clock switch failed
 *  4 = currently selected clock differs from selected clock source
 *  8 = PLL didn't lock in within given time frame
 *
 * Description:
 * Microchip's 16-Bit devices offer a safe 2-step start-up mode, using the internal FRC during power up, 
 * followed by a user defined switch-over to the desired oscillator. 
 * Though this can also be done in hardware automatically, this software-version of the switch-over offers
 * a better solution to verify each step and enables the user to implement some error handling in the case
 * of failure.
 * This function can be used to select a new oscillator at runtime. Each configuration step
 * will be verified before the next step is performed.
 *
 * Please Note:
 * If a oscillator switch-over is performed, additional settings in the _FOSCSEL and _FOSC
 * registers of the configuration bits may be required
 * ************************************************************************************************/

inline volatile uint16_t init_FRCCLK_Defaults(CPU_SPEED_DEFAULTS_e cpu_speed)
{
    volatile int16_t fres = 0;
    volatile OSC_CONFIG_t osc;
    
    osc.osc_type = OSCCON_xOSC_FRCPLL;
    osc.N1 = CLKDIV_PLLDIV_N1_1;
    
    switch(cpu_speed)
    {
        case CPU_SPEED_20_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_20;
            break;
        case CPU_SPEED_30_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_30;
            break;
        case CPU_SPEED_40_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_40;
            break;
        case CPU_SPEED_50_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_50;
            break;
        case CPU_SPEED_60_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_60;
            break;
        case CPU_SPEED_70_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_70;
            break;
        case CPU_SPEED_80_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_80;
            break;
        case CPU_SPEED_90_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_90;
            break;
        case CPU_SPEED_100_MIPS:
            osc.M = PLLFBD_PLLFBDIV_M_100;
            break;
        default:
            return(0);  
            break;
    }
    osc.N2 = PLLDIV_POST2DIV_N2N3_2;
    osc.N3 = PLLDIV_POST2DIV_N2N3_1; 
    
    fres = init_FOSC(osc);

    return(fres);
}


/*!init_FRCOSC(OSC_CONFIG_t osc_config)
 * ************************************************************************************************
 * Summary:
 * Initializes the internal RC oscillator divider and tuning register
 *
 * Parameters:
 *	OSC_CONFIG_t osc_config (includes oscillator type and PLL configuration)
 *
 * Returns:
 *  0 = unspecified clock failure detected
 *  1 = clock switch successful
 *  2 = clock switch failed
 *  4 = currently selected clock differs from selected clock source
 *  8 = PLL didn't lock in within given time frame
 *
 * Description:
 * Microchip's 16-Bit devices offer a safe 2-step start-up mode, using the internal FRC during power up, 
 * followed by a user defined switch-over to the desired oscillator. 
 * Though this can also be done in hardware automatically, this software-version of the switch-over offers
 * a better solution to verify each step and enables the user to implement some error handling in the case
 * of failure.
 * This function can be used to select a new oscillator at runtime. Each configuration step
 * will be verified before the next step is performed.
 *
 * Please Note:
 * If a oscillator switch-over is performed, additional settings in the _FOSCSEL and _FOSC
 * registers of the configuration bits may be required
 * ************************************************************************************************/
inline volatile uint16_t init_FRCOSC(CLKDIV_FRCDIVN_e frc_div, OSCTUN_TUN_e frc_tune)
{
#if defined (__P33SMPS_CH_MSTR__) || defined (__P33SMPS_CK__)

// Slave cores do not have access to the FRC oscillator tuning register
    
    volatile uint16_t err=0;

    // Set oscillator tuning
    // => FRC Oscillator tuning is only available on single core devices and 
    //    the master core of dual core devices
    if(frc_tune != OSCTUNbits.TUN)      // If oscillator tuning is requested...
    {
    	OSCTUNbits.TUN = frc_tune;           // Set Tuning Register for FRC Oscillator
    }

    // Set FRC divider
    if(frc_div != CLKDIVbits.FRCDIV)     // If fast RC oscillator scaling is requested...
    { CLKDIVbits.FRCDIV = frc_div; }     // Set FRC frequency divider

    return(err);
    
#else
    return(1);
#endif
    
}

/*!init_FOSC(OSC_CONFIG_t osc_config)
 * ************************************************************************************************
 * Summary:
 * Initializes the major oscillator and the PLL module step by step by using clock switching
 * in software. Each step is tested and verified
 *
 * Parameters:
 *	OSC_CONFIG_t osc_config (includes oscillator type and PLL configuration)
 *
 * Returns:
 *  0 = unspecified clock failure detected
 *  1 = clock switch successful
 *  2 = clock switch failed
 *  4 = currently selected clock differs from selected clock source
 *  8 = PLL didn't lock in within given time frame
 *
 * Description:
 * Microchip's 16-Bit devices offer a safe 2-step start-up mode, using the internal FRC during power up, 
 * followed by a user defined switch-over to the desired oscillator. 
 * Though this can also be done in hardware automatically, this software-version of the switch-over offers
 * a better solution to verify each step and enables the user to implement some error handling in the case
 * of failure.
 * This function can be used to select a new oscillator at runtime. Each configuration step
 * will be verified before the next step is performed.
 *
 * Please Note:
 * If a oscillator switch-over is performed, additional settings in the _FOSCSEL and _FOSC
 * registers of the configuration bits may be required
 * ************************************************************************************************/

inline volatile uint16_t init_FOSC(OSC_CONFIG_t osc_config)
{

uint16_t _n=0, err=0;

    // =============================================================================================
    // First make sure we are not running from a PLL derived source, when modifying the PLL settings.
    // This can be accomplished by intentionally switching to a known non-PLL setting, such as FRC.
    // Switch to FRC (no divider, no PLL), assuming we aren't already running from that.
    // =============================================================================================

    if(OSCCONbits.COSC != 0b000)
    {
        // NOSC = 0b000 = FRC without divider or PLL
        __builtin_write_OSCCONH(OSCCON_xOSC_FRC);  
        // Clear CLKLOCK and assert OSWEN = 1 to initiate switch-over
        __builtin_write_OSCCONL((OSCCON & 0x7E) | 0x01); 
        //Wait for switch over to complete.
        while((OSCCONbits.COSC != OSCCONbits.NOSC) && (_n++ < OSC_CLKSW_TIMEOUT)); 
        if (_n >= OSC_CLKSW_TIMEOUT) err = OSCERR_RST;
    }

    // Switch to target oscillator
	if ((OSCCONbits.COSC != osc_config.osc_type) && (OSCCONbits.CLKLOCK == 0))
	{
	// Switch to desired system clock, if clock switching is enabled

        #if defined (__P33SMPS_CH__) || defined (__P33SMPS_CK__)
    	// Configure PLL pre-scaler, PLL post-scaler, PLL divisor

        // Clock switch to desired CPU frequency with the PLL enabled in advance
        // Configure PLL pre-scaler, both PLL post-scalers, and PLL feedback divider
        PLLDIVbits.VCODIV = osc_config.VCODIV;    // FVCO Scaler 1:n
        CLKDIVbits.PLLPRE = osc_config.N1;   // N1 = PLL pre-scaler
        PLLFBDbits.PLLFBDIV = osc_config.M;  // M  = PLL feedback divider
        PLLDIVbits.POST1DIV = osc_config.N2; // N2 = PLL post-scalers #1
        PLLDIVbits.POST2DIV = osc_config.N3; // N3 = PLL post-scalers #2

        #elif defined (__P33SMPS_FJ__) || defined (__P33SMPS_FJA__) || defined (__P33SMPS_FJC__) || \
              defined (__P33SMPS_EP__)

        CLKDIVbits.PLLPRE	= osc_config.N1;		// N1 (non zero)
        PLLFBD				= (osc_config.M - 2);   // M  = PLLFBD 
        CLKDIVbits.PLLPOST	= osc_config.N2;        // N2 (non zero)

        #endif
        
        // Initiate Clock Switch to FRC Oscillator with PLL (NOSC=0b011)
        __builtin_write_OSCCONH(osc_config.osc_type);
        if(OSCCONbits.COSC != OSCCONbits.NOSC)
        {
            // Assert OSWEN and make sure CLKLOCK is clear, to initiate the switching operation
            __builtin_write_OSCCONL((OSCCON & 0x7F) | 0x01);    
            // Wait for clock switch to finish
            while((OSCCONbits.COSC != OSCCONbits.NOSC) && (_n++ < OSC_CLKSW_TIMEOUT));  
            if ((OSCCONbits.COSC != OSCCONbits.NOSC) || (_n >= OSC_CLKSW_TIMEOUT))
            { err = OSCERR_CSF; }
        }

	}
	else if ((OSCCONbits.COSC != osc_config.osc_type) && (OSCCONbits.CLKLOCK == 1))
	{ // If clock switching is disabled and the current clock differs from the desired clock,
	  // return error code
			err = OSCERR_CSD;
	}
        
    // Lock registers against accidental changes
    OSCCONbits.CLKLOCK = 1;
    
    while((OSCCONbits.LOCK != 1) && (_n++ < OSC_CLKSW_TIMEOUT)); // Wait n while loops for PLL to Lock
	if ((OSCCONbits.LOCK != 1) || (_n >= OSC_CLKSW_TIMEOUT)) // Error occurred? 
	{ err = OSCERR_PLL_LCK; } // => If so, return error code
    

// Return Success/Failure
    if (err == 0)
    { 
        return((1 - OSCCONbits.CF));					// Return oscillator fail status bit
    }													// (1=Success, 0=Failure)
    else
    {  return(err); }                                   // Return error code

}

/*!init_AUXCLK(AUXOSC_CONFIG_t aux_clock_config)
 * ************************************************************************************************
 * Summary:
 * Initializes the auxiliary clock and its PLL module step by step 
 * in software. Each step is tested and verified
 *
 * Parameters:
 *	AUXOSC_CONFIG_t aux_clock_config (includes oscillator settings and PLL configuration)
 *
 * Returns:
 *  0 = unspecified clock failure detected
 *  1 = clock switch successful
 *  2 = clock switch failed
 *  4 = currently selected clock differs from selected clock source
 *  8 = PLL didn't lock in within given time frame
 *
 * Description:
 * The auxiliary clock is generated by a separate PLL module, which is driven
 * from one of the main clock signals or PLL outputs. This auxiliary clock can
 * be used to drive ADCs, PWM, DACs and other peripherals. Each of them might
 * have individual requirements. Please refer to the specific peripheral sections
 * in the device data sheet to learn how to configure the auxiliary clock.
 *
 * ************************************************************************************************/

inline volatile uint16_t init_AUXCLK(AUXOSC_CONFIG_t aux_clock_config)
{
    volatile uint16_t _n=0;
    
	#if defined (__P33SMPS_CH__) || defined (__P33SMPS_CK__)

    // Set AVCO divider of Auxiliary PLL 
    APLLDIV1bits.AVCODIV   = aux_clock_config.AVCODIV;  // AVCO Scaler 1:n

    // Configure APLL pre-scaler, APLL post-scaler, APLL divisor
    ACLKCON1bits.APLLPRE   = aux_clock_config.N1;		// N1 (non zero)
	APLLFBD1bits.APLLFBDIV = aux_clock_config.M;        // M  = APLLFBD 
    APLLDIV1bits.APOST1DIV = aux_clock_config.N2;       // N2 (non zero)
    APLLDIV1bits.APOST2DIV = aux_clock_config.N3;       // N3 (non zero)

    // Select clock input source (either primary oscillator or internal FRC)
    ACLKCON1bits.FRCSEL = aux_clock_config.FRCSEL;
	
    // Set Enable-bit of Auxiliary PLL 
    ACLKCON1bits.APLLEN = aux_clock_config.APLLEN;

    // if user has not enabled the APLL module, exit here
    if(!ACLKCON1bits.APLLEN)
    { return(1); }
        
    // Wait 5000 while loops for APLL to Lock
    while((ACLKCON1bits.APLLCK != 1) && (_n++<OSC_CLKSW_TIMEOUT));		
	if (ACLKCON1bits.APLLCK != 1)	// PLL still not locked in? 
	{ return OSCERR_APLL_LCK; } // => If yes, return error code
    else
    { return(ACLKCON1bits.APLLCK); }

    #endif
    
}
 
/*!init_AUXCLK_Defaults(AUXOSC_CONFIG_t aux_clock_config)
 * ************************************************************************************************
 * Summary:
 * Initializes the auxiliary clock and its PLL module step by step 
 * in software. Each step is tested and verified
 *
 * Parameters:
 *	AUXOSC_CONFIG_t aux_clock_config (includes oscillator settings and PLL configuration)
 *
 * Returns:
 *  0 = unspecified clock failure detected
 *  1 = clock switch successful
 *  2 = clock switch failed
 *  4 = currently selected clock differs from selected clock source
 *  8 = PLL didn't lock in within given time frame
 *
 * Description:
 * The auxiliary clock is generated by a separate PLL module, which is driven
 * from one of the main clock signals or PLL outputs. This auxiliary clock can
 * be used to drive ADCs, PWM, DACs and other peripherals. Each of them might
 * have individual requirements. Please refer to the specific peripheral sections
 * in the device data sheet to learn how to configure the auxiliary clock.
 *
 * ************************************************************************************************/

 inline volatile uint16_t init_AUXCLK_500MHz(void)
 {
    volatile uint16_t fres = 0;
    volatile AUXOSC_CONFIG_t aux_clock_config;
    
    aux_clock_config.FRCSEL = PLLDIV_ACLKCON_FRCSEL_FRC;
    aux_clock_config.AVCODIV = APLLDIV_AVCODIV_FVCO_DIV_BY_1;
    aux_clock_config.N1 = ACLKCON_APLLDIV_N1_1;
    aux_clock_config.M = APLLFBD_APLLFBDIV_M_125;
    aux_clock_config.N2 = APLLDIV_POST2DIV_N2N3_2;
    aux_clock_config.N3 = APLLDIV_POST2DIV_N2N3_1;
    aux_clock_config.APLLEN = ACLKCON_APLLEN_ENABLED;
     
    fres = init_AUXCLK(aux_clock_config);
    
    return(fres);
 }


