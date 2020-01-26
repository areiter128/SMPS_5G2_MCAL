/*LICENSE *****************************************************************************************
 *
 * Software License Agreement
 *
 * Copyright (R) 2017 Microchip Technology Inc.  All rights reserved. Microchip licenses to you the
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

// Include Header Files
#include "p33SMPS_dsp.h"

/*!p33SMPS_dsp.c
 * ************************************************************************************************
 * Summary:
 * Driver file for the dsPIC33E and dsPIC33C DSP Configuration SFRs
 *
 * Description:
 * The DSP engine of dsPIC33 offers several options to optimize data processing for integer 
 * and fixed-point algorithmic, which can be defined and enabled using this library.
 * ***********************************************************************************************/


/*!smpsDSP_Initialize()
 * *****************************************************************************************************
 * Summary:
 * Initializes the DSP engine in accordance to user settings 
 *
 * Parameters: 
 *  16-bit wide DSP configuration data structure CORCON_t
 *
 * Description:
 * This routine writes a DSP user-configuration into the core configuration register and verifies
 * the data has been written correctly.
 * 
 * *****************************************************************************************************/

volatile uint16_t smpsDSP_Initialize(volatile CORCON_t dsp_cfg)
{
	volatile uint16_t fres = 0;
	
    CORCON = dsp_cfg.value;
	if(CORCON == (dsp_cfg.value & REG_CORCON_VALID_DATA_WRITE_MSK)) 
        fres = 1;
	
    return(fres);

} 


/*!smpsDSP_GetConfig()
 * *****************************************************************************************************
 * Summary:
 * Reads the DSP engine configuration 
 *
 * Parameters: 
 * (none) 
 *
 * Returns:
 * 16-bit wide DSP configuration data structure REGBLK_CORCON_t
 *
 * Description:
 * This routine writes a DSP user-configuration into the core configuration register and verifies
 * the data has been written correctly.
 * 
 * *****************************************************************************************************/

volatile CORCON_t smpsDSP_GetConfig(void)
{
	volatile CORCON_t dsp_cfg;
	
    dsp_cfg.value = (volatile uint16_t)(CORCON & REG_CORCON_VALID_DATA_READ_MSK);
	
    return(dsp_cfg);

} 

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// EOF
