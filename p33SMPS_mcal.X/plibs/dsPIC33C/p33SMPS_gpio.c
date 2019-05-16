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
/* @@p33SMPS_gpio.c
 * ***************************************************************************
 *
 * File:   p33SMPS_gpio.c
 * Author: M91406
 *
 * Created on October 27, 2017, 11:24 AM
 * ***************************************************************************/


#include "p33SMPS_gpio.h"

uint16_t gpio_reset(void) {

    /* Reset all device pins to digital function */
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    /* Reset all weak pull-up settings */
    #ifdef CNPUA
        CNPUA = 0x0000;
    #endif
    #ifdef CNPUB
        CNPUB = 0x0000;
    #endif
    #ifdef CNPUC
        CNPUC = 0x0000;
    #endif
    #ifdef CNPUD
        CNPUD = 0x0000;
    #endif
    
    /* Reset all weak pull-down settings */
    #ifdef CNPDA
        CNPDA = 0x0000;
    #endif
    #ifdef CNPDB
        CNPDB = 0x0000;
    #endif
    #ifdef CNPDC
        CNPDC = 0x0000;
    #endif
    #ifdef CNPDD
        CNPDD = 0x0000;
    #endif

    /* Reset all open-drain settings */
    #ifdef ODCA
        ODCA = 0x0000;
    #endif
    #ifdef ODCB
        ODCB = 0x0000;
    #endif
    #ifdef ODCC
        ODCC = 0x0000;
    #endif
    #ifdef ODCD
        ODCD = 0x0000;
    #endif

    /* Reset pin direction settings to input */
    #ifdef TRISA        
        TRISA = 0xFFFF;
    #endif
    #ifdef TRISB
        TRISB = 0xFFFF;
    #endif
    #ifdef TRISC        
        TRISC = 0xFFFF;
    #endif
    #ifdef TRISD        
        TRISD = 0xFFFF;
    #endif
        
    /* Reset port latch registers to high */
    #ifdef LATA
        LATA = 0xFFFF;
    #endif
    #ifdef LATB
        LATB = 0xFFFF;
    #endif
    #ifdef LATC
        LATC = 0xFFFF;
    #endif
    #ifdef LATD
        LATD = 0xFFFF;
    #endif
        
    return(1);
}
