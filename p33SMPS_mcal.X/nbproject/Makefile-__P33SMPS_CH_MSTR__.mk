#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-__P33SMPS_CH_MSTR__.mk)" "nbproject/Makefile-local-__P33SMPS_CH_MSTR__.mk"
include nbproject/Makefile-local-__P33SMPS_CH_MSTR__.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=__P33SMPS_CH_MSTR__
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=a
DEBUGGABLE_SUFFIX=a
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=a
DEBUGGABLE_SUFFIX=a
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=plibs/dsPIC33C/p33SMPS_dsp.c plibs/dsPIC33C/p33SMPS_hsadc.c plibs/dsPIC33C/p33SMPS_hspwm_c.c plibs/dsPIC33C/p33SMPS_oscillator.c plibs/dsPIC33C/p33SMPS_timer.c plibs/dsPIC33C/p33SMPS_pmd.c plibs/dsPIC33C/p33SMPS_pps.c plibs/dsPIC33C/p33SMPS_irq.c plibs/dsPIC33C/p33SMPS_gpio.c plibs/dsPIC33C/p33SMPS_mailboxes.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o
POSSIBLE_DEPFILES=${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o.d ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o

# Source Files
SOURCEFILES=plibs/dsPIC33C/p33SMPS_dsp.c plibs/dsPIC33C/p33SMPS_hsadc.c plibs/dsPIC33C/p33SMPS_hspwm_c.c plibs/dsPIC33C/p33SMPS_oscillator.c plibs/dsPIC33C/p33SMPS_timer.c plibs/dsPIC33C/p33SMPS_pmd.c plibs/dsPIC33C/p33SMPS_pps.c plibs/dsPIC33C/p33SMPS_irq.c plibs/dsPIC33C/p33SMPS_gpio.c plibs/dsPIC33C/p33SMPS_mailboxes.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-__P33SMPS_CH_MSTR__.mk dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33CH512MP508
MP_LINKER_FILE_OPTION=,--script=p33CH512MP508.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o: plibs/dsPIC33C/p33SMPS_dsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_dsp.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o: plibs/dsPIC33C/p33SMPS_hsadc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_hsadc.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o: plibs/dsPIC33C/p33SMPS_hspwm_c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_hspwm_c.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o: plibs/dsPIC33C/p33SMPS_oscillator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_oscillator.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o: plibs/dsPIC33C/p33SMPS_timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_timer.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o: plibs/dsPIC33C/p33SMPS_pmd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_pmd.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o: plibs/dsPIC33C/p33SMPS_pps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_pps.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o: plibs/dsPIC33C/p33SMPS_irq.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_irq.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o: plibs/dsPIC33C/p33SMPS_gpio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_gpio.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o: plibs/dsPIC33C/p33SMPS_mailboxes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_mailboxes.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o: plibs/dsPIC33C/p33SMPS_dsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_dsp.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_dsp.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o: plibs/dsPIC33C/p33SMPS_hsadc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_hsadc.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hsadc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o: plibs/dsPIC33C/p33SMPS_hspwm_c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_hspwm_c.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_hspwm_c.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o: plibs/dsPIC33C/p33SMPS_oscillator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_oscillator.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_oscillator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o: plibs/dsPIC33C/p33SMPS_timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_timer.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_timer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o: plibs/dsPIC33C/p33SMPS_pmd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_pmd.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pmd.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o: plibs/dsPIC33C/p33SMPS_pps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_pps.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_pps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o: plibs/dsPIC33C/p33SMPS_irq.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_irq.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_irq.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o: plibs/dsPIC33C/p33SMPS_gpio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_gpio.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_gpio.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o: plibs/dsPIC33C/p33SMPS_mailboxes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/plibs/dsPIC33C" 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o.d 
	@${RM} ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  plibs/dsPIC33C/p33SMPS_mailboxes.c  -o ${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ___P33SMPS_CH_MSTR__=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -std=gnu99 -finline-functions -O3 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/plibs/dsPIC33C/p33SMPS_mailboxes.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: archive
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX} 
	${MP_AR} $(MP_EXTRA_AR_PRE)  -omf=elf -r dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX} 
	${MP_AR} $(MP_EXTRA_AR_PRE)  -omf=elf -r dist/${CND_CONF}/${IMAGE_TYPE}/p33SMPS_mcal.X.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/__P33SMPS_CH_MSTR__
	${RM} -r dist/__P33SMPS_CH_MSTR__

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
