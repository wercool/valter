/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*
    Title: FPGA / CAP synchronization routines
*/

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <board.h>
#include <utility/trace.h>
#include "board_fpga_init.h"

//------------------------------------------------------------------------------
// Prototype of internal functions
//------------------------------------------------------------------------------
unsigned int BOARD_FPGA_init_mpblock_revC(void);
unsigned int BOARD_FPGA_fpga_synchro_revC(void);
void         BOARD_FPGA_init_mpbs_revC(volatile unsigned int* mpbs,char pun,char lp,char sup);
unsigned int BOARD_FPGA_init_mpblock_revB(void);
unsigned int BOARD_FPGA_fpga_synchro_revB(void);
void         BOARD_FPGA_init_mpbs_revB(volatile unsigned int* mpbs,char pun,char lp,char sup);

//------------------------------------------------------------------------------
/// FPGA access initialization
/// Returns : programmed clk out delay if success
///           error code with pattern CACAB0Fx if fail
//------------------------------------------------------------------------------
unsigned int BOARD_FPGA_InitMPBlock(void)
{
    int i;

    // if POR, wait until FPGA is ready
    if (!(RSTC_SR_REG&0x00000600)) {

        for(i=0;i<POWER_ON_TEMPO;++i) {};
    }

    if(CAP9_CHECK_REVISION_REG == REV_B_CHECK_VALUE) {

        // CAP9 revB detected
        TRACE_INFO("Init MPBlock rev B\n\r");
        return(BOARD_FPGA_init_mpblock_revB());
    }
    else {

        if(EXTENDED_CHIP_ID_REG) {

            // not a dev chip
            TRACE_INFO("No init MPBlock : it is not a dev chip\n\r");
            return(NOT_A_DEV_CHIP_ERROR);
        }
        else {

            // CAP9 revC detected
            TRACE_INFO("Init MPBlock rev C\n\r");
            return(BOARD_FPGA_init_mpblock_revC());
        }
    }
}

//------------------------------------------------------------------------------
/// FPGA access initialization - CAP9 RevC
//------------------------------------------------------------------------------
unsigned int BOARD_FPGA_init_mpblock_revC(void)
{
    unsigned int ret_value;

    // Enable clk
    *((unsigned int*)0xFFFFFC00)|=0x4;
    BOARD_FPGA_init_mpbs_revC(MPBS0,PULLUP_ON,MPIO_LP,MPIO_SUPPLY);

    TRACE_INFO("FPGA IF synchro loop...\n\r");

#ifdef __FPGA_IF_TYPE_DIV3
    FPGA_IF_TYPE_REG = 0x01;
#endif

    // DELAY LINE SETUP
    DELAY_CTRL_REG = 0x03;              // write in DELAY_CTRL register enable_delay = 1 and update_delay = 1
    while (!(DELAY_STATUS_REG & 0x1)){} // wait master_is_locked = 1

    // FPGA IF synchronization loop
    //ret_value = fpga_synchro_old();
    ret_value = BOARD_FPGA_fpga_synchro_revC();
    if ((ret_value&0xFFFF0000) == 0xCACA0000) {

        return(ret_value);
    }

    TRACE_INFO("Synchro done\n\r");

    // Switch to functionnal mode
    //outi(0x30DC00, 0xDC00);
    INIT_ARG_REG = 0x4C727354; // INIT_ARG = INIT_CMD_START_INIT
    INIT_ARG_REG = 0x4C6F634B; // INIT_ARG = INIT_CMD_START_INIT
    INIT_CMD_REG = 0x03;       // INIT_CMD = INIT_CMD_CYCLE_IDX_LOCKED

    // Return the programmed clock_out_delay value
    return(ret_value);
}

//------------------------------------------------------------------------------
/// FPGA access initialization - CAP9 RevC
//------------------------------------------------------------------------------
unsigned int BOARD_FPGA_fpga_synchro_revC(void)
{
    unsigned char clk_out_delay_mean;
    unsigned char clk_out_delay = 0;
    unsigned int time = 0;

    INIT_ARG_REG = 0x5C00;
    INIT_CMD_REG = 0x01;       // INIT_CMD = INIT_CMD_START_INIT

    //Search first match value
    time = 0;
    while( ((CYCLE_IDX_RESP_R_LSB_REG != CYCLE_IDX_RESP_F_LSB_REG) ||
            (CYCLE_IDX_RESP_R_MSB_REG != ~CYCLE_IDX_RESP_F_MSB_REG))
           && (time++ < FPGA_SYNCHRO_TIMEOUT1)) {

        DELAY_CTRL_REG=(clk_out_delay++ << 8)|0x1;
    }
    if(time > FPGA_SYNCHRO_TIMEOUT1) {

        return(FPGA_SYNCHRO_ERROR1);
    }
    clk_out_delay_mean = clk_out_delay;

    //Search last match value
    DELAY_CTRL_REG=(clk_out_delay++ << 8)|0x1;

    while( ((CYCLE_IDX_RESP_R_LSB_REG == CYCLE_IDX_RESP_F_LSB_REG) &&
            (CYCLE_IDX_RESP_R_MSB_REG == ~CYCLE_IDX_RESP_F_MSB_REG))
            && (time++ < FPGA_SYNCHRO_TIMEOUT2) ) {

        DELAY_CTRL_REG=(clk_out_delay++ << 8)|0x1;
    }
    if(time>FPGA_SYNCHRO_TIMEOUT2) {

        return(FPGA_SYNCHRO_ERROR2);
    }
    clk_out_delay_mean = (clk_out_delay_mean + clk_out_delay)/2;

    // Setup to middle match value
    DELAY_CTRL_REG = (clk_out_delay_mean << 8) | 0x1;

    // Cycle index select
    time = 0;
    while( (CYCLE_IDX_RESP_R_LSB_REG != 0x0A7014E0) && (time++ < FPGA_SYNCHRO_TIMEOUT3) ) {

        INIT_ARG_REG = 0x02; // INIT_ARG = 2
        INIT_CMD_REG = 0x02; // INIT_CMD = INIT_CMD_ADD_CYCLE
    }
    if(time>FPGA_SYNCHRO_TIMEOUT3) {

        return(FPGA_SYNCHRO_ERROR3);
    }

    // Return the programmed clock_out_delay value
    return ((unsigned int)clk_out_delay_mean);
}

//------------------------------------------------------------------------------
/// Enable CAP to FPGA clock, and configure pads - CAP9 RevC
//------------------------------------------------------------------------------
void BOARD_FPGA_init_mpbs_revC(volatile unsigned int* mpbs,char pun,char lp,char sup)
{
    unsigned int value;

    if (mpbs!=MPBS0 && mpbs!=MPBS1 && mpbs!=MPBS2 && mpbs!=MPBS3)
        return;
    value = *mpbs | MPBS_ENABLE_BIT;

    if (pun==PULLUP_ON) value&= ~(MPIOB_PUN_BIT|MPIOA_PUN_BIT);
    else if (pun==PULLUP_OFF) value|= (MPIOB_PUN_BIT|MPIOA_PUN_BIT);

    if (lp==LOWPOWER_OFF) value|= (MPIOB_LP_BIT|MPIOA_LP_BIT);
    else if(lp==LOWPOWER_ON)value&= ~(MPIOB_LP_BIT|MPIOA_LP_BIT);

    if (sup==SUPPLY_18) value|= (MPIOB_SUP_BIT|MPIOA_SUP_BIT);
    else if(sup==SUPPLY_33)value&= ~(MPIOB_SUP_BIT|MPIOA_SUP_BIT);

    *mpbs = value;
}

//------------------------------------------------------------------------------
/// FPGA access initialization - CAP9 RevB
//------------------------------------------------------------------------------
unsigned int BOARD_FPGA_init_mpblock_revB(void)
{
    unsigned int ret_value;

    // Enable clk
    BOARD_FPGA_init_mpbs_revB(MPBS0,PULLUP_ON,MPIO_LP,MPIO_SUPPLY);

    TRACE_INFO("FPGA IF synchro loop...\n\r");

#ifdef __FPGA_IF_TYPE_DIV3
    FPGA_IF_TYPE_REG = 0x01;
 #endif

    // DELAY LINE SETUP
    DELAY_CTRL_REG = 0x03;              // write in DELAY_CTRL register enable_delay = 1 and update_delay = 1
    while (!(DELAY_STATUS_REG & 0x1)){} // wait master_is_locked = 1

    // FPGA IF synchronization loop
    //ret_value = fpga_synchro_old();
    ret_value = BOARD_FPGA_fpga_synchro_revB();
    if ((ret_value&0xFFFF0000) == 0xCACA0000) {

        return(ret_value);
    }

    TRACE_INFO("Synchro done\n\r");

    // Switch to functionnal mode
    INIT_ARG_REG = 0x4C727354; // INIT_ARG = INIT_CMD_START_INIT
    INIT_ARG_REG = 0x4C6F634B; // INIT_ARG = INIT_CMD_START_INIT
    INIT_CMD_REG = 0x03;       // INIT_CMD = INIT_CMD_CYCLE_IDX_LOCKED

    // Return the programmed clock_out_delay value
    return(ret_value);
}

//------------------------------------------------------------------------------
/// CAP9 / FPGA synchronization loop - CAP9 RevB
//------------------------------------------------------------------------------
unsigned int BOARD_FPGA_fpga_synchro_revB(void)
{
    unsigned char clk_out_delay_mean;
    unsigned char clk_out_delay = 0;
    unsigned int time = 0;

    INIT_ARG_REG = 0x5C00;
    INIT_CMD_REG = 0x01;       // INIT_CMD = INIT_CMD_START_INIT

    //Search first match value
    time = 0;
    while( ((CYCLE_IDX_RESP_R_LSB_REG != CYCLE_IDX_RESP_F_LSB_REG) ||
            (CYCLE_IDX_RESP_R_MSB_REG != ~CYCLE_IDX_RESP_F_MSB_REG))
           && (time++<FPGA_SYNCHRO_TIMEOUT1)) {

        DELAY_CTRL_REG=(clk_out_delay++ << 8)|0x1;
    }
    if(time > FPGA_SYNCHRO_TIMEOUT1) {

        return(FPGA_SYNCHRO_ERROR1);
    }
    clk_out_delay_mean = clk_out_delay;

    //Search last match value
    DELAY_CTRL_REG=(clk_out_delay++ << 8)|0x1;
    while( ((CYCLE_IDX_RESP_R_LSB_REG == CYCLE_IDX_RESP_F_LSB_REG) &&
            (CYCLE_IDX_RESP_R_MSB_REG == ~CYCLE_IDX_RESP_F_MSB_REG))
           && (time++<FPGA_SYNCHRO_TIMEOUT2)) {

        DELAY_CTRL_REG=(clk_out_delay++ << 8)|0x1;
    }
    if(time>FPGA_SYNCHRO_TIMEOUT2) {

        return(FPGA_SYNCHRO_ERROR2);
    }
    clk_out_delay_mean = (clk_out_delay_mean + clk_out_delay)/2;

    // Setup to middle match value
    DELAY_CTRL_REG = (clk_out_delay_mean << 8) | 0x1;

    // Cycle index select
    time = 0;
    while( (CYCLE_IDX_RESP_R_LSB_REG != 0x0A7014E0) && (time++ < FPGA_SYNCHRO_TIMEOUT3)) {

        INIT_ARG_REG = 0x02; // INIT_ARG = 2
        INIT_CMD_REG = 0x02; // INIT_CMD = INIT_CMD_ADD_CYCLE
    }
    if(time>FPGA_SYNCHRO_TIMEOUT3) {

        return(FPGA_SYNCHRO_ERROR3);
    }

    // Return the programmed clock_out_delay value
    return ((unsigned int)clk_out_delay_mean);
}

//------------------------------------------------------------------------------
/// Enable CAP to FPGA clock, and configure pads - CAP9 RevB
//------------------------------------------------------------------------------
void BOARD_FPGA_init_mpbs_revB(volatile unsigned int* mpbs,char pun,char lp,char sup)
{
    unsigned int value;

    if (mpbs!=MPBS0 && mpbs!=MPBS1 && mpbs!=MPBS2 && mpbs!=MPBS3)
        return;
    value = *mpbs | MPBS_ENABLE_BIT;

    if (pun==PULLUP_ON) value&= ~(MPIOB_PUN_BIT|MPIOA_PUN_BIT);
    else if (pun==PULLUP_OFF) value|= (MPIOB_PUN_BIT|MPIOA_PUN_BIT);

    if (lp==LOWPOWER_ON) value|= (MPIOB_LP_BIT|MPIOA_LP_BIT);
    else if(lp==LOWPOWER_OFF)value&= ~(MPIOB_LP_BIT|MPIOA_LP_BIT);

    if (sup==SUPPLY_33) value|= (MPIOB_SUP_BIT|MPIOA_SUP_BIT);
    else if(sup==SUPPLY_18)value&= ~(MPIOB_SUP_BIT|MPIOA_SUP_BIT);

    *mpbs = value;
}

