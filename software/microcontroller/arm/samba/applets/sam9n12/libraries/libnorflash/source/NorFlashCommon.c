/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011, Atmel Corporation
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

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <board.h>
#include "NorFlashCommon.h"

#include <string.h>

/*----------------------------------------------------------------------------
 *        Macro
 *----------------------------------------------------------------------------*/

#define WRITE_COMMAND8(commandAddress, command) \
    {*(volatile uint8_t *)(commandAddress) = (uint8_t) command;}
#define WRITE_COMMAND16(commandAddress, command) \
    {*(volatile uint16_t *)(commandAddress) = (uint16_t) command;}
#define WRITE_COMMAND32(commandAddress, command) \
    {*(volatile uint32_t *)(commandAddress) = (uint32_t) command;}
    
#define WRITE_DATA8(dataAddress, data) \
    {(*((volatile uint8_t *) dataAddress)) = (uint8_t) data;}
#define WRITE_DATA16(dataAddress, data) \
    {(*((volatile uint16_t *) dataAddress)) = (uint16_t) data;}
#define WRITE_DATA32(dataAddress, data) \
    {(*((volatile uint32_t *) dataAddress)) = (uint32_t) data;}
    
#define READ_DATA8(dataAddress) \
    (*((volatile uint8_t *) dataAddress))
#define READ_DATA16(dataAddress) \
    (*((volatile uint16_t *) dataAddress))
#define READ_DATA32(dataAddress) \
    (*((volatile uint32_t *) dataAddress))
    
/*----------------------------------------------------------------------------
 *        Export functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Write a command to address.
 * \param busWidth Bus width in device.
 * \param commandAddress Command address offset.
 * \param command Command to be send.
 */
uint8_t WriteCommand(
    uint8_t busWidth, 
    uint32_t commandAddress, 
    uint32_t command)
{
    if (busWidth == 8 ){
        WRITE_COMMAND8(commandAddress, command);
    }
    else if( busWidth == 16 ){
        WRITE_COMMAND16(commandAddress, command);
    }
    else if(busWidth == 32 ){
        WRITE_COMMAND32(commandAddress, command);
    }
    return 0;
}


/**
 * \brief Reads data from the NorFlash chip into the provided buffer.
 * \param busWidth  Bus width in device.
 * \param address  Address of data.
 * \param buffer  Buffer where the data will be stored.
 */
void ReadRawData(
    uint8_t busWidth,
    uint32_t  address,
    uint8_t *buffer)
{
    if (busWidth == 8 ){    
            *buffer = READ_DATA8(address);
    }
    else if( busWidth == 16 ){
        
        uint16_t *buffer16 = (uint16_t *) buffer;
        *buffer16 = READ_DATA16(address);
    }
    else if(busWidth == 32 ){
        uint32_t *buffer32 = (uint32_t *) buffer;
        *buffer32 = READ_DATA32(address);
    }
}

/**
 * \brief Writes data to the NorFlash chip from the provided buffer.
 * \param busWidth  Bus width in device.
 * \param address  Address of data.
 * \param buffer  Buffer where the data will be stored.
 */
void WriteRawData(
    uint8_t busWidth,
    uint32_t  address,
    uint8_t *buffer)

{
    if (busWidth == 8 ){
        WRITE_DATA8(address, *buffer);
    }
    else if( busWidth == 16 ){
        uint16_t *buffer16 = (uint16_t *) buffer;
        WRITE_DATA16(address, *buffer16);
    }
    else if(busWidth == 32 ){
        uint32_t *buffer32 = (uint32_t *) buffer;
        WRITE_DATA32(address, *buffer32);
    }
}

