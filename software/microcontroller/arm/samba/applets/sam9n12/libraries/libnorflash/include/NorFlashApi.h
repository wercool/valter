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

//------------------------------------------------------------------------------
/// \unit
///
/// !!!Purpose
/// 
/// The API layer consists of several functions that allow user to do 
/// operations with flash in a unified way. As a result, future device changes 
/// will not necessarily lead to the code changes in the application environments.
/// In this %norflash library we support AMD and INTEL command set.
/// The API layar code accesses the Flash memory by calling the low-level code, 
/// so users do not have to concern themselves with the details of the special 
/// command sequences.
/// 
/// !!!Usage
/// 
/// !!!Usage
/// -# Flash program using NORFLASH_WriteData().
///    - The Program command is used to modify the data stored at the 
///      specified device address. 
/// -# erase a block within the flash using NORFLASH_EraseSector().
///    - Flash erase is performed on a block basis. An entire block is 
///      erased each time an erase command sequence is given. 
/// -# erase whole blocks within the flash using NORFLASH_EraseChip().
/// -# NORFLASH_Reset() function can be issued, between Bus Write cycles 
///    before the start of a program or erase operation, to return the 
///    device to read mode.
/// -# NORFLASH_ReadDeviceID() is used to retrieve information 
///    about the Flash Device type.
/// -# NORFLASH_ReadManufactoryID() is used to retrieve information 
///    about the Flash Device Manufactory ID.
//------------------------------------------------------------------------------

#ifndef NORFLASHAPI_H
#define NORFLASHAPI_H

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "NorFlashCFI.h"

/*----------------------------------------------------------------------------
 *        Type
 *----------------------------------------------------------------------------*/

/** Reset or exit CFI query mode function. */
typedef void (*fReset) (struct NorFlashInfo *, uint32_t );
/** Write buffer to pNorFlash function.*/
typedef uint8_t (*fWriteData)(struct NorFlashInfo *, uint32_t , uint8_t *, uint32_t );
/** Read manufactory function.*/
typedef uint32_t (*fReadManufactoryID)(struct NorFlashInfo *);
/** Read device ID code function.*/
typedef uint32_t (*fReadDeviceID)(struct NorFlashInfo *);
/** Full erase chip function.*/
typedef uint8_t (*fEraseChip) (struct NorFlashInfo *);
/** Erase single sector function.*/
typedef uint8_t (*fEraseSector)(struct NorFlashInfo *, uint32_t );


struct NorFlashOperations {
    /** Reset or exit CFI query mode function.*/
    void (*_fReset)(struct NorFlashInfo *pNorFlashInfo, uint32_t address);
    /** Write buffer to norflash function.*/
    uint8_t (*_fWriteData)(struct NorFlashInfo *pNorFlashInfo,
       uint32_t address,
       uint8_t *buffer,
       uint32_t size);
    /** Read manufactory function.*/
    uint32_t  (*_fReadManufactoryID)(struct NorFlashInfo *pNorFlashInfo);
    /** Read device ID code function.*/
    uint32_t (*_fReadDeviceID)(struct NorFlashInfo *pNorFlashInfo);
    /** Full erase chip function.*/
    uint8_t (*_fEraseChip) (struct NorFlashInfo *pNorFlashInfo);
    /** Erase single sector function.*/
    uint8_t (*_fEraseSector)(struct NorFlashInfo *pNorFlashInfo, uint32_t address);
};

/*----------------------------------------------------------------------------
 *        Export functions
 *----------------------------------------------------------------------------*/

extern void NORFLASH_Reset(struct NorFlash *norFlash, uint32_t address);

extern uint32_t NORFLASH_ReadManufactoryID(struct  NorFlash *norFlash);

extern uint32_t NORFLASH_ReadDeviceID(struct  NorFlash *norFlash);

extern uint8_t NORFLASH_EraseSector(
    struct NorFlash *norFlash, 
    uint32_t sectorAddr);

extern uint8_t NORFLASH_EraseChip(
    struct NorFlash *norFlash);

extern uint8_t NORFLASH_WriteData(
    struct NorFlash *norFlash,
    uint32_t address,
    uint8_t *buffer,
    uint32_t size);

extern uint8_t NORFLASH_ReadData(
    struct NorFlash *norFlash,
    uint32_t address,
    uint8_t *buffer,
    uint32_t size);


#endif //#ifndef NORFLASHAPI_H

