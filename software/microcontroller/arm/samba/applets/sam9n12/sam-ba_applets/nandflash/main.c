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

/** \page nandflash_applet Nand Flash Applet
 * Nand Flash applet is a small piece of software running on the target board, embedding the
 * NAND flash read, write, and erase algorithms etc. \n
 *
 * SAM-BA host controls commands and data transfering, it loads Nand flash applet and
 * executes it to implement NAND flash memory operation.
 * An applet consists of:
 * <ul>
 * <li>A mailbox which can share information between the applet and SAM-BA host.</li>
 * <li>Implement APPLET_CMD_INIT command to initializes NAND device, PMECC and reports memory size,
 * buffer address, size of buffer and current pmecc parameter header configuration through the mailbox.</li>
 * Note: Tries to detect NAND Flash device connected to EBI CS3, with data lines connected to D0-D7, 
 * then on NAND Flash connected to D16-D23. 
 * <li>A NAND flash programming algorithm works for APPLET_CMD_PMECC_HEADER command, it configure pemcc parameter 
 * which is made of 52 times the same 32-bit word.
 * <li>A NAND flash programming algorithm works for APPLET_CMD_WRITE command, it erases and programs
 * NAND flash data in buffer at the address which specified in mailbox.</li>
 * <li>Erase all algorithm is necessary to erasing entire flash memory. </li>
 * </ul>
 *
 * It reports command status and written/read size (for write/read algorithm) in mailbox to
 * SAM-BA host while the command is achieved.
 *
 *
*/
/*@{*/
/*@}*/


/**
 * \file
 *
 * Implementation of Nand flash applet for SAM-BA 2.10 (sam9xx5 only).
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "../common/applet.h"
#include <board.h>
#include <libnandflash.h>
#include <libpmecc.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define ECCHEADER_USEPMECC   1
#define ECCHEADER_SECTORS    2
#define ECCHEADER_SPARE      3
#define ECCHEADER_ECCBIT     4
#define ECCHEADER_SECTORSIZE 5
#define ECCHEADER_ECCOFFSET  6
#define ECCHEADER_ALL        7

#define ECC_ENABLE_SOFT      0
#define ECC_DISABLE_SOFT     1
#define ECC_ENABLE_PMECC     2
#define ECC_DISABLE_PMECC    3
#define ECC_ENABLE_INTEECC   4

/*----------------------------------------------------------------------------
 *        Local structures
 *----------------------------------------------------------------------------*/

/** \brief Structure for storing parameters for each command that can be
 * performed by the applet.*/

struct _Mailbox {

    /** Command send to the monitor to be executed. */
    unsigned int command;
    /** Returned status, updated at the end of the monitor execution. */
    unsigned int status;

    /** Input Arguments in the argument area */
    union {

        /** Input arguments for the Init command. */
        struct {

            /** Communication link used. */
            unsigned int comType;

            /** Trace level. */
            unsigned int traceLevel;

        } inputInit;

        /** Output arguments for the Init command. */
        struct {

            /** Memory size. */
            unsigned int memorySize;
            /** Buffer address. */
            unsigned int bufferAddress;
            /** Buffer size. */
            unsigned int bufferSize;
            /** Current pmecc parameter header configuration. */
            unsigned int pmeccParamHeader;

        } outputInit;

        /** Input arguments for the switchEccMode command. */
        struct {
            /** Ecc mode to be switched*/
            unsigned int eccMode;

        } inputEccMode;

        /** Output arguments for the switchEccMode command. */
        /* NONE */

        /** Input arguments for the trimffs command. */
        struct {
            /** Ecc mode to be switched*/
            unsigned int trim;

        } inputTrimMode;

        /** Output arguments for the TrimFfs command. */
        /* NONE */

        /** Input arguments for the pmeccHeader command. */
        struct {
            /** pmeccParam index*/
            unsigned int pmeccParamIndex;
            /** Pmecc value. */
            unsigned int pmeccParamValue;

        } inputPmeccHeader;

        /** Output arguments for the pmeccHeader command. */
        struct {
           /** Current pmecc parameter header configuration. */
            unsigned int pmeccParamHeader;
        } outputPmeccHeader;

        /** Input arguments for the Write command. */
        struct {

            /** Buffer address. */
            unsigned int bufferAddr;
            /** Buffer size. */
            unsigned int bufferSize;
            /** Memory offset. */
            unsigned int memoryOffset;

        } inputWrite;

        /** Output arguments for the Write command. */
        struct {

            /** Bytes written. */
            unsigned int bytesWritten;

        } outputWrite;

        /** Input arguments for the Read command. */
        struct {

            /** Buffer address. */
            unsigned int bufferAddr;
            /** Buffer size. */
            unsigned int bufferSize;
            /** Memory offset. */
            unsigned int memoryOffset;

        } inputRead;

        /** Output arguments for the Read command. */
        struct {

            /** Bytes read. */
            unsigned int bytesRead;

        } outputRead;

        /** Input arguments for the Full Erase command. */
        struct {

            /** Type of Erase to perform */
            unsigned int eraseType;

        } inputFullErase;

        /** Input arguments for the Batch Erase command. */
        struct {
            /** Type of Erase to perform */
            unsigned int eraseType;
            /** Batch number. */
            unsigned int batch;
        } inputBatchErase;

         /** Input arguments for the Block Erase command. */
        struct {

            /** Memory start offset to be erased. */
            unsigned int memoryOffsetStart;
            /** Memory end offset to be erased. */
            unsigned int memoryOffsetEnd;
        } inputBlocksErase;
        
        /** Output arguments for the Full Erase command. */
        /* NONE */

        /** Output arguments for the Batch Erase command. */
        struct {
            /** next eraseing batch. */
            unsigned int nextBatch;
        } outputBatchErase;
        /** Input arguments for the List Bad Blocks command. */
        /* NONE  */

        /** Output arguments for the List Bad Blocks command. */
        struct {

            /** Number of bad blocks found */
            unsigned int nbBadBlocks;
            /** Address of the buffer containing bad block list */
            unsigned int bufferAddress;

        } outputListBadBlocks;

        /** Input arguments for the Tag Block command. */
        struct {

            /** ID of the block to tag */
            unsigned int blockId;
            /** Data to be written in the bad block marker word */
            unsigned int tag;

        } inputTagBlock;

        /** Output arguments for the Tag Block command. */
        /* NONE */

    } argument;
};

/* Here is the structure to insert at beginning of NAND flash. That will inform RomCode about how to configure NAND reading & PMECC.*/
/* This header may have been written by the flash programmer at the beginning of Nandflash
   and contains Flash parameters required to configure correctly the PMECC.
   It has been written .
   It must be duplicated 49 times and we get the value for each field by
   applying a "majority test" algorithm.
   In case it is not present, we will try to detect flash structure with ONFI paramaters. */
typedef struct _nfParamHeader_t {
    unsigned int usePmecc          : 1;
    unsigned int nbSectorPerPage   : 3;   // 0, 1,2,3
    unsigned int spareSize         : 9;   // size of spare zone in bytes
    unsigned int eccBitReq         : 3;   // 0,1,2,3,4
    unsigned int sectorSize        : 2;   // 0 for 512 bytes, 1 for 1024 bytes per sector, other val for future use
    unsigned int eccOffset         : 9;   // offset of the first ecc byte in spare zone
    unsigned int reserved          : 1;
    unsigned int key               : 4;

} nfParamHeader_t;

/*----------------------------------------------------------------------------
 *         Global variables
 *----------------------------------------------------------------------------*/
/** End of program space (code + data). */
extern unsigned int _end;


/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/
/** Nandflash memory size. */
static unsigned int memSize;
/** Number of blocks in nandflash. */
static unsigned int numBlocks;
/** Size of one block in the nandflash, in bytes. */
static unsigned int blockSize;
/** Size of the data area of in the nandflash, in bytes.*/
static unsigned int pageSize = 0;
/** Size of the spare area of in the nandflash, in bytes.*/
static unsigned int spareSize;
/** Number of page per block */
static unsigned int numPagesPerBlock;
/** Nandflash bus width */
static unsigned char nfBusWidth = 8;

/** Pins used to access to nandflash. */
static const Pin pPinsNf[] = {PINS_NANDFLASH};
/** Nandflash device structure. */
static struct SkipBlockNandFlash skipBlockNf;
/** Global DMA driver instance for all DMA transfers in application. */
static sDmad dmad;
/** Global PMECC instance. */
static PmeccDescriptor pmeccDesc;
/** Global OnfiPageParameter instance. */
static OnfiPageParam OnfiPageParameter;
/** Address for transferring command bytes to the nandflash. */
static unsigned int cmdBytesAddr = BOARD_NF_COMMAND_ADDR;
/** Address for transferring address bytes to the nandflash. */
static unsigned int addrBytesAddr = BOARD_NF_ADDRESS_ADDR;
/** Address for transferring data bytes to the nandflash. */
static unsigned int dataBytesAddr = BOARD_NF_DATA_ADDR;
/** Nandflash chip enable pin. */
static const Pin nfCePin = BOARD_NF_CE_PIN;
/** Nandflash ready/busy pin. */
//static const Pin nfRbPin = BOARD_NF_RB_PIN;
static const Pin nfRbPin = {0, 0, 0, 0, 0};
/** Ecc type  0:ECC_NO, 1:ECC_SOFT, 2:ECC_PMECC 3:ECC_INTERNAL */
static unsigned int eccType = ECC_NO;
/** Ecc Correctability */
static unsigned char eccCorrectability = 2;
/** Sector size 0 for 512, 1 for 1024 */
static unsigned char sectorSize;
/** offset of the first ecc byte in spare zone */
static unsigned short eccOffset;
/** ParamHeader instance for NAND boot */
static nfParamHeader_t bootNfParamHeader[52];
/* ParamHeader */
nfParamHeader_t currentPmeccHeader, backupPmeccHeader;
/** Depending on DYN_TRACES, dwTraceLevel is a modifable runtime variable or a define */
uint32_t dwTraceLevel;
/*----------------------------------------------------------------------------
 *         Definiation
 *----------------------------------------------------------------------------*/
/** Polling or interrupt mode */
#define POLLING_MODE    1
#define ERASE_BATCH     8

/*----------------------------------------------------------------------------
 *         local functions
 *----------------------------------------------------------------------------*/
 /**
 * \brief Check is the pmecc parameter header is valid 
  \return 1 if the pmecc parameter header is valid; otherwise returns 0
 */
int isValidPmeccParam (nfParamHeader_t* pPmeccHeader)
{
   /* something for PMECC algo */
    unsigned int mm, nErrorCorrctionBits, nSectors, eccSizeByte, eccEndAddr,nbSectorsPerPage;
    /* Programmable Number of Sectors per page*/
    unsigned char sectorSizePerPage [4] = {1, 2, 4, 8};
    /* Number of ECC bits required */
    unsigned char eccBitReq2TT [5] = {2, 4, 8, 12, 24};
    if(pPmeccHeader->nbSectorPerPage > 3) return 1;
    if(pPmeccHeader->sectorSize > 1) return 1;
    if(pPmeccHeader->spareSize != spareSize) return 1;
    if(pPmeccHeader->eccBitReq > 4) return 1;
    if(pPmeccHeader->key != 0x0C) return 1;
    nbSectorsPerPage = (pPmeccHeader->sectorSize == 0) ? pageSize/512 : pageSize/1024;
    mm = (pPmeccHeader->sectorSize == 0)? 13: 14;
    nErrorCorrctionBits = eccBitReq2TT[pPmeccHeader->eccBitReq];
    nSectors = sectorSizePerPage[pPmeccHeader->nbSectorPerPage];
    if( nSectors > nbSectorsPerPage ) return 0;
    if ((( mm * nErrorCorrctionBits ) % 8 ) == 0)
    {
        eccSizeByte = ((mm * nErrorCorrctionBits ) / 8) * nSectors;
    }
    else 
    {
        eccSizeByte = (((mm * nErrorCorrctionBits ) / 8 ) + 1 ) * nSectors;
    }
    eccEndAddr = pPmeccHeader->eccOffset + eccSizeByte;
    if (pPmeccHeader->eccOffset > spareSize) {
        return 0;
    }                
    if ( eccEndAddr > spareSize) {
        return 0;
    }
    return 1;
}

/**
 * \brief Return numbers of sectors have been erased before.
 * \param pageSize  Size of page in byte.
 * \param sectorSize  Size of sector Size.
 * \param buf  Buffer containing the data area.
 * \return return numbers of sectors erased before.
*/
static unsigned int drop_ffs( unsigned char * buf)
{
    unsigned int l, i;
    for (i = blockSize - 1; i >= 0; i--) {
        if (buf[i] != 0xFF)
            break;
    }
    /* The resulting length must be aligned to the minimum ECC page size */
    l = i + 1;
    l = (l + pageSize - 1) / pageSize;
    
    return l;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief  Applet main entry. This function decodes received command and executes it.
 *
 * \param argc  always 1
 * \param argv  Address of the argument area..
 */
int main(int argc, char **argv)
{
    struct _Mailbox *pMailbox = (struct _Mailbox *) argv;
    unsigned int bufferSize, bufferAddr, memoryOffset, bytesToWrite;
    unsigned int bytesRead = 0;
    unsigned int nbBadBlocks = 0;
    unsigned int nbBlocks = 0;
    /* Temporary buffer used for non block aligned read / write  */
    unsigned int tempBufferAddr;
    unsigned short block, page, offset, i;
    /* Index in source buffer during buffer copy */
    unsigned int offsetInSourceBuff;
    /* Index in destination buffer during buffer copy */
    unsigned int offsetInTargetBuff;
    /* Errors returned by SkipNandFlash functions */
    unsigned char error = 0;
    /* Communication type with SAM-BA GUI. */
    unsigned char comType;
    /* current pmecc parameter header value */
    unsigned int currentPmeccHeaderValue;
    /* Index and value of pmecc command  */
    unsigned int nIndex, nValue;
    /* Number of ECC bits required */
    unsigned char eccBitReq2TT [5] = {2, 4, 8, 12, 24};
    /* Ecc mode to be swtich */
    unsigned int eccMode;
    unsigned int trimPage;
    /* Save communication link type */
    comType = pMailbox->argument.inputInit.comType;
    /* ---------------------------------------------------------- */
    /* INIT:                                                      */
    /* ---------------------------------------------------------- */
    if (pMailbox->command == APPLET_CMD_INIT) {
    
#if (DYN_TRACES == 1)
        dwTraceLevel = pMailbox->argument.inputInit.traceLevel;
#endif
        TRACE_INFO("-- NandFlash SAM-BA applet %s --\n\r", SAM_BA_APPLETS_VERSION);
        TRACE_INFO("-- %s\n\r", BOARD_NAME);
        TRACE_INFO("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
        TRACE_INFO("INIT command\n\r");

        /* Configure SMC for Nandflash accesses (done each time applet is launched because of old ROM codes) */
        BOARD_ConfigureNandFlash(nfBusWidth);
        PIO_Configure(pPinsNf, PIO_LISTSIZE(pPinsNf));
        /* Tries to detect NAND Flash device connected to EBI CS3, with data lines connected to D0-D7, 
           then on NAND Flash connected to D16-D23. */
        if (!NandEbiDetect()) {
            pMailbox->status = APPLET_DEV_UNKNOWN;
            TRACE_INFO("\tDevice Unknown\n\r");
            goto exit;
        }
        memset(&skipBlockNf, 0, sizeof(skipBlockNf));
        NandGetOnfiPageParam (&OnfiPageParameter);
        if (SkipBlockNandFlash_Initialize(&skipBlockNf,
                                         0,
                                         cmdBytesAddr,
                                         addrBytesAddr,
                                         dataBytesAddr,
                                         nfCePin,
                                         nfRbPin)) {
            pMailbox->status = APPLET_DEV_UNKNOWN;
            pMailbox->argument.outputInit.bufferSize = 0;
            pMailbox->argument.outputInit.memorySize = 0;
            TRACE_INFO("\tDevice Unknown\n\r");
        }
        else 
        {
            /* Check the data bus width of the NandFlash */
            nfBusWidth = NandFlashModel_GetDataBusWidth((struct NandFlashModel *)&skipBlockNf);
            /* Reconfigure bus width */
            if ( nfBusWidth != 8) {
                SMC->SMC_CS_NUMBER[3].SMC_MODE = SMC_MODE_READ_MODE | SMC_MODE_WRITE_MODE | SMC_MODE_DBW(nfBusWidth/16) | SMC_MODE_TDF_CYCLES(1); 
            }
            TRACE_INFO("\tNandflash driver initialized\n\r");
            pMailbox->argument.outputInit.bufferAddress = (unsigned int) &_end;
            /* Get device parameters */
            memSize = NandFlashModel_GetDeviceSizeInBytes(&skipBlockNf.ecc.raw.model);
            blockSize = NandFlashModel_GetBlockSizeInBytes(&skipBlockNf.ecc.raw.model);
            numBlocks = NandFlashModel_GetDeviceSizeInBlocks(&skipBlockNf.ecc.raw.model);
            pageSize = NandFlashModel_GetPageDataSize(&skipBlockNf.ecc.raw.model);
            spareSize = NandFlashModel_GetPageSpareSize(&skipBlockNf.ecc.raw.model);
            numPagesPerBlock = NandFlashModel_GetBlockSizeInPages(&skipBlockNf.ecc.raw.model);

            pMailbox->status = APPLET_SUCCESS;
            pMailbox->argument.outputInit.bufferSize = blockSize;
            pMailbox->argument.outputInit.memorySize = memSize;
            pMailbox->argument.outputInit.pmeccParamHeader = 0;
            TRACE_INFO("\tpageSize : 0x%x blockSize : 0x%x blockNb : 0x%x \n\r",  pageSize, blockSize, numBlocks);
        }
        /* By default, we use pmecc, except MICRON MLC nand with internal ECC controller */
        eccOffset = 2;
        /* By defaut, 2 error bit correction, eccOffset = 2 */
        PMECC_Initialize(&pmeccDesc, 0, eccCorrectability, pageSize, spareSize, eccOffset, 0);
        TRACE_INFO("\tNandflash PMECC initialized\n\r");
        DMAD_Initialize( &dmad, POLLING_MODE ); 
        if ( NandFlashConfigureDmaChannels( &dmad )) {
            pMailbox->status =APPLET_DEV_UNKNOWN;
            goto exit;
        }
        /* Initialize current pmecc parameter header, This 32-bit word is configured below */
        currentPmeccHeader.usePmecc = 1;
        currentPmeccHeader.nbSectorPerPage = pmeccDesc.pageSize >> 8;
        currentPmeccHeader.spareSize = spareSize;
        currentPmeccHeader.eccBitReq = pmeccDesc.errBitNbrCapability;
        currentPmeccHeader.sectorSize = pmeccDesc.sectorSize;
        currentPmeccHeader.eccOffset = pmeccDesc.eccStartAddr;
        currentPmeccHeader.reserved = 0;
        currentPmeccHeader.key = 12;
        memcpy(&backupPmeccHeader, &currentPmeccHeader, sizeof(nfParamHeader_t));
        memcpy(&currentPmeccHeaderValue, &currentPmeccHeader, sizeof(nfParamHeader_t));
        pMailbox->argument.outputInit.pmeccParamHeader = currentPmeccHeaderValue;
        /* The Boot Program reads the first page without ECC check, to determine if the NAND parameter 
           header is present. The header is made of 52 times the same 32-bit word (for redundancy
           reasons) which must contain NAND and PMECC parameters used to correctly perform the read of the rest 
           of the data in the NAND. */
        for (i = 0; i< 52; i++) memcpy(&bootNfParamHeader[i], &currentPmeccHeader, sizeof(nfParamHeader_t));
        NandSwitchEcc(ECC_PMECC);
        pMailbox->status = APPLET_SUCCESS;
    }

    /* ---------------------------------------------------------- */
    /* APPLET_CMD_TRIMFSS                                        */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_TRIMFFS) 
    {
        TRACE_INFO("TRIMFFS command\n\r");
        eccType = NandGetEccStatus();
        if (eccType != ECC_PMECC)
        {
            pMailbox->status = APPLET_PMECC_CONFIG;
            TRACE_INFO("\tOnly support for pmecc enabled.\n\r");
            goto exit;
        }
        
        NandSetTrimffs(pMailbox->argument.inputTrimMode.trim);
        TRACE_INFO("Current TRIMFFS is %d\n\r", isNandTrimffs());
        pMailbox->status = APPLET_SUCCESS;
    }
    /* ---------------------------------------------------------- */
    /* APPLET_CMD_PMECC_HEADER                                    */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_PMECC_HEADER) 
    {
        TRACE_INFO("PMECC Header configuration command\n\r");
        eccType = NandGetEccStatus();
        if (eccType != ECC_PMECC)
        {
            pMailbox->status = APPLET_PMECC_CONFIG;
            TRACE_INFO("\tOnly support for pmecc enabled.\n\r");
            goto exit;
        }
        nIndex = pMailbox->argument.inputPmeccHeader.pmeccParamIndex;
        nValue = pMailbox->argument.inputPmeccHeader.pmeccParamValue;
        memcpy(&backupPmeccHeader, &currentPmeccHeader, sizeof(nfParamHeader_t));
        switch ( nIndex )
        {
            case ECCHEADER_USEPMECC:
                TRACE_INFO("\tConfigure 'usePmecc' to %x.\n\r", nValue);
                backupPmeccHeader.usePmecc = nValue;
                break;
            case ECCHEADER_SECTORS:
                TRACE_INFO("\tConfigure 'nbSectorPerPage' to %x.\n\r", nValue);
                backupPmeccHeader.nbSectorPerPage = nValue;
                break;
            case ECCHEADER_SPARE:
                TRACE_INFO("\tConfigure 'spareSize' to %x.\n\r", nValue);
                backupPmeccHeader.spareSize = nValue;
                break;
            case ECCHEADER_ECCBIT:
                TRACE_INFO("\tConfigure 'eccBitReq' to %x.\n\r", nValue);
                backupPmeccHeader.eccBitReq = nValue;
                break;
            case ECCHEADER_SECTORSIZE:
                TRACE_INFO("\tConfigure 'sectorSize' to %x.\n\r", nValue);
                backupPmeccHeader.sectorSize = nValue;
                break;
            case ECCHEADER_ECCOFFSET:
                TRACE_INFO("\tConfigure 'eccOffset' to %x.\n\r", nValue);
                backupPmeccHeader.eccOffset = nValue;
                break;
           case ECCHEADER_ALL:
                TRACE_INFO("\tConfigure paramHeader to %x.\n\r", nValue);
                memcpy(&backupPmeccHeader, &nValue, sizeof(nfParamHeader_t));
                break;
           default:
               break;
        }
        if (!isValidPmeccParam (&backupPmeccHeader))
        {
            pMailbox->status = APPLET_PMECC_CONFIG;
            TRACE_INFO("\tSome parameter error in pmecc header configuration. \n\r");
            goto exit;
        }
        memcpy(&currentPmeccHeader, &backupPmeccHeader, sizeof(nfParamHeader_t));
        sectorSize = currentPmeccHeader.sectorSize;
        spareSize = currentPmeccHeader.spareSize;
        eccOffset = currentPmeccHeader.eccOffset;
        eccCorrectability = eccBitReq2TT[currentPmeccHeader.eccBitReq];

        /* The header is made of 52 times the same 32-bit word  */
        for (i = 0; i< 52; i++) memcpy(&bootNfParamHeader[i], &currentPmeccHeader, sizeof(nfParamHeader_t));
        if (currentPmeccHeader.usePmecc) {
            TRACE_INFO("\tNandflash PMECC nbSectorPerPage is %d\n\r", currentPmeccHeader.nbSectorPerPage );
            TRACE_INFO("\tNandflash PMECC spareSize is %d\n\r",spareSize );
            TRACE_INFO("\tNandflash PMECC eccBitReq is %d\n\r", eccCorrectability );
            TRACE_INFO("\tNandflash PMECC sectorSize is %d\n\r", currentPmeccHeader.sectorSize );
            TRACE_INFO("\tNandflash PMECC eccOffset is %d\n\r", currentPmeccHeader.eccOffset );
            PMECC_Initialize(&pmeccDesc, sectorSize, eccCorrectability, pageSize, spareSize , eccOffset, 0);
        }
        memcpy(&currentPmeccHeaderValue, &currentPmeccHeader, sizeof(nfParamHeader_t));
        pMailbox->argument.outputPmeccHeader.pmeccParamHeader = currentPmeccHeaderValue;
        TRACE_INFO("\tNandflash PMECC currentPmeccHeaderValue is %x\n\r", currentPmeccHeaderValue );
        pMailbox->status = APPLET_SUCCESS;
    }

    /* ---------------------------------------------------------- */
    /* Switch ECC mode:                                           */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_SWITCH_ECC) {
        eccMode = pMailbox->argument.inputEccMode.eccMode;
        error = 0;
        eccType = NandGetEccStatus();
        switch ( eccMode ) {
            case ECC_ENABLE_SOFT:
                TRACE_INFO("\tEnable software ECC\n\r");
                //if ( eccType != ECC_NO) error = 1;
                break;
            case ECC_DISABLE_SOFT:
                TRACE_INFO("\tDisable software ECC\n\r");
                if ( eccType == ECC_NO) error = 1;
                break;
            case ECC_ENABLE_PMECC:
                TRACE_INFO("\tEnable PMECC\n\r");
                //if ( eccType != ECC_NO) error = 1;
                break;
            case ECC_DISABLE_PMECC:
                TRACE_INFO("\tDisable PMECC\n\r");
                if ( eccType == ECC_NO) error = 1;
                break;
            case ECC_ENABLE_INTEECC:
                TRACE_INFO("\tEnable Micron Nandflash Internal Ecc\n\r");
                //if ( eccType != ECC_NO) error = 1;
                break;
            default:
                error = 1;
                break;
        }
        if ( error) {
            pMailbox->status = APPLET_FAIL;
            TRACE_INFO("\tSome parameter error in switching ecc mode. \n\r");
            goto exit;
        }
        if ( (eccMode == ECC_DISABLE_PMECC) || (eccMode == ECC_ENABLE_SOFT) || (eccMode == ECC_ENABLE_INTEECC)) {
            NandFlashFreeDma();
            PMECC_Disable();
        }
        if ( (eccMode == ECC_DISABLE_SOFT) || (eccMode == ECC_DISABLE_PMECC)) {
            NandSwitchEcc(ECC_NO);
        }
        if ( eccMode == ECC_ENABLE_SOFT) {
            NandSwitchEcc(ECC_SOFT);
        }
        if ( eccMode == ECC_ENABLE_INTEECC) {
           /* Detect if the NAND device have internal ECC controller (some MICRON nand )*/
            if (!NandEnableInternalEcc()) {
                pMailbox->status = APPLET_FAIL;
                TRACE_INFO("\tCan't detect a MICRON Nandflash device with internal ECC!\n\r");
                goto exit;
            }
        }
        if ( eccMode == ECC_ENABLE_PMECC) {
            eccOffset = 2;
            /* By defaut, 2 error bit correction, eccOffset = 2 */
            PMECC_Initialize(&pmeccDesc, 0, eccCorrectability, pageSize, spareSize, eccOffset, 0);
            TRACE_INFO("\tNandflash PMECC initialized\n\r");
            DMAD_Initialize( &dmad, POLLING_MODE ); 
            if ( NandFlashConfigureDmaChannels( &dmad ))
            {
                pMailbox->status =APPLET_DEV_UNKNOWN;
                goto exit;
            }
            /* Initialize current pmecc parameter header, This 32-bit word is configured below */
            currentPmeccHeader.usePmecc = 1;
            currentPmeccHeader.nbSectorPerPage = pmeccDesc.pageSize >> 8;
            currentPmeccHeader.spareSize = spareSize;
            currentPmeccHeader.eccBitReq = pmeccDesc.errBitNbrCapability;
            currentPmeccHeader.sectorSize = pmeccDesc.sectorSize;
            currentPmeccHeader.eccOffset = pmeccDesc.eccStartAddr;
            currentPmeccHeader.reserved = 0;
            currentPmeccHeader.key = 12;
            memcpy(&backupPmeccHeader, &currentPmeccHeader, sizeof(nfParamHeader_t));
            memcpy(&currentPmeccHeaderValue, &currentPmeccHeader, sizeof(nfParamHeader_t));
            pMailbox->argument.outputInit.pmeccParamHeader = currentPmeccHeaderValue;
            /* The Boot Program reads the first page without ECC check, to determine if the NAND parameter 
               header is present. The header is made of 52 times the same 32-bit word (for redundancy
               reasons) which must contain NAND and PMECC parameters used to correctly perform the read of the rest 
                of the data in the NAND. */
            for (i = 0; i< 52; i++) memcpy(&bootNfParamHeader[i], &currentPmeccHeader, sizeof(nfParamHeader_t));
            NandSwitchEcc(ECC_PMECC);
        }
        if ( eccMode != ECC_ENABLE_PMECC) {
            /* Set usePmecc as 0 in header */
            currentPmeccHeader.usePmecc = 0;
            currentPmeccHeader.nbSectorPerPage = 2;
            currentPmeccHeader.spareSize = spareSize;
            currentPmeccHeader.eccBitReq = 0;
            currentPmeccHeader.sectorSize = 0;
            currentPmeccHeader.eccOffset = 0;
            currentPmeccHeader.reserved = 0;
            currentPmeccHeader.key = 12;
            memcpy(&backupPmeccHeader, &currentPmeccHeader, sizeof(nfParamHeader_t));
            memcpy(&currentPmeccHeaderValue, &currentPmeccHeader, sizeof(nfParamHeader_t));
            pMailbox->argument.outputPmeccHeader.pmeccParamHeader = currentPmeccHeaderValue;
            /* The Boot Program reads the first page without ECC check, to determine if the NAND parameter 
            header is present. The header is made of 52 times the same 32-bit word (for redundancy
            reasons) which must contain NAND and PMECC parameters used to correctly perform the read of the rest 
            of the data in the NAND. */
            for (i = 0; i< 52; i++) memcpy(&bootNfParamHeader[i], &currentPmeccHeader, sizeof(nfParamHeader_t));
        }
        pMailbox->status = APPLET_SUCCESS;
    }

    /* ---------------------------------------------------------- */
    /* WRITE:                                                     */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_WRITE) {
        TRACE_INFO("SNED FILE command\n\r");
        eccType = NandGetEccStatus();
        if (eccType == ECC_PMECC) {
           PMECC_Initialize(&pmeccDesc, sectorSize, eccCorrectability, pageSize, spareSize , eccOffset, 0);
           TRACE_INFO("Pmecc initialized \n\r");
        } 
        memoryOffset = pMailbox->argument.inputWrite.memoryOffset;
        bufferAddr = pMailbox->argument.inputWrite.bufferAddr;
        tempBufferAddr = bufferAddr + blockSize;
        bytesToWrite = pMailbox->argument.inputWrite.bufferSize;

        TRACE_INFO("WRITE arguments : offset 0x%x, buffer at 0x%x, of 0x%x Bytes\n\r",
               memoryOffset, bufferAddr, bytesToWrite);
        
        pMailbox->argument.outputWrite.bytesWritten = 0;

        /* Check word alignment */
        if (memoryOffset % 4) {

            pMailbox->status = APPLET_ALIGN_ERROR;
            goto exit;
        }
       
        /* Retrieve page and block addresses */
        if (NandFlashModel_TranslateAccess(&(skipBlockNf.ecc.raw.model),
                                           memoryOffset,
                                           bytesToWrite,
                                           &block,
                                           &page,
                                           &offset)) {
            pMailbox->status = APPLET_FAIL;
            goto exit;
        }
        if (isNandTrimffs()) {
            NandSetTrimPage ( blockSize / pageSize );
            if (( offset == 0) && (bytesToWrite == blockSize)) {
               trimPage = drop_ffs((unsigned char *)bufferAddr);
               NandSetTrimPage((unsigned short)trimPage);
               if (trimPage <= (blockSize/pageSize)) {
                   TRACE_INFO(" -------------------------------------\n\r");
                   TRACE_INFO("TRIM JFSS @ page %d \n\r",trimPage);
                   TRACE_INFO(" -------------------------------------\n\r");
               }
            }
        }

        TRACE_INFO("WRITE at block 0x%x, page 0x%x, offset 0x%x\n\r", block, page, offset);
        if (page || offset || (bytesToWrite < blockSize)) {
            /* We are not block aligned, retrieve block content to update it */
            memset((unsigned int *)tempBufferAddr, 0xFF, blockSize);
            TRACE_INFO("Retrieve data in block,%d \n\r",block);
            error = SkipBlockNandFlash_ReadBlock(&skipBlockNf, block, (unsigned int *)tempBufferAddr);
            if (error == NandCommon_ERROR_BADBLOCK) {
                pMailbox->status = APPLET_BAD_BLOCK;
                goto exit;
            }
            if (error) {
                pMailbox->status = APPLET_FAIL;
                goto exit;
            }
            /* Fill retrieved block with data to be programmed */
            offsetInTargetBuff = (page * pageSize) + offset;
            offsetInSourceBuff = 0;
            while ((offsetInTargetBuff < blockSize) && (bytesToWrite > 0)) {
                *(unsigned int *)(tempBufferAddr + offsetInTargetBuff) = *(unsigned int *)(bufferAddr + offsetInSourceBuff);
                offsetInSourceBuff += 4;
                offsetInTargetBuff += 4;
                bytesToWrite -= 4;
            }
        }
        else {
            /* Write a full and aligned block */
            tempBufferAddr = bufferAddr;
            bytesToWrite = 0;
        }
        /* Erase target block */
        error = SkipBlockNandFlash_EraseBlock(&skipBlockNf, block, NORMAL_ERASE);
        if (error == NandCommon_ERROR_BADBLOCK) {
            pMailbox->status = APPLET_BAD_BLOCK;
            goto exit;
        }
        if (error) {
            pMailbox->status = APPLET_FAIL;
            goto exit;
        }
        /* Write target block */
        error = SkipBlockNandFlash_WriteBlock(&skipBlockNf, block, (unsigned int *)tempBufferAddr);
        if (error == NandCommon_ERROR_BADBLOCK) {
            pMailbox->status = APPLET_BAD_BLOCK;
            goto exit;
        }
        if (error) {
            pMailbox->status = APPLET_FAIL;
            goto exit;
        }
        pMailbox->argument.outputWrite.bytesWritten = pMailbox->argument.inputWrite.bufferSize - bytesToWrite;
        pMailbox->status = APPLET_SUCCESS;
    }

    /* ---------------------------------------------------------- */
    /* READ:                                                      */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_READ) {
        eccType = NandGetEccStatus();
        if (eccType == ECC_PMECC) { 
            PMECC_Initialize(&pmeccDesc, sectorSize, eccCorrectability, pageSize, spareSize , eccOffset, 0);
        }
        memoryOffset = pMailbox->argument.inputRead.memoryOffset;
        bufferAddr   = pMailbox->argument.inputRead.bufferAddr;
        tempBufferAddr = bufferAddr + blockSize;
        bufferSize   = pMailbox->argument.inputRead.bufferSize;

        TRACE_INFO("READ at offset: 0x%x buffer at : 0x%x of: 0x%x Bytes\n\r",
               memoryOffset, bufferAddr, bufferSize);

        pMailbox->argument.outputRead.bytesRead = 0;

        /* Check word alignment */
        if (memoryOffset % 4) {

            pMailbox->status = APPLET_ALIGN_ERROR;
            goto exit;
        }

        /* Retrieve page and block addresses*/
        if (NandFlashModel_TranslateAccess(&(skipBlockNf.ecc.raw.model),
                                           memoryOffset,
                                           bufferSize,
                                           &block,
                                           &page,
                                           &offset)) {
            pMailbox->status = APPLET_FAIL;
            goto exit;
        }
        
        TRACE_INFO("READ at block 0x%x, page 0x%x, offset in page 0x%x\n\r", block, page, offset);

        if (page || offset) {
            memset((unsigned int *)tempBufferAddr, 0xFF, blockSize);

            error = SkipBlockNandFlash_ReadBlock(&skipBlockNf, block, (unsigned int *)tempBufferAddr);
            if (error == NandCommon_ERROR_BADBLOCK) {
                pMailbox->status = APPLET_BAD_BLOCK;
                goto exit;
            }
            if (error) {
                pMailbox->status = APPLET_FAIL;
                goto exit;
            }

            /* Fill dest buffer with read data */
            offsetInSourceBuff = (page * pageSize) + offset;
            offsetInTargetBuff = 0;

            while ((offsetInSourceBuff < blockSize)
                    && (offsetInTargetBuff < blockSize)
                    && (bytesRead < bufferSize)) {

                *(unsigned int *)(bufferAddr + offsetInTargetBuff) = *(unsigned int *)(tempBufferAddr + offsetInSourceBuff);
                offsetInSourceBuff += 4;
                offsetInTargetBuff += 4;
                bytesRead += 4;
            }

            pMailbox->argument.outputRead.bytesRead = bytesRead;
            pMailbox->status = APPLET_SUCCESS;
        }
        else {

            memset((unsigned int *)bufferAddr, 0xFF, blockSize);
                
            error = SkipBlockNandFlash_ReadBlock(&skipBlockNf, block, (unsigned int *)bufferAddr);
            if (error == NandCommon_ERROR_BADBLOCK) {
                pMailbox->status = APPLET_BAD_BLOCK;
                goto exit;
            }
            if (error) {
                pMailbox->status = APPLET_FAIL;
                goto exit;
            }

            pMailbox->argument.outputRead.bytesRead = bufferSize;
            pMailbox->status = APPLET_SUCCESS;
        }
    }

    /* ---------------------------------------------------------- */
    /* SNED BOOT FILE:                                            */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_SENDBOOT) {
        TRACE_INFO("SNED BOOT FILE command\n\r");
        eccType = NandGetEccStatus();
        if (eccType == ECC_PMECC) { 
            PMECC_Initialize(&pmeccDesc, sectorSize, eccCorrectability, pageSize, spareSize , eccOffset, 0);
        }
        bufferAddr = pMailbox->argument.inputWrite.bufferAddr;
        bytesToWrite = pMailbox->argument.inputWrite.bufferSize;
        tempBufferAddr = bufferAddr + blockSize;
        memset((unsigned int *)tempBufferAddr, 0xFF, blockSize);
        error = SkipBlockNandFlash_EraseBlock(&skipBlockNf, 0, NORMAL_ERASE);
        if (error == NandCommon_ERROR_BADBLOCK) {

            pMailbox->status = APPLET_BAD_BLOCK;
            goto exit;
        }
        if (error) {

            pMailbox->status = APPLET_FAIL;
            goto exit;
        }
        /* The Boot Program reads the first page without ECC check, to determine if the NAND
           parameter header is present. The header is made of 52 times the same 32-bit word (for redundancy
           reasons) which must contain NAND and PMECC parameters used to correctly perform the read of the rest 
           of the data in the NAND.*/

        memcpy((unsigned int *)tempBufferAddr, (unsigned int *)bootNfParamHeader, 52 * 4);
        memcpy((unsigned int *)tempBufferAddr + 52, (unsigned int *)bufferAddr, bytesToWrite);
        /* Write target block */
        error = SkipBlockNandFlash_WriteBlock ( &skipBlockNf, 0, (unsigned int *)tempBufferAddr);
        if (error == NandCommon_ERROR_BADBLOCK) {

            pMailbox->status = APPLET_BAD_BLOCK;
            goto exit;
        }
        if (error) {
            pMailbox->status = APPLET_FAIL;
            goto exit;
        }
        pMailbox->status = APPLET_SUCCESS;
    }
    /* ---------------------------------------------------------- */
    /* FULL ERASE:                                                */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_FULL_ERASE) {

        TRACE_INFO("FULL ERASE command\n\r");
        TRACE_INFO("\tForce erase flag: 0x%x\n\r", pMailbox->argument.inputFullErase.eraseType);
        for (i = 0; i < numBlocks; i++) {

            /* Erase the page */
            if (SkipBlockNandFlash_EraseBlock(&skipBlockNf, i, pMailbox->argument.inputFullErase.eraseType)) {

                TRACE_INFO("Found block #%d BAD, skip it\n\r", i);
            }
        }
        TRACE_INFO("Full Erase achieved\n\r");
        pMailbox->status = APPLET_SUCCESS;
    }

    /* ---------------------------------------------------------- */
    /* BATCH FULL ERASE:                                          */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_BATCH_ERASE) {

        TRACE_INFO("BATCH ERASE command\n\r");
        block = pMailbox->argument.inputBatchErase.batch * (numBlocks / ERASE_BATCH);

        TRACE_INFO("Erase block from #%d to #%d\n\r", block, block + (numBlocks / ERASE_BATCH));
        for (i = block ; i < block + (numBlocks / ERASE_BATCH) ; i++) {

            /* Erase the block */
            if (SkipBlockNandFlash_EraseBlock(&skipBlockNf, i, pMailbox->argument.inputBatchErase.eraseType)) {

                TRACE_INFO("Found block #%d BAD, skip it\n\r", i);
            }
        }

        if ((pMailbox->argument.inputBatchErase.batch + 1) == ERASE_BATCH) {
            TRACE_INFO("Full Erase achieved, erase type is %d\n\r", pMailbox->argument.inputBatchErase.eraseType);
            pMailbox->argument.outputBatchErase.nextBatch = 0;
        }
        else {
            pMailbox->argument.outputBatchErase.nextBatch =  pMailbox->argument.inputBatchErase.batch + 1;
            TRACE_INFO("Batch Erase achieved\n\r");
        }
        pMailbox->status = APPLET_SUCCESS;
    }

    /* ---------------------------------------------------------- */
    /* ERASE_BLOCKS:                                              */
    /* ---------------------------------------------------------- */

    else if (pMailbox->command == APPLET_CMD_ERASE_BLOCKS) {

        TRACE_INFO("BLOCKS ERASE command\n\r");
        memoryOffset = pMailbox->argument.inputBlocksErase.memoryOffsetStart;
        if ((pMailbox->argument.inputBlocksErase.memoryOffsetEnd > memSize) || (pMailbox->argument.inputBlocksErase.memoryOffsetEnd < memoryOffset) ) {
            TRACE_INFO("Out of memory space\n\r");
            pMailbox->status = APPLET_ERASE_FAIL;
            goto exit;
        }
        nbBlocks = ((pMailbox->argument.inputBlocksErase.memoryOffsetEnd- memoryOffset)/ blockSize) + 1;

        TRACE_INFO("Erase blocks from %d  to %d \n\r",  memoryOffset / blockSize, (memoryOffset / blockSize)+ nbBlocks );
        /* Erase blocks */
        for (i =  memoryOffset / blockSize; i < memoryOffset / blockSize + nbBlocks ; i++) {
            if (SkipBlockNandFlash_EraseBlock(&skipBlockNf,  i , NORMAL_ERASE)) {
                 TRACE_INFO("Found block #%d BAD, skip it\n\r",  i);
            }
        }
        TRACE_INFO("Blocks Erase achieved\n\r");
        pMailbox->status = APPLET_SUCCESS;
    }
    /* ---------------------------------------------------------- */
    /* LIST BAD BLOCKS:                                           */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_LIST_BAD_BLOCKS) {

        TRACE_INFO("LIST BAD BLOCKS command\n\r");
        nbBadBlocks = 0;
        bufferAddr = (unsigned int) &_end;
        pMailbox->argument.outputListBadBlocks.bufferAddress = bufferAddr;

        for (i = 0; i < numBlocks; i++) {

            /* Erase the page */
            if (SkipBlockNandFlash_CheckBlock(&skipBlockNf, i) == BADBLOCK) {

                nbBadBlocks++;
                *((unsigned int *)bufferAddr) = i;
                bufferAddr += 4;
                TRACE_INFO("Found block #%d BAD\n\r", i);
            }
        }
        TRACE_INFO("LIST BAD BLOCKS achieved\n\r");
        pMailbox->argument.outputListBadBlocks.nbBadBlocks = nbBadBlocks;
        pMailbox->status = APPLET_SUCCESS;
    }

    /* ---------------------------------------------------------- */
    /* TAG BLOCK:                                                 */
    /* ---------------------------------------------------------- */
    else if (pMailbox->command == APPLET_CMD_TAG_BLOCK) {

        TRACE_INFO("TAG BLOCK command\n\r");
        bufferAddr = (unsigned int) &_end;
        block = pMailbox->argument.inputTagBlock.blockId;

        /* To tag the block as good, just erase it without bad block check */
        if ((unsigned char)pMailbox->argument.inputTagBlock.tag == 0xFF)
        {
            if (SkipBlockNandFlash_EraseBlock(&skipBlockNf, block, SCRUB_ERASE)) {

                TRACE_INFO("Cannot erase block %d\n\r", block);
                pMailbox->status = APPLET_FAIL;
                goto exit;
            }
        }
        else {
            for (i = 0; i < 2; i++) {

                /* Start by reading the spare */
                memset((unsigned char *)bufferAddr, 0xFF, NandCommon_MAXSPAREECCBYTES);

                TRACE_INFO("Tag to write : 0x%x\n\r", (unsigned char)pMailbox->argument.inputTagBlock.tag);

                NandSpareScheme_WriteBadBlockMarker((struct NandSpareScheme *)(NandFlashModel_GetScheme((struct NandFlashModel *)(&skipBlockNf))),
                                                    (unsigned char *)bufferAddr,
                                                    ((unsigned char)pMailbox->argument.inputTagBlock.tag));

                if (RawNandFlash_WritePage((struct RawNandFlash *)(&skipBlockNf), block, i, 0, (unsigned char *)bufferAddr)) {

                    TRACE_ERROR("Failed to write spare data of page %d of block %d\n\r", i, block);
                    pMailbox->status = APPLET_FAIL;
                    goto exit;
                }
            }
        }
        TRACE_INFO("TAG BLOCK achieved\n\r");
        pMailbox->status = APPLET_SUCCESS;
    }

exit :
    /* Acknowledge the end of command */
    TRACE_INFO("\tEnd of applet (command : %x --- status : %x)\n\r", pMailbox->command, pMailbox->status);

    /* Notify the host application of the end of the command processing */
    pMailbox->command = ~(pMailbox->command);
    /* Send ACK character */
    if (comType == DBGU_COM_TYPE) {
         /* Wait for the transmitter to be ready */
        while ( (DBGU->DBGU_SR & DBGU_SR_TXEMPTY) == 0 ) ;
        /* Send character */
         DBGU->DBGU_THR= 0x06 ;
    }
    return 0;
}

