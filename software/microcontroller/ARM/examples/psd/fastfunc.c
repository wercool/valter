#include "Board.h"
#include "fastfunc.h"

#define PAGE_SIZE 32
// ***************************************************************
// ** flashWrite - Write the given data to the given address in
// **              Flash memory. Erases the 256 byte page before
// **              writing.
// **              address must be DWORD-aligned.
// **              data must not cross a 256-byte boundary.
// **              Returns nonzero on successful write.
// **
FASTRUN int flashWrite(unsigned address, const void *data, unsigned size)
{
    unsigned i;
    unsigned page;
    unsigned *src  = (unsigned *)data;
    unsigned *dest = (unsigned *)address;

// Get base location of memory controller peripheral.
    volatile AT91PS_MC mc = AT91C_BASE_MC;

// Calculate the number of DWORDs to be written.
    size = (size + 3) / 4;

// Copy the data into the flash write buffer (this must be done 32 bits at a time).
/*
    for(i = 0; i < size; i++)
        dest[i] = src[i];
*/
    for(i = 0; i < PAGE_SIZE; i++)
    {
        dest[i] = src[i];
    }

    //disable interrupts
    unsigned int mask;
    mask = AT91C_BASE_AIC->AIC_IMR;
    AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;

    flashInit();
// Write the page to Flash memory.
    page = ((char *)dest - AT91C_IFLASH) / AT91C_IFLASH_PAGE_SIZE;
    //mc->MC_FCR = AT91C_MC_WRITE_KEY | AT91C_MC_FCMD_START_PROG | (page << 8);
    mc->MC_FCR = AT91C_MC_CORRECT_KEY | AT91C_MC_FCMD_START_PROG | (AT91C_MC_PAGEN & (page << 8)) ;

// Check for errors.
    if(0 != (mc->MC_FSR & AT91C_MC_PROGE))
        return 0;

// Wait for write cycle to complete.
    while(0 == (mc->MC_FSR & AT91C_MC_FRDY));

    //enable interrupts
    AT91C_BASE_AIC->AIC_IECR = mask;

    return 1;
}
