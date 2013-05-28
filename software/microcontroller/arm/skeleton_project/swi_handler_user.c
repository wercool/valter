/* User SWI Handler for SWI's not handled in swi_handler.S */

#include <stdio.h>
#include "Board.h"
#include "dbgu.h"

unsigned long SWI_Handler_User(unsigned long reg0,
    unsigned long reg1,
    unsigned long reg2,
    unsigned long swi_num )
{
    unsigned long res;

    res = 0;

    return res;
}
