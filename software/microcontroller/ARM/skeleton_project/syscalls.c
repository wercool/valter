/***********************************************************************/
/*                                                                     */
/*  SYSCALLS.C:  System Calls for the newlib                                      */
/*  most of this is from newlib-lpc and a Keil-demo                    */
/*                                                                     */
/*  These are "reentrant functions" as needed by                       */
/*  the WinARM-newlib-config, see newlib-manual.                       */
/*  Collected and modified by Martin Thomas                            */
/*                                                                     */
/***********************************************************************/

/* adapted for the SAM7 at91_lib DBGU - mthomas 4/2006 */

#include <stdlib.h>
#include <reent.h>
#include <sys/stat.h>

#include "Board.h"


_ssize_t _read_r(
    struct _reent *r, 
    int file, 
    void *ptr, 
    size_t len)
{
	unsigned char *p;
	
	while ( AT91F_US_RxReady((AT91PS_USART)AT91C_BASE_DBGU) == 0 ) {
		;
	}
    
	p = ptr;
	
	*p= AT91F_US_GetChar((AT91PS_USART)AT91C_BASE_DBGU);
    
	return (_ssize_t)1;
}


_ssize_t _write_r (
    struct _reent *r, 
    int file, 
    const void *ptr, 
    size_t len)
{
	size_t todo;
	const unsigned char *p;
	
	todo = len;
	p = ptr;
	
	for( ; todo != 0; todo--) {
		if ( *p == '\n' ) {
			while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_DBGU)) {
				;
			}
			AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_DBGU, '\r');
		}
		while (!AT91F_US_TxReady((AT91PS_USART)AT91C_BASE_DBGU)) {
			;
		}
		AT91F_US_PutChar((AT91PS_USART)AT91C_BASE_DBGU, *p++);
    }
	
	return (_ssize_t)len;			/* Number of bytes written.	*/
}


int _close_r(
    struct _reent *r, 
    int file)
{
	return 0;
}


_off_t _lseek_r(
    struct _reent *r, 
    int file, 
    _off_t ptr, 
    int dir)
{
	return (_off_t)0;	/*  Always indicate we are at file beginning.  */
}


int _fstat_r(
    struct _reent *r, 
    int file, 
    struct stat *st)
{
	/*  Always set as character device.				*/
	st->st_mode = S_IFCHR;
	/* assigned to strong type with implicit 	*/
	/* signed/unsigned conversion.  Required by 	*/
	/* newlib.					*/

	return 0;
}


int isatty(int file); /* avoid warning */

int isatty(int file)
{
	return 1;
}

void abort(void)
{
	while(1);
}

#if 0
static void _exit (int n) {
label:  goto label; /* endless loop */
}
#endif 


/* "malloc clue function" from newlib-lpc/Keil-Demo/"generic" */

/**** Locally used variables. ****/
// mt: "cleaner": extern char* end;
extern char end[];              /*  end is set in the linker command 	*/
				/* file and is the end of statically 	*/
				/* allocated data (thus start of heap).	*/

static char *heap_ptr;		/* Points to current end of the heap.	*/

/************************** _sbrk_r *************************************
 * Support function. Adjusts end of heap to provide more memory to
 * memory allocator. Simple and dumb with no sanity checks.

 *  struct _reent *r -- re-entrancy structure, used by newlib to
 *                      support multiple threads of operation.
 *  ptrdiff_t nbytes -- number of bytes to add.
 *                      Returns pointer to start of new heap area.
 *
 *  Note:  This implementation is not thread safe (despite taking a
 *         _reent structure as a parameter).
 *         Since _s_r is not used in the current implementation, 
 *         the following messages must be suppressed.
 */
void * _sbrk_r(
    struct _reent *_s_r, 
    ptrdiff_t nbytes)
{
	char  *base;		/*  errno should be set to  ENOMEM on error  */

	if (!heap_ptr) {	/*  Initialize if first time through.  */
		heap_ptr = end;
	}
	base = heap_ptr;	/*  Point to end of heap.  */
	heap_ptr += nbytes;	/*  Increase heap.  */
	
	return base;		/*  Return pointer to start of new heap area.  */
}