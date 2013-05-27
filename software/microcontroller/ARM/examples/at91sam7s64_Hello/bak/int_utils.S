.equ AIC_IVR,         (256) 
.equ AIC_FVR,         (260) 
.equ AIC_EOICR,       (304)
.equ AT91C_BASE_AIC,  (0xFFFFF000)
.equ    I_Bit,          0x80    /* when I bit is set, IRQ is disabled */
.equ    Mode_IRQ,       0x12
.equ    Mode_SVC,       0x13

        .text
        .arm
		.section .text
		
        .global NESTINTSTART
        .func   NESTINTSTART
NESTINTSTART:

        sub         lr, lr, #4
        stmfd       sp!, {lr}
        mrs         r14, SPSR
        stmfd       sp!, {r14}
        ldr         r14, =AT91C_BASE_AIC
	    str         r14, [r14, #AIC_IVR]
        msr         CPSR_c, #Mode_SVC
        stmfd       sp!, { r0-r12, lr}

		.endfunc


        .global NESTINTEND
        .func   NESTINTEND
NESTINTEND:

		ldmfd sp!,{r1}
        msr   spsr_c,r1
        ldmfd sp!,{r0-r12,pc}
	    msr         CPSR_c, #I_Bit | Mode_IRQ
        
		.endfunc

.end
