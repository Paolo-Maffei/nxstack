.module funcproxy_placeholder

	.area HOME    (CODE)
	.area GSINIT0 (CODE)
	.area GSINIT1 (CODE)
	.area GSINIT2 (CODE)
	.area GSINIT3 (CODE)
	.area GSINIT4 (CODE)
	.area GSINIT5 (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area CSEG    (CODE)

	.globl _FMAP
	.globl _callplaceholder
	

	.area CSEG    (CODE)
_bc_callplaceholder::
	xch	a,r4	;//save a
	mov a, #$_callplaceholder_no_params_on_stack
	jz no_stack_use
	
	pop acc		;//get return address
	mov R2,a	;
	pop acc		;
	mov R3,a	;
	
	push acc	;//make space for moving RA
	push acc	;//make space for moving RA
	push acc	;//make space for moving FMAP
	
	; now the stack is 3 bytes after parameters
	
	mov	a,SP;
	mov R0,a	;//target address, this will move parameters 3 bytes up the stack
	add a,#0xFD ; -3 //parameters are 3 bytes below SP  return address + extra byte reserved for FMAP
	mov R1,a	;//get from here,  get the original parameters
	
	
	mov R6,#$_callplaceholder_no_params_on_stack
	CJNE R6,#00,loop
	sjmp loopend
loop:
	
	;//for number of parameters
	;//copy parameter bytes
	mov a,@R1
	mov @R0, a
	
	;//move pointers
	dec R0
	dec R1
	;//loop end
	DJNZ R6, loop
loopend:
	
	
	;//save return address and _FMAP
	mov @R0, _FMAP
	dec R0
	mov a, R2
	mov @R0, a
	dec R0
	mov a, R3
	mov @R0, a
	
	mov _FMAP,#bank ;switch bank - do it right now, why wait
	
	xch a,r4
	;//SP set to the last parameter previously
	lcall _callplaceholder
	
	;//now restore
	xch a,r4
	pop acc;//remove one byte to get to the original position
	mov a, SP
	mov R0,a
	add a,#$_callplaceholder_no_params_on_stack_neg; -((param cnt)-1)
	inc a
	mov R1,a
	
	mov _FMAP, @R1;//restore FMAP
	
	dec R1		;//we are now at the return address
	
	mov a,@R1	;//move first byte of return address
	mov @R0,a
	
	dec R1		;//move down the stack
	dec R0
	
	mov a,@R1  ;//move second byte of return address
	mov @R0,a
	
	xch a,r4
	ret			;ret
	
no_stack_use:
	xch	a,r4
	push _FMAP
	mov _FMAP,#bank ;switch bank - do it right now, why wait
	lcall _callplaceholder
	pop _FMAP
	ret


