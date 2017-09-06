;//*****************************************************************************************
;//  Embedded System Design Using ARM Technology
;//  Final Project - Tree ARM
;//  Author: Nicholas Scoville
;//  File: load_buffer.asm
;//*****************************************************************************************

;//*****************************************************************************************
;//  Function:      __load_buffer

;//  Description:   load buffer (BUF) with RGB values from LED array (R_ADR, G_ADR, B_ADR)
;//  Parameters:    r0 = (R_ADR) Address of R values
;//					r1 = (G_ADR) Address of G values
;//					r2 = (B_ADR) Address of B values
;//					r3 = (BUF)	Buffer to write encoded RGB values to
;//  Return:        None
;//*****************************************************************************************

;//Register definitions for caller arguments
	.define		r0, R_ADR		;	register to hold address of Red values
	.define		r1, G_ADR		;	register to hold address of Green values
	.define		r2, B_ADR		;	register to hold address of Blue values
	.define		r3, BUF			;	register to hold address of BUF

;//Register definitions for function variables
	.define		r4, RED			;	register to hold Red values to be tested
	.define		r5, GREEN		;	register to hold Green values to be tested
	.define		r6, BLUE		;	register to hold Blue values to be tested
	.define		r7, ZERO		;	register to representation of 0 bit for BUF
	.define		r8, ONE			;	register to representation of 1 bit for BUF
	.define		r9, TMP			;	register to hold temp value for testing

	.global __load_buffer


;//Function checks each bit of each byte and loads the output buffer with the appropriate
;//bytes to represent 0's and 1's at the UART output.  0xFE represents a zero and 0xF0 a one.
;//The bytes also have to be reverse.
__load_buffer:
	STMFD	sp!, {RED,GREEN,BLUE,ZERO,ONE,TMP}	;	Save Registers used by function
	LDR		RED, [R_ADR]
	LDR		GREEN, [G_ADR]
	LDR		BLUE, [B_ADR]
	MOV		ZERO, #0xFE
	MOV		ONE, #0xF0
	MOV		TMP, #1
	ADD		BUF, #7
loop1	;//led 1
	TST		GREEN, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF, #8]
	STRBNE	ONE, [BUF, #8]
	TST		BLUE, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF, #16]
	STRBNE	ONE, [BUF, #16]
	TST		RED, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF], #-1
	STRBNE	ONE, [BUF], #-1
next1
	LSL		TMP, TMP, #1
	TST		TMP, #0x100
	BEQ		loop1
	ADD		BUF, #32
loop2	;//led 2
	TST		GREEN, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF, #8]
	STRBNE	ONE, [BUF, #8]
	TST		BLUE, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF, #16]
	STRBNE	ONE, [BUF, #16]
	TST		RED, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF], #-1
	STRBNE	ONE, [BUF], #-1
next2
	LSL		TMP, TMP, #1
	TST		TMP, #0x10000
	BEQ		loop2
	ADD		BUF, #32
loop3	;//led 3
	TST		GREEN, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF, #8]
	STRBNE	ONE, [BUF, #8]
	TST		BLUE, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF, #16]
	STRBNE	ONE, [BUF, #16]
	TST		RED, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF], #-1
	STRBNE	ONE, [BUF], #-1
next3
	LSL		TMP, TMP, #1
	TST		TMP, #0x1000000
	BEQ		loop3
	ADD		BUF, #32
loop4	;//led 4
	TST		GREEN, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF, #8]
	STRBNE	ONE, [BUF, #8]
	TST		BLUE, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF, #16]
	STRBNE	ONE, [BUF, #16]
	TST		RED, TMP
	ITE		EQ
	STRBEQ	ZERO, [BUF], #-1
	STRBNE	ONE, [BUF], #-1
next4
	LSLS	TMP, TMP, #1
	BNE		loop4
return
	LDMFD	sp!, {RED,GREEN,BLUE,ZERO,ONE,TMP}	;	Restore Registers used by function
	BX		lr									;	Return
