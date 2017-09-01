;//*****************************************************************************************
;//  Embedded System Design Using ARM Technology
;//  Lab 7 - Bubble Sort
;//  Author: Nicholas Scoville
;//  File: BubSort.asm
;//*****************************************************************************************

;//*****************************************************************************************
;//  Function:      __BubSort

;//  Description:   Sort Array (A) with (N) elements using the Bubble Sort Algorithm
;//  Parameters:    r0 = (A) Address of Array
;//					r1 = (N) Number of Array Elements
;//  Return:        None
;//*****************************************************************************************

;//Register definitions for caller arguments
	.define		r0, R_ADR		;	register to hold starting address of Array (A)
	.define		r1, G_ADR		;	register to hold Number of Array Elements (N)
	.define		r2, B_ADR		;
	.define		r3, BUF		;

;//Register definitions for function variables
	.define		r4, RED		;	register to hold first Array Element to be compared
	.define		r5, GREEN		;	register to hold first Array Element to be compared
	.define		r6, BLUE		;	register to hold swapped flag
	.define		r7, ZERO		;	register to hold loop count
	.define		r8, ONE		;	register to hold calculated top address of Array(A)
	.define		r9, TMP		;	register to hold temp value for swapping

	.global __load_buffer

__load_buffer:
	STMFD	sp!, {RED,GREEN,BLUE,ZERO,ONE,TMP}	;	Save Registers used by function
	LDR		RED, [R_ADR]
	LDR		GREEN, [G_ADR]
	LDR		BLUE, [B_ADR]
	MOV		ZERO, #0xFE
	MOV		ONE, #0xF0
	MOV		TMP, #1
	ADD		BUF, #7
loop1
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
loop2
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
loop3
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
loop4
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
	BX		lr				;	Return

