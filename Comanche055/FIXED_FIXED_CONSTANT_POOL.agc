# Copyright:	Public domain.
# Filename:	FIXED_FIXED_CONSTANT_POOL.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1200-1204
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#
# This source code has been transcribed or otherwise adapted from digitized
# images of a hardcopy from the MIT Museum.  The digitization was performed
# by Paul Fjeld, and arranged for by Deborah Douglas of the Museum.  Many
# thanks to both.  The images (with suitable reduction in storage size and
# consequent reduction in image quality as well) are available online at
# www.ibiblio.org/apollo.  If for some reason you find that the images are
# illegible, contact me at info@sandroid.org about getting access to the
# (much) higher-quality images which Paul actually created.
#
# Notations on the hardcopy document read, in part:
#
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  10:28 APR. 1, 1969
#
#	This AGC program shall also be referred to as
#			Colossus 2A

; ============================================================================
; FILE: FIXED_FIXED_CONSTANT_POOL.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Mathematical and physical constants pool stored in fixed (ROM) memory.
;        Contains high-precision values for π, e, gravitational parameters,
;        planetary radii, scaling factors, bit masks, and priority codes.
;        Referenced throughout computational code for orbital mechanics,
;        navigation, guidance calculations, and executive task scheduling
;        during all phases of the Apollo 11 mission.
;
; COMMENT-ONLY READERS: The mathematical constants the computer needed for
;        precise calculations of spacecraft motion and mission control.
; CODE-ALONG READERS: Study bit patterns, mathematical constants (π-derived),
;        priority codes for task scheduling, scaling factors for fixed-point
;        arithmetic, and predefined masks used throughout AGC programs.
; ============================================================================

# Page 1200
		BLOCK	02

		COUNT	02/FCONS

; ============================================================================
; INDEXED CONSTANT TABLE (18 VALUES)
; This table is indexed by address arithmetic throughout the AGC code.
; Order is critical - do not insert or remove quantities.
; ============================================================================

# THE FOLLOWING TABLE OF 18 VALUES IS INDEXED.  DO NOT INSERT OR REMOVE ANY QUANTITIES.

; Maximum positive values in AGC 15-bit signed arithmetic (+16383 decimal)
; Used for range checking, overflow detection, and as default maximum values.
; OCT 37777 = binary 011111111111111 = decimal +16383
DPOSMAX		OCT	37777		# MUST PRECEDE POSMAX
POSMAX		OCT	37777

LIMITS		=	NEG1/2

; Negative one-half in AGC fixed-point scaling (-0.5 decimal)
; Used by SIN routine for argument range reduction.
; OCT -20000 = binary 101000000000000 = decimal -8192 (represents -0.5 scaled)
; Must be positioned two locations before BIT14 for SIN algorithm indexing.
NEG1/2		OCT	-20000		# USED BY SIN ROUTINE (MUST BE TWO
					# LOCATIONS IN FRONT OF BIT14)

; ============================================================================
; BIT MASK TABLE
; Individual bit patterns for bit manipulation, masking, and testing.
; Each constant represents a single bit position in the 15-bit AGC word.
; Used throughout the code for flag testing, bit masking, and logical operations.
; ============================================================================

# BIT TABLE

; BIT15 = Sign bit position, also used as NEGMAX constant (most negative value)
; OCT 40000 = binary 100000000000000 = decimal -16384 (sign bit set)
BIT15		OCT	40000

; BIT14-BIT1: Individual bit positions for masking and testing
; BIT14 = OCT 20000 = binary 010000000000000 = decimal +8192
; Also represents 0.5 in certain scaling contexts (HALF constant)
BIT14		OCT	20000
BIT13		OCT	10000
BIT12		OCT	04000
BIT11		OCT	02000
BIT10		OCT	01000
BIT9		OCT	00400
BIT8		OCT	00200
BIT7		OCT	00100
BIT6		OCT	00040
BIT5		OCT	00020
BIT4		OCT	00010
BIT3		OCT	00004
BIT2		OCT	00002

; BIT1 = Least significant bit (also used as constant ONE)
; OCT 00001 = binary 000000000000001 = decimal +1
BIT1		OCT	00001


; ============================================================================
; DOUBLE PRECISION ZERO PAIR
; Critical constant pair for double-precision arithmetic operations.
; The AGC's double-precision instructions require NEG0 immediately before ZERO.
; ============================================================================

# DO NOT DESTROY THIS COMBINATION, SINCE IT IS USED IN DOUBLE PRECISION INSTRUCTIONS.

; Negative zero (-0) - required for double-precision operations
; In sign-magnitude representation, both +0 and -0 are valid representations.
; NEG0 must immediately precede ZERO for DCOMP and other DP instructions.
NEG0		OCT	-0		# MUST PRECEDE ZERO

; Positive zero (+0) - standard zero value
; Used extensively as initialization value and in comparisons.
; Must immediately follow NEG0 for double-precision instruction compatibility.
ZERO		OCT	0		# MUST FOLLOW NEG0
; ============================================================================
; SMALL INTEGER CONSTANTS
; Frequently-used small integer values for counting, indexing, and arithmetic.
; Many are aliased at end of file for different usage contexts.
; ============================================================================

# BIT1		OCT	00001
# NO.WDS	OCT	2		# INTERPRETER
# OCTAL3	OCT	3		# INTERPRETER
# R3D1		OCT	4		# PINBALL

; Constant five - used in loop counters and arithmetic operations
FIVE		OCT	5

# REVCNT	OCT	6		# INTERPRETER

; Constant seven - used for masking lower 3 bits (LOW3 alias)
SEVEN		OCT	7

# BIT4		OCT	00010
# R2D1		OCT	11		# PINBALL

; Octal 11 (decimal 9) - aliased as R2D1 for PINBALL display routines
OCT11		=	R2D1		# P20S

# BINCON	DEC	10		# PINBALL		(OCTAL 12)

; Decimal 11 - used for display field widths and counter limits
ELEVEN		DEC	11

# OCT14		OCT	14		# ALARM AND ABORT (FILLER)

; Octal 15 (decimal 13) - various utility uses
OCT15		OCT	15

# R1D1		OCT	16		# PINBALL
# Page 1201

; Mask for lower 4 bits (bits 1-4, value 0-15)
; OCT 17 = binary 000000000001111 = decimal 15
LOW4		OCT	17
# BIT5		OCT	00020
# ND1		OCT	21		# PINBALL
# VD1		OCT	23		# PINBALL
# OCT24		OCT	24		# SERVICE ROUTINES
# MD1		OCT	25		# PINBALL

; Combined bit mask for bits 4 and 5
; OCT 30 = binary 000000000011000 = decimal 24
BITS4&5		OCT	30

# OCT31		OCT	31		# SERVICE ROUTINES

; CALLCODE - used in bank calling mechanism for subroutine linkage
; OCT 00032 = decimal 26
CALLCODE	OCT	00032

# LOW5		OCT	37		# PINBALL
# 33DEC		DEC	33		# PINBALL		(OCTAL 41)
# 34DEC		DEC	34		# PINBALL		(OCTAL 42)

; Time constants for Digital Autopilot (DAP) testing
; TBUILDFX = 37 centiseconds (0.37 seconds) - DAP buildup time constant
TBUILDFX	DEC	37		# BUILDUP FOR CONVIENCE IN DAPTESTING

; TDECAYFX = 38 centiseconds (0.38 seconds) - DAP decay time constant
TDECAYFX	DEC	38		# CONVENIENCE FOR DAPTESTING

# BIT6		OCT	00040

; Octal 50 (decimal 40) - various utility uses
OCT50		OCT	50

; Decimal 45 - used in timing and display calculations
DEC45		DEC	45

; Superbank setting 011 - memory bank configuration bits
; Used for addressing upper memory regions in fixed (ROM) memory.
SUPER011	OCT	60		# BITS FOR SUPERBNK SETTING 011.

; ============================================================================
; TIME CONSTANTS
; Mission timing values in centiseconds (1/100 second units).
; AGC operates on 10-millisecond clock ticks, with time scaled to centiseconds.
; ============================================================================

; Half-second interval (50 centiseconds = 0.5 seconds)
; Used for timing delays, display updates, and crew interface response timing.
.5SEC		DEC	50
# BIT7		OCT	00100

; ============================================================================
; SUPERBANK CONFIGURATION CODES
; Memory bank selection codes for accessing different regions of fixed memory.
; AGC fixed memory organized in banks; superbank bits select which 4K or 8K region.
; ROPE = Read-Only Program Environment (core rope memory)
; ACM = Auxiliary Core Memory
; ============================================================================

; Superbank 100 - addresses last 4K of core rope memory
SUPER100	=	BIT7		# BITS FOR SUPERBNK SETTING 100
					# (LAST 4K OF ROPE)

; Superbank 101 - addresses first 8K of auxiliary core memory
; OCT 120 = binary 000000001010000 = decimal 80
SUPER101	OCT	120		# BITS FOR SUPERBNK SETTING 101
# OCT121	OCT	121		# SERVICE ROUTINES
					# (FIRST 8K OF ACM)

; Superbank 110 - addresses last 8K of auxiliary core memory
; OCT 140 = binary 000000001100000 = decimal 96
SUPER110	OCT	140		# BITS FOR SUPERBNK SETTING 110.
					# (LAST 8K OF ACM)

; One-second interval (100 centiseconds = 1.0 seconds)
; Used for countdown timers, display refresh, and mission event timing.
1SEC		DEC	100

# LOW7		OCT	177		# INTERPRETER
# BIT8		OCT	00200
# OT215		OCT	215		# ALARM AND ABORT
# 8,5		OCT	00220		# P20-P25 SUNDANCE

; Two-second interval (200 centiseconds = 2.0 seconds)
2SECS		DEC	200

# LOW8		OCT	377		# PINBALL
# BIT9		OCT	00400

; Guidance/Navigation control code - sets spacecraft control switch to G/N mode
; OCT 00401 = decimal 257 - command code for G/N control authority
GN/CCODE	OCT	00401		# SET S/C CONTROL SWITCH TO G/N

; Three-second interval (300 centiseconds = 3.0 seconds)
3SECS		DEC	300

; Four-second interval (400 centiseconds = 4.0 seconds)
4SECS		DEC	400

; Mask for lower 9 bits (bits 1-9, value 0-511)
; OCT 777 = binary 000111111111 = decimal 511
LOW9		OCT	777

# BIT10		OCT	01000
# 5.5DEGS	DEC	.03056		# P20-P25 SUNDANCE 	(OCTAL 00765)
# OCT1103	OCT	1103		# ALARM AND ABORT

; ============================================================================
; MATHEMATICAL AND DISPLAY CONSTANTS
; ============================================================================

; C5/2 - Mathematical constant (5/2 scaling factor = 0.0363551)
; Used in interpretive calculations for trigonometric and vector operations.
; OCT 01124 representation in fixed-point format.
C5/2		DEC	.0363551	#		   	(OCTAL 01124)

; V05N09 - DSKY Verb 05, Noun 09 combination code
; Verb 05 = Display data, Noun 09 = Alarm codes
; Crew uses this to request alarm code display on DSKY.
V05N09		VN	0509		# (SAME AS OCTAL 1211)

; OCT 1400 - Octal 1400 (decimal 768) utility constant
OCT1400		OCT	01400

; V06N22 - DSKY Verb 06, Noun 22 combination code  
; Verb 06 = Display decimal, Noun 22 = various display functions
V06N22		VN	0622

# MID5		OCT	1740		# PINBALL

; Bit mask for bits 2 through 10 (all bits except bit 1)
; OCT 1776 = binary 000001111111110 = decimal 1022
BITS2-10	OCT	1776

; Mask for lower 10 bits (bits 1-10, value 0-1023)
; OCT 1777 = binary 000001111111111 = decimal 1023
LOW10		OCT	1777

# Page 1202
# BIT11		OCT	02000
# 2K+3		OCT	2003		# PINBALL

; ============================================================================
; ERASABLE BANK (EBANK) AND PRIORITY (PRIO) CODES
; Memory bank selection and interrupt priority level definitions.
; EBANK codes select which 256-word bank of erasable (RAM) memory is active.
; PRIO codes set job priority levels in Executive scheduler (higher = more urgent).
; ============================================================================

; Combined opcode mask and bank setting for Bank 1
; OCT 2177 = low 7 bits mask (127) plus 2K bank offset
LOW7+2K		OCT	2177		# OP CODE MASK + BANK 1 FBANK SETTING.

; Erasable bank 5 selector - addresses erasable memory bank 5
; OCT 02400 = decimal 1280 = bank 5 starting address
EBANK5		OCT	02400

; Priority level 3 - medium-low priority for background tasks
; OCT 03000 = decimal 1536 - Executive uses for job scheduling
PRIO3		OCT	03000

; Erasable bank 7 selector - addresses erasable memory bank 7
; OCT 03400 = decimal 1792 = bank 7 starting address
EBANK7		OCT	03400

# LOW11		OCT	3777		# PINBALL
# BIT12		OCT	04000
# RELTAB	OCT	04025		# T4RUPT

; Priority level 5 - medium priority
PRIO5		OCT	05000

; Priority level 6 - medium-high priority
PRIO6		OCT	06000

; Priority level 7 - high priority
PRIO7		OCT	07000

# BIT13		OCT	10000
#		OCT	10003		# T4RUPT	RELTAB +1D
# 13,7,2	OCT	10102		# P20-P25 SUNDANCE

; ============================================================================
; HIGH-PRIORITY INTERRUPT LEVELS
; Priority codes 11-24 used for time-critical tasks including navigation
; updates, guidance computations, and display refresh. Higher numbers indicate
; greater urgency. Critical during landing when multiple systems compete for
; processor time (contributing to 1202 alarm conditions).
; ============================================================================

; Priority level 11 - above-medium priority
PRIO11		OCT	11000

# PRIO12	OCT	12000		# BANKCALL

; Priority level 13
PRIO13		OCT	13000

; Priority level 14
PRIO14		OCT	14000

#		OCT	14031		# T4RUPT	RELTAB +2D

; Priority level 15
PRIO15		OCT	15000

; Priority level 16
PRIO16		OCT	16000

# 85DEGS	DEC	.45556		# P20-P25 SUNDANCE	(OCTAL 16450)

; Priority level 17
PRIO17		OCT	17000

; Octal constant 17770 (decimal 8184)
OCT17770	OCT	17770

# BIT14		OCT	20000
#		OCT	20033		# T4RUPT	RELTAB +3D

; Priority level 21 - very high priority for critical real-time operations
PRIO21		OCT	21000
		BLOCK	03
		COUNT	03/FCONS

; Priority level 22 - used by service routines for housekeeping tasks
PRIO22		OCT	22000		# SERVICE ROUTINES

; Priority level 23
PRIO23		OCT	23000

; Priority level 24
PRIO24		OCT	24000

# 5/8+1		OCT	24001		# SINGLE PRECISION SUBROUTINES
#		OCT	24017		# T4RUPT	RELTAB +4D

; Priority level 25
PRIO25		OCT	25000

; Priority level 26
PRIO26		OCT	26000

; Priority level 27
PRIO27		OCT	27000

# CHRPRIO	OCT	30000		# PINBALL
#		OCT	30036		# T4RUPT	RELTAB +5D

; Priority level 31 - highest standard priority
PRIO31		OCT	31000

; C1/2 - Mathematical constant (π/4 = 0.7853134 radians = 45 degrees)
; Used in trigonometric calculations throughout guidance and navigation.
; Fundamental constant for angular computations in orbital mechanics.
C1/2		DEC	.7853134	#			(OCTAL 31103)

; Priority level 32
PRIO32		OCT	32000

; Priority level 33
PRIO33		OCT	33000

; Priority level 34
PRIO34		OCT	34000

#		OCT	34034		# T4RUPT	RELTAB +6D

; Priority level 35
PRIO35		OCT	35000

; Priority level 36
PRIO36		OCT	36000

# Page 1203

; Priority level 37 - critical priority for time-urgent operations
PRIO37		OCT	37000

; ============================================================================
; SPECIAL CONSTANTS AND INTERPRETER OPCODES
; Mathematical constants, interpreter operation codes, fractional values,
; and high/low bit masks used throughout computational routines.
; ============================================================================

; Fraction 63/64 + 1 = 1.984375 (OCT 37401)
; Near-unity scaling constant used in precision calculations.
63/64+1		OCT	37401

# MID7		OCT	37600		# PINBALL

; Special octal constants used for bit manipulation and testing
OCT37766	OCT	37766
OCT37774	OCT	37774
OCT37776	OCT	37776

# DPOSMAX	OCT	37777
# BIT15		OCT	40000
# OCT40001	OCT	40001		# INTERPRETER (CS 1 INSTRUCTION)

; DLOADCOD - Interpreter opcode for DLOAD operation (double-precision load)
; OCT 40014 = instruction code for loading double-precision values
DLOADCOD	OCT	40014

; DLOAD* - Interpreter indexed DLOAD operation code
; OCT 40015 = DLOAD with indirect addressing
DLOAD*		OCT	40015

#		OCT	40023		# T4RUPT	RELTAB +7D

; Bit 15 plus 6 offset (OCT 40040 = decimal 16416)
; Used for address calculations with high bit set
BIT15+6		OCT	40040

; Octal constant 40200 (decimal 16512)
; Used in specific computational contexts
OCT40200	OCT	40200

#		OCT	44035		# T4RUPT	RELTAB +8D
#		OCT	50037		# T4RUPT	RELTAB +9D
#		OCT	54000		# T4RUPT	RELTAB +10D

; Negative of bit 14: OCT 57777 = ~(bit 14) = all bits except bit 14 set
; Used as mask to clear bit 14 while preserving other bits
-BIT14		OCT	57777

# RELTAB11	OCT	60000		# T4RUPT

; C3/2 - Mathematical constant (-0.3216147)
; Used in trigonometric series expansions and coordinate transformations.
; Negative fractional value for specific computational algorithms.
C3/2		DEC	-.3216147	#			(OCTAL 65552)

; Bit mask with bits 13, 14, and 15 set (OCT 70000 = decimal 28672)
; Used to isolate or test the three highest bits
13,14,15	OCT	70000

; Fraction -1/8 = -0.125 (OCT 73777 in two's complement)
; Negative fractional constant for scaling operations
-1/8		OCT	73777

; High 4 bits mask (OCT 74000 = decimal 30720)
; Isolates the four highest bits (bits 12-15) when ANDed with value
HIGH4		OCT	74000

; -ENDERAS: Negative end of erasable memory marker (-2001 decimal)
; Used to mark boundary of erasable (RAM) memory region.
; Important for memory management and bounds checking.
-ENDERAS	DEC	-2001		#			(OCTAL 74056)

# HI5		OCT	76000		# PINBALL

; High 9 bits mask (OCT 77700 = decimal 32704)
; Isolates bits 7-15 (upper 9 bits of 15-bit word)
HIGH9		OCT	77700

# -ENDVAC	DEC	-45		# INTERPRETER		(OCTAL 77722)
# -OCT10	OCT	-10		#			(OCT 77767)
# NEG4		DEC	-4		#			(OCTAL 77773)

; ============================================================================
; COMMONLY USED NEGATIVE CONSTANTS
; Small negative integer values in two's complement representation.
; Used throughout AGC code for loop counters, decrements, and calculations.
; ============================================================================

; Negative three (-3 decimal = OCT 77774 in two's complement)
NEG3		DEC	-3

; Negative two (-2 decimal = OCT 77775 in two's complement)
NEG2		OCT	77775

; Negative one (-1 decimal = OCT 77776 in two's complement)
; Most frequently used negative constant throughout AGC code
NEGONE		DEC	-1

# Page 1204

# DEFINED BY EQUALS

# IT WOULD BE TO THE USERS ADVANTAGE TO OCCASIONALLY CHECK ANY OF THESE SYMBOLS IN ORDER TO PREVENT ANY
# ACCIDENTAL DEFINITION CHANGES.

; ============================================================================
; SYMBOL ALIASES AND ALTERNATE NAMES
; 
; The following block defines symbolic aliases using the "=" operator.
; These provide alternative names for constants already defined earlier,
; allowing different parts of the AGC code to use descriptive names
; appropriate to their context while referencing the same underlying value.
;
; COMMENT-ONLY READERS: These are like nicknames for numbers - the computer
;        needs the same values but different programs call them different names.
;
; CODE-ALONG READERS: These are assembler symbol aliases (not memory locations).
;        The assembler replaces each symbol with the value of the referenced
;        constant at assembly time. No memory overhead is incurred.
;
; CAUTION: As the original NASA comment states, users should verify these
;        definitions remain consistent to prevent accidental changes that
;        could affect multiple parts of the flight software.
; ============================================================================

; Negative integer aliases (-1 in various naming conventions)
MINUS1		=	NEG1
NEG1		=	NEGONE

; Small positive integer aliases (1-11)
; These provide readable names for bit values used as integer constants
ONE		=	BIT1		; 1 (bit 1 = 1)
TWO		=	BIT2		; 2 (bit 2 = 2)
THREE		=	OCTAL3		; 3
LOW2		=	THREE		; Alias for 3 (low 2 bits interpretation)
FOUR		=	BIT3		; 4 (bit 3 = 4)
SIX		=	REVCNT		; 6 (revolution counter context)
LOW3		=	SEVEN		; Alias for 7 (low 3 bits interpretation)
EIGHT		=	BIT4		; 8 (bit 4 = 8)
NINE		=	R2D1		; 9 (from pinball R2D1 constant)
TEN		=	BINCON		; 10 (binary conversion constant)
NOUTCON		=	ELEVEN		; 11 (noun output constant for display routines)

; Display-related octal value aliases
OCT23		=	VD1		; Octal 23 (verb display)
OCT25		=	MD1		; Octal 25 (mode display)

; Priority level aliases
; Priority values for job scheduling in EXEC and WAITLIST
; Higher bit positions = higher priority in AGC scheduling system
PRIO1		=	BIT10		; Priority level 1 (bit 10)
PRIO2		=	BIT11		; Priority level 2 (bit 11)
PRIO4		=	BIT12		; Priority level 4 (bit 12)
PRIO10		=	BIT13		; Priority level 10 (bit 13)
PRIO20		=	BIT14		; Priority level 20 (bit 14)
PRIO30		=	CHRPRIO		; Priority level 30 (character priority)

; EBANK (erasable bank) selection aliases
; Used for selecting which 256-word bank of erasable memory to access
EBANK3		=	OCT1400		; Erasable bank 3 selector
EBANK4		=	BIT11		; Erasable bank 4 selector (2K boundary)
EBANK6		=	PRIO3		; Erasable bank 6 selector

; Superbank configuration aliases
OCT120		=	SUPER101	; Octal 120 (Superbank 101 setting)
OCT140		=	SUPER110	; Octal 140 (Superbank 110 setting)

; Memory size and fractional value aliases
2K		=	BIT11		; 2048 words (bit 11 = 2K memory boundary)
QUARTER		=	BIT13		; 1/4 fractional value (0.25)
HALF		=	BIT14		; 1/2 fractional value (0.50)
POS1/2		=	HALF		; Positive one-half alias

; Interpreter-specific aliases
BIT13-14	=	PRIO30		# INTERPRETER USES IN PROCESSING STORECODE
OCT10001	=	CCSL		; Octal 10001 constant
OCT30002	=	TLOAD +1	; Octal 30002 (TLOAD operation + 1)
B12T14		=	PRIO34		; Bits 12 through 14 mask

; Maximum and special value aliases
NEGMAX		=	BIT15		; Negative maximum value (most negative: -16384)
VLOADCOD	=	BIT15		; VLOAD operation code (interpreter)
VLOAD*		=	OCT40001	; VLOAD with indexing operation code

; Miscellaneous aliases
OCT60000	=	RELTAB11	; Octal 60000 (relocatable table 11)
BANKMASK	=	HI5		; Bank field mask (high 5 bits for bank number extraction)

