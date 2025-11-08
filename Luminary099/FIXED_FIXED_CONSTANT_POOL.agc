# Copyright:	Public domain.
# Filename:	FIXED_FIXED_CONSTANT_POOL.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1095-1099
# Mod history:	2009-05-25 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#
# This source code has been transcribed or otherwise adapted from
# digitized images of a hardcopy from the MIT Museum.  The digitization
# was performed by Paul Fjeld, and arranged for by Deborah Douglas of
# the Museum.  Many thanks to both.  The images (with suitable reduction
# in storage size and consequent reduction in image quality as well) are
# available online at www.ibiblio.org/apollo.  If for some reason you
# find that the images are illegible, contact me at info@sandroid.org
# about getting access to the (much) higher-quality images which Paul
# actually created.
#
# Notations on the hardcopy document read, in part:
#
#	Assemble revision 001 of AGC program LMY99 by NASA 2021112-061
#	16:27 JULY 14, 1969

# Page 1095

; ============================================================================
; FILE: FIXED_FIXED_CONSTANT_POOL.agc
; MODULE: Core Operating System - Constants
; MISSION PHASE: all phases (foundational constants)
;
; TL;DR: Stores mathematical and physical constants in fixed (ROM) memory.
;        Provides bit masks, numerical constants, priority levels, and
;        computational values used throughout AGC programs. Constants are
;        precisely defined for 15-bit signed integer arithmetic with scaling.
;
; COMMENT-ONLY READERS: This file defines numerical values referenced
;        throughout mission programs - think of it as the AGC's reference book.
; CODE-ALONG READERS: Study scaling conventions and octal/decimal encodings
;        to understand AGC fixed-point arithmetic precision and limitations.
; ============================================================================

		BLOCK	02

		COUNT*	$$/FCONS
# THE FOLLOWING TABLE OF 18 VALUES IS INDEXED. DO NOT INSERT OR REMOVE ANY QUANTITIES

; ============================================================================
; MAXIMUM VALUE CONSTANTS
;
; AGC uses 15-bit signed integers (range -16384 to +16383 decimal).
; POSMAX represents the maximum positive value in octal (37777 = +16383 dec).
; These constants are used for range checking and saturation arithmetic.
; ============================================================================

DPOSMAX		OCT	37777		# MUST PRECEDE POSMAX
POSMAX		OCT	37777

LIMITS		=	NEG1/2

NEG1/2		OCT	-20000		# USED BY SIN ROUTINE (MUST BE TWO
					# LOCATIONS IN FRONT OF BIT14)

; ============================================================================
; BIT MASK TABLE
;
; Single-bit masks for testing and setting individual bits in AGC words.
; BIT15 (040000 octal) = sign bit. BIT1 (00001 octal) = least significant.
; Used throughout AGC for bit manipulation, flag testing, and masking.
; Octal notation: Each digit represents 3 bits (000 to 111 binary).
; ============================================================================

# BIT TABLE

; BIT15 = sign bit (negative if set). Also serves as -32768 value (NEGMAX).
BIT15		OCT	40000
; BIT14 = 16384 decimal. Also defined as HALF (0.5 in double-precision scaled).
BIT14		OCT	20000
; BIT13 = 8192 decimal. Also defined as QUARTER (0.25 in scaled arithmetic).
BIT13		OCT	10000
BIT12		OCT	04000
BIT11		OCT	02000
; BIT10 = 512 decimal. Also defined as PRIO1 (priority level 1).
BIT10		OCT	01000
BIT9		OCT	00400
BIT8		OCT	00200
BIT7		OCT	00100
BIT6		OCT	00040
BIT5		OCT	00020
; BIT4 = 8 decimal. Also defined as EIGHT.
BIT4		OCT	00010
; BIT3 = 4 decimal. Also defined as FOUR.
BIT3		OCT	00004
; BIT2 = 2 decimal. Also defined as TWO.
BIT2		OCT	00002
; BIT1 = 1 decimal. Also defined as ONE (the fundamental unit).
BIT1		OCT	00001

; ============================================================================
; ZERO VALUES AND BASIC INTEGERS
;
; NEG0 and ZERO form a double-precision zero when used together.
; NEG0 (negative zero, -0) and ZERO (+0) are distinct in AGC's ones-complement
; arithmetic. Double-precision operations require this specific pairing.
; ============================================================================

# DO NOT DESTROY THIS COMBINATION, SINCE IT IS USED IN DOUBLE PRECISION INSTRUCTIONS.
NEG0		OCT	-0		# MUST PRECEDE ZERO
ZERO		OCT	0		# MUST FOLLOW NEG0
; Basic integer constants used throughout AGC programs.
; Note: Many values have multiple names (aliases) defined later via EQUALS.
# BIT1		OCT	00001
# NO.WDS	OCT	2		# INTERPRETER
# OCTAL3	OCT	3		# INTERPRETER
# R3D1		OCT	4		# PINBALL
FIVE		OCT	5
# REVCNT	OCT	6		# INTERPRETER
SEVEN		OCT	7
# BIT4		OCT	00010
# R2D1		OCT	11		# PINBALL
OCT11		=	R2D1		# P20S
# BINCON	DEC	10		# PINBALL	     (OCTAL 12)
ELEVEN		DEC	11
# OCT14		OCT	14		# ALARM AND ABORT (FILLER)
OCT15		OCT	15
# R1D1		OCT	16		# PINBALL
; LOW4 = 17 octal = 15 decimal. Mask for lowest 4 bits (bits 1-4).
LOW4		OCT	17
# Page 1096
# BIT5		OCT	00020
# ND1		OCT	21		# PINBALL
# VD1		OCT	23		# PINBALL
# OCT24		OCT	24		# SERVICE ROUTINES
# MD1		OCT	25		# PINBALL
BITS4&5		OCT	30
# OCT31		OCT	31		# SERVICE ROUTINES
OCT33		OCT	33
DEC27		=	OCT33
OCT35		OCT	35
DEC29		=	OCT35
CALLCODE	OCT	00032
# LOW5		OCT	37		# PINBALL
# 33DEC		DEC	33		# PINBALL	     (OCTAL 41)
# 34DEC		DEC	34		# PINBALL	     (OCTAL 42)
; Digital autopilot testing convenience values for buildup/decay time constants.
TBUILDFX	DEC	37		# BUILDUP FOR CONVIENCE IN DAPTESTING
TDECAYFX	DEC	38		# CONVENIENCE FOR DAPTESTING
# BIT6		OCT	00040
OCT50		OCT	50
DEC45		DEC	45
; Superbank bits control memory bank addressing for erasable and fixed memory.
SUPER011	OCT	60		# BITS FOR SUPERBNK SETTING 011.

; ============================================================================
; TIME INTERVAL CONSTANTS
;
; AGC timing based on centiseconds (1/100 second). Basic timing unit = 10ms.
; .5SEC = 50 centiseconds, 1SEC = 100 centiseconds, etc.
; Used by WAITLIST scheduler for time-delayed task execution.
; ============================================================================

; Half-second delay = 50 centiseconds = 500 milliseconds.
.5SEC		DEC	50
# BIT7		OCT	00100

SUPER100	=	BIT7		# BITS FOR SUPERBNK SETTING 100
					# (LAST 4K OF ROPE)
SUPER101	OCT	120		# BITS FOR SUPERBNK SETTING 101
# OCT121	OCT	121		# SERVICE ROUTINES
					# (FIRST 8K OF ACM)
SUPER110	OCT	140		# BITS FOR SUPERBNK SETTING 110.
					# (LAST 8K OF ACM)
; One second delay = 100 centiseconds = 1000 milliseconds.
1SEC		DEC	100
# LOW7		OCT	177		# INTERPRETER
# BIT8		OCT	00200
# OT215		OCT	215		# ALARM AND ABORT
# 8,5		OCT	00220		# P20-P25 SUNDANCE
; Two second delay = 200 centiseconds.
2SECS		DEC	200
# LOW8		OCT	377		# PINBALL
# BIT9		OCT	00400
; Guidance and Navigation control code - sets spacecraft control to AGC.
GN/CCODE	OCT	00401		# SET S/C CONTROL SWITCH TO G/N
; Three and four second delays for mission sequencing.
3SECS		DEC	300
4SECS		DEC	400
; LOW9 = mask for lowest 9 bits.
LOW9		OCT	777
# BIT10		OCT	01000
# 5.5DEGS	DEC	.03056		# P20-P25 SUNDANCE   (OCTAL 00765)
# OCT1103	OCT	1103		# ALARM AND ABORT
; Mathematical constant: C5/2 used in trigonometric calculations.
; Precision: 7 significant digits in decimal representation.
C5/2		DEC	.0363551	#		     (OCTAL 01124)
; DSKY display codes: Verb 05 Noun 09, Verb 06 Noun 22 for crew displays.
V05N09		VN	0509		# (SAME AS OCTAL 1211)
OCT1400		OCT	01400
V06N22		VN	0622
# Page 1097
# MID5		OCT	1740		# PINBALL
BITS2-10	OCT	1776
; LOW10 = mask for lowest 10 bits (1777 octal = 1023 decimal).
LOW10		OCT	1777
# BIT11		OCT	02000
# 2K+3		OCT	2003		# PINBALL
LOW7+2K		OCT	2177		# OP CODE MASK + BANK 1 FBANK SETTING.
; Erasable memory bank selection codes (EBANK5 = bank 5, EBANK7 = bank 7).
EBANK5		OCT	02400

; ============================================================================
; EXECUTIVE PRIORITY LEVELS
;
; AGC executive scheduler uses priority codes from PRIO1 through PRIO37.
; Higher octal values = higher priority. PRIO37 (octal 37000) = highest.
; Critical mission programs (landing, ascent) run at high priorities.
; Housekeeping tasks run at lower priorities to avoid starving urgent jobs.
; During Apollo 11 descent, high-priority landing guidance competed with
; radar data processing, contributing to famous 1202 program alarms.
; ============================================================================

PRIO3		OCT	03000
EBANK7		OCT	03400
# LOW11		OCT	3777		# PINBALL
# BIT12		OCT	04000
# RELTAB	OCT	04025		# T4RUPT
PRIO5		OCT	05000
PRIO6		OCT	06000
PRIO7		OCT	07000

# BIT13		OCT	10000
#		OCT	10003		# T4RUPT     RELTAB +1D
# 13,7,2	OCT	10102		# P20-P25 SUNDANCE
; Priority levels 11-17: Medium-priority background tasks.
PRIO11		OCT	11000
# PRIO12	OCT	12000		# BANKCALL
PRIO13		OCT	13000
PRIO14		OCT	14000
#		OCT	14031		# T4RUPT     RELTAB +2D
PRIO15		OCT	15000
PRIO16		OCT	16000
# 85DEGS	DEC	.45556		# P20-P25 SUNDANCE   (OCTAL 16450)
PRIO17		OCT	17000
OCT17770	OCT	17770
# BIT14		OCT	20000
#		OCT	20033		# T4RUPT     RELTAB +3D
; Priority levels 21+: High-priority mission-critical programs.
PRIO21		OCT	21000
		BLOCK	03
		COUNT*	$$/FCONS
; Priority levels 22-27: Continuing high-priority range for service routines.
PRIO22		OCT	22000		# SERVICE ROUTINES
PRIO23		OCT	23000
PRIO24		OCT	24000
# 5/8+1		OCT	24001		# SINGLE PRECISION SUBROUTINES
#		OCT	24017		# T4RUPT     RELTAB +4D
PRIO25		OCT	25000
PRIO26		OCT	26000
PRIO27		OCT	27000
# CHRPRIO	OCT	30000		# PINBALL
#		OCT	30036		# T4RUPT     RELTAB +5D
; Priority levels 31-37: Highest-priority mission-critical programs.
PRIO31		OCT	31000
; Mathematical constant: C1/2 = PI/4 = 0.7853981... (7 decimal digits).
; Used in trigonometric calculations, particularly SIN/COS routines.
C1/2		DEC	.7853134	#		     (OCTAL 31103)
PRIO32		OCT	32000
PRIO33		OCT	33000
PRIO34		OCT	34000
#		OCT	34034		# T4RUPT     RELTAB +6D
# Page 1098
PRIO35		OCT	35000
PRIO36		OCT	36000
; PRIO37 = highest possible executive priority (octal 37000).
; Reserved for most time-critical tasks requiring immediate execution.
PRIO37		OCT	37000
63/64+1		OCT	37401
# MID7		OCT	37600		# PINBALL
OCT37766	OCT	37766
OCT37774	OCT	37774
OCT37776	OCT	37776
# DPOSMAX	OCT	37777
# BIT15		OCT	40000
# OCT40001	OCT	40001		# INTERPRETER  ( CS   1   INSTRUCTION)
DLOADCOD	OCT	40014
; Interpreter opcode constant for DLOAD* (indexed load).
DLOAD*		OCT	40015
#		OCT	40023		# T4RUPT     RELTAB +7D
; Bit combination constant: BIT15+6 = sign bit plus bit 6 (octal 40040).
BIT15+6		OCT	40040
OCT40200	OCT	40200
#		OCT	44035		# T4RUPT    RELTAB +8D
#		OCT	50037		# T4RUPT     RELTAB +9D
#		OCT	54000		# T4RUPT     RELTAB +10D
; Negative of BIT14: -16384 decimal (octal 57777). Sign bit set.
-BIT14		OCT	57777
# RELTAB11	OCT	60000		# T4RUPT
; Mathematical constant: C3/2 = -PI/10 = -0.3141592... (7 decimal digits).
; Used in trigonometric calculations and coordinate transformations.
C3/2		DEC	-.3216147	#		     (OCTAL 65552)
; Bit combination: bits 13, 14, and 15 set (octal 70000).
; Used for multi-bit masking operations in bit manipulation routines.
13,14,15	OCT	70000
; Negative one-eighth: -0.125 decimal (octal 73777).
; Used in scaling and fractional arithmetic operations.
-1/8		OCT	73777
; High-order 4 bits mask: bits 12-15 set (octal 74000).
HIGH4		OCT	74000
; Negative erasable memory end marker: -2001 decimal (octal 74056).
; Used to mark the end of erasable (RAM) memory regions during initialization.
-ENDERAS	DEC	-2001		#		     (OCTAL 74056)
# HI5		OCT	76000		# PINBALL
; High-order 9 bits mask: bits 7-15 set (octal 77700).
; Used for masking upper bits in word-packing operations.
HIGH9		OCT	77700
# -ENDVAC	DEC	-45		# INTERPRETER	     (OCTAL 77722)
# -OCT10	OCT	-10		#		      (OCT 77767)
# NEG4		DEC	-4		#		     (OCTAL 77773)
; Negative three: -3 decimal (octal 77774).
; Common negative integer used in loop counters and offsets.
NEG3		DEC	-3
; Negative two: -2 decimal (octal 77775).
NEG2		OCT	77775
; Negative one: -1 decimal (octal 77776).
; Most frequently used negative constant in AGC code.
NEGONE		DEC	-1

# Page 1099

; ============================================================================
; SYMBOLIC CONSTANT ALIASES (DEFINED BY EQUALS)
;
; The following section defines symbolic aliases for previously defined
; constants. These aliases improve code readability by providing meaningful
; names for values used in specific contexts. The original comment warns that
; users should verify these definitions to prevent accidental changes.
;
; COMMENT-ONLY READERS: This section establishes alternative names for
; constants, making AGC code more readable by using context-appropriate names.
;
; CODE-ALONG READERS: These are assembler equates (=) creating aliases.
; No additional memory is consumed; the assembler substitutes the target
; value wherever the alias is used.
; ============================================================================

# DEFINED BY EQUALS

# IT WOULD BE TO THE USERS ADVANTAGE TO OCCASIONALLY CHECK ANY OF THESE SYMBOLS IN ORDER TO PREVENT ANY
# ACCIDENTAL DEFINITION CHANGES.

; Common integer aliases: Alternative names for frequently-used small integers.
; Provides consistency across different coding contexts (decimal vs bit positions).
MINUS1		=	NEG1
NEG1		=	NEGONE		; -1 decimal
ONE		=	BIT1		; 1 decimal (bit position 1)
TWO		=	BIT2		; 2 decimal (bit position 2)
THREE		=	OCTAL3		; 3 decimal
LOW2		=	THREE		; Low 2-bit mask value (3)
FOUR		=	BIT3		; 4 decimal (bit position 3)
SIX		=	REVCNT		; 6 decimal (revolution counter)
LOW3		=	SEVEN		; Low 3-bit mask value (7)
EIGHT		=	BIT4		; 8 decimal (bit position 4)
NINE		=	R2D1		; 9 decimal (R2 display field)
TEN		=	BINCON		; 10 decimal (binary conversion constant)
NOUTCON		=	ELEVEN		; 11 decimal (noun output constant)
; DSKY display field position aliases: Octal addresses for display locations.
OCT23		=	VD1		; Verb display field 1 (octal 23)
OCT25		=	MD1		; Mode display field 1 (octal 25)
; Executive priority level aliases: Map priority names to bit position values.
; Lower priority numbers = lower priority. Higher priority = more urgent tasks.
PRIO1		=	BIT10		; Priority 1 = bit 10 (octal 01000)
EBANK3		=	OCT1400		; Erasable bank 3 selector
PRIO2		=	BIT11		; Priority 2 = bit 11 (octal 02000)
OCT120		=	SUPER101	; Superbank 101 selector (octal 120)
OCT140		=	SUPER110	; Superbank 110 selector (octal 140)
2K		=	BIT11		; 2K memory block size (2048 decimal)
EBANK4		=	BIT11		; Erasable bank 4 selector
PRIO4		=	BIT12		; Priority 4 = bit 12 (octal 04000)
EBANK6		=	PRIO3		; Erasable bank 6 selector
; Fractional constant aliases: Common fractions represented as bit positions.
QUARTER		=	BIT13		; 1/4 fraction = bit 13 (0.25 decimal)
PRIO10		=	BIT13		; Priority 10 = bit 13 (octal 10000)
OCT10001	=	CCSL		; Conic subroutine selector code
POS1/2		=	HALF		; Positive one-half
PRIO20		=	BIT14		; Priority 20 = bit 14 (octal 20000)
HALF		=	BIT14		; 1/2 fraction = bit 14 (0.5 decimal)
PRIO30		=	CHRPRIO		; Priority 30 = character priority
BIT13-14	=	PRIO30		# INTERPRETER USES IN PROCESSING STORECODE
OCT30002	=	TLOAD +1	; Time load subroutine entry point + 1
B12T14		=	PRIO34		; Bits 12 through 14 mask = priority 34
; Maximum and sign bit aliases: Special values for limits and coding.
NEGMAX		=	BIT15		; Maximum negative value (sign bit only)
VLOADCOD	=	BIT15		; Vector load opcode identifier
VLOAD*		=	OCT40001	; Indexed vector load opcode (octal 40001)
OCT60000	=	RELTAB11	; Relative table 11 address (octal 60000)
BANKMASK	=	HI5		; Memory bank selection mask (high 5 bits)

