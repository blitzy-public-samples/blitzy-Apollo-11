# Copyright:	Public domain.
# Filename:	INTERPRETIVE_CONSTANT.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1100-1101
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

; ============================================================================
; FILE: INTERPRETIVE_CONSTANT.agc
; MODULE: Core Operating System - Interpreter Support
; MISSION PHASE: All phases (foundational constants used throughout mission)
;
; TL;DR: Defines mathematical and vector constants specifically formatted for
;        the interpretive language virtual machine. These pre-defined values
;        enable high-level vector mathematics, coordinate transformations, and
;        computational operations used throughout guidance, navigation, and
;        control programs during all mission phases.
;
; COMMENT-ONLY READERS: Foundation constants that enable the complex orbital
;        calculations guiding the Lunar Module through descent and ascent.
; CODE-ALONG READERS: Study constant scaling conventions (0.5 = 1.0), double-
;        precision format (2DEC), and memory bank organization for interpreter.
; ============================================================================

# Page 1100
; ============================================================================
; INTERPRETIVE CONSTANTS - FIRST MEMORY BANK (INTPRET1)
;
; These constants are stored in the first interpreter memory bank and provide
; fundamental mathematical values for interpretive language programs. The AGC
; lacks floating-point hardware, so all constants use fixed-point scaled
; representation with double-precision format (2DEC = two-word decimal).
; ============================================================================
;
		SETLOC	INTPRET1
		BANK

		COUNT*	$$/ICONS
;
; Mathematical constant: one quarter (0.25) in double-precision format.
; Used throughout guidance equations for fractional computations.
;
DP1/4TH		2DEC	.25

; Unit vector components representing standard coordinate axes. In AGC scaled
; arithmetic, 0.5 represents the value 1.0 (scaling by 2^1). These unit vectors
; define X, Y, Z axes for coordinate frame transformations used in navigation.
;
UNITZ		2DEC	0		; Z-axis unit vector: (0, 0, 1) third component

UNITY		2DEC	0		; Y-axis unit vector: (0, 1, 0) second component

UNITX		2DEC	.5		; X-axis unit vector: (1, 0, 0) first component
					; (0.5 in storage represents 1.0 in computation)

; Zero vector definition: three consecutive zero values representing the
; null vector (0, 0, 0) used to initialize position/velocity calculations.
;
ZEROVECS	2DEC	0		; Zero vector first component

		2DEC	0		; Zero vector second component

		2DEC	0		; Zero vector third component

; Double-precision constants for common mathematical operations.
;
DPHALF		=	UNITX		; Half (0.5): Alias for UNITX constant
DPPOSMAX	OCT	37777		; Maximum positive double-precision value
		OCT	37777		; (both words set to maximum positive octal)

# Page 1101
# INTERPRETIVE CONSTANTS IN THE OTHER HALF-MEMORY

; ============================================================================
; INTERPRETIVE CONSTANTS - SECOND MEMORY BANK (INTPRET2)
;
; Duplicate constant definitions in the second interpreter memory bank. The
; AGC's memory architecture requires constants in both banks to enable
; efficient access without bank-switching overhead during time-critical
; interpretive computations (guidance during descent, ascent targeting).
; ============================================================================
;
		SETLOC	INTPRET2
		BANK

		COUNT*	$$/ICONS
;
; Unit vector components (duplicate definitions for second memory bank).
; Enable fast access during vector operations without memory bank switching.
;
ZUNIT		2DEC	0		; Z-axis unit vector: third component

YUNIT		2DEC	0		; Y-axis unit vector: second component

XUNIT		2DEC	.5		; X-axis unit vector: first component (0.5 = 1.0)

; Zero vector for second memory bank: Initialize state vectors and
; computational scratch areas during orbital integration routines.
;
ZEROVEC		2DEC	0		; Zero vector first component

		2DEC	0		; Zero vector second component

		2DEC	0		; Zero vector third component

; Special interpretive constants for computational algorithms. The ordering
; of the next three constants (-0, -6, -12) is CRITICAL and must be preserved
; for proper interpreter operation during mathematical function evaluation.
;
		OCT	77777		# -0 (negative zero in ones-complement AGC arithmetic)
					# -0, -6, -12 MUST REMAIN IN THIS ORDER
DFC-6		DEC	-6		# Decimal constant -6 for scaling operations
DFC-12		DEC	-12		# Decimal constant -12 for power-of-two shifts

; Maximum load values for integration routines. These constants define
; computational limits during orbital trajectory integration (Encke method).
; CRITICAL: LODPMAX and LODPMAX1 must remain adjacent and identical for
; numerical integration stability during powered flight guidance.
;
LODPMAX		2OCT	3777737777	# THESE TWO CONSTANTS MUST REMAIN

LODPMAX1	2OCT	3777737777	# ADJACENT AND THE SAME FOR INTEGRATION

; Alias definitions for common mathematical operations in second bank.
;
ZERODP		=	ZEROVEC		; Zero double-precision: Alias for ZEROVEC
HALFDP		=	XUNIT		; Half double-precision: Alias for XUNIT (0.5)


