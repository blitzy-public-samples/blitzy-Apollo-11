# Copyright:	Public domain.
# Filename:	MYSUBS.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	999-1001
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-20 RSB	Corrections: EBANK= changed from MPAC to KMPAC.
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

# Page 999
; ============================================================================
; FILE: MYSUBS.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: all-phases
;
; TL;DR: Utility subroutines for control system calculations including
;        mathematical operations, coordinate transformations, and common
;        functions used by TVC and DAP modules. Provides shared computational
;        routines to autopilot and guidance systems.
;
; COMMENT-ONLY READERS: This file contains helper math functions used by
;        other control and steering programs.
; CODE-ALONG READERS: Study utility subroutine implementations for control
;        system support functions.
; ============================================================================

		BANK	20
		SETLOC	MYSUBS
		BANK

		EBANK=	KMPAC
SPCOS1		EQUALS	SPCOS
SPSIN1		EQUALS	SPSIN
SPCOS2		EQUALS	SPCOS
SPSIN2		EQUALS	SPSIN


		COUNT	21/DAPMS

; ============================================================================
; SMALLMP - ONE AND ONE HALF PRECISION MULTIPLICATION ROUTINE
;
; This subroutine performs multiplication with extended precision, providing
; greater accuracy than standard single-precision operations. Used throughout
; the control system calculations where precision is critical for spacecraft
; attitude control and trajectory computations.
;
; OPERATION: Multiplies the value in A register by a double-precision value
; in KMPAC (KMPAC and KMPAC+1), producing a double-precision result. The
; algorithm splits the multiplication into two parts (AX and AY) and
; combines them using double-precision addition.
;
; INPUT:  A register contains multiplier (single precision)
;         KMPAC, KMPAC+1 contain multiplicand (double precision)
; OUTPUT: KMPAC, KMPAC+1 contain product (double precision)
; TIMING: 14 machine cycles
; ============================================================================
# ONE AND ONE HALF PRECISION MULTIPLICATION ROUTINE

SMALLMP		TS	KMPTEMP		# A(X+Y) - Store multiplier for later use
		EXTEND
		MP	KMPAC 	+1	# Multiply A by lower word of KMPAC
		TS	KMPAC 	+1	# AY - Store lower product term
		CAF	ZERO		# Clear A register
		XCH	KMPAC		# Exchange with upper word of KMPAC
		EXTEND
		MP	KMPTEMP		# AX - Multiply A by original multiplier
		DAS	KMPAC		# AX+AY - Double precision add, final result
		TC	Q		# Return to caller

; ============================================================================
; DPADD - DOUBLE PRECISION ANGLE ADDITION WITH OVERFLOW HANDLING
;
; This subroutine adds a double-precision angle to the accumulator KMPAC,
; with special handling for angular overflow conditions. Critical for
; attitude control computations where angles wrap around at 180 degrees.
; During spacecraft maneuvers, this routine ensures angle calculations
; remain within valid ranges even when rotation exceeds 360 degrees.
;
; OPERATION: Performs double-precision addition of angle in A,L registers
; to KMPAC. If overflow occurs (angle exceeds ±180 degrees), the routine
; wraps the angle back into valid range by adding or subtracting 360 degrees.
; This prevents angle representation errors during continuous rotations.
;
; INPUT:  A, L registers contain angle to add (scaled by 180 degrees)
;         KMPAC, KMPAC+1 contain current angle accumulator
; OUTPUT: KMPAC, KMPAC+1 contain sum with overflow correction
; TIMING: 6 machine cycles (normal), 22 machine cycles (with overflow)
;
; ANGLE SCALING: Angles are represented as fractions of 180 degrees.
; +1.0 = +180 degrees, -1.0 = -180 degrees, 0.5 = +90 degrees, etc.
; ============================================================================
# SUBROUTINE FOR DOUBLE PRECISION ADDITIONS OF ANGLES
# A AND L CONTAIN A DP(1S) ANGLE SCALED BY 180 DEGS TO BE ADDED TO KMPAC.
# RESULT IS PLACED IN KMPAC.  TIMING = 6 MCT (22 MCT ON OVERFLOW)

DPADD		DAS	KMPAC		# Double precision add to KMPAC
		EXTEND
		BZF	TSK 	+1	# NO OVERFLOW - Branch if no overflow occurred
		CCS	KMPAC		# Check sign of upper word to determine overflow direction
		TCF	DPADD+		# + OVERFLOW - Positive overflow, angle > +180 deg
		TCF	+2		# Skip negative overflow case
		TCF	DPADD-		# - OVERFLOW - Negative overflow, angle < -180 deg
		CCS	KMPAC 	+1	# Upper word was zero, check lower word
		TCF	DPADD2+		# UPPER = 0, LOWER + (small positive overflow)
		TCF	+2		# Skip complement case
		COM			# UPPER = 0, LOWER - (complement for negative)
		AD	POSMAX		# LOWER = 0, A=0 - Handle zero case
		TS	KMPAC 	+1	# CAN NOT OVERFLOW - Store corrected lower word
		CA	POSMAX		# UPPER WAS = 0 - Set upper word to max
TSK		TS	KMPAC		# Store corrected upper word
		TC	Q		# Return to caller with corrected angle

DPADD+		AD	NEGMAX		# KMPAC GREATER THAN 0 - Wrap positive overflow
		TCF	TSK		# Store and return

# Page 1000
; Negative overflow correction - angle wrapped below -180 degrees
DPADD-		COM			# Complement the negative overflow
		AD	POSMAX		# KMPAC LESS THAN 0 - Add 360 deg correction
		TCF	TSK		# Store corrected angle and return

; Small positive overflow when upper word was zero
DPADD2+		AD	NEGMAX		# CAN NOT OVERFLOW - Wrap small positive excess
		TS	KMPAC 	+1	# Store corrected lower word
		CA	NEGMAX		# UPPER WAS = 0 - Set upper to negative max
		TCF	TSK		# Store and return

# Page 1001 (empty page)
