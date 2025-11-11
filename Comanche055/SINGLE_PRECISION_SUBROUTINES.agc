# Copyright:	Public domain.
# Filename:	SINGLE_PRECISION_SUBROUTINES.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1207
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

# Page 1207

; ============================================================================
; FILE: SINGLE_PRECISION_SUBROUTINES.agc
; MODULE: CHIEFTAN Subsystem (Core OS)
; MISSION PHASE: all-phases
;
; TL;DR: Single-precision trigonometric subroutines (sine and cosine) using
;        polynomial approximation. Provides computationally efficient trig
;        functions for navigation, guidance, and attitude calculations
;        throughout all Apollo 11 mission phases.
;
; COMMENT-ONLY READERS: This file provides basic trigonometric functions
;        (sine and cosine) used throughout the mission for spacecraft
;        navigation and orientation calculations.
; CODE-ALONG READERS: Study polynomial approximation algorithms, argument
;        range reduction techniques, and single-precision fixed-point
;        trigonometric computation optimized for AGC constraints.
; ============================================================================

		BLOCK	02
# SINGLE PRECISION SINE AND COSINE

		COUNT	02/INTER

; ============================================================================
; SINGLE PRECISION TRIGONOMETRIC FUNCTIONS
;
; These subroutines compute sine and cosine values using polynomial
; approximation optimized for the AGC's single-precision arithmetic.
; Used extensively in navigation state updates, attitude computations,
; and coordinate frame transformations throughout the mission.
;
; For comment-only readers: The spacecraft computer needs to calculate
; angles for determining orientation in space and computing orbital paths.
; These mathematical functions provide those angle calculations.
;
; For code-along readers: Implementation uses Chebyshev polynomial
; approximation with argument range reduction to [-PI/4, +PI/4] for
; optimal accuracy within 15-bit single-precision constraints.
; ============================================================================

; SPCOS - Single Precision Cosine
; Entry: Accumulator (A) contains argument scaled at PI (1.0 = 180 degrees)
; Exit: A contains cosine result scaled at 1.0 (range -1.0 to +1.0)
; Method: Converts cos(x) to sin(x + PI/2) then calls SPSIN algorithm

SPCOS		AD	HALF		# ARGUMENTS SCALED AT PI
					# Add PI/2 (HALF = 0.5 at PI scaling)
					# Implements cos(x) = sin(x + PI/2)

; SPSIN - Single Precision Sine  
; Entry: Accumulator (A) contains argument scaled at PI
; Exit: A contains sine result scaled at 1.0
; Method: Range reduction followed by polynomial evaluation

SPSIN		TS	TEMK		# Store argument in temporary location
		TCF	SPT		# Transfer to range reduction logic
		CS	TEMK		# Complement for negative argument handling
; SPT - Sine/Cosine Argument Range Reduction
; Reduces arbitrary angle arguments to range [-PI/4, +PI/4] for accurate
; polynomial approximation. The AGC's fixed-point arithmetic requires
; keeping intermediate values within representable bounds.

SPT		DOUBLE			# Scale argument by 2
		TS	TEMK		# Store doubled argument
		TCF	POLLEY		# Branch if in primary range
		XCH	TEMK		# Exchange for range mapping
		INDEX	TEMK		# Indexed addressing for quadrant
		AD 	LIMITS		# Add quadrant-specific limit
		COM			# Complement for reflection
		AD	TEMK		# Add back argument
		TS	TEMK		# Store reduced argument
		TCF	POLLEY		# Proceed to polynomial evaluation
		TCF	ARG90		# Handle special 90-degree case
; POLLEY - Polynomial Evaluation for Sine/Cosine
; Computes sine using Chebyshev polynomial approximation:
; sin(x) ≈ x * (C1/2 + x² * (C3/2 + x² * C5/2))
; This nested form (Horner's method) minimizes multiplications.
;
; For comment-only readers: This routine performs the mathematical
; calculation of the sine function using a formula that approximates
; the true sine value with high accuracy.
;
; For code-along readers: Coefficients C1/2, C3/2, C5/2 are scaled
; Chebyshev polynomial coefficients stored in fixed memory. The
; algorithm computes x², then evaluates the nested polynomial form
; from innermost to outermost terms for numerical stability.

POLLEY		EXTEND			# Enable multiply mode
		MP	TEMK		# Multiply A by argument (x)
		TS	SQ		# Store x² (argument squared)
		EXTEND			# Enable multiply mode
		MP	C5/2		# Multiply by 5th-order coefficient
		AD	C3/2		# Add 3rd-order coefficient
		EXTEND			# Enable multiply mode
		MP	SQ		# Multiply by x²
		AD	C1/2		# Add 1st-order coefficient
		EXTEND			# Enable multiply mode
		MP	TEMK		# Multiply by x (final scaling)
		DDOUBL			# Double for proper scaling
		TS	TEMK		# Store final result
		TC	Q		# Return to caller (Q = return address)
; ARG90 - Special Case Handler for 90-Degree Arguments
; When argument equals exactly ±90 degrees (±PI/2), polynomial approximation
; becomes unstable. This routine returns exact values: sin(90°) = +1,
; sin(-90°) = -1, avoiding numerical precision issues.

ARG90		INDEX	A		# Use A as index for sign determination
		CS	LIMITS		# Complement of limit gives ±1
		TC	Q		# RESULT SCALED AT 1
					# Return exact ±1.0 for 90-degree args

; ============================================================================
; HISTORICAL NOTE: Single Precision Square Root (SPROOT)
;
; A single-precision square root subroutine originally existed in this file
; but was removed in Revision 51 of the master AGC program. The Assembly
; Contractor retained the implementation on punched cards for reference.
;
; For comment-only readers: An earlier version included a square root
; function, but it was removed before the Apollo 11 mission, likely because
; the interpretive language provided sufficient square root capability.
;
; For code-along readers: The deletion suggests that double-precision square
; root operations via the interpreter (SQRT opcode) provided adequate
; accuracy for mission requirements, making single-precision implementation
; redundant. This reflects the AGC development team's ongoing optimization
; to fit functionality within the 36K ROM constraint.
; ============================================================================
