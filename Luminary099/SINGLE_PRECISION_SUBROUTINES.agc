# Copyright:	Public domain.
# Filename:	SINGLE_PRECISION_SUBROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1102
# Mod history:	2009-05-25 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2010-12-31 JL	Fixed page number comment.
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

# Page 1102
; ============================================================================
; FILE: SINGLE_PRECISION_SUBROUTINES.agc
; MODULE: Core Operating System - Mathematical Utilities
; MISSION PHASE: All phases (utility subroutines)
;
; TL;DR: Implements fast single-precision trigonometric functions (sine and
;        cosine) using polynomial approximation. Provides performance-
;        optimized alternatives to double-precision interpretive math where
;        reduced precision is acceptable. Used throughout navigation and
;        guidance computations when speed is more critical than maximum
;        accuracy.
;
; COMMENT-ONLY READERS: These mathematical utilities enable the guidance
;        computer to perform rapid trigonometric calculations during time-
;        critical mission phases like descent and ascent.
; CODE-ALONG READERS: Study polynomial evaluation techniques and AGC single-
;        precision arithmetic. Note precision vs execution time tradeoffs.
; ============================================================================
		BLOCK	02

# SINGLE PRECISION SINE AND COSINE

; ============================================================================
; SINGLE PRECISION TRIGONOMETRIC FUNCTIONS
;
; The guidance computer frequently needs sine and cosine values during
; navigation and attitude control computations. These single-precision
; routines provide rapid trigonometric calculations using polynomial
; approximation, trading some accuracy for significant speed improvement
; over double-precision interpretive math. During the lunar landing, these
; fast calculations help maintain real-time guidance loop performance.
;
; SPCOS and SPSIN implement fifth-order polynomial approximations over the
; range 0 to PI/2, with automatic range reduction for larger arguments.
; Arguments are scaled at PI (1.0 = 180 degrees). Results scaled at 1.0.
; ============================================================================

		COUNT*	$$/INTER
SPCOS		AD	HALF		# ARGUMENTS SCALED AT PI
; SPCOS Entry Point: Compute single-precision cosine
; The cosine function is computed by adding PI/2 (HALF at PI scaling) to the
; argument and calling the sine routine, using the identity cos(x) = sin(x + PI/2).

SPSIN		TS	TEMK
; SPSIN Entry Point: Compute single-precision sine
; Input: Argument in A register, scaled at PI (1.0 = 180 degrees)
; Output: Sine value in A register, scaled at 1.0 (full scale = ±1.0)
; Temporary storage: TEMK holds argument during computation
; Method: Fifth-order polynomial approximation after range reduction

		TCF	SPT
		CS	TEMK
SPT		DOUBLE
; Range reduction to 0-PI/2 quadrant
; The polynomial approximation is accurate only over 0 to PI/2. Arguments
; outside this range are reduced by complementing and doubling operations
; that map the full circle to the first quadrant while preserving correct
; sign and magnitude relationships.

		TS	TEMK
		TCF	POLLEY
		XCH	TEMK
		INDEX	TEMK
		AD 	LIMITS
		COM
		AD	TEMK
		TS	TEMK
		TCF	POLLEY
		TCF	ARG90
; ============================================================================
; POLYNOMIAL EVALUATION SECTION
;
; Computes sine using fifth-order polynomial approximation:
; sin(x) ≈ C1*x + C3*x^3 + C5*x^5
;
; This polynomial provides excellent accuracy over the range 0 to PI/2
; while requiring only single-precision multiply operations. The AGC's
; hardware multiply instruction executes in two memory cycles (about 
; 23.4 microseconds), making this approach much faster than the interpretive
; double-precision trigonometric routines while maintaining sufficient
; accuracy for most guidance computations.
; ============================================================================

POLLEY		EXTEND
		MP	TEMK
; Compute x^2 and store in SQ (square)
; EXTEND enables next instruction to use multiply (MP) operation
; Multiply TEMK (x) by itself to get x^2

		TS	SQ
		EXTEND
		MP	C5/2
; Begin Horner's method evaluation: compute C5*x^2
; Polynomial coefficients C5/2, C3/2, C1/2 are pre-scaled by factor of 2
; to compensate for final DDOUBL operation

		AD	C3/2
		EXTEND
		MP	SQ
; Compute (C5*x^2 + C3)*x^2 = C5*x^4 + C3*x^2
; Horner's method reduces number of multiplications needed

		AD	C1/2
		EXTEND
		MP	TEMK
; Compute (C5*x^4 + C3*x^2 + C1)*x = C5*x^5 + C3*x^3 + C1*x
; This is the complete polynomial approximation

		DDOUBL
; Double the result twice (multiply by 4) to compensate for coefficient
; pre-scaling. DDOUBL is faster than two separate DOUBLE instructions.

		TS	TEMK
		TC	Q
; Return to caller with result in A register
; Q register contains return address stored by calling routine

; ============================================================================
; SPECIAL CASE: 90-DEGREE ARGUMENT HANDLER
;
; When the argument equals exactly PI/2 (90 degrees), the polynomial
; approximation would require evaluation at the boundary of its valid range.
; Instead, this special case handler directly returns the known exact value
; (sin(90°) = 1.0, cos(0°) = 1.0) for perfect accuracy without computation.
; ============================================================================

ARG90		INDEX	A
		CS	LIMITS
		TC	Q		# RESULT SCALED AT 1


