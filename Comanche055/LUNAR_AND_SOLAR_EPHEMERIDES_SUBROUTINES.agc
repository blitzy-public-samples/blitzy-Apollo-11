# Copyright:    Public domain.
# Filename:     LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 785-788
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-12 RSB	Adapted from Colossus249 file of the same
#				name and Comanche 055 page images.
#		2009-07-26 RSB	Added annotations related to computation
#				of the ephemeral(?) polynomials.
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#       Assemble revision 055 of AGC program Comanche by NASA
#       2021113-051.  April 1, 1969.
#
#       This AGC program shall also be referred to as Colossus 2A
#
#       Prepared by
#                       Massachusetts Institute of Technology
#                       75 Cambridge Parkway
#                       Cambridge, Massachusetts
#
#       under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

# Page 785

; ============================================================================
; FILE: LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: all-phases
;
; TL;DR: Moon and Sun position computation routines calculating celestial body
;        locations for navigation and guidance. Evaluates ephemeris polynomials
;        providing lunar and solar position vectors in inertial coordinates
;        throughout Apollo 11 mission timeline.
;
; COMMENT-ONLY READERS: This program calculated where the Moon and Sun were
;        located for navigation purposes during the mission.
; CODE-ALONG READERS: Study ephemeris polynomial evaluation, celestial position
;        computation, coordinate transformations for lunar/solar vectors.
; ============================================================================

# LUNAR AND SOLAR EPHEMERIDES SUBROUTINES
#
# FUNCTIONAL DESCRIPTION
#
#	THESE SUBROUTINES ARE USED TO DETERMINE THE POSITION AND VELOCITY
#	VECTORS OF THE SUN AND THE MOON RELATIVE TO THE EARTH AT THE
#	SPECIFIED GROUND ELAPSED TIME INPUT BY THE USER.
#
#	THE POSITION OF THE MOON IS STORED IN THE COMPUTER IN THE FORM OF
#	A NINTH DEGREE POLYNOMIAL APPROXIMATION WHICH IS VALID OVER A 15
#	DAY INTERVAL BEGINNING SHORTLY BEFORE LAUNCH.  THEREFORE THE TIME
#	INPUT BY THE USER SHOULD FALL WITHIN THIS 15 DAY INTERVAL.
#
#	LSPOS COMPUTES THE POSITION VECTORS OF THE SUN AND THE MOON.
#
#	LUNPOS COMPUTES THE POSITION VECTOR OF THE MOON.
#
#	LUNVEL COMPUTES THE VELOCITY VECTOR OF THE MOON.
#
#	SOLPOS COMPUTES THE POSITION VECTOR OF THE SUN.
#
# CALLING SEQUENCE
#
#	DLOAD	CALL
#		TIME		GROUND ELAPSED TIME
#		SUBROUTINE	LSPOS OR LUNPOS OR LUNVEL OR SOLPOS
#
# INPUT
#
#	1) SPECIFIED GROUND ELAPSED TIME IN CS x B-28 LOADED IN MPAC.
#
#	2) TIMEMO - TIME AT THE CENTER OF THE RANGE OVER WHICH THE LUNAR
#	   POSITION POLYNOMIAL IS VALID IN CS x B-42.
#
#	3) VECOEM - VECTOR COEFFICIENTS OF THE LUNAR POSITION POLYNOMIAL
#	   LOADED IN DESCENDING SEQUENCE IN METERS/CS**N x B-2
#
#	4) RESO - POSITION VECTOR OF THE SUN RELATIVE TO THE EARTH AT
#	   TIMEMO IN METERS x B-38.
#
#	5) VESO - VELOCITY VECTOR OF THE SUN RELATIVE TO THE EARTH AT
#	   TIMEMO IN METERS/CS x B-9.
#
#	6) OMEGAES - ANGULAR VELOCITY OF THE VECTOR RESO AT TIMEMO IN
#	   REV/CS x B+26.
#
#	ALL EXCEPT THE FIRST INPUT ARE INCLUDED IN THE PRE-LAUNCH
#	ERASABLE DATA LOAD.
#
# OUTPUT - LSPOS
# Page 786
#	1) 2D OF VAC AREA CONTAINS THE POSITION VECTOR OF THE SUN RELATIVE
#	   TO THE EARTH AT TIME INPUT BY THE USER IN METERS x B-38.
#
#	2) MPAC CONTAINS THE POSITION VECTOR OF THE MOON RELATIVE TO THE
#	   EARTH AT TIME INPUT BY THE USER IN METERS x B-29.
#
# OUTPUT - LUNPOS
#
#	MPAC CONTAINS THE POSITION VECTOR OF THE MOON RELATIVE TO THE
#	EARTH AT THE TIME INPUT BY USER IN METERS x B-29.
#
# OUTPUT - LUNVEL
#
#	MPAC CONTAINS THE VELOCITY VECTOR OF THE MOON RELATIVE TO THE
#	EARTH AT THE TIME INPUT BY THE USER IN METERS/CS x B-7.
#
# OUTPUT - SOLPOS
#
#	MPAC CONTAINS THE POSITION VECTOR OF THE SUN RELATIVE TO THE EARTH
#	AT TIME INPUT BY THE USER IN METERS x B-38.
#
# SUBROUTINES USED
#
#	NONE
#
# REMARKS
#
#	THE VAC AREA IS USED FOR STORAGE OF INTERMEDIATE AND FINAL RESULTS
#	OF COMPUTATIONS.
#
#	S1, X1 AND X2 ARE USED BY THESE SUBROUTINES.
#	PRELAUNCH ERASABLE DATA LOAD ARE ONLY ERASABLE STORAGE USED BY
#	THESE SUBROUTINES.
#	RESTARTS DURING OPERATION OF THESE SUBROUTINES MUST BE HANDLED BY
#	THE USER.

		BANK	36
		SETLOC	EPHEM
		BANK

		COUNT*	$$/EPHEM
		EBANK=	END-E7

; ============================================================================
; SUBROUTINE: LSPOS - Lunar and Solar Position Computation
;
; Throughout Apollo 11's journey from Earth to Moon and back, the spacecraft's
; navigation system needed to know precisely where the Moon and Sun were located
; relative to Earth. This information was critical for trajectory planning,
; navigation updates, and rendezvous operations.
;
; This subroutine computes both celestial body positions simultaneously,
; storing the Sun's position in the VAC area and the Moon's position in MPAC
; for use by navigation and guidance programs.
;
; TECHNICAL: Uses pre-loaded ephemeris polynomial coefficients (VECOEM) for
; lunar position and propagates solar position from reference time (TIMEMO)
; using angular velocity. Outputs in meters scaled B-38 (Sun) and B-29 (Moon).
; ============================================================================

LSPOS		AXT,2			# COMPUTES POSITION VECTORS OF BOTH THE
			RESA		# SUN AND THE MOON.  THE POSITION VECTOR
		AXT,1	GOTO		# OF THE SUN IS STORED IN 2D OF THE VAC
			RES		# AREA.  THE POSITION VECTOR OF THE MOON
			LSTIME		# IS STORED IN MPAC.
; ============================================================================
; SUBROUTINE: LUNPOS - Lunar Position Only
;
; When only the Moon's position is needed (not the Sun's), this entry point
; provides efficient computation. Used during lunar orbit operations and
; descent planning when solar position is not immediately required.
;
; TECHNICAL: Evaluates 9th-degree polynomial approximation of lunar position
; valid over 15-day mission window. Output in MPAC in meters scaled B-29.
; ============================================================================

LUNPOS		AXT,1	GOTO		# COMPUTES THE POSITION VECTOR OF THE MOON
			REM		# AND STORES IT IN MPAC.
			LSTIME

# Page 787

; ============================================================================
; SUBROUTINE: LUNVEL - Lunar Velocity Computation
;
; The Moon's velocity relative to Earth was needed for precise trajectory
; calculations, especially during translunar coast and lunar orbit insertion.
; This routine computes the time derivative of the lunar position polynomial.
;
; TECHNICAL: Evaluates derivative of position polynomial using scaled
; coefficients. Output velocity vector in MPAC in meters/cs scaled B-7.
; ============================================================================

LUNVEL		AXT,1	GOTO		# COMPUTES THE VELOCITY VECTOR OF THE MOON
			VEM		# AND STORES IT IN MPAC.
			LSTIME

; ============================================================================
; SUBROUTINE: SOLPOS - Solar Position Only
;
; When only the Sun's position is needed (not the Moon's), this entry point
; is used. Solar position was important for spacecraft thermal control and
; solar panel orientation during certain mission phases.
;
; TECHNICAL: Propagates solar position from reference time using angular
; velocity OMEGAES. Output in MPAC in meters scaled B-38.
; ============================================================================

SOLPOS		STQ	AXT,1		# COMPUTES THE POSITION VECTOR OF THE SUN
			X2		# AND STORES IT IN MPAC.
			RES
; ============================================================================
; TIME COMPUTATION SECTION
;
; All ephemeris computations require converting the current mission time
; into a time difference from the reference epoch (TIMEMO). This section
; computes: delta_t = current_time - TIMEMO
;
; The delta_t value is then used to evaluate polynomial approximations
; that describe how the Moon and Sun move through space over time.
;
; TECHNICAL: Input time in centiseconds (cs) scaled B-28, TEPHEM is current
; ephemeris time, TIMEMO is polynomial center time scaled B-42. Result scaled
; appropriately for polynomial evaluation (shift right 14D, left 16D = left 2D).
; ============================================================================

LSTIME		SETPD	SR
			0D
			14D
		TAD	DCOMP		; Compute time difference from ephemeris
			TEPHEM		; reference: delta_t = TEPHEM - TIMEMO
		TAD	DCOMP
			TIMEMO
		SL	SSP		; Scale time difference for polynomial
			16D		; evaluation and set loop counter
			S1
			6D
		GOTO
			X1
; ============================================================================
; SOLAR POSITION COMPUTATION (RES)
;
; The Sun's position relative to Earth changes slowly but predictably as
; Earth orbits the Sun. Rather than using a polynomial, the AGC propagates
; the Sun's position by rotating the reference position vector RESO through
; an angle based on elapsed time and angular velocity OMEGAES.
;
; This is essentially computing: Sun_position = Rotate(RESO, angle)
; where angle = OMEGAES * delta_t
;
; TECHNICAL: Implements rotation of RESO vector using angular velocity OMEGAES
; in rev/cs scaled B+26. Uses COS/SIN functions and cross products to construct
; rotation. VESO (solar velocity) used to establish rotation axis via cross
; product with RESO. Final result in meters scaled B-38.
; ============================================================================

RES		PUSH	DMP		# Compute rotation angle: OMEGAES * delta_t	PD- 2
			OMEGAES
		PUSH	COS		# Calculate cos(angle) for rotation		PD- 4
		VXSC	PDDL		# Scale RESO by cos(angle)			PD- 8
			RESO
		SIN	PDVL		# Calculate sin(angle) for rotation		PD-10
			RESO
		PUSH	UNIT		# Establish rotation axis from RESO		PD-16
		VXV	UNIT		; and VESO using cross product
			VESO
		VXV	VSL1		# Complete rotation transformation		PD-10
		VXSC	VAD		# Combine cos and sin components		PD-02
		VSL1	GOTO		# RES IN METERS x B-38 IN MPAC.
			X2
RESA		STODL	2D		# RES IN METERS x B-38 IN 2D OF VAC.	PD- 0

; ============================================================================
; LUNAR POSITION POLYNOMIAL EVALUATION (REM)
;
; The Moon's position was stored as a 9th-degree polynomial, pre-loaded before
; launch and valid for approximately 15 days covering the entire Apollo 11
; mission. This polynomial approximation was far more accurate than trying to
; compute lunar orbital mechanics in real-time with the AGC's limited processing.
;
; The evaluation uses Horner's method: working from highest degree term down,
; repeatedly multiplying by delta_t and adding the next coefficient. This is
; the most efficient polynomial evaluation method, minimizing multiplications.
;
; Polynomial form: Position = C0 + C1*t + C2*t^2 + ... + C9*t^9
; Horner's form: Position = C0 + t*(C1 + t*(C2 + t*(...)))
;
; TECHNICAL: Evaluates 9th-degree vector polynomial with coefficients VECOEM
; (vector coefficients loaded descending: C9, C8, ..., C0). Uses TIX,1 loop
; starting at index 54D (9 iterations * 6 = 54 for vector with 3 components).
; Output in meters scaled B-29 in MPAC.
; ============================================================================

REM		AXT,1	PDVL		# Initialize loop counter for 9-degree poly	PD- 2
			54D		; Index 54D = 9 terms * 6 words per vector
			VECOEM		; Load highest degree coefficient (C9)
REMA		VXSC	VAD*		; Horner's method: multiply by delta_t
			0D		; and add next coefficient
			VECOEM +60D,1	; VECOEM indexed by loop counter
		TIX,1	VSL2		; Loop 9 times through coefficients. REM IN METERS x B-29 IN MPAC.
			REMA
		RVQ
; ============================================================================
; LUNAR VELOCITY POLYNOMIAL EVALUATION (VEM)
;
; To compute the Moon's velocity, we need the time derivative of the position
; polynomial. For a polynomial P(t) = C0 + C1*t + C2*t^2 + ... + C9*t^9,
; the derivative is: P'(t) = C1 + 2*C2*t + 3*C3*t^2 + ... + 9*C9*t^8
;
; This routine evaluates that derivative polynomial, giving the Moon's
; velocity relative to Earth. This was essential for trajectory planning
; during translunar injection and lunar orbit insertion burns.
;
; TECHNICAL: Evaluates derivative of 9th-degree position polynomial. Each
; term multiplied by its degree: n*Cn*t^(n-1). Uses loop counter 48D for
; 8 iterations (derivative of 9th-degree is 8th-degree). Constants NINEB4
; (9.0 scaled B-4) and ONEB4 (1.0 scaled B-4) provide degree multipliers.
; Output velocity in meters/cs scaled B-7 in MPAC.
; ============================================================================

VEM		AXT,1	PDDL		# Initialize for derivative evaluation	PD- 2
			48D		; 48D = 8 terms * 6 words per vector
			NINEB4		; Load degree multiplier (starts at 9)
		PUSH	VXSC		# Multiply highest coefficient by degree	PD- 4
			VECOEM
VEMA		VXSC			; Horner's method for derivative:
			0D		; multiply by delta_t
# Page 788
		STODL	4D		; Store partial result			PD- 2
		DSU	PUSH		; Decrement degree counter (9,8,7...1)	PD- 4
			ONEB4
		VXSC*	VAD		; Scale coefficient by degree,
			VECOEM +54D,1	; add to accumulator
			4D
		TIX,1	VSL2		; Loop through all 8 derivative terms. VEM IN METERS/CS x B-7 IN MPAC.
			VEMA
		RVQ

; ============================================================================
; POLYNOMIAL EVALUATION CONSTANTS
;
; These constants provide the degree multipliers for velocity computation.
; NINEB4 = 9.0 (highest degree of position polynomial)
; ONEB4 = 1.0 (decrement value for degree counter)
; Scaled B-4 to match the coefficient scaling requirements.
; ============================================================================

NINEB4		2DEC	9.0 B-4

ONEB4		2DEC	1.0 B-4
