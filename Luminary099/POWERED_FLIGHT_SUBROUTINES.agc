# Copyright:	Public domain.
# Filename:	POWERED_FLIGHT_SUBROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1259-1267
# Mod history:	2009-05-26 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-01-06 JL	Fixed pseudo-label indentation.
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
; FILE: POWERED_FLIGHT_SUBROUTINES.agc
; MODULE: Powered Flight Navigation Utilities
; MISSION PHASE: descent/landing/ascent/rendezvous
;
; TL;DR: Provides coordinate transformation and matrix computation utilities
;        essential for navigation during engine burns. Handles transformations
;        between body-fixed and stable-member coordinate systems, computes
;        transformation matrices from IMU gimbal angles, and supports trajectory
;        calculations under thrust. Critical for accurate navigation during
;        descent engine, ascent engine, and RCS burns.
;
; COMMENT-ONLY READERS: These mathematical transformation routines ensure
;        the spacecraft knows its orientation during powered flight, enabling
;        precise control during lunar descent, ascent, and orbital maneuvers.
; CODE-ALONG READERS: Study the coordinate transformation algorithms (SMNB,
;        NBSM), gimbal angle processing (CDUTRIG family), and transformation
;        matrix computation (FLESHPOT) used throughout powered flight phases.
; ============================================================================

# Page 1259
		BANK	14		# SAME FBANK AS THE FINDCDUD SUB-PROGRAM
		SETLOC	POWFLITE
		BANK

		EBANK=	DEXDEX
		COUNT*	$$/POWFL

; ============================================================================
; COORDINATE TRANSFORMATION UTILITIES - CDU TRIGONOMETRY FAMILY
;
; During powered flight, the guidance computer must continuously track the
; spacecraft's orientation relative to the inertial measurement unit (IMU).
; The IMU measures attitude using three gimbals, and their angles (stored in
; CDU registers) define the transformation between coordinate systems.
;
; These routines compute sine and cosine values from gimbal angles, which are
; then used to construct transformation matrices. During lunar descent, this
; allows the computer to translate desired velocity changes (in inertial space)
; into engine gimbal commands (in body-fixed coordinates).
; ============================================================================

# CDUTRIG, CDUTRIG1, CDUTRIG2, AND CD*TR*GS ALL COMPUTE THE SINES AND
# COSINES OF THREE 2'S COMPLEMENT ANGLES AND PLACE THE RESULT, DOUBLE
# PRECISION, IN THE SAME ORDER AS THE INPUTS, AT SINCDU AND COSCDU.  AN
# ADDITIONAL OUTPUT IS THE 1'S COMPLEMENT ANGLES AT CDUSPOT.  THESE
# ROUTINES GO OUT OF THEIR WAY TO LEAVE THE MPAC AREA AS THEY FIND IT.
# EXCEPT FOR THE GENERALLY UNIMPORTANT MPAC +2.  THEY DIFFER ONLY IN
# WHERE THEY GET THE ANGLES, AND IN METHOD OF CALLING.
#
# CDUTRIG (AND CDUTRIG1, WHICH CAN BE CALLED IN BASIC) COMPUTE THE
# SINES AND COSINES FROM THE CURRENT CONTENTS OF THE CDU REGISTERS.
# THE CONTENTS OF CDUTEMP, ETC., ARE NOT TOUCHED SO THAT THEY MAY
# CONTINUE TO FORM A CONSISTENT SET WITH THE LATEST PIPA READINGS.
#
# CDUTRIG1 IS LIKE CDUTRIG EXCEPT THAT IT CAN BE CALLED IN BASIC.
#
# CD*TR*GS FINDS CDU VALUES IN CDUSPOT RATHER THAN IN CDUTEMP.  THIS
# ALLOWS USERS TO MAKE TRANSFORMATIONS USING ARBITRARY ANGLES, OR REAL
# ANGLES IN AN ORDER OTHER THAN X Y Z.  A CALL TO THIS ROUTINE IS
# NECESSARY IN PREPARATION FOR A CALL TO AX*SR*T IN EITHER OF ITS TWO
# MODES (SMNB OR NBSM).  SINCE AX*SR*T EXPECTS TO FIND THE SINES AND
# COSINES IN THE ORDER Y Z X THE ANGLES MUST HAVE BEEN PLACED IN CDUSPOT
# IN THIS ORDER.  CD*TR*GS NEED NOT BE REPEATED WHEN AX*SR*T IS CALLED
# MORE THAN ONCE, PROVIDED THE ANGLES HAVE NOT CHANGED.  NOTE THAT SINCE
; IT CLOBBERS BUF2 (IN THE SINE AND COSINE ROUTINES) CD*TR*GS CANNOT BE
# CALLED USING BANKCALL.  SORRY.
#
# CD*TR*G IS LIKE CD*TR*GS EXCEPT THAT IT CAN BE CALLED IN
# INTERPRETIVE.

; Technical details for code-along readers:
; - CDU (Coupling Data Unit) registers contain gimbal angles in 2's complement
; - Angles are converted to 1's complement for sine/cosine computation
; - Double-precision results ensure accuracy for matrix multiplications
; - MPAC (math register area) is preserved to avoid corrupting ongoing calculations

; CDUTRIG: Read current gimbal angles from CDU hardware registers.
; Used when real-time IMU orientation is needed for navigation calculations.
; Called from interpretive mode, returns to interpretive mode after completion.

CDUTRIG		EXIT
		TC	CDUTRIGS
		TC	INTPRET
		RVQ

; CD*TR*G: Process gimbal angles already stored in CDUSPOT memory.
; Used when working with saved or arbitrary angles rather than current IMU state.
; Called from interpretive mode, returns to interpretive mode after completion.

CD*TR*G		EXIT
		TC	CD*TR*GS
		TC	INTPRET
		RVQ

; CDUTRIGS: Transfer gimbal angles from hardware CDU registers to CDUSPOT buffer.
; CDUX, CDUY, CDUZ are hardware-mapped I/O registers containing gimbal angles
; measured by the IMU. These angles define spacecraft attitude in 3D space.

CDUTRIGS	CA	CDUX
		TS	CDUSPOT +4
		CA	CDUY
		TS	CDUSPOT
# Page 1260
		CA	CDUZ
		TS	CDUSPOT +2

; CD*TR*GS: Main computation routine for gimbal angle trigonometry.
; Processes three angles stored in CDUSPOT (Y, Z, X order for AX*SR*T compatibility).
; Computes double-precision sine and cosine for each angle, storing results
; in SINCDU and COSCDU arrays for subsequent transformation matrix construction.

CD*TR*GS	EXTEND
		QXCH	TEM2		; Save return address
		CAF	FOUR
; Loop processes three gimbal angles (counter: 4, 2, 0 for even addressing)
TR*GL**P	MASK	SIX		# MAKE IT EVEN AND SMALLER
		TS	TEM3		; Store loop index
		INDEX	TEM3
		CA	CDUSPOT		; Load angle in 2's complement format
		DXCH	MPAC		# STORING 2'S COMP ANGLE, LOADING MPAC
		DXCH	VBUF 	+4	# STORING MPAC FOR LATER RESTORATION
		TC	USPRCADR
		CADR	CDULOGIC	; Call sine/cosine computation routine
		EXTEND
		DCA	MPAC		; Retrieve computed sine/cosine result
		INDEX	TEM3
		DXCH	CDUSPOT		# STORING 1'S COMPLEMENT ANGLE
		TC	USPRCADR
		CADR	COSINE		; Compute cosine of angle
		DXCH	MPAC		; Retrieve double-precision cosine result
		INDEX	TEM3
		DXCH	COSCDU		# STORING COSINE
		EXTEND
		INDEX	TEM3
		DCA	CDUSPOT		# LOADING 1'S COMPLEMENT ANGLE
		TC	USPRCADR
		CADR	SINE 	+1	# SINE +1 EXPECTS ARGUMENT IN A AND L
		DXCH	VBUF 	+4	# BRINGING UP PRIOR MPAC TO BE RESTORED
		DXCH	MPAC		; Restore MPAC contents for next iteration
		INDEX	TEM3
		DXCH	SINCDU		; Store double-precision sine result
		CCS	TEM3		; Check and decrement loop counter (4→2→0)
		TCF	TR*GL**P	; Continue loop for remaining angles
		TC	TEM2		; Return to caller after processing all three angles
; ============================================================================
; TRANSITION: From Standard Trigonometry to High-Speed Trigonometry
;
; The CD*TR*GS family of routines provides accurate gimbal angle processing
; but requires significant computation time. During critical mission phases
; like lunar descent, where the guidance computer must update trajectory
; calculations every 2 seconds, faster alternatives are essential. QUICTRIG
; sacrifices some generality for a 10x speed improvement, executing in just
; 4.1 milliseconds instead of 41 milliseconds.
; ============================================================================

# Page 1261
# *******************************************************************************************************
# QUICTRIG, INTENDED FOR QUIDANCE CYCLE USE WHERE TIME IS CRITICAL, IS A MUCH FASTER VERSION OF CD*TR*GS.
# QUICTRIG COMPUTES AND STORES THE SINES AND COSINES OF THE 2'S COMPLEMENT ANGLES AT CDUSPOT, CDUSPOT +2,
# AND CDUSPOT +4.  UNLIKE CD*TR*GS, QUICTRIG DOES NOT LEAVE THE 1'S COMPLEMENT VERSIONS OF THE ANGLES IN
# CDUSPOT.  QUICTRIG'S EXECUTION TIME IS 4.1 MS;  THIS IS 10 TIMES AS FAST AS CD*TR*GS.  QUICTRIG MAY BE
# CALLED FROM INTERPRETIVE AS AN RTB OP-CODE, OR FROM BASIC VIA BANKCALL OR IBNKCALL.

; QUICTRIG: High-speed trigonometry for time-critical guidance calculations.
; Used during powered descent when guidance updates occur every 2 seconds.
; Speed optimization comes from using single-precision SPSIN/SPCOS routines
; instead of the more general but slower SINE/COSINE routines.
; Critical during lunar landing when computational load is at maximum.

QUICTRIG	INHINT			# INHINT SINCE DAP USES THE SAME TEMPS
		EXTEND
		QXCH	ITEMP1		; Save return address
		CAF	FOUR		; Initialize loop counter for three angles
 +4		MASK	SIX		; Make index even (4, 2, 0)
		TS	ITEMP2		; Store loop index
		INDEX	ITEMP2
		CA	CDUSPOT		; Load gimbal angle from CDUSPOT array
		TC	SPSIN		; Fast single-precision sine computation
		EXTEND
		MP	BIT14		# SCALE DOWN TO MATCH INTERPRETER OUTPUTS
		INDEX	ITEMP2
		DXCH	SINCDU		; Store sine result (double-precision)
		INDEX	ITEMP2
		CA	CDUSPOT		; Reload same angle for cosine computation
		TC	SPCOS		; Fast single-precision cosine computation
		EXTEND
		MP	BIT14		; Scale to match interpretive precision
		INDEX	ITEMP2
		DXCH	COSCDU		; Store cosine result (double-precision)
		CCS	ITEMP2		; Check and decrement loop counter
		TCF	QUICTRIG +4	; Process next angle (Y, then Z, then X)
		CA	ITEMP1		; All three angles processed, restore return address
		RELINT			; Re-enable interrupts
		TC	A		; Return to caller

; ============================================================================
; TRANSITION: From Trigonometry Computation to Coordinate Transformation
;
; Having computed sines and cosines of gimbal angles, the next step is
; transforming vectors between coordinate systems. During powered flight,
; the guidance computer continuously transforms between Navigation Base (NB)
; coordinates aligned with the spacecraft's reference frame and Stable
; Member (SM) coordinates aligned with the IMU platform. These transformations
; enable the computer to relate thrust vectors, velocity changes, and
; navigation updates to the inertial reference frame maintained by the IMU.
; ============================================================================

# Page 1262
#****************************************************************************
# THESE INTERFACE ROUTINES MAKE IT POSSIBLE TO CALL AX*SR*T, ETC., IN
# INTERPRETIVE.  LATER, WHERE POSSIBLE, THEY WILL BE ELIMINATED.
#
# THESE INTERFACE ROUTINES ARE PERMANENT.  ALL RESTORE USER'S EBANK
# SETTING. ALL ARE STRICT INTERPRETIVE SUBROUTINES, CALLED USING "CALL",
# RETURNING VIA QPRET.  ALL EXPECT AND RETURN THE VECTOR TO BE TRANSFORMED
# INTERPRETER-STYLE IN MPAC; COMPONENTS AT MPAC, MPAC +3, AND MPAC +5.
#
# TRG*SMNB AND TRG*NBSM BOTH EXPECT TO SEE THE 2'S COMPLEMENT ANGLES
# AT CDUSPOT (ORDER Y Z X, AT CDUSPOT, CDUSPOT +2, AND CDUSPOT +4; ODD
# LOCATIONS NEED NOT BE ZEROED).  TRG*NBSM DOES THE NB TO SM TRANSFORMATION;
# TRG*SMNB, VICE VERSA.
#
# CDU*NBSM DOES ITS TRANSFORMATION USING THE PRESENT CONTENTS OF
# THE CDL COUNTERS.  OTHERWISE IT IS LIKE TRG*NBSM.
#
# CDU*SMNB IS THE COMPLEMENT OF CDU*NBSM.

; Coordinate transformation interface routines for interpretive programs.
; These provide simple calling interfaces to the core AX*SR*T transformation
; engine, handling the conversions between interpretive and basic modes.

; CDU*SMNB: Transform vector from Stable Member to Navigation Base coordinates
; using current CDU gimbal angles. Used when guidance needs to express IMU
; measurements (accelerations, velocities) in spacecraft body frame.
CDU*SMNB	EXIT			; Exit interpretive mode
		TC	CDUTRIGS	; Compute sines/cosines from current CDU angles
		TCF	C*MM*N1		; Continue to common transformation code

; TRG*SMNB: Transform using angles already stored in CDUSPOT (not from CDU)
; Allows transformation with arbitrary angles for trajectory predictions.
TRG*SMNB	EXIT
		TC	CD*TR*GS	; Compute sines/cosines from CDUSPOT angles
C*MM*N1		TC	MPACVBUF	# AX*SR*T EXPECTS VECTOR IN VBUF
		CS	THREE		# SIGNAL FOR SM TO NB TRANSFORMATION.
C*MM*N2		TC	AX*SR*T		; Perform the actual coordinate rotation
		TC	INTPRET		; Return to interpretive mode
		VLOAD	RVQ		; Load result vector and return
			VBUF		; Result returned in MPAC via VBUF

; CDU*NBSM: Transform vector from Navigation Base to Stable Member coordinates
; using current CDU gimbal angles. Used when guidance needs to express
; spacecraft-relative thrust commands in inertial reference frame.
CDU*NBSM	EXIT
		TC	CDUTRIGS	; Compute sines/cosines from current CDU angles
		TCF	C*MM*N3

; TRG*NBSM: Transform using angles in CDUSPOT rather than current CDU values
TRG*NBSM	EXIT
		TC	CD*TR*GS	; Compute sines/cosines from CDUSPOT angles
C*MM*N3		TC	MPACVBUF	# FOR AX*SR*T
		CA	THREE		# SIGNAL FOR NB TO SM TRANSFORMATION
		TCF	C*MM*N2		; Use common transformation code

# *NBSM* AND *SMNB* EXPECT TO SEE THE SINES AND COSINES (AT SINCDU
# AND COSCDU) RATHER THAN THE ANGLES THEMSELVES.  OTHERWISE THEY ARE
# LIKE TRG*NBSM AND TRG*SMNB.
#
# NOTE THAT JUST AS CD*TR*GS NEED BE CALLED ONLY ONCE FOR EACH SERIES
# OF TRANSFORMATIONS USING THE SAME ANGLES, SO TOO ONLY ONE OF TRG*NBSM
# Page 1263
# AND TRG*SMNB NEED BE CALLED FOR EACH SERIES.  FOR SUBSEQUENT TRANFOR-
# MATIONS USE *NBSM* AND *SMNB*.

; *SMNB*: Optimized version skipping trigonometry when sines/cosines already
; computed. Used for multiple transformations with same gimbal angles.
; Critical for computational efficiency during powered flight when many vectors
; (thrust, velocity, position) need transformation with same orientation.
*SMNB*		EXIT
		TCF	C*MM*N1		; Jump directly to transformation logic

; *NBSM*: Optimized version for NB to SM transformation with pre-computed trig
*NBSM*		EXIT
		TCF	C*MM*N3		; Jump directly to transformation logic

; ============================================================================
; TRANSITION: From Interface Layer to Core Transformation Engine
;
; The AX*SR*T (axis rotation) routine is the mathematical heart of coordinate
; transformation during powered flight. All interface routines ultimately
; call this engine, which performs the actual vector rotation using sines
; and cosines computed from gimbal angles. During lunar descent and ascent,
; this routine executes continuously, transforming thrust vectors, velocity
; measurements, and position data between stable member and body coordinates.
; ============================================================================

# AX*SR*T COMBINES THE OLD SMNB AND NBSM.  FOR THE NB TO SM
# TRANSFORMATION, ENTER WITH +3 IN A.  FOR SM TO NB, ENTER WITH -3.
# THE VECTOR TO BE TRANSFORMED ARRIVES, AND IS RETURNED, IN VBUF.
# AX*SR*T EXPECTS TO FIND THE SINES AND COSINES OF THE ANGLES OF ROTATION
# AT SINCDU AND COSCDU, IN THE ORDER Y Z X.  A CALL TO CD*TR*GS, WITH
# THE 2'S COMPLEMENT ANGLES (ORDER Y Z X) AT CDUSPOT, WILL TAKE CARE OF
# THIS.  HERE IS A SAMPLE CALLING SEQUENCE:--
#		TC	CDUTRIGS
#		CS	THREE		# ("CA THREE" FOR NBSM)
#		TC	AX*SR*T
# THE CALL TO CD*TR*GS NEED NOT BE REPEATED, WHEN AX*SR*T IS CALLED MORE
# THAN ONCE, UNLESS THE ANGLES HAVE CHANGED.
#
# AX*SR*T IS GUARANTEED SAFE ONLY FOR VECTORS OF MAGNITUDE LESS THAN
# UNITY.  A LOOK AT THE CASE IN WHICH A VECTOR OF GREATER MAGNITUDE
# HAPPENS TO LIE ALONG AN AXIS OF THE SYSTEM TO WHICH IT IS TO BE TRANS-
# FORMED CONVINCES ONE THAT THIS IS A RESTRICTION WHICH MUST BE ACCEPTED.

; AX*SR*T - Core axis rotation transformation engine
; During powered flight, this routine executes repeatedly to transform vectors
; between coordinate frames. For lunar descent, it converts IMU measurements
; to body-frame thrust commands. For ascent, it transforms guidance outputs
; to gimbal control signals. The transformation is a 3x3 rotation matrix
; multiplication using Euler angles (Y-Z-X gimbal sequence).
;
; INPUTS: A register = +3 for NB→SM transformation, -3 for SM→NB
;         VBUF = input vector (three double-precision components)
;         SINCDU, COSCDU = sines and cosines of Y, Z, X gimbal angles
; OUTPUTS: VBUF = transformed vector
; ALGORITHM: Applies three successive rotations about Y, Z, X axes
AX*SR*T		TS	DEXDEX		# WHERE IT BECOMES THE INDEX OF INDEXES.
		EXTEND
		QXCH	RTNSAVER	; Save return address

; Rotation loop outer control - performs three axis rotations
; Entry with +3/-3 determines forward (NB→SM) or inverse (SM→NB) transform
R*TL**P		CCS	DEXDEX		#       	+3 --> 0	-3 --> 2
		CS	DEXDEX		# THUS:		+2 --> 1	-2 --> 1
		AD	THREE		#		+1 --> 2	-1 --> 0
		EXTEND
		INDEX	A		; Index into INDEXI table for component order
		DCA	INDEXI		; Load component indices for this rotation
		DXCH	DEXI		; Store in working registers

		CA	ONE		; Initialize loop counter for two components
		TS	BUF
		EXTEND
		INDEX	DEX1		; Load first vector component
		DCS	VBUF		; Double-precision load with sign complement
		TCF	LOOP1		# REALLY BE A SUBTRACT, AND VICE VERSA

; Inner loop - computes one component of transformed vector
; Formula: output[i] = input[j] * sin(angle) + input[k] * cos(angle)
; where j, k are determined by rotation axis and direction
LOOP2		DXCH	BUF		# LOADING VECTOR COMPONENT, STORING INDEX
# Page 1264
LOOP1		DXCH	MPAC		; Vector component to multiplicand register
		CA	SINSLOC		; Base address of sine array
		AD	DEX1		; Add component index offset
		TS	ADDRWD		; Store address for multiply subroutine

		TC	DMPSUB		# MULTIPLY AT SIN(CDUANGLE)
		CCS	DEXDEX		; Check transformation direction
		DXCH	MPAC		# NBSM CASE - use product as-is
		TCF	+3		; Skip negation
		EXTEND			# SMNB CASE - inverse transform needs sign flip
		DCS	MPAC		; Negate product for inverse rotation
		DXCH	TERM1TMP	; Save first term (sin component)

		CA	SIX		# SINCDU AND COSCDU (EACH 6 WORDS) MUST
		ADS	ADDRWD		#	BE CONSECUTIVE AND IN THAT ORDER
					; Advance address from SINCDU to COSCDU array

		EXTEND
		INDEX	BUF		; Load second vector component
		INDEX	DEX1		; Using double indexing for component access
		DCA	VBUF
		DXCH	MPAC		; Second component to multiplicand register
		TC	DMPSUB		# MULTIPLY BY COS(CDUANGLE)
		DXCH	MPAC		; Retrieve cosine product
		DAS	TERM1TMP	; Add to sine term: sin*component1 + cos*component2
		DXCH	TERM1TMP	; Load complete rotation result
		DDOUBL			; Scale by 2 (AGC fixed-point normalization)
		INDEX	BUF		; Store transformed component back to vector
		INDEX	DEX1
		DXCH	VBUF
		DXCH	BUF		# LOADING INDEX, STORING VECTOR COMPONENT

		CCS	A		# 'CAUSE THAT'S WHERE THE INDEX NOW IS
		TCF	LOOP2		; Process second component of this rotation

		EXTEND
		DIM	DEXDEX		# DECREMENT MAGNITUDE PRESERVING SIGN
					; Reduces +3→+2→+1→0 or -3→-2→-1→0

; Iteration control - performs three axis rotations (Y, Z, X gimbal sequence)
; After each rotation completes (two components computed), check if more axes remain
TSTPOINT	CCS	DEXDEX		# ONLY THE BRANCHING FUNCTION IS USED
		TCF	R*TL**P		; Positive: more rotations remain, continue
		TC	RTNSAVER	; Zero: all rotations complete, return
		TCF	R*TL**P		; Negative: more rotations remain, continue
		TC	RTNSAVER	; Zero from negative: all rotations complete

SINSLOC		ADRES	SINCDU		# FOR USE IN SETTING ADDRWD

; Component index table - defines which vector components participate in each rotation
; Critical constant table - do NOT modify! Values determine rotation sequence.
INDEXI		DEC	4		# **********   DON'T   ***********
		DEC	2		# **********   TOUCH   ***********
		DEC	0		# **********   THESE   ***********
# Page 1265
		DEC	4		# ********** CONSTANTS ***********
					; Specifies Y-Z-X Euler angle rotation order

; ============================================================================
; TRANSITION: From Vector Transformation to Matrix Calculation
;
; The AX*SR*T routine transforms individual vectors using rotation angles.
; The FLESHPOT routine (Body-Stable Member Transformation Matrix Calculator)
; computes and stores the complete 3x3 transformation matrix XNB that
; represents the orientation of the spacecraft body frame relative to the
; stable member (IMU) reference frame. This matrix is fundamental for all
; navigation computations during powered flight - it tells the computer
; exactly how the spacecraft is oriented in inertial space.
; ============================================================================

# ******************************************************************************

		BANK	10
		SETLOC	FLESHLOC
		BANK
		COUNT*	$$/POWFL

; ****************************************************************************
; ROUTINE: FLESHPOT (Body-Stable Member Transformation Matrix Calculator)
;
; PURPOSE: Computes the complete 3x3 body-to-stable-member transformation
;          matrix (XNB) from the current gimbal angles. This matrix defines
;          the orientation of the spacecraft body frame relative to the IMU
;          stable member frame.
;
; COMMENT-ONLY READERS: This routine calculates the spacecraft's orientation
; in space by combining the three gimbal angles into a single mathematical
; representation. Every navigation calculation during engine burns depends
; on this matrix to know which way the spacecraft is pointing.
;
; CODE-ALONG READERS: Computes XNB matrix using direction cosines from CDU
; angles. Matrix elements calculated by multiplying appropriate combinations
; of sines and cosines for Y-Z-X Euler angle rotation sequence.
; ****************************************************************************

# ROUTINE FLESHPOT COMPUTES THE BODY-STABLE MEMBER TRANSFORMATION MATRIX (COMMONLY CALLED XNB) AND STORES
# IT IN THE LOCATIONS SPECIFIED BY THE ECADR ENTERING IN A.

; CALCSMSC: Interpretive-callable interface to FLESHPOT
; Exits interpretive mode, calls FLESHPOT with default XNB storage location,
; returns to interpretive mode. Used when navigation programs need to update
; the transformation matrix during mission maneuvers.
CALCSMSC	EXIT			; Exit interpretive mode for basic call
		TC	BANKCALL	; Cross-bank call to FLESHPOT
		CADR	FLESHPOT -1	; Entry point loads XNB address
		TC	INTPRET		; Return to interpretive mode
		RVQ			; Return to caller

; Storage address for XNB transformation matrix (9 double-precision words)
XNBECADR	ECADR	XNB		; Extended address of XNB matrix

; FLESHPOT entry with default XNB address
; Loads XNBECADR into A register then falls through to main FLESHPOT
 -1		CAF	XNBECADR	; Load XNB storage address

; ****************************************************************************
; FLESHPOT Main Entry Point
;
; INPUT:  A register contains ECADR of storage location for XNB matrix
;         SINCDU and COSCDU contain sines and cosines of gimbal angles
;
; OUTPUT: 3x3 transformation matrix stored at specified ECADR location
;         Matrix represents body-to-stable-member coordinate transformation
;
; METHOD: Computes nine matrix elements as products of direction cosines
;         Uses Y-Z-X Euler angle rotation sequence matching AGC gimbal order
; ****************************************************************************
FLESHPOT	TS	TEM2		; Save ECADR temporarily
		XCH	EBANK		; Save current EBANK setting
		XCH	TEM2		; Get ECADR back
		MASK	LOW8		; Extract bank number from ECADR
		AD	OCT1400		; Add offset for erasable bank addressing
		TS	TEM1		; TEM1 = base address for matrix storage

; Matrix element XNB(1,1) = cos(Y) * cos(Z)
; This represents the component of the body X-axis along the stable member
; X-axis when the spacecraft is rotated through gimbal angles Y and Z.
		EXTEND
		DCA	COSCDUY		; Load cos(Y) into MPAC (double precision)
		DXCH	MPAC
		TC	DMP		; Multiply by cos(Z)
		ADRES	COSCDUZ		; Address of cos(Z)
		DXCH	MPAC		; Result: cos(Y) * cos(Z)
		DDOUBL			; Scale result (left shift for proper scaling)
		INDEX	TEM1
		DXCH	0		; Store in XNB(1,1) = COSY * COSZ

; Matrix element XNB(1,2) = sin(Z)
; Direct component - no multiplication needed for this element
		EXTEND
		DCA	SINCDUZ		; Load sin(Z) (double precision)
		INDEX	TEM1
		DXCH	2		; Store in XNB(1,2) = SINZ

; Matrix element XNB(1,3) = -sin(Y) * cos(Z)
; Negative sign indicates rotation direction in Y-Z-X Euler sequence
		EXTEND
		DCS	SINCDUY		; Load -sin(Y) (DCS provides negation)
		DXCH	MPAC
		TC	DMPSUB		; Multiply by cos(Z), ADDRWD set to COSCDUZ
# Page 1266
		DXCH	MPAC		; Retrieve result
		DDOUBL			; Scale for proper matrix element representation
		INDEX	TEM1
		DXCH	4		; Store in XNB(1,3) = -SINY * COSZ

; Matrix element XNB(2,1) = -sin(X) * cos(Z)
; Represents projection of body Y-axis onto stable member X-axis
		EXTEND
		DCS	SINCDUX		; Load -sin(X)
		DXCH	MPAC
		TC	DMPSUB		; Multiply by cos(Z), ADDRWD still COSCDUZ
		DXCH	MPAC		; Result: -sin(X) * cos(Z)
		DDOUBL			; Scale result
		DXCH	MPAC	+3	; Save temporarily in MPAC+3

; Beginning computation of XNB(2,2) and XNB(2,3) - complex matrix elements
; These require multiple trigonometric product accumulations
;
; XNB(2,2) = cos(X)*cos(Y) + sin(X)*sin(Y)*sin(Z)
; First term: Calculate sin(X)*sin(Z) as intermediate result
		EXTEND
		DCS	SINCDUX		; Load -sin(X)
		DXCH	MPAC
		TC	DMP		; Multiply by sin(Z)
		ADRES	SINCDUZ
		EXTEND
		DCS	MPAC		; Negate to get +sin(X)*sin(Z)
		DXCH	MPAC 	+5	; Save in MPAC+5
		TC	DMP		; Multiply by sin(Y)
		ADRES	SINCDUY
		DXCH	MPAC		; Result: sin(X)*sin(Y)*sin(Z)
		DDOUBL			; Scale for accumulation
		DDOUBL			; Double scaling for matrix element
		DXCH	MPAC 	+5	; Store partial result

; Second term for XNB(2,2): Calculate cos(Y)*cos(X) and accumulate
		DXCH	MPAC		; Get sin(X)*sin(Z) back
		TC	DMP		; Multiply by cos(Y)
		ADRES	COSCDUY
		DXCH	MPAC
		DDOUBL			; Scale result
		DDOUBL			; Double scaling
		DDOUBL
		DXCH	BUF

		EXTEND
		DCA	COSCDUY
		DXCH	MPAC
		TC	DMP
		ADRES	COSCDUX
		DXCH	MPAC
		DDOUBL
		DAS	MPAC 	+5	; Accumulate: Add cos(Y)*cos(X) to MPAC+5

; XNB(2,3) = -sin(Y)*cos(X) + sin(X)*sin(Z)*cos(Y)
; Final element of second row - combines rotation effects across all axes
		EXTEND
		DCA	SINCDUY		; Load sin(Y)
		DXCH	MPAC
		TC	DMPSUB		# ADDRWD SET TO COSCDUX
		DXCH	MPAC		; Result: sin(Y)*cos(X)

# Page 1267
		DDOUBL			; Scale result
		DAS 	BUF		; Accumulate in BUF (adds to previous computation)

; Prepare results for storage - finalize second row of matrix
		DXCH	BUF		; Retrieve accumulated XNB(2,3) result
		DXCH	MPAC		; Place in MPAC for storage

; Store second row of XNB transformation matrix
; Row 2 represents projection of body Y-axis onto stable member frame
		EXTEND
		DCA	MPAC		; Get XNB(2,3)
		INDEX	TEM1
		DXCH	14		# = - SINY COSX + SINX SINZ COSY

		EXTEND
		DCA	MPAC 	+3	; Get XNB(2,1) from temporary storage
		INDEX	TEM1
		DXCH	16		# = - SINX COSZ

		EXTEND
		DCA	MPAC 	+5	; Get XNB(2,2) from accumulator
		INDEX	TEM1
		DXCH	20		# = COSX COSY - SINX SINY SINZ

; ============================================================================
; THIRD ROW COMPUTATION: XNB(3,1), XNB(3,2), XNB(3,3)
; Third row computed as cross product of first two rows (row1 x row2)
; This ensures orthonormality of transformation matrix - critical for
; accurate spacecraft attitude representation during powered flight
; ============================================================================

		CA	TEM1		; Setup for vector cross product
		TS	ADDRWD		; Set address base pointer
		EXTEND
		DCA	Z		; Load return address base
		AD	FOUR		; Offset for cross product operation
		DXCH	LOC		; Store modified address
		CAF	BIT8		; Set operation mode flag
		TS	EDOP
		TCF	VXV		; Call vector cross product: Row1 x Row2 = Row3

; Store third row of XNB transformation matrix
; Row 3 represents projection of body Z-axis onto stable member frame
; This axis is perpendicular to both X and Y body axes
		DXCH	MPAC		; Get XNB(3,1) from cross product result
		DDOUBL			; Scale for storage
		INDEX	TEM1
		DXCH	6		; Store XNB(3,1)

		DXCH	MPAC 	+3	; Get XNB(3,2)
		DDOUBL			; Scale for storage
		INDEX 	TEM1
		DXCH	10		; Store XNB(3,2)

		DXCH	MPAC 	+5	; Get XNB(3,3)
		DDOUBL			; Scale for storage
		INDEX 	TEM1
		DXCH	12		; Store XNB(3,3)

; Matrix computation complete - all 9 elements of XNB now stored
; Transformation matrix ready for use by guidance and navigation systems
		CA	TEM2		; Restore E-bank register
		TS	EBANK
		TCF	SWRETURN	; Return to caller

