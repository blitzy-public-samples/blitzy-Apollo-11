# Copyright:	Public domain.
# Filename:	CSM_GEOMETRY.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	285-296
# Mod history:	2009-05-08 RSB	Adapted from the Colossus249/ file of the
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
; FILE: CSM_GEOMETRY.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Command/Service Module geometric relationships for optical navigation
;        and coordinate transformations. Defines sextant optical system geometry,
;        IMU mounting orientation via coordinate transformation matrices, angular
;        calibration constants, and Earth rotation calculations. Essential for
;        star sighting, landmark tracking, and navigation base coordinate system.
;
; COMMENT-ONLY READERS: This file contained the spacecraft's optical system
;        geometry and coordinate transformation data needed for star navigation
;        and precise attitude determination throughout the mission.
; CODE-ALONG READERS: Study coordinate transformation matrices encoding IMU
;        mounting geometry, sextant optics calculations, navigation base to
;        optical system conversions, and Earth rotation vector updates.
; ============================================================================

# Page 285
		BANK	22
		SETLOC	COMGEOM1
		BANK

; ============================================================================
; SEXTANT TO NAVIGATION BASE COORDINATE TRANSFORMATION
;
; The Command Module's sextant optical system allowed astronauts to sight on
; stars and landmarks for navigation updates. This routine converts the physical
; shaft and trunnion angles (how the sextant was positioned) into a direction
; vector in the spacecraft's navigation coordinate system.
;
; During Apollo 11, this transformation was essential for platform alignment
; checks and landmark tracking during lunar orbit.
; ============================================================================

# THIS ROUTINE TAKES THE SHAFT AND TRUNNION ANGLES AS READ BY THE CM OPTICAL SYSTEM AND CONVERTS THEM INTO A  UNIT
# VECTOR REFERENCED TO THE NAVIGATION BASE COORDINATE SYSTEM AND COINCIDENT WITH THE SEXTANT LINE OF SIGHT.
#
# THE INPUTS ARE  1) THE SEXTANT SHAFT AND TRUNNION ANGLES ARE STORED SP IN LOCATIONS 3 AND 5 RESPECTIVELY OF THE
# MARK VAC AREA.  2) THE COMPLEMENT OF THE BASE ADDRESS OF THE MARK VAC AREA IS STORED SP AT LOCATION X1 OF YOUR
# JOB VAC AREA.
#
# THE OUTPUT IS A HALF-UNIT VECTOR IN NAVIGATION BASE COORDINATES AND STORED AT LOCATION 32D OF THE VAC AREA. THE
# OUTPUT IS ALSO AVAILABLE AT MPAC.


		COUNT	23/GEOM

; The sextant line-of-sight vector is computed using spherical trigonometry
; from the shaft angle (SA) and trunnion angle (TA). The vector is initially
; expressed in the sextant optical coordinate system, then transformed to the
; navigation base coordinate system using the NB1NB2 transformation matrix.
;
; Coordinate system: Sextant optics are mechanically mounted relative to the
; IMU navigation base. The transformation matrix accounts for this fixed
; geometric relationship between the optical instrument and navigation system.

SXTNB		SLOAD*	RTB		# PUSHDOWN  00,02,04,(17D-19D),32D-36D
			5,1		# TRUNNION = TA (vertical angle of sextant)
			CDULOGIC
		RTB	PUSH
			SXTLOGIC	; Apply 19.775 degree offset correction
		SIN	SL1
		PUSH	SLOAD*		# PD2 = SIN(TA)
			3,1		# SHAFT = SA (horizontal rotation angle)
		RTB	PUSH		# PD4 = SA
			CDULOGIC

		COS	DMP		; X-component in optics frame
			2
		STODL	STARM		# COS(SA)SIN(TA)

		SIN	DMP		; Y-component in optics frame
		STADR
		STODL	STARM	+2	# SIN(SA)SIN(TA)

		COS			; Z-component in optics frame
		STOVL	STARM	+4	; STARM now holds line-of-sight in optics coords
			STARM		# STARM = 32D
		MXV	VSL1		; Transform from optics to navigation base
			NB1NB2		; Transformation matrix encoding IMU mounting
		STORE	32D		; Result: half-unit vector in nav base coords
		RVQ


; The sextant optical axis is not aligned with the zero reference of its shaft
; encoder. This routine applies a 19.775 degree correction to account for the
; mechanical offset between the sextant optical centerline and its mounting
; reference. This ensures accurate conversion from raw encoder readings to true
; optical angles.

SXTLOGIC	CAF	10DEGS-		# CORRECT FOR 19.775 DEGREE OFFSET
		ADS	MPAC		; Add correction angle to current shaft angle
		CAF	QUARTER		; Scale factor for trigonometric computation
		TC	SHORTMP		; Short multiply for efficient calculation
		TC	DANZIG		; Continue to trigonometric processing
# Page 286
; ============================================================================
; INVERSE SEXTANT ANGLE CALCULATION
;
; This is the inverse operation of SXTNB. Given a known star position, this
; routine calculates the shaft and trunnion angles needed to point the sextant
; at that star. During Apollo 11, this was used when the crew needed to acquire
; a specific star for platform alignment verification or navigation updates.
;
; The computation transforms the star vector from stable member (IMU) coordinates
; through the navigation base to the sextant optical coordinate system, then
; extracts the required mechanical positioning angles.
; ============================================================================

# CALCSXA COMPUTES THE SEXTANT SHAFT AND TRUNNION ANGLES REQUIRED TO POSITION THE OPTICS SUCH THAT A STAR LINE-
# OF-SIGHT LIES ALONG THE STAR VECTOR.  THE ROUTINE TAKES THE GIVEN STAR VECTOR AND EXPRESSES IT AS A VECTOR REF-
# ERENCED TO THE OPTICS COORDINATE SYSTEM.  IN ADDITION IT SETS UP THREE UNIT VECTORS DEFINING THE X,Y, AND Z AXES
# REFERENCED TO THE OPTICS COORDINATE SYSTEM.
#
# THE INPUTS ARE  1) THE STAR VECTOR REFERRED TO PRESENT STABLE MEMBER COORDINATES STORED AT STAR.  2) SAME ANGLE
# INPUT AS *SMNB*, I.E. SINES AND COSINES OF THE CDU ANGLES, IN THE ORDER Y Z X, AT SINCDU AND COSCDU.  A CALL
# TO CDUTRIG WILL PROVIDE THIS INPUT.
#
# THE OUTPUTS ARE THE SEXTANT SHAFT AND TRUNNION ANGLES STORED DP AT SAC AND PAC RESPECTIVELY.  (LOW ORDER PART
# EQUAL TO ZERO).


; Transform star vector from stable member coordinates to navigation base,
; then to optics coordinate system. The three unit vectors (X, Y, Z) defining
; the navigation base axes are also transformed to the optics system for the
; subsequent angle extraction.

CALCSXA		ITA	VLOAD		# PUSHDOWN 00-26D,28D,30D,32D-36D
			28D		; Save return address
			STAR		; Load star vector (stable member coords)
		CALL
			*SMNB*		; Transform: stable member to navigation base
		MXV	VSL1		; Transform: navigation base to optics coords
			NB2NB1		; NB2NB1 matrix encodes IMU-to-optics geometry
		STOVL	STAR		; Store transformed star vector
			HIUNITX		; Load X-axis unit vector
		STOVL	XNB1		; Store X in optics coordinates
			HIUNITY		; Load Y-axis unit vector
		STOVL	YNB1		; Store Y in optics coordinates
			HIUNITZ		; Load Z-axis unit vector
		STCALL	ZNB1		; Store Z in optics coordinates, compute angles
			SXTANG1
# Page 287
; ============================================================================
; SEXTANT ANGLE COMPUTATION (COMMON ROUTINE)
;
; This routine performs the core geometric calculation to determine shaft and
; trunnion angles. It works in any coordinate system as long as both the star
; vector and navigation base axes are expressed in the same frame. This
; flexibility allows it to be called from multiple contexts.
; ============================================================================

# SXTANG COMPUTES THE SEXTANT SHAFT AND TRUNNION ANGLES REQUIRED TO POSITION THE OPTICS SUCH THAT A STAR LINE-OF-
# SIGHT LIES ALONG THE STAR VECTOR.
#
# THE INPUTS ARE  1) THE STAR VECTOR REFERRED TO ANY COORDINATE SYSTEM STORED AT STAR.  2) THE NAVIGATION BASE
# COORDINATES REFERRED TO THE SAME COORDINATE SYSTEM.  THESE THREE HALF-UNIT VECTORS ARE STORED AT XNB, YNB, AND
# ZNB.
#
# THE OUTPUTS ARE THE SEXTANT SHAFT AND TRUNNION ANGLES STORED DP AT SAC AND PAC RESPECTIVELY.  (LOW ORDER PART
# EQUAL TO ZERO).

; Transform the navigation base coordinate axes from their current reference
; frame into the sextant optics coordinate system. This allows expressing the
; star line-of-sight direction in terms that can be converted to mechanical
; shaft and trunnion positioning angles.

SXTANG		ITA	RTB		# PUSHDOWN  16D,18D,22D-26D,28D
			28D		; Save return address
			TRANSP1		; Transpose operation: EREF WRT NB2
		VLOAD	MXV		; Transform X-axis of navigation base
			XNB
			NB2NB1		; Apply optics transformation matrix
		VSL1
		STOVL	XNB1		; Store transformed X
			YNB		; Load Y-axis of navigation base
		MXV	VSL1		; Transform Y-axis
			NB2NB1
		STOVL	YNB1		; Store transformed Y
			ZNB		; Load Z-axis of navigation base
		MXV	VSL1		; Transform Z-axis
			NB2NB1
		STORE	ZNB1		; Store transformed Z (all axes now in optics)

		RTB	RTB		; Matrix transpose operations
			TRANSP1
			TRANSP2

; With navigation base axes now expressed in optics coordinates, compute the
; shaft and trunnion angles using vector cross products and dot products to
; extract angular components. The shaft angle (SA) is computed first, followed
; by the trunnion angle (TA). Overflow conditions handle gimbal limit cases.

SXTANG1		VLOAD	VXV		; Compute shaft angle from cross product
			ZNB1		; Z-axis in optics coordinates
			STAR		; Cross with star vector
		BOV			; Check for overflow (vectors parallel)
			+1
		UNIT	BOV		; Normalize to unit vector
			ZNB=S1		; If parallel, use special case (270 deg)
		STORE	PDA		# PDA = UNIT(ZNB X S)

		DOT	DCOMP		; Extract sine component of shaft angle
			XNB1		; Dot product with X-axis, complement
		STOVL	SINTH		# SIN(SA) = PDA . -XNB
			PDA		; Reload perpendicular vector

		DOT			; Extract cosine component of shaft angle
			YNB1		; Dot product with Y-axis
		STCALL	COSTH		# COS(SA) = PDA . YNB
			ARCTRIG		; Compute shaft angle from sin/cos
# Page 288
; Shaft angle (SA) computed successfully above. Now compute trunnion angle (TA)
; by examining the star vector's alignment with the optics Z-axis. The trunnion
; angle measures the sextant's up/down rotation. Valid range is 0 to 90 degrees;
; out-of-range values indicate the star cannot be viewed with current gimbal limits.

		RTB			; Store shaft angle
			1STO2S		; Double precision conversion
		STOVL	SAC		; SAC = shaft angle result (double precision)
			STAR		; Reload star vector in optics coordinates
		BOV			; Check for overflow
			+1
		DOT	SL1		; Compute S . ZNB (cosine of trunnion angle)
			ZNB1		; Dot product with Z-axis, scale left 1
		ACOS			; ACOS to get trunnion angle
		BMN	SL2		; Branch if negative (invalid, below horizon)
			SXTALARM	# TRUNNION ANGLE NEGATIVE
		BOV	DSU		; Branch if overflow (angle > 90 degrees)
			SXTALARM	# TRUNNION ANGLE GREATER THAN 90 DEGREES
			20DEG-		; Subtract 19.775 degree optics offset
		RTB			; Valid trunnion angle computed
			1STO2S		; Convert to double precision
		STORE	PAC		# FOR FLIGHT USE, CULTFLAG IS ON IF
		CLRGO			# TRUNION IS GREATER THAN 90 DEG
			CULTFLAG	; Clear gimbal limit flag (star is visible)
			28D		; Return to caller

; Out-of-limits handling: Star position exceeds sextant mechanical gimbal range.
; Set default angles (shaft=270°, trunnion=19.775°) and flag the condition.
; During Apollo 11, crew would manually reposition spacecraft to bring star
; into sextant field of view if automatic tracking encountered this limit.

SXTALARM	SETGO			# ALARM HAS BEEN REMOVED FROM THIS
			CULTFLAG	; Set flag indicating gimbal limit reached
			28D		# SUBROUTINE,ALARM WILL BE SET BY MPI

; Special case: Star vector parallel to optics Z-axis (gimbal lock condition).
; Assign default shaft angle 270 degrees since rotation about Z is undefined.

ZNB=S1		DLOAD			; Handle ZNB parallel to STAR case
			270DEG		; Default shaft angle for gimbal singularity
		STODL	SAC		; Store 270 degrees as shaft angle
			20DEGS-		; Load 19.775 degree offset (negative form)
		STORE	PAC		; Store as trunnion angle
		CLRGO			; Clear gimbal limit flag
			CULTFLAG	; (This is a mathematical singularity, not limit)
			28D		; Return to caller
# Page 289
; ============================================================================
; TRANSITION: From optical navigation geometry to state vector computations
;
; The sextant geometry routines above enabled precise star sighting for
; navigation updates. These next routines prepare the resulting state vectors
; (position and velocity) for telemetry downlink to Mission Control in Houston.
; During Apollo 11, ground controllers monitored the spacecraft trajectory
; continuously, and these routines packaged navigation data for transmission.
; ============================================================================

# THESE TWO ROUTINES COMPUTE THE ACTUAL STATE VECTOR FOR LM, CSM BY ADDING
# THE CONIC R,V AND THE DEVIATIONSR,V.  THE STATE VECTORS ARE CONVERTED TO
# METERS B-29 AND METERS/CSEC B-7 AND STORED APPROPRIATELY IN RN,VN OR
# R-OTHER , V-OTHER FOR DOWNLINK.  THE ROUTINES NAMES ARE SWITCHED IN THE
# OTHER VEHICLES COMPUTER.
#
# INPUT
#	STATE VECTOR IN TEMPORARY STORAGE AREA
#	IF STATE VECTOR IS SCALED POS B27 AND VEL B5
#		SET X2 TO +2
#	IF STATE VECTOR IS SCALED POS B29 AND VEL B7
#		SET X2 TO 0
#
# OUTPUT
#	R(T) IN RN, V(T) IN VN, T IN PIPTIME
#		OR
#	R(T) IN R-OTHER, V(T) IN V-OTHER	(T IS DEFINED BY T-OTHER)


		BANK	23
		SETLOC	COMGEOM2
		BANK
		COUNT	10/GEOM

; SVDWN1: State Vector Downlink preparation for primary spacecraft (CSM in this
; computer, LM in the other vehicle's AGC). Combines conic trajectory prediction
; with navigation deviations, then scales position and velocity to standard units
; for telemetry transmission to Mission Control. During Apollo 11's translunar
; coast and lunar orbit, ground controllers used this data to verify trajectory.

SVDWN1		BOF	RVQ			# SW=1=AVETOMID DOING W-MATRIX INTEG
			AVEMIDSW	; Check if W-matrix integration active
			+1		; If active, skip update (RVQ returns)
		VLOAD	VSL*		; Load position deviation vector
			TDELTAV		; TDELTAV = position corrections (delta-R)
			0	-7,2	; Scale shift by X2 parameter (B-29 or B-27)
		VAD	VSL*		; Add to conic position
			RCV		; RCV = conic trajectory position
			0,2		; Scale to final B-29 meters
		STOVL	RN		; RN = updated position for this vehicle
			TNUV		; Load velocity deviation vector
		VSL*	VAD		; Scale and add velocity corrections
			0	-4,2	; Scale shift velocity (B-7 or B-5)
			VCV		; VCV = conic trajectory velocity
		VSL*			; Scale to final B-7 meters/centisecond
			0,2
		STODL	VN		; VN = updated velocity for this vehicle
			TET		; Load trajectory epoch time
		STORE	PIPTIME		; PIPTIME = time of state vector validity
		RVQ			; Return to caller

; SVDWN2: State Vector Downlink preparation for other spacecraft (LM as seen from
; CSM computer, or CSM as seen from LM computer). Uses same scaling and combination
; logic as SVDWN1 but stores result in R-OTHER/V-OTHER for rendezvous computations.
; Critical during Apollo 11's lunar orbit rendezvous when Eagle and Columbia tracked
; each other's positions for docking alignment.

SVDWN2		VLOAD	VSL*		; Load position deviation vector
			TDELTAV		; Same source data as SVDWN1
			0	-7,2	; Scale position corrections
		VAD	VSL*		; Add to conic position
			RCV		; Conic trajectory position
# Page 290
			0,2		; Scale to final B-29 meters
		STOVL	R-OTHER		; R-OTHER = other vehicle's position
			TNUV		; Load velocity deviation vector
		VSL*	VAD		; Scale and add velocity corrections
			0	-4,2	; Scale velocity corrections
			VCV		; Conic trajectory velocity
		VSL*			; Scale to final B-7 meters/centisecond
			0,2
		STORE	V-OTHER		; V-OTHER = other vehicle's velocity
		RVQ			; Return (time stored separately by caller)
# Page 291
; ============================================================================
; TRANSITION: From state vector computations to mathematical utility subroutines
;
; The geometry computations above required various mathematical functions.
; This section provides the natural logarithm subroutine used in trajectory
; calculations and other numerical algorithms throughout the guidance software.
; ============================================================================

# SUBROUTINE TO COMPUTE THE NATURAL LOG OF C(MPAC, MPAC +1).
#
#	ENTRY:	CALL
#			LOG
#
# SUBROUTINE RETURNS WITH -LOG IN DP MPAC.
#
# EBANK IS ARBITRARY..

		BANK	14
		SETLOC	POWFLIT2
		BANK
		COUNT	23/GEOM

; LOG: Natural logarithm computation using polynomial approximation.
; Algorithm: Normalize argument to range [0.5, 1.0] by shifting, compute log
; of normalized value using polynomial series, then add back log of shift factor.
; Returns negative logarithm in double-precision MPAC (ln(x) with sign flipped).
; Used in trajectory calculations requiring exponential decay or growth modeling.

LOG		NORM	BDSU		# GENERATES LOG BY SHIFTING ARG
			MPAC	+3	# UNTIL IT LIES BETWEEN .5 AND 1.
			NEARLY1		# THE LOG OF THIS PART IS FOUND AND THE
		EXIT			# LOG OF THE SHIFTED PART IS COMPUTED

		TC	POLY		# AND ADDED IN.  SHIFT COUNT STORED
					; Polynomial evaluation of log series
		DEC	2		# (N-1, SUPPLIED BY SMERZH)
		2DEC	0		# IN MPAC +3.
		2DEC	.031335467	; Polynomial coefficient a2
		2DEC	.0130145859	; Polynomial coefficient a1
		2DEC	.0215738898	; Polynomial coefficient a0

		CAF	ZERO		; Clear MPAC +2 for multiplication
		TS	MPAC	+2
		EXTEND			; Begin double-precision operations
		DCA	CLOG2/32	; Load ln(2)/32 constant
		DXCH	MPAC		; Position for multiplication
		DXCH	MPAC	+3	; Retrieve shift count
		COM			# LOAD POSITIVE SHIFT COUNT IN A.
		TC	SHORTMP		# MULTIPLY BY SHIFT COUNT.
					; Computes (shift count) * ln(2)/32

		DXCH	MPAC	+1	; Rearrange results
		DXCH	MPAC		; Position final sum
		DXCH	MPAC	+3
		DAS	MPAC		; Add polynomial result + shift correction
		TC	INTPRET		# RESULT IN MPAC, MPAC +1
					; Return to interpretive mode
		RVQ			; Return with -ln(x) in MPAC

; Mathematical constants for LOG subroutine
NEARLY1		2DEC	.999999999	; Upper normalization threshold

# Page 292
CLOG2/32	2DEC	.0216608494	; ln(2)/32 = 0.0216608494 for shift scaling

# Page 293
; ============================================================================
; TRANSITION: From mathematical utilities to Earth rotation tracking
;
; The LOG subroutine above provided computational precision for various
; algorithms. These EARTH ROTATOR routines solve the problem of predicting
; where a target location on Earth will be after the spacecraft coasts through
; space. During Apollo 11's transearth coast (July 21-24, 1969), Earth rotated
; beneath Columbia's trajectory. These routines computed where the landing site
; would be when the Command Module arrived 60 hours later, accounting for
; Earth's 15.04 degrees/hour rotation. Essential for targeting the Pacific
; Ocean recovery zone near 13.3°N 169.15°W where USS Hornet waited on July 24.
; ============================================================================

# SUBROUTINE NAME: 	EARTH ROTATOR	(EARROT1 OR EARROT2)		DATE:  		15 FEB 67
# MOD NO:  N +1								LOG SECTION:  	POWERED FLIGHT SUBROS
# MOD BY:  ENTRY GROUP (BAIRNSFATHER)
# FUNCTIONAL DESCRIPTION: 	THIS ROUTINE PROJECTS THE INITIAL EARTH TARGET VECTOR RTINIT AHEAD THROUGH
#	THE ESTIMATED TIME OF FLIGHT.  INITIAL CALL RESOLVES THE INITIAL TARGET VECTOR RTINIT INTO EASTERLY
#	AND NORMAL COMPONENTS RTEAST AND RTNORM .  INITIAL AND SUBSEQUENT CALLS ROTATE THIS VECTOR
#	ABOUT THE (FULL) UNIT POLAR AXIS UNITW THROUGH THE ANGLE WIE DTEAROT TO OBTAIN THE ROTATED
#	TARGET VECTOR RT .  ALL VECTORS EXCEPT UNITW ARE HALF UNIT.
#	THE EQUATIONS ARE
#		-    -        -                      -
#		RT = RTINIT + RTNORM (COS(WT) - 1) + RTEAST SIN(WT)
#	WHERE	WT = WIE DTEAROT
#		RTINIT = INITIAL TARGET VECTOR
#		-        -       -
#		RTEAST = UNITW*RTINIT
#		-        -        -
#		RTNORM = RTEAST*UNITW
#
#	FOR CONTINUOUS UPDATING, ONLY ONE ENTRY TO EARROT1 IS REQUIRED, WITH SUBSEQUENT ENTRIES AT EARROT2.
# CALLING SEQUENCE:	FIRST CALL			SUBSEQUENT CALL
#			STCALL	DTEAROT			STCALL	DTEAROT
#				EARROT1				EARROT2
#			C(MPAC) UNSPECIFIED		C(MPAC) = DTEAROT
#	PUSHLOC = PDL+0, ARBITRARY.  6 LOCATIONS USED.
#
# SUBROUTINES USED:  NONE
# NORMAL EXIT MODES:  RVQ
# ALARMS:  NONE
# OUTPUT:  RTEAST (-1)		.5 UNIT VECTOR EAST, COMPNT OF RTINIT	LEFT BY FIRST CALL
#	   RTNORM (-1)		.5 UNIT VECTOR NORML, COMPNT OF RTINIT	LEFT BY FIRST CALL
#	   RT	  (-1)		.5 UNIT TARGET VECTOR, ROTATED		LEFT BY ALL CALLS
#	   DTEAROT  (-28) CS	MAY BE CHANGED BY EARROT2, IF OVER 1 DAY
# ERASABLE INITIALIZATION REQUIRED:
#	   UNITW  (0)		UNIT POLAR VECTOR			PAD LOADED
#	   RTINIT (-1)		.5 UNIT INITIAL TARGET VECTOR		LEFT BY ENTRY
#	   DTEAROT  (-28) CS	TIME OF FLIGHT				LEFT BY CALLER
# DEBRIS:  QPRET, PDL+0 ... PDL+5
# Page 294
		EBANK=	RTINIT

; EARROT1: Earth target vector rotation - initial call.
; Decomposes the initial landing site target vector into easterly and normal
; components, then rotates it through Earth's rotation angle to predict where
; the landing zone will be at the end of the coast phase. During Apollo 11's
; return from the Moon, the Pacific splashdown point rotated ~902 degrees
; (2.5 full rotations) during the 60-hour transearth coast. This routine
; enabled precise targeting despite Earth rotating beneath the trajectory.

EARROT1		VLOAD	VXV		; Load polar axis and initial target vector
			UNITW		# FULL UNIT VECTOR (Earth's rotation axis)
			RTINIT		# .5 UNIT (initial landing site target vector)
		STORE	RTEAST		# .5 UNIT (easterly component of target)
;
; Decompose initial target vector into components perpendicular to rotation axis.
; RTEAST = easterly component (cross product of polar axis with target vector).
; During Apollo 11's return, RTINIT pointed toward the Pacific recovery zone.
; The easterly component captures how far "around" Earth the target is.
;
		VXV			; Compute normal component
			UNITW		# FULL UNIT (cross with polar axis again)
		STODL	RTNORM		# .5 UNIT (normal component of target)
			DTEAROT		# (-28) CS (time of flight in centiseconds)
;
; RTNORM = normal component (perpendicular to both polar axis and RTEAST).
; These two components (RTEAST and RTNORM) form a coordinate system that rotates
; with Earth. Now we can rotate the target through angle WIE*DTEAROT.
;

; EARROT2: Earth target vector rotation - subsequent calls.
; Rotates the previously decomposed target vector through the rotation angle.
; This entry point is used for continuous updates during the coast phase without
; re-decomposing the vector. Called repeatedly as time of flight estimate updates.

EARROT2		BOVB	DDV		; Check for overflow, divide time by rotation rate
			TCDANZIG	# RESET OVFIND, IF ON
			1/WIE		; 1/WIE = reciprocal of Earth rotation rate (rad/cs)
		BOV	PUSH		; Check if rotation exceeds one day
			OVERADAY	; Handle rotation > 24 hours specially
		COS	DSU		; Compute cos(rotation angle) - 0.5
			HIDPHALF	; HIDPHALF = 0.5 DP constant
		VXSC	PDDL		# XCH W PUSH LIST (scale normal component)
			RTNORM		# .5 UNIT (multiply by (cos(angle)-0.5))
		SIN	VXSC		; Compute sin(rotation angle), scale easterly component
			RTEAST		# .5 UNIT (multiply by sin(angle))
		VAD	VSL1		; Add scaled components, shift left
		VAD	UNIT		# INSURE THAT RT IS 'UNIT'.
			RTINIT		# .5 UNIT (add initial vector)
		STORE	RT		# .5 UNIT TARGET VECTOR (rotated landing site)
;
; This implements: RT = RTINIT + RTNORM*(cos(WT)-1) + RTEAST*sin(WT)
; where WT = WIE*DTEAROT (rotation angle = Earth rate * time of flight).
; The cosine and sine terms rotate the target vector around Earth's polar axis.
; Final UNIT operation ensures numerical precision maintains unit vector length.
; Result RT points to where the landing zone will be when spacecraft arrives.
;
		RVQ			; Return with rotated target vector in MPAC

; OVERADAY: Handle rotation angles exceeding one day.
; When time of flight exceeds approximately 24 hours, the rotation angle
; becomes large enough to cause numerical precision issues. This routine
; subtracts out full day increments (360 degrees) from DTEAROT, reducing
; the angle to its equivalent value within one rotation period.
; During Apollo 11's 60-hour transearth coast, this correction was applied
; multiple times as the spacecraft traveled from lunar orbit back to Earth.

OVERADAY	DLOAD	SIGN		; Load 1/WIE with sign of DTEAROT
			1/WIE		; Reciprocal rotation rate (one full rotation period)
			DTEAROT		; Time of flight (may exceed 24 hours)
		BDSU			; Subtract (with borrow for double precision)
			DTEAROT		; Reduce DTEAROT by one full day equivalent
		STORE	DTEAROT		; Store reduced time value
;
; This effectively computes: DTEAROT = DTEAROT - sign(DTEAROT)*2*PI/WIE
; Removes one full rotation (360 degrees) while preserving rotation direction.
; Maintains numerical accuracy for multi-day coast phases.
;
		GOTO			; Return to continue rotation calculation
			EARROT2		; Re-enter with reduced angle

; ============================================================================
; CSM GEOMETRY CONSTANTS AND COORDINATE TRANSFORMATION MATRICES
;
; This section defines critical constants for Command/Service Module geometry
; calculations throughout the Apollo 11 mission. Includes Earth rotation rate,
; optical system coordinate transformations, and angular calibration values.
; ============================================================================

; Earth Rotation Parameters:
; WIE = Earth's sidereal rotation rate = 0.1901487997 revolutions/day
;     = 7.292115 × 10^-5 radians/second (Earth rotates 15.04 degrees/hour)
; During Apollo 11's transearth coast (July 21-24), Earth completed 2.5 full
; rotations beneath Columbia's trajectory. These constants enabled precise
; prediction of the Pacific splashdown zone location.

#WIE		2DEC	.1901487997	; Earth rotation rate (revolutions per day)
1/WIE		2DEC	8616410		; Reciprocal: seconds per revolution (86164.1 sec)

; Navigation Base to Optics Coordinate Transformation Matrix (NB2NB1):
; Transforms vectors from navigation base coordinates to optics coordinates.
; The sextant and telescope optical systems are mounted at specific angles
; relative to the spacecraft navigation base. This 3×3 rotation matrix accounts
; for the mounting geometry, particularly the ~32.5 degree offset angle.
; Values represent direction cosines for coordinate frame rotation.

NB2NB1		2DEC	+.8431756920 B-1	; Matrix element [1,1] = cos(32.5°)
		2DEC	0			; Matrix element [1,2] = 0
		2DEC	-.5376381241 B-1	; Matrix element [1,3] = -sin(32.5°)
# Page 295
ZERINFLT	2DEC	0			; Zero vector element (row 2 start)
HALFNFLT	2DEC	.5			; Matrix element [2,2] = 1.0 (scaled by 0.5)
		2DEC	0			; Matrix element [2,3] = 0
		2DEC	+.5376381241 B-1	; Matrix element [3,1] = sin(32.5°)
		2DEC	0			; Matrix element [3,2] = 0
		2DEC	+.8431756920 B-1	; Matrix element [3,3] = cos(32.5°)

; Optics to Navigation Base Coordinate Transformation Matrix (NB1NB2):
; Inverse transformation matrix converting optics coordinates back to navigation
; base coordinates. Used when processing sextant shaft/trunnion angles to
; convert star line-of-sight vectors from optical reference frame to navigation
; frame for trajectory calculations. This inverse matrix is the transpose of
; NB2NB1 (rotation matrices are orthogonal, so inverse equals transpose).

NB1NB2		2DEC	+.8431756920 B-1	; Matrix element [1,1] = cos(32.5°)
		2DEC	0			; Matrix element [1,2] = 0
		2DEC	+.5376381241 B-1	; Matrix element [1,3] = sin(32.5°)
		2DEC	0			; Matrix element [2,1] = 0
		2DEC	.5			; Matrix element [2,2] = 1.0 (scaled)
		2DEC	0			; Matrix element [2,3] = 0
		2DEC	-.5376381241 B-1	; Matrix element [3,1] = -sin(32.5°)
		2DEC	0			; Matrix element [3,2] = 0
		2DEC	+.8431756920 B-1	; Matrix element [3,3] = cos(32.5°)

# Page 296
; Angular Calibration Constants:
; These constants correct for optical system mounting offsets and provide
; reference angles for sextant and telescope positioning calculations.

10DEGS-		DEC	3600		; -10 degrees in AGC angle units (360 units/deg)
					; Used for 19.775 degree sextant offset correction

270DEG		OCT	60000		# SHAFT 270 DEGREES	2S COMP.
		OCT	00000		; 270 degrees shaft angle (octal representation)
					; Reference position for sextant shaft calibration

20DEGS-		DEC	-07199		; -20 degrees in AGC double-precision angle units
		DEC	-00000		; Used for telescope pointing offset corrections

20DEG-		DEC	03600		; +20 degrees (single component, compare to 20DEGS-)
		DEC	00000		; Angular reference for optical system geometry
