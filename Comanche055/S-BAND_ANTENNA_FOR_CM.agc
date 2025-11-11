# Copyright:	Public domain.
# Filename:	S-BAND_ANTENNA_FOR_CM.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim.lawton@gmail.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	934-935
# Mod history:	2009-05-11 JVL	Adapted from the Colossus249/ file
#				of the same name, using Comanche055 page
#				images.
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
#    Assemble revision 055 of AGC program Comanche by NASA
#    2021113-051.  10:28 APR. 1, 1969
#
#    This AGC program shall also be referred to as
#            Colossus 2A

# Page 934
# S-BAND ANTENNA FOR CM

; ============================================================================
; FILE: S-BAND_ANTENNA_FOR_CM.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: all-phases
;
; TL;DR: S-band high-gain antenna Earth tracking and pointing algorithm for
;        Command Module communications. Computes antenna gimbal angles to
;        maintain communication link with Earth as spacecraft attitude changes,
;        essential for voice/telemetry downlink and command uplink throughout
;        Apollo 11 mission.
;
; COMMENT-ONLY READERS: This program kept the main radio antenna pointed at
;        Earth so Mission Control could stay in contact with the crew.
; CODE-ALONG READERS: Study antenna pointing geometry, gimbal angle computation
;        from Earth direction vector, and communication link maintenance algorithms.
; ============================================================================

		BANK	23
		SETLOC	SBAND
		BANK

		COUNT*	$$/R05
		EBANK=	EMSALT

; ============================================================================
; S-BAND ANTENNA POINTING ROUTINE (Verb 64 Entry Point)
;
; The S-band high-gain antenna provides the primary communication link between
; the Command Module and Earth. As the spacecraft maneuvers and travels through
; space, this routine continuously computes the gimbal angles needed to keep
; the antenna dish pointed precisely at Earth for voice, telemetry, and command
; transmissions. During Apollo 11, this maintained contact with Mission Control
; throughout the mission except during periods behind the Moon.
; ============================================================================

SBANDANT	TC	BANKCALL	# V 64 E GETS US HERE
		CADR	R02BOTH		# CHECK IF IMU IS ON AND ALIGNED
		TC	INTPRET
		RTB	CALL
			LOADTIME	# PICKUP CURRENT TIME SCALED B-28
			CDUTRIG		# COMPUTE SINES AND COSINES OF CDU ANGLES
		STCALL	TDEC1		# ADVANCE INTEGRATION TO TIME IN TDEC1
			CSMCONIC	# USING CONIC INTEGRATION
; Determine if spacecraft is in Earth's or Moon's sphere of influence.
; This affects the coordinate system used for computing the direction to Earth.
; During translunar coast, the reference switches from Earth-centered to
; Moon-centered, and during transearth return it switches back.

		SLOAD	BHIZ		# ORIGIN OF REFERENCE INERTIAL SYSTEM IS
			X2		# EARTH = 0, MOON = 2
			EISOI
		VLOAD
			RATT
		STORE	RCM		# MOVE RATT TO PREVENT WIPEOUT
		DLOAD	CALL		# MOON, PUSH ON
			TAT		# GET ORIGINAL TIME
			LUNPOS		# COMPUTE POSITION VECTOR OF MOON

; In lunar sphere of influence: compute Earth direction as negative of
; (Moon position + spacecraft position relative to Moon).
; This gives the vector from spacecraft to Earth.

		VAD	VCOMP		# R= -(REM+RCM) = NEG. OF S/C POS. VEC
			RCM
		GOTO
			EISOI +2

; In Earth sphere of influence: Earth direction is simply negative of
; spacecraft position vector (pointing from spacecraft toward Earth).

EISOI		VLOAD	VCOMP		# EARTH, R= -RCM
			RATT
; ============================================================================
; COORDINATE TRANSFORMATION: Inertial to Navigation Base Frame
;
; The Earth direction vector computed above is in inertial coordinates.
; To determine antenna gimbal angles, we must transform this vector into
; the spacecraft's navigation base coordinate system, which accounts for
; the current spacecraft attitude. This transformation uses the REFSMMAT
; (stable member matrix) and the current IMU gimbal angles.
; ============================================================================

		SETPD	MXV		# RCS TO STABLE MEMBER- B-1X B-29X B+1
			2D		# 2D
			REFSMMAT	# STABLE MEMBER.  B-1X B-29X B+1= B-29
		VSL1	PDDL		# 8D
			HI6ZEROS
		STOVL	YAWANG		# ZERO OUT YAWANG, SET UP FOR SMNB
			RCM		# TRANSFORMATION.  SM COORD.  SCALED B-29
		CALL
			*SMNB*
		STORE	R		# SAVE NAV. BASE COORDINATES
; ============================================================================
; ANTENNA GIMBAL ANGLE COMPUTATION
;
; The S-band antenna has two gimbal axes: yaw and pitch. We compute these
; angles from the Earth direction vector in navigation base coordinates.
; The yaw angle rotates the antenna in the horizontal plane (about the Z axis),
; while the pitch angle tilts it up or down. Together, these two angles aim
; the antenna dish precisely at Earth regardless of spacecraft attitude.
; ============================================================================

; First, compute unit vector pointing to Earth and project it into the
; XY plane (perpendicular to spacecraft Z axis). This projection is used
; to determine the yaw angle.

		UNIT	PDVL		# 14D
			R
		VPROJ	VSL2		# COMPUTE PROJECTION OF VECTOR INTO CM
			HIUNITZ		# XY-PLANE, R-(R.UZ)UZ
		BVSU	BOV		# CLEAR OVERFLOW INDICATOR IF SET
			R
			COVCNV
COVCNV		UNIT	BOV		# TEST OVERFLOW FOR INDICATION OF NULL
			NOADJUST	# VECTOR
; Compute yaw angle from the projection of Earth direction into XY plane.
; Yaw angle = arccos(URP · UX) where URP is the unit vector in XY plane
; pointing to Earth, and UX is the spacecraft +X axis. If the Y component
; is negative, the angle is in the range 180-360 degrees, so we compute
; 360 - angle to get the correct gimbal position.

		PUSH	DOT		# 20D
# Page 935
			HIUNITX		# COMPUTE YAW ANGLE = ACOS (URP.UX)
		SL1	ACOS		# REVOLUTIONS SCALED B0
		PDVL	DOT		# 22D YAWANG
			URP
			HIUNITY		# COMPUTE FOLLOWING- URP.UY
		SL1	BPL		# POSITIVE
			NOADJUST	# YES, 0- 180 DEGREES
		DLOAD	DSU		# NO, 181-360 DEGREES 20D
			DPPOSMAX	# COMPUTE 2 PI MINUS YAW ANGLE
		PUSH			# 22D YAWANG
; Compute pitch angle from the full Earth direction vector.
; Pitch angle = arccos(UR · UZ) - 90 degrees, where UR is the unit vector
; pointing to Earth and UZ is the spacecraft +Z axis. The 90-degree offset
; accounts for the antenna's mounting geometry on the Command Module.

NOADJUST	VLOAD	DOT		# COMPUTE PITCH ANGLE
			UR		# ACOS (UR.UZ) - PI/2
			HIUNITZ
		SL1	ACOS		# REVOLUTIONS B0
		DSU
			HIDP1/4
		STODL	RHOSB
			YAWANG
		STORE	GAMMASB		# PATCH FOR CHECKOUT
; ============================================================================
; CREW DISPLAY AND MANUAL ANTENNA POINTING
;
; The computed yaw and pitch angles are displayed to the crew on the DSKY
; using Verb 06 Noun 51. The crew can use these angles to manually position
; the antenna gimbal if automatic tracking is not available, or to verify
; that the automatic system is working correctly. During Apollo 11, maintaining
; this antenna lock was critical for voice communication with Mission Control,
; especially during lunar orbit when the spacecraft was frequently reorienting
; for photography and navigation sightings.
; ============================================================================

		EXIT
		CA	EXTVBACT	# IS BIT 5 STILL ON
		MASK	BIT5
		EXTEND
		BZF	ENDEXT		# NO, WE HAVE BEEN ANSWERED
		CAF	V06N51		# DISPLAY ANGLES
		TC	BANKCALL
		CADR	GOMARKFR
		TC	B5OFF		# TERMINATE
		TC	B5OFF
		TC	ENDOFJOB	# RECYCLE
		CAF	BIT3		# IMMEDIATE RETURN
		TC	BLANKET		# BLANK R3
		CAF	BIT1		# DELAY MINIMUM TIME TO ALLOW DISPLAY IN
		TC	BANKCALL
		CADR	DELAYJOB
		TCF	SBANDANT +2
; Display format: V06N51 shows yaw angle (R1) and pitch angle (R2) in revolutions.
; Crew can read these values and manually adjust antenna gimbals if needed.

V06N51		VN	0651

; Temporary storage locations used during antenna pointing calculations:
; RCM    = Spacecraft position vector (Command Module position)
; UR     = Unit vector pointing from spacecraft to Earth
; URP    = Projection of UR into spacecraft XY plane
; YAWANG = Computed yaw gimbal angle (rotation about Z axis)
; PITCHANG = Computed pitch gimbal angle (rotation perpendicular to Z)
; R      = Alias for RCM (same storage location)

RCM		EQUALS	2D
UR		EQUALS	8D
URP		EQUALS	14D
YAWANG		EQUALS	20D
PITCHANG	EQUALS	22D
R		EQUALS	RCM
		SBANK=	LOWSUPER
