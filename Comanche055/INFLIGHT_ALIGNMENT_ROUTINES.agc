# Copyright:	Public domain.
# Filename:	INFLIGHT_ALIGNMENT_ROUTINES.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1355-1364
# Mod history:	2009-05-14 RSB	Adapted from the Colossus249/ file of the
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

# Page 1355
; ============================================================================
; FILE: INFLIGHT_ALIGNMENT_ROUTINES.agc
; MODULE: CHIEFTAN Subsystem (Core OS)
; MISSION PHASE: all-phases
;
; TL;DR: In-flight IMU alignment procedures enabling platform realignment without
;        shutdown. Implements gyrocompass and optical alignment modes allowing
;        IMU reorientation during flight to correct drift or establish new
;        reference frames for mission phase transitions.
;
; COMMENT-ONLY READERS: This program realigned the navigation platform during
;        flight without having to shut it down.
; CODE-ALONG READERS: Study in-flight alignment algorithms, gyrocompass procedures,
;        platform reorientation without power-down, alignment mode transitions.
; ============================================================================

		BANK	22
		SETLOC	INFLIGHT
		BANK

		EBANK=	XSM

; ============================================================================
; TRANSITION: Gyro Torque Angle Computation
;
; The Inertial Measurement Unit's stable platform can drift during long missions.
; To correct this drift without shutting down the IMU, the spacecraft computer
; must calculate precise torquing angles for each of the three gyroscopes. This
; section computes those angles to physically reorient the platform to match
; the desired orientation.
; ============================================================================

# CALCGTA COMPUTES THE GYRO TORQUE ANGLES REQUIRED TO BRING THE STABLE MEMBER INTO THE DESIRED ORIENTATION.
#
# THE INPUT IS THE DESIRED STABLE MEMBER COORDINATES REFERRED TO PRESENT STABLE MEMBER COORDINATES. THE THREE
# HALF-UNIT VECTORS ARE STORED AT XDC, YDC, AND ZDC.
#
# THE OUTPUTS ARE THE THREE GYRO TORQUING ANGLES TO BE APPLIED TO THE Y, Z, AND X GYROS AND ARE STORED DP AT IGC,
# MGC, AND OGC RESPECTIVELY.

		COUNT	23/INFLT

; CALCGTA routine entry: Computes three gyro torquing angles (IGC, MGC, OGC)
; to physically reorient the stable member platform. These angles represent
; the amount each gyroscope must be torqued to bring the current platform
; orientation into alignment with the desired orientation.
;
; TECHNICAL DETAILS for code-along readers:
; - Uses vector algebra to compute rotation angles
; - XDC, YDC, ZDC are half-unit vectors defining desired orientation
; - Outputs IGC (inner gimbal), MGC (middle gimbal), OGC (outer gimbal)
; - All angles stored as fractions of revolution (0.5 = 180 degrees)

CALCGTA		ITA	DLOAD		# PUSHDOWN  00-03,16D-27D,34D-37D
			S2		# XDC = (XD1 XD2 XD3)
			XDC		# YDC = (YD1 YD2 YD3)
		PDDL	PDDL		# ZDC = (ZD1 ZD2 ZD3)
			HI6ZEROS
			XDC 	+4
		DCOMP	VDEF
		UNIT
		STODL	ZPRIME		# ZP = UNIT(-XD3 0 XD1) = (ZP1 ZP2 ZP3)
			ZPRIME

		SR1
		STODL	SINTH		# SIN(IGC) = ZP1
			ZPRIME 	+4
		SR1
		STCALL	COSTH		# COS(IGC) = ZP3
			ARCTRIG

		STODL	IGC		# Y GYRO TORQUING ANGLE   FRACTION OF REV.
			XDC 	+2
		SR1
		STODL	SINTH		# SIN(MGC) = XD2
			ZPRIME

		DMP	PDDL
			XDC 	+4	# PD00 = (ZP1)(XD3)
			ZPRIME 	+4

		DMP	DSU
			XDC		# MPAC = (ZP3)(XD1)
		STADR
		STCALL	COSTH		# COS(MGC) = MPAC - PD00
			ARCTRIG
# Page 1356
		STOVL	MGC		# Z GYRO TORQUING ANGLE   FRACTION OF REV.
			ZPRIME
		DOT
			ZDC
		STOVL	COSTH		# COS(OGC) = ZP . ZDC
			ZPRIME
		DOT
			YDC
		STCALL	SINTH		# SIN(OGC) = ZP . YDC
			ARCTRIG

		STCALL	OGC		# X GYRO TORQUING ANGLE   FRACTION OF REV.
			S2

# Page 1357
; ============================================================================
; TRANSITION: Angle Computation from Trigonometric Components
;
; Once the computer has calculated the sine and cosine values for a gyro
; torquing angle, it must determine the actual angle value. This arctangent-
; style computation handles all four quadrants correctly, ensuring the gyros
; receive the proper torquing commands to reorient the platform.
; ============================================================================

# ARCTRIG COMPUTES AN ANGLE GIVEN THE SINE AND COSINE OF THIS ANGLE.
#
# THE INPUTS ARE SIN/4 AND COS/4 STORED DP AT SINTH AND COSTH.
#
# THE OUTPUT IS THE CALCULATED ANGLE BETWEEN +.5 AND -.5 REVOLUTIONS AND STORED AT THETA. THE OUTPUT IS ALSO
# AVAILABLE AT MPAC.

; ARCTRIG routine: Arctangent-style angle computation from sine and cosine.
; This handles all four quadrants properly, unlike simple arcsin or arccos.
;
; TECHNICAL DETAILS for code-along readers:
; - Inputs: SINTH (sine/4), COSTH (cosine/4) in double precision
; - Output: THETA (angle as fraction of revolution, range ±0.5)
; - Handles special cases: zero crossings, sign changes, quadrant boundaries
; - Uses conditional logic to select ACOS or ASIN based on angle magnitude
; - Result also available in MPAC for immediate use

ARCTRIG		DLOAD	ABS		# PUSHDOWN  16D-21D
			SINTH
		DSU	BMN
			QTSN45		# ABS(SIN/4) - SIN(45)/4
			TRIG1		# IF (-45,45) OR (135,-135)

; For angles with large cosine component (near 0° or 180°), use ACOS for
; better numerical accuracy. This branch handles the 45°-135° and -135° to -45°
; ranges where sine magnitude exceeds cosine magnitude.

		DLOAD	SL1		# (45,135) OR (-135,-45)
			COSTH
		ACOS	SIGN
			SINTH
		STORE	THETA		# X = ARCCOS(COS) WITH SIGN(SIN)
		RVQ

; For angles with large sine component (near 90° or -90°), use ASIN for
; better numerical accuracy. This branch handles the -45° to 45° range and
; the 135° to -135° range where cosine magnitude is small.

TRIG1		DLOAD	SL1		# (-45,45) OR (135,-135)
			SINTH
		ASIN
		STODL	THETA		# X = ARCSIN(SIN) WITH SIGN(SIN)
			COSTH
		BMN
			TRIG2		# IF (135,-135)

		DLOAD	RVQ
			THETA		# X = ARCSIN(SIN)   (-45,45)

; Special handling for angles near ±180° where cosine is negative.
; Computes angle as ±0.5 revolution minus the arcsin value to correctly
; place angle in second or third quadrant.

TRIG2		DLOAD	SIGN		# (135,-135)
			HIDPHALF
			SINTH
		DSU
			THETA
		STORE	THETA		# X = .5 WITH SIGN(SIN) - ARCSIN(SIN)
		RVQ			#	(+) - (+) OR (-) - (-)

# Page 1358
# SMNB, NBSM, AND AXISROT, WHICH USED TO APPEAR HERE, HAVE BEEN
# COMBINED IN A ROUTINE CALLED AX*SR*T, WHICH APPEARS AMONG THE POWERED
# FLIGHT SUBROUTINES.

# Page 1359
; ============================================================================
; TRANSITION: CDU Driving Angle Computation
;
; The Coupling Data Units (CDUs) measure the actual gimbal angles of the IMU.
; To drive the stable member to a new orientation, the computer must calculate
; what CDU angles will position the gimbals correctly. This routine computes
; those driving angles by comparing the desired stable member orientation
; with the current navigation base orientation.
; ============================================================================

# CALCGA COMPUTES THE CDU DRIVING ANGLES REQUIRED TO BRING THE STABLE MEMBER INTO THE DESIRED ORIENTATION.
#
# THE INPUTS ARE  1) THE NAVIGATION BASE COORDINATES REFERRED TO ANY COORDINATE SYSTEM. THE THREE HALF-UNIT
# VECTORS ARE STORED AT XNB, YNB, AND ZNB.  2) THE DESIRED STABLE MEMBER COORDINATES REFERRED TO THE SAME
# COORDINATE SYSTEM ARE STORED AT XSM, YSM, AND ZSM.
#
# THE OUTPUTS ARE THE THREE CDU DRIVING ANGLES AND ARE STORED SP AT THETAD, THETAD +1, AND THETAD +2.

; CALCGA routine: Computes CDU (Coupling Data Unit) driving angles.
; These angles position the IMU gimbals to achieve the desired stable member
; orientation. During in-flight alignment, these CDU angles command the
; physical gimbal motors to rotate the platform smoothly into proper alignment.
;
; TECHNICAL DETAILS for code-along readers:
; - Inputs: XNB/YNB/ZNB (navigation base coords), XSM/YSM/ZSM (desired SM coords)
; - Outputs: THETAD, THETAD+1, THETAD+2 (three CDU angles in single precision)
; - Computes gimbal axis vectors via cross products
; - Uses ARCTRIG to resolve angles from trig components
; - Handles gimbal lock region (middle gimbal near 90°)

CALCGA		SETPD			# PUSHDOWN  00-05, 16D-21D, 34D-37D
			0

; Step 1: Compute middle gimbal axis (MGA) from cross product of outer and
; inner gimbal axes. XNB represents outer gimbal axis, YSM represents inner
; gimbal axis. The middle gimbal axis is perpendicular to both.

		VLOAD	VXV
			XNB		# XNB = OGA (OUTER GIMBAL AXIS)
			YSM		# YSM = IGA (INNER GIMBAL AXIS)
		UNIT	PUSH		# PD0 = UNIT(OGA X IGA) = MGA

; Step 2: Compute outer gimbal angle (OGC) by projecting middle gimbal axis
; onto navigation base Z and Y axes to get cosine and sine components.

		DOT	ITA
			ZNB
			S2
		STOVL	COSTH		# COS(OG) = MGA . ZNB
			0
		DOT
			YNB
		STCALL	SINTH		# SIN(OG) = MGA . YNB
			ARCTRIG
		STOVL	OGC
			0

; Step 3: Compute middle gimbal angle (MGC). This computation includes special
; handling for the case where middle gimbal is near 90 degrees (gimbal lock
; region). Uses cross product (MGA X OGA) dotted with IGA for cosine, and
; direct dot product IGA . OGA for sine.

		VXV	DOT		# PROVISION FOR MG ANGLE OF 90 DEGREES
			XNB
			YSM
		SL1
		STOVL	COSTH		# COS(MG) = IGA . (MGA X OGA)
			YSM
		DOT
			XNB
		STCALL	SINTH		# SIN(MG) = IGA . OGA
			ARCTRIG
		STORE	MGC

		ABS	DSU
			.166...
		BPL
			GIMLOCK1	# IF ANGLE GREATER THAN 60 DEGREES

; Step 4: Compute inner gimbal angle (IGC) by projecting desired stable
; member Z and X axes onto the middle gimbal axis. Normal path proceeds here
; if middle gimbal is in safe operating region (not near 90 degrees).

CALCGA1		VLOAD	DOT
			ZSM
			0
		STOVL	COSTH		# COS(IG) = ZSM . MGA
			XSM
# Page 1360
		DOT	STADR
		STCALL	SINTH		# SIN(IG) = XSM . MGA
			ARCTRIG

		STOVL	IGC
			OGC

; Step 5: Convert computed angles from fractions of revolution to CDU angle
; format. V1STO2S routine converts three DP angles to three SP CDU values.
; CPHIFLAG indicates whether to bypass certain CDU transformations.

		RTB	BONCLR
			V1STO2S
			CPHIFLAG
			S2
		STCALL	THETAD
			S2

; GIMBAL LOCK WARNING: If middle gimbal angle exceeds 60 degrees, the IMU
; approaches gimbal lock (singularity at 90 degrees where outer and inner
; gimbals align). This condition triggers alarm 00401 and sets the GLOKFAIL
; flag to warn the crew. The computation continues but with reduced accuracy.

GIMLOCK1	EXIT
		TC	ALARM
		OCT	00401
		TC	UPFLAG		# GIMBAL LOCK HAS OCCURRED
		ADRES	GLOKFAIL

		TC	INTPRET
		GOTO
			CALCGA1

# Page 1361
; ============================================================================
; TRANSITION: Coordinate System Transformation
;
; During in-flight alignment, the navigation platform must be reoriented to
; match a desired reference frame. To compute this transformation, the AGC
; uses star sightings observed in two different coordinate systems. AXISGEN
; constructs the transformation matrix between these coordinate systems by
; analyzing how two star vectors appear in each reference frame.
; ============================================================================

# AXISGEN COMPUTES THE COORDINATES OF ONE COORDINATE SYSTEM REFERRED TO ANOTHER COORDINATE SYSTEM.
#
# THE INPUTS ARE  1) THE STAR1 VECTOR REFERRED TO COORDINATE SYSTEM A STORED AT STARAD.  2) THE STAR2 VECTOR
# REFERRED TO COORDINATE SYSTEM A STORED AT STARAD +6.  3) THE STAR1 VECTOR REFERRED TO COORDINATE SYSTEM B STORED
# AT LOCATION 6 OF THE VAC AREA.  4) THE STAR2 VECTOR REFERRED TO COORDINATE SYSTEM B STORED AT LOCATION 12D OF
# THE VAC AREA.
#
# THE OUTPUT DEFINES COORDINATE SYSTEM A REFERRED TO COORDINATE SYSTEM B.  THE THREE HALF-UNIT VECTORS ARE STORED
# AT LOCATIONS XDC, XDC +6, XDC +12D, AND STARAD, STARAD +6, STARAD +12D.

; AXISGEN routine: Transforms coordinate system A to coordinate system B
; using star vector observations. This fundamental routine enables IMU
; realignment by computing the rotation matrix between the current platform
; orientation and the desired orientation.
;
; TECHNICAL DETAILS for code-along readers:
; - Inputs: Two star vectors (S1, S2) observed in both coordinate systems
; - Process: 1) Build orthonormal basis from cross products of star vectors
;           2) Compute transformation matrix via dot products of basis vectors
;           3) Store result at XDC, YDC, ZDC and STARAD locations
; - Uses indexed addressing (X1, X2) to iterate through vector components

AXISGEN		AXT,1	SSP		# PUSHDOWN  00-30D, 34D-37D
			STARAD 	+6
			S1
			STARAD 	-6

		SETPD
			0

; Step 1: Build orthonormal basis vectors for both coordinate systems A and B.
; The loop (AXISGEN1) executes twice, once for each coordinate system.
; For each system:
;   - Take first star vector (UA or UB)
;   - Compute cross product with second star vector to get perpendicular (VA or VB)
;   - Normalize to unit vector
;   - Compute third orthogonal vector (WA or WB) to complete right-handed basis

AXISGEN1	VLOAD*	VXV*		# 06D	UA = S1
			STARAD 	+12D,1	#	STARAD +00D	UB = S1
			STARAD 	+18D,1
		UNIT			# 12D	VA = UNIT(S1 X S2)
		STORE	STARAD 	+18D,1	#	STARAD +06D	VB = UNIT(S1 X S2)
		VLOAD*
			STARAD 	+12D,1

		VXV*	VSL1
			STARAD 	+18D,1	# 18D	WA = UA X VA
		STORE	STARAD 	+24D,1	#	STARAD +12D	WB = UB X VB

		TIX,1
			AXISGEN1

		AXC,1	SXA,1
			6
			30D

		AXT,1	SSP
			18D
			S1
			6

		AXT,2	SSP
			6
			S2
			2

; Step 2: Compute transformation matrix from system A to system B using the
; orthonormal basis vectors constructed above. The nested loop computes each
; row of the transformation matrix by taking dot products of basis vectors
; from system A with basis vectors from system B. This produces the three
; direction cosine vectors XDC, YDC, ZDC defining system A in terms of B.

AXISGEN2	XCHX,1	VLOAD*
			30D		# X1=-6 X2=+6	X1=-6 X2=+4	X1=-6 X2=+2
			0,1

# Page 1362
		VXSC*	PDVL*		# J=(UA)(UB1)	J=(UA)(UB2)	J=(UA)(UB3)
			STARAD 	+6,2
			6,1
		VXSC*
			STARAD 	+12D,2
		STOVL*	24D		# K=(VA)(VB1)	J=(VA)(VB2)	J=(VA)(VB3)
			12D,1

		VXSC*	VAD
			STARAD 	+18D,2	# L=(WA)(WB1)	J=(WA)(WB2)	J=(WA)(WB3)
		VAD	VSL1
			24D
		XCHX,1	UNIT
			30D
		STORE	XDC 	+18D,1	# XDC = L+J+K	YDC = L+J+K	ZDC = L+J+K

		TIX,1
			AXISGEN3

AXISGEN3	TIX,2
			AXISGEN2

; Step 3: Store final transformation matrix. The computed direction cosine
; vectors (XDC, YDC, ZDC) are copied to STARAD locations for subsequent use
; in alignment calculations. These three half-unit vectors completely define
; the orientation of coordinate system A relative to coordinate system B.

		VLOAD
			XDC
		STOVL	STARAD
			YDC
		STOVL	STARAD 	+6
			ZDC
		STORE	STARAD 	+12D

		RVQ

# Page 1363
; Mathematical constants used in in-flight alignment routines:
; QTSN45 = 0.1768 (approximately sin(45°)/sqrt(2), used in gimbal lock checks)
; .166... = 0.1666666667 (1/6, used in Taylor series expansions)

QTSN45		2DEC	.1768
.166...		2DEC	.1666666667

# Page 1364 (empty page)


