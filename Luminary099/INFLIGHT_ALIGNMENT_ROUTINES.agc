# Copyright:	Public domain.
# Filename:	INFLIGHT_ALIGNMENT_ROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1249-1258
# Mod history:	2009-05-26 RSB	Adapted from the corresponding
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
; FILE: INFLIGHT_ALIGNMENT_ROUTINES.agc
; MODULE: Navigation and Guidance
; MISSION PHASE: All phases requiring IMU realignment
;
; TL;DR: Provides mathematical subroutines for realigning the Inertial
;        Measurement Unit (IMU) during flight without powering down the
;        platform. Computes gyro torque angles and gimbal drive angles
;        needed to correct IMU drift using star sightings from the Alignment
;        Optical Telescope (AOT). Critical for maintaining navigation
;        accuracy throughout the mission.
;
; COMMENT-ONLY READERS: These routines keep the guidance system accurate by
;        correcting gradual IMU drift. Used whenever crew performs star
;        sightings to realign the platform during lunar orbit and descent.
; CODE-ALONG READERS: Study the coordinate transformation mathematics and
;        gimbal angle computations. Note the gimbal lock detection at 60
;        degrees and the vector cross-product operations for axis generation.
; ============================================================================

# Page 1249
		BANK	22
		SETLOC	INFLIGHT
		BANK

		EBANK=	XSM

; ============================================================================
; GYRO TORQUE ANGLE COMPUTATION
;
; The IMU (Inertial Measurement Unit) maintains the spacecraft's orientation
; reference through three gyroscopes. Over time, gyro drift causes the stable
; member to deviate from its intended orientation. When the crew performs
; star sightings through the AOT (Alignment Optical Telescope), these routines
; compute the exact angles needed to torque each gyro to correct the drift.
;
; The process: Crew identifies two known stars, computer compares actual 
; star directions to expected directions, then calculates correction angles
; for the three gyros (X, Y, and Z axes). These corrections are applied as
; electrical torque pulses to the gyroscopes, physically realigning the
; stable member without shutting down the IMU.
; ============================================================================

# CALCGTA COMPUTES THE GYRO TORQUE ANGLES REQUIRED TO BRING THE STABLE MEMBER INTO THE DESIRED ORIENTATION.
#
# THE INPUT IS THE DESIRED STABLE MEMBER COORDINATES REFERRED TO PRESENT STABLE MEMBER COORDINATES.  THE THREE
# HALF-UNIT VECTORS ARE STORED AT XDC, YDC, AND ZDC.
#
# THE OUTPUTS ARE THE THREE GYRO TORQUE ANGLES TO BE APPLIED TO THE Y, Z, AND X GYROS AND ARE STORED DP AT IGC,
# MGC, AND OGC RESPECTIVELY.

; CALCGTA (Calculate Gyro Torque Angles):
; Computes the three rotation angles needed to align the stable member
; from its current orientation to the desired orientation determined by
; star sightings. Uses coordinate transformation mathematics to derive
; the Y-axis, Z-axis, and X-axis gyro torque angles.
;
; Inputs: XDC, YDC, ZDC = desired coordinate system unit vectors (half-unit)
; Outputs: IGC = Y gyro angle, MGC = Z gyro angle, OGC = X gyro angle
;          All angles in fractions of revolution (±0.5 = ±180 degrees)

		COUNT*	$$/INFLT
CALCGTA		ITA	DLOAD		# PUSHDOWN 00-03, 16D-27D, 34D-37D
			S2		# XDC = (XD1 XD2 XD3)
			XDC		# YDC = (YD1 YD2 YD3)
; First gyro angle calculation: Y-axis (Inner Gimbal)
; Construct orthogonal vector ZPRIME perpendicular to XDC in the XZ plane.
; This intermediate vector enables computation of the first rotation angle.
		PDDL	PDDL		# ZDC = (ZD1 ZD2 ZD3)
			HI6ZEROS
			XDC 	+4
		DCOMP	VDEF
; Form vector (-XD3, 0, XD1) and normalize to unit length.
; This gives ZPRIME perpendicular to the Y-axis in current coordinates.
		UNIT
		STODL	ZPRIME		# ZP = UNIT(-XD3 0 XD1) = (ZP1 ZP2 ZP3)
			ZPRIME
; Compute sine and cosine of first gyro angle IGC from ZPRIME components.
; Using ZP1 and ZP3 components gives the rotation angle about Y-axis.
		SR1
		STODL	SINTH		# SIN(IGC) = ZP1
			ZPRIME 	+4
		SR1
		STCALL	COSTH		# COS(IGC) = ZP3
			ARCTRIG
; ARCTRIG converts sine/cosine to angle in revolutions.
		STODL	IGC		# Y GYRO TORQUING ANGLE   FRACTION OF REV.
			XDC 	+2
; Second gyro angle calculation: Z-axis (Middle Gimbal)
; Use XD2 component directly as sine, compute cosine from cross products.
		SR1
		STODL	SINTH		# SIN(MGC) = XD2
			ZPRIME
; Compute cosine using dot product relationships between coordinate axes.
		DMP	PDDL
			XDC 	+4	# PD00 = (ZP1)(XD3)
			ZPRIME 	+4

		DMP	DSU
			XDC		# MPAC = (ZP3)(XD1)
		STADR
		STCALL	COSTH		# COS(MGC) = MPAC - PD00
			ARCTRIG
# Page 1250
		STOVL	MGC		# Z GYRO TORQUING ANGLE   FRACTION OF REV.
			ZPRIME
; Third gyro angle calculation: X-axis (Outer Gimbal)
; Use dot products of ZPRIME with desired Z and Y axes to get sine/cosine.
		DOT
			ZDC
		STOVL	COSTH		# COS(OGC) = ZP . ZDC
			ZPRIME
		DOT
			YDC
		STCALL	SINTH		# SIN(OGC) = ZP . YDC
			ARCTRIG
; All three gyro torque angles now computed: IGC, MGC, OGC.
; These angles will be applied to Y, Z, and X gyros respectively to
; physically rotate the stable member into the desired orientation.
		STCALL	OGC		# X GYRO TORQUING ANGLE   FRACTION OF REV.
			S2

# Page 1251
; ============================================================================
; ARCTRIG - Arctangent Function Using Sine and Cosine
;
; Converts sine and cosine values into an angle. This is more robust than
; using arctangent alone because having both sine and cosine determines
; the angle uniquely in all four quadrants without ambiguity.
;
; Used throughout alignment calculations to convert coordinate transformations
; into physical gimbal rotation angles. Essential for the AGC since it lacks
; direct arctangent hardware - this routine implements the function using
; the interpreter's mathematical operations.
; ============================================================================

# ARCTRIG COMPUTES AN ANGLE GIVEN THE SINE AND COSINE OF THIS ANGLE.
#
# THE INPUTS ARE SIN/4 AND COS/4 STORED DP AT SINTH AND COSTH.
#
# THE OUTPUT IS THE CALCULATED ANGLE BETWEEN +.5 AND -.5 REVOLUTIONS AND STORED AT THETA.  THE OUTPUT IS ALSO
# AVAILABLE AT MPAC.

; ARCTRIG subroutine:
; Inputs: SINTH = sine/4 (scaled), COSTH = cosine/4 (scaled)
; Output: THETA = angle in revolutions (±0.5 rev = ±180 degrees)
; Method: Uses two-argument arctangent to determine angle in correct quadrant

; Quadrant determination logic: Check magnitude of sine to decide approach.
; If |sin| < sin(45°), use arcsin directly (more accurate near 0°).
; If |sin| > sin(45°), use arccos instead (more accurate near 90°).
ARCTRIG		DLOAD	ABS		# PUSHDOWN  16D-21D
			SINTH
		DSU	BMN
			QTSN45		# ABS(SIN/4) - SIN(45)/4
			TRIG1		# IF (-45,45) OR (135,-135)
; Large sine magnitude: angles near ±90° or ±270° (45° to 135° range).
; Use arccos for better numerical accuracy, then adjust sign.
		DLOAD	SL1		# (45,135) OR (-135,-45)
			COSTH
		ACOS	SIGN
			SINTH
		STORE	THETA		# X = ARCCOS(COS) WITH SIGN(SIN)
		RVQ
; Small sine magnitude: angles near 0° or ±180° (-45° to 45° range).
; Use arcsin for better numerical accuracy in this region.
TRIG1		DLOAD	SL1		# (-45,45) OR (135,-135)
			SINTH
		ASIN
		STODL	THETA		# X = ARCSIN(SIN) WITH SIGN(SIN)
			COSTH
		BMN
			TRIG2		# IF (135,-135)
; Angle in first or fourth quadrant (-45° to +45°): arcsin sufficient.
		DLOAD	RVQ
			THETA		# X = ARCSIN(SIN)   (-45,45)
; Angle in second or third quadrant (135° to -135° through ±180°).
; Compute as: ±180° - arcsin(sin) to get correct angle.
TRIG2		DLOAD	SIGN		# (135,-135)
			HIDPHALF
			SINTH
		DSU
			THETA
		STORE	THETA		# X = .5 WITH SIGN(SIN) - ARCSIN(SIN)
		RVQ			#	(+) - (+) OR (-) - (-)

# Page 1252
# SMNB, NBSM, AND AXISROT, WHICH USED TO APPEAR HERE, HAVE BEEN
# COMBINED IN A ROUTINE CALLED AX*SR*T, WHICH APPEARS AMONG THE POWERED
# FLIGHT SUBROUTINES.

# Page 1253
; ============================================================================
; CDU DRIVING ANGLE COMPUTATION
;
; The CDUs (Coupling Data Units) are the angle readout devices on each of
; the three IMU gimbals. They measure the actual physical gimbal positions.
; During IMU alignment, the gimbals must be rotated to specific angles to
; orient the stable member correctly.
;
; CALCGA computes the three gimbal angles needed to transform the navigation
; base (the spacecraft body frame) into the desired stable member orientation.
; These angles are sent to the gimbal drive motors which physically position
; the gimbals. During Apollo 11's mission, this routine was used whenever
; Aldrin performed star sightings to refine the IMU alignment.
; ============================================================================

# CALCGA COMPUTES THE CDU DRIVING ANGLES REQUIRED TO BRING THE STABLE MEMBER INTO THE DESIRED ORIENTATION.
#
# THE INPUTS ARE  1) THE NAVIGATION BASE COORDINATES REFERRED TO ANY COORDINATE SYSTEM.  THE THREE HALF-UNIT
# VECTORS ARE STORED AT XNB, YNB, AND ZNB.  2) THE DESIRED STABLE MEMBER COORDINATES REFERRED TO THE SAME
# COORDINATE SYSTEM ARE STORED AT XSM, YSM, AND ZSM.
#
# THE OUTPUTS ARE THE THREE CDU DRIVING ANGLES AND ARE STORED SP AT THETAD, THETAD +1, AND THETAD +2.

; CALCGA (Calculate Gimbal Angles):
; Computes the three physical gimbal rotation angles (outer, inner, middle)
; needed to orient the IMU stable member from navigation base coordinates.
;
; Inputs: XNB, YNB, ZNB = navigation base coordinate system (spacecraft body)
;         XSM, YSM, ZSM = desired stable member orientation
; Outputs: THETAD, THETAD+1, THETAD+2 = CDU angles in single precision
; Note: Includes gimbal lock detection if middle gimbal exceeds 60 degrees

; CALCGA computes CDU (gimbal) driving angles to align stable member.
; The IMU gimbals follow a sequence: Outer → Middle → Inner.
; Each gimbal angle must be computed from coordinate frame transformations.
CALCGA		SETPD			# PUSHDOWN 00-05, 16D-21D, 34D-37D
			0
; Step 1: Compute Middle Gimbal Axis (MGA) and Outer Gimbal (OG) angle.
; The outer gimbal axis is the spacecraft body X-axis (XNB).
; The inner gimbal axis is the desired Y stable member axis (YSM).
; Cross product gives the middle gimbal axis perpendicular to both.
		VLOAD	VXV
			XNB		# XNB = OGA (OUTER GIMBAL AXIS)
			YSM		# YSM = IGA (INNER GIMBAL AXIS)
		UNIT	PUSH		# PD0 = UNIT(OGA X IGA) = MGA
; Compute outer gimbal angle by projecting MGA onto body Y and Z axes.
; This determines the rotation of the outer gimbal from reference position.
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
; Step 2: Compute Middle Gimbal (MG) angle.
; This is the most critical angle for gimbal lock avoidance.
; If MG approaches 90 degrees, outer and inner gimbals align, losing a
; degree of freedom (gimbal lock). This routine checks for this condition.
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
; Gimbal lock check: Middle gimbal angle must not exceed 60 degrees.
; At 60° (= 0.1666... revolutions), the outer and inner gimbals approach
; alignment, which eliminates one rotational degree of freedom. If exceeded,
; alarm 00401 is triggered and GLOKFAIL flag set to warn crew.
		ABS	DSU
			.166...
		BPL
			GIMLOCK1	# IF ANGLE GREATER THAN 60 DEGREES
; Step 3: Compute Inner Gimbal (IG) angle.
; This is the final rotation axis that orients the stable member platform.
; The inner gimbal rotates the platform about the middle gimbal axis (MGA).
CALCGA1		VLOAD	DOT
			ZSM
			0
		STOVL	COSTH		# COS(IG) = ZSM . MGA
			XSM
# Page 1254
		DOT	STADR
		STCALL	SINTH		# SIN(IG) = XSM . MGA
			ARCTRIG
; Store all three gimbal angles as a vector THETAD for downstream use.
; These angles drive the CDU (Coupling Data Unit) to physically reposition
; the gimbals. The V1STO2S routine formats the data for gimbal servo control.
		STOVL	IGC
			OGC
		RTB
			V1STO2S
		STCALL	THETAD
			S2
; GIMLOCK1 handles gimbal lock detection during alignment.
; If middle gimbal angle exceeds 60 degrees, issue alarm 00401.
; During Apollo 11 mission, crew procedures required avoiding maneuvers
; that would bring IMU near gimbal lock. The GLOKFAIL flag indicates
; that the current alignment request cannot be completed safely.
GIMLOCK1	EXIT
		TC	ALARM
		OCT	00401
		TC	UPFLAG		# GIMBAL LOCK HAS OCCURRED
		ADRES	GLOKFAIL

		TC	INTPRET
		GOTO
			CALCGA1

# Page 1255
# AXISGEN COMPUTES THE COORDINATES OF ONE COORDINATE SYSTEM REFERRED TO ANOTHER COORDINATE SYSTEM.
#
# THE INPUTS ARE  1) THE STAR1 VECTOR REFERRED TO COORDINATE SYSTEM A STORED AT STARAD.  2) THE STAR2 VECTOR
# REFERRED TO COORDINATE SYSTEM A STORED AT STARAD +6.  3) THE STAR1 VECTOR REFERRED TO COORDINATE SYSTEM B STORED
# AT LOCATION 6 OF THE VAC AREA.  4) THE STAR2 VECTOR REFERRED TO COORDINATE SYSTEM B STORED AT LOCATION 12D OF
# THE VAC AREA.
#
# THE OUTPUT DEFINES COORDINATE SYSTEM A REFERRED TO COORDINATE SYSTEM B.  THE THREE HALF-UNIT VECTORS ARE STORED
# AT LOCATIONS XDC, XDC +6, XDC +12D, AND STARAD, STARAD +6, STARAD +12D.
; ============================================================================
; TRANSITION: From gimbal angle computation to coordinate transformation
;
; The AXISGEN routine transforms between two coordinate reference frames using
; star observations. During Apollo 11's inflight alignments in lunar orbit,
; crew sighted two known stars through the AOT (Alignment Optical Telescope).
; These star vectors, measured in both the desired reference frame and the
; current stable member frame, allow computation of the rotation matrix
; that defines how the IMU platform must be rotated. This transformation
; matrix becomes the input to CALCGTA for gyro torquing angle calculation.
; ============================================================================
;
; AXISGEN: Generate transformation matrix between coordinate systems.
; This routine constructs an orthonormal basis (three mutually perpendicular
; unit vectors) in each of two coordinate systems using two star sightings.
; The transformation matrix relating the two systems is then computed from
; the dot products of corresponding basis vectors.
;
; Inputs:
;   STARAD: Star 1 vector in coordinate system A (half-unit vector)
;   STARAD+6: Star 2 vector in coordinate system A (half-unit vector)
;   VAC area location 6: Star 1 vector in coordinate system B
;   VAC area location 12D: Star 2 vector in coordinate system B
;
; Outputs:
;   XDC, YDC, ZDC: Three half-unit vectors defining system A in system B
;   STARAD, STARAD+6, STARAD+12D: Same vectors (duplicate storage)
;
; Step 1: Construct orthonormal bases in both coordinate systems.
; For each system, we have two star vectors (S1, S2) that are not necessarily
; orthogonal. We compute three mutually perpendicular unit vectors:
;   U = S1 (first star direction)
;   V = UNIT(S1 × S2) (perpendicular to plane containing both stars)
;   W = U × V (completes right-handed orthonormal basis)
;
; This loop executes twice: once for coordinate system A, once for system B.
AXISGEN		AXT,1	SSP		# PUSHDOWN 00-30D, 34D-37D
			STARAD 	+6
			S1
			STARAD 	-6

		SETPD
			0
AXISGEN1	VLOAD*	VXV*		# 06D	UA = S1
			STARAD 	+12D,1	#	STARAD +00D	UB = S1
			STARAD 	+18D,1
		UNIT			# 12D	VA = UNIT(S1 X S2)
		STORE	STARAD 	+18D,1	#	STARAD +06D	VB = UNIT(S1 X S2)
		VLOAD*
			STARAD 	+12D,1
; Compute third basis vector as cross product of first two.
; This ensures right-handed orthogonal coordinate system.
		VXV*	VSL1
			STARAD 	+18D,1	# 18D	WA = UA X VA
		STORE	STARAD 	+24D,1	#	STARAD +12D	WB = UB X VB

		TIX,1
			AXISGEN1

; Step 2: Compute transformation matrix from system B to system A.
; The transformation matrix has three columns (XDC, YDC, ZDC), each computed
; by projecting the basis vectors of system A onto the basis vectors of
; system B. For each column i:
;   Column_i = (UA·UBi)·UBi + (VA·VBi)·VBi + (WA·WBi)·WBi
; This represents how each basis vector of system A is expressed in terms
; of system B basis vectors. The result is a rotation matrix.
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
; Double loop: outer loop (index 2) iterates over system B basis vectors,
; inner loop (index 1) iterates over system A basis vectors.
; For each of three columns of transformation matrix:
AXISGEN2	XCHX,1	VLOAD*
			30D		# X1=-6 X2=+6	X1=-6 X2=+4	X1=-6 X2=+2
			0,1

# Page 1256
		VXSC*	PDVL*		# J=(UA)(UB1)	J=(UA)(UB2)	J=(UA)(UB3)
			STARAD 	+6,2
			6,1
		VXSC*
			STARAD 	+12D,2
		STOVL*	24D		# K=(VA)(VB1)	J=(VA)(VB2)	J=(VA)(VB3)
			12D,1
; Sum the three weighted basis vector projections and normalize.
; Result is one column of the transformation matrix, stored at XDC+offset.
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

; Step 3: Store transformation matrix in duplicate locations.
; The three half-unit vectors (XDC, YDC, ZDC) define coordinate system A
; in terms of coordinate system B. These are also stored at STARAD for
; compatibility with downstream routines. After this transformation,
; CALCGTA can compute the gyro torque angles needed to rotate the IMU
; stable member from its current orientation to the desired orientation.
		VLOAD
			XDC
		STOVL	STARAD
			YDC
		STOVL	STARAD 	+6
			ZDC
		STORE	STARAD 	+12D

		RVQ

# Page 1257
; Mathematical constants used in inflight alignment routines:
;
; QTSN45: Sine of 45 degrees divided by 4 = sin(45°)/4 = 0.1768...
; Used in gimbal angle calculations where quarter-sine values are needed
; for coordinate transformations near gimbal gimbal boundaries.
QTSN45		2DEC	.1768

; .166...: One-sixth in decimal (0.1666...) = 60 degrees in revolutions.
; Used as threshold for gimbal lock detection. When middle gimbal angle
; exceeds 60 degrees (±0.1666 revolutions), outer and inner gimbals
; approach alignment, creating gimbal lock condition. Alarm 00401 triggered.
.166...		2DEC	.1666666667

# Page 1258 (empty page)

