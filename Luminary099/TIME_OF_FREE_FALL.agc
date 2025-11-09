# Copyright:	Public domain.
# Filename:	TIME_OF_FREE_FALL.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1268-1283
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
; FILE: TIME_OF_FREE_FALL.agc
; MODULE: Orbital Integration
; MISSION PHASE: trans-lunar/lunar-orbit/ascent/rendezvous/trans-earth
;
; TL;DR: Computes time-of-flight for ballistic (free-fall) trajectories
;        between two points along conic section orbits. Solves Lambert 
;        problem variations to determine transfer time from current position
;        to a specified terminal radius. Used for trajectory planning,
;        rendezvous targeting, and orbit transfer calculations.
;
; COMMENT-ONLY READERS: These mathematical routines calculate how long it
;        takes to coast along an orbit from one point to another without
;        engine thrust, essential for timing rendezvous maneuvers.
; CODE-ALONG READERS: Study the conic section mathematics, fixed-point
;        arithmetic scaling for Earth vs Moon centered coordinates, and
;        polynomial approximation techniques for efficient computation.
; ============================================================================

# Page 1268
;
; ============================================================================
; TIME OF FREE FALL (TFF) SUBROUTINES - USAGE AND SCALING
;
; These subroutines calculate the coasting time along orbital trajectories
; without engine thrust. The calculations work for both Earth-centered and
; Moon-centered orbits, supporting mission phases from translunar coast
; through lunar orbit operations and transearth return.
;
; The AGC's fixed-point arithmetic requires different scaling for Earth
; versus Moon centered coordinates due to the vastly different distances
; and gravitational parameters involved. The calling program must provide
; state vectors and constants scaled appropriately for the central body.
; ============================================================================
;
# THE TFF SUBROUTINES MAY BE USED IN EITHER EARTH OR MOON CENTERED COORDINATES.  THE TFF ROUTINES NEVER
# KNOW WHICH ORIGIN APPLIES.  IT IS THE USER WHO KNOWS, AND WHO SUPPLIES RONE, VONE, AND 1/SQRT(MU) AT THE
# APPROPRIATE SCALE LEVEL FOR THE PROPER PRIMARY BODY.
;
; SCALING FACTORS FOR FIXED-POINT ARITHMETIC:
;
; EARTH-CENTERED COORDINATES (translunar/transearth trajectory phases):
;   Position vectors scaled by 2^-29 meters (unit = 1.862 nanometers)
;   Velocity vectors scaled by 2^-7 meters/centisecond  
;   Gravitational parameter inverse: 1/sqrt(μ) scaled by 2^+17
;
; MOON-CENTERED COORDINATES (lunar orbit and landing phases):
;   Position vectors scaled by 2^-27 meters (unit = 7.451 nanometers)
;   Velocity vectors scaled by 2^-5 meters/centisecond
;   Gravitational parameter inverse: 1/sqrt(μ) scaled by 2^+14
;
; These scaling choices maximize precision within the AGC's 15-bit signed
; word constraints while representing the range of distances from Earth orbit
; (hundreds of km) through cislunar space (hundreds of thousands of km).
;
#
#	EARTH ORIGIN	POSITION	-29	METERS
#			VELOCITY	-7	METERS/CENTISECOND
#			1/SQRT(MU)	+17	SQRT(CS SQ/METERS CUBED)
#
#	MOON ORIGIN	POSITION	-27	METERS
#			VELOCITY	-5	METERS/CENTISECONDS
#			1/SQRT(MU)	+14	SQRT(CS SQ/METERS CUBED)
#
# ALL DATA PROVIDED TO AND RECEIVED FROM ANY TFF SUBROUTINE WILL BE AT ONE OF THE LEVELS ABOVE.  IN ALL CASES,
# THE FREE FALL TIME IS RETURNED IN CENTISECONDS AT (-28).  PROGRAM TFF/CONIC WILL GENERATE VONE/RTMU AND
# LEAVE IT IN VONE' AT (+10) IF EARTH ORIGIN AND (+9) IF MOON ORIGIN.
#
# THE USER MUST STORE THE STATE VECTOR IN RONE, VONE, AND MU IN THE FORM 1/SQRT(MU) IN TFF/RTMU
# AT THE PROPER SCALE BEFORE CALLING TFF/CONIC.  SINCE RONE, VONE ARE IN THE EXTENDED VERB STORAGE AREA,
# THE USER MUST ALSO LOCK OUT THE EXTENDED VERBS, AND RELEASE THEM WHEN FINISHED.
#
# PROGRAMS CALC/TFF AND CALC/TPER ASSUME THAT THE TERMINAL RADIUS IS LESS THAN THE PRESENT
# RADIUS.  THIS RESTRICTION CAN BE REMOVED BY A 15 W CODING CHANGE, BUT AT PRESENT IT IS NOT DEEMED NECESSARY.
#
# THE FOLLOWING ERASABLE QUANTITIES ARE USED BY THE TFF ROUTINES, AND ARE LOCATED IN THE PUSH LIST.
#
#					BELOW	E:  IS USED FOR EARTH ORIGIN SCALE
#						M:  IS USED FOR MOON  ORIGIN SCALE
#
#TFFSW		=	119D	# BIT1	0 = CALCTFF		1 = CALCTPER
TFFDELQ		=	10D	#	Q2-Q1			E: (-16)  M: (-15)
RMAG1		=	12D	#	ABVAL(RN)  M		E: (-29)  M: (-27)
#RPER		=	14D	#	PERIGEE RADIUS  M	E: (-29)  M: (-27)
TFFQ1		=	14D	#	R.V / SQRT(MUE)		E: (-16)  M: (-15)
#SDELF/2			#	SIN(THETA) /2
CDELF/2		=	14D	#	COS(THETA) /2
#RAPO		=	16D	#	APOGEE RADIUS  M	E: (-29)  M: (-27)
NRTERM		=	16D	#	TERMINAL RADIUS  M	E: (-29+NR)
				#					  M: (-27+NR)
RTERM		=	18D	#	TERMINAL RADIUS  M	E: (-29)  M: (-27)
TFFVSQ		=	20D	#	-(V SQUARED/MU)  1/M	E: (20)   M: (18)
TFF1/ALF	=	22D	#	SEMI MAJ AXIS  M	E: (-22-2 NA)
				#					  M: (-20-2 NA)
TFFRTALF	=	24D	#	SQRT(ALFA)		E:(10+NA) M: (9+NA)
TFFALFA		=	26D	#	ALFA  1/M		E:(26-NR) M: (24-NR)
TFFNP		=	28D	#	SEMI LATUS RECTUM  M	E: (-38+2 NR)
				#					  M: (-36+2 NR)
TFF/RTMU	=	30D	#	1/SQRT(MU)		E: (17)   M: (14)
NRMAG		=	32D	#	PRESENT RADIUS  M	E: (-29+NR)
				#					  M: (-27+NR)
TFFX		=	34D     #
TFFTEM		=	36D	#	TEMPORARY
# Page 1269
#		REGISTERS S1, S2 ARE UNTOUCED BY ANY TFF SUBROUTINE
#		INDEX REGISTERS X1, X2 ARE USED BY ALL TFF SUBROUTINES.  THEY ARE ESTAB-
#		LISHED IN TFF/CONIC AND MUST BE PRESERVED BETWEEN CALLS TO SUBSEQUENT
#		SUBROUTINES.
#		-NR				C(X1) = NORM COUNT OF RMAG
#		-NA				C(X2) = NORM COUNT OF SQRT(ABS(ALFA))

# Page 1270
;
; ============================================================================
; TRANSITION: From scaling definitions to conic parameter computation
;
; Before calculating time of flight, the AGC must first determine the
; orbital geometry from the current state vector. The following subroutine
; computes fundamental conic section parameters: angular momentum, semi-latus
; rectum, and semi-major axis (or its reciprocal ALFA). These parameters
; define whether the spacecraft is in an elliptical, parabolic, or hyperbolic
; trajectory - critical for accurate rendezvous timing and mission planning.
; ============================================================================
;
# SUBROUTINE NAME:  TFFCONIC						DATE:  01.29.67
# MOD NO:  0								LOG SECTION:  TIME OF FREE FALL
# MOD BY:  RR BAIRNSFATHER
# MOD NO:  1	MOD BY:  RR BAIRNSFATHER	DATE: 11 APR 67
# MOD NO:  2	MOD BY:  RR BAIRNSFATHER	DATE: 21 NOV 67		ADD MOON MU.
# MOD NO:  3	MOD BY:  RR BAIRNSFATHER	DATE: 21 MAR 68		ACCEPT DIFFERENT EARTH/MOON SCALES
;
; SUBROUTINE: TFFCONIC / TFFCONMU
;
; Computes conic section orbital parameters from current state vector.
; Determines orbit shape (ellipse, parabola, or hyperbola) and calculates
; geometric parameters required by subsequent time-of-flight calculations.
;
; CONIC ORBIT FUNDAMENTALS:
; - Angular momentum H = R × V determines orbital plane
; - Semi-latus rectum p = H²/μ determines orbit size for given eccentricity
; - ALFA = 2/R - V²/μ is reciprocal of semi-major axis (signed)
;   * ALFA > 0: Elliptical orbit (bound, will return to starting point)
;   * ALFA = 0: Parabolic orbit (escape trajectory, minimum energy)
;   * ALFA < 0: Hyperbolic orbit (excess energy, interplanetary trajectory)
;
; During Apollo 11's translunar coast, ALFA was negative (hyperbolic relative
; to Earth). In lunar orbit, ALFA was positive (elliptical). At rendezvous,
; these calculations ensured Eagle's orbit matched Columbia's precisely.
;
#
# FUNCTIONAL DESCRIPTION:  THIS SUBROUTINE IS CALLED TO COMPUTE THOSE CONIC PARAMETERS REQUIRED BY THE TFF
#	SUBROUTINES AND TO ESTABLISH THEM IN THE PUSH LIST AREA.  THE PARAMETERS ARE LISTED UNDER OUTPUT.
#	THE EQUATIONS ARE:
#		_   __ __
#		H = RN*VN			ANGULAR MOMENTUM
#		      _ _
#		LCP = H.H / MU			SEMI LATUS RECTUM
#		              __ __
#		ALFA = 2/RN - VN.VN / MU	RECIPROCAL SEMI-MAJOR AXIS, SIGNED
#
# 	AND ALFA IS POS FOR ELLIPTIC ORBITS
#	              0 FOR PARABOLIC ORBITS
#	            NEG FOR HYPERBOLIC ORBITS
#	SUBROUTINE ALSO COMPUTES AND SAVES RMAG.
#
# CALLING SEQUENCE:
#	TFFCONIC EXPECTS CALLER TO ENTER WITH CORRECT GRAVITATIONAL CONSTANT IN MPAC, IN THE FORM
#	1/SQRT(MU).  THE PROGRAM WILL SAVE IN TFF/RTMU.  THE SCALE IS DETERMINED BY WHETHER EARTH OR MOON
#	ORIGIN IS USED.  THE CALLER MUST LOCK OUT THE EXTENDED VERBS BEFORE PROVIDING STATE VECTOR IN RONE,
#	VONE AT PROPER SCALE.  THE EXTENDED VERBS MUST BE RESTORED WHEN THE CALLER IS FINISHED USING THE
#	TFF ROUTINES.
#
#	ENTRY POINT TFFCONMU EXPECTS THAT TFF/RTMU IS ALREADY LOADED.
#
#	TO SPECIFY MU:	DLOAD	CALL			 	IF MU ALREADY STORED:	CALL
#				YOURMU	1/RTMU E:(17) M:(14)					TFFCONMU
#				TFFCONIC
#	PUSHLOC = PDL+0, ARBITRARY IF LEQ 18D
#
# SUBROUTINES CALLED:  NONE
#
# NORMAL EXIT MODES:  RVQ
#
# ALARMS:  NONE
#
# OUTPUT:	THE FOLLOWING ARE STORED IN THE PUSH LIST AREA.
#		RMAG1		E:(-29) M:(-27)	M  RN, PRESENT RADIUS LENGTH.
#		NRMAG		E:(-29+NR)	M  RMAG, NORMALIZED
#				M:(-27+NR)
#		X1				-NR, NORM COUNT
#		TFFNP		E:(-38+2NR)	M  LCP, SEMI LATUS RECTUM, WEIGHTED BY NR.  	FOR VGAMCALC.
#				M:(-36+2NR)
#		TFF/RTMU	E:(17) M:(14)	1/SQRT(MU)
#		TFFVSQ		E:(20) M:(18)	1/M  -(V SQ/MU):  PRESENT VELOCITY, NORMALIZED. FOR VGAMCALC
#		TFFALFA		E:(26-NR)	1/M  ALFA, WEIGHTED BY NR
#				M:(24-NR)
#		TFFRTALF	E:(10+NA)	SQRT(ALFA), NORMALIZED
#				M:(9+NA)
# Page 1271
#		X2				-NA, NORMCOUNT
#		TFF1/ALF	E:(-22-2NA)	SIGNED SEMI MAJ AXIS, WEIGHTED BY NA
#				M:(-20-2NA)
#		PUSHLOC AT PDL+0
#
#	THE FOLLOWING IS STORED IN GENERAL ERASABLE
#		VONE'		E:(10) M:(9)	V/RT(MU), NORMALIZED VELOCITY
#
# ERASABLE INITIALIZATION REQUIRED:
#		RONE		E:(-29) M:(-27)	M  STATE VECTOR		LEFT BY CALLER
#		VONE		E:(-7) M:(-5)	M/CS  STATE VECTOR	LEFT BY CALLER
#		TFF/RTMU	E:(17) M:(14)	1/RT(CS SQ/M CUBE)	IF ENTER VIA TFFCONMU.
#
# DEBRIS:	QPRET		PDL+0 ... PDL+3

		BANK	33
		SETLOC	TOF-FF
		BANK

		COUNT*	$$/TFF
;
; Entry point TFFCONIC: Caller provides gravitational constant 1/sqrt(μ).
; Entry point TFFCONMU: Gravitational constant already stored in TFF/RTMU.
;
; The distinction allows flexibility for repeated calculations with the
; same central body (avoiding redundant μ storage) versus switching between
; Earth and Moon centered frames during translunar/transearth phases.
;
TFFCONIC	STORE	TFF/RTMU	# 1/SQRT(MU)	E:(17) M:(14)
;
; Load position vector RONE and normalize to unit vector, simultaneously
; computing magnitude (current orbital radius). The unit vector defines
; radial direction; magnitude becomes RMAG1 for subsequent calculations.
;
TFFCONMU	VLOAD	UNIT		# COME HERE WITH TFFRTMU LOADED.
			RONE		# SAVED RN.  M  E:(-29) M:(-27)
		PDDL			# UR/2 TO PDL+0, +5
			36D		# MAGNITUDE
		STORE	RMAG1		# M  E:(-29) M:(-27)

;
; Normalize RMAG to maximize fixed-point precision. Store norm count in X1
; for later scaling adjustments. Normalized radius (NRMAG) shifts magnitude
; into optimal range for AGC's 15-bit arithmetic operations.
;
		NORM
			X1		# -NR
		STOVL	NRMAG		# RMAG  M  E:(-29+NR) M:(-27+NR)
			VONE		# SAVED VN.  M/CS  E:(-7) M:(-5)
;
; Normalize velocity by dividing by sqrt(μ), producing dimensionless velocity
; scaled appropriately for conic equations. This transformation simplifies
; subsequent orbital energy and angular momentum calculations.
;
		VXSC
			TFF/RTMU	# E:(17) M:(14)
		STORE	VONE'		# VN/SQRT(MU)  E:(10) M:(9)
;
; Compute angular momentum H = R × V (cross product of position and velocity).
; Angular momentum vector is perpendicular to orbital plane and conserved
; throughout free-fall motion. Its magnitude determines orbit size/shape.
;
		VXSC	VXV
			NRMAG		# E:(-29+NR) M:(-27+NR)
					# UR/2 FROM PDL
		VSL1	VSQ		# BEFORE:  E:(-19+NR) M:(-18+NR)
;
; Semi-latus rectum p = H²/μ characterizes conic section perpendicular to
; major axis. For ellipse, p = a(1-e²). Used in VGAMCALC for flight path
; angle calculations and trajectory geometry.
;
		STODL	TFFNP		# LC P  M  E:(-38+2NR) M:(-36+2NR)
					# SAVE ALSO FOR VGAMCALC
			TFF1/4
		DDV	PDVL		# (2/RMAG)  1/M  E:(26-NR) M:(24-NR)
			NRMAG		# RMAG  M  E:(-29+NR) M:(-27+NR)
			VONE'		# SAVED VN.  E:(10) M:(9)
;
; Calculate ALFA = 2/R - V²/μ (reciprocal of semi-major axis, signed).
; This is the fundamental orbital energy parameter:
; - Negative ALFA: Hyperbolic (excess velocity, open trajectory)
; - Zero ALFA: Parabolic (exactly escape velocity)  
; - Positive ALFA: Elliptical (closed orbit, periodic motion)
;
		VSQ	DCOMP		# KEEP MPAC+2 HONEST FOR SQRT.
		STORE	TFFVSQ		# -(V SQ/MU)  E:(20) M:(18)
					# SAVE FOR VGAMCALC
		SR*	DAD
# Page 1272
;
; Complete ALFA calculation by adding the two components:
; ALFA = 2/R - V²/μ (reciprocal of semi-major axis)
;
			0 	-6,1	# GET -VSQ/MU  E:(26-NR) M:(24-NR)
		STADR
					# 2/RMAG  FROM PDL+2
		STORE	TFFALFA		# ALFA  1/M  E:(26-NR) M:(24-NR)
;
; Compute sqrt(|ALFA|) for subsequent time-of-flight calculations. The square
; root of ALFA appears in Kepler's equation for elliptical orbits and in
; hyperbolic anomaly calculations. Normalize to maximize precision.
;
		SL*	PUSH		# TEMP SAVE ALFA  E:(20) M:(18)
			0 	-6,1
		ABS	SQRT		# E:(10) M:(9)
		NORM
			X2		# X2 = -NA
		STORE	TFFRTALF	# SQRT( ABS(ALFA) )  E:(10+NA) M:(9+NA)
;
; Compute reciprocal 1/ALFA (semi-major axis) for apogee/perigee calculations.
; Special handling: If ALFA ≈ 0 (parabolic trajectory), set 1/ALFA = 0 as flag
; indicating infinite semi-major axis. Sign preserved through DSQ+SIGN sequence.
;
		DSQ	SIGN		# NOT SO ACCURATE, BUT OK
					# ALFA FROM PDL+2  E:(20) M:(18)
		BZE	BDDV		# SET 1/ALFA =0, TO SHOW SMALL ALFA
			+2
			TFF1/4
 +2		STORE	TFF1/ALF	# 1/ALFA  E:(-22-2NA) M:(-20-2NA)
;
; TFFCONIC computation complete. All conic parameters established in push list.
; Index registers X1 (norm count of RMAG) and X2 (norm count of ALFA) preserved
; for subsequent TFF subroutine calls requiring consistent scaling.
;
DUMPCNIC	RVQ

#							      39 W
# Page 1273
# SUBROUTINE NAME:  TFFRP/RA						DATE: 01.17.67
# MOD NO:  0								LOG SECTION:  TIME OF FREE FALL
# MOD NO:  1	MOD BY:  RR BAIRNSFATHER	DATE: 11 APR 67
# MOD NO:  2	MOD BY:  RR BAIRNSFATHER	DATE: 21 MAR 68		ACCEPT DIFFERENT EARTH/MOON SCALES
#									ALSO IMPROVE ACCURACY OF RAPO.
#
# FUNCTIONAL DESCRIPTION:  USED BY CALCTPER AND TFF DISPLAYS TO CALCULATE PERIGEE RADIUS AND ALSO
#	APOGEE RADIUS FOR A GENERAL CONIC.
#	PROGRAM GIVES PERIGEE RADIUS AS		APOGEE RADIUS IS GIVEN BY
#		RP = P/(1+E)				RA = (1+E) / ALFA
#	WHERE    2
#		E  = 1 - P ALFA
#	IF RA IS NEGATIVE OR SHOWS DIVIDE OVERFLOW, THEN RA = POSMAX BECAUSE
#		1. APOGEE RADIUS IS NOT MEANINGFUL FOR HYPERBOLA
#		2. APOGEE RADIUS IS NOT DEFINED FOR PARABOLA
#		3. APOGEE RADIUS EXCEEDS THE SCALING FOR ELLIPSE.
#
#	THIS SUBROUTINE REQUIRED THE SIGNED RECIPROCAL SEMI MAJ AXIS, ALFA, AND SEMI-LATUS RECTUM AS DATA.
#
# CALLING SEQUENCE:	CALL
#				TFFRP/RA
#	PUSHLOC = PDL+0, ARBITRARY IF LEQ 10D
#	C(MPAC) UNSPECIFIED
#
# SUBROUTINES CALLED:	NONE
#
# NORMAL EXIT MODE:	RVQ
#	IF ELLIPSE, WITHIN NORMAL SCALING, RAPO IS CORRECT.
#	OTHERWISE, RAPO = POSMAX.
#
# ALARMS:	NONE
#
# OUTPUT:	STORED IN PUSH LIST AREA.  SCALE OF OUTPUT AGREES WITH DATA SUPPLIED TO TFF/CONIC.
#	RPER	E:(-29) M:(-27)		M	PERIGEE RADIUS		DESTROYED BY CALCTFF/CALCTPER, TFFTRIG.
#	RAPO	E:(-29) M:(-27)		M	APOGEE RADIUS		WILL BE DESTROYED BY CALCTFF/CALCTPER
#	PUSHLOC AT PDL+0
#
# ERASABLE INITIALIZATION REQUIRED:
#	TFFALFA	E:(26-NR)		M	1/SEMI MAJ AXIS		LEFT BY TFFCONIC
#		M:(24-NR)
#	TFFNP	E:(-38+2NR)		M	LC P, SEMI LATUS RECTUM	LEFT BY TFFCONIC
#		M:(-36+2NR)
#	X1				-NR, NORM COUNT OF RMAG		LEFT BY TFFCONIC
#	X2				-NA, NORM COUNT OF ALFA		LEFT BY TFFCONIC
#
# DEBRIS:	QPRET, PDL+0 ... PDL+1

# Page 1274
RAPO		=	16D		# APOGEE RADIUS  M  E:(-29) M:(-27)
RPER		=	14D		# PERIGEE RADIUS  M  E:(-29) M:(-27)

TFFRP/RA	DLOAD	DMP
			TFFALFA		# ALFA  1/M  E:(26-NR) M:(24-NR)
			TFFNP		# LC P  M E:(-38+2NR) M:(-36+2NR)
		SR*	DCOMP		# ALFA P (-12+NR)
			0	 -8D,1	# ALFA P (-4)
		DAD	ABS		# (DCOMP GIVES VALID TP RESULT FOR SQRT)
					# (ABS PROTECTS SQRT IF E IS VERY NEAR 0)
			DP2(-4)
		SQRT	DAD		# E SQ = (1- P ALFA) (-4)
			TFF1/4
		PUSH	BDDV		# (1+E)  (-2)  TO PDL+0
			TFFNP		# LCP  M  E:(-38+2NR) M:(-36+2NR)
		SR*	SR*		# (DOES SR THEN SL TO AVOID OVFL)
			0,1		# X1=-NR
			0 	-7,1	# (EFFECTIVE SL)
		STODL	RPER		# PERIGEE RADIUS  M  E:(-29) M:(-27)
					# (1+E)  (-2)  FROM PDL+0
		DMP	BOVB
			TFF1/ALF	# E:(-22-2NA) M:(-20-2NA)
			TCDANZIG	# CLEAR OVFIND, IF ON.
		BZE	SL*
			MAXRA		# SET POSMAX IF ALFA=0
			0 	-5,2	# -5+NA
		SL*	BOV
			0,2
			MAXRA		# SET POSMAX IF OVFL.
		BPL			# CONTINUE WITH VALID RAPO.
			+3
MAXRA		DLOAD			# RAPO CALC IS NOT VALID.  SET RAPO =
			NEARONE		# POSMAX AS A TAG.
 +3		STORE	RAPO		# APOGEE RADIUS  M  E:(-29) M:(-27)
DUMPRPRA	RVQ

#								30 W
;
; ============================================================================
; TRANSITION: From conic parameter calculation to time-of-flight computation
;
; With the orbital parameters established by TFFCONIC (semi-major axis,
; eccentricity, angular momentum), the following routines compute the actual
; transfer time along the trajectory. CALCTFF calculates time to reach a 
; specified terminal radius, while CALCTPER calculates time to periapsis.
; These calculations support trajectory planning for orbit transfers and
; rendezvous maneuvers where precise timing is essential.
; ============================================================================
;
# Page 1275
# SUBROUTINE NAME:  CALCTPER / CALCTFF					DATE:  01.29.67
# MOD NO:  0								LOG SECTION:  TIME OF FREE FALL
# MOD BY:  RR BAIRNSFATHER
# MOD NO:  1	MOD BY:  RR BAIRNSFATHER	DATE: 21 MAR 67
# MOD NO:  2	MOD BY:  RR BAIRNSFATHER	DATE: 14 APR 67
# MOD BY:  3	MOD BY:  RR BAIRNSFATHER	DATE: 8 JUL 67		NEAR EARTH MUE AND NEG TFF (GONEPAST)
# MOD BY:  4	MOD BY:  RR BAIRNSFATHER	DATE: 21 NOV 67		ADD VARIABLE MU.
# MOD BY:  5	MOD BY:  RR BAIRNSFATHER	DATE: 21 MAR 68		ACCEPT DIFFERENT EARTH/MOON SCALES
#
# FUNCTIONAL DESCRIPTION:  PROGRAM CALCULATES THE FREE-FALL TIME OF FLIGHT FROM PRESENT POSITION RN AND
#	VELOCITY VN TO A RADIUS LENGTH SPECIFIED BY RTERM, SUPPLIED BY THE USER.  THE POSITION VECTOR
#	RN MAY BE ON EITHER SIDE OF THE CONIC, BUT RTERM IS CONSIDERED ON THE INBOUND SIDE.
#	THE EQUATIONS ARE:
#
#		Q2 = -SQRT(RTERM (2-RTERM ALFA) - LCP)	(INBOUND SIDE)	LEQ +- LCE/SQRT(ALFA)
#		     __ __
#		Q1 = RN.VN / SQRT(MU)					LEQ +- LCE/SQRT(ALFA)
#
# 		Z = NUM / DEN						LEQ +- 1/SQRT(ALFA)
#
#	WHERE, IF INBOUND
#		NUM = RTERM -RN						LEQ +- 2 LCE/ALFA
#		DEN = Q2+Q1						LEQ +- 2 LCE/SQRT(ALFA)
#
# 	AND, IF OUTBOUND
#		NUM = Q2-Q1						LEQ +- 2 LCE/SQRT(ALFA)
#		DEN = 2 - ALFA (RTERM + RN).				LEQ +- 2 LCE
#
#	IF 	ALFA ZZ < 1.0		(FOR ALL CONICS EXCEPT ELLIPSES HAVING ABS(DEL ECC ANOM) G 90 DEG)
#	THEN	X = ALFA Z Z
#	AND	TFF = (RTERM +RN -2 ZZ T(X) ) Z/SQRT(MU)
#		EXCEPT 	IF ALFA PNZ, AND IF TFF NEG,
#		THEN	TFF = 2 PI /(ALFA SQRT(ALFA)) + TFF
#	OR	IF ALFA ZZ GEQ 1.0	(FOR ELLIPSES HAVING ABS(DEL ECC ANOM) GEQ 90 DEG)
#	THEN	X = 1/ALFA Z Z
#	AND	TFF = (PI/SQRT(ALFA) -Q2 +Q1 +2(X T(X) -1) /ALFA Z) /ALFA SQRT(MU)
#	WHERE	T(X) IS A POLYNOMIAL APPROXIMATION TO THE SERIES
#		             2      3             2
#		1/3 - X/5 + X /7 - X /8 ...	(X  < 1.0)
#
# CALLING SEQUENC:	TIME TO RTERM			TIME TO PERIGEE
#			CALL				CALL
#				CALCTFF				CALCTPER
#			C(MPAC) = TERMNL RAD M		C(MPAC) = PERIGEE RAD M
#	FOR EITHER, E:(-29) M:(-27)
#	FOR EITHER, PUSHLOC = PDL+0, ARBITRARY IF LEQ 8D.
# Page 1276
#
# SUBROUTINES CALLED:	T(X), VIA RTB
#
# NORMAL EXIT MODE:	RVQ
#	HOWEVER, PROGRAM EXITS WITH ONE OF THE FOLLOWING VALUES FOR TFF (-28) CS IN MPAC.  USER MUST STORE.
#		A. TFF = FLIGHT TIME.  NORMAL CASE FOR POSITIVE FLIGHT TIME LESS THAN ONE ORBITAL PERIOD.
#		B. (THIS OPTION IS NO LONGER USED.)
#		C. TFF = POSMAX.  THIS INDICATES THAT THE CONIC FROM THE PRESENT POSITION WILL NOT RETURN TO
#		   THE SPECIFIED ALTITUDE.  ALSO INDICATES OUTBOUND PARABOLA OR HYPERBOLA.
#
# OUTPUT:	C(MPAC)		(-28) CS	TIME OF FLIGHT, OR TIME TO PERIGEE
#		TFFX		(0)		X					LEFT FOR ENTRY DISPLAY TFF ROUTINES
#		NRTERM		E:(-29+NR) M	RTERM, WEIGHTED BY NR			LEFT FOR ENTRY DISPLAY TFF ROUTINES
#				M:(-27+NR)
#		TFFTEM		E:(-59+2NR)	LCP Z Z SGN(SDELF)			LEFT FOR ENTRY DISPLAY TFF ROUTINES
#				M:(-55+2NR)	LCP /ALFA SGN(SDELF)			LEFT FOR ENTRY DISPLAY TFF ROUTINES
#		NOTE:	TFFTEM = PDL 36D AND WILL BE DESTROYED BY .:UNIT:.
#		RMAG1		E:(-29) M:(-27)	PDL 12 NOT TOUCHED.
#		TFFQ1		E:(-16) M:(-15)	PDL 14D
#		TFFDELQ		E:(-16) M:(-15)	PDL 10D
#		PUSHLOC AT PDL+0
#
# ERASABLE INITIALIZATION REQUIRED:
#		RONE		E:(-29) M:(-27)	M  STATE VECTOR				LEFT BY USER
#		VONE'		E:(+10) M:(+9)	VN/SQRT(NU)				LEFT BY TFF/CONIC
#		RMAG1		E:(-29) M:(-27)	PRESENT RADIUS, M			LEFT BY TFFCONIC
#		C(MPAC)		E:(-29) M:(-27)	RTERM, TERMINAL RADIUS LENGTH, M	LEFT BY USER
#
#		THE FOLLOWING ARE STORED IN THE PUSH LIST AREA.
#		TFF/RTMU	E:(17) M:(14)	1/SQRT(MU)				LEFT BY TFFCONIC.
#		NRMAG		E:(-29+NR)	M  RMAG, NORMALIZED			LEFT BY TFFCONIC
#				M:(-27+NR)
#		X1				-NR, NORM COUNT				LEFT BY TFFCONIC
#		TFFNP		E:(-38+2NR)	M  LCP, SEMI LATUS RECTUM, WEIGHT NR	LEFT BY TFFCONIC
#				M:(-36+2N4)
#		TFFALFA		E:(26-NR)	1/M  ALFA, WEIGHT NR			LEFT BY TFFCONIC
#				M:(24-NR)
#		TFFRTALF	E:(10+NA)	SQRT(ALFA), NORMALIZED			LEFT BY TFFCONIC
#				M:(9+NA)
#		X2				-NA, NORMCOUNT				LEFT BY TFFCONIC
#		TFF1/ALF	E:(-22-2NA)	SIGNED SEMI-MAJOR AXIS, WEIGHTED BY NA	LEFT BY TFFCONIC
#				M:(-20-2NA)
#
# DEBRIS:	QPRET, PDL+0 ... PDL+3
#		RTERM		E:(-29) M(-27)	RTERM, TERMINAL RADIUS LENGTH
#		RAPO		E:(-29) M(-27)	PDL 16D (=NRTERM)
#		RPER		E:(-29) M(-27)	PDL 14D (=TFFQ1)

# Page 1277
;
; CALCTFF/CALCTPER Entry Points
; These routines solve Kepler's equation to find transfer time along a conic
; trajectory. CALCTPER computes time to periapsis (closest approach point).
; CALCTFF computes time to reach an arbitrary terminal radius.
;
; The calculation uses a universal variable formulation that works for all
; conic sections (ellipse, parabola, hyperbola). The algorithm normalizes
; the eccentric anomaly change and uses a polynomial approximation for the
; time-of-flight function T(X).
;
CALCTPER	SETGO			# ENTER WITH RPER IN MPAC
			TFFSW		# Set flag: calculating time to periapsis
			+3
CALCTFF		CLEAR			# ENTER WITH RTERM IN MPAC
			TFFSW		# Clear flag: calculating time to terminal radius
 +3		STORE	RTERM		# E:(-29) M:(-27)
		SL*			; Normalize terminal radius by current radius
			0,1		# X1=-NR (normalization count from TFFCONIC)
		STORE	NRTERM		# RTERM  E:(-29+NR) M:(-27+NR)
		DMP	BDSU		; Compute (2 - ALFA*RTERM) for Q2 calculation
			TFFALFA		# ALFA  E:(26-NR) M:(24-NR)
			TFF1/4		; Subtract from 2.0 (stored as 1/4 at B-3)
		PUSH	DMP		# (2-ALFA RTERM)  (-3)  TO PDL+0
			NRTERM		# E:(-29+NR) M:(-27+NR)
					; Now computing RTERM*(2-ALFA*RTERM) for Q2
		PDDL	SR*		# RTERM(2-ALFA RTERM) TO PDL+2
					# E:(-32+NR) M:(-30+NR)
			TFFNP		# Load semi-latus rectum P  E:(-38+2NR) M:(-36+2NR)
			0 	-6,1	# X1 = -NR (shift right 6 positions, scaled by NR)
					; Semi-latus rectum P = h²/μ where h is angular momentum
		DCOMP	DAD		# DUE TO SHIFTS, KEEP PRECISION FOR SQRT
					; Computing P + RTERM(2-ALFA*RTERM) for Q2 formula
					# RTERM(2-ALFA RTERM) FROM PDL +2
					# E:(-32+NR) M:(-30+NR)
		SR*			# LEAVE  E:(-32) M:(-30)
			0,1		# X1 = -NR
		BOFF	DLOAD		# CHECK TFF /TPER SWITCH
			TFFSW		; Check if computing time-to-periapsis
			+2		# IF TFF (to terminal radius), CONTINUE
			TFFZEROS	# IF TPER (to periapsis), set Q2 = 0
					; For periapsis, Q2=0 because R·V=0 at closest approach
 +2		BMN	SQRT		# E:(-16) M:(-15)
					; Branch if negative (terminal radius unreachable)
			MAXTFF1		# NO FREE FALL CONIC TO RTERM FROM HERE
					# RESET PDL, SET TFF=POSMAX, AND EXIT.
					; This handles case where target radius cannot be
					; reached on current ballistic trajectory

		DCOMP	BOVB		# RT IS ON INBOUND SIDE.  ASSURE OVFIND=0
			TCDANZIG	# ANY PORT IN A STORM.
		STOVL	TFFTEM		# Store Q2  E:(-16) M:(-15)
					; Q2 = √[P + RTERM(2-ALFA*RTERM)] is generalized
					; momentum at terminal radius
			VONE'		# Load VN/√μ (normalized velocity)  E:(10) M:(9)
		DOT	SL3		; Compute dot product R·V
			RONE		# Current position RN (saved earlier) E:(-29) M:(-27)
					; This gives R·V/√μ, the current generalized momentum
		STORE	TFFQ1		# Q1 = current R·V/√μ, saved for later test
					# E:(-16) M:(-15)
					; Q1 tells us if we're moving toward or away from
					; primary body (positive = outbound, negative = inbound)
		BMN	BDSU		; Branch based on trajectory direction
			INBOUND		# If Q1 < 0, use alternate Z formulation
			TFFTEM		# Q2  E:(-16) M:(-15)

# OUTBOUND Z CALC CONTINUES HERE
;
; For outbound trajectories (moving away from primary body), compute the
; universal variable Z using the standard formulation. Z is related to the
; change in eccentric anomaly and is the key parameter for Kepler's equation.
;
		STODL	TFFX		# NUM = Q2-Q1 (numerator)  E:(-16) M:(-15)
					; Change in generalized momentum Q
			TFFALFA		# Load ALFA = -1/a (negative reciprocal of semi-major axis)
					# E:(26-NR) M:(24-NR)
		DMP	BDSU		; Compute denominator: (2-RTERM*ALFA) - ALFA*RMAG
# Page 1278
			NRMAG		# Current radius magnitude  E:(-29+NR) M:(-27+NR)
					# (2-RTERM ALFA)  (-3) FROM PDL+0
					; This gives denominator = ALFA*(RMAG - RTERM) + 2
SAVEDEN		PUSH	ABS		# DEN TO PDL+0	E:(-3) OR (-16)
					#               M:(-3) OR (-15)
					; Save denominator and take absolute value for test
		DAD	BOV		# INDETERMINANCY TEST
					; Check if denominator is effectively zero
			LIM(-22)	# Add 1.0-B(-22) as minimum threshold
			TFFXTEST	# GO IF DEN >/= B(-22) (denominator is safe)
					; If |DEN| < 2^-22, computation becomes indeterminate
		DLOAD	PDDL		# SET DEN=0 OTHERWISE
					; For near-zero denominators, prevent division errors
			TFFZEROS	; Load zero value
					# XCH ZERO WITH PDL+0
					; Replace indeterminate denominator with zero
		DLOAD	DCOMP		; Check orbit type via sign of ALFA
			TFFALFA		# ALFA  E:(26-NR) M:(24-NR)
					; ALFA < 0 → ellipse, ALFA = 0 → parabola, ALFA > 0 → hyperbola
		BMN	DLOAD		# FOR TPER:  Z INDET AT DELE/2=0 AND 90.
					; If elliptical (ALFA negative), handle specially
			TFFEL1		# ASSUME 90°, AND LEAVE 0 IN PDL: 1/Z=D/N
					; For ellipses at 90° eccentric anomaly change

					# Z INDET. AT PERIGEE FOR PARAB OR HYPERB.
					; For parabolic/hyperbolic orbits, time to periapsis
					; is indeterminate when already at closest approach
DUMPTFF1	RVQ			# RETURN TFF =0
					; Exit with zero time (already at target)

# INBOUND Z CALC CONTINUES HERE
;
; For inbound trajectories (moving toward primary body), use an alternate
; formulation for Z to avoid numerical instability. The inbound case requires
; different numerator and denominator to maintain accuracy when approaching
; periapsis or when Q1 is negative.
;
INBOUND		DLOAD			# RESET PDL+0
					; Clear push-down list for new calculation
		DLOAD	DSU		# ALTERNATE Z CALC
					; For inbound: Z = (RTERM-RN)/(Q2+Q1)
			RTERM		# Terminal radius  E:(-29) M:(-27)
			RMAG1		# Current radius magnitude  E:(-29) M:(-27)
		STODL	TFFX		# NUM = RTERM-RN (numerator)  E:(-29) M:(-27)
					; Radial distance change is numerator for inbound
			TFFTEM		# Q2 (generalized momentum at terminal radius)
					# E:(-16) M:(-15)
		DAD	GOTO		; Compute denominator = Q2 + Q1
			TFFQ1		# Q1 (current generalized momentum)  E:(-16) M:(-15)
					; Sum of momenta gives appropriate denominator
			SAVEDEN		# DEN = Q2+Q1  E:(-16) M:(-15)
					; Jump back to common processing path

TFFXTEST	DAD	PDDL		# (ABS(DEN) TO PDL+2)	E:(-3) OR (-16)
					#			M:(-3) OR (-15)
					; Save |DEN| after restoring from indeterminacy test
			DP(-22)		# RESTORE ABS(DEN) TO MPAC
					; Add back the B(-22) threshold value
			TFFX		# NUM  E:(-16) OR (-29)  M:(-15) OR (-27)
					; Load numerator for Z calculation
		DMP	SR*		; Z = NUM*sqrt(|ALFA|) / DEN
					; Universal variable formulation for conics
			TFFRTALF	# SQRT(ALFA)  E:(10+NA) M:(9+NA)
					; Square root of semi-major axis reciprocal
			0 	-3,2	# X2=-NA
					; Denormalize to match scaling
		DDV			# C(MPAC) = NUM*SQRT(ALFA)  E:(-3) OR (-16)
					#                           M:(-3) OR (-15)
					; Divide by denominator to get Z preliminary value
					# ABS(DEN) FROM PDL+2  E:(-3) OR (-16)
					#                      M:(-3) OR (-15)
		DLOAD	BOV		# (THE DLOAD IS SHARED WITH TFFELL)
					; Check for arithmetic overflow in division
			TFFX		# NUM  E:(-16) OR (-29)  M:(-15) OR (-27)
					; Reload NUM for alternate path if needed
			TFFELL		# USE EQN FOR DELE GEQ 90, LEQ -90
					; On overflow, use ellipse-specific equation
					; Indicates large eccentric anomaly change (>90°)

# OTHERWISE, CONTINUE FOR GENERAL CONIC FOR TFF EQN
;
; For trajectories with moderate eccentric anomaly changes (<90°), use the
; general conic time-of-flight equation based on the universal variable Z.
; This formulation works for all conic types (ellipse, parabola, hyperbola).
;
		DDV	STADR		; Complete Z calculation
					# DEN FROM PDL+0  E:(-3) OR (-16)
					#                 M:(-3) OR (-15)
					; Final division by denominator
		STORE	TFFTEM		# Z SAVE FOR SIGN OF SDELF
					; Store Z temporarily to preserve sign information
# Page 1279
					# Z final scale: E:(-13) M:(-12)
		PUSH	DSQ		# Z TO PDL+0
					; Save Z on stack and compute Z²
		PUSH	DMP		# Z SQ TO PDL+2  E:(-26) M:(-24)
					; Save Z² on stack and start computing Z²·P
			TFFNP		# LC P (semi-latus rectum)  E:(-38+2NR) M:(-36+NR)
					; Multiply by parameter for power series terms
		SL	SIGN		; Shift and apply sign
			5		; Left shift 5 bits for scaling alignment
			TFFTEM		# AFFIX SIGN FOR SDELF (ENTRY DISPLAY)
					; Apply sign of Z for sin(ΔE/2) computation
		STODL	TFFTEM		# P ZSQ  E:(-59+2NR) M:(-55+2NR)
					# (ARG IS USED IN TFF/TRIG)
					; Store P·Z² for trigonometric function computation
					# ZSQ FROM PDL+2  E:(-26) M:(-24)
					; Reload Z² from stack
		PUSH	DMP		# RESTORE PUSH LOC
					; Save Z² back to stack for later use
			TFFALFA		# ALFA  E:(26-NR) M:(24-NR)
					; Multiply Z² by ALFA to form X = Z²·ALFA
		SL*			; Denormalize X
			0,1		# X1=-NR (denormalization count)
					; Shift to proper scale for T(X) polynomial
		STORE	TFFX		# X (argument for power series)
					; X = Z²·ALFA scaled appropriately
		RTB	DMP		; Compute T(X) polynomial and multiply
			T(X)		# POLY (calls power series subroutine)
					; T(X) evaluates series for time equation
					# ZSQ FROM PDL+2  E:(-26) M:(-24)
					; Multiply T(X) result by Z²
		SR2	BDSU		# 2 ZSQ T(X)  E:(-29) M:(-27)
					; Right shift 2 (divide by 4), then reverse subtract
			RTERM		# RTERM  E:(-29) M:(-27)
					; Compute: RTERM - 2·Z²·T(X)
		DAD	DMP		; Add current radius and multiply by Z
			RMAG1		# Current radius  E:(-29) M:(-27)
					; Form: [RTERM - 2·Z²·T(X) + RN]
					# Z FROM PDL+0  E:(-13) M:(-12)
					; Multiply result by Z
		SR3	BPL		# TFF·SQRT(MU)  E:(-45) M:(-42)
					; Right shift 3 for scaling, branch if positive
			ENDTFF		# (NO PUSH UP)
					; If positive, go to completion
		PUSH	SIGN		# TFF·SQRT(MU) TO PDL+0
					; Save preliminary TFF to stack
			TFFQ1		# Q1 FOR GONEPAST TEST
					; Apply sign of Q1 to check trajectory direction
		BPL	DLOAD		# GONE PAST ?
					; If Q1 ≥ 0, spacecraft has passed periapsis
			NEGTFF		# YES. TFF < 0 (retrograde time)
					; Branch to return negative time
			TFF1/ALF	# 1/ALFA  E:(-22-2NA) M:(-20-2NA)
					; Check orbit type via reciprocal of ALFA
		DCOMP	BPL		# ALFA > 0 ?
					; Double complement to test sign
			NEGTFF		# NO. TFF IS NEGATIVE
					; For hyperbolic orbits (ALFA>0), time is negative

# CORRECT FOR ORBITAL PERIOD.
;
; For elliptical orbits where the transfer exceeds one orbital period,
; correct the raw TFF by subtracting the orbital period. This happens when
; calculating time to a radius beyond periapsis on a multi-revolution path.
; Orbital period = 2π / sqrt(|ALFA|) in dimensionless time units.
;
		DCOMP			# YES.  CORRECT FOR ORB PERIOD
					; Double complement (negate) 1/ALFA
		DMP	DDV		; Compute orbital period correction
			PI/16		# 2π scaled as (-5) [2π/16 to fit register]
					; Multiply to get 2π·|ALFA|^(-1)
			TFFRTALF	# SQRT(ALFA)  E:(10+NA) M:(9+NA)
					; Divide by sqrt(ALFA) to get period
		SL*	SL*		; Denormalize period value
			0 	-4,2	# X2=-NA (denormalization count)
			0 	-4,2	; Double shift left by NA positions
		SL*	DAD		; Final shift and add to TFF
			0,2		; One more denormalization shift
					; Add orbital period to raw TFF value
					# TFF·SQRT(MU) FROM PDL+0  E:(-45) M:(-42)
					; Retrieve preliminary TFF and add period
ENDTFF		DMP	BOV		# TFF·SQRT(MU) IN MPAC  E:(-45) M:(-42)
					; Final TFF computation entry point
			TFF/RTMU	# 1/sqrt(μ)  E:(17) M:(14)
					; Convert from TFF·sqrt(μ) to TFF centiseconds
		 	MAXTFF		# SET POSMAX IN OVFL
					; On overflow, return maximum time value
;
; Exit point: Returns time of free fall in centiseconds at scale (-28).
; Positive values indicate future time, negative indicates past event.
;
DUMPTFF2	RVQ			# RETURN TFF (-28) CS IN MPAC
					; Return to calling program with TFF result

# Page 1280
;
; For trajectories where the spacecraft has passed the target radius
; (retrograde in orbital motion), return negative time-of-flight.
;
NEGTFF		DLOAD			; Load negative TFF value
					# TFF·SQRT(MU) FROM PDL+0, NEGATIVE
					; Retrieve sign-corrected TFF from stack
		GOTO			; Jump to final scaling
			ENDTFF		; Complete TFF computation with proper sign
;
; Overflow protection: If TFF calculation overflows, return near-maximum
; representable time value instead of allowing numerical wraparound.
;
MAXTFF1		DLOAD			# RESET PDL (unused entry point)
MAXTFF		DLOAD	RVQ		; Return maximum time value
			NEARONE		; Constant near +1.0 at appropriate scale
					; Prevents overflow from corrupting computation

# TIME OF FLIGHT ELLIPSE WHEN DEL (ECCENTRIC ANOM) GEQ 90 AND LEQ -90.
;
; ============================================================================
; SUBROUTINE: TFFELL - Elliptical Orbit Time-of-Flight for Large Angles
;
; PURPOSE: Compute time-of-flight for elliptical orbits when the change in
;          eccentric anomaly (ΔECC) is ≥90° or ≤-90°. This handles transfers
;          that span more than a quarter of the orbital ellipse, requiring
;          specialized trigonometric formulation to avoid numerical instability.
;
; ENTRY CONDITIONS:
;   - NUM/DEN ratio from TFFX calculation in MPAC and PDL
;   - Orbit parameters (ALFA, Q1, Q2, P, RMAG) computed
;   - X1 register = -NR (normalization count for radius)
;   - X2 register = -NA (normalization count for sqrt(|ALFA|))
;
; METHOD:
;   For large eccentric anomaly changes, the standard Kepler equation becomes
;   numerically unstable. This routine uses an alternate formulation:
;   
;   1. Compute Z = 1/ALFA·(NUM/DEN) where NUM/DEN approximates the universal
;      variable relationship for large angle transfers
;   2. Calculate X = (ALFA·Z²)⁻¹ for the T(X) polynomial approximation
;   3. Use trigonometric identity: TFF = π/√ALFA + 2(X·T(X)-1)/(Z·ALFA) - ΔQ/ALFA
;   4. The π/√ALFA term accounts for the half-period contribution
;
; HISTORICAL CONTEXT:
;   This specialized calculation would be used for lunar orbit insertion burns,
;   transearth injection, or any maneuver requiring large orbital transfers.
;   During Apollo 11, such calculations were critical for planning the LOI
;   (Lunar Orbit Insertion) and TEI (Transearth Injection) burns.
;
; ============================================================================
;
					# NUM FROM TFFX.	E:(-16) OR (-29)
					#			M:(-15) OR (-27)
TFFELL		SL2			# NUM  E:(-14) OR (-27)  M:(-13) OR (-25)
					; Scale numerator by 4 (shift left 2)
		BDDV	PUSH		# TEMP SAVE D/N IN PDL+0
					; Divide NUM by DEN and save to stack
					# DEN FROM PDL+0  E:(-3)/(-16)  M:(-3)/(-15)
					# N/D TO PDL+0  E:(11) M:(10)
					; N/D ratio represents normalized Z parameter
TFFEL1		DLOAD	DSU		# (ENTER WITH D/N=0 IN PDL+0)
					; Alternate entry point for direct calls
			TFFTEM		# Q2  E:(-16) M:(-15)
					; Load Q2 (R₂·V/√μ at terminal position)
			TFFQ1		# Q1  E:(-16) M:(-15)
					; Subtract Q1 (R₁·V/√μ at present position)
		STODL	TFFDELQ		# Q2-Q1  E:(-16) M:(-15)
					; Store ΔQ = Q2-Q1 (change in momentum parameter)
					# D/N FROM PDL+0
					; Retrieve N/D ratio from stack
		STADR			; Store address for indexing
		STORE	TFFTEM		# D/N  E:(11) M:(10)
					; Save N/D ratio in temporary storage
		DMP	SL*		; Compute (1/ALFA)·Z
			TFF1/ALF	# 1/ALFA  E:(-22-2NA) M:(-20-2NA)
					; Multiply N/D by semi-major axis reciprocal
			0,2		# 1/ALFA Z  E:(-11-NA) M:(-10-NA)
					; Denormalize using -NA shift count from X2
		PUSH	DMP		# TO PDL+0
					; Save (1/ALFA)·Z to stack for later π term
			TFFTEM		# 1/Z  E:(11) M:(10)
					; Multiply by N/D again to get Z²
		SL*	BOVB		; Calculate X = (ALFA·Z²)⁻¹
			0,2		# X2= -NA
					; Final denormalization shift
			SIGNMPAC	# IN CASE X= 1.0, CONTINUE
					; Overflow branch if X approaches 1.0
		STORE	TFFX		# X=1/ALFA ZSQ
					; Store X parameter for T(X) polynomial
		RTB	DMP		; Evaluate T(X) polynomial series
			T(X)		# POLY
					; Call T(X) polynomial subroutine
					; Returns T(X) ≈ sin⁻¹(√X)/√X for elliptical case
			TFFX		; Multiply T(X) by X
					; Compute X·T(X) term for Kepler equation
		SR3	DSU		; Compute 2(X·T(X) - 1)
					; Shift right 3 positions (divide by 8)
			DP2(-3)		; Subtract 2.0 at scale (-3)
					; This gives (X·T(X) - 1)/4, then implicit *8 = 2(X·T(X)-1)
		DMP	PUSH		# 2(X T(X)-1) /Z ALFA	E:(-15-NA)
					#			M:(-14-NA)
					; Divide by Z·ALFA to normalize the time term
					; Save result to PDL for final TFF assembly
					# 1/ALFA Z FROM PDL+0	E:(-11-NA)
					#			M:(-10-NA)
					; (1/ALFA)·Z retrieved for π/√ALFA calculation
		DLOAD	DMP		# GET SIGN FOR SDELF
					; Determine sign for sin(Δθ/2) calculation
			TFFTEM		# 1/Z  E:(11) M:(10)
					; Load 1/Z (reciprocal of universal variable)
			RMAG1		# E:(-29) M:(-27)
					; Multiply by present radius magnitude
		SL2	DAD		; Scale by 4 and add momentum parameter
			TFFQ1		# Q1  E:(-16) M:(-15)
					; Add Q1 = R₁·V/√μ (present momentum parameter)
		STODL	TFFTEM		# (Q1+R 1/Z) =SGN OF SDELF  E:(-16) M:(-15)
					; Store sign indicator for SDELF = sin(Δθ/2)
					; This determines if transfer is prograde or retrograde
			TFFNP		# LC P  E:(-38+2NR) M:(-36+2NR)
					; Load semi-latus rectum P = h²/μ
		DMP	SL*		# CALC FOR ARG FOR TFF/TRIG.
					; Calculate P/ALFA for TFF/TRIG argument
# Page 1281
			TFF1/ALF	# 1/ALFA  E:(-22-2NA) M:(-20-2NA)
					; Multiply by 1/ALFA = 1/(semi-major axis)
			1,2		# X2=-NA
					; Normalize using NA scaling
		SIGN	SL*		; Affix sign and scale
			TFFTEM		# AFFIX SIGN FOR SDELF
					; Apply sign determined from momentum parameters
			0,2		; Scale by NA
		STODL	TFFTEM		# P/ALFA  E:(-59+2NR) M:(-55+2NR)
					# (ARG FOR USE IN TFF/TRIG)
					; Store P/ALFA argument for TFF/TRIG subroutine
					; This represents the geometry ratio for angle calculation
			TFF1/ALF	# 1/ALFA  E:(-22-2NA) M:(-20-2NA)
					; Load 1/ALFA for half-period calculation
		SQRT	DMP		; Compute π/√ALFA (half-period of orbit)
			PI/16		# PI (-4)
					; Multiply by π to get half orbital period
					; For large-angle transfers, must add half-period correction
		DAD			; Add the computed T(X) contribution
					# 2(XT(X)-1)/Z ALFA FROM PDL	E:(-15-NA)
					#				M:(-14-NA)
					; Retrieve 2(X·T(X)-1)/(Z·ALFA) from push-down list
					; This is the eccentric anomaly contribution
		SL*	DSU		; Scale and subtract momentum change
			0 	-1,2	; Scale correction for proper units
			TFFDELQ		# Q2-Q1  E:(-16) M:(-15)
					; Subtract ΔQ = (Q₂-Q₁)/ALFA
					; Final TFF = π/√ALFA + 2(X·T(X)-1)/(Z·ALFA) - ΔQ/ALFA
		DMP	SL*		; Convert to time units
			TFF1/ALF	# 1/ALFA  E:(-22-2NA) M:(-20-2NA)
					; Divide by ALFA to get time dimension
			0 	-3,2	; Additional scaling by -3·NA
					; Final normalization for time output
		SL*	GOTO		; Scale and branch to completion
			0 	-4,2	; Final scaling by -4·NA
					; Produces TFF·√μ in proper centiseconds scaling
			ENDTFF		# TFF SQRT(MU) IN MPAC E:(-45) M:(-42)
					; Transfer to common completion routine
					; Returns time-of-flight in centiseconds at (-28)
;
; ============================================================================
; TRANSITION: From TFFELL to T(X) Polynomial Subroutine
;
; The TFFELL subroutine has completed the time-of-flight calculation for
; large-angle elliptical transfers (ΔE ≥ 90° or ≤ -90°). The calculation
; involved the specialized formulation: TFF = π/√ALFA + 2(X·T(X)-1)/(Z·ALFA) - ΔQ/ALFA
;
; The following T(X) subroutine provides the polynomial approximation used
; by both CALCTFF/CALCTPER and TFFELL to evaluate the universal variable
; series expansion. This polynomial is central to all time-of-flight
; calculations in the AGC's conic trajectory computations.
; ============================================================================
;
# Page 1282
# PROGRAM NAME:  T(X)				DATE:  01.17.67
# MOD NO:  0					LOG SECTION:  TIME OF FREE FALL
# MOD BY:  RR BAIRNSFATHER
#
# FUNCTIONAL DESCRIPTION:  THE POLYNOMIAL T(X) IS USED BY TIME OF FLIGHT SUBROUTINES CALCTFF AND
#	CALCTPER TO APPROXIMATE THE SERIES
#		           2     3
#		1/3 -X/5 +X /7 -X /9 ...
#
#	WHERE	X = ALFA Z Z		IF ALFA Z Z LEQ 1
#		X = 1/(ALFA Z Z)	IF ALFA Z Z G 1
#
#	ALSO	X IS NEG FOR HYPERBOLIC ORBITS
#		X = 0 FOR PARABOLIC ORBITS
#		X IS POSITIVE FOR ELLIPTIC ORBITS
#
#	FOR FLIGHT 278, THE POLYNOMIAL T(X) IS FITTED OVER THE RANGE (0,+1) AND HAS A MAXIMUM
#	DEVIATION FROM THE SERIES OF 2 E-5.  (T(X) IS A CHEBYCHEV TYPE FIT AND WAS OBTAINED USING
#	MAX PROGRAM AUTOCURFIT294RRB AND IS VALID TO THE SAME TOLERANCE OVER THE RANGE (-.08,+1).)
#
# CALLING SEQUENCE:	RTB
#				T(X)
#	C(MPAC) = X
#
# SUBROUTINE CALLED:  NONE
#
# NORMAL EXIT MODE:  TC TANZIG
#
# ALARMS:  NONE
#
# OUTPUT:  C(MPAC) = T(X)
#
# ERASABLE INITIALIZATION REQUIRED:
#	C(MPAC) = X
#
# DEBRIS:  NONE
;
; ---------------------------------------------------------------------------
; SUBROUTINE: T(X) - Universal Variable Series Polynomial Approximation
; ---------------------------------------------------------------------------
;
; COMMENT-ONLY READERS:
; The T(X) polynomial is the mathematical heart of all time-of-flight
; calculations in the AGC. This compact polynomial approximation replaces
; an infinite series with just 6 coefficients, achieving 5-decimal accuracy
; for trajectory calculations spanning Earth orbit, lunar transfers, and
; descent to the Moon's surface. Apollo 11's entire trajectory was computed
; using this polynomial thousands of times during the mission.
;
; CODE-ALONG READERS:
; T(X) approximates the universal variable series: 1/3 - X/5 + X²/7 - X³/9 + ...
; The series converges for all conic sections:
;   - X < 0: Hyperbolic trajectories (escape/approach)
;   - X = 0: Parabolic trajectories (limiting case)
;   - X > 0: Elliptic trajectories (closed orbits)
;
; This Chebyshev polynomial fit was computed using MIT's AUTOCURFIT294RRB
; program and has maximum deviation of 2×10⁻⁵ over the range [0, +1],
; valid to same tolerance over [-0.08, +1]. The polynomial form enables
; efficient AGC interpretive evaluation using the POLY instruction.
;
; The universal variable X relates to the trajectory geometry:
;   X = ALFA·Z² where ALFA = 1/(semi-major axis), Z = universal anomaly
; This formulation unifies time-of-flight equations for all conic sections,
; eliminating separate ellipse/hyperbola/parabola logic.
;
; CALLING: RTB T(X) with X parameter in MPAC
; RETURNS: T(X) polynomial value in MPAC
; ---------------------------------------------------------------------------
;
T(X)		TC	POLY
				; Call polynomial evaluation routine
				; Evaluates: C₀ + C₁X + C₂X² + C₃X³ + C₄X⁴ + C₅X⁵
		DEC	4		# N-1
				; Polynomial degree minus 1 = 5-1 = 4
				; Indicates 6 coefficients follow (degree 5 polynomial)
		2DEC	3.333333333 E-1
				; C₀ = 1/3 = 0.333333... (constant term)
				; First term of series approximation
				; Dominates for small X (near-parabolic trajectories)

		2DEC*	-1.999819135 E-1*
				; C₁ = -0.199982 ≈ -1/5 (linear coefficient)
				; Second series term with Chebyshev correction
				; Error term: -0.0001809 from theoretical -0.2

		2DEC*	1.418148467 E-1*
				; C₂ = +0.141815 ≈ +1/7 (quadratic coefficient)
				; Third series term with polynomial fit adjustment
				; Deviation: -0.000899 from theoretical +0.142857

		2DEC* 	-1.01310997 E-1*
				; C₃ = -0.101311 ≈ -1/9 (cubic coefficient)
				; Fourth series term, Chebyshev optimized
				; Error: -0.009801 from theoretical -0.111111

		2DEC*	5.609004986 E-2*
				; C₄ = +0.056090 ≈ +1/11 (quartic coefficient)
				; Fifth series term with fit correction
				; Deviation: -0.034819 from theoretical +0.090909

		2DEC*	-1.536156925 E-2*
				; C₅ = -0.015362 ≈ -1/13 (quintic coefficient)
				; Sixth series term, highest-order correction
				; Large deviation from -0.076923 optimizes overall fit
				; Chebyshev minimax criterion minimizes maximum error

ENDT(X)		TC	DANZIG
				; Normal return from polynomial subroutine
				; MPAC contains T(X) approximation with <2×10⁻⁵ error

TCDANZIG	=	ENDT(X)

# Page 1283
# TFF CONSTANTS
;
; ============================================================================
; TFF Constants Section - Mathematical Parameters and Scaling Values
; ============================================================================
;
; COMMENT-ONLY READERS:
; These mathematical constants enable the Apollo Guidance Computer to
; calculate precise trajectory times across the vast distances between
; Earth and Moon. Despite the computer's limited 15-bit precision, these
; carefully chosen scaling values maintain accuracy from Earth orbit
; (400,000 km radius) down to the lunar surface (1,738 km radius).
;
; CODE-ALONG READERS:
; The constants below provide fixed mathematical values (π/16, scale
; factors, reference radii) used throughout all TFF subroutines. The
; dual-origin capability (Earth/Moon) requires different scale levels:
;   - Earth origin: position at 2^-29 meters, velocity at 2^-7 m/cs
;   - Moon origin: position at 2^-27 meters, velocity at 2^-5 m/cs
; The normalization counts NR=8 and NA=8 provide dynamic scaling to
; prevent overflow/underflow while maintaining ~5 decimal places of
; precision across eight orders of magnitude in trajectory dimensions.
; ============================================================================
;

		BANK	32

		SETLOC	TOF-FF1
		BANK

#						# NOTE:  ADJUSTED MUE FOR NEAR EARTH TRAJ.
#MUE		=	3.990815471 E10		# M CUBE/CS SQ
#RTMUE		=	1.997702549 E5 B-18*	# MODIFIED EARTH MU
#
#						# NOTE:  ADJUSTED MUE FOR NEAR EARTH TRAJ.
#MUM		=	4.902778 E8		# M CUBE/CS SQ
#RTMUM		2DEC*	2.21422176 E4 B-18*

				; Mathematical constant π/16 used in CALCTPER for periapsis time calculation
				; Scaled at 2^-4 (1/16), providing ~10 decimal places of precision
				; Used to convert normalized angles to time intervals
PI/16		2DEC	3.141592653 B-4

				; Maximum value at B(-22) scaling: 1.0 - 2^-22 (≈0.9999998)
				; Used as upper bound check to prevent overflow in semi-major axis calculation
LIM(-22)	2OCT	3777737700		# 1.0 -B(-22)

				; Minimum increment at B(-22) scaling: 2^-22 (≈2.384×10⁻⁷)
				; Provides precision floor for convergence tests
DP(-22)		2OCT	0000000100		# B(-22)

				; Scale factor 2^-3 = 1/8, used for normalization adjustments
DP2(-3)		2DEC	1 B-3

				; Scale factor 2^-4 = 1/16, fundamental scaling unit for angle/time conversion
DP2(-4)		2DEC	1 B-4			# 1/16

				; Launch pad radius (Earth surface radius at launch site)
				; Original value: 6373338 meters at B(-29) scaling = 20,909,901.57 feet
				; Now references shared RPAD constant from common definitions
# RPAD1		2DEC	6373338 B-29		# M (-29) = 20909901.57 FT

RPAD1		=	RPAD

				; Reference radius for 300,000-foot (≈91.4 km) altitude
				; 6464778 meters at B(-29) = Earth radius + 300K feet
				; Used in entry corridor and reentry trajectory calculations
R300K		2DEC	6464778 B-29		# (-29) M

				; Convergence threshold: 0.999999999 (nine nines of precision)
				; Used to detect when iterative solutions have converged to acceptable accuracy
NEARONE		2DEC	.999999999

				; Reference to common zero-initialized 6-word block
				; Used to clear MPAC registers and initialize state vectors
TFFZEROS	EQUALS	HI6ZEROS
				; Reference to common constant 1/4 at high precision
				; Used in T(X) polynomial approximation and angle bisection
TFF1/4		EQUALS	HIDP1/4

