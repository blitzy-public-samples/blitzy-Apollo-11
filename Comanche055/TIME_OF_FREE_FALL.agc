# Copyright:	Public domain.
# Filename:	TIME_OF_FREE_FALL.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1373-1388
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Colossus249/ file of the same
#				name, using Comanche055 page images.
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

# ============================================================================
# FILE: TIME_OF_FREE_FALL.agc
# MODULE: CHIEFTAN Subsystem (Core OS)
# MISSION PHASE: all-phases
#
# TL;DR: Free-fall trajectory time-of-flight calculations solving Lambert
#        problem variations. Computes transfer times between position states
#        for targeting and rendezvous applications throughout Apollo 11 orbital
#        mechanics computations. Used for mission planning calculations during
#        translunar coast, lunar orbit, and rendezvous operations.
#
# COMMENT-ONLY READERS: These routines calculated how long it would take the
#        spacecraft to coast between two points in space without engine thrust,
#        essential for timing rendezvous maneuvers and orbit transfers.
# CODE-ALONG READERS: Study time-of-flight computation algorithms, Lambert
#        problem solutions, conic section mathematics, and Kepler orbit
#        propagation supporting orbital mechanics throughout the mission.
# ============================================================================

# Page 1373
;
; ============================================================================
; SCALING AND COORDINATE SYSTEM INFORMATION
;
; The Time of Free Fall (TFF) routines work universally for both Earth-centered
; and Moon-centered coordinates. The calling program specifies which celestial
; body by providing position, velocity, and gravitational parameter (MU) at
; the appropriate scale factors. This flexibility allowed Apollo 11's guidance
; computer to perform the same orbital calculations during translunar coast
; (Earth-centered), lunar orbit operations (Moon-centered), and rendezvous
; maneuvers (either reference frame).
; ============================================================================
;
# 	THE TFF SUBROUTINES MAY BE USED IN EITHER EARTH OR MOON CENTERED COORDINATES.  THE TFF ROUTINES NEVER
# KNOW WHICH ORIGIN APPLIES.  IT IS THE USER WHO KNOWS, AND WHO SUPPLIES RONE, VONE AND 1/SQRT(MU) AT THE
# APPROPRIATE SCALE LEVEL FOR THE PROPER PRIMARY BODY.

; Scaling Factors for Earth-Centered Calculations:
; Position vectors scaled by 2^-29 meters (each unit = 1.86 nanometers)
; Velocity vectors scaled by 2^-7 meters/centisecond
; Gravitational parameter 1/SQRT(MU) scaled by 2^+17
;
#	EARTH ORIGIN	POSITION	-29	METERS
#			VELOCITY	-7	METERS/CENTISECOND
#			1/SQRT(MU)	+17	SQRT(CS SQ/METERS CUBED)
;
; Scaling Factors for Moon-Centered Calculations:
; Position vectors scaled by 2^-27 meters (larger unit due to smaller distances)
; Velocity vectors scaled by 2^-5 meters/centisecond
; Gravitational parameter 1/SQRT(MU) scaled by 2^+14 (Moon's weaker gravity)
;
#	MOON ORIGIN	POSITION	-27	METERS
#			VELOCITY	-5	METERS/CENTISECONDS
#			1/SQRT(MU)	+14	SQRT(CS SQ/METERS CUBED)

; All TFF subroutines return time-of-flight in centiseconds scaled by 2^-28.
; This uniform time scale (1 unit = 3.73 nanoseconds) allows precise timing
; calculations for orbital maneuvers. During Apollo 11's rendezvous between
; the Lunar Module Eagle and Command Module Columbia, these time computations
; determined when to ignite engines for precise orbit insertion.
;
# ALL DATA PROVIDED TO AND RECEIVED FROM ANY TFF SUBROUTINE WILL BE AT ONE OF THE LEVELS ABOVE.  IN ALL CASES,
# THE FREE FALL TIME IS RETURNED IN CENTISECONDS AT (-28).  PROGRAM TFF/CONIC WILL GENERATE VONE/RTMU AND
# LEAVE IT IN VONE' AT (+10) IF EARTH ORIGIN AND (+9) IF MOON ORIGIN.
; USAGE REQUIREMENTS FOR TFF SUBROUTINES:
; Calling program must prepare state vector (position RONE, velocity VONE) and
; gravitational parameter 1/SQRT(MU) in TFF/RTMU at proper scales before calling.
; Extended verb lockout required since RONE/VONE share storage with DSKY verbs.
; This prevented crew DSKY operations from corrupting orbital calculations during
; critical rendezvous computations.
;
; RESTRICTION: CALC/TFF and CALC/TPER assume terminal radius < present radius
; (descending toward target). This covered Apollo 11's descent to lower orbits.
; Ascending trajectory calculations would require code modification (15 words).
;
# 	THE USER MUST STORE THE STATE VECTOR IN RONE, VONE AND MU IN THE FORM 1/SQRT(MU) IN TFF/RTMU
# AT THE PROPER SCALE BEFORE CALLING TFF/CONIC.  SINCE RONE, VONE ARE IN THE EXTENDED VERB STORAGE AREA,
# THE USER MUST ALSO LOCK OUT THE EXTENDED VERBS, AND RELEASE THEM WHEN FINISHED.
# 	PROGRAMS CALC/TFF AND CALC/TPER ASSUME THAT THE TERMINAL RADIUS IS LESS THAN THE PRESENT
# RADIUS.  THIS RESTRICTION CAN BE REMOVED BY A 15 W CODING CHANGE, BUT AT PRESENT IT IS NOT DEEMED NECESSARY.
#
; ============================================================================
; ERASABLE MEMORY VARIABLES (PUSH LIST STORAGE)
;
; TFF subroutines use these temporary variables stored in the AGC's push list
; area (equivalent to modern stack memory). Variables persist across subroutine
; calls but are local to the TFF computation sequence. Notation: E: indicates
; Earth-origin scaling, M: indicates Moon-origin scaling.
; ============================================================================
;
# 	THE FOLLOWING ERASABLE QUANTITIES ARE USED BY THE TFF ROUTINES, AND ARE LOCATED IN THE PUSH LIST.
#

#				BELOW	E:  IS USED FOR EARTH ORIGIN SCALE
#					M:  IS USED FOR MOON  ORIGIN SCALE

#TFFSW		=	119D	# BIT1	0 = CALCTFF		1 = CALCTPER
TFFDELQ		=	10D	#	Q2-Q1			E: (-16)  M: (-15)
RMAG1		=	12D	#	ABVAL(RN) M		E: (-29)  M: (-27)
#RPER		=	14D	#	PERIGEE RADIUS M	E: (-29)  M: (-27)
TFFQ1		=	14D	#	R.V / SQRT(MUE)		E: (-16)  M: (-15)
#SDELF/2			#	SIN(THETA) /2
CDELF/2		=	14D	#	COS(THETA) /2
#RAPO		=	16D	#	APOGEE RADIUS M		E: (-29)  M: (-27)
NRTERM		=	16D	#	TERMINAL RADIUS M	E: (-29+NR)
				#				M: (-27+NR)
RTERM		=	18D	#	TERMINAL RADIUS M	E: (-29)  M: (-27)
TFFVSQ		=	20D	#	-(V SQUARED/MU)       1/M  E: (20)   M: (18)
TFF1/ALF	=	22D	#	SEMI MAJ AXIS  M	E: (-22-2 NA)
				#				M: (-20-2 NA)
TFFRTALF	=	24D	#	SQRT(ALFA)	E:(10+NA) M: (9+NA)
TFFALFA		=	26D	#	ALFA  1/M	E:(26-NR) M: (24-NR)
TFFNP		=	28D	#	SEMI LATUS RECTUM	M  E: (-38+2 NR)
				#				   M: (-36+2 NR)
TFF/RTMU	=	30D	#	1/SQRT(MU)		E: (17)   M: (14)
NRMAG		=	32D	#	PRESENT RADIUS  M	E: (-29+NR)
				#				M: (-27+NR)
TFFX		=	34D     #
TFFTEM		=	36D	#	TEMPORARY
# Page 1374
; REGISTER USAGE CONVENTIONS:
; S1, S2 registers: Preserved by all TFF subroutines (caller can rely on values)
; X1, X2 index registers: Established by TFFCONIC, must be preserved between calls
;   X1 contains -NR (normalization count for radius magnitude)
;   X2 contains -NA (normalization count for semi-major axis square root)
; These normalization counts optimize fixed-point arithmetic precision by tracking
; how many left-shifts were applied to scale values into optimal bit ranges.
;
#		REGISTERS S1, S2 ARE UNTOUCHED BY ANY TFF SUBROUTINE
#		INDEX REGISTERS X1, X2 ARE USED BY ALL TFF SUBROUTINES.  THEY ARE ESTAB-
#		LISHED IN TFF/CONIC AND MUST BE PRESERVED BETWEEN CALLS TO SUBSEQUENT
#		SUBROUTINES.
#		-NR				C(X1) = NORM COUNT OF RMAG
#		-NA				C(X2)= NORM COUNT OF SQRT(ABS(ALFA))

# Page 1375
;
; ============================================================================
; TRANSITION: From variable declarations to TFFCONIC subroutine
;
; TFFCONIC is the foundational routine that must be called first in any TFF
; computation sequence. It analyzes the current orbital trajectory, computes
; conic section parameters (semi-latus rectum, semi-major axis, eccentricity),
; and determines orbit type (ellipse/parabola/hyperbola). During Apollo 11's
; mission, these calculations were essential for planning every orbit change:
; translunar injection, lunar orbit insertion, descent orbit, and rendezvous.
; ============================================================================
;
# SUBROUTINE NAME:	TFFCONIC						DATE: 01.29.67
# MOD NO: 0									LOG SECTION: TIME OF FREE FALL
# MOD BY: RR BAIRNSFATHER
; TFFCONIC computes fundamental conic section parameters from position and velocity:
; - Semi-latus rectum (p): Characterizes orbit size and shape
; - Semi-major axis (a): Determines orbital period and energy
; - Eccentricity: Distinguishes elliptical, parabolic, or hyperbolic trajectories
; These parameters enable subsequent time-of-flight calculations for targeting.
;
# MOD NO: 1		MOD BY: RR BAIRNSFATHER		DATE: 11 APR 67
# MOD NO: 2		MOD BY: RR BAIRNSFATHER		DATE: 21 NOV 67		ADD MOON MU.
# MOD NO: 3		MOD BY: RR BAIRNSFATHER		DATE: 21 MAR 68		ACCEPT DIFFERENT EARTH/MOON SCALES
# FUNCTIONAL DESCRIPTION:	THIS SUBROUTINE IS CALLED TO COMPUTE THOSE CONIC PARAMETERS REQUIRED BY THE TFF
#	SUBROUTINES AND TO ESTABLISH THEM IN THE PUSH LIST AREA.  THE PARAMETERS ARE LISTED UNDER OUTPUT.
#	THE EQUATIONS ARE:
#		-   -  -
#		H = RN*VN			ANGULAR MOMENTUM
#		      - -
#		LCP = H.H / MU			SEMI LATUS RECTUM
#		              -  -
#		ALFA = 2/RN - VN.VN / MU	RECIPROCAL SEMI MAJ AXIS, SIGNED

# 	AND ALFA IS POS FOR ELLIPTIC ORBITS
#	              0 FOR PARABOLIC ORBITS
#	            NEG FOR HYPERBOLIC ORBITS.
#	SUBROUTINE ALSO COMPUTES AND SAVES RMAG.
# CALLING SEQUENCE:
#	TFFCONIC EXPECTS CALLER TO ENTER WITH CORRECT GRAVITATIONAL CONSTANT IN MPAC, IN THE FORM
#	1/SQRT(MU).  THE PROGRAM WILL SAVE IN TFF/RTMU .  THE SCALE IS DETERMINED BY WHETHER EARTH OR MOON
#	ORIGIN IS USED.  THE CALLER MUST LOCK OUT THE EXTENDED VERBS BEFORE PROVIDING STATE VECTOR IN RONE,
#	VONE AT PROPER SCALE.  THE EXTENDED VERBS MUST BE RESTORED WHEN THE CALLER IS FINISHED USING THE
#	TFF ROUTINES.
#	ENTRY POINT TFFCONMU EXPECTS THAT TFF/RTMU IS ALREADY LOADED.
#	TO SPECIFY MU:	DLOAD	CALL		# IF MU ALREADY STORED:		CALL
#				YOURMU	# 1/RTMU	E:(17) M:(14)			TFFCONMU
#				TFFCONIC
#	PUSHLOC = PDL+0, ARBITRARY IF LEQ 18D

# SUBROUTINES CALLED:	NONE
# NORMAL EXIT MODES:	RVQ
# ALARMS:	NONE
# OUTPUT:	THE FOLLOWING ARE STORED IN THE PUSH LIST AREA.
#		RMAG1	E:(-29) M:(-27)	M  RN, PRESENT RADIUS LENGTH.
#		NRMAG	E:(-29+NR)	M  RMAG, NORMALIZED
#			M:(-27+NR)
#		X1			-NR, NORM COUNT
#		TFFNP	E:(-38+2NR)	M  LCP, SEMI LATUS RECTUM, WEIGHTED BY NR.	FOR VGAMCALC
#			M:(-36+2NR)
#		TFF/RTMU E:(17) M:(14)		1/SQRT(MU)
#		TFFVSQ E:(20) M:(18)	1/M  -(V SQ/MU): PRESENT VELOCITY,NORMLIZED.	FOR VGAMCALC
#		TFFALFA	E:(26-NR)	1/M  ALFA, WEIGHTED BY NR
#			M:(24-NR)
#		TFFRTALF E:(10+NA)	SQRT(ALFA), NORMALIZED
#			 M:(9+NA)
# Page 1376
#		X2			-NA, NORM COUNT
#		TFF1/ALF E: (-22-2NA)	SIGNED SEMI MAJ AXIS, WEIGHTED BY NA
#			 M: (-20-2NA)
#		PUSHLOC AT PDL+0
#		THE FOLLOWING IS STORED IN GENERAL ERASABLE
#		VONE'	E:(10) M:(9)	V/RT(MU), NORMALIZED VELOCITY
# ERASABLE INITIALIZATION REQUIRED:
#		RONE	E:(-29) M:(-27)	M	STATE VECTOR		LEFT BY CALLER
#		VONE	E:(-7) M:(-5)	M/CS	STATE VECTOR		LEFT BY CALLER
#		TFF/RTMU E:(17) M:(14)	1/RT(CS SQ/M CUBE)		IF ENTER VIA TFFCONMU.
# DEBRIS:	QPRET.		PDL+0 ... PDL+3
#

		BANK	33
		SETLOC	TOF-FF
		BANK

		COUNT*	$$/TFF

; TFFCONIC ENTRY POINT: Caller provides 1/SQRT(MU) in MPAC
; This gravitational parameter distinguishes Earth (MU = 3.986e14 m^3/s^2)
; from Moon (MU = 4.903e12 m^3/s^2) orbits.
;
TFFCONIC	STORE	TFF/RTMU	# 1/SQRT(MU)		E: (17)  M: (14)

; TFFCONMU ENTRY POINT: Alternate entry when 1/SQRT(MU) already stored
; Used when multiple TFF calculations use same gravitational parameter.
;
; STEP 1: Compute position unit vector and magnitude
; Normalize position vector RONE to get direction and extract length RMAG.
; The AGC's UNIT instruction produces both unit vector (direction) and
; scalar magnitude, optimizing the computation compared to separate operations.
;
TFFCONMU	VLOAD	UNIT		# COME HERE WITH TFFRTMU LOADED.
			RONE		# SAVED RN.  M		E: (-29) M: (-27)
		PDDL			# UR/2 TO PDL+0, +5
			36D		# MAGNITUDE
		STORE	RMAG1		# M	E:(-29)	M:(-27)

; Normalize RMAG to optimize precision for subsequent calculations.
; X1 register stores normalization count (-NR) for later scaling adjustments.
;
		NORM
			X1		# -NR
		STOVL	NRMAG		# RMAG  M  E: (-29+NR) M: (-27+NR)
			VONE		# SAVED VN.  M/CS  	E: (-7)	M: (-5)
; STEP 2: Normalize velocity vector V/SQRT(MU)
; Creates dimensionless velocity used in orbital mechanics equations.
; Stored in VONE' for subsequent orbital parameter calculations.
;
		VXSC
			TFF/RTMU	# E:(17) M:(14)
		STORE	VONE'		# VN/SQRT(MU)  E: (10)	M: (9)

; STEP 3: Compute angular momentum H = R x V and semi-latus rectum p = H²/MU
; Angular momentum (cross product of position and velocity) is perpendicular
; to the orbital plane. Its magnitude determines orbit shape and size.
; Semi-latus rectum p characterizes the orbit's "width" at 90° from periapsis.
;
		VXSC	VXV
			NRMAG		# E: (-29+NR) M: (-27+NR)
					# UR/2 FROM PDL
		VSL1	VSQ		# BEFORE:  E:(-19+NR) M:(-18+NR)
		STODL	TFFNP		# LC P M    E:(-38+2NR) M:(-36+2NR)
					# SAVE ALSO FOR VGAMCALC
; STEP 4: Compute 2/R (needed for vis-viva equation)
; The vis-viva equation relates velocity, position, and orbital energy:
; V²/MU = 2/R - 1/a, where 'a' is the semi-major axis.
;
			TFF1/4
		DDV	PDVL		# (2/RMAG)  1/M  E:(26-NR)	M:(24-NR)
			NRMAG		# RMAG  M  E:(-29+NR)	M:(-27+NR)
			VONE'		# SAVED VN.  E:(10)	M:(9)
; STEP 5: Compute V²/MU (dimensionless specific orbital energy)
; Negative values indicate elliptical (bound) orbits like Apollo 11's lunar orbit.
; Zero indicates parabolic escape trajectory. Positive indicates hyperbolic.
;
		VSQ	DCOMP		# KEEP MPAC+2 HONEST FOR SQRT.
		STORE	TFFVSQ		# -(V SQ/MU)  E:(20)	M:(18)
					# SAVE FOR VGAMCALC
; STEP 6: Compute ALFA = 2/R - V²/MU = 1/a (reciprocal semi-major axis)
; From vis-viva equation. ALFA determines orbit type and energy:
;   ALFA > 0: Elliptical orbit (bound, Apollo 11's Earth/lunar orbits)
;   ALFA = 0: Parabolic trajectory (exactly escape velocity)
;   ALFA < 0: Hyperbolic trajectory (excess velocity, deep space missions)
;
		SR*	DAD
# Page 1377
			0 -6,1		# GET -VSQ/MU  E:(26-NR) M:(24-NR)
		STADR
					# 2/RMAG  FROM PDL+2
		STORE	TFFALFA		# ALFA  1/M  E:(26-NR) M:(24-NR)
; STEP 7: Compute SQRT(ABS(ALFA)) for time-of-flight equations
; Many orbital time equations use SQRT(ALFA). Taking absolute value handles
; both elliptical (positive) and hyperbolic (negative) cases uniformly.
; Normalize result for precision (X2 stores normalization count -NA).
;
		SL*	PUSH		# TEMP SAVE ALFA  E:(20)  M:(18)
			0 -6,1
		ABS	SQRT		# E:(10) M:(9)
		NORM
			X2		# X2 = -NA
		STORE	TFFRTALF	# SQRT( ABS(ALFA) )  E:(10+NA) M:(9+NA)
; STEP 8: Compute 1/ALFA (semi-major axis a)
; Semi-major axis determines orbital period via Kepler's third law.
; Special handling: If ALFA near zero (parabolic orbit), set 1/ALFA = 0.
; This prevents division-by-zero and indicates orbit approaching parabolic.
;
		DSQ	SIGN		# NOT SO ACCURATE, BUT OK
					# ALFA FROM PDL+2  E:(20) M:(18)
		BZE	BDDV		# SET 1/ALFA =0, TO SHOW SMALL ALFA
			+2
			TFF1/4
	+2	STORE	TFF1/ALF	# 1/ALFA  E:(-22-2 NA)	M:(-20-2 NA)
DUMPCNIC	RVQ
					#			39 W
# Page 1378
# SUBROUTINE NAME:	TFFRP/RA						DATE: 01.17.67
# MOD NO: 0									LOG SECTION: 	TIME OF FREE FALL
# MOD NO: 1		MOD BY: RR BAIRNSFATHER		DATE: 11 APR 67
# MOD NO: 2		MOD BY: RR BAIRNSFATHER		DATE: 21 MAR 68		ACCEPT DIFFERENT EARTH/MOON SCALES
#										ALSO IMPROVE ACCURACY OF RAPO.
# FUNCTIONAL DESCRIPTION:	USED BY CALCTPER AND TFF DISPLAYS TO CALCULATE PERIGEE RADIUS AND ALSO
#	APOGEE RADIUS FOR A GENERAL CONIC.
#	PROGRAM GIVES PERIGEE RADIUS AS		APOGEE RADIUS IS GIVEN BY
#		RP = P /(1+E)				RA = (1+E) / ALFA
#	WHERE    2
#		E  = 1 - P ALFA
#	IF RA IS NEGATIVE OR SHOWS DIVIDE OVERFLOW, THEN RA = POSMAX BECAUSE
#		1. APOGEE RADIUS IS NOT MEANINGFUL FOR HYPERBOLA
#		2. APOGEE RADIUS IS NOT DEFINED FOR PARABOLA
#		3. APOGEE RADIUS EXCEEDS THE SCALING FOR ELLIPSE.
#	THIS SUBROUTINE REQUIRED THE SIGNED RECIPROCAL SEMI MAJ AXIS, ALFA, AND SEMI LATUS RECTUM AS DATA.
# CALLING SEQUENCE: CALL
#			TFFRP/RA
#	PUSHLOC = PDL+0, ARBITRARY IF LEQ 10D
#	C(MPAC) UNSPECIFIED

# SUBROUTINES CALLED:	NONE
# NORMAL EXIT MODE:	RVQ
#		IF ELLIPSE, WITHIN NORMAL SCALING, RAPO IS CORRECT.
#		OTHERWISE, RAPO = POSMAX.
# ALARMS:	NONE
# OUTPUT:	STORED IN PUSH LIST AREA. SCALE OF OUTPUT AGREES WITH DATA SUPPLIED TO TFF/CONIC.
#		RPER	E:(-29) M:(-27)	 M	PERIGEE RADIUS		DESTROYED BY CALCTFF/CALCTPER, TFFTRIG.
#		RAPO	E:(-29) M:(-27)	 M	APOGEE RADIUS		WILL BE DESTROYED BY CALCTFF/CALCTPER
#		PUSHLOC AT PDL+0
# ERASABLE INITIALIZATION REQUIRED:
#		TFFALFA	E:(26-NR)	M	1/SEMI MAJ AXIS		LEFT BY TFFCONIC
#			M:(24-NR)
#		TFFNP	E: (-38+2NR)	M	LC P, SEMI LATUS RECTUM	LEFT BY TFFCONIC
#			M: (-36+2NR)
#		X1			-NR, NORM COUNT OF RMAG		LEFT BY TFFCONIC
#		X2			-NA, NORM COUNT OF ALFA		LEFT BY TFFCONIC
# DEBRIS:	QPRET, PDL+0 ... PDL+1

# Page 1379
; ============================================================================
; SUBROUTINE: TFFRP/RA - Compute Perigee and Apogee Radii
;
; COMMENT-ONLY READERS: This routine calculates the lowest (perigee) and
; highest (apogee) points of the spacecraft's elliptical orbit. For Apollo 11,
; this determined orbital altitude extremes during Earth and lunar orbits.
;
; CODE-ALONG READERS: Implements classical orbital mechanics equations:
;   Rp = p/(1+e)  [perigee radius]
;   Ra = (1+e)/ALFA  [apogee radius]
;   where e² = 1 - p·ALFA  [eccentricity]
; Handles special cases: hyperbola/parabola return POSMAX for apogee.
; ============================================================================
;
RAPO		=	16D		# APOGEE RADIUS  M  E:(-29) M:(-27)
RPER		=	14D		# PERIGEE RADIUS  M  E:(-29) M:(-27)

; STEP 1: Compute eccentricity e = SQRT(1 - p·ALFA)
; Eccentricity determines orbit shape:
;   e = 0: Circular (Apollo parking orbits aimed for low eccentricity)
;   0 < e < 1: Ellipse (typical Apollo orbits)
;   e = 1: Parabola, e > 1: Hyperbola
;
TFFRP/RA	DLOAD	DMP
			TFFALFA		# ALFA  1/M  E:(26-NR) M:(24-NR)
			TFFNP		# LC P  M   E:(-38+2NR) M:(-36+2NR)
		SR*	DCOMP		# ALFA P (-12+NR)
			0 -8D,1		# ALFA P (-4)
		DAD	ABS		# (DCOMP GIVES VALID TP RESULT FOR SQRT)
					# (ABS PROTECTS SQRT IF E IS VERY NEAR 0)
			DP2(-4)
		SQRT	DAD		# E SQ = (1- P ALFA)	(-4)
			TFF1/4
; STEP 2: Compute perigee radius Rp = p/(1+e)
; Perigee is the closest approach point to central body (Earth or Moon).
; For Apollo 11's lunar orbit, this was approximately 60 nautical miles altitude.
; The formula divides semi-latus rectum p by (1+eccentricity).
;
		PUSH	BDDV		# (1+E)  (-2)  TO PDL+0
			TFFNP		# LCP  M E:(-38+2NR)	M:(-36+2NR)
		SR*	SR*		# (DOES SR THEN SL TO AVOID OVFL)
			0,1		# X1=-NR
			0 -7,1		# (EFFECTIVE SL)
		STODL	RPER		# PERIGEE RADIUS  M  E:(-29) M:(-27)
					# (1+E)  (-2)  FROM PDL+0
; STEP 3: Compute apogee radius Ra = (1+e)/ALFA
; Apogee is the farthest point from central body in elliptical orbit.
; For Apollo 11's lunar orbit, this was approximately 170 nautical miles altitude.
; Special cases handled:
;   - If ALFA=0 (parabolic): Set Ra = POSMAX (infinite apogee)
;   - If ALFA<0 (hyperbolic): Set Ra = POSMAX (no apogee exists)
;   - If overflow during computation: Set Ra = POSMAX
;
		DMP	BOVB
			TFF1/ALF	# E:(-22-2NA) M:(-20-2NA)
			TCDANZIG	# CLEAR OVFIND, IF ON.
		BZE	SL*
			MAXRA		# SET POSMAX, IF ALFA=0
			0 -5,2		# -5+NA
		SL*	BOV
			0,2
			MAXRA		# SET POSMAX IF OVFL.
		BPL			# CONTINUE WITH VALID RAPO.
			+3
MAXRA		DLOAD			# RAPO CALC IS NOT VALID.  SET RAPO =
			NEARONE		# POSMAX AS A TAG.
	+3	STORE	RAPO		# APOGEE RADIUS  M  E:(-29)	M:(-27)
DUMPRPRA	RVQ

; ============================================================================
; TRANSITION: From orbital parameter calculations to time-of-flight computation
;
; With perigee/apogee radii computed, the guidance system now calculates the
; actual time required to travel between two points along the orbital path.
; This is critical for Apollo 11 mission planning - determining how long it
; will take to reach specific positions during orbit, rendezvous maneuvers,
; or trajectory corrections. The calculations handle all orbit types:
; elliptical (closed orbits like lunar or Earth orbits), parabolic (escape
; trajectories), and hyperbolic (fast escape or interplanetary trajectories).
; ============================================================================

					#			30 W
# Page 1380
# SUBROUTINE NAME:	CALCTPER / CALCTFF					DATE:	01.29.67
# MOD NO: 0									LOG SECTION:	TIME OF FREE FALL
# MOD BY: RR BAIRNSFATHER
# MOD NO: 1		MOD BY: RR BAIRNSFATHER		DATE: 21 MAR 67
# MOD NO: 2		MOD BY: RR BAIRNSFATHER		DATE: 14 APR 67
# MOD BY: 3		MOD BY: RR BAIRNSFATHER		DATE: 8 JUL 67		NEAR EARTH MUE AND NEG TFF (GONEPAST)
# MOD BY: 4		MOD BY: RR BAIRNSFATHER		DATE: 21 NOV 67		ADD VARIABLE MU.
# MOD BY: 5		MOD BY: RR BAIRNSFATHER		DATE: 21 MAR 68		ACCEPT DIFFERENT EARTH/MOON SCALES
# FUNCTIONAL DESCRIPTION:	PROGRAM CALCULATES THE FREE-FALL TIME OF FLIGHT FROM PRESENT POSITION RN AND
#	VELOCITY VN TO A RADIUS LENGTH SPECIFIED BY RTERM , SUPPLIED BY THE USER.  THE POSITION VECTOR
#	RN MAY BE ON EITHER SIDE OF THE CONIC, BUT RTERM IS CONSIDERED ON THE INBOUND SIDE.
#	THE EQUATIONS ARE

#		Q2 = -SQRT(RTERM (2-RTERM ALFA) - LCP)	(INBOUND SIDE))	LEQ +- LCE/SQRT(ALFA)
#		     -	-
#		Q1 = RN.VN / SQRT(MU)					LEQ +- LCE/SQRT(ALFA)

# 		Z = NUM / DEN						LEQ +- 1/SQRT(ALFA)

#	WHERE, IF INBOUND
#		NUM = RTERM -RN						LEQ +- 2 LCE/ALFA
#		DEN = Q2+Q1						LEQ +- 2 LCE/SQRT(ALFA)

# 	AND, IF OUTBOUND
#		NUM = Q2-Q1						LEQ +- 2 LCE/SQRT(ALFA)
#		DEN = 2 - ALFA (RTERM + RN) .				LEQ +- 2 LCE

#	IF 	ALFA ZZ < 1.0		(FOR ALL CONICS EXCEPT ELLIPSES HAVING ABS(DEL ECC ANOM) G 90 DEG)

#	THEN	X = ALFA Z Z
#	AND	TFF = (RTERM +RN -2 ZZ T(X) ) Z/SQRT(MU)

#		EXCEPT 	IF ALFA PNZ, AND IF TFF NEG,
#		THEN	TFF = 2 PI /(ALFA SQRT(ALFA)) + TFF

#	OR IF	ALFA ZZ GEQ 1.0		(FOR ELLIPSES HAVING ABS(DEL ECC ANOM) GEQ 90 DEG)

#	THEN	X = 1/ALFA Z Z
#	AND	TFF = (PI/SQRT(ALFA) -Q2 +Q1 +2(X T(X) -1) /ALFA Z) /ALFA SQRT(MU)

#	WHERE	T(X) IS A POLYNOMIAL APPROXIMATION TO THE SERIES
#			   2	 3			  2
#		1/3 -X/5 +X /7 -X /9...			(X < 1.0)

# CALLING SEQUENCE:	TIME TO RTERM			TIME TO PERIGEE
#			CALL				CALL
#				CALCTFF				CALCTPER
#			C(MPAC) = TERMNL RAD M		C(MPAC) = PERIGEE RAD M
#	FOR EITHER,	E: (-29)	M: (-27)
#	FOR EITHER, PUSHLOC = PDL+0 , ARBITRARY IF LEQ 8D.
# Page 1381
#
# SUBROUTINES CALLED:	T(X), VIA RTB
# NORMAL EXIT MODE: RVQ
#		HOWEVER, PROGRAM EXITS WITH ONE OF THE FOLLOWING VALUES FOR TFF (-28) CS IN MPAC.  USER MUST STORE.
#			A. TFF = FLIGHT TIME.  NORMAL CASE FOR POSITIVE FLIGHT TIME LESS THAN ONE ORBITAL PERIOD.
#			B. (THIS OPTION IS NO LONGER USED.)
#			C. TFF = POSMAX.  THIS INDICATES THAT THE CONIC FROM THE PRESENT POSITION WILL NOT RETURN TO
#			   THE SPECIFIED ALTITUDE.  ALSO INDICATES OUTBOUND PARABOLA OR HYPERBOLA.
# OUTPUT:	C(MPAC)		(-28) CS	TIME OF FLIGHT, OR TIME TO PERIGEE
#		TFFX		(0)		X,					LEFT FOR ENTRY DISPLAY TFF ROUTINES
#		NRTERM		E: (-29+NR)	M RTERM, WEIGHTED BY NR			LEFT FOR ENTRY DISPLAY TFF ROUTINES
#				M: (-27+NR)
#		TFFTEM		E: (-59+2NR)	LCP Z Z SGN(SDELF)			LEFT FOR ENTRY DISPLAY TFF ROUTINES
#				M: (-55+2NR)	LCP /ALFA SGN(SDELF)			LEFT FOR ENTRY DISPLAY TFF ROUTINES
#		NOTE:	TFFTEM = PDL 36D AND WILL BE DESTROYED BY .:UNIT:.
#		RMAG1	E:(-29) M:(-27)	PDL 12 NOT TOUCHED.
#		TFFQ1	E:(-16) M:(-15)	PDL 14D
#		TFFDELQ	E:(-16) M:(-15)	PDL 10D
#		PUSHLOC	AT PDL+0
# ERASABLE INITIALIZATION REQUIRED:
#		RONE	E:(-29) M:(-27)	M  STATE VECTOR					LEFT BY USER
#		VONE'	E:(+10) M:(+9)	VN/SQRT(NU)					LEFT BY TFF/CONIC
#		RMAG1	E:(-29) M:(-27)	PRESENT RADIUS, M				LEFT BY TFFCONIC
#		C(MPAC)	E:(-29) M:(-27)	RTERM, TERMINAL RADIUS LENGTH, M		LEFT BY USER
#
#		THE FOLLOWING ARE STORED IN THE PUSH LIST AREA.
#		TFF/RTMU	E:(17) M:(14)	1/SQRT(MU)				LEFT BY TFFCONIC.
#		NRMAG	E: (-29+NR)	M RMAG, NORMALIZED				LEFT BY TFFCONIC
#			M: (-27+NR)
#		X1			  -NR, NORM COUNT				LEFT BY TFFCONIC
#		TFFNP	E: (-38+2NR)	M   LCP, SEMI LATUS RECTUM, WEIGHT NR		LEFT BY TFFCONIC
#			M: (-36+2NR)
#		TFFALFA	 E: (26-NR)	  1/M  ALFA, WEIGHT NR				LEFT BY TFFCONIC
#			 M: (24-NR)
#		TFFRTALF  E:(10+NA)	  SQRT(ALFA), NORMALIZED			LEFT BY TFFCONIC
#			  M:(9+NA)
#		X2			  -NA, NORM COUNT				LEFT BY TFFCONIC
#		TFF1/ALF	E:(-22-2NA)	SIGNED SEMIMAJ AXIS, WEIGHTED BY NA	LEFT BY TFFCONIC
#				M:(-20-2NA)
# DEBRIS:	QPRET, PDL+0 ... PDL+3
#		RTERM	E:(-29) M:(-27)	RTERM, TERMINAL RADIUS LENGTH
#		RAPO	E:(-29) M:(-27)	PDL 16D	(=NRTERM)
#		RPER	E:(-29) M:(-27)	PDL 14D	 (=TFFQ1)

# Page 1382
; ----------------------------------------------------------------------------
; SUBROUTINE: CALCTPER / CALCTFF
; 
; PURPOSE: Calculate time-of-flight from present position to specified radius
;
; COMMENT-ONLY READERS: This routine answers "How long until we reach that
; altitude?" Used throughout Apollo 11 for timing orbital maneuvers, 
; rendezvous planning, and trajectory predictions. Two entry points:
;   - CALCTPER: Time to perigee (closest approach)
;   - CALCTFF:  Time to any specified radius
;
; CODE-ALONG READERS: Implements Kepler's equation solution using eccentric
; anomaly E. Main algorithm:
;   1. Compute Q2 = r·v at terminal radius (using energy equation)
;   2. Calculate ΔQ = Q2 - Q1 (change in r·v from present to terminal)
;   3. For elliptical orbits: Solve Kepler's equation via eccentric anomaly
;   4. For parabolic/hyperbolic: Use appropriate transcendental equations
;   5. Return time in centiseconds scaled at -28
;
; The method handles the "which side of the conic" problem by assuming
; RTERM is on the inbound (descending) side. Historical note: This code
; computed rendezvous times during lunar orbit and translunar trajectory
; time-to-periapsis calculations.
; ----------------------------------------------------------------------------
;
CALCTPER	SETGO			# ENTER WITH RPER IN MPAC
			TFFSW
			+3
CALCTFF		CLEAR			# ENTER WITH RTERM IN MPAC
			TFFSW
; Main computation begins: Calculate Q2 = r·v at terminal radius
; Q2 is computed from the energy equation: Q2^2 = p + r*(2-ALFA*r)
; where p is semi-latus rectum, ALFA = -1/semi-major axis (or -V^2/mu)
; This determines the radial velocity component at the target radius.
;
	+3	STORE	RTERM		# E: (-29) M: (-27)
		SL*
			0,1		# X1=-NR
		STORE	NRTERM		# RTERM  E: (-29+NR) M: (-27+NR)
		DMP	BDSU
			TFFALFA		# ALFA  E:(26-NR) M:(24-NR)
			TFF1/4
		PUSH	DMP		# (2-ALFA RTERM)  (-3)  TO PDL+0
			NRTERM		# E: (-29+NR) M: (-27+NR)
		PDDL	SR*		# RTERM(2-ALFA RTERM) TO PDL+2
					# E: (-32+NR) M: (-30+NR)
			TFFNP		# LC P  E:(-38+2NR) M:(-36+2NR)
			0 -6,1		# X1 = -NR
		DCOMP	DAD		# DUE TO SHIFTS, KEEP PRECISION FOR SQRT
					# RTERM(2-ALFA RTERM) FROM PDL+2
					# E: (-32+NR) M: (-30+NR)
		SR*			# LEAVE  E: (-32) M: (-30)
			0,1		# X1 = -NR
		BOFF	DLOAD		# CHECK TFF / TPER SWITCH
			TFFSW
			+2		# IF TFF, CONTINUE
			TFFZEROS	# IF TPER, SET Q2 = 0
; Take square root to get Q2 = ±sqrt[p + r*(2-ALFA*r)]
; Negative result means no valid free-fall path exists to target radius.
;
	+2	BMN	SQRT		# E: (-16) M: (-15)

			MAXTFF1		# NO FREE FALL CONIC TO RTERM FROM HERE
					# RESET PDL, SET TFF=POSMAX, AND EXIT.

		DCOMP	BOVB		# RT IS ON INBOUND SIDE.  ASSURE OVFIND=0
			TCDANZIG	# ANY PORT IN A STORM.
		STOVL	TFFTEM		# Q2  E: (-16) M: (-15)
; Calculate Q1 = r·v at present position
; Q1 = (r dot v)/sqrt(mu) using current state vector (RONE, VONE')
; This represents the current radial velocity component.
;
			VONE'		# VN/SQRT(MU)  E: (10) M: (9)
		DOT	SL3
			RONE		# SAVED RN.  E: (-29) M: (-27)
		STORE	TFFQ1		# Q1, SAVE FOR GONEPAST TEST.
					# E: (-16) M: (-15)
; Determine trajectory path: outbound (ascending) or inbound (descending)
; If Q1 < 0, spacecraft is on inbound (descending) trajectory leg.
; During Apollo 11 lunar descent, this determined which side of orbit.
;
		BMN	BDSU
			INBOUND		# USE ALTERNATE Z
			TFFTEM		# Q2  E: (-16) M: (-15)

# OUTBOUND Z CALC CONTINUES HERE
;
; For outbound trajectory: Z = (Q2 - Q1) / (2 - ALFA*RMAG - (2 - ALFA*RTERM))
; This Z parameter is key to the Kepler equation solution.
; The denominator can become very small, requiring special handling.
;
		STODL	TFFX		# NUM=Q2-Q1  E: (-16) M: (-15)
			TFFALFA		# ALFA  E: (26-NR) M: (24-NR)
		DMP	BDSU
# Page 1383
			NRMAG		# RMAG  E: (-29+NR) M: (-27+NR)
					# (2-RTERM ALFA)  (-3) FROM PDL+0
SAVEDEN		PUSH	ABS		# DEN TO PDL+0	E: (-3) OR (-16)
					#               M: (-3) OR (-15)
; Test denominator for near-zero condition (indeterminate Z)
; If denominator < 2^-22, we have special cases:
;   - Parabolic orbit (ALFA ≈ 0): Z is indeterminate at perigee
;   - Near-circular orbit: ΔE/2 ≈ 0° or 90°
; Apollo 11 translunar trajectories often approached these edge cases.
;
		DAD	BOV		# INDETERMINANCY TEST
			LIM(-22)	# =1.0-B(-22)
			TFFXTEST	# GO IF DEN >/= B(-22)
		DLOAD	PDDL		# SET DEN=0 OTHERWISE
			TFFZEROS
					# XCH ZERO WITH PDL+0
		DLOAD	DCOMP
			TFFALFA		# ALFA  E: (26-NR) M:( 24-NR)
		BMN	DLOAD		# FOR TPER:  Z INDET AT DELE/2=0 AND 90.
			TFFEL1		# ASSUME 90, AND LEAVE 0 IN PDL: 1/Z=D/N

					# Z INDET. AT PERIGEE FOR PARAB OR HYPERB.
DUMPTFF1	RVQ			# RETURN TFF =0

# INBOUND Z CALC CONTINUES HERE
;
; For inbound trajectory: Use alternate Z formulation to avoid numerical issues
; Z = (RTERM - RMAG) / (Q2 + Q1)
; This form is more stable for descending trajectories.
; Apollo 11 lunar descent used this path during powered descent phases.
;
INBOUND		DLOAD			# RESET PDL+0
		DLOAD	DSU		# ALTERNATE Z CALC
			RTERM		# E: (-29) M: (-27)
			RMAG1		# E: (-29) M: (-27)
		STODL	TFFX		# NUM=RTERM-RN	E: (-29) M: (-27)
			TFFTEM		# Q2  E: (-16)	M: (-15)
		DAD	GOTO
			TFFQ1		# Q1  E: (-16) M: (-15)
			SAVEDEN		# DEN = Q2+Q1  E: (-16) M: (-15)

; Compute Z parameter: Z = (NUM * sqrt(ALFA)) / DEN
; Z relates the change in eccentric anomaly to the geometry of the orbit.
; For ellipses: Z is proportional to sin(ΔE/2)
; For parabolas/hyperbolas: Z has analogous meaning in their equations.
;
TFFXTEST	DAD	PDDL		# (ABS(DEN) TO PDL+2))	E: (-3) OR (-16)
					#			M: (-3) OR (-15)
			DP(-22)		# RESTORE ABS(DEN) TO MPAC
			TFFX		# NUM E:(-16) OR (-29) M:(-15) OR (-27)
		DMP	SR*
			TFFRTALF	# SQRT(ALFA)  E:(10+NA) M:(9+NA)
			0 -3,2		# X2=-NA
		DDV			# C(MPAC) =NUM SQRT(ALFA)	E:(-3) OR (-16)
					#				M:(-3) OR (-15)
					# ABS(DEN) FROM PDL+2	E:(-3) OR (-16)
					#			M:(-3) OR (-15)
; Test for overflow: If Z too large, use alternate equation
; This handles cases where ΔE ≥ 90° or ΔE ≤ -90° (large angle changes).
; Apollo 11 orbital transfers sometimes required these large-angle equations.
;
		DLOAD	BOV		# (THE DLOAD IS SHARED WITH TFFELL)
			TFFX		# NUM  E: (-16) OR (-29)  M:(-15) OR (-27)
			TFFELL		# USE EQN FOR DELE GEQ 90, LEQ -90

# OTHERWISE, CONTINUE FOR GENERAL CONIC FOR TFF EQN
;
; Normal case: Use general conic equation with small-to-moderate angle changes
; Compute final Z and prepare for trigonometric solution.
;
		DDV	STADR
					# DEN FROM PDL+0	E: (-3) OR (-16)
					#			M: (-3) OR (-15)
		STORE	TFFTEM		# Z SAVE FOR SIGN OF SDELF.
# Page 1384
					# E: (-13) M: (-12)
;
; ============================================================================
; General Conic Time-of-Flight Equation (Universal Variable Formulation)
;
; This section implements the Battin-Gauss universal variable equation:
; TFF = Z * [RMAG + RTERM - 2*Z^2*T(X)] * sqrt(mu)
;
; where: X = ALFA * Z^2  (scaled parameter for series expansion)
;        T(X) = polynomial approximation of Stumpff function
;        Z = universal variable parameter (computed earlier)
;
; This equation works for all conic sections (ellipse, parabola, hyperbola)
; and was critical for Apollo 11's translunar and lunar orbit calculations.
; ============================================================================
;
		PUSH	DSQ		# Z TO PDL+0
		PUSH	DMP		# Z SQ TO PDL+2  E: (-26) M: (-24)
			TFFNP		# LC P  E: (-38+2NR) M: (-36+2NR)
		SL	SIGN
			5
			TFFTEM		# AFFIX SIGN FOR SDELF (ENTRY DISPLAY)
		STODL	TFFTEM		# P ZSQ  E: (-59+2NR) M: (-55+2NR)
					# (ARG IS USED IN TFF/TRIG)
					# ZSQ FROM PDL+2  E: (-26) M: (-24)
; Compute X = ALFA * Z^2 for Stumpff function argument
; X determines series convergence for T(X) polynomial approximation.
;
		PUSH	DMP		# RESTORE PUSH LOC
			TFFALFA		# ALFA  E: (26-NR) M: (24-NR)
		SL*
			0,1		# X1=-NR
		STORE	TFFX		# X
; Evaluate T(X) using polynomial approximation (RTB T(X))
; T(X) is a modified Stumpff function appearing in universal variable method.
;
		RTB	DMP
			T(X)		# POLY
					# ZSQ FROM PDL+2  E: (-26) M: (-24)
; Compute: RMAG + RTERM - 2*Z^2*T(X)
; This term represents the geometry of the transfer arc.
;
		SR2	BDSU		# 2 ZSQ T(X)  E: (-29) M: (-27)
			RTERM		# RTERM  E: (-29) M: (-27)
		DAD	DMP
			RMAG1		# E: (-29) M: (-27)
					# Z FROM PDL+0  E: (-13) M: (-12)
; Final multiplication by Z to get TFF*sqrt(mu)
; Result is time-of-flight in units of sqrt(mu).
;
		SR3	BPL		# TFF SQRT(MU)  E: (-45) M: (-42)
			ENDTFF		# (NO PUSH UP)
;
; Test for "gone past" condition: Has spacecraft already passed target radius?
; If Q1 (current radial velocity) has opposite sign from TFF, the spacecraft
; has already passed through the target radius on its current trajectory.
; During Apollo 11 targeting, this prevented attempting impossible maneuvers.
;
		PUSH	SIGN		# TFF SQRT(MU) TO PDL+0
			TFFQ1		# Q1 FOR GONEPAST TEST
		BPL	DLOAD		# GONE PAST ?
			NEGTFF		# YES. TFF < 0 .
			TFF1/ALF	# 1/ALFA  E: (-22-2NA) M: (-20-2NA)
;
; Test orbit type via ALFA sign:
; ALFA > 0: Elliptical orbit (requires period correction)
; ALFA = 0: Parabolic orbit (no period concept)
; ALFA < 0: Hyperbolic orbit (no period concept)
;
		DCOMP	BPL		# ALFA > 0 ?
			NEGTFF		# NO. TFF IS NEGATIVE.

# CORRECT FOR ORBITAL PERIOD.
;
; For elliptical orbits only: Adjust TFF by integer multiples of orbital period
; to ensure we select the correct orbital arc (short way vs. long way around).
; Period = 2π / sqrt(ALFA)
; This correction was critical for Apollo 11 rendezvous targeting calculations.
;
		DCOMP			# YES.  CORRECT FOR ORB PERIOD.
		DMP	DDV
			PI/16		# 2 PI (-5)
			TFFRTALF	# SQRT(ALFA)  E: (10+NA) M: (9+NA)
		SL*	SL*
			0 -4,2		# X2=-NA
			0 -4,2
		SL*	DAD
			0,2
					# TFF SQRT(MU) FROM PDL+0	E:(-45) M:(-42)
;
; Final conversion: Multiply by 1/sqrt(mu) to get TFF in centiseconds.
; During Apollo 11, this produced the actual coast time for trajectory targeting.
; Overflow protection ensures result stays within AGC word size limits.
;
ENDTFF		DMP	BOV		# TFF SQRT(MU) IN MPAC		E:(-45) M:(-42)
			TFF/RTMU	# E: (17) M: (14)
			MAXTFF		# SET POSMAX IN OVFL.

DUMPTFF2	RVQ			# RETURN TFF	(-28) CS IN MPAC.

# Page 1385
;
; NEGTFF: Handle negative time of flight (retrograde orbit segment).
; For certain orbit geometries, the computed TFF can be negative, indicating
; motion in the opposite direction. This simply loads the negative result and
; continues to the final scaling and return sequence.
;
NEGTFF		DLOAD
					# TFF SQRT(MU) FROM PDL+0, NEGATIVE.
		GOTO
			ENDTFF

;
; MAXTFF1/MAXTFF: Overflow protection for time of flight calculations.
; If TFF computation overflows AGC's 15-bit word limits, return NEARONE
; (0.999999999) as a safe maximum value to prevent mission software errors.
; Critical during rendezvous when extremely long transfer times were rejected.
;
MAXTFF1		DLOAD			# RESET PDL
MAXTFF		DLOAD	RVQ
			NEARONE

# TIME OF FLIGHT ELLIPSE WHEN DEL (ECCENTRIC ANOM) GEQ 90 AND LEQ -90.
;
; TFFELL: Elliptical orbit time of flight for large eccentric anomaly changes.
; When the spacecraft travels more than 90 degrees around an ellipse (large arc),
; different computational methods are required to maintain numerical precision.
; Used during Apollo 11's multi-revolution coast phases in Earth and lunar orbit.
;
; Method: Uses polynomial approximation T(X) for the infinite series:
;   1/3 - X/5 + X²/7 - X³/9 + ...
; where X = (1/α)Z² for elliptical orbits, ensuring accurate long-arc solutions.
;
					# NUM FROM TFFX. E: (-16) OR (-29)
					#		 M: (-15) OR (-27)
TFFELL		SL2			# NUM E:(-14) OR (-27)  M:(-13) OR (-25)
		BDDV	PUSH		# TEMP SAVE D/N IN PDL+0
					# DEN FROM PDL+0  E:(-3)/( 16)  M:(-3)/(-15)
					# N/D TO PDL+0  E: (11) M: (10)
TFFEL1		DLOAD	DSU		# (ENTER WITH D/N=0 IN PDL+0)
			TFFTEM		# Q2  E: (-16) M: (-15)
			TFFQ1		# Q1  E: (-16) M: (-15)
		STODL	TFFDELQ		# Q2-Q1  E: (-16) M: (-15)
					# D/N FROM PDL+0
		STADR
		STORE	TFFTEM		# D/N  E: (11) M: (10)
		DMP	SL*
			TFF1/ALF	# 1/ALFA  E: (-22-2NA) M: (-20-2NA)
			0,2		# 1/ALFA Z  E: (-11-NA) M: (-10-NA)
		PUSH	DMP		# TO PDL+0
			TFFTEM		# 1/Z  E: (11) M: (10)
		SL*	BOVB
			0,2		# X2= -NA
			SIGNMPAC	# IN CASE X= 1.0, CONTINUE
		STORE	TFFX		# X=1/ALFA ZSQ
		RTB	DMP
			T(X)		# POLY
			TFFX
		SR3	DSU
			DP2(-3)
		DMP	PUSH		# 2(X T(X)-1) /Z ALFA	E:(-15-NA)
					#			M:(-14-NA)
					# 1/ALFA Z FROM PDL+0	E:(-11-NA)
					#			M:(-10-NA)
		DLOAD	DMP		# GET SIGN FOR SDELF
			TFFTEM		# 1/Z  E: (11) M: (10)
			RMAG1		# E: (-29) M: (-27)
		SL2	DAD
			TFFQ1		# Q1  E: (-16) M: (-15)
		STODL	TFFTEM		# (Q1+R 1/Z) =SGN OF SDELF  E:(-16) M:(-15
			TFFNP		# LC P  E: (-38+2NR) M: (-36+2NR)
		DMP	SL*		# CALC FOR ARG FOR TFF/TRIG.
# Page 1386
			TFF1/ALF	# 1/ALFA  E:(-22-2NA) M:(-20-2NA)
			1,2		# X2=-NA
		SIGN	SL*
			TFFTEM		# AFFIX SIGN FOR SDELF
			0,2
		STODL	TFFTEM		# P/ALFA  E:(-59+2NR)	M:(-55+2NR)
					# (ARG FOR USE IN TFF/TRIG)
			TFF1/ALF	# 1/ALFA E:(-22-2NA) M:(-20-2NA)
		SQRT	DMP
			PI/16		# PI (-4)
		DAD
					# 2(XT(X)-1)/Z ALFA FROM PDL	E:(-15-NA)
					#				M:(-14-NA)
		SL*	DSU
			0 -1,2
			TFFDELQ		# Q2-Q1  E: (-16) M: (-15)
		DMP	SL*
			TFF1/ALF	# 1/ALFA  E:(-22-2NA) M:(-20-2NA)
			0 -3,2
		SL*	GOTO
			0 -4,2
			ENDTFF		# TFF SQRT(MU) IN MPAC E:(-45) M:(-42)

# Page 1387
# PROGRAM NAME:		T(X)				DATE:	01.17.67
# MOD NO:	0					LOG SECTION:	TIME OF FREE FALL
# MOD BY:  	RR BAIRNSFATHER
# FUNCTIONAL DESCRIPTION:	THE POLYNOMIAL T(X) IS USED BY TIME OF FLIGHT SUBROUTINES CALCTFF AND
#	CALCTPER TO APPROXIMATE THE SERIES
#			  2	3
#		1/3 -X/5 +X /7 -X /9 ...

#	WHERE	X = ALFA Z Z		IF ALFA Z Z LEQ	1
#		X = 1/(ALFA Z Z )	IF ALFA Z Z  G	1

#	ALSO	X IS NEG FOR HYPERBOLIC ORBITS
#		X = 0 FOR PARABOLIC ORBITS
#		X IS POSITIVE FOR ELLIPTIC ORBITS
#	FOR FLIGHT 278, THE POLYNOMIAL T(X) IS FITTED OVER THE RANGE (0,+1) AND HAS A MAXIMUM
#	DEVIATION FROM THE SERIES OF 2 E-5	(T(X) IS A CHEBYCHEV TYPE FIT AND WAS OBTAINED USING
#	MAC PROGRAM AUTCURFIT294RRB AND IS VALID TO THE SAME TOLERANCE OVER THE RANGE (-.08,+1). )
# CALLING SEQUENCE:	RTB
#				T(X)
#		C(MPAC) = X

# SUBROUTINE CALLED:	NONE
# NORMAL EXIT MODE:	TC DANZIG
# ALARMS:	NONE
# OUTPUT:	C(MPAC) = T(X)
# ERASABLE INITIALIZATION REQUIRED:
#		C(MPAC) = X
# DEBRIS:	NONE
;
; T(X): Polynomial approximation for Kepler's equation time series.
; Computes infinite series: T(X) = 1/3 - X/5 + X²/7 - X³/9 + X⁴/11 - ...
; Used when sine/cosine methods fail for large eccentric anomaly changes.
;
; The series converges for |X| < 1, providing high-precision time of flight
; when standard trigonometric approaches lose accuracy. AGC uses fixed-point
; arithmetic with careful scaling to maintain precision throughout iteration.
;
; Input: X in MPAC (dimensionless ratio)
; Output: T(X) series value in MPAC for use in Kepler time equation
; Range: Valid from -0.08 to +1.0 with high accuracy
;
T(X)		TC	POLY
		DEC	4		# N-1
		2DEC	3.333333333 E-1
		2DEC*	-1.999819135 E-1*
		2DEC*	1.418148467 E-1*
		2DEC* 	-1.01310997 E-1*
		2DEC*	5.609004986 E-2*
		2DEC*	-1.536156925 E-2*

ENDT(X)		TC	DANZIG

TCDANZIG	=	ENDT(X)

# Page 1388
# TFF CONSTANTS
;
; CONSTANTS: Physical and mathematical constants for time of flight calculations.
; These values define Earth and Moon gravitational parameters, geometric limits,
; and numerical precision boundaries used throughout TFF computations.
;
; Key constants:
;   1/RTMU   - Reciprocal square root of Earth's gravitational parameter (μ)
;   PI/16    - Pi divided by 16 for angular calculations
;   NEARONE  - Maximum time value (0.999999999) to prevent overflow
;   R300K    - 300 kilometer altitude radius for trajectory calculations
;   RPAD1    - Earth surface radius at pad (6373338 meters)
;
; Note: Gravitational parameter values are "adjusted" for computational
; efficiency in near-Earth trajectories, slightly different from standard
; astronomical values to optimize AGC's fixed-point arithmetic precision.
;
		BANK	32

		SETLOC	TOF-FF1
		BANK

#						# NOTE _  NOTE _ ADJUSTED MUE FOR NEAR EARTH TRAJ.

#MUE		=	3.990 815 471 E10 # M CUBE/CS SQ
#RTMUE		=	1.997702549 E5 B-18*	# MODIFIED EARTH MU

1/RTMU		2DEC*	.5005750271 E-5 B17*	# MODIFIED EARTH MU

#						# NOTE _  NOTE _ ADJUSTED MUE FOR NEAR EARTH TRAJ.

#MUM		=	4.902 778 E8		# M CUBE /CS SQ

#RTMUM		2DEC*	2.21422176 E4 B-18*
PI/16		2DEC	3.141592653 B-4
LIM(-22)	2OCT	3777737700		# 1.0 -B(-22)
DP(-22)		2OCT	0000000100		# B(-22)
DP2(-3)		2DEC	1 B-3
DP2(-4)		2DEC	1 B-4			# 1/16

# RPAD1		2DEC	6373338 B-29		# M (-29) =20 909 901.57 FT
RPAD1		=	RPAD
R300K		2DEC	6464778 B-29		# (-29) M
NEARONE		2DEC	.999999999
TFFZEROS	EQUALS	HI6ZEROS
TFF1/4		EQUALS	HIDP1/4
