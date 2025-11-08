# Copyright:	Public domain.
# Filename:	LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	984-987
# Mod history:	2009-05-24 HG	Transcribed from page images.
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
; FILE: LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
; MODULE: Celestial Mechanics and Navigation
; MISSION PHASE: All mission phases (earth-orbit/trans-lunar/lunar-orbit/
;                trans-earth)
;
; TL;DR: Computes accurate position vectors of the Moon and Sun relative to
;        the spacecraft for navigation and trajectory calculations. Uses
;        ephemeris polynomials with time-dependent terms to account for
;        celestial body motion. Essential for cislunar navigation, optical
;        sighting alignment, and trajectory targeting throughout the mission.
;
; COMMENT-ONLY READERS: These computations track where the Moon and Sun are
;        located at any given time during the mission, enabling the spacecraft
;        to navigate accurately between Earth and Moon.
; CODE-ALONG READERS: Study polynomial ephemeris evaluation, coordinate frame
;        transformations via KONMAT matrix, and scaling of celestial mechanics
;        calculations within AGC fixed-point arithmetic constraints.
; ============================================================================

# Page 984
# NAME - LSPOS  - LOCATE SUN AND MOON			DATE - 25 OCT 67
# MOD NO.1
# MOD BY NEVILLE					ASSEMBLY SUNDANCE
#
# FUNCTIONAL DESCRIPTION
#
#        COMPUTES UNIT POSITION VECTOR OF THE SUN AND MOON IN THE BASIC REFERENCE SYSTEM. THE SUN VECTOR S IS
# LOCATED VIA TWO ANGLES. THE FIRST ANGLE(OBLIQUITY) IS THE ANGLE BETWEEN THE EARTH EQUATOR AND THE ECLIPTIC. THE
# SECOND ANGLE IS THE LONGITUDE OF THE SUN MEASURED IN THE ECLIPTIC.
# THE POSITION VECTOR OF THE SUN IS
#	 -
#	 S=(COS(LOS), COS(OBL)*SIN(LOS), SIN(OBL)*SIN(LOS)), WHERE
#
#      LOS=LOS +LOS *T-(C *SIN(2PI*T)/365.24 +C *COS(2PI*T)/365.24)
#	      0    R     0                     1
#      LOS  (RAD) IS THE LONGITUDE OF THE SUN FOR MIGNIGHT JUNE 30TH OF THE PARTICULAR YEAR.
#         0
#      LOS  (RAD/DAY) IS THE MEAN RATE FOR THE PARTICULAR YEAR.
#    	  R
# LOS  AND LOS  ARE STORED AS LOSO AND LOSR IN RATESP.
#    0        R
# COS(OBL) AND SIN(OBL) ARE STORED IN THE MATRIX KONMAT.
# T, TIME MEASURED IN DAYS(24 HOURS), IS STORED IN TIMEP.
# C  AND C  ARE FUDGE FACTORS TO MINIMIZE THE DEVIATION. THEY ARE STORED AS ONE CONSTANT(CMOD), SINCE
#  0      1                               2  2 1/2
# C *SIN(X)+C *COS(X) CAN BE WRITTEN AS (C +C )   *SIN(X+PHI), WHERE PHI=ARCTAN(C /C ).
#  0         1                            0  1                                   1  0
#
# THE MOON IS LOCATED VIA FOUR ANGLES. THE FIRST IS THE OBLIQUITY. THE SECOND IS THE MEAN LONGITUDE OF THE MOON,
# MEASURED IN THE ECLIPTIC FROM THE MEAN EQUINOX TO THE MEAN ASCENDING NODE OF THE LUNAR ORBIT, AND THEN ALONG THE
# ORBIT. THE THIRD ANGLE IS THE ANGLE BETWEEN THE ECLIPTIC AND THE LUNAR ORBIT. THE FOURTH ANGLE IS THE LONGITUDE
# OF THE NODE OF THE MOON, MEASURED IN THE LUNAR ORBIT. LET THESE ANGLES BE OBL,LOM,IM, AND LON RESPECTIVELY.
#
# THE SIMPLIFIED POSITION VECTOR OF THE MOON IS
#   -
#   M=(COS(LOM), COS(OBL)*SIN(LOM)-SIN(OBL)*SIN(IM)*SIN(LOM-LON), SIN(OBL)*SIN(LOM)+COS(OBL)*SIN(IM)*SIN(LOM-LON))
#
#   WHERE
#      LOM=LOM +LOM *T-(A *SIN(2PI*T/27.5545)+A *COS(2PI*T/27.5545)+B *SIN(2PI*T/32)+B *COS(2PI*T/32)), AND
#	      0    R     0                     1                     0                1
#      LON=LON +LON
#	      0    R
# A , A , B  AND B  ARE STORED AS AMOD AND BMOD (SEE DESCRIPTION OF CMOD, ABOVE).  COS(OBL), SIN(OBL)*SIN(IM),
#  0   1   0      1
# SIN(OBL), AND COS(OBL)*SIN(IM) ARE STORED IN KONMAT AS K1, K2, K3 AND K4, RESPECTIVELY. LOM , LOM , LON , LON
# ARE STORED AS LOMO, LOMR, LONO, AND LONR IN RATESP.                                        0     R     0     R
# THE THREE PHIS ARE STORED AS AARG, BARG, AND CARG(SUN).  ALL CONSTANTS ARE UPDATED BY YEAR.
#
# CALLING SEQUENCE
# Page 985
#   CALL LSPOS.  RETURN IS VIA QPRET.
# ALARMS OR ABORTS
#   NONE
# ERASABLE INITIALIZATION REQUIRED
#   TEPHEM - TIME FROM MIGNIGHT 1 JULY PRECEDING THE LAUNCH TO THE TIME OF THE LAUNCH (WHEN THE AGC CLOCK WENT
# TO ZERO). TEPHEM IS TP WITH UNITS OF CENTI-SECONDS.
#   TIME2 AND TIME1 ARE IN MPAC AND MPAC +1 WHEN PROGRAM IS CALLED.
# OUTPUT
#   UNIT POSITIONAL VECTOR OF SUN IN VSUN.   (SCALED B-1)
#   UNIT POSITIONAL VECTOR OF MOON IN VMOON. (SCALED B-1)
# SUBROUTINES USED
#   NONE
# DEBRIS
#   CURRENT CORE SET,WORK AREA AND FREEFLAG
;
; ============================================================================
; CELESTIAL BODY EPHEMERIS COMPUTATION
;
; The AGC must continuously track the positions of the Sun and Moon to
; support navigation, optical alignment, and trajectory planning throughout
; the mission. This subroutine evaluates mathematical models (ephemerides)
; that predict celestial body positions based on orbital mechanics.
;
; During Apollo 11's journey from July 16-24, 1969, these calculations
; enabled the crew to perform star sightings for navigation updates and
; allowed trajectory programs to compute accurate maneuvers relative to
; Earth and Moon gravitational influences.
; ============================================================================
		BANK	04
		SETLOC	EPHEM
		BANK
		EBANK=	VSUN
		COUNT*	$$/EPHEM
;
; ============================================================================
; LSPOS - PRIMARY ENTRY POINT FOR LUNAR AND SOLAR POSITION COMPUTATION
;
; This subroutine is called by navigation and guidance programs whenever
; current Moon or Sun positions are needed. The spacecraft's mission time
; is used to evaluate time-dependent ephemeris polynomials that track
; orbital motion of celestial bodies.
;
; Input: TIME2 and TIME1 in MPAC contain current mission time
; Output: VSUN (Sun position vector), VMOON (Moon position vector)
; Both output vectors are unit vectors (scaled B-1, magnitude = 1.0)
; ============================================================================
;
LUNPOS		EQUALS	LSPOS
;
; Entry point LSPOS begins time computation.
; Mission elapsed time (stored in TIME2/TIME1) must be converted to days
; since the ephemeris epoch (midnight July 1 preceding launch).
;
LSPOS		SETPD	SR
			0
			14D		# TP
		TAD	DDV
## Comments in [...] are hand-written notations in original listing
			TEPHEM		# TIME OF LAUNCH [in centisec B 42]
			CSTODAY		# 24 HOURS-8640000 CENTI-SECS/DAY B-33
		STORE	TIMEP		# T IN DAYS [@ B 9 = 512 days]
;
; Time computation complete. TIMEP now contains mission time in days
; since midnight July 1, 1969. This serves as the independent variable
; for all ephemeris polynomial evaluations.
;
; The AGC converts centiseconds (TIME2/TIME1) to days by adding the
; launch offset (TEPHEM) and dividing by 8.64 million centiseconds/day.
; Scaling: Result stored at B9 provides 512-day range with ~0.16 second
; granularity, sufficient for cislunar navigation accuracy requirements.
;
		AXT,1	AXT,2		#	    [∴ granularity ≈ 0.164 sec]
			0
			0
;
; Initialize index registers for polynomial evaluation loop.
; X1 indexes through ephemeris coefficients (AMOD, BMOD, CMOD).
; X2 indexes through output storage (Moon longitude, Sun longitude, Node).
; FREEFLAG controls iteration through multiple polynomial terms.
;
		CLEAR
			FREEFLAG	# SWITCH BIT
;
; ============================================================================
; EPHEMERIS POLYNOMIAL CORRECTION TERMS
;
; This loop evaluates periodic correction terms that account for
; perturbations in lunar and solar motion. The Moon's orbit around Earth
; (period ~27.5545 days) and annual variations in Earth's orbit around
; the Sun (period 365.24 days) require sinusoidal corrections to the
; mean motion polynomials.
;
; For Moon: Corrections for 27.5545-day and 32-day periodicities (AMOD, BMOD)
; For Sun: Correction for 365.24-day periodicity (CMOD)
; Each correction has form: Amplitude * SIN(2π*T/Period + PhaseAngle)
; ============================================================================
;
POSITA		DLOAD
			KONMAT +2	# ZERO$
		STORE	GTMP
;
; POSITB loop: Compute sinusoidal correction terms
; Evaluates: Amplitude * SIN((T / Period) + PhaseAngle)
; where T is mission time in days, stored in TIMEP.
;
POSITB		DLOAD	DMP*
			TIMEP		# T
			VAL67 +4,1	# 1/27 OR 1/32 OR 1/365
# Page 986
		SL	DAD*
			8D
			VAL67 +2,1	# AARG
;
; Compute sine of argument: SIN(2π*T/Period + Phase)
; The computed angle determines where Moon/Sun is in its periodic cycle.
; During Apollo 11's 8-day mission (July 16-24), the Moon completed
; approximately 0.29 orbital periods around Earth.
;
		SIN	DMP*		# SIN(T/27+PHI) OR T/32 OR T/365
			VAL67,1		# (A0**2+A1**2)**1/2SIN(X+PHIA)
		DAD	INCR,1		# PLUS
			GTMP		# (B0**2+B1**2)**1/2SIN(X+PHIB)
		DEC	-6
		STORE	GTMP		# OR (C0**2+C1**2)**1/2SIN(X+PHIC)
;
; Accumulate correction term in GTMP. FREEFLAG controls whether to
; loop again for additional correction terms (Moon has 2, Sun has 1).
;
		BOFSET
			FREEFLAG
			POSITB
;
; POSITD: Complete longitude calculation
; Computes: L = L₀ + Rate*T - PeriodicCorrections
; where L₀ is initial longitude at epoch, Rate is mean daily motion
;
POSITD		DLOAD	DMP*
			TIMEP		# T
			RATESP,2	# LOMR,LOSR,LONR
		SL	DAD*
			5D
			RATESP +6,2	# LOMO,LOSO,LONO
;
; Subtract accumulated periodic corrections (stored in GTMP) to obtain
; true celestial longitude at current mission time.
; - For Moon: True mean longitude in ecliptic coordinates
; - For Sun: Ecliptic longitude of Sun
; - For Node: Longitude of lunar ascending node
;
		DSU
			GTMP
		STORE	STMP,2		# LOM,LOS,LON
;
; Loop control: Increment X2 index and determine if more bodies to process.
; Sequence: 1st iteration = Moon longitude (LOM)
;           2nd iteration = Sun longitude (LOS)  
;           3rd iteration = Node longitude (LON)
;
		SLOAD	INCR,2
			X2
		DEC	-2
		DAD	BZE
			RCB-13		# PLUS 2
			POSITE		# 2ND
		BPL
			POSITA		# 1ST
;
; ============================================================================
; MOON POSITION VECTOR COMPUTATION (3rd iteration)
;
; Computes Moon position vector in Basic Reference Coordinate System using
; spherical coordinates transformed through obliquity and inclination.
; The simplified lunar position equation accounts for:
; - Moon's mean longitude (LOM) in ecliptic
; - Obliquity of ecliptic relative to Earth equator
; - Inclination of lunar orbit (≈5.15°) relative to ecliptic
; - Longitude of ascending node (LON)
; ============================================================================
;
POSITF		DLOAD	DSU		# 3RD
			STMP		# LOM
			STMP +4		# LON
;
; Compute (LOM - LON): argument for longitude of Moon measured from its
; ascending node. This angle is needed because lunar orbit is inclined
; ~5.15° to the ecliptic.
;
		SIN	PDDL		# SIN(LOM-LON)
			STMP
		SIN	PDDL		# SIN LOM
			STMP
		COS	VDEF		# COS LOM
;
; Build 3-component vector (COS LOM, SIN LOM, SIN(LOM-LON))
; Transform through KONMAT to account for ecliptic obliquity and lunar
; orbit inclination. KONMAT contains pre-computed trigonometric products:
; K1 = COS(obliquity), K2 = SIN(obliquity)*SIN(inclination)
; K3 = SIN(obliquity), K4 = COS(obliquity)*SIN(inclination)
;
		MXV	UNIT
			KONMAT		# K1,K2,K3,K4,
;
; Normalize result to unit vector (magnitude = 1.0, scaled B-1).
; VMOON now contains Moon position unit vector in spacecraft reference frame.
; This vector points from spacecraft toward the Moon's center.
;
		STORE	VMOON
;
; ============================================================================
; SUN POSITION VECTOR COMPUTATION
;
; Computes Sun position vector in Basic Reference Coordinate System.
; The Sun's position is simpler than the Moon's because the Sun lies in
; the ecliptic plane (by definition). Only obliquity transformation needed.
; Sun position equation: S = (COS LOS, COS(OBL)*SIN LOS, SIN(OBL)*SIN LOS)
; where LOS is ecliptic longitude of Sun, OBL is obliquity of ecliptic.
;
; During Apollo 11 (July 16-24, 1969), the Sun was in Cancer constellation,
; moving approximately 1° per day along the ecliptic toward Leo.
; ============================================================================
;
		DLOAD	PDDL
			KONMAT +2	# ZERO
			STMP +2
;
; Load Sun's ecliptic longitude (LOS) computed in earlier iteration.
; STMP +2 contains LOS from POSITD calculation.
;
		SIN	PDDL		# SIN LOS
			STMP +2
		COS	VDEF		# COS LOS
;
; Build 3-component vector (COS LOS, SIN LOS, 0)
; The zero component is because Sun lies in ecliptic plane.
; Transform through KONMAT to rotate from ecliptic to equatorial frame.
; KONMAT rotation accounts for Earth's 23.45° axial tilt (obliquity).
;
		MXV	UNIT
			KONMAT
;
; Normalize to unit vector (magnitude = 1.0, scaled B-1).
; VSUN now points from spacecraft toward the Sun's center.
; Used for solar pressure computations, thermal analysis, and 
; optical navigation star sightings that must avoid solar glare.
;
		STORE	VSUN
;
; Return to calling routine via QPRET (Q register contains return address).
; Both VMOON and VSUN are now updated with current mission time positions.
;
		RVQ
# Page 987
;
; ============================================================================
; POSITE - Re-entry Point for Second Celestial Body Calculation
;
; This routine resets the accumulation register and branches back to POSITD
; to compute the second celestial body position (Sun after Moon computed).
; ============================================================================
;
POSITE		DLOAD
			KONMAT +2	# ZEROS
;
; Load zero value to reset GTMP accumulator for Sun longitude computation.
; GTMP accumulates periodic correction terms - must start at zero for new body.
;
		STORE	GTMP
;
; Branch back to POSITD to compute Sun's ecliptic longitude.
; The loop structure allows same longitude calculation code to serve both
; Moon (first pass) and Sun (second pass after POSITE re-entry).
;
		GOTO
			POSITD
;
; ============================================================================
; LUNVEL - Lunar Velocity Stub for Integration Compatibility
;
; Placeholder routine that returns immediately without computing lunar velocity.
; Orbital integration routines may call this, but velocity computation is
; handled elsewhere in the navigation system. This stub prevents program abort
; if integration attempts to invoke lunar velocity computation.
; ============================================================================
;
LUNVEL		RVQ                     #         TO FOOL INTEGRATION
		SETLOC	EPHEM1
		BANK

		COUNT*	$$/EPHEM
;
; ============================================================================
; MEMORY LOCATION DEFINITIONS FOR EPHEMERIS COMPUTATION
;
; These EQUALS statements define symbolic names for memory addresses used
; during celestial body position calculations. Using symbolic names improves
; code readability and allows assembler to manage actual address allocation.
; ============================================================================
;
; STMP (address 16D octal) - Temporary storage for ecliptic longitude angle
;    Holds intermediate values during Moon/Sun longitude computation.
;    Scaled as revolutions (1.0 = 360 degrees = 2*PI radians).
;
STMP		EQUALS	16D
;
; GTMP (address 22D octal) - Accumulator for periodic correction terms
;    Accumulates sine/cosine correction terms in POSITA/POSITB loop.
;    Reset to zero via POSITE before computing second celestial body.
;    Scaled to match longitude scaling (revolutions).
;
GTMP		EQUALS	22D
;
; TIMEP (address 24D octal) - Mission time in days
;    Converted from AGC centiseconds to days (24-hour periods).
;    Used as independent variable for all ephemeris polynomial evaluations.
;    Time zero reference is midnight July 1 preceding launch date.
;
TIMEP		EQUALS	24D

# *** END OF LEMP50S .115 ***
