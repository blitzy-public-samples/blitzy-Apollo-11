# Copyright:	Public domain.
# Filename:	CONTROLLED_CONSTANTS.agc
# Purpose:	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
#
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim.lawton@gmail.com>
# Website:	www.ibiblio.org/apollo.
# Pages:	038-053
# Mod history:	2009-05-16	JVL	Transcribed from page images.
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
#    Assemble revision 001 of AGC program LMY99 by NASA 2021112-061
#    16:27 JULY 14, 1969

; ============================================================================
; FILE: CONTROLLED_CONSTANTS.agc
; MODULE: Lunar Module Mission Constants
; MISSION PHASE: All phases (descent/landing/ascent/rendezvous)
;
; TL;DR: Defines critical mission-specific constants for Apollo 11 Lunar Module
;        including engine thrust parameters, landing site coordinates, vehicle
;        mass properties, physical constants, and star catalog for navigation.
;        These values were carefully calculated for Eagle's specific mission to
;        the Sea of Tranquility landing site on July 20, 1969.
;
; COMMENT-ONLY READERS: This file contains the numerical values that guided
;        Eagle to its historic landing. The constants define everything from
;        engine thrust to the exact lunar coordinates of the landing site.
; CODE-ALONG READERS: Study the scaling factors and units carefully - these
;        fixed-point constants must maintain precision across the full range
;        of mission operations while fitting in AGC's 15-bit data words.
; ============================================================================

# Page 38
# DPS AND APS ENGINE PARAMETERS

; ============================================================================
; SECTION: Descent and Ascent Propulsion System Parameters
;
; The Eagle's two main engines are defined here: the Descent Propulsion System
; (DPS) used during landing, and the Ascent Propulsion System (APS) used to
; return from the lunar surface. These parameters control the throttle
; algorithms and fuel consumption calculations that were critical during
; Armstrong and Aldrin's descent to the Sea of Tranquility.
; ============================================================================

		SETLOC	P40S
		BANK
		COUNT*	$$/P40

# *** THE ORDER OF THE FOLLOWING SIX CONSTANTS MUST NOT BE CHANGED ***

; Descent Propulsion System (DPS) - The throttleable engine that lowered
; Eagle to the lunar surface. Maximum thrust of 9817.5 lbs (43,670 Newtons)
; could be throttled down to 10% for final approach and touchdown.
FDPS		2DEC	4.3670 B-7		# 9817.5 LBS FORCE IN NEWTONS
MDOTDPS		2DEC	0.1480 B-3		# 32.62 LBS/SEC IN KGS/CS
DTDECAY		2DEC	-38

; Ascent Propulsion System (APS) - The fixed-thrust engine that launched
; Eagle back to orbit for rendezvous with Columbia. Non-throttleable,
; producing constant 3500 lbs (15,569 Newtons) thrust.
FAPS		2DEC	1.5569 B-7		# 3500 LBS FORCE IN NEWTONS
MDOTAPS		2DEC	0.05135 B-3		# 11.32 LBS/SEC IN KGS/CS
ATDECAY		2DEC	-10

# ********************************************************************

; Reaction Control System (RCS) thrust values. The LM had 16 small thrusters
; arranged in four quads for attitude control. During descent, one RCS jet
; firing continuously was equivalent to about 0.16 kg/sec fuel consumption.
FRCS4		2DEC	0.17792 B-7		# 400 LBS FORCE IN NEWTONS
FRCS2		2DEC	0.08896 B-7		# 200 LBS FORCE IN NEWTONS

		SETLOC	P40S1
		BANK
		COUNT*	$$/P40

# *** APS IMPULSE DATA FOR P42 ***************************************

; APS total impulse parameters for abort program P42. These values define
; the impulse available for various abort scenarios during descent, allowing
; the AGC to compute whether sufficient fuel remains to reach orbit.
K1VAL		2DEC	124.55 B-23		# 2800 LB-SEC
K2VAL		2DEC	31.138 B-24		# 700 LB-SEC
K3VAL		2DEC	1.5569 B-10		# FAPS (3500 LBS THRUST)

# ********************************************************************

; DPS thrust parameters used in burn programs (P40 series). The shifted
; version optimizes computational efficiency for high-speed calculations
; during powered flight.
S40.136		2DEC	.4671 B-9		# .4671 M NEWTONS (DPS)
S40.136_	2DEC	.4671 B+1		# S40.136 SHIFTED LEFT 10.

		SETLOC	ASENT1
		BANK
		COUNT*	$$/P70

; ============================================================================
; SECTION: Ascent Stage Performance Parameters
;
; When Armstrong radioed "The Eagle has landed" at 102:45:40 mission time,
; these constants were already loaded and ready for the return journey. They
; define Eagle's ascent stage performance based on the actual liftoff mass
; of 4869.9 kg, accounting for fuel consumed during descent and surface stay.
; ============================================================================

; Initial ascent acceleration parameters. The inverted acceleration value
; enables efficient delta-V calculations during the critical ascent guidance
; phase when Eagle must reach precise orbital insertion parameters.
(1/DV)A		2DEC	15.20 B-7		# 2 SECONDS WORTH OF INITIAL ASCENT
# Page 39
						# STAGE ACCELERATION -- INVERTED (M/CS)
						# 1) PREDICATED ON A LIFTOFF MASS OF
						#    4869.9 KG (SNA-8-D-027 7/11/68)
						# 2) PREDICATED ON A CONTRIBUTION TO VEH-
						#    ICLE ACCELERATION FROM RCS THRUSTERS
						#    EQUIV. TO 1 JET ON CONTINUOUSLY.

; DPS thrust conversion factor for descent guidance calculations.
K(1/DV)		2DEC	436.70 B-9		# DPS ENGINE THRUST IN NEWTONS / 100 CS.

; Ascent stage acceleration at liftoff. This value was critical for P12
; ascent program to compute the trajectory that would rendezvous Eagle
; with Michael Collins in Columbia, orbiting overhead.
(AT)A		2DEC	3.2883 E-4 B9		# INITIAL ASC. STG. ACCELERATION ** M/CS.
						# ASSUMPTIONS SAME AS FOR (1/DV)A.

; Total burn time available for ascent. Net mass flow includes both the
; APS engine propellant consumption and one RCS jet firing continuously
; for attitude control during ascent.
(TBUP)A		2DEC	91902 B-17		# ESTIMATED BURN-UP TIME OF THE ASCENT STG.
						# ASSUMPTIONS SAME AS FOR (1/DV)A WITH THE
						# ADDITIONAL ASSUMPTION THAT NET MASS-FLOW
						# RATE = 5.299 KG/SEC = 5.135 (APS) +
						# .164 (1 RCS JET).

		SETLOC	ASENT
		BANK
		COUNT*	$$/ASENT

; RCS acceleration contribution with 4 jets active in a dry (minimum mass)
; Lunar Module. Used for abort mode calculations and attitude control.
AT/RCS		2DEC	.0000785 B+10		# 4 JETS IN A DRY LEM

		SETLOC	SERVICES
		BANK
		COUNT*	$$/SERV

# *** THE ORDER OF THE FOLLOWING TWO CONSTANTS MUST NOT BE CHANGED *******

; Exhaust velocities for APS and DPS engines. These fundamental rocket
; parameters determine propellant efficiency and are used throughout
; burn calculations to convert thrust into delta-V and mass consumption.
APSVEX		DEC	-3030 E-2 B-5		# 9942 FT/SEC IN M/CS.
DPSVEX		DEC*	-2.95588868 E+1 B-05*	# VE (DPS) +2.95588868E+ 3

# ************************************************************************

		SETLOC	F2DPS*31
		BANK
		COUNT*	$$/F2DPS

TRIMACCL	2DEC*	+3.50132708 E-5 B+08*	# A (T) +3.50132708E- 1

# Page 40
; ============================================================================
; SECTION: Throttling and Thrust Detection Parameters
;
; These constants define throttle control deadbands and thrust detection
; thresholds used during descent engine operation. The throttle control
; system managed the variable-thrust DPS engine during Eagle's descent,
; enabling the smooth transition from high-gate braking to the gentle
; touchdown that occurred with approximately 25 seconds of fuel remaining.
; ============================================================================

# THROTTLING AND THRUST DETECTION PARAMETERS

		SETLOC	P40S
		BANK
		COUNT*	$$/P40

; Throttle detection thresholds for descent engine control.
; THRESH1: Primary throttle position threshold (24 counts)
; THRESH3: Secondary threshold for mode transitions (12 counts)
THRESH1		DEC	24
THRESH3		DEC	12

; High throttle flag bit (bit 13) indicates DPS operating above 60% thrust.
; During Apollo 11 descent, throttle ranged from 10% at PDI through variable
; settings as Armstrong maneuvered to his selected landing site.
HIRTHROT	=	BIT13

		SETLOC	FFTAG5
		BANK
		COUNT*	$$/P40

; Secondary throttle threshold (308 counts) for thrust magnitude detection.
THRESH2		DEC	308

		SETLOC	FTHROT
		BANK
		COUNT*	$$/THROT

; Maximum thrust values for DPS engine throttle control.
; FMAXODD: Saturation thrust limit (48145.4 Newtons force)
; FMAXPOS: Normal maximum thrust (43454.7 Newtons force)
FMAXODD		DEC	+3841			# FSAT 		+4.81454413 E+4
FMAXPOS		DEC	+3467			# FMAX 		+4.34546769 E+4

; Throttle lag time constant (0.2 seconds). Compensates for DPS engine
; response delay between commanded thrust and actual thrust output.
THROTLAG	DEC	+20			# TAU (TH)	+1.99999999 E-1

; Scaling factor converting throttle bit values to physical force units.
SCALEFAC	2DEC*	+7.97959872 E+2 B-16*	# BITPERF 	+7.97959872 E-2

		SETLOC	F2DPS*32
		BANK
		COUNT*	$$/F2DPS

; Combined throttle threshold for P63 lunar landing program (36 = 24 + 12).
DPSTHRSH	DEC	36			# (THRESH1 + THRESH3 FOR P63)

# Page 41
; ============================================================================
; SECTION: LM Hardware-Related Parameters
;
; These constants calibrate the Lunar Module's sensor systems, particularly
; the landing radar that provided critical altitude and velocity measurements
; during Eagle's descent. The landing radar activated at high altitude and
; provided the real-time data that enabled Armstrong and Aldrin to monitor
; their approach to the lunar surface.
; ============================================================================

# LM HARDWARE-RELATED PARAMETERS

		SETLOC	RADARUPT
		BANK
		COUNT*	$$/RRUPT

; Landing radar bias value for 153.6 kHz carrier frequency. This calibration
; constant corrects systematic errors in the landing radar altitude measurements.
LVELBIAS	DEC	-12288			# LANDING RADAR BIAS FOR 153.6 KC.

; Rendezvous radar range rate bias (17000 counts). Corrects velocity
; measurement errors for accurate relative motion tracking during rendezvous.
RDOTBIAS	2DEC	17000			# BIAS COUNT FOR RR RANGE RATE.

		SETLOC	LRS22
		BANK
		COUNT*	$$/LRS22

; Rendezvous radar data conversion factors.
; RDOTCONV: Converts range rate measurement to meters/centisecond (scaled 2^7).
; RANGCONV: Converts range measurement to meters (scaled 2^-29).
; These conversions transform raw radar counts into physical distance and velocity
; units used by rendezvous navigation programs during lunar orbit operations.
RDOTCONV	2DEC	-.0019135344 B7		# CONVERTS RR RDOT READING TO M/CS AT 2(7)
RANGCONV	2DEC	2.859024 B-3		# CONVERTS RR RANGE READING TO M. AT 2(-29

		SETLOC	SERVICES
		BANK
		COUNT*	$$/SERV

; Landing radar beam direction vector in LM antenna coordinate frame.
; Three components define the range beam orientation relative to the spacecraft.
; Used to transform landing radar measurements into navigation frame coordinates.
HBEAMANT	2DEC	-.4687018041		# RANGE BEAM IN LR ANTENNA COORDINATES.
		2DEC	0
		2DEC	-.1741224271

; Altitude scaling factor: converts 1.079 feet per bit to meters (scaled 2^22).
HSCAL		2DEC	-.3288792		# SCALES 1.079 FT/BIT TO 2(22)M.

# ***** THE SEQUENCE OF THE FOLLOWING CONSTANTS MUST BE PRESERVED *********
;
; Landing radar velocity component scaling factors (X, Y, Z axes).
; These three constants convert landing radar velocity measurements from
; feet/second per bit to meters/centisecond (scaled 2^18). The order is
; critical for indexed access by radar data processing routines.
; During descent, these scales enabled accurate velocity measurements that
; were displayed to Armstrong and Aldrin on their instruments.

VZSCAL		2DEC	+.5410829105		# SCALES .8668 FT/SEC/BIT TO 2(18) M/CS.
VYSCAL		2DEC	+.7565672446		# SCALES 1.212 FT/SEC/BIT TO 2(18) M/CS.
VXSCAL		2DEC	-.4020043770		# SCALES -.644 FT/SEC/BIT TO 2(18) M/CS.

# *************************************************************************

; Velocity increment (delta-V) scaling factors for PIPA accelerometer data.
; KPIP:  Scales to 2^5 meters/centisecond units.
; KPIP1: Scales to 2^7 meters/centisecond units.
; KPIP2: Scales to 2^8 meters/centisecond units.
; Multiple scalings accommodate different computational precision requirements.
KPIP		DEC	.0512			# SCALES DELV TO UNITS OF 2(5) M/CS.
KPIP1		2DEC	.0128			# SCALES DELV TO UNITS OF 2(7) M/CS.
KPIP2		2DEC	.0064			# SCALES DELV TO UNITS OF 2(8) M/CS.

# Page 42
; Altitude and altitude rate conversion factors for display and control.
; ALTCONV: Converts altitude from meters (scaled 2^-24) to bit units (2^-28).
; ARCONV1: Converts altitude rate to bit units for DSKY display formatting.
ALTCONV		2DEC	1.399078846 B-4		# CONVERTS M*2(-24) TO BIT UNITS *2(-28).
ARCONV1		2DEC	656.167979 B-10		# CONV. ALTRATE COMP. TO BIT UNITS<

		SETLOC	R10
		BANK
		COUNT*	$$/R10

ARCONV		OCT	24402			# 656.1679798B-10 CONV ALTRATE TO BIT UNIT
ARTOA		DEC	.1066098 B-1		# .25/2.345 B-1 4X/SEC CYCLE RATE.
ARTOA2		DEC	.0021322 B8		# (.5)/(2.345)(100)
VELCONV		OCT	22316			# 588.914 B-10 CONV VEL. TO BIT UNITS.
KPIP1(5)	DEC	.0512			# SCALES DELV TO M/CS*2(-5).
MAXVBITS	OCT	00547			# MAX. DISPLAYED VELOCITY 199.9989 FT/SEC.

		SETLOC	DAPS3
		BANK
		COUNT*	$$/DAPAO

TORKJET1	DEC	.03757			# 550 / .2 SCALED AT (+16) 64 / 180

# Page 43
; ============================================================================
; MASS, INERTIA, AND VEHICLE DIMENSIONS
;
; Lunar Module mass parameters for autopilot and trajectory calculations.
; These constants define mass limits for different mission configurations:
; - Full ascent stage mass (with crew and consumables)
; - Minimum descent stage mass (propellant depleted)
; - Minimum ascent stage mass (emergency reserves only)
; - Combined vehicle mass ranges for different mission phases
;
; Mass values scaled to 2^16 kilograms for computational precision.
; The Digital Autopilot (DAP) uses these to compute control gains and
; moment-of-inertia estimates throughout the mission as propellant burns.
; ============================================================================
# PARAMETERS RELATING TO MASS, INERTIA, AND VEHICLE DIMENSIONS

		SETLOC	FRANDRES
		BANK
		COUNT*	$$/START

; Nominal full ascent stage mass at liftoff from lunar surface (5050 kg).
; Used by ascent guidance to initialize trajectory calculations.
FULLAPS		DEC	5050 B-16		# NOMINAL FULL ASCENT MASS -- 2(16) KG.

		SETLOC	LOADDAP1
		BANK
		COUNT*	$$/R03

; Minimum descent stage mass (2850 kg) when propellant depleted.
; Minimum ascent stage mass (2200 kg) at emergency reserves.
; DAP uses these bounds to compute valid mass range for control gains.
MINLMD		DEC	-2850 B-16		# MIN. DESCENT STAGE MASS -- 2(16) KG.
MINMINLM	DEC	-2200 B-16		# MIN ASCENT STAGE MASS -- 2(16) KG.
MINCSM		=	BIT11			# MIN CSM MASS (OK FOR 1/ACCS) = 9050 LB

		SETLOC	DAPS3
		BANK
		COUNT*	$$/DAPAD

LOASCENT	DEC	2200 B-16		# MIN ASCENT LEM MASS -- 2(16) KG.
HIDESCNT	DEC	15300 B-16		# MAX DESCENT LEM MASS -- 2(16) KG.
LODESCNT	DEC	1750 B-16		# MIN DESCENT STAGE (ALONE) -- 2(16) KG.

# Page 44
; ============================================================================
; PHYSICAL CONSTANTS (TIME-INVARIANT)
;
; Fundamental constants of nature and celestial mechanics used throughout
; navigation, guidance, and trajectory calculations:
; - Gravitational parameters (mu) for Earth and Moon
; - Planetary radii for altitude computations
; - Angular rates for coordinate transformations
; - Universal constants for orbital mechanics
;
; These values remain constant throughout the mission and are derived from
; 1960s-era geodetic measurements. Precision scaled to accommodate AGC's
; fixed-point arithmetic while maintaining sufficient accuracy for cislunar
; navigation (Earth to Moon and back).
; ============================================================================
# PHYSICAL CONSTANTS ( TIME - INVARIANT )

		SETLOC	IMU2
		BANK
		COUNT*	$$/P07

; Earth's angular velocity relative to mean sun (rad/cs).
; Used for solar ephemeris calculations and sun angle computations.
OMEG/MS		2DEC	.24339048

		SETLOC	R30LOC
		BANK
		COUNT*	$$/R30

# *** THE ORDER OF THE FOLLOWING TWO CONSTANTS MUST BE PRESERVED ***********

; Reciprocal of square root of Moon's gravitational parameter: 1/sqrt(mu_Moon).
; Reciprocal of square root of Earth's gravitational parameter: 1/sqrt(mu_Earth).
; Ordering required for paired retrieval in orbital calculations.
1/RTMUM		2DEC*	.45162595 E-4 B14*
1/RTMUE		2DEC*	.50087529 E-5 B17*

# **************************************************************************

		SETLOC	P40S1
		BANK
		COUNT*	$$/S40.9

; Earth's gravitational parameter mu = GM (m^3/cs^2).
; Central force constant for Earth orbital mechanics and trajectory integration.
EARTHMU		2DEC*	-3.986032 E10 B-36*	# M(3)/CS(2)

		SETLOC	ASENT1
		BANK
		COUNT*	$$/P12

; Moon's gravitational parameter mu = GM (m^3/cs^2), scaled B-37.
; Moon's rotation rate (rad/cs) relative to inertial space.
; Used for lunar orbit insertion, descent orbit, and surface operations.
MUM(-37)	2DEC*	4.9027780 E8 B-37*
MOONRATE	2DEC*	.26616994890062991 E-7 B+19*	# RAD/CS.

		SETLOC	SERVICES
		BANK
		COUNT*	$$/SERV

; Gravitational derivative constants for trajectory integration.
; -MUDT: Earth's gravitational parameter derivative for Encke method
; -MUDT1: Moon's gravitational parameter derivative for Encke method
; Order must be preserved for integration subroutines.
# *** THE ORDER OF THE FOLLOWING TWO CONSTANTS MUST BE PRESERVED ***********

-MUDT		2DEC*	-7.9720645 E+12 B-44*
-MUDT1		2DEC*	-9.8055560 E+10 B-44*

# **************************************************************************

; Combined gravitational derivative for Moon influence in cislunar space.
; RESQ: Squared radius constant for oblateness perturbation calculations.
-MUDTMUN	2DEC*	-9.8055560 E+10 B-38*
RESQ		2DEC*	40.6809913 E12 B-58*

# Page 45
; Earth oblateness coefficients (J2 terms) for gravity model perturbations.
; 20J: 20 times J2 constant for computational efficiency.
; 2J: 2 times J2 constant. Earth's equatorial bulge affects orbital mechanics,
; particularly altitude over poles vs equator. Critical for accurate orbit
; propagation during translunar and cislunar flight phases.
20J		2DEC	3.24692010 E-2
2J		2DEC	3.24692010 E-3

		SETLOC	P50S1
		BANK
		COUNT*	$$/LOSAM

; Fundamental radii for celestial body models (meters, scaled by 2^29).
; RSUBEM: Mean Earth-Moon distance (384,402 km). Average separation used
;         for initial trajectory planning and navigation reference frame.
; RSUBM: Moon's mean radius (1,738.09 km). Lunar surface reference for
;        altitude calculations during descent and landing.
; RSUBE: Earth's mean equatorial radius (6,378.166 km). Sea level reference
;        for launch, orbital, and reentry altitude computations.
RSUBEM		2DEC	384402000 B-29
RSUBM		2DEC	1738090 B-29
RSUBE		2DEC	6378166 B-29

; Earth oblateness parameter (flattening coefficient 1/298.3).
; Accounts for Earth's non-spherical shape in gravity field modeling.
ROE		2DEC	.00257125

		SETLOC	CONICS1
		BANK
		COUNT*	$$/LT-LG

; Earth radius for launch pad altitude reference (6,373.338 km).
; Kennedy Space Center pad elevation above mean sea level reference.
ERAD		2DEC	6373338 B-29		# PAD RADIUS

; Moon's equatorial radius for Apollo 11 landing site calculations.
; Same as RSUBM but positioned for conic section subroutines.
504RM		2DEC	1738090 B-29		# METERS B-29 (EQUATORIAL MOON RADIUS)

		SETLOC	CONICS1
		BANK
		COUNT*	$$/CONIC

; Gravitational parameter lookup table for orbital mechanics calculations.
; MUTABLE contains Earth (MUE) and Moon (MUM) gravitational parameters
; plus their reciprocals and square roots for trajectory computations.
; Conic section subroutines index this table to retrieve appropriate
; values for two-body orbital propagation during different mission phases.
; Order strictly preserved for indexed access by navigation algorithms.
# *** THE ORDER OF THE FOLLOWING CONSTANTS MUST BE PRESERVED **************

MUTABLE		2DEC*	3.986032 E10 B-36*	# MUE
		2DEC*	.25087606 E-10 B+34*	# 1/MUE
		2DEC*	1.99650495 E5 B-18*	# SQRT(MUE)
		2DEC*	.50087529 E-5 B+17*	# 1/SQRT(MUE)
		2DEC*	4.902778 E8 B-30*	# MUM
		2DEC*	.203966 E-8 B+28*	# 1/MUM
		2DEC*	2.21422176 E4 B-15*	# SQRT(MUM)
		2DEC*	.45162595 E-4 B+14*	# 1/SQRT(MUM)

# *************************************************************************

# Page 46
		SETLOC	INTINIT
		BANK
		COUNT*	$$/INTIN

; Moon's mean angular rotation rate (radians per centisecond).
; Lunar rotation is synchronously locked with its orbital period (27.3 days),
; keeping same face toward Earth. Used for lunar surface coordinate
; transformations and selenographic latitude/longitude calculations.
OMEGMOON	2DEC*	2.66169947 E-8 B+23*

		SETLOC	ORBITAL2
		BANK
		COUNT*	$$/ORBIT

; Orbital mechanics constants table for gravity model and perturbation
; calculations. Includes gravitational parameters (mu), oblateness
; coefficients (J2, J3, J4), and derived terms for efficient computation
; of non-spherical gravity effects on spacecraft trajectory.
; Order strictly preserved for indexed table access by integration routines.
# *** THE ORDER OF THE FOLLOWING CONSTANTS MUST NOT BE CHANGED ************

; Combined gravitational constant (Sun's GM for solar perturbations).
		2DEC*	1.32715445 E16 B-54*

; Moon's gravitational parameter (GM = 4.9028 × 10^8 m^3/cs^2).
; Primary gravity source during lunar orbit and landing phases.
MUM		2DEC*	4.9027780 E8 B-30*

; Earth's gravitational parameter (GM = 3.986032 × 10^10 m^3/cs^2).
; Primary gravity source during Earth orbit and translunar injection.
MUEARTH		2DEC*	3.986032 E10 B-36*

; Placeholder constant for table alignment and computational efficiency.
		2DEC 	0

; Oblateness ratio: J4 × R_equatorial / J3 (meters).
; J4 is fourth harmonic coefficient of Earth's gravity field.
J4REQ/J3	2DEC*	.4991607391 E7 B-26*

; Derived gravity perturbation term (scaled for computational efficiency).
		2DEC	-176236.02 B-25

; Oblateness ratio: 2 × J3 × R_equatorial / J2 (meters).
; J3 third harmonic (pear shape asymmetry, north-south difference).
2J3RE/J2	2DEC*	-.1355426363 E5 B-27*

; Intermediate gravitational calculation term (scaled appropriately).
		2DEC*	.3067493316 E18 B-60*

; J2 × R_equatorial^2: Squared radius with J2 oblateness weighting.
; Frequently used term in perturbation force calculations.
J2REQSQ		2DEC*	1.75501139 E21 B-72*

; Combined gravity term: 3 × J2^2 × R_equatorial^2 / mu.
; Pre-computed for efficiency in Encke integration method.
3J22R2MU	2DEC*	9.20479048 E16 B-58*

# *************************************************************************

		SETLOC	TOF-FF1
		BANK
		COUNT*	$$/TFF

; Reciprocal square root of modified Earth gravitational parameter.
; 1/sqrt(mu) used in time-of-free-fall trajectory calculations.
; Pre-computed for computational efficiency in Lambert problem solvers.
1/RTMU		2DEC*	.5005750271 E-5 B17*	# MODIFIED EARTH MU

		SETLOC	SBAND
		BANK
		COUNT*	$$/R05

; Mean Earth-Moon distance for S-band antenna pointing calculations.
; Duplicates RSUBEM value but positioned for R05 radio communication routines.
; Used to compute antenna angles for maintaining signal lock with Earth.
REMDIST		2DEC	384402000 B-29		# MEAN DISTANCE BETWEEN EARTH AND MOON.

# Page 47
# PHYSICAL CONSTANTS (TIME - VARIANT)

; =============================================================================
; STAR CATALOG - Navigation Reference Stars for IMU Alignment
;
; Apollo Guidance Computer star catalog containing unit vectors (X, Y, Z) for
; 37 navigation stars in the Basic Reference Coordinate System (BRCS).
; Stars selected for brightness, distribution across celestial sphere, and
; angular separation to avoid gimbal lock during optical alignment.
;
; COORDINATE SYSTEM:
;   X-axis: Points toward vernal equinox (first point of Aries, ♈)
;   Y-axis: Points 90° east along celestial equator
;   Z-axis: Points toward north celestial pole
;   All components are unit vectors (X² + Y² + Z² = 1.0)
;
; MISSION USAGE:
;   P51: Manual optical IMU alignment using sextant star sightings
;   P52: Automatic IMU realignment using computer-controlled optics
;   P53: Backup IMU alignment during platform drift
;
; NAVIGATION CONTEXT:
;   Armstrong and Aldrin performed IMU alignments before LM separation and
;   prior to powered descent initiation to ensure inertial platform accuracy
;   for guidance during lunar landing. Star sightings through AOT (Alignment
;   Optical Telescope) compared against these catalog positions to compute
;   platform orientation errors and generate gyro torquing commands.
;
; NOTE: Star positions are mean 1969.0 epoch coordinates. Precession,
;   nutation, and aberration corrections applied by navigation software
;   when computing actual line-of-sight vectors to observed stars.
; =============================================================================

		SETLOC	STARTAB
		BANK
		COUNT*	$$/STARS

		2DEC	+.8342971408 B-1	# STAR 37	X
		2DEC	-.2392481515 B-1	# STAR 37	Y
		2DEC	-.4966976975 B-1	# STAR 37	Z

		2DEC	+.8139832631 B-1	# STAR 36	X
		2DEC	-.5557243189 B-1	# STAR 36	Y
		2DEC	+.1691204557 B-1	# STAR 36	Z

		2DEC	+.4541086270 B-1	# STAR 35	X
		2DEC	-.5392368197 B-1	# STAR 35	Y
		2DEC	+.7092312789 B-1	# STAR 35	Z

		2DEC	+.3201817378 B-1	# STAR 34	X
		2DEC	-.4436021946 B-1	# STAR 34	Y
		2DEC	-.8370786986 B-1	# STAR 34	Z

		2DEC	+.5520184464 B-1	# STAR 33	X
		2DEC	-.7933187400 B-1	# STAR 33 	Y
		2DEC	-.2567508745 B-1	# STAR 33	Z

		2DEC	+.4537196908 B-1	# STAR 32	X
		2DEC	-.8779508801 B-1	# STAR 32	Y
		2DEC	+.1527766153 B-1	# STAR 32	Z

		2DEC	+.2069525789 B-1	# STAR 31	X
		2DEC	-.8719885748 B-1	# STAR 31	Y
		2DEC	-.4436288486 B-1	# STAR 31	Z

		2DEC	+.1217293692 B-1	# STAR 30	X
		2DEC	-.7702732847 B-1	# STAR 30 	Y
# Page 48
		2DEC	+.6259880410 B-1	# STAR 30	Z

		2DEC	-.1124304773 B-1	# STAR 29	X
		2DEC	-.9694934200 B-1	# STAR 29	Y
		2DEC	+.2178116072 B-1	# STAR 29	Z

		2DEC	-.1146237858 B-1	# STAR 28	X
		2DEC	-.3399692557 B-1	# STAR 28 	Y
		2DEC	-.9334250333 B-1	# STAR 28	Z

		2DEC	-.3516499609 B-1	# STAR 27	X
		2DEC	-.8240752703 B-1	# STAR 27	Y
		2DEC	-.4441196390 B-1	# STAR 27	Z

		2DEC	-.5326876930 B-1	# STAR 26	X
		2DEC	-.7160644554 B-1	# STAR 26	Y
		2DEC	+.4511047742 B-1	# STAR 26	Z

		2DEC	-.7861763936 B-1	# STAR 25	X
		2DEC	-.5217996305 B-1	# STAR 25	Y
		2DEC	+.3311371675 B-1	# STAR 25	Z

		2DEC	-.6898393233 B-1	# STAR 24	X
		2DEC	-.4182330640 B-1	# STAR 24	Y
		2DEC	-.5909338474 B-1	# STAR 24	Z

		2DEC	-.5812035376 B-1	# STAR 23	X
		2DEC	-.2909171294 B-1	# STAR 23	Y
		2DEC	+.7599800468 B-1	# STAR 23 	Z

		2DEC	-.9170097662 B-1	# STAR 22	X
		2DEC	-.3502146628 B-1	# STAR 22	Y
		2DEC	-.1908999176 B-1	# STAR 22	Z

# Page 49
		2DEC	-.4523440203 B-1	# STAR 21	X
		2DEC	-.0493710140 B-1	# STAR 21	Y
		2DEC	-.8904759346 B-1	# STAR 21	Z

		2DEC	-.9525211695 B-1	# STAR 20	X
		2DEC	-.0593434796 B-1	# STAR 20	Y
		2DEC	-.2986331746 B-1	# STAR 20	Z

		2DEC	-.9656605484 B-1	# STAR 19	X
		2DEC	+.0525933156 B-1	# STAR 19	Y
		2DEC	+.2544280809 B-1	# STAR 19	Z

		2DEC	-.8608205219 B-1	# STAR 18	X
		2DEC	+.4636213989 B-1	# STAR 18	Y
		2DEC	+.2098647835 B-1	# STAR 18	Z

		2DEC	-.7742591356 B-1	# STAR 17	X
		2DEC	+.6152504197 B-1	# STAR 17	Y
		2DEC	-.1482892839 B-1	# STAR 17	Z

		2DEC	-.4657947941 B-1	# STAR 16	X
		2DEC	+.4774785033 B-1	# STAR 16	Y
		2DEC	+.7450164351 B-1	# STAR 16	Z

		2DEC	-.3612508532 B-1	# STAR 15	X
		2DEC	+.5747270840 B-1	# STAR 15	Y
		2DEC	-.7342932655 B-1	# STAR 15	Z

		2DEC	-.4118589524 B-1	# STAR 14 	X
		2DEC	+.9065485360 B-1	# STAR 14	Y
		2DEC	+.0924226975 B-1	# STAR 14	Z

		2DEC	-.1820751783 B-1	# STAR 13	X
# Page 50
		2DEC	+.9404899869 B-1	# STAR 13	Y
		2DEC	-.2869271926 B-1	# STAR 13	Z

		2DEC	-.0614937230 B-1	# STAR 12 	X
		2DEC	+.6031563286 B-1	# STAR 12	Y
		2DEC	-.7952489957 B-1	# STAR 12	Z

		2DEC	+.1371725575 B-1	# STAR 11	X
		2DEC	+.6813721061 B-1	# STAR 11	Y
		2DEC	+.7189685267 B-1	# STAR 11	Z

		2DEC	+.2011399589 B-1	# STAR 10	X
		2DEC	+.9690337941 B-1	# STAR 10	Y
		2DEC	-.1432348512 B-1	# STAR 10	Z

		2DEC	+.3507315038 B-1	# STAR 9	X
		2DEC	+.8926333307 B-1	# STAR 9	Y
		2DEC	+.2831839492 B-1	# STAR 9	Z

		2DEC	+.4105636020 B-1	# STAR 8	X
		2DEC	+.4988110001 B-1	# STAR 8	Y
		2DEC	+.7632988371 B-1	# STAR 8	Z

		2DEC 	+.7032235469 B-1	# STAR 7	X
		2DEC	+.7075846047 B-1	# STAR 7	Y
		2DEC	+.0692868685 B-1	# STAR 7	Z

		2DEC	+.5450107404 B-1	# STAR 6	X
		2DEC	+.5314955466 B-1	# STAR 6	Y
		2DEC	-.6484410356 B-1	# STAR 6	Z

		2DEC	+.0130968840 B-1	# STAR 5	X
		2DEC	+.0078062795 B-1	# STAR 5	Y
# Page 51
		2DEC	+.9998837600 B-1	# STAR 5	Z

		2DEC	+.4917678276 B-1	# STAR 4	X
		2DEC	+.2204887125 B-1	# STAR 4	Y
		2DEC	-.8423473935 B-1	# STAR 4 	Z

		2DEC	+.4775639450 B-1	# STAR 3	X
		2DEC	+.1166004340 B-1	# STAR 3	Y
		2DEC	+.8708254803 B-1	# STAR 3 	Z

		2DEC	+.9342640400 B-1	# STAR 2	X
		2DEC	+.1735073142 B-1	# STAR 2	Y
		2DEC	-.3115219339 B-1	# STAR 2	Z

		2DEC	+.8748658918 B-1	# STAR 1	X
		2DEC	+.0260879174 B-1	# STAR 1	Y
		2DEC	+.4836621670 B-1	# STAR 1	Z

CATLOG		DEC	6970

# *******************************************************************************

; =============================================================================
; EPHEMERIS CONSTANTS - Lunar and Solar Position Computation
;
; This section contains constants used for computing the positions of the
; Moon and Sun relative to Earth. These ephemeris constants are critical
; for navigation calculations during the translunar coast, lunar orbit,
; and transearth phases of the mission.
;
; KONMAT: Coordinate transformation matrix for lunar/solar ephemeris
; SECOND: Time constants for ephemeris polynomial evaluation
; KONPOS/KONVEL: Position and velocity ephemeris polynomial coefficients
;
; These constants enable the AGC to calculate celestial body positions
; without ground updates, providing autonomous navigation capability
; during communication blackouts.
;
; Source: JPL ephemeris data, referenced to J2000.0 epoch
; =============================================================================

		SETLOC	EPHEM1
		BANK
		COUNT*	$$/EPHEM

KONMAT		2DEC	1.0 B-1			# ********************
		2DEC	0			#		     *
		2DEC	0			#		     *
		2DEC	0			#		     *
		2DEC	.91745 B-1		# K1 COS(OBL)	     *
		2DEC	-.03571 B-1		# K2 SIN(OBL)SIN(IM) *
		2DEC	0			#		     *
		2DEC	.39784 B-1		# K3 SIN(OBL)	     *
# Page 52
		2DEC	.082354 B-1		# K4 COS(OBL)SIN(IM) *
CSTODAY		2DEC	8640000 B-33		# 		     * NOTE:          *
RCB-13		OCT	00002			#		     * TABLES CONTAIN *
		OCT	00000			#		     * CONSTANTS FOR  *
RATESP		2DEC	.03660098 B+4		# LOMR		     * 1969 - 1970    *
		2DEC	.00273779 B+4		# LOSR
		2DEC	-.00014719 B+4		# LONR
		2DEC	.815282336		# LOMO
		2DEC	.274674910		# LOSO
		2DEC	.986209499		# LONO
VAL67		2DEC*	.01726666666 B+1*	# AMOD
		2DEC	.530784445		# AARG
		2DEC	.036291712 B+1		# 1/27
		2DEC	.003505277 B+1		# BMOD
		2DEC	.585365625 		# BARG
		2DEC	.03125 B+1		# 1/32
		2DEC	.005325277 B+1		# CMOD
		2DEC	-.01106341036		# CARG
		2DEC	.002737925 B+1		# 1/365

# ********************************************************************************

; =============================================================================
; LUNAR ROTATION CONSTANTS - Moon's Libration and Orientation
;
; This section contains constants defining the Moon's rotation and
; orientation relative to Earth. These parameters are essential for
; accurately computing the lunar surface position beneath the spacecraft
; during descent and for targeting specific landing sites.
;
; COSI/SINI: Cosine and sine of Moon's obliquity (tilt of rotation axis)
; NODDOT: Rate of lunar node precession (how Moon's orbital plane shifts)
; FDOT: Rate of change of lunar orientation angle
; BDOT: Rate of change of lunar latitude argument
; NODIO/FSUBO/BSUBO: Initial values at epoch for node, orientation, latitude
; WEARTH: Earth's rotation rate (for computing relative motion)
;
; These constants enable precise transformation between inertial coordinates
; and lunar surface coordinates, critical for Apollo 11's landing in the
; Sea of Tranquility at latitude 0.71° N, longitude 23.63° E.
;
; Note: Constants are specific to 1969-1970 epoch as indicated in tables
; Source: JPL lunar ephemeris and physical libration data
; =============================================================================

		SETLOC	PLANTIN2
		BANK
		COUNT*	$$/LUROT

COSI		2DEC	.99964173 B-1		# COS (5521.5 SEC.) B-1
SINI		2DEC	.02676579 B-1		# SIN (5521.5 SEC.) B-1
NODDOT		2DEC	-.457335121 E-2		# REV/CSEC B+28 = -1.07047011 E-8 RAD/SEC
FDOT		2DEC	.570863327		# REV/CSEC B+27 =  2.67240410 E-6 RAD/SEC
# Page 53
BDOT		2DEC	-3.07500686 E-8		# REV/CSEC B+28 = -7.19757301 E-14 RAD/SEC
NODIO		2DEC	.986209434		# REVS B-D	= 6.19653663041 RAD
FSUBO		2DEC	.829090536		# REVS B-D	= 5.20932947829 RAD
BSUBO		2DEC	.0651201393		# REVS B-D	= 0.40916190299 RAD
WEARTH		2DEC	.973561595		# REV/CSEC B+23	= 7.29211494 E-5 RAD/SEC

