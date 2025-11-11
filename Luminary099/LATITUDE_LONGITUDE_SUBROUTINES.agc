# Copyright:	Public domain.
# Filename:	LATITUDE_LONGITUDE_SUBROUTINES.agc
# Purpose:	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
#
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim DOT lawton AT gmail DOT com>
# Website:	www.ibiblio.org/apollo.
# Pages:	1133-1139
# Mod history:	2009-05-28 JL	Updated from page images.
#		2011-01-06 JL	Fixed interpretive indentation.
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
; FILE: LATITUDE_LONGITUDE_SUBROUTINES.agc
; MODULE: Coordinate Transformation Utilities
; MISSION PHASE: All phases (launch through landing and ascent)
;
; TL;DR: Converts between Cartesian position vectors and geodetic coordinates
;        (latitude, longitude, altitude) for both Earth and Moon. Implements
;        Fischer ellipsoid model for Earth and spherical model for Moon.
;        Essential for navigation displays, landing site targeting, and ground
;        tracking computations throughout the mission.
;
; COMMENT-ONLY READERS: These routines translate the spacecraft's position
;        from raw navigation data into familiar geographic coordinates that
;        the crew and mission control can easily understand and verify.
; CODE-ALONG READERS: Study the geodetic coordinate transformation mathematics,
;        reference ellipsoid handling, and fixed-point arithmetic scaling used
;        to maintain precision within AGC's 15-bit word constraints.
; ============================================================================

; ============================================================================
; TRANSITION: Position Vector to Geographic Coordinates Conversion
;
; The guidance computer maintains the spacecraft's position as a Cartesian
; vector in the inertial reference frame. For navigation displays, landing
; site verification, and ground tracking correlation, this position must be
; converted to familiar latitude, longitude, and altitude coordinates relative
; to Earth or Moon surface. The LAT-LONG subroutine performs this essential
; transformation using geodetic coordinate mathematics.
; ============================================================================

# Page 1133
# SUBROUTINE TO CONVERT RAD VECTOR AT GIVEN TIME TO LAT,LONG AND ALT
#
# CALLING SEQUENCE
#
#   L-1	   CALL
#   L		  LAT-LONG
# SUBROUTINES USED
#
#  R-TO-RP,ARCTAN,SETGAMMA,SETRE
# ERASABLE INIT. REQ.
#
#   AXO,-AYO,AZO,TEPHEM (SET AT LAUNCH TIME)
# ALPHAV = POSITION VECTOR METERS B-29
#   MPAC-- TIME  (CSECS B-28)
#  ERADFLAG =1, TO COMPUTE EARTH RADIUS, =0 FOR FIXED EARTH RADIUS
#  LUNAFLAG=0 FOR EARTH,1 FOR MOON
# OUTPUT
#
#   LATITUDE IN LAT   (REVS. B-0)
#   LONGITUDE IN LONG	(REVS. B-0)
# ALTITUDE IN ALT METERS B-29

; LAT-LONG SUBROUTINE - POSITION VECTOR TO GEODETIC COORDINATES
;
; This routine converts the spacecraft's inertial position vector into
; latitude, longitude, and altitude above the planetary surface. For Earth,
; it uses the Fischer ellipsoid model accounting for Earth's oblate shape.
; For the Moon, it uses a simpler spherical model.
;
; The transformation involves:
; 1. Converting from inertial reference frame to planet-fixed rotating frame
; 2. Accounting for Earth's oblateness (flattening at poles) using gamma factor
; 3. Computing geodetic latitude (perpendicular to ellipsoid) not geocentric
; 4. Computing longitude from equatorial plane projections
; 5. Computing altitude as distance from reference ellipsoid surface
;
; During lunar landing, this routine converts the LM's position into latitude
; and longitude coordinates that Armstrong and Aldrin could compare against
; their expected landing site coordinates in the Sea of Tranquility.
		BANK	30
		SETLOC	LATLONG
		BANK

		COUNT*	$$/LT-LG
		EBANK=	ALPHAV
;
; Entry point for position vector to lat/lon/alt conversion.
; Input: ALPHAV contains position vector in meters scaled B-29 (1 bit = 1.86 nm)
;        Time in MPAC in centiseconds scaled B-28
;        LUNAFLAG: 0=Earth, 1=Moon
;        ERADFLAG: 0=fixed radius, 1=compute radius using ellipsoid
;
LAT-LONG	STQ	SETPD
			INCORPEX
			0D
		STOVL	6D		# SAVE TIME IN 6-7D FOR R-TO-RP
			ALPHAV
;
; Load position vector and compute its magnitude for altitude calculation.
; The magnitude (distance from planetary center) will be compared against
; the reference ellipsoid radius to determine altitude above surface.
;
		PUSH	ABVAL		# 0-5D= R FOR R-TO-RP
		STODL	ALPHAM		# ABS. VALUE OF R FOR ALT FORMULA BELOW
			ZEROVEC		# SET MPAC=0 FOR EARTH,NON-ZERO FOR MOON
;
; Branch based on planetary body. LUNAFLAG determines whether we're computing
; coordinates relative to Earth (oblate ellipsoid) or Moon (sphere).
;
		BOFF	COS		# USE COS(0) TO GET NON-ZERO IN MPAC
			LUNAFLAG	# 0=EARTH,1=MOON
			CALLRTRP
CALLRTRP	CALL
			R-TO-RP		# RP VECTOR CONVERTED FROM R B-29
;
; Transform from inertial reference frame to planet-fixed rotating frame.
; This accounts for Earth's rotation and converts the position vector from
; the stable inertial coordinate system to coordinates that rotate with the
; planet's surface. Called via PLANETARY_INERTIAL_ORIENTATION.agc.
		UNIT			# UNIT RP B-1
		STCALL	ALPHAV		# U2= 1/2 SINL FOR SETRE SUBR BELOW
			SETGAMMA	#  SET GAMMA=B2/A2 FOR EARTH,=1 FOR MOON
;
; Set gamma factor accounting for planetary oblateness.
; For Earth: gamma = B²/A² = 0.9933 where A=6378166m, B=6356784m (Fischer ellipsoid)
; For Moon: gamma = 1.0 (perfect sphere approximation)
; This factor converts geocentric to geodetic coordinates.
;
		CALL			#  SCALED B-1
			SETRE		# CALC RE METERS B-29
;
; Compute planetary radius at current latitude. For Earth, this varies from
; equatorial radius (6378.166 km) to polar radius (6356.784 km). For Moon,
; uses mean radius. Result scaled B-29 (1 bit = 1.86 nanometers).
;
		DLOAD	DSQ
			ALPHAV
		PDDL	DSQ
			ALPHAV +2
		DAD	SQRT
;
; Compute latitude from planet-fixed position vector components.
; ALPHAV contains [X, Y, Z] in planet-fixed rotating frame.
; Latitude computed from: sin(lat) = Z / sqrt(X² + Y² + gamma²Z²)
; Using geodetic latitude (perpendicular to ellipsoid) not geocentric.
;
# Page 1134
		DMP	SL1R
			GAMRP
		STODL	COSTH		# COS(LAT) B-1
			ALPHAV +4
		STCALL	SINTH		# SIN(LAT) B-1
			ARCTAN
;
; ARCTAN subroutine computes latitude angle from sin and cos components.
; Output in revolutions (1 revolution = 360 degrees) scaled B-0.
; Range: -0.5 to +0.5 revolutions (-180° to +180°).
;
		STODL	LAT		# LAT B0
			ALPHAV
;
; Compute longitude from equatorial plane projection.
; Longitude computed from: tan(lon) = Y / X in planet-fixed frame.
; For Earth, this gives longitude relative to Greenwich meridian.
; For Moon, relative to Moon's prime meridian (facing Earth at reference time).
;
		STODL	COSTH		# COS(LONG) B-1
			ALPHAV +2
		STCALL	SINTH		# SIN(LONG) B-1
			ARCTAN
		STODL	LONG		# LONG. REVS B-0 IN RANGE -1/2 TO 1/2
			ALPHAM
;
; Compute altitude above reference ellipsoid.
; Altitude = |position vector| - reference radius at this latitude.
; For lunar landing, this gives height above Moon's mean surface.
; At touchdown, ALT approaches zero as LM settles on lunar surface.
;
		DSU			# ALT= R-RE METERS B-29
			ERADM
		STCALL	ALT		# EXIT WITH ALT METERS B-29
			INCORPEX
; ============================================================================
; TRANSITION: Geodetic Coordinates to Position Vector Conversion
;
; The reverse transformation converts latitude, longitude, and altitude back
; into a Cartesian position vector. This is essential when the crew or ground
; control specifies a target location in familiar geographic coordinates that
; must be converted to navigation vectors for guidance computations.
; ============================================================================

# Page 1135
# SUBROUTINE TO CONVERT LAT,LONG,ALT AT GIVEN TIME TO RADIUS VECTOR
# CALLING SEQUENCE
#
#   L-1	   CALL
#   L		  LALOTORV
# SUBROUTINES USED
#
#  SETGAMMA,SETRE,RP-TO-R
# ERASABLE INIT. REQ.
#
#   AXO,AYO,AZO,TEPHEM  SET AT LAUNCH TIME
#   LAT-- LATITUDE  (REVS B0)
#   LONG-- LONGITUDE  (REVS B0)
# ALT--ALTITUDE	(METERS) B-29
#   MPAC-- TIME	 (CSECS B-28)
#   ERADFLAG =1 TO COMPUTE EARTH RADIUS, =0 FOR FIXED EARTH RADIUS
#  LUNAFLAG=0 FOR EARTH,1 FOR MOON
# OUTPUT
#
# R-VECTOR IN ALPHAV (METERS B-29)

; LALOTORV SUBROUTINE - GEODETIC COORDINATES TO POSITION VECTOR
;
; This routine performs the inverse transformation: converting latitude,
; longitude, and altitude into a Cartesian position vector. Essential for
; targeting computations when landing sites or orbital positions are specified
; in familiar lat/lon coordinates.
;
; During Apollo 11 mission planning, the landing site in the Sea of Tranquility
; was specified as approximately 0.67°N, 23.47°E. This routine would convert
; those coordinates into the position vector needed for guidance targeting.

;
; Entry point: Save return address and initialize push-down stack.
; Time parameter needed for planetary rotation correction.
;
LALOTORV	STQ	SETPD		# LAT,LONG,ALT TO R VECTOR
			INCORPEX
			0D
		STCALL	6D		# 6-7D= TIME FOR RP-TO-R
			SETGAMMA	# GAMMA=B2/A2 FOR EARTH,1 FOR MOON B-1
;
; Construct unit position vector in planet-fixed rotating frame from
; geodetic coordinates. The vector components are:
;   X = cos(latitude) * cos(longitude)
;   Y = cos(latitude) * sin(longitude)  
;   Z = gamma * sin(latitude)
; where gamma accounts for planetary oblateness (0.9933 for Earth, 1.0 for Moon).
;
		DLOAD	SIN		#           	COS(LONG)COS(LAT) IN MPAC
			LAT		#     UNIT RP=	SIN(LONG)COS(LAT)    2-3D
		DMPR	PDDL		# PD 2      	GAMMA*SIN(LAT)       0-1D
			GAMRP
			LAT		#     	  0-1D= GAMMA*SIN(LAT) B-2
		COS	PDDL		# PD4 	   2-3D=COS(LAT) B-1 TEMPORARILY
			LONG
		SIN	DMPR		# PD 2
		PDDL	COS		# PD 4 	  2-3D=SIN(LONG)COS(LAT) B-2
			LAT
		PDDL	COS		# PD 6 	   4-5D=COS(LAT) B-1 TEMPORARILY
			LONG
		DMPR	VDEF		# PD 4 	 MPAC=	COS(LONG)COS(LAT) B-2
;
; Normalize to unit vector. This represents direction from planet center
; to surface point at specified lat/lon, accounting for ellipsoidal shape.
;
		UNIT	PUSH		# 0-5D= UNIT RP FOR RP-TO-R SUBR.
;
; Compute planetary radius at specified latitude. For Earth, this accounts
; for ellipsoidal flattening: equatorial radius 6378.166 km, polar radius
; 6356.784 km. For Moon, uses mean radius 1738.09 km (spherical approximation).
;
		STCALL	ALPHAV		# ALPHAV +4= SINL FOR SETRE SUBR.
			SETRE		# RE METERS B-29
;
; Convert from planet-fixed rotating frame to inertial frame, accounting
; for planetary rotation since reference epoch. For Earth operations, corrects
; for Earth's rotation. For lunar landing, accounts for Moon's rotation
; (though Moon is tidally locked, small libration corrections still apply).
;
		DLOAD	BOFF		# SET MPAC=0 FOR EARTH,NON-ZERO FOR MOON
			ZEROVEC
			LUNAFLAG
			CALLRPRT
		COS			# USE COS(0) TO GET NON-ZERO IN MPAC
CALLRPRT	CALL
			RP-TO-R		# EXIT WITH UNIT R VECTOR IN MPAC
;
; Compute final position vector: r = (radius + altitude) * unit_vector.
; This gives the position vector in meters scaled B-29, pointing from planet
; center to specified location at given altitude above the surface.
;
		STODL	ALPHAV
			ERADM
# Page 1136
		DAD	VXSC		# (RE + ALT)(UNIT R) METERS B-30
			ALT
			ALPHAV
;
; Scale result correctly. Final position vector in ALPHAV represents the
; Cartesian coordinates needed for navigation computations, trajectory planning,
; or targeting calculations (e.g., aiming for landing site coordinates).
;
		VSL1			# R METERS B-29
		STCALL	ALPHAV		# EXIT WITH R IN METERS B-29
			INCORPEX

; ============================================================================
; TRANSITION: Ellipsoid Radius Computation
;
; The next section provides utility subroutines for computing planetary radius
; as a function of latitude, accounting for Earth's ellipsoidal shape. These
; calculations are fundamental to accurate geodetic coordinate transformations.
; ============================================================================

# SUBROUTINE TO COMPUTE EARTH RADIUS
#
# INPUT
#
#   1/2 SIN LAT IN ALPHAV +4
#
# OUTPUT
#
#   EARTH RADIUS IN ERADM AND MPAC (METERS B-29)

; GETERAD SUBROUTINE - EARTH RADIUS AT LATITUDE
;
; Computes Earth's radius as a function of geodetic latitude using the
; Fischer ellipsoid model (1960). Earth is flattened at the poles due to
; rotation, with equatorial radius 6378.166 km and polar radius 6356.784 km.
; This ~21 km variation must be accounted for in precise navigation.
;
; Formula: R(φ) = sqrt[(a²cos²φ + b²sin²φ) / (a²cos²φ/b² + sin²φ)]
; where a = equatorial radius, b = polar radius, φ = geodetic latitude
;
; The computation uses trigonometric identities and scaled arithmetic to
; maintain precision within the AGC's 15-bit word length constraints.

GETERAD		DLOAD	DSQ
			ALPHAV +4	# SIN**2(L)
;
; Compute cos²(latitude) from sin²(latitude) using identity: cos²φ = 1 - sin²φ
; Then calculate weighted combination accounting for ellipsoid flattening.
;
		SL1	BDSU
			DP1/2		# COS**2(L)
		DMPR	BDSU
			EE
			DP1/2
;
; Complete radius calculation using ellipsoid formula. The scaling constants
; EE and B2XSC encode the ellipsoid parameters (a² - b²)/a² and b² respectively,
; pre-scaled to match AGC fixed-point representation.
;
		BDDV	SQRT
			B2XSC
		SR4R
		STORE	ERADM
		RVQ

; ============================================================================
; EARTH ELLIPSOID CONSTANTS
; ============================================================================
;
; The following constants define the Fischer 1960 ellipsoid model used by
; Apollo for Earth geodetic calculations. These values are critical for
; accurate position determination during translunar coast, Earth orbit
; operations, and entry trajectory computations.
;
; Reference ellipsoid parameters:
;   Equatorial radius (a) = 6,378,166 meters (Earth's radius at equator)
;   Polar radius (b) = 6,356,784 meters (Earth's radius at poles)
;   Flattening = (a-b)/a = 1/298.3 (Earth bulges ~21 km at equator)
;
; This flattening results from Earth's rotation: centrifugal force causes
; equatorial bulge. For precise navigation, position errors up to 21 km
; would result if spherical approximation were used instead.
;
# THE FOLLOWING CONSTANTS WERE COMPUTED WITH A=6378166,B=6356784 METERS
# B2XSC= B**2 SCALED B-51
# B2/A2= B**2/A**2 SCALED B-1
# EE=(1-B**2/A**2) SCALED B-0
B2XSC		2DEC	.0179450689	# B**2 SCALED B-51
;
; B2XSC = b² scaled by 2^-51 for radius computation precision
; Value: (6356784)² / 2^51 = 0.0179450689
;
DP1/2		=	XUNIT
B2/A2		2DEC	.9933064884 B-1	# GAMMA= B**2/A**2 B-1
;
; B2/A2 = GAMMA = (b/a)² scaled B-1, the ellipsoid eccentricity parameter
; Value: (6356784/6378166)² = 0.9933064884
; Used to scale Z-component of position vector in geodetic transformations
;
EE		2DEC	6.6935116 E-3	# (1-B**2/A**2) B-0
;
; EE = ellipsoid eccentricity squared = 1 - (b/a)²
; Value: 1 - 0.9933064884 = 0.006693512 (dimensionless)
; Represents degree of Earth's oblateness in radius calculation formula
;

# Page 1137
# ARCTAN SUBROUTINE
#
# CALLING SEQUENCE
#
#   SIN THETA IN SINTH B-1
#   COS THETA IN COSTH B-1
#   CALL ARCTAN
#
# OUTPUT
#   ARCTAN THETA IN MPAC AND THETA B-0 IN RANGE -1/2 TO +1/2

; ============================================================================
; ARCTAN SUBROUTINE - TWO-ARGUMENT ARCTANGENT (ATAN2)
; ============================================================================
;
; Computes angle θ from sin(θ) and cos(θ), effectively implementing the
; atan2(y, x) function. This avoids the quadrant ambiguity of single-argument
; arctangent, correctly determining angles in all four quadrants.
;
; The algorithm normalizes the input, uses ASIN for the principal value, then
; adjusts the result based on the sign of cos(θ) to place θ in range [-0.5, +0.5]
; revolutions (equivalent to [-180°, +180°] or [-π, +π]).
;
; This is essential for latitude/longitude computation where angles must be
; correctly signed and quadrant-aware. During Apollo 11 lunar descent, these
; calculations determined the LM's position relative to the landing site.
;
; Special cases handled:
;   - sin=0, cos=0: Returns θ=0 (undefined atan, default to zero)
;   - cos=0: Returns θ=±0.25 revolutions (±90°) based on sign of sin
;   - Overflow conditions: Cleared and handled gracefully
;

ARCTAN		BOV
			CLROVFLW
;
; Entry point: Clear any overflow condition from previous operations.
; Then compute magnitude: sqrt(sin²θ + cos²θ) for normalization.
;
CLROVFLW	DLOAD	DSQ
			SINTH
		PDDL	DSQ
			COSTH
		DAD
;
; Check for zero magnitude (sin=0, cos=0 degenerate case).
; If zero, branch to return θ=0. Otherwise normalize by dividing sin by magnitude.
;
		BZE	SQRT
			ARCTANXX	# ATAN=0/0  SET THETA=0
		BDDV	BOV
			SINTH
			ATAN=90
;
; Compute principal value: θ = asin(sin/magnitude). This gives angle in range
; [-0.25, +0.25] revolutions. Then examine sign of cos(θ) to determine final
; quadrant adjustment.
;
		SR1	ASIN
		STORE	THETA
		PDDL	BMN
			COSTH
			NEGCOS
;
; Positive cosine: angle in range [-90°, +90°], return as-is.
;
		DLOAD	RVQ
;
; Negative cosine: angle in range [90°, 270°] (second or third quadrant).
; Must add or subtract 180° (0.5 revolutions) to place result correctly.
; Branch based on sign of theta (which reflects sign of sin).
;
NEGCOS		DLOAD	DCOMP
		BPL	DAD
			NEGOUT
			DP1/2
;
; Theta negative (third quadrant): add 0.5 to get angle in range [-180°, -90°].
; For example, if asin gave -30° and cos is negative, true angle is -150°.
;
ARCTANXX	STORE	THETA
		RVQ

;
; Theta positive (second quadrant): subtract 0.5 to get angle in range [90°, 180°].
; For example, if asin gave +30° and cos is negative, true angle is +150°.
;
NEGOUT		DSU	GOTO
			DP1/2
			ARCTANXX
;
; Special case: magnitude was zero (sin²+cos²=0), or division would give ±90°.
; Handle by returning ±0.25 revolutions (±90°) with sign matching sin(θ).
; This occurs when cos(θ)=0 exactly, placing angle on Y-axis.
;
ATAN=90		DLOAD	SIGN
			LODP1/4
			SINTH
		STORE	THETA
		RVQ

2DZERO		=	DPZERO

# Page 1138
# ..... SETGAMMA SUBROUTINE .....
# SUBROUTINE TO SET GAMMA FOR THE LAT-LONG AND LALOTORV SUBROUTINES
#
# GAMMA = B**2/A**2 FOR EARTH (B-1)
# GAMMA = 1 FOR MOON (B-1)
#
# CALLING SEQUENCE
#  L	   CALL
#  L+1		  SETGAMMA
#
# INPUT
#  LUNAFLAG=0 FOR EARTH,=1 FOR MOON
#
# OUTPUT
#  GAMMA IN GAMRP  (B-1)

; ============================================================================
; SETGAMMA SUBROUTINE - SET ELLIPSOID FLATTENING PARAMETER
; ============================================================================
;
; Configures the ellipsoid flattening parameter GAMMA based on the target
; celestial body. Earth is an oblate ellipsoid (flattened at poles) while
; the Moon is approximated as a sphere for Apollo navigation purposes.
;
; GAMMA = (b/a)² where:
;   For Earth: b = 6,356,784 m, a = 6,378,166 m → GAMMA = 0.9933064884
;   For Moon:  b = a (spherical approximation) → GAMMA = 1.0
;
; This parameter scales the Z-component (perpendicular to equatorial plane)
; in geodetic coordinate transformations. Earth's rotation causes ~21 km
; equatorial bulge, requiring ellipsoid correction. The Moon's slower rotation
; produces negligible flattening, so spherical model suffices.
;
; During Apollo 11, this subroutine was called for:
;   - Earth-based operations: Launch, translunar coast tracking, entry
;   - Lunar operations: Descent targeting, surface position, ascent
;
; Input: LUNAFLAG (0 = Earth, 1 = Moon)
; Output: GAMRP = GAMMA scaled B-1 (range 0.9933 to 1.0)
;

SETGAMMA	DLOAD	BOFF		# BRANCH FOR EARTH
			B2/A2		# EARTH GAMMA
			LUNAFLAG
			SETGMEX
;
; LUNAFLAG is set: Moon operations use spherical approximation (GAMMA = 1.0).
; Load single-precision constant 1.0 scaled B-1.
;
		SLOAD
			1B1		# MOON GAMMA
;
; Store GAMMA parameter in GAMRP memory location (erasable address 8D).
; Return to caller for subsequent geodetic transformation computations.
;
SETGMEX		STORE	GAMRP
		RVQ
GAMRP		=	8D

# Page 1139
# .....SETRE SUBROUTINE .....
# SUBROUTINE TO SET RE (EARTH OR MOON RADIUS)
#
# RE= RM FOR MOON
#  RE= RREF FOR FIXED EARTH RADIUS OR COMPUTED RF FOR FISCHER ELLIPSOID
#
# CALLING SEQUENCE
#  L	   CALL
#  L+1		  SETRE
#
# SUBROUTINES USED
#  GETERAD
#
# INPUT
#  ERADFLAG=0 FOR FIXED RE, 1 FOR COMPUTED RE
#  ALPHAV +4= 1/2 SINL IF GETERAD IS CALLED
#  LUNAFLAG=0 FOR EARTH,=1 FOR MOON
#
# OUTPUT
#  ERADM= 504RM FOR MOON (METERS B-29)
#  ERADM= ERAD OR COMPUTED RE FOR EARTH (METERS B-29)

; ============================================================================
; SETRE SUBROUTINE - SET PLANETARY RADIUS FOR ALTITUDE COMPUTATION
; ============================================================================
;
; Determines the appropriate planetary radius for geodetic calculations
; based on mission phase (Earth vs. Moon) and computation mode (fixed vs.
; latitude-dependent). Altitude is computed as: ALT = |position| - radius.
;
; For precise navigation, three radius computation modes are supported:
;
;   1. MOON OPERATIONS (LUNAFLAG=1):
;      - Uses mean lunar radius RM (approximately 1,738 km)
;      - Moon approximated as sphere (negligible flattening)
;      - RLS (landing site radius) used if ERADFLAG=0 for local precision
;
;   2. EARTH - FIXED RADIUS (ERADFLAG=0):
;      - Uses reference radius ERAD (typically 6,378 km equatorial)
;      - Simplified computation for non-critical mission phases
;      - Adequate for most navigation during translunar coast
;
;   3. EARTH - COMPUTED RADIUS (ERADFLAG=1):
;      - Calls GETERAD to compute latitude-dependent radius
;      - Accounts for Earth's ellipsoidal shape (21 km variation)
;      - Essential for entry trajectory and accurate Earth-based positioning
;
; During Apollo 11 mission:
;   - Launch/ascent: Earth radius for altitude displays to crew
;   - Lunar descent: Landing site radius for precise altitude above surface
;   - Entry: Latitude-dependent radius for trajectory control
;
; The subroutine manages return linkage through SETREX (stored Q register)
; and coordinates with GETERAD for latitude-dependent Earth radius.
;

SETRE		STQ	DLOAD
			SETREX
			504RM
;
; Save return address in SETREX. Load Moon radius constant (scaled 504RM).
; Check LUNAFLAG to determine Earth vs. Moon operations.
;
		BON	DLOAD		# BRANCH FOR MOON
			LUNAFLAG
			TSTRLSRM
			ERAD
;
; LUNAFLAG clear (Earth operations): Load reference Earth radius ERAD.
; Check ERADFLAG: if set, compute latitude-dependent radius via GETERAD.
; If clear, use fixed ERAD value for simplified computation.
;
		BOFF	CALL		# ERADFLAG=0 FOR FIXED RE,1 FOR COMPUTED
			ERADFLAG
			SETRXX
			GETERAD
;
; Store computed or fixed radius in ERADM and return to caller.
; Result in meters scaled B-29, ready for altitude computation.
;
SETRXX		STCALL	ERADM		# EXIT WITH RE OR RM METERS B-29
			SETREX
;
; Moon operations branch: Check ERADFLAG to select between landing site
; radius (RLS, for precision near landing target) or mean Moon radius (RM).
;
TSTRLSRM	BON	VLOAD		# ERADFLAG=0,SET R0=RLS
			ERADFLAG	#         =1     R0=RM
			SETRXX
			RLS
;
; RLS is position vector of landing site. Compute magnitude (absolute value)
; to get radial distance from Moon's center. Scale from B-27 to B-29 to match
; standard radius representation. During Apollo 11 descent, this provided
; altitude above the Sea of Tranquility reference surface.
;
		ABVAL	SR2R		# SCALE FROM B-27 TO B-29
		GOTO
			SETRXX
SETREX		=	S2
