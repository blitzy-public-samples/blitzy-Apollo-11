# Copyright:	Public domain.
# Filename:	PLANETARY_INERTIAL_ORIENTATION.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1243-1251
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

# Page 1243
# PLANETARY INERTIAL ORIENTATION

# ============================================================================
# FILE: PLANETARY_INERTIAL_ORIENTATION.agc
# MODULE: CHIEFTAN Subsystem (Core Operating System)
# MISSION PHASE: all-phases
#
# TL;DR: Coordinate frame transformation routines converting between inertial
#        reference frames and planetary rotating reference frames (Earth-fixed
#        and Moon-fixed). Implements REFSMMAT (Reference Stable Member Matrix)
#        computations enabling consistent coordinate transformations throughout
#        Apollo 11 navigation and guidance calculations.
#
# COMMENT-ONLY READERS: This module handled conversions between different
#        coordinate systems used throughout the mission for navigation and
#        spacecraft attitude control.
# CODE-ALONG READERS: Study coordinate frame transformations, REFSMMAT usage,
#        inertial to rotating frame conversions, transformation matrix
#        mathematics, and time-dependent planetary orientation computations.
# ============================================================================

# ============================================================================
# COORDINATE FRAME TRANSFORMATION OVERVIEW
#
# Throughout Apollo 11's mission, the AGC must work with multiple coordinate
# reference frames:
#
# INERTIAL REFERENCE FRAME (R vectors):
#   - Fixed relative to distant stars (non-rotating)
#   - Used for navigation state vectors and trajectory computations
#   - Basic reference system for all guidance calculations
#
# PLANETARY ROTATING FRAMES (RP vectors):
#   - Earth-fixed: Rotates with Earth (one rotation per day)
#   - Moon-fixed: Rotates with Moon (one rotation per lunar day ~27.3 Earth days)
#   - Used for landing site coordinates and surface-relative positions
#
# REFSMMAT (Reference Stable Member Matrix):
#   - Transformation matrix relating IMU orientation to reference coordinates
#   - Updated periodically through platform realignment procedures
#   - Critical for maintaining consistent attitude reference throughout mission
#
# The subroutines in this module convert position vectors between these frames,
# accounting for planetary rotation and libration (wobbling motion of the Moon).
# ============================================================================

# ..... RP-TO-R SUBROUTINE .....
# SUBROUTINE TO CONVERT RP (VECTOR IN PLANETARY COORDINATE SYSTEM,EITHER
# EARTH-FIXED OR MOON-FIXED) TO R (SAME VECTOR IN BASIC REF. SYSTEM)
#
# This transformation accounts for planetary rotation by applying the
# time-dependent transformation matrix M(T) and adding the effect of
# planetary libration (the LP x RP cross product term).
#
# For lunar landing operations, this converts landing site coordinates
# (expressed in Moon-fixed frame) to inertial reference frame coordinates
# needed for trajectory guidance computations.

#	R=MT(T)*(RP+LPXRP)	MT= M MATRIX TRANSPOSE

# CALLING SEQUENCE
#	L 	CALL
#	L+1		RP-TO-R

# SUBROUTINES USED
#	EARTHMX,MOONMX,EARTHL

# 	ITEMS AVAILABLE FROM LAUNCH DATA
#		504LM= THE LIBRATION VECTOR L OF THE MOON AT TIME TIMSUBL,EXPRESSED
#		IN THE MOON-FIXED COORD. SYSTEM		RADIANS B0
#			ITEMS NECESSARY FOR SUBR. USED (SEE DESCRIPTION OF SUBR.)

# INPUT
#	MPAC= 0 FOR EARTH,NON-ZERO FOR MOON
#	0-5D= RP VECTOR
#	6-7D= TIME

# OUTPUT
#	MPAC= R VECTOR METERS B-29 FOR EARTH, B-27 FOR MOON

		SETLOC	PLANTIN
		BANK

		COUNT*	$$/LUROT

# Entry point for planetary-to-inertial coordinate transformation.
# MPAC input determines planet selection: 0=Earth, non-zero=Moon.
RP-TO-R		STQ	BHIZ
			RPREXIT
			RPTORA
# Moon transformation path: Compute time-dependent orientation matrix.
		CALL			# COMPUTE M MATRIX FOR MOON
			MOONMX		# LP=LM FOR MOON	RADIANS B0
# Load lunar libration vector (Moon's rotational wobble parameters).
		VLOAD
			504LM
# Compute libration correction: LP x RP (cross product accounts for
# rotational wobble effect on position vector).
RPTORB		VXV	VAD
			504RPR
			504RPR
# Transform corrected planetary vector to inertial frame using M matrix.
# This is the core coordinate transformation: R = M^T(T) * (RP + LP x RP)
		VXM	GOTO
			MMATRIX		# MPAC=R=MT(T)*(RP+LPXRP)
			RPRPXXXX	# RESET PUSHLOC TO 0 BEFORE EXITING
# Earth transformation path: Compute Earth orientation matrix and libration.
RPTORA		CALL			# EARTH COMPUTATIONS
			EARTHMX		# M MATRIX B-1
# Compute Earth's libration vector (much smaller than Moon's libration).
		CALL
			EARTHL		# L VECTOR RADIANS B0
# Transform libration to planetary frame: LP = M(T) * L
		MXV	VSL1		# LP=M(T)*L 	RAD B-0
			MMATRIX
# Page 1244
		GOTO
			RPTORB

# Page 1245
# ============================================================================
# TRANSITION: From inertial-to-planetary to planetary-to-inertial conversion
#
# The previous subroutine converted planetary coordinates to inertial frame,
# used when computing trajectories from known surface positions. This inverse
# transformation converts inertial navigation state back to planetary frame,
# essential for displaying position relative to landing sites or Earth surface.
# ============================================================================

# ..... R-TO-RP SUBROUTINE .....
# SUBROUTINE TO CONVERT R (VECTOR IN REFERENCE COORD. SYSTEM) TO RP
# (VECTOR IN PLANETARY COORD SYSTEM) EITHER EARTH-FIXED OR MOON-FIXED
#
# This is the inverse transformation, converting from inertial reference
# frame back to planetary rotating frame. Used for computing spacecraft
# position relative to surface landmarks and landing site coordinates.

#	RP = M(T) * (R - L X R)

# CALLING SEQUENCE
#	L	CALL
#	L+1		R-TO-RP

# SUBROUTINES USED
#	EARTHMX,MOONMX,EARTHL

# INPUT
#	MPAC= 0 FOR EARTH,NON-ZERO FOR MOON
#	0-5D= R VECTOR
#	6-7D= TIME

#	ITEMS AVAILABLE FROM LAUNCH DATA
#		504LM= THE LIBRATION VECTOR L OF THE MOON AT TIME TIMSUBL,EXPRESSED
#		IN THE MOON-FIXED COORD. SYSTEM		RADIANS B0
#			ITEMS NECESSARY FOR SUBROUTINES USED (SEE DESCRIPTION OF SUBR.)

# OUTPUT
#	MPAC=RP VECTOR METERS B-29 FOR EARTH, B-27 FOR MOON

R-TO-RP		STQ	BHIZ
			RPREXIT
			RTORPA
		CALL
			MOONMX
		VLOAD	VXM
			504LM		# LP=LM
			MMATRIX
		VSL1			# L=MT(T)*LP 	RADIANS B0
RTORPB		VXV	BVSU
			504RPR
			504RPR
		MXV			# M(T)*(R-LXR)	B-2
			MMATRIX
RPRPXXXX	VSL1	SETPD
			0D
		GOTO
			RPREXIT
RTORPA		CALL			# EARTH COMPUTATIONS
			EARTHMX
		CALL
			EARTHL
		GOTO			# MPAC=L=(-AX,-AY,0) RAD B-0
			RTORPB

# Page 1246
# ============================================================================
# MOON ORIENTATION MATRIX COMPUTATION
#
# The Moon's orientation relative to inertial space changes due to:
# 1. Rotation about its axis (~27.3 Earth days period)
# 2. Nodal precession (wobble of rotation axis, period ~18.6 years)
# 3. Libration in longitude and latitude (rocking motion visible from Earth)
#
# These time-dependent angles must be computed from ephemeris data and
# combined into the transformation matrix M(T) used for all Moon-relative
# coordinate conversions during lunar landing operations.
# ============================================================================

# ..... MOONMX SUBROUTINE .....
# SUBROUTINE TO COMPUTE THE TRANSFORMATION MATRIX M FOR THE MOON

# CALLING SEQUENCE
#	L	CALL
#	L+1		MOONMX

# SUBROUTINES USED
#	NEWANGLE

# INPUT
#	6-7D= TIME
#	ITEMS AVAILABLE FROM LAUNCH DATA
#		BSUBO,BDOT
#		TIMSUBO,NODIO,NODDOT,FSUBO,FDOT
#		COSI= COS(I)	B-1
#		SINI= SIN(I)	B-1
#		I IS THE ANGLE BETWEEN THE MEAN LUNAR EQUATORIAL PLANE AND THE
#		PLANE OF THE ECLIPTIC (1 DEGREE 32.1 MINUTES)

# OUTPUT
#	MMATRIX= 3X3 M MATRIX B-1	(STORED IN VAC AREA)

MOONMX		STQ	SETPD
			EARTHMXX
			8D
		AXT,1			# B REQUIRES SL 0, SL 5 IN NEWANGLE
			5
		DLOAD	PDDL		# PD 10D	8-9D=BSUBO
			BSUBO		#		10-11D= BDOT
			BDOT
		PUSH	CALL		# PD 12D
			NEWANGLE	# EXIT WITH PD 8D AND MPAC= B	REVS B0
		PUSH	COS		# PD 10D
		STODL	COB		# PD 8D		COS(B) B-1
		SIN			#		SIN(B) B-1
		STODL	SOB		# 		SETUP INPUT FOR NEWANGLE
			FSUBO		# 			8-9D=FSUBO
		PDDL	PUSH		# PD 10D THEN 12D     10-11D=FDOT
			FDOT
		AXT,1	CALL		# F REQUIRES SL 1, SL 6 IN NEWANGLE
			4
			NEWANGLE	# EXIT WITH PD 8D AND MPAC= F REVS B0
		STODL	AVECTR +2	# SAVE F TEMP
			NODIO		#			8-9D=NODIO
		PDDL	PUSH		# PD 10D THEN 12D     10-11D=NODDOT
			NODDOT		#			MPAC=T
		AXT,1	CALL		# NODE REQUIRES SL 0, SL 5 IN NEWANGLE
			5
			NEWANGLE	# EXIT WITH PD 8D AND MPAC= NODI REVS B0
# Page 1247
		PUSH	COS		# PD 10D	8-9D= NODI REVS B0
		PUSH			# PD 12D      10-11D= COS(NODI) B-1
		STORE	AVECTR
		DMP	SL1R
			COB		#			COS(NODI) B-1
		STODL	BVECTR +2	# PD 10D  20-25D=AVECTR=COB*SIN(NODI)
		DMP	SL1R		#			SOB*SIN(NODI)
			SOB
		STODL	BVECTR +4	# PD 8D
		SIN	PUSH		# PD 10D		-SIN(NODI) B-1
		DCOMP			#         26-31D=BVECTR= COB*COS(NODI)
		STODL	BVECTR		# PD 8D			 SOB*COS(NODI)
			AVECTR +2	# MOVE F FROM TEMP LOC. TO 504F
		STODL	504F
		DMP	SL1R
			COB
		STODL	AVECTR +2
			SINNODI		# 8-9D=SIN(NODI) B-1
		DMP	SL1R
			SOB
		STODL	AVECTR +4	#			 0
			HI6ZEROS	#	8-13D= CVECTR= -SOB B-1
		PDDL	DCOMP		# PD 10D		COB
			SOB
		PDDL	PDVL		# PD 12D THEN PD 14D
			COB
			BVECTR
		VXSC	PDVL		# PD 20D	BVECTR*SINI B-2
			SINI
			CVECTR
		VXSC	VAD		# PD 14D	CVECTR*COSI B-2
			COSI
		VSL1
		STOVL	MMATRIX +12D	# PD 8D  M2=BVECTR*SINI+CVECTR*COSI B-1
		VXSC	PDVL		# PD 14D
			SINI		#		CVECTR*SINI B-2
			BVECTR
		VXSC	VSU		# PD 8D		BVECTR*COSI B-2
			COSI
		VSL1	PDDL		# PD 14D
			504F		# 8-13D=DVECTR=BVECTR*COSI-CVECTR*SINI B-1
		COS	VXSC
			DVECTR
		PDDL	SIN		# PD 20D  14-19D= DVECTR*COSF B-2
			504F
		VXSC	VSU		# PD 14D	  AVECTR*SINF B-2
			AVECTR
		VSL1
		STODL	MMATRIX +6	# M1= AVECTR*SINF-DVECTR*COSF B-1
			504F
# Page 1248
		SIN	VXSC		# PD 8D
		PDDL	COS		# PD 14D  8-13D=DVECTR*SINF B-2
			504F
		VXSC	VAD		# PD 8D		AVECTR*COSF B-2
			AVECTR
		VSL1	VCOMP
		STCALL	MMATRIX		# M0= -(AVECTR*COSF+DVECTR*SINF) B-1
			EARTHMXX
# ============================================================================
# TIME-DEPENDENT ANGLE COMPUTATION
#
# Planetary orientation angles change continuously as planets rotate.
# This subroutine extrapolates angles from reference epoch (TIMSUBO) to
# current mission time using angular rates (XDOT).
#
# Used for computing:
# - Earth rotation angle (updates every ~4 minutes for Earth's rotation)
# - Moon's nodal angle (precession of lunar orbit)
# - Lunar rotation angles (for Moon-fixed coordinate transformations)
#
# The computation accounts for both the base angle at epoch (X0) and
# accumulated rotation since epoch (XDOT * elapsed_time). Precise scaling
# is critical as angular rates vary by many orders of magnitude (Earth's
# daily rotation vs. Moon's 18.6-year nodal precession).
# ============================================================================

# COMPUTE X=X0+(XDOT)(T+T0)
# 8-9D= X0 (REVS B-0),PUSHLOC SET AT 12D
# 10-11D=XDOT (REVS/CSEC) SCALED B+23 FOR WEARTH,B+28 FOR NODDOT AND BDOT
#			AND B+27 FOR FDOT
# X1=DIFFERENCE IN 23 AND SCALING OF XDOT,=0 FOR WEARTH,5 FOR NDDOT AND
#					BDOT AND 4 FOR FDOT
# 6-7D=T (CSEC B-28), TIMSUBO= (CSEC B-42 TRIPLE PREC.)

NEWANGLE	DLOAD	SR		# ENTER PD 12D
			6D
			14D
		TAD	TLOAD		# CHANGE MODE TO TP
			TIMSUBO
			MPAC
		STODL	TIMSUBM		# T+T0 CSEC B-42
			TIMSUBM +1
		DMP			# PD 10D	MULT BY XDOT IN 10-11D
		SL*	DAD		# PD 8D		ADD X0 IN 8-9D AFTER SHIFTING
			5,1		#		SUCH THAT SCALING IS B-0
		PUSH	SLOAD		# PD 10D  SAVE PARTIAL (X0+XDOT*T) IN 8-9D
			TIMSUBM
		SL	DMP
			9D
			10D		# XDOT
		SL*	DAD		# PD 8D		SHIFT SUCH THAT THIS PART OF X
			10D,1		#		IS SCALED REVS/CSEC B-0
		BOV			# TURN OFF OVERFLOW IF SET BY SHIFT
			+1		# INSTRUCTION BEFORE EXITING
		RVQ			# MPAC=X= X0+(XDOT)(T+T0)	REVS B0

# Page 1249
# ============================================================================
# EARTH ORIENTATION MATRIX COMPUTATION
#
# Earth's orientation relative to inertial space is dominated by its
# rotation about the polar axis (one rotation every 23h 56m 4s sidereal day).
# The transformation matrix M(T) accounts for:
# 1. Earth's rotation angle at mission time T
# 2. Precession of equinoxes (slow wobble of Earth's axis, ~26,000 year period)
# 3. Nutation (short-period wobble, ~18.6 year period)
#
# Used for:
# - Converting launch site coordinates to inertial frame during ascent
# - Computing Earth-relative position during translunar and transearth coast
# - Entry corridor targeting relative to landing site coordinates
#
# The rotation angle changes by ~15 degrees per hour, requiring frequent
# updates for precise Earth-relative navigation.
# ============================================================================

# ..... EARTHMX SUBROUTINE .....
# SUBROUTINE TO COMPUTE THE TRANSFORMATION MATRIX M FOR THE EARTH

# CALLING SEQUENCE
#	L	CALL
#	L+1		EARTHMX

# SUBROUTINE USED
#	NEWANGLE

# INPUT
#	INPUT AVAILABLE FROM LAUNCH DATA	AZO REVS B-0
#						TEPHEM CSEC B-42
#	6-7D= TIME CSEC B-28

# OUTPUT
#	MMATRIX= 3X3 M MATRIX B-1 (STORED IN VAC AREA)

EARTHMX		STQ	SETPD		# SET 8-9D=AZO
			EARTHMXX
			8D		# 10-11D=WEARTH
		AXT,1			# FOR SL 5, AND SL 10 IN NEWANGLE
			0
		DLOAD	PDDL		# LEAVING PD SET AT 12D FOR NEWANGLE
			AZO
			WEARTH
		PUSH	CALL
			NEWANGLE
		SETPD	PUSH		# 18-19D=504AZ
			18D		#			 COS(AZ) SIN(AZ) 0
		COS	PDDL		# 20-37D=  MMATRIX=	-SIN(AZ) COS(AZ) 0 B-1
			504AZ		#			  0       0      1
		SIN	PDDL
			HI6ZEROS
		PDDL	SIN
			504AZ
		DCOMP	PDDL
			504AZ
		COS	PDVL
			HI6ZEROS
		PDDL	PUSH
			HIDPHALF
		GOTO
			EARTHMXX

# Page 1250
# ============================================================================
# EARTH LIBRATION VECTOR COMPUTATION
#
# The libration vector L accounts for small variations in Earth's rotation
# that deviate from pure rotation about the polar axis:
# 1. Polar motion: Wobble of Earth's rotation axis relative to the crust
#    (approximately 10-20 meters of pole displacement, ~430 day Chandler wobble)
# 2. UT1-UTC variations: Irregularities in Earth's rotation rate due to
#    atmospheric effects, tides, and core-mantle interactions
#
# For Apollo missions, Earth libration is generally much smaller than
# lunar libration because:
# - Earth's rotation is more stable (larger angular momentum)
# - Mission duration is short relative to Earth's irregularity timescales
# - Launch site coordinates (AXO, AYO) set at T0 provide sufficient accuracy
#
# The vector [−AX, −AY, 0] represents the displacement of the instantaneous
# rotation axis from the reference axis at launch time, expressed in radians.
# ============================================================================

# ..... EARTHL SUBROUTINE .....
# SUBROUTINE TO COMPUTE L VECTOR FOR EARTH

# CALLING SEQUENCE
#	L	CALL
#	L+1		EARTHL

# INPUT
#	AXO,AYO SET AT LAUNCH TIME WITH AYO IMMEDIATELY FOLLOWING AXO IN CORE

# OUTPUT
#		-AX
#	MPAC=	-AY	RADIANS B-0
#		  0

EARTHL		DLOAD	DCOMP
			AXO
		STODL	504LPL
			-AYO
		STODL	504LPL +2
			HI6ZEROS
		STOVL	504LPL +4
			504LPL
		RVQ

# Page 1251
# ============================================================================
# PLANETARY ORIENTATION CONSTANTS
#
# These constants define Earth and Moon rotation/orientation parameters
# used throughout coordinate frame transformations. All angular values are
# expressed in REVOLUTIONS (not degrees or radians) for computational efficiency.
#
# SCALING CONVENTIONS:
# - Angles: Revolutions (1 rev = 360° = 2π radians), scaled B-0
# - Angular rates: Revolutions/centisecond, various B-scalings
# - Time: Centiseconds (0.01 second), scaled B-28 or B-42
#
# LUNAR ORBIT GEOMETRY CONSTANTS:
# The Moon's orbit is inclined ~5° to the ecliptic plane. The angle I=5521.5
# seconds of arc (approximately 1.5336°) represents this inclination.
# COS(I) and SIN(I) are precomputed for efficiency in lunar coordinate
# transformations during translunar, lunar orbit, and transearth phases.
#
# LUNAR ORBITAL ELEMENT RATES (Delaunay elements):
# These describe the Moon's complex orbital motion relative to Earth:
# - NODDOT: Rate of change of ascending node (18.6-year regression cycle)
# - FDOT: Rate of change of argument of latitude (Moon's position in orbit)
# - BDOT: Rate of change of mean anomaly (elliptical orbit shape variation)
#
# The Moon's orbit exhibits significant perturbations from Earth's oblateness,
# solar gravity, and other effects requiring these time-dependent corrections.
# ============================================================================

# CONSTANTS AND ERASABLE ASSIGNMENTS
1B1		=	DP1/2		# 1 SCALED B-1
COSI		2DEC	.99964173 B-1	# COS(5521.5 SEC) B-1

SINI		2DEC	.02676579 B-1	# SIN(5521.5 SEC) B-1

; Subroutine exit addresses and return points
RPREXIT		=	S1		# R-TO-RP AND RP-TO-R SUBR EXIT
EARTHMXX	=	S2		# EARTHMX,MOONMX SUBR. EXITS

; Working storage for coordinate transformation computations
504RPR		=	0D		# 6 REGS	R OR RP VECTOR
SINNODI		=	8D		# 2		SIN(NODI)
DVECTR		=	8D		# 6		D VECTOR MOON
CVECTR		=	8D		# 6		C VECTR MOON
504AZ		=	18D		# 2	       AZ
TIMSUBM		=	14D		# 3		TIME SUB M (MOON) T+10 IN GETAZ
504LPL		=	14D		# 6		L OR LP VECTOR
AVECTR		=	20D		# 6		A VECTOR (MOON)
BVECTR		=	26D		# 6		B VECTOR (MOON)
MMATRIX		=	20D		# 18		M MATRIX
COB		=	32D		# 2		COS(B) B-1
SOB		=	34D		# 2		SIN(B) B-1
504F		=	6D		# 2		F(MOON)

; NODDOT: Rate of regression of Moon's ascending node
; The Moon's orbital plane precesses with period 18.6 years (6793 days).
; Negative rate indicates retrograde (westward) motion of node.
; Critical for computing Moon's orientation during lunar orbit operations.
NODDOT		2DEC	-.457335121 E-2	# REVS/CSEC B+28=-1.07047011 E-8  RAD/SEC

; FDOT: Rate of change of Moon's argument of latitude
; Describes Moon's motion along its orbit relative to ascending node.
; Period approximately 27.55 days (anomalistic month).
; Used for computing Moon-fixed coordinates during landing approach.
FDOT		2DEC	.570863327	# REVS/CSEC B+27= 2.67240410 E-6  RAD/SEC

; BDOT: Rate of change of Moon's mean anomaly
; Extremely slow variation in elliptical orbit shape (apsidal precession).
; Period approximately 8.85 years. Smallest rate requires highest scaling (B+28).
BDOT		2DEC	-3.07500686 E-8	# REVS/CSEC B+28=-7.19757301 E-14 RAD/SEC

; NODIO: Initial value of Moon's ascending node at reference epoch
; Angle from reference direction to point where Moon crosses ecliptic northward.
; Updated by NODDOT to compute current node position during mission.
NODIO		2DEC	.986209434	# REVS B-0      = 6.19653663041   RAD

; FSUBO: Initial value of Moon's argument of latitude at reference epoch
; Moon's angular position along orbit measured from ascending node.
; Updated by FDOT for current lunar libration calculations.
FSUBO		2DEC	.829090536	# REVS B-0	= 5.20932947829	  RAD

; BSUBO: Initial value of Moon's mean anomaly at reference epoch
; Related to Moon's position in elliptical orbit (periapsis to current position).
; Updated by BDOT for long-term lunar orientation accuracy.
BSUBO		2DEC	.0651201393	# REVS B=0	= 0.40916190299	  RAD

; WEARTH: Earth's sidereal rotation rate
; One complete rotation every 86164.0905 seconds (23h 56m 4.0905s).
; Equals 7.2921159 × 10^-5 radians/second or 15.041067°/hour.
; Used for Earth-fixed to inertial coordinate transformations during
; launch, Earth orbit, and entry phases. Critical for launch window
; timing and entry corridor targeting relative to recovery ships.
WEARTH		2DEC	.973561595	# REVS/CSEC B+23= 7.29211494 E-5  RAD/SEC
