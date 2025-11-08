# Copyright:	Public domain.
# Filename:	PLANETARY_INERTIAL_ORIENTATION.agc
# Purpose:	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
#
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim.lawton@gmail.com>
# Website:	www.ibiblio.org/apollo.
# Pages:	1140-1148
# Mod history:	2009-05-28	JVL	Updated from page images.
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
; FILE: PLANETARY_INERTIAL_ORIENTATION.agc
; MODULE: Navigation and Coordinate Transformations
; MISSION PHASE: all phases (launch through re-entry)
;
; TL;DR: Handles coordinate frame transformations between inertial reference
;        frames and planetary rotating frames (Earth-fixed and Moon-fixed).
;        Computes time-dependent rotation matrices accounting for planetary
;        rotation, precession, and lunar libration. Essential for navigation
;        state updates throughout all mission phases.
;
; COMMENT-ONLY READERS: This module enables the AGC to track the spacecraft's
;        position relative to both the rotating Earth/Moon surfaces and the
;        fixed stars used for navigation.
; CODE-ALONG READERS: Study the REFSMMAT (reference stable member matrix)
;        concept and how rotation matrices transform vectors between inertial
;        and body-fixed coordinate systems.
; ============================================================================

# Page 1140
; ============================================================================
; RP-TO-R SUBROUTINE: PLANETARY TO INERTIAL COORDINATE TRANSFORMATION
;
; This subroutine transforms position vectors from planetary coordinate
; systems (rotating with Earth or Moon) to the basic inertial reference
; frame used for navigation. The transformation accounts for planetary
; rotation and libration effects (wobbling motion).
;
; COMMENT-ONLY READERS: The AGC must convert between two reference frames:
;        the spinning planet below and the fixed stars above. This routine
;        performs that essential conversion for navigation calculations.
;
; CODE-ALONG READERS: The transformation equation R = M^T(t) * (RP + L×RP)
;        accounts for both rotation (M matrix) and libration (L vector cross
;        product). Libration causes the Moon to appear to wobble slightly as
;        viewed from Earth, requiring correction in lunar navigation.
; ============================================================================
# ..... RP-TO-R SUBROUTINE .....
# SUBROUTINE TO CONVERT RP (VECTOR IN PLANETARY COORDINATE SYSTEM,EITHER
#  EARTH-FIXED OR MOON-FIXED) TO R (SAME VECTOR IN THE BASIC REF. SYSTEM)
#
#  R=MT(T)*(RP+LPXRP)	 MT= M MATRIX TRANSPOSE
#
# CALLING SEQUENCE
#  L	   CALL
#  L+1		  RP-TO-R
#
# SUBROUTINES USED
#  EARTHMX,MOONMX,EARTHL
#
#    ITEMS AVAILABLE FROM LAUNCH DATA
#     504LM= THE LIBRATION VECTOR L OF THE MOON AT TIME TIMSUBL,EXPRESSED
#     IN THE MOON-FIXED COORD. SYSTEM	RADIANS	 B0
#	ITEMS NECESSARY FOR SUBR. USED (SEE DESCRIPTION OF SUBR.)
#
# INPUT
#  MPAC= 0 FOR EARTH,NON-ZERO FOR MOON
#  0-5D= RP VECTOR
#  6-7D= TIME
#
# OUTPUT
#  MPAC= R VECTOR METERS B-29 FOR EARTH, B-27 FOR MOON

		SETLOC	PLANTIN1
		BANK

		COUNT*	$$/LUROT

; Entry point for RP-TO-R conversion. MPAC input determines planet selection:
; zero for Earth transformations, non-zero for Moon transformations.
RP-TO-R		STQ	BHIZ
			RPREXIT
			RPTORA
; Moon transformation path: Compute Moon's rotation matrix M and libration
; vector LP. The Moon's physical libration causes apparent wobbling, requiring
; correction for accurate lunar surface position calculations during descent.
		CALL			# COMPUTE M MATRIX FOR MOON
			MOONMX		# LP=LM FOR MOON  RADIANS B0
		VLOAD
			504LM
; Compute L×RP (libration cross product with position vector) and add to RP.
; This accounts for the Moon's wobbling motion as it orbits Earth with
; slightly varying rotation rate and axis tilt.
RPTORB		VXV	VAD
			504RPR
			504RPR
; Apply rotation matrix transpose M^T to transform (RP + L×RP) from Moon-fixed
; coordinates to inertial reference frame. Result is position vector R in the
; basic reference system aligned with fixed stars for navigation.
		VXM	GOTO
			MMATRIX		# MPAC=R=MT(T)*(RP+LPXRP)
			RPRPXXXX	# RESET PUSHLOC TO 0 BEFORE EXITING
; Earth transformation path: Similar process for Earth-fixed to inertial
; conversion, accounting for Earth's rotation and polar motion.
RPTORA		CALL			# EARTH COMPUTATIONS
			EARTHMX		# M MATRIX B-1
		CALL
			EARTHL		# L VECTOR RADIANS B0
		MXV	VSL1		# LP=M(T)*L  RAD B-0
			MMATRIX
# Page 1141
		GOTO
			RPTORB
		SETLOC	PLANTIN
		BANK
		COUNT*	$$/LUROT

# Page 1142
; ============================================================================
; R-TO-RP SUBROUTINE: INERTIAL TO PLANETARY COORDINATE TRANSFORMATION
;
; This is the inverse of RP-TO-R, transforming inertial reference vectors
; to planetary rotating reference frames. Used when the AGC needs to express
; position relative to a planet's surface (for landing site targeting, ground
; tracking, or terrain mapping).
;
; COMMENT-ONLY READERS: While RP-TO-R converts from the spinning planet to
;        the fixed stars, R-TO-RP does the reverse - taking positions relative
;        to the stars and expressing them relative to a point on the surface.
;        Critical for targeting specific landing sites on the Moon.
;
; CODE-ALONG READERS: The transformation RP = M(t) * (R - L×R) first removes
;        libration effects (L×R term), then applies rotation matrix M to
;        convert from inertial to body-fixed coordinates. Note this is the
;        transpose operation relative to RP-TO-R.
; ============================================================================
# ..... R-TO-RP SUBROUTINE .....
# SUBROUTINE TO CONVERT R (VECTOR IN REFERENCE COORD. SYSTEM) TO RP
#  (VECTOR IN PLANETARY COORD SYSTEM) EITHER EARTH-FIXED OR MOON-FIXED
#
#  RP=M(T)*(R-LXR)
#
# CALLING SEQUENCE
#  L	   CALL
#  L+1		  R-TO-RP
#
# SUBROUTINES USED
#  EARTHMX,MOONMX,EARTHL
#
# INPUT
#  MPAC= 0 FOR EARTH, NON-ZERO FOR MOON
#  0-5D= R VECTOR
#  6-7D= TIME
#
#    ITEMS AVAILABLE FROM LAUNCH DATA
#     504LM= THE LIBRATION VECTOR L OF THE MOON AT TIME TIMSUBL,EXPRESSED
#     IN THE MOON-FIXED COORD. SYSTEM	RADIANS B0
#	ITEMS NECESSARY FOR SUBROUTINES USED (SEE DESCRIPTION OF SUBR.)
#
# OUTPUT
#  MPAC=RP VECTOR METERS B-29 FOR EARTH, B-27 FOR MOON

; Entry point: Store return address and test MPAC for planet selection.
; Zero selects Earth, non-zero selects Moon for coordinate transformation.
R-TO-RP		STQ	BHIZ
			RPREXIT
			RTORPA
; Moon path: Compute Moon transformation matrix and convert libration vector
; from Moon-fixed coordinates to inertial frame using M^T transformation.
		CALL
			MOONMX
		VLOAD	VXM
			504LM		# LP=LM
			MMATRIX
		VSL1			#  L=MT(T)*LP  RADIANS B0
; Compute L×R and subtract from R to remove libration effects. Then apply
; rotation matrix M to convert from inertial to Moon-fixed coordinates.
; This gives position relative to lunar surface, essential for landing site
; targeting during Apollo 11 descent to the Sea of Tranquility.
RTORPB		VXV	BVSU
			504RPR
			504RPR
		MXV			# M(T)*(R-LXR) B-2
			MMATRIX
; Reset push-down list pointer and exit with result in MPAC.
RPRPXXXX	VSL1	SETPD
			0D
		GOTO
			RPREXIT
; Earth path: Compute Earth transformation matrix and libration vector,
; then proceed with same L×R subtraction and rotation matrix application.
RTORPA		CALL			# EARTH COMPUTATIONS
			EARTHMX
		CALL
			EARTHL
		GOTO			# MPAC=L=(-AX,-AY,0) RAD B-0
			RTORPB

# Page 1143
; ============================================================================
; MOONMX SUBROUTINE: MOON TRANSFORMATION MATRIX COMPUTATION
;
; Computes the 3×3 rotation matrix M that transforms vectors between the
; Moon-fixed coordinate system and the inertial reference frame. Accounts
; for lunar orbital motion, axial rotation, and precession of the Moon's
; equatorial plane relative to the ecliptic.
;
; COMMENT-ONLY READERS: The Moon's rotation is not perfectly uniform - its
;        rotation axis wobbles slightly. This subroutine calculates exactly
;        how the Moon is oriented at any given time, crucial for targeting
;        the Eagle's landing site in the Sea of Tranquility.
;
; CODE-ALONG READERS: The M matrix is computed using Euler angle rotations
;        for the Moon's orbit node angle, inclination (1°32.1'), and rotation
;        angle. Time-dependent angles updated by NEWANGLE subroutine account
;        for orbital motion since reference epoch.
; ============================================================================
# ..... MOONMX SUBROUTINE .....
# SUBROUTINE TO COMPUTE THE TRANSFORMATION MATRIX M FOR THE MOON
#
# CALLING SEQUENCE
#  L	   CALL
#  L+1		  MOONMX
#
# SUBROUTINES USED
#  NEWANGLE
#
# INPUT
#  6-7D= TIME
#    ITEMS AVAILABLE FROM LAUNCH DATA
#     BSUBO,BDOT
#     TIMSUBO,NODIO,NODDOT,FSUBO,FDOT
#     COSI= COS(I) B-1
#     SINI= SIN(I) B-1
#	I  IS THE ANGLE BETWEEN THE MEAN LUNAR EQUATORIAL PLANE AND THE
#	PLANE OF THE ECLIPTIC  (1 DEGREE  32.1 MINUTES)
#
# OUTPUT
#  MMATRIX= 3X3 M MATRIX B-1   (STORED IN VAC AREA)

; Initialize: Store return address and set push-down list pointer to 8D.
; Prepare for three-angle Euler rotation computation (node, inclination, rotation).
MOONMX		STQ	SETPD
			EARTHMXX
			8D
; Compute angle B (lunar rotation angle) using initial value BSUBO and rate
; BDOT from launch data. NEWANGLE calculates B = BSUBO + BDOT*(T-TIMSUBO).
; This angle represents Moon's axial rotation since reference epoch.
		AXT,1			# B REQUIRES SL 0, SL 5 IN NEWANGLE
			5
		DLOAD	PDDL		# PD 10D	    8-9D=BSUBO
			BSUBO		#		    10-11D= BDOT
			BDOT
		PUSH	CALL		# PD 12D
			NEWANGLE	# EXIT WITH PD 8D AND MPAC= B  REVS B0
; Compute cos(B) and sin(B) for rotation matrix elements. Store for later
; use in constructing final 3×3 transformation matrix.
		PUSH	COS		# PD 10D
		STODL	COB		# PD 8D	   COS(B) B-1
		SIN			#	   SIN(B) B-1
		STODL	SOB		#           SETUP INPUT FOR NEWANGLE
			FSUBO		# 		      8-9D=FSUBO
; Compute angle F (argument of perilune - Moon's orbital position angle)
; using FSUBO initial value and FDOT rate. Tracks Moon's position in orbit
; around Earth for accurate transformation between frames.
		PDDL	PUSH		# PD 10D THEN 12D   10-11D=FDOT
			FDOT
		AXT,1	CALL		# F REQUIRES SL 1, SL 6 IN NEWANGLE
			4
			NEWANGLE	# EXIT WITH PD 8D AND MPAC= F REVS B0
		STODL	AVECTR +2	# SAVE F TEMP
			NODIO		#			8-9D=NODIO
; Compute node angle NODI (longitude of ascending node) where Moon's orbital
; plane crosses ecliptic. This angle precesses slowly over time due to
; gravitational perturbations from Earth and Sun.
		PDDL	PUSH		# PD 10D THEN 12D     10-11D=NODDOT
			NODDOT		#			MPAC=T
		AXT,1	CALL		# NODE REQUIRES SL 0, SL 5 IN NEWANGLE
			5
			NEWANGLE	# EXIT WITH PD 8D AND MPAC= NODI REVS B0
# Page 1144
; Compute cos(NODI) and sin(NODI) for node rotation. Begin constructing
; intermediate AVECTR and BVECTR rotation vectors that will form rows of
; the final transformation matrix M.
		PUSH	COS		# PD 10D   8-9D= NODI  REVS  B0
		PUSH			# PD 12D 10-11D= COS(NODI)  B-1
		STORE	AVECTR
		DMP	SL1R
			COB		#			  COS(NODI)    B-1
		STODL	BVECTR +2	# PD 10D   20-25D=AVECTR= COB*SIN(NODI)
		DMP	SL1R		#			  SOB*SIN(NODI)
			SOB
		STODL	BVECTR +4	# PD 8D
; Construct BVECTR components using node angle and rotation angle B.
; BVECTR represents one axis of the transformed coordinate system after
; applying first two Euler rotations (node and inclination).
		SIN	PUSH		# PD 10D		 -SIN(NODI)   B-1
		DCOMP			#          26-31D=BVECTR= COB*COS(NODI)
		STODL	BVECTR		# PD 8D			  SOB*COS(NODI)
			AVECTR +2	# MOVE F FROM TEMP LOC. TO 504F
		STODL	504F
		DMP	SL1R
			COB
		GOTO
			MOONMXA

		BANK	25
		SETLOC	PLANTIN3
		BANK
		COUNT*	$$/LUROT

; Continue constructing AVECTR and CVECTR rotation vectors. CVECTR is
; perpendicular to the lunar equatorial plane, pointing along rotation axis.
MOONMXA		STODL	AVECTR +2
			SINNODI		# 8-9D=SIN(NODI)  B-1
		DMP	SL1R
			SOB
		STODL	AVECTR +4	#			   0
			HI6ZEROS	#	  8-13D= CVECTR= -SOB  B-1
		PDDL	DCOMP		# PD 10D		  COB
			SOB
		PDDL	PDVL		# PD 12D THEN PD 14D
			COB
			BVECTR
; Compute second row M2 of transformation matrix using inclination angle I.
; M2 = BVECTR*sin(I) + CVECTR*cos(I) accounts for 1°32.1' tilt of Moon's
; equator relative to its orbital plane (ecliptic). This wobble causes
; libration in latitude visible from Earth.
		VXSC	PDVL		# PD 20D	 BVECTR*SINI  B-2
			SINI
			CVECTR
		VXSC	VAD		# PD 14D	 CVECTR*COSI  B-2
			COSI
		VSL1
		STOVL	MMATRIX +12D	# PD 8D  M2=BVECTR*SINI+CVECTR*COSI  B-1
; Compute DVECTR (intermediate rotation vector) and third row M3 of matrix.
; DVECTR = BVECTR*cos(I) - CVECTR*sin(I) is perpendicular to M2, forming
; an orthogonal triad with AVECTR and M2.
		VXSC	PDVL		# PD 14D
			SINI		#		 CVECTR*SINI  B-2
			BVECTR
		VXSC	VSU		# PD 8D		 BVECTR*COSI  B-2
			COSI
		VSL1	PDDL		# PD 14D
			504F		# 8-13D=DVECTR=BVECTR*COSI-CVECTR*SINI B-1
		COS	VXSC
# Page 1145
			DVECTR
; Compute first row M1 of transformation matrix using argument of perilune F.
; M1 = AVECTR*sin(F) - DVECTR*cos(F) points toward Moon's orbital position,
; completing the three-axis rotation from inertial to Moon-fixed frame.
		PDDL	SIN		# PD 20D  14-19D= DVECTR*COSF  B-2
			504F
		VXSC	VSU		# PD 14D	  AVECTR*SINF  B-2
			AVECTR
		VSL1
		STODL	MMATRIX +6	# M1= AVECTR*SINF-DVECTR*COSF  B-1
			504F
; Compute zeroth row M0 of transformation matrix. Sign convention requires
; negation to maintain right-handed coordinate system orientation. Final
; M matrix converts inertial coordinates to Moon-fixed coordinates.
		SIN	VXSC		# PD 8D
		PDDL	COS		# PD 14D  8-13D=DVECTR*SINF B-2
			504F
		VXSC	VAD		# PD 8D		AVECTR*COSF B-2
			AVECTR
		VSL1	VCOMP
		STCALL	MMATRIX		# M0= -(AVECTR*COSF+DVECTR*SINF)  B-1
			EARTHMXX

; ============================================================================
; NEWANGLE SUBROUTINE: TIME-DEPENDENT ANGLE COMPUTATION
;
; Computes time-varying astronomical angles using linear ephemeris formula:
; X = X0 + (XDOT)(T + T0), where X0 is angle at reference epoch, XDOT is
; angular rate, T is current time, and T0 is epoch offset.
;
; Used to calculate Moon's rotation angle B, orbital position angle F, and
; ascending node angle NODI, all of which change over time due to lunar
; motion. Handles triple-precision time arithmetic (42-bit time values) and
; multiple scaling factors for different angle types.
;
; COMMENT-ONLY READERS: The Moon's position and orientation change constantly
;        as it orbits Earth and rotates on its axis. This subroutine updates
;        these angles based on how much time has passed since the mission
;        reference epoch, ensuring the landing site coordinates remain accurate.
;
; CODE-ALONG READERS: Uses triple-precision arithmetic for time (CSEC B-42)
;        to maintain accuracy over multi-day missions. Variable scaling via
;        index register X1 handles different angular rate units (B+23 for
;        Earth rotation, B+28 for node/rotation, B+27 for orbital position).
; ============================================================================
# COMPUTE X=X0+(XDOT)(T+T0)
# 8-9D= X0 (REVS B-0),PUSHLOC SET AT 12D
# 10-11D=XDOT (REVS/CSEC) SCALED B+23 FOR WEARTH,B+28 FOR NODDOT AND BDOT
#			  AND B+27 FOR FDOT
#  X1=DIFFERENCE IN 23 AND SCALING OF XDOT,=0 FOR WEARTH,5 FOR NODDOT AND
#					   BDOT AND 4 FOR FDOT
# 6-7D=T (CSEC B-28), TIMSUBO= (CSEC B-42 TRIPLE PREC.)

; Compute elapsed time (T + T0) in triple precision. Input time T at current
; mission epoch added to reference time offset TIMSUBO. Triple precision
; (42 bits) provides centisecond accuracy over entire Apollo mission duration.
NEWANGLE	DLOAD	SR		# ENTER PD 12D
			6D
			14D
; Add T to triple-precision epoch time T0 (TIMSUBO). TAD performs triple-
; precision add, TLOAD retrieves TP result. Result stored in TIMSUBM for
; subsequent use in two-part multiplication (high and low word processing).
		TAD	TLOAD		# CHANGE MODE TO TP
			TIMSUBO
			MPAC
		STODL	TIMSUBM		# T+T0 CSEC B-42
			TIMSUBM +1
; First part: Multiply lower word of (T+T0) by angular rate XDOT. Index
; register X1 provides rate-dependent scaling via SL* instruction. Result
; added to reference epoch angle X0 to form partial result.
		DMP			# PD 10D  MULT BY XDOT IN 10-11D
		SL*	DAD		# PD 8D	  ADD X0 IN 8-9D AFTER SHIFTING
			5,1		#	  SUCH THAT SCALING IS B-0
		PUSH	SLOAD		# PD 10D SAVE PARTIAL (X0+XDOT*T) IN 8-9D
			TIMSUBM
; Second part: Multiply upper word of (T+T0) by XDOT to capture full triple-
; precision time span contribution. Combined with first part via DAD to yield
; complete angular displacement since reference epoch.
		SL	DMP
			9D
			10D		# XDOT
		SL*	DAD		# PD 8D	  SHIFT SUCH THAT THIS PART OF X
			10D,1		#	  IS SCALED REVS/CSEC B-0
; Clear any overflow flag from large shift operations. Final result X in MPAC
; represents current angle (Moon rotation, node position, or orbital position)
; accurate to centisecond resolution over Apollo 11's multi-day flight.
		BOV			# TURN OFF OVERFLOW IF SET BY SHIFT
			+1		# INSTRUCTION BEFORE EXITING
		RVQ			# MPAC=X= X0+(XDOT)(T+T0)  REVS B0

# Page 1146
; ============================================================================
; EARTHMX SUBROUTINE: EARTH TRANSFORMATION MATRIX COMPUTATION
;
; Computes the 3×3 rotation matrix M that transforms vectors between the
; Earth-fixed coordinate system and the inertial reference frame. Accounts
; for Earth's axial rotation since mission reference epoch.
;
; The Earth rotates uniformly at approximately 360°/day (1 revolution per
; sidereal day = 86164.0905 seconds). Matrix M constructed as simple rotation
; about Earth's polar axis (Z-axis rotation by azimuth angle AZ).
;
; COMMENT-ONLY READERS: Earth spins on its axis once per day. This subroutine
;        calculates Earth's current rotational position, crucial for tracking
;        ground stations and updating navigation during translunar coast phase
;        when Mission Control communicated with Apollo 11.
;
; CODE-ALONG READERS: M matrix is standard Z-axis rotation matrix with angle
;        AZ computed from launch epoch angle AZO plus Earth rotation rate
;        WEARTH times elapsed mission time. Matrix elements stored in VAC area
;        (MMATRIX) as row-major 3×3 array scaled B-1.
; ============================================================================
# ..... EARTHMX SUBROUTINE .....
# SUBROUTINE TO COMPUTE THE TRANSFORMATION MATRIX M FOR THE EARTH
#
# CALLING SEQUENCE
#  L	   CALL
#  L+1		  EARTHMX
#
# SUBROUTINE USED
#  NEWANGLE
#
# INPUT
#    INPUT AVAILABLE FROM LAUNCH DATA	  AZO  REVS B-0
#					  TEPHEM  CSEC B-42
#  6-7D= TIME CSEC B-28
#
# OUTPUT
#  MMATRIX= 3X3 M MATRIX B-1   (STORED IN VAC AREA)

		BANK	26
		SETLOC	PLANTIN1
		BANK
		COUNT*	$$/LUROT

; Compute current Earth rotation angle AZ by calling NEWANGLE with launch
; epoch angle AZO and Earth rotation rate WEARTH. Index X1=0 selects proper
; scaling for Earth rotation (B+23 angular rate scaling). Result is azimuth
; angle in revolutions B-0 representing Earth's rotational position.
EARTHMX		STQ	SETPD		# SET	8-9D=AZO
			EARTHMXX
			8D		# 10-11D=WEARTH
		AXT,1			# FOR SL 5, AND SL 10  IN NEWANGLE
			0
		DLOAD	PDDL		#   LEAVING PD SET AT 12D FOR NEWANGLE
			AZO
			WEARTH
		PUSH	CALL
			NEWANGLE
; Construct Z-axis rotation matrix from computed angle AZ. Matrix represents
; transformation from Earth-fixed (rotating) coordinates to inertial (fixed
; stars) reference frame. Matrix form is standard rotation about Z-axis:
;     [ cos(AZ)  sin(AZ)  0 ]
; M = [-sin(AZ)  cos(AZ)  0 ]   scaled B-1
;     [    0        0     1 ]
		SETPD	PUSH		# 18-19D=504AZ
			18D		#		     COS(AZ) SIN(AZ) 0
		COS	PDDL		# 20-37D=  MMATRIX= -SIN(AZ) COS(AZ) 0 B-1
			504AZ		#		      0       0      1
		SIN	PDDL
			HI6ZEROS
; Build matrix row-by-row in pushdown stack, then store in MMATRIX VAC area.
; First row: [cos(AZ), sin(AZ), 0]. Second row: [-sin(AZ), cos(AZ), 0].
; Third row: [0, 0, 1]. During Apollo 11's transearth coast, this matrix
; tracked rotating Earth position for ground station communication windows.
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

# Page 1147
; ============================================================================
; EARTHL SUBROUTINE: EARTH LIBRATION VECTOR COMPUTATION
;
; Computes the L vector (libration/orientation correction vector) for Earth
; coordinate system transformations. This vector represents small perturbations
; in Earth's orientation (polar motion, nutation) relative to the idealized
; rotating reference frame.
;
; The L vector components AX and AY describe Earth's instantaneous rotational
; axis offset from the nominal polar axis, measured in the equatorial plane.
; These values are determined at launch time from astronomical observations
; and remain essentially constant over the Apollo mission duration. Third
; component (Z-axis) is zero since polar motion is perpendicular to spin axis.
;
; COMMENT-ONLY READERS: Earth's rotation axis wobbles slightly due to various
;        effects (polar motion, precession, nutation). This subroutine provides
;        correction values ensuring navigation computations account for Earth's
;        true rotational orientation during ground tracking operations.
;
; CODE-ALONG READERS: Vector constructed as [-AXO, -AYO, 0] in radians B-0.
;        Negation of AXO and AYO aligns with coordinate transformation convention
;        used in RP-TO-R and R-TO-RP subroutines. Result stored in 504LPL VAC
;        location and returned in MPAC for matrix transformation operations.
; ============================================================================
# ..... EARTHL SUBROUTINE .....
# SUBROUTINE TO COMPUTE L VECTOR FOR EARTH
#
# CALLING SEQUENCE
#  L	   CALL
#  L+1		  EARTHL
#
# INPUT
#  AXO,AYO SET AT LAUNCH TIME WITH AYO IMMEDIATELY FOLLOWING AXO IN CORE
#
# OUTPUT
#	    -AX
#   MPAC=   -AY    RADIANS B-0
#	      0

		BANK	06
		SETLOC	EARTHLOC
		BANK
		COUNT*	$$/LUROT

; Load and negate AXO (X-component of Earth orientation correction), store
; in first component of L vector (504LPL). Then load negated AYO value for
; Y-component. Z-component set to zero since polar motion is equatorial plane.
EARTHL		DLOAD	DCOMP
			AXO
		STODL	504LPL
			-AYO
		STODL	504LPL +2
			LO6ZEROS
; Complete L vector stored in 504LPL VAC area and loaded into MPAC for return.
; This vector used in RP-TO-R transformation formula: R = MT(T)*(RP + LP×RP),
; where LP represents libration correction ensuring accurate Earth-fixed to
; inertial coordinate conversions during Apollo 11's translunar flight phase.
		STOVL	504LPL +4
			504LPL
		RVQ

# Page 1148
; ============================================================================
; CONSTANTS AND ERASABLE MEMORY ASSIGNMENTS
;
; This section defines symbolic constants and assigns VAC (vector accumulator)
; storage locations used by the planetary orientation subroutines. These
; definitions map logical variable names to physical pushdown stack locations
; in erasable memory, enabling coordinate transformation calculations without
; explicit address management.
;
; VAC locations are double-precision (DP) register pairs or vector triplets
; in the MPAC (multi-purpose accumulator) pushdown stack. Same physical
; locations serve different logical purposes in different subroutines through
; strategic reuse, conserving precious 2K erasable memory allocation.
;
; COMMENT-ONLY READERS: These symbolic names represent memory locations where
;        intermediate calculation results are temporarily stored during
;        coordinate transformations between Earth/Moon frames and inertial
;        reference frame.
;
; CODE-ALONG READERS: Note overlay strategy—same VAC locations (e.g., 8D)
;        assigned to SINNODI, DVECTR, and CVECTR for different subroutines.
;        This memory reuse pattern is characteristic of AGC programming under
;        severe RAM constraints (2048 words erasable memory total).
; ============================================================================
# CONSTANTS AND ERASABLE ASSIGNMENTS
1B1		=	DP1/2		# 1  SCALED B-1
RPREXIT		=	S1		# R-TO-RP AND RP-TO-R SUBR EXIT
EARTHMXX	=	S2		# EARTHMX,MOONMX SUBR. EXITS
504RPR		=	0D		# 6 REGS  R OR RP VECTOR
SINNODI		=	8D		# 2	  SIN(NODI)
DVECTR		=	8D		# 6	  D VECTOR MOON
CVECTR		=	8D		# 6	  C VECTR MOON
504AZ		=	18D		# 2	 AZ
TIMSUBM		=	14D		# 3	  TIME SUB M (MOON) T+T0 IN GETAZ
504LPL		=	14D		# 6	  L OR LP VECTOR
AVECTR		=	20D		# 6	  A VECTOR (MOON)
BVECTR		=	26D		# 6	  B VECTOR (MOON)
MMATRIX		=	20D		# 18	  M MATRIX
COB		=	32D		# 2	  COS(B) B-1
SOB		=	34D		# 2	  SIN(B) B-1
504F		=	6D		# 2	  F (MOON)
