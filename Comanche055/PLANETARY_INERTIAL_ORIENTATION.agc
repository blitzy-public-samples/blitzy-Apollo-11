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

; ============================================================================
; FILE: PLANETARY_INERTIAL_ORIENTATION.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Coordinate frame transformation routines converting between inertial
;        and planetary rotating reference frames. Implements REFSMMAT (Reference
;        Stable Member Matrix) computations enabling consistent coordinate
;        transformations throughout Apollo 11 navigation and guidance operations.
;
; COMMENT-ONLY READERS: This program handled conversions between different
;        coordinate systems used for navigation and control throughout the mission.
; CODE-ALONG READERS: Study coordinate frame transformations, REFSMMAT usage,
;        inertial to rotating frame conversion, transformation matrix mathematics.
; ============================================================================

; ============================================================================
; COORDINATE FRAME FUNDAMENTALS
;
; Apollo navigation requires multiple coordinate reference frames:
;
; INERTIAL FRAME (R vectors): Basic reference coordinate system fixed relative
; to the stars. Used for trajectory calculations and long-term navigation.
; The spacecraft's position and velocity are fundamentally defined in this frame.
;
; PLANETARY ROTATING FRAME (RP vectors): Coordinate system fixed to a planetary
; body (Earth-fixed or Moon-fixed) that rotates with the planet/moon. Used for
; landing site targeting, surface operations, and Earth/Moon-relative navigation.
;
; REFSMMAT (Reference Stable Member Matrix): The transformation matrix M that
; converts between these frames. Different for Earth and Moon due to different
; rotation rates, orientations, and lunar libration effects.
;
; LIBRATION: The Moon's slight wobbling motion as seen from Earth, caused by
; orbital eccentricity and axial tilt. The libration vector L accounts for this
; in Moon transformations, ensuring landing site coordinates remain accurate.
; ============================================================================

# ..... RP-TO-R SUBROUTINE .....
# SUBROUTINE TO CONVERT RP (VECTOR IN PLANETARY COORDINATE SYSTEM,EITHER
# EARTH-FIXED OR MOON-FIXED) TO R (SAME VECTOR IN BASIC REF. SYSTEM)

#	R=MT(T)*(RP+LPXRP)	MT= M MATRIX TRANSPOSE

; RP-TO-R: Planetary Rotating Frame to Inertial Reference Frame Conversion
;
; This subroutine transforms position vectors from a planet/moon-fixed rotating
; coordinate system to the inertial reference frame used for spacecraft trajectory
; computations. During Apollo 11's mission, this conversion was essential for:
; - Processing landing site coordinates (Moon-fixed) into navigation equations
; - Converting ground tracking data (Earth-fixed) to inertial trajectories
; - Integrating IMU measurements with planetary surface references
;
; The transformation accounts for planetary rotation and, for the Moon, libration
; effects. The mathematical form R = M^T(t) * (RP + L x RP) includes:
; - M^T(t): Time-dependent transformation matrix transpose (accounts for rotation)
; - L x RP: Cross product accounting for libration (Moon) or precession (Earth)
;
; For Moon operations, libration vector L (scaled in radians B0) represents the
; wobbling motion visible from Earth. This precision was critical for Apollo 11's
; landing in the Sea of Tranquility, ensuring targeting accuracy despite the Moon's
; complex rotational dynamics.

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

; RP-TO-R Implementation
;
; Entry point discriminates between Earth and Moon transformations based on
; MPAC sign (0=Earth, non-zero=Moon). The computation flow differs because:
; - Earth: Simple rotation, precession effects via L vector
; - Moon: Complex rotation plus libration vector from launch data (504LM)

RP-TO-R		STQ	BHIZ
			RPREXIT
			RPTORA
		CALL			# COMPUTE M MATRIX FOR MOON
			MOONMX		# LP=LM FOR MOON	RADIANS B0
		VLOAD
			504LM
; Common transformation path for both Earth and Moon after M matrix computed.
; Compute the cross product L x RP to account for rotation/libration effects,
; add to original RP vector, then apply transformation matrix.
RPTORB		VXV	VAD		; Form L x RP cross product
			504RPR		; Using RP vector from 0-5D
			504RPR		; Add RP to get (RP + L x RP)
		VXM	GOTO		; Apply M transpose matrix
			MMATRIX		; MPAC=R=MT(T)*(RP+LPXRP)
			RPRPXXXX	; RESET PUSHLOC TO 0 BEFORE EXITING
; Earth-specific transformation path: Compute Earth M matrix and L vector,
; transform L to proper frame, then proceed to common transformation.
RPTORA		CALL			# EARTH COMPUTATIONS
			EARTHMX		# M MATRIX B-1
		CALL
			EARTHL		; L VECTOR RADIANS B0
		MXV	VSL1		; LP=M(T)*L 	RAD B-0
			MMATRIX		; Transform L to rotating frame
# Page 1244
		GOTO
			RPTORB

# Page 1245
# ..... R-TO-RP SUBROUTINE .....
# SUBROUTINE TO CONVERT R (VECTOR IN REFERENCE COORD. SYSTEM) TO RP
# (VECTOR IN PLANETARY COORD SYSTEM) EITHER EARTH-FIXED OR MOON-FIXED

#	RP = M(T) * (R - L X R)

; ============================================================================
; TRANSITION: From Planetary-to-Inertial to Inertial-to-Planetary Conversion
;
; Having transformed planetary coordinates to inertial reference frame, this
; subroutine performs the inverse operation. During Apollo 11's mission, this
; conversion enabled the AGC to:
; - Display spacecraft position in Earth/Moon coordinates for crew awareness
; - Compute landing site approach vectors in Moon-fixed coordinates
; - Generate ground track data for mission control
; ============================================================================

; R-TO-RP: Inertial Reference Frame to Planetary Rotating Frame Conversion
;
; The inverse transformation from RP-TO-R, converting inertial position vectors
; to planet/moon-fixed rotating coordinates. This was essential during Apollo 11
; lunar descent when Armstrong needed to see altitude and range to the landing
; site in Moon-fixed coordinates.
;
; Mathematical form RP = M(t) * (R - L x R) inverts the previous operation:
; - First removes libration/rotation effects via cross product L x R
; - Then applies transformation matrix M(t) (not transpose this time)
; - Result is position in rotating frame moving with planetary surface
;
; During the final approach to the Sea of Tranquility, this conversion provided
; the crew with altitude above the lunar surface and horizontal distance to the
; target landing site, displayed on the DSKY and analog landing displays.

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

; R-TO-RP Implementation
;
; Similar structure to RP-TO-R but applies inverse mathematical operations.
; Key difference: Subtracts L x R instead of adding, uses M instead of M^T.

R-TO-RP		STQ	BHIZ		; Store return, branch if zero (Earth)
			RPREXIT		; Return address saved here
			RTORPA		; Jump to Earth path if MPAC=0
		CALL			; Moon path: Compute Moon M matrix
			MOONMX
		VLOAD	VXM		; Load Moon libration vector
			504LM		; LP=LM from launch data
			MMATRIX		; Transform: L = M^T * LP
		VSL1			; Scale result: L radians B0
; Common path: Compute cross product L x R, subtract from R, apply M matrix.
; This removes the rotation/libration component from inertial vector.
RTORPB		VXV	BVSU		; L x R cross product
			504RPR		; Using R vector from 0-5D
			504RPR		; Subtract: (R - L x R)
		MXV			; Apply M matrix (not transpose)
			MMATRIX		; Result: M(T)*(R-LXR) scaled B-2
RPRPXXXX	VSL1	SETPD		; Scale result and reset push-down list
			0D		; Set PUSHLOC to 0 before exit
		GOTO
			RPREXIT		; Return to caller
; Earth-specific path: Simpler computation without complex libration.
RTORPA		CALL			# EARTH COMPUTATIONS
			EARTHMX		; Get Earth transformation matrix
		CALL
			EARTHL		; Get Earth L vector
		GOTO			# MPAC=L=(-AX,-AY,0) RAD B-0
			RTORPB		; Proceed to common transformation

# Page 1246
# ..... MOONMX SUBROUTINE .....
# SUBROUTINE TO COMPUTE THE TRANSFORMATION MATRIX M FOR THE MOON

; ============================================================================
; MOONMX: Moon Transformation Matrix Computation
;
; Computes the time-dependent transformation matrix M for the Moon, accounting
; for the Moon's complex rotational dynamics. Unlike Earth's relatively simple
; rotation, the Moon exhibits:
; - Synchronous rotation (same face always toward Earth)
; - Libration: Apparent wobbling motion due to orbital eccentricity and axial tilt
; - Inclination of 1°32.1' between lunar equator and ecliptic plane
;
; During Apollo 11's descent to the Sea of Tranquility, accurate computation of
; this matrix was essential for:
; - Converting IMU-measured inertial position to Moon-fixed landing coordinates
; - Maintaining landing site targeting as the Moon's orientation changed
; - Providing crew with altitude and range displays relative to lunar surface
;
; The matrix computation uses orbital parameters from launch data (nodal angles,
; precession rates) and current mission time to account for lunar orientation
; changes since launch. Precision: B-1 scaling (dimensionless direction cosines).
; ============================================================================

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

; MOONMX Implementation: Three-Stage Angle Computation
;
; The Moon transformation matrix requires three time-varying angles computed from
; orbital parameters loaded at launch:
; 1. B angle: Related to Moon's orbital position and rotation
; 2. F angle: Lunar longitude, describing rotation about polar axis
; 3. NODE (NODI): Nodal angle, describing orbital plane orientation
;
; Each angle has an initial value (BSUBO, FSUBO, NODIO) and rate (BDOT, FDOT,
; NODDOT). NEWANGLE subroutine computes current angle = initial + rate*(T-T0).

MOONMX		STQ	SETPD		; Store return address, set push-down list
			EARTHMXX	; Return address (shared with EARTHMX)
			8D		; Initialize PUSHLOC to 8D
		AXT,1			# B REQUIRES SL 0, SL 5 IN NEWANGLE
			5		; Set index register for scaling
		DLOAD	PDDL		# Load B parameters from launch data
			BSUBO		# Initial B angle at TIMSUBO
			BDOT		# B angle rate (revolutions per unit time)
		PUSH	CALL		# Push BDOT, call angle computation
			NEWANGLE	; Returns current B angle in revolutions B0
; Compute trigonometric functions of B angle for matrix construction.
; COS(B) and SIN(B) are fundamental building blocks of transformation matrix.
		PUSH	COS		; Push B angle, compute cosine
		STODL	COB		; Store COS(B) scaled B-1
		SIN			; Compute SIN(B) from same angle
		STODL	SOB		; Store SIN(B) scaled B-1
; Compute F angle (lunar longitude) using same NEWANGLE procedure.
; F angle describes Moon's rotation about its polar axis.
			FSUBO		; Initial F angle at reference time
		PDDL	PUSH		; Push FSUBO onto stack
			FDOT		; F angle rate (rotation rate)
		AXT,1	CALL		; Set scaling index for NEWANGLE
			4		; Different scaling than B angle
			NEWANGLE	; Returns current F angle in revolutions B0
		STODL	AVECTR +2	; Temporarily store F in AVECTR+2
; Compute NODE angle (nodal angle describing orbital plane orientation).
; Critical for accounting for precession of Moon's orbital plane.
			NODIO		; Initial NODE angle at reference time
		PDDL	PUSH		; Push NODIO onto stack
			NODDOT		; NODE angle rate (nodal precession rate)
		AXT,1	CALL		; Set scaling index for NEWANGLE
			5		; Same scaling as B angle
			NEWANGLE	; Returns current NODI angle in revolutions B0
# Page 1247
; ============================================================================
; MATRIX CONSTRUCTION: Building M Matrix from Computed Angles
;
; The transformation matrix M is assembled from four intermediate vectors:
; AVECTR, BVECTR, CVECTR, DVECTR, each constructed from trigonometric
; combinations of the three angles (B, F, NODI). The final 3x3 matrix has
; rows M0, M1, M2 that transform Moon-rotating coordinates to inertial frame.
; ============================================================================
;
; Step 1: Compute trigonometric functions of NODE angle.
; SIN(NODI) needed immediately; COS(NODI) computed and saved for later use.
		PUSH	COS		# PD 10D	8-9D= NODI REVS B0
		PUSH			# PD 12D      10-11D= COS(NODI) B-1
		STORE	AVECTR		; Temporarily store SIN(NODI) in AVECTR
;
; Step 2: Construct BVECTR from NODE and B angle combinations.
; BVECTR components involve products of COS/SIN(NODI) with COS/SIN(B).
		DMP	SL1R		; Multiply SIN(NODI) * COS(B)
			COB		#			COS(NODI) B-1
		STODL	BVECTR +2	# PD 10D  20-25D=AVECTR=COB*SIN(NODI)
		DMP	SL1R		; Multiply SIN(NODI) * SIN(B)
			SOB
		STODL	BVECTR +4	# PD 8D   BVECTR+4 = SOB*SIN(NODI)
		SIN	PUSH		# PD 10D  Compute SIN(NODI) from NODI angle
		DCOMP			; Negate: -SIN(NODI) needed for BVECTR first component
		STODL	BVECTR		# PD 8D   26-31D=BVECTR= COB*COS(NODI)
					;                        SOB*COS(NODI)
					;                       -SIN(NODI)
;
; Step 3: Construct AVECTR from F and B angle combinations.
; Retrieve F angle from temporary storage and compute AVECTR components.
			AVECTR +2	# MOVE F FROM TEMP LOC. TO 504F
		STODL	504F		; Store F angle in proper location
		DMP	SL1R		; Compute COB * SIN(NODI) for AVECTR+2
			COB
		STODL	AVECTR +2	; Store in AVECTR+2
			SINNODI		# 8-9D=SIN(NODI) B-1
		DMP	SL1R		; Compute SOB * SIN(NODI) for AVECTR+4
			SOB
		STODL	AVECTR +4	; Store in AVECTR+4
;
; Step 4: Construct CVECTR (simpler 3-component vector).
; CVECTR has specific structure: (0, -SOB, COB) scaled B-1.
			HI6ZEROS	; Load zero for CVECTR first component
		PDDL	DCOMP		# PD 10D  Negate SIN(B) for second component
			SOB
		PDDL	PDVL		# PD 12D THEN PD 14D  Load COB for third component
			COB
			BVECTR		; Now CVECTR complete: (0, -SOB, COB)
;
; Step 5: Assemble M2 (third row of transformation matrix).
; M2 = BVECTR*SIN(I) + CVECTR*COS(I)
; where I is the inclination angle between lunar equator and ecliptic plane.
		VXSC	PDVL		# PD 20D	BVECTR*SINI B-2
			SINI		; Scale BVECTR by SIN(I) = SIN(1°32.1')
			CVECTR
		VXSC	VAD		# PD 14D	CVECTR*COSI B-2
			COSI		; Scale CVECTR by COS(I), then add to BVECTR*SINI
		VSL1			; Scale result to B-1
		STOVL	MMATRIX +12D	# PD 8D  M2=BVECTR*SINI+CVECTR*COSI B-1
					; M2 now stored as third row of MMATRIX
;
; Step 6: Construct DVECTR as intermediate vector for M0 and M1.
; DVECTR = BVECTR*COS(I) - CVECTR*SIN(I)
		VXSC	PDVL		# PD 14D
			SINI		; CVECTR*SINI B-2
			BVECTR
		VXSC	VSU		# PD 8D		BVECTR*COSI B-2
			COSI		; BVECTR*COS(I) - CVECTR*SIN(I)
		VSL1	PDDL		# PD 14D	Scale to B-1 and push
			504F		# 8-13D=DVECTR=BVECTR*COSI-CVECTR*SINI B-1
;
; Step 7: Assemble M1 (second row of transformation matrix).
; M1 = AVECTR*SIN(F) - DVECTR*COS(F)
; F angle describes Moon's rotation, combining with DVECTR for final orientation.
		COS	VXSC		; Compute COS(F) and scale DVECTR
			DVECTR
		PDDL	SIN		# PD 20D  14-19D= DVECTR*COSF B-2
			504F		; Compute SIN(F) from F angle
		VXSC	VSU		# PD 14D  AVECTR*SINF B-2
			AVECTR		; Scale AVECTR by SIN(F), subtract DVECTR*COS(F)
		VSL1			; Scale result to B-1
		STODL	MMATRIX +6	# M1= AVECTR*SINF-DVECTR*COSF B-1
					; M1 now stored as second row of MMATRIX
			504F		; Reload F angle for M0 computation
# Page 1248
;
; Step 8: Assemble M0 (first row of transformation matrix).
; M0 = -(AVECTR*COS(F) + DVECTR*SIN(F))
; Negative sign accounts for coordinate system handedness convention.
		SIN	VXSC		# PD 8D   Compute SIN(F) and scale DVECTR
		PDDL	COS		# PD 14D  8-13D=DVECTR*SINF B-2
			504F		; Compute COS(F) from F angle
		VXSC	VAD		# PD 8D		AVECTR*COSF B-2
			AVECTR		; Scale AVECTR by COS(F), add to DVECTR*SIN(F)
		VSL1	VCOMP		; Scale to B-1, then negate (complement)
		STCALL	MMATRIX		# M0= -(AVECTR*COSF+DVECTR*SINF) B-1
					; M0 stored as first row. MMATRIX now complete!
					; 3x3 transformation matrix ready for use.
			EARTHMXX

; ============================================================================
; TRANSITION: From Moon Matrix Computation to Angle Update Utility
;
; With MOONMX complete, we now encounter NEWANGLE, a general-purpose utility
; subroutine used by both Moon and Earth transformation computations. NEWANGLE
; calculates the current value of time-varying angles (B, F, NODE for Moon;
; azimuth for Earth) based on an initial value, a rate of change, and the
; elapsed time since epoch. This is essential because planetary bodies rotate
; continuously, requiring constant updates to transformation angles.
; ============================================================================

# COMPUTE X=X0+(XDOT)(T+T0)
# 8-9D= X0 (REVS B-0),PUSHLOC SET AT 12D
# 10-11D=XDOT (REVS/CSEC) SCALED B+23 FOR WEARTH,B+28 FOR NODDOT AND BDOT
#			AND B+27 FOR FDOT
# X1=DIFFERENCE IN 23 AND SCALING OF XDOT,=0 FOR WEARTH,5 FOR NDDOT AND
#					BDOT AND 4 FOR FDOT
# 6-7D=T (CSEC B-28), TIMSUBO= (CSEC B-42 TRIPLE PREC.)

; ============================================================================
; NEWANGLE SUBROUTINE: Time-Varying Angle Computation
;
; PURPOSE: Calculates current angle value using linear propagation formula:
;          X = X0 + (XDOT)(T + T0)
;
; where:  X     = Current angle value (output)
;         X0    = Initial angle at epoch time (input)
;         XDOT  = Rate of change of angle (input)
;         T     = Current time (input)
;         T0    = Epoch reference time (TIMSUBO constant)
;
; This subroutine handles the complex scaling required because different angles
; have different rates of change. Moon's NODE angle changes slowly (complete
; revolution in 18.6 years), while Earth's rotation is much faster (1 rev/day).
;
; INPUTS:
;   8-9D: X0 initial angle value (revolutions, scaled B-0)
;   10-11D: XDOT rate of change (revolutions/centisecond, various scalings)
;   6-7D: T current time (centiseconds, scaled B-28)
;   X1 (index register): Scaling difference correction factor
;
; OUTPUT:
;   MPAC: X current angle (revolutions, scaled B-0)
;
; SCALING NOTES:
;   XDOT scaling varies by angle type to maintain precision:
;   - WEARTH (Earth rotation): B+23
;   - NODDOT, BDOT (Moon angles): B+28
;   - FDOT (Moon F angle): B+27
;   X1 register compensates: 0 for WEARTH, 5 for NODDOT/BDOT, 4 for FDOT
; ============================================================================

NEWANGLE	DLOAD	SR		# ENTER PD 12D
			6D		; Load current time T from register 6D
			14D		; Right shift by 14 bits for scaling alignment
;
; Step 1: Compute total elapsed time (T + T0) in triple precision.
; T0 is the reference epoch time stored in TIMSUBO (B-42 scaling).
		TAD	TLOAD		# CHANGE MODE TO TP (triple precision)
			TIMSUBO		; Add T0 epoch time to current time T
			MPAC		; Load result back into MPAC
		STODL	TIMSUBM		# T+T0 CSEC B-42 (store in TIMSUBM)
			TIMSUBM +1	; Load middle word of triple-precision time
;
; Step 2: Compute high-precision term: XDOT * (middle word of T+T0).
; This handles the bulk of the angle change over mission duration.
		DMP			# PD 10D  Multiply by XDOT rate
			10D		; XDOT in register 10-11D (varies by angle type)
		SL*	DAD		# PD 8D  Shift left by X1 to align scaling
			5,1		; X1 index register provides scaling correction
					; Result now scaled as revolutions B-0
		DAD			; Add initial angle X0 from register 8-9D
					; Partial result: X0 + XDOT*(middle word of T+T0)
		PUSH	SLOAD		# PD 10D  Save partial result, load low word
			TIMSUBM		; Load least significant word of T+T0
;
; Step 3: Compute low-precision term: XDOT * (low word of T+T0).
; This captures fine-grained timing precision for angle accuracy.
		SL	DMP		; Shift low word left 9 bits for alignment
			9D
			10D		; Multiply by XDOT rate (same as before)
		SL*	DAD		# PD 8D  Shift by 10+X1 bits for final scaling
			10D,1		; X1 again provides angle-specific correction
					; Add this correction to partial result
		BOV			; Check for overflow from shift operations
			+1		; TURN OFF OVERFLOW IF SET BY SHIFT
					; INSTRUCTION BEFORE EXITING
		RVQ			; Return with result in MPAC
					; MPAC=X= X0+(XDOT)(T+T0)	REVS B0
					; Current angle now available for matrix computation

# Page 1249

; ============================================================================
; TRANSITION: From General Angle Utility to Earth Matrix Computation
;
; While MOONMX required complex libration calculations with three time-varying
; angles, Earth's transformation is simpler. EARTHMX computes a rotation matrix
; about Earth's polar axis, requiring only one angle: the azimuth (AZ) which
; represents Earth's rotation since the reference epoch. Earth rotates once
; per sidereal day (~23h 56m 4s), making WEARTH one of the fastest angular
; rates in the AGC. The resulting 3x3 matrix transforms between Earth-fixed
; (rotating) and inertial (non-rotating) coordinate frames.
; ============================================================================

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

; ============================================================================
; EARTHMX SUBROUTINE: Earth Transformation Matrix Construction
;
; PURPOSE: Constructs the 3x3 transformation matrix M that converts between
;          Earth-fixed (rotating) and inertial coordinate systems. This is
;          a simple rotation about Earth's polar axis (z-axis) by the current
;          azimuth angle AZ.
;
; MISSION CONTEXT: Throughout Apollo 11's mission, the AGC continuously tracked
; Earth's rotation to maintain accurate navigation fixes. During translunar
; coast, cislunar navigation, and transearth return, Earth-based tracking
; stations required coordinate transformations accounting for Earth's rotation.
;
; MATHEMATICAL FORM: The resulting matrix is a z-axis rotation:
;     M = [  cos(AZ)  sin(AZ)  0 ]
;         [ -sin(AZ)  cos(AZ)  0 ]  (all elements scaled B-1)
;         [    0        0      1 ]
;
; INPUTS:
;   AZO: Initial azimuth at epoch (revolutions, B-0 scaling)
;   WEARTH: Earth rotation rate (revolutions/centisecond, B+23 scaling)
;   6-7D: Current time T (centiseconds, B-28 scaling)
;   TEPHEM: Reference epoch time (centiseconds, B-42 triple precision)
;
; OUTPUT:
;   MMATRIX: 3x3 transformation matrix (B-1 scaling) stored in VAC area
;            9 double-precision words (18-19D through 32-33D)
;
; SCALING NOTES:
;   Earth rotates ~1 revolution per 86164 seconds (sidereal day).
;   WEARTH rate = 1/(86164*100) = 1.16057e-7 revolutions/centisecond.
;   Scaled at B+23, this becomes ~974 integer units, maintaining precision.
; ============================================================================

EARTHMX		STQ	SETPD		# Save return address, set push-down pointer
			EARTHMXX	; Return address stored for later exit
			8D		; Push-down starts at 8D for NEWANGLE setup
;
; Step 1: Set up for NEWANGLE call to compute current azimuth AZ.
; X1=0 because WEARTH uses B+23 scaling (no correction needed in NEWANGLE).
		AXT,1			# FOR SL 5, AND SL 10 IN NEWANGLE
			0		; Index register X1=0 for WEARTH scaling
		DLOAD	PDDL		# LEAVING PD SET AT 12D FOR NEWANGLE
			AZO		; Load initial azimuth from launch data (B-0)
			WEARTH		; Load Earth rotation rate (B+23)
		PUSH	CALL		; Push WEARTH, call angle update routine
			NEWANGLE	; Compute AZ = AZO + WEARTH*(T+TEPHEM)
;
; Step 2: Store computed azimuth and begin matrix construction.
; Earth's transformation is simple rotation about z-axis (polar axis).
		SETPD	PUSH		# 18-19D=504AZ
			18D		; Reset push-down pointer to 18D
					; Current azimuth now in 504AZ (B-0 scaling)
;
; Step 3: Construct first row of MMATRIX: [cos(AZ), sin(AZ), 0]
; This row transforms x-components from inertial to Earth-fixed frame.
		COS	PDDL		# Compute COS(AZ), push, load AZ again
			504AZ		; COS(AZ) now at 20-21D (B-1 scaling)
		SIN	PDDL		# Compute SIN(AZ), push
			HI6ZEROS	; Load zero constant (six words of zeros)
					; SIN(AZ) now at 22-23D (B-1)
					; Third element (z-component) = 0 at 24-25D
;
; Step 4: Construct second row of MMATRIX: [-sin(AZ), cos(AZ), 0]
; This row transforms y-components. Negative sine accounts for right-handed
; coordinate system convention during z-axis rotation.
		PDDL	SIN		# Push zeros, reload AZ and compute SIN(AZ)
			504AZ
		DCOMP	PDDL		# Negate SIN(AZ) for second row first element
			504AZ		; -SIN(AZ) now at 26-27D (B-1)
		COS	PDVL		# Compute COS(AZ), push, load zero vector
			HI6ZEROS	; COS(AZ) at 28-29D, zeros at 30-31D
;
; Step 5: Construct third row of MMATRIX: [0, 0, 1]
; Polar axis (z) unchanged by rotation about itself. Identity for z-component.
		PDDL	PUSH		# Push zero vector
			HIDPHALF	; Load constant 0.5 (represents 1.0 at B-1)
					; Third row: [0, 0, 1] at 32-33D through 36-37D
		GOTO			; Matrix construction complete, exit
			EARTHMXX	; Return to caller via saved address

# Page 1250

; ============================================================================
; TRANSITION: From Earth Matrix to Earth Libration Vector
;
; While EARTHMX computed the primary transformation matrix M accounting for
; Earth's rotation, the RP-TO-R conversion formula requires an additional
; term: R = M^T * (RP + L×RP), where L is the libration vector. For Earth,
; this "libration" is actually a correction for polar motion - the wobble of
; Earth's rotation axis relative to its geographic poles. This wobble is small
; (typically under 1 arcsecond) but significant for precision navigation.
; EARTHL constructs this correction vector from precomputed launch data.
; ============================================================================

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

; ============================================================================
; EARTHL SUBROUTINE: Earth Libration Vector Construction
;
; PURPOSE: Constructs the L vector (libration/polar motion correction) for
;          Earth coordinate transformations. This vector accounts for the
;          wobble of Earth's rotation axis relative to its crust.
;
; MISSION CONTEXT: Earth's rotation axis precesses and nutates over time due
; to gravitational influences from the Moon and Sun. Additionally, polar motion
; causes the instantaneous rotation axis to wander relative to Earth's crust
; by up to 15 meters (about 0.5 arcseconds). For Apollo 11's precision
; navigation requirements, this correction was pre-computed at launch and
; stored as constants AXO and AYO.
;
; MATHEMATICAL FORM: The libration vector is:
;     L = [ -AX ]
;         [ -AY ]  (all components in radians, B-0 scaling)
;         [  0  ]
;
; The negative signs align the correction with the reference frame convention.
; The zero z-component reflects that polar motion is primarily in the
; equatorial plane.
;
; INPUTS:
;   AXO: X-component of polar motion at launch epoch (radians, B-0)
;   AYO: Y-component of polar motion at launch epoch (radians, B-0)
;        Note: AYO stored immediately after AXO in memory (adjacent words)
;
; OUTPUT:
;   MPAC/504LPL: 3-component libration vector (radians, B-0 scaling)
;                Returns pointer to 504LPL location (6 words total)
;
; USAGE: This vector is used in the RP-TO-R transformation formula:
;        R = M^T * (RP + L×RP)
;        where the cross product L×RP provides first-order correction for
;        the rotating reference frame distortion due to polar motion.
;
; SCALING NOTES:
;   Typical values: AXO, AYO ~ 0.4 arcseconds = 0.4/206265 radians ~ 2e-6
;   At B-0 scaling, these become very small fractional values
;   The AGC's 15-bit precision handles this adequately for mission duration
; ============================================================================

EARTHL		DLOAD	DCOMP		; Load AXO (X polar motion component)
			AXO		; and negate it (DCOMP = double complement)
		STODL	504LPL		; Store -AX as first component of L vector
			-AYO		; Load -AYO (Y already negated in storage)
;
; Note on AYO storage: Launch data pre-negated AYO, so loading -AYO gives
; the correct -AY value. This avoids an extra DCOMP operation.
;
		STODL	504LPL +2	; Store -AY as second component (words 2-3)
			HI6ZEROS	; Load constant zero (6 words of zeros)
		STOVL	504LPL +4	; Store 0 as third component (words 4-5)
			504LPL		; Load address of complete L vector
;
; The L vector is now complete: [-AX, -AY, 0] stored in 504LPL location.
; MPAC contains vector address for caller's use in coordinate transformation.
;
		RVQ			; Return to caller with L vector in MPAC

# Page 1251

; ============================================================================
; TRANSITION: From Subroutines to Constants and Memory Assignments
;
; The preceding subroutines (RP-TO-R, R-TO-RP, MOONMX, NEWANGLE, EARTHMX,
; EARTHL) perform coordinate transformations between inertial and planetary
; reference frames. Those calculations require precise physical constants
; describing planetary motion and memory locations for intermediate results.
;
; This section defines:
; 1. Mathematical constants (1B1, COSI, SINI) for coordinate computations
; 2. Erasable memory assignments for vectors and matrices used in transformations
; 3. Physical constants describing lunar libration (NODDOT, FDOT, BDOT, etc.)
; 4. Earth's rotation rate (WEARTH) for inertial-to-rotating transformations
;
; All angle rates (DOT constants) are expressed in revolutions per centisecond,
; matching AGC's internal time representation. Initial angle values (suffix O)
; represent orientation at reference epoch.
; ============================================================================

# CONSTANTS AND ERASABLE ASSIGNMENTS

; ============================================================================
; MATHEMATICAL CONSTANTS FOR COORDINATE TRANSFORMATIONS
; ============================================================================

1B1		=	DP1/2		# 1 SCALED B-1
; Constant representing unity (1.0) with B-1 scaling.
; Equal to DP1/2, where DP1 is double-precision 1.0 scaled B0.
; Used in coordinate transformation calculations where B-1 scaling required.

COSI		2DEC	.99964173 B-1	# COS(5521.5 SEC) B-1
; Cosine of 5521.5 seconds of arc = 1.534°.
; Physical meaning: COS(1.534°) = 0.99964173
; This represents the cosine of the Moon's mean inclination to the ecliptic.
; The value 5521.5 arcseconds = 1°32'1.5" is the standard astronomical
; constant for lunar orbital inclination used in ephemeris calculations.
; Used in MOONMX to construct coordinate transformation matrices accounting
; for the tilt between lunar equator and ecliptic plane.

SINI		2DEC	.02676579 B-1	# SIN(5521.5 SEC) B-1
; Sine of 5521.5 seconds of arc = 1.534°.
; Physical meaning: SIN(1.534°) = 0.02676579
; Complementary to COSI, representing sine of Moon's inclination angle.
; Small value reflects Moon's nearly equatorial orbit relative to ecliptic.
; Used in MOONMX for transformation matrix construction.

; ============================================================================
; ERASABLE MEMORY ASSIGNMENTS FOR COORDINATE TRANSFORMATION VECTORS/MATRICES
;
; These assignments define temporary storage locations in erasable memory
; (RAM) for vectors, matrices, and intermediate values used by coordinate
; transformation subroutines. Many locations are reused (overlapping) by
; different subroutines to conserve the AGC's limited 2K erasable memory.
;
; Naming conventions:
; - 504xxx: Vectors and parameters used across multiple subroutines
; - xxxVECTR: Component vectors (AVECTR, BVECTR, CVECTR, DVECTR) for matrices
; - MMATRIX: Full 3×3 transformation matrix (18 words: 9 double-precision)
;
; Memory conservation strategy: The AGC reuses memory locations for different
; purposes when subroutines don't execute simultaneously. For example, DVECTR
; and CVECTR both use location 8D because MOONMX uses them at different times.
; This aggressive memory reuse was essential for fitting navigation software
; into 2K words of erasable storage.
; ============================================================================

RPREXIT		=	S1		# R-TO-RP AND RP-TO-R SUBR EXIT
; Return address storage for RP-TO-R and R-TO-RP coordinate transformation
; subroutines. Saved by STQ instruction, used by RVQ to return to caller.

EARTHMXX	=	S2		# EARTHMX,MOONMX SUBR. EXITS
; Return address storage for EARTHMX and MOONMX matrix computation subroutines.

504RPR		=	0D		# 6 REGS	R OR RP VECTOR
; Storage for position vector in either inertial (R) or planetary (RP) frame.
; 6 registers = 3 double-precision components (X, Y, Z) of position vector.
; Input to RP-TO-R and output from R-TO-RP coordinate transformations.

SINNODI		=	8D		# 2		SIN(NODI)
; Sine of ascending node angle NODI (computed by MOONMX at current time).
; 2 registers = 1 double-precision value. Used in lunar transformation matrix.

DVECTR		=	8D		# 6		D VECTOR MOON
; D component vector for Moon coordinate transformation matrix construction.
; 6 registers = 3 double-precision components. Note: Overlaps SINNODI and
; CVECTR storage (memory reuse - safe because used at different times).

CVECTR		=	8D		# 6		C VECTR MOON
; C component vector for Moon coordinate transformation matrix construction.
; 6 registers = 3 double-precision components. Constructed from COSI*BVECTR.

504AZ		=	18D		# 2	       AZ
; Earth azimuth angle (sidereal angle from Greenwich meridian to vernal equinox).
; 2 registers = 1 double-precision angle value in revolutions. Computed by EARTHMX
; to determine Earth's rotation angle at specified time for inertial-to-rotating
; coordinate transformations.

TIMSUBM		=	14D		# 3		TIME SUB M (MOON) T+10 IN GETAZ
; Time value for Moon computations, reference epoch for lunar angle calculations.
; 3 registers = 1 triple-precision time value in centiseconds.

504LPL		=	14D		# 6		L OR LP VECTOR
; Libration vector L (Moon) or polar motion vector (Earth) in planetary frame.
; 6 registers = 3 double-precision components. Note: Overlaps TIMSUBM storage.
; Libration represents apparent wobble of Moon as seen from Earth due to orbital
; eccentricity and axial tilt.

AVECTR		=	20D		# 6		A VECTOR (MOON)
; A component vector for Moon coordinate transformation matrix construction.
; First column of transformation matrix M. 6 registers = 3 DP components.

BVECTR		=	26D		# 6		B VECTOR (MOON)
; B component vector for Moon coordinate transformation matrix construction.
; Second column of transformation matrix M. 6 registers = 3 DP components.

MMATRIX		=	20D		# 18		M MATRIX
; Complete 3×3 transformation matrix M for coordinate frame rotations.
; 18 registers = 9 double-precision matrix elements (3 columns × 3 rows).
; Note: Starts at same location as AVECTR - the matrix is built by constructing
; AVECTR, BVECTR, CVECTR as columns, then the full matrix occupies 20D-37D.
; This is the REFSMMAT for planetary coordinate transformations.

COB		=	32D		# 2		COS(B) B-1
; Cosine of B angle (one of three lunar libration angles computed by NEWANGLE).
; 2 registers = 1 double-precision value, B-1 scaling.
; B represents libration in latitude - apparent nodding motion of Moon.

SOB		=	34D		# 2		SIN(B) B-1
; Sine of B angle (one of three lunar libration angles computed by NEWANGLE).
; 2 registers = 1 double-precision value, B-1 scaling.
; Complementary to COB for coordinate transformation calculations.

504F		=	6D		# 2		F(MOON)
; F angle - Moon's rotation angle about its polar axis (computed by NEWANGLE).
; 2 registers = 1 double-precision angle value in revolutions.
; Combined with B and NODI angles to fully describe lunar orientation.

; ============================================================================
; PHYSICAL CONSTANTS FOR LUNAR AND EARTH MOTION
;
; These constants describe the time-varying orientation of the Moon and Earth.
; All angle rates (DOT constants) are expressed in revolutions per centisecond,
; matching AGC's internal time representation where 1 centisecond = 0.01 seconds.
; Initial angle values (suffix O) represent orientation at reference epoch
; (specific date/time when constants were established for Apollo 11 mission).
;
; Lunar libration constants (NODDOT, FDOT, BDOT) describe the apparent wobble
; and rotation of the Moon as observed from Earth. These account for:
; - Regression of lunar nodes (NODDOT) - precession of orbital plane
; - Rotation about lunar axis (FDOT) - Moon's spin synchronized with orbit  
; - Libration in latitude (BDOT) - apparent nodding motion
;
; Scaling notation: B+n or B-n indicates binary scaling factor 2^n
; For angle rates: Multiply by 2π to convert from revolutions/cs to radians/sec
; ============================================================================

NODDOT		2DEC	-.457335121 E-2	# REVS/CSEC B+28=-1.07047011 E-8  RAD/SEC
; Rate of regression of lunar ascending node (NODE angle time derivative).
; Negative value indicates westward regression of orbital plane intersection
; with ecliptic. Physical meaning: The point where Moon crosses ecliptic plane
; moving northward shifts westward at rate of -1.07047011×10^-8 radians/second.
; This 18.6-year period is critical for eclipse prediction and lunar navigation.

FDOT		2DEC	.570863327	# REVS/CSEC B+27= 2.67240410 E-6  RAD/SEC
; Rate of Moon's rotation about its polar axis (F angle time derivative).
; Physical meaning: Moon rotates at 2.67240410×10^-6 radians/second.
; This rate precisely matches lunar orbital period (synchronous rotation),
; causing same face to always point toward Earth. This 27.3-day period
; is fundamental to moon-fixed coordinate transformations.
; Used by NEWANGLE to compute current F angle from reference epoch.

BDOT		2DEC	-3.07500686 E-8	# REVS/CSEC B+28=-7.19757301 E-14 RAD/SEC
; Rate of libration in latitude (B angle time derivative).
; Physical meaning: Apparent nodding motion at -7.19757301×10^-14 radians/second.
; This extremely small rate reflects long-period variations in Moon's orientation
; relative to ecliptic plane. Used by NEWANGLE for precise lunar orientation.

NODIO		2DEC	.986209434	# REVS B-0      = 6.19653663041   RAD
; Initial value of NODE angle at reference epoch (suffix O = "initial" or "zero").
; Value: 0.986209434 revolutions = 354.795397° = 6.19653663 radians.
; Represents ascending node position when Apollo 11 mission constants established.
; Combined with NODDOT to compute current NODE angle at any mission time.

FSUBO		2DEC	.829090536	# REVS B-0	= 5.20932947829	  RAD
; Initial value of F angle (lunar rotation) at reference epoch.
; Value: 0.829090536 revolutions = 298.473° = 5.20932948 radians.
; Represents Moon's rotational position when constants established.
; Combined with FDOT to compute current F angle at any mission time.

BSUBO		2DEC	.0651201393	# REVS B=0	= 0.40916190299	  RAD
; Initial value of B angle (libration in latitude) at reference epoch.
; Value: 0.0651201393 revolutions = 23.443° = 0.40916190 radians.
; Represents initial libration angle when constants established.
; Combined with BDOT to compute current B angle at any mission time.

WEARTH		2DEC	.973561595	# REVS/CSEC B+23= 7.29211494 E-5  RAD/SEC
; Earth's rotation rate (angular velocity of Earth about polar axis).
; Value: 7.29211494×10^-5 radians/second = 360°/23h 56m 4.1s (sidereal day).
; Used by EARTHMX to compute Earth's rotation angle (AZ) at specified time.
; Critical for inertial-to-Earth-fixed coordinate transformations during
; translunar coast, Earth orbit, and reentry phases when Earth orientation
; affects navigation computations and ground tracking visibility.
