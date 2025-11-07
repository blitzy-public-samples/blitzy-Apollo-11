# Copyright:	Public domain.
# Filename:	ANGLFIND.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	399-411
# Mod history:	2009-05-09 RSB	Adapted from the Colossus249/ file
#				of the same name, using Comanche055 page
#				images.
#		2009-05-22 RSB	In NOGOM2, TC ZEROEROR corrected to
#				CADR ZEROEROR.
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

# Page 399
; ============================================================================
; FILE: ANGLFIND.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Angle determination algorithms computing spacecraft attitude from IMU
;        gimbal angles. Performs coordinate frame transformations converting
;        platform orientation to body axes enabling attitude knowledge for
;        control and navigation systems throughout Apollo 11.
;
; COMMENT-ONLY READERS: This program calculated which way the spacecraft was
;        pointing from the navigation platform's measurements.
; CODE-ALONG READERS: Study angle determination from gimbal angles, coordinate
;        transformation mathematics, attitude computation algorithms.
; ============================================================================

		BANK	15
		SETLOC	KALCMON1
		BANK

		EBANK=	BCDU

		COUNT	22/KALC

# ============================================================================
# KALCMAN3 - MANEUVER ANGLE AND AXIS COMPUTATION
#
# This routine computes the angle and axis required to rotate the spacecraft
# from its current orientation to a desired target orientation. Used for
# automatic maneuvers during Apollo 11 mission phases including translunar
# coast attitude adjustments, lunar orbit photography passes, and docking
# alignment with the Lunar Module.
#
# The computation determines both how far to rotate (maneuver angle AM) and
# which axis to rotate around (maneuver axis COF). This information drives
# the RCS digital autopilot to execute smooth, fuel-efficient rotations.
# ============================================================================

KALCMAN3	TC	INTPRET
		RTB
			READCDUK	# PICK UP CURRENT CDU ANGLES
		STORE	BCDU		# STORE THE INITIAL S/C ANGLES
		
# The spacecraft's orientation is measured by three gimbals in the IMU
# (Inertial Measurement Unit). These gimbal angles (called CDU angles for
# Coupling Data Unit) must be converted into a mathematical representation
# called a direction cosine matrix (DCM) for computational purposes.

		AXC,2	TLOAD		# COMPUTE THE TRANSFORMATION FROM
			MIS		# INITIAL S/C AXES TO STABLE MEMBER AXES
			BCDU		# (MIS)
		CALL
			CDUTODCM
			
# Now compute the transformation matrix for the desired final orientation.
# CPHI contains the target gimbal angles that define where the spacecraft
# should be pointing after the maneuver completes.

		AXC,2	TLOAD		# COMPUTE THE TRANSFORMATION FROM
			MFS		# FINAL S/C AXES TO STABLE MEMBER AXES
			CPHI		# (MFS)
		CALL
			CDUTODCM

# ============================================================================
# TRANSITION: From orientation matrices to rotation computation
#
# With both current (MIS) and desired (MFS) orientation matrices computed,
# the AGC now calculates the rotation needed to transform from initial to
# final attitude. This involves matrix transposition and multiplication,
# fundamental operations in spacecraft attitude mathematics.
# ============================================================================

SECAD		AXC,1	CALL		# MIS AND MFS ARRAYS CALCULATED		$2
			MIS
			TRANSPOS
			
# Matrix transposition swaps rows and columns. The transpose of the initial
# orientation matrix (TMIS) represents the inverse transformation, converting
# from stable member coordinates back to spacecraft body coordinates.

		VLOAD
		STADR
		STOVL	TMIS +12D
		STADR
		STOVL	TMIS +6
		STADR
		STORE	TMIS		# TMIS = TRANSPOSE(MIS) SCALED BY 2

# Multiplying TMIS (transpose of initial) by MFS (final) produces MFI,
# the matrix representing the net rotation from current to desired attitude.
# This single matrix encodes both the rotation angle and rotation axis.

		AXC,1	AXC,2
			TMIS
			MFS
		CALL
			MXM3
		VLOAD	STADR
		STOVL	MFI +12D
		STADR
		STOVL	MFI +6
		STADR
		STORE	MFI		# MFI = TMIS MFS (SCALED BY 4)
		
# Compute transpose of MFI for subsequent calculations. The transpose will
# be used to extract the skew-symmetric part of the rotation matrix.

		SETPD	CALL		# TRANSPOSE MFI IN PD LIST
			18D
			TRNSPSPD
		VLOAD	STADR
		STOVL	TMFI +12D
		STADR
		STOVL	TMFI +6
# Page 400
		STADR
		STORE	TMFI		# TMFI = TRANSPOSE (MFI)  SCALED BY 4

# ============================================================================
# CALCULATE COFSKEW AND MFISYM
#
# A rotation matrix can be decomposed into symmetric and skew-symmetric parts.
# The skew-symmetric part (COFSKEW) directly reveals the rotation axis.
# For a rotation matrix R, the skew-symmetric part is (R - R^T)/2.
# ============================================================================

# Compute the three components of the skew-symmetric vector by taking
# differences between corresponding off-diagonal elements of MFI and TMFI.
# This vector points along the rotation axis and has magnitude 2*sin(angle).

		DLOAD	DSU
			TMFI +2
			MFI +2
		PDDL	DSU		# CALCULATE COF SCALED BY 2/SIN(AM)
			MFI +4
			TMFI +4
		PDDL	DSU
			TMFI +10D
			MFI +10D
		VDEF
		STORE	COFSKEW		# EQUALS MFISKEW

# CALCULATE AM AND PROCEED ACCORDING TO ITS MAGNITUDE

		DLOAD	DAD
			MFI
			MFI +16D
		DSU	DAD
			DP1/4TH
			MFI +8D
		STORE	CAM		# CAM = (MFI0+MFI4+MFI8-1)/2 HALF SCALE
		ARCCOS
		STORE	AM		# AM=ARCCOS(CAM)  (AM SCALED BY 2)
		DSU	BPL
			MINANG
			CHECKMAX
		EXIT			# MANEUVER LESS THAN 0.25 DEG
		INHINT			# GO DIRECTLY INTO ATTITUDE HOLD
		CS	ONE		# ABOUT COMMANDED ANGLES
		TS	HOLDFLAG	# NOGO WILL STOP ANY RATE AND SET UP FOR A
		TC	LOADCDUD	# GOOD RETURN
		TCF	NOGO

CHECKMAX	DLOAD	DSU
			AM
			MAXANG
		BPL	VLOAD
			ALTCALC		# UNIT
			COFSKEW		# COFSKEW
		UNIT
		STORE	COF		# COF IS THE MANEUVER AXIS
		GOTO			# SEE IF MANEUVER GOES THRU GIMBAL LOCK
			LOCSKIRT

; ============================================================================
; ALTERNATE ANGLE CALCULATION METHOD
; When the maneuver angle exceeds 170 degrees, the standard ARCCOS method
; becomes numerically unstable. This alternate method computes the maneuver
; axis using the symmetric matrix MFISYM and determines the angle from the
; largest component of the cofactor vector, providing stable results for
; large-angle spacecraft rotations.
; ============================================================================

ALTCALC		VLOAD	VAD		# IF AM GREATER THAN 170 DEGREES
			MFI
# Page 401
			TMFI
		VSR1
		STOVL	MFISYM
			MFI +6
		VAD	VSR1
			TMFI +6
		STOVL	MFISYM +6
			MFI +12D
		VAD	VSR1
			TMFI +12D
		STORE	MFISYM +12D	# MFISYM=(MFI+TMFI)/2	SCALED BY 4

; ============================================================================
; COF VECTOR CALCULATION (Alternate Method)
; For large maneuver angles, compute the maneuver axis (COF) from the diagonal
; elements of MFISYM. This calculation uses CAM = cos(AM) where AM is the
; maneuver angle. The cofactor components are extracted from MFISYM diagonal
; terms, providing a numerically stable determination of the rotation axis.
; ============================================================================

# CALCULATE COF

		DLOAD	SR1
			CAM
		PDDL	DSU		# PD0 CAM		 	       $4
			DPHALF
			CAM
		BOVB	PDDL		# PD2 1 - CAM			       $2
			SIGNMPAC
			MFISYM +16D
		DSU	DDV
			0
			2
		SQRT	PDDL		# COFZ = SQRT(MFISYM8-CAM)/(1-CAM)
			MFISYM +8D	#			  	 $ ROOT 2
		DSU	DDV
			0
			2
		SQRT	PDDL		# COFY = SQRT(MFISYM4-CAM)/(1-CAM) $ROOT2
			MFISYM
		DSU	DDV
			0
			2
		SQRT	VDEF		# COFX = SQRT(MFISYM-CAM)/(1-CAM) $ROOT 2
		UNIT
		STORE	COF

; ============================================================================
; MANEUVER AXIS COMPONENT SELECTION
; To maximize numerical accuracy, identify which component (X, Y, or Z) of
; the COF (maneuver axis) vector has the largest absolute value. The spacecraft
; attitude calculation then uses METHOD1, METHOD2, or METHOD3 depending on
; which axis component dominates. This technique minimizes division by small
; numbers and ensures stable gimbal angle computation.
; ============================================================================

# DETERMINE LARGEST COF AND ADJUST ACCORDINGLY

COFMAXGO	DLOAD	DSU
			COF
			COF +2
		BMN	DLOAD		# COFY G COFX
			COMP12
			COF
		DSU	BMN
			COF +4
# Page 402
			METHOD3		# COFZ G COFX OR COFY
		GOTO
			METHOD1		# COFX G COFY OR COFZ
COMP12		DLOAD	DSU
			COF +2
			COF +4
		BMN
			METHOD3		# COFZ G COFY OR COFX

; ============================================================================
; METHOD 2: Y-AXIS DOMINANT COMPONENT
; When the Y component of the COF vector is largest, use this calculation path
; to determine the correct signs of all three axis components. This involves
; checking the signs of COFSKEW and MFISYM elements and complementing COF
; components as needed to ensure the rotation direction matches the physical
; maneuver. This sign adjustment is critical for accurate attitude control.
; ============================================================================

METHOD2		DLOAD	BPL		# COFY MAX
			COFSKEW +2	# UY
			U2POS
		VLOAD	VCOMP
			COF
		STORE	COF
U2POS		DLOAD	BPL
			MFISYM +2	# UX UY
			OKU21
		DLOAD	DCOMP		# SIGN OF UX OPPOSITE TO UY
			COF
		STORE	COF
OKU21		DLOAD	BPL
			MFISYM +10D	# UY UZ
			LOCSKIRT
		DLOAD	DCOMP		# SIGN OF UZ OPPOSITE TO UY
			COF +4
		STORE	COF +4
		GOTO
			LOCSKIRT

; ============================================================================
; METHOD 1: X-AXIS DOMINANT COMPONENT  
; When the X component of the COF vector is largest, use this calculation path.
; Similar to METHOD2, this checks signs of relevant matrix elements (COFSKEW
; and MFISYM) to determine correct orientation of the rotation axis. Ensures
; that the computed maneuver axis correctly represents the spacecraft rotation.
; ============================================================================

METHOD1		DLOAD	BPL		# COFX MAX
			COFSKEW		# UX
			U1POS
		VLOAD	VCOMP
			COF
		STORE	COF
U1POS		DLOAD	BPL
			MFISYM +2	# UX UY
			OKU12
		DLOAD	DCOMP
			COF +2		# SIGN OF UY OPPOSITE TO UX
		STORE	COF +2
OKU12		DLOAD	BPL
			MFISYM +4	# UX UZ
			LOCSKIRT
		DLOAD	DCOMP		# SIGN OF UZ OPPOSITE TO UY
			COF +4
		STORE	COF +4
		GOTO
			LOCSKIRT

; ============================================================================
; METHOD 3: Z-AXIS DOMINANT COMPONENT
; When the Z component of the COF vector is largest, use this calculation path.
; Checks the sign of COFSKEW Z-component and adjusts signs of X and Y components
; based on MFISYM matrix elements. This completes the set of three methods that
; together handle all possible spacecraft orientations with maximum numerical
; stability by always working with the largest available component.
; ============================================================================

METHOD3		DLOAD	BPL		# COFZ MAX
# Page 403
			COFSKEW	+4	# UZ
			U3POS
		VLOAD	VCOMP
			COF
		STORE	COF
U3POS		DLOAD	BPL
			MFISYM +4	# UX UZ
			OKU31
		DLOAD	DCOMP
			COF		# SIGN OF UX OPPOSITE TO UZ
		STORE	COF
OKU31		DLOAD	BPL
			MFISYM +10D	# UY UZ
			LOCSKIRT
		DLOAD	DCOMP
			COF +2		# SIGN OF UY OPPOSITE TO UZ
		STORE	COF +2
		GOTO
			LOCSKIRT

# Page 404

; ============================================================================
; TRANSITION: From COF Calculation to Matrix Operations
;
; With the maneuver axis (COF) determined, control transfers to gimbal lock
; checking routines in GIMBAL_LOCK_AVOIDANCE.agc via LOCSKIRT label.
; The following subroutines provide fundamental matrix operations used
; throughout the angle determination process: MXM3 for matrix multiplication
; and TRANSPOS for matrix transposition. These operations are essential for
; coordinate transformations between spacecraft body axes and stable member
; reference frames.
; ============================================================================

; ============================================================================
; MXM3: 3x3 MATRIX MULTIPLICATION SUBROUTINE
; 
; Multiplies two 3x3 matrices and stores result in pushdown (PD) list.
; Used extensively in coordinate transformations, particularly for computing
; rotation matrices like MFI = TMIS × MFS which represents net spacecraft
; rotation from initial to final attitude.
;
; INPUTS:
;   XR1 - Address of first matrix (9 elements)
;   XR2 - Address of second matrix (9 elements)
;
; OUTPUTS:
;   PD list - Product matrix (9 elements, transposed form for efficiency)
;
; METHOD: Treats second matrix as three column vectors, multiplies first
; matrix by each column vector using MXV instruction, then transposes result.
; This approach leverages AGC's efficient vector-matrix multiply operation.
; ============================================================================

MXM3		SETPD			# MXM3 MULTIPLIES 2 3X3 MATRICES
			0		# AND LEAVES RESULT IN PD LIST
		DLOAD*	PDDL*		# ADDRESS OF 1ST MATRIX IN XR1
			12D,2		# ADDRESS OF 2ND MATRIX IN XR2
			6,2
		PDDL*	VDEF		# DEFINE VECTOR M2(COL 1)
			0,2
		MXV*	PDDL*		# M1XM2(COL 1) IN PD
			0,1
			14D,2
		PDDL*	PDDL*
			8D,2
			2,2
		VDEF	MXV*		# DEFINE VECTOR M2(COL 2)
			0,1
		PDDL*	PDDL*		# M1XM2(COL 2) IN PD
			16D,2
			10D,2
		PDDL*	VDEF		# DEFINE VECTOR M2(COL 3)
			4,2
		MXV*	PUSH		# M1XM2(COL 3) IN PD
			0,1
		GOTO
			TRNSPSPD	# REVERSE ROWS AND COLS IN PD AND
					# RETURN WITH M1XM2 IN PD LIST

; ============================================================================
; TRANSPOS: 3x3 MATRIX TRANSPOSE SUBROUTINE
; 
; Transposes a 3x3 matrix (exchanges rows and columns) and stores result in
; pushdown (PD) list. Used after matrix multiplication and for coordinate
; transformation operations where inverse rotation is needed. Since rotation
; matrices are orthogonal, transpose equals inverse.
;
; INPUTS:
;   XR1 - Address of matrix to transpose (9 elements)
;
; OUTPUTS:
;   PD list - Transposed matrix (9 elements)
;
; METHOD: Loads matrix into PD, then performs in-place swap of off-diagonal
; elements using element exchange pattern: (1,2)↔(2,1), (1,3)↔(3,1), (2,3)↔(3,2).
; Diagonal elements remain unchanged.
; ============================================================================

TRANSPOS	SETPD	VLOAD*		# TRANSPOS TRANSPOSES A 3X3 MATRIX
			0		#  AND LEAVES RESULT IN PD LIST
			0,1		# MATRIX ADDRESS IN XR1
		PDVL*	PDVL*
			6,1
			12D,1
		PUSH			# MATRIX IN PD

; TRNSPSPD: TRANSPOSE MATRIX IN PUSHDOWN LIST
; 
; Variant of TRANSPOS that operates on matrix already stored in PD list.
; Used when matrix is generated in PD during prior calculation and needs
; transposition before next operation. Avoids redundant load operation.

TRNSPSPD	DLOAD	PDDL		# ENTER WITH MATRIX IN PD LIST
			2
			6
		STODL	2
		STADR
		STODL	6
			4
		PDDL
			12D
		STODL	4
		STADR
		STODL	12D
			10D
		PDDL
# Page 405
			14D
		STODL	10D
		STADR
		STORE	14D
		RVQ			# RETURN WITH TRANSPOSED MATRIX IN PD LIST

; ============================================================================
; ANGLE DETERMINATION CONSTANTS
;
; MINANG and MAXANG define operational limits for maneuver angle calculations
; used in attitude determination algorithms. These values control when rotation
; angles are considered significant versus negligible.
; ============================================================================

MINANG		DEC	.00069375	# LOWER LIMIT (~0.04 deg) on maneuver angle
MAXANG		DEC	.472222		# UPPER LIMIT (~27 deg) on maneuver angle

; ============================================================================
; GIMBAL LOCK CONSTANTS
;
; These constants support gimbal lock detection and avoidance. Gimbal lock
; occurs when middle gimbal angle approaches ±90 degrees, causing loss of
; one degree of rotational freedom. Pre-computed trigonometric values enable
; efficient gimbal lock proximity checking without real-time sine/cosine
; computation during critical attitude updates.
;
; D = 60 degrees (MGA corresponding to gimbal lock threshold)
; NGL = 2 degrees (buffer angle to avoid divisions by zero near singularity)
; ============================================================================

SD		DEC	.433015		# = SIN(D) = SIN(60°) ≈ 0.866
K3S1		DEC	.86603		# = SIN(D) duplicate (compatibility)
K4		DEC	-.25		# = -COS(D) = -COS(60°) = -0.5
K4SQ		DEC	.125		# = COS²(D) = 0.25
SNGLCD		DEC	.008725		# = SIN(NGL)·COS(D) for buffer zone
CNGL		DEC	.499695		# = COS(NGL) = COS(2°) ≈ 0.9994
; ============================================================================
; READCDUK: READ CURRENT CDU ANGLES
;
; Reads the current Coupling Data Unit (CDU) gimbal angles from IMU hardware
; registers and loads them into MPAC for subsequent angle determination
; processing. CDU angles represent spacecraft attitude in gimbal coordinates
; (outer, inner, middle gimbal angles).
;
; The CDU angles are read from hardware I/O channels:
;   CDUZ - Z-axis (yaw) gimbal angle
;   CDUX - X-axis (pitch) gimbal angle
;   CDUY - Y-axis (roll) gimbal angle
;
; OUTPUTS:
;   MPAC - Loaded with current CDU angles (CDUZ in MPAC+2, CDUX/Y in MPAC)
;
; INTERRUPTS: Inhibited during read to ensure atomic snapshot of gimbal state.
; Prevents angle inconsistency if gimbal updates occur mid-read.
; ============================================================================

READCDUK	INHINT			# LOAD T(MPAC) WITH THE CURRENT CDU ANGLES
		CA	CDUZ
		TS	MPAC +2
		EXTEND
		DCA	CDUX		# Double precision load CDUX and CDUY
		RELINT
		TCF	TLOAD +6	# Jump to TLOAD+6 to continue processing
		BANK	16
		SETLOC	KALCMON2
		BANK

		COUNT*	$$/KALC

; ============================================================================
; CDUTODCM: CDU ANGLES TO DIRECTION COSINE MATRIX
;
; Converts three CDU gimbal angles into a 3x3 direction cosine matrix (DCM)
; representing spacecraft orientation relative to stable member (IMU platform)
; reference frame. This fundamental transformation enables all spacecraft
; attitude computations and control system operations.
;
; INPUTS:
;   MPAC - Three CDU angles (outer, inner, middle gimbal angles in revolutions)
;   XR2 - Address where resulting DCM will be stored
;
; OUTPUTS:
;   Matrix at address specified by XR2 - 3x3 direction cosine matrix
;
; METHOD: Computes DCM using gimbal rotation sequence. Each gimbal angle
; produces a rotation matrix; these are multiplied in proper sequence to
; produce net transformation from spacecraft body axes to stable member axes.
; Uses interpretive language for efficient matrix-vector operations.
;
; The DCM enables conversion of vectors between coordinate frames:
;   V_platform = DCM × V_spacecraft
; ============================================================================

CDUTODCM	AXT,1	SSP		# SUBROUTINE TO COMPUTE  DIRECTION COSINE
		OCT	3		# MATRIX RELATING S/C AXES TO STABLE
			S1		# MEMBER AXES FROM 3 CDU ANGLES IN T(MPAC)
		OCT	1		# SET XR1, S1 AND PD FOR LOOP
		STORE	7
		SETPD
			0
LOOPSIN		SLOAD*	RTB
			10D,1
			CDULOGIC
		STORE	10D		# LOAD PD WITH 0 SIN(PHI)
		SIN	PDDL		#	       2 COS(PHI)
			10D		#	       4 SIN(THETA)
		COS	PUSH		#	       6 COS(THETA)
		TIX,1	DLOAD		#	       8 SIN(PSI)
			LOOPSIN		#	      10 COS(PSI)
			6
		DMP	SL1
			10D
# Page 406
		STORE	0,2
		DLOAD
			4
		DMP	PDDL
			0		# (PD6 SIN(THETA)SIN(PHI))
			6
		DMP	DMP
			8D
			2
		SL1	BDSU
			12D
		SL1
		STORE	2,2
		DLOAD
			2
		DMP	PDDL		# (PD7 COS(PHI)SIN(THETA)) SCALED 4
			4
			6
		DMP	DMP
			8D
			0
		SL1
		DAD	SL1
			14D
		STORE	4,2
		DLOAD
			8D
		STORE	6,2
		DLOAD
			10D
		DMP	SL1
			2
		STORE	8D,2
		DLOAD
			10D
		DMP	DCOMP
			0
		SL1
		STORE	10D,2
		DLOAD
			4
		DMP	DCOMP
			10D
		SL1
		STORE	12D,2
		DLOAD
		DMP	SL1		# (PUSH UP 7)
			8D
		PDDL	DMP		# (PD7 COS(PHI)SIN(THETA)SIN(PSI)) SCALE4
			6
# Page 407
			0
		DAD	SL1		#  (PUSH UP 7)
		STADR			# C7=COS(PHI)SIN(THETA)SIN(PSI)
		STORE	14D,2
		DLOAD
		DMP	SL1		#  (PUSH UP 6)
			8D
		PDDL	DMP		#  (PD6 SIN(THETA)SIN(PHI)SIN(PSI)) SCALE4
			6
			2
		DSU	SL1		#  (PUSH UP 6)
		STADR
		STORE	16D,2		# C8=-SIN(THETA)SIN(PHI)SIN(PSI)
		RVQ			#  +COS(THETA)COS(PHI)
ENDOCM		EQUALS

		BANK	15
		SETLOC	KALCMON1
		BANK

; ============================================================================
; DELCOMP: COMPUTE ROTATION MATRIX FROM AXIS AND ANGLE
;
; Computes the DEL rotation matrix using Rodrigues' rotation formula, which
; represents rotation by angle A around unit vector U. This fundamental
; transformation converts axis-angle representation (intuitive for spacecraft
; maneuvers) into direction cosine matrix form (efficient for computation).
;
; MATHEMATICAL FORMULA:
;                *      *               --T           *
;   DEL = (IDMATRIX)COS(A) + UU (1-COS(A)) + UX SIN(A)    SCALED 1
;
; Where:
;   U = unit vector (DP scaled 2) along rotation axis [Ux, Uy, Uz]
;   A = rotation angle (DP scaled 2) in revolutions
;   UU^T = outer product of U with itself (dyadic product)
;   UX = cross-product matrix (skew-symmetric matrix of U)
;
; INPUTS:
;   MPAC - Rotation angle A (double precision scaled 2π)
;   COF - Starting address of unit vector U components [Ux, Uy, Uz]
;
; OUTPUTS:
;   DEL - 3x3 rotation matrix (scaled 1) stored as 9 sequential values
;
; METHOD: Computes diagonal terms first (Uᵢ²(1-cos A) + cos A), then
; off-diagonal terms using products of U components, sin A, and (1-cos A).
; Uses efficient double-precision interpretive operations with scaling
; adjustments to maintain numerical precision throughout calculation.
;
; USAGE: Called during attitude determination when computing transformation
; between coordinate frames separated by known rotation axis and angle.
; Essential for spacecraft maneuver calculations and navigation updates.
; ============================================================================

# CALCULATION OF THE MATRIX DEL......
#
#	*      *               --T           *
#	DEL = (IDMATRIX)COS(A)+UU (1-COS(A))+UX SIN(A)		SCALED 1
#
#             -
#	WHERE U IS A UNIT VECTOR (DP SCALED 2) ALONG THE AXIS OF ROTATION.
#	A IS THE ANGLE OF ROTATION (DP SCALED 2)
#					   -
#	UPON ENTRY THE STARTING ADDRESS OF U IS COF, AND A IS IN MPAC

		COUNT	22/KALC

DELCOMP		SETPD	PUSH		# MPAC CONTAINS THE ANGLE A
			0
		SIN	PDDL		# PD0 = SIN(A)
		COS	PUSH		# PD2 = COS(A)
		SR2	PDDL		# PD2 = COS(A)				$8
		BDSU	BOVB		# PD4 = 1-COS(A)			$2
			DPHALF
			SIGNMPAC

# COMPUTE THE DIAGONAL COMPONENTS OF DEL

		PDDL
			COF
		DSQ	DMP
			4
		DAD	SL3
# Page 408
			2
		BOVB
			SIGNMPAC
		STODL	DEL		# UX UX(U-COS(A)) +COS(A)		$1
			COF +2
		DSQ	DMP
			4
		DAD	SL3
			2
		BOVB
			SIGNMPAC
		STODL	DEL +8D		# UY UY(1-COS(A)) +COS(A)		$1
			COF +4
		DSQ	DMP
			4
		DAD	SL3
			2
		BOVB
			SIGNMPAC
		STORE	DEL +16D	# UZ UZ(1-COS(A)) +COS(A)		$1

# COMPUTE THE OFF DIAGONAL TERMS OF DEL

		DLOAD	DMP
			COF
			COF +2
		DMP	SL1
			4
		PDDL	DMP		# D6  UX UY (1-COS A)			$ 4
			COF +4
			0
		PUSH	DAD		# D8  UZ SIN A				$ 4
			6
		SL2	BOVB
			SIGNMPAC
		STODL	DEL +6
		BDSU	SL2
		BOVB
			SIGNMPAC
		STODL	DEL +2
			COF
		DMP	DMP
			COF +4
			4
		SL1	PDDL		# D6  UX UZ (1-COS A )		$ 4
			COF +2
		DMP	PUSH		# D8  UY SIN(A)
			0
		DAD	SL2
			6
# Page 409
		BOVB
			SIGNMPAC
		STODL	DEL +4		# UX UZ (1-COS(A))+UY SIN(A)
		BDSU	SL2
		BOVB
			SIGNMPAC
		STODL	DEL +12D	# UX UZ (U-COS(A))-UY SIN(A)
			COF +2
		DMP	DMP
			COF +4
			4
		SL1	PDDL		# D6  UY UZ (1-COS(A))		$ 4
			COF
		DMP	PUSH		# D6  UX SIN(A)
			0
		DAD	SL2
			6
		BOVB
			SIGNMPAC
		STODL	DEL +14D	# UY UZ(1-COS(A)) +UX SIN(A)
		BDSU	SL2
		BOVB
			SIGNMPAC
		STORE	DEL +10D	# UY UZ (1-COS(A)) -UX SIN(A)
		RVQ

; ============================================================================
; DCMTOCDU: CONVERT DIRECTION COSINE MATRIX TO CDU GIMBAL ANGLES
;
; Extracts three Euler angles (PHI, THETA, PSI) from a direction cosine matrix
; representing spacecraft attitude. This inverse transformation recovers the
; IMU gimbal angles (OGA, IGA, MGA) that Apollo astronauts saw on the DSKY
; display, converting from mathematical rotation matrix back to physical
; gimbal positions.
;
; The direction cosine matrix C relating spacecraft axes to stable member axes
; is parameterized by three successive rotations:
;   OGA (Outer Gimbal Angle) = PHI   - rotation about outer gimbal axis
;   IGA (Inner Gimbal Angle) = THETA - rotation about inner gimbal axis  
;   MGA (Middle Gimbal Angle) = PSI  - rotation about middle gimbal axis
;
; MATHEMATICAL EXTRACTION:
;   PSI (MGA)   = arcsin(C₃)                    extracted from matrix element 3
;   THETA (IGA) = arcsin(-C₆ / cos(PSI))        extracted from elements 6,3
;   PHI (OGA)   = arcsin(-C₅ / cos(PSI))        extracted from elements 5,3
;
; Quadrant resolution checks signs of C₀ and C₄ to determine proper angle
; quadrants, handling the ambiguity inherent in inverse trigonometric functions.
;
; INPUTS:
;   X1 - Complement of starting address for matrix (scaled 2)
;        Points to 9-element direction cosine matrix C₀ through C₈
;
; OUTPUTS:
;   V(MPAC) - Vector of CDU angles [OGA, IGA, MGA] scaled 2π (revolutions)
;   S1 - COS(MGA) retained for subsequent calculations (scaled 1)
;
; GIMBAL LOCK CONSIDERATION: Near ±90° middle gimbal angle, cos(PSI) → 0
; causing numerical instability in arcsin divisions. Calling routines must
; check for gimbal lock conditions before using this conversion.
;
; USAGE: Called after computing attitude transformations to display gimbal
; angles to crew, or when commanding IMU to desired orientation. Essential
; for converting between mathematical attitude representations and physical
; gimbal positions that astronauts monitor during flight.
; ============================================================================

# DIRECTION COSINE MATRIX TO CDU ANGLE ROUTINE
# X1 CONTAINS THE COMPLEMENT OF THE STARTING ADDRESS FOR MATRIX (SCALED 2)
# LEAVES CDU ANGLES SCALED 2PI IN V(MPAC)
# COS(MGA) WILL BE LEFT IN S1 (SCALED 1)
#
# THE DIRECTION COSINE MATRIX RELATING S/C AXES TO STABLE MEMBER AXES CAN BE WRITTEN AS***
#
#	C =COS(THETA)COS(PSI)
#	 0
#	C =-COS(THETA)SIN(PSI)COS(PHI)+SI (THETA)SIN(PHI)
#	 1
#	C =COS(THETA)SIN(PSI)SIN(PHI) + S N(THETA)COS(PHI)
#	 2
#	C =SIN(PSI)
#	 3
#	C =COS(PSI)COS(PHI)
#	 4
#	C =-COS(PSI)SIN(PHI)
#	 5
#	C =-SIN(THETA)COS(PSI)
#	 6
#	C =SIN(THETA)SIN(PSI)COS(PHI)+COS THETA)SIN(PHI)
#	 7
# Page 410
#	C =-SIN(THETA)SIN(PSI)SIN(PHI)+COS(THETA)COS(PHI)
#	 8
#
#	WHERE	PHI = OGA
#		THETA = IGA
#		PSI = MGA

DCMTOCDU	DLOAD*	ARCSIN
			6,1
		PUSH	COS		# PD +0		PSI
		SL1	BOVB
			SIGNMPAC
		STORE	S1
		DLOAD*	DCOMP
			12D,1
		DDV	ARCSIN
			S1
		PDDL*	BPL		# PD +2		THETA
			0,1		# MUST CHECK THE SIGN OF COS(THETA)
			OKTHETA		# TO DETERMINE THE PROPER QUADRANT
		DLOAD	DCOMP
		BPL	DAD
			SUHALFA
			DPHALF
		GOTO
			CALCPHI
SUHALFA		DSU
			DPHALF
CALCPHI		PUSH
OKTHETA		DLOAD*	DCOMP
			10D,1
		DDV	ARCSIN
			S1
		PDDL*	BPL		# PUSH DOWN PHI
			8D,1
			OKPHI
		DLOAD	DCOMP		# PUSH UP PHI
		BPL	DAD
			SUHALFAP
			DPHALF
		GOTO
			VECOFANG
SUHALFAP	DSU	GOTO
			DPHALF
			VECOFANG
OKPHI		DLOAD			# PUSH UP PHI
VECOFANG	VDEF	RVQ

# Page 411
; ============================================================================
; AUTOMATIC MANEUVER TERMINATION ROUTINES
;
; These routines handle abnormal termination of automatic spacecraft maneuvers,
; stopping commanded attitude rates and cleaning up the control system state.
; Called when angle determination or gimbal calculations detect errors that
; prevent safe continuation of the current maneuver.
; ============================================================================

; ----------------------------------------------------------------------------
; NOGOM2: Error termination with zero error call
;
; Entry point accessed when branch-on-zero-minus-or-zero (BZMF) instruction
; detects invalid computation result (NOGO -2 condition). Calls error handler
; to zero error registers before stopping the maneuver.
;
; USAGE: Invoked automatically by BZMF when angle calculation produces zero
; or negative result where positive value expected. Prevents propagation of
; invalid gimbal angles to control system.
; ----------------------------------------------------------------------------
# ROUTINE FOR TERMINATING AUTOMATIC MANEUVERS

NOGOM2		INHINT			# THIS LOCATION ACCESSED BY A BZMF NOGO -2
		TC	BANKCALL
		CADR	ZEROEROR

; ----------------------------------------------------------------------------
; NOGO: Standard maneuver termination
;
; Stops automatic attitude rate commands and schedules maneuver cleanup task.
; Disables interrupts during state transition to ensure atomic termination.
; Delegates final cleanup to ENDMANU routine via WAITLIST scheduling.
;
; PROCEDURE:
;   1. Disable interrupts (INHINT) for atomic state change
;   2. Stop rate commands to RCS jets (STOPRATE)
;   3. Schedule ENDMANU cleanup task with 20ms delay (CAF TWO)
;   4. Exit via ENDOFJOB to release processor
;
; USAGE: Called when maneuver must terminate due to error condition detected
; in angle determination, gimbal lock proximity, or other anomaly requiring
; safe abort of current attitude change. Ensures spacecraft returns to stable
; rate-damped state under crew control.
; ----------------------------------------------------------------------------
NOGO		INHINT
		TC	STOPRATE

					# TERMINATE MANEUVER
		CAF	TWO		# NOTE - ALL RETURNS ARE NOW MADE VIA
		TC	WAITLIST	# GOODEND
		EBANK=	BCDU
		2CADR	ENDMANU

		TCF	ENDOFJOB

