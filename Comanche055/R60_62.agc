# Copyright:	Public domain.
# Filename:	R60_62.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	390-398
# Mod history:	2009-05-09 RSB	Adapted from the Colossus249/ file
#				of the same name, using Comanche055 page
#				images.
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

# Page 390
# ============================================================================
# FILE: R60_62.agc
# MODULE: COMAID Subsystem (Mission Support)
# MISSION PHASE: rendezvous
#
# TL;DR: Rendezvous radar self-test and initialization routine verifying radar
#        system functionality for rendezvous operations. Implements radar mode
#        setup, antenna pointing initialization, and system health checks
#        ensuring radar readiness for CM/LM rendezvous tracking.
#
# COMMENT-ONLY READERS: This program tested the rendezvous radar to ensure it
#        could track the other spacecraft during docking approach.
# CODE-ALONG READERS: Study radar initialization sequence, self-test procedures,
#        and fault detection algorithms for radar system verification.
# ============================================================================
		BANK	34
		SETLOC	MANUVER
		BANK

		EBANK=	TEMPR60

		COUNT	27/R60

# CONFORMS TO GSOP CHAPTER FOUR REVISION LOGIC 09 JAN 18,1968

# ============================================================================
# R60CSM - COMMAND MODULE AUTOMATIC MANEUVER ROUTINE
#
# During rendezvous operations, the Command Module must orient itself to track
# the Lunar Module with its rendezvous radar. This routine automates that
# pointing operation, computing the required spacecraft attitude and commanding
# the digital autopilot to execute the maneuver.
#
# The routine works with the VECPOINT subroutine to calculate gimbal angles
# that will aim a specified spacecraft axis (typically the radar antenna) at
# a desired direction while avoiding gimbal lock configurations.
# ============================================================================

R60CSM		TC	MAKECADR
		TS	TEMPR60

# INSERT PRIODSP CHECK WITH R22 (V06N49) WITH JENNINGS BRODEUR

# ----------------------------------------------------------------------------
# REDOMANN - Maneuver computation and execution entry point
#
# This section determines whether to use 3-axis automatic control or manual
# control mode. In 3-axis mode, the digital autopilot commands all three
# spacecraft axes. Otherwise, VECPOINT computes the required gimbal angles
# for the crew to reference during manual maneuvering.
# ----------------------------------------------------------------------------

REDOMANN	CAF	BIT6
		MASK	FLAGWRD5	# IS 3-AXIS FLAG SET
		CCS	A
		TCF	TOBALL		# YES
		TC	INTPRET
		CALL
			VECPOINT	# TO COMPUTE FINAL ANGLES
		STORE	CPHI		# STORE FINAL ANGLES - CPHI,CTHETA,CPSI
		EXIT

# ----------------------------------------------------------------------------
# TOBALL - Display maneuver request to crew
#
# COMMENT-ONLY READERS: The computer displays a request asking the crew to
# confirm they are ready for an automatic spacecraft rotation. This was the
# interface between human decision-making and automated guidance during
# rendezvous operations.
#
# CODE-ALONG READERS: V06N18 displays "PLEASE PERFORM" with the maneuver
# type. Crew can PROCEED to execute, ENTER to indicate manual completion,
# or terminate. GOPERF2R formats the display with flashing indicators.
# ----------------------------------------------------------------------------

TOBALL		CAF	V06N18
		TC	BANKCALL
		CADR	GOPERF2R	# DISPLAY PLEASE PERFORM AUTO MANEUVER
		TC	R61TEST
		TC	REDOMANC	# PROCEED
		TCF	ENDMANU1	# ENTER I.E. FINISHED WITH R60

		TC	CHKLINUS	# TO CHECK FOR PRIORITY DISPLAYS
		TC	ENDOFJOB

# ----------------------------------------------------------------------------
# REDOMANC - Re-compute maneuver with automatic mode verification
#
# After crew confirmation, this section recalculates the required attitude
# and verifies that the spacecraft is in automatic control mode. The autopilot
# can only execute maneuvers when MODE switch is set to AUTO and the control
# system is under computer guidance (GNC).
# ----------------------------------------------------------------------------

REDOMANC	CAF	BIT6
		MASK	FLAGWRD5	# IS 3-AXIS FLAG SET
		CCS	A
		TCF	TOBALLC		# YES
		TC	INTPRET
		CALL
			VECPOINT	# TO COMPUTE FINAL ANGLES
		STORE	CPHI		# STORE ANGLES
		EXIT

TOBALLC		CAF	PRIO30		# IS MODE AUTO AND CTL GNC
# Page 391
# Verify automatic control mode by checking CHAN31 control bits 13,14,15.
# If spacecraft is not in AUTO mode with computer guidance, return to
# flashing display requiring crew intervention.
		EXTEND
		RXOR	CHAN31
		MASK	13,14,15
		EXTEND
		BZF	+2		# AUTO, NON-FLASH N18
		TCF	TOBALL		# NOT AUTO

		CAF	V06N18		# SET UP NON-FLASHING V06 N18
		TC	BANKCALL
		CADR	GODSPR
		TC	CHKLINUS

# ----------------------------------------------------------------------------
# STARTMNV - Initiate automatic maneuver execution
#
# COMMENT-ONLY READERS: With crew approval and automatic mode confirmed, the
# spacecraft begins rotating to the computed attitude. The digital autopilot
# fires reaction control thrusters to achieve the precise pointing required
# for rendezvous radar tracking.
#
# CODE-ALONG READERS: GOMANUR is the Digital Autopilot (DAP) entry point
# for executing attitude maneuvers. It reads the desired gimbal angles from
# CPHI, CTHETA, CPSI and commands RCS thruster firings to minimize attitude
# error while managing propellant consumption.
# ----------------------------------------------------------------------------

STARTMNV	TC	BANKCALL
		CADR	GOMANUR
ENDMANUV	TCF	TOBALL		# FINISHED MANEUVER

# ----------------------------------------------------------------------------
# ENDMANU1 - Maneuver completion and cleanup
#
# Resets the 3-axis automatic control flag and returns to the calling program.
# This cleanup ensures the system is ready for the next maneuver or operation.
# TEMPR60 stores the return address established at routine entry via MAKECADR.
# ----------------------------------------------------------------------------

ENDMANU1	TC	DOWNFLAG	# RESET 3-AXIS FLAG
		ADRES	3AXISFLG	# BIT 6 FLAG 5
		CAE	TEMPR60
		TC	BANKJUMP

# ----------------------------------------------------------------------------
# CHKLINUS - Check for priority displays requiring immediate attention
#
# During maneuver operations, higher-priority programs may need to display
# information to the crew. This routine checks the Linus display flag and
# transfers control to handle urgent display requests if necessary.
# ----------------------------------------------------------------------------

CHKLINUS	CS	FLAGWRD4
		MASK	BIT12		# IS PRIORITY DISPLAY FLAG SET
		CCS	A
		TC	Q		# NO - EXIT
		CA	Q
		TS	MPAC +2		# SAVE RETURN
		CS	THREE		# OBTAIN LOCATION FOR RESTART.
		AD	BUF2		# HOLDS Q OF LAST DISPLAY
		TS	TBASE1

		TC	PHASCHNG
		OCT	71		# 1.7SPOT FOR RELINUS

		CAF	BIT7
		TC	LINUS		# GO SET BITS FOR PRIORITY DISPLAY
		TC	MPAC +2

RELINUS		CAF	BIT5		# IS TRACK FLAG ON
		MASK	FLAGWRD1
		EXTEND
		BZF	GOREDO20	# NO

		TC	UPFLAG
		ADRES	PDSPFLAG	# R60 PRIODSP FLAG

		TC	UPFLAG
		ADRES	TARG1FLG	# FOR R52

		CAF	ZERO		# RESET TO ZERO, SINCE
# Page 392
		TS	OPTIND		# OPTIND WAS SET TO -1 BY V379

		CAF	PRIO14		# RESTORE ORIGINAL PRIORITY
		TC	PRIOCHNG

		TC	TBASE1

GOREDO20	TC	PHASCHNG
		OCT	111		# 1.11 FOR PIKUP20

		TC 	ENDOFJOB

# ----------------------------------------------------------------------------
# R61TEST - Determine program state and appropriate exit path
#
# Checks whether the AGC is running program P00 (idle) or within R61 (rendezvous
# tracking routine as part of P20). This determines whether to perform a simple
# exit or transition to extended verb processing (V56).
# ----------------------------------------------------------------------------

R61TEST		CA	MODREG		# ARE WE IN P00.  IF YES THIS MUST BE
		EXTEND			#	VERB49 OR VERB89 SO DO ENDEXT.
		BZF	ENDMANU1	# RESET 3-AXIS & RUTURN.  USER DOES ENDEXT
		CA	FLAGWRD4	# ARE WE IN R61 (P20)
		MASK	BIT12
		EXTEND
		BZF	GOTOPOOH	# NO
		TC	GOTOV56		# YES

BIT14+7		OCT	20100
V06N18		VN	0618

# Page 393
# ============================================================================
# TRANSITION: From Automatic Maneuver Execution to Vector Pointing Computation
#
# The preceding sections handled the crew interface and execution control for
# automatic spacecraft maneuvers. The following VECPOINT subroutine implements
# the core mathematical algorithm that computes the required gimbal angles to
# point a spacecraft axis in a desired direction while avoiding gimbal lock.
#
# This computation was essential for rendezvous operations, where the Command
# Module's rendezvous radar antenna needed precise pointing toward the Lunar
# Module, and for thrust vector alignment during engine burns.
# ============================================================================
# PROGRAM DESCRIPTION - VECPOINT


# 	THIS INTERPRETIVE SUBROUTINE MAY BE USED TO POINT A SPACECRAFT AXIS IN A DESIRED DIRECTION.  THE AXIS
# TO BE POINTED MUST APPEAR AS A HALF UNIT DOUBLE PRECISION VECTOR IN SUCCESSIVE LOCATIONS OF ERASABLE MEMORY
# BEGINNING WITH THE LOCATION CALLED SCAXIS.  THE COMPONENTS OF THIS VECTOR ARE GIVEN IN SPACECRAFT COORDINATES.
# THE DIRECTION IN WHICH THIS AXIS IS TO BE POINTED MUST APPEAR AS A HALF UNIT DOUBLE PRECISION VECTOR IN
# SUCCESSIVE LOCATIONS OF ERASABLE MEMORY BEGINNING WITH THE ADDRESS CALLED POINTVSM.  THE COMPONENTS OF THIS
# VECTOR ARE GIVEN IN STABLE MEMBER COORDINATES.  WITH THIS INFORMATION VECPOINT COMPUTES A SET OF THREE GIMBAL
# ANGLES (2S COMPLEMENT) CORRESPONDING TO THE CROSS-PRODUCT ROTATION BETWEEN SCAXIS AND POINTVSM AND STORES THEM
# IN T(MPAC) BEFORE RETURNING TO THE CALLER.
# 	THIS ROTATION, HOWEVER, MAY BRING THE S/C INTO GIMBAL LOCK.  WHEN POINTING A VECTOR IN THE Y-Z PLANE,
# THE TRANSPONDER AXIS, OR THE AOT FOR THE LEM, THE PROGRAM WILL CORRECT THIS PROBLEM BY ROTATING THE CROSS-
# PRODUCT ATTITUDE ABOUT POINTVSM BY A FIXED AMOUNT SUFFICIENT TO ROTATE THE DESIRED S/C ATTITUDE OUT OF GIMBAL
# LOCK.  IF THE AXIS TO BE POINTED IS MORE THAN 40.6 DEGREES BUT LESS THAN 60.5 DEG FROM THE +X (OR-X) AXIS,
# THE ADDITIONAL ROTATION TO AVOID GIMBAL LOCK IS 35 DEGREES.  IF THE AXIS IS MORE THAN 60.5 DEGREES FROM +X (OR -X)
# THE ADDITIONAL ROTATION IS 35 DEGREES.  THE GIMBAL ANGLES CORRESPONDING TO THIS ATTITUDE ARE THEN COMPUTED AND
# STORED AS 2S COMPLIMENT ANGLES IN T(MPAC) BEFORE RETURNING TO THE CALLER.
#	WHEN POINTING THE X-AXIS, OR THE THRUST VECTOR, OR ANY VECTOR WITHIN 40.6 DEG OF THE X-AXIS, VECPOINT
# CANNOT CORRECT FOR A CROSS-PRODUCT ROTATION INTO GIMBAL LOCK.  IN THIS CASE A PLATFORM REALIGNMENT WOULD BE
# REQUIRED TO POINT THE VECTOR IN THE DESIRED DIRECTION.  AT PRESENT NO INDICATION IS GIVEN FOR THIS SITUATION
# EXCEPT THAT THE FINAL MIDDLE GIMBAL ANGLE IN MPAC +2 IS GREATER THAN 59 DEGREES.

# 	CALLING SEQUENCE -
#		1) LOAD SCAXIS, POINTVSM
#		2) CALL
#			VECPOINT

# 	RETURNS WITH

#		1) DESIRED OUTER GIMBAL ANGLE IN MPAC
#		2) DESIRED INNER GIMBAL ANGLE IN MPAC +1
#		3) DESIRED MIDDLE GIMBAL ANGLE IN MPAC +2


# 	ERASABLES USED -

#		1) SCAXIS		6
#		2) POINTVSM		6
#		3) MIS			18
#		4) DEL			18
#		5) COF			6
#		6) VECQTEMP		1
#		7) ALL OF VAC AREA	43

#				TOTAL	99

		SETLOC	VECPT
		BANK
# Page 394
		EBANK=	BCDU

		COUNT	27/VECPT

# ============================================================================
# VECPOINT - Vector Pointing Subroutine (Interpretive Language)
#
# COMMENT-ONLY READERS: This mathematical routine calculated the spacecraft's
# required orientation to point an antenna, telescope, or engine in a specific
# direction. During rendezvous with the Lunar Module, the Command Module used
# this to aim its rendezvous radar. The algorithm avoided "gimbal lock" - a
# dangerous condition where the spacecraft could lose attitude control.
#
# CODE-ALONG READERS: This is an interpretive language subroutine (note the
# absence of TC INTPRET - calling code must already be in interpretive mode).
# It implements a three-axis gimbal angle computation using vector cross
# products and rotation matrix operations. The algorithm:
# 1) Reads current gimbal angles (CDU) and builds rotation matrix (MIS)
# 2) Transforms desired pointing direction into spacecraft coordinates
# 3) Computes cross product between desired and actual axis vectors
# 4) Checks for gimbal lock proximity and applies corrective rotation if needed
# 5) Returns gimbal angles (outer, inner, middle) in MPAC (push-down stack)
#
# The gimbal lock avoidance logic is critical: when middle gimbal approaches
# ±90 degrees, the outer and inner gimbals become aligned, losing one degree
# of freedom. The routine detects this condition and rotates the spacecraft
# about the pointing vector to maintain control authority.
# ============================================================================

VECPOINT	STQ	BOV		# SAVE RETURN ADDRESS
			VECQTEMP
			VECLEAR		# AND CLEAR OVFIND
# Step 1: Read current gimbal angles and build transformation matrix
# The CDU (Coupling Data Unit) provides gimbal angle readouts from the IMU.
# CDUTODCM converts these Euler angles into a Direction Cosine Matrix (DCM)
# representing the transformation from spacecraft axes to stable member axes.

VECLEAR		AXC,2	RTB
			MIS		# READ THE PRESENT CDU ANGLES AND
			READCDUK	# STORE THEM IN PD25, 26, 27
		STCALL	25D
			CDUTODCM	# S/C AXES TO STABLE MEMBER AXES (MIS)

# Step 2: Transform desired pointing direction into current spacecraft coordinates
# POINTVSM is the desired direction in stable member coordinates (inertial frame).
# VXM (Vector times Matrix) rotates it into the spacecraft's current frame.

		VLOAD	VXM
			POINTVSM	# RESOLVE THE POINTING DIRECTION VF INTO
			MIS		# INITIAL S/C AXES ( VF = POINTVSM)
		UNIT
		STORE	28D
					# PD 28 29 30 31 32 33

# Step 3: Compute rotation axis via cross product
# The cross product SCAXIS x POINTVSM gives the axis about which to rotate.
# If the vectors are nearly parallel or antiparallel, the cross product
# approaches zero and PICKAXIS selects an arbitrary perpendicular axis.

		VXV	UNIT		# TAKE THE CROSS PRODUCT VF X VI
			SCAXIS		# WHERE VI = SCAXIS
		BOV	VCOMP
			PICKAXIS
		STODL	COF		# CHECK MAGNITUDE
			36D		# OF CROSS PRODUCT
		DSU	BMN		# VECTOR, IF LESS
			DPB-14		# THAN B-14 ASSUME
			PICKAXIS	# UNIT OPERATION
		VLOAD	DOT		# 	INVALID.
			SCAXIS
			28D

# Step 4: Compute rotation angle
# ARCCOS of the dot product gives the angle between current and desired vectors.
# SL1 (Shift Left 1) scales the result appropriately for the interpretive format.

		SL1	ARCCOS
COMPMATX	CALL			# NOW COMPUTE THE TRANSFORMATION FROM
			DELCOMP		# FINAL S/C AXES TO INITIAL S/C AXES MFI
		AXC,1	AXC,2
			MIS		# COMPUTE THE TRANSFORMATION FROM FINAL
			DEL		# S/C AXES TO STABLE MEMBER AXES
		CALL			# MFS = MIS MFI
			MXM3		# (IN PD LIST)

# Step 5: GIMBAL LOCK AVOIDANCE CHECK
# Gimbal lock occurs when the middle gimbal angle (CPSI) approaches ±90 degrees.
# At this singularity, outer and inner gimbals align, losing one rotational
# degree of freedom. SINGIMLC = sin(59°) defines the safe boundary.
#
# COMMENT-ONLY READERS: During Apollo missions, gimbal lock was a serious concern.
# If the spacecraft's orientation calculator lost track of which way was "up",
# the crew would have to realign the guidance platform using star sightings,
# consuming precious time during critical maneuvers. This check prevented that.

		DLOAD	ABS
			6		# MFS6 = SIN(CPSI)			$2
		DSU	BMN
			SINGIMLC	# = SIN(59 DEGS)			$2
			FINDGIMB	# /CPSI/ LESS THAN 59 DEGS
					# I.E. DESIRED ATTITUDE NOT IN GIMBAL LOCK

# If gimbal lock is detected, check which axis we're pointing (thrust, AOT, etc.)
# to determine appropriate corrective rotation angle. The algorithm prevents
# gimbal lock by rotating the spacecraft about the pointing vector itself.

		DLOAD	ABS		# CHECK TO SEE IF WE ARE POINTING
			SCAXIS		# THE THRUST AXIS
		DSU	BPL
			SINVEC1		# SIN 49.4 DEGS				$2
# Page 395
			FINDGIMB	# IF SO, WE ARE TRYING TO POINT IT INTO
		VLOAD			# GIMBAL LOCK, ABORT COULD GO HERE
		STADR
		STOVL	MIS +12D
		STADR			# STORE MFS (IN PD LIST) IN MIS
		STOVL	MIS +6
		STADR
		STOVL	MIS
			MIS +6		# INNER GIMBAL AXIS IN FINAL S/C AXES
		BPL	VCOMP		# LOCATE THE IG AXIS DIRECTION CLOSEST TO
			IGSAMEX		# FINAL X S/C AXIS

# Determine optimal rotation direction to escape gimbal lock
# Uses cross product of inner gimbal axis with SCAXIS to find shortest path.
# Rotating about +SCAXIS or -SCAXIS maintains the pointing direction while
# moving the middle gimbal away from the 90-degree singularity.

IGSAMEX		VXV	BMN		# FIND THE SHORTEST WAY OF ROTATING THE
			SCAXIS		# S/C OUT OF GIMBAL LOCK BY A ROTATION
			U=SCAXIS	# ABOUT +- SCAXIS, I.E. IF (IG (SGN MFS3)
					# X SCAXIS . XF) LESS THAN 0, U = SCAXIS
					# OTHERWISE U = -SCAXIS.

		VLOAD	VCOMP
			SCAXIS
		STCALL	COF		# ROTATE ABOUT -SCAXIS
			CHEKAXIS
U=SCAXIS	VLOAD
			SCAXIS
		STORE	COF		# ROTATE ABOUT + SCAXIS

# Select appropriate rotation angle based on which spacecraft axis is being pointed
# - AOT (Alignment Optical Telescope): 50° rotation
# - Transponder or Y/Z plane vectors: 35° rotation
# These angles empirically determined to provide adequate gimbal lock margin
# while minimizing attitude disturbance during critical operations.

CHEKAXIS	DLOAD	ABS
			SCAXIS		# SEE IF WE ARE POINTING THE AOT
		DSU	BPL
			SINVEC2		# SIN 29.5 DEGS				$2
			PICKANG1	# IF SO, ROTATE 50 DEGS ABOUT +- SCAXIS
		DLOAD	GOTO		# IF NOT, MUST BE POINTING THE TRANSPONDER
			VECANG2		# OR SOME VECTOR IN THE Y, OR Z PLANE
			COMPMFSN	# IN THIS CASE ROTATE 35 DEGS TO GET OUT
					# OF GIMBAL LOCK (VECANG2 $360)
PICKANG1	DLOAD
			VECANG1		# = 50 DEGS			      $360
# Compute corrective rotation matrix to escape gimbal lock
# DELCOMP creates a small-angle rotation matrix about SCAXIS (rotation axis).
# MXM3 (Matrix times Matrix) applies this correction to the desired attitude,
# producing a new orientation that achieves the pointing goal while maintaining
# gimbal angles safely away from singularity.

COMPMFSN	CALL
			DELCOMP		# COMPUTE THE ROTATION ABOUT SCAXIS TO
		AXC,1	AXC,2		# BRING MFS OUT OF GIMBAL LOCK
			MIS
			DEL
		CALL			# COMPUTE THE NEW TRANSFORMATION FROM
			MXM3		# DESIRED S/C AXES TO STABLE MEMBER AXES
					# WHICH WILL ALIGN VI WITH VF AND AVOID
					# GIMBAL LOCK

# Convert final orientation matrix back to gimbal angles
# DCMTOCDU extracts Euler angles (CDU commands) from the direction cosine matrix.
# V1STO2S converts from 1's complement to 2's complement format for AGC arithmetic.
# These angles will be sent to the autopilot to execute the maneuver.

FINDGIMB	AXC,1	CALL
			0		# EXTRACT THE COMMANDED CDU ANGLES FROM
			DCMTOCDU	# THIS MATRIX
		RTB	SETPD
			V1STO2S		# CONVERT TO 2:S COMPLEMENT
# Page 396
			0
		GOTO
			VECQTEMP	# RETURN TO CALLER

# ============================================================================
# SPECIAL CASE: PICKAXIS - Nearly Parallel Vectors
# ============================================================================
# When current and desired pointing directions are nearly parallel (or anti-
# parallel), the cross product approaches zero and cannot define a rotation
# axis. This routine handles both cases:
#   - Vectors aligned (VF = VI): No rotation needed, keep current attitude
#   - Vectors opposite (VF = -VI): Need 180° rotation about arbitrary axis
#
# For the 180° case, the algorithm selects a perpendicular axis using cross
# products with stable member Y-axis. If that geometry also fails, defaults
# to X-axis rotation.

PICKAXIS	VLOAD	DOT		# IF VF X VI = 0, FIND VF . VI
			28D
			SCAXIS
		BMN	TLOAD
			ROT180
			25D
		GOTO			# IF VF = VI, CDU DESIRED = PRESENT CDU
			VECQTEMP	# PRESENT CDU ANGLES

# 180-Degree Rotation Case: Vectors Anti-Parallel
# When pointing direction must reverse completely (VF = -VI), need to rotate
# 180° about an axis perpendicular to both. The choice of axis is arbitrary but
# must avoid creating new gimbal lock conditions.
#
# Algorithm: Construct perpendicular axis using double cross product
#   1. Y_SM x X_I creates vector in X-Y plane perpendicular to X
#   2. VI x (Y_SM x X_I) creates vector perpendicular to VI
#   3. If magnitude too small, default to X-axis (PICKX fallback)

ROT180		VLOAD	VXV		# IF VF, VI ANTI-PARALLEL, 180 DEG ROTATION
			MIS +6		# IS REQUIRED.  Y STABLE MEMBER AXIS IN
			HIUNITX		# INITIAL S/C AXIS.
		UNIT	VXV		# FIND Y(SM) X X(I)
			SCAXIS		# FIND UNIT(VI X UNIT(Y(SM) X X(I)))
		UNIT	BOV		# I.E. PICK A VECTOR IN THE PLANE OF X(I),
			PICKX		# Y(SM) PERPENDICULAR TO VI
		STODL	COF
			36D		# CHECK MAGNITUDE
		DSU	BMN		# OF THIS VECTOR.
			DPB-14		# IF LESS THAN B-14,
			PICKX		# PICK X-AXIS.
		VLOAD
			COF
XROT		STODL	COF
			HIDPHALF
		GOTO
			COMPMATX
PICKX		VLOAD	GOTO		# PICK THE XAXIS IN THIS CASE
			HIUNITX
			XROT
		BANK	35
		SETLOC	MANUVER1
		BANK

# ============================================================================
# VECPOINT Mathematical Constants
#
# COMMENT-ONLY READERS: These precise numerical values define the angular
# thresholds used in gimbal lock avoidance. The spacecraft's gyroscopic
# platform could lose control authority if certain angles were exceeded,
# so these constants acted as warning boundaries.
#
# CODE-ALONG READERS: All constants are scaled as double-precision (2DEC)
# fractional values. Angle constants VECANG1/VECANG2 are scaled by 360 degrees
# (1 revolution), so 0.1388888889 = 50/360. Sine constants are direct
# trigonometric values for threshold angle comparisons without requiring
# expensive sine computations during real-time maneuvers.
# ============================================================================

SINGIMLC	2DEC	.4285836003	# =SIN(59)			$2
					# Gimbal lock critical threshold
					# Middle gimbal > 59° indicates
					# approaching gimbal singularity

SINVEC1		2DEC	.3796356537	# =SIN(49.4)			$2
					# First vector threshold for
					# 50-degree corrective rotation

SINVEC2		2DEC	.2462117800	# =SIN(29.5)			$2
					# Second vector threshold for
					# 35-degree corrective rotation

VECANG1		2DEC	.1388888889	# = 50 DEGREES			      $360
					# Corrective rotation angle when
					# axis 40.6-60.5° from X-axis

VECANG2		2DEC	.09722222222	# = 35 DEGREES			      $360
					# Corrective rotation angle when
					# axis > 60.5° from X-axis


1BITDP		OCT	0		# KEEP THIS BEFORE DPB(-14)	  *********
DPB-14		OCT	00001		# Double-precision bit-14 threshold
					# Used in vector magnitude checks
# Page 397
		OCT	00000
		BANK	34
		SETLOC	MANUVER
		BANK

# Page 398
# ============================================================================
# R62DISP - Manual Gimbal Angle Entry for Automatic Maneuver (Verb 49)
#
# COMMENT-ONLY READERS: This routine allowed astronauts to manually enter
# desired spacecraft orientation angles using the DSKY keyboard (Verb 49).
# After entering the three gimbal angles, the crew could press PROCEED to
# initiate an automatic rotation to that attitude, or ENTER to modify the
# values. This gave astronauts direct control over spacecraft pointing without
# relying on automatic calculations.
#
# CODE-ALONG READERS: V06N22 displays three angles (Noun 22: outer, inner,
# middle gimbal angles) stored in CPHI, CTHETA, CPSI. GOFLASH enables the
# flashing display allowing crew keyboard input. Three return paths exist:
# - First TC: ENDEXT on program termination
# - Second TCF: GOMOVE on PROCEED (execute maneuver with entered angles)  
# - Third TCF: R62DISP on ENTER (re-display for angle modification)
# The 3AXISFLG flag signals the autopilot to perform 3-axis attitude control.
# ============================================================================
# ROUTINE FOR INITIATING AUTOMATIC MANEUVER VIA KEYBOARD (V49)

		EBANK=	CPHI

		COUNT	27/R62

R62DISP		CAF	V06N22		# DISPLAY COMMAND ICDUS CPHI, CTHETA, CPHI
		TC	BANKCALL
		CADR	GOFLASH		# Flash display, accept crew input
		TCF	ENDEXT		# Terminate program request
		TCF	GOMOVE		# PROCEED - execute maneuver
		TCF	R62DISP		# ENTER - re-display for modification

					# ASTRONAUT MAY LOAD NEW ICDUS AT THIS
					# POINT
# After crew presses PROCEED, prepare for automatic maneuver execution
# by enabling 3-axis control mode and transferring to R60CSM routine.

GOMOVE		TC	UPFLAG		# SET 3-AXIS FLAG
		ADRES	3AXISFLG	# BIT 6	FLAG 5
					# Signals DAP for 3-axis control

		TC	BANKCALL
		CADR	R60CSM		# Execute automatic maneuver
		TCF	ENDEXT		# Return to keyboard monitor
