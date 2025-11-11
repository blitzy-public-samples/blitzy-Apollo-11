# Copyright:    Public domain.
# Filename:     RT8_OP_CODES.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1508-1516
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-07 RSB	Adapted from Colossus249/RT8_OP_CODES.agc
#				and page images.
#		2009-05-07 RSB	Oops! Left out the entire last page before.
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#       Assemble revision 055 of AGC program Comanche by NASA
#       2021113-051.  April 1, 1969.
#
#       This AGC program shall also be referred to as Colossus 2A
#
#       Prepared by
#                       Massachusetts Institute of Technology
#                       75 Cambridge Parkway
#                       Cambridge, Massachusetts
#
#       under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: RT8_OP_CODES.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Interpreter opcode implementations for RTB (Return to Bank) operations
;        and special function implementations. Extends interpretive language
;        with specialized operations called via RTB instruction throughout
;        Apollo 11 guidance and navigation calculations.
;
; COMMENT-ONLY READERS: The guidance computer's high-level language needed
;        special operations for complex calculations. This code extends the
;        computer's capabilities for navigation, sensor processing, and
;        mathematical operations during all mission phases.
; CODE-ALONG READERS: Study RTB (Return to Bank) opcode implementations.
;        These special functions are callable from interpretive code via RTB
;        instruction, providing utilities for angle conversions, sensor
;        reading, matrix operations, and coordinate transformations.
; ============================================================================

# Page 1508
		BANK	22
		SETLOC	RTBCODES
		BANK

		EBANK=	XNB
		COUNT*	$$/RTB

; ============================================================================
; RTB SPECIAL FUNCTIONS
;
; The following routines are callable from interpretive code using the RTB
; (Return to Bank) instruction. Each function performs specialized operations
; extending the interpreter's capabilities beyond basic opcodes.
;
; During Apollo 11's mission, these functions were used extensively for:
; - Reading spacecraft sensors (gyroscopes, accelerometers)
; - Converting between angle representations for navigation
; - Performing matrix operations for coordinate transformations
; - Normalizing vectors for attitude calculations
; ============================================================================

# LOAD TIME2, TIME1 INTO MPAC:

; LOADTIME - Read Mission Elapsed Time
; Called when guidance programs need current mission time for trajectory
; calculations. During Apollo 11, this provided timestamps for navigation
; state updates and event scheduling throughout the 8-day mission.
;
; Technical: Reads double-precision TIME2/TIME1 register pair (mission clock)
; into MPAC for interpretive processing. TIME increments every 10 milliseconds.

LOADTIME	EXTEND
		DCA	TIME2
		TCF	SLOAD2

# CONVERT THE SINGLE PRECISION 2'S COMPLEMENT NUMBER ARRIVING IN MPAC (SCALED IN HALF-REVOLUTIONS) TO A
# DP 1'S COMPLEMENT NUMBER SCALED IN REVOLUTIONS.

; CDULOGIC - Convert IMU Gimbal Angles to Standard Format
; The Inertial Measurement Unit (IMU) gimbal angles arrive from hardware
; as single-precision 2's complement numbers. This routine converts them
; to the double-precision 1's complement format used throughout guidance
; calculations. During Apollo 11's translunar coast and lunar orbit, these
; conversions enabled accurate attitude determination.
;
; Technical: Converts single precision 2's complement angle (scaled in
; half-revolutions, range ±180°) to double precision 1's complement
; (scaled in revolutions, range ±0.5). Uses CCS to test sign, multiplies
; by HALF (0.5) to rescale, stores in MPAC as DP number.

CDULOGIC	CCS	MPAC
		CAF	ZERO
		TCF	+3
		NOOP
		CS	HALF

		TS	MPAC +1
		CAF	ZERO
		XCH	MPAC
		EXTEND
		MP	HALF
		DAS	MPAC
		TCF	DANZIG		# MODE IS ALREADY AT DOUBLE-PRECISION

# READ THE PIPS INTO MPAC WITHOUT CHANGING THEM:

; READPIPS - Read Accelerometer Data from IMU
; The PIPAs (Pulsed Integrating Pendulous Accelerometers) measure spacecraft
; acceleration along three axes. This routine reads the current accumulated
; velocity increments without zeroing the counters. During Apollo 11's engine
; burns (translunar injection, lunar orbit insertion, transearth injection),
; these readings provided velocity change measurements for navigation updates.
;
; Technical: Reads PIPAX, PIPAY, PIPAZ hardware registers containing accumulated
; velocity increments (pulses scaled as cm/sec). INHINT/RELINT bracket prevents
; interrupt corruption during multi-word read. Stores as vector in MPAC with
; zero lower words (converting to DP format). Each PIPA pulse = 5.85 cm/sec.

READPIPS	INHINT
		CA	PIPAX
		TS	MPAC
		CA	PIPAY
		TS	MPAC +3
		CA	PIPAZ
		RELINT
		TS	MPAC +5

		CAF	ZERO
		TS	MPAC +1
		TS	MPAC +4
		TS	MPAC +6

VECMODE		TCF	VMODE

# FORCE TP SIGN AGREEMENT IN MPAC:

; SGNAGREE - Force Triple Precision Sign Agreement
; Ensures all words of a triple-precision number have consistent sign
; representation in 1's complement format. Prevents computational errors
; in multi-word arithmetic used throughout navigation calculations.
;
; Technical: Calls TPAGREE subroutine to normalize TP number in MPAC,
; ensuring sign bits agree across all three word pairs.

SGNAGREE	TC	TPAGREE

# Page 1509

		TCF	DANZIG

# CONVERT THE DP 1'S COMPLEMENT ANGLE SCALED IN REVOLUTIONS TO A SINGLE PRECISION 2'S COMPLEMENT ANGLE
# SCALED IN HALF-REVOLUTIONS.

; 1STO2S - Convert Standard Angle Format to Hardware Format
; Reverse of CDULOGIC. Converts double-precision 1's complement angles
; (used in guidance calculations) back to single-precision 2's complement
; (required by IMU gimbal hardware). During Apollo 11's IMU alignments and
; attitude maneuvers, this conversion prepared angle commands for the IMU
; gimbal drive electronics.
;
; Technical: Converts DP 1's complement angle (scaled in revolutions) to
; single precision 2's complement (scaled in half-revolutions). Calls
; 1TO2SUB for conversion logic, clears lower word, returns in MPAC.

1STO2S		TC	1TO2SUB
		CAF	ZERO
		TS	MPAC +1
		TCF	NEWMODE

# DO 1STO2S ON A VECTOR OF ANGLES:

; V1STO2S - Convert Vector of Three Angles to Hardware Format
; Performs 1STO2S conversion on a 3-component angle vector. Used when
; commanding all three IMU gimbal axes simultaneously during attitude
; maneuvers. Critical for Apollo 11's platform alignments using star
; sightings and attitude control during all mission phases.
;
; Technical: Applies 1TO2SUB to each of three vector components in MPAC,
; converting DP 1's complement to SP 2's complement representation.

V1STO2S		TC	1TO2SUB		# ANSWER ARRIVES IN A AND MPAC.

		DXCH	MPAC +5
		DXCH	MPAC
		TC	1TO2SUB
		TS	MPAC +2

		DXCH	MPAC +3
		DXCH	MPAC
		TC	1TO2SUB
		TS	MPAC +1

		CA	MPAC +5
		TS	MPAC

TPMODE		CAF	ONE		# MODE IS TP.
		TCF	NEWMODE

# V1STO2S FOR 2 COMPONENT VECTOR. USED BY RR.

; 2V1STO2S - Convert Two-Component Angle Vector for Rendezvous Radar
; Specialized version of V1STO2S for 2-component vectors. Used specifically
; for Rendezvous Radar (RR) antenna pointing angles (shaft and trunnion).
; During rendezvous operations, this would convert computed antenna pointing
; angles to hardware format for radar gimbal control.
;
; Technical: Applies 1TO2SUB to two vector components (not three), used for
; RR 2-axis gimbal system. Processes MPAC+3 and MPAC, returns via SLOAD2.

2V1STO2S	TC	1TO2SUB
		DXCH	MPAC +3
		DXCH	MPAC
		TC	1TO2SUB
		TS	L
		CA	MPAC +3
		TCF	SLOAD2

# SUBROUTINE TO DO DOUBLING AND 1'S TO 2'S CONVERSION:

; 1TO2SUB - Convert 1's Complement to 2's Complement with Doubling
; Core conversion subroutine used by 1STO2S, V1STO2S, 2V1STO2S. Performs
; the mathematical conversion from double-precision 1's complement (revolutions)
; to single-precision 2's complement (half-revolutions). This scaling conversion
; is essential because AGC hardware CDUs output angles in 2's complement format
; while interpretive calculations use 1's complement for mathematical operations.
;
; Technical: DDOUBL doubles the DP value, then converts to 2's complement via
; CCS test. Handles overflow conditions via LIMITS table. Returns via Q register.

1TO2SUB		DXCH	MPAC		# FINAL MPAC +1 UNSPECIFIED.
		DDOUBL
		CCS	A
		AD	ONE
		TCF	+2
		COM			# THIS WAS REVERSE OF MSU.

		TS	MPAC		# AND SKIP ON OVERFLOW.
# Page 1510
		TC	Q

		INDEX	A		# OVERFLOW UNCORRECT AND IN MSU.
		CAF	LIMITS
		ADS	MPAC
		TC	Q

# Page 1511
# SUBROUTINE TO INCREMENT CDUS

; INCRCDUS - Increment All Three IMU CDU Angles
; Applies a 3-component angular increment vector to the IMU Coupling Data Unit
; (CDU) angles THETAD (inner gimbal), THETAD+1 (middle gimbal), THETAD+2 (outer
; gimbal). Used during IMU fine alignment procedures where computed corrections
; are applied to platform orientation. During Apollo 11 mission, this would
; execute during P51/P52 star sighting alignments.
;
; Technical: Processes vector in MPAC (1's complement) and increments CDU
; angles at THETAD (2's complement). Calls CDUINC three times for each axis.

INCRCDUS	CAF	LOCTHETA
		TS	BUF		# PLACE ADRES(THETA) IN BUF.
		CAE	MPAC		# INCREMENT IN 1S COMPL.
		TC	CDUINC

		INCR	BUF
		CAE	MPAC +3
		TC	CDUINC

		INCR	BUF
		CAE	MPAC +5
		TC	CDUINC

		TCF	VECMODE

LOCTHETA	ADRES	THETAD

# THE FOLLOWING ROUTINE INCREMENTS IN 2S COMPLEMENT THE REGISTER WHOSE ADDRESS IS IN BUF BY THE 1S COMPL.
# QUANTITY FOUND IN TEM2.  THIS MAY BE USED TO INCREMENT DESIRED IMU AND OPTICS CDU ANGLES OR ANY OTHER 2S COMPL.
# (+0 UNEQUAL TO -0) QUANTITY.  MAY BE CALLED BY BANKCALL/SWCALL.

; CDUINC - Increment 2's Complement CDU Register with 1's Complement Value
; General-purpose routine for adding 1's complement angular increment to a
; 2's complement CDU angle register. Critical for IMU and optics (sextant/
; telescope) pointing updates. Address of target register arrives in BUF,
; increment value arrives in accumulator. The 2's complement representation
; is required because hardware CDU registers distinguish +0 from -0 (unlike
; 1's complement where +0 equals -0).
;
; Technical: Converts target from 2's to 1's complement via CCS, adds increment,
; converts result back to 2's complement handling overflow via LIMITS table.

CDUINC		TS	TEM2		# 1S COMPL.QUANT. ARRIVES IN ACC.  STORE IT
		INDEX	BUF
		CCS	0		# CHANGE 2S COMPLE. ANGLE(IN BUF) INTO 1S
		AD	ONE
		TCF	+4
		AD	ONE
		AD	ONE		# OVEFLOW HERE IF 2S COMPL. IS 180 DEG.
		COM

		AD	TEM2		# SULT MOVES FROM 2ND TO 3D QUAD.(OR BACK)
		CCS	A		# BACK TO 2S COMPL.
		AD	ONE
		TCF	+2
		COM
		TS	TEM2		# STORE 14BIT QUANTITY WITH PRESENT SIGN
		TCF	+4
		INDEX	A		# SIGN.
		CAF	LIMITS		# FIX IT,BY ADDING IN 37777 OR 40000
		AD	TEM2

		INDEX	BUF
		TS	0		# STORE NEW ANGLE IN 2S COMPLEMENT.
		TC	Q

# Page 1512
# RTB TO TORQUE GYROS, EXCEPT FOR THE CALL TO IMUSTALL.  ECADR OF COMMANDS ARRIVES IN X1.

; PULSEIMU - Issue Gyro Torque Pulses to IMU
; Sends computed torque commands to the IMU gyroscopes to correct platform
; orientation errors. Used during IMU alignment (P51/P52) and fine alignment
; maintenance. The gyro commands compensate for measured drift and alignment
; errors, keeping the stable member accurately oriented. During Apollo 11,
; this would execute during pre-maneuver alignments and navigation updates.
;
; Technical: ECADR (Extended Core Address) of gyro commands should be in X1.
; Calls IMUPULSE via BANKCALL to apply torque pulses to physical gyros.

PULSEIMU	INDEX	FIXLOC		# ADDRESS OF GYRO COMMANDS SHOULD BE IN X1
		CA	X1
		TC	BANKCALL
		CADR	IMUPULSE
		TCF	DANZIG

# Page 1513
# EACH ROUTINE TAKES A 3X3 MATRIX STORED IN DOUBLE PRECISION IN A FIXED AREA OF ERASABLE MEMORY AND REPLACES IT
# WITH THE TRANSPOSE MATRIX.  TRANSP1 USES LOCATIONS XNB+0,+1 THROUGH XNB+16D, 17D AND TRANSP2 USES LOCATIONS
# XNB1+0,+1 THROUGH XNB1+16D, 17D.  EACH MATRIX IS STORED BY ROWS.

; TRANSP1 & TRANSP2 - In-Place 3x3 Matrix Transpose Operations
; These routines transpose coordinate transformation matrices used throughout
; navigation and guidance. TRANSP1 operates on XNB matrix (navigation base to
; stable member transformation), TRANSP2 on XNB1. Matrix transpose is required
; for inverse coordinate transformations: if XNB transforms from frame A to
; frame B, then transpose(XNB) transforms from frame B to frame A.
; During Apollo 11, these would execute in orbital navigation computations
; and IMU alignment calculations.
;
; Technical: Swaps off-diagonal matrix elements in-place via DXCH operations.
; Matrix stored by rows in double-precision (18 words = 9 DP values).
; Operations: swap(row0,col1 <-> row1,col0), swap(row0,col2 <-> row2,col0),
; swap(row1,col2 <-> row2,col1).

XNBEB		ECADR	XNB
XNB1EB		ECADR	XNB1

		EBANK=	XNB

TRANSP1		CAF	XNBEB
		TS	EBANK
		DXCH	XNB +2
		DXCH	XNB +6
		DXCH	XNB +2

		DXCH	XNB +4
		DXCH	XNB +12D
		DXCH	XNB +4

		DXCH	XNB +10D
		DXCH	XNB +14D
		DXCH	XNB +10D
		TCF	DANZIG


		EBANK=	XNB1

TRANSP2		CAF	XNB1EB
		TS	EBANK
		DXCH	XNB1 +2
		DXCH	XNB1 +6
		DXCH	XNB1 +2

		DXCH	XNB1 +4
		DXCH	XNB1 +12D
		DXCH	XNB1 +4

		DXCH	XNB1 +10D
		DXCH	XNB1 +14D
		DXCH	XNB1 +10D
		TCF	DANZIG

# Page 1514
# THE SUBROUTINE SIGNMPAC SETS C(MPAC, MPAC +1) TO SIGN(MPAC).
# FOR THIS, ONLY THE CONTENTS OF MPAC ARE EXAMINED.  ALSO +0 YIELDS POSMAX AND -0 YIELDS NEGMAX.
#
# ENTRY MAY BE BY EITHER OF THE FOLLOWING:
#	1.	LIMIT THE SIZE OF MPAC ON INTERPRETIVE OVERFLOW:
#		ENTRY:		BOVB
#					SIGNMPAC
#	2.	GENERATE IN MPAC THE SIGNUM FUNCTION OF MPAC:
#		ENTRY:		RTB
#					SIGNMPAC
# IN EITHER CASE, RETURN IS TO TEH NEXT INTERPRETIVE INSTRUCTION IN THE CALLING SEQUENCE.

; SIGNMPAC - Generate Sign Function or Limit Overflow in MPAC
; Dual-purpose routine: (1) Limits interpretive arithmetic overflow by clamping
; to maximum representable value, or (2) computes signum function (returns +1
; for positive, -1 for negative). Used throughout guidance calculations to
; handle overflow conditions gracefully and for sign-extraction operations.
; During Apollo 11 descent, this would protect against computational overflow
; in landing guidance equations.
;
; Technical: Sets MPAC to +POSMAX (positive max) or -POSMAX (negative max)
; based on sign of original MPAC value. Handles +0 -> POSMAX, -0 -> NEGMAX.
; Entry via BOVB (Branch on Overflow) or RTB (Return to Bank) instruction.

SIGNMPAC	EXTEND
		DCA	DPOSMAX
		DXCH	MPAC
		CCS	A
DPMODE		CAF	ZERO		# SETS MPAC +2 TO ZERO IN THE PROCESS
		TCF	SLOAD2 +2
		TCF	+1
		EXTEND
		DCS	DPOSMAX
		TCF	SLOAD2

# RTB OP CODE NORMUNIT IS LIKE INTERPRETIVE INSTRUCTION UNIT, EXCEPT THAT IT CAN BE DEPENDED ON NOT TO BLOW
# UP WHEN THE VECTOR BEING UNITIZED IS VERY SMALL -- IT WILL BLOW UP WHEN ALL COMPONENT ARE ZERO.  IF NORMUNIT
# IS USED AND THE UPPER ORDER HALVES OF ALL COMPONENTS ARE ZERO, THE MAGNITUDE RETURNS IN 36D WILL BE TOO LARGE
# BY A FACTOR OF 2(13) AND THE SQUARED MAGNITUDE RETURNED AT 34D WILL BE TOO BIG BY A FACTOR OF 2(26).

; NORMUNIT/NORMUNX1 - Normalize Vector to Unit Length (Safe for Small Vectors)
; Computes unit vector (direction) from input vector, avoiding overflow/underflow
; that standard UNIT instruction suffers with very small vectors. Essential for
; navigation calculations where line-of-sight vectors to stars or landmarks may
; have small magnitudes due to measurement uncertainty. During Apollo 11 star
; sightings, this would normalize measured star direction vectors.
;
; Technical: Tests if upper halves of all three components are zero. If so,
; shifts all components left 13 bits before unitization to prevent underflow.
; Returns magnitude (scaled by 2^13 if shifted) at MPAC+36D, squared magnitude
; at MPAC+34D (scaled by 2^26 if shifted). NORMUNX1 entry point with X1 flag.

NORMUNX1	CAF	ONE
		TCF	NORMUNIT +1
NORMUNIT	CAF	ZERO
		AD	FIXLOC
		TS	MPAC +2
		TC	BANKCALL	# GET SIGN AGREEMENT IN ALL COMPONENTS
		CADR	VECAGREE
		CCS	MPAC
		TCF	NOSHIFT
		TCF	+2
		TCF	NOSHIFT
		CCS	MPAC +3
		TCF	NOSHIFT
		TCF	+2
		TCF	NOSHIFT
		CCS	MPAC +5
		TCF	NOSHIFT
		TCF	+2
		TCF	NOSHIFT
# Page 1515
; If all upper halves were zero, shift all three vector components left by 13
; bits (multiply by 2^13 = 8192) to prevent underflow in subsequent magnitude
; calculation. Shifting preserves relative precision while bringing small
; values into computable range.

		CA	MPAC +1		# SHIFT ALL COMPONENTS LEFT 13
		EXTEND
		MP	BIT14
		DAS	MPAC		# DAS GAINS A LITTLE ACCURACY
		CA	MPAC +4
		EXTEND
		MP	BIT14
		DAS	MPAC +3
		CA	MPAC +6
		EXTEND
		MP	BIT14
		DAS	MPAC +5
		CAF	THIRTEEN
		INDEX	MPAC +2
		TS	37D
OFFTUNIT	TC	POSTJUMP
		CADR	UNIT +1		# SKIP THE "TC VECAGREE" DONE AT UNIT

NOSHIFT		CAF	ZERO
		TCF	OFFTUNIT -2

# RTB VECSGNAG ...FORCES SIGN AGREEMENT OF VECTOR IN MPAC.

; VECSGNAG - Force Sign Agreement on Vector Components
; Ensures all three components of vector in MPAC have consistent sign
; representation (converts from mixed 1's complement to uniform format).
; Essential preprocessing for vector arithmetic operations. Used throughout
; orbital navigation and guidance calculations.
;
; Technical: Calls VECAGREE bank subroutine then returns to interpreter via
; DANZIG. Converts each component to positive magnitude with proper sign bit.

VECSGNAG	TC	BANKCALL
		CADR	VECAGREE
		TC	DANZIG

# Page 1516
# MODULE CHANGE FOR NEW LUNAR GRAVITY MODEL

; ============================================================================
; TRANSITION: From general RTB opcodes to lunar gravity perturbation model
;
; The final section implements J22 gravitational harmonic computation for
; improved lunar orbital accuracy. During Apollo 11 translunar and lunar orbit
; phases, non-spherical lunar gravity (oblateness) affects trajectory. This
; code computes the J22 perturbation term (equatorial ellipticity effect).
; ============================================================================

; QUALITY1/QUALITY2 - Compute J22 Lunar Gravity Harmonic Perturbation
; Calculates acceleration due to Moon's equatorial ellipticity (J22 harmonic).
; Lunar gravity is not perfectly spherical; equator is slightly elliptical.
; This perturbation affects precision orbital navigation during lunar orbit
; insertion and while in lunar orbit. For Apollo 11, this contributed to
; accurate state vector propagation during Michael Collins' solo lunar orbit.
;
; Technical: Implements J22 term = (J22 coefficient) * R^4 * function(lat,lon).
; Computation: 5/8 * (Y^2 - X^2) * unit_position_vector, scaled appropriately.
; MOONFLAG determines if lunar gravity model applies (vs Earth-centered).
; Uses interpretive instructions: DSQ (square), VXSC (vector scale), etc.

		SETLOC	MODCHG3
		BANK
QUALITY1	BOF	DLOAD
			MOONFLAG
			NBRANCH
			URPV
		DSQ	GOTO
			QUALITY2
		SETLOC	MODCHG2
		BANK
QUALITY2	PDDL	DSQ		# SQUARE INTO 2D, B2
			URPV	+2	# Y COMPONENT, B1
		DSU
		DMP	VXSC		# 5(Y**2-X**2)UR
			5/8		# CONSTANT, 5B3
			URPV		# VECTOR, RESULT MAXIMUM IS 5, SCALING
					# HERE B6
		VSL3	PDDL		# STORE SCALED B3 IN 2D, 4D, 6D FOR XYZ
			URPV		# X COMPONENT, B1
		SR1	DAD		# 2 X X COMPONENT FOR B3 SCALING
			2D		# ADD TO VECTOR X COMPONENT OF ANSWER.
					# SAME AS MULTIPLYING BY UNITX.  MAX IS 7.
		STODL	2D
			URPV	+2	# Y COMPONENT, B1
		SR1	BDSU		# 2 X Y COMPONENT FOR B3 SCALING
			4D		# SUBTRACT FROM VECTOR Y COMPONENT OF
					# ANSWER, SAME AS MULTIPLYING BY UNITY.
					# MAX IS 7.
		STORE 	4D		# 2D HAS VECTOR, B3.
		SLOAD	VXSC		# MULTIPLY COEFFICIENT TIMES VECTOR IN 2D
			E3J22R2M
		PDDL	RVQ		# J22 TERM X R**4 IN 2D, SCALED B61
			COSPHI/2	# SAME AS URPV +4, Z COMPONENT

# *** END OF CHIEFTAN.028 ***

