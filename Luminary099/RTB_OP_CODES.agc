# Copyright:	Public domain.
# Filename:	RTB_OP_CODES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1397-1401
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Luminary131/ file of the same
#				name, using Luminary099 page images.
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
; FILE: RTB_OP_CODES.agc
; MODULE: Interpreter Special Functions
; MISSION PHASE: all phases
;
; TL;DR: Implements special-purpose interpretive opcodes accessed via RTB
;        (Return To Bank) instructions. Provides utility routines for number
;        format conversions, angle transformations, IMU operations, vector
;        normalization, and sign handling. These functions extend the
;        interpreter's capabilities beyond basic mathematical operations.
;
; COMMENT-ONLY READERS: This file contains support routines called by the
;        interpretive language when performing specialized operations during
;        navigation, guidance, and IMU management throughout all mission phases.
; CODE-ALONG READERS: Study these RTB implementations to understand how the
;        interpreter virtual machine is extended with special functions,
;        particularly conversion between 1's and 2's complement representations
;        and AGC-specific angle scaling conventions.
; ============================================================================

# Page 1397
		BANK	22
		SETLOC	RTBCODES
		BANK

		EBANK=	XNB
		COUNT*	$$/RTB

; ============================================================================
; SECTION: TIME AND NUMBER FORMAT CONVERSION UTILITIES
;
; The following routines provide fundamental data format conversions used
; throughout the AGC software, particularly for handling time values and
; converting between the AGC's native 1's complement arithmetic and the
; 2's complement format used for certain spacecraft sensors and displays.
; ============================================================================

# LOAD TIME2, TIME1 INTO MPAC:

; LOADTIME - Load Current Mission Time Into MPAC
;
; Retrieves the AGC's double-precision mission elapsed time (stored in
; TIME2/TIME1 registers) and loads it into MPAC for interpretive processing.
; Mission time increments continuously from spacecraft power-on and is used
; for all guidance, navigation, and sequencing computations.
;
; Entry: RTB / LOADTIME (from interpretive code)
; Exit: MPAC contains current mission time in double precision
; Calls: SLOAD2 (interpreter dispatch routine)

LOADTIME	EXTEND
		DCA	TIME2		; Load mission time (TIME2 is high word, TIME1 is low)
		TCF	SLOAD2		; Store in MPAC and return to interpreter

# 	   CONVERT THE SINGLE PRECISION 2'S COMPLEMENT NUMBER ARRIVING IN MPAC (SCALED IN HALF-REVOLUTIONS) TO A
# DP 1'S COMPLEMENT NUMBER SCALED IN REVOLUTIONS.

; CDULOGIC - Convert CDU Angle Format for Interpretive Processing
;
; The Coupling Data Units (CDUs) on the IMU provide gimbal angles in a specific
; format: single-precision 2's complement scaled in half-revolutions. This
; routine converts those readings to the interpreter's preferred format:
; double-precision 1's complement scaled in full revolutions.
;
; This conversion is essential for navigation computations because the
; interpretive language uses 1's complement arithmetic throughout, while
; the physical CDU hardware outputs 2's complement values.
;
; Entry: MPAC contains single-precision 2's complement angle (half-rev scale)
; Exit: MPAC contains double-precision 1's complement angle (revolution scale)
; Scaling: Input scaled by 180° (half-rev), output scaled by 360° (full rev)

CDULOGIC	CCS	MPAC		; Test sign of input angle
		CAF	ZERO		; Positive: set MPAC+1 to zero
		TCF	+3
		NOOP			; (Skip on +0)
		CS	HALF		; Negative: set MPAC+1 to -0.5

		TS	MPAC +1		; Store low-order word
		CAF	ZERO
		XCH	MPAC		; Get original value, zero MPAC
		EXTEND
		MP	HALF		; Multiply by 0.5 (convert half-rev to rev)
		DAS	MPAC		; Add to form double-precision result
		TCF	DANZIG		; MODE IS ALREADY AT DOUBLE-PRECISION

# 	   FORCE TP SIGN AGREEMENT IN MPAC:

; SGNAGREE - Force Triple-Precision Sign Agreement
;
; In AGC 1's complement arithmetic, both +0 and -0 exist as distinct values.
; For triple-precision (TP) numbers spanning three words, all three words
; must have consistent signs to maintain arithmetic correctness. This routine
; calls TPAGREE to enforce that consistency.
;
; Entry: MPAC contains triple-precision value (possibly with sign disagreement)
; Exit: MPAC contains same value with signs properly aligned across all words
; Calls: TPAGREE (triple-precision sign agreement subroutine)

SGNAGREE	TC	TPAGREE		; Force sign agreement across TP words
		TCF	DANZIG		; Return to interpreter dispatch

# 	   CONVERT THE DP 1'S COMPLEMENT ANGLE SCALED IN REVOLUTIONS TO A SINGLE PRECISION 2'S COMPLEMENT ANGLE
# SCALED IN HALF-REVOLUTIONS.

; 1STO2S - Convert Single Angle from 1's Complement to 2's Complement
;
; The inverse of CDULOGIC: converts an angle from the interpreter's
; double-precision 1's complement format (scaled in revolutions) back to
; single-precision 2's complement (scaled in half-revolutions) for output
; to spacecraft systems or displays that expect 2's complement format.
;
; Used when sending computed angles to the DSKY, DAP, or other systems.
;
; Entry: MPAC contains DP 1's complement angle (revolution scale)
; Exit: MPAC contains SP 2's complement angle (half-revolution scale)
; Calls: 1TO2SUB (performs the actual conversion)

1STO2S		TC	1TO2SUB		; Convert format and scaling
		CAF	ZERO
		TS	MPAC +1		; Clear low-order word (now single precision)
		TCF	NEWMODE		; Set mode to single precision, return

# 	   DO 1STO2S ON A VECTOR OF ANGLES:

; V1STO2S - Convert Three-Component Angle Vector
;
; Performs 1STO2S conversion on a three-component vector (e.g., IMU gimbal
; angles for inner, middle, and outer gimbals). Each component is converted
; independently from DP 1's complement to SP 2's complement format.
;
; Entry: MPAC contains three DP angles (vector components in MPAC, MPAC+2, MPAC+4)
; Exit: MPAC contains three SP angles in 2's complement format
; Used by: IMU alignment routines, gimbal angle display formatting

V1STO2S		TC	1TO2SUB		# ANSWER ARRIVES IN A AND MPAC.

		DXCH	MPAC +5		; Save first converted component
		DXCH	MPAC		; Load second component
		TC	1TO2SUB		; Convert second component
# Page 1398
		TS	MPAC +2		; Store second converted component

		DXCH	MPAC +3		; Load third component
		DXCH	MPAC
		TC	1TO2SUB		; Convert third component
		TS	MPAC +1		; Store third converted component

		CA	MPAC +5		; Retrieve first component
		TS	MPAC		; Complete vector in MPAC, MPAC+1, MPAC+2

TPMODE		CAF	ONE		# MODE IS TP.
		TCF	NEWMODE		; Set mode to triple precision, return

# 	   V1STO2S FOR 2 COMPONENT VECTOR. USED BY RR.

; 2V1STO2S - Convert Two-Component Angle Vector
;
; Similar to V1STO2S but for two-component vectors. Specifically used by
; the Rendezvous Radar (RR) system which provides shaft and trunnion angles.
; The radar reports target bearing in a two-angle coordinate system rather
; than a full three-axis representation.
;
; Entry: MPAC contains two DP angles (RR shaft and trunnion)
; Exit: MPAC contains two SP 2's complement angles
; Used by: Rendezvous radar data processing during CSM/LM rendezvous

2V1STO2S	TC	1TO2SUB		; Convert first component (shaft angle)
		DXCH	MPAC +3		; Save result, load second component
		DXCH	MPAC
		TC	1TO2SUB		; Convert second component (trunnion angle)
		TS	L		; Store in L register
		CA	MPAC +3		; Retrieve first component
		TCF	SLOAD2		; Load into MPAC and return to interpreter

; ============================================================================
; SECTION: 1'S TO 2'S COMPLEMENT CONVERSION SUBROUTINE
;
; The fundamental conversion routine used by all the above functions.
; Converts double-precision 1's complement to single-precision 2's complement
; while simultaneously changing the scaling from revolutions to half-revolutions.
; ============================================================================

# 	   SUBROUTINE TO DO DOUBLING AND 1'S TO 2'S CONVERSION:

; 1TO2SUB - Core Conversion from 1's Complement to 2's Complement
;
; This subroutine performs the actual mathematical transformation. The AGC
; uses 1's complement internally (where -0 and +0 are distinct: 77777 and 00000),
; but many external interfaces expect standard 2's complement. Additionally,
; the scaling changes by a factor of 2.
;
; The conversion algorithm: Take the DP value, double it (converting from
; full revolutions to half-revolutions), then apply 2's complement conversion
; which involves adding 1 to positive values. Overflow conditions are handled
; by clamping to maximum positive or negative limits.
;
; Entry: MPAC contains DP 1's complement value
; Exit: A register and MPAC contain SP 2's complement result
; Note: MPAC+1 (low word) is left unspecified after conversion

1TO2SUB		DXCH	MPAC		# FINAL MPAC +1 UNSPECIFIED.
		DDOUBL			; Double the DP value (shift left 1 bit)
		CCS	A		; Check sign and magnitude
		AD	ONE		; Positive: add 1 (2's complement adjustment)
		TCF	+2		; Continue
		COM			# THIS WAS REVERSE OF MSU.
					; Negative: complement (2's comp conversion)
		TS	MPAC		# AND SKIP ON OVERFLOW.
		TC	Q		; Normal return if no overflow

		INDEX	A		# OVERFLOW UNCORRECT AND IN MSU.
		CAF	LIMITS		; Overflow occurred: clamp to limits
		ADS	MPAC		; Add limit value (37777 or 40000)
		TC	Q		; Return with clamped result

; ============================================================================
; SECTION: IMU CDU ANGLE INCREMENT ROUTINE
;
; Used during IMU gyro pulsing to update CDU (Coupling Data Unit) registers
; that track gimbal angles. Must handle mixed 1's and 2's complement arithmetic
; and manage angle wraparound at ±180 degrees.
; ============================================================================

# 	   THE FOLLOWING ROUTINE INCREMENTS IN 2S COMPLEMENT THE REGISTER WHOSE ADDRESS IS IN BUF BY THE 1S COMPL.
# QUANTITY FOUND IN TEM2. THIS MAY BE USED TO INCREMENT DESIRED IMU AND OPTICS CDU ANGLES OR ANY OTHER 2S COMPL.
# (+0 UNEQUAL TO -0) QUANTITY. MAY BE CALLED BY BANKCALL/SWCALL.

; CDUINC - Increment IMU CDU Angle with Number Format Conversion
;
; This routine adds a 1's complement increment to a 2's complement angle register.
; During IMU operations, gyro torque pulses cause gimbal angle changes that must
; be tracked. The CDU (Coupling Data Unit) outputs are in 2's complement, but
; the AGC uses 1's complement internally.
;
; The routine converts the 2's complement angle to 1's complement, adds the
; increment, converts back to 2's complement, and handles wraparound at the
; ±180 degree boundaries (±37777 octal in half-revolution scaling).
;
; Entry: A register contains 1's complement increment
;        BUF contains address of 2's complement angle to update
; Exit: Updated angle stored back to address in BUF
; Uses: TEM2 for temporary storage

CDUINC		TS	TEM2		# 1S COMPL.QUANT. ARRIVES IN ACC. STORE IT
		INDEX	BUF		; Indirect addressing
		CCS	0		# CHANGE 2S COMPL. ANGLE(IN BUF)INTO 1S
		AD	ONE		; Positive: subtract 1 (2's to 1's conversion)
		TCF	+4		; Continue with conversion
		AD	ONE		; Handle +0 case
# Page 1399
		AD	ONE		# OVERFLOW HERE IF 2S COMPL. IS 180 DEG.
		COM			; Negative: complement and subtract 1

		AD	TEM2		# SULT MOVES FROM 2ND TO 3D QUAD.(OR BACK)
					; Add the 1's complement increment
		CCS	A		# BACK TO 2S COMPL.
		AD	ONE		; Positive: add 1 (1's to 2's conversion)
		TCF	+2		; Continue
		COM			; Negative: complement and add 1
		TS	TEM2		# STORE 14BIT QUANTITY WITH PRESENT SIGN
		TCF	+4		; No overflow, skip limit handling
		INDEX	A		#  SIGN.
		CAF	LIMITS		# FIX IT,BY ADDING IN 37777 OR 40000
		AD	TEM2		; Clamp result to valid angle range

		INDEX	BUF
		TS	0		# STORE NEW ANGLE IN 2S COMPLEMENT.
		TC	Q		; Return to caller

# Page 1400
; ============================================================================
; SECTION: IMU GYRO TORQUING COMMAND
;
; Sends pulse trains to the IMU gyros to torque the platform. Used during
; IMU alignment, gyrocompassing, and drift compensation. The pulses physically
; rotate the gyroscopes to null platform errors.
; ============================================================================

# 	   RTB TO TORQUE GYROS, EXCEPT FOR THE CALL TO IMUSTALL. ECADR OF COMMANDS ARRIVES IN X1.

; PULSEIMU - Issue Torque Pulses to IMU Gyros
;
; This RTB opcode sends commanded gyro torque pulses to the IMU. The three-axis
; pulse counts are fetched from the address specified in X1, then passed to
; the IMUPULSE routine which sequences the actual hardware interface.
;
; During platform alignment, the gyros are torqued to null errors measured by
; star sightings or accelerometer readings. During normal operation, compensation
; pulses correct for gyro drift. Each pulse represents a small angular increment
; (approximately 0.00028 degrees per pulse).
;
; Entry: X1 contains ECADR (erasable address) of gyro pulse commands
; Exit: Returns to interpreter via DANZIG after pulses issued
; Calls: IMUPULSE (via BANKCALL) to perform hardware interface

PULSEIMU	INDEX	FIXLOC		# ADDRESS OF GYRO COMMANDS SHOULD BE IN X1
		CA	X1		; Fetch erasable address from X1
		TC	BANKCALL	; Cross-bank call to IMU routine
		CADR	IMUPULSE	; IMUPULSE performs actual gyro pulsing
		TCF	DANZIG		; Return to interpreter dispatch loop

# Page 1401
; ============================================================================
; SECTION: SIGNUM FUNCTION AND OVERFLOW LIMITING
;
; SIGNMPAC implements the mathematical signum function (sign extraction) for
; the interpreter. Also serves as an overflow handler, clamping overflowed
; values to maximum limits. Dual entry points support both use cases.
; ============================================================================

# 	   THE SUBROUTINE SIGNMPAC  SETS C(MPAC, MPAC +1) TO SIGN(MPAC).
# FOR THIS, ONLY THE CONTENTS OF MPAC ARE EXAMINED.   ALSO +0 YIELDS POSMAX AND -0 YIELDS NEGMAX.
#
# ENTRY MAY BE BY EITHER OF THE FOLLOWING:
# 1.	   LIMIT THE SIZE OF MPAC ON INTERPRETIVE OVERFLOW:
# ENTRY:	  BOVB
#			 SIGNMPAC
# 2.	   GENERATE IN MPAC THE SIGNUM FUNCTION OF MPAC:
# ENTRY:	  RTB
#			 SIGNMPAC
# IN EITHER CASE, RETURN IS TO  THE NEXT INTERPRETIVE INSTRUCTION IN THE CALLING SEQUENCE.

; SIGNMPAC - Extract Sign and Set to Maximum Magnitude
;
; This RTB opcode implements the signum function, replacing MPAC's value with
; +1 (POSMAX) if positive, -1 (NEGMAX) if negative. This is useful for
; normalizing direction vectors or handling magnitude-independent operations.
;
; Additionally serves as an overflow handler via BOVB (Branch on Overflow).
; When computations exceed DP limits, this clamps results to ±POSMAX, preventing
; uncontrolled error propagation through subsequent calculations.
;
; The AGC's 1's complement arithmetic has both +0 and -0. This routine maps
; +0 to POSMAX and -0 to NEGMAX, maintaining sign information even for zero.
;
; Entry: MPAC contains DP value whose sign is to be extracted
; Exit: MPAC contains ±DPOSMAX (±1 in DP scaling)
; Return: Via SLOAD2 to interpreter dispatch

SIGNMPAC	EXTEND
		DCA	DPOSMAX		; Load +1 in DP format (37777 77777)
		DXCH	MPAC		; Swap with MPAC, saving original in A,L
		CCS	A		; Test sign of original MPAC value
DPMODE		CAF	ZERO		# SETS MPAC +2 TO ZERO IN THE PROCESS
		TCF	SLOAD2 +2	; Positive: keep +DPOSMAX in MPAC
		TCF	+1		; +0 case: also keep +DPOSMAX
		EXTEND
		DCS	DPOSMAX		; Negative: load -DPOSMAX (complement)
		TCF	SLOAD2		; Return with sign-adjusted result

; ============================================================================
; SECTION: SAFE VECTOR NORMALIZATION (UNIT VECTOR GENERATION)
;
; NORMUNIT is a robust version of the UNIT instruction that handles very small
; vectors without overflow. Essential for navigation calculations where vector
; magnitudes can vary widely (e.g., near-Earth vs cislunar positions).
; ============================================================================

#     RTB OP CODE NORMUNIT IS LIKE INTERPRETIVE INSTRUCTION UNIT, EXCEPT THAT IT CAN BE DEPENDED ON NOT TO BLOW
# UP WHEN THE VECTOR BEING UNITIZED IS VERY SMALL -- IT WILL BLOW UP WHEN ALL COMPONENTS ARE ZERO.   IF NORMUNIT
# IS USED AND THE UPPER ORDER HALVES OF ALL COMPONENTS ARE ZERO, THE MAGNITUDE RETURNED IN 36D WILL BE TOO LARGE
# BY A FACTOR OF 2(13) AND THE SQUARED MAGNITUDE RETURNED AT 34D WILL BE TOO BIG BY A FACTOR OF 2(26).

; NORMUNIT / NORMUNX1 - Safe Unit Vector Generation
;
; Creates a unit vector (magnitude = 1) from the input vector in MPAC. Unlike
; the basic UNIT instruction, NORMUNIT pre-scales small vectors to prevent
; overflow during the magnitude calculation. This is critical for navigation
; where position vectors can range from meters to hundreds of thousands of
; kilometers.
;
; Algorithm: 1) Check if high-order words of all three components are zero
;            2) If so, left-shift entire vector by 13 bits (multiply by 8192)
;            3) Call standard UNIT routine to normalize
;            4) Record shift count so calling code can adjust magnitude results
;
; NORMUNX1 is an alternate entry that uses X1 as the result storage base instead
; of the default FIXLOC location.
;
; Entry: MPAC contains 3-component DP vector (6 words total)
;        FIXLOC (or X1) contains base address for magnitude results
; Exit: MPAC contains normalized unit vector
;       Location 36D (offset from base) contains magnitude (may need scaling)
;       Location 34D contains squared magnitude (may need scaling)
; Note: If pre-scaled, magnitude is 2^13 too large, magnitude² is 2^26 too large

NORMUNX1	CAF	ONE		; Use X1 as base address
		TCF	NORMUNIT +1
NORMUNIT	CAF	ZERO		; Use FIXLOC as base address
		AD	FIXLOC
		TS	MPAC +2		; Store base address for later use
		TC	BANKCALL	# GET SIGN AGREEMENT IN ALL COMPONENTS
		CADR	VECAGREE	; Force triple-precision sign consistency
		CCS	MPAC		; Check X component high word
		TCF	NOSHIFT		; Non-zero: no shift needed
		TCF	+2		; Zero: check if -0 or +0
		TCF	NOSHIFT		; Negative: no shift needed
		CCS	MPAC +3		; Check Y component high word
		TCF	NOSHIFT
		TCF	+2
		TCF	NOSHIFT
		CCS	MPAC +5		; Check Z component high word
		TCF	NOSHIFT
		TCF	+2
		TCF	NOSHIFT
; All high-order words are zero: left-shift vector by 13 bits
# Page 1402
		CA	MPAC +1		# SHIFT ALL COMPONENTS LEFT 13
		EXTEND
		MP	BIT14		; Multiply by 2^13 (octal 20000)
		DAS	MPAC		# DAS GAINS A LITTLE ACCURACY
					; Double Add to Storage: Add A,L to MPAC
		CA	MPAC +4		; Y component low word
		EXTEND
		MP	BIT14
		DAS	MPAC +3		; Update Y component
		CA	MPAC +6		; Z component low word
		EXTEND
		MP	BIT14
		DAS	MPAC +5		; Update Z component
		CAF	THIRTEEN	; Record shift count (13 bits)
		INDEX	MPAC +2
		TS	37D		; Store at base+37D for caller
OFFTUNIT	TC	POSTJUMP
		CADR	UNIT +1		# SKIP THE "TC VECAGREE" DONE AT UNIT
					; Enter UNIT past VECAGREE (already done)

NOSHIFT		CAF	ZERO		; No shift needed
		TCF	OFFTUNIT -2	; Store zero shift count and proceed

; ============================================================================
; SECTION: VECTOR SIGN AGREEMENT
;
; VECSGNAG ensures all components of a vector have internally consistent sign
; representation (high and low words agree). Essential for reliable vector
; operations in the AGC's 1's complement arithmetic system.
; ============================================================================

# RTB VECSGNAG   ...FORCES SIGN AGREEMENT OF VECTOR IN MPAC.

; VECSGNAG - Enforce Triple-Precision Sign Agreement for Vector
;
; Forces sign agreement in all three components of the vector stored in MPAC.
; The AGC's 1's complement representation can have inconsistent signs between
; the high and low words of a double-precision value after certain operations.
;
; This routine calls VECAGREE which examines each of the three vector components
; (X, Y, Z) and ensures that if the high-order word is positive (or +0), the
; low-order word is also positive (or +0), and similarly for negative values.
; This prevents computational errors that could propagate through subsequent
; vector operations.
;
; Critical for navigation and guidance where vector accuracy directly affects
; spacecraft trajectory. Sign disagreement can cause small errors that accumulate
; over the mission duration.
;
; Entry: MPAC contains 3-component DP vector (MPAC through MPAC+5)
; Exit: MPAC contains same vector with enforced sign agreement
; Return: Via DANZIG to interpreter dispatch loop

VECSGNAG	TC	BANKCALL	; Cross-bank call to vector utility
		CADR	VECAGREE	; VECAGREE enforces triple-precision consistency
		TC	DANZIG		; Return to interpreter

# *** END OF SKIPPER .087 ***

