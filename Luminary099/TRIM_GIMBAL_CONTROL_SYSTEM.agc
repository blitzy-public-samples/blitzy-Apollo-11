# Copyright:	Public domain.
# Filename:	TRIM_GIMBAL_CNTROL_SYSTEM.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1472-1485
# Mod history:	2009-05-27 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
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
; FILE: TRIM_GIMBAL_CONTROL_SYSTEM.agc
; MODULE: Digital Autopilot Gimbal Trim System
; MISSION PHASE: descent/landing/ascent
;
; TL;DR: Controls engine gimbal positioning for the Descent Propulsion System
;        (DPS) and Ascent Propulsion System (APS) to align thrust vector with
;        spacecraft center of gravity. Continuously compensates for CG shifts
;        during propellant consumption, ensuring stable attitude control and
;        preventing unwanted rotation during powered flight phases.
;
; COMMENT-ONLY READERS: This system automatically adjusted the engine bell
;        angle during descent and ascent to keep the thrust pushing through
;        the spacecraft's center. As fuel burned, the center shifted, and
;        these calculations kept Eagle stable during the critical landing.
; CODE-ALONG READERS: Study the drive setting algorithm implementing optimal
;        control law for gimbal positioning. Note Newton iteration square root
;        computation and the integration with RCS autopilot coast detection.
; ============================================================================

# Page 1472
		BANK	21
		EBANK=	QDIFF
		SETLOC	DAPS4
		BANK

		COUNT*	$$/DAPGT

; ============================================================================
; GIMBAL TRIM SYSTEM (GTS) ENTRY POINT
;
; The Lunar Module's descent and ascent engines are mounted on gimbals that
; can pivot to direct thrust through the spacecraft's center of gravity. As
; propellant burns during powered flight, the CG shifts continuously. This
; system automatically adjusts gimbal angles to maintain thrust alignment,
; preventing unwanted rotation and maintaining stable attitude control.
;
; During Apollo 11's descent, this system operated continuously from powered
; descent initiation through landing, adjusting the engine bell angle dozens
; of times to compensate for fuel consumption and keep Eagle stable.
; ============================================================================

# CONTROL REACHES THIS POINT UNDER EITHER OF THE FOLLOWING TWO CONDITIONS ONCE THE DESCENT ENGINE AND THE DIGITAL
# AUTOPILOT ARE BOTH ON:
#	A) THE TRIM GIMBAL CONTROL LAW WAS ON DURING THE PREVIOUS Q,R-AXIS TIME5 INTERRUPT (OR THE DAPIDLER
#	   INITIALIZATION WAS SET FOR TRIM GIMBAL CONTROL AND THIS IS THE FIRST PASS), OR
#	B) THE Q,R-AXES RCS AUTOPILOT DETERMINED THAT THE VEHICLE WAS ENTERING (OR HAD JUST ENTERED) A COAST
#	   ZONE WITH A SMALL OFFSET ANGULAR ACCELERATION.
# GTS IS THE ENTRY TO THE GIMBAL TRIM SYSTEM FOR CONTROLLING ATTITUDE ERRORS AND RATES AS WELL AS ACCELERATIONS.

; Entry conditions: Descent engine throttled on and digital autopilot active.
; Either continuing from previous trim control pass, or RCS autopilot detected
; coast zone entry with small residual angular acceleration requiring trim.

; GTS main entry point - configure for gimbal trim control mode
GTS		CAF	NEGONE		# MAKE THE NEXT PASS THROUGH THE DAP BE
		TS	COTROLER	#   THROUGH RCS CONTROL,
		CAF	FOUR		#   AND ENSURE THAT IT IS NOT A SKIP.
		TS	SKIPU
		TS	SKIPV

; Gimbal trim mode initialization. Sets control mode indicators and timers
; to coordinate between gimbal control and RCS thruster control systems.
		CAF	TWO
		TS	INGTS		# SET INDICATOR OF GTS CONTROL POSITIVE.
		TS	QGIMTIMR	# SET TIMERS TO 200 MSEC TO AVOID BOTH
		TS	RGIMTIMR	# RUNAWAY AND INTERFERENCE BY NULLING.

; ============================================================================
; DRIVE SETTING ALGORITHM - OPTIMAL GIMBAL CONTROL LAW
;
; This algorithm computes the optimal gimbal drive direction to null attitude
; errors, rates, and angular accelerations. The control law balances current
; attitude error with predicted trajectory to minimize control effort.
;
; The mathematics implement a time-optimal bang-bang controller that drives
; the gimbal actuator in discrete positive or negative steps, computing when
; to reverse drive direction for smooth approach to the desired null state.
; ============================================================================

# THE DRIVE SETTING ALGORITHM
#
# DEL = SGN(OMEGA + ALPHA*ABS(ALPHA)/(2*K))
#					      2               1/2                  2       3/2
# NEGUSUM = ERROR*K + ALPHA*(DEL*OMEGA + ALPHA /(3*K)) + DEL*K   (DEL*OMEGA + ALPHA /(2*K))
#
# DRIVE = -SGN(NEGUSUM)

; DEL: Sign of rate plus scaled acceleration (determines drive direction)
; NEGUSUM: Weighted sum of error, rate, and acceleration terms
; DRIVE: Final gimbal drive command (positive or negative)
; K: Control gain constant for this axis (KQ or KR)
; OMEGA: Angular rate for this axis (EDOTQ or EDOTR)
; ALPHA: Angular acceleration (AOSQ or AOSR)
; ERROR: Attitude error requiring correction

; Save sign register for later restoration. The SR contains control flags
; that must be preserved across gimbal calculations.
		CA	SR		# SAVE THE SR.  SHIFT IT LEFT TO CORRECT
		AD	A		# FOR THE RIGHT SHIFT DUE TO EDITING.
		TS	SAVESR

; ============================================================================
; DUAL-AXIS GIMBAL CONTROL LOOP
;
; The descent engine gimbal has two independent axes: Q (pitch) and R (yaw).
; This loop processes both axes sequentially, computing optimal drive commands
; for each. The indexer (QRCNTR) selects between Q-axis (0) and R-axis (2)
; to access the appropriate rate, acceleration, and gain parameters.
; ============================================================================

GTSGO+ON	CAF	TWO		# SET INDEXER FOR R-AXIS CALCULATIONS.
		TCF	GOQTRIMG +1

; Q-axis gimbal control entry point (pitch axis)
GOQTRIMG	CAF	ZERO		# SET INDEXER FOR Q-AXIS CALCULATIONS
		TS	QRCNTR
# Page 1473
# RSB 2009 -----------------------------------------------------------------------
# Everything between this line and the similar line below was simply filled-in
# as-is from Luminary 131, and then verified to assemble to the proper binary
# values.  This area is blank on the Luminary 099 print-out, as if the printer
# ribbon had run out.

; Collect angular acceleration (AOS) for this axis. AOSQ for Q-axis (pitch),
; AOSR for R-axis (yaw). AOS scaled at pi/2 radians per second squared.
		INDEX	QRCNTR		# AOS SCALED AT PI/2
		CA	AOSQ
		EXTEND
		MP	BIT2		# RESCALE AOS TO PI/4
		EXTEND
		BZF	GTSQAXIS -3	# USE FULL SCALE FOR LARGER AOS ESTIMATES.

; Set acceleration limits for range checking. POSMAX and NEGMAX define
; maximum allowable acceleration values to prevent gimbal overdriving.
		INDEX	A
		CS	LIMITS		# LIMITS +1 CONTAINS NEGMAX.
		XCH	L		# LIMITS -1 CONTAINS POSMAX.

; Collect angular rate (EDOT) for this axis. Rates stored adjacently:
; EDOTQ (Q-axis rate), EDOTR (R-axis rate). Scaled at pi/4 rad/sec.
		CCS	QRCNTR		# PICK UP RATE FOR THIS AXIS.  RATE CELLS
		INDEX	A		# USE ADJACENT, NOT SEPARATED.  AT PI/4
		CA	EDOTQ
GTSQAXIS	DXCH	WCENTRAL

; Collect control gain constant K for this axis (KQ or KR). Gain determines
; control authority and response speed for gimbal positioning.
		INDEX	QRCNTR		# COLLECT K FOR THIS AXIS
		CA	KQ
		TS	KCENTRAL

; Check for zero control authority. If K=0, no gimbal control possible,
; avoid attempting to drive engine bell (would hit mechanical stops).
		EXTEND			# CONTROL AUTHORITY ZERO.  AVOID DRIVING
		BZF	POSDRIVE +1	# ENGINE BELL TO THE STOPS.

; ============================================================================
; NEGUSUM COMPUTATION - FIRST TERM: ERROR * K
;
; Computes the first term of the control algorithm: attitude error multiplied
; by control gain. QDIFF (or RDIFF) contains the attitude error in radians,
; scaled at pi. This represents how far the current gimbal angle is from the
; desired angle needed to align thrust with the center of gravity.
; ============================================================================

		INDEX	QRCNTR		# QDIFF, RDIFF ARE STORED IN D.P.
		CAE	QDIFF

; Compute K*ERROR term. This represents the proportional control component,
; directly relating attitude error to required gimbal correction.
ALGORTHM	EXTEND			# Q(R)DIFF IS THETA (ERROR) SCALED AT PI.
		MP	KCENTRAL	# FORM K*ERROR AT PI(2)/2(8), IN D.P.
		LXCH	K2THETA
		EXTEND
		MP	BIT5		# RESCALE TO 4*PI(2)
		DXCH	K2THETA
		EXTEND
		MP	BIT5		# FIRST TERM OF NEGUSUM IN K2THETA.
		ADS	K2THETA +1	# NO CARRY NEEDED	D.P. AT 4*PI(2)

; ============================================================================
; COMPUTE ALPHA^2/(2*K) - ACCELERATION SCALING TERM
;
; This term represents the relationship between angular acceleration and
; control authority. Larger accelerations relative to gain require larger
; corrections. Division by K scales the correction appropriately for the
; available control authority. Overflow protection ensures safe operation.
; ============================================================================

		CS	ACENTRAL	# FORM ALPHA(2)/(2*K) AT 16*PI, IN D.P.,
		EXTEND			# LIMITING QUOTIENT TO AVOID OVERFLOW.
		MP	BIT14		# -ALPHA/2 IN A, SCALED AT PI/4
		EXTEND
		MP	ACENTRAL	# -ALPHA(2)/2 IN A,L, SCALED AT PI(2)/16)
		AD	KCENTRAL
		EXTEND
		BZMF	HUGEQUOT	# K-ALPHA(2)/2 SHOULD BE PNZ FO DIVISION

; Perform double-precision division: ALPHA^2/2 divided by K
; Uses AGC's two-stage division (high-order then low-order quotient)
		EXTEND
		DCS	A		# ALPHA(2)/2 - K
		AD	KCENTRAL
# RSB 2009 -----------------------------------------------------------------------
		EXTEND
		DV	KCENTRAL	# HIGH ORDER OF QUOTIENT.
		XCH	A2CNTRAL
		CA	L		# SHIFT UP THE REMAINDER.
		LXCH	7		# ZERO LOW-ORDER DIVIDEND.
		EXTEND
# Page 1474
		DV	KCENTRAL
		XCH	A2CNTRAL +1	# QUOTIENT STORED AT 16*PI , D.P.
		TCF	HAVEQUOT

; Overflow protection: If division would overflow (acceleration extremely
; large relative to control authority), limit quotient to maximum value.
HUGEQUOT	CA	POSMAX
		TS	L
		DXCH	A2CNTRAL	# LIMITED QUOTIENT STORED AT 16*PI, D.P.

; ============================================================================
; COMPUTE FUNCTION1 = OMEGA + ALPHA*ABS(ALPHA)/(2*K)
;
; This function determines the direction of optimal gimbal drive. When
; positive, the vehicle needs to drive in one direction; when negative,
; in the opposite direction. The term ALPHA*ABS(ALPHA)/(2*K) represents
; the acceleration's contribution to the decision, weighted by control
; authority. Taking absolute value ensures proper sign handling.
; ============================================================================

HAVEQUOT	CA	WCENTRAL
		EXTEND
		MP	BIT9		# RESCALE OMEGA AT 16*PI IN D.P.
		DXCH	K2CNTRAL	# LOWER WORD OVERLAYS OMEGA IN WCENTRAL

; Store OMEGA (angular rate) in FUNCTION register for upcoming calculation
		EXTEND
		DCA	K2CNTRAL
		DXCH	FUNCTION

; Apply sign to ALPHA^2/(2*K) based on ALPHA sign to get ALPHA*ABS(ALPHA)/(2*K)
		CA	ACENTRAL	# GET ALPHA*ABS(ALPHA)/(2*K)
		EXTEND
		BZMF	+4		# IF ALPHA NEGATIVE, NEGATE THE QUOTIENT

		EXTEND
		DCA	A2CNTRAL	# ALPHA POSITIVE, USE A2CNTRAL AS-IS
		TCF	+3

		EXTEND
		DCS	A2CNTRAL	# ALPHA NEGATIVE, NEGATE A2CNTRAL

		DAS	FUNCTION	# OMEGA + ALPHA*ABS(ALPHA)/(2*K) AT 16*PI

; ============================================================================
; DETERMINE DEL = SGN(FUNCTION1)
;
; DEL represents the optimal drive direction: +1 to increase gimbal angle,
; -1 to decrease it. This binary decision implements bang-bang control,
; driving the gimbal at maximum rate toward the optimal position. The sign
; of FUNCTION1 determines which direction minimizes attitude error.
; ============================================================================

		CCS	FUNCTION	# DEL = +1 FOR FUNCT1 GREATER THAN ZERO.
		TCF	POSFNCT1	# OTHERWISE DEL = -1
		TCF	+2
		TCF	NEGFNCT1

		CCS	FUNCTION +1	# USE LOW ORDER WORD SINCE HIGH IS ZERO
POSFNCT1	CAF	BIT1		# DEL = +1 (drive in positive direction)
		TCF	+2
NEGFNCT1	CS	BIT1		# DEL = -1 (drive in negative direction)
		TS	DEL

; ============================================================================
; COMPUTE DEL*OMEGA
;
; Multiply angular rate OMEGA by the drive direction DEL. This signed rate
; will be used in subsequent terms of the NEGUSUM control law. If DEL is
; negative, we negate OMEGA; if positive, OMEGA stays as-is; if zero
; (which shouldn't happen), we zero the result.
; ============================================================================

		CCS	DEL		# REPLACE OMEGA BY DEL*OMEGA
		TCF	FUNCT2		# POSITIVE DEL VALUE.  PROCEED.
		TCF	DEFUNCT		# ZERO DEL (shouldn't occur)
		TCF	NEGFNCT2	# NEGATIVE DEL

DEFUNCT		TS	K2CNTRAL	# Zero case: clear OMEGA
		TS	K2CNTRAL +1
		TCF	FUNCT2

# Page 1475
NEG1/3		DEC	-.33333

NEGFNCT2	EXTEND			# Negate OMEGA for negative DEL
		DCS	K2CNTRAL
		DXCH	K2CNTRAL

; ============================================================================
; COMPUTE DEL*OMEGA + ALPHA^2/(2*K)
;
; This intermediate result appears in multiple terms of the NEGUSUM equation.
; It combines the signed rate (DEL*OMEGA) with the acceleration scaling term.
; This sum will be used both in the square root term and in computing the
; ALPHA*(DEL*OMEGA + ALPHA^2/(3*K)) contribution to NEGUSUM.
; ============================================================================

FUNCT2		EXTEND
		DCA	A2CNTRAL
		DAS	K2CNTRAL	# DEL*OMEGA + ALPHA(2)/(2*K) AT 16*PI,D.P.

; ============================================================================
; COMPUTE ALPHA^2/(3*K) FROM ALPHA^2/(2*K)
;
; We need ALPHA^2/(3*K) for one term of NEGUSUM. Rather than recompute from
; scratch, we multiply the already-computed ALPHA^2/(2*K) by -1/3 to get
; -ALPHA^2/(6*K), then add DEL*OMEGA + ALPHA^2/(2*K) to get the desired
; DEL*OMEGA + ALPHA^2/(3*K). This approach saves computation time.
;
; Mathematical derivation:
;   -ALPHA^2/(6*K) + [DEL*OMEGA + ALPHA^2/(2*K)]
;   = DEL*OMEGA + ALPHA^2/(2*K) - ALPHA^2/(6*K)
;   = DEL*OMEGA + [3*ALPHA^2/(6*K) - ALPHA^2/(6*K)]
;   = DEL*OMEGA + 2*ALPHA^2/(6*K)
;   = DEL*OMEGA + ALPHA^2/(3*K)
; ============================================================================

FUNCT3		CA	A2CNTRAL
		EXTEND
		MP	NEG1/3		# Multiply high-order by -1/3
		DXCH	A2CNTRAL
		CA	L
		EXTEND
		MP	NEG1/3		# Multiply low-order by -1/3
		ADS	A2CNTRAL +1
		TS	L
		TCF	+2		# A2CNTRAL NOW CONTAINS  -ALPHA(2)/(6*K),
		ADS	A2CNTRAL	# SCALED AT 16*PI, IN D.P.

		EXTEND
		DCA	K2CNTRAL	# DEL*OMEGA + ALPHA(2)/(3*K) IN A2CNTRAL,
		DAS	A2CNTRAL	# SCALED AT 16*PI, D.P.

; ============================================================================
; COMPUTE ALPHA*(DEL*OMEGA + ALPHA^2/(3*K)) AND ACCUMULATE IN NEGUSUM
;
; This is the second term of the NEGUSUM control law equation. We multiply
; the angular acceleration ALPHA by the previously computed
; [DEL*OMEGA + ALPHA^2/(3*K)]. The result is added to K2THETA, which
; accumulates the complete NEGUSUM value used to determine drive direction.
;
; This term represents the acceleration's contribution to the control law,
; weighted by both the current rate and a portion of the acceleration itself.
; ============================================================================

		CA	A2CNTRAL
		EXTEND
		MP	ACENTRAL	# Multiply high-order by ALPHA
		DAS	K2THETA		# Accumulate into NEGUSUM (K2THETA)
		CA	A2CNTRAL +1
		EXTEND
		MP	ACENTRAL	# ACENTRAL MAY NOW BE OVERLAID.
		ADS	K2THETA +1
		TS	L
		TCF	+2		# TWO TERMS OF NEGUSUM ACCUMULATED, SO FAR
		ADS	K2THETA		# SCALED AT 4*PI(2), IN D.P.

; ============================================================================
; COMPUTE K*(DEL*OMEGA + ALPHA^2/(2*K)) FOR SQUARE ROOT TERM
;
; The third term of NEGUSUM requires computing the square root of
; K*(DEL*OMEGA + ALPHA^2/(2*K)). This section prepares that argument by
; multiplying the previously saved [DEL*OMEGA + ALPHA^2/(2*K)] by the
; control authority K. The result is stored in FUNCTION for the square
; root subroutine GTSQRT.
;
; This term represents the "energy" in the system that needs to be
; dissipated, combining rate and acceleration effects scaled by authority.
; ============================================================================

GETROOT		CA	K2CNTRAL	# K*(DEL*OMEGA + ALPHA(2)/(2*K)) IS THE
		EXTEND			# TERM FOR WHICH A SQUARE ROOT IS NEEDED.
		MP	KCENTRAL	# K AT PI/2(8)
		DXCH	FUNCTION
		CA	K2CNTRAL +1
		EXTEND
		MP	KCENTRAL
		ADS	FUNCTION +1
		TS	L
		TCF	+2
		ADS	FUNCTION	# DESIRED TERM IN FUNCTION, AT PI(2)/16
# Page 1476

; ============================================================================
; CHECK DEL SIGN AND PREPARE FOR SQUARE ROOT
;
; If DEL is positive, proceed directly to square root computation (RSTOFGTS).
; If DEL is negative, we must negate K2CNTRAL before computing the square
; root, ensuring the square root argument is positive. After the root is
; computed, its sign will be corrected based on DEL in the final NEGUSUM
; calculation. If DEL is zero (shouldn't occur), skip the square root term.
; ============================================================================

		CCS	DEL
		TCF	RSTOFGTS	# DEL positive, compute square root
		TCF	NEGUSUM		# DEL zero, skip square root term
		TCF	NEGATE		# DEL negative, negate then compute
		TCF	NEGUSUM		# DEL negative magnitude zero

NEGATE		EXTEND
		DCS	K2CNTRAL	# Negate K2CNTRAL for negative DEL
		DXCH	K2CNTRAL
		TCF	RSTOFGTS	# Proceed to square root computation

		BANK	16
		EBANK=	NEGUQ
		SETLOC	DAPS1
		BANK

# THE WRCHN12 SUBROUTINE SETS BITS 9,10,11,12 OF CHANNEL 12 ON THE BASIS OF THE CONTENTS OF NEGUQ,NEGUR WHICH ARE
# THE NEGATIVES OF THE DESIRED ACCELERATION CHANGES.  ACDT+C12 SETS Q(R)ACCDOT TO REFLECT THE NEW DRIVES.
#
# WARNING:  ACDT+C12 AND WRCHN12 MUST BE CALLED WITH INTERRUPT INHIBITED.

BGIM		OCTAL	07400
CHNL12		EQUALS	ITEMP6
; ============================================================================
; ACDT+C12 - Update acceleration estimates and write gimbal drive commands.
;
; This subroutine updates the stored acceleration (jerk) estimates for both
; Q and R axes based on the latest gimbal drive decisions (NEGUQ, NEGUR).
; It then translates the drive commands into hardware control bits and writes
; them to channel 12, which controls the descent engine gimbal actuators.
;
; The gimbal drive bits in channel 12 are:
;   BIT9  (Q-axis negative drive)  BIT10 (Q-axis positive drive)
;   BIT11 (R-axis negative drive)  BIT12 (R-axis positive drive)
;
; INPUTS:
;   NEGUQ, NEGUR: Drive commands (+1, 0, -1) for Q and R axes
;   ACCDOTQ, ACCDOTR: Acceleration derivative magnitudes (scaled PI/2^7)
;
; OUTPUTS:
;   QACCDOT, RACCDOT: Updated signed acceleration derivatives
;   CHAN12: Gimbal drive control bits written to hardware
;
; CALLING SEQUENCE:
;   This routine is called when CALLGMBL flag is set in RCSFLAGS
; ============================================================================

ACDT+C12	CS	NEGUQ		# Load complement of Q-axis drive command
		EXTEND			# Prepare for multiplication
		MP	ACCDOTQ		# Multiply: -NEGUQ * |ACCDOTQ|
		LXCH	QACCDOT		# Store signed Q acceleration derivative
		CS	NEGUR		# Load complement of R-axis drive command
		EXTEND
		MP	ACCDOTR		# Multiply: -NEGUR * |ACCDOTR|
		LXCH	RACCDOT		# Store signed R acceleration derivative

; Translate Q-axis drive command to channel 12 control bits.
; NEGUQ = +1 → drive negative → BIT9
; NEGUQ = -1 → drive positive → BIT10
; NEGUQ = 0  → no drive → no bits set

		CCS	NEGUQ		# Check sign of Q drive command
		CAF	BIT10		# Positive NEGU: drive negative (BIT10)
		TCF	+2		# Skip to store
		CAF	BIT9		# Negative NEGU: drive positive (BIT9)
		TS	CHNL12		# Store Q-axis gimbal control bits

; Translate R-axis drive command to channel 12 control bits.
; NEGUR = +1 → drive negative → BIT11
; NEGUR = -1 → drive positive → BIT12
; NEGUR = 0  → no drive → no bits set

		CCS	NEGUR		# Check sign of R drive command
		CAF	BIT12		# Positive NEGU: drive negative (BIT12)
		TCF	+2		# Skip to accumulate
		CAF	BIT11		# Negative NEGU: drive positive (BIT11)
		ADS	CHNL12		# Add R-axis bits to Q-axis bits

; Write combined gimbal drive commands to channel 12.
; First, clear existing gimbal drive bits (BGIM mask), then set new bits.

		CS	BGIM		# Load complement of gimbal bit mask
		EXTEND
		RAND	CHAN12		# Read channel 12 and clear gimbal bits
		AD	CHNL12		# Add new gimbal drive bits
		EXTEND
		WRITE	CHAN12		# Write updated control word to hardware
# Page 1477
		CS	CALLGMBL	# Load complement of CALLGMBL flag bit
		MASK	RCSFLAGS	# Clear request bit in RCSFLAGS
		TS	RCSFLAGS	# Update flags (turn off ACDT+C12 request)

		TC	Q		# Return to caller

		BANK	21
		EBANK=	QDIFF
		SETLOC	DAPS4
		BANK

# Page 1478
# SUBROUTINE TIMEGMBL:	MOD 0, OCTOBER 1967, CRAIG WORK
#
# TIMEGMBL COMPUTES THE DRIVE TIME NEEDED FOR THE TRIM GIMBAL TO POSITION THE DESCENT ENGINE NOZZLE SO AS TO NULL
# THE OFFSET ANGULAR ACCELERATION ABOUT THE Q (OR R) AXIS.  INSTEAD OF USING AOSQ(R), TIMEGMBL USES .4*AOSQ(R),
# SCALED AT PI/8.                         FOR EACH AXIS, THE DRIVE TIME IS COMPUTED AS ABS(ALPHA/ACCDOT).  A ZERO
# ALPHA OR ACCDOT OR A ZERO QUOTIENT TURNS OFF THE GIMBAL DRIVE IMMEDIATELY.  OTHERWISE, THE GIMBAL IS TURNED ON
# DRIVING IN THE CORRECT DIRECTION. THE Q(R)GIMTIMR IS SET TO TERMINATE THE DRIVE AND Q(R)ACCDOT
# IS STORED TO REFLECT THE NEW ACCELERATION DERIVATIVE.  NEGUQ(R) WILL CONTAIN +1,+0,-1 FOR A Q(R)ACCDOT VALUE
# WHICH IS NEGATIVE, ZERO, OR POSITIVE.
#
# INPUTS:  AOSQ,AOSR, SCALED AT P1/2, AND ACCDOTQ, ACCDOTR AT PI/2(7).    PI/2(7).
#
# OUTPUTS:   NEW GIMBAL DRIVE BITS IN CHANNEL 12,NEGUQ,NEGUR,QACCDOT AND RACCDOT, THE LAST SCALED AT PI/2(7).
#	     Q(R)GIMTIMR WILL BE SET TO TIME AND TERMINATE GIMBAL DRIVE(S)
#
# DEBRIS:  A,L,Q, ITEMPS 2,3,6, RUPTREG2 AND ACDT+C12 DEBRIS.
#
# EXITS:  VIA TC Q.
#
# ALARMS, ABORTS, :  NONE
#
# SUBROUTINES:  ACDT+C12, IBNKCALL
#
# WARNING:  THIS SUBROUTINE WRITES INTO CHANNEL 12 AND USES THE ITEMPS.  THEREFORE IT MAY ONLY BE CALLED WITH
# INTERRUPT INHIBITED.
#
# ERASABLE STORAGE CONFIGURATION (NEEDED BY THE INDEXING METHODS):
# NEGUQ		ERASE	+2			# NEGATIVE OF Q-AXIS GIMBAL DRIVE
# (SPWORD)	EQUALS	NEGUQ +1		# ANY S.P. ERASABLE NUMBER, NOW THRSTCMD
# NEGUR		EQUALS	NEGUQ +2		# NEGATIVE OF R-AXIS GIMBAL DRIVE
# ACCDOTQ	ERASE	+2			# Q-JERK TERM SCALED AT PI/2(7) RAD/SEC(3)
# (SPWORD)	EQUALS	ACCDOTQ +1		# ANY S.P. ERASABLE NUMBER NOW QACCDOT
# ACCDOTR	EQUALS	ACCDOTQ +2		# R-JERK TERM SCALED AT PI/2(7) RAD/SEC(3)
#						# ACCDOTQ,ACCDOTR ARE MAGNITUDES.
# AOSQ		ERASE   +4			# Q-AXIS ACC., D.P. AT PI/2 R/SEC(2)
# AOSR		EQUALS  AOSQ +2			# R-AXIS ACCELERATION SCALED AT PI/2 R/S2

QRNDXER		EQUALS	ITEMP6
OCT23146	OCTAL	23146			# DECIMAL .6
NZACCDOT	EQUALS	ITEMP3

; ============================================================================
; TIMEGMBL - Compute gimbal drive time and enable engine gimbal actuators.
;
; This routine is the non-GTS-control-law interface to the gimbal trim system.
; It computes how long the descent engine gimbal should be driven to null out
; offset angular accelerations (AOSQ, AOSR) without using the full attitude
; error and rate feedback of the GTS control law. TIMEGMBL is called by the
; AOSTASK when offset accelerations are detected but full GTS control is not
; warranted.
;
; For each axis (Q and R), TIMEGMBL:
; 1. Checks if ACCDOT (acceleration derivative) is positive (non-zero)
; 2. Scales AOS (offset angular acceleration) to 0.4*AOS for drive computation
; 3. Computes drive time as: TIME = ABS(0.4*AOS) / ACCDOT
; 4. Sets gimbal timer (QGIMTIMR or RGIMTIMR) to terminate drive automatically
; 5. Stores drive direction in NEGUQ or NEGUR (+1, 0, or -1)
; 6. Calls ACDT+C12 to update acceleration estimates and write to hardware
;
; Drive time limit: Maximum 2 seconds. If computed time exceeds 2 seconds,
; ALLOWGTS flag is cleared to prevent GTS control law activation until AOSTASK
; re-approves (prevents runaway gimbal motion).
;
; SCALING:
;   AOSQ, AOSR: PI/2 radians/sec^2 (offset angular acceleration)
;   ACCDOTQ, ACCDOTR: PI/2^7 radians/sec^3 (jerk magnitude, always positive)
;   Drive time: 2^14/100 seconds (waitlist time units)
;
; This routine is a critical safety mechanism. During Apollo 11's descent,
; small offset accelerations were continuously corrected by this system,
; maintaining stable attitude control while conserving RCS propellant.
; ============================================================================

TIMEGMBL	CAF	ONE			# Initialize ALLOWGTS flag
		TS	ALLOWGTS		# (permits GTS control law if needed)

		CAF	TWO			# Set index for R-axis (first pass)
		LXCH	Q			# Save return address in L register
		LXCH	RUPTREG2		# Transfer to RUPTREG2 for safekeeping
# Page 1479
		TCF	+2			# Skip to R-axis processing first

; TIMQGMBL - Second pass entry point for Q-axis processing.
; After R-axis is complete, control falls through here for Q-axis.

TIMQGMBL	CAF	ZERO			# Set index for Q-axis (second pass)
		TS	QRNDXER			# QRNDXER = 0 selects AOSQ, ACCDOTQ

; ============================================================================
; Main processing loop - executes twice (R-axis with QRNDXER=2, then Q-axis
; with QRNDXER=0). INDEX instruction uses QRNDXER to select register pairs:
;   QRNDXER=0: AOSQ/ACCDOTQ/NEGUQ/QGIMTIMR (Q-axis)
;   QRNDXER=2: AOSR/ACCDOTR/NEGUR/RGIMTIMR (R-axis)
; ============================================================================

		INDEX	QRNDXER		# Select ACCDOTQ or ACCDOTR
		CA	ACCDOTQ			# Load acceleration derivative (PI/2^7)
		EXTEND
		BZMF	TGOFFNOW		# If ACCDOT ≤ 0, turn off gimbal
		TS	NZACCDOT		# Store non-zero positive ACCDOT

; Retrieve offset angular acceleration (AOS) for this axis.
; AOS represents steady-state angular acceleration that must be nulled
; by engine gimbal trimming. Typical causes: CG offset, asymmetric mass
; distribution, or external disturbance torques.

ALPHATRY	INDEX	QRNDXER		# Select AOSQ or AOSR
		CS	AOSQ			# Load -AOS (complement for sign handling)
		EXTEND
		BZF	TGOFFNOW		# If AOS = 0, no correction needed

; Scale AOS from PI/2 to PI/8 and multiply by 0.4 for drive computation.
; The 0.4 factor implements a conservative gimbal drive strategy:
; - Full AOS correction would command: TIME = AOS/ACCDOT
; - Using 0.4*AOS provides: TIME = 0.4*AOS/ACCDOT
; This prevents overshoot and allows iterative convergence to steady state.
;
; The computation: -AOS * 0.6 - AOS = -1.6*AOS
; This is then scaled by additional factors to produce 0.4*AOS for the drive.
; The conservative 0.4 factor ensures smooth gimbal motion without overshooting.

		TS	Q			# Save -AOS in Q register for later addition
		EXTEND				# Prepare for multiplication
		MP	OCT23146		# Multiply: -AOS * 0.6 (OCT23146 ≈ 0.6 scaled)
		AD	Q			# Add back -AOS: result = -0.6*AOS + (-AOS)
		TS	L			# Store to L, testing for overflow
		TCF	SETNEGU			# No overflow: proceed to compute drive time

; Overflow handling - scaled AOS magnitude exceeds register capacity.
; If the multiplication/addition overflows, the offset acceleration is very large.
; In this case, immediately start maximum drive without computing time, and
; disable further GTS attitude-rate control until AOSTASK revalidates conditions.

		CS	A			# Recover -SGN(AOS) in A register (sign only)
		INDEX	QRNDXER			# Select NEGUQ or NEGUR based on axis
		XCH	NEGUQ			# Store drive direction (+1 or -1)
		TCF	NOTALLOW		# Set timer to 31 (2 sec), clear ALLOWGTS

; ============================================================================
; SETNEGU - Determine drive direction and magnitude
;
; COMMENT-ONLY READERS: The computer determines which direction to move the
; engine gimbal based on the sign of the offset acceleration. If the spacecraft
; is rotating in one direction, the gimbal must tilt the thrust vector in the
; opposite direction to cancel the unwanted rotation.
;
; CODE-ALONG READERS: This section extracts the sign of the scaled AOS value
; and computes the absolute magnitude for drive time calculation. The drive
; direction is stored as +1 (positive AOS) or -1 (negative AOS) in NEGUQ/NEGUR.
; The absolute magnitude is stored in ITEMP2 for division by ACCDOT.
; ============================================================================

SETNEGU		EXTEND				# Check sign of scaled -AOS value
		BZMF	POSALPH			# Branch if -AOS ≥ 0 (i.e., AOS ≤ 0)

; Case 1: -AOS is negative (AOS is positive)
; Drive direction must be negative (-1) to counter positive acceleration.

		COM				# Complement to get +AOS (absolute value)
		TS	ITEMP2			# Store -ABS(0.4*AOS) scaled at PI/8
		CS	BIT1			# Load -1 for drive direction
		TCF	POSALPH +2		# Skip to common code

; Case 2: -AOS is zero or positive (AOS is zero or negative)
; Drive direction must be positive (+1) to counter negative acceleration.

POSALPH		TS	ITEMP2			# Store -ABS(0.4*AOS) scaled at PI/8
		CA	BIT1			# Load +1 for drive direction
	+2	INDEX	QRNDXER			# Select NEGUQ or NEGUR based on axis
		TS	NEGUQ			# Store SGN(AOS) as NEGU for this axis

; ============================================================================
; Drive time computation - Calculate how long to apply gimbal drive
;
; COMMENT-ONLY READERS: The computer calculates how many milliseconds to run
; the gimbal actuators to correct the attitude error. Longer drive times for
; larger errors, but never more than 2 seconds to prevent runaway motion.
; During Apollo 11's descent, these small corrections occurred continuously,
; keeping the engine precisely aligned without crew intervention.
;
; CODE-ALONG READERS: Compute drive time = ABS(0.4*AOS) / ACCDOT, with safety
; limit at 2 seconds. The algorithm first checks if the drive time would exceed
; 2 seconds by testing if 2*ACCDOT > ABS(0.4*AOS). If the check passes, it
; performs the actual division and scales the result to waitlist time units
; (2^14/100 seconds ≈ 163.84 seconds per unit, so 1 centisecond = 1.6384 units).
; ============================================================================

		CA	NZACCDOT		# Load ACCDOT (PI/2^7 radians/sec^3)
		EXTEND
		MP	BIT12			# Multiply by 2: 2*ACCDOT scaled at PI/8
		AD	ITEMP2			# Add: 2*ACCDOT - ABS(0.4*AOS), at PI/8
		EXTEND
		BZMF	NOTALLOW		# If result ≤ 0, drive time > 2 seconds

; Drive time is acceptable (<2 seconds). Perform the division:
; DRIVE_TIME = ABS(0.4*AOS) / ACCDOT
; The OCT00240 constant (decimal 10/1024 ≈ 0.00977) provides additional scaling
; to convert the result into waitlist time units (2^14/100 seconds per unit).

		CS	ITEMP2			# Load +ABS(0.4*AOS) (negate the negative)
		EXTEND				# Prepare for multiplication
		MP	OCT00240		# Scale: ABS(0.4*AOS) * (10/1024)
		EXTEND				# Prepare for division
		DV	NZACCDOT		# Divide by ACCDOT: result in waitlist units
# Page 1480
		EXTEND				# Check if result is zero
		BZF	TGOFFNOW		# Zero drive time means no correction needed

; Drive time computed successfully and is non-zero. Store in appropriate timer.

		TCF	DRIVEON			# Store time in QGIMTIMR or RGIMTIMR

; ============================================================================
; TGOFFNOW - Disable gimbal drive for this axis
;
; Reached when: ACCDOT ≤ 0, AOS = 0, or computed drive time = 0.
; Action: Set NEGU to zero (no drive command) and proceed to completion check.
; ============================================================================

TGOFFNOW	CAF	ZERO			# Zero drive command (gimbal off)
		INDEX	QRNDXER			# Select NEGUQ or NEGUR
		TS	NEGUQ			# Store zero (disables gimbal for axis)

		TCF	DONEYET			# Check if both axes complete

; ============================================================================
; NOTALLOW - Drive time exceeds 2-second safety limit
;
; COMMENT-ONLY READERS: When the required correction is too large, the computer
; refuses to apply it automatically. Instead, it sets a 2-second timer and
; disables further automatic gimbal control until the monitoring task (AOSTASK)
; re-evaluates the situation. This prevents runaway gimbal motion that could
; destabilize the spacecraft.
;
; CODE-ALONG READERS: The 2-second limit check failed (2*ACCDOT < ABS(0.4*AOS)),
; indicating the computed drive time would exceed the safety threshold. Set the
; gimbal timer to OCT31 (≈2 seconds in waitlist units) and clear ALLOWGTS flag
; to disable GTS attitude-rate control. AOSTASK must explicitly re-enable GTS
; after validating that conditions are safe for gimbal control.
; ============================================================================

NOTALLOW	CAF	OCT31			# Load maximum timer value (≈2 seconds)
		INDEX	QRNDXER			# Select QGIMTIMR or RGIMTIMR
		TS	QGIMTIMR		# Set timer to 2-second maximum
		CAF	ZERO			# Clear ALLOWGTS flag
		TS	ALLOWGTS		# Disable GTS attitude-rate control
						# (requires AOSTASK re-approval)
		TCF	DONEYET			# Check if both axes complete

; ============================================================================
; DRIVEON - Store computed drive time in gimbal timer
;
; A register contains the drive time in waitlist units (2^14/100 seconds).
; Store this value in QGIMTIMR (Q-axis) or RGIMTIMR (R-axis) depending on
; QRNDXER index. The timer will count down during each TIME5 interrupt,
; and the gimbal drive will automatically terminate when the timer reaches zero.
; ============================================================================

DRIVEON		INDEX	QRNDXER			# Select axis timer
		TS	QGIMTIMR		# Store drive time in QGIMTIMR or RGIMTIMR

; ============================================================================
; DONEYET - Check if both axes have been processed
;
; QRNDXER = 2: R-axis just completed, loop back to process Q-axis
; QRNDXER = 0: Q-axis just completed, both axes done, call ACDT+C12 and return
; ============================================================================

DONEYET		CCS	QRNDXER			# Check axis counter
		TCF	TIMQGMBL

		DXCH	RUPTREG3		# PROTECT IBNKCALL ERASABLES.  ACDT+C12
		DXCH	ITEMP2			# LEAVES ITEMPS2,3 ALONE.

		TC	IBNKCALL		# TURN OFF CHANNEL BITS, SET Q(R)ACCDOTS.
		CADR	ACDT+C12

		DXCH	ITEMP2			# RESTORE ERASABLES FOR IBNKCALL.
		DXCH	RUPTREG3

		TC	RUPTREG2		# RETURN TO CALLER.

OCT00240	OCTAL	00240			# DECIMAL 10/1024

; ============================================================================
; TRANSITION: From Channel Control to Square Root Computation
;
; The trim gimbal control system requires computing square roots to evaluate
; the drive setting algorithm. The GTSQRT subroutine provides a specialized
; square root function optimized for the AGC's fixed-point arithmetic. This
; mathematical foundation enables the computer to determine the precise gimbal
; angles needed to keep the engine thrust vector aligned with the spacecraft's
; constantly shifting center of gravity as propellant is consumed.
; ============================================================================

; SQUARE ROOT SUBROUTINE FOR GIMBAL TRIM CALCULATIONS
;
; COMMENT-ONLY READERS: The computer must calculate square roots to determine
; proper engine gimbal angles. This mathematical routine handles the fixed-point
; arithmetic limitations of the AGC hardware while maintaining precision needed
; for accurate thrust vector control during powered descent and ascent.
;
; CODE-ALONG READERS: GTSQRT implements a specialized square root algorithm for
; double-precision fixed-point values. It handles AGC-specific scaling challenges,
; returning both the square root value and a shift factor (SHFTFLAG) that must be
; applied to obtain the true result. The algorithm processes the 14 most significant
; bits to balance precision with computational efficiency.

# Page 1481
# THE FOLLOWING SECTION IS A CONTINUATION OF THE TRIM GIMBAL CONTROL FROM THE LAST GTS ENTRY. THE QUANTITY NEGUSUM
# IS COMPUTED FOR EACH AXIS (Q,R), .707*DEL*FUNCTION(3/2) + K2THETA = NEGUSUM.  NEW DRIVES ARE ENTERED TO CH 12.
#
# THE SUBROUTINE GTSQRT ACCEPTS A DOUBLE PRECISION VALUE IN FUNCTION, FUNCTION +1 AND RETURNS A SINGLE-PRECISION
# SQUARE ROOT OF THE FOURTEEN MOST SIGNIFICANT BITS OF THE ARGUMENT.  ALSO, THE CELL SHFTFLAG CONTAINS A BINARY
# EXPONENT S, SUCH THAT THE SQUARE ROOT (RETURNED IN THE A REGISTER) MUST BE SHIFTED RIGHT (MULTIPLIED BY 2 TO THE
# POWER (-S)) IN ORDER TO BE THE TRUE SQUARE ROOT OF THE FOURTEEN MOST SIGNIFICANT BITS OF FUNCTION, FUNCTION +1.
# SQUARE ROOT ERROR IS NOT MORE THAN 2 IN THE 14TH SIGNIFICANT BIT.  CELLS CLOBBERED ARE A,L,SHFTFLAG,ININDEX,
# HALFARG,SCRATCH,SR,FUNCTION, FUNCTION +1.  GTSQRT IS CALLED BY TC GTSQRT AND RETURNS VIA TC Q OR TC FUNCTION +1.
# ZERO OR NEGATIVE ARGUMENTS YIELD ZERO FOR SQUARE ROOTS.

; GTSQRT ENTRY POINT - Argument Validation Phase
; The subroutine first validates the input argument stored in the double-precision
; cells FUNCTION and FUNCTION+1. Since square roots of negative numbers are
; undefined in real arithmetic, the routine returns zero for any negative or zero
; argument. This validation prevents mathematical errors during gimbal calculations.

GTSQRT		CCS	FUNCTION
		TCF	GOODARG		# FUNCTION is positive.  Take square root.
		TCF	+2		# High order word is zero.  Check lower word.
		TCF	ZEROOT		# Negative argument.  Return zero result.

; Check the lower word when high-order word is zero.
; For double-precision values, if the high word is zero, the magnitude might
; still be non-zero if the low word contains data. This handles values smaller
; than 1.0 in the scaled representation.

		CA	FUNCTION +1
		EXTEND
		BZMF	ZEROOT		# Branch if zero or minus - return zero

		TCF	ZEROHIGH	# Non-zero low word - proceed with calculation

; ZEROOT - Return zero for invalid arguments.
; When the input is zero or negative, the gimbal control algorithm treats this
; as a null condition requiring no square root computation. This simplifies
; downstream calculations and prevents numerical instability.

ZEROOT		CA	ZERO
		TS	SHFTFLAG	# Clear shift flag (no scaling needed)
		TC	Q		# Return to caller with zero in A

; ZEROHIGH - Handle case where only low word has data.
; The 14 most significant bits reside in the lower word. Swap the words to
; position the significant bits in the high word for standard processing.
; Set SHFTFLAG to 7 to account for the implicit scaling difference.

ZEROHIGH	XCH	FUNCTION	# 14 most significant bits are in the
		XCH	FUNCTION +1	# lower word. Exchange them for processing.
		CA	SEVEN		# Shift factor = 7 for this bit position
		TCF	GOODARG +1	# Continue to main square root algorithm

; GOODARG - Main square root algorithm entry for positive arguments.
; Initialize the scaling loop which normalizes the argument to an optimal
; range for the Newton-Raphson iteration. The AGC's fixed-point arithmetic
; requires careful scaling to maintain precision while avoiding overflow.

GOODARG		CA	ZERO
		TS	SHFTFLAG	# Initialize shift flag (will track scaling)
		CA	TWELVE		# Initialize the scaling loop counter.
		TS	ININDEX		# ININDEX = 12 (starting index)
		TCF	SCALLOOP	# Enter scaling loop

; SCALSTRT - Scaling determined, ready for root computation.
; The argument has been scaled to an optimal range. Load FUNCTION into A
; register and proceed to the final scaling step before Newton iteration.

SCALSTRT	CA	FUNCTION
		TCF	SCALDONE	# Jump to final scaling and root calculation

; MULBUSH - "Around the mulberry bush" - iterative scaling refinement.
; This whimsically-named section (typical of AGC programmers' humor) performs
; iterative range checking to determine the optimal scaling factor. Each pass
; through the loop compares the argument against progressively smaller reference
; values (powers of 4) to bracket the magnitude.

MULBUSH		CA	NEG2		# If arg is not less than 1/4, index is
		ADS	ININDEX		# zero, indicating no shift needed.
		EXTEND			# Decrement index by 2 (next power of 4)
		BZMF	SCALSTRT	# Branch if arg not less than 1/4.
					# Otherwise compare arg with reference
					# which is 4 times larger than last.

; SCALLOOP - Iterative scaling loop to normalize argument.
; Compares argument magnitude against reference values (powers of 2) stored
; in indexed locations. The loop determines the optimal scaling factor to
; bring the argument into the range [1/4, 1) for accurate square root computation.

SCALLOOP	CS	FUNCTION	# Complement of FUNCTION for comparison
		INDEX	ININDEX		# Index into BIT15 reference table
		AD	BIT15		# Reference magnitude less or equal to 1/4
		EXTEND
		BZMF	MULBUSH		# If arg not less than reference, go
					# around the mulberry bush once more.

; Argument is less than reference - scaling factor found.
; The ININDEX value now represents the optimal scaling for square root computation.
; Load the scale magnitude and perform division to normalize the argument.

# Page 1482
		INDEX	ININDEX
		CA	BIT15		# This is the scale magnitude 2**(-ININDEX)
		XCH	HALFARG		# Save scale divisor to HALFARG
		EXTEND			# Rescale the argument via division.
		DCA	FUNCTION	# Load double-precision argument
		EXTEND
		DV	HALFARG		# Divide by scale factor to normalize
					# ININDEX AND SHFTFLAG preserve info for
					# rescaling after root process.

; SCALDONE - Argument scaled, begin Newton-Raphson square root iteration.
; This section implements a three-iteration Newton-Raphson method to compute
; the square root. The algorithm uses the iterative formula:
;     X(n+1) = (X(n) + HALFARG/X(n)) / 2
; where HALFARG is the scaled argument divided by 2. Three iterations provide
; sufficient precision for gimbal control calculations.

SCALDONE	EXTEND
		QXCH	FUNCTION +1	# Save return address Q in FUNCTION+1
		EXTEND
		MP	BIT14		# Multiply scaled arg by 0.5 (BIT14 = 0.5)
		TS	HALFARG		# Store half of argument for Newton formula
		MASK	BIT13		# Test magnitude with BIT13 mask
		CCS	A		# Check if argument requires special handling
		CA	OCT11276	# Large argument: use OCT11276 as initial guess
		AD	ROOTHALF	# Initial guess is √(1/2) or POSMAX
		TC	ROOTCYCL	# First Newton iteration
		TC	ROOTCYCL	# Second Newton iteration
		TC	ROOTCYCL	# Third Newton iteration (sufficient precision)
		TC	FUNCTION +1	# Return via saved address in FUNCTION+1
# ****************************************************************************************************************

; ============================================================================
; RSTOFGTS - "Rest of GTS" - Complete NEGUSUM calculation after square root
;
; This section completes the gimbal trim system drive setting algorithm after
; the square root operation returns. It computes the product term:
;     K^(1/2) * (DEL*OMEGA + ALPHA^2/(2*K))
; and combines it with:
;     DEL * (DEL*OMEGA + ALPHA^2/(2*K))
; These terms are part of the NEGUSUM algorithm for determining gimbal drive
; direction during powered descent or ascent when the engine is providing
; thrust vector control.
; ============================================================================

RSTOFGTS	TC	GTSQRT		# Call square root subroutine
PRODUCT		XCH	K2CNTRAL	# Exchange: A ← K2CNTRAL, K2CNTRAL ← sqrt
		EXTEND
		MP	K2CNTRAL	# Multiply: sqrt * K2CNTRAL
		DXCH	K2CNTRAL	# Store double-precision product
		EXTEND			# Prepare for multiplication
		MP	L		# Multiply by L register
		ADS	K2CNTRAL +1	# Add to low word: K^(1/2)*(DEL*OMEGA + ALPHA^2/(2*K))
		TS	L		# Save result to L register
		TCF	+2		# Skip overflow correction if no overflow
		ADS	K2CNTRAL	# Handle overflow into high word
					# Product term: DEL*(DEL*OMEGA + ALPHA^2/(2*K))
					# now accumulated in K2CNTRAL

; DOSHIFT - Apply scaling correction from square root computation.
; The GTSQRT subroutine returns scaling information in ININDEX that must be
; applied to reverse the normalization performed before square root calculation.
; This section rescales the result back to the original magnitude range.

DOSHIFT		CA	ININDEX		# Load scaling index from GTSQRT
		EXTEND			# Multiply in factor 2^(-S), returned
		MP	BIT14		# by the GTSQRT subroutine (×0.5)
		ADS	SHFTFLAG	# Add to shift flag to combine scalings
		EXTEND
		BZF	ADDITIN		# If zero, no multiplicative scaling needed
		INDEX	SHFTFLAG	# Index into power-of-2 table
		CA	BIT15		# Load 2^SHFTFLAG for scaling multiplication
# Page 1483
		XCH	K2CNTRAL	# Exchange: prepare for scaled multiply
		EXTEND
		MP	K2CNTRAL	# Scale high word of K2CNTRAL
		DAS	K2THETA		# Add to K2THETA (double-precision)
		XCH	K2CNTRAL	# Get K2CNTRAL back for low word scaling
		EXTEND
		MP	K2CNTRAL +1	# Scale low word of K2CNTRAL
		ADS	K2THETA  +1	# Add to low word of K2THETA
		TS	L		# Save for overflow handling
		TCF	 +2		# Skip overflow correction if no overflow
		ADS	K2THETA		# Handle overflow into high word

		TCF	NEGUSUM		# Jump to sign test and drive decision

; ADDITIN - Simple addition path when no multiplicative scaling needed.
; If SHFTFLAG is zero, the K2CNTRAL term can be added directly to K2THETA
; without multiplicative scaling correction.

ADDITIN		EXTEND
		DCA	K2CNTRAL	# Load double-precision K2CNTRAL
		DAS	K2THETA		# Add directly to K2THETA term
; NEGUSUM - Final NEGUSUM sign test determines gimbal drive direction.
; The sign of NEGUSUM (negative of U-sum in control theory) determines whether
; the engine gimbal should drive positive or negative to correct attitude errors.
; This is the final step of the drive setting algorithm:
;     DRIVE = -SGN(NEGUSUM)
; where NEGUSUM includes attitude error, rate, and acceleration terms.

NEGUSUM		CCS	K2THETA		# Test sign of high order word
		TCF	NEGDRIVE	# Positive: drive negative (NEGUSUM positive)
		TCF	 +2		# +0: check low word
		TCF	POSDRIVE	# Negative: drive positive (NEGUSUM negative)

		CCS	K2THETA  +1	# High word zero, test low word sign
NEGDRIVE	CA	BIT1		# NEGUSUM positive: drive = -1 (negative)
		TCF	+2		# Skip to reversal test
POSDRIVE	CS	BIT1		# NEGUSUM negative: drive = +1 (positive)
					# (Stop gimbal for zero NEGUSUM via +2 skip)
		TS	L		# Save new drive command for reversal test
		INDEX	QRCNTR		# Index to Q or R axis NEGU register
		XCH	NEGUQ		# Exchange: A ← old NEGU, NEGUQ ← new NEGU

; Reversal detection prevents gimbal oscillation.
; If the drive command reverses sign (positive to negative or vice versa),
; insert a zero-drive pause to prevent mechanical resonance and propellant waste.
; This implements hysteresis in the gimbal control law.

		EXTEND
		MP	L		# Multiply old NEGU * new NEGU
		CCS	L		# Check sign of product
		TCF	LOUPE		# Positive: same sign, continue driving

		TCF	ZEROLOUP	# Zero: no reversal issue

		TCF	REVERSAL	# Negative: drive reversed, insert pause
		TCF	ZEROLOUP	# -0: no reversal issue

; REVERSAL - Insert zero-drive pause when gimbal drive direction reverses.
; When the drive command reverses sign (old NEGU * new NEGU < 0), this
; subroutine prevents gimbal oscillation by inserting a pause. During this
; pause, the acceleration term is zeroed and gimbal drive bits are cleared
; from channel 12, allowing mechanical settling before the reversed drive begins.
; This hysteresis prevents propellant waste from rapid drive reversals.

REVERSAL	INDEX	QRCNTR		# Index to Q or R axis acceleration register
		TS	QACCDOT		# Zero acceleration term (A contains zero from CCS)
		INDEX	QRCNTR		# Index to gimbal drive bit mask for this axis
		CS	GMBLBITA	# Load complement of gimbal drive bit
		EXTEND
		WAND	CHAN12		# AND with channel 12: clear gimbal drive bits
					# Stops engine gimbal actuator for this axis

; ZEROLOUP - Request acceleration update and channel 12 update call.
; After determining gimbal drive (or zero-drive pause), set flag to request
; ACDT+C12 call, which will update acceleration estimates and write gimbal
; drive commands to the engine actuators via channel 12.

ZEROLOUP	CS	RCSFLAGS	# Load complement of RCS flags
		MASK	CALLGMBL	# Mask CALLGMBL bit (request for ACDT+C12 call)
		ADS	RCSFLAGS	# Add to RCSFLAGS: set request bit
# Page 1484
; LOUPE - Axis iteration control.
; Check if both Q and R axes have been processed. If QRCNTR is positive (2),
; R-axis was just computed, so loop back to compute Q-axis. If QRCNTR is zero,
; both axes complete, so restore status register and close out the task.

LOUPE		CCS	QRCNTR		# Check axis counter: 2 (R done) or 0 (Q done)?
		TCF	GOQTRIMG	# Positive: R-axis done, compute Q-axis next

		CA	SAVESR		# Restore status register (saved at GTS entry)
		TS	SR		# SR contains interrupt state and flags

; GOCLOSE - Terminate the gimbal trim task.
; Both Q and R axes have been processed. Restore the status register and
; transfer control to CLOSEOUT routine to terminate this DAP task (JASK).
; This completes one iteration of the trim gimbal control system.

GOCLOSE		EXTEND			# Double-precision transfer control
		DCA	CLOSEADR	# Load 2CADR of CLOSEOUT routine
		DTCB			# Transfer control to CLOSEOUT in correct bank

		EBANK=	AOSQ		# E-bank assignment for CLOSEOUT
CLOSEADR	2CADR	CLOSEOUT	# Address of task termination routine

TWELVE		EQUALS	OCT14
ROOTHALF	OCTAL	26501		# SQUARE ROOT OF 1/2
GMBLBITA	OCTAL	01400		# INDEXED WRT GMBLBITB   DO NOT MOVE******
OCT11276	OCTAL	11276		# POSMAX - ROOTHALF
GMBLBITB	OCTAL	06000		# INDEXED WRT GMBLBITA   DO NOT MOVE******

# SUBROUTINE ROOTCYCL:	BY CRAIG WORK,3 APRIL 68
# ROOTCYCL IS A SUBROUTINE WHICH EXECUTES ONE NEWTON SQUARE ROOT ALGORITHM ITERATION.  THE INITIAL GUESS AT THE
# SQUARE ROOT IS PRESUMED TO BE IN THE A REGISTER AND ONE-HALF THE SQUARE IS TAKEN FROM HALFARG.  THE NEW APPROXI-
; ============================================================================
; ROOTCYCL - Single Newton-Raphson iteration for square root computation.
;
; This subroutine performs one iteration of the Newton-Raphson square root
; algorithm:  X_new = (X + ARG/X) / 2
;
; The algorithm iteratively refines an estimate X to converge toward sqrt(ARG).
; Each iteration approximately doubles the number of correct significant digits.
;
; INPUTS:
;   A register: X (current estimate of square root)
;   HALFARG: ARG/2 (half of the value whose square root is being computed)
;
; OUTPUTS:
;   A register: Refined approximation to sqrt(ARG)
;
; DEBRIS: A, L, SR, SCRATCH
;
; CALLING SEQUENCE:
;   TC  ROOTCYCL     (called from address LOC)
;   (returns to LOC+1 via TC Q)
;
; WARNING: If initial guess X is not greater than sqrt(ARG), divide or add
; overflow may occur. GTSQRT ensures proper initial guess through scaling.
; ============================================================================

ROOTCYCL	TS	SCRATCH		# Store current estimate X
		TS	SR		# Store X/2 in SR (shift right divides by 2)
		CA	HALFARG		# Load ARG/2 into A
		ZL			# Clear L register for division
		EXTEND
		DV	SCRATCH		# Divide: (ARG/2) / X = (ARG/X)/2
		AD	SR		# Add X/2: result = (X + ARG/X)/2
		TC	Q		# Return refined estimate in A
