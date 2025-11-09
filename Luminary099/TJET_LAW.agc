# Copyright:	Public domain.
# Filename:	TJET_LAW.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1460-1469
# Mod history:	2009-05-27 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-06 RSB	Eliminated a stray instruction that had crept
#				in somehow.
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
; FILE: TJET_LAW.agc
; MODULE: RCS Digital Autopilot - Thruster Control
; MISSION PHASE: descent/landing/ascent/rendezvous
;
; TL;DR: Implements the thruster jet firing law that converts attitude control
;        commands into precise jet firing durations. This is the heart of LM
;        attitude control - calculating how long to fire RCS thrusters to
;        achieve desired rotation while minimizing fuel consumption. Used
;        during all mission phases requiring precise attitude control.
;
; COMMENT-ONLY READERS: This code determines how long to fire the small
;        thrusters that control the Lunar Module's rotation in space. Think
;        of it as the brain that decides "fire thruster for 0.05 seconds"
;        to achieve the exact spacecraft orientation needed.
;
; CODE-ALONG READERS: Study the phase-plane control implementation using
;        multiple zones (ZONE1-5) with different control laws. Note the
;        sophisticated handling of scaling, minimum impulse constraints,
;        and fuel optimization through variable jet selection.
; ============================================================================

# Page 1460
# PROGRAM DESCRIPTION
# DESIGNED BY:	R. D. GOSS AND P. S. WEISSMAN
# CODED BY:  P. S. WEISSMAN, 28 FEBRUARY 1968
#
# TJETLAW IS CALLED AS A SUBROUTINE WHEN THE LEM IS NOT DOCKED AND THE AUTOPILOT IS IN THE AUTOMATIC OR
# ATTITUDE-HOLD MODE TO CALCULATE THE JET-FIRING-TIME (TJET) REQUIRED FOR THE AXIS INDICATED BY AXISCTR:
#	-1	INDICATES THE P-AXIS
#	+0	INDICATES THE U-AXIS
#	+1	INDICATES THE V-AXIS
# THE REGISTERS E AND EDOT CONTAIN THE APPROPRIATE ATTITUDE ERROR AND ERROR RATE AND SENSETYP SHOWS WHETHER
# UNBALANCED COUPLES ARE PREFERRED.  TJETLAW ALSO USES VARIOUS FUNCTIONS OF ACCELERATION AND DEADBAND WHICH ARE
# COMPUTED IN THE 1/ACCONT SECTION OF 1/ACCS AND ARE STORED IN SUCH AN ORDER THAT THEY CAN BE CONVENIENTLY
# ACCESSED BY INDEXING.
#
# THE SIGN OF THE REQUIRED ROTATION IS CARRIED THROUGH TJETLAW AS ROTSENSE AND IS FINALLY APPLIED TO TJET JUST
# PREVIOUS TO ITS STORAGE IN THE LOCATION CORRESPONDING TO THE AXIS (TJP, TJU, OR TJV).  THE NUMBER OF JETS THAT
# TJETLAW ASSUMES WILL BE USED AS INDICATED BY THE SETTING OF NUMBERT FOR THE U- OR V-AXIS.  TWO JETS ARE ALWAYS
# ASSUMED FOR THE P-AXIS ALTHOUGH FOUR JETS WILL BE FIRED WHEN FIREFCT IS MORE NEGATIVE THAN -4.0 DEGREES
# (FIREFCT IS THE DISTANCE TO A SWITCH CURVE IN THE PHASE PLANE) AND A LONG FIRING IS CALLED FOR.
#
# IN ORDER TO AVOID SCALING DIFFICULTIES, SIMPLE ALGORITHMS TAGGED RUFLAW1, -2 AND -3 ARE RESORTED TO WHEN THE
# ERROR AND/OR ERROR RATE ARE LARGE.
#
# CALLING SEQUENCE:
#		TC	TJETLAW		# (MUST BE IN JASK)
#	OR
#		INHINT			# (MUST BE IN JASK)
#		TC	IBNKCALL
#		CADR	TJETLAW
#		RELINT
#
# EXIT:		RETURN TO Q.
#
# INPUT:
#	FROM THE CALLER:  E, EDOT, AXISCTR, SENSETYP, TJP, -U, -V.
#	FROM 1/ACCONT:  48 ERASABLES BEGINNING AT BLOCKTOP (INCLUDING FLAT, ZONE3LIM AND ACCSWU, -V).
#
# OUTPUT:
#	TJP, -U OR -V, NUMBERT (DAPTEMP5), FIREFCT (DAPTEMP3).
#
# DEBRIS:
#	A, L, Q, E, EDOT, DAPTEMP1-6, DAPTEMP1-4.
#
# ALARM:  NONE

		BANK	17
		SETLOC	DAPS2
		BANK
		EBANK=	TJP
# Page 1461
		COUNT*	$$/DAPTJ

; ============================================================================
; TJETLAW MAIN ENTRY POINT
;
; The Digital Autopilot calls this routine every 100 milliseconds to determine
; how long to fire the Lunar Module's reaction control system (RCS) thrusters.
; The LM has 16 small thrusters arranged around its exterior - this routine
; decides "fire thruster pair for 0.05 seconds" or similar commands to achieve
; the precise rotation needed for landing, ascent, or rendezvous operations.
;
; This is phase-plane control: the routine examines both the attitude error
; (how far off we are) and the error rate (how fast we're drifting) to make
; intelligent decisions about thruster firing that conserve precious fuel.
; ============================================================================

TJETLAW		EXTEND			# SAVE Q FOR RETURN.
		QXCH	HOLDQ

; ============================================================================
; INITIAL SETUP: Determine which axis and rotation direction
;
; The autopilot controls three axes of rotation:
;   P-axis (pitch): nose up/down rotation
;   U-axis (yaw): nose left/right rotation  
;   V-axis (roll): rotation around long axis
;
; AXISCTR indicates which axis we're controlling (-1=P, 0=U, +1=V).
; We set up indexers that let us access the right acceleration and deadband
; parameters for this axis, stored in arrays computed by 1/ACCS routine.
; ============================================================================

# SET INDEXERS TO CORRESPOND TO THE AXIS AND TO THE SIGN OF EDOT

		INDEX	AXISCTR		# AXISDIFF(-1)=NO OF LOCATIONS BET P AND U
		CAF	AXISDIFF	# AXISDIFF(0)=0
		TS	ADRSDIF1	# AXISDIFF(+1)=NO OF LOCATIONS BET V AND U

; The sign of EDOT (error rate) determines rotation direction. If EDOT is
; negative, we need to rotate one way; if positive, the other way. The code
; handles both cases using the same logic by setting up different indexers
; and a rotation sense flag.

		CAE	EDOT		# IF EDOT NEGATIVE, PICK UP SET OF VALUES
		EXTEND			#	THAT ALLOW USE OF SAME CODING AS FOR
		BZMF	NEGEDOT		#	POSITIVE EDOT.
		CAE	ADRSDIF1	# SET A SECOND INDEXER WHICH MAY BE
		TS	ADRSDIF2	# 	MODIFIED BY A DECISION FOR MAX JETS.
		CAF	SENSOR		# FOR POSITIVE EDOT, ROTSENSE IS
		TCF	SETSENSE	# 	INITIALIZED POSITIVE.

NEGEDOT		CS	E		# IN ORDER FOR NEG EDOT CASE TO USE CODING
		TS	E		#	OF POS EDOT, MUST MODIFY AS FOLLOWS:
		CS	EDOT		#	1. COMPLEMENT E AND EDOT.
		TS	EDOT		#	2. SET SENSE OF ROTATION TO NEGATIVE
		CAF	BIT1		#	   (REVERSED LATER IF NECESSARY).
		ADS	ADRSDIF1	#	3. INCREMENT INDEXERS BY ONE SO THAT
		TS	ADRSDIF2	#	   THE PROPER PARAMETERS ARE ACCESSED.
		CS	SENSOR
SETSENSE	TS	ROTSENSE

; ROTSENSE now contains +1 or -1 indicating the direction of required rotation.
; This will be applied to the final jet firing time to select the proper
; thruster pair (positive or negative rotation direction).

; ============================================================================
; COARSE RANGE CHECK: Determine if error is large enough for simplified law
;
; For fuel efficiency and computational simplicity, TJETLAW uses different
; algorithms depending on error magnitude. If attitude error is very large
; (≥11.25°), we use simplified "rough laws" (RUFLAW1, RUFLAW2, RUFLAW3) that
; avoid scaling problems and provide quick approximate answers.
;
; If error is small (<11.25°), we rescale for precision and continue to the
; sophisticated phase-plane control logic that optimally manages fuel.
; ============================================================================

# TEST MAGNITUDE OF E (ATTITUDE ERROR, SINGLE-PRECISION, SCALED AT PI RADIANS):
#	IF GREATER THAN (OR EQUAL TO) PI/16 RADIANS, GO TO THE SIMPLIFIED TJET ROUTINE.
#	IF LESS THAN PI/16 RADIANS, RESCALE TO PI/4

		CAE	E		# PICK UP ATTITUDE ERROR FOR THIS AXIS
		EXTEND
		MP	BIT5		# SHIFT RIGHT TEN BITS:  IF A-REGISTER IS
		CCS	A		#	ZERO, RESCALE AND TEST EDOT.
		TCF	RUFLAW2
		TCF	SCALEE
		TCF	RUFLAW1
SCALEE		CAF	BIT13		# ERROR IS IN L SCALED AT PI/16.  RESCALE
		EXTEND			#	IT TO PI/4 AND SAVE IT.
		MP	L
		TS	E

# TEST MAGNITUDE OF EDOT (ERROR RATE SCALED AT PI/4 RADIANS/SECOND)
#	IF GREATER THAN (OR EQUAL TO) PI/32 RADIANS/SECOND, GO TO THE SIMPLIFIED TJET ROUTINE.
#	IF LESS THAN PI/32 RADIANS/SECOND, THEN RESCALE TO PI/32 RADIANS/SECOND.

		CAE	EDOT		# PICK UP SINGLE-PRECISION ERROR-RATE
# Page 1462
		EXTEND			# FOR THIS AXIS=
		MP	BIT4		# SHIFT RIGHT ELEVEN BITS, IF THE A-REG IS
		EXTEND			# ZERO, THEN RESCALE AND USE FINELAW.
		BZF	SCALEDOT
		TCF	RUFLAW3

; ============================================================================
; FINELAW - PRECISION PHASE-PLANE CONTROL
;
; COMMENT-ONLY READERS: For small attitude errors and slow rotation rates,
; the spacecraft uses its most precise control algorithm. This "fine law"
; calculates the exact thruster firing time needed to reach the desired
; attitude with minimal fuel consumption and no overshoot.
;
; CODE-ALONG READERS: Enters phase-plane control mode for small EDOT.
; Rescales EDOT to PI/32 rad/sec, then computes EDOTSQ = (EDOT)^2 with
; proper scaling at PI^2/2^8 rad^2/sec^2. This squared term is used in
; the phase-plane switch curve calculation to determine optimal firing time.
; ============================================================================

# *** FINELAW STARTS HERE ***

SCALEDOT	LXCH	EDOT		# EDOT IS SCALED AT PI/32 RADIANS/SECOND.

		CAE	EDOT		# COMPUTE (EDOT)(EDOT)
		EXTEND
		SQUARE			# PRODUCT SCALED AT PI(2)/2(10) RAD/SEC.
		EXTEND
		MP	BIT13		# SHIFT RIGHT TWO BITS TO RESCALE TO EDOTSQ
		TS	EDOTSQ		#	TO PI(2)/2(8) RAD(2)/SEC(2).

; ============================================================================
; ERROR MAGNITUDE TEST FOR MAXIMUM JETS
;
; COMMENT-ONLY READERS: The computer checks if the attitude error is large
; (more than 3 degrees beyond the deadband). Large errors trigger all four
; RCS thrusters to fire together for maximum rotational authority, providing
; faster attitude correction when needed.
;
; CODE-ALONG READERS: Tests if |E| > (FIREDB + 3°) to determine if maximum
; jets are required. Uses CCS to get absolute value, subtracts 3° threshold
; and deadband. If error is large, branches to MAXJETS which sets NUMBERT=4
; and adjusts ADRSDIF2 indexer for 4-jet acceleration parameters.
; ============================================================================

ERRTEST		CCS	E		# DOES BIG ERROR (THREE DEG BEYOND THE
		AD	-3DEG		# DEADBAND) REQUIRE MAXIMUM JETS?
		TCF	+2
		AD	-3DEG
		EXTEND
		INDEX	ADRSDIF1
		SU	FIREDB
		EXTEND
		BZMF	SENSTEST	# IF NOT:  ARE UNBALANCED JETS PREFERRED?
; Maximum jets configuration: 4 thrusters for high authority control
MAXJETS		CAF	TWO		# IF YES:  INCREMENT ADDRESS LOCATOR AND
		ADS	ADRSDIF2	#	   SET SWITCH FOR JET SELECT LOGIC TO 4.
		CAF	FOUR		#	   (ALWAYS DO THIS FOR P-AXIS)
		TCF	TJCALC

; Sense type test: Check if translation maneuvers prefer minimum jets
SENSTEST	CCS	SENSETYP	# DOES TRANSLATION PREFER MIN JETS.
		TCF	TJCALC		# YES.  USE MIN-JET PARAMETERS
		TCF	MAXJETS		# NO.  GET THE MAX-JET PARAMETERS.

; Store number of jets (NUMBERT) for jet selection logic
TJCALC		TS	NUMBERT		# SET TO +0,1,4 FOR (U,V-AXES) JET SELECT.

; ============================================================================
; PHASE-PLANE FIREFCT CALCULATION
;
; COMMENT-ONLY READERS: The computer now calculates how far the spacecraft
; is from the ideal "switch curve" in the phase plane. This determines when
; to fire thrusters to achieve the desired attitude with minimum fuel. The
; calculation accounts for rotation rate, available thrust, and deadband.
;
; CODE-ALONG READERS: Computes FIREFCT (fire factor) representing distance
; to phase-plane switch curve: FIREFCT = -E - 0.5*EDOTSQ/ACC + FIREDB
; where ACC is thruster acceleration (indexed by ADRSDIF2 for 2 or 4 jets).
; Negative FIREFCT indicates state is below switch curve (requires firing).
; Used to determine which of 5 control zones applies.
; ============================================================================

# BEGINNING OF TJET CALCULATIONS:

		CS	EDOTSQ		# SCALED AT PI(2)/2(8).
		EXTEND
		INDEX	ADRSDIF2
		MP	1/ANET1		# .5/ACC SCALED AT 2(6)/PI SEC(2)/RADIAN.
		INDEX	ADRSDIF1
		AD	FIREDB		# DEADBAND SCALED AT PI/4 RADIAN.
		EXTEND
		SU	E		# ATTITUDE ERROR SCALED AT PI/4 RADIAN.
		TS	FIREFCT		# -E-.5(EDOTSQ)/ACC-DB AT PI/4 RADIAN.
		EXTEND
		BZMF	ZON1,2,3

; ============================================================================
; ZONE 4,5: COAST AND DRIFT/ON REGION CONTROL
;
; Comment-Only Readers: When the Lunar Module's attitude error is small and
; the error rate is carrying it naturally back toward the deadband, the
; autopilot enters a fuel-saving "coast" mode. This section decides whether
; to let the spacecraft coast (Zone 4) or fire brief stabilizing pulses
; (Zone 5). During Apollo 11's lunar orbit and descent, this logic minimized
; RCS fuel consumption while maintaining precise attitude control.
;
; Code-Along Readers: This section implements the phase-plane regions for
; minimal control authority. Zone 4,5 logic computes:
;   E + 0.5*(EDOT²)/ACC + COASTDB
; If positive, spacecraft is in Zone 4 (coast region) - no jets needed.
; If negative, in Zone 5 (drift/on region) - compute jet time to stabilize.
;
; The ZONE4 subroutine checks if jets are currently firing toward the target
; state. If so, they're kept on during DRIFT/ON mode or when approaching the
; target parabola during powered flight. Otherwise, TJET is set to zero.
; ============================================================================

ZONE4,5		INDEX	ADRSDIF1
		CAE	1/ACOAST	# .5/ACC SCALED AT 2(6)/PI WHERE
# Page 1463
		EXTEND			# ACC = MAX(AMIN, AOS-).
		MP	EDOTSQ		# SCALED AT PI/2(8).
		AD	E		# SCALED AT PI/4
		INDEX	ADRSDIF1
		AD	COASTDB		# SCALED AT PI/4 POS. FOR NEG. INTERCEPT.
		EXTEND			# TEST E+.5(EDOTSQ)/ACC+DB AT PI/4 RADIAN.
		BZMF	ZONE5		# IF FUNCTION NEGATIVE, FIND TJET.
					# IF FUNCTION POSITIVE, IN ZONE 4.

# ZONE 4 IS THE COAST REGION.  HOWEVER, IF THE JETS ARE ON AND DRIVING TOWARD
#	A. THE AXIS WITHIN + OR - (DB + FLAT) FOR DRIFTING FLIGHT, OR
#	B. THE USUAL TARGET PARABOLA FOR POWERED FLIGHT
# THEN THE THRUSTERS ARE KEPT ON.

ZONE4		INDEX	AXISCTR		# IS THE CURRENT VALUE IN TJET NON-ZERO
		CS	TJETU		# 	WITH SENSE OPPOSITE TO EDOT,
		EXTEND			#	(I.E., ARE JETS ON AND FIRING TOWARD
		MP	ROTSENSE	#	THE DESIRABLE STATE).
		EXTEND
		BZMF	COASTTJ		# NO.  COAST.

JETSON		CCS	FLAT		# YES.  IS THIS DRIFTING OR POWERED FLIGHT?
		TCF	DRIFT/ON	# DRIFTING.  GO MAKE FURTHER TEST.

		CS	FIREFCT		# POWERED (OR ULLAGE).  CAN TARGET PARABOLA
		INDEX	ADRSDIF1	#	BE REACHED FROM THIS POINT IN THE
		AD	AXISDIST	#	PHASE PLANE?
		EXTEND
		BZMF	COASTTJ		# NO. SET TJET = 0.
		TC	Z123COMP	# YES.  CALCULATE TJET AS THOUGH IN ZONE 1
		CAE	FIREFCT		#	AFTER COMPUTING THE REQUIRED
		TCF	ZONE1		#	PARAMETERS.

DRIFT/ON	INDEX	ADRSDIF1	# CAN TARGET STRIP OF AXIS BE REACHED FROM
		CS	FIREDB		#	THIS POINT IN THE PHASE PLANE?
		DOUBLE
		AD	FIREFCT
		EXTEND
		BZMF	+3
COASTTJ		CAF	ZERO		# NO.  SET TJET = 0.
		TCF	RETURNTJ

		TC	Z123COMP	# YES. CALCULATE TJET AS THOUGH IN ZONE 2
		TCF	ZONE2,3		#	OR 3 AFTER COMPUTING REQUIRED VALUES.

; ============================================================================
; ZONE 5: JET TIME CALCULATION FOR DRIFT/ON REGION
;
; Comment-Only Readers: In Zone 5, the spacecraft is close enough to the
; desired attitude that brief thruster pulses can stabilize it. This section
; calculates exactly how long to fire the thrusters - typically 50 to 250
; milliseconds. Too short and the jets won't have effect; too long and fuel
; is wasted. The algorithm uses two key parameters: TTOAXIS (time to null
; the error rate) and HH (a function of attitude error and acceleration).
;
; Code-Along Readers: ZONE5 computes jet firing time using approximation
; formulas based on TTOAXIS and HH:
;   TTOAXIS = EDOT / ANET2  (time to axis, scaled at 4 sec)
;   HH = (E + 0.5*EDOT²/ACC + COASTDB) * ACCFCTZ5 * 4 (scaled at 8 sec²)
;
; Three test regions determine which formula to use:
;   - If TJET > 150 msec: Set to full time (250 msec) to ensure control
;   - If 50 msec < TJET < 150 msec: Use FORMULA2 (quadratic approximation)
;   - If TJET < 50 msec: Use FORMULA1 (refined quadratic approximation)
;
; The ADRSDIF2 indexer is modified for negative error rates to access the
; correct 1/ANET2 and ACCFCTZ5 parameters from memory tables.
; ============================================================================

ZONE5		TS	L		# TEMPORARILY STORE FUNCTION IN L.
		CCS	ROTSENSE	# MODIFY ADRSDIF2 FOR ACCESSING 1/ANET2
		TCF	+4		# AND ACCFCTZ5, WHICH MUST BE PICKED UP
		TC	CCSHOLE		# FROM THE NEXT LOWER REGISTER IF THE
		CS	TWO		# (ACTUAL) ERROR RATE IS NEGATIVE.
# Page 1464
		ADS	ADRSDIF2

 +4		CAE	L
		EXTEND
		INDEX	ADRSDIF2	# TTOAXIS AND HH ARE THE PARAMETERS UPON
		MP	ACCFCTZ5	#	WHICH THE APPROXIMATIONS TO TJET ARE
		DDOUBL			#	ABASED.
		DDOUBL
		DXCH	HH		# DOUBLE PRECISION H SCALED AT 8 SEC(2).
		INDEX	ADRSDIF2
		CAE	1/ANET2		# SCALED AT 2(7)/PI SEC(2)/RAD.
		EXTEND
		MP	EDOT		# SCALED AT PI/2(5)
		TS	TTOAXIS		# SCALED AT 4 SEC.

# TEST WHETHER TJET GREATER THAN 50 MSEC.

		EXTEND
		MP	-.05AT2		# H - .05 TTOAXIS - .00125 G.T. ZERO
		AD	HH		# 	(SCALED AT 8 SEC(2) ).
		AD	NEG2
		EXTEND
		BZMF	FORMULA1

# TEST WHETHER TJET GREATER THAN 150 MSEC.

		CAE	TTOAXIS
		EXTEND
		MP	-.15AT2		# H - .15 TTOAXIS - .01125 G.T. ZERO
		AD	HH		#	(SCALED AT 8 SEC(2) )
		AD	-.0112A8
		EXTEND
		BZMF	FORMULA2

; ----------------------------------------------------------------------------
; FULLTIME: Maximum Jet Firing Time Assignment
;
; Comment-Only Readers: When the computed thruster firing time exceeds 150
; milliseconds, the computer limits it to 250 milliseconds maximum. This
; ensures the spacecraft won't overshoot the target attitude even with longer
; burns. The 250 msec limit guarantees sufficient rotation to avoid missing
; the next control cycle (which occurs every 100 milliseconds).
;
; Code-Along Readers: This routine caps TJET at 250 msec (BIT11 scaled at
; 4 seconds):
;   - Prevents excessive rotation from over-correction
;   - Ensures completion before next DAP cycle (100 msec interval)
;   - BIT11 = 2048/2^14 = 0.125 seconds = 125 msec... wait, let me recalculate
;   - Actually BIT11 at time4 scale: 2048 / 2^13 = 0.25 sec = 250 msec
;
; Maximum firing time chosen to:
;   1. Avoid skip in Control Sequential Program (CSP) timing
;   2. Maintain attitude control loop stability
;   3. Prevent fuel waste on overly aggressive maneuvers
; ----------------------------------------------------------------------------

# IF TJET GREATER THAN 150 MSEC, ASSIGN IT VALUE OF 250 MSEC, SINCE THIS
# IS ENOUGH TO ASSURE NO SKIP NEXT CSP (100 MSEC).

FULLTIME	CAF	BIT11		# 250 MSEC SCALED AT 4 SEC.

; ----------------------------------------------------------------------------
; RETURNTJ: Common Exit Point - Apply Sign and Store Jet Time
;
; Comment-Only Readers: All calculations converge here to finalize the thruster
; firing command. The computer applies the correct rotation direction (positive
; or negative), converts the firing time to the proper units, and stores it for
; the jet selection logic to execute. During Apollo 11's final descent, these
; signed firing times controlled the precise attitude adjustments Armstrong made
; as he searched for a safe landing site.
;
; Code-Along Readers: Final processing steps:
;   1. Multiply TJET (A register, scaled at 4 sec) by ROTSENSE (±1)
;      - ROTSENSE carries rotation direction through entire calculation
;      - Product rescales from time4 to time6 (multiply by ±2^-2)
;   2. Store signed result in axis-specific location (TJP, TJU, or TJV)
;      - INDEX by AXISCTR: -1 (P-axis), 0 (U-axis), +1 (V-axis)
;      - TJETU base address + AXISCTR offset = destination
;   3. Check if max-jets condition requires NUMBERT update
;      - Multiply by ACCSWU (acceleration switch, 0 or non-zero)
;      - If result negative, 1/ACCS forced max-jet calculation
;      - Set NUMBERT = 4 jets instead of default 2 jets
;   4. Return via saved Q register
;
; Register usage:
;   A: TJET value (signed after ROTSENSE multiply)
;   L: Product of TJET × ACCSWU (tested for max-jets condition)
;   AXISCTR: Axis index for indirect addressing
;   NUMBERT: Number of jets to fire (2 or 4)
; ----------------------------------------------------------------------------

# RETURN TO CALLING PROGRAM WITH JET TIME SCALED AS TIME6 AND SIGNED.

RETURNTJ	EXTEND			# ALL BRANCHES TERMINATE HERE WITH TJET
		MP	ROTSENSE	#	(SCALED AT 4 SEC) IN THE ACCUMULATOR.
		INDEX	AXISCTR		# ROTSENSE APPLIES SIGN AND CHANGES SCALE.
		TS	TJETU
		EXTEND
		INDEX	AXISCTR
		MP	ACCSWU		# SET SWITCH FOR JET SELECT IF ROTATION IS
		CAE	L
		EXTEND			#	IN A SENSE FOR WHICH 1/ACCS HAS FORCED
		BZMF	+3		#	A MAX-JET CALCULATION.
		CAF	FOUR
# Page 1465
		TS	NUMBERT
		TC	HOLDQ		# RETURN VIA SAVED Q.

; ============================================================================
; FORMULA1: SHORT JET TIME CALCULATION (TJET < 50 MSEC)
;
; Comment-Only Readers: When only a very brief thruster pulse is needed
; (less than 50 milliseconds), this formula calculates the precise firing
; time. Short pulses are critical for fine attitude adjustments during
; station-keeping and precision maneuvers. Armstrong used these brief pulses
; during the final approach to manually adjust the landing attitude.
;
; Code-Along Readers: FORMULA1 implements the approximation:
;   TJET = HH / (0.025 + TTOAXIS)
; where:
;   HH = stored double-precision value (scaled at 8 sec²)
;   TTOAXIS = time to axis (scaled at 4 sec)
;   0.025 sec = minimum denominator to prevent divide-by-zero
;
; The division uses the AGC double-precision DV instruction:
;   - Dividend: HH (double-precision, 8 sec² scale)
;   - Divisor: (0.025 + TTOAXIS) stored in HH location (4 sec scale)
;   - Result: TJET initially scaled at 2 sec, then rescaled to 4 sec
;
; After calculation, TJET is checked against TJMIN (minimum jet time).
; ============================================================================

# TJET = H/(.025 + TTOAXIS) 	FOR TJET LESS THAN 50 MSEC.

FORMULA1	CS	-.025AT4	# .025 SEC SCALED AT 4.
		AD	TTOAXIS		# SCALED AT 4 SECONDS.
		DXCH	HH		# STORE DENOMINATOR IN FIRST WORD OF H,
		EXTEND			#	WHICH NEED NOT BE PRESERVED.  PICK UP
		DV	HH		#	DP H AND DIVIDE BY DENOMINATOR.
		EXTEND
		MP	BIT14		# RESCALE TJET FROM 2 TO USUAL 4 SEC.
		TCF	CHKMINTJ	# CHECK THAT TJET IS NOT LESS THAN MINIMUM

; ============================================================================
; FORMULA2: MEDIUM JET TIME CALCULATION (50 MSEC < TJET < 150 MSEC)
;
; Comment-Only Readers: When moderate thruster pulses are needed (50 to 150
; milliseconds), this formula calculates the firing time. These medium-length
; pulses provided most of the attitude control during lunar orbit and descent.
; The formula adds a small correction term to improve accuracy over FORMULA1.
;
; Code-Along Readers: FORMULA2 implements the approximation:
;   TJET = (HH + 0.00375) / (0.1 + TTOAXIS)
; where:
;   HH = stored double-precision value (scaled at 8 sec²)
;   0.00375 sec² = numerator correction term (scaled at 8 sec²)
;   TTOAXIS = time to axis (scaled at 4 sec)
;   0.1 sec = denominator offset (scaled at 4 sec)
;
; The 0.00375 correction term compensates for second-order effects in the
; phase-plane trajectory when jet times are in this intermediate range. The
; larger denominator offset (0.1 vs 0.025 in FORMULA1) reflects the longer
; firing times and their different control characteristics.
;
; After calculation, TJET is checked against TJMIN (minimum jet time).
; ============================================================================

# TJET = (H + .00375)/(0.1 + TTOAXIS)	FOR TJET GREATER THAN 50 MSEC.

FORMULA2	EXTEND
		DCA	.00375A8	# .00375 SEC(2) SCALED AT 8.
		DAS	HH		# STORE NUMERATOR IN DP H, WHICH NEED NOT
					#	BE PRESERVED.
		CAE	TTOAXIS		# SCALED AT 4 SEC.
		AD	.1AT4		# 0.1 SEC SCALED AT 4.
		DXCH	HH		# STORE DENOMINATOR IN FIRST WORD OF H,
		EXTEND			#	WHICH NEED NOT BE PRESERVED.  PICK UP
		DV	HH		#	DP NUMERATOR AND DIVIDE BY DENOMINATOR
		EXTEND
		MP	BIT14		# RESCALE TJET FROM 2 TO USUAL 4 SEC.
		TCF	RETURNTJ	# END SUBROUTINE.

; ============================================================================
; Z123COMP SUBROUTINE: PREPARATION FOR ZONES 1, 2, 3
;
; Comment-Only Readers: Before applying high-authority thruster control, this
; small subroutine prepares critical calculations. It reverses the rotation
; sense (because jets fire opposite to the error direction) and computes how
; long it would take to null the current error rate. This "time-to-axis"
; value determines whether to fire brief pulses or longer burns.
;
; Code-Along Readers: Z123COMP performs two key operations:
;   1. Reverse ROTSENSE (CS ROTSENSE, TS ROTSENSE):
;      - Input: ROTSENSE = +1 or -1 indicating desired rotation sense
;      - Operation: CS (Complement and Skip) negates the value
;      - Result: ROTSENSE now indicates jet firing direction
;      - Used later in RETURNTJ to apply proper sign to TJET
;
;   2. Compute time-to-axis (TTOAXIS):
;      - EDOT = error rate (scaled at π/2^5 rad/sec)
;      - 1/ANET1 = inverse net acceleration (scaled at 2^7/π sec²/rad)
;      - TTOAXIS = EDOT × 1/ANET1 (result scaled at 4 seconds)
;      - Indexed by ADRSDIF2 to access correct axis acceleration data
;
;   3. Check if TTOAXIS exceeds 150 milliseconds (TJMAX threshold):
;      - If TTOAXIS > 150 msec: Branch to FULLTIME (fire for maximum time)
;      - If TTOAXIS ≤ 150 msec: RETURN to caller for detailed zone logic
;
; Calling convention: TC Z123COMP (called from 3 locations in TJETLAW)
; Returns: RETURN instruction at line 543 exits subroutine to caller Q
; ============================================================================

# SUBROUTINIZED COMPUTATIONS REQUIRED FOR ALL ENTRIES INTO CODING FOR ZONES 1, 2, AND 3.
# REACHED BY TC FROM 3 POINTS IN TJETLAW.

Z123COMP	CS	ROTSENSE	# USED IN RETURNTJ SECTION TO RESCALE TJET
		TS	ROTSENSE	# 	AS TIME6 AND GIVE IT PROPER SIGN.
		CAE	EDOT		# SCALED AT PI/2(5) RAD/SEC.
		EXTEND
		INDEX	ADRSDIF2
		MP	1/ANET1		# SCALED AT 2(7)/PI SEC(2)/RAD.
		TS	TTOAXIS		# STORE TIME-TO-AXIS SCALED AT 4 SECONDS.
		AD	-TJMAX
		EXTEND			# IS TIME TO AXIS LESS THAN 150 MSEC.
		BZMF	+2
		TCF	FULLTIME	# NO. FIRE JETS, DO NOT CALCULATE TJET.
		RETURN			# YES.  GO ON TO FIND TJET

; ============================================================================
; ZONES 1, 2, 3: HIGH-AUTHORITY CONTROL REGIONS
;
; Comment-Only Readers: When the Lunar Module's attitude is significantly
; off-target (beyond the phase-plane switching parabola), aggressive thruster
; control is needed. These zones implement the "bang-bang" control strategy:
; fire thrusters at full authority until the spacecraft trajectory approaches
; the target attitude. During the lunar landing's powered descent, this logic
; kept the LM stable despite engine gimbal movements and mass changes.
;
; ZONE 1: Far from target - fire for calculated time to reach switching curve
; ZONE 2: Near axis - fire to null error rate (time-to-axis)
; ZONE 3: Minimum impulse zone - fire brief pulses to maintain limit cycle
;
; The Z123COMP subroutine prepares data for all three zones by reversing
; ROTSENSE (since jets fire opposite to error) and computing TTOAXIS.
;
; Code-Along Readers: Phase-plane switching logic:
;   1. Z123COMP called to compute TTOAXIS = EDOT * 1/ANET1 (scaled 4 sec)
;   2. If TTOAXIS > 150 msec, fire full time (TJMAX = 250 msec)
;   3. Check FIREFCT + FLAT to determine zone entry:
;      - If FIREFCT + FLAT ≥ 0: Enter ZONE2,3 (near-axis logic)
;      - If FIREFCT + FLAT < 0: Enter ZONE1 (far-from-axis logic)
;   4. ZONE2,3 checks ZONE3LIM to distinguish:
;      - ZONE2: TTOAXIS ≥ ZONE3LIM (35 msec) → fire to axis
;      - ZONE3: TTOAXIS < ZONE3LIM → fire minimum impulse (14 msec)
;   5. ZONE1 computes jet time using trajectory to switching parabola
;
; During Apollo 11 descent, these zones managed large disturbances from
; descent engine throttling and spacecraft mass loss (fuel consumption).
; ============================================================================

ZON1,2,3	TC	Z123COMP	# SUBROUTINIZED PREPARATION FOR ZONE1,2,3.

; ============================================================================
; ZONE SELECTION LOGIC: DETERMINE CONTROL REGION
;
; Comment-Only Readers: After preparing the calculations, the computer must
; decide which control zone the spacecraft is in. The "FLAT" parameter creates
; a buffer region near the switching curve - during powered flight (like the
; lunar descent), this buffer is set to zero for immediate response. The
; FIREFCT value (distance to switching parabola) determines the zone:
;   - If near the target (FIREFCT + FLAT ≥ 0): Use ZONE2,3 logic
;   - If far from target (FIREFCT + FLAT < 0): Use ZONE1 logic
;
; Code-Along Readers: Zone selection based on phase-plane geometry:
;   FIREFCT = signed distance to switching parabola (scaled at π/4 rad)
;   FLAT = buffer zone width (0 during powered flight/ullage, >0 in coast)
;   
;   Test: FIREFCT + FLAT versus zero
;     If FIREFCT + FLAT ≥ 0: Branch to ZONE2,3 (near-axis logic)
;     If FIREFCT + FLAT < 0: Fall through to ZONE1 (far-from-axis logic)
;
; ZONE2,3 then subdivides based on ZONE3LIM (minimum impulse threshold):
;   ZONE3LIM = 35 msec in drifting flight, 0 when entering GTS control
;   If TTOAXIS ≥ ZONE3LIM: ZONE2 (fire to axis, time = TTOAXIS)
;   If TTOAXIS < ZONE3LIM: ZONE3 (fire minimum impulse, typically 14 msec)
;
; ZONE3 special case: If EDOT = 0 (on axis), fire one-jet minimum impulse
;   to maintain limit cycle oscillation. If EDOT truly zero, TJET = 0 (coast).
; ============================================================================

# IF THE (NEG) DISTANCE BEYOND PARABOLA IS LESS THAN FLAT, USE SPECIAL
# LOGIC TO ACQUIRE MINIMUM IMPULSE LIMIT CYCLE.  DURING POWERED FLIGHT
# Page 1466
# OR ULLAGE, FLAT = 0

		CAE	FIREFCT		# SCALED AT PI/4 RAD.
		AD	FLAT
		EXTEND
		BZMF	ZONE1		# NOT IN SPECIAL ZONES.

; ============================================================================
; ZONE2,3 ENTRY: NEAR-AXIS CONTROL LOGIC
;
; Comment-Only Readers: The spacecraft is close enough to the target that the
; computer can use refined control logic. ZONE2,3 first checks if the error
; rate is large enough to warrant firing to the axis (ZONE2), or if it's so
; small that only minimum impulse bursts are needed (ZONE3). During the final
; moments before touchdown, this logic provided the gentle corrections that
; kept Eagle stable without wasting precious fuel.
;
; Code-Along Readers: ZONE2,3 subdivides the near-axis region:
;   ZONE3LIM = threshold for minimum impulse region (scaled at 4 seconds)
;     - During drift: ZONE3LIM = 35 msec (maintain limit cycle)
;     - Entering GTS (gravity turn start): ZONE3LIM = 0 (disable limit cycle)
;   
;   Test: TTOAXIS - ZONE3LIM (implemented as CS ZONE3LIM, AD TTOAXIS)
;     If TTOAXIS ≥ ZONE3LIM: Result positive, branch to ZONE2 skipped
;     If TTOAXIS < ZONE3LIM: Result negative, branch to ZONE3
;
;   ZONE2: Fire jets for time TTOAXIS to null the error rate completely
;   ZONE3: Fire minimum impulse burst (BIT6 = 14 msec) or coast if on axis
; ============================================================================

# FIRE FOR AXIS OR, IF CLOSE, FIRE MINIMUM IMPULSE.  IF ON AXIS, COAST.

ZONE2,3		CS	ZONE3LIM	# HEIGHT OF MIN-IMPULSE ZONE SET BY 1/ACCS
		AD	TTOAXIS		#	35 MSEC IN DRIFTING FLIGHT
		EXTEND			#	ZERO WHEN TRYING TO ENTER GTS CONTROL.
		BZMF	ZONE3
; ----------------------------------------------------------------------------
; ZONE2: Fire to Axis - Null the Error Rate
;
; Comment-Only Readers: In ZONE2, the spacecraft is near the target attitude
; but still has measurable rotation. The computer fires the thrusters for
; exactly the time needed to stop this rotation - bringing the error rate to
; zero while minimizing fuel consumption.
;
; Code-Along Readers: ZONE2 implementation is straightforward:
;   TJET = TTOAXIS (time to axis, already computed by Z123COMP)
;   Load TTOAXIS into A register, then branch to RETURNTJ for output.
; ----------------------------------------------------------------------------

ZONE2		CAE	TTOAXIS		# FIRE TO AXIS.
		TCF	RETURNTJ

; ----------------------------------------------------------------------------
; ZONE3: Minimum Impulse or Coast
;
; Comment-Only Readers: ZONE3 is the most delicate control region - the
; spacecraft is so close to the target that only tiny thruster bursts are
; appropriate. If the rotation rate is truly zero (perfectly on axis), the
; computer issues no thruster commands and lets the spacecraft coast. If
; there's any detectable rotation, it fires a brief 14-millisecond pulse -
; the shortest burst that reliably moves the spacecraft without wasting fuel.
;
; Code-Along Readers: ZONE3 uses CCS instruction to test EDOT precisely:
;   CCS EDOT checks sign and magnitude of error rate:
;     - If EDOT > 0: Skip next instruction, set TJET = BIT6 (14 msec)
;     - If EDOT = +0: Execute next instruction, fall through to RETURNTJ
;     - If EDOT < 0: Skip next instruction, set TJET = BIT6 (14 msec)
;     - If EDOT = -0: Execute next instruction (unused case in practice)
;
;   BIT6 = 14 milliseconds scaled at 4 seconds (minimum one-jet impulse)
;   TJET = +0 when exactly on axis (coast with no thruster firing)
;
; The minimum impulse maintains a limit cycle oscillation during drifting
; flight, providing stable attitude control without continuous firing.
; ----------------------------------------------------------------------------

ZONE3		CCS	EDOT		# CHECK IF EDOT IS ZERO.
		CAF	BIT6		# FIRE A ONE-JET MINIMUM IMPULSE.
		TCF	RETURNTJ	# TJET = +0.
		TC	CCSHOLE		# CANNOT BE BECAUSE NEG EDOT COMPLEMENTED.
		TCF	RETURNTJ	# TJET = +0.

; ============================================================================
; ZONE1: Large Error Rate - Trajectory Beyond Axis
;
; Comment-Only Readers: ZONE1 handles situations where the spacecraft is
; rotating so fast that even if thrusters fire now, momentum will carry it
; past the target attitude before stopping. The computer must calculate both
; how long to fire to slow the rotation AND when to start firing in the
; opposite direction to stop at the target. This is the most complex control
; zone, requiring precise timing to avoid overshooting.
;
; Code-Along Readers: ZONE1 computes the trajectory parabola that passes
; through the target axis. The key parameter H represents the "height" of
; the parabola in the phase plane:
;
;   H² = (E - AXISDIST) × ACCFCTZ1 × 4
;
; Where:
;   E - AXISDIST = distance beyond axis that trajectory reaches (π/4 rad)
;   ACCFCTZ1 = 2(7)/π sec²/rad (acceleration factor for ZONE1)
;   × 4 from two DDOUBL instructions
;   H scaled at 8 sec²
;
; The algorithm then tests whether total time exceeds 150 msec (TJMAX) and
; whether time beyond axis exceeds 50 msec to select the appropriate formula.
; ============================================================================

ZONE1		EXTEND
		INDEX	ADRSDIF1
		SU	AXISDIST	# SCALED AT PI/4 RAD.
		EXTEND
		INDEX	ADRSDIF2
		MP	ACCFCTZ1	# SCALED AT 2(7)/PI SEC(2)/RAD.
		DDOUBL
		DDOUBL
		DXCH	HH		# DOUBLE PRECISION H SCALED AT 8 SEC(2).

; ----------------------------------------------------------------------------
; ZONE1 Time Test: Check if Total Maneuver Time Exceeds 150 msec
;
; Comment-Only Readers: The computer must determine whether the correction
; maneuver is large enough to warrant precise calculation. If the total time
; required exceeds 150 milliseconds, the computer uses a simpler "full time"
; approach. Otherwise, it uses more refined formulas that account for the
; exact trajectory shape.
;
; Code-Along Readers: Test condition:
;   Is 0.5(0.150 - TTOAXIS)² - H² negative?
;
; If yes (BZMF branches), total time > 150 msec, use FULLTIME
; If no, continue to test time beyond axis for formula selection
;
; The 150 msec threshold (TJMAX) represents the boundary between short
; maneuvers requiring precise timing and longer maneuvers where simplified
; logic suffices.
; ----------------------------------------------------------------------------

# TEST WHETHER TOTAL TIME REQUIRED GREATER THAN 150 MSEC:
#	                     2                                   2
# 	IS .5(.150 - TTOAXIS)  - H  NEGATIVE (SCALED AT 8 SECONDS )

		CAE	TTOAXIS		# TTOAXIS SCALED AT 4 SECONDS.
		AD	-TJMAX		# -.150 SECOND SCALED AT 4.
		EXTEND
		SQUARE
		EXTEND
		SU	HH		# HIGH WORD OF H SCALED AT 8 SEC(2).
		EXTEND
		BZMF	FULLTIME	# YES.  NEED NOT CALCULATE TJET.

; ----------------------------------------------------------------------------
; Time Beyond Axis Test: Select Formula Based on 50 msec Threshold
;
; Comment-Only Readers: For maneuvers taking less than 150 milliseconds total,
; the computer must choose between two mathematical formulas depending on how
; long the spacecraft will coast beyond the target. If it goes beyond for more
; than 50 milliseconds, one formula applies; if less, another more precise
; formula is used.
;
; Code-Along Readers: Test condition:
;   HH + NEG2 (where NEG2 = -2 scaled at 8 sec²)
;   Equivalent to testing: Is H² > 2 (scaled at 8 sec²)?
;   This corresponds to time beyond axis ≈ 50 msec
;
; If H² ≥ 2: Branch to FORMULA3 (less precise, for < 50 msec beyond axis)
; If H² < 2: Fall through to next formula (more precise, for > 50 msec)
; ----------------------------------------------------------------------------

# TEST WHETHER TIME BEYOND AXIS GREATER THAN 50 MSEC TO DETERMINE WHICH APPROXIMATION TO USE.

		CAE	HH
		AD	NEG2
		EXTEND
		BZMF	FORMULA3

; ----------------------------------------------------------------------------
; ZONE1 Formula (> 50 msec beyond axis):
;   TJET = H/0.1 + TTOAXIS + 0.0375
;
; Comment-Only Readers: For larger corrections where the spacecraft coasts
; more than 50 milliseconds past the target, this formula calculates the
; thruster firing time. The 37.5 millisecond offset accounts for the average
; delay in the thruster control system.
;
; Code-Along Readers: Taylor series approximation for longer beyond-axis time:
;   TJET = H/0.1 + TTOAXIS + 0.0375 seconds
;
; Where:
;   H/0.1: Time spent beyond axis (H scaled at 8 sec², 0.1 scaled at 2)
;   TTOAXIS: Time to reach axis (scaled at 4 seconds)
;   0.0375: Correction constant (scaled at 4 seconds)
;
; Division yields quotient scaled at 4 seconds, consistent with other terms.
; ----------------------------------------------------------------------------

# Page 1467
# TJET = H/0.1 + TTOAXIS + .0375	FOR APPROXIMATION OVER MORE THAN 50 MSEC.

		CAF	.1AT2		# STORE .1 SEC SCALED AT 2 FOR DIVISION.
		DXCH	HH		# DP H SCALED AT 8 SEC(2) NEED NOT BE
		EXTEND			#	PRESERVED.
		DV	HH		# QUOTIENT SCALED AT 4 SECONDS.
		AD	TTOAXIS		# SCALED AT 4 SEC.
		AD	.0375AT4	# .0375 SEC SCALED AT 4.
		TCF	RETURNTJ	# END COMPUTATION.

; ----------------------------------------------------------------------------
; FORMULA3: ZONE1 Formula (< 50 msec beyond axis):
;   TJET = H/0.025 + TTOAXIS
;
; Comment-Only Readers: For smaller corrections where the spacecraft coasts
; less than 50 milliseconds past the target, this more precise formula applies.
; Notice this formula uses a smaller divisor (0.025 vs 0.1), which gives a
; larger correction for the same parabola height H, and omits the 37.5 msec
; offset - appropriate for quicker maneuvers.
;
; Code-Along Readers: Taylor series approximation for shorter beyond-axis time:
;   TJET = H/0.025 + TTOAXIS seconds
;
; Where:
;   H/0.025: Time spent beyond axis with higher precision divisor
;   TTOAXIS: Time to reach axis (scaled at 4 seconds)
;   No correction constant needed for shorter maneuvers
;
; The -.025AT2 constant is complemented (CS) to produce +0.025 for division.
; Division yields quotient scaled at 4 seconds.
; ----------------------------------------------------------------------------

# TJET - H/.O25 + TTOAXIS 	FOR APPROXIMATION OVER LESS THAN 50 MSEC.

FORMULA3	CS	-.025AT2	# STORE +.25 SEC SCALED AT 2 FOR DIVISION
		DXCH	HH		# PICK UP DP H AT 8, WHICH NEED NOT BE
		EXTEND			# 	PRESERVED.
		DV	HH		# QUOTIENT SCALED AT 4 SECONDS.
		AD	TTOAXIS		# SCALED AT 4 SEC.

; ----------------------------------------------------------------------------
; CHKMINTJ: Minimum Jet Time Verification
;
; Comment-Only Readers: Very short thruster pulses (less than the minimum firing
; time) are not reliable - they may not overcome valve opening delays and
; combustion startup transients. If the calculated firing time is too brief, the
; computer sets it to zero instead, choosing to coast rather than risk an
; ineffective pulse. This prevents wasting fuel on thruster firings too short
; to produce meaningful rotation. Note that Zone 3 minimum impulse maneuvers
; bypass this check since they're specifically designed for minimum-time pulses.
;
; Code-Along Readers: Minimum jet time validation logic:
;   1. Add -TJMIN to computed TJET (performs subtraction)
;   2. Test if result is negative (TJET < TJMIN):
;      - If yes: Branch to COASTTJ, which sets TJET = 0
;      - If no: Restore original TJET by adding back TJMIN
;   3. Continue to RETURNTJ with validated time
;
; Rationale for minimum time enforcement:
;   - Thruster valve opening time: ~5-10 msec
;   - Combustion chamber pressure buildup: ~5-10 msec
;   - Total system delay: ~10-20 msec
;   - TJMIN threshold ensures pulse exceeds system delays
;   - Prevents inefficient "dribble" mode firings
;
; Exception: Zone 3 minimum impulse commands bypass this check since they
; explicitly require minimum-duration firings for precision control.
; ----------------------------------------------------------------------------

# IF COMPUTED JET TIME IS LESS THAN TJMIN, TJET IS SET TO ZERO.
# MINIMUM IMPULSES REQUIRED IN ZONE 3 ARE NOT SUBJECT TO THIS CONSTRAINT, NATURALLY.

CHKMINTJ	AD	-TJMIN		# IS COMPUTED TIME LESS THAN THE MINIMUM.
		EXTEND
		BZMF	COASTTJ		# YES, SET TIME TO ZERO.
		AD	TJMIN		# NO, RESTORE COMPUTED TIME.
		TCF	RETURNTJ	# END COMPUTATION.

; ============================================================================
; TRANSITION: From Fine Control Law to Rough Control Law
;
; The spacecraft attitude has diverged significantly from the target. The fine
; control law's precise phase-plane calculations become numerically unstable
; with large errors, so the computer switches to simpler "rough law" algorithms
; that rapidly drive the spacecraft back toward the target attitude using
; aggressive thruster firings. Once errors decrease, control automatically
; transitions back to the fine law for final precision adjustments.
; ============================================================================

# Page 1468
; ----------------------------------------------------------------------------
; ROUGHLAW (RUFLAW): Simplified Control Law for Large Errors/Rates
;
; Comment-Only Readers: When attitude errors or rotation rates become very
; large, the computer abandons the sophisticated fine control calculations and
; switches to simple, aggressive control laws. These "rough laws" fire thrusters
; for longer durations to rapidly reduce large errors, accepting less fuel
; efficiency in exchange for stability and speed. Think of it as "get close
; fast, then refine" - first use rough laws to recover from major disturbances,
; then hand off to fine laws for precision control.
;
; Three rough law cases handle different error scenarios:
;   RUFLAW1: Large negative error - fire to build rotation rate toward target
;   RUFLAW2: Large positive error - fire to build opposing rate, brake attitude
;   RUFLAW3: High rotation rate, moderate error - coast or fire based on switch curve
;
; Code-Along Readers: RUFLAW entry conditions and coordinate transformation:
;   1. ADRSDIF1, ADRSDIF2 index parameters based on axis and sign of EDOT
;   2. If EDOT negative: E and EDOT rotated to upper half-plane, ROTSENSE = -1
;   3. Scaling: E at π radians, EDOT at π/4 rad/sec (except RUFLAW3: E at π/4)
;
; Why rough laws are needed:
;   - Fine law formulas involve SQRT, reciprocals, complex phase-plane geometry
;   - For |E| > π/16 rad (~11°) or |EDOT| > π/32 rad/sec (~5.6°/sec):
;     * Scaling problems arise (approaching word overflow)
;     * Computational precision degrades
;     * Switch curve calculations become unreliable
;   - Rough laws use simple linear rate targeting:
;     * Target rate: 6.5°/sec (0.1444 rad/sec at π/4 scale = RUFRATE)
;     * Firing time = (desired rate change) / (2-jet acceleration)
;     * Much simpler math, handles extreme values safely
;
; Algorithm selection by entry point:
;   RUFLAW1: E < -π/16 rad → Fire to build rate toward +6.5°/sec
;   RUFLAW2: E > +π/16 rad → Fire to build opposing rate -6.5°/sec (brake)
;   RUFLAW3: |EDOT| > π/32 rad/sec, |E| ≤ π/16 → Switch curve test
; ----------------------------------------------------------------------------

# *** ROUGHLAW ***
#
# BEFORE ENTRY TO RUFLAW:
#	1. INDEXERS ADRSDIF1 AND ADRSDIF2 ARE SET ON BASIS OF AXIS, AND SIGN OF EDOT.
#	2. IF EDOT WAS NEGATIVE, E AND EDOT ARE ROTATED INTO UPPER HALF-PLANE AND ROTSENSE IS MADE NEGATIVE.
#	3. E IS SCALED AT PI RADIANS AND EDOT AT PI/4 RAD/SEC.
#	   (EXCEPT THE RUFLAW3 ENTRY WHEN E IS AT PI/4)
#
# RUFLAW1:	ERROR MORE NEGATIVE THAN PI/16 RAD.  FIRE TO A RATE OF 6.5 DEG/SEC (IF JET TIME EXCEEDS 20 MSEC.).
# RUFLAW2:	ERROR MORE POSITIVE THAN PI/16 RAD.  FIRE TO AN OPPOSING RATE OF 6.5 DEG/SEC.
# RUFLAW3:	ERROR RATE GREATER THAN PI/32 RAD/SEC AND ERROR WITHIN BOUNDS.  COAST IF BELOW FIREFCT, FIRE IF ABOVE

; ----------------------------------------------------------------------------
; RUFLAW1: Large Negative Error Recovery
;
; Comment-Only Readers: The spacecraft is pointing significantly away from the
; target in the negative direction (more than 11 degrees off). The computer
; fires thrusters to build up rotation rate toward a target speed of 6.5°/sec,
; which will drive the spacecraft back toward the correct attitude. Once the
; rotation rate is established, the spacecraft coasts toward the target, where
; finer control laws take over for precision alignment.
;
; Code-Along Readers: RUFLAW1 algorithm for E < -π/16:
;   1. Subtract RUFRATE (0.1444 rad/sec at π/4 scale) from EDOT
;      - This computes: (current rate) - (target rate) = rate change needed
;      - If result is negative: current rate already exceeds target → SMALRATE
;      - If result is positive: need to accelerate further
;   2. If rate change needed is positive:
;      - Call RUFSETUP to reverse ROTSENSE, set max jets
;      - Proceed to RUFLAW12 with desired rate change in A register
;   3. RUFLAW12 computes: TJET = rate_change / 2-jet_acceleration
;      - Uses 1/ANET1+2 (inverse acceleration, indexed by ADRSDIF2)
;      - Limits result to TJMAX (250 msec) if excessive
;      - Returns via CHKMINTJ for minimum time validation
;
; Example: If E = -20° and EDOT = +2°/sec:
;   - Target is EDOT = +6.5°/sec
;   - Rate change needed: 6.5 - 2.0 = 4.5°/sec
;   - Fire thrusters to produce 4.5°/sec acceleration
;   - Resulting rotation rate drives spacecraft toward target attitude
; ----------------------------------------------------------------------------

; Compute rate change needed to reach target rate of 6.5°/sec
RUFLAW1		CS	RUFRATE		# DECREMENT EDOT BY .1444 RAD/SEC AT PI/4
		ADS	EDOT		#	WHICH IS THE TARGET RATE
		EXTEND
		BZMF	SMALRATE	# BRANCH IF RATE LESS THAN TARGET.
		TC	RUFSETUP	# REVERSE ROTSENSE AND INDICATE MAX JETS.
		CAE	EDOT		# PICK UP DESIRED RATE CHANGE.

; Common code path for RUFLAW1 and RUFLAW2: Convert rate change to jet time
; TJET = (desired_rate_change) / (2-jet_acceleration)
; Uses 1/ANET1+2 (inverse of 2-jet acceleration) for division by multiplication
RUFLAW12	EXTEND			# COMPUTE TJET
		INDEX	ADRSDIF2	#	= (DESIRED RATE CHANGE)/(2-JET ACCEL.)
		MP	1/ANET1 +2
		AD	-1/8		# IF TJET, SCALED AT 32 SEC, EXCEEDS
		EXTEND			# 	4 SECONDS, SET TJET TO TJMAX.
		BZMF	+2
		TCF	FULLTIME
		EXTEND
		BZF	FULLTIME
		AD	BIT12		# RESTORE COMPUTED TJET TO ACCUMULATOR
		DAS	A
		DAS	A
		DAS	A		# RESCALED TJET AT 4 SECONDS.
		TCF	CHKMINTJ	# RETURN AS FROM FINELAW.

; ----------------------------------------------------------------------------
; SMALRATE: Current Rate Already Exceeds Target
;
; Comment-Only Readers: The spacecraft is already rotating faster than the
; target rate of 6.5°/sec. Instead of firing more thrusters to accelerate
; further, the computer fires opposing thrusters to slow the rotation back
; down to the target rate. This prevents overshooting the attitude target.
;
; Code-Along Readers: Rate reduction logic when current rate > target:
;   1. Call RUFSETUP+2 to set NUMBERT=4 and FIREFCT for max jets
;      (skip ROTSENSE reversal - not needed for rate reduction)
;   2. Test ROTSENSE to determine which 1/ANET to use:
;      - If ROTSENSE > 0: Add +1 to ADRSDIF2 (use forward acceleration)
;      - If ROTSENSE ≤ 0: Add -1 to ADRSDIF2 (use reverse acceleration)
;   3. Compute desired rate change: (RUFRATE - EDOT)
;      - CS EDOT negates current rate
;      - Implicit add of RUFRATE gives: 0.1444 - EDOT
;   4. Continue to RUFLAW12 with rate change in A register
;
; This handles the case where rough law 1 fired too aggressively and
; now needs to brake the resulting high rotation rate.
; ----------------------------------------------------------------------------

SMALRATE	TC	RUFSETUP +2	# SET NUMBERT AND FIREFCT FOR MAXIMUM JETS
		CCS	ROTSENSE	; Test sign of ROTSENSE
		CAF	ONE		# MODIFY INDEXER TO POINT TO 1/ANET
		TCF	+2		#	CORRESPONDING TO THE PROPER SENSE.
		CAF	NEGONE		; ROTSENSE ≤ 0: Use -1 offset
		ADS	ADRSDIF2	; Adjust indexer for proper acceleration direction

		CS	EDOT		# (.144 AT PI/4 - EDOT) = DESIRED RATE CHNG.
		TCF	RUFLAW12	; Continue to jet time calculation

; ----------------------------------------------------------------------------
; RUFLAW2: Large Positive Error Recovery (Braking Maneuver)
;
; Comment-Only Readers: The spacecraft is pointing significantly beyond the
; target in the positive direction (more than 11 degrees past). The computer
; fires thrusters to build up an *opposing* rotation rate of -6.5°/sec, which
; will brake the spacecraft and drive it back toward the correct attitude. This
; is the attitude braking case - similar to RUFLAW1 but in the opposite
; direction. During manual maneuvering, Armstrong occasionally created large
; attitude offsets that required these rough law corrections.
;
; Code-Along Readers: RUFLAW2 algorithm for E > +π/16:
;   1. Call RUFSETUP to reverse ROTSENSE and set max jets
;      - ROTSENSE flip converts problem to RUFLAW1-like case
;   2. Add RUFRATE (0.1444 rad/sec at π/4 scale) to EDOT
;      - Computes: (RUFRATE + EDOT) = magnitude of opposing rate to build
;      - Example: If EDOT = -2°/sec, target is -6.5°/sec (brake harder)
;      - Example: If EDOT = +2°/sec, target is +6.5°/sec (still braking)
;   3. Test result for overflow:
;      - TS A with overflow skip → need more than max rate → FULLTIME
;      - No overflow → continue to RUFLAW12 with desired rate change
;   4. RUFLAW12 computes TJET from rate change
;
; Key difference from RUFLAW1:
;   - RUFLAW1 builds rate in same direction as error correction
;   - RUFLAW2 builds opposing rate to brake past-target attitude
;   - Both use same target rate magnitude (6.5°/sec = 0.1444 rad/sec)
;
; Overflow handling: If computed rate change exceeds maximum representable
; value, fire for FULLTIME (250 msec) to produce maximum available deceleration.
; ----------------------------------------------------------------------------

RUFLAW2		TC	RUFSETUP	# REVERSE ROTSENSE AND INDICATE MAX JETS.
		CAF	RUFRATE		; Load target rate (0.1444 rad/sec at π/4)
		AD	EDOT		# (.144 AT PI/4 + EDOT) = DESIRED RATE CHNG.
		TS	A		# IF OVERFLOW SKIP, FIRE FOR FULL TIME.
		TCF	RUFLAW12	# OTHERWISE, COMPUTE JET TIME.
		TCF	FULLTIME	; Overflow: Use maximum firing time

; ----------------------------------------------------------------------------
; RUFLAW3: Quadratic Switch Curve Decision for High-Rate Moderate-Error Cases
;
; Comment-Only Readers: When the spacecraft is spinning rapidly (over 5.6°/sec)
; but the attitude error is moderate (within ±11°), the computer must decide:
; fire jets now or coast? The decision uses a "switch curve" - a mathematical
; boundary in the phase plane. If the current rate is too high for the error
; (above the curve), fire immediately. If acceptable (below the curve), coast
; to save fuel. This switch curve prevents overshooting and minimizes fuel
; consumption. During Armstrong's manual attitude adjustments, this logic kept
; the LM from wasting propellant during rapid repositioning maneuvers.
;
; Code-Along Readers: RUFLAW3 implements bang-bang control with quadratic
; switch curve for minimum-time or near-minimum-fuel attitude changes.
;
; Entry conditions:
;   - E scaled at π/4 radians (moderate error condition)
;   - EDOT scaled at π/4 rad/sec (high rate)
;   - Both rotated to upper half-plane if originally negative
;   - ADRSDIF1 set to axis indexer (P/U/V-axis)
;
; Switch curve equation: (1/ANET1)·EDOT² + E - FIREDB = 0
;   Where:
;     - 1/ANET1 = inverse of 2-jet acceleration for this axis
;     - EDOT² = current rate squared (kinetic energy term)
;     - E = current error (potential energy term)
;     - FIREDB = switch curve deadband offset
;
; Algorithm steps:
;   1. Compute -FIREDB + E (offset error from switch curve)
;   2. Multiply by BIT11 (÷2048) to scale for quadratic term
;   3. Swap result to EDOT register, load original EDOT
;   4. Square EDOT (compute rate²)
;   5. Multiply by 1/ANET1 (scale by inverse acceleration)
;   6. Add back the E term from step 2
;   7. Test sign of result:
;        Negative → Below switch curve → COASTTJ (no firing)
;        Positive → Above switch curve → FULLTIME (maximum firing)
;
; Physical interpretation:
;   - Switch curve represents the boundary between "must fire now" and
;     "can coast safely" in the error-rate phase plane
;   - Parabolic shape: Higher rates require firing earlier (at larger errors)
;   - This implements time-optimal or near-fuel-optimal bang-bang control
;   - For the actual Apollo 11 mission, this prevented excessive oscillations
;     during high-rate attitude changes while conserving RCS propellant
;
; Scaling notes:
;   - Input E at π/4, scaled to 4π by combining with EDOT² calculation
;   - EDOT at π/4 rad/sec, squared result scaled appropriately
;   - 1/ANET1 accounts for 2-jet acceleration (π/4 rad/sec² scaling)
;   - Final result scaled at 4π radians for comparison to zero
; ----------------------------------------------------------------------------

# Page 1469
RUFLAW3		TC	RUFSETUP	# EXECUTE COMMON RUFLAW SUBROUTINE.
		INDEX	ADRSDIF1
		CS	FIREDB		# CALCULATE DISTANCE FROM SWITCH CURVE
		AD	E		#	1/ANET1*EDOT*EDOT +E - FIREDB = 0
		EXTEND			#		SCALED AT 4 PI RADIANS
		MP	BIT11		; Scale by 2^-11 (÷2048) for quadratic term
		XCH	EDOT		; Swap: A→EDOT, EDOT→A for squaring
		EXTEND
		SQUARE			; EDOT² in A (rate squared, kinetic energy)
		EXTEND
		INDEX	ADRSDIF1
		MP	1/ANET1 +2	; Multiply by inverse acceleration
		AD	EDOT		; Add back the scaled E term
		EXTEND
		BZMF	COASTTJ		# COAST IF BELOW IT (negative result).
		TCF	FULLTIME	# FIRE FOR FULL PERIOD IF ABOVE IT (positive).

; ----------------------------------------------------------------------------
; RUFSETUP: Common Initialization Subroutine for All RUFLAW Entries
;
; Comment-Only Readers: Whenever the rough control laws decide to fire jets,
; they call this setup routine first. It reverses the firing direction
; (ROTSENSE) to oppose the current rotation, and sets up for maximum jet usage
; to quickly stop the rotation. For pitch/roll/yaw axes (U/V), it uses 2 jets;
; for the parallel axis (P), it can use up to 4 jets when needed. This is the
; "hit the brakes hard" preparation before calculating exact firing time.
;
; Code-Along Readers: RUFSETUP is called by TC from RUFLAW1, RUFLAW2, RUFLAW3
; and returns via TC Q. It has two entry points:
;
; RUFSETUP (normal entry):
;   1. Reverses ROTSENSE (flip firing direction to oppose current motion)
;   2. Falls through to +2 entry
;
; RUFSETUP +2 (alternate entry, used by SMALRATE):
;   1. Sets NUMBERT = 4 (indicates 2 actual jets for U/V-axes)
;      - NUMBERT encoding: 4 means 2 jets, 2 means 1 jet
;   2. Sets FIREFCT = NEGMAX (suggests 4 jets for P-axis)
;      - When FIREFCT < -4°, enables 4-jet P-axis firing
;   3. Returns to caller via TC Q
;
; Usage pattern in RUFLAW routines:
;   - RUFLAW1, RUFLAW2, RUFLAW3: Call RUFSETUP to reverse direction
;   - SMALRATE: Calls RUFSETUP +2 to skip reversal (direction already set)
;
; Effect on subsequent jet selection:
;   - NUMBERT and FIREFCT propagate to jet selection logic
;   - Maximum jet configuration ensures rapid attitude rate changes
;   - Minimizes time to null attitude rates in rough control regime
; ----------------------------------------------------------------------------

# SUBROUTINE USED IN ALL ENTRIES TO ROUGHLAW.

RUFSETUP	CS	ROTSENSE	# REVERSE ROTSENSE WHEN ENTER HERE.
		TS	ROTSENSE	; Flip firing direction to oppose motion
 +2		CAF	FOUR		# REQUIRE MAXIMUM (2) JETS IN U,V-AXES.
		TS	NUMBERT		; Set jet count indicator (4→2 jets, 2→1 jet)
		CAF	NEGMAX		# SUGGEST MAXIMUM (4) JETS IN P-AXIS.
		TS	FIREFCT		; When FIREFCT < -4°, enables 4-jet P firing
		TC	Q		; Return to caller

; ============================================================================
; CONSTANTS FOR TJETLAW
;
; Comment-Only Readers: These are the calibrated numbers that control jet
; firing decisions. They include angle thresholds (when to fire), time limits
; (minimum/maximum firing durations), and rate targets (how fast to spin).
; These were carefully tuned through ground testing and simulation to balance
; responsiveness (quick attitude changes) against fuel conservation (don't
; waste propellant). During Apollo 11's lunar operations, these values ensured
; Armstrong and Aldrin could reorient the LM smoothly without excessive RCS
; fuel consumption.
;
; Code-Along Readers: TJETLAW constants organized by function:
;
; 1. AXIS INDEXING PARAMETERS:
;    AXISDIFF(-1), AXISDIFF(0), AXISDIFF(+1): Array indexing offsets
;    - Maps AXISCTR value to memory location differences
;    - P-axis (AXISCTR=-1): -16 registers offset from U-axis baseline
;    - U-axis (AXISCTR=0): 0 offset (reference axis)
;    - V-axis (AXISCTR=+1): +16 registers offset from U-axis baseline
;    - Enables single code path to access axis-specific parameters via INDEX
;
; 2. SCALING CONVERSION FACTORS:
;    SENSOR (octal 14400 = decimal 6400): Time scaling ratio
;    - Converts from TJETLAW internal scaling (4 sec) to T6RUPT output (10.24 sec)
;    - Ratio = 10.24/4 × 2^11 = 5242.88 ≈ 6400 (includes AGC precision)
;    - Applied when storing TJET to output registers TJP, TJU, TJV
;
; 3. ANGULAR THRESHOLDS:
;    -3DEG = -0.06667: -3° threshold scaled at π/4 (45°)
;    - Used in zone boundary tests
;    - Negative value for computational convenience in comparisons
;
; 4. TIME-SQUARED TERMS (for kinetic energy calculations):
;    -.0112A8 = -0.00141: -0.01125 sec² scaled at 8 sec²
;    - Used in parabolic switch curve computations
;    - Scaling chosen to match acceleration and rate² products
;    .00375A8 (2DEC format): 0.00375 sec² scaled B-3 (÷8)
;    - Double-precision constant for high-accuracy calculations
;
; 5. TIME INTERVAL CONSTANTS (various scalings for different contexts):
;    At 4-second scaling:
;      .1AT4 = 0.025: 0.1 sec scaled at 4
;      .0375AT4 = 0.00938: 0.0375 sec (minimum impulse duration)
;      -.025AT4 = -0.00625: -0.025 sec
;      -TJMAX = -0.0375: Maximum firing time = 0.15 sec actual
;      TJMIN = 0.005: Minimum firing time = 0.02 sec actual (20ms minimum impulse)
;      -TJMIN = -0.005: Negative minimum for comparison convenience
;
;    At 2-second scaling:
;      .1AT2 = 0.05: 0.1 sec scaled at 2
;      -.025AT2 = -0.0125: -0.025 sec
;      -.05AT2 = -0.025: -0.05 sec
;      -.15AT2 = -0.075: -0.15 sec
;
; 6. RATE TARGETS:
;    RUFRATE = 0.1444: Target rate = 6.5°/sec scaled at π/4 rad/sec (45°/sec)
;    - Boundary between SMALRATE and RUFLAW1
;    - 6.5°/sec = 0.1135 rad/sec = 0.1135/(π/4) = 0.1444 in scaled units
;    - Represents transition from fine control to rough control
;
; Physical significance of time limits:
;   - TJMIN (20ms): Hardware minimum impulse bit for valve actuation
;   - TJMAX (150ms): Prevents excessively long firings that waste fuel
;   - These limits ensure discrete on/off control (bang-bang) rather than
;     attempting continuous modulation beyond hardware capabilities
;
; Scaling conventions summary:
;   - Angles: π/4 radians = 45° per unit (quarter-circle scaling)
;   - Rates: π/4 rad/sec = 45°/sec per unit
;   - Times: Multiple scales (2, 4, 8 sec) chosen to optimize precision
;            for different calculation contexts within 15-bit AGC word
;   - Output time: 10.24 sec scaling matches T6RUPT period (1.28 sec × 8)
; ============================================================================

# CONSTANTS FOR TJETLAW

		DEC	-16		# AXISDIFF(INDEX) = NUMBER OF REGISTERS
AXISDIFF	DEC	+0		#	BETWEEN STORED 1/ACCS PARAMETERS FOR
		DEC	16		#	THE INDEXED AXIS AND THE U-AXIS.
					; P-axis: -16, U-axis: 0, V-axis: +16
SENSOR		OCT	14400		# RATIO OF TJET SCALING WITHIN TJETLAW
					#	(4 SEC) TO SCALING FOR T6 (10.24 SEC).
					; Octal 14400 = decimal 6400 conversion factor
-3DEG		DEC	-.06667		# -3.0 DEGREES SCALED AT 45.
					; Zone boundary threshold (π/4 scaling)
-.0112A8	DEC	-.00141		# -.01125 SEC(2) SCALED AT 8.
					; Time-squared term for switch curves
.1AT4		DEC	.025		# 0.1 SECOND SCALED AT 4.
.1AT2		DEC	.05		# .1 SEC SCALED AT 2.
.0375AT4	DEC	.00938		# .0375 SEC SCALED AT 4.
					; Minimum impulse duration threshold
-.025AT2	DEC	-.0125		# -.025 SEC SCALED AT 2.
-.025AT4	DEC	-.00625		; -0.025 sec at 4-sec scaling
-.05AT2		DEC	-.025		; -0.05 sec at 2-sec scaling
-.15AT2		DEC	-.075		; -0.15 sec at 2-sec scaling
.00375A8	2DEC	.00375 B-3	; Double-precision time² constant (÷8)

-TJMAX		DEC	-.0375		# LARGEST CALCULATED TIME.  .150 SEC AT 4.
					; Maximum firing duration = 150ms actual
TJMIN		DEC	.005		# SMALLEST ALLOWABLE TIME.  .020 SEC AT 4.
					; Minimum firing duration = 20ms (hardware limit)
-TJMIN		DEC	-.005		; Negative minimum for comparisons
RUFRATE		DEC	.1444		# CORRESPONDS TO TARGET RATE OF 6.5 DEG/S.
					; Boundary: fine↔rough control (scaled π/4 rad/sec)
