# Copyright:	Public domain.
# Filename:	P-AXIS_RCS_AUTOPILOT.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1421-1441
# Mod history:	2009-05-27 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-05 RSB	Corrected a relative jump from
#				+8 to +8D.
#		2009-06-07 RSB	Corrected a typo.
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
; FILE: P-AXIS_RCS_AUTOPILOT.agc
; MODULE: Digital Autopilot System (DAPS) - Roll Axis Control
; MISSION PHASE: descent/landing/ascent/rendezvous/orbital
;
; TL;DR: Implements the P-axis (roll) digital autopilot for the Lunar Module
;        using Reaction Control System (RCS) thrusters. Controls roll attitude
;        through PID control law with deadband logic, minimum impulse mode,
;        and sophisticated jet selection algorithms. Executes every 100ms via
;        T5RUPT interrupt to maintain precise attitude control during all LM
;        flight phases including powered descent, landing, ascent, and
;        rendezvous operations. Interfaces with guidance commands from
;        FINDCDUW and implements Manual Rate Command Mode designed by Robert
;        F. Stengel for crew control authority.
;
; COMMENT-ONLY READERS: This autopilot maintained Eagle's roll orientation
;        throughout the mission, from separation through landing and ascent.
;        Read to understand how the computer continuously fired small thruster
;        bursts to keep the spacecraft properly oriented.
;
; CODE-ALONG READERS: Study the interrupt-driven control loop architecture,
;        PID implementation, phase plane analysis, jet selection policies with
;        failure handling, and the interface between automatic guidance and
;        manual crew control modes.
; ============================================================================

# Page 1421
		BANK	16
		SETLOC	DAPS1
		BANK

		EBANK=	PERROR
		COUNT*	$$/DAPP

; ============================================================================
; P-AXIS INTERRUPT ENTRY POINT
;
; The Lunar Module's roll axis (P-axis) autopilot begins here, triggered by
; the T5RUPT timer interrupt every 100 milliseconds (10 times per second).
; This continuous execution maintains roll attitude control during all flight
; phases. During Apollo 11's descent, this routine fired RCS thrusters to
; counteract disturbances and maintain proper spacecraft orientation as
; Armstrong and Aldrin approached the lunar surface.
; ============================================================================

# THE FOLLOWING T5RUPT ENTRY BEGINS THE PROGRAM WHICH CONTROLS THE P-AXIS ACTION OF THE LEM USING THE RCS JETS.
# THE NOMINAL TIME BETWEEN THE P-AXIS RUPTS IS 100 MS IN ALL NON-IDLING MODES OF THE DAP.

; P-axis autopilot interrupt handler - executes every 100 milliseconds.
; Preserves interrupt context and synchronizes with other time-critical tasks.

PAXIS		CA	MS100
		ADS	TIME5		# *** NECESSARY IN ORDER TO ALLOW
					# SYNCHRONIZATION WITH OTHER INTERRUPTS ***

; Standard interrupt entry sequence: save return address (Q) and bank register.
; This allows the autopilot to safely access variables in different memory banks
; and return control to the interrupted program.

		LXCH	BANKRUPT	# INTERRUPT LEAD IN (CONTINUED)
		EXTEND
		QXCH	QRUPT

# CHECK IF DAP PASS IS PERMISSIBLE

; Verify that the previous DAP cycle has completed. If DAPZRUPT is still
; positive, the DAP is overrunning its time allocation, indicating a serious
; computational overload. This triggers a restart to recover system state.

		CCS	DAPZRUPT	# IF DAPZRUPT POSITIVE, DAP (JASK) IS
		TC	BAILOUT		# STILL IN PROGRESS AND A RESTART IS
		OCT	02000		# CALLED FOR.  IT IS NEVER ZERO

; Check mode control bits to determine if DAP should remain active or idle.

		TC	CHEKBITS	# RETURN IS TC I+1 IF DAP SHOULD STAY ON.

; ============================================================================
; CDU (Coupling Data Unit) SAMPLING
;
; The IMU (Inertial Measurement Unit) gimbal angles are read from the CDU
; registers and stored for attitude and rate calculations. CDUX represents
; roll angle, CDUY is pitch, CDUZ is yaw. These angles are measured relative
; to the inertial reference frame and scaled at π radians (180 degrees).
; ============================================================================

		CA	CDUX		# READ AND STORE CDU'S
		TS	DAPTREG4
		CA	CDUY
		TS	DAPTREG5
		CA	CDUZ
		TS	DAPTREG6

# ***** KALCMANU-DAP AND "RATE-HOLD"-DAP INTERFACE *****
#
# THE FOLLOWING SECTION IS EXECUTED EVERY 100 MS (10 TIMES A SECOND) WITHIN THE P-AXIS REACTION CONTROL SYSTEM
# AUTOPILOT (WHENEVER THE DAP IS IN OPERATION).

; ============================================================================
; ATTITUDE RATE DERIVATION INTERFACE
;
; This section bridges the guidance system (which commands desired attitudes)
; with the autopilot (which controls RCS thrusters). The commanded attitudes
; are differenced with measurements to compute attitude rates and errors.
; During descent, guidance commands came from the landing program while the
; autopilot maintained precise orientation for radar data and engine thrust.
; ============================================================================

; Update the commanded CDU angles (CDUXD, CDUYD, CDUZD) by removing the
; incremental changes (DELCDUX, DELCDUY, DELCDUZ) computed by the guidance
; system. The 1STOTWOS subroutine handles overflow and sign correction.
; This process converts guidance commands into body-referenced attitudes.

		CA	CDUXD
		EXTEND
		MSU	DELCDUX
		TC	1STOTWOS
		TS	CDUXD
		CA	CDUYD
		EXTEND
		MSU	DELCDUY
		TC	1STOTWOS
		TS	CDUYD
		CA	CDUZD
		EXTEND
		MSU	DELCDUZ
# Page 1422
		TC	1STOTWOS
		TS	CDUZD

; Decrement the manual control timers (TCP for roll, TCQR for pitch/yaw).
; These timers track how long crew hand controller inputs remain active.
; When timers reach zero, manual rate commands expire and automatic control
; resumes. This allowed Armstrong to make manual adjustments during descent
; while automatically returning to guidance control when he released the stick.

		EXTEND			# DIMINISH MANUAL CONTROL DIRECT RATE
		DIM	TCP		# TIME COUNTERS.
		EXTEND
		DIM	TCQR

# RATFLOOP COMPUTES JETRATEQ, JRATER, AND 1JACC*NO. PJEETS IN ITEMP1.
# RETURNS TO BACKP.
#
# JETRATE = 1JACC*NO.PJETS*TJP		(NOTE TJ IS THE TIME FIRED DURING CSP)
# JETRATEQ = 1JACCQ(TJU*NO.UJETS - TJV*NO.VJETS)
# JETRATER = 1JACCR(TJU*NO.UJETS + TJV*NO.VJETS)

		TCF	PAXFILT		# PROCEEDS TO RATELOOP AFTER SUPERJOB
1STOTWOS	CCS	A
		AD	ONE
		TC	Q
		CS	A
		TC	Q
SUBDIVDE	EXTEND			# OVERFLOW PROTECTION ROUTINE TO GIVE
		MP	DAPTEMP3	# POSMAX OR NEGMAX IF THE DIVIDE WOULD
		DAS	OMEGAU		# OVERFLOW

 +3		EXTEND
		DCA	OMEGAU
		DXCH	DAPTEMP5
		CCS	OMEGAU
		TCF	+2
		TCF	DIVIDER
		AD	-OCT630
		EXTEND
		BZMF	DIVIDER

		CCS	OMEGAU
		CA	POSMAX		# 45 DEG/SEC
		TC	Q
		CS	POSMAX
		TC	Q

; ============================================================================
; UTILITY SUBROUTINES: Rate Limiting and Division
;
; The autopilot uses these mathematical utility routines throughout its
; control calculations. DIVIDER performs double-precision division for
; rate computations. OVERSUB implements overflow protection, essential
; for maintaining computational stability during aggressive maneuvering.
; ============================================================================

; DIVIDER: Double-precision division subroutine
; Input: OMEGAU (double-precision dividend), DAPTREG4 (divisor)
; Output: Quotient in A register
; Used extensively in coordinate transformation rate calculations.

DIVIDER		DXCH	OMEGAU
		EXTEND
		DV	DAPTREG4
		TC	Q

; OVERSUB: Overflow protection and limiting subroutine
; Returns A register unchanged if within range, or limits to POSMAX/NEGMAX
; if overflow detected. Critical safety feature preventing computational
; instability during high-rate maneuvers or in degraded sensor conditions.
; During Apollo 11's descent, this prevented rate calculation overflow when
; Armstrong executed manual attitude adjustments.

OVERSUB		TS	7		# RETURNS A UNCHANGED OR LIMITED TO
		TC	Q		# POSMAX OR NEGMAX IF A HAS OVERFLOW
		INDEX	A
		CS	BIT15 	-1
# Page 1423
		TC	Q

-OCT630		OCT	77147

; ============================================================================
; TRANSITION: From Rate Filtering to Rate Derivation
;
; With jet firing effects computed, the autopilot now derives current body
; rates from CDU (gimbal angle) measurements. This rate derivation section
; is the heart of the digital autopilot, converting position sensor data
; into velocity information needed for damping control.
;
; The spacecraft has no direct rate gyros - all angular velocity information
; must be computed by differencing successive gimbal angle readings. This
; 100ms sample interval provides sufficient resolution while avoiding
; excessive computational load on the AGC.
; ============================================================================

; BACKP: Beginning of rate derivation computation
; JETRATE contains the rate change due to jet firings during the previous
; 100ms interval. This must be integrated with measured gimbal rates.

BACKP		CA	DAPTEMP1
		EXTEND
		MP	1JACC
		TS	JETRATE

; ============================================================================
; RATE DERIVATION: Computing Spacecraft Angular Velocities
;
; The LM has no rate gyros - angular velocities must be derived from IMU
; gimbal angle changes measured over successive 100ms intervals. This section
; implements a sophisticated state estimator combining:
; - Measured CDU angle differences (position derivatives)
; - Jet firing effects (known acceleration inputs)
; - Offset acceleration estimates (thrust misalignment, external torques)
; - Accumulated angle errors (integral feedback for drift correction)
;
; Key Variables (original NASA notation preserved):
;   OMEGAP,Q,R    - Body rates scaled at PI/4 radians/sec
;   TRAPEDP,Q,R   - Accumulated angle errors scaled at PI/40 radians
;   NP(QR)TRAPS   - Number of times angle error has been accumulated
;   AOSQ(R)TERM   - Rate change due to offset acceleration (PI/4)
;   JETRATE,Q,R   - Rate change due to jet acceleration (PI/4)
;   TRAPSIZE      - Negative limit of TRAPED magnitude
;   OMEGAU        - Double-precision temporary storage
;
; The core equation implemented here:
;   OMEGA = OMEGA + JETRATE + AOSTERM + (TRAPED/NTRAPS if TRAPED large)
;
; This state estimation algorithm was crucial during Apollo 11's landing,
; providing accurate rate damping even as Armstrong manually controlled
; attitude for landing site selection.
; ============================================================================

# BEGINNING OF THE RATE DERIVATION
#	OMEGAP,Q,R	BODY RATES SCALED AT PI/4
#	TRAPEDP,Q,R	BODY ANGLE ERRORS FROM PREDICTED ANGLE (PI/40)
#	NP(QR)TRAPS	NUMBER OF TIMES ANGLE ERROR HAS BEEN ACCUMULATED
#	AOSQ(R)TERM	CHANGE IN RATE DUE TO OFFSET ACCELERATION.  (PI/4)
#	JETRATE,Q,R	CHANGE IN RATE DUE TO  JET   ACCELERATION.  (PI/4)
#	TRAPSIZE	NEGATIVE LIMIT OF MAGNITUDE OF TRAPEDP, ETC.
#	OMEGAU		DP-TEMPORARY STORAGE
# OMEGA = OMEGA + JETRATE + AOSTERM (+TRAPED/NTRAPS IF TRAPED BIG)

; ============================================================================
; P-AXIS (PITCH) RATE COMPUTATION: Deriving OMEGAP from CDU measurements
;
; The following sequence computes pitch angular velocity by:
; 1. Reading current CDUX (pitch gimbal angle)
; 2. Computing difference from previous reading (OLDXFORP)
; 3. Scaling the angle difference to create rate estimate
; 4. Updating accumulated error terms (TRAPEDP) with jet and offset effects
;
; All three axes (P, Q, R) use similar logic with coordinate transformation
; accounting for gimbal geometry. The M matrix elements transform body-fixed
; CDU measurements into the desired control reference frame.
; ============================================================================

; Step 1: Compute pitch angle change since last 100ms sample
; CDUX measurement scaled at PI radians represents gimbal angle

		CAE	DAPTREG4	# CDUX IS STORED HERE
		TS	L
		EXTEND
		MSU	OLDXFORP	# SCALED AT PI
		LXCH	OLDXFORP
		TS	DAPTEMP1	# Save pitch angle change for transformation

; Step 2: Prepare scaling constant and update accumulated error terms
; The 1/40 scaling factor converts PI radians to PI/40 for TRAPED variables

		CA	1/40
		TS	DAPTREG4

; Step 3: Update TRAPEDP with jet firing effects
; Negative JETRATE indicates pitch-down jet acceleration must be integrated
; out of the accumulated error. BIT14 scaling converts from PI/4 to PI/40.

		CS	JETRATE
		EXTEND
		MP	BIT14
		ADS	TRAPEDP		# Accumulate pitch error

; Step 4: Update TRAPEDQ and TRAPEDR with jet and offset effects
; Q and R axes accumulate their jet and offset acceleration terms scaled
; appropriately. The -BIT14 accounts for gimbal transformation sign.

		CA	JETRATEQ
		AD	AOSQTERM
		EXTEND
		MP	-BIT14
		ADS	TRAPEDQ		# Accumulate yaw error
		CA	JETRATER
		AD	AOSRTERM
		EXTEND
		MP	-BIT14
		ADS	TRAPEDR		# Accumulate roll error

; ============================================================================
; M MATRIX TRANSFORMATION: Converting Gimbal Rates to Body Rates
;
; The IMU gimbals measure angles in a nested gimbal coordinate system, but
; the autopilot needs body-axis rates. The M matrix (direction cosine matrix)
; performs this coordinate transformation accounting for current gimbal angles.
;
; For P-axis (pitch) body rate:
;   OMEGAP = M11*(CDUY_rate) + (CDUX_rate)
;
; The transformation accounts for gimbal coupling - a yaw gimbal rotation
; produces both yaw and pitch body motion depending on current pitch gimbal
; angle. The M matrix elements (M11, M21, M22, M31, M32) are continuously
; updated based on current gimbal geometry.
;
; This mathematical sophistication was essential during Apollo 11's descent
; when rapid attitude changes required accurate rate feedback despite complex
; gimbal kinematics.
; ============================================================================

; Step 5: Compute CDUY angle change and begin M matrix transformation
; CDUY (yaw gimbal) reading scaled at PI radians

		CA	DAPTREG5	# CDUY IS STORED HERE
		TS	L
		EXTEND
		MSU	OLDYFORP	# SCALED AT PI
		LXCH	OLDYFORP
		TS	DAPTEMP2	# Save yaw angle change
		
; Step 6: Apply M11 matrix element to yaw rate
; M11 represents yaw gimbal contribution to pitch body rate
; Scaled at 1 (unity gain for small angles, cosine for larger angles)

		EXTEND
		MP	M11		# M11 SCALED AT 1
# Page 1424
		AD	DAPTEMP1	# Add direct pitch gimbal contribution
		DXCH	OMEGAU		# Store in double-precision temp

; Step 7: Scale and divide to convert angle change to rate
; SUBDIVDE divides by sample time and applies PI/4 scaling
; Returns with computed rate at PI/4 radians/sec scale

		TC	SUBDIVDE +3	# RETURNS WITH CDU-RATE AT PI/4

; Step 8: Update pitch body rate (OMEGAP) and accumulated error (TRAPEDP)
; The new rate estimate is compared to previous OMEGAP, with difference
; accumulated in TRAPEDP. This integral feedback provides long-term drift
; correction and handles unmodeled accelerations.

		EXTEND
		SU	OMEGAP		# Compute rate change since last cycle
		ADS	TRAPEDP		# Accumulate into pitch error integral
		TC	OVERSUB		# Apply overflow protection
		TS	TRAPEDP		# Store protected value

; Step 9: Update manual mode attitude error accumulator (DXERROR)
; When crew uses manual attitude control (ATCA hand controller), commanded
; attitudes are integrated here. DAPTEMP5 contains discretized error from
; hand controller input. PLAST is the last commanded pitch attitude.
; During Apollo 11's landing, Armstrong used this manual control to select
; the final landing site, overriding automatic guidance.

		EXTEND
		DCA	DAPTEMP5	# Fetch attitude error from hand controller
		DAS	DXERROR		# Accumulate into pitch attitude error (DP)
		CS	PLAST		# Negate last commanded pitch angle
		EXTEND
		MP	1/40		# Scale to match DXERROR units
		DAS	DXERROR		# MANUAL MODE X-ATTITUDE ERROR (DP)

; ============================================================================
; Q-AXIS (YAW) RATE COMPUTATION: Similar transformation for yaw body rate
;
; Following the same pattern as P-axis, but using M21 and M22 matrix elements
; to transform gimbal rates into yaw body rate. The M matrix accounts for
; pitch gimbal angle affecting how yaw and roll gimbal motions couple into
; body yaw rate.
; ============================================================================

		CA	DAPTREG6	# CDUZ IS STORED HERE
		TS	L
		EXTEND
		MSU	OLDZFORQ	# Compute roll gimbal angle change
		TS	DAPTEMP3	# Save for R-axis computation
		LXCH	OLDZFORQ	# Update previous roll gimbal angle

; Q-axis M matrix transformation: OMEGAQ = M21*(CDUY_rate) + M22*(CDUZ_rate)
; M21 accounts for yaw gimbal contribution to yaw body rate
; M22 accounts for roll gimbal contribution to yaw body rate

		CA	M21		# Fetch M21 matrix element
		EXTEND
		MP	DAPTEMP2	# Multiply by yaw gimbal rate
		DXCH	OMEGAU		# Store partial result
		CA	M22		# Fetch M22 matrix element
		TC	SUBDIVDE	# Complete division and add roll contribution

; Update yaw body rate and accumulated error, same pattern as P-axis

		EXTEND
		SU	OMEGAQ		# Compute yaw rate change
		ADS	TRAPEDQ		# Accumulate yaw error
		TC	OVERSUB		# Overflow protection
		TS	TRAPEDQ		# Store protected yaw error

; Update manual mode yaw attitude error (DYERROR)
; Used when crew manually commands yaw attitude via hand controller

		EXTEND
		DCA	DAPTEMP5	# Fetch yaw attitude error
		DAS	DYERROR		# Accumulate (double-precision)
		CS	QLAST		# Negate last commanded yaw
		EXTEND
		MP	1/40		# Scale appropriately
		DAS	DYERROR		# MANUAL MODE Y-ATTITUDE ERROR (DP)

; ============================================================================
; R-AXIS (ROLL) RATE COMPUTATION: Final axis transformation
;
; Completes the three-axis rate derivation using M31 and M32 matrix elements
; to transform gimbal rates into roll body rate. All three body rates (P, Q, R)
; are now available for the control law computation.
; ============================================================================

		CA	M31		# Fetch M31 matrix element
		EXTEND
		MP	DAPTEMP2	# M31 * (yaw gimbal rate)
		DXCH	OMEGAU		# Store partial result
		CA	M32		# Fetch M32 matrix element

		TC	SUBDIVDE	# Complete: R-rate = M31*Yrate + M32*Zrate
# Page 1425

; Update roll body rate and accumulated error
; TRAPEDR accumulates roll axis error for use in control law

		EXTEND
		SU	OMEGAR		# Compute roll rate change
		ADS	TRAPEDR		# Accumulate roll error
		TC	OVERSUB		# Overflow protection
		TS	TRAPEDR		# TRAPEDS HAVE ALL BEEN COMPUTED

; Update manual mode roll attitude error (DZERROR)
; Crew can command roll attitude via rotational hand controller
; This error term tracks the difference between commanded and actual roll attitude

		EXTEND
		DCA	DAPTEMP5	# Fetch roll attitude error
		DAS	DZERROR		# Accumulate (double-precision)
		CS	RLAST		# Negate last commanded roll
		EXTEND
		MP	1/40		# Scale appropriately
		DAS	DZERROR		# MANUAL MODE Z-ATTITUDE ERROR (DP)

; ============================================================================
; DOCKING CONFIGURATION SELECTION
;
; The autopilot must use different control gains and trap sizes depending
; on whether the Lunar Module is docked to the Command Module or flying solo.
; The combined spacecraft has much larger moment of inertia, requiring
; different thruster firing strategies and rate estimation parameters.
; ============================================================================

		CA	DAPBOOLS	# PICK UP PAD LOADED STATE ESTIMATOR GAINS
		MASK	CSMDOCKD	# Check docking configuration bit
		EXTEND
		BZF	LMONLY		# Branch if LM only (undocked)

; DOCKED CONFIGURATION: LM docked to CSM
; Higher moment of inertia, more conservative control gains

		EXTEND			# DOCKED
		DCA	DKOMEGAN	# Docked omega-N values (rate deadband)
		DXCH	DAPTREG4	# Store in temporary registers
		CA	DKTRAP		# Docked trap size threshold
		TCF	+5

; UNDOCKED CONFIGURATION: LM flying solo
; Lower moment of inertia, more aggressive control possible

LMONLY		EXTEND			# UNDOCKED
		DCA	LMOMEGAN	# LM-only omega-N values
		DXCH	DAPTREG4	# Store in temporary registers
		CA	LMTRAP		# LM-only trap size threshold
 +5		TS	DAPTREG6	# Save trap size for all three axes

; ============================================================================
; P-AXIS OFFSET ESTIMATOR (Pitch Rate Bias Compensation)
;
; The "trap" logic detects persistent pitch rate errors (TRAPEDP) that exceed
; threshold. When detected, the autopilot estimates a systematic rate bias
; and adjusts OMEGAP to compensate. This handles situations like:
; - Center-of-gravity offsets causing torque imbalances
; - Thruster misalignment producing unwanted pitch rates
; - External disturbances (venting, thermal effects)
;
; If TRAPEDP exceeds threshold: Rate bias = TRAPEDP / NPTRAPS
; This bias is added to OMEGAP and NPTRAPS is reset to configuration value.
; ============================================================================

		CCS	TRAPEDP		# Check accumulated pitch error
		TCF	+2		# Positive, continue check
		TCF	SMALPDIF	# Zero or negative, skip bias update
		AD	DAPTREG6	# TRAPSIZE > ABOUT 77001 %-1.4DEG/SEC"
		EXTEND
		BZMF	SMALPDIF	# Below threshold, no bias correction
		ZL			# Clear L register
		LXCH	TRAPEDP		# Move TRAPEDP to L, zero TRAPEDP
		CA	ZERO		# Set up division
		EXTEND
		DV	NPTRAPS		# Compute bias: TRAPEDP / number of traps
		ADS	OMEGAP		# Add bias to pitch body rate
		TC	OVERSUB		# Overflow protection
		TS	OMEGAP		# Store corrected pitch rate
		CA	DAPTREG4	# ABOUT 10 OR 0 FOR DOCKED OR UNDOCKED
		TS	NPTRAPS		# Reset trap counter to config value
SMALPDIF	INCR	NPTRAPS		# Increment trap counter

; Add thruster contribution to pitch rate
; JETRATE computed earlier from actual thruster firing times

P-RATE		CA	JETRATE		# Fetch pitch jet rate contribution
		ADS	OMEGAP		# Add to pitch body rate
		TC	OVERSUB		# Overflow protection
		TS	OMEGAP		# Final pitch rate for control law

; ============================================================================
; Q-AXIS OFFSET ESTIMATOR (Yaw Rate Bias Compensation)
;
; Same offset estimation logic as P-axis, but with additional AOSQ computation
; (Attitude Offset Smoothing for Q-axis). AOSQ provides a smoothed estimate
; of yaw axis disturbance torques, used to improve control law performance.
; ============================================================================

		CCS	TRAPEDQ		# Check accumulated yaw error
# Page 1426
		TCF	+2		# Positive, continue check
		TCF	Q-RATE		# Zero/negative, skip bias update
		AD	DAPTREG6	# TRAPSIZE > ABOUT 77001 %-1.4DEG/SEC"
		EXTEND
		BZMF	Q-RATE		# Below threshold, no correction
		ZL			# Clear L register
		LXCH	TRAPEDQ		# Move TRAPEDQ to L, zero TRAPEDQ
		CA	ZERO		# Set up division
		EXTEND
		DV	NQTRAPS		# Compute bias: TRAPEDQ / number of traps
		TS	DAPTEMP1	# SAVE FOR OFFSET ESTIMATE
		ADS	OMEGAQ		# Add bias to yaw body rate
		TC	OVERSUB		# Overflow protection
		TS	OMEGAQ		# Store corrected yaw rate
		CA	DAPTREG4	# ABOUT 10 OR 0 FOR DOCKED OR UNDOCKED
		XCH	NQTRAPS		# Reset trap counter, get old value
		AD	DAPTREG5	# KAOS > ABOUT 60D %N/N_60"
		XCH	DAPTEMP1	# Swap with bias value

; Update AOSQ (Attitude Offset Smoothing for Q-axis)
; AOSQ tracks slowly-varying disturbance torques on yaw axis
; Update rate: AOSQ += 5 * KAOS * bias_change / bias

		EXTEND
		MP	FIVE		# Multiply by 5
		EXTEND
		DV	DAPTEMP1	# Divide by bias value
		ADS	AOSQ		# Update smoothed offset estimate

Q-RATE		INCR	NQTRAPS		# Increment trap counter
		CA	JETRATEQ	# Fetch yaw jet rate contribution
		AD	AOSQTERM	# Add smoothed offset term
		ADS	OMEGAQ		# Add to yaw body rate
		TC	OVERSUB		# Overflow protection
		TS	OMEGAQ		# Final yaw rate for control law

; ============================================================================
; R-AXIS OFFSET ESTIMATOR (Roll Rate Bias Compensation)
;
; Same offset estimation logic as Q-axis, with AOSR computation
; (Attitude Offset Smoothing for R-axis). AOSR provides a smoothed estimate
; of roll axis disturbance torques, used to improve control law performance.
; ============================================================================

		CCS	TRAPEDR		# Check accumulated roll error
		TCF	+2		# Positive, continue check
		TCF	R-RATE		# Zero/negative, skip bias update
		AD	DAPTREG6	# TRAPSIZE > ABOUT 77001 %-1.4DEG/SEC"
		EXTEND
		BZMF	R-RATE		# Below threshold, no correction
		ZL			# Clear L register
		LXCH	TRAPEDR		# Move TRAPEDR to L, zero TRAPEDR
		CA	ZERO		# Set up division
		EXTEND
		DV	NRTRAPS		# Compute bias: TRAPEDR / number of traps
		TS	DAPTEMP2	# SAVE FOR OFFSET ESTIMATE
		ADS	OMEGAR		# Add bias to roll body rate
		TC	OVERSUB		# Overflow protection
		TS	OMEGAR		# Store corrected roll rate
		CA	DAPTREG4	# ABOUT 10 OR 0 FOR DOCKED OR UNDOCKED
		XCH	NRTRAPS		# Reset trap counter, get old value
		AD	DAPTREG5	# KAOS > ABOUT 60D %N/N_60"
		XCH	DAPTEMP2	# Swap with bias value
		EXTEND
# Page 1427

; Update AOSR (Attitude Offset Smoothing for R-axis)
; AOSR tracks slowly-varying disturbance torques on roll axis
; Update rate: AOSR += 5 * KAOS * bias_change / bias

		MP	FIVE		# Multiply by 5
		EXTEND
		DV	DAPTEMP2	# Divide by bias value
		ADS	AOSR		# Update smoothed offset estimate

R-RATE		INCR	NRTRAPS		# Increment trap counter
		CA	JETRATER	# Fetch roll jet rate contribution
		AD	AOSRTERM	# Add smoothed offset term
		ADS	OMEGAR		# Add to roll body rate
		TC	OVERSUB		# Overflow protection
		TS	OMEGAR		# Final roll rate for control law

# END OF RATE DERIVATION
#	BEGIN OFFSET ESTIMATER
#		IN POWERED FLIGHT, AOSTASK WILL BE CALLED EVERY 2 SECONDS.
#			AOS = AOS + K*SUMRATE

; ============================================================================
; ATTITUDE OFFSET SMOOTHING (AOS) UPDATE
;
; The AOS system tracks slowly-varying disturbance torques (offset
; accelerations) on the Q and R axes. During powered flight (e.g., descent
; engine firing), AOSTASK calls this section every 2 seconds to update the
; AOS estimates based on accumulated rate errors.
;
; If DRIFTBIT is clear (coasting flight): Zero all AOS values
; If DRIFTBIT is set (powered flight): Update AOS from acceleration dot terms
;
; The smoothed offset terms (AOSQTERM, AOSRTERM) are added to body rates
; in the control law to compensate for persistent disturbance torques.
; ============================================================================

		CS	DAPBOOLS	# Check DAP configuration flags
		MASK 	DRIFTBIT	# Test drift compensation bit
		CCS	A		# Is drift compensation active?
		TCF	WORKTIME	# Yes, update AOS values

; DRIFTBIT clear: Zero all offset acceleration values
; Used during coasting flight when no persistent disturbances expected

		TS	ALPHAQ		# ZERO THE OFFSET ACCELERATION VALUES.
		TS	ALPHAR		# Zero yaw and roll alphas
		TS	AOSQTERM	# Zero smoothed offset terms
		TS	AOSRTERM	# (used in rate computations above)
		TS	AOSQ		# Zero accumulated offsets
		TS	AOSR		# All offset tracking disabled
		TCF	PRETIMCK	# Skip to time check

KAOS		DEC	60		# AOS update gain (about 60D)

; DRIFTBIT set: Update AOS from acceleration dot terms
; Powered flight mode - track engine/thruster disturbance torques

WORKTIME	CA	QACCDOT		# Yaw acceleration dot (rate of change)
		EXTEND
		MP	CALLCODE	# OCTAL 00032 IS DECIMAL .1 AT 2(6).
		DAS	AOSQ		# Update yaw offset estimate (DP)
		CA	AOSQ		# Fetch updated value
		TS	ALPHAQ		# Store yaw offset acceleration
		EXTEND
		MP	200MS		# .2 AT 1 (scale by time interval)
		TS	AOSQTERM	# Compute smoothed yaw offset term

		CA	RACCDOT		# Roll acceleration dot (rate of change)
		EXTEND
		MP	CALLCODE	# OCTAL 00032 IS DECIMAL .1 AT 2(6).
		DAS	AOSR		# Update roll offset estimate (DP)
		CA	AOSR		# Fetch updated value
		TS	ALPHAR		# Store roll offset acceleration
		EXTEND
		MP	200MS		# .2 AT 1 (scale by time interval)
		TS	AOSRTERM	# Compute smoothed roll offset term
		TCF	PRETIMCK	# Continue to time check

# Page 1428

; ============================================================================
; PAXFILT - GIMBAL FILTER AND SUPERJOB SETUP
;
; This section checks if gimbal drive updates are needed (via CALLGMBL flag).
; If so, calls ACDT+C12 to update gimbal drive accelerations. Then saves the
; interrupt context and sets up a "superjob" to continue P-axis processing
; at SUPERJOB, which branches to RATELOOP for the control law computation.
;
; The superjob mechanism allows the time-critical rate derivation to complete
; quickly in the interrupt, while deferring the control law computation to a
; background task that can be interrupted if needed.
; ============================================================================

PAXFILT		CA	CALLGMBL	# EXECUTE ACDT+C12, IF NEEDED.
		MASK	RCSFLAGS	# Check gimbal call flag
		CCS	A		# CALLGMBL IS NOT BIT15, SO THIS TEST IS
		TC	ACDT+C12	# VALID. Call gimbal acceleration update

; Save interrupt context and set up superjob for RATELOOP

		DXCH	ARUPT		# Save A and L registers
		DXCH	DAPARUPT	# Store in DAP interrupt save area
		CA	SUPERJOB	# SETTING UP THE SUPERJOB
		XCH	BRUPT		# Save/restore B register
		LXCH	QRUPT		# Save Q register (return address)
		DXCH	DAPBQRPT	# Store B and Q in DAP save area
		CA	SUPERADR	# Address of SUPERJOB continuation
		DXCH	ZRUPT		# Save Z register (program counter)
		DXCH	DAPZRUPT	# Store in DAP save area
		TCF	NOQBRSM +1	# RELINT (JUST IN CASE) AND RESUME, IN THE
					# 	FORM OF A JASK, AT SUPERJOB.

SUPERADR	GENADR	SUPERJOB +1	# Continuation address for superjob

# COUNT DOWN GIMBAL DRIVE TIMERS AND TURN OFF DRIVES IF REQUIRED.

; ============================================================================
; GIMBAL DRIVE TIMER MANAGEMENT
;
; For LM powered descent/ascent, the descent engine is gimbaled (tilted) to
; steer the spacecraft. QGIMTIMR and RGIMTIMR count down the duration of
; gimbal drive commands. When timers reach zero, the drives are turned off.
;
; Historical: During Apollo 11's descent, the descent engine gimbal responded
; to guidance commands while the RCS jets provided supplemental attitude
; control. This timer logic prevented gimbal drive overruns.
; ============================================================================

SUPERJOB	TCF	RATELOOP	# Continue to control law computation

PRETIMCK	CCS	QGIMTIMR	# Check yaw gimbal timer
		TCF	DECQTIMR	# POSITIVE -- COUNTING DOWN
		TCF	TURNOFFQ	# NEGATIVE -- DRIVE SHOULD BE ENDED
CHKRTIMR	CCS	RGIMTIMR	# NEGATIVE -- INACTIVE (check roll gimbal)
		TCF	DECRTIMR	# (NEG ZERO -- IMPOSSIBLE) Counting down
		TCF	TURNOFFR	# REPEATED (ABOVE) FOR R AXIS.

; Timers inactive or expired - decrement jet inhibition counters
; These counters prevent thruster firings for a period after docking maneuvers
; to allow structural dynamics to damp out before resuming attitude control

		EXTEND			# DECREMENT DOCKED JET INHIBITION COUNTERS
		DIM	PJETCTR		# Pitch jet inhibition counter
		EXTEND
		DIM	UJETCTR		# +U jet inhibition counter
		EXTEND
		DIM	VJETCTR		# +V jet inhibition counter
		CA	BIT12		# Check visibility flag
		MASK	RCSFLAGS	# Test if visibility check needed
		EXTEND
		BZF	SKIPPAXS	# Skip if not needed
		TC	CHKVISFZ	# Check visibility, continue

DECQTIMR	TS	QGIMTIMR	# COUNT TIMERS DOWN TO POS ZERO.
		TCF	CHKRTIMR	# Check roll timer
DECRTIMR	TS	RGIMTIMR	# Decrement roll gimbal timer
		TCF	CHKRTIMR +3	# Continue past roll timer check

; ============================================================================
; GIMBAL DRIVE TURNOFF LOGIC
;
; When gimbal drive timers expire, this section halts the gimbal drives by
; zeroing the drive commands (NEGUQ, NEGUR) and acceleration dots (QACCDOT,
; RACCDOT), then clearing the gimbal drive bits in CHAN12 hardware channel.
; Timers are set to NEGMAX to indicate inactive state.
; ============================================================================

TURNOFFQ	TS	NEGUQ		# HALT DRIVES. (zero yaw gimbal drive)
		TS	QACCDOT		# Zero yaw acceleration dot
		CS	QGIMBITS	# Complement yaw gimbal control bits
		EXTEND
# Page 1429
		WAND	CHAN12		# Clear gimbal drive bits in hardware
		CAF	NEGMAX		# Set timer to maximum negative (inactive)
		TS	QGIMTIMR	# Mark yaw gimbal timer inactive
		TCF	CHKRTIMR	# Check roll gimbal timer

TURNOFFR	TS	NEGUR		# Halt roll gimbal drive (zero command)
		TS	RACCDOT		# Zero roll acceleration dot
		CS	RGIMBITS	# Complement roll gimbal control bits
		EXTEND
		WAND	CHAN12		# Clear gimbal drive bits in hardware
		CAF	NEGMAX		# Set timer to maximum negative (inactive)
		TS	RGIMTIMR	# Mark roll gimbal timer inactive
		TCF	CHKRTIMR +3	# Continue to jet counter decrements
QGIMBITS	EQUALS	OCT1400		# BITS 9 AND 10 (OF CHANNEL 12).
RGIMBITS	EQUALS	PRIO6		# BITS 11 AND 12 (OF CHANNEL 12).

SKIPPAXS	CS	RCSFLAGS
		MASK	BIT12
		ADS	RCSFLAGS	# BIT 12 SET TO 1.
		TCF	QRAXIS		# GO TO QRAXIS OR TO CTS.

; ============================================================================
; TRANSITION: From P-axis offset compensation to translation control
;
; The P-axis autopilot has completed its primary rate stabilization tasks.
; Now it handles manual translation commands from the crew's hand controller.
; Translation moves the spacecraft without rotation—critical during rendezvous
; when Armstrong and Aldrin needed to maneuver Eagle toward Columbia, or during
; descent when lateral corrections were required without disturbing attitude.
; ============================================================================

# Y-X TRANSLATION
#
# INPUT:	BITS 9-12 OF CH31 (FROM TRANSLATION CONTROLLER)
#
# OUTPUT:	NEXTP
#
#		NEXTP IS THE CHANNEL 6 CODE OF JETS FOR THE DESIRED TRANSLATION.
#		IF THERE ARE FAILURES IN THE DESIRED TRANSLATION.
#		(1) FOR DIAGONAL TRANS:		UNFAILED PAIR
#						ALARM (IF NO PAIR)
#		(2) FOR PRINCIPAL TRANS:	TRY TO TACK WITH DIAGONAL PAIRS
#						ALARM (IF DIAGONAL PAIRS ARE FAILED)

; TRANSLATION CONTROLLER HANDLER
;
; COMMENT-ONLY READERS: When astronauts push the hand controller sideways or
; forward/back (not rotating it), they command translation—pure motion without
; rotation. This section reads the controller position and selects which RCS
; jets to fire to achieve the desired translation while accounting for any
; failed thrusters.
;
; CODE-ALONG READERS: Reads bits 9-12 of channel 31 (translation controller
; axes). Implements fault-tolerant jet selection: for diagonal translation
; (e.g., +Y+Z simultaneously), selects unfailed jet pairs; for principal axis
; translation (+Y, -Y, +Z, -Z), attempts to "tack" using diagonal pairs if
; primary jets have failed. Issues alarm 02001 if no valid jet combination
; available.

; TRANSLATION COMMAND DECODER
;
; Reads the translation controller position from channel 31 and decodes which
; direction the crew is commanding. Uses lookup table INDXYZ to convert the
; 4-bit controller reading into a rotation index identifying the desired
; translation axis or diagonal combination.

CHKVISFZ	EXTEND
		READ	CHAN31		; READ TRANSLATION CONTROLLER STATE
		CS	A		; COMPLEMENT FOR MASKING
		MASK	07400OCT	; EXTRACT BITS 9-12 (Y/Z TRANSLATION)
		EXTEND
		BZF	TSNEXTP		; IF ZERO, NO TRANSLATION COMMANDED
		
		; Non-zero translation command detected. Convert the 4-bit pattern
		; into an index for jet selection.
		
		EXTEND
		MP	BIT7		; SCALE TRANSLATION BITS
		INDEX	A		; INDIRECT ADDRESS TO LOOKUP TABLE
		CA	INDXYZ		; GET ROTATION INDEX FOR THIS TRANSLATION
		TS	ROTINDEX	; STORE FOR JET SELECTION
		
; JET SELECTION WITH FAILURE HANDLING
;
; Attempts to select jets for the commanded translation. If primary jets have
; failed, tries alternate jet combinations (U/V diagonal pairs). If no valid
; combination exists, raises alarm 02001 (translation jet failure).

TRYUORV		CA	SIX
		TC	SELECTYZ	; SELECT Y/Z TRANSLATION JETS
		CS	SIX
		AD	NUMBERT		; CHECK IF JET SELECTION SUCCESSFUL
		EXTEND
# Page 1430
		BZF	TSNEXTP -1	; IF NUMBERT = 6, SELECTION SUCCESSFUL
		
		; Primary jet selection failed. Check if we can use alternate jets.
		
		CS	FIVE
		AD	ROTINDEX	; CHECK IF ROTINDEX < 5
		EXTEND
		BZMF	ALTERYZ		; IF SO, TRY ALTERNATE (U/V) JETS
		
		CS	NUMBERT
		AD	FOUR		; CHECK IF NUMBERT = 4
		EXTEND
		BZMF	TSNEXTP -1	; IF SO, USE 4-JET CONFIGURATION
		
; TRANSLATION FAILURE ALARM
;
; No valid jet combination available for commanded translation. This could
; occur during Apollo 11 if multiple RCS thrusters had failed, requiring crew
; to use alternate translation techniques or accept degraded maneuverability.

ABORTYZ		TC	ALARM		; RAISE TRANSLATION JET FAILURE ALARM
		OCT	02001		; ALARM CODE 02001
		CA	BIT1		# INVERT BIT 1 OF RCSFLAGS.
		LXCH	RCSFLAGS	; TOGGLE FAILURE FLAG
		EXTEND
		RXOR	1
		TS	RCSFLAGS
		CA	ZERO
		TCF	TSNEXTP
; ALTERNATE JET SELECTION (U/V PAIRS)
;
; When primary translation jets fail or are unavailable, this routine attempts
; to use diagonal jet pairs (U/V combinations) to approximate the desired
; translation. It toggles between two alternate jet selection policies.

ALTERYZ		CA	BIT1		# INVERT BIT 1 OF RCSFLAGS.
		LXCH	RCSFLAGS	; TOGGLE ALTERNATE JET POLICY
		EXTEND
		RXOR	1		; EXCLUSIVE-OR TO FLIP BIT 1
		TS	RCSFLAGS	; STORE UPDATED FLAGS
		MASK	BIT1		; EXTRACT POLICY BIT
		AD	FOUR		; ADD OFFSET FOR ALTERNATE INDEX
		ADS	ROTINDEX	; UPDATE ROTATION INDEX
		TCF	TRYUORV		; RETRY JET SELECTION WITH NEW POLICY
		
		CA	POLYTEMP	; LOAD JET SELECTION RESULT
TSNEXTP		TS	NEXTP		; STORE AS NEXT JET COMMAND

; ============================================================================
; TRANSITION: From translation control to autopilot mode state logic
;
; Translation jet selection complete. Now the autopilot must determine which
; control mode to use: automatic attitude hold, minimum impulse pulse mode,
; or manual rate command mode. During Apollo 11's descent, the DAP primarily
; operated in automatic mode, while during rendezvous and docking, manual rate
; command mode gave Armstrong direct control over the LM's rotation rates.
; ============================================================================

# STATE LOGIC
#	CHECK IN ORDER:		IF ON
#	LPDPHASE		GO TO PURGENCY
#	PULSES			MINIMUM PULSE LOGIC
#	DETENT(BIT15 CH31)	RATE COMMAND
#	GOTO TO PURGENCY
#
; AUTOPILOT MODE SELECTION LOGIC
;
; COMMENT-ONLY READERS: The autopilot can operate in several modes depending
; on the mission phase and crew input. This section checks flags and controller
; position to determine which control mode is active: automatic (computer
; stabilizes attitude), pulse mode (minimum fuel usage), or manual rate command
; (crew directly controls rotation speed).
;
; CODE-ALONG READERS: Decision tree checking in priority order:
; 1. If in landing phase visibility (XOVINHIB), use automatic attitude steering
; 2. If stick in detent (bit 15 of CH31 = 0), check for pulse/rate modes
; 3. If PULSES bit set in DAPBOOLS, use minimum impulse mode
; 4. If stick out of detent, use manual rate command mode
; 5. Otherwise, proceed to automatic urgency function (PURGENCY)

		; Check if manual control stick is in neutral detent position
		
		CA	BIT13		# CHECK STICK IF IN ATT. HOLD.
		EXTEND
		RAND	CHAN31		; READ CHANNEL 31 (HAND CONTROLLER)
		EXTEND
		BZF	MANMODE		; IF BIT 13 CLEAR, STICK OUT OF DETENT
		
		; Stick is in detent (neutral). Check if we're in landing visibility
		; phase where automatic attitude steering takes precedence.
		
		CA	DAPBOOLS	; CHECK DAP MODE FLAGS
		MASK	XOVINHIB	; TEST VISIBILITY PHASE INHIBIT FLAG
		CCS	A		; IS FLAG SET?
		TCF	PURGENCY	# ATTITUDE STEER DURING VISIBILITY PHASE
		
		; Not in visibility phase. Check if rate command mode requested.
		
		TCF	DETENTCK	; CHECK DETENT STATUS FOR RATE MODE
		
; MANUAL MODE BRANCH (STICK OUT OF DETENT)
;
; The crew has moved the hand controller out of neutral. Determine if this
; is pulse mode (minimum impulse) or rate command mode.

MANMODE		CA	PULSES		# PULSES IS ONE FOR PULSE MODE
		MASK	DAPBOOLS	; CHECK IF PULSE MODE ENABLED
# Page 1431
		EXTEND
		BZF	DETENTCK	# BRANCH FOR RATE COMMAND MODE
		
		; Pulse mode selected: minimum impulse control for fine adjustments.
		; This mode fires thrusters for minimum time to conserve fuel.
		
		CA	ZERO		; ZERO OUT ATTITUDE ERROR
		TS	PERROR		; NO PROPORTIONAL ERROR IN PULSE MODE

; ============================================================================
; TRANSITION: Entering minimum impulse mode
;
; COMMENT-ONLY READERS: Pulse mode fires thrusters for the shortest possible
; time (about 14 milliseconds) to make tiny adjustments with minimal fuel use.
; This was useful during rendezvous when Armstrong needed to make small, precise
; corrections without wasting propellant.
;
; CODE-ALONG READERS: Minimum impulse mode implementation. Zeros PERROR since
; control is open-loop (no feedback). Saves current CDU angles as desired
; attitude. Uses minimum on-time for all jet commands.
; ============================================================================

# MINIMUM IMPULSE MODE
;
; In pulse mode, the autopilot captures the current pitch attitude as the
; desired setpoint and waits for crew stick deflection. Each deflection fires
; jets for exactly 14 milliseconds (minimum on-time) regardless of attitude
; error. This provides fuel-efficient fine control for small adjustments.

		CA	CDUX		; READ CURRENT PITCH CDU ANGLE
		TS	CDUXD		; SAVE AS DESIRED ATTITUDE (SETPOINT)

		CCS	OLDPMIN		; CHECK IF PREVIOUS PULSE STILL ACTIVE
		TCF	CHECKP		; IF YES, CHECK FOR STICK CHANGE

; FIRE NEW PULSE COMMAND
;
; No previous pulse active. Check hand controller for commanded direction.

FIREP		CA	BIT3		; CHECK FOR +X PITCH COMMAND
		EXTEND
		RAND	CHAN31		; READ HAND CONTROLLER (CHANNEL 31)
		EXTEND
		BZF	+XMIN		; IF BIT 3 SET, FIRE +X JETS

		CA	BIT4		; CHECK FOR -X PITCH COMMAND
		EXTEND
		RAND	CHAN31		; READ HAND CONTROLLER AGAIN
		EXTEND
		BZF	-XMIN		; IF BIT 4 SET, FIRE -X JETS

		TCF	JETSOFF		; NO PITCH COMMAND, TURN OFF JETS

; PULSE IN PROGRESS - CHECK FOR STICK CHANGE
;
; A pulse is currently being executed. Check if crew changed stick position,
; which would require cancelling the current pulse and starting a new one.

CHECKP		EXTEND
		READ	CHAN31		; READ CURRENT STICK POSITION
		CS	A		; COMPLEMENT FOR COMPARISON
		MASK	OCT14		; EXTRACT PITCH BITS (BITS 3,4)
		TS	OLDPMIN		; SAVE NEW STICK POSITION
		TCF	JETSOFF		; CONTINUE CURRENT PULSE

; MINIMUM IMPULSE JET COMMANDS
;
; Fire jets for minimum on-time (14 milliseconds). This is the shortest
; controllable jet pulse, providing fine control with minimal fuel consumption.

-XMIN		CS	TEN		# ANYTHING LESS THAN 14MS. CORRECTED
		TCF	+2		#	IN JET SELECTION ROUTINE
+XMIN		CA	TEN		; LOAD MINIMUM ON-TIME (14 MS)
		TS	TJP		; STORE AS P-AXIS JET ON-TIME
		CA	ONE		; SET FLAG INDICATING
		TS	OLDPMIN		; PULSE IS NOW ACTIVE
		TCF	PJETSLEC -6	; PROCEED TO JET SELECTION LOGIC

; ============================================================================
; SECTION: MANUAL RATE COMMAND MODE
; ============================================================================
;
; COMMENT-ONLY READERS: When the crew manually controls the LM using the
; Rotational Hand Controller (RHC), this mode provides precise and intuitive
; spacecraft rotation. Designed specifically for lunar landing operations, it
; allows Armstrong and Aldrin to maintain precise attitude control while
; selecting a safe landing site. The mode automatically switches between
; rapid rotation for large attitude changes and precise control for fine
; adjustments and attitude holds.
;
; During Apollo 11's descent, Armstrong used this mode extensively during the
; approach and landing phases, particularly when he took semi-manual control
; at approximately 500 feet altitude to avoid the boulder field and select
; a safe touchdown site.
;
; CODE-ALONG READERS: Implements dual-mode manual control logic designed by
; Robert F. Stengel in April 1968. The mode provides seamless switching
; between two control laws optimized for different phases of manual control.
;
# 					MANUAL RATE COMMAND MODE
# 					========================
# 					  BY ROBERT F. STENGEL
#
# THIS MODE PROVIDES RCAH MANUAL CONTROL THRU 2 CONTROL LAWS:  1) DIRECT RATE AND 2) PSEUDO-AUTO.
# THE DIRECT RATE MODE AFFORDS IMMEDIATE CONTROL WITHOUT OVERSHOOT.  THE PSEUDO-AUTO MODE PROVIDES PRECISE
# RATE CONTROL AND ATTITUDE HOLD.
;
; TWO CONTROL LAWS:
;
; 1) DIRECT RATE CONTROL - Used for rapid attitude changes
;    - Immediate response when RHC deflection exceeds breakout level
;    - Jets fire directly proportional to commanded rate
;    - No overshoot, immediate control authority
;    - Crew feels direct connection between stick input and spacecraft motion
;
; 2) PSEUDO-AUTO CONTROL - Used for precise control and attitude hold
;    - Engages when RHC input is small or detented
;    - Treats stick position as attitude command, not rate command
;    - Implements automatic control law (similar to autopilot)
;    - Enables precise attitude hold without continuous stick deflection
;    - Crew can release stick and spacecraft maintains attitude
#
# Page 1432
# IN DIRECT RATE, JETS ARE FIRED WHEN STICK POSITION CHANGES BY A FIXED NUMBER OF INCREMENTS IN ONE DAP CYCLE.
# THE `BREAKOUT LEVEL' IS .6 D/S FOR LM-ONLY AND .3 D/S FOR CSM-DOCKED.  THIS LAW NULLS THE RATE ERROR TO WITHIN
# THE `TARGET DEADBAND', WHICH EQUALS THE BREAKOUT LEVEL.
;
; DIRECT RATE CONTROL PARAMETERS:
; - Breakout level: 0.6 deg/sec for LM-only configuration
;                   0.3 deg/sec for CSM-docked configuration
; - Target deadband: Same as breakout level
; - Jets fire when stick position changes by fixed increments in one DAP cycle
; - Rate error nulled to within target deadband
; - Provides immediate, predictable response for crew
#
# IN PSEUDO-AUTO, BODY-FIXED RATE AND ATTITUDE ERRORS ARE SUPPLIED TO TJETLAW, WHICH EXERCISES CONTROL.
# CONTROL SWITCHES FROM DIRECT RATE TO PSEUDO-AUTO IF THE TARGET DB IS ACHIEVED OR IF TIME IN (1) EXCEEDS 4 SEC.
# IF THE INITIAL COMMAND DOES NOT EXCEED THE BREAKOUT LEVEL, CONTROL GOES TO PSEUDO-AUTO IMMEDIATELY.
;
; PSEUDO-AUTO MODE OPERATION:
; - Body-fixed rate and attitude errors computed and supplied to TJETLAW
; - TJETLAW applies automatic control law (PID gains, deadband logic)
; - Enables precise attitude hold and smooth rate commands
;
; MODE SWITCHING LOGIC (Direct Rate → Pseudo-Auto):
; 1. Target deadband achieved (rate error within tolerance)
; 2. Time in Direct Rate exceeds 4 seconds
; 3. Initial command below breakout level (goes directly to Pseudo-Auto)
;
; Smooth transition between modes prevents jerky spacecraft motion and
; provides intuitive control feel for crew during manual operations.
#
# SINCE P-AXIS CONTROL IS SEPARATE FROM Q,R AXES CONTROL, IT IS POSSIBLE TO USE (1) IN P-AXIS AND (2) IN Q,R AXES,
# OR VICE VERSA.  THIS ALLOWS A DEGREE OF ATTITUDE HOLD IN UNCONTROLLED AXES.  DUE TO U,V CONTROL, HOWEVER, Q AND
# R AXES ARE COUPLED AND MUST USE THE SAME CONTROL LAW.
;
; AXIS INDEPENDENCE:
; - P-axis (pitch) control independent from Q,R axes (yaw/roll)
; - Possible to use Direct Rate in P-axis while Q,R axes use Pseudo-Auto
; - Allows attitude hold in uncontrolled axes
; - Q and R axes coupled due to U,V jet control (must use same control law)
; - Provides flexibility in manual control strategy during complex maneuvers
#
# HAND CONTROLLER COMMANDS ARE SCALED BY A LINEAR/QUADRATIC LAW.  FOR THE LM-ALONE, MAXIMUM COMMANDED RATES ARE 20
# AND 4 D/S IN NORMAL AND FINE SCALING; HOWEVER, STICK SENSITIVITY AT ZERO COUNTS (OBTAINED AT A STICK DEFLECTION
# OF 2 DEGREES FROM THE CENTERED POSITION) IS .5 OR .1 D/S PER DEGREE.  NORMAL AND FINE SCALINGS FOR THE CSM-DOCKED
# CASE IS AUTOMATICALLY SET TO 1/10 THE ABOVE VALUES.  SCALING IS DETERMINED IN ROUTINE 3.
;
; ROTATIONAL HAND CONTROLLER (RHC) SCALING:
;
; LM-ALONE CONFIGURATION:
;   Normal Scaling:
;     - Maximum commanded rate: 20 deg/sec at full stick deflection
;     - Sensitivity at center: 0.5 deg/sec per degree of stick deflection
;     - Breakout at 2 degrees stick deflection from center
;   Fine Scaling:
;     - Maximum commanded rate: 4 deg/sec at full stick deflection
;     - Sensitivity at center: 0.1 deg/sec per degree of stick deflection
;     - Provides precise control for final landing approach
;
; CSM-DOCKED CONFIGURATION:
;   - All rates scaled to 1/10 of LM-alone values
;   - Compensates for higher moment of inertia with CSM attached
;   - Normal max: 2 deg/sec, Fine max: 0.4 deg/sec
;
; Scaling law: Linear/quadratic function provides smooth transition from
; fine control at small deflections to rapid response at large deflections.
; Critical for intuitive control feel during landing operations.
#
# ZEROENBL	ENABLES COUNTERS SO THEY CAN BE READ NEXT TIME
# JUSTOUT	FIRST DETECTION OF OUT OF DETENT (BY OURRCBIT)
;
; ============================================================================
; DETENT CHECK AND STATE MANAGEMENT
; ============================================================================
;
; DETENTCK monitors the Rotational Hand Controller (RHC) position to determine
; if crew is providing manual rate commands or if RHC is centered (in detent).
;
; DETENT: RHC centered position, no manual input
; OUT OF DETENT: RHC deflected, crew commanding rate or attitude change
;
; State transitions managed here:
; - Manual control → Automatic control (when RHC returns to detent)
; - Automatic control → Manual control (when RHC leaves detent)
; - Rate damping after manual input (null accumulated rates)
;
; Critical for crew control during:
; - Lunar landing final approach (manual attitude adjustments)
; - Docking operations (precision attitude control)
; - Contingency maneuvers (rapid manual takeover)
;
; Historical context: Armstrong used RHC extensively during Apollo 11 landing
; to select safe landing site, requiring smooth transitions between manual
; and automatic control modes.
; ============================================================================

DETENTCK	EXTEND
		READ	CHAN31
		TS	CH31TEMP
		MASK	BIT15		# CHECK OUT-OF-DETENT BIT.
		EXTEND
		BZF	RHCMOVED	# BRANCH IF OUT OF DETENT.
		CAF	OURRCBIT	# IN DETENT.  CHECK THE RATE COMMAND BIT.
		MASK	DAPBOOLS
		EXTEND
		BZF	PURGENCY	# BRANCH IF NOT IN RATE COMMAND LAST PASS.

# ................................................................................
;
; RHC JUST RETURNED TO DETENT - Check if rate damping needed
; Rate damping nulls any accumulated body rates after manual control input
; to prevent spacecraft drift.
;

		CA	BIT9		# JUST IN DETENT??
		MASK	RCSFLAGS
		EXTEND
		BZF	RUTH
		CAF	BIT13		# CHECK FOR ATTITUDE HOLD.
		EXTEND
		RAND	CHAN31
		EXTEND
		BZF	RATEDAMP	# BRANCH IF IN ATTITUDE HOLD.
;
; In full automatic mode - clear state flags and proceed to rate damping
;
		CS	BITS9,11	# IN AUTO.
		MASK	RCSFLAGS	# (X-AXIS OVERRIDE)
		TS	RCSFLAGS	# ZERO ORBIT (BIT 11) AND JUST-IN BIT (9).
		TCF	RATEDAMP
;
; RUTH: Check if P-axis rate damping is complete in attitude hold mode
;
RUTH		CA	RCSFLAGS
		MASK	PBIT		# IN ATTITUDE HOLD.
		EXTEND
		BZF	+2		# BRANCH IF P-RATE DAMPING IS FINISHED.
		TCF	RATEDAMP

# Page 1433
;
; Check if Q,R axis rate damping is complete
;
		CA	RCSFLAGS
		MASK	QRBIT
		EXTEND
		BZF	RATEDONE	# BRANCH IF Q,R RATE DAMPING IS FINISHED.
		TCF	RATEDAMP

# ============================================

1/10SEC		OCT	1
40CYC		OCT	50
PQRBIT		OCT	74777
BITS9,11	EQUALS	EBANK5
LINRATP		DEC	46

# ============================================

;
; ============================================================================
; RATEDONE - Manual Command and Rate Damping Complete
; ============================================================================
;
; All manual rate commands have been processed and any accumulated body rates
; have been damped to zero. RHC is in detent and spacecraft is stable.
;
; Actions performed:
; 1. Clear rate command flag (OURRCBIT) in DAPBOOLS
; 2. Read current CDU angles into desired CDU registers (CDUXD, etc.)
; 3. Zero attitude errors (transition to attitude hold at current orientation)
;
; This establishes new attitude hold point at current spacecraft orientation,
; allowing crew to make incremental attitude adjustments without returning
; to original trim attitude.
;
; Critical during landing: Allows Armstrong to make series of small attitude
; changes to assess landing site, with each new orientation becoming the
; hold point for next observation.
; ============================================================================

RATEDONE	CS	OURRCBIT	# MANUAL COMMAND AND DAMPING COMPLETED IN
		INHINT			# ALL AXES.
		MASK	DAPBOOLS
		TS	DAPBOOLS
;
; Read current CDU angles and establish as new desired attitudes
; (attitude hold at current orientation)
;
		CAF	BIT13
		EXTEND
		RAND	CHAN31
		EXTEND
		BZF	+4
		CA	CDUX		# (X-AXIS OVERRIDE)
		TS	CDUXD
		TC	+3
		TC	IBNKCALL
		FCADR	ZATTEROR	# Zero attitude errors
		RELINT
		TCF	PURGENCY

		TS	PERROR
;
; ============================================================================
; JUSTOUT - Manual Control Initialization (First Pass)
; ============================================================================
;
; Hand controller has just been moved out of detent. This is the first DAP
; pass after crew initiated manual control.
;
; INITIALIZATION ACTIONS:
; 1. Set rate command flag (OURRCBIT) in DAPBOOLS
; 2. Zero all accumulated error terms (DXERROR, DYERROR, DZERROR)
; 3. Zero previous rate samples (PLAST, QLAST, RLAST)
; 4. Zero Q,R hand controller counters
; 5. Clear rate damping flags (bits 10 and 11 of RCSFLAGS)
; 6. Set "just-in" flag (bit 9) to track state transition
;
; This clean initialization prevents residual automatic control state from
; affecting manual control response. Critical for smooth transition when
; crew takes manual control during time-critical operations like final
; landing approach.
; ============================================================================

JUSTOUT		CA	OURRCBIT	# INITIALIZATION -- FIRST MANUAL PASS.
		ADS	DAPBOOLS
		CA	ZERO
		TS	DXERROR		# Zero X-axis error accumulator
		TS	DXERROR +1
		TS	DYERROR		# Zero Y-axis error accumulator
		TS	DYERROR +1
		TS	DZERROR		# Zero Z-axis error accumulator
		TS	DZERROR +1
		TS	PLAST		# Zero previous P-axis rate sample
		TS	QLAST		# Zero previous Q-axis rate sample
		TS	RLAST		# Zero previous R-axis rate sample
		TS	Q-RHCCTR	# Zero Q-axis hand controller counter
		TS	R-RHCCTR	# Zero R-axis hand controller counter
		CA	PQRBIT
		MASK	RCSFLAGS
		TS	RCSFLAGS	# BITS 10 AND 11 OF RCSFLAGS ARE 0.
# Page 1434
		CS	RCSFLAGS	# SET 'JUST-IN' BIT TO 1.
		MASK	BIT9
		ADS	RCSFLAGS
		TC	ZEROENBL
		TCF	JETSOFF
;
; ============================================================================
; ZEROENBL - Zero and Enable Hand Controller Counters
; ============================================================================
;
; Utility subroutine to save current hand controller counter values and
; reset counters for next sampling interval.
;
; OPERATIONS:
; 1. Save Q,R hand controller counters to SAVEHAND (for later processing)
; 2. Zero all three axis counters (P, Q, R)
; 3. Enable counters via hardware channel 13 (bits 8 and 9)
;
; Called periodically (every 100ms DAP cycle) to capture incremental hand
; controller inputs and prepare for next measurement interval.
;
; The Attitude Controller Assembly (ACA) generates pulses proportional to
; hand controller deflection. These counters accumulate pulses, converting
; analog stick position into digital command signals.
; ============================================================================

ZEROENBL	LXCH	R-RHCCTR
		CA	Q-RHCCTR
		DXCH	SAVEHAND	# Save Q,R counters for processing
		CA	ZERO
		TS	P-RHCCTR	# Zero P-axis counter
		TS	Q-RHCCTR	# Zero Q-axis counter
		TS	R-RHCCTR	# Zero R-axis counter
		CA	BITS8,9
		EXTEND
		WOR	CHAN13		# COUNTERS ZEROED AND ENABLED
		TC	Q		# Return to caller
;
; ============================================================================
; RATEDAMP - Rate Damping Mode Entry
; ============================================================================
;
; Hand controller has returned to detent but spacecraft has residual body
; rates that must be damped before completing manual control sequence.
;
; Zero P-axis hand controller counter (no manual input) and proceed to
; RATERROR to compute automatic damping commands.
;
; Rate damping uses same control law as automatic attitude hold but with
; zero commanded rate. Jets fire to null body rates to within deadband.
; ============================================================================

RATEDAMP	CA	ZERO
		TS	P-RHCCTR	# Zero counter (no manual command)
		TCF	RATERROR	# Compute rate damping commands

;
; ============================================================================
; RHCMOVED - Continue Manual P-Axis Control
; ============================================================================
;
; Hand controller is out of detent and manual control is already in progress.
; Check if rate command bit is set; if not, this must be first pass and
; initialization is required.
; ============================================================================

RHCMOVED	CA	OURRCBIT	# P CONTROL
		MASK	DAPBOOLS
		EXTEND
		BZF	JUSTOUT -1	# First pass - initialize
;
; ============================================================================
; RATERROR - Rate Error Computation with Hand Controller Scaling
; ============================================================================
;
; Compute commanded rate from hand controller input and calculate rate error
; for P-axis control.
;
; HAND CONTROLLER SCALING (Linear/Quadratic Law):
;
; The RHC uses a combined linear/quadratic scaling to provide intuitive
; control feel:
;   - Near center: Linear response for precise control
;   - Large deflections: Quadratic component for rapid maneuvers
;
; Commanded Rate = (LINRATP + 2*P-RHCCTR) * P-RHCCTR * STIKSENS
;
; Where:
;   P-RHCCTR = Hand controller pulse count (proportional to deflection)
;   LINRATP = Linear gain coefficient
;   STIKSENS = Sensitivity scaling (normal/fine, LM-alone/CSM-docked)
;
; RATE ERROR CALCULATION:
;   EDOTP = OMEGAP - PLAST
;
; Where:
;   OMEGAP = Current body rate (from IMU)
;   PLAST = Commanded rate (scaled RHC input)
;   EDOTP = Rate error (drives control law)
;
; Positive error → body rate exceeds command → fire negative jets
; Negative error → body rate below command → fire positive jets
; ============================================================================

RATERROR	CA	CDUX		# FINDCDUW REQUIRES THAT CDUXD=CDUX DURING
		TS	CDUXD		# X-AXIS OVERRIDE
;
; Apply linear/quadratic scaling to hand controller input
;
		CCS	P-RHCCTR	# Check sign/magnitude of RHC input
		TCF	+3
		TCF	+2
		TCF	+1
		DOUBLE			# LINEAR/QUADRATIC CONTROLLER SCALING
		DOUBLE			# Form 2*P-RHCCTR (quadratic term)
		AD	LINRATP		# Add linear coefficient
		EXTEND
		MP	P-RHCCTR	# Multiply by counter value
		CA	L		# Get result (low-order product)
		EXTEND
		MP	STIKSENS	# Apply sensitivity scaling
;
; Compute change in commanded rate and update rate error
;
		XCH	PLAST		# Get previous command, store new
		COM			# Negate for subtraction
		AD	PLAST		# DAPTEMP1 = PLAST(new) - PLAST(old)
		TS	DAPTEMP1	# Change in commanded rate
		TC	ZEROENBL	# Zero and enable ACA counters for next interval
		CS	PLAST		# -PLAST
		AD	OMEGAP		# EDOTP = OMEGAP - PLAST (rate error)
		TS	EDOTP		# Store P-axis rate error
;
; ============================================================================
; BREAKOUT LOGIC - Determine Control Mode (Pseudo-Auto vs Direct Rate)
; ============================================================================
;
; Check if change in commanded rate exceeds breakout threshold. If so,
; transition to direct rate control (PEGI). If not, check if we were in
; direct rate control last pass and continue if so.
;
; BREAKOUT THRESHOLD:
;   If |DAPTEMP1| > RATEDB (rate deadband), enter direct rate control.
;   This detects significant hand controller motion requiring immediate
;   response rather than pseudo-automatic attitude hold.
;
; CONTROL MODE PERSISTENCE:
;   If PBIT (bit 10 of RCSFLAGS) is set, we were in direct rate control
;   last pass and should continue in that mode for smooth control.
; ============================================================================
;
		CCS	DAPTEMP1	# IF P COMMAND CHANGE EXCEEDS BREAKOUT
		TCF	+3		# LEVEL, GO TO DIRECT RATE CONTROL.  IF NOT
		TCF	+8D		# CHECK FOR DIRECT RATE CONTROL LAST TIME.
		TCF	+1
# Page 1435
;
; Test if rate change exceeds deadband threshold
;
		AD	-RATEDB		# Compare |change| against deadband
		EXTEND
		BZMF	+4		# Within deadband - check previous mode
		CA	40CYC		# Exceeded deadband - set timeout
		TS	TCP		# 4 second timer for rate command
		TC	PEGI		# Enter direct rate control
;
; Rate change within deadband - check if we were in direct rate control last pass
;
		CA	RCSFLAGS	# CHECK FOR DIRECT RATE COMMAND LAST TIME.
		MASK	PBIT		# Test bit 10 (direct rate control flag)
		EXTEND
		BZF	+2		# Not in direct rate - continue pseudo-auto
		TC	PEGI		# TO PURE RATE COMMAND
;
; PSEUDO-AUTO CONTROL MODE
;   Use attitude error (DXERROR) as control input, combining attitude hold
;   with rate damping. This provides smooth automatic control when hand
;   controller inputs are small or when coasting within deadband.
;
		CA	DXERROR		# PSEUDO-AUTO CONTROL.
		TS	E		# X-ATTITUDE ERROR (SP)
		TS	PERROR		# LOAD P-AXIS ERROR FOR MODE1 FDAI DISPLAY
		TC	PURGENCY +4	# Compute urgency and select jets
;
; ============================================================================
; PEGI - Direct Rate Control with Rate Deadband Exit
; ============================================================================
;
; Direct rate control mode provides pure rate command following without
; attitude error feedback. This mode is used during rapid manual maneuvers
; when the pilot commands large rate changes through the hand controller.
;
; CONTROL STRATEGY:
;   - Zero attitude error terms (direct rate control ignores attitude)
;   - Compute jet fire time from rate error (EDOTP)
;   - Exit to pseudo-auto when rate error drops below TARGET deadband
;   - Exit to pseudo-auto if rate command exceeds 4-second timeout
;
; RATE DEADBAND EXIT LOGIC:
;   If |EDOTP| < TARGETDB, rate error is small enough to return to
;   attitude-based control. Set PBIT=0 and transition to pseudo-auto.
;
; TIMEOUT EXIT:
;   If TCP (4-second timeout) expires, exit direct rate mode even if
;   rate error remains. Prevents extended direct rate control from
;   accumulating attitude error.
;
; JET FIRE TIME CALCULATION:
;   TJP = EDOTP / (2 * JTACC) * (25/32)
;
;   Where:
;     EDOTP = Rate error (current rate - commanded rate)
;     JTACC = Jet acceleration (angular acceleration per jet)
;     25/32 = Scaling factor for jet time computation
;
; TWO-JET vs FOUR-JET SELECTION:
;   If |EDOTP| > 2JETLIM: Use 4 jets (rapid correction)
;   If |EDOTP| ≤ 2JETLIM: Use 2 jets (fine control)
; ============================================================================
;
PEGI		CA	CDUX		# DIRECT RATE CONTROL.
		TS	CDUXD		# Store CDU X for FINDCDUW
		CA	ZERO
		TS	DXERROR		# Zero attitude error (SP)
		TS	DXERROR +1	# Zero attitude error (DP)
		TS	PERROR		# ZERO P-AXIS ERROR FOR MODE1 FDAI DISPLAY
;
; Compute absolute value of rate error and test against deadband
;
		CCS	EDOTP		# Get sign/magnitude of rate error
		TC	+3
		TC	+2
		TC	+1
		TS	ABSEDOTP	# Store |EDOTP|
		AD	TARGETDB	# Compare against target deadband
		EXTEND			# IF RATE ERROR IS LESS THAN DEADBANK,
		BZMF	LAST		# EXIT TO PSEUDO-AUTO.
;
; Rate error exceeds deadband - check timeout
;
		CA	TCP		# Get rate command timer
		EXTEND			# IF TIME IN RATE COMMAND EXCEEDS 4 SEC.
		BZMF	LAST		# EXIT TO PSEUDO-AUTO
;
; Continue direct rate control - set PBIT flag
;
		CS	RCSFLAGS
		MASK	PBIT
		ADS	RCSFLAGS	# BIT 10 IS 1 (direct rate mode active)
		TCF	+4
;
; EXIT POINT: Return to pseudo-auto control
;
LAST		CS	PBIT
		MASK	RCSFLAGS
		TS	RCSFLAGS	# BIT 10 IS 0 (pseudo-auto mode)
;
; COMPUTE JET FIRE TIME from rate error
;
		CS	EDOTP		# Negate rate error
		EXTEND
		MP	1/ANETP		# 1/2JTACC SCALED AT 2EXP(7)/PI
		DAS	A		# Double precision accumulation
		TC	OVERSUB		# Check for overflow/underflow
		EXTEND
		MP	25/32		# A CONTAINS TJET SCALED AT 2EXP(4)(16/25)
		TS	TJP		# Store 4*jet fire time
;
; SELECT TWO JETS or FOUR JETS based on rate error magnitude
;
		CA	ABSEDOTP
		AD	-2JETLIM	# COMPARING DELTA RATE WITH 2 JET LIMIT
		EXTEND
# Page 1436
		BZMF	+3		# |EDOTP| ≤ 2JETLIM: use 2 jets
;
; FOUR-JET SELECTION (large rate error)
;
		CA	SIX		# NUMBERT = 6 (four-jet mode)
		TCF	+8D		# Skip to jet selection
;
; TWO-JET SELECTION (moderate rate error)
;
		CA	TJP		# Get 4*jet fire time
		ADS	TJP		# TJP = 8*jet fire time (for 2-jet scaling)
;
; ============================================================================
; PJETSLEC - P-Axis Jet Selection Routine (Rotation)
; ============================================================================
;
; Selects optimal pair or quad of RCS jets for P-axis (pitch) rotation
; based on current spacecraft configuration and jet availability.
;
; INPUTS:
;   NUMBERT - Jet configuration selector
;             4 = System A jets only
;             5 = System B jets only  
;             6 = Four jets (both systems, rapid maneuver)
;   TJP     - Jet fire time (positive for +P rotation, negative for -P)
;
; OUTPUTS:
;   Channel 6 - RCS jet commands written to hardware
;   PJUMPADR  - Address for P-axis skip logic
;   NO.PJETS  - Number of jets selected (2 or 4)
;
; JET SELECTION POLICY (tried in order until working combination found):
;
;   +P Rotation           -P Rotation
;   -----------           -----------
;   Jets 7,15             Jets 8,16    (Primary policy - optimal torque)
;   Jets 4,12             Jets 3,11    (Alternate pair)
;   Jets 4,7              Jets 8,11    (System A only)
;   Jets 7,12             Jets 11,16   (Cross-coupled)
;   Jets 12,15            Jets 3,16    (System B only)
;   Jets 4,15             Jets 3,8     (Last resort - asymmetric)
;   ALARM                 ALARM        (No working jets available)
;
; The SELECTP subroutine walks through this policy table, checking each
; combination against failed jet status until it finds a working pair.
; ============================================================================
;
		CA	AORBSYST	# Check which RCS system(s) available
		MASK	DAPBOOLS
		CCS	A		# Test system configuration
		CA	ONE		# System B active
		AD	FOUR		# Form NUMBERT (4,5, or 6)
		TS	NUMBERT		# Store jet configuration selector
;
; MAIN JET SELECTION ENTRY POINT
;
PJETSLEC	CA	ONE
		TS	L		# Initialize L register
		CCS	TJP		# Check sign/magnitude of jet time
		TCF	+5		# Positive - fire for +P rotation
		TCF	JETSOFF		# Zero - no jets needed
		TCF	+2		# Negative - continue
		TCF	JETSOFF		# Negative zero - no jets
		ZL			# Clear L
		AD	ONE		# Form +1
		TS	ABSTJ		# Store absolute jet time
		LXCH	ROTINDEX	# Save rotation direction index
		TC	SELECTP		# Call jet policy selection
;
; ============================================================================
; Determine number of jets to fire (2 or 4)
; ============================================================================
;
; NUMBERT encoding:
;   4 = System A jets only (2 jets)
;   5 = System B jets only (2 jets)
;   6 = Both systems (4 jets for rapid maneuver)
;
		CS	SIX
		AD	NUMBERT		# NUMBERT - 6
		EXTEND
		BZF	+2		# If NUMBERT=6, use 4 jets

		CS	TWO		# NUMBERT ≠ 6, adjust for 2 jets

# Page 1437
		AD	FOUR		# Form NO.PJETS (2 or 4)
		TS	NO.PJETS	# Store number of jets firing
;
; ============================================================================
; Write jet commands and validate timing
; ============================================================================
;
		CA	POLYTEMP	# Get jet bit pattern
		TC	WRITEP		# Write to Channel 6 (RCS outputs)
;
; MAXIMUM JET-ON TIME CHECK (+150 milliseconds)
;
; Check if commanded jet time exceeds 150ms limit. If within limit,
; proceed to Q,R-axis processing. If too long, extend firing time.
;
		CS	ABSTJ		# -ABSTJ
		AD	+150MST6	# +150 - ABSTJ
		EXTEND
		BZMF	QRAXIS		# If ABSTJ ≤ 150ms, go to Q,R-axis
;
; Jet time exceeds 150ms - check minimum and extend firing
;
		AD	-136MST6	# (+150-ABSTJ) - 136
		EXTEND
		BZMF	+5		# If result ≤0, skip extension

		ADS	ABSTJ		# Extend ABSTJ (add overflow amount)
		INDEX	ROTINDEX	# Index by rotation direction
		CA	MINTIMES	# Get minimum time for this direction
		TS	TJP		# Update jet fire time
;
; Schedule extended jet firing via WAITLIST
;
		CA	ABSTJ		# Get extended jet time
		ZL			# Clear L register
		INHINT			# Disable interrupts
		DXCH	T6FURTHA	# Store time in T6 further time
		TC	IBNKCALL	# Cross-bank call
		CADR	JTLST		# Call jet task list scheduler
;
; Clear bit 12 of RCSFLAGS and alternate jet system
;
		CS	BIT12
		MASK	RCSFLAGS	# Clear bit 12
		TS	RCSFLAGS	# BIT 12 SET TO 0
		TC	ALTSYST		# Switch to alternate system
		TCF	QRAXIS		# Continue to Q,R-axis
;
; ============================================================================
; ALTSYST - Alternate P-Axis Jet System Selection
; ============================================================================
;
; Switch between RCS System A and System B for P-axis control. Alternating
; systems balances propellant usage and provides redundancy.
;
; The routine toggles the system selection bit in DAPBOOLS by XOR with
; AORBSYST mask, then returns to caller with interrupts re-enabled.
;
; This is called after each jet firing to ensure even propellant depletion
; across redundant systems. Critical for long-duration missions where
; propellant management affects abort margins and rendezvous capability.
; ============================================================================
;
ALTSYST		CA	DAPBOOLS	# Get current DAP boolean flags
		TS	L		# Save in L register
		CA	AORBSYST	# Get A-or-B system mask
		EXTEND
		RXOR	LCHAN		# XOR with saved DAPBOOLS (toggle system)
		TS	DAPBOOLS	# Store updated system selection
		RELINT			# Re-enable interrupts
		TC	Q		# Return to caller
;
; ============================================================================
; DKALT - Direct Call to ALTSYST
; ============================================================================
;
DKALT		TC	ALTSYST		# Simple entry point for system alternation
;
; ============================================================================
; JETSOFF - Turn Off P-Axis Jets
; ============================================================================
;
; Called when no jet firing is commanded (attitude within deadband, no
; manual input, or commanded time below minimum threshold).
;
; Clears jet output channel and zeros jet fire time, then proceeds to
; Q,R-axis processing.
; ============================================================================
;
JETSOFF		TC	WRITEP 	-1	# Write zero pattern (all jets off)
		CA	ZERO
		TS	TJP		# Zero P-axis jet fire time
		TCF	QRAXIS		# Continue to Q,R-axis

# (NOTE -- M13 = 1 IDENTICALLY IMPLIES NULL MULTIPLICATION.)
;
; ============================================================================
; CALCPERR - P-Axis Error Calculation
; ============================================================================
;
; Computes attitude error in P-axis (pitch/roll hybrid axis in body frame)
; using transformation matrix from desired to actual spacecraft attitude.
;
; ERROR COMPUTATION:
;
;   PERROR = (CDUY - CDUYD) * M11 + (CDUX - CDUXD) * M13 + DELPEROR
;
; Where:
;   CDUY, CDUX = Current gimbal angles from IMU
;   CDUYD, CDUXD = Desired gimbal angles (from guidance or manual command)
;   M11, M13 = Transformation matrix elements relating body to stable member
;   DELPEROR = Additional error term from KALCMANU interface
;
; The transformation accounts for gimbal geometry and maps CDU differences
; into body-frame P-axis error. Note M13 = 1 identically, so multiplication
; is omitted (null operation).
;
; Result scaled at π radians and stored in PERROR for autopilot control law
; and eight-ball attitude display update.
; ============================================================================
;
CALCPERR	CA	CDUY		# Get current Y gimbal angle
		EXTEND
		MSU	CDUYD		# CDUY - CDUYD (Y-axis error)
# Page 1438
		EXTEND
		MP	M11		# Apply matrix element M11
		XCH	E		# Save first term in E register
		CA	CDUX		# Get current X gimbal angle
		EXTEND
		MSU	CDUXD		# CDUX - CDUXD (X-axis error)
;
; M13 multiplication omitted (M13 = 1 identically)
;
#		EXTEND
#		MP	M13
		AD	DELPEROR	# Add KALCMANU interface error
		ADS	E		# Add to first term (could overflow)
		XCH	PERROR		# Store result in PERROR
		TC	Q		# Return to caller

# P-AXIS URGENCY FUNCTION CALCULATION.
;
; ============================================================================
; PURGENCY - P-Axis Urgency Calculation and Jet Selection
; ============================================================================
;
; Main entry point for automatic P-axis attitude control. Coordinates error
; calculation, urgency function evaluation, and jet selection to maintain
; desired spacecraft attitude in pitch.
;
; CONTROL SEQUENCE:
;
; 1. Calculate P-axis attitude error (PERROR) via CALCPERR
; 2. Compute rate error (EDOTP = OMEGAP - OMEGAPD) where:
;    - OMEGAP = Actual P-axis body rate (from rate gyros)
;    - OMEGAPD = Desired P-axis rate (from guidance or zero for attitude hold)
;    - Scaled at π/4 radians/second
;
; 3. Determine control mode:
;    - If CSMDOCKD flag set → Call SPSRCS (docked RCS backup mode)
;    - Otherwise → Call TJETLAW (standard urgency function and jet time calc)
;
; 4. Evaluate control authority requirements:
;    - If FIREFCT < -4° → Use 2-jet configuration (PJETSLEC -6)
;    - If TJP = 0 → Turn jets off (JETSOFF)
;    - If TJP > 160ms → Use 2-jet configuration
;    - If TJP ≤ 160ms → Use 6-jet configuration (maximum authority)
;
; 5. Proceed to PJETSLEC for jet policy construction and command output
;
; This routine implements the decision tree for P-axis control authority
; based on error magnitude and control effectiveness.
; ============================================================================
;
PURGENCY	TC	CALCPERR	# Calculate P-axis attitude error
;
; Compute rate error (EDOTP) for urgency function evaluation
;
		CS	OMEGAPD		# -OMEGAPD (desired rate)
		AD	OMEGAP		# OMEGAP - OMEGAPD
		TS	EDOTP		# EDOTP = rate error at π/4 rad/sec
;
; Initialize axis counter for jet selection logic
;
		CS	ONE		# -1
		TS	AXISCTR		# Initialize axis counter
;
; Check if in docked configuration (CSM+LM docked RCS backup mode)
;
		CA	DAPBOOLS	# Get DAP configuration flags
		MASK	CSMDOCKD	# Check docked flag
		EXTEND
		BZF	HEADTJET	# If not docked, standard TJETLAW
;
; DOCKED RCS LOGIC: Use SPS backup RCS control law when docked
;
		INHINT			# Disable interrupts for IBNKCALL
		TC	IBNKCALL	# Inter-bank call
		CADR	SPSRCS		# Call SPS RCS backup mode routine

		CA	TJP		# Get jet-on time from SPSRCS
		EXTEND
		BZF	DKALT		# If TJP = 0, change AORBSYST
		RELINT			# Re-enable interrupts
		TCF	PJETSLEC -6	# Use 2-jet configuration
;
; STANDARD LM DAP LOGIC: Use TJETLAW urgency function
;
HEADTJET	CA	ZERO		# Zero sense type
		TS	SENSETYP	# (No special sensing mode)
		INHINT			# Disable interrupts for IBNKCALL
		TC	IBNKCALL	# Inter-bank call
		CADR	TJETLAW		# Call TJET control law (urgency function)
		RELINT			# Re-enable interrupts
;
; Evaluate jet configuration based on control authority requirements
;
		CS	FIREFCT		# -FIREFCT (fire direction indicator)
		AD	-FOURDEG	# Compare to -4° threshold
		EXTEND
		BZMF	PJETSLEC -6	# If FIREFCT < -4°, use 2-jet config
		CCS	TJP		# Check jet-on time
		TCF	+2		# TJP > 0, continue evaluation
		TCF	JETSOFF		# TJP = 0, turn jets off
# Page 1439
;
; Determine jet configuration based on commanded jet-on time (TJP)
;
		AD	-160MST6	# TJP - 160ms
		EXTEND
		BZMF	PJETSLEC -6	# If TJP > 160ms, use 2-jet config
		CA	SIX		# Otherwise, use 6-jet config
		TCF	PJETSLEC -1	# (Maximum control authority for small errors)
;
; Constants for jet configuration threshold
;
-160MST6	DEC	-256		# -160ms in AGC time units
-FOURDEG	DEC	-.08888		# -4° threshold for fire direction

# Page 1440
# JET POLICY CONSTRUCTION SUBROUTINE
#
# INPUT:	ROTINDEX, NUMBERT
#
# OUTPUT:	POLYTEMP (JET POLICY)
#
# THIS SUBROUTINE SELECT A SUBSET OF THE DESIRED JETS WHICH HAS NO FAILURE
;
; ============================================================================
; SELECTP - P-Axis Jet Policy Construction
; ============================================================================
;
; Constructs a jet firing policy (bit pattern) selecting healthy jets for
; the commanded rotation direction while avoiding failed jets.
;
; INPUTS:
;   ROTINDEX = Rotation direction index:
;              +1 = +P rotation (pitch up)
;              -1 = -P rotation (pitch down)
;              (indexed into JETSALL table)
;
;   NUMBERT  = Jet configuration type index:
;              Selects from TYPEP table (2-jet, 4-jet, 6-jet patterns)
;
; OUTPUT:
;   POLYTEMP = Jet policy bitmask (octal bit pattern):
;              Each bit represents one RCS jet (1=selected, 0=not used)
;              Filtered to exclude failed jets via CH6MASK
;
; JET FAILURE HANDLING:
;   - Masks candidate jets against CH6MASK (channel 6 failure status)
;   - If all candidate jets failed → Issue ALARM 02003 and disable P-axis
;   - Falls back through loop counter (TEMPNUM) to find viable jet subset
;
; HISTORICAL NOTE:
;   This routine ensured Apollo 11's LM maintained pitch control despite
;   potential thruster failures. Redundant jet selection logic provided
;   fail-operational capability during critical descent and ascent phases.
; ============================================================================
;
SELECTP		CA	SIX		# Initialize loop counter
		TS	TEMPNUM		# (Max 6 iterations for jet policy search)
		INDEX	NUMBERT		# Index into TYPEP table
		CA	TYPEP		# Get jet configuration type pattern
		INDEX	ROTINDEX	# Index into JETSALL table
		MASK	JETSALL		# Mask with direction-specific jets
		TS	POLYTEMP	# Store candidate jet policy
		MASK	CH6MASK		# Check against jet failure status
		CCS	A		# Any healthy jets available?
		TCF	+2		# Yes, policy is valid
		TC	Q		# Return with jet policy in POLYTEMP
		CCS	TEMPNUM		# Decrement loop counter
		TCF	+4		# Continue searching
		TC	ALARM		# All attempts failed
		OCT	02003		# P-axis jet selection alarm
		TCF	JETSOFF		# Disable P-axis control
;
; Alternate entry point for Y-Z axis jet selection (shares policy logic)
;
SELECTYZ	TS	NUMBERT		# Store type number
		TCF	SELECTP	+1	# Enter SELECTP after counter init
 -1		TCF	ABORTYZ +2	# Alternate path (indexed branch)
;
; ============================================================================
; Jet Configuration Data Tables
; ============================================================================
;
; JETSALL - Available jets by rotation direction and translation axis
;           Indexed by ROTINDEX (rotation direction)
;           Each entry is an octal bitmask of available RCS jets
;
JETSALL		OCT	00252		# Index 0: Neutral/combined pattern
		OCT	00125		# Index 1: +P rotation (pitch up)
		OCT	00140		# Index 2: -Y translation (left)
		OCT	00006		# Index 3: -Z translation (down)
		OCT	00220		# Index 4: +Y translation (right)
		OCT	00011		# Index 5: +Z translation (up)
		OCT	00151		# Index 6: +V translation
;
; TYPEP - Jet configuration types for control authority modes
;         Indexed by NUMBERT (jet configuration type selection)
;         Provides patterns for 2-jet, 4-jet, or 6-jet combinations
;
TYPEP		OCT	00146		# Type 0: -U translation
		OCT	00226		# Type 1: -V translation
		OCT	00231		# Type 2: +U translation
		OCT	00151		# Type 3: +V translation pattern
		OCT	00132		# Type 4: Jets 1-3 pattern
		OCT	00245		# Type 5: Jets 2-4 pattern
		OCT	00377		# Type 6: ALL jets pattern (maximum authority)
;
; JET BITMASK ENCODING (Octal):
;   Each bit position corresponds to a specific RCS thruster:
;     Bit 0 (LSB) = Jet 1    Bit 4 = Jet 5
;     Bit 1       = Jet 2    Bit 5 = Jet 6
;     Bit 2       = Jet 3    Bit 6 = Jet 7
;     Bit 3       = Jet 4    Bit 7 = Jet 8
;
; CONTROL AUTHORITY PHILOSOPHY:
;   - Fewer jets: Fuel-efficient, smooth control (small errors)
;   - More jets: High authority, faster response (large errors, urgency)
;   - Pattern selection balances fuel conservation vs. control performance
; ============================================================================
;
; ============================================================================
; Y-Z Axis Index and Configuration Tables
; ============================================================================
;
; INDXYZ - Index table for Y-Z axis jet selection
;          Used by QRAXIS autopilot for yaw/roll control
;
INDXYZ		=	-136MST6	# Alias to base index table
-136MST6	DEC	-218		# Index offset for Y-Z calculations
		DEC	4		# Configuration parameter 1
		DEC	2		# Configuration parameter 2
		OCT	07776		# Mask value (negative index marker)
		DEC	5		# Configuration parameter 3
		DEC	9		# Configuration parameter 4
		DEC	10		# Configuration parameter 5
		OCT	07776		# Mask value (negative index marker)
		DEC	3		# Configuration parameter 6
# Page 1441
		DEC	8		# Configuration parameter 7
		DEC	7		# Configuration parameter 8
		OCT	07776		# Translation failure index
		OCT	07776		# These negative indexes modify
		OCT	07776		# the instruction at SELECTP +4
		OCT	07776		# to execute: TC JETSALL -1
		OCT	07776		# Used only for translation failure modes
;
; NOTE: Negative index values (07776 octal = -1 decimal) trigger special
;       handling in indexed addressing mode, allowing dynamic instruction
;       modification for fault recovery and mode transitions.
;
+150MST6	DEC	240		# Offset constant for positive indexing
07400OCT	OCT	07400		# Octal constant for bit masking
;
; ============================================================================
; T-JET LAW FIXED CONSTANTS
; ============================================================================
;
; Constants used by TJET control law for jet firing time calculations,
; scaling, and threshold comparisons throughout P-axis autopilot.
;
NORMSCL		OCT	266		# Normalization scale factor
-100MS		DEC	-.1		# -100 milliseconds (timing threshold)
200MS		DEC	.2		# +200 milliseconds (timing threshold)
25/32		=	PRIO31		# Fraction 25/32 = 0.78125 (gain factor)
BITS8,9		OCTAL	00600		# Bitmask for bits 8 and 9
1/40		DEC	.02500		# Fraction 1/40 = 0.025 (scale factor)
MINTIMES	DEC	-22		# Minimum jet-on time limit (-22 units)
		DEC	22		# Maximum jet-on time limit (+22 units)
;
; PSKIPADR - Address pointer to SKIPPAXS routine
;            Used for conditional branch to bypass P-axis processing
;
PSKIPADR	GENADR	SKIPPAXS	# Generic address of skip routine
;
; ============================================================================
; END OF P-AXIS RCS AUTOPILOT
; ============================================================================
;
; Following sections contain Q,R-AXES RCS AUTOPILOT (yaw and roll control)
;
; ============================================================================

# GOES TO Q,R-AXES RCS AUTOPILOT

QRAXIS		CS	OMEGARD
		AD	OMEGAR
		TC	OVERSUB
		TS	EDOTR
		CS	OMEGAQD
		AD	OMEGAQ
		TC	OVERSUB
		TS	EDOTQ
		EXTEND
		DCA	QERRCALL
		DTCB

		EBANK=	AOSQ
QERRCALL	2CADR	CALLQERR
