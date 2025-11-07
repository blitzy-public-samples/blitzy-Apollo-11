# Copyright:	Public domain.
# Filename:	TVCDAPS.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	961-978
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-20 RSB	Corrections:  Eliminated an extraneous EXTEND,
#				added a missing instruction to PFORWARD.
#		2000-05-21 RSB	Wrong opcode was used with DELBRTMP and
#				DELBRTMP +1 operands in 4 places.  Corrected
#				an MP operation in 2CASFLTR.
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

# Page 961
; ============================================================================
; FILE: TVCDAPS.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth (SPS burns)
;
; TL;DR: TVC digital autopilot implementing PID control loops for pitch, yaw,
;        and roll axes during SPS main engine burns. Generates engine gimbal
;        commands to correct attitude errors and maintain desired spacecraft
;        orientation during critical Apollo 11 maneuvers including translunar
;        injection (TLI), lunar orbit insertion (LOI), and transearth injection
;        (TEI). Responds to body-axis rate commands from cross-product steering
;        during powered flight or maintains attitude hold during impulsive burns.
;
; COMMENT-ONLY READERS: This autopilot steered the main rocket engine to keep
;        the spacecraft pointed correctly during major burns like the one that
;        sent Apollo 11 to the Moon and the one that brought it home to Earth.
; CODE-ALONG READERS: Study PID control law implementation with integral error
;        accumulation, 6th-order cascade filter structure (CSM or CSM/LM
;        configuration), gimbal command generation, and actuator limiting.
; ============================================================================
;
# PROGRAM NAME....TVCDAP, CONSISTING OF PITCHDAP, YAWDAP, ETC.
# LOG SECTION...TVCDAPS				SUBROUTINE...DAPCSM
# MODIFIED BY SCHLUNDT				21 OCTOBER 1968
;
; ============================================================================
; TRANSITION: TVC System Overview
;
; The Command Module's Service Propulsion System (SPS) engine is the primary
; propulsion system for major maneuvers during the mission. This digital
; autopilot (DAP) keeps the spacecraft pointed in the correct direction during
; engine burns by gimbaling (tilting) the engine to correct for any attitude
; errors. Without this autopilot, even small misalignments during the critical
; translunar injection burn could have sent Apollo 11 off course to miss the
; Moon entirely.
; ============================================================================
;
# FUNCTIONAL DESCRIPTION....

#	SELF-PERPETUATING T5 TASKS WHICH GENERATE THE COMMAND SIGNALS
#	FOR THE PITCH AND YAW SPS GIMBAL ACTUATORS DURING TVC (SPS) BURNS,
#	IN RESPONSE TO BODY-AXIS RATE COMMANDS FROM CROSS-PRODUCT STEERING
#	(S40.8).  IF NO STEERING (IMPULSIVE BURNS) MAINTAINS ATTITUDE-HOLD
#	ABOUT THE REFERENCE (INITIAL) DIRECTIONS (ZERO RATE COMMANDS).

#	THE PITCH AND YAW LOOPS ARE SEPARATE, BUT STRUCTURED IDENTICALLY.
#	EACH ATTITUDE-RATE LOOP INCLUDES GIMBAL ANGLE RATE DERIVATION,
#	GIMBAL/BODY AXIS TRANSFORMATION, BODY-AXIS ATTITUDE ERROR
#	INTEGRATION WITH ERROR LIMITING, THE GENERALIZED 6TH-ORDER FILTER
#	FOR CSM OR CSM/LM OPERATION. A FILTER OUTPUT LIMITER.
#	CG-OFFSET TRACKER FILTER, AND THE CG-TRACKER MINOR LOOP.

#	THE DAPS ARE CYCLIC, CALLING EACH OTHER AT 1/2 THE DAP SAMPLE
#	TIME, AS DETERMINED BY T5TVCDT.  THE ACTUATOR COMMANDS ARE
#	REGENERATED AS ANALOG VOLTAGES BY THE OPTICS ERROR COUNTERS, WHICH
#	TRANSMIT THE SIGNAL TO THE ACTUATOR SERVOS WHEN THERE IS PROPER CDU
#	MODING.

# CALLING SEQUENCE.... (TYPICALLY)

#	T5 CALL OF TVCDAPON (TVCINITIALIZE) BY DOTVCON (P40)
#	T5 CALL OF DAPINIT (TVCDAPS) BY TVCINIT4 (TVCINITIALIZE)
#	T5 CALL OF PITCHDAP BY DAPINIT
#	T5 CALL OF YAWDAP BY PITCHDAP
#	T5 CALL OF PITCHDAP BY YAWDAP
#		   ETC.
#	(AUTOMATIC SEQUENCING FROM TVCDAPON)

# NORMAL EXIT MODE....RESUME

# ALARM OR ABORT EXIT MODES....NONE

# SUBROUTINES CALLED....

#	HACK FOR STROKE TEST (V68) WAVEFORM GENERATION
#	PCOPY, YCOPY FOR COPY-CYCLES (USED ALSO BY TVC RESTART PACKAGE)
#	DAPINIT FOR INITIAL CDUS FOR RATE MEASUREMENTS
#	ERRORLIM, ACTLIM FOR INPUT (ATTITUDE-ERROR INTEGRATION) AND
#		OUTPUT (ACTUATOR COMMAND) LIMITING, COMMON TO PITCH AND
#		YAW DAPS
#	FWDFLTR (INCLUDING OPTVARK) AND PRECOMP, TO COMPUTE FILTER
#		OUTPUTS AND STORAGE VALUES
#	RESUME

# Page 962
# OTHER INTERFACES....

#	S40.8 CROSS-PRODUCT STEERING FOR BODY AXIS RATE COMMANDS OMEGAY,ZC
#	S40.15 FOR THE INITIAL DAP GAINS VARK AND 1/CONACC
#	TVCEXECUTIVE FOR DAP GAIN UPDATES AND TMC LOOP OPERATIONS
#	TVCRESTART PACKAGE FOR TVC RESTART PROTECTION.

# ERASABLE INITIALIZATION REQUIRED....

# 	PAD-LOAD ERASABLES ( SEE ERASABLE ASSIGNMENTS )
#	CONFIGURATION BITS (14, 13) OF DAPDATR1 AS IN R03
#	ENGINE-ON BIT (11.13) FOR RESTARTS
#	TVCPHASE FOR RESTARTS ( SEE DOTVCON, AND TVCINIT4 )
#	T5 BITS (15,14 OF FLAGWRD6) FOR RESTARTS
#	MISCELLANEOUS VARIABLES SET UP OR COMPUTED BY TVCDAPON....TVCINIT4,
#		INCLUDING THE ZEROING OF TEMPORARIES BY MRCLEAN
#	CDUX,Y,Z AND SINCDUX....COSCDUX AS PREPARED BY QUICTRIG (WITH
#		UPDATES EVERY 1/2 SECOND)
#	ALSO G+N PRIMARY, TVC ENABLE, AND OPTICS ERROR COUNTER ENABLE
#		UNLESS BENCH-TESTING.
#
# OUTPUT....
#
#	TVCPITCH AND TVCYAW WITH COUNTER RELEASE (11.14 AND 11.13 INCREMEN-
#		TAL COMMANDS TO OPTICS ERROR COUNTERS), FILTER NODES, BODY-
#		AXIS ATTITUDE ERROR INTEGRATOR, TOTAL ACTUATOR COMMANDS,
#		OFFSET-TRACKER-FILTER OUTPUTS, ETC.
# DEBRIS....

#	MUCH, SHAREABLE WITH RCS/ENTRY, IN EBANK6 ONLY

		BANK	17
		SETLOC	DAPS2
		BANK

		EBANK=	BZERO

		COUNT*	$$/DAPS

# Page 963
# PITCH TVCDAP STARTS HERE....(INCORPORATES CSM/LEM DAP FILTER, MODOR DESIGN)
;
; ============================================================================
; PITCHDAP - Pitch Axis Digital Autopilot
;
; This routine controls the pitch gimbal of the SPS engine. During a burn,
; if the spacecraft starts to rotate unintentionally (pitch up or down), this
; autopilot detects the rotation and commands the engine to tilt in the
; opposite direction to counteract it. The pitch axis is perpendicular to the
; spacecraft's longitudinal axis - think of it as nodding "yes" motion.
;
; For Apollo 11, this autopilot maintained precise attitude control during the
; translunar injection burn that sent the spacecraft to the Moon, the lunar
; orbit insertion burn that captured Apollo 11 into lunar orbit, and the
; transearth injection burn that brought the crew home.
;
; For code-along readers: This is a self-perpetuating T5 task that executes
; every DAP cycle (typically 40-80ms depending on T5TVCDT). It computes gimbal
; angle rates from CDU measurements, performs body-to-gimbal axis transformation,
; integrates attitude errors with limiting, runs a 6th-order cascade filter
; optimized for CSM or CSM/LM configuration, and generates actuator commands
; within physical gimbal limits (±6 degrees).
; ============================================================================
;
PITCHDAP	LXCH	BANKRUPT	# T5 ENTRY, NORMAL OR VIA DAPINIT
		EXTEND
		QXCH	QRUPT
;
; T5 Task Setup - Scheduling the Next Autopilot Cycle
; The pitch and yaw autopilots alternate, each calling the other at half the
; DAP sample rate. This creates a continuous control loop during engine burns.
; After pitch completes, it schedules yaw to run next; yaw then schedules pitch.
;
		CAF	YAWT5		# SET UP T5 CALL FOR YAW AUTOPILOT (LOW-
		TS	T5LOC		#	ORDER PART OF 2CADR ALREADY THERE)
		CAE	T5TVCDT
		TS	TIME5

;
; Stroke Test Check - Pre-Flight Gimbal Testing
; Before flight, crew uses V68 to run a stroke test that exercises the engine
; gimbals through their full range of motion to verify the actuators work.
; During actual burns, STROKER is zero and this test is bypassed.
;
PSTROKER	CCS	STROKER		# (STRKFLG) CHECK FOR STROKE TEST
		TC	HACK		# TEST-START OR TEST-IN-PROGRESS
		TCF	+2		# NO-TEST
		TC	HACK		# TEST-IN-PROGRESS
;
; CDU Rate Computation - Measuring Spacecraft Rotation
; The spacecraft's attitude is measured by resolvers (CDUs - Coupling Data Units)
; on the IMU gimbals. By differencing the current CDU reading with the previous
; reading taken 1/2 DAP cycle ago, we compute the rotation rate. CDUY measures
; rotation about the Y-axis (affects both pitch and yaw due to gimbal coupling).
; CDUZ measures rotation about the Z-axis (also affects both pitch and yaw).
;
; Rate limiting prevents sensor glitches from commanding excessive gimbal motion.
;
PCDUDOTS	CAE	CDUY		# COMPUTE CDUYDOT (USED BY PITCH AND YAW)
		XCH	PCDUYPST
		EXTEND
		MSU	PCDUYPST
		TCR	RLIMTEST	#	RATE TEST
		TS	MCDUYDOT	#	(MINUS, SC.AT 1/2TVCDT REVS/SEC)

		CAE	CDUZ		# COMPUTE CDUZDOT (USED BY PITCH AND YAW)
		XCH	PCDUZPST
		EXTEND
		MSU	PCDUZPST
		TCR	RLIMTEST	#	RATE TEST
		TS	MCDUZDOT	#	(MINUS, SC.AT 1/2TVCDT REVS/SEC)
		TCF	PINTEGRL

RLIMTEST	TS	TTMP1		# TEST FOR EXCESSIVE CDU RATES (GREATER
		EXTEND			#	THAN 2.33 DEG IN ONE SAMPLE PERIOD)
		MP	1/RTLIM
		EXTEND
		BZF	+3
		CAF	ZERO
		TS	TTMP1
		CAE	TTMP1
		TC	Q
;
; ============================================================================
; PINTEGRL - Pitch Attitude Error Integration (Integral Term of PID Control)
;
; The "I" (Integral) term of the PID controller accumulates attitude errors
; over time. If the spacecraft has been slowly drifting off the desired
; orientation for several seconds, this integral builds up and commands a
; stronger corrective gimbal deflection to eliminate the steady-state error.
;
; For code-along readers: This implements integral windup limiting. The error
; integrator PERRB accumulates the difference between commanded body-axis rate
; (OMEGAYC from cross-product steering) and actual rate (computed from CDU
; measurements). The integrator is limited by ERRORLIM to prevent excessive
; integral windup during large attitude maneuvers or rate-limited situations.
; ============================================================================
;
PINTEGRL	EXTEND			# COMPUTE INTEGRAL OF BODY-AXIS PITCH-RATE
		DCA	PERRB		#	ERROR, SC.AT B-1 REVS
		DXCH	ERRBTMP

		EXTEND
		DCA	OMEGAYC
		DAS	ERRBTMP

# Page 964
;
; Body-Axis Transformation - Converting Gimbal Rates to Body Rates
; The CDU measurements are in the gimbal coordinate frame, but the autopilot
; needs to know the spacecraft's rotation rate in its own body-axis frame.
; This trigonometric transformation accounts for the current gimbal angles
; to compute the true body-axis pitch rate OMEGAYB.
;
; For code-along readers: This uses direction cosines (COSCDUZ, COSCDUX, SINCDUX)
; and the measured CDU rates (MCDUYDOT, MCDUZDOT) to perform the coordinate
; transformation. The result is scaled at 1/2TVCDT revs per basic time unit.
;
		CS	COSCDUZ		# PREPARE BODY-AXIS PITCH RATE, OMEGAYB
		EXTEND
		MP	COSCDUX
		DDOUBL
		EXTEND
		MP	MCDUYDOT
		DDOUBL
		DXCH	OMEGAYB

		CS	MCDUZDOT
		EXTEND
		MP	SINCDUX
		DDOUBL
		DAS	OMEGAYB		# (COMPLETED OMEGAYB, SC.AT 1/2TVCDT REVS)
;
; Error Integration and Limiting
; Subtract the actual body rate from the commanded rate to get the error,
; accumulate this error over time, and limit the integrated error to prevent
; excessive integral windup. ERRORLIM constrains ERRBTMP to reasonable bounds.
;
		EXTEND			# PICK UP -OMEGAYB (SIGN CHNG, INTEGRATE)
		DCS	OMEGAYB
		DAS	ERRBTMP

PERORLIM	TCR	ERRORLIM	# PITCH BODY-AXIS-ERROR INPUT LIMITER

;
; ============================================================================
; Forward Filter Path - The Heart of the Control Law (6th-Order Cascade Filter)
;
; This section implements the main control algorithm that converts attitude
; errors into gimbal commands. The 6th-order filter provides smooth, stable
; control response while rejecting high-frequency noise and preventing
; oscillations. The filter adapts automatically between CSM-only and CSM/LM
; configurations, using different gain settings based on the vehicle's mass
; properties (computed by TVCMASSPROP).
;
; For code-along readers: FWDFLTR implements a cascade of three 2nd-order
; sections (or two sections for 4th-order mode). Each section has state
; variables stored in PTMPn. The variable gain package (OPTVARK) adjusts
; loop gains based on the 1/CONACC acceleration estimate. This sophisticated
; filter design was critical for Apollo's precision attitude control during
; critical maneuvers like translunar injection and lunar orbit insertion.
; ============================================================================
;
PFORWARD	EXTEND			# 	PREPARE THE FILTER STORAGE LOCATIONS
		DCA	PTMP1		#	FOR THE PITCH CHANNEL
		DXCH	TMP1
		EXTEND
		DCA	PTMP3
		DXCH	TMP3
		EXTEND
		DCA	PTMP5
		DXCH	TMP5

		TCR	FWDFLTR		# GO COMPUTE PRESENT OUTPUT
					# (INCLUDES VARIABLE GAIN PACKAGE)
;
; CG Offset Compensation
; As propellant burns and the spacecraft's mass decreases, the center of
; gravity shifts. PDELOFF compensates for this CG offset to maintain accurate
; thrust vector alignment through the vehicle's center of mass.
;
POFFSET		EXTEND
		DCA	PDELOFF
		DAS	CMDTMP		# NO SCALED AT B+0 ASCREV
;
; Actuator Command Limiting
; The physical gimbal actuators can only deflect the engine ±6 degrees. ACTLIM
; ensures the computed command stays within these physical hardware limits and
; rounds the command to the resolution of the actuator servos.
;
PACLIM		TCR	ACTLIM		# ROUND OFF & LIMIT PITCH ACTUATOR COMMAND

;
; Actuator Command Output
; The incremental gimbal command is computed and sent to the TVCPITCH error
; counter, which generates the analog voltage that drives the pitch actuator
; servo. The servo physically tilts the SPS engine to the commanded angle.
; During Apollo 11's translunar injection, lunar orbit insertion, and transearth
; injection burns, this output continuously adjusted engine pointing to maintain
; the precise trajectory that brought the crew safely to the Moon and back.
;
; For code-along readers: TVCPITCH is a hardware counter that's converted to
; analog voltage by the optics error counter circuits. BIT11 in CHAN14 releases
; the count to the CDU when proper moding is established. The command is an
; incremental update (difference from previous PCMD) to minimize quantization
; effects and support restart protection.
;
POUT		CS	PCMD		# INCREMENTAL PITCH COMMAND
		AD	CMDTMP
		ADS	TVCPITCH	# UPDATE THE ERROR COUNTER (NO RESTART-
					#	PROTECT. SINCE ERROR CNTR ZEROED)

		CAF	BIT11		# BIT FOR TVCPITCH COUNT RELEASE
		EXTEND
		WOR	CHAN14
;
; ============================================================================
; Precompensation - Updating Filter State for Next Cycle
;
; After computing the current output, the filter's internal state variables
; must be updated for the next DAP cycle. PRECOMP performs this "bookkeeping"
; for the cascade filter sections, storing the current values that will become
; the "previous values" in the next iteration.
; ============================================================================
;
PPRECOMP	EXTEND			#	PREPARE THE FILTER STORAGE FOR PITCH
# Page 965
		DCA	PTMP2
		DXCH	TMP2
		EXTEND
		DCA	PTMP4
		DXCH	TMP4
		EXTEND
		DCA	PTMP6
		DXCH	TMP6

		TCR	PRECOMP		#	TO THE FILTER FOR PRECOMPUTATION
;
; CG Offset Tracker Filter Update
; As the SPS engine burns propellant during maneuvers, the spacecraft's center
; of gravity shifts continuously. This first-order tracker filter (with time
; constant determined by E(-AT)) estimates the CG offset and feeds it back to
; the control loop. This adaptive compensation ensures accurate thrust vector
; control throughout the entire burn, from full tanks at translunar injection
; to nearly-empty tanks at lunar orbit insertion.
;
; For code-along readers: DELPBAR is the tracker state, E(-AT) is exp(-alpha*dt)
; where alpha is the tracker bandwidth, and 1-E(-AT) provides the complementary
; term. This exponentially-weighted filter smoothly tracks CG migration while
; rejecting high-frequency disturbances.
;
DELBARP		CAE	DELPBAR +1	# UPDATE PITCH OFFSET-TRACKER-FILTER
		EXTEND
		MP	E(-AT)
		TS	DELBRTMP +1
		CAE	DELPBAR
		EXTEND
		MP	E(-AT)
		DAS	DELBRTMP
		CAE	CMDTMP
		EXTEND
		MP	1-E(-AT)
		DAS	DELBRTMP
;
; Copycycle - Protecting Filter State for Restart
; The copycycle operation saves all computed values to their permanent storage
; locations with restart protection. If a restart occurs during the copycycle,
; TVCPHASE allows recovery without corrupting the DAP state.
;
PCOPYCYC	TCR	PCOPY		# PITCH COPYCYCLE
;
; Pitch DAP Cycle Complete
; The pitch autopilot cycle is complete. Control returns to the executive,
; which will schedule the next T5 task (YAWDAP) at the proper time.
;
PDAPEND		TCF	RESUME		# PITCH DAP COMPLETED
# Page 966
# PITCH TVCDAP COPYCYCLE SUBROUTINE (CALLED VIA PITCH TVCDAP OR TVC RESTART PACKAGE)
;
; ============================================================================
; PCOPY - Pitch Copycycle Subroutine with Restart Protection
;
; The copycycle transfers all temporary computation results to their permanent
; storage locations. This operation is restart-protected: TVCPHASE is
; incremented at the start and again at completion. If a restart occurs during
; the copycycle, the restart package can detect the incomplete state and
; properly recover the autopilot.
;
; This restart protection was critical during Apollo missions. If a transient
; electrical disturbance caused an AGC restart during a critical SPS burn,
; the TVC system had to resume seamlessly without introducing attitude errors
; that could compromise the trajectory.
;
; For code-along readers: The copycycle moves filter states (TMP1-TMP6 to
; PTMP1-PTMP6), error integrator state (ERRBTMP to PERRB), actuator command
; (CMDTMP to PCMD), and CG offset tracker state (DELBRTMP to DELPBAR). The
; TVCPHASE variable provides phase-marker restart protection as described in
; the TVCRESTARTS module.
; ============================================================================
;
PCOPY		INCR	TVCPHASE	# RESTART-PROTECT THE COPYCYCLE.	(1)
					#	NOTE POSSIBLE RE-ENTRY FROM RESTART
					#	PACKAGE, SHOULD A RESTART OCCUR
					#	DURING PITCH COPYCYCLE.
;
; Copy filter state variables from working storage to permanent storage.
; The six double-precision state variables (TMP1-TMP6) hold the internal
; states of the 6th-order cascade filter sections.
;
		EXTEND
		DCA	TMP1
		DXCH	PTMP1
		EXTEND
		DCA	TMP2
		DXCH	PTMP2
		EXTEND
		DCA	TMP3
		DXCH	PTMP3
		EXTEND
		DCA	TMP4
		DXCH	PTMP4
		EXTEND
		DCA	TMP5
		DXCH	PTMP5
		EXTEND
		DCA	TMP6
		DXCH	PTMP6
;
; Copy remaining DAP state variables: error integrator, actuator command,
; and CG offset tracker. AK1 receives a copy of the attitude error for
; potential display on the pitch attitude error needles.
;
PMISC		EXTEND			# MISC....PITCH-RATE-ERROR INTEGRATOR
		DCA	ERRBTMP
		TS	AK1		#	FOR PITCH NEEDLES, SC.AT B-1 REVS
		DXCH	PERRB

		CAE	CMDTMP		#	PITCH ACTUATOR COMMAND
		TS	PCMD

		EXTEND			# 	PITCH OFFSET-TRACKER-FILTER
		DCA	DELBRTMP
		DXCH	DELPBAR

		INCR	TVCPHASE	# PITCH COPYCYCLE COMPLETED		(2)

		TC	Q

# Page 967
# YAW TVCDAP STARTS HERE....(INCORPORATES CSM/LEM DAP FILTER, MODOR DESIGN)
;
; ============================================================================
; YAWDAP - YAW AXIS DIGITAL AUTOPILOT
;
; The yaw axis autopilot is structurally identical to PITCHDAP but operates
; on the spacecraft's yaw (Z) axis. Called alternately with PITCHDAP at half
; the DAP sample time. Generates gimbal commands to the SPS yaw actuator to
; correct attitude errors about the Z-axis during main engine burns.
;
; Like PITCHDAP, this routine:
; 1. Integrates body-axis rate error (commanded vs actual yaw rate)
; 2. Transforms gimbal rates to body-axis rates using CDU angles
; 3. Filters the error through the 6th-order cascade filter
; 4. Applies CG-offset tracking compensation
; 5. Limits and outputs the actuator command to TVCYAW error counter
;
; Called by PITCHDAP via T5 interrupt scheduling. Sets up reciprocal T5 call
; back to PITCHDAP to maintain alternating execution sequence.
; ============================================================================

YAWDAP		LXCH	BANKRUPT	# T5 ENTRY, NORMAL
		EXTEND
		QXCH	QRUPT
;
; T5 interrupt entry. Save interrupted bank and return address for restoration.
; Standard T5 task entry sequence mirrors PITCHDAP structure.
;
		CAF	PITCHT5		# SET UP T5 CALL FOR PITCH AUTOPILOT (LOW-
		TS	T5LOC		#	ORDER PART OF 2CADR ALREADY THERE)
		CAE	T5TVCDT
		TS	TIME5
;
; Set up T5 interrupt to call PITCHDAP on next cycle, maintaining the
; alternating pitch/yaw autopilot execution sequence at half DAP sample time.
; This creates the continuous pitch→yaw→pitch→yaw cycling during TVC operation.
;

YSTROKER	CCS	STROKER		# (STRKFLG) CHECK FOR STROKE TEST
		TC	HACK		# TEST-START OR TEST-IN-PROGRESS
		TCF	+2		# NO-TEST
		TC	HACK		# TEST-IN-PROGRESS
;
; Stroke test check for V68 waveform generation. If active, HACK routine
; overrides normal autopilot command with test pattern for actuator validation.
; Identical logic to PITCHDAP's PSTROKER.
;
					# USE BODY RATES FROM PITCHDAP (PCDUDOTS)
;
; ============================================================================
; YINTEGRL - YAW BODY-AXIS ATTITUDE ERROR INTEGRATION
;
; Integrates the yaw body-axis rate error over time. The error is the
; difference between the commanded yaw rate (OMEGAZC from cross-product
; steering) and the actual body yaw rate (OMEGAZB derived from gimbal rates).
;
; Body-axis yaw rate OMEGAZB computed from gimbal rates via transformation:
;   OMEGAZB = COSCDUZ*SINCDUX*MCDUYDOT - COSCDUX*MCDUZDOT
;
; Where MCDUYDOT and MCDUZDOT are the measured Y and Z gimbal angular rates.
; The transformation accounts for gimbal geometry to produce true spacecraft
; body-axis yaw rate about the Z-axis.
; ============================================================================

YINTEGRL	EXTEND			# COMPUTE INTEGRAL OF BODY-AXIS YAW-RATE
		DCA	YERRB		# 	ERROR, SC.AT B-1 REVS
		DXCH	ERRBTMP
;
; Load accumulated yaw attitude error from previous cycle.
; YERRB is the integrated history of yaw pointing error scaled at B-1 revs.
;
		EXTEND
		DCA	OMEGAZC
		DAS	ERRBTMP
;
; Add commanded yaw rate OMEGAZC from cross-product steering (S40.8).
; For impulsive burns without steering, OMEGAZC = 0 (attitude hold mode).
; Commanded rate represents desired change in yaw attitude per cycle.
;
		CAE	COSCDUZ		# PREPARE BODY-AXIS YAW-RATE, OMEGAZB
		EXTEND
		MP	SINCDUX
		DDOUBL
		EXTEND
		MP	MCDUYDOT
		DDOUBL
		DXCH	OMEGAZB
;
; First term of yaw rate transformation: COSCDUZ * SINCDUX * MCDUYDOT
; This represents the contribution of Y-gimbal motion to body Z-axis rate.
; Double precision multiplication with DDOUBL maintains computational accuracy.
; Gimbal angles from IMU CDUs updated by DAPINIT/YCOPY at start of cycle.
;
		CS	MCDUZDOT
		EXTEND
		MP	COSCDUX
		DDOUBL
		DAS	OMEGAZB		# (COMPLETED OMEGAZB, SC.AT 1/2TVCDT REVS)
;
; Second term: -COSCDUX * MCDUZDOT (complement sign for proper axis convention)
; Add to first term to complete OMEGAZB body-axis yaw rate calculation.
; Scaling: 1/2TVCDT revolutions (half the DAP sample time interval).
; Result is actual spacecraft yaw angular rate measured from gimbal motion.
;
		EXTEND			# PICK UP -OMEGAZB (SIGN CHNG, INTEGRATE)
		DCS	OMEGAZB
		DAS	ERRBTMP
;
; Subtract actual body yaw rate OMEGAZB from commanded rate to form rate error.
; Integration accumulates pointing error: if actual rate < commanded rate,
; error builds up indicating spacecraft is falling behind desired yaw attitude.
; Result in ERRBTMP is body-axis yaw attitude error ready for filter input.
;

YERORLIM	TCR	ERRORLIM	# YAW	  BODY-AXIS-ERROR INPUT LIMITER
;
; Apply attitude error limiter to prevent integrator windup. Limits magnitude
; of accumulated yaw error to prevent excessive control authority buildup when
; spacecraft cannot track commanded rates. Result stored in ERRBTMP for filter.
;

YFORWARD	EXTEND			# 	PREPARE THE FILTER STORAGE LOCATIONS
		DCA	YTMP1		#	FOR THE YAW CHANNEL
# Page 968
		DXCH	TMP1
		EXTEND
		DCA	YTMP3
		DXCH	TMP3
		EXTEND
		DCA	YTMP5
		DXCH	TMP5
;
; Load yaw channel filter state variables into common working storage TMP1/3/5.
; The 6th-order cascade filter maintains six state variables representing the
; filter's internal dynamics. These are the odd-numbered states (1, 3, 5)
; used by FWDFLTR to compute the current filter output (actuator command).
;
		TCR	FWDFLTR		# GO COMPUTE PRESENT OUTPUT
					# (INCLUDES VARIABLE GAIN PACKAGE)
;
; Call forward filter computation routine. FWDFLTR applies the generalized
; 6th-order filter to the attitude error, computing the filtered actuator
; command. Includes variable gain package (OPTVARK) which adjusts filter
; gains based on spacecraft configuration (CSM-only vs CSM/LM docked).
; Filter output returned in CMDTMP scaled at B+0 ASCREV (arc-seconds).
;

YOFFSET		EXTEND
		DCA	YDELOFF
		DAS	CMDTMP		# NOW SCALED AT B+0 ASCREV
;
; Add CG-offset compensation term to actuator command. YDELOFF accounts for
; displacement between spacecraft center-of-gravity and gimbal pivot point.
; When CG shifts (propellant usage, CSM/LM docking), gimbal command must be
; adjusted to produce desired body-axis torque about the actual CG location.
;

YACLIM		TCR	ACTLIM		# ROUND OFF & LIMIT YAW ACTUATOR COMMAND
;
; Apply output limiter to prevent gimbal actuator saturation. Limits command
; magnitude to physical actuator range. Rounds to appropriate resolution for
; error counter transmission to gimbal servo. Limited command in CMDTMP.
;

YOUT		CS	YCMD		# INCREMENTAL YAW COMMAND
		AD	CMDTMP
		ADS	TVCYAW		# UPDATE THE ERROR COUNTER (NO RESTART-
					#	PROTECT, SINCE ERROR CNTR ZEROED)
;
; Compute incremental yaw command as difference between new command (CMDTMP)
; and previous command (YCMD). Add increment to TVCYAW error counter which
; drives the SPS yaw gimbal actuator servo. Error counter transmits analog
; voltage to gimbal servo to position the engine for yaw attitude control.
; No restart protection needed as error counter zeroed on TVC initialization.
;
		CAF	BIT12		# BIT FOR TVCYAW COUNT RELEASE
		EXTEND
		WOR	CHAN14
;
; Release TVCYAW error counter data to CDU output channel. BIT12 enables
; transmission of error counter value to optics error counter hardware which
; converts digital count to analog voltage for gimbal actuator servo.
; Command reaches actuator when proper CDU moding established by TVCEXEC.
;

YPRECOMP	EXTEND			#	PREPARE THE FILTER STORAGE FOR YAW
		DCA	YTMP2
		DXCH	TMP2
		EXTEND
		DCA	YTMP4
		DXCH	TMP4
		EXTEND
		DCA	YTMP6
		DXCH	TMP6
;
; Load yaw channel even-numbered filter states (2, 4, 6) into common working
; storage. PRECOMP will compute updated state variables for next cycle based
; on current error and outputs, maintaining filter dynamics for next iteration.
;
		TCR	PRECOMP		#	TO THE FILTER FOR PRECOMPUTATION
;
; Call precomputation routine to update filter state variables for next cycle.
; Computes new values of TMP2/4/6 based on current inputs and outputs, storing
; results back into YTMP2/4/6 during copycycle. This prepares filter states
; for next autopilot execution maintaining continuous dynamic compensation.
;

DELBARY		CAE	DELYBAR +1	# UPDATE YAW OFFSET-TRACKER-FILTER
		EXTEND
		MP	E(-AT)
		TS	DELBRTMP +1
		CAE	DELYBAR
		EXTEND
		MP	E(-AT)
		DAS	DELBRTMP
		CAE	CMDTMP
		EXTEND
		MP	1-E(-AT)
		DAS	DELBRTMP
;
; Update CG-offset tracker filter for yaw axis. This first-order filter
; tracks CG displacement by comparing commanded actuator angle with measured
; attitude error. Filter uses exponential decay constant E(-AT) to smooth
; CG position estimate over time. Computation: DELBRTMP = E(-AT)*DELYBAR +
; (1-E(-AT))*CMDTMP. Result will be stored in DELYBAR during YCOPY copycycle.
; This adaptive tracking compensates for changing CG location as propellant
; is consumed and spacecraft configuration changes during the mission.
;

# Page 969
YCOPYCYC	TCR	YCOPY		# YAW COPYCYCLE
;
; Call yaw copycycle subroutine to transfer computed values from temporary
; working storage back into permanent yaw channel state variables. Updates
; YTMP1-6 with new filter states, DELYBAR with CG-offset tracker output,
; and YERRB with integrated attitude error. Copycycle is restart-protected
; to ensure consistency if restart occurs during state variable updates.
;

YDAPEND		TCF	RESUME		# YAW DAP COMPLETED
;
; Yaw autopilot cycle complete. Return via RESUME to restore interrupted
; program. T5 interrupt scheduled by this routine will trigger PITCHDAP on
; next cycle, maintaining continuous alternating pitch/yaw autopilot execution
; during SPS burn. This structure ensures both axes receive equal control
; attention at twice the effective sample rate compared to sequential execution.
;

# Page 970
# TVCDAP COPYCYCLE SUBROUTINE (CALLED VIA YAW   TVCDAP OR TVC RESTART PACKAGE)
;
; ============================================================================
; YCOPY - YAW CHANNEL COPYCYCLE SUBROUTINE
;
; Transfers computed yaw channel values from temporary working storage into
; permanent yaw state variables. Called at end of YAWDAP cycle and also by
; TVC restart package if restart occurs during copycycle execution.
;
; Restart Protection Strategy:
; TVCPHASE incremented at entry provides restart protection. If restart occurs
; during copycycle, restart package uses TVCPHASE=3 to re-enter this routine
; and complete the state variable updates, ensuring consistency of yaw channel
; control data despite interruption.
;
; Updated State Variables:
; - YTMP1-6: Six filter state variables for 6th-order cascade filter
; - YERRB: Integrated yaw body-axis attitude error
; - YCMD: Current yaw actuator command
; - DELYBAR: CG-offset tracker filter output
; ============================================================================

YCOPY		INCR	TVCPHASE	# RESTART-PROTECT THE COPYCYCLE.	(3)
					#	NOTE POSSIBLE RE-ENTRY FROM RESTART
					#	PACKAGE, SHOULD A RESTART OCCUR
					#	DURING YAW   COPYCYCLE.
;
; Increment TVCPHASE to 3 for restart detection. If restart occurs during
; following state updates, restart routine recognizes incomplete copycycle and
; re-enters here to complete transfer. Ensures yaw channel state consistency.
;
		EXTEND
		DCA	TMP1
		DXCH	YTMP1
		EXTEND
		DCA	TMP2
		DXCH	YTMP2
		EXTEND
		DCA	TMP3
		DXCH	YTMP3
		EXTEND
		DCA	TMP4
		DXCH	YTMP4
		EXTEND
		DCA	TMP5
		DXCH	YTMP5
		EXTEND
		DCA	TMP6
		DXCH	YTMP6
;
; Transfer all six filter state variables from temporary working storage
; (TMP1-6) to permanent yaw channel storage (YTMP1-6). These states represent
; the internal dynamics of the 6th-order cascade filter computed by FWDFLTR
; and PRECOMP. Filter maintains continuous dynamic compensation across cycles.
; Double-precision exchanges (DXCH) preserve full computational accuracy.
;

YMISC		EXTEND			# MISC....YAW-RATE-ERROR INTEGRATOR
		DCA	ERRBTMP
		TS	AK2		#	FOR YAW   NEEDLES, SC.AT B-1 REVS
		DXCH	YERRB
;
; Transfer integrated yaw attitude error from ERRBTMP to YERRB. This preserves
; accumulated error history for next autopilot cycle. AK2 receives upper word
; for display on yaw rate-error needles visible to crew, providing real-time
; attitude error indication. Scaled at B-1 revolutions for display formatting.
;
		CAE	CMDTMP
		TS	YCMD
;
; Store current yaw actuator command for next cycle. YCMD will be subtracted
; from next cycle's command to compute incremental update for error counter.
; Preserves commanded gimbal position across DAP cycles for smooth control.
;
		EXTEND
		DCA	DELBRTMP
		DXCH	DELYBAR
;
; Transfer CG-offset tracker filter output from temporary to permanent storage.
; DELYBAR maintains estimate of center-of-gravity displacement used to compute
; gimbal offset compensation. Filter adapts to changing CG as propellant burns
; and spacecraft configuration changes during mission.
;
		CAF	ZERO		# YAW	 COPYCYCLE COMPLETED
		TS	TVCPHASE	#	RESET TVCPHASE
;
; Reset TVCPHASE to zero indicating copycycle complete. All yaw state variables
; successfully updated and consistent. Restart protection no longer needed until
; next copycycle begins. TVCPHASE now ready for pitch copycycle or next yaw cycle.
;
		TC	Q
;
; Return to caller (YAWDAP or TVC restart package). Yaw channel state variables
; updated and ready for next autopilot cycle. Alternating pitch/yaw execution
; continues with PITCHDAP scheduled via T5 interrupt from YAWDAP.

# Page 971
# ============================================================================
; SUBROUTINES COMMON TO BOTH PITCH AND YAW DAPS
;
; The following subroutines are shared utility functions used by both the
; pitch and yaw autopilot loops. These functions handle initialization,
; input/output limiting, and filter computations that are identical for
; both control axes.
; ============================================================================


# ============================================================================
; DAPINIT - DAP INITIALIZATION PACKAGE FOR CDU RATE MEASUREMENTS
;
; PURPOSE:
; Initializes the digital autopilot by setting up the T5 timer for the first
; PITCHDAP call and capturing initial CDU (Coupling Data Unit) gimbal angles
; for rate derivation. This is the entry point called by TVCINIT4 during TVC
; system initialization.
;
; OPERATION:
; 1. Schedules the first PITCHDAP call via T5 timer interrupt
; 2. Reads current CDUY (yaw gimbal) and CDUZ (pitch gimbal) angles
; 3. Stores these as "past values" for the rate differentiator
; 4. The next DAP cycle will compute rates as (current - past) / delta-time
;
; CONTROL FLOW:
; Called by TVCINIT4 → Sets up T5 interrupt → Reads CDUs → Exits to NOQRSM
;
; MISSION CONTEXT:
; This initialization occurs at the start of every SPS burn (TLI, LOI, TEI).
; Capturing the initial gimbal angles is critical because the autopilot
; computes gimbal angular rates by differencing CDU readings between cycles.
; Without accurate initialization, the first autopilot cycle would compute
; incorrect rates from stale data.
; ============================================================================

DAPINIT		LXCH	BANKRUPT	# T5 RUPT ENTRY (CALLED BY TVCINT4)

		; Schedule the first PITCHDAP call using the T5 timer interrupt.
		; The timing calculation sets TIME5 to trigger PITCHDAP at the
		; next DAP cycle time (T5TVCDT defines half the DAP sample period).
		; This arithmetic computes: TIME5 = 2*T5TVCDT - 1 - POSMAX
		
		CAF	NEGONE		# 	SET UP
		AD	T5TVCDT		#	T5 CALL FOR PITCHDAP IN TVCDT SECS
		AD	NEGMAX		#	(T5TVCDT = POSMAX - TVCDT/2 +1)
		AD	T5TVCDT
		TS	TIME5		# Store computed time in T5 interrupt timer
		CAF	PITCHT5		#	(BBCON ALREADY THERE)
		TS	T5LOC		# Set T5 interrupt vector to PITCHDAP entry

		; Read the current CDU gimbal angles and store as "past values"
		; for the rate differentiator. The autopilot computes gimbal rates
		; as (current angle - past angle) / sample time. These initial
		; readings establish the baseline for the first rate computation.
		
		CAE	CDUY		# READ AND STORE CDUS FOR DIFFERENTIATOR
		TS	PCDUYPST	#	PAST-VALUES (pitch axis CDU yaw)
		CAE	CDUZ		# Read pitch gimbal angle
		TS	PCDUZPST	# Store as past value (pitch axis CDU pitch)

		TCF	NOQRSM		# Exit without restoring Q (interrupt entry)


# ============================================================================
; ERRORLIM - BODY-AXIS ATTITUDE ERROR INPUT LIMITER
;
; PURPOSE:
; Limits the integrated attitude error to prevent control system windup.
; When attitude errors become too large (spacecraft pointing significantly
; off target), this limiter prevents the integrator from accumulating
; excessively large values that could cause control instability or
; actuator saturation.
;
; OPERATION:
; 1. Checks if ERRBTMP exceeds the error limit (±ERRLIM)
; 2. If within limits, returns unchanged
; 3. If exceeding limits, clamps ERRBTMP to ±ERRLIM
;
; TECHNICAL DETAILS:
; - Uses multiplication by 1/ERRLIM to check for overflow
; - BZF (Branch if Zero) detects when error is within limits
; - CCS (Count, Compare, Skip) determines sign for bipolar limiting
; - Only the upper word of ERRBTMP is checked and limited
;
; CONTROL SYSTEM CONTEXT:
; This is a critical anti-windup protection. During large attitude errors
; (e.g., during engine ignition transients or spacecraft maneuvering),
; the attitude error integrator could accumulate to very large values.
; Without limiting, recovery from such errors would be slow and could
; cause oscillation or overshoot when the spacecraft finally returns to
; the desired attitude.
; ============================================================================

ERRORLIM	CAE	ERRBTMP		# CHECK FOR INPUT-ERROR LIMIT
		EXTEND			#	CHECKS UPPER WORD ONLY
		MP	1/ERRLIM	# Multiply by reciprocal to test for overflow
		; If (ERRBTMP * 1/ERRLIM) fits in register without overflow,
		; then ERRBTMP < ERRLIM and no limiting is needed.
		; If overflow occurs, the result will be non-zero in overflow bits.
		
		EXTEND
		BZF	+6		# Branch if zero (no overflow) - within limits
		
		; Error exceeds limits. Determine sign and apply appropriate limit.
		CCS	ERRBTMP		# Check sign of error
		CAF	ERRLIM		# Positive error → apply +ERRLIM
		TCF	+2		# Skip negative limit
		CS	ERRLIM		# Negative error → apply -ERRLIM
		TS	ERRBTMP		# LIMIT WRITES OVER UPPER WORD ONLY
		; The limited value replaces the original integrated error,
		; preventing windup while maintaining correct sign.

		TC	Q		# Return to caller


# ============================================================================
; ACTLIM - ACTUATOR COMMAND OUTPUT LIMITER
;
; PURPOSE:
; Limits the gimbal actuator commands to prevent exceeding the physical
; limits of the SPS engine gimbal actuators. The actuators have finite
; travel range, and commanding beyond this range would cause saturation,
; loss of control authority, and potential mechanical damage.
;
; OPERATION:
; 1. Rounds up the double-precision command value (CMDTMP, CMDTMP+1)
; 2. Checks if the command exceeds actuator saturation limit (±ACTSAT)
; 3. If within limits, returns unchanged
; 4. If exceeding limits, clamps CMDTMP to ±ACTSAT
;
; TECHNICAL DETAILS:
; - CMDTMP is double-precision (two words: CMDTMP and CMDTMP+1)
; - Lower word is doubled and added to upper word for rounding
; - Multiplication by 1/ACTSAT detects saturation condition
; - CCS determines sign for bipolar limiting
;
; MISSION CONTEXT:
; The SPS gimbal actuators have limited angular range (approximately
; ±6 degrees). During high-rate maneuvers or large attitude errors,
; the autopilot might compute commands exceeding this range. This limiter
; ensures commands stay within actuator capability while maintaining
; maximum available control authority. Physical actuator limits were a
; critical constraint during Apollo lunar orbit insertion and trans-earth
; injection burns.
; ============================================================================

ACTLIM		CAE	CMDTMP	+1	# ROUND UP FOR OUTPUT
		; Round the double-precision command value by adding the lower
		; word (shifted left one bit) to the upper word. This provides
		; 0.5 LSB rounding for the final single-precision output.
		
		DOUBLE			# Shift lower word left (multiply by 2)
		TS	L		# Store in L register
		CAF	ZERO		# Clear accumulator
		AD	CMDTMP		# Add upper word (with carry from L if any)
		; Accumulator now contains rounded single-precision command

		EXTEND			# CHECK FOR ACTUATOR COMMAND LIMIT
		MP	1/ACTSAT	# Multiply by reciprocal to test saturation
		; If command within ±ACTSAT, multiplication result will be zero
		; in the overflow detection logic.
		
		EXTEND
# Page 972
		BZF	+6		# Branch if within limits (no saturation)
		
		; Command exceeds actuator limits. Clamp to ±ACTSAT based on sign.
		CCS	CMDTMP		# APPLY LIMITS - check sign
		CAF	ACTSAT		# Positive command → limit to +ACTSAT
		TCF	+2		# Skip negative limit
		CS	ACTSAT		# Negative command → limit to -ACTSAT
		TS	CMDTMP		# LIMITS WRITE OVER CMDTMP
		; The actuator will receive a command at its maximum capability,
		; providing full available control authority without exceeding
		; mechanical or electrical limits.

		TC	Q		# Return to caller


# FILTER COMPUTATIONS FOR PRESENT OUTPUT................
# ============================================================================
; FWDFLTR - GENERALIZED 6TH-ORDER FORWARD-PATH FILTER WITH CG-OFFSET TRACKER
;
; PURPOSE:
; Implements a sophisticated 6th-order cascade filter in the TVC autopilot
; forward path to generate actuator commands from body attitude errors. The
; filter provides stable, well-damped response characteristics for both
; CSM-only and CSM/LM docked configurations, adapting its structure based
; on the LM docking state.
;
; FILTER ARCHITECTURE:
; The 6th-order filter is implemented as three cascaded 2nd-order sections:
; - First cascade (1DAPCAS): Processes initial attitude error signal
; - Second cascade (2DAPCAS): Further filters and shapes response
; - Third cascade (3DAPCAS): Final stage, ONLY active when LM is docked
;
; When LM is undocked (CSM-only), the filter operates as a 4th-order system
; (two cascades). When LM is docked (CSM/LM), all three cascades are active
; for the full 6th-order response needed to control the heavier, more complex
; configuration.
;
; CG-OFFSET TRACKER MINOR LOOP:
; Includes a center-of-gravity offset tracker that compensates for shifts
; in the spacecraft CG location as propellant is consumed. The tracker uses
; the previous actuator command as an initial condition and runs in a minor
; feedback loop to maintain accurate thrust vector alignment through the
; instantaneous CG.
;
; FILTER GAIN CALCULATIONS:
; Uses filter coefficients from the N10 parameter table (loaded by
; TVCEXECUTIVE based on current mass properties from TVCMASSPROP.agc):
; - Numerator coefficients (Nxy): Scale input signals
; - Denominator coefficients (Dxy): Define feedback paths and pole locations
; - Coefficients are dynamically updated as vehicle mass and inertia change
;
; INPUTS:
; - ERRBTMP, ERRBTMP+1: Body attitude error (from ERRORLIM, scaled B-1 revs)
; - N10 table: Filter coefficients for current vehicle configuration
; - DAP1, DAP2, DAP3: Previous filter state storage values
; - DAPDATR1 bit 13: LM docking configuration bit
;
; OUTPUTS:
; - CMDTMP: Filtered actuator command (scaled 1 ASCREV = 85.41 arcsec/bit)
; - DAP1, DAP2, DAP3: Updated filter state values for next cycle
; - DELBRTMP: Internal storage for cascade filter outputs
;
; MISSION CONTEXT:
; This filter is the heart of the TVC autopilot's control law. During Apollo 11:
; - TLI burn (translunar injection): Used 4th-order CSM-only filter
; - LOI burn (lunar orbit insertion): Used 4th-order CSM-only filter
; - TEI burn (trans-earth injection): Used 4th-order CSM-only filter (LM jettisoned)
;
; The filter coefficients were carefully tuned by MIT Instrumentation Laboratory
; engineers to provide stable control across the wide range of vehicle
; configurations and mass properties encountered during the mission.
; ============================================================================

; Initialize filter state storage and output accumulator to zero.
; These locations will be updated by each cascade filter section as it
; computes intermediate and final results. Zero initialization ensures
; clean startup conditions for the filter computation cycle.

FWDFLTR		CAF	ZERO
		TS	DAP1		; Clear first cascade state storage
		TS	DAP2		; Clear second cascade state storage
		TS	DAP3		; Clear third cascade state storage
		TS	CMDTMP		; Clear actuator command output accumulator

		TS	DELBRTMP	; Clear intermediate cascade output storage

; ============================================================================
; FIRST CASCADE (1DAPCAS) - INITIAL ATTITUDE ERROR PROCESSING
;
; Implements first 2nd-order filter section:
;   Transfer function: H1(s) = N10*s^2 + N11*s + N12
;                              -------------------------
;                              D10*s^2 + D11*s + D12
;
; The double-precision attitude error (ERRBTMP, ERRBTMP+1) is multiplied by
; the N10 coefficient to scale the input signal for the first cascade. This
; section provides initial shaping of the control response.
; ============================================================================

1DAPCAS		CAE	ERRBTMP +1	# FIRST DAP CASCADE
		EXTEND
		MP	N10		#	N10
		TS	DAP1	+1
		CA	ERRBTMP
		EXTEND
		MP	N10		#	N10
		DAS	DAP1		; Add result to DAP1 double-precision
		DXCH	TMP1		; Retrieve previously computed intermediate
		DAS	DAP1		; Complete first cascade computation

; ============================================================================
; SECOND CASCADE (2DAPCAS) - INTERMEDIATE FILTER STAGE
;
; Implements second 2nd-order filter section, using output from first cascade.
; Transfer function: H2(s) = N20*s^2 + N21*s + N22
;                            -------------------------
;                            D20*s^2 + D21*s + D22
;
; This cascade further shapes the control response, providing additional poles
; and zeros to achieve desired closed-loop stability and damping. The N20
; coefficient (stored at N10+5) scales the input from the first cascade.
; ============================================================================

2DAPCAS		CAE	DAP1	+1	# SECOND DAP CASCADE
		EXTEND
		MP	N10	+5	#	N20
		TS	DAP2	+1
		CA	DAP1
		EXTEND
		MP	N10	+5	#	N20
		DAS	DAP2
		DXCH	TMP3		; Retrieve previously computed intermediate
		DAS	DAP2		; Complete second cascade computation

; ============================================================================
; LM DOCKING CONFIGURATION TEST
;
; The filter order depends on whether the Lunar Module is docked to the
; Command/Service Module. Bit 14 of DAPDATR1 indicates docking status:
; - BIT14 = 1: LM docked (CSM/LM configuration) → use full 6th-order filter
; - BIT14 = 0: LM undocked (CSM-only) → use 4th-order filter (skip 3rd cascade)
;
; The heavier, more complex CSM/LM docked configuration requires the additional
; poles and zeros of the third cascade for adequate stability and performance.
; The lighter CSM-only configuration achieves good performance with just two
; cascades, and skipping the third cascade reduces computational load.
; ============================================================================

		CAE	DAPDATR1	# TEST FOR LEM ON OR OFF
		MASK	BIT14		; Isolate LM docking status bit
		CCS	A		; Test if bit is set
		TCF	3DAPCAS		# LEM ON - proceed to third cascade
		EXTEND			# LEM OFF - bypass third cascade
		DCA	DAP2		; Copy second cascade output directly
		DXCH	DAP3		; Store as final cascade result
		TCF	OPTVARK		; Skip to variable gain application

# Page 973
; ============================================================================
; THIRD CASCADE (3DAPCAS) - FINAL FILTER STAGE FOR CSM/LM CONFIGURATION
;
; Implements third 2nd-order filter section, ONLY when LM is docked.
; Transfer function: H3(s) = N30*s^2 + N31*s + N32
;                            -------------------------
;                            D30*s^2 + D31*s + D32
;
; This final cascade provides the additional dynamic compensation required to
; control the CSM/LM docked configuration, which has significantly different
; mass, inertia, and structural flexibility characteristics compared to the
; CSM alone. The N30 coefficient (stored at N10+10D) scales the input.
; ============================================================================

3DAPCAS		CAE	DAP2	+1	# THIRD DAP CASCADE
		EXTEND
		MP	N10	+10D	#	N30
		TS	DAP3	+1
		CA	DAP2
		EXTEND
		MP	N10	+10D	#	N30
		DAS	DAP3
		DXCH	TMP5
		DAS	DAP3		; Complete third cascade computation

; ============================================================================
; OPTVARK - VARIABLE GAIN APPLICATION AND SCALING
;
; Applies the time-varying VARK gain to the final filter output and performs
; final scaling to produce the actuator command. This gain is updated by
; TVCEXECUTIVE based on current vehicle mass properties (from TVCMASSPROP.agc)
; to maintain consistent closed-loop performance as propellant is consumed.
;
; VARIABLE GAIN CONCEPT:
; As the SPS burns and consumes propellant, the vehicle's mass decreases and
; its moment of inertia changes. To maintain consistent attitude control
; performance (same bandwidth, damping ratio, settling time), the autopilot
; gains must be adjusted. VARK provides this adaptation, with values computed
; from current mass and inertia by TVCEXECUTIVE.
;
; SIGN INVERSION:
; The complement (CS) operations here implement a sign change required by the
; forward-path loop structure. The physical actuator servos require a specific
; polarity to generate corrective torques in the proper direction.
;
; SCALING CHAIN:
; Input: DAP3 contains filter output (double-precision)
; Step 1: Multiply by VARK (scaled at 1/(8 ASCREV) of actual value)
;         Result scaled at B+3 ASCREVS
; Step 2: Double twice (DDOUBL, DDOUBL) to shift scaling
;         Result scaled at B+1 ASCREVS (1 ASCREV = 85.41 arcseconds/bit)
; Output: CMDTMP ready for actuator Digital-to-Analog Converter (DAC)
;
; ACTUATOR INTERFACE:
; The output CMDTMP drives the Optics Error Counters, which generate analog
; voltages for the SPS gimbal actuator servos. The DACs have an inherent gain
; of (B+1 ASCREVS), so the CMDTMP scaling must account for this.
; ============================================================================

OPTVARK		CS	DAP3	+1	# VARIABLE GAIN PACKAGE
		EXTEND			# (ALSO, SIGN CHANGE IN FORWARD LOOP)
		MP	VARK		; Apply variable gain (scaled 1/(8 ASCREV))
		TS	CMDTMP	+1	; Store lower word of result
		CS	DAP3		; Get upper word of filter output (negated)
		EXTEND
		MP	VARK		; Apply variable gain to upper word
		DAS	CMDTMP		; Combine with lower word result

		DXCH	CMDTMP		; Retrieve result (scaled B+3 ASCREVS)
		DDOUBL			; First doubling (now B+2 ASCREVS)
		DDOUBL			; Second doubling (now B+1 ASCREVS)
		DXCH	CMDTMP		; Store final actuator command
					# NOTE - THERE IS AN INHERANT GAIN OF
					# (B+1 ASCREVS) ON THE OUTPUT DACS.

		TC	Q		; Return to caller (PITCHDAP or YAWDAP)


# FILTER PRECOMPUTATIONS FOR NEXT PASS................

; ============================================================================
; PRECOMP - FILTER STATE PRECOMPUTATION FOR NEXT DAP CYCLE
;
; FUNCTIONAL DESCRIPTION:
; Computes the internal state variables (TMP1-TMP6) for the next iteration of
; the forward filter (FWDFLTR). This precomputation is performed at the end of
; the current DAP cycle so that the computed states are ready when the next
; cycle begins. The routine implements the difference equation computations
; using the filter coefficients (N10+0 through N10+14D) for each cascade.
;
; DIGITAL FILTER IMPLEMENTATION:
; Each cascade in the filter is implemented as a second-order difference
; equation of the form:
;    y(k) = N11*u(k) + N12*u(k-1) - D11*y(k-1) - D12*y(k-2)
; where u is input, y is output, and k is the discrete time index.
;
; The TMP variables store the intermediate terms for the next iteration:
;   TMP1, TMP2: First cascade state storage
;   TMP3, TMP4: Second cascade state storage  
;   TMP5, TMP6: Third cascade state storage (only if LM docked)
;
; COMPUTATIONAL STRUCTURE:
; For each cascade:
; 1. Multiply current input by numerator coefficient N11/2 (first-order term)
; 2. Multiply current output by denominator coefficient D11/2 (feedback term)
; 3. Combine results and double to get N11*input - D11*output
; 4. Store in first TMP location for cascade
; 5. Multiply previous input by numerator coefficient N12 (second-order term)
; 6. Multiply previous output by denominator coefficient D12 (feedback term)
; 7. Combine and store in second TMP location for cascade
;
; COEFFICIENT SCALING:
; The N11/2 and D11/2 coefficients are stored at half their actual values,
; requiring the DDOUBL operation to restore proper scaling. This technique
; prevents overflow in the multiply-accumulate operations.
;
; LM DOCKING CONFIGURATION:
; The third cascade precomputation is performed only when the LM is docked
; (BIT13 of DAPDATR1 clear), matching the 6th-order filter structure used
; during FWDFLTR execution. When LM is separated, only 4th-order filter
; states need precomputation.
;
; TIMING:
; PRECOMP executes during each DAP cycle, after FWDFLTR has computed the
; actuator command. The precomputed states sit in memory until the next
; T5TVCDT interrupt triggers the next DAP iteration.
; ============================================================================

PRECOMP		CAF	ZERO		# ***** FIRST CASCADE FILTER **********
		TS	TTMP1		; Initialize temporary accumulators
		TS	TTMP2		; for first cascade computation

; -----------------------------------------------------------------------
; FIRST CASCADE - FIRST-ORDER TERMS (N11*input - D11*output)
; -----------------------------------------------------------------------
		CA	ERRBTMP +1	# MULTIPLY INPUT BY
		EXTEND
		MP	N10	+1	#	N11/2
		TS	TTMP1	+1	; Store lower word of (N11/2)*input
		CA	ERRBTMP		; Get upper word of current input
		EXTEND
		MP	N10	+1	#	N11/2
		DAS	TTMP1		; TTMP1 now contains (N11/2)*input

		CS	DAP1	+1	# MULTIPLY OUTPUT BY
		EXTEND
		MP	N10	+3	# 	D11/2
		TS	TTMP2	+1	; Store lower word of (D11/2)*output
		CS	DAP1		; Get upper word of current output (negated)

# Page 974
		EXTEND
		MP	N10	+3	#	D11/2
		DAS	TTMP2		; TTMP2 now contains -(D11/2)*output

		DXCH	TTMP2		; Retrieve -(D11/2)*output
		DAS	TTMP1		; Combine: (N11/2)*input - (D11/2)*output
		DXCH	TTMP1		; Retrieve combined result
		DDOUBL			; Double to account for /2 scaling in coeffs
		DAS	TMP2		; Store as first-order term for next cycle

		DXCH	TMP2		; Move first-order term to TMP1
		DXCH	TMP1		; TMP1 ready for next FWDFLTR iteration

; -----------------------------------------------------------------------
; FIRST CASCADE - SECOND-ORDER TERMS (N12*prev_input - D12*prev_output)
; -----------------------------------------------------------------------
		CAF	ZERO
		TS	TTMP1		; Clear temporary accumulators
		TS	TMP2		; for second-order term computation

		CA	ERRBTMP	+1	# MULTIPLY INPUT BY
		EXTEND			# SECOND-ORDER NUMERATOR COEFF.
		MP	N10	+2	#	N12
		TS	TTMP1	+1	; Store lower word of N12*input
		CA	ERRBTMP		; Get upper word of current input
		EXTEND
		MP	N10	+2	# 	N12
		DAS	TTMP1		; TTMP1 now contains N12*input

		CS	DAP1	+1	# MULTIPLY OUTPUT BY
		EXTEND
		MP	N10	+4	# 	D12
		TS	TMP2	+1	; Store lower word of D12*output
		CS	DAP1		; Get upper word of current output (negated)
		EXTEND
		MP	N10	+4	#	D12
		DAS	TMP2		; TMP2 now contains -D12*output

		DXCH	TTMP1		; Retrieve N12*input
		DAS	TMP2		; Combine: N12*input - D12*output
				; TMP2 ready for next FWDFLTR iteration

; ============================================================================
; SECOND CASCADE FILTER PRECOMPUTATION
; ============================================================================
2CASFLTR	CAF	ZERO		# *****SECOND CASCADE FILTER*****
		TS	TTMP1		; Initialize temporary accumulators
		TS	TTMP2		; for second cascade computation

; -----------------------------------------------------------------------
; SECOND CASCADE - FIRST-ORDER TERMS (N21*input - D21*output)
; Input is DAP1 (output from first cascade), output is DAP2
; -----------------------------------------------------------------------
		CA	DAP1	 +1	# MULTIPLY INPUT BY
		EXTEND
		MP	N10	+6	#	N21/2
		TS	TTMP1	+1	; Store lower word of (N21/2)*input
		CA	DAP1		; Get upper word (output from first cascade)
		EXTEND
		MP	N10	+6	#	N21/2
# Page 975
		DAS	TTMP1		; TTMP1 now contains (N21/2)*input

		CS	DAP2	+1	# MULTIPLY OUTPUT BY
		EXTEND
		MP	N10	+8D	# 	D21/2
		TS	TTMP2	+1	; Store lower word of (D21/2)*output
		CS	DAP2		; Get upper word of second cascade output (negated)
		EXTEND
		MP	N10	+8D	#	D21/2
		DAS	TTMP2		; TTMP2 now contains -(D21/2)*output

		DXCH	TTMP2		; Retrieve -(D21/2)*output
		DAS	TTMP1		; Combine: (N21/2)*input - (D21/2)*output
		DXCH	TTMP1		; Retrieve combined result
		DDOUBL			; Double to account for /2 scaling
		DAS	TMP4		; Store as first-order term for next cycle

		DXCH	TMP4		; Move first-order term to TMP3
		DXCH	TMP3		; TMP3 ready for next FWDFLTR iteration

; -----------------------------------------------------------------------
; SECOND CASCADE - SECOND-ORDER TERMS (N22*prev_input - D22*prev_output)
; -----------------------------------------------------------------------
		CAF	ZERO
		TS	TTMP1		; Clear temporary accumulators
		TS	TMP4		; for second-order term computation

		CA	DAP1	+1	# MULTIPLY INPUT BY
		EXTEND
		MP	N10	+7	#	N22
		TS	TTMP1	+1	; Store lower word of N22*input
		CA	DAP1		; Get upper word of first cascade output
		EXTEND
		MP	N10	+7	# 	N22
		DAS	TTMP1		; TTMP1 now contains N22*input

		CS	DAP2	+1	# MULTIPLY OUTPUT BY
		EXTEND
		MP	N10	+9D	# 	D22
		TS	TMP4	+1	; Store lower word of D22*output
		CS	DAP2		; Get upper word of second cascade output (negated)
		EXTEND
		MP	N10	+9D	#	D22
		DAS	TMP4		; TMP4 now contains -D22*output

		DXCH	TTMP1		; Retrieve N22*input
		DAS	TMP4		; Combine: N22*input - D22*output
				; TMP4 ready for next FWDFLTR iteration

; -----------------------------------------------------------------------
; CONFIGURATION TEST: LEM ATTACHED?
; If the Lunar Module is NOT docked (LEM OFF), the CSM uses only two
; cascades. Third cascade is only activated for CSM/LM docked configuration
; due to different mass properties and control characteristics.
; -----------------------------------------------------------------------
		CAE	DAPDATR1	# TEST FOR LEM ON OR OFF
		MASK	BIT13		; Extract LEM ON/OFF configuration bit
		CCS	A		; Test if LEM is docked
		TC	Q		# EXIT IF LEM OFF (skip third cascade)

# Page 976
; ============================================================================
; THIRD CASCADE FILTER PRECOMPUTATION
; This cascade is ONLY used when the Lunar Module is docked to the CSM.
; The added mass and different inertia properties require an additional
; filter stage to maintain stability and performance during SPS burns.
; ============================================================================
3CASFLTR	CAF	ZERO		# *****THIRD CASCADE FILTER*****
		TS	TTMP1		; Initialize temporary accumulators
		TS	TTMP2		; for third cascade computation

; -----------------------------------------------------------------------
; THIRD CASCADE - FIRST-ORDER TERMS (N31*input - D31*output)
; Input is DAP2 (output from second cascade), output is DAP3
; -----------------------------------------------------------------------
		CA	DAP2	 +1	# MULTIPLY INPUT BY (1/2)
		EXTEND
		MP	N10	+11D	#	N31/2
		TS	TTMP1	+1	; Store lower word of (N31/2)*input
		CA	DAP2		; Get upper word (output from second cascade)
		EXTEND
		MP	N10	+11D	#	N31/2
		DAS	TTMP1		; TTMP1 now contains (N31/2)*input

		CS	DAP3	+1	; Get lower word of third cascade output (negated)
		EXTEND
		MP	N10	+13D	# 	D31/2
		TS	TTMP2	+1	; Store lower word of (D31/2)*output
		CS	DAP3		; Get upper word of third cascade output (negated)
		EXTEND
		MP	N10	+13D	#	D31/2
		DAS	TTMP2		; TTMP2 now contains -(D31/2)*output

		DXCH	TTMP2		; Retrieve -(D31/2)*output
		DAS	TTMP1		; Combine: (N31/2)*input - (D31/2)*output
		DXCH	TTMP1		; Retrieve combined result
		DDOUBL			; Double to account for /2 scaling
		DAS	TMP6		; Store as first-order term for next cycle

		DXCH	TMP6		; Move first-order term to TMP5
		DXCH	TMP5		; TMP5 ready for next FWDFLTR iteration

; -----------------------------------------------------------------------
; THIRD CASCADE - SECOND-ORDER TERMS (N32*prev_input - D32*prev_output)
; -----------------------------------------------------------------------
		CAF	ZERO
		TS	TTMP1		; Clear temporary accumulators
		TS	TMP6		; for second-order term computation

		CA	DAP2	+1	# MULTIPLY INPUT BY
		EXTEND
		MP	N10	+12D	#	N32
		TS	TTMP1	+1	; Store lower word of N32*input
		CA	DAP2		; Get upper word of second cascade output
		EXTEND
		MP	N10	+12D	# 	N32
		DAS	TTMP1		; TTMP1 now contains N32*input

		CS	DAP3	+1	; Get lower word of third cascade output (negated)
		EXTEND
		MP	N10	+14D	# 	D32
		TS	TMP6	+1	; Store lower word of D32*output
		CS	DAP3		; Get upper word of third cascade output (negated)
		EXTEND
# Page 977
		MP	N10	+14D	#	D32
		DAS	TMP6		; TMP6 now contains -D32*output

		DXCH	TTMP1		; Retrieve N32*input
		DAS	TMP6		; Combine: N32*input - D32*output
				; TMP6 ready for next FWDFLTR iteration

		TC	Q		; Return to caller (FWDFLTR)

# Page 978
; ============================================================================
; AUTOPILOT CONSTANTS AND SCALING FACTORS
; These constants define operational limits, scaling conversions, and timing
; parameters for the TVC digital autopilot. During Apollo 11's critical burns
; (TLI, LOI, TEI), these constants ensured the SPS engine gimbal commands
; stayed within safe actuator limits while maintaining precise attitude control.
; ============================================================================

# CONSTANTS FOR AUTOPILOTS

# NOTE....1 ASCREV (ACTUATOR CMD SCALING) = 85.41 ARCSEC/BIT OR 1.07975111 REVS (85.41x16384/3600/360)

#	  1 SPASCREV (SPECIAL ACTUATOR CMD SCALING) = 1.04620942 REVS

; -----------------------------------------------------------------------
; ACTUATOR SATURATION LIMITS
; The SPS engine gimbals have physical travel limits. ACTSAT defines the
; maximum gimbal deflection (6 degrees) in scaled units. Exceeding this
; limit would demand impossible gimbal angles from the actuator servos.
; -----------------------------------------------------------------------
ACTSAT		DEC	253		# ACTUATOR LIMIT (6 DEG), SC.AT 1ASCREV
1/ACTSAT	DEC	.0039525692	# RECIPROCAL (1/253)
				; Used for inverse scaling in control computations

; -----------------------------------------------------------------------
; ATTITUDE ERROR LIMITS
; ERRLIM caps the integrated attitude error to prevent wind-up in the
; control loop. At 45 degrees (B-3 revs), the limit prevents excessive
; gimbal commands from building up during large attitude errors.
; -----------------------------------------------------------------------
ERRLIM		EQUALS	BIT13		# FILTER INPUT LIMIT....B-3 REVS (45DEG),
1/ERRLIM	EQUALS	BIT3		# 	SC.AT B-1 REV, AND ITS RECIPROCAL

; -----------------------------------------------------------------------
; T5 TASK ADDRESSES
; PITCHT5, DAPT5, and YAWT5 define the sequence for cyclic autopilot tasks.
; The pitch and yaw DAPs call each other alternately at half the sample rate,
; creating interleaved control updates for smooth gimbal commands during burns.
; -----------------------------------------------------------------------
PITCHT5		GENADR	PITCHDAP	# UPPER WORDS OF T5 2CADRS, LOWER WORDS
DAPT5		GENADR	DAPINIT		#	(BBCON) ALREADY THERE.  ORDER IS
YAWT5		GENADR	YAWDAP		#	REQUIRED.

; -----------------------------------------------------------------------
; GIMBAL RATE AND FILTER TIME CONSTANTS
; 1/RTLIM: Gimbal rate limiting factor (2.33 degrees threshold)
; 1-E(-AT) and E(-AT): First-order filter time constants
;   For either T=40ms (1/A=4sec) or T=80ms (1/A=8sec) sample rates
; -----------------------------------------------------------------------
1/RTLIM		DEC	0.004715	# .004715(CDUDIF) = 0 IF CDUIF < 2.33 DEG
1-E(-AT)	OCT	00243		# AT = .01SEC....EITHER(1/A=4SEC, T=40MS),
E(-AT)		OCT	37535		#		     OR(1/A=8SEC, T=80MS)
				; Exponential filter coefficients for rate damping
