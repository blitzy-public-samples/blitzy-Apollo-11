# Copyright:	Public domain.
# Filename:	RCS-CSM_DIGITAL_AUTOPILOT.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1002-1024
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
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

; ============================================================================
; FILE: RCS-CSM_DIGITAL_AUTOPILOT.agc
; MODULE: TVCDAPS Subsystem (Thrust Vector Control and Digital Autopilot)
; MISSION PHASE: all-phases
;
; TL;DR: Reaction Control System digital autopilot for Command/Service Module
;        implementing thruster firing logic for three-axis attitude control.
;        Executes minimum impulse mode, attitude deadband control, and propellant
;        conservation strategies throughout all Apollo 11 mission phases from
;        Earth orbit through translunar coast, lunar orbit, and transearth return.
;
; COMMENT-ONLY READERS: This autopilot controlled the small steering jets (RCS
;        thrusters) that kept Columbia pointed in the right direction throughout
;        the mission. It managed attitude control during coasting flight, orbital
;        maneuvers, and when the crew manually commanded spacecraft rotations.
;
; CODE-ALONG READERS: Study RCS thruster firing logic using minimum impulse
;        control, attitude deadband implementation, Kalman-like rate filtering
;        for angular velocity estimation, manual rotation command processing from
;        the Rotational Hand Controller (RHC), FDAI error needle display logic,
;        and propellant management strategies for three-axis attitude hold.
; ============================================================================

; ============================================================================
; TRANSITION: RCS Digital Autopilot Architecture Overview
;
; The RCS-CSM Digital Autopilot (DAP) maintains spacecraft attitude using
; 12 reaction control system thrusters arranged in four quads around the
; Service Module. During Apollo 11, this autopilot kept Columbia stable during:
; - Translunar and transearth coast (maintaining precise attitude for navigation)
; - Lunar orbit operations (enabling Mike Collins to track landmarks)
; - Manual rotations for photography and rendezvous alignment
; - SPS engine burn preparation (precise attitude alignment before ignition)
;
; The autopilot executes in three phases spread across multiple 100ms T5 timer
; interrupts to prevent interrupt starvation. Phase 1 computes attitude rates
; using a Kalman-like filter. Phase 2 updates the filter state. Phase 3 (in
; JET_SELECTION_LOGIC.agc) selects which thrusters to fire based on minimum
; impulse control law and deadband logic.
;
; Technical implementation: Uses 100ms sample period, three-axis attitude
; control with cross-coupling compensation, minimum impulse bit logic for
; fuel conservation, and rate-plus-attitude feedback control law.
; ============================================================================

# Page 1002
# T5 INTERRUPT PROGRAM FOR THE RCS-CSM AUTOPILOT
# 	START OF T5 INTERRUPT PROGRAM

; ============================================================================
; RCS DIGITAL AUTOPILOT - T5 INTERRUPT HANDLER
;
; The RCS (Reaction Control System) autopilot maintains spacecraft attitude
; using 16 small thruster jets arranged in four quads around the Service Module.
; This interrupt-driven program executes every 100 milliseconds (configurable
; to 20ms, 60ms, or 80ms intervals) to:
;   - Read spacecraft attitude from the IMU (Inertial Measurement Unit)
;   - Calculate attitude errors relative to desired orientation
;   - Determine which RCS jets to fire to correct errors
;   - Update crew displays showing spacecraft orientation
;
; The autopilot operates in several modes:
;   - ATTITUDE HOLD: Maintains current spacecraft orientation
;   - AUTOMATIC MANEUVER: Executes programmed attitude changes
;   - MANUAL ROTATION: Responds to crew Rotational Hand Controller (RHC)
;   - FREE DRIFT: No active control when IMU is off or in FREE mode
;
; Throughout Apollo 11's mission, this autopilot maintained Columbia's
; orientation during coast phases, orbital operations, and rendezvous.
; ============================================================================

		BANK	20
		SETLOC	DAPS3
		BANK

		COUNT	21/DAPRC

		EBANK=	KMPAC
; REDORCS - RESTART ENTRY POINT
; If autopilot execution is interrupted by a restart, execution resumes here.
; The T5PHASE variable tracks which phase of the autopilot is executing.
REDORCS		LXCH	BANKRUPT	# RESTART OF AUTOPILOT COMES HERE
		CA	T5PHASE		# ON A T5 RUPT.
		EXTEND
		BZMF	+2		# IF T5PHASE +0, -0, OR -, RESET TO -
		TCF	+3		# IF T5PHASE +, LEAVE IT +.  DO A FRESHDAP
		CS	ONE
		TS	T5PHASE
		EXTEND
		DCA	RCSLOC
		DXCH	T5LOC		# HOOK UP T5RUPT TO AUTOPILOT
		TCF	RCSATT +1
		EBANK=	KMPAC
RCSLOC		2CADR	RCSATT

; ============================================================================
; RCSATT - MAIN AUTOPILOT ENTRY POINT (T5 INTERRUPT)
;
; Every 100 milliseconds, the TIME5 counter triggers a T5 interrupt that
; invokes RCSATT. This is the heart of the RCS autopilot control loop.
;
; COMMENT-ONLY READERS: Think of this as the autopilot "waking up" 10 times
; per second to check if the spacecraft is pointing the right way, and if not,
; firing small jets to correct the orientation. The crew could also take manual
; control using the Rotational Hand Controller (like a joystick).
;
; CODE-ALONG READERS: The interrupt handler first checks IMU status via CHAN31
; BIT15 to determine if autonomous control is enabled. The S/C CONT (Spacecraft
; Control) switch position determines control authority:
;   - CMC position: Computer has full control (BIT15 = 0)
;   - S/C position: Crew has manual control (BIT15 = 1)
; If IMU is off or manual control is active, the autopilot zeros attitude
; errors and sets HOLDFLAG positive to indicate no automatic corrections.
; ============================================================================

RCSATT		LXCH	BANKRUPT	# SAVE BB
		EXTEND			# SAVE Q
		QXCH	QRUPT
; Check IMU and control mode status from CHAN31.
; BIT15 = 0 means IMU power is on and spacecraft control switch is in CMC mode.
; BIT15 = 1 means manual control or IMU is off.
		CAF	BIT15		# BIT15 CHAN31 = 0 IF IMU POWER IS ON AND
		EXTEND			# S/C CONT SW IS IN CMC (I.E. IF G/C AUTO
		RAND	CHAN31		# PILOT IS FULLY ENABLED)
		EXTEND
		BZF	SETT5		# IF G/C AUTOPILOT IS FULLY ENABLED,
					# GO TO SETT5

; IMU is off or manual control is active. Set NORATE flag and zero errors.
; NORATE (BIT14 of RCSFLAGS) indicates rate damping is disabled.
		CS	RCSFLAGS	# IF G/C AUTOPILOT IS NOT FULLY ENABLED,
		MASK	BIT14
		ADS	RCSFLAGS	# SET NORATE FLAG,
		CAF	POSMAX
		TS	HOLDFLAG	# SET HOLDFLAG +,
		CAF	ZERO		# ZERO ERRORX, ERRORY, AND ERRORZ,
		TS	ERRORX
		TS	ERRORY
		TS	ERRORZ
; Check if spacecraft is in FREE drift mode (no active control).
; BIT14 CHAN31 = 1 means FREE mode is active.
		CAF	BIT14
		EXTEND
		RAND	CHAN31		# AND CHECK FREE FUNCTION (BIT14 CHAN31).
		EXTEND
# Page 1003
		BZF	SETT5		# IF IN FREE MODE, GO TO SETT5.

; If not in FREE mode and autopilot is disabled, schedule reinitialization.
; The autopilot will restart with a fresh configuration after 100ms.
		TS	T5PHASE		# IF NOT IN FREE MODE,
		CAF	OCT37766	# SCHEDULE REINITIALIZATION (FRESHDAP)
		TS	TIME5		# IN 100 MS VIA T5RUPT

; ZEROJET turns off all RCS thruster commands within 14 milliseconds.
; This ensures no jets are firing when the autopilot is not in control.
		TCR	ZEROJET		# ZERO JET CHANNELS IN 14 MS VIA ZEROJET

		TCF	KMATRIX
; TIMING CONSTANTS for T5 interrupt intervals (TIME5 counter values)
DELTATT		OCT	37770		# 80MS (TIME5)
DELTATT2	OCT	37776		# 20MS (TIME5)
ONESEK		DEC	16284		# 1 SEC(TIME5)
CHAN5		EQUALS	5
CHAN6		EQUALS	6
PRIO34A		=	PRIO34

; ============================================================================
; T5 PHASED EXECUTION STRATEGY
;
; The T5 autopilot program is computationally intensive. To prevent blocking
; other critical interrupts for too long, it's divided into three phases that
; execute sequentially across multiple T5 interrupts:
;
;   T5PHASE = 1: Read IMU, calculate attitude errors, perform rate filtering
;   T5PHASE = 2: Apply control laws, compute desired jet torques
;   T5PHASE = 3: Execute jet selection logic to determine which jets fire
;
; After phase 3 completes, T5PHASE resets to 1 and the cycle repeats.
; This phased approach allows higher-priority interrupts (T3RUPT, DSRUPT) to
; execute between autopilot phases, maintaining system responsiveness.
;
; CODE-ALONG READERS: T5PHASE is checked at the start of each interrupt.
; The variable also serves as a flag during initialization (FRESHDAP) where
; T5PHASE is set positive to indicate a fresh start is needed.
; ============================================================================

# 		CHECK PHASE OF T5 PROGRAM

# 	BECAUSE OF THE LENGTH OF THE T5 PROGRAM, IT HAS BEEN DIVIDED INTO
# THREE PARTS, T5PHASE1, T5PHASE2, AND THE JET SELECTION LOGIC,
# TO ALLOW FOR THE EXECUTION OF OTHER
# INTERRUPTS.  T5PHASE IS ALSO USED IN THE INITIALIZATION OF THE AUTOPILOT
# VARIABLES AT TURN ON.
# THE CODING OF T5PHASE IS...

#		+ = INITIALIZE T5 RCS-CSM AUTOPILOT
#    T5PHASE = +0 = PHASE2 OF THE T5 PROGRAM
#		- = RESTART DAP
#	       -0 = PHASE1 OF THE T5 PROGRAM

; SETT5 - T5 PHASE DISPATCHER
; Uses CCS (Count, Compare, and Skip) instruction to check T5PHASE sign and value:
;   Positive: Autopilot needs initialization -> FRESHDAP
;   +0: Phase 2 is ready to execute -> T5PHASE2  
;   Negative: Restart in progress -> REDAP
;   -0: Phase 1 is ready to execute -> continue below
SETT5		CCS	T5PHASE
		TCF	FRESHDAP	# TURN ON AUTOPILOT
		TCF	T5PHASE2	# BRANCH TO PHASE2 OF PROGRAM
		TCF	REDAP		# RESTART AUTOPILOT

; T5 PHASE 1 - ATTITUDE AND RATE MEASUREMENT
; This phase reads IMU gimbal angles and computes spacecraft angular rates.
; Execution time budget: ~20ms before Phase 2 interrupt.
		TS	T5PHASE		# PHASE 1 RESET FOR PHASE 2
		CA	TIME5
		TS	T5TIME		# USED IN COMPENSATING FOR DELAYS IN T5
		CAF	DELTATT2	# RESET FOR T5RUPT IN 20MS FOR PHASE2
		TS	TIME5		# OF PROGRAM

# Page 1004
# IMU STATUS CHECK

; ============================================================================
; IMU STATUS CHECK AND RATE FILTER INITIALIZATION
;
; COMMENT-ONLY READERS: Before calculating how to steer the spacecraft, the
; autopilot checks if the IMU (the gyroscopic instrument that measures which
; way the spacecraft is pointed) is working properly. If the IMU is off or
; failed, the autopilot switches to a safe mode where it stops trying to
; automatically control attitude and waits for the IMU to come back online.
;
; CODE-ALONG READERS: IMODES33 BIT6 indicates IMU status (0=OK, 1=failed).
; If IMU is unavailable, the NORATE flag (BIT14 of RCSFLAGS) is set to
; disable rate computations, and HOLDFLAG is set to stop automatic maneuvers.
; The rate filter must be initialized before it can provide useful rate
; estimates - the NORATE flag tracks initialization status.
; ============================================================================

		CS	IMODES33	# CHECK IMU STATUS
		MASK	BIT6		# BIT6 = 0  IMU OK
		CCS	A		# BIT6 = 1 NO IMU
		TCF	RATEFILT
; IMU is not available. Set flags for safe mode.
FREECHK		CS	RCSFLAGS	# BIT14 INDICATES THAT RATES HAVE NOT BEEN
		MASK	BIT14		# INITIALIZED
		ADS	RCSFLAGS
		CAF	BIT14		# NO ATTITUDE REFERENCE
		TS	HOLDFLAG	# STOP ANY AUTOMATIC STEERING AND PREPARE
					# TO PICK UP CDU ANGLES UPON RESUMPTION OF
					# ATTITUDE HOLD
		EXTEND
		RAND	CHAN31		# CHECK FOR FREE MODE
		EXTEND
		BZF	KRESUME1	# IN FREE MODE PROVIDE FREE CONTROL ONLY
		TCF	REINIT		# .....TILT...............................
BITS4,5		OCT	30

; Check if rate filter has been initialized (NORATE flag clear).
; If not initialized, skip rate derivation this cycle.
RATEFILT	CA	RCSFLAGS	# SEE IF RATEFILTER HAS BEEN INITIALIZED
		MASK	BIT14
		EXTEND			# IF SO, PROCEED WITH RATE DERIVATION
		BZF	+2
		TCF	KMATRIX		# IF NOT, SKIP RATE DERIVATION

; ============================================================================
; RATE FILTER - SPACECRAFT ANGULAR RATE ESTIMATION
;
; The rate filter estimates spacecraft angular rates (how fast the spacecraft
; is rotating in pitch, yaw, and roll) by examining changes in IMU gimbal
; angles (CDU values) between successive T5 interrupts.
;
; COMMENT-ONLY READERS: To smoothly control the spacecraft's orientation, the
; autopilot needs to know not just which way it's pointed, but how fast it's
; rotating. The rate filter computes rotation speed by comparing the current
; pointing direction to where it was 100 milliseconds ago, then smooths out
; measurement noise using a mathematical filter (Kalman filter).
;
; CODE-ALONG READERS: This is a discrete-time Kalman filter implementation.
; State variables DRHO (rate residual) and ADOT (filtered rate) are updated
; each cycle. GAIN1 and GAIN2 are Kalman gains selected based on LEM docking
; status (different gains for docked vs undocked configurations due to changed
; spacecraft inertia). Execution time: ~7.72ms.
;
; Filter equations (see comments below for mathematical notation):
;   DRHO = DELRHO - 0.1*ADOT + (1-GAIN1)*DRHO_prev
;   ADOT = ADOT_prev + GAIN2*DRHO + KMJ*DFT
; Where DELRHO is the measured angle change transformed to body axes.
; ============================================================================

# 	RATE FILTER	TIMING = 7.72 MS

# RATE FILTER EQUATIONS
# DRHO = DELRHO - (.1)ADOT + (1 = GAIN1)DRHO
#					    -1
# ADOT = ADOT   + GAIN2 DRHO + KMJ DFT
#	     -1
#	-        *     -     -
# WHERE DELRHO = AMGB (CDU - CDU  )
#			 	-1

; ============================================================================
; TRANSITION: From attitude error integration to rate filter computation
;
; The autopilot has now computed attitude errors in all three axes. The next
; critical task is estimating angular velocity (body rates) from IMU gimbal
; angle changes. Since the AGC lacks direct rate gyros, this Kalman-like
; filter estimates rates by differencing gimbal angles over time, filtering
; out noise and acceleration effects. Accurate rate estimates are essential
; for damping spacecraft rotations and preventing oscillations during all
; mission phases from Earth orbit through translunar coast to lunar orbit.
; ============================================================================

; RATE FILTER COMPUTATION (DRHOLOOP, ADOTLOOP)
;
; COMMENT-ONLY READERS: The spacecraft needs to know how fast it's rotating
; in order to stop rotations smoothly without overshooting. Since Columbia
; had no direct rotation rate sensors, the guidance computer estimated rotation
; rates by comparing the current spacecraft attitude with the attitude from a
; moment ago. This "rate filter" smooths out sensor noise and spacecraft
; vibrations to produce clean rate estimates for thruster control.
;
; CODE-ALONG READERS: This section implements a Kalman-like filter for
; estimating angular velocity (OMEGA) in each axis by differencing consecutive
; IMU gimbal angle measurements. The filter processes three axes sequentially
; using index SPNDX (0,1,2 for roll, pitch, yaw). The core filter equation is:
;   DRHO(n) = DELRHO - 0.1*ADOT + (1-GAIN1)*DRHO(n-1)
;   ADOT(n) = ADOT(n-1) + GAIN2*DRHO + (1/I)*torque
; where DELRHO = matrix*(CDU_current - CDU_previous) is the measured rate,
; DRHO is the filtered rate estimate, ADOT is angular acceleration, and
; GAIN1/GAIN2 are Kalman filter gains selected based on operational mode.
; The filter attenuates high-frequency noise while tracking true body rates.

		CAF	TWO
DRHOLOOP	TS	SPNDX
		DOUBLE
		TS	DPNDX
		INDEX	DPNDX
		CS	DRHO		# DRHO SCALED 180 DEGS
		EXTEND
		INDEX	ATTKALMN	# PICK UP DESIRED FILTER GAIN
		MP	GAIN1
		INDEX	DPNDX
		DAS	DRHO		# (1 -.064)DRHO
		EXTEND
# Page 1005
		INDEX	DPNDX
		DCS	ADOT
		DXCH	KMPAC		# -(.1)ADOT
		CA	QUARTER
		TC	SMALLMP
		DXCH	KMPAC
		INDEX	DPNDX
		DAS	DRHO
		CCS	SPNDX
		TCF	DRHOLOOP

		CA	CDUX		# MEASURED BODY RATES--
		XCH	RHO
		EXTEND
		MSU	RHO		# -        *     -     -
		COM			# DELRHO = AMGB (CDU - CDU  )
					#			  -1
		ZL
		DXCH	DELTEMPX
		CA	CDUY
		XCH	RHO1
		EXTEND
		MSU	RHO1
		COM
		TS	T5TEMP		# (CDUY - RHO1)	   SCALED 90 DEGS
		EXTEND
		MP	AMGB1
		DAS	DELTEMPX	# DELTEMPX = (CDUX-RHO) + AMGB1(CDUY-RHO1)
					# MUST BE DOUBLE PRECISION OR WILL LOSE
					# PULSES
		CA	AMGB4
		EXTEND
		MP	T5TEMP
		DXCH	DELTEMPY
		CA	AMGB7
		EXTEND
		MP	T5TEMP
		DXCH	DELTEMPZ
		CA	CDUZ
		XCH	RHO2
		EXTEND
		MSU	RHO2
		COM
		TS	T5TEMP		# (CDUZ - RHO2)    SCALED 90 DEGS
		EXTEND
		MP	AMGB5
		DAS	DELTEMPY	# DELTEMPY =AMGB4(CDUY-RHO1)
					#		  + AMGB5(CDUZ-RHO2)
		CA	AMGB8
		EXTEND

# Page 1006
		MP	T5TEMP
		DAS	DELTEMPZ	# DELTEMPZ = AMBG7(CDUY-RHO1)
					#		  + AMGB8(CDUZ-RHO2)

; ANGULAR ACCELERATION UPDATE (ADOTLOOP)
;
; COMMENT-ONLY READERS: After computing rotation rates, the autopilot now
; updates its estimate of angular acceleration (how fast the rotation rate
; is changing). This helps predict future motion and enables smoother control.
; The acceleration estimate accounts for thruster torques and external
; disturbances like solar pressure or residual atmospheric drag.
;
; CODE-ALONG READERS: ADOTLOOP updates angular acceleration (ADOT) using:
;   ADOT(n) = ADOT(n-1) + GAIN2*DRHO + (1/Inertia)*Disturbance_Torque
; The loop processes three axes sequentially. GAIN2 is the Kalman filter
; innovation gain. The disturbance torque term (DFT) accounts for external
; torques not modeled by the RCS jets. ADOT is scaled as angular acceleration
; in radians/sec^2 (scaled by appropriate power of 2). This acceleration
; estimate feeds back into DRHOLOOP for improved rate estimation.

		CAF	TWO
ADOTLOOP	TS	SPNDX
		DOUBLE
		TS	DPNDX
		EXTEND
		INDEX	DPNDX
		DCA	DELTEMPX
		INDEX	DPNDX
		DAS	DRHO
		EXTEND
		INDEX	DPNDX
		DCA	DELTEMPX
		INDEX	DPNDX
		DAS	MERRORX
		INDEX	DPNDX
		CA	DRHO
		DOUBLE			# N.B.
		DOUBLE			# N.B.
		EXTEND
		INDEX	ATTKALMN	# PICK UP DESIRED FILTER GAINS
		MP	GAIN2
		INDEX	DPNDX		# ADOT   + (.16)(.1)DRHO
		DAS	ADOT		#     -1
		INDEX	SPNDX		# S/C TORQUE TO INERTIA RATIO
		CA	KMJ		# SCALED (450)(1600)/(57.3)(16384)=1/1.3
		EXTEND
		INDEX	SPNDX
		MP	DFT
		INDEX	DPNDX
		DAS	ADOT		# KMJ(DFT)
		CCS	SPNDX
		TCF	ADOTLOOP	# END CALCULATION OF VEHICLE RATES
KMATRIX		CA	ATTSEC
		MASK	LOW4
		CCS	A
		TCF	TENTHSEK
		CAF	PRIO34		# CALL FOR 1 SEC UPDATE OF TRANSFORMATION
		TC	NOVAC		# MATRIX FROM GIMBAL AXES TO BODY AXES
		EBANK=	KMPAC
		2CADR	AMBGUPDT

		CAF	NINE

TENTHSEK	TS	ATTSEC

; ============================================================================
; TRANSITION: Attitude Hold and Deadband Control - Propellant Conservation
;
; The RCS autopilot's attitude hold mode is the operational implementation of
; deadband control - a fundamental strategy for minimizing propellant consumption
; while maintaining spacecraft orientation. The HOLDFLAG variable orchestrates
; this mode switching between active maneuvering and quiescent attitude hold.
;
; HOLDFLAG States and Operational Meaning:
;   + (positive)  = Capture current attitude and establish new hold reference
;                   Inhibits automatic steering commands
;                   Triggered by: manual RHC input, autopilot initialization,
;                   Free mode, IMU power off, or SCS control mode
;   +0 (plus zero) = Maintaining previously established attitude hold reference
;                   No thruster activity unless errors exceed deadband limits
;                   This is the propellant-conserving idle state
;   - (negative)   = Executing automatic maneuver commanded by guidance programs
;                   Active attitude changes for burns, tracking, or maneuvers
;   -0 (minus zero) = Reserved (not currently used in operational logic)
;
; Deadband Implementation Philosophy:
; Rather than continuously correcting tiny attitude errors (which would waste
; propellant through endless micro-corrections), the autopilot implements an
; implicit deadband through its control gains and minimum impulse logic. Small
; errors within the deadband produce commanded torques below the minimum impulse
; threshold, preventing thruster firing. Only when accumulated error or rate
; exceeds thresholds does the jet selection logic activate thrusters.
;
; During Apollo 11's mission phases, attitude hold mode was critical for:
; - Long coast periods (translunar/transearth) where propellant conservation
;   was paramount - the CSM maintained passive thermal control ("barbecue roll")
;   attitude for hours with minimal RCS usage
; - Post-SPS burn stabilization after TLI, LOI, and TEI maneuvers
; - Platform stability during P51/P52 star sighting for IMU realignment
; - Holding inertial attitude during Mike Collins' landmark tracking observations
; - Maintaining communication antenna pointing during coast phases
;
; The steering programs (attitude maneuver routine, LEM tracking) must explicitly
; set HOLDFLAG negative to command the autopilot. Astronaut actions (RHC inputs,
; mode switch changes) can immediately override automatic maneuvers by setting
; HOLDFLAG positive, giving crew ultimate authority over spacecraft attitude.
;
; This section processes automatic steering commands by interpolating between
; commanded gimbal angles (CDUXD, CDUYD, CDUZD) and incremental updates
; (DELCDUX, DELCDUY, DELCDUZ), providing smooth attitude profiles during
; multi-axis maneuvers while respecting crew override authority at all times.
; ============================================================================

# Page 1007
# WHEN AUTOMATIC MANEUVERS ARE BEING PERFORMED, THE FOLLOWING ANGLE ADDITION MUST BE MADE TO PROVIDE A SMOOTH
# SEQUENCE OF ANGULAR COMMANDS TO THE AUTOPILOT--

#	CDUXD = CDUXD + DELCDUX		(DOUBLE PRECISION)
#	CDUYD = CDUYD + DELCDUY		(DOUBLE PRECISION)
#	CDUZD = CDUZD + DELCDUZ		(DOUBLE PRECISION)

# THE STEERING PROGRAMS-
#	1) ATTITUDE MANEUVER ROUTINE
#	2) LEM TRACKING

# SHOULD GENERATE THE DESIRED ANGLES (CDUXD, CDUYD, CDUZD) AS WELL AS THE INCREMENTAL ANGLES (DELCDUX, DELCDUY,
# DELCDUZ) SO THAT THE GIMBAL ANGLE COMMANDS CAN BE INTERPOLATED BETWEEN UPDATES.

# HOLDFLAG CODING-

#	+ = GRAB PRESENT CDU ANGLES AND STORE IN THETADX, THETADY, THETADZ
#	    AND PERFORM ATTITUDE HOLD ABOUT THESE ANGLES
#	    ALSO IGNORE AUTOMATIC STEERING
#	    SET = + BY
#		1) INITIALIZATION PHASE OF AUTOPILOT
#		2) OCCURANCE OF RHC COMMANDS
#		3) FREE MODE
#		4) SWITCH OVER TO ATTITUDE HOLD FROM AUTO
#		   WHILE DOING AUTOMATIC STEERING (IN THIS CASE
#		   HOLDFLAG IS NOT ACTUALLY SET TO +, BUT THE LOGIC
#		   FUNCTIONS AS IF IT WERE.)
#		5) S/C CONTROL SWITCH IS SCS
#		6) IMU POWER OFF
#      +0 = IN ATTITUDE HOLD ABOUT A PREVIOUSLY ESTABLISHED REFERENCE
#	- = PERFORMING AUTOMATIC MANEUVER
#      -0 = NOT USED AT PRESENT


# 	NOTE THAT THIS FLAG MUST BE SET = - BY THE STEERING PROGRAM IF IT IS TO COMMAND THE AUTOPILOT.
# SINCE ASTRONAUT ACTION MAY CHANGE THE HOLDFLAG SETTING, IT SHOULD BE MONITORED BY THE STEERING PROGRAM TO
# DETERMINE IF THE AUTOMATIC SEQUENCE HAS BEEN INTERRUPTED AND IF SO, TAKE THE APPROPRIATE ACTION.

		CS	HOLDFLAG
		EXTEND
		BZMF	DACNDLS		# IF HOLDFLAG +0,-0,+, BYPASS AUTOMATIC
					# COMMANDS
DCDUINCR	CAF	TWO
DELOOP		TS	SPNDX
		DOUBLE
		TS	DPNDX
		EXTEND
		INDEX	A
		DCA	CDUXD
# Page 1008
		DXCH	KMPAC
		EXTEND
		INDEX	DPNDX
		DCA	DELCDUX
		TC	DPADD
		EXTEND
		DCA	KMPAC
		INDEX	SPNDX
		TS	THETADX
		INDEX	DPNDX
		DXCH	CDUXD
		CCS	SPNDX
		TCF	DELOOP

; ============================================================================
; TRANSITION: FDAI Error Needle Display - Crew Situation Awareness
;
; The Flight Director Attitude Indicator (FDAI) - nicknamed the "8-ball" -
; was the primary attitude reference display for Apollo astronauts. Located
; on the main display console, it showed spacecraft orientation through a
; rotating sphere marked with pitch, roll, and yaw references. Three error
; needles surrounding the ball provided quantitative attitude error feedback.
;
; This section implements the digital-to-analog converter (DAC) interface that
; drives the FDAI error needles, providing three display modes:
;
; MODE 1 (V61E): Autopilot Following Errors
;   Displays how well the autopilot tracks commanded attitudes. During Apollo 11,
;   Mike Collins monitored these errors during automatic SPS burns (TLI, LOI, TEI)
;   to verify the guidance computer was maintaining proper attitude. Needles
;   centered meant the autopilot was successfully tracking navigation commands.
;
; MODE 2 (V62E): Total Attitude Errors (N22 Reference)
;   Fly-to display showing errors between current attitude and target gimbal
;   angles in N22. Used during manual maneuvers to desired attitudes, such as
;   aligning for landmark tracking or setting up for rendezvous observations
;   of the LM Eagle.
;
; MODE 3 (V63E): Total Astronaut Attitude Errors (N17 Reference)
;   Fly-to display using N17 as reference. Crew could "capture" current attitude
;   into N17 using V60E, then manually maneuver while monitoring return errors.
;   Useful for returning to a previous reference attitude after completing
;   off-nominal maneuvers.
;
; During critical mission phases, these displays provided essential feedback:
; - SPS burn monitoring: Centered needles confirmed proper guidance tracking
; - Manual photography: Error needles guided precise attitude positioning
; - Rendezvous operations: Enabled accurate LM tracking attitude maintenance
; - Star sighting: Helped maintain stable platform for P51/P52 alignments
;
; The DAC outputs drive analog meter movements with ±5-degree full-scale
; deflection, providing intuitive visual feedback that complemented digital
; DSKY displays. This combination of analog and digital interfaces optimized
; crew situation awareness during all mission phases.
; ============================================================================

# Page 1009
# RCS-CSM AUTOPILOT ATTITUDE ERROR DISPLAY

# THREE TYPES OF ATTITUDE ERRORS MAY BE DISPLAYED ON THE FDAI-

#	MODE 1)	AUTOPILOT FOLLOWING ERRORS		SELECTED BY V61E
#		GENERATED INTERNALLY BY THE AUTOPILOT

#	MODE 2)	TOTAL ATTITUDE ERRORS			SELECTED BY V62E
#		WITH RESPECT TO THE CONTENTS OF N22

#	MODE 3)	TOTAL ASTRONAUT ATTITUDE ERRORS		SELECTED BY V63E
#		WITH RESPECT TO THE CONTENTS OF N17

# MODE 1 IS PROVIDED AS A MONITOR OF THE RCS DAP AND ITS ABILITY TO TRACK AUTOMATIC STEERING COMMANDS.  IN THIS
# MODE THE ATTITUDE ERRORS WILL BE ZEROED WHEN THE CMC MODE SWITCH IS IN FREE

# MODE 2 IS PROVIDED TO ASSIST THE CREW IN MANUALLY MANEUVERING THE S/C TO THE ATTITUDE (GIMBAL ANGLES) SPECIFIED
# IN N22.  THE ATTITUDE ERRORS WRT THESE ANGLES AND THE CURRENT CDU ANGLES ARE RESOLVED INTO S/C CONTROL AXES
# AS A FLY-TO INDICATOR.

# MODE 3 IS PROVIDED TO ASSIST THE CREW IN MANUALLY MANEUVERING THE S/C TO THE ATTITUDE (GIMBAL ANGLES) SPECIFIED
# IN N17.  THE ATTITUDE ERRORS WRT THESE ANGLES AND THE CURRENT CDU ANGLES ARE RESOLVED INTO S/C CONTROL AXES
# AS A FLY-TO INDICATOR.

# 	V60 IS PROVIDED TO LOAD N17 WITH A SNAPSHOT OF THE CURRENT CDU ANGLES, THUS SYNCHRONIZING THE MODE 3 DISPLAY
# WITH THE CURRENT S/C ATTITUDE.  THIS VERB MAY BE USED AT ANY TIME.

# 	THESE DISPLAYS WILL BE AVAILABLE IN ANY MODE (AUTO, HOLD, FREE, G+N, OR SCS) ONCE THE RCS DAP HAS BEEN
# INITIATED VIA V46E.  MODE 1, HOWEVER, WILL BE MEANINGFUL ONLY IN G+N AUTO OR HOLD.  THE CREW MAY PRESET (VIA
# V25N17) AN ATTITUDE REFERENCE (DESIRED GIMBAL ANGLES) INTO N17 AT ANY TIME.

DACNDLS		CS	RCSFLAGS	# ALTERNATE BETWEEN FDAIDSP1 AND FDAIDSP2
		MASK	BIT4
		EXTEND
		BZF	FDAIDSP2

FDAIDSP1	ADS	RCSFLAGS
		TC	NEEDLER
KRESUME1	TCF	RESUME		# END PHASE 1

# Page 1010
# FDAI ATTITUDE ERROR DISPLAY SUBROUTINE

# PROGRAM DESCRIPTION:	D. KEENE  5/24/67

# 	THIS SUBROUTINE IS USED TO DISPLAY ATTITUDE ERRORS ON THE FDAI VIA THE DIGITAL TO ANALOG CONVERTERS (DACS)
# IN THE CDUS.  CARE IS TAKEN TO METER OUT THE APPROPRIATE NUMBER OF PULSES TO THE IMU ERROR COUNTERS AND PREVENT
# OVERFLOW, TO CONTROL THE RELAY SEQUENCING, AND TO AVOID INTERFERENCE WITH THE COARSE ALIGN LOOP WHICH ALSO USES
# THE DACS.


# CALLING SEQUENCE:

# 	DURING THE INITIALIZATION SECTION OF THE USER'S PROGRAM, BIT3 OF RCSFLAGS SHOULD BE SET TO INITIATE THE
# TURN-ON SEQUENCE WITHIN THE NEEDLES PROGRAM:

#		CS	RCSFLAGS	# IN EBANK6
#		MASK	BIT3
#		ADS	RCSFLAGS

# THEREAFTER, THE ATTITUDE ERRORS GENERATED BY THE USER SHOULD BE TRANSFERED TO THE FOLLOWING LOCATIONS IN EBANK6:

#		AK	SCALED 180 DEGREES	NOTE:	THESE LOCATIONS ARE SUBJECT
#		AK1	SCALED 180 DEGREES		TO CHANGE
#		AK2	SCALED 180 DEGREES

# FULL SCALED DEFLECTION CORRESPONDS TO 16 7/8 DEGREES OF ATTITUDE ERROR
#		(= 384 BITS IN IMU ERROR COUNTER)

# A CALL TO NEEDLER WILL THEN UPDATE THE DISPLAY:

#		INHINT
#		TC	IBNKCALL	# NOTE: EBANK SHOULD BE SET TO E6
#		CADR	NEEDLER
#		RELINT

# 	THIS PROCESS SHOULD BE REPEATED EACH TIME THE ERRORS ARE UPDATED.  AT LEAST 3 PASSES THRU THE PROGRAM ARE
# REQUIRED BEFORE ANYTHING IS ACTUALLY DISPLAYED ON THE ERROR METERS.
# NOTE: EACH CALL TO NEEDLER MUST BE SEPARATED BY AT LEAST 50MS TO ASSURE PROPER RELAY SEQUENCING.

# ERASABLE USED:
#		AK		CDUXCMD
#		AK1		CDUYCMD
#		AK2		CDUZCMD
#		EDRIVEX		A,L,Q
#		EDRIVEY		T5TEMP
#		EDRIVEZ		SPNDX

# SWITCHES:	RCSFLAGS	BITS 3,2

# I/O CHANNELS:	CHAN12		BIT 4 (COARSE ALIGN - READ ONLY)
# Page 1011
#		CHAN12		BIT 6 (IMU ERROR COUNTER ENABLE)
#		CHAN14		BIT 13,14,15 (DAC ACTIVITY)


# SIGN CONVENTION<	AK = THETAC - THETA
#		WHERE	THETAC = COMMAND ANGLE
#			THETA = PRESENT ANGLE


NEEDLER		CAF	BIT4		# CHECK FOR COARSE ALIGN ENABLE
		EXTEND			# IF IN COARSE ALIGN DO NOT USE IMU
		RAND	CHAN12		# ERROR COUNTERS.  DONT USE NEEDLES
		EXTEND
		BZF	NEEDLER1
		CS	RCSFLAGS	# SET BIT3 FOR INITIALIZATION PASS
		MASK	BIT3
		ADS	RCSFLAGS
		TC	Q

NEEDLER1	CA	RCSFLAGS
		MASK	SIX
		EXTEND
		BZF	NEEDLES3
		MASK	BIT3
		EXTEND
		BZF	NEEDLER2	# BIT3 = 0, BIT2 = 1

		CS	BIT6		# FIRST PASS BIT3 = 1
		EXTEND			# DISABLE IMU ERROR COUNTER TO ZERO DACS
		WAND	CHAN12		# MUST WAIT AT LEAST 60 MS BEFORE
NEEDLE11	CS	ZERO		# ENABLING COUNTERS.
		TS	AK		# ZERO THE INPUTS ON FIRST PASS
		TS	AK1
		TS	AK2
		TS	EDRIVEX		# ZERO THE DISPLAY REGISTERS
		TS	EDRIVEY
		TS	EDRIVEZ
		TS	CDUXCMD		# ZERO THE OUT COUNTERS
		TS	CDUYCMD
		TS	CDUZCMD
		CS	SIX		# RESET RCSFLAGS FOR PASS2
		MASK	RCSFLAGS
		AD	BIT2
		TS	RCSFLAGS
		TC	Q		# END PASS1

NEEDLER2	CAF	BIT6		# ENABLE IMU ERROR COUNTERS
		EXTEND
		WOR	CHAN12
		CS	SIX		# RESET RCSFLAGS TO DISPLAY ATTITUDE
# Page 1012
		MASK	RCSFLAGS	# ERRORS    WAIT AT LEAST 4 MS FOR
		TS	RCSFLAGS	# RELAY CLOSURE
		TC	Q

NEEDLES3	CAF	BIT6		# CHECK TO SEE IF IMU ERROR COUNTER
		EXTEND			# IS ENABLED
		RAND	CHAN12
		EXTEND			# IF NOT RECYCLE NEEDLES
		BZF	NEEDLER +5

NEEDLES		CAF	TWO
DACLOOP		TS	SPNDX
		CS	QUARTER
		EXTEND
		INDEX	SPNDX
		MP	AK
		TS	L
		CCS	A
		CA	DACLIMIT
		TCF	+2
		CS	DACLIMIT
		AD	L
		TS	T5TEMP		# OVFLO CHK
		TCF	+4
		INDEX	A		# ON OVERFLOW LIMIT OUTPUT TO +-384
		CAF	DACLIMIT
		TS	L
		INDEX	SPNDX
		CS	EDRIVEX		# CURRENT VALUE OF DAC
		AD	L
		INDEX	SPNDX
		ADS	CDUXCMD
		INDEX	SPNDX
		LXCH	EDRIVEX
		CCS	SPNDX
		TCF	DACLOOP
		CAF	13,14,15
		EXTEND
		WOR	CHAN14		# SET DAC ACTIVITY BITS
		TC	Q

REINIT		CAF	DELAY200	# ........TILT LOGIC
		TS	TIME5		# REINITIALIZE DAP IN 200MS
		TS	T5PHASE
		TCF	RESUME
DELAY200	DEC	16364		# 200MS

		DEC	-384

# Page 1013
DACLIMIT	DEC	16000
		DEC	384

# Page 1014
# INITIALIZATION PROGRAM FOR RCS-CSM AUTOPILOT

# THE FOLLOWING QUANTITIES WILL BE ZEROED AND SHOULD APPEAR IN CONSECUTIVE LOCATIONS IN MEMORY AFTER WBODY

# WBODY		(+1)		DFT			TAU2
# WBODY1	(+1)		DFT1			BIAS
# WBODY2	(+1)		DFT2			BIAS1
# ADOT		(+1)		DRHO	(+1)		BIAS2
# ADOT1		(+1)		DRHO1	(+1)		ERRORX
# ADOT2		(+1)		DRHO2	(+1)		ERRORY
# MERRORX	(+1)		ATTSEC			ERRORZ
# MERRORY	(+1)		TAU
# MERRORZ	(+1)		TAU1
FRESHDAP	CAF	ONE		# RESET HOLDFLAG TO STOP AUTOMATIC
		TS	HOLDFLAG	# STEERING AND PREPARE TO PICK UP AN
					# ATTITUDE HOLD REFERENCE

REDAP		TC	IBNKCALL	# DECODE DAPDATR1, DAPDATR2 FOR DEADBANDS
		CADR	S41.2		# RATES, QUADFAILS, QUAD MANAGEMENT

		TC	IBNKCALL	# DECODE IXX, IAVG, AND CONVERT
		CADR	S40.14		# TO AUTOPILOT GAINS

		CAF	NO.T5VAR	# NO. LOCATIONS TO BE ZEROED MINUS ONE
ZEROT5		TS	SPNDX		# ZERO ALL NECESSARY ERASABLE REGISTERS
		CAF	ZERO
		INDEX	SPNDX
		TS	WBODY
		CCS	SPNDX
		TCF	ZEROT5
		TCR	ZEROJET

		CS	ZERO
		TS	CHANTEMP	# INITIALIZE MINIMUM IMPULSE CONTROL

		TS	CH31TEMP	# INITIALIZE RHC POSITION MEMORY FOR
					# MANUAL RATE MODES

		CAF	=.24
		TS	SLOPE		# INITIALIZE SWITCHING LOGIC SLOPE

		CAF	FOUR
		TS	T5TIME		# PHASE 0 RESETS FOR PHASE 2 INTERRUPT IN
					# 60 MS.  PHASE 2 RESETS FOR PHASE 1 RUPT
					# IN (80MS - T5TIME(40MS)).  THEREFORE
					# PHASE 1 (RATEFILTER) BEGINS CYCLING 100
					# MS FROM NOW AND EVERY 100MS THEREAFTER.

		CAF	ELEVEN
		TS	ATTKALMN	# RESET TO PICK UP KALMAN FILTER GAINS
					# TO INITIALIZE THE S/C ANGULAR RATES
# Page 1015
		CA	CDUX
		TS	RHO
		CA	CDUY
		TS	RHO1
		CA	CDUZ
		TS	RHO2
		CAF	ZERO		# RESET AUTOPILOT TO BEGIN EXECUTING
		TS	T5PHASE		# PHASE2 OF PROGRAM

		CS	IMODES33	# CHECK IMU STATUS
		MASK	BIT6		# IF BIT6 = 0 IMU IN FINE ALIGN
		CCS	A		# IF BIT6 = 1 IMU NOT READY
		TCF	IMUAOK
		TS	ATTKALMN	# CANNOT USE IMU
		CAF	RCSINITB	# PROVIDE FREE CONTROL ONLY
		TCF	RCSSWIT		# DONT START UP RATE FILTER
					# SIGNAL NO RATE FILTER

IMUAOK		CAF	PRIO34		# START MATRIX INITIALIZATION
		TC	NOVAC		# BYPASS IF IMU NOT IN FINE ALIGN
		EBANK=	KMPAC
		2CADR	AMBGUPDT

		CAF	RCSINIT		# CLEAR BIT14 -ASSUME WE HAVE A GOOD IMU
RCSSWIT		TS	RCSFLAGS	# CLEAR BIT1  -INITIALIZE T6 PROGRAM
					#   SET BIT3  -INITIALIZE NEEDLES
					# CLEAR BIT4  -RESET FOR FDAIDSP1
		CAF	T5WAIT60	# NEXT T5RUPT 60 MS FROM NOW TO ALLOW IMU
					# ERROR COUNTER TO ZERO.
					# (MINIMUM DELAY = 15 MS)
		TS	TIME5		# SINCE ATTKALMN IS +11, PROGRAM WILL THEN
		TC	RESUME		# PICK UP THE KALMAN FILTER GAINS.  RATE
					# FILTER WILL BEGIN OPERATING ZOOMS FROM
					# NOW

# CONSTANTS USED IN INITIALIZATION PROGRAM

NO.T5VAR	DEC	36
=.24		DEC	.24		# = SLOPE OF 0.6/SEC
RCSINIT		OCT	00004
RCSINITB	OCT	20004
T5WAIT60	DEC	16378		# = 6 CS
		EBANK=	KMPAC
T6ADDR		2CADR	T6START

ZEROJET		CAF	ELEVEN		# ZERO BLAST2, BLAST1, BLAST, YWORD2,
		TS	SPNDX		# YWORD1, PWORD2, PWORD1, RWORD2,
		CAF	ZERO		# AND RWORD1.

# Page 1016
		INDEX	SPNDX
		TS	RWORD1
		CCS	SPNDX
		TCF	ZEROJET +1

		CAF	FOUR
		TS	BLAST1 +1
		CAF	ELEVEN
		TS	BLAST2 +1

		CS	BIT1
		MASK	RCSFLAGS
		TS	RCSFLAGS	# RESET BIT1 OF RCSFLAGS TO 0

		EXTEND
		DCA	T6ADDR
		DXCH	T6LOC
		CAF	=+14MS		# ENABLE T6RUPT TO SHUT OFF JETS IN 14 MS.
		TS	TIME6
		CAF	BIT15
		EXTEND
		WOR	CHAN13

		TC	Q

T5PHASE2	CCS	ATTKALMN	# IF (+) INITIALIZE RATE ESTIMATE
		TCF	KALUPDT

		TCF	+2		# ONLY IF ATTKALMN POSITIVE
		TCF	+1
		CA	DELTATT2	# RESET FOR PHASE3 IN 20 MS
		XCH	TIME5		# (JET SELECTION LOGIC)
		ADS	T5TIME		# TO COMPENSATE FOR DELAYS IN T5RUPT

		CA	RCSFLAGS	# IF A HIGH RATE AUTO MANEUVER IS IN
		MASK	BIT15		# PROGRESS (BIT 15 OF RCSFLAGS SET), SET
		EXTEND			# ATTKALMN TO -1
		BZF	NOHIAUTO	# OTHERWISE SET ATTKALMN TO 0.
		CS	ONE
NOHIAUTO	TS	ATTKALMN

# Page 1017

; ============================================================================
; TRANSITION: Manual Rotation Commands - Crew Control Interface
;
; During Apollo 11, Command Module Pilot Mike Collins used the Rotational Hand
; Controller (RHC) to manually command spacecraft attitude changes for various
; mission operations including:
; - Photography sessions of Earth and Moon during translunar coast
; - Landmark tracking in lunar orbit (P22 program support)
; - Attitude alignment before SPS engine burns
; - Rendezvous alignment to track Eagle during LM operations
; - Star sighting for P51/P52 IMU alignment programs
;
; The RHC provides three-axis rate commands (roll, pitch, yaw) in discrete
; levels: small rate (~0.05°/sec), medium rate (~0.2°/sec), large rate (~2°/sec).
; Channel 31 bits 0-5 encode these commands, which this section decodes to set
; forced firing flags and body rate commands for the autopilot.
;
; When manual commands are detected, the autopilot maintains rate damping while
; allowing the crew to control spacecraft orientation. This hybrid control gives
; astronauts precise authority over attitude while the AGC handles thruster
; coordination and propellant conservation.
; ============================================================================

# MANUAL ROTATION COMMANDS

		CS	OCT01760	# RESET FORCED FIRING BITS (BITS 10 TO 5
		MASK	RCSFLAGS	# OF RCSFLAGS) TO ZERO
		TS	RCSFLAGS

		EXTEND
		READ	CHAN31
		TS	L
		CA	CH31TEMP
		EXTEND
		RXOR	LCHAN
		MASK	MANROT		# = OCT00077
		EXTEND
		BZMF	NOCHANGE

		LXCH	A
		TS	CH31TEMP	# SAVE CONTENTS OF CHANNEL 31 IN CH31TEMP

		CA	L
		EXTEND
		MP	BIT5		# PUT BITS 6-1 OF A IN BITS 10-5 OF L
		CA	L
		ADS	RCSFLAGS	# SET FORCED FIRING BITS FOR AXES WITH
					# CHANGES IN COMMAND.  BITS 10,9 FOR
					# ROLL, BITS 8,7 FOR YAW, BITS 6,5 FOR
					# PITCH

		CS	RCSFLAGS	# SET RATE DAMPING FLAGS (BITS 13,12,AND
		MASK	OCT16000	# 11 OF RCSFLAGS)
		ADS	RCSFLAGS

NOCHANGE	CS	CH31TEMP
		MASK	MANROT
		EXTEND
		BZMF	AHFNOROT	# IF NO MANUAL COMMANDS, GO TO AHFNOROT

		TS	HOLDFLAG	# SET HOLDFLAG +

		TC	STICKCHK	# WHEN THE RHC IS OUT OF DETENT, PMANNDX,
					# YMANNDX, AND RMANNDX ARE ALL SET, BY
					# MEANS OF STICKCHK, TO 0, 1, OR 2 FOR NO,
					# +, OR - ROTATION RESPECTIVELY AS
					# COMMANDED BY THE RHC.

					# HOWEVER, IT IS WELL TO NOTE THAT AFTER
					# THE RHC IS RETURNED TO DETENT, THE
					# PROGRAM BRANCHES TO AHFNOROT AND AVOIDS
					# STICKCHK SO PMANNDX, YMANNDX, AND
					# RMANNDX ARE NOT RESET TO ZERO BUT RATHER
					# LEFT SET TO THEIR LAST OUT OF DETENT
# Page 1018
					# VALUES.

		CS	FLAGWRD1	# SET STIKFLAG TO INFORM STEERING
		MASK	BIT14		# PROGRAMS (P20) THAT ASTRONAUT HAS
		ADS	FLAGWRD1	# ASSUMED ROTATIONAL CONTROL OF SPACECRAFT

		CAF	BIT14
		EXTEND
		RAND	CHAN31
		EXTEND
		BZMF	FREEFUNC

		CA	RCSFLAGS	# EXAMINE RCSFLAGS TO SEE IF RATE FILTER
		MASK	BIT14		# HAS BEEN INITIALIZED
		CCS	A		# IF SO, PROCEED WITH MANUAL RATE COMMANDS
		TCF	REINIT		# .....TILT, RECYCLE TO INITIALIZE FILTER

		CS	FIVE		# IF MANUAL MANEUVER IS AT HIGH RTE, SET
		AD	RATEINDX	# ATTKALMN TO -1.
		EXTEND			# OTHERWISE, LEAVE ATTKALMN ALONE.
		BZMF	+3
		CS	ONE
		TS	ATTKALMN

		CAF	TWO		# AUTO-HOLD MANUAL ROTATION
SETWBODY	TS	SPNDX
		DOUBLE
		TS	DPNDX
		INDEX	SPNDX		# RMANNDX = 0 NO ROTATION
		CA	RMANNDX		#	  = 1  + ROTATION
		EXTEND			#  	  = 2  - ROTATION
		BZF	NORATE		# IF NO ROTATION COMMAND ON THIS AXIS,
					# GO TO NORATE.

		AD	RATEINDX	# RATEINDX = 0  0.05 DEG/SEC
		TS	Q		#          = 2  0.2  DEG/SEC
		INDEX	Q		#          = 4  0.5  DEG/SEC
		CA	MANTABLE -1	#          = 6  2.0  DEG/SEC
		EXTEND
		MP	BIT9		# MULTIPLY MANTABLE BY 2 TO THE -6
		INDEX	DPNDX		# TO GET COMMANDED RATE.
		DXCH	WBODY		# SET WBODY TO COMMANDED RATE.

		CA	RCSFLAGS
		MASK	OCT16000	# IS RATE DAMPING COMPLETED (BITS 13,12 AND
		EXTEND			# 11 OF RCSFLAGS ALL ZERO.)  IF SO, GO TO
		BZF	MERUPDAT	# MERUPDAT TO UPDATE CUMULATIVE ATTITUDE
					# ERROR.

# Page 1019
ZEROER		CA	ZERO		# ZEROER ZEROS MERRORS
		ZL
		INDEX	DPNDX
		DXCH	MERRORX
		TCF	SPNDXCHK

NORATE		ZL
		INDEX	DPNDX
		DXCH	WBODY		# ZERO WBODY FOR THIS AXIS
		CA	RCSFLAGS
		MASK	OCT16000
		EXTEND			# IS RATE DAMPING COMPLETED
		BZF	SPNDXCHK	# YES, KEEP CURRENT MERRORX GO TO SPNDXCHK
		TCF	ZEROER		# NO,GO TO ZEROER

MERUPDAT	INDEX	Q		# MERRORX=MERRORX+MEASURED CHANGE IN ANGLE
		CS	MANTABLE -1	# -COMMANDED CHANGE IN ANGLE
		EXTEND			# THE ADDITION OF MEASURED CHANGE IN ANGLE
		MP	BIT7		# HAS ALREADY BEEN DONE IN THE RATE FILTER
		INDEX	DPNDX		# COMMANDED CHANGE IN ANGLE = WBODY TIMES
		DAS	MERRORX		# .1SEC = MANTABLE ENTRY TIMES 2 TO THE -8

SPNDXCHK	INDEX	DPNDX
		CA	MERRORX
		INDEX	SPNDX
		TS	ERRORX		# ERRORX = HIGH ORDER WORD OF MERRORX
		CCS	SPNDX
		TCF	SETWBODY

; ============================================================================
; TRANSITION: From Autopilot Computations to Jet Selection Logic
;
; All three-axis attitude errors (ERRORX, ERRORY, ERRORZ) have been computed
; and the autopilot control law calculations are complete. Control now
; transfers to the JETS routine (in AUTOMATIC_MANEUVERS.agc) which interfaces
; with the jet selection logic to translate these attitude errors into actual
; RCS thruster firing commands. The jet selection logic (JET_SELECTION_LOGIC.agc)
; determines optimal thruster pairs to minimize propellant consumption while
; achieving the commanded attitude changes.
;
; Throughout Apollo 11's mission, this handoff occurred every 100 milliseconds
; during autopilot operation, with the spacecraft's 16 RCS thrusters firing
; in carefully coordinated patterns to maintain attitude control during coast
; phases, maneuvers, and while docked with the Lunar Module.
; ============================================================================

		TCF	JETS

OCT01760	OCT	01760		# FORCED FIRING BITS MASK

OCT01400	OCT	01400		# ROLL FORCED FIRING MASK	ORDER OF
OCT00060	OCT	00060		# PITCH FORCED FIRING MASK	DEFINITION
OCT00300	OCT	00300		# YAW FORCED FIRING MASK	MUST BE
					#				PRESERVED
					#			      FOR INDEXING
MANROT		OCT	77
OCT16000	OCT	16000		# RATE DAMPING FLAGS MASK
MANTABLE	DEC	.0071111
		DEC	-.0071111
		DEC	.028444
		DEC	-0.028444
		DEC	.071111
		DEC	-.071111
		DEC	.284444
		DEC	-.284444
=+14MS		DEC	23
FREEFUNC	INDEX	RMANNDX		# ACCELERATION
# Page 1020
		CA	FREETAU		# COMMANDS
		TS	TAU
		INDEX	PMANNDX
		CA	FREETAU		# FREETAU	0 SEC
		TS	TAU1		# +1	       +0.10 SEC
		INDEX	YMANNDX		# +2	       -0.10 SEC
		CA	FREETAU		# (+3)		0 SEC
		TS	TAU2
		TCF	T6PROGM

FREETAU		DEC	0
		DEC	480
		DEC	-480
		DEC	0

T6PROGM		CAF	ZERO		# FOR MANUAL ROTATIONS
		TS	ERRORX
		TS	ERRORY
		TS	ERRORZ
		TCF	T6PROG

; ============================================================================
; TRANSITION: Kalman Filter Gains - Optimal Attitude Rate Estimation
;
; The RCS autopilot uses a discrete Kalman filter to estimate spacecraft body
; rates from IMU gyroscope measurements while minimizing noise effects. This
; optimal estimation technique was cutting-edge technology in 1969, representing
; sophisticated control theory implemented within the AGC's limited computational
; resources.
;
; Two gain tables are defined here:
; - GAIN1: Ten gain values for iterative filter convergence during startup
;   (scaled values from 0.064 to 0.9342, representing filter confidence growth)
; - GAIN2: Complementary gains for steady-state operation (scaled by 10)
;
; During Apollo 11, these gains enabled:
; - Smooth attitude hold despite IMU noise and thruster impulse disturbances
; - Accurate rate damping for manual rotations by Mike Collins
; - Precise attitude control during critical SPS burns (TLI, LOI, TEI)
; - Stable platform for landmark tracking and star sighting operations
;
; The filter initialization sequence uses progressively larger GAIN1 values
; to converge from uncertain initial conditions to accurate rate estimates
; within approximately one second. Once converged, the filter provides optimal
; rate feedback to the autopilot control law with minimal computational overhead
; (~100 AGC instructions per axis per 20ms control cycle).
;
; This Kalman filter implementation demonstrates the sophistication of Apollo
; flight software - bringing state-of-the-art estimation theory from academic
; research into operational spacecraft systems.
; ============================================================================

# Page 1021
		DEC	.2112		# FILTER GAIN FOR TRANSLATION, LEM ON
		DEC	.8400		# FILTER GAIN FOR TRANSLATION 2(ZETA)WN DT
		DEC	.2112		# FILTER GAIN FOR 2 DEGREES/SEC MANEUVERS
GAIN1		DEC	.0640		# KALMAN FILTER GAINS FOR INITIALIZATION
		DEC	.3180		# OF ATTITUDE RATES
		DEC	.3452
		DEC	.3774
		DEC	.4161
		DEC	.4634
		DEC	.5223
		DEC	.5970
		DEC	.6933
		DEC	.8151
		DEC	.9342

		DEC	.0174		# FILTER GAIN FOR TRANSLATION, LEM ON
		DEC	.3600		# FILTER GAIN FOR TRANSLATION (WN)(WN)DT
		DEC	.0174		# FILTER GAIN FOR 2 DEGREES/SEC MANEUVERS
GAIN2		DEC	.0016		# SCALED 10
		DEC	.0454
		DEC	.0545
		DEC	.0666
		DEC	.0832
		DEC	.1069
		DEC	.1422
		DEC	.1985
		DEC	.2955
		DEC	.4817
		DEC	.8683
STICKCHK	TS	T5TEMP
		MASK	THREE		# INDICES FOR MANUAL ROTATION
		TS	PMANNDX
		CA	T5TEMP
		EXTEND			# MAN RATE  0  0 RATE (DP)
		MP	QUARTER		# 	   +1 	+RATE (DP)
		TS	T5TEMP		#          +2	-RATE (DP)
		MASK	THREE		#	  (+3)	0 RATE (DP)
		TS	YMANNDX
		CA	T5TEMP
		EXTEND
		MP	QUARTER
		TS	RMANNDX
		TC	Q


KALUPDT		TS	ATTKALMN	# INITIALIZATION OF ATTITUDE RATES USING
					# KALMAN FILTER TAKES 1.1 SEC

		CA	DELTATT		# =1SEC - 80MS
		AD	T5TIME		# + DELAYS
# Page 1022
		TS	TIME5
		TCF	+3
		CAF	DELTATT2	# SAFETY PLAY TO ASSURE
		TS	TIME5		# A T5RUPT

KRESUME2	CS	ZERO		# RESET FOR PHASE1
		TS	T5PHASE		# RESUME INTERRUPTED PROGRAM
		TCF	RESUME

FDAIDSP2	CS	BIT4		# RESET FOR FDAIDSP1
		MASK	RCSFLAGS
		TS	RCSFLAGS

		CS	FLAGWRD0	# ON - DISPLAY ONE OF THE TOTAL ATTITUDE
		MASK	BIT9		# ERRORS
		EXTEND
		BZF	FDAITOTL
		EXTEND
		DCS	ERRORX		# OFF -DISPLAY AUTOPILOT FOLLOWING ERROR
		DXCH	AK
		CS	ERRORZ
		TS	AK2
		TCF	RESUME		# END PHASE 1

FDAITOTL	CA	FLAGWRD9
		MASK	BIT6
		EXTEND
		BZF	WRTN17		# IS N22ORN17 (BIT6 OF FLAGWRD9) = 0
					# IF SO, GO TO WRTN17
WRTN22		EXTEND			# OTHERWISE, CONTINUE ON TO WRTN22 AND
		DCA	CTHETA		# GET SET TO COMPUTE TOTAL ATTITUDE
		DXCH	WTEMP		# ERROR WRT N22 BY PICKING UP THE THREE
		CA	CPHI		# COMPONENTS OF N22

GETAKS		EXTEND			# COMPUTE TOTAL ATTITUDE ERROR FOR
		MSU	CDUX		# DISPLAY ON FDAI ERROR NEEDLES
		TS	AK
		CA	WTEMP
		EXTEND
		MSU	CDUY
		TS	T5TEMP
		EXTEND
		MP	AMGB1
		ADS	AK
		CA	T5TEMP
		EXTEND
		MP	AMGB4
# Page 1023
		TS	AK1
		CA	T5TEMP
		EXTEND
		MP	AMGB7
		TS	AK2
		CA	WTEMP +1
		EXTEND
		MSU	CDUZ
		TS	T5TEMP
		EXTEND
		MP	AMGB5
		ADS	AK1
		CA	T5TEMP
		EXTEND
		MP	AMGB8
		ADS	AK2
		TCF	RESUME		# END PHASE1 OF RCS DAP

WRTN17		EXTEND			# GET SET TO COMPUTE TOTAL ASTRONAUT
		DCA	CPHIX +1	# ATTITUDE ERROR WRT N17 BY PICKING UP
		DXCH	WTEMP		# THE THREE COMPONENTS OF N17
		CA	CPHIX
		TCF	GETAKS

# Page 1024 (empty page)
