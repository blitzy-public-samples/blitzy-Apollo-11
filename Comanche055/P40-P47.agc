# Copyright:	Public domain.
# Filename:	P40-P47.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	684-736
# Mod history:	2009-05-11 RSB	Adapted from the Colossus249/ file
#				of the same name, using Comanche055 page
#				images.
#		2009-05-20 RSB	In S20.1, a DMP DDV was corrected to DMPR DDV.
#		2009-05-22 RSB	In BESTTRIM, TC PACTOFF corrected to
#				TS PACTOFF.
#		2009-05-23 RSB	Prior to the 2CADR at T5IDLDAP, added an
#				SBANK.
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

# Page 684
# PROGRAM DESCRIPTION ** P40CSM **

; ============================================================================
; FILE: P40-P47.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth
;
; TL;DR: Service Propulsion System (SPS) and RCS burn programs managing main
;        engine firing sequences. P40/P41 control SPS and RCS burns respectively.
;        Implements complete burn sequencing: ullage motor firing for propellant
;        settling, engine ignition verification, thrust monitoring, and precise
;        cutoff. Critical for Apollo 11 TLI (July 16), LOI (July 19), and TEI
;        (July 21) maneuvers - the mission's most critical engine burns.
;
; COMMENT-ONLY READERS: These programs controlled the large rocket engine burns
;        that sent Apollo to the Moon, put it in lunar orbit, and brought it home.
; CODE-ALONG READERS: Study engine sequencing logic, ullage control, thrust
;        vector integration, navigation state extrapolation during burns.
; ============================================================================

		EBANK=	DAPDATR1
		BANK	31
		SETLOC	P40S
		BANK

		COUNT	24/P40

; ============================================================================
; P40CSM - SERVICE PROPULSION SYSTEM (SPS) BURN PROGRAM ENTRY
;
; This is the entry point for P40, the primary program used for major engine
; burns during the Apollo 11 mission. P40 controlled the Service Module's large
; SPS engine for critical maneuvers:
; - Translunar Injection (TLI) on July 16, 1969 - sending Apollo toward the Moon
; - Lunar Orbit Insertion (LOI) on July 19, 1969 - braking into lunar orbit
; - Trans-Earth Injection (TEI) on July 21, 1969 - returning home from the Moon
;
; The program performs complete burn sequencing from pre-ignition setup through
; engine cutoff, handling thrust vector control, velocity-to-be-gained monitoring,
; and navigation state extrapolation during powered flight.
; ============================================================================

P40CSM		TC	DOWNFLAG
		ADRES	ENG2FLAG

		TC	INTPRET
;
; Steering configuration: Check if this is an external delta-V burn (flag XDELVFLG).
; For external ΔV burns (ground-computed targeting), zero the steering term CSTEER.
; For internal burns (AGC-computed), use the steering term from ECSTEER.
;
		SLOAD	BOFF
			ECSTEER		# IS THIS AN EXTERNAL DELTA V BURN
			XDELVFLG
			P40S/C		# NO	CSTEER = ECSTEER
		DLOAD			# YES	CSTEER = ZERO
			HI6ZEROS
P40S/C		STODL	CSTEER
;
; Thrust magnitude setup: For P40, the SPS engine thrust is 20,000 pounds-force.
; This value (FENG) is stored in F for use throughout the burn calculations.
; P41 (RCS burn program) enters at P40S/F with its own thrust value.
;
			FENG		# SET UP THRUST FOR P40 20,000 LBS
P40S/F		STODL	F		# P41 ENTERS HERE
;
; Time of Ignition (TIG) initialization: Store the planned TIG as NOMTIG (nominal TIG).
; This preserves the original ignition time even if the actual TIG gets adjusted
; during navigation updates (by routine P40S/SV).
;
			TIG		# ORIGINAL TIG MAY BE SLIPPED BY P40S/SV
		STORE	NOMTIG		# SET ORIGINAL TIME OF IGNITION FOR S40.9

		EXIT
;
; Inertial Measurement Unit (IMU) readiness check: Before any burn, verify that
; the spacecraft's gyroscopes and accelerometers are functioning properly and
; correctly aligned. R02BOTH checks IMU status and prompts crew if issues exist.
;
		TC	BANKCALL
		CADR	R02BOTH		# IMU STATUS CHECK

; ============================================================================
; PREFERRED ATTITUDE COMPUTATION AND MANEUVER
;
; Before engine ignition, the spacecraft must rotate to the correct attitude
; for the burn. This section calculates where the spacecraft should point, then
; commands the autopilot to perform the attitude maneuver.
; ============================================================================

P40PVA		TC	INTPRET
;
; S40.1: Compute velocity-to-be-gained (VGTIG) and unit thrust vector (UT).
; VGTIG represents the desired velocity change from the burn, computed from
; targeting data or Lambert solutions. UT is the direction the engine must point.
;
		CALL
			S40.1		# COMPUTE VGTIG,UT
;
; S40.2,3: Calculate preferred gimbal angles from the thrust direction.
; These angles tell the spacecraft what pitch, yaw, and roll attitude achieves
; the required engine pointing direction. Results stored for R60 autopilot.
;
		CALL
			S40.2,3		# COMPUTE PREFERRED ATTITUDE
		SET	EXIT
			PFRATFLG
;
; Narrow the attitude control deadband for precise maneuver: During critical
; engine burns, tighter attitude control is needed. SETMINDB configures the
; autopilot for ±0.5 degree deadband (vs. ±5 degrees for coasting flight).
;
P40SXTY		TCR	SETMINDB -1	# NARROW DEADBAND FOR MANEUVER (EBANK6)
		RELINT
;
; R60CSM: Execute the attitude maneuver. The autopilot rotates the spacecraft
; to the preferred attitude, using RCS thrusters. Crew monitors FDAI (Flight
; Director Attitude Indicator) needles during the rotation. Program waits here
; until the maneuver completes and attitude is within acceptable error.
;
		TC	BANKCALL
		CADR	R60CSM		# ATTITUDE MANEUVER
;
; Initialize velocity-to-be-gained update cycle counter: Set NBRCYCLS to -1
; to indicate the first cycle of UPDATEVG (velocity-to-be-gained monitoring
; during the burn). This counter tracks navigation updates during powered flight.
;
		CS	ONE		# FOR UPDATEVG
		TS	NBRCYCLS
;
; Enable the countdown clock task: Set TIMRFLAG to allow CLOKTASK to begin
; displaying time-to-ignition (TIG) countdown on the DSKY. Crew monitors this
; display as ignition approaches, with updates showing TIG in minutes:seconds.
;
		TC	UPFLAG
		ADRES	TIMRFLAG	# ALLOW CLOCKTASK

		TC	P41/P40
		TC	P41/DSP		# P41

; ============================================================================
; BURN DISPLAY INITIALIZATION
;
; Set up the DSKY displays that crew will monitor during the burn countdown
; and execution. V06N40 displays velocity-to-be-gained magnitude (R2) and
; time-to-ignition (R3), updating continuously until engine cutoff.
; ============================================================================

P40TTOG		CAF	V06N40		# INITIALIZE FOR CLOCKTASK WHICH IS CALLED
# Page 685
		TS	NVWORD1		# BELOW

		TC	INTPRET
;
; Compute velocity-to-be-gained magnitude for display: Take absolute value
; of VGTIG vector to get scalar magnitude in feet per second. This appears
; on the DSKY as R2, showing crew how much velocity change the burn will provide.
; For Apollo 11's LOI burn, this was approximately 2,900 ft/sec.
;
		VLOAD	ABVAL		# FOR R2
			VGTIG
		STODL	VGDISP
;
; Initialize total delta-V accumulator: Zero DVTOTAL before the burn begins.
; During engine firing, DVTOTAL accumulates the actual velocity change achieved,
; allowing comparison with the desired VGTIG.
;
			HI6ZEROS
		STORE	DVTOTAL
		EXIT

		EXTEND
;
; Configure steering update exit: Set AVEGEXIT to point to STEERADS routine.
; After average-G integration during the burn, control transfers here to update
; steering commands based on current velocity-to-be-gained residuals.
;
		DCA	STEERADS	# SET FOR UPDATEVG AND TEST FOR STEERING
		DXCH	AVEGEXIT	# AFTER AVERAGE G

; ============================================================================
; GIMBAL TRIM AND ENGINE TEST OPTION
;
; Before the burn, crew can optionally command gimbal trim (V33) or full gimbal
; test (V34) via DSKY. Gimbal trim centers the engine bell; gimbal test exercises
; the full range of thrust vector control actuators to verify proper operation.
; For critical burns like LOI, crew typically performed gimbal test to ensure
; the engine could gimbal properly before committing to the burn.
; ============================================================================

P40GMB		CAF	P40CKLS2	# (4.1 PROTECTION)
		TC	BANKCALL
;
; GOPERF1: Display "PLEASE PERFORM" message, awaiting crew verb entry.
; Crew responds with V33 (gimbal trim only) or V34 (proceed without test).
; This decision point gives crew manual control over pre-burn checkout sequence.
;
		CADR	GOPERF1
		TCF	POST41		# V34
		TCF	TST,TRIM	# V33
;
; Gimbal trim marker: Set MRKRTMP = -1 to indicate trim-only operation.
; This flag determines the delay duration: 5 seconds for trim-only vs.
; 18 seconds for full gimbal test. TRIMONLY entry bypasses the test logic.
;
TRIMONLY	CS	BIT1		# SET MRKRTEMP FOR GIMBAL TRIM (-1)
	+1	TS	MRKRTMP		# ENTRY FROM TST,TRIM

;
; Restart protection setup: Initialize CNTR = 0 for normal entry into S40.6
; (pre-ignition sequence). If computer restart occurs during burn preparation,
; CNTR = 1 signals restart recovery logic to resume from appropriate point.
;
		CAF	ZERO		# SET CNTR	+0 FOR RESTART LOGIC IN S40.6
		TS	CNTR		#	+0 SAYS NORMAL ENTRY
					#	+1 (PRE40.6) SAYS RESTART ENTRY

;
; Schedule S40.6 task on WAITLIST: S40.6 contains pre-ignition sequencing that
; begins 35 seconds before TIG. Scheduled 1 centisecond from now to allow
; current job to complete before S40.6 initiates ullage and ignition countdown.
;
		CAF	ONE
		TC	WAITLIST
		EBANK=	DAPDATR1
		2CADR	S40.6

;
; Gimbal test/trim delay: If MRKRTMP is positive (gimbal test), delay 18 seconds
; for full actuator stroke test. If negative (trim only), delay 5 seconds for
; gimbal centering. This timing allows mechanical gimbal movement to complete
; and stabilize before proceeding to pre-ignition sequence.
;
		CCS	MRKRTMP		# TEST TO FIND TIME TO WAIT FOR GIMBAL TEST
		CAF	18SEC		# PLUS, DELAY FOR 18 SECONDS
		TCF	+2		# HOLE
		CAF	5SEC		# DELAY FOR TRIM ONLY TASK
		TC	BANKCALL
		CADR	DELAYJOB
;
; Phase change setup for parallel tasks: Set up two restart protection groups:
; - Group 6.2: PRE40.6 (pre-ignition sequence) and CLOKTASK (countdown display)
; - Group 4.23: P40S/SV (state vector integration)
; This ensures proper restart recovery if computer restart occurs during setup.
;
		TC	2PHSCHNG
		OCT	40026		# 6.2 = PRE40.6(-0CS), CLOKTASK(100CS)
		OCT	00234		# 4.23 = P40S/SV (PRIO12)
;
; P40S/RS: Schedule countdown clock display task. CLOKTASK updates the DSKY
; time-to-ignition display every 100 centiseconds (1 second), showing crew
; the countdown to engine ignition in MM:SS format.
;
P40S/RS		CAF	ONE
		TC	WAITLIST	# P41/SDP
		EBANK=	TIG
		2CADR	CLOKTASK

		RELINT

; ============================================================================
; TRANSITION: From gimbal trim to navigation state vector update
;
; After verifying engine gimbal operation, the program now updates the spacecraft's
; position and velocity (state vector) by integrating forward to TIG minus 30 seconds.
; This ensures navigation computations use the most current orbital data, accounting
; for any drift since the last update. Critical for accurate burn execution.
; ============================================================================

; ============================================================================
; P40S/SV - STATE VECTOR INTEGRATION TO TIG-30 SECONDS
;
; Integrates the spacecraft's position and velocity from current time to 30 seconds
; before ignition. This provides an accurate starting point for computing the burn
; trajectory and monitoring velocity-to-be-gained during engine firing.
;
; For Apollo 11's LOI burn on July 19, 1969, this integration accounted for the
; spacecraft's motion along its approach trajectory, ensuring the burn would
; execute at precisely the planned point in the orbit.
; ============================================================================

P40S/SV		TCR	E7SETTER	# JOB, 4.23 PRETECTS, PREO12
		EBANK=	TIG
# Page 686
;
; Compute integration target time: TIG minus 29.96 seconds (approximately 30 seconds
; before ignition). This gives the computer time to complete integration and prepare
; for the burn sequence while maintaining precision in the integration endpoint.
;
		TC	INTPRET
		DLOAD	DSU
			TIG		# Time of ignition
			SEC29.96	# 29.96 seconds constant
		STORE	TDEC1		# Target time for state vector integration

;
; Call MIDTOAV1 subroutine (in ORBITAL_INTEGRATION.agc) to integrate the state
; vector forward from current time to TDEC1 (TIG-30). This uses Encke's method
; for precision orbital integration, accounting for Earth/Moon gravity, solar
; perturbations, and spacecraft trajectory evolution.
;
		CALRB			# RETURN IN BASIC
			MIDTOAV1
		TCF	+2		# Integration successful, continue
		TC	P40SNEWM	# INTEGRATION TIME GREATER THAN ALLOWED

;
; Integration succeeded. Now compute the time when the countdown display should
; blank (go blank 5 seconds before the display reaches zero, for ullage startup).
; Store the delta-time for scheduling TIGBLNK task.
;
P40SET		EXTEND
		DCA	MPAC		# DELTA TIME TO PREREAD (INT.INIT.)
		DXCH	P40TMP		# Save delta-time temporarily
		EXTEND
		DCS	5SECDP		# FOR TIGBLNK
		DAS	P40TMP		# P40TMP now holds time to blank display
		EXTEND
		DCA	P40TMP
		TC	LONGCALL	# Schedule TIGBLNK task to execute at TIG-35
		EBANK=	TIG
		2CADR	TIGBLNK

;
; Set restart protection for TIGBLNK task (Group 4.21). If computer restarts
; during this phase, TIGBLNK will be properly restarted at the computed time.
;
		TC	PHASCHNG
		OCT	20214		# 4.21 = TIGBLNK (P40TMP CS)

		TCF	ENDOFJOB

;
; P40BLNKR: Display cleanup routine called by TIGBLNK when countdown reaches
; blank point. Clears residual display data before ullage motor ignition.
;
P40BLNKR	TC	BANKCALL
		CADR	CLEANDSP	# REMOVE RESIDUE
		TCF	ENDOFJOB

;
; P40SNEWM: Integration time exceeds maximum allowed interval (indicating TIG
; is too far in future for single integration step). Reset TIG to current time
; plus 29.96 seconds and retry integration. This handles cases where burn was
; scheduled far in advance and integration limit was reached.
;
		EBANK=	TIG
P40SNEWM	EXTEND
		DCA	PIPTIME1	# Current time from IMU
		DXCH	TIG		# SET NEW TIG FOR 06 40
		EXTEND
		DCA	SEC29.96
		DAS	TIG		# New TIG = now + 30 seconds
		TCF	P40SET		# FOR LONGCALL OF TIG-30 (OR -35)

; ============================================================================
; TRANSITION: From pre-burn setup to post-burn sequence
;
; After engine cutoff, the program transitions to post-burn activities including
; final velocity change verification, attitude stabilization, and return to
; mission program control. For Apollo 11's major burns, this phase confirmed
; successful orbit insertion and prepared for the next mission phase.
; ============================================================================

; ============================================================================
; POSTBURN - POST-BURN SEQUENCE AND CLEANUP
;
; Entered automatically after SPS engine cutoff or manually via V99N40.
; Displays final ΔV achieved (V16N40) and allows crew to select continuation
; options: proceed to finish (V34), bypass to RCS-only mode, or recycle display.
;
; Comment-only readers: After the rocket engine shuts down, this routine confirms
; the burn was successful and prepares the spacecraft for the next mission phase.
; ============================================================================

		EBANK=	DAPDATR1
POSTBURN	CAF	V16N40		# Display verb 16 noun 40: ΔV achieved
		TC	BANKCALL
		CADR	REFLASH		# Flash display, await crew response
		TCF	POST41		# V34 GO FINISH
		TCF	P40RCS		# PROCEED (or V99N40 direct entry)
		TCF	POSTBURN	# RECYCLE

;
; P40RCS: Transition to RCS-only mode after SPS burn. Used when SPS is bypassed
; or as contingency mode. Switches autopilot interface to use small RCS thrusters
; instead of large SPS engine for any remaining attitude control.
;
P40RCS		EXTEND			# V99N40 ENTERS HERE ON A P40 BYPASS SPS
		DCA	ACADN85		# Address of noun 85 calculation routine
		DXCH	AVEGEXIT	# Set exit from average-G for noun 85
		CAF	2SECS		# WAIT FOR CALCN85 VIA AVEGEXIT
		TC	BANKCALL
# Page 687
		CADR	DELAYJOB	# Delay 2 seconds for calculation completion

;
; P40MINDB: Set minimum deadband for post-burn attitude hold. Tightens attitude
; control tolerances to maintain precise orientation after burn completion.
;
P40MINDB	TCR	SETMINDB -1
		RELINT

;
; TIGNOW: Display final velocity change to crew (V16N85). Shows ΔV components
; in three axes, allowing crew to verify burn performance matched prediction.
; For Apollo 11's LOI burn, this confirmed the spacecraft successfully entered
; lunar orbit with the planned orbital parameters.
;
TIGNOW		TC	PHASCHNG
		OCT	05024		# TYPE C GROUP 4 BELOW FOR NOUN 85
		OCT	20000		# PRIO 20
		CAF	V16N85B		# Verb 16 noun 85: ΔV display (three axes)
		TC	BANKCALL
		CADR	REFLASH
		TCF	POST41		# FINISH P40/P41
		TCF	POST41		# V03 PROCEED WITH REST OF THE CLEAN-UP
		TCF	TIGNOW		# V32 NOT GSOP RESPONSE BUT REDISPLAY N85

;
; POST41: Final cleanup and return to mission program control. Restores normal
; autopilot exit and returns to POO (Program 00, idle state) or next scheduled
; program. Burn sequence is complete.
;
POST41		EXTEND
		DCA	SERVCADR	# Service routine address
		DXCH	AVEGEXIT	# Restore normal average-G exit
		TCF	GOTOPOOH	# Return to POO or next program

; ============================================================================
; AUTOPILOT DEADBAND CONSTANTS AND CONFIGURATION
;
; MINDB/MAXDB define attitude control deadband limits in degrees * 180.
; Deadband is the angular tolerance within which the autopilot does not fire
; thrusters - reducing propellant consumption while maintaining attitude control.
; ============================================================================

MINDB		DEC	46		# Minimum deadband: 46/180 = 0.256 degrees
MAXDB		DEC	455		# Maximum deadband: 455/180 = 2.53 degrees

; ============================================================================
; SETMINDB - SET MINIMUM AUTOPILOT DEADBAND
;
; Configures the digital autopilot (DAP) to use minimum deadband for tight
; attitude control. Called before critical maneuvers (attitude changes) and
; after burns to maintain precise orientation. Must be called with interrupts
; inhibited (INHINT) to ensure atomic update of DAP parameters.
;
; Loads current gimbal angles (CDUX, CDUY, CDUZ) into desired attitude
; (THETADX, THETADY, THETADZ), then sets minimum deadband in ADB register.
; This tells the autopilot to hold current attitude within tight tolerance.
; ============================================================================

		EBANK=	DAPDATR1
	-1	INHINT			# Entry point -1: caller provides INHINT
SETMINDB	CA	CDUX		# ROUTINE FOR SETTING
		TS	THETADX		# THE MINIMUM DEADBAND
		EXTEND			# IN AUTOPILOT
		DCA	CDUY		# Load Y and Z gimbal angles (double precision)
		DXCH	THETADY		# Store as desired attitude Y and Z
		CA	MINDB		# SHOULD BE CALLED UNDER
		TS	ADB		# INTERRUPT INHIBITED - set minimum deadband
		CS	BIT4		# EBANK = E6 - clear bit 4 in DAPDATR1
		MASK	DAPDATR1	# (DAP configuration control)
		TS	DAPDATR1
		TC	Q		# Return to caller

; ============================================================================
; SETMAXDB - SET MAXIMUM AUTOPILOT DEADBAND
;
; Configures the digital autopilot (DAP) to use maximum deadband for coarse
; attitude control. Called during coast phases when tight attitude control
; is not required, allowing larger attitude errors before thruster firing.
; This conserves RCS propellant during long coast periods.
;
; Sets maximum deadband in ADB register and updates DAPDATR1 configuration
; bit. Must be called with interrupts inhibited for atomic update.
; ============================================================================

		EBANK=	DAPDATR1
	-1	INHINT			# Entry point -1: caller provides INHINT
SETMAXDB	CA	MAXDB		# ROUTINE FOR SETTING
		TS	ADB		# THE MAXIMUM DEADBAND IN AUTOPILOT
		CS	DAPDATR1
		MASK	BIT4		# SHOULD BE CALLED UNDER
		ADS	DAPDATR1	# INTERRUPT INHIBITED - set bit 4
		TC	Q		# EBANK = E6 - return to caller

# Page 688

; ============================================================================
; PROGRAM DESCRIPTION ** P41CSM **
;
; P41 is the RCS-ONLY burn program for Command/Service Module. Unlike P40
; which uses the large SPS engine, P41 uses only the small RCS thrusters
; for velocity changes. This is used for minor orbit adjustments, attitude
; maneuvers with velocity change, and as backup if SPS cannot be used.
;
; For Apollo 11, P41 provided capability for small orbital corrections using
; RCS thrusters in 2-jet or 4-jet configurations, conserving SPS propellant
; for major burns (TLI, LOI, TEI).
; ============================================================================

		SETLOC	P40S2
		BANK

		EBANK=	DAPDATR1
		COUNT	24/P41

; ============================================================================
; P41CSM - ENTRY POINT FOR RCS-ONLY BURN PROGRAM
;
; Initializes P41 for RCS thruster burn instead of SPS engine. Sets ENG2FLAG
; to indicate RCS mode, clears CSTEER (no external steering), and computes
; thrust level based on 2-jet or 4-jet configuration selected by crew.
; ============================================================================

P41CSM		TC	UPFLAG
		ADRES	ENG2FLAG	# SET FOR RCS

;\n; Clear external steering (CSTEER = 0) since P41 is internally controlled.\n; RCS burns are typically short and use onboard guidance, not ground-computed
; targeting like external ΔV burns.
;
		TC	INTPRET
		DLOAD
			HI6ZEROS	# FOR P41 CSTEER =0
		STORE	CSTEER

;
; Compute RCS thrust level based on jet configuration. NJETSFLG indicates
; whether crew selected 2-jet mode (1 quad active) or 4-jet mode (2 quads).
; 2-jet mode: FRCS2 thrust (one RCS quad, four thrusters)
; 4-jet mode: FRCS2 + FRCS2 = double thrust (two RCS quads, eight thrusters)
;
		DLOAD	BON
			FRCS2		# 2JET THRUST FOR S40.1
			NJETSFLG
			P40S/F		# NJETS = 1: 2-JET mode, use FRCS2
		DAD	GOTO		# NJETS = 0: 4-JET mode
			FRCS2		# Double thrust: FRCS2 + FRCS2
			P40S/F		# Continue to P40S/F (common thrust setup)

		SETLOC	P40S
		BANK

P41/P40		CS	MODREG
		MASK	ONE		# P41EXITS AT CALL LOC +1
		EXTEND
		BZF	+2		# P41
		INCR	Q		# P40 EXITS AT CALL LOC +2
		TC	Q

TTG/0		CAF	PRIO20		# TASK (4.4 PROTECTS IN P41)
; Apollo 11 reached the critical moment when the burn countdown approached zero.
; The computer now transitions from monitoring mode to active burn execution,
; initiating the TIGNOW task that will control engine ignition sequencing.

		TC	NOVAC		; Schedule new task for immediate execution
		EBANK=	DAPDATR1
		2CADR	TIGNOW		; Task: Begin engine ignition sequence now

; The clock task has completed its countdown. The burn is now underway or complete.
; Disable the timer flag to stop further clock displays until next burn sequence.

P40CLK		TC	DOWNFLAG	; Clear timer flag
		ADRES	TIMRFLAG	; Timer routine no longer needed

		TCF	TASKOVER	; Task complete, return to executive

; ============================================================================
; TRANSITION: From P40 (SPS) to P41 (RCS) Display Handling
;
; P41 uses the smaller RCS thrusters instead of the main SPS engine. The display
; requirements differ because RCS burns are shorter and less critical than major
; SPS maneuvers. This section prepares the appropriate display format showing
; velocity-to-be-gained in body coordinates for crew monitoring.
; ============================================================================

P41/DSP		CAF	V06N85B		# SET UP FOR NONFLASH V 06 N85 BY CLOCKJOB
		TS	NVWORD1		; Store display verb/noun code

; For RCS burns (P41), the crew needs to see velocity-to-be-gained (VG) displayed
; in body coordinates relative to spacecraft orientation, not inertial coordinates.
; This allows them to understand thrust direction in terms of spacecraft attitude.

		TC	INTPRET		; Enter interpreter for vector calculations
# Page 689
		CALL			# COMPUTE
			P40CNV85	#	VGTIG IN CTRL COORDS
		EXIT			; Return to native AGC code
		EXTEND			# DO CONTROL COORD CALCULATION AFTER AVEG
		DCA	ACADN85		; Load address for post-average-G calculation
		DXCH	AVEGEXIT	; Set exit point after average G routine
		TC	2PHSCHNG	; Phase change for restart protection
		OCT	40036		# 6.3=CLOKTASK(100CS)
		OCT	234		# 4.23=P40S/SV(PRIO12)

		TCF	P40S/RS		; Continue to restart/recycle logic
; During an RCS burn, the display must be refreshed to show updated VG values
; as the burn progresses. The crew watches these values decrease toward zero,
; confirming the burn is achieving the desired velocity change.

P41REDSP	CAF	V16N85B		# ENTER FROM P41 SIDE OF TIGAVEG
		TS	NVWORD1		# REDISPLAY NONFLASHING
		CAF	SEC29.96 +1	; Schedule redisplay in ~30 seconds
		TC	WAITLIST	; Add to timer-driven task list
		EBANK=	DAPDATR1
		2CADR	TTG/0		; Task: Update time-to-go display

		CS	BIT3		; Complement bit 3 for phase logic
		TCF	TTGPHS		; Continue to phase change handling
; P40CNV85: Convert velocity-to-be-gained from inertial to body coordinates.
; This transformation allows the crew to understand burn direction relative to
; spacecraft orientation. Critical for manual monitoring during RCS burns where
; attitude may shift during thrust application.

P40CNV85	STQ	SETPD		; Save return address
			QTEMP1		; Store Q register in temp location
			0		; Initialize push-down stack pointer
		VLOAD	PUSH		; Load vector and push onto stack
			VGPREV		# EQUALS VGTIG (TARGETTING INPUT)
		CALL			; Perform coordinate transformation
			S41.1		; Subroutine: Inertial to body coordinates
		STCALL	VGBODY		; Store result in body coordinate VG
			QTEMP1		; Return to caller

; CALCN85: Update velocity-to-be-gained and refresh body coordinate display.
; Called periodically during burn to show crew decreasing VG values.

		EBANK=	DAPDATR1
CALCN85		TC	INTPRET		; Enter interpreter mode
		CALL
			UPDATEVG	# NEW VG, S40.8 (+MAYBE S40.9)
		CALL
			P40CNV85	# COMPUTE VGBODY
		EXIT			; Return to native AGC code
		TC	SERVXT		; Service display and exit
; ============================================================================
; THRUST AND TIMING CONSTANTS
;
; These constants define the physical performance characteristics of the
; spacecraft propulsion systems. The SPS (Service Propulsion System) main engine
; produces 20,500 pounds of thrust - enough to perform major maneuvers like
; translunar injection, lunar orbit insertion, and transearth injection. The
; RCS (Reaction Control System) ullage thrusters are much smaller at ~200 pounds,
; used for propellant settling and attitude control.
; ============================================================================

FENG		2DEC	9.1188544 B-7	# SPS THRUST (20500LBS), SC.AT B+7 NEWT/E4
					; Main engine thrust scaled for AGC computation

FRCS2		2DEC	.087437837 B-7	# RCS ULLAGE (199.6COS10 LBS), SC.AT
					; RCS ullage thrust for propellant settling
					#	B+7 NEWTONS/E+4
; Timing constants for display updates and burn sequence coordination.
; Times scaled in centiseconds (1 CS = 0.01 seconds).

SEC24.96	DEC	2496		; ~25 seconds in centiseconds
SEC29.96	2DEC	2996		; ~30 seconds for display refresh

18SEC		DEC	1800		; 18 seconds in centiseconds
P40CKLS2	OCT	204		; Clock phase code
40CST5		OCT	37730		# 40 CS FOR THE T5 CLOCK
OCT12		=	TEN		; Octal 12 = decimal 10
# Page 690
V1683		VN	1683
V06N85B		VN	0685
V16N85B		VN	1685
V06N40		VN	0640
V16N40		VN	1640
OCT27/24	OCT	27
OCT53		OCT	53
OCT35		OCT	35
		EBANK=	DAPDATR1
T5IDL24		2CADR	T5IDLOC

; Mass flow rate for SPS engine. During a burn, the spacecraft loses approximately
; 63.8 pounds per second of propellant mass. This affects spacecraft mass properties
; and must be accounted for in guidance calculations to maintain thrust accuracy.

3MDOT		DEC	86.6175796 B-16	# 3SEC MASS LOSS (63.8 LBS/SEC), SC.AT
					# B+16 KB/SEC (NOT, EMDOT IS PAD-LOADED,
					# BUT 3MDOT IS NOT A CRITICAL QUANTITY, SO
					# IT CAN REMAIN IN FIXED MEMORY)

; ============================================================================
; GIMBAL TRIM AND TIG DISPLAY BLANKING
;
; As time-to-ignition (TIG) approaches, the DSKY display is blanked to prevent
; crew distraction during the critical final seconds. The gimbal drive system
; is tested and trimmed to ensure engine can be gimbaled for thrust vector control.
; ============================================================================

TST,TRIM	CAF	BIT1		# SET UP FOR GIMB DRIVE TEST AND TRIM (+1)
		TCF	TRIMONLY +1	; Continue to trim-only processing

; At TIG-30 seconds, blank the DSKY and prepare for burn. The crew has completed
; all pre-burn checks. The computer now takes full control of the ignition sequence.

TIGBLNK		CAF	5SEC		# CALL TIGAVEG IN FIVE SEC AT TIG-30
		TC	WAITLIST	; Schedule average-G calculation
		EBANK=	TIG
		2CADR	TIGAVEG		; Task: Compute average G during burn

		CAF	ZERO		# DISABLE HERE, NOT IN P40BLNKR
		TS	NVWORD1		; Clear display verb/noun (blank DSKY)

		CAF	PRIO14		; Schedule high-priority task
		TC	NOVAC		; Create new task
		EBANK=	TIG
		2CADR	P40BLNKR	# DON'T PROTECT -- RESTARTS BLANK DSKY

		CS	OCT37		# 4.37 = TIGAVEG (500CS)
P40TSK		TC	NEWPHASE	; Change phase for restart protection
		OCT	4		; Phase code 4
		TC	TASKOVER	; Task complete, return to executive

		EBANK=	TIG
ACADN83		2CADR	CALCN83

		EBANK=	TIG
SERVCADR	2CADR	SERVEXIT

		EBANK=	DAPDATR1
ACADN85		2CADR	CALCN85

# Page 691
# PROGRAM DESCRIPTION ** P47CSM **

		COUNT	24/P47

		EBANK=	TIG
P47CSM		TC	BANKCALL	# IMU STATUS CHECK
		CADR	R02BOTH
		TC	INTPRET
		CALRB
			MIDTOAV2
		CA	MPAC +1		# DELTA TIME TO RPEREAD (LESS THAN 100
		TS	P40TMP		#	CS, WITH A TPAGREE, INT.INIT.)
		TC	WAITLIST
		EBANK=	TIG
		2CADR	TIGON		# TIGON IS REQUIRED TO MATHCHTAT AND AVEG

		TC	PHASCHNG
		OCT 	40574		# A, 4.57 = TIGON (P40TMP CS)
		TCF	ENDOFJOB

		EBANK=	P40TMP
TIGON		EXTEND
		DCA	ACADN83
		DXCH	AVEGEXIT
		CAF	PRIO30		# FORCE ZEROING OF N83 BEFORE SERVICER
		TC	NOVAC
		EBANK=	TIG
		2CADR	P47BODY

		CS	BIT2		# 4.2 = PRECHECK (-0CS), P47BODY (PRIO30)
		TCF	TTGPHS

		EBANK=	TIG
CALCN83		TC	INTPRET
		SETPD			# SET UP PUSHLIST FOR S41.1
			0
		VLOAD	VAD
			DELVCTL
			DELVREF
		STORE	DV47TEMP	# FOR COPYCYCLE BELOW
		PUSH	CALL
			S41.1
		STCALL	DELVIMU
			S11.1		# CALC. VI, H, HDOT FOR NOUN 62
		EXIT
		TC	PHASCHNG
		OCT	10035
# Page 692
		CAF	FIVE
		TC	GENTRAN
		ADRES	DV47TEMP
		ADRES	DELVCTL

		TC	SERVXT
P47BODY		TC	INTPRET
		VLOAD
			HI6ZEROS
		STORE	DELVIMU		# CLEAR DISPLAY AND ACCUMULATOR STORAGE
		STORE	DELVCTL		# UPON INITIATION OR ENTER RESPONSE
		EXIT
P47BOD		CAF	PRIO15		# LOWER PRIO THAN CALCN83 (20)
		TC	PRIOCHNG	#	TO PREVENT INTERRUPTION OF CALCN83
		TC	PHASCHNG
		OCT	05024		# TYPE C GROUP 4 BELOW FOR NOUN 83
		OCT	15000		# PRIO 15
P47/DSP		CAF	V1683
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	GOTOPOOH
		TCF	P47BODY		# RECYCLE -- CLEAR ACCUMULATED VELOCITY

# Page 693
; ============================================================================
; TRANSITION: From display and crew monitoring to burn countdown sequence
;
; The crew has confirmed the burn parameters displayed on the DSKY. With 30
; seconds remaining until Time of Ignition (TIG), the AGC begins the automated
; countdown sequence. This is the final checkpoint before engine ignition.
; During Apollo 11's critical burns (TLI on July 16, LOI on July 19, TEI on
; July 21), the crew monitored these final seconds while the computer managed
; the precise timing and sequencing required for successful engine start.
; ============================================================================

# ROUTINE ** TIG-30 ** DESCRIPTION

		EBANK=	TIG
		COUNT	24/P40

; TIG-30 (Time of Ignition minus 30 seconds)
; This routine executes 30 seconds before engine ignition. It performs final
; burn parameter updates and schedules the TIG-5 task for 25 seconds later.
; During the actual Apollo 11 mission, this countdown timing was critical for
; synchronizing navigation state updates with engine start.

TIGAVEG		TC	P41/P40		# TASK (4.37 PROTECTS)
		TCF	P41REDSP

; Unblank the DSKY display to ensure crew can monitor burn countdown.
; V06N40 displays time-to-go, velocity-to-be-gained magnitude, and ΔV total.
		CAF	V06N40		# UNBLANK DISPLAY
		TS	NVWORD1

; Schedule TIG-5 task to execute in 24.96 seconds (2496 centiseconds).
; This provides 5-second final preparation window before ignition.
		CAF	SEC24.96
		TC	WAITLIST
		EBANK=	TIG
		2CADR	TIG-5

; Update restart protection phase to 4.6 for TIG-5 sequence.
; Phase management ensures proper restart recovery if power transient occurs.
		CS	SIX		# 4.6 = TIG-5 (2496CS), PRECHECK (-0CS)
TTGPHS		TC	NEWPHASE	# ENTRY FROM P41REDSP (P41) WITH A=-4, OR
		OCT	4		#       FROM TIGON    (P47) WITH A=-1

; Check if navigation servicer routine has already been restarted.
; Prevents multiple concurrent servicer tasks during burn sequence.
PRECHECK	CCS	PHASE5		# HAS SERVICER BEEN RESTARTED
		TCF	TASKOVER	# YES, DON'T START ANOTHER ONE
		TC	POSTJUMP
		CADR	PREREAD

# Page 694
; ============================================================================
; TIG-5 (Time of Ignition minus 5 seconds)
;
; With only 5 seconds until ignition, this routine performs the final burn
; preparation. It schedules TIG-0 for immediate engine start, flashes V99 on
; the DSKY to request crew proceed authorization, and initiates final burn
; time calculation (S40.13). This is the last crew decision point - they must
; press PROCEED to authorize ignition, or TERMINATE to abort the burn.
;
; Historical note: During Apollo 11's critical burns, the crew's PROCEED at
; this V99 flash was the final human authorization before the computer took
; full control of engine ignition and thrust vector steering.
; ============================================================================

		EBANK=	TIG
TIG-5		CAF	5SEC
		TC	WAITLIST
		EBANK=	DAPDATR1
		2CADR	TIG-0

; Flash V99 on DSKY requesting crew PROCEED authorization for ignition.
; BIT9 negative (octal 777) triggers the flash sequence.
; Crew must press PROCEED within 5 seconds to authorize burn.
		CS	BIT9		# WILL CAUSE V99 FLASH
		TS	NVWORD1

; Update restart protection phases:
; 4.7 = TIG-0 will execute in 5 seconds (500 centiseconds)
; 3.3 = S40.13 burn time calculation job (priority 20)
		TC	2PHSCHNG
		OCT	40074		# A, 4.7 = TIG-0 (500CS)
		OCT	00033		# A, 3.3 = S40.13 (PRIO20)

; Start S40.13 job to compute final burn time based on current velocity-to-
; be-gained (VG) and expected thrust acceleration. This calculation runs
; concurrently with crew authorization to have precise cutoff time ready.
		CAF	PRIO20
		TC	FINDVAC
		EBANK=	TGO
		2CADR	S40.13

		TCF	TASKOVER

# Page 695
; ============================================================================
; TIG-0 (Time of Ignition - Zero seconds) and IGNITION
;
; This is the moment of truth. TIG-0 executes at the exact planned ignition
; time. It checks that the crew has pressed PROCEED on the V99 flash, then
; immediately transfers to the IGNITION routine which commands engine start.
;
; IGNITION performs the actual engine-on command by setting the ENGONFLG and
; writing the engine-on discrete signal to the spacecraft's output channels.
; For the SPS (Service Propulsion System), this energizes the engine valves
; allowing hypergolic propellants (nitrogen tetroxide and Aerozine 50) to
; combine and ignite on contact. The SPS produces 20,500 pounds of thrust.
;
; Historical context: Every major Apollo 11 maneuver - Translunar Injection
; (TLI) to leave Earth orbit, Lunar Orbit Insertion (LOI) to capture into
; lunar orbit, and Transearth Injection (TEI) to return home - depended on
; this code executing flawlessly at the precise millisecond.
; ============================================================================

		EBANK=	DAPDATR1	# TASK, 4.7 PHASE, OR 4.77 (-0CS) IN R40
TIG-0		CS	FLAGWRD7	# SET IGN FLAG
		MASK	BIT13
		ADS	FLAGWRD7

; Check ASTNFLAG (astronaut flag) to verify crew pressed PROCEED on V99.
; If PROCEED not yet received, wait (BZF TASKOVER) rather than ignite.
; This ensures crew authorization before engine start.
		CAE	FLAGWRD7	# CHECK ASTN FLAG FOR V99 RESPONSE
		MASK	BIT12
		EXTEND
		BZF	TASKOVER	# WAIT FOR V99P

; Crew has authorized ignition. Clear V99 flash from display in case of
; restart during the V99 sequence, then proceed to engine ignition.
		CAF	V06N40		# CLEAR THE V99 (IN CASE OF A RESTART
		TS	NVWORD1		#	DURING THE V99 SEQUENCE)

; Update phase to 4.61 for IGNITION routine protection during restarts.
; V99PJOB (crew response handler) sets up the IGNITION task with this phase.
		TC	PHASCHNG	# V99P HAS COME ALREADY, DO IGNITION NOW
		OCT	00614		# A, 4.61 = IGNITION (-0CS) TBASE OLD

; ============================================================================
; IGNITION - Engine Start Sequence
;
; This routine executes at TIG (Time of Ignition) to command engine start.
; It captures current gimbal position for the roll DAP reference, records
; ignition time for navigation integration, sets the engine-on flag, and
; sends the discrete engine-on command to the spacecraft hardware.
; ============================================================================

IGNITION	CAE	CDUX		# SAVE FOR ROLL DAP REFERENCE OGAD
		TS	OGAD		#	V99PJOB (CLOCKJOB) SETS UP IGNITION
		EXTEND			# 	TASK (4.61 PROTECTION)
		DCA	TIME2		#	FOR RESTARTS
		DXCH	TEVENT

; Set ENGONFLG (Engine On Flag) to indicate powered flight in progress.
; This flag coordinates between guidance, navigation, and control systems
; during the burn. Navigation uses it to switch to powered flight integration.
		CS	FLAGWRD5	# SET ENGONFLG
		MASK	BIT7
		ADS	FLAGWRD5

; Command engine ignition by writing BIT13 to DSALMOUT (output channel 11).
; For SPS burns, this discrete signal opens propellant valves. The hypergolic
; propellants ignite on contact, producing 20,500 lbs thrust within ~1 second.
; For Apollo 11: TLI burn (357 seconds), LOI burn (357 seconds), TEI burn (151s).
SPSON		CAF	BIT13		# TURN ON SPS ENGINE
		EXTEND
		WOR	DSALMOUT

; ============================================================================
; IMPULSE MODE CHECK AND TVC PREPARATION
;
; The spacecraft has reached ignition (TIG-0). Now the guidance computer
; determines whether this is an "impulsive" burn (so short that steering
; corrections are unnecessary) or a "non-impulsive" burn requiring continuous
; thrust vector control. Short RCS burns are impulsive. Long SPS burns like
; Apollo 11's TLI, LOI, and TEI require continuous steering as the trajectory
; evolves during the multi-minute burn.
; ============================================================================

IMPULCHK	CAF	BIT9		# CHECK FOR IMPULSIVE BURN
		MASK	FLAGWRD2
		; Test IMPULSW flag (bit 9 of FLAGWRD2).
		; Impulsive burns skip steering and use fixed attitude throughout.
		; Non-impulsive burns require continuous guidance updates and TVC.
		
		CCS	A
		TCF	IMPLBURN	# IMPULSIVE
		; Branch to IMPLBURN for short, fixed-attitude burns.
		
		CS	FLAGWRD6	# NON-IMPULSIVE, SET STRULLSW FOR STEERULL
		MASK	BIT13
		ADS	FLAGWRD6
		; Set STRULLSW flag (bit 13 of FLAGWRD6) for non-impulsive burns.
		; This flag enables steering logic and schedules ullage motor shutoff.
		; During Apollo 11's major SPS burns, this path was taken to enable
		; continuous cross-product steering throughout the burn.

; ============================================================================
; PREPARE FOR THRUST VECTOR CONTROL (PREPTVC)
;
; The SPS engine is about to ignite. This routine prepares the guidance
; computer to take over attitude control from the RCS thrusters and hand it
; to the TVC (Thrust Vector Control) system, which gimbals the main engine
; to steer the spacecraft. During Apollo 11's critical burns (TLI, LOI, TEI),
; this transition from RCS to TVC was essential for precise trajectory control.
; ============================================================================

PREPTVC		CS	OCT60000	# RESET T5 BITS
		MASK	FLAGWRD6
		TS	FLAGWRD6
		; Clear T5 control bits (bits 15,14) in FLAGWRD6.
		; These bits control which DAP (Digital AutoPilot) is active.
		; Clearing them prepares for transition from RCS DAP to TVC DAP.

		EXTEND			# KILL RCS
		DCA	T5IDL24
		DXCH	T5LOC
		; Disable RCS DAP by setting T5LOC to idle state.
		; T5 is the task that drives the RCS thrusters for attitude control.
		; Must shut down RCS before TVC takes over to prevent conflicting
		; control commands between thrusters and engine gimbal.

		CS	THREE		# 4.3 = DOTVCON (40CS)
		TC	NEWPHASE
		OCT	4
		; Set restart phase 4.3 = DOTVCON with 40 centisecond timeout.
		; If power failure occurs, restart protection will resume at DOTVCON
		; after 0.4 seconds. This ensures the burn sequence continues even if
		; the AGC momentarily loses power during engine ignition.

# Page 696
		TC	FIXDELAY
		DEC	40		# 0.4 SECOND DELAY FOR THRUST BUILDUP
		; Wait 40 centiseconds (0.4 seconds) for engine thrust to build up.
		; The SPS engine doesn't reach full thrust instantly - this delay
		; allows pressure to stabilize in the combustion chamber before TVC
		; attempts to gimbal the engine. Gimbaling during thrust buildup could
		; cause mechanical damage or unstable control.

; ============================================================================
; ACTIVATE THRUST VECTOR CONTROL (DOTVCON)
;
; Engine ignition complete. Thrust is building. Now hand over attitude control
; to the TVC system which will gimbal the SPS engine to steer the spacecraft.
; The transition from RCS thrusters to engine gimbaling must be smooth to
; avoid attitude disturbances. During Apollo 11's translunar injection burn,
; this routine activated TVC to maintain precise trajectory control throughout
; the 5-minute 48-second burn that sent the spacecraft toward the Moon.
; ============================================================================

DOTVCON		CS	BIT1		# SET TVCPHASE = TVCDAPON CALL (FRESHDAP)
		TS	TVCPHASE
		; Set TVCPHASE = -1 to indicate TVC initialization required.
		; Next TVC cycle will call TVCDAPON to establish fresh TVC DAP state.
		
		CAF	ZERO		# SET TVCEXECUTIVE PHASE
		TS	TVCEXPHS
		; Initialize TVC executive phase counter to zero.
		; TVCEXPHS tracks which phase of TVC processing is active during
		; each guidance cycle (gimbal commands, steering updates, etc.).
		
		CS	OCT60000	# SET T5 BITS TO INDICATE TVC TAKEOVER ....
		MASK	FLAGWRD6	#	BITS 15,14 = 10
		AD	BIT15
		TS	FLAGWRD6
		; Set T5 control bits to 10 (binary) in FLAGWRD6.
		; Bits 15,14 = 10 indicates TVC DAP is now in control.
		; The T5 task will now execute TVC logic instead of RCS logic.

		CAF	THREE		# 6.3 = CLOKTASK (100CS), DROPPING PRE40.6
		TS	L		#	WHICH IS HANDLED NOW BY REDOTVC
		COM
		DXCH	-PHASE6
		; Set phase 6 restart protection to CLOKTASK (100 centiseconds).
		; Drops PRE40.6 phase since REDOTVC now handles those functions.
		; If restart occurs during burn, will resume at CLOKTASK.

		EXTEND			# STORE RCS ATTITUDE ERRORS FOR USE IN
		DCS	ERRORY		# INITIALIZING TVC ATTITUDE ERRORS
		DXCH	ERRBTMP
		; Save current attitude errors from RCS DAP (ERRORY, ERRORZ).
		; TVC will use these as initial conditions to ensure smooth
		; transition. Without this, TVC would start with zero error and
		; might make unnecessary attitude corrections.

		CS	FIVE		# 4.5 = DOSTRULL (160 CS)
		TC	NEWPHASE
		OCT	4
		; Set restart phase 4.5 = DOSTRULL with 160 centisecond timeout.
		; Restart protection now covers the upcoming ullage shutoff sequence.

		CAF	POSMAX		# SET TIME5 FOR STARTING RIGHT AWAY
		TS	TIME5
		; Set TIME5 to maximum positive value to trigger immediate execution.
		; Next T5RUPT interrupt will immediately start TVC processing.
		
		EXTEND
		DCA	TVCON2C		# (TVCDAPON)
		DXCH	T5LOC		# (KILLS RCS DAP)
		; Install TVC DAP entry point (TVCDAPON) into T5LOC.
		; T5 task now calls TVC logic on every interrupt cycle.
		; This final step completes the transition from RCS to TVC control.
		; The SPS engine is now gimbaling to maintain attitude.

		TC	FIXDELAY	# 0.4 + 1.6 = 2.0 SEC FOR ULLAGE-OFF AND
		DEC	160		# 	STEERING (IF NON-IMPULSIVE)
		; Wait 160 centiseconds (1.6 seconds) for next phase.
		; Total delay from PREPTVC: 0.4 + 1.6 = 2.0 seconds after ignition.
		; This allows thrust to stabilize and TVC to establish control before
		; ullage motors shut off and steering begins (if non-impulsive burn).

		; ============================================================================
		; DOSTRULL - ULLAGE MOTOR SHUTOFF AND STEERING ACTIVATION
		;
		; For non-impulsive burns, this routine performs two critical final actions:
		; 1) Activates main guidance steering (STEERULL sets STEERSW flag)
		; 2) Shuts off RCS ullage jets (ULAGEOFF zeros channel 5)
		;
		; For impulsive burns, only ullage shutoff occurs (steering already active).
		; The STRULLSW flag (FLAGWRD6 bit 13) determines which path to follow.
		;
		; After ullage shutoff, restart protection Group 4 is terminated since the
		; critical engine ignition sequence is complete. The burn now continues
		; under TVC control with guidance steering active.
		; ============================================================================

DOSTRULL	CAF	BIT13		# CHECK STRULLSW FOR IMPULSIVE BURN
		MASK	FLAGWRD6
		CCS	A
		TCR	STEERULL	# NON-IMPULSIVE, STEERING AND ULLAGE OFF
		TCR	ULAGEOFF	# ULLAGE OFF (ONLY, OR AGAIN)

		EXTEND
		DCA	NEG0		# KILL GROUP 4 (DP NEG0 = -0,+0)
		DXCH	-PHASE4

ENDIGN		TCF	TASKOVER

		; STEERULL - Enable guidance steering for non-impulsive burns.
		; Sets STEERSW flag (FLAGWRD2 bit 11) to activate the steering loop.
		; The guidance computer now uses computed thrust direction angles to
		; command the TVC system, steering the spacecraft toward the target.

STEERULL	CS	FLAGWRD2	# SET STEERSW
		MASK	BIT11
		ADS	FLAGWRD2

# Page 697

		; ULAGEOFF - Shut off RCS ullage motors by zeroing channel 5.
		; Ullage jets have served their purpose of settling propellant aft in the
		; tanks during SPS ignition. Main engine thrust now maintains propellant
		; position. Shutting off ullage conserves RCS propellant for later use.

ULAGEOFF	CAF	ZERO
		EXTEND
		WRITE	CHAN5		# ZERO CHANNEL 5
		TC	Q

		; ============================================================================
		; IMPLBURN - IMPULSIVE BURN HANDLING
		;
		; For impulsive (very short duration) burns, the full 2-second delay and
		; ullage-off sequencing is bypassed. Instead, this routine:
		;
		; 1) Resets STRULLSW (clears FLAGWRD6 bit 13) to prevent ullage-off call
		; 2) Sets up ENGINOFF task to shut down engine after burn duration (TGO)
		; 3) Prepares for immediate transition to PREPTVC and TVC DAP takeover
		;
		; Impulsive burns are typically very short (<6 seconds) midcourse corrections
		; where the simplified sequence reduces computational overhead and allows
		; faster response. The engine will fire for the computed TGO duration then
		; automatically shut off via the ENGINOFF task scheduled below.
		; ============================================================================

IMPLBURN	CS	BIT13		# RESET STRULLSW (COULD BE AN IMPULSIVE
		MASK	FLAGWRD6	#	ENGINE FAIL)
		TS	FLAGWRD6

		TCR	E7SETTER

		; Prepare TIG display for impulsive burn countdown (CLOCKTASK will update).
		; Load TGO (time-to-go) into TIG display location and add current time.
		; This allows the crew to monitor burn completion via the display.

		EBANK=	TIG
		EXTEND			# PREPARE FOR R1 OF V06N40 (CLOCKTASK)
		DCA	TGO
		DXCH	TIG
		EXTEND
		DCA	TIME2
		DAS	TIG

		; Set up restart phase 3.15 = ENGINOFF with variable delta-time = TGO+1 cs.
		; This schedules the engine shutdown task to execute after burn completes.
		; Also set up immediate continuation task IMPLCONT with -0 cs delay.

		TC	2PHSCHNG
		OCT	40153		# A, 3.15 = ENGINOFF (TGO+1) .... NOT GROUP
		OCT	07014		# C, DELTAT NEXT, TASK BELOW, IN
		DEC	-0		# -0 CS
		EBANK=	DAPDATR1
		2CADR	IMPLCONT

		; Schedule ENGINOFF task on WAITLIST to execute TGO+1 centiseconds from now.
		; The +1 cs margin ensures complete burn before shutdown command.

		CAE	TGO +1		# (TPAGREE IN S40.13, LESS THAN 600CS)
		TC	WAITLIST
		EBANK=	TGO
		2CADR	ENGINOFF

		; IMPLCONT - Continue impulsive burn setup after ENGINOFF is scheduled.
		; Reset IMPULSW flag since impulsive burn logic is complete.
		; Initialize V97VCNTR for mass accounting during the burn.

IMPLCONT	CS	BIT9		# RESET IMPULSW, ENGINOFF IS NOW SET UP
		MASK	FLAGWRD2
		TS	FLAGWRD2

		TCR	E6SETTER
		EBANK=	DAPDATR1

		CAF	ZERO		# SET UP V97VCNTR IN CASE ENGINOFF (MASS-=
		TS	V97VCNTR	#	BACK) ARRIVES BEFORE TVCDAPON

		; Branch to PREPTVC to continue with TVC DAP preparation (bypassing
		; the 2-second ullage delay used for non-impulsive burns).

		TCF	PREPTVC

		; ============================================================================
		; ENGINOFF - ENGINE SHUTDOWN TASK (SCHEDULED BY WAITLIST)
		;
		; This task executes at TGO+1 centiseconds after burn ignition, providing
		; the precise engine cutoff at the end of the burn. For Apollo 11, this
		; routine shut down the SPS engine after major maneuvers (TLI, LOI, TEI).
		;
		; Sequence:
		; 1) Save current spacecraft mass (CSMMASS) for mass accounting
		; 2) Kill Group 3 restart protection (burn sequence complete)
		; 3) Schedule immediate DOSPSOFF execution to command engine shutdown
		;
		; The mass saved here will be used by MASSBACK calculations to compute
		; propellant consumption during the burn, updating spacecraft inertial
		; properties for subsequent navigation and attitude control.
		; ============================================================================

		EBANK=	TGO		# E7 FORCED BY 3.15SPOT VARIABLE DELTA-T
ENGINOFF	TCR	E6SETTER	# TASK, 3.15 PHASE (TGO+1 CS)	GET E6
		EBANK=	DAPDATR1
		CAE	CSMMASS
		TS	MASSTMP		# COPYCYCLE FOR MASSBACK
# Page 698
		TC	2PHSCHNG
		OCT	00003		# KILL GROUP 3 PROTECTION OF ENGINOFF, DO
		OCT	40634		# A, 4.63 = DOSPSOFF (-0CS)

		; DOSPSOFF - Execute SPS engine shutdown immediately (0 cs delay).
		; Calls SPSOFF subroutine to command engine off, update mass properties,
		; and establish the engine shutdown event time (TEVENT) for navigation.

DOSPSOFF	TCR	SPSOFF		# SHUTDOWN SPS, MASS UPDATES, ETC.
		CS	OCT27/24	# (OCTAL 27)
		TC	NEWPHASE
		OCT	4		# 4.27 = DOTVCRCS (250 CS)

		; Delay 250 centiseconds (2.5 seconds) to allow SPS engine tailoff.
		; The SPS engine doesn't shut off instantly - propellant flow continues
		; briefly after shutdown command. This delay ensures complete thrust
		; decay before switching from TVC DAP to RCS DAP control.

		TC	FIXDELAY	# 2.5 SECOND DELAY FOR SPS TAILOFF
		DEC	250

		; ============================================================================
		; DOTVCRCS - TRANSITION FROM TVC TO RCS CONTROL (POST-BURN SEQUENCING)
		;
		; After the SPS engine shuts down and thrust decays, the spacecraft must
		; transition from Thrust Vector Control (TVC) using engine gimbals to
		; Reaction Control System (RCS) thrusters for attitude control.
		;
		; Sequence (critical for smooth transition):
		; 1) Widen attitude deadband to absorb cutoff transients (SETMAXDB)
		; 2) Activate RCS DAP - kills TVCDAPS task, waits 0.6s for TVC termination
		; 3) Update mass properties for new spacecraft configuration (fuel consumed)
		; 4) Wipe out TVC data structures and turn off CLOCKTASK burn timer
		; 5) Schedule POSTBURN job to display final burn results to crew (V16N40)
		;
		; For Apollo 11, this transition occurred after TLI (entering translunar
		; trajectory), LOI (lunar orbit insertion July 19), and TEI (transearth
		; injection July 21) - the mission's three most critical SPS burns.
		; ============================================================================

DOTVCRCS	TCR	SETMAXDB	# WIDE DEADBAND FOR CUTOFF TRANSIENT

		TC	IBNKCALL	# SET UP RCS DAP (KILLS TVCDAPS, SETS T5
		CADR	RCSDAPON	#	BITS, WAITS 0.6SEC FOR TVCEXEC DIE)

		TC	IBNKCALL	# UPDATE WEIGHT/G AND MASS-PROPERTIES FOR
		CADR	MASSPROP	#	RCS DAP STARTUP IN 0.6 SECONDS

		TCR	TVCZAP		# WIPE OUT TVC, TURN OFF CLOKTASK

		TC	PHASCHNG
		OCT	00354		# A, 4.35 = POSTBURN (NOVAC, PRIO12)
		CAF	PRIO12		# SET UP POSTBURN V16N40 JOB
		TC	NOVAC
		EBANK=	DAPDATR1	# (SET MAXDB IN POST41)
		2CADR	POSTBURN

		TCF	TASKOVER

		; ============================================================================
		; SPSOFF - SERVICE PROPULSION SYSTEM ENGINE SHUTDOWN SUBROUTINE
		;
		; Commands the SPS engine to shut down and records the shutdown event time
		; for navigation state vector integration. This is the actual hardware
		; command that stops the engine - a critical operation that must succeed.
		;
		; Actions:
		; 1) Record current TIME2 in TEVENT (engine-off event for navigation)
		; 2) Clear ENGONFLG (FLAGWRD5 bit 7) - marks engine as off for restarts
		; 3) Send engine-off command (clear bit 13 in DSALMOUT channel)
		; 4) Issue S-IVB cutoff command (legacy Apollo hardware interface)
		;
		; Once ENGONFLG is cleared, any restart will know the engine is off and
		; will not attempt thrust-related calculations during recovery.
		; ============================================================================

		EBANK=	DAPDATR1
SPSOFF		EXTEND			# ESTABLISH SPSOFF TEVENT
		DCA	TIME2
		DXCH	TEVENT
		CS	BIT7		# RESET ENGONFLG
		MASK	FLAGWRD5
		TS	FLAGWRD5	# (RESTARTS WILL SHUT DOWN SPS NOW)
		CS	BIT13		# SHUT DOWN SPS ENGINE
		EXTEND
		WAND	DSALMOUT

		; Issue S-IVB cutoff command via CHAN12 bit 14. This is a legacy hardware
		; interface from Apollo program design - the S-IVB is the Saturn V third
		; stage used for translunar injection. While the CSM's own SPS engine
		; has shut down, this backup command ensures any connected stage receives
		; the cutoff signal for redundancy.

		CAF	BIT14		# ISSUE SIV CUTOFF COMMAND
		EXTEND			# FOR POSSIBLE BACK-UP USE
		WOR	CHAN12

		; ============================================================================
		; MASSBACK - RESTORE MASS ACCOUNTING AFTER BURN
		;
		; During the burn, the TVC executive (TVCEXEC) decrements spacecraft mass
		; every 0.5 seconds to track propellant consumption. However, the actual
		; shutdown may occur between these 0.5-second updates, causing the mass
		; to be slightly over-decremented. MASSBACK corrects this error.
		;
		; Algorithm:
		; 1) Use V97VCNTR (saved counter value, accurate even if V97 was active)
		; 2) Multiply by EMDOT (propellant flow rate, scaled at B+3 kg/cs)
		; 3) Multiply by 100 (convert to centiseconds, 1 second = 100 cs)
		; 4) Add to MASSTMP (pre-burn mass minus actual consumption)
		; 5) Store in CSMMASS (updated spacecraft mass)
		;
		; Accuracy: Within 5 centiseconds of propellant flow (approximately
		; 1.44 kg or 0.4 bits), sufficient for post-burn navigation.
		;
		; CRITICAL: Accurate mass is essential for navigation state vector
		; integration, attitude control gains, and rendezvous targeting.
		; ============================================================================

MASSBACK	CAE	V97VCNTR	# RESTORE PART OF PRE-DECREMENTED MASS
					#	V97CNTR = VCNTR UNLESS V97 IS
					#	ACTIVE.  ONLY V97CNTR IS THEN RIGHT.
		EXTEND			# VCNTR COUNTS 1/2-SECONDS IN TVC EXEC
		MP	EMDOT		#	MDOT, SC.AT B+3 KG/CS
		LXCH	A
# Page 699
		EXTEND
		MP	1SEC		# DEC 100
		AD	MASSTMP		# CORRECTION IS ACCURATE TO 5 CS OF FLOW
		TS	CSMMASS		#	(1.44 KG OR 0.4 BITS)

		; Check if TVC trim update is safe to perform. For very short burns
		; (less than 0.4 seconds), the TVC system may not have fully initialized
		; before shutdown. These checks prevent updating trim values with
		; invalid or incomplete data.
		;
		; Two conditions must be satisfied:
		; 1) TVCPHASE must not be -1 (initialization complete)
		; 2) FLAGWRD6 OCT60000 bits must indicate TVC was actually active

		CA	TVCPHASE	# CHECK IF OK FOR TRIM UPDATE
		AD	ONE		#	THESE CHECKS ARE ONLY NEEDED
		EXTEND			#	FOR A LESS THAN 0.4 SEC BURN
		BZF	BTRIMR		# NO.  INITIALIZATION NOT COMPLETE
		CS	FLAGWRD6	# YES, CHECK IF TVC
		MASK	OCT60000
		EXTEND
		BZMF	BTRIMR		# NO, NOT TVC YET

		; ============================================================================
		; BESTTRIM - UPDATE TVC ACTUATOR TRIM OFFSETS WITH POST-BURN VALUES
		;
		; The TVC system uses actuators (hydraulic pistons) to gimbal the SPS
		; engine nozzle for thrust vector control. During the burn, the digital
		; autopilot (DAP) filters the actuator commands to learn the trim offsets
		; needed to maintain proper thrust alignment.
		;
		; DELPBAR and DELYBAR are the filtered pitch and yaw trim values computed
		; during the burn. Storing these as PACTOFF and YACTOFF allows future
		; burns to start with pre-compensated actuator positions, reducing
		; initial attitude disturbances.
		;
		; This trim learning improves burn-to-burn performance as the mission
		; progresses (TLI → LOI → TEI), adapting to propellant depletion effects
		; and vehicle mass distribution changes.
		; ============================================================================

BESTTRIM	CAE	DELPBAR		# UPDATE TRIMS WITH DELFILTER VALUES
		TS	PACTOFF
		CAE	DELYBAR
		TS	YACTOFF
BTRIMR		TC	Q
		EBANK=	DAPDATR1
STEERADS	2CADR	STEERING

.6SECT5		OCT	37703
5SECDP		DEC	0		# MAKE DP 5SEC
5SEC		DEC	500
OCT02202	OCT	02202		# BITS 2, 8, 11 FOR CHANNEL 12 TVC/OPTICS
		EBANK=	DAPDATR1
TVCON2C		2CADR	TVCDAPON

		; ============================================================================
		; TVCZAP - COMPLETE SHUTDOWN AND CLEANUP AFTER ENGINE BURN
		;
		; This routine performs the final hardware and software cleanup after
		; engine shutdown. It runs with interrupts inhibited (INHINT) to ensure
		; atomic execution of critical channel commands and flag updates.
		;
		; Actions performed:
		; 1) Disable TVC and optics error controls via CHAN12 (OCT02202 mask clears
		;    bits 2, 8, 11: TVC enable, optics error control, optics DAC control)
		; 2) Re-enable optics DAC for normal operation (WAND clears specific bits)
		; 3) Enable T4RUPT optics monitoring by clearing OPTIND bit 1
		;    (permits optics-zero calibration but not optics-drive commands)
		; 4) Clear NVWORD1 display register in case CLOCKJOB is waiting
		; 5) Stop CLOKTASK by clearing TIMRFLAG (bit 11 of FLAGWRD7)
		;
		; This ensures clean transition from powered flight back to coast phase
		; operations, with optics subsystem restored for navigation sightings.
		; ============================================================================

	-1	INHINT
TVCZAP		CS	OCT02202	# DISABLE TVC AND OPT ERR CNTRLS, REENGAGE
		EXTEND			#	OPTICS DAC
		WAND	CHAN12
		CS	BIT1		# ENABLE T4RUPT OPTICS MONITOR .... PERMIT
		TS	OPTIND		#	OPTICS-ZERO BUT NOT OPTICS-DRIVE
		CAF	ZERO		# CLEAR NVWORD1 IN CASE CLOCKJOB WAITING
		TS	NVWORD1
		CS	BIT11		# CLEAR TIMRFLAG TO STOP CLOKTASK
		MASK	FLAGWRD7
		TS	FLAGWRD7
		TC	Q
		; ============================================================================
		; UPDATEVG - UPDATE VELOCITY-TO-BE-GAINED DURING BURN
		;
		; This routine is called periodically during engine burns (via AVEGEXIT
		; after average-G integration) to update the remaining velocity change
		; needed to achieve the target orbit. It computes the deviation between
		; actual achieved ΔV and desired ΔV, updating guidance for the next cycle.
		;
		; For external ΔV burns (XDELVFLG set), this routine is bypassed and
		; control goes directly to S40.8 for final state vector updates.
		;
		; The routine tracks burn progress through NBRCYCLS (number of guidance
		; cycles executed). On first call (NBRCYCLS < 0), it initializes the
		; ΔV tracking. On subsequent calls, it accumulates the actual ΔV achieved
		; (DELVSUM) and compares against reference (DELVREF).
		;
		; Restart protection: Uses Type B restart to ensure proper recovery if
		; interrupted. The phase change at OCT 10035 protects the ΔV accumulation.
		; ============================================================================

		EBANK=	DAPDATR1
UPDATEVG	STQ	BON
			QTEMP1
			XDELVFLG
			CALL40.8

		SLOAD	BMN
			NBRCYCLS
			SETUP.9
# Page 700

		; Accumulate total ΔV achieved by adding current cycle's DELVSUM to
		; the reference DELVREF. Store in DELVSUMP for restart protection.

		VLOAD	VAD
			DELVSUM
			DELVREF
		STORE	DELVSUMP
		EXIT
		CA	ONE
		AD	NBRCYCLS
		TS	NBRCYCLP

		TC	PHASCHNG	# TYPE B RESTART RESTART BELOW AND 5.3 REREADACCS
		OCT	10035

		; Complete the update by copying NBRCYCLP to NBRCYCLS and DELVSUMP
		; to DELVSUM after restart protection point.

		CA	NBRCYCLP
		TS	NBRCYCLS
		TC	INTPRET
		VLOAD
			DELVSUMP
		STORE	DELVSUM

CALL40.8	CALL
			S40.8
		GOTO
			QTEMP1

		; ============================================================================
		; SETUP.9 - PREPARE FOR BURN GUIDANCE CYCLE DISPLAY UPDATE
		;
		; This section prepares data for S40.9 which updates the crew displays
		; during the burn. It computes VGPREV (previous velocity-to-be-gained)
		; for display rate-of-change calculations.
		;
		; On first cycle (FIRSTFLG set), skip VGPREV computation since there is
		; no previous cycle data. On subsequent cycles, compute:
		;
		; VGPREV = (BDT normalized and scaled) + VGTEMP - DELVSUM
		;
		; where BDT is the desired total ΔV, VGTEMP is temporary guidance storage,
		; and DELVSUM is the accumulated ΔV achieved so far. The NORM instruction
		; handles zero BDT gracefully. The VSR* with index X1 performs variable
		; right shift for proper scaling (0 to -14 decimal shifts based on
		; normalization result).
		; ============================================================================

SETUP.9		BON	SLOAD
			FIRSTFLG
			SURELY.9
			NBRCYCLP
		NORM	VXSC		# (NORM HANDLES ZERO PROPERLY)
			X1
			BDT
		VSR*	VAD
			0 -14D,1
			VGTEMP
		VSU
			DELVSUM
		STORE	VGPREV

		; ============================================================================
		; SURELY.9 - LAUNCH BURN DISPLAY UPDATE TASK
		;
		; This section creates a high-priority job to update crew displays during
		; the burn. S40.9 runs at priority 10, updating DSKY displays with current
		; velocity-to-be-gained and time-to-go values.
		;
		; After job creation, restart protection is established for both the
		; display update job (phase A at 1.5 = REDO40.9, priority 10) and for
		; continuation of UPDATEVG (phase 10035).
		;
		; Initial state vectors (RINIT, VINIT) are saved from current navigation
		; state (RN, VN) at time TNIT (PIPTIME). DELLT4 tracks time since last
		; TPASS4 update. These provide reference conditions for burn monitoring.
		; ============================================================================

SURELY.9	EXIT
		CAF	PRIO10
		TC	FINDVAC
		EBANK=	DAPDATR1
		2CADR	S40.9

		TC	2PHSCHNG
		OCT	00051		# A, 1.5 = REDO40.9, PRIO 10
		OCT	10035
		TC	INTPRET
		VLOAD
			RN		# ACTIVE VEHICLE RADIUS VECTOR AT T1
		STOVL	RINIT
# Page 701
			VN		# ACTIVE VEHICLE VELOCITY VECTOR AT T1
		STODL	VINIT
			PIPTIME
		STORE	TNIT
		BDSU
			TPASS4
		STOVL	DELLT4
			HI6ZEROS
		STODL	DELVSUM
			HI6ZEROS
		STORE	NBRCYCLS
		GOTO
			CALL40.8

; ============================================================================
; STEERING ROUTINE
;
; COMMENT-ONLY READERS: During the burn, the guidance computer continuously
; calculates steering commands to point the spacecraft's engine in the correct
; direction. This routine ensures the thrust vector aligns with the desired
; flight path, making constant adjustments as the spacecraft accelerates.
;
; CODE-ALONG READERS: This is the main steering executive called during powered
; flight. It updates the velocity-to-be-gained (VG) vector by calling UPDATEVG,
; then checks the IMPULSW flag to determine if the burn is still active. During
; active thrust, it monitors engine status via DSALMOUT channel and sets up the
; ENGINOFF task to execute at the calculated TGO (time-to-go) for precise cutoff.
; ============================================================================

		EBANK=	DAPDATR1
STEERING	TC	INTPRET
		CALL
			UPDATEVG
		EXIT
		CAF	BIT9		# CHECK IMPULSW
		MASK	FLAGWRD2
		CCS	A
		TCF	+3		# PRE-IGNITE, REQUEST ENG-OFF, OR POST-OFF
SERVXT		TC	POSTJUMP
		CADR	SERVEXIT
		CAF	BIT13		# CHECK ENGINE-ON/-OFF
		EXTEND
		RAND	DSALMOUT
		EXTEND
		BZF	SERVXT		# ENGINE-OFF, SO PRE-IGNITE OR POST-OFF
		TCR	E7SETTER
		EBANK=	TIG
		INHINT
		EXTEND
		DCA	TIG
		DXCH	MPAC
		EXTEND
		DCS	TIME2
		DAS	MPAC
		TCR	DPAGREE
		CAE	MPAC +1		# (LESS THAN 6 (OR 4) SECONDS TO GO)
		CCS	A		# PROTECT AGAINST NEG/ZRO W.L. CALL
		TCF	+3
		TCF	+2
		CAF	ZERO
		AD	ONE
		XCH	L
		CA	ZERO
		DXCH	TGO
		CA	TGO +1
		TC	WAITLIST
# Page 702
		EBANK=	TGO
		2CADR	ENGINOFF

		TC	2PHSCHNG
		OCT	40153		# A, 3.15 = ENGINOFF (TGO+1) .... NOTE GROUP
		OCT	10035		# B, 5.3 = REREADAC, AND START BELOW
		TC	DOWNFLAG	# CLEAR IMPULSW, ENGINOFF IS NOW SET UP
		ADRES	IMPULSW		# RESTARTS OK
		TCF	SERVXT

# Page 703
# ROUTINE ** CLOKTASK ** DESCRIPTION

; ============================================================================
; CLOKTASK - COUNTDOWN CLOCK TASK
;
; COMMENT-ONLY READERS: This task runs once per second during countdown to
; ignition, displaying the time remaining until the burn starts. It's the
; routine that gives the crew their countdown timer on the DSKY display,
; updating every second as they approach the critical engine ignition moment.
;
; CODE-ALONG READERS: CLOKTASK is a WAITLIST task scheduled to run every second
; (1SEC interval) during pre-burn countdown. It checks TIMRFLAG to determine if
; countdown display is active. If active, it computes TTOGO (time-to-go until
; TIG) by subtracting current TIME2 from target ignition time TIG. The routine
; then schedules CLOCKJOB to update the V06N40 display showing countdown time.
; ============================================================================

		EBANK=	TIG
CLOKTASK	CAF	BIT11		# IS TIMRFLAG SET
		MASK	FLAGWRD7
		CCS	A
		TCF	CLOCKON
		TC	PHASCHNG
		OCT	00006		# KILL RESTART
		TC	TASKOVER

CLOCKON		EXTEND
		DCA	TIME2
		DXCH	TTOGO
		EXTEND
		DCS	TIG
		DAS	TTOGO

SETCLOCK	CAF	1SEC
		TC	WAITLIST
		EBANK=	TIG
		2CADR	CLOKTASK

		CCS	NVWORD1
		TCF	+3
		TCF	SETTB6

		TCF	+1
		CS	V06N85B		# CHECK FOR V06N85B (P41)
		AD	NVWORD1
		EXTEND
		BZF	SETUPDYN	# V06N85, SO UPDATE N85 FOR DYNAMIC DISP

		CAF	PRIO27
		TC	NOVAC
		EBANK=	DAPDATR1
		2CADR	CLOCKJOB

SETTB6		CS	TIME1		# SET GROUP6 TIMEBASE
		TS	TBASE6
		TCF	TASKOVER

SETUPDYN	CAF	PRIO27		# SET UP A JOB TO UPDATE N85 (FOR P41=V06)
		TC	FINDVAC
		EBANK=	DAPDATR1
		2CADR	DYNDISP

		TCF	SETTB6		# CLOSE OUT CLOCKTASK
# Page 704
DYNDISP		TC	INTPRET		# UPDATE N85 FOR A DYNAMIC V06N85 IN P41.
		CALL			#	PRIOR TO BLANKING AND AVEG (V16N85)
			P40CNV85
		EXIT
		TCF	CKNVWRD1

# Page 705
# ROUTINE ** CLOCKJOB ** DESCRIPTION

; ============================================================================
; CLOCKJOB - CLOCK DISPLAY UPDATE JOB
;
; COMMENT-ONLY READERS: This routine updates the countdown display on the DSKY
; and checks for crew inputs. The astronauts can monitor the countdown and use
; verb commands (V99 to proceed with ignition, V97 if engine fails) to control
; the burn sequence.
;
; CODE-ALONG READERS: CLOCKJOB runs as a VAC area job to update display and
; handle crew interaction. It first captures current CDU (Coupling Data Unit)
; gimbal angles for IMU orientation reference, then calls QUICTRIG to compute
; trigonometric quantities. The routine then checks NVWORD1 to determine display
; mode: V06N40 normal countdown, V99 engine-on-enable flash (awaiting ignition),
; or V97 engine-failure flash. Routes to appropriate verb handlers based on mode.
; ============================================================================

		EBANK=	DAPDATR1
CLOCKJOB	CA	CDUX
		TS	CDUSPOTX
		CA	CDUY
		TS	CDUSPOTY
		CA	CDUZ
		TS	CDUSPOTZ
		TC	BANKCALL
		CADR	QUICTRIG
; =============================================================================
; DISPLAY AND ENGINE STATUS HANDLING
;
; The computer monitors engine performance and interacts with the crew through
; the DSKY. If the SPS engine fails to ignite or produces insufficient thrust,
; the computer flashes V97 (engine failure) requesting crew decision. If all
; is nominal, V99 flashes (engine ready) for crew confirmation to proceed.
;
; During Apollo 11's critical burns (TLI, LOI, TEI), the crew watched these
; displays intently, ready to abort if the engine failed to respond.
; =============================================================================

; Check NVWORD1 to determine which display mode is needed: normal display,
; engine-failure flash (V97), or engine-ready flash (V99).

CKNVWRD1	INHINT
		CCS	NVWORD1		# DETERMINE FUNCTION, INDICATED BY NVWORD1
		TCF	NOFLASH		; Positive: normal display mode
		TCF	ENDOFJOB	; Zero: no display needed, job complete
		TCF	ENGREQST	# SPS ENGINE-ON-ENABLE V99 FLASH

; SPS engine failed to ignite properly. Flash V97 on DSKY requesting crew
; decision: Terminate burn (T), Proceed with backup plan (P), or Enter data (E).
; This is a critical moment - the crew must decide whether to retry ignition
; or switch to an abort contingency.

FAILDSP		CAF	V06N40		# SPS ENGINE-FAILED V97 FLASH
		TC	BANKCALL
		CADR	CLOCPLAY	; Flash display and wait for crew response
		TCF	V97T		# TERMINATE - crew aborts burn
		TCF	V97P		# PROCEED - crew attempts backup procedure
		TCF	V97E		# ENTER - crew provides corrective data

; SPS engine is ready and waiting for crew confirmation to ignite. Flash V99
; on DSKY: crew verifies all systems ready, then presses PROCEED to ignite.
; This final "Go" gives the astronauts control over the exact moment of ignition.

ENGREQST	CAF	V06N40
		TC	BANKCALL
		CADR	CLOCPLAY	# LINUS MAKES IT A REDO, INHINT OK
		TCF	V99T		# TERMINATE - crew aborts burn
		TCF	V99P		# PROCEED - crew confirms, ignite engine
		TCF	V99E		# ENTER - crew adjusts parameters

; Normal display mode: show current verb/noun data on DSKY without flashing.
; Used when no crew decision is required - the burn is progressing nominally.

NOFLASH		CAE	NVWORD1		# DISPLAY NVWORD1 NORMALLY
		TC	BANKCALL
		CADR	REGODSP		; Display on DSKY without flashing

; =============================================================================
; ERASABLE MEMORY BANK SWITCHING UTILITIES
;
; The AGC's 2K erasable memory is divided into banks. These subroutines switch
; between EBANK6 (DAP data) and EBANK7 (TIG and burn parameters). Bank switching
; is necessary because the AGC can only directly address 256 words at a time.
; =============================================================================

E7SETTER	CAF	EBANK7		; Switch to EBANK7 for TIG access
		TS	EBANK		; Set erasable bank register
		EBANK=	TIG		; Assembler notation: TIG is in EBANK7
		TC	Q		; Return to caller

E6SETTER	CAF	EBANK6		# SET UP EBANK6
		TS	EBANK		; Switch to EBANK6 for DAP data
		EBANK=	DAPDATR1	; Assembler notation: DAPDATR1 is in EBANK6
		TC	Q		; Return to caller

	; ============================================================================
; V99E - VERB 99 ENGINE-FAILURE HANDLER (ENTER KEY)
;
; COMMENT-ONLY READERS: If the astronaut determines the engine failed to ignite
; (no thrust indication at expected ignition time), they press ENTER after the
; V99 flash appears. This emergency handler immediately cancels all burn-related
; tasks and transitions to RCS backup control, ensuring the spacecraft can still
; maneuver even though the main engine failed.
;
; CODE-ALONG READERS: V99E is the engine-failure acknowledgment handler, invoked
; when crew presses ENTER during V99 flash (engine-on enable). It calls 2PHSCHNG
; to kill phase change group 6 (PRE40.6/CLOKTASK protection) and schedules V99EJOB
; at priority 27. The job calls TVCZAP to terminate TVC DAP and CLOKTASK, then
; branches to P40RCS for V16N85 post-burn operations using RCS-only control.
; ============================================================================

	EBANK=	DAPDATR1
V99E		TC	2PHSCHNG
		OCT	00006		# KILL PRE40.6/CLOKTASK PROTECTION
		OCT	05024		# C, PRIORITY NEXT, JOB BELOW
# Page 706
		OCT	27000
V99EJOB		TCR	TVCZAP -1	# WIPE OUT TVC, CLOKTASK
		TCF	P40RCS		# V16N85 POST-BURN OPERATIONS

	; ============================================================================
; V99T - VERB 99 TIMEOUT HANDLER (NO IGNITION DETECTED)
;
; COMMENT-ONLY READERS: If the engine fails to ignite within the expected time
; window, this handler automatically terminates the burn attempt. The computer
; detects that ignition never occurred and gracefully shuts down all burn-related
; activities, returning the spacecraft to a safe coast configuration.
;
; CODE-ALONG READERS: V99T is invoked when the V99 flash times out without crew
; action (engine never ignited). Also serves as entry point from V97T flow (engine
; thrust loss during burn). Kills phase group 6 and schedules V99TJOB at priority
; 27. The job calls TVCZAP to terminate TVC DAP and CLOKTASK, then proceeds to
; POST41 which executes AVEGEXIT (average G termination), SETMAXDB (restore wide
; deadband), and GOTOPOOH (return to idling program state).
; ============================================================================

	EBANK=	DAPDATR1
V99T		TC	2PHSCHNG	# (ENTRY FROM V97T FLOW TOO)
		OCT	00006		# KILL PRE40.6/CLOKTASK PROTECTION
		OCT	05024		# C, PRIORITY NEXT, JOB BELOW
		OCT	27000
V99TJOB		TCR	TVCZAP -1	# WIPE OUT TVC, CLOKTASK
		TCF	POST41		# AVEGEXIT, SETMAXDB, GOTOPOOH

; ============================================================================
; V99P - VERB 99 PROCEED HANDLER (ENGINE IGNITION GO-AHEAD)
;
; COMMENT-ONLY READERS: When the astronaut sees the V99 flash and the engine has
; successfully ignited (thrust detected), they press PROCEED to confirm ignition.
; This tells the computer "Yes, engine is running, continue with the burn." The
; guidance computer then proceeds with active thrust control for the remainder
; of the burn.
;
; CODE-ALONG READERS: V99P is the crew proceed response to V99 flash (engine-on
; enable). It checks the ASTN flag (BIT12 of FLAGWRD7) to detect restart entries.
; If this is first entry, sets ASTN flag and checks IGN flag (BIT13) for TIG-0
; arrival status. If TIG-0 already occurred, immediately schedules IGNITION task
; via WAITLIST at 10ms (BIT1). If TIG-0 not yet reached, clears V99 flash to
; V06N40 display and waits for time-triggered ignition sequence. This implements
; crew-confirmed engine start coordination with time-based ignition events.
; ============================================================================

V99P		INHINT
		CAE	FLAGWRD7	# CHECK ASTN FLAG FOR PRIOR V99P
		MASK	BIT12
		CCS	A
		TCF	V99P/TIG	# YES, THIS MUST BE A RESTART ENTRY

ASTNV99P	CAF	BIT12		# SET ASTN FLAG
		ADS	FLAGWRD7
		CAE	FLAGWRD7	# CHECK IGN FLAG FOR TIG-0 ARRIVAL
		MASK	BIT13
		EXTEND
		BZF	V99P/TIG	# NO, CLEAR THE V99 AND WAIT FOR TIG-0

ENDV99PI	CAF	BIT1		# TIG-0 HAS COME ALREADY
		TC	WAITLIST	# SET UP IGNITION HERE
		EBANK=	DAPDATR1
		2CADR	IGNITION

V99P/TIG	CAF	V06N40		# CLEAR THE V99 FLASH AND WAIT FOR TIG-0
		TS	NVWORD1
ENDV99P		TCF	ENDOFJOB

; ============================================================================
; V97T - VERB 97 THRUST LOSS DETECTOR (ENGINE FAILURE DURING BURN)
;
; COMMENT-ONLY READERS: During an active burn, the computer continuously monitors
; engine thrust. If thrust is suddenly lost (engine failure), this emergency
; handler immediately shuts down the engine, accounts for propellant consumed up
; to the failure point, and transitions to backup RCS control. This automatic
; response ensures crew safety when the main engine fails mid-burn - a contingency
; that thankfully never occurred during Apollo 11's three critical burns (TLI, LOI, TEI).
;
; CODE-ALONG READERS: V97T detects SPS thrust loss during powered flight. Kills
; phase group 6 (CLOKTASK) and schedules V97TTASK immediately (-0 centiseconds).
; The task first disables CLOCKJOB by zeroing NVWORD1, then adjusts spacecraft
; mass for 3 seconds of propellant flow (3MDOT) accounting for 2-4 second engine
; failure detection lag. Calls SPSOFF to command engine shutdown and perform mass
; update via MASSBACK. After 2.5 second delay (250 centiseconds) allowing for
; possible thrust tail-off or false alarm, V97TRCS enables RCS DAP and schedules
; V99T job to execute TVCZAP cleanup and POST41 termination sequence.
; ============================================================================

		EBANK=	CSMMASS
V97T		TC	2PHSCHNG
		OCT	00006		# KILL GROUP 6 (CLOKTASK)
		OCT	40674		# A, 4.67 = V97TTASK (-0 CS), TBASE NOW
		CAF	BIT1
		TC	TWIDDLE
		ADRES	V97TTASK	# KEEP EBANK6 FOR MASSES, SPSOFF, ETC.
		TCF	ENDOFJOB

		EBANK=	CSMMASS
V97TTASK	CAF	ZERO		# DISABLE CLOCKJOB
		TS	NVWORD1
		CAF	3MDOT		# 3 SECONDS OF MDOT (2-4 SEC ENGFAIL
		AD	CSMMASS		#	DETECTION) NOT LOST BECAUSE THRUST
		TS	MASSTMP		#	FAILED.  COPYCYCLE FOR MASSBACK
# Page 707
		TC	PHASCHNG
		OCT	05014		# C, DELTAT NEXT, TASK BELOW, IN
		DEC	-0		# -0 CS

		TCR	SPSOFF		# SHUTDOWN SPS ENGINE, MASS UPDATE, ETC.
		TC	PHASCHNG
		OCT	00714		# A, 4.71 = V97TRCS (250 CS), TBASE OLD
		TC	FIXDELAY	# DELAY 2.5 SECONDS FOR (POSSIBLE) TAIL-
		DEC	250		#	OFF (FALSE THRUST-LOSS)

		EBANK=	DAPDATR1
V97TRCS		TC	IBNKCALL	# RCS DAP IN 0.6SEC, SETTING T5 BITS TO
		CADR	RCSDAPON	#	KILL TVCEXEC/TVCROLLDAP STARTS
		CAF	PRIO27		# SET UP V99T FOR TVCZAP AND POST41 (SET-
		TC	NOVAC		#	MAXDB AND GOTOPOOH)
		EBANK=	DAPDATR1	# EBANK6 FOR SETMAXDB IN POST41
		2CADR	V99T

ENDV97T		TCF	TASKOVER

	; ============================================================================
; V97P - VERB 97 PROCEED (RESTART AFTER THRUST FAILURE)
;
; COMMENT-ONLY READERS: After detecting thrust loss (V97T), the computer waits for
; the crew to manually restart the engine. When the crew presses PROCEED (V97P),
; this handler re-enables thrust monitoring, restarts mass flow tracking, and
; restores normal burn operations. The 2-second delay allows engine thrust to
; stabilize before R40 burn monitoring resumes. This manual restart capability
; provided crews with control over engine restart timing after thrust failures.
;
; CODE-ALONG READERS: V97P handles crew PROCEED after thrust loss detection. 
; Schedules V97PTASK immediately (-0 centiseconds). The task restores VCNTR from
; saved V97VCNTR value to re-enable TVCEXEC mass updates (errors possible if false
; thrust-loss or poor synchronization between manual engine restart and PROCEED).
; Redisplays V06N40 by setting NVWORD1. Sets IDLEFAIL flag allowing R41-bypass
; during potentially unfavorable S40.8 synchronization. Sets STEERSW flag to
; re-enable steering computations. After 200 centisecond delay (2 seconds),
; R40ENABL clears IDLEFAIL to fully restore R40 burn monitoring, then kills
; phase group 4 to complete restart sequence.
; ============================================================================

	EBANK=	V97VCNTR
V97P		TC	PHASCHNG
		OCT	40734		# A, 4.73 = V97PTASK (-0 CS), TBASE NOW
		CAF	BIT1
		TC	TWIDDLE
		ADRES	V97PTASK
		TCF	ENDOFJOB

		EBANK=	V97VCNTR
V97PTASK	CAE	V97VCNTR	# GET MASS UPDATES (TVCEXEC) GOING AGAIN
		TS	VCNTR		#	(ERRORS IF FLASE THRUST-LOSS AND/OR
					#	POOR SYNC OF MANUAL ENGINE-ON AND
					#	THE VERB 97 PROCEED)
		CAF	V06N40		# REDISPLAY V06N40
		TS	NVWORD1
		TC	UPFLAG		# SET IDLEFAIL TO ALLOW R41-BYPASS, IN
		ADRES	IDLEFAIL	#	CASE OF UNFAVORABLE S40.8 SYNCH
		TC	UPFLAG		# SET STEERSW TO RE-ENABLE STEERING
		ADRES	STEERSW
		TC	PHASCHNG
		OCT	00134		# A, 4.13 = R40ENABL (200 CS), TBASE OLD
		TC	FIXDELAY	# WAIT 2 SECONDS, THEN
		DEC	200

		EBANK=	WHOCARES
R40ENABL	TC	DOWNFLAG	# RE-ENABLE R40 BY CLEARING IDLEFAIL
		ADRES	IDLEFAIL
		TC	PHASCHNG
		OCT	00004		# KILL GROUP 4
# Page 708
ENDV97P		TCF	TASKOVER

	; ============================================================================
; V97E - VERB 97 ENTER (TERMINATE BURN AFTER THRUST FAILURE)
;
; COMMENT-ONLY READERS: After thrust failure (V97T), if the crew cannot restart
; the engine, they press ENTER to terminate the burn early. This handler shuts
; down the engine safely, accounts for the lost fuel, and prepares the spacecraft
; for a possible reignition attempt. The display shows "59X59" indicating burn
; termination. After 2.5 seconds for engine tail-off, the system switches to RCS
; attitude control and prepares to flash V99, asking if the crew wants to attempt
; reignition. This gave Apollo crews the ability to recover from engine failures
; during critical maneuvers like lunar orbit insertion.
;
; CODE-ALONG READERS: V97E handles crew ENTER after thrust loss, initiating early
; burn termination. Schedules V97ETASK immediately (-0 CS). Task sets TIG display
; to 59X59 (complement of OCT24) indicating burn termination status, redisplays
; V06N40. Accounts for 3 seconds of propellant flow (3MDOT) during 2-4 second
; engine failure detection interval - adds to CSMMASS and stores in MASSTMP for
; MASSBACK copycycle (thrust failed so propellant not consumed). Calls SPSOFF
; immediately to shut down engine. After 250 CS (2.5 second) delay for possible
; tail-off or false thrust-loss recovery, V97E40.6 schedules PRE40.6 restart entry
; via WAITLIST to trim engine. Calls RCSDAPON to activate RCS DAP in 0.6 seconds,
; setting T5 bits to kill TVCEXEC/TVCROLLDAP task starts while maintaining narrow
; deadband for possible reignition. QUICKIGN section clears ASTNFLAG (PRIO14 bit)
; and sets IGNFLAG (BIT13) in FLAGWRD7 for immediate V99 response. After 30 CS
; delay for PRE40.6 completion, V99FLASH negates BIT9 in NVWORD1 to cause V99
; flash, sets up phase groups for TIG-0 PREPTVC and S40.13 burn program at PRIO20,
; then waits for CLOCKJOB immediate reaction to flashing V99.
; ============================================================================

	EBANK=	WHOCARES
V97E		TC	PHASCHNG
		OCT	40534		# A, 4.53 = V97ETASK (-0 CS), TBASE NOW
		CAF	BIT1
		TC	WAITLIST
		EBANK=	TIG
		2CADR	V97ETASK

		TCF	ENDOFJOB

		EBANK=	TIG
V97ETASK	CS	OCT24		# FORCE R1 OF V06N40 TO READ  59X59
		TS	TIG
		CAF	V06N40		# REDISPLAY V06N40
		TS	NVWORD1
		TCR	E6SETTER	# RETURN TO EBANK6 FOR REST OF V97ETASK
		EBANK=	CSMMASS
		CAF	3MDOT		# 3 SECONDS OF MDOT (2-4 SEC ENGFAIL
		AD	CSMMASS		#	DETECTION) NOT LOST BECAUSE THRUST
		TS	MASSTMP		#	FAILED....COPYCYCLE FOR MASSBACK
		TC	PHASCHNG
		OCT	00754		# A, 4.75 = SPSOFF97 (-0 CS), TBASE OLD
SPSOFF97	TCR	SPSOFF
		TC	PHASCHNG
		OCT	00114		# A, 4.11 = V97E40.6 (250 CS), TBASE OLD
		TC	FIXDELAY	# DELAY 2.5 SECONDS FOR (POSSIBLE) TAIL-
		DEC	250		#	OFF (FALSE THRUST-LOSS)

		EBANK=	DAPDATR1
V97E40.6	CAF	BIT1
		TC	WAITLIST
		EBANK=	CNTR
		2CADR	PRE40.6		# USE S40.6 RESTART ENTRY TO TRIM ENGINE

		TC	IBNKCALL	# RCS DAP IN 0.6SEC, SETTING T5 BITS TO
		CADR	RCSDAPON	#	KILL TVCEXEC/TVCROLLDAP STARTS.
					#	LEAVE NARROW DEADBAND FOR REIGNITE.

		TC	2PHSCHNG
		OCT	00026		# A, 6.2 = PRE40.6 (-0 CS), CLOKTASK (1 SEC)
		OCT	05014		# C, DELTAT NEXT, TASK BELOW, IN
		DEC	-0		# -0 CS.

QUICKIGN	CS	PRIO14		# CLEAR ASTNFLAG AND SET IGNFLAG FOR
		MASK	FLAGWRD7	#	IMMEDIATE V99 RESPONSE.
		AD	BIT13
		TS	FLAGWRD7
		TC	FIXDELAY	# DELAY TO ALLOW TIME FOR PRE40.6
# Page 709
		DEC	30

V99FLASH	CS	BIT9		# CAUSE V99 TO FLASH
		TS	NVWORD1
		TC	2PHSCHNG
		OCT	40774		# A, 4.77 = TIG-0 (-0 CS) TBASE FOR PREPTVC
		OCT	00033		# A, 3.3 = S40.13 (PRIO 20)
		CAF	PRIO20		# SET UP TIMEBURN
		TC	FINDVAC
		EBANK=	TGO
		2CADR	S40.13

ENDV97E		TCF	TASKOVER	# WAIT FOR CLOCKJOB (IMMEDIATE) REACTION
					# 	TO FLASHING V99 RESPONSE.

; ============================================================================
; S40.1 - COMPUTE STATE VECTORS AND THRUST DIRECTION FOR BURN
;
; COMMENT-ONLY READERS: Before every engine burn (like TLI to the Moon, LOI into
; lunar orbit, or TEI back to Earth), the guidance computer must calculate exactly
; where the spacecraft will be at ignition time (TIG) and which direction to point
; the engine. S40.1 performs this critical computation. It propagates the current
; position forward to ignition time using precise orbital mechanics, then either:
; (1) uses a simple "impulsive" calculation for standard burns, or (2) employs the
; sophisticated Lambert targeting algorithm when aiming for a specific point in
; space (like rendezvous maneuvers). The output tells the autopilot exactly where
; to point the engine bell for optimal trajectory. This routine ran before Apollo
; 11's TLI burn on July 16, 1969, computing the precise aim point that would send
; Columbia and Eagle toward the Moon.
;
; CODE-ALONG READERS: S40.1 computes burn state vectors RTIG, VTIG at ignition
; time TIG and thrust unit vector UT. Calls CSMPREC precision integration to
; propagate current state forward to TIG. For external ΔV burns (XDELVFLG=1),
; uses specified DELVSIN velocity change directly. For aimpoint steering 
; (XDELVFLG=0), calls INITVEL Lambert targeting to compute velocity toward target
; RTARG with arrival time TPASS4, using CSTEER steering parameter from ECSTEER.
; Calls CALCGRAV to compute gravitational acceleration at RTIG. Calls MIDGIM for
; gimbal angle computations. Outputs: UT (1/2 unit thrust vector B1), VGTIG 
; (initial velocity-to-be-gained B7 M/CS), DELVLVC (VG in local vertical coords),
; BDT (velocity change in 2 seconds for S40.13), and F (nominal thrust B7 M-NEWT).
; Returns normally at L+2, or if Lambert unsolvable returns with NOSOFLAG=1.
; Critical subroutine for all SPS/RCS main burns.
; ============================================================================

# MOD N02				LOG SECTION P40-P47
# MOD BY ZELDIN
#
# FUNCTIONAL DESCRIPTION
#	COMPUTE INITIAL THRUST DIRECTION(UT) AND INITIAL VALUE OF VG
#	VECTOR(VGTIG).
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		S40.1
#
# NORMAL EXIT MODE
#	AT L+2 OF CALLING SEQUENCE (GOTO L+2) NORMAL RETURN OR
#	ERROR RETURN IF NOSOFLAG =1
#
# SUBROUTINES CALLED
#	CSMPREC
#	INITVEL
#	CALCGRAV
#	MIDGIM
#
# ALARM OR ABORT EXIT MODES
#	L+2 OF CALLING SEQUENCE, UNSOLVABLE CONIC IF NOSOFLAG=1
#
# ERASABLE INITIALIZATION REQUIRED
#	WEIGHT/G	ANTICIPATED VEHICLE MASS	SP B16 KGM
#	XDELVFLG	1=DELTA-V MANEUVER, 0=AIMPT STEER
#   IF DELTA-V MANEUVER:
#	DELVSIN		SPECIFIED DELTA-V REQUIRED IN
#			INERTIAL COORDS. OF ACTIVE VEHICLE
#			AT TIME OF IGNITION		VECTOR B7 M/CS
#	DELVSAB		MAG. OF DELVSIN			DP B7 M/CS
#	RTIG		POSITION AT TIME OF IGNITION	VECTOR B29 M
#	VTIG		VELOCITY AT TIME OF IGNITION	VECTOR B7 M/CS.
#	CSTEER = 0					DP
#   IF AIMPOINT STEERING:
#   IF AIMPT STEER
#	TIG		TIME OF IGNITION		DP B28 CS
#	RTARG		POSITION TARGET TIME		VECTOR B29 M
#	CSTEER = ECSTEER (GR 0) 			DP B1
# Page 710
#	TPASS4 -- TIME OF ARRIVAL AT AIMPOINT
#
# OUTPUT
#	UT		1/2 UNIT VECTOR ALIGNED WITH THRUST DIRETION IN REF COOR
#	VGTIG		INITIAL VALUE OF VELOCITY
#			TO BE GAINED (INERT. COORD.)		VECTOR B7 M/CS
#	DELVLVC		VGTIG IN LOC. VERT. COORDS.		B7 M/CS
#	F		NOMINAL THRUST FOR ENG USED FOR S40.13	DP B7 M-NEWT
#	BDT		V REQUIRED AT TIG -V REQUIRED AT (TIG-2SEC)
#	-GDT		FOR S40.13				VECT B7 M/CS
#	RTIG		CALC IN S40.1B (AIMPT) FOR S40.2,3	VECTOR B29M
#			POSITION AT TIME OF IGNITION
#
# DEBRIS	QTEMP1
#		MPAC, QPRET
#		PUSHLIST
#		RTX2,RTX1

		BANK	14
		SETLOC	P40S1
		BANK

		COUNT	16/S40.1

S40.1		SET	VLOAD
			FIRSTFLG
			LO6ZEROS
		STORE	BDT
		STQ	BOF
			QTEMP
			XDELVFLG
			S40.1B		# LAMBERT
		VLOAD	ABVAL		# EXTERNAL DELTA-V
			DELVSIN
		STORE	DELVSAB		# COMPUTE FOR P30/P40 INTERFACE
					#	THUS PERMITTING MODULE-ONLY CHANGE
		SETPD	VLOAD
			0
			VTIG
		STORE	VINIT
		VXV	UNIT
			RTIG
		STOVL	UT		# UP IN UT
			RTIG
		STORE	RINIT
		VSQ	PDDL
			36D
		DMPR	DDV
			THETACON
		DMP	DMP
			DELVSAB
			WEIGHT/G
		DDV
# Page 711
			F
		STOVL	14D
			DELVSIN

		DOT	VXSC
			UT
			UT
		VSL2	PUSH		# (DELTAV.UP)UP SCALED AT 2(+7) P.D.L. 0
		BVSU	PDDL		# DELTA VP SCALED AT 2(+7) P.D.L. 6
			DELVSIN
			14D
		SIN	PDVL
			6D
		VXV	UNIT
			UT
		VXSC	STADR
		STOVL	VGTIG		# UNIT(VP X UP)SIN(THETAT/2) IN VGTIG.
		UNIT	PDDL		# UNIT(DELTA VP) IN P.D.L. 6
			14D
		COS	VXSC
		VAD	VXSC
			VGTIG
			36D
		VSL2 	VAD
		STADR
		STORE	VGTIG		# VG IGNITION SCALED AT 2(+7) M/CS

		UNIT
		STOVL	UT		# THRUST DIRECTION SCALED AT 2(+1)
			VGTIG
		PUSH	SET
			AVFLAG
		CALL
			MIDGIM		# VGTIG IN LV COOR AT 2(+7)M/CS IN DELVLVC
		GOTO
			QTEMP
S40.1B		DLOAD	DSU		# LAMBERT
			TIG
			TWODT
		STODL	TDEC1
			TPASS4
		DSU
			TDEC1
		STCALL	DELLT4
			AGAIN
		VLOAD
			VIPRIME
		STODL	UT
			TIG
		STORE	TDEC1
# Page 712
		BDSU
			TPASS4
		STCALL	DELLT4
			AGAIN
		VLOAD	PUSH
			DELVEET3
		STORE	VGTIG
		SET	CALL
			AVFLAG
			MIDGIM
		SETPD	GOTO
			0
			CALCUT

THETACON	2DEC	.31830989 B-8

		SETLOC	P40S3
		BANK

		COUNT	24/S40.1

EP4(45)H	2DEC	.125

EP4(10)H	2DEC	.027777777

AGAIN		STQ	CALL
			QTEMP1
			THISPREC
		SXA,2	SXA,1
			RTX2
			RTX1
		VLOAD
			RATT
		STORE	RTIG
		STOVL	RINIT
			VATT
		STORE	VTIG
		STORE	VINIT
		SETPD	SLOAD
			0
			HI6ZEROS
		PDDL	BON
			EP4(45)H
			NORMSW
			+3
		DLOAD
			EP4(10)H
		PUSH	CALL
			INITVEL
		SETPD	GOTO
# Page 713
			0
			QTEMP1
CALCUT		VLOAD	CALL
			RTIG
			CALCGRAV	# GDELTAT IN MPAC AT 2(+7) M/CS
		VSL1	V/SC
			200CS		# G AT 2(-5) M/CS. CS
		PDVL	VSU
			VIPRIME
			UT
		V/SC	VSU
			200CS
		VXSC	VSL2
			CSTEER
		STOVL	12D		# B.C SCALED AT 2(-15) PDL 12D
			VGTIG
		UNIT	PUSH		# UG PDL 0 SCALED AT 2(+1)

		DOT	VXSC
			12D
			0
		VSL2	BVSU
			12D
		STODL	12D		# Q PDL SCALED AT 2(-5)
			F
		SRR	DDV
			4
			WEIGHT/G
		DSQ	PDVL		# F/MASS SQUARED PDL 6 AT 2(-10M/(CS.CS)
			12D
		VSQ
		BDSU	SQRT
		VXSC	VSL1
		VAD	UNIT
			12D
		STCALL	UT
			QTEMP
200CS		2DEC	200 B-12

# Page 714
# PROGRAM DESCRIPTION S40.2,3		DATE 15 NOV 66
# MOD NO 2				LOG SECTIONS P40-P47
# MOD BY ZELDIN
#
# FUNCTIONAL DESCRIPTION
#
#	COMPUTE GIMBAL ANGLES IF THRUSTING OCCURRED WITH PRESENT IMU
#	ORIENTATION, WINGS LEVEL SPACECRAFT, HEADS UP
#	COMPUTE X AXIS OF ENGINE BELL
#	COMPUTE PREFERRED IMU ORIENTATION (XSCREF)
#	FOR THIS CALCULATION, ASSUME X AXIS OF SC ALONG UT INITIALLY,
#	YSC=UNIT(XXR), ZSC=UNIT(XX(XXR)) AND ROTATE ENGINE BELL ALONG UT.
#	NEW SC AXES WILL BE APPROX. WINGS LEVEL AND NEW SC AXES IN REF.
#	COORDS. WILL BE PREFERRED IMU ORIENTATION.
#	COMPUTE DESIRED THRUST DIRECTION IN SM COORDS.
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		S40.2,3
#
# NORMAL EXIT MODE
#	AT L+2 OF CALLING SEQUENCE (GOTO L+2)
#
# SUBROUTINES CALLED
#	CALCGA
#
# ALARM OR ABORT MODES
#	NONE
#
# ERASABLE INITIALIZATION REQUIRED
#	PACTOFF		TOTAL PITCH TRIM ANGLE		SP AT 1.0795111 REV.
#	YACTOFF		TOTAL YAW   TRIM ANGLE		SP AT 1.0795111 REV.
#	UT		DESIRED THRUST DIRECTION	VECT. B2 M/(CS.CS)
#	RTIG		POSITION AT TIME OF IGNITION	VECT. B29 M
#	ENG2FLAG	ON=RCS  OFF=SPS
#
# OUTPUT
#	SCAXIS		UNIT VECT. ALIGNED WITH ENG BELL IN SC COOR.	B1
#	XSCREF		UNIT VECTORS ALIGNED WTH PREFERRED IMU		B1
#	YSCREF
#	ZSCREF
#	GIMBAL ANGLES IN THETAD
#	POINTVSM	UNIT VECT ALONG DESIRED THRUST DIRECTION IN SM	B1
#
# DEBRIS
#	PUSHLIST, QPRET, MPAC
#	QTEMP	TEMP. ERASABLE

		BANK	24
		SETLOC	P40S
		BANK
		COUNT*	$$/S40.2
S40.2,3		VLOAD	MXV
			UT
			REFSMMAT
		VSL1	STQ
			QTEMP
		STORE	POINTVSM	# THRUST IN SM AT 2
		SETPD	BON
			0
# Page 715
			ENG2FLAG
			S40.2,3B
		DLOAD
			HI6ZEROS
		PUSH	SLOAD		# ZERO PDL 0
			YACTOFF
		DMP	SL1
			TRIMSCAL
		DAD	PUSH
			YBIAS
		COS	PDDL		# COS(Y +Y0) PDL 2
		SIN	PUSH		# SIN(Y +Y0) PDL 4
		SLOAD
			PACTOFF
		DMP	SL1
			TRIMSCAL
		DAD	PUSH
			PBIAS
		COS	PDDL		# COS(P +P0) PDL 6
		SIN	PUSH		# SIN(P +P0) PDL 8D
		STODL	ZSCREF		# SIN(P +P0)
			6
		DMP	SL1
			4
		DCOMP	PDDL		# -SIN(Y+Y0)COS(P+P0) PDL 10
			6
		DMP	SL1
			2
		VDEF

		STODL	XSCREF		# PD POINTER AT 6 NEW SC X AXIS SCALED AT
			ZSCREF
		DMP	SL1
			4
		PDDL	DMP
			ZSCREF
			2
		SL1	DCOMP
		VDEF

		STODL	ZSCREF		# PD POINTER AT 4 NEW SC Z AXIS SCALED AT 2
		VDEF

		STODL	YSCREF		# PD POINTER AT 0 NEW SC Y AXIS SCALED AT 2
			ZSCREF
		PDDL	PDDL
			YSCREF
			XSCREF
		VDEF
# Page 716
		STOVL	SCAXIS		# ENGINE BELL SCALED AT 2
			UT
		PDVL	UNIT
			RTIG
		VXV	VCOMP
			0
		UNIT	PUSH
		CALL
			TSTRXUT
		VXV	VCOMP
			0
		VSL1	PDVL		# 2 RF/SC IN PDL 12D
			XSCREF
		VXM	VSL1
			0
		STOVL	XSCREF		# X OF PREF. IMU,X OF SC IN REF COOR. AT 2
			YSCREF
		VXM	VSL1
			0
		STOVL	YSCREF		# Y OF PREF. IMU,Y OF SC IN REF COOR. AT 2
			ZSCREF
		VXM	VSL1
			0
		STORE	ZSCREF		# Z OF PREF. IMU,Z OF SC IN REF COOR. AT 2
		SETPD	GOTO
			0
			QTEMP
S40.2,3B	VLOAD
			UNITX
		STOVL	SCAXIS
			UT
		STORE	XSCREF
		VXV	UNIT
			RTIG
		STCALL	6D
			TSTRXUT
		STORE	YSCREF
		VXV	VCOMP
			XSCREF
		VSL1
		STCALL	ZSCREF		# ZNB AXIS IN REF COOR
			QTEMP
TSTRXUT		DLOAD	BHIZ
			36D
			BADVCTOR
		VLOAD	RVQ
			6D
BADVCTOR	VLOAD	UNIT
			RTIG
		PDVL	UNIT
# Page 717
			VTIG
		VSR3	VAD
		VXV	UNIT
			UT
		VCOMP
		STORE	6D
		RVQ
TRIMSCAL	2DEC	1.07975111 B-1

YBIAS		2DEC	+.00263888889	# YAW	MECH BIAS (+0.95 DEG, THRUST ON)

PBIAS		2DEC	-.00597222222	# PITCH	MECH BIAS (-2.15 DEG, THRUST ON)

					# REFERENCE, TRW 68.6520.3.3-40 27 FEB, 1968

# PROGRAM DESCRIPTION S41.1		DATE 8 DEC 66
# MOD NO 1				LOG SECTION P40-P47
# MOD BY ZELDIN
#
# FUNCTIONAL DESCRIPTION
#
#	COMPUTE VELOCITY TO BE GAINED INITIALLY IN REF COORDS.
#	TO CONTROL COORDS.
#
# CALLING SEQUENCE
#
#	L	CALL
#	L+1		S41.1
#
# NORMAL EXIT MODE
#
#	AT L +2 OF CALLING SEQUENCE
#
# SUBROUTINES CALLED:
#
#	CALCSMSC
#	CDUTRIG
#
# ALARM OR ABORT MODES
#
#	NONE
#
# ERASABLE INITIALIZATION REQUIRED
#
#	VG IN REF. COORD. PDL L POINTER AT L+5.  S41.1 WILL RETURN WITH
# 	POINTER AT L (L MUST BE LESS THAN OR = TO 14D)
#
# OUTPUT
#
#	MPAC CONTAINS VG IN CONTROL COORDS		VECT. B7 M/CS
#
# DEBRIS:
#
#	QTEMP		TEMP ERASABLE
#	QPRET

		COUNT	22/S41.1

		SETLOC	P40S5
		BANK

S41.1		STQ	CALL
			QTEMP
			CDUTRIG
		VLOAD
		MXV	CALL
			REFSMMAT
			*SMNB*
# Page 718
		MXV	VXSC
			QUADROT
			TENBNK14	# VG IN CONTROL COORD IN MPAC SCALED AT
		VSL5	GOTO		# VG IN CONTROL COORDS. IN MPAC AT 2(+7)
			QTEMP
TENBNK14	2DEC	10. B-4

# Page 719
# NAME		S40.8 -- CROSS PRODUCT STEERING
# FUNCTION	(1) UPDATES THE VELOCITY-TO-BE-GAINED VECTOR.
#		(2) GENERATES ANGULAR RATE STEERING COMMANDS FOR AUTOPILOT.
#		(3) ESTABLISHES ENGINE CUT-OFF SIGNALS AT APPROPRIATE TIMES.
#		(4) INITIATES THRUST-FAIL ROUTINE, R40
# CALLING SEQ	CALL S40.6
# INPUT		VGPREV 		LAST VALUE OF THE VELOCITY-TO-BE-GAINED VECTOR
#				PRIOR TO UPDATING IN METERS/CS AT +7.
#		DELVREF		CHANGE IN VEHICLE VELOCITY SINCE LAST MEASUREMENT
#				IN METERS/CS AT +7.
#		BDT		EFFECT OF RATE OF CHANGE OF REQUIRED VELOCITY AND
#				GRAVITY DURING DT UPON VELOCITY-TO-BE-GAINED IN
#				METERS/CS AT +7.
#		CSTEER		A SCALAR OF THE STEERING LAW, SC.AT B+1, USED FOR
#				SPS AIMPOINT STEERING MANEUVERS.
#		IDLEFAIL	A FLAG TO INHIBIT (IDLE) THE THRUST-FAIL ROUTINE.
#		STEERSW		A SWITCH TO PRECLUDE NEEDLESS CONDUCT OF STEERING.
#		REFSMMAT, DAPDATR1, PIPTIME
#		EREPFRAC, ETDECAY, KPRIMEDT FOR TVC.
# OUTPUT	TTOGO		TIME REMAINING FOR ENGINE BURN IN CS AT +28.
#		OMEGAC		DP VECTOR RATE COMMAND, SC.AT 1/(2TVCDT) REVS/SEC.
#		VG, VGPREV, VGDISP, TGO, TIG, SCALED AS NOTED IN CODING
#		STEERSW, IMPULSW, NVWORD1
#		REPFRAC, CNTR, VCNTR, VCNTRTMP FOR TVC (R40 INTERFACING)
# DEBRIS	OMEGAXC, +1
# SUBROUTINES USED:  *SMNB*, ALARM

		SETLOC	P40S1
		BANK
		EBANK=	DAPDATR1
		COUNT	16/S40.8

S40.8		SETPD	STQ
SPBIT1			00D
			QTEMP
		VLOAD	BVSU		# CONSTRUCT DELVG, SC.AT B+7 M/CS
			DELVREF
			BDT
		VAD
			VGPREV
		STORE	VG		# VELOCITY-TO-BE-GAINED, SC.AT B+7 M/CS

		ABVAL
		STORE	VGDISP		# FOR DISPLAY PURPOSES
		EXIT
		TC	PHASCHNG
		OCT	10035		# TYPE B RESTART RESTART BELOW AND 5.3 REREADAC

		TC	INTPRET
		VLOAD
# Page 720
			VG
		STORE	VGPREV
		BOFF	VLOAD
			STEERSW		# SKIP TGO AND CROSS-PRODUCT
			QTEMP
			DELVREF
		ABVAL	PUSH		# CHECK FOR LOTHRUST
		SLOAD	DMP
			DVTHRESH	# SC.AT B-2 M/CS
			DPB-9
		BDSU
		BMN	EXIT
			LOTHRUST
		CAE	DAPDATR1	# ENABLE TVCDAP CG TRACKING
		MASK	BIT14
		CCS	A
		CAF	BIT1
		INDEX	A		# LM-OFF, LM-ON VALUE
		CAE	EREPFRAC
		TS	REPFRAC

		TC	INTPRET
TGOCALC		VLOAD	BVSU		# GET DELVG
			DELVREF
			BDT
		UNIT
		DOT	PUSH		# (00D)
			VG
		BPL	DDV		# ANGLE SHOULD BE GREATER THAN PI/2
			INCRSVG		#	DISPLAY ALARM IF NOT
			2VEXHUST
		DAD	DMP		# (DOT PRODUCT UP FROM 00D)
			LODPHALF
		NORM	SR1
			X1
		PDDL	NORM
			36D		# (MAG DELVG)
			X2
		BDDV
		XSU,2	SL*
			X1
			0 -9D,2
		DMP	PUSH		# (00D)
			-FOURDT
		SLOAD	SR
			ETDECAY		# ETDECAY SC.AT B+14 CS
			14D
		BDSU	STADR
		STORE	TGO		# TIME TO GO IN CS. AT +28
		DAD
# Page 721
			PIPTIME
		STODL	TIG
			TGO
		DSU	BMN
			FOURSEC
			S40.81

XPRODUCT	VLOAD	VXSC
			BDT
			CSTEER
		VSL2	VSU
			DELVREF
		UNIT	PDVL
			VG
		UNIT	VXV
		MXV	CALL
			REFSMMAT	# (REFSMMAT/2)
			*SMNB*
		VXSC
			KPRIMEDT	# (KPRIMEDT SCIAT PI/8 RAD)
OMEGACLC	STORE	OMEGAC
		GOTO
			QTEMP

		SETLOC	DAPS7
		BANK
		COUNT	17/S40.8

TWODT		2DEC	200.0 B-28	# 2 SEC

-FOURDT		2DEC	-800 B-18	# -4(200CS), SC.AT B+18CS (-4 FOR SCALING)

2VEXHUST	2DEC	63.020792 B-7	# 2(10338.0564 FPS), SC.AT B+7 M/CS

FOURSEC		2DEC	400.0 B-28	# 4 SEC

DPB-9		2DEC	1 B-9

		SETLOC	DAPS6
		BANK

		COUNT	20/S40.8

S40.81		SET	VLOAD		# TGO LESS THAN 4 SECONDS
			IMPULSW		# FOR ENGINE-OFF CALL
			HI6ZEROS
RATEZRO		STORE	OMEGAC		# TVC TO ATTITUDE HOLD
		EXIT
		CAF	POSMAX		# INHIBIT SWITCHOVER/TVC EG TRACKING
		TS	CNTR
# Page 722
		TC	INTPRET
		CLEAR	GOTO
			STEERSW		# RESTARTS OK
			QTEMP
INCRSVG		EXIT			# ALARM INDICATING THAT THRUST IS POINTING
		TC	ALARM		# IN WRONG DIRECTION.
		OCT	01407
		TC	INTPRET
		GOTO
			QTEMP

LOTHRUST	BON	VLOAD		# THRUST FAILURE (LO-OR-NO) INDICATED
			IDLEFAIL	# SET BY V97P.  ALLOWS 1 BYPASS IN CASE OF
			QTEMP		#	UNFAVORABLE S40.8 SYNCH.
			HI6ZEROS	# START OF ENGINE-FAIL (R40) OPERATIONS
		STORE	OMEGAC		# PUT TVC IN ATTITUDE HOLD
		EXIT

		CS	ZERO
		TS	VCNTR		# KILL CSMMASS UPDATING
		TS	VCNTRTMP	# (TVCEXEC LOGIC REQUIRES THIS TOO)
		TS	REPFRAC		# KILL TVCDAP CG TRIM TRACKING
		TS	NVWORD1		# SET UP ENGINE-FAIL V97FLASH (CLOCKJOB)

		TC	INTPRET
		CLEAR	GOTO		# INHIBIT STEERING AND TGO CALC (MANUAL
			STEERSW		# 	SHUTDOWN IF NOT SET UP AGAIN)
			QTEMP		# RESTARTS OK

# Page 723
# NAME		S40.9 -- VTOGAIN (AIMPOINT MANEUVERS ONLY)
# FUNCTION	(1) GENERATES REQUIRED VELOCITY AND VELOCITY-TO-BE-GAINED
#		VECTORS FOR USE DURING AIMPOINT MANEUVERS.
#		(2) UPDATES THE B VECTOR WHICH IS USED IN THE FINAL
#		CALCULATION OF EXTRAPOLATING THE VELOCITY-TO-BE-GAINED.
# CALLING SEQ	VIA FINDVEC AS NEW JOB.
# INPUT		RNIT	ACTIVE VEHICLE RADIUS VECTOR IN METERS AT +29.
#		VNIT	ACTIVE VEHICLE VELOCITY VECTOR IN METERS/CS AT +7.
#		VRPREV	LAST COMPUTED VELOCITY REQUIRED VECTOR IN
#			METERS/CS AT +7.
#		NONTIG	TIME OF IGN.  USED IN TARGETTING ROUTINES B+28
#		DELLT4	TRANSFER TIME FROM PIPTIME TO TARGET B+28
#		TNIT	TIME OF RNIT AND VNIT IN CS AT +28
#		GDT/2	HALF OF VELOCITY GAINED IN DELTA T TIME DUE TO
#			ACCELERATION OF GRAVITY IN METERS/CS AT +7.
#		DELVREF	CHANGE IN VELOCITY DURING LAST 2 SEC IN
#			METERS/CS AT +7.
#		NORMSW	SET=CENTRAL ANGLE BETWEEN RTARG AND RTIG IS BETWEEN
#			165 TO 195 DEGREES.
#			RESET=CENTRAL ANGLE OUTSIDE CONE DESCRIBED ABOVE.
# OUTPUT	VGTEMP	VELOCITY TO BE GAINED VECTOR IN METERS/CS AT +7.
#		COGA	INPUT OF INITIAL GUESS FOR LAMBERT FROM S40.1
#			OR PREVIOUS PASS THRU S40.9.
#		GOBL/2	OBLATENESS TERM IN AVG GRAV CALC: GOBL*RSQ/MU
#		VRPREV/	VELOCITY REQUIRED VECTOR IN METERS/CS AT +7.
#		BDT	B VECTOR IN METERS/CS AT +7.
# SUBROUTINES USED -- INITVEL

		SETLOC	P40S1
		BANK

		EBANK=	NBRCYCLS
		COUNT	16/S40.9

S40.9		TC	INTPRET
		SETPD	DLOAD
			00D
			LO6ZEROS
		PDDL
			EP4(45)L
		BON	DLOAD
			NORMSW
			+2
			EP4(10)L
		PUSH
		CLEAR	CALL
			GUESSW
			HAVEGUES
		EXIT
		TC	PHASCHNG	# SAVE TIME BY NOT REDOING LAMBERT CALCS
		OCT	05021		# C, PRIORITY NEXT, JOB BELOW
# Page 724
		OCT	10000
		TC	INTPRET
ENDLAMB		BON
			FIRSTFLG
			FIRSTTME
		VLOAD	VSU
			VIPRIME
			VRPREV
		PDDL	DSU
			TNIT
			TNITPREV
		SL	BDDV
			17D
			200CSHI
		VXSC
		VSU	VSL1
			GDT/2
		STORE	BDT
FIRSTTME	SLOAD	DCOMP
			RTX2
		BMN
			MOONCASE
		VLOAD	UNIT
			RN
		DLOAD	DSU
			PIPTIME
			NOMTIG
		DMP	DDV
			EARTHMU
			34D
		VXSC	VAD
			GOBL/2
			VGTEMP		# NOTE: NO TEST IS MADE TO SUBTRACT GOBL
		STORE	VGTEMP		# INSIDE 165-195 DEGREE CONE AREA.
MOONCASE	EXIT
		TC	PHASCHNG
		OCT	04021		# C, JOB BELOW

COPY40.9	TC	INTPRET
		DLOAD
			TNIT
		STOVL	TNITPREV
			VIPRIME
		STORE	VRPREV
		CLEAR	EXIT
			FIRSTFLG
	-2	CS	ONE		# REDO40.9 (RESTART) ENTRY TO END S40.9
		TS	NBRCYCLS
ENDS40.9	TC	PHASCHNG
		OCT	00001
# Page 725
		TCF	ENDOFJOB

REDO40.9	TC	INTPRET		# S40.9 RESTARTS COME HERE TO GRACEFULLY
		VLOAD			#	TERMINATE S40.9 SO THAT IT CAN BE
			LO6ZEROS	#	SET UP WITH LATEST R,V,T NEXT PASS
		STODL	DELVSUM		#	(TYPE C PHASE POINTS '04021' WILL
			LO6ZEROS	#	FORCE NORMAL S40.9 TERMINATIONS,
		STOVL	NBRCYCLS	#	RATHER THAN LOSE TIME OF BRAND NEW
			VGPREV		#	PASS -- QUICK OLD DATA BETTER THAN
		STORE	VGTEMP		#	NONE) NOW CAN GO THRU SETUP.9
		EXIT			#	WITHOUT DISTURBING VGPREV.
		TCF	ENDS40.9 -2	# STORE 0,0 COVERED NBRCYCLS,P -- FIX UP S

200CSHI		2DEC	200 B-12

EARTHMU		2DEC*	-3.986032 E10 B-36*

EP4(45)L	2DEC	.125

EP4(10)L	2DEC	.027777777

# Page 726
# NAME:  		S40.13 -- TIMEBURN
#
# FUNCTION		(1) DETERMINE WHETHER A GIVEN COMBINATION OF VELOCITY-TO-
#			BE-GAINED AND ENGINE CHOICE RESULT IN A BURN TIME SUFFICIENT
#			TO ALLOW STEERING AT THE VEHICLE DURING THE BURN, AND
#			(2) THE MAGNITUDE OF THE RESULTING BURN TIME -- IF IT IS SHORT --
#			AND THE ASSOCIATED TIME OF THE ENGINE OFF SIGNAL.
#
# CALLING SEQUENCE	VIA FINDVAC AS A NEW JOB
#
# INPUT			VGTIG -- VELOCITY TO BE GAINED VECTOR (METERS/CS) AT +7
#			WEIGHT/G -- MASS OF VEHICLE IN KGM AT TIG
#			F -- ENGINE THRUST IN M.NEWTONS AT +7
#			MDOT -- RATE OF DECREASE OF VEHICLE MASS DURING ENGINE BURN
#				IN KILOGRAMS/CENTISECOND AT +3.  THIS SCALING MAY
#				REQUIRE MODIFICATION FOR SATURN BURNS.
#
# OUTPUT		IMPULSW		ZERO FOR STEERING
#					ONE FOR ATTITUDE HOLD
#			TGO		TIME TO BURN IN CENTISECONDS AT +14
#			THE QUANTITY M.NEWTON SHALL BE USED TO EXPRESS WEIGHT IN TERMS OF
#			(KILOGRAM*METER)/(CENTISECOND*CENTISECOND)
#			(1) M.NEWTON = (10000) NEWTONS.

		EBANK=	TGO
		COUNT	16/40.13

S40.13		TC	INTPRET
		SETPD	SET
			00D
			IMPULSW		# ASSUME NO STEERING UNTIL FOUND OTHERWISE
		VLOAD	ABVAL
			VGTIG		# VELOCITY TO BE GAINED AT +7
		EXIT
		CAF	BIT7		# TEST +X TRANSLATION
		EXTEND
		RXOR	CHAN31
		MASK	BIT7
		EXTEND
		BZF	NOTADDUL
		TC 	INTPRET
		PDDL	DDV		# 00D = MAG OF VGTIG AT +7
			S40.135		# COMPENSATION FOR 2 JET ULLAGE AT +24
			WEIGHT/G	# MASS IN KGMS AT +16
		BON	SL1		# DOUBLE CORRECTION IF FOUR JETS
			NJETSFLG
			S40.130
S40.130		BDSU
		PDDL	DDV		# 00D = MAG OF VGTIG CORRECTED FOR ULLAGE
			K1VAL		# M.NEWTON-CS AT +24
			WEIGHT/G
		BDSU	BMN
			00D
			S40.131		# TGO LESS THAN 100 CS
		PDDL	DMP		# 02D = TEMP1 AT +7
# Page 727
			EMDOT		# SPS FLOW RATE SC.AT B+3 KG/CS (SP, NOTE)
			3.5SEC		# 350 CS AT +14
		BDSU	PDDL
			WEIGHT/G
			FANG
		DMP	SR2
			5SECOND		# 500 CS AT +14
		DDV	PUSH		# 04D = TEMP2
		BDSU	BPL
			02D
			S40.133		# TGO GREATER THAN 600 CS
		DLOAD	BDDV
		DMP	DAD
			5SECOND		# 500 CS AT +14
			1SEC2D		# 100 CS AT +14
		GOTO
			S40.132
S40.131		DLOAD	DMP		# TGO LESS THAN 100 CS
			WEIGHT/G
		DAD	DDV
			K2VAL		# M.NEWTON CS AT +24
			K3VAL		# M.NEWTON AT +10
S40.132		EXIT
		EBANK=	TGO
		TC	TPAGREE
		CA	MPAC
		XCH	L
		CA	ZERO
		DXCH	TGO		# TGO IN CS AT +28
		TC	S40.134
S40.133		CLEAR	EXIT		# WILL STEER VEHICLE
			IMPULSW
S40.134		TC	PHASCHNG	# KILL GROUP 3
		OCT	3

		TCF	ENDOFJOB

NOTADDUL	TC	INTPRET
		GOTO
			S40.130 +1	# DO NOT COMPENSATE FOR 7 SEC OF ULLAGE
		SETLOC	DAPS7
		BANK

		COUNT	17/40.13

K1VAL		=	EK1VAL		# DP PAD LOAD B+23 NEWTON-SEC/E+2
K2VAL		=	EK2VAL		# DP PAD LOAD B+23 NEWTON-SEC/E+2
K3VAL		=	EK3VAL		# DP PAD LOAD B+09 NEWTONS/E+4
1SEC2D		2DEC	100.0 B-14	# 100.0 CS AT +14
# Page 728
3.5SEC		2DEC	350.0 B-13	# 350 CS AT +13

5SECOND		2DEC	500.0 B-14	# 500.0 CS AT +14

S40.135		2DEC	69.6005183 B-23	# IMPULSE FROM 7.96 SECS OF 2-JET FIRING
					# 	7.96 (199.6)COS(10) LB-SEC, SC.AT
					#	B+23 NEWTON-SEC/E+2 (7 SEC ULLAGE
					#	TO GO, PLUS 0.96 SEC FROM PIPTIME)

# Page 729
# NAME		S40.6 GIMBAL DRIVE TEST AND/OR GIMBAL TRIM
# MOD NO 5				DATE 9 MARCH, 1967
# MOD BY ENGEL				LOG SECTION P40-P47
#
# FUNCTIONAL DESCRIPTION
#	GIMBAL DRIVE TEST....0,+2,-2,0 DEGREE ENGINE COMMANDS, AT 2 SECOND
#		INTERVALS, FIRST IN PITCH, THEN IN YAW.  ASTRONAUT VERIFICATION
#		OF GIMBAL MOTION ON GPI
#	GIMBAL TRIM....AFTER A 4 SECOND DELAY, ENGINE COMMANDED TO
#		PRE-COMPUTED TRIM POSITION.  ASTRONAUT VERIFICATION ON GPI.
#	PRE40.6....RESTART ENTRY TO RE-DO S40.6, ONLY IF RCS IS ON --- IF TVC
#		IS NOT ON --- PRIMARILY TO GET ACTUATORS TRIMMED FOR IGNITION.
#		BYPASS 4 SEC DELAY.  SPEED IS CRITICAL NEAR IGNITION.
#		IF TVC IS ON (TVCDAPON OR LATER) THEN REDOTVC WILL TAKE CARE
#		OF RESTARTING ACTUATORS.
#
# CALLING SEQUENCE....
#	WAITLIST, WITH 2CADR FOR S40.6 (OR PRE40.6), WITH EBANK= CNTR
#
# NORMAL EXIT MODE -- FIXDELAY, TASKOVER
#
# SUBROUTINES CALLED....
#	OUTPUT (INTERNAL)
#	FIXDELAY
#
# ALARM OR ABORT EXIT MODES --- NONE
#
# ERASABLE INITIALIZATION REQUIRED
#	CNTR = +0, NORMALLY SET BY THE P40 CALL AT TST,TRIM.
#	MRKRTMP....POSITIVE FOR GIMBAL DRIVE TEST AND GIMBAL TRIM (BOTH)
#		   NEGATIVE FOR GIMBAL TRIM ONLY
#	PACTOFF, YACTOFF SC.AT 85.41 ARCSEC/BIT (V48N48 P, YTRIM)
#	"SC CONT" SWITCH AT "CMC" (A/P CONTROL SWITCH AT "GNC")
#	ACTIVE SPS GIMBAL MOTOR POWER(S), PITCH, YAW
#
# OUTPUT
#	TVCYAW, TVCPITCH (BITS RELEASED)
#	TVC ENABLE AND OPTICS ERROR COUNTER ENABLE
#
# DEBRIS
#	TBMPR60, CNTR

		BANK	17
		SETLOC	DAPS6
		BANK

		EBANK=	CNTR
		COUNT	20/S40.6

PRE40.6		CS	FLAGWRD6	# RESTART ENTRY TO S40.6 (DO NOT PERMIT
		MASK	OCT60000	#	IF TVC, BITS 15,14 = 1,0)
		EXTEND
		BZMF	+2
		TCF	TASKOVER	# TVC, REDOTVC WILL REESTABLISH INTERFACE

		CS	BIT1		# RCS, SO DO S40.6, GIMTRIM ONLY
# Page 730
		TS	MRKRTMP

		CAF	BIT1		# FOR REVISED S40.6 TIMING FOR RESTARTS...
		TS	CNTR		# TO INDICATE A RESTART ENTRY (CNTR 1S
					#	NORMALLY +0, BY S40.6)

		EBANK=	CNTR
S40.6		CS	ZERO		# INHIBIT OPTICS ACTIVITY
		TS	OPTIND

		CS	BIT2		# DISENABLE OPTICS ERROR COUNTERS (ZERO,
		EXTEND			# 	AND INHIBIT PULSE TRANSMISSION --
		WAND	CHAN12		#	NORMAL STATE)

		CAF	OCT02200	# TVC ENABLE (SPS SERVO AMPS SEE DAC
		EXTEND			#	VOLTAGES) AND DISENGAGE OPTICS/DAC
		WOR	CHAN12

		TC	FIXDELAY	# 60MS PROCEDURAL DELAY (40MS MINIMUM) FOR
		DEC	6		#	RELAY LATCHING

		CAF	BIT2		# ENABLE OPTICS ERROR COUNTERS
		EXTEND
		WOR	CHAN12

		TC	FIXDELAY	# 20MS PROCEDURAL DELAY (4MS MINIMUM) FOR
		DEC	2		#	RELAY LATCHING

RSTRTST		CCS	CNTR		# CHECK FOR RESTART ENTRY (PRE40.6)
		TCF	GIMTRIM +2	# RESTART ENTRY....BYPASS 4 SECOND DELAY
					#	TST,TRIM SETS +0 ON NORMAL ENTRY

		CAE	MRKRTMP		# CHECK FOR TEST/TRIM OR TRIM ONLY
		TS	CNTR		#	MRKRTMP SAVES CNTR FOR RESTARTS
		EXTEND
		BZMF	GIMTRIM		# (TRIM ONLY)

GDTSETUP	CS	ZERO		# GIMBAL DRIVE TEST SETUP, FOR PITCH
		TS	CNTR

GIMDTEST	CAF	+2ACTDEG	# GIMBAL DRIVE TEST, 1ST INCREMENT
		TC	OUTPUT		#	(LEAVES GIMBAL AT +2 DEG)
		CAF	-4ACTDEG	# 2ND INCREMENT (LEAVES GIMBAL AT -2)
		TC	OUTPUT
		CAF	+2ACTDEG	# 3RD INCREMENT (LEAVES GIMBAL AT -0)
		TC	OUTPUT

		CS	CNTR		# CHECK FOR COMPLETION OF YAW TEST.
# Page 731
		CCS	A
		TCF	GIMTRIM		# COMPLETED, GO TO GIMBAL TRIM ROUTINE
		CS	BIT1		# SET UP YAW TEST
		TS	CNTR
		TCF	GIMDTEST	# FOR YAW TEST

OUTPUT		EXTEND			# OUTPUT THE INCREMENT....SAVE Q
		QXCH	TEMPR60

		INDEX	CNTR
		TS	TVCPITCH

		INDEX	CNTR
		CAF	BIT11
		EXTEND
		WOR	CHAN14

		TC	FIXDELAY	# WAIT 2SEC, WHILE ASTRONAUT VERIFIES
		DEC	200		# 	GIMBAL MOTION ON GPI
		TC	TEMPR60

GIMTRIM		TC	FIXDELAY	# WAIT 4 SECONDS BEFORE GIMBAL TRIM
		DEC	400

	+2	CS	ZERO		# PICK UP TRIM VALUES AND OUTPUT THEM
		AD	PACTOFF		#	(AVOID +0) ENTRY POINT FROM RSTRTST
		TS	TVCPITCH	#	ON A RESTART, TO AVOID 4SEC DELAY
		CS	ZERO
		AD	YACTOFF
		TS	TVCYAW

		CAF	PRIO6		# RELEASE THE COUNTERS, BITS 11,12
		EXTEND
		WOR	CHAN14

ENDS40.6	TCF	TASKOVER

OCT02200	OCT	02200		# BITS 8,11 FOR CHANNEL 12 TVC/OPTICS
-4ACTDEG	DEC	-168		# -2(+2ACTDEG), WHOLE BITS, NO ROUNDUP
+2ACTDEG	DEC	+84		# +2 DEG, SC.AT 85.41 ARCSEC/BIT (+84D)

# CALLED BY "DONOUN46" (VERB 48), OR DIRECTLY BY "FRESHDAP" (RCS DAP) VIA IBNKCALL

		COUNT	20/S41.2

S41.2		CA	DAPDATR1
# Page 732
		MASK	THREE
		AD	A
		TS	RATEINDX

		INHINT
		CAE	DAPDATR1	# IS LEM ATTACHED (BITS 14,13 OF DAPDATR1
		MASK	PRIO30		#	=10)
		AD	-BIT14		# (OCT57777)
		EXTEND
		BZF	TOGETHER	# YES

		CS	BIT2		# NO, UNSET FLAG
		MASK	FLAGWRD7
		TS	FLAGWRD7

		TCF	+4

TOGETHER	CS	FLAGWRD7	# ATTACHED, SET FLAG FOR INTEGRATION
		MASK	BIT2
		ADS	FLAGWRD7

		RELINT

		CA	DAPDATR1
		MASK	BIT4
		EXTEND
		BZMF	+2		# DEC 46 MEANS NARROW DB
		CA	DEC409
		AD	DEC46		# DEC 455 MEANS WIDE DB
		TS	ADB

		CA	DAPDATR1
		MASK	BIT7		# QUAD BD
		EXTEND
		BZMF	+2
		CA	ONE
		TS	XTRANS
		CA	DAPDATR1
		MASK	BIT10		# QUAD AC
		EXTEND
		BZMF	+2
		CS	ONE
		ADS	XTRANS

		INHINT
		EXTEND
		BZF	+5		# CLEAR NJETSFLG (4 JETS, OR NO JETS)
		CS	FLAGWRD1	# SET NJETSFLG (2 JETS, AC OR BD QUADS)
		MASK	BIT15		# NJETSFLG = 1 FOR 2 JET ULLAGE (AC OR BD)
		ADS	FLAGWRD1
# Page 733
		TCF	+4
		CS	BIT15		# KJETSFLG = 0 FOR 4 JET (OR 0 JET) ULLAGE
		MASK	FLAGWRD1
		TS	FLAGWRD1
		RELINT
		CA	DAPDATR2
		MASK	BIT13
		EXTEND
		BZMF	+2
		TCF	+2
		CS	ONE
		COM
		TS	ACORBD		# MINUS FOR A-C, PLUS FOR B-D

		CA	DAPDATR2
		MASK	BIT10
		CCS	A
		TCF	+4
		CA	ONE
		TS	RACFAIL
		TCF	BDFAIL
		CA	ZERO
		TS	RACFAIL
		CA	DAPDATR2
		MASK	BIT4
		CCS	A
		TCF	BDFAIL
		CS	ONE
		TS	RACFAIL
BDFAIL		CA	DAPDATR2
		MASK	BIT7
		CCS	A
		TCF	+4
		CA	ONE
		TS	RBDFAIL
		TC	Q
		CA	ZERO
		TS	RBDFAIL
		CA	DAPDATR2
		MASK	BIT1
		CCS	A
		TC	Q
		CS	ONE
		TS	RBDFAIL
		TC	Q

# DAPFIG ENTRY VIA TC POSTJUMP AS JOB FROM "STABLISH" (VERB 46)

		BANK	42
		SETLOC	EXTVBS
# Page 734
		BANK

DAPFIG		CS	BIT9		# TURN OFF SIVB TAKEOVER
		EXTEND
		WAND	CHAN12
		CAE	DAPDATR1	# DETERMINE VEHICLE CONFIGURATION
		EXTEND
		MP	BIT3		#	RIGHT SHIFT 4 OCTAL DIGITS
		MASK	THREE		#	(IN CASE BIT 15 IS USED)
		INDEX	A
		TCF	+1		#	BRANCH BASED ON CONFIG....

		TCF	NODAPUP		# CM.......ACTIVATE NODAP
		TCF	RCSDAPUP	#	CSM......ACTIVATE RCSDAP
		TCF	RCSDAPUP	#	CSM/LEM..ACTIVATE RCSDAP
		TC	POSTJUMP
		CADR	SATSTKON
RCSDAPUP	INHINT			# CALL TO ACTIVATE RCSDAP, AND RETURN
		TCR	IBNKCALL
		CADR	RCSDAPON
		RELINT
		TCF	ENDFIG		# CAME IN VIA V46, GO OUT VIA GOPIN
NODAPUP		EXTEND			# T5 IDLE FOR NODAP (DON'T WORRY ABOUT T)
		DCA	T5IDLDAP
		DXCH	T5LOC
		TC	DOWNFLAG	# RESET T5-USAGE FLAGS FOR NODAP
		ADRES	DAPBIT1		# BIT 15 FLAG 6 = 0
		TC	DOWNFLAG
		ADRES	DAPBIT2		# BIT 14 FLAG 6 = 0
		INHINT
		TC	IBNKCALL	# ZERO JET CHANNELS IN 14 MS AND THEN
		CADR	ZEROJET		# LEAVE THE T6 CLOCK DISABLED.
		RELINT
		CAF	BIT1		# KILL KALCMANU JOB
		TS	HOLDFLAG
ENDFIG		TC	POSTJUMP	# CAME IN VIA V46, GO OUT VIA GOPIN
		CADR	GOPIN
		SBANK=	PINSUPER	# Added by RSB 2009
		EBANK=	PACTOFF
T5IDLDAP	2CADR	T5IDLOC

		SBANK=	LOWSUPER
		BANK	17
		SETLOC	DAPS6
		BANK

DEC409		DEC	409
DEC46		DEC	46

# Page 735

# CALLED BY "DONOUN47" (VERB 48), OR DIRECTLY BY "FRESHDAP" (RCS DAP)
S40.14		CAE	IXX		# RCS ENTRY
		EXTEND
		MP	CONTONE
		TS	J/M

		CA	IAVG
		EXTEND
		MP	CONTONE
		TS	J/M1

		TS	J/M2

		EXTEND
		DCA	CONTTWO
		EXTEND
		DV	IXX
		TS	KMJ

		EXTEND
		DCA	CONTTWO
		EXTEND
		DV	IAVG
		TS	KMJ1

		TS	KMJ2

		TC	Q

CONTONE		DEC	.662034		# 2PI/M
CONTTWO		2DEC	.00118

		COUNT 	24/TVNG

		BANK	31
		SETLOC	P40S
		BANK

POS-2.5		OCT	37405
		EBANK=	DAPDATR1
RCSCADR		2CADR	RCSUP

6SECT5		OCT	37704
		COUNT	21/RCSUP

		BANK	20

		SETLOC	DAPS3
		BANK

# Page 736

RCSUP		LXCH	BANKRUPT
		EXTEND
		QXCH	QRUPT

		TCR	RCSDAPON	# ACTIVATE RCS DAP

		TCF	RESUME

		EBANK=	DAPDATR1
RCSADDR		2CADR	RCSATT

0.6SECT5	OCT	37704

					# RCSDAPON ENTRY MUST BE UNDER INT-INHIBIT
RCSDAPON	CAF	0.6SECT5	# 0.6 SEC ALLOWS TVCEXEC/ROLLDAP TO DIE
	+1	TS	TIME5		# ENTRY FROM R00TOP00
		TS	T5PHASE		# WILL CAUSE FRESHDAP (+1)

		CS	RCSFLAGS	# SET BIT3 TO REINITIALIZE FDAI ERROR
		MASK	BIT3		#	DISPLAY, IN CASE SC CONT SWITCH
		ADS	RCSFLAGS	#	IN SCS NOT GNC (GUIDEMODE PRIMARY)

		EXTEND
		DCA	RCSADDR		# (RCSATT)
		DXCH	T5LOC

		CS	OCT60000	# SEE BITS 15,14 TO 01 TO INDICATE
		MASK	FLAGWRD6	#	T5 TAKEOVER BY RCSDAP
		AD	BIT14
		TS	FLAGWRD6	# KILLS TVCEXEC AND ROLLDAP STARTS

		TC	Q		# RETURN TO CALLER (TVCDAPOF OR RCSDAPUP)


