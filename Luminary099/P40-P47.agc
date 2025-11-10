# Copyright:	Public domain.
# Filename:	P40-P47.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	752-784
# Mod history:	2009-05-19 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-01-06 JL	Fixed interpretive indentation.
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
; FILE: P40-P47.agc
; MODULE: Powered Flight Programs
; MISSION PHASE: lunar-orbit/descent/ascent/trans-earth
;
; TL;DR: Service Propulsion System (SPS), Reaction Control System (RCS), and
;        Ascent Propulsion System (APS) burn execution programs for the Lunar
;        Module. P40 manages SPS burns (large ΔV), P41 handles RCS burns (small
;        adjustments), P42 controls APS burns (ascent), and P47 provides burn
;        alternatives. Includes complete burn sequencing: ullage motor firing,
;        ignition, thrust monitoring, guidance updates, steering commands, and
;        thrust termination based on velocity-to-be-gained (VG) calculations.
;
; COMMENT-ONLY READERS: This file controls the engine firing sequences that
;        change the spacecraft's velocity during orbit maneuvers, descent
;        preparations, and ascent. While the Lunar Module primarily used the
;        Descent Propulsion System (DPS) and Ascent Propulsion System (APS),
;        these routines demonstrate the guidance computer's burn control logic.
;
; CODE-ALONG READERS: Study the integration of burn targeting (from P30_P37),
;        master ignition sequencing (BURN_BABY_BURN), navigation state updates
;        during thrust, guidance computations (S40.1, S40.2,3, S40.8, S40.13),
;        Lambert aimpoint steering (S40.9), and coordinate transformations
;        (S41.1). Note the restart protection via Group 4 and the interaction
;        with the Digital Autopilot (DAP) for attitude control during burns.
; ============================================================================

# Page 752
; ============================================================================
; POWERED FLIGHT PROGRAM SUITE: P40-P47
;
; These programs execute engine burns to change the spacecraft's velocity
; and trajectory. The guidance computer controls burn timing, monitors thrust,
; updates navigation state, computes steering commands, and terminates thrust
; when the desired velocity change (velocity-to-be-gained, or VG) is achieved.
;
; P40: Service Propulsion System (SPS) burns - large ΔV maneuvers
; P41: Reaction Control System (RCS) burns - small ΔV adjustments, manual mode
; P42: Ascent Propulsion System (APS) burns - lunar surface ascent
; P47: Alternative burn program for specialized maneuvers
;
; Historical Note: While the Lunar Module carried a Descent Propulsion System
; (DPS) and an Ascent Propulsion System (APS), the Command/Service Module used
; the Service Propulsion System (SPS). This code was designed to be adaptable
; across different Apollo spacecraft configurations.
; ============================================================================

# PROGRAM DESCRIPTION: P40BOTH		DECEMBER 22, 1966
# MOD 03 BY PETER ADLER			MARCH 3, 1967
# CALLED VIA JOB FROM V37E
#
# FUNCTIONAL DESCRIPTION
#
#	1)	TO COMPUTE A PREFERRED IMU ORIENTATION AND A PREFERRED VEHICLE ATTITUDE FOR A LM DPS
#		THRUSTING MANEUVER.
# 	(There is no item #2 in the original program listing --- RSB 2009.)
#	3)	TO DO THE VEHICLE MANEUVER TO THE THRUSTING ATTITUDE.
#	4)	TO CONTROL THE PGNCS DURING COUNTDOWN, IGNITION, THRUSTING, AND THRUST TERMINATION OF A
#		PGNCS CONTROLLED DPS MANEUVER.
#	5)	IN POSTBURN --- ZERO RENDEZVOUS COUNTER, MAINTAIN VG CALCULATIONS FOR POSSIBLE RCS MANEUVER,
#		SET MAXIMUM DEADBAND IN DAP, RESET STEERLAW CSTEER TO ZERO.
#
#	NOTE:	P42, WHICH IS IN THIS LOG SECTION, DOES THE SAME FOR AN APS BURN, AND P41 DOES 1-3 FOR
#		RCS PLUS DISPLAYS PARAMETERS FOR MANUAL CONTROL.
#
# SUBROUTINES USED
#
#	R02		IMU STATUS CHECK
#	S40.1		COMPUTATION OF THRUST DIRECTION
#	S40.13		LENGTH OF BURN
#	S40.2,3		PREFERRED IMU ORIENTATION
#	S40.8		X PRODUCT STEERING
#	S40.9		LAMBERT VTOGAIN
#	R60LEM		ATTITUDE MANEUVER
#	LEMPREC		EXTRAPOLATE STATE VECTOR
#	PREREAD		AVERAGE G, SERVICER
#	ALLCOAST	DAP COASTING INITIALIZATION
#	CLOKTASK	ERGO CLOCKJOB -- COUNT DOWN
#	PHASCHANG, INTPRET, FLAGUP, FLAGDOWN, WAITLIST, LONGCALL, GOFLASH, GOFLASHR, GOPERF1, ALARM,
#	PRIOLARM, GOTOPOOH, ENDOFJOB, BANKCALL, SETMAXDB, SETMINDB, CHECKMM, FLATOUT, OUTFLAT,
#	KILLTASK, SGNAGREE, TPAGREE, ETC.
#
; ============================================================================
; CREW INTERFACE AND DISPLAYS
;
; The astronauts monitor and control engine burns through the DSKY (Display
; and Keyboard) using a series of verb/noun displays. These displays show
; burn parameters, request crew confirmation at critical points, and provide
; real-time updates during engine firing.
;
; Key DSKY Displays During Burn Sequence:
; V50N25 - Requests crew confirmation: autopilot to Primary Guidance (PGNCS),
;          auto-throttle mode engaged, automatic attitude control enabled
; V06N40 - Shows Time-To-Ignition (TTI), Velocity-to-be-Gained (VG), and
;          delta-V magnitude (DELTAVM), updated once per second by CLOKTASK
; V50N99 - Flashing display requests crew to perform engine-on enable switch
; V06N40 - During burn: Time-Go (TG) to cutoff, VG remaining, DELTAVM
; V16N40 - Final burn results: TG, VG, DELTAVM at thrust termination
; V16N85 - Post-burn: Components of VG in body axes for possible RCS correction
; V05N09 - Displays program alarm codes if anomalies detected
; V50N07 - Requests crew to select Program 00 (idle) after burn complete
;
; Via R30 orbital parameter display routine:
; V06N44 - Altitude at apogee (HAPO), perigee altitude (PERI), time-from-ignition
; V06N35 - Time to next perigee passage, displayed in hours-minutes-seconds
; ============================================================================
;
; ============================================================================
; ALARM CODES AND ABORT LOGIC
;
; The guidance computer monitors for unsafe conditions and alerts the crew:
;
; Alarm 1706: P40 selected when descent stage already staged (descent engine
;             unavailable). Only response: V34E (terminate) returns to Program 00.
;
; Alarm 1703: Time-to-ignition (TIG) less than 45 seconds away - insufficient
;             time for burn preparation. Crew options: V34E (abort burn, return
;             to P00) or V33E (slip TIG forward by 45 seconds, recycle preparation).
; ============================================================================

# RESTARTS VIA GROUP 4
#
# DISPLAYS
#
#	V50N25	203 A/P TO PGNCS, AUTO-THROTTLE MODE, AUTO ATTITUDE CONTROL
#	V06N40	TTI, VG, DELTAVM (DISPLAYED ONCE/SECOND BY CLOKTASK)
#	V50N99	PLEASE PERFORM ENGINE ON ENABLE
#	V06N40	TG (TIME TO GO TO CUTOFF), VG, DELTAVM -- ONCE/SECOND
#	V16N40	FINAL VALUES OF TG, VG, DELTAVM
#	V16N85	COMP OF VG (BODY AXES) FOR POSS. RCS MANUAL MANEUVER
#	V05N09	POSSIBLE ALARMS
#	V50N07	PLEASE SELECT P00
#
# Page 753
#	VIA R30
#
#	V06N44	HAPO, PERI, TFF
#	V06N35	TIME TO PERIGEE, HMS
#
# ALARM OR ABORT EXIT MODES
#
#	PROGRAM ALARM, FLASHING DISPLAY OF ALARM CODE 1706 IF P40 SELECTED WITH DESCENT UNIT STAGED.
#	V34E (TERMINATE) IS THE ONLY RESPONSE ACCEPTED.  TC GOTOPOOH.
#
#	PROGRAM ALARM, FLASH CODE 1703:  TIG LESS THAN 45 SECS AWAY.  V34E= GOTOPOOH OR V33E= SLIP
#	TIG BY 45 SECS.
#
# ERASABLE INITIALIZATION
#
# DEBRIS
#
# OUTPUT
#
#	SEE SUBROUTINES E.G.:  S40.1, S40.2,3, S40.13, S40.8, S40.9, TRIMGIMB
#	XDELVFLG = 1 FOR EXT DELV COMPUTATION
#	         = 0 FOR AIMPT (LAMBERT COMP

		COUNT*	$$/P40
		EBANK=	WHICH

		BANK	36
		SETLOC	P40S
		BANK

; ============================================================================
; TRANSITION: Program Entry Point
;
; The crew has selected a burn program via Verb 37 Enter (V37E) on the DSKY.
; The executive scheduler has initiated this program as a job. The guidance
; computer now begins the burn preparation sequence: checking spacecraft
; configuration, verifying IMU status, loading engine parameters, computing
; the required thrust direction, calculating preferred spacecraft orientation,
; and maneuvering to the burn attitude.
; ============================================================================

; P40LM: Service Propulsion System (SPS) Burn Program
; This program controls large delta-V maneuvers using the Service Module's
; main engine. The burn sequence includes: attitude maneuver to thrust
; orientation, countdown to ignition, engine start verification, closed-loop
; guidance during burn, and thrust termination when velocity-to-be-gained
; reaches zero.

P40LM		TC	PHASCHNG	; Restart protection via Phase Change
		OCT	04024		; Group 4, Phase 2, Priority 4

		CAF	P40ADRES	# INITIALIZATION FOR BURNBABY
		TS	WHICH		; Store P40 address for BURN_BABY_BURN routine

		; Check if Ascent Propulsion System flag is set (APS configuration)
		; If APS is active, branch to alarm - P40 requires SPS/DPS, not APS
		CA	FLGWRD10	; Load flag word 10
		MASK	APSFLBIT	; Check APS flag bit
		CCS	A		; Check result: zero means SPS/DPS OK
		TCF	P40ALM		; APS active - issue alarm 1706
		
		; Verify IMU (Inertial Measurement Unit) is operational
		; IMU provides spacecraft attitude and acceleration measurements
		; essential for guidance during powered flight
		TC	BANKCALL	# GO DO IMU STATUS CHECK ROUTINE.
		CADR	R02BOTH		; Call R02 IMU status check (both CM and LM)

		CS	DAPBOOLS	# INITIALIZE DVMON
		MASK	CSMDOCKD
		CCS	A
		CAF	THRESH1
		AD	THRESH3
		TS	DVTHRUSH
		CAF	FOUR
		TS	DVCNTR
# Page 754
		TC	INTPRET		# LOAD CONSTANTS FOR DPS BURN
		VLOAD	CLEAR		# LOAD F, MDOT, TDECAY
			FDPS
			NOTHROTL
		STORE	F
		SLOAD
			DPSVEX
P40IN		DCOMP	SR1
		STCALL	VEX		# LOAD EXHAUST VELOCITY FOR TGO COMP.
			S40.1		# COMPUTES UT AND VGTIG
		CALL
			S40.2,3		# COMPUTES PREFERRED IMU ORIENTATION
		EXIT

		INHINT
		TC	IBNKCALL
		CADR	PFLITEDB	# ZERO ATTITUDE ERRORS, SET DB TO ONE DEG.

		; Complete burn preparation and transfer to master ignition routine
		TC	P40SXT4		; Perform attitude maneuver, then continue

#	********************************

		; IMU status verified, engine configured, thrust direction computed.
		; Transfer control to BURN_BABY_BURN master ignition routine which
		; handles ullage motor firing, engine start sequencing, ignition
		; verification, and transition to closed-loop guidance.
		TCF	BURNBABY	; Enter burn execution sequence

#	********************************

; P40SXT4: Attitude Maneuver for Burn Preparation
; The spacecraft must rotate to the proper orientation before engine ignition.
; This routine calls R60LEM to perform the attitude maneuver using the Reaction
; Control System (RCS) thrusters to align the spacecraft with the computed
; thrust direction.

P40SXT4		EXTEND
		QXCH	P40/RET		; Save return address in P40/RET
P41MANU		RELINT			; Re-enable interrupts

		; Configure attitude maneuver for vector pointing mode
		; 3AXISFLG cleared: R60 will use VECPOINT for single-axis orientation
		TC	DOWNFLAG	# CLEAR 3AXISFLG -- R60 USE VECPOINT.
		ADRES	3AXISFLG	; Flag address for 3-axis control mode

		; Execute attitude maneuver to burn orientation
		; R60LEM rotates spacecraft to align thrust axis with computed direction
		; using minimum fuel RCS firing logic
		TC	BANKCALL
		CADR	R60LEM		# DO ATTITUDE MANEUVER ROUTINE
		TC	P40/RET		; Return to caller (P40 or P41 sequence)

		EBANK=	TRKMKCNT

; ============================================================================
; POSTBURN: Burn Completion and Final Display
;
; The engine has been shut down and the spacecraft is now coasting. This
; routine displays final burn results to the crew showing the achieved
; velocity change and any residual velocity-to-be-gained that might require
; an RCS correction burn. The crew reviews these values to verify the burn
; accomplished its intended trajectory change.
; ============================================================================

POSTBURN	CA	Z		; Load zero
		TS	DISPDEX		; Clear display index
		EXTEND			; Extended instruction follows
		DCA	ACADN85		; Load address of AVEG (average G) computation
		DXCH	AVEGEXIT	; Store as AVEG exit address
		
		; Display final burn results: Time-Go (should be zero), final VG
		; (velocity-to-be-gained remaining), and total delta-V magnitude
		CAF	V16N40		; Verb 16 Noun 40: Display decimal values
		TC	BANKCALL	; V16N40 shows: TG, VG, DELTAVM
		CADR	GOFLASHR	; Flash display, wait for crew response
		
		; Crew response options:
		TC	TERM40		; ENTER key: Accept results, terminate program
		TCF	TIGNOW		; PROCEED key: Perform RCS trim maneuver
		TC	POSTBURN	; Recycle display if needed
# Page 755
P40PHS1		TC	PHASCHNG
		OCT	00014
		TCF	ENDOFJOB

; ============================================================================
; TIGNOW: RCS Trim Maneuver for Residual Velocity
;
; After main engine shutdown, small residual velocity errors may remain due
; to engine performance variations or guidance uncertainties. This routine
; allows the crew to perform a manual RCS (Reaction Control System) correction
; maneuver to eliminate the remaining velocity-to-be-gained (VG). The DSKY
; displays VG components in body axes so the crew can manually fire RCS jets
; in the appropriate direction using the hand controller.
; ============================================================================

TIGNOW		INHINT			; Inhibit interrupts
		TC	IBNKCALL	; Inter-bank call
		CADR	ZATTEROR	; Zero attitude error for manual control
		TC	IBNKCALL	; Set minimum deadband in DAP
		CADR	SETMINDB	; Allows precise manual RCS control
		RELINT			; Re-enable interrupts
		
		; Display VG components in body axes for manual RCS trim
		; V16N85: Shows velocity-to-be-gained in X, Y, Z body coordinates
		; Crew uses translational hand controller to fire RCS jets manually
		CAF	V16N85B		; Verb 16 Noun 85: VG body axes display
		TC	BANKCALL
		CADR	REFLASHR	; Reflash display, wait for crew
		TC	TERM40		; ENTER: Accept, terminate program
		TCF	TERM40		; PROCEED: Also terminate
		TC	-5		; Recycle: Re-display VG

		TCF	P40PHS1		; Return to phase change sequence

; ============================================================================
; TERM40: Program Termination and Cleanup
;
; The burn program has completed successfully. This routine performs cleanup:
; restoring the DAP attitude deadband to normal coasting values, zeroing the
; rendezvous tracking counter, clearing display index, and returning control
; to Program 00 (idle mode). The spacecraft is now in a stable coasting
; configuration following the completed maneuver.
; ============================================================================

TERM40		EXTEND			; Extended instruction follows
		DCA	SERVCADR	; Load servicer routine address
		DXCH	AVEGEXIT	; Store as average-G exit vector
		CAF	ZERO		; Load zero constant
		TS	TRKMKCNT	# ZERO RENDZVS CNTERS
		CA	Z		; Load zero from Z register
		TS	DISPDEX		; Clear display index
		
		; Restore DAP attitude control deadband to normal (wider) limits
		; During burn, deadband was tightened for precision steering
		INHINT			; Inhibit interrupts
		TC	IBNKCALL
		CADR	RESTORDB	; Restore deadband to coasting configuration
		RELINT			; Re-enable interrupts
		
		; Return to Program 00 idle mode
		TC	GOTOPOOH	; Transfer to P00 (POO = Program Zero Zero)

		EBANK=	WHICH
		COUNT*	$$/P41

; ============================================================================
; P41LM: RCS Manual Burn Program
;
; Program 41 prepares for and monitors manual RCS (Reaction Control System)
; burns where the crew uses the hand controller to fire RCS jets rather than
; having the computer automatically control a main engine. This is used for
; small velocity corrections where manual control provides better situational
; awareness than automated burns.
;
; Unlike P40 (automatic SPS) or P42 (automatic APS), P41 displays the required
; velocity change in body axes and allows the crew to manually fire RCS jets
; in the appropriate direction while monitoring the remaining velocity-to-be-
; gained. This gives the crew direct control for precision maneuvers.
; ============================================================================

P41LM		CAF	P41ADRES	# INITIALIZATION FOR BURNBABY
		TS	WHICH		; Store P41 address for burn sequencing

		; Check IMU status before maneuvering
		TC	BANKCALL
		CADR	R02BOTH		; R02: IMU alignment status check

		; Enter interpretive mode for vector/matrix calculations
		TC	INTPRET		# BOTH LM
		
		; Select appropriate RCS thrust level based on jet configuration
		; NJETSFLG indicates whether using 2-jet or 4-jet RCS mode
		BON	DLOAD		# IF NJETSFLAG IS SET, LOAD Z JET F
			NJETSFLG	; Branch if using 2-jet translation mode
			P41FJET1	; Use 2-jet thrust constant
			FRCS4		# IF NJETSFLG IS CLEAR, LOAD 4 JET F

P41FJET		STCALL	F		; Store thrust constant F, call initialization
			P41IN
			
P41FJET1	DLOAD			; Load 2-jet RCS thrust constant
# Page 756
			FRCS2		; 2-jet RCS thrust force
		STORE	F		; Store as F for burn calculations

	; Compute thrust direction and velocity-to-be-gained at ignition time
	; S40.1 calculates unit thrust vector UT and velocity change VGTIG
P41IN		CALL
			S40.1		# BOTH
			
	; Calculate preferred IMU orientation for manual RCS control
	; S40.2,3 computes optimal IMU gimbal angles to minimize RCS fuel usage
P41NORM		CALL
			S40.2,3		# CALCULATE PREFERRED IMU ORIENTATION AND
		EXIT			# SET PFRATFLG.

	; Initialize attitude control for manual RCS burn
	; Zero attitude error counters and set tight deadband for crew control
		INHINT
		TC	IBNKCALL
		CADR	ZATTEROR	# ZERO ATTITUDE ERRORS
		TC	IBNKCALL
		CADR	SETMINDB	# SET 0.3 DEGREE DEADBAND
		TC	P40SXT4		; Perform attitude maneuver to burn attitude

	; Transform velocity-to-be-gained from inertial reference frame
	; to LM body axes for display to crew on DSKY
		TC	INTPRET
		VLOAD	CALL		# TRANSFORM VELOCITY-TO-BE-GAINED AT TIG
			VGTIG		# FROM REFERENCE COORDINATES TO LM BODY-
			S41.1		# AXIS COORDINATES FOR V16N85 DISPLAY.
		STORE	VGBODY		# (SCALED AT 2 (+7) METERS/CENTISECOND)
		EXIT

	; Display V16N85: VG in body axes for manual RCS burn execution
	; Crew uses these values to monitor burn performance manually
		CAF	V16N85B
		TC	BANKCALL
		CADR	GODSPRET

	; Start dynamic display job to continuously update burn parameters
		CAF	PRIO5
		TS	DISPDEX		# FOR SAFETY ONLY
		TC	FINDVAC
		EBANK=	VGPREV
		2CADR	DYNMDISP

	; Configure restart protection groups for burn monitoring phase
		TC	2PHSCHNG
		OCT	00076		# GROUP 6 RESTARTS AT REDO6.7
		OCT	04024		# GROUP 4 RESTARTS HERE

#	********************************

	; Transfer to burn monitoring routine (common path with P40/P42)
		TCF	B*RNB*B*

#	********************************

	; Wait routine for display blanking period (TIG-35 to TIG-30 seconds)
	; Display blanked to allow crew focus during critical countdown phase
BLNKWAIT	CAF	1SEC
		TC	BANKCALL
		CADR	DELAYJOB

	; Restart entry point for display update loop
	; Checks if we're in blanking period before restoring display
REDO6.7		CA	DISPDEX		# ON A RESTART, DO NOT PUT UP DISPLAY IF
		AD	TWO		# BLANKING (BETWEEN TIG-35 AND TIG-30)

#	********************************
# Page 757

		EXTEND
		BZF	BLNKWAIT	; If in blanking period, wait

	; Restore V16N85 display after blanking or restart
		CAF	V16N85B
		TC	BANKCALL
		CADR	GODSPRET

	; Reset job priority for display update task
		CAF	PRIO5
		TC	PRIOCHNG

	; Dynamic display update job - runs once per second before TIG-35
	; Continuously updates VG display as trajectory calculations refine
DYNMDISP	CA	DISPDEX		# A NON-POSITIVE DISPDEX INDICATES PAST
		EXTEND			# TIG-35, SO SERVICER WILL BE DOING THE
		BZMF	ENDOFJOB	# UPDATING OF NOUN 85.  STOP DYNMDISP.
		TC	INTPRET
		VLOAD	CALL		; Transform latest VG to body axes
			VGPREV
			S41.1
		STORE	VGBODY
		EXIT
		CAF	1SEC
		TC	BANKCALL
		CADR	DELAYJOB	; Wait 1 second
		TCF	DYNMDISP	; Loop back for next update

	; N85 calculation routine called by servicer after TIG-35
	; Updates VG display during final countdown and burn
CALCN85		TC	INTPRET
		CALL
			UPDATEVG	; Recalculate velocity-to-be-gained
		VLOAD	CALL
			VGPREV
			S41.1		; Transform to body axes
		STORE	VGBODY
		EXIT
		TC	POSTJUMP
		CADR	SERVEXIT

; ============================================================================
; TRANSITION: From P41 (RCS manual burn) to P42 (APS automatic burn)
;
; P42 manages automatic APS (Ascent Propulsion System) burns for the LM.
; Used during lunar ascent from surface to orbit, P42 controls ignition,
; thrust monitoring, and cutoff for the APS engine. During Apollo 11's
; ascent from Tranquility Base on July 21, 1969, this program lifted the
; Eagle back into orbit for rendezvous with Columbia.
; ============================================================================

		COUNT*	$$/P42
		EBANK=	WHICH

; P42LM: Ascent Propulsion System (APS) automatic burn program
; Entry point for LM ascent burns using APS engine
; Performs: attitude setup, burn monitoring, thrust termination
P42LM		TC	PHASCHNG
		OCT	04024		; Restart protection group 4

		CAF	P42ADRES	# INITIALIZATION FOR BURNBABY.
		TS	WHICH		; Store engine type for BURNBABY routine

	; Verify ascent stage is present (APS only available after staging)
	; If descent stage still attached, alarm and terminate
		CS	FLGWRD10
		MASK	APSFLBIT	; Check ascent stage flag
		CCS	A
		TC	P40ALM		; Alarm if APS not available
		
; P42STAGE: Entry point after ascent staging verification complete
P42STAGE	TC	BANKCALL
# Page 758
		CADR	R02BOTH		; IMU status check and initialization

	; Initialize delta-V monitoring thresholds for APS burn
	; DVMON tracks velocity accumulation during thrust
		CAF	THRESH2		# INITIALIZE DVMON
		TS	DVTHRUSH	; Set velocity threshold
		CAF	FOUR
		TS	DVCNTR		; Initialize counter for 4 samples

	; Load APS engine parameters into thrust calculation variables
	; FAPS = APS thrust (Newtons), MDOTAPS = propellant flow rate (kg/s)
		TC	INTPRET
		SET	VLOAD		# LOAD FAPS, MDOTAPS, AND ATDECAY INTO
			AVFLAG		# F, MDOT, AND TDECAY BY VECTOR LOAD.
			FAPS		; APS thrust vector
		STORE	F		; Store as active thrust parameter
		SLOAD	GOTO
			APSVEX		; APS exhaust velocity
			P40IN		; Jump to common initialization routine

		EBANK=	WHICH

; ============================================================================
; TRANSITION: From P42 (APS automatic burn) to P47 (midcourse correction)
;
; P47 executes midcourse trajectory correction maneuvers during cislunar
; coast phases. These small velocity adjustments correct navigation errors
; accumulated during long coast periods between major burns, ensuring the
; spacecraft arrives at its intended target with acceptable accuracy.
; ============================================================================

		COUNT*	$$/P47

; P47LM: Midcourse trajectory correction program
; Entry point for small ΔV corrections during trans-lunar or trans-earth coast
; Used to correct navigation errors before major maneuvers
P47LM		TC	BANKCALL
		CADR	R02BOTH		; IMU status check and initialization

	; Call MIDTOAV2 to compute midcourse correction requirements
	; Compares current trajectory with desired trajectory
		TC	INTPRET
		CALRB
			MIDTOAV2	; Compute required velocity change

	; Schedule P47 main body execution after computation completes
		CA	MPAC +1		; Result from MIDTOAV2
		TC	TWIDDLE		; Schedule task
		ADRES	STARTP47	; Jump to main P47 routine

		TCF	ENDOFJOB	; Return to scheduler

; STARTP47: Main body of P47 midcourse correction program
; Performs velocity change calculation and displays results to crew
STARTP47	TC	PHASCHNG
		OCT	05014		; Restart protection group 5
		OCT	77777

	; Set up average G computation exit handler
	; CALCN83 computes delta-V in IMU coordinates
		EXTEND
		DCA	ACADN83		; Address of CALCN83 routine
		DXCH	AVEGEXIT	; Store as average G exit routine
		CAF	PRIO20
		TC	FINDVAC		; Find available VAC area
		EBANK=	DELVIMU
		2CADR	P47BODY		; Execute P47 main body

		TCF	REDO4.2		# CHECKS PHASE 5 AND GOES TO PREREAD
					# SEE TIG-30 IN BURNBABY

; CALCN83: Compute delta-V in IMU coordinates
; Called as exit from average G computation
; Transforms velocity change from control to IMU reference frame
CALCN83		TC	INTPRET
	; Add control system delta-V to reference delta-V
	; Result is total velocity change required in control coordinates
		VLOAD	VAD
			DELVCTL		; Control system velocity change
			DELVREF		; Reference velocity change
		STORE	DELVSIN		# TEMP STORAGE FOR RESTARTS
# Page 759
	; Transform velocity change from control to IMU coordinates
	; S41.1 performs coordinate transformation using current REFSMMAT
		CALL
			S41.1		; Control to IMU transformation
		STORE	DELVIMU		; Store delta-V in IMU coordinates
		EXIT
		TC	PHASCHNG
		OCT	10035		# REREADAC AND HERE

	; Restore delta-V in control coordinates for display
		TC	INTPRET
		VLOAD
			DELVSIN		; Retrieve from temporary storage
		STORE	DELVCTL		; Restore control coordinate delta-V
		EXIT

	; Return to servicer routine
		TC	POSTJUMP
		CADR	SERVEXIT	; Exit to servicer

; P47BOD: Display midcourse correction parameters to crew
; Verb 16 Noun 83: Display computed delta-V components
; Crew can proceed with maneuver or terminate program
P47BOD		CAF	V1683		; Verb 16 Noun 83
		TC	BANKCALL
		CADR	GOFLASHR	; Display and wait for response
		TC	GOTOPOOH	; V34: Terminate program
		TC	GOTOPOOH	; Backup terminate

	; V33 (Proceed): Continue to next phase
		TCF	P47BODY		; Recycle to recompute

	; Return to P40 Phase 1 for burn execution
		TCF	P40PHS1		; Continue to burn sequence

; P47BODY: Initialize P47 midcourse correction computation
; Sets delta-V vectors to zero before computation
; Main entry point for P47 computation cycle
P47BODY		TC	INTPRET
	; Initialize delta-V vectors to zero
		VLOAD
			HI6ZEROS	; Load zero vector
		STORE	DELVIMU		; Zero IMU delta-V
		STORE	DELVCTL		; Zero control delta-V
		EXIT
		TC	P47BOD		; Display results to crew

; ============================================================================
; SHARED BURN TERMINATION ROUTINES (P40-P47)
;
; The following routines handle engine cutoff and post-burn initialization
; for all P40-series programs. IMPLBURN schedules engine-off task, clears
; burn-related flags, and disables ullage motors. ENGINOFF initiates the
; post-burn cleanup and coast phase setup.
; ============================================================================

		COUNT*	$$/P40

; IMPLBURN: Impulse burn termination (instant cutoff)
; Used for very short burns or impulsive maneuvers
; Schedules immediate engine shutdown and clears burn flags
IMPLBURN	CA	TGO 	+1	; Load time-to-go (low word)
		TC	GETDT		; Convert to waitlist delta-time
		TC	TWIDDLE		; Schedule task on waitlist
		ADRES	ENGOFTSK	; Engine-off task

	; Clear all burn-related status flags
		TC	DOWNFLAG	# TURN OFF IGNFLAG
		ADRES	IGNFLAG		; Ignition flag off
		TC	DOWNFLAG	# TURN OFF ASTNFLG
		ADRES	ASTNFLAG	; Astronaut takeover flag off
		TC	DOWNFLAG	# TURN OFF IMPULSW
		ADRES	IMPULSW		; Impulse switch flag off

	; Protect engine-off task from restart disruption
		TC	PHASCHNG	# RESTART PROTECT ENGOFTSK (ENGINOFF)
		OCT	40114		; Phase change protection

	; Brief delay before disabling ullage motors
	; Ensures propellant has settled before ullage cutoff
		TC	FIXDELAY	# WAIT HALF A SECOND
		DEC	50		; 500 milliseconds
# Page 760
		TC	NOULLAGE	# TURN OFF ULLAGE

		TC	TASKOVER	; Return to waitlist scheduler

; ENGOFTSK: Waitlist task wrapper for engine shutdown
; Allows ENGINOFF to be called from waitlist or direct bank call
; Provides consistent interface for burn termination
ENGOFTSK	TC	IBNKCALL	# THIS CODING ALLOWS ENGINOFF ET AL TO BE
		CADR	ENGINOFF	# USED BOTH BY WAITLIST AND BY TC IBNKCALL
		TC	TASKOVER	; Return to waitlist scheduler

; ENGINOFF: Main engine cutoff and post-burn initialization
; Schedules post-burn processing at lower priority than time-critical tasks
; Begins transition from powered flight to coast phase
ENGINOFF	CAF	PRIO12		# MUST BE LOWER PRIO THAN CLOCKJOB
		TC	FINDVAC		; Find available VAC area
		EBANK=	TRKMKCNT	; Set E-bank for post-burn variables
		2CADR	POSTBURN	; Schedule post-burn cleanup routine

ENGINOF2	CAF	BIT1
		TC	WAITLIST
		EBANK=	OMEGAQ
		2CADR	COASTSET

ENGINOF1	CS	FLAGWRD7	# SET THE IDLE BIT.
		MASK	IDLEFBIT
		ADS	FLAGWRD7

		TC	NOULLAGE

ENGINOF4	EXTEND
		DCA	TIME2
		DXCH	TEVENT

ENGINOF3	CS	ENGONBIT	# INSURE ENGONFLG IS CLEAR.
		MASK	FLAGWRD5
		TS	FLAGWRD5
		CS	PRIO30		# ENGINOF3 IS USED AS A PRE-ENGINE ARM
		EXTEND			# SUBROUTINE.
		RAND	DSALMOUT
		AD	PRIO20		# TURN OFF THE ENGINE -- DPS OR APS
		EXTEND
		WRITE	DSALMOUT

		CS	DAPBOOLS	# TURN OFF TRIM GIMBAL
		MASK	USEQRJTS
		ADS	DAPBOOLS

		CS	HIRTHROT	# ZERO AUTO-THROTTLE WHENEVER THE ENGINE
		TS	THRUST		# IS TURNED OFF.
		CAF	BIT4		# THE HARDWARE DOES SO ONLY WHEN THE
		EXTEND			# ENGINE IS DISARMED.
		WOR	CHAN14

		TC	ISWRETRN
# Page 761
COASTSET	TC	IBNKCALL	# DO DAP COASTING INITIALIZATION
		CADR	ALLCOAST
		TC	TASKOVER

		EBANK=	OMEGAQ
UPDATEVG	STQ	CALL
			QTEMP1
			S40.8		# X-PRODUCT STEERING
		BON	BON
			XDELVFLG
			QTEMP1
			NORMSW
			180SETUP
		DLOAD	DSU
			PIPTIME
			TIGSAVE
		DSU	BMN
			TNEWA
			GETRANS
		DLOAD	DAD
			TIGSAVE
			TNEWA
		STORE	TIGSAVEP
180SETUP	EXIT
		CCS	PHASE2
		TCF	NO.9
		CAF	PRIO10
		INHINT
		TC	FINDVAC
		EBANK=	VG
		2CADR	S40.9		# LAMBERT VTOGAIN

		TC	2PHSCHNG
		OCT	00172		# 2.17SPOT FOR S40.9
		OCT	10035		# HERE AND REREADAC AFTER RESTART

ENDSTEER	TC	INTPRET
		DLOAD
			TIGSAVEP
		STOVL	TIGSAVE
			RN
		STOVL	RINIT
			VN
		STORE	VINIT
GETRANS		DLOAD	DSU
			TPASS4
			PIPTIME
		STCALL	DELLT4
			QTEMP1

# Page 762
NO.9		TC	INTPRET
		GOTO
			QTEMP1
STEERING	TC	INTPRET

		CALL
			UPDATEVG
		EXIT

		EBANK=	DVCNTR
NSTEER		INHINT
		CA	EBANK7
		TS	EBANK
		CS	FLAGWRD2	# CHECK IMPULSE SWITCH.  IT IS SET EITHER
		MASK	IMPULBIT	# BY S40.13 IF TBURN<6 SECS OR BY S40.8 IF
		CCS	A		# STEERING IS ALMOST DONE.

		TCF	+5		# IMPULSW = 0	EXIT
		CS	FLAGWRD7	# IMPULSW = 1	WHY?  CHECK IDLEFLAG
		MASK	IDLEFBIT	#	(IDLEFLAG = 0 --> DVMON ON)
		CCS	A
		TCF	+3		# DVMON ON --> THRUSTING --> IMPULSW VIA S40.8
		TC	POSTJUMP	# DVMON OFF --> IMPULSW ON VIA S40.13 --> EXIT
		CADR	SERVEXIT

		TC	IBNKCALL
		CADR	STOPRATE

		TC	DOWNFLAG	# TURN OFF IMPULSW
		ADRES	IMPULSW

		TC	UPFLAG
		ADRES	IDLEFLAG	# TURN OFF DVMON

		INHINT
		EXTEND
		DCA	TIG
		DXCH	MPAC
		EXTEND
		DCS	TIME2
		DAS	MPAC
		TC	TPAGREE
		CAE	MPAC 	+1
		TC	GETDT
		TC	TWIDDLE
		ADRES	ENGOFTSK
		TC	2PHSCHNG
		OCT	40114		# ENGOFTSK (ENGINOFF)
		OCT	00035		# SERVICER -- REREADAC
# Page 763
		TCF	ENDOFJOB

GETDT		CCS	A
		TCF	+3
		TCF	+2
		CAF	ZERO
		AD	ONE
		XCH	L
		CAF	ZERO
		DXCH	TGO
		CA	TGO 	+1
		TC	Q

# **************************************

SEC15DP		OCT	00000		# DON'T SEPARATE
SEC15		DEC	1500		# DON'T SEPARATE
SEC30DP		2DEC	3000

SEC45DP		OCT	00000		# DON'T MOVE FROM JUST BEFORE SEC45
SEC45		DEC	4500
5SECDP		OCT	00000		# DON'T MOVE FROM JUST BEFORE 5SEC
5SEC		DEC	500
26SECS		DEC	2600
V16N40		VN	1640
V16N85B		VN	1685
V1683		VN	1683
SEC01		=	1SEC
ACADN85		=	P41TABLE +2

		EBANK=	DELVIMU
ACADN83		2CADR	CALCN83

# ============================================================================
# SUBROUTINE: S40.1 - Compute Initial Thrust Direction and Velocity-to-Gain
# ============================================================================
#
# This subroutine computes the initial thrust direction (UT) and initial
# velocity-to-be-gained vector (VGTIG) at time of ignition (TIG). It supports
# two burn modes:
#
# 1. DELTA-V MANEUVER (XDELVFLG=1): Fixed velocity change in inertial coords
#    - Uses pre-computed DELVSIN vector (desired delta-v)
#    - Thrust direction aligns with required velocity change
#    - Used for precise orbital adjustments and rendezvous burns
#
# 2. AIMPOINT STEERING (XDELVFLG=0): Lambert targeting to reach position
#    - Uses RTARG (target position) and DLTARG (time to target)
#    - Computes trajectory to intercept specific location
#    - Used for landing site targeting and trajectory corrections
#
# The computed thrust direction (UT) becomes input to the attitude control
# system (R60LEM) which maneuvers the spacecraft to point engines correctly.
# The velocity-to-gain (VGTIG) initializes the guidance equations that will
# continuously update thrust direction throughout the burn.
#
# COMMENT-ONLY READERS: This is where the guidance computer calculates which
# direction to point the engine before ignition. For delta-v burns, it points
# along the desired velocity change. For aimpoint steering, it solves the
# geometry problem of hitting a target position at a specific time.
#
# CODE-ALONG READERS: This subroutine uses the interpreter for vector math.
# Key operations include VXSC (vector times scalar), UNIT (normalize vector),
# and VSQ (vector dot product with itself). Scaling: positions B29 meters,
# velocities B7 meters/centisecond, UT output B2 dimensionless unit vector.
# ============================================================================

# ******************************************
# Page 764
# PROGRAM DESCRIPTION: S40.1		DATE: 15 NOV 66
# MOD N02				LOG SECTION P40-P47
# MOD BY ZELDIN AND ADAPTED BY TALAYCO
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
#	LEMPREC
#	INITVEL
#	CALCGRAV
#	MIDGIM
#
# ALARM OR ABORT EXIT MODES
#	L+2 OF CALLING SEQUENCE, UNSOLVABLE CONIC IF NOSOFLAG=1
#
# ERASABLE INITIALIZATION REQUIRED
#	WEIGHT/G	ANTICIPATED VEHICLE MASS	DP B16 KGM
#	XDELVFLG	1=DELTA-V MANEUVER, 0=AIMPT STEER
#	F		THRUST FOR ENGINE USED
#   IF DELTA-V MANEUVER:
#	DELVSIN		SPECIFIED DELTA-V REQUIRED IN
#			INERTIAL COORDS. OF ACTIVE VEHICLE
#			AT TIME OF IGNITION		VECTOR B7 M/CS
#	DELVSAB		MAG. OF DELVSIN			DP B7 M/CS
#	RTIG		POSITION AT TIME OF IGNITION	VECTOR B29 M
#	VTIG		VELOCITY AT TIME OF IGNITION	VECTOR B7 M/CS.
#   IF AIMPT STEER:
#	TIG		TIME OF IGNITION		DP B28 CS
#	RTARG		POSITION TARGET TIME		VECTOR B29 M
#	CSTEER		C FOR STEER LAW			DP B2
#	DLTARG		TARGET TIME-IGNITION TIME	DP B28 CS
#
# OUTPUT
#	UT		DESIRED THRUST DIRECTION	VECT. B2 M/(CS.CS)
#	VGTIG		INITIAL VALUE OF VELOCITY
#			TO BE GAINED (INERT. COORD.)	VECTOR B7 M/CS
#	DELVLVC		VGTIG IN LOC. VERT. COORDS.	B7 M/CS
#	BDT		V REQUIRED AT TIG -V REQUIRED AT (TIG-2SEC)
#	   -GDT		FOR S40.13			VECT B7 M/CS
#	RTIG		CALC IN S40.1B (AIMPT) FOR S40.2,3	VECTOR B27M
#			POSITION AT TIME OF IGNITION
#
# DEBRIS	QTEMP1
#		MPAC, QPRET
#		PUSHLIST

		BANK	14
		SETLOC	P40S1
		BANK
# Page 765
		COUNT*	$$/S40.1
# S40.1 Entry Point
# Save ignition time (TIG) and test which burn mode is active.
S40.1		STQ	DLOAD
			QTEMP
			TIG
		STORE	TIGSAVE
# Test XDELVFLG: If set (=1), use delta-v mode. If clear (=0), branch to
# S40.1B for aimpoint steering mode.
DELVTEST	BOFF
			XDELVFLG
			S40.1B
# DELTA-V MODE: Compute thrust direction from specified delta-v vector.
# This section calculates the thrust angle (THETAT) and initial velocity-to-
# gain vector. The algorithm computes UT (up vector) perpendicular to the
# orbital plane, then decomposes DELVSIN into parallel and perpendicular
# components relative to UT.
CALCTHET	SETPD	VLOAD
			0
			VTIG
		STORE	VINIT
		VXV	UNIT
			RTIG
		STOVL	UT		# UP IN UT
			RTIG
		STORE	RINIT
# Calculate thrust angle THETAT from orbit geometry and burn parameters.
# Formula: THETAT = f(orbital radius, mass ratio, delta-v magnitude).
# THETACON is a constant related to steering geometry. WEIGHT/G and F are
# mass ratio terms. DELVSAB is the magnitude of the desired delta-v.
		VSQ	PDDL
			36D
		DMP	DDV
			THETACON
		DMP	DMP
			DELVSAB
			WEIGHT/G
		DDV
			F
		STOVL	14D
			DELVSIN
# Decompose DELVSIN into components parallel and perpendicular to UT.
# Parallel component: (DELVSIN . UT) * UT
# Perpendicular component: DELVSIN - parallel component
# This allows steering logic to handle out-of-plane velocity changes.
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
# Page 766
		STORE	VGTIG		# VG IGNITION SCALED AT 2(+7) M/CS
# Final VGTIG calculation combines perpendicular and parallel components
# using trigonometric functions of THETAT/2. This produces the initial
# velocity-to-be-gained vector that will be updated during the burn by
# the guidance steering equations (S40.8).
		UNIT
		STOVL	UT		# THRUST DIRECTION SCALED AT 2(+1)
			VGTIG
# Convert VGTIG to local vertical coordinates for use by guidance.
		PUSH	CALL
			GET.LVC		# VGTIG IN LV COOR AT 2(+7) M/CS IN DELVLVC
		GOTO
			QTEMP
# ============================================================================
# S40.1B - AIMPOINT STEERING MODE (Lambert Targeting)
# ============================================================================
# This branch executes when XDELVFLG=0, indicating the burn is targeted to
# reach a specific position (RTARG) at a specific time (TIG + DLTARG).
# The Lambert problem solver (INITVEL) computes the required initial velocity
# to achieve the intercept. This mode is used for landing site targeting and
# trajectory corrections where the goal is to reach a specific location rather
# than achieve a specific velocity change.
S40.1B		DLOAD
			TIG
		STORE	TDEC1
		BDSU
			TPASS4
# Extrapolate state vector from current time (TPASS4) to ignition time (TIG).
		STCALL	DELLT4		# INTERCEPT TIME -- TIG.
			LEMPREC
		VLOAD	SETPD		# LOAD STATE VECTOR AT TIG FOR INITVEL.
			RATT
			0
		STORE	RTIG
		STORE	RINIT
		UNIT
		STOVL	UNIT/R/
			VATT
		STORE	VTIG
		STORE	VINIT
# Prepare parameters for Lambert problem solver (INITVEL).
# NUMIT = 0 (iteration counter), EPS1 = 10 degrees, EPS2 = 35 degrees.
# The epsilon values control the convergence tolerance for the iterative
# Lambert solution. NORMSW flag selects between normal (10°) and coarse
# (35°) tolerance modes.
		DLOAD	PDDL		# NUMIT = 0
			ZEROVECS
			EPS1
		BOFF	DAD
			NORMSW
			SMALLEPS
			EPS2		# EPSILON4 = 10 DEGREES OR 45 DEGREES.
SMALLEPS	PUSH	SXA,1
			RTX1
		SXA,2	CALL
			RTX2
# Call Lambert problem solver (INITVEL) to compute the required velocity
# at TIG that will cause the spacecraft to reach RTARG at time TIG + DLTARG.
# The solution accounts for gravitational perturbations during coast and
# provides the required delta-v in DELVEET3.
			INITVEL
		VLOAD	PUSH
			DELVEET3	# VGTIG = VR - VN.
		STORE	VGTIG
# Compute thrust direction unit vector (UT) from velocity-to-be-gained.
# The required delta-v (VGTIG) from INITVEL defines the thrust direction.
		UNIT			# UT = UNIT (VGTIG)
		STODL	UT
			36D
# Convert VGTIG to local vertical coordinates for display to crew.
# The GET.LVC subroutine transforms the inertial velocity vector into
# the up-cross-range-downrange coordinate frame relative to the target
# position, providing intuitive display values.
		STCALL	VGDISP		# CONVERT VGTIG (IN PUSHLIST) TO LOCAL
			GET.LVC		# VERTICAL COORDINATES.
		GOTO
			QTEMP

EPS1		2DEC*	2.777777778 E-2*	# 10 DEGREES AT 1 REVOLUTION

# Page 767
EPS2		2DEC*	9.722222222 E-2*	# 35 DEGREES AT 1 REVOLUTION.

THETACON	2DEC	.31830989 B-8

# Page 768
# SUBROUTINE NAME:  S40.2,3		MOD. NO. 3, DATE APRIL 4, 1967
# MODIFICATION BY:  JONATHON D. ADDELSTON (ADAMS ASSOCIATES)
# MOD. NO. 4:  JULY 18, 1967: PETER ADLER (MIT/IL)
# MOD. NO. 5:  OCTOBER 18, 1967:  PETER ADLER (MIT/IL)
# ORIGINALLY BY:  SAYDEAN ZELDIN (MIT INSTRUMENTATION LAB) AND RICHARD TALAYCO (SYSTEM DELVELOPMENT CORP)
#
# S40.2,3 COMPUTES "POINTVSM" WHICH IS THE HALF-UNIT DESIRED THRUST VECTOR IN STABLE-MEMBER COORDINATES FROM "UT"
# WHICH IS THE SAME VECTOR IN REFERENCE COORDINATES.  IT DETERMINES THE CORRECT VALUES FOR "SCAXIS" USING THE +X
# AXIS FOR DPS, APS, AND RCS BURNS.  THE "WINGS-LEVEL HEADS-UP" LM ORIENTATION IS THEN COMPUTED IN REFERENCE
# COORDINATES.  THESE VECTORS ALSO DEFINE THE "PREFERRED IMU ORIENTATION".  UPON COMPLETION OF THIS CALCULATION,
# THE "PREFERRED ATTITUDE COMPUTED" FLAG IS SET (PFRATFLG).
#
# CALLING SEQUENCE:
#	L	CALL			# INTERPRETIVE CALL.
#	L +1		S40.2,3
#	L +2	(RETURN)		# GIMBAL ANGLE VECTOR IN MPAC.
#
# SUBROUTINES CALLED:  NONE.
#
# NORMAL RETURN:  L +2 (SEE CALLING SEQUENCE ABOVE).
#
# ALARM/ABORT MODES:  NONE.
#
# INPUT:
#	1.	REFSMMAT	MATRIX FROM REFERENCE TO STABLE-MEMBER COORDINATES SCALED AT 2.
#	2.	UT		HALF-UNIT DESIRED THRUST DIRECTION.
#	3.	RTIG		POSITION AT TIG IN REFERENCE COORDINATES.
#
# OUTPUT:
#	1.	`XSCREF'	WINGS-LEVEL HEADS-UP LM ORIENTATION
#		`YSCREF'	IN REFERENCE COORDINATES
#		`ZSCREF'	(PREFERRED IMU ORIENTATION).
#	2.	POINTVSM	DESIRED THRUST DIRECTION IN STABLE-MEMBER COORDINATES.
#	3.	SCAXIS		HALF-UNIT OF AXIS TO ALIGN IN STABLE-MEMBER COORDINATES.
#	4.	PFRATFLG	INTERPRETIVE FLAG.  ON: PREFERRED ORIENTATION COMPUTED; OFF: NOT COMPUTED.
#
# DEBRIS:  NONE
# Page 769
# ============================================================================
# S40.2,3 - COMPUTE PREFERRED IMU ORIENTATION AND VEHICLE ATTITUDE
# ============================================================================
# This subroutine calculates the "wings-level heads-up" LM orientation for
# a thrusting maneuver. It transforms the desired thrust direction (UT) from
# reference coordinates to stable-member (IMU) coordinates, then computes
# the spacecraft body axes that define the preferred attitude. For the LM,
# this assumes +X axis thrust direction (DPS, APS, or RCS burns).
#
# The "preferred IMU orientation" is defined by:
#   XSCREF = Thrust direction (along velocity-to-be-gained)
#   YSCREF = Cross-product of thrust direction and position vector (perpendicular)
#   ZSCREF = Completes right-handed coordinate system
#
# This orientation provides optimal visibility and control authority during burns.

		COUNT*	$$/S40.2
# Transform thrust direction vector from reference coordinates to stable-member
# (IMU) coordinates. This defines the direction the spacecraft must point
# relative to the IMU platform orientation.
S40.2,3		VLOAD			# UT:  DESIRED THRUST DIRECTION (HALF-UNIT)
			UT		# (PUT INTO TOP OF PUSH-DOWN-LIST.)
		MXV	VSL1		# TRANSFORM THRUST DIRECTION TO STABLE-
			REFSMMAT	# MEMBER FROM REFERENCE COORDS (RESCALE).
		STOVL	POINTVSM	# SAVE FOR "VECPOINT" ROUTINE (LEMMANU).
# Set spacecraft axis for alignment. The LM always uses +X axis thrust
# direction for DPS, APS, and most RCS maneuvers.
			UNITX		# SCAXIS SET TO +X, FOR P40 AND P42 AND
		STOVL	SCAXIS		# FOR P41 IF RCS NOT -X,+Y,-Y,+Z,-Z.

# Compute the spacecraft reference axes defining the preferred orientation.
# XSCREF = Thrust direction (along velocity-to-be-gained)
# YSCREF = Perpendicular to thrust and position vector (cross-product direction)
# ZSCREF = Completes right-handed coordinate system
			UT		# ASSUME +X BURN ALWAYS, EVEN FOR RCS.
PLUSX		STORE	XSCREF		# XSCREF = UT (DESIRED THRUST DIRECTION)
		VXV	UNIT		# RTIG = POSITION AT TIME-OF-IGNITION.
			RTIG		# YSCREF = UNIT(UT X RTIG)
# Check if thrust direction is nearly radial (magnitude of cross product small).
# If UT is parallel to RTIG, the cross product becomes ill-defined.
		PDDL	BHIZ
			36D		# TEST MAGNITUDE OF UT X RTIG
			FIXY		# IF SMALL, USE UT X VTIG AS YSC
# Normal case: YSCREF is successfully computed from UT x RTIG.
# Complete the right-handed coordinate system by computing ZSCREF.
STORY		VLOAD	STADR
		STORE	YSCREF
		VXV	VSL1		# COMPUTE (YSCREF X XCREF), BUT FOR A
			XSCREF		# RIGHT HANDED SYSTEM, NEED (X CROSS Y).
		VCOMP			# ZSCREF = - (YSCREF X XSCREF)
		STORE	ZSCREF		#        = + (XSCREF X YSCREF)

		SET	RVQ
			PFRATFLG
# Special case: If thrust direction is nearly radial (UT parallel to RTIG),
# compute YSCREF using velocity vector instead. This provides a well-defined
# perpendicular direction when position-based cross product fails.
FIXY		VLOAD	VXV		# IN THIS CASE,
			XSCREF		# YSCREF = UNIT(XSCREF X VTIG)
			VTIG
		UNIT	PUSH
		GOTO
			STORY

# ============================================================================
# TRANSITION: From attitude computation to burn guidance
#
# With the preferred spacecraft orientation computed (XSCREF, YSCREF, ZSCREF),
# the next phase involves continuous guidance during the burn. The S40.8
# subroutine updates the velocity-to-be-gained during engine firing, computes
# time-to-cutoff, and provides steering commands to the Digital Autopilot.
# This is the core guidance loop executing throughout the burn.
# ============================================================================

# Page 770
# ============================================================================
# SUBROUTINE: S40.8 - BURN GUIDANCE AND STEERING COMMANDS
# ============================================================================
#
# PURPOSE: Core guidance loop executing throughout engine burn. Updates the
# velocity-to-be-gained (VG) vector, computes time remaining until engine
# cutoff (TGO), and generates continuous steering commands for the Digital
# Autopilot (DAP) to maintain proper thrust direction.
#
# HISTORICAL CONTEXT:
# This subroutine ran continuously during every major engine burn in the
# Apollo 11 mission - from lunar orbit insertion to descent orbit insertion
# to the final descent phase. It ensures the spacecraft maintains the correct
# attitude while the guidance algorithm recomputes the desired thrust direction
# based on current position and velocity errors.
#
# OPERATIONAL MODE:
# Called repeatedly (typically once per second) during thrusting phase.
# Operates in two modes:
#   1) LAMBERT (aimpoint) mode: For precise maneuvers targeting specific orbital
#      states. Uses BDT vector to extrapolate VG for future guidance.
#   2) EXTERNAL DELTA-V mode: For simple velocity change maneuvers without
#      specific targeting constraints.
#
# SUBROUTINE S40.8
# MODIFIED APRIL 3, 1968 BY PETER ADLER, MIT/IL
#
# DESCRIPTION
#	S40.8 UPDATES THE VELOCITY-TO-BE-GAINED VECTOR, VG, (AND FOR LAMBERT TARGETTED BURNS ALSO EXTRAPOLATES VG
#	USING THE BDT VECTOR) COMPUTES THE TIME FOR ISSUING THE ENGINE OFF COMMAND, TGO, AND CALLS THE ROUTINE
#	"FINDCDUW", WHICH GENERATES STEERING COMMANDS FOR THE DAP.
#
# CALLING SEQUENCE
#	L-1	CALL
#	L		S40.8
#	L+1			INTERPRETIVE RETURN
#
# ALARM
#	IF VG . DELVREF IS NEGATIVE (VG AND DELVREF OVER 90 DEGREES APART), BYPASS TGO AND STEERING COMPUTATIONS
#	AND SET ALARM 1407.  RETURN TO CALLER NORMALLY.
#
# INPUT AND INITIALIZATION
#	VGPREV		REFERENCE	2(7) M/CS
#	DELVREF		REFERENCE	2(7) M/CS
#	BDT		REFERENCE	2(7) M/CS
#	TDECAY		TAIL-OFF TIME	2(28) CS
#	XDELVFLG	1 = EXTERNAL DELTA-V; 0 = LAMBERT (AIMPOINT)
#	STEERSW		1 = DO STEERING AND TGO COMPUTATIONS; 0 = VG UPDATE ONLY
#	FIRSTFLG	1 = GONE TO LAMBERT AT LEAST ONCE; 0 = HAVEN'T GONE TO LAMBERT YET.
#
# NOTE:  VGTIG EQUALS VGPREV
#
# OUTPUT
#	STEERSW		SEE INPUT
#	INPULSW		1 = ENGINE OFF IN TGO CENTISECONDS; 0 = CONTINUE BURN
#	TGO		TIME TO CUT-OFF 2(28) CS
# 	SEE FINDCDUW FOR STEERING OUTPUTS.
#
# SUBROUTINE CALLED
#	FINDCDUW
#
# DEBRIS
#	MPACS, PUSHLIST

		COUNT*	$$/S40.8
# Page 771
# ============================================================================
# S40.8 ENTRY POINT - VELOCITY-TO-BE-GAINED UPDATE
# ============================================================================
#
# Check burn mode and compute updated VG vector. The XDELVFLG flag determines
# whether this is a simple delta-V maneuver or a Lambert-targeted aimpoint burn.
S40.8		BOF			# GENERATE VR IF NOT EXTERNAL DELTA-V BURN
			XDELVFLG
			RASTEER1
# EXTERNAL DELTA-V MODE:
# Compute velocity-to-be-gained as the difference between the original
# velocity target (VGPREV, set at ignition) and the current integrated
# velocity change (DELVREF, updated continuously during burn).
		VLOAD	VSU
			VGPREV
			DELVREF
VGAIN*		STORE	VG		# VELOCITY TO BE GAINED SCALED AT (7) M/CS
# Transform VG from reference coordinates to stable member (IMU) coordinates
# for steering computations. The DAP needs thrust direction in IMU frame.
		MXV	VSL1
			REFSMMAT
		STORE	UNFC/2
# Compute magnitude of VG for display on DSKY during burn.
# Crew monitors this value decreasing toward zero as cutoff approaches.
BDTOK		VLOAD	ABVAL
			VG
		STORE	VGDISP
# ============================================================================
# TIME-TO-CUTOFF (TGO) COMPUTATION
# ============================================================================
#
# Compute the time remaining until engine cutoff. This involves checking
# the angle between the desired velocity change (DELVREF) and the remaining
# velocity-to-be-gained (VG). If these vectors are more than 90 degrees apart,
# the burn has overshot its target - an alarm condition.
TGDCALC		SETPD	VLOAD
			0
			VG
		STOVL	VGPREV
			DELVREF
# Check if steering computation is enabled. If STEERSW=0, skip TGO computation
# and return immediately (VG update only mode).
		BOFF	VCOMP
			STEERSW
			QPRET
# Compute the dot product between DELVREF and VG to check alignment.
# If cos(angle) < 0, the vectors point in opposite directions (>90 deg apart).
		UNIT
		DOT	PUSH
			VG
		BPL	DDV
			ALARMIT		# DELV IS MORE THAN 90 DEGREES FROM VG.
			VEX
# Normal case: Compute TGO using exhaust velocity and acceleration factors.
# TGO = (VG magnitude) / (effective acceleration)
		DAD	DMP
			DPHALF
		SR	DDV
			10D
			36D
		DMP	DAD
			-FOURDT
			TDECAY
		STORE	TGO
# Compute absolute time of engine cutoff (TIG = current time + TGO).
# This is displayed to the crew and used for cutoff timing.
		DAD
			PIPTIME
		STODL	TIG
			TGO
# Check if TGO is less than 4 seconds. If so, the burn is too short for
# continuous steering - switch to attitude hold (IMPULSW=1) mode instead.
		DSU	BPL
			FOURSECS	# 400 CS
			FINDCDUW -2
# SHORT BURN MODE (TGO < 4 seconds):
# Set IMPULSW flag to disable steering updates. The DAP maintains fixed
# attitude for the brief remaining burn duration.
		SET	CLRGO
			IMPULSW
			STEERSW
			QPRET

# ============================================================================
# ALARMIT - BURN OVERSHOOT ALARM HANDLER
# ============================================================================
#
# This section handles the error condition where the achieved velocity change
# (DELVREF) has exceeded the desired velocity change (VG) by more than 90
# degrees. This indicates serious guidance error - the burn has overshot or
# is thrusting in the wrong direction.
#
# ALARM 01407: Indicates burn guidance error. Crew must evaluate whether to
# continue burn or abort. Common causes: incorrect state vector, incorrect
# target parameters, or guidance software malfunction.
ALARMIT		EXIT

# Page 772
		TC	ALARM
		OCT	01407
		TC	INTPRET
# Skip TGO computation but still call FINDCDUW to update steering commands
# with whatever VG remains. This allows recovery if error is transient.
		GOTO			# SKIP TGO COMPUTATION BUT CALL FINDCDUW.
			FINDCDUW -2	# FINDCDUW WILL EXIT TO UPDATEVG +3.

# Burn timing constants for TGO computation
-FOURDT		2DEC	-800 B-18	# -4 (200 CS.) B(-18)
FOURSECS	2DEC	400		# 400 CS SCALED AT 2(+28) CS
2VEXHUST	=	VEX

# ============================================================================
# TRANSITION: From Steering and Guidance Updates to Burn Timing Computation
#
# With steering commands computed and attitude control engaged, attention now
# shifts to precise burn duration management. The S40.13 (TIMEBURN) subroutine
# calculates time-to-go (TGO) based on velocity-to-be-gained (VG) and engine
# characteristics. This computation determines whether the burn is long enough
# for continuous steering or requires impulse mode (fixed attitude). For DPS
# burns, throttle profile affects TGO calculation. For APS burns (no throttle
# control), TGO is computed directly from constant thrust acceleration.
# ============================================================================

# Page 773
# NAME:  		S40.13 -- TIMEBURN
#
# FUNCTION		(1) DETERMINE WHETHER A GIVEN COMBINATION OF VELOCITY TO
#			BE GAINED AND ENGINE CHOICE RESULT IN A BURN TIME
#			SUFFICIENT TO ALLOW STEERING AT THE VEHICLE DURING THE
#			BURN
#			(2) THE MAGNITUDE OF THE RESULTING BURN TIME -- IF IT
#			IS SHORT -- AND THE ASSOCIATED TIME OF THE ENGINE OFF
#			SIGNAL
#
# CALLING SEQUENCE	VIA FINDVAC AS A NEW JOB
#
# INPUT			VGTIG -- VELOCITY TO BE GAINED VECTOR (METERS/CS) AT +7
#			WEIGHT/G -- MASS OF VEHICLE IN KGM AT +16
#			F -- APS ENGINE THRUST IN M.NEWTONS AT +7
#				AND ALSO FOR RCS ENGINE
#			MDOT -- RATE OF DECREASE OF VEHICLE MASS DURING ENGINE
#				BURN IN KILOGRAMS/CS AT +3.  THIS SCALING MAY
#				REQUIRE MODIFICATION FOR SATURN BURNS.
#			ENG1FLAG -- SWITCH TO DECIDE WHETHER APS OR DPS ENGINE IS USED
#				=0	DPS
#				=1	APS
#
# OUTPUT		IMPULSW		ZERO FOR STEERING
#					ONE FOR ATTITUDE HOLD
#			NOTHROTL	ZERO FOR THROTTLING
#					ONE TO INHIBIT THROTTLING
#			TGO		TIME TO BURN IN CS
#
#			THE QUANTITY M.NEWTON = 10000 NEWTONS WILL BE USED TO EXPRESS
#			FORCE.

		EBANK=	TGO
		COUNT*	$$/40.13

# ============================================================================
# S40.13 -- TIMEBURN (Burn Duration Computation)
# ============================================================================
#
# PURPOSE:
# Computes time-to-go (TGO) for engine burn based on velocity-to-be-gained
# (VG), engine thrust acceleration, vehicle mass, and propellant flow rate.
# The computation accounts for decreasing vehicle mass as propellant burns,
# using different algorithms for DPS (throttleable) and APS (fixed thrust).
#
# ALGORITHM OVERVIEW:
# 1. Compute magnitude of VG vector (velocity still needed)
# 2. Correct VG for ullage motor acceleration (4 seconds of 2-jet RCS)
# 3. Branch to DPS or APS computation based on engine type
# 4. Use rocket equation with mass flow to compute TGO
# 5. Set IMPULSW flag if burn is too short for steering (< 4 seconds)
#
# The fundamental equation (for constant thrust F, mass flow MDOT):
#   VG = (F/MDOT) * ln(M0 / (M0 - MDOT*TGO))
# Solved iteratively for TGO using Taylor series approximations.
# ============================================================================

S40.13		TC	INTPRET
# Initialize pushdown stack pointer to 00D (empty stack).
# Clear IMPULSW flag - assume steering is enabled unless burn is very short.
		SETPD	CLEAR
			00D
			IMPULSW		# ASSUME NO STEERING UNTIL FOUND OTHERWISE
# Load velocity-to-be-gained vector VGTIG and compute magnitude.
# This represents total velocity change needed from current state to target.
		VLOAD	ABVAL
			VGTIG		# VELOCITY TO BE GAINED AT +7
# Push VG magnitude onto stack at 00D. Correct for ullage acceleration.
# During LM burns, 2-jet RCS ullage motors fire for ~4 seconds before
# main engine ignition to settle propellants. This provides additional
# velocity change that must be subtracted from required VG.
		PDDL	DMP		# 00D = MAG OF VGTIG AT +7
			4SEC(17)	# CORRECT VG FOR 4 SECS OF 2 JET ULLAGE
			FRCS2
# Compute ullage ΔV = (RCS thrust / vehicle mass) * 4 seconds.
# Divide by vehicle mass (WEIGHT/G) and scale result.
		DDV	SL1		# SCALE
			WEIGHT/G
# Subtract ullage ΔV from required VG. Push corrected VG onto stack.
# This is the velocity change that the main engine must provide.
		BDSU	PUSH
# Check engine type: APSFLAG = 0 for DPS, = 1 for APS.
# APS has fixed thrust; DPS can throttle. Branch accordingly.
		BOFF	SET
			APSFLAG
			S40.13D		# FOR DPS ENGINE
# APS ENGINE PATH: Fixed thrust, no throttle control.
# Set NOTHROTL flag to disable any throttle commands.
			NOTHROTL
# For APS: Compute available thrust acceleration = K1VAL / vehicle_mass.
# K1VAL is APS thrust in M.NEWTONS-CS units (scaled force constant).
		DLOAD	DDV		# 00D = MAG OF VGTIG CORRECTED
			K1VAL		# M.NEWTONS-CS AT +24
			WEIGHT/G
# Subtract corrected VG from available acceleration. Check if result is
# negative (VG too large) or if VG is very small (< 1 m/cs, ~100 CS burn).
		BDSU	BMN
# Page 774
			00D
			S40.131		# TGO LESS THAN 100 CS
# NORMAL BURN DURATION (VG ≥ 1 m/cs, TGO ≥ 100 CS):
# Use full rocket equation accounting for mass depletion. Push VG onto
# stack at 02D as TEMP1. Multiply by propellant mass flow rate MDOT.
		PDDL	DMP		# 02D = TEMP1 AT +7
			MDOT

# MDOT REPRESENTS THE RATE OF DECREASE OF VEHICLE MASS DURING ENGINE
# BURN IN KILOGRAMS/CS.  WHEN SATURN IS USED, THE SCALING MAY
# REQUIRE ADJUSTMENT.

# Continue full rocket equation computation for normal-length burns.
# This section computes TGO using iterative solution accounting for
# exponentially decreasing vehicle mass as propellant is consumed.
# 
# The computation uses the relationship:
#   VG = (F/MDOT) * ln(M0 / Mf)
# where M0 = initial mass, Mf = final mass = M0 - MDOT*TGO
# Rearranged: TGO = (M0/MDOT) * (1 - exp(-VG*MDOT/(F*M0)))
# This is approximated using series expansion for computational efficiency.
			3.5SEC		# 350 CS AT +14
# Subtract 350 CS from mass depletion calculation. Push result.
# Load thrust acceleration F and compute F*500CS (thrust impulse over 5 sec).
		BDSU	PDDL
			WEIGHT/G
			F
		DMP	SR2		# SCALE
			5SECS
# Divide thrust impulse by intermediate mass calculation. Push as TEMP2 at 04D.
# This represents characteristic velocity increment over computation interval.
		DDV	PUSH		# 04D = TEMP2
# Subtract TEMP1 (02D) from result. If positive, computed TGO is valid.
# If negative, propellant insufficient for desired VG - branch to DPS logic.
		BDSU	BPL
			02D
			S40.13D
# Valid TGO computed. Load final mass ratio term and apply corrections.
# Multiply by 5 seconds scaling factor and add base offset of 100 CS.
		DLOAD	BDDV
		DMP	DAD
			5SECS
			1SEC2D		# 100 CS AT +14
# Jump to S40.132 to set IMPULSW flag and exit interpretive mode.
		GOTO
			S40.132

# ============================================================================
# S40.131 -- SHORT BURN COMPUTATION (TGO < 100 CS)
# ============================================================================
# For very short burns where VG < 1 m/cs, vehicle mass change is negligible.
# Use simplified linear relationship: TGO ≈ VG / (F/M) = VG * M / F
# This avoids numerical precision issues in logarithmic rocket equation.
S40.131		DLOAD	DMP
			WEIGHT/G
# Shift right 1 bit for scaling. Push intermediate result.
		SR1	PUSH
# Add constant K2VAL (thrust parameter) and divide by K3VAL (acceleration
# parameter) to compute TGO for impulse-mode burn with fixed attitude.
		DAD	DDV
			K2VAL		# M.NEWTON CS AT +24
			K3VAL		# M.NEWTON CS AT +10

# ============================================================================
# S40.132 -- IMPULSE MODE EXIT (TGO < 4 seconds)
# ============================================================================
# Burns shorter than 4 seconds use impulse mode: fixed attitude throughout
# burn, no continuous steering. Set IMPULSW flag and return TGO to caller.
S40.132		SET	EXIT
			IMPULSW
# Store computed TGO from MPAC to TGO variable for use by burn control.
# TGO determines countdown timer and engine cutoff timing.
S40.132*	TC	TPAGREE
		CA	MPAC
		XCH	L
		CA	ZERO
		DXCH	TGO
		TCF	S40.134

# ============================================================================
# S40.13D -- DPS ENGINE BURN TIME COMPUTATION
# ============================================================================
# Descent Propulsion System with throttle capability. Computes TGO accounting
# for variable thrust profile if NOTHROTL is clear (throttle enabled).
S40.13D		DLOAD	DMP		# FOR DPS ENGINE
			00D
			WEIGHT/G
# Push VG*mass product. Check if using APS (should not happen in this path,
# but provides safety check). If APSFLAG set, branch to APS computation.
		PUSH	BON
			APSFLAG
			APSTGO
# DPS with throttle enabled: Divide by throttle curve parameter S40.136.
# Clear NOTHROTL flag to enable throttle commands during burn.
		DDV	CLEAR
			S40.136
			NOTHROTL
# Check for numerical overflow in division. If overflow occurs, TGO
# computation exceeded valid range - branch to alternate computation.
		BOV	PUSH
# Page 775
			S40.130V
# Check if computed TGO exceeds 600 CS (6 seconds). Long burns require
# different handling due to significant mass depletion and steering updates.
S40.127		DSU	BPL
			6SEC		# 600.0 CS AT +14
			S40.138
# TGO ≤ 6 seconds: Add 600 CS offset and use impulse mode (fixed attitude).
		DAD	GOTO
			6SEC
			S40.132
# Normal exit path from burn time computation.
S40.133		EXIT
# Restart protection: Set phase to 3 for restart recovery, then end job.
# TGO is now computed and stored for use by burn control and countdown.
S40.134		TC	PHASCHNG
		OCT	00003
		TC	ENDOFJOB

# ============================================================================
# S40.130V -- OVERFLOW RECOVERY FOR TGO COMPUTATION
# ============================================================================
# Called when TGO computation overflows (VG very large or mass very small).
# Recomputes TGO using alternate scaling to avoid numeric overflow.
S40.130V	DLOAD	SR4		# RECOMPUTED TGO IN TIMER UNITS
		DDV
			S40.136_	# S40.136 SHIFTED LEFT 10
# Store recomputed TGO and exit to restart protection path.
		STORE	TGO
		EXIT
		TCF	S40.134		# REJOIN COMMON CODING FOR RESTART PROTECT

# ============================================================================
# S40.138 -- LONG BURN HANDLING (TGO > 600 CS)
# ============================================================================
# Burns longer than 89 seconds require throttle. If TGO > 89 seconds,
# disable throttle (NOTHROTL) to use 10% fixed thrust for initial phase.
S40.138		DSU	BPL
			89SECS
			STORETGO
# TGO > 89 seconds: Set NOTHROTL flag to disable throttle commands.
# Throttle will be re-enabled later in burn as propellant depletes.
		SET
			NOTHROTL
# Store computed TGO value from MPAC to TGO variable.
STORETGO	DLOAD			# LOAD TGO AT 2(14)
		EXIT
		TCF	S40.132*

# ============================================================================
# APSTGO -- APS ENGINE TGO COMPUTATION
# ============================================================================
# Ascent Propulsion System: Fixed thrust, simpler computation than DPS.
# Divide VG*mass by APS fixed thrust acceleration, scale, and store.
APSTGO		DDV	SL2
			FAPS
		GOTO
			STORETGO +1
# ============================================================================
# TIMEBURN CONSTANTS -- Timing Parameters for Burn Computation
# ============================================================================
# These constants define time intervals used in TGO calculations, scaled
# as double-precision values with specific binary scaling factors.

1SEC2D		2DEC	100.0 B-14	# 100.0 CS (1 second) scaled at +14
					# Base time offset for normal burn calculations

3.5SEC		2DEC	350.0 B-13	# 350 CS (3.5 seconds) scaled at +13
					# Ullage and ignition buildup correction time

5SECS		2DEC	500.0 B-14	# 500.0 CS (5 seconds) scaled at +14
					# Thrust impulse integration interval

6SEC		2DEC	600.0 B-14	# 600.0 CS (6 seconds) scaled at +14
					# Threshold for long burn vs impulse mode

89SECS		2DEC	8900.0 B-14	# 8900 CS (89 seconds) scaled at +14
					# Threshold for throttle enable (DPS long burns)

# FUNCTION		(1) GENERATES REQUIRED VELOCITY AND VELOCITY-TO-BE-GAINED
#			VECTORS FOR USE DURING AIMPOINT MANEUVERS EVERY TWO
#			COMPUTATION CYCLES (4 SECONDS).
#			(2) UPDATES THE B VECTOR WHICH IS USED IN THE FINAL
#			CALCULATION OF EXTRAPOLATING THE VELOCITY-TO-BE-GAINED
#			THROUGH ONE 2-SECOND INTERVAL INTO THE FUTURE.
#
# CALLING SEQUENCE	VIA FINDVAC AS NEW JOB.
#
# INPUT			RN	ACTIVE VEHICLE RADIUS VECTOR IN METERS AT +29
#			VN	ACTIVE VEHICLE VELOCITY VECTOR IN METERS/CS AT +7
# Page 776
#			VPREV	LAST COMPUTED VELOCITY REQUIRED VECTOR IN
#				METERS/CS AT +7.
#			TIG	TIME OF IGNITION IN CS AT +28.
#			DLTARG	COMPUTATION CYCLE INTERVAL = 200 CS AT +28.
#			PIPTIME	TIME OF RN AND VN IN CS AT +28.
#			GDT/2	HALF OF VELOCITY GAINED IN DELTA T TIME DUE TO
#				ACCELERATION OF GRAVITY IN METERS/CS AT +7.
#			DELVREF	CHANGE IN VELOCITY DURING LAST 2 SEC IN
#				METERS/CS AT +7.
#
# OUTPUT		VGPREV	VELOCITY TO BE GAINED VECTOR IN METERS/CS AT +7.
#			VGDISP	MAG OF VGPREV FOR DISPLAY PURPOSES.
#			VRPREV	VELOCTY REQUIRED VECTOR IN METERS/CS AT +7.
#			BDT	B VECTOR IN METERS/CS AT +7.
#
# SUBROUTINES USED	INITVEL

		# ============================================================================
# S40.9 -- LAMBERT VTOGAIN (VELOCITY-TO-BE-GAINED) COMPUTATION
# ============================================================================
# Computes the velocity-to-be-gained (VG) for steering during powered flight
# using Lambert targeting solution. VG = Vrequired - Vcurrent, representing
# the velocity change still needed to achieve the targeted trajectory.
#
# This subroutine solves the two-point boundary value problem: given current
# position/velocity and target position at a future time, compute the required
# velocity change. Used by guidance to generate steering commands during burns.
#
# MISSION CONTEXT:
# During SPS burns (translunar injection, lunar orbit insertion, transearth
# injection), S40.9 continuously recomputes VG as the spacecraft state evolves.
# The guidance law (S40.8) uses VG to command attitude changes that null the
# velocity error by engine cutoff.
#
# INPUTS:
#   RN       Current position vector (inertial coordinates)
#   VN       Current velocity vector (inertial coordinates)
#   RTARG    Target position vector at cutoff time
#   TGO      Time-to-go until cutoff
#   NORMSW   Normal/180-degree transfer mode flag
#
# OUTPUTS:
#   DELVEET3 Velocity-to-be-gained vector VG = Vrequired - Vcurrent
#   VIPRIME  Required velocity at current position for Lambert solution
#
# ALGORITHM:
# 1. Compute geometric parameters (chord vector, transfer angle)
# 2. Solve Lambert's problem for velocity at current position
# 3. Include Earth oblateness correction if in Earth sphere of influence
# 4. Subtract current velocity to obtain VG steering command
# ============================================================================

EBANK=	VGPREV
		COUNT*	$$/S40.9
# Entry point for VG computation. Initialize push-down list and set flags.
S40.9		TC	INTPRET
		SETPD
			00D
# Set AVFLAG indicating LEM is active vehicle (for coordinate frame selection).
# Load initial epsilon value for transfer angle computation.
		SET	DLOAD
			AVFLAG		# SET AVFLAG FOR LEM ACTIVE
			HI6ZEROS
		PDDL
			EPS1
# Select epsilon tolerance based on NORMSW flag: 10° for normal transfer,
# 45° for 180-degree transfer (used when target is nearly opposite).
		BOFF	DAD		# EPSILON4 = 10 OR 45 DEGREES.
			NORMSW
			EPSSMALL
			EPS2
# Push epsilon value and call HAVEGUES to solve Lambert's problem using
# initial guess iteration method.
EPSSMALL	PUSH	CALL
			HAVEGUES
# Normal exit: VG computed successfully. Set restart phase and end job.
# Main program (P40-P47) will read DELVEET3 for next steering cycle.
ENDS40.9	EXIT
		TC	PHASCHNG
		OCT	2
		TCF	ENDOFJOB

# ============================================================================
# RASTEER1 -- LAMBERT GEOMETRIC PARAMETER COMPUTATION
# ============================================================================
# Computes geometric parameters for Lambert's problem solution:
#   - Chord vector C = RTARG - RN (connecting current and target positions)
#   - Semi-parameter SS = (R1 + R2 + C)/2 (Lambert geometry parameter)
#   - Scaling factors for numeric stability in subsequent computations
#
# Lambert's problem: Given two position vectors R1, R2 and transfer time,
# find the connecting orbit. Geometry involves transfer angle, chord length,
# and semi-perimeter of the triangle formed by R1, R2, and the focus.
# ============================================================================

# Load current position vector and compute magnitude R1 = |RN|.
RASTEER1	VLOAD	ABVAL
			RN
# Load scaling index RTX2 for numeric precision control. Shift left by
# index amount to scale R1 magnitude appropriately.
		LXC,2	SL*
			RTX2
			0,2
# Store scaled R1 magnitude. Load target position vector RTARG.
		STOVL	RMAG
			RTARG
# Compute chord vector: C = RTARG - RN. Normalize with unit normalization
# routine that also returns magnitude in 36D.
		VSU	RTB
			RN
			NORMUNX1
# Store unit chord vector IC (intermediate chord). Load chord magnitude
# from 36D (result of NORMUNX1 operation).
		STODL	IC
			36D		# C(36D) = ABVAL(C)
# Add scaling factors and shift left for precision. This adjusts the chord
# magnitude C to the same scaling as position magnitudes R1, R2.
		XAD,2	SL*
			X1
# Page 777
			0,2
# Store adjusted chord magnitude at temporary location 30D.
		STORE	30D
# Normalize chord magnitude (finding exponent X2) and multiply by R1 magnitude.
# This forms the product R1*C needed for semi-parameter computation.
		NORM	DMP
			X2
			RMAG
# Accumulate scaling exponents: X1 (from normalization) combined with index
# from X2. Store combined scaling factor for later use.
		NORM	XAD,2
			X1
			X1
		SXA,2
			MUSCALE
# Store scaled R1*C product. Load chord magnitude from temporary storage.
		STODL	R1C		#			2(+56 -X)
			30D
# Shift right 1 bit (divide by 2) for averaging computation.
		SR1	PDDL
# Load R1 magnitude, shift right 1 (divide by 2).
			RMAG
		SR1	PDDL
# Load R2 magnitude (target radius), shift right 1 (divide by 2).
			RTMAG
		SR1	DAD
# Add all three half-values: C/2 + R1/2 + R2/2. Store address for further use.
		DAD	STADR
# Store result as SS = (R1 + R2 + C)/2, the semi-perimeter of Lambert triangle.
# SS is fundamental parameter for solving Lambert's equation.
		STORE	SS		# SS = (R1 + R2 + C)/2
# Compute (SS - C)*MU/A - MUASTEER. This forms part of Lambert geometry
# calculation relating semi-parameter to gravitational parameter.
		DSU	DMP
			30D
			MU/A
		BDSU
			MUASTEER
# Push result. Load SS and compute (SS - R1), the difference between
# semi-parameter and current radius.
		PDDL	DSU
			SS
			RMAG
# Normalize the difference, shift right 1 bit, then divide by R1*C product.
# Multiply result to form geometric ratio for Lambert solution.
		NORM	SR1
			X1
		DDV	DMP
			R1C
# Apply scaling correction from accumulated exponents. Shift left using
# combined scaling factor from MUSCALE.
		XSU,2	SL*
			X1
			1,2
# Load MUSCALE back into index register for subsequent operations.
		LXA,2
			MUSCALE
# Take square root and apply sign from GEOMSGN (geometric sign based on
# transfer angle). Result is the 'A' parameter in Lambert formulation.
		SQRT	SIGN
			GEOMSGN
# Store +/- A parameter at temporary location 32D for velocity calculation.
		STORE	32D		# + OR - A
# ============================================================================
# MUASTEER -- LAMBERT B PARAMETER AND TRANSFER TIME COMPUTATION
# ============================================================================
# Compute the 'B' parameter for Lambert solution and verify transfer time.
# Lambert's equation relates transfer time to geometry parameters A and B.
# ============================================================================

# Load SS, multiply by MU/A ratio, subtract MUASTEER constant.
		DLOAD	DMP
			SS
			MU/A
		BDSU
			MUASTEER
# Push result. Load SS and compute (SS - R2), difference between semi-parameter
# and target radius magnitude.
		PDDL	DSU
			SS
			RTMAG
# Normalize the difference, shift right 1, divide by R1*C, and multiply.
# This forms geometric ratio for B parameter computation.
		NORM	SR1
			X1
		DDV	DMP

# Page 778
			R1C
# Apply scaling corrections and shift left. Take square root for B parameter
# (unsigned initially).
		XSU,2	SL*
			X1
			1,2
# Take square root and push. This is -B (without sign yet).
		SQRT	PDDL		# -B (NO SIGN)
# Load SS and compute (SS - C)/SS ratio. This forms part of transfer angle
# calculation in Lambert geometry.
			SS
		DSU	DDV
			30D
			SS
# Take square root, push result. Shift right 1 and compute arcsine to get
# transfer angle component.
		SQRT	PUSH
		SR1	ASIN
# Multiply by 2π+3 constant, push result. Reload (C/SS) ratio and divide
# by SS for normalized transfer geometry.
		DMP	PDDL
			2PI+3
		PDDL	DDV
			30D
			SS
# Branch on overflow to next instruction. Take square root and multiply for
# transfer angle contribution.
		BOV
			+1
		SQRT	DMP
# Shift right 3 bits and subtract (BDSU). Apply geometric sign from GEOMSGN.
		SR3	BDSU
		SIGN	PDDL
			GEOMSGN
# Push 2π+3 constant, shift right 2, and subtract to form angle difference.
			2PI+3
		SR2	DSU
# Multiply by SS, push result. Load SS again, shift right 3, take square root.
		DMP	PDDL
			SS
			SS
		SR3	SQRT
# Multiply accumulated values together. Push result and shift left 3 bits.
		DMP
		PDDL	SL3
# Load MUASTEER, take square root, and divide (BDDV). This gives computed
# transfer time for the Lambert solution.
			MUASTEER
		SQRT	BDDV
# Subtract TPASS4 (desired transfer time) and add current time PIPTIME.
# Result is time difference between computed and desired transfer.
		DSU	DAD
			TPASS4
			PIPTIME
# Store time comparison result. Load B parameter and apply sign based on
# geometric configuration (sign matches A parameter sign).
		STODL	30D
		SIGN
			30D		# B WITH SIGN
# ============================================================================
# VELOCITY VECTOR COMPUTATION -- TWO GEOMETRIC CASES
# ============================================================================
# Compute required velocity vector VIPRIME using Lambert B parameter.
# Two cases: normal transfer (less than 180°) and long-way transfer (180° case).
# ============================================================================

# Store B parameter with sign at 30D for velocity computations. Branch on
# NORMSW flag: if set, transfer angle is near 180° requiring special handling.
		STORE	30D
		BON	VLOAD
			NORMSW
			180MESS
# Normal case (transfer angle < 180°): Load unit chord vector IC, subtract
# unit radial vector UNIT/R/, and compute unit vector of the difference.
			IC
		VSU	UNIT
			UNIT/R/
# Multiply unit difference by B parameter (30D), push result. Load chord IC
# and add unit radial to form sum direction.
		VXSC	PDVL
			30D
			IC
		VAD	UNIT
			UNIT/R/
# Page 779
# Entry point for normal case velocity computation. Multiply unit sum by A
# parameter (32D) and add B*unit_diff to get velocity vector.
GETVRVG1	VXSC	VAD
			32D
# Common exit path for both geometric cases. Load RTX2 scaling index and apply
# right shift to convert from computational units to physical velocity units.
GETVRVG2	LXC,2	VSR*
			RTX2
			0 	-1,2
# Store computed velocity vector as VIPRIME (velocity at current position for
# Lambert solution). Jump to ASTREND-2 to compute velocity-to-be-gained.
		STORE	VIPRIME
		GOTO
			ASTREND -2
# ============================================================================
# 180MESS -- 180-DEGREE TRANSFER ANGLE SPECIAL CASE
# ============================================================================
# When transfer angle is near 180°, standard Lambert formulation becomes
# numerically unstable. Use alternative geometric construction to compute
# velocity vectors for this long-way transfer case.
# ============================================================================

# Load unit chord IC and dot with unit radial UNIT/R/ to check geometric
# configuration. Branch if dot product is negative (different handling).
180MESS		VLOAD	DOT
			IC
			UNIT/R/
		BMN	VLOAD
			NEGPROD
# Positive dot product case: Load IC, shift right 1 (divide by 2), push.
# Load unit radial, shift right 1, add to IC/2 and compute unit vector.
			IC
		VSR1	PDVL
			UNIT/R/
		VSR1	VAD
		UNIT
# Push unit sum vector. Complement (VCOMP) for A parameter direction.
# Cross product with UN vector, apply geometric sign from GEOMSGN.
		PUSH	VCOMP		# FOR A
		VXV	SIGN
			UN
			GEOMSGN
# Compute unit vector of cross product result, multiply by B parameter (30D),
# push result. This is B direction vector: UNIT(IC-IR) ±B.
		UNIT	VXSC
			30D
		PDVL			# UNIT(IC-IR)	+-B
# Jump to normal velocity computation path (GETVRVG1) with constructed vectors.
		GOTO
			GETVRVG1
# Negative dot product case: Alternative geometric construction for opposite
# radial/chord alignment. Load unit radial, shift right 1, push. Load IC,
# shift right 1, subtract radial/2 and compute unit of difference.
NEGPROD		VLOAD	VSR1
			UNIT/R/
		PDVL	VSR1
			IC
		VSU	UNIT
# Push unit difference vector. Cross product with UN vector for B direction,
# apply geometric sign.
		PUSH
		VXV	SIGN
			UN		# FOR B
			GEOMSGN
# Compute unit cross product, multiply by A parameter (32D), push. Load next
# vector, multiply by B parameter (30D), add to A term.
		UNIT	VXSC
			32D
		PDVL
		VXSC	VAD
			30D
# Jump to common scaling path (GETVRVG2) to complete velocity computation.
		GOTO
			GETVRVG2

# ============================================================================
# ASTREND -- COMPUTE VELOCITY-TO-BE-GAINED AND CHECK OBLATENESS
# ============================================================================
# Subtract current velocity VN1 from required velocity VIPRIME to get
# steering command DELVEET3. Check if oblateness correction is needed.
# ============================================================================

# Subtract current velocity from required velocity. Result is velocity-to-be-
# gained (VG) vector for steering guidance.
		VSU
			VN1
# Store VG as DELVEET3 (steering command output). Load RTX2 scale factor and
# branch if zero (Earth sphere -- apply oblateness correction).
ASTREND		STORE	DELVEET3
FIRSTTME	SLOAD	BZE
			RTX2
			GETGOBL
# RTX2 nonzero indicates Moon sphere. No oblateness correction needed.
# Load DELVEET3 and continue to completion.
		VLOAD	GOTO		# NO OBLATENESS COMP IF IN MOON SPHERE
# Page 780
			DELVEET3
			NOGOBL

# ============================================================================
# GETGOBL -- EARTH OBLATENESS CORRECTION
# ============================================================================
# For Earth-centered orbits, compute gravitational oblateness correction term
# to account for Earth's non-spherical gravity field (J2 effect).
# ============================================================================

# Load current position RN, compute unit radial direction. This is UNITGOBL
# direction for oblateness perturbation.
GETGOBL		VLOAD	UNIT		# CALCULATE OBLATENESS TERM.
			RN
# Compute time interval (T-TIG): Load current PIPTIME, subtract burn ignition
# time GOBLTIME. Oblateness acceleration magnitude formula: -(MU/R²)(T-TIG).
		DLOAD	DSU
			PIPTIME		#              2
			GOBLTIME	# G    = -(MU/R )(UNITGOBL)(T-TIG)
# Multiply time interval by Earth gravitational parameter EARTHMU. Divide by
# R² (34D contains /RN/(2) from UNIT operation above).
		DMP	DDV		#  OBL
			EARTHMU
			34D		# 34D = /RN/ (2) FROM UNIT OPERATION.
# Scale UNITGOBL by oblateness magnitude. Add correction to DELVEET3.
# Final velocity-to-be-gained: VG = VR + GOBL - VN.
		VXSC	VAD
			UNITGOBL
			DELVEET3	# OUTPUT FROM INITVEL VG = VR - VN
# Store corrected VG as DELVEET3 (steering output). Jump to VGAIN* for attitude
# steering computation.
NOGOBL		STORE	DELVEET3	# VG = VR + GOBL - VN
		GOTO
			VGAIN*

# ============================================================================
# S40.9 COMPLETION -- CONSTANTS
# ============================================================================

# Mathematical constant 2π + 3 scaled B-2 (scaled by 2^-2 = 0.25).
# Used in Lambert solution geometry calculations.
2PI+3		2DEC	3.141592653 B-2

# Page 781
# ============================================================================
# TRIMGIMB (FORMERLY S40.6) -- DPS ENGINE GIMBAL TRIM CALIBRATION
# ============================================================================
# MOD 0		24 FEB 67	PETER ADLER
#
# PURPOSE:
# Calibrate Descent Propulsion System (DPS) engine gimbal actuators to minimize
# thrust vector misalignment from center of gravity. This procedure establishes
# mechanical zero reference by driving gimbals to hard stops, then backs off to
# computed trim position based on vehicle mass distribution.
#
# HISTORICAL CONTEXT:
# DPS engine could gimbal ±6 degrees in pitch and roll to vector thrust through
# the shifting CG as propellant was consumed. Accurate gimbal trim was critical
# for efficient attitude control during lunar descent and ascent abort scenarios.
#
# PROCEDURE:
# 1. Drive engine to full +PITCH and +ROLL (gimbal locks at mechanical stops)
# 2. Wait 1 minute to ensure actuators reach stops (hard reference position)
# 3. Drive engine in -PITCH and -ROLL for computed calibration times
# 4. Final gimbal position minimizes thrust/CG offset for current vehicle mass
#
# CALLING SEQUENCE:
#	VIA WAITLIST FROM R03 (pre-ignition calibration routine)
#
# INPUT:
#	PITTIME		Time to run from full +PITCH to trim position (centiseconds)
#	ROLLTIME	Time to run from full +ROLL to trim position (centiseconds)
#
# SUBROUTINES USED:
#	WAITLIST, FIXDELAY, VARDELAY, FLAGUP, FLAGDOWN, NOVAC
#
# TECHNICAL NOTES:
# Gimbal actuator rates specified by Grumman Aerospace (GAEC) based on DPS
# actuator performance. Timing accuracy critical for correct trim position.
# ============================================================================

		COUNT*	$$/S40.6
		EBANK=	ROLLTIME	# OCTAL MASKS: PRIO5=05000 EBANK5=02400

# ============================================================================
# TRIMGIMB ENTRY -- INITIALIZE GIMBAL CALIBRATION SEQUENCE
# ============================================================================

# Clear GMBDRVSW flag. This flag will be set when first axis (pitch or roll)
# completes its trim motion, indicating one calibration axis is finished.
TRIMGIMB	TC	DOWNFLAG	# GMBDRVSW FLAG IS SET WHEN EITHER ROLL OR
		ADRES	GMBDRVSW	# PITCH IS COMPLETED, WHICHEVER IS FIRST.

# ============================================================================
# PHASE 1: DRIVE TO MECHANICAL STOPS (REFERENCE POSITION)
# ============================================================================

# Turn off any active -PITCH or -ROLL commands by clearing bits in Channel 12.
# Complement PRIO5 mask and write-AND to channel.
		CS	PRIO5		# TURN OFF - PITCH, - ROLL, IF ON.
		EXTEND
		WAND	CHAN12
# Turn on +PITCH and +ROLL commands by setting bits in Channel 12. This drives
# engine gimbals to positive mechanical stops (hard reference for calibration).
		CAF	EBANK5		# TURN ON + PITCH, + ROLL.
		EXTEND
		WOR	CHAN12
# Wait 1 minute (6000 centiseconds) to ensure gimbal actuators reach mechanical
# stops. Actuator slew rate requires this dwell time for full travel.
		TC	FIXDELAY	# WAIT ONE MINUTE TO MAKE SURE ENGINE IS
		DEC	6000		# AT FULL + PITCH AND FULL + ROLL

# ============================================================================
# PHASE 2: DRIVE TO TRIM POSITION (CALIBRATED OFFSET)
# ============================================================================

# Turn off +PITCH and +ROLL commands now that stops are reached.
		CS	EBANK5		# TURN OFF + PITCH, + ROLL.
		EXTEND
		WAND	CHAN12
# Turn on -PITCH and -ROLL commands. Engine will now drive back toward neutral,
# stopping after computed time intervals to achieve trim position.
		CAF	PRIO5		# TURN ON - PITCH, - ROLL.
		EXTEND
		WOR	CHAN12
# Load pitch trim time (PITTIME) and schedule PITCHOFF task via TWIDDLE to stop
# pitch axis motion when calibration time expires.
		CAE	PITTIME		# GET TIME TO SHUT OFF - PITCH AND SET UP
		TC	TWIDDLE		# TWIDDLE-TASK TO TURN IT OFF THEN
		ADRES	PITCHOFF

# Load roll trim time (ROLLTIME) and wait for that duration (VARDELAY). When
# roll time expires, control returns here to shut off roll axis.
		CAE	ROLLTIME	# GET TIME TO SHUT OFF - ROLL AND GO AWAY
		TC	VARDELAY	# UNTIL THEN
# Roll trim time expired. Turn off -ROLL command by clearing bit 12 in Chan 12.
# Roll axis is now at trim position.
		CS	BIT12
		EXTEND
		WAND	CHAN12		# SHUT OFF ROLL

# ============================================================================
# ROLLOVER -- CHECK IF BOTH AXES COMPLETED CALIBRATION
# ============================================================================

# Check GMBDRVSW flag to see if other axis (pitch) already completed. If here
# inline, roll just finished. If here from PITCHOFF, pitch just finished.
ROLLOVER	CA	FLAGWRD6	# IF HERE INLINE (ROLL DONE) IS PITCH DONE
		MASK	GMBDRBIT	# IF HERE FROM PITCHOFF, IS ROLL DONE?
		EXTEND
# Branch if flag not set (other axis still running). Will set flag and exit.
		BZF	PITCHOFF +4	# NO.  SET FLAG, ROLL OR PITCH DONE.
# Both axes completed. Gimbal trim calibration finished. Schedule NOVAC job at
# priority 10 to return control to R03 (calling routine).
		CAF	PRIO10		# RETURN TO R03.
		TC	NOVAC
		EBANK=	WHOCARES
# Page 782
		2CADR	TRIMDONE

		TC	TASKOVER

# ============================================================================
# PITCHOFF -- PITCH AXIS TRIM COMPLETION HANDLER
# ============================================================================

# This task is scheduled by TWIDDLE when pitch trim time (PITTIME) expires.
# It shuts off pitch axis motion and checks if roll is also complete.
PITCHOFF	CS	BIT10
		EXTEND
		WAND	CHAN12		# SHUT OFF PITCH
# Branch to ROLLOVER to check if roll axis also completed its trim motion.
		TCF	ROLLOVER	# SEE IF ROLL HAS FINISHED ALSO.
# If here (not branched), roll is done but pitch just finished. Set GMBDRVSW
# flag to indicate first axis completed, then exit via TASKOVER.
		TC	UPFLAG		# ROLL DONE; OR PITCH DONE; BUT NOT BOTH.
		ADRES	GMBDRVSW
		TC	TASKOVER

# Page 783
# ============================================================================
# S41.1 -- REFERENCE-TO-BODY COORDINATE TRANSFORMATION FOR VELOCITY VECTORS
# ============================================================================
# SUBROUTINE NAME:  S41.1	MOD. NO. 0	DATE: FEBRUARY 28, 1967
# MOD. NO. 1	DATE: JANUARY 23, 1968, BY PETER ADLER (MIT/IL)
#
# AUTHOR: JONATHON D. ADDLESTON (ADAMS ASSOCIATES)
#
# PURPOSE:
# Transforms velocity vectors from the inertial reference frame (REF) to the
# Lunar Module body coordinate system (LM body axes). This transformation is
# essential for displaying velocity information to the crew in body-relative
# coordinates and for computing thrust commands in the LM reference frame.
#
# OPERATIONAL CONTEXT:
# During powered flight, the guidance computer must transform computed velocity
# vectors (from orbital mechanics calculations in inertial space) into the LM's
# body-fixed coordinate system. This allows the crew to understand velocity in
# terms of forward/back, left/right, up/down relative to the spacecraft, and
# enables the autopilot to command proper thrust vectoring.
#
# TRANSFORMATION CHAIN:
# 1. Reference Frame → Stable Member (via REFSMMAT matrix multiplication)
# 2. Stable Member → Body Coordinates (via *SMNB* using current CDU angles)
#
# The transformation accounts for:
# - IMU platform orientation (REFSMMAT defines reference-to-stable-member)
# - Spacecraft attitude (CDU angles define stable-member-to-body)
# - Gimbal angles from Inertial Measurement Unit (IMU)
#
# MATHEMATICAL FORMULATION:
# V(LM) = *SMNB* × REFSMMAT × V(REF)
# where V(LM) is velocity in LM body axes, V(REF) is velocity in reference frame
#
# SPECIFICALLY, IT IS USED TO TRANSFORM A VELOCITY (SCALED AT 2(+7) METERS/CENTISECOND) FROM REFERENCE TO LM AXIS
# COORDINATES.  FIRST THE VECTOR IS TRANSFORMED TO THE STABLE MEMBER COORDINATES BY THE MATRIX REFSMMAT.  THIS
# LEAVES THE VECTOR IN MPAC, SCALED AT 2(+8) METERS/CENTISECOND.  THEN
# THE SUBROUTINE CDUTRIG IS CALLED TO SET UP THE DOUBLE-PRECISION CDU VECTOR ALONG WITH ITS SINES AND COSINES.
# THE VECTOR IS THEN TRANSFORMED FROM STABLE MEMBER COORDINATES TO SPACECRAFT (OR LM) COORDINATES BY THE
# SUBROUTINE *SMNB*.  FINALLY, THE VECTOR IS RESCALED TO 2(+7) METERS/CENTISECOND, AND CONTROL IS RETURNED BO THE
# CALLER WITH C(MPAC) = VELOCITY(LM).
#
# CALLING SEQUENCE (INTERPRETIVE):
#	L	VLOAD	CALL
#	L +1		VELOCITY(REF)		# SCALED AT 2(+7) M/CS IN REFERENCE COORDS.
#	L +2		S41.1
#	L +3	STORE	VELOCITY(LM)		# SCALED AT 2(+7) M/CS IN LM BODY AXIS SYS.
#
# SUBROUTINES CALLED:
#	1.	CDUTRIG,
#			WHICH CALLS CDULOGIC.
#	2.	*SMNB*  (Stable Member to Navigation Base transformation)
#
# NORMAL RETURN:  L +3 (SEE CALLING SEQUENCE, ABOVE.)
#
# ALARM/ABORT MODES:  NONE.
#
# RESTART PROTECTION:  NONE.
#
# Page 784
# INPUT:
#	1.	REFSMMAT:	Reference to Stable Member transformation matrix
#	2.	CDUX, CDUY, CDUZ:  Current gimbal angles from IMU
#	3.	VELOCITY (REF) IN MPAC:  Velocity vector in reference frame
#			scaled at 2(+7) meters/centisecond
#
# OUTPUT:
#	1.	CSUSPOT:	DOUBLE PRECISION CDU VECTOR, ORDERED Y,Z,X.
#	2.	SINCDU:		HALF SINES OF CDUSPOT COMPONENTS
#	3.	COSCDU:		HALF COSINES OF CDUSPOT COMPONENTS.
#	4.	MPAC:		VELOCITY(LM) (SCALED AT 2(+7) METERS/CENTISECOND)
#			Velocity vector in LM body-fixed coordinates
#
# SCALING NOTES:
# Input:  2(+7) meters/centisecond = 0.0078125 m/cs resolution
# Intermediate: 2(+8) meters/centisecond after REFSMMAT (half-unit matrix)
# Output: 2(+7) meters/centisecond after rescaling
#
# DEBRIS:  NONE.
#
# CHECKOUT STATUS:  CODED
# ============================================================================

		COUNT*	$$/S41.1

# ============================================================================
# S41.1 ENTRY -- TWO-STEP COORDINATE TRANSFORMATION
# ============================================================================

# Step 1: Transform from reference frame to stable member coordinates.
# MXV multiplies MPAC vector by REFSMMAT matrix. VSL1 shifts left 1 bit to
# rescale from 2(+7) to 2(+8) m/cs, accounting for REFSMMAT half-unit scaling.
S41.1		MXV	VSL1		# CONVERT VECTOR IN MPAC FROM REF AT 2(+7)
			REFSMMAT	# TO SM AND RESCALE DUE TO HALF-UNIT MATRIX

# Step 2: Transform from stable member to body coordinates using current CDU
# gimbal angles. CDU*SMNB calls CDUTRIG to compute sines/cosines of gimbal
# angles, then calls *SMNB* to perform the rotation. Returns via RVQ to the
# original caller of S41.1 with transformed vector in MPAC at 2(+7) m/cs.
		GOTO			# CONVERT TO BODY AT 2(+7) USING PRESENT
			CDU*SMNB	# CDU ANGLES.  CDU*SMNB WILL RETURN
					# VIA RVQ TO THE CALLER OF S41.1.
