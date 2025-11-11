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
;
# Page 752
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

;
; ============================================================================
; PROGRAM: P40LM - SERVICE PROPULSION SYSTEM (SPS) BURN EXECUTION
;
; This program manages large velocity change (ΔV) maneuvers using the Service
; Propulsion System engine. P40 controls the complete burn sequence from
; ignition preparation through thrust termination, coordinating with guidance
; computations, navigation state updates, and the Digital Autopilot (DAP).
;
; BURN SEQUENCE:
;   1. IMU status check and attitude maneuver to thrust orientation
;   2. Ullage motor firing to settle propellants
;   3. Engine ignition via BURN_BABY_BURN master ignition routine
;   4. Thrust monitoring with velocity-to-be-gained (VG) tracking
;   5. Guidance updates during burn (steering law, trim gimbal control)
;   6. Velocity cutoff when VG approaches zero
;   7. Post-burn: maintain VG calculations, reset DAP parameters
;
; MISSION CONTEXT: While the Lunar Module does not use SPS (that's the Command
; Module's main engine), this code demonstrates the AGC's burn control logic
; applicable to all propulsion systems. The LM primarily uses DPS (Descent
; Propulsion System) and APS (Ascent Propulsion System) controlled by P42.
;
; HISTORICAL NOTE: During Apollo 11, similar burn control logic governed the
; critical descent engine throttling that Neil Armstrong monitored during the
; final approach to the lunar surface.
; ============================================================================
;
P40LM		TC	PHASCHNG
		OCT	04024

		CAF	P40ADRES	# INITIALIZATION FOR BURNBABY
; The crew has selected Program 40 via DSKY verb V37. The guidance computer
; now prepares for a Service Propulsion System burn by verifying spacecraft
; configuration and computing the optimal attitude for the thrusting maneuver.
;
		TS	WHICH

		CA	FLGWRD10
		MASK	APSFLBIT
		CCS	A
		TCF	P40ALM
		TC	BANKCALL	# GO DO IMU STATUS CHECK ROUTINE.
; Check if the descent stage has been staged (STAGEFLG bit). If staged,
; this is an error condition for P40 which requires the descent stage attached.
;
		CADR	R02BOTH

		CS	DAPBOOLS	# INITIALIZE DVMON
		MASK	CSMDOCKD
		CCS	A
		CAF	THRESH1
		AD	THRESH3
		TS	DVTHRUSH
		CAF	FOUR
		TS	DVCNTR
; IMU status verification via R02BOTH ensures the Inertial Measurement Unit
; is properly aligned and operational before beginning attitude maneuvers.
;
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

;
; R02BOTH performs a comprehensive IMU (Inertial Measurement Unit) status check,
; verifying the gyroscopes and accelerometers are operating within acceptable
; parameters. If the IMU has drifted beyond allowable limits, the crew must
; perform a platform realignment before proceeding with the burn. During Apollo 11,
; IMU alignment was critical - any drift could result in pointing errors that
; would waste propellant or miss the target orbit.
;
		INHINT
		TC	IBNKCALL
		CADR	PFLITEDB	# ZERO ATTITUDE ERRORS, SET DB TO ONE DEG.

		TC	P40SXT4

#	********************************

		TCF	BURNBABY

#	********************************

P40SXT4		EXTEND
		QXCH	P40/RET
P41MANU		RELINT
; After thrust termination, the guidance computer performs post-burn cleanup:
; - Zero the rendezvous counter to reset timing references
; - Maintain velocity-to-be-gained (VG) calculations for possible manual RCS
; - Set maximum deadband in Digital Autopilot for propellant conservation
; - Reset steering law parameter CSTEER to zero
;

		TC	DOWNFLAG	# CLEAR 3AXISFLG -- R60 USE VECPOINT.
		ADRES	3AXISFLG

		TC	BANKCALL
		CADR	R60LEM		# DO ATTITUDE MANEUVER ROUTINE
		TC	P40/RET

		EBANK=	TRKMKCNT
POSTBURN	CA	Z
		TS	DISPDEX
		EXTEND
		DCA	ACADN85
		DXCH	AVEGEXIT
		CAF	V16N40
		TC	BANKCALL
		CADR	GOFLASHR
		TC	TERM40
		TCF	TIGNOW
		TC	POSTBURN
# Page 755
P40PHS1		TC	PHASCHNG
		OCT	00014
		TCF	ENDOFJOB

TIGNOW		INHINT
		TC	IBNKCALL
		CADR	ZATTEROR
		TC	IBNKCALL
		CADR	SETMINDB
		RELINT
		CAF	V16N85B
		TC	BANKCALL
		CADR	REFLASHR
		TC	TERM40
		TCF	TERM40
		TC	-5

		TCF	P40PHS1

TERM40		EXTEND
		DCA	SERVCADR
		DXCH	AVEGEXIT
		CAF	ZERO
		TS	TRKMKCNT	# ZERO RENDZVS CNTERS
		CA	Z
		TS	DISPDEX
		INHINT
		TC	IBNKCALL
		CADR	RESTORDB
		RELINT
		TC	GOTOPOOH

		EBANK=	WHICH
		COUNT*	$$/P41
;
; ============================================================================
; PROGRAM: P41LM - REACTION CONTROL SYSTEM (RCS) BURN EXECUTION
;
; This program manages small velocity change maneuvers using the Reaction
; Program 41 handles smaller velocity changes using Reaction Control System
; thrusters. The RCS provides fine-tuning capability with either 4-jet or
; 2-jet configurations, trading thrust level against propellant consumption.
;
; Control System (RCS) thrusters. Unlike P40's large SPS burns, P41 handles
; fine-tuning maneuvers and displays parameters for manual crew control.
;
; RCS BURN FEATURES:
;   - Uses 4-jet or 2-jet RCS configuration (controlled by NJETSFLG)
;   - Computes thrust direction via S40.1 subroutine
;   - Calculates preferred IMU orientation via S40.2,3
;   - Transforms velocity-to-be-gained from reference to body coordinates
;   - Displays VG in body axes (V16N85) for manual RCS maneuvering
;   - Sets minimum deadband (0.3 degrees) for attitude control precision
; Transform velocity-to-be-gained from reference (inertial) coordinates to
; LM body axes via S41.1. This allows the crew to understand the required
; velocity change in terms of forward/back, left/right, up/down motions.
;
;
; The crew can monitor the velocity components required in pitch, yaw, and
; roll axes, enabling manual thruster firing to achieve the computed ΔV.
; This provides backup capability if automatic guidance is unavailable.
;
; RCS THRUST LEVELS:
;   FRCS4 = Four-jet thrust force (higher thrust, faster maneuver)
;   FRCS2 = Two-jet thrust force (lower thrust, fuel conservation)
; ============================================================================
;
P41LM		CAF	P41ADRES	# INITIALIZATION FOR BURNBABY
		TS	WHICH

		TC	BANKCALL
		CADR	R02BOTH

		TC	INTPRET		# BOTH LM
		BON	DLOAD		# IF NJETSFLAG IS SET, LOAD Z JET F
			NJETSFLG
			P41FJET1
			FRCS4		# IF NJETSFLG IS CLEAR, LOAD 4 JET F

P41FJET		STCALL	F
			P41IN
P41FJET1	DLOAD
# Page 756
			FRCS2
		STORE	F

P41IN		CALL
			S40.1		# BOTH
P41NORM		CALL
			S40.2,3		# CALCULATE PREFERRED IMU ORIENTATION AND
		EXIT			# SET PFRATFLG.

		INHINT
		TC	IBNKCALL
		CADR	ZATTEROR	# ZERO ATTITUDE ERRORS
		TC	IBNKCALL
		CADR	SETMINDB	# SET 0.3 DEGREE DEADBAND
		TC	P40SXT4

		TC	INTPRET
		VLOAD	CALL		# TRANSFORM VELOCITY-TO-BE-GAINED AT TIG
			VGTIG		# FROM REFERENCE COORDINATES TO LM BODY-
			S41.1		# AXIS COORDINATES FOR V16N85 DISPLAY.
		STORE	VGBODY		# (SCALED AT 2 (+7) METERS/CENTISECOND)
		EXIT

		CAF	V16N85B
		TC	BANKCALL
		CADR	GODSPRET

		CAF	PRIO5
		TS	DISPDEX		# FOR SAFETY ONLY
		TC	FINDVAC
		EBANK=	VGPREV
		2CADR	DYNMDISP

		TC	2PHSCHNG
		OCT	00076		# GROUP 6 RESTARTS AT REDO6.7
		OCT	04024		# GROUP 4 RESTARTS HERE

#	********************************

		TCF	B*RNB*B*

#	********************************

BLNKWAIT	CAF	1SEC
		TC	BANKCALL
		CADR	DELAYJOB

REDO6.7		CA	DISPDEX		# ON A RESTART, DO NOT PUT UP DISPLAY IF
		AD	TWO		# BLANKING (BETWEEN TIG-35 AND TIG-30)

#	********************************
# Page 757

		EXTEND
		BZF	BLNKWAIT

		CAF	V16N85B
		TC	BANKCALL
		CADR	GODSPRET

		CAF	PRIO5
		TC	PRIOCHNG

DYNMDISP	CA	DISPDEX		# A NON-POSITIVE DISPDEX INDICATES PAST
		EXTEND			# TIG-35, SO SERVICER WILL BE DOING THE
		BZMF	ENDOFJOB	# UPDATING OF NOUN 85.  STOP DYNMDISP.
		TC	INTPRET
		VLOAD	CALL
			VGPREV
			S41.1
		STORE	VGBODY
		EXIT
		CAF	1SEC
		TC	BANKCALL
; Program 42 controls the Ascent Propulsion System for lunar liftoff. During
; Initialize delta-V monitor (DVMON) with threshold THRESH2. DVMON tracks
; the accumulated velocity change during the burn, providing a backup cutoff
; mechanism if the primary VG calculation fails. This redundancy protects
; against guidance computer errors that could lead to excessive propellant use.
;
; Apollo 11, this program executed Eagle's ascent on July 21, 1969, inserting
; the LM into orbit for rendezvous with Columbia. Unlike the throttleable DPS,
; the APS burns at fixed thrust with precise velocity cutoff for orbital accuracy.
;
		CADR	DELAYJOB
		TCF	DYNMDISP

CALCN85		TC	INTPRET
		CALL
			UPDATEVG
		VLOAD	CALL
			VGPREV
			S41.1
		STORE	VGBODY
; Load APS engine parameters: thrust force (FAPS), mass flow rate (MDOTAPS),
; Engine performance parameters define thrust characteristics:
; - FAPS: Ascent Propulsion System thrust force (Newtons, scaled 2^+14)
; - MDOTAPS: Propellant mass flow rate (kg/cs, scaled 2^+3)
; - APSVEX: Exhaust velocity (meters/cs, scaled 2^+7)
; These constants were calibrated from ground testing and used throughout
; the mission for trajectory prediction and guidance computations.
;
; and thrust decay constant (ATDECAY). These constants define the engine
; performance characteristics used throughout the guidance computations.
;
		EXIT
		TC	POSTJUMP
		CADR	SERVEXIT

		COUNT*	$$/P42
		EBANK=	WHICH

;
; ============================================================================
; PROGRAM: P42LM - ASCENT PROPULSION SYSTEM (APS) BURN EXECUTION
;
; This program manages lunar ascent burns using the Ascent Propulsion System
; Load current spacecraft mass from CSMMASS or LEMMASS depending on which
; vehicle is active. Mass decreases during the burn as propellant is consumed,
; affecting thrust-to-weight ratio and requiring continuous guidance updates.
; Accurate mass knowledge is critical for precise velocity targeting.
;
; engine. P42 controlled the critical Eagle ascent from the lunar surface on
; July 21, 1969, inserting the Lunar Module into orbit for rendezvous with
; Columbia (the Command Module piloted by Michael Collins).
;
; APS BURN CHARACTERISTICS:
;   - Fixed thrust (no throttling capability unlike DPS)
;   - Guidance via S40.8 (cross-product steering) and S40.13 (burn length)
;   - Thrust magnitude: FAPS (Ascent Propulsion System thrust force)
;   - Mass flow rate: MDOTAPS (propellant consumption rate)
;   - Velocity of exhaust: APSVEX (specific impulse parameter)
;
; ASCENT SEQUENCE:
;   1. Verify APS staging complete (APSFLBIT check)
;
; The RCS thrust magnitude depends on the jet configuration selected by NJETSFLG:
;   NJETSFLG = 0: Four-jet configuration (FRCS4, maximum thrust ~100 lbf total)
;   NJETSFLG = 1: Two-jet configuration (FRCS2, reduced thrust ~50 lbf total)
; Four-jet mode provides faster maneuvers but consumes more propellant. Two-jet
; mode conserves fuel for long-duration attitude hold or fine trim adjustments.
;
;   2. IMU status check via R02BOTH
;   3. Initialize delta-V monitor (DVMON) with THRESH2 threshold
;   4. Load APS parameters (FAPS, MDOTAPS, ATDECAY) via vector operations
;   5. Execute burn with continuous guidance updates
;   6. Cutoff based on velocity-to-be-gained reaching target
;
; HISTORICAL SIGNIFICANCE: This code executed during Eagle's ascent, beginning
; approximately 21.5 hours after landing. The ascent burn lasted about 7 minutes,
; achieving the precise orbital insertion required for rendezvous with Columbia.
; ============================================================================
;
P42LM		TC	PHASCHNG
		OCT	04024

		CAF	P42ADRES	# INITIALIZATION FOR BURNBABY.
		TS	WHICH

		CS	FLGWRD10
		MASK	APSFLBIT
		CCS	A
		TC	P40ALM
P42STAGE	TC	BANKCALL
# Page 758
		CADR	R02BOTH
		CAF	THRESH2		# INITIALIZE DVMON
		TS	DVTHRUSH
		CAF	FOUR
		TS	DVCNTR

		TC	INTPRET
; Time-to-ignition (TTI) has reached zero. The guidance computer now initiates
; the burn sequence: ullage motor firing (if required), engine valve opening,
; and transition to thrusting guidance. The countdown to ignition is complete.
;
		SET	VLOAD		# LOAD FAPS, MDOTAPS, AND ATDECAY INTO
			AVFLAG		# F, MDOT, AND TDECAY BY VECTOR LOAD.
			FAPS
		STORE	F
		SLOAD	GOTO
			APSVEX
			P40IN

		EBANK=	WHICH

		COUNT*	$$/P47
;
; ============================================================================
; Display V06N40 to crew: Time-To-Ignition (TTI), Velocity-to-be-Gained (VG),
; and Delta-V Monitor (DELTAVM). Updated once per second via CLOKTASK, these
; values allow the crew to monitor burn progress and verify guidance computer
;
; The velocity-to-be-gained vector is now transformed from reference (inertial)
; coordinates to Lunar Module body axes. This transformation allows the crew
; to understand the required velocity change in terms of forward/back, left/right,
; and up/down motion relative to the spacecraft, making manual RCS control intuitive.
;
; computations. The crew can manually terminate the burn if anomalies occur.
;
; PROGRAM: P47LM - BURN EXECUTION ALTERNATIVE / DELTA-V MONITOR
;
; This program provides an alternative burn execution path with midcourse
; averaging capabilities. P47 calls MIDTOAV2 to compute averaged parameters
; and sets up a delayed task (STARTP47) for burn execution.
; CLOKTASK (Clock Task) runs once per second during countdown and burn,
; updating the DSKY display with current burn parameters. Time-to-ignition
; counts down to zero, velocity-to-be-gained shows required ΔV, and delta-V
; monitor shows accumulated thrust. The crew uses these displays to monitor
; guidance computer performance and mission timeline.
;
;
; The twiddle task mechanism schedules STARTP47 to begin after a computed
; time interval, allowing the guidance computer to coordinate burn timing
; with navigation state updates and crew readiness confirmation.
;
; This program demonstrates the AGC's flexible task scheduling architecture,
; where burn execution can be deferred and coordinated with other real-time
; operations through the WAITLIST timer-driven scheduler.
; ============================================================================
;
P47LM		TC	BANKCALL
		CADR	R02BOTH
		TC	INTPRET
		CALRB
			MIDTOAV2

		CA	MPAC +1
		TC	TWIDDLE
		ADRES	STARTP47

		TCF	ENDOFJOB

STARTP47	TC	PHASCHNG
		OCT	05014
		OCT	77777

		EXTEND
		DCA	ACADN83
		DXCH	AVEGEXIT
		CAF	PRIO20
		TC	FINDVAC
;
; Setting minimum deadband (0.3 degrees) tightens the Digital Autopilot's attitude
; control tolerance. The deadband is the angular error zone within which the DAP
; does not fire thrusters. Minimum deadband ensures precise attitude maintenance
; during manual RCS burns, allowing the crew to accurately null out velocity
; components without the DAP counteracting their inputs.
;
		EBANK=	DELVIMU
		2CADR	P47BODY

		TCF	REDO4.2		# CHECKS PHASE 5 AND GOES TO PREREAD
					# SEE TIG-30 IN BURNBABY

CALCN83		TC	INTPRET
		VLOAD	VAD
			DELVCTL
			DELVREF
		STORE	DELVSIN		# TEMP STORAGE FOR RESTARTS
# Page 759
		CALL
			S41.1
; Engine ignition requires precise sequencing:
; 1. Ullage motors fire to settle propellants (prevent vapor ingestion)
; 2. Engine valves open to allow propellant flow
; 3. Igniter fires to initiate combustion
; 4. Thrust builds to nominal level over several seconds
; 5. Guidance computer monitors thrust buildup via accelerometer readings
;
		STORE	DELVIMU
		EXIT
		TC	PHASCHNG
		OCT	10035		# REREADAC AND HERE

		TC	INTPRET
		VLOAD
			DELVSIN
		STORE	DELVCTL
		EXIT

		TC	POSTJUMP
		CADR	SERVEXIT

P47BOD		CAF	V1683
		TC	BANKCALL
		CADR	GOFLASHR
		TC	GOTOPOOH
		TC	GOTOPOOH

		TCF	P47BODY

		TCF	P40PHS1

P47BODY		TC	INTPRET
		VLOAD
			HI6ZEROS
		STORE	DELVIMU
; Subroutine S40.1 computes the required thrust direction vector from the
; velocity-to-be-gained (VG). This unit vector points in the direction the
; spacecraft must accelerate to null the velocity error. The computation
; normalizes VG to unit length, accounting for fixed-point scaling factors.
;
		STORE	DELVCTL
		EXIT
		TC	P47BOD

		COUNT*	$$/P40
IMPLBURN	CA	TGO 	+1
		TC	GETDT
		TC	TWIDDLE
		ADRES	ENGOFTSK
		TC	DOWNFLAG	# TURN OFF IGNFLAG
		ADRES	IGNFLAG
		TC	DOWNFLAG	# TURN OFF ASTNFLG
;
; Display V16N85 shows the three components of velocity-to-be-gained in body axes.
; The crew can manually fire RCS thrusters to null out each component, achieving
; the computed velocity change without automatic guidance. This provides critical
; backup capability if the primary guidance system experiences issues.
;
		ADRES	ASTNFLAG
		TC	DOWNFLAG	# TURN OFF IMPULSW
		ADRES	IMPULSW
		TC	PHASCHNG	# RESTART PROTECT ENGOFTSK (ENGINOFF)
		OCT	40114

		TC	FIXDELAY	# WAIT HALF A SECOND
		DEC	50
; Subroutines S40.2 and S40.3 compute the preferred IMU orientation (REFSMMAT)
; for the burn. The preferred orientation minimizes gimbal motion during thrust,
; reducing the risk of gimbal lock and simplifying attitude control. The
; computation considers both thrust direction and sun/star visibility for
; post-burn navigation alignment.
;
# Page 760
		TC	NOULLAGE	# TURN OFF ULLAGE

		TC	TASKOVER

ENGOFTSK	TC	IBNKCALL	# THIS CODING ALLOWS ENGINOFF ET AL TO BE
		CADR	ENGINOFF	# USED BOTH BY WAITLIST AND BY TC IBNKCALL
		TC	TASKOVER

ENGINOFF	CAF	PRIO12		# MUST BE LOWER PRIO THAN CLOCKJOB
		TC	FINDVAC
		EBANK=	TRKMKCNT
		2CADR	POSTBURN

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
; R60LEM executes the attitude maneuver to achieve the required thrust
; orientation. The Digital Autopilot computes the optimal rotation path,
; fires RCS thrusters to initiate the rotation, and maintains attitude
; during coast. The maneuver must complete before ignition countdown reaches
; the minimum safe time (typically 45 seconds before TIG).
;
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
; LEMPREC (Lunar Module Precision) extrapolates the state vector from current
; time to time-of-ignition (TIG). This prediction accounts for gravitational
; acceleration, ensuring guidance computations use the correct position and
; velocity at burn initiation. The extrapolation uses Encke method numerical
; integration for precision in the Moon's gravitational field.
;
			XDELVFLG
			QTEMP1
			NORMSW
			180SETUP
;
; P42 begins by verifying the Ascent Propulsion System is properly staged and
; ready for ignition. The APSFLBIT check ensures the APS engine has been armed
; and the descent stage has been separated (if applicable). During Apollo 11's
; ascent on July 21, 1969, this check confirmed Eagle was ready to lift off.
;
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
; Velocity cutoff occurs when velocity-to-be-gained approaches zero. The
; guidance computer monitors VG magnitude and commands engine shutdown when
; the remaining velocity error is smaller than the engine's minimum impulse
; capability. This ensures precise trajectory targeting without over-burning.
;
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
; Restart protection ensures the guidance computer can recover from power
; transients during critical burn phases. If a restart occurs during powered
; flight, the program resumes at a safe checkpoint with guidance state restored
; from protected memory. This capability was essential during Apollo 11's descent
; when 1202 program alarms occurred due to computational overload.
;
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
;
; The APS thrust parameters are loaded from fixed memory constants:
;   FAPS:    Ascent engine thrust force (approximately 3500 lbf fixed thrust)
;   MDOTAPS: Propellant mass flow rate (determines burn time for given delta-V)
;   APSVEX:  Effective exhaust velocity (related to specific impulse)
;   ATDECAY: Thrust decay coefficient (models engine shutdown transient)
;
		TC	Q

# **************************************

SEC15DP		OCT	00000		# DON'T SEPARATE
SEC15		DEC	1500		# DON'T SEPARATE
; If automatic guidance fails, the crew can take manual control of the burn.
; Display V50N25 prompts: '203 A/P TO PGNCS, AUTO-THROTTLE MODE, AUTO ATTITUDE
; CONTROL' indicating the computer is in full automatic mode. The crew can
; override to manual throttle and attitude control if needed, though this
; requires extensive training and real-time trajectory computation skills.
;
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
;
; The delta-V monitor (DVMON) tracks the accumulated velocity change during the
; burn, comparing it against the THRESH2 threshold to detect thrust anomalies.
; If the actual velocity change deviates significantly from the expected profile,
; DVMON can trigger an alarm, alerting the crew to possible engine malfunctions
; such as partial thrust loss or premature shutdown.
;
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
S40.1		STQ	DLOAD
; Velocity-to-be-gained updates occur at high frequency during burns,
; typically every 2 seconds. Each update integrates accelerometer readings,
; subtracts gravity effects computed from current position, and applies
; navigation corrections from Kalman filtering. The resulting VG vector
; drives both steering law (for attitude) and cutoff logic (for burn duration).
;
			QTEMP
			TIG
		STORE	TIGSAVE
DELVTEST	BOFF
			XDELVFLG
			S40.1B
CALCTHET	SETPD	VLOAD
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
		DMP	DDV
			THETACON
		DMP	DMP
			DELVSAB
			WEIGHT/G
		DDV
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
; During powered flight, the guidance computer continuously updates the
; velocity-to-be-gained (VG) vector. VG represents the velocity change still
; required to achieve the targeted trajectory. As the engine thrusts, VG
; decreases toward zero, at which point thrust cutoff occurs.
;
		UNIT	PDDL		# UNIT(DELTA VP) IN P.D.L. 6
			14D
		COS	VXSC
		VAD	VXSC
			VGTIG
			36D
		VSL2 	VAD
;
; The guidance system initializes velocity-to-be-gained computations by loading
; the current navigation state (position and velocity vectors) and computing
; the time remaining until ignition (TGO = time-to-go). Lambert guidance will
; continuously update the required velocity vector as the spacecraft coasts
; toward the ignition point.
;
		STADR
# Page 766
		STORE	VGTIG		# VG IGNITION SCALED AT 2(+7) M/CS
; The VG update accounts for:
; - Actual velocity change from engine thrust (measured via accelerometers)
; - Gravity effects (computed from current position)
; - Navigation state errors (corrected via Kalman filtering)
;

		UNIT
		STOVL	UT		# THRUST DIRECTION SCALED AT 2(+1)
			VGTIG
		PUSH	CALL
			GET.LVC		# VGTIG IN LV COOR AT 2(+7) M/CS IN DELVLVC
		GOTO
			QTEMP
S40.1B		DLOAD
			TIG
		STORE	TDEC1
		BDSU
			TPASS4
; Cross-product steering (S40.8) computes attitude error as the cross product
; of current thrust direction and desired thrust direction. The resulting error
; vector is perpendicular to both, indicating the rotation axis and magnitude
; needed to align thrust with the velocity error direction. This elegant
; technique enables efficient attitude control during powered flight.
;
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
			INITVEL
		VLOAD	PUSH
			DELVEET3	# VGTIG = VR - VN.
		STORE	VGTIG
; The steering law computes the required thrust direction to null the velocity
; error. Cross-product steering (via S40.8) generates attitude commands that
; orient the spacecraft so engine thrust is aligned with the desired velocity
; change direction. The Digital Autopilot then fires RCS thrusters to achieve
; and maintain this attitude.
;
		UNIT			# UT = UNIT (VGTIG)
		STODL	UT
			36D
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
;
; Guidance updates occur at regular intervals (typically every 2 seconds) during
; the burn. Each update recomputes the required steering direction based on
; current position, velocity, and remaining time to cutoff. This closed-loop
; guidance corrects for thrust variations, mass flow rate uncertainties, and
; navigation state errors, ensuring the burn achieves the targeted orbit.
;
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

		COUNT*	$$/S40.2
S40.2,3		VLOAD			# UT:  DESIRED THRUST DIRECTION (HALF-UNIT)
			UT		# (PUT INTO TOP OF PUSH-DOWN-LIST.)
		MXV	VSL1		# TRANSFORM THRUST DIRECTION TO STABLE-
			REFSMMAT	# MEMBER FROM REFERENCE COORDS (RESCALE).
		STOVL	POINTVSM	# SAVE FOR "VECPOINT" ROUTINE (LEMMANU).
			UNITX		# SCAXIS SET TO +X, FOR P40 AND P42 AND
		STOVL	SCAXIS		# FOR P41 IF RCS NOT -X,+Y,-Y,+Z,-Z.

			UT		# ASSUME +X BURN ALWAYS, EVEN FOR RCS.
PLUSX		STORE	XSCREF		# XSCREF = UT (DESIRED THRUST DIRECTION)
		VXV	UNIT		# RTIG = POSITION AT TIME-OF-IGNITION.
			RTIG		# YSCREF = UNIT(UT X RTIG)
		PDDL	BHIZ
			36D		# TEST MAGNITUDE OF UT X RTIG
			FIXY		# IF SMALL, USE UT X VTIG AS YSC
STORY		VLOAD	STADR
		STORE	YSCREF
		VXV	VSL1		# COMPUTE (YSCREF X XCREF), BUT FOR A
			XSCREF		# RIGHT HANDED SYSTEM, NEED (X CROSS Y).
		VCOMP			# ZSCREF = - (YSCREF X XSCREF)
		STORE	ZSCREF		#        = + (XSCREF X YSCREF)

		SET	RVQ
			PFRATFLG
FIXY		VLOAD	VXV		# IN THIS CASE,
			XSCREF		# YSCREF = UNIT(XSCREF X VTIG)
			VTIG
		UNIT	PUSH
		GOTO
			STORY
# Page 770
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
;
; During the burn, the guidance computer continuously monitors velocity-to-be-gained
; (VG) and compares it against the cutoff threshold. When VG magnitude drops below
; the threshold (typically a few feet per second), the engine shutdown sequence
; initiates. This velocity-based cutoff ensures precise orbit insertion regardless
; of slight variations in thrust or mass flow rate.
;
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
S40.8		BOF			# GENERATE VR IF NOT EXTERNAL DELTA-V BURN
			XDELVFLG
			RASTEER1
		VLOAD	VSU
			VGPREV
			DELVREF
VGAIN*		STORE	VG		# VELOCITY TO BE GAINED SCALED AT (7) M/CS
		MXV	VSL1
			REFSMMAT
		STORE	UNFC/2
BDTOK		VLOAD	ABVAL
			VG
		STORE	VGDISP
TGDCALC		SETPD	VLOAD
			0
			VG
		STOVL	VGPREV
			DELVREF
		BOFF	VCOMP
			STEERSW
			QPRET
		UNIT
;
; After engine cutoff, the guidance computer maintains velocity-to-be-gained
; calculations for several seconds to support possible RCS trim maneuvers. Small
; residual velocity errors (typically a few feet per second) may remain due to
; engine shutdown transients or thrust misalignment. The crew can manually null
; these residuals using RCS thrusters, as displayed on V16N85.
;
		DOT	PUSH
			VG
		BPL	DDV
			ALARMIT		# DELV IS MORE THAN 90 DEGREES FROM VG.
			VEX
		DAD	DMP
			DPHALF
		SR	DDV
			10D
			36D
		DMP	DAD
			-FOURDT
			TDECAY
		STORE	TGO
		DAD
			PIPTIME
		STODL	TIG
			TGO
		DSU	BPL
			FOURSECS	# 400 CS
			FINDCDUW -2
		SET	CLRGO
			IMPULSW
			STEERSW
			QPRET

ALARMIT		EXIT

# Page 772
		TC	ALARM
		OCT	01407
		TC	INTPRET
		GOTO			# SKIP TGO COMPUTATION BUT CALL FINDCDUW.
			FINDCDUW -2	# FINDCDUW WILL EXIT TO UPDATEVG +3.

-FOURDT		2DEC	-800 B-18	# -4 (200 CS.) B(-18)
FOURSECS	2DEC	400		# 400 CS SCALED AT 2(+28) CS
2VEXHUST	=	VEX

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
S40.13		TC	INTPRET
		SETPD	CLEAR
			00D
			IMPULSW		# ASSUME NO STEERING UNTIL FOUND OTHERWISE
		VLOAD	ABVAL
			VGTIG		# VELOCITY TO BE GAINED AT +7
		PDDL	DMP		# 00D = MAG OF VGTIG AT +7
			4SEC(17)	# CORRECT VG FOR 4 SECS OF 2 JET ULLAGE
			FRCS2
		DDV	SL1		# SCALE
; Calculate the time-to-go (TGO) until thrust cutoff. TGO computation uses
; the Tsiolkovsky rocket equation accounting for:
; - Current velocity-to-be-gained magnitude (VGMAG)
; - Engine thrust level (FTHRUST)
; - Propellant mass flow rate (MDOT)
; - Current spacecraft mass (CSMMASS or LEMMASS)
; - Exhaust velocity (VEX - specific impulse parameter)
;
			WEIGHT/G
		BDSU	PUSH
		BOFF	SET
			APSFLAG
			S40.13D		# FOR DPS ENGINE
			NOTHROTL
		DLOAD	DDV		# 00D = MAG OF VGTIG CORRECTED
;
; Lambert guidance solves the classical two-point boundary value problem: given
; current position and a target position, compute the velocity change required
; to achieve intercept in a specified time. This is fundamental to rendezvous,
; orbit transfer, and landing guidance. The solution accounts for gravitational
; acceleration during the transfer.
;
			K1VAL		# M.NEWTONS-CS AT +24
			WEIGHT/G
		BDSU	BMN
; The guidance computer displays TGO to the crew on the DSKY (via V06N40),
; updating once per second during the burn. Crew monitors this countdown to
; verify the guidance computer is controlling thrust duration properly.
;
; Lambert's problem solves for the velocity required to transfer between two
; The WAITLIST timer-driven scheduler manages time-delayed tasks during burns.
; Phase changes (via PHASCHANG) coordinate transitions between guidance phases:
; pre-burn coast, ignition, thrusting, cutoff, post-burn. Each phase has
; specific computational requirements and restart protection groups, ensuring
; robust operation even during power transients.
;
; positions in a specified time. Named after Johann Heinrich Lambert (1728-1777),
; this classical orbital mechanics problem is fundamental to rendezvous guidance.
; The solution accounts for gravitational acceleration and computes the optimal
; ΔV to achieve the targeted state at the predicted time.
;
# Page 774
			00D
			S40.131		# TGO LESS THAN 100 CS
		PDDL	DMP		# 02D = TEMP1 AT +7
			MDOT

# MDOT REPRESENTS THE RATE OF DECREASE OF VEHICLE MASS DURING ENGINE
# BURN IN KILOGRAMS/CS.  WHEN SATURN IS USED, THE SCALING MAY
# REQUIRE ADJUSTMENT.

			3.5SEC		# 350 CS AT +14
		BDSU	PDDL
			WEIGHT/G
			F
		DMP	SR2		# SCALE
			5SECS
		DDV	PUSH		# 04D = TEMP2
		BDSU	BPL
			02D
			S40.13D
		DLOAD	BDDV
		DMP	DAD
			5SECS
			1SEC2D		# 100 CS AT +14
		GOTO
			S40.132
S40.131		DLOAD	DMP
			WEIGHT/G
		SR1	PUSH
		DAD	DDV
			K2VAL		# M.NEWTON CS AT +24
			K3VAL		# M.NEWTON CS AT +10
S40.132		SET	EXIT
			IMPULSW
S40.132*	TC	TPAGREE
		CA	MPAC
		XCH	L
		CA	ZERO
		DXCH	TGO
		TCF	S40.134

S40.13D		DLOAD	DMP		# FOR DPS ENGINE
			00D
			WEIGHT/G
		PUSH	BON
; Fixed-point arithmetic scaling is critical for AGC precision:
; - Position vectors: scaled by 2^+29 meters (1 unit ≈ 1.86 nanometers)
; - Velocity vectors: scaled by 2^+7 meters/centisecond (1 unit ≈ 7.8 mm/cs)
; - Time values: scaled by 2^+28 centiseconds (1 unit ≈ 3.73 nanoseconds)
; These scaling factors maximize precision within the 15-bit signed word length
; while representing cislunar trajectory parameters. All computations must
; account for these scales to maintain accuracy.
;
			APSFLAG
			APSTGO
		DDV	CLEAR
			S40.136
			NOTHROTL
		BOV	PUSH
# Page 775
			S40.130V
S40.127		DSU	BPL
			6SEC		# 600.0 CS AT +14
			S40.138
		DAD	GOTO
			6SEC
			S40.132
S40.133		EXIT
S40.134		TC	PHASCHNG
		OCT	00003
		TC	ENDOFJOB
S40.130V	DLOAD	SR4		# RECOMPUTED TGO IN TIMER UNITS
		DDV
			S40.136_	# S40.136 SHIFTED LEFT 10
		STORE	TGO
		EXIT
		TCF	S40.134		# REJOIN COMMON CODING FOR RESTART PROTECT

S40.138		DSU	BPL
			89SECS
			STORETGO
;
; The navigation state (position and velocity) is extrapolated forward to the
; time of ignition using conic propagation. This accounts for gravitational
; acceleration and ensures the guidance computation uses the predicted state
; at TIG (time of ignition), not the current state which will be outdated by
; the time the burn actually begins.
;
		SET
			NOTHROTL
STORETGO	DLOAD			# LOAD TGO AT 2(14)
		EXIT
		TCF	S40.132*

APSTGO		DDV	SL2
			FAPS
		GOTO
			STORETGO +1
1SEC2D		2DEC	100.0 B-14	# 100.0 CS AT +14

3.5SEC		2DEC	350.0 B-13	# 350 CS AT +13

5SECS		2DEC	500.0 B-14	# 500.0 CS AT +14

6SEC		2DEC	600.0 B-14	# 600.0 CS AT +14

89SECS		2DEC	8900.0 B-14

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

		EBANK=	VGPREV
		COUNT*	$$/S40.9
;
; ============================================================================
; SUBROUTINE: S40.9 - LAMBERT AIMPOINT GUIDANCE (VTOGAIN)
;
; This subroutine computes the velocity-to-be-gained (VG) vector required to
; achieve a targeted trajectory using Lambert guidance. Lambert's problem
;
; Gimbal trim timing is critical. The drive motors run at constant angular velocity
; (approximately 4 degrees per second for pitch and roll). Starting from full
; positive gimbal position (+6 degrees), the motors run for PITTIME and ROLLTIME
; centiseconds to reach the computed trim positions. Accurate timing ensures
; thrust vector alignment within 0.5 degrees of the center-of-mass line.
;
; solves for the velocity change needed to transfer between two orbital
; positions in a specified time, fundamental to rendezvous and orbit transfer.
;
; INPUTS:
;   RN:       Current position vector (meters, scaled 2^+29)
;   VN:       Current velocity vector (meters/cs, scaled 2^+7)
;   VPREV:    Last computed required velocity vector (meters/cs, scaled 2^+7)
;   TIG:      Time of ignition (centiseconds, scaled 2^+28)
;   DLTARG:   Computation cycle interval = 200 cs (scaled 2^+28)
;   PIPTIME:  Time of RN and VN measurement (cs, scaled 2^+28)
;   GDT/2:    Half of gravity-induced velocity change (meters/cs, scaled 2^+7)
;   DELVREF:  Velocity change during last 2 seconds (meters/cs, scaled 2^+7)
;
; OUTPUTS:
;   VGPREV:   Velocity-to-be-gained vector (meters/cs, scaled 2^+7)
;   VGDISP:   Magnitude of VGPREV for crew display purposes
;   VRPREV:   Required velocity vector (meters/cs, scaled 2^+7)
;   BDT:      B vector guidance term (meters/cs, scaled 2^+7)
;
; GUIDANCE PARAMETERS:
;   EPS1, EPS2: Epsilon angles controlling guidance sensitivity
;   NORMSW:     Normal steering switch (determines epsilon = 10° or 45°)
;   AVFLAG:     Active vehicle flag (set for LM active during rendezvous)
;
; This subroutine calls HAVEGUES to initialize Lambert trajectory computation
; and returns via ENDS40.9 after setting up phase change for job scheduling.
; ============================================================================
;
S40.9		TC	INTPRET
		SETPD
			00D
		SET	DLOAD
			AVFLAG		# SET AVFLAG FOR LEM ACTIVE
			HI6ZEROS
		PDDL
			EPS1
		BOFF	DAD		# EPSILON4 = 10 OR 45 DEGREES.
			NORMSW
			EPSSMALL
			EPS2
EPSSMALL	PUSH	CALL
			HAVEGUES
ENDS40.9	EXIT
		TC	PHASCHNG
		OCT	2
		TCF	ENDOFJOB

RASTEER1	VLOAD	ABVAL
			RN
		LXC,2	SL*
			RTX2
			0,2
		STOVL	RMAG
			RTARG
		VSU	RTB
			RN
			NORMUNX1
		STODL	IC
			36D		# C(36D) = ABVAL(C)
		XAD,2	SL*
			X1
# Page 777
			0,2
		STORE	30D
		NORM	DMP
			X2
			RMAG
		NORM	XAD,2
			X1
			X1
		SXA,2
			MUSCALE
		STODL	R1C		#			2(+56 -X)
			30D
		SR1	PDDL
			RMAG
		SR1	PDDL
			RTMAG
		SR1	DAD
		DAD	STADR
		STORE	SS		# SS = (R1 + R2 + C)/2
		DSU	DMP
			30D
			MU/A
		BDSU
			MUASTEER
		PDDL	DSU
			SS
			RMAG
		NORM	SR1
			X1
		DDV	DMP
			R1C
		XSU,2	SL*
			X1
; Engine gimbal control allows thrust vector steering through the spacecraft's
; center of mass. Without proper trim, off-center thrust creates torques that
; waste propellant through RCS counter-torques. The gimbal drive motors position
; the engine in pitch and roll axes to null these unwanted torques.
;
			1,2
		LXA,2
			MUSCALE
;
; The gimbal drive motors physically rotate the engine thrust vector to align
; with the spacecraft center of mass. Proper trim minimizes unwanted torques
; that would require continuous RCS thruster corrections, saving propellant.
; The drives run at constant speed, so trim position is achieved by timing
; the drive-on period (PITTIME for pitch, ROLLTIME for roll).
;
		SQRT	SIGN
			GEOMSGN
		STORE	32D		# + OR - A
		DLOAD	DMP
			SS
			MU/A
		BDSU
			MUASTEER
		PDDL	DSU
			SS
			RTMAG
		NORM	SR1
			X1
		DDV	DMP

# Page 778
			R1C
		XSU,2	SL*
			X1
			1,2
		SQRT	PDDL		# -B (NO SIGN)
			SS
		DSU	DDV
			30D
			SS
		SQRT	PUSH
		SR1	ASIN
		DMP	PDDL
			2PI+3
		PDDL	DDV
;
; When both pitch and roll gimbal drives complete their trim positioning, the
; GMBDRVSW flag is tested to determine which axis finished first. The second
; axis to complete schedules the TRIMDONE job, which returns control to the
; calling program (R03). This dual-completion logic ensures both axes reach
; their trim positions before continuing the burn sequence.
;
			30D
			SS
		BOV
			+1
		SQRT	DMP
		SR3	BDSU
		SIGN	PDDL
			GEOMSGN
			2PI+3
		SR2	DSU
		DMP	PDDL
			SS
			SS
		SR3	SQRT
		DMP
		PDDL	SL3
			MUASTEER
		SQRT	BDDV
		DSU	DAD
			TPASS4
			PIPTIME
		STODL	30D
		SIGN
			30D		# B WITH SIGN
		STORE	30D
		BON	VLOAD
			NORMSW
			180MESS
			IC
		VSU	UNIT
			UNIT/R/
		VXSC	PDVL
			30D
			IC
		VAD	UNIT
			UNIT/R/
# Page 779
GETVRVG1	VXSC	VAD
			32D
GETVRVG2	LXC,2	VSR*
			RTX2
			0 	-1,2
		STORE	VIPRIME
		GOTO
			ASTREND -2
180MESS		VLOAD	DOT
			IC
			UNIT/R/
		BMN	VLOAD
			NEGPROD
			IC
		VSR1	PDVL
			UNIT/R/
		VSR1	VAD
		UNIT
		PUSH	VCOMP		# FOR A
		VXV	SIGN
			UN
			GEOMSGN
		UNIT	VXSC
			30D
		PDVL			# UNIT(IC-IR)	+-B
; REFSMMAT (Reference to Stable Member Matrix) defines the IMU orientation
; relative to the reference (inertial) coordinate system. This matrix is a
; half-unit matrix (scaled by 2^-1) requiring the computation to account for
; this scaling. The transformation sequence: Reference → Stable Member → Body
; converts abstract inertial vectors into intuitive crew-reference coordinates.
;
		GOTO
			GETVRVG1
NEGPROD		VLOAD	VSR1
			UNIT/R/
		PDVL	VSR1
			IC
		VSU	UNIT
		PUSH
		VXV	SIGN
			UN		# FOR B
			GEOMSGN
		UNIT	VXSC
			32D
		PDVL
		VXSC	VAD
			30D
		GOTO
			GETVRVG2
		VSU
			VN1
ASTREND		STORE	DELVEET3
FIRSTTME	SLOAD	BZE
			RTX2
			GETGOBL
		VLOAD	GOTO		# NO OBLATENESS COMP IF IN MOON SPHERE
# Page 780
			DELVEET3
			NOGOBL
GETGOBL		VLOAD	UNIT		# CALCULATE OBLATENESS TERM.
			RN
		DLOAD	DSU
			PIPTIME		#              2
			GOBLTIME	# G    = -(MU/R )(UNITGOBL)(T-TIG)
		DMP	DDV		#  OBL
			EARTHMU
			34D		# 34D = /RN/ (2) FROM UNIT OPERATION.
		VXSC	VAD
			UNITGOBL
			DELVEET3	# OUTPUT FROM INITVEL VG = VR - VN
NOGOBL		STORE	DELVEET3	# VG = VR + GOBL - VN
		GOTO
			VGAIN*

2PI+3		2DEC	3.141592653 B-2

# Page 781
# TRIMGIMB	(FORMERLY S40.6)
# MOD 0		24 FEB 67	PETER ADLER
#
# FUNCTION:
#	TRIMS DPS ENGINE TO MINIMIZE THRUST/CG OFFSET.  ENGINE IS GIMBALLED TO FULL + PITCH AND + ROLL (TO LOCK)
#	FOR REFERENCE AND IS THEN BROUGHT BACK TO TRIM POSITION BY RUNNING FOR THE PROPER TIMES (TO BE
;
; The first transformation step converts from reference (inertial) coordinates
; to stable member (IMU platform) coordinates using REFSMMAT. The stable member
; is the physical gyro-stabilized platform inside the IMU that maintains a fixed
; inertial orientation. REFSMMAT defines the relationship between the desired
; reference frame (mission-specific, such as lunar-local vertical) and the
; stable member's actual orientation.
;
#	SPECIFIED BY GAEC) IN - PITCH AND - ROLL.
#
# CALLING SEQUENCE:
#	VIA WAITLIST FROM R03
#
# INPUT:
#	PITTIME		TIME TO RUN FROM FULL + PITCH TO TRIM (CS)
#	ROLLTIME	TIME TO RUN FROM FULL + ROLL TO TRIM (CS)
#
# SUBROUTINES USED:
#	WAITLIST, FIXDELAY, VARDELAY, FLAGUP, FLAGDOWN, NOVAC

		COUNT*	$$/S40.6
		EBANK=	ROLLTIME	# OCTAL MASKS: PRIO5=05000 EBANK5=02400

;
; ============================================================================
; SUBROUTINE: TRIMGIMB - ENGINE GIMBAL TRIM CONTROL
;
; This subroutine controls the engine gimbal drive motors to trim the thrust
; vector during a burn. Proper gimbal trim alignment ensures thrust passes
; through the spacecraft's center of mass, preventing unwanted torques that
; would require RCS thruster corrections and waste propellant.
;
; TRIM SEQUENCE:
;   1. Turn off pitch and roll drives (clear PRIO5 bits in CHAN12)
;   2. Turn on +PITCH and +ROLL drives (set EBANK5 bits)
;   3. Wait 1 minute (6000 cs) to reach full positive gimbal position
;   4. Turn off +PITCH and +ROLL drives
;   5. Turn on -PITCH and -ROLL drives (opposite direction)
;   6. Run PITTIME centiseconds for pitch trim position
;   7. Run ROLLTIME centiseconds for roll trim position
;
; The second transformation step converts from stable member coordinates to
; spacecraft body coordinates using the CDU (Coupling Display Unit) gimbal angles.
; The CDUs measure the physical gimbal angles of the IMU platform relative to
; the spacecraft body. This transformation accounts for any rotation between the
; stable member's inertial orientation and the spacecraft's current attitude.
;
;   8. Shut off drives when trim positions achieved
;
; INPUTS:
;   PITTIME:  Time to run from full +pitch to trim position (centiseconds)
;   ROLLTIME: Time to run from full +roll to trim position (centiseconds)
;
; The gimbal drive uses a twiddle-task mechanism (PITCHOFF) to shut off pitch
; after PITTIME expires. Roll shuts off via VARDELAY after ROLLTIME. The first
; axis to complete sets GMBDRVSW flag; when both complete, TRIMDONE job is
; scheduled to return control to the calling program (R03).
;
; CHANNEL CONTROL:
;   CHAN12 BIT10: Pitch gimbal drive control
;   CHAN12 BIT12: Roll gimbal drive control
;   PRIO5: Negative drive direction bits
;   EBANK5: Positive drive direction bits
; ============================================================================
;
TRIMGIMB	TC	DOWNFLAG	# GMBDRVSW FLAG IS SET WHEN EITHER ROLL OR
		ADRES	GMBDRVSW	# PITCH IS COMPLETED, WHICHEVER IS FIRST.

		CS	PRIO5		# TURN OFF - PITCH, - ROLL, IF ON.
		EXTEND
		WAND	CHAN12
		CAF	EBANK5		# TURN ON + PITCH, + ROLL.
		EXTEND
		WOR	CHAN12
		TC	FIXDELAY	# WAIT ONE MINUTE TO MAKE SURE ENGINE IS
		DEC	6000		# AT FULL + PITCH AND FULL + ROLL
		CS	EBANK5		# TURN OFF + PITCH, + ROLL.
		EXTEND
		WAND	CHAN12
		CAF	PRIO5		# TURN ON - PITCH, - ROLL.
		EXTEND
		WOR	CHAN12
		CAE	PITTIME		# GET TIME TO SHUT OFF - PITCH AND SET UP
		TC	TWIDDLE		# TWIDDLE-TASK TO TURN IT OFF THEN
		ADRES	PITCHOFF

		CAE	ROLLTIME	# GET TIME TO SHUT OFF - ROLL AND GO AWAY
		TC	VARDELAY	# UNTIL THEN
		CS	BIT12
		EXTEND
		WAND	CHAN12		# SHUT OFF ROLL
ROLLOVER	CA	FLAGWRD6	# IF HERE INLINE (ROLL DONE) IS PITCH DONE
		MASK	GMBDRBIT	# IF HERE FROM PITCHOFF, IS ROLL DONE?
		EXTEND
		BZF	PITCHOFF +4	# NO.  SET FLAG, ROLL OR PITCH DONE.
		CAF	PRIO10		# RETURN TO R03.
		TC	NOVAC
		EBANK=	WHOCARES
# Page 782
		2CADR	TRIMDONE

		TC	TASKOVER
PITCHOFF	CS	BIT10
		EXTEND
		WAND	CHAN12		# SHUT OFF PITCH
		TCF	ROLLOVER	# SEE IF ROLL HAS FINISHED ALSO.
		TC	UPFLAG		# ROLL DONE; OR PITCH DONE; BUT NOT BOTH.
		ADRES	GMBDRVSW
		TC	TASKOVER

# Page 783
# SUBROUTINE NAME:  S41.1	MOD. NO. 0	DATE: FEBRUARY 28, 1967
# MOD. NO. 1	DATE: JANUARY 23, 1968, BY PETER ADLER (MIT/IL)
#
# AUTHOR: JONATHON D. ADDLESTON (ADAMS ASSOCIATES)
#
# S41.1 PERFORMS THE COORDINATE SYSTEM TRANSFORMATION FROM THE REFERENCE FRAME TO THE BODY OF THE LM.
# SPECIFICALLY, IT IS USED TO TRANSFORM A VELOCITY (SCALED AT 2(+7) METERS/CENTISECOND) FROM REFERENCE TO LM AXIS
# COORDINATES.  FIRST THE VECTOR IS TRANSFORMED TO THE STABLE MEMBER COORDINATES BY THE MATRIX REFSMMAT.  THIS
# LEAVES THE VECTOR IN MPAC, SCALED AT 2(+8) METERS/CENTISECOND.  THEN
# THE SUBROUTINE CDUTRIG IS CALLED TO SET UP THE DOUBLE-PRECISION CDU VECTOR ALONG WITH ITS SINES AND COSINES.
# THE VECTOR IS THEN TRANSFORMED FROM STABLE MEMBER COORDINATES TO SPACECRAFT (OR LM) COORDINATES BY THE
# SUBROUTINE *SMNB*.  FINALLY, THE VECTOR IS RESCALED TO 2(+7) METERS/CENTISECOND, AND CONTROL IS RETURNED BO THE
# CALLER WITH C(MPAC) = VELOCITY(LM).
#
# CALLING SEQUENCE:
#	L	VLOAD	CALL
#	L +1		VELOCITY(REF)		# SCALED AT 2(+7) M/CS IN REFERENCE COORDS.
#	L +2		S41.1
#	L +3	STORE	VELOCITY(LM)		# SCALED AT 2(+7) M/CS IN LM BODY AXIS SYS.
#
# SUBROUTINES CALLED:
#	1.	CDUTRIG,
#			WHICH CALLS CDULOGIC.
#	2.	*SMNB*
#
# NORMAL RETURN:  L +3 (SEE CALLING SEQUENCE, ABOVE.)
;
; The subroutine returns via RVQ (Return Via Q register), restoring program
; control to the caller with the transformed velocity vector in MPAC (Multi-Purpose
; Accumulator). The result is now in body coordinates, suitable for crew display
; or for integration with the Digital Autopilot's body-axis control laws.
;
#
# ALARM/ABORT MODES:  NONE.
#
# RESTART PROTECTION:  NONE.
#
# Page 784
# INPUT:
#	1.	REFSMMAT.
#	2.	CDUX, CDUY, CDUZ.
#	3.	VELOCITY (REF) IN MPAC.
#
# OUTPUT:
#	1.	CSUSPOT:	DOUBLE PRECISION CDU VECTOR, ORDERED Y,Z,X.
#	2.	SINCDU:		HALF SINES OF CDUSPOT COMPONENTS
#	3.	COSCDU:		HALF COSINES OF CDUSPOT COMPONENTS.
#	4.	MPAC:		VELOCITY(LM) (SCALED AT 2(+7) METERS/CENTISECOND)
#
# DEBRIS:  NONE.
#
# CHECKOUT STATUS:  CODED

		COUNT*	$$/S41.1
;
; ============================================================================
; SUBROUTINE: S41.1 - REFERENCE TO BODY COORDINATE TRANSFORMATION
;
; This subroutine transforms a velocity vector from reference (inertial)
; coordinates to Lunar Module body-axis coordinates using the current IMU
; gimbal angles. This transformation is essential for displaying velocity
; information to the crew in intuitive body-axis terms (forward/back,
; left/right, up/down) rather than abstract inertial coordinates.
;
; TRANSFORMATION SEQUENCE:
;   1. Transform from Reference to Stable Member coordinates via REFSMMAT
;      (Reference to Stable Member Matrix, the IMU orientation)
;   2. Scale result by half due to REFSMMAT being a half-unit matrix
;   3. Transform from Stable Member to Body coordinates via CDU angles
;      (Current Display Unit angles = IMU gimbal angles)
;
; INPUTS:
;   MPAC:         Velocity vector in reference coordinates (meters/cs, 2^+7)
;   REFSMMAT:     Reference to Stable Member transformation matrix (half-unit)
;   CDUX,CDUY,CDUZ: Current IMU gimbal angles from Coupling Display Units
;
; OUTPUTS:
;   CSUSPOT:      Double-precision CDU vector, ordered Y, Z, X
;   SINCDU:       Half sines of CDUSPOT components (for rotation matrices)
;   COSCDU:       Half cosines of CSUSPOT components (for rotation matrices)
;   MPAC:         Velocity vector in LM body coordinates (meters/cs, 2^+7)
;
; The subroutine returns to caller via RVQ (Return Via Q register) after
; calling CDU*SMNB (CDU times Stable Member to Navigation Base transformation).
;
; MISSION USE: During burns, the crew monitors velocity-to-be-gained in body
; axes on the DSKY display (V16N85), enabling manual RCS corrections if needed.
; This transformation makes those body-axis components meaningful to the crew.
; ============================================================================
;
S41.1		MXV	VSL1		# CONVERT VECTOR IN MPAC FROM REF AT 2(+7)
			REFSMMAT	# TO SM AND RESCALE DUE TO HALF-UNIT MATRIX
		GOTO			# CONVERT TO BODY AT 2(+7) USING PRESENT
			CDU*SMNB	# CDU ANGLES.  CDU*SMNB WILL RETURN
					# VIA RVQ TO THE CALLER OF S41.1.
