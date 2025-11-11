# Copyright:	Public domain.
# Filename:	DAPIDLER_PROGRAM.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1410-1420
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

# Page 1410
; ============================================================================
; FILE: DAPIDLER_PROGRAM.agc
; MODULE: Digital Autopilot (DAP) Idler Program
; MISSION PHASE: all-phases (continuous background operation)
;
; TL;DR: Background autopilot housekeeping program that runs 10 times per
;        second during idle periods. Monitors spacecraft mode switches and IMU
;        status to determine when the DAP should activate. Performs DAP
;        initialization, calculates attitude errors, and drives the FDAI
;        (Flight Director Attitude Indicator) error display needles.
;
; COMMENT-ONLY READERS: This program runs quietly in the background, waiting
;        for the crew to enable the autopilot and ensuring the attitude error
;        display shows current spacecraft orientation relative to desired.
; CODE-ALONG READERS: Study the executive scheduler integration, mode switch
;        polling logic, and FDAI error display interface via DAC channels.
; ============================================================================
;
# THE DAPIDLER PROGRAM IS STARTED BY FRESH START AND RESTART.		  THE DAPIDLER PROGRAM IS DONE 10 TIMES
# PER SECOND UNTIL THE ASTRONAUT DESIRES THE DAP TO WAKE UP, AND THE IMU AND CDUS ARE READY FOR USE BY THE DAP.
# THE NECESSARY INITIALIZATION OF THE DAP IS DONE BY THE DAPIDLER PROGRAM.
		BANK	16
		SETLOC	DAPS1
		BANK

		EBANK=	AOSQ

		COUNT*	$$/DAPID

; ============================================================================
; CHEKBITS - Mode Switch Status Check
;
; This subroutine examines the spacecraft mode select switch on the crew
; control panel. If the switch is in the OFF position (both bit 13 and 14
; set in channel 31), the DAP remains idle with no attitude error display.
; This gives the crew positive control over when the autopilot becomes active.
; ============================================================================

CHEKBITS	EXTEND
		READ	CHAN31		# IF BOTH BIT13 AND BIT14 ARE ONE, THEN
		COM			# THE MODE SELECT SWITCH IS IN THE OFF
		MASK	BIT13-14	# POSITION, AND SO THE DAP SHOULD BE OFF,
		EXTEND			# WITH NO ATTITUDE ERROR DISPLAY.
		BZF	MOREIDLE

; The DAP requires the Inertial Measurement Unit (IMU) to be operational
; before it can control spacecraft attitude. This section verifies IMU status.

		CS	IMODES33
		MASK	BIT6
		CCS	A
		TCF	JUMPDSP
		CS	RCSFLAGS	# IMU NOT USABLE.  SET UP INITIALIZATION
		MASK	BIT3		# FLAG FOR ATT ERROR DISPLAY ROUTINE.
		ADS	RCSFLAGS
		TCF	SHUTDOWN

; The spacecraft can be controlled by either the Primary Guidance and Navigation
; Control System (PGNCS - the AGC) or the Abort Guidance System (AGS). This
; check ensures PGNCS has control before activating the DAP.

CHEKMORE	CAF	BIT10		# BIT 10 OF 30 IS PGNCS CONTROL OF S/C
		EXTEND
		RAND	CHAN30		# BITS IN 30 ARE INVERTED
		CCS	A
		TCF	MOREIDLE

		RETURN

# Page 1411
; ============================================================================
; DAPIDLER - Main DAP Idler Program Entry Point
;
; This routine is invoked by the executive scheduler 10 times per second
; (every 100 milliseconds) to perform background autopilot housekeeping.
; It checks system readiness, initializes the DAP when appropriate, and
; maintains the attitude error display on the FDAI.
; ============================================================================

# DAPIDLER ENTRY.

DAPIDLER	LXCH	BANKRUPT	# INTERRUPT LEAD INS (CONTINUED)
		EXTEND
		QXCH	QRUPT

; The DAP requires the 1/ACCS (once-per-acceleration) job to be set up
; before it can begin controlling the spacecraft. This job reads accelerometer
; data from the IMU and updates the spacecraft velocity estimate. This section
; ensures the 1/ACCJOB initialization happens exactly once after each restart.

		CA	RCSFLAGS
		MASK	BIT13
		CCS	A		# CHECK IF 1/ACCJOB HAS BEEN SET UP SINCE
		TCF	CHECKUP		# THE LAST FRESH START OR RESTART.
		CA	BIT13
		ADS	RCSFLAGS	# BIT 13 IS 1.
		CAF	PRIO27
		TC	NOVAC		# SET UP JOB TO DO A LITTLE INITIALIZATION
		EBANK=	AOSQ		#   AND EXECUTE 1/ACCS.
		2CADR	1/ACCSET	# (WILL BRANCH TO MOREIDLE ON ACCSOKAY)

CHECKUP		TC	CHEKBITS	# CHECK TO SEE IF LM DAP IS TO GO ON AND
					#   DO ERROR DISPLAY.

		CAE	DAPBOOLS	# IF 1/ACCS HAS NOT BEEN COMPLETED, IDLE.
		MASK	ACCSOKAY	#   NOTE: ONLY FRESH START AND RESTART
		EXTEND			# 	  KNOCK THIS BIT DOWN.
		BZF	MOREIDLE

; ============================================================================
; STARTDAP - DAP Initialization Sequence
;
; When all readiness conditions are met (mode switch on, IMU operational,
; PGNCS in control, 1/ACCS running), this section initializes the DAP.
; It zeros out attitude errors, desired rotation rates, and time-to-jet-fire
; counters, preparing the autopilot for active spacecraft control.
; ============================================================================

STARTDAP	TC	IBNKCALL	# ZERO ATTITUDE ERROR AND DESIRED RATES.
		FCADR	ZATTEROR
		CAF	ZERO		# ********** INITIALIZE: **********
		TS	TJP
		TS	TJU
		TS	TJV
		TS	OMEGAP		# RATES IN BODY (PILOT) COORDINATES.
		TS	OMEGAQ
		TS	OMEGAR
		TS	TRAPEDP
		TS	TRAPEDQ
		TS	TRAPEDR
; Zero trapezoidal integration terms for P, Q, R axes, resetting DAP's
; velocity accumulation from previous flight operations.
		TS	AOSQ		# OFFSET ACCELERATION ESTIMATES.
		TS	AOSQ +1
		TS	AOSR
		TS	AOSR +1
; AOS (Acceleration Offset Estimate) terms account for external forces
; like venting or docking probe contact. Starting from zero baseline.
		TS	ALPHAQ		# COPIES OF OFFSET ESTIMATES FOR DOWNLIST.
		TS	ALPHAR
		TS	NEGUQ
		TS	NEGUR
; Alpha and Negu terms support Kalman filtering of rate estimates,
; improving attitude control accuracy during maneuvers and engine burns.
		TS	AOSQTERM	# QRAXIS RATE DERIVATION TERMS AND KALMAN
		TS	AOSRTERM	# FILTER INITIALIZATION TERMS.
		TS	QACCDOT		# DESCENT ACCELERATION DERIVATIVE EST.
		TS	RACCDOT
# Page 1412
; Descent engine gimbal control and RCS jet management initialization:
		TS	ALLOWGTS	# AOSTASK FLAG FOR QRAXIS RCS CONTROL USE.
		TS	COTROLER	# DO TRYGTS ON FIRST PASS (WILL GO TO RCS)
		TS	INGTS		# RECOGNIZE FIRST GTS PASS AS SUCH.
; The Gimbal Trim System (GTS) uses descent engine gimbaling for attitude
; control during powered descent. These flags ensure proper first-pass logic.
		TS	QGIMTIMR	# STOP GIMBAL DRIVES.  (PROBABLY WOULD BE
		TS	RGIMTIMR	#   GOOD ENOUGH JUST TO INACTIVATE TIMERS)
; Gimbal drive timers zeroed to halt any residual gimbal motion from
; previous operations. Critical for safe engine ignition configuration.
		TS	OLDPMIN		# MINIMUM IMPULSE MODE ERASABLES
		TS	OLDQRMIN
; Minimum impulse mode allows finer attitude control using brief RCS pulses
; rather than continuous firing. Old values cleared for fresh operation.
		TS	PJETCTR		# INITIALIZE DOCKED JET INHIBITION
		TS	UJETCTR		# COUNTERS
		TS	VJETCTR
; Jet inhibition counters prevent thruster firing when docked or when
; structural loads would be excessive. Initialized to zero for normal ops.
CALLGMBL	EQUALS	BIT5		# RCSFLAGS INITIALIZATION.
		CS	MANFLAG
		MASK	RCSFLAGS	# NEGUQ(R) HAVE BEEN GENERATED.
		TS	RCSFLAGS
; Clear manual control flags from RCSFLAGS. DAP begins in automatic mode,
; ready to maintain commanded attitude without astronaut stick inputs.

# SET UP "OLD" MEASURED CDU ANGLES:

		EXTEND
		DCA	CDUX		# OLDXFORP AND OLDYFORP
		DXCH	OLDXFORP
		CA	CDUZ
		TS	OLDZFORQ
; CDU (Coupling Data Unit) angles from IMU gimbals are stored as reference
; baseline. On subsequent passes, these "old" values enable rate calculation
; by measuring angular change over time (delta-theta / delta-time = omega).
		CS	RCSFLAGS
		MASK	BIT12
		ADS	RCSFLAGS	# BIT 12 SET TO 1.
; Bit 12 indicates DAP initialization complete. Other routines check this
; flag before trusting DAP state variables.
		CA	FOUR
		TS	SKIPU
		TS	SKIPV
; Skip counters delay execution of U and V axis control laws on first passes,
; allowing system state to stabilize before full three-axis control engages.
		CA	POSMAX
		TS	TIME6
		TS	T6NEXT
		TS	T6FURTHA
		CA	ZERO
		TS	T6NEXT +1
		TS	T6FURTHA +1
		TS	NXT6ADR
; T6 timer infrastructure manages RCS jet firing sequences. Setting to POSMAX
; (maximum positive value) effectively disables scheduled firings until DAP
; calculates new jet commands. Critical for preventing spurious thruster fires.
		TS	NEXTP
		TS	NEXTU
		TS	NEXTV
; Next jet fire time registers zeroed for P, U, V axes. DAP will compute
; appropriate firing times based on attitude errors and rate damping needs.
		CS	TEN
		TS	DAPZRUPT	# JASK NOT IN PROGRESS, INITIALIZE NEG.
; DAPZRUPT counter tracks JASK (jet activity scheduling) status. Negative
; value indicates no RCS firing sequence currently active.
		CA	TWO
		TS	NPTRAPS
		TS	NQTRAPS
		TS	NRTRAPS
; Trapezoidal integration pass counters set to 2. These track integration
; cycles for each axis, ensuring numerical stability in rate calculations.
		EXTEND
		DCA	PAXADIDL
		DXCH	T5ADR
; T5ADR points to next routine for TIME5 interrupt (100ms cadence). Set to
; PAXIS routine which handles P-axis (roll) autopilot calculations. After
; STARTDAP completes, normal DAP cycle begins with P-axis processing.
SETTIME5	CAF	MS100
		TS	TIME5
; Schedule next DAPIDLER execution in 100 milliseconds. This establishes
; the 10 Hz heartbeat for all DAP background operations.
# Page 1413
		TCF	RESUME
; Return from interrupt, restoring processor state and continuing mission
; program execution. DAP is now fully initialized and running.
		EBANK=	AOSQ
IDLERADR	2CADR	DAPIDLER

; ============================================================================
; TRANSITION: From DAP Initialization to Idle Mode Operations
;
; When DAP cannot yet take full control (IMU not ready, mode switch in wrong
; position, or ACCS not completed), DAPIDLER branches to MOREIDLE. This
; section calculates attitude errors for crew display even though autopilot
; is not actively controlling the spacecraft. Astronauts can monitor their
; orientation relative to desired attitude while manually maneuvering.
;
; If conditions require DAP to be completely off (Mode Select Switch in OFF
; position), SHUTDOWN ensures all jet commands are cleared and gimbal drives
; are halted, leaving the spacecraft in a safe quiescent state.
; ============================================================================

MOREIDLE	TC	IBNKCALL	# CALCULATE Q,R-AXES ATTITUDE ERRORS.
		CADR	QERRCALC
; Compute attitude errors for Q and R axes (pitch and yaw in pilot
; coordinates). Even when DAP is idle, crew needs error display on FDAI
; (Flight Director Attitude Indicator) for manual control reference.
		TC	IBNKCALL
		CADR	CALCPERR	# CALCULATE P AXIS ATTITUDE ERRORS.
; Compute attitude error for P axis (roll in pilot coordinates). With all
; three axes computed, DAPIDLER will drive FDAI needles showing crew their
; orientation relative to commanded attitude.

SHUTDOWN	EXTEND
		DCA	IDLERADR
		DXCH	T5ADR
; Point T5ADR back to DAPIDLER itself. In 100ms, system will re-check
; whether conditions have changed to allow DAP activation.
		CAF	ZERO		# KILL ANY POSSIBLE JET REQUESTS
		TS	NEXTP
		TS	NEXTU
		TS	NEXTV
; Clear all scheduled RCS jet firing times. Absolutely critical to prevent
; uncommanded thruster fires when DAP is supposed to be inactive.
		EXTEND			# COMMAND JETS OFF.
		WRITE	CHAN5
		EXTEND
		WRITE	CHAN6
; Write zeros to output channels 5 and 6, which control RCS jet solenoid
; valves. Hardware-level assurance that no thrusters can fire.
		CS	BGIM23		# TURN TRIM GIMBAL OFF
		EXTEND
		WAND	CHAN12
; Clear bits in channel 12 to disable descent engine gimbal trim drives.
; Prevents any uncommanded engine movement during shutdown state.
		TCF	SETTIME5	# RETURN IN 100 MSEC.
; Schedule next DAPIDLER pass in 100ms. System will repeatedly check for
; DAP enable conditions, ready to activate when crew selects proper mode.

MANFLAG		OCT	03021
BGIM23		OCTAL	07400
		EBANK=	OMEGAP
PAXADIDL	2CADR	PAXIS

MS100		=	OCT37766
COSMG		=	ITEMP1
JUMPDSP		EXTEND			# TRANSFER TO BANK 20
		DCA	DSPCADR		# FOR ATTITUDE ERROR DISPLAYS
		DTCB

		EBANK=	AK
DSPCADR		2CADR	ALTDSPLY

# Page 1414
		BANK	20
		SETLOC	DAPS3
		BANK
		COUNT*	$$/NEEDL

# PROGRAM: ALTDSPLY
#
; ============================================================================
; TRANSITION: From Idle DAP Logic to Attitude Display Alternation
;
; The ALTDSPLY program (Bank 20) manages the FDAI (Flight Director Attitude
; Indicator) needle displays visible to the crew. It implements a display
; alternation strategy that switches between two modes every 100ms: attitude
; errors (spacecraft orientation relative to commanded attitude) when the bit
; is zero, and rate errors (rotational velocities) when the bit is one.
;
; This alternation provides complete situational awareness with limited display
; hardware. During Apollo 11's lunar descent on July 20, 1969, Armstrong and
; Aldrin relied on these alternating FDAI displays to monitor spacecraft
; orientation and rotation rates during the final approach to Tranquility Base.
; ============================================================================
#
# MOD 0.  6 DEC 1967
#
# AUTHOR:  CRAIG WORK, DON KEENE, MIT IL
#
# MOD 3 BY DON KEENE AUG 1, 1968 MOVED PROGRAM TO BANK 20
#
# PROGRAM DESCRIPTION:
#
# ALTDSPLY REVERSES THE DSPLYALT BIT OF RCSFLAGS EACH TIME IT IS CALLED, WHICH IS PRESUMABLY EVERY 100 MS.
# IF THE REVERSED BIT IS ONE, NEEDLER IS CALLED TO DISPLAY ATTITUDE ERRORS.  IF THE BIT IS ZERO, THE ATTITUDE ERR-
# ORS ARE CALCULATED AS 1) DAP FOLLOWING ERRORS, IF NEEDLFLG = 0, AND 2) TOTAL ATTITUDE ERRORS FOR NEEDLFLG = 1.
#
# WARNING:  ALTDSPLY MAY ONLY BE CALLED WITH INTERRUPT INHIBITED.
#
# WARNING:  EBANK MUST BE SET TO 6 WHEN USING THIS ROUTINE.
#
# INPUT:  RCSFLAGS AND 1) IF NEEDLFLG=0, INPUT PERROR,QERROR,RERROR.
# 		       2) IF NEEDLFLG=1, INPUT CPHI,CTHETA,CPSI,CDUX,CDUY,CDUZ,M11,M21,M32,M22,M32. (GPMATRIX)
#
# OUTPUTS:  RCSFLAGS WITH DSPLYALT REVERSED,AK,AK1,AK2,+ NEEDLER OUTPUTS.
#
# ENTRY:   TCF    ALTDSPLY
#
# EXIT:    TCF    CHEKMORE
#
# ALARM OR ABORT EXITS:  NONE
#
# SUBPROGRAMS CALLED: NEEDLER, OVERSUB2
#
# DEBRIS:  A,L,AND NEEDLER DEBRIS.

ALTDSPLY	CA	RCSFLAGS	# INVERT THE DISPLAY ALTERNATION BIT.
		TS	L
		CA	DSPLYALT
		EXTEND
		RXOR	LCHAN
		TS	RCSFLAGS
; Toggle bit 4 (DSPLYALT) of RCSFLAGS using exclusive-OR operation. This
; elegant bit-flip ensures automatic alternation every 100ms: attitude display
; this pass, rate display next pass, attitude again, and so on.

		MASK	DSPLYALT
		CCS	A		# IS ALTERNATION FLAG ZERO?
		TCF	NEEDLER
; If DSPLYALT bit is now 1 (after toggle), we're in rate display mode.
; Skip error calculation and jump directly to NEEDLER to display the rate
; errors that were computed earlier (stored in AK, AK+1, AK+2).

		CAE	FLAGWRD0	# NEEDLFLG WILL INDICATE TOTAL OR DAP AT-
# Page 1415
		MASK	NEEDLBIT	# TITUDE ERROR DISPLAY REQUEST.
		CCS	A
		TCF	DSPLYTOT	# TOTAL ERROR IS NEEDED IN AK,AK +1,AK +2
; If DSPLYALT=0, we need to calculate attitude errors. Check NEEDLFLG to
; determine error type: if set, display total attitude errors (spacecraft
; orientation vs commanded); if clear, display DAP following errors (autopilot
; tracking performance). NEEDLFLG typically set when DAP is active.

		CS	QERROR		# YES. DISPLAY ATT ERRORS ON THE ,-BALL.
		TS	AK +1		# ERROR COMPLEMENTS ARE INPUT TO NEEDLER.
		CS	RERROR
		TS	AK +2
		CS	PERROR
		XCH	AK
; NEEDLFLG=0: Load DAP following errors (PERROR, QERROR, RERROR) into
; AK/AK+1/AK+2 for display. Errors are complemented (negated) because NEEDLER
; expects error complements for correct needle deflection direction on FDAI.

		TCF	RETNMORE	# DISPLAY THESE THE NEXT TIME THROUGH
; Skip DSPLYTOT calculation and proceed to RETNMORE, which calls NEEDLER to
; drive the FDAI needles with these DAP following errors.

# CALCULATE GIMBAL ANGLE TOTAL ERRORS, RESOLVE INTO PILOT AXES, STORE TOTAL ERRORS FOR NEEDLER.  Q-AXIS FIRST.

; ============================================================================
; DSPLYTOT Subroutine: Total Attitude Error Calculation
;
; Computes total attitude errors by comparing commanded attitude (CPHI, CTHETA,
; CPSI) with actual CDU gimbal angles (CDUX, CDUY, CDUZ), then resolves these
; gimbal-frame errors into pilot (body) axes using the GPMATRIX transformation.
;
; The GPMATRIX (M11, M21, M22, M31, M32, M33) relates navigation platform gimbal
; coordinates to pilot body axes. This transformation is essential because crew
; sees errors relative to spacecraft body axes (where they're sitting), not
; relative to gimbal geometry. Result: FDAI needles show errors in intuitive
; pilot coordinates that match physical spacecraft orientation.
; ============================================================================

DSPLYTOT	EXTEND
		QXCH	ITEMP1		# SAVE Q FOR CHEKBITS RETURN.
; Save return address from Q register to ITEMP1. DSPLYTOT performs complex
; calculations and needs to preserve Q for proper return path.

		CA	CTHETA		# DESIRED ATTITUDE, Y-AXIS, 2'S COMP.
		EXTEND			# SUBTRACT CURRENT ATTITUDE.
		MSU	CDUY		# DIFFERENCE SCALED AT PI, 1'S COMP.
		TS	AK		# SAVE FOR R-ERROR CALCULATION.
; Compute Y-gimbal error: commanded CTHETA minus actual CDUY. This is the
; middle gimbal error in 1's complement format scaled at π radians (180°).
; Error stored temporarily in AK for use in both Q and R axis calculations.
		EXTEND
		MP	M21		# (CHTETA-CDUY)*M21 SCALED AT PI RADIANS.
		XCH	AK +1		# STORE FIRST TERM OF Q ERROR.
; Multiply Y-gimbal error by M21 matrix element. This transforms the middle
; gimbal rotation into its contribution to pilot Q-axis (pitch) error.
		CA	CPSI		# DESIRED ATTITUDE,Z-AXIS, 2'S COMP.
		EXTEND			# SUBTRACT CURRENT ATTITUDE.
		MSU	CDUZ		# DIFFERENCE SCALED AT PI, 1'S COMP.
		TS	AK +2		# SAVE Z-AXIS TERM FOR R ERROR CALCULATION
; Compute Z-gimbal error: commanded CPSI minus actual CDUZ. This is the outer
; gimbal error, also scaled at π radians in 1's complement format.
		EXTEND
		MP	M22		# (CPSI-CDUZ)*M22, SCALED AT PI RADIANS.
		AD	AK +1		# Q ERROR COMPLETE	   , AT PI RAD.
; Multiply Z-gimbal error by M22 matrix element and add to previous term.
; Q-axis error now complete: includes contributions from both Y and Z gimbal
; rotations, properly transformed into pilot pitch axis. During Apollo 11
; descent, this Q-axis error showed Armstrong his pitch angle relative to
; desired attitude for landing approach visibility.
		TC	OVERSUB2	# PIN NEEDLES IN CASE OF OVERFLOW
		TS	AK +1
; Limit Q-axis error to prevent FDAI needle from going off-scale. OVERSUB2
; clamps values to maximum displayable range. Critical for crew trust: needles
; must never "peg" beyond their physical stops.

# R ERROR CALCULATION NEXT.

		CA	AK		# Y-AXIS DIFFERENCE STORED BY Q-AXIS CALC.
		EXTEND
		MP	M31		# (CTHETA-CDUY)*M31, SCALED AT PI RADIANS.
		XCH	AK +2		# FIRST TERM OF R ERROR.
; Compute R-axis (yaw) error first term: Y-gimbal error (saved earlier in AK)
; multiplied by M31. Transforms middle gimbal rotation into its yaw contribution.
					# Z-AXIS DIFFERENCE, STORED BY A CALC. IS
		EXTEND			# RECOVERED BY THE EXCHANGE.
		MP	M32		# (CPSI-CDUZ)*M32, SCALED AT PI RADIANS.
		AD	AK +2		# R ERROR COMPLETE	   , AT PI RAD.
; Second term: Z-gimbal error (recovered from AK+2 by exchange) multiplied by
; M32, then added to first term. R-axis error now complete, incorporating both
; Y and Z gimbal contributions transformed into pilot yaw axis.
		TC	OVERSUB2	# PIN NEEDLES IN CASE OF OVERFLOW.
		TS	AK +2
; Limit R-axis error to displayable range. All three axes (P, Q, R) use same
; overflow protection to ensure FDAI needles remain within physical limits.

# NOW CALCULATE P ERROR.  (NOTE THAT M13 = 1, SCALED AT 1, SO THE MULTIPLICATION IS BY-PASSED.)
# Page 1416
		CA	AK		# Y-AXIS DIFFERENCE STORED BY Q AXIS CALC.
		EXTEND
		MP	M11		# (CTHETA-CDUY)*M11 SCALED AT PI RADIANS.
		XCH	AK		# FIRST TERM OF P ERROR IN AK, AT PI RAD.
; Compute P-axis (roll) error first term: Y-gimbal error multiplied by M11.
; Transforms middle gimbal rotation into its roll contribution.
		CAE	CPHI		# DESIRED ATTITUDE, X-AXIS, 2'S COMP.
		EXTEND			# SUBTRACT CURRENT X ATTITUDE.
		MSU	CDUX		# X-AXIS DIFFERENCE, 1'S COMP, AT PI RAD.
; Compute X-gimbal error: commanded CPHI minus actual CDUX. This is the inner
; gimbal error, which directly represents roll orientation.

# M13 = 1, SO BYPASS THE MULTIPLICATION.
#		EXTEND
#		MP	M13		  (CPHI-CDUX)*M13 SCALED AT PI RADIANS.
; M13 matrix element equals 1.0 (scaled at unity), so multiplication is
; unnecessary. This is a mathematical property of the gimbal transformation:
; inner gimbal rotation maps directly to pilot roll axis with unit coefficient.

		AD	AK		# P ERROR COMPLETE	, SCALED AT PI RAD
		TC	OVERSUB2	# PIN NEEDLES IN CASE OF OVERFLOW.
		TS	AK
; P-axis error complete: sum of Y-gimbal contribution (M11 term) and X-gimbal
; error (M13=1.0 term). Clamped to displayable range to protect FDAI needles.
; All three pilot-axis errors (P, Q, R) now computed and stored in AK, AK+1,
; AK+2, ready for NEEDLER to drive the error display on the 8-ball.

		EXTEND
		QXCH	ITEMP1		# RESTORE Q FOR CHEKBITS RETURN.
; Restore return address from ITEMP1 back to Q register, preparing for proper
; return from DSPLYTOT subroutine.

		TCF	RETNMORE	# DISPLAY THESE THE NEXT TIME THROUGH
; Jump to RETNMORE which calls NEEDLER to update FDAI needles with the total
; attitude errors just computed. Next DAPIDLER pass (100ms later) will alternate
; to rate display mode, giving crew both attitude and rate error information.

# Page 1417
# FDAI ATTITUDE ERROR DISPLAY SUBROUTINE
#
# PROGRAM DESCRIPTION:    D. KEENE   5/24/67
#
# MOD 1 BY CRAIG WORK, 12 DEC 67
#
# MOD 2 BY CRAIG WORK, 6 APRIL 68 CONVERTS ATTITUDE ERROR DISPLAY SCALING FROM 16 7/8 DEG. TO 42 3/16 DEGREES.
#
#     THIS SUBROUTINE IS USED TO DISPLAY ATTITUDE ERRORS ON THE FDAI VIA THE DIGITAL TO ANALOG CONVERTERS (DACS)
# IN THE CDUS.  CARE IS TAKEN TO METER OUT THE APPROPRIATE NUMBER OF PULSES TO THE IMU ERROR COUNTERS AND PREVENT
# OVERFLOW, TO CONTROL THE RELAY SEQUENCING, AND TO AVOID INTERFERENCE WITH THE COARSE ALIGN LOOP WHICH ALSO USES
# THE DACS.
#
#
# CALLING SEQUENCE:
#
#     DURING THE INITIALIZATION SECTION OF THE USER'S PROGRAM, BIT3 OF RCSFLAGS SHOULD BE SET TO INITIATE THE
# TURN-ON SEQUENCE WITHIN THE NEEDLES PROGRAM:
#
#	   CS	  RCSFLAGS	  IN EBANK6
#	   MASK	  BIT3
#	   ADS	  RCSFLAGS
#
# THEREAFTER, THE ATTITUDE ERRORS GENERATED BY THE USER SHOULD BE TRANSFERRED TO THE FOLLOWING LOCATIONS IN EBANK6:
#
#	   AK	  SCALED 180 DEGREES  NOTE: THESE LOCATIONS ARE SUBJECT
#	   AK1	  SCALED 180 DEGREES	    TO CHANGE
#	   AK2	  SCALED 180 DEGREES
#
# FULL SCALED DEFLECTION OF THE NEEDLES CORRESPONDS TO 5 1/16 DEGREES, WHILE 384 BITS IN THE IMU ERROR COUNTER
# CORRESPONDS TO 42 3/16 DEGREES.  (DAC MAXIMUM CAPACITY IS 384 BITS.) 46 BITS EFFECTIVELY PIN THE NEEDLES.
#
# A CALL TO NEEDLER WILL THEN UPDATE THE DISPLAY:
#
#	   INHINT
#	   TC	  IBNKCALL	 NOTE: EBANK SHOULD BE SET TO E6
#	   CADR	  NEEDLER
#	   RELINT
#
#     THIS PROCESS SHOULD BE REPEATED EACH TIME THE ERRORS ARE UPDATED.  AT LEAST 3 PASSES THRU THE PROGRAM ARE
# REQUIRED BEFORE ANYTHING IS ACTUALLY DISPLAYED ON THE ERROR METERS.
# NOTE: EACH CALL TO NEEDLER MUST BE SEPARATED BY AT LEAST 50MS TO ASSURE PROPER RELAY SEQUENCING.
#
# ERASABLES USED:
#		       AK	  CDUXCMD
#		       AK1	  CDUYCMD
#		       AK2	  CDUZCMD
#		       EDRIVEX	  A,L,Q
#		       EDRIVEY	  T5TEMP
#		       EDRIVEZ	  DINDX
# Page 1418
#
# SWITCHES:	       RCSFLAGS	  BITS 3,2
#
# I/O CHANNELS:	       CHAN12	  BIT 4 (COARSE ALIGN - READ ONLY)
#		       CHAN12	  BIT 6 (IMU ERROR COUNTER ENABLE)
#		       CHAN14	  BIT 13,14,15 (DAC ACTIVITY)
#
#
# SIGN CONVENTION<   AK = THETAC - THETA
#	    WHERE    THETAC = COMMAND ANGLE
#		     THETA  = PRESENT ANGLE

; ============================================================================
; NEEDLER Subroutine: FDAI Error Needle Driver with 3-Pass Initialization
;
; Controls the Flight Director Attitude Indicator (FDAI) error needles that
; Armstrong and Aldrin monitored during Apollo 11 descent. Converts attitude
; or rate errors into DAC (Digital-to-Analog Converter) commands that
; physically deflect the three error needles overlaying the 8-ball display.
;
; THREE-PASS INITIALIZATION SEQUENCE (Required for IMU Error Counter):
;
;   Pass 1 (RCSFLAGS bits 3,2 = 1,0): First initialization
;     - Disable IMU error counter (CHAN12 bit 6 = 0)
;     - Zero all error inputs (AK, AK1, AK2)
;     - Zero all DAC output registers (EDRIVEX/Y/Z, CDUXCMD/Y/Z)
;     - Set RCSFLAGS to 0,1 for Pass 2
;     - WAIT 60ms minimum before Pass 2
;
;   Pass 2 (RCSFLAGS bits 3,2 = 0,1): Second initialization
;     - Enable IMU error counter (CHAN12 bit 6 = 1)
;     - Clear RCSFLAGS bits 3,2 to 0,0 for normal operation
;     - WAIT 4ms minimum for relay closure
;
;   Pass 3+ (RCSFLAGS bits 3,2 = 0,0): Normal operation
;     - Verify IMU error counter enabled
;     - Convert errors (AK, AK+1, AK+2) scaled at π radians to DAC commands
;     - Output scaled ±1800° (±16000 DAC units, limited to ±384 on overflow)
;     - Set CHAN14 DAC activity bits to trigger output
;
; This multi-pass sequence ensures proper IMU error counter state before
; driving FDAI needles. Critical during powered descent when Armstrong needed
; reliable attitude reference to assess autopilot performance.
; ============================================================================

NEEDLER		CA	RCSFLAGS
		MASK	SIX
		EXTEND
		BZF	NEEDLES3
; Check RCSFLAGS bits 2 and 3 (mask SIX = bits 2&3). Branch to NEEDLES3 if
; both bits are zero (normal operation mode after initialization complete).
; If either bit is set, we're in initialization sequence.

		MASK	BIT3
		EXTEND
		BZF	NEEDLER2	# BIT3 = 0, BIT2 = 1
; Within initialization: check BIT3 specifically. If BIT3=0 but BIT2=1,
; branch to NEEDLER2 (Pass 2: enable IMU error counter). If BIT3=1,
; continue to Pass 1 (first initialization with counter disable).

; ============================================================================
; PASS 1: First Initialization - Disable IMU Error Counter and Zero All DACs
;
; RCSFLAGS bits 3,2 = 1,0 entering this section (BIT3=1 detected above).
; Disable IMU error counter to safely zero all DAC registers without
; inadvertent needle motion. Must wait minimum 60ms after disabling counter
; before Pass 2 re-enables it (relay settling time requirement).
; ============================================================================
		CS	BIT6		# FIRST PASS BIT3 = 1
		EXTEND			# DISABLE IMU ERROR COUNTER TO ZERO DACS
		WAND	CHAN12		# MUST WAIT AT LEAST 60 MS BEFORE
; Clear bit 6 of CHAN12 (write AND with complement) to disable IMU error
; counter. This prevents spurious CDU pulses during DAC zeroing.

NEEDLE11	CS	ZERO		# ENABLING COUNTERS.
		TS	AK		# ZERO THE INPUTS ON FIRST PASS
		TS	AK1
		TS	AK2
; Zero all three error input registers (AK, AK1, AK2). These normally hold
; attitude or rate errors scaled at π radians that drive FDAI needles.

		TS	EDRIVEX		# ZERO THE DISPLAY REGISTERS
		TS	EDRIVEY
		TS	EDRIVEZ
; Zero FDAI error drive registers for all three axes (pitch, yaw, roll).

		TS	CDUXCMD		# ZERO THE OUT COUNTERS
		TS	CDUYCMD
		TS	CDUZCMD
; Zero CDU command output registers that interface with IMU error counters.

		CS	SIX		# RESET RCSFLAGS FOR PASS2
		MASK	RCSFLAGS
		AD	BIT2
		TS	RCSFLAGS
; Clear bits 2 and 3 of RCSFLAGS (mask with complement of SIX), then set
; only bit 2 (add BIT2). Result: bits 3,2 = 0,1 for Pass 2 entry next time.

		TCF	RETNMORE
; Return from this initialization pass. DAPIDLER will call NEEDLER again
; after minimum 60ms wait for relay settling.

; ============================================================================
; PASS 2: Second Initialization - Enable IMU Error Counter
;
; RCSFLAGS bits 3,2 = 0,1 entering this section (detected by BZF NEEDLER2).
; At least 60ms has elapsed since Pass 1 disabled the counter. Now safe to
; re-enable IMU error counter for normal operation. Clear initialization
; flags to enter normal display mode. Must wait minimum 4ms for relay closure
; before Pass 3 begins normal DAC output operation.
; ============================================================================
NEEDLER2	CAF	BIT6		# ENABLE IMU ERROR COUNTERS
		EXTEND
		WOR	CHAN12
; Set bit 6 of CHAN12 (write OR) to enable IMU error counter. Counter can
; now safely accept CDU command pulses to drive FDAI needles.

		CS	SIX		# RESET RCSFLAGS TO DISPLAY ATTITUDE
		MASK	RCSFLAGS	# ERRORS    WAIT ATLEAST 4 MS FOR
		TS	RCSFLAGS	# RELAY CLOSURE
; Clear bits 2 and 3 of RCSFLAGS (mask with complement of SIX). Result:
; bits 3,2 = 0,0 for normal operation. Next NEEDLER call enters Pass 3.

		TCF	RETNMORE
; Return from Pass 2. After 4ms minimum wait for relay closure, subsequent
; NEEDLER calls will execute normal DAC output operation (NEEDLES3).

; ============================================================================
; PASS 3+: Normal Operation - Verify IMU Counter Enabled, Then Drive DACs
;
; RCSFLAGS bits 3,2 = 0,0 (initialization complete, detected by BZF NEEDLES3).
; Verify IMU error counter is enabled before processing error needle outputs.
; If counter unexpectedly disabled (hardware fault or crew switch action),
; re-enter initialization sequence from Pass 1.
; ============================================================================
NEEDLES3	CAF	BIT6		# CHECK TO SEE IF IMU ERROR COUNTER
		EXTEND			# IS ENABLED
		RAND	CHAN12
; Read CHAN12 and test bit 6 (IMU error counter enable). Should be 1 after
; Pass 2 initialization. If zero, hardware fault or crew action disabled it.

# Page 1419
		CCS	A		# IF NOT, RE-INITIALIZE NEEDLER.
		TCF	NEEDLES
; If IMU counter enabled (A > 0), branch to NEEDLES for normal DAC output.
; If disabled (A = 0), fall through to re-initialization setup.

		CS	RCSFLAGS	# SET UP INITIALIZATION FLAG IN RCSFLAGS.
		MASK	BIT3
		ADS	RCSFLAGS
; Set bit 3 of RCSFLAGS (bits 3,2 become 1,0) to trigger Pass 1 re-init
; on next NEEDLER call. This handles unexpected loss of IMU counter enable.

		TCF	RETNMORE
; Return without DAC output. Next NEEDLER call re-executes 3-pass sequence.

; ============================================================================
; NEEDLES Main Loop: Convert Error Values to DAC Commands for Three Axes
;
; Processes three attitude/rate error values (AK, AK+1, AK+2) stored at
; consecutive memory locations. Each error scaled at π radians = 180°.
; Converts to DAC units: ±1800° range = ±16000 DAC units (with ±384 limit
; on overflow). Computes incremental change from previous output (EDRIVEX/Y/Z)
; and adds to CDU command registers (CDUXCMD/Y/Z) that drive FDAI needles.
;
; Loop executes three times (DINDX = 2, 1, 0) for roll, pitch, yaw axes.
; During Apollo 11 descent, Armstrong monitored these needles continuously
; to verify autopilot performance and assess need for manual override.
; ============================================================================
NEEDLES		CAF	TWO
; Initialize loop counter to 2 (will process indices 2, 1, 0 for three axes).

DACLOOP		TS	DINDX
; Store current loop index (2=roll/Z, 1=pitch/Y, 0=yaw/X axis).

		CS	ONETENTH	# RESCALE INPUTS TO + OR - 1800 DEGREES.
		EXTEND
		INDEX	DINDX
		MP	AK
; Multiply error value (indexed: AK+DINDX) by complement of ONETENTH constant.
; Scales π radians input to ±1800° output range (10× scaling factor).
; Result in A register (upper word) and L register (lower word).

		TS	L
; Save rescaled error value to L register for overflow checking.

		CCS	A
		CA	DACLIMIT
		TCF	+2
		CS	DACLIMIT
; Check sign of rescaled value. If positive, load positive DACLIMIT.
; If negative, skip 2 instructions and load negative DACLIMIT (CS).
; DACLIMIT = 384 (maximum DAC output to prevent needle over-travel).

		AD	L
		TS	T5TEMP		# OVFLO CHK
		TCF	+4
; Add limit to rescaled value and store to T5TEMP. If overflow occurs
; (value exceeds ±16000 representable range), trap and apply limit.

		INDEX	A		# ON OVERFLOW LIMIT OUTPUT TO +-384
		CAF	DACLIMIT
		TS	L
; On overflow: use A register (sign indicator) to index DACLIMIT table,
; loading either +384 or -384 limit value to L register.

		INDEX	DINDX
		CS	EDRIVEX		# CURRENT VALUE OF DAC
		AD	L
; Load complement of previous DAC output (indexed: EDRIVEX+DINDX), add new
; scaled error value. Computes incremental change needed for this cycle.

		INDEX	DINDX
		ADS	CDUXCMD
; Add incremental change to CDU command register (indexed: CDUXCMD+DINDX).
; This register drives IMU error counter to deflect FDAI error needle.

		INDEX	DINDX
		LXCH	EDRIVEX
; Store new error value to display register (indexed: EDRIVEX+DINDX) using
; exchange. This becomes "previous value" for next NEEDLES call.

		CCS	DINDX
		TCF	DACLOOP
; Decrement loop index. If still positive (2→1→0), repeat for next axis.
; When index reaches zero and decrements to negative, fall through to finish.

		CAF	13,14,15
		EXTEND
		WOR	CHAN14		# SET DAC ACTIVITY BITS
; Set bits 13, 14, 15 of CHAN14 to signal DAC hardware that new output
; commands are ready. This triggers physical needle motion on FDAI display.

		TCF	RETNMORE
; Return to DAPIDLER main loop. FDAI needles now reflect current attitude
; or rate errors for crew monitoring.

; ============================================================================
; Constants and Tables for NEEDLER Subroutine
; ============================================================================

		DEC	-384
DACLIMIT	DEC	16000
		DEC	384
; DACLIMIT table indexed by overflow sign for limiting DAC output.
; -384 (negative limit), 16000 (normal scale), +384 (positive limit).
; ±384 prevents FDAI needle over-travel; ±16000 is full ±1800° scale.

ONETENTH	OCT	03146		# DECIMAL +0.1, SCALED AT 1.
; Constant 0.1 in octal scaled at 1. Used for error value rescaling.
; Complement (CS ONETENTH) multiplies by 10 for degree conversion.

DSPLYALT	EQUALS	BIT4		# 100 MS ALTERNATION BIT IN RCSFLAGS
; Bit 4 of RCSFLAGS used as 100ms alternation timer flag for displays.

; ============================================================================
; OVERSUB2: Overflow Limiting Utility Subroutine
;
; Checks A register for overflow. Returns A unchanged if no overflow,
; or limits to POSMAX/NEGMAX if overflow detected. Duplicate coding exists
; in bank 16 for cross-bank compatibility.
; ============================================================================
OVERSUB2	TS	7		# RETURNS  A  UNCHANGED  OR LIMITED TO
		TC	Q		# POSMAX OR NEGMAX IF A HAS OVERFLOW
; Store A to location 7, triggering overflow trap if overflow. If no
; overflow, return immediately with A unchanged via Q (return address).

		INDEX	A
# Page 1420
		CS	LIMITS		# DUPLICATE CODING IN  BANK 16
		TC	Q
; On overflow: A contains sign indicator. Index LIMITS table to load
; complement of appropriate limit (POSMAX or NEGMAX), then return.

; ============================================================================
; RETNMORE: Return to CHEKMORE Subroutine
;
; Standard return sequence from DAPIDLER processing back to CHEKMORE
; section of main loop. Uses DTCB (Double Transfer Control Bank) for
; cross-bank return via 2CADR (two-word address).
; ============================================================================
RETNMORE	EXTEND			# RETURN TO CHEKMORE
		DCA	MORECADR
		DTCB
; Load double-word address MORECADR and transfer control back to CHEKMORE,
; handling bank switching automatically. Returns DAPIDLER to idle loop.

		EBANK=	AOSQ
MORECADR	2CADR	CHEKMORE
; Two-word address specifying CHEKMORE entry point with EBANK=AOSQ.

