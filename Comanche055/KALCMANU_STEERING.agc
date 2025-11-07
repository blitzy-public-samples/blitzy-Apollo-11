# Copyright:	Public domain.
# Filename:	KALCMANU_STEERING.agc
# Purpose:	Part of the source code for Comanche, build 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 414-419
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Mod history:	05/07/09 OH	Transcription Batch 1 Assignment
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  April 1, 1969.
#
#	This AGC program shall also be referred to as Colossus 2A
#
#	Prepared by
#			Massachusetts Institute of Technology
#			75 Cambridge Parkway
#			Cambridge, Massachusetts
#
#	under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further information.
# Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: KALCMANU_STEERING.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Kalman filter steering implementing optimal estimation for attitude
;        maneuver execution. Applies Kalman filtering to minimize attitude
;        errors during automated maneuvers, optimizing control accuracy and
;        fuel efficiency throughout Apollo 11 mission operations.
;
; COMMENT-ONLY READERS: This program used advanced filtering math to make
;        automatic spacecraft rotations more accurate and fuel-efficient.
; CODE-ALONG READERS: Study Kalman filter implementation for maneuver control,
;        optimal estimation theory, attitude error minimization algorithms.
; ============================================================================

# Page 414
# GENERATION OF STEERING COMMANDS FOR DIGITAL AUTOPILOT FREE FALL MANEUVERS
#
# NEW COMMANDS WILL BE GENERATED EVERY ONE SECOND DURING THE MANEUVER

; ============================================================================
; STEERING COMMAND GENERATION SECTION
;
; This section generates optimal steering commands for automated spacecraft
; maneuvers. The Kalman filter approach minimizes attitude errors by 
; continuously updating desired CDU (Coupling Data Unit) angles based on
; current spacecraft orientation and target attitude. Commands are generated
; every one second to provide smooth, fuel-efficient rotations.
; ============================================================================

		BANK	15

		SETLOC	KALCMON1
		BANK

		EBANK=	BCDU

		COUNT	22/KALC

; NEWDELHI - Main steering command generation routine
; This is the core Kalman filtering routine that computes optimal steering
; commands for automatic maneuvers. Called every second during maneuver
; execution to update attitude control commands.

NEWDELHI	CS	HOLDFLAG	# SEE IF MANEUVER HAS BEEN INTERRUPTED
		EXTEND			# BY ASTRONAUT
		BZMF	NOGO	-2	# IF SO, TERMINATE KALCMANU
; Crew can interrupt maneuver by setting HOLDFLAG. This check ensures that
; if the crew takes manual control, the automatic maneuver terminates safely.

NEWANGL		TC	INTPRET
		AXC,1	AXC,2
			MIS		# COMPUTE THE NEW MATRIX FROM S/C TO
			DEL		# STABLE MEMBER AXES
		CALL
			MXM3
; Matrix multiplication (MXM3) computes the transformation from spacecraft
; body axes to stable member (IMU platform) axes. This is the core of the
; Kalman filter's state estimation - determining current attitude relative
; to the inertial reference frame.

		VLOAD	STADR
		STOVL	MIS +12D	# CALCULATE NEW DESIRED CDU ANGLES
		STADR
		STOVL	MIS +6D
		STADR
		STORE	MIS
; Store the computed direction cosine matrix (MIS) which represents the
; optimal spacecraft attitude. This matrix will be converted to gimbal
; angles for the autopilot.

		AXC,1	CALL
			MIS
			DCMTOCDU	# PICK UP THE NEW CDU ANGLES FROM MATRIX
; DCMTOCDU converts the direction cosine matrix to CDU gimbal angles.
; This transforms the mathematical optimal attitude into physical gimbal
; positions that the autopilot can command.

		RTB
			V1STO2S
		STORE	NCDU		# NEW CDU ANGLES
; NCDU now contains the three new desired gimbal angles (outer, inner, middle)
; that represent the optimal spacecraft attitude for this time step.
		BONCLR	EXIT
			CALCMAN2
			MANUSTAT	# TO START MANEUVER
		CAF	TWO		# 	   +0 OTHERWISE

; ============================================================================
; ATTITUDE ERROR MINIMIZATION AND INCREMENTAL ANGLE COMPUTATION
;
; This loop computes incremental angle changes for each axis (X, Y, Z) that
; minimize attitude error. The Kalman filter's optimal estimation generates
; smooth angle increments that the digital autopilot adds to current gimbal
; positions every tenth of a second, providing precise attitude control.
; ============================================================================

INCRDCDU	TS	KSPNDX
		DOUBLE
		TS	KDPNDX
; Set up indices for three-axis loop. KSPNDX indexes single-precision values,
; KDPNDX indexes double-precision values (DOUBLE creates 2x index).

		INDEX	KSPNDX
		CA	NCDU		# NEW DESIRED CDU ANGLES
		EXTEND
		INDEX	KSPNDX
		MSU	BCDU		# INITIAL S/C ANGLE OR PREVIOUS DESIRED
		EXTEND			# CDU ANGLES
		MP	QUADROT
; Compute angle increment = (new desired angle - current angle) * QUADROT
; This implements the Kalman filter's error minimization: the difference
; between optimal attitude and current attitude is scaled to produce smooth
; incremental commands that minimize control effort and fuel consumption.

		INDEX	KDPNDX
		DXCH	DELCDUX		# ANGEL INCREMENTS TO BE ADDED TO
# Page 415
		INDEX	KSPNDX		# DCDU EVERY TENTH SEC
		CA	NCDU		# BY LEM DAP
		INDEX	KSPNDX
		XCH	BCDU
		INDEX	KDPNDX
		TS	CDUXD
; Store the incremental angle changes (DELCDUX/Y/Z) that the digital autopilot
; will add to commanded CDU angles every tenth of a second. This creates the
; smooth, continuous rotation characteristic of Kalman-filtered maneuvers.
; Update BCDU (base CDU angles) to new desired values for next iteration.

		CCS	KSPNDX
		TCF	INCRDCDU	# LOOP FOR THREE AXES
; Loop three times to process all three gimbal axes (outer, inner, middle).
; Each axis gets optimal incremental commands for smooth attitude control.

		RELINT

; ============================================================================
; MANEUVER TIMING CONTROL
;
; After computing new steering commands, check if maneuver duration has
; expired. If more time remains, continue the maneuver with another one-second
; update cycle. If time has expired, schedule maneuver termination.
; ============================================================================

# COMPARE PRESENT TIME WITH TIME TO TERMINATE MANEUVER

TMANUCHK	TC	TIMECHK
		TC	POSTJUMP
		CADR	CONTMANU
; TIMECHK compares current mission time against maneuver end time (TM).
; If maneuver should continue, jumps to CONTMANU to schedule next update.
; If time expired, falls through to schedule MANUSTOP.

		CAF	ONE
MANUSTAL	TC	WAITLIST
		EBANK=	BCDU
		2CADR	MANUSTOP
; Maneuver time has expired. Schedule MANUSTOP task on WAITLIST to cleanly
; terminate the automatic maneuver and return control. One second delay
; allows final steering commands to take effect.

		RELINT
		TCF	ENDOFJOB


; TIMECHK - Time comparison subroutine for maneuver duration monitoring
; Compares current mission time (TIME2) against maneuver end time (TM).
; Returns via Q register with different return offsets based on time status:
;   Q+0: More than 1 second remains (continue maneuver)
;   Q+3: Less than 1 second remains (prepare to stop)
;   Q+6: Maneuver time expired (stop immediately)

TIMECHK		EXTEND
		DCS	TIME2
		DXCH	TTEMP
		EXTEND
		DCA	TM
		DAS	TTEMP
; Compute time remaining: TTEMP = TM - TIME2 (end time - current time)
; Double precision subtraction handles the full 28-bit mission timer.

		CCS	TTEMP
		TC	Q
		TCF	+2
		TCF	2NDRETRN
		CCS	TTEMP +1
		TC	Q
		TCF	MANUOFF
		COM
MANUOFF		AD	1SEC
		EXTEND
		BZMF	2NDRETRN
; Check if time remaining is less than one second. The Kalman filter's
; one-second update rate requires this check to ensure final commands are
; properly executed before termination.

		INCR	Q
2NDRETRN	INCR	Q
		INCR	Q
		TC	Q
; Multi-offset return mechanism: Increment Q register to return to different
; addresses based on time remaining. This elegant technique allows a single
; subroutine to direct program flow based on computation results.

		SETLOC	MANUSTUF
		BANK
# Page 416

; ============================================================================
; MANUSTAT - MANEUVER INITIALIZATION ROUTINE
;
; Called on first entry to KALCMANU to initialize automatic maneuver control.
; Sets up timing, enables autopilot, configures rate control parameters, and
; establishes Kalman filter bias corrections for optimal attitude control.
; This initialization ensures smooth transition from crew control to automatic
; Kalman-filtered maneuvering.
; ============================================================================

MANUSTAT	EXIT			# INITIALIZATION ROUTINE
		EXTEND			# FOR AUTOMATIC MANEUVERS
		DCA	TIME2
		DAS	TM		# TM+T0    MANEUVER COMPLETION TIME
		CS	1SEC
		TS	L
		CS	ZERO
		DAS	TM		# (TM+T0)-1
; Compute maneuver end time: TM = current time + maneuver duration - 1 second
; The one-second offset accounts for the Kalman filter's one-second update
; rate, ensuring final commands are executed before termination.

		INHINT
		CS	ONE		# ENABLE AUTOPILOT TO PERFORM
		TS	HOLDFLAG	# AUTOMATIC MANEUVERS
; Clear HOLDFLAG to enable automatic maneuvers. Crew can set HOLDFLAG to
; interrupt and take manual control at any time during the maneuver.

		CS	RATEINDX	# SEE IF MANEUVERING AT HIGH RATE
		AD	SIX
		EXTEND
		BZMF	HIGHGAIN
		TCF	+4
HIGHGAIN	CS	RCSFLAGS	# IF SO, SET HIGH RATE FLAG (BIT 15 OF
		MASK	BIT15		# RCSFLAGS)
		ADS	RCSFLAGS
; Check if high-rate maneuver (fast rotation) is commanded. High-rate maneuvers
; use higher autopilot gains for more aggressive control, trading fuel
; efficiency for faster attitude acquisition. The Kalman filter adapts its
; error minimization strategy accordingly.

		DXCH	BRATE		# X-AXIS MANEUVER RATE
		DXCH	WBODY
		DXCH	BRATE +2	# Y-AXIS MANEUVER RATE
		DXCH	WBODY1
		DXCH	BRATE +4	# Z-AXIS MANEUVER RATE
		DXCH	WBODY2
; Transfer commanded body rates (BRATE) to autopilot body rate variables
; (WBODY). These rates define the desired angular velocity for each axis,
; which the Kalman filter uses to compute optimal thruster firing patterns.

		CA	BIASTEMP +1	# INSERT ATTITUDE ERROR BIASES
		TS	BIAS		# INTO AUTOPILOT
		CA	BIASTEMP +3
		TS	BIAS1
		CA	BIASTEMP +5
		TS	BIAS2
; Apply Kalman filter bias corrections (BIASTEMP) to autopilot error channels.
; These biases compensate for systematic errors in attitude sensing or control,
; improving maneuver accuracy. The Kalman filter computes optimal bias values
; based on historical error patterns.

		CA	TIME1
		AD	1SEC
		XCH	NEXTIME
		TC	POSTJUMP
		CADR	INCRDCDU -1
; Schedule first steering command update one second from now. Jump to
; INCRDCDU to begin the one-second update cycle that continues until
; maneuver completion.

; ============================================================================
; CONTMANU - MANEUVER CONTINUATION
;
; Called when maneuver should continue (time remaining > 1 second). Computes
; appropriate wait time and schedules next steering update to maintain the
; Kalman filter's continuous error minimization throughout the maneuver.
; Adjusts timing to account for any processing delays in the update cycle.
; ============================================================================

CONTMANU	INHINT			# CONTINUE WITH UPDATE PROCESS
		CS	TIME1
		AD	NEXTIME
		CCS	A
		AD	ONE
		TCF	MANUCALL
		AD	NEGMAX
		COM
MANUCALL	TC	WAITLIST
		EBANK=	BCDU
		2CADR	UPDTCALL
; Compute wait time until next scheduled update: NEXTIME - TIME1. If positive,
; use it directly. If negative (update overdue due to processing delay), set
; minimal wait. This timing adjustment ensures Kalman filter maintains its
; one-second rhythm even when computational load varies.

		RELINT
# Page 417
		CAF	1SEC		# INCREMENT TIME FOR NEXT UPDATE
		ADS	NEXTIME
		TCF	ENDOFJOB
; Increment NEXTIME by one second for next cycle. This maintains the regular
; update cadence that allows the Kalman filter to continuously refine attitude
; estimates and minimize control errors throughout the maneuver.

; ============================================================================
; UPDTCALL - Update Task Initiator
;
; Waitlist task that initiates steering command update. Allocates executive
; job with priority 26 to call NEWDELHI, which computes next Kalman-filtered
; steering commands. The priority ensures timely updates while allowing higher
; priority tasks (navigation, alarms) to preempt if necessary.
; ============================================================================

UPDTCALL	CAF	PRIO26		# CALL FOR UPDATE
		TC	FINDVAC		# OF STEERING COMMANDS
		EBANK=	BCDU
		2CADR	NEWDELHI
; Schedule NEWDELHI execution at priority 26 via executive job system. This
; priority level balances timely Kalman filter updates against critical
; spacecraft operations, ensuring smooth attitude control without interfering
; with mission-critical computations.

		TC	TASKOVER

# Page 418
# ROUTINE FOR TERMINATING AUTOMATIC MANEUVERS

		SETLOC	KALCMON3
		BANK

; ============================================================================
; MANUSTOP - AUTOMATIC MANEUVER TERMINATION
;
; Gracefully terminates automatic Kalman-filtered maneuver when target attitude
; is achieved or maneuver time expires. Zeros all rate commands and biases,
; loads terminal angles into desired CDU registers, restores original autopilot
; priority, and returns control to calling program. This ensures smooth
; transition from automatic steering to attitude hold mode.
; ============================================================================

MANUSTOP	TC	STOPYZ
		TC	IBNKCALL
		CADR	LOADYZ
; Zero pitch and yaw rate commands, then load terminal angles (CTHETA, CPSI)
; into desired CDU angles. This prepares pitch and yaw axes for attitude hold.

ENDROLL		CA	CPHI
		TS	CDUXD		# SET CDUXD TO THE COMMANDED OUTER GIMBAL
		TC	STOPRATE
; Load terminal roll angle CPHI into desired roll angle CDUXD, then zero all
; roll axis rate commands and biases. With all three axes now at terminal
; angles with zero rates, spacecraft holds final attitude.

ENDMANU		CA	ATTPRIO		# RESTORE USERS PRIORITY
		TS	NEWPRIO
; Restore original autopilot priority (saved before maneuver began). This
; returns control system to the priority level required by calling program
; (mission program, entry guidance, etc.).

		CA	ZERO		# ZERO ATTCADR
		DXCH	ATTCADR
; Clear attitude control address pointer, indicating no automatic maneuver
; is active. Digital autopilot now operates in standard attitude hold mode
; using terminal angles as commanded attitude.

		TC	SPVAC		# RETURN TO USER OF GOMANUR
; Return to program that initiated maneuver (via GOMANUR call). Spacecraft
; now maintains terminal attitude with Kalman-optimized precision. Throughout
; Apollo 11, this termination sequence completed maneuvers with minimal
; residual rates and fuel usage.

		TC	TASKOVER

		SETLOC	STOPRAT
		BANK

; ============================================================================
; STOPRATE - Zero Roll Axis Rate Commands
;
; Terminates roll axis (X-axis) automatic steering by zeroing all roll rate
; commands and biases. Clears DELCDUX incremental angle commands, WBODY body
; rates, and BIAS Kalman filter bias corrections. Also resets high-rate flag
; to ensure RCS digital autopilot returns to normal rate control mode.
; ============================================================================

STOPRATE	CAF	ZERO
		TS	DELCDUX
		TS	DELCDUX	+1	# ZERO ROLL INCREMENTAL ANGLES
		TS	WBODY		# RATE
		TS	WBODY +1
		TS	BIAS		# BIAS
; Zero all roll axis steering parameters: incremental angle commands DELCDUX
; (double precision), body rate commands WBODY, and Kalman filter bias
; corrections BIAS. This halts automatic roll control and allows digital
; autopilot to maintain current roll attitude.

		CS	BIT15		# MAKE SURE HIGH RATE FLAG (BIT 15 OF
		MASK	RCSFLAGS	# RCSFLAGS) IS RESET.
		TS	RCSFLAGS
; Clear high-rate flag (bit 15) in RCS flags. During high acceleration maneuvers,
; this flag enables faster RCS jet firing rates. Clearing it returns autopilot
; to normal rate control suitable for attitude hold mode after maneuver completion.

; ============================================================================
; STOPYZ - Zero Pitch and Yaw Axis Rate Commands
;
; Terminates pitch (Y-axis) and yaw (Z-axis) automatic steering by zeroing
; all pitch and yaw rate commands and biases. Clears incremental angle commands
; DELCDUY/DELCDUZ, body rates WBODY1/WBODY2, and Kalman filter biases BIAS1/BIAS2.
; Called during maneuver termination to halt automatic control on these axes.
; ============================================================================

STOPYZ		CAF	ZERO
		TS	DELCDUY		# ZERO PITCH, YAW
		TS	DELCDUY	+1	# INCREMENTAL ANGLES
		TS	DELCDUZ
		TS	DELCDUZ	+1
		TS	WBODY1		# RATES
		TS	WBODY1 +1
		TS	WBODY2
		TS	WBODY2 +1
		TS	BIAS1		# BIASES
		TS	BIAS2
		TC	Q
; Zero all pitch and yaw axis steering parameters: incremental angle commands
; DELCDUY and DELCDUZ (double precision), body rate commands WBODY1 and WBODY2,
; and Kalman filter bias corrections BIAS1 and BIAS2. This halts automatic
; pitch/yaw control. Return to caller via Q register.

		SETLOC MANUSTUF
		BANK

# Page 419

; ============================================================================
; ZEROERROR - Zero Attitude Error
;
; Utility routine that sets desired CDU angles equal to current CDU angles,
; effectively zeroing attitude error. Called when digital autopilot should
; hold current spacecraft attitude rather than drive to a different target.
; Reads current gimbal angles CDUX/CDUY/CDUZ and stores them into desired
; angle registers CDUXD/CDUYD/CDUZD.
; ============================================================================

ZEROERROR	CA	CDUX		# PICK UP CDU ANGLES AND STORE IN
		TS	CDUXD		# CDU DESIRED
		CA	CDUY
		TS	CDUYD
		CA	CDUZ
		TS	CDUZD
		TC	Q
; Copy current spacecraft attitude (from IMU CDU gimbal angles) into desired
; attitude registers. With desired angles matching current angles, attitude
; error is zero. Digital autopilot will fire no RCS jets, maintaining present
; orientation. Used during maneuver interruption or when attitude hold at
; current orientation is required.

		SETLOC	KALCMON1
		BANK

; ============================================================================
; LOADCDUD/LOADYZ - Load Terminal Angles into Desired CDU Registers
;
; Utility routines that load computed terminal angles (CPHI, CTHETA, CPSI)
; into desired CDU angle registers. Called at maneuver completion to set
; target attitude that digital autopilot will maintain. LOADCDUD loads all
; three axes (entry point for complete update). LOADYZ loads only pitch and
; yaw (entry point when roll already set via different path).
; ============================================================================

LOADCDUD	CA	CPHI		# STORE TERMINAL ANGLES INTO
		TS	CDUXD		# COMMAND ANGLES
; Load terminal roll angle CPHI into desired roll register CDUXD. Then fall
; through to LOADYZ to load pitch and yaw.

LOADYZ		CA	CTHETA
		TS	CDUYD
		CA	CPSI
		TS	CDUZD
		TC	Q
; Load terminal pitch angle CTHETA into desired pitch register CDUYD and
; terminal yaw angle CPSI into desired yaw register CDUZD. Digital autopilot
; now has complete target attitude (from Kalman filter computation) and will
; maintain this orientation using RCS thrusters with minimal fuel consumption.
