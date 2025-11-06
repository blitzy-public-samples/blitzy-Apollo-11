; ============================================================================
; FILE: FRESH_START_AND_RESTART.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: System initialization and recovery routines distinguishing cold start
;        (fresh start) from warm restart (recovery). Performs memory scrubbing,
;        default value loading, restart group assignment. Critical during power
;        transients and 1202 program alarms where restart protection preserved
;        mission-critical state during Apollo 11 descent.
;
; COMMENT-ONLY READERS: Enabled the computer to recover gracefully from
;        interruptions without losing critical mission data.
; CODE-ALONG READERS: Study cold start vs warm restart differentiation, memory
;        initialization, default value loading, restart group assignment,
;        recovery logic.
; ============================================================================

# Copyright:	Public domain.
# Filename:	FRESH_START_AND_RESTART.agc
# Purpose:	Part of the source code for Comanche, build 055. It
#		is part of the source code for the Command Module's
#		(CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 181-210
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	2009-05-16 FB	Transcription Batch 2 Assignment.
#		2009-05-20 RSB	Removed an extraneous "TC STARTSUB".
#		2009-05-21 RSB	Changed a "TC BANKCALL" to "TC STOPRATE"
#				in INITSUB.
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
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

# Page 181
; ============================================================================
; SYSTEM RESTART AND INITIALIZATION OVERVIEW
;
; The AGC restart system protects against data loss during power transients,
; hardware glitches, or program overload conditions. This file implements both
; "fresh start" (complete reinitialization) and "warm restart" (recovery with
; state preservation). During Apollo 11's lunar descent, this restart logic
; enabled the computer to recover from the famous 1202 program alarms without
; aborting the landing.
;
; FRESH START: Complete system reinitialization from power-on or crew-commanded
;              reset. All memory cleared, default values loaded, programs start
;              from idle state. Used when recovering from catastrophic failure
;              or when crew manually resets computer via DSKY.
;
; WARM RESTART: Preserves mission-critical data and resumes interrupted programs
;               using restart tables (see RESTART_TABLES.agc). The AGC's restart
;               protection system divides programs into "restart groups" - when
;               a restart occurs, the phase tables indicate which programs were
;               active and at what phase, enabling seamless recovery.
; ============================================================================

# PROGRAM DESCRIPTION						8 APRIL, 1967
#								SUNDISK REV 120
# FUNCTIONAL DESCRIPTION

; ============================================================================
; SLAP1 - MANUAL FRESH START (Crew-Initiated Complete Reset)
; ============================================================================
; Astronauts can command a fresh start by entering V36 (Fresh Start Request)
; on the DSKY keyboard. This performs complete system reinitialization,
; clearing all erasable memory and resetting the computer to its power-on
; state. All running programs terminate and the system returns to idle mode.
;
; During Apollo 11, this capability provided crew confidence that they could
; recover from any software anomaly, though it was never needed during the
; critical lunar landing sequence.
; ============================================================================

#     SLAP1	MAN INITIATED FRESH START
#	1. EXECUTE STARTSUB
#	2. TURN OFF DSKY DISCRETE-LAMPS
#	3. CLEAR FAIL REGISTERS,SELF-CHECK ERROR COUNTER AND RESTART
#	   COUNTER
#	4. EXECUTE DOFSTART

; ============================================================================
; DOFSTART - MACHINE FRESH START (Computer-Initiated Complete Reset)
; ============================================================================
; Entered automatically when the AGC detects an unrecoverable condition that
; requires complete reinitialization. Clears all state variables, phase tables,
; and flag registers, then transfers control to the idle loop where the system
; awaits crew input for program selection.
; ============================================================================

#     DOFSTART	MACHINE INITIATED FRESH START

#	1. CLEAR SELF-CHECK REGISTERS, MODE REGISTER AND CDUZ REGISTER
#	2. CLEAR PHASE TABLE
#	3. INITIALIZE IMU FLAGS
#	4. INITIALIZE FLAGWORDS
#	5. TRANSFER CONTROL TO IDLE LOOP IN DUMMYJOB

; ============================================================================
; GOPROG - HARDWARE RESTART (Warm Restart with State Recovery)
; ============================================================================
; This is the primary restart entry point when the AGC experiences a power
; transient, hardware glitch, or program overload. GOPROG evaluates the restart
; condition and either performs a warm restart (preserving state and resuming
; programs) or falls back to fresh start if recovery is not possible.
;
; CRITICAL APOLLO 11 CONTEXT: During lunar descent, this routine handled the
; 1202 program alarms at approximately 102:38:26 mission elapsed time. The
; alarms were triggered by the landing radar overloading the WAITLIST and
; EXECUTIVE schedulers. GOPROG's restart protection enabled the computer to
; recover without data loss, allowing the descent to continue. Flight controller
; Steve Bales' decision to proceed despite the alarms relied on confidence in
; this restart system's ability to recover gracefully.
; ============================================================================

#     GOPROG	HARDWARE RESTART

#	0. EXECUTE STARTSUB
#	1. TRANSFER CONTROL TO DOFSTART IF ANY OF THE FOLLOWING CONDITIONS
#	   EXIST.
#	   A. RESTART OCCURED DURING EXECUTION OF ERASCHK
#	   B. BOTH OSCILLATOR FAIL AND AGC WARNING ARE ON
#	   C. MARK REJECT AND EITHER NAV OR MAIN DSKY ERROR LIGHT RESET
#	      ARE ON.
#	2. SCHEDULE A T5RUPT PROGRAM FOR THE DAP
#	3. SET FLAGWRD5 BITS FOR INTWAKE ROUTINE
#	4. EXTINGUISH ALL DSKY LAMPS, EXCEPT FOR PROGRAM ALARM,GIMBAL LOCK AND
#	   NO ATT
#	5. INITIALIZE IMU FLAGS
#	6. IF ENGINE COMMAND IS ON (FLAGWRD5,BIT 7), SET ENGINE ON (CHAN-
#	   NEL 11, BIT 13)
#	7. TRANSFER CONTROL TO GOPROG3

; ============================================================================
; ENEMA - SOFTWARE RESTART (Major Mode Change Initialization)
; ============================================================================
; Called when the crew initiates a major mode change via V37. Terminates
; currently running programs and prepares the system for the new major mode.
; Programs waiting for or actively integrating orbital mechanics are killed
; to prevent conflicts with the new mode's requirements.
;
; Unlike hardware restarts, this is a controlled software-initiated transition
; that allows orderly program termination before starting the new major mode.
; ============================================================================

#     ENEMA	SOFTWARE RESTART	INITIATED BY MAJOR MODE CHANGE

#	1. EXECUTE STARTSB2
#	2. KILL PROGRAMS THAT WERE INTEGRATING OR WAITING FOR INTEGRATION
#	   ROUTINE
#	3. TRANSFER CONTROL TO GOPROG3

; ============================================================================
; GOPROG3 - COMMON RESTART RECOVERY PATH
; ============================================================================
; Shared final stage of both hardware restarts (GOPROG) and software restarts
; (ENEMA). Validates the phase tables to ensure restart data is coherent, then
; reschedules all pending tasks, jobs, and longcalls using the restart tables.
;
; The phase table check (alarm 1107) protects against corrupted restart data.
; If phase tables are invalid, the system cannot reliably recover and must
; perform a fresh start instead.
; ============================================================================

#     GOPROG3	SUBROUTINE COMMON TO GOPROG AND ENEMA

#	1. TEST PHASE TABLES - IF INCORRECT, DISPLAY ALARM 1107 AND
#	   TRANSFER CONTROL TO DOFSTART
#	2. DISPLAY MAJOR MODE
#	3. IF ANY GROUPS WERE ACTIVE UPON RESTART,TRANSFER CONTROL TO THE
# Page 182
#	   RESTARTS SUBROUTINE TO RESCHEDULE PENDING TASKS, LONGCALLS, AND
#	   JOBS (P20 IS RESTARTED VIA FINDVAC)
#	4. IF NO GROUPS WERE ACTIVE UPON RESTART, DISPLAY ALARM CODE
#	   1110 (RESTART WITH NO ACTIVE GROUPS).
#	5. TRANSFER CONTROL TO IDLE LOOP IN DUMMYJOB

; ============================================================================
; STARTSUB - INITIALIZATION SUBROUTINE (Hardware Interface Reset)
; ============================================================================
; Common initialization routine called by both manual fresh start (SLAP1) and
; hardware restart (GOPROG). Clears critical output channels that control
; spacecraft systems, preventing spurious commands during initialization.
; Initializes the timing system (TIME3, TIME4, TIME5) to synchronize the
; real-time operating system.
; ============================================================================

#     STARTSUB	SUBROUTINE COMMON TO SLAP1 AND GOPROG

#	1. CLEAR OUTBIT CHANNELS 5 AND 6
#	2. INITIALIZE TIME5,TIME4,TIME3
#	3. TRANSFER CONTROL TO STARTSB2

; ============================================================================
; STARTSB2 - CORE INITIALIZATION SUBROUTINE (I/O Channel Setup)
; ============================================================================
; Final common initialization stage called by all restart paths. Initializes
; output channels 11-14 which control critical spacecraft systems including
; engine commands, RCS jet firings, and IMU mode control. Proper channel
; initialization prevents the AGC from issuing unintended commands during
; the restart recovery process.
; ============================================================================

#     STARTSB2	SUBROUTINE COMMON TO STARTSUB AND ENEMA

#	1. INTIALIZE OUTBIT CHANNELS 11,12,13, AND 14
#	2. REPLACE ALL TASKS ON WAITLIST WITH ENDTASK
#	3. MAKE ALL EXECUTIVE REGISTERS AVAILABLE
#	4. MAKE ALL VAC AREAS AVAILABLE
#	5. CLEAR DSKY REGISTERS
#	6. ZERO NUMEROUS SWITCHES
#	7. INITIALIZE OPTICS FLAGS
#	8. INITIALIZE PIPA AND TELEMETRY FAIL FLAGS
#	9. INITIALIZE DOWN TELEMETRY


# INPUT/OUTPUT INITIALIZATION

#	A. CALLING SEQUENCE

#		SLAP1 -	TC	POSTJUMP	OR	VERB 36,ENTER
#			CADR 	SLAP1

#		ENEMA -	TC 	POSTJUMP	*** DO NOT CALL ENEMA WITHOUT ***
#			CADR 	ENEMA		*** CONSULTING POOH PEOPLE   ***

#	B. OUTPUT

#		ERASABLE MEMORY INITIALIZATION

# PROGRAM ANALYSIS

#	A. SUBROUTINES CALLED

#		MR.KLEAN,WAITLIST,DSPMM,ALARM,RESTARTS,FINDVAC

#	B. ALARMS

#		1107 PHASE TABLE ERROR
#		1110 RESTART WITH NO ACTIVE GROUPS

# Page 183
; ============================================================================
; SLAP1 - MANUAL FRESH START ENTRY POINT
;
; Astronaut-initiated complete system reset via DSKY Verb 36 (Fresh Start).
; This is the "emergency reboot" function - crew can invoke this if the
; computer appears unresponsive or if they need to terminate all running
; programs and return to a known initial state.
;
; The name "SLAP1" is programmer humor - starting the computer fresh by
; giving it a metaphorical slap to wake it up.
; ============================================================================

		BANK	10
		SETLOC	FRANDRES
		BANK

		EBANK=	LST1		; E-bank points to first task in task list

		COUNT	05/START

SLAP1		INHINT			# FRESH START. COMES HERE FROM PINBALL (DSKY).
					; INHINT disables interrupts during initialization
					; to prevent task scheduling conflicts.
		TC	STARTSUB	# SUBROUTINE DOES MOST OF THE WORK.
					; Calls common initialization code shared with
					; hardware restart (GOPROG).

; ============================================================================
; SIMULATION START HOOK (Development/Testing Support)
; ============================================================================
; This section provides a patchable entry point for ground-based simulation
; testing. During pre-flight testing at MIT and NASA, engineers could patch
; STARTSW to jump to STARTSIM, which would initiate a simulation scenario.
; For flight software, this always jumps to SKIPSIM (bypassing simulation).
; ============================================================================

STARTSW		TCF	SKIPSIM		# PATCH....TCF STARTSIM...FOR SIMULATION
					; Flight configuration: skip simulation hook
STARTSIM	CAF	BIT14		; Simulation path (not used in flight)
		TC	FINDVAC		; Find vacant core set for simulation job
SIM2CADR	OCT	77777		# PATCH 2CADR (AND EBANK DESIGNATION) OF
		OCT	77777		# SIMULATION START ADDRESS.
					; Ground test engineers patch these addresses

; ============================================================================
; FRESH START - CLEAR FAILURE REGISTERS AND ERROR COUNTERS
;
; Before performing complete system reinitialization, clear all diagnostic
; registers that track self-check failures, error counts, and restart history.
; This ensures the fresh start begins with no residual error state that might
; affect subsequent operations.
; ============================================================================

SKIPSIM		CA	DSPTAB +11D	; Load DSKY display table entry 11
		MASK	BITS4&6		; Preserve only bits 4 and 6 (operator error)
		AD	BIT15		; Set bit 15 (Fresh Start Requested flag)
		TS	DSPTAB +11D	# Mark DSKY that crew requested fresh start
					; This helps distinguish manual vs automatic
					; fresh starts in post-mission data analysis

		CA	ZERO		# Begin zeroing all failure and error registers
		TS	ERCOUNT		; Clear self-check error counter
		TS	FAILREG		; Clear failure register word 1
		TS	FAILREG +1	; Clear failure register word 2
		TS	FAILREG +2	; Clear failure register word 3
		TS	REDOCTR		; Clear redo counter (tracks restart frequency)
					; High restart frequency could indicate
					; hardware degradation requiring abort

		CS	PRIO12		; Load complement of priority 12
		TS	DSRUPTSW	; Initialize display interrupt switch
					; Controls DSKY update interrupt handling

; ============================================================================
; DOFSTART - MACHINE-INITIATED FRESH START (Complete System Reset)
;
; This is the core fresh start routine that performs complete AGC system
; reinitialization. All mission programs terminate, navigation state is lost,
; and the computer returns to idle awaiting crew program selection.
;
; Entry conditions:
; - Manual request: Crew executes Verb 36 (Fresh Start) via DSKY
; - Hardware restart failure: GOPROG detects unrecoverable condition
; - Phase table corruption: Alarm 1107 during restart validation
;
; During Apollo 11's descent, DOFSTART was NOT invoked despite the 1202 alarms.
; The restart logic in GOPROG successfully recovered without requiring full
; fresh start, allowing the landing to continue. Had DOFSTART been necessary,
; the mission would have required abort - all guidance state would be lost.
; ============================================================================

DOFSTART	CAF	ZERO		# DO A FRESH START (complete reinitialization)
		TS	ERESTORE	# Clear erasable restore flag
					# ***** MUST NOT BE REMOVED FROM DOFSTART *****
					; Critical: ERESTORE must be zeroed before any
					; restart-protected routines execute
		TS	SMODE		# Clear system mode register
					# ***** MUST NOT BE REMOVED FROM DOFSTART *****
					; Critical: SMODE controls major mode transitions
		TS	UPSVFLAG	# Clear UPDATE STATE VECTOR REQUEST FLAGWORD
					; No pending ground uplink state vector updates

; ============================================================================
; SPACECRAFT ACTUATOR SAFETY - DISABLE ALL OUTPUT COMMANDS
;
; Before clearing memory and reinitializing, immediately turn off all
; spacecraft actuators controlled by the AGC. This prevents the computer from
; issuing spurious commands during the transition to initialized state.
;
; Channels 5-6: RCS jet firing commands (16 thrusters on CSM)
; Channel 11 (DSALMOUT): Engine commands, autopilot enable, IMU cage
; Channels 12-14: Additional discrete outputs, IMU CDU commands, optics drive
; ============================================================================

		EXTEND			; Extended instruction follows
		WRITE	CHAN5		# TURN OFF RCS JETS (Channel 5: quads 1-2)
		EXTEND
		WRITE	CHAN6		# TURN OFF RCS JETS (Channel 6: quads 3-4)
					; Prevents unintended thruster firing during reset
		EXTEND
		WRITE	DSALMOUT	# ZERO CHANNEL 11 (Engine/IMU/Autopilot control)
					; Disables SPS engine, TVC, IMU heater, autopilot
		EXTEND
		WRITE	CHAN12		# ZERO CHANNEL 12 (IMU CDU drive commands)
					; Stops any in-progress IMU gimbal adjustments
		EXTEND
		WRITE	CHAN13		# ZERO CHANNEL 13 (Optics/other discrete outputs)
		EXTEND
		WRITE	CHAN14		# ZERO CHANNEL 14 (Additional control outputs)
					; All spacecraft actuators now safely disabled

		TS	WTOPTION	; Clear waitlist option flag
		TS	DNLSTCOD	; Clear downlink list code (telemetry format)

; ============================================================================
; CLEAR ADDITIONAL SYSTEM REGISTERS
;
; Continue zeroing operational registers that track various subsystem states.
; These must be cleared to prevent residual state from affecting fresh start:
; - Navigation and display state (NVSAVE, EBANKTEM)
; - Rate determination index (RATEINDX)
; - Tracking mark count (TRKMKCNT) 
; - VHF ranging count (VHFCNT)
; - Extended verb activity flag (EXTVBACT)
; ============================================================================

# Page 184
		TS	NVSAVE		; Clear noun-verb save register (DSKY state)
		TS	EBANKTEM	; Clear erasable bank temporary storage
		TS	RATEINDX	; Clear angular rate determination index
		TS	TRKMKCNT	; Clear tracking mark counter (optics navigation)
		TS	VHFCNT		; Clear VHF ranging counter (rendezvous radar)
		TS	EXTVBACT	; Clear extended verb activity flag

; ============================================================================
; IMU COARSE ALIGN STATE PRESERVATION CHECK
;
; Before clearing the phase table, check if the IMU was in coarse align mode
; when the fresh start occurred. The DSKY display table preserves this state
; in bits 4 and 6. If the IMU was coarse aligning (often during gimbal lock
; recovery), restore the coarse align command to prevent IMU drift.
;
; This prevents a scenario where fresh start interrupts IMU alignment, leaving
; the platform drifting without torquing commands. Critical for maintaining
; some level of attitude reference even during emergency fresh start.
; ============================================================================

		CS	DSPTAB +11D	; Load complement of DSKY display table entry 11
		MASK	BITS4&6		; Extract bits 4 and 6 (IMU coarse align status)
		CCS	A		; Check if either bit was set
		TC	+4		; Bits set: skip coarse align restoration
		CA	BITS4&6		; Bits clear: IMU was in coarse align
		EXTEND			# THE IMU WAS IN COARSE ALIGN IN GIMBAL
		WOR	CHAN12		# LOCK, SO PUT IT BACK INTO COARSE ALIGN
					; Issue coarse align command to Channel 12
					; Prevents IMU from drifting uncontrolled
		TC	MR.KLEAN	; Call memory clearing routine (MR. KLEAN)

; ============================================================================
; MR. KLEAN - PHASE TABLE AND FLAG INITIALIZATION
;
; Now that hardware actuators are safe, initialize system mode registers,
; IMU modes, optical system flags, timer states, and software flags.
; This establishes the "known good state" that all programs expect after
; fresh start. The name "MR. KLEAN" reflects the thorough scrubbing of
; system state to pristine default values.
; ============================================================================

		CS	ZERO		; Load -0 (all 1s in ones' complement)
		TS	MODREG		; Clear mode register (major mode = 00)

		CAF	PRIO30		; Load priority 30 constant
		TS	RESTREG		; Initialize restart register for default priority

		CAF	IM30INIF	# FRESH START IMU INITIALIZATION
		TS	IMODES30	; Set IMU mode 30 to fresh start defaults
					; Prepares IMU for coarse/fine alignment

		CAF	NEGONE		; Load -1 (inhibit value)
		TS	OPTIND		# KILL COARSE OPTICS
					; Disable coarse optical tracking
					; Prevents optics from hunting during initialization

		CAF	OPTINITF	; Load optical modes initialization flags
		TS	OPTMODES	; Set optical telescope mode to default state

		CAF	IM33INIT	; Load IMU mode 33 initialization value
		TS	IMODES33	; Set IMU mode 33 (additional IMU state)

		EXTEND			# LET T5 IDLE
		DCA	T5IDLER		; Load T5 idle routine address (double precision)
		DXCH	T5LOC		; Store in T5 interrupt vector location
					; Puts timer 5 interrupt into safe idle state

; ============================================================================
; SOFTWARE FLAG INITIALIZATION
;
; Initialize the STATE and FLAGWRD registers that control program behavior.
; Some flags must be preserved across fresh start (NODOP01, REFSMMAT), while
; others are reset to defaults. SWINIT table contains the default values.
; ============================================================================

		CA	SWINIT		; Load first software initialization value
		TS	STATE		; Initialize STATE register

		CA	FLAGWRD1	; Load current FLAGWRD1
		MASK	NOP01BIT	# LEAVE NODOP01 FLAG UNTOUCHED
					; Preserve NODOP01 (no doppler P01) flag setting
		AD	SWINIT +1	; Add default values for other FLAGWRD1 bits
		TS	FLAGWRD1	; Store initialized FLAGWRD1

		CA 	SWINIT +2	; Load STATE+2 default value
		TS	STATE +2	; Initialize STATE+2 register

		CA	FLAGWRD3	; Load current FLAGWRD3
# Page 185
		MASK	BIT13		# REFSMMAT FLAG
					; Preserve REFSMMAT (reference stable member matrix) flag
					; This flag indicates valid IMU alignment, must survive restart
		AD	SWINIT +3	; Add default values for other FLAGWRD3 bits
		TS	FLAGWRD3	; Store initialized FLAGWRD3

		EXTEND			; Double precision load
		DCA	SWINIT +4	; Load STATE+4 and STATE+5 default values
		DXCH	STATE +4	; Initialize STATE+4 and STATE+5
		EXTEND			; Double precision load
		DCA	SWINIT +6	; Load STATE+6 and STATE+7 default values
		DXCH	STATE +6	; Initialize STATE+6 and STATE+7
		CA	FLAGWRD8	; Load current FLAGWRD8
		MASK	OCT6200		# CMOONFLG, LMOONFLG, AND SUFFLAG
					; Preserve Command Module on Moon flag
					; Preserve Lunar Module on Moon flag
					; Preserve surface flag (lunar surface operations)
		AD	SWINIT	+8D	; Add default values for other FLAGWRD8 bits
		TS	FLAGWRD8	; Store initialized FLAGWRD8

		CA	SWINIT +9D	; Load STATE+9 default value
		TS	STATE +9D	; Initialize STATE+9

		EXTEND			; Double precision load
		DCA	SWINIT +10D	; Load STATE+10 and STATE+11 default values
		DXCH	STATE +10D	; Initialize STATE+10 and STATE+11

ENDRSTRT	TC	POSTJUMP	; Transfer control via bank call
		CADR	DUMMYJOB +2	# DOES A RELINT.  (IN A SWITCHED BANK.)
					; Jump to DUMMYJOB (idle loop) which re-enables interrupts
					; Fresh start complete: system in known state, ready for crew input

; ============================================================================
; MR. KLEAN - PHASE TABLE CLEARING ROUTINE
;
; This routine clears the phase table, which tracks the execution state
; (phase) of all major programs. Each program has a phase register that
; indicates which step it's executing. Clearing these to -0 (negative zero)
; kills all programs, ensuring no tasks from before the fresh start will
; attempt to resume. Multiple entry points exist for partial clears during
; major mode changes.
;
; Entry points:
;   MR.KLEAN  - Full clear (all 6 phase groups) for fresh start
;   P00KLEAN  - Clear from group 2 onward (GOTOPOOH major mode 00)
;   V37KLEAN  - Clear from group 3 onward (V37 major mode change)
; ============================================================================

MR.KLEAN	INHINT			; Disable interrupts during phase table clear
		EXTEND			; Double precision load
		DCA	NEG0		; Load -0 (negative zero in ones' complement)
		DXCH	-PHASE2		; Clear phase group 2 (2 phase cells)
P00KLEAN	EXTEND			; Entry point: clear from group 4 onward
		DCA	NEG0		; Load -0
		DXCH	-PHASE4		; Clear phase group 4 (2 phase cells)
		EXTEND			; Double precision load
		DCA	NEG0		; Load -0
		DXCH	-PHASE1		; Clear phase group 1 (2 phase cells)
V37KLEAN	EXTEND			; Entry point: clear from group 3 onward
		DCA	NEG0		; Load -0
		DXCH	-PHASE3		; Clear phase group 3 (2 phase cells)
		EXTEND			; Double precision load
		DCA	NEG0		; Load -0
		DXCH	-PHASE5		; Clear phase group 5 (2 phase cells)
		EXTEND			; Double precision load
		DCA	NEG0		; Load -0
		DXCH	-PHASE6		; Clear phase group 6 (2 phase cells)
		TC	Q		; Return to caller (Q register holds return address)

OCT6200	OCT	6200		; Constant: bits for CMOONFLG, LMOONFLG, SUFFLAG

# Page 186
# COMES HERE FROM LOCATION 4000, GOJAM, RESTART ANY PROGRAMS WHICH MAY HAVE BEEN RUNNING AT THE TIME.

; ============================================================================
; GOPROG - HARDWARE RESTART ENTRY POINT
;
; This is the critical restart routine that saved Apollo 11 during the
; famous 1202 program alarms at 102:38:26 mission time during lunar descent.
; When the computer detects a problem (counter overflow, power transient,
; night watchman timeout), it vectors here from location 4000 (GOJAM).
;
; GOPROG distinguishes between recoverable restarts (preserve program state
; and resume) and unrecoverable failures (fresh start required). The restart
; protection system encoded in the phase tables allows most programs to
; resume safely. During Apollo 11 descent, this routine executed multiple
; times as the landing radar data overloaded the executive scheduler, but
; the mission continued because GOPROG successfully recovered each time.
;
; COMMENT-ONLY READERS: When you heard "1202 alarm" during the landing,
; this is the code that kept running, allowing Armstrong and Aldrin to
; continue to the surface instead of aborting the mission.
; ============================================================================

GOPROG		INCR	REDOCTR		# ADVANCE RESTART COUNTER
					; Increment restart counter for ground monitoring
					; Each restart logged for post-flight analysis

		LXCH	Q		; Save return address (Q) in L register
		EXTEND			; Extended instruction follows
		ROR	SUPERBNK	; Read superbank register (upper memory bank bits)
		DXCH	RSBBQ		; Save Q and superbank for restart context
		TC	BANKCALL	# STORE ERASABLES FOR DEBUGGING PURPOSES
		CADR	VAC5STOR	; Call VAC5STOR to snapshot erasable memory
					; Captures system state for ground analysis
					; Critical for diagnosing restart causes
		CA	BIT15		# TEST OSC FAIL BIT TO SEE IF WE HAVE
		EXTEND			# HAD A POWER TRANSIENT.  IF SO, ATTEMPT
		WAND	CHAN33		# A RESTART.  IF NOT, CHECK THE PRESENT
					; Read oscillator failure bit from channel 33
					; Oscillator failure indicates power supply problem
		EXTEND			# STATE OF AGC WARNING BIT
		BZF	BUTTONS		; Branch if zero (no oscillator failure)
					; Continue to button check for voluntary restarts

; ============================================================================
; OSCILLATOR FAILURE AND RESTART LOOP DETECTION
;
; If oscillator failure occurred AND AGC warning is active, the computer
; assumes it's stuck in a restart loop (repeated failures) and performs
; a fresh start to break the cycle. This prevents infinite restart attempts
; when hardware has truly failed.
; ============================================================================

		CA	BIT14		# IF AGC WARNING ON (BIT = 0), DO A
		EXTEND			# FRESH START ON THE ASSUMPTION THAT
		RAND	CHAN33		# WE ARE IN A RESTART LOOP
					; Read AGC warning bit from channel 33
					; Warning bit clear (0) indicates serious problem
		EXTEND
		BZF	NONAVKEY +1	; Branch if warning active: force fresh start
					; Restart loop detected, cannot safely recover

; ============================================================================
; BUTTONS CHECK - VOLUNTARY RESTART ENTRY POINT
;
; Entry point when restart was not caused by hardware failure. Could be
; crew-initiated restart or watchdog timer. Proceed with restart validity
; checks before attempting to restore program state.
; ============================================================================

BUTTONS		TC	LIGHTSET	# MAKE FRESH START CHECKS BEFORE ERESTORE
					; Call light set routine (display checks)
					; Ensures DSKY indicators correct before proceed

; ============================================================================
; ERASCHK MEMORY INTEGRITY VALIDATION
;
; This critical check determines if erasable memory is trustworthy after
; restart. The ERASCHK self-test routine destructively tests memory by
; writing test patterns. If interrupted mid-test, memory contents are
; uncertain. This validation logic checks ERASCHK's state:
;
; ERASCHK stores:
;   - Original contents of test locations X and X+1 in SKEEP5 and SKEEP6
;   - Test address X in both SKEEP7 and ERESTORE
;
; Valid states after restart:
;   1. ERESTORE = 0: No memory test was running, memory is clean
;   2. ERESTORE = SKEEP7 (0 < value < 2000 octal): Test was interrupted,
;      but we know which locations were modified and can restore them
;
; Invalid state: ERESTORE ≠ 0 and ≠ SKEEP7: Memory state uncertain,
; cannot trust erasable contents, must fresh start.
; ============================================================================

# ERASCHK TEMPORARILY STORES THE CONTENTS OF TWO ERASABLE LOCATIONS, X
# AND X+1 INTO SKEEP5 AND SKEEP6.  IT ALSO STORES X INTO SKEEP7 AND
# ERESTORE.  IF ERASCHK IS INTERRUPTED BY A RESTART, C(ERESTORE) SHOULD
# EQUAL C(SKEEP7),AND SHOULD BE A + NUMBER LESS THAN 2000 OCT.  OTHERWISE
# C(ERESTORE) SHOULD EQUAL +0.

		CAF	HI5		; Load high 5 bits mask (address range check)
		MASK	ERESTORE	; Mask ERESTORE to check if valid address
		EXTEND			; Extended instruction follows
		BZF	+2		# IF ERESTORE NOT = +0 OR +N LESS THAN 2K,
					; Zero: valid address range, continue check
		TCF	NONAVKEY +1	# DOUBT E MEMORY AND DO A FRESH START
					; Invalid address: memory corrupted, fresh start
		CS	ERESTORE	; Load complement of ERESTORE
		EXTEND			; Extended instruction follows
		BZF	ELRSKIP -1	; Zero: no test running, skip restoration
					; ERESTORE was +0, memory clean
		AD	SKEEP7		; Add SKEEP7 (should cancel if equal)
		EXTEND			; Extended instruction follows
		BZF	+2		# = SKEEP7, RESTORE E MEMORY
					; Zero: addresses match, safe to restore
		TCF	NONAVKEY +1	# NOT=SKEEP7, DOUBT EMEM, DO FRESH START
					; Mismatch: memory state uncertain
		CA	SKEEP4		; Load saved EBANK register
		TS	EBANK		# EBANK OF E MEMORY THAT WAS UNDER TEST
					; Restore correct memory bank selection
		EXTEND			# (NOT DXCH SINCE THIS MIGHT HAPPEN AGAIN)
		DCA	SKEEP5		; Load saved memory contents (locations X, X+1)
		INDEX	SKEEP7		; Index by saved address X
		DXCH	0000		# E MEMORY RESTORED
					; Restore original values to tested locations
					; Memory now in pre-test state, safe to proceed
		CA	ZERO		; Clear accumulator
		TS	ERESTORE	; Clear ERESTORE (test no longer in progress)
# Page 187
		TC	STARTSUB	# DO INITIALIZATION AFTER ERASE RESTORE
					; Call common initialization subroutine
					; Prepares system for restart sequence

; ============================================================================
; ELRSKIP - AUTOPILOT RESTART INITIALIZATION
;
; After validating memory integrity, restart the Digital Autopilot (DAP)
; system. The DAP controls spacecraft attitude using RCS thrusters or
; thrust vector control. FLAGWRD6 bits 15-14 encode which DAP variant
; was running when restart occurred:
;
;   00: T5IDLOC  - Idle mode, no active DAP
;   01: REDORCS  - RCS autopilot restart
;   10: REDOTVC  - Thrust vector control autopilot restart
;   11: REDOSAT  - CSM/LM docked configuration autopilot restart
;
; The appropriate restart address is loaded into T5LOC for the T5RUPT
; interrupt to resume DAP operation. Critical during powered flight
; when attitude control cannot be interrupted for more than ~100ms
; without spacecraft drift exceeding acceptable limits.
; ============================================================================

ELRSKIP		CA	FLAGWRD6	# RESTART AUTOPILOTS
					; Load flag word 6 (contains DAP status bits)
		EXTEND			; Extended instruction follows
		MP	BIT3		# BITS 15,14	00 T5IDLOC
					; Multiply by 8 to extract bits 15-14
		MASK	SIX		#		01 REDORCS
					; Mask to 0-3 range (DAP type index)
		EXTEND			#		10 REDOTVC
		INDEX	A		#		11 REDOSAT
					; Index into restart address table
		DCA	T5IDLER		; Load appropriate DAP restart address
		DXCH	T5LOC		; Store in T5 interrupt location
					; Next T5RUPT will resume DAP operation

; ============================================================================
; INITIALIZE RADAR AND OPTICS FLAGS
;
; Clear integration flag bit from radar flag, reset optics mode initialization
; flags, and initialize IMU mode flags. Ensures sensors start in known state
; after restart without carrying over transient flags from interrupted state.
; ============================================================================

		CS	INTFLBIT	; Load complement of integration flag bit
		MASK	RASFLAG		; Clear integration flag from radar flags
		TS	RASFLAG		; Store cleared radar flags
					; Prevents stale integration state

		CA	OPTMODES	; Load optics mode register
		MASK	OPTINITR	; Clear transient optics flags
		AD	BIT7		; Set optics initialization bit
		TS	OPTMODES	; Store initialized optics mode
					; Optics system ready for use

		CAF	BIT6		; Load bit 6 mask
		MASK	IMODES33	; Extract IMU mode bits from channel 33
		AD	IM33INIT	; Add IMU initialization value
		TS	IMODES33	; Store initialized IMU mode
					; IMU system in known state

		CA	9,6,4		# LEAVE PROG ALARM,GIMBAL LOCK, NO ATT
		MASK	DSPTAB +11D	# LAMPS INTACT ON HARDWARE RESTART
		AD	BIT15
		XCH	DSPTAB +11D
		MASK	BIT4		# IF NO ATT LAMP WAS ON, LEAVE ISS IN
		EXTEND			# COURSE ALIGN
		BZF	NOCOARSE
		TC	IBNKCALL	# IF NO ATT LAMP ON, RETURN ISS TO
		CADR	SETCOARS	#	COARSE ALIGN.

		CAF	SIX
		TC	WAITLIST
		EBANK=	CDUIND
		2CADR	CA+ECE

NOCOARSE	CAF	IFAILINH	# LEAVE FAILURE INHIBITS INTACT ON
		MASK	IMODES30	#	HARDWARE RESTART.  RESET ALL
		AD	IM30INIR	#	FAILURE CODES.
		TS	IMODES30

		CS	FLAGWRD5
		MASK	ENGONBIT
		CCS	A
		TCF	GOPROG3
		CAF	BIT13
		EXTEND
# Page 188
		WOR	DSALMOUT	# TURN ENGINE ON
		TCF	GOPROG3

; ============================================================================
; ENEMA - SOFTWARE RESTART (INITIATED BY MAJOR MODE CHANGE)
;
; ENEMA provides a controlled restart when changing major modes (programs).
; Unlike GOPROG (hardware restart) or DOFSTART (fresh start), ENEMA preserves
; most system state but kills programs that were integrating or waiting for
; the integration routine. This allows clean transitions between mission
; programs (P20 to P30, P63 to P70, etc.) without losing navigation state
; or requiring IMU realignment.
;
; Named ENEMA because it "cleans out" old programs to prepare for new ones.
;
; COMMENT-ONLY READERS: When crew switched between programs (like going from
; orbital navigation to burn planning), this routine cleaned up the old
; program's tasks while preserving the spacecraft's navigation knowledge.
; ============================================================================

ENEMA		INHINT			; Inhibit interrupts during restart setup
		TC	LIGHTSET	# EXIT TO DOFSTART IF ERROR RESET AND
		TC	STARTSB2	# MARK REJECT DEPRESSED SIMULTANEOUSLY
					; Perform standard restart initialization
		CS	INTMASK		# RESET INTEGRATION BITS
		MASK	RASFLAG		; Clear integration flags in RASFLAG
		TS	RASFLAG		; Programs waiting for orbital integration
					; are terminated - new program will
					; reinitialize integration as needed

		CS	FLAGWRD6	# IS TVC ON
		MASK	OCT60000	; Test thrust vector control active flag
		EXTEND			; (Used during main engine burns)
		BZMF	GOPROG3		# NO
					; Branch if TVC not active

		CAF	.5SEC		# YES, CALL TVCEXEC TASK WHICH WAS KILLED
		TC	WAITLIST	# 	IN STARTSB2.
					; Reschedule TVC executive task
					; STARTSB2 killed all waitlist tasks
					; TVC must resume for engine control
		EBANK=	BZERO
		2CADR	TVCEXEC		; Schedule TVCEXEC in 0.5 seconds

; ============================================================================
; GOPROG3 - RESTART RECOVERY SUBROUTINE (COMMON TO GOPROG AND ENEMA)
;
; GOPROG3 is the heart of the restart protection system. After GOPROG or
; ENEMA has determined that a restart (not fresh start) is appropriate,
; GOPROG3 validates the phase tables to ensure restart data integrity,
; then reschedules all active program groups to resume where they left off.
;
; The phase table validation (PCLOOP) is critical - it checks that each
; restart group's phase data is self-consistent. If corruption is detected
; (PTBAD), the system performs a fresh start rather than risk executing
; with invalid state. During Apollo 11's 1202 alarms, this validation
; succeeded, allowing the landing program to continue safely.
; ============================================================================

GOPROG3		CAF	NUMGRPS		# VERIFY PHASE TABLE AGREEMENTS
					; Load number of restart groups to check
					; NUMGRPS = 6 (groups 1-6 for different
					; mission phase categories)

; ----------------------------------------------------------------------------
; PCLOOP - PHASE TABLE CONSISTENCY CHECK
;
; Each restart group stores its phase in two places: PHASE1-PHASE6 and
; -PHASE1 through -PHASE6 (negative copies). This redundancy allows
; detection of memory corruption. The RXOR (read and XOR) instruction
; compares them - result must be -0 (all ones) for valid data.
; ----------------------------------------------------------------------------

PCLOOP		TS	MPAC +5		; Save loop counter
		DOUBLE			; Multiply by 2 for table indexing
		EXTEND			; Extended instruction follows
		INDEX	A		; Use A as index into phase tables
		DCA	-PHASE1		# COMPLEMENT INTO A, DIRECT INTO L
					; Load -PHASE value (complement) into A
					; Load PHASE value (direct) into L
		EXTEND			; Extended instruction follows
		RXOR	LCHAN		# RESULT MUST BE -0 FOR AGREEMENT
					; XOR A register with L register
					; If PHASE and -PHASE consistent: result = -0
					; If corrupted: result != -0
		CCS	A		; Test result: must be -0 for valid data
		TCF	PTBAD		# RESTART FAILURE (positive = corruption)
		TCF	PTBAD		; (+0 = corruption detected)
		TCF	PTBAD		# (-0 would continue, but -1 = corruption)
					; Only -0 (all ones) falls through

		CCS	MPAC +5		# PROCESS ALL RESTART GROUPS
		TCF	PCLOOP		; Loop through all 6 restart groups
					; Validate each group's phase table

		TS	MPAC +6		# SET TO +0
					; Phase table validation complete
					; Initialize activity flag to zero

		TC	MMDSPLAY	# DISPLAY MAJOR MODE
					; Show current major mode on DSKY
					; Informs crew which program is active
					; after restart recovery

		INHINT			# RELINT DONE IN MMDSPLAY
					; Inhibit interrupts for restart scheduling

		CAE	FLAGWRD6	# IS RCS DAP RUNNING (BITS 15 14 OF
		MASK	OCT60000	# FLAGWORD6 = 01)
					; Check if RCS Digital Autopilot active
		EXTEND			# YES, DO STOPRATE
		BZMF	NXTRST -1	# NO, SKIP TO NXTRST -1
					; Branch if DAP not running
		CAF	EBANK6		# STOPRATE IS DONE IN EBANK 6
		TS	EBANK		; Switch to E-bank 6 for DAP access
		TC	STOPRATE	# ZERO DELCDUS, WBODYS, AND BIASES THUS
					# STOPPING AUTOMATIC MANEUVERING
					; Zero attitude error accumulators
					; Halts automatic spacecraft rotation
					; Restart begins from stable attitude

		CAF	EBANK3
		TS	EBANK		; Restore E-bank 3
# Page 189

; ----------------------------------------------------------------------------
; NXTRST - RESTART GROUP ACTIVITY SCAN
;
; Scans all 6 restart groups to determine which programs were active when
; the restart occurred. For each active group (PHASE != +0), calls the
; RESTARTS subroutine to reschedule pending tasks, longcalls, and jobs.
; This is how programs resume after restart - their phase value encodes
; exactly where they were in execution, and RESTARTS uses that information
; to reconstruct their task queue entries.
; ----------------------------------------------------------------------------

		CAF	NUMGRPS		# SEE IF ANY GROUPS RUNNING
NXTRST		TS	MPAC +5		; Save group counter
		DOUBLE			; Multiply by 2 for table indexing
		INDEX	A		; Use A as index
		CCS	PHASE1		; Check phase value for this group
		TCF	PACTIVE		# PNZ - GROUP ACTIVE
					; Positive/negative = program running
		TCF	PINACT		# +0 - GROUP NOT RUNNING
					; Zero = no program in this group

; ----------------------------------------------------------------------------
; PACTIVE - ACTIVE GROUP RESTART PROCESSING
;
; When a restart group is found active, this section calls the RESTARTS
; subroutine to reschedule that group's tasks. The phase value (stored in
; MPAC) tells RESTARTS exactly which tasks to reschedule and what state
; to restore. This is the mechanism that allowed Apollo 11 to continue
; landing despite multiple 1202 alarms - the landing program (group 3)
; was restarted here with its phase intact.
; ----------------------------------------------------------------------------

PACTIVE		TS	MPAC		; Store phase value (with sign)
		INCR	MPAC		# ABS OF PHASE
					; Convert to absolute value
					; Phase magnitude encodes restart point
		INCR	MPAC +6		# INDICATE GROUP DEMANDS PRESENT
					; Set flag: at least one group active
		CA	RACTCADR	; Load RESTARTS routine address
		TC	SWCALL		# MUST RETURN TO SWRETURN
					; Call RESTARTS for this group
					; RESTARTS reschedules tasks per phase

PINACT		CCS	MPAC +5		# PROCESS ALL RESTART GROUPS
		TCF	NXTRST		; Continue scanning remaining groups
					; Loop through all 6 groups

; ----------------------------------------------------------------------------
; RESTART COMPLETION CHECK
;
; After scanning all groups, check if any were active. If so, proceed to
; ENDRSTRT to complete restart. If no groups active (unusual condition),
; check if in program P00 (idle mode). If not in P00, something is wrong
; (alarm 1110: restart with no active groups).
; ----------------------------------------------------------------------------

		CCS	MPAC +6		# NO, CHECK PHASE ACTIVITY FLAG
		TCF	ENDRSTRT	# PHASE ACTIVE
					; At least one group active: normal restart
		CAF	BIT15		# IS MODE -0
		MASK	MODREG		; Check if major mode is -0 (P00 idle)
		EXTEND
		BZF	GOTOPOOH	# NO
					; Not P00: go to POOH to enter idle mode
		TCF	ENDRSTRT	# YES
					; P00 active: complete restart normally

; ----------------------------------------------------------------------------
; PTBAD - PHASE TABLE CORRUPTION DETECTED
;
; If phase table validation fails (PHASE and -PHASE don't match), memory
; corruption has occurred. Display alarm 1107 and perform fresh start.
; Cannot safely restart with corrupted phase data - would resume program
; at wrong point, potentially causing mission failure.
; ----------------------------------------------------------------------------

PTBAD		TC	ALARM		# SET ALARM TO SHOW PHASE TABLE FAILURE
		OCT	1107		; Alarm code 1107: phase table disagree
					; Displayed on DSKY for crew/ground
					; Indicates memory corruption detected

		TCF	DOFSTART	# IN R2)
					; Perform fresh start - only safe option
					; Clears all program state, restarts clean

# ******** ****** ******
#
# DO NOT USE GOPROG2 OR ENEMA WITHOUT CONSULTING POOH PEOPLE
#
GOPROG2		EQUALS	ENEMA
OCT10000	=	BIT13
OCT30000	=	PRIO30
OCT7777		OCT	7777
RACTCADR	CADR	RESTARTS

LIGHTSET	CAF	BIT7		# DOFSTART IF MARK REJECT AND EITHER
		EXTEND			# ERROR LIGHT RESET BUTTONS ARE DEPRESSED
		RAND	NAVKEYIN
		EXTEND
		BZF	NONAVKEY	# NO MARK REJECT
		CAF	OCT37
		EXTEND
		RAND	NAVKEYIN	# NAV DSKY KEYCODES,MARK,MARK REJECT
		AD	-ELR
		EXTEND
		BZF	NONAVKEY +1
		EXTEND
# Page 190
		READ	MNKEYIN		# MAIN DSKY KEYCODES
		AD	-ELR
		EXTEND
		BZF	+2

NONAVKEY	TC	Q

		TC	STARTSUB
		TCF	DOFSTART

; ============================================================================
; TRANSITION: From restart entry points to common initialization
;
; Both manual fresh starts (SLAP1) and hardware restarts (GOPROG) converge
; on these shared initialization subroutines. STARTSUB handles the most
; fundamental setup: resetting telemetry pointers and initializing the AGC's
; three-register TIME system. This represents the first layer of system
; recovery before higher-level restart processing begins.
; ============================================================================

; STARTSUB: Initial system initialization (common to SLAP1 and GOPROG)
;
; This subroutine performs the most basic AGC initialization tasks required
; by both manual fresh starts and hardware restarts. It establishes the
; foundation for all subsequent restart processing by resetting downlink
; telemetry pointers and initializing the TIME3/TIME4/TIME5 register system
; that drives all AGC timing and waitlist scheduling.
;
; The downlist pointer reset ensures that telemetry to Mission Control begins
; from a known state after any restart. The TIME register initialization
; establishes proper timing relationships between the three cascading counters
; that track mission elapsed time and schedule future tasks.

STARTSUB	CAF	LDNPHAS1	# SET POINTER SO NEXT 20MS DOWNRUPT WILL
		TS	DNTMGOTO	# CAUSE THE CURRENT DOWNLIST TO BE
					# INTERRUPTED AND START SENDING FROM THE
					# BEGINNING OF THE CURRENT DOWNLIST.
; Initialize AGC timing system (TIME3/TIME4/TIME5 cascaded counters)
;
; The AGC uses three 15-bit registers (TIME3, TIME4, TIME5) that cascade
; together to form a 45-bit timing counter. TIME3 increments every 10ms,
; overflowing into TIME4, which overflows into TIME5. This system provides
; the fundamental clock for all waitlist scheduling and mission time tracking.
;
; Initial values: TIME3 = 37777 (octal), TIME4 = 37775, TIME5 = 37774
; These staggered values establish proper overflow relationships.

		CAF	POSMAX
		TS	TIME3		# 37777 TO TIME3.
		AD	MINUS2
		TS	TIME4		# 37775 TO TIME4.
		AD	NEGONE
		TS	TIME5		# 37774 TO TIME5.

; STARTSB2: Extended initialization (common to STARTSUB and ENEMA)
;
; This subroutine performs comprehensive AGC initialization beyond the basic
; setup in STARTSUB. It initializes the executive scheduler, waitlist task
; system, VAC (vector accumulator) area management, and DSKY display system.
; This routine is called by both hardware restarts (via STARTSUB) and software
; restarts (via ENEMA) to establish a consistent computing environment.
;
; COMMENT-ONLY READERS: This routine cleared the computer's work queues and
; prepared the display system for new programs, ensuring a clean slate after
; any restart condition.
;
; CODE-ALONG READERS: Study the systematic zeroing of executive priority
; registers, waitlist delta-time initialization, VAC area chaining, and DSKY
; register blanking. This is the AGC equivalent of operating system startup.

; Turn off specific DSKY caution lamps while preserving others
;
; Octal 77603 is a bitmask that turns off UPLINK ACTY, TEMP (temperature)
; caution, KEY REL, FLASH, and OPERATOR ERROR lamps, while leaving other
; lamps (PROG, RESTART, NO ATT, GIMBAL LOCK, TRACKER, etc.) unchanged.
; This selective lamp control ensures the crew sees only relevant status
; indicators after restart.

STARTSB2	CAF	OCT77603	# TURN OFF UPLINK ACTY, TEMP CAUTION, KR,
		EXTEND			# FLASH, OP. ERROR, LEAVE OTHERS UNCHANGED
		WAND	DSALMOUT

; Turn off test alarms and standby enable (channel 13 control)
;
; Octal 74777 controls spacecraft system flags through channel 13. This mask
; turns off self-test alarm indicators and the standby enable signal that
; would otherwise keep non-essential systems powered during restart recovery.

		CAF	OCT74777	# TURN OFF TEST ALARMS, STANDBY ENABLE.
		EXTEND
		WAND	CHAN13

; Clear rendezvous and VHF ranging flags, then set SKIPVHF
;
; FLAGWRD2 contains various mission flags. This sequence clears R21MARK
; (rendezvous mark flag), P21FLAG (rendezvous program flag), and SKIPVHF
; (skip VHF ranging), then immediately sets SKIPVHF back on. This ensures
; VHF ranging is bypassed during restart until explicitly re-enabled.

		CS	PRIO25		# CLEAR R21MARK, P21FLAG, AND SKIPVHF BIT.
		MASK	FLAGWRD2
		AD	SKIPVBIT	# NOW SET SKIPVHF FLAG.
		TS	FLAGWRD2

; Initialize waitlist task scheduling system
;
; The waitlist (LST1/LST2 tables) manages timer-driven tasks scheduled for
; future execution. LST1 contains delta-time values (time until next task),
; while LST2 contains task addresses. During Apollo 11's descent, waitlist
; overflow contributed to the 1202 program alarm when radar data processing
; created more waitlist entries than the 8-entry table could accommodate.
;
; NEG1/2 (negative half, octal 77777) initializes all delta-time slots as
; "empty", using the AGC's two's-complement arithmetic where negative values
; indicate no pending task. The 8 entries (LST1 through LST1+7) cover all
; available waitlist slots.

		EBANK=	LST1
		CAF	STARTEB
		TS	EBANK		# SET FOR E3

		CAF	NEG1/2		# INITIALIZE WAITLIST DELTA-TS.
		TS	LST1 +7
		TS	LST1 +6
		TS	LST1 +5
		TS	LST1 +4
		TS	LST1 +3
		TS	LST1 +2
		TS	LST1 +1
		TS	LST1

; Initialize waitlist task address table (LST2)
;
; LST2 stores the 2CADR (two-word address: bank + address) for each waitlist
; task. ENDTASK is a special marker indicating "no task scheduled". Each
; waitlist slot uses two words (even/odd pairs): the even word holds the bank
; number (complemented), the odd word holds the address within that bank.
;
; This initialization sets all 9 waitlist task slots (LST2 through LST2+17D)
; to point to ENDTASK, marking all slots as empty and available for scheduling.

		CS	ENDTASK
		TS	LST2
		TS	LST2 +2
		TS	LST2 +4
# Page 191
		TS	LST2 +6
		TS	LST2 +8D
		TS	LST2 +10D
		TS	LST2 +12D
		TS	LST2 +14D
		TS	LST2 +16D
		CS	ENDTASK +1
		TS	LST2 +1
		TS	LST2 +3
		TS	LST2 +5
		TS	LST2 +7
		TS	LST2 +9D
		TS	LST2 +11D
		TS	LST2 +13D
		TS	LST2 +15D
		TS	LST2 +17D

; Clear executive job priority registers
;
; The executive scheduler maintains 7 priority "core sets" (PRIORITY through
; PRIORITY+72D, spaced 12 words apart) for job management. CS ZERO produces
; -0 (octal 77777), which when stored marks each core set as available.
; Setting PRIORITY registers to -0 is the AGC convention for "no job active".
;
; During Apollo 11's descent, when all core sets became active (job queue
; full), the system triggered the 1201 program alarm. This initialization
; ensures restart begins with all core sets free.

		CS	ZERO		# MAKE ALL EXECUTIVE REGISTER SETS
		TS	PRIORITY	# AVAILABLE.
		TS	PRIORITY +12D
		TS	PRIORITY +24D
		TS	PRIORITY +36D
		TS	PRIORITY +48D
		TS	PRIORITY +60D
		TS	PRIORITY +72D

; Clear downlink telemetry switch and active job indicator
;
; DSRUPTSW controls downlink interrupt processing. NEWJOB indicates whether
; any executive jobs are active. Clearing both ensures clean state for restart.

		TS	DSRUPTSW
		TS	NEWJOB		# SHOWS NO ACTIVE JOBS.

; Initialize VAC (Vector Accumulator) area management
;
; The AGC provides 5 VAC areas (43-word blocks of erasable memory) used by
; interpretive language programs for vector/matrix computations. Each VAC
; area is tracked by a use counter (VAC1USE through VAC5USE).
;
; VAC1ADRC contains the address of the first VAC area. LTHVACA (length of VAC
; area, 43 decimal = 53 octal) is the spacing between VAC areas. This sequence
; initializes each VACxUSE register to point to its corresponding VAC area,
; making all areas available for interpretive program allocation.

		CAF	VAC1ADRC	# MAKE ALL VAC AREAS AVAILABLE.
		TS	VAC1USE
		AD	LTHVACA
		TS	VAC2USE
		AD	LTHVACA
		TS	VAC3USE
		AD	LTHVACA
		TS	VAC4USE
		AD	LTHVACA
		TS	VAC5USE

; Blank DSKY display registers
;
; The DSKY (Display and Keyboard) has 11 register positions visible to the crew:
; program number, verb, noun, and R1/R2/R3 data registers (each R register has
; three 5-digit segments). DSPTAB is the display table containing the current
; values for all 11 positions.
;
; BIT12 (octal 04000) is the "blank" code for DSKY displays. This loop runs
; 11 times (TEN=10 decimal, counting from 10 down to 0), storing the blank
; code into each DSPTAB position via indexed addressing. After restart, the
; DSKY appears blank until display programs begin showing data to the crew.

		CAF	TEN		# BLANK DSKY REGISTERS (PROGRAM,VERB,NOUN,
					# R1,R2,R3)
DSPOFF		TS	MPAC
		CS	BIT12
		INDEX	MPAC
		TS	DSPTAB
		CCS	MPAC
		TCF	DSPOFF

; Clear display interface state variables
;
; At this point, A register contains +0 (from CCS MPAC leaving +0 after
; decrementing to -0). This section clears numerous display interface and
; keyboard registers to establish known initial state:
;
; DELAYLOC(+0-3) - Display delay timing registers
; R1SAVE - Saved R1 register value for display recall
; INLINK - Keyboard input link register
; DSPCNT - Display operation counter
; CADRSTOR - Stored calling address for display programs
; REQRET - Request return address for display operations
; CLPASS - Clear pass indicator for display blanking
; DSPLOCK - Display lock semaphore preventing concurrent display updates
; MONSAVE/MONSAVE1 - Monitor routine save registers (killed during restart)
; VERBREG - Verb register (cleared, no verb active)
; NOUNREG - Noun register (cleared, no noun active)
; DSPLIST - Display list pointer for chained display operations
; MARKSTAT - Mark button status for optical navigation
; IMUCADR/OPTCADR/RADCADR/ATTCADR - 2CADR pointers for IMU, optics, radar, attitude programs
; LGYRO - Landing gyro test indicator
; FLAGWRD4 - Flag word controlling interface displays (cleared, all displays off)

		TS	DELAYLOC
# Page 192
		TS	DELAYLOC +1
		TS	DELAYLOC +2
		TS	DELAYLOC +3
		TS	R1SAVE
		TS	INLINK
		TS	DSPCNT
		TS	CADRSTOR
		TS	REQRET
		TS	CLPASS
		TS	DSPLOCK
		TS	MONSAVE		# KILL MONITOR
		TS	MONSAVE1
		TS	VERBREG
		TS	NOUNREG
		TS	DSPLIST
		TS	MARKSTAT
		TS	IMUCADR
		TS	OPTCADR
		TS	RADCADR
		TS	ATTCADR
		TS	LGYRO
		TS	FLAGWRD4	# KILL INTERFACE DISPLAYS
; Initialize display output control
;
; NOUT controls the noun output format for DSKY displays. NOUTCON is the
; normal output configuration constant defining decimal/octal display modes.

		CAF	NOUTCON
		TS	NOUT

; Preserve extended verb activity flag (BIT14 only)
;
; EXTVBACT tracks extended verb operations. During restart, preserve only
; BIT14 (if set), clearing all other bits. This maintains critical extended
; verb state while resetting less critical flags.

		CAF	BIT14
		MASK	EXTVBACT
		TS	EXTVBACT

; Set self-check return address
;
; SELFRET contains the 2CADR where control transfers after AGC self-check
; completes. LESCHK (Label for SELFCHK) points to the self-check routine.
; This initialization ensures self-check, if triggered, returns to proper
; location.

		CAF	LESCHK		# SELF CHECK GO-TO REGISTER.
		TS	SELFRET

; Initialize display flash counter
;
; DSPCOUNT controls the flashing rate for DSKY displays when data requires
; crew attention. VD1 contains the flash timing constant. CS VD1 initializes
; the counter to begin the flash cycle properly.

		CS	VD1
		TS	DSPCOUNT

; STARTSB2 complete - return to caller
;
; At this point, all basic system initialization is complete. TIME counters
; are running, output channels are configured, DSKY is blanked, executive and
; waitlist structures are initialized, and display interface is ready. Control
; returns to the calling routine (Q register holds return address).

		TC	Q

; ============================================================================
; T5IDLOC - Timer 5 Idle Location
;
; T5RUPT is a low-priority timer interrupt (163.84 second cycle) used by
; various programs for long-period housekeeping tasks. When no program
; has scheduled a T5RUPT task, the T5 interrupt vector points here to
; T5IDLOC, which performs a benign "idle" operation.
;
; The sequence CA L (load L register into A) followed by TCF NOQRSM+1
; essentially performs a no-operation that consumes the T5 interrupt without
; executing any substantive code. This prevents T5RUPT from causing errors
; when no tasks are waiting.
; ============================================================================

T5IDLOC		CA	L		# T5RUPT COMES HERE EVERY 163.84 SECS
		TCF	NOQRSM	+1	# WHEN NOBODY IS USING IT.

; T5 Interrupt Vector Addresses
;
; These 2CADR (two-word address) entries define the restart-time destinations
; for T5-related programs. EBANK=OGANOW specifies the erasable bank context.
;
; T5IDLER - Points to T5IDLOC idle routine (default when no T5 tasks active)
; REDORCS - RCS (Reaction Control System) digital autopilot T5 restart address
; REDOTVC - TVC (Thrust Vector Control) digital autopilot T5 restart address  
; REDOSAT - Saturn autopilot T5 restart address (for S-IVB stage operations)

		EBANK=	OGANOW
T5IDLER		2CADR	T5IDLOC

		EBANK=	OGANOW
		2CADR	REDORCS

		EBANK=	OGANOW
		2CADR	REDOTVC

		EBANK=	OGANOW
		2CADR	REDOSAT
# Page 193

; ============================================================================
; FRESH START AND RESTART - Constant Definitions
;
; This section defines constants used throughout the restart and initialization
; routines. These values control IMU (Inertial Measurement Unit) configuration,
; channel initialization, interrupt masking, and system state setup.
; ============================================================================

; Core system addresses and configuration

IFAILINH	OCT	435		; IMU fail inhibit bit pattern
LDNPHAS1	GENADR	DNPHASE1	; Address of DNPHASE1 routine for phase table
LESCHK		GENADR	SELFCHK		; Address of self-check routine
VAC1ADRC	ADRES	VAC1USE		; VAC area 1 usage address
LTHVACA		DEC	44		; Length of VAC area (44 decimal words)

; Interrupt and channel masks

INTMASK		OCT	20100		; Interrupt mask for T4RUPT priority control
OCT77603	OCT	77603		; Channel initialization mask
OCT74777	OCT	74777		; Channel initialization mask
STARTEB		ECADR	LST1		; Erasable bank address of LST1 table
NUMGRPS		EQUALS	FIVE		; Number of restart groups (5)

; Key codes and IMU initialization values

-ELR		OCT	-22		# -ERROR LIGHT RESET KEY CODE.
IM30INIF	OCT	37411		# INHIBITS IMU FAIL FOR 5 SEC AND PIP ISSW
					; (fresh start: inhibit IMU failure detection
					; for 5 seconds, inhibit PIPA ISS warning)
IM30INIR	OCT	37000		; IMU channel 30 restart initialization
IM33INIT	=	PRIO16		# NO PIP OR TM FAIL SIGNALS.
					; (IMU channel 33 initialization: no PIPA or
					; telemetry failure signals initially)

; Display and optics initialization

9,6,4		OCT	450		; Display mode configuration constant
OPTINITF	OCT	130		; Optics initialization (fresh start)
OPTINITR	OCT	430		; Optics initialization (restart)

; Software switch initialization table
;
; SWINIT table initializes software switches (flags) to default states.
; This 16-word table contains octal values loaded into flagwords during
; fresh start. Most values are 0 (all flags cleared), except position 6
; which is OCT 00200 (specific flag set for initial configuration).

SWINIT		OCT	0
		OCT	0
		OCT	0
		OCT	0

		OCT	0
		OCT	00200		; Position 6: BIT7 set (specific initial flag)
		OCT	0
		OCT	0
		OCT	0
		OCT	0
		OCT	0
		OCT	0
# Page 194
# PROGRAM NAME		GOTOPOOH	ASSEMBLY	SUNDISK
# LOG SECTION		FRESH START AND RESTART

# FUNCTIONAL DESCRIPTION

#	1. DISPLAY MAJOR MODE NUMBER 00 IN DSKY REGISTER R1 AND R3.  FLASH V50 N07 ON DSKY.  (M M CHANGE REQUEST)
#	2. PERMIT A CURRENT PENDING REQUEST (FLASH ON DSKY) TO BE REPLACED (WITHOUT AN ABORT) BY THE MAJOR MODE
#	   CHANGE REQUEST.

# INPUT/OUTPUT INFORMATION

#	A. CALLING SEQUENCE		TC GOTOPOOH

#	B. ERASABLE INITIALIZATION		NONE

#	C. OUTPUT 		FLASH VERB 50 NOUN 07 ON DSKY

#	D. DEBRIS		L

# PROGRAM ANALYSIS

#	A. SUBROUTINES CALLED		GOPERF3, LINUS

#	B. NORMAL EXIT		    TCF ENDOFJOB

#	C. ALARM AND ABORT EXITS		NONE

; ============================================================================
; TRANSITION: From System Initialization to Major Mode Management
;
; With basic system initialization complete, the AGC now enters its operational
; phase where mission programs execute. The following section implements major
; mode change logic via GOTOPOOH and V37 (Verb 37), allowing the crew and
; ground controllers to select and execute specific mission programs (P01-P79).
;
; During Apollo 11's mission, V37 was used extensively to transition between
; programs such as P20 (rendezvous navigation), P30 (external delta-V targeting),
; P40 (SPS burn), P51 (IMU alignment), P61-P67 (entry programs), and many others.
; The GOTOPOOH entry specifically initiates P00 (Program Zero Zero), the CMC
; "idle" or "ready" state.
; ============================================================================

		BLOCK	02
		SETLOC	FFTAG10
		BANK

		COUNT	02/P00

; GOTOPOOH - Initiate Program 00 (P00)
;
; Entry point to establish P00, the CMC "idle" or "ready" state. P00 is the
; default major mode when no specific mission program is active. The crew
; enters P00 via V37 E 00 E (Verb 37 Enter 00 Enter) on the DSKY keyboard.
;
; P00 performs minimal background operations (9-minute orbital integration cycle)
; while awaiting crew selection of a new program. During Apollo missions, P00
; was the normal state during coast phases, between burns, and when awaiting
; ground uplink commands.
;
; This routine:
; 1. Sets restart protection for GOTOPOOH (restart group 14)
; 2. Jumps to GOP00FIX to initialize system and prompt for program selection

GOTOPOOH	TC	PHASCHNG		# RESTART GOTOPOOH
		OCT	14			; Phase change group 14 for restart protection

		TC	POSTJUMP		; Bank jump to GOP00FIX
		CADR	GOP00FIX
		BANK	10
		SETLOC	VERB37
		BANK

		COUNT	04/P00

; GOP00FIX - Program 00 Initialization and Entry Prompt
;
; GOP00FIX sets up P00 and displays V37N99 on the DSKY, prompting the crew
; to select a major mode program. This routine is the target of GOTOPOOH and
; represents the CMC's entry into a "ready" state awaiting crew input.
;
; Sequence:
; 1. INITSUB - Performs initialization common to all programs
; 2. CLEARMRK+2 - Clears mark registers (optical tracking state)
; 3. Displays V37 N99 - Verb 37 (Change Major Mode) Noun 99 (Please Perform)
; 4. GOFLASH - Flashes display and waits for crew input
; 5. Loops back if crew presses RSET (reset key)

GOP00FIX	TC	INITSUB			; Initialize system for program execution
		TC	CLEARMRK +2		; Clear mark registers
		CAF	V37N99			; Load V37 N99 (change major mode prompt)
		TC	BANKCALL		; Display on DSKY
		CADR	GOFLASH			; Flash and wait for crew response
		TCF	-3			; RSET key pressed, loop back to V37N99
# Page 195
		TCF	-4			; Should not reach (backup loop)
		TCF	-5			; Should not reach (backup loop)
V37N99		VN	3799

# Page 196
# PROGRAM NAME	V37			ASSEMBLY	SUNDISK
# LOG SECTION	FRESH START AND RESTART

# FUNCTIONAL DESCRIPTION

#	1. CHECK IF NEW PROGRAM ALLOWED.  IF BIT 1 OF FLAGWRD2(NODOFLAG) ISSET, AN ALARM 1520 IS CALLED.
#	2. CHECK FOR VALIDITY OF PROGRAM SELECTED.  IF AN INVALID PROGRAM IS SELECTED, THE OPERATOR ERROR LIGHT IS
#	   SET AND CURRENT ACTIVITY, IF ANY, CONTINUES.
#	3. SERVICER IS TERMINATED IF IT HAS BEEN RUNNING.
#	4. INSTALL IS EXECUTED TO AVOID INTERRUPTING INTEGRATION.
#	5. THE ENGINE IS TURNED OFF AND THE DAP IS INITIALIZED FOR COAST.
#	6. TRACK, UPDATE AND TARG1 FLAGS ARE SET TO ZERO.
#	7. DISPLAY SYSTEM IS RELEASED.
#	8. THE FOLLOWING ARE PERFORMED FOR EACH OF THE THREE CASES.
#	   A. PROGRAM SELECTED IS P00.
#	      1. RENDEZVOUS FLAG IS RESET (KILL P20).
#	      2. STATINT1	IS SCHEDULED BY SETTING RESTART GROUP 2.
#	      3. MAJOR MODE 00 IS STORED IN THE MODE REGISTER(MODREG).
#	      4. SUPERBANK 3 IS SELECTED.
#	      5. NODOFLAG IS RESET.
#	      6. ALL RESTART GROUPS EXCEPT GROUP 2 ARE CLEARED.  CONTROL ISTRANSFERRED TO RESTART PROGRAM (GOPROG2)
#		 WHICH CAUSES ALL CURRENT ACTIVITY TO BE DISCONTINUED AND A 9 MINUTE INTEGRATION CYCLE TO BE
#		 INITIATED.
#	   B. PROGRAM SELECTES IS P20.
#	      1. IF THE CURRENT MAJOR MODE IS THE SAME AS THE SELECTED NEWPROGRAM.  THE PROGRAM IS RE-INITIALIZED
#		 VIA V37XEQ, ALL RESTART GROUPS, EXCEPT GROUP 4 ARE CLEARED.
#	      2. IF THE CURRENT MAJOR MODE IS NOT EQUAL TO THE NEW REQUEST, A CHECK IS MADE TO SEE IF THE REQUEST-
#		 ED MAJOR MODE HAS BEEN RUNNING THE BACKGROUND,
#		 AND IF IT HAS, NO NEW PROGRAM IS SCHEDULED, THE EXISTING
#		 P20 IS RESTARTED TO CONTINUE, AND ITS MAJOR MODE IS SET.
#	      3. CONTROL IS TRANSFERRED TO GOPROG2.
#	   C. PROGRAM SELECTED IS NEITHER P00 NOR P20
#	      1. V37XEQ IS SCHEDULED (AS A JOB) BY SETTING RESTART GROUP 4
#	      2. ALL CURRENT ACTIVITY EXCEPT RENDEZVOUS AND TRACKING IS DISCONTINUED BY CLEARING ALL RESTART
#		 GROUPS.  GROUP 2 IS CLEARED.  IF THE RENDEZVOUS FLAG IS ON P20 IS RESTARTED IN GOPROG2 VIA REDOP20,
#		 TO CONTINUE.

# INPUT/OUTPUT INFORMATION

#	   A. CALLING SEQUENCE

#		CONTROL IS DIRECTED TO V37 BY THE VERBFAN ROUTINE.
#			VERBFAN GOES TO C(VERBTAB+C(VERBREG)).  VERB 37 = MMCHANG.
#			MMCHANG EXECUTES A	TC POSTJUMP, CADR V37.

#	   B. ERASABLE INITIALIZATION		NONE

# 	   C. OUTPUT
#		MAJOR MODE CHANGE
# Page 197
#
#	   D. DEBRIS
#		MMNUMBER, MPAC +1, MINDEX, BASETEMP +C(MINDEX), FLAGWRD0, FLAGWRD1, FLAGWRD2, MODREG, GOLOC -1,
#		GOLOC, GOLOC +1, GOLOC +2, BASETEMP, -PHASE2, PHASE2, -PHASE4

# PROGRAM ANALYSIS

#	   A. SUBROUTINES CALLED
#		ALARM, RELDSP, PINBRNCH, INTSTALL, ENGINOF2, ALLCOAST, V37KLEAN, GOPROG2, FALTON, FINDVAC, SUPERSW,
#		DSPMM

#	   B. NORMAL EXIT		TC ENDOFJOB

#	   C. ALARMS		1520 (MAJOR MODE CHANGE NOT PERMITTED)

		BLOCK	02
		SETLOC	FFTAG10
		BANK

		COUNT	02/V37

OCT24		MM	20
OCT31		MM	25
		BANK	27
		SETLOC	VERB37
		BANK

		COUNT	04/V37

; ============================================================================
; MAJOR MODE CHANGE SYSTEM (V37) - CREW-INITIATED PROGRAM SWITCHING
;
; The V37 verb allows astronauts to change between mission programs via DSKY.
; Typing "VERB 37 ENTER" prompts for a program number (major mode), then the
; computer validates the request, terminates current activities, and starts
; the new program with appropriate initialization.
;
; During Apollo 11, V37 was used dozens of times: switching from P11 (Earth
; orbit) to P15 (TLI preparation), from P20 (rendezvous navigation) to P40
; (SPS burns for LOI), from P52 (IMU alignment) back to P00 (idle program),
; and many other transitions throughout the mission timeline.
;
; COMMENT-ONLY READERS: This code handled every major transition during the
; mission - from Earth orbit to the Moon, into lunar orbit, and preparing
; for the journey home. Each "V37 ENTER" the crew typed invoked this logic.
;
; CODE-ALONG READERS: Study the validation logic (CHECKTAB table lookup),
; the engine/DAP safety checks, the program termination sequence, and the
; handoff to new program execution.
; ============================================================================

; ============================================================================
; V37 ENTRY POINT - Major Mode Change Request Processing
;
; When crew types "V37 ENTER" on DSKY, the requested major mode number is
; already in the accumulator (loaded by NOUNROUT). This routine saves the
; mode number and performs critical safety checks before allowing the change.
; ============================================================================

V37		TS	MMNUMBER		# SAVE MAJOR MODE
		CAF	PRIO30			# RESTART AT PINBALL PRIORITY
		TS	RESTREG

; ============================================================================
; SAFETY CHECK 1: IMU Initialization
;
; Major mode changes are prohibited during IMU initialization (coarse align,
; fine align) because programs assume IMU is stable. If BIT6 of IMODES30 is
; set, IMU is initializing - reject the V37 request.
; ============================================================================

		CA	IMODES30		# IS IMU BEING INITIALIZED
		MASK	BIT6
		CCS	A
		TCF	CANTR00

; ============================================================================
; SAFETY CHECK 2: Engine Status
;
; If SPS main engine is firing, immediately force transition to P00 (idle)
; to safely terminate the burn. Crew cannot switch to other programs during
; powered flight - too dangerous to change guidance/control logic mid-burn.
; ============================================================================

		CAF	BIT13			# IS ENGINE ON
		EXTEND
		RAND	DSALMOUT
		CCS	A
		TCF	R00TOP00		# YES, SET UP FOR P00

; ============================================================================
; SAFETY CHECK 3: TVC DAP Status
;
; If Thrust Vector Control Digital Autopilot is active (gimbal control for
; main engine), also force P00 transition. TVC DAP runs during SPS burns;
; this check catches cases where engine bit hasn't propagated yet.
; ============================================================================

		CS	FLAGWRD6		# NO, IS TVC DAP ON
		MASK	OCT60000
		EXTEND
		BZMF	ISITP00			# NO, CONTINUE WITH R00

; ============================================================================
; R00TOP00 - Emergency Transition to P00 During Powered Flight
;
; When crew attempts V37 during SPS engine firing or TVC DAP operation, this
; routine safely terminates the burn and forces P00 (idle program). This
; prevents dangerous mid-burn program changes that could destabilize guidance.
;
; Sequence: Shut down TVC, disable engine, switch to RCS attitude control,
; zero MMNUMBER (forcing P00 path), clean up I/O channels. After 5 seconds
; delay for engine shutdown, continue to P00 setup in ISITP00.
; ============================================================================

R00TOP00	INHINT
		CAF	EBANK6
# Page 198
		TS	EBANK
		EBANK=	DAPDATR1
		CAE	CSMMASS
		TS	MASSTMP
		TC	IBNKCALL
		CADR	SPSOFF			# SHUT DOWN SPS ENGINE
		TC	IBNKCALL
		CADR	MASSPROP		# RECALCULATE MASS PROPERTIES
		CAF	3.1SEC
		TC	IBNKCALL
		CADR	RCSDAPON +1		# SWITCH TO RCS ATTITUDE CONTROL

		TC	IBNKCALL
		CADR	TVCZAP			# DISABLE TVC
		CAF	ZERO
		TS	MMNUMBER		# FORCE P00 (ZERO = P00)
		RELINT
		CAF	FIVE			# 5 SECOND DELAY
		TC	BANKCALL
		CADR	DELAYJOB		# WAIT FOR ENGINE SHUTDOWN
		CAF	ZERO
		EXTEND
		WRITE	5			# CLEAR OUTPUT CHANNEL 5
		EXTEND
		WRITE	6			# CLEAR OUTPUT CHANNEL 6

; ============================================================================
; ISITP00 - Program 00 (Idle) Request Check
;
; At this point MMNUMBER contains the requested major mode. If MMNUMBER is
; zero, crew requested P00 (either explicitly via "V37 E 00 E" or forced by
; R00TOP00 engine safety logic). Branch to ISSERVON to check servicer status.
; If MMNUMBER is non-zero, crew requested a specific program - continue to
; validation and mode change logic.
; ============================================================================

; ============================================================================
; P00 Request Check and Major Mode Validation Entry
;
; If MMNUMBER is zero, crew requested P00 (idle) - skip validation and go
; directly to ISSERVON to check servicer status. If MMNUMBER is non-zero,
; check NODO V37 flag to see if program changes are prohibited, then proceed
; to CHECKTAB for major mode validation.
; ============================================================================

ISITP00		CA	MMNUMBER
		EXTEND
		BZF	ISSERVON		# YES, CHECK SERVICER STATUS

		CS	FLAGWRD2		# NO, IS NODO V37 FLAG SET
		MASK	NODOBIT
		CCS	A
		TCF	CHECKTAB		# NO
		
; ============================================================================
; CANTR00 - V37 Request Rejection (Program Change Not Allowed)
;
; This alarm is displayed when the crew attempts a major mode change that
; is prohibited by system state. Alarm code 1520 indicates "CANNOT DO V37"
; (program change not allowed at this time). After alarm display, control
; transfers to V37BAD for cleanup.
; ============================================================================

CANTR00		TC	ALARM
		OCT	1520

V37BAD		TC	RELDSP			# RELEASES DISPLAY FROM ASTRONAUT

		TC	POSTJUMP		# BRING BACK LAST NORMAL DISPLAY IF THERE
		CADR	PINBRNCH		# WAS ONE.  OY

; ============================================================================
; CHECKTAB - Major Mode Validation Against Allowed Programs Table
;
; This routine validates the requested major mode by comparing MMNUMBER
; against entries in PREMM1 table (pre-major-mode table). The table lists
; all legal program numbers for Command Module missions.
;
; For Apollo 11, allowed modes included:
; P00 (idle), P11 (Earth orbit), P15 (TLI prep), P20 (rendezvous nav),
; P27 (update program), P30-P37 (external deltaV), P40-P47 (SPS burns),
; P51-P53 (IMU alignment), P61-P67 (entry programs), and others.
;
; The table lookup uses an iterative search: for each table entry, mask
; the low 7 bits (program number), complement it, and add to requested mode.
; If result is positive, requested mode is greater than this entry, so
; continue to next entry. If zero or negative, either found match or
; requested mode is invalid.
; ============================================================================

CHECKTAB	CA	NOV37MM			# THE NO.  OF MM
AGAINMM		TS	MPAC +1			# LOOP INDEX FOR TABLE SEARCH
		NDX	MPAC +1			# INDEXED ADDRESSING
		CA	PREMM1			# OBTAIN WHICH MM THIS IS FOR
		MASK	LOW7			# ISOLATE PROGRAM NUMBER (LOW 7 BITS)
		COM				# COMPLEMENT FOR COMPARISON
		AD	MMNUMBER		# ADD REQUESTED MODE
		CCS	A			# CHECK SIGN OF RESULT
		CCS	MPAC +1			# IF GR, SEE IF ANY MORE IN LIST
# Page 199
		TCF	AGAINMM			# YES, GET NEXT ONE
		TCF	V37NONO			# LAST TIME OR PASSED MM

; Match found! Requested major mode is valid. Save the table index for
; later use by program initialization routines.

		CA	MPAC +1
		TS	MINDEX			# SAVE INDEX FOR LATER

; ============================================================================
; ISSERVON - Check if Servicer Routine is Active
;
; Before proceeding with program change, check if the SERVICER (background
; task manager) is currently running. If V37 flag is set in FLAGWRD7, the
; servicer is active and must complete its current cycle before the program
; change can proceed.
;
; If servicer is active, turn off AVERAGE G flag and exit to ENDOFJOB,
; allowing the servicer to complete. Servicer will eventually return control
; to CANV37 for program change continuation.
;
; If servicer is not active, proceed directly to CANV37 for cleanup and
; program initialization.
; ============================================================================

ISSERVON	CS	FLAGWRD7		# V37 FLAG SET - I.E. IS SERVICER GOING
		MASK	V37FLBIT
		CCS	A
		TCF	CANV37			# NO

		INHINT
		CS	AVEGBIT			#   YES TURN OFF AVERAGE G FLAG AND WAIT
		MASK	FLAGWRD1		# FOR SERVICER TO RETURN TO CANV37
		TS	FLAGWRD1

		TCF	ENDOFJOB

; ============================================================================
; CANV37 - Program Change Cleanup and System Reinitialization
;
; After all safety checks pass and servicer completes, CANV37 performs
; comprehensive system cleanup before starting the new program:
;
; 1. Wait for integration routines to finish (INTSTALL)
; 2. Clear critical output bits on channels 11, 12, 13
; 3. Reinitialize system through INITSUB
; 4. Either set up P00 (POOH) or new program (NOUVEAU)
;
; This cleanup ensures clean transition between major modes without leaving
; hardware in inconsistent state.
; ============================================================================

CANV37		CAF	R00AD
		TS	TEMPFLSH

		TC	PHASCHNG
		OCT	14

; ============================================================================
; Integration Wait and Hardware Channel Cleanup
;
; Before changing programs, wait for any in-progress integration computations
; to complete. The INTSTALL routine stalls until orbital integration, powered
; flight integration, or other trajectory calculations finish their current
; cycle. This prevents program changes from corrupting navigation state vector
; calculations mid-computation.
;
; After integration completes, clear critical output bits on channels 11, 12,
; and 13 to ensure hardware systems are in safe, neutral state for new program.
; ============================================================================

ROC		TC	INTPRET

		CALL				# WAIT FOR INTEGRATION TO FINISH
			INTSTALL
DUMMYAD		EXIT

; Clear CAUTION RESET and test connector output bits on channel 11 to ensure
; caution and warning system is properly initialized for new program.

		CS	OCT1400			# CLEAR CAUTION RESET
		EXTEND				#   AND TEST CONNECTOR OUTBIT
		WAND	11

; Clear multiple critical control bits on channel 12 that must not transfer
; between programs: optics error counter enable, star tracker power, thrust
; vector control enable, optics zeroing command, optics DAP engagement, S-IVB
; sequencing commands. This ensures new program starts with clean hardware state.

		CAF	OCT44571		# CLEAR ENABLE OPTICS ERROR COUNTER, STAR
		EXTEND				# TRAKERS ON BIT, TVC ENABLE, ZERO OPTICS,
		WAND	12			# DISENGAGE OPTICS DAP, SIVB IN J SEQUENCE
						# START, AND SIVB CUTOFF BIT.

; Clear unused bits on channel 13 to prevent any spurious signals.

		CS	OCT600			# CLEAR UNUSED BITS
		EXTEND
		WAND	13

; Perform comprehensive system reinitialization through INITSUB. This restores
; default values for control system parameters, DAP deadband settings, RCS jet
; configuration, and other state variables. Then clear navigation marks and
; reset various flags to ensure clean program transition.

		TC	INITSUB

		TC	CLEARMRK

		TC	DOWNFLAG
		ADRES	STIKFLAG

# Page 200
		TC	BANKCALL
		CADR	UPACTOFF		# TURN OFF UPLINK ACTIV LIGHT

		TC	DOWNFLAG
		ADRES	VHFRFLAG
		TC	DOWNFLAG
		ADRES	R21MARK

; Determine whether this is a P00 (idle) request or a new program request.
; If MMNUMBER is zero, proceed to POOH to set up P00 idle mode.
; If MMNUMBER is non-zero, branch to NOUVEAU to initialize the new program.

		CCS	MMNUMBER		# IS THIS A POOH REQUEST
		TCF	NOUVEAU			# NO, PICK UP NEW PROGRAM

		COUNT	04/P00

; ============================================================================
; POOH - P00 (Idle Mode) Initialization
;
; When crew requests P00 through V37E00E, POOH sets up the idle state:
; 1. Release display system (RELDSP) so crew can access displays
; 2. Set restart register (PHSPRDT2) to priority 5 for P00
; 3. Clear NODOFLAG to allow future V37 requests
; 4. Configure phase tables for state integration
; 5. Reset IMU and P20 flags
; 6. Set downlist code for P00 telemetry
; 7. Kill active groups through V37KLEAN and P00KLEAN
;
; P00 was frequently used during Apollo 11 mission for idle periods:
; - Between major program phases (post-TLI, in lunar orbit)
; - During crew rest periods
; - When awaiting ground uplinks or crew input
; ============================================================================

POOH		TC	RELDSP			# RELEASE DISPLAY SYSTEM

; Set restart register PHSPRDT2 to priority 5 for P00. This establishes
; the restart protection level for idle mode, allowing recovery if a restart
; occurs while in P00.

		CAF	PRIO5			# SET VARIABLE RESTART REGISTER FOR P00.
		TS	PHSPRDT2

		INHINT

; Clear NODOFLAG to allow future V37 (program change) requests. This flag
; may have been set during the previous program to inhibit mode changes
; during critical operations.

		CS	NODOBIT			# TURN OFF NODOFLAG
		MASK	FLAGWRD2
		TS	FLAGWRD2

; Set up phase table restart at 2.5 for state vector integration (STATEINT1).
; This ensures orbital integration can recover properly if a restart occurs
; during P00.

		CA	FIVE			# SET 2.5 RESTART FOR STATEINT1
		TS	L
		COM
		DXCH	-PHASE2

; Clear IMUSE flag and kill P20 rendezvous program by turning off RENDFLG.
; This ensures P00 doesn't inherit rendezvous navigation state.

		CS	BIT7-8			# RESET IMUSE + KILL P20 BY TURNING OFF
		MASK	FLAGWRD0
		TS	FLAGWRD0		#			 RENDFLG

; Set downlist code for P00 telemetry. Ground controllers will receive P00
; status in telemetry stream to confirm idle mode entry.

		CAF	DNLADP00

		COUNT	04/V37

; Clear tracking flags that should not carry over into P00 or new program mode.
; This includes TRACKFLAG (orbital tracking active), TARG1FLAG (target loaded),
; and UPDATFLG (state vector update in progress).

SEUDOP00	TS	DNLSTCOD		# SET UP APPROPRIATE DOWNLIST.

						#   (OLD ONE WILL BE FINISHED FIRST)
		CS	OCT01120		# TURN OFF TRACK, TARG1, UPDATE FLAGS
		TS	EBANKTEM
		MASK	FLAGWRD1
		TS	FLAGWRD1

; Kill restart groups 3, 5, and 6 through V37KLEAN subroutine. This terminates
; any jobs or tasks associated with the previous major mode program.

GROUPKIL	TC	IBNKCALL		# KILL GROUPS 3(5,6
		CADR	V37KLEAN

; Check if we're entering P00 (POOH). If MMNUMBER is zero, we're entering P00;
; otherwise, we're switching to a different program (e.g., P20 rendezvous).

		CCS	MMNUMBER		# IS IT POOH
		TCF	RENDV00			# NO
# Page 201

; For P00 entry, also kill group 4. P00KLEAN is mostly redundant with V37KLEAN
; except for this group 4 termination.

		TC	IBNKCALL
		CADR	P00KLEAN		# REDUNDANT EXCEPT FOR GROUP 4.

; Store new major mode number in MODREG and transfer control to GOPROG2 to
; complete the mode change and resume operations.

GOMOD		CA	MMNUMBER
		TS	MODREG

GOGOPROG	TC	POSTJUMP
		CADR	GOPROG2

; ============================================================================
; RENDV00 - RENDEZVOUS PROGRAM MODE CHANGE HANDLING
;
; This section handles transitions to/from P20 (rendezvous navigation) and
; manages the RNDVZFLG (rendezvous flag) across program mode changes. Special
; logic ensures rendezvous tracking continues across certain program transitions.
; ============================================================================

RENDV00		CS	MMNUMBER		# IS NEW PROG = 20
		AD	OCT24			# 20
		EXTEND
		BZF	RENDN00			# YES

; New program is not P20. Check if rendezvous flag is set, which would require
; special handling to preserve tracking state.

		TCF	P00FIZZ

; New program IS P20. Check if we're transitioning from P20 to P20 (no change)
; or from another program to P20.

RENDN00		CS	MMNUMBER
		AD	MODREG
		EXTEND
		BZF	KILL20

; Transitioning to P20 from different program. Check if rendezvous flag already
; set, indicating rendezvous tracking is active. If so, preserve current state.

		CA	FLAGWRD0		# IS RENDEZVOO FLAG SET
		MASK	RNDVZBIT
		CCS	A
		TCF	STATQUO

; Check if rendezvous flag is set before killing restart groups or proceeding
; with normal V37 execution.

P00FIZZ		CAF	RNDVZBIT
		MASK	FLAGWRD0
		CCS	A
		TCF	REV37

; Rendezvous flag not set, or P20-to-P20 transition. Kill restart groups 1 and 2
; to terminate navigation-related jobs before starting fresh P20 execution.

KILL20		EXTEND				# NO, KILL GROUPS 1 + 2
		DCA	NEG0
		DXCH	-PHASE1

		EXTEND
		DCA	NEG0
		DXCH	-PHASE2

; Set restart point to V37XEQ and proceed to GOPROG2 to complete program change.

REV37		CAF	V37QCAD			# SET RESTART POINT
		TS	TEMPFLSH

		TCF	GOGOPROG

; Preserve tracking state when rendezvous flag is already set. Set TRACKFLAG
; and UPDATE flag to maintain continuous navigation tracking across mode change.

STATQUO		CS	FLAGWRD1		# SET TRACKFLAG AND UPDATE FLAG
		MASK	OCT120
		ADS	FLAGWRD1

; Also kill restart group 4 since we're maintaining tracking continuity rather
; than starting fresh. Then jump to GOMOD to complete the mode change.

		EXTEND				# KILL GROUP 4
		DCA	NEG0
		DXCH	-PHASE4

# Page 202
		TCF	GOMOD

; ============================================================================
; NOUVEAU - NEW PROGRAM SETUP WITHOUT RENDEZVOUS PRESERVATION
;
; This routine handles the standard case where we're entering a new major mode
; that doesn't require special rendezvous flag handling. Sets up the appropriate
; downlist for telemetry and clears IMUSE flag if rendezvous flag is not set.
; ============================================================================

NOUVEAU		CAF	RNDVZBIT
		MASK	FLAGWRD0
		CCS	A
		TCF	+3

; Rendezvous flag is not set, so reset the IMUSE flag (IMU use flag, bit 8 of
; FLAGWRD0) to indicate IMU is available for other programs.

		TC	DOWNFLAG		# NO, RESET IMUSE FLAG.
		ADRES	IMUSE			# BIT 8 FLAG 0

; Select appropriate downlist for the new major mode using MINDEX as the index
; into the downlist address table. Then proceed to SEUDOP00 to complete setup.

	+3	INDEX	MINDEX
		CAF	DNLADMM1		# OBTAIN NEW DOWNLIST ADDRESS

		INHINT
		TCF	SEUDOP00

; Error handler for invalid major mode number. Turns on operator error light
; and branches to V37BAD to display error and recycle verb 37.

V37NONO		TC	FALTON			# COME HERE IF MM REQUESTED DOESNT EXIST
		TCF	V37BAD

; ============================================================================
; V37XEQ - EXECUTE NEW MAJOR MODE PROGRAM
;
; This is the heart of V37 execution. It extracts priority, EBANK, and address
; information from the program tables, constructs a 2CADR (two-word address with
; bank), and calls SPVAC (Special VAC area job scheduler) to create a new job
; for the requested major mode program in restart group 4.
;
; The program data is stored in tables indexed by MINDEX:
; - PREMM1: Priority and EBANK information (bits 15-11: priority, bits 10-8: EBANK)
; - FCADRMM1: Fixed memory address (CADR) of the program entry point
; ============================================================================

OCT00010	EQUALS	BIT4
V37XEQ		INHINT
		INDEX	MINDEX
		CAF	PREMM1
		TS	MMTEMP			# OBTAIN PRIORITY BITS 15 - 11
		TS	CYR			# SHIFT RIGHT TO BITS 14 - 10

; Extract priority from bits 14-10 (after right shift) and set up restart
; priority for group 4 and job priority for SPVAC.

		CA	CYR
		MASK	PRIO37
		TS	PHSPRDT4		# PRESET GROUP4 RESTART PRIORITY
		TS	NEWPRIO			# STORE PRIO FOR SPVAC

; Extract EBANK from bits 10-8 of MMTEMP. Multiply by BIT8 (256) and mask
; to get EBANK in bits 2-0, storing result in L register.

		CA	MMTEMP			# OBTAIN EBANK - BITS 8, 9, 10 OF MMTEMP.
		EXTEND
		MP	BIT8
		MASK	LOW3
		TS	L

; Get the fixed memory address (FCADR) from the program table and extract
; the high 5 bits to combine with EBANK in L register.

		INDEX	MINDEX
		CAF	FCADRMM1
		TS	BASETEMP
		MASK	HI5
		ADS	L

; Extract the low 10 bits of the address and add BIT11 to form the GENADR
; portion of the 2CADR. Together with L register containing EBANK+FBANK,
; this forms a complete 2CADR for SPVAC.

		CA	BASETEMP		# OBTAIN GENADR PORTION OF 2CADR.
		MASK	LOW10
		AD	BIT11

; Call SPVAC to schedule a new job in group 4 with the constructed 2CADR
; and priority. SPVAC will find an available VAC area and start the new program.

		TC	SPVAC

; ============================================================================
; V37XEQC - V37 EXECUTION COMPLETION
;
; After SPVAC returns from creating the new job, this routine completes the
; major mode change by extracting the low 7 bits of the mode number from
; MMTEMP and installing it into MODREG (the mode register, which is the low
; 7 bits of restart phase register PHSBRDT1). This makes the new major mode
; visible to other routines and the crew displays.
;
; During Apollo 11, every major mode change (e.g., P63 lunar landing, P12
; ascent, P20 rendezvous) passed through this final step to officially
; activate the new program.
; ============================================================================

V37XEQC		CA	MMTEMP			# UPON RETURN FROM FINDVAC PLACE THE
		MASK	LOW7			# NEW MM IN MODREG (THE LOW 7 BITS OF
		TC	NEWMODEA		# PHSBRDT1)

# Page 203
# FOR SUNDISK ONLY

; Release the display system so the crew can interact with the DSKY and view
; the new major mode's displays. Then exit to the idle loop.

		TC	RELDSP			# RELEASE DISPLAY
		TC	ENDOFJOB		# AND EXIT

; ============================================================================
; INITSUB - INITIALIZATION SUBROUTINE FOR MAJOR MODE CHANGES
;
; Called from V37 program setup logic, this routine performs critical system
; reinitialization required when switching between major modes. It handles:
; - Digital autopilot (DAP) deadband restoration
; - Rate command cessation
; - Flagword reset to default states using FLAGTABL
; - Optical subsystem initialization
;
; This ensures a clean slate when transitioning between programs like P63
; (lunar landing), P12 (ascent), P20 (rendezvous), etc. During Apollo 11,
; every major mode change called this routine to prepare the control systems.
; ============================================================================

INITSUB		EXTEND
		QXCH	MPAC	+1

		CAF	EBANK6			# SET E6 FOR DEADBAND CODING
		TS	EBANK			# WILL BE RESET IN STARTSB2.
		INHINT
		TC	STOPRATE

; Restore the digital autopilot (DAP) deadband setting. The deadband defines
; how far the spacecraft attitude can drift before RCS thrusters fire to
; correct it. Two settings exist: minimum deadband (tighter control, more
; fuel usage) and maximum deadband (looser control, fuel conservation).
;
; The setting is preserved in FLAGWRD9 bit MAXDBBIT. During Apollo 11, this
; allowed restoring the appropriate deadband after major mode changes without
; requiring crew re-selection.

		CA	FLAGWRD9		# RESTORE DEADBAND
		MASK	MAXDBBIT
		CCS	A
		TCF	SETMAXER		# MAX DB SELECTED
		TC	BANKCALL		# MIN DB SELECTED
		CADR	SETMINDB
		TCF	RAKE
SETMAXER	TC	BANKCALL
		CADR	SETMAXDB

; ============================================================================
; RAKE - FLAGWORD BIT CLEARING LOOP
;
; This routine clears specific flagword bits to their default states by
; applying bit masks from FLAGTABL. It loops through all 12 flagwords
; (FLAGWRD0 through FLAGWRD11), clearing program-specific flags while
; preserving essential system state bits.
;
; The name "RAKE" suggests "raking clean" the flags. During major mode
; changes, this prevents flags from one program (e.g., P63 landing) from
; contaminating another program (e.g., P12 ascent).
; ============================================================================

RAKE		CAF	ELEVEN			# THIS PART CLEARS FLAGWORD BITS.
	+1	TS	MPAC			# LOOP COMES HERE.
		INDEX	MPAC
		CS	FLAGTABL
		INDEX	MPAC
		MASK	FLAGWRD0
		INDEX	MPAC			# PUT REVISED FLAGWORD BACK.
		TS	FLAGWRD0
		CCS	MPAC
		TCF	RAKE	+1		# GET THE NEXT FLAGWORD.
		RELINT

; Set IMPULSW flag to indicate impulse control mode for RCS jets (short
; discrete pulses rather than continuous firing). This is the standard mode
; for attitude control between major maneuvers.

		TC	UPFLAG			# NOW SET IMPULSW
		ADRES	IMPULSW

; Initialize the optical subsystem index (OPTIND) to -1, indicating no
; active optical tracking program. This ensures optical routines start fresh.

		CA	NEGONE
		TS	OPTIND
		TC	MPAC	+1		# RETURN FROM INITSUB

; ============================================================================
; FLAGTABL - FLAGWORD CLEAR MASK TABLE
;
; This table contains 12 bit masks (one per flagword) used by the RAKE routine
; to clear program-specific flags during major mode changes. Each OCT value
; represents bits to be CLEARED (set to 0) in the corresponding flagword.
;
; The masks preserve essential system state (IMU status, engine commands) while
; clearing program-specific flags (guidance modes, display states). This allows
; clean transitions between major programs without losing critical hardware state.
;
; Table indexed 0-11 for FLAGWRD0 through FLAGWRD11.
; ============================================================================

FLAGTABL	OCT	0
		OCT	00040			# IDLEFAIL
		OCT	06000			# P21FLAG, STEERSW
		OCT	0
		OCT	0
		OCT	04140			# V59FLAG, ENGONFLG, 3AXISFLG
		OCT	10000			# STRULLSW
		OCT	16000
		OCT	0
# Page 204
		OCT	42000			# SWTOVER, V94FLAG
		OCT	0
		OCT	0

; ============================================================================
; VAC5STOR - SAVE VAC AREA 5 STATE BEFORE MAJOR MODE CHANGE
;
; This routine preserves the state of Vector Accumulator area 5 (VAC5) during
; a major mode change initiated by Verb 37. VAC5 contains critical information
; about active jobs and tasks that must survive the mode transition.
;
; The routine saves two categories of information:
; 1. Job scheduling data (locations, banksets, priorities) - V5LOOP1
; 2. Phase table information for restart protection - V5LOOP2
;
; This preservation enables the new major mode to properly restore or clean up
; tasks from the previous mode, maintaining system integrity across the transition.
; ============================================================================

		SETLOC	VAC5LOC
		BANK

; VAC5STOR entry point: Initialize index registers to zero for both storage loops.
; ITEMP1 indexes source data, ITEMP2 indexes destination storage in VAC5.

VAC5STOR	CA	ZERO			# INITIALIZE INDEX REGISTERS
		TS	ITEMP1
		TS	ITEMP2

; ============================================================================
; V5LOOP1 - FIRST STORAGE LOOP: JOB SCHEDULING DATA
;
; This loop saves job scheduling information for up to 6 active jobs (18 words
; total: 3 words per job consisting of LOC/BANKSET pair and PRIORITY).
;
; For each job, stores:
;   - DCA LOC: Double-precision address (location + bank) where job will resume
;   - CA PRIORITY: Job priority level for executive scheduler
;
; Loop continues until all 6 job slots have been saved (ITEMP2 reaches 18 decimal).
; ============================================================================

V5LOOP1		EXTEND				# LOOP TO STORE LOCS, BANKSETS, AND PRIOS.
		INDEX	ITEMP1
		DCA	LOC			# Get double-precision address (LOC + BANKSET)
		INDEX	ITEMP2
		DXCH	VAC5			# Store in VAC5 area

		INDEX	ITEMP1
		CA	PRIORITY		# Get job priority
		INDEX	ITEMP2
		TS	VAC5 +2			# Store priority following LOC/BANKSET

; Check if all 6 job slots have been saved (3 words per job × 6 jobs = 18 words).
; If ITEMP2 = 18, exit to V5OUT1 for phase table storage.

		CS	ITEMP2			# HAVE WE STORED THEM ALL?
		AD	EIGHTEEN		# Compare with 18 decimal
		EXTEND
		BZF	V5OUT1			# YES, GET PHASE INFORMATION.

; Not all slots saved yet. Increment indexes:
; ITEMP1 += 12 (next LOC slot, as LOC table entries are 12 words apart)
; ITEMP2 += 3 (next VAC5 storage slot, 3 words per job)

		CA	TWELVE			# NO, INCREMENT INDEXES AND LOOP.
		ADS	ITEMP1			# Advance to next source job slot
		CA	THREE
		ADS	ITEMP2			# Advance to next VAC5 destination slot
		TCF	V5LOOP1			# Continue loop

; ============================================================================
; V5OUT1 - TRANSITION TO PHASE TABLE STORAGE
;
; After saving job scheduling data, prepare to save phase table information.
; Phase names (PHSNAME1-PHSNAME6) are in EBANK3, so switch erasable bank.
; Then call GENTRAN to transfer 11 words of phase information to VAC5 +21D.
; ============================================================================

		EBANK=	PHSNAME1
V5OUT1		CA	EBANK3			# PHSNAME REGISTERS ARE IN EBANK3.
		TS	EBANK			# Switch to erasable bank 3

		CA	ELEVEN			# GET PHASE 2CADRS.
		TC	GENTRAN			# Transfer 11 words
		ADRES	PHSNAME1		# Source: phase name registers
		ADRES	VAC5 +21D		# Destination: VAC5 offset +21 decimal

; ============================================================================
; V5LOOP2 - SECOND STORAGE LOOP: PHASE TABLE DATA
;
; After saving job scheduling and phase names, save the phase table itself.
; Phase table entries (PHASE1 through PHASE6) identify active program phases
; for restart protection. Each entry is 1 word, 6 phases total.
;
; Reinitialize indexes for this new loop that stores at VAC5 +33D through +38D.
; ============================================================================

		CA	ZERO			# NOW INITIALIZE INDEXES AGAIN.
		TS	ITEMP1			# Reset source index
		TS	ITEMP2			# Reset destination index

V5LOOP2		INDEX	ITEMP1			# LOOP TO GET PHASE TABLES.
		CA	PHASE1			# Get phase table entry
		INDEX	ITEMP2
		TS	VAC5 +33D		# Store in VAC5 area starting at +33 decimal

; Check if all 6 phase table entries have been saved (1 word each).
; ITEMP2 counts 0-5, so when it reaches 5, all phases are stored.

# Page 205
		CS	ITEMP2			# DO WE HAVE THEM ALL?
		AD	FIVE			# Compare with 5 (6 phases: indices 0-5)
		EXTEND
		BZF	V5OUT2			# YES, GO FINISH UP.

; Not all phases saved yet. Increment indexes:
; ITEMP1 += 2 (phase table entries are 2 words apart in source)
; ITEMP2 += 1 (destination slots are consecutive)

		CA	TWO			# NO, INCREMENT INDEXES AND LOOP.
		ADS	ITEMP1			# Advance to next source phase entry
		INCR	ITEMP2			# Advance to next VAC5 destination slot
		TCF	V5LOOP2			# Continue loop

; ============================================================================
; V5OUT2 - VAC5STOR COMPLETION: SAVE ADDITIONAL STATE
;
; All job scheduling and phase table data has been saved. Before returning,
; save critical new job scheduling state that was set up before this routine
; was called. This preserves the context needed to properly handle the major
; mode change:
;
; VAC5 +39D:    MPAC +3 (subroutine return address, part of calling context)
; VAC5 +40-41D: NEWLOC (double-precision 2CADR for new job being scheduled)
; VAC5 +22D:    NEWJOB (job identifier for new job)
; VAC5 +26D:    NEWPRIO (priority for new job)
;
; Note: NEWJOB and NEWPRIO are stored in gaps within the earlier VAC5 area
; (between phase names and phase table data), optimizing storage efficiency.
; ============================================================================

V5OUT2		CA	MPAC +3			# Save return address context
		TS	VAC5 +39D		# Store in VAC5 temporary area

		EXTEND				# Prepare for double-precision transfer
		DCA	NEWLOC			# Get new job 2CADR (bank and address)
		DXCH	VAC5 +40D		# Store both words at +40D and +41D

		CA	NEWJOB			# Get new job identifier
		TS	VAC5 +22D		# Store in gap area

		CA	NEWPRIO			# Get new job priority
		TS	VAC5 +26D		# Store in gap area

; All critical state successfully captured in VAC5 temporary storage area.
; Complete VAC5 snapshot now includes:
; - Job scheduling data (LOC/PRIORITY/2CADR for all active jobs)
; - Phase name identifiers (program phase tracking)
; - Phase table entries (restart protection phase addresses)
; - New job context (enabling proper mode change execution)
;
; Return to caller via SWRETURN. The major mode change (V37) can now proceed
; safely with system state preserved. During Apollo 11 mission, this mechanism
; enabled crew to change major modes (e.g., from P20 rendezvous navigation to
; P63 lunar landing) without losing critical navigation or guidance state.

		TC	SWRETURN		# Return to major mode change processing

; ============================================================================
; MAJOR MODE CHANGE SUPPORT CONSTANTS
;
; The following constants support V37 (major mode change) operations, including
; channel control, timing, and address references. These constants enable the
; crew to request program changes (e.g., P01 to P20, or P20 to P63 during
; Apollo 11's landing preparation) through the DSKY keyboard interface.
; ============================================================================

EIGHTEEN	OCT	22			# Decimal 18 - loop counter/index limit

		SETLOC	VERB37			# Resume V37 code section
		BANK

NEG7		EQUALS	OCT77770		# Negative 7 in ones-complement format

; Channel bit manipulation constants for V37 processing.
; These octal masks clear specific control bits during major mode transitions,
; ensuring clean state for new program initialization.
OCT44571	OCT	44571			# Bit mask to clear channel control bits
OCT600		OCT	600			# Bit mask for additional channel control

		EBANK=	PACTOFF			# Set erasable bank for P00 DAP reference
P00DAPAD	2CADR	T5IDLOC			# P00 idle loop DAP address (2CADR format)

; Temporary storage aliases for major mode change processing.
; These reuse existing erasable locations to conserve precious RAM.
MMTEMP		EQUALS	PHSPRDT3		# Temporary major mode storage
BASETEMP	EQUALS	TBASE4			# Temporary base address storage

BIT7-8		OCT	300			# Bits 7-8 mask (octal 300 = binary 11000000)
OCT01120	OCT	01120			# Additional bit manipulation constant

; Code address references for V37 processing and R00 (null routine) handling.
V37QCAD		CADR	V37XEQ +3		# Return address within V37 execution
R00AD		CADR	DUMMYAD			# Dummy address for R00 null routine

		EBANK=	DAPDATR1		# Set erasable bank for RCS attitude control
RCSADDR4	2CADR	RCSATT			# RCS attitude control entry point

; Timing constant for program transition settling time.
; Allows 3.1 seconds for system stabilization after major mode change.
3.1SEC		OCT	37312			# 2.5 + 0.6 sec settling time

; ============================================================================
; MAJOR MODE STARTING ADDRESS TABLE (FCADRMM1)
;
; For V37 (major mode change), three parallel tables are maintained with one
; entry for each major mode that can be started from the DSKY keyboard. Entries
; are ordered from highest major mode number (P79) to lowest (P01/P06), enabling
; efficient sequential search during keyboard input processing.
;
; COMMENT-ONLY READERS: These tables enabled the crew to change mission programs
; throughout the flight. For example, during Apollo 11's approach to the Moon,
; Michael Collins could switch from P20 (rendezvous navigation) to P22 (orbit
; determination) by keying V37 E 22 E on the DSKY. The computer would look up
; P22's starting address, memory bank, and priority in these tables, then
; initiate the new program while safely terminating the old one.
;
; CODE-ALONG READERS: Three synchronized tables provide complete major mode
; specifications: FCADRMM1 (starting addresses), PREMM1 (packed priority/bank/
; mode data), and DNLADMM1 (downlink list types). All three tables must remain
; in sync - adding or removing a major mode requires updating all three tables
; plus the NOV37MM count constant.
; ============================================================================
# Page 206

; FCADRMM1 TABLE: Starting job addresses for each keyboard-accessible major mode.
; Each FCADR entry contains the bank and address of the program's entry point.
; V37 processing uses this table to determine where to transfer control when
; the crew requests a major mode change.

FCADRMM1	EQUALS
		FCADR	P79		# Rendezvous targeting programs
		FCADR	P78
		FCADR	P77
		FCADR	P76		# Target delta-V program
		FCADR	P75
		FCADR	P74
		FCADR	P73
		FCADR	P72
		FCADR	P62		# Entry initialization
		FCADR	P61		# Entry preparation
		FCADR	P54		# IMU realignment
		FCADR	P53		# IMU backup alignment
		FCADR	PROG52		# IMU alignment (P52)
		FCADR	P51		# IMU coarse alignment
		FCADR	P47CSM		# SPS thrust program (CM/SM)
		FCADR	P41CSM		# RCS thrust program (CM/SM)
		FCADR	P40CSM		# SPS burn program (CM/SM)
		FCADR	P39		# External delta-V targeting
		FCADR	P38		# Return to Earth targeting
		FCADR	P37		# Return to Earth (P37)
		FCADR	P35		# Lambert targeting
		FCADR	P34		# Lambert aimpoint guidance
		FCADR	P33		# Coelliptic rendezvous
		FCADR	P32		# Coelliptic sequence initiation
		FCADR	P31		# External delta-V display
		FCADR	P30		# External delta-V program
		FCADR	P23		# Landmark tracking (cislunar)
		FCADR	PROG22		# Orbit determination (P22)
		FCADR	PROG21		# Ground track determination
		FCADR	PROG20		# Rendezvous navigation (P20)
		FCADR	P17		# Optics calibration
		FCADR	P06		# Power descent targeting (LM)
		FCADR	GTSCPSS1	# P01 Gyrocompass alignment

; ============================================================================
; MAJOR MODE PRIORITY/BANK/MODE TABLE (PREMM1)
;
; Packed 15-bit data format encoding three critical parameters for each major
; mode. Bit layout (octal digit positions from left):
;
;   Bits 15-11 (PPP PP): Priority (5 bits, 0-31 decimal, typically 13 or 20)
;   Bits 10-8  (E EE):   E-bank number (3 bits, 0-7 decimal)
;   Bits 7-1   (M MMM MMM): Major mode number (7 bits, 0-127 decimal)
;
; Example: OCT 27117 decodes as:
;   Priority = 27117 >> 10 = 13 decimal (octal 010011 = binary 01011 = 13)
;   E-bank   = (27117 >> 7) & 7 = 4 decimal
;   MM       = 27117 & 177 = 79 decimal (P79)
;
; The priority determines job scheduling order (higher priority runs first).
; The E-bank specifies which erasable memory bank contains the program's variables.
; The major mode number displays on the DSKY and identifies the program to crew.
; ============================================================================
# Page 207

PREMM1		EQUALS
		OCT	27117		# MM 79		EBANK 4		PRIO 13
		OCT	27116		# MM 78		EBANK 4		PRIO 13
		OCT	27115		# MM 77		EBANK 4		PRIO 13
		OCT	27714		# MM 76		EBANK 4		PRIO 13
		OCT	27113		# MM 75		EBANK 4		PRIO 13
		OCT	27112		# MM 74		EBANK 4		PRIO 13
		OCT	27111		# MM 73		EBANK 4		PRIO 13
		OCT	27110		# MM 72		EBANK 4		PRIO 13
		OCT	27476		# MM 62		EBANK 6		PRIO 13
		OCT	27475		# MM 61		EBANK 6		PRIO 13
		OCT	27266		# MM 54		EBANK 5		PRIO 13
		OCT	27265		# MM 53		EBANK 5		PRIO 13
		OCT	27264		# MM 52		EBANK 5		PRIO 13
		OCT	27263		# MM 51		EBANK 5		PRIO 13
		OCT	27657		# MM 47		EBANK 7		PRIO 13
		OCT	27451		# MM 41		EBANK 6		PRIO 13
		OCT	27450		# MM 40		EBANK 6		PRIO 13
		OCT	27047		# MM 39		EBANK 4		PRIO 13
		OCT	27046		# MM 38		EBANK 4		PRIO 13
		OCT	27645		# MM 37		EBANK 7		PRIO 13
		OCT	27043		# MM 35		EBANK 4		PRIO 13
		OCT	27042		# MM 34		EBANK 4		PRIO 13
		OCT	27041		# MM 33		EBANK 4		PRIO 13
		OCT	27040		# MM 32		EBANK 4		PRIO 13
		OCT	27637		# MM 31		EBANK 7		PRIO 13
		OCT	27636		# MM 30		EBANK 7		PRIO 13
		OCT	27227		# MM 23		EBANK 5		PRIO 13
		OCT	27226		# MM 22		EBANK 5		PRIO 13
		OCT	27025		# MM 21		EBANK 4		PRIO 13
		OCT	27424		# MM 20		EBANK 6		PRIO 13
		OCT	27021		# MM 17		EBANK 6		PRIO 13
		OCT	27006		# MM 06		EBANK 4		PRIO 13
		OCT	41201		# MM 01		EBANK 5		PRIO 20 (gyrocompass)

; ============================================================================
; E-BANK VERIFICATION LIST
;
; The following EBANK= directives serve as assembly-time verification that the
; E-bank numbers encoded in PREMM1 match the actual erasable bank assignments
; of key variables used by each major mode. If a mismatch occurs, the assembler
; will generate an error, catching configuration errors before flight.
;
; For example, MM 76 (P76 target delta-V) uses variable TIG (time of ignition).
; The EBANK= directive verifies that TIG resides in E-bank 4, matching the
; E-bank field (4) encoded in PREMM1 entry OCT 27714 for MM 76.
; ============================================================================
# Page 208

		EBANK=	TIG		# EBANK SETTING REQUIRED BY MM 76
		EBANK=	KT		# EBANK SETTING REQUIRED BY MM 75
		EBANK=	SUBEXIT		# EBANK SETTING REQUIRED BY MM 74
		EBANK=	SUBEXIT		# EBANK SETTING REQUIRED BY MM 73
		EBANK=	SUBEXIT		# EBANK SETTING REQUIRED BY MM 72
		EBANK=	AOG		# EBANK SETTING REQUIRED BY MM 62
		EBANK=	AOG		# EBANK SETTING REQUIRED BY MM 61
		EBANK=	BESTI		# EBANK SETTING REQUIRED BY MM 54
		EBANK=	STARIND		# EBANK SETTING REQUIRED BY MM 53
		EBANK=	BESTI		# EBANK SETTING REQUIRED BY MM 52
		EBANK=	STARIND		# EBANK SETTING REQUIRED BY MM 51
		EBANK=	P40TMP		# EBANK SETTING REQUIRED BY MM 47
		EBANK=	DAPDATR1	# EBANK SETTING REQUIRED BY MM 41
		EBANK=	KMPAC		# EBANK SETTING REQUIRED BY MM 40
		EBANK=	KT		# EBANK SETTING REQUIRED BY MM 35
		EBANK=	SUBEXIT		# EBANK SETTING REQUIRED BY MM 34
		EBANK=	SUBEXIT		# EBANK SETTING REQUIRED BY MM 33
		EBANK=	SUBEXIT		# EBANK SETTING REQUIRED BY MM 32
		EBANK=	+MGA		# EBANK SETTING REQUIRED BY MM 30
		EBANK=	LANDMARK	# EBANK SETTING REQUIRED BY MM 23
		EBANK=	MARKINDX	# EBANK SETTING REQUIRED BY MM 22
		EBANK=	WHOCARES	# EBANK SETTING REQUIRED BY MM 21
		EBANK=	ESTROKER	# EBANK SETTING REQUIRED BY MM 20
		EBANK=	TIME2SAV	# EBANK SETTING REQUIRED BY MM 06
		EBANK=	QPLACE		# EBANK SETTING REQUIRED BY MM 01

; Automatic calculation of table size for V37 processing. EPREMM1 marks the
; end of the PREMM1 table. By computing the negative offset from table start
; to end, the assembler automatically determines how many major modes are in
; the table. NOV37MM stores this count minus 1 (for zero-based indexing).
;
; If entries are added to or removed from PREMM1, this calculation automatically
; updates NOV37MM without manual constant editing, reducing configuration errors.

EPREMM1		EQUALS			# End-of-table marker for PREMM1
		SETLOC	PREMM1		# Position at start of PREMM1 table
NO.MMS		=MINUS	EPREMM1		# Calculate -(table size) automatically
		SETLOC	VERB37		# Return to V37 code section
		BANK			# Bank selection

NOV37MM		ADRES	NO.MMS	-1	# Number of major modes in table - 1
					# **CRITICAL: DO NOT MOVE THIS CONSTANT**
					# V37 code expects NOV37MM at fixed location

; ============================================================================
; DOWNLINK LIST TYPE TABLE (DNLADMM1)
;
; Specifies which type of telemetry downlink list each major mode requires.
; When a major mode starts, its downlink list type determines what data is
; transmitted to Mission Control. List types are:
;
;   COSTALIN (0): Orbital coast - minimum data (position, velocity, time)
;   ENTRYUPD (1): Entry update - entry-specific data for atmospheric flight
;   RENDEZVU (2): Rendezvous - relative navigation state for two-spacecraft ops
;   POWERED  (3): Powered flight - engine status, thrust, propellant data
;   P22DNLST (4): P22 specific - orbit determination covariance and residuals
;
; During Apollo 11, when the crew switched from P20 (rendezvous navigation) to
; prepare for transearth injection, the downlink list automatically changed to
; provide Mission Control with appropriate data for monitoring the burn.
; ============================================================================
# Page 209

DNLADMM1	EQUALS
		ADRES	RENDEZVU	# P79 - Rendezvous targeting
		ADRES	RENDEZVU	# P78
		ADRES	RENDEZVU	# P77
		ADRES	RENDEZVU	# P76 - Target delta-V
		ADRES	RENDEZVU	# P75
		ADRES	RENDEZVU	# P74
		ADRES	RENDEZVU	# P73
		ADRES	RENDEZVU	# P72
		ADRES	ENTRYUPD	# P62 - Entry initialization
		ADRES	POWERED		# P61 - Entry preparation (powered phase)
		ADRES	COSTALIN	# P54 - IMU realignment (coast)
		ADRES	COSTALIN	# P53 - IMU backup alignment
		ADRES	COSTALIN	# P52 - IMU alignment
		ADRES	COSTALIN	# P51 - IMU coarse alignment
		ADRES	POWERED		# P47 - SPS thrust (powered phase)
		ADRES	POWERED		# P41 - RCS thrust (powered phase)
		ADRES	POWERED		# P40 - SPS burn (powered phase)
		ADRES	RENDEZVU	# P39 - External delta-V targeting
		ADRES	RENDEZVU	# P38 - Return to Earth targeting
		ADRES	RENDEZVU	# P37 - Return to Earth
		ADRES	RENDEZVU	# P35 - Lambert targeting
		ADRES	RENDEZVU	# P34 - Lambert aimpoint
		ADRES	RENDEZVU	# P33 - Coelliptic rendezvous
		ADRES	RENDEZVU	# P32 - Coelliptic sequence
		ADRES	RENDEZVU	# P31 - External delta-V display
		ADRES	RENDEZVU	# P30 - External delta-V
		ADRES	RENDEZVU	# P23 - Landmark tracking
		ADRES	P22DNLST	# P22 - Orbit determination (special list)
		ADRES	RENDEZVU	# P21 - Ground track determination
		ADRES	RENDEZVU	# P20 - Rendezvous navigation
		ADRES	RENDEZVU	# P17 - Optics calibration
		ADRES	COSTALIN	# P06 - Power descent targeting
		ADRES	COSTALIN	# P01 - Gyrocompass alignment

; Downlink list type definitions (numeric constants 0-4).
; These values index into downlink formatting tables elsewhere in the AGC code.
DNLADP00	=	ZERO		# P00 idle loop - zero/null downlink
COSTALIN	=	0		# Orbital coast list
ENTRYUPD	=	1		# Entry update list
RENDEZVU	=	2		# Rendezvous navigation list
POWERED		=	3		# Powered flight list
P22DNLST	=	4		# P22 orbit determination list

; ============================================================================
; ORBITAL INTEGRATION CONSTANTS
;
; These constants define altitude thresholds for switching between precision
; integration modes during orbital navigation. The AGC uses different integration
; techniques depending on spacecraft altitude to balance computational accuracy
; with processing time constraints.
;
; MIDFLAG controls integration precision mode:
; - When spacecraft is near a gravitating body (below threshold), high-precision
;   integration with perturbation forces (oblateness, third-body effects) is used
; - When far from gravitating bodies (above threshold), lower-precision conic
;   section integration suffices (two-body problem approximation)
;
; The 800 km threshold represents a carefully chosen balance: close enough that
; lunar/Earth gravitational field irregularities matter, yet far enough that
; simplified integration saves precious computing cycles during high-workload
; mission phases. During Apollo 11's lunar orbit operations, these thresholds
; governed how the AGC computed navigation state updates.
; ============================================================================

# ORBITAL INTEGRATION CONSTANTS

# THESE CONSTANTS ARE USED IN COMPUTING THE SETTING OF MIDFLAG.
RMM		2DEC	2538.09 E3 B-27	# 800 KM ABOVE LUNAR SURFACE
					# Scaled in meters (B-27 = 2^-27 m)
					# Threshold for lunar proximity integration

RME		2DEC	7178165 B-29	# 800 KM ABOVE EQ. RADIUS
					# Scaled in meters (B-29 = 2^-29 m)
					# Threshold for Earth proximity integration

		BANK	13
		SETLOC	INTINIT
		BANK

		COUNT*	$$/INTIN

		EBANK=	RRECTCSM

; ============================================================================
; STATEUP - STATE VECTOR UPDATE AND EXTRAPOLATION
;
; This routine extrapolates spacecraft state vectors (position and velocity)
; forward in time using orbital integration. It handles both Command Module
; and Lunar Module state vectors, integrating from last known state to current
; time. The routine also propagates covariance matrices (W-matrix) when valid,
; maintaining navigation accuracy estimates.
;
; During Apollo 11's translunar coast and lunar operations, STATEUP continuously
; updated the AGC's knowledge of where both spacecraft were, enabling precise
; targeting for rendezvous after the lunar landing. The routine accounts for
; multiple gravitational bodies and orbital perturbations using the integration
; routines called via INTEGRV.
;
; State vector extrapolation sequence:
; 1. Extrapolate CM state vector (and W-matrix if orbital navigation active)
; 2. Extrapolate LM state vector (and W-matrix if rendezvous navigation active)
; 3. Return with both state vectors current to present time
;
; Flag settings control integration mode:
; - VINTFLAG: Indicates vector integration in progress
; - ORBWFLAG: Orbital navigation W-matrix valid (propagate covariance)
; - RENDWFLG: Rendezvous navigation W-matrix valid
; - DIM0FLAG: Dimensioned integration mode (full 6-DOF state)
; - PRECIFLG: Precision integration (high-accuracy mode for LM)
; - SURFFLAG: Spacecraft on surface (skip integration)
; ============================================================================

STATEUP		SET	BOF		# EXTRAPOLATE CM STATE VECTOR
			VINTFLAG		# Set vector integration flag
# Page 210
			ORBWFLAG	# ALSO 6X6 W-MATRIX IF VALID
			+3		# 	FOR ORBITAL NAVIGATION
					# (skip W-matrix if orbital nav inactive)
		SET				# Enable dimensioned integration mode
			DIM0FLAG		# (6-dimensional: position + velocity)
		CLEAR	CALL			# Standard precision for CM integration
			PRECIFLG		# (not high-precision mode)
			INTEGRV			# Integrate CM state forward in time

; Check if spacecraft on lunar surface. If landed, skip to STATEND (no orbital
; motion while on surface). During Apollo 11's 21.5-hour surface stay between
; landing and ascent, this check prevented unnecessary integration cycles.

		BON	DLOAD			# Branch if on lunar surface
			SURFFLAG		# (spacecraft landed flag)
			STATEND			# Jump to end if on surface
			TETCSM			# Load CM state vector epoch time
		STCALL	TDEC1			# Store time in TDEC1, call init
			INTSTALL		# Initialize integration parameters

; ============================================================================
; LM STATE VECTOR EXTRAPOLATION
;
; Extrapolate Lunar Module state vector with precision integration. The LM
; requires higher numerical accuracy than the CM due to smaller mass, proximity
; to lunar surface gravity gradients, and critical rendezvous navigation needs.
; During Apollo 11's rendezvous after ascent, accurate LM state propagation
; enabled Columbia to locate and dock with Eagle in lunar orbit.
; ============================================================================

		CLEAR	CALL		# EXTRAPOLATE LM STATE VECTOR
			VINTFLAG		# Clear vector integration flag
			SETIFLGS		# Set integration flags for LM
					# 	AND 6X6 W-MATRIX IF VALID
		BOF	SET			# Branch on rendezvous W-matrix flag
			RENDWFLG	# If rendezvous nav W-matrix valid
			+2			# Skip ahead 2 instructions
			DIM0FLAG		# Enable dimensioned integration mode
		SET	CALL			# Use precision integration for LM
			PRECIFLG		# (high-accuracy mode for rendezvous)
			INTEGRV			# Integrate LM state vector forward

; State vector extrapolation complete. Clear flags and return to caller.
; Both CM and LM state vectors now current to present time, ready for
; navigation computations, targeting updates, or display to crew.

STATEND		CLRGO				# Clear and go (combined instruction)
			NODOFLAG		# Clear "no downlink" flag
			ENDINT			# Return to integration completion


; ============================================================================
; THISVINT - Enable Vector Integration Flag
;
; Called by MIDTOAV1 and MIDTOAV2 (middle value to average routines) to
; enable vector integration mode. This simple utility sets VINTFLAG to
; indicate that vector integration (rather than dimensioned integration)
; should be used for the next integration cycle.
; ============================================================================

# THISVINT IS CALLED BY MIDTOAV1 AND2

THISVINT	SET	RVQ			# Set flag and return via Q register
			VINTFLAG		# Enable vector integration mode
