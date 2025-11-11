# Copyright:    Public domain.
# Filename:     ALARM_AND_ABORT.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1493-1496
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-07 RSB	Adapted from Colossus249 file of the same
#				name, and page images. Corrected various
#				typos in the transcription of program
#				comments, and these should be back-ported
#				to Colossus249.
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#       Assemble revision 055 of AGC program Comanche by NASA
#       2021113-051.  April 1, 1969.
#
#       This AGC program shall also be referred to as Colossus 2A
#
#       Prepared by
#                       Massachusetts Institute of Technology
#                       75 Cambridge Parkway
#                       Cambridge, Massachusetts
#
#       under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: ALARM_AND_ABORT.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Program alarm system implementing alarm detection, display, history
;        recording, and abort mode logic. Defines complete alarm code catalog
;        including famous 1201 (executive overflow) and 1202 (waitlist overflow)
;        alarms that occurred during Apollo 11 lunar descent at ~102:38:26 MET.
;        Flight controller Steve Bales made critical "Go" decision allowing
;        landing to continue despite alarms.
;
; COMMENT-ONLY READERS: The alarm system that warned astronauts and Mission
;        Control of problems, including the famous 1202 alarms during lunar
;        landing. Read to understand how the AGC communicated critical issues
;        and how restart protection allowed the landing to continue safely.
;
; CODE-ALONG READERS: Study alarm detection logic, complete alarm code catalog
;        (1201/1202/1203-1207), DSKY alarm display integration, alarm history
;        recording in FAILREG, abort mode logic, and 1202 alarm historical
;        context during Apollo 11 descent. Understand how cooperative design
;        between software (restart protection) and humans (flight controller
;        decisions) enabled mission success despite computer overload.
; ============================================================================
;
; ALARM SYSTEM ARCHITECTURE OVERVIEW
;
; The Apollo Guidance Computer alarm system serves as the critical communication
; channel between the computer and the crew when computational problems arise.
; Unlike modern computers that might simply crash or freeze, the AGC was designed
; to detect, report, and often recover from error conditions while maintaining
; control of the spacecraft.
;
; ALARM CODE CATALOG (Command Module - Comanche055):
;
; 1201 - EXECUTIVE OVERFLOW (Job queue full)
;        Trigger: Too many jobs requested simultaneously via NOVAC/FINDVAC
;        Cause: Computational workload exceeds available core sets (7 maximum)
;        Severity: Non-critical if brief, critical if sustained
;        Crew Response: Monitor; abort mission if alarm persists
;        Historical Context: Occurred during Lunar Module descent in other
;        spacecraft (LM computer). Executive can handle brief overloads via
;        restart protection but sustained overflow indicates serious problem.
;
; 1202 - WAITLIST OVERFLOW (Timer task queue full)
;        Trigger: Too many tasks scheduled in timer-driven WAITLIST (9 maximum)
;        Cause: Excessive interrupt-driven task scheduling, typically from
;        sensor data processing (radar updates, IMU reads, display updates)
;        Severity: Non-critical if brief, critical if sustained
;        Crew Response: Monitor; abort mission if alarm persists
;        Historical Context: THE FAMOUS APOLLO 11 ALARM - At mission elapsed
;        time 102:38:26 on July 20, 1969, during Eagle's descent to the lunar
;        surface, this alarm code flashed on the DSKY multiple times. Caused
;        by landing radar data overloading the WAITLIST task scheduler. Flight
;        controller Steve Bales (GUIDO position) made the critical "Go" decision
;        to continue descent based on backroom support from Jack Garman, who
;        recognized the alarm as a known overload condition that the restart
;        system could handle. The alarm recurred several times but the AGC's
;        restart protection preserved critical navigation state, allowing the
;        landing to proceed. Armstrong landed with approximately 25 seconds of
;        fuel remaining at 102:45:40 MET. This alarm is one of the most famous
;        moments in computing history - software reliability enabling humanity's
;        first lunar landing.
;
; 1203 - WAITLIST TASK SCHEDULING ERROR
;        Trigger: Attempting to schedule more than 9 tasks in WAITLIST
;        Cause: Software attempting to exceed WAITLIST capacity
;        Severity: Critical - indicates programming error or system malfunction
;        Crew Response: Abort mission phase
;        Note: See WAITLIST.agc line 91 for alarm trigger code
;
; 1103 - CCS HOLE FAILURE (Unexpected zero in CCS instruction)
;        Trigger: CCS (Count, Compare, Skip) instruction encounters +0 when
;        expecting positive, negative, or -0 value
;        Cause: Programming error or memory corruption
;        Severity: Critical - indicates serious software or hardware problem
;        Crew Response: Abort mission phase
;        Note: CCS instruction has 4 possible outcomes based on examined value
;
; 0217 - CURTAINS ALARM (Generic critical failure)
;        Trigger: Various critical system failures
;        Severity: Critical - mission abort recommended
;        Crew Response: Evaluate situation; typically abort mission phase
;
; ALARM STORAGE ARCHITECTURE:
;
; The AGC stores up to three simultaneous alarm codes in FAILREG (failure
; register) array. When an alarm occurs, the code is stored in the first
; available FAILREG location (FAILREG, FAILREG+1, or FAILREG+2). The alarm
; system is designed to preserve all alarm codes even during computational
; overload conditions, enabling ground analysis of problems.
;
; ALARM DISPLAY INTEGRATION:
;
; Alarms illuminate the PROG (Program) alarm light on the DSKY via DSPTAB+11D
; manipulation. Priority alarms additionally display V05N09 (Verb 05 Noun 09)
; which prompts the crew to request alarm code display. The crew can then use
; V05N09 to read alarm codes from FAILREG for communication to Mission Control.
;
; RESTART PROTECTION INTEGRATION:
;
; The alarm system works intimately with the AGC's restart protection system
; (see RESTARTS_ROUTINE.agc and FRESH_START_AND_RESTART.agc). When 1201/1202
; alarms occur due to computational overload, the restart system preserves
; critical navigation state and restarts interrupted computations from known
; safe points. This design allowed Apollo 11 to continue descent despite
; multiple 1202 alarms - the alarms warned of the overload condition but the
; restart protection prevented loss of navigation state.
;
; ABORT MODE LOGIC:
;
; The alarm system provides multiple abort entry points (BAILOUT, POODOO,
; ABORT, CCSHOLE) that handle different failure severities. POODOO is the
; "graceful abort" that cleans up system state before aborting. BAILOUT
; provides immediate abort capability. The system stores complete machine
; state via VAC5STOR to enable ground-based debugging of abort conditions.
;

# Page 1493
; ============================================================================
; SECTION: NON-ABORTIVE ALARM REPORTING
;
; This section implements the primary alarm reporting mechanism for non-critical
; conditions. When AGC software detects a problem that requires crew awareness
; but does not immediately threaten mission safety, it calls the ALARM subroutine
; to illuminate the PROG alarm light and record the alarm code in FAILREG.
; ============================================================================
;
# 	THE FOLLOWING SUBROUTINE MAY BE CALLED TO DISPLAY A NON-ABORTIVE ALARM CONDITION. IT MAY BE CALLED
# EITHER IN INTERRUPT OR UNDER EXECUTIVE CONTROL.
#
# 	CALLING SEQUENCE IS AS FOLLOWS:
#
#		TC	ALARM
#		OCT	NNNNN
#					# (RETURNS HERE)
;
; ALARM SUBROUTINE - Report non-abortive alarm condition
;
; The ALARM subroutine can be called from either interrupt context or from
; normal executive-controlled code. It stores the alarm code in the first
; available FAILREG location and illuminates the PROG alarm light on the DSKY.
; The calling sequence places the alarm code (in octal) immediately after the
; TC ALARM instruction, and execution returns to the instruction following the
; alarm code.
;
; During Apollo 11 lunar descent, this mechanism reported the 1202 alarms that
; famously required a split-second GO/NO-GO decision from Mission Control.
; The alarm system performed flawlessly - it warned of the computational
; overload while the restart protection system preserved navigation state,
; enabling the landing to continue safely.
;
		BLOCK	02
		SETLOC	FFTAG7
		BANK

		EBANK=	FAILREG

		COUNT	02/ALARM

# ALARM TURNS ON THE PROGRAM ALARM LIGHT, BUT DOES NOT DISPLAY.
;
; ALARM Entry Point - Non-abortive alarm reporting
;
; This is the main entry point for non-critical alarms. The routine preserves
; the return address, fetches the alarm code from the instruction following
; the TC ALARM call, stores it in FAILREG, and illuminates the PROG light.
;

ALARM		INHINT
;
; Disable interrupts to ensure atomic alarm recording. During Apollo 11's
; 1202 alarms, this ensured that multiple alarm conditions could be recorded
; accurately even under computational overload.

		CA	Q
ALARM2		TS	ALMCADR
;
; ALARM2 entry point (alternate entry skipping Q save) - Store return address
; in ALMCADR (alarm caller address). The Q register contains the address of
; the instruction following TC ALARM, which holds the alarm code.

		INDEX	Q
		CA	0
BORTENT		TS	L
;
; BORTENT entry point (abort entry with alarm code in A register)
; Fetch alarm code from memory location pointed to by Q (indexed addressing)
; and store in L register for processing. For normal ALARM calls, this
; retrieves the OCT alarm code. For abort entries, code is already in A.

PRIOENT		CA	BBANK
 +1		EXTEND
		ROR	SUPERBNK	# ADD SUPER BITS.
		TS	ALMCADR +1
;
; PRIOENT entry point (priority alarm entry)
; Store bank information with super-bank bits in ALMCADR+1. This preserves
; the complete memory context (including bank number) where the alarm occurred,
; enabling ground-based debugging of alarm conditions.

LARMENT		CA	Q		# STORE RETURN FOR ALARM
		TS	ITEMP1
;
; LARMENT entry point (late-arrival alarm entry)
; Preserve return address in ITEMP1 temporary storage. The Q register must be
; saved because subsequent operations may modify it before return.

		CA	LOC
		TS	LOCALARM
		CA	BANKSET
		TS	BANKALRM
;
; Store complete location context: LOC (program counter) in LOCALARM and
; BANKSET (current bank) in BANKALRM. This state information is critical
; for post-mission analysis of when and where alarms occurred. During the
; Apollo 11 1202 alarms, this data helped NASA engineers understand the
; exact computational state when radar data overloaded the WAITLIST.

CHKFAIL1	CCS	FAILREG		# IS ANYTHING IN FAILREG
		TCF	CHKFAIL2	# YES TRY NEXT REG
		LXCH	FAILREG
		TCF	PROGLARM	# TURN ALARM LIGHT ON FOR FIRST ALARM
;
; Check if first FAILREG location is available (contains zero). CCS (Count,
; Compare, Skip) tests the value: if positive, negative, or -0, FAILREG is
; already occupied so try next register. If +0, store alarm code from L
; register into FAILREG via LXCH (exchange L with memory). Then proceed to
; PROGLARM to illuminate the PROG alarm light on DSKY.
;
; The AGC can store up to three simultaneous alarm codes, allowing multiple
; problems to be recorded even during system overload like the Apollo 11
; descent 1202 alarms.

CHKFAIL2	CCS	FAILREG +1
		TCF	FAIL3
		LXCH	FAILREG +1
		TCF	MULTEXIT
;
; Check if second FAILREG location (FAILREG+1) is available. If occupied,
; proceed to check third location. If available, store alarm code and exit
; via MULTEXIT (multiple alarm exit path).

FAIL3		CA	FAILREG +2
# Page 1494
		MASK	POSMAX
		CCS	A
		TCF	MULTFAIL
		LXCH	FAILREG +2
		TCF	MULTEXIT
;
; Check if third FAILREG location (FAILREG+2) is available. Mask with POSMAX
; to clear sign bit, then test with CCS. If occupied (positive value), all
; three FAILREG locations are full - proceed to MULTFAIL (multiple failure
; handler). If available (+0), store alarm code and exit via MULTEXIT.
;
; During Apollo 11 descent, the 1202 alarm occurred multiple times but was
; recorded successfully in available FAILREG locations, allowing Mission
; Control to track the alarm history.

PROGLARM	CS	DSPTAB +11D
		MASK	OCT40400
		ADS	DSPTAB +11D
;
; PROGLARM - Illuminate PROG alarm light on DSKY
;
; This routine turns on the PROG (Program) alarm light by setting bit 8 in
; DSPTAB+11D (display table entry 11 decimal). The light illumination alerts
; the crew that an alarm condition has been detected and logged in FAILREG.
;
; CS (Complement and Skip) inverts DSPTAB+11D, MASK extracts the alarm light
; bit (OCT40400 = bit 8), and ADS (Add to Storage) sets the bit, illuminating
; the light. This is the famous yellow PROG light that flashed during Apollo
; 11's lunar descent, prompting Neil Armstrong's "Give us a reading on the
; 1202 program alarm" call to Mission Control at 102:38:26 MET.
;
; The display system integration (via DSPTAB) ensures alarm visibility even
; during computational overload. The crew sees the light immediately and can
; request alarm code details using Verb 05 Noun 09.

MULTEXIT	XCH	ITEMP1		# OBTAIN RETURN ADDRESS IN A
		RELINT
		INDEX	A
		TC	1
;
; MULTEXIT - Multiple alarm exit path
;
; Restore return address from ITEMP1 to A register, re-enable interrupts via
; RELINT (Release Interrupts), and return to caller via indexed TC (Transfer
; Control). The INDEX A instruction adds the value in A to the address of the
; next instruction, effectively implementing an indirect return.
;
; This clean return path ensures that calling code continues execution normally
; after alarm recording, maintaining spacecraft control even during alarm
; conditions. During Apollo 11's 1202 alarms, this return mechanism allowed
; the guidance and landing computations to continue without interruption.
;

; ============================================================================
; TRANSITION: From FAILREG storage to overflow handling
;
; When all three FAILREG locations are full (indicating three simultaneous
; alarm conditions), the system must handle additional alarms gracefully.
; MULTFAIL stores the new alarm in FAILREG+2 with bit 15 set to indicate
; overflow condition, then returns via normal exit path.
; ============================================================================

MULTFAIL	CA	L
		AD	BIT15
		TS	FAILREG +2

		TCF	MULTEXIT
;
; MULTFAIL - Multiple failure overflow handler
;
; When all three FAILREG locations already contain alarm codes, this handler
; records the new alarm code in FAILREG+2 with bit 15 set as an overflow
; indicator. Load alarm code from L register, add BIT15 (bit 15 = 040000
; octal) to flag the overflow condition, store result in FAILREG+2 
; (overwriting the oldest alarm), then exit via MULTEXIT.
;
; This ensures that even when more than three simultaneous alarms occur (an
; extremely rare condition indicating serious system problems), the most recent
; alarm code is preserved with overflow indication. The bit 15 flag alerts
; ground controllers that alarm history has been lost due to excessive failures.
;
; During normal operation, this path should never execute. If it does, it
; indicates catastrophic system failure requiring immediate abort consideration.

; ============================================================================
; SECTION: PRIORITY ALARM DISPLAY (PRIOLARM)
;
; Priority alarms demand immediate crew attention by automatically displaying
; V05N09 on the DSKY, prompting acknowledgment. Unlike basic alarms that only
; illuminate the PROG light, priority alarms interrupt crew activities and
; require explicit response.
;
; This mechanism was critical during Apollo 11 descent when multiple 1202 alarms
; appeared. Each alarm automatically displayed on the DSKY, forcing Armstrong
; and Aldrin to acknowledge the computer overload while Mission Control evaluated
; whether to abort. The priority display ensured no alarm went unnoticed during
; the most critical 12 minutes of the mission.
; ============================================================================

# PRIOLARM DISPLAYS V05N09 VIA PRIODSPR WITH 3 RETURNS TO THE USER FROM THE ASTRONAUT AT CALL LOC +1,+2,+3 AND
# AN IMMEDIATE RETURN TO THE USER AT CALL LOC +4. EXAMPLE FOLLOWS,
#		CAF	OCTXX		# ALARM CODE
#		TC	BANKCALL
#		CADR	PRIOLARM
#
#		...	...
#		...	...
#		...	...		# ASTRONAUT RETURN
#		TC	PHASCHNG	# IMMEDIATE RETURN TO USER. RESTART
#		OCT	X.1		# PHASE CHANGE FOR PRIO DISPLAY
;
; Calling sequence: Load alarm code in accumulator, call PRIOLARM via BANKCALL.
; PRIOLARM provides four return points: +1, +2, +3 for crew-initiated returns
; during alarm acknowledgment (via ENTR key on DSKY), and +4 for immediate
; program continuation. Caller must provide restart protection via TC PHASCHNG
; after return point +3 to enable recovery if restart occurs during display.

		BANK	10
		SETLOC	DISPLAYS
		BANK

		COUNT	10/DSPLA
;
; PRIOLARM is located in the DISPLAYS bank (Bank 10) to minimize cross-bank
; calls from display interface routines. This placement optimization reduces
; execution time for time-critical alarm presentation.

PRIOLARM	INHINT			# * * * KEEP IN DISPLAY ROUTINES BANK
		TS	L		# SAVE ALARM CODE
;
; Entry point for priority alarm display. Called via BANKCALL with alarm code
; in accumulator. INHINT disables interrupts to ensure atomic alarm processing.
; Alarm code saved in L register for later storage in FAILREG.

		CA	BUF2		# 2 CADR OF PRIOLARM USER
		TS	ALMCADR
		CA	BUF2 +1
		TC	PRIOENT +1	# * LEAVE L ALONE
;
; Retrieve caller's return address from BUF2/BUF2+1 (BANKCALL buffer containing
; calling location CADR) and store in ALMCADR for return processing. Transfer
; control to PRIOENT+1 which processes alarm storage and initiates display.
; Comment "LEAVE L ALONE" emphasizes that L register must preserve alarm code.

-2SEC		DEC	-200		# *** DONT MOVE
;
; Constant -2SEC = -200 decimal = -2 seconds in centiseconds (AGC time unit).
; Used by priority display timing logic to set 2-second display duration.
; Comment "DONT MOVE" indicates this constant must remain at specific address
; for addressing mode compatibility (likely INDEX instruction dependency).

		CAF	V05N09
		TCF	PRIODSPR
;
; Load V05N09 (Verb 05 Noun 09) display code and transfer to PRIODSPR (Priority
; Display Processor) which presents the verb/noun on DSKY and manages crew
; interaction. V05N09 requests crew to acknowledge alarm and optionally display
; alarm code via subsequent verb entry.
;
; Once PRIODSPR completes crew interaction, control returns to caller via one
; of four return points depending on crew response and program timing requirements.

; ============================================================================
; SECTION: ABORT ROUTINES (BAILOUT and POODOO)
;
; These routines handle catastrophic system failures requiring abort and restart.
; Unlike recoverable alarms, abort conditions indicate the AGC cannot safely
; continue current program execution and must initiate emergency shutdown sequence.
;
; BAILOUT: Program-initiated abort (software detects unrecoverable condition)
; POODOO: Critical abort with system state preservation for debugging
;
; During Apollo missions, abort conditions were extremely rare but potentially
; mission-threatening. The abort system provided last-resort protection when
; guidance computer failures made continued operation unsafe. Fortunately,
; these routines were never invoked during actual Apollo 11 lunar landing.
; ============================================================================

# Page 1495
		BLOCK	02
		SETLOC	FFTAG13
		BANK

		COUNT	02/ALARM
;
; Abort routines located in fixed-fixed memory block for maximum reliability.
; Must be accessible from any bank during emergency conditions.

BAILOUT		INHINT
		CA	Q
		TS	ALMCADR
;
; BAILOUT - Program-initiated abort entry point
;
; Called when software detects unrecoverable error requiring immediate program
; termination and restart attempt. INHINT disables interrupts to prevent
; additional failures during abort processing. Q register (return address)
; saved in ALMCADR to record abort location for post-abort analysis.

		TC	BANKCALL
		CADR	VAC5STOR
;
; Call VAC5STOR to preserve current state of vector accumulator (VAC area)
; erasable memory. This snapshot enables ground controllers and onboard crew
; to diagnose failure cause after restart. Preserving computational state is
; critical for understanding what went wrong and determining if safe to continue.

		INDEX	ALMCADR
		CAF	0
		TC	BORTENT
;
; Retrieve abort code from location following caller's TC BAILOUT instruction
; (indexed by ALMCADR = return address), load into accumulator, transfer to
; BORTENT (abort entry) which processes abort code storage and initiates
; emergency restart sequence.

OCT40400	OCT	40400
;
; Constant OCT40400 = 040400 octal = bit 14 and bit 8 set.
; Used for masking DSPY register bits during alarm light control.
; Bit 14 controls PROG (Program Alarm) light on/off state.

		INHINT
WHIMPER		CA	TWO
		AD	Z
		TS	BRUPT
		RESUME
		TC	POSTJUMP	# RESUME SENDS CONTROL HERE
		CADR	ENEMA
;
; WHIMPER - Abort completion and restart initiation
;
; After abort processing completes, WHIMPER prepares for emergency restart.
; Computation: Load constant TWO, add Z register (program counter), store in
; BRUPT (break point) to preserve restart location. RESUME re-enables interrupts
; briefly, then POSTJUMP transfers to ENEMA routine which performs actual
; restart sequence and Fresh Start initialization.
;
; This multi-step sequence ensures orderly shutdown even during catastrophic
; failure. By preserving program counter +2 in BRUPT, restart logic can
; potentially resume from a safe restart point rather than cold start, preserving
; mission timeline where possible.

		SETLOC	FFTAG7
		BANK
;
; POODOO routine placed in FFTAG7 location for accessibility from core alarm
; processing paths. This location ensures minimal bank switching overhead
; during emergency abort conditions.

POODOO		INHINT
		CA	Q
		TS	ALMCADR
;
; POODOO - Critical abort with state preservation
;
; Entry point for severe system failures requiring abort with maximum state
; information preserved. Name "POODOO" is programmer's slang (from Yiddish
; "putz," meaning "useless person") indicating system has become non-functional
; and must restart. More severe than BAILOUT, POODOO implies fundamental
; system integrity compromise.
;
; INHINT disables interrupts immediately. Return address from Q register saved
; in ALMCADR to record exact failure point for post-restart debugging analysis.

		TC	BANKCALL
		CADR	VAC5STOR	# STORE ERASABLES FOR DEBUGGING PURPOSES.
;
; Call VAC5STOR via BANKCALL to preserve complete vector accumulator area state.
; Comment emphasizes debugging purpose: captured state enables ground controllers
; to analyze failure conditions after restart, potentially determining root cause
; and whether mission can safely continue.
;
; This preservation is critical for post-abort decision making. During lunar
; descent, any POODOO abort would have forced immediate ascent engine ignition
; (abort to orbit). Preserved state helps Mission Control determine if second
; landing attempt is safe or if mission must return to Earth.

		INDEX	ALMCADR
		CAF	0
ABORT2		TC	BORTENT
;
; Retrieve abort code from caller's inline constant (indexed by saved return
; address in ALMCADR), load into accumulator. Transfer to BORTENT for abort
; processing. ABORT2 provides alternate entry point when abort code is already
; in accumulator (bypassing the indexed CAF 0 instruction).
;
; Abort code indicates failure type to ground controllers and restart logic.
; Different abort codes trigger different recovery strategies or indicate
; mission-ending failures requiring immediate crew safety actions.

OCT77770	OCT	77770		# DONT MOVE
;
; Constant OCT77770 = 077770 octal = 32760 decimal = all bits except bit 15.
; Used for masking operations in alarm and abort processing. Comment "DONT MOVE"
; indicates this constant must remain at specific address for addressing
; compatibility (likely referenced via fixed address or INDEX dependency).

		CA	V37FLBIT	# IS AVERAGE G ON
		MASK	FLAGWRD7
		CCS	A
		TC	WHIMPER -1	# YES.  DONT DO POODOO.  DO BAILOUT.
;
; POODOO abort protection logic: Check if Average G computation is active
; (V37FLBIT flag in FLAGWRD7). If Average G is running (flag set), abort
; via WHIMPER-1 (bailout path) instead of full POODOO sequence. This prevents
; POODOO abort from disrupting critical navigation state updates during
; powered flight phases where Average G provides backup guidance.
;
; Rationale: During engine burns, Average G (accelerometer-based velocity
; integration) provides redundant navigation state. POODOO's aggressive state
; clearing could corrupt this backup data. BAILOUT preserves more state,
; enabling safer restart during powered flight.

		TC	DOWNFLAG
		ADRES	STATEFLG
;
; Clear STATEFLG (state vector flag) indicating navigation state is no longer
; valid after abort. Prevents subsequent navigation routines from using
; corrupted position/velocity data.

		TC	DOWNFLAG
# Page 1496
		ADRES	REINTFLG
;
; Clear REINTFLG (reintegration flag) indicating orbital integration must
; restart from fresh initial conditions after abort. Previous integration
; history is discarded.

		TC	DOWNFLAG
		ADRES	NODOFLAG
;
; Clear NODOFLAG (node flag) indicating orbital node calculations must be
; recomputed. Abort invalidates any in-progress orbital geometry computations.

		TC	BANKCALL
		CADR	MR.KLEAN
;
; Call MR.KLEAN (Mr. Clean) routine which performs comprehensive erasable
; memory cleanup, clearing job queues, resetting program counters, and
; restoring AGC to known safe state. Name is humorous reference to cleaning
; product mascot, emphasizing thorough memory sanitization before restart.
;
; MR.KLEAN prepares AGC for Fresh Start sequence, ensuring no corrupted data
; persists into restarted programs. Critical for preventing cascade failures
; where abort condition propagates through restart.

		TC	WHIMPER
;
; Transfer to WHIMPER routine which completes abort sequence and initiates
; restart via ENEMA (restart entry point). After MR.KLEAN sanitizes memory
; and flags are cleared, WHIMPER preserves minimal restart state and transfers
; control to Fresh Start initialization.

; ============================================================================
; SECTION: SPECIALIZED ABORT ENTRY POINTS
;
; CCSHOLE and CURTAINS provide specialized abort and alarm entry points for
; specific failure modes requiring unique handling sequences.
; ============================================================================

CCSHOLE		INHINT
		CA	Q
		TS	ALMCADR
		TC	BANKCALL
		CADR	VAC5STOR
		CA	OCT1103
		TC	ABORT2
;
; CCSHOLE - CCS (Count, Compare, and Skip) instruction failure abort
;
; Specialized abort entry point for failures detected during CCS instruction
; execution. CCS is AGC's conditional branch instruction used for arithmetic
; sign testing and looping control. Failures here indicate severe computational
; integrity problems.
;
; Sequence: INHINT disables interrupts, Q register saved in ALMCADR, VAC5STOR
; preserves vector accumulator state, abort code 1103 loaded, transfer to
; ABORT2 for standard abort processing.
;
; Name "CCSHOLE" (CCS-hole, like "hellhole") indicates this is an escape route
; from catastrophic CCS instruction failures. Such failures are extremely rare
; and indicate hardware malfunction or memory corruption.

OCT1103		OCT	1103
;
; Abort code 1103 (octal) indicates CCS instruction failure detected.
; Ground controllers seeing this code would know computational logic integrity
; is compromised, potentially requiring hardware inspection before mission
; continuation.
CURTAINS	INHINT
		CA	Q
		TC	ALARM2
OCT217		OCT	00217
		TC	ALMCADR		# RETURN TO USER
;
; CURTAINS - Fatal alarm indicating mission-ending failure
;
; Specialized alarm entry point for catastrophic failures where recovery is
; unlikely but immediate abort is not appropriate. Name "CURTAINS" is slang
; meaning "the end" (as in theatrical curtain closing on final act), indicating
; this alarm represents mission termination scenario.
;
; Unlike POODOO/BAILOUT which attempt restart, CURTAINS issues alarm 217 (octal)
; and returns to caller, allowing program to continue briefly for orderly
; shutdown or crew takeover. This provides opportunity for manual control
; transition during critical flight phases.
;
; Sequence: INHINT disables interrupts, Q register loaded to accumulator,
; transfer to ALARM2 (alarm processing with code in next location), alarm
; code 217 follows, return to caller via saved address in ALMCADR.
;
; During lunar landing, a CURTAINS alarm would have prompted immediate manual
; takeover by Armstrong or abort decision by Mission Control. The alarm allows
; brief continued execution for crew situational awareness before committing
; to abort or manual control.
;
; Alarm code 217 (octal) = 143 (decimal) indicates terminal system failure.
; This code would trigger maximum priority response from Mission Control and
; crew, potentially initiating emergency abort procedures.

; ============================================================================
; SECTION: ALTERNATE ALARM ENTRY POINTS
;
; DOALARM, VARALARM, and ABORT provide specialized alarm entry points for
; various calling scenarios throughout the AGC programs.
; ============================================================================

DOALARM		EQUALS	ENDOFJOB
;
; DOALARM - Job completion with alarm checking
;
; DOALARM equates to ENDOFJOB, providing semantically meaningful name when
; terminating job execution with potential alarm condition. When program job
; completes but has queued alarm, using TC DOALARM clarifies intent versus
; generic TC ENDOFJOB call.
;
; This equivalence allows clean job termination with alarm processing integrated
; into executive scheduler. Jobs calling DOALARM are removed from job queue
; while any pending alarms are processed before next job selection.

# CALLING SEQUENCE FOR VARALARM
#
#		CAF	(ALARM)
#		TC	VARALARM
#
# VARALARM TURNS ON PROGRAM ALARM LIGHT BUT DOES NOT DISPLAY
;
; VARALARM calling sequence: Load alarm code constant into accumulator via CAF,
; transfer control to VARALARM. Unlike ALARM which displays alarm code on DSKY,
; VARALARM only illuminates PROG (Program Alarm) light without display update.
;
; Use case: Non-critical alarms where crew awareness (via lit PROG light) is
; sufficient without interrupting current DSKY display. Crew can subsequently
; query alarm status via verb commands when convenient.

VARALARM	INHINT

		TS	L		# SAVE USERS ALARM CODE
;
; VARALARM - Variable alarm with light only (no display)
;
; Entry point for alarms that illuminate PROG light but do not force DSKY
; display update. INHINT disables interrupts for atomic alarm processing.
; Alarm code from accumulator saved in L register for subsequent storage.

		CA	Q		# SAVE USERS Q
		TS	ALMCADR
;
; Save return address from Q register into ALMCADR, enabling return to caller
; after alarm processing completes. Preserves calling context.

		TC	PRIOENT
OCT14		OCT	14		# DONT MOVE
;
; Transfer to PRIOENT (priority entry) which handles alarm code storage in
; FAILREG and PROG light activation. Constant OCT14 (octal 14 = decimal 12)
; follows for addressing mode compatibility. Comment "DONT MOVE" indicates
; this constant's address is critical (likely INDEX instruction dependency).

		TC	ALMCADR		# RETURN TO USER
;
; Return control to caller via saved return address in ALMCADR. Program
; continues execution with PROG light illuminated, alerting crew to alarm
; condition without disrupting current operations.

ABORT		EQUALS	BAILOUT		# *** TEMPORARY UNTIL ABORT CALLS OUT
;
; ABORT - Temporary synonym for BAILOUT
;
; ABORT equates to BAILOUT routine, providing alternate name during code
; development. Comment "TEMPORARY UNTIL ABORT CALLS OUT" indicates this
; equivalence is placeholder until dedicated ABORT routine is implemented
; with specialized abort processing logic.
;
; In operational Apollo 11 code, this equivalence remains, meaning any
; TC ABORT instruction transfers to BAILOUT abort sequence with state
; preservation and restart attempt.
;
; ============================================================================
; END OF ALARM_AND_ABORT.agc DOCUMENTATION
;
; This file implements the complete alarm and abort system for the Apollo
; Guidance Computer. Every alarm displayed during Apollo 11's mission - including
; the famous 1201 and 1202 alarms during lunar descent - was processed by the
; routines documented above.
;
; The alarm system's robust design allowed Armstrong and Aldrin to continue
; descent despite computer overload alarms, ultimately achieving successful
; landing at Tranquility Base on July 20, 1969 at 102:45:40 mission elapsed time.
;
; Steve Bales, GUIDO flight controller, made the critical "Go" decision allowing
; landing continuation when 1202 alarms appeared. The alarm system's ability to
; preserve navigation state through restart protection enabled this decision.
;
; This code represents one of the most critical safety systems in aerospace
; history - the guardian that protected both crew and mission while enabling
; humanity's first lunar landing.
; ============================================================================
