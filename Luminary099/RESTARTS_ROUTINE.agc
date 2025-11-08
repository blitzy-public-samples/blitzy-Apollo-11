# Copyright:	Public domain.
# Filename:	RESTARTS_ROUTINE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1303-1309
# Mod history:	2009-05-27 OH	Transcribed from page images.
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
; FILE: RESTARTS_ROUTINE.agc
; MODULE: Restart Recovery System
; MISSION PHASE: All phases - critical during descent/landing/ascent
;
; TL;DR: Implements the AGC's restart recovery mechanism that restores task
;        state and resumes program execution after power transients or system
;        overload conditions (such as the famous 1202 alarm). Reads phase
;        information from restart tables, determines job type (JOB/WAITLIST/
;        LONGCALL), calculates timing for deferred tasks, and safely resumes
;        interrupted operations without losing mission-critical computations.
;
; COMMENT-ONLY READERS: This is the safety net that saved Apollo 11's landing.
;        When the computer overloaded during descent, this routine recovered
;        the guidance calculations and allowed the mission to continue.
; CODE-ALONG READERS: Study the state machine logic that interprets restart
;        table entries, handles bank-switched memory access, computes delta
;        times for task scheduling, and coordinates with EXECUTIVE/WAITLIST.
; ============================================================================

# Page 1303
		BANK	01
		SETLOC	RESTART
		BANK

		EBANK=	PHSNAME1	# GOPROG MUST SWITCH TO THIS EBANK

; ============================================================================
; RESTART ENTRY POINT - Main Recovery Routine
;
; The AGC restart system saved Apollo 11 from aborting the lunar landing.
; During powered descent on July 20, 1969, at approximately 102:38:26 mission
; elapsed time, the computer triggered program alarm 1202 due to executive
; job queue overflow. This routine executed repeatedly during the descent,
; each time restoring critical guidance and navigation tasks from their
; saved states in the restart tables, allowing the landing to continue.
;
; This entry point is called by FRESH_START_AND_RESTART after a restart
; condition is detected (power transient, BAILOUT, or program alarm). The
; routine must determine what type of task was interrupted and restart it
; appropriately - whether it was a JOB, WAITLIST call, or LONGCALL.
; ============================================================================

		COUNT*	$$/RSROU
RESTARTS	CA	MPAC +5		# GET GROUP NUMBER -1
; Entry point from restart initialization. MPAC+5 contains the restart group
; number (0-6) which indexes into the restart tables. Each group corresponds
; to major mission program phases requiring protection.

		DOUBLE			# SAVE FOR INDEXING
		TS	TEMP2G
; Double the group number and store for indexing into double-precision
; restart table entries. TEMP2G now contains (group*2) for table lookup.

		CA	PHS2CADR	# SET UP EXIT IN CASE IT IS AN EVEN
		TS	TEMPSWCH	# TABLE PHASE
; Prepare default return address for even-numbered restart phases which
; have two-part table entries. TEMPSWCH holds the continuation address.

		CA	RTRNCADR	# TO SAVE TIME ASSUME IT WILL GET NEXT
		TS	GOLOC +2	# GROUP AFTER THIS
; Optimistically set up return to process next restart group. If this
; assumption is wrong, GOLOC+2 will be overwritten with correct address.

		CA	TEMPPHS
		MASK	OCT1400
; TEMPPHS contains the restart phase information encoding the task type.
; Bit pattern 1400 (octal) distinguishes variable restarts from table restarts.
; Variable restarts encode task info directly in phase; table restarts use
; indirect lookup through the CADRTAB/PRDTTAB restart tables.

		CCS	A		# IS IT A VARIABLE OR TABLE RESTART
		TCF	ITSAVAR		# IT:S A VARIABLE RESTART

; ============================================================================
; TRANSITION: From restart type detection to handling table restarts
;
; Table restarts fall through here when bit 1400 is clear. These restarts
; use indirect addressing through restart table arrays to determine the
; task to resume. Special case: phase X.1 restarts the DSKY display system.
; ============================================================================

GETPART2	CCS	TEMPPHS		# IS IT AN X.1 RESTART
		CCS	A
		TCF	ITSATBL		# NO, ITS A TABLE RESTART
; Double CCS checks for X.1 restart phase encoding (phase value = 1).
; X.1 restarts occur after fresh starts to initialize the crew display system.

		CA	PRIO14		# IT IS AN X.1 RESTART, THEREFORE START
		TC	FINDVAC		# THE DISPLAY RESTART JOB
		EBANK=	LST1
		2CADR	INITDSP
; Special handling for display system restart. INITDSP (priority 14 job)
; reinitializes the DSKY display interface after power-up or restart,
; ensuring the crew can see program numbers, nouns, verbs, and data.
; This is critical for crew situational awareness during all mission phases.

		TC	RTRNCADR	# FINISHED WITH THIS GROUP, GET NEXT ONE

; ============================================================================
; VARIABLE RESTART HANDLING
;
; Variable restarts encode all restart information directly in the phase
; value, without requiring table lookup. The phase encoding specifies whether
; the interrupted task was a JOB, WAITLIST call, or LONGCALL, and provides
; the target address and timing information for resumption.
; ============================================================================

ITSAVAR		MASK	OCT1400		# IS IT TYPE B ?
		CCS	A
		TCF	ITSLIKEB	# YES,IT IS TYPE B
; Type B variable restarts have bit 1400 set and require special handling
; for task priority and scheduling through the EXECUTIVE job queue.

		EXTEND			# STORE THE JOB (OR TASK) 2CADR FOR EXIT
		NDX	TEMP2G
		DCA	PHSNAME1
		DXCH	GOLOC
; Load the 2CADR (bank and address pair) of the interrupted task from the
; PHSNAME1 table indexed by restart group. This double-precision address
; is where execution will resume after restart recovery completes.
; GOLOC now contains the continuation point for this restart group.

		CA	TEMPPHS		# SEE IF THIS IS A JOB, TASK, OR A LONGCAL
		MASK	OCT7
		AD	MINUS2
		CCS	A
		TCF	ITSLNGCL	# ITS A LONGCALL
; The low 3 bits of TEMPPHS encode the task type:
; 0 = LONGCALL (longterm subroutine call)
; 1 = WAITLIST task (timer-driven deferred execution)
; 2 = JOB (executive-scheduled task requiring VAC area)
; Subtract 2 and check sign to determine which type this restart represents.

RTRNCADR	TC	SWRETURN	# CANT GET HERE
# Page 1304
		TCF	ITSAWAIT
; CCS branches: Negative result -> WAITLIST task

		TCF	ITSAJOB		# ITS A JOB
; CCS branches: Zero result -> JOB requiring VAC area allocation

; ============================================================================
; WAITLIST TASK RESTART
;
; WAITLIST tasks are timer-driven operations scheduled through the delta-time
; queue managed by T4RUPT. During descent, WAITLIST handled periodic updates
; for guidance, navigation, and throttle control. When a restart occurred
; during the 1202 alarm, these time-critical tasks needed careful scheduling
; to resume at the correct time relative to the mission timeline.
; ============================================================================

ITSAWAIT	CA	WTLTCADR	# SET UP WAITLIST CALL
		TS	GOLOC -1
; WTLTCADR contains the address of the WAITLIST scheduling routine.
; Store in GOLOC-1 so the task will be submitted to WAITLIST for
; timer-driven execution rather than immediate job queue insertion.

		NDX	TEMP2G		# DIRECTLY STORED
		CA	PHSPRDT1
; Load the phase restart time from PHSPRDT1 table, indexed by restart group.
; This value encodes either:
; - Direct delta-time value (if positive)
; - Zero for immediate execution
; - Negative for indirect time lookup
; - Special encoding for phase-relative timing

TIMETEST	CCS	A		# IS IT AN IMMEDIATE RESTART
		INCR	A		# NO.
		TCF	FINDTIME	# FIND OUT WHEN IT SHOULD BEGIN
; Positive value: This WAITLIST task should execute after a computed delay.
; Branch to FINDTIME to calculate when the task should resume relative to
; current mission time, accounting for elapsed time during the restart.

		TCF	ITSINDIR	# STORED INDIRECTLY
; Negative value: Time is stored indirectly through another memory location.
; This allows dynamic timing based on mission phase state variables.

		TCF	IMEDIATE	# IT WANTS AN IMMEDIATE RESTART
; Zero value: Execute this WAITLIST task immediately with minimal delay.
; Used for time-critical operations that must resume as soon as possible.

; ============================================================================
; INDIRECT TIME LOOKUP (Fixed-Fixed Memory Required)
;
; This section must execute in fixed-fixed memory (common bank) because it
; accesses erasable memory that may be in a different E-bank than the current
; one. Bank-switched memory access requires careful preservation of the BB
; register which controls E-bank selection.
; ============================================================================

# ***** THIS MUST BE IN FIXED FIXED *****

		BLOCK	02
		SETLOC	FFTAG2
		BANK

		COUNT*	$$/RSROU
ITSINDIR	LXCH	GOLOC +1	# GET THE CORRECT E BANK IN CASE THIS IS
		LXCH	BB		# SWITCHED ERRASIBLE
; Save GOLOC+1 and BB register. BB controls erasable bank selection for
; switched erasable memory access. This swap allows safe bank switching
; to access timing data that may reside in a different erasable bank.

		NDX	A		# GET THE TIME INDIRECTLY
		CA	1
; A register contains negative address pointer for indirect timing lookup.
; Index through this address (plus 1) to fetch the actual delta-time value
; stored elsewhere in erasable memory. This indirection allows mission-phase-
; dependent timing based on computed values rather than fixed constants.

		LXCH	BB		# RESTORE THE BB AND GOLOC
		LXCH	GOLOC +1
; Restore original BB (erasable bank) and GOLOC+1 registers after the
; bank-switched memory access completes. System state is now consistent.

		TCF	FINDTIME	# FIND OUT WHEN IT SHOULD BEGIN

# ***** YOU MAY RETURN TO SWITCHED FIXED *****

		BANK 	01
		SETLOC	RESTART
		BANK

; ============================================================================
; DELTA-TIME COMPUTATION FOR WAITLIST SCHEDULING
;
; FINDTIME calculates when a WAITLIST task should execute by comparing the
; saved time base (TBASE1) from when the task was originally scheduled to
; the current mission time (TIME1). The difference determines how much time
; elapsed during the restart, allowing proper rescheduling with compensated
; delta-time values.
; ============================================================================

		COUNT*	$$/RSROU
FINDTIME	COM			# MAKE NEGITIVE SINCE IT WILL BE SUBTRACTD
		TS	L		# AND SAVE
; Complement the restart time value (making it negative) and save in L
; register. This value will be subtracted from the computed elapsed time
; to determine the remaining delay before task execution.

		NDX	TEMP2G
		CS	TBASE1
		EXTEND
		SU	TIME1
; Calculate elapsed time since task was scheduled: (current TIME1 - TBASE1).
; TBASE1 is the time base when the WAITLIST task was originally queued.
; TIME1 is the current mission clock incremented every 10 milliseconds.
; The difference shows how much time passed during the restart period.

		CCS	A
		COM
; Handle time overflow and sign correction. AGC time registers wrap at
; 2^14 centiseconds (~27 minutes). CCS detects overflow and complements
; the result to maintain correct time difference magnitude.

# Page 1305
		AD	OCT37776
		AD	ONE
		AD	L
; Adjust the elapsed time and combine with the saved original delta-time
; from L register. OCT37776 provides scaling adjustment for proper time
; resolution in centiseconds (0.01 second units).

		CCS	A
		CA	ZERO
		TCF	+2
		TCF	+1
; Check if computed delta-time is positive. If task deadline has passed
; (negative or zero result), force immediate execution by loading zero.
; Otherwise preserve the computed future scheduling time.

IMEDIATE	AD	ONE
		TC	GOLOC -1
; Add 1 centisecond minimum delay and submit task to WAITLIST scheduler
; through GOLOC-1 (contains WTLTCADR). The task will execute after the
; computed delay, properly synchronized with mission timeline.
; ============================================================================
; TYPE B VARIABLE RESTART HANDLING
;
; Type B restarts are hybrid entries combining variable restart information
; with table-based continuation. The initial phase provides the first task
; to restart, then processing continues through the restart tables to handle
; additional associated tasks for that restart group.
; ============================================================================

ITSLIKEB	CA	RTRNCADR	# TYPE B,	      SO STORE RETURN IN
		TS	TEMPSWCH	# TEMPSWCH IN CASE OF AN EVEN PHASE
; Set up return address for potential even-phase table processing.
; Type B restarts may have multi-part table entries requiring continuation.

		CA	PRT2CADR	# SET UP EXIT TO GET TABLE PART OF THIS
		TS	GOLOC +2	# VARIABLE TYPE OF PHASE
; PRT2CADR points to GETPART2 routine which handles the table-based
; continuation of Type B restarts. After processing the variable component,
; execution will continue through associated restart table entries.

		CA	TEMPPHS		# MAKE THE PHASE LOOK RIGHT FOR THE TABLE
		MASK	OCT177		# PART OF THIS VARIABLE PHASE
		TS	TEMPPHS
; Mask off the Type B identifier bits (retaining low 177 octal bits) to
; convert the phase encoding to standard table format. This allows the
; table processing logic to correctly interpret the remaining phase data.

		EXTEND
		NDX	TEMP2G		# OBTAIN THE JOB:S 2CADR
		DCA	PHSNAME1
		DXCH	GOLOC
; Retrieve the job's 2CADR (bank and address) from the restart tables using
; indexed addressing. PHSNAME1 contains the entry point address for the job.
; Store in GOLOC for later execution handoff.

; ============================================================================
; TRANSITION: From Type B setup to JOB restart execution
;
; With the variable component of Type B restarts processed and table 
; continuation configured, we now handle JOB restart execution. The priority
; value determines whether we use FINDVAC (search for available core set) or
; NOVAC (force specific core set allocation). This distinction is critical
; for proper task scheduling during restart recovery.
; ============================================================================

ITSAJOB		NDX	TEMP2G		# NOW ADD THE PRIORITY AND LET:S GO
		CA	PHSPRDT1
; Load the job priority from restart table. Priority determines both execution
; order and allocation method. During Apollo 11's descent, job priorities
; ensured guidance computations resumed before lower-priority telemetry tasks.

CHKNOVAC	TS	GOLOC -1	# SAVE PRIO UNTIL WE SEE IF ITS
		EXTEND			# A FINDVAC OR A NOVAC
		BZMF	ITSNOVAC
; Test priority sign using BZMF (Branch on Zero or Minus). Positive priority
; uses FINDVAC to search for next available VAC area. Negative priority uses
; NOVAC to force allocation of a specific core set, preserving computational
; state from the interrupted job.

		CAF	FVACCADR	# POSITIVE, SET UP FINDVAC CALL.
		XCH	GOLOC -1	# PICK UP PRIO,
		TC	GOLOC -1	# AND GO
; Priority is positive: call FINDVAC with priority in A register. FINDVAC
; searches the 7 core sets for the next available VAC area and initiates
; the job. This is the normal path for most restarted computational tasks.

ITSNOVAC	CAF	NOVACADR	# NEGATIVE,
		XCH	GOLOC -1	# SET UP NOVAC CALL,
		COM			# CORRECT PRIO,
		TC	GOLOC -1	# AND GO
; Priority is negative: call NOVAC to force allocation. Complement the priority
; to restore positive value (NOVAC expects positive priority). NOVAC allocates
; a specific core set, critical when resuming interrupted vector computations
; that have partial results in specific VAC registers.

; ============================================================================
; TRANSITION: From variable restart processing to table restart processing
;
; Table restarts handle sequences of operations that were interrupted during
; execution. Unlike variable restarts (single entry point), table restarts
; contain multiple phases, each representing a step in a longer process.
; The phase number determines where in the sequence to resume execution.
; This mechanism enabled Apollo 11's guidance computer to recover gracefully
; from the 1202 program alarms during descent.
; ============================================================================

ITSATBL		TS	CYR		# FIND OUT IF THE PHASE IS ODD OR EVEN
		CCS	CYR
		TCF	+1		# IT:S EVEN
		TCF	ITSEVEN
; Determine if the phase number is odd or even. Phase parity controls table
; indexing and restart sequencing. Odd phases indicate the first part of a
; two-part operation; even phases indicate continuation or completion.

		CA	RTRNCADR	# IN CASE THIS IS THE SECOND PART OF A
		TS	GOLOC +2	# TYPE B RESTART, WE NEED PROPER EXIT
; For Type B restarts (two-part operations), set up return address to continue
; processing subsequent restart groups after this table entry completes.
; Type B restarts were used for complex operations like orbital integration
; that spanned multiple restart intervals.

		CA	TEMPPHS		# SET UP POINTER FOR FINDING OUR PLACE IN
		TS	SR		# THE RESTART TABLES
		AD	SR
# Page 1306
		NDX	TEMP2G
		AD	SIZETAB +1
		TS	POINTER
; Calculate index into restart tables using phase number and group offset.
; TEMPPHS contains the phase within this group. SIZETAB +1 contains the base
; offset for this restart group's table entries. The sum gives absolute
; pointer to the specific restart table entry for this phase.

CONTBL2		EXTEND			# FIND OUT WHAT:S IN THE TABLE
		NDX	POINTER
		DCA	CADRTAB		# GET THE 2CADR
; Retrieve the table entry's 2CADR using calculated pointer. CADRTAB contains
; an array of restart entries. Each entry specifies what operation to resume:
; JOB, WAITLIST task, or LONGCALL. The 2CADR format encodes both the target
; address and the restart type through sign and bit encoding.

		LXCH	GOLOC +1	# STORE THE BB INFORMATION
; Store the bank portion (BB) of the 2CADR in GOLOC +1. The L register now
; contains the bank where the restart target resides. This bank information
; is critical for inter-bank transfers in the AGC's banked memory architecture.

		CCS	A		# IS IT A JOB OR IS IT  TIMED
		INCR	A		# POSITIVE. MUST BE A JOB
		TCF	ITSAJOB2
; Test the sign of the address portion. Positive address indicates a JOB
; restart (computational task requiring VAC allocation). Negative or zero
; indicates a time-deferred operation (WAITLIST or LONGCALL).

		INCR	A		# MUST BE EITHER A WAITLIST OR LONGCALL
		TS	GOLOC		# LET-S STORE THE CORRECT CADR
; Address was negative. Increment to correct the negative encoding and store
; the positive address. Now determine if this is a WAITLIST call (time-based
; task scheduling) or LONGCALL (deferred subroutine with time parameter).

		CA	WTLTCADR	# SET UP OUR EXIT TO WAITLIST
		TS	GOLOC -1
; Prepare for potential WAITLIST execution by setting up the exit address.
; If the entry turns out to be a WAITLIST task, control will transfer through
; this address to schedule the task with the WAITLIST timer system.

		CA	GOLOC +1	# NOW FIND OUT IF IT IS A WAITLIST CALL
		MASK	BIT10		# THIS SHOULD BE ONE IF WE HAVE -BB
		CCS	A		# FOR THAT MATTER SO SHOULD BE BITS 9,8,7,
					# 6,5, AND LAST BUT NOT LEAST (PERHAPS NOT
					# IN IMPORTANCE ANYWAY. BIT 4
		TCF	ITSWTLST	# IT IS A WAITLIST CALL
; Examine bit 10 of the bank information to distinguish WAITLIST from LONGCALL.
; Bit 10 set indicates WAITLIST (time-deferred task). Bit 10 clear indicates
; LONGCALL (subroutine call with time parameter). This encoding allows the
; restart tables to specify precise operation types for recovery.

		NDX	POINTER		# OBTAIN THE ORIGINAL DELTA T
		CA	PRDTTAB		# ADDRESS FOR THIS LONGCALL
; This is a LONGCALL restart. Retrieve the address of the delta-time parameter
; from PRDTTAB (parameter data table). LONGCALL operations store their time
; parameters in erasable memory, allowing restart logic to recalculate
; remaining time after interruption.

		TCF	ITSLGCL1	# NOW GO GET THE DELTA TIME
; Branch to ITSLGCL1 to retrieve the delta-time value. LONGCALL processing
; requires bank switching to access the parameter in its correct erasable
; bank, then calculating how much time remains before the call should execute.

# ***** THIS MUST BE IN FIXED FIXED *****

		BLOCK	02
		SETLOC	FFTAG2
		BANK
; CRITICAL: This section must reside in fixed-fixed memory (unswitched bank).
; Bank switching operations require stable code location that doesn't move
; during the switch itself. Fixed-fixed guarantees the code remains accessible
; regardless of current bank configuration.

		COUNT*	$$/RSROU
ITSLGCL1	LXCH	GOLOC +1	# OBTAIN THE CORRECT E BANK
		LXCH	BB
		LXCH	GOLOC +1	# AND PRESERVE OUR E AND F BANKS
; Perform careful bank switching to access the LONGCALL's delta-time parameter.
; Save current bank context (E and F banks), load the target erasable bank
; where the time parameter resides, preserving all register states for later
; restoration. This dance is necessary because the parameter may be in a
; different erasable bank than the current context.

		EXTEND			# GET THE DELTA TIME
		NDX	A
		DCA	0
; Retrieve the double-precision delta-time value using indirect indexed
; addressing. The A register contains the address of the time parameter.
; DCA 0 with indexing loads both words of the time value (TIME2 format:
; high word and low word representing centiseconds).

		LXCH	GOLOC +1	# RESTORE OUR E AND F BANK
		LXCH	BB		# RESTORE THE TASKS E AND F BANKS
		LXCH	GOLOC +1	# AND PRESERVE OUR L
# Page 1307
		TCF	ITSLGCL2	# NOT LET:S PROCESS THIS LONGCALL
; Restore the original bank context by reversing the earlier bank switches.
; The delta-time value is now in A,L registers. Transfer control to ITSLGCL2
; in switched fixed memory to complete LONGCALL processing and determine if
; the call should execute immediately or be rescheduled.

# ***** YOU MAY RETURN TO  SWITCHED FIXED *****

		BANK	01
		SETLOC	RESTART
		BANK
; Return to switched fixed memory. Bank-sensitive code can now execute safely
; with the delta-time value retrieved and bank context restored.

		COUNT*	$$/RSROU
ITSLGCL2	DXCH	LONGTIME
; Store the retrieved delta-time in LONGTIME (double-precision workspace).
; This preserves the original time parameter for recalculation of remaining
; time. During restart, we must determine if the scheduled event should
; execute immediately or be rescheduled with adjusted timing.

		EXTEND			# CALCULATE TIME LEFT
		DCS	TIME2
		DAS	LONGTIME
		EXTEND
		DCA	LONGBASE
		DAS	LONGTIME
; Calculate remaining time until the LONGCALL should execute:
; TIME_REMAINING = (LONGTIME - TIME2) + LONGBASE
; TIME2 is the current mission elapsed time. LONGBASE is a reference time
; set when the LONGCALL was originally scheduled. If TIME_REMAINING is
; positive, the call is still in the future. If negative or zero, execute
; immediately. This algorithm ensures restart doesn't cause premature or
; duplicate execution.

		CCS	LONGTIME	# FIND OUT HOW THIS SHOULD BE RESTARTED
		TCF	LONGCLCL
		TCF	+2
		TCF	IMEDIATE -3
		CCS	LONGTIME +1
		TCF	LONGCLCL
		NOOP			# CAN:T GET HERE    *********
		TCF	IMEDIATE -3
		TCF	IMEDIATE
; Double-precision time test. If LONGTIME (high word) is positive, significant
; time remains—branch to LONGCLCL to reschedule. If high word is zero, test
; low word. If low word is also zero or negative, time has expired—branch to
; IMEDIATE to execute the LONGCALL immediately. This careful double-precision
; comparison ensures accurate timing even for long-duration deferred calls.

LONGCLCL	CA	LGCLCADR	# WE WILL GO TO LONGCALL
		TS	GOLOC -1
; Time remaining is positive—the LONGCALL is not yet due. Set up transfer
; to LONGCALL scheduling routine. LGCLCADR points to the LONGCALL entry
; that will reschedule the call with the calculated remaining time.

		EXTEND			# PREPARE OUR ENTRY TO LONGCALL
		DCA	LONGTIME
		TC	GOLOC -1
; Load the remaining time (double-precision) into A,L registers as the
; parameter for LONGCALL. Transfer control to LONGCALL scheduler which will
; add this task back to the deferred execution queue. The LONGCALL will
; fire when TIME2 advances to equal the rescheduled target time.

; ============================================================================
; TRANSITION: From table LONGCALL to variable LONGCALL processing
;
; The previous section handled LONGCALL restarts from table entries (part of
; multi-phase operations). This section handles LONGCALL restarts from
; variable restart groups (single-phase deferred operations). Both paths
; converge on the same delta-time retrieval and remaining-time calculation
; logic, but arrive through different restart table structures.
; ============================================================================

ITSLNGCL	CA	WTLTCADR	# ASSUME IT WILL GO TO WAITLIST
		TS	GOLOC -1
; Variable LONGCALL restart entry point. Optimistically assume this will
; become a WAITLIST call (most deferred operations use WAITLIST for periodic
; execution). The time calculation logic below will determine if LONGCALL
; or WAITLIST scheduling is appropriate.

		NDX	TEMP2G
		CS	PHSPRDT1	# GET THE DELTA T ADDRESS
; Retrieve the delta-time parameter address from PHSPRDT1 table. Complement
; is used because PHSPRDT1 stores the address as negative to encode that this
; is a LONGCALL (vs. positive for direct JOB priority). The complement
; operation restores the positive address needed for indexing.

		TCF	ITSLGCL1	# NOW GET THE DELTA TIME
; Branch to ITSLGCL1 (fixed-fixed section) to retrieve the actual delta-time
; value using bank-safe operations. This is the same path used by table
; LONGCALLs, providing unified time-parameter retrieval logic.

; ============================================================================
; TRANSITION: From LONGCALL/table processing to WAITLIST restart processing
;
; WAITLIST restarts handle periodic tasks scheduled through the timer system.
; Unlike LONGCALLs (one-shot deferred calls), WAITLIST tasks may reschedule
; themselves, creating recurring execution patterns. The Apollo 11 guidance
; system used WAITLIST extensively for sensor sampling and display updates.
; ============================================================================

ITSWTLST	CS	GOLOC +1	# CORRECT THE BBCON INFORMATION
		TS	GOLOC +1
; Complement the bank information to correct the encoding. WAITLIST entries
; store bank as negative to distinguish from LONGCALL. Restore positive bank
; value for proper bank switching during task execution.

		NDX	POINTER		# GET THE DT AND FIND OUT IF IT WAS STORED
		CA	PRDTTAB		# DIRECTLY OR INDIRECTLY
; Retrieve the delta-time parameter descriptor from PRDTTAB using the
; calculated pointer. The value indicates whether the time is stored directly
; in the table (immediate value) or indirectly (address of erasable location
; containing the time). This encoding allows flexible time specification.

		TCF	TIMETEST	# FIND OUT HOW THE TIME IS STORED
; Branch to TIMETEST to determine storage mode and retrieve the actual
; time value. TIMETEST handles both direct and indirect time storage,
; then calculates if the WAITLIST call should execute immediately or be
; rescheduled with remaining time.

# Page 1308
; ============================================================================
; TRANSITION: From WAITLIST processing to table JOB restart processing
;
; This section handles JOB restarts that are part of table restart entries.
; Unlike variable JOB restarts (ITSAJOB), table JOBs are embedded in multi-
; phase restart sequences. The priority is retrieved from PRDTTAB rather
; than PHSPRDT1, reflecting the different table structure used for phase-
; based restart organization.
; ============================================================================

ITSAJOB2	XCH	GOLOC		# STORE THE CADR
; Store the job's CADR (bank and address) retrieved from the table entry.
; Exchange operation preserves the address in GOLOC while loading previous
; GOLOC value into A register (which is then discarded). This is a common
; AGC idiom for single-word storage.

		NDX	POINTER		# ADD THE PRIORITY AND LET:S GO
		CA	PRDTTAB
; Retrieve the job priority from PRDTTAB using indexed addressing. POINTER
; was calculated from the phase number and table structure, locating the
; correct priority value for this specific table entry. Priority determines
; execution order after restart.

		TCF	CHKNOVAC
; Transfer to CHKNOVAC to execute the job. This is the same path used by
; variable JOB restarts. CHKNOVAC tests priority sign to determine whether
; FINDVAC (search for available core set) or NOVAC (force specific core set)
; should be used for job initiation.

; ============================================================================
; TRANSITION: From table content processing to even-table restart handling
;
; Even-numbered phase restart tables contain two entries per phase (X.0 and
; X.2 phases). This section calculates the pointer to locate the correct
; entry within the table based on the phase number. The multiply-by-three
; algorithm optimizes for AGC's limited instruction set, using three additions
; rather than a multiply instruction (which AGC lacks for small constants).
; ============================================================================

ITSEVEN		CA	TEMPSWCH	# SET UP FOR EITHER THE SECOND PART OF THE
		TS	GOLOC +2	# TABLE, OR A RETURN FOR THE NEXT GROUP
; Load TEMPSWCH into GOLOC+2 to prepare exit path. TEMPSWCH was set during
; initialization to either PHS2CADR (continue with second table entry) or
; RTRNCADR (proceed to next restart group). This pre-configured branching
; allows efficient table traversal without repeated conditional tests.

		NDX	TEMP2G		# SET UP POINTER FOR OUR LOCATION WITHIN
		CA	SIZETAB		# THE TABLE
		AD	TEMPPHS		# THIS MAY LOOK BAD BUT LET:S SEE YOU DO
		AD	TEMPPHS		# BETTER IN TIME OR NUMBERR OF LOCATIONS
		AD	TEMPPHS
		TS	POINTER
; Calculate pointer offset within table: POINTER = SIZETAB + (3 * TEMPPHS)
; The three additions multiply TEMPPHS by 3 efficiently. Each table entry
; occupies 3 words (CADR, priority/time, flags), so multiplying phase number
; by 3 locates the correct entry. SIZETAB provides the base offset for this
; group's table. The comment acknowledges this looks unusual but challenges
; anyone to do better given AGC's instruction set limitations.

		TCF	CONTBL2		# NOW PROCESS WHAT IS IN THE TABLE
; Branch to CONTBL2 to process the table entry pointed to by POINTER. CONTBL2
; will decode whether this entry is a JOB, WAITLIST, or LONGCALL and route
; to the appropriate handler (ITSAJOB2, ITSWTLST, or table LONGCALL logic).

; ============================================================================
; TRANSITION: From first table entry to second table entry processing
;
; This entry point handles the second entry in a two-entry even table phase.
; After processing the X.0 phase entry, execution returns here to process
; the X.2 phase entry. The pointer is adjusted by 3 to skip to the second
; entry, and exit is configured to proceed to the next restart group since
; this is the final entry for this phase.
; ============================================================================

PHSPART2	CA	THREE		# SET THE POINTER FOR THE SECOND HALF OF
		ADS	POINTER		# THE TABLE
; Increment pointer by 3 to locate the second entry. Each table entry spans
; 3 words, so adding 3 moves from the X.0 entry to the X.2 entry. ADS (Add
; to Storage) performs POINTER = POINTER + 3, positioning for the second
; entry's CADR retrieval.

		CA	RTRNCADR	# THIS WILL BE OUR LAST TIME THROUGH THE
		TS	GOLOC +2	# EVEN TABLE , SO AFTER IT GET THE NEXT
					# GROUP
; Set exit path to RTRNCADR (return to get next group). Since this is the
; second and final entry for this phase, after processing we must move to
; the next restart group rather than looping back for more entries. This
; ensures systematic traversal through all restart groups.

		TCF	CONTBL2		# SO LET:S GET THE SECOND ENTRY IN THE TBL
; Branch to CONTBL2 to process the second table entry. CONTBL2 uses the
; updated POINTER to retrieve and decode this entry, completing the even
; table phase restart before advancing to the next restart group.

; ============================================================================
; DATA DEFINITIONS AND CONSTANTS
;
; This section defines memory locations and address constants used throughout
; the restart recovery routine. EQUALS directives alias temporary storage
; locations to MPAC (Multi-Purpose Accumulator) areas and VAC (Vector
; Accumulator) regions, minimizing erasable memory usage by reusing locations
; across non-overlapping execution contexts.
; ============================================================================

TEMPPHS		EQUALS	MPAC
; Temporary storage for phase number extracted from restart tables. Holds the
; phase value (X.0, X.1, X.2, etc.) during restart processing to determine
; which code entry point should be resumed. Aliases to MPAC word 0.

TEMP2G		EQUALS	MPAC +1
; Temporary storage for group number index (group number - 1). Used for
; indexed addressing to retrieve restart parameters from PHSNAME1, PHSPRDT1,
; and related tables. The doubled group number serves as a double-word index
; for CADR retrieval. Aliases to MPAC word 1.

POINTER		EQUALS	MPAC +2
; Calculated pointer offset within restart tables. For even-numbered phases,
; computed as SIZETAB + (3 * TEMPPHS) to locate the specific table entry.
; Each entry occupies 3 words (CADR, priority/time, flags). Aliases to MPAC
; word 2.

TEMPSWCH	EQUALS	MPAC +3
; Temporary switch variable controlling execution flow through table entries.
; Holds either PHS2CADR (continue to second table entry) or RTRNCADR (proceed
; to next restart group). Pre-configured during initialization to eliminate
; conditional branching during table traversal. Aliases to MPAC word 3.

GOLOC		EQUALS	VAC5 +20D
; Target location for restart execution transfer. During restart processing,
; GOLOC is loaded with the 2CADR (bank and address) of the code that should
; resume execution. The final TC GOLOC instruction transfers control to the
; restored task. Located in VAC area 5, offset 20 decimal.

MINUS2		EQUALS	NEG2
; Constant value -2 used for phase type detection. Subtracting 2 from the
; phase bits distinguishes between JOB, WAITLIST, and LONGCALL restart types.
; Aliases to system-defined NEG2 constant.

OCT177		EQUALS	LOW7
; Octal constant 177 (binary 001111111, 7 low-order bits set) used as mask
; for extracting phase number bits from restart table entries. The mask
; isolates the phase value while clearing flag bits in upper positions.
; Aliases to system-defined LOW7 constant.

; ============================================================================
; ADDRESS CONSTANTS (GENADR)
;
; GENADR directive generates interpretive addresses (bank and offset) for
; subroutine entry points. These constants enable bank-safe transfers during
; restart recovery, allowing code in one bank to call routines in different
; banks without explicit bank-switching sequences. Critical for the restart
; system since recovery code must access scheduling routines across multiple
; fixed-memory banks.
; ============================================================================

PHS2CADR	GENADR	PHSPART2
; Address of PHSPART2 subroutine (second table entry processor). Used as
; continuation address when even-table phase has two entries requiring
; sequential processing. Stored in TEMPSWCH during initialization.

PRT2CADR	GENADR	GETPART2
; Address of GETPART2 subroutine (restart type classifier). Used as return
; address after completing variable restart processing. Routes execution to
; table restart handling for X.0 and X.2 phases.

LGCLCADR	GENADR	LONGCALL
; Address of LONGCALL scheduler entry point. Used when restarting deferred
; operations that have not yet reached their scheduled execution time. The
; remaining time is passed as parameter to reschedule the LONGCALL.

FVACCADR	GENADR	FINDVAC
; Address of FINDVAC routine (find vacant core set). Used for JOB restarts
; with positive priority. FINDVAC searches the 7 core sets for the next
; available VAC area and initiates job execution in that core set.

WTLTCADR	GENADR	WAITLIST
; Address of WAITLIST scheduler entry point. Used when restarting periodic
; timer-driven tasks. WAITLIST maintains the delta-time queue for tasks
; scheduled at specific future times, supporting recurring execution patterns.

NOVACADR	GENADR	NOVAC
; Address of NOVAC routine (allocate specific core set). Used for JOB restarts
; with negative priority. NOVAC forces allocation of a designated core set,
; preserving computational state for resumed vector operations with partial
; results in VAC registers.
