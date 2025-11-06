# Copyright:	Public domain.
# Filename:	EXECUTIVE.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1208-1220
# Mod history:	2009-05-14 RSB	Adapted from the Colossus249/ file of the
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

# Page 1208
# ============================================================================
# FILE: EXECUTIVE.agc
# MODULE: CHIEFTAN Subsystem (Core Operating System)
# MISSION PHASE: all-phases
#
# TL;DR: Core task scheduler implementing cooperative multitasking via priority
#        job queue (7 core sets). Manages job scheduling (NOVAC, FINDVAC), task
#        switching, priority-based execution. During Apollo 11 descent at
#        ~102:38:26 MET, executive overflow triggered 1201 program alarm when
#        job queue saturated from landing radar data processing. Robust restart
#        protection and alarm handling enabled safe landing continuation despite
#        computational overload. Foundation of AGC real-time operating system.
#
# COMMENT-ONLY READERS: The master scheduler managing all computer tasks, whose
#        overflow caused the 1201 alarms during lunar landing.
# CODE-ALONG READERS: Study task scheduler architecture, cooperative multitasking
#        model, priority job queue (7 core sets), job management (NOVAC/FINDVAC),
#        task switching, 1201 alarm context (job queue overflow), restart integration.
# ============================================================================

		BLOCK	02

# ============================================================================
# EXECUTIVE OVERVIEW - AGC REAL-TIME OPERATING SYSTEM CORE
#
# The Executive implements cooperative multitasking for the Apollo Guidance
# Computer. It manages a priority-based job queue using 7 "core sets" - memory
# areas that store job state (program counter, bank, priority). Jobs voluntarily
# yield control, allowing higher-priority work to execute. This design enabled
# the AGC to manage guidance, navigation, display updates, and control
# simultaneously within severe memory constraints (2K erasable RAM).
#
# COOPERATIVE MULTITASKING MODEL:
# Unlike modern preemptive schedulers, jobs must explicitly yield via CHANG1/CHANG2.
# Higher-priority jobs can interrupt lower-priority ones, but within a priority
# level, jobs run to completion or voluntary suspension. This deterministic
# behavior was critical for real-time spaceflight control.
#
# 7 CORE SETS (Job Queue Structure):
# Each core set stores one job's execution state:
#   - LOC: Program counter (where job will resume)
#   - BBANK/FBANK: Memory bank context
#   - PRIORITY: Job priority level (determines scheduling order)
# Core sets are numbered 1-7. The Executive finds vacant cores via FINDVAC,
# allocates them to new jobs, and swaps between cores during job changes.
#
# CRITICAL APOLLO 11 CONTEXT - 1201 PROGRAM ALARM:
# During lunar descent at ~102:38:26 mission elapsed time, all 7 core sets
# became occupied. Landing radar data created computational load exceeding
# Executive capacity. When NOVAC/FINDVAC found no vacant cores, 1201 alarm
# triggered. Flight controller Steve Bales recognized this as non-critical
# overflow (lower-priority tasks delayed, mission-critical guidance continued).
# Restart protection preserved navigation state. Bales called "Go" and landing
# proceeded successfully. This demonstrated AGC's graceful degradation under
# overload - a key design achievement.
# ============================================================================

# TO ENTER A JOB REQUEST REQUIRING NO VAC AREA:
#
# NOVAC (NO VAC AREA) - Schedule basic jobs without interpreter requirement
#
# Purpose: Creates new job without requiring VAC (Vector Accumulator) area.
# Used for simple jobs that execute entirely in native AGC code without
# interpretive language operations. Faster than FINDVAC since no VAC allocation.
#
# Calling sequence:
#   TC  NOVAC           ; Call with A register containing priority
#   2CADR  JOBENTRY     ; Followed by job's 2CADR (bank + address)
#   ; Returns here after job scheduled
#
# During Apollo 11 descent, NOVAC was used for time-critical tasks like
# radar data processing and display updates. When all 7 core sets became
# occupied, NOVAC calls triggered 1201 alarms as Executive could not
# allocate new jobs until existing ones completed or yielded.

		COUNT	02/EXEC

NOVAC		INHINT			; Disable interrupts during job creation
		AD	FAKEPRET	# LOC(MPAC +6) - LOC(QPRET)
		TS	NEWPRIO		# PRIORITY OF NEW JOB + NOVAC C(FIXLOC)
					; Store priority in NEWPRIO for scheduler

		EXTEND
		INDEX	Q		# Q WILL BE UNDISTURBED THROUGHOUT.
		DCA	0		# 2CADR OF JOB ENTERED.
					; Q register holds return address
					; DCA 0 indexed by Q fetches 2CADR following TC NOVAC
		DXCH	NEWLOC		; Store job's 2CADR (bank and address) in NEWLOC
		CAF	EXECBANK	; Load Executive bank number
		XCH	FBANK		; Switch to Executive bank for scheduling
		TS	EXECTEM1	; Save previous bank for restoration
		TCF	NOVAC2		# ENTER EXECUTIVE BANK.
					; Continue in Executive's switched bank

# TO ENTER A JOB REQUEST REQUIRING A VAC AREA - E.G., ALL (PARTIALLY) INTERPRETIVE JOBS.
#
# FINDVAC (FIND VAC AREA) - Schedule interpretive jobs requiring VAC
#
# Purpose: Creates new job requiring VAC (Vector Accumulator) memory area.
# Required for all jobs using interpretive language (TC INTPRET) for vector/
# matrix operations, navigation computations, guidance calculations. Finds
# vacant core set AND allocates VAC area (MPAC stack space).
#
# Calling sequence:
#   CA  PRIORITY        ; Load priority into A register
#   TC  FINDVAC         ; Call FINDVAC
#   2CADR  JOBENTRY     ; Followed by job's 2CADR
#   ; Returns here after job scheduled
#
# Most guidance and navigation jobs during lunar landing used FINDVAC due to
# interpretive math requirements. When descent radar data processing created
# excessive computational load, FINDVAC found all 7 core sets occupied,
# triggering 1201 alarms. Lower-priority FINDVAC requests delayed until cores
# became available, allowing mission-critical guidance to continue.

FINDVAC		INHINT			; Disable interrupts during job creation
		TS	NEWPRIO		; Store priority from A register in NEWPRIO
		EXTEND
		INDEX	Q		; Q holds return address
		DCA	0		; Fetch 2CADR following TC FINDVAC call
SPVACIN		DXCH	NEWLOC		; Store job's 2CADR in NEWLOC
		CAF	EXECBANK	; Load Executive bank number
		XCH	FBANK		; Switch to Executive bank
		TCF	FINDVAC2	# OFF TO EXECUTIVE SWITCHED-BANK.
					; Continue in Executive's switched bank for VAC allocation

# TO ENTER A FINDVAC WITH THE PRIORITY IN NEWPRIO TO THE 2CADR ARRIVING IN A AND L:
# USERS OF SPVAC MUST INHINT BEFORE STORING IN NEWPRIO.
#
# SPVAC (SPECIAL VAC) - Alternate FINDVAC entry with 2CADR in A/L registers
#
# Purpose: Variant of FINDVAC where job's 2CADR arrives in A and L registers
# rather than immediately following the call. Allows dynamic job address
# computation. Priority must be pre-stored in NEWPRIO with interrupts inhibited.
#
# Used by restart system and dynamic job creation where target address
# computed at runtime rather than fixed at assembly time.

SPVAC		XCH	Q		; Swap Q (return address) with A
		AD	NEG2		; Adjust Q by -2 to compensate for SPVACIN offset
		XCH	Q		; Restore adjusted Q
		TCF	SPVACIN		; Continue through FINDVAC entry path
					; A and L already contain job 2CADR

# ============================================================================
# TASK SWITCHING AND COOPERATIVE MULTITASKING
#
# CHANG1 and CHANG2 implement cooperative task switching - the mechanism by
# which jobs voluntarily yield control so higher-priority work can execute.
# Unlike preemptive systems that forcibly interrupt tasks, AGC jobs must
# explicitly call CHANG1 (basic jobs) or CHANG2 (interpretive jobs) to
# suspend execution.
#
# This cooperative model was essential for deterministic real-time behavior.
# Mission-critical guidance calculations could run without unexpected
# interruption. During Apollo 11 descent, proper CHANG usage allowed radar
# processing, guidance updates, and display refreshes to share the single CPU.
# ============================================================================

# TO SUSPEND A BASIC JOB SO A HIGHER PRIORITY JOB MAY BE SERVICED:
#
# CHANG1 - Suspend basic (non-interpretive) job for task switch
#
# Purpose: Voluntarily suspend current basic job, saving its state in a core
# set, and switch to highest-priority waiting job. Called when job has
# completed a logical unit of work and can yield CPU.
#
# Job state saved:
#   - LOC (program counter from Q register - where to resume)
#   - BBANK (memory bank context)
#   - Priority level
#
# Executive selects next job by scanning core sets for highest priority.
# If no other jobs ready, returns to idle loop. Current job resumes when
# it becomes highest priority again.

CHANG1		LXCH	Q		; Move return address from Q to L register
					; L now holds where current job will resume
		CAF	EXECBANK	; Load Executive bank number
		XCH	BBANK		; Switch to Executive bank for job swap
		TCF	CHANJOB		; Continue to job change logic

# TO SUSPEND AN INTERPRETIVE JOB:
#
# CHANG2 - Suspend interpretive job for task switch
#
# Purpose: Voluntarily suspend current interpretive job (one using TC INTPRET).
# Different from CHANG1 because interpretive jobs have negative LOC to indicate
# they're in interpreter mode, requiring different state restoration.
#
# Called from within interpretive code sequences when job yields CPU. During
# lunar landing, guidance computations in interpretive language would CHANG2
# periodically to allow display updates and radar processing.

CHANG2		CS	LOC		# NEGATIVE LOC SHOWS JOB = INTERPRETIVE.
					; Complement LOC to make negative
					; Signals Executive this is interpretive job requiring
					; special handling on resume (return via interpreter)
# ITRACE (4) REFERS TO "CHANG2".
# Page 1209
		TS	L
	+2	CAF	EXECBANK
 		TS	BBANK
		TCF	CHANJOB -1

# Page 1210
# ============================================================================
# JOB SLEEP/WAKE MECHANISM - Event-Driven Synchronization
#
# JOBSLEEP and JOBWAKE implement event-driven synchronization, allowing jobs
# to suspend while waiting for I/O completion, sensor data, or other events.
# Sleeping job releases its core set, making it available for other work.
# When event occurs, another routine calls JOBWAKE to reschedule the job.
#
# Critical for efficient CPU utilization during Apollo missions. Guidance jobs
# could sleep while waiting for radar data, display jobs while waiting for
# DSKY input, navigation jobs while waiting for IMU updates. Without this
# mechanism, jobs would busy-wait, consuming precious computational cycles.
# ============================================================================

# TO VOLUNTARILY SUSPEND A JOB UNTIL THE COMPLETION OF SOME ANTICIPATED EVENT (I/O EVENT ETC.):
#
# JOBSLEEP - Put current job to sleep pending event
#
# Purpose: Suspend current job until external event occurs (I/O completion,
# timer expiration, data ready, etc.). Job state saved but core set marked
# inactive. Job will not execute until explicitly awakened by JOBWAKE.
#
# Calling sequence:
#   CA  SLEEPADDR       ; Address where job will resume when awakened
#   TC  JOBSLEEP        ; Put job to sleep
#   ; Job suspended here until JOBWAKE called
#
# During lunar descent, radar processing jobs would JOBSLEEP while waiting for
# landing radar data. When radar data interrupt occurred, interrupt handler
# would JOBWAKE the processing job to handle new measurements.

JOBSLEEP	TS	LOC		; Store resume address in LOC
		CAF	EXECBANK	; Load Executive bank
		TS	FBANK		; Store in FBANK for bank context
		TCF	JOBSLP1		; Continue to sleep processing

# TO AWAKEN A JOB PUT TO SLEEP IN THE ABOVE FASHION:
#
# JOBWAKE - Wake sleeping job and reschedule it
#
# Purpose: Awaken job previously suspended via JOBSLEEP. Job rescheduled
# via FINDVAC/NOVAC procedures with original priority. Called by interrupt
# handlers, I/O completion routines, or other jobs when event occurs.
#
# Calling sequence:
#   CA  JOBADDR         ; Address of sleeping job (saved when it slept)
#   TC  JOBWAKE         ; Wake the job
#   ; Returns after job rescheduled
#
# Essential for responsive system behavior. Sleeping jobs awakened immediately
# when data ready, minimizing latency between event and processing.

JOBWAKE		INHINT			; Disable interrupts during wake processing
		TS	NEWLOC		; Store job resume address in NEWLOC
		CS	TWO		# EXIT IS VIA FINDVAC/NOVAC PROCEDURES.
		ADS	Q		; Adjust Q for FINDVAC/NOVAC entry
					; Job will be rescheduled through normal job creation path
		CAF	EXECBANK	; Load Executive bank
		XCH	FBANK		; Switch to Executive bank
		TCF	JOBWAKE2	; Continue wake processing in Executive bank

# ============================================================================
# DYNAMIC PRIORITY CHANGE - Runtime Job Priority Adjustment
#
# PRIOCHNG allows running job to change its own priority dynamically. Essential
# for adaptive scheduling where job urgency changes based on mission phase or
# computational needs. Higher priority ensures job executes before lower-priority
# work when multiple jobs ready.
#
# During descent, guidance jobs could boost priority when approaching critical
# phase transitions (braking to approach, approach to landing). Display jobs
# could lower priority when crew attention not required.
# ============================================================================

# TO CHANGE THE PRIORITY OF A JOB CURRENTLY UNDER EXECUTION:
#
# PRIOCHNG - Change current job's priority
#
# Purpose: Dynamically adjust priority of currently executing job. Job continues
# running if still highest priority after change. If lower priority jobs now
# have higher priority, job yields via implicit CHANG-style task switch.
#
# Calling sequence:
#   CA  NEWPRIORITY     ; Load new priority value in A register
#   TC  PRIOCHNG        ; Change priority
#   ; Returns here after priority changed and job remains highest
#
# Returns when current job again becomes highest priority (may be immediate
# if new priority still highest, or after other jobs complete if new priority
# lower than waiting jobs).

PRIOCHNG	INHINT			# NEW PRIORITY ARRIVES IN A. RETURNS TO
		TS	NEWPRIO		# CALLER AS SOON AS NEW JOB PRIORITY IS
					; Store new priority value
		CAF	EXECBANK	# HIGHEST. PREPARE FOR POSSIBLE BASIC-
		XCH	BBANK		# STYLE CHANGE-JOB.
					; Switch to Executive bank for priority change
		TS	BANKSET		; Save previous bank
		CA	Q		; Load return address from Q
		TCF	PRIOCH2		; Continue to priority change logic

# ============================================================================
# JOB TERMINATION - Releasing Core Sets
#
# ENDOFJOB marks current job complete, releases its core set for reuse, and
# switches to next highest priority job. Critical for core set recycling in
# 7-core-set system. Without proper job termination, cores would remain
# occupied indefinitely, eventually causing 1201 alarms.
#
# Every job must end via ENDOFJOB (never just halt or infinite loop). Ensures
# Executive can reclaim resources and maintain system responsiveness.
# ============================================================================

# TO REMOVE A JOB FROM EXECUTIVE CONSIDERATIONS:
#
# ENDOFJOB - Terminate current job and release core set
#
# Purpose: Signal current job has completed all work. Executive marks core set
# as available (PRIORITY = -0), allowing NOVAC/FINDVAC to reuse it for new jobs.
# Execution switches to next highest priority waiting job.
#
# Calling sequence:
#   TC  ENDOFJOB        ; Called at end of job's execution
#   ; Never returns - execution switches to next job
#
# During Apollo 11 descent, proper ENDOFJOB usage allowed radar processing jobs
# to complete, release cores, and make room for new processing cycles. When
# jobs failed to terminate promptly (long computations), cores remained occupied
# longer, contributing to 1201 alarm conditions.

ENDOFJOB	CAF	EXECBANK	; Load Executive bank
		TS	FBANK		; Switch to Executive bank
		TCF	ENDJOB1		; Continue to job termination logic

ENDFIND		CA	EXECTEM1	# RETURN TO CALLER AFTER JOB ENTRY
		TS	FBANK		# COMPLETE.
		TCF	Q+2
EXECBANK	CADR	FINDVAC2

FAKEPRET	ADRES	MPAC -36D	# LOC(MPAC +6) - LOC(QPRET)

# Page 1211
# ============================================================================
# SWITCHED-BANK EXECUTIVE ROUTINES
#
# FINDVAC2 and NOVAC2 are core scheduler routines residing in Executive's
# switched bank (Bank 01). These implement the actual resource allocation
# and job creation after initial entry processing in fixed bank.
# ============================================================================

# ============================================================================
# VAC AREA ALLOCATION - Finding Interpretive Work Space
#
# FINDVAC2 searches for available VAC (Vector Accumulator) area - memory
# workspace required by interpretive language jobs. AGC has 5 VAC areas
# (VAC1-VAC5), each providing MPAC stack space for vector/matrix operations.
#
# During Apollo 11 descent at mission time ~102:38:26, excessive landing
# radar data processing created high demand for VAC areas. Guidance equations,
# navigation updates, and display formatting all required interpretive math.
# When all 5 VAC areas occupied, FINDVAC2 triggered 1201 alarm ("NO VAC AREAS").
#
# Steve Bales (GUIDO) recognized 1201 as resource exhaustion, not guidance
# failure. With Jack Garman's support, he made "Go" call allowing landing to
# continue. Lower-priority jobs delayed until VAC areas freed, but mission-
# critical guidance maintained control. This alarm demonstrated AGC's graceful
# degradation under overload.
# ============================================================================

# LOCATE AN AVAILABLE VAC AREA.

		BANK	01		; Executive's switched bank
		COUNT	01/EXEC

FINDVAC2	TS	EXECTEM1	# (SAVE CALLER'S BANK FIRST.)
					; Store caller's bank number in EXECTEM1 for return
					; Entry from FINDVAC after bank switch
		CCS	VAC1USE		; Check if VAC area 1 available (CCS = Count, Compare, Skip)
					; VAC1USE contains +0 if available, -0 if in use
		TCF	VACFOUND	; VAC1 available - use it
		CCS	VAC2USE		; VAC1 occupied, try VAC2
		TCF	VACFOUND	; VAC2 available
		CCS	VAC3USE		; Try VAC3
		TCF	VACFOUND	; VAC3 available
		CCS	VAC4USE		; Try VAC4
		TCF	VACFOUND	; VAC4 available
		CCS	VAC5USE		; Try VAC5 (last VAC area)
		TCF	VACFOUND	; VAC5 available
					;
					; ALL 5 VAC AREAS OCCUPIED - TRIGGER 1201 ALARM
					; This condition occurred during Apollo 11 descent when
					; interpretive jobs (guidance, navigation, displays) all
					; needed VAC areas simultaneously. System overloaded but
					; not failing - alarm indicated resource exhaustion.
		TC	BAILOUT		; Call alarm handler
		OCT	1201		# NO VAC AREAS.
					; Alarm code 1201: "Executive overflow - no VAC areas"
					; Job requesting VAC delayed until area becomes available
					; Mission can continue if lower-priority jobs tolerate delay

# ============================================================================
# VAC AREA RESERVATION
# Available VAC area found - reserve it for this job and store VAC address
# in low 9 bits of priority word for job's reference.
# ============================================================================
VACFOUND	AD	TWO		# RESERVE THIS VAC AREA BY STORING A ZERO
					; A register contains address of VACxUSE register
					; Adding TWO advances to actual VAC area address
		ZL			# IN ITS VAC USE REGISTER AND STORE THE
					; Zero L register
		INDEX	A		# ADDRESS OF THE FIRST WORD OF IT IN THE
					; Indexed addressing: use A as pointer
		LXCH	0 -1		# LOW NINE BITS OF THE PRIORITY WORD.
					; Exchange L with VACxUSE (marks VAC in use: -0)
					; Load L with VAC area starting address
		ADS	NEWPRIO		; Add VAC address to NEWPRIO (stores in low 9 bits)
					; NEWPRIO now contains both priority and VAC pointer

# ============================================================================
# CORE SET ALLOCATION - Finding Job Execution Slot
#
# NOVAC2 searches for available core set (job execution slot). AGC executive
# manages 7 core sets (numbered 0-6), each containing 11 registers for job
# state: LOC (program counter), BANKSET, PUSHLOC (stack pointer), PRIORITY,
# and 7-word push-down stack.
#
# Core set 0 has special significance - it's always loaded first and sets
# OVFIND (overflow indicator) and FIXLOC (fixed-bank return address). Other
# core sets (1-6) handle additional concurrent jobs.
#
# During Apollo 11 descent, all 7 core sets could be occupied when guidance,
# navigation, display updates, telemetry, and radar processing all executed
# simultaneously. When NOVAC2 finds no available core sets, it triggers 1202
# alarm - the other alarm during lunar landing.
# ============================================================================
NOVAC2		CAF	ZERO		# NOVAC ENTERS HERE. FIND A CORE SET.
					; NOVAC (no VAC needed) jobs enter directly here
					; FINDVAC jobs arrive here after VAC allocation
		TS	LOCCTR		; Initialize core set counter to 0
		CAF	NO.CORES	# SEVEN SETS OF ELEVEN REGISTERS EACH.
					; Load constant 6 (will loop through cores 0-6)
NOVAC3		TS	EXECTEM2	; Save remaining core count
		INDEX	LOCCTR		; Index by current core set number
		CCS	PRIORITY	# EACH PRIORITY REGISTER CONTAINS -0 IF
					; Check PRIORITY register for this core set
					; -0 (minus zero) = available, +value = active job
		TCF	NEXTCORE	# THE CORRESPONDING CORE SET IS AVAILABLE.
					; Positive = active job, try next core
NO.CORES	DEC	6		; Constant: 6 (for 7 core sets: 0 through 6)
		TCF	NEXTCORE	# AN ACTIVE JOB HAS A POSITIVE PRIORITY
					; Zero or negative = try next core
					# BUT A DORMANT JOB'S PRIORITY IS NEGATIVE
					; Dormant jobs (sleeping) have negative priority
					; Active jobs have positive priority
					; Available cores have -0 (minus zero)

# Page 1212
# ============================================================================
# CORE SET FOUND - Job Slot Initialization
# Available core set located - initialize its registers and determine if
# this new job should preempt currently running job based on priority.
# ============================================================================
CORFOUND	CA	NEWPRIO		# SET THE PRIORITY OF THIS JOB IN THE CORE
					; Load new job's priority (with VAC address in low 9 bits)
		INDEX	LOCCTR		# SET'S PRIORITY REGISTER AND SET THE
					; Index by core set number
		TS	PRIORITY	# JOB'S PUSH-DOWN POINTER AT THE BEGINNING
					; Store priority in this core set's PRIORITY register
					; Positive priority indicates active job
		MASK	LOW9		# OF THE WORK AREA AND OVERFLOW INDICATOR
					; Extract low 9 bits (VAC area address)
		INDEX	LOCCTR		; Index by core set number
		TS	PUSHLOC		# OFF TO PREPARE FOR INTERPRETIVE PROGRAMS
					; Initialize PUSHLOC (push-down stack pointer) to
					; beginning of VAC work area (or 0 for NOVAC jobs)

		CCS	LOCCTR		# IF CORE SET ZERO IS BEING LOADED, SET UP
					; Check if this is core set 0 (special core)
		TCF	SETLOC		# OVFIND AND FIXLOC IMMEDIATELY.
					; Not core 0, skip to normal setup
					; Core 0 requires special initialization:
		TS	OVFIND		; Store 0 in OVFIND (overflow indicator off)
					; OVFIND detects interpretive overflow in core 0
		CA	PUSHLOC		; Load VAC address from PUSHLOC
		TS	FIXLOC		; Store in FIXLOC (return address register)
					; FIXLOC used for fixed-bank subroutine returns

# ============================================================================
# Special Case: Multiple Job Activation
# Handles rare case where job being created is actually an awakened job that
# was put to sleep (via JOBSLEEP). Check if this job should immediately become
# the running job without going through normal priority comparison.
# ============================================================================
SPECTEST	CCS	NEWJOB		# SEE IF ANY ACTIVE JOBS WAITING (RARE).
					; NEWJOB contains core set # of highest priority job
					; or +0 if no comparison needed yet
		TCF	SETLOC		# MUST BE AWAKENED BUT UNCHANGED JOB.
					; Positive = already have active job to compare
		TC	CCSHOLE		; +0 = special case, skip through CCS holes
		TC	CCSHOLE		; -0 case (not used, but CCS requires 4 exits)
		TS	NEWJOB		# +0 SHOWS ACTIVE JOB ALREADY SET.
					; Store +0 in NEWJOB (indicates job already determined)
		DXCH	NEWLOC		; Load new job's 2CADR
		DXCH	LOC		; Store directly into core 0's LOC registers
					; This job runs immediately (no priority comparison)
		TCF	ENDFIND		; Done - exit to run this job

# ============================================================================
# Normal Job Setup and Priority Comparison
# Initialize core set's LOC registers with job's 2CADR, then compare new job's
# priority with currently highest priority to determine if immediate switch needed.
# ============================================================================
SETLOC		DXCH	NEWLOC		# SET UP THE LOCATION REGISTERS FOR THIS
					; Load new job's 2CADR (bank + address)
		INDEX	LOCCTR		; Index by core set number
		DXCH	LOC		; Store in this core set's LOC registers
					; Job now fully initialized in core set
		INDEX	NEWJOB		# THIS INDEX INSTRUCTION INSURES THAT THE
					; NEWJOB contains current highest priority core set #
		CS	PRIORITY	# HIGHEST ACTIVE PRIORITY WILL BE COMPARED
					; Load complement of highest priority job's priority
		AD	NEWPRIO		# WITH THE NEW PRIORITY TO SEE IF NEWJOB
					; Add new job priority - result shows relationship
		EXTEND			# SHOULD BE SET TO SIGNAL A SWITCH.
					; Prepare for branch instruction
		BZMF	ENDFIND		; Branch if result is Zero, Minus, or Minus zero
					; New priority ≤ current highest: no switch needed
		CA	LOCCTR		# LOCCTR IS LEFT SET AT THIS CORE SET IF
					; New priority > current: this job should run
		TS	NEWJOB		# THE CALLER WANTS TO LOAD ANY MPAC
					; Store this core set # in NEWJOB (signals switch)
		TCF	ENDFIND		# REGISTERS, ETC.
					; Done - return to caller who may load MPAC registers

# ============================================================================
# Try Next Core Set - Scan for Available Job Slot
# Current core set occupied - advance to next core set and continue search.
# If all 7 core sets checked and none available, trigger 1202 alarm.
#
# During Apollo 11 descent at mission time ~102:38:26, multiple 1202 alarms
# occurred when all 7 core sets were occupied. Landing radar data processing,
# guidance computations, navigation updates, display formatting, and telemetry
# all required executive jobs simultaneously. Like 1201 alarms, these indicated
# resource exhaustion under heavy load, not system failure.
#
# Flight controller Steve Bales and support engineer Jack Garman recognized
# 1202 as acceptable overload condition. System would delay lower-priority
# jobs until core sets became available. Mission-critical guidance and control
# maintained execution. This demonstrated AGC's robust priority-based scheduling
# enabling safe landing despite computational saturation.
# ============================================================================
NEXTCORE	CAF	COREINC		; Load core increment constant (11 registers per core)
		ADS	LOCCTR		; Add to LOCCTR - advance to next core set
					; LOCCTR now points to next core's registers
		CCS	EXECTEM2	; Decrement remaining core count
					; EXECTEM2 tracks how many cores left to check
		TCF	NOVAC3		; More cores to check - loop back
					; Continue scanning for available core set
					;
					; ALL 7 CORE SETS OCCUPIED - TRIGGER 1202 ALARM
					; This condition occurred multiple times during Apollo 11
					; descent when executive job demand exceeded capacity.
					; Jobs: guidance equations, navigation updates, radar
					; processing, display updates, telemetry formatting.
					; System overloaded but functional - mission-critical
					; jobs continued execution, lower-priority delayed.
		TC	BAILOUT		# NO CORE SETS.
					; Call alarm handler - job remains queued
		OCT	1202		; Alarm code 1202: "Executive overflow - no core sets"
					; Job creation delayed until core set becomes available
					; Priority scheduling ensures guidance/control unaffected
# Page 1213
# ============================================================================
# CHANJOB - Core Task Switching Mechanism
#
# The heart of AGC's cooperative multitasking. Swaps execution state between
# core set 0 (currently running job) and another core set (new job to run).
# Each core set contains complete execution context:
#   - LOC: Program counter (where job will resume)
#   - BANKSET: Bank registers (memory bank configuration)  
#   - MPAC: Multi-Purpose Accumulator (8 words of working storage)
#   - PUSHLOC: Interpreter pushdown list pointer
#   - PRIORITY: Job priority level (determines scheduling order)
#
# Task switching preserves complete state, enabling jobs to suspend and resume
# transparently. During Apollo 11 descent, CHANJOB executed every few
# milliseconds, rapidly cycling between guidance, navigation, displays, and
# telemetry jobs. This created illusion of parallel execution on single CPU.
#
# ENTRY:
#   Entry -2: For interpretive jobs (negative LOC indicates interpretive)
#   Entry -1: With BANKSET already loaded
#   Entry CHANJOB: Normal entry point
#   NEWJOB: Relative address of new job's core set (0, 12, 24, 36, 48, 60, 72)
#
# EXIT:
#   Execution continues at new job's saved LOC with complete state restored
# ============================================================================
# THE FOLLOWING ROUTINE SWAPS CORE SET 0 WITH THAT WHOSE RELATIVE ADDRESS IS IN NEWJOB.

	-2	LXCH	LOC		; Entry -2: LOC in L register (for interpretive jobs)
	-1	CAE	BANKSET		# BANKSET, NOT BBANK, HAS RIGHT CONTENTS.
				; Entry -1: BANKSET already loaded
				; CAE loads from erasable memory to A register
CHANJOB		INHINT			; Main entry: Disable interrupts during context swap
				; Critical section - cannot allow interrupt during
				; partial state swap or system would be inconsistent
		EXTEND			; Enable next instruction's extended addressing
		ROR	SUPERBNK	# PICK UP CURRENT SBANK FOR BBCON
				; Read current superbank, rotate right through A
				; SUPERBNK contains high-order bank bits for >32K addressing
		XCH	L		# LOC IN A AND BBCON IN L.
				; Exchange A and L: LOC now in A, BBCON in L
				; Preparing to save current job's program counter
	+4	INDEX	NEWJOB		# SWAP LOC AND BANKSET.
				; Use NEWJOB as index - points to new job's core set
				; INDEX makes next instruction use NEWJOB as offset
		DXCH	LOC		; Double exchange: (A,L) ↔ (LOC, LOC+1) at NEWJOB
				; Swaps current job's LOC/BBCON with new job's
		DXCH	LOC		; Second DXCH completes three-way swap
				; Core set 0 and target core now have exchanged LOC/BBCON

		CAE	BANKSET		; Load new job's bank configuration
		EXTEND
		WRITE	SUPERBNK	# SET SBANK FOR NEW JOB.
				; Write to SUPERBNK register, setting memory banking
				; for new job's address space
		DXCH	MPAC		# SWAP MULTI-PURPOSE ACCUMULATOR AREAS.
				; Begin swapping 8-word MPAC (working storage)
				; MPAC used by interpretive language for vector/matrix ops
		INDEX	NEWJOB		; Index to target core set's MPAC
		DXCH	MPAC		; Swap MPAC words 0-1
		DXCH	MPAC		; Complete three-way swap for MPAC 0-1
		DXCH	MPAC +2		; Swap MPAC words 2-3
		INDEX	NEWJOB
		DXCH	MPAC +2
		DXCH	MPAC +2		; Complete swap for MPAC 2-3
		DXCH	MPAC +4		; Swap MPAC words 4-5
		INDEX	NEWJOB
		DXCH	MPAC +4
		DXCH	MPAC +4		; Complete swap for MPAC 4-5
		DXCH	MPAC +6		; Swap MPAC words 6-7
		INDEX	NEWJOB
		DXCH	MPAC +6
		DXCH	MPAC +6		; Complete swap for MPAC 6-7
				; All 8 words of MPAC now swapped between jobs

		CAF	ZERO
		XCH	OVFIND		# MAKE PUSHLOC NEGATIVE IF OVFIND NZ.
				; Check overflow indicator for interpreter
				; OVFIND non-zero means pushdown list overflowed
		EXTEND
		BZF	+3		; Branch if OVFIND was zero (no overflow)
		CS	PUSHLOC		; Complement PUSHLOC to make negative
		TS	PUSHLOC		; Negative PUSHLOC indicates overflow condition

		DXCH	PUSHLOC		; Swap PUSHLOC and PRIORITY
				; PUSHLOC: Interpreter pushdown list pointer
				; PRIORITY: Job priority level and VAC area assignment
		INDEX	NEWJOB
		DXCH	PUSHLOC		; Swap with target core set
		DXCH	PUSHLOC		# SWAPS PUSHLOC AND PRIORITY.
				; Complete three-way swap of PUSHLOC/PRIORITY
		CAF	LOW9		# SET FIXLOC TO BASE OF VAC AREA.
				; Load mask for low 9 bits
		MASK	PRIORITY	; Extract VAC area number from PRIORITY
				; Low 9 bits encode VAC area assignment (0-4 for VAC1-5)
		TS	FIXLOC		; Store VAC area base address in FIXLOC
				; FIXLOC points to this job's interpretive workspace

		CCS	PUSHLOC		# SET OVERFLOW INDICATOR ACCORDING TO
				; Check sign of PUSHLOC to detect overflow
		CAF	ZERO		; PUSHLOC positive: no overflow
		TCF	ENDPRCHG -1	; Continue to priority change epilogue

# Page 1214
		CS	PUSHLOC		; CCS took negative branch: PUSHLOC was negative
		TS	PUSHLOC		; Restore original (positive) PUSHLOC value
					; CS undoes the earlier complement operation
		CAF	ONE		; Load constant 1
		XCH	OVFIND		; Exchange with OVFIND, setting overflow flag
		TS	NEWJOB		; Save old OVFIND in NEWJOB (typically zero)

# ============================================================================
# ENDPRCHG - Priority Change Epilogue and Job Dispatch
#
# Final stage of task switching. Re-enables interrupts and dispatches to
# the new job's saved location. Handles both basic jobs (native AGC code)
# and interpretive jobs (virtual machine code) differently.
#
# Basic jobs: Positive LOC addresses, dispatched directly via DTCB
# Interpretive jobs: Negative LOC addresses, require interpreter setup
#
# This completes the context swap started by CHANJOB. The new job resumes
# execution exactly where it left off, with complete state restored.
# ============================================================================
ENDPRCHG	RELINT			; Re-enable interrupts - critical section complete
					; Context swap finished, safe to allow interrupts
		DXCH	LOC		# BASIC JOBS HAVE POSITIVE ADDRESSES, SO
					; Load LOC (program counter) into A and L
					; LOC contains address where new job will resume
		EXTEND			# DISPATCH WITH A DTCB.
		BZMF	+2		# IF INTERPRETIVE, SET UP EBANK, ETC.
					; Branch if minus or zero (negative LOC = interpretive)
					; Negative LOC indicates job uses interpreter VM
		DTCB			; Dispatch To Core Bank (basic job resume)
					; DTCB performs cross-bank jump to LOC address
					; Basic job continues native AGC instruction execution
# Page 1215
		COM			# EPILOGUE TO JOB CHANGE FOR INTERPRETIVE
					; CCS branch: LOC was negative (interpretive job)
					; Complement LOC to recover original positive address
		AD	ONE		; Add 1 to complete two's complement conversion
		TS	LOC		# RESUME.
					; Store corrected LOC address
					; Interpretive job will resume at this address
		TCF	INTRSM		; Transfer to interpreter resume routine
					; INTRSM sets up interpreter environment (EBANK, etc.)
					; and begins executing interpretive instructions
					; Interpretive job continues VM instruction execution

# ============================================================================
# JOBSLP1 - Complete Job Sleep Preparations
#
# Completes the JOBSLEEP operation initiated earlier. Marks the current job
# as asleep by negating its PRIORITY (negative priority = sleeping job).
# Saves bank context, then scans for the next highest priority job to run.
#
# Jobs sleep when waiting for I/O completion, timer events, or other
# asynchronous conditions. A sleeping job consumes no CPU time until
# awakened by JOBWAKE when its awaited event occurs.
#
# ENTRY:
#   Called from JOBSLEEP with LOC already saved
#
# EXIT:
#   Current job suspended (negative priority), execution transferred to
#   highest priority active job via EJSCAN
# ============================================================================
# COMPLETE JOBSLEEP PREPARATIONS.

JOBSLP1		INHINT			; Disable interrupts for atomic sleep operation
		CS	PRIORITY	# NNZ PRIORITY SHOWS JOB ASLEEP.
					; Complement priority to make it negative
					; Negative priority indicates job is sleeping
		TS	PRIORITY		; Store negative priority back to memory
					; Job now marked as asleep, won't be scheduled
		CAF	LOW7		; Load mask for low 7 bits (bank number)
		MASK	BBANK		; Extract current bank from BBANK register
		EXTEND			; Enable extended addressing for next instruction
		ROR	SUPERBNK	# SAVE OLD SUPERBANK VALUE.
					; Rotate right through superbank to capture
					; high-order address bits for >32K memory
		TS	BANKSET		; Save complete bank configuration in BANKSET
					; Needed when job wakes up and resumes
		CS	ZERO		; Load negative zero (-0)
JOBSLP2		TS	BUF +1		# HOLDS - HIGHEST PRIORITY.
					; Initialize buffer with -0 (lowest possible priority)
					; BUF +1 tracks highest priority found during scan
		TCF	EJSCAN		# SCAN FOR HIGHEST PRIORITY ALA ENDOFJOB.
					; Transfer to end-of-job scanner to find next job
					; Same logic as ENDOFJOB: find active job to run

# ============================================================================
# NUCHANG2 - Rapid Job Change for Core Set 0
#
# Handles special case where new job is destined for core set 0 (running
# position). Must act quickly to prevent race condition where NEWJOB might
# be cleared to +0 by another process before job change completes.
#
# This is a critical timing window during task scheduling. If NEWJOB becomes
# +0 before we sample it, we take alternate path through NUDIRECT instead
# of normal CHANJOB. The comment "VERY RARE CASE" indicates this race
# condition seldom occurs in practice, but code handles it correctly.
#
# The activity light illumination signals to crew and ground controllers
# that AGC is processing jobs actively (not idle or hung).
# ============================================================================
NUCHANG2	INHINT			# QUICK... DONT LET NEWJOB CHANGE TO +0 .
					; Disable interrupts immediately - timing critical
					; Prevent NEWJOB from changing during this check
		CCS	NEWJOB		; Count, compare, skip: test NEWJOB value
					; CCS checks if NEWJOB positive, zero, or negative
		TCF	+3		# NEWJOB STILL PNZ
					; NEWJOB still positive non-zero: proceed normally
					; Skip the alternate path below
		RELINT			# NEWJOB HAS CHANGED TO +0. WAKE UP JOB
					; NEWJOB became +0: re-enable interrupts
					; Take alternate wake-up path for this edge case
		TCF	ADVAN +2	# VIA NUDIRECT.  (VERY RARE CASE.)
					; Transfer to NUDIRECT+2 alternate job dispatch
					; Handles the +0 case without full CHANJOB overhead

		CAF	TWO		; NEWJOB still valid: load constant 2
					; Bit pattern for activity light control
		EXTEND			; Enable extended addressing mode
		WOR	DSALMOUT	# TURN ON ACTIVITY LIGHT
					; Write OR to output channel DSALMOUT
					; Illuminates activity light on DSKY display
					; Visible crew feedback: computer actively working
		DXCH	LOC		# AND SAVE ADDRESS INFO FOR BENEFIT OF
					; Double exchange to save LOC and LOC+1
					; Preserves return address for sleeping job
		TCF	CHANJOB + 4	#  POSSIBLE SLEEPING JOB.
					; Jump into CHANJOB routine at entry +4
					; Bypasses initial setup, directly swaps core sets

# Page 1216
# TO WAKE UP A JOB, EACH CORE SET IS FOUND TO LOCATE ALL JOBS WHICH ARE ASLEEP.  IF THE FCADR IN THE
# LOC REGISTER OF ANY SUCH JOB MATCHES THAT SUPPLIED BY THE CALLER, THAT JOB IS AWAKENED.  IF NO JOB IS FOUND,
# LOCCTR IS SET TO -1 AND NO FURTHER ACTION TAKES PLACE.

; ============================================================================
; JOBWAKE2 - Core Set Scanner for Sleeping Job Wake-Up
;
; Scans all 7 core sets to locate sleeping jobs whose LOC address matches
; the wake-up address in NEWLOC. When a match is found, awakens that job
; by re-complementing its negative priority back to positive.
;
; This implements event-driven task synchronization. Jobs that called
; JOBSLEEP are waiting for specific events (I/O completion, timer expiry,
; external conditions). When the event occurs, JOBWAKE is called with the
; event's address, and this scanner finds all jobs sleeping on that address.
;
; During Apollo 11 mission, this mechanism enabled efficient CPU utilization:
; jobs sleep during I/O wait (landing radar data, IMU readings, DSKY input)
; rather than busy-waiting. When hardware completes operation, corresponding
; job awakens and processes result. This cooperative scheduling allowed AGC's
; limited 85 microsecond instruction time to support multiple concurrent tasks.
;
; ALGORITHM:
;   For each of 7 core sets (LOCCTR = 0, 12, 24, 36, 48, 60, 72):
;     Check PRIORITY: positive = active, negative = sleeping
;     If sleeping:
;       Compare core set's LOC with NEWLOC address
;       If match: re-complement PRIORITY (negative to positive) = awaken job
;       If no match: continue scanning
;     If active: skip to next core set (12 registers forward)
;   If no sleeping job found matching NEWLOC: set LOCCTR = -1
;
; ENTRY:
;   A: (arbitrary, will be saved in EXECTEM1)
;   NEWLOC: FCADR of address to match against sleeping jobs
;
; EXIT:
;   LOCCTR: Index to awakened core set (0-72), or -1 if no match
;   NEWPRIO: Priority value of awakened job
; ============================================================================
JOBWAKE2	TS	EXECTEM1
					; Save accumulator in temporary storage
					; Preserve caller's register state
		CAF	ZERO		# BEGIN CORE SET SCAN.
					; Load zero to initialize scanner
		TS	LOCCTR		; LOCCTR = 0: start at first core set (index 0)
					; Will increment by 12 for each core set scanned
		CAF	NO.CORES	; Load number of core sets to scan (7 core sets)
					; NO.CORES constant = 7 (defined elsewhere)
JOBWAKE4	TS	EXECTEM2	; Save remaining core sets count in EXECTEM2
					; Countdown: 7, 6, 5, 4, 3, 2, 1
		INDEX	LOCCTR		; Use LOCCTR as index to access core set registers
					; Indexed addressing: base + LOCCTR offset
		CCS	PRIORITY	# Count, compare, skip on indexed PRIORITY
					; Check sign of PRIORITY at current core set:
					;   Positive = active job (normal operation)
					;   Negative = sleeping job (waiting for event)
					;   Zero = vacant core set (no job assigned)
		TCF	JOBWAKE3	# ACTIVE JOB - CHECK NEXT CORE SET.
					; Priority positive: job is active, not sleeping
					; Skip to JOBWAKE3 to advance to next core set
COREINC		DEC	12		# 12 REGISTERS PER CORE SET.
					; Core set size constant: 12 words
					; Each core set contains: LOC, PRIORITY, BANKSET, etc.
		TCF	WAKETEST	# SLEEPING JOB - SEE IF CADR MATCHES.
					; Priority negative: job is sleeping on some event
					; Jump to WAKETEST to check if it's waiting for NEWLOC

JOBWAKE3	CAF	COREINC		; Load core set increment (12 registers)
					; Prepare to advance to next core set
		ADS	LOCCTR		; Add 12 to LOCCTR: advance to next core set
					; LOCCTR sequence: 0, 12, 24, 36, 48, 60, 72
					; Indexes to start of each of 7 core sets
		CCS	EXECTEM2	; Decrement and test remaining core sets counter
					; CCS decrements EXECTEM2 by 1, tests result
		TCF	JOBWAKE4	; More core sets to scan: loop back to JOBWAKE4
					; Continue scanning next core set
		CS	ONE		# EXIT IF SLEEPING JOB NOT FOUND.
					; Load -1: signal that no matching job found
					; -1 indicates wake-up failed (no job sleeping on NEWLOC)
		TS	LOCCTR		; Set LOCCTR = -1 to indicate failure
					; Caller can test LOCCTR to see if wake succeeded
		TCF	ENDFIND		; Exit scanner: no sleeping job matched NEWLOC
					; Transfer to ENDFIND cleanup routine

WAKETEST	CS	NEWLOC		; Complement NEWLOC (prepare for comparison)
					; CS inverts all bits: complement for subtraction
		INDEX	LOCCTR		; Use LOCCTR to index into sleeping job's core set
					; Access LOC register of sleeping job
		AD	LOC		; Add sleeping job's LOC to -NEWLOC
					; Effectively: LOC - NEWLOC
					; Result zero if addresses match
		EXTEND			; Enable extended instruction mode
		BZF	+2		# IF MATCH.
					; Branch on Zero to Fixed: if A = 0, skip +2 lines
					; Zero means LOC == NEWLOC: found matching job!
		TCF 	JOBWAKE3	# EXAMINE NEXT CORE SET IF NO MATCH.
					; Non-zero means LOC != NEWLOC: not this job
					; Continue scanning other core sets

					; *** MATCH FOUND: Wake up this sleeping job ***
		INDEX	LOCCTR		# RE-COMPLEMENT PRIORITY TO SHOW JOB AWAKE
					; Use LOCCTR to access matched core set's PRIORITY
		CS	PRIORITY	; Complement negative priority: make it positive
					; Sleeping job has negative priority
					; CS(negative) = positive: job now active
		TS	NEWPRIO		; Save awakened priority in NEWPRIO for caller
					; Caller may need to know awakened job's priority
		INDEX	LOCCTR		; Index to matched core set's PRIORITY again
		TS	PRIORITY	; Store positive priority back to core set
					; Job officially awakened: scheduler will run it
					; Job transitions: sleeping (neg) → active (pos)

		CS	FBANKMSK	# MAKE UP THE 2CADR OF THE WAKE ADDRESS
		MASK	NEWLOC		# USING THE CADR IN NEWLOC AND THE EBANK
		AD	2K		# HALF OF BBANK SAVED IN BANKSET.
		XCH	NEWLOC
		MASK	FBANKMSK
		INDEX	LOCCTR
		AD	BANKSET
		TS	NEWLOC +1

		CCS	LOCCTR		# SPECIAL TREATMENT IF THIS JOB WAS
		TCF	SETLOC		# ALREADY IN THE RUN (0) POSITION.
		TCF	SPECTEST

# Page 1217
# PRIORITY CHANGE. CHANGE THE CONTENTS OF PRIORITY AND SCAN FOR THE JOB OF HIGHEST PRIORITY.

; ============================================================================
; PRIOCH2 - DYNAMIC PRIORITY CHANGE
; ============================================================================
; When a job needs to change its own priority level (e.g., shifting from time-
; critical guidance computation to lower-priority housekeeping), this routine
; safely updates the PRIORITY register and rescans all 7 core sets to find
; the new highest-priority job. If the priority change still leaves this job
; as highest priority, execution continues. Otherwise, control transfers to
; the newly-determined highest-priority job.
;
; This mechanism allows the AGC to dynamically adapt to changing mission phase
; requirements. During Apollo 11's lunar descent, jobs would adjust their own
; priorities as mission phases transitioned (braking → approach → landing).
;
; Entry: A contains return address (LOC)
;        NEWPRIO contains the priority adjustment value
; Exit: May transfer control to EJSCAN if priority change causes job switch
; Modifies: LOC, BUF, PRIORITY, A

PRIOCH2		TS	LOC		; Save return address for this job
		CAF	ZERO		# SET FLAG TO TELL ENDJOB SCANNER IF THIS
		TS	BUF		# JOB IS STILL HIGHEST PRIORITY.
					; BUF=0 flags this as priority change (not end-of-job)
		CAF	LOW9		; Mask to extract priority value (bits 0-8)
		MASK	PRIORITY	; Get current priority component
		AD	NEWPRIO		; Add priority adjustment (may be + or -)
		TS	PRIORITY	; Update priority register with new value
		COM			; Complement for EJSCAN comparison logic
		TCF	JOBSLP2		# AND TO EJSCAN.
					; Jump to EJSCAN to find new highest-priority job

# Page 1218
# RELEASE THIS CORE SET AND VAC AREA AND SCAN FOR THE JOB OF HIGHEST ACTIVE PRIORITY.

; ============================================================================
; ENDJOB1 - END OF JOB (RELEASE CORE SET AND VAC AREA)
; ============================================================================
; When a job completes execution, this routine releases its allocated core set
; and VAC area (vector accumulator memory), making these resources available
; for new jobs. The routine then scans all 7 priority registers to determine
; which waiting job should execute next.
;
; NOVAC jobs (no VAC area): Simply release the core set
; FINDVAC jobs (with VAC area): Release both core set AND VAC area
;
; During Apollo 11 mission, jobs would end naturally after completing their
; purpose (e.g., IMU alignment complete, navigation update finished, display
; update done). Efficient resource reclamation prevented job queue saturation
; and reduced likelihood of 1201/1202 alarms.
;
; Entry: Job has completed execution
; Exit: Transfers control to next highest-priority waiting job (or DUMMYJOB if idle)
; Modifies: BUF, PRIORITY, L, A

ENDJOB1		INHINT			; Disable interrupts during job termination
		CS	ZERO		; Set BUF+1 = -0 to flag "active jobs exist"
		TS	BUF +1		; (used later to detect idle condition)
		XCH	PRIORITY	; Get current job's priority register
		MASK	LOW9		; Extract priority value (bits 0-8)
		TS	L		; Save in L for later processing

		CS	FAKEPRET	; -LOC(MPAC+6) constant
		AD	L		; Subtract from priority to check VAC status

		EXTEND
		BZMF	EJSCAN		# NOVAC ENDOFJOB
					; Branch if NOVAC job (result ≤ 0, no VAC to release)

					; For FINDVAC jobs: release VAC area pointer
		CCS	L		; Test priority value
		INDEX	A		; Use as index to VAC pointer table
		TS	0		; Clear the VAC area pointer (mark available)

; ============================================================================
; EJSCAN - PRIORITY SCAN ACROSS ALL 7 CORE SETS
; ============================================================================
; Systematically examines all 7 priority registers (PRIORITY through PRIORITY+72D)
; to find the job with the highest active priority. Each core set is checked in
; sequence, and the highest-priority waiting job is identified for execution.
;
; The 7 core sets represent the fundamental job queue structure of the AGC
; executive. Each core set can hold one job. Priorities determine execution
; order when multiple jobs are waiting. This scanning process is the heart
; of the cooperative multitasking scheduler.
;
; Core Set Layout (12 decimal word spacing):
;   PRIORITY +0D  = Core set 0 (highest priority slot)
;   PRIORITY +12D = Core set 1
;   PRIORITY +24D = Core set 2
;   PRIORITY +36D = Core set 3
;   PRIORITY +48D = Core set 4
;   PRIORITY +60D = Core set 5
;   PRIORITY +72D = Core set 6 (lowest priority slot)
;
; Uses EJ1 comparison subroutine to track highest priority found so far.
; BUF+1 accumulates the highest priority value encountered.
; BUF tracks the core set location of that highest-priority job.

EJSCAN		CCS	PRIORITY +12D	; Check core set 1 priority register
		TC	EJ1		; Positive = active job, compare priority
		TC	CCSHOLE		; +0 impossible (reserved value)
		TCF	+1		; Negative or -0 = vacant, continue scan

		CCS	PRIORITY +24D	# EXAMINE EACH PRIORITY REGISTER TO FIND
		TC	EJ1		# THE JOB OF HIGHEST ACTIVE PRIORITY.
		TC	CCSHOLE		; Core set 2 priority check
		TCF	+1

		CCS	PRIORITY +36D	; Core set 3 priority check
		TC	EJ1
-CCSPR		-CCS	PRIORITY	; Core set 0 priority check (special label for addressing)
		TCF	+1

		CCS	PRIORITY +48D	; Core set 4 priority check
		TC	EJ1
		TC	CCSHOLE
		TCF	+1

		CCS	PRIORITY +60D	; Core set 5 priority check
		TC	EJ1
		TC	CCSHOLE
		TCF	+1

		CCS	PRIORITY +72D	; Core set 6 priority check (lowest priority)
		TC	EJ1
		TC	CCSHOLE
		TCF	+1

# Page 1219
# EVALUATE THE RESULTS OF THE SCAN.

; ============================================================================
; SCAN RESULTS EVALUATION
; ============================================================================
; After examining all 7 core sets, evaluate the scan results to determine
; the next action:
;   1) No active jobs → Go to DUMMYJOB (idle loop, turn off activity light)
;   2) Priority change & still highest → Return to job (ENDPRCHG)
;   3) New highest-priority job found → Switch to it (CHANJOB)
;
; This is the decision point where the executive determines whether the
; AGC has work to do or should idle. During Apollo 11's mission, the
; computer alternated between intense computational bursts (guidance updates,
; navigation processing) and idle periods waiting for the next timed event.

		CCS	BUF +1		# SEE IF THERE ARE ANY ACTIVE JOBS WAITING
		TC	CCSHOLE		; Positive = active job found during scan
		TC	CCSHOLE		; +0 impossible

		TCF	+2		; Negative = no active jobs, go idle
		TCF	DUMMYJOB	; Transfer to idle loop
		CCS	BUF		# BUF IS ZERO IF THIS IS A PRIOCHNG AND
		TCF	+2		# CHANGED PRIORITY IS STILL HIGHEST.
		TCF	ENDPRCHG -1	; Priority change but job still highest: resume

					; Otherwise: new highest-priority job identified
		INDEX	A		# OTHERWISE, SET NEWJOB TO THE RELATIVE
		CAF	0 -1		# ADDRESS OF THE NEW JOB'S CORE SET.
		AD	-CCSPR		; Compute relative core set address
		TS	NEWJOB		; Store as target for job switch
		TCF	CHANJOB -2	; Transfer control to new job

; ============================================================================
; EJ1 - PRIORITY COMPARISON SUBROUTINE
; ============================================================================
; Called during EJSCAN to compare newly-examined priority value against the
; highest priority found so far. Implements a "running maximum" algorithm
; tracking which core set has the highest-priority waiting job.
;
; Entry: A contains priority value just read from core set
;        BUF+1 contains negative of highest priority seen so far (-0 initially)
;        Q contains return address (location of CCS instruction in EJSCAN)
; Logic: If (new priority) > (old high priority), update tracking registers
;        Otherwise, continue scan without update
; Modifies: BUF, BUF+1, BUF+2, A

EJ1		TS	BUF +2		; Save newly-examined priority value
		AD	BUF +1		# - OLD HIGH PRIORITY.
					; Add to negative of previous high (compare)
		CCS	A		; Test result of comparison
		CS	BUF 	+2	; Positive: new > old, negate new priority
		TCF	EJ2		# NEW HIGH PRIORITY.
					; Update tracking registers with new maximum
		NOOP			; Zero: equal priorities (unusual)
		INDEX	Q		; Negative: old ≥ new, no update needed
		TC	2		# PROCEED WITH SEARCH.
					; Return to next CCS instruction in EJSCAN

; ============================================================================
; EJ2 - UPDATE HIGHEST PRIORITY TRACKING
; ============================================================================
; Records the newly-discovered highest priority job location and value.
; BUF+1 = negative of highest priority (for comparison math)
; BUF = return address pointing to the CCS instruction that found this priority
;       (used later to compute core set index)

EJ2		TS	BUF +1		; Store negative of new highest priority
		EXTEND
		QXCH	BUF		# FOR LOCATING CCS PRIORITY + X INSTR.
					; Save Q (return address) to identify core set
		INDEX	BUF		; Return to EJSCAN using saved address
		TC	2		; Continue scanning remaining core sets

# Page 1220
# IDLING AND COMPUTER ACTIVITY (GREEN) LIGHT MAINTENANCE. THE IDLING ROUTINE IS NOT A JOB IN ITSELF,
# BUT RATHER A SUBROUTINE OF THE EXECUTIVE.

; ============================================================================
; DUMMYJOB - IDLE STATE HANDLER & ACTIVITY LIGHT CONTROL
; ============================================================================
; The AGC has no active jobs requiring immediate execution. Enter idle state
; while monitoring for new work arriving via interrupts (WAITLIST timers,
; KEYRUPT crew input, sensor data, uplink commands).
;
; This is the NORMAL operational state between mission events. The AGC was
; designed to spend most time in this idle loop, conserving power and awaiting
; the next guidance cycle, navigation update, or crew command.
;
; ACTIVITY LIGHT CONTROL:
; - Green "COMP ACTY" light on DSKY shows computer executing jobs
; - Light OFF during idle (power saving, crew knows system is waiting)
; - Light ON when jobs execute (crew knows system is computing)
; - Critical for crew monitoring: extinguished light during expected active
;   period would indicate computer failure
;
; IDLE LOOP OPERATION:
; 1. Set NEWJOB to -0 (idle flag, distinguishes from +0 priority)
; 2. Re-enable interrupts (RELINT) - allows WAITLIST, KEYRUPT to wake system
; 3. Turn off activity light
; 4. Poll NEWJOB repeatedly (ADVAN loop)
; 5. When interrupt creates new job, NEWJOB becomes non-zero
; 6. Exit idle, turn light back on, dispatch new job
;
; This is NOT the 1201/1202 alarm code path. Resource exhaustion alarms
; occur earlier in FINDVAC2/NOVAC2 before reaching DUMMYJOB. This routine
; handles successful job completion followed by normal idle state.
;
; During Apollo 11 descent, the system rapidly cycled between active jobs
; (guidance, navigation, display updates) and brief idle periods. The frequent
; 1201/1202 alarms indicated jobs were competing for resources faster than
; they could complete, preventing clean idle states.

		EBANK=	SELFRET		# SELF-CHECK STORAGE IN EBANK.

DUMMYJOB	CS	ZERO		# SET NEWJOB TO -0 FOR IDLING.
					; Negative zero distinguishes idle from
					; priority 0 job
		TS	NEWJOB		; Store idle flag
		RELINT			; Re-enable interrupts for wake-up events
					; (WAITLIST timers, KEYRUPT, UPRUPT)
		CS	TWO		# TURN OFF THE ACTIVITY LIGHT.
					; Bit 1 of DSALMOUT controls green COMP ACTY
		EXTEND
		WAND	DSALMOUT	; Write AND to channel 11 (clear bit)
					; Activity light extinguished = idle state

; ============================================================================
; ADVAN - IDLE LOOP (AWAIT NEW JOB)
; ============================================================================
; Tight polling loop checking NEWJOB for interrupt-created work.
; Interrupts (T4RUPT, KEYRUPT, etc.) call NOVAC/FINDVAC which sets NEWJOB
; to priority+address of new job. When NEWJOB becomes non-zero, exit idle.

ADVAN		CCS	NEWJOB		# IS A NEWJOB ACTIVE ?
					; CCS: +, +0, -0, -
		TCF	NUCHANG2	# YES... ONE REQUIRING A CHANGE JOB.
					; Positive = new job priority, do CHANJOB
		CAF	TWO		# NEW JOB ALREADY IN POSITION FOR
					; +0 result: job ready at NEWLOC
		TCF	NUDIRECT	# EXECUTION.
					; Skip CHANJOB, dispatch directly

; -0 result falls through to self-check dispatch (unused in normal idle)
; Self-check job invoked for diagnostic testing

		CA	SELFRET		; Load self-check return address
		TS	L		# PUT RETURN ADDRESS IN L.
		CAF	SELFBANK	; Load self-check bank number
		TCF	SUPDXCHZ + 1	# AND DISPATCH JOB.
					; Enter job without full CHANJOB setup

		EBANK=	SELFRET
SELFBANK	BBCON	SELFCHK		; Bank/address of self-check routine

; ============================================================================
; NUDIRECT - EXIT IDLE AND DISPATCH READY JOB
; ============================================================================
; New job address already loaded in NEWLOC by interrupt handler.
; Turn activity light back on and dispatch job without CHANJOB overhead.

NUDIRECT	EXTEND			# TURN THE GREEN LIGHT BACK ON.
					; Bit 1 = COMP ACTY illuminated
		WOR	DSALMOUT	; Write OR to channel 11 (set bit)
					; Crew sees computer actively computing
		DXCH	LOC		# JOBS STARTED IN THIS FASHION MUST BE
					; Load job address from NEWLOC to LOC/LOC+1
		TCF	SUPDXCHZ	; Dispatch without priority change
					; (job already in highest-priority core set)

		BLOCK	2		# IN FIXED-FIXED SO OTHERS MAY USE.
					; Fixed-fixed location allows cross-bank calls

		COUNT	02/EXEC

# SUPDXCHZ - ROUTINE TO TRANSFER TO SUPERBANK.
# CALLING SEQUENCE
#		TCF	SUPDXCHZ	# WITH 2CADR OF DESIRED LOCATION IN A + L.

; ============================================================================
; SUPDXCHZ - SUPERBANK TRANSFER DISPATCHER
; ============================================================================
; Transfers control to a job whose code resides in a superbank (bank > 7).
; AGC memory banking system:
; - Banks 0-7: Fixed-fixed and fixed-switched (normal addressing)
; - Banks 8-35+: Superbanks (require special SUPERBNK register setup)
;
; Entry: A contains bank number (upper half of 2CADR)
;        L contains address within bank (lower half of 2CADR)
; Operation:
; 1. Swap A and L (put address in A, bank in L)
; 2. Write bank number to SUPERBNK channel (switches memory bank)
; 3. Store bank in BBANK for tracking
; 4. Transfer control to address now in L
;
; Used by job dispatcher when starting jobs in superbank locations.
; Most executive routines are in lower banks; mission programs (P##)
; and complex guidance algorithms often reside in superbanks.

SUPDXCHZ	XCH	L		# BASIC.
					; Swap address to A, bank to L
+1		EXTEND			; Enable extended instruction
		WRITE	SUPERBNK	; Write L (bank number) to SUPERBNK channel
					; Hardware switches active memory bank
		TS	BBANK		; Store bank number in BBANK register
					; (software tracking of current bank)
		TC	L		; Transfer control to address in L
					; Job begins execution in selected superbank

; ============================================================================
; EXECUTIVE CONSTANTS
; ============================================================================

NEG100		OCT	77677		; -100 decimal (octal two's complement)
					; Used in priority calculations

; ============================================================================
; RESTART INTEGRATION
; ============================================================================
; The EXECUTIVE integrates with the AGC restart protection system to enable
; recovery from power transients, hardware glitches, or program alarms without
; losing mission-critical computational state.
;
; RESTART PHILOSOPHY:
; AGC operates in harsh space environment with cosmic ray hits, power
; fluctuations, and hardware faults. Rather than crash-and-reboot, the AGC
; implements continuous restart protection allowing recovery within 1-2
; execution cycles (~170-340ms).
;
; RESTART TABLES (see RESTART_TABLES.agc):
; - Jobs organized into restart groups (0-6) and phases (0-7)
; - Each phase spot contains job priority (or delta time) and 2CADR
; - Positive priority = FINDVAC job (needs VAC area)
; - Negative priority = NOVAC job (no VAC area)
; - Phase encoding allows restart system to restore jobs at correct point
;   in mission program execution
;
; RESTART SEQUENCE:
; 1. Transient/fault detected by hardware or self-check
; 2. Restart system reads current restart group/phase from erasable
; 3. RESTART_TABLES.agc consulted for jobs active in that phase
; 4. Jobs recreated via NOVAC/FINDVAC with saved priorities
; 5. Execution resumes within ~170-340ms of disruption
; 6. Mission continues with minimal computational loss
;
; EXECUTIVE ROLE IN RESTART:
; - NOVAC/FINDVAC provide standard job creation interface for restart system
; - Core set allocation provides isolated execution contexts
; - Priority management ensures mission-critical jobs resume first
; - CHANJOB mechanism allows restart to place jobs in correct execution state
;
; 1201/1202 ALARM RESTART PROTECTION:
; During Apollo 11 descent, the executive was frequently near saturation.
; When 1201/1202 alarms occurred, restart protection ensured that:
; - Active guidance computations preserved state
; - Navigation filter maintained accuracy
; - Display updates continued to crew
; - Engine control remained responsive
; Without restart protection, alarm conditions would have required abort.
; With restart protection, Mission Control had confidence to continue ("We're
; Go on that alarm" - Steve Bales, GUIDO, at 102:38:40 MET).
;
; RESTART DEMONSTRATION:
; The fact that Apollo 11 landed successfully despite 5+ program alarms during
; descent demonstrates the robustness of the restart system. Each alarm
; represented a momentary loss of computational capacity, but restart
; protection allowed immediate recovery and continuation of guidance.
;
; Armstrong and Aldrin trusted their lives to this code. It worked.



