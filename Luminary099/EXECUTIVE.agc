# Copyright:	Public domain.
# Filename:	EXECUTIVE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1103-1114
# Mod history:	2009-05-25 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-01-06 JL	Fixed pseudo-label indentation.
#		2011-05-08 JL	Removed workaround.

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

# ==============================================================================
# FILE: EXECUTIVE.agc
# MODULE: Core Operating System - Task Scheduler
# MISSION PHASE: All phases (continuous operation throughout mission)
#
# TL;DR: Implements the AGC's real-time cooperative multitasking executive 
#        scheduler, managing job priorities across seven core sets with VAC 
#        (vector accumulator) area allocation. This is the heart of the AGC 
#        operating system that schedules all mission programs, guidance 
#        calculations, and display updates. During Apollo 11's descent, this 
#        scheduler triggered the famous 1201/1202 program alarms when radar 
#        data processing overloaded the available job queue capacity, but 
#        restart protection allowed the landing to continue successfully.
#
# COMMENT-ONLY READERS: This file contains the "air traffic controller" of 
#        the AGC - it decides which program runs when. Understanding this 
#        helps explain how the computer juggled landing guidance, radar 
#        tracking, and crew displays all at once during the final 12 minutes 
#        to the lunar surface.
#
# CODE-ALONG READERS: Study the NOVAC (no-VAC) and FINDVAC (find-VAC) entry 
#        points to understand job request handling. The seven PRIORITY 
#        registers and five VAC area allocation registers are the foundation 
#        of the cooperative multitasking system. Line 147 shows the 1201 
#        alarm trigger when all VAC areas are exhausted.
# ==============================================================================

# Page 1103
		BLOCK	02

# ==============================================================================
# NOVAC - Create New Job Without VAC Area
# ==============================================================================
# COMMENT-ONLY READERS: The Executive scheduler manages all computational tasks
# running on the AGC. When the guidance computer needs to start a new task that
# doesn't require temporary workspace (called a VAC area), it uses NOVAC (NO VAC).
# This is the simplest way to schedule work - just tell the computer what needs
# to be done and at what priority level.
#
# CODE-ALONG READERS: NOVAC is one of two primary entry points into the Executive
# scheduler. It schedules a new job without allocating a VAC (Vector Accumulator)
# area from the interpretive stack. The caller provides:
#   - Priority in A register (lower numbers = higher priority)
#   - 2CADR (two-word address) of job entry point following the TC NOVAC instruction
# NOVAC is used for pure assembly language jobs that don't need interpreter workspace.
# ==============================================================================

# TO ENTER A JOB REQUEST REQUIRING NO VAC AREA:

		COUNT*	$$/EXEC
NOVAC		INHINT			# Disable interrupts during job scheduling
					# Caller provides priority in A register
		AD	FAKEPRET	# LOC(MPAC +6) - LOC(QPRET)
					# Add offset to convert priority encoding
		TS	NEWPRIO		# PRIORITY OF NEW JOB + NOVAC C(FIXLOC)
					# Store final priority for new job
					# Lower numerical values = higher priority
					# (Priority 0 = highest, used for critical tasks)

		EXTEND			# Next instruction is double-precision
		INDEX	Q		# Q WILL BE UNDISTURBED THROUGHOUT.
					# Q holds return address after TC NOVAC
					# Index by Q to fetch 2CADR from caller's code
		DCA	0		# 2CADR OF JOB ENTERED.
					# Fetch two-word address of job entry point
					# Format: Bank number + address within bank
		DXCH	NEWLOC		# Store job entry address in NEWLOC (double)
		CAF	EXECBANK	# Load Executive's fixed bank address
		XCH	FBANK		# Switch to Executive bank
		TS	EXECTEM1	# Save caller's bank for later restoration
		TCF	NOVAC2		# ENTER EXECUTIVE BANK.
					# Continue processing in Executive bank

# ==============================================================================
# FINDVAC - Create New Job With VAC Area
# ==============================================================================
# COMMENT-ONLY READERS: Most guidance computations (trajectory calculations,
# navigation updates, landing guidance) use complex vector and matrix mathematics.
# These computations need temporary workspace called a VAC (Vector Accumulator) area.
# FINDVAC finds an available workspace and schedules the job. During Apollo 11's
# descent, heavy use of FINDVAC for guidance calculations contributed to the
# famous 1202 program alarm when the computer became overloaded.
#
# CODE-ALONG READERS: FINDVAC is the second primary entry point into the Executive.
# It schedules a new job AND allocates a 44-word VAC area from the MPAC interpretive
# stack. VAC areas provide workspace for interpretive language operations (vector math,
# matrix operations, trigonometric functions). The caller provides:
#   - Priority in A register
#   - 2CADR of job entry point following the TC FINDVAC instruction
# FINDVAC is required for any job that uses TC INTPRET to enter interpretive mode.
# ==============================================================================

# TO ENTER A JOB REQUEST REQUIREING A VAC AREA -- E.G., ALL (PARTIALLY) INTERPRETIVE JOBS.

FINDVAC		INHINT			# Disable interrupts during job scheduling
		TS	NEWPRIO		# Store priority directly (already in A)
					# Priority determines scheduling order
		EXTEND			# Next instruction is double-precision
		INDEX	Q		# Index by return address in Q
					# Q points to 2CADR in caller's code
		DCA	0		# Fetch 2CADR of job entry point
					# Bank number + address within bank
SPVACIN		DXCH	NEWLOC		# Store job entry address in NEWLOC
					# SPVACIN: alternate entry when 2CADR
					# already loaded in A,L registers
		CAF	EXECBANK	# Load Executive's bank address
		XCH	FBANK		# Switch to Executive's fixed bank
		TCF	FINDVAC2	# OFF TO EXECUTIVE SWITCHED-BANK.
					# Continue to find available VAC area

# ==============================================================================
# SPVAC - Special Priority VAC Job Entry
# ==============================================================================
# COMMENT-ONLY READERS: SPVAC provides a specialized way to schedule high-priority
# computational tasks when the priority is already known and stored separately.
#
# CODE-ALONG READERS: SPVAC is an alternate entry to FINDVAC when:
#   - Priority is pre-stored in NEWPRIO (caller must INHINT first)
#   - 2CADR arrives in A,L registers (not from code following TC instruction)
# This entry point adjusts Q (return address) by -2 before continuing to SPVACIN.
# ==============================================================================

# TO ENTER A FINDVAC WITH THE PRIORITY IN NEWPRIO TO THE 2CADR ARRIVING IN A AND L:
# USERS OF SPVAC MUST INHINT BEFORE STORING IN NEWPRIO.

SPVAC		XCH	Q		# Exchange Q with A (save Q, get junk)
		AD	NEG2		# Subtract 2 from return address
					# Adjust Q because no 2CADR follows
		XCH	Q		# Restore adjusted Q
		TCF	SPVACIN		# Enter FINDVAC processing with 2CADR in A,L

# ==============================================================================
# CHANG1 - Suspend Basic Job for Higher Priority Service
# ==============================================================================
# COMMENT-ONLY READERS: During critical mission phases, some tasks must yield
# control to more urgent operations. CHANG1 allows a basic (native AGC code)
# job to voluntarily suspend itself so a higher priority job can run.
#
# CODE-ALONG READERS: CHANG1 suspends a basic (non-interpretive) job:
#   - Saves return address in L register (LXCH Q)
#   - Switches to Executive bank for job suspension processing
#   - Jumps to CHANJOB to complete suspension and schedule next job
# Used when a job determines it should yield to higher priority work.
# ==============================================================================

# TO SUSPEND A BASIC JOB SO A HIGHER PRIORITY JOB MAY BE SERVICED:

CHANG1		LXCH	Q		# Save return address in L
		CAF	EXECBANK	# Load Executive bank address
		XCH	BBANK		# Switch to Executive bank
		TCF	CHANJOB		# Suspend this job, run next priority job

# ==============================================================================
# CHANG2 - Suspend Interpretive Job
# ==============================================================================
# COMMENT-ONLY READERS: CHANG2 handles suspension for interpretive jobs (those
# running vector/matrix calculations using the AGC's interpretive language).
# These jobs use different state preservation than basic jobs.
#
# CODE-ALONG READERS: CHANG2 suspends interpretive language jobs:
#   - Negates LOC register (negative value marks job as interpretive)
#   - Stores in L register for job context preservation
#   - Has alternate entry at +2 for different calling conventions
#   - Switches to Executive bank and jumps to CHANJOB-1
# Interpretive jobs have different stack/state that requires special handling.
# ==============================================================================

# TO SUSPEND AN INTERPRETIVE JOB:

CHANG2		CS	LOC		# NEGATIVE LOC SHOWS JOB = INTERPRETIVE.
					# Negate location pointer
# ITRACE (4) REFERS TO "CHANG2"
		TS	L		# Store negated LOC in L
# Page 1104
 +2		CAF	EXECBANK	# Alternate entry: load Executive bank
		TS	BBANK		# Switch to Executive bank
		TCF	CHANJOB -1	# Suspend job (offset entry for interpretive)

# Page 1105
# ==============================================================================
# JOBSLEEP - Voluntarily Suspend Job Until Event Completion
# ==============================================================================
# COMMENT-ONLY READERS: Sometimes the spacecraft must wait for hardware to 
# complete an operation—radar measurements, sensor readings, engine responses.
# JOBSLEEP allows a program to pause itself until the hardware signals completion,
# freeing the computer to work on other tasks during the wait.
#
# CODE-ALONG READERS: JOBSLEEP suspends a job pending external event:
#   - Stores current location in LOC (preserving execution point)
#   - Switches to Executive bank for suspension processing
#   - Jumps to JOBSLP1 to complete suspension logic
# Used for I/O operations, sensor waits, hardware response delays. Job remains
# suspended until JOBWAKE is called (typically by interrupt handler when
# hardware signals completion). Critical for efficient multitasking when
# waiting for landing radar, IMU readings, or engine throttle responses.
# ==============================================================================

# TO VOLUNTARILY SUSPEND A JOB UNTIL THE COMPLETION OF SOME ANTICIPATED EVENT (I/O EVENT ETC.):

JOBSLEEP	TS	LOC		# Store return address/location
		CAF	EXECBANK	# Load Executive bank address
		TS	FBANK		# Switch to Executive bank
		TCF	JOBSLP1		# Complete suspension

# ==============================================================================
# JOBWAKE - Awaken a Previously Suspended Job
# ==============================================================================
# COMMENT-ONLY READERS: When the hardware operation completes (radar gets data,
# sensor finishes reading), an interrupt signals the computer. JOBWAKE responds
# by reactivating the sleeping job so it can continue processing with the new data.
#
# CODE-ALONG READERS: JOBWAKE reactivates a job suspended by JOBSLEEP:
#   - Disables interrupts (INHINT) for atomic state update
#   - Stores new execution location in NEWLOC (A register contains wake address)
#   - Adjusts return address (CS TWO, ADS Q) to exit via FINDVAC/NOVAC
#   - Switches to Executive bank and jumps to JOBWAKE2 for reactivation
# Called by interrupt handlers when I/O completes. Job is placed back into
# core set queue at its original priority. During lunar landing, this awakens
# guidance after radar data arrives or throttle command completes.
# ==============================================================================

# TO AWAKEN A JOB PUT TO SLEEP IN THE ABOVE FASHION:

JOBWAKE		INHINT			# Disable interrupts for atomic update
		TS	NEWLOC		# Store wake-up address
		CS	TWO		# EXIT IS VIA FINDVAC/NOVAC PROCEDURES.
					# Adjust return address
		ADS	Q		# Add -2 to Q for proper exit path
		CAF	EXECBANK	# Load Executive bank address
		XCH	FBANK		# Switch to Executive bank
		TCF	JOBWAKE2	# Complete job reactivation

# ==============================================================================
# PRIOCHNG - Change Priority of Currently Executing Job
# ==============================================================================
# COMMENT-ONLY READERS: Mission conditions change dynamically. A background task
# computing orbital parameters might suddenly need to yield to urgent guidance
# calculations during descent. PRIOCHNG allows a running program to adjust its
# own priority, ensuring critical work gets immediate attention while routine
# tasks wait their turn. This self-awareness prevents less important work from
# blocking mission-critical operations.
#
# CODE-ALONG READERS: PRIOCHNG changes priority of the currently executing job:
#   - New priority arrives in A register (caller sets before calling)
#   - INHINT disables interrupts for atomic priority update
#   - Stores new priority in NEWPRIO
#   - Switches to Executive bank (CAF EXECBANK, XCH BBANK)
#   - Saves previous bank in BANKSET for restoration
#   - Loads return address from Q register
#   - Jumps to PRIOCH2 to complete priority change and job rescheduling
# Returns to caller only when the job's new priority makes it highest priority
# again. If new priority is lower, job may be preempted by higher-priority work.
# During lunar landing, guidance jobs use this to dynamically adjust priority
# based on descent phase requirements.
# ==============================================================================

# TO CHANGE THE PRIORITY OF A JOB CURRENTLY UNDER EXECUTION:

PRIOCHNG	INHINT			# NEW PRIORITY ARRIVES IN A.  RETURNS TO
					# Disable interrupts for atomic update
		TS	NEWPRIO		# CALLER AS SOON AS NEW JOB PRIORITY IS
					# Store new priority value
		CAF	EXECBANK	# HIGHEST.  PREPARE FOR POSSIBLE BASIC-
					# Load Executive bank address
		XCH	BBANK		# STYLE CHANGE-JOB.
					# Switch to Executive bank
		TS	BANKSET		# Save previous bank for restoration
		CA	Q		# Load return address
		TCF	PRIOCH2		# Complete priority change and reschedule

# ==============================================================================
# ENDOFJOB - Terminate Currently Executing Job
# ==============================================================================
# COMMENT-ONLY READERS: When a program completes its work—orbital calculations
# finished, display update done, navigation state stored—it calls ENDOFJOB to
# gracefully terminate itself. The Executive removes it from the schedule,
# freeing its core set for new work. The computer immediately shifts attention
# to the next highest-priority task waiting to execute.
#
# CODE-ALONG READERS: ENDOFJOB terminates the currently executing job:
#   - Loads EXECBANK address (CAF EXECBANK)
#   - Switches to Executive bank (TS FBANK)
#   - Jumps to ENDJOB1 to complete termination processing
# Unlike CHANG1/CHANG2 (which suspend for later resumption), ENDOFJOB permanently
# removes the job from executive scheduling. The job's core set becomes available
# for new job allocation via NOVAC/FINDVAC. Control never returns to caller;
# the Executive immediately dispatches the next highest-priority job.
# ==============================================================================

# TO REMOVE A JOB FROM EXECUTIVE CONSIDERATIONS:

ENDOFJOB	CAF	EXECBANK	# Load Executive bank address
		TS	FBANK		# Switch to Executive bank
		TCF	ENDJOB1		# Complete job termination

# ==============================================================================
# ENDFIND - Return to Caller After VAC Area Allocation Complete
# ==============================================================================
# Restores caller's bank from EXECTEM1 and returns via Q+2
# (skipping one word, typical for AGC subroutine return conventions)
# ==============================================================================
ENDFIND		CA	EXECTEM1	# RETURN TO CALLER AFTER JOB ENTRY
		TS	FBANK		# COMPLETE. Restore caller's bank
		TCF	Q+2		# Return to caller (skip next word)

# Core Executive bank address constant pointing to FINDVAC2
EXECBANK	CADR	FINDVAC2

# Interpretive mode setup constant: offset from QPRET to MPAC+6
FAKEPRET	ADRES	MPAC -36D	# LOC(MPAC +6) - LOC(QPRET)

# ==============================================================================
# Page 1106
# FINDVAC2 - Locate Available VAC Area (Bank-Switched Continuation)
# ==============================================================================
# COMMENT-ONLY READERS: The Executive maintains five VAC (Vector Accumulator)
# areas—temporary work spaces for interpretive programs doing vector and matrix
# math. When a guidance calculation needs workspace, FINDVAC2 searches through
# all five VAC areas looking for one not currently in use. If all five are
# occupied, the computer triggers a 1201 program alarm—the same alarm that
# occurred during Apollo 11's descent when too many tasks tried to run
# simultaneously. Ground control recognized this as a non-critical overload,
# and the mission continued to successful landing.
#
# CODE-ALONG READERS: FINDVAC2 is the bank-01 continuation of FINDVAC:
#   - Saves caller's bank in EXECTEM1
#   - Sequentially tests VAC1USE through VAC5USE with CCS (count and skip)
#   - CCS returns +0 if VAC area is in use, -0 if available
#   - First available VAC found branches to VACFOUND
#   - If all VAC areas occupied: triggers BAILOUT1 with code 1201 (NO VAC AREAS)
# The 1201 alarm is recoverable—Executive can continue with existing jobs.
# During Apollo 11 descent, Steve Bales (GUIDO) recognized 1201 as safe to
# continue, making the critical "Go" call that kept the landing on track.
# ==============================================================================

		BANK	01
		COUNT*	$$/EXEC
FINDVAC2	TS	EXECTEM1	# (SAVE CALLER'S BANK FIRST.)
		CCS	VAC1USE		# Test VAC area 1
		TCF	VACFOUND	# +0 means in use, branch if available
		CCS	VAC2USE		# Test VAC area 2
		TCF	VACFOUND
		CCS	VAC3USE		# Test VAC area 3
		TCF	VACFOUND
		CCS	VAC4USE		# Test VAC area 4
		TCF	VACFOUND
		CCS	VAC5USE		# Test VAC area 5
		TCF	VACFOUND
		LXCH	EXECTEM1	# All VAC areas occupied!
		CA	Q		# Prepare for alarm
		TC	BAILOUT1	# Trigger program alarm
		OCT	1201		# NO VAC AREAS.

# ==============================================================================
# VACFOUND - Reserve Located VAC Area and Prepare Priority Word
# ==============================================================================
# COMMENT-ONLY READERS: The computer found an available VAC workspace. It marks
# this area as "occupied" so no other program tries to use it simultaneously,
# then records which VAC area was allocated for later reference when the job
# completes its vector calculations.
#
# CODE-ALONG READERS: VACFOUND reserves the found VAC area:
#   - AD TWO: Adjusts A from VAC test result to proper index
#   - INDEX A / LXCH 0 -1: Stores zero at VACnUSE register (marks as occupied)
#   - Stores VAC area starting address in low 9 bits of NEWPRIO
#   - Falls through to NOVAC2 to locate core set for job execution
# ==============================================================================
VACFOUND	AD	TWO		# RESERVE THIS VAC AREA BY STORING A ZERO
		ZL			# IN ITS VAC USE REGISTER AND STORE THE
		INDEX	A		# ADDRESS OF THE FIRST WORD OF IT IN THE
		LXCH	0 	-1	# LOW NINE BITS OF THE PRIORITY WORD.
		ADS	NEWPRIO		# Add VAC address to priority word

# ==============================================================================
# NOVAC2 - Find Available Core Set for Job Execution
# ==============================================================================
# COMMENT-ONLY READERS: Every executing job needs a "core set"—eleven registers
# holding the job's state (where it's executing, its priority, its work area).
# The Executive maintains seven core sets, allowing up to seven jobs to be
# scheduled simultaneously. NOVAC2 searches through all seven, looking for one
# not currently assigned to an active or dormant job.
#
# CODE-ALONG READERS: NOVAC2 (entry point from NOVAC, which needs core but not
# VAC area) searches the seven core sets:
#   - Initializes LOCCTR (core set index) to zero
#   - Loads NO.CORES (decimal 7) as loop counter
#   - NOVAC3: Tests each PRIORITY register with CCS
#   - PRIORITY = -0: Available core set, continues to CORFOUND
#   - PRIORITY > 0: Active job, continues search (NEXTCORE)
#   - PRIORITY < 0: Dormant job (suspended), continues search
# If all seven core sets occupied, system has reached maximum job capacity.
# ==============================================================================
NOVAC2		CAF	ZERO		# NOVAC ENTERS HERE.  FIND A CORE SET.
		TS	LOCCTR		# Initialize core set index
		CAF	NO.CORES	# SEVEN SETS OF ELEVEN REGISTERS EACH.
NOVAC3		TS	EXECTEM2	# Save loop counter
		INDEX	LOCCTR		# Index to current core set
		CCS	PRIORITY	# EACH PRIORITY REGISTER CONTAINS -0 IF
		TCF	NEXTCORE	# THE CORESPONDING CORE SET IS AVAILABLE.
NO.CORES	DEC	7		# Seven core sets total
		TCF	NEXTCORE	# AN ACTIVE JOB HAS A POSITIVE PRIORITY
					# BUT A DORMANT JOB'S PRIORITY IS NEGATIVE

# ==============================================================================
# Page 1107
# CORFOUND - Configure Located Core Set for New Job
# ==============================================================================
# COMMENT-ONLY READERS: The computer found an available core set. It now
# configures this set for the new job: storing the job's priority, setting up
# its workspace pointer, and preparing the overflow indicator. If this is core
# set zero (the highest-priority set), additional setup ensures proper tracking
# of the currently executing task.
#
# CODE-ALONG READERS: CORFOUND initializes the allocated core set:
#   - Stores NEWPRIO into PRIORITY register (job's priority and VAC address)
#   - Masks low 9 bits (VAC address) and stores in PUSHLOC (work area pointer)
#   - Special case for core set 0 (LOCCTR=0):
#     * Sets OVFIND to 0 (overflow indicator off)
#     * Copies PUSHLOC to FIXLOC (fixed-point work area tracking)
#   - Falls through to SPECTEST to check for job switching
# ==============================================================================
CORFOUND	CA	NEWPRIO		# SET THE PRIORITY OF THIS JOB IN THE CORE
		INDEX	LOCCTR		# SET'S PRIORITY REGISTER AND SET THE
		TS	PRIORITY	# JOB'S PUSH-DOWN POINTER AT THE BEGINNING
		MASK	LOW9		# OF THE WORK AREA AND OVERFLOW INDICATOR.
		INDEX	LOCCTR		# Index to current core set
		TS	PUSHLOC		# OFF TO PREPARE FOR INTERPRETIVE PROGRAMS.

		CCS	LOCCTR		# IF CORE SET ZERO IS BEING LOADED, SET UP
		TCF	SETLOC		# OVFIND AND FIXLOC IMMEDIATELY.
		TS	OVFIND		# Core set 0: Initialize overflow indicator
		CA	PUSHLOC		# Copy workspace pointer
		TS	FIXLOC		# to fixed-point location tracker

# ==============================================================================
# SPECTEST - Check for Active Job Requiring Immediate Switch
# ==============================================================================
# COMMENT-ONLY READERS: After setting up the new job, the computer checks
# whether this new job has higher priority than the currently executing task.
# If so, it immediately switches execution to the new job. Otherwise, the new
# job waits its turn while the current task continues.
#
# CODE-ALONG READERS: SPECTEST determines if job switch is needed:
#   - CCS NEWJOB: Tests if job switch already pending
#   - If NEWJOB = +0: Active job waiting, must set location registers
#   - If NEWJOB = -0: No switch pending, set NEWJOB = +0 and copy NEWLOC to LOC
#   - CCSHOLE: Handles intermediate CCS results (not zero cases)
# Falls through to ENDFIND if no switch needed, or SETLOC if switch required.
# ==============================================================================
SPECTEST	CCS	NEWJOB		# SEE IF ANY ACTIVE JOBS WAITING (RARE).
		TCF	SETLOC		# MUST BE AWAKENED OUT UNCHANGED JOB.
		TC	CCSHOLE		# Handle +0 case
		TC	CCSHOLE		# Handle -0 case
		TS	NEWJOB		# +0 SHOWS ACTIVE JOB ALREADY SET.
		DXCH	NEWLOC		# Copy new location
		DXCH	LOC		# to current location registers
		TCF	ENDFIND		# Return to caller

# ==============================================================================
# SETLOC - Establish Location Registers and Check Priority
# ==============================================================================
# COMMENT-ONLY READERS: The computer prepares to schedule the new job by
# recording where it should begin execution and comparing its priority against
# the currently running task to determine if an immediate switch is warranted.
#
# CODE-ALONG READERS: SETLOC sets up location tracking and priority comparison:
#   - DXCH NEWLOC / DXCH LOC: Copies new location to core set's LOC registers
#   - INDEX NEWJOB: Uses existing job's core set index
#   - CS PRIORITY / AD NEWPRIO: Compares priorities (negative if new is lower)
#   - EXTEND: Prepares for next operation (typically for extended instructions)
# Falls through to continue priority evaluation and job switching logic.
# ==============================================================================
SETLOC		DXCH	NEWLOC		# SET UP THE LOCATION REGISTERS FOR THIS
		INDEX	LOCCTR		# job's core set
		DXCH	LOC		# Copy to location registers
		INDEX	NEWJOB		# THIS INDEX INSTRUCTION INSURES THAT THE
		CS	PRIORITY	# HIGHEST ACTIVE PRIORITY WILL BE COMPARED
		AD	NEWPRIO		# WITH THE NEW PRIORITY TO SEE IF NEWJOB
		EXTEND			# SHOULD BE SET TO SIGNAL A SWITCH.
		BZMF	ENDFIND
		CA	LOCCTR		# LOCCTR IS LEFT SET AT THIS CORE SET IF
		TS	NEWJOB		# THE CALLER WANTS TO LOAD ANY MPAC
		TCF	ENDFIND		# REGISTERS, ETC.

; ==============================================================================
; NEXTCORE - Find Next Available Core Set (1202 Alarm Source)
; ==============================================================================
; COMMENT-ONLY READERS: If all seven "work areas" (core sets) are occupied,
; the AGC has no place to run a new job. This is the routine that triggers
; the famous 1202 PROGRAM ALARM. During Apollo 11's lunar descent on July 20,
; 1969, at approximately 102:38:26 mission time, this alarm occurred because
; the computer was overloaded with rendezvous radar data processing. Flight
; controller Steve Bales, backed by Jack Garman, recognized this as a non-
; critical overload and gave the "GO" decision that allowed the landing to
; continue. This code's restart protection system successfully recovered from
; the overload without losing critical landing guidance data.
;
; CODE-ALONG READERS: NEXTCORE advances to the next core set location:
;   - CAF COREINC / ADS LOCCTR: Increment core set pointer by fixed offset
;   - CCS EXECTEM2: Check if more core sets remain to try (countdown counter)
;   - TCF NOVAC3: If cores remain, continue searching
;   - BAILOUT1 / OCT 1202: If all seven cores occupied, trigger 1202 alarm
; The 1202 alarm code signals "EXECUTIVE OVERFLOW - NO CORE SETS" to crew.
; ==============================================================================

NEXTCORE	CAF	COREINC
		ADS	LOCCTR
		CCS	EXECTEM2
		TCF	NOVAC3
		LXCH	EXECTEM1
		CA	Q
		TC	BAILOUT1	# NO CORE SETS AVAILABLE.
		OCT	1202
# Page 1108
; ==============================================================================
; CHANJOB - Swap Active Core Sets (Context Switching)
; ==============================================================================
; COMMENT-ONLY READERS: When the computer needs to switch from one task to
; another (like switching from navigation calculations to landing guidance),
; it must swap all the "working registers" between the two tasks. This is
; similar to a modern operating system switching between programs. The routine
; saves everything about the current task and loads everything about the new
; task, allowing seamless resumption when switched back.
;
; CODE-ALONG READERS: CHANJOB performs a complete core set context switch:
;   Entry points at CHANJOB-2 and CHANJOB-1 allow pre-loading LOC and BANKSET
;   - INHINT: Disable interrupts during critical swap operation
;   - DXCH LOC (indexed): Swap location registers with new job's core set
;   - WRITE SUPERBNK: Switch to new job's super bank
;   - DXCH MPAC (indexed): Swap Multi-Purpose Accumulator working registers
; All seven core sets maintain complete independent working state, enabling
; the AGC to rapidly switch between multiple concurrent jobs.
; ==============================================================================
# THE FOLLOWING ROUTINE SWAPS CORE SET 0 WITH THAT WHOSE RELATIVE ADDRESS IS IN NEWJOB.

 -2		LXCH	LOC
 -1		CAE	BANKSET		# BANKSET, NOT BBANK, HAS RIGHT CONTENTS.
CHANJOB		INHINT
		EXTEND
		ROR	SUPERBNK	# PICK UP CURRENT SBANK FOR BBCON
		XCH	L		# LOC IN A AND BBCON IN L.
 +4		INDEX	NEWJOB		# SWAP LOC AND BANKSET.
		DXCH	LOC
		DXCH	LOC

		CAE	BANKSET
		EXTEND
		WRITE	SUPERBNK	# SET SBANK FOR NEW JOB.
		DXCH	MPAC		# SWAP MULTI-PURPOSE ACCUMULATOR AREAS.
		INDEX	NEWJOB
		DXCH	MPAC
		DXCH	MPAC
		DXCH	MPAC 	+2
		INDEX	NEWJOB
		DXCH	MPAC 	+2
		DXCH	MPAC	+2
		DXCH	MPAC 	+4
		INDEX	NEWJOB
		DXCH	MPAC 	+4
		DXCH	MPAC 	+4
		DXCH	MPAC 	+6
		INDEX	NEWJOB
		DXCH	MPAC 	+6
		DXCH	MPAC 	+6

		CAF	ZERO
		XCH	OVFIND		# MAKE PUSHLOC NEGATIVE IF OVFIND NZ.
		EXTEND
		BZF	+3
		CS	PUSHLOC
		TS	PUSHLOC

		DXCH	PUSHLOC
		INDEX	NEWJOB
		DXCH	PUSHLOC
		DXCH	PUSHLOC		# SWAPS PUSHLOC AND PRIORITY.
		CAF	LOW9		# SET FIXLOC TO BASE OF VAC AREA.
		MASK	PRIORITY
		TS	FIXLOC

		CCS	PUSHLOC		# SET OVERFLOW INDICATOR ACCORDING TO
		CAF	ZERO
		TCF	ENDPRCHG -1

# Page 1109
		CS	PUSHLOC
		TS	PUSHLOC
		CAF	ONE
		XCH	OVFIND
		TS	NEWJOB

ENDPRCHG	RELINT
		DXCH	LOC		# BASIC JOBS HAVE POSITIVE ADDRESSES, SO
		EXTEND			# DISPATCH WITH A DTCB.
		BZMF	+2		# IF INTERPRETIVE, SET UP EBANK, ETC.
		DTCB
# Page 1110
		COM			# EPILOGUE TO JOB CHANGE FOR INTERPRETIVE
		AD	ONE
		TS	LOC		# RESUME
		TCF	INTRSM

# COMPLETE JOBSLEEP PREPARATIONS.

JOBSLP1		INHINT
		CS	PRIORITY	# NNZ PRIORITY SHOWS JOB ASLEEP.
		TS	PRIORITY
		CAF	LOW7
		MASK	BBANK
		EXTEND
		ROR	SUPERBNK	# SAVE OLD SUPERBANK VALUE.
		TS	BANKSET
		CS	ZERO
JOBSLP2		TS	BUF 	+1	# HOLDS -- HIGHEST PRIORITY.
		TCF	EJSCAN		# SCAN FOR HIGHEST PRIORITY ALA ENDOFJOB.

NUCHANG2	INHINT			# QUICK... DON'T LET NEWJOB CHANGE TO +0.
		CCS	NEWJOB
		TCF	+3		# NEWJOB STILL PNZ
		RELINT			# NEW JOB HAS CHANGED TO +0.  WAKE UP JOB
		TCF	ADVAN 	+2	# VIA NUDIRECT.  (VERY RARE CASE.)

		CAF	TWO
		EXTEND
		WOR	DSALMOUT	# TURN ON ACTIVITY LIGHT
		DXCH	LOC		# AND SAVE ADDRESS INFO FOR BENEFIT OF
		TCF	CHANJOB +4	# 	POSSIBLE SLEEPINT JOB.

# Page 1111
# TO WAKE UP A JOB, EACH CORE SET IS FOUND TO LOCATE ALL JOBS WHICH ARE ASLEEP.  IF THE FCADR IN THE
# LOC REGISTER OF ANY SUCH JOB MATCHES THAT SUPPLIED BY THE CALLER, THAT JOB IS AWAKENED.  IF NO JOB IS FOUND,
# LOCCTR IS SET TO -1 AND NO FURTHER ACTION TAKES PLACE.

JOBWAKE2	TS	EXECTEM1
		CAF	ZERO		# BEGIN CORE SET SCAN
		TS	LOCCTR
		CAF	NO.CORES
JOBWAKE4	TS	EXECTEM2
		INDEX	LOCCTR
		CCS	PRIORITY
		TCF	JOBWAKE3	# ACTIVE JOB -- CHECK NEXT CORE SET.
COREINC		DEC	12		# 12 REGISTERS PER CORE SET.
		TCF	WAKETEST	# SLEEPING JOB -- SEE IF CADR MATCHES.

JOBWAKE3	CAF	COREINC
		ADS	LOCCTR
		CCS	EXECTEM2
		TCF	JOBWAKE4
		CS	ONE		# EXIT IF SLEEPIN JOB NOT FOUND.
		TS	LOCCTR
		TCF	ENDFIND

WAKETEST	CS	NEWLOC
		INDEX	LOCCTR
		AD	LOC
		EXTEND
		BZF	+2		# IF MATCH.
		TCF 	JOBWAKE3	# EXAMINE NEXT CORE SET IF NO MATCH.

		INDEX	LOCCTR		# RE-COMPLEMENT PRIORITY TO SHOW JOB AWAKE
		CS	PRIORITY
		TS	NEWPRIO
		INDEX	LOCCTR
		TS	PRIORITY

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

# Page 1112
# PRIORITY CHANGE.  CHANGE THE CONTENTS OF PRIORITY AND SCAN FOR THE JOB OF HIGHEST PRIORITY.

PRIOCH2		TS	LOC
		CAF	ZERO		# SET FLAG TO TELL ENDJOB SCANNER IF THIS
		TS	BUF		# JOB IS STILL HIGHEST PRIORITY.
		CAF	LOW9
		MASK	PRIORITY
		AD	NEWPRIO
		TS	PRIORITY
		COM
		TCF	JOBSLP2		# AND TO EJSCAN.

# Page 1113
# RELEASE THIS CORE SET AND VAC AREA AND SCAN FOR THE JOB OF HIGHEST ACTIVE PRIORITY.

ENDJOB1		INHINT
		CS	ZERO
		TS	BUF 	+1
		XCH	PRIORITY
		MASK	LOW9
		TS	L

		CS	FAKEPRET
		AD	L

		EXTEND
		BZMF	EJSCAN		# NOVAC ENDOFJOB

		CCS	L
		INDEX	A
		TS	0

; ==============================================================================
; EJSCAN - Scan for Highest Priority Waiting Job
; ==============================================================================
; COMMENT-ONLY READERS: After a job finishes, the computer must decide which
; waiting job to run next. This routine searches through all waiting jobs and
; picks the one with the highest priority number. Think of it like a hospital
; emergency room triage system - the most critical case gets handled first.
; If no jobs are waiting, the computer runs a "dummy" idle job that just waits
; for something to do.
;
; CODE-ALONG READERS: EJSCAN scans the PRIORITY table to find highest priority:
;   - Uses CCS to examine each PRIORITY register (13 total, spaced 12D apart)
;   - Calls EJ1 if priority is positive (job waiting)
;   - EJ1 compares priority against current highest, updates NEWJOB if higher
;   - Scan sequence: +12D, +24D, +36D, +0 (via -CCSPR), +48D, +60D, +72D
;   - Returns with NEWJOB pointing to highest priority job's core set
;   - If no jobs waiting, NEWJOB = -0 (triggers DUMMYJOB idle loop)
; Priority values: +0 (lowest) to +77777 (highest). Landing guidance and
; alarm handling have highest priorities to ensure mission-critical response.
; ==============================================================================

EJSCAN		CCS	PRIORITY +12D
		TC	EJ1
		TC	CCSHOLE
		TCF	+1

		CCS	PRIORITY +24D	# EXAMINE EACH PRIORITY REGISTER TO FIND
		TC	EJ1		# THE JOB OF HIGHEST ACTIVE PRIORITY.
		TC	CCSHOLE
		TCF	+1

		CCS	PRIORITY +36D
		TC	EJ1
-CCSPR		-CCS	PRIORITY
		TCF	+1

		CCS	PRIORITY +48D
		TC	EJ1
		TC	CCSHOLE
		TCF	+1

		CCS	PRIORITY +60D
		TC	EJ1
		TC	CCSHOLE
		TCF	+1

		CCS	PRIORITY +72D
		TC	EJ1
		TC	CCSHOLE
		TCF	+1

		CCS	PRIORITY +84D
# Page 1114
		TC	EJ1
		TC	CCSHOLE
		TCF	+1

# Page 1115
# EVALUATE THE RESULTS OF THE SCAN.

		CCS	BUF 	+1	# SEE IF THERE ARE ANY ACTIVE JOBS WAITING
		TC	CCSHOLE
		TC	CCSHOLE

		TCF	+2
		TCF	DUMMYJOB
		CCS	BUF		# BUF IS ZERO IS THIS IS A PRIOCHNG AND
		TCF	+2		# CHANGED PRIORITY IS STILL HIGHEST.
		TCF	ENDPRCHG -1

		INDEX	A		# OTHERWISE, SET NEWJOB TO THE RELATIVE
		CAF	0 	-1	# ADDRESS OF THE NEW JOB'S CORE SET.
		AD	-CCSPR
		TS	NEWJOB
		TCF	CHANJOB -2

; ==============================================================================
; EJ1 - Priority Comparison Helper for EJSCAN
; ==============================================================================
; COMMENT-ONLY READERS: When scanning for the highest priority job, this helper
; routine compares each job's priority against the current champion. If a job
; has higher priority than the current winner, it becomes the new champion.
; It's like updating the leader in a race as each contestant crosses a checkpoint.
;
; CODE-ALONG READERS: EJ1 performs priority comparison logic:
;   - Entry: A contains priority value being examined
;   - TS BUF+2: Save priority value for comparison
;   - AD BUF+1: Add negative of previous high priority (performs subtraction)
;   - CCS A: Test if new priority is higher (positive result means higher)
;   - If higher: CS BUF+2 to get negative priority, jump to EJ2 to update
;   - If not higher: Return via INDEX Q / TC 2 to continue scan
; BUF+1 stores negative of highest priority found so far, enabling efficient
; comparison via addition. This pattern minimizes instruction count in the
; time-critical priority scan loop.
; ==============================================================================

EJ1		TS	BUF 	+2
		AD	BUF 	+1	# - OLD HIGH PRIORITY.
		CCS	A
		CS	BUF 	+2
		TCF	EJ2		# NEW HIGH PRIORITY.
		NOOP
		INDEX	Q
		TC	2		# PROCEED WITH SEARCH.

; ==============================================================================
; EJ2 - Update Highest Priority Job Found
; ==============================================================================
; COMMENT-ONLY READERS: When a higher-priority job is found during the scan,
; this routine records it as the new winner. It saves the job's priority and
; location, then continues searching the remaining jobs to see if an even
; higher priority job exists.
;
; CODE-ALONG READERS: EJ2 updates the champion job information:
;   - Entry: A contains negative of new highest priority
;   - TS BUF+1: Store negative priority (used for future comparisons in EJ1)
;   - EXTEND / QXCH BUF: Save return address Q into BUF (points to CCS instr)
;   - INDEX BUF / TC 2: Return to scanning loop two instructions after CCS
; The return address in BUF points to the "CCS PRIORITY + X" instruction that
; found this priority. By indexing BUF and executing TC 2, we return to the
; next scan step. This indirect return mechanism enables the unrolled scan loop.
; ==============================================================================

EJ2		TS	BUF 	+1
		EXTEND
		QXCH	BUF		# FOR LOCATING CCS PRIORITY + X INSTR.
		INDEX	BUF
		TC	2

# Page 1116
# IDLING AND COMPUTER ACTIVITY (GREEN) LIGHT MAINTENANCE. THE IDLING ROUTINE IS NOT A JOB IN ITSELF,
# BUT RATHER A SUBROUTINE OF THE EXECUTIVE.

		EBANK=	SELFRET		# SELF-CHECK STORAGE IN EBANK.

; ==============================================================================
; DUMMYJOB - Executive Idle Loop and Activity Light Management
; ==============================================================================
; COMMENT-ONLY READERS: When no jobs are ready to run, the AGC enters an idle
; state. This routine turns off the computer activity light on the DSKY, signaling
; to the crew that the computer is waiting for work. It then loops, checking for
; new jobs. During Apollo 11's descent, you would see the activity light flicker
; as the computer rapidly switched between processing guidance calculations and
; brief idle moments. The crew monitored this light as an indicator that the
; computer was working normally.
;
; CODE-ALONG READERS: DUMMYJOB implements the executive idle state:
;   - CS ZERO: Create -0 value (all bits set)
;   - TS NEWJOB: Store -0 to indicate idle state (distinguishes from +0)
;   - RELINT: Enable interrupts (allows T4RUPT to schedule new jobs)
;   - CS TWO / EXTEND / WAND DSALMOUT: Turn off bit 1 (activity light)
;   - Fall through to ADVAN: Begin checking for new jobs
; The activity light (DSALMOUT bit 1) provides crew visibility into computer
; workload. During heavy computation (landing guidance), the light stays on
; continuously. During coast phases, it flickers as jobs arrive sporadically.
; ==============================================================================

DUMMYJOB	CS	ZERO		# SET NEWJOB TO -0 FOR IDLING.
		TS	NEWJOB
		RELINT
		CS	TWO		# TURN OFF THE ACTIVITY LIGHT.
		EXTEND
		WAND	DSALMOUT

; ==============================================================================
; ADVAN - Advance to Next Job (Check for New Work)
; ==============================================================================
; COMMENT-ONLY READERS: This is the executive's main loop. After completing one
; job or while idle, the computer checks if new work has arrived. If a job is
; waiting, the executive switches to it. If not, it keeps checking. During the
; lunar landing, this loop ran continuously, ensuring that every guidance
; calculation, display update, and throttle command was processed immediately
; as it became ready.
;
; CODE-ALONG READERS: ADVAN checks NEWJOB and dispatches accordingly:
;   - CCS NEWJOB: Test NEWJOB value (positive, zero, or negative)
;   - If positive: New job needs core set switch -> NUCHANG2
;   - If +0: Job already in current core set -> CAF TWO / TCF NUDIRECT
;   - If -0: Still idle -> fall through to continue idle loop
;   - If negative: Should not occur (indicates error state)
; NEWJOB encoding: positive = core set number (1-7), +0 = same core, -0 = idle.
; This three-way test enables efficient dispatch without additional comparisons.
; ==============================================================================

ADVAN		CCS	NEWJOB		# IS THE NEWJOB ACTIVE?
		TCF	NUCHANG2	# YES... ONE REQUIRING A CHANGE JOB.
		CAF	TWO		# NEW JOB ALREADY IN POSITION FOR
		TCF	NUDIRECT	# EXECUTION

; ------------------------------------------------------------------------------
; NEWJOB Dispatch Paths:
;   - Positive: Job in different core set -> NUCHANG2 (change job)
;   - +0: Job in current core set -> NUDIRECT (dispatch immediately)
;   - -0: Idle state, fall through below
; During Apollo 11's descent, the executive rapidly switched between guidance
; jobs in different core sets, executing this dispatch logic thousands of times
; per second as the computer balanced landing calculations, display updates, and
; throttle control commands.
; ------------------------------------------------------------------------------

; ==============================================================================
; Self-Check Dispatch (Fall-Through from -0 NEWJOB)
; ==============================================================================
; COMMENT-ONLY READERS: When the computer is idle (NEWJOB = -0), this code
; checks if the self-check diagnostic routine should run. Self-check runs in
; background during idle time to verify the computer is functioning correctly.
; During the Apollo 11 mission, self-check continuously validated memory and
; instruction execution, providing confidence that the guidance computer remained
; healthy throughout the critical descent and landing.
;
; CODE-ALONG READERS: Self-check dispatch mechanism:
;   - CA SELFRET: Load self-check return address from erasable memory
;   - TS L: Store return address in L register (lower half of 2CADR)
;   - CAF SELFBANK: Load self-check bank number (upper half of 2CADR)
;   - TCF SUPDXCHZ +1: Transfer to superbank dispatcher (skip XCH L)
; This constructs a 2CADR (two-word address: bank + address) pointing to the
; self-check routine and dispatches it via the bank-switching mechanism.
; ==============================================================================

		CA	SELFRET
		TS	L		# PUT RETURN ADDRESS IN L.
		CAF	SELFBANK
		TCF	SUPDXCHZ +1	# AND DISPATCH JOB.

		EBANK=	SELFRET
SELFBANK	BBCON	SELFCHK		# Bank and address of self-check routine

; ==============================================================================
; NUDIRECT - Direct Job Dispatch (Same Core Set)
; ==============================================================================
; COMMENT-ONLY READERS: When a new job is ready and already in the current core
; set, this routine dispatches it immediately. It turns on the green computer
; activity light to show the crew that the AGC is working, then transfers control
; to the job. During the lunar landing, you would see this light illuminate as
; the computer executed guidance calculations, processed radar data, and updated
; the DSKY displays in rapid succession.
;
; CODE-ALONG READERS: NUDIRECT dispatches job without core set change:
;   - Entry: A contains TWO (loaded by caller at line 928)
;   - EXTEND / WOR DSALMOUT: Turn on bit 1 (activity light) in DSALMOUT
;   - DXCH LOC: Load job address from LOC (2CADR stored by NOVAC/FINDVAC)
;   - TCF SUPDXCHZ: Dispatch via superbank transfer
; The activity light (DSALMOUT bit 1) provides crew feedback. When continuously
; lit during descent, it confirmed the computer was processing the heavy guidance
; workload. Flickering indicated lighter computational loads during coast phases.
; ==============================================================================

NUDIRECT	EXTEND			# TURN THE GREEN LIGHT BACK ON.
		WOR	DSALMOUT
		DXCH	LOC		# JOBS STARTED IN THIS FASHION MUST BE
		TCF	SUPDXCHZ

		BLOCK	2		# IN FIXED-FIXED SO OTHERS MAY USE.

		COUNT*	$$/EXEC

; ==============================================================================
; SUPDXCHZ - Superbank Transfer Routine
; ==============================================================================
; COMMENT-ONLY READERS: This is the final step in dispatching a job to execute.
; The AGC's memory is organized into "banks" (like chapters in a book), and this
; routine switches the computer to the correct memory bank before jumping to the
; job's starting address. Every time the computer switches from one task to
; another during the mission - from guidance to display updates to navigation -
; this routine executes to make the memory bank transition. During Apollo 11's
; landing, this routine executed thousands of times as the computer juggled all
; the systems needed to safely reach the lunar surface.
;
; CODE-ALONG READERS: SUPDXCHZ performs bank-switching and transfer:
; Entry conditions:
;   - A register: Bank number (high-order address bits)
;   - L register: Address within bank (low-order address bits)
;   - Together A+L form a "2CADR" (two-word address)
;
; Instruction sequence:
;   SUPDXCHZ: XCH L         - Swap A and L (bank now in L, address now in A)
;   +1: EXTEND              - Next instruction is extended (I/O operation)
;   WRITE SUPERBNK          - Write bank number to SUPERBNK hardware register
;   TS BBANK                - Store bank number in BBANK for interrupt recovery
;   TC L                    - Transfer control to address (job entry point)
;
; The +1 entry point skips the XCH L for self-check dispatch (line 964) where
; the 2CADR is already properly ordered. This saves one instruction cycle.
;
; SUPERBNK write: Hardware register that physically switches memory banks
; BBANK storage: Software copy used by interrupt handlers to restore bank state
; TC L: Final transfer executes job at computed address in selected bank
;
; This is one of the most frequently executed routines in the entire AGC,
; running every time the executive switches between jobs. Its efficiency was
; critical to the real-time performance during time-critical mission phases.
; ==============================================================================

# SUPDXCHZ -- ROUTINE TO TRANSFER TO SUPEBANK.
# CALLING SEQUENCE:
#		TCF	SUPDXCHZ	# WITH 2CADR OF DESIRED LOCATION IN A + L.

SUPDXCHZ	XCH	L		# BASIC.
 +1		EXTEND			# +1 entry: skip XCH when already ordered
		WRITE	SUPERBNK	# Switch memory bank hardware register
		TS	BBANK		# Save bank for interrupt recovery
		TC	L		# Transfer to job entry point

; ------------------------------------------------------------------------------
; NEG100: Constant used by executive scheduling algorithms
; Value: Octal 77677 = Decimal -100 (two's complement 15-bit representation)
; Purpose: Threshold value for priority comparisons and time calculations
; ------------------------------------------------------------------------------
NEG100		OCT	77677

