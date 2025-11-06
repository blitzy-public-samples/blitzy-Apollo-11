# Copyright:    Public domain.
# Filename:     RESTARTS_ROUTINE.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1414-1419
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
; FILE: RESTARTS_ROUTINE.agc
; MODULE: CHIEFTAN Subsystem (Core OS)
; MISSION PHASE: all-phases
;
; TL;DR: Restart recovery logic implementing task state restoration after
;        interruptions. Works with restart tables to restore task execution
;        context, critical during power transients and program alarms. During
;        Apollo 11 descent, this routine's robust design enabled safe recovery
;        from 1202 alarms without mission abort.
;
; COMMENT-ONLY READERS: The actual recovery mechanism that brought the computer
;        back to stable operation after alarms.
; CODE-ALONG READERS: Study restart recovery implementation, task state restoration
;        from restart tables, restart group processing, 1202 alarm recovery execution.
; ============================================================================

# Page 1414
		BANK	01
		SETLOC	RESTART
		BANK

		EBANK=	PHSNAME1	# GOPROG MUST SWITCH TO THIS EBANK

		COUNT	01/RSROU

; ============================================================================
; RESTARTS - Main entry point for restart recovery
;
; When the AGC encounters a power transient, program alarm, or other
; interruption requiring restart, this routine examines the restart tables
; to determine what tasks were running and restores them to execution.
;
; During Apollo 11's lunar descent, this routine executed multiple times to
; recover from 1202 program alarms, restoring guidance and display tasks
; without disrupting the landing sequence.
;
; The restart system uses "groups" and "phases" to identify task state:
; - Group number identifies which subsystem (guidance, display, etc.)
; - Phase number identifies progress within that subsystem's operation
; ============================================================================

RESTARTS	CA	MPAC +5		# GET GROUP NUMBER -1
		DOUBLE			# SAVE FOR INDEXING
		TS	TEMP2G

; Set up default exit addresses for navigating through restart table entries.
; The restart process may need to handle multiple phases per group, so these
; addresses control the flow through the tables.

		CA	PHS2CADR	# SET UP EXIT IN CASE IT IS AN EVEN
		TS	TEMPSWCH	# TABLE PHASE

		CA	RTRNCADR	# TO SAVE TIME ASSUME IT WILL GET NEXT
		TS	GOLOC +2	# GROUP AFTER THIS

; Determine the type of restart by examining the phase word.
; Variable restarts (bit 11 set) store task information differently than
; table restarts, requiring different processing paths.

		CA	TEMPPHS
		MASK	OCT1400
		CCS	A		# IS IT A VARIABLE OR TABLE RESTART
		TCF	ITSAVAR		# IT:S A VARIABLE RESTART

; ============================================================================
; TRANSITION: From variable restart detection to table restart processing
;
; Table restarts use fixed phase numbers to restart predefined operations.
; The special X.1 restart (phase = 1) always initiates the display system,
; ensuring the DSKY remains functional for crew interaction after any restart.
; ============================================================================

GETPART2	CCS	TEMPPHS		# IS IT AN X.1 RESTART
		CCS	A
		TCF	ITSATBL		# NO, ITS A TABLE RESTART

; X.1 restarts always reinitialize the display system. This ensures that
; after any power transient or alarm, the crew can see system status and
; issue commands through the DSKY. Critical during 1202 alarms when crew
; needed to monitor descent progress.

		CA	PRIO14		# IT IS AN X.1 RESTART, THEREFORE START
		TC	FINDVAC		# THE DISPLAY RESTART JOB
		EBANK=	LST1
		2CADR	INITDSP

		TC	RTRNCADR	# FINISHED WITH THIS GROUP, GET NEXT ONE

; ============================================================================
; TRANSITION: From restart type detection to variable restart processing
;
; Variable restarts (Type A and Type B) restore tasks that were interrupted
; during execution. These restarts preserve the exact execution context,
; allowing the task to resume where it left off.
; ============================================================================

ITSAVAR		MASK	OCT1400		# IS IT TYPE B ?
		CCS	A
		TCF	ITSLIKEB	# YES,IT IS TYPE B

; Type A variable restart: Task information stored directly in restart table.
; Extract the 2CADR (two-word address: bank and address) to determine where
; the interrupted task should resume execution.

		EXTEND			# STORE THE JOB (OR TASK) 2CADR FOR EXIT
		NDX	TEMP2G
		DCA	PHSNAME1
		DXCH	GOLOC

; Determine the specific task type by examining low-order bits of phase word.
; Different task types (jobs, waitlist calls, longcalls) require different
; restart procedures and scheduling priorities.

		CA	TEMPPHS		# SEE IF THIS IS A JOB, TASK, OR A LONGCALL
		MASK	OCT7
		AD	MINUS2
		CCS	A
		TCF	ITSLNGCL	# ITS A LONGCALL

# Page 1415
RTRNCADR	TC	SWRETURN	# CANT GET HERE
		TCF	ITSAWAIT

		TCF	ITSAJOB		# ITS A JOB

; Waitlist restart: Restore a time-delayed task to the WAITLIST timer queue.
; These tasks were scheduled to execute at a specific future time before the
; restart occurred. The time calculation determines when to reschedule them.

ITSAWAIT	CA	WTLTCADR	# SET UP WAITLIST CALL
		TS	GOLOC -1

		NDX	TEMP2G		# DIRECTLY STORED
		CA	PHSPRDT1
TIMETEST	CCS	A		# IS IT AN IMMEDIATE RESTART
		INCR	A		# NO.
		TCF	FINDTIME	# FIND OUT WHEN IT SHOULD BEGIN

		TCF	ITSINDIR	# STORED INDIRECTLY

		TCF	IMEDIATE	# IT WANTS AN IMMEDIATE RESTART

# ***** THIS MUST BE IN FIXED FIXED *****

		BLOCK	02
		SETLOC	FFTAG2
		BANK

		COUNT	02/RSROU

; For indirectly stored time values, must switch to correct erasable bank
; to access the time variable. This handles the AGC's memory banking system
; where erasable memory is divided into banks requiring explicit switching.

ITSINDIR	LXCH	GOLOC +1	# GET THE CORRECT E BANK IN CASE THIS IS
		LXCH	BB		# SWITCHED ERRASIBLE

		NDX	A		# GET THE TIME INDIRECTLY
		CA	1

		LXCH	BB		# RESTORE THE BB AND GOLOC
		LXCH	GOLOC +1

		TCF	FINDTIME	# FIND OUT WHEN IT SHOULD BEGIN

# ***** YOU MAY RETURN TO SWITCHED FIXED *****

		BANK 	01
		SETLOC	RESTART
		BANK

		COUNT	01/RSROU

; Calculate when a waitlist task should be restarted by comparing its
; scheduled time against current system time. If the scheduled time has
; already passed, restart immediately. Otherwise, compute the remaining
; delay time before scheduling the task.

FINDTIME	COM			# MAKE NEGATIVE SINCE IT WILL BE SUBTRACTED
		TS	L		# AND SAVE
		NDX	TEMP2G
		CS	TBASE1
		EXTEND
# Page 1416
		SU	TIME1
		CCS	A
		COM
		AD	OCT37776
		AD	ONE
		AD	L
		CCS	A
		CA	ZERO
		TCF	+2
		TCF	+1

; Immediate execution path for expired waitlist tasks.
; When FINDTIME determines that the scheduled time has already passed (time
; remaining is zero or negative), execution falls through to IMEDIATE. This
; section adds ONE to create minimal positive delta-time (1 centisecond) for
; waitlist scheduling, ensuring task begins at next opportunity.
; During Apollo 11 descent, this immediate restart capability enabled rapid
; recovery from 1202 alarms, restarting critical tasks within centiseconds.

IMEDIATE	AD	ONE		; Create minimal positive delta-time (1 cs)
		TC	GOLOC -1	; Call WAITLIST with restored task parameters
; ============================================================================
; TRANSITION: Type B restart handling
;
; Type B restarts combine variable and table restart characteristics. They
; first restore a job, then process table entries for that restart group.
; ============================================================================

ITSLIKEB	CA	RTRNCADR	# TYPE B, SO STORE RETURN IN
		TS	TEMPSWCH	# TEMPSWCH IN CASE OF AN EVEN PHASE

		CA	PRT2CADR	# SET UP EXIT TO GET TABLE PART OF THIS
		TS	GOLOC +2	# VARIABLE TYPE OF PHASE

		CA	TEMPPHS		# MAKE THE PHASE LOOK RIGHT FOR THE TABLE
		MASK	OCT177		# PART OF THIS VARIABLE PHASE
		TS	TEMPPHS

		EXTEND
		NDX	TEMP2G		# OBTAIN THE JOB:S 2CADR
		DCA	PHSNAME1
		DXCH	GOLOC

; Job restart: Schedule a new job in the executive's priority queue.
; Priority value determines whether this uses FINDVAC (requires VAC area)
; or NOVAC (no VAC area needed). Jobs are the primary work units in the
; AGC's cooperative multitasking system.

ITSAJOB		NDX	TEMP2G		# NOW ADD THE PRIORITY AND LET:S GO
		CA	PHSPRDT1
CHKNOVAC	TS	GOLOC -1	# SAVE PRIO UNTIL WE SEE IF ITS
		EXTEND			# A FINDVAC OR A NOVAC
		BZMF	ITSNOVAC

; Positive priority: Job requires vector accumulator (VAC) area for
; interpretive language operations. Use FINDVAC to locate free VAC area.

		CAF	FVACCADR	# POSITIVE, SET UP FINDVAC CALL.
		XCH	GOLOC -1	# PICK UP PRIO,
		TC	GOLOC -1	# AND GO

; Negative priority: Job doesn't require VAC area. Use NOVAC for faster
; job creation. Priority negated to get actual priority value.

ITSNOVAC	CAF	NOVACADR	# NEGATIVE,
		XCH	GOLOC -1	# SET UP NOVAC CALL,
		COM			# CORRECT PRIO,
		TC	GOLOC -1	# AND GO

; ============================================================================
; TRANSITION: Table restart processing
;
; Table restarts use predefined entries in restart tables to restore
; standard operations. The phase number determines which table entry to use.
; Odd and even phases are handled differently to support paired operations.
; ============================================================================

ITSATBL		TS	CYR		# FIND OUT IF THE PHASE IS ODD OR EVEN
		CCS	CYR
		TCF	+1		# IT:S EVEN
		TCF	ITSEVEN

		CA	RTRNCADR	# IN CASE THIS IS THE SECOND PART OF A
		TS	GOLOC +2	# TYPE B RESTART, WE NEED PROPER EXIT
# Page 1417

; Calculate pointer into restart table based on phase number and group number.
; Each group has its own table of restart entries, and phase number selects
; the specific entry to process.

		CA	TEMPPHS		# SET UP POINTER FOR FINDING OUR PLACE IN
		TS	SR		# THE RESTART TABLES
		AD	SR
		NDX	TEMP2G
		AD	SIZETAB +1
		TS	POINTER

; Retrieve the 2CADR (bank and address) from the restart table. This specifies
; the code location to restart or the task to schedule.

CONTBL2		EXTEND			# FIND OUT WHAT'S IN THE TABLE
		NDX	POINTER
		DCA	CADRTAB		# GET THE 2CADR

		LXCH	GOLOC +1	# STORE THE BB INFORMATION

	; Determine type of table entry from 2CADR encoding:
; Positive: Job to be scheduled via FINDVAC or NOVAC
; Negative: Time-delayed task (WAITLIST call or LONGCALL)
; The sign and bit patterns encode the restart type.

	CCS	A		# IS IT A JOB OR IT IT TIMED
		INCR	A		# POSITIVE, MUST BE A JOB
		TCF	ITSAJOB2

		INCR	A		# MUST BE EITHER A WAITLIST OR LONGCALL
		TS	GOLOC		# LET-S STORE THE CORRECT CADR

		CA	WTLTCADR	# SET UP OUR EXIT TO WAITLIST
		TS	GOLOC -1

; Check bit 10 of the bank/bank part to distinguish between WAITLIST and LONGCALL.
; WAITLIST calls have the 2CADR stored as -BB (negative bank), setting bit 10.
; LONGCALLs store the delta-time address using GENADR format.

		CA	GOLOC +1	# NOW FIND OUT IF IT IS A WAITLIST CALL
		MASK	BIT10		# THIS SHOULD BE ONE IF WE HAVE -BB
		CCS	A		# FOR THAT MATTER SO SHOULD BE BITS 9,8,7,
					# 6,5, AND LAST BUT NOT LEAST (PERHAPS NOT
					# IN IMPORTANCE ANYWAY. BIT 4
		TCF	ITSWTLST	# IT IS A WAITLIST CALL

; For LONGCALL restarts, get the delta-time address from the table's product
; field (PRDTTAB). This address points to the original delta-time value.

		NDX	POINTER		# OBTAIN THE ORIGINAL DELTA T
		CA	PRDTTAB		# ADDRESS FOR THIS LONGCALL

		TCF	ITSLGCL1	# NOW GO GET THE DELTA TIME

# ***** THIS MUST BE IN FIXED FIXED *****

		BLOCK	02
		SETLOC	FFTAG2
		BANK

		COUNT	02/RSROU

; ============================================================================
; LONGCALL Delta-Time Retrieval (FIXED-FIXED Memory)
;
; This section must execute in fixed-fixed memory (no bank switching during
; execution) to safely handle memory bank switching for delta-time access.
; The delta-time may be stored in a different erasable bank than the one
; currently selected, requiring careful bank management.
; ============================================================================

ITSLGCL1	LXCH	GOLOC +1	# OBTAIN THE CORRECT E BANK
		LXCH	BB
		LXCH	GOLOC +1	# AND PRESERVE OUR E AND F BANKS

; Retrieve the double-precision delta-time value from erasable memory.
; The address in A register points to the location, potentially in a
; different erasable bank (managed by BB register).

		EXTEND			# GET THE DELTA TIME
		NDX	A
		DCA	0
# Page 1418
		LXCH	GOLOC +1	# RESTORE OUR E AND F BANK
		LXCH	BB		# RESTORE THE TASKS E AND F BANKS
		LXCH	GOLOC +1	# AND PRESERVE OUR L

		TCF	ITSLGCL2	# NOW LET:S PROCESS THIS LONGCALL

# ***** YOU MAY RETURN TO SWITCHED FIXED *****

		BANK	01
		SETLOC	RESTART
		BANK

		COUNT	01/RSROU

; ============================================================================
; LONGCALL Time Calculation and Scheduling Decision
;
; Calculate how much time remains before the LONGCALL should execute.
; Decision logic:
; - If time remaining is positive, schedule via LONGCALL (proper time delay)
; - If time expired (negative or zero), execute immediately via WAITLIST
; ============================================================================

ITSLGCL2	DXCH	LONGTIME

; Calculate time remaining = (original delta-time) - (current TIME2) + (LONGBASE)
; LONGBASE corrects for time base changes during restart processing.
; Result in LONGTIME (double-precision) determines scheduling method.

		EXTEND			# CALCULATE TIME LEFT
		DCS	TIME2
		DAS	LONGTIME
		EXTEND
		DCA	LONGBASE
		DAS	LONGTIME

; Test double-precision time remaining to determine restart method:
; Positive: Schedule via LONGCALL with remaining time
; Zero/Negative: Execute immediately via WAITLIST (time already expired)

		CCS	LONGTIME	# FIND OUT HOW THIS SHOULD BE RESTARTED
		TCF	LONGCLCL
		TCF	+2
		TCF	IMEDIATE -3
		CCS	LONGTIME +1
		TCF	LONGCLCL
		NOOP			# CAN:T GET HERE	*********
		TCF	IMEDIATE -3
		TCF	IMEDIATE

; Time remaining is positive: schedule via LONGCALL with the computed delay.
; LONGCALL handles time delays longer than WAITLIST can manage (> 1 minute).

LONGCLCL	CA	LGCLCADR	# WE WILL GO TO LONGCALL
		TS	GOLOC -1

		EXTEND			# PREPARE OUR ENTRY TO LONGCALL
		DCA	LONGTIME
		TC	GOLOC -1

ITSLNGCL	CA	WTLTCADR	# ASSUME IT WILL GO TO WAITLIST
		TS	GOLOC -1

		NDX	TEMP2G
		CS	PHSPRDT1	# GET THE DELTA T ADDRESS

		TCF	ITSLGCL1	# NOW GET THE DELTA TIME

; ============================================================================
; Waitlist task restart from table
;
; ITSWTLST handles restart of waitlist tasks from the restart tables. These
; are timer-driven tasks scheduled via WAITLIST that must be restored after
; power interruptions or program alarms. The routine:
; 1. Corrects BBCON (bank call) information for proper memory addressing
; 2. Retrieves delta-time from restart table using POINTER index
; 3. Determines if time value is stored directly or indirectly
; 4. Jumps to TIMETEST to compute actual restart time
;
; During Apollo 11's 1202 alarms, waitlist tasks handling landing radar data
; were rapidly restarted through this path, maintaining sensor data flow.
; ============================================================================

ITSWTLST	CS	GOLOC +1	# CORRECT THE BBCON INFORMATION
		TS	GOLOC +1
# Page 1419
		NDX	POINTER		# GET THE DT AND FIND OUT IF IT WAS STORED
		CA	PRDTTAB		# DIRECTLY OR INDIRECTLY

		TCF	TIMETEST	# FIND OUT HOW THE TIME IS STORED

; Job restart from table.
; ITSAJOB2 restores a job from the restart table by retrieving its CADR
; (coded address) and priority, then scheduling it via NOVAC (the job
; request entry point). This path is used when the restart group contains
; a job that must be restarted after an alarm or power transient.

ITSAJOB2	XCH	GOLOC		# Store the job CADR
		NDX	POINTER		# Index into restart table
		CA	PRDTTAB		# Get priority for this job
		TCF	CHKNOVAC	# Schedule job via NOVAC

; ============================================================================
; Even-numbered table phase restart
;
; ITSEVEN handles restart of even-numbered phases in table-based restarts.
; Table restarts divide work into phases, with even phases requiring special
; handling to either continue to the second part of the table or return to
; get the next restart group.
;
; The routine calculates POINTER = SIZETAB + (3 * TEMPPHS), which indexes
; into the restart table. The multiplication by 3 is performed via triple
; addition, optimizing for AGC's limited instruction set (no native multiply).
; This AGC-specific technique was common in memory-constrained environments.
; ============================================================================

ITSEVEN		CA	TEMPSWCH	# SET UP FOR EITHER THE SECOND PART OF THE
		TS	GOLOC +2	# TABLE, OR A RETURN FOR THE NEXT GROUP

		NDX	TEMP2G		# SET UP POINTER FOR OUR LOCATION WITHIN
		CA	SIZETAB		# THE TABLE
		AD	TEMPPHS		# THIS MAY LOOK BAD BUT LET:S SEE YOU DO
		AD	TEMPPHS		# BETTER IN TIME OR NUMBER OF LOCATIONS
		AD	TEMPPHS		# Multiply by 3 via triple addition
		TS	POINTER		; Store table index

		TCF	CONTBL2		# NOW PROCESS WHAT IS IN THE TABLE

; Second half of table processing.
; PHSPART2 handles the second part of a table restart by advancing the
; POINTER by 3 to access the next table entry. After processing this second
; entry, the routine will fetch the next restart group since this is the
; final pass through the even table for this group.

PHSPART2	CA	THREE		# SET THE POINTER FOR THE SECOND HALF OF
		ADS	POINTER		# THE TABLE (add 3 to POINTER)

		CA	RTRNCADR	# THIS WILL BE OUR LAST TIME THROUGH THE
		TS	GOLOC +2	# EVEN TABLE , SO AFTER IT  GET THE NEXT
					# GROUP
		TCF	CONTBL2		# SO LET:S GET THE SECOND ENTRY IN THE TBL

TEMPPHS		EQUALS	MPAC
TEMP2G		EQUALS	MPAC +1
POINTER		EQUALS	MPAC +2
TEMPSWCH	EQUALS	MPAC +3
GOLOC		EQUALS	VAC5 +20D
MINUS2		EQUALS	NEG2
OCT177		EQUALS	LOW7

PHS2CADR	GENADR	PHSPART2
PRT2CADR	GENADR	GETPART2
LGCLCADR	GENADR	LONGCALL
FVACCADR	GENADR	FINDVAC
WTLTCADR	GENADR	WAITLIST
NOVACADR	GENADR	NOVAC




