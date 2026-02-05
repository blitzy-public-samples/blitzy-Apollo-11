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

# Page 1414
# ============================================================================
# RESTARTS - RESTART DISPATCHER ROUTINE
# ============================================================================
# [MODERN EQUIVALENT: State Machine Dispatcher for Fault Recovery]
#
# PURPOSE: This routine is the central dispatcher for processing restart
#          phases. It is called from GOPROG3 in FRESH_START_AND_RESTART
#          via RACTCADR for each active restart group.
#
# ENTRY CONDITIONS:
#   - MPAC+5 contains group number - 1, doubled for indexing
#   - TEMPPHS contains the phase value for this group
#   - Phase registers PHASEn and PHSNAMEn contain restart state
#
# DISPATCH LOGIC (Type discrimination via MASK OCT1400):
#   - OCT1400 bits set: Variable restart (ITSAVAR) - Type A, B, or C
#   - OCT1400 bits clear: Table restart (ITSATBL) or X.1 display restart
#
# TERMINOLOGY TRANSLATION:
#   RESTARTS = State Machine Dispatcher
#   ITSAVAR = Variable Phase Handler (Type A/B/C)
#   ITSATBL = Table Phase Handler
#   ITSAJOB = Job Restart Handler
#   ITSAWAIT = Waitlist Task Handler
#   FINDTIME = Deadline Calculator
# ============================================================================
		BANK	01
		SETLOC	RESTART
		BANK

		EBANK=	PHSNAME1	# GOPROG MUST SWITCH TO THIS EBANK

		COUNT	01/RSROU

# --------------------------------------------------------------------------
# RESTARTS ENTRY - Group Number and Exit Path Setup
# [MODERN: Initialize dispatcher with current group and default return paths]
# --------------------------------------------------------------------------
RESTARTS	CA	MPAC +5		# GET GROUP NUMBER -1
		DOUBLE			# SAVE FOR INDEXING
		TS	TEMP2G
				# [MODERN: TEMP2G = (group-1)*2 for indexed access
				#  to PHASEn, PHSNAMEn, TBASEn registers]

		CA	PHS2CADR	# SET UP EXIT IN CASE IT IS AN EVEN
		TS	TEMPSWCH	# TABLE PHASE
				# [MODERN: TEMPSWCH points to PHSPART2 for double
				#  table entries (even phases have two restarts)]

		CA	RTRNCADR	# TO SAVE TIME ASSUME IT WILL GET NEXT
		TS	GOLOC +2	# GROUP AFTER THIS
				# [MODERN: Default exit returns to GOPROG3 to
				#  process the next restart group in sequence]

# --------------------------------------------------------------------------
# TYPE DISCRIMINATION - Variable vs Table Restart
# [MODERN: OCT1400 = bits 10,11 - if set, variable restart (Type A/B/C)]
# --------------------------------------------------------------------------
		CA	TEMPPHS
		MASK	OCT1400
		CCS	A		# IS IT A VARIABLE OR TABLE RESTART
		TCF	ITSAVAR		# IT:S A VARIABLE RESTART
				# [MODERN: If bits 10 or 11 set in phase, it's a
				#  variable restart with 2CADR in PHSNAMEn]

# --------------------------------------------------------------------------
# X.1 DISPLAY RESTART - Special Case for Phase = X.1
# [MODERN: X.1 phase = display restart. Phase ending in .1 triggers INITDSP
#  job at PRIO14 to reinitialize DSKY displays after restart. This handles
#  display state recovery without restarting the underlying program logic.]
# --------------------------------------------------------------------------
GETPART2	CCS	TEMPPHS		# IS IT AN X.1 RESTART
		CCS	A
		TCF	ITSATBL		# NO, ITS A TABLE RESTART

		CA	PRIO14		# IT IS AN X.1 RESTART, THEREFORE START
		TC	FINDVAC		# THE DISPLAY RESTART JOB
		EBANK=	LST1
		2CADR	INITDSP
				# [MODERN: Schedule INITDSP job to restore DSKY
				#  display state - priority 14 (moderate)]

		TC	RTRNCADR	# FINISHED WITH THIS GROUP, GET NEXT ONE

# ============================================================================
# ITSAVAR - VARIABLE PHASE RESTART HANDLER
# ============================================================================
# [MODERN: Handles Type A (fixed), Type B (combined), and Type C (variable)
#  phase restarts where restart information is stored in phase registers
#  rather than compiled into restart tables. Variable restarts store the
#  job/task 2CADR in PHSNAMEn register at runtime via PHASCHNG calls.]
# ============================================================================
ITSAVAR		MASK	OCT1400		# IS IT TYPE B ?
		CCS	A
		TCF	ITSLIKEB	# YES,IT IS TYPE B
				# [MODERN: Type B has BOTH bits 10,11 set -
				#  combines variable job + fixed table entry]

		EXTEND			# STORE THE JOB (OR TASK) 2CADR FOR EXIT
		NDX	TEMP2G
		DCA	PHSNAME1
		DXCH	GOLOC
				# [MODERN: Fetch stored 2CADR from PHSNAMEn
				#  for this group. This is the restart address
				#  that was saved when PHASCHNG was called.]

# --------------------------------------------------------------------------
# JOB/TASK/LONGCALL DISCRIMINATION
# [MODERN: Low 3 bits (OCT7) of phase determine restart type:
#   Bits 0-2 value + MINUS2 then CCS:
#   Result > 0: ITSLNGCL (longcall - time > 163.84 sec)
#   Result = 0: TC SWRETURN/TCF ITSAWAIT (waitlist task)
#   Result < 0: TCF ITSAJOB (job via FINDVAC or NOVAC)]
# --------------------------------------------------------------------------
		CA	TEMPPHS		# SEE IF THIS IS A JOB, TASK, OR A LONGCALL
		MASK	OCT7
		AD	MINUS2
		CCS	A
		TCF	ITSLNGCL	# ITS A LONGCALL

# Page 1415
RTRNCADR	TC	SWRETURN	# CANT GET HERE
		TCF	ITSAWAIT
				# [MODERN: CCS result = 0 falls through here
				#  to process as waitlist task]

		TCF	ITSAJOB		# ITS A JOB
				# [MODERN: CCS result < 0 means it's a job
				#  to be scheduled via FINDVAC or NOVAC]

# ============================================================================
# ITSAWAIT - WAITLIST TASK RESTART HANDLER
# ============================================================================
# [MODERN: Time-Based Task Restart Handler. Schedules tasks that need to
#  execute at a specific time relative to the restart. Uses WAITLIST
#  primitive for short delays (< 163.84 seconds).]
# ============================================================================
ITSAWAIT	CA	WTLTCADR	# SET UP WAITLIST CALL
		TS	GOLOC -1
				# [MODERN: Prepare exit through WAITLIST routine
				#  to schedule the time-delayed task]

		NDX	TEMP2G		# DIRECTLY STORED
		CA	PHSPRDT1
				# [MODERN: Fetch delta time from PHSPRDT1 table
				#  entry for this group's restart]
# --------------------------------------------------------------------------
# TIMETEST - Delta Time Storage Discrimination
# [MODERN: TIMETEST determines how the delta time is stored:
#   Positive non-zero: Time stored directly - calculate via FINDTIME
#   Zero (CCS +0): Time stored indirectly via -GENADR (TCF ITSINDIR)
#   Negative (-1/OCT 77777): Immediate restart requested (TCF IMEDIATE)]
# --------------------------------------------------------------------------
TIMETEST	CCS	A		# IS IT AN IMMEDIATE RESTART
		INCR	A		# NO.
		TCF	FINDTIME	# FIND OUT WHEN IT SHOULD BEGIN
				# [MODERN: Calculate remaining time until task
				#  should execute based on saved time base]

		TCF	ITSINDIR	# STORED INDIRECTLY
				# [MODERN: Time stored at address pointed to by
				#  -GENADR, requires E-bank switch to access]

		TCF	IMEDIATE	# IT WANTS AN IMMEDIATE RESTART
				# [MODERN: OCT 77777 (-0) in PRDTTAB signals
				#  "restart immediately" - don't calculate time]

# ***** THIS MUST BE IN FIXED FIXED *****

		BLOCK	02
		SETLOC	FFTAG2
		BANK

		COUNT	02/RSROU

# --------------------------------------------------------------------------
# ITSINDIR - INDIRECT TIME FETCH
# [MODERN: When time is stored indirectly (-GENADR in PRDTTAB), this routine
#  temporarily switches E-banks to access the time value at the pointed-to
#  location. This allows time storage in any E-bank, not just the restart
#  table's E-bank.]
# --------------------------------------------------------------------------
ITSINDIR	LXCH	GOLOC +1	# GET THE CORRECT E BANK IN CASE THIS IS
		LXCH	BB		# SWITCHED ERRASIBLE
				# [MODERN: Swap in BBCON from 2CADR to access
				#  the correct E-bank for the time value]

		NDX	A		# GET THE TIME INDIRECTLY
		CA	1
				# [MODERN: A contains -GENADR, so NDX A + CA 1
				#  fetches the word at (address pointed by A)+1]

		LXCH	BB		# RESTORE THE BB AND GOLOC
		LXCH	GOLOC +1
				# [MODERN: Restore original E/F bank settings
				#  after fetching the indirect time value]

		TCF	FINDTIME	# FIND OUT WHEN IT SHOULD BEGIN

# ***** YOU MAY RETURN TO SWITCHED FIXED *****

		BANK 	01
		SETLOC	RESTART
		BANK

		COUNT	01/RSROU

# ============================================================================
# FINDTIME - DELTA TIME CALCULATION
# ============================================================================
# [MODERN: Relative Deadline Calculator for Task Scheduling]
#
# PURPOSE: Calculate remaining time until task should execute.
#
# ALGORITHM:
#   Delta time = (TBASEn + original_delta) - TIME1
#   If result positive: Use as waitlist delay
#   If negative or zero: Immediate restart (task is overdue)
#
# TIME1 is the current 14-bit cycle counter (centiseconds, wraps at 16384cs)
# TBASEn is the time base value saved for this restart group when task started
# Original_delta is the delay that was originally requested
#
# The arithmetic handles the wraparound of the 14-bit timer by using
# complement and sign checking operations.
# ============================================================================
FINDTIME	COM			# MAKE NEGATIVE SINCE IT WILL BE SUBTRACTED
		TS	L		# AND SAVE
				# [MODERN: L = -(original delta time)]
		NDX	TEMP2G
		CS	TBASE1
				# [MODERN: A = -(saved time base for this group)]
		EXTEND
# Page 1416
		SU	TIME1
				# [MODERN: A = -TBASE - TIME1 = -(TBASE + TIME1)]
		CCS	A
		COM
		AD	OCT37776
		AD	ONE
		AD	L
				# [MODERN: Final result = time remaining until
				#  task should execute, accounting for wraparound]
		CCS	A
		CA	ZERO
		TCF	+2
		TCF	+1
				# [MODERN: If time has already passed (result <= 0)
				#  then restart immediately with minimal delay]
IMEDIATE	AD	ONE
		TC	GOLOC -1
				# [MODERN: Jump to WAITLIST (or LONGCALL) with
				#  calculated delay time in A register]
# ============================================================================
# ITSLIKEB - TYPE B PHASE HANDLER
# ============================================================================
# [MODERN: Combined variable job + fixed table entry. Type B restarts
#  process a variable job first (stored in PHSNAMEn), then return to process
#  the table portion of this phase. This allows a single phase to restart
#  both a dynamically-specified job AND a statically-defined table entry.]
# ============================================================================
ITSLIKEB	CA	RTRNCADR	# TYPE B, SO STORE RETURN IN
		TS	TEMPSWCH	# TEMPSWCH IN CASE OF AN EVEN PHASE
				# [MODERN: Save normal return for after table part]

		CA	PRT2CADR	# SET UP EXIT TO GET TABLE PART OF THIS
		TS	GOLOC +2	# VARIABLE TYPE OF PHASE
				# [MODERN: After job starts, return to GETPART2
				#  to process the table portion of Type B]

		CA	TEMPPHS		# MAKE THE PHASE LOOK RIGHT FOR THE TABLE
		MASK	OCT177		# PART OF THIS VARIABLE PHASE
		TS	TEMPPHS
				# [MODERN: Clear upper bits so phase looks like
				#  a regular table phase for subsequent lookup]

		EXTEND
		NDX	TEMP2G		# OBTAIN THE JOB:S 2CADR
		DCA	PHSNAME1
		DXCH	GOLOC
				# [MODERN: Fetch the variable job's 2CADR from
				#  PHSNAMEn register - this is the job to start]

# ============================================================================
# ITSAJOB - JOB RESTART HANDLER
# ============================================================================
# [MODERN: Job Scheduling Dispatch. Determines whether to use FINDVAC
#  (job with VAC interpreter work area) or NOVAC (job without VAC area)
#  based on the sign of the priority value in PHSPRDT1.
#
#  Positive priority = FINDVAC (job needs VAC area for interpretive code)
#  Negative priority = NOVAC (job runs in basic only, no VAC needed)]
# ============================================================================
ITSAJOB		NDX	TEMP2G		# NOW ADD THE PRIORITY AND LET:S GO
		CA	PHSPRDT1
				# [MODERN: Fetch priority from PHSPRDT1 table
				#  for this restart group]
CHKNOVAC	TS	GOLOC -1	# SAVE PRIO UNTIL WE SEE IF ITS
		EXTEND			# A FINDVAC OR A NOVAC
		BZMF	ITSNOVAC
				# [MODERN: Check sign - if zero or negative,
				#  use NOVAC; if positive, use FINDVAC]

		CAF	FVACCADR	# POSITIVE, SET UP FINDVAC CALL.
		XCH	GOLOC -1	# PICK UP PRIO,
		TC	GOLOC -1	# AND GO
				# [MODERN: FINDVAC path - allocate VAC area
				#  and schedule job at specified priority]

ITSNOVAC	CAF	NOVACADR	# NEGATIVE,
		XCH	GOLOC -1	# SET UP NOVAC CALL,
		COM			# CORRECT PRIO,
		TC	GOLOC -1	# AND GO
				# [MODERN: NOVAC path - schedule job without
				#  VAC area, negate priority to get true value]

# ============================================================================
# ITSATBL - TABLE PHASE RESTART HANDLER
# ============================================================================
# [MODERN: Table-Driven Restart Handler]
#
# This handles restarts where the restart information is compiled into
# the RESTART_TABLES.agc file rather than stored at runtime.
#
# EVEN/ODD PHASE DISCRIMINATION (via CCS CYR - Cycle Right register):
#   Odd phase (X.3, X.5, etc.): Single table entry at X.YSPOT (3 words)
#   Even phase (X.2, X.4, etc.): Double table entry (6 words, two restarts)
#
# TABLE STRUCTURE:
#   SIZETAB: Index table for calculating restart entry positions
#   CADRTAB: Contains 2CADR restart addresses
#   PRDTTAB: Contains priority (jobs) or delta-time (tasks)
#
# Source: Comanche055/RESTART_TABLES.agc:97-99
# ============================================================================
ITSATBL		TS	CYR		# FIND OUT IF THE PHASE IS ODD OR EVEN
		CCS	CYR
		TCF	+1		# IT:S EVEN
		TCF	ITSEVEN
				# [MODERN: CYR (Cycle Right) shifts A right,
				#  moving bit 0 to bit 14. CCS then tests if
				#  original bit 1 was set (even vs odd phase)]

# --------------------------------------------------------------------------
# ODD TABLE PHASE POINTER CALCULATION
# [MODERN: For odd phases (single entry), calculate index into restart tables
#  using formula: POINTER = phase + (phase/2) + SIZETAB[group+1]
#  This accounts for 3-word entries in the table structure.]
# --------------------------------------------------------------------------
		CA	RTRNCADR	# IN CASE THIS IS THE SECOND PART OF A
		TS	GOLOC +2	# TYPE B RESTART, WE NEED PROPER EXIT
# Page 1417
		CA	TEMPPHS		# SET UP POINTER FOR FINDING OUR PLACE IN
		TS	SR		# THE RESTART TABLES
		AD	SR
		NDX	TEMP2G
		AD	SIZETAB +1
		TS	POINTER
				# [MODERN: POINTER = TEMPPHS + TEMPPHS/2 +
				#  SIZETAB[group]. Each odd entry is 3 words,
				#  so multiply by 1.5 via SR (shift right)]

# --------------------------------------------------------------------------
# CONTBL2 - TABLE ENTRY PROCESSING
# --------------------------------------------------------------------------
# [MODERN: Process a restart table entry. The table contains 3 words:
#   Word 0 (PRDTTAB): Priority (if job) or delta-time (if task)
#   Word 1 (CADRTAB): First word of 2CADR (CADR portion)
#   Word 2 (CADRTAB+1): Second word of 2CADR (BB portion)
#
# The sign of the CADR determines the restart type:
#   Positive CADR: Job restart (via FINDVAC or NOVAC)
#   Negative CADR: Timed restart (waitlist or longcall)]
# --------------------------------------------------------------------------
CONTBL2		EXTEND			# FIND OUT WHAT'S IN THE TABLE
		NDX	POINTER
		DCA	CADRTAB		# GET THE 2CADR
				# [MODERN: Fetch 2CADR from table - A=CADR, L=BB]

		LXCH	GOLOC +1	# STORE THE BB INFORMATION
				# [MODERN: Save bank bits for later dispatch]

		CCS	A		# IS IT A JOB OR IT IT TIMED
		INCR	A		# POSITIVE, MUST BE A JOB
		TCF	ITSAJOB2
				# [MODERN: Positive CADR = job restart]

		INCR	A		# MUST BE EITHER A WAITLIST OR LONGCALL
		TS	GOLOC		# LET-S STORE THE CORRECT CADR
				# [MODERN: Negative CADR = timed task, negate
				#  to get actual CADR and store for dispatch]

# --------------------------------------------------------------------------
# WAITLIST/LONGCALL DISCRIMINATION
# [MODERN: BIT10 of BBCON distinguishes waitlist from longcall:
#   BIT10 set (negative BBCON stored as -BBCON): Waitlist task
#   BIT10 clear (positive BBCON stored as GENADR): Longcall task]
# --------------------------------------------------------------------------
		CA	WTLTCADR	# SET UP OUR EXIT TO WAITLIST
		TS	GOLOC -1
				# [MODERN: Assume waitlist, will change if longcall]

		CA	GOLOC +1	# NOW FIND OUT IF IT IS A WAITLIST CALL
		MASK	BIT10		# THIS SHOULD BE ONE IF WE HAVE -BB
		CCS	A		# FOR THAT MATTER SO SHOULD BE BITS 9,8,7,
					# 6,5, AND LAST BUT NOT LEAST (PERHAPS NOT
					# IN IMPORTANCE ANYWAY. BIT 4
		TCF	ITSWTLST	# IT IS A WAITLIST CALL
				# [MODERN: BIT10 set = -BBCON format = waitlist]

		NDX	POINTER		# OBTAIN THE ORIGINAL DELTA T
		CA	PRDTTAB		# ADDRESS FOR THIS LONGCALL
				# [MODERN: PRDTTAB contains GENADR of the
				#  double-precision delta time for longcall]

		TCF	ITSLGCL1	# NOW GO GET THE DELTA TIME

# ***** THIS MUST BE IN FIXED FIXED *****

		BLOCK	02
		SETLOC	FFTAG2
		BANK

		COUNT	02/RSROU

# --------------------------------------------------------------------------
# ITSLGCL1 - LONGCALL DELTA TIME FETCH
# [MODERN: Fetch double-precision delta time from switched erasable memory.
#  Temporarily switches to the E-bank containing the time value.]
# --------------------------------------------------------------------------
ITSLGCL1	LXCH	GOLOC +1	# OBTAIN THE CORRECT E BANK
		LXCH	BB
		LXCH	GOLOC +1	# AND PRESERVE OUR E AND F BANKS
				# [MODERN: Swap in task's BBCON to access its
				#  E-bank where the delta time is stored]

		EXTEND			# GET THE DELTA TIME
		NDX	A
		DCA	0
				# [MODERN: A contains GENADR, so NDX A + DCA 0
				#  fetches the DP time at the pointed address]
# Page 1418
		LXCH	GOLOC +1	# RESTORE OUR E AND F BANK
		LXCH	BB		# RESTORE THE TASKS E AND F BANKS
		LXCH	GOLOC +1	# AND PRESERVE OUR L
				# [MODERN: Restore original bank settings while
				#  preserving the fetched time in A,L]

		TCF	ITSLGCL2	# NOW LET:S PROCESS THIS LONGCALL

# ***** YOU MAY RETURN TO SWITCHED FIXED *****

		BANK	01
		SETLOC	RESTART
		BANK

		COUNT	01/RSROU

# ============================================================================
# ITSLGCL2 - LONGCALL TIME CALCULATION
# ============================================================================
# [MODERN: Calculate remaining time for longcall restart.
#
# ALGORITHM:
#   LONGTIME = original_delta_time - (TIME2 - LONGBASE)
#   Where:
#     original_delta_time: The delay originally requested
#     TIME2: Current double-precision time counter
#     LONGBASE: Time base value when longcall was originally started
#
#   If result > 0: Schedule via LONGCALL with remaining time
#   If result <= 0: Task is overdue, restart immediately via WAITLIST]
# ============================================================================
ITSLGCL2	DXCH	LONGTIME
				# [MODERN: Store fetched delta time in LONGTIME]

		EXTEND			# CALCULATE TIME LEFT
		DCS	TIME2
		DAS	LONGTIME
		EXTEND
		DCA	LONGBASE
		DAS	LONGTIME
				# [MODERN: LONGTIME = delta - TIME2 + LONGBASE
				#  = delta - (TIME2 - LONGBASE)
				#  = remaining time until task should execute]

		CCS	LONGTIME	# FIND OUT HOW THIS SHOULD BE RESTARTED
		TCF	LONGCLCL
		TCF	+2
		TCF	IMEDIATE -3
		CCS	LONGTIME +1
		TCF	LONGCLCL
		NOOP			# CAN:T GET HERE	*********
		TCF	IMEDIATE -3
		TCF	IMEDIATE
				# [MODERN: Check if time > 0 (use longcall)
				#  or time <= 0 (use immediate waitlist restart)]

LONGCLCL	CA	LGCLCADR	# WE WILL GO TO LONGCALL
		TS	GOLOC -1
				# [MODERN: Time remaining > 163.84 sec, must
				#  use LONGCALL instead of WAITLIST]

		EXTEND			# PREPARE OUR ENTRY TO LONGCALL
		DCA	LONGTIME
		TC	GOLOC -1
				# [MODERN: Call LONGCALL with DP time in A,L
				#  and 2CADR already stored in GOLOC]

# --------------------------------------------------------------------------
# ITSLNGCL - VARIABLE LONGCALL HANDLER
# [MODERN: Handle longcall from variable restart. PHSPRDT1 contains
#  the -GENADR of the DP delta time location (negated for sign indication).]
# --------------------------------------------------------------------------
ITSLNGCL	CA	WTLTCADR	# ASSUME IT WILL GO TO WAITLIST
		TS	GOLOC -1
				# [MODERN: Start with waitlist exit, will change
				#  to longcall if time > 163.84 seconds]

		NDX	TEMP2G
		CS	PHSPRDT1	# GET THE DELTA T ADDRESS
				# [MODERN: Negate to get positive GENADR of
				#  the DP delta time storage location]

		TCF	ITSLGCL1	# NOW GET THE DELTA TIME

# --------------------------------------------------------------------------
# ITSWTLST - WAITLIST TABLE TASK HANDLER
# [MODERN: Handle waitlist task from table restart. The BBCON was stored
#  negated (-BBCON) in the table, so negate it back to get correct value.]
# --------------------------------------------------------------------------
ITSWTLST	CS	GOLOC +1	# CORRECT THE BBCON INFORMATION
		TS	GOLOC +1
				# [MODERN: Negate -BBCON back to positive BBCON
				#  for proper bank switching on task dispatch]
# Page 1419
		NDX	POINTER		# GET THE DT AND FIND OUT IF IT WAS STORED
		CA	PRDTTAB		# DIRECTLY OR INDIRECTLY
				# [MODERN: Fetch delta time or indirect pointer
				#  from PRDTTAB for this restart entry]

		TCF	TIMETEST	# FIND OUT HOW THE TIME IS STORED
				# [MODERN: Use TIMETEST to discriminate direct
				#  vs indirect time storage and process]

# --------------------------------------------------------------------------
# ITSAJOB2 - TABLE JOB HANDLER
# [MODERN: Handle job restart from table. Store CADR and fetch priority
#  from PRDTTAB, then dispatch via CHKNOVAC for FINDVAC/NOVAC selection.]
# --------------------------------------------------------------------------
ITSAJOB2	XCH	GOLOC		# STORE THE CADR
				# [MODERN: Store job CADR for later dispatch]

		NDX	POINTER		# ADD THE PRIORITY AND LET:S GO
		CA	PRDTTAB
				# [MODERN: Fetch priority from table entry]

		TCF	CHKNOVAC
				# [MODERN: Check priority sign to determine
				#  FINDVAC (positive) vs NOVAC (negative)]

# ============================================================================
# ITSEVEN - EVEN PHASE HANDLER (Double Table Entry)
# ============================================================================
# [MODERN: Even phases have TWO restart entries (6 words total). ITSEVEN
#  calculates the pointer for the first entry and processes it. After the
#  first entry is dispatched, control returns to PHSPART2 (via TEMPSWCH)
#  to process the second entry.
#
#  POINTER = SIZETAB[group] + phase * 3
#  (Each even entry pair takes 6 words = 2 entries * 3 words per entry)]
# ============================================================================
ITSEVEN		CA	TEMPSWCH	# SET UP FOR EITHER THE SECOND PART OF THE
		TS	GOLOC +2	# TABLE, OR A RETURN FOR THE NEXT GROUP
				# [MODERN: TEMPSWCH = PHS2CADR, so after first
				#  entry we return to PHSPART2 for second entry]

		NDX	TEMP2G		# SET UP POINTER FOR OUR LOCATION WITHIN
		CA	SIZETAB		# THE TABLE
		AD	TEMPPHS		# THIS MAY LOOK BAD BUT LET:S SEE YOU DO
		AD	TEMPPHS		# BETTER IN TIME OR NUMBER OF LOCATIONS
		AD	TEMPPHS
		TS	POINTER
				# [MODERN: POINTER = SIZETAB[group] + phase*3
				#  Multiplies by 3 via three ADDs (efficient
				#  given no multiply instruction available)]

		TCF	CONTBL2		# NOW PROCESS WHAT IS IN THE TABLE
				# [MODERN: Process first of two table entries]

# --------------------------------------------------------------------------
# PHSPART2 - SECOND ENTRY OF EVEN PHASE
# [MODERN: Called after first entry of even phase is dispatched. Advances
#  pointer by 3 words to second entry and processes it as final restart
#  for this phase group.]
# --------------------------------------------------------------------------
PHSPART2	CA	THREE		# SET THE POINTER FOR THE SECOND HALF OF
		ADS	POINTER		# THE TABLE
				# [MODERN: Advance pointer by 3 words to the
				#  second entry in this even phase pair]

		CA	RTRNCADR	# THIS WILL BE OUR LAST TIME THROUGH THE
		TS	GOLOC +2	# EVEN TABLE , SO AFTER IT  GET THE NEXT
					# GROUP
				# [MODERN: After second entry, return to GOPROG3
				#  to process next restart group]
		TCF	CONTBL2		# SO LET:S GET THE SECOND ENTRY IN THE TBL

# ============================================================================
# TEMPORARY STORAGE AND ADDRESS CONSTANTS
# ============================================================================
# [MODERN: Register assignments for dispatcher state. Uses MPAC area
#  (which is available during restart since no interpretive code running)
#  and VAC5 area for exit addressing.]
#
# TEMPPHS (MPAC): Current phase value being processed
# TEMP2G (MPAC+1): Group number doubled for indexed access to phase registers
# POINTER (MPAC+2): Index into restart tables (PRDTTAB/CADRTAB)
# TEMPSWCH (MPAC+3): Exit switch - PHSPART2 for even, RTRNCADR for odd phases
# GOLOC (VAC5+20D): Exit 2CADR storage for dispatcher
#   GOLOC-1: Entry routine address (FINDVAC/NOVAC/WAITLIST/LONGCALL)
#   GOLOC: Job/task CADR
#   GOLOC+1: Job/task BBCON (bank bits)
#   GOLOC+2: Return address after job/task starts
# ============================================================================
TEMPPHS		EQUALS	MPAC
TEMP2G		EQUALS	MPAC +1
POINTER		EQUALS	MPAC +2
TEMPSWCH	EQUALS	MPAC +3
GOLOC		EQUALS	VAC5 +20D
MINUS2		EQUALS	NEG2
OCT177		EQUALS	LOW7

# --------------------------------------------------------------------------
# ADDRESS CONSTANTS FOR DISPATCH TARGETS
# [MODERN: GENADR creates a 12-bit address constant for TC/TCF jumps
#  to the various Executive and timing primitives used for restart dispatch.]
# --------------------------------------------------------------------------
PHS2CADR	GENADR	PHSPART2
				# [MODERN: Return address for second even entry]
PRT2CADR	GENADR	GETPART2
				# [MODERN: Return address for Type B table part]
LGCLCADR	GENADR	LONGCALL
				# [MODERN: Entry to LONGCALL for delays > 163.84s]
FVACCADR	GENADR	FINDVAC
				# [MODERN: Entry to FINDVAC - job with VAC area]
WTLTCADR	GENADR	WAITLIST
				# [MODERN: Entry to WAITLIST - time-delayed task]
NOVACADR	GENADR	NOVAC
				# [MODERN: Entry to NOVAC - job without VAC area]




