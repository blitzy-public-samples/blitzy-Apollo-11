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

# Page 1303
		BANK	01
		SETLOC	RESTART
		BANK

		EBANK=	PHSNAME1	# GOPROG MUST SWITCH TO THIS EBANK

		COUNT*	$$/RSROU
# ==========================================================================
# RESTARTS - STATE MACHINE DISPATCHER FOR RESTART RECOVERY
# ==========================================================================
# [Modern: State Machine Dispatcher for Phase-Based Restart Recovery]
#
# PURPOSE: This routine dispatches restart processing for active restart
#          groups. It examines the phase encoding to determine the restart
#          type (Variable Type A/B/C or Table restart) and invokes the
#          appropriate handler.
#
# ENTRY: Via RACTCADR from GOPROG3 in FRESH_START_AND_RESTART.agc after
#        phase table validation has completed.
#        - MPAC+5 contains the group number minus 1 (0-5 for groups 1-6)
#        - TEMPPHS contains the absolute value of the phase (|PHASE|+1)
#
# TYPE DISCRIMINATION LOGIC:
#        The phase encoding uses bits 10-11 (OCT1400 mask) to determine type:
#        - If OCT1400 bits are set: Variable restart (ITSAVAR)
#          -- Further discrimination for Type B vs Type C
#        - If OCT1400 bits clear: Table restart (ITSATBL) or X.1 display
#          -- X.1 phases (phase ends in .1) trigger display restart
#          -- Other phases use the restart tables (CADRTAB/PRDTTAB)
#
# PHASE ENCODING SUMMARY (from PHASE_TABLE_MAINTENANCE.agc):
#        Type A: TL0 00P PPP PPP GGG - Fixed phase, table restart
#        Type B: TL1 DAP PPP PPP GGG - Combined variable job + table entry
#        Type C: TL0 1AD XXX CJW GGG - Variable phase (job/task/longcall)
#
# EXIT: Via SWRETURN at RTRNCADR to process next restart group, or
#       directly to job/task/longcall restart handlers.
#
# SOURCE: Luminary099/RESTARTS_ROUTINE.agc:35
# ==========================================================================
RESTARTS	CA	MPAC +5		# GET GROUP NUMBER -1
		DOUBLE			# DOUBLE FOR DOUBLE-WORD INDEXING
		TS	TEMP2G		# [Modern: Store 2*(group-1) for table lookup]

		CA	PHS2CADR	# SET UP EXIT IN CASE IT IS AN EVEN
		TS	TEMPSWCH	# TABLE PHASE (RETURNS TO PHSPART2)

		CA	RTRNCADR	# TO SAVE TIME ASSUME IT WILL GET NEXT
		TS	GOLOC +2	# GROUP AFTER THIS (DEFAULT EXIT PATH)

		CA	TEMPPHS		# [Modern: Extract restart type from phase]
		MASK	OCT1400		# OCT1400 = BITS 10-11 (TYPE DISCRIMINATOR)
		CCS	A		# IS IT A VARIABLE OR TABLE RESTART
		TCF	ITSAVAR		# BITS SET: VARIABLE RESTART (TYPE B OR C)

# --------------------------------------------------------------------------
# GETPART2 - X.1 DISPLAY RESTART CHECK
# --------------------------------------------------------------------------
# [Modern: Display State Recovery Handler]
#
# An X.1 phase (phase ending in .1) indicates display restart is needed.
# This starts the INITDSP job at PRIO14 to reinitialize displays after
# restart. Used primarily in manned flight modes to restore display state.
# --------------------------------------------------------------------------
GETPART2	CCS	TEMPPHS		# IS IT AN X.1 RESTART
		CCS	A		# DOUBLE CCS: IF A=1 (X.1), SKIP TO +4
		TCF	ITSATBL		# NO, ITS A TABLE RESTART (PHASE > 1)

		CA	PRIO14		# IT IS AN X.1 RESTART, THEREFORE START
		TC	FINDVAC		# THE DISPLAY RESTART JOB
		EBANK=	LST1		# [Modern: Schedule display recovery job]
		2CADR	INITDSP

		TC	RTRNCADR	# FINISHED WITH THIS GROUP, GET NEXT ONE

# --------------------------------------------------------------------------
# ITSAVAR - VARIABLE PHASE RESTART HANDLER (TYPE B OR TYPE C)
# --------------------------------------------------------------------------
# [Modern: Variable Phase Handler - dispatches based on phase encoding]
#
# Handles variable (Type B or C) restarts where job/task/longcall address
# is stored in PHSNAME registers rather than fixed restart tables.
#
# Type discrimination:
#   - If OCT1400 still set after second MASK: Type B (combined job+table)
#   - Otherwise: Type C (pure variable - job, waitlist, or longcall)
#
# For Type C, bits 0-2 (OCT7 mask) encode the restart category:
#   - Bit 0 (W): Waitlist task
#   - Bit 1 (J): Job
#   - Bit 2 (C): Longcall
# The AD MINUS2 / CCS A pattern discriminates:
#   - Result > 0: ITSLNGCL (longcall, bit 2 set: C=1, value 4-2=2 > 0)
#   - Result = 0: ITSAWAIT (waitlist, bit 0 set: W=1, value 1-2=-1, +0)
#   - Result < 0: ITSAJOB  (job, bit 1 set: J=1, value 2-2=0, then -1)
# --------------------------------------------------------------------------
ITSAVAR		MASK	OCT1400		# IS IT TYPE B ?
		CCS	A		# CHECK IF OCT1400 BITS STILL SET
		TCF	ITSLIKEB	# YES, TYPE B (COMBINED JOB + TABLE)

		EXTEND			# TYPE C: STORE THE JOB/TASK 2CADR FOR EXIT
		NDX	TEMP2G		# INDEX BY 2*(GROUP-1)
		DCA	PHSNAME1	# GET 2CADR FROM VARIABLE STORAGE
		DXCH	GOLOC		# [Modern: Load restart address into GOLOC]

		CA	TEMPPHS		# SEE IF THIS IS A JOB, TASK, OR A LONGCAL
		MASK	OCT7		# EXTRACT CJW BITS (BITS 0-2)
		AD	MINUS2		# SUBTRACT 2 FOR CCS DISCRIMINATION
		CCS	A		# [Modern: Three-way branch based on type]
		TCF	ITSLNGCL	# RESULT > 0: ITS A LONGCALL (C BIT SET)

RTRNCADR	TC	SWRETURN	# CANT GET HERE (CCS SKIP LOCATION)
# Page 1304
		TCF	ITSAWAIT	# RESULT = 0: WAITLIST TASK (W BIT SET)

		TCF	ITSAJOB		# RESULT < 0: ITS A JOB (J BIT SET)

# --------------------------------------------------------------------------
# ITSAWAIT - WAITLIST TASK RESTART HANDLER
# --------------------------------------------------------------------------
# [Modern: Time-Based Task Restart Handler]
#
# Sets up a WAITLIST call to restart a time-delayed task. The delta time
# may be stored directly, indirectly, or may request immediate restart.
#
# TIMETEST discriminates three cases via CCS on PHSPRDT1 value:
#   - Positive non-zero: Direct delta time, calculate remaining via FINDTIME
#   - Zero (+0): ITSINDIR - time stored indirectly via -GENADR
#   - Negative (-1 or -0): IMEDIATE - immediate restart (OCT 77777 pattern)
# --------------------------------------------------------------------------
ITSAWAIT	CA	WTLTCADR	# SET UP WAITLIST CALL
		TS	GOLOC -1	# [Modern: Store WAITLIST entry address]

		NDX	TEMP2G		# INDEX BY 2*(GROUP-1)
		CA	PHSPRDT1	# GET PRIORITY/DELTA-TIME FROM STORAGE
TIMETEST	CCS	A		# IS IT AN IMMEDIATE RESTART
		INCR	A		# POSITIVE: RESTORE ORIGINAL VALUE
		TCF	FINDTIME	# FIND OUT WHEN IT SHOULD BEGIN

		TCF	ITSINDIR	# ZERO: TIME STORED INDIRECTLY (-GENADR)

		TCF	IMEDIATE	# NEGATIVE: IMMEDIATE RESTART (OCT 77777)

# ***** THIS MUST BE IN FIXED FIXED *****

		BLOCK	02
		SETLOC	FFTAG2
		BANK

		COUNT*	$$/RSROU
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

		COUNT*	$$/RSROU
# --------------------------------------------------------------------------
# FINDTIME - DELTA TIME CALCULATION FOR WAITLIST RESTART
# --------------------------------------------------------------------------
# [Modern: Relative Deadline Calculator for Task Scheduling]
#
# Calculates the remaining time until a waitlist task should fire:
#   remaining_time = (TBASE + original_delta) - TIME1
#
# Where:
#   - TBASE1-6: Time base values for each restart group (set at PHASCHNG)
#   - TIME1: Current 14-bit cycle counter (incrementing)
#   - original_delta: The delta time task was originally scheduled with
#
# Algorithm:
#   1. Complement delta time (for subtraction)
#   2. Compute -(TBASE + (-delta)) - TIME1 = TIME1 - TBASE - delta
#   3. Handle 14-bit overflow via OCT37776 adjustment
#   4. If result positive: task overdue, restart immediately
#   5. If result negative/zero: use as WAITLIST delay
#
# The OCT37776 value handles the 14-bit counter wraparound case.
# --------------------------------------------------------------------------
FINDTIME	COM			# MAKE NEGITIVE SINCE IT WILL BE SUBTRACTD
		TS	L		# SAVE -DELTA IN L
		NDX	TEMP2G		# INDEX BY 2*(GROUP-1)
		CS	TBASE1		# GET -TBASE FOR THIS GROUP
		EXTEND
		SU	TIME1		# A = -TBASE - TIME1
		CCS	A		# [Modern: Handle counter wraparound]
		COM			# POSITIVE: COMPLEMENT BACK
# Page 1305
		AD	OCT37776	# ADJUST FOR 14-BIT OVERFLOW
		AD	ONE		# +1 FOR ONES COMPLEMENT ARITHMETIC
		AD	L		# ADD -DELTA: RESULT = TIME1-TBASE-DELTA
		CCS	A		# [Modern: Check if task is overdue]
		CA	ZERO		# POSITIVE: OVERDUE, USE ZERO DELAY
		TCF	+2		# SKIP TO IMEDIATE
		TCF	+1		# NEGATIVE: USE REMAINING TIME
IMEDIATE	AD	ONE		# ENSURE MINIMUM 10MS DELAY (1 COUNT)
		TC	GOLOC -1	# [Modern: Jump to WAITLIST via GOLOC-1]
# --------------------------------------------------------------------------
# ITSLIKEB - TYPE B PHASE RESTART HANDLER
# --------------------------------------------------------------------------
# [Modern: Combined Variable Job + Fixed Table Entry Handler]
#
# Type B phases combine a variable job restart with a fixed table entry.
# This routine processes the variable (job) portion first, then sets up
# to process the table portion via GETPART2.
#
# The phase encoding TL1 DAP PPP PPP GGG contains both:
#   - Variable job address in PHSNAME registers
#   - Fixed phase (PPP) for table lookup after job is scheduled
# --------------------------------------------------------------------------
ITSLIKEB	CA	RTRNCADR	# TYPE B, SO STORE RETURN IN
		TS	TEMPSWCH	# TEMPSWCH IN CASE OF AN EVEN PHASE

		CA	PRT2CADR	# SET UP EXIT TO GET TABLE PART OF THIS
		TS	GOLOC +2	# VARIABLE TYPE OF PHASE (TO GETPART2)

		CA	TEMPPHS		# MAKE THE PHASE LOOK RIGHT FOR THE TABLE
		MASK	OCT177		# EXTRACT PPP BITS (PHASE 0-127)
		TS	TEMPPHS		# [Modern: Strip type bits, keep phase]

		EXTEND
		NDX	TEMP2G		# INDEX BY 2*(GROUP-1)
		DCA	PHSNAME1	# OBTAIN THE JOB'S 2CADR FROM STORAGE
		DXCH	GOLOC		# [Modern: Load job address into GOLOC]

# --------------------------------------------------------------------------
# ITSAJOB - JOB RESTART HANDLER
# --------------------------------------------------------------------------
# [Modern: Job Scheduling Dispatch - allocates work areas based on type]
#
# Restarts a job using either FINDVAC or NOVAC based on priority sign:
#   - Positive PHSPRDT1: FINDVAC (job requires VAC area for interpreter)
#   - Negative PHSPRDT1: NOVAC (basic job, no VAC area needed)
#
# VAC areas provide temporary storage for interpretive computations.
# If no VAC areas available, FINDVAC triggers Alarm 1201.
# If no core sets available, NOVAC/FINDVAC trigger Alarm 1202.
# --------------------------------------------------------------------------
ITSAJOB		NDX	TEMP2G		# INDEX BY 2*(GROUP-1)
		CA	PHSPRDT1	# GET PRIORITY FROM STORAGE
CHKNOVAC	TS	GOLOC -1	# SAVE PRIO UNTIL WE SEE IF ITS
		EXTEND			# A FINDVAC OR A NOVAC
		BZMF	ITSNOVAC	# [Modern: Branch if priority negative]

		CAF	FVACCADR	# POSITIVE PRIO: SET UP FINDVAC CALL
		XCH	GOLOC -1	# PICK UP PRIO (NOW IN A)
		TC	GOLOC -1	# AND GO TO FINDVAC

ITSNOVAC	CAF	NOVACADR	# NEGATIVE PRIO: SET UP NOVAC CALL
		XCH	GOLOC -1	# SET UP NOVAC CALL,
		COM			# CORRECT PRIO (MAKE POSITIVE)
		TC	GOLOC -1	# AND GO TO NOVAC

# --------------------------------------------------------------------------
# ITSATBL - TABLE-DRIVEN RESTART HANDLER
# --------------------------------------------------------------------------
# [Modern: Table Phase Handler - uses restart routing tables (CADRTAB/PRDTTAB)]
#
# Processes restarts defined in the fixed restart tables. Uses the CYR
# (Cycle Right) register to detect odd vs even phase:
#   - ODD phase: Single table entry at X.YSPOT (one restart)
#   - EVEN phase: Double table entry (two restarts at single checkpoint)
#
# Table structure (from RESTART_TABLES.agc):
#   - SIZETAB: Index offsets to find correct table position for each group
#   - CADRTAB (12001): Contains 2CADR restart addresses
#   - PRDTTAB (12000): Contains priority (jobs) or delta-time (tasks)
#
# Entry type identification:
#   - Positive CADR: Job restart (ITSAJOB2)
#   - Negative CADR with BIT10 set: Waitlist task (ITSWTLST)
#   - Negative CADR without BIT10: Longcall (ITSLGCL1)
# --------------------------------------------------------------------------
ITSATBL		TS	CYR		# FIND OUT IF THE PHASE IS ODD OR EVEN
		CCS	CYR		# [Modern: Cycle right, test low bit]
		TCF	+1		# LOW BIT=0: IT'S EVEN (ORIGINAL ODD+1)
		TCF	ITSEVEN		# LOW BIT=1: IT'S ODD (DOUBLE ENTRY)

		CA	RTRNCADR	# IN CASE THIS IS THE SECOND PART OF A
		TS	GOLOC +2	# TYPE B RESTART, WE NEED PROPER EXIT

		CA	TEMPPHS		# SET UP POINTER FOR FINDING OUR PLACE IN
		TS	SR		# THE RESTART TABLES (SHIFT RIGHT)
		AD	SR		# PHASE + PHASE/2 = 1.5*PHASE (3 WORDS/ENTRY)
# Page 1306
		NDX	TEMP2G		# INDEX BY 2*(GROUP-1)
		AD	SIZETAB +1	# ADD ODD TABLE BASE OFFSET
		TS	POINTER		# [Modern: POINTER = table entry address]

CONTBL2		EXTEND			# FIND OUT WHAT'S IN THE TABLE
		NDX	POINTER		# INDEX INTO RESTART TABLES
		DCA	CADRTAB		# GET THE 2CADR (CADR IN A, BBCON IN L)

		LXCH	GOLOC +1	# STORE THE BB (BANK) INFORMATION

		CCS	A		# IS IT A JOB OR IS IT TIMED TASK
		INCR	A		# POSITIVE CADR: MUST BE A JOB
		TCF	ITSAJOB2	# [Modern: Go to job handler]

		INCR	A		# NEGATIVE: WAITLIST OR LONGCALL
		TS	GOLOC		# STORE THE CORRECT CADR (COMPLEMENTED)

		CA	WTLTCADR	# SET UP OUR EXIT TO WAITLIST (DEFAULT)
		TS	GOLOC -1

		CA	GOLOC +1	# NOW FIND OUT IF IT IS A WAITLIST CALL
		MASK	BIT10		# THIS SHOULD BE ONE IF WE HAVE -BBCON
		CCS	A		# BIT10 SET IN NEGATIVE BBCON INDICATES
					# WAITLIST (BITS 4-10 SET IN -BBCON)
					# [Modern: Discriminate waitlist vs longcall]
		TCF	ITSWTLST	# BIT10 SET: IT IS A WAITLIST CALL

		NDX	POINTER		# BIT10 CLEAR: IT'S A LONGCALL
		CA	PRDTTAB		# GET DELTA T ADDRESS FOR THIS LONGCALL

		TCF	ITSLGCL1	# NOW GO GET THE DELTA TIME

# ***** THIS MUST BE IN FIXED FIXED *****

		BLOCK	02
		SETLOC	FFTAG2
		BANK

		COUNT*	$$/RSROU
# --------------------------------------------------------------------------
# ITSLGCL1 - LONGCALL DELTA TIME RETRIEVAL (FIXED-FIXED BANK)
# --------------------------------------------------------------------------
# [Modern: Switched-Bank Delta Time Fetch for Longcall Restart]
#
# Fetches the double-word delta time for a longcall from switched erasable.
# Must be in fixed-fixed bank (BLOCK 02) to access all erasable banks.
# Uses bank switching via BB register to access correct E-bank.
# --------------------------------------------------------------------------
ITSLGCL1	LXCH	GOLOC +1	# OBTAIN THE CORRECT E BANK
		LXCH	BB		# SWAP CURRENT BB WITH TASK'S BB
		LXCH	GOLOC +1	# AND PRESERVE OUR E AND F BANKS

		EXTEND			# GET THE DELTA TIME (DOUBLE PRECISION)
		NDX	A		# A CONTAINS -GENADR OF DELTA TIME
		DCA	0		# LOAD DP DELTA TIME INTO A,L

		LXCH	GOLOC +1	# RESTORE OUR E AND F BANK
		LXCH	BB		# RESTORE THE TASKS E AND F BANKS
		LXCH	GOLOC +1	# AND PRESERVE OUR L (LOW WORD OF DT)
# Page 1307
		TCF	ITSLGCL2	# NOW LET'S PROCESS THIS LONGCALL

# ***** YOU MAY RETURN TO  SWITCHED FIXED *****

		BANK	01
		SETLOC	RESTART
		BANK

		COUNT*	$$/RSROU
# --------------------------------------------------------------------------
# ITSLGCL2 - LONGCALL TIME CALCULATION AND DISPATCH
# --------------------------------------------------------------------------
# [Modern: Long-Duration Task Deadline Calculator]
#
# Calculates remaining time for longcall restart:
#   remaining = original_delta - (TIME2 - LONGBASE)
#             = original_delta - TIME2 + LONGBASE
#
# Where:
#   - original_delta: DP delta time from erasable storage
#   - TIME2: Current double-precision time (28-bit)
#   - LONGBASE: Time base when longcall was originally scheduled
#
# If remaining time > 0: Schedule via LONGCALL routine
# If remaining time <= 0: Task is overdue, restart immediately via WAITLIST
# --------------------------------------------------------------------------
ITSLGCL2	DXCH	LONGTIME	# STORE DP DELTA TIME

		EXTEND			# CALCULATE TIME LEFT
		DCS	TIME2		# GET -TIME2 (CURRENT TIME)
		DAS	LONGTIME	# LONGTIME = DELTA - TIME2
		EXTEND
		DCA	LONGBASE	# GET LONGBASE (ORIGINAL TIME BASE)
		DAS	LONGTIME	# LONGTIME = DELTA - TIME2 + LONGBASE

		CCS	LONGTIME	# FIND OUT HOW THIS SHOULD BE RESTARTED
		TCF	LONGCLCL	# HIGH WORD POSITIVE: USE LONGCALL
		TCF	+2		# HIGH WORD +0: CHECK LOW WORD
		TCF	IMEDIATE -3	# HIGH WORD NEGATIVE: OVERDUE, IMMEDIATE
		CCS	LONGTIME +1	# CHECK LOW WORD
		TCF	LONGCLCL	# LOW WORD POSITIVE: USE LONGCALL
		NOOP			# CAN'T GET HERE (CCS SKIP)
		TCF	IMEDIATE -3	# LOW WORD NEGATIVE: OVERDUE, IMMEDIATE
		TCF	IMEDIATE	# LOW WORD -0: IMMEDIATE

LONGCLCL	CA	LGCLCADR	# WE WILL GO TO LONGCALL
		TS	GOLOC -1	# [Modern: Set up LONGCALL entry]

		EXTEND			# PREPARE OUR ENTRY TO LONGCALL
		DCA	LONGTIME	# LOAD REMAINING DP DELTA TIME
		TC	GOLOC -1	# AND GO TO LONGCALL

# --------------------------------------------------------------------------
# ITSLNGCL - VARIABLE LONGCALL RESTART HANDLER (TYPE C LONGCALL)
# --------------------------------------------------------------------------
# [Modern: Variable Phase Longcall Handler]
#
# Handles longcall restarts from variable phase storage (Type C with C bit).
# Gets delta time address from PHSPRDT1 (stored as -GENADR).
# --------------------------------------------------------------------------
ITSLNGCL	CA	WTLTCADR	# ASSUME IT WILL GO TO WAITLIST
		TS	GOLOC -1	# (MAY CHANGE TO LONGCALL LATER)

		NDX	TEMP2G		# INDEX BY 2*(GROUP-1)
		CS	PHSPRDT1	# GET THE DELTA T ADDRESS (COMPLEMENT)

		TCF	ITSLGCL1	# NOW GET THE DELTA TIME

# --------------------------------------------------------------------------
# ITSWTLST - TABLE WAITLIST RESTART HANDLER
# --------------------------------------------------------------------------
# [Modern: Table-Driven Waitlist Task Handler]
#
# Handles waitlist restarts from restart tables. Corrects BBCON sign and
# retrieves delta time from PRDTTAB.
# --------------------------------------------------------------------------
ITSWTLST	CS	GOLOC +1	# CORRECT THE BBCON INFORMATION
		TS	GOLOC +1	# (WAS STORED NEGATIVE IN TABLE)

		NDX	POINTER		# GET THE DT AND FIND OUT IF IT WAS STORED
		CA	PRDTTAB		# DIRECTLY OR INDIRECTLY

		TCF	TIMETEST	# FIND OUT HOW THE TIME IS STORED

# Page 1308
# --------------------------------------------------------------------------
# ITSAJOB2 - TABLE JOB RESTART HANDLER
# --------------------------------------------------------------------------
# [Modern: Table-Driven Job Handler]
#
# Handles job restarts from restart tables. Gets priority from PRDTTAB
# and uses CHKNOVAC to dispatch to FINDVAC or NOVAC.
# --------------------------------------------------------------------------
ITSAJOB2	XCH	GOLOC		# STORE THE CADR

		NDX	POINTER		# INDEX INTO PRDTTAB
		CA	PRDTTAB		# GET PRIORITY FOR THIS JOB

		TCF	CHKNOVAC	# GO CHECK FINDVAC VS NOVAC

# --------------------------------------------------------------------------
# ITSEVEN - EVEN PHASE (DOUBLE TABLE ENTRY) HANDLER
# --------------------------------------------------------------------------
# [Modern: Double-Entry Restart Handler]
#
# Even phases (G.EVEN like 4.6) have two table entries (6 words total).
# This routine sets up the pointer for the first entry and processes it.
# After first entry completes, PHSPART2 handles the second entry.
#
# POINTER calculation: SIZETAB + 3*PHASE
#   (3 words per entry: PRDTTAB + 2CADR)
# --------------------------------------------------------------------------
ITSEVEN		CA	TEMPSWCH	# SET UP FOR EITHER THE SECOND PART OF THE
		TS	GOLOC +2	# TABLE, OR A RETURN FOR THE NEXT GROUP

		NDX	TEMP2G		# INDEX BY 2*(GROUP-1)
		CA	SIZETAB		# GET EVEN TABLE BASE OFFSET
		AD	TEMPPHS		# ADD PHASE * 3 (3 WORDS PER ENTRY)
		AD	TEMPPHS		# [Modern: POINTER = BASE + 3*PHASE]
		AD	TEMPPHS
		TS	POINTER

		TCF	CONTBL2		# NOW PROCESS WHAT IS IN THE TABLE

# --------------------------------------------------------------------------
# PHSPART2 - SECOND ENTRY OF DOUBLE TABLE RESTART
# --------------------------------------------------------------------------
# [Modern: Double-Entry Second Phase Handler]
#
# Processes the second entry of a double (even) restart. Called after
# the first entry has been dispatched to job/task/longcall handler.
# --------------------------------------------------------------------------
PHSPART2	CA	THREE		# SET THE POINTER FOR THE SECOND HALF OF
		ADS	POINTER		# THE TABLE (ADVANCE BY 3 WORDS)

		CA	RTRNCADR	# THIS WILL BE OUR LAST TIME THROUGH THE
		TS	GOLOC +2	# EVEN TABLE, SO AFTER IT GET THE NEXT
					# GROUP
		TCF	CONTBL2		# SO LET'S GET THE SECOND ENTRY IN THE TBL

# --------------------------------------------------------------------------
# TEMPORARY STORAGE AND CONSTANT DEFINITIONS
# --------------------------------------------------------------------------
# [Modern: Local Variable Allocation and Address Constants]
#
# Temporary storage uses MPAC area (shared with interpreter) since
# RESTARTS runs with interrupts inhibited and interpreter is not active.
#
# GOLOC area (VAC5+20D) stores the restart target address and is used
# to transfer control to job/task/longcall scheduling routines.
# --------------------------------------------------------------------------
TEMPPHS		EQUALS	MPAC		# CURRENT PHASE VALUE (|PHASE|+1)
TEMP2G		EQUALS	MPAC +1		# 2*(GROUP-1) FOR INDEXING
POINTER		EQUALS	MPAC +2		# TABLE POINTER FOR RESTART TABLES
TEMPSWCH	EQUALS	MPAC +3		# EXIT SWITCH (PHS2CADR OR RTRNCADR)
GOLOC		EQUALS	VAC5 +20D	# RESTART TARGET ADDRESS STORAGE
					# GOLOC-1: ROUTINE ENTRY (FINDVAC/NOVAC/WAITLIST)
					# GOLOC+0: CADR OF TARGET
					# GOLOC+1: BBCON OF TARGET
					# GOLOC+2: RETURN ADDRESS
MINUS2		EQUALS	NEG2		# CONSTANT -2 FOR CCS DISCRIMINATION
OCT177		EQUALS	LOW7		# MASK FOR PHASE BITS (0-127)

# --------------------------------------------------------------------------
# ADDRESS CONSTANTS FOR RESTART DISPATCH
# --------------------------------------------------------------------------
# [Modern: Jump Table Addresses for Restart Handlers]
#
# GENADR generates a 12-bit address suitable for TC/TCF instructions.
# These provide indirect jumps to Executive and Waitlist routines.
# --------------------------------------------------------------------------
PHS2CADR	GENADR	PHSPART2	# SECOND PART OF EVEN PHASE
PRT2CADR	GENADR	GETPART2	# TABLE PART OF TYPE B PHASE
LGCLCADR	GENADR	LONGCALL	# LONGCALL SCHEDULING ROUTINE
FVACCADR	GENADR	FINDVAC		# JOB WITH VAC AREA (INTERPRETER)
WTLTCADR	GENADR	WAITLIST	# TIME-DELAYED TASK SCHEDULING
NOVACADR	GENADR	NOVAC		# JOB WITHOUT VAC AREA (BASIC)
