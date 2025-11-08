# Copyright:	Public domain.
# Filename:	PHASE_TABLE_MAINTENANCE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1294-1302
# Mod history:	2009-05-26 OH	Transcribed from page images.
#		2009-06-05 RSB	A few lines at the bottom of page image
#				1294 were truncated.  I've fixed the page
#				image and added those missing lines here.
#		2011-05-07 JL	Flagged SBANK= workaround for future
#				removal.

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
; FILE: PHASE_TABLE_MAINTENANCE.agc
; MODULE: Phase Table Management
; MISSION PHASE: All phases (launch/earth-orbit/trans-lunar/lunar-orbit/
;                descent/landing/ascent/rendezvous/trans-earth/re-entry)
;
; TL;DR: Manages program phase state for AGC restart protection system.
;        Maintains phase tables that allow the computer to restart mission
;        programs at safe points after power transients or system resets.
;        Provides PHASCHNG routine for updating phase information and
;        NEWMODEX/NEWMODEA for displaying current program mode on DSKY.
;
; COMMENT-ONLY READERS: This code ensures that if the guidance computer
;        experiences a restart during critical mission operations (like
;        lunar landing), it can resume at a safe checkpoint rather than
;        starting over from scratch. Think of it as a sophisticated save
;        system for the spacecraft's computer.
; CODE-ALONG READERS: Implements three types of phase changes (Type A, B, C)
;        encoded in octal parameters. Integrates with RESTART_TABLES.agc
;        for actual restart data and EXECUTIVE.agc for job scheduling.
;        Critical for 1202 alarm recovery during Apollo 11 descent.
; ============================================================================

# Page 1294
# SUBROUTINE TO UPDATE THE PROGRAM NUMBER DISPLAY ON THE DSKY.

		COUNT*	$$/PHASE
		BLOCK	02
		SETLOC	FFTAG1
		BANK

; ============================================================================
; MODE REGISTER UPDATE AND DISPLAY ROUTINES
;
; These routines update the MODREG (mode register) which holds the current
; program number (P00-P99), and trigger the DSKY display to show the new
; program mode to the crew. During Apollo 11's descent, the crew watched
; program numbers transition from P63 (braking phase) through P64 (approach)
; to P66 (landing) on their DSKY displays.
; ============================================================================

NEWMODEX	INDEX	Q		# UPDATE MODREG.  ENTRY FOR MODE IN FIXED.
		CAF	0		# Load mode number from location after TC
		INCR	Q		# Increment Q to skip mode parameter

; Mode number now in A register. Store it and display to crew.
NEWMODEA	TS	MODREG		# ENTRY FOR MODE IN A. Store program mode
MMDSPLAY	CAF	+3		# DISPLAY MAJOR MODE. Prepare for display
PREBJUMP	LXCH	BBANK		# PUTS BBANK IN L. Save bank register
		TCF	BANKJUMP	# PUTS Q INTO A. Jump with bank switch
		CADR	SETUPDSP	# Address of display setup routine

# RETURN TO CALLER +3 IF MODE = THAT AT CALLER +1.  OTHERWISE RETURN TO CALLER +2.

; The CHECKMM routine allows program code to verify the current mode before
; taking action. For example, during landing the computer checks whether
; P66 (landing mode) is active before processing certain radar data.

CHECKMM		INDEX	Q		# Check if current mode matches expected
		CS	0		# Load complement of expected mode
		AD	MODREG		# Add current mode (subtract via complement)
		EXTEND			# Extended instruction follows
		BZF	Q+2		# Branch if zero (mode match) to Q+2
		TCF	Q+1		# NO MATCH - return to caller +2

TCQ		=	Q+2 +1		# Symbolic definition: Q+3 return address

		BANK	14
		SETLOC	PHASETAB
		BANK

		COUNT*	$$/PHASE
; SETUPDSP schedules a display job to update the DSKY major mode display.
; This runs asynchronously via NOVAC so the calling program doesn't have to
; wait for display hardware updates. The crew sees the new program number
; appear on their DSKY within a fraction of a second.

SETUPDSP	INHINT			# Disable interrupts during setup
		DXCH	RUPTREG1	# SAVE CALLER'S RETURN 2CADR for job
		CAF	PRIO30		# Priority 30 for display job
		TC	NOVAC		# Schedule display job (no VAC area needed)
		EBANK=	MODREG		# Set erasable bank to access MODREG
		2CADR	DSPMMJOB	# Address of display job routine

		DXCH	RUPTREG1	# Restore caller's return address
		RELINT			# Re-enable interrupts
		DXCH	Z		# RETURN to caller

DSPMMJOB	EQUALS	DSPMMJB		# Display job defined elsewhere

		BLOCK	02
		SETLOC	FFTAG1
		BANK

# Page 1295
; ============================================================================
; PHASCHNG - PRIMARY RESTART PHASE CHANGE ROUTINE
;
; PHASCHNG is the heart of the AGC's restart protection system. During
; Apollo 11's descent, when the 1202 program alarm occurred due to computer
; overload, this system allowed the AGC to restart gracefully and continue
; the landing rather than aborting. Without PHASCHNG, every computer restart
; would have forced a mission abort.
;
; The routine provides three distinct ways to update phase information:
; Type A (fixed), Type B (variable+fixed), and Type C (variable). Each
; "group" (1-7) represents a major program area, and each "phase" within
; a group represents a specific checkpoint within that program.
; ============================================================================

# PHASCHNG IS THE MAIN WAY OF MAKING PHASE CHANGES FOR RESTARTS.  THERE ARE THREE FORMS OF PHASCHNG, KNOWN AS TYPE
# A, TYPE B, AND TYPE C.  THEY ARE ALL CALLED AS FOLLOWS, WHERE OCT XXXXX CONTAINS THE PHASE INFORMATION,
#		TC	PHASCHNG
#		OCT	XXXXX

; UNDERSTANDING RESTART GROUPS AND PHASES:
;
; During a mission, multiple programs run concurrently (guidance, navigation,
; display updates). Each major program area is assigned a "group" (1-7).
; Within each group, execution progresses through numbered phases (0-127).
; When a restart occurs, the AGC consults the phase tables to determine
; what jobs, tasks, or longcalls should be restarted for each active group.

# TYPE A IS CONCERNED WITH FIXED PHASE CHANGES, THAT IS, PHASE INFORMATION THAT IS STORED PERMANENTLY.  THESE
# OPTIONS ARE, WHERE G STANDS FOR A GROUP AND .X FOR THE PHASE,
#	G.0		INACTIVE, WILL NOT PERMIT A GROUP G RESTART
#	G.1		WILL CAUSE THE LAST DISPLAY TO BE REACTIVATED, USED MAINLY IN MANNED FLIGHTS
#	G.EVEN		A DOUBLE TABLE RESTART, CAN CAUSE ANY COMBINATION OF TWO JOBS, TASKS, AND/OR
#			LONGCALL TO BE RESTARTED.
#	G.ODD NOT .1	A SINGLE TABLE RESTART, CAN CAUSE EITHER A JOB, TASK, OR LONGCALL RESTART.
#
; TYPE A PHASE CHANGES - Fixed restart checkpoints:
; G.0 = Group inactive (no restart permitted) - used when program completes
; G.1 = Reactivate last display - ensures crew sees current program state
; G.EVEN = Double table (two restart entries) - complex restart scenarios
; G.ODD (not .1) = Single table (one restart entry) - simple restart point

# THIS INFORMATION IS PUT INTO THE OCTAL WORD AFTER TC PHASCHNG AS FOLLOWS
#	TL0 00P PPP PPP GGG
# WHERE EACH LETTER OR NUMBER STANTS FOR A BIT.  THE G'S STAND FOR THE GROUP, OCTAL 1-7, THE P'S FOR THE PHASE,
# OCTAL 0 - 127.  0'S MUST BE 0.  IF ONE WISHES TO HAVE THE TBASE OF GROUP G TO BE SET AT THIS TIME,
# T IS SET TO 1, OTHERWISE IT IS SET TO 0.  SIMILARLY IF ONE WISHES TO SET LONGBASE, THEN L IS SET TO 1, OTHERWISE
# IT IS SET TO 0.  SOME EXAMPLES,
#		TC	PHASCHNG	# THIS WILL CAUSE GROUP 3 TO BE SET TO 0,
#		OCT	00003		# MAKING GROUP 3 INACTIVE
#
#		TC	PHASCHNG	# IF A RESTART OCCURS THIS WOULD CAUSE
#		OCT	00012		# GROUP 2 TO RESTART THE LAST DISPLAY
#
#		TC	PHASCHNG	# THIS SETS THE TBASE OF GROUP 4 AND IN
#		OCT	40064		# CASE OF A RESTART WOULD START UP THE TWO
#					# THINGS LOCATED IN THE DOUBLE 4.6 RESTART
#					# LOCATION.
#
#		TC	PHASCHNG	# THIS SETS LONGBASE AND UPON A RESTART
#		OCT	20135		# CAUSES 5.13 TO BE RESTARTED (SINCE
#					# LONGBASE WAS SET THIS SINGLE ENTRY
#					# SHOULD BE A LONGCALL)
#
#		TC	PHASCHNG	# SINCE BOTH TBASE4 AND LONGBASE ARE SET,
#		OCT	60124		# 4.12 SHOULD CONTAIN BOTH A TASK AND A
#					# LONGCALL TO BE RESTARTED
#
# TYPE C PHASCHNG CONTAINS THE VARIABLE TYPE OF PHASCHNG INFORMATION.  INSTEAD OF THE INFORMATION BEING IN A
# PERMANENT FORM, ONE STORES THE DESIRED RESTART INFORMATION IN A VARIABLE LOCATION.  THE BITS ARE AS FOLLOWS,
#	TL0 1AD XXX CJW GGG
# WHERE EACH LETTER OR NUMBER STANDS FOR A BIT.  THE G'S STAND FOR THE GROUP, OCTAL 1 - 7.  IF THE RESTART IS TO
# BE BY WAITLIST, W IS SET TO 1, IF IT IS A JOB, J IS SET TO 1, IF IT IS A LONGCALL, C IS SET TO 1.  ONLY ONE OF
# THESE THREE BITS MAY BE SET.  X'S ARE IGNORED, 1 MUST BE 1, AND 0 MUST BE 0.  AGAIN T STANDS FOR THE TBASE,
# Page 1296
# AND L FOR LONGBASE.  THE BITS A AND D ARE CONCERNED WITH THE VARIABLE INFORMATION.  IF D IS SET TO 1, A PRIORITY
# OR DELTA TIME WILL BE READ FROM THE NEXT LOCATION AFTER THE OCTAL INFORMATION., IF THIS IS TO BE INDIRECT, THAT
# IS, THE NAME OF A LOCATION CONTAINING THE INFORMATION (DELTA TIME ONLY), THEN THIS IS GIVEN AS THE -GENADR OF
# THAT LOCATION WHICH CONTAINS THE DELTA TIME.  IF THE OLD PRIORITY OR DELTA TIME IS TO BE USED, THAT WHICH IS
# ALREADY IN THE VARIABLE STORAGE, THEN D IS SET TO 0.  NEXT THE A BIT IS USED.  IF IT IS SET TO 0, THE ADDRESS
# THAT WOULD BE RESTARTED DURING A RESTART IS THE NEXT LOCATION AFTER THE PHASE INFORMATION, THAT IS, EITHER
# (TC PHASCHNG) +2 OR +3, DEPENDING ON WHETHER D HAD BEEN SET OR NOT.  IF A IS SET TO 1, THEN THE ADDRESS THAT
# WOULD BE RESTARTED IS THE 2CADR THAT IS READ FROM THE NEXT TWO LOCATION.  EXAMPLES,
#	AD	TC	PHASCHNG	# THIS WOULD CAUSE LOCATION AD +3 TO BE
#	AD+1	OCT	05023		# RESTARTED BY GROUP THREE WITH A PRIORITY
#	AD+2	OCT	23000		# OF 23.  NOTE UPON RETURNING IT WOULD
#	AD+3				# ALSO GO TO AD+3
#
#	AD	TC	PHASCHNG	# GROUP 1 WOULD CAUSE CALLCALL TO BE
#	AD+1	OCT	27441		# BE STARTED AS A LONGCALL FROM THE TIME
#	AD+2	-GENADR	DELTIME		# STORED IN LONGBASE (LONGBASE WAS SET) BY
#	AD+3	2CADR	CALLCALL	# A DELTA TIME STORED IN DELTIME.  THE
#	AD+4				# BBCON OF THE 2CADR SHOULD CONTAIN THE E
#	AD+5				# BANK OF DELTIME.  PHASCHNG RETURNS TO
#					# LOCATION AD+5
#
# NOTE THAT IF A VARIABLE PRIORITY IS GIVEN FOR A JOB, THE JOB WILL BE RESTARTED AS A NOVAC IF THE PRIORITY IS
# NEGATIVE, AS A FINDVAC IF THE PRIORITY IS POSITIVE.
#
# TYPE B PHASCHNG IS A COMBINATION OF VARIABLE AND FIXED PHASE CHANGES.  IT WILL START UP A JOB AS INDICATED
# BELOW AND ALSO START UP ONE FIXED RESTART, THAT IS EITHER AN G.1 OR A G.ODD OR THE FIRST ENTRY OF G.EVEN
# DOUBLE ENTRY.  THE BIT INFORMATION IS AS FOLLOW,
#	TL1 DAP PPP PPP GGG
# WHERE EACH LETTER OR NUMBER STANDS FOR A BIT.  THE G'S STAND FOR THE GROUP, OCTAL 1 - 7, THE P'S FOR THE FIXED
# PHASE INFORMATION, OCTAL 0 - 127.  1 MUST BE 1.  AND AGAIN T STANDS FOR THE TBASE AND L FOR LONGBASE.  D THIS
# TIME STANDS ONLY FOR PRIORITY SINCE THIS WILL BE CONSIDERED A JOB, AND IT MUST BE GIVEN DIRECTLY IF GIVEN.
# AGAIN A STANDS FOR THE ADDRESS OF THE LOCATION TO BE RESTARTED, 1 IF THE 2CADR IS GIVEN, OR 0 IF IT IS TO BE
# THE NEXT LOCATION.  (THE RETURN LOCATION OF PHASCHNG) EXAMPLES,
#	AD	TC	PHASCHNG	# TBASE IS SET AND A RESTART CAUSE GROUP 3
#	AD+1	OCT	56043		# TO START THE JOB AJOBAJOB WITH PRIORITY
#	AD+2	OCT	31000		# 31 AND THE FIRST ENTRY OF 3.4SPOT (WE CAN
#	AD+3	2CADR	AJOBAJOB	# ASSUME IT IS A TASK SINCE WE SET TBASE3)
#	AD+4				# UPON RETURN FROM PHASCHNG CONTROL WOULD
#	AD+5				# GO TO AD+5
#
#	AD	TC	PHASCHNG	# UPON A RESTART THE LAST DISPLAY WOULD BE
#	AD+1	OCT	10015		# RESTARTED AND A JOB WITH THE PREVIOUSLY
#	AD+2				# STORED PRIORITY WOULD BE BEGUN AT AD+2
#					# BY MEANS OF GROUP 5
# Page 1297
# THE NOVAC-FINDVAC CHOICE FOR JOBS HOLDS HERE ALSO -- NEGATIVE PRIORITY CAUSES A NOVAC CALL, POSITIVE A FINDVAC.

# SUMMARY OF BITS:
#	TYPE A		TL0 00P PPP PPP GGG
#	TYPE B		TL1 DAP PPP PPP GGG
#	TYPE C		TL0 1AD XXX CJW GGG

# Page 1298
# 2PHSCHNG IS USED WHEN ONE WISHES TO START UP A GROUP OR CHANGE A GROUP WHILE UNDER THE CONTROL OF A DIFFERENT
# GROUP.  FOR EXAMPLE, CHANGE THE PHASE OF GROUP 3 WHILE THE PORTION OF THE PROGRAM IS UNDER GROUP 5.  ALL 2PHSCHNG
# CALLS ARE MADE IN THE FOLLOWING MANNER,
#		TC	2PHSCHNG
#		OCT	XXXXX
#		OCT	YYYYY
# WHERE OCT XXXXX MUST BE OF TYPE A AND OCT YYYYY MAY BE OF EITHER TYPE A OR TYPE B OR TYPE C.  THERE IS ONE
# DIFFERENCE --- NOTE: IF LONGBASE IS TO BE SET THIS INFORMATION IS GIVEN IN THE OCT YYYYY INFORMATION, IT WILL
# BE DISREGARDED IF GIVEN WITH THE OCT XXXXX INFORMATION.  A COUPLE OF EXAMPLES MAY HELP,
#	AD	TC	2PHACHNG	# SET TBASE3 AND IF A RESTART OCCURS START
#	AD+1	OCT	40083		# THE TWO ENTRIES IN 3.8 TABLE LOCATION
#	AD+2	OCT	05025		# THIS IS OF TYPE C, SET THE JOB TO BE
#	AD+3	OCT	18000		# TO BE LOCATION AD+4, WITH A PRIORITY 18,
#	AD+4				# FOR GROUP 5 PHASE INFORMATION.

## [WORKAROUND] RSB 2004
		SBANK=	PINSUPER
## [WORKAROUND]

		COUNT*	$$/PHASE
2PHSCHNG	INHINT			# THE ENTRY FOR A DOUBLE PHASE CHANGE
		NDX	Q
		CA	0
		INCR	Q
		TS	TEMPP2

		MASK	OCT7
		DOUBLE
		TS	TEMPG2

		CA	TEMPP2
		MASK	OCT17770	# NEED ONLY 1770, BUT WHY GET A NEW CONST.
		EXTEND
		MP	BIT12
		XCH	TEMPP2

		MASK	BIT15
		TS	TEMPSW2		# INDICATES WHETHER TO SET TBASE OR NOT

		INDEX	Q
		CA	0
		INCR	Q
		TS	TEMPSW

		TCF	PHASJUMP

; ============================================================================
; TRANSITION: From 2PHSCHNG initialization to main PHASCHNG processing
;
; The 2PHSCHNG routine has extracted the first octal parameter and stored
; group/phase information in temporary variables. Now control merges with
; the main PHASCHNG entry point to process the phase change request. During
; restart operations, this code determines which jobs, tasks, or programs
; should be restarted and with what priority levels.
; ============================================================================

PHASCHNG	INHINT			# NORMAL PHASCHNG ENTRY POINT.
		INDEX	Q
		CA	0
		INCR	Q
PHSCHNGA	INHINT			# FIRST OCTAL PARAMETER IN A.
# Page 1299
; Store the octal phase parameter and mark this as a single PHASCHNG call
; (not a 2PHSCHNG double phase change). The phase parameter encodes the
; group number, phase information, and control bits for restart behavior.
		TS	TEMPSW
		CA	ONE
		TS	TEMPSW2
		
; Transfer control to PHSCHNG2 in the switched erasable bank where phase
; table storage occurs. This bank switching is necessary to access the
; phase table arrays (PHASE1, -PHASE1, PHSPRDT1, PHSNAME1) which track
; restart state for all seven restart groups.
PHASJUMP	EXTEND
		DCA	ADRPCHN2	# OFF TO SWITCHED BANK
		DTCB

		EBANK=	LST1
ADRPCHN2	2CADR	PHSCHNG2

; ============================================================================
; ONEORTWO - Determine Phase Change Type and Process Parameters
;
; After storing phase information, control reaches this section which examines
; control bits to determine if this is Type B or Type C phase change, whether
; to use old or new priority values, and whether a 2CADR restart address is
; explicitly provided. These variations allow PHASCHNG to support different
; restart scenarios across the mission.
; ============================================================================

ONEORTWO	LXCH	TEMPBBCN
		LXCH	BBANK
		LXCH	TEMPBBCN

; Check bit 13 to determine phase change type. If set, this is Type B which
; has special priority handling. Otherwise it's Type C with inline parameters.
		MASK	OCT14000	# SEE WHAT KIND OF PHASE CHANGE IT IS
		CCS	A
		TCF	CHECKB		# IT IS OF TYPE `B'.

; For Type C, check bit 7 of phase value to see if new priority is provided
; or if we should reuse the priority from the previous phase of this group.
		CA	TEMPP
		MASK	BIT7
		CCS	A		# SHALL WE USE THE OLD PRIORITY
		TCF	GETPRIO		# NO GET A NEW PRIORITY (OR DELTA T)

; Reuse old priority (or delta-time) from PHSPRDT1 array for this restart
; group. This maintains continuity when phase changes within same subsystem.
OLDPRIO		NDX	TEMPG		# USE THE OLD PRIORITY (OR DELTA T)
		CA	PHSPRDT1 -2
		TS	TEMPPR

; Check bit 8 of phase value to determine if explicit 2CADR is provided.
; If bit 8 set, fetch 2CADR from inline parameter. Otherwise use caller's
; return address as the restart entry point.
CON1		CA	TEMPP		# SEE IF A 2CADR IS GIVEN
		MASK 	BIT8
		CCS	A
		TCF	GETNEWNM

; No explicit 2CADR provided. Use caller's return address (Q) and current
; bank (BB) as the restart entry point. This is common for phase changes
; within a single routine that wants to restart at the calling location.
		CA	Q
		TS	TEMPNM
		CA	BB
		EXTEND			# PICK UP USER'S SUPERBANK
		ROR	SUPERBNK
		TS	TEMPBB

; Transfer back to CON2 in switched bank to complete phase table updates.
TOCON2		CA	CON2ADR		# BACK TO SWITCHED BANK
		LXCH	TEMPBBCN
		DTCB

CON2ADR		GENADR	CON2

; Fetch new priority (or delta-time) from inline parameter following the
; TC PHASCHNG call. Increment Q to skip past this parameter for proper return.
GETPRIO		NDX	Q		# DON'T CARE IF DIRECT OR INDIRECT
		CA	0		# LEAVE THAT DECISION TO RESTARTS
		INCR	Q		# OBTAIN RETURN ADDRESS
# Page 1300
		TCF	CON1 -1

; Fetch explicit 2CADR (address and bank) from inline double-precision
; parameter. This allows restart at arbitrary entry points for complex
; phase transitions involving different subsystems or mission programs.
GETNEWNM	EXTEND
		INDEX	Q
		DCA	0
		DXCH	TEMPNM
		CA	TWO
		ADS	Q		# OBTAIN RETURN ADDRESS

		TCF	TOCON2

OCT14000	EQUALS	PRIO14
TEMPG		EQUALS	ITEMP1
TEMPP		EQUALS	ITEMP2
TEMPNM		EQUALS	ITEMP3
TEMPBB		EQUALS	ITEMP4
TEMPSW		EQUALS	ITEMP5
TEMPSW2		EQUALS	ITEMP6
TEMPPR		EQUALS	RUPTREG1
TEMPG2		EQUALS	RUPTREG2
TEMPP2		EQUALS	RUPTREG3

TEMPBBCN	EQUALS	RUPTREG4
BB		EQUALS	BBANK

		BANK	14
		SETLOC	PHASETAB
		BANK

		EBANK=	PHSNAME1
		COUNT*	$$/PHASE
		
; ============================================================================
; PHSCHNG2 - Main Phase Change Processing Routine
;
; This routine executes in bank 14 with access to the phase table storage
; arrays. It decodes the octal phase parameter into its component fields:
; - Group number (bits 0-2): Identifies which restart group (1-7)
; - Phase information (bits 3-9): Encodes the phase within the group
; - Control bits (bits 10-15): Specify TBASE/LONGBASE setting and type
;
; The decoded information is stored in the phase tables to enable proper
; restart behavior if a power failure or other restart condition occurs.
; ============================================================================

PHSCHNG2	LXCH	TEMPBBCN
		
; Extract group number (bits 0-2) by masking with OCT7 and doubling
; to convert group number 1-7 into table index 2-14 (array stride of 2).
		CA	TEMPSW
		MASK	OCT7
		DOUBLE
		TS	TEMPG

; Extract phase information (bits 3-9) by masking and scaling. The phase
; value determines restart behavior: .0 = inactive, .1 = display restart,
; .EVEN = double table restart, .ODD = single table restart.
		CA	TEMPSW
		MASK	OCT17770
		EXTEND
		MP	BIT12
		TS	TEMPP

; Extract control bits (bits 10-15) for TBASE/LONGBASE and type checking.
		CA	TEMPSW
		MASK	OCT60000
		XCH	TEMPSW
		MASK	OCT14000
		CCS	A
# Page 1301
		TCF	ONEORTWO
		
; Begin storing phase information into the phase table for this restart
; group. The PHASE1 array maintains the current phase for each of the
; seven restart groups, enabling the system to determine which jobs/tasks
; to restart after a power failure or other restart condition.
		CA	TEMPP		# START STORING THE PHASE INFORMATION
		NDX	TEMPG
		TS	PHASE1 -2

; Check if this is a single PHASCHNG or a double 2PHSCHNG. For 2PHSCHNG,
; TEMPSW2 will be zero and we need to store phase information for the second
; restart group as well. This allows atomic updates of two restart groups,
; critical when transitioning between major mission phases.
BELOW1		CCS	TEMPSW2		# IS IT A PHASCHNG OR A 2PHSCHNG
		TCF	BELOW2		# IT'S A PHASCHNG

		TCF	+1		# IT'S A 2PHSCHNG
		
; For 2PHSCHNG, store the second group's phase information in -PHASE1 array.
; The complement of TEMPP2 is stored to indicate the second phase is active.
; This dual-phase mechanism allows coordinated restart of related subsystems.
		CS	TEMPP2
		LXCH	TEMPP2
		NDX	TEMPG2
		DXCH	-PHASE1 -2

		CCS	TEMPSW2
		NOOP			# CAN'T GET HERE
		TCF	BELOW2

; Set TBASE for the second restart group using complement of current time.
; This establishes timing reference for the second group's restart operations.
		CS	TIME1
		NDX	TEMPG2
		TS	TBASE1 -2

; Process control bits to determine if TBASE and/or LONGBASE should be set
; for this restart group. TBASE provides timing reference for time-dependent
; restarts, while LONGBASE supports extended mission phases like translunar
; coast and lunar orbit operations.
BELOW2		CCS	TEMPSW		# SEE IF WE SHOULD SET TBASE OR LONGBASE
		TCF	BELOW3		# SET LONGBASE ONLY
		TCF	BELOW4		# SET NEITHER

; Set TBASE for this restart group to complement of current mission time.
; Complement is used for AGC arithmetic conventions in time calculations.
		CS	TIME1		# SET TBASE TO BEGIN WITH
		NDX	TEMPG
		TS	TBASE1 -2

; Check if LONGBASE should also be set (both T-bit and L-bit enabled).
		CA	TEMPSW		# SHALL WE NOW SET LONGBASE
		AD	BIT14COM
		CCS	A
		NOOP			# ***** CAN'T GET HERE *****
BIT14COM	OCT	17777		# ***** CAN'T GET HERE *****
		TCF	BELOW4		# NO WE NEED ONLY SET TBASE

; Set LONGBASE to current double-precision mission time. LONGBASE provides
; extended time reference for long-duration phases such as translunar coast,
; lunar orbit operations, and transearth return. The double-precision TIME2
; captures full mission elapsed time with higher resolution than TIME1.
BELOW3		EXTEND			# SET LONGBASE
		DCA	TIME2
		DXCH	LONGBASE

; Store complement of phase information in -PHASE1 array to complete phase
; table update. The negative phase storage is used by restart logic to 
; determine phase transitions and maintain restart protection during mission
; operations. With phase tables updated, interrupts can be re-enabled safely.
BELOW4		CS	TEMPP		# AND STORE THE FINAL PART OF THE PHASE
		NDX	TEMPG
		TS	-PHASE1 -2

; Restore return address, re-enable interrupts, and return to caller using
; DTCB (dual return to caller in bank). Phase change is now complete with
; all restart tables properly updated.
		CA	Q
		LXCH	TEMPBBCN
		RELINT
		DTCB
# Page 1302
; ============================================================================
; CON2 - Store Phase Information and Parameters
;
; This section completes phase table updates by storing the phase value,
; priority (or delta-time), and 2CADR name information into the phase tables.
; These values enable proper restart behavior by identifying which job or task
; should execute and with what parameters when a restart occurs.
; ============================================================================

CON2		LXCH	TEMPBBCN

; Store phase information in PHASE1 array for this restart group. The phase
; value determines restart type: .0=inactive, .1=display, .EVEN=double table,
; .ODD=single table restart.
		CA	TEMPP
		NDX	TEMPG
		TS	PHASE1 -2

; Store priority value (for jobs/tasks) or delta-time (for WAITLIST entries)
; in PHSPRDT1 array. This determines restart scheduling and timing.
		CA	TEMPPR
		NDX	TEMPG
		TS	PHSPRDT1 -2

; Store 2CADR (address and bank) of restart routine in PHSNAME1 array. This
; identifies which job, task, or LONGCALL should execute when this restart
; group is activated by a restart condition.
		EXTEND
		DCA	TEMPNM
		NDX	TEMPG
		DXCH	PHSNAME1 -2

; Continue to BELOW1 to check for 2PHSCHNG second group processing.
		TCF	BELOW1

		BLOCK	03
		SETLOC	FFTAG6
		BANK

		COUNT*	$$/PHASE
		
; ============================================================================
; CHECKB - Type B Phase Change Priority Handling
;
; Type B phase changes use bit 12 to indicate whether a new priority value
; should be fetched from an inline parameter or if the previous priority for
; this restart group should be reused. This provides flexibility for Type B
; phase changes that are triggered by 2PHSCHNG double-group updates.
; ============================================================================

; Check bit 12 of phase word. If set, fetch new priority from inline parameter.
; If clear, reuse the priority stored in PHSPRDT1 from previous phase change.
CHECKB		MASK	BIT12		# SINCE THIS IS OF TYPE B, THIS BIT WOULD
		CCS	A		# BE HERE IF WE ARE TO GET A NEW PRIORITY
		TCF	GETPRIO		# IT IS, SO GET NEW PRIORITY

; Bit 12 clear - reuse old priority for this restart group.
		TCF	OLDPRIO		# IT ISN'T, USE THE OLD PRIORITY.


