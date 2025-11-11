# Copyright:    Public domain.
# Filename:     SERVICE_ROUTINES.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1485-1492
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
; FILE: SERVICE_ROUTINES.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Utility service functions providing common operations for mission
;        programs. Implements flag manipulation routines, time delay services,
;        block memory transfer, display light control, and helper functions
;        used throughout Apollo 11 Command Module software to support diverse
;        program needs across all mission phases.
;
; COMMENT-ONLY READERS: This file contains helper functions used by other
;        programs for common calculations and data handling throughout the
;        Apollo 11 mission.
; CODE-ALONG READERS: Study utility function implementations including flag
;        bit manipulation, job delay mechanisms, memory block transfers, and
;        service routine architecture patterns used across the AGC operating system.
; ============================================================================

# Page 1485
		BLOCK	3
		SETLOC	FFTAG6
		BANK
		COUNT	03/FLAG

; ============================================================================
; UPENT2 - FLAG BIT SET ROUTINE (Entry Point 2)
;
; Sets specified bit(s) in a flagword to 1 (true state). Used throughout
; mission programs to enable software flags controlling program behavior.
; During Apollo 11, flags controlled guidance modes, display states, and
; system configurations.
;
; Input:  A register contains flagword number (0-7) and bit pattern
; Output: Specified bit(s) set to 1 in target flagword
;         A register contains modified flagword value
;
; Preserves interrupt-safe operation by using INHINT/RELINT pair.
; ============================================================================

UPENT2		TS	L		# WHICH FLAGWORD IS IT
		MASK	OCT7		# Extract flagword number (0-7)
		XCH	L		# SAVE IN L FOR INDEXING

		MASK	OCT77770	# OBTAIN THE BIT INFORMATION
		INHINT			# PREVENT INTERUPTS (critical section)
		TS	ITEMP1		# STORE THE BIT INFORMATION TEMPORARIALY

		NDX	L		# Index to correct flagword
		CS	FLAGWRD0	# Complement current flagword state
		MASK	ITEMP1		# Mask to isolate bits to set
		NDX	L		# Index to correct flagword again
		ADS	FLAGWRD0	# Add to set bits (double complement logic)
		RELINT			# RELEASE INTERUPT INHIBIT

		INCR	Q		# OBTAIN THE CORRECT RETURN ADDRESS
		TC	Q		# RETURN

; ============================================================================
; DOWNENT2 - FLAG BIT CLEAR ROUTINE (Entry Point 2)
;
; Clears specified bit(s) in a flagword to 0 (false state). Used throughout
; mission programs to disable software flags controlling program behavior.
; During Apollo 11, clearing flags disabled guidance modes, cleared display
; states, and modified system configurations.
;
; Input:  A register contains flagword number (0-7) and bit pattern
; Output: Specified bit(s) cleared to 0 in target flagword
;         A register contains modified flagword value
;
; Preserves interrupt-safe operation by using INHINT/RELINT pair.
; ============================================================================

DOWNENT2	TS	L		# WHICH FLAGWORD IS IT
		MASK	OCT7		# Extract flagword number (0-7)
		XCH	L		# SAVE IN L FOR INDEXING

		MASK	OCT77770	# OBTAIN THE BIT INFORMATION
		COM			# Complement to prepare clear mask

		INHINT			# PREVENT INTERUPTS (critical section)
		NDX	L		# Index to correct flagword
		MASK	FLAGWRD0	# Mask clears specified bits
		NDX	L		# Index to correct flagword again
		TS	FLAGWRD0	# Store cleared flagword
		RELINT			# RELEASE INTERUPT INHIBIT

		INCR	Q		# OBTAIN THE CORRECT RETURN ADDRESS
		TC	Q		# RETURN

OCT7		EQUALS	SEVEN
		BANK	10

# Page 1486
#
#	UPFLAG AND DOWNFLAG ARE ENTIRELY GENERAL FLAG SETTING AND CLEARING SUBROUTINES.  USING THEM, WHETHER OR
# NOT IN INTERRUPT, ONE MAY SET OR CLEAR ANY SINGLE, NAMED BIT IN ANY ERASABLE REGISTER, SUBJECT OF COURSE TO
# EBANK SETTING.  A "NAMED" BIT, AS THE WORD IS USED HERE, IS ANY BIT WITH A NAME FORMALLY ASSIGNED BY THE YUL
# ASSEMBLER.
#
#	AT PRESENT THE ONLY NAMED BITS ARE THOSE IN THE FLAGWORDS.  ASSEMBLER CHANGES WILL MAKE IT POSSIBLE TO
# NAME ANY BIT IN ERASABLE MEMORY.
#
#	CALLING SEQUENCES ARE AS FOLLOWS:-
#
#			TC	UPFLAG			TC	DOWNFLAG
#			ADRES	NAME OF FLAG		ADRES	NAME OF FLAG
#
#	RETURN IS TO THE LOCATION FOLLOWING THE "ADRES" ABOUT .58 MS AFTER THE "TC".
#
#	UPON RETURN A CONTAINS THE CURRENT FLAGWRD SETTING.

; ============================================================================
; UPFLAG/DOWNFLAG - GENERAL PURPOSE FLAG MANIPULATION
;
; These subroutines provide safe flag bit manipulation callable from both
; normal programs and interrupt handlers. During Apollo 11, mission programs
; used these routines to control guidance modes, display states, and system
; configurations. The interrupt-safe design prevented race conditions when
; multiple programs accessed shared flags.
;
; Execution time: 0.58 milliseconds from TC to return
; Used throughout: Launch, TLI, lunar orbit, entry programs
; ============================================================================

		BLOCK	02
		SETLOC	FFTAG1
		BANK
		COUNT*	$$/FLAG

UPFLAG		CA	Q		# Save return address
		TC	DEBIT		# Decode flag address and bit position
		COM			# +(15 - BIT) prepare set operation
		EXTEND
		ROR	LCHAN		# SET BIT using rotate operation
COMFLAG		INDEX	ITEMP1		# Index to correct flagword
		TS	FLAGWRD0	# Store modified flagword
		LXCH	ITEMP3		# Restore L register
		RELINT			# Release interrupt inhibit
		TC	L		# Return to caller


DOWNFLAG	CA	Q		# Save return address
		TC	DEBIT		# Decode flag address and bit position
		MASK	L		# RESET BIT using mask operation
		TCF	COMFLAG		# Common completion path

; ============================================================================
; DEBIT - FLAG ADDRESS DECODER
;
; Helper subroutine that decodes a flag name address into flagword number
; and bit position. Uses division by 15 to separate flagword index from
; bit position within that word.
;
; Input:  Q register contains return address with flag ADRES following
; Output: ITEMP1 = flagword index, ITEMP2 = bit position
;         L register = current flagword state
;         A register = bit mask for operation
; ============================================================================

DEBIT		AD	ONE		# GET DE BITS (address following TC)
		INHINT			# Begin critical section
		TS	ITEMP3		# Save adjusted return address
		CA	LOW4		# DEC15 (constant 15)
		TS	ITEMP1		# Divisor for flagword computation
		INDEX	ITEMP3		# Index to flag address
		CA	0 -1		# Retrieve ADRES of named flag
		TS	L		# Save flag address
		CA	ZERO		# Clear A for division
# Page 1487
		EXTEND
		DV	ITEMP1		# A = FLAGWRD index, L = (15 - BIT position)
		DXCH	ITEMP1		# Store results (A to ITEMP1, L to ITEMP2)
		INDEX	ITEMP1		# Index to computed flagword
		CA	FLAGWRD0	# Load current flagword state
		TS	L		# CURRENT STATE saved in L
		INDEX	ITEMP2		# Index by bit position
		CS	BIT15		# -(15 - BIT) for bit mask generation
		TC	Q		# Return to UPFLAG or DOWNFLAG

# Page 1488
# DELAYJOB- A GENERAL ROUTINE TO DELAY A JOB A SPECIFIC AMOUNT OF TIME BEFORE PICKING UP AGAIN.
#
# ENTRANCE REQUIREMENTS...
#		CAF	DT		# DELAY JOB FOR DT CENTISECS
#		TC	BANKCALL
#		CADR	DELAYJOB

; ============================================================================
; DELAYJOB - JOB DELAY SERVICE ROUTINE
;
; Suspends current job execution for a specified time period measured in
; centiseconds (1/100 second). During Apollo 11, mission programs used this
; to pace display updates, coordinate timing between guidance phases, and
; implement programmed delays in burn sequences.
;
; The routine allocates a delay slot, suspends the calling job via JOBSLEEP,
; schedules a WAITLIST task to wake the job, and returns control after the
; delay expires. This allows other jobs to execute during the delay period,
; maximizing AGC processor utilization.
;
; Calling sequence:
;   CAF   DT         (delay time in centiseconds)
;   TC    BANKCALL
;   CADR  DELAYJOB
;
; Returns after DT centiseconds to instruction following CADR.
; Aborts with alarm 1104 if no delay slots available (system overload).
; ============================================================================

		BANK	06
		SETLOC	DLAYJOB
		BANK

# THIS MUST REMAIN IN BANK 0 ****************************************

		COUNT	00/DELAY

DELAYJOB	INHINT			# Begin critical section
		TS	Q		# STORE DELAY DT in Q for WAITLIST

		CAF	DELAYNUM	# Number of available delay slots
DELLOOP		TS	RUPTREG1	# Save current slot index
		INDEX	A		# Index to delay location
		CA	DELAYLOC	# IS THIS DELAYLOC AVAILABLE
		EXTEND
		BZF	OK2DELAY	# YES (zero means available)

		CCS	RUPTREG1	# NO, TRY NEXT DELAYLOC
		TCF	DELLOOP		# Loop through remaining slots

		TC	BAILOUT		# NO AVAILABLE LOCS - system overload
		OCT	1104		# Abort code 1104

OK2DELAY	CA	TCSLEEP		# SET WAITLIST IMMEDIATE RETURN address
		TS	WAITEXIT	# Configure sleep exit point

		CA	FBANK		# Current bank number
		AD	RUPTREG1	# Combine with slot index
		TS	L		# STORE BBANK FOR TASK CALL

		CAF	WAKECAD		# STORE CADR FOR TASK CALL (waker address)
		TCF	DLY2 -1		# DLY is in WAITLIST routine - schedule wake

TCGETCAD	TC	MAKECADR	# GET CALLERS FCADR (return address)

		INDEX	RUPTREG1	# Index to allocated slot
		TS	DELAYLOC	# SAVE DELAY CADRS for wakeup

		TC	JOBSLEEP	# Suspend this job until timer expires

; ============================================================================
; WAKER - DELAY COMPLETION TASK
;
; WAITLIST task scheduled by DELAYJOB to wake the suspended job after the
; requested delay time expires. Clears the delay slot and resumes job
; execution at the point following the original DELAYJOB call.
; ============================================================================

WAKER		CAF	ZERO		# Zero to clear slot
		INDEX	BBANK		# Index by bank number
		XCH	DELAYLOC	# MAKE DELAYLOC AVAILABLE (get return CADR)
# Page 1489
		TC	JOBWAKE		# Wake the suspended job

		TC	TASKOVER	# End this WAITLIST task

TCSLEEP		GENADR	TCGETCAD -2	# Sleep entry point address
WAKECAD		GENADR	WAKER		# Waker task address

# Page 1490
# GENTRAN, A BLOCK TRANSFER ROUTINE.
#
# WRITTEN BY D. EYLES
# MOD 1 BY KERNAN				UTILITYM REV 17 11/18/67
#
# MOD 2 BY SCHULENBERG (REMOVE RELINT)	SKIPPER REV 4 2/28/68
#
#	THIS ROUTINE IS USEFULL FOR TRANSFERING N CONSECUTIVE ERASABLE OR FIXED QUANTITIES TO SOME OTHER N
# CONSECUTIVE ERASABLE LOCATIONS.  IF BOTH BLOCKS OF DATA ARE IN SWITCHABLE EBANKS, THEY MUST BE IN THE SAME ONE.
#
#	GENTRAN IS CALLABLE IN A JOB AS WELL AS A RUPT.  THE CALLING SEQUENCE IS:
#
#					I	CA	N-1		# # OF QUANTITIES MINUS ONE.
#					I +1	TC	GENTRAN		# IN FIXED-FIXED.
#					I +2	ADRES	L		# STARTING ADRES OF DATA TO BE MOVED.
#					I +3	ADRES	M		# STARTING ADRES OF DUPLICATION BLOCK.
#					I +4				# RETURNS HERE.
#
#	GENTRAN TAKES 25 MCT'S (300 MICROSECONDS) PER ITEM + 5 MCT'S (60 MICS) FOR ENTERING AND EXITING.
#
#	A, L AND ITEMP1 ARE NOT PRESERVED.

; ============================================================================
; GENTRAN - GENERAL BLOCK TRANSFER ROUTINE
;
; Copies N consecutive memory locations from source address to destination
; address. Written by Don Eyles, this efficient routine served Apollo 11
; programs during state vector transfers, display buffer updates, and data
; structure copying operations throughout the mission.
;
; The routine operates in a tight loop, transferring one word per iteration
; from high addresses to low addresses (backward copy). This prevents data
; corruption when source and destination ranges overlap.
;
; Performance: 300 microseconds per word transferred, plus 60 microseconds
; overhead for entry/exit. Callable from both jobs and interrupts.
;
; Calling sequence:
;   CA    N-1        (number of words minus one)
;   TC    GENTRAN
;   ADRES L          (starting address of source data)
;   ADRES M          (starting address of destination)
;   (returns here after transfer complete)
;
; Constraint: If both blocks in switchable EBANKS, must be in same bank.
; Destroys: A register, L register, ITEMP1
; ============================================================================

		BLOCK	02
		SETLOC	FFTAG4
		BANK

		EBANK=	ITEMP1

		COUNT*	$$/TRAN

GENTRAN		INHINT			# Begin critical section (prevent interrupts)
		TS	ITEMP1		# SAVE N-1 (loop counter for word count)
		INDEX	Q		# C(Q) = ADRES L (source start address)
		AD	0		# ADRES (L + N - 1) - compute end of source
		INDEX	A		# Index to last source location
		CA	0		# C(ABOVE) - load data word from source
		TS	L		# SAVE DATA in L register temporarily
		CA	ITEMP1		# Restore loop counter
		INDEX	Q		# Index by return address
		AD	1		# ADRES (M + N - 1) - compute end of destination
		INDEX	A		# Index to last destination location
		LXCH	0		# STUFF IT - exchange L with memory (store data)
		CCS	ITEMP1		# LOOP UNTIL N-1 = 0 (decrement and test counter)
		TCF	GENTRAN +1	# Continue loop (counter was positive)
		TCF	Q+2		# RETURN TO CALLER (all words transferred)

# Page 1491
# B5OFF		ZERO BIT 5 OF EXTVBACT, WHICH IS SET BY TESTXACT.
#
#		MAY BE USED AS NEEDED BY ANY EXTENDED VERB WHICH HAS DONE TESTXACT

; ============================================================================
; B5OFF - CLEAR EXTENDED VERB ACTIVITY FLAG BIT
;
; Service routine to clear bit 5 of EXTVBACT (extended verb activity flag
; word). Extended verbs use this bit (set by TESTXACT) to indicate activity
; status. When verb processing completes, this routine clears the flag to
; signal completion and terminate the current job.
;
; During Apollo 11, extended verbs controlled critical crew operations such
; as IMU alignment, orbit parameter displays, and system status queries.
; This cleanup routine ensured proper verb completion tracking.
;
; Calling: TC BANKCALL / CADR B5OFF (from extended verb completion code)
; Returns: Does not return (terminates job via ENDOFJOB)
; ============================================================================

		COUNT*	$$/EXTVB

B5OFF		CS	BIT5		# Complement of bit 5 for clearing
		MASK	EXTVBACT	# Clear bit 5 in EXTVBACT flag word
		TS	EXTVBACT	# Store updated flag word
		TC	ENDOFJOB	# Terminate this job (does not return)

# Page 1492
# SUBROUTINES TO TURN OFF AND TURN ON TRACKER FAIL LIGHT.

; ============================================================================
; TRFAILOF - TURN OFF TRACKER FAIL LIGHT
; TRFAILON - TURN ON TRACKER FAIL LIGHT
;
; These paired service routines control the TRACKER indicator light on the
; DSKY display panel. The tracker fail light warns the crew of automatic
; star tracker failures during optical navigation operations.
;
; During Apollo 11's translunar coast and lunar orbit phases, the sextant
; and telescope could automatically track stars for navigation updates.
; When the automatic tracker lost lock on a star, these routines illuminated
; or extinguished the warning light to inform the crew (Collins in CM,
; Armstrong/Aldrin in LM before separation).
;
; TRFAILOF implementation notes:
; - Manipulates DSPTAB +11D (display table word controlling indicator lights)
; - Clears tracker fail bit (OCT40200) while preserving other light states
; - Additionally checks OPTMODES bit 7 (OCDU fail flag) to ensure that if
;   both OCDU fail and tracker fail were lit, OCDU fail remains visible
;   after tracker fail clears (prevents masking OCDU failure indication)
;
; TRFAILON implementation notes:
; - Sets tracker fail bit (OCT40200) in display table word
; - Preserves other indicator light states during bit manipulation
;
; Both routines:
; - Execute in interrupt-inhibited section (INHINT/RELINT) for atomic updates
; - Prevent display corruption from concurrent DSKY update interrupts
; - Return via Q register to caller
;
; Calling: TC BANKCALL / CADR TRFAILOF (or TRFAILON)
; Returns: Via Q register after light state updated
; ============================================================================

TRFAILOF	INHINT			# Begin critical section (inhibit interrupts)
		CS	OCT40200	# TURN OFF TRACKER LIGHT - complement of tracker bit
		MASK	DSPTAB +11D	# Clear tracker fail bit in display table word
		AD	BIT15		# Add sign bit for proper display table format
		TS	DSPTAB +11D	# Store updated display control word
		CS	OPTMODES	# TO INSURE THAT OCDU FAIL WILL GO ON
		MASK	BIT7		# AGAIN IF IT WAS ON IN ADDITION TO
		ADS	OPTMODES	# TRACKER FAIL - preserve OCDU fail indication

REQ		RELINT			# Release interrupt inhibit (critical section end)
		TC	Q		# Return to caller

TRFAILON	INHINT			# Begin critical section (inhibit interrupts)
		CS	DSPTAB	+11D	# TURN ON - load complement of display table word
		MASK	OCT40200	# Isolate tracker fail bit position
		ADS	DSPTAB +11D	# Set tracker fail bit in display table (add to set)
		TCF	REQ		# Jump to release interrupts and return
