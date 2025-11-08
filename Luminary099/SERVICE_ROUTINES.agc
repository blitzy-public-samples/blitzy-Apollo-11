# Copyright:	Public domain.
# Filename:	SERVICE_ROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1374-1380
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Luminary131/ file of the same
#				name, using Luminary099 page images.
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
; FILE: SERVICE_ROUTINES.agc
; MODULE: Service Utilities
; MISSION PHASE: All phases (utility support functions)
;
; TL;DR: Provides essential utility service functions used throughout the
;        Lunar Module AGC programs. Includes flag manipulation routines for
;        setting/clearing status bits, job delay scheduling for time-based
;        operations, block memory transfer utilities, and display interface
;        support functions. These are foundational operations called by
;        mission programs, guidance algorithms, and crew interface routines.
;
; COMMENT-ONLY READERS: These utilities are the "plumbing" that supports
;        mission-critical operations - think of them as common tools used
;        throughout the spacecraft's computer to manage timing, memory, and
;        status tracking.
; CODE-ALONG READERS: Study the interrupt-safe flag manipulation, waitlist
;        integration for job delays, and efficient block transfer algorithm.
;        Note the careful register preservation and timing constraints.
; ============================================================================

# Page 1374
		BANK	10
		SETLOC	DISPLAYS
		BANK
		COUNT*	$$/DSPLA

; ============================================================================
; UPENT2 / DOWNENT2 - FLAGWORD4 Bit Manipulation Routines
;
; These routines provide interrupt-safe setting and clearing of multiple bits
; in FLAGWRD4, used for display interface control. The spacecraft computer
; uses flags to track operational states and coordinate between programs.
; ============================================================================

UPENT2		INHINT			; Disable interrupts for atomic operation
		MASK	OCT77770	; Extract bits to set from A register
		TS	L		; Save bit pattern in L register
		CS	FLAGWRD4	; Read complement of current FLAGWRD4
		MASK	L		; Combine with bits to set
		ADS	FLAGWRD4	; Add to FLAGWRD4 (sets specified bits)
; JOIN point allows both routines to share common exit code.
JOIN		RELINT			; Re-enable interrupts
		TCF	Q+1		; Return to caller (skip ADRES operand)

; DOWNENT2 clears (resets) bits in FLAGWRD4 instead of setting them.
DOWNENT2	INHINT			; Disable interrupts for atomic operation
		MASK	OCT77770	; Extract bits to clear
		COM			; Complement gives clear mask
		MASK	FLAGWRD4	; Clear specified bits in FLAGWRD4
		TS	FLAGWRD4	; Store result
		TCF	JOIN		; Common exit path

OCT7		EQUALS	SEVEN

# Page 1375
#     UPFLAG AND DOWNFLAG ARE ENTIRELY GENERAL FLAG SETTING AND CLEARING SUBROUTINES.   USING THEM, WHETHER OR
# NOT IN INTERRUPT, ONE MAY SET OR CLEAR ANY SINGLE, NAMED BIT IN ANY ERASABLE REGISTER, SUBJECT OF COURSE TO
# EBANK SETTING.   A "NAMED" BIT, AS THE WORD IS USED HERE, IS ANY BIT WITH A NAME FORMALLY ASSIGNED BY THE YUL
# ASSEMBLER.
#
#     AT PRESENT THE ONLY NAMED BITS ARE THOSE IN THE FLAGWORDS.   ASSEMBLER CHANGES WILL MAKE IT POSSIBLE TO
# NAME ANY BIT IN ERASABLE MEMORY.
#
#     CALLING SEQUENCES ARE AS FOLLOWS :-
#		TC	UPFLAG			TC	DOWNFLAG
#		ADRES	NAME OF FLAG		ADRES	NAME OF FLAG
#
#     RETURN IS TO THE LOCATION FOLLOWING THE "ADRES" ABOUT .58 MS AFTER THE "TC".
#     UPON RETURN A CONTAINS THE CURRENT FLAGWRD SETTING.

; ============================================================================
; TRANSITION: From Display-Specific Flags to General Flag Operations
;
; The AGC uses "flags" (status bits) extensively to coordinate operations
; between different programs and track spacecraft states. The following
; routines provide a safe, general-purpose way to set or clear any named
; flag bit in the system, ensuring proper interrupt handling to prevent
; race conditions. During lunar operations, these routines were called
; thousands of times to manage program states.
; ============================================================================

		BLOCK	02
		SETLOC	FFTAG1
		BANK
		COUNT*	$$/FLAG

; UPFLAG sets a named flag bit to 1 (true state).
; Called by programs to signal state changes or enable features.
UPFLAG		CA	Q		; Get return address from Q register
		TC	DEBIT		; Compute flagword address and bit position
		COM			; Complement yields +(15 - BIT) for setting
		EXTEND
		ROR	LCHAN		; Rotate to set the bit
; Common flag update path used by both UPFLAG and DOWNFLAG.
COMFLAG		INDEX	ITEMP1		; Indexed addressing to correct FLAGWRD
		TS	FLAGWRD0	; Store updated flagword
		LXCH	ITEMP3		; Restore L register (contains return address)
		RELINT			; Re-enable interrupts (safe to do so now)
		TC	L		; Return to caller

; DOWNFLAG clears a named flag bit to 0 (false state).
; Called by programs to signal completion or disable features.
DOWNFLAG	CA	Q		; Get return address from Q register
		TC	DEBIT		; Compute flagword address and bit position
		MASK	L		; Mask to reset (clear) the bit
		TCF	COMFLAG		; Use common update path

; DEBIT is a helper subroutine that decodes the ADRES operand to determine
; which flagword and which bit position within that word. It uses division
; to compute: Flagword number = address / 15, Bit position = address MOD 15.
DEBIT		AD	ONE		; Adjust address (+1)
		INHINT			; Disable interrupts during computation
		TS	ITEMP3		; Save adjusted address
		CA	LOW4		; Load 15 decimal (DEC15) for division
		TS	ITEMP1		; Divisor in ITEMP1
		INDEX	ITEMP3		; Indexed load of flag address
		CA	0 -1		; Get ADRES operand from caller
		TS	L		; Place in L for division
		CA	ZERO		; Clear A for extended precision division
# Page 1376
		EXTEND			; Enter extended instruction mode
		DV	ITEMP1		; Divide by 15: A = FLAGWRD#, L = bit position
		DXCH	ITEMP1		; Store quotient and remainder in ITEMP1/ITEMP2
		INDEX	ITEMP1		; Index to correct flagword
		CA	FLAGWRD0	; Load current flagword value
		TS	L		; Save current state in L
		INDEX	ITEMP2		; Index to bit position
		CS	BIT15		; Load complement of bit mask: -(15 - BIT)
		TC	Q		; Return to UPFLAG or DOWNFLAG caller

# Page 1377
# DELAYJOB- A GENERAL ROUTINE TO DELAY A JOB A SPECIFIC AMOUNT OF TIME BEFORE PICKING UP AGAIN.
#
# ENTRANCE REQUIREMENTS...
#		CAF	DT		# DELAY JOB FOR DT CENTISECS
#		TC	BANKCALL
#		CADR	DELAYJOB

; ============================================================================
; TRANSITION: From Basic Flag Operations to Time-Based Job Scheduling
;
; Many spacecraft operations require precise timing - waiting for engine
; warm-up, delaying telemetry transmission, or coordinating sequential
; operations. DELAYJOB allows a running program to suspend itself for a
; specified time period (in centiseconds, 1/100ths of a second), then
; automatically resume execution. During the lunar landing, this routine
; managed timing for radar sampling, display updates, and system monitoring.
; ============================================================================

		BANK	06
		SETLOC	DLAYJOB
		BANK

; THIS MUST REMAIN IN BANK 0 TO BE ACCESSIBLE FROM ALL PROGRAMS *************

		COUNT*	$$/DELAY
; DELAYJOB suspends the calling job for a specified time interval.
; The job is placed on the WAITLIST and will automatically resume after
; the delay expires. Time is specified in centiseconds (0.01 seconds).
; Example: 500 centiseconds = 5 seconds delay.
DELAYJOB	INHINT			; Disable interrupts for atomic operation
		TS	Q		; Store delay time (DT) in Q register
		CAF	DELAYNUM	; Load number of available delay slots
; Search for an available DELAYLOC slot to store job resume information.
DELLOOP		TS	RUPTREG1	; Store current slot index
		INDEX	A		; Indexed load
		CA	DELAYLOC	; Is this delay slot free? (zero if free)
		EXTEND
		BZF	OK2DELAY	; Branch if zero (slot available)

		CCS	RUPTREG1	; Decrement slot counter and test
		TCF	DELLOOP		; Try next slot

; No available delay slots - this is an error condition.
		DXCH	BUF2		; Save state for abort
		TC	BAILOUT1	; Abort with error code
		OCT	1104		; Delay slot overflow error code

; Found an available slot - set up delayed job wake-up.
OK2DELAY	CA	TCSLEEP		; Load JOBSLEEP return address
		TS	WAITEXIT	; Set up immediate return from WAITLIST

		CA	FBANK		; Get caller's bank number
		AD	RUPTREG1	; Combine with slot index for BBANK
		TS	L		; Store in L register

		CAF	WAKECAD		; Load CADR of WAKER task
		TCF	DLY2 -1		; Enter WAITLIST delay routine (DLY in WAITLIST.agc)

; TCGETCAD is called by WAITLIST to get the caller's full CADR.
TCGETCAD	TC	MAKECADR	; Construct complete CADR from caller info

		INDEX	RUPTREG1	; Index to delay slot
		TS	DELAYLOC	; Save resume CADR in delay slot

		TC	JOBSLEEP	; Put this job to sleep until timer expires

; WAKER is the task that executes when the delay time expires.
; It wakes up the sleeping job so it can resume execution.
WAKER		CAF	ZERO		; Load zero to clear slot
		INDEX	BBANK		; Index using saved bank
		XCH	DELAYLOC	; Clear delay slot (make available) and get CADR
# Page 1378
		TC	JOBWAKE		; Wake up the sleeping job

		TC	TASKOVER	; This task is complete

; Address constants for DELAYJOB internal operations.
TCSLEEP		GENADR	TCGETCAD -2
WAKECAD		GENADR	WAKER

# Page 1379
# GENTRAN, A BLOCK TRANSFER ROUTINE.
# WRITTEN BY D. EYLES
# MOD 1 BY KERNAN						UTILITYM REV 17 11/18/67
# MOD 2 BY SCHULENBERG (REMOVE RELINT)   SKIPPER REV 4 2/28/68
#
# 	   THIS ROUTINE IS USEFULL FOR TRANSFERING N CONSECUTIVE ERASABLE OR FIXED QUANTITIES TO SOME OTHER N
# CONSECUTIVE ERASABLE LOCATIONS.  IF BOTH BLOCKS OF DATA ARE IN SWITCHABLE EBANKS, THEY MUST BE IN THE SAME ONE.
#
# 	   GENTRAN IS CALLABLE IN A JOB AS WELL AS A RUPT.  THE CALLING SEQUENCE IS:
#	I	CA	N-1		# # OF QUANTITIES MINUS ONE.
#	I +1	TC	GENTRAN		# IN FIXED-FIXED.
#	I +2	ADRES	L		# STARTING ADRES OF DATA TO BE MOVED.
#	I +3	ADRES	M		# STARTING ADRES OF DUPLICATION BLOCK.
#	I +4				# RETURNS HERE.
#
# 	   GENTRAN TAKES 25 MCT'S (300 MICROSECONDS) PER ITEM + 5 MCT'S (60 MICS) FOR ENTERING AND EXITING.
# 	   A, L, AND ITEMP1 ARE NOT PRESERVED.

; ============================================================================
; TRANSITION: From Time-Based Operations to Memory Transfer Utilities
;
; While flags track individual states and delays manage timing, block memory
; transfers handle bulk data movement. During lunar operations, GENTRAN
; efficiently copied navigation state vectors, display data buffers, and
; guidance parameters between memory locations - essential for coordinating
; between different spacecraft computer programs.
; ============================================================================

		BLOCK	02
		SETLOC	FFTAG4
		BANK

		EBANK=	ITEMP1

		COUNT*	$$/TRAN

; GENTRAN performs high-speed block memory transfers of N consecutive words.
; Optimized loop processes data from high addresses to low (reverse order)
; for efficient indexed addressing. Used throughout AGC for copying state
; vectors, saving/restoring register sets, and duplicating data structures.
; Timing: 300 microseconds per word + 60 microseconds overhead.
GENTRAN		INHINT			; Disable interrupts during transfer
		TS	ITEMP1		; Save N-1 (count minus one) in ITEMP1
		INDEX	Q		; Q contains return address (I+1)
		AD	0		; Add source start address: ADRES(L + N - 1)
		INDEX	A		; Index to source location
		CA	0		; Load data word from source
		TS	L		; Temporarily store in L register
		CA	ITEMP1		; Restore counter
		INDEX	Q		; Q+1 points to destination address
		AD	1		; Add destination start address: ADRES(M + N - 1)
		INDEX	A		; Index to destination location
		LXCH	0		; Store data (L) to destination, load destination
		CCS	ITEMP1		; Decrement counter and test
		TCF	GENTRAN +1	; Loop if more words remain
		TCF	Q+2		; Return to caller (skip ADRES operands)

# Page 1380
# B5OFF   ZERO BIT 5 OF EXTVBACT, WHICH IS SET BY TESTXACT.
# 	   MAY BE USED AS NEEDED BY ANY EXTENDED VERB WHICH HAS DONE TESTXACT

; ============================================================================
; B5OFF - Extended Verb Activity Flag Cleanup
;
; Extended verbs (V37 and higher) use the EXTVBACT flag word to track their
; operational state. Bit 5 of EXTVBACT is set by the TESTXACT routine when
; checking for extended verb conflicts. B5OFF clears this bit after the
; extended verb completes, making the system ready for the next extended
; verb operation. This cleanup prevents false conflict detection on
; subsequent DSKY verb entries.
; ============================================================================

		COUNT*	$$/EXTVB

; B5OFF clears bit 5 of EXTVBACT extended verb activity flag.
; Called by extended verb routines after completing operations that set
; TESTXACT state. Ensures clean flag state for next extended verb sequence.
B5OFF		CS	BIT5		; Load complement of bit 5 mask
		MASK	EXTVBACT	; Clear bit 5, preserve other bits
		TS	EXTVBACT	; Update extended verb activity flags
		TC	ENDOFJOB	; Terminate this job and return to scheduler
