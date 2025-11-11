; ============================================================================
; FILE: AGC_BLOCK_TWO_SELF_CHECK.agc
; MODULE: AGC Block II Self-Check Diagnostics
; MISSION PHASE: all phases (continuous background diagnostic)
;
; TL;DR: Implements comprehensive self-test diagnostics for the AGC Block II
;        computer running continuously as a zero-priority background job.
;        Tests erasable memory (RAM), fixed memory (core rope ROM), counter
;        registers, and shift/cycle registers. Detects hardware failures and
;        reports errors via the alarm system, ensuring the LM guidance
;        computer remains healthy throughout the mission from Earth orbit
;        through lunar landing and return.
;
; COMMENT-ONLY READERS: This is the computer's built-in health monitoring
;        system, constantly checking itself to catch hardware failures before
;        they affect the mission. Think of it as the AGC's immune system.
;
; CODE-ALONG READERS: Study the clever test algorithms for memory validation,
;        the use of checksums for fixed memory verification, and the priority
;        structure allowing self-check to run without interfering with
;        critical guidance programs. Note the different SMODE options that
;        control test coverage and error handling behavior.
; ============================================================================

# Copyright:	Public domain.
# Filename:	AGC_BLOCK_TWO_SELF_CHECK.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1284-1293
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
; SELF-CHECK PROGRAM OVERVIEW
;
; The AGC continuously monitors its own hardware health while Armstrong and
; Aldrin prepare for lunar landing. This self-check program runs silently
; in the background, testing memory and registers without interrupting the
; critical guidance computations that will guide Eagle to the surface.
;
; If a hardware fault is detected, the computer immediately alerts the crew
; via the DSKY alarm light and stores diagnostic information for troubleshooting.
; ============================================================================

# Page 1284
# PROGRAM DESCRIPTION				DATE:  20 DECEMBER 1967
# PROGRAM NAME -- SELF-CHECK			LOG SECTION:  AGC BLOCK TWO SELF-CHECK
# MOD NO -- 1					ASSEMBLY SUBROUTINE UTILITYM REV 25
# MOD BY -- GAUNTT
#
#
# FUNCTIONAL DESCRIPTION
#
#	PROGRAM HAS TWO MAIN PARTS.  THE FIRST IS SELF-CHECK WHICH RUNS AS A ZERO PRIORITY JOB WITH NO CORE SET, AS
#	PART OF THE BACK-UP IDLE LOOP.  THE SECOND IS SHOW-BANKSUM WHICH RUNS AS A REGULAR EXECUTIVE JOB WITH ITS OWN
# 	STARTING VERB.
#	THE PURPOSE OF SELF-CHECK IS TO CHECK OUT VARIOUS PARTS OF THE COMPUTER AS OUTLINED BELOW IN THE OPTIONS.
#	THE PURPOSE OF SHOW-BANKSUM IS TO DISPLAY THE SUM OF EACH BANK, ONE AT A TIME.
#	IN ALL THERE ARE 7 POSSIBLE OPTIONS IN THIS BLOCK II VERSION OF SELF-CHECK.  MORE DETAIL DESCRIPTION MAY BE
#	FOUND IN E-2065 BLOCK II AGC SELF-CHECK AND SHOW BANKSUM BY EDWIN D. SMALLY DECEMBER 1966, AND ADDENDA 2 AND 3.
#	THE DIFFERENT OPTIONS ARE CONTROLLED BY PUTTING DIFFERENT NUMBERS IN THE SMODE REGISTER (NOUN 27).  BELOW IS
# 	A DESCRIPTION OF WHAT PARTS OF THE COMPUTER THAT ARE CHECKED BY THE OPTIONS, AND THE CORRESPONDING NUMBER, IN
#	OCTAL, TO LOAD INTO SMODE.
#		+-4		ERASABLE MEMORY
#		+-5		FIXED MEMORY
#		+-1,2,3,6,7,10	EVERYTHING IN OPTIONS 4 AND 5.
#		-0		SAME AS +-10 UNTIL AN ERROR IS DETECTED.
#		+0		NO CHECK, PUTS COMPUTER INTO BACKUP IDLE LOOP.
#
#
# WARNINGS
#
#	USE OF E MEMORY RESERVED FOR SELF-CHECK (EVEN IN IDLE LOOP) AS TEMP STORAGE BY OTHER PROGRAMS IS DANGEROUS.
#	SMODE SET GREATER THAN OCT 10 PUTS COMPUTER INTO BACKUP IDLE LOOP.
#
#
# CALLING SEQUENCE
#
#	TO CALL SELF-CHECK KEY IN
#		V 21 N 27 E	OPTION NUMBER E
#	TO CALL SHOW-BANKSUM KEY IN
#		V 91 E		DISPLAYS FIRST BANK
#		V 33 E		PROCEED, DISPLAYS NEXT BANK
#
#
# EXIT MODES, NORMAL AND ALARM
#	SELF-CHECK NORMALLY CONTINUES INDEFINITELY UNLESS THERE IS AN ERROR DETECTED.  IF SO + OPTION NUMBERS PUT
#	COMPUTER INTO BACKUP IDLE LOOP, - OPTION NUMBERS RESTART THE OPTION.
#
#	THE -0 OPTION PROCEEDS FROM THE LINE FOLLOWING THE LINE WHERE THE ERROR WAS DETECTED.
#	SHOW-BANKSUM PROCEEDS UNTIL A TERMINATE IS KEYED IN (V 34 E).  THE COMPUTER IS PUT INTO THE BACKUP IDLE LOOP.
#
#
# OUTPUT
# Page 1285
#	SELF-CHECK UPON DETECTING AN ERROR LOADS THE SELF-CHECK ALARM CONSTANT (01102) INTO THE FAILREG SET AND
#	TURNS ON THE ALARM LIGHT.  THE OPERATOR MAY THEN DISPLAY THE THREE FAILREGS BY KEYING IN V 05 N 09 E.  FOR FURTHER
# 	INFORMATION HE MAY KEY IN V 05 N 08 E, THE DSKY DISPLAY IN R1 WILL BE ADDRESS+1 OF WHERE THE ERROR WAS DETECTED,
#	IN R2 THE BBCON OF SELF-CHECK, AND IN R3 THE TOTAL NUMBER OF ERRORS DETECTED BY SELF-CHECK SINCE THE LAST MAN
#	INITIATED FRESH START (SLAP1).
#	SHOW-BANKSUM STARTING WITH BANK 0 DISPLAYS IN R1 THE BANK SUM (A +-NUMBER EQUAL TO THE BANK NUMBER), IN R2
#	THE BANK NUMBER, AND IN R3 THE BUGGER WORD.
#
#
# ERASABLE INITIALIZATION REQUIRED
#
#	ACCOMPLISHED BY FRESH START
#		SMODE SET TO +0
#
#
# DEBRIS
#
#	ALL EXITS FROM THE CHECK OF ERASABLE (ERASCHK) RESTORE ORIGINAL CONTENTS TO REGISTERS UNDER CHECK.
#	EXCEPTION IS A RESTART.  RESTART THAT OCCURS DURING ERASCHK RESTORES ERASABLE, UNLESS THERE IS EVIDENCE TO DOUBT
#	E MEMORY, IN WHICH CASE PROGRAM THEN DOES A FRESH START (DOFSTART).


; ============================================================================
; SELF-CHECK CONSTANTS AND MEMORY ALLOCATION
;
; These carefully chosen test constants enable the self-check routines to
; validate different aspects of the AGC hardware. Each constant serves as
; a known value for comparison or a pattern for detecting stuck bits and
; memory failures.
; ============================================================================

		BANK	25
		SETLOC	SELFCHEC
		BANK

		COUNT*	$$/SELF

; Self-check bit masks (SBIT1 through SBIT15)
; Used to isolate individual bit positions during hardware tests.
; Each corresponds to one bit position in the 15-bit AGC word.

SBIT1		EQUALS	BIT1
SBIT2		EQUALS	BIT2
SBIT3		EQUALS	BIT3
SBIT4		EQUALS	BIT4
SBIT5		EQUALS	BIT5
SBIT6		EQUALS	BIT6
SBIT7		EQUALS	BIT7
SBIT8		EQUALS	BIT8
SBIT9		EQUALS	BIT9
SBIT10		EQUALS	BIT10
SBIT11		EQUALS	BIT11
SBIT12		EQUALS	BIT12
SBIT13		EQUALS	BIT13
SBIT14		EQUALS	BIT14
SBIT15		EQUALS	BIT15

; Positive test constants (S+ZERO through S+7)
; Standard small integers used throughout self-check algorithms.

S+ZERO		EQUALS	ZERO
S+1		EQUALS	BIT1
S+2		EQUALS	BIT2
S+3		EQUALS	THREE
S+4		EQUALS	FOUR
S+5		EQUALS	FIVE
S+6		EQUALS	SIX
# Page 1286
S+7		EQUALS	SEVEN

; Bit field masks and test patterns
; S8BITS: Lower 8 bits (octal 00377) for byte-level testing
; S10BITS: Lower 10 bits (octal 01777) for erasable address range
; S13BITS: All 13 bits (octal 17777) for testing address fields

S8BITS		EQUALS	LOW8		# 00377
CNTRCON		=	OCT50		# USED IN CNTRCHK
ERASCON1	OCTAL	00061		# USED IN ERASCHK
ERASCON2	OCTAL	01373		# USED IN ERASCHK
ERASCON6	=	OCT1400		# USED IN ERASCHK
ERASCON3	OCTAL	01461		# USED IN ERASCHK
ERASCON4	OCTAL	01773		# USED IN ERASCHK
S10BITS		EQUALS	LOW10		# 01777, USED IN ERASCHK

; Fixed memory (core rope) test constants
; SBNK03: Bank 3 code (06000) for bank switching validation
; -MAXADRS: Maximum address constant for range checking
; SUPRCON: Super bank constant (60017) for extended addressing

SBNK03		EQUALS	PRIO6		# 06000, USED IN ROPECHK
-MAXADRS	=	HI5		# FOR ROPECHK
SIXTY		OCTAL	00060
SUPRCON		OCTAL	60017		# USED IN ROPECHK
S13BITS		OCTAL	17777

; Cycle/shift register test patterns
; CONC+S1, CONC+S2: Concatenated shift patterns that exercise all bit
; transitions when cycled through the accumulator and L register.

CONC+S1		OCTAL	25252		# USED IN CYCLSHFT
CONC+S2		OCTAL	52400		# USED IN CYCLSHFT
ERASCON5	OCTAL	76777

; Negative test constants (S-1 through S-7)
; Used to test signed arithmetic and negative value handling.

S-7		=	OCT77770
S-4		EQUALS	NEG4
S-3		EQUALS	NEG3
S-2		EQUALS	NEG2
S-1		EQUALS	NEGONE
S-ZERO		EQUALS	NEG0

		EBANK=	LST1
ADRS1		ADRES	SKEEP1
SELFADRS	ADRES	SELFCHK		# SELFCHK RETURN ADDRESS.  SHOULD BE PUT
					# IN SELFRET WHEN GOING FROM SELFCHK TO
					# SHOWSUM AND PUT IN SKEEP1 WHEN GOING
					# FROM SHOWSUM TO SELF-CHECK.

; ============================================================================
; TRANSITION: From constants definition to error handling routines
;
; The self-check program now begins its operational logic. The PRERRORS routine
; handles failures detected during any diagnostic test. When a test fails, this
; code preserves the system state, records the failure location, increments the
; error counter, triggers the crew alarm, and determines whether to restart the
; test or enter the backup idle loop based on the SMODE setting.
; ============================================================================

; ERROR RESPONSE ROUTINE
; PRERRORS: Called when any self-check test detects a failure (erasable memory
;           corruption, fixed memory checksum mismatch, counter malfunction, etc.)
;
; COMMENT-ONLY READERS: When the guidance computer detects a hardware problem
; during self-test, this routine alerts the crew via the alarm light and DSKY
; display. The crew can then examine the FAILREG registers (V05 N09) to see
; error details including the memory address where the failure occurred.
;
; CODE-ALONG READERS: Error handling sequence: 1) Restore any modified erasable
; registers to original values (if ERESTORE flag set), 2) Save failure address
; from Q register, 3) Increment total error count (ERCOUNT), 4) Trigger alarm
; code 01102 via ALARM2 routine, 5) Check SMODE sign to determine response.
PRERRORS	CA	ERESTORE	# IS IT NECESSARY TO RESTORE ERASABLE
		EXTEND
		BZF	ERRORS		# NO
; If ERESTORE flag is set, the erasable memory check routine (ERASCHK) has
; modified two erasable registers under test. Before processing the error,
; restore these registers to their original values to prevent false failures.
		EXTEND
		DCA	SKEEP5		; Load original values from SKEEP5+6
		INDEX	SKEEP7		; SKEEP7 contains address of modified registers
		DXCH	0000		# RESTORE THE TWO ERASABLE REGISTERS
		CA	S+ZERO
		TS	ERESTORE	; Clear restoration flag (set to +0)

; ERRORS: Error processing begins here after any necessary register restoration
ERRORS		INHINT			; Inhibit interrupts during error recording
		CA	Q		; Q register holds return address = failure location
		TS	SFAIL		# SAVE Q FOR FAILURE LOCATION
		TS	ALMCADR		# FOR DISPLAY WITH BBANK AND ERCOUNT
; The crew can now key in V05 N08 E to display:
; R1 = ALMCADR (address+1 where error detected)
; R2 = BBANK (bank number where self-check resides)
; R3 = ERCOUNT (total errors since last fresh start)
		INCR	ERCOUNT		# KEEP TRACK OF NUMBER OF MALFUNCTIONS.

; Trigger self-check alarm on DSKY (alarm light illuminates, code 01102 displayed)
TCALARM2	TC	ALARM2
		OCT	01102		# SELF-CHECK MALFUNCTION INDICATOR

; Check SMODE register sign to determine error response strategy
		CCS	SMODE		; Test SMODE for positive/negative/zero
; If SMODE positive: Enter backup idle loop (halt self-check after error)
; If SMODE negative: Restart the test option that failed (continue self-check)
; If SMODE -0: Continue from line following error (special debug mode)
SIDLOOP		CA	S+ZERO		; Positive SMODE detected
		TS	SMODE		; Set SMODE to +0 (idle loop mode)
# Page 1287
		TC	SELFCHK		# GO TO IDLE LOOP
		TC	SFAIL		# CONTINUE WITH SELF-CHECK (negative SMODE)

; -1CHK: Utility routine to verify that accumulator contains -1 (octal 77777)
; Used throughout self-check to validate checksum results and arithmetic operations
; If A not equal to -1, branches to PRERRORS to report failure
-1CHK		CCS	A		; Test A: +, +0, -, -0
		TCF	PRERRORS	; A was positive, not -1, error detected
		TCF	PRERRORS	; A was +0, not -1, error detected
		CCS	A		; A was negative, test again
		TCF	PRERRORS	; A was between -1 and -0, not exactly -1
		TC	Q		; A equals -1 exactly, test passed, return

; ============================================================================
; SMODE DECODER - Test Option Selector
;
; SMODECHK: Main self-check control routine, called repeatedly from SELFCHK.
; Checks SMODE register to determine which diagnostic tests to run, periodically
; yields to WAITLIST to check for higher-priority jobs (via CHECKNJ), then
; dispatches to the appropriate test option (ERASCHK, ROPECHK, etc.)
;
; COMMENT-ONLY READERS: This routine ensures self-check runs as a low-priority
; background task, yielding control whenever the guidance computer has actual
; mission work to perform. The SMODE value controls testing intensity.
;
; CODE-ALONG READERS: SKEEP1 holds return address within current test routine.
; SCOUNT increments each pass through test loop. CHECKNJ calls POSTJUMP to
; allow WAITLIST servicing without losing self-check context.
; ============================================================================

SMODECHK	EXTEND
		QXCH	SKEEP1		; Save return address (within test routine)
		TC	CHECKNJ		# CHECK FOR NEW JOB
; Test SMODE register sign to determine action:
; Positive SMODE: No testing, enter idle loop
; +0: Backup idle loop only
; Negative SMODE: Execute test option(s)
; -0: Special debug mode, continue after error
		CCS	SMODE		; Test SMODE for +, +0, -, -0
		TC	SOPTIONS	; Positive, decode option number
		TC	SMODECHK +2	# TO BACKUP IDLE LOOP (+0 value)
		TC	SOPTIONS	; Negative, decode option number
		INCR	SCOUNT		; -0 special mode, increment pass counter
		TC	SKEEP1		# CONTINUE WITH SELF-CHECK

; SOPTIONS: Decodes SMODE option number and dispatches to appropriate test
; SMODE options (see program header for complete descriptions):
;   +-1,2,3,6,7,10: Full test (erasable + fixed memory)
;   +-4: Erasable memory test only
;   +-5: Fixed memory (rope core) checksum only
;   >10 (octal): Illegal, enters idle loop
SOPTIONS	AD	S-7		; Subtract 7 from SMODE (test for option > 10 octal)
		EXTEND
		BZMF	+2		# FOR OPTIONS BELOW NINE.
BNKOPTN		TC	SIDLOOP		# ILLEGAL OPTION.  GO TO IDLE LOOP.
		INCR	SCOUNT		# FOR OPTIONS BELOW NINE.
		AD	S+7		; Restore option number in A (0-7 range)

; Index into option dispatch table (options 1-10 octal)
		INDEX	A		; Indexed TC, uses A as offset
		TC	SOPTION1	; Jump to SOPTIONx based on SMODE value
; Option dispatch table:
SOPTION1	TC	SKEEP1		# WAS TC+TCF (option 1: full test)
SOPTION2	TC	SKEEP1		# WAS IN:OUT1 (option 2: full test)
SOPTION3	TC	SKEEP1		# WAS COUNTCHK (option 3: full test)
SOPTION4	TC	ERASCHK		# Option 4: Erasable memory test only
SOPTION5	TC	ROPECHK		# Option 5: Fixed memory (rope) checksum only
SOPTION6	TC	SKEEP1		# Option 6: Full test
SOPTION7	TC	SKEEP1		# Option 7: Full test
SOPTON10	TC	SKEEP1		# CONTINUE WITH SELF-CHECK (option 10: full test)

; CHECKNJ: Checks for higher-priority jobs in WAITLIST
; Called periodically by SMODECHK to ensure self-check yields to mission-critical
; tasks. Uses POSTJUMP to allow WAITLIST scheduler to preempt if necessary.
CHECKNJ		EXTEND
		QXCH	SELFRET		# SAVE RETURN ADDRESS WHILE TESTING NEWJOB
		TC	POSTJUMP	# TO SEE IF ANY JOBS HAVE BECOME ACTIVE.
		CADR	ADVAN		; Jump to ADVAN routine, returns here if no jobs

; SELFCHK: Main entry point for self-check from backup idle loop
; Called continuously when no mission-critical work is scheduled
SELFCHK		TC	SMODECHK	# ** CHARLEY, COME IN HERE (entry point comment)

; ============================================================================
; ERASCHK - Erasable Memory Test (RAM Diagnostic)
;
; Tests the 2K words of erasable memory (RAM) by writing test patterns,
; reading back values, and verifying correctness. Tests both unswitched
; erasable (bank 0, addresses 0000-1777 octal) and switchable erasable
; banks (E1-E7, addresses 1400-1777 per bank).
;
; COMMENT-ONLY READERS: This routine verifies the computer's working memory
; is functioning correctly. During Apollo 11's mission, RAM held navigation
; state vectors, guidance parameters, and crew display data. Any RAM failure
; would be catastrophic, so this test runs continuously during idle periods.
;
; CODE-ALONG READERS: Test uses SKEEP registers for bookkeeping:
;   SKEEP7: Current test address (lowest of pair being checked)
;   SKEEP6: Backup of value at address X+1
;   SKEEP5: Backup of value at address X
;   SKEEP4: Saved EBANK register value during ERASLOOP
;   SKEEP3: Last address to check (highest address in range)
;   SKEEP2: Controls bank selection for switchable erasable
;
; Test duration: Approximately 7 seconds for complete erasable sweep
; ============================================================================

# SKEEP7 HOLDS LOWEST OF TWO ADDRESSES BEING CHECKED.
# SKEEP6 HOLDS B(X+1).
# SKEEP5 HOLDS B(X).
# SKEEP4 HOLDS C(EBANK) DURING ERASLOOP AND CHECKNJ.
# SKEEP3 HOLDS LAST ADDRESS BEING CHECKED (HIGHEST ADDRESS).
# Page 1288
# SKEEP2 CONTROLS CHECKING OF NON-SWITCHABLE ERASABLE MEMORY WITH BANK NUMBERS IN EB.
# ERASCHK TAKES APPROXMATELY 7 SECONDS

; Begin erasable test with unswitched bank (E0, addresses 1461-1777)
ERASCHK		CA	S+1		; Set SKEEP2 to +1 (unswitched erasable flag)
		TS	SKEEP2
0EBANK		CA	S+ZERO		; Select EBANK 0 (unswitched erasable)
		TS	EBANK
		CA	ERASCON3	# 01461 (starting address, avoids self-check workspace)
		TS	SKEEP7		# STARTING ADDRESS
		CA	S10BITS		# 01777 (highest address in unswitched erasable)
		TS	SKEEP3		# LAST ADDRESS CHECKED
		TC	ERASLOOP	; Call erasable test loop

; E134567B: Test entry point for switchable erasable banks 1,3,4,5,6,7
; Each bank has 400 octal words (addresses 1400-1777) that switch based on EBANK
E134567B	CA	ERASCON6	# 01400 (starting address in switchable region)
		TS	SKEEP7		# STARTING ADDRESS
		CA	S10BITS		# 01777 (highest address in bank)
		TS	SKEEP3		# LAST ADDRESS CHECKED
		TC	ERASLOOP	; Test this bank's switchable erasable

; 2EBANK: Test entry point for erasable bank 2 (special handling)
; Bank E2 has restricted address range 1400-1773 (avoids self-check variables)
2EBANK		CA	ERASCON6	# 01400 (starting address)
		TS	SKEEP7		# STARTING ADDRESS
		CA	ERASCON4	# 01773 (restricted upper limit for E2)
		TS	SKEEP3		# LAST ADDRESS CHECKED
		TC	ERASLOOP	; Test bank E2's accessible addresses

; NOEBANK: Test entry point for unswitched erasable (low addresses 61-1373)
; These addresses are always accessible regardless of EBANK setting
; Contains central registers, interrupt vectors, and core system variables
NOEBANK		TS	SKEEP2		# +0 (clear bank selection flag)
		CA	ERASCON1	# 00061 (starting address, above fixed registers)
		TS	SKEEP7		# STARTING ADDRESS
		CA	ERASCON2	# 01373 (highest address before switchable region)
		TS	SKEEP3		# LAST ADDRESS CHECKED

; ============================================================================
; ERASLOOP - Main Erasable Memory Test Loop
;
; Test algorithm for each address pair (X, X+1):
; 1. Save original values of X and X+1 to SKEEP5 and SKEEP6
; 2. Write address values into X and X+1 (address as data)
; 3. Read back and verify: X + ~(X+1) should equal -1
; 4. Write complement of address values into X and X+1
; 5. Read back and verify: ~X + (X+1) should equal -1
; 6. Restore original values to X and X+1
; 7. Check for higher-priority jobs via CHECKNJ
; 8. Move to next address pair or next bank
;
; COMMENT-ONLY READERS: This loop tests each memory location by writing known
; patterns, reading them back, and verifying correctness. The test uses the
; address itself as the test pattern - a memory location should be able to
; hold its own address. Any mismatch indicates RAM hardware failure.
;
; CODE-ALONG READERS: INHINT disables interrupts during critical test section
; to prevent other code from corrupting test addresses. ERESTORE flag controls
; restart protection - if restart occurs during test, original values are
; restored before restart completes. Test uses double-precision operations
; (DCA/DXCH) to test adjacent word pairs efficiently.
; ============================================================================

ERASLOOP	INHINT			; Disable interrupts during test
		CA	EBANK		# STORES C(EBANK)
		TS	SKEEP4
		EXTEND
		NDX	SKEEP7
		DCA	0000
		DXCH	SKEEP5		# STORES C(X) AND C(X+1) IN SKEEP6 AND 5.
		CA	SKEEP7
		TS	ERESTORE	# IF RESTART, RESTORE C(X) AND C(X+1)
		TS	L
		INCR	L
		NDX	A
		DXCH	0000		# PUTS OWN ADDRESS IN X AND X +1
		NDX	SKEEP7
		CS	0001		# CS X+1
		NDX	SKEEP7
		AD	0000		# AD X
		TC	-1CHK
		CA	ERESTORE	# HAS ERASABLE BEEN RESTORED
		EXTEND
# Page 1289
		BZF	ELOOPFIN	# YES, EXIT ERASLOOP.
		EXTEND
		NDX	SKEEP7
		DCS	0000		# COMPLEMENT OF ADDRESS OF X AND X+1
		NDX	SKEEP7
		DXCH	0000		# PUT COMPLEMENT OF ADDRESS OF X AND X+1
		NDX	SKEEP7
		CS	0000		# CS X
		NDX	SKEEP7
		AD	0001		# AD X+1
		TC	-1CHK
		CA	ERESTORE	# HAS ERASABLE BEEN RESTORED
		EXTEND
		BZF	ELOOPFIN	# YES, EXIT ERASLOOP.
		EXTEND
		DCA	SKEEP5
		NDX	SKEEP7
		DXCH	0000		# PUT B(X) AND B(X+1) BACK INTO X AND X+1
		CA	S+ZERO
		TS	ERESTORE	# IF RESTART, DO NOT RESTORE C(X), C(X+1)
ELOOPFIN	RELINT
		TC	CHECKNJ		# CHECK FOR NEW JOB
		CA	SKEEP4		# REPLACES B(EBANK)
		TS	EBANK
		INCR	SKEEP7
		CS	SKEEP7
		AD	SKEEP3
		EXTEND
		BZF	+2
		TC	ERASLOOP	# GO TO NEXT ADDRESS IN SAME BANK

; ============================================================================
; BANK SWITCHING LOGIC - Move to Next Erasable Bank
;
; When all addresses in current bank have been tested, switch to next bank.
; Erasable memory organization: Bank 0 (unswitched), Banks 1-7 (switched E).
; This logic sequences through: E0 -> no switch -> E1,E3,E4,E5,E6,E7 -> E2
;
; COMMENT-ONLY READERS: After testing one bank of RAM, the self-check moves
; to the next bank to continue testing. This ensures all 2K words of erasable
; memory receive comprehensive testing.
;
; CODE-ALONG READERS: SKEEP2 flag controls bank sequencing. SBIT9 (bit 9 set)
; increments EBANK register to next bank. Special handling for E2 (bank 2)
; which is checked separately. ERASCON5 = 76777 used to detect E2 boundary.
; ============================================================================

		CCS	SKEEP2		; Check bank switching flag
		TC	NOEBANK		; Already in switched banks, continue
		INCR	SKEEP2		# PUT +1 IN SKEEP2 (set flag for switched)
		CA	EBANK		; Load current bank number
		AD	SBIT9		; +400 octal, increment to next bank
		TS	EBANK		; Store new bank number
		AD	ERASCON5	# 76777, CHECK FOR BANK E2 (adds 76777)
		EXTEND
		BZF	2EBANK		; Jump if result is zero (reached E2)
		CCS	EBANK		; Check if positive (banks 1,3,4,5,6,7)
		TC	E134567B	# GO TO EBANKS 1,3,4,5,6, AND 7
		CA	ERASCON6	# END OF ERASCHK (restore bank)
		TS	EBANK		; Reset EBANK register
; ============================================================================
; CNTRCHK - Counter and Special Register Check
;
; ORIGINAL NASA DESCRIPTION:
# CNTRCHK PERFORMS A CS OF ALL REGISTERS FROM OCT. 60 THROUGH OCT. 10.
# INCLUDED ARE ALL COUNTERS, T6-1, CYCLE AND SHIFT, AND ALL RUPT REGISTERS
;
; Test registers from octal 60 down to octal 10, including:
; - TIME6 through TIME1 counters (for task scheduling)
; - CYCLE and SHIFT registers (bit manipulation)
; - All interrupt (RUPT) registers (interrupt state storage)
;
; Method: Perform complement (CS) operation on each register to verify
; the register can be read without causing hardware fault. Does not verify
; register contents, only that access succeeds.
;
; COMMENT-ONLY READERS: This routine checks that all the computer's special
; hardware registers (timers, interrupt storage) can be accessed without error.
; It touches each register to ensure the hardware is functioning.
;
; CODE-ALONG READERS: Loop counts down from 50 octal (40 decimal) to zero,
; accessing registers at addresses 60+index down to 10 octal. CS (complement)
; operation tests read access. SBIT4 = +10 octal for address arithmetic.
; ============================================================================

CNTRCHK		CA	CNTRCON		# 00050 (starting count)
CNTRLOOP	TS	SKEEP2		; Save loop counter
		AD	SBIT4		# +10 OCTAL (address offset)
		INDEX	A		; Indexed addressing: address = A + 0
		CS	0000		; Complement register to test read access
# Page 1290
		CCS	SKEEP2		; Check counter, skip if zero
		TC	CNTRLOOP	; Continue loop if counter positive

; ============================================================================
; CYCLSHFT - Cycle and Shift Register Verification
;
; ORIGINAL NASA DESCRIPTION:
# CYCLSHFT CHECKS THE CYCLE AND SHIFT REGISTERS
;
; Tests the AGC's bit manipulation registers:
; - CYR (Cycle Right): Rotates bits right by one position
; - CYL (Cycle Left): Rotates bits left by one position
; - SR (Shift Right): Shifts bits right (logical shift, fills with sign bit)
; - EDOP (Edit Opcode): Extracts specific bit fields
;
; Test method:
; 1. Load test pattern CONC+S1 (25252 octal) into all four registers
; 2. Add results multiple times - each add causes register to cycle/shift
; 3. Verify sum equals -1 (proves registers operated correctly)
; 4. Repeat with different operations to test all register functions
;
; COMMENT-ONLY READERS: This section tests the computer's ability to move
; bits within a word - essential for data formatting, display output, and
; bit-field manipulation throughout the AGC software.
;
; CODE-ALONG READERS: Each TS to CYR/CYL/SR/EDOP causes automatic operation.
; CYR rotates right on read, CYL rotates left on read, SR shifts right on read,
; EDOP edits on read. Multiple AD operations accumulate transformed values.
; Final sum must equal -1 for test to pass. Two -1CHK calls verify both
; operation sequences produce correct cumulative results.
; ============================================================================

CYCLSHFT	CA	CONC+S1		# 25252 (test pattern: alternating bits)
		TS	CYR		# C(CYR) = 12525 (after right rotate)
		TS	CYL		# C(CYL) = 52524 (after left rotate)
		TS	SR		# C(SR) = 12525 (after right shift)
		TS	EDOP		# C(EDOP) = 00125 (after edit operation)
		AD	CYR		# 37777		C(CYR) = 45252 (rotates again)
		AD	CYL		# 00-12524	C(CYL) = 25251 (rotates again)
		AD	SR		# 00-25251	C(SR) = 05252 (shifts again)
		AD	EDOP		# 00-25376	C(EDOP) = +0 (edit exhausted)
		AD	CONC+S2		# C(CONC+S2) = 52400 (add second constant)
		TC	-1CHK		; Verify sum equals -1
		AD	CYR		# 45252 (continue accumulation)
		AD	CYL		# 72523
		AD	SR		# 77775
		AD	EDOP		# 77775
		AD	S+1		# 77776 (add +1)
		TC	-1CHK		; Verify second sum equals -1

; Test complete - increment self-check pass counter and continue
		INCR	SCOUNT +1	; Increment iteration counter
		TC	SMODECHK	; Return to mode check for next option

; ============================================================================
; ROPECHK - Fixed Memory (Core Rope) Checksum Verification
;
; Computes and verifies checksums of AGC fixed (read-only) memory banks.
; Fixed memory uses core rope technology - programs woven into magnetic cores
; during manufacture. This test detects any corruption in the 36K words of ROM.
;
; SKEEP register usage (NASA documentation):
# SKEEP1 HOLDS SUM
# SKEEP2 HOLDS PRESENT CONTENTS OF ADDRESS IN ROPECHK AND SHOWSUM ROUTINES
# SKEEP2 HOLDS BANK NUMBER IN LOW ORDER BITS DURING SHOWSUM DISPLAY
# SKEEP3 HOLDS PRESENT ADDRESS (00000 TO 01777 IN COMMON FIXED BANKS)
#			       (04000 TO 07777 IN FXFX BANKS)
# SKEEP3 HOLDS BUGGER WORD DURING SHOWSUM DISPLAY
# SKEEP4 HOLDS BANK NUMBER AND SUPER BANK NUMBER
# SKEEP5 COUNTS 2 SUCCESSIVE TC SELF WORDS
# SKEEP6 CONTROLS ROPECHK OR SHOWSUM OPTION
# SKEEP7 CONTROLS WHEN ROUNTINE IS IN COMMON FIXED OR FIXED FIXED BANKS
;
; Memory organization:
; - Common Fixed: Banks shared across all (addresses 0-1777 octal)
; - Fixed-Fixed: Banked memory requiring bank switching (banks 02, 03, 04...)
;
; Special handling: "TC SELF" words are self-referential jump instructions
; used as bugger words (checksums). Two consecutive TC SELF words signal
; end of bank checksum.
;
; COMMENT-ONLY READERS: This verifies that the permanent program memory
; (the flight software woven into the computer's core rope) has not been
; corrupted. Any corruption would cause mission failure, so this check
; is critical for ensuring the AGC can safely control the spacecraft.
;
; CODE-ALONG READERS: Checksums computed by adding all words in each bank.
; SKEEP6 = -0 for ROPECHK mode, +0 for SHOWSUM mode. SKEEP7 tracks whether
; processing common fixed (positive) or fixed-fixed banks (negative).
; SUPDACAL performs super bank addressing for banks above 03.
; ============================================================================

ROPECHK		CA	S-ZERO		# * -0 (negative zero)
		TS	SKEEP6		# * -0 FOR ROPECHK MODE
STSHOSUM	CA	S+ZERO		# * SHOULD BE ROPECHK (entry point note)

		TS	SKEEP4		# BANK NUMBER (initialize to +0)
		CA	S+1
; Common Fixed Memory Checksum - Banks Unswitched (0-1777 octal addresses)
COMMFX		TS	SKEEP7		; +1 = processing common fixed banks
		CA	S+ZERO
		TS	SKEEP1		; Initialize checksum accumulator
		TS	SKEEP3		; Initialize address counter
		CA	S+1
		TS	SKEEP5		# COUNTS DOWN 2 TC SELF WORDS (bugger sentinel)
COMADRS		CA	SKEEP4		; Load bank number
		TS	L		# TO SET SUPER BANK (L register for banking)
		MASK	HI5		; Isolate high 5 bits (super bank number)
# Page 1291
		AD	SKEEP3		; Add address offset within bank
		TC	SUPDACAL	# SUPER DATA CALL (handles super bank addressing)
		TC	ADSUM		; Accumulate into checksum
		AD	SBIT11		# 02000 (add address increment)
		TC	ADRSCHK		; Check if address range complete

; ============================================================================
; Fixed-Fixed Bank Checksum - Banked Memory (requires bank switching)
;
; Processes banks 02, 03, and higher (addresses 04000-07777 octal per bank).
; Each fixed-fixed bank contains 2K words requiring explicit bank selection.
;
; SKEEP7 stores negative value to indicate fixed-fixed mode.
; BZF tests whether to start at bank 02 (04000) or bank 03 (06000).
; ============================================================================

FXFX		CS	A		; Complement accumulator
		TS	SKEEP7		; Negative value = processing fixed-fixed banks
		EXTEND
		BZF	+3		; Branch if zero to select bank 03
		CA	SBIT12		# 04000, STARTING ADDRESS OF BANK 02
		TC	+2		; Skip next instruction
		CA	SBNK03		# 06000, STARTING ADDRESS OF BANK 03
		TS	SKEEP3		; Set starting address for this bank
		CA	S+ZERO
		TS	SKEEP1		; Initialize checksum accumulator
		CA	S+1
		TS	SKEEP5		# COUNTS DOWN 2 TC SELF WORDS (bugger sentinel)
; Loop through each address in fixed-fixed bank
FXADRS		INDEX	SKEEP3		; Indexed addressing: use SKEEP3 as address
		CA	0000		; Load word from current address
		TC	ADSUM		; Accumulate into checksum
		TC	ADRSCHK		; Check if address range complete

; ============================================================================
; ADSUM - Add to Checksum Subroutine
;
; Accumulates the word in A register into the running checksum (SKEEP1).
; Uses double-add technique: add word, then add +0 to propagate carry.
; Returns with address increment in A register.
;
; Entry: A = word to add to checksum
; Exit:  A = address increment for next location
;        SKEEP1 = updated checksum
;        SKEEP2 = word that was added
; ============================================================================

ADSUM		TS	SKEEP2		; Save the word being checksummed
		AD	SKEEP1		; Add to running checksum
		TS	SKEEP1		; Store updated checksum
		CAF	S+ZERO		; Load +0
		AD	SKEEP1		; Add to checksum (propagates carries)
		TS	SKEEP1		; Store checksum with carry propagated
		CS	SKEEP2		; Load complement of word
		AD	SKEEP3		; Add current address (for return value)
		TC	Q		; Return to caller

; ============================================================================
; ADRSCHK - Address Range Check Subroutine
;
; Verifies whether checksumming is complete for current bank. Checks:
; 1. If address reached maximum (1777 octal relative address)
; 2. If sentinel counter SKEEP5 is zero (bugger words processed)
; 3. If current word is a TC SELF bugger word (negative zero in L)
;
; Entry: A = address increment from ADSUM
; Exit: Continues checksum or proceeds to next bank/completion
; ============================================================================

ADRSCHK		LXCH	A		; Exchange A and L (save address increment)
		CA	SKEEP3		; Load current address
		MASK	LOW10		# RELATIVE ADDRESS (isolate low 10 bits)
		AD	-MAXADRS	# SUBTRACT MAX RELATIVE ADDRESS = 1777
		EXTEND
		BZF	SOPTION		# CHECKSUM FINISHED IF LAST ADDRESS
		CCS	SKEEP5		# IS CHECKSUM FINISHED (bugger sentinel)
		TC	+3		# NO (positive value, continue)
		TC	+2		# NO (zero value, continue)
		TC	SOPTION		# GO TO ROPECHK SHOWSUM OPTION (negative)
		CCS	L		# -0 MEANS A TC SELF WORD (bugger)
		TC	CONTINU		; Positive: not bugger word
		TC	CONTINU		; Positive zero: not bugger word
		TC	CONTINU		; Negative: not bugger word
		CCS	SKEEP5		; Negative zero in L = bugger word found
		TC	CONTINU +1	; Bugger sentinel still active
		CA	S-1		; Bugger sentinel exhausted
# Page 1292
		TC	CONTINU +1	# AD IN THE BUGGER WORD (decrement sentinel)
; Continue checksumming addresses within current bank
CONTINU		CA	S+1		# MAKE SURE TWO CONSECUTIVE TC SELF WORDS
		TS	SKEEP5		; Reset bugger sentinel counter
		CCS	SKEEP6		# * Check if SHOWSUM mode active
		CCS	NEWJOB		# * +1, SHOWSUM (check for higher priority job)
		TC	CHANG1		# * Higher priority job waiting
		TC	+2		# * No higher priority job
		TC	CHECKNJ		# -0 IN SKEEP6 FOR ROPECHK (yield to jobs)

; Increment to next address and continue checksumming
ADRS+1		INCR	SKEEP3		; Advance to next address
		CCS	SKEEP7		; Check mode: common fixed (+) or fixed-fixed (-)
		TC	COMADRS		; Positive: common fixed path
		TC	COMADRS		; Positive zero: common fixed path
		TC	FXADRS		; Negative: fixed-fixed path
		TC	FXADRS		; Negative zero: fixed-fixed path

; ============================================================================
; NXTBNK - Advance to Next Bank
;
; Moves checksumming to the next bank in sequence. Handles transitions:
; - Regular bank increments (add 02000 octal)
; - Super bank boundary transitions (banks 10-17 to 20-27, etc.)
; - Detection of final bank (LSTBNKCH comparison)
;
; Bank numbering: 02, 03, 04-07, 10-17, 20-27, 30-37, 40-43
; ============================================================================

NXTBNK		CS	SKEEP4		; Load complement of current bank
		AD	LSTBNKCH	# LAST BANK TO BE CHECKED (user-specified)
		EXTEND
		BZF	ENDSUMS		# END OF SUMMING OF BANKS (all banks done)
		CA	SKEEP4		; Load current bank number
		AD	SBIT11		; Add 02000 (increment bank by 1)
		TS	SKEEP4		# 37 TO 40 INCRMTS SKEEP4 BY END RND CARRY
		TC	CHKSUPR		; Check for super bank boundary
; Special transition from bank 17 to bank 20
17TO20		CA	SBIT15		; Load bit 15 value
		ADS	SKEEP4		# SET FOR BANK 20 (add to SKEEP4)
		TC	GONXTBNK	; Continue to next bank processing
; Check if super bank boundary crossed (banks x7 -> (x+1)0)
CHKSUPR		MASK	HI5		; Isolate high 5 bits (super bank number)
		EXTEND
		BZF	NXTSUPR		# INCREMENT SUPER BANK (super bank changed)
; Special transition from bank 27 to bank 30 (first super bank)
27TO30		AD	S13BITS		; Add bit pattern to test for bank 27
		EXTEND
		BZF	+2		# BANK SET FOR 30 (transition needed)
		TC	GONXTBNK	; Not bank 27, continue normally
		CA	SIXTY		# FIRST SUPER BANK (load super bank bits)
		ADS	SKEEP4		; Add super bank offset to reach bank 30
		TC	GONXTBNK	; Continue to next bank processing
; Super bank boundary crossed - set for next super bank
NXTSUPR		AD	SUPRCON		# SET BNK 30 + INCR SUPR BNK AND CANCEL
		ADS	SKEEP4		# ERC BIT OF THE 37 TO 40 ADVANCE
; Select processing mode for next bank: common fixed or fixed-fixed
GONXTBNK	CCS	SKEEP7		; Check mode selector
		TC	COMMFX		; Positive: common fixed bank
		CA	S+1		; Zero: prepare for fixed-fixed
		TC	FXFX		; Go to fixed-fixed processing
		CA	SBIT7		# HAS TO BE LARGER THAN NO OF FXSW BANKS
		TC	COMMFX		; Negative: go to common fixed

; ============================================================================
; SOPTION - Bank Checksum Completion
;
; Completes checksum for one bank. Extracts bank number from SKEEP4,
; handling super bank encoding. Verifies checksum equals bank number.
;
; Fixed memory checksums: Each bank sum should equal its bank number.
; This self-checking property allows verification of memory integrity.
;
; If SHOWSUM mode (SKEEP6 = +1), displays sum to crew via DSKY.
; If ROPECHK mode (SKEEP6 = -0), silently verifies and continues.
; ============================================================================

SOPTION		CA	SKEEP4		; Load bank number with super bank bits
		MASK	HI5		# = BANK BITS (bits 11-15)
		TC	LEFT5		; Shift left 5 bits
		TS	L		# BANK NUMBER BEFORE SUPER BANK
# Page 1293
		CA	SKEEP4		; Reload bank number
		MASK	S8BITS		# = SUPER BANK BITS (bits 8-10)
		EXTEND
		BZF	SOPT		# BEFORE SUPER BANK (banks 02-07)
		TS	SR		# SUPER BANK NECESSARY (save super bits)
		CA	L		; Load base bank number
		MASK	SEVEN		; Isolate low 3 bits
		AD	SR		; Add super bank offset
		TS	L		# BANK NUMBER WITH SUPER BANK (final)
; Check mode: SHOWSUM display or ROPECHK silent verification
SOPT		CA	SKEEP6		# * Load mode selector
		EXTEND			# *
		BZF	+2		# * ON -0 CONTINUE WITH ROPE CHECK
		TC	SDISPLAY	# * ON +1 GO TO DISPLAY OF SUM
		CCS	SKEEP1		# FORCE SUM TO ABSOLUTE VALUE
		TC	+2		; Positive, already absolute
		TC	+2		; Positive zero, already absolute
		AD	S+1		; Negative, make positive (|sum|)
		TS	SKEEP1		; Store absolute value of checksum
; Verify checksum equals bank number
BNKCHK		CS	L		# = - BANK NUMBER
		AD	SKEEP1		; Add checksum
		AD	S-1		; Adjust for comparison
		TC	-1CHK		# CHECK SUM (should equal -1 if correct)
		TC	NXTBNK

		EBANK=	NEWJOB
LSTBNKCH	BBCON*			# * CONSTANT, LAST BANK.

