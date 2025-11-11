# Copyright:	Public domain.
# Filename:	AGC_BLOCK_TWO_SELF-CHECK.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1394-1403
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Colossus249/ file of the same
#				name, using Comanche055 page images.
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
#    Assemble revision 055 of AGC program Comanche by NASA
#    2021113-051.  10:28 APR. 1, 1969
#
#    This AGC program shall also be referred to as
#            Colossus 2A

# ============================================================================
# FILE: AGC_BLOCK_TWO_SELF-CHECK.agc
# MODULE: CHIEFTAN Subsystem (Core OS)
# MISSION PHASE: all-phases (system verification)
#
# TL;DR: Self-test diagnostics verifying AGC hardware and instruction execution.
#        Implements memory testing, instruction validation, I/O verification,
#        and error detection/reporting ensuring computer health throughout
#        Apollo 11 mission. Critical for detecting hardware failures.
#
# COMMENT-ONLY READERS: This program tested the computer itself to make sure
#        all circuits and instructions were working correctly throughout the
#        entire Apollo 11 mission from launch through splashdown.
# CODE-ALONG READERS: Study self-test implementation, instruction validation,
#        memory testing patterns, error detection/reporting mechanisms, and
#        test coverage strategies for mission-critical computer verification.
# ============================================================================

# Page 1394
# PROGRAM DESCRIPTION				DATE  20 DECEMBER 1967
# PROGRAM NAME - SELF-CHECK			LOG SECTION  AGC BLOCK TWO SELF-CHECK
# MOD NO -  1					ASSEMBLY SUBROUTINE UTILITYM REV 25
# MOD BY - GAUNTT
#
#
# FUNCTIONAL DESCRIPTION
#
# 	PROGRAM HAS TWO MAIN PARTS. THE FIRST IS SELF-CHECK WHICH RUNS AS A ZERO PRIORITY JOB WITH NO CORE SET, AS
# PART OF THE BACK-UP IDLE LOOP. THE SECOND IS SHOW-BANKSUM WHICH RUNS AS A REGULAR EXECUTIVE JOB WITH ITS OWN
# STARTING VERB.
# 	THE PURPOSE OF SELF-CHECK IS TO CHECK OUT VARIOUS PARTS OF THE COMPUTER AS OUTLINED BELOW IN THE OPTIONS.
# 	THE PURPOSE OF SHOW-BANKSUM IS TO DISPLAY THE SUM OF EACH BANK , ONE AT A TIME.
# 	IN ALL THERE ARE  7 POSSIBLE OPTIONS IN THIS BLOCK II VERSION OF SELF-CHECK. MORE DETAIL DESCRIPTION MAY BE
# FOUND IN E-2065 BLOCK II AGC SELF-CHECK AND SHOW BANKSUM BY EDWIN D. SMALLY DECEMBER 1966, AND ADDENDA 2 AND 3.
# 	THE DIFFERENT OPTIONS ARE CONTROLLED BY PUTTING DIFFERENT NUMBERS IN THE SMODE REGISTER (NOUN 27). BELOW IS
# A DESCRIPTION OF WHAT PARTS OF THE COMPUTER THAT ARE CHECKED BY THE OPTIONS, AND THE CORRESPONDING NUMBER, IN
# OCTAL, TO LOAD INTO SMODE.
# +-4	ERASABLE MEMORY
# +-5	FIXED MEMORY
# +-1,2,3,6,7,10   EVERYTHING IN OPTIONS 4 AND 5.
# -0	SAME AS +-10 UNTIL AN ERROR IS DETECTED.
# +0	NO CHECK, PUTS COMPUTER INTO THE BACKUP IDLE LOOP.
#
#
# WARNINGS
# 	USE OF E MEMORY RESERVED FOR SELF-CHECK (EVEN IN IDLE LOOP) AS TEMP STORAGE BY OTHER PROGRAMS IS DANGEROUS.
# 	SMODE SET GREATER THAN OCT 10 PUTS COMPUTER INTO BACKUP IDLE LOOP.
#
#
# CALLING SEQUENCE
#
# 	TO CALL SELF-CHECK KEY IN
# 	     V 21 N 27 E  OPTION NUMBER E
# 	TO CALL SHOW-BANKSUM KEY IN
# 	     V 91 E	    DISPLAYS FIRST BANK
# 	     V 33 E	    PROCEED, DISPLAYS NEXT BANK
#
#
# EXIT MODES, NORMAL AND ALARM
#
# 	SELF-CHECK NORMALLY CONTINUES INDEFINITELY UNLESS THERE IS AN ERROR DETECTED. IF SO + OPTION NUMBERS PUT
# COMPUTER INTO BACKUP IDLE LOOP, - OPTION NUMBERS RESTART THE OPTION.
# 	THE -0 OPTION PROCEEDS FROM THE LINE FOLLOWING THE LINE WHERE THE ERROR WAS DETECTED.
# 	SHOW-BANKSUM PROCEEDS UNTIL A TERMINATE IS KEYED IN (V 34 E). THE COMPUTER IS PUT INTO THE BACKUP IDLE LOOP
#
#
#
# OUTPUT
# Page 1395
# 	SELF-CHECK UPON DETECTING AN ERROR LOADS THE SELF-CHECK ALARM CONSTANT (01102) INTO THE FAILREG SET AND
# TURNS ON THE ALARM LIGHT. THE OPERATOR MAY THEN DISPLAY THE THREE FAILREGS BY KEYING IN V 05 N 09 E. FOR FURTHER
# INFORMATION HE MAY KEY IN V 05 N 08 E, THE DSKY DISPLAY IN R1 WILL BE ADDRESS+1 OF WHERE THE ERROR WAS DETECTED,
# IN R2 THE BBCON OF SELF-CHECK, AND IN R3 THE TOTAL NUMBER OF ERRORS DETECTED BY SELF-CHECK SINCE THE LAST MAN
# INITIATED FRESH START (SLAP1).
# 	SHOW-BANKSUM STARTING WITH BANK 0 DISPLAYS IN R1 THE BANK SUM (A +-NUMBER EQUAL TO THE BANK NUMBER), IN R2
# THE BANK NUMBER, AND IN R3 THE BUGGER WORD.
#
#
# ERASABLE INITIALIZATION REQUIRED
# 	ACCOMPLISHED BY FRESH START
# 		SMODE SET TO +0
#
# DEBRIS
# 	ALL EXITS FROM THE CHECK OF ERASABLE (ERASCHK) RESTORE ORIGINAL CONTENTS TO REGISTERS UNDER CHECK.
# EXCEPTION IS A RESTART. RESTART THAT OCCURS DURING ERASCHK RESTORES ERASABLE, UNLESS THERE IS EVIDENCE TO DOUBT
# E MEMORY, IN WHICH CASE PROGRAM THEN DOES A FRESH START (DOFSTART).

; ============================================================================
; FILE: AGC_BLOCK_TWO_SELF-CHECK.agc
; MODULE: CHIEFTAN Subsystem (Core OS)
; MISSION PHASE: all-phases (system verification)
;
; TL;DR: Self-test diagnostics verifying AGC hardware and instruction execution.
;        Implements memory testing, instruction validation, I/O verification,
;        and error detection/reporting ensuring computer health throughout
;        Apollo 11 mission. Critical for detecting hardware failures.
;
; COMMENT-ONLY READERS: This program tested the computer itself to make sure
;        all circuits and instructions were working correctly.
; CODE-ALONG READERS: Study self-test implementation, instruction validation,
;        memory testing, error detection/reporting, test coverage strategies.
; ============================================================================

# ============================================================================
; TRANSITION: Self-Check Program Initialization
;
; This section defines constants and entry points for the AGC self-test
; diagnostics. Throughout Apollo 11's mission, this program ran continuously
; as a background job, testing the computer's memory, registers, and
; instruction execution to detect any hardware failures that could endanger
; the mission. The self-check provided critical confidence that the AGC
; remained healthy during all mission phases from launch through splashdown.
# ============================================================================

		BANK	25
		SETLOC	SELFCHEC
		BANK

		COUNT	43/SELF

; ----------------------------------------------------------------------------
; CONSTANT DEFINITIONS
;
; Self-check uses named constants for bit patterns, addresses, and test
; values. These constants enable testing specific bits in AGC words and
; defining memory boundaries for the various diagnostic routines.
; ----------------------------------------------------------------------------

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

S+ZERO		EQUALS	ZERO
S+1		EQUALS	BIT1
S+2		EQUALS	BIT2
S+3		EQUALS	THREE
S+4		EQUALS	FOUR
S+5		EQUALS	FIVE
# Page 1396
S+6		EQUALS	SIX
S+7		EQUALS	SEVEN
S8BITS		EQUALS	LOW8		# 00377
CNTRCON		=	OCT50		# USED IN CNTRCHK
ERASCON1	OCTAL	00061		# USED IN ERASCHK
ERASCON2	OCTAL	01373		# USED IN ERASCHK
ERASCON6	=	OCT1400		# USED IN ERASCHK
ERASCON3	OCTAL	01461		# USED IN ERASCHK
ERASCON4	OCTAL	01773		# USED IN ERASCHK
S10BITS		EQUALS	LOW10		# 01777, USED IN ERASCHK
SBNK03		EQUALS	PRIO6		# 06000, USED IN ROPECHK
-MAXADRS	=	HI5		# FOR ROPECHK
SIXTY		OCTAL	00060
SUPRCON		OCTAL	60017		# USED IN ROPECHK
S13BITS		OCTAL	17777
CONC+S1		OCTAL	25252		# USED IN CYCLSHFT
CONC+S2		OCTAL	52400		# USED IN CYCLSHFT
ERASCON5	OCTAL	76777
S-7		=	OCT77770
S-4		EQUALS	NEG4
S-3		EQUALS	NEG3
S-2		EQUALS	NEG2
S-1		EQUALS	NEGONE
S-ZERO		EQUALS	NEG0

		EBANK=	LST1
ADRS1		ADRES	SKEEP1
SELFADRS	ADRES	SELFCHK		# SELFCHK RETURN ADDRESS. SHOULD BE PUT
					# IN SELFRET WHEN GOING FROM SELFCHK TO
					# SHOWSUM AND PUT IN SKEEP1 WHEN GOING
					# FROM SHOWSUM TO SELF-CHECK.

; ============================================================================
; ERROR HANDLING AND REPORTING
;
; When self-check detected a hardware problem during Apollo 11, it needed to
; alert the crew without crashing the computer. The PRERRORS routine carefully
; restores any registers being tested, then signals alarm 01102 on the DSKY.
; Armstrong and Aldrin would see the ALARM light illuminate and could display
; error details using V05 N09 to see which test failed and where.
; ============================================================================

; The PRERRORS routine first checks if erasable memory registers need to be
; restored before reporting the error. This prevents the error handler itself
; from corrupting memory state, which would make debugging impossible.

PRERRORS	CA	ERESTORE	# IS IT NECESSARY TO RESTORE ERASABLE
		EXTEND
		BZF	ERRORS		# NO
		EXTEND
		DCA	SKEEP5
		INDEX	SKEEP7
		DXCH	0000		# RESTORE THE TWO ERASABLE REGISTERS
		CA	S+ZERO
		TS	ERESTORE

; With registers safely restored, the error is now reported to the crew.
; The failure address (Q register) is saved in SFAIL and ALMCADR for later
; display via V05 N08. The error counter (ERCOUNT) tracks total failures
; since the last fresh start, helping ground controllers assess computer health.

ERRORS		INHINT
		CA	Q
		TS	SFAIL		# SAVE Q FOR FAILURE LOCATION
		TS	ALMCADR		# FOR DISPLAY WITH BBANK AND ERCOUNT
		INCR	ERCOUNT		# KEEP TRACK OF NUMBER OF MALFUNCTIONS.

; Alarm 01102 illuminates the ALARM light on the DSKY. The crew can then
; key in V05 N09 to display the three FAILREG values, or V05 N08 to see
; the failure address, bank code, and total error count. This non-intrusive
; error reporting allowed Apollo 11 to continue flying even if self-check
; detected marginal hardware behavior.

TCALARM2	TC	ALARM2
		OCT	01102		# SELF-CHECK MALFUNCTION INDICATOR

; After reporting the alarm, behavior depends on SMODE setting:
; Positive SMODE: Enter idle loop (safe mode, stop testing)
; Negative SMODE: Restart the current test option
; -0 SMODE: Continue from next line after error (keep going)

		CCS	SMODE
SIDLOOP		CA	S+ZERO
# Page 1397
		TS	SMODE
		TC	SELFCHK		# GO TO IDLE LOOP
		TC	SFAIL		# CONTINUE WITH SELF-CHECK

; ----------------------------------------------------------------------------
; -1CHK: Minus-One Validation Routine
;
; This utility verifies that the accumulator (A) contains exactly -1 (octal
; 177777). The CCS instruction checks A for +zero, +positive, -zero, or
; -negative. If A is exactly -1, it cycles through the CCS logic correctly
; and returns via Q. Any other value triggers PRERRORS.
; ----------------------------------------------------------------------------

-1CHK		CCS	A
		TCF	PRERRORS
		TCF	PRERRORS
		CCS	A
		TCF	PRERRORS
		TC	Q

; ============================================================================
; SMODECHK: Self-Check Mode Dispatcher
;
; This routine runs as part of the backup idle loop, checking SMODE (noun 27)
; to determine which diagnostic option to execute. Throughout Apollo 11's
; mission, SMODE controlled what parts of the computer were being tested:
; +-4 checked erasable memory, +-5 checked fixed memory, other values ran
; comprehensive tests. The crew could change SMODE at any time via DSKY.
; ============================================================================

SMODECHK	EXTEND
		QXCH	SKEEP1
		TC	CHECKNJ		# CHECK FOR NEW JOB

; SMODE interpretation: The CCS instruction divides SMODE into four cases:
; +value: Run the specified option, then idle
; +0: Stay in backup idle loop (no testing)
; -0: Continuous testing mode until error detected
; -value: Run option, restart on error (aggressive testing)

		CCS	SMODE
		TC	SOPTIONS
		TC	SMODECHK +2	# TO BACKUP IDLE LOOP
		TC	SOPTIONS
		INCR	SCOUNT
		TC	SKEEP1		# CONTINUE WITH SELF-CHECK

; ----------------------------------------------------------------------------
; SOPTIONS: Option Number Decoder
;
; Validates that SMODE specifies a legal option (0-7 octal). Options above
; 10 octal are illegal and force the computer into backup idle loop.
; Valid options jump to the corresponding test routine via indexed TC.
; ----------------------------------------------------------------------------

SOPTIONS	AD	S-7
		EXTEND
		BZMF	+2		# FOR OPTIONS BELOW NINE.
BNKOPTN		TC	SIDLOOP		# ILLEGAL OPTION.  GO TO IDLE LOOP.
		INCR	SCOUNT		# FOR OPTIONS BELOW NINE.
		AD	S+7

; The option number now indexes into the jump table below. During Apollo 11,
; options 4 (erasable memory check) and 5 (fixed memory check) were the primary
; diagnostics. Options 1-3, 6-7 were present in earlier AGC versions but replaced
; with simple returns (TC SKEEP1) in Comanche 055.

		INDEX	A
		TC	SOPTION1
SOPTION1	TC	SKEEP1		# WAS TC+TCF
SOPTION2	TC	SKEEP1		# WAS IN:OUT1
SOPTION3	TC	SKEEP1		# WAS COUNTCHK
SOPTION4	TC	ERASCHK		# Check 2K words of erasable RAM
SOPTION5	TC	ROPECHK		# Check 36K words of fixed ROM (core rope)
SOPTION6	TC	SKEEP1
SOPTION7	TC	SKEEP1
SOPTON10	TC	SKEEP1		# CONTINUE WITH SELF-CHECK

; ----------------------------------------------------------------------------
; CHECKNJ: Check for New Job
;
; Since SELF-CHECK runs as a zero-priority idle loop task, it must periodically
; yield to higher-priority jobs. This routine checks if the executive has
; scheduled any real mission programs, and if so, suspends self-check to let
; them run. This ensures diagnostic testing never interferes with guidance,
; navigation, or control operations.
; ----------------------------------------------------------------------------

CHECKNJ		EXTEND
		QXCH	SELFRET		# SAVE RETURN ADDRESS WHILE TESTING NEWJOB
		TC	POSTJUMP	# TO SEE IF ANY JOBS HAVE BECOME ACTIVE.
		CADR	ADVAN

; ============================================================================
; SELFCHK: Main Self-Check Entry Point
;
; This is the primary entry into the self-check diagnostic system. The program
; runs continuously in the backup idle loop whenever no mission-critical tasks
; are active. On Apollo 11, this meant self-check ran during quiet periods in
; Earth orbit, translunar coast, and lunar orbit—always monitoring computer
; health without interfering with guidance or control.
; ============================================================================

SELFCHK		TC	SMODECHK	# ** CHARLEY, COME IN HERE

; ============================================================================
; ERASABLE MEMORY CHECK (ERASCHK)
;
; MISSION CONTEXT:
; The AGC's 2K words of erasable RAM (core memory) store all navigation state
; vectors, guidance parameters, crew inputs, and computational intermediate
; results. A single bit flip in erasable memory could cause catastrophic
; mission failure—incorrect thrust commands, navigation errors, or control
; system instability. ERASCHK detects RAM failures before they affect mission
; operations.
;
; TECHNICAL APPROACH:
; The test reads each erasable location, complements all bits, writes back the
; complement, reads to verify, then restores the original value. This detects
; stuck-at-0, stuck-at-1, and retention failures. Testing runs on live mission
; data, so interrupts are disabled during read-modify-restore sequences to
; prevent data corruption.
;
; MEMORY REGIONS TESTED:
; - 0EBANK (E-bank 0):  Addresses 1461-1777 octal
; - E134567B (E-banks 1,3,4,5,6,7): Addresses 1400-1777 octal in each bank
; - 2EBANK (E-bank 2): Addresses 1400-1773 octal
; - NOEBANK (unswitched): Addresses 0061-1373 octal
;
; The complete test takes approximately 7 seconds. During Apollo 11, this
; continuous background verification ensured the computer remained healthy
; throughout the 8-day mission.
;
; REGISTER USAGE:
; SKEEP7 holds lowest address being checked
; SKEEP6 holds original content of address X+1
; SKEEP5 holds original content of address X
; SKEEP4 holds EBANK register during test
; SKEEP3 holds highest address being checked
; SKEEP2 controls checking of switchable vs. non-switchable erasable
; ============================================================================

# SKEEP7 HOLDS LOWEST OF TWO ADDRESSES BEING CHECKED.
# SKEEP6 HOLDS B(X+1).
# SKEEP5 HOLDS B(X).
# SKEEP4 HOLDS C(EBANK) DURING ERASLOOP AND CHECKNJ.
# Page 1398
# SKEEP3 HOLDS LAST ADDRESS BEING CHECKED (HIGHEST ADDRESS).
# SKEEP2 CONTROLS CHECKING OF NON-SWITCHABLE ERASABLE MEMORY WITH BANK NUMBERS IN EB.
# ERASCHK TAKES APPROXMATELY 7 SECONDS

; First test E-bank 0 (the primary bank with most mission-critical variables).

ERASCHK		CA	S+1
		TS	SKEEP2
0EBANK		CA	S+ZERO
		TS	EBANK
		CA	ERASCON3	# 01461
		TS	SKEEP7		# STARTING ADDRESS
		CA	S10BITS		# 01777
		TS	SKEEP3		# LAST ADDRESS CHECKED
		TC	ERASLOOP

E134567B	CA	ERASCON6	# 01400
		TS	SKEEP7		# STARTING ADDRESS
		CA	S10BITS		# 01777
		TS	SKEEP3		# LAST ADDRESS CHECKED
		TC	ERASLOOP

2EBANK		CA	ERASCON6	# 01400
		TS	SKEEP7		# STARTING ADDRESS
		CA	ERASCON4	# 01773
		TS	SKEEP3		# LAST ADDRESS CHECKED
		TC	ERASLOOP

NOEBANK		TS	SKEEP2		# +0
		CA	ERASCON1	# 00061
		TS	SKEEP7		# STARTING ADDRESS
		CA	ERASCON2	# 01373
		TS	SKEEP3		# LAST ADDRESS CHECKED

; ----------------------------------------------------------------------------
; ERASLOOP: Main Erasable Test Loop
;
; Tests two consecutive memory locations (X and X+1) using the "address-in-data"
; method. The test writes each location's own address into itself, verifies
; correct storage, then writes the complement of the address, verifies that,
; and finally restores the original mission data.
;
; COMMENT-ONLY READERS: This is the heart of the memory test. The computer
; writes test patterns into RAM, verifies they stored correctly, then puts
; the real data back before resuming mission operations. If any bit is stuck
; at 0 or 1, the verification will fail and trigger an alarm.
;
; CODE-ALONG READERS: The test exploits a clever property—if locations X and
; X+1 store values X and (X+1), then CS(X+1) + X = -1. This provides strong
; error detection with minimal instructions. Interrupts are disabled during
; test/restore to prevent mission code from reading corrupted data.
; ----------------------------------------------------------------------------

ERASLOOP	INHINT			# Disable interrupts—mission data will be corrupted temporarily
		CA	EBANK		# STORES C(EBANK)
		TS	SKEEP4
		EXTEND
		NDX	SKEEP7
		DCA	0000		# Read X and X+1 (double-precision indexed load)
		DXCH	SKEEP5		# STORES C(X) AND C(X+1) IN SKEEP6 AND 5.
		CA	SKEEP7
		TS	ERESTORE	# IF RESTART, RESTORE C(X) AND C(X+1)
		TS	L		# Put address X in L register
		INCR	L		# L now contains X+1
		NDX	A		# A still contains X
		DXCH	0000		# PUTS OWN ADDRESS IN X AND X +1
					# (Location X now contains value X, location X+1 contains X+1)
		NDX	SKEEP7
		CS	0001		# CS X+1 (complement of location X+1)
		NDX	SKEEP7
		AD	0000		# AD X (add location X)
		TC	-1CHK		# Verify CS(X+1) + X = -1 (must be true if X stored X and X+1 stored X+1)
		CA	ERESTORE	# HAS ERASABLE BEEN RESTORED
# Page 1399
		EXTEND
		BZF	ELOOPFIN	# YES, EXIT ERASLOOP.

; Phase 2: Test with complemented address pattern. This detects different failure
; modes than phase 1. If a bit is stuck-at-0, it will pass the first test but fail
; when we write the complement (which tries to set that bit to 1).

		EXTEND
		NDX	SKEEP7
		DCS	0000		# COMPLEMENT OF ADDRESS OF X AND X+1
		NDX	SKEEP7
		DXCH	0000		# PUT COMPLEMENT OF ADDRESS OF X AND X+1
					# (Location X now contains ~X, location X+1 contains ~(X+1))
		NDX	SKEEP7
		CS	0000		# CS X (complement of location X, which is ~~X = X)
		NDX	SKEEP7
		AD	0001		# AD X+1 (add location X+1, which is ~(X+1))
		TC	-1CHK		# Verify CS(~X) + ~(X+1) = X + ~(X+1) = -1
		CA	ERESTORE	# HAS ERASABLE BEEN RESTORED
		EXTEND
		BZF	ELOOPFIN	# YES, EXIT ERASLOOP.

; Phase 3: Restore original mission data. This is critical—navigation state,
; guidance parameters, and all mission variables must be restored exactly
; before re-enabling interrupts and resuming flight operations.

		EXTEND
		DCA	SKEEP5		# Get original values from backup storage
		NDX	SKEEP7
		DXCH	0000		# PUT B(X) AND B(X+1) BACK INTO X AND X+1
		CA	S+ZERO
		TS	ERESTORE	# IF RESTART, DO NOT RESTORE C(X), C(X+1)
ELOOPFIN	RELINT			# Re-enable interrupts—memory restored, safe to resume
		TC	CHECKNJ		# CHECK FOR NEW JOB (yield to mission programs if needed)
		CA	SKEEP4		# REPLACES B(EBANK)
		TS	EBANK		# Restore original E-bank setting
		INCR	SKEEP7		# Advance to next address
		CS	SKEEP7		# Complement current address
		AD	SKEEP3		# Subtract from maximum address (forms SKEEP3 - SKEEP7)
		EXTEND
		BZF	+2		# If zero, we've reached the end of this region
		TC	ERASLOOP	# GO TO NEXT ADDRESS IN SAME BANK
		CCS	SKEEP2		# Check if more banks remain (SKEEP2 = bank counter)
		TC	NOEBANK		# Continue to non-switched erasable test
		INCR	SKEEP2		# PUT +1 IN SKEEP2.

; This code handles E-bank (erasable memory bank) switching. The AGC has
; multiple banks of erasable memory accessed via the EBANK register. By
; systematically switching banks and re-running ERASLOOP, self-check tests
; all 2K words of erasable RAM across all banks.

		CA	EBANK		# Get current bank number
		AD	SBIT9		# Add 0400 octal (advances to next bank)
		TS	EBANK		# Switch to next E-bank
		AD	ERASCON5	# 76777, CHECK FOR BANK E2
		EXTEND
		BZF	2EBANK		# Special handling for E-bank 2
		CCS	EBANK		# Check if we've tested all banks
		TC	E134567B	# GO TO EBANKS 1,3,4,5,6, AND 7
		CA	ERASCON6	# END OF ERASCHK (all banks tested)
		TS	EBANK		# Restore to default bank
; ----------------------------------------------------------------------------
; CNTRCHK: Counter and Special Register Test
;
; This routine tests hardware counters and special-purpose registers (addresses
; octal 10 through 60). These include:
; - Timer counters T6-T1 (timing critical mission events)
; - Cycle and shift registers (CYR, CYL, SR, EDOP)
; - Interrupt save registers (preserving state during interrupts)
;
; The test simply reads each register using CS (complement and skip if zero),
; which verifies the address decoding and read paths work. Full functional
; testing of timers happens during actual mission operations.
; ----------------------------------------------------------------------------

# CNTRCHK PERFORMS A CS OF ALL REGISTERS FROM OCT. 60 THROUGH OCT. 10.
# INCLUDED ARE ALL COUNTERS, T6-1, CYCLE AND SHIFT, AND ALL RUPT REGISTERS
CNTRCHK		CA	CNTRCON		# 00050
CNTRLOOP	TS	SKEEP2		# Save loop counter
		AD	SBIT4		# +10 OCTAL (advance to next register address)
		INDEX	A		# Use computed address as index
# Page 1400
		CS	0000		# Read register (complement validates read path)
		CCS	SKEEP2		# Decrement loop counter, continue if non-zero
		TC	CNTRLOOP

; ----------------------------------------------------------------------------
; CYCLSHFT: Cycle and Shift Register Hardware Test
;
; Tests the AGC's bit-manipulation hardware registers:
; - CYR (Cycle Right): Rotates A register right, filling vacated bits from right
; - CYL (Cycle Left): Rotates A register left, filling vacated bits from left
; - SR (Shift Right): Shifts A right, filling with sign bit (arithmetic shift)
; - EDOP (Edit Opcode): Special editing register used by certain instructions
;
; The test writes pattern 25252 octal (alternating bits) to each register,
; then reads them back. Each register performs its shift/cycle operation
; automatically on write, so the read value differs from the written value
; in a predictable way. Summing all results should produce -1 (all bits set).
; ----------------------------------------------------------------------------

# CYCLSHFT CHECKS THE CYCLE AND SHIFT REGISTERS
CYCLSHFT	CA	CONC+S1		# 25252 octal (alternating bit pattern)
		TS	CYR		# C(CYR) = 12525 (cycled right 1 position)
		TS	CYL		# C(CYL) = 52524 (cycled left 1 position)
		TS	SR		# C(SR) = 12525 (shifted right, sign extended)
		TS	EDOP		# C(EDOP) = 00125 (editing operation result)
		AD	CYR		# 37777		C(CYR) = 45252 (accumulate)
		AD	CYL		# 00-12524	C(CYL) = 25251
		AD	SR		# 00-25251	C(SR) = 05252
		AD	EDOP		# 00-25376	C(EDOP) = +0
		AD	CONC+S2		# C(CONC+S2) = 52400 (sum should be -1)
		TC	-1CHK		# Verify sum = -1 (all registers working)
		AD	CYR		# 45252 (second accumulation test)
		AD	CYL		# 72523
		AD	SR		# 77775
		AD	EDOP		# 77775
		AD	S+1		# 77776 (sum should again be -1)
		TC	-1CHK		# Verify second sum = -1

; ERASCHK complete. Increment iteration counter and check for mode change.

		INCR	SCOUNT +1	# Count completed erasable test iterations
		TC	SMODECHK	# Return to dispatcher to check for new option
# SKEEP1 HOLDS SUM
# SKEEP2 HOLDS PRESENT CONTENTS OF ADDRESS IN ROPECHK AND SHOWSUM ROUTINES
# SKEEP2 HOLDS BANK NUMBER IN LOW ORDER BITS DURING SHOWSUM DISPLAY
# SKEEP3 HOLDS PRESENT ADDRESS (00000 TO 01777 IN COMMON FIXED BANKS)
#			       (04000 TO 07777 IN FXFX BANKS)
; ============================================================================
; SECTION 3: FIXED MEMORY (ROM) CHECKSUM TEST
;
; ROPECHK verifies the integrity of core rope memory (36K words of read-only
; program storage). The AGC's program lives in "fixed" memory - literally
; woven copper wires through magnetic cores during manufacturing. Any
; manufacturing defect or cosmic ray damage could cause mission-critical
; failures, so this test checksums every bank of ROM.
;
; The test works bank-by-bank:
; 1. Sum all words in a bank (2K words per bank)
; 2. The sum should equal the bank number (a deliberate "checksum constant"
;    embedded during assembly by placing a "bugger word" at the end of each
;    bank that makes the sum work out correctly)
; 3. Special handling for "TC SELF" instructions (self-modifying code markers)
;
; During Apollo 11 descent, this test ran continuously in the background.
; If it had detected ROM corruption, the mission would have been aborted.
; The fact that you're reading this means it worked flawlessly.
; ============================================================================

# SKEEP3 HOLDS BUGGER WORD DURING SHOWSUM DISPLAY
# SKEEP4 HOLDS BANK NUMBER AND SUPER BANK NUMBER
# SKEEP5 COUNTS 2 SUCCESSIVE TC SELF WORDS
# SKEEP6 CONTROLS ROPECHK OR SHOWSUM OPTION
# SKEEP7 CONTROLS WHEN ROUNTINE IS IN COMMON FIXED OR FIXED FIXED BANKS

ROPECHK		CA	S-ZERO		# *
		TS	SKEEP6		# * -0 FOR ROPECHK (vs +1 for SHOWSUM display)
STSHOSUM	CA	S+ZERO		# * Entry point for SHOWSUM (bank display mode)

		TS	SKEEP4		# BANK NUMBER (initialize to bank 0)
		CA	S+1
COMMFX		TS	SKEEP7		# +1 indicates common fixed memory region
		CA	S+ZERO
		TS	SKEEP1		# Initialize checksum accumulator to zero
		TS	SKEEP3		# Initialize address within bank to zero
		CA	S+1
		TS	SKEEP5		# COUNTS DOWN 2 TC SELF WORDS
COMADRS		CA	SKEEP4		# Get current bank number
		TS	L		# TO SET SUPER BANK (banks 30-37 use super banking)
# Page 1401
		MASK	HI5		# Extract bank bits (bits 10-14)
		AD	SKEEP3		# Add relative address within bank
		TC	SUPDACAL	# SUPER DATA CALL (fetch data using bank switching)
		TC	ADSUM		# Add fetched word to running checksum
		AD	SBIT11		# 02000 (advance address)
		TC	ADRSCHK		# Check if bank complete

; Fixed-fixed memory: Banks 2-3 (addresses 04000-07777)
; Unlike common-fixed, these banks can be accessed directly without super-bank
; addressing. The AGC has only two fixed-fixed banks, so the code alternates
; between checking bank 2 (starting at 04000) and bank 3 (starting at 06000).

FXFX		CS	A		# Complement A to get negative of SKEEP7 state
		TS	SKEEP7		# Store bank indicator (toggles between banks 2/3)
		EXTEND
		BZF	+3		# Branch if zero (to bank 03)
		CA	SBIT12		# 04000, STARTING ADDRESS OF BANK 02
		TC	+2		# Skip next instruction
		CA	SBNK03		# 06000, STARTING ADDRESS OF BANK 03
		TS	SKEEP3		# Set starting address for this bank
		CA	S+ZERO
		TS	SKEEP1		# Initialize checksum accumulator
		CA	S+1
		TS	SKEEP5		# COUNTS DOWN 2 TC SELF WORDS
FXADRS		INDEX	SKEEP3		# Index by address within bank
		CA	0000		# Read word from fixed-fixed memory
		TC	ADSUM		# Add to checksum
		TC	ADRSCHK		# Check if bank complete

; ADSUM: Checksum Accumulator Subroutine
; Adds the fetched word to the running checksum, with special handling for
; "TC SELF" instructions. Returns with A containing next address increment.

ADSUM		TS	SKEEP2		# Save fetched word temporarily
		AD	SKEEP1		# Add to running checksum
		TS	SKEEP1		# Store updated checksum
		CAF	S+ZERO		# Clear A, prepare for end-around carry
		AD	SKEEP1		# Re-add checksum (handles overflow)
		TS	SKEEP1		# Store final checksum value
		CS	SKEEP2		# Get complement of fetched word
		AD	SKEEP3		# Add current address
		TC	Q		# Return to caller

; ADRSCHK: Address Check - Determines if Bank Checksum is Complete
; Checks if we've reached the end of the current bank (address 1777 octal
; relative). If so, verifies checksum. If not, continues to next word.
; Special handling for "TC SELF" marker words which appear in pairs.

ADRSCHK		LXCH	A		# Save address increment in L
		CA	SKEEP3		# Get current address
		MASK	LOW10		# RELATIVE ADDRESS (within bank, 0-1777)
		AD	-MAXADRS	# SUBTRACT MAX RELATIVE ADDRESS = 1777.
		EXTEND
		BZF	SOPTION		# CHECKSUM FINISHED IF LAST ADDRESS.
		CCS	SKEEP5		# IS CHECKSUM FINISHED (TC SELF counter)
		TC	+3		# NO (still positive, continue)
		TC	+2		# NO (still +0, continue)
		TC	SOPTION		# GO TO ROPECHK SHOWSUM OPTION
; The AGC uses special "TC SELF" marker words in pairs to mark bank boundaries.
; These markers aren't executable code - they're placeholders that help the
; checksum algorithm locate the "bugger word" (the correction value that makes
; each bank's checksum equal the bank number). When ADSUM detects a TC SELF,
; it returns -0 in L. This code watches for two consecutive TC SELF words,
; then knows the next word is the bugger word to include in the checksum.

		CCS	L		# -0 MEANS A TC SELF WORD (marker)
		TC	CONTINU		# Not TC SELF, continue normally
		TC	CONTINU		# Positive zero, continue
		TC	CONTINU		# Negative, continue
		CCS	SKEEP5		# Was this second consecutive TC SELF?
		TC	CONTINU +1	# No, first TC SELF, don't add bugger yet
# Page 1402
		CA	S-1		# Yes, second TC SELF - decrement counter
		TC	CONTINU +1	# AD IN THE BUGGER WORD (will be added next)

; Normal address increment: reset TC SELF counter and continue testing.
; In SHOWSUM display mode, also checks NEWJOB to yield processor if higher
; priority work arrives (self-check runs at zero priority, lowest of all).

CONTINU		CA	S+1		# MAKE SURE TWO CONSECUTIVE TC SELF WORDS
		TS	SKEEP5		# Reset TC SELF counter
		CCS	SKEEP6		# * Check ROPECHK vs SHOWSUM mode
		CCS	NEWJOB		# * +1 for SHOWSUM (display mode)
		TC	CHANG1		# * Higher priority job waiting, yield
		TC	+2		# * No job waiting, continue
		TC	CHECKNJ		# -0 IN SKEEP6 FOR ROPECHK

; ADRS+1: Increment Address and Continue
; Advances to next word in current bank and returns to appropriate handler.

ADRS+1		INCR	SKEEP3		# Advance to next address in bank
		CCS	SKEEP7		# Check bank type indicator
		TC	COMADRS		# Positive: common-fixed bank
		TC	COMADRS		# +0: common-fixed bank
		TC	FXADRS		# Negative: fixed-fixed bank
		TC	FXADRS		# -0: fixed-fixed bank

; NXTBNK: Advance to Next Bank
; The AGC's memory has gaps in the bank numbering: banks 0-1 (unswitched),
; 2-3 (fixed-fixed), then 4-27 regular, then 30-37 (super-banks with extended
; addressing). This routine handles the complex sequencing through these banks.

NXTBNK		CS	SKEEP4		# Get complement of current bank number
		AD	LSTBNKCH	# LAST BANK TO BE CHECKED (bank 43 octal)
		EXTEND
		BZF	ENDSUMS		# END OF SUMMING OF BANKS.
		CA	SKEEP4		# Get current bank number
		AD	SBIT11		# Add 02000 (advances bank by 1)
		TS	SKEEP4		# 37 TO 40 INCRMTS SKEEP4 BY END RND CARRY
		TC	CHKSUPR		# Check if special bank transition needed
17TO20		CA	SBIT15		# Special handling for bank 17→20 transition
		ADS	SKEEP4		# SET FOR BANK 20 (skip banks 18-19)
		TC	GONXTBNK	# Go to next bank
CHKSUPR		MASK	HI5		# Extract high bits to check bank range
		EXTEND
		BZF	NXTSUPR		# INCREMENT SUPER BANK (banks 30-37)
27TO30		AD	S13BITS		# Check for bank 27→30 transition
		EXTEND
		BZF	+2		# BANK SET FOR 30
		TC	GONXTBNK	# Normal bank sequence, continue
		CA	SIXTY		# FIRST SUPER BANK (bank 30 = 60 octal)
		ADS	SKEEP4		# Jump to super-bank region
		TC	GONXTBNK	# Go to next bank
NXTSUPR		AD	SUPRCON		# SET BNK 30 + INCR SUPR BNK AND CANCEL
		ADS	SKEEP4		# ERC BIT OF THE 37 TO 40 ADVANCE.
GONXTBNK	CCS	SKEEP7		# Check bank type (common vs fixed-fixed)
		TC	COMMFX		# Positive: common-fixed bank, restart COMMFX
		CA	S+1		# +0: prepare for fixed-fixed
		TC	FXFX		# Restart FXFX for fixed-fixed banks
		CA	SBIT7		# HAS TO BE LARGER THAN NO OF FXSW BANKS.
		TC	COMMFX		# Continue with common-fixed processing

; ============================================================================
; SOPTION: Show/Validate Bank Checksum
; After completing a bank checksum, this routine:
; 1. Computes the actual bank number (including super-bank adjustments)
; 2. Either displays the checksum (SHOWSUM mode) or validates it (ROPECHK)
; 3. The checksum should equal -(bank number) - 1, the "bugger word"
; ============================================================================

SOPTION		CA	SKEEP4		# Get raw bank number
		MASK	HI5		# = BANK BITS (extract basic bank)
		TC	LEFT5		# Shift left 5 bits to get bank position
# Page 1403
		TS	L		# BANK NUMBER BEFORE SUPER BANK adjustment
		CA	SKEEP4		# Get raw bank again
		MASK	S8BITS		# = SUPER BANK BITS (banks 30-37)
		EXTEND
		BZF	SOPT		# BEFORE SUPER BANK (banks 0-27)
		TS	SR		# SUPER BANK NECESSARY - save super bits
		CA	L		# Get basic bank number
		MASK	SEVEN		# Keep only low 3 bits
		AD	SR		# Add super-bank offset
		TS	L		# BANK NUMBER WITH SUPER BANK correction
SOPT		CA	SKEEP6		# * Check ROPECHK vs SHOWSUM mode
		EXTEND			# *
		BZF	+2		# * ON -0 CONTINUE WITH ROPE CHECK.
		TC	SDISPLAY	# * ON +1 GO TO DISPLAY OF SUM.
		CCS	SKEEP1		# FORCE SUM TO ABSOLUTE VALUE.
		TC	+2		# Already positive, continue
		TC	+2		# +0, continue
		AD	S+1		# Negative: make positive by adding 1
		TS	SKEEP1		# Save absolute value of checksum
BNKCHK		CS	L		# = - BANK NUMBER (compute bugger word)
		AD	SKEEP1		# Add checksum (should cancel out)
		AD	S-1		# Subtract 1 (bugger word is -bank-1)
		TC	-1CHK		# CHECK SUM (should be -1 if correct)
		TC	NXTBNK		# Move to next bank

		EBANK=	NEWJOB
LSTBNKCH	BBCON*			# * CONSTANT, LAST BANK.
		SBANK=	LOWSUPER
