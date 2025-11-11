# Copyright:    Public domain.
# Filename:     INTER-BANK_COMMUNICATION.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1103-1106
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-08 RSB	Adapted from Colossus249/ file of same name
#				and page images. Corrected various typos
#				in the transcription of program comments,
#				and these should be back-ported to
#				Colossus249.
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
; FILE: INTER-BANK_COMMUNICATION.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Memory bank switching routines enabling access to AGC's 36K fixed
;        memory organized in 2K banks. Implements BANKCALL mechanism for
;        cross-bank subroutine calls with return address preservation. Essential
;        architectural infrastructure for addressing memory beyond single bank's
;        2048-word limit throughout Apollo 11 mission.
;
; COMMENT-ONLY READERS: Allowed the computer to access all its program memory
;        by switching between different memory banks.
; CODE-ALONG READERS: Study memory bank switching routines, BANKCALL mechanism,
;        fixed-fixed and fixed-erasable transitions, return address preservation
;        using Q register and BUF2 temporary storage.
; ============================================================================

# Page 1103
# THE FOLLOWING ROUTINE CAN BE USED TO CALL A SUBROUTINE IN ANOTHER BANK. IN THE BANKCALL VERSION, THE
# CADR OF THE SUBROUTINE IMMEDIATELY FOLLOWS THE TC BANKCALL INSTRUCTION, WITH C(A) AND C(L) PRESERVED.

; ============================================================================
; AGC MEMORY BANKING ARCHITECTURE OVERVIEW
;
; The Apollo Guidance Computer has 36K words (36,864 words) of fixed memory
; (read-only core rope) organized into banks of 2K words (2048 words) each.
; A single bank address space can only directly access 2K of memory at a time.
; To access code in other banks, the AGC uses bank switching.
;
; Key concepts:
; - FBANK register: Holds the current fixed memory bank number (0-37 octal)
; - CADR (Combined Address): 16-bit value containing both bank number (upper
;   bits) and relative address within bank (lower 10 bits in LOW10 mask)
; - Q register: Contains return address for subroutine calls
; - BUF2/BUF2+1: Double-precision temporary storage for preserving A and L
;   registers across bank switches
;
; This code was essential during every phase of Apollo 11: guidance programs
; called navigation routines in other banks, which called interpreter code in
; yet other banks, requiring seamless bank switching thousands of times per
; second throughout the mission.
; ============================================================================

		BLOCK	02
		COUNT	02/BANK

; BANKCALL: Cross-bank subroutine call with A and L register preservation.
;
; Calling sequence:
;        TC      BANKCALL
;        CADR    TARGETROUTINE      ; Combined address of routine in another bank
;        ...                         ; Return here after subroutine completes
;
; The AGC program counter (Q register) points to the CADR after TC BANKCALL.
; This routine fetches the CADR, switches to the target bank, preserves the
; calling routine's A and L registers, and jumps to the target subroutine.
; Upon return via SWRETURN, control passes to the instruction after the CADR.

BANKCALL	DXCH	BUF2		# SAVE INCOMING A,L.
		INDEX	Q		# PICK UP CADR.
		CA	0
		INCR	Q		# SO WE RETURN TO THE LOC. AFTER THE CADR.

# SWCALL IS  IDENTICAL TO BANKCALL, EXCEPT THAT THE CADR ARRIVES IN A.

; SWCALL: Bank switch and call with CADR already in accumulator.
;
; This is the common code path for both BANKCALL and programmatic bank calls.
; The CADR in A register contains:
;   - Upper bits: Target bank number
;   - Lower 10 bits (LOW10 mask): Relative address within target bank
;
; The routine performs these operations:
; 1. Store CADR to L register
; 2. Exchange L with FBANK, switching to target bank and saving return bank
; 3. Mask out bank bits to get relative address (0-2047 within bank)
; 4. Exchange with Q to set up return address
; 5. Restore original A,L if this was a BANKCALL
; 6. Jump to target address (10000 octal = base of current bank + offset)

SWCALL		TS	L
		LXCH	FBANK		# SWITCH BANKS, SAVING RETURN.
		MASK	LOW10		# GET SUB-ADDRESS OF CADR.
		XCH	Q		# A,L NOW CONTAINS DP RETURN.
		DXCH	BUF2		# RESTORING INPUTS IF THIS IS A BANKCALL.
		INDEX	Q
		TC	10000		# SETTING Q TO SWRETURN

; SWRETURN: Return from bank-switched subroutine to original caller's bank.
;
; The called subroutine executes TC Q (or RETURN) which brings control here.
; This routine:
; 1. Preserves the return value in A register via BUF2+1
; 2. Restores the original bank number from FBANK
; 3. Restores A register
; 4. Transfers control to return address (stored in BUF2)
;
; This completed the round-trip: caller's bank → target bank → caller's bank,
; with A and L register contents preserved throughout. Critical for maintaining
; computational state during complex mission calculations spanning multiple banks.

SWRETURN	XCH	BUF2 +1		# COMES HERE TO RETURN TO CALLER. C(A,L)
		XCH	FBANK		# ARE PRESERVED FOR RETURN.
		XCH	BUF2 +1
		TC	BUF2

# THE FOLLOWING ROUTINE CAN BE USED AS A UNILATERAL JUMP WITH C(A,L) PRESERVED AND THE CADR IMMEDIATELY
# FOLLOWING THE TC POSTJUMP INSTRUCTION.

; POSTJUMP: One-way jump to another bank (no return path preserved).
;
; Unlike BANKCALL which sets up a return path, POSTJUMP is a permanent
; transfer of control to another bank. Used when a program phase completes
; and needs to hand off to a different program in another bank.
;
; Calling sequence:
;        TC      POSTJUMP
;        CADR    TARGETROUTINE      ; Jump destination in another bank
;
; During Apollo 11 mission, POSTJUMP was used for major program transitions
; such as completing orbital navigation and starting entry preparations.

POSTJUMP	XCH	Q		# SAVE INCOMING C(A).
		INDEX	A		# GET CADR.
		CA	0

# BANKJUMP IS THE SAME AS POSTJUMP, EXCEPT THAT THE CADR ARRIVES IN A.

; BANKJUMP: One-way bank jump with CADR already in accumulator.
;
; Common code path for both POSTJUMP and programmatic bank jumps.
; Extracts bank number, switches to target bank via FBANK register,
; extracts relative address, and performs unconditional jump (TCF)
; to the target location.

BANKJUMP	TS	FBANK
		MASK	LOW10
		XCH	Q		# RESTORING INPUT C(A) IF THIS WAS A
Q+10000		INDEX	Q		# POSTJUMP.
PRIO12		TCF	10000		# PRIO12 = TCF	10000 = 12000

# Page 1104
# THE FOLLOWING ROUTINE GETS THE RETURN CADR SAVED BY SWCALL OR BANKCALL AND LEAVES IT IN A.

; MAKECADR: Reconstruct the return CADR from BUF2 double-precision storage.
;
; After a BANKCALL, the return address is stored in BUF2 as:
;   BUF2:   Lower 10 bits (relative address within bank)
;   BUF2+1: Bank number in upper bits
;
; This routine combines these components back into a complete CADR in the
; accumulator, useful for routines that need to determine their caller's
; location for logging or dynamic program flow decisions.

MAKECADR	CAF	LOW10
		MASK	BUF2
		AD	BUF2 +1
		TC	Q

; SUPDACAL: Superbank data access - read data from extended memory banks.
;
; The AGC's later Block II configuration includes "superbanks" providing
; additional fixed memory beyond the original 36K. Accessing superbank data
; requires setting both FBANK and SUPERBNK registers.
;
; This routine:
; 1. Saves current FBANK and SUPERBNK state
; 2. Sets target FBANK and SUPERBNK from the CADR in accumulator
; 3. Reads one word from the target address with interrupts disabled
;    (interrupts don't save SUPERBNK, so INHINT/RELINT protect this)
; 4. Restores original FBANK and SUPERBNK
; 5. Returns with data word in accumulator
;
; Note: DCA (double-precision load) prevented here by display interface
; constraints, so only single-word access supported.

SUPDACAL	TS	MPTEMP
		XCH	FBANK		# SET FBANK FOR DATA.
		EXTEND
		ROR	SUPERBNK	# SAVE FBANK IN BITS 15-11, AND
		XCH	MPTEMP		# SUPERBANK IN BITS 7-5.
		MASK	LOW10
		XCH	L		# SAVE REL. ADR. IN BANK, FETCH SUPERBITS.
		INHINT			# BECAUSE RUPT DOES NOT SAVE SUPERBANK.
		EXTEND
		WRITE	SUPERBNK	# SET SUPERBANK FOR DATA.
		INDEX	L
		CA	10000		# PINBALL (FIX MEM DISP) PREVENTS DCA HERE
		XCH	MPTEMP		# SAVE 1ST WD, FETCH OLD FBANK AND SBANK.
		EXTEND
		WRITE	SUPERBNK	# RESTORE SUPERBANK.
		RELINT
		TS	FBANK		# RESTORE FBANK.
		CA	MPTEMP		# RECOVER FIRST WORD OF DATA.
		RETURN			# 24 WDS. DATACALL 516 MU, SUPDACAL 432 MU

; ============================================================================
; INTERRUPT-SAFE BANK CALL ROUTINES
; ============================================================================
;
; IBNKCALL, ISWCALL, ISWRETRN: Interrupt-protected versions of BANKCALL,
; SWCALL, and SWRETURN for use within interrupt service routines.
;
; KEY DIFFERENCE FROM STANDARD ROUTINES:
; These use RUPTREG3 and RUPTREG4 (dedicated interrupt registers) instead of
; BUF2 for saving the return address. This prevents conflict when an interrupt
; occurs during a BANKCALL in the main program:
;
;   Main program executes:  TC BANKCALL (saves return in BUF2)
;   Interrupt occurs:       ISR needs to call another bank
;   ISR executes:          TC IBNKCALL (saves in RUPTREG3, not BUF2)
;   ISR returns:           ISWRETRN restores from RUPTREG3
;   Main program resumes:  SWRETURN still has valid BUF2
;
; USAGE CONTEXT:
; - Timer interrupt handlers (T4RUPT) calling guidance computations
; - Keyboard interrupt handlers (KEYRUPT) calling display routines
; - Downlink interrupt handlers accessing telemetry formatting routines
;
; During Apollo 11's descent, the T4RUPT handler used IBNKCALL to invoke
; landing guidance equations in other banks while the main program was
; executing display updates via BANKCALL. The separate register sets
; prevented the famous 1202 alarm from causing return address corruption.

# Page 1105
# THE FOLLOWING ROUTINES ARE IDENTICAL TO BANKCALL AND SWCALL EXCEPT THAT THEY ARE USED IN INTERRUPT.

IBNKCALL	DXCH	RUPTREG3	# USES RUPTREG3,4 FOR DP RETURN ADDRESS.
		INDEX	Q
		# Was CAF --- RSB 2009
		CA	0
		INCR	Q

ISWCALL		TS	L
		LXCH	FBANK
		MASK	LOW10
		XCH	Q
		DXCH	RUPTREG3
		INDEX	Q
		TC	10000

ISWRETRN	XCH	RUPTREG4
		XCH	FBANK
		XCH	RUPTREG4
		TC	RUPTREG3

; ============================================================================
; USPRCADR: Access interpretive code in another bank
; ============================================================================
;
; This routine enables the interpreter (the AGC's virtual machine for
; high-level vector/matrix operations) to execute interpretive instructions
; located in a different bank than the current BBANK setting.
;
; CALLING SEQUENCE:
;   L:   TC   USPRCADR
;   L+1: CADR INTPRETX    ; Address of interpretive code
;                         ; Return is to L+2 when interpretive code exits
;
; MECHANISM:
; 1. Saves accumulator to LOC
; 2. Sets up EXIT instruction in EDOP (so interpretive code can return)
; 3. Saves current BBANK (interpreter's bank) to BANKSET for later restore
; 4. Switches FBANK to the target bank containing the interpretive code
; 5. Jumps to the interpretive code sequence
; 6. When interpretive code executes EXIT, control returns to L+2
;
; USAGE CONTEXT:
; Guidance and navigation routines frequently use interpretive language for
; orbital mechanics calculations. During Apollo 11, the landing guidance
; (written in interpretive code) was spread across multiple banks. USPRCADR
; allowed the guidance executive to call trajectory computation subroutines
; regardless of which bank they resided in, maintaining modularity while
; working within the AGC's 2K bank size constraint.

# 2. USPRCADR ACCESSES INTERPRETIVE CODING IN OTHER THAN THE USER'S FBANK.  THE CALLING SEQUENCE IS AS FOLLOWS:
#	L	TC	USPRCADR
#	L+1	CADR	INTPRETX	# INTPRETX IS THE INTERPRETIVE CODING
#					# RETURN IS TO L+2

USPRCADR	TS	LOC		# SAVE A
		CA	BIT8
		TS	EDOP		# EXIT INSTRUCTION TO EDOP
		CA	BBANK
		TS	BANKSET		# USER'S BBANK TO BANKSET
		INDEX	Q
		CA	0
		TS	FBANK		# INTERPRETIVE BANK TO FBANK
		MASK	LOW10		# YIELDS INTERPRETIVE RELATIVE ADDRESS
		XCH	Q		# INTERPRETIVE ADDRESS TO Q, FETCHING L+1
		XCH	LOC		# L+1 TO LOC, RETRIEVING ORIGINAL A
		TCF	Q+10000

; ============================================================================
; SUPERSW: Superbank switching routine
; ============================================================================
;
; This routine switches the AGC's superbank setting to access extended fixed
; memory beyond the basic 32K words. The AGC's memory architecture includes:
;
; FIXED MEMORY ORGANIZATION:
; - Banks 00-27: Always accessible (lower fixed memory, 28 banks × 2K = 56K words)
; - Banks 30-37: Superbank 3 (default, 8 banks × 2K = 16K words)
; - Banks 40-47: Superbank 4 (extended, 8 banks × 2K = 16K words)
; - Banks 50-57: Superbank 5 (reserved, not user-accessible)
; - Banks 60-67: Superbank 6 (reserved, not user-accessible)
;
; The superbank bits (bits 7-6-5 of the control word) are written to hardware
; channel 07 to physically switch which set of 8 banks appears in the upper
; fixed memory address space (2000-3777 octal per bank).
;
; CALLING SEQUENCE:
;   CAF  SUPER100      ; Load desired superbank setting (e.g., SUPER100 = octal 100)
;   TC   SUPERSW       ; Switch to that superbank
;   ; ... access banks 40-47 ...
;
; Or with BBCON (combined bank and superbank):
;   CAF  ABBCON         ; ABBCON contains BBCON with superbank bits
;   TC   SUPERSW        ; Superbank bits extracted and written to channel 07
;
; USAGE RESTRICTION:
; *** THIS ROUTINE MUST BE CALLED ONLY FROM BANKS 00-27 ***
; Programs residing in superbanks (30+) must NOT call SUPERSW, as the
; bank switching would invalidate the return address in the Q register.
;
; HISTORICAL CONTEXT:
; Apollo 11's software grew to exceed the original 36K fixed memory capacity.
; Superbanking extended available program storage to accommodate additional
; mission programs, including rendezvous targeting and contingency procedures.
; The Command Module AGC used Superbank 4 (banks 40-43) for extended mission
; functionality, while Superbank 3 remained the default for core flight software.

# Page 1106
# THERE ARE FOUR POSSIBLE SETTINGS FOR CHANNEL 07.  (CHANNEL 07 CONTAINS SUPERBANK SETTING.)
#
#					PSEUDO-FIXED	 OCTAL PSEUDO
# SUPERBANK	SETTING	S-REG. VALUE	BANK NUMBERS	 ADDRESSES
# ----------	-------	------------	------------ 	  ------------
# SUPERBANK 3	  0XX	2000 - 3777	   30 - 37	 70000 - 107777		(WHERE XX CAN BE ANYTHING AND
#										WILL USUALLY BE SEEN AS 11)
# SUPERBANK 4	  100	2000 - 3777	   40 - 47	 110000 - 127777	(AS FAR AS IT CAN BE SEEN,
#										ONLY BANKS 40-43 WILL EVER BE
#										AND ARE PRESENTLY AVAILABLE)
# SUPERBANK 5	  101	2000 - 3777	   50 - 57	 130000 - 147777	(PRESENTLY NOT AVAILABLE TO
#										THE USER)
# SUPERBANK 6	  110	2000 - 3777	   60 - 67	 150000 - 167777	(PRESENTLY NOT AVAILABLE TO
#										THE USER)
# ***  THIS ROUTINE MAYBE CALLED BY ANY PROGRAM LOCATED IN BANKS 00 - 27.  I.E., NO PROGRAM LIVING IN ANY
# SUPERBANK SHOULD USE SUPERSW.  ***
#
# SUPERSW MAYBE CALLED IN THIS FASHION:
#	CAF	ABBCON		WHERE  --  ABBCON   BBCON  SOMETHIN  --
#	TCR	SUPERSW		(THE SUPERBNK BITS ARE IN THE BBCON)
#	...	  ...
#	 .	   .
#	 .	   .
# OR IN THIS FASHION :
#	CAF	SUPERSET	WHERE SUPERSET IS ONE OF THE FOUR AVAILABLE
#	TCR	SUPERSW		SUPERBANK BIT CONSTANTS:
#	...	  ...			SUPER011 OCTAL  60
#	 .	   .			SUPER100 OCTAL 100
#	 .	   .			SUPER101 OCTAL 120
#					SUPER110 OCTAL 140

SUPERSW		EXTEND
		WRITE	SUPERBNK	# WRITE BITS 7-6-5 OF THE ACCUMULATOR INTO
					# CHANNEL 07
		TC	Q		# TC TO INSTRUCTION FOLLOWING
					# 	TC SUPERSW



