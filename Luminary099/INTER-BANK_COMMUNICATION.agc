# Copyright:	Public domain.
# Filename:	INTER-BANK_COMMUNICATION.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	998-1001
# Mod history:	2009-05-24 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-05-08 JL	Removed workaround.

; ============================================================================
; FILE: INTER-BANK_COMMUNICATION.agc
; MODULE: Core Operating System - Memory Management
; MISSION PHASE: All phases (foundation for all mission operations)
;
; TL;DR: Implements memory bank switching routines enabling the AGC to access
;        code and data beyond its limited directly-addressable memory space.
;        The AGC has 36K words of fixed (ROM) memory divided into banks, but
;        can only directly address 2K words at a time. These routines manage
;        cross-bank subroutine calls, preserving registers and return addresses.
;
; COMMENT-ONLY READERS: This code enables the Lunar Module's computer to use
;        all its available memory by switching between memory "banks" as needed
;        during mission operations.
; CODE-ALONG READERS: Study the BANKCALL/SWCALL mechanism for understanding
;        AGC memory architecture, bank switching overhead, and CADR addressing.
; ============================================================================

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

# Page 998
# 	   THE FOLLOWING ROUTINE CAN BE USED TO CALL A SUBROUTINE IN ANOTHER BANK. IN THE BANKCALL VERSION, THE
# CADR OF THE SUBROUTINE IMMEDIATELY FOLLOWS THE  TC BANKCALL  INSTRUCTION, WITH C(A) AND C(L) PRESERVED.

; ============================================================================
; BANKCALL - Cross-Bank Subroutine Call Routine
;
; The AGC's memory architecture divides 36K words of fixed (core rope) memory
; into banks. Only one bank can be directly addressed at a time (the current
; FBANK setting determines which 1K-word bank is accessible at addresses
; 02000-03777 octal). BANKCALL enables calling subroutines in other banks.
;
; A CADR (Combined Address) contains both the bank number and the relative
; address within that bank, allowing routines to reference code anywhere in
; fixed memory. During Apollo 11's mission, every guidance calculation, every
; display update, and every control system command relied on this mechanism
; to access the full AGC program.
;
; Calling sequence:
;        TC    BANKCALL
;        CADR  TARGETROUTINE    ; Combined address (bank + offset)
;        ...                    ; Returns here after TARGETROUTINE completes
;
; The BANKCALL preserves A and L registers across the call, switches to the
; target bank, executes the subroutine, then restores the original bank and
; register contents upon return.
; ============================================================================

		BLOCK	02
		COUNT*	$$/BANK
BANKCALL	DXCH	BUF2		# SAVE INCOMING A,L.
		INDEX	Q		# PICK UP CADR.
		CA	0
		INCR	Q		# SO WE RETURN TO THE LOC. AFTER THE CADR.

# 	   SWCALL IS IDENTICAL TO BANKCALL, EXCEPT THAT THE CADR ARRIVES IN A.

; SWCALL - Switch and Call (CADR in Accumulator)
;
; This is the common entry point for both BANKCALL (which fetches the CADR
; from memory) and direct SWCALL usage (where the CADR is already in the A
; register). The routine extracts the bank number from the CADR, switches to
; that bank, and transfers control to the target address.
;
; Technical details: FBANK register holds current bank number. The low 10 bits
; of the CADR contain the relative address within the bank (0-1777 octal,
; mapping to absolute addresses 02000-03777). The upper bits specify the bank.

SWCALL		TS	L
		LXCH	FBANK		# SWITCH BANKS, SAVING RETURN.
		MASK	LOW10		# GET SUB-ADDRESS OF CADR.
		XCH	Q		# A,L NOW CONTAINS DP RETURN.
		DXCH	BUF2		# RESTORING INPUTS IF THIS IS A BANKCALL.
		INDEX	Q
		TC	10000		# SETTING Q TO SWRETURN.

; SWRETURN - Return from Cross-Bank Call
;
; When a cross-bank subroutine completes, it returns here to restore the
; original bank context. The return address (bank + offset) was saved in
; BUF2 and BUF2+1 by the calling sequence. This routine switches back to
; the caller's bank and transfers control to the instruction following the
; original BANKCALL or SWCALL.
;
; During Apollo 11's descent, this mechanism enabled the landing guidance
; routines to call navigation subroutines, which in turn called mathematical
; subroutines in other banks - all transparent to the programmer.

SWRETURN	XCH	BUF2 	+1	# COMES HERE TO RETURN TO CALLER. C(A,L)
		XCH	FBANK		# ARE PRESERVED FOR RETURN.
		XCH	BUF2 	+1
		TC	BUF2

# 	   THE FOLLOWING ROUTINE CAN BE USED AS A UNILATERAL JUMP WITH C(A,L) PRESERVED AND THE CADR IMMEDIATELY
# FOLLOWING THE TC POSTJUMP INSTRUCTION.

; ============================================================================
; POSTJUMP and BANKJUMP - Cross-Bank Unconditional Transfer
;
; Unlike BANKCALL which expects to return to the caller, POSTJUMP and BANKJUMP
; perform one-way transfers to code in another bank. This is used for program
; phase transitions where the current program terminates and a new program
; begins in a different bank.
;
; POSTJUMP fetches the CADR from the memory location following the TC POSTJUMP
; instruction. BANKJUMP expects the CADR already loaded in the A register.
; Both preserve the contents of A and L registers across the jump.
;
; During Apollo 11's mission, these jumps occurred at major phase transitions:
; switching from orbit navigation to descent programs, or from landing programs
; to post-landing housekeeping routines.
; ============================================================================

POSTJUMP	XCH	Q		# SAVE INCOMING C(A).
		INDEX	A		# GET CADR.
		CA	0

# 	   BANKJUMP IS THE SAME AS POSTJUMP, EXCEPT THAT THE CADR ARRIVES IN A.

BANKJUMP	TS	FBANK
		MASK	LOW10
		XCH	Q		# RESTORING INPUT C(A) IF THIS WAS A
Q+10000		INDEX	Q		# POSTJUMP.
PRIO12		TCF	10000		# PRIO12 = TCF	 10000 = 12000

# Page 999
# 	   THE FOLLOWING ROUTINE GETS THE RETURN CADR SAVED BY SWCALL OR BANKCALL AND LEAVES IT IN A.

; MAKECADR - Reconstruct Return Address
;
; This utility routine reconstructs the full CADR (bank + address) from the
; return address components saved in BUF2 and BUF2+1 during a BANKCALL or
; SWCALL. The resulting CADR is left in the A register.
;
; This is useful when a subroutine needs to examine or modify its return
; address, or when implementing higher-level control flow mechanisms.

MAKECADR	CAF	LOW10
		MASK	BUF2
		AD	BUF2 	+1
		TC	Q

; ============================================================================
; SUPDACAL - Superbank Data Call
;
; The AGC Block II has extended memory addressing through "superbanks" which
; allow access to memory beyond the basic 36K words. SUPDACAL enables reading
; data from these extended memory regions while properly managing both the
; FBANK (fixed bank) and SUPERBNK (superbank) registers.
;
; This routine:
; 1. Saves current FBANK and SUPERBNK settings
; 2. Sets FBANK and SUPERBNK to access the target data location
; 3. Reads one word from the specified address
; 4. Restores original FBANK and SUPERBNK settings
; 5. Returns with the data word in A register
;
; Interrupts are inhibited during superbank switching because interrupt
; handling does not automatically save/restore the SUPERBNK register, and
; an interrupt during the switch could corrupt the superbank state.
;
; Performance: 432 machine units (vs 516 for DATACALL). During Apollo 11's
; mission, efficient data access was critical for real-time guidance updates.
; ============================================================================

SUPDACAL	TS	MPTEMP
		XCH	FBANK		# SET FBANK FOR DATA.
		EXTEND
		ROR	SUPERBNK	# SAVE FBANK IN BITS 15-11, AND
		XCH	MPTEMP		#  SUPERBANK IN BITS  7-5.
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

# Page 1000
# 	   THE FOLLOWING ROUTINES ARE IDENTICAL TO BANKCALL AND SWCALL EXCEPT THAT THEY ARE USED IN INTERRUPT.

; ============================================================================
; IBNKCALL, ISWCALLL, ISWRETRN - Interrupt-Safe Bank Call Routines
;
; During Apollo 11's mission, interrupts occurred frequently: every 10
; milliseconds for T4RUPT (timer), plus additional interrupts for keyboard
; input (KEYRUPT), uplink data (UPRUPT), and radar data (RADARRUPT).
;
; When interrupt service routines need to call subroutines in other banks,
; they cannot use the normal BANKCALL/SWCALL routines because those use
; BUF2/BUF2+1 for the return address, and those locations might already
; contain the return address from an interrupted BANKCALL in the mainline code.
;
; The interrupt-safe versions use RUPTREG3 and RUPTREG4 instead, which are
; reserved specifically for interrupt handling. This prevents corruption of
; return addresses when an interrupt occurs during a cross-bank call.
;
; IBNKCALL: Interrupt Bank Call - fetches CADR from following instruction
; ISWCALLL: Interrupt Switch Call - CADR arrives in A register
; ISWRETRN: Interrupt Switch Return - returns to interrupted routine
;
; These routines were critical during the 1202 program alarms at 102:38:26
; mission time, when interrupt-driven radar processing was competing with
; executive job scheduling for computational resources.
; ============================================================================

IBNKCALL	DXCH	RUPTREG3	# USES RUPTREG3,4 FOR DP RETURN ADDRESS.
		INDEX	Q
		CAF	0
		INCR	Q

ISWCALLL	TS	L
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

# 2. USPRCADR ACCESSES INTERPRETIVE CODING IN OTHER THAN THE USER'S FBANK.  THE CALLING SEQUENCE IS AS FOLLOWS:

# L		TC	USPRCADR
# L+1		CADR	INTPRETX	  INTPRETX IS THE INTERPRETIVE CODING
#					  RETURN IS TO L+2

; ============================================================================
; USPRCADR - User Procedure Address Call to Interpretive Code
;
; The AGC has two execution modes: native AGC assembly instructions and the
; interpretive language (a virtual machine for high-level vector/matrix math).
; USPRCADR enables a native AGC routine to call interpretive code located in
; a different bank, then return to native code upon completion.
;
; The interpretive language was crucial during Apollo 11's guidance
; computations - calculating landing trajectories, orbital mechanics, and
; navigation state updates. Most guidance equations were written in interpretive
; language because it provided double-precision arithmetic, vector operations,
; and trigonometric functions that would be prohibitively complex in native AGC.
;
; Calling sequence:
;        TC    USPRCADR
;        CADR  TARGETINTERPRETIVE    ; Combined address of interpretive routine
;        ...                          ; Returns here in native mode
;
; This routine sets up FBANK to access the target interpretive code, configures
; the exit instruction (EDOP) to return to native mode, and preserves the
; calling context for proper return.
; ============================================================================

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

# Page 1001
# THERE ARE FOUR POSSIBLE SETTINGS FOR CHANNEL 07. (CHANNEL 07 CONTAINS SUPERBANK SETTING.)
#					    PSEUDO-FIXED      OCTAL PSEUDO
# SUPERBANK	SETTING	    S-REG. VALUE    BANK NUMBERS      ADDRESSES
# ----------	-------	    ------------     ------------      ------------
#
# SUPERBANK 3	  OXX	    2000 - 3777	       30 - 37	      70000 - 107777	(WHERE XX CAN BE ANYTHING AND
#										WILL USUALLY BE SEEN AS 11)
# SUPERBANK 4	  100	    2000 - 3777        40 - 47	      110000 - 127777	(AS FAR AS IT CAN BE SEEN,
#										ONLY BANKS 40-43 WILL EVER BE
#										AND ARE PRESENTLY AVAILABLE)
# SUPERBANK 5	  101	    2000 - 3777	       50 - 57	      130000 - 147777	(PRESENTLY NOT AVAILABLE TO
#										THE USER)
# SUPERBANK 6	  110	    2000 - 3777	       60 - 67	      150000 - 167777	(PRESENTLY NOT AVAILABLE TO
#										THE USER)
# ***  THIS ROUTINE MAYBE CALLED BY ANY PROGRAM LOCATED IN BANKS 00 - 27.  I.E., NO PROGRAM LIVING IN ANY
# SUPERBANK SHOULD USE SUPERSW.  ***
#
# SUPERSW MAYBE CALLED IN THIS FASHION:
#
#	   CAF	  ABBCON	  WHERE  --  ABBCON   BBCON  SOMETHIN  --
#	   TCR	  SUPERSW	  (THE SUPERBNK BITS ARE IN THE BBCON)
#	   ...	    ...
#	    .	     .
#	    .	     .
# OR IN THIS FASHION :
#	   CAF	  SUPERSET	  WHERE SUPERSET IS ONE OF THE FOUR AVAILABLE
#	   TCR	  SUPERSW	  SUPERBANK BIT CONSTANTS:
#	   ...	    ...					  SUPER011 OCTAL  60
#	    .	     .					  SUPER100 OCTAL 100
#	    .	     .					  SUPER101 OCTAL 120
#							  SUPER110 OCTAL 140

; ============================================================================
; SUPERSW - SUPERBANK SWITCH ROUTINE
;
; This routine allows programs to directly set the superbank register,
; enabling access to extended memory banks 30-67. The AGC's superbank
; mechanism extends the fixed memory address space beyond the standard banks.
;
; COMMENT-ONLY READERS: The Lunar Module's computer memory is organized into
; multiple "superbanks" providing additional storage. This routine switches
; which superbank is active, allowing access to different areas of memory.
;
; CODE-ALONG READERS: The superbank setting is stored in bits 7-6-5 of the
; accumulator and written to hardware channel 07 (SUPERBNK). Four superbank
; values are available (3, 4, 5, 6) corresponding to bank ranges 30-37,
; 40-47, 50-57, and 60-67 respectively. This routine must only be called
; from banks 00-27 to avoid conflicts.
;
; USAGE: Load A with superbank bits (constants: SUPER011=60, SUPER100=100,
;        SUPER101=120, SUPER110=140), then TC SUPERSW. Return is to Q.
; ============================================================================

SUPERSW		EXTEND
		WRITE	SUPERBNK	# WRITE BITS 7-6-5 OF THE ACCUMULATOR INTO
					# CHANNEL 07
		TC	Q		# TC TO INSTRUCTION FOLLOWING
					#   TC  SUPERSW

