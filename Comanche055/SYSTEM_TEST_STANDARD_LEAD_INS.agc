# Copyright:	Public domain.
# Filename:	SYSTEM_TEST_STANDARD_LEAD_INS.agc
# Purpose:	Part of the source code for Comanche, build 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 420-422
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Mod history:	05/07/09 OH	Transcription Batch 1 Assignment
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  April 1, 1969.
#
#	This AGC program shall also be referred to as Colossus 2A
#
#	Prepared by
#			Massachusetts Institute of Technology
#			75 Cambridge Parkway
#			Cambridge, Massachusetts
#
#	under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further information.
# Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: SYSTEM_TEST_STANDARD_LEAD_INS.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases (system verification)
;
; TL;DR: System self-test entry points and test sequence organization defining
;        AGC self-check procedures. Provides test coverage for computer hardware,
;        instruction execution, memory integrity, and I/O operations enabling
;        system health verification throughout Apollo 11 mission.
;
; COMMENT-ONLY READERS: This file defined entry points for computer self-test
;        routines that verified the AGC was working correctly throughout the
;        mission, ensuring reliable operation during critical flight phases.
; CODE-ALONG READERS: Study self-test entry point organization, erasable memory
;        utility routines (E/BKCALL, E/CALL, E/JOBWAK), and test sequence
;        structure connecting to AGC_BLOCK_TWO_SELF-CHECK.agc.
; ============================================================================


# Page 420
		EBANK=	XSM

		BANK	33
		SETLOC	E/PROG1
		BANK

		COUNT*	$$/P07

; ============================================================================
; ERASABLE MEMORY UTILITY ROUTINES
;
; The AGC architecture divides memory into banks requiring special handling
; when programs in erasable (RAM) memory need to call routines in fixed
; (ROM) memory banks. These utility routines simplify such cross-bank calls.
;
; These routines were essential for system test programs that needed to
; execute self-check procedures while residing in erasable memory, enabling
; verification of AGC health without consuming precious fixed memory space.
; ============================================================================

# SPECIAL PROGRAMS TO EASE THE PANGS OF ERASABLE MEMORY PROGRAMS.
#
# E/BKCALL	FOR DOING BANKCALLS FROM AND RETURNING TO ERASABLE.
#
# THIS ROUTINE IS CALLABLE FROM ERASABLE OR FIXED.  LIKE BANKCALL, HOWEVER, SWITCHING BETWEEN S3 AND S4
# IS NOT POSSIBLE.
#
# THE CALLING SEQUENCE IS:
#
#	TC	BANKCALL
#	CADR	E/BKCALL
#	CADR	ROUTINE		# WHERE YOU WANT TO GO IN FIXED.
#	RETURN HERE FROM DISPLAY TERMINATE, BAD STALL OR TC Q.
#	RETURN HERE FROM DISPLAY PROCEED OR GOOD RETURN FROM STALL.
#	RETURN HERE FROM DISPLAY ENTER OR RECYCLE.
#
# THIS ROUTINE REQUIRES TWO ERASABLES (EBUF2, +1) IN UNSWITCHED WHICH ARE UNSHARED BY INTERRUPTS AND
# OTHER EMEMORY PROGRAMS.
#
# A + L ARE PRESERVED THROUGH BANKCALL AND E/BKCALL.

; E/BKCALL - ERASABLE BANK CALL UTILITY
; Enables programs residing in erasable memory to call fixed memory routines
; and return properly to erasable addresses. Critical for self-test programs.
; Preserves A and L registers across the call, maintaining computational state.

E/BKCALL	DXCH	BUF2		# SAVE A,L AND GET DP RETURN.
		DXCH	EBUF2		# SAVE DP RETURN.
		INCR	EBUF2		# RETURN +1 BECAUSE DOUBLE CADR.
		
; Construct bank call context preserving current erasable bank number.
; The AGC uses bank switching to address more than 2K words of memory.
; This sequence captures the current bank state so return can restore it.

		CA	BBANK
		MASK	LOW10		# GET CURRENT EBANK.  (SBANK SOMEDAY)
		ADS	EBUF2	+1	# FORM BBCON.  (WAS FBANK)
		
; Execute the target routine in fixed memory via SWCALL.
; The called routine address comes from the CADR following the E/BKCALL call.

		NDX	EBUF2
		CA	0 -1		# GET CADR OF ROUTINE.
		TC	SWCALL		# GO TO ROUTINE, SETTING Q TO SWRETURN
					# AND RESTORING A + L.
					
; Multiple return paths support different DSKY display responses.
; Self-test programs use these paths to handle crew interaction during testing.

		TC	+4		# TX Q, V34, OR BAD STALL RETURN.
		TC	+2		# PROCEED OR GOOD STALL RETURN.
		INCR	EBUF2		# ENTER OR RECYCLE RETURN.
		INCR	EBUF2
		
; E/SWITCH - Return to erasable memory restoring proper bank context.

E/SWITCH	DXCH	EBUF2
		DTCB

# Page 421
# E/CALL	FOR CALLING A FIXED MEMORY INTERPRETIVE SUBROUTINE FROM ERASABLE AND RETURNING TO ERASABLE.
#
# THE CALLING SEQUENCE IS...
#
#	RTB
#		E/CALL
#	CADR	ROUTINE			# THE INTERPRETIVE SUBROUTINE YOU WANT.
#					# RETURNS HERE IN INTERPRETIVE.

; E/CALL - ERASABLE INTERPRETIVE CALL UTILITY
; Enables interpretive language programs in erasable memory to call
; interpretive subroutines residing in fixed memory banks. The AGC interpreter
; (a virtual machine for vector/matrix math) uses different instruction format
; than native AGC code, requiring special call handling across memory types.

E/CALL		LXCH	LOC		# ADRES -1 OF CADR.
		INDEX	L
		CA	L		# CADR IN A.
		INCR	L
		INCR	L		# RETURN ADRES IN L.
		
; Store target subroutine address and return address in EBUF2 buffer.
; This allows the interpretive call to execute indirectly while preserving
; the erasable return context.

		DXCH	EBUF2		# STORE CADR AND RETURN.
		
; Enter interpretive mode and execute the target subroutine.
; The called routine must exit via RVQ (Return Via Q) or equivalent
; interpretive return instruction to properly restore execution context.

		TC	INTPRET
		CALL
			EBUF2		# INDIRECTLY EXECUTE ROUTING.  IT MUST
		EXIT			# LEAVE VIA RVQ OR EQUIVALENT.
		
; Restore return address and resume interpretive execution in erasable memory.

		LXCH	EBUF2 +1	# PICK UP RETURN.
		TCF	INTPRET +2	# SET LOC AND RETURB TO CALLER

# Page 422
# E/JOBWAK	FOR WAKING UP ERASABLE MEMORY JOBS.
#
# THIS ROUTINE MUST BE CALLED IN INTERRUPT OR WITH INTERRUPTS INHIBITED.
#
# THE CALLING SEQUENCE IS:
#
#	INHINT
#	  .
#	  .
#	CA	WAKEADR		# ADDRESS OF SLEEPING JOB
#	TC	IBNKCALL
#	CADR	E/JOBWAK
#	  .			# RETURNS HERE
#	  .
#	  .
#	RELINT			# IF YOU DID AN INHINT.

		BANK	33
		SETLOC	E/PROG
		BANK

		COUNT*	 $$/P07

; E/JOBWAK - ERASABLE JOB WAKEUP UTILITY
; Wakes up executive jobs residing in erasable memory from sleeping state.
; Must execute with interrupts inhibited (INHINT) to prevent race conditions
; in the executive scheduler's job queue. Used by self-test routines to
; coordinate test sequence execution with other AGC operations.

E/JOBWAK	TC	JOBWAKE		# ARRIVE IWTH ADRES IN A.
		
; Clear the fixed memory bit (bit 11) from the job address since the job
; resides in erasable memory. AGC addresses use bit 11 to distinguish
; between fixed (ROM) and erasable (RAM) memory locations.

		CS	BIT11
		NDX	LOCCTR
		ADS	LOC		# KNOCK FIXED MEMORY BIT OUT OF ADRES.
		TC	RUPTREG3	# RETURN


; ============================================================================
; SYSTEM TEST STANDARD LEAD-IN ENTRY POINTS
;
; This section provides standardized entry points at fixed addresses for
; system self-check and prelaunch alignment programs. These entry points
; enable System Test Guidance (STG) and hybrid simulation labs to execute
; comprehensive AGC verification procedures with the Colossus/Comanche
; flight software.
;
; The entry points are positioned at specific memory addresses (33,2000 and
; 33,2001) to maintain compatibility with test equipment and ground support
; procedures developed for earlier AGC program versions. During Apollo 11
; mission preparation, ground controllers used these entry points to verify
; Command Module computer health before launch and during mission phases.
; ============================================================================

# THESE PROGRAMS ARE PROVIDED TO ALLOW OVERLAY OF BANKS 30 THRU 33 OF THE 205 VERSIONS OF SYSTEM TESTS AND
# PRELAUNCH ALIGN.  THE INTENT IS TO ALLOW THE STG AND HYBRID LABS TO RUN ALL THE TESTS WITH COLOSSUS.


		BANK	33
		SETLOC	TESTLEAD
		BANK

		COUNT	33/COMST

		EBANK=	QPLACE

; COMPVER - COMPUTER VERIFICATION ENTRY POINT
; Standard entry point at address 33,2000 for initiating computer verification
; self-check procedures. Routes to GCOMPVER which executes comprehensive AGC
; hardware and software validation including erasable memory tests, fixed
; memory checksum verification, and instruction execution validation.
; Used extensively during prelaunch checkout and mission readiness verification.

COMPVER		TC	GCOMPVER	# MUST BE 33,2000.

; GTSCPSS1 - GET SYSTEM TEST SELF-CHECK PASS ENTRY POINT  
; Standard entry point at address 33,2001 for system test control and pass
; sequencing. Routes to GTSCPSS which manages test execution flow, result
; reporting, and coordination with ground support equipment during comprehensive
; system verification campaigns.

GTSCPSS1	TC	GTSCPSS		# MUST BE AT 33,2001

; REDO - RESTART TEST SEQUENCE ENTRY POINT
; Enables test restart and continuation by displaying Major Mode 07 on the
; DSKY and transitioning to IMU (Inertial Measurement Unit) performance tests.
; Provides recovery mechanism if test sequences are interrupted or if specific
; test phases need repetition during troubleshooting.

REDO		TC	NEWMODEX	# DISPLAY MM 07.
		MM	07		# FALL INTO IMUTEST

