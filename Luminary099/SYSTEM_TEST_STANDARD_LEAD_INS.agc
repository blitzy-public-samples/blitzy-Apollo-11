# Copyright:	Public domain.
# Filename:	SYSTEM_TEST_STANDARD_LEAD_INS.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	370-372
# Mod history:	2009-05-17 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
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
; FILE: SYSTEM_TEST_STANDARD_LEAD_INS.agc
; MODULE: System Test Infrastructure
; MISSION PHASE: All phases (background diagnostics)
;
; TL;DR: Provides standardized entry point routines enabling erasable memory
;        programs to interface with system self-check and test facilities.
;        Includes bankcall support, interpretive subroutine invocation, and
;        job wakeup mechanisms for programs executing from RAM rather than
;        core rope ROM. Critical for AGC self-diagnostic operations.
;
; COMMENT-ONLY READERS: These routines enable the guidance computer to test
;        itself continuously during flight, ensuring reliability.
; CODE-ALONG READERS: Study memory bank switching mechanics and erasable-to-
;        fixed memory calling conventions unique to AGC architecture.
; ============================================================================

# Page 370
		EBANK=	XSM

		BANK	33
		SETLOC	E/PROG
		BANK

		COUNT*	$$/P07

; ============================================================================
; ERASABLE MEMORY PROGRAM SUPPORT ROUTINES
;
; The AGC architecture divides memory into "fixed" (core rope ROM, 36K words)
; and "erasable" (RAM, 2K words). Most programs execute from fixed memory,
; but system test and diagnostic routines often run from scarce erasable
; memory to preserve ROM space. These utilities provide standardized calling
; conventions for erasable memory programs to invoke fixed memory subroutines
; and manage job scheduling.
;
; This infrastructure enables the AGC self-check program (V21N27) to
; continuously validate computer operation during flight without requiring
; dedicated ROM banks. During Apollo 11's mission, these routines enabled
; background diagnostics to run during coast phases and on the lunar surface.
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
#	CADR	ROUTINE		# WHERE TO WANT TO GO IN FIXED.
#	RETURN HERE FROM DISPLAY TERMINATE, BAD STALL OR TC Q.
#	RETURN HERE FROM DISPLAY PROCEED OR GOOD RETURN FROM STALL.
#	RETURN HERE FROM DISPLAY ENTER OR RECYCLE.
#
# THIS ROUTINE REQUIRES TWO ERASABLES (EBUF2, +1) IN UNSWITCHED WHICH ARE UNSHARED BY INTERRUPTS AND
# OTHER EMEMORY PROGRAMS.
#
# A + L ARE PRESERVED THROUGH BANKCALL AND E/BKCALL.

; E/BKCALL: Erasable Memory Bankcall Facility
;
; Purpose: Enables programs running from erasable (RAM) memory to call
; subroutines in fixed (ROM) memory and return safely to erasable. Standard
; BANKCALL mechanism assumes fixed-to-fixed transitions; E/BKCALL handles
; the more complex erasable-to-fixed-to-erasable call chain.
;
; Technical Context: AGC memory banking requires explicit bank number tracking.
; When calling from erasable memory (which has its own bank structure), the
; return address must encode both the erasable bank number and address within
; that bank. E/BKCALL constructs proper "BBCON" (bank-address combination)
; for erasable return addresses.
;
; Return Conventions: Three return points support DSKY display interactions:
; - Return+0: Display TERMINATE, bad stall, or TC Q (normal completion)
; - Return+1: Display PROCEED or good stall return
; - Return+2: Display ENTER or recycle request
;
; Register Preservation: A and L registers preserved across call, enabling
; test routines to maintain computational state through bank transitions.

; Entry point: Preserve registers and capture return address
E/BKCALL	DXCH	BUF2		# SAVE A,L AND GET DP RETURN.
		DXCH	EBUF2		# SAVE DP RETURN.
		INCR	EBUF2		# RETURN +1 BECAUSE DOUBLE CADR.

; Build erasable bank return address (BBCON format)
; AGC erasable memory organized in banks; return address must encode
; both bank number and address within bank for proper return from fixed.
		CA	BBANK
		MASK	LOW10		# GET CURRENT EBANK.  (SBANK SOMEDAY)
		ADS	EBUF2	+1	# FORM BBCON.  (WAS FBANK)

; Retrieve target subroutine address and transfer control
		NDX	EBUF2
		CA	0 	-1	# GET CADR OF ROUTINE.
		TC	SWCALL		# GO TO ROUTINE, SETTING Q TO SWRETURN
					# AND RESTORING A + L.

; Three-way return dispatch based on subroutine completion status
; Supports DSKY display verb interactions during system test operations
		TC	+4		# TX Q, V34, OR BAD STALL RETURN.
		TC	+2		# PROCEED OR GOOD STALL RETURN.
		INCR	EBUF2		# ENTER OR RECYCLE RETURN.
		INCR	EBUF2

; Return to erasable memory caller using BBCON stored in EBUF2
E/SWITCH	DXCH	EBUF2
		DTCB			# Double Transfer Control to Bank

# Page 371
# E/CALL	FOR CALLING A FIXED MEMORY INTERPRETIVE SUBROUTINE FROM ERASABLE AND RETURNING TO ERASABLE.
#
# THE CALLING SEQUENCE IS...
#
#	RTB
#		E/CALL
#	CADR	ROUTINE			# THE INTERPRETIVE SUBROUTINE YOU WANT.
#					# RETURNS HERE IN INTERPRETIVE.

; ============================================================================
; E/CALL: Erasable-to-Fixed Interpretive Subroutine Invocation
;
; Purpose: Enables interpretive language programs running from erasable memory
; to call interpretive subroutines located in fixed memory and return to
; erasable interpretive execution.
;
; Context: AGC interpretive language is a virtual machine providing high-level
; vector/matrix operations atop the basic AGC instruction set. Interpretive
; code executes more slowly than native AGC code but uses less memory.
; System test programs in interpretive mode use E/CALL to access fixed-memory
; mathematical and navigation subroutines.
;
; Calling Mechanism: RTB (Return to Basic) instruction exits interpreter,
; calls E/CALL in basic AGC mode, which then re-enters interpreter to execute
; the target subroutine before returning to erasable interpretive code.
;
; Return Protocol: Called subroutine must exit via RVQ (Return Via Q) or
; equivalent interpreter return instruction. E/CALL then restores erasable
; interpretive context and resumes execution at return address.
; ============================================================================

; Extract CADR (combined address) of target subroutine from calling sequence
E/CALL		LXCH	LOC		# ADRES -1 OF CADR.
		INDEX	L
		CA	L		# CADR IN A.
		INCR	L
		INCR	L		# RETURN ADRES IN L.
		DXCH	EBUF2		# STORE CADR AND RETURN

; Enter interpreter mode and invoke target subroutine indirectly
		TC	INTPRET		# Transfer to interpreter virtual machine
		CALL
			EBUF2		# INDIRECTLY EXECUTE ROUTINE.  IT MUST
		EXIT			# LEAVE VIA RVQ OR EQUIVALENT.

; Return to erasable interpretive code at saved return address
		LXCH	EBUF2	+1	# PICK UP RETURN.
		TCF	INTPRET	+2	# SET LOC AND RETURN TO CALLER.

# Page 372
# E/JOBWAK	FOR WAKING UP ERASABLE MEMORY JOBS.
#
# THIS ROUTINE MUST BE CALLED IN INTERRUPT OR WITH INTERRUPTS INHIBITED.
#
# THE CALLING SEQUENCE IS:
#
#	INHINT
#	...
#	CA	WAKEADR		# ADDRESS OF SLEEPING JOB
#	TC	IBNKCALL
#	CADR	E/JOBWAK
#	...			# RETURNS HERE
#	RELINT			# IF YOU DID AND INHINT.

		BANK	33
		SETLOC	E/PROG
		BANK

		COUNT*	 $$/P07

; ============================================================================
; E/JOBWAK: Erasable Memory Job Wakeup Facility
;
; Purpose: Wakes up AGC executive jobs that are sleeping in erasable memory,
; enabling test programs and diagnostics stored in RAM to participate in the
; AGC's priority job scheduling system.
;
; Context: AGC executive manages multiple concurrent "jobs" (programs) using
; priority-based cooperative multitasking. Jobs can sleep (suspend execution)
; and be awakened later. Standard JOBWAKE assumes fixed memory job addresses;
; E/JOBWAK handles erasable memory jobs by clearing the fixed-memory address
; bit (bit 11) that would cause bank switching errors.
;
; Interrupt Safety: MUST be called with interrupts inhibited (INHINT) or
; from within interrupt service routine. Job scheduling data structures are
; not protected against concurrent modification; interrupt during wakeup
; could corrupt executive job queues.
;
; Usage: System test routines use E/JOBWAK to coordinate multiple diagnostic
; jobs running from erasable memory, enabling complex multi-phase self-check
; sequences without consuming fixed memory job table entries.
; ============================================================================

; Wake the job and adjust address for erasable memory location
E/JOBWAK	TC	JOBWAKE		# ARRIVE IWTH ADRES IN A.

; Clear bit 11 to remove fixed-memory bank indicator from erasable address
; Bit 11 set indicates fixed memory address requiring bank switching.
; Erasable addresses must have bit 11 clear to prevent incorrect banking.
		CS	BIT11		# Complement of bit 11 (all bits set except 11)
		NDX	LOCCTR
		ADS	LOC		# KNOCK FIXED MEMORY BIT OUT OF ADRES.

; Return to caller (interrupt or INHINT-protected code)
		TC	RUPTREG3	# RETURN

