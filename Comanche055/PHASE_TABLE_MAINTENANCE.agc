# Copyright:	Public domain.
# Filename:	PHASE_TABLE_MAINTENANCE.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1404-1413
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

; ============================================================================
; FILE: PHASE_TABLE_MAINTENANCE.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Mission phase state management tracking current program phase for
;        restart protection. Maintains phase tables enabling AGC restart recovery
;        to resume correct program state, critical for fault tolerance throughout
;        Apollo 11 mission operations from launch through splashdown.
;
; COMMENT-ONLY READERS: This program tracked which phase of each program was
;        running so restarts could resume at the right place after power glitches.
; CODE-ALONG READERS: Study phase table organization, program state tracking,
;        restart phase encoding for recovery coordination with RESTARTS_ROUTINE.
; ============================================================================

# Page 1404
# SUBROUTINE TO UPDATE THE PROGRAM NUMBER DISPLAY ON THE DSKY.

; ============================================================================
; PROGRAM MODE DISPLAY UPDATE ROUTINES
;
; These routines update the major mode display (MODREG) shown to the crew on
; the DSKY. Throughout Apollo 11's mission, crew monitored the major mode
; number (P00-P99) to know which program the AGC was executing. Mode changes
; displayed program transitions like P63 lunar landing, P12 ascent, or P00 idle.
; ============================================================================

		COUNT	02/PHASE
		BLOCK	02
		SETLOC	FFTAG1
		BANK

; NEWMODEX - Update mode register with value from fixed memory
; Entry: Q points to octal constant containing new mode number
; Called when mode number is stored in program code
NEWMODEX	INDEX	Q		# UPDATE MODREG.  ENTRY FOR MODE IN FIXED.
		CAF	0		; Load mode number from address in Q
		INCR	Q		; Increment return address to skip constant

; NEWMODEA - Update mode register with value already in A register
; Entry: A register contains new mode number
; Both paths continue to MMDSPLAY to show mode to crew
NEWMODEA	TS	MODREG		# ENTRY FOR MODE IN A.
MMDSPLAY	CAF	+3		# DISPLAY MAJOR MODE.
PREBJUMP	LXCH	BBANK		# PUTS BBANK IN L
		TCF	BANKJUMP	# PUTS Q INTO A
		CADR	SETUPDSP	; Cross-bank call to display routine

# RETURN TO CALLER +3 IF MODE = THAT AT CALLER +1.  OTHERWISE RETURN TO CALLER +2.

; CHECKMM - Check if current major mode matches expected value
; Used by programs to verify they're running in correct mode before proceeding.
; Entry: Q points to octal constant with expected mode number
; Exit: Q+3 if match (skip error handling), Q+2 if mismatch (execute error code)
CHECKMM		INDEX	Q		; Indexed addressing using return address
		CS	0		; Load and complement expected mode from caller
		AD	MODREG		; Subtract current mode (complement add = subtract)
		EXTEND			; Prepare for branch instruction
		BZF	Q+2		; Branch if zero (match): return to caller+3
		TCF	Q+1		# NO MATCH - return to caller+2

; TCQ - Common return macro equivalent to Q+2+1 (returns to caller+3)
TCQ		=	Q+2 +1

		BANK	14
		SETLOC	PHASETAB
		BANK

		COUNT	10/PHASE

; SETUPDSP - Schedule major mode display job for DSKY update
; This routine cannot execute display updates directly because it may be called
; from interrupt contexts. Instead, it schedules a background job (DSPMMJOB) at
; priority 30 to safely update the DSKY without interfering with time-critical
; operations. Crew saw major mode updates within milliseconds of program transitions.
SETUPDSP	INHINT			; Disable interrupts for critical section
		DXCH	RUPTREG1	# SAVE CALLER-S RETURN 2CADR
		CAF	PRIO30		# 	EITHER A TASK OR JOB CAN COME TO
		TC	NOVAC		#	NEWMODEX - create new job
		EBANK=	MODREG		; Set erasable bank for job
		2CADR	DSPMMJOB	; Job entry point (display major mode job)

		DXCH	RUPTREG1	; Restore caller's return address
		RELINT			; Re-enable interrupts
		DXCH	Z		# RETURN to caller

; DSPMMJOB - Actual display job that updates DSKY (defined in PINBALL routines)
DSPMMJOB	EQUALS	DSPMMJB

		BLOCK	02
# Page 1405
		SETLOC	FFTAG1
		BANK

# Page 1406
# PHASCHNG IS THE MAIN WAY OF MAKING PHASE CHANGES FOR RESTARTS.  THERE ARE THREE FORMS OF PHASCHNG, KNOWN AS TYPE
# A, TYPE B, AND TYPE C.  THEY ARE ALL CALLED AS FOLLOWS, WHERE OCT XXXXX CONTAINS THE PHASE INFORMATION,
#
#		TC	PHASCHNG
#		OCT	XXXXX
#
# TYPE A IS CONCERNED WITH FIXED PHASE CHANGES, THAT IS, PHASE INFORMATION THAT IS STORED PERMANENTLY.  THESE
# OPTIONS ARE, WHERE G STANDS FOR A GROUP AND .X FOR THE PHASE,
#
#	G.0		INACTIVE, WILL NOT PERMIT A GROUP G RESTART
#	G.1		WILL CAUSE THE LAST DISPLAY TO BE REACTIVATED, USED MAINLY IN MANNED FLIGHTS
#	G.EVEN		A DOUBLE TABLE RESTART, CAN CAUSE ANY COMBINATION OF TWO JOBS, TASKS, AND/OR
#			LONGCALL TO BE RESTARTED.
#	G.ODD NOT .1	A SINGLE TABLE RESTART, CAN CAUSE EITHER A JOB, TASK, OR LONGCALL RESTART.
#
# THIS INFORMATION IS PUT INTO THE OCTAL WORD AFTER TC PHASCHNG AS FOLLOWS

; ============================================================================
; TRANSITION: From Display Management to Phase Table Maintenance
;
; The AGC's restart protection system was crucial for Apollo 11's success.
; When power fluctuations or hardware glitches occurred, the computer needed
; to recover and resume operations at the correct program phase. The PHASCHNG
; routine records which phase each program has reached, enabling RESTARTS
; routine to restore correct execution state. This became critical during
; Apollo 11's descent when the 1202 alarm occurred - restart protection
; allowed the computer to recover without aborting the landing.
; ============================================================================
#
#	TL0 00P PPP PPP GGG
#
# WHERE EACH LETTER OR NUMBER STANDS FOR A BIT.  THE G:S STAND FOR THE GROUP, OCTAL 1-7, THE P:S FOR THE PHASE,
# OCTAL 0 - 127.  0:S MUST BE 0.  IF ONE WISHES TO HAVE THE TBASE OF GROUP G TO BE SET AT THIS TIME,
# T IS SET TO 1, OTHERWISE IT IS SET TO 0.  SIMILARLY IF ONE WISHES TO SET LONGBASE, THEN L IS SET TO 1, OTHERWISE
# IT IS SET TO 0.  SOME EXAMPLES,
#
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
#		TC	PHASCHNG	# THIS SETS LONGBASE AND UPON A RESTART
#		OCT	20135		# CAUSES 5.13 TO BE RESTARTED (SINCE
#					# LONGBASE WAS SET THIS SINGLE ENTRY
#					# SHOULD BE A LONGCALL)
#		TC	PHASCHNG	# SINCE BOTH TBASE4 AND LONGBASE ARE SET,
#		OCT	60124		# 4.12 SHOULD CONTAIN BOTH A TASK AND A
#					# LONGCALL TO BE RESTARTED
#
# TYPE C PHASCHNG CONTAINS THE VARIABLE TYPE OF PHASCHNG INFORMATION.  INSTEAD OF THE INFORMATION BEING IN A
# PERMANENT FORM, ONE STORES THE DESIRED RESTART INFORMATION IN A VARIABKE LOCATION.  THE BITS ARE AS FOLLOWS,
#
#	TL0 1AD XXX CJW GGG
#
# WHERE EACH LETTER OR NUMBER STANDS FOR A BIT.  THE G:S STAND FOR THE GROUP, OCTAL 1 - 7.  IF THE RESTART IS TO
# BE BY WAITLIST, W IS SET TO 1, IF IT IS A JOB, J IS SET TO 1, IF IT IS A LONGCALL, C IS SET TO 1.  ONLY ONE OF
# THESE THREE BITS MAY BE SET.  X:S ARE IGNORED  1 MUST BE 1, AND 0 MUST BE 0.  AGAIN T STANDS FOR THE TBASE,
# Page 1407
# AND L FOR LONGBASE.  THE BITS A AND D ARE CONCERNED WITH THE VARIABLE INFORMATION.  IF D IS SET TO 1, A PRIORITY
# OR DELTA TIME WILL BE READ FROM THE NEXT LOCATION AFTER THE OCTAL INFORMATION, IF THIS IS TO BE INDIRECT, THAT
# IS, THE NAME OF A LOCATION COMT+INING THE INFORMATION (DELTA TIME ONLY), THEN THIS IS GIVEN AS THE -GENADR OF
# THAT LOCATION WHICH CONTAINS THE DELTA TIME.  IF THE OLD PRIORITY OR DELTA TIME IS TO BE USED, THAT WHICH IS
# ALREADY IN THE VARIABLE STORAGE, THEN D IS SET TO 0.  NEXT THE A BIT IS USED.  IF IT IS SET TO 0, THE ADDRESS
# THAT WOULD BE RESTARTED DURING A RESTART IS THE NEXT LOCATION AFTER THE PHASE INFORMATION, THAT IS, EITHER
# (TC PHASCHNG) +2 OR +3, DEPENDING ON WHETHER D HAD BEEN SET OR NOT.  IF A IS SET TO 1, THEN THE ADDRESS THAT
# WOULD BE RESTARTED IS THE 2CADR THAT IS READ FROM THE NEXT TWO LOCATIONS.  EXAMPLES,
#
#	AD	TC	PHASCHNG	# THIS WOULD CAUSE LOCATION AD +3 TO BE
#	AD+1	OCT	05023		# RESTARTED BY GROUP THREE WITHA PRIORITY
#	AD+2	OCT	23000		# OF 23.  NOTE UPON RETURNING IT WOULD
#	AD+3				# ALSO GO TO AD+3
#
#	AD	TC	PHASCHNG	# GROUP 1 WOULD CAUSE CALLCALL TO BE
#	AD+1	OCT	27441		# BE STARTED AS A LONGCALL FROM THE TIME
#	AD+2	-GENADR	DELTIME		# STORED IN LONGBASE (LONGBASE WAS SET) BY
#	AD+3	2CADR	CALLCALL	# A DELTATIME STORED IN DELTIME.  THE
#	AD+4				# BBCON OF THE 2CADR SHOULD CONTAIN THE E
#	AD+5				# BANK OF DELTIME.  PHASCHNG RETURNS TO
#					# LOCATION AD+5
#
# NOTE THAT IF A VARIABLE PRIORITY IS GIVEN FOR A JOB, THE JOB WILL BE RESTARTED AS A NOVAC IF THE PRIORITY IS
# NEGATIVE, AS A FINDVAC IF THE PRIORITY IS POSITIVE.
#
# TYPE B PHASCHNG IS A COMBINATION OF VARIABLE AND FIXED PHASE CHANGES.  IT WILL START UP A JOB AS INDICATED
# BELOW AND ALSO START UP ONE FIXED RESTART, THAT IS EITHER AN G.1 OR A G.ODD OR THE FIRST ENTRY OF G.EVEN
# DOUBLE ENTRY.  THE BIT INFORMATION IS AS FOLLOWS,
#
#	TL1 DAP PPP PPP GGG
#
# WHERE EACH LETTER OR NUMBER STANDS FOR A BIT.  THE G:S STAND FOR THE GROUP, OCTAL 1 - 7. THE P:S FOR THE FIXED
# PHASE INFORMATION, OCTAL 0 - 127.  1 MUST BE 1.  AND AGAIN T STANDS FOR THE TBASE AND L FOR LONGBASE.  D THIS
# TIME STANDS ONLY FOR PRIORITY SINCE THIS WILL BE CONSIDERED A JOB, AND IT MUST BE GIVEN DIRECTLY IF GIVEN.
# AGAIN A STANDS FOR THE ADDRESS OF THE LOCATION TO BE RESTARTED, 1 IF THE 2CADR IS GIVEN, OR 0 IF IT IS TO BE
# THE NEXT LOCATION.(THE RETURN LOCATION OF PHASCHNG) EXAMPLES,
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
# Page 1408
# THE NOVAC-FINDVAC CHOICE FOR JOBS HOLDS HERE ALSO - NEGATIVE PRIORITY CAUSES A NOVAC CALL, POSITIVE A FINDVAC.


# SUMMARY OF BITS:
#	TYPE A		TL0 00P PPP PPP GGG
#	TYPE B		TL1 DAP PPP PPP GGG
#	TYPE C		TL0 1AD XXX CJW GGG

# Page 1409
# 2PHSCHNG IS USED WHEN ONE WISHES TO START UP A GROUP OR CHANGE A GROUP WHILE UNDER THE CONTROL OF A DIFFERENT
# GROUP.  FOR EXAMPLE, CHANGE THE PHASE OF GROUP 3 WHILE THE PORTION OF THE PROGRAM IS UNDER GROUP 5.  ALL 2PHSCHNG
# CALLS ARE MADE IN THE FOLLOWING MANNER,

#		TC	2PHSCHNG
#		OCT	XXXXX
#		OCT	YYYYY

# WHERE OCT XXXXX MUST BE OF TYPE A AND OCT YYYYY MAY BE OF EITHER TYPE A OR TYPE B OR TYPEC.  THERE IS ONE
# DIFFERENCE --- NOTE- IF LONGBASE IS TO BE SET THIS INFORMATION IS GIVEN IN THE OCT YYYYY INFORMATION, IT WILL
# BE DISREGARDED IF GIVEN WITH THE OCT XXXXX INFORMATION.  A COUPLE OF EXAMPLES MAY HELP.

#	AD	TC	2PHACHNG	# SET TBASE3 AND IF A RESTART OCCURS START
#	AD+1	OCT	40083		# THE TWO ENTRIES IN 3.8 TABLE LOCATION
#	AD+2	OCT	05025		# THIS IS OF TYPE C. SET THE JOB TO BE
#	AD+3	OCT	18000		# TO BE LOCATION AD+4, WITH A PRIORITY 18,
#	AD+4				# FOR GROUP 5 PHASE INFORMATION.

		COUNT	02/PHASE

; ============================================================================
; 2PHSCHNG - Double Phase Change Routine
;
; COMMENT-ONLY READERS: This routine allowed the AGC to manage two different
; mission program groups simultaneously. For example, during rendezvous
; operations, one program might monitor orbital position while another
; controlled thruster firing - both needing independent restart protection.
;
; CODE-ALONG READERS: 2PHSCHNG handles two phase change operations in one call.
; The first octal word (XXXXX) must be Type A, the second (YYYYY) can be
; Type A, B, or C. This enables complex program coordination while maintaining
; restart protection for both groups. Critically used during Apollo 11's
; translunar coast and lunar orbit operations.
; ============================================================================
2PHSCHNG	INHINT			# THE ENTRY FOR A DOUBLE PHASE CHANGE
		NDX	Q		; Index by return address Q
		CA	0		; Load first octal word (Type A phase info)
		INCR	Q		; Advance Q to next word
		TS	TEMPP2		; Save complete first phase word

		MASK	OCT7		; Extract group number (bits 0-2, octal 1-7)
		DOUBLE			; Multiply by 2 for table indexing
		TS	TEMPG2		; Store group index for first phase change

		CA	TEMPP2		; Reload first phase word
		MASK	OCT17770	# NEED ONLY 1770, BUT WHY GET A NEW CONST.
		EXTEND			; Extract phase number (bits 3-9)
		MP	BIT12		; Shift phase bits into position
		XCH	TEMPP2		; Store phase info, retrieve original word

		MASK	BIT15		; Check T bit (set TBASE flag)
		TS	TEMPSW2		# INDICATES WHETHER TO SET TBASE OR NOT
					; Non-zero = set TBASE for first group

		TCF	PHASCHNG +3	; Continue with second phase change

; ============================================================================
; PHASCHNG - Single Phase Change Entry Point
;
; COMMENT-ONLY READERS: This is the heart of the AGC's restart protection system.
; Every mission program called PHASCHNG to record its current execution state.
; If power failed or alarms occurred (like the famous 1202 alarm during Apollo 11's
; descent), the restart routines used this phase information to resume operations
; at the correct point without losing mission progress.
;
; CODE-ALONG READERS: PHASCHNG is the main entry point for single phase changes.
; It sets TEMPSW2 to ONE (differentiating from 2PHSCHNG entry), loads the phase
; word from the instruction following TC PHASCHNG, then transfers control to
; PHSCHNG2 in bank 14 to complete the phase update. Used hundreds of times
; throughout mission programs to maintain restart protection state.
; ============================================================================
PHASCHNG	INHINT			; Disable interrupts during critical phase update
		CA	ONE		# INDICATESWE CAME FROM A PHASCHNG ENTRY
		TS	TEMPSW2		; TEMPSW2=1 means single phase change (not double)

		NDX	Q		; Index by return address Q
		CA	0		; Load phase word (XXXXX) following TC PHASCHNG
		INCR	Q		; Advance Q past phase word for proper return
		TS	TEMPSW		; Save complete phase word in TEMPSW
# Page 1410
		EXTEND			; Extended instruction mode
		DCA	ADRPCHN2	# OFF TO SWITCHED BANK
					; Load 2CADR (bank + address) for PHSCHNG2
		DTCB			; Transfer control to other bank (PHSCHNG2)
					; PHSCHNG2 does the actual phase table update

		EBANK=	LST1		; Erasable bank where phase tables reside
ADRPCHN2	2CADR	PHSCHNG2	; Cross-bank address to PHSCHNG2 routine

; ============================================================================
; ONEORTWO - Phase Change Type Dispatcher (from PHSCHNG2)
;
; COMMENT-ONLY READERS: After storing phase information, the AGC needed to
; determine what additional restart information was required. Some programs
; needed to record task priorities, others needed memory addresses, and some
; needed timing information. This section decoded which information to capture.
;
; CODE-ALONG READERS: ONEORTWO is called from PHSCHNG2 after initial phase
; decoding. It checks bits 13-14 of the phase word to determine Type B or Type C,
; then extracts priority/delta-time and 2CADR information as specified by bits
; 7-8 of the phase portion. This enables flexible restart configuration.
; ============================================================================
ONEORTWO	LXCH	TEMPBBCN	; Restore bank register for cross-bank work
		LXCH	BBANK		; Exchange with current BBANK
		LXCH	TEMPBBCN	; Save result for later restoration

		MASK	OCT14000	# SEE WHAT KIND OF PHASE CHANGE IT IS
					; Bit 13 determines Type B vs Type C
		CCS	A		; Check if bit 13 is set
		TCF	CHECKB		# IT IS OF TYPE :B: (bit 13 set)
					; Type B = variable job + fixed restart

		CA	TEMPP		; Type C path (bit 13 clear)
		MASK	BIT7		; Check bit 7 of phase portion
		CCS	A		# SHALL WE USE THE OLD PRIORITY
		TCF	GETPRIO		# NO GET A NEW PRIORITY (OR DELTA T)
					; Bit 7 set = new priority follows phase word

; ============================================================================
; TRANSITION: From phase word decoding to priority determination
;
; The AGC must determine what priority to assign the restarted job or task.
; Two options exist: reuse the priority stored in the phase table from the
; previous execution (OLDPRIO), or fetch a new priority value from the
; instruction stream following the phase word (GETPRIO).
; ============================================================================

OLDPRIO		NDX	TEMPG		# USE THE OLD PRIORITY (OR DELTA T)
		CA	PHSPRDT1 -2	; Load priority from phase table entry
					; PHSPRDT1 = phase priority/delta-T table
					; Indexed by group number (TEMPG)
		TS	TEMPPR		; Store in temporary priority register
					; Will be written back to PHSPRDT1 later

; CON1: Check if restart job/task name (2CADR) provided explicitly
; Bit 8 of phase portion controls whether 2CADR follows in instruction stream

CON1		CA	TEMPP		# SEE IF A 2CADR IS GIVEN
		MASK 	BIT8		; Check bit 8 of phase portion
		CCS	A		; If bit 8 set, new 2CADR follows
		TCF	GETNEWNM	; Get new name (2CADR) from instruction stream

		; Bit 8 clear: Use current job/task name and bank
		CA	Q		; Load return address (caller's location)
		TS	TEMPNM		; Store as job/task name address
		CA	BB		; Load caller's bank register
		EXTEND			# PICK UP USERS SUPERBANK
		ROR	SUPERBNK	; Rotate right through SUPERBNK (bits 8-10)
					; Combines bank and superbank for full address
		TS	TEMPBB		; Store complete bank information

TOCON2		CA	CON2ADR		# BACK TO SWITCHED BANK
		LXCH	TEMPBBCN	; Restore caller's bank to L register
		DTCB			; Double transfer control to bank (jump to CON2)
					; Continues phase table update in different bank

CON2ADR		GENADR	CON2		; Generated address for CON2 routine

; GETPRIO: Fetch new priority (or delta-T) from instruction stream
; When bit 7 of phase word is set, priority follows the phase word in memory

GETPRIO		NDX	Q		# DON:T CARE IF DIRECT OR INDIRECT
					; Indexed by return address Q
		CA	0		# LEAVE THAT DECISION TO RESTARTS
					; Load word following TC PHASCHNG call
					; This is the priority/delta-T value
		INCR	Q		# OBTAIN RETURN ADDRESS
					; Increment Q to skip over priority word
					; Now Q points to actual return address
		TCF	CON1 -1		; Jump to CON1-1 (stores priority first)
					; The -1 entry stores priority then continues

; GETNEWNM: Fetch new job/task name (2CADR) from instruction stream
; When bit 8 of phase word is set, a 2CADR follows in memory specifying
; the job/task/longcall address to restart

GETNEWNM	EXTEND			; Extended instruction follows
# Page 1411
		INDEX	Q		; Indexed by return address
		DCA	0		; Load double word (2CADR) following phase word
					; 2CADR = address + bank for restart target
		DXCH	TEMPNM		; Store in TEMPNM (address) and TEMPBB (bank)
					; This identifies which job/task to restart
		CA	TWO		; Load constant 2
		ADS	Q		# OBTAIN RETURN ADDRESS
					; Add 2 to Q to skip over 2CADR (2 words)
					; Now Q points to actual return address

		TCF	TOCON2		; Jump to bank transfer, continue at CON2

; ============================================================================
; Temporary storage assignments for PHASCHNG routines
; Uses interrupt temporary storage and rupt registers to avoid memory conflicts
; ============================================================================

OCT14000	EQUALS	PRIO14		; Constant for priority 14 (octal 14000)
TEMPG		EQUALS	ITEMP1		; Temporary group number storage
TEMPP		EQUALS	ITEMP2		; Temporary phase portion storage
TEMPNM		EQUALS	ITEMP3		; Temporary job/task name (address)
TEMPBB		EQUALS	ITEMP4		; Temporary bank number
TEMPSW		EQUALS	ITEMP5		; Temporary switch word (first phase word)
TEMPSW2		EQUALS	ITEMP6		; Temporary switch word (second phase word)
TEMPPR		EQUALS	RUPTREG1	; Temporary priority/delta-T storage
TEMPG2		EQUALS	RUPTREG2	; Temporary group number (second entry)
TEMPP2		EQUALS	RUPTREG3	; Temporary phase portion (second entry)

TEMPBBCN	EQUALS	RUPTREG4	; Temporary bank storage for CON2
BB		EQUALS	BBANK		; Current bank register alias

		BANK	14		; Switch to bank 14
		SETLOC	PHASETAB	; Set location in phase table area
		BANK

		EBANK=	PHSNAME1	; Erasable bank for phase name tables
		COUNT	10/PHASE	; Instruction count group

; ============================================================================
; PHSCHNG2: Core phase table update routine (Bank 14)
;
; This routine performs the actual phase table updates. It decodes the phase
; word bit fields to extract group number, phase number, and control bits,
; then updates the appropriate phase table entries (PHASE1, -PHASE1, TBASE1,
; PHSPRDT1, PHSNAME1). During the Apollo 11 mission, these phase tables
; enabled the AGC to recover from power transients and restart interruptions,
; ensuring continuous guidance and control despite momentary failures.
;
; The phase word format (octal XXXXX after TC PHASCHNG):
;   Bits 1-3:   Group number (0-7) * 2 for table index
;   Bits 4-11:  Phase portion (phase number and control bits)
;   Bits 12-13: Type control (00=Type A, 01=Type B, 10=Type C)
;   Bits 14-15: Time base control (TBASE and/or LONGBASE)
; ============================================================================

PHSCHNG2	LXCH	TEMPBBCN	; Save caller's bank in L register
		CA	TEMPSW		; Load phase word (TEMPSW)
		MASK	OCT7		; Extract group number (bits 1-3)
		DOUBLE			; Multiply by 2 for table indexing
					; Phase tables use double-word entries
		TS	TEMPG		; Store group index for table access

		CA	TEMPSW		; Reload phase word
		MASK	OCT17770	; Extract phase portion (bits 4-11)
					; OCT17770 = octal 17770 isolates phase field
		EXTEND			; Extended instruction follows
		MP	BIT12		; Multiply by BIT12 (octal 4000)
					; Shifts phase bits into correct position
		TS	TEMPP		; Store phase portion in TEMPP
					; Contains phase number and control bits

		CA	TEMPSW		; Reload phase word again
		MASK	OCT60000	; Extract time base bits (bits 14-15)
					; OCT60000 = octal 60000 = TBASE control
		XCH	TEMPSW		; Exchange: A gets old TEMPSW, TEMPSW gets time bits
					; Preserves time base info in TEMPSW
		MASK	OCT14000	; Extract type bits (bits 12-13) from old TEMPSW
					; OCT14000 = octal 14000 isolates type field
		CCS	A		; Check if type bits are non-zero
		TCF	ONEORTWO	; Non-zero: Type B or C (calls ONEORTWO)
					; Zero: Type A (falls through to single entry)

# Page 1412
		; Single-entry restart logic (Type A or single-entry Type C)
		; Store phase information into phase table indexed by group number

		CA	TEMPP		# START STORING THE PHASE INFORMATION
					; Load phase portion into A register
		NDX	TEMPG		; Index by group number (TEMPG)
		TS	PHASE1 -2	; Store phase in PHASE1 table
					; PHASE1 table contains active phase number
					; for each restart group

BELOW1		CCS	TEMPSW2		# IS IT A PHASCHNG OR A 2PHSCHNG
					; Check if second phase word provided
					; TEMPSW2 set by 2PHSCHNG, zero for PHASCHNG
		TCF	BELOW2		# IT:S A PHASCHNG
					; Positive: single phase change, continue
					; at BELOW2 to complete update

		; Double-entry Type B restart handling (2PHSCHNG call)
		; Process second phase word for second restart table entry

		TCF	+1		# IT:S A 2PHSCHNG
					; Fall through to process second entry
		CS	TEMPP2		; Complement second phase portion
		LXCH	TEMPP2		; Exchange: L gets complemented value,
					; TEMPP2 gets restored
					; Prepares for -PHASE1 table update
		NDX	TEMPG2		; Index by second group number
		DXCH	-PHASE1 -2	; Store second phase in -PHASE1 table
					; Double-word exchange stores both phase
					; and complemented phase for Type B restart

		; Check if second entry needs time base update
		; TEMPSW2 controls second phase word processing

		CCS	TEMPSW2		; Check second switch word status
		NOOP			# CAN:T GET HERE
					; Positive case unreachable
		TCF	BELOW2		; Zero: jump to common TBASE logic
					; Continue with single-entry processing

		; Negative TEMPSW2: Update time base for second entry
		; This path taken when Type B restart needs time synchronization

		CS	TIME1		; Complement TIME1 (current mission time)
		NDX	TEMPG2		; Index by second group number
		TS	TBASE1 -2	; Store as time base for second restart
					; TBASE1 provides reference time for delta-T

; ============================================================================
; BELOW2: Time base selection logic for first phase entry
; 
; TEMPSW controls which time bases to update based on restart timing needs:
;   Positive: Set LONGBASE only (long waitlist delays)
;   Zero: Set neither time base (no timing reference needed)
;   Negative: Set TBASE1, possibly also LONGBASE (standard timing)
;
; Time bases provide reference points for restart recovery, enabling AGC
; to resume waitlist tasks with correct delta-time calculations after restart.
; ============================================================================

BELOW2		CCS	TEMPSW		# SEE IF WE SHOULD SET TBASE OR LONGBASE
					; Check TEMPSW to determine time base update
		TCF	BELOW3		# SET LONGBASE ONLY
					; Positive: jump to LONGBASE-only update
		TCF	BELOW4		# SET NEITHER
					; Zero: skip time base updates entirely

		; Negative TEMPSW: Set TBASE1 as primary time reference
		; TBASE1 stores TIME1 complement for standard waitlist tasks

		CS	TIME1		# SET TBASE TO BEGIN WITH
					; Complement TIME1 (current mission time)
		NDX	TEMPG		; Index by group number
		TS	TBASE1 -2	; Store in TBASE1 table entry for this group
					; Reference time for delta-T in restart

		; Now check if we should ALSO set LONGBASE
		; Tests bit 14 of TEMPSW for dual time base requirement

		CA	TEMPSW		# SHALL WE NOW SET LONGBASE
					; Load TEMPSW again
		AD	BIT14COM	; Add complement of bit 14 (OCT 17777)
					; Tests bit 14: if set, result positive
		CCS	A		; Check result
		NOOP			# ***** CAN'T GET HERE *****
					; Positive case mathematically unreachable
BIT14COM	OCT	17777		# ***** CAN'T GET HERE *****
					; Bit 14 complement constant
		TCF	BELOW4		# NO WE NEED ONLY SET TBASE
					; Zero/negative: TBASE1 sufficient, continue

; ============================================================================
; BELOW3: Set LONGBASE for extended-duration waitlist tasks
;
; LONGBASE provides time reference for waitlist tasks with delays exceeding
; normal TBASE1 range. Uses TIME2 (double-precision mission time) for
; greater precision in long-duration delta-time calculations during restart.
; ============================================================================

BELOW3		EXTEND			# SET LONGBASE
					; Extended instruction follows
		DCA	TIME2		; Load double-precision TIME2 (mission time)
					; DCA = Double precision Channel to A
		DXCH	LONGBASE	; Store in LONGBASE (double-word exchange)
					; Provides extended range time reference

; ============================================================================
; BELOW4: Finalize phase storage and return from PHASCHNG
;
; Completes phase table update by storing complemented phase in -PHASE1 table.
; This complemented copy enables restart logic to detect phase changes and
; restore correct program execution point after AGC restart events.
; ============================================================================

BELOW4		CS	TEMPP		# AND STORE THE FINAL PART OF THE PHASE
					; Complement phase word (prepare negative copy)
		NDX	TEMPG		; Index by group number
		TS	-PHASE1 -2	; Store in -PHASE1 table entry
					; Complemented phase provides change detection

		; Restore return address and exit phase change routine
		; Return to caller with interrupts re-enabled

		CA	Q		; Load return address from Q register
		LXCH	TEMPBBCN	; Exchange with saved BBANK in TEMPBBCN
					; Restores caller's bank context
		RELINT			; Re-enable interrupts (phase change complete)
		DTCB			; Return to caller in proper bank
					; DTCB = Double Transfer Control to Bank
; ============================================================================
; CON2: Alternative entry point for storing phase table entries
;
; Used when time base setup was handled earlier or not required. Stores
; complete phase information into three parallel tables:
;   PHASE1: Current phase number and control bits
;   PHSPRDT1: Phase priority for restart scheduling
;   PHSNAME1: Phase name/restart 2CADR for job/task reactivation
;
; After table update, jumps to BELOW1 to handle single vs double phase logic.
; ============================================================================

CON2		LXCH	TEMPBBCN	; Restore BBANK from TEMPBBCN to L register
					; Preserves bank context for return
# Page 1413
		; Store phase number into PHASE1 table
		; PHASE1 contains current active phase for each restart group

		CA	TEMPP		; Load phase portion (from phase word parsing)
		NDX	TEMPG		; Index by group number
		TS	PHASE1 -2	; Store in PHASE1 table entry for this group
					; Enables restart to identify current phase

		; Store phase priority into PHSPRDT1 table
		; Priority determines restart scheduling order

		CA	TEMPPR		; Load phase priority (extracted earlier)
		NDX	TEMPG		; Index by group number
		TS	PHSPRDT1 -2	; Store in PHSPRDT1 priority table
					; Controls which restart jobs run first

		; Store phase name/restart address into PHSNAME1 table
		; Double-precision 2CADR enables restart to reactivate job/task

		EXTEND			; Extended instruction follows
		DCA	TEMPNM		; Load double-word phase name/restart 2CADR
					; Contains bank and address for restart entry
		NDX	TEMPG		; Index by group number
		DXCH	PHSNAME1 -2	; Store in PHSNAME1 table (double exchange)
					; Restart uses this to resume execution

		; Phase tables now contain complete restart information for this group
		; Continue to BELOW1 to handle PHASCHNG vs 2PHSCHNG logic

		TCF	BELOW1		; Jump to check for second phase entry
					; BELOW1 handles single vs double phase

		BLOCK	02
		SETLOC	FFTAG1
		BANK

		COUNT	02/PHASE

; ============================================================================
; CHECKB: Type B phase change priority determination
;
; Examines bit 12 of phase word to determine if new priority should be
; obtained for Type B double-table restart. Type B allows two jobs/tasks/
; longcalls to restart, and bit 12 signals whether first entry needs new
; priority or should reuse existing priority from table.
;
; Bit 12 set: Get new priority (first restart job not yet scheduled)
; Bit 12 clear: Use old priority (first restart job already active)
; ============================================================================

CHECKB		MASK	BIT12		# SINCE THIS IS OF TYPE B, THIS BIT SHOULD
					; Isolate bit 12 from phase word in A
					; BIT12 = octal 4000
		CCS	A		# BE HERE IF WE ARE TO GET A NEW PRIORITY
					; Check if bit 12 is set (positive result)
		TCF	GETPRIO		# IT IS, SO GET NEW PRIORITY
					; Bit 12 set: obtain new priority for job
					; Jumps to GETPRIO to calculate priority

		; Bit 12 clear: reuse existing priority from phase table
		; First restart entry already scheduled, use its priority

		TCF	OLDPRIO		# IT ISN:T, USE THE OLD PRIORITY
					; Jumps to OLDPRIO to retrieve stored priority
					; Maintains scheduling consistency
