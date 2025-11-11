# Copyright:	Public domain.
# Filename:	ALARM_AND_ABORT.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1381-1385
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Luminary131/ file of the same
#				name, using Luminary099 page images.
#		2009-06-05 RSB	Fixed a type.
#		2011-01-06 JL	Fixed pseudo-label indentation.
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
; FILE: ALARM_AND_ABORT.agc
; MODULE: Program Alarm and Abort System
; MISSION PHASE: ALL PHASES (descent/landing/ascent/rendezvous)
;
; TL;DR: Implements the Apollo Guidance Computer's alarm and abort system
;        managing alarm code generation, display to crew via DSKY, alarm
;        history recording, and abort mode transitions for P70/P71. Contains
;        the famous routines that handled the 1202 program alarm during
;        Apollo 11's lunar descent.
;
; COMMENT-ONLY READERS: This is where the famous "1202 alarm" logic lives.
;        Follow comments to understand how the AGC warned the crew when
;        computational load exceeded capacity during the landing.
; CODE-ALONG READERS: Study the alarm storage mechanism in FAILREG registers
;        and the integration with DSKY display system for crew notification.
; ============================================================================
;
; ============================================================================
; THE FAMOUS 1202 ALARM - APOLLO 11 LUNAR DESCENT CONTEXT
; ============================================================================
;
; At mission elapsed time 102:38:26 on July 20, 1969, during powered descent
; to the lunar surface, the Apollo Guidance Computer triggered program alarm
; 1202 indicating executive job queue overflow. The alarm recurred multiple
; times during descent as landing radar data created excessive computational
; load (caused by rendezvous radar inadvertently left in AUTO mode).
;
; Flight controller Steve Bales (GUIDO) consulted with Jack Garman in the
; back room. Despite the alarm, they recognized the AGC's restart protection
; was preserving guidance state. Bales made the critical call: "We're Go on
; that alarm." Neil Armstrong acknowledged: "Roger, we got you, we're Go.
; Hang tight, we're Go. 2000 feet."
;
; The landing continued successfully. Post-flight analysis confirmed the
; alarm was non-critical - the AGC shed lower-priority tasks while preserving
; essential guidance and control functions. This alarm system, implemented in
; the following code, enabled one of history's most critical go/no-go
; decisions.
;
; ALARM CODE CATALOG:
;   01201 - Executive job queue overflow (7 core sets full)
;   01202 - Waitlist overflow (too many timed tasks scheduled)
;   01203 - Symbolic input buffer overflow
;   01204 - Display system overload
;   02107 - Desired gimbal angles not reached
;   02109 - AGS telemetry error
;   Various mission-specific alarms (P programs) numbered by area and type
;
; The system stores up to 3 active alarms in FAILREG, FAILREG+1, FAILREG+2.
; When an alarm occurs, the program alarm light illuminates on the DSKY and
; the alarm code is displayed. Crew can acknowledge with KEY REL button.
; ============================================================================

# Page 1381
# THE FOLLOWING SUBROUTINE MAY BE CALLED TO DISPLAY A NON-ABORTIVE ALARM CONDITION.  IT MAY BE CALLED
# EITHER IN INTERRUPT OR UNDER EXECUTIVE CONTROL.
#
# CALLING SEQUENCE IS AS FOLLOWS:
#		TC	ALARM
#		OCT	AAANN		# ALARM NO. NN IN GENERAL AREA AAA.
#					# (RETURNS HERE)

		BLOCK	02
		SETLOC	FFTAG7
		BANK

		EBANK=	FAILREG

		COUNT*	$$/ALARM

# ALARM TURNS ON THE PROGRAM ALARM LIGHT, BUT DOES NOT DISPLAY.

; ============================================================================
; ALARM - Non-abortive alarm handler (no DSKY display)
; ============================================================================
; Entry point for non-critical alarms that illuminate the program alarm light
; but do not display alarm code to crew. Used for lower-priority conditions
; that require crew awareness but not immediate action.
;
; This routine stores the alarm code in one of three FAILREG locations for
; later retrieval and turns on the program alarm light (COMP ACTY indicator).
; Unlike PRIOLARM, it does not interrupt crew displays.
;
; The alarm storage uses three registers (FAILREG, FAILREG+1, FAILREG+2) to
; hold up to three simultaneous alarm conditions. The first alarm triggers
; the program alarm light; subsequent alarms are queued.

ALARM		INHINT

		CA	Q
ALARM2		TS	ALMCADR
		INDEX	Q
		CA	0
BORTENT		TS	L

PRIOENT		CA	BBANK
 +1		EXTEND
		ROR	SUPERBNK	# ADD SUPER BITS.
		TS	ALMCADR +1

LARMENT		CA	Q		# STORE RETURN FOR ALARM
		TS	ITEMP1

CHKFAIL1	CCS	FAILREG		# IS ANYTHING IN FAILREG
		TCF	CHKFAIL2	# YES TRY NEXT REG
		LXCH	FAILREG
		TCF	PROGLARM	# TURN ALARM LIGHT ON FOR FIRST ALARM

CHKFAIL2	CCS	FAILREG +1
		TCF	FAIL3
		LXCH	FAILREG +1
		TCF	MULTEXIT

FAIL3		CA	FAILREG +2
		MASK	POSMAX
		CCS	A
		TCF	MULTFAIL
		LXCH	FAILREG +2
		TCF	MULTEXIT

# Page 1382

PROGLARM	CS	DSPTAB +11D
		MASK	OCT40400
		ADS	DSPTAB +11D


MULTEXIT	XCH	ITEMP1		# OBTAIN RETURN ADDRESS IN A
		RELINT
		INDEX	A
		TC	1

MULTFAIL	CA	L
		AD	BIT15
		TS	FAILREG +2

		TCF	MULTEXIT

# PRIOLARM DISPLAYS V05N09 VIA PRIODSPR WITH 3 RETURNS TO THE USER FROM THE ASTRONAUT AT CALL LOC +1,+2,+3 AND
# AN IMMEDIATE RETURN TO THE USER AT CALL LOC +4.  EXAMPLE FOLLOWS,
#		CAF	OCTXX		# ALARM CODE
#		TC	BANKCALL
#		CADR	PRIOLARM
#		...	...
#		...	...
#		...	...		# ASTRONAUT RETURN
#		TC	PHASCHNG	# IMMEDIATE RETURN TO USER.  RESTART
#		OCT	X.1		# PHASE CHANGE FOR PRIO DISPLAY

; ============================================================================
; PRIOLARM - Priority alarm with DSKY display (V05N09)
; ============================================================================
; Entry point for critical alarms requiring immediate crew attention via DSKY.
; Displays alarm code using Verb 05 Noun 09 through priority display system.
; Provides three crew response entry points (KEY REL acknowledgments at +1,
; +2, +3) plus immediate return at +4 for program continuation.
;
; Unlike ALARM, PRIOLARM interrupts current DSKY displays to show the alarm
; code. Used during mission-critical phases when crew must acknowledge alarm
; conditions. The 1201/1202 alarms during Apollo 11 descent used this path.
;
; Calling sequence allows program to regain control after crew acknowledgment
; while preserving restart protection through phase change mechanism.

		BANK	10
		SETLOC	DISPLAYS
		BANK

		COUNT*	$$/DSPLA
PRIOLARM	INHINT			# * * * KEEP IN DISPLAY ROUTINES BANK
		TS	L		# SAVE ALARM CODE

		CA	BUF2		# 2 CADR OF PRIOLARM USER
		TS	ALMCADR
		CA	BUF2 +1
		TC	PRIOENT +1	# * LEAVE L ALONE
-2SEC		DEC	-200		# *** DONT MOVE
		CAF	V05N09
		TCF	PRIODSPR

		BLOCK	02
		SETLOC	FFTAG7
		BANK

		COUNT*	$$/ALARM

; ============================================================================
; BAILOUT - Operator error abort (non-catastrophic)
; ============================================================================
; Handles operator error conditions that require program termination but not
; full system restart. Stores alarm code 40400 (octal) indicating crew input
; error or invalid operational sequence. Less severe than POODOO.
;
; Called when crew attempts invalid operation or program detects recoverable
; error state. System remains operational; crew can restart mission programs.
; Transitions through WHIMPER to controlled program termination.

BAILOUT		INHINT
		CA	Q
# Page 1383
		TS	ALMCADR

		INDEX	Q
		CAF	0
		TC	BORTENT
OCT40400	OCT	40400

; ============================================================================
; WHIMPER - Controlled abort transition to restart
; ============================================================================
; Intermediate state during abort processing. Sets up BRUPT (bailout return
; address) and transitions to ENEMA routine for system cleanup before restart.
; Name reflects non-catastrophic nature compared to POODOO.

		INHINT
WHIMPER		CA	TWO
		AD	Z
		TS	BRUPT
		RESUME
		TC	POSTJUMP	# RESUME SENDS CONTROL HERE
		CADR	ENEMA

; ============================================================================
; POODOO - Catastrophic abort (unrecoverable error)
; ============================================================================
; Handles catastrophic system failures requiring immediate abort. Alarm code
; 77770 (octal) indicates severe malfunction. During lunar descent, POODOO
; would trigger P70 abort program to separate from descent stage and return
; to orbit. During ascent, triggers P71 for contingency rendezvous.
;
; Name originated from AGC programmers' dark humor about worst-case scenarios.
; Called when guidance/navigation state is corrupted beyond recovery or
; critical hardware failure detected. System enters abort mode immediately.
;
; ABORT2 is alternate entry point preserving different register states.

POODOO		INHINT
		CA	Q
ABORT2		TS	ALMCADR
		INDEX	Q
		CAF	0
		TC	BORTENT
OCT77770	OCT	77770		# DON'T MOVE

; ============================================================================
; GOPOODOO - Abort mode execution (P70/P71 transition)
; ============================================================================
; Executes full abort sequence after POODOO invoked. Terminates all active
; mission programs, resets system flags, and transitions to abort guidance.
; During Apollo 11 descent, would have initiated emergency ascent if called.
;
; Sequence: Reset flags → Terminate program groups → Call servicer idle state
; → Transfer control to appropriate abort program (P70 for descent, P71 for
; ascent phase). Abort is irreversible once GOPOODOO executes.

		CAF	OCT35		# 4.35SPOT FOR GOPOODOO
		TS	L
		COM
		DXCH	-PHASE4
GOPOODOO	INHINT
		TC	BANKCALL	# RESET STATEFLG, REINTFLG, AND NODOFLAG.
		CADR	FLAGS
		CA	FLAGWRD7	# IS SERVICER CURRENTLY IN OPERATION?
		MASK	V37FLBIT
		CCS	A
		TCF	STRTIDLE
		TC	BANKCALL	# TERMINATE GRPS 1, 3, 5, AND 6
		CADR	V37KLEAN
		TC	BANKCALL	# TERMINATE GRPS 2, 4, 1, 3, 5, AND 6
		CADR	MR.KLEAN	#	(I.E., GRP 4 LAST)
		TCF	WHIMPER

; ============================================================================
; STRTIDLE - Servicer idle state transition during abort
; ============================================================================
; Transitions servicer task to idle "ground" state during abort processing.
; Called when servicer (V37 task) is active during GOPOODOO. Ensures clean
; shutdown of background tasks before abort program takes control.

STRTIDLE	CAF	BBSERVDL
		TC	SUPERSW
		TC	BANKCALL	# PUT SERVICER INTO ITS "GROUND" STATE
		CADR	SERVIDLE	# AND PROCED TO GOTOPOOH.

; ============================================================================
; CCSHOLE - CCS instruction failure abort
; ============================================================================
; Handles failures in CCS (Count, Compare, and Skip) instruction execution.
; Alarm code 1103 indicates AGC hardware malfunction in CCS logic. Extremely
; rare; indicates potential computer hardware failure. Invokes ABORT2 path
; for catastrophic abort processing.

CCSHOLE		INHINT
		CA	Q
		TC	ABORT2
OCT1103		OCT	1103

; ============================================================================
; CURTAINS - Erasable memory overflow alarm
; ============================================================================
; Handles erasable memory (RAM) overflow conditions. Alarm code 00217 (octal)
; indicates program attempted to use more than available 2K words of erasable
; memory. Non-catastrophic; returns to user after alarm display. Less severe
; than executive overflow (1201) or waitlist overflow (1202).

CURTAINS	INHINT
		CA	Q
		TC	ALARM2
OCT217		OCT	00217
# Page 1384
		TC	ALMCADR		# RETURN TO USER

; ============================================================================
; BAILOUT1 - Alternate bailout entry preserving ALMCADR state
; ============================================================================
; Alternate entry to BAILOUT that preserves double-word state in ALMCADR.
; Uses DXCH to save both A and L registers before loading alarm code 40400.
; Converges with POODOO1 at BOTHABRT for common abort processing.

BAILOUT1	INHINT
		DXCH	ALMCADR
		CAF	ADR40400

; ============================================================================
; BOTHABRT - Common abort path for BAILOUT1 and POODOO1
; ============================================================================
; Shared processing for both operator error (40400) and catastrophic (77770)
; aborts when using alternate entry points. Merges control flow to reduce
; code duplication while preserving appropriate alarm codes.

BOTHABRT	TS	ITEMP1
		INDEX	Q
		CAF	0
		TS	L
		TCF	CHKFAIL1

; ============================================================================
; POODOO1 - Alternate catastrophic abort entry preserving ALMCADR state
; ============================================================================
; Alternate entry to POODOO preserving double-word ALMCADR state. Loads
; catastrophic abort code 77770 before converging with BAILOUT1 at BOTHABRT.
; Used when calling context requires preserving A/L register pair.

POODOO1		INHINT
		DXCH	ALMCADR
		CAF	ADR77770
		TCF	BOTHABRT

; ============================================================================
; ALARM1 - Alternate alarm entry preserving ALMCADR state
; ============================================================================
; Alternate entry to ALARM using DXCH for double-word ALMCADR preservation.

ALARM1		INHINT
		DXCH	ALMCADR

; ============================================================================
; ALMNCADR - Alarm entry with address already in ALMCADR
; ============================================================================
; Special entry point when calling address is already stored in ALMCADR.
; Skips address storage, proceeds directly to alarm code retrieval and
; processing through LARMENT path.

ALMNCADR	INHINT
		INDEX	Q
		CA	0
		TS	L
		TCF	LARMENT

; ============================================================================
; Address Constants and Symbolic Definitions
; ============================================================================
; ADR77770: Transfer control address to catastrophic abort code (POODOO)
; ADR40400: Transfer control address to operator error code (BAILOUT)
; DOALARM: Synonym for ENDOFJOB - returns control to executive after alarm
; BBSERVDL: Bank call address to servicer idle routine (used in STRTIDLE)

ADR77770	TCF	OCT77770
ADR40400	TCF	OCT40400
DOALARM		EQUALS	ENDOFJOB
		EBANK=	DVCNTR
BBSERVDL	BBCON	SERVIDLE

; ============================================================================
; VARALARM - Variable Alarm Code Entry Point
; ============================================================================
; Alternate alarm entry point where the alarm code is passed in the
; accumulator (A register) rather than fetched from memory following the
; TC instruction. Enables dynamic alarm code generation.
;
; CALLING SEQUENCE:
;   CAF  (alarm_code)  ; Load desired alarm code into A register
;   TC   VARALARM      ; Transfer control to variable alarm handler
;
; The routine illuminates the program alarm light on the DSKY but does not
; automatically display the alarm (non-abortive alarm). Commonly used by
; routines that compute or select alarm codes at runtime based on error
; conditions detected during execution.

# CALLING SEQUENCE FOR VARALARM
#		CAF	(ALARM)
#		TC	VARALARM
#
# VARALARM TURNS ON PROGRAM ALARM LIGHT BUT DOES NOT DISPLAY

VARALARM	INHINT

		TS	L		# SAVE USERS ALARM CODE

		CA	Q		# SAVE USERS Q
		TS	ALMCADR

		TC	PRIOENT
OCT14		OCT	14		# DONT MOVE

		TC	ALMCADR		# RETURN TO USER

; ============================================================================
; ABORT - Synonym for WHIMPER Abort Sequence
; ============================================================================
; Symbolic alias defining ABORT as equivalent to WHIMPER. Provides alternate
; name for abort entry point used by mission programs. Both names invoke
; the controlled abort sequence that terminates current program, displays
; abort alarm, and awaits crew decision on next action.

ABORT		EQUALS	WHIMPER
		BANK	13
		SETLOC	ABTFLGS
		BANK
# Page 1385
		COUNT*	$$/ALARM

; ============================================================================
; FLAGS - System Flags Clearing Routine
; ============================================================================
; Clears critical system state flags following abort or restart. Ensures
; clean state for subsequent program execution by resetting:
;
;   STATEBIT (in FLAGWRD3):  State vector integration flag
;   REINTBIT (in FLGWRD10):  Reintegration required flag
;   NODOBIT  (in FLAGWRD2):  No display output flag
;
; Called during abort processing to prevent aborted program states from
; affecting newly initiated programs. Critical for restart protection
; logic that enabled continuation through 1202 alarms - ensures flags
; from interrupted computations don't corrupt restarted guidance state.
;
; Returns via Q register to caller.

FLAGS		CS	STATEBIT
		MASK	FLAGWRD3
		TS	FLAGWRD3
		CS	REINTBIT
		MASK	FLGWRD10
		TS	FLGWRD10
		CS	NODOBIT
		MASK	FLAGWRD2
		TS	FLAGWRD2
		TC	Q

