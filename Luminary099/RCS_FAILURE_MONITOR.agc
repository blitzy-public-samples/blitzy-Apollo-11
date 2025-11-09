# Copyright:	Public domain.
# Filename:	RCS_FAILURE_MONITOR.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	190-192
# Mod history:	2009-05-19 HG	Transcribed from page images.
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
; FILE: RCS_FAILURE_MONITOR.agc
; MODULE: Reaction Control System Failure Detection and Isolation
; MISSION PHASE: descent/landing/ascent/rendezvous
;
; TL;DR: Monitors RCS (Reaction Control System) thruster performance to detect
;        failed thrusters during all mission phases. When crew operates isolation
;        valve switches in response to Grumman failure-detection warning lamps,
;        this routine updates DAP (Digital Autopilot) jet-selection masks to
;        exclude failed thrusters and reconfigures the autopilot to use only
;        functional jets for attitude control.
;
; COMMENT-ONLY READERS: This safety-critical monitor runs every 480 milliseconds
;        throughout the mission, watching for thruster failures and ensuring the
;        autopilot never attempts to fire jets that have been isolated.
; CODE-ALONG READERS: Study the bit manipulation logic for translating channel 32
;        valve-closure bits into channel 5/6 DAP masks, and note the restart
;        protection strategy ensuring mask consistency despite potential interrupts.
; ============================================================================

# Page 190
; ============================================================================
; FAILURE DETECTION SYSTEM OVERVIEW
; ============================================================================
;
; The Lunar Module's RCS (Reaction Control System) provides attitude control
; through 16 small thrusters arranged in four quads around the spacecraft.
; Grumman's hardware failure-detection circuitry continuously monitors thruster
; performance and illuminates warning lamps in the cockpit when a jet fails.
; The crew responds by operating isolation valve switches that close propellant
; valves to failed thruster pairs, preventing propellant waste and potential
; hazards. These switch closures set bits in channel 32 of the AGC.
;
; This routine, called every 480 milliseconds by the T4RUPT timer interrupt,
; translates the channel 32 valve-closure bits into the CH5MASK and CH6MASK
; words used by the Digital Autopilot's jet-selection logic. The masks tell
; the autopilot which thrusters are available for attitude control maneuvers.
;
; RESTART PROTECTION STRATEGY:
; The code is designed to survive AGC restarts (power transients) with minimal
; consequence. Bit operations use logical OR/AND rather than simple assignment
; to avoid corrupting masks if a restart occurs mid-update. The one vulnerable
; case (restart between mask update and PVALVEST update, coupled with the crew
; reversing the switch at exactly that moment) is extremely unlikely and can be
; corrected by the crew re-cycling the affected switch slowly.
; ============================================================================

# PROGRAM DESCRIPTION
#
# AUTHOR:  J S MILLER
#
# MODIFIED 6 MARCH 1968 BY P S WEISSMAN TO SET UP JOB FOR 1/ACCS WHEN THE MASKS ARE CHANGED.
#
#      THIS ROUTINE IS ATTACHED TO T4RUPT, AND IS ENTERED EVERY 480 MS.  ITS FUNCTION IS TO EXAMINE THE LOW 8 BITS
# OF CHANNEL 32 TO SEE IF ANY ISOLATION-VALVE CLOSURE BITS HAVE APPEARED OR DISAPPEARED (THE CREW IS WARNED OF JET
# FAILURES BY LAMPS LIT BY THE GRUMMAN FAILURE-DETECTION CIRCUITRY; THEY MAY RESPOND BY OPERATING SWITCHES WHICH
# ISOLATE PAIRS OF JETS FROM THE PROPELLANT TANKS AND SET BITS IN CHANNEL 32).  IN THE EVENT THAT CHANNEL 32 BITS
# DIFFER FROM 'PVALVEST', THE RECORD OF ACTIONS TAKEN BY THIS ROUTINE, THE APPROPRIATE BITS IN 'CH5MASK' &
# 'CH6MASK', USED BY THE DAP JET-SELECTION LOGIC, ARE UPDATED, AS IS 'PVALVEST'.  TO SPEED UP & SHORTEN THE
# ROUTINE, NO MORE THAN ONE CHANGE IS ACCEPTED PER ENTRY.  THE HIGHEST-NUMBERED BIT IN CHANNEL 32 WHICH REQUIRES
# ACTION IS THE ONE PROCESSED.
#
#      THE CODING IN THE FAILURE MONITOR HAS BEEN WRITTEN SO AS TO HAVE ALMOST COMPLETE RESTART PROTECTION.  FOR
# EXAMPLE, NO ASSUMPTION IS MADE WHEN SETTING A 'CH5MASK' BIT TO 1 THAT THE PREVIOUS STATE IS 0, ALTHOUGH IT OF
# COURSE SHOULD BE.  ONE CASE WHICH MAY BE SEEN TO EVADE PROTECTION IS THE OCCURRENCE OF A RESTART AFTER UPDATING
# ONE OR BOTH DAP MASK-WORDS BUT BEFORE UPDATING 'PVALVEST', COUPLED WITH A CHANGE IN THE VALVE-BIT BACK TO ITS
# FORMER STATE.  THE CONSEQUENCE OF THIS IS THAT THE NEXT ENTRY WOULD NOT SEE THE CHANGE INCOMPLETELY INCORP-
# ORATED BY THE LAST PASS (BECAUSE IT WENT AWAY AT JUST THE RIGHT TIME), BUT THE DAP MASK-WORDS WILL BE INCORRECT.
# THIS COMBINATION OF EVENTS SEEMS QUITE REMOTE, BUT NOT IMPOSSIBLE UNLESS THE CREW OPERATES THE SWITCHES AT HALF-
# SECOND INTERVALS OR LONGER.  IN ANY EVENT, A DISAGREEMENT BETWEEN REALITY AND THE DAP MASKS WILL BE CURED IF
# THE MISINTERPRETED SWITCH IS REVERSED AND THEN RESTORED TO ITS CORRECT POSITION (SLOWLY).
#
# CALLING SEQUENCE:
#
#		TCF	RCSMONIT	(IN INTERRUPT MODE, EVERY 480 MS.)
#
# EXIT:  TCF  RCSMONEX  (ALL PATHS EXIT VIA SUCH AN INSTRUCTION)
RCSMONEX	EQUALS	RESUME

# ERASABLE INITIALIZATION REQUIRED:
#
#	   VIA FRESH START:  PVALVEST	       = +0  (ALL JETS ENABLED)
#			     CH5MASK, CH6MASK  = +0  (ALL JETS OK)
#
# OUTPUT:  CH5MASK & CH6MASK UPDATED  (1'S WHERE JETS NOT TO BE USED, IN CHANNEL 5 & 6 FORMAT)
#	   PVALTEST UPDATED  (1.5 WHEN VALVE CLOSURES HAVE BEEN TRANSLATED INTO CH5MASK & CH6MASK; CHAN 32 FORMAT)
#	   JOB TO DO 1/ACCS.
#
# DEBRIS:  A, L, Q AND DEBRIS OF NOVAC.
#
# SUBROUTINE CALLED:  NOVAC.

		EBANK=	CH5MASK

		BANK	23
		SETLOC	RCSMONT
		BANK
# Page 191
		COUNT*	$$/T4RCS

RCSMONIT	EQUALS	RCSMON

; ============================================================================
; MAIN FAILURE MONITOR ENTRY POINT
; ============================================================================
;
; This routine is invoked every 480 milliseconds by the T4RUPT timer interrupt.
; It checks if any crew member has operated an isolation valve switch since the
; last check, indicating response to a thruster failure warning lamp.
;
; The algorithm processes only one valve change per entry to keep execution time
; short within the interrupt context. If multiple switches change, the highest-
; numbered bit is processed first, with remaining changes handled on subsequent
; 480ms entries.
;
; CHANNEL 32 BIT MAPPING (low 8 bits):
; Bit 8-1: Each bit corresponds to an isolation valve switch position
;          1 = valve closed (thruster pair isolated from propellant tanks)
;          0 = valve open (thruster pair available for use)
; ============================================================================

RCSMON		CS	ZERO
		EXTEND
		RXOR	CHAN32		# PICK UP + INVERT INVERTED CHANNEL 32.
		MASK	LOW8		# KEEP JET-FAIL BITS ONLY.
		TS	Q

; ============================================================================
; CHANGE DETECTION LOGIC
; ============================================================================
;
; The Lunar Module's crew has switches to manually isolate failed RCS thrusters.
; This section detects when any switch position has changed since the last check.
; The detection uses XOR logic to identify bits that differ between the previous
; state (PVALVEST) and current state (channel 32).
;
; Mathematical approach: Form (P' AND C) OR (P AND C') where P = previous state,
; C = current state, ' = complement. Result is non-zero if any bit changed.
; ============================================================================

		CS	PVALVEST	#       -   -
		MASK	Q		# FORM PC + PC.
		TS	L		#   (P = PREVIOUS ISOLATION VALVE STATE,
		CS	Q		#    C = CURRENT VALVE STATE (CH 32)).
		MASK	PVALVEST
		ADS	L		# RESULT NZ INDICATES ACTION REQUIRED.

		EXTEND
		BZF	RCSMONEX	# QUIT IF NO ACTION REQUIRED.

		EXTEND
		MP	BIT7		# MOVE BITS 8 - 1 OF A TO 14 - 7 OF L.
		XCH	L		# ZERO TO L IN THE PROCESS.

; ============================================================================
; HIGHEST-BIT SELECTION LOOP
; ============================================================================
;
; This loop identifies which specific thruster pair isolation bit has changed.
; The algorithm finds the highest-numbered changed bit (bit 8 down to bit 1),
; processing one change per 480ms cycle for deterministic behavior.
;
; Loop mechanism: Repeatedly doubles the bit position until overflow occurs.
; L register is incremented and doubled each iteration until the DOUBLE
; instruction causes overflow, indicating the highest set bit has been found.
; The -3 label allows backward branching in AGC assembly.
; ============================================================================

 -3		INCR	L
		DOUBLE			# BOUND TO GET OVERFLOW IN THIS LOOP.
		OVSK			# SINCE WE ASSURED INITIAL NZ IN A.
		TCF	-3

; ============================================================================
; BIT EXTRACTION AND VALVE STATE DETERMINATION
; ============================================================================
;
; Having identified which bit changed, extract that specific bit and determine
; whether the isolation valve was opened or closed by the crew.
;
; The INDEX L instruction uses the L register (containing bit position 1-8)
; to select the appropriate bit from the BIT8-1 table. This bit is saved in Q
; register, then masked with PVALVEST to check the previous valve state.
;
; CCS (Count, Compare, and Skip) tests the result:
;   - Positive (>0): Valve was previously closed, now opened → branch to VOPENED
;   - Zero/Negative: Valve was previously open, now closed → continue to closure logic
; ============================================================================

		INDEX	L
		CA	BIT8 -1		# SAVE THE RELEVANT BIT (8 - 1).
		TS	Q
		MASK	PVALVEST	# LOOK AT PREVIOUS VALVE STATE BIT.
		CCS	A
		TCF	VOPENED		# THE VALVE HAS JUST BEEN OPENED.

; ============================================================================
; VALVE CLOSURE PATH - ISOLATE FAILED THRUSTER PAIR
; ============================================================================
;
; The crew has closed an isolation valve in response to a thruster failure
; detected by Grumman's hardware failure detection circuitry. This path
; updates the DAP jet selection masks to prevent the autopilot from attempting
; to fire the failed thruster pair.
;
; Process for both Channel 5 and Channel 6 jets:
;   1. Complement current mask (CS instruction inverts all bits)
;   2. Use INDEX L to fetch appropriate inhibit pattern from lookup table
;   3. MASK operation isolates the specific jets to inhibit
;   4. ADS (Add to Storage) sets the inhibit bit(s) in the mask
;
; The 5FAILTAB and 6FAILTAB tables map each isolation valve bit position
; to the corresponding thruster jet bit patterns in Channels 5 and 6.
; Once masks are updated, record the action in PVALVEST to prevent
; re-processing on the next 480ms cycle.
; ============================================================================

		CS	CH5MASK		# THE VALVE HAS JUST BEEN CLOSED.
		INDEX	L
		MASK	5FAILTAB
		ADS	CH5MASK		# SET INHIBIT BIT FOR CHANNEL 5 JET.

		CS	CH6MASK
		INDEX	L
		MASK	6FAILTAB
		ADS	CH6MASK		# SET INGIBIT BIT FOR CHANNEL 6 JET.

		CA	Q
		ADS	PVALVEST	# RECORD ACTION TAKEN.

		TCF	1/ACCFIX	# SET UP 1/ACCJOB AND EXIT.

# Page 192
; ============================================================================
; VALVE OPENING PATH - RESTORE THRUSTER PAIR TO SERVICE
; ============================================================================
;
; The crew has reopened an isolation valve, indicating the thruster pair is
; ready to return to service. This path clears the corresponding inhibit bits
; in the DAP jet selection masks, allowing the autopilot to once again use
; these thrusters for attitude control.
;
; Process differs from closure path - uses complement of table values:
;   1. Complement the table entry (CS instruction)
;   2. MASK with current CH5MASK/CH6MASK to clear the specific bits
;   3. TS (Transfer to Storage) writes the updated mask back
;
; This technique uses bitwise masking to turn off specific bits without
; disturbing other mask bits that may represent different failed jets.
; The complement-and-mask operation ensures only the relevant inhibit bits
; are cleared, maintaining isolation of any other failed thruster pairs.
; ============================================================================

VOPENED		INDEX	L		# A VALVE HAS JUST BEEN OPENED.
		CS	5FAILTAB
		MASK	CH5MASK
		TS	CH5MASK		# REMOVE INHIBIT BIT FOR CHANNEL 5 JET.

		INDEX	L
		CS	6FAILTAB
		MASK	CH6MASK
		TS	CH6MASK		# REMOVE INHIBIT BIT FOR CHANNEL 6 JET.

		CS	Q
		MASK	PVALVEST
		TS	PVALVEST	# RECORD ACTION TAKEN.

; ============================================================================
; AUTOPILOT RECONFIGURATION - SCHEDULE 1/ACCJOB
; ============================================================================
;
; After updating the jet selection masks, schedule the 1/ACCJOB to recalculate
; the DAP's acceleration parameters and thruster switch curves. This is
; critical because losing one or more thrusters changes the spacecraft's
; effective control authority - the remaining thrusters must be commanded
; differently to achieve the same attitude rates.
;
; NOVAC (Schedule New Job in VAC area) creates a new executive job with:
;   - Priority 27 (moderate priority, will run soon but not immediately)
;   - Entry point: 1/ACCJOB subroutine
;   - EBANK context: AOSQ erasable bank
;
; The 1/ACCJOB will update TJETLAW switch curves, ensuring the Digital
; Autopilot adapts to the new thruster configuration. During Apollo 11's
; mission, this automatic reconfiguration capability was essential for
; continued attitude control even with thruster failures.
; ============================================================================

1/ACCFIX	CAF	PRIO27		# SET UP 1/ACCS SO THAT THE SWITCH CURVES
		TC	NOVAC		#   FOR TJETLAW CAN BE MODIFIED IF CH5MASK
		EBANK=	AOSQ		#   HAS BEEN ALTERED.
		2CADR	1/ACCJOB

		TCF	RCSMONEX	# EXIT.

; ============================================================================
; THRUSTER ISOLATION MAPPING TABLES
; ============================================================================
;
; These lookup tables map Channel 32 isolation valve bits (set by crew
; switches) to the corresponding thruster control bits in Channel 5 and
; Channel 6 output registers. The LM has 16 RCS thrusters organized into
; 8 pairs, with each pair controlled by a single propellant isolation valve.
;
; HARDWARE INTERFACE CONTEXT:
; - Channel 32 (input): Reads crew-operated isolation valve switch positions
;   * Bits 8-1 represent 8 thruster pairs
;   * Bit set (1) = valve CLOSED (thrusters isolated from propellant)
;   * Bit clear (0) = valve OPEN (thrusters available)
;
; - Channel 5 (output): Controls 8 thruster solenoid valves
;   * AGC sends firing commands via specific bit patterns
;   * Multiple bits may be set simultaneously for coupled jet firings
;
; - Channel 6 (output): Controls 8 additional thruster solenoid valves
;   * Complements Channel 5 for complete 16-thruster control
;   * Bit patterns coordinated for optimal attitude control torques
;
; TABLE ORGANIZATION:
; Tables are indexed by L register (1-8) using INDEX instruction.
; "EQUALS -1" directive places table base address one location before
; first entry, so INDEX L with L=1 accesses first table entry.
;
; OCTAL BIT PATTERNS:
; Each octal value represents which thruster bit(s) to inhibit when
; the corresponding isolation valve is closed. Patterns are NOT simple
; one-to-one mappings because thruster pair isolation affects specific
; solenoid combinations based on the LM's plumbing configuration.
; ============================================================================

5FAILTAB	EQUALS	-1		# CH 5 JET BIT CORRESPONDING TO CH 32 BIT:
		OCT	00040		# 8
		OCT	00020		# 7
		OCT	00100		# 6
		OCT	00200		# 5
		OCT	00010		# 4
		OCT	00001		# 3
		OCT	00004		# 2
		OCT	00002		# 1

6FAILTAB	EQUALS	-1		# CH 6 JET BIT CORRESPONDING TO CH 32 BIT:
		OCT	00010		# 8
		OCT	00020		# 7
		OCT	00004		# 6
		OCT	00200		# 5
		OCT	00001		# 4
		OCT	00002		# 3
		OCT	00040		# 2
		OCT	00100		# 1


