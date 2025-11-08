# Copyright:	Public domain.
# Filename:	T6-RUPT_PROGRAMS.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1403-1405
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
; FILE: T6-RUPT_PROGRAMS.agc
; MODULE: Digital Autopilot (DAP) Time Control
; MISSION PHASE: descent/landing/ascent/rendezvous (all LM flight phases)
;
; TL;DR: Implements Timer 6 interrupt service routines enabling the LM Digital
;        Autopilot to precisely control Reaction Control System (RCS) jet firing
;        times. T6JOBCHK polls for pending interrupts when hardware interrupts
;        are disabled, while DOT6RUPT processes actual Timer 6 interrupts to
;        maintain attitude control during all mission phases from separation
;        through docking. This code executed during Eagle's lunar descent,
;        landing, surface operations, and ascent rendezvous with Columbia.
;
; COMMENT-ONLY READERS: This routine manages the timing mechanism that controls
;        the LM's attitude control jets, keeping Eagle stable during all flight
;        phases including the critical landing and ascent sequences.
; CODE-ALONG READERS: Study TIME6 counter management, interrupt priority
;        handling, and DAP interface to understand real-time RCS jet control
;        coordination with 5-millisecond timing precision requirements.
; ============================================================================

# Page 1403
; ============================================================================
; DIGITAL AUTOPILOT TIMER 6 CONTROL SYSTEM
;
; The Lunar Module maintains precise attitude control during all flight phases
; through 16 small Reaction Control System (RCS) jets. These jets must fire
; in carefully timed pulses to avoid wasting propellant while keeping Eagle
; stable. The Timer 6 (T6) interrupt system provides the timing mechanism that
; tells each jet exactly when to turn on and off.
;
; During Apollo 11's descent on July 20, 1969, this timing system coordinated
; hundreds of jet firings per minute to counteract the descent engine's
; thrust while Armstrong and Aldrin descended toward the Sea of Tranquility.
; ============================================================================

# PROGRAM NAMES:	(1) T6JOBCHK	MOD. NO. 5	OCTOBER 2, 1967
#			(2) DOT6RUPT
# MODIFICATION BY:	LOWELL G. HULL (A.C.ELECTRONICS)
#
# THESE PROGRAMS ENABLE THE LM DAP TO CONTROL THE THRUST TIMES OF THE REACTION CONTROL SYSTEM JETS BY USING TIME6.
; ============================================================================
; TIMER 6 CONTROL CONVENTIONS (CRITICAL - DO NOT VIOLATE)
;
; Timer 6 belongs exclusively to the Digital Autopilot. No other program may
; modify TIME6 or enable its counter. This strict ownership prevents conflicts
; that could cause jets to fire at wrong times, potentially destabilizing the
; spacecraft or wasting critical propellant reserves.
; ============================================================================

# SINCE THE LM DAP MAINTAINS EXCLUSIVE CONTROL OVER TIME6 AND ITS INTERRUPTS, THE FOLLOWING CONVENTIONS HAVE BEEN
# ESTABLISHED AND MUST NOT BE TAMPERED WITH:
#	1.	NO NUMBER IS EVER PLACED INTO TIME6 EXCEPT BY LM DAP.
#	2.	NO PROGRAM OTHER THAN LM DAP ENABLES THE TIME6 COUNTER.
#	3.	TO USE TIME6, THE FOLLOWING SEQUENCE IS ALWAYS EMPLOYED:
#		A.	A POSITIVE (NON-ZERO) NUMBER IS STORED IN TIME6.
#		B.	THE TIME6 CLOCK IS ENABLED.
#		C.	TIME6 IS INTERROGATED AND IS:
#			I.	NEVER FOUND NEGATIVE (NON-ZERO) OR +0.
#			II.	SOMETIMES FOUND POSITIVE (BETWEEN 1 AND 240D) INDICATING THAT IT IS ACTIVE.
#			III.	SOMETIMES FOUND POSMAX INDICATING THAT IT IS INACTIVE AND NOT ENABLED.
#			IV.	SOMETIMES FOUND NEGATIVE ZERO INDICATING THAT:
#				A.	A T6RUPT IS ABOUT TO OCCUR AT THE NEXT DINC, OR
#				B.	A T6RUPT IS WAITING IN THE PRIORITY CHAIN, OR
#				C.	A T6RUPT IS IN PROCESS NOW.
#	4.	ALL PROGRAMS WHICH OPERATE IN EITHER INTERRUPT MODE OR WITH INTERRUPT INHIBITED MUST CALL T6JOBCHK
#		EVERY 5 MILLISECONDS TO PROCESS A POSSIBLE WAITING T6RUPT BEFORE IT CAN BE HONORED BY THE HARDWARE.
#      (5.	PROGRAM JTLST, IN Q,R-AXES, HANDLES THE INPUT LIST.)
#
# T6JOBCHK CALLING SEQUENCE:
#		L	TC	T6JOBCHK
#		L+1	(RETURN)
#
# DOT6RUPT CALLING SEQUENCE:
#			DXCH	ARUPT		# T6RUPT LEAD IN AT LOCATION 4004.
#			EXTEND
#			DCA	T6ADR
#			DTCB
#
# SUBROUTINES CALLED:	DOT6RUPT CALLS T6JOBCHK.
#
# NORMAL EXIT MODES:	T6JOBCHK RETURNS TO L +1.
#			DOT6RUPT TRANSFERS CONTROL TO RESUME.
#
# ALARM/ABORT MODES:	NONE.
#
# INPUT:	TIME6		NXT6ADR		OUTPUT:		TIME6		NXT6ADR		CHANNEL 5
#		T6NEXT		T6NEXT +1			T6NEXT		T6NEXT +1	CHANNEL 6
#		T6FURTHA	T6FURTHA +1			T6FURTHA	T6FURTHA +1	BIT15/CH13
#
# DEBRIS:	T6JOBCHK CLOBBERS A.  DOT6RUPT CLOBBERS NOTHING.

; ============================================================================
; T6JOBCHK - TIMER 6 INTERRUPT POLLING ROUTINE
;
; When the AGC is running with interrupts disabled (INHINT mode), hardware
; interrupts cannot be serviced immediately. To prevent TIME6 interrupts from
; being delayed too long, any program running with interrupts disabled must
; call T6JOBCHK at least every 5 milliseconds to manually check if a T6
; interrupt is waiting and process it before jet timing becomes inaccurate.
;
; During critical mission phases like powered descent, guidance programs
; disable interrupts briefly for precise calculations. This polling mechanism
; ensures jet control remains accurate even during those brief interrupt
; blackout periods.
; ============================================================================

		BLOCK	02
# Page 1404
		BANK	17
		SETLOC	DAPS2
		BANK
		EBANK=	T6NEXT
		COUNT*	$$/DAPT6

; Check TIME6 counter to determine if interrupt needs processing.
; TIME6 states indicate different conditions:
;   Positive (1-240): Counter actively counting down, no action needed
;   POSMAX: Counter disabled and inactive
;   Negative zero (-0): Interrupt pending or in progress - must process now

T6JOBCHK	CCS	TIME6		# CHECK TIME6 FOR WAITING T6RUPT:
		TC	Q		# NONE: CLOCK COUNTING DOWN.
		TC	CCSHOLE		# +0 case (unused, trap for safety)
		TC	T6JOBCHK +3	# Negative non-zero falls through to check again

# CONTROL PASSES TO T6JOB ONLY WHEN C(TIME6) = -0 (I.E., WHEN A T6RUPT MUST BE PROCESSED).

; ============================================================================
; T6JOB - PROCESS PENDING TIMER 6 INTERRUPT
;
; This section executes when TIME6 reaches negative zero, indicating a T6
; interrupt is waiting to be processed. The code must carefully coordinate
; the next jet firing sequence, updating the queue of pending jet commands
; and writing the next jet firing pattern to the RCS control channels.
;
; Each interrupt processes one jet firing event from the queue. Multiple
; interrupts may occur in rapid succession during high-activity periods like
; the descent engine ignition or landing touchdown, when many jets fire
; simultaneously to counteract disturbances.
; ============================================================================

T6JOB		CAF	POSMAX		# DISABLE CLOCK: NEEDED SINCE RUPT OCCURS
		EXTEND			# 1 DINC AFTER T6 = 77777. FOR 625 MUSECS
		WAND	CHAN13		# MUST NOT HAVE T6 = +0 WITH ENABLE SET

; Advance the jet firing queue: move future events forward to become current.
; T6FURTHA (furthest event) → T6NEXT (next event) → TIME6 (current timer)
; This three-stage queue ensures smooth jet control even when multiple jets
; need to fire in rapid succession.

		CA	POSMAX		# Load POSMAX (largest positive value)
		ZL			# Zero L register for double precision ops
		DXCH	T6FURTHA	# Get furthest queued event time
		DXCH	T6NEXT		# Move to next event slot
		LXCH	NXT6ADR		# Get address of jet control routine
		TS	TIME6		# Set TIME6 to new countdown value

; Check if TIME6 value is valid (< PRIO37 threshold). If overflow occurs,
; disable timer; otherwise enable for next countdown.

		AD	PRIO37		# Check against priority threshold
		TS	A		# Store result
		TCF	ENABLET6	# No overflow: enable timer for countdown
		CA	POSMAX		# Overflow: disable timer
		TS	TIME6		# Set TIME6 to POSMAX (inactive state)
		TCF	GOCH56		# Jump to channel output routine

; Enable Timer 6 for next countdown cycle by setting enable bit in Channel 13.
; This allows the hardware timer to begin counting down from the value just
; loaded into TIME6, generating the next interrupt when the countdown reaches
; negative zero.

ENABLET6	CA	BIT15		# Get Timer 6 enable bit
		EXTEND			# Extended instruction follows
		WOR	CHAN13		# Write OR to Channel 13 (enable timer)
		CA	T6NEXT		# Check next queued event
		AD	PRIO37		# Test validity
		TS	A		# Store for overflow test
		TCF	GOCH56		# Valid: proceed to output
		CA	POSMAX		# Invalid: mark as inactive
		TS	T6NEXT		# Clear next event slot

; Write the jet firing command pattern to RCS control channels.
; The address in L register (NXT6ADR) determines which channel write routine
; to execute: WRITEP for Channel 6, WRITEU for Channel 5, or WRITEV for both.

GOCH56		INDEX	L		# Use L as index register
		TCF	WRITEP -1	# Transfer to appropriate write routine

		BLOCK	02
		SETLOC	FFTAG9
		BANK
		EBANK=	CDUXD
		COUNT*	$$/DAPT6

; ============================================================================
; WRITEP - WRITE JET FIRING PATTERN TO CHANNEL 6 (P-AXIS JETS)
;
; Channel 6 controls the pitch (P-axis) RCS jets. The bit pattern in NEXTP
; specifies which jets should fire: bits correspond to specific jet pairs.
; When a bit is set, the corresponding jet valve opens, allowing propellant
; to flow and produce thrust that rotates the LM in pitch.
;
; During Apollo 11's descent, these jets fired hundreds of times to maintain
; proper pitch attitude as the descent engine's thrust vector varied.
; ============================================================================

		CA	NEXTP		# Load next pitch jet pattern
WRITEP		EXTEND			# Extended instruction mode
		WRITE	CHAN6		# Write jet command to Channel 6 hardware
# Page 1405
		TC	Q		# Return to caller

; ============================================================================
; WRITEU - WRITE JET FIRING PATTERN TO CHANNEL 5 BITS 2,3,4 (U-AXIS JETS)
;
; Channel 5 controls translation (U-axis) jets in bits 2, 3, and 4, masked
; by octal 00314. The routine reads the current channel state, clears the
; U-axis jet bits, then sets the new pattern. This read-modify-write sequence
; preserves other control bits in Channel 5 while updating only jet commands.
;
; Translation jets move the LM sideways or up/down without rotating. During
; final approach to the lunar surface, these jets executed Armstrong's manual
; repositioning commands to avoid the boulder field.
; ============================================================================

		CA	NEXTU		# Load next U-axis jet pattern
WRITEU		TS	L		# Save in L register
		CS	00314OCT	# Get complement of U-axis bit mask
		EXTEND			# Extended instruction mode
		RAND	CHAN5		# Read Channel 5 and clear U-axis bits
		AD	L		# Add new jet pattern
		EXTEND			# Extended instruction mode
		WRITE	CHAN5		# Write updated pattern to Channel 5
		TC	Q		# Return to caller

; ============================================================================
; WRITEV - WRITE JET FIRING PATTERN TO CHANNEL 5 BITS 5,6,7 (V-AXIS JETS)
;
; Similar to WRITEU but controls V-axis translation jets in bits 5, 6, and 7.
; The mask 00314 (octal) = 11001100 (binary) covers both U and V axis bits.
; This routine uses the positive mask (not complemented) to set V-axis bits.
; ============================================================================

		CA	NEXTV		# Load next V-axis jet pattern
WRITEV		TS	L		# Save in L register
		CA	00314OCT	# Get V-axis bit mask (positive)
		TCF	-9D		# Jump back to common masking logic

00314OCT	OCT	00314		# Mask for U and V axis jets in Channel 5

		BANK	17
		SETLOC	DAPS2
		BANK

		EBANK=	T6NEXT
		COUNT*	$$/DAPT6

; ============================================================================
; DOT6RUPT - TIMER 6 INTERRUPT SERVICE ROUTINE
;
; This is the actual interrupt entry point, invoked by the AGC hardware when
; TIME6 counts down through negative zero. The interrupt lead-in code (in
; INTERRUPT_LEAD_INS.agc) transfers control here via the T6RUPT vector at
; location 4004. This routine saves interrupt context, calls T6JOBCHK to
; process the pending jet command, then restores context and returns.
;
; Every Timer 6 interrupt processes exactly one jet firing event. During
; high-activity periods (engine ignition, landing impact, major attitude
; changes), T6 interrupts may occur every few milliseconds. The interrupt
; priority system ensures jet timing remains precise even when competing with
; guidance computations and display updates.
;
; During Apollo 11's powered descent, this interrupt fired continuously,
; coordinating jet pulses that kept Eagle stable while the descent engine
; throttled from full thrust down to near-idle for the final approach.
; ============================================================================

DOT6RUPT	LXCH	BANKRUPT	# (INTERRUPT LEAD INS CONTINUED)
		EXTEND			# Save interrupted program's bank
		QXCH	QRUPT		# Save return address

		TC	T6JOBCHK	# CALL T6JOBCHK to process interrupt

		TCF	RESUME		# END TIME6 INTERRUPT PROCESSOR.

