# Copyright:	Public domain.
# Filename:	KEYRUPT_UPRUPT.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1338-1340
# Mod history:	2009-05-27 RSB	Adapted from the corresponding
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
; FILE: KEYRUPT_UPRUPT.agc
; MODULE: Interrupt Handlers - Crew and Ground Input
; MISSION PHASE: All phases (launch through landing through ascent)
;
; TL;DR: Implements two critical interrupt handlers for human input to the
;        Apollo Guidance Computer. KEYRUPT processes astronaut button presses
;        on the DSKY (Display and Keyboard), enabling crew control during all
;        mission phases. UPRUPT handles ground station uplink commands from
;        Mission Control, with triple-character redundancy validation and
;        lockout protection against transmission errors.
;
; COMMENT-ONLY READERS: This code manages how astronaut commands and ground
;        control commands reach the computer. Essential throughout the mission.
; CODE-ALONG READERS: Study interrupt priority handling, input buffering, and
;        the triple redundancy validation scheme for uplink data integrity.
; ============================================================================

# Page 1338
		BANK	14
		SETLOC	KEYRUPT
		BANK
		COUNT*	$$/KEYUP

; ============================================================================
; KEYRUPT INTERRUPT HANDLER
;
; The astronauts control the Apollo Guidance Computer through the DSKY
; (Display and Keyboard) unit. When Armstrong or Aldrin presses any button
; (VERB, NOUN, numeric keys 0-9, +/-, ENTR, CLR, KEY REL, RSET), the DSKY
; hardware triggers a keyboard interrupt. This interrupt handler captures
; that button press and routes it for processing by the display system.
;
; During critical mission phases like lunar landing, the crew uses the DSKY
; to monitor altitude and velocity (Verb 16 Noun 60), request program changes
; (Verb 37), and acknowledge alarms. Every button press flows through here.
; ============================================================================

KEYRUPT1	TS	BANKRUPT
		XCH	Q
		TS	QRUPT
		TC	LODSAMPT	# TIME IS SNATCHED IN RUPT FOR NOUN 65.
;
; Interrupt context save: Store A register and return address (Q register)
; to preserve state before processing the keyboard input. This allows the
; interrupted program to resume exactly where it left off after button
; handling completes.
;
		CAF	LOW5
		EXTEND
		RAND	MNKEYIN		# CHECK IF KEYS 5M-1M ON
;
; Read the keyboard input channel (MNKEYIN) to determine which button was
; pressed. The DSKY hardware encodes button presses as 5-bit codes in the
; low 5 bits of the input channel. Mask with LOW5 to extract button code.
;
KEYCOM		TS	RUPTREG4
		CS	FLAGWRD5
		MASK	DSKYFBIT
		ADS	FLAGWRD5
;
; Set the DSKY FLAG (DSKYFBIT in FLAGWRD5) to indicate fresh keyboard input
; is available. The display system monitors this flag to know when crew
; input requires processing. This flag ensures no button presses are lost
; even if the computer is busy with guidance calculations.
;

ACCEPTUP	CAF	CHRPRIO		# (NOTE: RUPTREG4 = KEYTEMP1)
		TC	NOVAC
		EBANK=	DSPCOUNT
		2CADR	CHARIN
;
; Schedule a background job to process the keyboard input through the CHARIN
; routine (CHARacter INput processor). NOVAC finds an available job slot in
; the executive scheduler and assigns CHRPRIO priority to this task.
;
; The button code is passed to CHARIN via MPAC. CHARIN will interpret the
; 5-bit code and update the DSKY display accordingly, whether the crew is
; entering a verb, noun, or numerical data during mission operations.
;
		CA	RUPTREG4
		INDEX	LOCCTR
		TS	MPAC		# LEAVE 5 BIT KEY CDE IN MPAC FOR CHARIN
;
; Store the 5-bit keyboard code in MPAC for CHARIN to process. The INDEX
; instruction provides indirect addressing through LOCCTR to place the code
; in the correct MPAC location for the scheduled job.
;
		TC	RESUME
;
; Return from interrupt, restoring the interrupted program's context.
; The keyboard button press is now queued for processing, and normal
; program execution continues (guidance, navigation, or display updates).
;

# Page 1339
# UPRUPT PROGRAM

; ============================================================================
; TRANSITION: From Keyboard Input to Ground Station Uplink
;
; While KEYRUPT handles crew input from inside the spacecraft, UPRUPT manages
; commands from Mission Control in Houston. During the Apollo 11 mission,
; ground controllers used uplink to send state vector updates, target loads
; for maneuvers, and time-critical information. This ground-to-spacecraft
; data link was essential throughout the mission.
; ============================================================================

; ============================================================================
; UPRUPT INTERRUPT HANDLER - GROUND STATION UPLINK
;
; Mission Control communicates with the LM through the uplink data channel.
; When the ground station transmits a command word, the AGC's receiver
; triggers an uplink interrupt. This handler validates the received data
; using triple-character redundancy (each 5-bit code transmitted three times)
; to protect against radio transmission errors.
;
; Critical uplink commands during Apollo 11 included navigation updates,
; program loads, and the ERROR RESET code that could unlock the uplink after
; transmission failures. This validation system prevented garbled radio
; signals from corrupting the guidance computer's memory.
; ============================================================================

UPRUPT		TS	BANKRUPT
		XCH	Q
		TS	QRUPT
		TC	LODSAMPT	# TIME IS SNATCHED IN RUPT FOR NOUN 65.
;
; Interrupt context save: Store registers as with KEYRUPT. The uplink
; interrupt has lower priority than timer interrupts but higher than
; keyboard, reflecting the importance of ground commands.
;
		CAF	ZERO
		XCH	INLINK
		TS	KEYTEMP1
;
; Read the uplink word from INLINK channel and clear it atomically using XCH
; instruction. The uplink word contains 15 bits: three copies of a 5-bit
; character code sent with triple redundancy for error detection. Store the
; complete word in KEYTEMP1 for validation processing.
;
		CAF	BIT3		# TURN ON UPACT LIGHT
		EXTEND			# (BIT 3 OF CHANNEL 11)
		WOR	DSALMOUT
;
; Illuminate the UPLINK ACTIVITY indicator light on the DSKY. This informs
; the crew that Mission Control is sending commands. Armstrong and Aldrin
; would see this light during state vector updates and program loads. The
; light remains on until the update program completes or an error reset.
;
; ============================================================================
; TRIPLE CHARACTER REDUNDANCY VALIDATION
;
; The uplink data format sends each 5-bit character three times in one 15-bit
; word: [5-bit code][5-bit code][5-bit code]. This triple redundancy protects
; against radio noise and transmission errors during the quarter-million mile
; journey between Earth and the Moon. The following code validates that all
; three copies match exactly.
; ============================================================================

UPRPT1		CAF	LOW5		# TEST FOR TRIPLE CHAR REDUNDANCY
		MASK	KEYTEMP1	# LOW5 OF WORD
		XCH	KEYTEMP1	# LOW5 INTO KEYTEMP1
;
; Extract the low 5 bits (first character copy) from the uplink word.
; After masking, KEYTEMP1 contains only the first 5-bit code, which
; becomes the reference for comparison with the other two copies.
;
		EXTEND
		MP	BIT10		# SHIFT RIGHT 5
		TS	KEYTEMP2
		MASK	LOW5		# MID 5
		AD	HI10
		TC	UPTEST
;
; Extract the middle 5 bits (second character copy) by shifting the original
; word right 5 positions and masking. Compare this middle copy against the
; low copy by calling UPTEST. If they don't match, the uplink word is bad
; and will be rejected (TMFAIL2 path).
;
		CAF	BIT10
		EXTEND
		MP	KEYTEMP2	# SHIFT RIGHT 5
		MASK	LOW5		# HIGH 5
		COM
		TC	UPTEST
;
; Extract the high 5 bits (third character copy) by shifting right another
; 5 positions. Complement this value and compare with KEYTEMP1 through
; UPTEST. The UPTEST routine adds the values; if all three copies match,
; the sum works out correctly and the code is accepted.
;

; ============================================================================
; UPLINK VALIDATION PASSED - CHECK FOR ERROR RESET OR LOCKOUT STATE
;
; The redundancy check succeeded: all three character copies matched. Now
; determine if this is a special ERROR RESET code from Mission Control, or
; a normal uplink command that must be checked against the lockout flag.
; ============================================================================

UPCK		CS	ELRCODE		# CODE IS GOOD. IF CODE = 'ERROR RESET',
		AD	KEYTEMP1	# CLEAR UPLOCKFL(SET BIT4 OF FLAGWRD7 = 0)
		EXTEND			# IF CODE DOES NOT = 'ERROR RESET', ACCEPT
		BZF	CLUPLOCK	# CODE ONLY IF UPLOCKFL IS CLEAR (=0).
;
; Check if the received code equals ELRCODE (octal 22, the ERROR RESET code).
; If it matches, branch to CLUPLOCK to clear the uplink lockout flag. This
; allows Mission Control to re-enable uplink after a previous transmission
; failure locked out further uplink activity for safety.
;
		CAF	UPLOCBIT	# TEST UPLOCKFL FOR 0 OR 1
		MASK	FLAGWRD7
		CCS	A
		TC	RESUME		# UPLOCKFL = 1
		TC	ACCEPTUP	# UPLOCKFL = 0
;
; For normal commands (not ERROR RESET), check the uplink lockout flag
; (UPLOCKFL, bit 4 of FLAGWRD7). If the flag is set (=1), uplink is locked
; and the command is rejected. If clear (=0), uplink is enabled and the
; command proceeds to ACCEPTUP for processing by the update program.
;
; This lockout mechanism prevents corrupted commands from reaching the
; computer after a transmission error, waiting for ground confirmation.
;

CLUPLOCK	CS	UPLOCBIT	# CLEAR UPLOCKFL (I.E.,SET BIT 4 OF )
		MASK	FLAGWRD7	# FLAGWRD7 = 0)
		TS	FLAGWRD7
		TC	ACCEPTUP
;
; ERROR RESET code received from Mission Control. Clear the uplink lockout
; flag (bit 4 of FLAGWRD7) to re-enable uplink commands. This code path is
; critical when Mission Control detects transmission problems and needs to
; unlock the uplink channel before sending corrective data.
;
; After clearing UPLOCKFL, continue to ACCEPTUP to process the ERROR RESET
; code itself through the normal update program path.
;

; ============================================================================
; TMFAIL2 - UPLINK REDUNDANCY CHECK FAILURE HANDLER
;
; If the three character copies in the uplink word don't match, the data is
; corrupted and must be rejected. This routine sets the uplink lockout flag
; to prevent further commands until Mission Control sends an ERROR RESET code.
; During critical mission phases like descent or ascent, a locked uplink
; required immediate ground controller attention to restore communications.
; ============================================================================

					# CODE IS BAD
TMFAIL2		CS	FLAGWRD7	# LOCK OUT FURTHER UPLINK ACTIVITY
		MASK	UPLOCBIT	# (BY SETTING UPLOCKFL = 1) UNTIL
		ADS	FLAGWRD7	# 'ERROR RESET' IS SENT VIA UPLINK.
;
; Transmission error detected. Set the uplink lockout flag (UPLOCKFL, bit 4
; of FLAGWRD7) to 1. This prevents the AGC from accepting any further uplink
; commands until Mission Control sends the ERROR RESET code. The lockout
; protects the computer from processing corrupted navigation data or program
; loads that could jeopardize the mission.
;
; The DSKY UPLINK ACTIVITY light remains on, and the downlink telemetry sends
; all flagwords to the ground, alerting Mission Control of the lockout state.
;
		TC	RESUME
;
; Resume interrupted program. The corrupted uplink word is discarded and the
; crew/ground must retransmit. This safety mechanism was critical during the
; Apollo 11 mission when reliable ground-to-spacecraft data transfer was
; essential for navigation accuracy.
; ============================================================================
; UPTEST - TRIPLE REDUNDANCY COMPARISON SUBROUTINE
;
; This subroutine is called twice during uplink validation to compare the
; three character copies. Through careful arithmetic, it determines if all
; three 5-bit codes match. If they match, it returns to the caller (TC Q).
; If they don't match, it branches to TMFAIL2 to lock out uplink activity.
; ============================================================================

UPTEST		AD	KEYTEMP1
;
; Add the incoming value (in A register) to KEYTEMP1 (the first character
; copy). The calling code has prepared values such that if all three copies
; match, this arithmetic produces a predictable result (zero after adding
; the constant HI10 or after complementing and adding).
;
# Page 1340
		CCS	A
		TC	TMFAIL2
HI10		OCT	77740
		TC	TMFAIL2
		TC	Q
;
; Count, compare, and skip (CCS) tests the sum. If positive or negative
; (non-zero after accounting for HI10 constant), the character copies don't
; match - branch to TMFAIL2. If zero (match), fall through to TC Q which
; returns to caller to continue validation or accept the uplink code.
;
; The HI10 constant (octal 77740) is strategically placed in the CCS test
; sequence. This octal value equals -40 decimal in ones' complement, which
; when added during the first UPTEST call, creates the correct arithmetic
; to detect mismatches in the triple-redundant character codes.
;

; ============================================================================
; UPLINK SYSTEM CONSTANTS AND OPERATIONAL NOTES
; ============================================================================

ELRCODE		OCT	22
;
; ERROR RESET code (octal 22). When Mission Control transmits this special
; code, the AGC clears the uplink lockout flag and re-enables the uplink
; channel. This code is critical for recovering from transmission errors.
;

# 'UPLINK ACTIVITY LIGHT' IS TURNED OFF BY .....
#	   1.	  VBRELDSP
#	   2.	  ERROR RESET
#	   3.	  UPDATE PROGRAM(P27) ENTERED BY V70,V71,V72,AND V73.
#
; COMMENT-ONLY READERS: The UPLINK ACTIVITY light on the DSKY informs the crew
; that Mission Control is sending data. It remains lit during transmission and
; is turned off when: (1) the verb display is released, (2) ERROR RESET is
; received, or (3) the crew enters the update program (P27) via specific verb
; sequences. This provides visual confirmation of ground-to-spacecraft data
; transfer, essential during Apollo 11's navigation updates.
;
#				    _
# THE RECEPTION OF A BAD CODE(I.E  CCC FAILURE) LOCKS OUT FURTHER UPLINK ACTIVITY BY SETTING BIT4 OF FLAGWRD7 = 1.
# THIS INDICATION WILL BE TRANSFERRED TO THE GROUND BY THE DOWNLINK WHICH DOWNLINKS ALL FLAGWORDS.
# WHEN UPLINK ACTIVITY IS LOCKED OUT ,IT CAN BE ALLOWED WHEN THE GROUND UPLINKS AND 'ERROR RESET' CODE.
# (IT IS RECOMMENDED THAT THE 'ERROR LIGHT RESET' CODE IS PRECEEDED BY 16 BITS THE FIRST OF WHICH IS 1 FOLLOWED
# BY 15 ZEROES. THIS WILL ELIMINATE EXTRANEOUS BITS FROM INLINK WHICH MAY HAVE BEEN LEFT OVER FROM THE ORIGINAL
# FAILURE)
# UPLINK ACTIVITY IS ALSO ALLOWED(UNLOCKED) DURING FRESH START WHEN FRESH START SETS BIT4 OF FLAGWRD7 = 0.
;
; CODE-ALONG READERS: The triple-character redundancy check (CCC) validates
; uplink data integrity. If validation fails, UPLOCKFL (bit 4 of FLAGWRD7)
; is set to 1, locking out all further uplink commands until ERROR RESET.
; The lockout state is downlinked to Mission Control via telemetry.
;
; Ground controllers can unlock the uplink by sending ERROR RESET (octal 22),
; preferably preceded by a 16-bit sequence (1 followed by 15 zeros) to clear
; residual bits in INLINK from the original transmission failure. The FRESH
; START routine also unlocks uplink by clearing bit 4 of FLAGWRD7.
;
		CS	XDSPBIT
