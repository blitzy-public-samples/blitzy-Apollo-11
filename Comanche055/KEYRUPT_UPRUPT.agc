# Copyright:    Public domain.
# Filename:     KEYRUPT_UPRUPT.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1449-1451
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-07 RSB	Adapted from Colossus249 file of the same
#				name, and page images. Corrected various
#				typos in the transcription of program
#				comments, and these should be back-ported
#				to Colossus249.
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
; FILE: KEYRUPT_UPRUPT.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Keyboard interrupt handler (KEYRUPT) for DSKY button presses and
;        uplink interrupt handler (UPRUPT) for ground command reception.
;        Provides input buffering and interrupt-driven handling for crew
;        interface and Mission Control communications throughout Apollo 11.
;
; COMMENT-ONLY READERS: This handled every button press from astronauts on
;        the spacecraft's keyboard and every command sent from Mission Control
;        on Earth, enabling continuous communication throughout the mission.
; CODE-ALONG READERS: Study interrupt handlers for crew DSKY input and ground
;        uplink commands, input buffering mechanisms, triple-character
;        redundancy checking, and uplink lock-out protection.
; ============================================================================

# Page 1449
		BANK	14
		SETLOC	KEYRUPT
		BANK
		COUNT*	$$/KEYUP

; ============================================================================
; KEYRUPT1 - KEYBOARD INTERRUPT HANDLER
;
; COMMENT-ONLY READERS: This routine activates instantly whenever an astronaut
; presses any button on the DSKY (Display and Keyboard) panel - whether typing
; a program number, entering navigation data, or requesting information. Every
; button press from Michael Collins in Columbia's cabin during Apollo 11 was
; processed through this interrupt handler.
;
; CODE-ALONG READERS: KEYRUPT1 is a priority interrupt handler triggered when
; crew presses DSKY keys. It preserves interrupt context, captures time for
; NOUN 65 timing displays, reads the 5-bit key code from channel 15, schedules
; CHARIN job via NOVAC to process the keystroke, then resumes normal operation.
; Interrupt priority allows immediate response to crew input.
; ============================================================================

KEYRUPT1	TS	BANKRUPT
		XCH	Q
		TS	QRUPT
; Interrupt context now saved. BANKRUPT preserves bank register, QRUPT holds
; return address. This allows KEYRUPT to safely call subroutines and access
; other memory banks without corrupting the interrupted program's state.

		TC	LODSAMPT	# TIME IS SNATCHED IN RUPT FOR NOUN 65.
; Capture current mission time for NOUN 65 displays. NOUN 65 shows time of last
; keyboard activity, allowing crew to verify DSKY responsiveness and providing
; timestamp for logged commands. This timing was crucial during critical mission
; phases when crew needed to confirm system status.

		CAF	LOW5
		EXTEND
		RAND	MNKEYIN		# CHECK IF KEYS 5M-1M ON
; Read 5-bit key code from MNKEYIN channel (channel 15). Each DSKY button has
; unique 5-bit code (values 0-31). RAND instruction performs logical AND to
; extract only the low 5 bits containing the key identifier.

KEYCOM		TS	RUPTREG4
		CS	FLAGWRD5
		MASK	BIT15
		ADS	FLAGWRD5
; Store key code in RUPTREG4 for later processing. Update FLAGWRD5 bit 15 to
; indicate keyboard activity detected. This flag notifies display routines that
; new input is available for processing.

; ============================================================================
; ACCEPTUP - COMMON KEY/UPLINK PROCESSING
;
; Both keyboard and uplink interrupts converge here to schedule character
; processing. This shared code path ensures consistent handling whether input
; comes from astronaut button presses or ground controller transmissions.
; ============================================================================

ACCEPTUP	CAF	CHRPRIO		# (NOTE: RUPTREG4 = KEYTEMP1)
		TC	NOVAC
		EBANK=	DSPCOUNT
		2CADR	CHARIN
; Schedule CHARIN job via NOVAC (executive scheduler) at CHRPRIO priority.
; CHARIN processes the character by updating displays, executing verbs, loading
; nouns, or handling special functions. NOVAC allows interrupt handler to return
; quickly while CHARIN runs under executive control. The 2CADR provides both
; bank and address for CHARIN entry point.

		CA	RUPTREG4
		INDEX	LOCCTR
		TS	MPAC		# LEAVE 5 BIT KEY CDE IN MPAC FOR CHARIN
; Transfer key code from RUPTREG4 into MPAC (multi-purpose accumulator) where
; CHARIN expects to find it. INDEX LOCCTR provides proper MPAC location based
; on current core set. This completes the handoff from interrupt handler to
; scheduled job.

		TC	RESUME
; Return from interrupt. RESUME restores saved registers (A, Q, bank) and
; returns control to the interrupted program at the exact instruction where it
; was preempted. From the interrupted program's perspective, no time has passed.

# Page 1450
# UPRUPT PROGRAM

; ============================================================================
; UPRUPT - UPLINK INTERRUPT HANDLER
;
; COMMENT-ONLY READERS: While astronauts could control the spacecraft locally,
; Mission Control in Houston maintained constant communication with Columbia
; via radio uplink. This routine processed every command sent from Earth -
; navigation updates, program changes, emergency procedures. During Apollo 11's
; translunar coast, lunar orbit, and return journey, ground controllers used
; uplink to send state vector updates and mission-critical commands to keep
; the crew on course for their historic landing.
;
; CODE-ALONG READERS: UPRUPT handles ground command reception via S-band uplink.
; Unlike KEYRUPT's simple 5-bit key codes, uplink uses 15-bit words with
; triple-character redundancy (three copies of 5-bit code) for error detection.
; This redundancy is essential given the 240,000-mile transmission distance and
; potential radio interference. UPRUPT validates redundancy, manages uplink
; activity light (bit 3 of channel 11), and enforces uplink lock-out protection
; to prevent command corruption after transmission errors.
; ============================================================================

UPRUPT		TS	BANKRUPT
		XCH	Q
		TS	QRUPT
; Save interrupt context (bank register and return address) just like KEYRUPT.
; All interrupt handlers must preserve these registers to allow safe return to
; the interrupted program regardless of which bank or subroutine was executing.

		TC	LODSAMPT	# TIME IS SNATCHED IN RUPT FOR NOUN 65.
; Capture mission time for NOUN 65 uplink activity timestamp. This allows crew
; and ground controllers to verify uplink reception timing and troubleshoot
; communication issues. During Apollo 11, tracking uplink timing helped Mission
; Control coordinate their transmissions with optimal antenna coverage windows.

		CAF	ZERO
		XCH	INLINK
		TS	KEYTEMP1
; Read 15-bit uplink word from INLINK channel and clear channel to zero in one
; atomic exchange operation. The 15-bit word contains three copies of a 5-bit
; character code for triple-redundancy error detection. KEYTEMP1 now holds the
; received word for redundancy validation.

		CAF	BIT3		# TURN ON UPACT LIGHT
		EXTEND			# (BIT 3 OF CHANNEL 11)
		WOR	DSALMOUT
; Illuminate UPLINK ACTIVITY indicator light on DSKY panel (bit 3 of channel 11)
; to signal crew that ground transmission received. Michael Collins would see
; this light flash during uplink reception, confirming radio contact with Earth.
; Light remains on until explicitly turned off by verb execution, error reset,
; or update program entry (see comments at end of file).
; ============================================================================
; UPRPT1 - TRIPLE CHARACTER REDUNDANCY VALIDATION
;
; COMMENT-ONLY READERS: Radio transmission from Earth to the Moon travels
; 240,000 miles through space where cosmic rays and solar radiation can corrupt
; data. To ensure command integrity, every uplink character is sent three times
; in a single 15-bit word. The computer checks that all three copies match
; exactly before accepting the command. A single mismatched bit triggers uplink
; lockout, preventing corrupted commands from affecting spacecraft systems.
; Mission Control must then send a special ERROR RESET code to restore uplink
; capability.
;
; CODE-ALONG READERS: The 15-bit uplink word contains three 5-bit copies of
; the same character code: bits 0-4 (low 5), bits 5-9 (mid 5), bits 10-14
; (high 5). This routine extracts each 5-bit section, performs pairwise
; comparison, and validates that all three copies are identical. Uses shift-right
; operations via multiplication by BIT10 (octal 400 = shift right 5 positions).
; ============================================================================

UPRPT1		CAF	LOW5		# TEST FOR TRIPLE CHAR REDUNDANCY
		MASK	KEYTEMP1	# LOW5 OF WORD
		XCH	KEYTEMP1	# LOW5 INTO KEYTEMP1
; Extract low 5 bits (bits 0-4) from received uplink word. LOW5 mask (octal 37)
; isolates these bits. XCH simultaneously stores extracted code in KEYTEMP1 and
; loads remaining bits into A register for further processing.

		EXTEND
		MP	BIT10		# SHIFT RIGHT 5
		TS	KEYTEMP2
; Multiply by BIT10 (octal 400) which shifts remaining bits right by 5 positions.
; This moves bits 5-14 into positions 0-9 for mid-5 extraction. Result saved
; in KEYTEMP2 for second extraction pass.

		MASK	LOW5		# MID 5
		AD	HI10
		TC	UPTEST
; Extract mid 5 bits (bits 5-9 from original, now in positions 0-4). Add HI10
; (octal 77740) to form comparison value, then call UPTEST to verify mid-5
; matches low-5. HI10 constant facilitates the comparison logic in UPTEST.

		CAF	BIT10
		EXTEND
		MP	KEYTEMP2	# SHIFT RIGHT 5
; Shift right 5 more positions to expose high 5 bits (bits 10-14 from original,
; now in positions 0-4). Second multiplication by BIT10 completes extraction of
; all three redundant copies.

		MASK	LOW5		# HIGH 5
		COM
		TC	UPTEST
; Extract high 5 bits, complement the value (COM instruction), and verify
; against low-5 copy via UPTEST. Complementing simplifies the comparison logic.
; If all three copies match, redundancy validation passes and code is accepted.

; ============================================================================
; UPOK - PROCESS VALIDATED UPLINK CODE
;
; COMMENT-ONLY READERS: Once the computer confirms all three character copies
; match, it must check whether uplink commands are currently locked out. If a
; previous transmission error occurred, the uplink system enters protective
; lockout mode - refusing all commands until Mission Control sends the special
; ERROR RESET code (octal code 22). This safety feature prevents cascading
; errors from corrupted commands. During Apollo 11, maintaining reliable uplink
; was critical for navigation updates and mission timeline coordination with
; Houston.
;
; CODE-ALONG READERS: After redundancy validation passes, UPOK implements two-
; stage protection logic. First checks if received code equals ELRCODE (Error
; Light Reset, octal 22). If match, branches to CLUPLOCK to clear UPLOCKFL
; flag and restore uplink capability. If not ERROR RESET, tests UPLOCKFL state
; (bit 4 of FLAGWRD7). If flag set (=1), uplink locked, ignore command and
; RESUME. If flag clear (=0), uplink active, proceed to ACCEPTUP for normal
; character processing. This protects against command corruption after detected
; transmission errors.
; ============================================================================

UPOK		CS	ELRCODE		# CODE IS GOOD. IF CODE = 'ERROR RESET',
		AD	KEYTEMP1	# CLEAR UPLOCKFL(SET BIT4 OF FLAGWRD7 = 0)
		EXTEND			# IF CODE DOES NOT = 'ERROR RESET', ACCEPT
		BZF	CLUPLOCK	# CODE ONLY IF UPLOCKFL IS CLEAR (=0).
; Compare received code (KEYTEMP1) against ELRCODE (octal 22) by computing
; complement-sum (CS ELRCODE plus AD KEYTEMP1). If result zero (BZF), codes
; match - this is ERROR RESET command, branch to CLUPLOCK to restore uplink.
; If non-zero, proceed to lock status check.

		CAF	BIT4		# TEST UPLOCKFL FOR 0 OR 1.
		MASK	FLAGWRD7
		CCS	A
; Test uplink lock flag UPLOCKFL (bit 4 of FLAGWRD7). MASK isolates bit 4,
; CCS (Count, Compare, Skip) tests if result positive (flag set = locked) or
; zero (flag clear = active).

		TC	RESUME		# UPLOCKFL = 1
; Uplink locked (UPLOCKFL = 1) due to previous error. Ignore this command,
; turn off UPACT light via normal interrupt exit path, and return to interrupted
; program. Command discarded for safety. Ground controllers must send ERROR
; RESET before further commands accepted.

		TC	ACCEPTUP	# UPLOCKFL = 0
; Uplink active (UPLOCKFL = 0). Valid command received with correct redundancy
; and no lockout condition. Branch to ACCEPTUP (shared with KEYRUPT handler)
; to schedule CHARIN job for character processing. From here, uplink commands
; follow same processing path as keyboard input.

; ============================================================================
; CLUPLOCK - CLEAR UPLINK LOCK FLAG (RESTORE UPLINK CAPABILITY)
;
; COMMENT-ONLY READERS: When Mission Control sends the special ERROR RESET
; code after an uplink transmission error, this routine restores the command
; link. The uplink activity light on the DSKY turns on whenever uplink data
; arrives. Once the ERROR RESET is processed, astronauts can again receive
; navigation updates, program commands, and state vector corrections from
; Houston. During Apollo 11, maintaining this Earth-spacecraft command link
; was essential for course corrections and mission timeline coordination.
;
; CODE-ALONG READERS: Clears UPLOCKFL flag (bit 4 of FLAGWRD7) by computing
; complement of BIT4 (resulting in mask with bit 4 = 0, all others = 1), then
; MASK with current FLAGWRD7 to preserve other flags while clearing bit 4.
; After unlock, proceeds to ACCEPTUP to process the ERROR RESET code itself
; through normal character input path.
; ============================================================================

CLUPLOCK	CS	BIT4		# CLEAR UPLOCKFL (I.E., SET BIT 4 OF
		MASK	FLAGWRD7	# FLAGWRD7 = 0)
		TS	FLAGWRD7
; Clear uplink lock flag: CS BIT4 produces mask (octal 77773) with bit 4 zero.
; MASK with FLAGWRD7 preserves bits 0-3 and 5-14 while forcing bit 4 to zero.
; Store result back to FLAGWRD7, unlocking uplink command reception.

		TC	ACCEPTUP
; Proceed to ACCEPTUP to schedule CHARIN job for processing the ERROR RESET
; code itself. Uplink now restored to normal operation.

; ============================================================================
; TMFAIL2 - TRIPLE CHARACTER COMPARISON FAILURE (LOCK OUT UPLINK)
;
; COMMENT-ONLY READERS: When the computer detects that the three copies of an
; uplink character don't match, it means cosmic radiation or transmission error
; corrupted the data during the 240,000-mile journey from Earth. Rather than
; risk executing a corrupted command that could endanger the spacecraft, the
; computer immediately locks out all further uplink commands. The astronauts
; see the UPLINK ACTIVITY light illuminate on the DSKY. Mission Control, seeing
; the lock flag in telemetry downlink data, must diagnose the issue and send
; an ERROR RESET code (preceded by a clearing pattern) to restore the command
; link. During Apollo 11, this protection prevented potentially catastrophic
; command corruption.
;
; CODE-ALONG READERS: Sets UPLOCKFL flag (bit 4 of FLAGWRD7 = 1) to lock out
; uplink. Computes complement of FLAGWRD7 (CS instruction), masks bit 4 to
; isolate that bit position, then adds (ADS) back to FLAGWRD7 to set the bit
; while preserving other flag states. After setting lock flag, executes RESUME
; to exit interrupt and return to interrupted program. Corrupted code discarded,
; UPACT light remains on, telemetry downlinks lock status to ground.
; ============================================================================

					# CODE IS BAD
TMFAIL2		CS	FLAGWRD7	# LOCK OUT FURTHER UPLINK ACTIVITY
		MASK	BIT4		# (BY SETTING UPLOCKFL = 1) UNTIL
		ADS	FLAGWRD7	# 'ERROR RESET' IS SENT VIA UPLINK.
; Set uplink lock flag: CS FLAGWRD7 produces complement, MASK BIT4 isolates
; bit 4 position, ADS (Add to Storage) adds this to FLAGWRD7, forcing bit 4 = 1
; while preserving other flags. Uplink now locked until ERROR RESET received.

		TC	RESUME
; Exit interrupt via RESUME. Corrupted code discarded, UPACT light left on,
; UPLOCKFL downlinked to ground via telemetry for ground controllers to see
; uplink failure status.
; ============================================================================
; UPTEST - COMPARE CHARACTER COPIES FOR REDUNDANCY VALIDATION
;
; COMMENT-ONLY READERS: This small comparison routine is called twice during
; uplink character validation - once to compare the middle copy against the low
; copy, and once to compare the high copy against the low copy. If any
; comparison fails, the entire character is rejected and uplink locks out. Only
; when both comparisons pass does the computer accept the command.
;
; CODE-ALONG READERS: Comparison subroutine called via TC (Transfer Control)
; from UPRPT1 redundancy validation loop. On entry, A register contains one of
; the extracted 5-bit character copies (either mid-5 with HI10 added, or high-5
; complemented). Adds KEYTEMP1 (contains low-5 copy for reference). Uses CCS
; (Count, Compare, Skip) to test result: if non-zero (copies don't match),
; branches to TMFAIL2 to lock uplink. If zero (copies match), falls through to
; TC Q to return to caller. HI10 constant (octal 77740) acts as both comparison
; facilitator and failure branch target via clever encoding.
; ============================================================================

UPTEST		AD	KEYTEMP1
; Add reference copy (low 5 bits in KEYTEMP1) to extracted copy in A register.
; Calling routine has prepared A register with appropriate value (mid-5 + HI10,
; or complemented high-5). If copies match, sum produces predictable value that
; will test as zero after CCS adjustments.

# Page 1451
		CCS	A
; Count, Compare, Skip instruction tests sum result. Four possible branches:
; +value: copies don't match, branch to TMFAIL2 (next instruction)
; +zero: would skip one, but HI10 constant here causes TMFAIL2 branch
; -zero: would skip two, executes TC Q (return to caller, comparison passed)
; -value: copies don't match, branch to TMFAIL2 (instruction after HI10)

		TC	TMFAIL2
; CCS result positive (non-zero): character copies don't match. Branch to
; TMFAIL2 to set uplink lock flag and discard corrupted code.

HI10		OCT	77740
; Constant used in redundancy checking. Value octal 77740 (binary 111111111110000)
; serves dual purpose: added to mid-5 extraction to facilitate comparison logic,
; and positioned here to catch certain CCS branch cases for comparison failure.
; Elegant encoding reduces code size while maintaining comparison accuracy.

		TC	TMFAIL2
; CCS result negative (non-zero): character copies don't match. Branch to
; TMFAIL2 to lock uplink.

		TC	Q
; CCS result minus-zero: character copies match. Return to caller via Q register
; (return address). Caller proceeds to next comparison or acceptance.

; ============================================================================
; ELRCODE - ERROR LIGHT RESET CODE CONSTANT
;
; CODE-ALONG READERS: Special uplink code value (octal 22) that clears the
; uplink lock flag (UPLOCKFL) after transmission error. When UPOK routine
; detects received code matches ELRCODE, branches to CLUPLOCK to restore uplink
; capability. Ground controllers must send this code to recover from triple-
; character redundancy validation failure.
; ============================================================================

ELRCODE		OCT	22
; Error Light Reset code = octal 22. Special command value recognized by UPOK
; routine to unlock uplink after transmission error lockout.

; ============================================================================
; UPLINK ACTIVITY LIGHT CONTROL AND ERROR RECOVERY PROCEDURES
;
; COMMENT-ONLY READERS: The UPLINK ACTIVITY (UPACT) light on the spacecraft's
; DSKY illuminates whenever data arrives from Earth, providing visual feedback
; to astronauts that Mission Control is communicating. This light turns off
; automatically when display updates complete, when ERROR RESET is processed,
; or when navigation update programs (P27) are entered by the crew. If uplink
; transmission errors occur (detected by triple-character mismatch), the
; computer locks out all further uplink commands to protect mission safety.
; Ground controllers see this lockout status in telemetry and must send a
; special ERROR RESET code - preceded by a clearing bit pattern - to restore
; the command link. During Apollo 11, this protection system ensured that only
; verified, uncorrupted commands from Houston could affect spacecraft systems
; during the mission's critical phases.
;
; CODE-ALONG READERS: Original NASA comments document UPACT light control and
; error recovery procedures. Light extinguished by: (1) VBRELDSP display release
; routine, (2) ERROR RESET code reception, (3) P27 update program entry via
; V70/V71/V72/V73 verbs. Triple-character redundancy failure (CCC failure) sets
; UPLOCKFL (bit 4 of FLAGWRD7 = 1), locking uplink. Flag downlinked to ground
; via telemetry. Recovery requires ground uplink of ERROR RESET (octal 22),
; recommended preceded by clearing pattern (1 followed by 15 zeros) to flush
; residual bits from INLINK register. FRESH START also clears UPLOCKFL during
; system initialization, providing alternative unlock path during restart.
; ============================================================================

# 'UPLINK ACTIVITY LIGHT' IS TURNED OFF BY .....
#	1.	VBRELDSP
#	2.	ERROR RESET
#	3.	UPDATE PROGRAM(P27) ENTERED BY V70,V71,V72,AND V73.
#
#
#					-
# THE RECEPTION OF A BAD CODE(I.E. CCC FAILURE) LOCKS OUT FURTHER UPLINK ACTIVITY BY SETTING BIT4 OF FLAGWRD7 = 1.
# THIS INDICATION WILL BE TRANSFERRED TO THE GROUND BY THE DOWNLINK WHICH DOWNLINKS ALL FLAGWORDS.
# WHEN UPLINK ACTIVITY IS LOCKED OUT ,IT CAN BE ALLOWED WHEN THE GROUND UPLINKS AND 'ERROR RESET' CODE.
# (IT IS RECOMMENDED THAT THE 'ERROR LIGHT RESET' CODE IS PRECEEDED BY 16 BITS THE FIRST OF WHICH IS 1 FOLLOWED
# BY 15 ZEROS. THIS WILL ELIMINATE EXTRANEOUS BITS FROM INLINK WHICH MAY HAVE BEEN LEFT OVER FROM THE ORIGINAL
# FAILURE).
# UPLINK ACTIVITY IS ALSO ALLOWED(UNLOCKED) DURING FRESH START WHEN FRESH START SETS BIT4 OF FLAGWRD7 = 0.

