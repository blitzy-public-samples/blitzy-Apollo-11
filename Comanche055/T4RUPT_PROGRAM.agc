# Copyright:	Public domain.
# Filename:	T4RUPT_PROGRAM.agc
# Purpose:	Part of the source code for Comanche, build 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 133-169
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	10/05/09 FB	Transcription of Batch FB-1 Assignment.
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
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: T4RUPT_PROGRAM.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Timer 4 interrupt handler executing at highest priority every 10
;        milliseconds throughout mission. Increments mission timer, samples
;        IMU CDU angles, dispatches WAITLIST tasks, manages DSKY displays,
;        and monitors critical spacecraft systems. Critical for real-time
;        operation and timing precision during Apollo 11 descent when 1202
;        alarms highlighted computational load.
;
; COMMENT-ONLY READERS: Every 10 milliseconds this routine updated the mission
;        clock, refreshed the astronauts' display, and handled time-critical
;        spacecraft measurements including the gyroscope platform orientation.
; CODE-ALONG READERS: Study highest-priority interrupt handler, 10ms timing
;        cycle, WAITLIST task dispatching, TIME register increment, IMU CDU
;        sampling, DSKY display management, and system health monitoring.
; ============================================================================

# Page 133
; ============================================================================
; TIMER 4 INTERRUPT HANDLER (T4RUPT)
;
; T4RUPT is the highest priority interrupt in the AGC system, executing
; every 10 milliseconds (100 times per second). This interrupt is the
; heartbeat of the Apollo Guidance Computer's real-time operating system.
;
; During every mission phase from launch through splashdown, T4RUPT
; maintained critical timing precision, updated navigation measurements,
; refreshed crew displays, and dispatched time-scheduled tasks. During
; Apollo 11's lunar descent, when the famous 1202 program alarms occurred,
; this interrupt continued to execute reliably despite executive scheduler
; overload, demonstrating the AGC's robust priority interrupt structure.
;
; TIMING: 10 millisecond cycle (±85 microsecond instruction time precision)
; PRIORITY: Highest (cannot be interrupted except by hardware failures)
; DURATION: Must complete within 10ms to avoid missing next interrupt tick
; ============================================================================

		BANK	12
		SETLOC	T4RUP
		BANK

		COUNT	06/T4RPT

; T4RUPT ENTRY POINT
; Every 10 milliseconds, hardware timer 4 triggers this interrupt, forcing
; the AGC to save its current work and execute this time-critical routine.
; Context (A register, Q return address, bank register) is saved to allow
; return to interrupted program after T4RUPT completion.

T4RUPT		TS	BANKRUPT	; Save A register and current bank
		EXTEND			; Enable extended instruction
		QXCH	QRUPT		; Save return address (Q register)

; DISPLAY UPDATE CONTROL
; DSRUPTSW counter cycles 7-6-5-4-3-2-1-0-7-6... controlling display update
; frequency. Every 8th T4RUPT (every 80 milliseconds) triggers quick display
; update (QUIKDSP) for DSKY eleven-segment displays. Other T4RUPTs execute
; normal housekeeping (NORMT4).

		CCS	DSRUPTSW	# GOES 7(-1)0 AROUND AND AROUND
		TCF	NORMT4 +1	; DSRUPTSW was positive, decrement and continue
		TCF	NORMT4		; DSRUPTSW was +0, continue normal path

		TCF	QUIKDSP		; DSRUPTSW was negative, do quick display update

; NORMAL T4RUPT PROCESSING PATH
; Seven out of eight T4RUPTs follow this path, resetting DSRUPTSW counter
; and preparing for full housekeeping sequence.

NORMT4		CAF	SEVEN		; Load constant 7
		TS	RUPTREG1	; Store in RUPTREG1 (used by downstream routines)
		TS	DSRUPTSW	; Reset DSRUPTSW to 7, restarting 8-cycle count

		COUNT	02/T4RPT

74K		=	HIGH4

; ============================================================================
; RELAY TABLE (RELTAB)
;
; Packed table controlling DSKY indicator lamp and relay operations.
; Upper 4 bits encode relay word selection, lower 5 bits encode relay code.
; This table maps display channel data to physical hardware outputs.
; ============================================================================

# RELTAB IS A PACKED TABLE. RELAYWORD CODE IN UPPER 4 BITS, RELAY CODE
# IN LOWER 5 BITS.

		BLOCK	02
		SETLOC	FFTAG12
		BANK

RELTAB		OCT	04025
		OCT	10003
		OCT	14031
		OCT	20033
		OCT	24017
		OCT	30036
		OCT	34034
		OCT	40023
		OCT	44035
		OCT	50037
		OCT	54000
RELTAB11	OCT	60000

# Page 134
# SWITCHED-BANK PORTION

		BANK	12
		SETLOC	T4RUP
		BANK

		COUNT	06/T4RPT

; ============================================================================
; DSKY DISPLAY DRIVER (CDRVE)
;
; Controls DSKY eleven-segment numeric displays and indicator lamps visible
; to the crew. During mission operations, astronauts relied on this display
; system to monitor navigation state, program status, and vehicle systems.
; Armstrong and Aldrin watched altitude, velocity, and fuel quantities on
; these displays during the lunar descent.
;
; DSPTAB +11D contains packed relay control data. This routine extracts
; relay codes and sends commands to output channel OUT0, controlling
; physical relay hardware that drives the DSKY display segments.
; ============================================================================

CDRVE		CCS	DSPTAB +11D	; Check display table entry 11 (decimal)
		TC	DSPOUT		; Positive: call display output routine
		TC	DSPOUT		; +0: call display output routine

		XCH	DSPTAB +11D	; Exchange A with DSPTAB +11D
		MASK	LOW11		; Mask to retain lower 11 bits
		TS	DSPTAB +11D	; Store masked value back
		AD	RELTAB11	; Add relay table entry 11
		EXTEND			; Enable extended instruction
		WRITE	OUT0		; Write relay command to output channel 0
		TC	HANG20		; Call 20-millisecond hang routine

# Page 135
# DSPOUT PROGRAM, PUTS OUT DISPLAYS

; ============================================================================
; DISPLAY OUTPUT PROGRAM (DSPOUT/DSPOUTSB)
;
; Scans DSPTAB (display table) to find pending display requests and outputs
; them to the DSKY hardware via channel OUT0. During mission operations, this
; routine updated the numeric displays showing altitude, velocity, time, and
; other critical flight data that the crew monitored continuously.
;
; DSPTAB contains 11 entries (decimal 10, octal 12). Each entry has display
; data in lower 11 bits and relay control codes in upper bits. The routine
; performs two passes through the table to ensure all displays are serviced.
;
; DISPLAY UPDATE FREQUENCY: Every 80 milliseconds (8 T4RUPT cycles)
; TABLE SCAN TIME: Approximately 1-2 milliseconds for complete scan
; ============================================================================

DSPOUTSB	TS	NOUT		; Save NOUT (display request count)
		CS	ZERO		; Load -0
		TS	DSRUPTEM	# SET TO -0 FOR 1ST PASS THRU DSPTAB
		XCH	DSPCNT		; Exchange A with DSPCNT (display counter)
		AD	NEG0		# TO PREVENT +0
		TS	DSPCNT		; Initialize display table index

; DISPLAY TABLE SCAN LOOP
; Scans DSPTAB backward from entry 11 to 0, looking for negative entries
; (indicating pending display requests). Positive entries are skipped.

DSPSCAN		INDEX	DSPCNT		; Use DSPCNT as index
		CCS	DSPTAB		; Check DSPTAB entry
		CCS	DSPCNT		# IF DSPTAB ENTRY +, SKIP
		TCF	DSPSCAN	-2	# IF DSPCNT +, AGAIN
		TCF	DSPLAY		# IF DSPTAB ENTRY -, DISPLAY
TABLNTH		OCT	12		# DEC 10 LENGTH OF DSPTAB
		CCS	DSRUPTEM	# IF DSRUPTEM=+0, 2ND PASS THRU DSPTAB
120MRUPT	DEC	16372		# (DSPCNT = 0).  +0 INTO NOUT.
		TS	NOUT		; Store in NOUT
		TC	Q		; Return to caller
		TS	DSRUPTEM	# IF DSRUPTEM=-0, 1ST PASS THRU DSPTAB
		CAF	TABLNTH		# (DSPCNT=0).+0 INTO DSRUPTEM. PASS AGAIN
		TCF	DSPSCAN -1	; Continue scanning

; DISPLAY OUTPUT HANDLER
; Found negative DSPTAB entry indicating display request. Extract relay codes
; from RELTAB, combine with display data, and write to output channel.

DSPLAY		AD	ONE		; Increment to make positive
		INDEX	DSPCNT		; Use DSPCNT as index
		TS	DSPTAB		# REPLACE POSITIVELY
		MASK	LOW11		; REMOVE BITS 12 TO 15
		TS	DSRUPTEM	; Save display data
		CAF	HI5		; Load constant for upper 5 bits
		INDEX	DSPCNT		; Use DSPCNT as index
		MASK	RELTAB		# PICK UP BITS 12 TO 15 OF RELTAB ENTRY
		AD	DSRUPTEM	; Combine relay code with display data
		EXTEND			; Enable extended instruction
		WRITE	OUT0		# WRITE CHANNEL 10
		TCF	Q+1		# *** NORMAL RETURN SKIPS ONE

; DISPLAY OUTPUT DISPATCHER
; Checks if DSKY display is enabled (FLAGWRD5) and if display requests exist
; (NOUT). Only outputs displays when DSKY is powered and requests are pending.

DSPOUT		CCS	FLAGWRD5	# DON'T DISPLAY UNLESS DSKY FLAG ON
		CAF	ZERO		; DSKY enabled
		TCF	NODSPOUT	; DSKY disabled, skip display
		CCS	NOUT		; Check display request count
		TC	DSPOUTSB	; Requests pending, call display output
		TCF	NODSPOUT	# NO DISPLAY REQUESTS

; 20-MILLISECOND HANG ROUTINE
; Adjusts DSRUPTSW counter and sets TIME4 for next interrupt. This timing
; control ensures precise 10ms T4RUPT intervals throughout mission operations.

HANG20		CS	11,14,9		; Load complement of octal 22400
		ADS	DSRUPTSW	; Add to DSRUPTSW (display interrupt switch)

		CAF	20MRUPT		; Load 20-millisecond interrupt constant

SETTIME4	TS	TIME4		; Set timer 4 for next interrupt

# Page 136
# THE STATUS OF THE PROCEED PUSHBUTTON IS MONITORED EVERY 120 MILLISECONDS VIA THE CHANNEL 32 BIT 14 INBIT.
# THE STATE OF THIS INBIT IS COMPARED WITH ITS STATE DURING THE PREVIOUS T4RUPT AND IS PROCESSED AS FOLLOWS.
#	IF PREV ON AND NOW ON 	-- BYPASS
#	IF PREV ON AND NOW OFF	-- UPDATE IMODES33
#	IF PREV OFF AND NOW ON	-- UPDATE IMODES33 AND PROCESS VIA PINBALL
#	IF PREV OFF AND NOW OFF	-- BYPASS
# THE LOGIC EMPLOYED REQUIRES ONLY 9 MCT (APPROX. 108 MICROSECONDS) OF COMPUTER TIME WHEN NO CHANGES OCCUR.

; ============================================================================
; PROCEED BUTTON MONITOR (PROCEEDE)
;
; Checks the PROCEED pushbutton on the DSKY every 120 milliseconds (12 T4RUPT
; cycles). During mission operations, the crew pressed PROCEED to acknowledge
; program requests or advance through mission sequences. For example, during
; lunar descent, the crew pressed PROCEED to confirm major program transitions.
;
; The routine detects button state changes by comparing the current state
; (CHAN32 bit 14) with the previous state (IMODES33 bit 14). When the button
; transitions from OFF to ON (button press), it schedules the PROCKEY task
; via NOVAC to process the input through the PINBALL display interface system.
;
; MONITORING FREQUENCY: Every 120 milliseconds (12 × 10ms)
; PROCESSING TIME: 9 MCT (~108 microseconds) when no change occurs
; ============================================================================

PROCEEDE	CA	IMODES33	# MONITOR FOR PROCEED BUTTON
		EXTEND			; Enable extended instruction
		RXOR	CHAN32		# CHECK IF BIT 14 DIFFERENT
		MASK	BIT14		; Isolate PROCEED button bit (bit 14)
		EXTEND			; Enable extended instruction
		BZF	T4JUMP		# NO CHANGE -- branch to T4JUMP

		LXCH	IMODES33	; Load previous state into L register
		EXTEND			; Enable extended instruction
		RXOR	LCHAN		; XOR with current channel state
		TS	IMODES33	# UPDATE IMODES33 with new state
		MASK	BIT14		; Isolate PROCEED button bit
		CCS	A		; Check sign and magnitude
		TCF	T4JUMP		# WAS ON -- NOW OFF (button released)

		CAF	CHRPRIO		# WAS OFF -- NOW ON (button pressed)
		TC	NOVAC		; Schedule PROCKEY task
		EBANK=	DSPCOUNT	; Set extended bank
		2CADR	PROCKEY		; Address of PROCEED key handler

# Page 137
# JUMP TO APPROPRIATE ONCE-PER SECOND (0.96 SEC ACTUALLY) ACTIVITY

; ============================================================================
; PERIODIC TASK DISPATCHER (T4JUMP)
;
; Dispatches to periodic monitoring routines approximately once per second
; (actually every 0.96 seconds = 96 T4RUPT cycles). The RUPTREG1 register
; contains an index (0-7) that cycles through values, routing execution to:
;
;   OPTTEST  - Optics drive test
;   OPTMON   - Optics monitor
;   IMUMON   - IMU (Inertial Measurement Unit) monitor
;   RESUME   - Return to interrupted program
;
; This time-sharing approach distributes system health monitoring tasks across
; multiple T4RUPT cycles, preventing any single interrupt from consuming too
; much processing time. During Apollo 11's descent, these routines monitored
; the IMU platform that provided spacecraft orientation data critical for
; guidance and navigation.
;
; DISPATCH FREQUENCY: Every ~960 milliseconds (96 × 10ms)
; ============================================================================

T4JUMP		INDEX	RUPTREG1	; Use RUPTREG1 as index (0-7)
		TCF	+1		; Jump to indexed entry in table

		TCF	OPTTEST		; Index 0: Test optics drive
		TCF	OPTMON		; Index 1: Monitor optics
		TCF	IMUMON		; Index 2: Monitor IMU status
		TCF	RESUME		; Index 3: Resume interrupted program
		TCF	OPTTEST		; Index 4: Test optics drive
		TCF	OPTMON		; Index 5: Monitor optics
		TCF	IMUMON		; Index 6: Monitor IMU status
		TCF	RESUME		; Index 7: Resume interrupted program

OPTTEST		TC	IBNKCALL	; Inter-bank call
		CADR	OPTDRIVE	; Address of optics drive routine

20MRUPT		=	OCT37776	# (DEC 16382)

; ============================================================================
; NO DISPLAY OUTPUT PATH (NODSPOUT)
;
; Executed when no display output is needed. Turns off display relays by
; writing zeros to OUT0 channel, then sets up the next CDRVE (channel drive)
; timer event 120 milliseconds later. This conserves power and prevents
; unnecessary relay activation when the DSKY display is not changing.
;
; The relay system controls the DSKY's electroluminescent displays and
; indicator lights that showed the crew critical information during all
; mission phases.
; ============================================================================

NODSPOUT	EXTEND			# TURN OFF RELAYS (extended instruction)
		WRITE	OUT0		; Write zeros to output channel 0

		CAF	120MRUPT	# SET FOR NEXT CDRVE (120ms interval)
		TCF	SETTIME4	; Transfer control to timer setup

; ============================================================================
; QUICK DISPLAY SERVICE (QUIKDSP)
;
; Fast display service routine called when rapid DSKY updates are needed.
; Checks the display switch state (DSRUPTSW bit 14) to determine whether
; to output display data or turn off relays. This routine alternates between
; writing display data and turning off relays to implement the multiplexed
; display system.
;
; The DSKY uses time-multiplexed relay-driven displays. Display segments are
; activated in sequence, with relays turning on briefly then off to prevent
; overheating while maintaining apparent continuous display to the crew.
;
; TIMING: 20ms intervals for display updates
; ============================================================================

QUIKDSP		CAF	BIT14		; Load bit 14 constant
		MASK	DSRUPTSW	; Check display switch state
		EXTEND			; Enable extended instruction
		BZF	QUIKOFF		; WROTE LAST TIME, NOW TURN OFF RELAYS

		CCS	NOUT		; Check output counter
		TC	DSPOUTSB	; Transfer to display output subroutine
		TCF	NODSPY		; NOUT=0 OR BAD RETURN FROM DSPOUTSB
		CS	BIT14		; GOOD RETURN (WE DISPLAYED SOMETHING)
QUIKRUPT	ADS	DSRUPTSW	; Add to display switch (toggle bit 14)

		CAF	20MRUPT		; Load 20ms interrupt constant
		TS	TIME4		; Set TIME4 for next interrupt

		CAF	BIT9		; Load bit 9 constant
		ADS	DSRUPTSW	; Add to display switch register

		TC	RESUME		; Return to interrupted program

; ============================================================================
; NO DISPLAY SPY (NODSPY)
;
; Handles the case when no display data needs to be written. Turns off the
; display relays by writing zeros to OUT0, then synchronizes the T4 timer
; for the next display service cycle.
; ============================================================================

NODSPY		EXTEND			; Enable extended instruction
		WRITE	OUT0		; Turn off display relays

; ============================================================================
; SYNCHRONIZE T4 TIMER (SYNCT4)
;
; Synchronizes the TIME4 register for display timing. Adds 20ms to TIME4 to
; schedule the next quick display interrupt. Manages the DSRUPTSW display
; switch state to coordinate display multiplexing timing.
;
; The display system requires precise 20ms timing to maintain stable, flicker-
; free display output while preventing relay overheating.
; ============================================================================

SYNCT4		CAF	20MRUPT		; Load 20ms interrupt constant
		ADS	TIME4		; Add to TIME4 (schedule next interrupt)

		CAF	BIT9		; Load bit 9 constant
# Page 138
		ADS	DSRUPTSW	; Add to display switch register
		CCS	DSRUPTSW	; Check display switch sign/magnitude
		TC	RESUME		; Positive: return to interrupted program
OCT37737	OCT	37737		; Constant OCT 37737
		TC	SYNCT4		; Zero or negative: resynchronize
		TC	RESUME		; Return to interrupted program

; ============================================================================
; QUICK RELAY OFF (QUIKOFF)
;
; Turns off display relays after a display write cycle. Resets DSRUPTSW bit 14
; to configure for display output on the next pass, implementing the write-then-
; off cycle that prevents relay overheating while maintaining display visibility.
;
; This alternating pattern (display ON briefly, then OFF) occurs every 20ms,
; fast enough that the crew perceives continuous display illumination.
; ============================================================================

QUIKOFF		EXTEND			; Enable extended instruction
		WRITE	OUT0		; Turn off display relays
		CAF	BIT14		# RESET DSRUPTSW TO SEND DISPLAY NEXT PASS
		TCF	QUIKRUPT	; Transfer to QUIKRUPT to set timer

11,14,9		OCT	22400

# Page 139
; ============================================================================
; SECTION: Inertial Measuring Unit Monitoring (IMUMON)
;
; The IMU (Inertial Measurement Unit) is the spacecraft's primary navigation
; sensor, containing three gyroscopes and three accelerometers (PIPAs) that
; measure attitude and acceleration. This monitoring section executes every
; 480 milliseconds to check six critical IMU status bits:
;
;   - Temperature within limits (prevents gyro drift from overheating)
;   - Turn-on request (initiates 90-second warmup and calibration)
;   - IMU failure indication (major system fault)
;   - CDU failure (gimbal angle readout fault)
;   - Cage mode (platform held fixed for alignment)
;   - Operate mode (normal navigation operation)
;
; During Apollo 11's mission, the IMU operated continuously from before
; launch through splashdown, providing the attitude reference and velocity
; data that guided the spacecraft across 240,000 miles to the Moon and back.
; Gimbal lock warnings (when middle gimbal approached 90 degrees) alerted
; the crew to avoid losing the attitude reference frame.
;
; COMMENT-ONLY READERS: This section monitors the navigation sensor health
;        every half-second, ensuring the gyroscopes and accelerometers that
;        told the spacecraft where it was and how it was oriented remained
;        functional throughout the mission.
;
; CODE-ALONG READERS: Study the bit-change detection algorithm that compares
;        CHAN30 hardware inputs against the previous state (IMODES30), then
;        dispatches to specialized handlers via indexed jump table (IFAILJMP).
; ============================================================================
# PROGRAM NAME:  IMUMON
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM IS ENTERED EVERY 480 MS.  IT DETECTS CHANGES OF THE IMU STATUS BITS IN
# CHANNEL 30 AND CALLS THE APPROPRIATE SUBROUTINES.  THE BITS PROCESSED AND THEIR RELEVANT SUBROUTINES ARE:
#
#	FUNCTION		BIT	SUBROUTINE CALLED
#	--------		---	-----------------
#	TEMP IN LIMITS		 15	TLIM
#	ISS TURN-ON REQUEST	 14	ITURNON
#	IMU FAIL		 13	IMUFAIL (SETISSW)
#	IMU CDU FAIL		 12	ICDUFAIL (SETISSW)
#	IMU CAGE		 11	IMUCAGE
#	IMU OPERATE		  9	IMUOP
#
# THE LAST SAMPLED STATE OF THESE BITS IS LEFT IN IMODES30.  ALSO, EACH SUBROUTINE CALLED FINDS THE NEW
# VALUE OF THE BIT IN A, WITH Q SET TO THE PROPER RETURN LOCATION NXTIFAIL.
#
# CALLING SEQUENCE:  T4RUPT EVERY 480 MILLISECONDS.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  TLIM, ITURNON, SETISSW, IMUCAGE, IMUOP.
#
# ERASABLE INITIALIZATION:
#	FRESH START OR RESTART WITH NO GROUPS ACTIVE:  C(IMODES30) = OCT 37411.
#	RESTART WITH ACTIVE GROUPS:	C(IMODES30) = (B(IMODES30)AND(OCT 00035)) PLUS OCT 37400.
#					THIS LEAVES IMU FAIL BITS INTACT.
#
# ALARMS:  NONE.
#
# EXIT:  TNONTEST.
#
# OUTPUT:  UPDATED IMODES30 WITH CHANGES PROCESSED BY APPROPRIATE SUBROUTINE.

; IMU status monitoring main routine - executes every 480 milliseconds
; Detects bit transitions in CHANNEL 30 hardware inputs (IMU status register)
; by comparing current state against previously sampled state (IMODES30).
; Any detected changes trigger specialized handler routines via indexed jump.

IMUMON		CA	IMODES30	# SEE IF THERE HAS BEEN A CHANGE IN THE
		EXTEND			# RELEVANT BITS OF CHAN 30.
		RXOR	CHAN30		# CHECK IF BITS 9,11-15 CHANGED
		MASK	30RDMSK		# Isolate bits 9, 11-15 (IMU status bits)
		EXTEND
		BZF	TNONTEST	# NO CHANGE IN STATUS
					# No bit transitions detected - proceed to
					# turn-on processing test without status updates

; Bit change detected - update IMODES30 with new hardware state
; RUPTREG1 holds bitmask showing which bits changed (XOR result)

		TS	RUPTREG1	# SAVE BITS WHICH HAVE CHANGED.
		LXCH	IMODES30	# UPDATE IMODES30.
		EXTEND			# Load previous state into L register
		RXOR	LCHAN		# XOR with current channel state
		TS	IMODES30	# Store updated IMU mode state for next cycle
					# IMODES30 now reflects current hardware state

; Begin bit scan algorithm to process each changed bit individually
; Uses successive doubling to identify bit positions that changed

		CS	ONE		# Initialize bit position counter to -1
		XCH	RUPTREG1	# Retrieve change mask into A register
		EXTEND
# Page 140
		BZMF	TLIM		# CHANGE IN IMU TEMP.
					# Bit 15 (sign bit) set means temperature
					# status changed - handle immediately
		TCF	NXTIFBIT	# BEGIN BIT SCAN.
					# Start scanning from bit 14 downward

; Bit scan loop - identifies which specific bit(s) changed by successive doubling
; Algorithm: Increment bit position counter, double the change mask, and when
; overflow occurs we've found a changed bit. Dispatches to handler via jump table.

 	-1	AD	ONE		# (RE-ENTERS HERE FROM NXTIFAIL.)
					# Re-entry point: restore change mask and continue
NXTIFBIT	INCR	RUPTREG1	# ADVANCE BIT POSITION NUMBER.
					# RUPTREG1 = bit position (14, 13, 12, 11, 9)
 	+1	DOUBLE			# Shift change mask left one bit
 		TS	A		# SKIP IF OVERFLOW.
					# When bit shifts into sign position (bit 15),
					# overflow occurs and skip is NOT taken
		TCF	NXTIFBIT	# LOOK FOR BIT.
					# No overflow yet - keep scanning for changed bit

; Changed bit found - bit position is in RUPTREG1 (14, 13, 12, 11, or 9)
; Dispatch to appropriate handler routine via indexed jump table IFAILJMP

		XCH	RUPTREG2	# SAVE OVERFLOW-CORRECTED DATA.
					# Save remaining change mask for next iteration
		INDEX	RUPTREG1	# SELECT NEW VALUE OF THIS BIT.
		CAF	BIT14		# Load bit pattern corresponding to position
					# Indexed CAF retrieves BIT14, BIT13, etc.
		MASK	IMODES30	# Extract current state of this bit from IMODES30
					# Result: 0 if bit is now clear, non-zero if set
		INDEX	RUPTREG1	# Index into handler jump table
		TC	IFAILJMP	# Dispatch to: ITURNON, IMUFAIL, ICDUFAIL,
					# IMUCAGE, or IMUOP based on bit position
					# Handler routine finds new bit value in A reg

; Re-entry point after handler completes - check for additional bit changes

NXTIFAIL	CCS	RUPTREG2	# PROCESS ANY ADDITIONAL CHANGES.
					# Check if more bits changed (saved mask > 0)
		TCF	NXTIFBIT -1	# More bits to process - restore mask and loop

# Page 141
; ============================================================================
; TRANSITION: From IMU Status Monitoring to Turn-On Request Processing
;
; With individual IMU status bit changes processed, attention now shifts to
; detecting and honoring IMU initialization requests. The ISS (Inertial
; Subsystem) turn-on sequence required careful timing: both TURN-ON (bit 14)
; and OPERATE (bit 9) requests must arrive, then the platform undergoes a
; 90-second caging period for gyro warmup before the ICDU (gimbal angle
; counters) are zeroed for proper gimbal lock monitoring.
;
; During Apollo 11 pre-launch, this initialization sequence executed during
; countdown when the IMU was powered on approximately 2.5 hours before liftoff,
; establishing the stable platform reference that would guide the spacecraft
; to the Moon. The two-sample delay (480ms between checks) ensured all hardware
; signals stabilized before initialization processing began.
; ============================================================================
# PROGRAM NAME:  TNONTEST.
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM HONORS REQUESTS FOR ISS INITIALIZATION.  ISS TURN-ON (CHANNEL 30 BIT 14)
# AND ISS OPERATE (CHANNEL 30 BIT 9) REQUESTS ARE TREATED AS A PAIR AND PROCESSING TAKES PLACE .480 SECONDS
# AFTER EITHER ONE APPEARS.  THIS INITIALIZATION TAKES ON ONE OF THE FOLLOWING THREE FORMS:
#
#	1) ISS TURN-ON:  IN THIS SITUATION THE COMPUTER IS OPERATING WHEN THE ISS IS TURNED ON.  NOMINALLY,
#	BOTH ISS TURN-ON AND ISS OPERATE APPEAR.  THE PLATFORM IS CAGED FOR 90 SECONDS AND THE ICDU'S ZEROED
#	SO THAT AT THE END OF THE PROCESS THE GIMBAL LOCK MONITOR WILL FUNCTION PROPERLY.
#
#	2) ICDU INITIALIZATION:  IN THIS CASE THE COMPUTER WAS PROBABLY TURNED ON WITH THE ISS IN OPERATE OR
#	A FRESH START WAS DONE WIT THE ISS IN OPERATE.  IN THIS CASE ONLY ISS OPERATE IS ON.  THE ICDU'S ARE
#	ZEROED SO THE GIMBAL LOCK MONITOR WILL FUNCTION.  AN EXCEPTION IS IF THE ISS IS IN GIMBAL LOCK AFTER
#	A RESTART, THE ICDU'S WILL NOT BE ZEROED.
#
#	3) RESTART WITH RESTARTABLE PROGRAM USING THE IMU:  IN THIS CASE, NO INITIALIZATION TAKES PLACE SINCE
#	IT IS ASSUMED THAT THE USING PROGRAM DID THE INITIALIZATION AND THEREFORE T4RUPT SHOULD NOT INTERFERE.
#
# IMODES30 BIT 7 IS SET = 1 BY THE FIRST BIT (CHANNEL 30 BIT 14 OR 9) WHICH ARRIVES.  FOLLOWING THIS, TNONTEST IS
# ENTERED, FINDS BIT 7 = 1 BUT BIT 8 = 0, SO IT SETS BIT 8 = 1 AND EXITS.  THE NEXT TIME IT FINDS BIT 8 = 1 AND
# PROCEEDS, SETTING BITS 8 AND 7 = 0.  AT PROCTNON, IF ISS TURN-ON REQUEST IS PRESENT, THE ISS IS CAGED (ZERO +
# COARSE).  IF ISS OPERATE IS NOT PRESENT PROGRAM ALARM 00213 IS ISSUED.  AT THE END OF A 90 SECOND CAGE, BIT 2
# OF IMODES30 IS TESTED.  IF IT IS = 1, ISS TURN-ON WAS NOT PRESENT FOR THE ENTIRE 90 SECONDS.  IN THAT CASE, IF
# THE ISS TURN-ON REQUEST IS PRESENT THE 90 SECOND WAIT IS REPEATED,  OTHERWISE NO ACTION OCCURS UNLESS A PROGRAM
# WAS WAITING FOR THE INITIALIZATION IN WHICH CASE THE PROGRAM IS GIVEN AN IMUSTALL ERROR RETURN.  IF THE DELAY
# WENT PROPERLY, THE ISS DELAY OUTBIT IS SENT AND THE ICDU'S ZEROED.  A TASK IS INITIATED TO REMOVE THE PIPA FAIL
# INHIBIT BIT IN 10.24 SECONDS.  IF A MISSION PROGRAM WAS WAITING IT IS INFORMED VIA ENDIMU.
#
# AT PROCTNON, IF ONLY ISS OPERATE IS PRESENT (OPONLY), THE CDU'S ARE ZEROED UNLESS THE PLATFORM IS IN COARSE
# ALIGN (= GIMBAL LOCK HERE) OR A MISSION PROGRAM IS USING THE IMU (INUSEFLG = 1).
#
# CALLING SEQUENCE:  T4RUPT EVERY 480 MILLISECONDS AFTER IMUMON.
#
# JOBS OR TASKS INITIATED:  1) ENDTNON, 90 SECONDS AFTER CAGING STARTED.  2) ISSUP, 4 SECONDS AFTER CAGING DONE.
#	3) PFAILOK, 10.24 SECONDS AFTER INITIALIZATION COMPLETED.  4) UNZ2, 320 MILLISECONDS AFTER ZEROING
#	STARTED.
#
# SUBROUTINES CALLED: CAGESUB, CAGESUB2, ZEROICDU, ENDIMU, IMUBAD, NOATTOFF, SETISSW, VARDELAY.
#
# ERASABLE INITIALIZATION:  SEE IMUMON.
#
# ALARMS:  PROGRAM ALARM 00213 IF ISS TURN-ON REQUESTED WITHOUT ISS OPERATE.
#
# EXIT:  ENDTNON EXITS TO C33TEST.  TASKS HAVING TO DO WITH INITIALIZATION EXIT AS FOLLOWS:  MISSION PROGRAM
# WAITING AND INITIALIZATION COMPLETE, EXIT TO ENDIMU, MISSION PROGRAM WAITING AND INITIALIZATION FAILED, EXIT TO
# IMUBAD, IMU NOT IN USE, EXIT TO TASKOVER.
#
# OUTPUT:  ISS INITIALIZED.

; Turn-on testing implements two-sample delay for signal stabilization.
; IMODES30 bit 7 = turn-on sequence initiated (first sample detected)
; IMODES30 bit 8 = second sample taken (ready to process)
; This 480-millisecond delay ensures both TURN-ON and OPERATE signals present.

TNONTEST		CS	IMODES30	# AFTER PROCESSING ALL CHANGES, SEE IF IT
# Page 142
			MASK	BIT7		# IS TIME TO ACT ON A TURN-ON SEQUENCE.
			CCS	A		# Check if bit 7 = 0 (no turn-on pending)
			TCF	C33TEST		# NO -- EXAMINE CHANNEL 33.
					# No turn-on sequence active, skip to C33 test

; Turn-on sequence initiated - check if this is first or second sample

			CAF	BIT8		# SEE IF FIRST SAMPLE OR SECOND.
			MASK	IMODES30	# Extract bit 8 (second sample flag)
			CCS	A		# Check if bit 8 = 1
			TCF	PROCTNON	# REACT AFTER A SECOND SAMPLE.
					# Second sample complete - process turn-on

; First sample detected - set bit 8 and wait for next T4RUPT cycle (480ms)

			CAF	BIT8		# IF FIRST SAMPLE, SET BIT TO REACT NEXT
			ADS	IMODES30	# TIME.
					# Mark that first sample taken, will process
					# on next cycle if signals still present
			TCF	C33TEST		# Exit to channel 33 testing

; ============================================================================
; Process IMU Turn-On Requests After Two-Sample Delay
;
; Two initialization scenarios handled:
; 1) ISS TURN-ON + OPERATE: Full initialization with 90-second cage period
; 2) OPERATE only: ICDU zeroing without caging (computer fresh start case)
; ============================================================================
# PROCESS IMU TURN-ON REQUESTS AFTER WAITING 1 SAMPLE FOR ALL SIGNALS TO ARRIVE.

PROCTNON		CS	BITS7&8		# Clear turn-on processing flags
			MASK	IMODES30	# Remove bits 7 and 8 from IMODES30
			TS	IMODES30	# Two-sample sequence complete
			MASK	BIT14		# SEE IF TURN-ON REQUEST.
					# Check if TURN-ON signal (bit 14) present
			CCS	A
			TCF	OPONLY		# OPERATE ON ONLY.
					# Only OPERATE present - skip caging

; Full turn-on sequence: both TURN-ON and OPERATE should be present
; Verify OPERATE signal present (bit 9), otherwise issue alarm 00213

			CS	IMODES30	# IF TURN-ON REQUEST, WE SHOULD HAVE IMU
			MASK	BIT9		# OPERATE.
					# Extract OPERATE signal status
			CCS	A		# Check if OPERATE = 0 (not present)
			TCF	+3		# OPERATE present - proceed with caging

; TURN-ON requested but OPERATE missing - hardware configuration error

			TC	ALARM		# ALARM IF NOT
			OCT	213		# Alarm 00213: ISS turn-on without operate
					# Crew must verify IMU power configuration

; Begin 90-second caging period for gyro stabilization
; Platform held in ZERO+COARSE mode while gyroscopes warm up

 	+3		TC	CAGESUB		# Initiate platform caging sequence
					# Sends ZERO and COARSE commands to IMU

; Schedule ENDTNON task 90 seconds in future to complete initialization
; During Apollo 11 pre-launch, this 90-second period allowed gyros to
; reach thermal equilibrium before navigation operations began

 			CAF	90SECS		# Load 90-second delay constant (9000 centiseconds)
			TC	WAITLIST	# Schedule delayed task on waitlist
			EBANK=	CDUIND		# Set erasable bank for task execution
			2CADR	ENDTNON		# Address of completion handler
					# ENDTNON will zero ICDUs and enable platform

			TCF	C33TEST		# Continue with channel 33 testing

; Re-enter 90-second delay if turn-on request still present
; Used when multiple turn-on attempts needed due to hardware issues

RETNON			CAF	90SECS		# Restart 90-second delay period
			TC	VARDELAY	# Schedule delay via variable waitlist entry

; ============================================================================
; ENDTNON - Complete IMU Turn-On Sequence After 90-Second Cage Period
;
; This task executes after the 90-second gyro warmup completes. It checks
; whether the turn-on was successful by examining the turn-on request fail
; bit (IMODES30 bit 2). Three possible outcomes:
;
; 1) Success: Send ISS DELAY COMPLETE, zero ICDUs, enable platform
; 2) Still pending: Re-enter 90-second delay (TURN-ON signal still present)
; 3) Failure: Jump to IMUBAD handler if program was waiting for IMU
;
; During Apollo 11, this sequence completed successfully during pre-launch
; countdown, enabling the IMU for navigation from liftoff through splashdown.
; ============================================================================

ENDTNON			CS	BIT2		# RESET TURN-ON REQUEST FAIL BIT.
			MASK	IMODES30	# Extract all bits except bit 2
			XCH	IMODES30	# Store cleared version, retrieve old
			MASK	BIT2		# IF IT WAS OFF, SEND ISS DELAY COMPLETE.
					# Check previous state of fail bit
			EXTEND
			BZF	ENDTNON2	# Bit was OFF - successful turn-on
# Page 143
; Turn-on request fail bit was ON - check if TURN-ON signal still present
; If yes: Hardware still trying to initialize, restart 90-second delay
; If no:  Turn-on attempt failed, alert any waiting programs

			CAF	BIT14		# IF IT WAS ON AND TURN-ON REQUEST NOW.
			MASK	IMODES30	# PRESENT, RE-ENTER 90 SEC DELAY IN WL.
					# Check if TURN-ON signal (bit 14) still active
			EXTEND
			BZF	RETNON		# Still present - retry 90-second delay

; TURN-ON signal no longer present - turn-on attempt has failed
; Check if any program was waiting for IMU availability (STATE bit IMUSEFLG)

			CS	STATE		# IF IT IS NOT ON NOW, SEE IF A PROG WAS
			MASK	IMUSEFLG	# WAITING.
					# Check if program blocked on IMU
			CCS	A		# A = 0 if program waiting
			TCF	TASKOVER	# No program waiting - just exit
			TC	POSTJUMP	# Program waiting - notify of failure
			CADR	IMUBAD		# UNSUCCESSFUL TURN-ON.
					# Abort waiting program with IMU bad status

; ============================================================================
; ENDTNON2 - Successful Turn-On Completion Path
; 
; The 90-second gyro warmup completed successfully. Now send ISS DELAY COMPLETE,
; turn off the NO ATT (No Attitude) lamp on the DSKY, zero the ICDU gimbal
; angle counters, and enable the Digital Autopilot (DAP).
;
; During Apollo 11 pre-launch, this sequence enabled the navigation platform
; that would guide the spacecraft across 240,000 miles to the Moon and back.
; ============================================================================

ENDTNON2		CAF	BIT15		# SEND ISS DELAY COMPLETE.
			EXTEND			# Write to output channel
			WOR	CHAN12		# TURN OFF ISS DELAY COUNTER
					# Set bit 15 - signals caging complete
			TC 	IBNKCALL	# TURN OFF NO ATT LAMP.
			CADR	NOATTOFF	# Clear "No Attitude" warning on DSKY
					# Crew now has valid attitude reference

; ============================================================================
; UNZ2 - Zero ICDU Gimbal Angle Counters
;
; Calls ZEROICDU to clear CDUX, CDUY, CDUZ counters. Establishes zero reference
; for gimbal angles. Middle gimbal (CDUY) starts at 0° - gimbal lock warnings
; trigger when CDUY approaches ±90° (Euler angle singularity).
; ============================================================================

UNZ2			TC	ZEROICDU	# Zero all three gimbal counters
					# Sets CDUX = CDUY = CDUZ = 0
					# Establishes attitude reference frame

; Disable ZERO and COARSE align modes in IMU (clear bits 4 & 5 on channel 12)
; Platform transitions from coarse alignment to fine alignment mode

			CS	BITS4&5		# REMOVE ZERO AND COARSE.
			EXTEND			# Write AND mask to channel
			WAND	CHAN12		# Clear bits 4 & 5 - disable caging
					# IMU now in fine align/operate mode

; Wait 10 seconds for ICDU counters to track gimbal positions
; Hardware needs time to synchronize digital counters with actual gimbal angles

			CAF	BIT11		# WAIT 10 SECS FOR CTRS TO FIND GIMBALS
			TC	VARDELAY	# 10 seconds = 1000 centiseconds = 2^10
					# Schedules task on waitlist

; ============================================================================
; ISSUP - IMU Successfully Up and Operational
;
; Clear all initialization inhibit flags, enable Digital Autopilot, and
; remove temporary discretes. IMU now fully operational for navigation.
; ============================================================================

ISSUP			CS	OCT54		# REMOVE CAGING, IMU FAIL INHIBIT, AND
			MASK	IMODES30	# ICDUFAIL INHIBIT FLAGS.
					# OCT54 = bits 2,3,4,5 cleared
			TS	IMODES30	# Platform operational, failures enabled

; Enable Digital Autopilot (DAP) by clearing bit 6 in IMODES33
; DAP can now use IMU data for attitude control and thruster firing

			CS	BIT6		# ENABLE DAP
			MASK	IMODES33	# Clear NODAP flag (bit 6)
			TS	IMODES33	# Autopilot now has attitude reference

; Set ISS warning system - may have been inhibited during turn-on
; Enables crew alerting if IMU develops problems during mission

			TC	SETISSW		# ISS WARNING MIGHT HAVE BEEN INHIBITED.
					# Re-enables IMU failure monitoring

; Remove ISS DELAY COMPLETE discrete (clear bit 15 on channel 12)
; Temporary signal no longer needed now that initialization finished

			CS	BIT15		# REMOVE IMU DELAY COMPLETE DISCRETE.
			EXTEND			# Write AND mask to channel
			WAND	CHAN12		# Clear bit 15 - clean up output state

; Wait 4 more seconds before enabling PIPA (accelerometer) failure alarms
; Accelerometers need additional stabilization time after platform enabled
; During Apollo 11 flight, false PIPA alarms during initialization would
; have caused unnecessary crew concern

			CAF	4SECS		# DONT ENABLE PROG ALARM ON PIP FAIL FOR
			TC	WAITLIST	# ANOTHER 4 SECS.
					# 4 seconds = 400 centiseconds
			EBANK=	CDUIND		# Set erasable bank for task
			2CADR	PFAILOK		# Task address: enable PIPA fail alarms
					# After 4 seconds, PIPA failures alert crew

			TCF	TASKOVER	# IMU initialization complete - exit task

; ============================================================================
; OPONLY - Handle OPERATE-Only Signal (Without Turn-On Request)
;
; This handler processes the case where OPERATE mode is commanded but the
; TURN-ON request is NOT present. This typically occurs during IMU alignment
; procedures when the platform needs to be re-zeroed without a full turn-on
; sequence. The routine must be careful not to zero the ICDUs if the IMU is
; in COARSE ALIGN mode, as the gimbals may be near gimbal lock (middle gimbal
; near ±90°). Zeroing counters during gimbal lock would lose attitude reference.
;
; COMMENT-ONLY READERS: This handles IMU re-initialization during flight without
;        going through the full 90-second warmup. Used during in-flight alignment
;        procedures to re-establish the attitude reference.
;
; CODE-ALONG READERS: Study the gimbal lock safety check (testing COARSE ALIGN
;        bit 4 on channel 12) and the program usage check (IMUSEFLG in STATE).
; ============================================================================

OPONLY			CAF	BIT4		# Check if COARSE ALIGN active
# Page 144
			EXTEND			# IF OPERATE ON ONLY AND WE ARE IN COARSE
			RAND	CHAN12		# ALIGN, DON'T ZERO THE CDUS BECAUSE WE
					# Read bit 4 (COARSE ALIGN) from channel 12
			CCS	A		# MIGHT BE IN GIMBAL LOCK. USE V41N20 TO
			TCF	C33TEST		# RECOVER.
					# If COARSE ALIGN active, skip zeroing
					# Crew must use V41N20 to recover manually

; COARSE ALIGN not active - safe to zero ICDUs
; But first check if any program is currently using the IMU

			CAF	IMUSEFLG	# OTHERWISE, ZERO THE COUNTERS
			MASK	STATE		# UNLESS SOMEONE IS USING THE IMU.
					# Check if IMUSEFLG bit set in STATE
			CCS	A		# A = 0 if no program using IMU
			TCF	C33TEST		# Program using IMU - skip zeroing
					# Don't interrupt active navigation

; Safe to zero ICDUs - no programs using IMU, not in COARSE ALIGN
; Set turn-on flags and proceed to zero sequence

			TC	CAGESUB2	# SET TURNON FLAGS.
					# Prepares flags for zeroing operation

; ============================================================================
; ISSZERO - Zero ICDUs Without Full Turn-On Sequence
;
; This entry point zeros the gimbal angle counters after verifying it's safe
; to do so. Used during in-flight IMU alignment procedures (e.g., P51/P52/P53
; fine align programs) to re-establish attitude reference without the full
; 90-second gyro warmup that TNONTEST requires.
; ============================================================================

ISSZERO			TC	IBNKCALL	# TURN OFF NO ATT LAMP.
			CADR	NOATTOFF	#     IMU CAGE OFF ENTRY.
					# Clear DSKY warning - attitude valid

; Send ISS CDU ZERO discrete to IMU (set bit 5 on channel 12)
; Commands hardware to zero the gimbal angle counters

			CAF	BIT5		# ISS CDU ZERO
			EXTEND			# Write to output channel
			WOR	CHAN12		# Set bit 5 - zero ICDU counters
					# Hardware zeros CDUX, CDUY, CDUZ

; Zero software copies of gimbal angles
; Synchronizes AGC memory with hardware ICDU counters

			TC	ZEROICDU	# Zero CDUX, CDUY, CDUZ in memory
					# Software now matches hardware

; Wait 300 milliseconds for AGS (Abort Guidance System) to receive signal
; AGS in LM must be notified of IMU zero to maintain its own attitude reference
; Wait ensures AGS has time to process the change before continuing

			CAF	BIT6		# WAIT 300 MS FOR AGS TO RECEIVE SIGNAL.
			TC	WAITLIST	# 300ms = 30 centiseconds = 2^5 + 2^4 + ...
					# Actually BIT6 = 64 centiseconds = 640ms
			EBANK=	OPTMODES	# Set erasable bank for task
			2CADR	UNZ2		# Continue to UNZ2 after delay
					# Completes zeroing sequence

			TCF	C33TEST		# Proceed to channel 33 monitoring

; ============================================================================
; TRANSITION: From IMU mode switching to system health monitoring
;
; Having completed IMU initialization and zeroing sequences, T4RUPT now
; performs continuous health monitoring of critical spacecraft telemetry
; systems. Channel 33 flip-flops detect hardware failures: PIPA failures
; (bit 13), downlink telemetry too fast (bit 12), and uplink too fast
; (bit 11). These checks run every 480 milliseconds throughout the mission.
; ============================================================================

# Page 145
# PROGRAM NAME:  C33TEST
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM MONITORS THREE FLIP-FLOP INBITS OF CHANNEL 33 AND CALLS THE APPROPRIATE
# SUBROUTINE TO PROCESS A CHANGE.  IT IS ANALOGOUS TO IMUMON, WHICH MONITORS CHANNEL 30, EXCEPT THAT IT READS
# CHANNEL 33 WITH A WAND INSTRUCTION BECAUSE A `WRITE' PULSE IS REQUIRED TO RESET THE FLIP-FLOPS.  THE BITS
# PROCESSED AND THE SUBROUTINES CALLED ARE:
#	BIT	FUNCTION		SUBROUTINE
#	---	--------		----------
#	 13	PIPA FAIL		PIPFAIL
#	 12	DOWNLINK TOO FAST	DNTMFAST
#	 11	UPLINK TOO FAST		UPTMFAST
#
# UPON ENTRY TO THE SUBROUTINE, THE NEW BIT STATE IS IN A.
#
# CALLING SEQUENCE:  EVERY 480 MILLISECONDS AFTER TNONTEST.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  PIPFAIL, DNTMFAST AND UPTMFAST ON BIT CHANGES.
#
# ERASABLE INITIALIZATION:  C(IMODES33) = OCT 16000 ON A FRESH START OR RESTART, THEREFORE, THESE ALARMS WILL
# REAPPEAR IF THE CONDITIONS PERSIST.
#
# ALARMS:  NONE.
#
# EXIT:  GLOCKMON.
#
# OUTPUT:  UPDATED BITS 13, 12, AND 11 OF IMODES33 WITH CHANGES PROCESSED.

; Channel 33 flip-flops detect critical telemetry and sensor failures.
; Every 480 milliseconds, this routine reads channel 33 with WAND instruction
; (which resets the hardware flip-flops) and compares against the previous
; state stored in IMODES33 to detect changes requiring crew notification.

C33TEST		CA	IMODES33		# SEE IF RELEVANT CHAN33 BITS HAVE
		MASK	33RDMSK			# Isolate bits 13,12,11 from stored state
		TS	L			# CHANGED.
		CAF	33RDMSK			# Mask for bits 13,12,11
		EXTEND
		WAND	CHAN33			# RESETS FLIP-FLOP INPUTS
					# Read channel 33 (hardware resets on read)
		EXTEND
		RXOR	LCHAN			# XOR with previous state to find changes
		EXTEND
		BZF	GLOCKMON		# ON NO CHANGE.

; One or more bits have changed - process each change sequentially.
; Update IMODES33 to reflect current hardware state, then scan through
; changed bits (using DOUBLE-shift method) to call appropriate handlers:
; PIPFAIL for bit 13, DNTMFAST for bit 12, UPTMFAST for bit 11.

		TS	RUPTREG1		# SAVE BITS WHICH HAVE CHANGED
		LXCH	IMODES33		# Exchange L and IMODES33
		EXTEND
		RXOR	LCHAN			# XOR to update IMODES33
		TS	IMODES33		# UPDATED IMODES33.

		CAF	ZERO			# Initialize bit position counter
		XCH	RUPTREG1		# Get changed bits into A
		DOUBLE				# Start bit scanning (shift left)
# Page 146
		TCF	NXTIBT +1		# SCAN FOR BIT CHANGES.

; Bit scanning loop: repeatedly DOUBLE (shift left) until bit falls off the
; left edge (carry), indicating which bit position changed. RUPTREG1 counts
; bit position (0,1,2 for bits 13,12,11). Same elegant scanning technique
; used in IMUMON for channel 30.

 	-1	AD	ONE			# Restore after overflow
NXTIBT		INCR	RUPTREG1		# Advance bit position counter
 	+1	DOUBLE				# Shift left - bit falls off when found
 		TS	A			# (CODING IDENTICAL TO CHAN 30).
		TCF	NXTIBT			# Loop until bit found

; Bit position identified - dispatch to appropriate handler based on which
; bit changed. Get new bit value from IMODES33 and call handler via C33JMP
; table (PIPFAIL for bit 13, DNTMFAST for 12, UPTMFAST for 11).

		XCH	RUPTREG2		# Save A for multiple-change processing
		INDEX	RUPTREG1		# GET NEW VALUE OF BIT WHICH CHANGED.
		CAF	BIT13			# Get bit 13, 12, or 11 based on index
		MASK	IMODES33		# Extract new state of changed bit
		INDEX	RUPTREG1		# Index into C33JMP dispatch table
		TC	C33JMP			# Call handler with bit state in A

NXTFL33		CCS	RUPTREG2		# PROCESS POSSIBLE ADDITIONAL CHANGES.
		TCF	NXTIBT -1		# More bits changed - process next
		# Fall through when all bit changes processed

; ============================================================================
; TRANSITION: From telemetry health monitoring to gimbal lock protection
;
; Channel 33 monitoring complete - all hardware flip-flop changes processed
; and crew alerted as needed. T4RUPT now checks the Inertial Subsystem (ISS)
; gimbal angles every 480 milliseconds to prevent catastrophic gimbal lock.
; When middle gimbal angle nears 90 degrees, the IMU loses attitude reference.
; GLOCKMON provides three-tier protection: lamp warning at 70°, forced coarse
; align at 85° to prevent runaway. Critical during Apollo 11 translunar coast
; when improper maneuvers could trigger gimbal lock emergency.
; ============================================================================

# Page 147
# PROGRAM NAME:  GLOCKMON
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM MONITORS THE CDUZ COUNTER TO DETERMINE WHETHER THE ISS IS IN GIMBAL LOCK
# AND TAKES ACTION IF IT IS.  THREE REGIONS OF MIDDLE GIMBAL ANGLE (MGA) ARE USED:
#
#	1) ABS(MGA) LESS THAN OR EQUAL TO 70 DEGREES -- NORMAL MODE.
#	2) ABS(MGA) GREATER THAN 70 DEGREES AND LESS THAN OR EQUAL TO 85 DEGREES -- GIMBAL LOCK LAMP TURNED ON.
#	3) ABS(MGA) GREATER THAN 85 DEGREES -- ISS PUT IN COARSE ALIGN AND NO ATT LAMP TURNED ON.
#
# CALLING SEQUENCE:  EVERY 480 MILLISECONDS AFTER C33TEST.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:	1) SETCOARS WHEN ABS(MGA) GREATER THEN 85 DEGREES AND ISS NOT IN COARSE ALIGN.
#			2) LAMPTEST BEFORE TURNING OFF GIMBAL LOCK LAMP.
#
# ERASABLE INITIALIZATION:
#		1) FRESH START OR RESTART WITH NO GROUPS ACTIVE:  C(CDUZ) = 0, IMODES30 BIT 6 = 0, IMODES33 BIT 1 = 0.
#		2) RESTART WITH GROUPS ACTIVE:	SAME AS FRESH START EXCEPT C(CDUZ) NOT CHANGED SO GIMBAL MONITOR
#						PROCEEDS AS BEFORE.
#
# ALARMS:	1) MGA REGION (2) CAUSES GIMBAL LOCK LAMP TO BE LIT.
#		2) MGA REGION (3) CAUSES THE ISS TO BE PUT IN COARSE ALIGN AND THE NO ATT LAMP TO BE LIT IF EITHER NOT
#		   SO ALREADY.

; Gimbal lock occurs when middle gimbal angle approaches 90 degrees, causing
; loss of one degree of freedom. CDUZ counter holds middle gimbal angle (MGA)
; in 2's complement scaled units. CCS tests sign and magnitude to classify
; into three regions: safe (<70°), warning (70-85°), critical (>85°).

GLOCKMON	CCS	CDUZ			# Test sign and magnitude of MGA
		TCF	GLOCKCHK		# SEE IF MAGNITUDE OF MGA IS GREATER THAN
		TCF	SETGLOCK		# 70 DEGREES. (positive zero - safe)
		TCF	GLOCKCHK		# (negative value)
		TCF	SETGLOCK		# (negative zero - safe)

; MGA magnitude is positive (either + or -). Subtract 70 degrees threshold.
; If result negative or zero, MGA is safe (<70°). If positive, check 85°.

GLOCKCHK	AD	-70DEGS			# Subtract 70 degree threshold
		EXTEND
		BZMF	SETGLOCK -1		# NO LOCK. (magnitude < 70 degrees)

; MGA between 70° and 85° - warning region. Subtract additional 15 degrees
; to test if MGA exceeds 85° (critical region requiring forced coarse align).

		AD	-15DEGS			# SEE IF ABS(MGA) GREATER THAN 85 DEGREES
		EXTEND				# (70+15=85 total)
		BZMF	NOGIMRUN		# 70-85° range: just light lamp

; MGA exceeds 85 degrees - CRITICAL! IMU must be forced into coarse align
; mode to prevent gimbal runaway. Check if already in coarse align (bit 4
; of IMODES30); if not, call SETCOARS to safing mode and light NO ATT lamp.

		CAF	BIT4			# IF SO, SYSTEM SHOULD BE IN COARSE ALIGN
		EXTEND				# TO PREVENT GIMBAL RUNAWAY.
		RAND	CHAN12			# Read IMU mode bit from channel 12
		CCS	A			# Test if coarse align bit set
		TCF	NOGIMRUN		# Already in coarse align - just light lamp

; System NOT in coarse align but MGA >85° - emergency! Force IMU into coarse
; align mode immediately to prevent gimbal runaway, then schedule ISS error
; counter enable after 60ms settling time. This safing action prevents loss
; of attitude reference during critical mission phases.

		TC	IBNKCALL		# GO INTO COARSE ALIGN.
		CADR	SETCOARS		# Force IMU to coarse align mode

		CAF	SIX			# ENABLE ISS ERROR COUNTERS IN 60 MS.
		TC	WAITLIST		# Schedule CA+ECE task for error recovery
# Page 148
		EBANK=	CDUIND
		2CADR	CA+ECE			# Coarse align + error counter enable

; ============================================================================
; NOGIMRUN: MGA in warning region (70-85 degrees)
; SETGLOCK: Gimbal lock lamp state management
;
; If MGA between 70-85 degrees, light gimbal lock lamp to warn crew without
; forcing coarse align. SETGLOCK performs intelligent lamp control: compares
; desired state (A register: BIT6=on, ZERO=off) with current lamp state in
; DSPTAB+11D. Only inverts lamp if state change needed. Prevents unnecessary
; relay writes and respects IMU caging operations and lamp test mode.
; ============================================================================

NOGIMRUN	CAF	BIT6			# TURN ON GIMBAL LOCK LAMP.
		TCF	SETGLOCK		# Jump to lamp control logic

; Entry point for "turn lamp OFF" - clear desired state to zero
 -1		CAF	ZERO			# Desired state: lamp off
SETGLOCK	AD	DSPTAB +11D		# SEE IF PRESENT STATE OF GIMBAL LOCK LAMP
		MASK	BIT6			# AGREES WITH DESIRED STATE BY HALF ADDING
		EXTEND				# THE TWO. (XOR logic via half-add)
		BZF	GLOCKOK			# OK AS IS. (desired = current)

; State change needed. Check if we're trying to turn lamp ON while IMU is
; being caged (coarse align bit set in IMODES30). Don't light gimbal lock
; lamp during IMU cage operation - misleading to crew.

		MASK	DSPTAB +11D		# IF OFF, DON'T TURN ON IF IMU BEING CAGED.
		CCS	A			# Is lamp currently OFF?
		TCF	GLAMPTST		# TURN OFF UNLESS LAMP TEST IN PROGRESS.

; Lamp currently OFF, trying to turn ON. Check IMU cage status.
		CAF	BIT6			# Coarse align bit mask
		MASK	IMODES30		# Check if IMU being caged
		CCS	A			# Caging in progress?
		TCF	GLOCKOK			# Yes - don't turn on lamp

; Lamp state change permitted - invert the gimbal lock lamp bit in display table
GLINVERT	CS	DSPTAB +11D		# INVERT GIMBAL LOCK LAMP.
		MASK	BIT6			# Extract gimbal lock bit (inverted)
		AD	BIT15			# TO INDICATE CHANGE IN DSPTAB +11D.
		XCH	DSPTAB +11D		# Swap with old value
		MASK	OCT37737		# Keep only the other bits
		ADS	DSPTAB +11D		# Add back to complete the inversion
		TCF	GLOCKOK			# Done with gimbal lock monitoring

; Special handling for turning lamp OFF: check if lamp test is in progress.
; If lamp test active (crew testing all indicator lamps), defer lamp turn-off
; until test completes. Otherwise proceed with inversion.

GLAMPTST	TC	LAMPTEST		# TURN OFF UNLESS LAMP TEST IN PROGRESS.
		TCF	GLOCKOK			# Lamp test active - defer turn-off
		TCF	GLINVERT		# Safe to turn off - invert state

; Gimbal angle thresholds in scaled units (half-revolutions = 180 degrees)
-70DEGS		DEC	-.38888			# -70 DEGREES SCALED IN HALF-REVOLUTIONS.
-15DEGS		DEC	-.08333			# -15 degrees (85 = 70+15)

; ============================================================================
; TRANSITION: From gimbal lock protection to IMU temperature monitoring
;
; Gimbal lock monitoring complete - crew properly warned of dangerous gimbal
; angles. T4RUPT now handles IMU temperature monitoring via TLIM routine.
; The Inertial Measurement Unit contains precision gyroscopes and accelerometers
; requiring stable thermal environment. Channel 30 bit 15 signals when IMU
; temperature exceeds operational limits. TLIM maintains TEMP lamp on DSKY to
; alert crew of thermal problems requiring corrective action (heater cycling,
; mission timeline adjustment). Critical during translunar coast when thermal
; control is passive and IMU must remain within narrow temperature band.
; ============================================================================

# Page 149
# PROGRAM NAME:  TLIM.
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM MAINTAINS THE TEMP LAMP (BIT 4 OF CHANNEL 11) ON THE DSKY TO AGREE WITH
# THE TEMP SIGNAL FROM THE ISS (BIT 15 OF CHANNEL 30).  HOWEVER, THE LIGHT WILL NOT BE TURNED OFF IF A LAMP TEST
# IS IN PROGRESS.
#
# CALLING SEQUENCE:  CALLED BY IMUMON ON A CHANGE OF BIT 15 OF CHANNEL 30.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  LAMPTEST.
#
# ERASABLE INITIALIZATION:  FRESH START AND RESTART TURN THE TEMP LAMP OFF.
#
# ALARMS:  TEMP LAMP TURNED ON WHEN THE IMU TEMP GOES OUT OF LIMITS.
#
# EXIT:  NXTIFAIL.
#
# OUTPUT:  SERVICE OF TEMP LAMP.		  IN A, EXCEPT FOR TLIM.

; IMU temperature lamp control - synchronizes DSKY TEMP lamp (channel 11 bit 4)
; with IMU temperature sensor status (channel 30 bit 15). Simple logic: if IMU
; reports over-temp, light lamp to alert crew. Respects lamp test mode.

; Entry point: bit 15 of channel 30 has changed state. Clear this bit from
; change accumulator (POSMAX mask) and store for continued multi-bit processing.

TLIM		MASK	POSMAX			# REMOVE BIT FROM WORD OF CHANGES AND SET
		TS	RUPTREG2		# DSKY TEMP LAMP ACCORDINGLY.

; Check IMU temperature status from IMODES30 (contains current channel 30 state).
; CCS test: positive = normal temp, zero = normal temp, negative = overtemp.
; If positive or zero, branch to TEMPOK to potentially turn lamp off.

		CCS	IMODES30
		TCF	TEMPOK			# Positive - temperature normal
		TCF	TEMPOK			# Zero - temperature normal

; Temperature out of limits (IMODES30 negative). Illuminate TEMP lamp to alert
; crew immediately. During Apollo 11, IMU thermal control was critical throughout
; mission - especially during translunar coast when passive thermal control
; required precise spacecraft attitude for balanced heating/cooling.

		CAF	BIT4			# TURN ON LAMP.
		EXTEND
		WOR	DSALMOUT		# Set bit 4 of channel 11 (TEMP lamp)
		TCF	NXTIFAIL		# Continue monitoring other IMU signals

; Temperature normal (within limits). Before turning lamp off, check if lamp
; test is in progress. LAMPTEST returns +0 if test active, preventing lamp
; from being turned off during test sequence.

TEMPOK		TC	LAMPTEST		# IF TEMP NOW OK, DON'T TURN OFF LAMP IF
		TCF	NXTIFAIL		# LAMP TEST IN PROGRESS.

; Lamp test not active and temperature normal - safe to turn off TEMP lamp.
; Clear bit 4 of DSALMOUT (channel 11) using WAND (write AND) with complemented
; bit mask. Crew sees lamp extinguish, confirming IMU thermal status nominal.

		CS	BIT4			# Complement to create clear mask
		EXTEND
		WAND	DSALMOUT		# TURN OFF LAMP (clear bit 4)
		TCF	NXTIFAIL		# Continue monitoring other signals

; ============================================================================
; TRANSITION: From IMU Temperature Monitoring to ISS Turn-On Sequence Management
;
; Having monitored IMU thermal status and crew warning lamps, the program now
; transitions to managing the ISS (IMU) turn-on sequence. The ITURNON routine
; handles the complex 90-second initialization sequence required when powering
; up the IMU. This was critical during Apollo 11's pre-launch countdown and
; after any IMU power cycling. The routine prevents premature turn-on attempts
; and issues alarm 00207 if the turn-on request signal fails during the delay.
; ============================================================================

# Page 150
# PROGRAM NAME:  ITURNON.
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM IS CALLED BY IMUMON WHEN A CHANGE OF BIT 14 OF CHANNEL 30 (ISS TURN-ON
# REQUEST) IS DETECTED.  UPON ENTRY, ITURNON CHECKS IF A TURN-ON DELAY SEQUENCE HAS FAILED, AND IF SO, IT EXITS.
# IF NOT, IT CHECKS WHETHER THE TURN-ON REQUEST CHANGE IS TO ON OR OFF.  IF ON, IT SETS BIT 7 OF IMODES30 TO 1 SO
# THAT TNONTEST WILL INITIATE THE ISS INITIALIZATION SEQUENCE.  IF OFF, THE TURN-ON DELAY SIGNAL, CHANNEL 12 BIT
# 15, IS CHECKED AND IF IT IS ON, ITURNON EXITS.  IF THE DELAY SIGNAL IS OFF, PROGRAM ALARM 00207 IS ISSUED, BIT 2
# OF IMODES30 IS SET TO 1 AND THE PROGRAM EXITS.
#
# THE SETTING OF BIT 2 OF IMODES30 (ISS DELAY SEQUENCE FAIL) INHIBITS THIS ROUTINE AND IMUOP FROM
# PROCESSING ANY CHANGES.  THIS BIT WILL BE RESET BY THE ENDTNON ROUTINE WHEN THE CURRENT 90 SECOND DELAY PERIOD
# ENDS.
#
# CALLING SEQUENCE:  FROM IMUMON WHEN ISS TURN-ON REQUEST CHANGES STATE.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  ALARM, IF THE ISS TURN-ON REQUEST IS NOT PRESENT FOR 90 SECONDS.
#
# ERASABLE INITIALIZATION:  FRESH START AND RESTART SET BIT 15 OF CHANNEL 12 AND BITS 2 AND 7 OF IMODES30 TO 0,
# AND BIT 14 OF IMODES30 TO 1.
#
# ALARMS: PROGRAM ALARM 00207 IS ISSUED IF THE ISS TURN-ON REQUEST SIGNAL IS NOT PRESENT FOR 90 SECONDS.
#
# EXIT:  NXTIFAIL.
#
# OUTPUT:  BIT 7 OF IMODES30 TO START ISS INITIALIZATION, OR BIT 2 OF IMODES30 AND PROGRAM ALARM 00207 TO INDICATE
# A FAILED TURN-ON SEQUENCE.

; ISS TURN-ON SEQUENCE MANAGER
; The ISS (IMU) requires a 90-second initialization delay after receiving the
; turn-on request. This routine monitors the turn-on request discrete signal
; from channel 12 and coordinates the multi-phase initialization sequence.
; During Apollo 11's pre-launch and any IMU power cycling, this ensured proper
; gyro warm-up and stabilization before accepting navigation data.
;
; Check if a 90-second delay is already in progress. BIT2 of IMODES30 flags
; an active delay sequence. If set, we must not process any new turn-on or
; turn-off requests until the current wait completes. This prevents overlapping
; initialization sequences that could corrupt IMU state.

ITURNON		CAF	BIT2		# IF DELAY REQUEST HAS GONE OFF
		MASK	IMODES30	# PREMATURELY, DO NOT PROCESS ANY CHANGES
		CCS	A		# UNTIL THE CURRENT 90 SEC WAIT EXPIRES.
		TCF	NXTIFAIL	# Delay in progress, skip processing

; No delay in progress. Determine if this is a turn-on request (bit 14 clear)
; or turn-off monitoring (bit 14 set). BIT14 of IMODES30 tracks the current
; state: 0 = ISS just turned on, 1 = ISS turned off or waiting for turn-on.
; During Apollo 11, the IMU remained powered throughout most mission phases,
; but this logic handled pre-launch power-up and any contingency power cycles.

		CAF	BIT14		# SEE IF JUST ON OR OFF.
		MASK 	IMODES30	# Extract state bit
		EXTEND
		BZF	ITURNON2	# IF JUST ON (bit 14 clear, go to turn-on path)

; Turn-off monitoring path (BIT14 set). The ISS is off or waiting for turn-on.
; Check channel 12 bit 15 to see if the "delay present" discrete signal is
; active. This signal confirms the 90-second turn-on delay is being maintained
; by the spacecraft's power distribution system. If the signal is present,
; the system is functioning correctly and no action is needed.

		CAF	BIT15		# Prepare to check delay present signal
		EXTEND			# SEE IF DELAY PRESENT DISCRETE HAS BEEN
		RAND	CHAN12		# SENT.  IF SO, ACTION COMPLETE (bit 15 read)
		EXTEND			# Result: 0 if signal absent, non-zero if present
		BZF	+2		# If signal absent (BZF), skip to alarm handling
		TCF	NXTIFAIL	# Signal present, all OK, continue monitoring

; Delay present signal is absent - the 90-second turn-on delay has failed or
; been interrupted prematurely. This indicates a hardware problem with the IMU
; power system or signal routing. Set BIT2 of IMODES30 to flag the failure
; and issue program alarm 00207 to alert the crew. During Apollo 11, this alarm
; would require crew troubleshooting or a mission rule decision about IMU usage.

		CAF	BIT2		# IF NOT, SET BIT TO INDICATE REQUEST NOT
		ADS	IMODES30	# PRESENT FOR FULL DURATION (flag failure)
		TC	ALARM		# Call alarm subroutine
		OCT	207		# Alarm code 00207: ISS turn-on sequence failed
		TCF	NXTIFAIL	# Continue monitoring other signals

# Page 151
; Turn-on path: ISS just turned on (BIT14 was clear). Set BIT7 of IMODES30 to
; initiate the ISS initialization sequence. BIT7 signals that the system should
; wait one additional T4RUPT sample (10 milliseconds) before starting the full
; IMU initialization routines (coarse align, gyro torquing, CDU zeroing).
; This brief pause allows power supply voltages to stabilize and the IMU gimbal
; position sensors to settle before commanding any gimbal motion. During Apollo 11's
; pre-launch IMU power-up, this ensured clean initialization before loading the
; launch pad alignment reference frame (REFSMMAT).

ITURNON2	CS	IMODES30	# SET BIT7 TO INDICATE WAIT OF 1 SAMPLE
		MASK	BIT7		# Extract bit 7 (complemented)
		ADS	IMODES30	# Add to IMODES30, setting bit 7
		TCF	NXTIFAIL	# Initialization flagged, continue monitoring

; ============================================================================
; TRANSITION: From ISS Turn-On Sequence to IMU Cage Processing
;
; Having managed the ISS turn-on initialization sequence, the program now
; transitions to processing the IMU CAGE button. When the crew presses the
; IMU cage button (channel 30, bit 11), this routine immediately terminates
; all IMU operations - pulse trains to gyros and CDUs cease, error counters
; are zeroed, and the NO ATT lamp illuminates. This "cages" the IMU into a
; safe, non-navigating state. During Apollo 11, the crew would cage the IMU
; before any realignment procedure (such as P51/P52 star sightings) or if
; the IMU exhibited erratic behavior requiring reinitialization.
; ============================================================================

# Page 152
# PROGRAM NAME:  IMUCAGE.
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM PROCESSES CHANGES OF THE IMUCAGE INBIT, CHANNEL 30 BITS 11.  IF THE BIT
# CHANGES TO 0 (CAGE BUTTON PRESSED), THE ISS IS CAGED (ICDU ZERO + COARSE ALIGN + NO ATT LAMP) UNTIL THE
# ASTRONAUT SELECTS ANOTHER PROGRAM TO ALIGN THE ISS.  ANY PULSE TRAINS TO THE ICDU'S AND GYRO'S ARE TERMINATED,
# THE ASSOCIATE OUTCOUNTERS ARE ZEROED AND THE GYRO'S ARE DE-SELECTED.  NO ACTION OCCURS WHEN THE BUTTON IS
# RELEASED (INBIT CHANGES TO 1).
#
# CALLING SEQUENCE:  BY IMUMON WHEN IMU CAGE BIT CHANGES.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  CAGESUB.
#
# ERASABLE INITIALZATION:  FRESH START AND RESTART SET BIT 11 OF IMODES30 TO 1.
#
# ALARMS: NONE.
#
# EXIT:  NXTIFAIL.
#
# OUTPUT:  ISS CAGED, COUNTERS ZEROED, PULSE TRAINS TERMINATED AND NO ATT LAMP LIT.

; IMU CAGE BUTTON HANDLER
; Entry from IMUMON after detecting a change on channel 30 bit 11 (IMU CAGE button).
; The A register contains the button state: 0 = button pressed (cage), +1 = button released.
; When the crew presses the IMU cage button, this routine immediately puts the IMU into
; a safe, non-navigating state by terminating all motion commands, zeroing all control
; outputs, and illuminating the NO ATT lamp. During Apollo 11, the crew would cage the IMU
; before any platform realignment (P51/P52 star sightings) or if the IMU exhibited erratic
; behavior requiring reinitialization. The caged state persists until another program
; (such as P51) re-aligns and activates the platform.

IMUCAGE		CCS	A		# NO ACTION IF GOING OFF.
		TCF	ISSZERO		# Button released (A = +1), no action needed
		
; Button pressed (A = 0): begin IMU cage sequence. First step is to immediately terminate
; all pulse trains to the ICDUs (gimbal position servos), optics CDUs (telescope/sextant
; position servos), and gyros (torquing commands). This halts any motion in progress and
; prevents the IMU from receiving conflicting commands during the cage sequence.
; Bits in OCT77000 (octal 77000 = binary 111111000000000000) correspond to pulse train
; enable bits in channel 14. Complementing this value (CS OCT77000) creates a mask that
; clears all these enable bits when ANDed with the channel.

		CS	OCT77000	# TERMINATE ICDU, OPTICS, GYRO PULSE TRAINS
		EXTEND			# Prepare for write-AND operation
		WAND	CHAN14		# Clear pulse train enable bits in channel 14

; Second step: disable multiple IMU and control system functions via channel 12. The mask
; OCT272 (octal 272 = binary 010111010) corresponds to control bits that must be cleared
; to safely cage the IMU. Complementing OCT272 creates a mask that simultaneously disables:
;   - TVC (Thrust Vector Control) enable: Prevents engine gimbal commands
;   - IMU error counter enable: Stops accumulating gyro drift and CDU errors
;   - Zero ICDU enable: Halts any active ICDU zeroing sequence
;   - Coarse align enable: Terminates any coarse alignment in progress
;   - Optics error counter enable: Stops accumulating optical tracking errors
; This ensures the IMU subsystems are in a known, inactive state.

		CS	OCT272		# KNOCK DOWN TVC ENABLE, IMU ERROR COUNTER
		EXTEND			#   ENABLE, ZERO ICDU, COARSE ALIGN
		WAND	CHAN12		#   ENABLE, OPTICS ERR CNTR ENABLE (clear control bits)

; Third step: ensure the main engine (SPS for CM) is turned off. This is a safety measure
; during the cage sequence. BIT13 in DSALMOUT corresponds to the engine on/off control.
; Clearing this bit via write-AND guarantees the engine is commanded off before proceeding
; with IMU reinitialization. During Apollo 11, this prevented any inadvertent engine firing
; during platform realignment maneuvers.

		CS	BIT13		# TURN OFF ENGINE
		EXTEND			# Prepare for write-AND
		WAND	DSALMOUT	# Clear engine enable bit

; Fourth step: call CAGESUB1, a subroutine that performs the core cage operation.
; CAGESUB1 sets the ICDU zero mode (commanding all three gimbal CDUs to zero),
; initiates coarse align mode (locking all three gimbals), and illuminates the NO ATT
; (No Attitude) lamp on the DSKY to alert the crew that the IMU is not providing valid
; attitude data. This visual indication is critical for crew situational awareness.

		TC	CAGESUB1	# Perform core cage: zero CDUs, coarse align, light NO ATT

; Fifth step: call RNDREFDR (Rendezvous Reference Driver reset) via inter-bank call.
; This subroutine clears software flags related to IMU tracking mode, the REFSMMAT
; (Reference Stable Member Matrix defining the platform's inertial reference frame),
; and IMU drift compensation flags. Clearing these flags ensures that any subsequent
; IMU initialization starts from a clean state without residual configuration from
; previous navigation modes.

		TC	IBNKCALL	# KNOCK DOWN TRACK, REFSMMAT, DRIFT FLAGS
		CADR	RNDREFDR	# Inter-bank call to reference reset routine

; Sixth step: zero all IMU command out-counters. These erasable memory locations accumulate
; incremental commands to the three CDUs (X, Y, Z gimbal position servos) and the gyros
; (torquing commands for drift compensation). Setting them to CS ZERO (complement of zero,
; which is -0 = octal 177777 = all bits set, but when stored as TS, becomes +0) ensures
; no residual commands remain that could cause unexpected motion when the IMU is later
; reactivated. This is critical for clean initialization in subsequent alignment programs.

		CS	ZERO		# ZERO COMMAND OUT-COUNTERS
		TS	CDUXCMD		# Clear X-axis CDU command accumulator
		TS	CDUYCMD		# Clear Y-axis CDU command accumulator
		TS	CDUZCMD		# Clear Z-axis CDU command accumulator
		TS	GYROCMD		# Clear gyro torquing command accumulator

; Final step: de-select the gyros by clearing additional control bits in channel 14.
; The mask OCT740 (octal 740 = binary 111100000) corresponds to gyro select and enable bits.
; Complementing OCT740 creates a mask that clears these bits via write-AND, electrically
; disconnecting the gyros from the AGC's control circuits. This is done at least 27 MCT
; (Machine Cycle Times = 27 * 11.7 microseconds ≈ 316 microseconds) after pulse train
; termination to ensure all pending gyro pulses have completed and the hardware has settled.
; De-selecting the gyros prevents any spurious signals during the cage state and conserves
; power. The IMU is now fully caged and will remain in this safe state until the crew
; initiates a new alignment program (P51/P52/P53).

		CS	OCT740		# HAVING WAITED AT LEAST 27 MCT FROM
		EXTEND			# GYRO PULSE TRAIN TERMINATION, WE CAN
		WAND	CHAN14		# DE-SELECT THE GYROS (clear select bits)
# Page 153
		TCF	NXTIFAIL	# Cage complete, continue monitoring

; ============================================================================
; TRANSITION: From IMU Cage Handler to IMU Operate Discrete Monitor
;
; With the IMU cage button handler complete, we now transition to monitoring
; the ISS OPERATE discrete signal (channel 30 bit 9), which indicates whether
; the IMU hardware is powered on or off. The IMUOP routine responds to changes
; in this signal: when the IMU is turned on, it requests initialization via
; TNONTEST; when turned off unexpectedly while programs are using it, it
; issues alarm 00214 to alert the crew of the loss of navigation capability.
; This monitoring is essential for crew awareness and system state management.
; ============================================================================

# Page 154
# PROGRAM NAME:  IMUOP.
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM PROCESSES CHANGES IN THE ISS OPERATE DISCRETE, BIT 9 OF CHANNEL 30.
# IF THE INBIT CHANGES TO 0, INDICATING ISS ON, IMUOP GENERALLY SETS BIT 7 OF IMODES30 TO 1 TO REQUEST ISS
# INITIALIZATION VIA TNONTEST.  AN EXCEPTION IS DURING A FAILED ISS DELAY DURING WHICH BIT 2 OF IMODES30 IS SET
# TO 1 AND NO FURTHER INITIALIZATION IS REQUIRED.  WHEN THE INBIT CHANGES TO 1, INDICATING ISS OFF, IMUSEFLG IS
# TESTED TO SEE IF ANY PROGRAM WAS USING THE ISS.  IF SO, PROGRAM ALARM 00214 IS ISSUED.
#
# CALLING SEQUENCE:  BY IMUMON WHEN BIT 9 OF CHANNEL 30 CHANGES.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  ALARM, IF ISS IS TURNED OFF WHILE IN USE.
#
# ERASABLE INITIALIZATION:  ON FRESH START AND RESTART, BIT 9 OF IMODES30 IS SET TO 1 EXCEPT WHEN THE GIMBAL LOCK
# LAMP IS ON, IN WHICH CASE IT IS SET TO 0.  THIS PREVENTS ICDU ZERO BY TNONTEST WITH THE ISS IN GIMBAL LOCK.
#
# ALARMS:  PROGRAM ALARM 00214 IF THE ISS IS TURNED OFF WHILE IN USE.
#
# EXIT:  NXTIFAIL.
#
# OUTPUT:  ISS INITIALIZATION REQUEST (IMODES30 BIT 7) OR PROGRAM ALARM 00214.

; ISS OPERATE DISCRETE MONITOR
; Entry from IMUMON after detecting a change on channel 30 bit 9 (ISS OPERATE discrete).
; The A register contains the discrete state: 0 = ISS turning on, +NZ = ISS turning off.
; This routine handles both power-on initialization requests and unexpected power-off
; detection. During Apollo 11, the crew controlled ISS power via the IMU OPERATE switch.
; Unexpected IMU power loss during navigation would be catastrophic, so alarm 00214 alerts
; the crew immediately if the IMU turns off while programs are actively using it.

IMUOP		EXTEND				# IF OPERATE JUST ON, WAIT 1 SAMPLE.
		BZF	IMUOP2			# A = 0, ISS turning on: branch to IMUOP2

; ISS TURNING OFF branch: A register was +NZ, meaning the ISS OPERATE discrete has transitioned
; from on to off. This can occur due to crew action (switching off the IMU) or hardware failure.
; Either way, the AGC must immediately safeguard against any programs attempting to use the now-
; unavailable navigation data. The response sequence: disable autopilot, reset reference data
; tracking flags, clear mission mode flags, and check if any program was actively using the IMU.
; If so, issue program alarm 00214 to alert the crew of lost navigation capability.

; First step: disable the Digital Autopilot (DAP) by clearing bit 6 in IMODES33. The DAP relies
; on IMU data for attitude control. Without valid gyro and accelerometer data, the DAP cannot
; safely compute thruster firing commands. This prevents the autopilot from issuing erroneous
; control commands based on stale or invalid IMU data, which could cause dangerous spacecraft
; motions (unwanted rotations, thrust vector misalignment during burns).

		CS	IMODES33		# DISABLE DAP
		MASK	BIT6			# Isolate bit 6 (DAP enable flag)
		ADS	IMODES33		# Add to IMODES33 (clearing bit 6 via complement)

; Second step: call RNDREFDR (Reference Data Reset) via inter-bank call to reset all tracking
; and reference matrix flags. These flags indicate the validity state of the IMU's reference
; coordinate system (REFSMMAT), optical tracking data, and gyro drift compensation parameters.
; With the IMU now offline, all this navigation reference data becomes invalid and must be
; marked as such to prevent programs from using stale data. When the IMU is later powered back
; on, alignment programs (P51/P52/P53) will re-establish valid reference data and set these
; flags again. This is the same reference reset routine called by IMUCAGE.

		TC	IBNKCALL		# KNOCK DOWN TRACK, REFSMMAT, DRIFT FLAGS
		CADR	RNDREFDR		# Inter-bank call to reference data reset

; Third step: clear RENDEZVOUS and IMUUSE flags in the STATE register (bits 7 and 8). These
; mission mode flags indicate that rendezvous operations are in progress and that programs are
; actively using IMU data. By masking STATE with the complement of BITS7&8 and exchanging back,
; we clear both flags. However, we preserve the old STATE value to check if IMUUSE was set.

		CS	BITS7&8			# KNOCK DOWN RENDEZVOUS, IMUUSE FLAGS
		MASK	STATE			# Preserve all bits except 7 and 8
		XCH	STATE			# Update STATE, retrieve old value in A
		COM				# Complement old STATE
		MASK	IMUSEFLG		# Isolate IMUUSE flag (was it set before?)
		CCS	A			# Test: was IMUUSE flag set in old STATE?
		TCF	NXTIFAIL		# IMUUSE was 0: no program using IMU, safe to power off

; Fourth step: issue program alarm 00214 if IMUUSE flag was set, indicating a program was actively
; using the IMU when it powered off. This is a critical alarm during mission operations. For
; example, if the IMU turns off during a navigation program (P20-series for rendezvous, P30-series
; for maneuvers, P51-P53 for alignment), the crew loses immediate navigation capability. The alarm
; allows them to take corrective action: switch IMU back on, check circuit breakers, or switch
; to backup navigation methods (sextant sightings, ground tracking). During Apollo 11, unexpected
; IMU loss during critical phases (translunar coast, lunar orbit, rendezvous) would require
; immediate crew response and possible mission plan changes.

		TC	ALARM			# IMUUSE was set: issue alarm
		OCT	214			# Program alarm 00214: IMU turned off during use
		TCF	NXTIFAIL		# Continue failure monitoring after alarm

; ISS TURNING ON branch: arrived here via BZF when A register was zero, meaning the ISS OPERATE
; discrete has just transitioned from off to on. The crew has switched the IMU to OPERATE mode,
; requesting initialization of the inertial platform. However, we must check if a previous
; turn-on sequence is still in progress (the 90-second ISS turn-on delay from ITURNON). If so,
; we wait for that sequence to complete. If not, we proceed to initiate the 90-second warm-up.

IMUOP2		CAF	BIT2			# SEE IF FAILED ISS TURN-ON SEQ IN PROG.
		MASK	IMODES30		# Check bit 2: ISS turn-on sequence active?
		CCS	A			# Test result
		TCF	NXTIFAIL		# IF SO, DON'T PROCESS UNTIL PRESENT 90
						# seconds expires (wait for current sequence)
		TCF	ITURNON2		# No active sequence: start new 90-sec turn-on

; ============================================================================
; TRANSITION: From ISS Operate Discrete Monitoring to PIPA Failure Detection
;
; Having monitored the ISS OPERATE discrete (IMU power on/off), the failure
; monitoring system now shifts to the Pulsed Integrating Pendulous Accelerometers
; (PIPAs)—the IMU's critical acceleration sensors. The three PIPAs (one per axis)
; measure spacecraft acceleration by counting pulses from pendulous accelerometers.
; PIPA failure means loss of acceleration measurement capability, which directly
; impacts navigation state propagation. Without valid PIPA data, the AGC cannot
; accurately integrate velocity and position. The PIPFAIL routine consolidates
; the hardware PIPA FAIL discrete (channel 33 bit 13) into software state flags
; and determines the appropriate crew alert level—either an ISS warning lamp or,
; if the IMU is actively operating, program alarm 00212 indicating immediate loss
; of navigation acceleration data during mission operations.
; ============================================================================

# Page 155
# PROGRAM NAME:  PIPFAIL
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM PROCESSES CHANGES OF BIT 13 OF CHANNEL 33, PIPA FAIL.  IT SETS BIT 10 OF
# IMODES30 TO AGREE.  IT CALLS SETISSW IN CASE A PIPA FAIL NECESSITATES AN ISS WARNING.  IF NOT, I.E., IMODES30
# BIT 1 = 1, AND A PIPA FAIL IS PRESENT AND THE ISS NOT BEING INITIALIZED, PROGRAM ALARM 0212 IS ISSUED.
#
# CALLING SEQUENCE:  BY C33TEST ON CHANGES OF CHANNEL 33 BIT 13.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  1) SETISSW, AND 2) ALARM (SEE FUNCTIONAL DESCRIPTION).
#
# ERASABLE INITIALIZATION:  SEE IMUMON FOR INITIALIZATION OF IMODES30.  THE RELEVANT BITS ARE 5, 7, 8, 9, AND 10.
#
# ALARMS:  PROGRAM ALARM 00212 IF PIPA FAIL IS PRESENT BUT NEITHER ISS WARNING IS TO BE ISSUED NOR THE ISS IS
# BEING INITIALIZED.
#
# EXIT:  NXTFL33.
#
# OUTPUT:  PROGRAM ALARM 00212 AND ISS WARNING MAINTENANCE.

; PIPA FAILURE HANDLER
; Entry from C33TEST after detecting a change on channel 33 bit 13 (PIPA FAIL discrete).
; The A register contains the discrete state: 0 = PIPA failure present, +NZ = PIPA operating.
; PIPAs are the IMU's three Pulsed Integrating Pendulous Accelerometers that measure spacecraft
; acceleration along the X, Y, and Z axes of the stable member. They output acceleration pulses
; that the AGC integrates to compute velocity changes. PIPA failure means the AGC loses real-time
; acceleration measurement—navigation state propagation becomes unreliable. This handler copies
; the hardware PIPA FAIL state into software flags (bit 10 of IMODES30), calls SETISSW to evaluate
; if an ISS warning lamp is appropriate, and issues alarm 00212 if the IMU is actively operating.

; First step: set or clear bit 10 in IMODES30 to match the PIPA FAIL hardware discrete state.
; This consolidates all ISS warning information into a single register (IMODES30) for easier
; monitoring. The CCS instruction tests the A register (PIPA state from channel 33 bit 13).
; If A is positive (PIPA operating normally), we set bit 10 to 1. If A is zero (PIPA failed),
; bit 10 remains 0. The MASK -BIT10 preserves all other bits while XCH/ADS updates bit 10.

PIPFAIL		CCS	A			# SET BIT10 IN IMODES30 SO ALL ISS WARNING
		CAF	BIT10			# INFO IS IN ONE REGISTER.
		XCH	IMODES30		# A > 0: PIPA OK, load BIT10 into A
		MASK	-BIT10			# Preserve all bits except 10 in old IMODES30
		ADS	IMODES30		# Add BIT10 (if PIPA OK) to update IMODES30

; Second step: call SETISSW (Set ISS Warning) to evaluate all ISS warning conditions and determine
; if the ISS WARNING lamp should be illuminated for the crew. SETISSW examines multiple failure
; indicators (PIPA fail, IMU fail, IMU temperature, gimbal lock risk) and turns on the lamp if
; any warning-level condition exists. The lamp provides immediate visual crew alert in peripheral
; vision while they focus on displays. If SETISSW determines the condition warrants only a lamp
; (not a program alarm), it sets bit 1 of IMODES30, and we exit without further action.

		TC	SETISSW			# Evaluate if ISS warning lamp needed

; Third step: check if PIPA failure requires a more urgent program alarm beyond the warning lamp.
; We test bit 1 of IMODES30 (ISS warning lamp state set by SETISSW). If bit 1 is set (lamp on),
; the condition has been handled by the warning system, and we exit. If bit 1 is clear (no lamp),
; we proceed to check if the IMU is in an operational state where PIPA failure is alarm-worthy.

		CS	IMODES30		# IF PIP FAIL DOESN'T LIGHT ISS WARNING, DO
		MASK	BIT1			# A PROGRAM ALARM IF IMU OPERATING BUT NOT
		CCS	A			# CAGED OR BEING TURNED ON.
		TCF	NXTFL33			# ISS warning lamp is on: exit, crew alerted

; No ISS warning lamp: check if the IMU is in a state where PIPA failure is alarm-worthy. We load
; IMODES30 and mask with OCT1720 (octal 1720 = binary 001111010000). This mask checks bits 10, 9, 8,
; 7, and 5 simultaneously. These bits represent: bit 10 = PIPA operational flag (set by this routine),
; bit 9 = IMU fail state, bit 8 = IMU temperature out of limits, bit 7 = IMU being caged or turned on,
; bit 5 = gimbal lock condition. If ANY of these bits is set, we exit without alarm because the
; condition is either not a PIPA failure (bit 10 set means PIPA OK) or the IMU is not in normal
; operating state (being initialized, caged, failed, or in gimbal lock). Only if ALL these bits are
; clear (A = 0 after CCS) do we proceed to issue the program alarm.

		CA	IMODES30		# Load IMODES30 to check operational state
		MASK	OCT1720			# Check bits 10,9,8,7,5 (PIPA state, IMU conditions)
		CCS	A			# Test result: A > 0 means at least one bit set
		TCF	NXTFL33			# ABOVE CONDITION NOT MET.

; All checked bits clear: PIPA has failed (bit 10 = 0) AND the IMU is in normal operating state (not
; caged, not warming up, not failed, not in gimbal lock). This is the critical alarm condition: loss
; of acceleration measurement during active navigation. Issue program alarm 00212 to alert crew.
; During Apollo 11 operations, this alarm would have required immediate crew assessment and possible
; mission rule consultation with flight controllers.

		TC	ALARM			# Issue PIPA failure alarm
		OCT	212			# Program alarm 00212
		TCF	NXTFL33			# Continue channel 33 monitoring

; ============================================================================
; TRANSITION: From PIPA Failure Detection to Telemetry Link Monitoring
;
; Having handled IMU sensor failures (gimbal lock, IMU failure, PIPA failure),
; the channel 33 monitoring system now shifts to the spacecraft's communication
; links with Earth. The AGC monitors two critical telemetry data rate discretes:
; DOWNLINK TOO FAST (bit 12) and UPLINK TOO FAST (bit 11). These discretes
; indicate that the ground station (MSFN—Manned Space Flight Network) or the
; spacecraft's S-band communication system cannot keep pace with the data rate
; being commanded. During Apollo 11, Mission Control relied on constant telemetry
; streams for monitoring spacecraft health, navigation state, and system status.
; If data rates exceed hardware capabilities, the link degrades or fails entirely,
; blinding controllers to spacecraft state. DNTMFAST and UPTMFAST issue immediate
; program alarms (01105 and 01106) if these discretes transition to 0 (failure
; state), alerting the crew to reduce commanded data rates or troubleshoot the
; communication system before critical mission data is lost.
; ============================================================================

# Page 156
# PROGRAM NAMES:  DNTMFAST, UPTMFAST
#
# FUNCTIONAL DESCRIPTION:  THESE PROGRAMS PROCESS CHANGES OF BITS 12 AND 11 OF CHANNEL 33.  IF A BIT CHANGES TO A
# 0, A PROGRAM ALARM IS ISSUED.  THE ALARMS ARE:
#
#	BIT	ALARM	CAUSE
#	---	-----	-----
#	 12	01105	DOWNLINK TOO FAST
#	 11	01106	UPLINK TOO FAST
#
# CALLING SEQUENCE:  BY C33TEST ON A BIT CHANGE.
#
# SUBROUTINES CALLED:  ALARM, IF A BIT CHANGES TO A 0.
#
# ERASABLE INITIALIZATION:  FRESH START OR RESTART, BITS 12 AND 11 OF IMODES33 ARE SET TO 1.
#
# ALARMS:  SEE FUNCTIONAL DESCRIPTION.
#
# EXIT:  NXTFL33.
#
# OUTPUT:  PROGRAM ALARM ON A BIT CHANGE TO 0.

; ---- DNTMFAST: Downlink Telemetry Too Fast Monitor ----
;
; Entry: Called by C33TEST when bit 12 of channel 33 changes state. A register contains the isolated
; bit value: A > 0 if the DOWNLINK TOO FAST discrete is now set (1 = normal state, downlink OK),
; A = 0 if the discrete has just transitioned to 0 (failure state, downlink too fast). This routine
; determines whether to issue an alarm based on the new bit state.
;
; The DOWNLINK TOO FAST discrete indicates that the spacecraft's telemetry system is transmitting
; data to Earth (via S-band to MSFN ground stations) faster than the communication link can reliably
; handle. This can occur during high-rate telemetry modes, memory dumps, or when link margins are
; degraded due to antenna pointing errors, increasing range, or RF interference. If the downlink
; exceeds capacity, telemetry data becomes corrupted or lost, blinding Mission Control to spacecraft
; state. During Apollo 11, continuous telemetry monitoring was essential for flight controllers to
; validate navigation, assess system health, and make real-time decisions during critical phases
; (TLI, LOI, descent, landing). Program alarm 01105 alerts the crew to reduce downlink data rate
; (switch to low-rate mode) or troubleshoot the S-band antenna pointing.
;
; COMMENT-ONLY READERS: When the radio link to Earth was overloaded and couldn't keep up with the
; spacecraft's data transmission, this routine alerted the crew to slow down the data rate so
; Mission Control wouldn't lose critical spacecraft information.
;
; CODE-ALONG READERS: Test A register value (already isolated bit 12). If A > 0, bit just changed
; to 1 (downlink now OK): exit without alarm. If A = 0, bit just changed to 0 (downlink failed):
; issue program alarm 01105 to alert crew.

DNTMFAST	CCS	A			# DO PROG ALARM IF TM TOO FAST.
		TCF	NXTFL33			# A > 0: bit changed to 1 (downlink OK), exit

; Bit 12 changed to 0: downlink transmission rate exceeds system capacity. Issue program alarm 01105
; (DOWNLINK TOO FAST). The crew should respond by switching to lower telemetry data rate mode or
; checking S-band antenna pointing. Mission Control will see intermittent or corrupted telemetry
; until the issue is resolved. During critical mission phases, this alarm requires immediate action.

		TC	ALARM			# Bit = 0: downlink too fast, issue alarm
		OCT	1105			# Program alarm 01105: DOWNLINK TOO FAST
		TCF	NXTFL33			# Continue channel 33 monitoring

; ---- UPTMFAST: Uplink Telemetry Too Fast Monitor ----
;
; Entry: Called by C33TEST when bit 11 of channel 33 changes state. A register contains the isolated
; bit value: A > 0 if the UPLINK TOO FAST discrete is now set (1 = normal state, uplink OK), A = 0
; if the discrete has just transitioned to 0 (failure state, uplink too fast). This routine determines
; whether to issue an alarm based on the new bit state.
;
; The UPLINK TOO FAST discrete indicates that Mission Control is sending commands to the spacecraft
; faster than the AGC's command receiver and processing system can reliably handle. This can occur if
; ground controllers queue multiple urgent commands (state vector updates, verb/noun entries, program
; mode changes, maneuver targeting parameters) in rapid succession during time-critical operations.
; If the uplink rate exceeds the AGC's command buffer capacity, commands may be corrupted, dropped,
; or executed out of sequence, potentially causing navigation errors, incorrect burn parameters, or
; missed emergency procedures. During Apollo 11, Mission Control relied on uplink commands to send
; navigation corrections computed by ground-based systems, update mission timelines, and activate
; backup modes. Program alarm 01106 alerts the crew to request that Mission Control reduce the uplink
; command rate, allowing the AGC to properly process each command.
;
; COMMENT-ONLY READERS: When Mission Control on Earth was sending commands too rapidly for the
; computer to process them safely, this routine warned the crew to ask the ground to slow down so
; no commands would be lost or corrupted.
;
; CODE-ALONG READERS: Test A register value (already isolated bit 11). If A > 0, bit just changed
; to 1 (uplink now OK): exit without alarm. If A = 0, bit just changed to 0 (uplink failed): issue
; program alarm 01106 to alert crew.

UPTMFAST	CCS	A			# SAME AS DNLINK TOO FAST WITH DIFFERENT
		TCF	NXTFL33			# ALARM CODE.

; Bit 11 changed to 0: uplink command rate exceeds AGC processing capacity. Issue program alarm 01106
; (UPLINK TOO FAST). The crew should communicate with Mission Control (via voice loop) to reduce the
; uplink command rate. Ground controllers will queue commands more slowly to ensure reliable reception
; and execution. This is critical during powered flight phases where command timing is essential.

		TC	ALARM			# Bit = 0: uplink too fast, issue alarm
		OCT	1106			# Program alarm 01106: UPLINK TOO FAST
		TCF	NXTFL33			# Continue channel 33 monitoring

# Page 157
; ============================================================================
; TRANSITION: From IMU failure monitoring to crew warning system
;
; The spacecraft's Inertial Measurement Unit (IMU) provides critical attitude
; and navigation data. When failures occur in the IMU, ICDUs (gimbal angle
; sensors), or PIPAs (accelerometers), the crew must be immediately warned
; via the ISS WARNING lamp on the main display panel. This routine evaluates
; multiple failure conditions and their inhibit states to determine proper
; lamp status, ensuring the crew is always aware of inertial system health.
; ============================================================================

# PROGRAM NAME:  SETISSW
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM TURNS THE ISS WARNING LAMP ON AND OFF (CHANNEL 11 BIT 1 = 1 FOR ON,
# 0 FOR OFF) DEPENDING ON THE STATUS OF IMODES30 BITS 13 (IMU FAIL) AND 4 (INHIBIT IMU FAIL), 12 (ICDU FAIL) AND
# 3 (INHIBIT ICDU FAIL), AND 10 (PIPA FAIL) AND 1 (INHIBIT PIPA FAIL).  THE LAMP IS LEFT ON IF A LAMP TEST IS IN
# PROGRESS.
#
# CALLING SEQUENCE:  CALLED BY IMUMON ON CHANGES TO IMU FAIL AND ICDU FAIL.  CALLED BY IFAILOK AND PFAILOK UPON
# REMOVAL OF THE FAIL INHIBITS.  CALLED BY PIPFAIL WHEN THE PIPA FAIL DISCRETE CHANGES.  IT IS CALLED BY PIPUSE
# SINCE THE PIPA FAIL PROGRAM ALARM MAY NECESSITATE AN ISS WARNING, AND LIKEWISE BY PIPFREE WHEN THE ALARM DEPARTS
# AND IT IS CALLED BY IMUZERO3 AND ISSUP AFTER THE FAIL INHIBITS HAVE BEEN REMOVED.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  NONE.
#
# ERASABLE INITIALIZATION:
#
#	1) IMODES30 -- SEE IMUMON.
#	2) IMODES33 BIT 1 = 0 (LAMP TEST NOT IN PROGRESS).
#
# ALARMS:  ISS WARNING.
#
# EXIT: VIA Q.
#
# OUTPUT: ISS WARNING LAMP SET PROPERLY.

; The ISS WARNING lamp provides critical crew notification of inertial system
; failures. The lamp is located on the main display panel and alerts the crew
; to IMU, ICDU, or PIPA failures. The lamp state depends on three failure
; conditions (bits 13, 12, 10 of IMODES30) and three corresponding inhibit
; flags (bits 4, 3, 1 of IMODES30). If any failure is present AND its inhibit
; is not set, the lamp turns on. During lamp test mode (IMODES33 bit 1), the
; lamp remains on regardless of failure status.

; ISS WARNING lamp control logic: The routine performs a complex bit manipulation
; to check if any of three failure conditions (IMU, ICDU, PIPA) are active AND
; not inhibited. The algorithm uses multiplication and rotation to align failure
; bits with their corresponding inhibit bits for comparison.

SETISSW		CAF	OCT15			# SET ISS WARNING USING THE FAIL BITS IN
		MASK	IMODES30		# BITS 13, 12, AND 10 OF IMODES30 AND THE
		EXTEND				# FAILURE INHIBIT BITS IN POSITIONS
		MP	BIT10			# 4, 3, AND 1.
		
; After masking to get inhibit bits (4,3,1), multiply by BIT10 (octal 400)
; shifts these bits left 8 positions, aligning them with failure bit positions.
; This prepares for comparison of failures against their inhibits.
		
		CA	IMODES30
		EXTEND
		ROR	LCHAN			# 0 INDICATES FAILURE
		
; Rotate IMODES30 right, bringing failure bits (13,12,10) into alignment with
; the shifted inhibit bits. The ROR instruction rotates through L register.
		
		COM
		MASK	OCT15000
		
; Complement (COM) inverts bits so 0=failure becomes 1=failure for testing.
; Mask with OCT15000 (bits 15,13,12,10) isolates the failure indication bits.
; If result is non-zero, at least one uninhibited failure exists.
		
		CCS	A
		TCF	ISSWON			# FAILURE.

; No uninhibited failures detected - turn off ISS WARNING lamp unless lamp test
; is in progress. During lamp test (IMODES33 bit 1 = 1), all warning lamps
; remain illuminated for crew verification regardless of actual system status.

ISSWOFF		CAF	BIT1			# DON'T TURN OFF ISS WARNING IF LAMP TEST
		MASK	IMODES33		# IN PROGRESS.
		CCS	A
		TC	Q			# Lamp test active - leave lamp on
		
; Lamp test not active - safe to turn off ISS WARNING lamp since no failures.
; DSALMOUT channel controls display system alarm outputs including ISS WARNING.

		CS	BIT1
		EXTEND
		WAND	DSALMOUT		# TURN OFF ISS WARNING.
		TC	Q

; Uninhibited failure detected - turn on ISS WARNING lamp and issue program
; alarm to inform crew of the specific failure condition. VARALARM generates
; appropriate alarm codes based on which failure(s) are active (IMU fail 00210,
; ICDU fail 00211, or PIPA fail via program alarm 01206).

ISSWON		EXTEND
# Page 158
		QXCH	ITEMP6			# Save return address in temp
		TC	VARALARM		# TELL EVERYONE WHAT CAUSED THE ISS WARNING
		
; VARALARM examines IMODES30 failure bits and issues corresponding program
; alarms, providing crew with specific failure information beyond the lamp.
		
		CAF	BIT1
		EXTEND
		WOR	DSALMOUT		# TURN ON ISS WARNING
		
; Write OR operation sets bit 1 of DSALMOUT channel high, illuminating the
; ISS WARNING lamp on the main display panel. This provides immediate visual
; feedback to crew of inertial system degradation.
		
		TC	ITEMP6			# Return to caller

; ============================================================================
; IMU CAGE SUBSYSTEM ROUTINES
;
; When the IMU is caged (gyros locked to spacecraft body frame) or turned on
; from a powered-off state, multiple spacecraft systems must be reconfigured.
; The error counters driving the IMU gimbals must be disabled, the NO ATT
; lamp must illuminate (indicating attitude reference is invalid), and various
; mode flags must be set to inhibit autopilot functions that depend on valid
; attitude data. These routines prepare the spacecraft for IMU reinitialization.
; ============================================================================

; CAGESUB: Configure IMU control channels for cage or turn-on sequence.
; Disables the torque error counters that normally drive gimbal positioning,
; removes IMU delay compensation, and commands the IMU to zero and coarse align
; mode. This prepares the platform for reorientation.

CAGESUB		CS	BIT15+6			# SET OUTBITS + INTERNAL FLAGS FOR
		EXTEND				# SYSTEM TURN-ON OR CAGE.  DISABLE THE
		WAND	CHAN12			# ERROR COUNTER AND REMOVE THE IMU DELAY COMP.
		
; Clear bits 15 and 6 of channel 12 (IMU CDU command channel). Bit 15 disables
; error counter drive, bit 6 removes delay compensation. These must be off
; during cage/turn-on to prevent gimbal motion commands.
		
		CAF	BITS4&5			# SEND ZERO AND COARSE.
		EXTEND
		WOR	CHAN12
		
; Set bits 4 and 5 of channel 12, commanding IMU to zero and coarse align mode.
; In this mode, gyros are caged and platform accepts coarse alignment commands
; for initial orientation setup.

; CAGESUB1: Illuminate the NO ATT (No Attitude) lamp on main display panel.
; This critical crew warning indicates the IMU is not providing valid attitude
; reference. The crew must not rely on attitude displays or autopilot functions
; until the IMU is realigned and the lamp is extinguished.

CAGESUB1	CS	DSPTAB +11D		# TURN ON NO ATT LAMP
		MASK	OC40010
		ADS	DSPTAB +11D
		
; Sets appropriate bits in display table entry 11D to command NO ATT lamp on.
; The display system updates output channels based on this table during
; regular display refresh cycles.

; CAGESUB2: Set internal mode flags to reflect caging/turn-on state.
; Inhibits all ISS warning information (since IMU failures are expected during
; cage/turn-on), sets cage/turn-on status flags, and disables Digital Autopilot
; automatic and hold modes (which require valid IMU data).

CAGESUB2	CS	IMODES30		# SET FLAGS TO INDICATE CAGING OR TURN-ON
		MASK	OCT75			# AND INHIBIT ALL ISS WARNING INFO
		ADS	IMODES30
		
; OCT75 (bits 6,5,4,3,2,1,0) includes the three ISS warning inhibit bits plus
; cage/turn-on status flags. Setting these prevents spurious failure warnings
; during the normal cage/initialization sequence.

		CS	IMODES33		# DISABLE DAP AUTO AND HOLD MODES
		MASK	BIT6
		ADS	IMODES33
		
; Clear bit 6 of IMODES33, disabling DAP (Digital Autopilot) automatic and
; hold modes. These autopilot functions require valid IMU attitude reference
; and must not operate while IMU is caged or reinitializing.

		TC	Q			# Return to caller

; IMUFAIL and ICDUFAIL entry points: These aliases allow the same SETISSW
; routine to be called via different symbolic names when IMU or ICDU failures
; are detected, improving code readability without duplicating logic.

IMUFAIL		EQUALS	SETISSW
ICDUFAIL	EQUALS	SETISSW

# Page 159
# ============================================================================
; TRANSITION: From IMU cage/align routines to optics monitoring
;
; Having completed the IMU inertial subsystem monitoring and cage operations,
; attention now shifts to the spacecraft's optics subsystem. The jump tables
; and constants that follow support both IMU failure handling and optics mode
; transitions, bridging between the two critical navigation sensor systems.
; ============================================================================

# JUMP TABLES AND CONSTANTS.

; CHANNEL 30 DISPATCH JUMP TABLE (IMU Discrete Input Monitor)
; Indexed vector table for handling IMU-related discrete signals from Channel 30.
; Each entry corresponds to a specific bit transition requiring servicing.
; Used by interrupt handlers to quickly dispatch to appropriate IMU routines.

IFAILJMP	TCF	ITURNON			# CHANNEL 30 DISPATCH.
		TCF	IMUFAIL
		TCF	ICDUFAIL
		TCF	IMUCAGE
30RDMSK		OCT	76400			# (BIT 10 NOT SAMPLED HERE).
		TCF	IMUOP

; CHANNEL 33 DISPATCH JUMP TABLE (IMU Status Monitor)
; Indexed vector table for handling IMU status signals from Channel 33.
; Monitors temperature sensors and operational status bits requiring rapid response.

C33JMP		TCF	PIPFAIL			# CHANNEL 33 DISPATCH.
		TCF	DNTMFAST
		TCF	UPTMFAST

# SUBROUTINE TO SKIP IF LAMP TEST NOT IN PROGRESS.

; LAMP TEST CHECK SUBROUTINE
; Determines whether DSKY lamp test is in progress by checking IMODES33 bit 1.
; During lamp test, certain alarm lights must remain on regardless of system
; status. Returns via ZOPFIN3 if lamp test active, falls through otherwise.

LAMPTEST	CS	IMODES33		# BIT 1 OF IMODES33 = 1 IF LAMP TEST IN
		MASK	BIT1			# PROGRESS.
		TCF	ZOPFIN3

; CONSTANT DEFINITIONS FOR IMU AND OPTICS MONITORING
; These octal and decimal constants define timing thresholds, bit masks,
; and operational parameters for inertial and optical subsystem monitoring.

33RDMSK		EQUALS	PRIO16			; Channel 33 read mask
OC40010		OCT	40010			; Mode bit pattern
OCT54		OCT	54			; Timing constant
OCT75		OCT	75			; Status mask
OCT272		OCT	00272			; Control word
BITS7&8		OCT	300			; Bits 7 and 8 mask
OCT1720		OCT	1720			; Configuration pattern
OCT740		OCT	00740			; Status bits
OCT15000	EQUALS	PRIO15			; Priority 15 constant
OCT77000	OCT	77000			; Full status mask
-BIT10		OCT	-1000			; Negative bit 10

90SECS		DEC	9000			; 90 second timeout (centiseconds)
120MS		=	OCT14			# (DEC12) 120 millisecond interval
GLOCKOK		EQUALS	RESUME			; Gimbal lock check complete

# Page 160
# ============================================================================
; TRANSITION: From IMU monitoring to Optics CDU monitoring
;
; The Command Module optics system consists of a scanning telescope (sextant)
; and direct-view telescope mounted in the spacecraft navigation station.
; Both instruments can be positioned by Optics Coupling Data Units (OCDUs)
; providing shaft and trunnion angle control. This routine monitors OCDU
; health status and manages mode transitions between manual, computer-controlled,
; and zero-positioning operations essential for star tracking navigation.
; ============================================================================

# OPTICS MONITORING AND ZERO ROUTINES

; OPTICS MONITOR MAIN ENTRY POINT
; Continuously monitors optics subsystem discrete inputs from Channels 30 and 33
; to detect Optics CDU (OCDU) failures and manage mode transitions. The optics
; system is critical for navigational star sightings used to update the IMU
; platform alignment during translunar and transearth coast phases.

OPTMON		CA	OPTMODES		# MONITOR OPTICS INBITS IN CHAN 30 AND 33
		EXTEND
		RXOR	CHAN30			# LOOK FOR OCDU FAIL BIT CHANGE
		MASK	BIT7
		TS	RUPTREG1		# STORE CHANGE BIT
		CCS	A
		TC	OCDUFTST		# PROCESS OCDUFAIL BIT CHANGE

33OPTMON	CCS	OPTIND			# BYPASS IF TVC TAKEOVER
		TCF	+4
		TCF	+3
		TCF	+2
		TCF	RESUME

		CA	OPTMODES		# LOOK FOR OPTICS MODE SWITCH CHANGE
		EXTEND
		RXOR	CHAN33
		MASK	OCTHIRTY
		ADS	RUPTREG1		# STORE INBIT CHANGES
		LXCH	OPTMODES
		EXTEND
		RXOR	LCHAN
		TS	OPTMODES		# UPDATE OPTMODES TO SHOW BIT CHANGES

		COM				# SAMPLE CURRENT SWITCH SETTING
		MASK	OCTHIRTY
		EXTEND
		BZF	SETSAMP			# MANUAL-SET ZERO IN SWSAMPLE

		MASK	BIT5			# SEE IF CSC
		CCS	A
		TC	+2			# CSC-SET SWSAMPLE POS
		CAF	NEGONE			# ZOPTICS-SET SWSAMPLE (-1)
SETSAMP		TS	SWSAMPLE		# CURRENT OPTICS SWITCH SETTING

PROCESSW	CCS	DESOPMOD		# BRANCH ON PREVIOUS SETTING
		TC	CSCDES			# CSC
		TC	MANUDES			# MANUAL
		TC	ZOPTDES			# ZERO OPTICS
# Page 161
ZOPTDES		CCS	SWSAMPLE		# IS SWITCH STILL AT ZOPTICS
		TC	ZTOCSC			# NOW AT CSC
		TC	ZTOMAN			# MANUAL
		TC	ZOPFIN1			# ZOPTICS-SEE IF ZOPT PROCESSING	// Should be TC ZOPFINI
		TC	SETDESMD		# ZOPT NOT PROCESSING-NO ACTION

		CCS	ZOPTCNT			# ZOPT PROCESSING-CHECK COUNTER
		TC	SETCNT			# 32 SAMPLE NOT FINISHED-SET COUNTER
		TC	SETZOEND		# 32 SAMPLE WAIT COMPLETED-SET UP ZOP END

ZTOMAN		TC	ZOPFIN1			# ZOP TO MANUAL-IS ZOPT DONE		// Should be TC ZOPFINI
		TC	SETDESMD		# YES-NORMAL EXIT

ZOPALARM	TC	ALARM			# ALARM-SWITCHED ALTERED WHILE ZOPTICS
		OCT	00116
		CAF	OCT13			# PROCESSING-SET RETURN OPTION
		TS	WTOPTION

		TC	CANZOPT			# CANCEL ZOPT

		TC	SETDESMD

ZTOCSC		TC	ZOPFIN1			# SEE IF ZOPT PROCESSING		// Should be TC ZOPFINI
		TC	MANTOCSC +3		# NO-CHECK RETURN TO COARS OPT
		TC	ALARM			# ZOPT PROCESSING-ALARM
		OCT	00116
		TC	CANZOPT			# CANCEL ZOPT
		TC	MANTOCSC		# ZERO CNT-LOOK FOR COARS OPT RETURN

COARSLOK	CAF	BIT9			# IF COARS OPT SINCE FSTART GO TO L+2
		TCF	ZOPFIN2			# IF NOT GO TO L+1
ZOPFIN1		CAF	BIT1			# SEE IF END ZOPT TASK WORKING	// Label should be ZOPFINI
		MASK	OPTMODES
		CCS	A
		TC	RESUME			# ZOPT TASK WORKING-WAIT ONE SAMPLE PERIOD

		CAF	BIT3			# TEST IF ZOPTICS PROCESSING
ZOPFIN2		MASK	OPTMODES		# RETURNS TO L+1 PROCESSING AND
ZOPFIN3		CCS	A
		INCR	Q			# L+2 IF NOT
		TC	Q

CANZOPT		CS	SIX			# CANCEL ZERO OPTICS
		MASK	OPTMODES		# ZERO ZOPT PROCESSING BIT-ENABLE OCDUFAIL
		TS	OPTMODES
		CS	BIT1			# MAKE SURE ZERO OCDU IS OFF
		EXTEND
		WAND	CHAN12
		TC	Q

# Page 162
MANUDES		CCS	SWSAMPLE		# SEE IF SWITCH STILL IN MANUAL MODE
		TC	MANTOCSC		# NOW AT CSC
		TC	MANTOMAN		# STILL MANUAL
		CCS	WTOPTION		# ZOPTICS-LOOK AT ZOPTICS RETURN OPTION
		TC	+2			# 5 SEC RETURN GOOD-CONTINUE ZOPTICS
		TC	OPTZERO			# ZOPTICS MUST START ANEW

		TC	INITZOPT		# SHOW ZERO OPTICS PROCESSING
		TC	SETDESMD		# NORMAL EXIT

MANTOMAN	CCS	WTOPTION		# DECREMENT RETURN OPTION TIME
		TS	WTOPTION
		TC	SETDESMD

MANTOCSC	CAF	ZERO			# CANCEL ZOPT RETURN OPTION IF SET
		TS	WTOPTION
		TS	ZOPTCNT

		TC	COARSLOK		# CHECK FOR COARS OPT RETURN
		TC	SETDESMD		# NO COARS TASK-NO ACTION

		CAF	ONE			# SET COARS OPT WORKING
		TS	OPTIND
		CAF	BIT2			# ENABLE OPTICS CDU ERROR CNTS
		EXTEND
		WOR	CHAN12

		TC	SETDESMD

CSCDES		CCS	SWSAMPLE		# SEE IF SWITCH STILL AT CSC
		TC	SETDESMD		# STILL AT CSC
		TC	CSCTOMAN		# MANUAL
CSCTOZOP	CAF	OCT40			# ZOPTICS-INITIALIZE FOR ZOPT
		TS	ZOPTCNT
		TC	INITZOPT

CSCTOMAN	CCS	OPTIND			# SEE IF COARS WORKING
		TC	CANCOARS		# COARS WORKING-SWITCH NOT CSC-KILL COARS
		TC	CANCOARS
		TC	+1			# NO COARS-NORMAL EXIT
		TC	SETDESMD
# Page 163
CANCOARS	CA	NEGONE
		TS	OPTIND			# SET OPTIND (-1) TO SHOW NOT WORKING
		CS	BIT2			# DISABLE OCDU ERR CNTS
		EXTEND
		WAND	CHAN12
		CS	OPTMODES		# SET RETURN-TO-COARS BIT
		MASK	BIT9
		ADS	OPTMODES

		TC	SETDESMD
OPTZERO		TC	INITZOPT		# INITIALIZE ZERO OPTICS

		CA	OCT40			# SET UP 32 SAMPLE WAIT
SETCNT		TS	ZOPTCNT
SETDESMD	CA	SWSAMPLE		# SET CURRENT SWITCH INDICATION-RESUME
		TS	DESOPMOD
		TC	RESUME

SETZOEND	CAF	BIT1			# SEND ZERO OPTICS CDU
		EXTEND
		WOR	CHAN12
		CA	200MS			# HOLD ZERO CDU FOR 200 MS
		TC	WAITLIST
		EBANK=	OPTMODES
		2CADR	ENDZOPT

		CS	OPTMODES		# SHOW ZOPTICS TASK WORKING
		MASK	BIT1
		ADS	OPTMODES

		TC	SETDESMD

ENDZOPT		TC	ZEROPCDU		# ZERO OCDU COUNTERS
		CS	BIT1			# TURN OFF ZERO OCDU
		EXTEND
		WAND	CHAN12
		CAF	200MS			# DELAY 200MS FOR CDUS TO RESYNCHRONIZE
		TC	VARDELAY

		CS	OPTMODES		# SHOW ZOPTICS SINCE LAST FRESH START
		MASK	BIT10			#	OR RESTART
		ADS	OPTMODES

		CS	SEVEN			# ENABLE OCDUFAIL-SHOW OPTICS COMPLETE
		MASK	OPTMODES
		TS	OPTMODES

		TC	OCDUFTST		# CHECK OCDU FAIL BIT AFTER ENABLE.
# Page 164
		TC	TASKOVER

ZEROPCDU	CAF	ZERO
		TS	CDUS			# ZERO IN CDUS, -20 IN CDUT
		TS	ZONE			# INITIALIZE SHAFT MONITOR ZONE
		CS	20DEGS
		TS	CDUT
		TC	Q

INITZOPT	CAF	ZERO			# INITIALIZE ZOPTICS-INHIBIT OCDUFAIL
		TS	WTOPTION		# AND SHOW OPTICS PROCESSING
		CS	OPTMODES		# SET ZERO OPTICS PROCESSING
		MASK	SIX			#	OPTICS CDU FAIL INHIBITED
		ADS	OPTMODES
		TC	Q

# Page 165
OCDUFTST	CAF	BIT7			# SEE IF OCDUFAIL ON OR OFF
		EXTEND
		RAND	CHAN30
		CCS	A
		TCF	OPFAILOF		# OCDUFAIL LIGHT OFF

		CAF	BIT2			# OCDUFAIL LIGHT ON UNLESS INHIBITED
		MASK	OPTMODES
		CCS	A
		TC	Q			# OCDUFAIL INHIBITED

OPFAILON	CAF	BIT8			# ON BIT
		AD	DSPTAB	+11D
		MASK	BIT8
SETOFF		EXTEND
		BZF	TCQ			# NO CHANGE

		TS	L
		CA	DSPTAB	+11D
		EXTEND
		RXOR	LCHAN
		MASK	POSMAX
		AD	BIT15			# SHOW ACTION WANTED
		TS	DSPTAB	+11D
		TC	Q

OPFAILOF	CAF	BIT1			# DON'T TURN OFF IF LAMP TEST
		MASK	IMODES33
		CCS	A
		TC	Q			# LAMP TEST IN PROGRESS

		CAF	BIT8			# TURN OFF OCDUFAIL LIGHT
		MASK	DSPTAB	+11D
		TCF	SETOFF

OCT13		=	ELEVEN
OCTHIRTY	EQUALS	BITS4&5
20DEGS		DEC	7199
OCT40		EQUALS	BIT6
200MS		EQUALS	OCT24

# Page 166
# OPTICS CDU DRIVING PROGRAM

		BANK	10
		SETLOC	OPTDRV
		BANK
		COUNT*	$$/SXT

# ============================================================================
# SHAFT STOP MONITOR - ZONE UPDATE
#
# The Command Module sextant has mechanical stops preventing the shaft axis
# from rotating continuously. This routine monitors shaft position and updates
# a "zone" indicator to track which quadrant the optics are pointed in.
# Critical for preventing commands that would drive the sextant into its stops
# during navigation marks and star sightings.
#
# COMMENT-ONLY READERS: The sextant could only rotate about 270 degrees before
#        hitting stops. This routine tracked position to avoid crashes.
# CODE-ALONG READERS: Zone tracking algorithm using 45-degree threshold checks
#        and sign preservation for shaft stop avoidance logic.
# ============================================================================

; Shaft stop monitoring determines which angular zone the sextant occupies:
; Zone 0: Shaft within ±45° of zero (safe central region)
; Zone +: Shaft >45° positive (approaching positive mechanical stop)
; Zone -: Shaft >45° negative (approaching negative mechanical stop)
; This prevents commanding the sextant into its physical rotation limits.

OPTDRIVE	CA	CDUS			# GRAB OPTIC SHAFT CDU
		TS	L			; Save shaft angle in L for sign check
		CCS	A			# GET ABS(CDUS)
		AD	13,14,15		; Constant = -45 degrees
		TCF	+2			# ABS(CDUS) - 45 DEG
		TCF	-2
		EXTEND
		BZMF	OZONE			# LESS THAN 45 DEG-SET ZONE 0
		
		; Shaft is beyond 45° from zero
		; If zone already set to + or -, preserve it
		; If zone was zero, set it to sign of current shaft position
		CA	ZONE			# IF ZONE ZERO, CHANGE TO + OR - OTHERWISE
		EXTEND				# DON'T MESS WITH ZONE
		BZF	+2
		TCF	CONTDRVE		# JUST CONTINUE
		XCH	L			# GREATER THAN 45 DEG-SET ZONE TO SIGN CDU
		TCF	OZONE	+1
		
OZONE		CAF	ZERO			# ABS(CDUS) LESS THAN 90 DEG-ZONE ZERO
		TS	ZONE			; Central safe zone: no stop danger
		COUNT*	$$/T4RUPT
		
; Check OPTIND to dispatch to appropriate optics operation
; OPTIND = 0: Process both axes
; OPTIND = 1: Process both axes  
; OPTIND = 2,3: No operation
CONTDRVE	CCS	OPTIND
		TC	+4			# WORK COARS OPTICS
		TC	+3			# WORK COARS OPTICS
		TC	RESUME			# NO OPT
		TC	RESUME			# NO OPT

; Verify optics are under computer control before driving
; SWSAMPLE indicates mode switch position: CMC (+1) or MANUAL (0 or -1)
		CA	SWSAMPLE		# SEE IF SWITCH AT CMC
		EXTEND
		BZMF	RESUME			# ZERO (-1)	MANUAL (+0)

; Check that optics CDU has been properly zeroed after last fresh start
; BIT10 in OPTMODES indicates zero calibration status
; If not zeroed, alarm 00120 alerts crew to recalibrate sextant
		CAF	BIT10			# SEE IF OCDUS ZEROED SINCE LAST FSTART
		MASK	OPTMODES
		CCS	A
		TC	+3
		TC	ALARM			# OPTICS NOT ZEROED
		OCT	00120			; Alarm: Optics require zero calibration

; Check if error counters are enabled in channel 12
; Error counters track optics CDU pulse commands
; BIT2 must be set to enable shaft/trunnion pulse counting
		CA	BIT2			# SEE IF ERR CNTS ENABLED
		EXTEND
		RAND	CHAN12
		EXTEND
		BZF	SETBIT			# CNTS NOT ENABLED-DO IT AND RESUME

; Begin processing both optics axes: shaft (OPTIND=1) and trunnion (OPTIND=0)
; OPTIND acts as axis selector in indexed addressing throughout drive logic
		CAF	ONE			# INITIALIZE OPTIND
# Page 167
; Main optics command processing loop
; OPTIND=1: Process shaft axis, OPTIND=0: Process trunnion axis
; Loop processes shaft first, then trunnion
OPT2		TS	OPTIND
		EXTEND
		BZF	TRUNCMD			# CHECK TRUNION COMMAND

; Compute angular error for current axis:
; DESOPTT/DESOPTS contains desired angle from navigation program
; CDUT/CDUS contains current CDU angle from sextant
; Error = Desired - Actual
GETOPCMD	INDEX	OPTIND
		CA	DESOPTT			# PICK UP DESIRED OPT ANGLE
		EXTEND
		INDEX	OPTIND
		MSU	CDUT			# GET DIFFERENCE
		
; Scale command by BIT13 (1/8192) for pulse count conversion
; Double precision arithmetic handles full angular range
		EXTEND
		MP	BIT13
		XCH	L
		DOUBLE
		TS	ITEMP1
		TCF	+2			# NO OVFL

		ADS	L			# WITH OVFL
		
; Store computed command for this axis
; COMMANDO+1 = shaft command, COMMANDO = trunnion command
STORCMD		INDEX	OPTIND
		LXCH	COMMANDO		# STORE COMMAND
		CCS	OPTIND
		TCF	OPT2			# GET NEXT COMMAND

		TS	ITEMP1			# INITIALIZE SEND INDICATOR TO ZERO
		COUNT*	$$/SXT

# ============================================================================
# SHAFT STOP AVOIDANCE
#
# Prevent commanding the sextant shaft into mechanical stops at ±90 degrees.
# If shaft is beyond 90° and ZONE indicates stop danger, zero the shaft
# command if it would drive further into the stop. Critical safety feature
# preventing hardware damage during automated optical tracking.
# ============================================================================

; First check: Is shaft beyond ±90° from zero?
; If shaft is within ±90°, no stop danger exists
		CCS	CDUS			# IF CDUS GREATER THAN + OR - 90 DEG CHECK
		AD	NEG1/2			# FOR POSSIBLE STOP PROBLEM
		TCF	+2
		TCF	-2
		EXTEND
		BZMF	CMDSETUP		# CDU LESS THAN 90 DEG, NO PROBLEMS

; Shaft is beyond 90° - check if ZONE indicates stop danger
; ZONE=0: Safe central region, allow commands
; ZONE=+/-: Near positive/negative stop, check command direction
		CA	ZONE
		EXTEND
		BZF	CMDSETUP		# ZONE=3, NORMAL COMMAND
		
; Compare signs of ZONE and shaft COMMAND
; If signs differ, command moves away from stop (safe)
; If signs match, command moves toward stop (danger - must zero command)
		MASK	BIT15			# GRAB SIGN OF ZONE
		TS	L
		CA	COMMANDO +1		; Shaft command
		MASK	BIT15			# GRAB SIGN OF SHAFT COMMAND
		EXTEND
		RXOR	LCHAN			; XOR signs
		CCS	A
		TCF	CMDSETUP		# SIGN ZONE NOT EQUAL TO SIGN COMMAND
		
; Signs match - command moves toward stop
; Final check: Is desired position within safe ±90° region?
; If so, allow command (moving toward safe region)
; If not, zero command (would drive into stop)
		CCS	DESOPTS			# SEE IF DESOPTS BETWEEN -90 AND +90
		AD	NEG1/2
		TCF	+2			# ABS(DESOPTS) - 90 DEG
		TCF	-2
		EXTEND
# Page 168
		BZMF	+2			# DESOPTS IN FIRST OR FOURTH QUAD
		TCF	CMDSETUP		; Desired position safe - allow command
		
; Desired position is beyond ±90° AND command moves toward stop
; Protection action: Zero the shaft command to prevent stop collision
		CS	COMMANDO +1		# REVERSE REGULAR COMMAND
		TS	COMMANDO +1		; Command now zeroed

		COUNT*	$$/T4RPT

; ============================================================================
; COMMAND SETUP AND DISPATCH
; 
; Process both axis commands (shaft and trunnion) starting with trunnion.
; Commands are checked for sign and magnitude, limited to MAXPLS if needed.
; Zero commands skip pulse transmission to save processing time.
; ============================================================================

CMDSETUP	CAF	ONE			# SET OPTIND
		TS	OPTIND			; Start with trunnion (OPTIND=1)
		INDEX	A			; Use OPTIND to select axis
		CCS	COMMANDO		# GET SIGN OF COMMAND
		TC	POSOPCMD		; Positive command - process magnitude
		TC	NEXTOPT	+1		# ZERO COMMAND-SKIP SEND INDICATOR
		TC	NEGOPCMD		; Negative command - process magnitude
		TC	NEXTOPT	+1		# ZERO COMMAND

; Special handling for large trunnion commands
; If error exceeds 45°, limit command to POSMAX to prevent excessive slew rate
; This entry point is reached from OPTDRIVE when OPTIND=0 (trunnion axis)
TRUNCMD		CS	CDUT			# IF COMMAND GREATER THAN 45 DEG-COMMAND
		AD	DESOPTT			# 45 DEG
		TS	Q			; Save error for comparison
		TC	GETOPCMD		# LESS THAN 45 DEG-NORMAL OPERATION

; Error exceeds 45° - use maximum command with appropriate sign
		CCS	A			# GREATER THAN 45 DEG-USE OPSMAX WITH
		CA	POSMAX			# CORRECT SIGN
		TC	+2			; Positive large error
		CS	POSMAX			; Negative large error
		TS	L
		TC	STORCMD			; Store limited command
; Positive command processing
; Entry: A contains positive command magnitude (from CCS COMMANDO)
; Limits command to MAXPLS if necessary to prevent pulse counter overflow
POSOPCMD	AD	MAXPLS1			; Test against maximum pulse limit
		EXTEND
		BZMF	DELOPCMD		# COMMAND LESS THAN MAX PULSE
		CS	MAXPLS			# GREATER THAN MAX PULSE-USE MAX PULSE

; Store command and proceed to next axis
; ITEMP1 tracks whether any non-zero commands exist (send indicator)
; Process both trunnion and shaft (OPTIND cycles 1→0)
NEXTOPT		INCR	ITEMP1			# SET SEND INDICATOR
		AD	NEG0			# MAKE SURE ZERO COMMAND IS -ZERO
		INDEX	OPTIND			; Select axis (1=trunnion, 0=shaft)
		TS	CDUTCMD			# STORE PULSE IN SEND REG

		CCS	OPTIND			; Check which axis just processed
		TC	CMDSETUP +1		# GET NEXT OPT (was trunnion, do shaft)

; Both axes processed - check if any commands need transmission
		CCS	ITEMP1			# ARE ANY PULSES TO GO
		TCF	SENDOCMD		# YES-SEND EM
		TC	RESUME			# NO (all zero commands)

; Negative command processing  
; Entry: A contains negative command magnitude (from CCS COMMANDO)
; Limits command magnitude to MAXPLS if necessary
NEGOPCMD	AD	MAXPLS1			; Test magnitude against limit
		EXTEND
		BZMF	DELOPCMD		# LESS THAN MAX PULSE
		CA	MAXPLS			# MAX PULSES (magnitude limited)
		TCF	NEXTOPT			; Store and continue
# Page 169
; Small command handling (within MAXPLS limit)
; Command magnitude is acceptable - use as-is
DELOPCMD	INDEX	OPTIND
		XCH	COMMANDO		# SET UP SMALL COMMAND
		TCF	NEXTOPT			; Store and continue

; Transmit optics commands to hardware
; Sets bits 11 and 12 in channel 14 to enable OCDU drive
SENDOCMD	CAF	11,12			# SEND OCDU DRIVE COMMANDS
		EXTEND
		WOR	CHAN14			; Write-OR to channel 14
		TC	RESUME			; Return to interrupted program

; Enable OCDU error counters
; Called to activate optical unit CDU error counting for next control cycle
SETBIT		CAF	BIT2			# ENABLE OCDU ERR CNTS
		EXTEND
		WOR	CHAN12			; Write-OR bit 2 to channel 12
		TC	RESUME			# START COARS NEXT TIME AROUND

; Optics command limits
; Maximum pulses per control cycle to prevent hardware overflow
; Values increased from original -80/-79 to -165/-164
MAXPLS		DEC	-165			# WAS -80
MAXPLS1		DEC	-164			# WAS -79
11,12		EQUALS	PRIO6			; Channel 14 bits for OCDU drive enable

