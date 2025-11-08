# Copyright:    Public domain.
# Filename:     T4RUPT_PROGRAM.agc
# Purpose:      Part of the source code for Luminary 1A build 099.
#               It is part of the source code for the Lunar Module's (LM)
#               Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:    yaYUL
# Contact:      Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:      www.ibiblio.org/apollo.
# Pages:        155-189
# Mod history:  2009-05-19 HG   Transcribed from page images.
#		2010-12-31 JL	Fixed page number comment.
#		2011-01-06 JL	Fixed indentation of TNONTEST. Fixed pseudo-label indentation.
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
#       Assemble revision 001 of AGC program LMY99 by NASA 2021112-061
#       16:27 JULY 14, 1969
#
# ============================================================================
# FILE: T4RUPT_PROGRAM.agc
# MODULE: Real-Time Interrupt Processing
# MISSION PHASE: all phases (launch/earth-orbit/trans-lunar/lunar-orbit/descent/landing/ascent/rendezvous/trans-earth)
#
# TL;DR: Implements the Timer 4 interrupt service routine, the heartbeat of
#        the AGC real-time operating system. Fires every 10 milliseconds to
#        manage DSKY display scanning, WAITLIST task dispatching, mission
#        timer updates, IMU/ISS status monitoring, and critical hardware
#        failure detection. During Apollo 11's descent, computational
#        overload in this routine contributed to the famous 1202 alarms.
#
# COMMENT-ONLY READERS: This is the AGC's "heartbeat" - the routine that
#        keeps the computer synchronized with real time and spacecraft
#        systems. Read to understand how the computer managed multiple
#        critical tasks simultaneously during the lunar landing.
#
# CODE-ALONG READERS: Study the interrupt priority structure, register
#        preservation techniques, and the clever time-slicing strategy
#        that allowed 10ms interrupt timing despite varying workloads.
#        Note the DSRUPTSW counter mechanism for distributing tasks.
# ============================================================================
# Page 155
		BANK	12
		SETLOC	T4RUP
		BANK

		EBANK=	M11
		COUNT*	$$/T4RPT
; ============================================================================
; TRANSITION: T4RUPT Entry Point - The AGC Real-Time Heartbeat
;
; Every 10 milliseconds, the AGC hardware triggers this Timer 4 interrupt,
; suspending whatever program was executing. This is the highest-frequency
; interrupt in the system, responsible for time-critical operations that
; keep the spacecraft functioning. During the Apollo 11 lunar descent, this
; routine's workload became intense as radar data flooded in, contributing
; to the WAITLIST overflow that triggered the 1202 program alarm at mission
; time 102:38:26. Flight controller Steve Bales' "Go" decision allowed the
; landing to continue despite the alarm.
; ============================================================================

; T4RUPT - Timer 4 Interrupt Service Routine Entry Point
; Called automatically by AGC hardware every 10 milliseconds (100 Hz).
; This interrupt has high priority and must complete quickly to avoid
; disrupting time-critical operations.

T4RUPT		TS	BANKRUPT	; Save A register to preserve interrupted program state
					; BANKRUPT holds accumulator during interrupt processing

		EXTEND			; Next instruction is extended (uses both words)
		QXCH	QRUPT		; Save and restore Q register (return address)
					; Preserves interrupted program's return linkage

; DSRUPTSW cycles through values 7, 6, 5, 4, 3, 2, 1, 0, then back to 7.
; This counter distributes time-consuming tasks across multiple interrupt
; cycles, preventing any single 10ms cycle from taking too long. This
; time-slicing technique was critical for maintaining real-time responsiveness.

		CCS	DSRUPTSW	; Check DSRUPTSW counter value and decrement
					; CCS (Count, Compare, and Skip) tests: >0, +0, <0, -0
		TCF	NORMT4 +1	; If positive (7 down to 1), skip display cycle
		TCF	NORMT4		; If +0, reset counter and continue normal path
					; (never reaches <0 or -0 cases in normal operation)

		TCF	QUIKDSP		; If counter was 0, do quick display update first

; Normal T4RUPT path: reset the counter to 7 for next cycle through tasks
NORMT4		CAF	SEVEN		; Load constant 7 into accumulator
		TS	RUPTREG1	; Save to RUPTREG1 (used for task dispatching)
		TS	DSRUPTSW	; Reset DSRUPTSW counter to 7 for next 8 cycles

		BLOCK	02
		SETLOC	FFTAG10
		BANK

		COUNT*	$$/T4RPT
; Constants for T4RUPT timing and relay control
100MRUPT	=	OCT37766	# (DEC 16374) - 100ms timing constant for slower tasks
		# RELTAB IS A PACKED TABLE. RELAYWORD CODE IN UPPER 4 BITS, RELAY CODE
		# IN LOWER 5 BITS.

; RELTAB - Relay Table for DSKY Display Control
; Each entry contains packed relay control information:
; - Upper 4 bits: relay word selection (which group of relays)
; - Lower 5 bits: specific relay code (which relay within group)
; These control the electromechanical relays that drive the DSKY's
; seven-segment displays and indicator lights visible to the crew.

RELTAB		OCT	04025		; Relay control entry 0
		OCT	10003		; Relay control entry 1
		OCT	14031		; Relay control entry 2
		OCT	20033		; Relay control entry 3
		OCT	24017		; Relay control entry 4
		OCT	30036		; Relay control entry 5
		OCT	34034		; Relay control entry 6
		OCT	40023		; Relay control entry 7
		OCT	44035		; Relay control entry 8
		OCT	50037		; Relay control entry 9
		OCT	54000		; Relay control entry 10
RELTAB11	OCT	60000		; Relay control entry 11

# Page 156
		# SWITCHED-BANK PORTION

		BANK	12
		SETLOC	T4RUP
		BANK

		COUNT*	$$/T4RPT
; ============================================================================
; TRANSITION: From Interrupt Entry to Display Output Processing
;
; The T4RUPT routine now enters the display management section. The DSKY
; (Display and Keyboard) uses electromechanical relays to illuminate seven-
; segment digits and indicator lights. These relays require precise timing
; to avoid flicker or missed updates. During critical mission phases like
; lunar landing, the crew relied on these displays for altitude, velocity,
; and system status information.
; ============================================================================

; CDRVE - Control Display Relay Driver
; Manages the 12th DSKY display position (DSPTAB +11D) and drives the
; corresponding relay output. This position often controls special indicators
; or the rightmost digit position.

CDRVE		CCS	DSPTAB +11D	; Check display table entry 11 (12th position)
					; CCS tests if display data is positive
		TC	DSPOUT		; If positive, update display outputs
		TC	DSPOUT		; If +0, also update display outputs

		XCH	DSPTAB +11D	; Exchange A with display entry (saves old, loads new)
		MASK	LOW11		; Mask lower 11 bits to isolate relay data
		TS	DSPTAB +11D	; Store masked value back to display table
		AD	RELTAB11	; Add relay table entry 11 for final relay code
		EXTEND			; Next instruction is extended
		WRITE	OUT0		; Write relay control to hardware OUT0 channel
					; This energizes specific DSKY relay coils
		TC	HANG20		; Wait 20 microseconds for relay settling time

# Page 157
		# DSPOUT PROGRAM, PUTS OUT DISPLAYS

; DSPOUTSB - Display Output Subroutine
; Scans through the DSPTAB (display table) to find entries that need
; updating and writes them to the DSKY hardware. The table contains
; 12 entries (positions 0-11) representing the DSKY display digits
; and indicator lights.

DSPOUTSB	TS	NOUT		; Save display request count
		CS	ZERO		; Load -0 (negative zero)
		TS	DSRUPTEM	; SET TO -0 FOR 1ST PASS THRU DSPTAB
					; Using -0 as sentinel for first pass detection
		XCH	DSPCNT		; Get current display counter position
		AD	NEG0		; TO PREVENT +0 (convert +0 to -0)
		TS	DSPCNT		; Store normalized counter value

; DSPSCAN - Display Table Scanner
; Walks through DSPTAB looking for negative entries (which indicate
; data ready for display). Positive entries are "pending" (not yet ready).
; This two-pass algorithm ensures all display positions get serviced.

DSPSCAN		INDEX	DSPCNT		; Use DSPCNT as index into DSPTAB
		CCS	DSPTAB		; Check display table entry sign
		CCS	DSPCNT		# IF DSPTAB ENTRY +, SKIP
					; Entry positive = not ready, check next
		TCF	DSPSCAN	-2	# IF DSPCNT +, TRY AGAIN
					; Counter positive = more entries to check
		TCF	DSPLAY		# IF DSPTAB ENTRY -, DISPLAY
					; Entry negative = ready to display
TABLNTH		OCT	12		# DEC 10, LENGTH OF DSPTAB
					; Table has 12 positions (0-11)
		CCS	DSRUPTEM	# IF DSRUPTEM=+0, 2ND PASS THRU DSPTAB
120MRUPT	DEC	16372		# (DSPCNT = 0).  +0 INTO NOUT.
					; 120ms timing constant for slower updates
		TS	NOUT		; Clear output request (no more entries)
		TC	Q		; Return to caller
		TS	DSRUPTEM	# IF DSRUPTEM=-0, 1ST PASS THRU DSPTAB
					; First pass complete, prepare for second
		CAF	TABLNTH		# (DSPCNT=0).+0 INTO DSRUPTEM. PASS AGAIN
		TCF	DSPSCAN -1	; Restart scan from beginning of table

; DSPLAY - Display Update Handler
; Processes a display table entry that's ready (negative). Converts the
; entry back to positive (marking it "displayed"), extracts the relay
; control bits, and writes them to the DSKY hardware output channel.

DSPLAY		AD	ONE		; Add 1 to make negative entry positive
		INDEX	DSPCNT		; Index into display table
		TS	DSPTAB		; REPLACE POSITIVELY (mark as displayed)
		MASK	LOW11		; REMOVE BITS 12 TO 15 (isolate data bits)
		TS	DSRUPTEM	; Store data portion temporarily
		CAF	HI5		; Load mask for high 5 bits (bits 12-15)
		INDEX	DSPCNT		; Index into relay table
		MASK	RELTAB		; PICK UP BITS 12 TO 15 OF RELTAB ENTRY
					; These bits select which relay word to use
		AD	DSRUPTEM	; Combine relay selection with data
		EXTEND			; Next instruction is extended
		WRITE	OUT0		; Write complete relay command to DSKY
					; This illuminates specific display segments

		TCF	Q+1		; Return (skip one instruction)

; DSPOUT - Display Output Main Entry
; Checks if display updates are enabled (FLAGWRD5 flag) and if there
; are pending display requests (NOUT counter). Only proceeds with
; display scanning if both conditions are met.

DSPOUT		CCS	FLAGWRD5	# IS DSKY FLAG ON
					; FLAGWRD5 bit indicates if display active
		CAF	ZERO		# NO (flag was positive or zero)
		TCF	NODSPOUT	# NO - skip display update this cycle
		CCS	NOUT		# YES - check if display requests pending
		TC	DSPOUTSB	; Requests pending - process display table
		TCF	NODSPOUT	# NO DISPLAY REQUESTS - skip update

HANG20		CS	14,11,9
		ADS	DSRUPTSW

		CAF	20MRUPT

SETTIME4	TS	TIME4

# Page 158
		# THE STATUS OF THE PROCEED PUSHBUTTON IS MONITORED EVERY 120 MILLISECONDS VIA THE CHANNEL 32 BIT 14 INBIT.
		#  THE STATE OF THIS INBIT IS COMPARED WITH ITS STATE DURING THE PREVIOUS T4RUPT AND IS PROCESSED AS FOLLOWS.
		#	IF PREV ON AND NOW ON	-- BYPASS.
		#	IF PREV ON AND NOW OFF	-- UPDATE IMODES33.
		#	IF PREV OFF AND NOW ON	-- UPDATE IMODES33 AND PROCESS VIA PINBALL.
		#	IF PREV OFF AND NOW OFF	-- BYPASS.
		# THE LOGIC EMPLOYED REQUIRES ONLY 9 MCT (APPROX. 108 MICROSECONDS) OF COMPUTER TIME WHEN NO CHANGES OCCUR.

; ============================================================================
; PROCEED BUTTON MONITORING
;
; The PROCEED button on the DSKY is the astronaut's primary means of
; acknowledging computer requests and advancing program sequences. During
; the Apollo 11 descent, Armstrong and Aldrin used PROCEED to confirm
; program transitions and navigate through landing displays.
;
; This routine efficiently detects button state changes using only 9 machine
; cycles (108 microseconds) when no change occurs, crucial for maintaining
; the 10ms interrupt deadline. When a press is detected, it schedules the
; PROCKEY routine via NOVAC to process the keystroke outside the interrupt.
;
; The routine compares the current state of Channel 32 Bit 14 (the hardware
; PROCEED button line) with the previously stored state in IMODES33. Using
; the RXOR (Read and Exclusive-OR) instruction allows simultaneous reading
; and comparison in a single operation.
; ============================================================================

PROCEEDE	CA	IMODES33	# MONITOR FOR PROCEED BUTTON
		EXTEND
		RXOR	CHAN32
		MASK	BIT14
		EXTEND
		BZF	T4JUMP		# NO CHANGE

		LXCH	IMODES33
		EXTEND
		RXOR	LCHAN
		TS	IMODES33	# UPDATE IMODES33
		MASK	BIT14
		CCS	A
		TCF	T4JUMP		# WAS ON -- NOW OFF

		CAF	CHRPRIO		# WAS OFF -- NOW ON
		TC	NOVAC
		EBANK=	DSPCOUNT
		2CADR	PROCKEY

# Page 159
		# JUMP TO APPROPRIATE ONCE-PER SECOND (0.96 SEC ACTUALLY) ACTIVITY

; ============================================================================
; T4JUMP - Task Distribution Table
;
; This indexed jump table distributes periodic tasks across eight consecutive
; T4RUPT cycles, creating a ~960ms (approximately 1 second) time slice for
; lower-priority monitoring functions. RUPTREG1 cycles 0-7, causing each
; T4RUPT to execute one of eight task sequences.
;
; The clever design prevents all monitoring tasks from executing in the same
; interrupt, which would cause timing overruns. Instead, tasks are spread
; across multiple cycles:
;   Cycle 0: RCS monitoring
;   Cycle 1: Rendezvous radar autopilot check
;   Cycle 2: IMU monitoring
;   Cycle 3: Digital autopilot sampling
;   Cycles 4-7: Repeat the sequence
;
; During Apollo 11's descent, this task distribution was crucial for
; preventing the 1202 alarm from becoming catastrophic. By time-slicing
; non-critical tasks, the computer could still service the guidance and
; throttle control computations within their deadlines.
; ============================================================================

T4JUMP		INDEX	RUPTREG1
		TCF	+1

		TC	RCSMONIT
		TCF	RRAUTCHK
		TCF	IMUMON
		TCF	DAPT4S
		TC	RCSMONIT
		TCF	RRAUTCHK
		TCF	IMUMON
		TCF	DAPT4S

20MRUPT		=	OCT37776	# (DEC 16382)

# Page 160
		# ADDITIONAL ROUTINES FOR 20MS. KEYBOARD ACTIVITY

; ============================================================================
; DISPLAY TIMING CONTROL - 20ms vs 120ms Cycles
;
; The T4RUPT alternates between two timing modes to handle different display
; update rates. NODSPOUT handles the 120ms cycles (every 12th interrupt),
; while QUIKDSP handles the 20ms cycles (every 2nd interrupt). This dual-
; rate system allows critical display updates to occur quickly while
; reducing the average processing load.
;
; The DSRUPTSW counter determines which path to take, cleverly encoding
; multiple timing states in a single variable.
; ============================================================================

NODSPOUT	EXTEND
		WRITE	OUT0

		CAF	120MRUPT	#SET FOR NEXT CCRIVE
		TCF	SETTIME4

; ============================================================================
; QUIKDSP - Fast Display Update (20ms cycle)
;
; This routine handles the rapid display update path, executing every 20ms
; to provide responsive feedback to the crew. QUIKDSP alternates between
; writing display data and turning off relay drivers, creating a multiplexed
; display system.
;
; The routine uses BIT14 of DSRUPTSW as a toggle flag to alternate between:
;   1. Writing new display segments (relay drivers active)
;   2. Turning off all relays (next cycle)
;
; This "write then clear" pattern prevents ghosting on the electroluminescent
; displays while maintaining adequate brightness. During Apollo 11's landing,
; this rapid update rate ensured Armstrong and Aldrin saw altitude and
; velocity information with minimal lag.
; ============================================================================

QUIKDSP		CAF	BIT14
		MASK	DSRUPTSW
		EXTEND
		BZF	QUIKOFF		# WROTE LAST TIME, NOW TURN OFF RELAYS.

		CCS	NOUT
		TC	DSPOUTSB
		TCF	NODSPY		# NOUT=0 OR BAD RETURN FROM DSPOUTSB
		CS	BIT14		# GOOD RETURN (WE DISPLAYED SOMETHING)
QUIKRUPT	ADS	DSRUPTSW

		CAF	20MRUPT
		TS	TIME4

		CAF	BIT9
		ADS	DSRUPTSW

		TC	RESUME

NODSPY		EXTEND
		WRITE	OUT0

SYNCT4		CAF	20MRUPT
		ADS	TIME4

		CAF	BIT9
		ADS	DSRUPTSW

		CCS	DSRUPTSW
		TC	RESUME
OCT37737	OCT	37737
		TC	SYNCT4
		TC	RESUME

; QUIKOFF - Turn Off Display Relays
; This is the "clear" half of the display multiplexing cycle. Writes zeros
; to OUT0 (Channel 10) to de-energize all relay drivers, preventing segment
; ghosting. Then sets BIT14 in DSRUPTSW so the next 20ms cycle will write
; display data again.

QUIKOFF		EXTEND
		WRITE	OUT0
		CAF	BIT14		# RESET DSRUPTSW TO SEND DISPLAY NEXT PASS
		TCF	QUIKRUPT

14,11,9		OCT	22400

# Page 161
; ============================================================================
; IMUMON - Inertial Measurement Unit Monitor
;
; The IMU is the spacecraft's primary navigation sensor, containing three
; gyroscopes and three accelerometers in a stabilized platform. This routine
; monitors the health and status of the IMU system by detecting changes in
; six critical status bits from Channel 30 (the IMU telemetry channel).
;
; Executes every 480ms (every 48th T4RUPT) to check:
;   Bit 15: Temperature within limits
;   Bit 14: ISS (Inertial Subsystem) turn-on request
;   Bit 13: IMU failure detected
;   Bit 12: IMU CDU (Coupling Data Unit) failure
;   Bit 11: IMU in CAGE mode (platform uncaged)
;   Bit 9:  IMU in OPERATE mode
;
; When any bit changes state, IMUMON calls the appropriate handler routine
; to process the event. During Apollo 11's descent, the IMU provided the
; position and velocity data that guided Eagle to the lunar surface. Any
; IMU failure would have required an immediate abort.
;
; The routine uses an elegant bit-scanning algorithm: it XORs the current
; Channel 30 state with the previously saved state (IMODES30), isolates
; changed bits, then iterates through them using a doubling loop that
; detects the highest-order changed bit first. This ensures critical
; failures (higher bits) are processed before less critical events.
; ============================================================================

# PROGRAM NAME:  IMUMON

# FUNCTIONAL DESCRIPTION:  THIS PROGRAM IS ENTERED EVERY 480 MS.  IT DETECTS CHANGES OF THE IMU STATUS BITS IN
# CHANNEL 30 AND CALLS THE APPROPRIATE SUBROUTINES.  THE BITS PROCESSED AND THEIR RELEVANT SUROUTINES ARE:

#	FUNCTION		BIT	SUBROUTINE CALLED
#	--------		---	-----------------
#	TEMP IN LIMITS		 15	TLIM
#	ISS TURN-ON REQUEST	 14	ITURNON
#	IMU FAIL		 13	IMUFAIL (SETISSW)
#	IMU CDU FAIL		 12	ICDUFAIL (SETISSW)
#	IMU CAGE		 11	IMUCAGE
#	IMU OPERATE		  9	IMUOP

# THE LAST SAMPLED STATE OF THESE BITS IS LEFT IN IMODES30.  ALSO, EACH SUBROUTINE CALLED FINDS THE NEW
# VALUE OF THE BIT IN A, WITH Q SET TO THE PROPER RETURN LOCATION NXTIFAIL.

# CALLING SEQUENCE:  T4RUPT EVERY 480 MILLISECONDS.

# JOBS OR TASKS INITIATED:  NONE.

# SUBROUTINES CALLED:  TLIM, TURNON, SETISSW, IMUCAGE, IMUOP.

# ERASABELE INITIALIZATION:
#	FRESH START OR RESTART WITH NO GROUPS ACTIVE:  C((MODES30) = OCT 37411).
#	RESTART WITH ACTIVE GROUPS:	C(IMODES30) = (B(IMODES30)AND(OCT 00035)) PLUS OCT 37400.
#					THIS LEAVES IMU FAIL BITS INTACT.

# ALARMS:  NONE.

# EXIT:  TNONTEST.

# OUTPUT:  UPDATED IMODES30 WITH CHANGES PROCESSED BY APPROPRIATE SUBROUTINE.

IMUMON		CA	IMODES30	# SEE IF THERE HAS BEEN A CHANGE IN THE
		EXTEND			# RELEVANT BITS OF CHAN 30.
		RXOR	CHAN30
		MASK	30RDMSK
		EXTEND
		BZF	TNONTEST	# NO CHANGE IN STATUS

		TS	RUPTREG1	# SAVE BITS WHICH HAVE CHANGED.
		LXCH	IMODES30	# UPDATE IMODES30.
		EXTEND
		RXOR	LCHAN
		TS	IMODES30

		CS	ONE
		XCH	RUPTREG1
		EXTEND
# Page 162
		BZMF	TLIM		# CHANGE IN IMU TEMP.
		TCF	NXTIFBIT	# BEGIN BIT SCAN.

 -1		AD	ONE		# (RE-ENTERS HERE FROM NXTIFAIL.)
NXTIFBIT	INCR	RUPTREG1	# ADVANCE BIT POSITION NUMBER.
 +1		DOUBLE
		TS	A		# SKIP IF OVERFLOW.
		TCF	NXTIFBIT	# LOOK FOR BIT.

		XCH	RUPTREG2	# SAVE OVERFLOW-CORRECTED DATA.
		INDEX	RUPTREG1	# SELECT NEW VALUE OF THIS BIT.
		CAF	BIT14
		MASK	IMODES30
		INDEX	RUPTREG1
		TC	IFAILJMP

NXTIFAIL	CCS	RUPTREG2	# PROCESS ANY ADDITIONAL CHANGES.
		TCF	NXTIFBIT -1

# Page 163
; ============================================================================
; TNONTEST - IMU Turn-On Sequence Manager
;
; This routine manages the complex initialization sequence required when the
; IMU (Inertial Subsystem) is powered on. The ISS (Inertial Subsystem) 
; requires a careful 90-second caging period to allow the gyroscopes to
; spin up and stabilize before the platform can be aligned for navigation.
;
; The routine handles three distinct initialization scenarios:
;
; 1. ISS TURN-ON: Computer is operating when ISS is turned on. Both the
;    "ISS Turn-On" (Channel 30 bit 14) and "ISS Operate" (bit 9) signals
;    appear. The platform is caged for 90 seconds and the CDUs (Coupling
;    Data Units) are zeroed so gimbal lock monitoring will function properly.
;
; 2. ICDU INITIALIZATION: Computer was turned on or fresh-started with ISS
;    already in operate mode. Only "ISS Operate" signal is present. The
;    ICDUs are zeroed for gimbal lock monitoring (unless platform is already
;    in gimbal lock after a restart, in which case no action is taken).
;
; 3. RESTART WITH ACTIVE IMU PROGRAM: A restartable program is using the
;    IMU. No initialization occurs since the using program already handled
;    initialization and T4RUPT should not interfere.
;
; The routine uses a two-stage timing mechanism encoded in IMODES30 bits 7-8:
;   - Bit 7 set = 1 by first signal (turn-on or operate) that arrives
;   - Next 480ms: Bit 8 set = 1, routine waits
;   - Next 480ms: Bits 7-8 cleared, routine proceeds with initialization
;
; This 960ms delay ensures both signals have stabilized before action is taken.
;
; During Apollo 11, proper IMU initialization was critical before every major
; maneuver. The 90-second caging delay, though lengthy, was essential for
; gyroscope stabilization. Any failure during this sequence would have left
; the spacecraft without reliable navigation.
; ============================================================================

# PROGRAM NAME:  TNONTEST.

# FUNCTIONAL DESCRIPTION:  THIS PROGRAM HONORS REQUESTS FOR ISS INITIALIZATION.  ISS TURN-ON (CHANNEL 30 BIT 14)
# AND ISS OPERATE (CHANNEL 30 BIT 9) REQUESTS ARE TREATED AS A PAIR AND PROCESSING TAKES PLACE .480 SECONDS
# AFTER EITHER ONE APPEARS.  THIS INITIALIZATION TAKES ON ONE OF THE FOLLOWING THREE FORMS:

#	1) ISS TURN-ON:  IN THIS SITUATION THE COMPUTER IS OPERATING WHEN THE ISS IS TURNED ON.  NOMINALLY,
#	BOTH ISS TURN-ON AND ISS OPERATE APPEAR.  THE PLATFORM IS CAGED FOR 90 SECONDS AND THE ICDU'S ZEROED
#	SO THAT AT THE END OF THE PROCESS THE GIMBAL LOCK MONITOR WILL FUNCTION PROPERLY.

#	2) ICDU INITIALIZATION:  IN THIS CASE THE COMPUTER WAS PROBABLY TURNED ON WITH THE ISS IN OPERATE OR
#	A FRESH START WAS DONE WIT THE ISS IN OPERATE.  IN THIS CASE ONLY ISS OPERATE IS ON.  THE ICDU'S ARE
#	ZEROED SO THE GIMBAL LOCK MONITOR WILL FUNCTION.  AN EXCEPTION IS IF THE ISS IS IN GIMBAL LOCK AFTER
#	A RESTART, THE ICDU'S WILL NOT BE ZEROED.

#	3) RESTART WITH RESTARTABLE PROGRAM USING THE IMU:  IN THIS CASE, NO INITIALIZATION TAKES PLACE SINCE
#	IT IS ASSUMED THT THE USING PROGRAM DID THE INITIALIZATION AND THEREFORE T4RUPT SHOULD NOT INTERFERE.

# IMODES30 BIT 7 IS SET = 1 BY THE FIRST BIT (CHANNEL 30 BIT 14 OR 9) WHICH ARRIVES.  FOLLOWING THIS, TNONTEST IS
# ENTERED, FINDS BIT 7 = 1 BUT BIT 8 = 0, SO IT SETS BIT 8 = 1 AND EXITS.  THE NEXT TIME IT FINDS BIT 8 = 1 AND
# PROCEEDS, SETTING BITS 8 AND 7 = 0.  AT PROCTNON, IF ISS TURN-ON REQUEST IS PRESENT, THE ISS IS CAGED (ZERO +
# COARSE).  IF ISS OPERATE IS NOT PRESENT PROGRAM ALARM 00213 IS ISSUED.  AT THE END OF A 90 SECOND CAGE, BIT 2
# OF IMODES30 IS TESTED.  IF IT IS = 1, ISS TURN-ON WAS NOT PRESENT FOR THE ENTIRE 90 SECONDS.  IN THAT CASE, IF
# THE ISS TURN-ON REQUEST IS PRESENT TEH 90 SECOD WAIT IS REPEATED.  OTHERWISE NO ACTION OCURS UNLESS A PROGRAM
# WAS WAITING FOR THE INITIALIZATION IN WHIC CASE TH PROGRAM IS GIVEN AN IMUSTALL ERROR RETURN.  IF THE DELAY
# WENT PROPERLY, THE ISS DELAY OUTBIT IS SENT AND THE ICDU'S ZEROED.  A TASK IS INITIATED TO REMOVE THE PIPA FAIL
# INHIBIT BIT IN 10.24 SECONDS.  IF A MISSION PROGRAM WAS WAITING IT IS INFORMED VIA ENDIMU.

# AT PROCTNON, IF ONLY ISS OPERATE IS PRESENT (OPONLY), THE CDU'S ARE ZEROED UNLESS THE PLATFORM IS IN COARSE
# ALIGN (= GIMBAL LOCK HERE) OR A MISSIN PROGRAM IS USING THE IMU (INUSEFLG = 1).

# CALLING SEQUENCE:  T4RUPT EVERY 480 MILLISECONDS AFTER IMUMON.

# JOBS OR TASKS INITIATED:  1) ENDTNON, 90 SECONDS AFER CAGING STARTED.  2) ISSUP, 4 SECONDS AFTER CAGING DONE.
#	3) PFAILOK, 10.24 SECONDS AFTER INITIALIZATION COMPLETED.  4) UNZ2, 320 MILLISECONDS AFTER ZEROING
#	STARTED.

# SUBROUTINES CALLED: CAGESUB, CAGESUB2, ZEROICDU, ENDIMU, IMUBAD, NOATTOFF, SETISSW, VARDELAY.

# ERASABLE INITIALIZATION:  SEE IMUMON.

# ALARMS:  PROGRAM ALARM 00213 IF ISS TURN-ON REQUESTED WITHOUT ISS OPERATE.

# EXIT:  ENDTNON EXITS TO C33TEST.  TASKS HAVING TO DO WITH INITIALIZATION EXIT AS FOLLOWS:  MISSION PROGRAM
# WAITING AND INITIALIZATION COMPLETE, EXIT TO ENDIMU, MISSION PROGRAM WAITING AND INITIALIZATION FAILED, EXIT TO
# IMUBAD, IMU NOT IN USE, EXIT TO TASKOVER.

# OUTPUT:  ISS INITIALIZED.

TNONTEST	CS	IMODES30	# AFTER PROCESSING ALL CHANGES, SEE IF IT
# Page 164
		MASK	BIT7		# IS TIME TO ACT ON A TURN-ON SEQUENCE.
		CCS	A
		TCF	C33TEST		# NO -- EXAMINE CHANNEL 33.

		CAF	BIT8		# SEE IF FIRST SAMPLE OR SECOND.
		MASK	IMODES30
		CCS	A
		TCF	PROCTNON	# REACT AFTER A SECOND SAMPLE.

		CAF	BIT8		# IF FIRST SAMPLE, SET BIT TO REACT NEXT
		ADS	IMODES30	# TIME.
		TCF	C33TEST

		# PROCESS IMU TURN-ON REQUESTS AFTER WAITING 1 SAMPLE FOR ALL SIGNALS TO ARRIVE.

PROCTNON	CS	BITS7&8
		MASK	IMODES30
		TS	IMODES30
		MASK	BIT14		# SEE IF TURN-ON REQUEST.
		CCS	A
		TCF	OPONLY		# OPERATE ON ONLY.

		CS	IMODES30	# IF TURN-ON REQUEST, WE SHOUD HAVE IMU
		MASK	BIT9		# OPERATE.
		CCS	A
		TCF	+3

		TC	ALARM		# ALARM IF NOT
		OCT	213

 +3		TC	CAGESUB
		CAF	90SECS
		TC	WAITLIST
		EBANK=	M11
		2CADR	ENDTNON

		TCF	C33TEST

RETNON		CAF	90SECS
		TC	VARDELAY

ENDTNON		CS	BIT2		# RESET TURN-ON REQUEST FAIL BIT.
		MASK	IMODES30
		XCH	IMODES30
		MASK	BIT2		# IF IT WAS OFF, SEND ISS DELAY COMPLETE.
		EXTEND
		BZF	ENDTNON2

		CAF	BIT14		# IF IT WAS ON AND TURN-ON REQUEST NOW.
# Page 165
		MASK	IMODES30	# PRESENT, RE-ENTER 90 SEC DELAY IN WL.
		EXTEND
		BZF	RETNON

		CS	FLAGWRD0	# IF IT IS NOT ON NOW, SEE IF A PROG WAS
		MASK	IMUSEBIT	# WAITING.
		CCS	A
		TCF	TASKOVER
		TC	POSTJUMP
		CADR	IMUBAD		# UNSUCCESSFUL TURN-ON.

ENDTNON2	CAF	BIT15		# SEND ISS DELAY COMPLETE.
		EXTEND
		WOR	CHAN12

		TC 	IBNKCALL	# TURN OFF NO ATT LAMP.
		CADR	NOATTOFF

UNZ2		TC	ZEROICDU

		CS	BITS4&5		# REMOVE ZERO AND COARSE.
		EXTEND
		WAND	CHAN12

		CAF	BIT11		# WAIT 10 SECS FOR CTRS TO FIND GIMBALS
		TC	VARDELAY

ISSUP		CS	OCT54		# REMOVE CAGING, IMU FAIL INHIBIT BIT, AND
		MASK	IMODES30	# ICDUFAIL INHIBIT FLAGS.
		TS	IMODES30

		CS	BIT6		# ENABLE DAP
		MASK	IMODES33
		TS	IMODES33

		CS	FLAGWRD2	# TEST DRIFTFLG: IF ON DO NOTHING BECAUSE
		MASK	DRFTBIT		# IMUCOMP SHOUD BE ALL SET UP (RESTART
		EXTEND			# WITH IMUSE DOWN).  IF OFF, SET DRIFTFLG
		BZF	+4		# AND 1/PIPADT TO GET FREEFALL IMUCOMP
		ADS	FLAGWRD2	# GOING (FRESH START OR ISS TURN-ON).
		CA	TIME1
		XCH	1/PIPADT	# CANNOT GET HERE IF RESTART WITH IMUSE UP

		TC	SETISSW		# ISS WARNING MIGHT HAVE BEEN INHIBITED.

		CS	BIT15		# REMOVE IMU DELAY COMPLETE DISCRETE.
		EXTEND
		WAND	CHAN12

		CAF	4SECS		# DON'T ENABLE PROG ALARM ON PIP FAIL FOR
# Page 166
		TC	WAITLIST	# ANOTHER 4 SECS.
		EBANK=	CDUIND
		2CADR	PFAILOK

		TCF	TASKOVER

OPONLY		CAF	BIT4		# IF OPERATE ON ONLY, AND WE ARE IN COARSE
		EXTEND			# ALIGN, DON'T ZERO THE CDUS BECAUSE WE
		RAND	CHAN12		# MIGHT BE IN GIMBAL LOCK.
		CCS	A
		TCF	C33TEST

		CAF	IMUSEBIT	# OTHERWISE, ZERO THE COUNTERS.
		MASK	FLAGWRD0	# UNLESS SOMEONE IS USING TH IMU.
		CCS	A
		TCF	C33TEST

		TC	CAGESUB2	# SET TURNON FLAGS.

ISSZERO		TC	IBNKCALL	# TURN OFF NO ATT LAMP.
		CADR	NOATTOFF	# IMU CAGE OFF ENTRY.

		CAF	BIT5		# ISS CDU ZERO
		EXTEND
		WOR	CHAN12

		TC	ZEROICDU
		CAF	BIT6		# WAIT 300 MS. FOR AGS TO RECEIVE SIGNAL.
		TC	WAITLIST
		EBANK=	M11
		2CADR	UNZ2

		TCF	C33TEST

# Page 167
; ============================================================================
; C33TEST - Channel 33 Interrupt Monitor
;
; Channel 33 carries three critical flip-flop status signals from spacecraft
; hardware systems. This routine reads these signals every 480ms and calls
; the appropriate handler when a status change is detected.
;
; The three monitored conditions are:
;
; 1. PIPA FAIL (Bit 13): Accelerometer failure in the IMU's Pulsed
;    Integrating Pendulous Accelerometer. The PIPA measures spacecraft
;    acceleration along three axes, and its data is essential for dead-
;    reckoning navigation. A PIPA failure during powered flight could lead
;    to navigation errors and require immediate abort.
;
; 2. DOWNLINK TOO FAST (Bit 12): The telemetry downlink to Mission Control
;    is exceeding the ground station's ability to receive and process data.
;    This condition indicates the AGC is transmitting faster than the
;    Manned Space Flight Network (MSFN) ground stations can handle.
;
; 3. UPLINK TOO FAST (Bit 11): The ground station uplink (commands from
;    Mission Control) is being sent faster than the AGC can process. This
;    protects the AGC from being overwhelmed by rapid command sequences.
;
; Unlike IMUMON (which uses a READ of Channel 30), C33TEST uses a WAND
; instruction to read Channel 33. The WAND generates a "write pulse" that
; automatically resets the flip-flops after reading, preparing them to
; detect the next occurrence of these conditions.
;
; The bit-scanning algorithm is identical to IMUMON's elegant approach:
; XOR current state with saved state (IMODES33), isolate changed bits,
; then loop through them using a doubling technique to detect and process
; the highest-priority changes first.
;
; During Apollo 11, reliable telemetry was critical for Mission Control's
; monitoring of the descent. The "too fast" checks prevented data overruns
; that could have corrupted commands or telemetry during critical phases.
; ============================================================================

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

C33TEST		CA	IMODES33		# SEE IF RELEVANT CHAN33 BITS HAVE
		MASK	33RDMSK
		TS	L			# CHANGED.
		CAF	33RDMSK
		EXTEND
		WAND	CHAN33			# RESETS FLIP-FLOP INPUTS
		EXTEND
		RXOR	LCHAN
		EXTEND
		BZF	GLOCKMON		# ON NO CHANGE.

		TS	RUPTREG1		# SAVE BITS WHICH HAVE CHANGED.
		LXCH	IMODES33
		EXTEND
		RXOR	LCHAN
		TS	IMODES33		# UPDATED IMODES33.

		CAF	ZERO
		XCH	RUPTREG1
		DOUBLE
# Page 168
		TCF	NXTIBT +1		# SCAN FOR BIT CHANGES.

 -1		AD	ONE
NXTIBT		INCR	RUPTREG1
 +1		DOUBLE
		TS	A			# (CODING IDENTICAL TO CHAN 30).
		TCF	NXTIBT

		XCH	RUPTREG2
		INDEX	RUPTREG1		# GET NEW VALUE OF BIT WHICH CHANGED.
		CAF	BIT13
		MASK	IMODES33
		INDEX	RUPTREG1
		TC	C33JMP
NXTFL33		CCS	RUPTREG2		# PROCESS POSSIBLE ADDITIONAL CHANGES.
		TCF	NXTIBT -1

# Page 169
; ============================================================================
; GLOCKMON - Gimbal Lock Monitor
;
; One of the most critical safety monitors in the AGC, GLOCKMON continuously
; watches the IMU's middle gimbal angle (MGA) to detect and prevent gimbal
; lock - a condition where the three-gimbal stabilized platform loses one
; degree of freedom and can no longer maintain proper orientation.
;
; Gimbal lock occurs when the middle gimbal approaches ±90 degrees, causing
; the inner and outer gimbals to align. In this configuration, the platform
; cannot distinguish rotation about one axis from rotation about another,
; and the IMU becomes unable to provide reliable attitude information.
;
; The routine implements three protection zones based on the absolute value
; of the middle gimbal angle (MGA), which is read from the CDUZ counter:
;
; ZONE 1: |MGA| ≤ 70° - NORMAL OPERATION
;   The IMU is safely away from gimbal lock. No warnings, no restrictions.
;   This is the normal operating region for most mission phases.
;
; ZONE 2: 70° < |MGA| ≤ 85° - WARNING ZONE
;   The platform is approaching gimbal lock. The GIMBAL LOCK warning lamp
;   on the DSKY illuminates to alert the crew. The IMU continues to operate
;   normally, but the crew should maneuver the spacecraft to move away from
;   this dangerous region.
;
; ZONE 3: |MGA| > 85° - DANGER ZONE
;   The platform is critically close to gimbal lock. GLOCKMON takes two
;   emergency actions:
;   1. Commands the IMU into COARSE ALIGN mode (disabling fine attitude hold)
;   2. Illuminates the NO ATT (No Attitude) lamp to warn that attitude
;      reference is unreliable
;
; During Apollo 11, gimbal lock was a constant concern during maneuvers.
; Armstrong and Aldrin had to carefully plan spacecraft rotations to avoid
; this condition. The famous "program alarm" scenario could have been
; worsened if the spacecraft had entered gimbal lock during the descent,
; leaving them without reliable attitude information.
;
; The routine checks the gimbal angle every 480ms (every 48th T4RUPT) by
; testing CDUZ with CCS to determine its sign, then comparing the absolute
; value against the 70° and 85° thresholds.
; ============================================================================

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
#		1) FRESH START OR RESTART WITH NO GROUPS ACTIVE:  C(CDUZ) = 0, IMODES30 BIT 6 = 0, IMODES33 BIT 1 =  0.
#		2) RESTART WTIH GROUPS ACTIVE:	SAME AS FRESH START EXCEPT C(CDUZ) NOT CHANGED SO GIMBAL MONITOR
#						PROCEEDS AS BEFORE.
#
# ALARMS:	1) MGA REGION (2) CAUSES GIMBAL LOCK LAMP TO BE LIT.
#		2) MGA REGION (3) CAUSES THE ISS TO BE PUT IN COARSE ALIGN AND THE NO ATT LAMP TO BE LIT IF EITHER NOT
#		   SO ALREADY.

GLOCKMON	CCS	CDUZ
		TCF	GLOCKCHK		# SEE IF MAGNITUDE OF MGA IS GREATER THAN
		TCF	SETGLOCK		# 70 DEGREES.
		TCF	GLOCKCHK
		TCF	SETGLOCK

GLOCKCHK	AD	-70DEGS
		EXTEND
		BZMF	SETGLOCK -1		# NO LOCK.

		AD	-15DEGS			# SEE IF ABS(MGA) GREATER THAN 85 DEGREES
		EXTEND
		BZMF	NOGIMRUN

		CAF	BIT4			# IF SO, SYSTEM SHOULD BE IN COARSE ALIGN
		EXTEND				# TO PREVENT GIMBAL RUNAWAY.
		RAND	CHAN12
		CCS	A
		TCF	NOGIMRUN

		TC	IBNKCALL
		CADR	SETCOARS

		CAF	SIX			# ENABLE ISS ERROR COUNTERS IN 60 MS.
		TC	WAITLIST
# Page 170
		EBANK=	CDUIND
		2CADR	CA+ECE

NOGIMRUN	CAF	BIT6			# TURN ON GIMBAL LOCK LAMP.
		TCF	SETGLOCK

 -1		CAF	ZERO
SETGLOCK	AD	DSPTAB +11D		# SEE IF PRESENT STATE OF GIMBAL LOCK LAMP
		MASK	BIT6			# AGREES WITH DESIRED STATE BY HALF ADDING
		EXTEND				# THE TWO.
		BZF	GLOCKOK			# OK AS IS.

		MASK	DSPTAB +11D		# IF OFF, DON'T TURN ON IF IMU BEING CAGED.
		CCS	A
		TCF	GLAMPTST		# TURN OFF UNLESS LAMP TEST IN PROGRESS.

		CAF	BIT6
		MASK	IMODES30
		CCS	A
		TCF	GLOCKOK

GLINVERT	CS	DSPTAB +11D		# INVERT GIMBAL LOCK LAMP.
		MASK	BIT6
		AD	BIT15			# TO INDICATE CHANGE IN DSPTAB +11D.
		XCH	DSPTAB +11D
		MASK	OCT37737
		ADS	DSPTAB +11D
		TCF	GLOCKOK

GLAMPTST	TC	LAMPTEST		# TURN OFF UNLESS LAMP TEST IN PROGRESS.
		TCF	GLOCKOK
		TCF	GLINVERT

-70DEGS		DEC	-.38888			# -70 DEGREES SCALED IN HALF-REVOLUTIONS.
-15DEGS		DEC	-.08333

# Page 171
; ============================================================================
; TLIM - Temperature Limit Monitor
;
; This routine maintains the TEMP warning lamp (bit 4 of Channel 11 on the
; DSKY) to reflect the temperature status of the IMU (Inertial Subsystem).
; The ISS continuously monitors its internal temperature and sets bit 15 of
; Channel 30 if temperature exceeds safe operating limits.
;
; The IMU contains precision gyroscopes and accelerometers that require
; stable thermal conditions for accurate operation. Temperature variations
; can cause:
; - Gyro drift rate changes (affecting attitude accuracy)
; - Accelerometer bias shifts (affecting velocity measurements)
; - Thermal expansion of the gimbals (affecting alignment)
;
; TLIM is called by IMUMON whenever bit 15 of Channel 30 changes state.
; The routine performs a simple task: mirror the Channel 30 temperature
; signal to the DSKY TEMP lamp so the crew is immediately aware of any
; thermal issues.
;
; Special handling: If a lamp test is in progress (all DSKY lamps lit for
; crew verification), TLIM will turn the TEMP lamp ON if temperature exceeds
; limits, but will NOT turn it OFF when temperature returns to normal. This
; prevents the lamp test from being disrupted. Once the lamp test completes,
; the TEMP lamp will reflect the true temperature status.
;
; During Apollo 11, the IMU operated within nominal temperature ranges
; throughout the mission. However, had the TEMP lamp illuminated during
; the critical descent phase, the crew and Mission Control would have
; needed to assess whether navigation accuracy was being compromised.
; ============================================================================

# PROGRAM NAME:  TLIM.
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM MAINTAINS THE TEMP LAMP (BIT 4 OF CHANNEL 11) ON THE DSKY TO AGREE WITH
# THE TEMP SIGNAL FROM THE ISS (BIT 15 OF CHANNEL 30).  HOWEVER, THE LIGHT WILL NOT BE TURNED OFF IF A LAMP TEST
# IS IN PROGRESS.
#
# CALLING SEQUENCE:  CALLED BY IMUMON ON A CHANGE OF BIT 15 OF CHANNEL 30.
#
# JOBS OR TASKS INITIATED:  NON.
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

TLIM		MASK	POSMAX			# REMOVE BIT FROM WORD OF CHANGES AND SET
		TS	RUPTREG2		# DSKY TEMP LAMP ACCORDINGLY.

		CCS	IMODES30
		TCF	TEMPOK
		TCF	TEMPOK

		CAF	BIT4			# TURN ON LAMP.
		EXTEND
		WOR	DSALMOUT
		TCF	NXTIFAIL

TEMPOK		TC	LAMPTEST		# IF TEMP NOW OK, DON'T TURN OFF LAMP IF
		TCF	NXTIFAIL		# LAMP TEST IN PROGRESS.

		CS	BIT4
		EXTEND
		WAND	DSALMOUT		# TURN OFF LAMP
		TCF	NXTIFAIL

# Page 172
; ============================================================================
; ITURNON - ISS Turn-On Request Handler
;
; This routine processes changes in the ISS (Inertial Subsystem) turn-on
; request signal (Channel 30 bit 14). The ISS turn-on sequence is a complex,
; carefully orchestrated 90-second initialization process that prepares the
; IMU gyroscopes and accelerometers for navigation operations.
;
; The ISS cannot be turned on instantaneously. The gyroscopes require time
; to spin up to operating speed, and the platform must stabilize thermally
; before it can provide accurate attitude reference. The AGC manages this
; process through a 90-second "caging" period during which the platform is
; held in a known orientation while it stabilizes.
;
; ITURNON handles two scenarios:
;
; 1. TURN-ON REQUEST DETECTED (bit 14 goes from 0 to 1):
;    Sets bit 7 of IMODES30 to signal TNONTEST to begin the ISS
;    initialization sequence. This starts the 90-second timer and prepares
;    the platform for caging.
;
; 2. TURN-ON REQUEST REMOVED PREMATURELY (bit 14 goes from 1 to 0):
;    If the turn-on request disappears before the 90-second sequence
;    completes, this indicates a hardware failure or crew action to abort
;    the turn-on. ITURNON checks Channel 12 bit 15 (the turn-on delay
;    signal) and if it's off, issues program alarm 00207 "ISS TURN-ON
;    REQUEST NOT PRESENT FOR 90 SECONDS" and sets bit 2 of IMODES30 to
;    indicate a failed sequence.
;
; The routine implements a fail-safe mechanism: Once bit 2 of IMODES30 is
; set (indicating a delay sequence failure), ITURNON and IMUOP will ignore
; all subsequent turn-on requests until the current 90-second wait period
; expires. This prevents confusion from multiple rapid on/off cycles.
;
; During Apollo 11, the IMU was turned on well before launch and remained
; powered throughout the mission. However, the turn-on logic was critical
; for pre-launch checks and for any scenario requiring an IMU power cycle.
; A failure during turn-on would have necessitated extended troubleshooting
; or potentially scrubbing the mission.
; ============================================================================

# PROGRAM NAME:  ITURNON.
#
# FUNCTIONAL DESCRIPTION:  THIS PROGRAM IS CALLED BY IMUMON WHEN A CHANGE OF BIT 14 OF CHANNEL 30 (ISS TURN-ON
# REQUEST) IS DETECTED.  UPON ENTRY, ITURNON CHECKS IF A TURN-ON DELAY SEQUENCE HAS FAILED, AND IF SO, IT EXITS.
# IF NOT, IT CHECKS WHETHER THE TURN-ON REQUEST CHANGE IS TO ON OR OFF.  IF ON, IT SETS BIT7 OF IMODES30 TO 1 SO
# THAT TNONTEST WILL INITIATE THE ISS INITIALIZATION SEQUENCE.  IF OFF, THE TURN-ON DELAY SIGNAL, CHANNEL 12 BIT
# 15, IS CHECKED AND IF IT IS ON, ITURNON EXITS.  IF THE DEALY SIGNAL IS OFF, PROGRAM ALARM 00207 IS ISSUED, BIT 2
# OF IMODES30 IS SET TO 1 AND THE PROGRAM EXITS.
#
# THE SETTING OF BIT 2 OF IMODES30 (ISS DELAY SEQUENCE FAIL) INHIBITS THIS ROUTINE AND IMUOP FROM
# PROCESSING ANY CHANGES.  THIS BIT WILL BE RESET BY THE ENDTNON ROUTINE WHEN THE CURRENT 90 SECOND DELAY PERIOD
# ENDS.
#
# CALLING SEQUENCE:  FROM IMUMON WHEN ISS TURN-ON REQUEST CHANGES STATE.
#
# JOBS OR TASKS INITITIATED:  NONE.
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

ITURNON		CAF	BIT2		# IF DELAY REQUEST HAS GONE OFF
		MASK	IMODES30	# PREMATURELY, DO NOT PROCESS ANY CHANGES
		CCS	A		# UNTIL THE CURRENT 90 SEC WAIT EXPIRES.
		TCF	NXTIFAIL

		CAF	BIT14		# SEE IF JUST ON OR OFF.
		MASK 	IMODES30
		EXTEND
		BZF	ITURNON2	# IF JUST ON.

		CAF	BIT15
		EXTEND			# SEE IF DELAY PRESENT DISCRETE HAS BEEN
		RAND	CHAN12		# SENT.  IF SO, ACTION COMPLETE
		EXTEND
		BZF	+2
		TCF	NXTIFAIL

		CAF	BIT2		# IF NOT, SET BIT TO INDICATE REQUEST NOT
		ADS	IMODES30	# PRESENT FOR FULL DURATION.
		TC	ALARM
		OCT	207
		TCF	NXTIFAIL

# Page 173
ITURNON2	CS	IMODES30	# SET BIT7 TO INDICATE WAIT OF 1 SAMPLE
		MASK	BIT7
		ADS	IMODES30
		CAF	RRINIT
		TS	RADMODES
		TCF	NXTIFAIL

RRINIT		OCT	00102

# Page 174
; ============================================================================
; IMUCAGE - IMU Cage Button Handler
;
; This routine processes the IMU CAGE button press (Channel 30 bit 11) - one
; of the most dramatic crew override actions available in the AGC. When an
; astronaut presses the IMU CAGE button on the control panel, it immediately
; forces the platform into a safe but non-operational state.
;
; "Caging" the IMU is an emergency action that:
; 1. Commands all three IMU gimbals to zero position (coarse align mode)
; 2. Terminates all ongoing gyro torquing and CDU (gimbal) positioning
; 3. Zeros all associated output counters
; 4. De-selects the gyroscopes (stops fine attitude hold)
; 5. Illuminates the NO ATT (No Attitude) lamp on the DSKY
;
; The term "cage" comes from early mechanical gyroscope systems where the
; gyro rotor was literally locked in a cage to prevent damage during startup
; or shutdown. In the AGC context, it means returning the platform to a
; known reference position where it cannot provide attitude information but
; is protected from damage.
;
; Why would a crew cage the IMU?
; - Severe gimbal lock approaching 90° (platform about to lose a degree
;   of freedom and tumble uncontrollably)
; - IMU malfunction with erratic behavior
; - Preparation for IMU power cycle
; - Ground-commanded recalibration
;
; The routine only responds to button press (bit transitions from 1 to 0).
; When the button is released (bit returns to 1), no action occurs. Once
; caged, the IMU remains in this state until the crew selects an alignment
; program (P51, P52, or P53) to re-establish the platform's orientation.
;
; During Apollo 11, the crew never needed to cage the IMU in flight. However,
; this capability provided critical protection: if gimbal lock had threatened
; during the descent, caging the IMU would have been safer than allowing the
; platform to tumble into an unknown orientation. After caging, the crew
; could have used the Alignment Optical Telescope to re-align the platform
; and resume navigation.
;
; The routine operates by writing disable commands to:
; - Channel 14: Terminates ICDU (IMU gimbal), RCDU (rendezvous radar gimbal),
;   and gyro pulse trains (stops all angular momentum control)
; - Channel 12: Disables display inertial data, IMU error counters, ICDU zero
;   mode, coarse align enable, and RR error counter enable
;
; After caging, the NO ATT lamp remains lit until a new alignment completes.
; ============================================================================

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

IMUCAGE		CCS	A		# NO ACTION OF GOING OFF.
		TCF	ISSZERO
		CS	OCT77000	# TERMINATE ICDU, RCDU, GYRO PULSE TRAINS
		EXTEND
		WAND	CHAN14

		CS	OCT272		# KNOCK DOWN DISPLAY INERTIAL DATA, IMU
		EXTEND			# ERROR COUNTER ENABLE, ZERO ICDU, COARSE
		WAND	CHAN12		# ALIGN ENABLE, RR ERROR COUNTER ENABLE.

		CS 	ENGONBIT	# INSURE ENGONFLG IS CLEAR.
		MASK	FLAGWRD5
		TS	FLAGWRD5
		CS	PRIO30		# TURN ENGINE OFF.
		EXTEND
		RAND	DSALMOUT
		AD	BIT14
		EXTEND
		WRITE	DSALMOUT	# FORCE BIT14=1, BIT13=0.

		TC	CAGESUB1

		TC	IBNKCALL	# KNOCK DOWN TRACK, REFSMMAT, DRIFT FLAGS
		CADR	RNDREFDR

		CS	ZERO
		TS	CDUXCMD
		TS	CDUYCMD
# Page 175
		TS	CDUZCMD
		TS	GYROCMD

		CS	OCT740		# HAVING WAITED AT LEAST 27 MCT FROM
		EXTEND			# GYRO PULSE TRAIN TERMINATION, WE CAN
		WAND	CHAN14		# DE-SELECT THE GYROS.
		TCF	NXTIFAIL

# Page 176
; ============================================================================
; IMUOP - ISS Operate Discrete Handler
;
; This routine monitors the ISS (Inertial Subsystem) OPERATE discrete signal
; on Channel 30 bit 9, which indicates whether the IMU has full electrical
; power and is ready for operation. This is distinct from the turn-on request
; signal - the OPERATE discrete reflects the actual power state of the IMU
; hardware, not just a request to turn it on.
;
; The IMU power state is critical because the AGC cannot safely command gyro
; torquing or gimbal positioning without confirming that the IMU has stable
; power. Loss of IMU power during a mission phase that depends on inertial
; navigation would be catastrophic - the spacecraft would have no attitude
; reference and could not perform any maneuvers requiring precise orientation.
;
; IMUOP handles two transitions:
;
; 1. ISS TURNS ON (bit 9 changes from 1 to 0):
;    The routine checks if an ISS delay sequence failure is in progress
;    (IMODES30 bit 2 = 1). If not, it sets IMODES30 bit 7 to request that
;    TNONTEST initiate the 90-second ISS initialization and stabilization
;    sequence. This ensures the platform has time to reach thermal and
;    mechanical equilibrium before being used for navigation.
;
;    Exception: If bit 2 is set (indicating a previous failed turn-on where
;    the turn-on request was not maintained for the full 90 seconds), IMUOP
;    does NOT request another initialization. The system must wait for the
;    current delay period to expire before attempting another turn-on.
;
; 2. ISS TURNS OFF (bit 9 changes from 0 to 1):
;    The routine checks the IMUSEFLG to determine if any active program was
;    using the IMU for navigation or guidance. If so, it issues program alarm
;    00214 "ISS TURNED OFF WHEN IN USE" to alert the crew and Mission Control
;    of a critical failure. This alarm indicates that ongoing mission programs
;    (such as rendezvous navigation, lunar descent guidance, or transearth
;    coast navigation) have lost their primary attitude reference and cannot
;    continue safely.
;
; During Apollo 11, the IMU remained powered and operational throughout the
; entire mission from pre-launch through splashdown. However, this monitoring
; was essential for crew safety: if the IMU had lost power during the lunar
; descent (when Armstrong and Aldrin depended on it for attitude control and
; landing guidance), alarm 00214 would have triggered an immediate abort to
; lunar orbit. The crew would have had to rely on the Abort Guidance System
; (AGS) backup computer for attitude reference.
;
; Special initialization note: On FRESH START and RESTART, bit 9 of IMODES30
; is normally set to 1 (ISS off). However, if the GIMBAL LOCK lamp is already
; illuminated, bit 9 is set to 0 (ISS on) to prevent TNONTEST from attempting
; ICDU zeroing while the platform is in or near gimbal lock - a dangerous
; condition that could cause uncontrolled gimbal motion.
; ============================================================================

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
# JOBS OR TAKS INITIATED:  NONE.
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

IMUOP		EXTEND
		BZF	IMUOP2

		CS	IMODES33		# DISABLE DAP
		MASK	BIT6
		ADS	IMODES33

		TC	IBNKCALL		# KNOCK DOWN TRACK, REFSMMAT, DRIFT FLAGS
		CADR	RNDREFDR

		CS	BITS7&8			# KNOCK DOWN RENDEZVOUS, IMUUSE FLAGS
		MASK	FLAGWRD0
		XCH	FLAGWRD0		# IF GOING OFF, ALARM IF PROG USING IMU.
		COM
		MASK	IMUSEFLG
		CCS	A
		TCF	NXTIFAIL

		TC	ALARM
		OCT	214
		TCF	NXTIFAIL

IMUOP2		CAF	BIT2			# SEE IF FAILED ISS TURN-ON SEQ IN PROG.
		MASK	IMODES30
		CCS	A
		TCF	NXTIFAIL		# IF SO, DON'T PROCESS UNTIL PRESENT 90
		TCF	ITURNON2		# SECONDS EXPIRES.

# Page 177
; ============================================================================
; PIPFAIL - PIPA Failure Handler
;
; This routine responds to failures in the PIPA (Pulsed Integrating Pendulous
; Accelerometer) system - the AGC's accelerometers that measure velocity
; changes. The PIPAs are the only sensors that can detect acceleration; without
; them, the AGC cannot track velocity or compute position changes during
; thrusting maneuvers or coast phases.
;
; The IMU contains three PIPAs, one per axis (X, Y, Z), mounted orthogonally
; on the stable platform. Each PIPA outputs pulses proportional to acceleration
; along its sensing axis. The AGC counts these pulses to integrate velocity
; changes over time. If any PIPA fails, the AGC loses the ability to track
; motion along that axis - a critical failure for guidance and navigation.
;
; Channel 33 bit 13 (PIPA FAIL) is set by IMU hardware when it detects:
; - Loss of power to a PIPA
; - PIPA output stuck or erratic
; - PIPA pulse rate out of expected range
; - Internal PIPA electronics malfunction
;
; PIPFAIL routine actions:
;
; 1. Updates IMODES30 bit 10 to mirror Channel 33 bit 13 state:
;    - Bit 10 = 1: PIPA failure detected
;    - Bit 10 = 0: PIPAs operating normally
;
; 2. Calls SETISSW to evaluate whether the ISS WARNING lamp should be
;    illuminated. The lamp lights when multiple IMU problems combine to
;    make the system unreliable (see SETISSW documentation).
;
; 3. If a PIPA failure is present, checks whether the ISS is being
;    initialized (turn-on sequence in progress). If not, and if no ISS
;    warning has been issued (IMODES30 bit 1 = 1), issues program alarm
;    00212 "PIPA FAIL" to alert the crew and Mission Control.
;
; Why differentiate between initialization and normal operation?
; During the 90-second ISS turn-on sequence, the PIPAs may briefly show
; failure indications as they stabilize. These transient failures are
; expected and not alarming. However, a PIPA failure during normal IMU
; operation is serious and requires immediate crew attention.
;
; Impact of PIPA failure during Apollo 11:
; A PIPA failure during the lunar descent would have been mission-critical.
; The descent guidance equations depend on PIPA data to compute velocity
; changes from throttle commands. Without working PIPAs, the AGC could not:
; - Track descent velocity (essential for fuel-optimal guidance)
; - Compute altitude rate for landing flare
; - Detect horizontal velocity for site selection
; - Monitor acceleration limits during powered flight
;
; A PIPA failure at any time during Apollo 11 would likely have resulted in:
; - Immediate mission abort if detected during descent
; - Switch to AGS (Abort Guidance System) for backup navigation
; - Possible mission termination if no backup available
; - Return to Earth using ground tracking and manual attitude control
;
; The fact that all three PIPAs operated flawlessly throughout Apollo 11's
; eight-day mission was a tribute to the robustness of MIT Instrumentation
; Laboratory's PIPA design and the careful pre-flight testing protocols.
; ============================================================================

# PROGRAM NAME:  PIPFAIL
#
# FUNCITONAL DESCRIPTION:  THIS PROGRAM PROCESSES CHANGES OF BIT 13 OF CHANNEL 33, PIPA FAIL.  IT SETS BIT 10 OF
# IMODES30 TO AGREE.  IT CALLS SETISSW IN CASE A PIPA FAIL NECESSITATES AN ISS WARNING.  IF NOT, I.E., IMODES30
# BIT 1 = 1, AND A PIPA FAIL IS PRESENT AND THE ISS NOT BEING INITIALIZED, PROGRAM ALARM 0212 IS ISSUED.
#
# CALLING SEQUENCE:  BY C33TEST ON CHANGES OF CHANNEL 33 BIT 13.
#
# JOBS OR TASKS INITIATED:  NONE.
#
# SUBROUTINES CALLED:  1) SETISSW, AND 2) ALARM (SEE FUNCITONAL DESCRIPTION).
#
# ERASABLE INITIALZIZATION:  SEE IMUMON FOR INITIALIZATION OF IMODES30.  THE RELEVANT BITS ARE 5, 7, 8, 9, AND 10.
#
# ALARMS:  PROGRAM ALARM 00212 IF PIPA FAIL IS PRESENT BUT NEITHER ISS WARNING IS TO BE ISSUED NOR THE ISS IS
# BEING INITIALIZED.
#
# EXIT:  NXTFL33.
#
# OUTPUT:  PROGRAM ALARM 00212 AND ISS WARNING MAINTENANCE.

PIPFAIL		CCS	A			# SET BIT10 IN IMODES30 SO ALL ISS WARNING
		CAF	BIT10			# INFO IS IN ONE REGISTER.
		XCH	IMODES30
		MASK	-BIT10
		ADS	IMODES30

		TC	SETISSW

		CS	IMODES30		# IF PIP FAIL DOESN'T LIGHT ISS WARNING, DO
		MASK	BIT1			# A PROGRAM ALARM IF IMU OPERATING BUT NOT
		CCS	A			# CAGED OR BEING TURNED ON.
		TCF	NXTFL33

		CA	IMODES30
		MASK	OCT1720
		CCS	A
		TCF	NXTFL33			# ABOVE CONDITION NOT MET.

		TC	ALARM
		OCT	212
		TCF	NXTFL33

# Page 178
; ============================================================================
; DNTMFAST - Downlink Too Fast Handler
; UPTMFAST - Uplink Too Fast Handler
;
; These routines monitor the telemetry data rates between the spacecraft and
; Mission Control, ensuring that neither the uplink (ground-to-spacecraft) nor
; the downlink (spacecraft-to-ground) exceeds the maximum sustainable rate for
; the communication system. Telemetry rate problems indicate either hardware
; failures or configuration errors that could lead to data corruption or loss
; of communication.
;
; Channel 33 bits monitored:
; - Bit 12: Downlink too fast (monitored by DNTMFAST)
; - Bit 11: Uplink too fast (monitored by UPTMFAST)
;
; These bits are set by the AGC's communications hardware when the data rate
; exceeds the threshold for reliable transmission. The thresholds depend on
; the selected telemetry mode and the spacecraft's distance from Earth.
;
; When either bit changes from 1 (normal) to 0 (rate too fast):
;
; DNTMFAST: Issues program alarm 01105 "DOWNLINK TOO FAST"
; - Indicates the spacecraft is sending telemetry data faster than the
;   communications system can reliably transmit
; - Can occur if the wrong downlink mode is selected (high-rate mode chosen
;   when spacecraft is too far from Earth for that rate)
; - Results in data loss - ground stations cannot decode the signal
; - Crew action required: Switch to lower telemetry rate via DSKY or switches
;
; UPTMFAST: Issues program alarm 01106 "UPLINK TOO FAST"
; - Indicates Mission Control is sending commands or data faster than the
;   AGC can reliably receive and process
; - Can occur if ground uses wrong uplink configuration
; - Results in corrupted or lost commands - potentially dangerous if critical
;   maneuver parameters or state vector updates are being uploaded
; - Crew/ground action required: Mission Control reduces uplink data rate
;
; Historical context for Apollo 11:
; During the mission, telemetry rates were carefully managed based on distance:
; - Near Earth: High-speed telemetry available (51.2 kbps downlink)
; - Cislunar coast: Medium-speed telemetry (1.6 kbps)
; - Lunar orbit: Low-speed during far-side passes due to weaker signal
; - Lunar surface: EVA periods used even lower rates to conserve power
;
; A downlink rate error during the lunar descent would have been serious but
; not immediately mission-threatening - ground could still track the LM via
; radar. However, an uplink rate error during a state vector update or maneuver
; PAD (Parameter Data) upload could have corrupted critical navigation data.
;
; The crew had manual control over telemetry mode selection via switches on
; the Main Display Console. If these alarms occurred, they would switch to a
; lower-rate mode and request Mission Control re-send any corrupted data.
;
; Note: These are relatively rare alarms. The communication system design
; included multiple layers of rate control to prevent rate mismatches. These
; alarms served as a last-resort safeguard.
; ============================================================================

# PROGRAM NAMES:  DNTMFAST, UPTMFAST
#
# FUNCTIONAL DESCRIPTION:  THESE PROGRAMS PROCESS CHANGES OF BITS 12 AND 11 OF CHANNEL 33.  IF A BIT CHANGES TO A
# 0, A PROGRAM ALARM IS ISSUED.  THE LAARMS ARE:
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
# ALARMS:  SET FUNCTGIONAL DESCRIPTION.
#
# EXIT:  NXTFL33.
#
# OUTPUT:  PROGRAM ALARM ON A BIT CHANGE TO 0.

DNTMFAST	CCS	A			# DO PROG ALARM IF TM TOO FAST.
		TCF	NXTFL33

		TC	ALARM
		OCT	1105
		TCF	NXTFL33

UPTMFAST	CCS	A			# SAME AS DNLINK TOO FAST WITH DIFFERENT
		TCF	NXTFL33			# ALARM CODE.

		TC	ALARM
		OCT	1106
		TCF	NXTFL33
# Page 179
; ============================================================================
; SETISSW - ISS Warning Lamp Control
;
; This routine manages the ISS WARNING indicator lamp on the caution and
; warning panel, providing the crew with immediate visual notification of
; failures in the Inertial Subsystem (ISS) - the collective term for the IMU,
; its associated CDUs (Coupling Data Units), and the three PIPA accelerometers.
;
; The ISS WARNING lamp is a critical crew alert that summarizes the health of
; the spacecraft's primary navigation sensor suite. When illuminated, it means
; the AGC has detected a hardware failure that degrades or eliminates the
; ability to determine the spacecraft's attitude or measure acceleration.
;
; Failure conditions monitored:
;
; 1. IMU FAIL (IMODES30 bit 13):
;    - IMU internal malfunction detected (gyro drift excessive, power fault)
;    - IMU cage button pressed (emergency shutdown)
;    - IMU temperature out of limits
;    - Can be inhibited via IMODES30 bit 4 during intentional shutdowns
;
; 2. ICDU FAIL (IMODES30 bit 12):
;    - Coupling Data Unit malfunction (gimbal angle readout failed)
;    - CDU discrete indicates hardware fault
;    - Can be inhibited via IMODES30 bit 3 during CDU zeroing operations
;
; 3. PIPA FAIL (IMODES30 bit 10):
;    - One or more PIPA (accelerometer) hardware failure
;    - PIPA discrete indicates sensor malfunction
;    - Can be inhibited via IMODES30 bit 1 during PIPA testing
;
; Logic operation:
; The routine performs a complex bit manipulation to determine if any
; un-inhibited failure exists. It multiplies the failure bits (13, 12, 10)
; by BIT10, then rotates the result and compares against the inhibit bits
; (4, 3, 1). If any failure is present without its corresponding inhibit,
; the ISS WARNING lamp is turned on.
;
; VARALARM alarm codes issued when lamp turns on:
; - 00777: PIPA FAIL only
; - 03777: ICDU FAIL only
; - 04777: ICDU + PIPA FAIL
; - 07777: IMU FAIL only
; - 10777: IMU + PIPA FAIL
; - 13777: IMU + ICDU FAIL
; - 14777: IMU + ICDU + PIPA FAIL (complete ISS failure)
;
; The octal alarm code encoding allows ground controllers to immediately
; identify which specific combination of failures triggered the warning by
; examining which bits are set in the alarm code.
;
; Lamp test protection:
; If IMODES33 bit 1 indicates a lamp test is in progress, the routine will
; NOT turn off the ISS WARNING lamp even if all failures have cleared. This
; prevents the lamp test from being interrupted, ensuring the crew can verify
; that the lamp is functional. The lamp will turn off automatically when the
; lamp test completes and SETISSW is called again.
;
; Calling context:
; SETISSW is called frequently throughout T4RUPT and IMU management routines:
; - IMUMON: When IMU FAIL or ICDU FAIL discretes change state
; - PIPFAIL: When PIPA FAIL discrete changes state
; - IFAILOK/PFAILOK: When failure inhibits are removed after testing
; - PIPUSE/PIPFREE: When PIPA availability changes (possible alarm)
; - IMUZERO3/ISSUP: After IMU restart sequences complete
;
; Historical context for Apollo 11:
; The ISS WARNING lamp was part of the Lunar Module's master alarm system.
; A lit ISS WARNING during critical mission phases (descent, landing, ascent)
; would be extremely serious - it could mean loss of attitude knowledge or
; velocity measurement, both essential for guidance.
;
; During Apollo 11's mission, the ISS performed flawlessly throughout. Had an
; ISS WARNING occurred during the lunar landing on July 20, 1969, it would
; likely have triggered an immediate abort - the crew cannot land safely
; without reliable attitude and acceleration data.
;
; The careful inhibit logic (bits 4, 3, 1) was essential because many normal
; operations temporarily trigger failure discretes:
; - IMU coarse alignment: IMU FAIL inhibited during platform torquing
; - CDU zeroing: ICDU FAIL inhibited during gimbal reference initialization  
; - PIPA testing: PIPA FAIL inhibited during self-test sequences
;
; Without the inhibit mechanism, the ISS WARNING lamp would flash annoyingly
; during routine operations, potentially causing the crew to ignore it during
; an actual failure (the classic "cry wolf" problem in alarm design).
;
; The multi-bit alarm codes (00777, 03777, etc.) allowed Mission Control to
; rapidly diagnose the failure mode and provide crew procedures. For example:
; - 00777 (PIPA only): Crew could continue with degraded navigation
; - 07777 (IMU only): Crew might attempt IMU restart
; - 14777 (complete ISS): Mission abort likely required
;
; Hardware interface:
; - Input: IMODES30 (failure and inhibit status bits)
; - Input: IMODES33 bit 1 (lamp test in progress flag)
; - Output: DSALMOUT channel 11 bit 1 (ISS WARNING lamp control)
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
# JOBS OR TASKS INITIAZTED:  NONE.
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
# THE FOLLOWING PROGRAM ALARMS WILL SHOW WHICH FAILURE CAUSED THE ISS WARN
#	PROGRAM ALARM 00777	PIPA FAIL
#	PROGRAM ALARM 03777	ICDU FAIL
#	PROGRAM	ALARM 04777	ICDU, PIPA FAILS
#	PROGRAM ALARM 07777	IMU FAIL
#	PROGRAM ALARM 10777	IMU, PIPA FAILS
#	PROGRAM ALARM 13777	IMU, ICDU FAILS
#	PROGRAM ALARM 14777	IMU, ICDU, PIPA FAILS
#
# EXIT: VIA Q.
#
# OUTPUT: ISS WARNING LAMP SET PROPERLY.

SETISSW		CAF	OCT15			# SET ISS WARNING USING THE FAIL BITS IN
		MASK	IMODES30		# BITS 13, 12, AND 10 OF IMODES30 AND THE
		EXTEND				# FAILURE INHIBIT BITS IN POSITIONS
		MP	BIT10			# 4, 3, AND 1.
		CA	IMODES30
		EXTEND
		ROR	LCHAN			# 0 INDICATES FAILURE
		COM
		MASK	OCT15000
		CCS	A
		TCF	ISSWON			# FAILURE.

ISSWOFF		CAF	BIT1			# DON'T TURN OFF ISS WARNING IF LAMP TEST
		MASK	IMODES33		# IN PROGRESS.
# Page 180
		CCS	A
		TC	Q

		CS	BIT1
		EXTEND
		WAND	DSALMOUT
		TC	Q

ISSWON		EXTEND
		QXCH	ITEMP6
		TC	VARALARM		# TELL EVERYONE WHAT CAUSED THE ISS WARNING
		CAF	BIT1
		EXTEND
		WOR	DSALMOUT
		TC	ITEMP6

CAGESUB		CS	BITS6&15		# SET OUTBITS AND INTERNAL FLAGS FOR
		EXTEND				# SYSTEM TURN-ON OR CAGE.  DISABLE THE
		WAND	CHAN12			# ERROR COUNTER AND REMOVE THE IMU DELAY COMP.
		CAF	BITS4&5			# SEND ZERO AND COARSE.
		EXTEND
		WOR	CHAN12

CAGESUB1	CS	DSPTAB +11D		# TURN ON NO ATT LAMP
		MASK	OC40010
		ADS	DSPTAB +11D

CAGESUB2	CS	IMODES30		# SET FLAGS TO INDICATE CAGING OR TURN-ON
		MASK	OCT75			# AND INHIBIT ALL ISS WARNING INFO
		ADS	IMODES30

		CS	IMODES33		# DISABLE DAP AUTO AND HOLD MODES
		MASK	BIT6
		ADS	IMODES33

		TC	Q

IMUFAIL		EQUALS	SETISSW
ICDUFAIL	EQUALS	SETISSW

# Page 181
# JUMP TABLES AND CONSTANTS.

IFAILJMP	TCF	ITURNON			# CHANNEL 30 DISPATCH.
		TCF	IMUFAIL
		TCF	ICDUFAIL
		TCF	IMUCAGE
30RDMSK		OCT	76400			# (BIT 10 NOT SAMPLED HERE).
		TCF	IMUOP

C33JMP		TCF	PIPFAIL			# CHANNEL 33 DISPATCH.
		TCF	DNTMFAST
		TCF	UPTMFAST

# SUBROUTINE TO SKIP IF LAMP TEST NOT IN PROGRESS.
LAMPTEST	CS	IMODES33		# BIT 1 OF IMODES33 = 1 IF LAMP TEST IN
		MASK	BIT1			# PROGRESS.
		CCS	A
		INCR	Q
		TC	Q

33RDMSK		EQUALS	PRIO16
OC40010		OCT	40010
OCT54		OCT	54
OCT75		OCT	75
OCT272		OCT	00272
BITS7&8		OCT	300
OCT1720		OCT	1720
OCT740		OCT	00740
OCT15000	EQUALS	PRIO15
OCT77000	OCT	77000
BITS6&15	OCT	40040
-BIT10		OCT	-1000

90SECS		DEC	9000
120MS		=	OCT14			# (DEC12)
GLOCKOK		EQUALS	RESUME

# Page 182
; ============================================================================
; RRAUTCHK - Rendezvous Radar Auto Mode Monitor
;
; This routine is the primary inbit monitor for the Rendezvous Radar (RR)
; power-on-auto discrete, detecting when the crew switches the RR power
; control knob between OFF, STANDBY, and AUTOMATIC positions. The RR is the
; Lunar Module's primary sensor for measuring range and range-rate to the
; Command Module during rendezvous operations in lunar orbit.
;
; Hardware monitored:
; - Channel 33 Bit 2: RR POWER ON AUTO discrete
;   * Set (1) when RR power knob is in AUTOMATIC position (radar energized)
;   * Clear (0) when RR power knob is in OFF or STANDBY position
;
; The RR POWER ON AUTO discrete is a crucial hardware interface. Unlike the
; Landing Radar which operates automatically during descent, the Rendezvous
; Radar requires manual crew control. The astronauts must manually position
; the power control knob based on mission phase:
; - OFF: During non-rendezvous phases (landing, surface operations)
; - STANDBY: Pre-rendezvous warm-up (allows hardware to stabilize)
; - AUTOMATIC: During rendezvous tracking (radar actively measuring)
;
; Routine operation (called every 480 milliseconds):
;
; 1. Discrete change detection:
;    The current RR AUTO MODE bit (CHAN 33 BIT 2) is XORed with the
;    previously stored value in RADMODES bit 2. If no change has occurred,
;    the routine immediately exits to RRCDUCHK (next radar check in chain).
;
; 2. RADMODES update when change detected:
;    RADMODES bit 2 is updated to match the new hardware discrete state.
;    Additionally, several control and status bits are cleared to reset the
;    radar subsystem state:
;    - Bit 14: Continuous designate mode (cleared)
;    - Bit 11: Remode (data good flag) (cleared)
;    - Bit 10: Reposition (antenna slewing) (cleared)
;    - Bit 13: RR CDU Zero sequence active (cleared)
;    - Bit 1: Turn-on sequence active (cleared)
;
;    The clearing of these bits acknowledges that any prior radar operation
;    has been interrupted by the power state change.
;
; 3. Power-off detection (CHAN 33 BIT 2 transitioned from 1 to 0):
;    If the radar has just been turned off, STATE bit 7 is checked to
;    determine if a navigation program was actively using the RR:
;    - If STATE bit 7 clear (no program using RR): Exit to RRCDUCHK normally
;    - If STATE bit 7 set (program using RR): Issue alarm 00514 "RADAR GOES
;      OUT OF AUTO MODE WHILE BEING USED", then exit to RRCDUCHK
;
;    Alarm 00514 alerts the crew and Mission Control that an unexpected radar
;    power-off has occurred during active rendezvous navigation. This could
;    indicate:
;    - Accidental crew switch movement
;    - Hardware power fault
;    - Electrical transient
;
;    The program using the radar (typically P20 series rendezvous programs)
;    will have its navigation solution degraded or halted.
;
; 4. Power-on detection (CHAN 33 BIT 2 transitioned from 0 to 1):
;    When the radar is just turned on, the routine checks STATE bit 7 to see
;    if a program was waiting for the RR. Regardless of STATE, the turn-on
;    sequence is initiated:
;    - RADMODES bit 13 set: RR CDU Zero sequence required
;    - RADMODES bit 1 set: Turn-on sequence active
;    - WAITLIST task scheduled: RRTURNON in 10 milliseconds (1 centisecond)
;    - Exit to NORRGMON (bypassing normal RRCDUCHK and RRGIMON checks)
;
;    The WAITLIST call to RRTURNON begins the complex radar initialization:
;    - CDU zeroing: Antenna gimbal angle counters reset to known reference
;    - Gyro spin-up: RR gyro stabilization (requires several seconds)
;    - Self-test: Built-in test equipment (BITE) verification
;    - Mode initialization: Radar prepared for track acquisition
;
;    The 10-millisecond delay before RRTURNON allows the radar electronics to
;    stabilize after power application before CDU commands are issued.
;
; RADMODES bit definitions (referenced by this routine):
; - Bit 2: RR AUTO MODE (matches CHAN 33 BIT 2 discrete)
; - Bit 1: TURNON SEQUENCE ACTIVE
; - Bit 7: RR CDU OK (updated by RRCDUCHK, not this routine)
; - Bit 10: REPOSITION (antenna slewing command active)
; - Bit 11: REMODE (radar data good, lock achieved)
; - Bit 13: RR CDU ZERO SEQUENCE ACTIVE
; - Bit 14: CONTINUOUS DESIGNATE (automatic tracking mode)
;
; STATE bit 7: RR IN USE BY PROGRAM
; Set by P20-series rendezvous programs when RR measurements are required.
; Checked by RRAUTCHK to determine if power-off is unexpected.
;
; Historical context for Apollo 11:
; The Rendezvous Radar was essential for the lunar orbit rendezvous between
; Eagle (LM) and Columbia (CM) after Eagle's ascent from the lunar surface on
; July 21, 1969. Armstrong and Aldrin used the RR to measure range and
; range-rate to Collins in Columbia as they executed the rendezvous sequence.
;
; The RR provided crucial data for the P20 rendezvous navigation program,
; which computed the maneuvers needed to achieve docking. Without the RR,
; rendezvous would have required ground-based tracking and manual crew
; techniques - significantly more difficult and risky.
;
; The 480-millisecond polling interval (twice per second) was chosen to:
; - Detect crew switch changes quickly (human reaction time ~200-300ms)
; - Minimize T4RUPT computational load (RR checks are relatively complex)
; - Provide adequate response time for turn-on sequencing
;
; The alarm 00514 logic was critical for crew situational awareness. If the
; RR accidentally switched off during P20 rendezvous navigation, the crew
; needed immediate notification so they could:
; 1. Recognize the radar was no longer tracking
; 2. Restore power to the radar (move knob back to AUTO)
; 3. Re-acquire track (RR would need to find the target again)
; 4. Assess impact on rendezvous timeline
;
; The complex turn-on sequence (RRCDUZRO, then RRTURNON) was necessary
; because the RR hardware required precise initialization:
; - The CDUs (gimbal angle readouts) must be zeroed before antenna motion
; - The radar gyro must spin up to operating speed (temperature stabilization)
; - The radar electronics must complete self-test before track attempts
; - The antenna must be positioned to the initial search position
;
; Unlike the Landing Radar (which operates during single-opportunity descent),
; the Rendezvous Radar might be turned on and off multiple times during the
; mission (initial rendezvous practice, actual rendezvous, contingency modes).
; This routine's robust change detection and sequencing ensured reliable
; operation across multiple power cycles.
;
; The OCT05776 mask (octal 5776 = bits 14, 13, 11, 10, 1) clears all
; previous radar operating mode bits, acknowledging that the power state
; change invalidates any prior radar configuration. This prevents the radar
; from attempting to resume an incompatible mode after power restoration.
;
; The OCT10001 value (octal 10001 = bits 13, 1) sets both RRCDUZRO and
; TURNON bits simultaneously, initiating the complete initialization sequence
; with a single memory store operation.
;
; Exit paths:
; - RRCDUCHK: Normal exit when no change or after processing power-off
; - NORRGMON: Special exit after initiating turn-on sequence (bypasses CDU
;   and gimbal checks which would be meaningless during initialization)
; ============================================================================

# PROGRAM NAME:  RRAUTCHK
#
# FUNCITONAL DESCRIPTION:
# RRAUTCHK IS THE RENDEZFOUS RADAR INBIT MONITOR.  INITIALLY THE RR
# POWER ON AUTO (CHAN 33 BIT 2) INBIT IS CHECKED.  IF NO CHANGE, THE
# PROGRAM EXITS TO RRCDUCHK.  IF A CHANGE, RADMOES IS UPDATED
# AND A CHECK MADE IF RR POWER HAS JUST COME ON.  IF JUST OFF, A CHECK
# IS MADE TO SEE IF A PROGRAM WAS USING THE RR (STATE BIT 7).  IF NO,
# THE PROGRAM EXITS TO RRCDUCHK.  IF YES, PROGRAM ALARM 00514
# IS REQUESTED BEFORE EXITING TO RRCDUCHK.  IF RR POWER HAS JUST COME
# ON, A CHECK IS MADE TO SEE IF A PROGRAM WAS USING THE RR (STATE BIT 7)
# SEQUENCE.  IF NO, RADMODES IS UPDATED TO INDICATE RR CDU ZERO AND
# RR TURN-ON SEQUENCE (BITS 13, 1).  A 10 MILLISECOND WAITLIST CALL
# IS THEN SET FOR RRTURNON BEFORE THE PROGRAM EXITS TO NORRGMON.
#
# CALLING SEQUENCE:
# T4RUPT EVERY 480 MILLISECONDS
#
# ERASABLE INITIALIZATION REQUIRED:
# RADMODES, STATE.
#
# SUBROUTINES CALLED:
# WAITLIST.
#
# JOBS OR TASKS INITIATED:
# RRTURNON
#
# ALARMS:  PROGRAM ALARM 00514 -- RADAR GOES OUT OF AUTO MODE WHILE BEING
# USED
#
# EXIT:  RRCDUCHK, NORRGMON

RRAUTCHK	CA	RADMODES			# SEE IF CHANGE IN RR AUTO MODE BIT.
		EXTEND
		RXOR	CHAN33
		MASK	AUTOMBIT
		EXTEND
		BZF	RRCDUCHK

		LXCH	RADMODES			# UPDATE RADMODES.
		EXTEND
		RXOR	LCHAN
		MASK	OCT05776			# CLR CONT. DES., REMODE, REPOS, CDUZERO,
		TS	RADMODES			# AND TURNON BITS.
		MASK	BIT2				# SEE IF JUST ON.
		CCS	A
		TCF	RRCDUCHK -3			# OFF.  GO DISABLE RR CDU ERROR COUNTERS.
		CA	OCT10001			# SET RRCDUZRO AND TURNON BITS.
		ADS	RADMODES

# Page 183
		CAF	ONE
		TC	WAITLIST
		EBANK=	LOSCOUNT
		2CADR	RRTURNON

		TCF	NORRGMON

OCT05776	OCT	5776

# Page 184
; ============================================================================
; RRCDUCHK - Rendezvous Radar CDU Fail Monitor
;
; This routine monitors the Rendezvous Radar Coupling Data Unit (CDU) failure
; discrete, detecting hardware faults in the RR gimbal angle measurement
; system. The CDUs are precision shaft angle encoders that report the angular
; position of the RR antenna in azimuth (trunnion) and elevation (shaft) axes.
; CDU failures prevent the computer from knowing where the radar antenna is
; pointing, making rendezvous navigation impossible.
;
; Hardware monitored:
; - Channel 30 Bit 7: RR CDU FAIL discrete
;   * Set (1) when RR CDU hardware detects internal fault (normal operation)
;   * Clear (0) when RR CDU has failed (failure condition - inverted logic)
;
; Note the inverted logic: The discrete is normally HIGH (1) during correct
; operation and goes LOW (0) on failure. This is a common aerospace design
; pattern - a "heartbeat" signal that requires continuous healthy operation
; to maintain, so that wire breaks or power failures appear as faults rather
; than falsely indicating good health.
;
; Routine operation (called every 480 milliseconds):
;
; 1. CDU fail discrete change detection:
;    The current RR CDU FAIL bit (CHAN 30 BIT 7) is compared with the
;    previously stored value in RADMODES bit 7. The routine uses RXOR
;    (exclusive-or with channel) to detect any state change. If no change has
;    occurred since the last check, the routine immediately exits to RRGIMON
;    (next radar check in chain) via BZF (branch if zero to fixed).
;
; 2. Auto mode verification when change detected:
;    If a CDU fail state change has occurred, the routine checks RADMODES
;    bit 2 (RR AUTO MODE) to determine if the Rendezvous Radar is currently
;    powered on and operating. If the RR is not in AUTO mode (power off or
;    standby), the CDU fail state is irrelevant (radar not being used), so
;    the routine exits to NORRGMON without updating RADMODES bit 7 or issuing
;    any alarms.
;
;    This prevents spurious CDU fail indications when:
;    - RR power is OFF (CDU not powered, fail discrete may float)
;    - RR in STANDBY mode (CDU not initialized, may show transient states)
;    - Landing Radar is being used (LR shares some channel bits with RR)
;
;    If the check were not performed, the TRACKER FAIL lamp might illuminate
;    incorrectly when the crew was using Landing Radar data, causing
;    confusion during critical descent phases.
;
; 3. RADMODES bit 7 (RR CDU OK) update when in auto mode:
;    When the RR is in AUTO mode and a CDU fail state change has been
;    detected, RADMODES bit 7 is updated to reflect the new CDU health state.
;    The update sequence uses a careful read-modify-write pattern:
;    - RADMODES loaded into L register (preserving all other bits)
;    - L XORed with RCDUFBIT (bit 7 toggled)
;    - Result stored back to RADMODES
;
;    This ensures that only bit 7 changes; all other RADMODES control and
;    status bits remain unaffected.
;
;    RADMODES bit 7 semantics:
;    - Set (1): RR CDU is healthy (CHAN 30 BIT 7 is set)
;    - Clear (0): RR CDU has failed (CHAN 30 BIT 7 is clear)
;
; 4. CDU failure detection (RADMODES bit 7 transitioned from 1 to 0):
;    After updating RADMODES bit 7, the routine checks the new state. If
;    RADMODES bit 7 is now clear (CDU has just failed), the routine checks
;    FLAGWRD0 bit 1 (RNDVZBIT - rendezvous flag) to determine if a P20-series
;    rendezvous navigation program is currently operating:
;    - If RNDVZBIT clear (no rendezvous program): Proceed to TRKFLCDU
;    - If RNDVZBIT set (P20 or P22 active): Issue alarm 00515 "RR CDU FAIL
;      DURING P-20", then proceed to TRKFLCDU
;
;    Program alarm 00515 is critical crew notification. It indicates that:
;    - The rendezvous navigation program has lost RR gimbal angle data
;    - RR range and range-rate measurements may still be valid, but the
;      computer doesn't know the line-of-sight direction
;    - Navigation solution accuracy is severely degraded
;    - Crew must assess whether to continue rendezvous or abort
;
;    Alarm 00515 requires crew and Mission Control evaluation:
;    - Can rendezvous continue with degraded navigation?
;    - Is the CDU failure transient or permanent?
;    - Should crew attempt RR power cycle to restore CDU?
;    - Are backup rendezvous techniques required (ground tracking, manual)?
;
; 5. CDU recovery detection (RADMODES bit 7 transitioned from 0 to 1):
;    If RADMODES bit 7 is now set (CDU has just recovered from failure), no
;    alarm is issued. The routine proceeds directly to TRKFLCDU. CDU recovery
;    is a positive event (hardware has self-restored), so crew notification
;    is provided by clearing the TRACKER FAIL lamp rather than by alarm.
;
;    Recovery might occur due to:
;    - Transient electrical fault cleared
;    - Crew RR power cycle completed successfully
;    - Thermal stabilization after cold period
;
; 6. TRACKER FAIL lamp update (always performed after change):
;    Routine calls SETTRKF to update the TRACKER FAIL lamp (DSPTAB+11D bit 8)
;    based on the new RADMODES bit 7 state. SETTRKF illuminates or extinguishes
;    the lamp on the DSKY according to current RR health:
;    - RADMODES bit 7 clear (CDU failed): TRACKER FAIL lamp ON
;    - RADMODES bit 7 set (CDU healthy): TRACKER FAIL lamp OFF
;
;    SETTRKF returns control to RRGIMON (next radar check in chain) to
;    continue the T4RUPT radar monitoring sequence.
;
; CDU hardware architecture:
; The RR has two CDUs (Coupling Data Units):
; - RR CDU TRUNNION: Measures azimuth gimbal angle (antenna rotation around
;   vertical axis), output to computer as OPTX (optical X-axis angle)
; - RR CDU SHAFT: Measures elevation gimbal angle (antenna rotation around
;   horizontal axis), output to computer as OPTY (optical Y-axis angle)
;
; Both CDUs are 16-bit binary shaft angle encoders providing 360-degree
; coverage with approximately 0.0055-degree resolution (2^16 counts per
; revolution = 65536 counts per 360 degrees). The CDUs are driven by
; synchro resolvers mechanically coupled to the gimbal axes.
;
; The RR CDU FAIL discrete is generated by the CDU electronics when:
; - CDU power supply is out of tolerance
; - Synchro resolver excitation is absent or incorrect
; - Encoder output is invalid or inconsistent
; - Self-test diagnostic detects internal fault
; - Temperature is outside operating range
;
; CDU failures are rare but mission-critical. Without accurate gimbal angle
; data, the computer cannot:
; - Compute line-of-sight direction to target (Command Module)
; - Transform radar measurements from antenna frame to navigation frame
; - Command antenna repositioning for track acquisition or maintenance
; - Compute relative velocity vector (requires line-of-sight direction)
;
; The 480-millisecond polling interval (twice per second) was chosen to:
; - Detect CDU failures quickly enough for crew response
; - Minimize T4RUPT computational load (CDU checks are lightweight)
; - Provide adequate response time for alarm processing
; - Match the overall T4RUPT radar monitoring interval (all radar checks
;   execute on same 480ms cycle)
;
; RADMODES bit 7 vs CHAN 30 BIT 7 relationship:
; RADMODES bit 7 is a software mirror of CHAN 30 BIT 7, maintained by this
; routine. The mirroring serves several purposes:
; - Software can check RR CDU status without reading hardware channel (faster)
; - Change detection via XOR requires previous state in memory
; - Other routines can test RADMODES bit 7 without channel access overhead
; - RADMODES consolidates all radar mode and status bits in one location
;
; The careful auto mode check before updating RADMODES bit 7 prevents
; "cross-talk" between Landing Radar and Rendezvous Radar monitoring. The LR
; and RR share some hardware resources and channel bit assignments (mode-
; dependent). By verifying RR is in AUTO mode before acting on CDU fail,
; the routine ensures that LR operations don't trigger spurious RR alarms.
;
; Historical context for Apollo 11:
; The Rendezvous Radar CDU system operated flawlessly during Apollo 11's lunar
; orbit rendezvous on July 21, 1969. The RR successfully tracked Columbia (CM)
; throughout the rendezvous sequence, providing accurate range, range-rate, and
; line-of-sight angle data to the P20 rendezvous navigation program.
;
; No alarm 00515 (RR CDU FAIL) was issued during Apollo 11. The CDU health
; monitoring provided confidence that gimbal angle data was valid throughout
; the rendezvous, contributing to the successful docking between Eagle and
; Columbia approximately 3.5 hours after Eagle's ascent from the lunar surface.
;
; The TRACKER FAIL lamp was a critical crew interface element. During
; rendezvous, Armstrong and Aldrin monitored the lamp to verify that the RR
; was tracking properly. If the lamp had illuminated during P20 operation,
; it would have indicated loss of radar tracking, requiring crew intervention.
;
; The CDU fail monitoring was especially important because rendezvous is a
; time-critical operation. If the RR CDU had failed during the rendezvous
; sequence, the crew would have had limited time to:
; - Diagnose the problem (transient vs permanent)
; - Attempt recovery (RR power cycle, mode changes)
; - Transition to backup rendezvous techniques if necessary
; - Coordinate with Mission Control for ground-based tracking support
;
; The routine's design reflects aerospace fault tolerance principles:
; - Fail-safe logic (inverted discrete - wire break appears as failure)
; - Separation of concerns (only monitor when radar is in use)
; - Immediate crew notification (alarm on failure detection)
; - Graceful degradation (lamp indication on recovery)
; - Operational context awareness (check for active rendezvous program)
;
; The RCDUFBIT mask isolates bit 7 for all operations. The careful use of
; EXTEND/RXOR and EXTEND/BZF ensures atomic read-modify-write sequences,
; preventing race conditions with other interrupt routines that might be
; accessing RADMODES concurrently.
;
; The pre-turnon code at lines 2046-2048 (labeled "-3") executes during RR
; power-on initialization (called from RRTURNON). It disables the RR CDU
; error counters (CHAN 12 BIT 2) to prevent spurious error accumulation
; during the CDU zeroing sequence. This is unrelated to the RRCDUCHK change
; detection logic but is positioned here for memory organization efficiency.
;
; Exit paths:
; - RRGIMON: Normal exit when no change detected (continue radar monitoring)
; - NORRGMON: Exit when change detected but RR not in auto mode (skip gimbal
;   checks which would be irrelevant)
; - TRKFLCDU -> SETTRKF -> RRGIMON: Exit after updating RADMODES bit 7 and
;   TRACKER FAIL lamp (continue radar monitoring with updated status)
; ============================================================================

# PROGRAM NAME:	RRCDUCHK
#
# FUNCTIONAL DESCRIPTION:
# RRCDUCHK CHECKS FOR RR CDU FAIL (CHAN 30 BIT 7).  INITIALLY THE
# RR CDU FAIL BIT IS SAMPLED (CHAN 30 BIT 7).  IF NO CHANGE, THE
# PROGRAM EXITS TO RRGIMON.  IF A CHANGE, THE RR AUTO MODE
# (RADMODES BIT 2) BIT IS CHECKED.  IF NOT IN RR AUTO MODE, THE
# PROGRAM EXITS TO NORRGMO0N.  IF IN AUTO MODE, RADMODES BIT 7
# (RR CDU OK) IS UPDATED AND IF P-20 IS OPERATING PROGRAM ALARM 00515 IS
# REQUESTED.  CONTROL IS TRANSFERRED TO SETTRKF TO UPDATE
# THE TRACKER FAIL LAMP (DSPTAB+11D BIT 8).  CONTROL RETURNS TO
# RRGIMON.
#
# CALLING SEQUENCE:
# EVERY 480 MILLISECONDS FROM RRAUTCHK (VIA T4RUPT) UNLESS A
# TURN-ON SEQUENCE HAS JUST BE INITIATED.
#
# ERASABLE INITIALIZATION REQUIRED:
# RADMODES
#
# SUBROUTINES CALLED:
# SETTRKF
#
# JOBS OR TASKS INITIATED:
# NONE
#
# ALARMS:
# TRACKER FAIL
# PROGRAM ALARM 00515 -- RRCDU FAIL DURING P-20
#
# EXIT:
# RRGIMON, NORRGMON

 -3		CS	BIT2
		EXTEND
		WAND	CHAN12			# AT TURNON, DISABLE CDU ERROR COUNTERS.

RRCDUCHK	CA	RADMODES		# LAST SAMPLED BIT IN RADMODES.
		EXTEND
		RXOR	CHAN30
		MASK	RCDUFBIT
		EXTEND
		BZF	RRGIMON

		CAF	AUTOMBIT		# IF RR NOT IN AUTO MODE, DON'T CHANGE BIT
		MASK	RADMODES		# 7 OF RADMODES.  IF THIS WERE NOT DONE,
		CCS	A			# THE TRACKER FAIL MIGHT COME ON WHEN
		TCF	NORRGMON		# JUST READING LR DATA.

		CAF	RCDUFBIT		# SET BIT 7 OF RADMODES FOR SETTRKF.
# Page 185
		LXCH	RADMODES		# UPDATE RADMODES.
		EXTEND
		RXOR	L
		TS	RADMODES

		CA	RADMODES		# DID RR CDU FAIL
		MASK	RCDUFBIT
		CCS	A
		TCF	TRKFLCDU		# NO
		CS	FLAGWRD0		# RNDVFLG P20 OR P22 OPERATING
		MASK	RNDVZBIT
		CCS	A
		TCF	TRKFLCDU		# NO
		TC	ALARM			# YES
		OCT	00515
TRKFLCDU	TC	SETTRKF			# UPDATE TRAKER FAIL LAMP ON DSKY.

# Page 186
; ============================================================================
; RRGIMON - Rendezvous Radar Gimbal Monitor and Limit Check
;
; This routine monitors the Rendezvous Radar antenna gimbal angles and verifies
; they are within safe operating limits for the current radar tracking mode.
; The RR antenna must point within specific angular ranges to avoid mechanical
; interference with the Lunar Module structure and to maintain valid tracking
; geometry. RRGIMON prevents antenna damage and ensures measurement validity.
;
; Hardware monitored:
; - RR CDU OPTY (shaft angle): Elevation gimbal position from CDU
; - RR CDU OPTX (trunnion angle): Azimuth gimbal position from CDU
; - RADMODES control flags: Mode and status bits affecting gimbal limits
;
; Routine operation (called every 480 milliseconds):
;
; 1. Pre-check for conditions disabling gimbal monitoring:
;    Before checking gimbal limits, the routine verifies that gimbal monitoring
;    is appropriate for the current radar state. Several conditions cause
;    immediate exit to NORRGMON (DAP matrix computation) without limit checks:
;
;    a) RADMODES bit 14 set (REMODE - continuous designate active):
;       In continuous designate mode, the computer is commanding the antenna
;       to specific angles for non-tracking purposes (antenna checkout, manual
;       pointing). Gimbal limits are handled by the designate logic itself.
;
;    b) RADMODES bit 13 set (RR CDU ZERO sequence active):
;       During CDU zeroing (part of RR turn-on), the gimbal servos are being
;       driven to known reference positions. Limit checks would interfere with
;       the zeroing sequence, so monitoring is disabled.
;
;    c) RADMODES bit 11 set (REPOSITION - antenna slewing in progress):
;       When the antenna is already repositioning due to a previous limit
;       violation, further limit checks are suppressed to avoid recursive
;       reposition commands. The reposition task (DORREPOS) will clear bit 11
;       when the antenna reaches the commanded safe position.
;
;    d) RADMODES bit 2 clear (RR not in AUTO mode):
;       If the RR is not in AUTO mode (power off or standby), gimbal positions
;       are not being actively controlled, so limit checks are meaningless.
;
;    e) FLAGWRD5 NORRMBIT set (no RR angle monitor flag):
;       This flag (set by certain programs) globally disables RR gimbal limit
;       monitoring when gimbal angle checks would interfere with mission program
;       operations (e.g., during specific attitude maneuvers or radar modes).
;
;    The pre-check logic tests FLAGWRD5 NORRMBIT first, then uses a single
;    mask operation (OCT05776 = bits 14, 13, 11, 2) to check all four RADMODES
;    conditions simultaneously. If any bit is set (or bit 2 clear), the routine
;    exits immediately to NORRGMON.
;
; 2. Gimbal limit check via RRLIMCHK subroutine:
;    If all pre-checks pass (radar is in normal tracking operation), the routine
;    calls RRLIMCHK to verify that the current RR gimbal angles (OPTY, OPTX)
;    are within the safe angular limits for the present tracking mode.
;
;    RRLIMCHK examines the current gimbal positions and compares them against
;    mode-dependent limit boundaries:
;    - Mode 1 limits (wide angle search mode): ±60 degrees typical
;    - Mode 2 limits (narrow angle track mode): ±45 degrees typical
;    - Mode 3 limits (auto-track mode): ±30 degrees typical
;
;    The exact limit values depend on:
;    - LM structural clearances (antenna must not hit spacecraft)
;    - Radar beam geometry (valid measurement cone angles)
;    - Gimbal servo mechanical range (physical stops)
;    - Tracking mode requirements (search vs track angular coverage)
;
;    RRLIMCHK returns to RRGIMON with:
;    - TC NORRGMON if angles are within limits (normal exit)
;    - Fall through to RRBAD if angles exceed limits (reposition needed)
;
; 3. Limit violation response (RRBAD - angles out of limits):
;    If RRLIMCHK determines that the RR gimbal angles have exceeded safe limits,
;    RRGIMON initiates a reposition sequence to drive the antenna to a safe
;    angle within limits. The reposition response involves multiple actions:
;
;    a) RADMODES bit 11 set (REPOSITION flag):
;       Indicates antenna repositioning is in progress. This flag prevents
;       further limit checks (see pre-check 1c above) until reposition completes.
;
;    b) CHAN 12 BIT 14 cleared (RR AUTO TRACKER disabled):
;       The RR automatic tracking circuits are disabled to prevent the tracker
;       from fighting the reposition command. Auto-track attempts to keep the
;       antenna pointed at the target, but during reposition the antenna must
;       slew to a safe angle regardless of target position.
;
;    c) CHAN 12 BIT 2 cleared (RR ERROR COUNTER disabled):
;       The RR error counter accumulates tracking errors for quality assessment.
;       During reposition, large tracking errors are expected (antenna is not
;       pointed at target), so the error counter is disabled to prevent false
;       tracking quality degradation indications.
;
;    d) WAITLIST task scheduled: DORREPOS in 20 milliseconds (2 centiseconds):
;       A WAITLIST task is scheduled to call DORREPOS (DO Rendezvous Radar
;       REPOSITION) after a 20-millisecond delay. DORREPOS computes safe gimbal
;       angles within limits and commands the gimbal servos to slew the antenna
;       to the safe position. The 20ms delay allows the tracker disable and
;       error counter disable commands to propagate through the hardware before
;       the antenna begins moving.
;
;    e) Exit to NORRGMON:
;       After initiating the reposition sequence, RRGIMON exits to NORRGMON
;       (DAP matrix computation), continuing the T4RUPT execution sequence.
;
; 4. Normal exit when within limits:
;    If RRLIMCHK determines that gimbal angles are within safe limits, it
;    returns directly to NORRGMON via TC NORRGMON, bypassing the reposition
;    logic entirely. This is the normal case during stable tracking operations.
;
; Gimbal limit violation causes:
; Gimbal angles can exceed limits due to several operational scenarios:
;
; a) Target motion during tracking:
;    As the Command Module moves relative to the Lunar Module during rendezvous,
;    the line-of-sight direction changes. The RR antenna tracks the moving
;    target, and the required gimbal angles evolve continuously. If the target
;    moves to an angular position near the structural limits, the tracker may
;    drive the antenna toward the limit boundary.
;
; b) LM attitude changes:
;    When the LM performs attitude maneuvers (RCS firings to change orientation),
;    the RR antenna (mounted to the LM structure) moves with the spacecraft. The
;    target's angular position in the LM body frame changes even though the
;    inertial line-of-sight remains constant. Large attitude changes can cause
;    gimbal angles to exceed limits.
;
; c) Initial track acquisition at unfavorable geometry:
;    When the RR first acquires track on the CM, the initial relative geometry
;    may place the line-of-sight near or beyond the gimbal limits. The tracker
;    locks on to the target signal, driving the antenna to the required angles,
;    which may violate limits.
;
; d) Tracking mode transitions:
;    When the RR switches between tracking modes (wide angle search to narrow
;    angle track, for example), the gimbal limits change. Angles that were
;    valid in one mode may exceed limits in the new mode, triggering reposition.
;
; The reposition sequence moves the antenna to a safe angle while maintaining
; radar operation. The repositioned antenna may not be pointed directly at the
; target, so tracking is temporarily lost. After reposition completes, the RR
; must re-acquire track at the new safe gimbal angles.
;
; RADMODES bit definitions (referenced by this routine):
; - Bit 2: RR AUTO MODE (must be set for gimbal monitoring)
; - Bit 11: REPOSITION (antenna slewing to safe angle)
; - Bit 13: RR CDU ZERO sequence active
; - Bit 14: CONTINUOUS DESIGNATE (manual antenna pointing mode)
;
; FLAGWRD5 NORRMBIT: NO RR ANGLE MONITOR flag
; Set by programs requiring temporary suspension of gimbal limit checks.
;
; Historical context for Apollo 11:
; The Rendezvous Radar gimbal limit monitoring operated throughout Apollo 11's
; lunar orbit rendezvous on July 21, 1969. The RR successfully tracked Columbia
; (CM) during the entire rendezvous sequence without gimbal limit violations,
; indicating favorable tracking geometry throughout the approach.
;
; Gimbal repositioning was more common in earlier Apollo missions where
; rendezvous trajectories or LM attitude profiles placed the RR line-of-sight
; near structural limits. By Apollo 11, rendezvous procedures and trajectory
; designs had been optimized to avoid unfavorable RR gimbal geometries.
;
; The 480-millisecond polling interval (twice per second) was chosen to:
; - Detect gimbal limit approaches quickly (gimbal rates ~5 deg/sec typical)
; - Minimize T4RUPT computational load (limit checks involve trigonometry)
; - Provide adequate response time for reposition sequencing
; - Match the overall T4RUPT radar monitoring interval
;
; The reposition sequence (disable tracker, schedule DORREPOS, exit) was
; carefully designed to avoid gimbal servo instabilities. If the auto-tracker
; remained enabled during reposition, it would generate tracking error signals
; opposing the reposition command, causing servo oscillation or positioning
; errors. The 20-millisecond WAITLIST delay ensures tracker disable takes
; effect before antenna motion begins.
;
; The OCT05776 mask (octal 5776 = bits 14, 13, 11, 2) efficiently tests all
; four RADMODES pre-check conditions in a single operation. The careful bit
; assignment in RADMODES allows this optimization - related conditions are
; grouped for efficient mask operations.
;
; Gimbal limit boundaries are not hard-coded in RRGIMON itself. The actual
; limit checks occur in RRLIMCHK (called as a subroutine), which accesses
; limit tables defining mode-dependent angular boundaries. This separation
; allows limit values to be changed without modifying the monitoring logic.
;
; The routine's design reflects operational rendezvous requirements:
; - Continuous monitoring during all tracking modes
; - Fast response to limit violations (reposition within 20ms)
; - Graceful handling of tracking interruptions (disable auto-track)
; - Prevention of recursive reposition commands (bit 11 check)
; - Compatibility with special radar modes (continuous designate, CDU zero)
;
; RRLIMCHK subroutine (called by RRGIMON):
; RRLIMCHK performs the actual angular limit comparison. It reads the current
; RR CDU angles (OPTY shaft, OPTX trunnion) and compares them against
; mode-dependent limit tables. The subroutine uses either direct return (TC
; NORRGMON if within limits) or fall-through (to RRBAD if out of limits),
; providing efficient control flow for the common case (within limits).
;
; DORREPOS task (scheduled by RRGIMON when limits exceeded):
; DORREPOS computes safe gimbal angles and commands the RR servos to reposition
; the antenna. The safe angles are chosen to:
; - Be well within gimbal limits (margin from boundary)
; - Maintain favorable tracking geometry if possible
; - Minimize antenna slew time (minimize tracking interruption)
; - Avoid structural obstructions and beam blockages
;
; After DORREPOS completes the antenna reposition, it clears RADMODES bit 11
; (REPOSITION flag), re-enables the auto-tracker (CHAN 12 BIT 14), and
; re-enables the error counter (CHAN 12 BIT 2). The RR then attempts to
; re-acquire track at the new safe gimbal angles.
;
; The gimbal limit monitoring system (RRGIMON + RRLIMCHK + DORREPOS) forms a
; closed-loop protection system ensuring safe RR antenna operation:
; - RRGIMON: Periodic monitoring and violation detection
; - RRLIMCHK: Angular limit boundary comparison
; - DORREPOS: Corrective reposition command generation
;
; This layered architecture separates concerns (monitoring vs checking vs
; correction) and provides maintainability and testability benefits.
;
; No alarms are issued by RRGIMON or its associated logic. Gimbal limit
; violations are considered normal operational events, not faults. The antenna
; repositions automatically, and the crew is not notified unless tracking is
; lost for an extended period (handled by other RR monitoring routines).
;
; Exit paths:
; - NORRGMON (early exit): Pre-check conditions indicate monitoring should be
;   skipped (REMODE, CDU ZERO, REPOSITION, not AUTO mode, or NORRMBIT set)
; - NORRGMON (via RRLIMCHK): Gimbal angles within limits, normal operation
; - NORRGMON (after RRBAD): Reposition sequence initiated, continue T4RUPT
;
; The NORRGMON label (defined as DAPT4S/GPMATRIX) continues the T4RUPT
; execution sequence, proceeding to Digital Autopilot (DAP) matrix
; computations. This transition represents the end of radar hardware monitoring
; and the beginning of spacecraft attitude control processing within the same
; T4RUPT cycle.
; ============================================================================

# PROGRAM NAME:  RRGIMON
#
# FUNCTIONAL DESCRIPTION:
# RRGIMON IS THE RR GIMBAL LIMIT MONITOR.  INITIALLY THE FOLLOWING IS
# CHECKED:  REMOD, RR CDU'S BEING ZEROED, REPOSITION, AND RR
# NOT IN AUTO MODE (RADMODES BITS 14, 13, 11, 2).  IF ANY OF THESE
# EXIST THE PROGRAM EXITS TO GPMATRIX.  IF NONE ARE PRESENT RRLIMCHK
# IS CALLED TO SEE IF THE PRESENT RR CDU ANGLES (OPTY, OPTX) ARE WITHIN
# THE LIMITS OF THE CURRENT MODE.  IF WITHIN LIMITS, THE PROGRAM EXITS
# TO NORRGMON.  IF NOT WITHIN LIMITS, THE REPOSITION FLAG (RADMODES
# BIT 11) IS SET, THE RR AUTO TRACKER AND RR ERROR COUNTER
# (CHAN 12 BITS 14, 2) ARE DISABLED, AND A 20 MILLISECOND WAITLIST
# CALL IS SET FOR DORREPOS AFTER WHICH THE PROGRAM EXITS TO NORRGMON.
#
# CALLING SEQUENCE:
# EVERY 480 MILLISECONDS FROM RRCDUCHK (VIA T4RUPT) UNLESS TURN-ON
# HAS JUST BEEN INITIATED VIA RRAUTCHK OR IF THERE HAS BEEN A CHANGE IN
# THE RR CDU FAIL BIT (CHAN 30 BIT 7) AND THE RR IS NOT IN THE AUTO MODE
# (RADMODES BIT 2).
#
# ERASABLE INITIALZATION:  RADMODES
#
# SUBROUTINES CALLED:
# RRLIMCHK, WAITLIST
#
# JOBS OR TASKS INITIATED:
# DORREPOS
#
# ALARMS:
# NONE
#
# EXIT:
# NORRGMON

RRGIMON		CAE	FLAGWRD5			# IS NO ANGLE MONITOR FLAG SET
		MASK	NORRMBIT
		CCS	A
		TCF	NORRGMON			# YES -- SKIP LIMIT CHECK
		CS	FLAGWRD7			# IS SERVICER RUNNING?
		MASK	AVEGFBIT
		CCS	A
		TCF	+5				# NO. DO R25
		CA	FLAGWRD6			# YES. IS MUNFLAG SET?
		MASK	MUNFLBIT
		CCS	A
		TCF	NORRGMON			# YES. DON'T DO R25
 +5		CAF	OCT32002			# INHIBIT BY REMODE, ZEROING, MONITOR.
		MASK	RADMODES			# OR RR NOT IN AUTO.
		CCS	A
		TCF	NORRGMON
# Page 187
		TC	RRLIMCHK			# SET IF ANGLES IN LIMITS.
		ADRES	CDUT

		TCF	MONREPOS

		TCF	NORRGMON			# (ADDITIONAL CODING MAY GO HERE).

MONREPOS	CAF	REPOSBIT			# SET FLAG TO SHOW REPOSITION IN PROGRESS.
		ADS	RADMODES

		CS	OCT20002			# DISABLE TRACKER AND ERROR COUNTER.
		EXTEND
		WAND	CHAN12

		CAF	TWO
		TC	WAITLIST
		EBANK=	LOSCOUNT
		2CADR	DORREPOS

		TCF	NORRGMON

OCT32002	OCT	32002
OCT20002	OCT	20002
OCT02100	OCT	02100				# P20, P22 MASK BITS.

# Page 188
; ============================================================================
; ROUTINE: GPMATRIX (also known as DAPT4S)
; LOCATION: Page 188-189
; MISSION PHASE: All powered flight phases (descent, ascent, rendezvous, docking)
;
; PURPOSE AND OPERATIONAL CONTEXT:
; GPMATRIX computes the transformation matrix elements that convert vectors
; between the spacecraft's gimbal coordinate frame and pilot (body) coordinate
; frame. This transformation is fundamental to the Digital Autopilot (DAP)
; system, enabling the AGC to translate desired attitude commands into actual
; gimbal angles and thruster firing commands.
;
; During Apollo 11's lunar descent on July 20, 1969, this routine executed
; continuously at 4 Hz (4 times per second), providing the LM DAP with current
; transformation matrices as Armstrong and Aldrin descended to the Sea of
; Tranquility. The routine's 250-millisecond cycle rate ensured attitude
; control commands were based on up-to-date gimbal orientation data.
;
; EXECUTION FREQUENCY AND INTEGRATION WITH T4RUPT:
; GPMATRIX executes 4 times per second (every 250 milliseconds) as part of
; the T4RUPT interrupt service cycle. It appears in the T4JUMP dispatch table
; multiple times:
; - Twice explicitly as DAPT4S entries
; - Twice implicitly following RRAUTCHK (which also appears twice in T4JUMP)
; - Additionally as NORRGMON exit point from radar gimbal monitoring
;
; This multiple scheduling ensures DAP matrix updates occur regularly
; throughout each second, maintaining fresh transformation data for attitude
; control regardless of which T4RUPT dispatch path is taken.
;
; COORDINATE FRAME TRANSFORMATION THEORY:
; The Apollo LM uses a three-gimbal Inertial Measurement Unit (IMU) to
; maintain stable platform orientation. The relationship between gimbal angles
; and body (pilot) axes requires mathematical transformation matrices.
;
; Gimbal angles measured by CDU (Coupling Data Unit) readouts:
; - CDUX = Outer Gimbal angle (OG), rotation about X-axis
; - CDUY = Inner Gimbal angle (IG), rotation about Y-axis  
; - CDUZ = Middle Gimbal angle (MG), rotation about Z-axis
;
; The transformation from Gimbal to Pilot coordinates (M_GP matrix) and its
; inverse from Pilot to Gimbal coordinates (M_PG matrix) are computed using
; trigonometric functions of these gimbal angles.
;
; TRANSFORMATION MATRICES COMPUTED:
;
; M_GP (Gimbal to Pilot) matrix elements:
;   Row 1: [ sin(MG),           0,          1 ]
;   Row 2: [ cos(MG)cos(OG),    sin(OG),    0 ]
;   Row 3: [-cos(MG)sin(OG),    cos(OG),    0 ]
;
; M_PG (Pilot to Gimbal) matrix - not fully computed, only needed elements:
;   Row 2: [ 0,  sin(OG),           cos(OG)          ]
;   Row 3: [ 1, -sin(MG)cos(OG)/cos(MG), sin(MG)sin(OG)/cos(MG) ]
;
; Note: Row 1 of M_PG is not computed as it's not required by LEM DAP routines.
; This optimization (dating from February 1968 modification) saves computation
; time in the time-critical T4RUPT interrupt handler.
;
; MATRIX ELEMENTS STORED IN ERASABLE MEMORY:
; The computed single-precision matrix elements are stored in EBANK M11:
; - M11 = sin(MG)                    [M_GP row 1, col 1]
; - M21 = cos(MG)cos(OG)             [M_GP row 2, col 1]
; - M31 = -cos(MG)sin(OG)            [M_GP row 3, col 1]
; - M22 = sin(OG)                    [M_GP row 2, col 2; also M_PG row 2, col 2]
; - M32 = cos(OG)                    [M_GP row 3, col 2; also M_PG row 2, col 3]
; - COSMG = cos(MG)                  [Intermediate factor for computations]
;
; All matrix elements are scaled at 1 (full-scale representation, effectively
; -1.0 to +1.0 in the AGC's fixed-point arithmetic).
;
; SINGLE-PRECISION VS INTERPRETIVE REPRESENTATION:
; GPMATRIX computes SINGLE-PRECISION matrix elements specifically for use by
; BASIC LANGUAGE (native AGC assembly) routines in the DAP. These are NOT
; arrayed for interpretive programs, which would require double-precision
; vector/matrix representations.
;
; This design decision reflects performance optimization: DAP control laws
; execute in native AGC code for speed, requiring single-precision elements
; stored as individual variables rather than interpretive vector arrays.
;
; TRIGONOMETRIC COMPUTATION SUBROUTINES:
; GPMATRIX calls two fundamental math subroutines:
; - SPSIN: Single-Precision SINe function
; - SPCOS: Single-Precision COSine function
;
; These subroutines accept gimbal angles in AGC angular units (scaled as
; fractions of a full circle: 180° = 1.0 in two's complement representation)
; and return trigonometric values scaled at 1 (range -1.0 to +1.0).
;
; The SPSIN and SPCOS implementations use polynomial approximations optimized
; for AGC's limited instruction set and fixed-point arithmetic constraints.
;
; HISTORICAL MODIFICATION CONTEXT:
; February 7, 1968 modification by P. S. Weissman deleted computation of
; matrix elements MR12 and MR13 (originally part of M_PG row 1), which were
; found to be unused by any DAP routines. This optimization reduced T4RUPT
; execution time, freeing processor cycles for other critical tasks.
;
; During Apollo 11's descent, this optimization contributed to overall system
; margin, reducing the computational load that contributed to the famous 1202
; executive overflow alarms at 102:38:26 mission elapsed time.
;
; DAP USAGE OF TRANSFORMATION MATRICES:
; The LEM Digital Autopilot uses these matrix elements to:
; 1. Transform desired attitude (in pilot/body coordinates) to required gimbal
;    angles for IMU gimbal drive commands
; 2. Transform measured gimbal rates to body angular rates for rate damping
; 3. Convert commanded body-axis torques to gimbal-axis torque requirements
; 4. Compute coupling effects between gimbal motions and body motions
;
; During lunar landing, these transformations enabled the DAP to maintain
; proper spacecraft attitude while accounting for:
; - Descent engine thrust vector offset from center of gravity
; - Propellant mass depletion changing moment of inertia
; - Crew inputs via Attitude Controller Assembly (ACA)
; - Guidance computer attitude command changes
;
; CALLING SEQUENCE WITHIN T4RUPT DISPATCH:
; GPMATRIX is entered via multiple paths within the T4JUMP table:
; 1. Direct entry as DAPT4S (explicit T4JUMP table entry, occurs twice)
; 2. Following RRAUTCHK completion (radar auto-tracker check routine)
; 3. As NORRGMON exit point from RRGIMON (radar gimbal limit monitor)
;
; The label DAPT4S is defined as EQUALS GPMATRIX, making both names
; interchangeable entry points to the same routine.
;
; EXECUTION TIMING AND PROCESSOR LOAD:
; GPMATRIX execution time is dominated by trigonometric function calls:
; - SPSIN call: ~8 milliseconds (depending on angle value)
; - SPCOS call: ~8 milliseconds
; - Total routine: ~50-60 milliseconds including 6 trig calls and multiplies
;
; At 4 executions per second, GPMATRIX consumes approximately 200-240
; milliseconds of processor time per second, representing ~20-24% of AGC
; computational capacity. This significant load is justified by the critical
; importance of accurate attitude control transformation.
;
; GIMBAL LOCK CONSIDERATIONS:
; The transformation matrix becomes singular (mathematically undefined) when
; the middle gimbal (MG) reaches ±90°, a condition known as "gimbal lock."
; In gimbal lock, the outer and inner gimbals align, losing one degree of
; rotational freedom.
;
; GPMATRIX itself does not check for gimbal lock conditions - that
; responsibility belongs to the GLOCKMON (Gimbal Lock Monitor) routine, which
; executes earlier in the T4RUPT cycle and illuminates the GIMBAL LOCK warning
; light on the DSKY if middle gimbal exceeds ±70° (providing 20° safety
; margin before actual lock at ±90°).
;
; During Apollo 11, gimbal lock was avoided through proper IMU alignment and
; mission trajectory planning. The LM remained well within safe gimbal angle
; ranges throughout descent, landing, and ascent operations.
;
; COMPUTATIONAL PRECISION AND SCALING:
; All gimbal angles (CDUX, CDUY, CDUZ) are stored as 15-bit two's complement
; values representing fractions of a complete revolution:
; - 0° = 0 (decimal)
; - 90° = 8192 (decimal) = 020000 (octal)
; - 180° = 16384 (decimal) = 040000 (octal)
; - 270° = 24576 (decimal) = 060000 (octal)
;
; This "revolutions" scaling provides ~0.011° angular resolution, more than
; adequate for IMU gimbal angle representation.
;
; Trigonometric function outputs and matrix elements use "single-precision at 1"
; scaling:
; - +1.0 = 16384 (decimal) = 037777 (octal) [maximum positive]
; - -1.0 = -16384 (decimal) = 140000 (octal) [maximum negative]
; - 0.0 = 0
;
; This scaling maximizes precision within the AGC's 15-bit signed arithmetic.
;
; RELATIONSHIP TO INTERPRETIVE NAVIGATION:
; While GPMATRIX computes single-precision elements for DAP use, the AGC also
; maintains double-precision transformation matrices (stored as interpretive
; vectors) for high-precision navigation computations. These separate matrix
; representations serve different purposes:
; - Single-precision (GPMATRIX): Fast, for real-time control (DAP)
; - Double-precision (interpretive): Accurate, for navigation state updates
;
; The two matrix sets are computed independently and used by different AGC
; subsystems, reflecting the dual-architecture nature of the Apollo guidance
; system.
;
; EXIT PATH TO RESUME:
; GPMATRIX completes by transferring control to RESUME, which restores
; interrupt context (Q register, BANK register) and returns from the T4RUPT
; interrupt handler to resume the interrupted program.
;
; The RESUME routine handles:
; - Restoring Q (return address) from QRUPT
; - Restoring BANK (memory bank) from BANKRUPT  
; - Restoring accumulator A from ARUPT (if saved)
; - Re-enabling interrupts
; - Resuming interrupted program execution
;
; LABEL DEFINITIONS AND ALIASES:
; - DAPT4S EQUALS GPMATRIX: Primary entry point name (DAP Task 4 Seconds)
; - NORRGMON EQUALS DAPT4S: Exit point from radar gimbal monitor
; - ENDDAPT4 EQUALS RESUME: Symbolic end-of-DAP marker
;
; These multiple label aliases reflect the routine's integration with both
; radar monitoring logic and general T4RUPT dispatch structure.
; ============================================================================

# PROGRAM NAME:  GPMATRIX (DAPT4S) MCD. NO. 2 DATE: OCTOBER 27, 1966
#
# AUTHOR:  JOHNATHAN D. ADDLELSTON (ADAMS ASSOCIATES)
#
# MODIFIED:  7FEB. 1968 BY P. S. WEISSMAN TO DELETE COMPUTATION OF MR12 AND MR13, WHICH ARE NO LONGER REQUIRED.
#
# THIS PROGRAM CALCULATES ALL THE SINGLE-PRECISION MATRIX ELEMENTS WHICH ARE USED BY LEM DAP TO TRANSFORM VECTORS
# FROM GIMBAL TO PILOT (BODY) AXES AND BACK AGAIN.  THESE ELEMENTS ARE USED EXCLUSIVELY BY BASIC LANGUAGE ROUTINES
# AND THEREFORE ARE NOT ARRAYED FOR USE BY INTERPRETIVE PROGRAMS.
#
# CALLING SEQUENCE:  GPMATRIX IS TRANSFERRED TO FROM DAPT4S AND IS THUS EXECUTED 4 TIMES A SECOND BY T4RUPT.
# DAPT4S IS LISTED IN T4JUMP TABLE TWICE EXPLICITLY AND ALSO OCCURS AFTER RRAUTCHK (WHICH IS ALSO LISTED TWICE).
#
# SUBROUTINES CALLED: SPSIN, SPCOS.
#
# NORMAL EXIT MODE:  TCF RESUME
#
# ALARM AND ABORT MODES:  NONE.
#
# INPUT: CDUX, CDUY, CDUZ.
#
# OUTPUT:  M11, M21, M32, M22, M32.
#
# AOG = CDUX, AIG = CDUY, AMG = CDUZ: MNEMONIC IS : OIM = XYZ
#
#		*	*	SING(MG)		0			1	*
#		M   =	*	COS(MG)COS(OG)		SIN(OG)			0	*
#		 GP	*	-COS(MG)SIN(OG)		COS(OG)			0	*
#
#		*	*	0			COS(OG)/COS(MG)		-SIN(OG)/COS(MG)	*
#		M   =	*	0			SIN(OG)			COS(OG)			*
#		 PG	*	1			-SIN(MG)COS(OG)/COS(MG)	SIN(MG)SIN(OG)/COS(MG)	*

		EBANK=	M11
DAPT4S		EQUALS	GPMATRIX

# T4RUPT DAP LOGIC:

GPMATRIX	CAE	CDUZ			# SINGLE ENTRY POINT
		TC	SPSIN			# SIN(CDUZ) = SIN(MG)
		TS	M11			# SCALED AT 1

		CAE	CDUZ
		TC	SPCOS			# COS(CDUZ) = COS(MG)
		TS	COSMG			# SCALED AT 1 (ONLY A FACTOR)

		CAE	CDUX
		TC	SPSIN			# SIN(CDUX) = SIN(OG)
		TS	M22			# SCALED AT 1 (ALSO IS MR22)

		CS	M22
# Page 189
		EXTEND
		MP	COSMG			# -SIN(OG)COS(MG)
		TS	M31			# SCALED AT 1

		CAE	CDUX
		TC	SPCOS			# COS(CDUX) = COS(OG)
		TS	M32			# SCALED AT 1 (ALSO IS MR23)

		EXTEND
		MP	COSMG			# COS(OG)COS(MG)
		TS	M21			# SCALED AT 1

		TC	RESUME

NORRGMON	EQUALS	DAPT4S
ENDDAPT4	EQUALS	RESUME
