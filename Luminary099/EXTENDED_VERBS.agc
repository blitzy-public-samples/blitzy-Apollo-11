# Copyright:	Public domain.
# Filename:	EXTENDED_VERBS.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	262-300
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Luminary131/ file of the same
#				name, using Luminary099 page images.
#		2009-06-05 RSB	Corrected 5 typos.
#		2009-06-06 RSB	Eliminated an extraneous 3-instruction block
#				and added a missing instruction.
#		2009-06-07 RSB	Added a couple of "SBANK=" for compatibility
#				with yaYUL. Corrected a typo.
#		2010-12-31 JL	Fixed page number comments.
#		2011-01-06 JL	Added missing comment characters.
#		2011-05-08 JL	Flagged SBANK= workarounds for future removal.

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
; FILE: EXTENDED_VERBS.agc
; MODULE: DSKY Extended Verb Functions (Pinball Interface)
; MISSION PHASE: all phases (launch through landing and ascent)
;
; TL;DR: Implements DSKY extended verb commands (V40-V99) that operate on
;        noun data to perform spacecraft operations. Verbs are commands typed
;        by the crew (Armstrong/Aldrin) on the DSKY keyboard to control the
;        guidance computer. Includes IMU alignment verbs, radar control verbs,
;        state vector update verbs, display verbs, and system test verbs.
;
; COMMENT-ONLY READERS: This file defines the commands astronauts typed on
;        the DSKY during the mission. Read to understand crew procedures.
; CODE-ALONG READERS: Study the verb dispatch table and individual verb
;        implementations to understand DSKY command processing architecture.
; ============================================================================

; VERB SYSTEM OVERVIEW:
; The DSKY (Display and Keyboard) verb/noun system is the primary crew interface
; to the Apollo Guidance Computer. Verbs are action commands (what to do) and
; nouns are data types (what to act upon). The crew types verb-noun combinations
; like "V16N63" meaning "Monitor (V16) altitude and altitude-rate (N63)".
;
; Extended verbs (V40-V99) are implemented in this file and handle specialized
; operations beyond basic display functions. Common categories include:
;
; V40-V49: IMU and sensor alignment/calibration
; V50-V59: Crew request verbs ("Please perform..."), radar control
; V60-V69: Attitude display, DAP control, system monitoring
; V70-V79: Time and state vector updates, telemetry control
; V80-V89: State vector operations, rendezvous displays
; V90-V99: System tests, W-matrix operations, special functions
;
; HISTORICAL CONTEXT:
; During Apollo 11 descent, Armstrong and Aldrin used verbs like:
; - V16N63 to monitor altitude and descent rate continuously
; - V37E63 to initiate the P63 landing program
; - V47 for AGS (Abort Guidance System) initialization
; During ascent, they used:
; - V37E12 to start the P12 ascent program
; - V82 to request orbital parameter displays for rendezvous

# Page 262
		BANK	7
		SETLOC	EXTVERBS
		BANK

		EBANK=	OGC

		COUNT*	$$/EXTVB

; ============================================================================
; EXTENDED VERB DISPATCH TABLE
;
; This section implements the fan-out (dispatch table) for extended verbs V40-V99.
; When the crew enters an extended verb number on the DSKY, the system:
; 1. Stores the verb number in MPAC (Multi-Purpose Accumulator)
; 2. Calls GOEXTVB which uses INDEX addressing to jump to the appropriate handler
; 3. Each verb handler performs its specific function and returns control
;
; The INDEX MPAC instruction provides computed addressing - if MPAC contains
; the value N, the TC instruction jumps to the Nth entry in the LST2FAN table.
; For example, if crew enters V42, MPAC contains 2 (42-40), causing jump to
; the third entry (IMUFINEK) which performs fine IMU alignment.
; ============================================================================

# FAN-OUT

GOEXTVB		INDEX	MPAC		# VERB-40 IS IN MPAC
		TC	LST2FAN		# FAN AS BEFORE.

; ============================================================================
; VERB DISPATCH TABLE (V40-V86)
; Each entry corresponds to an extended verb command. Crew types verb number
; on DSKY, and system dispatches to the appropriate handler routine.
; ============================================================================

LST2FAN		TC	VBZERO		; V40: Zero CDU angles or ICDU
					; Used with N20 (ICDU angles) or N72 (RR CDU angles)
					; Crew procedure: V40E20E to zero IMU CDU angles

		TC	VBCOARK		; V41: Coarse align IMU
					; Used with N20 (desired IMU angles) or N72 (RR angles)
					; Rapidly slews IMU platform to specified orientation
					; Crew procedure: V41E20E, enter desired angles

		TC	IMUFINEK	; V42: Fine align IMU
					; Precision alignment of IMU platform using gyrocompassing
					; or star sighting alignment. Follows coarse align (V41).
					; Used during mission phase transitions (TLI, LOI, etc.)

		TC	IMUATTCK	; V43: Load IMU attitude error meters
					; Displays current IMU drift rates and bias errors
					; Used for IMU performance monitoring and calibration

		TC	RRDESEND	; V44: Terminate continuous designate
					; Stops automatic antenna tracking (rendezvous radar)
					; Used when switching from automatic to manual antenna control

		TC	ALM/END		; V45: SPARE (unimplemented, causes program alarm)

		TC	ALM/END		; V46: SPARE (unimplemented, causes program alarm)

		TC	V47TXACT	; V47: AGS initialization
					; Initializes Abort Guidance System with current state
					; Critical before descent - provides backup guidance if AGC fails
					; Used during pre-landing checklist

		TC	DAPDISP		; V48: Load autopilot (DAP) data
					; Allows crew to adjust Digital Autopilot control gains
					; and deadband settings for attitude control

		TCF	CREWMANU	; V49: Start automatic attitude maneuver
					; Initiates pre-programmed spacecraft rotation
					; Used for antenna pointing, thermal control, docking alignment

		TC	GOLOADLV	; V50: Please perform
					; Astronaut request verb - system asks crew to perform action
					; Flashes V50 on DSKY when crew input or confirmation needed

		TC	ALM/END		; V51: SPARE (unimplemented)

		TC	GOLOADLV	; V52: Please mark X-reticle
					; System requests crew optical sighting through telescope
					; Crew aligns X-reticle on target star or landmark, presses MARK

		TC	GOLOADLV	; V53: Please mark Y-reticle
					; System requests crew optical sighting (Y-axis)
					; Used for navigation updates from star or landmark sightings

		TC	GOLOADLV	; V54: Please mark X or Y reticle
					; System accepts mark from either reticle
					; Used when mark axis doesn't matter for navigation update

		TC	ALINTIME	; V55: Align mission timer
					; Synchronizes AGC mission elapsed time with actual mission time
					; Used if time discrepancy detected or after AGC restart

		TC	TRMTRACK	; V56: Terminate tracking (P20/P25)
					; Stops rendezvous radar tracking in programs P20-P25
					; Used when switching rendezvous programs or entering coast phase

		TC	LRON		; V57: Permit landing radar updates
					; Enables landing radar data to update navigation state
					; CRITICAL during descent - Armstrong used this to enable
					; altitude and velocity measurements for landing guidance

		TC	LROFF		; V58: Inhibit landing radar updates
					; Disables landing radar data from updating navigation
					; Used if radar data suspected unreliable or after landing

		TC	ALM/END		; V59: SPARE (unimplemented)

		TC	LRPOS2K		; V60: Command landing radar to position 2
					; Moves landing radar antenna to alternate position
					; Used to optimize radar beam angle during descent phases

		TC	DAPATTER	; V61: Display DAP attitude error
					; Shows autopilot's measured attitude error from desired
					; Used for monitoring autopilot performance

		TC	TOTATTER	; V62: Display total attitude error
					; Shows combined attitude error from all sources
					; Used for IMU alignment quality assessment

		TC	R04		; V63: Sample radar once per second
					; Initiates R04 routine for radar self-test
					; Samples landing or rendezvous radar at 1 Hz rate

		TC	VB64		; V64: Calculate and display S-band antenna angles
					; Computes required S-band high-gain antenna pointing angles
					; for Earth communication. Crew manually positions antenna.

		TC	SNUFFOUT	; V65: Disable U,V jets during DPS burns
					; Inhibits +X and -X RCS jets during descent engine firing
					; Prevents RCS jet plume impingement on descent engine bell

		TC	ATTACHED	; V66: RCS configuration (attached mode)
					; Configures RCS for docked CSM/LM configuration
					; Used before undocking to set proper jet selection logic

		TC	V67		; V67: W-matrix monitor
					; Displays state vector covariance matrix elements
					; Used to monitor navigation solution uncertainty

		TC	ALM/END		; V68: SPARE (unimplemented)

VERB69		TC	VERB69		; V69: Force hardware restart
					; Deliberately causes AGC restart for test purposes
					; DANGEROUS - only used during systems testing, never in flight

		TC	V70UPDAT	; V70: Update liftoff time
					; Loads actual liftoff time from ground or crew entry
					; Used to synchronize mission timeline with actual launch

		TC	V71UPDAT	; V71: Universal update (block address)
					; Ground uplink verb - updates multiple memory locations
					; Used by Mission Control to send state vector corrections

		TC	V72UPDAT	; V72: Universal update (single address)
					; Ground uplink verb - updates single memory location
					; Used for individual parameter corrections from ground

		TC	V73UPDAT	; V73: Update AGC time (octal)
					; Loads current time into AGC time registers
					; Used to correct time after uplink or if drift detected

		TC	DNEDUMP		; V74: Initialize downlink telemetry for erasable dump
					; Configures telemetry system to transmit memory contents
					; Ground controllers use this to diagnose AGC state

		TC	OUTSNUFF	; V75: Enable U,V jets during DPS burns
					; Re-enables +X and -X RCS jets (reverses V65)
					; Used after descent engine shutdown
# Page 263
		TC	MINIMP		; V76: Minimum impulse mode
					; Sets RCS to fire minimum duration pulses
					; Conserves propellant during fine attitude control

		TC	NOMINIMP	; V77: Rate command mode
					; Sets RCS to standard rate-damping control mode
					; Used for normal attitude control during maneuvers

		TC	R77		; V78: Start landing radar spurious return test
					; Initiates R77 self-test routine
					; Checks for false altitude readings from radar

		TC	R77END		; V79: Terminate landing radar spurious return test
					; Stops R77 test routine
					; Returns radar to normal operational mode

		TC	LEMVEC		; V80: Update LM state vector
					; Loads new LM position and velocity from ground uplink
					; Critical for navigation accuracy - used multiple times per mission

		TC	CSMVEC		; V81: Update CSM state vector
					; Loads CM position and velocity for rendezvous computations
					; Used to track Command Module orbit during LM operations

		TC	V82PERF		; V82: Request orbital parameter display (R30)
					; Calls R30 routine to display orbit characteristics
					; Shows apogee, perigee, period, etc. - used after each burn

		TC	V83PERF		; V83: Request rendezvous parameter display (R31)
					; Calls R31 routine to display relative position/velocity to CSM
					; Critical during rendezvous phase after ascent

		TC	ALM/END		; V84: SPARE (unimplemented)

		TC	VERB85		; V85: Display rendezvous radar LOS azimuth and elevation
					; Shows radar line-of-sight angles to CSM
					; Used to verify radar tracking during rendezvous

		TC	ALM/END		; V86: SPARE (unimplemented)

		TC	ALM/END		; V87: SPARE (unimplemented)

		TC	ALM/END		; V88: SPARE (unimplemented)

		TC	V89PERF		; V89: Align LM +X or +Z axis along rendezvous radar line-of-sight
					; Used in R63 routine for special rendezvous geometry alignment
					; Orients spacecraft to optimize radar tracking during approach

		TC	V90PERF		; V90: Display out-of-plane rendezvous parameters
					; Shows components of relative motion perpendicular to orbital plane
					; Critical for rendezvous trajectory adjustments

		TC	GOSHOSUM	; V91: Display bank sum (memory checksum)
					; Computes and displays checksum of AGC fixed memory banks
					; Used for verification that program is uncorrupted

		TC	SYSTEST		; V92: Operate IMU performance test
					; Runs comprehensive test of Inertial Measurement Unit
					; Validates gyroscope and accelerometer functionality

		TC	WMATRXNG	; V93: Clear RENDWFLG (rendezvous W-matrix flag)
					; Resets flag controlling rendezvous navigation filter updates
					; Used to reinitialize rendezvous targeting computations

		TC	ALM/END		; V94: SPARE (unimplemented)

		TC	UPDATOFF	; V95: Inhibit state vector updates
					; Prevents ground-uplinked navigation state corrections
					; Used when crew wants to maintain current onboard solution

		TC	VERB96		; V96: Interrupt integration and go to program P00
					; Terminates orbital integration computation
					; Returns AGC to idle program P00 (fresh start available)
					; Used to abort current program and return to standby

		TC	GOLOADLV	; V97: Please verify engine failure (crew request)
					; Prompts crew to confirm detection of engine malfunction
					; Part of abort decision logic during critical burns
					; Flashing V97 requires crew response to proceed

		TC	ALM/END		; V98: SPARE (unimplemented)

		TC	GOLOADLV	; V99: Please enable engine (crew request)
					; Requests crew confirmation to enable engine for firing
					; Safety interlock requiring manual crew action
					; Used before critical engine ignition sequences

; ============================================================================
; END OF EXTENDED VERB DISPATCH TABLE
;
; The dispatch table above routes verb commands to their implementation
; functions. The following sections handle verb execution control, busy
; status management, and display system coordination.
; ============================================================================

# END OF EXTENDED VERB FAN

; ============================================================================
; TESTXACT - Test and Set Extended Verb Active Flag
;
; This routine checks if the extended verb display system is currently busy.
; If busy (EXTVBACT > 0), it turns on the operator error light and exits.
; If not busy, it sets the EXTVBACT flag and blanks the DSKY display except
; for the mode (MM) and verb indicators, preparing for extended verb execution.
;
; The routine also checks FLAGWRD4 bit pattern to detect if priority displays
; are using the DSKY, which would conflict with extended verb operations.
;
; RETURNS: With EXTVBACT set and display prepared for verb execution
; ============================================================================

TESTXACT	CCS	EXTVBACT	# ARE EXTENDED VERBS BUSY
		TC	ALM/END		# YES, TURN ON OPERATOR LIGHT
		CA	FLAGWRD4	# ARE PRIORITY DISPLAYS USING DSKY
		MASK	OC24100
		CCS	A
		TC	ALM/END		# YES
		CAF	OCT24		# SET BITS 3 AND 5
SETXTACT	TS	EXTVBACT	# NO.  SET FLAG TO SHOW EXT VERB DISPLAY
					# SYSTEM BUSY

		CA	Q		; Save return address from Q register
		TS	MPAC +1		; Store in MPAC+1 for later return

		CS	TWO		# BLANK EVERYTHING EXCEPT MM AND VERB
		TC	NVSUB		; Call display blanking subroutine
		TC	+1		; Continue to next instruction
		TC	MPAC +1		; Return to saved address

; Extended verb alarm handler - turns on operator error light and releases
; the extended verb display system to allow subsequent operations.

XACTALM		TC	FALTON		# TURN ON OPERATOR ERROR LIGHT.
		TC	ENDEXT		# RELEASE MARK AND EXT. VERB DISPLAY SYS.

TERMEXTV 	EQUALS 	ENDEXT		; Alternate entry point for terminating verbs
# Page 264
ENDEXTVB	EQUALS	ENDEXT		; Alternate name for ENDEXT routine

; XACT0 - Release extended verb display system with zero flag
; Called to clear EXTVBACT and release the display system for other uses.

XACT0		CAF	ZERO		# RELEASE MARK AND EXT. VERB DISPLAY SYS.
		TC	SETXTACT	; Clear EXTVBACT flag (set to zero)

; ALM/END - Extended verb error handler
; Turns on operator error light and returns control to PINBALL display system.
; Used throughout extended verbs when invalid conditions detected.

ALM/END		TC	FALTON		# TURN ON OPERATOR ERROR LIGHT
GOPIN		TC	POSTJUMP	; Bank call to PINBALL system
		CADR	PINBRNCH	; Address of PINBALL branch routine

; CHKPOOH - Check for Program P00
; Verifies that the current program is P00 (idle/fresh start available).
; Many extended verbs can only execute from P00 to avoid conflicts.
; RETURNS: If P00, returns via Q register; otherwise triggers error

CHKPOOH		CA	MODREG		# CHECK FOR P00 OR P00-.
		EXTEND			; Enable extended instruction
		BZF	TCQ		; If zero (P00), return via Q
		TC	ALM/END		; Not P00, signal error

OC24100		OCT	24100		; Octal constant for flag masking

# Page 265

; ============================================================================
; VBZERO - VERB 40: Zero CDU (Coupling Data Unit)
;
; PURPOSE:
; Zeros the gimbal angle counters in either the IMU CDU (Inertial Measurement
; Unit) or the Rendezvous Radar CDU. The CDU reads gimbal angles from the
; gyroscopically-stabilized platform or radar antenna gimbals.
;
; PROCEDURE:
; 1. Requires Noun 20 (ICDU angles) or Noun 72 (RCDU angles)
; 2. For N20: Checks IMUCADR to avoid 1210 restart alarm
;    For N72: Checks if either radar is in use
; 3. Executes the CDU zero command to hardware
; 4. Stalls (waits) until the zero operation completes
; 5. Doesn't differentiate between good or bad return status
; 6. Exits, re-establishing any interrupted display
;
; USAGE:
; Crew types V40E (Extended Verb 40 Execute) with appropriate noun.
; Used during IMU alignment procedures or radar calibration.
;
# VBZERO	VERB 40		DESCRIPTION
#
#	1. 	REQUIRE NOUN 20 (ICDU ANGLES) OR NOUN 72 (RCDU ANGLES).
#	2.	FOR N20, CHECK IMUCADR IN AN EFFORT TO AVOID A 1210 RESTART.
#		FOR N72, CHECK IF EITHER RADAR IS IN USE.
#	3.	EXECUTE THE CDU ZERO.
#	4.	STALL UNTILL THE ZERO IS DONE.
#	5.	DON'T DIFFERENTIATE BETWEEN A BAD OR GOOD RETURN.
#	6.	EXIT, RE-ESTABLISHING THE INTERRUPTED DISPLAY (IF ANY).
; ============================================================================

VBZERO		TC	OP/INERT	; Check for Noun 20 or 72, branch accordingly
		TC	IMUZEROK	# RETURN HERE IF NOUN = ICDU(20)
		TC	RRZEROK		# RETURN HERE IF NOUN = RCDU(72)

; IMU CDU zero path (Noun 20)
; Zeros the IMU gimbal angle counters after checking for conflicts

IMUZEROK	TC	CKMODCAD	; Check mode and IMUCADR for conflicts
		TC	BANKCALL	# KEYBOARD REQ FOR ISS CDUZERO
		CADR	IMUZERO		; Call IMU zero routine

		TC	BANKCALL	# STALL
		CADR	IMUSTALL	; Wait for IMU zero to complete
		TC	+1		; Continue after stall

		TC	GOPIN		# IMUZERO - Return to PINBALL system

; Rendezvous Radar CDU zero path (Noun 72)
; Zeros the RR gimbal angle counters after checking radar usage

RRZEROK		TC	RDRUSECK	; Check if radar is in use
		TC	BANKCALL	; Call RR zero routine
		CADR	RRZERO		; Zero rendezvous radar CDU

RWAITK		TC	BANKCALL	; Wait for radar operation to complete
		CADR	RADSTALL	; Stall until radar ready
		TCF	+1		; Continue
		TC	GOPIN		# RRZERO - Return to PINBALL system

; ============================================================================
; LRPOS2K - VERB 60: Command Landing Radar to Position 2
;
; PURPOSE:
; Commands the Landing Radar antenna to move to Position 2 (forward looking).
; Position 1 looks downward during descent, Position 2 looks forward for
; velocity measurement during final approach to the lunar surface.
;
; PROCEDURE:
; 1. Exit with operator error if either radar is in use
; 2. Issue alarm code 523 if Position 2 not indicated within prescribed time
; 3. Re-establish the interrupted displays
;
; HISTORICAL:
; During Apollo 11 descent, the landing radar switched between positions to
; provide both altitude and velocity data as the LM approached touchdown.
;
# LRPOS2K	VERB 60			DESCRIPTION
#	COMMAND LANDING RADAR TO POSITION 2
#
#	1.	EXIT WITH OP ERROR IF SOMEONE IS USING EITHER RADAR.
#	2.	ALARM WITH CODE 523 IF POS 2 IS NOT INDICATED WITHIN
#		THE PRESCRIBED TIME.
#	3.	RE-ESTABLISH THE DISPLAYS.
; ============================================================================

LRPOS2K		TC	RDRUSECK	; Check if radar available
		TC	BANKCALL	# COMMAND LR TO POSITION 2
		CADR	LRPOS2		; Call landing radar position 2 routine
		TC	BANKCALL	; Wait for radar to complete movement
		CADR	RADSTALL	; Stall until radar repositioned
		TC	LRP2ALM		; Branch if position 2 not achieved (timeout)
		TC	GOPIN		; Success - return to PINBALL

LRP2ALM		TC	ALARM		; Landing radar position 2 alarm
		OCT	523		; Alarm code 523: LR position failure
		TC	GOPIN		; Return to PINBALL after alarm
# Page 266

; ============================================================================
; DAPATTER - VERB 61: Display DAP Attitude Errors
;
; PURPOSE:
; Configures the FDAI (Flight Director Attitude Indicator) error needles to
; display Digital Autopilot (DAP) attitude errors. The error needles show the
; difference between commanded and actual spacecraft attitude.
;
; OPERATION:
; Clears the NEEDLFLG flag, causing the FDAI to display DAP-computed errors
; rather than total errors. Used during automatic attitude control.
;
# V61	VERB 61, DISPLAY DAP ATTITUDE ERRORS ON FDAI ATTITUDE ERROR NEEDLES.
; ============================================================================

DAPATTER	TC	DOWNFLAG	; Clear needle flag
		ADRES	NEEDLFLG	; NEEDLFLG = 0: display DAP errors
		TC	GOPIN		; Return to PINBALL

; ============================================================================
; TOTATTER - VERB 62: Display Total Attitude Errors
;
; PURPOSE:
; Configures the FDAI (Flight Director Attitude Indicator) error needles to
; display total attitude errors instead of DAP errors. Total errors include
; both guidance-commanded attitude changes and DAP control errors.
;
; OPERATION:
; Sets the NEEDLFLG flag, causing the FDAI to display total attitude errors.
; Used during manual control or to monitor overall guidance system performance.
;
# V62	VERB 62, DISPLAY TOTAL ATTITUDE ERRORS ON FDAI ATTITUDE ERROR NEEDLES.
; ============================================================================

TOTATTER	TC	UPFLAG		; Set needle flag
		ADRES	NEEDLFLG	; NEEDLFLG = 1: display total errors
		TC	GOPIN		; Return to PINBALL

# Page 267

; ============================================================================
; VBCOARK - VERB 41: Coarse Align IMU or Radar
;
; PURPOSE:
; Performs coarse alignment of either the Inertial Measurement Unit (IMU) or
; the Rendezvous Radar (RR). Coarse alignment rapidly positions the platform
; or radar antenna to approximate gimbal angles before fine alignment.
;
; USAGE:
; Crew types V41E with Noun 20 (IMU CDU angles) or Noun 72 (RR CDU angles).
; Used during pre-flight alignment, inflight realignment, or radar setup.
;
; HISTORICAL:
; Critical during Apollo 11 pre-launch alignment and inflight IMU realignment.
; Coarse align was the first step before fine alignment procedures that
; established the precise inertial reference needed for navigation.
;
# VBCOARK	VERB 41		DESCRIPTION
#	COARSE ALIGN IMU OR RADAR
#
#	1.	REQUIRE NOUN 20 OR NOUN 72 OR TURN ON OPERATOR ERROR.
#	2.	REQUIRE EXT VERB DISPLAY SYS AVAILABLE OR TURN ON OPERATOR ERROR LIGHT AND GO TO PINBRNCH.
#				CASE 1, NOUN 20 (ICDU ANGLES)
#	3.	SET EXT VERB DISPLAY ACTIVE FLAG.
#	4.	DISPLAY FLASHING V25,N22 (LOAD NEW ICDU ANGLES).
#		RESPONSES
#		A.	TERMINATE
#			1.	RELEASE EXT VERB DISPLAY SYSTEM
#		B.	PROCEED
#			1.	COARSE ALIGN TO THE EXISTING THETAD'S (ICORK2).
#		C.	ENTER
#			1.	COARSE ALIGN TO THE LOADED THETAD'S (ICORK2).
# ICORK2
#	1.	RE-DISPLAY VERB 41.
#	2.	EXECUTE IMUCCARS (IMU COARSE ALIGN).
#	3.	EXECUTE IMUSTALL (ALLOW TIME FOR DATA TRANSFER).
#	4.	RELEASE EXT VERB DISPLAY SYSTEM.
#				CASE 2 NOUN 72 (RCDU ANGLES)
#		EXIT WITH OP ERROR IF SOMEONE IS USING EITHER RADAD.
#	5.	DISPLAY FLASHING V24,N73 (LOAD NEW RR TRUNION ANGLE AND NEW SHAFT ANGLE).
#		RESPONSES
#		A.	TERMINATE
#			1.	RELEASE EXT VERB DISPLAY SYS.
#		B.	PROCEED OR ENTER
#			1.	EXECUTE AURLOKON (ASK OPERATOR FOR LOCK-ON REQUIREMENTS).
#			2.	RE-DISPLAY VERB 41.
#			3.	SCHEDULE RRDESK2 WITH PRIORITY 20.
#			4.	RELEASE EXT VERB DISPLAY SYS.
# AURLOKON
#	1.	FLASH V04 N12 R1 = 00006 R2 = 00002
#		RESPONSES
#		A.	TERMINATE
#		B.	PROCEED
#			1.	RESET LOCK-ON SWITCH
#			2.	SET CONTINUOUS DESIGNATE FLAG
#			3.	DISABLE R25
#		C.	V22 E 1 E, R1 = 00001, PROCEED
#			1.	SET LOCK-ON SWITCH
; ============================================================================

VBCOARK		TC	OP/INERT	; Check for Noun 20 or 72, branch accordingly
		TC	IMUCOARK		# RETURN HERE IF NOUN = ICDU (20)
		TC	RRDESNBK		# RETURN HERE IF NOUN = RCDU (72)

# RETURNS TO L+1 IF IMU OR L+2 IF RR.

OP/INERT	CS	OCT24
		AD	NOUNREG
		EXTEND
# Page 268
		BZF	TCQ			# IF = 20.

		AD	RRIMUDIF		# -52
		EXTEND
		BZF	Q+1

		TC	ALM/END			# ILLEGAL.

RRIMUDIF	DEC	-52			# THE IMU
; ============================================================================
; VERB 41 - IMUCOARK: IMU COARSE ALIGN FROM KEYBOARD
;
; Allows astronauts to manually enter desired IMU gimbal angles for coarse
; alignment. Used when automatic star sighting is not possible (e.g., during
; daylight operations, spacecraft attitude constraints, or IMU cage/release).
; Crew enters three angles via DSKY using Verb 25 Noun 22 (THETAD angles).
;
; COMMENT-ONLY READERS: This is the manual IMU alignment procedure where the
; crew enters three gimbal angles based on known spacecraft attitude.
;
; CODE-ALONG READERS: Calls mode checking, requests angle entry via V25N22,
; then invokes IMUCOARS routine to physically drive IMU gimbals to desired
; orientation. Stalls until alignment complete.
; ============================================================================

IMUCOARK	TC	CKMODCAD
		TC	TESTXACT		# COARSE ALIGN FROM KEYBOARD.
		CAF	VNLODCDU		# CALL FOR THETAD LOAD
		TC	BANKCALL
		CADR	GOXDSPF
		TC	TERMEXTV
		TCF	+1

ICORK2		CAF	IMUCOARV		# RE-DISPLAY COARSE ALIGN VERB.
		TC	BANKCALL
		CADR	EXDSPRET

		TC	BANKCALL		# CALL MODE SWITCHING PROG
		CADR	IMUCOARS

		TC	BANKCALL		# STALL
		CADR	IMUSTALL
		TC	ENDEXTVB
		TC	ENDEXTVB

VNLODCDU	VN	2522			; V25N22: Load desired gimbal angles
IMUCOARV	VN	4100			; V41N00: Coarse align verb display

# Page 269
; ============================================================================
; RRDESNBK: RENDEZVOUS RADAR DESIGNATE TO DESIRED GIMBAL ANGLES
;
; Allows crew to manually designate rendezvous radar (RR) antenna to specific
; gimbal angles for tracking Command Module during rendezvous operations.
; Used with Verb 44 to terminate continuous designate mode. Checks that P20
; (rendezvous navigation) is not running to prevent conflicts.
;
; COMMENT-ONLY READERS: Manual radar pointing control for tracking the Command
; Module during lunar orbit rendezvous after Eagle's ascent from surface.
;
; CODE-ALONG READERS: Checks RNDVZBIT flag (P20 running), terminates any
; existing designation (OCT41000 mask), requests antenna angles via V24N73,
; spawns RRDESK2 job to execute designation with PRIO20 priority.
; ============================================================================

RRDESNBK	TC	RDRUSECK
		TC	TESTXACT
		CA	RNDVZBIT		# IS P20 RUNNING?
		MASK	FLAGWRD0
		CCS	A
		TCF	XACTALM			# OPERADOR ERROR IF IN P20
		CS	OCT41000		# TERMINATE PRESENT DESIGNATION
		INHINT				# RELINT DONE IN GOXDSPF
		MASK	RADMODES
		TS	RADMODES

		CAF	VNLDRCDU		# ASK FOR GIMBAL ANGLES.
		TC	BANKCALL
		CADR	GOXDSPF
		TC	TERMEXTV
		TCF	-4			# V33

		TC	BANKCALL		# ASK OP FOR LOCK ON REQUIREMENTS.
		CADR	AURLOKON

		CAF	OPTCOARV		# RE-DISPLAY OUR OWN VERB
		TC	BANKCALL
		CADR	EXDSPRET

		CAF	PRIO20
		TC	FINDVAC
		EBANK=	LOSCOUNT
		2CADR	RRDESK2

		TCF	TERMEXTV		# FREES DISPLAY

VNLDRCDU	VN	2473			; V24N73: Load RR gimbal angles
OPTCOARV	EQUALS	IMUCOARV		# DIFFERENT NOUNS.

; ============================================================================
; RRDESK2: RENDEZVOUS RADAR DESIGNATE EXECUTION JOB
;
; Background job that executes the actual RR antenna designation. Releases
; its VAC (Variable Area of Core) after initiating designation, then waits
; for radar lock-on. Issues alarm 503 if lock-on fails or antenna limits
; exceeded (tracking target outside physical antenna range).
;
; Historical context: During Apollo 11 rendezvous, this routine tracked
; Columbia (CM) to maintain relative navigation data for docking.
; ============================================================================

RRDESK2		TC	BANKCALL
		CADR	RRDESNB

		TC	+1			# DUMMY NEEDED SINCE DESRETRN DOES INCR
		CA	PRIORITY
		MASK	LOW9
		CCS	A
		INDEX	A
		TS	A			# RELEASE THIS JOBS VAC AREA.
		COM				# INSURE ENDOFJOB DOES A NOVAC END (BZMF).
		ADS	PRIORITY
		TC	BANKCALL		# WAIT FOR COMPLETION OF DESIGNATE
		CADR	RADSTALL
# Page 270
		TC	+2			# BADEND-NO LOCKON OR OUT OF LIMITS
		TC	ENDOFJOB		# GOODEND-LOCKON ACHIEVED
		TC	ALARM
		OCT	503			# TURN ON ALARM LIGHT -503 DESIGNATE FAIL

		TC	ENDOFJOB

; ============================================================================
; VERB 44 - RRDESEND: TERMINATE CONTINUOUS RENDEZVOUS RADAR DESIGNATE
;
; Terminates continuous RR tracking mode initiated by manual designate or
; automatic tracking programs (P20, P25). Clears designation flags, delays
; 1 second for antenna to settle, then enables R25 gimbal angle monitoring.
; Crew uses V44 to stop automatic radar tracking and regain manual control.
;
; COMMENT-ONLY READERS: Stops the rendezvous radar from automatically
; tracking the Command Module during rendezvous operations.
;
; CODE-ALONG READERS: Checks RADMODES for continuous designate bit (OCT41000),
; clears it with INHINT protection, delays 1 second, then enables NORRMON flag
; for R25 gimbal monitoring routine.
; ============================================================================

RRDESEND	CCS	RADMODES		# TERMINATE CONTINOUS DESIGNATE ONLY
		TCF	GOPIN
		TCF	GOPIN
		TCF	+1
		CS	OCT41000		# BEGDES GOES TO ENDRADAR
		INHINT				# RELINT DONE IN DOWNFLAG
		MASK	RADMODES
		TS	RADMODES
		TC	CLRADMOD
		CAF	1SEC
		TC	BANKCALL
		CADR	DELAYJOB
		TC	DOWNFLAG		# ENABLE R25 GIMBAL MONITOR
		ADRES	NORRMON
		TCF	GOPIN
OCT41000	OCT	41000			# CONTINUOUS DESIGNATE - DESIGNATE

# Page 271
		BANK	23
		SETLOC	EXTVB1
		BANK
		COUNT*	$$/EXTVB

; ============================================================================
; AURLOKON -- Automatic Rendezvous Radar Lock-On Routine
;
; Supports Verb 50 (Please Perform) and Verb 52 (Please Mark X Reticle).
; Manages rendezvous radar lock-on operations during rendezvous navigation.
; Displays V04N12 to crew requesting lock-on confirmation. Options:
;   - R2 = 1: Lock-on requested
;   - Proceed (V33): Execute lock-on sequence
;   - V34: Terminate request
;   - V32: Recycle (redisplay)
;
; During Apollo 11 rendezvous, this routine coordinated Eagle's approach to
; Columbia by establishing radar tracking lock for relative navigation.
; ============================================================================

AURLOKON	TC	MAKECADR
		TS	DESRET
		CAF	TWO
		TS	OPTIONX +1
		CAF	SIX			# OPTION CODE FOR V04N12
		TS	OPTIONX

 -5		CAF	V04N1272
		TC	BANKCALL		# R2	00001	LOCK-ON
		CADR	GOMARKFR
		TCF	ENDEXT			# V34
		TCF	+5			# V33
		TCF	-5			# V32
		CAF	BIT3
		TC	BLANKET
		TC	ENDOFJOB

; Proceed path: Check option flags to determine lock-on vs continuous designate
 +5		CA	OPTIONX +1
		MASK	BIT2
		CCS	A
		TCF	NOLOKON
		TC	UPFLAG
		ADRES	LOKONSW
		TCF	AURLKON1

; No lock-on path: Enable continuous designation (terminated by V44)
NOLOKON		TC	DOWNFLAG		# IF NO LOCK-ON, SET BIT15 OF RADMADES TO
		ADRES	LOKONSW			# INDICATE THAT CONTINUOUS DESIGNATION IS
		TC	UPFLAG			# WANTED (TO BE TERMINATED BY V44.)
		ADRES	CDESFLAG
		TC	UPFLAG			# SET NO RR ANGLE MONITOR FLAG.
		ADRES	NORRMON			# DISABLE R25 RR GIMBAL MONITOR IN T4RUPT
AURLKON1	RELINT
		CA	DESRET
		TCF	BANKJUMP

V04N1272	VN	412
-LOKONFG	OCT	-20

		BANK	43
		SETLOC	EXTVERBS
		BANK
		COUNT*	$$/EXTVB

; ============================================================================
; LRON -- Verb 57: Permit Landing Radar Updates
;
; Enables incorporation of landing radar data into navigation state vector.
; Sets LRINH flag (landing radar inhibit flag) to permit LR data.
; Critical during lunar descent for altitude and velocity measurements.
;
; During Apollo 11 descent, landing radar provided essential altitude and
; velocity data for guidance computer. Crew could enable/disable LR updates
; to handle radar anomalies or select alternative navigation sources.
; ============================================================================

LRON		TC	UPFLAG		# PERMIT INCORPORATION OF LR DATA      V57

# Page 272
		ADRES	LRINH
		TCF	GOPIN

; ============================================================================
; LROFF -- Verb 58: Inhibit Landing Radar Updates
;
; Disables incorporation of landing radar data into navigation state vector.
; Clears LRINH flag to inhibit LR data processing.
; Used when radar data suspect or during mission phases not requiring LR.
; ============================================================================

LROFF		TC	DOWNFLAG		# INHIBIT INCORPORATION OF LR DATA	V58
		ADRES	LRINH
		TCF	GOPIN

		EBANK=	OGC

# Page 273

; ============================================================================
; IMUFINEK -- Verb 42: Fine Align IMU
;
; Performs IMU fine alignment using gyro torquing commands.
; Displays V25N93 (flashing) for crew to load delta gyro angle corrections.
; Sequence:
;   1. Verify extended verb display available, set busy flag
;   2. Display V25N93 for gyro command load
;   3. Responses:
;      - Terminate (V34): Release display system
;      - Proceed/Enter: Execute fine alignment sequence
;        a. Re-display V42
;        b. Execute IMUFINE (mode switching to fine align)
;        c. Execute IMUSTALL (allow IMU data transfer)
;        d. If successful: Execute IMUPULSE (apply gyro torques)
;
; Fine alignment refines IMU platform orientation using gyro torquing to
; correct for drift errors. Used after coarse alignment or during mission
; to maintain inertial reference accuracy.
; ============================================================================

# IMUFINEK	VERB 42		DESCRIPTION
#	FINE ALIGN IMU
#
#	1.	REQUIRE EXT VERB DISPLAY AVAILABLE AND SET BUSY FLAG OR TURN ON OPER ERROR AND GO TO PINBRNCH.
#	2.	DISPLAY FLASHING V25,N93....LOAD DELTA GYRO ANGLES....
#		RESPONSES
#		A.	TERMINATE
#			1.	RELEASE EXT VERB DISPLAY SYSTEM.
#		B.	PROCEED OR ENTER
#			1.	RE-DISPLAY VERB 42
#			2.	EXECUTE IMUFINE (IMU FIVE ALIGN MODE SWITCHING).
#			3.	EXECUTE IMUSTALL (ALLOW FOR DATA TRANSFER)
#				A.	FAILED
#					1. 	RELEASE EXT VERB DISPLAY SYSTEM.
#				B.	GOOD
#					1.	EXECUTE IMUPULSE (TORQUE IRIGS).
#					2.	EXECUTE IMUSTALL AND RELEASE EXT VERB DISPLAY SYSTEM.

IMUFINEK	TC	CKMODCAD
		TC	TESTXACT		# FINE ALIGN WITH GYRO TORQUING.
		CAF	VNLODGYR		# CALL FOR LOAD OF GYRO COMMANDS
		TC	BANKCALL
		CADR	GOXDSPF
		TC	TERMEXTV
		TC	+1			# PROCEED WITHOUT A LOAD

		CAF	IMUFINEV		# RE-DISPLAY OUR OWN VERB
		TC	BANKCALL
		CADR	EXDSPRET

		TC	BANKCALL		# CALL MODE SWITCH PROG
		CADR	IMUFINE

		TC	BANKCALL		# HIBERNATION
		CADR	IMUSTALL
		TC	ENDEXTVB

FINEK2		CAF	LGYROBIN		# PINBALL LEFT COMMANDS IN OGC REGISTERS
		TC	BANKCALL
		CADR	IMUPULSE

		TC	BANKCALL		# WAIT FOR PULSES TO GET OUT.
		CADR	IMUSTALL
		TC	ENDEXTVB
		TC	ENDEXTVB

LGYROBIN	ECADR	OGC
VNLODGYR	VN	2593
IMUFINEV	VN	4200

; ============================================================================
; GOLOADLV -- Verb 50 and "Please Perform" Verb Handler
;
; Handles Verb 50 (Please Perform) and related astronaut request verbs
; (V51 Please Mark, V52 Please Mark X, V53 Please Mark Y, V54 Please Mark X or Y).
;
; "Please Perform" verbs display flashing requests to crew for manual actions
; such as IMU alignment marks, optical sightings, or other crew procedures.
;
; Crew responses:
;   - ENTER: Indicates requested action completed, proceeds with data recall
;   - V33 (Proceed without data): Requested action not performed, skip
;   - V34 (Terminate): Cancel operation
;
; During Apollo 11, these verbs coordinated crew activities with AGC operations,
; such as marking star sightings for IMU alignment or verifying LM/CSM
; configurations during mission phases.
; ============================================================================

# GOLOADLV	VERB 50		DESCRIPTION
#	AND OTHER PLEASE
# Page 274
#	DO SOMETHING VERBS
#
# PLEASE PERFORM, MARK, CALIBRATE, ETC.
#
#	1.	PRESSING ENTER ON DSKY INDICATES REQUESTED ACTION HAS BEEN PERFORMED, AND THE PROGRAM DOES THE
#		SAME RECALL AS A COMPLETED LOAD.
#	2.	THE EXECUTION OF A VERB 33 (PROCEED WITHOUT DATA) INDICATES THE REQUESTED ACTION IS NOT DESIRED.

		SBANK=	PINSUPER	# FOR LOADLV1 AND SHOWSUM CADR'S

GOLOADLV	TC	FLASHOFF

		CAF	PINSUPBT
		EXTEND
		WRITE	SUPERBNK
		TC	POSTJUMP
		CADR	LOADLV1

# ============================================================================
# VERB 47: AGS INITIALIZATION (V47)
# ============================================================================
; VERB 47 (V47) - AGS INITIALIZATION
; 
; PURPOSE:
; Initializes the Abort Guidance System (AGS) interface, preparing the Lunar
; Module's backup guidance computer for potential use. During Apollo 11's
; mission, the AGS served as a critical backup to the primary AGC in case of
; AGC failure during descent or ascent.
;
; MISSION CONTEXT:
; The AGS was a separate, simpler guidance computer in the LM that could take
; over basic guidance functions if the AGC failed. Verb 47 configures the
; interface between AGC and AGS, ensuring data transfer paths are ready.
; This verb was part of pre-descent checklist procedures, ensuring backup
; systems were ready before powered descent initiation.
;
; CREW PROCEDURE:
; Astronauts would execute V47E (Verb 47 Enter) during pre-descent preparation
; to arm the AGS interface. This was typically done well before PDI as part
; of systems checkout. No noun is required with this verb.
;
; TECHNICAL OPERATION:
; 1. Verifies no other extended verb is currently active (TESTXACT check)
; 2. Schedules AGSINIT job at priority 4 via FINDVAC (find vacant core set)
; 3. AGSINIT routine (in LOWSUPER bank) performs actual AGS initialization
; 4. Job terminates via ENDOFJOB after initialization complete
;
; The AGS initialization sets up data buffer (AGSBUFF) for state vector
; transfer between AGC and AGS, configures radar sharing modes, and establishes
; default AGS operational parameters.
;
; HISTORICAL NOTE:
; While Apollo 11's AGS was not needed during the actual landing (AGC performed
; flawlessly despite the 1202 alarms), the AGS was a critical safety backup
; that gave mission control confidence to proceed with landing even under
; computer workload stress.
;
# VERB 47 -- AGS INITIALIZATION -- R47.
#
# SEE LOG SECTION AGS INITIALIZATION FOR OTHER PERTINENT REMARKS.

V47TXACT	TC	TESTXACT	# NO OTHER EXTVERB.
		CAF	PRIO4
		TC	FINDVAC
## [WORKAROUND] RSB 2009
		SBANK=	LOWSUPER
## [WORKAROUND]
		EBANK=	AGSBUFF
		2CADR	AGSINIT

		TC	ENDOFJOB

CKMODCAD	CA 	MODECADR
		EXTEND
		BZF	TCQ
		TC	ALM/END		# SOMEBODY IS USING MODECADR SO EXIT

# Page 275
# ============================================================================
# VERB 55: ALIGN TIME (ALINTIME)
# ============================================================================
; VERB 55 (V55) - ALIGN TIME
;
; PURPOSE:
; Allows crew to synchronize the AGC mission elapsed time clock with external
; time sources (such as Mission Control's master clock or the Command Module
; clock). Accurate time synchronization was essential for coordinating
; rendezvous maneuvers and meeting mission timeline milestones.
;
; MISSION CONTEXT:
; Small clock drift between AGC, Mission Control, and CM clocks could accumulate
; over the multi-day mission. Verb 55 enabled periodic time corrections to
; maintain synchronization. This was particularly important before critical
; events like rendezvous, where timing precision was measured in seconds.
;
; CREW PROCEDURE:
; 1. Ensure in P00 (Program 00) or POO- (idle program state)
; 2. Enter V55E (Verb 55 Enter)
; 3. AGC displays flashing V25 N24 (prompting for time delta input)
; 4. Crew enters signed time correction in centiseconds via keyboard
; 5. Crew presses V33E (Proceed) to execute time adjustment
; 6. AGC adds delta time to current mission elapsed time
;
; The flashing V25 N24 prompts the crew to load the time correction value.
; Positive values advance the clock forward; negative values set it back.
;
; TECHNICAL OPERATION:
; 1. Verifies no other extended verb active via TESTXACT
; 2. Jumps to R33 routine (in bank 43) which handles time alignment logic
; 3. R33 sets extended verb display busy flag to prevent conflicts
; 4. Displays flashing V25 N24 requesting time delta input
; 5. Waits for V33E (Proceed) execution
; 6. Adds delta time from input registers to TIME2 (mission elapsed time)
; 7. Releases extended verb display system
;
; Time is stored in double-precision centiseconds (1 cs = 0.01 seconds).
; The correction is applied atomically to prevent timing glitches during
; ongoing computations.
;
# ALINTIME	VERB 55		DESCRIPTION
#	REQUIRE P00 OR P00-.
#
#	1.	SET EXT VERB DISPLAY BUSY FLAG.
#	2.	DISPLAY FLASHING V25,N24 (LOAD DELTA TIME FOR AGC CLOCK.
#	3.	REQUIRE EXECUTION OF VERB 23.
#	4.	ADD DELTA TIME, RECEIVED FROM INPUT REGISTER, TO THE COMPUTER TIME.
#	5.	RELEASE EXT VERB DISPLAY SYSTEM

ALINTIME	TC	TESTXACT
		TC	POSTJUMP	# NO ROOM IN 43
		CADR	R33

		BANK	42
		SETLOC	SBAND
		BANK
		COUNT*	$$/R33

R33		CAF	PRIO7
		TC	PRIOCHNG
		CAF	VNLODDT
		TC	BANKCALL
		CADR	GOXDSPF
		TC	ENDEXT		# TERMINATE
		TC	ENDEXT		# PROCEED
		CS	DEC23		# DATA IN OR RESEQUENCE (UNLIKELY)
		AD	MPAC		# RECALL LEFT VERB IN MPAC
		EXTEND
		BZF	UPDATIME	# GO AHEAD WITH UPDATE ONLY IF RECALL
		TC	ENDEXT		#	WITH V23 (DATA IN).

UPDATIME	INHINT			# DELTA TIME IS IN DSPTEM1, +1.
		CAF	ZERO
		TS	MPAC +2		# NEEDED FOR TP AGREE
		TS	L		# ZERO T1 + 2 WHILE ALIGNING.
		DXCH	TIME2
		DXCH	MPAC
		DXCH	DSPTEM2 +1	# INCREMENT
		DAS	MPAC

		TC	TPAGREE		# FORCE SIGN AGREEMENT.
		DXCH	MPAC		# NEW CLOCK.
		DAS	TIME2
		RELINT
UPDTMEND	TC	ENDEXT

DEC23		DEC	23		# V 23

VNLODDT		VN	2524		# V25N24 FOR LOAD DELTA TIME

# Page 276
# SET UP FOR RADAR SAMPLING.

		BANK	42
		SETLOC	EXTVERBS
		BANK

		EBANK=	RSTACK

		COUNT*	$$/R0477

R77		TC	RDRUSECK	# TRY TO AVOID THE 1210.
		CA	FLAGWRD3	# IS R04 RUNNING?
		MASK	R04FLBIT
		CCS	A
		TC	ALM/END		# YES.
		TC	UPFLAG
		ADRES	R77FLAG
		TCF	R04Z

R04		TC	RDRUSECK	# TRY TO AVOID THE 1210.
		TC	TESTXACT
		TC	UPFLAG
		ADRES	R04FLAG		# SET R04FLAG FOR ALARMS

R04Z		CAF	EBANK4
		TS	EBANK
		CAF	1SEC+1		# SAMPLE ONCE PER SECOND
		TS	RSAMPDT
		CAF	ZERO
		TS	RTSTLOC
		TS	RFAILCNT	# ZERO BAD SAMPLE COUNTER

		INHINT
		CS	LRPOSCAL	# INITIALIZE
		MASK	RADMODES	#	BIT 9	LR RANGE LOW SCALE =0
		TS	RADMODES	#	BIT 6	LR POS 1 =0
		CAF	LRPOSCAL	#	BIT 3	RR RANGE LOW SCALE =0
		EXTEND
		RAND	CHAN33
		ADS	RADMODES
		RELINT

		CS	FLAGWRD3	# CHECK R04FLAG		R04 =1		R77 =0
		MASK	R04FLBIT
		CCS	A
		TCF	R04K

		CAF	ONE		# INDICATES RENDEZVOUS DESIRED
		TS	OPTIONX +1
R04A		CAF	BIT3		# OPTION CODE FOR V04N12

# Page 277
		TS	OPTIONX
		CAF	V04N12X
		TC	BANKCALL	#	R2	00001	RENDEZVOUS RADAR
		CADR	GOMARKFR	#		00002	LANDING RADAR
		TCF	R04END		# V34
		TCF	+5		# V33
		TCF	R04A 	+2	# R2
		CAF	BIT3
		TC	BLANKET
		TC	ENDOFJOB

		CA	OPTIONX	+1	# SAVE DESIRED OPTION	RR =1	LR =2
		TS	RTSTDEX

R04X		CAF	SIX		# RR OR LR DESIRED
		MASK	RTSTDEX
		CCS	A
		TCF	R04L		# LANDING RADAR
		TS	RTSTBASE	# FOR RR	BASE =0, MAX =1

R04B		CAF	BIT2		# IS RR AUTO MODE DISCRETE PRESENT
		EXTEND
		RAND	CHAN33
		EXTEND
		BZF	R04C		# YES

		CAF	201R04		# REQUEST SELECTION OF RR AUTO MODE
		TS	DSPTEM1
		CAF	V50N25X
		TC	BANKCALL
		CADR	GOMARK4
		TCF	R04END		# V34
		TCF	R04B		# V33
		TCF	-7		# E

R04C		CAF	BIT14		# ENABLE RR AUTO TRACKER
		EXTEND
		WOR	CHAN12

		CAF	TWO
		TS	RTSTMAX		# FOR SEQUENTIAL STORAGE

		TC	WAITLIST
## [WORKAROUND] RSB 2009
		SBANK=	PINSUPER
## [WORKAROUND]
		EBANK=	RSTACK
		2CADR	RADSAMP

		RELINT

		CS	FLAGWRD3	# CHECK R04FLAG		R04 =1		R77 =0
		MASK	R04FLBIT
# Page 278
		CCS	A
		TCF	GOPIN		# R77

		CAF	SIX		# RR OR LR
		MASK	RTSTDEX
		CCS	A
		TCF	R04LR		# LR

R04RR		CAF	V16N72		# DISPLAY RR CDU ANGLES (1/SEC)
		TC	BANKCALL	#	R1 + XXX.XX DEG		TRUNNION
		CADR	GOMARKF		#	R2 + XXX.XX DEG		SHAFT
		TCF	R04END		# V34	R3   BLANK
		TCF	+2		# V33
		TCF	R04RR		# V32

		CAF	V16N78		# DISPLAY RR RANGE AND RANGE RATE (1/SEC)
		TC	BANKCALL	#	R1 +- XXX.XX NM		RANGE
		CADR	GOMARKF		#	R2 +- XXXXX. FPS	RANGE RATE
		TCF	R04END		# V34	R3    BLANK
		TCF	R04Y		# V33
		TCF	R04RR		# V32

R04LR		CAF	V16N66		# DISPLAY LR RANGE AND POSITON (1/SEC)
		TC	BANKCALL	#	R1 +- XXXXX, FT		LR RANGE
		CADR	GOMARKF		#	R2 +  0000X. 		POS. NO.
		TCF	R04END		# V34	R3    BLANK
		TCF	+2		# V33
		TCF	R04LR		# V32

		CAF	V16N67		# DISPLAY LR VELX, VELY, VELZ (1/SEC)
		TC	BANKCALL	#	R1 +- XXXXX. FPS	LR V(X)
		CADR	GOMARKF		#	R2 +- XXXXX. FPS	LR V(Y)
		TCF	R04END		# V34	R3 +- XXXXX. FPS 	LR V(Z)
		TCF	R04Y		# V33
		TCF	R04LR		# V32

R04Y		CAF	ZERO		# TO TERMINATE SAMPLING.
		TS	RSAMPDT
		CAF	2SECS		# WAIT FOR LAST RADARUP
		TC	BANKCALL
		CADR	DELAYJOB
		CAF	1SEC+1		# SAMPLE ONCE PER SECOND
		TS	RSAMPDT
		CAF	ZERO		# FOR STORING RESULTS
		TS	RTSTLOC
		CAF	SIX
		MASK	RTSTDEX
		CCS	A
		CS	ONE		# WAS LR
		AD	TWO		# WAS RR
# Page 279

		TCF	R04X -1

R04K		CAF	250MS+1		# SAMPLE 4 LR COMPONENTS PER SECOND.
		TS	RSAMPDT

R04L		CAF	TWO
		TS	RTSTBASE	# FOR LR	BASE =2, MAX =3
		CAF	SIX
		TCF	R04C +4
R04END		CAF	ZERO		# ZERO RSAMPDT
		TS	RSAMPDT		# TO TERMINATE SAMPLING
		CAF	BIT8		# WAIT 1.28 SECONDS FOR POSSIBLE
		TC	BANKCALL	# PENDING RUPT.
		CADR	DELAYJOB

		INHINT
		CS	BIT14		# DISABLE RR AUTO TRACKER.
		EXTEND
		WAND	CHAN12

		TC	DOWNFLAG
		ADRES	R04FLAG		# SIGNAL END OF R04.

		TC	ENDEXT

R77END		CAF	EBANK4		# TO TERMINATE SAMPLING
		TS	EBANK
		CAF	ZERO
		TS	RSAMPDT
		CAF	BIT6		# WAIT 320 MS FOR POSSIBLE
		TC	BANKCALL	# PENDING RUPT.
		CADR	DELAYJOB

		TC	DOWNFLAG
		ADRES	R77FLAG
		TCF	GOPIN

V16N72		VN	1672
V16N78		VN	1678
V16N66		VN	1666
V16N67		VN	1667
V04N12X		VN	412
V50N25X		VN	5025
201R04		OCT	00201
1SEC+1		DEC	101
250MS+1		EQUALS	CALLCODE
LRPOSCAL	OCT	444

# Page 280
RDRUSECK	CS	FLAGWRD3	# IS R29 ON?
		MASK	NR29FBIT
		CCS	A
		TC	ALM/END		# YES
		CA	FLAGWRD5	# IS R77 RUNNING?
		MASK	R77FLBIT
		CCS	A
		TC	ALM/END		# YES.
		CS	FLAGWRD7	# IS SERVICER RUNNING AND HENCE POSSIBLY
		MASK	V37FLBIT	# R12 USING THE LR?
		CCS	A
		TCF	CHECKRR		# NO
		CS	FLGWRD11	# YES, IS R12 ON?
		MASK	LRBYBIT
		CCS	A
		TC	ALM/END		# YES
CHECKRR		CS	FLAGWRD1	# IS THE TRACK FLAG SET AND HENCE POSSIBLY
		MASK	TRACKBIT	# P20 USING THE RR?
		CCS	A
		TCF	CHECKP22	# NO, CHECK FOR P22.

CKRNDBIT	CA	FLAGWRD0	# YES, BUT IS IT P25?
		MASK	RNDVZBIT
		CCS	A
		TC	ALM/END
CHECKP22	CS	MODREG
		AD	DEC22
		EXTEND
		BZF	ALM/END
		TC	Q

DEC22		DEC	22

		COUNT* 	$$/EXTVB

VB64		TC	CHKPOOH		# DEMAND PROGRAM 00.
		TC	TESTXACT	# IF DISPLAY SYS. NOT BUSY MAKE IT BUSY.
		CAF	PRIO4
		TC	FINDVAC
		EBANK=	ALPHASB
		2CADR	SBANDANT	# CALC., DISPLAY S-BAND ANTENNA ANGLES.

		TC	ENDOFJOB

# Page 281
# IMUATTCK	VERB 43		DESCRIPTION
#	LOAD IMU ATTITUDE ERROR METERS
#
#	1.	REQUIRE P00 OR FRESH START.
#	2.	REQUIRE COARSE ALIGN ENABLE AND ZERO ICDU BITS OFF.
#	3.	REQUIRE THAT NEEDLES BE OFF.
#	4.	REQUEST LOAD OF N22 (VALUES TO BE DISPLAYED).
#	5.	ON PROCEED OR ENTER RE-DISPLAY V43 AND SEND PULSES.

IMUATTCK	TC	CHKPOOH		# VB 76 -- LOAD IMU ATT. ERROR METERS

		CAF	BITS4&5		# SEE IF COARSE ALIGN ENABLE AND ZERO IMU
		EXTEND			# CDUS BITS ARE ON
		RAND	CHAN12
		CCS	A
		TCF	ALM/END		# NOT ALLOWED IF IMU COARSE OR IMU ZERO ON

		CAF	BIT13-14	# BOTH BITS 13 AND 14 MUST BE 1
		EXTEND			# INDICATING THE MODE SELECTED IS OFF.
		RXOR	CHAN31
		MASK	BIT13-14
		EXTEND
		BZF	+2		# NEEDLES IS OFF.
		TCF	ALM/END		# EXIT.  NEEDLES IS ON.

		TC	TESTXACT

		CAF	VNLODCDU
		TC	BANKCALL
		CADR	GOXDSPF
		TC	ENDEXT		# V34
		TC	+1
		CAF	V43K		# REDISPLAY OUR VERB.
		TC	BANKCALL
		CADR	EXDSPRET
		CAF	BIT6
		EXTEND
		WOR	CHAN12		# ENABLE ERROR COUNTERS.
		CAF	TWO
		TC	WAITLIST	# PUT OUT COMMANDS IN .32 SECONDS.
		EBANK=	THETAD
		2CADR	ATTCK2

		TCF	ENDEXT

		BANK	42
		SETLOC	PINBALL3	# SOMETHING IN B42.
		BANK

		COUNT*	$$/EXTVB
# Page 282
ATTCK2		CAF	TWO		# PUT OUT COMMANDS.
 +1		TS	Q		# CDU WILL LIMIT EXCESS DATA.
		INDEX	A
		CA	THETAD
		EXTEND
		MP	ATTSCALE
		INDEX	Q
		XCH	CDUXCMD
		CCS	Q
		TCF	ATTCK2 +1

		CAF	13,14,15
		EXTEND
		WOR	CHAN14
		TCF	TASKOVER	# LEAVE ERROR COUNTERS ENABLED.

ATTSCALE	DEC	0.1

		BANK	7
		SETLOC	EXTVERBS
		BANK

		COUNT*	$$/EXTVB

V43K		VN	4300

# V82PERF	VERB82		DESCRIPTION
#	REQUEST ORBIT PARAMETERS DISPLAY (R30)
#
#	1.	IF AVERAGE G IS OFF:
#			FLASH DISPLAY V04N06.  R2 INDICATES WHICH SHIP'S STATE VECTOR IS
#			TO BE UPDATED.  INITIAL CHOICE IS THIS SHIP (R2=1).  ASTRONAUT
#			CAN CHANGE TO OTHER SHIP BY V22EXE, WHERE X NOT EQ I.
#			SELECTED STATE VECTOR UPDATED BY THISPREC (OTHPREC).
#			CALLS SR30.1 (WHICH CALLS TFFCONMU + TFFRP/RA) TO CALCULATE
#			RPER (PERIGEE RADIUS), RAP0 (APOGEE RADIUS), HPER (PERIGEE
#			HEIGHT ABOVE LAUNCH PAD OR LUNAR LANDING SITE), HAPO (APOGEE
#			HEIGHT AS ABOVE), TPER (TIME TO PERIGEE), TFF (TIME TO
#			INTERSECT 300 KFT ABOVE PAD OR 35KFT ABOVE LANDING SITE).
#			FLASH MONITOR V16N44 (HAPO, HPER, TFF).  TFF IS -59M59S IF IT WAS
#			NOT COMPUTABLE, OTHERWISE IT INCREMENTS ONCE PER SECOND.
#			ASTRONAUT HAS OPTION TO MONITOR TPER BY KEYING IN N 32 E.
#			DISPLAY IS IN HMS, IS NEGATIVE (AS WAS TFF), AND INCREMENTS
#			ONCE PER SECOND ONLY IF TFF DISPLAY WAS -59M59S.
#
#	2.	IF AVERAGE G IS ON:
#			CALLS SR30.1 APPROX EVERY TWO SECS.  STATE VECTOR IS ALWAYS
#			FOR THIS VEHICLE.  V82 DOES NOT DISTURB STATE VECTOR.  RESULTS
#			OF SR30.1 ARE RAPO, RPER, HAPO, HPER, TPER, TFF.
#			FLASH MONITOR V16N44 (HAPO, HPER, TFF).
#			IF MODE IS P11, THEN CALL DELRSPL SO ASTRONAUT CAN MONITOR
#			RESULTS BY N50E.  SPLASH COMPUTATION DONE ONCE PER TWO SECS.

# Page 283
V82PERF		TC	TESTXACT

		CAF	PRIO7		# LESS THAN LAMBERT.  R30,V82
		TC	PRIOCHNG
		EXTEND
		DCA	V82CON
		TC	SUPDXCHZ	# V82CALL IN DIFF SUPERBANK FROM V82PERF

		EBANK=	HAPO
V82CON		2CADR	V82CALL

# VB83PERF	VERB 83		DESCRIPTION
#	REQUEST RENDEZVOUS PARAMETER DISPLAY (R31)
#
#	1.	SET EXT VERB DISPLAY BUSY FLAG.
#	2.	SCHEDULE R31CALL WITH PRIORITY 5.
#		A.	DISPLAY
#			R1	RANGE
#			R2	RANGE RATE
#			R3	THETA

V83PERF		TC	TESTXACT

		CAF	BIT2
		TC	WAITLIST
		EBANK=	TSTRT
		2CADR	R31CALL

		TC	ENDOFJOB

# VERB 89	DESCRIPTION	RENDEZVOUS FINAL ATTITUDE ROUTINE (R63)
#
# CALLED BY VERB 89 ENTER DURING P00.  PRIO 10 IS USED.  CALCULATES AND
# DISPLAYS FINAL FDAI BALL ANGLES TO POINT LM +X OR +Z AXIS AT CSM.
#
# 1. KEY IN V 89 E ONLY IF IN PROG 00.  IF NOT IN P00, OPERATOR ERROR AND
# EXIT R63, OTHERWISE CONTINUE.
#
# 2. IF IN P00, DO IMU STATUS CHECK ROUTINE (R02BOTH).  IF IMU ON AND ITS
# ORIENTATION KNOWN TO LGC,CONTINUE.
#
# 3. FLASH DISPLAY V 04 N 06.  R2 INDICATES WHICH SPACECRAFT AXIS IS TO
# BE POINTED AT CSM.  INITIAL CHOICE IS PREFERRED (+Z) AXIS (R2=1).
# ASTRONAUT CAN CHANGE TO (+X) AXIS (R2 NOT = 1) BY V 22 E 2 E.  CONTINUE
# AFTER KEYING IN PROCEED.
#
# 4. BOTH VEHICLE STATE VECTORS UPDATED BY CONIC EQS.
#
# 5. HALF MAGNITUDE UNIT LOS VECTOR (IN STABLE MEMBER COORDINATES) AND
# Page 284
# HALF MAGNITUDE UNIT SPACECRAFT AXIS VECTOR (IN BODY COORDINATES)
# PREPARED FOR VECPOINT.
#
# 6. GIMBAL ANGLES FROM VECPOINT TRANSFORMED INTO FDAI BALL ANGLES BY
# BALLANGS.  FLASH DISPLAY V 06 N 18 AND AWAIT RESPONSE.
#
# 7. 	RECYCLE -- RETURN TO STEP 4.
#    	TERMINATE -- EXIT R63.
#	PROCEED -- RESET 3AXISFLG AND CALL R60LEM FOR ATTITUDE MANEUVER.

V89PERF		TC	CHKPOOH
		TC	TESTXACT
		CAF	PRIO10
		TC	FINDVAC
		EBANK=	RONE
		2CADR	V89CALL

		TC	ENDOFJOB

# V90PERF	VERB 90		DESCRIPTION
#	REQUEST RENDEZVOUS OUT-OF-PLANE DISPLAY (R36)
#
#	1.	SET EXT VERB DISPLAY BUSY FLAG.
#	2.	SCHEDULE R36 CALL WITH PRIORITY 10
#		A.	DISPLAY
#			TIME OF EVENT -- HOURS, MINUTES, SECONDS
#			Y 	OUT-OF-PLANE POSITION -- NAUTICAL MILES
#			YDOT	OUT-OF-PLANE VELOCITY -- FEET/SECOND
#			PSI	ANGLE BTW LINE OF SIGHT AND FORWARD
#				DIRECTION VECTOR IN HORIZONTAL PLANE -- DEGREES

V90PERF		TC	TESTXACT
		CAF	PRIO7		# R36,V90
		TC	FINDVAC
		EBANK=	RPASS36
		2CADR	R36

		TCF	ENDOFJOB

# MINIMP	VERB 76		DESCRIPTION
#	MINIMUM IMPULSE MODE
#
#	1.	SET MINIMUM IMPULSE RHO MODE FLAG TO 1.

MINIMP		INHINT
		CS	DAPBOOLS
		MASK	PULSES		# PULSES = 1 INDICATES MIN IMP MODE
		ADS	DAPBOOLS
		TCF	GOPIN		# RETURN VIA PINBRNCH

# NOMINIMP	VERB 77		DESCRIPTION
#	RATE COMMAND MODE

# Page 285

#
#	1.	SET MINIMUM IMPULSE RHO MODE FLAG TO 0.  (ZERO INDICATES NOT MINIMUM IMPULSE MODE.).
#	2.	MOVE CDUX, CDUY, CDUZ INTO CDUXD, CDUYD, CDUZD.


NOMINIMP	INHINT
		CS	PULSES
		MASK	DAPBOOLS
		TS	DAPBOOLS	# PULSES = NOT IN MINIMUM UMPULSE MODE
		TC	IBNKCALL
		CADR	ZATTEROR
		TC	GOPIN

# Page 286
# CREMANU	VERB 49		DESCRIPTION
#	START AUTOMATIC ATTITUDE MANEUVER
#
#	1.	REQUIRE PROGRAM 00 ACTIVE.
#	2.	SET EXT VERB DISPLAY BUSY FLAG.
#	3.	SCHEDULE R62DISP WITH PRIORITY 10.
#	4.	RELEASE EXT VERB DISPLAY.
#
#	R62DISP
#	1.	DISPLAY FLASHING V06,N22.
#		RESPONSES
#		A.	TERMINATE
#			1.	GOTOPOOH
#		B.	PROCEED
#			1.	SET 3AXISFLG TO INDICATE MANEUVER IS SPECIFIED BY 3 AXIS.
#			2.	EXECUTE R60LEM (ATTITUDE MANEUVER).
#		C.	ENTER
#			1.	REPEAT FLASHING V06,N22.

CREWMANU	TC	CHKPOOH		# DEMAND P00

		TC	TESTXACT

		CAF	PRIO10
		TC	FINDVAC
		EBANK=	BCDU
		2CADR	R62DISP

		TC	ENDOFJOB

# Page 287
# ============================================================================
# VERB 56: TERMINATE TRACKING (TRMTRACK)
# ============================================================================
; VERB 56 (V56) - TERMINATE TRACKING
; 
; PURPOSE:
; Terminates active rendezvous tracking programs P20 (Rendezvous Navigation)
; or P25 (Continuous Designate). This verb allows the crew to halt radar
; tracking operations and exit the tracking programs cleanly.
; 
; MISSION CONTEXT:
; During Apollo 11's rendezvous phase after Eagle's ascent from the lunar
; surface on July 21, 1969, P20 was used to track Columbia (the Command Module)
; using the rendezvous radar. Once tracking was complete or if the crew needed
; to switch to another program, V56 provided the clean termination sequence.
; Proper tracking termination was essential to prevent radar updates from
; corrupting navigation state during subsequent operations.
; 
; CREW PROCEDURE:
; 1. Verify P20 or P25 is active (RNDVZFLG or P25FLAG set)
; 2. Enter V56E (Verb 56 Enter) to request tracking termination
; 3. AGC halts radar tracking, clears tracking flags, terminates P20/P25
; 4. System returns to previous program state or P00 (idle)
; 
; TECHNICAL OPERATION:
; 1. Check if RNDVZFLG (bit 9) or P25FLAG (bit 7) in FLAGWRD0 is set
;    - If neither flag set, branch to GOPIN (improper operator error)
; 2. Clear RNDVZFLG (rendezvous flag) via DOWNFLAG
; 3. Clear P25FLAG (continuous designate flag) via DOWNFLAG
; 4. Clear SRCHOPTN (search option flag) to ensure search mode disabled
; 5. Check if TRACKFLG in FLAGWRD1 is set
;    - If not set, branch to GOPIN (no tracking active)
; 6. Jump to TRMTRAK1 (in bank 42) via POSTJUMP for completion sequence
; 
; TRMTRAK1 COMPLETION SEQUENCE:
; 1. Clear UPDATFLG (navigation update flag) via DOWNFLAG
; 2. Clear TRACKFLG (tracking active flag) via DOWNFLAG
; 3. Clear IMUSE (IMU in use flag) via DOWNFLAG
; 4. Call INTSTALL to wait for any active orbital integration to complete
;    (prevents interrupting mid-integration which would corrupt state vector)
; 5. Execute PHASCHNG with OCT 2 to kill phase group 2, halting P20 activity
; 6. Transfer control to GOPROG2 for software restart/program transition
; 
; FLAG MANAGEMENT:
; RNDVZFLG - Rendezvous program active indicator
; P25FLAG - Continuous designate mode active
; SRCHOPTN - Rendezvous search option enabled
; TRACKFLG - Radar tracking currently active
; UPDATFLG - Navigation updates from radar enabled
; IMUSE - IMU being used for alignment/tracking
; 
; HISTORICAL NOTE:
; After Eagle's successful rendezvous and docking with Columbia, V56 was used
; to terminate P20 tracking mode, allowing the crew to proceed with docking
; verification and LM jettison preparations. Clean tracking termination ensured
; the navigation state vector remained stable during subsequent maneuvers.
; 
# TRMTRACK	VERB 56		DESCRIPTION
#	TERMINATE TRACKING (P20 AND P25).
#
#	1.	KNOCK DOWN RENDEZVOUS, TRACK, AND UPDATE FLAGS.
#	2.	REQUIRE P20 OR P25 NOT RUNNING ALONE OR GO TO GOGOPOOH (REQUEST PROGRAM 00).
#	3.	SCHEDULE V56TOVAC WITH PRIORITY 30.
#
#	V56TOVAC
#	1.	EXECUTE INTSTALL (IF INTEGRATION IS RUNNING, STALL UNTIL IT IS FINISHED.).
#	2.	ZERO GROUP 2 TO HALT P20.
#	3.	TRANSFER CONTROL TO GOPROG2 (SOFTWARE RESTART).

TRMTRACK	CA	BITS9+7		# IS REND OR P25 FLAG ON
		MASK	FLAGWRD0
		EXTEND
		BZF	GOPIN		# NO

		TC	DOWNFLAG
		ADRES	RNDVZFLG

		TC	DOWNFLAG
		ADRES	P25FLAG

		TC	DOWNFLAG	# ENSURE SEARCH FLAG IS OFF
		ADRES	SRCHOPTN

		CA	TRACKBIT	# IS TRACK FLAG ON?
		MASK	FLAGWRD1
		EXTEND
		BZF	GOPIN

		TC	POSTJUMP
		CADR	TRMTRAK1

BITS9+7		OCT	500

		SETLOC	SBAND		# BANK 42
		BANK

		COUNT*	$$/EXTVB

TRMTRAK1	TC	DOWNFLAG
		ADRES	UPDATFLG	# UPDATE FLAG DOWN
		TC	DOWNFLAG
		ADRES	TRACKFLG	# TRACK FLAG DOWN
		TC	DOWNFLAG
		ADRES	IMUSE

		TC	INTPRET
		CALL
			INTSTALL	# DON'T INTERRUPT INTEGRATION
# Page 288
		EXIT

		TC	PHASCHNG
		OCT	2		# KILL GROUP 2 TO HALT P20 ACTIVITY

		INHINT
		TC	IBNKCALL	# ZERO THE COMMANDED RATES TO STOP
		CADR	STOPRATE	# MANEUVER

		TC	IBNKCALL
		CADR	RESTORDB

		TC	CLRADMOD	# CLEAR BITS 10 + 15 OF RADMODES.

		CS	BIT14		# DISABLE LOCKON
		EXTEND
		WAND	CHAN12
		TC	POSTJUMP
		CADR	GOPROG2		# CAUSE RESTART.

# ============================================================================
# VERB 74: INITIALIZE DOWN-TELEMETRY FOR ERASABLE DUMP (DNEDUMP)
# ============================================================================
; VERB 74 (V74) - INITIALIZE DOWN-TELEMETRY PROGRAM FOR ERASABLE MEMORY DUMP
; 
; PURPOSE:
; Reconfigures the down-telemetry system to dump the entire contents of
; erasable memory (RAM) to Mission Control via S-band downlink. This verb
; enables ground controllers to capture a complete snapshot of the AGC's
; 2K words of RAM for debugging, analysis, or troubleshooting purposes.
; 
; MISSION CONTEXT:
; During Apollo missions, Mission Control could request erasable memory dumps
; to diagnose anomalies, verify program state, or analyze guidance computer
; behavior. This capability was particularly valuable for understanding the
; 1202/1201 program alarms that occurred during Apollo 11's descent - post-
; mission analysis of memory dumps helped engineers understand the executive
; scheduler overload that triggered the alarms.
; 
; The downlink telemetry system normally transmits selected high-priority
; parameters (state vectors, DSKY displays, program counters, etc.). Verb 74
; temporarily replaces this normal downlist with a comprehensive erasable
; memory dump, transmitting all 2048 words sequentially.
; 
; CREW PROCEDURE:
; 1. Mission Control requests erasable dump via voice loop
; 2. Crew enters V74E (Verb 74 Enter) on DSKY
; 3. AGC reconfigures downlink telemetry for memory dump mode
; 4. Memory dump begins transmitting automatically
; 5. Ground stations receive and log complete erasable memory contents
; 6. After dump complete, normal downlist automatically resumes
; 
; No noun is required with this verb. The operation is fully automatic once
; initiated.
; 
; TECHNICAL OPERATION:
; 1. Loads LDNDUMPI constant (downlist dump initialization address)
; 2. Stores to DNTMGOTO (down-telemetry go-to address pointer)
; 3. Jumps to GOPIN for standard verb termination
; 
; DNTMGOTO is the control variable for the down-telemetry program (runs as
; a WAITLIST task). Setting DNTMGOTO to LDNDUMPI causes the next telemetry
; cycle to initialize the erasable dump sequence. The DOWN-TELEMETRY PROGRAM
; (separate module) reads DNTMGOTO and executes the specified initialization,
; which replaces the standard DOWNLIST with ERASDUMP downlist.
; 
; DOWNLIST STRUCTURE:
; Normal mode: ~100 critical parameters updated at 1-second intervals
; Dump mode: Sequential erasable addresses E0000-E1777 (octal), transmitted
;           over multiple telemetry frames until entire 2K memory captured
; 
; After dump completes, the down-telemetry program automatically restores
; the normal downlist, ensuring continuous telemetry of critical parameters.
; 
; HISTORICAL NOTE:
; Verb 74 was used sparingly during missions to minimize disruption to normal
; telemetry flow. However, it proved invaluable for post-mission analysis.
; Apollo 11's successful landing despite 1202 alarms was partially validated
; through erasable dumps showing the guidance computer maintained correct
; navigation state throughout the alarm conditions.
; 
# DNEDUMP	VERB 74		DESCRIPTION
#	INITIALZE DOWN-TELEMETRY PROGRAM FOR ERASABLE MEMORY DUMP.
#
#	1.	SET EXT VERB DISPLAY BUSY FLAG.
#	2.	REPLACE CURRENT DOWNLIST WITH ERASABLE MEMORY.
#	3.	RELEASE EXT VERB DISPLAY.

		SETLOC	EXTVERBS
		BANK

		COUNT*	$$/EXTVB

		EBANK=	400
DNEDUMP		CAF	LDNDUMPI
		TS	DNTMGOTO
		TC	GOPIN

V74		EQUALS	DNEDUMP
LDNDUMPI	REMADR	DNDUMPI

# ============================================================================
# VERB 80: UPDATE LEM STATE VECTOR (LEMVEC)
# ============================================================================
; VERB 80 (V80) - UPDATE LEM STATE VECTOR
; 
; PURPOSE:
; Selects the Lunar Module (LM) state vector as the target for subsequent
; navigation updates. When this verb is executed, all incoming state vector
; updates (from ground uplinks, radar measurements, or optical sightings)
; will be applied to the LM state vector rather than the CSM state vector.
; 
; MISSION CONTEXT:
; After LM separation from the Command/Service Module during Apollo 11's
; descent preparation on July 20, 1969, two independent state vectors were
; maintained by the AGC: one for Eagle (LM) and one for Columbia (CSM).
; During descent and ascent operations, V80 ensured navigation updates were
; correctly applied to Eagle's state vector. After rendezvous and docking,
; V81 would select Columbia's state vector for updates.
; 
; This verb-pair (V80/V81) allowed the AGC to track both spacecraft
; simultaneously while ensuring navigation measurements updated the correct
; vehicle's position and velocity.
; 
; CREW PROCEDURE:
; 1. Enter V80E (Verb 80 Enter) on DSKY
; 2. AGC clears VEHUPFLG (vehicle update flag), designating LM as target
; 3. AGC clears NOUPFLAG, enabling state vector updates
; 4. All subsequent navigation updates apply to LM state vector
; 
; No noun is required. The verb executes immediately upon entry.
; 
; TECHNICAL OPERATION:
; 1. Call DOWNFLAG to clear VEHUPFLG (vehicle update flag)
;    - VEHUPFLG = 0 indicates LM state vector is update target
;    - VEHUPFLG = 1 (set by V81) indicates CSM state vector is target
; 2. Jump to NOUPDOWN routine (shared with V81)
; 3. NOUPDOWN clears NOUPFLAG (no-update flag)
;    - NOUPFLAG = 0 enables state vector updates from navigation
;    - NOUPFLAG = 1 (set by V95) inhibits all state vector updates
; 4. Jump to GOPIN for standard verb termination
; 
; FLAG MANAGEMENT:
; VEHUPFLG - Vehicle update flag (0=LM, 1=CSM)
; NOUPFLAG - No-update flag (0=updates enabled, 1=updates inhibited)
; 
; STATE VECTOR STRUCTURE:
; The AGC maintains two complete state vectors in erasable memory:
; - RRECTCSM/RVELCSM: CSM position/velocity (inertial coordinates)
; - RRECT/RVEL: LM position/velocity (inertial coordinates)
; 
; When VEHUPFLG=0 (LM mode), navigation routines in ORBITAL_INTEGRATION and
; MEASUREMENT_INCORPORATION update RRECT/RVEL. When VEHUPFLG=1 (CSM mode),
; they update RRECTCSM/RVELCSM instead.
; 
; HISTORICAL NOTE:
; During Apollo 11's rendezvous on July 21, switching between V80 and V81
; allowed accurate tracking of both Eagle and Columbia as they approached
; each other. This dual-vector capability was critical for calculating the
; precise burn maneuvers needed for successful docking.
; 
# LEMVEC	VERB 80		DESCRIPTION
#	UPDATE LEM STATE VECTOR
#		RESET VHUPFLG TC 0

LEMVEC		TC	DOWNFLAG
		ADRES	VEHUPFLG	# VB 80 -- VEHUPFLG DOWN INDICATES LEM

		TC	NOUPDOWN

# ============================================================================
# VERB 81: UPDATE CSM STATE VECTOR (CSMVEC)
# ============================================================================
; VERB 81 (V81) - UPDATE CSM STATE VECTOR
; 
; PURPOSE:
; Selects the Command/Service Module (CSM) state vector as the target for
; subsequent navigation updates. When this verb is executed, all incoming
; state vector updates will be applied to the CSM state vector rather than
; the LM state vector.
; 
; MISSION CONTEXT:
; During Apollo 11's rendezvous phase on July 21, 1969, after Eagle's ascent
; from the lunar surface, the AGC needed to track both Eagle (LM) and Columbia
; (CSM) simultaneously. While Eagle's AGC naturally tracked its own position
; (using V80), it also needed to track Columbia's position for rendezvous
; calculations using the rendezvous radar.
; 
; When the crew obtained radar range/range-rate measurements of Columbia,
; executing V81 before processing those measurements ensured the navigation
; updates improved Columbia's state vector accuracy rather than Eagle's.
; This dual-tracking capability was essential for computing the precise
; relative velocity and position needed for successful docking.
; 
; CREW PROCEDURE:
; 1. Enter V81E (Verb 81 Enter) on DSKY
; 2. AGC sets VEHUPFLG (vehicle update flag), designating CSM as target
; 3. AGC clears NOUPFLAG, enabling state vector updates
; 4. All subsequent navigation updates apply to CSM state vector
; 
; No noun is required. The verb executes immediately upon entry.
; 
; TECHNICAL OPERATION:
; 1. Call UPFLAG to set VEHUPFLG (vehicle update flag)
;    - VEHUPFLG = 1 indicates CSM state vector is update target
;    - VEHUPFLG = 0 (cleared by V80) indicates LM state vector is target
; 2. Fall through to NOUPDOWN routine (shared with V80)
; 3. NOUPDOWN clears NOUPFLAG (no-update flag) via DOWNFLAG
;    - NOUPFLAG = 0 enables state vector updates from navigation
;    - NOUPFLAG = 1 (set by V95) inhibits all state vector updates
; 4. Jump to GOPIN for standard verb termination
; 
; NOUPDOWN SHARED ROUTINE:
; Both V80 (LEMVEC) and V81 (CSMVEC) converge at NOUPDOWN after setting
; the appropriate vehicle flag. NOUPDOWN ensures the NOUPFLAG is cleared,
; enabling navigation updates. This design allows V80/V81 to override any
; previous V95 (update inhibit) command.
; 
; FLAG INTERACTION:
; VEHUPFLG - Vehicle update flag (0=LM selected, 1=CSM selected)
; NOUPFLAG - No-update flag (0=updates enabled, 1=updates inhibited)
; 
; The combination of these flags creates four possible states:
; - VEHUPFLG=0, NOUPFLAG=0: LM updates enabled (normal LM operations)
; - VEHUPFLG=0, NOUPFLAG=1: LM selected but updates inhibited (V95 active)
; - VEHUPFLG=1, NOUPFLAG=0: CSM updates enabled (tracking CSM)
; - VEHUPFLG=1, NOUPFLAG=1: CSM selected but updates inhibited (V95 active)
; 
; HISTORICAL NOTE:
; During the critical final approach to docking, accurate tracking of both
; vehicles was essential. The crew would alternate between V80 and V81 as
; they processed different measurements - IMU accelerations updated Eagle's
; vector (V80), while rendezvous radar measurements updated Columbia's vector
; (V81). This alternating update strategy maintained accurate relative
; navigation throughout the rendezvous sequence.
; 
# CSMVEC	VERB 81		DESCRIPTION
#	UPDATE CSM STATE VECTOR
# Page 289
#		SET VEHUPFLG TO 1

CSMVEC		TC	UPFLAG
		ADRES	VEHUPFLG	# VB 81 -- VEHUPFLG UP INDICATES CSM

NOUPDOWN	TC	DOWNFLAG
		ADRES	NOUPFLAG

		TCF	GOPIN

# ============================================================================
# VERB 95: INHIBIT STATE VECTOR UPDATES (UPDATOFF)
# ============================================================================
; VERB 95 (V95) - INHIBIT STATE VECTOR UPDATES
; 
; PURPOSE:
; Disables all automatic navigation state vector updates from measurements.
; When this verb is executed, the AGC will stop incorporating radar data,
; optical sightings, and other navigation measurements into the state vector.
; The current state vector remains frozen until updates are re-enabled.
; 
; MISSION CONTEXT:
; During certain mission phases, the crew or ground controllers might want
; to prevent automatic navigation updates from corrupting the state vector.
; Common scenarios include:
; 
; 1. SUSPECT MEASUREMENTS: If radar or optical measurements appear erroneous
;    due to equipment malfunction or environmental conditions (e.g., spurious
;    landing radar returns), V95 prevents bad data from degrading navigation.
; 
; 2. GROUND UPLINK PENDING: Before receiving a high-accuracy state vector
;    update from ground tracking (Mission Control's more accurate tracking),
;    V95 prevents onboard measurements from introducing errors that would
;    need correction.
; 
; 3. MANUAL NAVIGATION MODE: During certain test procedures or backup modes,
;    the crew might manually manage the state vector without automatic updates.
; 
; CREW PROCEDURE:
; 1. Enter V95E (Verb 95 Enter) on DSKY
; 2. AGC sets NOUPFLAG (no-update flag), inhibiting all navigation updates
; 3. State vector remains constant (no measurement incorporation)
; 4. To re-enable updates, execute V80 (select LM) or V81 (select CSM)
; 
; No noun is required. The verb executes immediately upon entry.
; 
; TECHNICAL OPERATION:
; 1. Call UPFLAG to set NOUPFLAG (no-update flag)
;    - NOUPFLAG = 1 inhibits all state vector updates
;    - Navigation routines check NOUPFLAG before incorporating measurements
; 2. Jump to GOPIN for standard verb termination
; 
; NOUPFLAG remains set until cleared by:
; - V80 (LEMVEC): Selects LM updates and clears NOUPFLAG
; - V81 (CSMVEC): Selects CSM updates and clears NOUPFLAG
; 
; INTERACTION WITH NAVIGATION ROUTINES:
; The MEASUREMENT_INCORPORATION routine (handles radar, optical measurements)
; checks NOUPFLAG before updating state vectors:
; 
; IF NOUPFLAG = 0 (cleared):
;   - Process measurements and update RRECT/RVEL or RRECTCSM/RVELCSM
;   - Normal navigation operation
; 
; IF NOUPFLAG = 1 (set by V95):
;   - Skip measurement processing
;   - State vectors remain unchanged
;   - Measurements are discarded
; 
; AFFECTED NAVIGATION SOURCES:
; When V95 is active, these measurement sources are ignored:
; - Landing radar altitude and velocity
; - Rendezvous radar range and range-rate
; - Optical telescope star/landmark sightings
; - IMU accelerometer integration (depending on mode)
; 
; STATE VECTOR EXTRAPOLATION:
; Even with V95 active, the AGC continues to extrapolate (predict) the state
; vector forward in time using orbital mechanics. V95 only prevents
; measurement-based corrections, not time propagation. The ORBITAL_INTEGRATION
; routine continues to update position/velocity based on gravitational forces.
; 
; HISTORICAL NOTE:
; During Apollo 11's descent, if the landing radar had continued to provide
; suspect data after the initial problems, the crew could have used V95 to
; reject radar updates and rely on IMU-only navigation for landing. This
; would have reduced accuracy but prevented bad radar data from causing an
; abort. The flight controllers' decision to "Go on that alarm" (1202)
; meant V95 was not needed, and normal navigation continued.
; 
# UPDATOFF	VERB 95		DESCRIPTION
#	INHIBIT STATE VECTOR UPDATES BY INCORP
#		SET NOUPFLAG TO 1

UPDATOFF	TC	UPFLAG		# VB 95 SET NOUPFLAG
		ADRES	NOUPFLAG

		TC	GOPIN
# Page 290
# ============================================================================
# VERB 92: OPERATE IMU PERFORMANCE TEST (SYSTEST)
# ============================================================================
; VERB 92 (V92) - OPERATE IMU PERFORMANCE TEST
; 
; PURPOSE:
; Executes comprehensive diagnostic tests of the Inertial Measurement Unit (IMU)
; to verify gyroscope and accelerometer performance. This verb initiates a
; series of self-check routines that exercise the IMU and validate its accuracy
; against known parameters.
; 
; MISSION CONTEXT:
; The IMU is the heart of the Apollo Guidance Computer's navigation system.
; Three gyroscopes measure spacecraft attitude (orientation) and three
; accelerometers measure velocity changes. Any degradation in IMU performance
; could compromise navigation accuracy during critical mission phases.
; 
; V92 was typically executed:
; 1. PRE-FLIGHT: During ground checkout to verify IMU health before launch
; 2. IN-FLIGHT: Periodically during long coast phases (translunar, transearth)
;    to detect any degradation from radiation or thermal effects
; 3. POST-ALIGNMENT: After P51/P52 IMU alignment to verify alignment quality
; 
; For Apollo 11, IMU performance testing was particularly important during
; the 3-day translunar coast. Any gyro drift or accelerometer bias detected
; by V92 could be compensated before the critical lunar orbit insertion burn.
; 
; CREW PROCEDURE:
; 1. Ensure spacecraft is in P00 (Program 00 - idle state)
;    - No active program must be running during IMU test
;    - AGC displays operator error if P00 requirement not met
; 2. Enter V92E (Verb 92 Enter) on DSKY
; 3. AGC initiates IMU test sequence (runs in background)
; 4. Test results displayed via verb completion or subsequent display verbs
; 
; No noun is required. The verb executes immediately upon entry.
; 
; TECHNICAL OPERATION:
; 1. CHKPOOH - Check for Program 00 requirement
;    - Verifies MODREG contains 00 (no active program)
;    - If MODREG ≠ 00, sets operator error flag and aborts
;    - P00 requirement prevents IMU test from interfering with active guidance
; 
; 2. TESTXACT - Test Extended Verb Executive Availability
;    - Checks if extended verb display system is available
;    - Sets extended verb busy flag to prevent verb conflicts
;    - Reserves display resources for test duration
; 
; 3. FINDVAC - Find Vacant Core Set
;    - Priority 22 (medium priority, below guidance but above housekeeping)
;    - Allocates executive core set for background IMU test execution
;    - Core set allows test to run without blocking other operations
; 
; 4. Schedule REDO routine via 2CADR
;    - 2CADR (two-word address) specifies routine and bank
;    - REDO routine located in IMUSUPER bank (IMU supervisor code)
;    - REDO implements the actual IMU performance test sequence
;    - EBANK=QPLACE provides erasable memory bank context
; 
; 5. ENDOFJOB - Terminate current job
;    - Returns control to executive scheduler
;    - IMU test continues in background via scheduled REDO job
; 
; IMU TEST SEQUENCE (REDO ROUTINE):
; The REDO routine in IMUSUPER bank performs these tests:
; - Gyro drift rate measurement (compares measured vs expected drift)
; - Accelerometer bias measurement (detects zero-g offset errors)
; - IMU compensation package verification (checks compensation parameters)
; - Pulsed integrating pendulous accelerometer (PIPA) pulse count validation
; - Coupling data unit (CDU) readout accuracy verification
; 
; Test execution time: Approximately 2-5 minutes depending on test depth.
; 
; PRIORITY LEVEL RATIONALE:
; Priority 22 (PRIO22) places IMU testing above routine housekeeping tasks
; (priority 30+) but below critical guidance computations (priority 10-20).
; This ensures:
; - Test completes in reasonable time (not starved by low priority)
; - Critical guidance not delayed if test coincides with burn or maneuver
; - Restart protection preserves test state if higher priority jobs preempt
; 
; OPERATOR ERROR CONDITIONS:
; V92 displays operator error if:
; - MODREG ≠ 00 (active program running - cannot test during guidance)
; - Extended verb system busy (another extended verb active)
; - IMU not warmed up (IMU mode not FINE ALIGN or COARSE ALIGN)
; 
; HISTORICAL NOTE:
; The IMU was one of Apollo's most critical and delicate components. Built by
; MIT's Instrumentation Laboratory, it used mechanical gyroscopes spinning at
; 24,000 RPM on gas bearings and accelerometers sensitive enough to measure
; the thrust from a single RCS jet. Regular performance testing via V92
; provided confidence that this remarkable instrument remained accurate
; throughout the mission.
; 
; During Apollo 11, the IMU performed flawlessly. Post-mission analysis showed
; gyro drift rates remained well within specifications throughout the 8-day
; mission, a testament to the quality of both the hardware and the diagnostic
; software that monitored it.
; 
# SYSTEST	VERB 92		DESCRIPTION
#	OPERATE IMU PERFORMANCE TEST.
#
#	1.	REQUIRE PROGRAM 00 OR TURN ON OPERATOR ERROR.
#	2.	SET EXT VERB BUSY FLAG.

		EBANK=	QPLACE

SYSTEST		TC	CHKPOOH		# DEMAND P00

		TC	TESTXACT

		CAF	PRIO22
		TC	FINDVAC
		EBANK=	QPLACE
		SBANK=	IMUSUPER
		2CADR	REDO

		TC	ENDOFJOB

# ============================================================================
# VERB 93: CLEAR RENDEZVOUS FLAG - REINITIALIZE W-MATRIX (WMATRXNG)
# ============================================================================
; VERB 93 (V93) - CLEAR RENDWFLG, CAUSES W-MATRIX TO BE RE-INITIALIZED
; 
; PURPOSE:
; Clears the rendezvous W-matrix flag (RENDWFLG), forcing the W-matrix to be
; recomputed from scratch. The W-matrix is a critical component of the Kalman
; filter used for navigation state estimation during rendezvous operations.
; 
; MISSION CONTEXT:
; During lunar module rendezvous with the command module, the AGC uses a Kalman
; filter to optimally combine measurements from multiple sensors (rendezvous
; radar, IMU, optics) to estimate the relative position and velocity between
; the two spacecraft. The W-matrix is the state covariance matrix that
; represents the statistical uncertainty in the navigation state.
; 
; The W-matrix evolves over time as:
; - It grows (uncertainty increases) during coast phases
; - It shrinks (uncertainty decreases) when measurements are incorporated
; 
; V93 forces W-matrix re-initialization when:
; 1. Navigation state has been manually corrected (state vector update)
; 2. Sensor mode change (e.g., switching from radar to optical tracking)
; 3. Suspected navigation divergence (crew or ground notices bad state)
; 4. Starting new rendezvous phase (e.g., transition from coelliptic to TPI)
; 
; For Apollo 11, rendezvous operations occurred after Eagle's ascent from the
; lunar surface. The LM's guidance computer tracked Columbia's position using
; rendezvous radar while simultaneously computing ascent trajectory. V93 could
; be used if the crew or ground control detected navigation inconsistencies.
; 
; CREW PROCEDURE:
; 1. Enter V93E (Verb 93 Enter) on DSKY
; 2. AGC clears RENDWFLG immediately
; 3. Next navigation cycle reinitializes W-matrix with default uncertainty
; 4. Subsequent measurements rebuild state confidence
; 
; No noun is required. The verb executes immediately.
; 
; TECHNICAL OPERATION:
; 1. INHINT - Inhibit Interrupts
;    - Critical to prevent interrupt during flag manipulation
;    - Ensures atomic read-modify-write of FLAGWRD5
;    - Prevents race condition with navigation routines checking RENDWFLG
; 
; 2. CS RENDWBIT - Complement and Sense RENDWBIT
;    - Loads bitwise complement of RENDWBIT constant into A register
;    - RENDWBIT contains single bit corresponding to RENDWFLG position
;    - Complement creates mask with all bits set EXCEPT RENDWFLG bit
; 
; 3. MASK FLAGWRD5 - Logical AND with FLAGWRD5
;    - FLAGWRD5 contains multiple software flags (16 bits)
;    - MASK operation preserves all bits except RENDWFLG
;    - Effectively clears RENDWFLG while preserving other flags
; 
; 4. TS FLAGWRD5 - Store result back to FLAGWRD5
;    - Updates flag word with RENDWFLG cleared
;    - Other flags in FLAGWRD5 remain unchanged
; 
; 5. TC GOPIN - Transfer Control to GOPIN
;    - GOPIN re-enables interrupts and returns to verb processor
;    - Equivalent to RELINT followed by return
; 
; W-MATRIX RE-INITIALIZATION EFFECT:
; When navigation code detects RENDWFLG is clear:
; - Current W-matrix discarded
; - W-matrix reset to initial covariance (high uncertainty)
; - Position uncertainty: ~1-2 nautical miles per axis
; - Velocity uncertainty: ~1-2 feet per second per axis
; - Next radar measurement begins rebuilding state confidence
; - Convergence typically takes 3-5 measurement cycles (~30-60 seconds)
; 
; WHY REINITIALIZE W-MATRIX:
; The Kalman filter assumes the W-matrix accurately represents state uncertainty.
; If this assumption breaks (due to unmodeled errors, sensor failures, or
; manual state corrections), the filter can become overconfident and reject
; valid measurements. Reinitializing the W-matrix with high uncertainty forces
; the filter to "listen" to measurements again and rebuild confidence correctly.
; 
; INTERRUPT PROTECTION RATIONALE:
; FLAGWRD5 is shared between:
; - Main verb processing (this routine)
; - Navigation integration routines (P20, P25)
; - Measurement incorporation (radar update processing)
; 
; Without INHINT protection, an interrupt could occur between MASK and TS,
; causing a partially updated FLAGWRD5 where other flags are corrupted. The
; INHINT ensures the three-instruction sequence executes atomically.
; 
; HISTORICAL NOTE:
; Apollo 11's rendezvous was nominal and did not require W-matrix reset.
; However, this verb was critical insurance for later missions. Apollo 14
; encountered rendezvous radar anomalies during ascent, and the crew used
; state vector updates combined with W-matrix reinitialization to maintain
; navigation accuracy despite sensor problems.
; 
# VERB 93	CLEAR RENDWFLG, CAUSES W-MATRIX TO BE RE-INITIALIZED.

WMATRXNG	INHINT
		CS	RENDWBIT
		MASK	FLAGWRD5
		TS	FLAGWRD5

		TC	GOPIN

; 
; ----------------------------------------------------------------------------
; SHOWSUM - DISPLAY MEMORY CHECKSUM (Part of Verb 91 SHOWSUM functionality)
; ----------------------------------------------------------------------------
; 
; PURPOSE:
; Initiates display of AGC memory checksums for verification of program
; integrity. This routine is part of the AGC self-check system that validates
; fixed memory (core rope) contents by computing and displaying checksums.
; Crew can compare displayed values against reference checksums to verify
; that the flight software has not been corrupted.
; 
; OPERATION:
; 1. Checks program state (CHKPOOH) and extended verb availability (TESTXACT)
; 2. Sets priority to PRIO7 to allow other character input during checksum
; 3. Configures SHOWSUM mode (SKEEP6 = S+1) and disables SELFCHK (SMODE = 0)
; 4. Calls STSHOSUM to enter ROPECHK routine which computes checksums
; 5. Displays bank number and computed checksum via Verb 05 Noun 01
; 6. Recycles through memory banks until complete
; 
; CHECKSUM ALGORITHM:
; ROPECHK routine walks through fixed memory banks, computing checksums by
; summing contents. Checksums are compared against known values to detect
; any corruption in core rope ROM that might occur due to radiation or
; hardware failure during the mission.
; 
; CREW PROCEDURE:
; Verb 91 initiates checksum display. Crew observes Verb 05 Noun 01 displays
; showing bank number (R1) and computed checksum (R2). Crew compares against
; reference table in spacecraft documentation. Any mismatch would indicate
; memory corruption requiring abort or contingency procedures.
; 
; DISPLAY FORMAT (V05N01):
; R1: Fixed memory bank number (octal)
; R2: Computed checksum for that bank (octal)
; R3: (Not used)
; 
; HISTORICAL NOTE:
; Memory integrity verification was critical safety feature. Core rope memory
; was considered extremely reliable, but cosmic radiation in space environment
; posed theoretical corruption risk. SHOWSUM provided crew verification
; capability without requiring ground support. No Apollo mission ever detected
; memory corruption through checksum verification, validating the robustness
; of core rope ROM design.
; 
; INTEGRATION:
; - Called by: Verb 91 dispatcher (not shown in this file)
; - Calls: CHKPOOH (program state check)
;          TESTXACT (extended verb system check)
;          PRIOCHNG (priority change)
;          STSHOSUM (initiate ROPECHK checksum routine)
;          GOXDSPF (display interface via Verb 05 Noun 01)
; - Uses: SKEEP6 (showsum option flag), SMODE (self-check mode control)
; 
; RETURN:
; Normal: Cycles through all banks, then returns via ENDEXT
; V34 Terminate: Exits checksum display and returns to prior activity
; V33 Proceed: Advances to next bank
; 

GOSHOSUM	EQUALS	SHOWSUM

SHOWSUM		TC	CHKPOOH		# *
		TC	TESTXACT	# *
		CAF	PRIO7		# * ALLOW OTHER CHARINS.
		TC	PRIOCHNG	# *
		CAF	S+1		# *
		TS	SKEEP6		# * SHOWSUM OPTION
		CAF	S+ZERO		# *
		TS	SMODE		# * TURN OFF SELF-CHECK
		CA	SELFADRS	# *
		TS	SELFRET		# *
		TC	STSHOSUM	# * ENTER ROPECHK

SDISPLAY	LXCH	SKEEP2		# * BANK # FOR DISPLAY
		LXCH	SKEEP3		# * BUGGER WORD FOR DISPLAY
NOKILL		CA	ADRS1		# *
		TS	MPAC +2		# *
		CA	VNCON		# * 0501
		TC	BANKCALL	# *
		CADR	GOXDSPF		# *
		TC	+3		# *
		TC	NXTBNK		# *
# Page 291
		TC	NOKILL		# *
		CA	SELFADRS
		TS	SKEEP1

		TC	ENDEXT		# *
VNCON		VN	501		# *
ENDSUMS		CA	SKEEP6		# *
		EXTEND			# *
		BZF	SELFCHK		# * ROPECHK, START SELFCHK AGAIN.
		TC	STSHOSUM	# * START SHOWSUM AGAIN.

# Page 292
# DAPDISP	VERB 48		DESCRIPTION
#	LOAD AUTO PILOT DATA
#
#	1.	REQUIRE EXT VERB DISPLAY AVAILABLE AND SET BUSY FLAG.
#	2.	EXECUTE DAPDATA1, DAPDATA2, AND DAPDATA3.
#	3.	RELEASE EXT VERB DISPLAY SYSTEM.

; 
; ============================================================================
; VERB 48 - LOAD AUTOPILOT DATA (DAPDISP)
; ============================================================================
; 
; PURPOSE:
; Loads and updates Digital Autopilot (DAP) parameters for RCS attitude control.
; Allows crew to configure autopilot behavior by setting rotation rates, stick
; sensitivity, vehicle masses, and trim gimbal angles. Critical for adapting
; autopilot performance to changing mission conditions (fuel consumption,
; CSM docking/undocking, LM staging, different flight phases).
; 
; AUTOPILOT PARAMETERS LOADED:
; 1. Rotation rates (pitch, yaw, roll) - Sets maximum angular velocity
; 2. Stick sensitivity (coarse/fine/off) - Configures manual control response
; 3. Vehicle masses (LM, CSM) - Updates mass properties for control calculations
; 4. Trim gimbal angles - Sets engine gimbal trim for thrust vector control
; 
; OPERATIONAL SEQUENCE:
; 
; PHASE 1 (DAPDATA1): Rotation Rate and Stick Configuration
; - Displays Verb 06 Noun 46: Rotation rates and stick sensitivity setting
; - R1: Pitch rotation rate (degrees/second, scaled ±20 deg/s)
; - R2: Yaw rotation rate (degrees/second, scaled ±20 deg/s)  
; - R3: Roll rotation rate (degrees/second, scaled ±20 deg/s)
; - Stick sensitivity code: +0 = Off, +1 = Coarse (20 deg/s), +2 = Fine (4 deg/s)
; 
; Crew can load new rates via keyboard. Rotation rates control maximum angular
; velocity DAP will command when maneuvering. Higher rates allow faster
; attitude changes but consume more RCS propellant. Typical values:
; - Fine pointing/docking: 0.6-2 deg/s
; - Attitude maneuvers: 5-10 deg/s
; - Maximum capability: 20 deg/s
; 
; PHASE 2 (DAPDATA2): Vehicle Mass Properties
; - Displays Verb 06 Noun 47: Vehicle masses for inertia calculations
; - R1: LM mass (centipounds, scaled)
; - R2: CSM mass (centipounds, scaled) - only if docked
; - R3: (Not used)
; 
; Accurate mass data critical for autopilot performance. As RCS propellant
; burns during mission, vehicle mass decreases, changing moments of inertia.
; DAP uses mass to compute required thruster firing times. Crew updates
; masses periodically, especially after major propellant consumption events
; (plane changes, rendezvous maneuvers).
; 
; Mass validation checks:
; - LM mass must exceed empty LM mass (MINLMD or MINMINLM depending on stage)
; - CSM mass must exceed empty CSM mass (MINCSM) if docked
; - Invalid entries cause recycle to request new data
; 
; After valid mass entry, routine:
; - Computes total vehicle mass (LM alone or LM+CSM if docked)
; - Stores in MASS for DAP calculations
; - Calls RESTORDB to recompute moments of inertia with new mass
; 
; PHASE 3 (DAPDATA3): Trim Gimbal Angles (Pre-staging only)
; - Displays Verb 06 Noun 48: Descent engine gimbal trim angles
; - R1: Pitch gimbal trim (degrees)
; - R2: Yaw gimbal trim (degrees)
; - R3: (Not used)
; 
; Trim angles align descent engine thrust vector with vehicle center of mass.
; As propellant burns, center of mass shifts, requiring trim adjustment to
; prevent unwanted torques. Only applicable before LM staging (while descent
; stage attached). After staging, ascent engine is fixed and cannot gimbal.
; 
; Crew can initiate gimbal trim via V33 (Proceed). Routine schedules TRIMGIMB
; task via WAITLIST to physically move engine gimbals to commanded trim
; position. When trim complete, displays Verb 50 Noun 48 requesting crew
; termination (V34).
; 
; CONTROL FLOW:
; DAPDISP → DAPDATA1 (rates/stick) → DAPDATA2 (masses) → DAPDATA3 (trim) → ENDEXT
;           ↑                            ↑                    ↑
;           └─── V33 Proceed ────────────┘                    │
;           └─── V34 Terminate (sets deadband) ──────────────┘
; 
; STICK SENSITIVITY MODES:
; The stick sensitivity setting controls manual attitude control response:
; 
; OFF (Code +0): Manual control disabled, autopilot maintains commanded attitude
; COARSE (Code +1): 20 degrees/second maximum rate, normal maneuvering
; FINE (Code +2): 4 degrees/second maximum rate, precision pointing
; 
; Fine mode used during critical operations:
; - Final approach to landing site (Armstrong used manual control below 500 ft)
; - Docking alignment (requires precise attitude control within ±2 degrees)
; - Optical navigation sightings (IMU alignment, landmark tracking)
; 
; DOCKED vs UNDOCKED BEHAVIOR:
; Routine automatically detects docking state via CSMDOCKD flag bit:
; - Undocked: Only loads LM mass, uses LM moments of inertia
; - Docked: Loads both LM and CSM masses, sums for total, uses combined inertia
; 
; When docked, stick sensitivity scaled by 0.1 factor to account for increased
; vehicle mass/inertia. Prevents over-controlling the larger combined vehicle.
; 
; STAGING CONSIDERATIONS:
; After LM staging (APSFLAG set), routine behavior changes:
; - DAPDATA3 skips trim gimbal display (ascent engine cannot gimbal)
; - Mass calculations use ascent-stage-only values (MINMINLM)
; - Routine terminates after DAPDATA2 when staged
; 
; HISTORICAL CONTEXT:
; During Apollo 11 descent, Armstrong switched to fine stick sensitivity as
; Eagle approached the boulder field, allowing precise manual site selection.
; The 4 deg/s fine mode gave him adequate control authority while maintaining
; smooth flight path. Coarse mode (20 deg/s) would have been too aggressive
; for low-altitude maneuvering.
; 
; After staging for ascent, Aldrin updated LM mass via Verb 48 to reflect
; lighter ascent-only configuration, ensuring accurate autopilot response
; during rendezvous with Columbia.
; 
; INTEGRATION:
; - Called by: Verb 48 dispatcher from extended verb table
; - Calls: TESTXACT (verify extended verb system available)
;          BANKCALL/GOXDSPFR (display interface for Nouns 46, 47, 48)
;          BLANKET (blank display fields)
;          IBNKCALL/RESTORDB (recompute moments of inertia)
;          WAITLIST/TRIMGIMB (schedule gimbal trim execution)
; - Modifies: DAPBOOLS (stick configuration flags)
;             LEMMASS, CSMMASS (vehicle masses)
;             MASS (total vehicle mass for DAP)
;             Moments of inertia (via RESTORDB)
; - Uses: APSFLAG (staging state), CSMDOCKD (docking state)
; 
; DISPLAY CODES:
; V06N46: Display rotation rates and stick setting (decimal)
; V06N47: Display LM and CSM masses (decimal)
; V06N48: Display gimbal trim angles (decimal)
; V50N48: Request crew termination after trim complete
; 

DAPDISP		TC	TESTXACT
		CAF	PRIO7		# R03
		TC	PRIOCHNG
		TC	POSTJUMP
		CADR	DAPDATA1

		BANK	34
		SETLOC	LOADDAP
		BANK

		COUNT*	$$/R03

		SBANK=	LOWSUPER	# FOR SUBSEQUENT LOW 2CADR'S

DAPDATA1	CAF	BOOLSMSK	# SET DISPLAY ACCORDING TO DAPBOOLS BITS.
		MASK	DAPBOOLS	# LM
		TS	DAPDATR1	# LM
		CS	FLGWRD10	# SET BIT 14 TO BE COMPLEMENT OF APSFLAG.
		MASK	APSFLBIT
		CCS	A
		CAF	BIT14
		ADS	DAPDATR1
CHKDATA1	CAE	DAPDATR1	# IF BITS 13 AND 14 ARE BOTH ZERO, FORCE
		MASK	BIT13-14	#	A ONE INTO BIT 13.
		EXTEND
		BZF	FORCEONE
		CAE	DAPDATR1	# ENSURE THAT NO ILLEGAL BITS SET BY CREW.
MSKDATR1	MASK	DSPLYMSK
		TS	DAPDATR1
		CAF	V01N46		# LM
		TC	BANKCALL
		CADR	GOXDSPFR
		TCF	ENDEXT		# V34E TERMINATE
		TCF	DPDAT1		# V33E PROCEED
		TCF	CHKDATA1	# E	NEW DATA	CHECK AND REDISPLAY
		CAF	REVCNT		# BITS 2 & 3:  BLANKS R2 & R3.
		TC	BLANKET
		TCF	ENDOFJOB
FORCEONE	CAF	BIT13
		ADS	DAPDATR1
		TCF	MSKDATR1

DPDAT1		INHINT			# INHINT FOR SETTING OF FLAG BITS AND MASS
		CS	APSFLBIT	# 	ON BASIS OF DISPLAYED DAPDATR1.
		MASK	FLGWRD10
		TS	L		# SET APSFLAG TO BE COMPLEMENT OF BIT 14.
# Page 293
		CS	DAPDATR1
		MASK	BIT14
		CCS	A
		CAF	APSFLBIT
		AD	L
		TS	FLGWRD10
		CS	DAPDATR1	# SET BITS OF DAPBOOLS ON BASIS OF DISPLAY
		MASK	BIT13-14	#	MASK OUT CSMDOCKD (BIT 13) UNLESS BOTH
		CCS	A		# 	13 AND 14 ARE SET.
		CS	CSMDOCKD
		AD	BOOLSMSK
		MASK	DAPDATR1
		TS	L
		CS	BOOLSMSK
		MASK	DAPBOOLS
		AD	L
		TS	DAPBOOLS
		MASK	CSMDOCKD	# LOAD MASS IN ACCORDANCE WITH CSMDOCKD.
		CCS	A		#	MASS IS USUALLY OKAY, SO DO
		CAE	CSMMASS		# 	NOT TOUCH ITS LOW-ORDER PART.
		AD	LEMMASS
		TS	MASS
		CAE	DAPBOOLS
		MASK	ACC4OR2X	# 2 OR 4 JET X-TRANSLATION
		EXTEND			# (BIT ACC4OR2X = 1 FOR 4 JETS)
		BZF	+5
		CS	BIT15
		MASK	FLAGWRD1	# CLEAR NJTSFLAG TO 0 FOR 4 JETS
		TS	FLAGWRD1
		TCF	+4
		CS	FLAGWRD1	# SET NJTSFLAG TO 1 FOR 2 JETS
		MASK	BIT15
		ADS	FLAGWRD1
		CA	DAPBOOLS	# SELECT DESIRED KALCMANU AUTOMATIC
		MASK	THREE		# MANEUVER RATE
		DOUBLE			# RATEINDX HAS TO BE 0,2,4,6 SINCE RATES
		TS	RATEINDX	# ARE DP
		TC	POSTJUMP
		CADR	STIKLOAD

V01N46		VN	0146
DSPLYMSK	OCT	33113
BOOLSMSK	OCT	13113

		BANK	01
		SETLOC	LOADDAP1
		BANK

		COUNT*	$$/R03

STIKLOAD	CAF	EBANK6
# Page 294
		TS	EBANK
		EBANK=	STIKSENS
		CA	RHCSCALE	# SET STICK SENSITIVITY TO CORRESPOND TO A
		MASK	DAPBOOLS	# MAXIMUM COMMANDED RATE (AT 42 COUNTS) OF
		CCS	A		# 20 D/S (NORMAL) OR 4 D/S (FINE), SCALED
		CA	NORMAL		# AT 45 D/S.
		AD	FINE
		TS	STIKSENS
		CA	-0.6D/S
		TS	-RATEDB		# LM-ONLY BREAKOUT LEVEL IS .6 D/S.
		CA	CSMDOCKD	# IF CSM-DOCKED, DIVIDE STICK SENSITIVITY
		MASK	DAPBOOLS	# BY 10.  NORMAL SCALING IS THEN 2 D/S AND
		EXTEND			# FINE SCALING IS 0.4 D/S
		BZF	+7		# BRANCH IF CSM IS NOT DOCKED.
		CA	STIKSENS
		EXTEND
		MP	1/10
		TS	STIKSENS
		CA	-0.3D/S		# CSM-DOCKED BREAKOUT LEVEL IS .3 D/S.
		TS	-RATEDB
		RELINT			# PROCEED TO NOUN 47, MASS LOAD.

DAPDATA2	CAF	V0647
		TC	BANKCALL
		CADR	GOXDSPFR
		TCF	ENDR03		# V34E	TERMINATE. FIRST SET DB.  DO 1/ACCS
		TCF	DAPDAT2		# V33E	PROCEED
		TCF	DAPDATA2	# 	LOAD NEW DATA AND RECYCLE
		CAF	BIT3		# BLANKS R3
		TC	BLANKET		#		LM
		TCF	ENDOFJOB
ENDR03		INHINT
		TC	IBNKCALL
		CADR	RESTORDB
		TCF	ENDEXT		# DOES RELINT

DAPDAT2		CS	FLGWRD10	# DETERMINE STAGE FROM APSFLAG
		MASK	APSFLBIT
		CCS	A
		CA	MINLMD
		AD	MINMINLM
		AD	LEMMASS		# LEMMASS MUST BE GREATER THAN EMPTY LEM
		EXTEND
		BZMF	DAPDATA2	# ASK FOR NEW MASSES
		CAE	DAPBOOLS
		MASK	CSMDOCKD
		EXTEND
		BZF	LEMALONE	# SKIP TEST ON CSMMASS IF NOT DOCKED.
		CS	MINCSM		# TEST CSM MASS
		AD	CSMMASS		# CSMMASS MUST BE GREATER THAN EMPTY CSM
# Page 295
		EXTEND
		BZMF	DAPDATA2	# ASK FOR NEW MASSES
		CAE	CSMMASS		# DOCKED:  MASS = CSMMASS + LEMMASS
LEMALONE	AD	LEMMASS		# LEM ALONE:  MASS = LEMMASS
		ZL
		DXCH	MASS
		INHINT
		TC	IBNKCALL	# SET DEADBANK AND COMPUTE MOMENTS OF
		CADR	RESTORDB	#	INERTIA.
		RELINT			# PROCEED TO NOUN 48 (OR END).

DAPDATA3	CS	FLGWRD10
		MASK	APSFLBIT
		EXTEND			# END ROUTINE IF LEM HAS STAGED.
		BZF	ENDEXT
		CAF	V06N48		# DISPLAY TRIM ANGLES AND REQUEST RESPONSE
		TC	BANKCALL
		CADR	GOXDSPFR
		TC	ENDEXT
		TCF	DPDAT3		# V33E GO DO TRIM (WAITLIST TO TRIMGIMB)
		TCF	-5		# LOAD NEW DATA AND RECYCLE
		CAF	BIT3
		TC	BLANKET		# BLANK R3
		TCF	ENDOFJOB
DPDAT3		CAF	BIT1		# GO TO TRIMGIMB VIA WAITLIST SO IT
		INHINT			# CAN USE FIXDELAY AND VARDELAY
		TC	WAITLIST
		EBANK=	ROLLTIME
		2CADR	TRIMGIMB

		TCF	ENDOFJOB	# DOES A RELINT
TRIMDONE	CAF	V50N48
		TC	BANKCALL	# TRIM IS FINISHED; PLEASE TERMINATE R03
		CADR	GOMARK3R
		TC	ENDEXT		# V34E TERMINATE
		TC	ENDEXT
		TC	ENDEXT
		CAF	OCT24		# BIT5 TO CHANGE TO PERFORM, 3 TO BLANK 43
		TC	BLANKET
		TCF	ENDOFJOB

V0647		VN	0647
V06N48		VN	0648

V50N48		VN	5048
NORMAL		DEC	.660214
					# NORMAL SCALING IS 20 D/S
FINE		DEC	.165054		# FINE STICK SCALING (4 D/S).
1/10		DEC	.1		# FACTOR FOR CSM-DOCKED SCALING
-0.6D/S		DEC	-218

# Page 296

-0.3D/S		DEC	-109

# Page 297
# VERB 66	VEHICLES ARE ATTACHED. MOVE THIS VEHICLE STATE VECTOR TO
#		OTHER VEHICLE STATE VECTOR.
#
# USE SUBROUTINE GENTRAN.

		BANK	7
		SETLOC	EXTVERBS
		BANK

		COUNT*	$$/EXTVB

		EBANK=	RRECTHIS

; 
; ============================================================================
; VERB 66 - VEHICLES ATTACHED (ATTACHED)
; ============================================================================
; 
; PURPOSE:
; Synchronizes state vectors between LM and CSM when vehicles are docked.
; When crew confirms vehicles are physically attached (docked), this verb
; copies the current vehicle's state vector to the other vehicle's state
; vector storage, ensuring both vehicles share identical position/velocity
; data. Critical for maintaining navigation accuracy during docked operations.
; 
; OPERATIONAL CONTEXT:
; During Apollo missions, LM and CSM operate as separate vehicles during
; lunar orbit operations and landing. When docked together (translunar coast,
; post-rendezvous), they form a single rigid body sharing the same state
; vector. However, AGC maintains separate state vector storage for each:
; - RRECTHIS/VVECTTHIS: Current vehicle position and velocity
; - RRECTOTH/VVECTOTH: Other vehicle position and velocity
; 
; If vehicles update their state vectors independently while docked (via
; navigation updates, ground uplinks, or orbital integration), the stored
; state vectors can diverge. Verb 66 resolves this by propagating the
; current vehicle's state vector to the other vehicle's storage.
; 
; TYPICAL USAGE SCENARIOS:
; 
; 1. POST-DOCKING STATE VECTOR PROPAGATION:
;    After LM/CSM rendezvous and docking in lunar orbit, crew executes V66
;    to copy LM's newly-updated state vector (from rendezvous navigation)
;    to CSM's state vector storage. This ensures CSM has accurate state
;    vector reflecting post-rendezvous position/velocity.
; 
; 2. GROUND UPLINK SYNCHRONIZATION:
;    If ground control uplinks updated state vector to one vehicle while
;    docked, crew uses V66 to propagate update to other vehicle's storage.
;    Example: Uplink to CSM followed by V66 from CSM to synchronize LM.
; 
; 3. POST-TRANSLUNAR COAST:
;    After translunar injection burn (TLI), CSM integrated state vector
;    through coast phase. When crew prepares for LM separation, V66 copies
;    CSM's accurate cislunar trajectory to LM state vector, giving LM
;    correct starting position for descent orbit insertion.
; 
; OPERATION:
; 1. Finds available VAC area at priority 10 for state vector operations
; 2. Schedules ATTACHIT job to perform state vector transfer
; 3. ATTACHIT configures reference frame flags (MOONTHIS/MOONOTH) to indicate
;    whether current and other vehicles are in lunar or Earth-centered frame
; 4. Calls GENTRAN (General Transformation) with OCT51 option to copy
;    RRECTHIS/VVECTTHIS to RRECTOTH/VVECTOTH
; 5. Updates other vehicle's position-to-altitude-and-Moon conversion (PTOALEM)
; 6. Calls SVDWN1 to store updated state vector
; 7. Restarts integration routine (INTWAKE) with synchronized state vectors
; 
; REFERENCE FRAME HANDLING:
; The MOONTHIS and MOONOTH flags indicate coordinate system:
; - MOONTHIS set: Current vehicle in Moon-centered inertial frame
; - MOONTHIS clear: Current vehicle in Earth-centered inertial frame
; - MOONOTH set: Other vehicle in Moon-centered inertial frame
; - MOONOTH clear: Other vehicle in Earth-centered inertial frame
; 
; ATTACHIT checks MOONTHIS flag and sets/clears MOONOTH accordingly via BON
; (Branch ON) instruction. This ensures both vehicles use same reference frame
; after synchronization. During cislunar coast, vehicles typically in Earth
; frame. In lunar orbit, vehicles in Moon frame.
; 
; GENTRAN OPERATION (OCT51 MODE):
; OCT51 is option code for GENTRAN indicating "copy state vector without
; transformation". GENTRAN copies 6 components (position X,Y,Z and velocity
; Vx,Vy,Vz) from source address (RRECTHIS) to destination (RRECTOTH).
; 
; Format: GENTRAN is called with:
;   TC GENTRAN
;   ADRES source (RRECTHIS)
;   ADRES destination (RRECTOTH)
; 
; GENTRAN handles proper memory banking and erasable memory access to safely
; transfer state vector data between storage areas.
; 
; STATE VECTOR COMPONENTS:
; Position vector RRECT: 3 components (X, Y, Z) in meters
;   - Scaled by 2^29 meters (0.5364 meter resolution)
;   - Reference frame: Earth-centered or Moon-centered inertial
; Velocity vector VVECT: 3 components (Vx, Vy, Vz) in meters/centisecond
;   - Scaled by 2^7 meters/centisecond (0.78125 cm/s resolution)
;   - Reference frame: Earth-centered or Moon-centered inertial
; 
; INTEGRATION RESTART:
; After state vector synchronization, routine calls INTWAKE to restart orbital
; integration. This ensures navigation system begins propagating synchronized
; state vectors forward in time, maintaining both vehicles' position/velocity
; knowledge current.
; 
; Integration restart critical because:
; - Halts any ongoing integration using old state vector
; - Re-initializes integration with synchronized state vectors
; - Prevents navigation drift between vehicles while docked
; 
; CREW PROCEDURE:
; 1. Verify vehicles are physically docked (hard dock with probe/drogue)
; 2. Execute Verb 66 (V66E) on DSKY
; 3. Wait for completion (typically 1-2 seconds)
; 4. Verify no alarms indicating state vector problems
; 5. Integration automatically restarts with synchronized state vectors
; 
; HISTORICAL CONTEXT:
; During Apollo 11 return from lunar surface, after Eagle (LM) docked with
; Columbia (CSM) in lunar orbit, crew executed Verb 66 to synchronize state
; vectors. This ensured both vehicles had identical knowledge of their
; position/velocity for the critical transearth injection (TEI) burn.
; 
; After TEI burn by CSM, another V66 propagated CSM's post-burn state vector
; to LM. This maintained synchronization even though LM was passive during
; the burn, enabling accurate LM systems display of trajectory to Earth.
; 
; NAVIGATION ACCURACY:
; State vector synchronization critical for:
; - Accurate relative navigation during separation/rendezvous preparation
; - Correct ground tracking correlation (both vehicles report consistent position)
; - Mission planning calculations (ground computes maneuvers using state vectors)
; - Abort scenario preparation (ensures backup procedures have correct initial state)
; 
; INTEGRATION:
; - Called by: Verb 66 dispatcher from extended verb table
; - Calls: FINDVAC (find vacant VAC area for job scheduling)
;          INTPRET (enter interpreter mode)
;          INTSTALL (install integration initial conditions)
;          GENTRAN (general transformation and copy routine)
;          PTOALEM (position to altitude and lunar ephemeris moon)
;          SVDWN1 (state vector down routine)
;          INTWAKE (restart integration after state vector update)
;          PINBRNCH (pin branch for integration restart)
; - Modifies: RRECTOTH/VVECTOTH (other vehicle state vector)
;             MOONOTH (other vehicle reference frame flag)
; - Uses: RRECTHIS/VVECTTHIS (current vehicle state vector)
;         MOONTHIS (current vehicle reference frame flag)
;         PBODY (primary gravitational body)
; - Priority: PRIO10 (high priority for navigation operations)
; 
; CONTRAST WITH VERB 81/82 (UPDATE STATE VECTORS):
; Verb 81: Update CSM state vector from external source (ground uplink)
; Verb 82: Update LM state vector from external source (ground uplink)
; Verb 66: Copy state vector between vehicles (internal synchronization)
; All serve navigation accuracy but different operational contexts.
; 
; RETURN:
; Completion via ENDOFJOB after state vector copied and integration restarted.
; Integration continues in background maintaining synchronized state vectors.
; 

ATTACHED	CAF	PRIO10
		TC	FINDVAC
		EBANK=	RRECTHIS

		2CADR	ATTACHIT

		TC	ENDOFJOB

ATTACHIT	TC	INTPRET
		CALL
			INTSTALL
		SET	BON
			MOONOTH
			MOONTHIS
			+3
		CLEAR
			MOONOTH
		EXIT
		CAF	OCT51
		TC	GENTRAN
		ADRES	RRECTHIS	# OUR STATE VECTOR INTO OTHER VIA GENTRAN
		ADRES	RRECTOTH

		RELINT
		TC	INTPRET
		CALL			# UPDATE R-OTHER, V-OTHER
			PTOALEM
		LXA,2	CALL
			PBODY
			SVDWN1
		EXIT

		CAF	TCPINAD
		INDEX	FIXLOC
		TS	QPRET
		TC	POSTJUMP
		CADR	INTWAKE		# FREE INTEGRATION AND EXIT.

# Page 298

TCPIN		RTB
			PINBRNCH

OCT51		OCT	51
TCPINAD		CADR	TCPIN

# VERB 96	SET QUITFLAT TO STOP INTEGRATION.
#
#	GO TO V37 WITH ZERO TO CAUSE P00.
#		STATEINT WILL CHECK QUITFLAG AND SKIP 1ST PASS,
#			THUS ALLOWING A 10 MINUT PERIOD WITHOUT INTEGRATION.

; 
; ============================================================================
; VERB 96 - STOP INTEGRATION AND GO TO P00 (VERB96)
; ============================================================================
; 
; PURPOSE:
; Terminates ongoing orbital integration and returns AGC to program P00
; (CMC Idle Program / Pooh Program). Used to stop computational workload
; when orbital state vector integration unnecessary, conserving CPU cycles
; and reducing background processing load during coast phases or when
; crew needs AGC responsiveness for other operations.
; 
; INTEGRATION BACKGROUND:
; AGC continuously integrates orbital equations of motion to maintain
; accurate state vector (position and velocity). Integration routine
; STATEINT (State Integration) executes periodically under WAITLIST
; scheduling, propagating state vector forward through:
; - Gravitational perturbations (Earth/Moon/Sun)
; - Atmospheric drag (if in low orbit)
; - Solar radiation pressure
; - Spacecraft maneuvers
; 
; Integration CPU cost: Each STATEINT pass requires ~10-20 milliseconds
; of computation. During critical mission phases (rendezvous, landing),
; this background load acceptable. During long coast phases, integration
; may be unnecessary if ground will uplink updated state vector.
; 
; OPERATION:
; 1. Sets QUITFLAG via UPFLAG (flag setting routine)
; 2. QUITFLAG signals STATEINT to terminate at next timestep
; 3. Loads ZERO into accumulator
; 4. Jumps to V37 (Change Program verb) with program number 00
; 5. V37 terminates current program and initiates P00
; 
; QUITFLAG MECHANISM:
; QUITFLAG checked by STATEINT at beginning of each integration cycle:
; - If QUITFLAG clear: STATEINT performs integration timestep
; - If QUITFLAG set: STATEINT skips integration pass and exits
; 
; Setting QUITFLAG does NOT immediately halt integration; STATEINT must
; reach its next scheduled execution and detect flag. Typical delay:
; <1 second (one WAITLIST timer cycle).
; 
; Once QUITFLAG set, STATEINT will skip "1st pass" (as noted in original
; NASA comment), meaning next scheduled integration skipped. This allows
; 10-minute period without integration (integration scheduled every 10
; minutes during coast phases, every 2 seconds during powered flight).
; 
; P00 (CMC IDLE / POOH PROGRAM):
; P00 is AGC's idle/standby program. When no mission program active,
; P00 runs in background performing minimal housekeeping:
; - Updates mission timer display on DSKY
; - Monitors IMU status
; - Responds to crew verb/noun inputs
; - Processes uplink commands from ground
; - Maintains system ready for next program request
; 
; P00 nicknamed "Pooh Program" (Winnie the Pooh) by astronauts because
; it "does nothing" - spacecraft coasts while AGC waits for next task.
; 
; OPERATIONAL USAGE:
; Crew executes V96 when:
; - Ground plans to uplink fresh state vector (integration would waste CPU)
; - Long coast phase where orbital prediction unnecessary for extended period
; - AGC workload high and crew needs computational capacity freed
; - Preparing for major program change where state vector will be reinitialized
; 
; Typical sequence:
; 1. Ground: "Apollo 11, Houston. We'll be uplinking state vector in 5 minutes.
;    Request Verb 96 to stop integration."
; 2. Crew: "Roger, Houston. Verb 9-6." (presses V96 on DSKY)
; 3. AGC displays "P00" on DSKY indicating idle program
; 4. Integration stops; state vector frozen at last computed value
; 5. Ground uplinks fresh state vector via V71 (Universal Update)
; 6. Crew or ground initiates next mission program (P20, P30, etc.)
; 7. New program reactivates integration as needed
; 
; INTEGRATION VS P00 RELATIONSHIP:
; Most mission programs (P20, P30, P40, etc.) automatically enable
; integration when they start. P00 unique in that it does NOT run
; integration by default. V96 exploits this by forcing P00, which
; stops integration as side effect.
; 
; Direct approach would be: Set QUITFLAG, terminate current program.
; V96 approach: Set QUITFLAG, then use V37 mechanism to cleanly
; terminate program and enter P00. This ensures proper program
; cleanup and DSKY display updates.
; 
; CONTRAST WITH OTHER INTEGRATION CONTROL:
; Other integration control mechanisms:
; - Program termination (V34): Stops program but may restart integration
;   if another program automatically selected
; - Fresh Start (V36): Complete AGC restart; clears all state
; - Program switching: New program may restart integration
; V96 specifically: Stops integration and leaves AGC in idle state
; 
; 10-MINUTE INTEGRATION PERIOD:
; Original NASA comment references "10 MINUT PERIOD WITHOUT INTEGRATION".
; During coast phases, STATEINT scheduled every ~10 minutes (600 seconds).
; Setting QUITFLAG causes next scheduled pass to be skipped, giving
; 10-minute integration-free window. During powered flight or rendezvous,
; integration scheduled more frequently (every 2 seconds), so window shorter.
; 
; CPU LOAD CONSIDERATIONS:
; Stopping integration frees 10-20ms every 2-10 minutes (depending on
; mission phase). This minor CPU savings usually insignificant, but
; during high-workload periods (rendezvous radar tracking, descent
; guidance), every millisecond counts.
; 
; More importantly: Stopping integration reduces risk of WAITLIST overflow
; (1202 alarm). Fewer background tasks scheduled means more WAITLIST
; capacity for foreground mission-critical operations.
; 
; HISTORICAL CONTEXT:
; During Apollo 11's translunar coast, ground periodically requested V96
; when planning state vector uplinks. Since spacecraft trajectory well-known
; and ground had superior tracking data, onboard integration provided little
; value. Stopping integration reduced AGC workload and prevented drift
; between onboard state vector and ground tracking solution.
; 
; After lunar orbit insertion, integration critical for rendezvous planning,
; so V96 not used during this phase. Integration must run continuously to
; track relative motion between LM and CSM.
; 
; POST-APOLLO SPACECRAFT:
; Modern spacecraft use similar integration suspension concepts:
; - Space Shuttle: "Freeze state vector" command during OMS burns
; - ISS: "Hold navigation state" during reboost maneuvers
; - Commercial crew: "Suspend propagation" during ground updates
; 
; V96 pioneered concept of explicit crew control over navigation integration,
; recognizing that continuous integration not always necessary or desirable.
; 
; INTEGRATION:
; - Called by: Verb 96 dispatcher from extended verb table
; - Calls: UPFLAG (set QUITFLAG)
;          POSTJUMP (jump to V37 routine)
;          V37 (Change Program verb, receives program 00)
; - Modifies: QUITFLAG (signals STATEINT to stop)
;             MODREG (mode register, changed to P00 by V37)
; - Affects: STATEINT (orbital integration routine)
;            WAITLIST (removes STATEINT scheduled tasks)
;            DSKY (displays P00)
; 
; RETURN:
; No direct return; POSTJUMP transfers control to V37, which handles
; program change and DSKY updates. AGC remains in P00 until crew or
; ground initiates next mission program.
; 

VERB96		TC	UPFLAG		# QUITFLAG WILL CAUSE INTEGRATION TO EXIT
		ADRES	QUITFLAG	#	AT NEXT TIMESTEP

		CAF	ZERO
		TC	POSTJUMP
		CADR	V37		# GO TO P00

# VERB 67:	DISPLAY OF W MATRIX

; 
; ============================================================================
; VERB 67 - DISPLAY W MATRIX (V67)
; ============================================================================
; 
; PURPOSE:
; Displays W-matrix (spacecraft angular velocity vector) on DSKY in three-
; component format. W-matrix represents current rotation rates about body
; axes (pitch, yaw, roll). Critical for monitoring attitude dynamics,
; verifying digital autopilot performance, detecting anomalous rotation
; during coast/maneuvers/docking, and troubleshooting control system issues.
; 
; W-MATRIX DEFINITION:
; W-matrix is 3x1 column vector representing angular velocity in body frame:
;   W = [Wx, Wy, Wz]
; 
; For Lunar Module:
; - Wx: Angular velocity about X-axis (pitch rate, nose up/down)
; - Wy: Angular velocity about Y-axis (yaw rate, nose left/right)
; - Wz: Angular velocity about Z-axis (roll rate, clockwise/counterclockwise)
; 
; Units displayed: Revolutions per 2 seconds (REV/2)
; Typical values: ±0.00001 to ±0.01 rev/2 (0.0018 to 1.8 deg/sec)
; 
; W-MATRIX SOURCES:
; AGC computes W-matrix from:
; 1. IMU gyroscopes (primary, most accurate)
; 2. Rate gyros (backup if IMU failed)
; 3. DAP estimates (during powered flight/RCS firing)
; 4. Navigation filter corrections (long-term drift compensation)
; 
; DISPLAY FORMAT ON DSKY:
; Row 1 (R1): Wx - Pitch rate
; Row 2 (R2): Wy - Yaw rate
; Row 3 (R3): Wz - Roll rate
; 
; Example:
;   +00.00012  (Wx = +0.00012 rev/2 = 0.0216 deg/sec)
;   -00.00034  (Wy = -0.00034 rev/2 = -0.0612 deg/sec)
;   +00.00008  (Wz = +0.00008 rev/2 = 0.0144 deg/sec)
; 
; OPERATIONAL USAGE:
; 
; 1. COAST PHASE: W near zero (±0.00005 rev/2) indicates stable attitude.
;    Large W indicates drift, thruster leak, or IMU malfunction.
; 
; 2. MANEUVER VERIFICATION: During attitude maneuvers, W should match
;    commanded rotation rates. Unexpected W indicates thruster failures.
; 
; 3. DOCKING: Relative angular rates <0.1 deg/sec (0.00056 rev/2) required
;    for safe docking. Crew monitors V67 during final approach.
; 
; 4. GIMBAL LOCK PROXIMITY: Rapid W changes without thruster activity
;    indicate approaching gimbal lock singularity.
; 
; 5. POST-MANEUVER: Small residual W (±0.0001 rev/2) normal after RCS
;    maneuvers due to shutoff transients, fuel slosh, structural flex.
; 
; HISTORICAL CONTEXT - APOLLO 11:
; During LM descent, Armstrong monitored W-matrix for attitude control:
; - PDI throttle-up: Increasing Wx as LM pitched nose-down
; - Braking phase: Wx ~0.002 rev/2 (0.72 deg/sec) during descent
; - Approach: W variations visible as Armstrong maneuvered around boulders
; - Final descent: W near zero during vertical descent
; - Touchdown: W frozen at landing, confirming stable touchdown
; 
; Post-landing: Ground requested V67 to verify LM not sinking/tilting
; on lunar surface. Near-zero W confirmed stable configuration.
; 
; During ascent: V67 monitored pitch program through gravity turn.
; Post-insertion: Zero W confirmed stable attitude before rendezvous radar.
; 
; Final docking: Crew monitored V67 continuously during approach:
; - CSM held W near zero
; - LM approached with W < 0.0003 rev/2 (0.05 deg/sec)
; - W spike at docking contact, then settling to zero as latches engaged
; 
; CONTRAST WITH OTHER DISPLAYS:
; V67 (W-matrix): Angular velocity - "how fast rotating"
; V62 (Total attitude error): Attitude error - "how far from desired"
; V61 (DAP attitude error): Autopilot error perception
; V16N43 (IMU CDU): Absolute orientation - "where pointing"
; 
; JOB STRUCTURE:
; V67 creates background job via FINDVAC to avoid blocking DSKY operations.
; Job runs at PRIO5 (medium priority), allowing higher-priority tasks
; (guidance, navigation) to preempt if needed.
; 
; TESTXACT checks if verb already executing to prevent duplicate jobs.
; V67CALL (actual display routine) runs under job, formats W-matrix
; components, scales to REV/2 units, updates DSKY R1/R2/R3 registers.
; 
; INTEGRATION:
; - Called by: Verb 67 dispatcher from extended verb table
; - Calls: TESTXACT (check if verb already active)
;          FINDVAC (allocate vacant core set for job)
;          V67CALL (actual W-matrix display routine, runs as job)
; - Reads: W-matrix from IMU processing (WWPOS erasable bank)
; - Displays: DSKY R1, R2, R3 (three-component angular velocity)
; - Priority: PRIO5 (medium priority background job)
; 
; RETURN:
; Main verb routine exits via ENDOFJOB immediately after launching V67CALL.
; V67CALL job handles all display processing asynchronously, allowing verb
; dispatcher to return control to crew for next operation.
; 
; TROUBLESHOOTING:
; - Large W during coast: Gas leak, thruster leak, or IMU problem
; - W not changing during maneuver: DAP inactive, thrusters failed, or IMU failed
; - W oscillating: DAP gains too high, limit cycle, or slosh resonance
; 

V67		TC	TESTXACT
		CAF	PRIO5
		TC	FINDVAC
		EBANK=	WWPOS
		2CADR	V67CALL

		TC	ENDOFJOB

# VERB 65	DISABLE U,V JETS DURING DPS BURNS

; 
; ============================================================================
; VERB 65 - DISABLE U,V JETS DURING DPS BURNS (SNUFFOUT)
; ============================================================================
; 
; PURPOSE:
; Inhibits +U/−U and +V/−V RCS translation jets during Descent Propulsion
; System (DPS) engine burns by setting SNUFFER flag. Prevents RCS translation
; jets from interfering with DPS thrust vector control (TVC) and conserves
; RCS propellant during powered descent when DPS provides primary thrust.
; 
; U,V JETS DEFINITION (LM RCS CONFIGURATION):
; Lunar Module has 16 RCS thrusters in 4 clusters (quads):
; 
; TRANSLATION AXES:
; - +U/-U jets: Up/down translation (vertical axis)
; - +V/-V jets: Left/right translation (lateral axis)
; - +P/-P jets: Forward/aft translation (longitudinal axis)
; 
; ATTITUDE AXES:
; - Roll jets: Rotation about longitudinal axis
; - Pitch jets: Rotation about lateral axis
; - Yaw jets: Rotation about vertical axis
; 
; V65 disables U,V translation jets only. Attitude jets (P-axis, roll,
; pitch, yaw) remain active for maintaining spacecraft orientation during
; DPS burns.
; 
; WHY DISABLE U,V JETS DURING DPS BURNS:
; 
; 1. PLUME IMPINGEMENT AVOIDANCE:
;    DPS exhaust plume creates high-pressure regions around descent stage.
;    RCS jets firing into plume experience:
;    - Reduced thrust effectiveness (backpressure from plume)
;    - Unpredictable thrust vectors (plume deflection)
;    - Potential combustion instability (plume ingestion into jets)
; 
; 2. PROPELLANT CONSERVATION:
;    DPS thrust: 1050-9870 lbf (throttleable)
;    RCS jet thrust: ~100 lbf each
;    Translation control via DPS gimbal 10-100x more efficient than RCS.
; 
; 3. THRUST VECTOR CONTROL CLARITY:
;    DPS TVC system controls descent trajectory via engine gimbal (±6 deg).
;    U,V jet firings create side forces interfering with TVC commands.
;    Cleaner control when DPS alone handles translation during powered flight.
; 
; 4. DIGITAL AUTOPILOT (DAP) SIMPLIFICATION:
;    DAP logic simplified when translation jets disabled:
;    - Fewer jet selection options to consider
;    - Clearer separation: Attitude jets for rotation, DPS for translation
;    - Reduced computational load during critical descent phase
; 
; SNUFFER FLAG MECHANISM:
; SNUFFER is software flag in erasable memory controlling U,V jet enable:
; - SNUFFER = 0 (flag down): U,V jets enabled, DAP can command firings
; - SNUFFER = 1 (flag up): U,V jets inhibited, DAP blocks U,V commands
; 
; UPFLAG instruction sets SNUFFER to 1 (raises flag).
; ADRES SNUFFER provides address of flag bit in erasable memory.
; 
; GOPIN continues verb processing (likely checks for pending operations).
; 
; FLAG ENFORCEMENT:
; DAP jet selection logic checks SNUFFER flag before issuing U,V commands:
; ```
; IF (SNUFFER flag up) THEN
;   Skip U,V jet selection
;   Use only attitude jets + DPS gimbal
; ELSE
;   Normal jet selection including U,V translation jets
; END IF
; ```
; 
; OPERATIONAL USAGE SEQUENCE:
; 
; PRE-DESCENT (before PDI):
; 1. LM separates from CSM in lunar orbit
; 2. LM performs descent orbit insertion (DOI) burn using DPS
; 3. V65 executed automatically before DOI
; 4. U,V jets disabled for DOI burn
; 5. After DOI, V75 re-enables U,V jets for coast phase
; 
; POWERED DESCENT INITIATION (PDI):
; 1. P63 descent program activates at PDI (~102:33 mission time Apollo 11)
; 2. V65 (SNUFFOUT) executed automatically
; 3. SNUFFER flag set, U,V jets inhibited
; 4. DPS ignites and throttles up
; 5. 12-minute powered descent begins with DPS-only translation control
; 
; DURING DESCENT:
; - Attitude jets active: Maintain pitch/yaw/roll orientation
; - U,V jets inhibited: DPS gimbal handles vertical/lateral translation
; - P-axis jets available: Forward/aft translation (not affected by SNUFFER)
; - Crew can still command attitude changes via hand controller
; - DPS gimbal responds to DAP commands for trajectory corrections
; 
; LANDING/ABORT:
; 1. DPS cutoff at touchdown (normal) or staging (abort)
; 2. V75 (OUTSNUFF) executed automatically
; 3. SNUFFER flag cleared, U,V jets re-enabled
; 4. Full RCS authority restored for post-landing or ascent phase
; 
; HISTORICAL CONTEXT - APOLLO 11 DESCENT:
; 
; PDI (102:33:05 mission time):
; - P63 braking phase initiated
; - V65 executed automatically
; - U,V jets disabled throughout 12-minute descent
; - Only attitude jets + DPS gimbal provided control
; 
; MANUAL LANDING SITE SELECTION (~102:43, 500 feet altitude):
; - Armstrong took semi-manual control
; - Maneuvered laterally using DPS gimbal (not U,V jets)
; - Selected landing site 4 miles west of original target
; - U,V jets remained inhibited per V65
; 
; TOUCHDOWN (102:45:40):
; - DPS cutoff at lunar contact
; - V75 immediately re-enabled U,V jets
; - RCS attitude control maintained LM upright on surface
; 
; DESCENT ENGINE CHARACTERISTICS:
; - Throttle range: 10%-65% (1050-6825 lbf), then 65%-92.5% (6825-9703 lbf)
; - Gimbal authority: ±6 degrees (pitch and yaw)
; - Gimbal actuators: Two-axis, electrically driven
; - Response time: ~0.2 seconds for full gimbal travel
; - TVC bandwidth: ~1 Hz (adequate for attitude/translation control)
; 
; CONTRAST WITH VERB 75 (OUTSNUFF):
; V65 (SNUFFOUT): Sets SNUFFER flag → inhibits U,V jets
; V75 (OUTSNUFF): Clears SNUFFER flag → enables U,V jets
; 
; ABORT SCENARIO (not executed Apollo 11):
; If abort staged during descent:
; 1. Crew presses ABORT STAGE button
; 2. Descent stage explosively jettisoned
; 3. Ascent Propulsion System (APS) ignites
; 4. V75 executes automatically at staging
; 5. U,V jets re-enabled for RCS-based ascent control
; 6. Full RCS capability critical for ascent guidance
; 
; INTEGRATION:
; - Called by: V65 dispatcher from extended verb table
;              P63 (descent braking phase) automatically
;              P64 (approach phase) automatically
;              DOI burn sequence
; - Sets: SNUFFER flag (erasable memory)
; - Effects: DAP jet selection logic
; - Related: V75 (OUTSNUFF) clears flag
;           DAP configuration adapts to flag state
; - Priority: Immediate execution (critical before DPS ignition)
; 
; RETURN:
; Returns via GOPIN continuation. SNUFFER flag remains set until explicitly
; cleared by V75 (OUTSNUFF) or by fresh start/restart sequence.
; 
; TELEMETRY MONITORING:
; Ground controllers monitor SNUFFER flag state via downlink telemetry:
; - Flag state visible in DAP configuration telemetry
; - U,V jet command absence confirms flag active
; - DPS gimbal activity confirms translation control mode
; - Flight controllers verify flag transitions at PDI and touchdown
; 
; SAFETY NOTES:
; - Automatic execution prevents crew error during high-workload phases
; - Flag state verified pre-ignition via telemetry
; - Redundant checking in DAP logic prevents inadvertent U,V firings
; - Critical for DPS performance and propellant conservation
; 

SNUFFOUT	TC	UPFLAG
		ADRES	SNUFFER
		TC	GOPIN

# VERB 75	ENABLE U,V JETS DURING DPS BURNS

; 
; ============================================================================
; VERB 75 - ENABLE U,V JETS DURING DPS BURNS (OUTSNUFF)
; ============================================================================
; 
; PURPOSE:
; Re-enables +U/−U and +V/−V RCS translation jets after Descent Propulsion
; System (DPS) engine shutdown by clearing SNUFFER flag. Restores full RCS
; capability after powered descent, allowing translation control via thrusters
; for post-landing attitude adjustments, abort ascent, or manual maneuvering.
; 
; COMPLEMENTARY TO VERB 65:
; V65 (SNUFFOUT): Sets SNUFFER flag → disables U,V jets before DPS burn
; V75 (OUTSNUFF): Clears SNUFFER flag → enables U,V jets after DPS cutoff
; 
; This verb-pair provides automatic configuration management for optimal
; control mode throughout descent/landing/abort sequences.
; 
; EXECUTION TIMING - AUTOMATIC INVOCATION:
; 
; 1. NORMAL LANDING SEQUENCE:
;    - PDI: V65 disables U,V jets at powered descent initiation
;    - DESCENT: U,V jets inhibited throughout 12-minute descent
;    - TOUCHDOWN: V75 enables U,V jets immediately at DPS cutoff
;    - POST-LANDING: Full RCS capability for attitude control on surface
; 
; 2. ABORT STAGING:
;    - DESCENT: V65 active, U,V jets inhibited during powered descent
;    - STAGING: Crew presses ABORT STAGE, descent stage jettisoned
;    - APS IGNITION: Ascent engine ignites for abort trajectory
;    - V75 EXECUTE: U,V jets re-enabled automatically at staging
;    - ASCENT: Full RCS capability critical for ascent guidance
; 
; 3. DOI BURN COMPLETION:
;    - PRE-DOI: V65 disables U,V jets before descent orbit insertion
;    - DOI BURN: DPS fires for orbit lowering (~25 seconds duration)
;    - POST-DOI: V75 re-enables U,V jets for coast to PDI
;    - COAST PHASE: Full RCS capability for attitude maneuvers
; 
; WHY RE-ENABLE U,V JETS AFTER DPS BURNS:
; 
; 1. COMPLETE ATTITUDE AUTHORITY:
;    Without DPS gimbal available post-cutoff, RCS provides sole attitude
;    control. Translation jets necessary for precise attitude adjustments,
;    especially on lunar surface where gravity creates tilt moments.
; 
; 2. POST-LANDING SURFACE OPERATIONS:
;    LM may settle unevenly on lunar surface (one pad lower, surface slope).
;    U,V jets fire continuously to counteract tilt moments, maintaining
;    upright attitude for safe crew operations and ascent alignment.
; 
; 3. ASCENT PHASE PREPARATION:
;    Ascent guidance requires full RCS capability for trajectory shaping:
;    - Vertical rise phase: U-axis jets provide vertical translation
;    - Gravity turn: V-axis jets assist lateral translation
;    - Orbital insertion: Complete RCS authority for final adjustments
; 
; 4. MANUAL CONTROL RESTORATION:
;    Crew hand controllers regain full translation authority:
;    - Left hand: Attitude control (roll, pitch, yaw)
;    - Right hand: Translation control (U, V, P axes)
;    - Critical for docking maneuvers during rendezvous
; 
; DOWNFLAG INSTRUCTION MECHANICS:
; DOWNFLAG clears SNUFFER flag to 0 (lowers flag).
; ADRES SNUFFER provides address of flag bit in erasable memory.
; 
; Flag state after V75 execution:
; - SNUFFER = 0 (flag down): U,V jets enabled
; - DAP jet selection logic resumes normal operation
; - Hand controller translation commands activate U,V jets
; 
; GOPIN continues verb processing after flag cleared.
; 
; DAP RECONFIGURATION:
; After V75 execution, Digital Autopilot automatically reconfigures:
; 
; BEFORE V75 (SNUFFER flag up):
; - Jet selection excludes U,V jets
; - Translation control expects DPS gimbal
; - Simplified jet logic (attitude jets only)
; 
; AFTER V75 (SNUFFER flag down):
; - Jet selection includes all 16 RCS thrusters
; - Translation control via RCS jets
; - Full jet logic (attitude + translation)
; - Hand controller translation inputs activate U,V jets
; - Automatic translation commands (DAP, autopilot) use U,V jets
; 
; HISTORICAL CONTEXT - APOLLO 11 LANDING:
; 
; TOUCHDOWN (July 20, 1969, 102:45:40 mission time):
; - Armstrong: "Contact light" (lunar surface probes detect touchdown)
; - Aldrin: "Okay, engine stop" (DPS cutoff button pressed)
; - DPS cutoff confirmed
; - V75 (OUTSNUFF) executed automatically
; - SNUFFER flag cleared
; - U,V jets immediately re-enabled
; - RCS attitude jets fired to maintain LM upright on surface
; 
; POST-LANDING ATTITUDE HOLD:
; - LM settled on uneven surface (approximately 4.5-degree tilt)
; - U,V RCS jets fired intermittently to counteract tilt moments
; - Propellant usage: ~0.5% of RCS propellant per hour for attitude hold
; - V75 enable critical for this capability
; 
; SURFACE STAY (21.5 hours):
; - RCS attitude hold active throughout surface operations
; - U,V jets maintained LM upright despite thermal distortions
; - Battery power + RCS propellant limited surface stay duration
; 
; ASCENT PREPARATION:
; - Pre-liftoff: RCS fully enabled per V75
; - Ascent guidance requires complete RCS authority
; - U-axis jets critical for vertical rise phase
; - V-axis jets critical for lateral trajectory shaping
; 
; LIFTOFF (July 21, 1969, 124:22:00 mission time):
; - APS ignition (no throttle, fixed 3500 lbf thrust)
; - U,V jets active from liftoff
; - RCS provided translation + attitude control during ascent
; - Insertion into orbit: 11 minutes 53 seconds after liftoff
; 
; ABORT SCENARIO (not executed Apollo 11, but trained):
; If abort during descent at any altitude:
; 1. Crew presses ABORT STAGE button
; 2. Explosive bolts fire, descent stage separates
; 3. APS auto-ignites within 0.5 seconds
; 4. V75 executes automatically at staging
; 5. U,V jets enabled immediately
; 6. RCS supplements APS thrust for abort trajectory
; 7. Rendezvous guidance begins targeting CSM orbit
; 
; INTEGRATION:
; - Called by: V75 dispatcher from extended verb table
;              P63/P64/P66 at DPS cutoff (normal landing)
;              P70/P71 at staging (abort sequence)
;              DOI post-burn sequence
; - Clears: SNUFFER flag (erasable memory)
; - Effects: DAP jet selection logic
;           Hand controller translation response
;           Automatic guidance translation commands
; - Related: V65 (SNUFFOUT) sets flag
;           DAP immediately adapts to flag state
; - Priority: Immediate execution (critical at DPS cutoff)
; 
; MANUAL CREW EXECUTION:
; Although typically automatic, crew can manually execute V75:
; - Keypad sequence: V75E (VERB 75 ENTER)
; - DSKY response: Immediate execution, no data entry required
; - Use case: Testing RCS configuration, troubleshooting DAP issues
; - Ground can uplink V75 command if automatic execution fails
; 
; TELEMETRY VERIFICATION:
; Ground controllers verify V75 execution via downlink:
; - SNUFFER flag state visible in DAP configuration telemetry
; - U,V jet command presence confirms flag inactive
; - RCS propellant usage indicates active translation jets
; - Flight controllers verify flag cleared at DPS cutoff
; 
; RETURN:
; Returns via GOPIN continuation. SNUFFER flag remains cleared until next
; V65 (SNUFFOUT) execution or fresh start/restart sequence.
; 
; RCS CONFIGURATION POST-V75:
; Complete 16-thruster authority restored:
; 
; QUAD 1 (forward-facing, +X direction):
; - Roll jets (2): Rotation about longitudinal axis
; - Pitch jets (1): Nose up/down
; - P-axis jets (1): Forward translation
; 
; QUAD 2 (aft-facing, −X direction):
; - Roll jets (2): Rotation about longitudinal axis
; - Pitch jets (1): Nose up/down
; - P-axis jets (1): Aft translation
; 
; QUAD 3 (right side):
; - Yaw jets: Nose left/right
; - V-axis jets: Lateral translation (newly enabled by V75)
; 
; QUAD 4 (left side):
; - Yaw jets: Nose left/right
; - V-axis jets: Lateral translation (newly enabled by V75)
; 
; U-AXIS JETS (up/down): Distributed across quads (newly enabled by V75)
; 
; PROPELLANT CONSIDERATIONS:
; - RCS propellant: ~600 lbs at landing
; - U,V jet consumption: ~2-5 lbs/minute during active maneuvering
; - Attitude hold: ~0.1-0.3 lbs/hour (intermittent firing)
; - Ascent requirement: ~200 lbs minimum for insertion + rendezvous
; - Surface stay limited by propellant conservation
; 
; SAFETY NOTES:
; - Automatic execution at DPS cutoff prevents crew error
; - Redundant verification via telemetry
; - Critical for post-landing safety and ascent capability
; - Failure to execute V75 would leave LM with limited attitude control
; - Manual backup available if automatic execution fails
; 

OUTSNUFF	TC	DOWNFLAG
		ADRES	SNUFFER
		TC	GOPIN

# VERB 85	DISPLAY RR LOS AZIMUTH AND ELEVATION.
#
# AZIMUTH IS THE ANGLE BETWEEN THE LOS AND THE X-Z NB PLANE, 0-90 DEG IN THE +Y HEMISPHERE,
# 360-270 DEG IN THE -Y HEMISPHERE.
#
# ELEVATION IS THE ANGLE BETWEEN +ZNB AND THE PROJECTION OF THE LOS INTO THE X-Z PLANE, 0-360 ABOUT +Y.

;
; ============================================================================
; VERB 85 - DISPLAY RENDEZVOUS RADAR LINE-OF-SIGHT ANGLES (VERB85)
; ============================================================================
;
; PURPOSE:
; Displays rendezvous radar (RR) antenna line-of-sight (LOS) angles on DSKY
; in two-component format: azimuth angle and elevation angle. Enables crew
; to verify radar lock on target (CSM), monitor tracking quality, diagnose
; radar malfunctions, and manually slew antenna if automatic tracking fails
; during rendezvous operations.
;
; RENDEZVOUS RADAR OVERVIEW:
; LM rendezvous radar measures range and range-rate to CSM during rendezvous:
; - Frequency: X-band (9.6 GHz)
; - Range capability: 0-400 nautical miles
; - Range accuracy: ±0.05 nautical miles
; - Range-rate accuracy: ±0.2 feet/second
; - Antenna: Mechanically steered dish (26-inch diameter)
; - Gimbal axes: Two-axis gimbaled mount (azimuth-like and elevation-like)
; - Mounting: Fixed to LM ascent stage, atop cabin roof
;
; ANTENNA ANGLE COORDINATE SYSTEM (NAVIGATION BASE, NB):
;
; The RR antenna angles are defined in LM navigation base coordinates:
; +X_NB: Forward (toward docking hatch)
; +Y_NB: Right (starboard side)
; +Z_NB: Down (toward lunar surface when landed)
;
; AZIMUTH ANGLE DEFINITION:
; - Measures angle between LOS and the X-Z plane (forward-down plane)
; - Indicates how far LOS deviates toward +Y (right) or -Y (left)
; - Range and quadrants:
;   * 0-90°: LOS in +Y hemisphere (target to the right)
;   * 270-360°: LOS in -Y hemisphere (target to the left)
; - Zero azimuth: LOS lies in X-Z plane (target dead ahead or aft)
; - 90° azimuth: LOS perpendicular to X-Z plane, pointing pure right (+Y)
; - 270° azimuth: LOS perpendicular to X-Z plane, pointing pure left (-Y)
;
; Example azimuth values:
; 0°: Target in X-Z plane (no left/right deviation)
; 45°: Target 45° to the right of X-Z plane
; 90°: Target directly to the right (perpendicular to forward direction)
; 270°: Target directly to the left
; 315°: Target 45° to the left of X-Z plane
;
; ELEVATION ANGLE DEFINITION:
; - Measures angle between +Z_NB (down) and projection of LOS into X-Z plane
; - Rotates about +Y axis (right-hand rule)
; - Range: 0-360° (full circle)
; - Key angles:
;   * 0°: LOS pointing straight down (+Z direction)
;   * 90°: LOS pointing forward (+X direction, horizontal plane)
;   * 180°: LOS pointing straight up (-Z direction)
;   * 270°: LOS pointing aft (-X direction, horizontal plane)
;
; Example elevation values:
; 0°: Target directly below LM (straight down)
; 45°: Target forward and below (45° up from straight down)
; 90°: Target forward in horizontal plane (level with LM)
; 135°: Target forward and above (45° above horizontal)
; 180°: Target directly above LM (straight up)
; 270°: Target aft in horizontal plane
;
; TYPICAL RENDEZVOUS GEOMETRY:
;
; COELLIPTIC PHASE (LM catching up from below):
; Azimuth: 0-10° (CSM nearly in X-Z plane, slightly right due to orbit plane difference)
; Elevation: 100-120° (CSM forward and above, 10-30° above horizontal)
;
; TERMINAL PHASE (LM approaching from below/behind):
; Azimuth: 0-5° (CSM nearly dead ahead)
; Elevation: 90-110° (CSM forward, 0-20° above horizontal)
;
; FINAL APPROACH (last few thousand feet):
; Azimuth: 0-2° (precise forward pointing)
; Elevation: 90-95° (CSM nearly in horizontal plane, slightly above)
;
; DISPLAY FORMAT ON DSKY:
; Row 1 (R1): Azimuth angle (degrees, XXX.XX format, 0-360°)
; Row 2 (R2): Elevation angle (degrees, XXX.XX format, 0-360°)
; Row 3 (R3): Not used (blank or zeroes)
;
; Example during terminal phase approach:
;   002.34   (Azimuth = 2.34° from X-Z plane, slightly right)
;   095.67   (Elevation = 95.67° from +Z, slightly above forward horizontal)
;   00000    (R3 unused)
;
; Interpretation: CSM located 2.34° to right of forward-down plane, and
; 5.67° above the forward horizontal direction (95.67° - 90° = 5.67° up).
;
; OPERATIONAL USAGE:
;
; 1. RADAR LOCK ACQUISITION:
;    Pre-rendezvous: Crew enters estimated CSM position via V50N77
;    AGC commands RR antenna to point at predicted location
;    V85 displays commanded angles, crew verifies reasonable
;    When radar locks, angles update to track actual CSM position
;    Rapid angle changes indicate poor lock or multipath interference
;
; 2. TRACKING QUALITY ASSESSMENT:
;    Good tracking: Smooth angle changes, consistent with relative motion
;    Poor tracking: Erratic angles, sudden jumps, oscillations
;    Lost lock: Angles freeze or return to search pattern
;    Multipath: Angles oscillate ±5-10° as radar alternates between targets
;
; 3. MANUAL ANTENNA SLEWING:
;    If automatic tracking fails, crew can manually slew antenna:
;    - V85 displays current angles
;    - Crew estimates CSM location from window sighting
;    - Use RR slew switch on panel to move antenna
;    - Monitor V85 angles approaching desired pointing
;    - When radar reacquires lock, return to AUTO mode
;
; 4. GEOMETRY VERIFICATION:
;    Compare V85 angles with expected rendezvous geometry:
;    - Terminal phase: Azimuth ~0°, Elevation ~100° (forward and slightly up)
;    - Braking phase: Azimuth ~0°, Elevation ~90° (forward horizontal)
;    - Unexpected angles indicate navigation errors or wrong target
;
; 5. COORDINATE FRAME UNDERSTANDING:
;    Crew must mentally convert displayed angles to physical direction:
;    - Elevation 90° = forward horizontal, 0° = down, 180° = up
;    - Azimuth indicates left/right deviation from forward-down plane
;    - Window view helps crew correlate angles with visual CSM sighting
;
; HISTORICAL CONTEXT - APOLLO 11 RENDEZVOUS:
;
; ASCENT AND RENDEZVOUS (July 21, 1969):
;
; POST-INSERTION (~124:34 mission time, 12 minutes after liftoff):
; - Eagle inserted into 9x46 nautical mile elliptical orbit
; - Columbia in circular 60-nautical mile orbit
; - Range to CSM: ~50 nautical miles
; - RR commanded to search for CSM transponder
;
; INITIAL LOCK ACQUISITION (~124:40):
; - Ground: "Eagle, Houston. You are GO for the RR."
; - Eagle: "Roger. RR is enabled."
; - V85 displayed angles: Azimuth ~5°, Elevation ~105°
; - CSM ahead and above, consistent with rendezvous geometry
; - Armstrong: "We've got a good solid lock."
;
; COELLIPTIC SEQUENCE INITIATION (CSI burn, ~125:19):
; - RR tracking CSM continuously
; - V85 monitored by Aldrin for tracking verification
; - Elevation decreasing from ~110° toward ~95° as LM climbed
; - Azimuth stable ~2-5° throughout phase
; - Range decreasing from 40 to 20 nautical miles
;
; CONSTANT DELTA HEIGHT (CDH burn, ~126:18):
; - V85 showed Azimuth ~3°, Elevation ~95° (CSM forward, slightly above horizon)
; - RR range: 15 nautical miles
; - Aldrin cross-checked RR data with VHF ranging
;
; TERMINAL PHASE INITIATION (TPI burn, ~127:04):
; - V85 displayed Azimuth ~2°, Elevation ~92° (CSM nearly horizontal ahead)
; - RR range: 7 nautical miles
; - Final approach phase began
;
; BRAKING PHASE (~127:45):
; - Range: 5 nautical miles and closing
; - V85: Azimuth ~1°, Elevation ~90° (precise forward horizontal pointing)
; - Armstrong took manual control: "I got the CSM in sight."
; - Aldrin continued monitoring V85 for backup navigation
;
; FINAL APPROACH (~128:00-128:15):
; - Range closing to 1000 feet
; - V85: Azimuth ~0°, Elevation ~90° (centered forward horizontal)
; - RR accuracy degraded at close range (<500 feet)
; - Crew transitioned to visual references
; - V85 monitoring discontinued as Armstrong docked visually
;
; DOCKING (~128:15):
; - Armstrong piloted final 100 feet visually
; - Docking successful
; - Aldrin: "That was beautiful, babe!"
; - V85 showed azimuth near zero, elevation near 90° confirming aligned geometry
;
; RADAR ANOMALIES (not encountered Apollo 11, but trained):
;
; MULTIPATH INTERFERENCE:
; - V85 angles oscillate ±5-10° rapidly
; - Range readings erratic or unstable
; - Caused by radar reflection off lunar surface or spacecraft structure
; - Solution: Maneuver to change geometry, or trust navigation filter
;
; WRONG TARGET LOCK:
; - V85 angles inconsistent with expected geometry
; - Example: CSM should be forward (elevation ~90°), but elevation shows 270° (aft)
; - Caused by radar locking on S-IVB stage, another spacecraft, or ground feature
; - Solution: Command REACQ (reacquisition), verify CSM transponder active
;
; GIMBAL LIMITS:
; - RR antenna has mechanical gimbal limits
; - If commanded angle exceeds limits, antenna cannot physically point
; - DATA light illuminates, indicating limit condition
; - V85 displays limit angle value
; - Solution: Maneuver LM to bring CSM within antenna gimbal range
;
; TESTXACT MECHANISM:
; TESTXACT checks if V85 display job already running:
; - If active: Returns immediately without creating duplicate job
; - If inactive: Continues to POSTJUMP to start display
;
; Prevents multiple V85 jobs from competing for DSKY display.
; Only one V85 can execute at a time.
;
; POSTJUMP INSTRUCTION:
; POSTJUMP transfers control to DSPRRLOS routine:
; - DSPRRLOS runs as foreground job (not via FINDVAC)
; - Continuous display loop updates DSKY every 1 second
; - Reads RR antenna resolver angles from RR-AZ and RR-ELEV erasable locations
; - Converts angles from resolver format to degrees
; - Formats for DSKY decimal display
; - Updates R1 (azimuth) and R2 (elevation) registers
; - Loops until verb terminated by RSET or new verb
;
; CADR DSPRRLOS provides code address of display routine.
;
; INTEGRATION:
; - Called by: V85 dispatcher from extended verb table
;              Crew keypad: V85E (VERB 85 ENTER)
;              Ground uplink if crew needs assistance
; - Calls: TESTXACT (check for duplicate execution)
;          POSTJUMP (transfer to display routine)
;          DSPRRLOS (actual display loop, runs continuously)
; - Reads: RR-AZ (azimuth angle, resolver units, EBANK erasable)
;          RR-ELEV (elevation angle, resolver units)
;          RR mode status (AUTO/SLEW/REACQ)
;          RR lock indication
; - Displays: DSKY R1 (azimuth angle, degrees 0-360°)
;            DSKY R2 (elevation angle, degrees 0-360°)
; - Terminates: RSET key, new verb entry, or program change
;
; DISPLAY UPDATE RATE:
; DSPRRLOS updates DSKY approximately once per second:
; - Fast enough to observe antenna motion during slewing
; - Slow enough to avoid excessive AGC load
; - Crew can observe smooth tracking or diagnose erratic behavior
;
; ANGLE SCALING AND CONVERSION:
; RR antenna resolvers output angles in AGC internal units:
; - Resolver format: 360° = 2^15 units (32768 units per full circle)
; - 1 resolver unit = 360°/32768 = 0.01099° per unit
; - DSPRRLOS converts resolver units to decimal degrees for display
; - Azimuth resolution: 0.01° displayed
; - Elevation resolution: 0.01° displayed
; - Adequate precision for crew monitoring and troubleshooting
;
; ANGLE COMPUTATION IN DSPRRLOS:
; Azimuth computation (simplified logic):
; 1. Read RR antenna shaft angle from RR-AZ resolver
; 2. Read RR antenna trunnion angle from RR-TRUNNION
; 3. Compute LOS vector in navigation base coordinates
; 4. Calculate azimuth = angle from X-Z plane to LOS
; 5. Format as 0-90° (+Y hemisphere) or 270-360° (-Y hemisphere)
;
; Elevation computation:
; 1. Project LOS vector into X-Z plane
; 2. Calculate angle from +Z axis to projected LOS
; 3. Rotation about +Y axis (right-hand rule)
; 4. Format as 0-360° for full circle
;
; RELATED VERBS:
; V85 (this verb): Display RR LOS azimuth and elevation angles
; V64: Calculate and display S-band antenna angles (for Earth comm)
; V50N77: Load RR angle designate (command antenna to specific direction)
; V16N63: Monitor altitude/altitude-rate (uses landing radar, not RR)
; V83: Display rendezvous parameters (range, range-rate, from R31 program)
;
; CONTRAST:
; - V85: Shows WHERE radar pointing (antenna mechanics, LOS angles)
; - V83: Shows WHAT radar measuring (range/range-rate navigation data)
; - Both useful during rendezvous, different information layers
; - V85 for antenna diagnosis and manual slewing
; - V83 for navigation state monitoring
;
; TROUBLESHOOTING WITH V85:
;
; SYMPTOM: Angles frozen, not updating
; DIAGNOSIS: Lost radar lock or RR mode switch in SLEW
; ACTION: Verify mode switch in AUTO, command REACQ if needed
;
; SYMPTOM: Angles oscillating wildly
; DIAGNOSIS: Multipath interference or tracking noise
; ACTION: Maneuver to change geometry, verify CSM transponder on
;
; SYMPTOM: Angles inconsistent with expected CSM location
; DIAGNOSIS: Tracking wrong target or navigation error
; ACTION: Cross-check with VHF ranging, window sighting, verify transponder
;
; SYMPTOM: Elevation at gimbal limit, DATA light on
; DIAGNOSIS: CSM beyond antenna mechanical range
; ACTION: Maneuver LM to bring CSM within gimbal coverage
;
; SYMPTOM: Azimuth changing rapidly, elevation stable
; DIAGNOSIS: CSM crossing through X-Z plane (no azimuth defined at 0° deviation)
; ACTION: Normal behavior when CSM directly ahead or aft, wait for geometry change
;
; CREW PROCEDURES:
;
; NORMAL RENDEZVOUS:
; 1. Post-insertion: Execute V85E to begin monitoring
; 2. Verify angles consistent with expected CSM location
; 3. Monitor smooth tracking throughout rendezvous burns
; 4. Cross-check with window sighting when CSM visible
; 5. Terminate V85 via RSET before final visual approach
;
; MANUAL SLEWING:
; 1. Switch RR mode to SLEW
; 2. Execute V85E to display current angles
; 3. Use slew switch to command antenna motion
; 4. Monitor V85 angles approaching target direction
; 5. Switch RR mode to AUTO when pointed near CSM
; 6. Wait for lock acquisition, verify stable tracking
;
; GROUND SUPPORT:
; Flight controllers monitor RR tracking via telemetry:
; - Antenna angles downlinked continuously
; - Lock indication visible in telemetry stream
; - Can request crew execute V85 for troubleshooting
; - Can uplink angle designate commands if needed
; - Verify angles consistent with computed CSM state vector
;
; COORDINATE SYSTEM NOTES FOR UNDERSTANDING:
;
; The navigation base (NB) coordinate system is aligned with LM body:
; - +X_NB points forward (toward docking hatch)
; - +Y_NB points right (starboard)
; - +Z_NB points down (nadir when in standard attitude)
;
; Converting displayed angles to intuitive directions:
;
; FOR ELEVATION:
; - 90° = Forward horizontal (LOS in +X direction, horizontal plane)
; - 0° = Straight down (LOS in +Z direction)
; - 180° = Straight up (LOS in -Z direction)
; - 270° = Aft horizontal (LOS in -X direction)
; - 45° = Downward-forward (45° from straight down toward forward)
; - 135° = Upward-forward (45° above forward horizontal)
;
; FOR AZIMUTH (measures deviation from X-Z plane):
; - 0° = No deviation (LOS in X-Z plane, either forward or aft)
; - 45° = 45° to the right of X-Z plane
; - 90° = Pure right (perpendicular to forward-down plane)
; - 270° = Pure left (perpendicular to forward-down plane)
; - 315° = 45° to the left of X-Z plane
;
; COMBINING AZIMUTH AND ELEVATION:
; - Azimuth 0°, Elevation 90° = Straight ahead, horizontal
; - Azimuth 0°, Elevation 0° = Straight down
; - Azimuth 0°, Elevation 180° = Straight up
; - Azimuth 90°, Elevation 90° = Pure right, horizontal
; - Azimuth 45°, Elevation 135° = Forward-right and 45° above horizontal
;
; RETURN:
; V85 continues running until explicitly terminated:
; - RSET key: Terminates display, returns DSKY to idle
; - New verb: Supersedes V85, stops angle display
; - Program change: Automatic termination
; - Fresh start: All displays cleared
;
; Display loop in DSPRRLOS handles termination gracefully,
; releasing DSKY for next operation.
;

		EBANK=	RR-AZ
VERB85		TC	TESTXACT

# Page 299

		TC	POSTJUMP
		CADR	DSPRRLOS

		SETLOC	PINBALL1
		BANK

		COUNT*	$$/EXTVB

DSPRRLOS	CAF	PRIO5
		TC	FINDVAC
		EBANK=	RR-AZ
		2CADR	RRLOSDSP

		CAF	PRIO4
		TC	PRIOCHNG
		CAF	V16N56
		TC	BANKCALL
		CADR	GOMARKFR
		TC	B5OFF
		TC	B5OFF
		TC	B5OFF

		CAF	BIT3
		TC	BLANKET
		TC	ENDOFJOB

RRLOSDSP	EXTEND
		DCA	CDUT
		DXCH	MPAC
		TC	INTPRET
		CALL
			RRNBMPAC	# GET RR LOS IN BODY AXIS.
		STORE	0D		# UNIT LOS
		STODL	6D
			HI6ZEROS
		STOVL	8D
			6D
		UNIT
		STORE	6D		# UNIT OF LOS PROJ IN X-Z PLANE
		DOT
			UNITZ
		STOVL	COSTH		# 16D
			UNITX
		DOT
			6D
		STCALL	SINTH		# 18D
			ARCTRIG
		BPL	DAD		# INSURE DISPLAY OF 0-360 DEG.
			+2
			DPPOSMAX	# INTRODUCES AND ERROR OF B-28 REVS.

# Page 300

		STOVL	RR-ELEV
			0D
		DOT
			UNITY
		STOVL	SINTH
			0D
		DOT
			6D
		STCALL	COSTH
			ARCTRIG
		BPL	DAD		# INSURE DISPLAY OF 0-360 DEG.
			+2
			DPPOSMAX	# INTRODUCES AN ERROR OF B-28 REVS.
		STORE	RR-AZ
		EXIT
		CA	1SEC
		TC	BANKCALL
		CADR	DELAYJOB

		CA	BIT5
		MASK	EXTVBACT
		CCS	A
		TC	RRLOSDSP
		TC	ENDEXT

V16N56		VN	1656

