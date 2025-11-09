# Copyright:	Public domain.
# Filename:	DAP_INTERFACE_SUBROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1406-1409
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
; FILE: DAP_INTERFACE_SUBROUTINES.agc
; MODULE: Digital Autopilot Interface Utilities
; MISSION PHASE: descent/landing/ascent/rendezvous
;
; TL;DR: Provides common interface subroutines enabling mission control
;        programs to configure and coordinate with the LM Digital Autopilot.
;        Manages attitude deadband settings (tight, normal, wide), zeroes
;        commanded rates and attitude errors, and handles coast mode setup
;        when engines shut down. Critical for crew control authority during
;        landing and ascent phases.
;
; COMMENT-ONLY READERS: These routines allow mission programs to tell the
;        autopilot how tightly to hold the spacecraft's attitude. During
;        critical phases like landing, tighter control is needed.
; CODE-ALONG READERS: Study the deadband configuration mechanism and rate
;        command zeroing logic. Note the NOVAC job setup for repositioning
;        TJETLAW switch curves when deadband changes.
; ============================================================================

# Page 1406
		BANK	20
		SETLOC	DAPS3
		BANK

		EBANK=	CDUXD
		COUNT*	$$/DAPIF

; ============================================================================
; TRANSITION: DAP Interface Utilities Overview
;
; The Lunar Module's autopilot must adapt to different mission phases.
; During powered descent, the crew needs precise attitude control for landing
; site visibility. During coast phases, looser control conserves RCS fuel.
; These subroutines provide the interface allowing mission programs (P63
; landing, P12 ascent, rendezvous programs) to reconfigure autopilot behavior
; without directly manipulating DAP internals.
; ============================================================================

# MOD 0		DATE	11/15/66	BY GEORGE W. CHERRY
# MOD 1			 1/23/67	MODIFICATION BY PETER ADLER
#
# FUNCTIONAL DESCRIPTION
#	HEREIN ARE A COLLECTION OF SUBROUTINES WHICH ALLOW MISSION CONTROL PROGRAMS TO CONTROL THE MODE
#	AND INTERFACE WITH THE DAP.
#
# CALLING SEQUENCES
#	IN INTERRUPT OR WITH INTERRUPT INHIBITED
#		TC	IBNKCALL
#		FCADR	ROUTINE
#	IN A JOB WITHOUT INTERRUPT INHIBITED
#		INHINT
#		TC	IBNKCALL
#		FCADR	ROUTINE
#		RELINT
#
# OUTPUT
#	SEE INDIVIDUAL ROUTINES BELOW
#
# DEBRIS
#	A, L, AND SOMETIMES MDUETEMP			ODE	NOT IN PULSES MODE

# Page 1407
# SUBROUTINE NAMES:
#	SETMAXDB, SETMINDB, RESTORDB, PFLITEDB
# MODIFIED:	30 JANUARY 1968 BY P S WEISSMAN TO CREATE RESTORDB.
# MODIFIED:	1 MARCH 1968 BY P S WEISSMAN TO SAVE EBANK AND CREATE PFLITEDB
#
# FUNCTIONAL DESCRIPTION:
#	SETMAXDB -- SET DEADBAND TO 5.0 DEGREES
#	SETMINDB -- SET DEADBAND TO 0.3 DEGREE
#	RESTORDB -- SET DEADBAND TO MAX OR MIN ACCORDING TO SETTINGS OF DBSELECT BIT OF DAPBOOLS
#	PFLITEDB -- SET DEADBAND TO 1.0 DEGREE AND ZERO THE COMMANDED ATTITUDE CHANGE AND COMMANDED RATE
#
#	ALL ENTRIES SET UP A NOVAC JOB TO DO 1/ACCS SO THAT THE TJETLAW SWITCH CURVES ARE POSITIONED TO
#	REFLECT THE NEW DEADBAND.  IT SHOULD BE NOTED THAT THE DEADBAND REFERS TO THE ATTITUDE IN THE P-, U-, AND V-AXES.
#
# SUBROUTINE CALLED:	NOVAC
#
# CALLING SEQUENCE:	SAME AS ABOVE
#			OR	TC RESTORDB +1    FROM ALLCOAST
#
# DEBRIS:		A, L, Q, RUPTREG1, (ITEMPS IN NOVAC)

; ============================================================================
; DEADBAND CONFIGURATION SUBROUTINES
;
; The autopilot maintains the LM's attitude within a "deadband" - an allowed
; angular deviation from the commanded attitude. Tighter deadbands (smaller
; angles) provide precise pointing but consume more RCS fuel through frequent
; thruster firings. Wider deadbands reduce fuel usage during long coast phases.
;
; Three standard deadband settings:
; - NARROW (0.3 deg): Maximum precision for landing visibility and docking
; - POWER (1.0 deg): Balanced control during powered flight phases
; - WIDE (5.0 deg): Fuel-efficient coasting during orbital operations
;
; After changing deadband, the TJETLAW thruster firing logic must recalculate
; its switch curves. This is handled asynchronously via a NOVAC job to avoid
; blocking the calling program.
; ============================================================================

; RESTORDB - Restore crew-selected deadband setting
; This subroutine reads the DBSELECT bit from DAPBOOLS to determine whether
; the crew has selected narrow or wide deadband via the DSKY. During nominal
; operations, astronauts could toggle between precision and fuel-saving modes.

RESTORDB	CAE	DAPBOOLS	# DETERMINE CREW-SELECTED DEADBAND.
		MASK	DBSELECT	# Extract deadband selection bit.
		EXTEND			# Check if bit is zero (narrow) or one (wide).
		BZF	SETMINDB	# Branch to narrow deadband if bit cleared.

; SETMAXDB - Configure wide deadband for fuel-efficient coasting
; Sets attitude deadband to 5.0 degrees. Used during long coast phases between
; major maneuvers when precise pointing is not required. Reduces RCS thruster
; activity, conserving propellant for critical mission phases.

SETMAXDB	CAF	WIDEDB		# SET 5 DEGREE DEADBAND.
 +1		TS	DB		# Store new deadband value in DB variable.

; After deadband change, recalculate thruster firing switch curves.
; The 1/ACCS job repositions TJETLAW decision boundaries to match new deadband.
; This ensures thruster on/off transitions occur at appropriate attitude errors.

		EXTEND			# SET UP JOB TO RE-POSITION SWITCH CURVES.
		QXCH	RUPTREG1	# Save return address in RUPTREG1.
CALLACCS	CAF	PRIO27		# Priority 27 job for 1/ACCS calculation.
		TC	NOVAC		# Schedule asynchronous job via NOVAC.
		EBANK=	AOSQ		# Set EBANK for 1/ACCJOB data access.
		2CADR	1/ACCJOB	# Address of acceleration calculation routine.

		TC	RUPTREG1	# RETURN TO CALLER.

; SETMINDB - Configure narrow deadband for precision attitude control
; Sets attitude deadband to 0.3 degrees. Used during critical phases requiring
; precise pointing: final landing approach, rendezvous docking, IMU alignment.
; Armstrong used narrow deadband during manual landing site selection.

SETMINDB	CAF	NARROWDB	# SET 0.3 DEGREE DEADBAND.
		TCF	SETMAXDB +1	# Jump to common deadband storage logic.

; PFLITEDB - Configure powered flight deadband
; Sets deadband to 1.0 degree for powered flight phases (descent, ascent).
; This intermediate setting balances control precision with RCS fuel efficiency.
; Also zeros commanded attitude changes and rates to start from clean state.
; Called at ignition of DPS (descent) or APS (ascent) engines.

PFLITEDB	EXTEND			# THE RETURN FROM CALLACCS IS TO RUPTREG1.
		QXCH	RUPTREG1	# Save return address for later.
		TC	ZATTEROR	# ZERO THE ERRORS AND COMMANDED RATES.
		CAF	POWERDB		# SET DB TO 1.0 DEG.
		TS	DB		# Store powered flight deadband value.
		TCF	CALLACCS	# SET UP 1/ACCS AND RETURN TO CALLER.

; Deadband constant definitions (scaled at 45 degrees = full scale)
; AGC uses scaled fixed-point arithmetic where 45 degrees represents maximum
; scale value. All angles stored as fractions of 45 degrees.

NARROWDB	OCTAL	00155		# 0.3 DEGREE SCALED AT 45.
					# (0.3/45 = 0.00667 = octal 00155)
# Page 1408
WIDEDB		OCTAL	03434		# 5.0 DEGREES SCALED AT 45.
					# (5.0/45 = 0.11111 = octal 03434)
POWERDB		DEC	.02222		# 1.0 DEGREE SCALED AT 45.
					# (1.0/45 = 0.02222 decimal)

; ============================================================================
; TRANSITION: From Deadband Configuration to Attitude Error Management
;
; Having configured the deadband (allowable attitude deviation), the DAP
; now needs utilities to zero accumulated errors when modes change or
; engines ignite. ZATTEROR and STOPRATE clear attitude errors and rate
; commands, ensuring the autopilot starts from a clean state without
; unwanted control responses from previous flight phases.
; ============================================================================

; ZATTEROR - Zero Attitude Errors and Commanded Rates
; Critical utility called during mode transitions, engine ignition, fresh starts.
; Clears all commanded rates and attitude errors to prevent control transients.
; Preserves current IMU gimbal angles (CDUX/Y/Z) by saving to desired values
; (CDUXD/YD/ZD), establishing current attitude as new reference target.
; Note: Does NOT zero AK (Kalman filter scaled error) to avoid corrupting
; asynchronous filter computations running in separate jobs.

ZATTEROR	CAF	EBANK6		# Select EBANK 6 for DAP variables.
		XCH	EBANK		# Switch to EBANK 6 for erasable access.
		TS	L		# SAVE CALLERS EBANK IN L.
					# (Preserves caller's bank for restoration)
		CAE	CDUX		# Load current X-axis gimbal angle.
		TS	CDUXD		# Store as desired X angle reference.
		CAE	CDUY		# Load current Y-axis gimbal angle.
		TS	CDUYD		# Store as desired Y angle reference.
		CAE	CDUZ		# Load current Z-axis gimbal angle.
		TS	CDUZD		# Store as desired Z angle reference.
					# Current attitude becomes new target, zeroing
					# attitude errors implicitly.
		TCF	STOPRATE +3	# Jump to STOPRATE to zero rate commands.

; STOPRATE - Zero Commanded Rates and Accumulated Attitude Errors
; Clears all rate commands (OMEGAPD/QD/RD) and attitude change accumulators
; (DELCDUX/Y/Z, DELPEROR/QEROR/REROR) to stop all commanded spacecraft motion.
; Two entry points: STOPRATE (saves EBANK), +3 (assumes EBANK6 already set).
; Called during mode transitions, engine shutdowns, and by ZATTEROR.

STOPRATE	CAF	EBANK6		# Select EBANK 6 for DAP rate variables.
		XCH	EBANK		# Switch to EBANK 6.
		TS	L		# SAVE CALLERS EBANK IN L.
					# (Normal entry point preserves caller's bank)
 +3		CAF	ZERO		# Load zero to clear variables.
					# (+3 entry assumes EBANK6 already set)
		TS	OMEGAPD		# Zero P-axis commanded rate.
		TS	OMEGAQD		# Zero Q-axis commanded rate.
		TS	OMEGARD		# Zero R-axis commanded rate.
		TS	DELCDUX		# Zero X-axis attitude change accumulator.
		TS	DELCDUY		# Zero Y-axis attitude change accumulator.
		TS	DELCDUZ		# Zero Z-axis attitude change accumulator.
		TS	DELPEROR	# Zero P-axis error accumulator.
		TS	DELQEROR	# Zero Q-axis error accumulator.
		TS	DELREROR	# Zero R-axis error accumulator.
					# All commanded motion cleared, autopilot
					# will maintain current attitude passively.
		LXCH	EBANK		# RESTORE CALLERS EBANK.
		TC	Q		# Return to caller.

; ============================================================================
; TRANSITION: From Individual DAP Commands to Complete Mode Transitions
;
; The preceding subroutines handle specific DAP configuration tasks:
; adjusting deadbands, zeroing errors, stopping rates. ALLCOAST combines
; these operations into a comprehensive engine-off configuration routine.
; When main engines shut down (DPS after landing, APS after rendezvous,
; or during fresh starts), the spacecraft enters a passive coasting state.
; ALLCOAST prepares the autopilot for this mode by clearing all control
; state, resetting deadbands, and enabling drift compensation.
; ============================================================================

# SUBROUTINE NAME:	ALLCOAST
# WILL BE CALLED BY FRESH STARTS AND ENGINE OFF ROUTINES.
#
# CALLING SEQUENCE:	(SAME AS ABOVE)
#
# EXIT:			RETURN TO Q.
#
# SUBROUTINES CALLED:	STOPRATE, RESTORDB, NOVAC
#
# ZERO:			(FOR ALL AXES) AOS, ALPHA, AOSTERM, OMEGAD, DELCDU, DELEROR
#
# OUTPUT:		DRIFTBIT/DAPBOOLS, OE, JOB TO DO 1/ACCS
#
# DEBRIS:		A, L, Q, RUPTREG1, RUPTREG2, (ITEMPS IN NOVAC)

; ALLCOAST - Configure Autopilot for Engine-Off Coasting
; Called after engine shutdown (landing, rendezvous completion, aborts) and
; during fresh starts. Performs complete DAP state reset: clears all commanded
; rates and errors, zeros attitude offset (AOS) accumulators and filter terms,
; sets DRIFTBIT to enable IMU drift compensation, restores crew-selected
; deadband. This ensures clean autopilot behavior when transitioning from
; powered flight to passive attitude hold during orbital coast phases.

ALLCOAST	EXTEND			# SAVE Q FOR RETURN
		QXCH	RUPTREG2	# Store return address in RUPTREG2.
# Page 1409
		TC	STOPRATE	# CLEAR RATE INTERFACE.  RETURN WITH A=0
					# Zeros all commanded rates and error terms.
		LXCH	EBANK		#   AND L=EBANK6.  SAVE CALLER'S EBANK.
					# STOPRATE returns with A=0 for zeroing below.
		TS	AOSQ		# Zero Q-axis attitude offset state (low).
		TS	AOSQ +1		# Zero Q-axis attitude offset state (high).
		TS	AOSR		# Zero R-axis attitude offset state (low).
		TS	AOSR +1		# Zero R-axis attitude offset state (high).
		TS	ALPHAQ		# FOR DOWNLIST.
					# Zero Q-axis alpha filter term (telemetry).
		TS	ALPHAR		# Zero R-axis alpha filter term (telemetry).
		TS	AOSQTERM	# Zero Q-axis AOS filter integrator.
		TS	AOSRTERM	# Zero R-axis AOS filter integrator.
					# All attitude offset accumulators cleared,
					# ensuring no residual drift compensation
					# from previous powered flight phase.
		LXCH	EBANK		# RESTORE EBANK (EBANK6 NO LONGER NEEDED)

		CS	DAPBOOLS	# SET UP DRIFTBIT
					# Load complement of DAPBOOLS register.
		MASK	DRIFTBIT	# Isolate DRIFTBIT position.
		ADS	DAPBOOLS	# Set DRIFTBIT in DAPBOOLS (enable drift comp).
					# Drift compensation needed during long coast
					# to counteract IMU gyro drift errors.
		TC	RESTORDB +1	# RESTORE DEADBANK TO CREW-SELECTED VALUE.
					# Reconfigure deadband per crew preference
					# and recalculate TJETLAW switch curves.

		TC	RUPTREG2	# RETURN.

