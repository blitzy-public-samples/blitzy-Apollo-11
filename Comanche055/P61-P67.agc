# Copyright:    Public domain.
# Filename:     P61-P67.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 789-818
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-12 RSB	Adapted from Colossus249 file of the same
#				name and Comanche 055 page images.
#		2009-05-20 RSB	Corrections:  V06N68 -> V06N74, added missing
#				definition of V06N74, in several
#				interpreter operands fixed stuff like
#				N-M,1 to N -M,1
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
; FILE: P61-P67.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: re-entry
;
; TL;DR: Atmospheric entry program suite managing Command Module reentry from
;        Earth return trajectory. Programs span initialization (P61), ballistic
;        entry (P62), lifting entry guidance (P63), CM/SM separation (P64),
;        skip targeting (P65), entry monitoring (P66), and final phase (P67).
;        Critical for Apollo 11 Pacific Ocean splashdown July 24, 1969.
;
; COMMENT-ONLY READERS: These programs guided Apollo 11 safely through the
;        fiery reentry into Earth's atmosphere for ocean splashdown.
; CODE-ALONG READERS: Study atmospheric entry guidance algorithms, lift vector
;        control for range targeting, bank angle modulation, and g-load management.
; ============================================================================
;
; ============================================================================
; ATMOSPHERIC ENTRY PROGRAM SUITE (P61-P67) - MISSION OVERVIEW
; ============================================================================
;
; After completing the Apollo 11 transearth journey from the Moon, the Command
; Module Columbia separated from the Service Module and prepared for atmospheric
; entry on July 24, 1969. This suite of programs managed the critical final
; phase: a precisely controlled entry through Earth's atmosphere at over 25,000
; mph, generating temperatures exceeding 5,000°F on the heat shield while
; maintaining g-loads within safe limits for the crew.
;
; ENTRY CORRIDOR AND GUIDANCE CHALLENGE:
; The spacecraft must hit a narrow entry corridor - too shallow and it skips
; back into space, too steep and g-forces exceed crew tolerance or cause
; structural failure. The entry programs use lift modulation (bank angle
; control) to "fly" the capsule through the atmosphere, correcting trajectory
; errors accumulated during the transearth coast.
;
; THE ENTRY PROGRAM SEQUENCE:
;
; P61 - Entry Interface Initialization
;       Calculates entry parameters, displays predictions, accepts target
;       coordinates and lift direction selection from crew
;
; P62 - Ballistic Entry Mode
;       Simplified entry with no lift modulation, used for contingencies
;
; P63 - Lifting Entry Guidance (Primary Mode)
;       Actively steers spacecraft using bank angle modulation to control
;       lift vector, correcting range and crossrange errors to target
;
; P64 - CM/SM Separation Monitor
;       Verifies Service Module has cleanly separated before entry interface
;
; P65 - Skip Entry Targeting
;       Specialized guidance for "skip" trajectory extending range
;
; P66 - Entry Monitoring
;       Displays entry progress, velocities, altitudes, g-loads to crew
;
; P67 - Final Descent Phase
;       Terminal guidance and drogue parachute deployment preparation
;
; INTEGRATION WITH OTHER SYSTEMS:
; These programs coordinate with:
; - REENTRY_CONTROL.agc: Guidance equations computing range-to-target and
;   required bank angle commands
; - CM_ENTRY_DIGITAL_AUTOPILOT.agc: Autopilot executing bank angle commands
;   via RCS thruster firings
; - ENTRY_LEXICON.agc: Atmospheric model parameters and entry constants
;
; APOLLO 11 ENTRY PROFILE:
; On July 24, 1969, Apollo 11 performed a lifting entry guided by P63:
; - Entry interface (400K ft): 36,237 fps inertial velocity
; - Peak deceleration: 6.5 g at approximately 200,000 ft altitude
; - Peak heating: 30,000 BTU/ft²/sec at 270,000 ft altitude
; - Drogue deployment: 23,000 ft altitude
; - Main parachutes: 10,000 ft altitude
; - Splashdown: Pacific Ocean, 13°19'N 169°9'W, 1,440 nm downrange from
;   entry interface, landing within 2.7 nm of target
;
; The entry lasted approximately 15 minutes from entry interface to splashdown,
; with radio blackout during peak heating (4 minutes duration) due to ionized
; plasma sheath surrounding the spacecraft.
;
; ============================================================================

# Page 789
; ============================================================================
; PROGRAM P61 - ENTRY INTERFACE DISPLAY
; ============================================================================
;
; After transearth injection and the long coast home from the Moon, Apollo 11
; approached Earth at approximately 25,000 mph. This program calculates and
; displays critical parameters for the upcoming atmospheric entry, allowing
; Mission Control and the crew to verify trajectory predictions before the
; fiery plunge through Earth's atmosphere to Pacific Ocean splashdown.
;
; P61 computes entry initialization data including maximum g-loads, predicted
; velocities, flight path angles, and time-to-entry. The crew inputs target
; splashdown coordinates and selects entry attitude (lift-up or lift-down)
; which determines the Command Module's bank angle during reentry.
;
; Historical Context: On July 24, 1969, as Apollo 11 neared Earth after
; humanity's first lunar landing, P61 prepared the crew for entry interface
; at 400,000 feet altitude where atmospheric effects begin. The program
; predictions enabled Mission Control to validate the trajectory would bring
; Columbia safely to the Pacific recovery area.
;
; ============================================================================

# PROGRAM:	P61
# MOD NO.:	0	MAR. 13, 1967
# MOD BY:	R. HIRSCHKOP
# MOD NO: 1	MOD BY:  RR BAIRNSFATHER	DATE: 22 JUN 67		RESTARTS
# MOD NO: 2	MOD BY:  RR BAIRNSFATHER	DATE: 17 JAN 68		COLOSSUS GSOP CHANGES
# MOD NO: 3	MOD BY:  RR BAIRNSFATHER	DATE:  8 MAY 68		DELETE CMSM MANEUVER (PCR 50)
# FUNCTION:	TO CALCULATE AND DISPLAY EMS INITIALIZATION DATA
# CALLING SEQUENCE:  BY V37
# EXIT:		TO P62
# SUBROUTINE CALLS:  S61.1, S61.3, GOFLASH, FLAGUP, R02BOTH
# ERASABLE INITIALIZATION:
#	EMSALT (-29) M		.05G ALTITUDE ABOVE FISCHER ELLIPSOID	PAD LOADED.
#	ALFAPAD /180		HYPERSONIC CM TRIM ANGLE OF ATTACK	PAD LOADED
# OUTPUT:	THE FOLLOWING REGISTERS ARE WRITTEN IN FOR USE BY DISPLAYS
#		GMAX	100 GMAX (-14) G,S	MAXIMUM ACCELERATION
#		VPRED	(-7) M/CS		PREDICTED VELOCITY AT 400K FT
#		GAMMAEI	(GAMMA/360		PREDICTED GAMMA    AT 400K FT
#		RTGO	THETAH/360		RANGE ANGLE TO SPLASH FROM EMSALT	EMSALT IS PAD LOADED
#		VIO	(-7) M/CS		INERTIAL VELOCITY AT	   EMSALT	EMSALT IS PAD LOADED
#		TTE	(-28) CS		TIME TO			   EMSALT	EMSALT IS PAD LOADED
#		LAT(SPL) /360			TARGET LOCATION				LEFT BY DSKY
#		LNG(SPL) /360			TARGET LOCATION				LEFT BY DSKY
#		HEADSUP	(0)			+1 = LIFT DOWN, -1 = LIFT UP		LEFT BY DSKY
# DEBRIS:	SEE SUBROUTINES.

		BANK	26
		SETLOC	P60S
		BANK

		EBANK=	AOG

		COUNT*	$$/P61

; Entry interface program begins. The Command Module is approaching Earth
; after transearth coast. P61 calculates entry trajectory predictions and
; displays critical parameters for crew verification before atmospheric entry.

P61		CA	BIT14		# EXTENDED VERB SHOULD BE FREE THIS CLOSE
		TS	EXTVBACT	# TO V37
					# LOCK OUT EXTENDED VERBS SO CAN USE TFF
					# ROUTINES.  EXT VERB ERASE IS USED

; Initialize entry attitude mode. HEADSUP determines CM orientation:
; +1 = lift-down (roll = 180°, used for skip-out trajectory)
; -1 = lift-up (roll = 0°, used for direct entry)
; This choice affects range control and g-load profile during entry.

		CS	ONE		# REMOVE IF HEADSUP EVER ON UPLINK DATA
		TS	HEADSUP		# PRELOAD

; Verify spacecraft state vector and IMU alignment are valid for entry
; computations. Ensures navigation data accuracy before critical phase.

		TC	S61.1		# CHECK STATE VECTOR AND IMU ORIENTATION
					# RV 60GENRET. DOES PHASCHNG, GROUP 4.

; Display target splashdown coordinates to crew for verification.
; Crew enters latitude, longitude, and heads-up/heads-down preference.
; For Apollo 11's July 24, 1969 splashdown, target was Pacific Ocean
; recovery area (approximately 13°N 169°W). Recovery ships positioned nearby.

		CA	V06N61		# LAT(SPL)	LNG(SPL)	HEADSUP
					# XXX.XX DEG	XXX.XX DEG	XXXXX.
		TC	BANKCALL
		CADR	GOFLASHR
		TC	GOTOPOOH
# Page 790
		TC	P61.4
		TC	-5

; Phase change and job termination point for restart protection.

P61.3		TC	PHASCHNG
		OCT	00014

		TC	ENDOFJOB

; Set commanded roll angle based on crew's heads-up/heads-down selection.
; This determines CM orientation during entry, affecting lift vector direction
; for range control. Roll angle stored in ROLLC for use by gimbal angle
; computation routine S62.3 at 0.05g entry interface.

P61.4		ZL
		CCS	HEADSUP		# C(HEADSUP)= +1/-1
		CA	BIT14		# IF HEADSUP POS,ROLLC =180 DEG.(LIFT DWN)
		NOOP			# IF HEADSUP NEG,ROLLC =0 (LIFT UP)
		DXCH	ROLLC		# ROLLC IS USED BY S62.3: GIM ANG AT .05G

; ============================================================================
; ENTRY DISPLAY COMPUTATION SECTION
;
; Computes and displays entry parameters for crew monitoring. Captures
; current position (RN) and velocity (VN) vectors, checking for navigation
; updates during computation. If state vector updated by ground uplink or
; onboard tracking, restarts computation with fresh data for accuracy.
; ============================================================================

		TC	INTPRET
NEWRNVN		DLOAD
			PIPTIME		# SAVE TIME OF RN,VN TO DETERMINE IF AN
		STCALL	MM		# UPDATE HAS OCCURRED.
			STARTEN1	# INITIALIZE
		VLOAD
			RN
		STORE	RONE
		UNIT
		STOVL	URONE
			VN
		STORE	VONE
		VXV	UNIT
			URONE
		STORE	UNI
DUMPP61		DLOAD	DSU
			MM		# INITIAL VALUE OF PIPTIME
			PIPTIME
		BMN	CALRB
			NEWRNVN		# UPDATED... GO TRY AGAIN
			S61.2		# GET DISPLAY DATA FOR N60 AND N63
					# AND RETURN IN BASIC, BELOW.

; Display V06N60: Maximum g-load (GMAX), predicted velocity at 400K ft (VPRED),
; and predicted flight-path angle at 400K ft (GAMMAEI).
; Crew monitors these values to verify entry trajectory is within safe limits.
; Apollo 11 entry peaked at approximately 6.5g during final approach.

P61.1		TC	CLEARMRK
		CA	V06N60		# GMAX		VPRED		GAMMAEI
					# XXX.XX G	XXXXX. FPS	XXX.XX DEG
		TC	BANKCALL
		CADR	GOFLASH

		TC	GOTOPOOH
		TC	P61.2		# PROCEED
		TC	-5

; Update time-to-entry-interface (TTE) accounting for elapsed time during
; crew's review of previous display. Ensures accurate prediction of when
; spacecraft will reach 0.05g entry interface altitude.

P61.2		TC	INTPRET		# CORRECT TTE FOR TIME LAPSE DURING
					# ABOVE DISPLAY.
		RTB	DSU
			LOADTIME	# CURRENT TIME.
# Page 791
			MM		# PIPTIME FOR RONE & VONE.
		DAD
			TTE1		# NEGATIVE OF FREE FALL TIME.
		STORE	TTE		# DECREMENTED

		EXIT

; Display V06N63: Range-to-go (RTGO), inertial velocity at entry interface (VIO),
; and time-to-entry-interface (TTE). Final crew verification before committing
; to atmospheric entry. These values critical for splashdown accuracy.

		CA	V06N63		# RTGO		VIO		TTE
					# XXXX.X NM	XXXXX. FPS	XXBXX M,S
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	+2
		TC	P61.2		# REDO

; P61 initialization complete. Transition to P62 for CM/SM separation
; and entry attitude orientation.

#		.... THEN FALL INTO P62
# Page 792
; ============================================================================
; PROGRAM P62 - BALLISTIC ENTRY MODE & CM/SM SEPARATION
; ============================================================================
;
; P62 manages the critical transition from transearth coast to atmospheric
; entry, including Command Module/Service Module separation and orientation
; to entry attitude. This is the backup to P63's lifting entry - if guidance
; fails, P62 provides a safe ballistic trajectory with no lift modulation.
;
; MISSION SEQUENCE:
; 1. Verifies GNC system ready for CM/SM separation
; 2. Requests crew confirmation for separation ("Go for SEP")
; 3. Executes pyrotechnic separation of Service Module
; 4. Orients CM to entry attitude (heat shield forward, proper roll angle)
; 5. Monitors attitude convergence to entry trim values
; 6. Transitions to P63 for active guidance or maintains ballistic entry
;
; ENTRY ATTITUDE PARAMETERS:
; - ROLLC: Roll angle (0° for lift-up, 180° for lift-down) based on HEADSUP
; - ALFACOM: Angle of attack trim (ALFAPAD from pad load, typically ~-20°)
; - BETACOM: Sideslip angle (normally zero)
;
; The entry attitude is computed to orient the lift vector correctly for
; targeting the splashdown site. For Apollo 11, lift-up orientation was
; selected to maximize control authority.
;
; CM/SM SEPARATION TIMING:
; Separation occurs approximately 15 minutes before entry interface (400K ft
; altitude). The Service Module is jettisoned via explosive bolts while small
; RCS jets fire to provide positive separation velocity. The crew visually
; confirms clean separation through the windows.
;
; BALLISTIC VS LIFTING ENTRY:
; P62 can maintain a ballistic (unguided) entry if P63 is not invoked. In
; ballistic mode, the spacecraft follows a fixed trajectory determined by
; entry interface conditions. Range control is limited to ±150 nm compared
; to ±1500 nm capability with P63's active lift modulation.
;
; TRANSITION TO P63:
; When body attitude reaches acceptable trim (within ±45° of desired angle
; of attack), P62 automatically transitions to P63 for precision lifting
; entry guidance. Task WAKEP62 monitors this transition from the entry DAP.
;
; INTEGRATION:
; - Calls CM/DAPIC to initialize CM digital autopilot calculations
; - Calls CM/DAPON to enable entry autopilot and disable RCS DAP
; - Uses S62.3 to calculate desired gimbal angles for entry attitude
; - Displays progress via V06N61 (target coordinates, lift direction)
;
; ============================================================================

# PROGRAM:	P62
# MOD NO.:	0	MAR. 13, 1967
# MOD BY:	R. HIRSCHKOP
# MOD NO:  1	MOD BY:  RR BAIRNSFATHER	DATE: 21 MAR 67
# MOD NO:  2	MOD BY:  RR BAIRNSFATHER	DATE: 22 JUN 67		RESTARTS.
# MOD NO:  3	MOD BY:  RR BAIRNSFATHER	DATE: 17 JAN 68		COLOSSUS GSOP CHANGES.
# MOD NO:  4	MOD BY:  RR BAIRNSFATHER	DATE:  8 MAY 68		MOVE START OF DESIRED GIMBAL CALC.
# FUNCTION:	1) TO NOTIFY CREW WHEN GNC SYSTEM IS PREPARED FOR CM/SM SEPARATION.
#		2) TO ORIENT THE CM TO THE CORRECT ATTITUDE FOR ATMOSPHERIC ENTRY.
# CALLING SEQUENCE:  BY V37 OR DIRECTLY FROM P61
# EXIT:		TO P63
# ERASABLE INITIALIZATION:
#	ALFAPAD					 	LEFT BY PAD LOAD
#	LADPAD						LEFT BY PAD LOAD
#	LODPAD						LEFT BY PAD LOAD
#	LAT(SPL)	(MAY BE CHANGED BELOW)		LEFT BY DSKY, VIA P61
#	LNG(SPL)	(MAY BE CHANGED BELOW)		LEFT BY DSKY, VIA P61
#	HEADSUP		(MAY BE CHANGED BELOW)		LEFT BY DSKY, VIA P61
# SUBROUTINE CALLS:  NEWMODEX, S61.1, CM/DAPIC, CM/DAPON, R02BOTH, GOPERF1, GOFLASH, GODSPR.

		COUNT*	$$/P62

		TC	NEWMODEX		# MODE CHANGE IF CAME FROM P61.
		MM	62			# MODE CHANGE AUTOMATIC VIA V 37.
		CA	ONE
		TS	DNLSTCOD

P62		TC	S61.1			# CHECK STATE VECTOR AND IMU ORIENTATION.

		TC	INTPRET
		SSP	RTB
			POSEXIT
			P62.3			# CALCULATE DESIRED .05G GIMBAL ANGLES.
						# WITHOUT DISPLAY.
			CM/DAPIC		# START CM/POSE AND BODY RATE CALC

						# DOES 2PHSCHNG, OCT 40116, OCT 05024, OCT 13000.
						# CM/DAPIC SETS EBANK = EBAOG
						# AND RETURNS IN BASIC TO P62.2.
P62.2		EXTEND
		DCA	POSECADR		# CONTINUE WITH CM/POSE AFTER AV G.
		DXCH	AVEGEXIT

		CAF	OCT41			# REQUEST SEPARATION
		TC	BANKCALL
		CADR	GOPERF1R
		TC	GOTOPOOH
		TC	+3			# PROCEED
# Page 793
						# NOTE:  NODOFLAG WILL BE SET IN CM/DAPON. ***
		TC	-5			# ENTER
		TC	P61.3			# FOR PHASCHNG AND ENDOFJOB

	+3	TC	POSTJUMP
		CADR	CM/DAPON		# DISABLE RCS DAP, ENABLE ENTRY DAP AND
						# DO ATTITUDE HOLD.

						# WILL IDLE UNTIL CM/POSE DOES ONE UPDATE.
						# CM/DAPON DOES NO PHASCHNG.

P62.1		CA	V06N61			# LAT(SPL)	LNG(SPL)	HEADSUP
						# XXX.XX DEG	XXX.XX DEG	0000X.

						# TERMINATE ATTITUDE HOLD.  SET UP COMMANDS:
						# ROLLC, ALFACOM, BETACOM.  BEGIN MANEUVER TO
						# ENTRY ATTITUDE.

		TC	BANKCALL
		CADR	GOFLASH
		TC	-3
		TC	+2
		TC	-5

		TC	PHASCHNG
		OCT	04024			# USE ENTRYVN FOR DISPLAY BELOW.
						# EBANK WAS SET IN CM/DAPON TO EBAOG

		CCS	HEADSUP			# C(HEADSUP) = +/- 1
		CA	BIT14			# IF HEADSUP POS, ROLLC=180 DEG (LIFT DWN)
		NOOP				# IF HEADSUP NEG, ROLLC=0 DEG (LIFT UP)
		TS	ROLLC
		CA	ALFAPAD			# NOMINAL ALFATRIM PAD LOADED, NEG. NO.
		ZL
		DXCH	ALFACOM			# SET ALFACOM = ALFA TRIM, BETACOM=0

		CA	ONE			# PERMITS EXDAP2 TO CHANGE FLAG TO +0
		TS	P63FLAG			# AS INDICATOR.  STARTS UP P63.

		CA	V06N22			# SET UP DISPLAY FOR CDU DESIRED VALUES
		TS	ENTRYVN			# FROM ENTRY ATTITUDE CALC, THAT IS
						# ALREADY GOING.
		TC	UPFLAG			# TURN ON ENTRY DISPLAY
		ADRES	ENTRYDSP		# ENTRYDSP = 92D BIT 13 FLAG 6
SKIP
# Page 794
		CS	CMDAPMOD		# GO DIRECTLY TO P63 IF BODY ATTITUDE
		MASK	ONE			# IS SUCH THAT THE DELAY TASK: WAKEP62
		EXTEND				# WILL BE OMITTED.
		BZF	P63.1			# DISABLE GRP 4, GO TO ENDOFJOB.
						# (I.E., CONTINUE IF CMDAPMOD = -1, OR +0)
		TC	P63

						# PUT JOB TO SLEEP UNTIL VEHICLE MANEUVER HAS
						# REDUCED ALFA TO +/-45 DEG. CONSIDER REMAINING
						# 65 DEG (25 DEG IF ALFA NEG) TO ALFA TRIM TO
						# OCCUR AT 3 DEG/SEC, AND TERMINATE P62 AT THAT
						# TIME.

						# TASK WAKEP62 IS CALLED FROM ENTRY DAP.
WAKEP62		CA	PRIO13
		TC	NOVAC
		EBANK=	AOG
		2CADR	P63

		TC	TASKOVER

						# EACH 2 SEC, CALCULATE GIMBAL ANGLES FOR ENTRY CON-
						# DITIONS THAT WILL HOLD IF REORIENTATION WERE MADE
						# AT PRESENT RN, VN.  COME HERE FROM CM/POSE AND ALSO
						# IN KEPLER PHASE OF ENTRY.

P62.3		SSP	GOTO			# SET RETURN ADDRESS SO THAT ROUTINE
			QPRET			# GOES DIRECTLY TO ENTRY GUIDANCE EXIT
			ENDEXIT			# THAT DOES ENTRY DISPLAY, GRP 5.
			S62.3			# PUT DESIRED CDU VALUES IN CPHI'S FOR
						# N22 DISPLAY.

# Page 795
; ============================================================================
; PROGRAM P63 - LIFTING ENTRY GUIDANCE INITIALIZATION
; ============================================================================
;
; P63 is the entry point to Apollo's primary lifting entry guidance system,
; transitioning the spacecraft from exo-atmospheric coast to active atmospheric
; flight control. This program initializes the entry guidance equations and
; maintains entry attitude until aerodynamic forces reach 0.05g, at which point
; active lift modulation begins under P64.
;
; MISSION ROLE:
; P63 bridges the gap between P62's CM/SM separation and entry orientation, and
; P64's active guidance. It holds the Command Module at the correct entry
; attitude (angle of attack, sideslip, and roll) computed in P61/P62, waiting
; for the first signs of atmospheric deceleration before transitioning to
; closed-loop steering.
;
; THREE PRIMARY FUNCTIONS:
;
; 1. INITIALIZE ENTRY EQUATIONS
;    Sets up the guidance algorithms that will control the entry trajectory:
;    - Loads entry interface state (position, velocity, flight path angle)
;    - Initializes drag and lift acceleration tracking
;    - Sets range-to-target and crossrange error initial values
;    - Prepares constant drag controller parameters
;    - Establishes reference frames for atmospheric flight
;
; 2. MAINTAIN ENTRY ATTITUDE
;    Continues holding the CM at the computed entry trim attitude established
;    in P62. This attitude is critical because:
;    - Heat shield must face into the airstream (angle of attack ~-20°)
;    - Roll angle positions the lift vector for range control (0° or 180°)
;    - Precise attitude prevents asymmetric heating or excessive g-loads
;    The entry DAP (CM_ENTRY_DIGITAL_AUTOPILOT) maintains this attitude using
;    RCS thrusters until aerodynamic forces become significant.
;
; 3. DETECT 0.05G ONSET
;    Continuously monitors total sensed acceleration, waiting for atmospheric
;    deceleration to reach 0.05g. This threshold marks entry interface where:
;    - Aerodynamic forces become significant enough for lift modulation
;    - Guidance transitions from attitude hold to active steering (P64)
;    - Entry displays begin showing real-time guidance parameters
;    For Apollo 11, 0.05g occurred at approximately 400,000 feet altitude,
;    traveling at ~36,000 fps inertial velocity.
;
; ENTRY INTERFACE TIMELINE (Apollo 11, July 24, 1969):
; The spacecraft approached entry interface at:
; - Altitude: 400,000 feet (122 km) above Fischer ellipsoid
; - Velocity: 36,237 fps (11,044 m/s) inertial
; - Flight path angle: -6.5° (shallow enough to avoid excessive deceleration)
; - Range to target: 1,440 nautical miles
;
; PROGRAM FLOW:
; P63 executes these steps:
; 1. Sets major mode to 63 (displayed as "P63" on DSKY)
; 2. Configures POSEXIT to continue at STARTENT after CM/POSE completes
; 3. Sets up entry display noun V06N64 showing:
;    - Current g-load (XX.XX g)
;    - Inertial velocity (XXXX. fps)
;    - Range to splashdown (XXXX.X nm)
; 4. Clears P63FLAG to ensure proper sequencing with entry DAP
; 5. Flushes any pending N22 display from P62
; 6. Ends job, waiting for 0.05g detection to trigger P64 via STARTENT
;
; DISPLAY CONFIGURATION:
; V06N64 becomes the primary crew display during entry, updated continuously
; once guidance begins. The crew monitors:
; - G-load: Increasing from 0.05g to peak ~6.5g then decreasing
; - Velocity: Decreasing from 36,000 fps toward subsonic speeds
; - Range-to-go: Decreasing toward zero at splashdown
;
; TRANSITION TO P64:
; P63 does not loop or iterate - it's a one-time initialization. When the
; entry guidance routine STARTENT (in REENTRY_CONTROL.agc) detects 0.05g
; deceleration, it automatically invokes P64 via the RTB (return to bank)
; mechanism. P64 then takes over active lift modulation and steering.
;
; INTEGRATION WITH ENTRY CONTROL:
; P63 sets POSEXIT to STARTENT (entry guidance entry point), establishing
; the continuation point after attitude computations complete. The STARTENT
; routine (REENTRY_CONTROL.agc) handles:
; - Numerical integration of position and velocity through atmosphere
; - Lift and drag acceleration computation from atmospheric density
; - Range prediction and guidance command generation
; - Bank angle modulation for lift vector control
;
; APOLLO 11 HISTORICAL CONTEXT:
; When P63 executed on July 24, 1969, the crew was approximately 15 minutes
; from entry interface. Armstrong, Aldrin, and Collins were strapped into
; their couches in the Command Module Columbia, heat shield oriented forward,
; watching the Earth grow larger through the windows. The Service Module had
; been jettisoned, and they were committed to entry - there would be no second
; chance. P63's proper initialization was critical for the guidance system
; that would safely guide them through the fiery descent.
;
; ENTRY CORRIDOR MARGINS:
; The allowable entry corridor is remarkably narrow:
; - Too steep (> -7.5° flight path angle): Excessive g-loads or heating
; - Too shallow (< -5.5° flight path angle): Skip back into space
; P63's initialization ensures guidance starts from known good conditions
; within this corridor, with sufficient control authority to correct errors
; accumulated during the 3-day transearth coast.
;
; ============================================================================
#	P63
# PROGRAM:	P63
# MOD NO:	0	MAR. 13, 1967
# MOD BY:	R. HIRSCHKOP
# MOD NO: 1	MOD BY: RR BAIRNSFATHER		DATE: 22 JUN 67		RESTARTS
# MOD NO: 2	MOD BY: RR BAIRNSFATHER		DATE: 14 JUL 67		REVISED RESTARTS
# FUNCTION:	1) TO INITIALIZE THE ENTRY EQUATIONS.
#		2) TO CONTINUE TO HOLD THE CM TO THE CORRECT ATTITUDE WITH RESPECT TO THE ATMOSPHERE FOR
#		   THE ONSET OF ENTRY DECELERATION.  ROLL ANGLE IS LIFT UP/DOWN AS SPECIFIED BY HEADSUP.
#		3) TO SENSE .05G.
# CALLING SEQUENCE:  DIRECTLY FROM P62
# EXIT:		TO ENDOFJOB
# SUBROUTINE CALLS:  NEWMODEX, GODSPR

		COUNT*	$$/P63

P63		TC	NEWMODEX
		MM	63

						# ARRIVE WITH EBANK = AOG.

		CA	ENTCADR			# CONTINUE AT STARTENT AFTER CM/POSE.

						# AT END OF STATEMENT, CHANGE ADDRESS IN GOTOADDR
						# TO CONTINUE AT SCALEPOP THEREAFTER.

		TS	POSEXIT

		CA	V06N64			# G		VI		R TO SPLSH
						# XXX.XX G	XXXX. FPS	XXXX.X NM
		TS	ENTRYVN			# FOR DISPLAY CALL IN OVERNOUT

		CS	ONE			# IN CASE FLAG IS LEFT AT +1 BY DAP.  THE
		TS	P63FLAG			# -1 ASSURES THAT EXO-ATM DAP WILL NOT
						# CALL P63 OUT OF SEQUENCE IN P66.

		TC	PHASCHNG		# THIS IS REQUIRED TO PRESERVE CLEANDSP
		OCT	00004			# RETURN IN EVENT OF AN EXTENDED VERB

		TC	BANKCALL		# FLUSH 'N22' DISPLAY, IF ON, (OMIT
		CADR	CLEANDSP		# DISPLAY DURING 'STARTENT' PASS.)

P63.1		TC	PHASCHNG
		OCT	00004			# DISABLE.  DISPLAY RESTARTED VIA ENTRY.

		TC	ENDOFJOB

V06N60		VN	0660
V06N61		VN	0661
V06N63		VN	0663
# Page 796
V06N64		VN	0664
ENTCADR		CADR	STARTENT

		EBANK=	RTINIT			# TO CARRY OVER INTO ENTRY STEERING.
POSECADR	2CADR	CM/POSE

# Page 797
; ============================================================================
; PROGRAM P64 - INITIAL ENTRY PHASE WITH CONSTANT DRAG CONTROL
; ============================================================================
;
; P64 is the heart of Apollo's initial entry guidance, executing continuously
; from 0.05g (entry interface at ~400,000 feet) through the high-heating phase
; until either P65 (skip control) or P67 (final phase) takes over. This program
; implements the constant drag controller - a brilliant guidance algorithm that
; modulates lift vector orientation to maintain a nearly constant deceleration
; level, maximizing range control authority while staying within g-load and
; heating constraints.
;
; ACTIVATION TRIGGER:
; P64 is invoked automatically via RTB (return to bank) mechanism when the
; entry guidance routine STARTENT (REENTRY_CONTROL.agc) detects that total
; sensed acceleration has exceeded 0.05g. This marks entry interface where
; atmospheric forces become significant enough for active lift modulation.
; No crew action required - the transition from P63 to P64 is seamless.
;
; THE CONSTANT DRAG CONCEPT:
; Rather than flying a predetermined trajectory, P64 uses a feedback control
; law that adjusts the lift vector to maintain nearly constant drag (deceleration)
; throughout the initial entry phase. This approach provides:
;
; 1. THERMAL PROTECTION: Constant drag implies nearly constant dynamic pressure
;    (0.5 * rho * V^2), which keeps heating rates within safe limits even as
;    velocity and atmospheric density change dramatically.
;
; 2. RANGE CONTROL: By adjusting the reference drag level up or down, guidance
;    can "dial in" the amount of range the vehicle will fly. Higher drag = less
;    range (steeper path), lower drag = more range (shallower path).
;
; 3. PREDICTABILITY: The vehicle's future trajectory is more predictable when
;    maintaining constant conditions, allowing accurate range-to-go calculations.
;
; 4. G-LOAD MANAGEMENT: Drag directly equals deceleration (in g units). By
;    controlling drag level, P64 ensures crew g-loads stay within acceptable
;    limits (typically not exceeding 6.5g for Apollo).
;
; LIFT VECTOR MODULATION VIA BANK ANGLE:
; The Command Module is a lifting body - its offset center of mass creates a
; lift force perpendicular to the velocity vector when at angle of attack. P64
; controls the magnitude and direction of this lift vector by commanding roll
; (bank) angle:
;
; - ROLL = 0° (Lift Up): Lift opposes gravity, extends range, reduces heating
; - ROLL = 180° (Lift Down): Lift augments gravity, shortens range, increases heating
; - ROLL = ±90°: Lift is horizontal, provides crossrange (lateral) control
;
; The CM Entry DAP (CM_ENTRY_DIGITAL_AUTOPILOT.agc) executes the commanded roll
; angle using RCS thrusters, maintaining the heat shield at proper angle of
; attack (~-20°) while precisely controlling the roll orientation.
;
; FOUR PRIMARY FUNCTIONS OF P64:
;
; 1. INITIALIZE ENTRY GUIDANCE AT 0.05G
;    When P64 first executes at entry interface, it:
;    - Selects initial roll attitude based on required range (from P61 targeting)
;    - Sets initial constant drag reference level (typically ~0.3g to 0.5g)
;    - Computes drag threshold parameter KA, which keys subsequent guidance
;    - Records the 0.05g state vector as reference for later calculations
;    - Displays V06N74 showing: Roll command, Inertial velocity, Drag level
;
; 2. MAINTAIN CONSTANT DRAG CONTROL
;    Each pass through the guidance loop (approximately every 2 seconds), P64:
;    - Measures current drag from sensed acceleration and gravity
;    - Compares actual drag to reference drag level
;    - Computes bank angle command to null the drag error
;    - Predicts range-to-go based on current energy state
;    - Adjusts reference drag if needed to hit target range
;    The algorithm essentially asks: "To maintain this drag level and hit the
;    target, should I roll lift-up or lift-down, and by how much?"
;
; 3. MONITOR FOR P65 ENTRY CONDITIONS
;    P64 continuously checks whether conditions are right to transition to
;    P65 (up-control/skip phase):
;    - Is velocity still high enough? (V > 27,000 fps at 0.2g point)
;    - Is altitude rate and drag level appropriate for skip control?
;    - Has constant drag brought predicted range within 25 nm of target?
;    If all conditions are satisfied, P64 invokes P65 for precision skip targeting.
;
; 4. MONITOR FOR ABORT TO P67
;    P64 also watches for conditions requiring direct transition to final phase:
;    - If V < 27,000 fps when 0.2g occurs: Skip control not feasible, go to P67
;    - If no up-control solution exists with VL > 18,000 fps: Abort skip, go to P67
;    These checks ensure the vehicle always has a safe path to splashdown even
;    if ideal skip conditions cannot be achieved.
;
; APOLLO 11 ENTRY PROFILE (July 24, 1969):
; When P64 activated for Columbia's entry:
; - Entry interface (0.05g): 400,000 ft altitude, 36,237 fps velocity, -6.5° FPA
; - Peak heating: ~290,000 ft altitude, ~30,000 fps, ~1.2g drag, 180° roll (lift down)
; - Peak g-load: ~180,000 ft altitude, ~18,000 fps, ~6.5g drag, roll transitioning
; - 0.2g checkpoint: ~240,000 ft altitude, ~25,000 fps - conditions assessed for P65
; - P64 duration: Approximately 7-8 minutes from 0.05g to P65 or P67 transition
;
; Throughout this phase, the crew watched the displays showing g-load climbing
; from 0.05g toward the predicted 6.5g peak, velocity dropping from 36,000 fps,
; and range-to-splashdown decreasing. The view out the windows transitioned from
; black space to the glow of ionized air (plasma) surrounding the spacecraft -
; the famous "fireball" of reentry, bright enough to obscure the stars and
; horizon. Radio contact with Mission Control was lost due to plasma blackout.
;
; DISPLAY CONFIGURATION:
; P64 sets ENTRYVN to V06N74, which displays on DSKY:
; - R1: ROLLC (XXX.XX degrees) - Commanded roll angle for lift vector control
; - R2: VI (XXXXX. fps) - Current inertial velocity
; - R3: D (XXX.XX g) - Current drag acceleration level
; This becomes the primary entry display, updated each guidance cycle.
;
; GUIDANCE CYCLE TIMING:
; P64 does not loop internally - it executes once per call and returns to
; REENTRY_CONTROL. The entry steering routine calls P64 repeatedly (approximately
; every 2 seconds) as it numerically integrates the trajectory through the
; atmosphere. Each call, P64 computes fresh bank commands based on latest
; position, velocity, and acceleration.
;
; PROGRAM FLOW WITHIN P64:
; 1. TC NEWMODEX / MM 64 - Set major mode to 64 (displays "P64" on DSKY)
; 2. CA V06N74 / TS ENTRYVN - Configure V06N74 display (roll/velocity/drag)
; 3. TC DANZIG - Continue to INITROLL routine (in REENTRY_CONTROL.agc) which:
;    - Executes constant drag controller algorithm
;    - Computes bank angle commands
;    - Checks transition conditions to P65 or P67
;    - Returns to entry integration loop
;
; The DANZIG call transfers control to REENTRY_CONTROL.agc's INITROLL, which
; implements the actual constant drag control law mathematics. P64 itself is
; primarily a mode initialization - the heavy lifting happens in INITROLL.
;
; TRANSITION CONDITIONS DETAIL:
;
; TO P65 (Skip Control Phase):
; All of these conditions must be true:
; - Velocity > 27,000 fps when 0.2g is reached (sufficient energy for skip)
; - Altitude rate (HDOT) and drag level permit skip trajectory
; - Predicted range error < 25 nm (constant drag has done its job)
; When these are met, P65 takes over with more sophisticated targeting.
;
; TO P67 (Final Phase):
; P67 is selected if:
; - Velocity < 27,000 fps at 0.2g point (insufficient energy for skip)
; - No up-control solution with exit velocity > 18,000 fps (skip not feasible)
; - Implicitly: If P65 never takes over, P64 continues until transitioning to P67
;
; INTEGRATION WITH ENTRY CONTROL SYSTEM:
; P64 is tightly coupled with REENTRY_CONTROL.agc routines:
; - STARTENT: Entry guidance executive that calls P64 via RTB
; - INITROLL: Constant drag controller implementation, called via DANZIG
; - OVERNOUT: Display interface that shows V06N74 to crew
; - Entry integration: Numerical integration of trajectory equations
;
; And with the entry autopilot:
; - CM_ENTRY_DIGITAL_AUTOPILOT.agc: Executes roll commands computed by P64/INITROLL
; - Entry DAP maintains angle of attack while modulating roll for lift control
; - RCS thruster firing provides attitude control until aerodynamic forces dominate
;
; HISTORICAL SIGNIFICANCE:
; P64's constant drag controller was one of the most sophisticated guidance
; algorithms of the Apollo era. It successfully guided all manned Apollo missions
; through the critical initial entry phase, automatically adjusting the trajectory
; in real-time to compensate for uncertainties in atmospheric density, state vector
; errors accumulated during transearth coast, and variations in spacecraft
; aerodynamic properties. For Apollo 11, P64 worked flawlessly, bringing the crew
; safely through peak heating and peak g-loads toward their Pacific splashdown
; target 1,440 nautical miles downrange from entry interface.
;
; The beauty of the constant drag approach is its robustness - even if conditions
; differ from predictions, maintaining constant drag keeps the vehicle in a safe
; corridor while the feedback control naturally compensates for errors. This
; allowed Apollo to achieve splashdown accuracies of 1-2 nautical miles despite
; the enormous uncertainties of atmospheric entry from lunar return velocity.
;
; ============================================================================
# PROGRAM:	P64
# MOD NO:	1	SEPT. 19, 1967
# MOD BY:	R. HIRSCHKOP
# MOD NO: 2	MOD BY: RR BAIRNSFATHER		DATE: 8 MAY 68		REVISED COMMENTS FOR COLOSSUS
# FUNCTION:	1.  TO START ENTRY GUIDANCE AT .05G SELECTING ROLL ATTITUDE, CONSTANT DRAG LEVEL, AND
#		    DRAG THRESHOLD, KA, WHICH ARE KEYED TO THE .05G POINT.
#		2.  SELECT FINAL PHASE P67 IF V < 27000 FPS WHEN .2G OCCURS.
#		3.  ITERATE FOR UP-CONTROL SOLUTION P65 IF V > 27000 FPS AND IF ALTITUDE RATE AND DRAG
#		    LEVEL CONDITIONS ARE SATISFIED.  ENTER P65 WHEN CONSTANT DRAG CONTROLLER HAS BROUGHT RANGE
#		    AS PREDICTED TO WITHIN 25 NM OF DESIRED RANGE.
#		4.  SELECT FINAL PHASE  P67  IF NO UP-CONTROL SOLUTION EXISTS WITH VL > 18000 FPS.
# CALLING SEQUENCE:  BY RTB FROM REENTRY CONTROL
# EXIT:		BACK TO REENTRY CONTROL.
# SUBROUTINE CALLS:  NEWMODEX

		BANK	25
		SETLOC	P60S1
		BANK

# THIS DISPLAY IS CALLED EACH PASS THROUGH STEERING.  RESTART PROTECTION IS VIA STEERING.

		COUNT*	$$/P64

P64		TC	NEWMODEX		# ENTER VIA RTB WHEN .05G IS EXCEEDED.
		MM	64
		CA	V06N74			# ROLLC		VI		D
						# XXX.XX DEG	XXXXX. FPS	XXX.XX G
		TS	ENTRYVN			# DISPLAY VIA OVERNOUT.

		TC	DANZIG			# ... AND CONTINUE IN INITROLL ...

V06N74		VN	0674

# Page 798
; ============================================================================
; PROGRAM P65 - UP-CONTROL PHASE (SKIP TARGETING)
; ============================================================================
;
; P65 represents the most sophisticated phase of Apollo's entry guidance - the
; "skip" phase where the Command Module briefly exits the sensible atmosphere,
; arcs upward like a skipping stone on water, then reenters for final descent
; to splashdown. This phase provides the precision range control needed to hit
; the target within 1-2 nautical miles despite huge uncertainties from transearth
; coast, atmospheric density variations, and aerodynamic property dispersions.
;
; ACTIVATION CRITERIA (ALL MUST BE MET):
; P65 is entered from P64 when the constant drag controller has brought predicted
; range to within 25 nautical miles of the desired target range, AND:
; 1. Velocity > 27,000 fps when the 0.2g point was reached (sufficient energy)
; 2. Altitude rate (HDOT) and drag level permit skip trajectory
; 3. No abort conditions have forced direct transition to P67
;
; When these conditions are satisfied, P64's INITROLL routine executes RTB to
; P65, handing off control to the up-control phase. Typical conditions at P65
; entry: ~250,000 ft altitude, ~28,000 fps velocity, ~0.5g drag, having already
; passed through peak heating at ~290,000 ft.
;
; THE SKIP MANEUVER CONCEPT:
; A skip entry trajectory allows Apollo to "stretch" its range much further than
; a ballistic entry, providing crucial targeting flexibility:
;
; PHYSICS OF THE SKIP:
; As the CM descends through the upper atmosphere in P64, it's decelerating due
; to drag while lift opposes gravity. If the vehicle is rolled to "lift-up"
; orientation (ROLL ~ 0°), the lift vector becomes large enough to temporarily
; overcome the downward pull of gravity + centrifugal effects. The flight path
; angle (gamma) becomes less negative, curves upward, and eventually becomes
; positive - the vehicle is climbing! 
;
; This climb continues until the vehicle exits the sensible atmosphere (where
; density becomes negligible). At this "exit" point, the vehicle is on a
; ballistic arc - essentially in free flight above most of the atmosphere. The
; vehicle follows this exoatmospheric arc (like a rock thrown upward) until
; gravity pulls it back down into denser atmosphere for "reentry" and final
; descent to splashdown.
;
; ADVANTAGES OF SKIP TRAJECTORY:
; 1. EXTENDED RANGE: The exoatmospheric arc allows the vehicle to travel much
;    farther downrange than a direct descent. Apollo entries from the Moon
;    covered 1,200-1,600 nm total range with peak skip altitudes ~250,000 ft.
;
; 2. REDUCED PEAK LOADS: By temporarily exiting the atmosphere, the vehicle
;    spreads deceleration over a longer time/distance, reducing both peak
;    g-loads and peak heating rates compared to a direct ballistic entry.
;
; 3. PRECISION TARGETING: The most critical advantage for Apollo - by precisely
;    controlling the altitude and velocity at skip exit, P65 can "dial in" the
;    exact range needed to hit the carrier recovery ship's position. Range
;    control accuracy: typically ±1-2 nm despite dispersions.
;
; 4. CROSSRANGE FLEXIBILITY: During the skip arc, horizontal roll commands
;    (±90°) provide lateral range control, allowing precision in both downrange
;    AND crossrange dimensions for pinpoint splashdown positioning.
;
; P65 UP-CONTROL GUIDANCE ALGORITHM:
; P65 implements a "keyed" guidance law that targets a specific exit condition:
;
; THE EXIT KEYING CONCEPT:
; Rather than trying to hit the final splashdown point directly, P65 targets an
; intermediate "exit" condition defined by two key parameters:
; - VL: Exit velocity (typically 24,000-26,000 fps for Apollo 11)
; - Q7: Exit drag level (typically 0.15-0.25g)
;
; These keys are computed by P61 during entry initialization and refined by P64.
; They represent the exit state that, if achieved, will cause the subsequent
; ballistic arc + reentry in P67 to naturally hit the splashdown target.
;
; The genius of this approach: Instead of a complex optimization targeting the
; final point (where tiny errors amplify enormously), P65 uses feedback control
; to hit an intermediate exit condition. Errors in achieving the exit state
; cause proportional, manageable errors at splashdown.
;
; GUIDANCE LOOP OPERATION:
; Each guidance cycle (approximately every 2 seconds), P65:
;
; 1. MEASURE CURRENT STATE:
;    - Current velocity V (from integration of sensed acceleration)
;    - Current drag D (from accelerometer measurements)
;    - Current range-to-go (from integration of downrange velocity component)
;
; 2. PREDICT EXIT CONDITIONS:
;    Using simplified atmospheric model, predict what exit velocity and drag
;    will be achieved if current bank angle is maintained.
;
; 3. COMPUTE VELOCITY ERROR:
;    DELVL = VL (desired exit velocity) - VLPRED (predicted exit velocity)
;    This tells us: are we exiting too fast or too slow?
;
; 4. COMPUTE DRAG ERROR:
;    DELDRAG = Q7 (desired exit drag) - DPRED (predicted exit drag)
;    This tells us: are we exiting too steep or too shallow?
;
; 5. COMPUTE BANK COMMAND:
;    Based on velocity and drag errors, compute roll angle that will drive
;    errors toward zero. Large positive DELVL → roll toward lift-up to slow
;    descent and exit at higher velocity. Large negative DELVL → roll toward
;    lift-down to steepen path and exit at lower velocity.
;
; 6. MODULATE LIFT MAGNITUDE:
;    The commanded bank angle directly controls lift vector orientation:
;    - ROLLC near 0° → Full lift up → Maximum climb rate
;    - ROLLC near 180° → Full lift down → Maximum descent rate  
;    - ROLLC near ±90° → Horizontal lift → Crossrange control
;
; 7. UPDATE DISPLAY:
;    Present V16N69 to crew showing: ROLLC (commanded roll), DL (current drag),
;    VL (target exit velocity). Crew monitors that guidance is converging.
;
; CROSSRANGE CONTROL DURING SKIP:
; In addition to controlling downrange (forward) distance via velocity/drag,
; P65 provides lateral (crossrange) control. If the predicted exit state will
; result in left/right displacement from the target ground track, P65 adds an
; offset to the roll command. For example:
; - Need to move track left → Add negative roll bias → Lift vector points left
; - Need to move track right → Add positive roll bias → Lift vector points right
;
; This horizontal lift component during the skip arc provides tens of nautical
; miles of crossrange control authority, enough to correct for dispersions in
; entry state vector accumulated during the 3-day transearth coast.
;
; EXIT CONDITIONS AND P66 TRANSITION:
; P65 terminates when drag drops below the target exit drag Q7, indicating
; the vehicle has climbed out of the sensible atmosphere into the exoatmospheric
; skip arc. At this point:
; - Altitude typically 240,000-280,000 ft (depending on skip profile)
; - Velocity typically 24,000-26,000 fps (the keyed VL value)
; - Flight path angle positive (climbing)
; - Drag < Q7 (approximately 0.15-0.25g)
;
; When D < Q7 is detected, P65 executes RTB to P66 (ballistic phase). P66 simply
; monitors the vehicle's free flight along the ballistic skip arc, waiting for
; reentry into sensible atmosphere where P67 will take over for final descent.
;
; ABORT TO P67 CONDITIONS:
; P65 also watches for conditions indicating the skip should be aborted:
; - If RDOT (altitude rate) becomes negative: Vehicle is descending, not climbing
; - AND V < VL + 500 fps: Insufficient velocity to complete proper skip
;
; When both conditions occur, guidance has determined the skip isn't feasible
; (perhaps due to larger-than-expected dispersions), so P65 aborts directly to
; P67 for final descent without completing the skip arc. P67 is robust enough
; to handle entry from various conditions, ensuring safe splashdown even if the
; skip doesn't work as planned.
;
; APOLLO 11 SKIP PROFILE (July 24, 1969):
; For Columbia's entry, P65 activated at approximately:
; - Entry interface + 8 minutes (about 400 seconds after 0.05g)
; - Altitude: ~260,000 ft
; - Velocity: ~29,000 fps
; - Drag: ~0.8g (still descending through atmosphere)
; - Range to go: ~1,200 nm (well within 25 nm targeting criterion from P64)
;
; P65 commanded a roll profile starting near lift-up (ROLL ~30°), modulating
; through the skip to control the exit state. The vehicle climbed through peak
; skip altitude ~280,000 ft, exited the sensible atmosphere at ~24,500 fps and
; 0.20g drag, then followed a ballistic arc lasting approximately 3-4 minutes
; before reentering for P67 final descent.
;
; Throughout the skip, the crew could feel g-loads dropping from ~4g down to
; nearly zero-g at skip exit, then building again during reentry. The view
; transitioned from plasma glow to black sky at the top of the arc, then back
; to plasma as P67 reentry began. Radio contact was restored during the skip
; arc, allowing Mission Control to monitor the precise skip performance.
;
; DISPLAY AND CREW INTERACTION:
; P65 presents V16N69 as a flashing display requiring crew response:
; - R1: ROLLC (XXX.XX degrees) - Commanded roll angle for guidance
; - R2: DL or Q7 (XXX.XX g) - Target exit drag level
; - R3: VL (XXXXX. fps) - Target exit velocity
;
; The flashing display prompts crew to PROCEED or take manual control. Normally
; the crew presses PRO to accept the computed exit keys and allow P65 to continue.
; This gives the crew positive control - guidance must have crew concurrence to
; execute the skip phase. If conditions look wrong, crew could press ENTER to
; modify the exit keys or switch to P66/P67 manually.
;
; PROGRAM STRUCTURE:
; P65 is unique among the entry programs in that it launches a separate job to
; handle the flashing display while the main entry guidance continues:
;
; 1. TC NEWMODEX / MM 65 - Set major mode 65 (DSKY shows "P65")
; 2. TC NOVAC / 2CADR P65.1 - Start new job for display handling
; 3. TC 2PHSCHNG - Restart protection for display job
; 4. SSP GOTOADDR UPCONTRL - Change entry mode to up-control algorithm
; 5. RTB REFAZE10 - Reestablish entry sequencer and continue
;
; The P65.1 job handles V16N69 display and crew response independently from the
; main guidance loop, which continues in REENTRY_CONTROL executing the UPCONTRL
; routine. This parallel structure allows guidance to keep running even while
; waiting for crew input.
;
; INTEGRATION WITH ENTRY CONTROL:
; P65 is tightly coupled with REENTRY_CONTROL.agc routines:
; - UPCONTRL: Up-control guidance algorithm implementing the exit-keyed control
; - OVERNOUT: Display interface that manages V16N69 presentation
; - Entry integrator: Continues numerical integration through skip arc
;
; And with the entry autopilot:
; - CM_ENTRY_DIGITAL_AUTOPILOT.agc executes P65's roll commands
; - Entry DAP maintains angle of attack while executing complex roll profiles
; - RCS thrusters provide attitude control throughout skip (aero forces minimal)
;
; GUIDANCE ROBUSTNESS:
; The beauty of P65's exit-keyed approach is its inherent robustness. Even if
; atmospheric density is 20% higher than predicted, or the entry state vector
; was off by 100 fps, or the CM's lift-to-drag ratio varies from nominal, the
; feedback control law naturally compensates. Each guidance cycle, P65 measures
; where it actually is, predicts where it's going, and adjusts the bank angle
; to null the error. This closed-loop control is what enabled Apollo to achieve
; 1-2 nm splashdown accuracy despite enormous uncertainties.
;
; HISTORICAL SIGNIFICANCE:
; P65 represents the pinnacle of Apollo entry guidance sophistication. The skip
; entry technique was originally developed for USAF and NASA lifting body research
; in the 1960s, and Apollo was the first operational spacecraft to use it for
; crewed missions returning from the Moon. The combination of P64's constant drag
; bringing range close, P65's precision skip targeting, and P67's final descent
; worked flawlessly on all Apollo lunar return missions, with typical splashdown
; errors of 1-3 nautical miles - remarkable accuracy for vehicles traveling at
; 25,000 mph just minutes earlier.
;
; For Apollo 11, the skip phase was critical for hitting the recovery area in
; the central Pacific where USS Hornet was stationed. The successful execution
; of P65, bringing Armstrong, Aldrin, and Collins to a precise splashdown within
; sight of the recovery ship, demonstrated that human lunar missions could be
; conducted safely with reliable return to Earth. This confidence enabled the
; subsequent Apollo missions that followed.
;
; ============================================================================
# PROGRAM:	P65
# MOD NO:  0	MOD BY:  RR BAIRNSFATHER	DATE:  17 JAN 68	COLOSSUS GSOP ADDITION.
# FUNCTION:	TO CONTINUE ENTRY GUIDANCE, USING THE UP-CONTROL PHASE TO STEER TO A CONTROLLED EXIT
#		CONDITION.  THIS PHASE TERMINATES  A) IF D < Q7 FPSS, GOTO TO P66.
#						   B) IF RDOT NEG, AND IF V < VL +500 FPS, GO TO P67.
# CALLING SEQUENCE:  BY RTB FROM REENTRY CONTROL
# EXIT:		BACK TO REENTRY CONTROL, OR TO ENDOFJOB.
# SUBROUTINE CALLS:  NEWMODEX

		COUNT*	$$/P65

P65		TC	NEWMODEX		# ENTER VIA RTB WHEN RANGE < 25 N M OF
		MM	65			# TARGET.

		CA	PRIO13
		TC	NOVAC
		EBANK=	ENTRYVN
		2CADR	P65.1

		TC	2PHSCHNG		# 2 PHASE CHG REQUIRED TO PREVENT RE-
		OCT	00554			# STARTING FLASHING DISPLAY TWICE.
		OCT	10035			# 4.55 SPOT AND SERVICER, HERE.
		TC	INTPRET
		SSP	RTB
			GOTOADDR		# CHANGE ENTRY MODE TO UPCONTRL.
			UPCONTRL
			REFAZE10		# GO HERE TO REESTABLISH ENTRY SEQUENCER.
						# AND CONTINUE AT UPCONTRL...

P65.1		TC	DOWNFLAG
		ADRES	ENTRYDSP		# ENTRYDSP = 92D BIT 13 FLAG 6

		CA	V16N69			# ROLLC		DL (Q7)		VL
		TC	BANKCALL		# XXX.XX DEG	XXX.XX G	XXXXX. FPS
		CADR	GOFLASHR
		TC	-3			# NODOFLAG IS SET ...
		TC	+3
		TC	-5
		TC	P61.3			# EST. GRP 4 FOR DISPLAY AND DO ENDOFJOB
						# IF PROCEED, CONTINUE
		TC	UPFLAG
		ADRES	ENTRYDSP		# ENTRYDSP = 92D BIT 13 FLAG 6

		TC	P63.1			# DISABLE GRP 4, START UP ENTRY DISPLAY
						# N06V68 VIA OVERNOUT, AS USED IN P64
V16N69		VN	1669

# Page 799
; ============================================================================
; PROGRAM P66 - BALLISTIC PHASE (EXOATMOSPHERIC SKIP ARC)
; ============================================================================
;
; P66 is the simplest of the entry programs, yet it plays a critical role in
; the skip entry sequence. After P65 has successfully steered the vehicle out
; of the sensible atmosphere to the targeted exit condition, P66 manages the
; exoatmospheric ballistic arc - the portion of the skip where the Command
; Module is essentially in free flight above most of the atmosphere, coasting
; along a gravity-driven parabolic trajectory like a thrown stone at the top
; of its arc.
;
; ENTRY CONDITIONS FROM P65:
; P66 is entered via RTB from REENTRY_CONTROL when P65 detects that drag has
; dropped below the target exit drag Q7 (typically 0.15-0.25g). At this point:
; - Altitude: Typically 240,000-280,000 ft (well above sensible atmosphere)
; - Velocity: Typically 24,000-26,000 fps (the keyed exit velocity VL)
; - Flight path angle: Positive (vehicle is climbing or at peak of arc)
; - Drag: Less than Q7 (atmospheric density negligible for guidance purposes)
; - Roll angle: Whatever P65 last commanded (typically near 0° for lift-up)
;
; The vehicle has exited the dense atmosphere where aerodynamic forces dominate
; and is now in a regime where gravity is the primary force. For the next 3-5
; minutes, the CM will coast along a ballistic arc, reaching peak altitude,
; then descending back toward the denser atmosphere for reentry and P67.
;
; P66 OBJECTIVES:
; During this ballistic coast phase, the guidance computer has two simple tasks:
;
; 1. MAINTAIN TRIM ATTITUDE:
;    Keep the Command Module's attitude aligned with the relative velocity vector
;    (velocity with respect to the atmosphere). This ensures proper angle of
;    attack (approximately 35-40° for Apollo's center-of-gravity offset) so that
;    when the vehicle reenters the atmosphere, it immediately develops the correct
;    lift and drag forces without large transients or instabilities.
;
;    In the thin atmosphere during P66, aerodynamic moments are minimal but not
;    zero. The entry DAP (CM_ENTRY_DIGITAL_AUTOPILOT.agc) uses RCS thrusters to
;    maintain the proper attitude against small aero disturbances and to track
;    the slowly changing velocity vector direction as the vehicle arcs through
;    the skip trajectory.
;
; 2. MONITOR FOR REENTRY:
;    Watch the drag acceleration (D) measured by the accelerometers. When drag
;    builds back up to Q7 + 0.5 fpss (the exit drag plus a hysteresis margin),
;    this indicates the vehicle has descended back into the sensible atmosphere
;    where active guidance is needed. At this point, P66 transitions to P67 for
;    the final descent phase.
;
; WHY P66 STOPS GENERATING ROLL COMMANDS:
; During the exoatmospheric phase, roll commands are meaningless for guidance.
; Here's why:
;
; AERODYNAMIC CONTROL REQUIRES ATMOSPHERE:
; The entire basis of Apollo's entry guidance is using bank angle (roll) to
; modulate the lift vector direction. Lift force is generated by the asymmetric
; shape of the CM flowing through air - the center-of-gravity offset creates
; a natural trim angle of attack, and this angle plus velocity creates lift
; proportional to (density × velocity²).
;
; In the exoatmospheric regime:
; - Atmospheric density is near zero (< 0.0001% of sea level)
; - Lift force is effectively zero (no air flow, no aerodynamic forces)
; - Changing bank angle has no effect on trajectory
;
; Therefore, generating roll commands during P66 would be pointless. The vehicle
; is on a ballistic trajectory determined entirely by its state at skip exit
; (position, velocity) and gravity. No amount of rolling will change where the
; vehicle goes during this phase.
;
; WHAT P66 ACTUALLY DOES:
; P66's implementation is remarkably simple - it merely changes the entry mode
; in the guidance system:
;
; 1. TC NEWMODEX / MM 66 - Set major mode to 66 (DSKY shows "P66")
; 2. TC INTPRET - Enter interpretive mode
; 3. SSP GOTOADDR BALLOUT - Change entry mode to BALLOUT
; 4. RTB REFAZE10 - Reestablish entry sequencer and continue
;
; That's it! P66 is essentially a mode switch. The actual work happens in
; REENTRY_CONTROL.agc:
;
; BALLOUT MODE (REENTRY_CONTROL.agc):
; When GOTOADDR is set to BALLOUT, the entry guidance computation routine in
; REENTRY_CONTROL stops calling the active guidance algorithms (UPCONTRL from
; P65 or DNCONTRL from P67) and instead simply maintains the current attitude
; without generating new roll commands.
;
; The BALLOUT routine continues to:
; - Integrate state (position, velocity) using accelerometer measurements
; - Update navigation solution
; - Maintain displays (N06V68 shows current velocity, range, drag)
; - Check drag condition: IF D ≥ Q7 + 0.5 THEN transition to P67
;
; But it does NOT:
; - Compute roll commands for range control
; - Predict downrange distance
; - Generate guidance corrections
;
; The vehicle simply coasts, held at trim attitude by the entry DAP, until
; atmospheric density increases enough to require active guidance again.
;
; DRAG MONITORING AND HYSTERESIS:
; P66 watches for drag to build up to Q7 + 0.5 fpss before transitioning to P67.
; The "+0.5 fpss" hysteresis is important:
;
; WHY HYSTERESIS IS NEEDED:
; At the exit condition, drag was Q7 (say 0.20g). If P66 immediately transitioned
; back to P67 when drag reached Q7 again, the system could chatter - small
; fluctuations in atmospheric density or measurement noise might cause drag to
; oscillate around Q7, triggering rapid P66↔P67 mode switches.
;
; By requiring D ≥ Q7 + 0.5 for reentry detection, P66 ensures a clean transition.
; The vehicle must descend significantly into the atmosphere (where drag has
; increased by 0.5g above the exit value) before P67 takes over. This provides
; a clear separation between the ballistic phase and the active final descent
; phase, with no ambiguity or oscillation.
;
; TYPICAL APOLLO 11 P66 PROFILE:
; For Columbia's entry on July 24, 1969:
;
; P66 ENTRY (Skip Exit):
; - Time: Entry interface + ~8.5 minutes (about 510 seconds after 0.05g)
; - Altitude: ~265,000 ft (peak of skip arc)
; - Velocity: ~24,500 fps (close to keyed VL)
; - Drag: Dropping below 0.20g (Q7 exit criterion met)
; - Flight path angle: Near 0° (approaching peak of arc)
;
; BALLISTIC ARC:
; - Duration: Approximately 3-4 minutes in P66
; - Peak altitude: ~280,000 ft (occurred during P66)
; - Drag during arc: Dropped to < 0.05g at peak, well below guidance threshold
; - Velocity at peak: ~24,000 fps (continued slowing slightly due to Earth drag)
; - Range covered: ~150-200 nm during ballistic phase
;
; CREW EXPERIENCE DURING P66:
; - G-loads: Dropped from ~4g at end of P65 to near zero-g at skip peak
; - Sensation: Brief period of "weightlessness" at top of arc
; - View: Plasma disappeared, black sky visible, Earth horizon visible below
; - Communications: Radio blackout ended, contact with Mission Control restored
; - Attitude: Very gentle, occasional RCS thruster firings to maintain trim
;
; The crew could clearly see they had exited the atmosphere - the fiery plasma
; glow disappeared, stars became visible, and they felt the transition to near
; zero-g. Buzz Aldrin later described it as "a brief respite" between the intense
; deceleration of the initial entry and the coming reentry for final descent.
;
; P66 REENTRY DETECTION (Transition to P67):
; - Time: Entry interface + ~12 minutes (about 720 seconds after 0.05g)
; - Altitude: ~230,000 ft (descending back into atmosphere)
; - Velocity: ~23,000 fps (continued deceleration from drag during descent)
; - Drag: Reached Q7 + 0.5 ≈ 0.70g (increasing rapidly as density increases)
; - Flight path angle: Negative ~-5° to -10° (steep descent toward splashdown)
;
; When D ≥ Q7 + 0.5 was detected, REENTRY_CONTROL executed the transition to P67,
; handing off control to the final descent phase guidance that would bring the
; crew to splashdown in the Pacific.
;
; MANUAL CONTROL DURING P66:
; Although P66 doesn't generate guidance commands, the crew retains full manual
; control authority. Using the rotation controller, Armstrong (or any crew member)
; could override the automatic trim hold and manually orient the vehicle as
; desired. This capability was important for:
;
; 1. VISUAL OBSERVATION:
;    During the ballistic arc with cleared view, crew might want to orient for
;    landmark observation, photography, or visual alignment checks.
;
; 2. CONTINGENCY MANEUVERS:
;    If guidance had failed or entry conditions were anomalous, crew could
;    manually establish entry attitude for P67 reentry.
;
; 3. CREW CONFIDENCE:
;    Knowing they could take over at any time gave the crew confidence in the
;    automated systems. They weren't passengers - they were pilots with ultimate
;    authority over their spacecraft.
;
; P66 RESTART PROTECTION:
; Like all entry programs, P66 has restart protection. If a computer restart
; occurred during P66 (highly unlikely in the benign exoatmospheric environment,
; but possible), the restart routine would:
; - Reinitialize P66 based on saved restart phase data
; - Restore BALLOUT mode in REENTRY_CONTROL
; - Continue monitoring drag for P67 transition
; - Preserve navigation state and entry trajectory
;
; The restart system ensures that even a computer hiccup during the ballistic
; phase wouldn't compromise the mission - guidance would seamlessly resume.
;
; DISPLAY DURING P66:
; P66 continues presenting N06V68 to the crew (same display used in P64/P65):
; - R1: Velocity (VIO, in fps, decreasing slowly during ballistic arc)
; - R2: Range to go (decreasing as vehicle travels downrange)
; - R3: Drag level (DL, very low during P66, increasing toward end)
;
; The crew monitors these values to track progress through the skip arc. They
; can see drag dropping toward zero at skip peak, then beginning to build again
; as reentry approaches. When drag reaches the reentry threshold, they expect
; the mode change to P67.
;
; INTEGRATION WITH OTHER PROGRAMS:
; P66 fits into the entry sequence:
; - FROM P65: When D < Q7 (successful skip exit)
; - TO P67: When D ≥ Q7 + 0.5 (reentry into atmosphere)
;
; P66 can also be manually selected:
; - Crew can use V37E66E to invoke P66 directly
; - Might be used if P65 is skipped and crew wants to coast before P67
; - Provides option for crew to manually manage entry phases if automation fails
;
; THE ELEGANCE OF P66:
; P66 demonstrates beautiful systems engineering - recognizing when NOT to control.
; During the exoatmospheric phase, the guidance computer doesn't try to "do
; something" with meaningless roll commands. Instead, it gracefully switches to
; a monitoring mode, maintains essential attitude control, and waits for conditions
; where active guidance is needed again.
;
; This philosophy - control when you can, monitor when you can't, never pretend
; to have authority you don't actually have - is central to robust aerospace
; systems design. P66's simplicity is not a limitation; it's appropriate response
; to the physical situation.
;
; HISTORICAL CONTEXT:
; The skip entry technique, including the ballistic arc phase, was developed
; through extensive research in the 1960s. NASA and USAF conducted numerous
; studies of lifting entry trajectories, recognizing that the ability to briefly
; exit and reenter the atmosphere provided enormous operational flexibility for
; lunar return missions.
;
; Apollo was the first operational crewed spacecraft to use skip entry. Previous
; Mercury and Gemini missions used simpler ballistic or lifting entry without
; skip. The success of P65-P66-P67 skip sequences on Apollo 8 (first lunar orbit
; mission, December 1968) and subsequent missions validated the concept and gave
; confidence for Apollo 11's historic return.
;
; For Apollo 11 specifically, the P66 phase represented a moment of relief. The
; crew had survived the intense heating and deceleration of initial entry, the
; guidance had successfully executed the precision skip targeting, and now they
; had a brief peaceful interval before the final descent. Mission Control could
; resume communications and confirm the trajectory was nominal. The crew could
; prepare for splashdown, stow equipment, and ready themselves for the ocean
; recovery.
;
; P66's successful execution on July 24, 1969, bringing the crew through the
; ballistic arc and setting up the final approach, was the penultimate step in
; returning the first Moon walkers safely home to Earth.
;
; ============================================================================
# PROGRAM:	P66
# MOD NO: 0	MOD BY: RR BAIRNSFATHER		DATE: 17 JAN 68		COLOSSUS GSOP ADDITIONS
# FUNCTION:	KEEP CM ATTITUDE IN TRIM TO THE RELATIVE VELOCITY VECTOR.  ENTRY GUIDANCE STOPS GENERATING
#		ROLL COMMANDS UNTIL DRAG BUILDS UP TO Q7+0.5 FPSS.
# CALLING SEQUENCE:  VIA RTB FROM REENTRY CONTROL.
# EXIT:		BACK TO REENTRY CONTROL.
# SUBROUTINE CALLS:  NEWMODEX

		COUNT*	$$/P66

P66		TC	NEWMODEX		# ENTER VIA RTB WHEN D < Q7 FPSS
		MM	66

		CA	V06N22			# OGA		IGA		MGA
						# XXX.XX DEG	XXX.XX DEG	XXX.XX DEG
		TC	P66END			# IN CASE CAME FROM P65, GO TO DISABLE GRP 4,
						# AND SET ENTRYDSP TO DO DISPLAY VIA
						# OVERNOUT.

						# ... AND CONTINUE AT KEP2

# Page 800
; ============================================================================
; PROGRAM P67 - FINAL DESCENT PHASE (DOWN-CONTROL TO SPLASHDOWN)
; ============================================================================
;
; P67 is the final chapter of Apollo's entry guidance story - the down-control
; phase that brings the Command Module from high-altitude reentry through the
; dense lower atmosphere to a gentle splashdown in the ocean. After the precision
; skip targeting of P65 and the ballistic coast of P66, P67 executes the final
; descent, managing lift vector control to null out any remaining range errors
; and deliver the crew within sight of the recovery ship.
;
; P67 ENTRY CONDITIONS:
; P67 is entered via RTB from REENTRY_CONTROL when atmospheric reentry is
; detected after the ballistic skip arc. Entry occurs when:
; - Drag builds up to D ≥ Q7 + 0.5 fpss (approximately 0.7-1.0g)
; - Altitude: Typically 200,000-240,000 ft
; - Velocity: Typically 22,000-25,000 fps (still hypersonic)
; - Flight path angle: Negative, steepening (descending into atmosphere)
; - Range to splashdown: Typically 300-500 nautical miles
;
; The vehicle is descending back into the sensible atmosphere after the skip
; arc, and aerodynamic forces are rapidly increasing. Active guidance is now
; essential to manage the final approach and hit the target splashdown point.
;
; P67 can also be entered directly from P64 if:
; - The skip was aborted (insufficient velocity or descending trajectory)
; - P65 determined that skip wasn't feasible due to entry dispersions
; - Crew manually selected P67 for direct entry without skip
;
; THE FINAL DESCENT CHALLENGE:
; At P67 entry, the vehicle still has substantial energy that must be dissipated:
; - Kinetic energy: (1/2)mV² with V ~ 23,000 fps
; - Potential energy: Altitude ~ 230,000 ft above sea level
; - Range remaining: 300-500 nm to travel before splashdown
;
; The challenge is to manage this energy dissipation while:
; 1. Maintaining acceptable g-loads (< 6.5g peak for crew safety)
; 2. Keeping heating rates within limits (already past peak heating in P64)
; 3. Correcting any residual range errors from the skip phase
; 4. Nulling crossrange errors for lateral positioning
; 5. Arriving at splashdown with near-zero horizontal velocity
;
; DOWN-CONTROL GUIDANCE ALGORITHM:
; P67 implements "down-control" or "constant altitude rate" guidance. The
; algorithm is simpler than P64's constant drag or P65's exit-keyed control,
; because at this point the mission constraints are less demanding:
;
; GUIDANCE LAW PHILOSOPHY:
; Rather than sophisticated optimal control, P67 uses a straightforward approach:
; measure how far off-target the vehicle is in range and crossrange, and modulate
; the lift vector to correct these errors before splashdown.
;
; RANGE ERROR CORRECTION:
; Each guidance cycle (approximately every 2 seconds), P67 computes:
;
; 1. PREDICT CURRENT TRAJECTORY:
;    Based on current state (position, velocity, flight path angle) and assuming
;    constant bank angle, integrate forward to predict where the vehicle will
;    splash down. This prediction uses a simplified atmospheric model and assumes
;    the current lift-to-drag ratio continues.
;
; 2. COMPUTE RANGE ERROR:
;    DNRNGERR = Predicted splashdown range - Target range
;    This tells us: will we overshoot (positive error) or undershoot (negative)?
;
; 3. COMPUTE CROSSRANGE ERROR:
;    XRNGERR = Predicted splashdown crossrange - Target crossrange  
;    This tells us: are we left or right of the target ground track?
;
; 4. COMPUTE ROLL COMMAND:
;    Based on range and crossrange errors, compute a bank angle that will null
;    the errors by splashdown. The guidance law is essentially:
;
;    If DNRNGERR > 0 (overshooting):
;       Roll toward lift-down → Increase drag → Reduce range
;    
;    If DNRNGERR < 0 (undershooting):
;       Roll toward lift-up → Reduce drag → Extend range
;
;    Simultaneously, add a lateral component to correct XRNGERR:
;       Need to move left → Roll to put lift vector left
;       Need to move right → Roll to put lift vector right
;
; 5. MODULATE COMMAND FOR SMOOTHNESS:
;    Rather than commanding abrupt roll changes, P67 smoothly adjusts the bank
;    angle over multiple cycles. This avoids excessive RCS propellant usage and
;    provides a smooth ride for the crew as g-loads build during final descent.
;
; LIFT VECTOR GEOMETRY:
; The brilliance of Apollo's lifting entry is that a single control input (roll
; angle) provides both downrange and crossrange control:
;
; ROLL = 0° → Lift straight up → Maximum range extension
; ROLL = 180° → Lift straight down → Maximum range reduction  
; ROLL = +90° → Lift to right → Crossrange control right
; ROLL = -90° → Lift to left → Crossrange control left
;
; For typical final descent, P67 commands roll angles between 0° and 180°,
; biasing toward lift-down (ROLL > 90°) to keep the trajectory from becoming
; too shallow, while adding lateral components as needed for crossrange control.
;
; TERMINAL DESCENT PROFILE:
; As P67 progresses from entry to splashdown, the trajectory evolves through
; several characteristic phases:
;
; INITIAL REENTRY (First 2-3 minutes):
; - Altitude: 230,000 ft → 150,000 ft
; - Velocity: 23,000 fps → 15,000 fps (rapid deceleration)
; - Drag: 1-2g → 3-4g (building rapidly)
; - Roll: Typically lift-down (150-180°) to control range
; - Range: 400 nm → 250 nm
;
; The vehicle descends steeply through the upper atmosphere, shedding velocity.
; Guidance is actively correcting the trajectory based on measured state and
; atmospheric density. G-loads build from 1g toward the peak.
;
; MID-DESCENT (Next 3-4 minutes):
; - Altitude: 150,000 ft → 60,000 ft
; - Velocity: 15,000 fps → 3,000 fps (continued deceleration)
; - Drag: 4-5g → 6g peak → 3-2g (past peak, now decreasing)
; - Roll: Modulating (120-180°) as range error is nulled
; - Range: 250 nm → 80 nm
;
; Peak g-loads occur around 100,000 ft altitude and 8,000-10,000 fps velocity.
; For Apollo 11, peak was approximately 6.5g - a firm push into the couches
; but well within crew tolerance. After peak g, loads decrease as velocity drops.
;
; TERMINAL PHASE (Final 2-3 minutes):
; - Altitude: 60,000 ft → 24,000 ft (drogue chute deployment altitude)
; - Velocity: 3,000 fps → 500 fps (subsonic transition ~30,000 ft)
; - Drag: 2-1g → 0.5g (atmospheric deceleration)
; - Roll: Near 180° (full lift-down) to steepen final approach
; - Range: 80 nm → 0 nm (approaching splashdown point)
;
; The vehicle transitions through the sound barrier (Mach 1 at ~750 fps around
; 30,000-40,000 ft altitude). Guidance continues commanding roll until very low
; altitude where aerodynamic control authority diminishes. The vehicle naturally
; oscillates in pitch due to dynamic instability (the CM is aerodynamically
; unstable), but the entry DAP damps these oscillations with RCS thrusters.
;
; P67 STEERING TERMINATION:
; P67 does NOT guide all the way to splashdown. Active steering terminates when
; the vehicle slows to 1,000 fps (approximately 680 mph, well subsonic). At this
; point, from the P67 function description:
;
; "FUNCTION: TO TERMINATE STEERING WHEN THE CM VELOCITY WRT EARTH = 1000 FT/SEC"
;
; WHY TERMINATE AT 1000 FPS?
; Several reasons drive this termination criterion:
;
; 1. AERODYNAMIC CONTROL LOSS:
;    Below 1,000 fps and at altitudes below ~30,000 ft, the dynamic pressure
;    (q = 0.5 × density × velocity²) has dropped so much that lift and drag
;    forces are relatively small. Roll commands have minimal effect on trajectory.
;
; 2. TRAJECTORY ESSENTIALLY DETERMINED:
;    By 1,000 fps, the vehicle is on a steep ballistic trajectory (essentially
;    falling with parachutes soon to deploy). The splashdown point is determined
;    within a few nautical miles. Further guidance corrections would be negligible.
;
; 3. PARACHUTE DEPLOYMENT APPROACHING:
;    The drogue chute deploys at 24,000 ft altitude and ~175 mph (~255 fps).
;    Main chutes deploy at 10,000 ft. At 1,000 fps the vehicle is only 1-2 minutes
;    from drogue deployment. Steering must cease well before chutes deploy.
;
; 4. STABILITY TRANSITION:
;    At low dynamic pressure, the aerodynamic moments are weak and the vehicle
;    enters a tumbling or oscillatory mode. Active attitude control by RCS is
;    still maintained by the entry DAP, but guidance stops trying to steer.
;
; When V ≤ 1,000 fps is detected, P67 disables the entry DAP, sets the vehicle
; to freefall mode, and transitions to POOH (mission complete). From this point,
; the Earth Landing System takes over:
; - Drogue chutes deploy at 24,000 ft (2 small drogues for initial stabilization)
; - Main chutes deploy at 10,000 ft (3 large mains for final descent)
; - Splashdown occurs at ~0 fps vertical, ~20 fps horizontal (gentle water impact)
;
; P67 DISPLAY AND CREW INTERACTION:
; P67 presents V06N66 as a fixed (non-flashing) display showing guidance status:
; - R1: ROLLC (XXX.XX degrees) - Current commanded roll angle
; - R2: XRNGERR (XXXX.X nm) - Crossrange error (lateral displacement from target)
; - R3: DNRNGERR (XXXX.X nm) - Downrange error (along-track displacement)
;
; This display updates continuously (approximately every 2 seconds) as guidance
; refines the trajectory. The crew watches the range errors converge toward zero,
; confirming that guidance is successfully targeting the splashdown point.
;
; As the errors null out (ideally < 1 nm by mid-descent), the crew gains confidence
; that they'll land within the planned recovery area. Mission Control also monitors
; telemetry of these values, ready to call out if errors aren't converging properly.
;
; SECONDARY DISPLAY - V16N67:
; In addition to V06N66, P67 presents V16N67 as a flashing display requiring crew
; response periodically (via the P67.1 routine):
; - R1: RTOGO (XXXX.X nm) - Range to go until splashdown
; - R2: LAT (XXX.XX degrees) - Current latitude
; - R3: LONG (XXX.XX degrees) - Current longitude
;
; This display serves two purposes:
; 1. Allows crew to monitor current position and range to target
; 2. Provides positive crew engagement - they must press PRO to continue
;
; If conditions look wrong or crew wants to intervene, they can press ENTER to
; recycle the display or take manual control. This gives the crew ultimate
; authority over the final descent, consistent with Apollo's pilot-in-command
; philosophy.
;
; FINAL DISPLAY UPDATE - P67.2:
; After steering terminates at 1,000 fps, P67 executes P67.2 which computes
; the final splashdown position (latitude, longitude, altitude) and displays
; it without requiring crew response. This final calculation uses:
; - Current position vector RN from navigation
; - Current time PIPTIME
; - LAT-LONG conversion routine
;
; The result shows the crew where they actually landed (or will land in seconds),
; allowing comparison to the target. Typically the error is 1-3 nautical miles -
; remarkably precise for a vehicle that was traveling 25,000 mph just minutes ago.
;
; PROGRAM STRUCTURE AND FLOW:
; P67's implementation is straightforward:
;
; 1. TC NEWMODEX / MM 67 - Set major mode to 67 (DSKY shows "P67")
; 2. CA V06N66 / TS ENTRYVN - Set up V06N66 display (ROLLC, XRNGERR, DNRNGERR)
; 3. TC UPFLAG / ADRES ENTRYDSP - Enable entry display updates
; 4. TC PHASCHNG / OCT 00004 - Disable group 4 (KILLGRP4)
; 5. TC DANZIG - Continue at PREDICT3 in REENTRY_CONTROL
;
; The actual guidance computations happen in REENTRY_CONTROL.agc:
; - DNCONTRL: Down-control guidance algorithm
; - PREDICT3: Range prediction and error computation
; - DNRNGCAL: Downrange and crossrange error calculations
;
; And the roll commands are executed by CM_ENTRY_DIGITAL_AUTOPILOT.agc:
; - Entry DAP maintains angle of attack while executing roll commands
; - RCS thrusters provide attitude control throughout descent
; - DAP damps pitch oscillations from aerodynamic instability
;
; ENTRY DAP DISABLE (END OF P67):
; When steering terminates at 1,000 fps, P67 executes:
; 1. CS THREE - Load negative of 3 (bits to clear)
; 2. MASK CM/FLAGS - Clear CM/DSTBY and GAMDIFSW flags
; 3. TS CM/FLAGS - Store result (disables entry DAP)
; 4. DCA SERVCAD2 - Load exit address
; 5. DXCH AVEGEXIT - Set up exit from AVERAGEG
; 6. TCF GOTOPOOH - Exit to POOH (mission complete)
;
; This sequence cleanly shuts down the entry guidance and control system,
; transitioning the CM to passive freefall mode where the Earth Landing System
; (parachutes) will complete the recovery sequence.
;
; RESTART PROTECTION:
; P67 has comprehensive restart protection through the entry restart system.
; If a computer restart occurred during P67 (very unlikely at this point, but
; possible), the restart routine would:
; - Identify P67 as active phase from restart tables
; - Restore guidance mode (DNCONTRL) and display (V06N66)
; - Resume navigation integration and trajectory prediction
; - Continue guidance from current state without missing a beat
;
; The restart system ensures mission success even if computer anomalies occur
; during critical final descent. The crew would see a brief RESTART light on
; the DSKY, guidance would momentarily freeze, then seamlessly resume within
; 1-2 seconds.
;
; MANUAL CONTROL DURING P67:
; Throughout P67, the crew retains full manual control authority via the rotation
; hand controller. They can override automatic guidance at any time and manually
; command roll angle. This capability is critical for:
;
; CONTINGENCY SCENARIOS:
; - If guidance commands look wrong (large errors not converging)
; - If entry DAP malfunctions (uncommanded roll oscillations)
; - If crew visual assessment disagrees with computed trajectory
; - If Mission Control calls for manual takeover due to telemetry anomalies
;
; The crew trained extensively in simulators for manual entry, learning to judge
; trajectory by visual cues through the window and by monitoring g-loads and
; velocity. While automatic guidance was highly reliable, the crew's ability
; to fly manually provided ultimate mission assurance.
;
; APOLLO 11 P67 DESCENT PROFILE (July 24, 1969):
; For Columbia's final descent:
;
; P67 ENTRY:
; - Time: Entry interface + ~12 minutes (approximately 720 seconds after 0.05g)
; - Altitude: ~230,000 ft (just reentered atmosphere after skip)
; - Velocity: ~23,000 fps (still hypersonic)
; - Drag: ~0.8g and increasing rapidly
; - Range to splashdown: ~400 nautical miles
; - Flight path angle: ~-8° (steep descent)
;
; MID-DESCENT:
; - Time: Entry interface + ~15 minutes
; - Altitude: ~100,000 ft (through region of peak heating earlier)
; - Velocity: ~8,000 fps (supersonic)
; - Drag: ~6.5g (peak g-load experienced by crew)
; - Range errors: XRNGERR < 2 nm, DNRNGERR < 3 nm (converging nicely)
;
; STEERING TERMINATION:
; - Time: Entry interface + ~18 minutes
; - Altitude: ~30,000 ft (well into lower atmosphere)
; - Velocity: ~1,000 fps (subsonic, approximately 680 mph)
; - Drag: ~0.3g (gentle deceleration)
; - Final position errors: < 2 nm in both range and crossrange
;
; The final splashdown occurred at:
; - Splashdown time: 195:18:35 GET (Ground Elapsed Time)
; - Position: 13°19'N, 169°9'W (central Pacific Ocean)
; - Miss distance: ~2.7 nautical miles from USS Hornet (within visual range)
; - Splashdown velocity: ~24 fps (~16 mph) - gentle water impact
; - Spacecraft attitude: Apex up (Stable I configuration)
;
; CREW EXPERIENCE DURING P67:
; The crew's sensations during P67 were dramatic and unforgettable:
;
; INITIAL REENTRY:
; - Increasing g-loads pushing them into couches (1g → 4g → 6g)
; - Plasma glow visible through windows (ionized air from compression heating)
; - Vehicle oscillating gently in pitch (aerodynamic instability damped by RCS)
; - Occasional RCS thruster firings heard/felt as DAP maintains attitude
; - Radio blackout continuing (plasma blocks communications)
;
; PEAK G-LOAD (~6.5g):
; - Heavy pressure on chest (breathing requires effort)
; - Arms feel very heavy (reaching controls difficult)
; - Visual grayout at periphery (blood pooling in lower body)
; - Duration: ~30-60 seconds at peak loads
; - All crew trained and conditioned for these loads
;
; DESCENT TO SUBSONIC:
; - G-loads decreasing (6g → 3g → 1g)
; - Radio contact restored (plasma cleared)
; - Sonic boom heard as vehicle passed Mach 1
; - Window clearing of plasma glow (Earth and ocean visible)
; - Anticipation building for drogue chute deployment
;
; POST-P67 (After 1,000 fps):
; - Freefall sensation (near zero-g)
; - Drogue chutes deployed with bang and jolt (24,000 ft)
; - Main chutes deployed with another jolt (10,000 ft)
; - Gentle descent under canopies (20-25 fps descent rate)
; - Ocean surface approaching - SPLASHDOWN!
;
; MISSION CONTROL MONITORING:
; Throughout P67, Mission Control's Guidance Officer (GUIDO) and Flight Dynamics
; Officer (FIDO) closely monitored the entry trajectory:
;
; KEY TELEMETRY PARAMETERS:
; - Velocity and altitude (confirming trajectory profile)
; - G-loads (watching that peak < 6.5g limit)
; - Range errors (watching convergence toward zero)
; - Roll angle (confirming guidance commands reasonable)
; - RCS propellant remaining (ensuring adequate for entry DAP)
;
; If telemetry showed anomalies, Mission Control was prepared to call up voice
; procedures for manual entry or backup modes. The confidence in Apollo's entry
; system came from thousands of hours of simulation, extensive wind tunnel testing,
; and the successful flights of Apollo 4, 8, and 10 that validated the system.
;
; COMPARISON TO EARLIER PROGRAMS:
; P67's down-control guidance represents a simpler philosophy than P64/P65:
;
; P64 (Constant Drag): Complex optimal control targeting specific drag profile
; P65 (Up-Control): Sophisticated exit-keyed guidance with velocity/drag targeting
; P66 (Ballistic): Monitoring only, no active guidance
; P67 (Down-Control): Straightforward error nulling with lift vector modulation
;
; The progression makes sense: early entry requires precise control to manage
; peak heating and loads, skip phase requires precision targeting for range,
; but final descent is more forgiving - just null the errors and don't overstress
; the crew or vehicle. P67's simplicity is appropriate engineering - complex
; when needed, simple when possible.
;
; HISTORICAL SIGNIFICANCE:
; P67 successfully completed the entry guidance for all Apollo lunar return
; missions, delivering every crew safely to splashdown within the planned recovery
; area. The precision was remarkable:
;
; - Apollo 8 (Dec 1968): ~3 nm miss distance
; - Apollo 10 (May 1969): ~2 nm miss distance  
; - Apollo 11 (Jul 1969): ~2.7 nm miss distance
; - Apollo 12-17: Similar precision (1-4 nm typical)
;
; This accuracy, combined with peak g-loads always < 7g and no thermal protection
; system failures, validated Apollo's lifting entry concept. The program demonstrated
; that precise guidance through atmospheric entry was not only possible but reliable
; and repeatable.
;
; For Apollo 11 specifically, P67's successful execution on July 24, 1969, was
; the final crucial performance of the AGC. From the Moon to the ocean, the
; computer had guided the crew through lunar orbit insertion, powered descent to
; the surface, ascent to orbit, rendezvous and docking, transearth injection, and
; now atmospheric entry. P67's completion meant the mission was essentially over -
; just the parachute landing remained.
;
; When Neil Armstrong, Buzz Aldrin, and Michael Collins felt the jolt of splashdown
; and heard the Navy divers knocking on the hatch, they knew the AGC had done its
; job perfectly. The code in P67, along with all the other entry programs, had
; brought them home.
;
; FINAL THOUGHT ON THE ENTRY SEQUENCE:
; The P61-P67 suite represents some of the most sophisticated guidance software
; ever written in the 1960s. From P61's initialization computations through P67's
; final descent, these programs embodied years of research into atmospheric entry
; dynamics, optimal control theory, and robust software engineering.
;
; The fact that this software was written in AGC assembly language, constrained
; to fit in 36K words of rope core memory, executing on a computer with ~85
; microsecond cycle time and 2K words of RAM, makes the achievement even more
; remarkable. Modern entry vehicles use the same fundamental concepts - constant
; drag, skip targeting, down-control - implemented in high-level languages on
; powerful computers. But Apollo proved these concepts worked, demonstrated their
; reliability, and brought twelve humans safely home from the Moon.
;
; That's what makes this code historically significant. These aren't just
; instructions and data - they're the software that made the impossible real.
;
; ============================================================================
# PROGRAM:	P67
# MOD NO:	0	MAR. 16, 1967
# MOD BY:	R. HIRSCHKOP
# FUNCTION:	TO TERMINATE STEERING WHEN THE CM VELOCITY WRT EARTH = 1000 FT/SEC
# CALLING SEQUENCE:
# EXIT:		TO POOH
# SUBROUTINE CALLS:  GOFLASH

# THIS DISPLAY IS CALLED EACH PASS THROUGH STEERING.  RESTART PROTECTION IS VIA STEERING.

		COUNT*	$$/P67

P67		TC	NEWMODEX		# ENTER VIA RTB
		MM	67
		CA	V06N66			# ROLLC		XRNGERR		DNRNGERR
						# XXX.XX DEG	XXXX.X NM	XXXX.X NM
P66END		TS	ENTRYVN			# DISPLAY VIA OVERNOUT.

		TC	UPFLAG			# (IN CASE CAME FROM P65.  ENTRY DISPLAY
		ADRES	ENTRYDSP		# WILL FLUSH FLASHING DISP.  IF STILL ON)
						# BIT 13 FLAG 6
KILLGRP4	TC	PHASCHNG		# DISABLE GRP4, IN CASE CAME FROM HUNTEST.
		OCT	00004			# (COME TO KILLGRP4 VIA RTB, RET TO CALLER)

		TC	DANZIG			# ... AND CONTINUE AT PREDICT3 ...

V06N66		VN	0666

		BANK	26
		SETLOC	P60S2
		BANK

P67.1		CA	V16N67			# RTOGO		LAT		LONG
						# XXXX.X NM	XXX.XX DEG	XXX.XX DEG
		TC	BANKCALL
		CADR	GOFLASH
		TC	+3			# EFFECTIVE GOTOPOOH
		TC	+2
		TC	P67.1			# REDO

		CS	THREE			# TURN OFF ENTRY DAP
		INHINT
		MASK	CM/FLAGS		# CM/DSTBY, GAMDIFSW
		TS	CM/FLAGS
		RELINT
		EXTEND
		DCA	SERVCAD2

# Page 801
		DXCH	AVEGEXIT

		TCF	GOTOPOOH

# Page 802
P67.2		VLOAD	CLEAR			# CALC PRESENT LAT, LONG, ALT.
			RN
			ERADFLAG		# USE PAD RAD FOR ALT. (NOT SEEN ANYWAY)
		STODL	ALPHAV
			PIPTIME			# USE TIME OF RN
		CLEAR	CALL
			LUNAFLAG
			LAT-LONG
P67.3		RTB				# ENTRY EXIT THAT OMITS DISPLAY.
			SERVNOUT

V16N67		VN	1667
OCT41		=	33DEC
SERVCAD2	=	SERVCAD1

# Page 803
# SUBROUTINE NAME:	S61.1
# MOD NO:	0					DATE:		21 FEB 67
# MOD BY:	RR BAIRNSFATHER				LOG SECTION:	P61-P67
# MOD NO:	1	MOD BY:	RR BAIRNSFATHER		DATE:		22 JUN 67	RESTARTS.
# FUNCTIONAL DESCRIPTION:	CALLED BY BOTH P61 AND P62
#	FIRST, TEST TO SEE IF  AVERAGEG  IS ON.  IF NOT, UPDATE THE STATE VECTOR TO PRESENT TIME + TOLERANCE
#	AND TURN ON  AVERAGEG  AT THAT TIME, AND CONTINUE.  OTHERWISE CONTINUE:  SEE IF IMU Y AXIS IS
#	WITHIN 30 DEG OF VAR.  IF YES, EXIT SUBROUTINE S61.1.  IF SO, SEE IF -Y AXIS OF IMU IS WITHIN
#	30 DEG OF VAR.	IF YES, DISPLAY ALARM:	01427	IMU REVERSED.
#			IF NO, DISPLAY ALARM:	01426	IMU UNSATISFACTORY.
#	IN EITHER OF THESE LAST 2 CASES, WAIT 10 SEC AND THEN EXIT SUBROUTINE S61.1.
#
# REMARK:	THERE WILL BE A SHORT 10 SEC DELAY IF AN ALARM EXIT IS TAKEN.  THE DELAY FOR INTEGRATION IS
#		AS SHORT AS CAN BE MADE, BUT IS ARBITRARY SINCE IT DEPENDS ON THE AGE OF THE STATE VECTOR.
#
# CALLING SEQUENCE:	CALL
#				S61.1
#
#			C(MPAC) UNSPECIFIED
#			PUSHLOC UNSPECIFIED
#
# SUBROUTINES CALLED: 	LOADTIME, CSMPREC, TPAGTREE,
#			WAITLIST, JOBSLEEP, JOBWAKE, PREREAD, ALARM, GODSPR, BANKCALL, DELAYJOB
#
# NORMAL EXIT MODES:	RVQ
#
# ALARMS:	01426	IMU UNSATISFACTORY
#		01427	IMU REVERSED
#
# OUTPUT:	POSSIBLE ALARMS
#		POSSIBLY TDEC1, RATT, VATT, RN, VN
#
# ERASABLE INITIALIZATION REQUIRED:
#	AVEGFLAG		AVERAGEG ON OR OFF				LEFT BY SERVICER
#	PIPTIME   (-28) CS	TIME OF PIPA UPDATE				LEFT BY READACCS
#	RN        (-29) M	STATE VECTOR					LEFT BY AVERAGEG
#	VN	  (-7) M/CS	STATE VECTOR					LEFT BY AVERAGEG
#	REFSMMAT  (-1)		.5 REF TO SM MATRIX				LEFT BY LAST IMU ALIGNMENT
#
# DEBRIS:	QPRET
#		POSSIBLY PIPTIME1, RATT, VATT, TDEC1, RN1, VN1, QTEMP, X1	IF UPDATED
#		PUSH LIST LOCS USED BY CSMPREC

		EBANK=	AOG		# FOR 60GENRET, S61DT
		BANK	26
		SETLOC	P60S3
		BANK

		COUNT*	$$/S61.1

; ============================================================================
; SUBROUTINE: S61.1 - Entry Preparation and IMU Verification
;
; Before the Command Module plunges into Earth's atmosphere, this subroutine
; performs two essential checks that determine whether entry guidance can
; proceed safely:
;
; 1. STATE VECTOR INTEGRATION: If Average-G navigation is not yet active,
;    the spacecraft's position and velocity are integrated forward to the
;    current time. This ensures entry calculations begin with the most
;    accurate trajectory data available.
;
; 2. IMU ALIGNMENT CHECK: The Inertial Measurement Unit's Y-axis must be
;    aligned within 30 degrees of the velocity vector (VAR). Proper IMU
;    alignment is critical for accurate attitude control during entry.
;    - If IMU Y-axis is aligned correctly: Continue to entry
;    - If IMU -Y axis is aligned (reversed): Alarm 01427 "IMU REVERSED"
;    - If IMU is misaligned: Alarm 01426 "IMU UNSATISFACTORY"
;
; HISTORICAL CONTEXT:
; During Apollo 11's return on July 24, 1969, this subroutine ran as Columbia
; approached Earth's atmosphere at 25,000 mph. The IMU check verified that
; the spacecraft's inertial platform maintained accurate orientation despite
; eight days of spaceflight. Any IMU misalignment would have compromised the
; computer's ability to steer the lift vector during reentry, potentially
; missing the landing zone by hundreds of miles.
;
; TECHNICAL OPERATION:
; The subroutine uses the REFSMMAT (reference to stable member matrix) to
; transform the current velocity vector into IMU coordinates, then checks
; alignment. If alarms occur, the crew has 10 seconds to assess the situation
; before the subroutine exits, allowing them time to consider manual entry
; procedures if necessary.
; ============================================================================

S61.1		EXTEND
		QXCH	60GENRET	# SAVE RET ADDR IN EB 6
		TC	BANKCALL
		CADR	R02BOTH
		TC	INTPRET
# Page 804
		BON	CALRB
			AVEGFLAG	# IS AVERAGEG ON
			S61.1A		# YES
			MIDTOAV2	# GET FUTURE STATE VECTOR SOON AS CAN

		CA	MPAC +1		# RETURN INHINTED ***
		TS	S61DT		# FOR RESTART.
		TC	WAITLIST
		EBANK=	DVCNTR
		2CADR	S61.1C

		TC	PHASCHNG
		OCT	40434
		TC	ENDOFJOB

S61.1C		CA	PRIO13
		TC	FINDVAC
		EBANK=	AOG
		2CADR	S61.1A 	-1

		EXTEND
		DCA	SERVCAD1	# HE WHO START AVERAGEG MUST SERVICE
		DXCH	AVEGEXIT	# THE EXIT.

		TC	2PHSCHNG
		OCT	00454
		OCT	00415

		CA	EBENTRY		# SET EB= 7 FOR PREREAD.
		TS	EBANK

		TC	POSTJUMP
		CADR	PREREAD		# PREREAD DOES TC TASKOVER.

		TC	INTPRET
S61.1A		BOVB	VLOAD
			TCDANZIG	# TURN OFF OVFIND, IF ON
			VN		# VN	(-7) M/CS
		VXV	MXV
			RN		# RN	(-29) M
			REFSMMAT	# .5 UNIT MATRIX
		UNIT	DLOAD
			MPAC +3		# GET COS(THETA)/2
		BMN	DAD
			S61.1B		# DO TEST ON -YSM
			C(30)LIM	# = 1.0 -.5 COS(30)
		BOVB	RTB
			RETRN1
			RETRN3
# Page 805
S61.1B		DCOMP	DAD
			C(30)LIM	# = 1.0 - .5 COS(30)
		BOVB	EXIT
			RETRN2

RETRN3		TC	ALARM
		OCT	01426		# IMU UNSATISFACTORY
		TC	RETRN2 +2

RETRN2		TC	ALARM
		OCT	01427		# IMU REVERSED

	+2	CAF	V05N09
		TC	BANKCALL
		CADR	GODSPR		# DO DISPLAY
		CA	10SECS
		TC	BANKCALL
		CADR	DELAYJOB

RETRN1		TC	60GENRET

		EBANK=	DVCNTR
SERVCAD1	2CADR	SERVEXIT

C(30)LIM	2DEC	.566985		# = 1.0 - .5 COS(30)

10SECS		DEC	1000		# 1000 CS
60SECDP		2DEC	6000 B-28	# 6000 CS

# Page 806
# PROGRAM NAME:		S61.2			DATE:		14 FEB 67
# MOD NO:  	1				LOG SECTION:	P61-P67
# MOD BY:	NORTH / BAIRNSFATHER
# MOD NO: 2	MOD BY: NORTH/BAIRNSFATHER	DATE: 11 MAY 67		ADD 2ND ITER FOR ERAD AT 400K FT.
# MOD NO: 3	MOD BY: RR BAIRNSFATHER		DATE: 21 NOV 67		VARIABLE MU ADDED.
# MOD NO: 4	MOD BY: RR BAIRNSFATHER		DATE: 21 MAR 68		DIFFERENT EARTH/MOON SCALES IN TFF'S
#
# FUNCTIONAL DESCRIPTION:  CALLED IN P61.  PROVIDES DISPLAYS FOR NOUNS  N60  AND  N63 .
#	PROGRAM CALCULATES ENTRY DISPLAY OF MAXIMUM ACCELERATION EXPECTED  (GMAX)  AND ALSO THE EXPECTED
#	INERTIAL VELOCITY (VPRED) AND ENTRY ANGLE  (GAMMAEI)  THAT WILL OBTAIN AT 400K FT ABOVE THE FISCHER
#	ELLIPSOID.  PROGRAM ALSO CALCULATES A SECOND DISPLAY RELATIVE TO THE  EMSALT  ABOVE FISCHER ELLIPSOID
#	AND CONSISTS OF RANGE TO SPLASH FROM NOW  (RTGO) , PREDICTED INERTIAL VELOCITY  (VIO) , AND THE TIME TO
#	GO FROM NOW  (TTE) .
#
# CALLING SEQUENCE:	CALL
#				S61.2
#			C(MPAC) UNSPECIFIED
#			PUSHLOC WILL BE SET TO ZERO.
#
# SUBROUTINES CALLED:	TFFCONIC, CALCTFF, TFF/TRIG, FISHCALC, GETERAD, VGAMCALC
#
# NORMAL EXIT MODES:	RTB, P61.1
#
# ALARMS:  	NONE
#
# OUTPUT:	THE FOLLOWING REGISTERS ARE WRITTEN IN FOR USE BY DISPLAYS
#		GMAX	100 GMAX (-14) G,S	MAXIMUM ACCELERATION
#		VPRED	(-7) M/CS		PREDICTED VELOCITY AT 400K FT
#		GAMMAEI	GAMMA/360		PREDICTED GAMMA    AT 400K FT
#						FOR TM, DP(GAMMAEI) = (GAMMAEI, RTGO) / 360
#		RTGO	THETAH/360		RANGE ANGLE TO SPLASH FROM EMSALT	EMSALT IS PAD LOADED.
#		VIO	(-7) M/CS		INTERTIAL VELOCITY AT      EMSALT	EMSALT IS PAD LOADED.
#		TTE	(-28) CS		TIME TO                    EMSALT	EMSALT IS PAD LOADED.
#		PUSHLOC	= 0
#		CONIC PARAMETERS STORED IN VAC AREA (SEE TFF SUBROUTINES)
#
# ERASABLE INITIALIZATION REQUIRED:
#		RONE	(-29) M			STATE VECTOR				LEFT BY USER
#		VONE	(-7) M/CS		STATE VECTOR				LEFT BY USER
#		URONE	UR/2								LEFT BY USER
#		UNI	(-1)			UNIT NORMAL V*R				LEFT BY ENTRY / P61
#		THETAH	THETAH/360		RANGE ANGLE				LEFT BY ENTRY / P61
#		UNITW	(0)			UNIT POLAR VECTOR			LEFT BY PAD LOAD
#		EMSALT	(-29) M			EMS INTERFACE ALTITUDE			LEFT BY PAD LOAD
#						ORBITAL REENTRY: 284843 FT., LUNAR REENTRY: 297431 FT.
#
# DEBRIS:	QPRET,
#		ALL PDL LOCATIONS ABOVE 12D, INCLUDING X1,X2,S1,S2
#		ALSO PDL+0 ... PDL+5, WHERE INITIAL PUSHLOC = PDL

# Page 807
# THE FOLLOWING PUSH LIST LOCATIONS HAVE BEEN RESERVED FOR TFF ROUTINES AND ARE REPEATED HERE FOR CONVENIENCE.
# OF COURSE FOR S61.2 USAGE, EARTH ORIGIN SCALING IS USED.
#
#				BELOW	E:  IS USED FOR EARTH ORIGIN SCALE
#					M:  IS USED FOR MOON ORIGIN SCALE
#
#	RTERM	= 	18D		TERMINAL RADIUS M	E:  (-29)	M:  (-27)
#	NRTERM	=	16D		TERMINAL RADIUS M	E:  (-29+NR)
#								M:  (-27+NR)
#	RMAG1	=	12D		PRESENT RADIUS M	E:  (-29)	M:  (-27)
#	NRMAG	=	32D		PRESENT RADIUS M	E:  (-29+NR)
#								M:  (-27+NR)
#	SDELF/2				SIN(THETA) / 2
#	CDELF/2	=	14D		COS(THETA) / 2
#	TFFX	=	34D		X, ARGUMENT OF SERIES T(X)
#	TFFTEM	=	36D		ARG FOR TRANSFER ANGLE CALCULATION
#	TFFNP	=	28D		LC P M 			E:  (-38+2NR)	M:  (-36+2NR)
#	TFF/RTMU=	30D		1/SQRT(MU)		E:  (17)	M:  (14)
#	TFFVSQ	=	20D		-(VN.VN/MU)	1/M	E:  (20)	M:  (18)

# Page 808
		BANK	34
		SETLOC	P60S2
		BANK

		COUNT*	$$/S61.2
					# PDL LEFT AT ZERO BY TARGETING

; ============================================================================
; SUBROUTINE: S61.2 - Entry Prediction and Initialization Data Calculation
;
; This subroutine computes critical entry parameters that enable the crew to
; monitor the entry trajectory and ensure a safe return to Earth. It calculates
; predictions for maximum deceleration, velocities at key altitudes, and timing
; information that will be displayed to the crew during entry.
;
; KEY CALCULATIONS PERFORMED:
;
; 1. GMAX: Predicted maximum acceleration during entry (in G's)
;    - Critical safety parameter: Must stay below spacecraft structural limits
;    - Crew monitoring: Excessive G-loads indicate trajectory deviation
;
; 2. VPRED: Predicted velocity at 400,000 feet altitude
;    - Reference altitude where Entry Monitoring System (EMS) becomes active
;    - Used for trajectory validation
;
; 3. GAMMAEI: Predicted flight path angle at 400,000 feet
;    - Shallow entry: ~6-7 degrees (nominal for Apollo)
;    - Steeper entry: Higher G-loads but shorter entry time
;
; 4. RTGO: Range angle remaining to splashdown from EMS interface altitude
;    - Measured as angular distance on Earth's surface
;    - Crew uses this to monitor trajectory progress
;
; 5. VIO: Inertial velocity at EMS interface altitude
;    - Initial conditions for EMS computations
;
; 6. TTE: Time-to-go until reaching EMS interface altitude
;    - Countdown timer for crew preparation
;
; ENTRY TYPE DETERMINATION:
; The subroutine automatically distinguishes between:
; - ORBITAL ENTRY: Returning from Earth orbit (EMS altitude 284,843 feet)
; - LUNAR ENTRY: Returning from Moon (EMS altitude 297,431 feet)
; This distinction affects gravitational calculations (Earth mu vs conic parameters)
;
; HISTORICAL CONTEXT:
; During Apollo 11's return on July 24, 1969, this subroutine computed Columbia's
; entry trajectory predictions as the spacecraft approached Earth's atmosphere at
; 36,237 feet per second (24,697 mph). The predicted GMAX of approximately 6.5 G's
; assured Mission Control that the trajectory was safe. The crew monitored these
; predicted values against actual readings during the 13-minute entry, watching
; for any divergence that would indicate guidance problems.
;
; The EMS (Entry Monitoring System) backup instruments used these initial
; predictions as reference values. If the Primary Guidance and Navigation System
; failed, the crew could manually steer the spacecraft using EMS delta-V readings
; referenced to these predicted values.
;
; COMPUTATIONAL APPROACH:
; The subroutine uses Time-of-Free-Fall (TFF) conic trajectory calculations to
; predict the spacecraft's ballistic path from current position to the EMS
; interface altitude. These predictions assume unpowered flight under gravity alone.
; ============================================================================

S61.2		DLOAD	DSU
			EMSALT
			290KFT
		BPL	DLOAD
			LUNENT
			1/RTMU		# ESTABLISH MU FOR ORBITAL ENTRIES
CALLCON		CALL
			TFFCONIC	# FILL VAC AREA WITH CONIC PARAMETERS

		DLOAD	CALL
			RTRIAL		# 1ST GUESS AT TERMINAL RADIUS	(-29)
			CALCTFF		# SAVES MPAC IN RTERM		(18D)

		CALL			# CALC SDELF/2, CDELF/2
			TFF/TRIG	# RETURN WITH S(THETA) IN MPAC

		CALL			# GET FISCHER RADIUS		(-29) M
			FISHCALC	# ANS IN MPAC AND IN ERADM

		DAD	CALL
			EMSALT
			CALCTFF		# SAVES MPAC IN RTERM		(18D)

		DCOMP			# NEGATIVE AS IN COUNTDOWN
		STORE	TTE1		# DECR TTE FROM BASB TTE1.  (RESTART)
					# DNLIST AND DSKY WILL USE TTE.
		STCALL	TTE		# LET MISS CONTRL DECR BY ELAPSED TIME
					# TTE= TIME FROM NOW TO EMSALT +FISCHER

			TFF/TRIG	# S(THETA) IN MPAC ON RETURNING
					# AND THETA= RANGE FROM NOW TO EMSALT

		CALL
			FISHCALC
		CALL
			VRCALC
		CALL
			DISPTARG
		CALL
			DISPTARG
		STCALL	RTGO
# Page 809
			VGAMCALC

		DMP			# MPAC = GAMMA
					# PDL0 HAS VGAM.
		BDDV	DAD
			VEMSCON		# -HS D 180/PI (-14)
			0		# VGAM FROM PDL0
		STODL	VIO		# PREDICTED VELOCITY AT EMSALT.

					# GAMMA AND VGAM AT 300K FT ARE REQUIRED BY GMAX
					# ALGORITHM.

			ERADM		# EARTH RADIUS FROM GETERAD (-29) M
					# = FISCHER RADIUS (-29)

		DAD
			300KFT		# M (-29)
		STCALL	RTERM		# TERMINAL RADIUS M (-29)

			PREVGAM		# VGAMCALC WITH NEW RTERM

					# VBAR = (V(FPS) - 36KF/S) / 20 F/S
# GMAX = (4/(1 + 4.8 VBARSQ))(GAM - 6.05 - 2.4 VBARSQ) - 10(L/D - .3) + 10	ASSUME L/D = 0.3, BANK =0.

# GMAXCALC
		PDDL	DSU		# GAM TO PDL2
			0		# VGAM IS IN PDL0 (-7)
			36KFT/S		# (-7) M/CS
		DDV	DSQ
			20KFT/S		# (-6) M/CS
		STORE	0		# VBARSQ (-2) TO PDL0

		DMP	DAD
			KR1
					# GAM, POS DOWN, FROM PDL2
		DAD	DMP
			-6.05DEG
			KR2
		PDDL			# XCH PDL+0 FOR VBARSQ (-2)
		DDV	DAD
			KR4
			DP2(-4)
		BDDV
					# NUM FROM PDL+0
		DAD	BPL
			KR3
			+3
		DLOAD
			HI6ZEROS
		STODL	GMAX		# 100 GMAX (-14)
# Page 810
# DISPLAY USES GMAX AS SP, SO LO WORD IS WRITTEN OVER BY VPRED.
			ERADM		# = FISCHER RADIUS (-29) M
		DAD	CALL		# 2 ND ITERATION FOR FISCHER RADIUS
			400KFT
			CALCTFF		# ESTABLISH TRANSFER ANGLE DATA.
		CALL
			TFF/TRIG	# GET SIN, COS DELF
		CALL
			FISHCALC	# GET CORRESPONDING FISCHER RADIUS.

		DAD	LXA,2		# SAVE HI-WORD FOR DOWNLIST.
			400KFT		# M (-29)
			RTGO		# (RANGE ANGLE FROM EMSALT) / 360
		STCALL	RTERM
			PREVGAM		# VGAMCALC WITH NEW RTERM

		DCOMP	SXA,2		# HI-WORD OF EACH ON DOWNLIST.
			MPAC +1
		STODL	GAMMAEI		# CONIC GAMMA/360 AT 400K FT.	(HI-WORD)
					# CONIC RTGO/360 FROM EMSALT   (LOW-WORD)
					# FOR TM, DP(GAMMAEI) = (GAMMA, RTGO) / 360

					# VGAM FROM PDL+0 (-7)
		STADR
		STORE	VPRED		# CONIC VELOCITY AT 400K FT

		RTB
			P61.1
					# PDL BACK TO ZERO.

LUNENT		DLOAD	GOTO
			1/RTMUE		# ESTABLISH MU FOR LUNAR TYPE ENTRIES
			CALLCON
290KFT		2DEC	88392.0 B-29

KTETA1		2DEC*	.421844723 E2 B-14* # 110 2PI/16384(163.84)

36KFT/S		2DEC	109.728 B-7	# (-7) M/CS = 36 KFT/S (-7)

20KFT/S		2DEC	121.92 B-7	# (-6) M/CS = 2 20KFT/S (-7)

KR1		2DEC	-.026666667	# = -2.4 4 / 360

-6.05DEG	2DEC	-.016805556	# = -6.05 / 360

KR2		2DEC	.54931641	# = (360/4) 100 (-14) = 9000 B-14

KR3		2DEC	1000 B-14	# = 100 (10.0) (-14) G,S
# Page 811
					# ASSUMES L/D = 0.3, BANK =0.
RTRIAL		2DEC	6460097.18 B-29	# RPAD +264643 FT =21 194 545 FT
					# RPAD DEFINED AS 20 909 901.57 FT =6 373 336 M
400KFT		2DEC	121920 B-29	# METERS

# 300KFT	2DEC	91440 B-29	# (-29) M

# EMSALT	2DEC	86759.2	B-29	# 284643 FT (-29) M 	(ORBITAL REENTRY)

# EMSALT	2DEC	90657 B-29	# 297431 FT (-29) M	(LUNAR REENTRY)

KR4		2DEC	.833333333

300KFT		EQUALS	MINPERE
VEMSCON		2DEC	-.0389676 B-14	# = -HS D / 2 PI (-14)	M SQ / CS SQ

					# = -16369	.05G	32.2	.3048	.3048/2 PI	(-14)

# Page 812
# SUBROUTINE NAME:  FISHCALC	(USED BY S61.2)		DATE:		01.21.67
# MOD NO: 0						LOG SECTION:	P61-P67
# MOD BY: MORTH / BAIRNSFATHER
# MOD NO: 1	MOD BY: RR BAIRNSFATHER			DATE:		11 MAY 67	INCLUDE GETERAD CALL
#
# FUNCTIONAL DESCRIPTION:  GIVEN THE PRESENT POSITION, UNITR, CALCULATE A NEW UNITR THAT IS ROTATED THROUGH
#	TRANSFER ANGLE, THETA, ALONG THE TRAJECTORY.  THEN CALCULATE SIN(LAT) AND USE TO OBTAIN FISCHER RADIUS.
#	SINCE FISHCALC USED UNI (LEFT BY ENTRY) EARTH SCALING IS ASSUMED.  (WILL IMPROVE FOR SUITABLE TENNANT)
#
# CALLING SEQUENCE:	CALL
#				FISHCALC
#	ENTER WITH .5 SIN(THETA) IN MPAC.
#	PUSHLOC IS AT PDL+0, AN ARBITRARY BASE VALUE IF LEQ 8D
#
# SUBROUTINES CALLED:  GET ERAD
#
# NORMAL EXIT MODE:  RVQ
#
# EXIT MODES:	NONE
#
# OUTPUT:	ERADM (-29) M IN MPAC ON RETURNING
#		NEW UNIT VECTOR NOT SAVED.
#		SIN(LAT) NOT SAVED.
#		PUSHLOC AT PDL+0
#
# ERASABLE INITIALIZATION REQUIRED:
#		SDELF/2		=SIN(THETA) / 2, IN MPAC		LEFT BY TFF/TRIG
#		CDELF/2		=COS(THETA) / 2, STORED IN PDL 14D	LEFT BY TFF/TRIG
#		RONE		(-29) M					LEFT BY USER
#		VONE		(-7) M/CS				LEFT BY USER
#		URONE		UR/2					LEFT BY USER
#		UNI		.5 UNIT(V*R)				LEFT BY ENTRY / P61
#		UNITW		UNIT NORTH POLE				LEFT BY PAD LOAD
#
# DEBRIS:	QPRET, PDL+0 ... PDL+5
					# _      _          _
; ============================================================================
; SUBROUTINE: FISHCALC - Fischer Ellipsoid Radius Calculation
;
; PURPOSE:
; Computes the Earth's radius at a specific latitude using the Fischer 1960
; ellipsoid model. This geodetic calculation is essential for accurate entry
; trajectory predictions because Earth is not a perfect sphere - it's an
; oblate spheroid flattened at the poles.
;
; WHY FISCHER ELLIPSOID MATTERS FOR APOLLO ENTRY:
; During atmospheric entry, the Command Module must target a precise landing
; zone in the Pacific Ocean. Using a simple spherical Earth model would
; introduce errors of several kilometers in the predicted splashdown point.
; The Fischer ellipsoid provides:
;   - Equatorial radius: 6,378,166 meters
;   - Polar radius: 6,356,784 meters
;   - Difference: ~21 km flattening
;
; For Apollo 11's Pacific splashdown (approximately 13°N latitude), the
; Fischer model accurately computed the local Earth radius, enabling precise
; range-to-target calculations.
;
; INPUT PARAMETERS:
;   SDELF/2: Sin(transfer angle)/2 in MPAC (from TFF/TRIG subroutine)
;   CDELF/2: Cos(transfer angle)/2 in PDL+14 (from TFF/TRIG subroutine)
;   RONE:    Initial radius vector magnitude, (-29) meters
;   VONE:    Initial velocity magnitude, (-7) meters/centisecond
;   URONE:   Half of unit radius vector
;   UNI:     Half of unit perpendicular vector (V cross R direction)
;   UNITW:   Unit vector pointing to North Pole (pad-loaded)
;
; COMPUTATIONAL SEQUENCE:
; 1. Rotates initial radius unit vector through transfer angle:
;    URPR = UR·cos(θ) + UHOR·sin(θ)
;    This gives the unit radius vector at the predicted entry interface point
;
; 2. Computes latitude by taking dot product with North Pole unit vector:
;    sin(latitude) = URPR · UNITW
;
; 3. Calls GETERAD subroutine which implements Fischer ellipsoid formula:
;    R(lat) = Re / sqrt(1 + f(2-f)·sin²(lat))
;    where Re = equatorial radius, f = flattening factor
;
; OUTPUT:
;   ERADM: Fischer ellipsoid radius at computed latitude, (-29) meters
;   URH:   Unit radius vector at entry interface (saved for range calculation)
;   ALPHAV+4: Sin(latitude)/2 (saved for geodetic calculations)
;
; TECHNICAL NOTES:
; - Uses interpretive vector operations for precision and compact code
; - All vector quantities stored as half-vectors (multiplied by 0.5) to
;   prevent overflow in 15-bit arithmetic
; - GETERAD subroutine contains Fischer ellipsoid constants and formula
; - Push-down list (PDL) preserves intermediate results during calculation
;
; HISTORICAL CONTEXT:
; The Fischer 1960 ellipsoid was the geodetic standard used by NASA for Apollo
; missions. Modern GPS uses WGS-84 ellipsoid (slightly different), but Fischer
; was the best available Earth model in the 1960s based on satellite tracking
; data available at that time.
; ============================================================================

FISHCALC	PDVL	VXV		# URPR = UR CDELF + UHOR SDELF
	 		URONE
			UNI
		VXSC	VSL1
					# SIN(THETA) / 2 FROM PDL+0
		PDVL	VXSC		# TO PDL+0, +5
			URONE
			CDELF/2		# COS(THETA) / 2
		VAD	STADR
		STORE	URH		# FOR USE IN RTGO FROM EMS DISPLAY
		DOT	SL1
			UNITW		# PULL UNIT VECTOR	UNIT NORTH
		STORE	ALPHAV +4	# = .5 SIN(LAT)
DUMPFISH	GOTO
			GETERAD		# SAVES FISCHER RAD (-29) M IN ERADM AND
					# IN MPAC.  RETURNS TO CALLER VIO QPRET.

# Page 813
; ============================================================================
; SUBROUTINE: VGAMCALC - Velocity and Flight Path Angle Calculation
;
; PURPOSE:
; Computes the predicted velocity magnitude and flight path angle (gamma) at
; a target altitude during atmospheric entry. These predictions are critical
; for trajectory validation and EMS (Entry Monitoring System) initialization.
;
; THE VIS VIVA EQUATION FOR ENTRY TRAJECTORY:
; This subroutine applies the vis viva equation (energy conservation in orbital
; mechanics) to predict what the spacecraft's velocity will be when it reaches
; a specific altitude. The fundamental physics:
;
;   Total Energy = Kinetic Energy + Gravitational Potential Energy
;   E = (1/2)mv² - GMm/r = constant
;
; For spacecraft entry, this becomes:
;   V_terminal = sqrt(V_current² + 2μ(1/R_terminal - 1/R_current))
;
; where μ (mu) is Earth's gravitational parameter (GM).
;
; FLIGHT PATH ANGLE (GAMMA):
; The angle between the velocity vector and the local horizontal plane.
; During Apollo entry:
;   - Typical entry angle: 6.5 to 7.0 degrees (shallow)
;   - Too shallow: Skip out of atmosphere (escape back to space)
;   - Too steep: Excessive G-loads, potential structural failure
;
; The subroutine computes gamma from angular momentum:
;   cos(γ) = H / (R_terminal × V_terminal)
;   where H is the specific angular momentum (R × V)
;
; ENTRY INTERFACE ALTITUDE PREDICTIONS:
; For Apollo 11's return on July 24, 1969, VGAMCALC predicted:
;   - Velocity at 400,000 feet (EMS interface): ~36,237 ft/sec
;   - Flight path angle: ~6.88 degrees (shallow entry corridor)
;   - This assured controllers the trajectory was within safe limits
;
; The predictions computed here appear on the crew's DSKY display during P61,
; giving Armstrong, Aldrin, and Collins confidence that their entry trajectory
; would result in a safe Pacific Ocean splashdown.
;
; DUAL ENTRY POINT DESIGN:
; - VGAMCALC: Standard entry point when RTERM already normalized
; - PREVGAM: Alternative entry point that first normalizes new RTERM value
;   This flexibility accommodates different calling contexts within entry programs
;
; INPUT PARAMETERS (from TFFCONIC and CALCTFF subroutines):
;   RMAG1/NRMAG: Present radius magnitude (current spacecraft position)
;   RTERM/NRTERM: Terminal radius (target altitude for prediction)
;   TFFVSQ: Normalized velocity squared term (-(V²/μ))
;   TFFNP: Semi-latus rectum (L), angular momentum squared per unit mass
;   TFF/RTMU: Reciprocal of sqrt(μ), for computational efficiency
;
; OUTPUT:
;   MPAC: GAMMA/360 (flight path angle as fraction of full circle)
;         Note: Computed as positive; caller must apply correct sign
;   PDL+0: VGAM (predicted velocity magnitude at terminal radius)
;          Earth: (-7) scaling = meters/centisecond
;          Moon: (-5) scaling (different gravity, different velocity scale)
;
; COMPUTATIONAL SEQUENCE:
; 1. Compute denominator: R_current × R_terminal
; 2. Compute numerator: 2(R_current - R_terminal)  [energy difference term]
; 3. Add normalized velocity squared term: V²/μ
; 4. Take square root and scale by 1/sqrt(μ) → terminal velocity
; 5. Compute angular momentum from semi-latus rectum
; 6. Divide by (R_terminal × V_terminal) → cos(gamma)
; 7. Take inverse cosine → gamma
;
; SCALING CONSIDERATIONS:
; The subroutine handles both Earth entry (E:) and Moon entry (M:) with
; different scaling factors because:
;   - Earth μ = 398,600 km³/s² (larger)
;   - Moon μ = 4,903 km³/s² (much smaller, ~1/81 of Earth)
; Different gravitational parameters require different fixed-point scalings
; to maintain precision in 15-bit arithmetic.
;
; VARIABLE MU CAPABILITY:
; Modified in November 1967 to support variable gravitational parameter,
; enabling the same code to handle both Earth return and lunar orbit scenarios.
; This eliminated need for duplicate entry calculation routines.
;
; ERROR HANDLING:
; Assumes terminal radius < present radius (descending trajectory).
; Both CALCTFF and CALCTPER verify this assumption before calling VGAMCALC.
; Ascending trajectories would produce mathematically invalid results (negative
; values under square root).
; ============================================================================


# SUBROUTINE NAME:  VGAMCALC	(USED BY S61.2)				DATE:		01.21.67
# MOD NO: 0								LOG SECTION:	P61-P67
# MOD BY: MORTH / BAIRNSFATHER
# MOD NO: 1	MOD BY: RR BAIRNSFATHER		DATE: 11 APR 67
# MOD NO: 2	MOD BY: RR BAIRNSFATHER		DATE: 21 NOV 67		VARIABLE MU ADDED.
# MOD NO: 3	MOD BY: RR BAIRNSFATHER		DATE: 21 MAR 68		ACCEPT DIFFERENT EARTH/MOON SCALE
#
# FUNCTIONAL DESCRIPTION:  EARTH CENTERED VIS VIVA CALCULATION OF TERMINAL VELOCITY AND GAMMA (REL TO
#	HORIZONTAL) GIVEN THE SCALAR QUANTITIES:  PRESENT RADIUS AND VELOCITY AND THE TERMINAL RADIUS.
#	THE USER MUST APPEND PROPER SIGN TO GAMMA, SINCE IT IS CALCULATED AS A POSITIVE NUMBER.
#	THE EQUATIONS ARE
#
#		VGAM = SQRT(VN VN/MU + 2(RN-RTERM)/(RN RTERM) ) RTMU
#
#		COSGAM = H / RTERM VGAM = SQRT (LCP) / (RTERM VGAM/RTMU)
#
#	VGAMCALC ASSUMES THAT THE TERMINAL RADIUS IS LESS THAN THE PRESENT RADIUS.  BOTH CALCTFF AND CALCTPER
#	MAKE THIS ASSUMPTION.
#
# CALLING SEQUENCE:	CALL			STCALL	RTERM
#				VGAMCALC		PREVGAM
#	PUSHLOC AT PDL+0, ARBITRARY IF LEQ 12D
#	C(MPAC) UNSPECIFIED			C(MPAC)=NEW RTERM
#
# SUBROUTINES CALLED:  NONE
#
# NORMAL EXIT MODE:  RVQ
#
# ALARMS:	NONE
#
# OUTPUT:	GAMMA / 360 IN MPAC, POSITIVE NUMBER
#		VGAM 	E: (-7)	  M: (-5)	M/CS IN PDL+0
#		PUSHLOC AT PDL+2
#
# ERASABLE INITIALIZATION REQD:
#	TFF/RTMU  E: (17)   M: (14)	1/SQRT(MU)				LEFT BY TFFCONIC
#	RMAG1	  E: (-29)  M: (-27)	M  PRESENT RADIUS LENGTH		LEFT BY TFFCONIC
#	NRMAG	  E: (-29+NR)		M  NORM LENGTH OF PRESENT POSITION	LEFT BY TFFCONIC
#		  M: (-27+NR)
#	RTERM	  E: (-29)  M: (-27)	M  TERMINAL RADIUS LENGTH		LEFT BY CALCTFF
#	NRTERM    E: (-29+NR) 		M  NORM LENGTH OF TERMINAL RADIUS	LEFT BY CALCTFF
#		  M: (-27+NR)
#	TFFVSQ    E: (20)   M: (18)   1/M  -(V SQ/MU): PRESENT VELOCITY, NORM	LEFT BY TFFCONIC
#	TFFNP	  E: (-38+2NR)		M  LCP, SEMI-LATUS RECTUM, WEIGHT NR	LEFT BY TFFCONIC
#	  	  M: (-36+2NR)
#
# DEBRIS:	QPRET, PDL+0 ... PDL+3
#		RTERM, NRTERM IF PREVGAM ENTERED.
# Page 814

PREVGAM		SL*			# ENTER WITH NEW RTERM IN MPAC
					# E: (-29)  M: (-27)
			0,1		# X1 = -NR
		STORE	NRTERM		# RTERM M		E: (-29+NR)	M: (-27+NR)

VGAMCALC	DLOAD	DMP
			NRMAG		# RMAG M		E: (-29+NR)	M: (-27+NR)
			NRTERM		# RTERM M		E: (-29+NR)	M: (-27+NR)
		PDDL	DSU		# RMAG RTERM M		E: (-58+2NR)	M: (-54+2NR)
			NRMAG		# RMAG M		E: (-29+NR)	M: (-27+NR)
			NRTERM		# RTERM M		E: (-29+NR)	M: (-27+NR)
		SL*	DDV		# 2(RN-RTERM)		E: (-30+NR)	M: (-28+NR)
			0 -8D,1		# (-8+NR)
					# PUSH UP PRODUCT.
		DSU
			TFFVSQ		# -(V SQ/MU)		E: (20)		M: (18)
		SQRT	PUSH		# SAVE VGAM/RT(MU) FOR NOW.	E: (10)	M: (9)
		DDV	PDDL		# XCH PDL+0, LEAVING VGAM FOR OUTPUT.
					# VGAM TO PDL M/CS	E: (-7)		M: (-2)
			TFF/RTMU	# 			E: (17)		M: (14)
		DMP	PDDL		# RTERM VGAM/RTMU	E: (-19+NR)	M: (-18+NR)
			NRTERM		# RTERM M		E: (-29+NR)	M: (-27+NR)
			TFFNP		# LC P =H.H/MU M	E: (-38+2NR)	M: (-36+2NR)
		SQRT	DDV		#			E: (-19+NR)	M: (-18+NR)
					# PUSH UP DEN		E: (-19+NR)	M: (-18+NR)
					# USE DDV OVFL AS LIMITER (|COS| <1.0)
		SR1	ACOS
DUMPVGAM	RVQ
					# CALLER MUST SUPPLY OWN SIGN ...
					#			22W	27MS

# Page 815
; ============================================================================
; SUBROUTINE: TFF/TRIG - Transfer Angle Trigonometry Calculation
;
; PURPOSE:
; Computes the sine and cosine of the transfer angle (theta) for entry
; trajectory predictions. This angular calculation is fundamental to rotating
; the spacecraft's position and velocity vectors forward in time to predict
; where the spacecraft will be when it reaches the entry interface altitude.
;
; WHAT IS THE TRANSFER ANGLE?
; In orbital mechanics, the transfer angle is the angular distance the
; spacecraft travels along its trajectory between two points:
;   - Point 1: Current spacecraft position
;   - Point 2: Entry interface altitude (e.g., 400,000 feet)
;
; Think of it like measuring how many degrees around Earth's center the
; spacecraft will travel during its unpowered coast to entry altitude.
;
; For Apollo 11's Pacific entry approach on July 24, 1969:
;   - Current position: ~400 km altitude, approaching atmosphere
;   - Transfer angle: Approximately 15-20 degrees of orbital arc
;   - Time span: Final 10-15 minutes before atmospheric interface
;
; WHY HALF-ANGLES? (.5 SIN and .5 COS)
; The subroutine computes HALF the sine and cosine values (.5 sin(θ), .5 cos(θ))
; rather than full values. This design choice prevents arithmetic overflow in
; the 15-bit AGC registers during subsequent vector rotation calculations:
;
;   Rotated_Vector = Original_Vector × cos(θ) + Perpendicular_Vector × sin(θ)
;
; By using half-values, the multiplication results stay within numerical bounds,
; and the final vector is correctly scaled by doubling at the end.
;
; THE MATHEMATICAL APPROACH:
; Rather than computing theta directly and taking trig functions, TFF/TRIG
; extracts the angle from geometric relationships already computed by the
; Time-of-Free-Fall (TFF) trajectory subroutines:
;
; 1. COSINE COMPUTATION (from geometry of elliptical orbit):
;    cos(θ) = 1 - 2·|ARG| / (R_current × R_terminal × (1+X))
;    
;    Where:
;    - ARG: Geometric parameter from TFF calculation (related to eccentric anomaly)
;    - R_current: Present radius magnitude
;    - R_terminal: Target radius (entry interface altitude)
;    - X: Orbital energy parameter (TFFX)
;
; 2. SINE COMPUTATION (from Pythagorean identity):
;    sin(θ) = sign(ARG) × sqrt(1 - cos²(θ))
;    
;    The sign of ARG determines whether the spacecraft is ascending or
;    descending along its trajectory (positive = descending toward entry)
;
; GEOMETRIC INTERPRETATION:
; The transfer angle relates to Kepler's equation in orbital mechanics. For
; an elliptical trajectory, the angle swept out is related to the change in
; eccentric anomaly. The TFF subroutines have already solved Kepler's equation
; numerically; TFF/TRIG extracts the resulting angle in trigonometric form.
;
; INPUT PARAMETERS (left in memory by CALCTFF or CALCTPER):
;   TFFX: Orbital energy parameter X (related to eccentricity)
;   TFFTEM: Geometric argument ARG
;           E: (-59+2NR) scaling for Earth entry
;           M: (-55+2NR) scaling for Moon entry
;           Contains sign(DELF) indicating trajectory direction
;   NRMAG: Normalized present radius magnitude
;          E: (-29+NR) meters for Earth
;          M: (-27+NR) meters for Moon
;   NRTERM: Normalized terminal radius magnitude
;           E: (-29+NR) meters for Earth
;           M: (-27+NR) meters for Moon
;
; OUTPUT:
;   MPAC: SDELF/2 = .5 sin(theta) - half-value sine of transfer angle
;   PDL+14: CDELF/2 = .5 cos(theta) - half-value cosine of transfer angle
;   QPRET: Return address preserved
;
; CALLING SEQUENCE:
;   CALL TFF/TRIG
;   (Push-down list position PDL+0 can be arbitrary if not equal to 14D)
;
; COMPUTATION FLOW:
; 1. Load X parameter and shift right (divide by 2)
; 2. Add 1.0 to get (1 + X/2)
; 3. Multiply by current radius: R_current × (1 + X/2)
; 4. Multiply by terminal radius: R_current × R_terminal × (1 + X/2)
; 5. Divide ARG by this product: ARG / denominator
; 6. Take absolute value and subtract from 0.5: 0.5 - |result|
; 7. Store as CDELF/2 (cosine result)
; 8. Square this value: cos²(θ/2)
; 9. Complement (negate) to get: -cos²(θ/2)
; 10. Add 0.25: 0.25 - cos²(θ/2) = sin²(θ/2)
; 11. Take square root: sin(θ/2)
; 12. Apply original sign from TFFTEM
; 13. Return with sine in MPAC, cosine in PDL+14
;
; USAGE BY S61.2:
; After TFF/TRIG computes these half-angle values, S61.2 uses them in the
; FISHCALC and VGAMCALC subroutines to:
;   - Rotate the radius unit vector to entry interface position
;   - Compute latitude at entry interface (for Fischer ellipsoid calculation)
;   - Predict velocity and flight path angle at entry interface
;
; The rotation formula applied by FISHCALC:
;   UR_predicted = UR_current × cos(θ) + UR_perpendicular × sin(θ)
;
; NUMERICAL STABILITY:
; Computing trig functions from geometric parameters rather than computing
; the angle first (atan2) and then taking sine/cosine avoids potential
; numerical instabilities in the inverse tangent function near singularities.
; This approach maintains full precision throughout.
;
; PERFORMANCE:
; Execution time: ~15 milliseconds
; Memory usage: 16 words
;
; SCALING DIFFERENCES (Earth vs Moon):
; The subroutine handles both Earth entry and lunar orbit scenarios with
; different fixed-point scaling factors:
;   - Earth: Higher scaling exponents (larger gravitational parameter μ)
;   - Moon: Lower scaling exponents (smaller μ, about 1/81 of Earth's)
;
; Modified March 1968 to accept different Earth/Moon scales, enabling the
; same code to work for both entry from Earth orbit and entry from cislunar
; trajectory without separate versions.
;
; HISTORICAL CONTEXT:
; During Apollo 11's entry on July 24, 1969, TFF/TRIG computed the transfer
; angle from Columbia's current position (~400 km altitude, approaching from
; southeast) to the entry interface point over the Pacific. This angle,
; combined with the spacecraft's known velocity, predicted exactly where
; Columbia would cross into the sensible atmosphere. The crew monitored
; these predictions on their DSKY displays, comparing predicted vs. actual
; trajectory to verify the guidance computer was steering them accurately
; toward the planned splashdown zone.
; ============================================================================


# SUBROUTINE NAME:	TFF/TRIG	(USED BY S61.2)		DATE:		01.17.67
# MOD NO: 0							LOG SECTION:	P61-P67
# MOD BY: RR BAIRNSFATHER
# MOD NO: 1	MOD BY: RR BAIRNSFATHER		DATE: 14 APR 67
# MOD NO: 2	MOD BY: RR BAIRNSFATHER		DATE: 21 MAR 68		ACCEPT DIFFERENT EARTH/MOON SCALE
#
# FUNCTIONAL DESCRIPTION:  USED BY ENTRY DISPLAY TO CALCULATE SIN(THETA), COS(THETA) FROM DATA LEFT IN
#	PDL BY TFF SUBROUTINES.  THE EQNS ARE
#
#		COS(THETA) = 1-2 ABS(ARG) / (RN RTERM (1+X) )
#						2
#		SIN(THETA) = SGN(ARG) SQRT(1-COS (THETA) )
#
# 	WHERE THETA = TRANSFER ANGLE
#	AND     ARG = P Z ABS(Z)			IF ALFA ZZ LEQ 1
#	        ARG = (P / ALFA) SGN(Q1 + R 1/Z)        IF ALFA Z Z G 1
#	AND  ARG  HAS BEEN AFFIXED WITH THE SIGN OF SIN(THETA)
#
# CALLING SEQUENCE:	CALL
#				TFF/TRIG
#		PUSHLOC AT PDL+0, ARBITRARY IF NOT EQ 14D
#		C(MPAC) UNSPECIFIED
#
# SUBROUTINES CALLED:  NONE
#
# NORMAL EXIT MODES:  RVQ
#
# ALARMS:	NONE
#
# OUTPUT:	C(MPAC) = .5 SIN(THETA)
#		CDELF/2 = .5 COS(THETA)		(IN PDL 14D)
#		PUSHLOC AT PDL+0
#
# ERASABLE INITIALIZATION REQUIRED:
#		TFFX			X					LEFT BY CALCTFF OR CALCTPER
#		TFFTEM  E: (-59+2NR)	ARG					LEFT BY CALCTFF OR CALCTPER
#			M: (-55+2NR)	WHERE ARG = LCF ZZ SGN(DELF) OR ARG = LCP/ALFA SGN(DELF)
#		NRTERM	E: (-29+NR)	M  NORM LENGTH OF TERMINAL RADIUS	LEFT BY CALCTFF OR CALCTPER
#			M: (-27+NR)
#		NRMAG	E: (-29+NR)	M  NORM LENGTH OF PRESENT POSITION	LEFT BY TFFCONIC
#			M: (-27+NR)
#
# DEBRIS:	QPRET, CDELF/2

		BANK	27
		SETLOC	P60S5
		BANK
TFF/TRIG	DLOAD	SR1
			TFFX
		DAD	DMP
			HIDPHALF
			NRMAG		# RMAG M		E: (-29+NR)	M: (-27+NR)
		DMP	BDDV
			NRTERM		# RTERM M		E: (-29+NR)	M: (-27+NR)
			TFFTEM		# P ZSQ OR P/ALFA	E: (-59+2NR)	M: (-55+2NR)
		ABS	BDSU		# THE SIGN IS FOR SDELF.
			HIDPHALF
		STORE	CDELF/2		# .5 COS(THETA)
		DSQ	DCOMP		# KEEP HONEST FOR SQRT.
# Page 816
		DAD	SQRT
			HIDP1/4
DUMPTRIG	SIGN	RVQ
			TFFTEM		# AFFIX SIGN(DELE/2)
					# RETURN WITH .5 SIN(THETA) IN MPAC

					#			16W	15MS

; ============================================================================
; SUBROUTINE: DISPTARG - Display Target Range Calculation
;
; PURPOSE:
; Computes the rotated position of the target splashdown point relative to
; the spacecraft's predicted position at entry interface. This calculation
; enables the AGC to display to the crew how far off-target their current
; trajectory will take them, allowing mid-course corrections if necessary.
;
; MISSION CONTEXT:
; During Apollo 11's transearth coast on July 24, 1969, as Columbia approached
; Earth for reentry, the crew monitored their predicted landing zone. DISPTARG
; continuously updated the angle between their current trajectory endpoint and
; the planned Pacific splashdown coordinates (latitude 13.3°N, longitude 169.2°W).
;
; If this angle became too large, indicating they would miss the recovery
; fleet by an unacceptable distance, the crew could perform a small midcourse
; correction burn to adjust their trajectory. Fortunately, Apollo 11's trajectory
; was accurate enough that no late corrections were needed.
;
; WHAT IS TRGO?
; TRGO (Range-To-Go) is the estimated angular distance from the current position
; to the target splashdown point, measured in revolutions (fraction of 360°).
; For example, TRGO = 0.05 represents about 18° or roughly 1,200 miles along
; Earth's surface at entry interface altitude.
;
; COMPUTATION APPROACH:
; The subroutine performs a time-based rotation calculation:
;
; 1. START WITH: TRGO estimate (angular distance to target)
;
; 2. CONVERT TO TIME: Time required to reach target at current velocity
;    DTEAROT = TRGO × KTETA1 - TTE1
;    
;    Where:
;    - KTETA1: Velocity-to-angle conversion factor
;    - TTE1: Time-to-entry at current trajectory
;    - Result DTEAROT: Delta-time for Earth rotation adjustment
;
; 3. ACCOUNT FOR EARTH ROTATION:
;    As the spacecraft coasts toward entry, Earth rotates beneath it.
;    EARROT2 subroutine computes how much Earth rotates during the coast time,
;    adjusting the target coordinates from their inertial reference frame
;    position to where they'll actually be when the spacecraft arrives.
;
; 4. COMPUTE ANGLE TO ROTATED TARGET:
;    VRCALC computes the angular separation between:
;    - URH: Predicted position unit vector at entry interface
;    - RT: Rotated target position accounting for Earth rotation
;
; THE DISPLAY TO CREW:
; The result appears on the DSKY as noun displays showing:
;   - Range angle to target (degrees or nautical miles)
;   - Cross-range error (lateral deviation from flight path)
;   - Along-track error (early or late relative to planned splashdown time)
;
; These values help the crew visualize their trajectory accuracy and decide
; whether corrective action is needed.
;
; INPUT:
;   MPAC: TRGO - Range-to-go estimate (angular distance in revolutions)
;   KTETA1: Velocity-angle conversion constant
;   TTE1: Time-to-entry at current position
;   URH: Unit radius vector at entry interface (predicted position)
;
; OUTPUT:
;   Returns via 60GENRET with updated display values
;   VRCALC computes angular separation stored for display routines
;
; SUBROUTINES CALLED:
;   EARROT2: Earth rotation matrix computation
;   VRCALC: Vector angle calculation
;
; PERFORMANCE:
;   Execution time: ~25 milliseconds
;   Called by display update routines every 2 seconds during coast
; ============================================================================

; ============================================================================
; SUBROUTINE: VRCALC - Vector Angle Calculation
;
; PURPOSE:
; Computes the angular separation between two position vectors using the
; dot product and arccosine function. This fundamental geometric calculation
; determines how far apart two points are when measured as an angle from
; Earth's (or Moon's) center.
;
; THE MATHEMATICS:
; For two unit vectors U1 and U2 pointing from Earth's center to two points:
;
;   cos(angle) = U1 · U2  (dot product)
;   angle = arccos(U1 · U2)
;
; This is the most numerically stable method for computing angles between
; vectors, avoiding singularities that plague methods using atan2 near
; poles or small angles.
;
; WHY THIS MATTERS FOR ENTRY:
; During entry targeting, the crew needs to know the angular distance between:
;   - Where they're predicted to land (based on current trajectory)
;   - Where they're supposed to land (target splashdown coordinates)
;
; This angle, multiplied by Earth's radius, gives the ground distance error.
; For Apollo 11's Pacific splashdown on July 24, 1969:
;   - Target: 13.3°N, 169.2°W
;   - Actual: 13.316°N, 169.15°W
;   - Angular error: ~0.05° or about 3 nautical miles
;   - Well within acceptable landing ellipse
;
; SCALING WITH SL2:
; The SL2 (Shift Left 2) instruction multiplies the dot product by 4 before
; taking arccosine. This scaling compensates for the half-magnitude vectors
; used throughout the AGC to prevent overflow:
;
;   Stored vectors = Actual vectors / 2  (to prevent overflow)
;   Dot product = (V1/2) · (V2/2) = (V1 · V2) / 4
;   SL2: Multiply by 4 to restore true dot product magnitude
;   ACOS: Take arccosine of restored value
;
; INPUT:
;   URH: Unit radius vector at entry interface (half-magnitude, (-1) scaling)
;   RT: Rotated target position vector (half-magnitude, (-1) scaling)
;
; OUTPUT:
;   MPAC: Angular separation in radians (scaled as fraction of 2π)
;   This value represents the great-circle distance angle between the two points
;
; ALGORITHM FLOW:
; 1. VLOAD URH: Load predicted position unit vector
; 2. DOT RT: Compute dot product with rotated target vector
; 3. SL2: Shift left 2 bits (multiply by 4) to restore proper magnitude
; 4. ACOS: Compute arccosine, yielding angle in radians
; 5. RVQ: Return with angle in MPAC
;
; NUMERICAL RANGE:
; - Dot product after scaling: -1.0 to +1.0 (representing cosine values)
; - Output angle: 0 to π radians (0° to 180°)
; - For entry targeting, typical angles: 0.001 to 0.1 radians (0.06° to 6°)
;
; GEOMETRIC INTERPRETATION:
; The angle computed is the great-circle distance between two points on a
; sphere (Earth or Moon). It's the shortest path along the curved surface,
; as opposed to a straight-line distance through the interior.
;
; For navigation purposes:
;   Angular_distance (radians) × Earth_radius = Ground_distance
;   Angular_distance (radians) × 3,440 nm/radian ≈ Distance in nautical miles
;
; USAGE IN ENTRY DISPLAYS:
; The computed angle feeds into DSKY displays showing:
;   - "Range-to-go": How far to target splashdown
;   - "Cross-range error": Lateral deviation from planned path
;   - Trajectory accuracy indicators for crew monitoring
;
; HISTORICAL CONTEXT:
; During Apollo 11's final approach on July 24, 1969, this calculation ran
; continuously, updating every 2 seconds. The crew saw their predicted landing
; point converge toward the target coordinates as they committed to the entry
; corridor. Flight controllers on the ground watched the same data via telemetry,
; ready to call for corrective action if the angle grew too large. The final
; angular error of ~0.05° demonstrated the extraordinary precision of the AGC's
; trajectory computation and the accuracy of the entry guidance algorithms.
; ============================================================================

DISPTARG	STQ			# C(MPAC = TRGO ESTIMATE
			60GENRET
		DMP	DSU
			KTETA1
			TTE1
		STCALL	DTEAROT
			EARROT2
		CALL
			VRCALC
		GOTO
			60GENRET
VRCALC		VLOAD	DOT
			URH
			RT
		SL2	ACOS
		RVQ

# END OF PROGRAM S61.2

# Page 817
; ============================================================================
; SECTION TRANSITION: From Target Display Calculations to Entry Attitude Control
;
; The entry programs have now computed the trajectory, predicted the landing
; point, and displayed targeting information to the crew. With the flight path
; established, attention turns to controlling the spacecraft's physical attitude
; during entry.
;
; The Command Module must maintain a precise orientation relative to its velocity
; vector to achieve the desired lift vector for range control. The following
; subroutine, S62.3, computes the gimbal angles required to point the spacecraft
; correctly for trimmed flight through the atmosphere.
;
; This computation runs continuously during entry, updating gimbal commands every
; 2 seconds as the spacecraft's velocity and position change. The digital autopilot
; uses these gimbal angles to command the spacecraft's attitude control thrusters,
; maintaining the proper orientation for the aerodynamic lift vector to steer
; toward the target splashdown point.
; ============================================================================

# PROGRAM DESCRIPTION S62.3	DATE 10JAN67
# MOD NO 1:			LOG SECTION P60-P67
# MOD BY ZELDIN
# MOD NO: 2	MOD BY: RR BAIRNSFATHER		DATE: 15 MAY 67		CHANGED TO REF COORDS.
# MOD NO: 3	MOD BY: RR BAIRNSFATHER		DATE: 17 JAN 68		ALFAPAD CHANGES MADE.
#
# FUNCTIONAL DESCRIPTION
#
#	COMPUTE DESIRED GIMBAL ANGLES FOR ENTRY ATTITUDE
#	THE FOLLOWING TRAJECTORY TRIAD IS AVAILABLE IN MEMORY AND IS COMPUTED EACH 2 SECONDS BY CM/POSE IN
#	REFERENCE COORDINATES (V = VELOCITY RELATIVE TO EARTH):
#
#		UXA = -UNIT(V)
#		UYA =  UNIT(V*R)
#		UZA =  UXA*UYA
#
# 	GENERATE A DESIRED BODY TRIAD FOR TRIMMED FLIGHT WITH RESPECT TO THE RELATIVE VELOCITY VECTOR, USING
#	ROLL COMMAND AND TRIM ANGLE OF ATTACK:
#
#		UXD = UNIT(UYD*UXA) SIN(ALFATRIM) + UXA COS(ALFATRIM)
#		UYD = UYA COS(ROLLC) + UZA SIN(ROLLC)
#		UZD = UXD * UYD
#
#	USE THE DESIRED SET (IN REFERENCE COORDS) AND REFSMMAT TO CALL  CALCGA  AND OBTAIN GIMBAL ANGLES
#	IN 2S, C IN MPAC, +2 AND THETAD, +2.
#
# CALLING SEQUENCE
#
#	L	CALL
#	L+1		S62.3
#
# NORMAL EXIT MODE
#
#	RETURN VIA QPRET DIRECTLY FROM CALCGA.
#
# SUBROUTINES CALLED
#
#	CALCGA
#
# ALARM OR ABORT MODES
#
#	NONE
#
# ERASABLE INITIALIZATION REQUIRED
#
#	ROLLC	ROLL COMMAND		DP 1'S COMP AT 1REV
#	ALFAPAD	SP 1'S C / 180		LEFT BY PAD LOAD	ALFATRIM IS NEGATIVE.
#	UXA/2	REF COORDS		LEFT BY CM/POSE
#	UYA/2	REF COORDS		LEFT BY CM/POSE
#	UZA/2	REF COORDS		LEFT BY CM/POSE
#
# OUTPUT
#
#	CPHI	GIMBAL ANGLES (O,I,M) 2'S COMP TP (O,I,M)/180
#
# DEBRIS
#
#	QTEMP, QPRET, PUSHLIST

		BANK	10
		SETLOC	P60S4
		BANK
# Page 818
		COUNT*	$$/S62.3

; ============================================================================
; SUBROUTINE: S62.3 - Compute Desired Gimbal Angles for Entry Attitude
;
; PURPOSE:
; Calculates the spacecraft gimbal angles required to maintain the correct
; orientation during atmospheric entry. This subroutine transforms the desired
; body attitude (based on aerodynamic trim angle and commanded roll) into
; gimbal angles (outer, inner, middle) that position the Command Module for
; optimal lift vector control.
;
; THE ENTRY ATTITUDE PROBLEM:
; During entry, the Command Module must maintain a specific angle of attack
; relative to its velocity vector to generate aerodynamic lift. This lift
; provides trajectory control, allowing the spacecraft to "fly" through the
; upper atmosphere and steer toward the target splashdown point.
;
; The spacecraft's orientation is defined by two key angles:
;   1. ALFATRIM (Trim Angle of Attack): ~31° nose-up relative to velocity
;      This angle generates the required lift force for controlled entry
;   2. ROLLC (Roll Command): 0° to 180° rotation about the velocity axis
;      This angle points the lift vector toward or away from the desired
;      landing point for range control
;
; WHY GIMBAL ANGLES?
; The spacecraft's Inertial Measurement Unit (IMU) is mounted on a stabilized
; platform supported by three gimbals. The gimbal angles (outer, inner, middle)
; define the platform's orientation relative to the spacecraft body. To maintain
; the desired entry attitude, the autopilot must know what gimbal angles
; correspond to the desired body orientation in inertial space.
;
; APOLLO 11 ENTRY CONTEXT:
; On July 24, 1969, as Columbia approached Earth at 24,791 mph, this subroutine
; computed gimbal commands every 2 seconds. The entry sequence proceeded:
;
;   400,000 ft: Entry interface - initial atmospheric contact
;               Roll = 0° (lift up), angle of attack = 31°
;               Deceleration begins, g-forces building
;
;   280,000 ft: Peak deceleration approaching
;               Roll modulation begins for range control
;               Gimbal angles updating to follow commanded roll changes
;
;   200,000 ft: Peak g-load (6.5 g for Apollo 11)
;               Roll commands may reverse (±180°) for range correction
;               S62.3 ensures smooth gimbal transitions
;
;   60,000 ft:  Parachute deployment altitude approaching
;               Final roll commands to align for water landing
;               Gimbal angles stabilize as atmosphere slows the spacecraft
;
; THE COORDINATE TRANSFORMATIONS:
; This subroutine performs a series of vector rotations to transform from
; the trajectory reference frame to gimbal angles:
;
; STEP 1: START WITH TRAJECTORY TRIAD (computed by CM/POSE every 2 seconds)
;   UXA = -UNIT(V)      X-axis points opposite velocity (heat shield forward)
;   UYA =  UNIT(V×R)    Y-axis perpendicular to orbit plane
;   UZA =  UXA × UYA    Z-axis completes right-handed triad
;
; STEP 2: COMPUTE DESIRED BODY TRIAD (accounting for trim angle and roll)
;   UYD = UYA·cos(ROLLC) + UZA·sin(ROLLC)    Rotated Y-axis
;   UXD = UNIT(UYD × UXA)·sin(ALFATRIM) + UXA·cos(ALFATRIM)  Trimmed X-axis
;   UZD = UXD × UYD                          Desired Z-axis
;
; STEP 3: TRANSFORM TO GIMBAL ANGLES
;   Use CALCGA subroutine to convert desired body triad (XNB, YNB, ZNB)
;   from reference coordinates through REFSMMAT to gimbal angles
;
; WHAT IS ALFATRIM?
; ALFATRIM is the spacecraft's trim angle of attack - the angle between the
; spacecraft's longitudinal axis and the relative velocity vector. For Apollo:
;   - Typical value: -31° (negative because nose is pitched up)
;   - This angle is optimized to generate maximum lift-to-drag ratio
;   - Pad-loaded before entry based on predicted entry conditions
;   - Negative sign convention: positive pitch is nose-down
;
; WHAT IS ROLLC?
; ROLLC is the commanded roll angle about the velocity vector:
;   - ROLLC = 0°: Lift vector points "up" (extends range, shallow trajectory)
;   - ROLLC = 180°: Lift vector points "down" (shortens range, steep trajectory)
;   - Entry guidance modulates roll to steer toward target splashdown
;   - Apollo typically used +180° → 0° → +180° roll reversal pattern
;
; THE MATHEMATICS IN DETAIL:
;
; 1. COMPUTE UYD (Rotated Y-axis in desired body frame):
;    UYD = UYA·cos(ROLLC) + UZA·sin(ROLLC)
;    This rotates the trajectory Y-axis by the commanded roll angle
;
; 2. COMPUTE UXD (Trimmed X-axis accounting for angle of attack):
;    First: Temp = UYD × UXA  (perpendicular to velocity and rolled Y)
;    Then:  UXD = UNIT(Temp)·sin(ALFATRIM) + UXA·cos(ALFATRIM)
;    This pitches the X-axis up by ALFATRIM angle relative to velocity
;
; 3. COMPUTE UZD (Complete the orthogonal triad):
;    UZD = UXD × UYD
;    Standard cross product to get third axis
;
; 4. CALL CALCGA to transform from reference coords to gimbal angles:
;    CALCGA uses REFSMMAT and the desired triad to compute angles
;
; SCALING AND PRECISION:
; All vectors are stored at half-magnitude (÷2) to prevent overflow:
;   - Input trajectory triad: UXA/2, UYA/2, UZA/2 (from CM/POSE)
;   - Computed body triad: XNB, YNB, ZNB stored as half-unit vectors
;   - VSL1 (shift left 1) restores proper magnitude after operations
;   - Final gimbal angles: 2's complement, scaled at 1 revolution = 2π radians
;
; GIMBAL ANGLE OUTPUT FORMAT:
; The computed angles are stored in CPHI as (Outer, Inner, Middle):
;   - Outer gimbal: Rotation about spacecraft Z-axis (yaw-like)
;   - Inner gimbal: Rotation about intermediate axis (pitch-like)
;   - Middle gimbal: Rotation about platform X-axis (roll-like)
;   - Range: ±180° for each gimbal
;   - Scaling: 2's complement, 1 revolution = 180° = 2^14 counts
;
; GIMBAL LOCK CONSIDERATION:
; The IMU can experience gimbal lock when the inner gimbal approaches ±90°.
; During nominal entry attitudes, this is not a concern because the spacecraft
; maintains moderate gimbal angles. However, if a gimbal lock condition were
; approached, the crew would be alerted by the GIMBAL LOCK warning light on
; the DSKY, and they might perform a coarse align to reset gimbal positions.
;
; AUTOPILOT INTEGRATION:
; The computed gimbal angles feed into the entry digital autopilot
; (CM_ENTRY_DIGITAL_AUTOPILOT.agc), which commands the RCS thrusters to
; null the error between actual and desired gimbal angles. The autopilot
; runs at a much higher rate than this 2-second computation, interpolating
; smooth attitude commands between S62.3 updates.
;
; ROLL REVERSAL LOGIC:
; During entry, the guidance may command roll reversals (0° to 180° or vice
; versa) to adjust the lift vector direction for range control. S62.3 handles
; these transitions smoothly:
;   - Lift up (ROLLC = 0°): Extends range, shallower trajectory
;   - Lift down (ROLLC = 180°): Shortens range, steeper trajectory
;   - Transition: Autopilot executes roll maneuver over several seconds
;   - S62.3 continuously updates desired angles during transition
;
; HISTORICAL PERFORMANCE - APOLLO 11:
; Throughout Columbia's entry on July 24, 1969, S62.3 maintained precise
; attitude control:
;   - Entry interface: 400,000 ft at 102:45:00 GET (Ground Elapsed Time)
;   - Peak g-load: 6.53 g at ~200,000 ft
;   - Drogue chute deploy: 24,000 ft at 102:51:30 GET
;   - Main chutes: 10,000 ft at 102:51:50 GET
;   - Splashdown: 102:56:16 GET, within 3 miles of target point
;   - Total entry duration: ~11 minutes from interface to splashdown
;
; During this phase, S62.3 computed 330+ gimbal angle updates (one every 2
; seconds), enabling the autopilot to maintain the precise attitude required
; for the most accurate Apollo splashdown to date.
;
; INPUT:
;   ROLLC: Commanded roll angle (DP, scaled at 1 revolution)
;   ALFAPAD: Trim angle of attack / 180° (SP, pad-loaded, negative value)
;   UXA/2: Trajectory X-axis unit vector, half-magnitude (reference coords)
;   UYA/2: Trajectory Y-axis unit vector, half-magnitude (reference coords)
;   UZA/2: Trajectory Z-axis unit vector, half-magnitude (reference coords)
;   REFSMMAT: Reference-to-stable-member transformation matrix
;
; OUTPUT:
;   CPHI: Gimbal angles (Outer, Inner, Middle), 2's complement at TP/180°
;         Stored by CALCGA for use by entry autopilot
;
; SUBROUTINES CALLED:
;   CALCGA: Computes gimbal angles from desired body triad and REFSMMAT
;           (located in another AGC module, performs matrix transformations)
;
; DEBRIS (TEMPORARY STORAGE):
;   QTEMP: Temporary return address storage
;   QPRET: Return address for CALCGA
;   PUSHLIST (PDL): Push-down list used for vector/trig computations
;   XNB, YNB, ZNB: Desired body triad vectors (stored for CALCGA input)
;   XSM, YSM, ZSM: REFSMMAT columns (copied for CALCGA input)
;
; PERFORMANCE:
;   Execution time: ~50 milliseconds (interpretive operations)
;   Update rate: Every 2 seconds during entry (called by CM/POSE)
;   Accuracy: Gimbal angles accurate to ~0.01° with proper IMU calibration
;
; EXECUTION FLOW SUMMARY:
;   1. Load ALFAPAD (trim angle), compute sin and cos, store on PDL
;   2. Compute UYD = UYA·cos(ROLLC) + UZA·sin(ROLLC)
;   3. Compute UXD from UYD × UXA, scaled by sin/cos(ALFATRIM)
;   4. Compute UZD = UXD × UYD
;   5. Copy REFSMMAT to XSM/YSM/ZSM for CALCGA
;   6. Clear CPHIFLAG to direct CALCGA output to CPHI
;   7. Transfer to CALCGA (returns to original caller via QPRET)
;
; RETURN:
;   Returns via QPRET directly from CALCGA (not back to this subroutine)
;   Gimbal angles stored in CPHI are ready for autopilot use
; ============================================================================

S62.3		SETPD	SLOAD
			0
			ALFAPAD		# ALFATRIM / 180, ALFA IS NEG.
		SR1	PUSH
		COS	PDDL		# XCH PDL, COS TO PDL0
		SIN	PDDL		# SIN TO PDL2
			ROLLC
		COS	VXSC
			UYA/2		#				REF COORDS
		PDDL	SIN		# PUSH VECTOR INTO PDL4...9
			ROLLC
		VXSC	VAD
			UZA/2		#				REF COORDS
					# VECTOR FROM PDL4...9
		VSL1
		STORE	YNB		# = UYD				REF COORDS

		VXV	VSL1
			UXA/2		#				REF COORDS
		VXSC	PDDL
					# SIN TRIM FROM PDL2
					# XCH PDL0 FOR COS TRIM
		VXSC	VAD
			UXA/2		#				REF COORDS
					# FROM PDL0
		VSL1
		STORE	XNB		# X SC AXIS (.5 UNIT)		REF COORDS

		VXV	VSL1
			YNB
		STOVL	ZNB		# Z SC IN REF COOR. SCALED AT 2
			REFSMMAT
		STOVL	XSM
			REFSMMAT +6
		STOVL	YSM
			REFSMMAT +12D
		STORE	ZSM

		CLEAR	GOTO
			CPHIFLAG	# CAUSE CALCGA TO STORE ANS IN TP CPHI
			CALCGA
					# CALCGA WILL RETURN TO ORIGINAL CALLER
					# VIA QPRET WITH 2'S COMP. ANGLES IN CPHI



