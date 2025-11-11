# Copyright:	Public domain.
# Filename:	FRESH_START_AND_RESTART.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	211-237
# Mod history:	2009-05-19 HG	Transcribed from page images.
#		2010-12-31 JL	Fixed page number comments.
#		2011-01-06 JL	Fixed pseudo-label indentation.
#		2011-01-06 JL	Added missing comment characters.
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
; FILE: FRESH_START_AND_RESTART.agc
; MODULE: System Initialization and Recovery
; MISSION PHASE: All phases (launch through landing through ascent)
;
; TL;DR: Implements cold boot (Fresh Start) and fault recovery (Restart)
;        procedures for the Apollo Guidance Computer. Handles memory
;        initialization, system state restoration, and program selection
;        (V37 Major Mode Change). Critical for recovering from power
;        transients and the famous 1202 program alarms during Apollo 11's
;        lunar landing without losing guidance state.
;
; COMMENT-ONLY READERS: This code ensures the computer can recover from
;        failures instantly. During the landing, when the 1202 alarms
;        occurred, this restart protection allowed the mission to continue.
; CODE-ALONG READERS: Study the restart table mechanism, memory integrity
;        checking (ERASCHK), and state preservation logic that enabled
;        fault-tolerant real-time operation.
; ============================================================================

# Page 211
		BANK	10
		SETLOC	FRANDRES
		BANK

		EBANK=	LST1

		COUNT*	$$/START	# FRESH AND RESTART

; ============================================================================
; FRESH START ENTRY POINT
;
; The Lunar Module's computer has just been powered on or the crew has
; requested a complete system reset via the DSKY. This "Fresh Start"
; procedure clears all memory and reinitializes the guidance computer to
; a known safe state, ready to begin a new mission program.
;
; During Apollo 11, fresh starts were performed during checkout and after
; LM separation to ensure clean initial conditions before critical phases.
; ============================================================================

SLAP1		INHINT			# FRESH START.  COMES HERE FROM PINBALL.
		TC	STARTSUB	# SUBROUTINE DOES MOST OF THE WORK

; Simulation mode hook. During ground testing, this could be patched to
; start simulation routines. For flight operations, this jumps to SKIPSIM.

STARTSW		TCF	SKIPSIM		# PATCH....TCF STARTSIM FOR SIMULATION
STARTSIM	CAF	BIT14
		TC	FINDVAC
SIM2CADR	OCT	77777		# PATCH 2CADR (AND EBANK DESIGNATION) OF
		OCT	77777		# SIMULATION START ADDRESS.

; Initialize DSKY display indicators. All indicator lights are turned off
; except GIMBAL LOCK and NO ATT (No Attitude) warnings, which remain on
; until the IMU (Inertial Measurement Unit) is properly aligned and ready.
; The PROG light (bit 15) is illuminated to indicate fresh start completion.

SKIPSIM		CA	DSPTAB +11D	# TURN OFF ALL DSPTAB +11D LAMPS
		MASK	BITS4&6		# EXCEPT THE GIMBAL LOCK & NO ATT ONLY ON
		AD	BIT15		# REQUESTED FRESH START.
		TS	DSPTAB +11D

; Initialize downlink telemetry system for one complete memory dump to
; Mission Control. This allows ground controllers to verify computer state
; after the fresh start.

		CA	BIT12		# INITIALIZE DOWNLINK EARASABLE MEMORY
		TS	DUMPCNT		# DUMP FOR ONE PASS

; Clear all system fault counters and failure registers. Fresh start means
; the computer begins with no memory of previous errors or malfunctions.
; During Apollo 11, this ensured clean initialization for each mission phase.

		CA	ZERO
		TS	ERCOUNT
		TS	FAILREG
		TS	FAILREG +1
		TS	FAILREG +2
		TS	REDOCTR

		CS	PRIO12
		TS	DSRUPTSW

; ============================================================================
; TRANSITION: From initial setup to engine and RCS safety checks
;
; The computer has cleared its error registers and initialized telemetry.
; Now it must ensure all propulsion systems are in a safe state: descent
; and ascent engines off, RCS thrusters off. This prevents any accidental
; firing during initialization - critical for crew safety.
; ============================================================================

DOFSTART	CAF	BIT14		# INSURE ENGINE IS OFF.
		EXTEND
		WRITE	DSALMOUT
		CS	ZERO
		TS	THRUST

; Initialize attitude control flags and set display priority. RCSFLAGS = 4
; enables proper attitude error display on the DSKY. RESTREG priority 30
; ensures display updates have high scheduling priority.

DOFSTRT1	CAF	FOUR
		TS	RCSFLAGS	# INITIALIZE ATTITUDE ERROR DISPLAYS.
		CA	PRIO30
		TS	RESTREG		# SUPER BANK PRIORITY FOR DISPLAYS.

; Zero-initialize critical system variables. DAP (Digital Autopilot)
; variables, channel masks for RCS control, failure monitoring state,
; and mode indicators all start at known zero values. ERESTORE and SMODE
; are marked as MUST NOT BE REMOVED - their initialization is critical
; for restart protection to function correctly.

		CA	ZERO
		TS	ABDELV		# DAP INITIALIZATION
		TS	NVSAVE
		TS	EBANKTEM
# Page 212
		TS	CH5MASK
		TS	CH6MASK
		TS	PVALVEST	# FOR RCS FAILURE MONITOR
		TS	ERESTORE	# ***** MUST NOT BE REMOVED FROM DOFSTART
		TS	SMODE		# ***** MUST NOT BE REMOVED FROM DOFSTART
		TS	DNLSTCOD	# SELECT P00 DOWNLIST
		TS	AGSWORD		# ALLOW AGS INITIALIZATION
		TS	UPSVFLAG	# ZERO UPDATE STATE VECTOR REQUEST FLAGWRD

; Turn off all RCS (Reaction Control System) thrusters by writing zero to
; output channels 5, 6, 12, 13, and 14. These channels control the 16 RCS
; jets arranged in four quads around the LM. Ensuring all jets are off
; prevents unintended spacecraft rotation during computer initialization.

		EXTEND
		WRITE	CHAN5		# TURN OFF RCS JETS.
		EXTEND
		WRITE	CHAN6		# TURN OFF RCS JETS.
		EXTEND
		WRITE	CHAN12
		EXTEND
		WRITE	CHAN13
		EXTEND
		WRITE	CHAN14
; Check if IMU (Inertial Measurement Unit) was in COARSE ALIGN mode when
; gimbal lock occurred. If so, restore it to COARSE ALIGN by setting bits
; in CHAN12. The IMU must be properly aligned before navigation can begin.

		CS	DSPTAB +11D
		MASK	BITS4&6
		CCS	A
		TC	+4
		CA	BITS4&6
		EXTEND			# THE IMU WAS IN COARSE ALIGN IN GIMBAL
		WOR	CHAN12		# LOCK, SO PUT IT BACK INTO COARSE ALIGN.

; Call MR.KLEAN subroutine to clear all phase tables. This resets the state
; of all running programs and prepares the computer for a clean program start.

 +4		TC	MR.KLEAN

; Initialize mode register to zero. MODREG tracks the current major mode
; program (P00, P63, P12, etc.). Zero indicates no program is running yet.

		CS	ZERO
		TS	MODREG

; Initialize IMU operating mode. IM30INIF sets the IMU to initial state,
; waiting for crew-initiated alignment procedure (P52 or P57). Until aligned,
; the NO ATT indicator remains lit on the DSKY.

		CAF	IM30INIF	# FRESH START IMU INITIALIZATION
		TS	IMODES30

; ============================================================================
; TRANSITION: From IMU setup to Digital Autopilot (DAP) initialization
;
; With propulsion systems safe and IMU mode set, the computer now initializes
; the Digital Autopilot parameters. The DAP controls the LM's attitude using
; RCS thrusters, maintaining proper orientation during all mission phases.
; These initial values provide conservative control until ground controllers
; upload optimized parameters via uplink.
; ============================================================================

; Initialize DAP attitude control parameters. MAXDB sets maximum attitude
; deadband (the allowed drift before thrusters fire). RATEINDX = 4 sets
; initial rotational rate for Kalman filter maneuvers. DAPBOOLS initializes
; autopilot configuration flags.

		CAF	MAXDB
		TS	DB
		CAF	FOUR
		TS	RATEINDX	# INITIALZE KALCMANU RATE
		CA	BOOLSTRT
		TS	DAPBOOLS
		CAF	EBANK6
		TS	EBANK
		EBANK=	HIASCENT

; Initialize hand controller sensitivity (STIKSENS) and rate deadband
; (-RATEDB) for crew manual control. HIASCENT holds maximum ascent stage
; mass - used by autopilot acceleration calculations (1/ACCS) until ground
; uploads actual vehicle mass via pad load.

		CA	STIKSTRT
		TS	STIKSENS
		CA	RATESTRT
		TS	-RATEDB
		CAF	FULLAPS		# INITIALIZE MAXIMUM ASCENT MASS FOR USE
		TS	HIASCENT	#   BY 1/ACCS UNTIL THE PAD LOAD IS DONE.
; Load DAP filter gains with best-estimate default values. These control
; parameters tune the autopilot's response to attitude errors:
; - DKTRAP/LMTRAP: 0.14 degree error trap for docked/LM-only configurations
; - DKKAOSN/LMKAOSN: 6 second gain for angle (alpha) control
; - LMOMEGAN: Unity gain for LM angular rate (omega)
; - DKOMEGAN: 1 second gain for docked angular rate
; - DKDB: 1.4 degree deadband for docked configuration
; Ground controllers may overwrite these via pad load uplink.

		CA	77001OCT	#     LOAD DAP FILTER GAINS PAD LOAD.
# Page 213
		TS	DKTRAP		#       TO BEST PRESENT ESTIMATE OF GOODIES
		TS	LMTRAP		# .14 DEG
		CA	60DEC
		TS	DKKAOSN
		TS	LMKAOSN		# 6 SEC GAIN FOR ALPHA
		CA	ZERO
		TS	LMOMEGAN	# UNITY GAIN
		CA	TEN
		TS	DKOMEGAN	# 1 SEC GAIN FOR OMEGA
		CAF	BIT8		# SET DOCKED DB TO 1.4 DEG.  MAY OVERWRITE
		TS	DKDB		#	WITH PAD LOAD.

; Initialize IMU mode 33 with DAP and attitude error display both disabled
; until IMU zero procedure completes. BIT6 keeps displays off.

		CAF	IM33INIT
		AD	BIT6		# KEEP BOTH DAP AND ERROR-NEEDLES DISPLAY
		TS	IMODES33	#	OFF UNTIL ICDU ZERO IS FINISHED.

; ============================================================================
; TRANSITION: From DAP gains to STATE switch initialization
;
; With autopilot parameters loaded, the computer now initializes system state
; flags (STATE +0 through STATE +11D). These control critical mission modes
; like surface operations, lunar/cislunar navigation, and propulsion status.
; Certain flags are preserved from previous state (REFSMFLG, SURFFLAG, etc.)
; to maintain mission continuity across fresh starts.
; ============================================================================

; Initialize system state flags from SWINIT table. The STATE array holds
; critical mission status flags. On fresh start, most flags reset to default
; values (loaded from SWINIT), but specific flags are preserved by masking:
; - REFSMFLG (REFSMMAT valid): Preserves inertial reference frame alignment
; - SURFFLAG (on lunar surface): Maintains surface operation mode
; - CMOONFLG/LMOONFLG (Moon influence): Preserves gravitational sphere
; - APSFLAG (APS engine): Maintains ascent propulsion configuration

		EXTEND			# INITIALIZE SWITCHES ONLY ON FRESH START.
		DCA	SWINIT
		DXCH	STATE
		CA	SWINIT +2
		TS	STATE +2
		CA	REFSMBIT	# DO NOT ALTER REFSMFLG ON FRESH START.
		MASK	STATE +3
		AD	SWINIT +3
		TS	STATE +3
		EXTEND
		DCA	SWINIT +4
		DXCH	STATE +4
		EXTEND
		DCA	SWINIT +6
		DXCH	STATE +6
		CA	SURFFBIT	# DO NOT ALTER	SURFFLAG ON FRESH START.
		AD	CMOONBIT	#		CMOONFLG
		AD	LMOONBIT	#		LMOONFLG
		MASK	STATE +8D
		AD	SWINIT +8D
		TS	STATE +8D
		CA	SWINIT +9D
		TS	STATE +9D
		CA	APSFLBIT	# DO NOT ALTER APSFLAG ON FRESH START.
		MASK	STATE +10D
		AD	SWINIT +10D
		TS	STATE +10D
		CAF	SWINIT +11D
		TS	STATE +11D

; Fresh start initialization complete. Transfer control to DUMMYJOB which
; re-enables interrupts (RELINT) and allows normal operations to begin.
; POSTJUMP performs bank switch since DUMMYJOB resides in another memory bank.

ENDRSTRT	TC	POSTJUMP	# NOW IN ANOTHER BANK.
		CADR	DUMMYJOB +2	# PICKS UP AT RELINT.	(DON'T ZERO NEWJOB)

; ============================================================================
; MR.KLEAN - Phase Table Cleanup Subroutine
;
; Clears program phase tables (-PHASE1 through -PHASE6) by loading NEG0
; (negative zero) into each. These phase tables track the execution state
; of major programs (P00-P99, V37). Multiple entry points allow selective
; clearing: MR.KLEAN clears all, P00KLEAN skips -PHASE2, V37KLEAN skips
; -PHASE2 and -PHASE4. Used during program termination and mode changes.
; ============================================================================

MR.KLEAN	INHINT
# Page 214
		EXTEND
		DCA	NEG0
		DXCH	-PHASE2

; P00KLEAN entry: Clears all phase tables except -PHASE2 (preserves P00 state)

P00KLEAN	EXTEND
		DCA	NEG0
		DXCH	-PHASE4

; V37KLEAN entry: Clears remaining phase tables (preserves P00 and current program)

V37KLEAN	EXTEND
		DCA	NEG0
		DXCH	-PHASE1
		EXTEND
		DCA	NEG0
		DXCH	-PHASE3
		EXTEND
		DCA	NEG0
		DXCH	-PHASE5
		EXTEND
		DCA	NEG0
		DXCH	-PHASE6
		TC	Q

# Page 215
; ============================================================================
; TRANSITION: From Fresh Start to Restart Recovery
;
; The computer now enters GOPROG, the restart recovery entry point. When the
; AGC encounters a fault (power transient, watchdog timeout, or operator-
; initiated restart), control transfers here from location 4000. During 
; Apollo 11's lunar landing, this code executed repeatedly to recover from 
; the 1202 program alarms at mission time 102:38:26. The restart system 
; preserved guidance state, allowing the landing to continue safely despite 
; computational overload from radar data processing.
; ============================================================================

; GOPROG - Restart Recovery Entry Point
;
; Called from location 4000 (GOJAM) when a restart occurs. Restores system 
; state from restart tables (defined in RESTART_TABLES.agc) and resumes 
; program execution at protected restart points. Critical for fault tolerance.
;
; The restart system divides the AGC's computational tasks into "restart 
; groups" - logical phases of work that can be safely restarted. Each group
; has a restart point defined in RESTART_TABLES.agc. When a restart occurs,
; GOPROG identifies which group was executing and jumps to that group's
; restart point to resume work.
;
; During the famous 1202 alarms, this code executed multiple times per second,
; each time recovering the guidance computation state and allowing the landing
; program to continue. Flight controllers Steve Bales and Jack Garman 
; recognized the alarms as non-critical restart recoveries, not guidance 
; failures, leading to the "Go" decision that saved the landing.

# COMES HERE FROM LOCATION 4000, GOJAM, RESTART ANY PROGRAMS WHICH MAY HAVE BEEN RUNNING AT THE TIME.

		EBANK=	LST1
; Increment restart counter (REDOCTR). This tracks restart frequency for
; ground monitoring. Excessive restart rate indicates hardware problems
; requiring mission abort consideration.

GOPROG		INCR	REDOCTR		# ADVANCE RESTART COUNTER.

		LXCH	Q
		EXTEND
		ROR	SUPERBNK
		DXCH	RSBBQ
		CA	DSPTAB +11D
		MASK	BIT4
		EXTEND
		BZF	+4
		AD	BIT6		# SET ERROR COUNTER ENABLE
		EXTEND
		WOR	CHAN12		# ISS WAS IN COARSE ALIGN SO GO BACK TO
; Set DSKY indicator lamps based on system state. This routine (LIGHTSET)
; updates the display to show crew the current IMU status, program alarms,
; and attitude reference availability after restart recovery.

BUTTONS		TC	LIGHTSET

; ============================================================================
; Erasable Memory Integrity Check (ERASCHK Protection)
;
; Before continuing with restart, verify erasable memory integrity. The
; ERASCHK routine (used during memory tests) temporarily stores memory
; contents in SKEEP5/SKEEP6 locations and sets ERESTORE to track the
; test address. If a restart interrupts ERASCHK, ERESTORE will contain
; the test address (positive number < 2000 octal). Otherwise ERESTORE = +0.
;
; This check prevents restarting with corrupted erasable memory. If memory
; appears bad, force a fresh start to re-initialize all variables rather
; than attempting to continue with potentially invalid guidance state.
; ============================================================================

# ERASCHK TEMPORARILY STORES THE CONTENST OF TWO ERASABLE LOCATIONS, X
# AND X+1 INTO SKEEP5 AND SKEEP6.  IT ALSO STORES X INTO SKEEP7 AND
# ERESTORE.  IF ERASCHK IS INTERRUPTED BY A RESTART, C(ERESTORE) SHOULD
# EQUAL C(SKEEP7), AND SHOULD BE A + NUMBER LESS THAN 2000 OCT.  OTHERWISE
# C(ERESTORE) SHOULD EQUAL +0.

; Check if ERESTORE contains valid address (bits 11-15 should be zero for
; valid erasable address < 2000 octal). HI5 masks upper 5 bits.

		CAF	HI5
		MASK	ERESTORE
		EXTEND
		BZF	+2		# IF ERESTORE NOT = +0 OR +N LESS THAN 2K,
		TCF	NONAVKEY +3	# DO FRESH START -- E MEMORY MIGHT BE BAD

; ERESTORE is valid format. Check if it equals +0 (normal case - no ERASCHK
; was interrupted) or contains an address (ERASCHK was interrupted).

		CS	ERESTORE
		EXTEND
		BZF	DORSTART	# = +0 CONTINUE WITH RESTART.

; ERESTORE contains an address. Verify it matches SKEEP7 (cross-check that
; both safekeeping locations agree, ensuring data integrity).

		AD	SKEEP7
		EXTEND
		BZF	+2		# = SKEEP7, RESTORE E MEMORY.
		TCF	NONAVKEY +3	# DO FRESH START -- E MEMORY MIGHT BE BAD

; Memory integrity verified. Restore the two erasable locations that were
; under test when ERASCHK was interrupted. This prevents data loss during
; memory validation routines.

		CA	SKEEP4
		TS	EBANK		# EBANK OF E MEMORY THAT WAS UNDER TEST.
		EXTEND			# (NOT DXCH SINCE THIS MIGHT HAPPEN AGAIN)
		DCA	SKEEP5
		INDEX	SKEEP7
		DXCH	0000		# E MEMORY RESTORED
		CA	ZERO
		TS	ERESTORE

; Erasable memory restored successfully. Continue with restart initialization.
; STARTSUB performs common initialization for both fresh start and restart,
; setting up display, navigation, and control system default states.

DORSTART	TC	STARTSUB	# DO INITIALIZATION AFTER ERASE RESTORE.

; ============================================================================
; SETINFL - Set Indicator Flags and Lamps After Restart
;
; After restart recovery completes, configure DSKY lamps and IMU modes to
; reflect the current spacecraft state. Critical lamps (PROG ALARM, GIMBAL 
; LOCK, NO ATT) remain lit if they were on before the restart, ensuring crew
; awareness of ongoing conditions. This prevents the restart from clearing
; warnings that still apply to the mission situation.
;
; During Apollo 11's landing, when 1202 alarms triggered restarts, this code
; preserved the PROG lamp illumination so Armstrong and Aldrin remained aware
; of the alarm condition even as the guidance system recovered and continued
; operating.
; ============================================================================

; Clear internal flag bit (INTFLBIT) in FLAGWRD10. This resets the "inflight
; flag" that distinguishes in-flight operations from ground testing modes.

SETINFL		CS	INTFLBIT
		MASK	FLGWRD10
		TS	FLGWRD10
# Page 216

; Preserve critical DSKY lamps across restart. Bits 9, 6, 4 select PROG ALARM,
; GIMBAL LOCK, and NO ATT lamps respectively. These lamps indicate conditions
; that persist across restarts and must remain visible to crew.

		CA	9,6,4		# LEAVE PROG ALARM, GIMBAL LOCK, NO ATT
		MASK	DSPTAB +11D	# LAMPS INTACT ON HARDWARE RESTART
		AD	BIT15
		XCH	DSPTAB +11D
; Configure IMU (Inertial Measurement Unit) failure inhibits. IFAILINH masks
; bits that prevent spurious failure indications during restart transients.
; Preserve these inhibits while resetting actual failure codes. The IMU
; provides critical navigation data (gyros and accelerometers), and failure
; indications must be accurate - neither false alarms nor missed faults.

		CAF	IFAILINH	# LEAVE IMU FAILURE INHIBITS INTACT ON
		MASK	IMODES30	# HARDWARE RESTART, RESET ALL FAILURE
		AD	IM30INIR	# CODES.
		TS	IMODES30

; Ensure telemetry downlist matches AGS (Abort Guidance System) configuration.
; AGSWORD indicates if AGS is operational. The correct downlist format ensures
; Mission Control receives appropriate data during different mission phases.

		CA	AGSWORD		# BE SURE OF CORRECT DOWNLIST
		TS	DNLSTCOD

; ============================================================================
; Engine State Restoration After Restart
;
; Restore descent engine state based on pre-restart configuration. If the
; descent engine was firing when restart occurred (ENGONBIT set in FLAGWRD5),
; re-enable it. Otherwise, ensure it remains off. This prevents restarts
; during critical powered flight from inadvertently shutting down the engine.
;
; During lunar landing, restarts must NOT interrupt descent engine thrust.
; Loss of thrust during final approach would cause uncontrolled descent and
; mission failure. This code ensures continuous thrust across restart events.
; ============================================================================

; Enable throttle counter and thrust drive hardware interface (CHAN14 bit 4).
; The throttle counter tracks engine command timing for smooth throttle changes.

		CA	BIT4		# TURN ON THROTTLE COUNTER
		EXTEND
		WOR	CHAN14		# TURN ON THRUST DRIVE

; Check if engine was on before restart (ENGONBIT flag in FLAGWRD5). This flag
; is set when descent engine ignition occurs and cleared at engine shutdown.

		CS	FLAGWRD5
		MASK	ENGONBIT
		CCS	A
		TCF	+5

; Engine was ON before restart. Command engine ON via DSALMOUT (discrete alarm
; output channel). Bit 13 controls descent engine on/off discrete signal.

		CAF	BIT13
		EXTEND
		WOR	DSALMOUT	# TURN ENGINE ON
		TCF	GOPROG3

; Engine was OFF before restart. Command engine OFF via DSALMOUT bit 14.
; This ensures engine state consistency across restart boundary.

 +5		CAF	BIT14
		EXTEND
		WOR	DSALMOUT	# TURN ENGINE OFF
		TCF	GOPROG3

; ============================================================================
; ENEMA - Emergency Fresh Start Entry Point
;
; Alternative entry point for system initialization, typically used by ground
; commands or diagnostic programs. The name "ENEMA" is programmer humor - this
; entry performs a "clean-out" of system state similar to a fresh start but
; preserves certain configuration data.
; ============================================================================

ENEMA		INHINT
		TC	STARTSB1
		TCF	GOPROG2A

; GOPROG2 - Continue restart processing with alternate initialization path.
; Calls STARTSB2 subroutine for secondary initialization tasks.

GOPROG2		TC	STARTSB2

; GOPROG2A - Set DSKY indicator lamps and clear restart-related flag bits.

GOPROG2A	TC	LIGHTSET

; Clear restart-specific flag bits (bits 7 and 14 in FLGWRD10). These bits
; track restart processing state and must be cleared before resuming normal
; program execution to prevent restart logic from executing again.

		CS	RSFLGBTS	# CLEAR BITS 7 AND 14.
		MASK	FLGWRD10
		TS	FLGWRD10

; ============================================================================
; GOPROG3 - Phase Table Integrity Verification
;
; Before allowing programs to resume execution after restart, verify that
; the phase tables are internally consistent. Phase tables (defined in
; PHASE_TABLE_MAINTENANCE.agc) track which phase of each program was
; executing when restart occurred. Each phase is stored twice - once as
; the phase value, once as its complement. This redundancy detects memory
; corruption.
;
; The AGC uses "complementary storage" for critical data: store both the
; value and its complement, then verify they are exact opposites. If memory
; corruption occurs, the value and complement will not match, indicating
; unreliable data. This check prevents restarting programs with corrupted
; phase information, which would cause incorrect program behavior.
; ============================================================================

; Loop through all restart groups (NUMGRPS = number of phase table entries).
; For each group, verify phase and complement-phase agree.

GOPROG3		CAF	NUMGRPS		# VERIFY PHASE TABLE AGREEMENTS
PCLOOP		TS	MPAC +5
		DOUBLE
		EXTEND
		INDEX	A

; Load phase value into A (complemented by DCA instruction), and direct
; phase value into L. The -PHASE1 table contains complement-stored phases.

		DCA	-PHASE1		# COMPLEMENT INTO A, DIRECT INTO L.
		EXTEND

; RXOR (Register XOR) compares A and L. For complementary storage, result
; must be -0 (all bits set = 177777 octal). Any other result indicates
; memory corruption in phase table.

		RXOR	LCHAN		# RESULT MUST BE -0 FOR AGREEMENT.

; CCS (Count, Compare, Skip) tests result. If phase and complement don't match
; (result != -0), branch to PTBAD. The CCS instruction branches based on sign:
; +, +0, -, -0. Three TCF PTBAD instructions handle first three conditions.

		CCS	A
		TCF	PTBAD		# RESTART FAILURE.
		TCF	PTBAD
		TCF	PTBAD
# Page 217

; If result is -0 (correct), continue to next restart group. MPAC+5 counts
; down through all groups.

		CCS	MPAC +5		# PROCESS ALL RESTART GROUPS.
		TCF	PCLOOP

; All phase tables verified successfully. Set MPAC+6 to +0 to track whether
; any restart groups are active (incremented later if groups running).

		TS	MPAC +6		# SET TO +0.

; Display current major mode on DSKY. MMDSPLAY shows MM XX on display register
; R1. This confirms to crew that restart completed and displays active program.

		TC	MMDSPLAY	# DISPLAY MAJOR MODE

		INHINT			# RELINT DONE IN MMDSPLAY

; ============================================================================
; Post-Restart Program State Reinitialization
;
; After successful restart, clear certain program flags to force programs
; to reinitialize themselves. This ensures programs don't resume with stale
; state that may be inconsistent after the restart event.
; ============================================================================

; Clear DIDFLAG (Display Interface Flag). R10 display formatting routine will
; re-initialize itself if it was operating during restart. Forces fresh setup
; of DSKY display interface state.

		CS	DIDFLBIT	# CLEAR DIDFLAG IN ORDER TO FORCE R10 TO
		MASK	FLAGWRD1	# RE-INITIALIZE ITSELF IF IT HAD BEEN
		TS	FLAGWRD1	# OPERATION AT THE TIME OF THE RESTART.

; Clear RODFLAG (Rate-Of-Descent Flag). P66 (LM rate-of-descent monitoring
; program during landing) will re-initialize and continue if it was active.
; Ensures rate monitoring resumes with correct state after restart.

		CS	RODFLBIT	# CLEAR RODFLAG.  IF P66 IS IN OPERATION
		MASK	FLAGWRD1	#	IT WILL RE-INITIALIZE ITSELF AND
		TS	FLAGWRD1	#	CONTINUE.

; Clear P21FLAG. P21 (ground track determination program) will compute new
; base state vectors rather than using pre-restart values. Ensures navigation
; state is freshly computed after restart.

		CS	P21FLBIT	# CLEAR P21 FLAG SO THAT P21 WILL COMPUTE
		MASK	FLAGWRD0	# NEW BASE STATE VECTORS.
		TS	FLAGWRD0

; ============================================================================
; Active Restart Group Processing
;
; Scan all restart groups to find which programs were executing when restart
; occurred. For each active group, restart the program at its saved phase.
; The AGC can manage multiple programs simultaneously through restart groups.
; Each group tracks one program's execution state across restart events.
;
; Phase values encode program state: Positive phase = active program that must
; be restarted. +0 or negative phase = inactive group with no program running.
; ============================================================================

; Loop through all restart groups (NUMGRPS total). Check each group's phase
; to determine if program was active.

		CAF	NUMGRPS		# SEE IF ANY GROUPS RUNNING.
NXTRST		TS	MPAC +5
		DOUBLE

; Load phase value using indexed addressing. CCS tests sign of phase.

		INDEX	A
		CCS	PHASE1
		TCF	PACTIVE		# PNZ -- GROUP ACTIVE.
		TCF	PINACT		# +0 -- GROUP NOT RUNNING.

; ============================================================================
; PACTIVE - Restart Active Program Group
;
; An active restart group has been found. The positive phase value indicates
; which phase the program was in when restart occurred. Call RESTARTS routine
; (via SWCALL) to resume program execution at the saved phase point.
;
; During Apollo 11 landing, when 1202 alarms triggered restarts, the descent
; guidance program (in one restart group) was restarted at its current phase,
; allowing continuous guidance computation despite the restart event.
; ============================================================================

PACTIVE		TS	MPAC
		INCR	MPAC		# ABS OF PHASE.

; Set flag indicating at least one restart group has active program. MPAC+6
; was initialized to +0, incremented here for each active group found.

		INCR	MPAC +6		# INDICATE GROUP DEMANDS PRESENT.

; Call RESTARTS routine to resume program at saved phase. SWCALL performs
; bank call with automatic return to SWRETURN after restart processing.

		CA	RACTCADR
		TC	SWCALL		# MUST RETURN TO SWRETURN.

; ============================================================================
; PINACT - Process Inactive Restart Group
;
; This restart group has no active program (phase = +0). Continue checking
; remaining groups. After processing all groups, if no active programs found,
; system enters idle state displaying P00 (Program 00).
; ============================================================================

PINACT		CCS	MPAC +5		# PROCESS ALL RESTART GROUPS.
		TCF	NXTRST

; All restart groups processed. Check if any active programs were found.
; MPAC+6 = 0 means no programs active, proceed to idle handling.

		CCS	MPAC +6		# NO, CHECK PHASE ACTIVITY FLAG
		TCF	ENDRSTRT	# PHASE ACTIVE

; No active programs found. Check if system mode is -0 (idle mode). If so,
; restart complete. Otherwise, jump to GOTOPOOH to enter idle state.

		CAF	BIT15		# IS MODE -0
		MASK	MODREG
		EXTEND
		BZF	GOTOPOOH	# NO
		TCF	ENDRSTRT	# YES

; ============================================================================
; PTBAD - Phase Table Corruption Detected
;
; Phase table integrity check failed (phase and complement don't match). This
; indicates memory corruption. Trigger alarm 1107 and perform complete fresh
; start to re-initialize all memory. Restart cannot proceed safely with
; corrupted phase tables, as this would resume programs at incorrect points.
; ============================================================================

PTBAD		TC	ALARM		# SET ALARM TO SHOW PHASE TABLE FAILURE.
		OCT	1107

; After alarm, perform fresh start sequence (DOFSTRT1). This completely
; reinitializes erasable memory, clearing all program state and starting fresh.

		TCF	DOFSTRT1
#******** ****** ******
# Page 218
# DO NOT USE GOPROG2 OR ENEMA WITHOUT CONSULTING POOH PEOPLE.

; ============================================================================
; Restart Initialization Constants
;
; Constants used during fresh start and restart initialization. These values
; configure DAP (Digital Autopilot), RCS control system, and attitude control
; parameters to safe initial states.
; ============================================================================

OCT10000	=	BIT13
OCT30000	=	PRIO30
OCT7777		OCT	7777

; DAP control rate initialization. 20 degrees/second maximum companded rate
; for attitude control during fresh start. Ensures gentle initial control.

STIKSTRT	DEC	0.825268	# 20 D/S MAXIMUM COMPANDED RATE
RATESTRT	DEC	-218

; Address of RESTARTS routine in restart tables. Used by PACTIVE to call
; program restart logic for active restart groups.

RACTCADR	CADR	RESTARTS

; Boolean flags initial state. Configures system operational mode flags.

BOOLSTRT	OCT	21312

; Attitude angle constant: 0.14 degrees scaled at 4.5 degrees per bit.
; Used for fine attitude control threshold during initialization.

77001OCT	OCT	77001		# .14 DEG SCALED AT 4.5 DEG
60DEC		DEC	60

; Restart flag bits: Configures which restart protection features are enabled.

RSFLGBTS	OCT	20100

; Maximum deadband: 5 degrees attitude deadband scaled at 45 degrees. Defines
; acceptable attitude error range before DAP commands corrective thruster firing.

MAXDB		OCTAL	03434		# 5 DEG ATTITUDE DEADBAND, SCALED AT 45.

; ============================================================================
; LIGHTSET - Crew-Initiated Fresh Start via DSKY Keys
;
; Astronaut can initiate fresh start by simultaneously pressing Mark Reject
; and Error Reset keys on DSKY. This provides crew override capability to
; force complete system reinitialization if guidance computer behavior becomes
; problematic. Detects key combination and branches to fresh start sequence.
; ============================================================================

LIGHTSET	CAF	BIT5		# CHECK FOR MARK REJECT AND ERROR RESET

; Read navigation keyboard input to check for Mark Reject key.

		EXTEND
		RAND	NAVKEYIN
		EXTEND
		BZF	NONAVKEY	# NO MARK REJECT

; Mark Reject pressed. Check if Error Reset (keys 2M and 5M) also pressed.

		EXTEND
		READ	MNKEYIN		# CHECK IF KEYS 2M AND 5M ON
		AD	-ELR		# MAIN DSKY KEYCODE (BITS 1-5)
		EXTEND
		BZF	+2

; Key combination not detected. Return to caller without fresh start.

NONAVKEY	TC	Q

; Crew-commanded fresh start detected. Call STARTSUB to initialize downlink
; and telemetry, then branch to full fresh start (DOFSTART) or fresh start
; without engine reset (DOFSTRT1).

		TC	STARTSUB
		TCF	DOFSTART
 +3		TC	STARTSUB
		TCF	DOFSTRT1	# DO FRESH START BUT DON'T TOUCH ENGINE

# Page 219
# INITIALIZATION COMMON TO BOTH FRESH START AND RESTART.

; ============================================================================
; STARTSUB - Common Initialization for Fresh Start and Restart
;
; This critical subroutine performs initialization tasks shared by both cold
; boot (fresh start) and fault recovery (restart). Called after restart logic
; determines system state, this routine prepares downlink telemetry, resets
; timing counters, initializes radar modes, and sets up display system.
;
; COMMENT-ONLY READERS: The guidance computer is preparing to communicate with
; Mission Control and reinitialize timing systems for mission operations.
;
; CODE-ALONG READERS: Subroutine uses TC linkage with return via Q register.
; Initializes downlink interrupt pointer, timing registers TIME3/TIME4/TIME5,
; radar mode configuration, and AGC-to-LGC mode flags.
; ============================================================================

		EBANK=	AOSQ
; Downlink telemetry communication with Mission Control is critical for
; monitoring spacecraft state. This initialization ensures the downlink
; begins transmitting from a known starting point in the current data list.
STARTSUB	CAF	LDNPHAS1	# SET POINTER SO NEXT 20MS DOWNRUPT WILL
		TS	DNTMGOTO	# CAUSE THE CURRENT DOWNLIST TO BE
					# INTERRUPTED AND START SENDING FROM THE
					# BEGINNING OF THE CURRENT DOWNLIST.
		
; Initialize radar mode configuration by reading hardware switch position
; (CHAN33 BIT6) and combining with software initialization value. Landing
; radar and rendezvous radar modes controlled through this setting.
		CAF	BIT6
		EXTEND
		RAND	CHAN33		# Read radar mode select switch
		AD	RMODINIT	# Add software initialization bits
		TS	RADMODES	# Store combined radar mode configuration

; ============================================================================
; Timing System Initialization
;
; TIME3, TIME4, TIME5 registers control waitlist timing and task scheduling.
; These are set to staggered maximum values (POSMAX, POSMAX-2, POSMAX-1) to
; initialize timing chains. The AGC's timing system is critical for all
; time-dependent operations including guidance updates and crew displays.
; ============================================================================
STARTSB1	CAF	POSMAX		# Initialize timing registers to maximum
		TS	TIME3		# TIME3 = +37777 (maximum positive value)
		AD	MINUS2
		TS	TIME4		# TIME4 = +37775 (slightly offset)
		AD	NEGONE
		TS	TIME5		# TIME5 = +37774 (further offset)

		CAF	EBANK6		# Set erasable memory bank for access to
		TS	EBANK		# DAP and control system variables

; ============================================================================
; Digital Autopilot (DAP) Initialization
;
; The DAP controls spacecraft attitude using RCS thrusters. During restart,
; the DAP must be carefully reinitialized to prevent erratic thruster firing.
; This section disables TIME6 interrupt (thruster firing scheduler), clears
; the accelerometer calibration flag, and sets up the DAPIDLER background task.
;
; COMMENT-ONLY READERS: The attitude control system is being reset to ensure
; thrusters don't fire unexpectedly during system recovery.
; ============================================================================
		CS	BIT13		# CAUSE DAPIDLER TO CALL 1/ACCS
		MASK	RCSFLAGS
		TS	RCSFLAGS	# ZERO BIT 13 (clear accel cal flag)
		CAF	POSMAX		# DISABLE TIME6 CLOCK.  JUST IN CASE A T6
		TS	T6NEXT		#	RUPT IS ALREADY IN THE PRIORITY CHAIN,
		EXTEND			#	ENSURE THAT ITS INPUTS WILL RENDER IT
		WAND	CHAN13		#	INEFFECTUAL (mask TIME6 enable).
		CAF	ZERO
		TS	NXT6ADR		# Clear TIME6 task address
		TS	NEXTP		# Clear next task priority

		CS	ACCSOKAY	# Clear accelerometer calibration OK flag
		MASK	DAPBOOLS	# (accelerometers must be recalibrated
		TS	DAPBOOLS	# after any restart event)

		EXTEND			# SET T5RUPT FOR DAPIDLER PROGRAM.
		DCA	IDLEADR		# Load DAPIDLER address (idle DAP task)
		DXCH	T5ADR		# Install in T5 interrupt vector

; ============================================================================
; Hardware I/O Channel and Software Flag Restoration
;
; During restart, certain hardware channels and software flags must be
; carefully managed. Engine control, IMU (gyroscope) settings, and radar
; flags are selectively preserved or cleared to ensure safe recovery without
; disrupting critical spacecraft systems mid-operation.
;
; COMMENT-ONLY READERS: The computer is making sure engine commands and
; navigation system settings remain safe during the recovery process.
; ============================================================================
STARTSB2	CAF	OCT30001	# DURING SOFTWARE RESTART, DO NOT DISTURB
		EXTEND			# ENGINE ON, OFF AND ISS WARNING.
		WAND	DSALMOUT	# Preserve engine state and alarm flags

		CS	READRBIT	# CLEAR READRFLG FOR R29
		MASK	FLAGWRD3	# (R29 is rendezvous tracking routine)
		TS	FLAGWRD3
# Page 220

; Clear radar mode flags for tasks that were terminated by the restart.
; If R29 (rendezvous navigation) was active, clear its radar designation
; state so it doesn't resume with stale target data.
		CS	FLAGWRD3	# DURING SOFTWARE RESTART, CLEAR TURNON,
		MASK	NR29FBIT	# REPOSITION, CDU ZERO AND REMODE BITS
		EXTEND			# IN RADMODES, SINCE TASKS ASSOCIATED
		BZF	+2		# WITH THESE BITS HAVE BEEN KILLED
		CAF	BIT10		# ALSO IF R29 HAD BEEN REQUESTED.
		AD	OCT32001	# (NOR29FLG = 0) CLEAR BIT 10 RADMODES
		COM			# TO MAKE R29 FORGET IT HAD STARTED
		MASK	RADMODES	# DESIGNATING (clear radar designation bits)
		TS	RADMODES
; Preserve IMU (Inertial Measurement Unit) state during restart. The IMU
; provides spacecraft attitude information through gyroscopes. Its alignment
; state must not be disturbed or the spacecraft loses attitude reference.
		CAF	OCT27470	# DURING SOFTWARE RESTART, DO NOT DISTURB
		EXTEND			# IMU FLAGS.  (COARSE ALIGN ENABLE, ZERO
		WAND	CHAN12		# IMU CDUS, ENABLE IMU COUNTER) AND GIMBAL
					# TRIM DRIVES.  LEAVE RR LOCKON ENABLE
					# ALONE (preserve IMU alignment state).

		CS	NORRMBIT	# ENABLE R25 (rendezvous radar test).
		MASK	FLAGWRD5
		TS	FLAGWRD5

		CS	R77FLBIT	# CLEAR R77FLAG
		MASK	FLAGWRD5	# (R77 is IMU performance test routine)
		TS	FLAGWRD5
		
; Preserve telemetry and interrupt configuration. CHAN13 controls several
; critical interrupt enables and status flags that must remain stable.
		CAF	OCT74160	# DURING SOFTWARE RESTART, DO NOT DISTURB
		EXTEND			# TELEMETRY FLAGS, RESET TRAP FLAGS, AND
		WAND	CHAN13		# ENABLE T6RUPT FLAG (preserve state).

		CAF	BIT12		# REENABLE RUPT10 (RUPT QUICKLY
		EXTEND			#	RESUMES EXCEPT DURING P64 landing)
		WOR	CHAN13		# Enable RUPT10 (10-second time interrupt)

		CAF	BIT6		# DURING SOFTWARE RESTART, DO NOT DISTURB
		EXTEND			# GYRO ENABLE FLAG (preserve gyro state).
		WAND	CHAN14

; ============================================================================
; Waitlist Initialization - Timer Task Scheduling
;
; The WAITLIST is the AGC's timer-driven task scheduler. It maintains a
; delta-time queue of tasks to be executed at future times. LST1 holds
; delta-times between tasks, LST2 holds task addresses. Initializing these
; to known states (NEG1/2 for times, ENDTASK marker for addresses) ensures
; the waitlist starts empty and stable.
;
; COMMENT-ONLY READERS: The computer's timing system that schedules future
; tasks is being cleared and reset to a safe, empty state.
;
; CODE-ALONG READERS: NEG1/2 = negative maximum time, indicating infinite
; delay (no pending tasks). ENDTASK is sentinel value marking empty slots.
; This was critical during Apollo 11's 1202 alarm - waitlist overflow
; contributed to the alarm, so restart must cleanly reinitialize it.
; ============================================================================
		EBANK=	LST1
		CAF	STARTEB
		TS	EBANK		# SET FOR E3 (erasable bank 3 access)

		CAF	NEG1/2		# INITIALIZE WAITLIST DELTA-TS.
		TS	LST1 +7		# LST1 = delta-time queue
		TS	LST1 +6		# Initialize all 8 slots to NEG1/2
		TS	LST1 +5		# (maximum negative = infinite delay)
		TS	LST1 +4
		TS	LST1 +3
		TS	LST1 +2
		TS	LST1 +1
		TS	LST1		# All waitlist slots now empty

		CS	ENDTASK		# ENDTASK marker indicates empty slot
		TS	LST2		# LST2 = task address queue
# Page 221
		TS	LST2 +2		# Initialize all task address slots
		TS	LST2 +4		# with ENDTASK sentinel values
		TS	LST2 +6
		TS	LST2 +8D
		TS	LST2 +10D
		TS	LST2 +12D
		TS	LST2 +14D
		TS	LST2 +16D	# (even words = high-order address)
		CS	ENDTASK +1
		TS	LST2 +1		# (odd words = low-order address + bank)
		TS	LST2 +3
		TS	LST2 +5
		TS	LST2 +7
		TS	LST2 +9D
		TS	LST2 +11D
		TS	LST2 +13D
		TS	LST2 +15D
		TS	LST2 +17D	# Waitlist now completely empty

; ============================================================================
; Executive Job Scheduler Initialization
;
; The EXECUTIVE manages cooperative multitasking through "jobs" running at
; different priority levels. The AGC has 7 priority levels (priority 12
; through 77 octal), each with its own register set (core set). Initializing
; PRIORITY table entries to negative zero marks all core sets as available.
;
; COMMENT-ONLY READERS: The computer's job scheduling system is being reset,
; making all task execution slots available for new mission programs.
;
; CODE-ALONG READERS: Each PRIORITY entry represents one core set. CS ZERO
; produces negative zero (all ones), which marks the core set as unused.
; During Apollo 11's 1202 alarm, executive overflow meant no free core sets
; were available. Restart recovery must ensure clean initialization.
; ============================================================================
		CS	ZERO		# MAKE ALL EXECUTIVE REGISTER SETS
		TS	PRIORITY	# AVAILABLE. (negative zero = unused)
		TS	PRIORITY +12D	# Priority 12 through 77 octal
		TS	PRIORITY +24D	# (7 priority levels)
		TS	PRIORITY +36D	# Each 12D offset = one core set
		TS	PRIORITY +48D	# containing 12 registers
		TS	PRIORITY +60D	# (A, L, Q, EB, FB, Z, BB, etc.)
		TS	PRIORITY +72D
		TS	PRIORITY +84D	# All 7 core sets now available

		TS	DSRUPTSW	# Display system interrupt switch
		TS	NEWJOB		# SHOWS NO ACTIVE JOBS.

; ============================================================================
; VAC (Vector Accumulator) Area Initialization
;
; The AGC provides 5 VAC areas for interpretive language programs to use as
; working storage for vector and matrix operations. Each VAC area must be
; marked available by setting its USE flag to point to its own address.
; LTHVACA = length of VAC area, used to compute successive VAC addresses.
;
; COMMENT-ONLY READERS: Temporary memory storage areas used for navigation
; and guidance calculations are being cleared and made available.
;
; CODE-ALONG READERS: VAC areas are 43-word blocks in erasable memory used
; by interpretive programs for pushdown stack operations. Setting VACxUSE
; to VACxADRC marks area as available. The interpreter checks these flags
; before allocating VAC space for vector/matrix computations.
; ============================================================================
		CAF	VAC1ADRC	# MAKE ALL VAC AREAS AVAILABLE.
		TS	VAC1USE		# VAC1 address → VAC1USE (marks available)
		AD	LTHVACA		# Add VAC length to get next VAC address
		TS	VAC2USE		# VAC2 address → VAC2USE
		AD	LTHVACA		# Continue chain to mark all VACs available
		TS	VAC3USE		# VAC3 address → VAC3USE
		AD	LTHVACA
		TS	VAC4USE		# VAC4 address → VAC4USE
		AD	LTHVACA
		TS	VAC5USE		# VAC5 address → VAC5USE (all 5 VACs ready)

; ============================================================================
; DSKY Display Table Clearing
;
; The DSPTAB (Display Table) contains data displayed on the DSKY (Display
; and Keyboard unit). This loop clears 11 display registers (R1, R2, R3
; across multiple display channels) by writing negative BIT12 to each entry.
; The countdown loop starts at TEN (octal 10 = decimal 8) and decrements.
;
; COMMENT-ONLY READERS: The astronaut's display panel is being cleared,
; erasing any previous numbers shown on the DSKY readout.
;
; CODE-ALONG READERS: MPAC used as loop counter (10 down to 0). CS BIT12
; produces sign control bits to blank displays. INDEX MPAC enables indexed
; addressing to step through DSPTAB entries. CCS MPAC decrements counter
; and tests for zero to control loop exit.
; ============================================================================
		CAF	TEN		# Loop counter (11 iterations)
DSPOFF		TS	MPAC		# R1, R2, R3 display registers
		CS	BIT12		# Sign bit pattern to clear display
		INDEX	MPAC		# Indexed addressing into DSPTAB
		TS	DSPTAB		# Clear this display table entry
		CCS	MPAC		# Count-Compare-Skip: decrement and test
		TCF	DSPOFF		# Loop until all displays cleared

; ============================================================================
; Additional System Variable Initialization
;
; Final initialization of various system control variables used by display
; routines, verb/noun processing, and keyboard input handling.
; ============================================================================
# Page 222
		TS	DELAYLOC	# Display delay timing
		TS	DELAYLOC +1
		TS	DELAYLOC +2
		TS	R1SAVE		# Display register save area
		TS	INLINK		# Keyboard input link
		TS	DSPCNT		# Display counter
		TS	CADRSTOR	# Return address storage
		TS	REQRET		# Request return flag
		TS	CLPASS		# Clear pass counter
		TS	DSPLOCK		# Display lock flag
		TS	MONSAVE		# KILL MONITOR (clear display monitoring)
		TS	MONSAVE1	# Monitor save continued
		TS	VERBREG		# VERB register (no active verb)
		TS	NOUNREG		# NOUN register (no active noun)
		TS	DSPLIST		# Display list pointer
		TS	MARKSTAT	# Mark status (optical tracking)
		TS	EXTVBACT	# MAKE EXTENDED VERBS AVAILABLE
		TS	IMUCADR		# IMU calibration routine address
		TS	OPTCADR		# Optics routine address
		TS	RADCADR		# Radar routine address
		TS	ATTCADR		# Attitude maneuver routine address
		TS	LGYRO		# Last gyro torquing command
		TS	FLAGWRD4	# KILL INTERFACE DISPLAYS

; Initialize NOUT register for astronaut display output control.
		CAF	NOUTCON		# NOUT initialization constant
		TS	NOUT		# Output display mode control

; Configure IMU mode register, preserving critical mode bit while resetting
; PIPA (Pulse Integrating Pendulous Accelerometer) and downlist fail flags.
		CS	ONE		# Sample limit initialization
		TS	SAMPLIM		# (-1 = no sampling limit)
		CAF	BIT6		# Preserve existing mode bit
		MASK	IMODES33	# LEAVE BIT 6 UNCHANGED (important mode state)
		AD	IM33INIT	# NO PIP OR TM FAILS. BIT6=0 IN THIS WORD.
		TS	IMODES33	# IMU modes now initialized cleanly

; Set up self-check return address for AGC self-test diagnostics.
		CAF	LESCHK		# SELF CHECK GO-TO REGISTER.
		TS	SELFRET		# Return point after self-check completes

; Initialize display system timing countdown register.
		CS	VD1		# Display timing constant (negative)
		TS	DSPCOUNT	# Display countdown initialized

; ============================================================================
; STARTSUB Return
;
; STARTSUB has completed all common initialization tasks for both fresh
; starts and restarts. Control returns to the calling routine (SLAP1 for
; fresh start, restart routines for restart recovery).
;
; COMMENT-ONLY READERS: The computer's core systems are now reset and ready.
; Control returns to continue the startup or restart sequence.
;
; CODE-ALONG READERS: TC Q performs subroutine return using Q register,
; which holds the return address set by the calling TC STARTSUB instruction.
; ============================================================================
		TC	Q		# Return to caller (STARTSUB complete)

; ============================================================================
; Constants and Initialization Tables for Fresh Start and Restart
;
; This section defines constants and table entries used by the initialization
; routines above. These include addresses, mode initialization values, and
; default settings for various AGC subsystems.
;
; COMMENT-ONLY READERS: These are preset values the computer uses to
; configure its systems during startup.
;
; CODE-ALONG READERS: Constants are stored in fixed (ROM) memory. GENADR
; generates address with implied bank info. 2CADR is two-word address (bank +
; address). ADRES is simple address within current bank. OCT/DEC specify
; octal/decimal numeric constants.
; ============================================================================
		EBANK=	AOSQ
IDLEADR		2CADR	DAPIDLER	# DAP idler program address (two-word)

IFAILINH	OCT	435		# IMU fail inhibit bit pattern
LDNPHAS1	GENADR	DNPHASE1	# Downlink phase 1 address
LESCHK		GENADR	SELFCHK		# Self-check routine address
VAC1ADRC	ADRES	VAC1USE		# VAC area 1 address constant
OCT32001	OCT	32001		# Initialization constant (bit pattern)
LTHVACA		DEC	44		# Length of VAC area = 44 decimal words

# Page 223
OCT27470	OCT	27470		# Mode initialization constant
OCT74160	OCT	74160		# Mode initialization constant
OCT30001	OCT	30001		# Mode initialization constant
STARTEB		EQUALS	EBANK3		# Erasable bank for startup = bank 3
NUMGRPS		EQUALS	FIVE		# Number of restart groups = 5
-ELR		OCT	-22		# -ERROR LIGHT RESET KEY CODE (DSKY key)
IM30INIF	OCT	37411		# INHIBITS IMU FAIL FOR 5 SEC AND PIP ISSW
IM30INIR	OCT	37000		# IMU mode 30 initialization (restart)
IM33INIT	=	PRIO16		# NO PIP OR TM FAIL SIGNALS (IMU mode 33)
9,6,4		OCT	450		# Radar sampling mode bits (9, 6, 4)
RMODINIT	OCT	00102		# Radar mode initialization (bit 2 set)

; State Vector Initialization Table - 14 words of initial flag states
; Used by SWINIT routine to set up software flag registers during fresh start.
SWINIT		OCT	0		# FLAGWRD0 initialization
		OCT	0		# FLAGWRD1 initialization
		OCT	0		# FLAGWRD2 initialization
		OCT	02000		# BIT 11 = NOR29FLG (flagword 3)
		OCT	0		# FLAGWRD4 initialization
		OCT	0		# FLAGWRD5 initialization
		OCT	0		# FLAGWRD6 initialization
		OCT	00100		# BIT 6 set (flagword 7)

		OCT	0		# FLAGWRD8 initialization
		OCT	0		# FLAGWRD9 initialization
		OCT	0		# FLAGWRD10 initialization
		OCT	40000		# BIT 15 = LRBYPASS (landing radar bypass)

; ============================================================================
; TRANSITION: From Initialization Routines to Major Mode Change Interface
;
; Having completed system initialization (fresh start or restart), the
; AGC is now ready to accept crew commands for major program changes.
; GOTOPOOH requests a return to P00 (idle program) by flashing V37
; (major mode change verb) on the DSKY. V37 is the AGC's mechanism for
; transitioning between major programs during flight operations.
;
; During Apollo 11, the crew used V37 to transition between programs:
; from P20 (rendezvous navigation) to P63 (lunar landing braking phase),
; and from P63 to P00 after landing. This interface was critical for
; managing the mission timeline and workload distribution.
; ============================================================================
# Page 224
# PROGRAM NAME		GOTOPOOH		ASSEMBLY SUNDANCE
# LOG SECTION		FRESH START AND RESTART
#
# FUNCTIONAL DESCRIPTION
#
#	FLASH V 37 ON DSKY MM CHANGE REQUEST
#
# INPUT/OUTPUT INFORMATION
#
#	A. CALLING SEQUENCE			TC GOTOPOOH
#	B. ERASABLE INITIALIZATION		NONE
#	C. OUTPUT FLASH V 37 ON DSKY
#	D. DEBRIS				L
#
# PROGRAM ANALYSIS
#
#	A. SUBROUTINES CALLED			PRIODSPR, LINUS
#	B. NORMAL EXIT				TCF ENDOFJOB
#	C. ALARM AND ABORT EXITS		NONE

		BLOCK	03
		SETLOC	FFTAG5
		BANK

		COUNT*	$$/P00
; Program Change Request - GOTOPOOH
;
; COMMENT-ONLY READERS: This routine requests a return to the idle program
; (P00 or "Pooh") by displaying "V37" flashing on the DSKY. The crew sees
; this request and can accept it by pressing PROCEED, or reject it by
; pressing TERMINATE. This is how the computer asks permission to change
; major programs.
;
; CODE-ALONG READERS: GOTOPOOH sets phase register to 4.33 (group 4, phase 3)
; via -PHASE4 to mark P00 restart point, then jumps to GOP00FIX which
; handles the actual DSKY V37 flash. The phase setting allows restart
; recovery if power is lost during the transition.
GOTOPOOH	CAF	OCT33			# 4.33 SPOT FOR GOP00FIX
		TS	L			# Store in L register
		COM				# Complement to get negative
		DXCH	-PHASE4			# Set phase 4.33 for restart protection

		TC	POSTJUMP		# Bank jump to GOP00FIX
		CADR	GOP00FIX		# (cross-bank address)
; Major mode (MM) constants for program identification
OCT24		MM	20			# Major mode 20
OCT31		MM	25			# Major mode 25

		BANK	20
		SETLOC	VERB37
		BANK

		COUNT*	$$/P00			# VERB 37 AND P00 IN BANK 4.
; GOP00FIX - Go to P00 Fixup Routine
;
; This routine prepares the AGC for transition to P00 (idle program) by
; clearing flags and releasing display systems, then flashing V37 N99 to
; request crew permission for the mode change.
;
; COMMENT-ONLY READERS: Before asking the crew to approve changing to the
; idle program, the computer cleans up by turning off override modes and
; ullage thrusters, and releasing control of the display system. It then
; displays the flashing "V37 N99" request.
;
; CODE-ALONG READERS: Flag clearing via DOWNFLAG ensures clean state.
; XOVINFLG = x-axis override flag (for attitude control). ULLAGFLG =
; ullage thruster flag (small thrusters that settle propellant before
; main engine ignition). CLEARMRK releases optical navigation mark system.
; GOFLASH displays V37 N99 with flashing verb, waiting for crew response.
GOP00FIX	TC	DOWNFLAG		# ALLOW X-AXIS OVERRIDE
		ADRES	XOVINFLG		# (clear X-axis override flag)

		TC	DOWNFLAG		# INSURE THAT ULLAGE IS OFF
		ADRES	ULLAGFLG		# (clear ullage thruster flag)
# Page 225
		TC	CLEARMRK +2		# RELEASE MARK DISPLAY SYSTEM.
		CAF	V37N99			# Load V37 N99 display code
		TC	BANKCALL		# Call display flash routine
		CADR	GOFLASH			# (GOFLASH handles flashing display)
		TCF	-3			# Return to flash again if needed
		TCF	-4			# Return paths from GOFLASH
		TCF	-5			# (handles PROCEED, TERMINATE, etc.)

; V37 N99 Display Code
; V37 = Major mode change request verb
; N99 = Program number to load (entered by crew)
V37N99		VN	3799			# Verb 37, Noun 99

# Page 226
# PROGRAM NAME		V37			ASSEMBLY SUNDANCE
#
# LOG SECTION		FRESH START AND RESTART
#
# FUNCTIONAL DESCRIPTION
#
#	1. CHECK IF NEW PROGRAM ALLOWED.  IF BIT 1 OF FLAGWRD2 (NODOFLAG) IS SET, AN ALARM 1520 IS CALLED.
#	2. CHECK FOR VALIDITY OF PROGRAM SELECTED.  IF AN INVALID PROGRAM IS SELECTED, THE OPERATOR ERROR LIGHT IS
#	   SET AND CURRENT ACTIVITY, IF ANY, CONTINUE.
#	3. SERVICER IS TERMINATED IF IT HAS BEEN RUNNING.
#	4. INSTALL IS EXECUTED TO AVOID INTERRUPTING INTEGRATION.
#	5. THE ENGINE IS TURNED OFF AND THE DAP IS INITIALIZED FOR COAST.
#	6. TRACK AND UPDATE FLAGS ARE SET TO ZERO.
#	7. DISPLAY SYSTEM IS RELEASED.
#	8. THE FOLLOWING ARE PERFORMED FOR EACH OF THE THREE CASES.
#		A. PROGRAM SELECTED IS P00
#			1. RENDEZVOUS AND P25 FLAGS ARE RESET.  (KILL P20 AND P25)
#			2. STATINT1 IS SCHEDULED BY SETTING RESTART GROUP 2.
#			3. MAJOR MODE 00 IS STORED IN THE MODE REGISTER (MODREG).
#			4. SUPERBANK 3 IS SELECTED.
#			5. NODOFLAG IS RESET.
#			6. ALL RESTART GROUPS EXCEPT GROUP2 ARE CLEARED. CONTROL IS TRANSFERRED TO RESTART PROGRAM (GOPROG2)
#			   WHICH CAUSES ALL CURRENT ACTIVITY TO BE DISCONTINUED AND A 9 MINUTE INTEGRATION CYCLE TO BE
#			   INITIATED.
#		B. PROGRAM SELECTES IS P20 OR P25.
#			1. IF THE CURRENT MAJOR MODE IS THE SAME AS THE SELECTED NEWPROGRAM.  THE PROGRAM IS RE-INITIALIZED
#			   VIA V37XEQ, ALL RESTART GROUPS, EXCEPT GROUP 4 ARE CLEARED.
#			2. IF THE CURRENT MAJOR MODE IS NOT EQUAL TO THE NEW REQUEST, A CHECK IS MADE TO SEE IF THE REQUEST-
#			   ED MAJOR MODE HAS BEEN RUNNING THE BACKGROUND,
#			   AND IF IT HAS, NO NEW PROGRAM IS SCHEDULED, THE EXISTING
#			   P20 OR P25 IS RESTARTED TO CONTINUE, AND ITS MM IS SET.
#			3. CONTROL IS TRANSFERRED TO GOPROG2.
#		C. PROGRAM SELECTED IS NEITHER P00, P20, NOR P25
#			1. V37XEQ IS SCHEDULED (AS A JOB) BY SETTING RESTART GROUP 4
#			2. ALL CURRENT ACTIVITY EXCEPT RENDEZVOUS AND TRACKING IS DISCONTINUED BY CLEARING ALL RESTART
#			   GROUPS.  IF THE RENDEZVOUS OR THE P25 FLAG IS ON, GROUP 2 IS NOT CLEARED, ALLOWING THESE PROGRAMS
#			   TO CONTINUE.
#
# INPUT/OUTPUT INFORMATION
#
#	A. CALLING SEQUENCE
#		CONTROL IS DIRECTED TO V37 BY THE VERBFAN ROUTINE.
#		VERBFAN GOES TO C(VERBTAB+C(VERBREG)). VERB 37 = MMCHANG.
#		MMCHANG EXECUTES A `TC POSTJUMP', CADR V37.
#
#	B. ERASABLE INITIALIZATION		NONE
#
# 	C. OUTPUT
# Page 227
#		MAJOR MOD CHANGE
#
#	D. DEBRIS
#		MMNUMBER, MPAC +1, MINDEX, BASETEMP +C(MINDEX), FLAGWRD0, FLAGWRD1, FLAGWRD2, MODREG, GOLOC -1,
#		GOLOC, GOLOC +1, GOLOC +2, BASETEMP, -PHASE2, PHASE2, -PHASE4
#
# PROGRAM ANALYSIS
#
#	A. SUBROUTINES CALLED
#		ALARM, RELDSP, PINBRNCH, INTSTALL, ENGINOF2, ALLCOAST, V37KLEAN, GOPROG2, FALTON, FINDVAC, SUPERSW,
#		DSPMM
#
#	B. NORMAL EXIT				TC ENDOFJOB
#
; ============================================================================
; TRANSITION: From Display Flash Request to Major Mode Change Execution
;
; After GOP00FIX flashes V37 N99 and the crew enters a program number,
; control arrives here at V37 to validate and execute the major mode
; change. This is the AGC's central dispatcher for switching between
; major programs during flight operations.
;
; During Apollo 11's descent on July 20, 1969, the crew used V37 to
; transition from P63 (lunar landing braking phase) to P00 after the
; historic touchdown at 102:45:40 mission time. Throughout the mission,
; V37 was the primary mechanism for crew-initiated program changes:
; from P20 (rendezvous navigation) to P63 (landing), from P00 to P12
; (ascent), and between all major operational modes.
; ============================================================================
#	C. ALARMS				1520 (MAJOR MODE CHANGE NOT PERMITTED)

; V37 - Major Mode Change Routine
;
; COMMENT-ONLY READERS: When the crew enters a program number in response
; to the flashing V37 display, this routine checks if the requested
; program change is allowed and safe. It verifies the IMU is not being
; initialized, checks for special abort programs (P70/P71), validates
; the requested program against a table of permitted transitions, and
; ensures no conflicting operations are in progress. Only after all
; safety checks pass does it actually switch to the new program.
;
; CODE-ALONG READERS: A register contains the requested major mode number
; (from crew DSKY entry). MMNUMBER stores it for subsequent validation.
; RESTREG = restart register set to PRIO30 (priority 30) ensuring restart
; recovery at pinball (display system) priority if power is lost during
; the transition. Multiple validation gates follow before program change.
V37		TS	MMNUMBER		# SAVE MAJOR MODE (from A register)
		CAF	PRIO30			# RESTART AT PINBALL PRIORITY
		TS	RESTREG			# (ensures display system recovery)

; IMU Initialization Check
; The Inertial Measurement Unit (IMU) cannot be disturbed during its
; initialization sequence. If IMU initialization is in progress (BIT6
; of IMODES30 set), the mode change is rejected with alarm 1520.
;
; COMMENT-ONLY READERS: The computer checks if the navigation gyroscope
; platform is being aligned. If so, changing programs now would disrupt
; the delicate alignment process, so the request is rejected.
		CA	IMODES30		# IS IMU BEING INITIALIZED
		MASK	BIT6			# Check initialization flag
		CCS	A			# Is bit set?
		TCF	CANTROD			# Yes, can't change - alarm 1520

; Abort Program Priority Check (P70/P71)
; P70 and P71 are the abort programs for LM ascent. They have special
; priority and bypass normal mode change restrictions. If either is
; requested, immediate setup follows.
;
; COMMENT-ONLY READERS: If the crew requests an abort program (P70 or
; P71), the computer recognizes this as an emergency and immediately
; prepares for the abort sequence, skipping normal validation checks.
		CS	MMNUMBER		# IS P70 REQUESTED?
		AD	DEC70			# Compare with 70
		EXTEND
		BZF	SETUP70			# YES - immediate abort setup
		AD	ONE			# IS P71 REQUESTED?
		EXTEND
		BZF	SETUP71			# YES - immediate abort setup

; P00 (Idle Program) Check
; P00 is the idle/standby program. Transitioning to P00 requires
; checking if the SERVICER routine is active, as it must be allowed
; to complete its current cycle before P00 takes over.
		CA	MMNUMBER		# IS NEW REQUEST P00
		EXTEND
		BZF	ISSERVON		# YES, CHECK SERVICER STATUS

; NODO Flag Check
; The NODO flag (No Do flag) blocks V37 major mode changes when set.
; This protects critical operations from interruption. If NODO is set,
; the mode change is rejected with alarm 1520.
;
; COMMENT-ONLY READERS: Certain critical operations set a "do not
; disturb" flag that prevents program changes. If this flag is set,
; the mode change request is rejected to protect the ongoing operation.
		CS	FLAGWRD2		# NO, IS NODO V37 FLAG SET
		MASK	NODOBIT			# Check NODO bit
		CCS	A			# Is it clear?
		TCF	CHECKTAB		# YES - proceed to table validation
CANTROD		TC	ALARM			# NO - reject with alarm
		OCT	1520			# "MAJOR MODE CHANGE NOT PERMITTED"

; Mode Change Rejected - Display Release
; If the mode change is rejected for any reason, release the display
; system and return to previous state via PINBRNCH.
V37BAD		TC	RELDSP			# RELEASES DISPLAY FROM ASTRONAUT

		TC	POSTJUMP		# BRING BACK LAST NORMAL DISPLAY IF THERE
		CADR	PINBRNCH		# WAS ONE.  OTHERWISE DO AN EOJ.

; Major Mode Table Validation
;
; CHECKTAB validates the requested major mode number by searching through
; a table (PREMM1) of legal program transitions. The table contains entries
; specifying which program numbers are allowed to be initiated via V37.
; This prevents the crew from accidentally starting invalid or dangerous
; program combinations.
;
; COMMENT-ONLY READERS: The computer now searches through its list of
; allowed programs to verify that the crew's requested program number is
; safe and valid. If the program isn't in the list, the request will be
; rejected. This safety check prevents accidental activation of incorrect
; or conflicting programs during critical mission phases.
;
; CODE-ALONG READERS: NOV37MM is the table size. The search uses indexed
; addressing (NDX) to iterate through PREMM1 table entries. Each entry's
; lower 7 bits (MASK LOW7) contain the program number. The search compares
; requested MMNUMBER against each table entry until match found or table
; exhausted. MPAC+1 serves as loop counter/index. MINDEX preserves the
; matching table index for subsequent processing.
CHECKTAB	CA	NOV37MM			# INDEX FOR MM TABLES (table size)

# Page 228
; Table Search Loop
; Iterates through PREMM1 table comparing each entry against requested
; major mode number. Loop terminates on match or table exhaustion.
AGAINMM		TS	MPAC +1			# Store current table index
		NDX	MPAC +1			# Indexed addressing using MPAC+1
		CA	PREMM1			# OBTAIN WHICH MM THIS IS FOR
		MASK	LOW7			# Extract program number (bits 0-6)
		COM				# Complement for comparison
		AD	MMNUMBER		# Add requested mode number
		CCS	A			# Compare result: A>0 if entry<requested
		CCS	MPAC +1			# IF GR, SEE IF ANY MORE IN LIST
		TCF	AGAINMM			# YES, GET NEXT ONE (decrement & loop)
		TCF	V37NONO			# LAST TIME OR PASSED MM (not found)

; Match Found - Save Table Index
; The requested program number matches a table entry. Save the index
; for use in subsequent validation steps (checking current program
; compatibility with requested program).
		CA	MPAC +1			# Retrieve matching index
		TS	MINDEX			# SAVE INDEX FOR LATER

; ============================================================================
; TRANSITION: From Table Validation to Servicer Check
;
; The requested program number has been validated. Before proceeding with
; the program change, the AGC must check if the Servicer routine (which
; performs periodic navigation and guidance updates) is currently active.
; If the Servicer is running, the program change must wait for it to
; complete to avoid interrupting critical computations.
; ============================================================================

; Servicer Active Check
;
; ISSERVON checks whether the Servicer routine is currently running. The
; Servicer performs periodic navigation state updates and must complete
; its cycle before major mode changes can be processed. If active, V37
; sets up a return path and suspends until Servicer completes.
;
; COMMENT-ONLY READERS: The computer has validated the program change
; request. Before switching programs, it checks if a critical background
; task (the Servicer) is running navigation updates. If so, the program
; change waits for that task to complete, ensuring navigation data isn't
; corrupted mid-update.
;
; CODE-ALONG READERS: V37FLBIT in FLAGWRD7 indicates Servicer active state.
; Bit complemented then tested - if zero (Servicer active), sets OUTROUTE
; to V37RETAD for return path after Servicer completes, turns off AVEGFLAG,
; and exits job. If Servicer not active, proceeds directly to CANV37.
ISSERVON	CS	FLAGWRD7		# V37 FLAG SET -- I.E., IS SERVICER GOING
		MASK	V37FLBIT		# Check Servicer active bit
		CCS	A			# Is Servicer currently running?
		TCF	CANV37			# NO - proceed with mode change

; Servicer is Active - Set Up Return Path
; The Servicer is currently running. Turn off the averaging flag to
; prevent interference, set the output route for return after Servicer
; completes, and exit this job. V37 will resume at V37RET when Servicer
; finishes its cycle.
		TC	DOWNFLAG		# YES, TURN OFF THE AVERAGE FLAG AND
		ADRES	AVEGFLAG		# WAIT FOR SERVICER TO RETURN TO CANV37

		CAF	V37RETAD		# Set return address
		TS	OUTROUTE		# For Servicer to call when complete

		TCF	ENDOFJOB		# Suspend V37 until Servicer completes

; ============================================================================
; V37 Return Point After Servicer Completion
;
; V37RET is called when the Servicer has completed its navigation update
; cycle and the program change can now proceed. This routine checks if
; certain rendezvous programs (P20, P22, P25) are currently running, as
; these require special phase change timing (2.0SPOT, 2.11SPT, or 2.7SPT).
;
; COMMENT-ONLY READERS: The Servicer has finished its navigation update.
; The computer now checks if the spacecraft is performing a rendezvous
; maneuver, since those programs require careful coordination when
; switching modes to avoid disrupting the ongoing rendezvous calculations.
;
; CODE-ALONG READERS: Tests RNDVZBIT (P20/P22 active) and P25FLBIT to
; determine appropriate phase change code: 2.7SPT for P20/P22, 2.11SPT
; for P25, or 2.0SPT otherwise. These correspond to different restart
; protection phase values for smooth program transitions.
V37RET		CS	FLAGWRD0		# IS P20 OR P22 RUNNING?
		MASK	RNDVZBIT		# Check rendezvous program bit
		CCS	A			# Test result
		TCF	+2			# NO. CHECK FOR P25.
		TCF	2.7SPT			# YES. DO 2.7SPOT
		CS	FLAGWRD0		# IS P25 RUNNING?
		MASK	P25FLBIT		# Check P25 active bit
		CCS	A			# Test result
2.0SPT		CA	OCT37667		# Neither P20/P22/P25: use standard phase
2.11SPT		AD	BIT5			# P25 running: adjust phase value
2.7SPT		AD	OCT40072		# P20/P22 running: use rendezvous phase
		TC	PHSCHNGA		# Execute phase change

; ============================================================================
; CANV37 - Common V37 Program Change Processing
;
; All paths (direct from ISSERVON or return from V37RET) converge here to
; begin the actual program change sequence. This routine initializes the
; super bank register, sets up restart protection for the transition, and
; prepares to call R00 (the program zero routine) which clears the current
; program state before loading the new program.
;
; COMMENT-ONLY READERS: The checks are complete and the computer is ready
; to change programs. It first clears some internal state to prepare for
; the transition, ensuring the new program will start with a clean slate.
;
; CODE-ALONG READERS: Clears SUPERBNK to reset fixed-fixed bank register.
; Sets TEMPFLSH to R00AD (R00 entry point) for restart protection during
; transition. PHASCHNG with OCT 14 establishes phase table entry for
; restart group 4, priority 1, protecting the program change sequence.
CANV37		CAF	ZERO			# Clear super bank register
		EXTEND
		WRITE	SUPERBNK		# Reset fixed-fixed bank addressing

		CAF	R00AD			# Get R00 routine address
		TS	TEMPFLSH		# Set for restart protection

		TC	PHASCHNG		# Establish restart phase
		OCT	14			# Group 4, priority 1

# Page 229

; ============================================================================
; R00 - Program Zero Initialization Routine
;
; R00 is the "program zero" routine that prepares the AGC for a major mode
; change by clearing the current program's state. It waits for any ongoing
; integration calculations to complete, clears various program flags, and
; determines whether to proceed to a new program (NOUVEAU) or to P00 idle
; mode (POOH). This routine ensures a clean transition between programs.
;
; COMMENT-ONLY READERS: This is the "program zero" state - like clearing
; your desk before starting new work. The computer waits for any ongoing
; calculations to finish, turns off various flags that controlled the
; previous program, and prepares to either load the new requested program
; or return to idle mode.
;
; CODE-ALONG READERS: Enters interpreter mode to call INTSTALL (stall until
; integration completes). Clears critical flags: 3AXISFLG (3-axis attitude
; control), R04FLAG (R04 routine active), MUNFLAG (lunar surface flag),
; XOVINFLG (X-axis override). Tests MMNUMBER: if zero, goes to POOH (P00
; idle); if non-zero, proceeds to NOUVEAU to load new major mode.
R00		TC	INTPRET			# Enter interpreter mode

; Wait for Integration to Complete
; Before changing programs, any ongoing trajectory integration must finish
; to ensure navigation state consistency. INTSTALL stalls execution until
; the integration cycle completes.
		CALL				# WAIT FOR INTEGRATION TO FINISH
			INTSTALL		# Stall until integration done
DUMMYAD		EXIT				# Return to native AGC mode

; Clear Program-Specific Flags
; Various flags controlling the previous program's behavior must be cleared
; to prevent interference with the new program. These flags control attitude
; modes, routine activity, and operational states.
		TC	DOWNFLAG		# Clear flag
		ADRES	3AXISFLG		# RESET 3-AXIS FLAG

		CAF	LRBYBIT			# CLEAN UP THE R12 FLAGWORD.
		TS	FLGWRD11		# Set landing radar bypass bit

		TC	DOWNFLAG		# INSURE THAT THE R04FLAG IS CLEAR.
		ADRES	R04FLAG			# R04 routine not active

		TC	DOWNFLAG		# INSURE MUNFLAG IS CLEAR.
		ADRES	MUNFLAG			# Lunar surface flag cleared

		TC	DOWNFLAG		# ALLOW X-AXIS OVERRIDE.
		ADRES	XOVINFLG		# Clear X-axis override inhibit

; Determine Next Action: POOH (P00) or NOUVEAU (New Program)
; MMNUMBER holds the requested program number. If zero, this is a request
; to return to P00 idle mode (POOH). If non-zero, proceed to load the new
; program (NOUVEAU).
		CCS	MMNUMBER		# IS THIS A POOH REQUEST
		TCF	NOUVEAU			# NO, PICK UP NEW PROGRAM

; ============================================================================
; POOH - Return to P00 Idle Mode
;
; POOH handles the transition to Program 00 (P00), the AGC's idle mode.
; When no active program is running, P00 provides basic housekeeping and
; waits for crew input. This section ensures all engines are off, coasting
; flight mode is enabled, tracking flags are cleared, and various program
; groups are terminated cleanly.
;
; COMMENT-ONLY READERS: "POOH" returns the computer to idle mode - like
; putting the car in park. All engines shut down, the spacecraft enters
; coasting flight, and the computer waits for the crew to select a new
; program via the DSKY keyboard.
;
; CODE-ALONG READERS: Releases DSKY display system via RELDSP. Clears
; various operational flags (NODOFLAG, P20/P25 flags, tracking flags).
; Sets P00 downlist code. Calls ENGINOF1 (engine off), ALLCOAST (coasting
; flight with RESTORDB), V37KLEAN to kill program groups 1,3,5,6, then
; P00KLEAN for group 4. Transfers to GOPROG2 to display P00 on DSKY.
; ============================================================================

POOH		TC	RELDSP			# RELEASE DISPLAY SYSTEM

; Set P00 Integration Priority
; P00 uses lower-priority integration (PRIO5) compared to active programs.
; This allows P00's orbital integration to run in the background without
; interfering with higher-priority tasks if a new program starts.
		CAF	PRIO5			# SET VARIABLE RESTART PRIORITY FOR
		TS	PHSPRDT2		# P00 INTEGRATION.

; Clear Address Mode
; CLRADMOD clears the radar address mode and performs INHINT to disable
; interrupts during this critical transition sequence.
		TC	CLRADMOD		# CLRADMOD DOES AN INHINT.

; Clear Operational Flags
; Various flags controlling active program modes must be cleared when
; entering P00. NODOFLAG controls whether IMU drift compensation is active.
		CS	NODOBIT			# TURN OFF NODOFLAG.
		MASK	FLAGWRD2		# Clear bit in flag word 2
		TS	FLAGWRD2

; Set Restart Phase for State Integration
; Configure restart protection to resume at STATEINT1 if a restart occurs
; during P00. Phase value of -5 identifies the restart point.
		CA	FIVE			# SET RESTART FOR STATEINT1
		TS	L			# Store in L register
		COM				# Complement to make negative
		DXCH	-PHASE2			# Set phase value

; Clear Rendezvous and IMU Flags
; P20 (rendezvous navigation), P25 (orbit determination), IMU in-use flag,
; and REMDFLG (radar enhanced mode display) are all cleared when entering
; P00 idle mode.
		CS	OCT700			# TURN OFF P20, P25, IMU IN USE FLAG
		MASK	FLAGWRD0		# Clear bits in flag word 0
		TS	FLAGWRD0		#			 REMDFLG

; Set P00 Downlist Code
; Configure telemetry downlink to transmit P00-appropriate data format.
; DNLADP00 selects the P00 downlist, which provides basic housekeeping
; telemetry rather than active program-specific data.
		CAF	DNLADP00		# P00 downlist code

SEUDOP00	TS	DNLSTCOD		# SET UP APPROPRIATE DOWNLIST CODE
		TS	AGSWORD			#   (CURRENT LIST WILL BE COMPLETED BEFORE
						#     NEW ONE IS STARTED)

; Ensure All Engines Are Off
; Before entering P00 idle mode, all propulsion systems must be disabled.
; ENGINOF1 shuts down any active engine (DPS or APS).
		TC	IBNKCALL		# Cross-bank call
		CADR	ENGINOF1		# Turn off all engines

# Page 230

; Enable Coasting Flight Mode
; ALLCOAST configures the AGC for unpowered coasting flight, which is the
; normal state during P00. This routine also calls RESTORDB to restore the
; DAP (Digital Autopilot) database to its default coasting configuration.
		TC	IBNKCALL		# INSURE ALLCOAST.
		CADR	ALLCOAST		# DOES A RESTORDB.

; Clear Tracking and Update Flags
; Turn off flags related to optical tracking and state vector updates.
; These flags control whether the AGC is actively processing tracking marks
; or incorporating navigation updates - operations not needed in P00 idle.
		CS	OCT120			# TURN OFF TRACK, UPDATE FLAGS
		TS	EBANKTEM		# Store for EBANK manipulation
		MASK	FLAGWRD1		# Clear bits in flag word 1
		TS	FLAGWRD1

; Kill Program Groups 1, 3, 5, 6
; The AGC organizes jobs into groups for termination control. V37KLEAN
; terminates jobs in groups 1 (display), 3, 5, and 6, which are used by
; various active programs. This clears out tasks from the previous program.
		TC	IBNKCALL		# KILL GROUPS 1,3,5,6
		CADR	V37KLEAN		# Clean up program groups

; Test for True P00 vs Program Change
; If MMNUMBER is zero, this is a true return to P00 idle. If non-zero,
; it's a transition through "POOH" logic on the way to another program
; (RENDV00 path handles special cases like P22 rendezvous).
		CCS	MMNUMBER		# IS IT POOH
		TCF	RENDV00			# NO

; Kill Program Group 4
; P00KLEAN terminates jobs in group 4, which handles additional program-
; specific tasks. This is called "redundant" in the original comment because
; most work was done by V37KLEAN, but group 4 requires separate cleanup.
GOMOD		TC	IBNKCALL		# REDUNDANT EXCEPT FOR GROUP 4
		CADR	P00KLEAN		# Clean up group 4

; Set Major Mode Register
; MODREG holds the current major mode (program number) displayed on the
; DSKY. Setting it from MMNUMBER (which is zero for P00) prepares for the
; GOPROG2 display update.
		CA	MMNUMBER		# Get requested program number
		TS	MODREG			# Set major mode register

; Display P00 and Complete Transition
; GOPROG2 updates the DSKY to display the current program number (P00) and
; performs final housekeeping for the program change. The spacecraft is now
; in P00 idle mode, awaiting crew input for the next program.
GOGOPROG	TC	POSTJUMP		# Jump to GOPROG2
		CADR	GOPROG2			# Display program and complete

; ============================================================================
; RENDV00 - Rendezvous Program Transition Logic
;
; This section handles the special case of transitioning between rendezvous
; programs (P20, P22, P25). These programs share computational resources and
; flags that must be managed carefully during transitions. The logic
; determines whether to clear rendezvous flags completely (RESET22) or
; preserve them for program continuity (STATQUO).
;
; COMMENT-ONLY READERS: When switching between rendezvous programs (used
; for docking with the Command Module after lunar ascent), the computer must
; decide whether to keep tracking data or start fresh. This complex logic
; ensures smooth transitions without losing critical rendezvous information.
;
; CODE-ALONG READERS: Tests current program (MODREG) and new program
; (MMNUMBER) against P20/P22/P25. If either is P22, clears rendezvous flags
; via RESET22. If new program is P20/P25 and old wasn't, checks if P20/P25
; is already running - if so, preserves group 2. Otherwise handles various
; transition cases via RENDN00 and STATQUO paths.
; ============================================================================

RENDV00		CS	MODREG			# IS CURRENT PROGRAM 22
		AD	OCT26			# Test against P22 (octal 26)
		EXTEND
		BZF	RESET22			# YES -- CLEAR RENDEZVOUS FLAG

		CS	MMNUMBER		# IS NEW PROGRAM P22
		AD	OCT26			# Test against P22
		EXTEND
		BZF	RESET22			# Yes, go clear flags

		AD	NEG2			# IS NEW PROGRAM = P20 OR P25
		EXTEND
		BZF	RENDN00			# YES (P20 = octal 24)
		AD	FIVE			# Test for P25
		EXTEND
		BZF	RENDN00			# YES (P25 = octal 31)

		CA	OCT500			# NO, IS EITHER P20 OR P25 RUNNING
		MASK	FLAGWRD0		# Check P20/P25 flags
		CCS	A			# Test result
		TCF	P00FIZZ			# YES, LEAVE GROUP 2 TO PICK UP P20 OR P25

; Clear Rendezvous and IMU Flags
; When transitioning away from rendezvous programs, clear the rendezvous
; flag, P25 flag, and IMU in-use flag. This ensures a clean break from
; rendezvous mode. CLRADMOD clears radar address mode.
RESET22		CS	OCT700			# CLEAR RENDEZVOUS, P25
		MASK	FLAGWRD0		# AND IMU IN USE FLAGS
		TS	FLAGWRD0		# Update flag word
		TC	CLRADMOD		# Clear address mode
# Page 231

; Kill Group 2
; Terminate all jobs in group 2 by setting phase to -0. Group 2 typically
; handles integration and navigation updates. Setting -PHASE2 to -0 marks
; this group for termination.
KILL2		EXTEND				# NO, KILL 2
		DCA	NEG0			# Get -0 (double precision)
		DXCH	-PHASE2			# Set phase to kill group 2

; Set Restart Point for V37
; If a restart occurs after this point, execution will resume at V37XEQ.
; TEMPFLSH holds the restart address for flash display recovery.
P00FIZZ		CAF	V37QCAD			# RESTART POINT FOR V37XEQ
		TS	TEMPFLSH		# Store restart address

		TCF	GOGOPROG		# Continue to display program

; Rendezvous Program Continuity Logic
; RENDN00 handles the case where we're transitioning to a rendezvous program
; (P20 or P25). It must determine whether we're continuing an existing
; rendezvous sequence or starting a new one.
RENDN00		CS	MODREG			# Get current program
		AD	OCT24			# Test against P20
		EXTEND
		BZF	KILL2			# P20 OR P25 ON TOP OF P20 OR P25 --

		AD	FIVE			# Test against P25
		EXTEND
		BZF	KILL2			# Kill group 2 if same

		CA	OCT500			# Check P20/P25 flags
		MASK	FLAGWRD0
		AD	MMNUMBER		# Add new program number
		COM				# Complement
		AD	P20REG			# IS IT 20 AND IS RENDEZVOUS FLAG ON
		EXTEND
		BZF	STATQUO			# YES
		AD	OCT305			# IS IT 25 AND IS P25 BIT ON
		EXTEND
		BZF	STATQUO			# YES, LEAVE AS IS
		TCF	KILL2			# No continuity, kill group 2

; Maintain Status Quo - Preserve Tracking
; When continuing a rendezvous sequence, preserve the tracking flag and
; update flag. This maintains navigation data continuity between related
; rendezvous programs, preventing loss of tracking marks or state updates.
STATQUO		CS	FLAGWRD1		# SET TRACKFLAG
		MASK	OCT120			#	UPDATE FLAG
		ADS	FLAGWRD1		# Add to flag word (set bits)

		TCF	GOMOD			# Continue with program change

; NOUVEAU - New Program Setup
; Handle initialization for programs that aren't rendezvous-related or don't
; require the complex transition logic of RENDV00. This simpler path clears
; the IMU in-use flag if no rendezvous programs are active, sets up the
; appropriate downlist code, and continues to the program change sequence.
;
; COMMENT-ONLY READERS: For most program changes, the computer performs
; simpler cleanup: if no rendezvous tracking is active, it releases the IMU
; (Inertial Measurement Unit) for other uses and configures telemetry for
; the new program.
NOUVEAU		CAF	OCT500			# IS P20 OR P25 FLAG SET
		MASK	FLAGWRD0		# Test rendezvous flags
		CCS	A			# Check result
		TCF	+3			# YES (skip IMU flag clear)
		TC	DOWNFLAG		# NO, RESET IMUINUSE FLAG
		ADRES	IMUSE			# Clear IMU in-use flag

		INDEX	MINDEX			# Use program index
		CAF	DNLADMM1		# OBTAIN APPROPIRATE DOWNLIST ADDRESS

		INHINT				# Disable interrupts
		TCF	SEUDOP00		# Set downlist and continue

; Invalid Program Number Handler
; If the crew requests a program (via V37) that doesn't exist in the valid
; program table (PREMM/DNLADMM tables), this handler displays an operator
; error and returns to V37 for a new entry.
V37NONO		TC	FALTON			# COME HERE IF MM REQUESTED DOESN'T EXIST
# Page 232
		TCF	V37BAD

OCT00010	EQUALS	BIT4
OCT500		OCT	500			# BITS 7 AND 9
OCT305		OCT	305
OCT26		OCT	26
P20REG		OCT	124

; ============================================================================
; V37XEQ - Verb 37 Execute (Major Mode Change Implementation)
;
; This is the core execution routine that actually starts the new major mode
; program requested by the crew. After validation by CHECKTAB and table
; lookup of MINDEX, this routine extracts the program's configuration data
; from PREMM1 (priority, EBANK, mode number), builds a 2CADR from FCADRMM1,
; and calls SPVAC (Special FINDVAC) to start the new job with proper priority
; and bank settings.
;
; The routine performs sophisticated bit manipulation to extract:
; - Bits 14-10 of PREMM1: Priority for the new job
; - Bits 8-10 of PREMM1: EBANK (erasable bank) setting
; - Bits 0-6 of PREMM1: Major mode number
; - FCADRMM1 entry: Fixed bank and address of starting routine
;
; COMMENT-ONLY READERS: Once the computer has validated the crew's program
; request, this routine actually launches the new program. It extracts all
; the technical parameters needed (priority level, memory bank, starting
; address) from lookup tables and starts the new job. After successful
; startup, the new major mode number appears on the DSKY display, and the
; computer is running the requested program.
;
; CODE-ALONG READERS: Classic AGC table-driven dispatch. MINDEX (computed by
; CHECKTAB) indexes into PREMM1 to get packed configuration word containing
; priority (bits 14-10), EBANK (bits 8-10), and MM number (bits 0-6). Same
; MINDEX indexes FCADRMM1 to get FCADR of start address. Builds 2CADR in
; L,A registers (EBANK in L via BIT8 multiply, address in A via LOW10 mask).
; SPVAC starts job at computed 2CADR with extracted priority. On return from
; SPVAC, stores new MM number via NEWMODEA, releases display via RELDSP.
; ============================================================================

V37XEQ		INHINT				# Disable interrupts during setup
		INDEX	MINDEX			# Use MINDEX computed by CHECKTAB
		CAF	PREMM1			# Load configuration word: prio (14-10), EBANK (10-8), MM (6-0)
		TS	MMTEMP			# Save complete config word
		TS	CYR			# Also store in CYR for right-shift to extract priority

; Extract priority field (bits 14-10) and set up restart protection.
; Priority determines job scheduling order relative to other programs.
		CA	CYR			# Retrieve config word
		MASK	PRIO37			# Mask to extract bits 14-10 (priority)
		TS	PHSPRDT4		# Set Group 4 restart priority for recovery
		TS	NEWPRIO			# Store priority for SPVAC call

; Extract EBANK field (bits 10-8) which specifies erasable memory bank.
; EBANK determines which 256-word erasable bank is accessible to program.
		CA	MMTEMP			# Retrieve config word
		EXTEND				# Enable multiply
		MP	BIT8			# Multiply by 256: shifts EBANK bits to proper position
		MASK	LOW3			# Mask bits 0-2: isolated EBANK value
		TS	L			# Save EBANK in L register (lower accumulator)

; Extract FCADR (bank+address) from table and construct BBCON.
; BBCON combines fixed bank (bits 14-10) with EBANK for 2CADR.
		INDEX	MINDEX			# Use same index
		CAF	FCADRMM1		# Load FCADR of program start address
		TS	BASETEMP		# Save for later address extraction
		MASK	HI5			# Extract bits 14-10 (fixed bank number)
		ADS	L			# Add to EBANK -> complete BBCON in L

; Extract address portion and add bit 11 flag for SPVAC protocol.
		CA	BASETEMP		# Retrieve FCADR
		MASK	LOW10			# Extract bits 0-9 (address within bank)
		AD	BIT11			# Add bit 11 flag (SPVAC protocol requirement)

; Launch the new major mode program with computed 2CADR and priority.
		TC	SPVAC			# Call Special FINDVAC: starts job at (L,A) with NEWPRIO

; Return point from SPVAC after new program job successfully started.
; Update MODREG with new major mode number and release DSKY display.
V37XEQC		CA	MMTEMP			# Retrieve original config word
		MASK	LOW7			# Extract bits 0-6 (major mode number)
		TC	NEWMODEA		# Update MODREG (stored in PHSPRDT1 low 7 bits)
						# This makes new MM visible on DSKY display

		TC	RELDSP			# Release display system for new program's use
		TC	ENDOFJOB		# Terminate V37 job, new program now running

; Symbolic constants and address aliases used by V37 major mode change logic.
NEG7		EQUALS	OCT77770		# Constant -7 octal for major mode validation

; Temporary storage allocation via EQUALS (uses existing memory locations).
MMTEMP		EQUALS	PHSPRDT3		# Config word storage: reuses Phase 3 restart priority
BASETEMP	EQUALS	TBASE4			# FCADR storage: reuses Time Base 4 location
V37QCAD		CADR	V37XEQ +3		# Return address within V37XEQ for restart logic
R00AD		CADR	DUMMYAD			# Dummy address for R00 (idle program) dispatch

# Page 233
; Additional address constants for program dispatch and configuration.
V37RETAD	CADR	V37RET			# Return address for V37 completion
OCT37667	OCT	37667			# Configuration constant (specific bit pattern)
OCT40072	OCT	40072			# Configuration constant (specific bit pattern)
OCT700		OCT	700			# Mask constant (octal 700 = bits 8,7,6)

; ============================================================================
; SETUP70/SETUP71 - Abort Program Dispatcher
;
; Quick setup routines to launch P70 (LM abort program) or P71 (abort staging)
; with minimal overhead. SETUP71 adds 3 to base address for P71 variant entry.
; Both use DTCB (Double Transfer Control to Bank) for efficient cross-bank jump.
; ============================================================================

SETUP71		CAF	THREE			# P71 offset: 3 words beyond P70 entry
SETUP70		TS	Q			# Store offset in Q
		EXTEND				# Enable DCA (double precision load)
		DCA	P70CADR			# Load 2CADR of P70 abort program
		AD	Q			# Add offset (0 for P70, 3 for P71)
		DTCB				# Double Transfer Control to Bank: jump to computed address

; Constants for P70/P71 abort program dispatch.
DEC70		DEC	70			# Decimal 70 (program number)
		EBANK=	R			# EBANK setting for P70/P71
P70CADR		2CADR	P70			# Full 2CADR of P70 abort program start

; ============================================================================
; VERB 37 MAJOR MODE DATA TABLES
;
; The astronaut uses Verb 37 (V37) to request a program change. When the crew
; enters V37 followed by a program number (e.g., V37 E 63 E for lunar landing),
; the AGC searches these parallel tables to find the start address, priority,
; and memory bank for the requested program. These tables enable the flexible
; program switching that allowed Armstrong and Aldrin to move from orbital
; navigation to powered descent to abort programs as needed during Apollo 11.
;
; COMMENT-ONLY READERS: These tables are the "program menu" the computer uses
; when you request a program change via the DSKY. Think of it as a lookup
; table mapping program numbers to their launch configurations.
;
; CODE-ALONG READERS: Three parallel tables indexed by major mode number:
; FCADRMM1 (start addresses), PREMM1 (priority/ebank/MM encoded), and
; DNLADMM1 (downlink telemetry selection codes). All ordered highest-to-lowest
; major mode number to optimize sequential search in CHECKTAB.
; ============================================================================

# FOR VERB 37 TWO TABLES ARE MAINTAINED.  EACH TABLE HAS AN ETRY FOR EACH
# MAJOR MODE THAT CAN BE STARTED FROM THE KEYBOARD.  THE ENTRIES ARE PUT
# INTO THE TABLE WITH THE ENTRY FOR THE HIGHEST MAJOR MODE COMING FIRST,
# TO THE LOWEST MAJOR MODE WHICH IS THE LAST ENTRY IN EACH TABLE.
#
# THE FCADRMM TABLE CONTAINS THE FCADR OF THE STARTING JOB OF
# THE MAJOR MODE.  FOR EXAMPLE,
#
#	FCADRMM1	FCADR	P79		# START OF P 79
#			FCADR	PROG18		# START OF P 18
#			FCADR	P01		# START OF P 01
#
# NOTE:		THE FIRST ENTRY MUST BE LABELED FCADRMM1.
# -----

; FCADRMM1 - Program Start Address Table
; Each entry is an FCADR (Fixed-memory CADR) pointing to the initialization
; routine for that major mode. Ordered from highest (P79) to lowest (P06).
; When crew requests program change, V37 uses CHECKTAB to scan this list.

FCADRMM1	FCADR	P79			# Rendezvous targeting (terminal phase)
		FCADR	P78			# Rendezvous targeting variant
		FCADR	P76			# Target delta-V program
		FCADR	P75			# Rendezvous Lambert targeting
		FCADR	P74			# Rendezvous Lambert targeting variant
		FCADR	P73			# Rendezvous Lambert targeting variant
		FCADR	P72			# Rendezvous Lambert targeting variant
		FCADR	LANDJUNK		# P68 Landing confirmation program
		FCADR	P63LM			# ** P63 LUNAR LANDING - Apollo 11's historic descent **
		FCADR	P57			# Lunar surface alignment program
		FCADR	PROG52			# P52 IMU realignment program
		FCADR	P51			# P51 IMU orientation program
		FCADR	P47LM			# CM/LM thrust program (LM variant)
		FCADR	P42LM			# External delta-V (LM variant)
		FCADR	P41LM			# RCS external delta-V (LM variant)
		FCADR	P40LM			# DPS external delta-V (LM variant)
		FCADR	P39			# Rendezvous targeting
		FCADR	P38			# Rendezvous targeting variant
# Page 234
		FCADR	P35			# Lambert aim point guidance
		FCADR	P34			# Lambert targeting (NCC1/TPI)
		FCADR	P33			# Coelliptic sequence initiation
		FCADR	P32			# Rendezvous height adjustment
		FCADR	P31			# Display orbital parameters
		FCADR	P30			# External delta-V guidance
		FCADR	PROG25			# P25 Rendezvous tracking
		FCADR	PROG22			# P22 Lunar landmark tracking
		FCADR	PROG21			# P21 Ground track determination
		FCADR	PROG20			# P20 Rendezvous navigation
		FCADR	P12LM			# ** P12 POWERED ASCENT - Eagle's return to orbit **
		FCADR	P06			# P06 Backup IMU orientation

; PREMM1 - Priority/E-Bank/Major Mode Configuration Table
; Each entry is a packed 15-bit word containing three fields:
;   Bits 11-14 (5 bits): PRIORITY (job scheduling priority)
;   Bits 8-10  (3 bits): E-BANK number (erasable memory bank for variables)
;   Bits 0-7   (7 bits): MAJOR MODE number (program identifier)
;
; COMMENT-ONLY READERS: When you request program P63 (lunar landing), the AGC
; needs to know what priority to give it (high priority so it won't be
; interrupted), which memory bank to use for its variables, and which program
; number it is. All three pieces of information are packed into one word to
; save precious memory.
;
; CODE-ALONG READERS: Bit packing format PPP PPE EEM MMM MMM enables single
; word lookup. Example: OCT 27677 = binary 010111 111 0111111 = Priority 13,
; E-Bank 7, Major Mode 63 (P63 lunar landing). The unpacking is done in the
; V37 dispatcher using MASK operations to extract each field.

# THE PREMM TABLE CONTAINS THE E-BANK, MAJOR MODE, AND PRIORITY
# INFORMATION, IT IS IN THE FOLLOWING FORM,
#
#	PPP PPE EEM MMM MMM
#
#	WHERE THE	7 M BITS CONTAIN THE MAJOR MODE NUMBER
#			3 E BITS CONTAIN THE E-BANK NUMBER
#			5 P BITS CONTAIN THE PRIORITY AT WHICH THE JOB IS
#			    TO BE STARTED
#
#	FOR EXAMPLE,
#
#		PREMM1		OCT	67213		# PRIORITY	33
#							# E-BANK	5
#							# MAJOR MODE	11
#				OCT	25437		# PRIORITY	12
#							# E-BANK	6
#							# MAJOR MODE	31
#
# NOTE:		THE FIRST ENTRY MUST BE LABELED PREMM1

PREMM1		OCT	27717		# MM 79		EBANK 7		PRIO 13 (Rendezvous targeting)
		OCT	27716		# MM 78		EBANK 7		PRIO 13
		OCT	27714		# MM 76		EBANK 7		PRIO 13 (Target delta-V)
		OCT	27713		# MM 75		EBANK 7		PRIO 13 (Lambert targeting)
		OCT	27712		# MM 74		EBANK 7		PRIO 13
		OCT	27711		# MM 73		EBANK 7		PRIO 13
		OCT	27710		# MM 72		EBANK 7		PRIO 13
		OCT	27704		# MM 68		EBANK 7		PRIO 13 (Landing confirmation)
		OCT	27677		# ** MM 63 P63 LUNAR LANDING ** EBANK 7 PRIO 13
		OCT	27271		# MM 57		EBANK 5		PRIO 13 (Surface alignment)
		OCT	27264		# MM 52		EBANK 5		PRIO 13 (IMU realignment)
		OCT	27263		# MM 51		EBANK 5		PRIO 13 (IMU orientation)
		OCT	27657		# MM 47		EBANK 7		PRIO 13
		OCT	27652		# MM 42		EBANK 7		PRIO 13
		OCT	27651		# MM 41		EBANK 7		PRIO 13
		OCT	27650		# MM 40		EBANK 7		PRIO 13 (DPS burn program)
		OCT	27647		# MM 39		EBANK 7		PRIO 13
		OCT	27646		# MM 38		EBANK 7		PRIO 13
# Page 235
		OCT	27643		# MM 35		EBANK 7		PRIO 13
		OCT	27642		# MM 34		EBANK 7		PRIO 13
		OCT	27641		# MM 33		EBANK 7		PRIO 13 (Coelliptic)
		OCT	27640		# MM 32		EBANK 7		PRIO 13 (Rendezvous height)
		OCT	27637		# MM 31		EBANK 7		PRIO 13 (Display orbital params)
		OCT	27636		# MM 30		EBANK 7		PRIO 13 (External delta-V)
		OCT	27631		# MM 25		EBANK 7		PRIO 13 (Rendezvous tracking)
		OCT	27626		# MM 22		EBANK 7		PRIO 13 (Landmark tracking)
		OCT	27625		# MM 21		EBANK 7		PRIO 13 (Ground track)
		OCT	27624		# MM 20		EBANK 7		PRIO 13 (Rendezvous nav)
		OCT	27614		# ** MM 12 P12 POWERED ASCENT ** EBANK 7 PRIO 13
		OCT	27006		# MM 06		EBANK 4		PRIO 13 (Backup orientation)

; ============================================================================
; NOV37MM Constant - Table Size Definition
;
; This constant defines the number of entries in each of the three parallel
; Verb 37 tables (FCADRMM1, PREMM1, DNLADMM1) minus one. The value 29 means
; there are 30 major modes available for crew selection via Verb 37.
;
; The constant is used by CHECKTAB to determine loop bounds when scanning
; the tables for keyboard-entered program numbers.
; ============================================================================

# NOTE:		THE FOLLOWING CONSTANT IS THE NUMBER OF ENTRIES IN EACH OF
# -----		THE ABOVE LISTS-1 (I.E., THE NUMBER OF MAJOR MODES (EXCEPT P00)
#		THAT CAN BE CALLED FROM THE KEYBOARD MINUS ONE)

NOV37MM		DEC	29		# MM'S -1

; ============================================================================
; DNLADMM1 - Downlink Telemetry Selection Table for V37 Major Modes
;
; COMMENT-ONLY READERS:
; This table specifies which telemetry data set should be sent to Mission
; Control for each program. Different programs monitor different spacecraft
; systems, so they downlink different data. For example, rendezvous programs
; send relative position and velocity data, while descent programs send
; altitude, velocity, and engine performance data.
;
; CODE-ALONG READERS:
; Each entry contains an ADRES (address) pointing to a downlink list name.
; The names resolve to numeric codes (0-5) defined at end of this section:
;   COSTALIN (0) - Coasting/alignment programs (minimal telemetry)
;   AGSUPDAT (1) - AGS update programs
;   RENDEZVU (2) - Rendezvous navigation programs
;   ORBMANUV (3) - Orbital maneuver programs
;   DESASCNT (4) - Descent/ascent guidance programs
;   LUNRSALN (5) - Lunar surface alignment programs
;
; The telemetry code is stored in DNLSTCOD and used by the downlink program
; to select the appropriate data format and content for ground transmission.
;
; This table is ordered highest-to-lowest major mode (MM 79 → MM 12) to
; match FCADRMM1 and PREMM1 ordering for parallel indexing by CHECKTAB.
; ============================================================================

DNLADMM1	ADRES	RENDEZVU	# P79		Rendezvous targeting
		ADRES	RENDEZVU	# P78		Lambert targeting
		ADRES	RENDEZVU			# (P76 omitted)
		ADRES	RENDEZVU	# P75		Coelliptic targeting
		ADRES	RENDEZVU	# P74		CSI/CDH targeting
		ADRES	RENDEZVU	# P73		Concentric targeting
		ADRES	RENDEZVU	# P72		Coelliptic sequence
		ADRES	DESASCNT	# P68		Landing confirmation
		ADRES	DESASCNT	# P63		** LUNAR LANDING ** Descent telemetry
		ADRES	LUNRSALN	# P57		Lunar surface alignment
		ADRES	COSTALIN	# P52		Auto IMU align (optics)
		ADRES	COSTALIN	# P51		Manual IMU align
		ADRES	ORBMANUV	# P47		Thrust monitor
		ADRES	ORBMANUV	# P42		APS maneuver
		ADRES	ORBMANUV	# P41		RCS maneuver
		ADRES	ORBMANUV	# P40		DPS maneuver
		ADRES	RENDEZVU	# P39		TPI search
		ADRES	RENDEZVU	# P38		Return to LM
		ADRES	RENDEZVU	# P35		Lambert targeting
		ADRES	RENDEZVU	# P34		Orbit navigation
		ADRES	RENDEZVU	# P33		Concentric approach
		ADRES	RENDEZVU	# P32		Coelliptic approach
		ADRES   RENDEZVU	# P31LM		Display orbital params (LM)
		ADRES	RENDEZVU	# P30		External delta-V
		ADRES	RENDEZVU	# P25		Rendezvous tracking
		ADRES	LUNRSALN	# P22		Landmark tracking
		ADRES	RENDEZVU	# P21		Ground track
		ADRES	RENDEZVU	# P20		Rendezvous navigation
		ADRES	DESASCNT	# P12		** POWERED ASCENT ** Ascent telemetry
		ADRES	COSTALIN	# P06		Backup orientation

; ============================================================================
; Downlink Telemetry Code Definitions
;
; COMMENT-ONLY READERS:
; These codes tell the AGC which set of data to transmit to Mission Control.
; Each number corresponds to a different downlink list tailored to the type
; of operation: coasting flight needs minimal data, while landing and ascent
; require extensive engine and guidance data for ground monitoring.
;
; CODE-ALONG READERS:
; These constants define the numeric values stored in DNLSTCOD and used by
; the downlink program (DOWN-TELEMETRY) to select data format. Values 0-5
; index into downlink list tables that specify which memory locations to
; sample and transmit to ground stations via S-band telemetry.
;
; DNLADP00 (0) is used for Program 00 (idling in lunar orbit or on surface).
; ============================================================================

DNLADP00	=	ZERO		# Program 00 downlink (minimal telemetry)
COSTALIN	=	0		# Coasting/alignment programs
# Page 236
AGSUPDAT	=	1		# AGS update programs
RENDEZVU	=	2		# Rendezvous navigation programs
ORBMANUV	=	3		# Orbital maneuver programs
DESASCNT	=	4		# Descent/ascent guidance programs (critical)
LUNRSALN	=	5		# Lunar surface alignment programs

		BANK	13
		SETLOC	INTINIT
		BANK

		COUNT*	$$/INTIN

		EBANK=	RRECTCSM

; ============================================================================
; SECTION TRANSITION: From Verb 37 Data Tables to P00 Integration
;
; COMMENT-ONLY READERS:
; We now transition from the program selection mechanism to the background
; task that keeps the spacecraft navigation state updated. When the crew
; selects Program 00 (P00), the AGC doesn't sit idle - it continuously
; updates the spacecraft position and velocity estimates by integrating
; (mathematically predicting) forward based on gravitational forces.
;
; This integration runs automatically in the background, keeping navigation
; data fresh so any new program the crew selects starts with accurate
; position and velocity information. During Apollo 11's mission, P00 ran
; during coasting phases in lunar orbit and on the lunar surface between
; major operations.
;
; CODE-ALONG READERS:
; BANK 13 contains the integration initialization routines. The STATEUP
; routine is called periodically by the executive scheduler when MODREG=0
; (Program 00). It performs orbital integration of both the Command Module
; (CM) and Lunar Module (LM) state vectors, and updates the W-matrix
; (navigation uncertainty covariance) if applicable.
;
; The integration uses the INTEGRV routine (ORBITAL_INTEGRATION.agc) which
; implements Encke's method for precise trajectory propagation accounting
; for lunar oblateness and solar/Earth gravitational perturbations.
; ============================================================================

# THIS ROUTINE DOES THE P00 INTEGRATION

; ============================================================================
; STATEUP - Program 00 State Vector Integration Routine
;
; COMMENT-ONLY READERS:
; When the spacecraft is in Program 00 (the "waiting" mode between major
; operations), this routine runs periodically to keep the navigation state
; current. It calculates where the spacecraft is and how fast it's moving
; by mathematically predicting forward from the last known state.
;
; If the Lunar Module is on the surface (SURFFLAG set), the routine handles
; this specially since surface operations don't require orbital integration.
; If rendezvous navigation is active, it also maintains the uncertainty
; estimates (W-matrix) that tell the crew how confident the computer is
; about the relative position between the LM and CM.
;
; CODE-ALONG READERS:
; Entry: Called by executive when MODREG = 0 (P00)
; Returns: Via ENDINT after integration complete
;
; Key Flags:
;   VINTFLAG - Vector integration flag (set = integrate this pass)
;   SURFFLAG - Surface flag (set = LM on lunar surface)
;   RENDWFLG - Rendezvous W-matrix valid flag
;   DIM0FLAG - Use 6x6 W-matrix (clear) or 9x9 (set)
;   D6OR9FLG - 6 or 9 dimensional W-matrix selector
;   PRECIFLG - Precision integration flag (clear = 4-step logic enabled)
;   NODOFLAG - No-do flag (integration abort condition)
;
; Subroutines called:
;   INTSTALL - Integration setup (sets TDEC1 from TETCSM)
;   SETIFLGS - Set integration flags
;   INTEGRV  - Perform Encke integration (ORBITAL_INTEGRATION.agc)
; ============================================================================

; Command Module state vector integration begins here. Set VINTFLAG to
; enable vector integration this pass, then check if LM is on lunar surface.

STATEUP		SET	BOF		# EXTRAPOLATE CM STATE VECTOR
			VINTFLAG
			SURFFLAG	# ALSO 6X6 W-MATRIX IF LM ON LUNAR
			DOINT		# 	SURFACE AND W-MATRIX VALID

; If SURFFLAG is clear (LM not on surface, in orbital flight), check if
; rendezvous W-matrix is valid. If RENDWFLG is clear (W-matrix invalid),
; skip to DOINT for simple integration. If set (W-matrix valid), also
; integrate the 6x6 W-matrix by setting DIM0FLAG.

		BOF	SET		#	FOR RENDEZVOUS NAVIGATION.
			RENDWFLG
			DOINT
			DIM0FLAG

; Perform the integration. Clear PRECIFLG to engage 4-time-step logic in
; the INTEGRV routine, which provides more efficient computation for P00
; background integration (vs. powered flight which requires precision mode).

DOINT		CLEAR	CALL
			PRECIFLG	# ENGAGES 4-TIME STEP LOGIC IN INTEGRATION
			INTEGRV		# WHEN MODREG = 0

; After CM integration completes, check SURFFLAG again. If LM is on surface,
; skip LM integration (no orbital motion to integrate). Otherwise, set up
; for LM state vector integration.

		BON	DLOAD
			SURFFLAG
			NO-INT		# If on surface, skip to NO-INT
			TETCSM		# Load CM conic integration time
		STCALL	TDEC1		# Store as integration start time
			INTSTALL	# Initialize integration parameters

; Now perform LM state vector integration. Clear VINTFLAG (only CM was
; integrated in the first call), call SETIFLGS to configure for LM integration.

		CLEAR	CALL		# EXTRAPOLATE LM STATE VECTOR
			VINTFLAG
			SETIFLGS

; Check if rendezvous W-matrix should be integrated. If RENDWFLG is clear
; (W-matrix invalid), skip to DOINT2. If set (W-matrix valid), integrate
; the 9x9 W-matrix for the LM by setting DIM0FLAG and D6OR9FLG.

		BOF			# ALSO 9X9 W-MATRIX IF W IS VALID
			RENDWFLG
			DOINT2
		SET	SET
			DIM0FLAG	# Enable W-matrix integration
			D6OR9FLG	# Select 9x9 matrix (LM, not 6x6 CM)

; Perform LM integration. Unlike CM integration, LM uses precision mode
; (PRECIFLG set) which disables the 4-time-step optimization, providing
; higher accuracy needed when LM is close to the Moon during rendezvous.

DOINT2		SET	CALL
			PRECIFLG	# DISENGAGE 4 TIME STEP LOGIC IN INTEG.
			INTEGRV		# Call integration for LM state vector

; Integration complete. Clear NODOFLAG (integration was successful) and
; exit to ENDINT which terminates this background task.

NO-INT		CLRGO
			NODOFLAG	# Clear no-do flag (integration complete)
			ENDINT		# Exit integration routine
# Page 237
# THISVINT IS CALLED BY MIDTOAV1 AND 2

; ============================================================================
; THISVINT - Clear Vector Integration Flag
;
; COMMENT-ONLY READERS:
; This is a simple utility routine that turns off the vector integration
; flag. When certain operations need to temporarily halt background state
; vector integration, they call this routine. Think of it as a "pause
; integration" switch.
;
; CODE-ALONG READERS:
; Called by: MIDTOAV1 and MIDTOAV2 (mid-course navigation routines)
; Entry: Any time integration needs to be suspended
; Exit: Returns to caller via RVQ (return via Q register)
;
; Operation: Simply clears VINTFLAG to disable vector integration, then
; returns. The calling routine typically performs some operation that
; requires integration to be paused, then re-enables it afterward.
; ============================================================================

THISVINT	CLEAR	RVQ
			VINTFLAG	# Clear vector integration flag


