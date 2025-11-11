# Copyright:	Public domain.
# Filename:	TVCINITIALIZE.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim.lawton@gmail.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	936-944
# Mod history:	2009-05-11 JVL	Adapted from the Colossus249/ file
#				of the same name, using Comanche055 page
#				images.
#		2009-05-20 RSB	Corrections:  +80 -> +8D, added 4 missing
#				lines in TVCINIT1, changed the capitalization
#				of a couple of the "Page N" comments,
#				corrected a couple of lines in LOADCOEFF.
#		2009-05-22 RSB	In LOADCOEF, DXCH N10 +14D corrected to
#				TS N10 +14D.  Also, various comment-marks
#				were added to comments following this
#				change.
#
# This source code has been transcribed or otherwise adapted from digitized
# images of a hardcopy from the MIT Museum.  The digitization was performed
# by Paul Fjeld, and arranged for by Deborah Douglas of the Museum.  Many
# thanks to both.  The images (with suitable reduction in storage size and
# consequent reduction in image quality as well) are available online at
# www.ibiblio.org/apollo.  If for some reason you find that the images are
# illegible, contact me at info@sandroid.org about getting access to the
# (much) higher-quality images which Paul actually created.
#
# Notations on the hardcopy document read, in part:
#
#    Assemble revision 055 of AGC program Comanche by NASA
#    2021113-051.  10:28 APR. 1, 1969
#
#    This AGC program shall also be referred to as
#            Colossus 2A

; ============================================================================
; FILE: TVCINITIALIZE.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth (SPS burns)
;
; TL;DR: Thrust Vector Control (TVC) initialization establishing engine gimbal
;        setup and control gains configuration before SPS burns. Prepares digital
;        autopilot for main engine firing by loading mass properties, setting
;        control parameters, and verifying gimbal actuator readiness.
;
; COMMENT-ONLY READERS: This program prepared the main engine's steering system
;        before major rocket burns like going to and from the Moon.
; CODE-ALONG READERS: Study TVC initialization sequence, gimbal setup procedures,
;        control gain configuration, and pre-burn verification checks.
; ============================================================================

# Page 937
# NAME		TVCDAPON (TVC DAP INITIALIZATION AND STARTUP CALL)
# LOG SECTION...TVCINITIALIZE			SUBROUTINE...DAPCSM
# MODIFIED BY SCHLUNDT				21 OCTOBER 1968
# FUNCTIONAL DESCRIPTION
#	PERFORMS TVCDAP INITIALIZATION (GAINS, TIMING PARAMETERS, FILTER VARIABLES, ETC.)
#	COMPUTES STEERING (S40.8) GAIN KPRIMEDT, AND ZEROES PASTDELV,+1 VARIABLE
#	MAKES INITIALIZATION CALL TO "NEEDLER" FOR TVC DAP NEEDLES-SETUP
#	PERFORMS INITIALIZATION FOR ROLL DAP
#	CALLS TVCEXECUTIVE AT TVCEXEC, VIA WAITLIST
#	CALLS TVCDAP CDU-RATE INITIALIZATION PKG AT DAPINIT  VIA T5
#	PROVIDES FOR LOADING OF LOW-BANDWIDTH COEFFS AND GAINS AT SWICHOVR
# CALLING SEQUENCE - T5LOC=2CADR(TVCDAPON,EBANK=BZERO), T5=.6SECT5
#	IN PARTICULAR, CALLED BY "DOTVCON" IN P40
#	MRCLEAN AND TVCINIT4 ARE POSSIBLE TVC-RESTART ENTRY POINTS
# NORMAL EXIT MODE
#	TCF RESUME
# SUBROUTINES CALLED
#	NEEDLER, MASSPROP
# ALARM OR ABORT EXIT MODES
#	NONE
# ERASABLE INITIALIZATION REQUIRED
#	CSMMASS, LEMMASS, DAPDATR1 (FOR MASSPROP SUBROUTINE)
#	TVC PAD LOADS (SEE EBANK6 IN ERASABLE ASSIGNMENTS)
#	PACTOFF, YACTOFF, CDUX
#	TVCPHASE AND THE T5 BITS OF FLAGWRD6 (SET AT DOTVCON IN P40)
# OUTPUT
#	ALL TVC AND ROLL DAP ERASABLES, FLAGWRD6 (BITS 13,14), T5, WAITLIST
# DEBRIS
#	NONE

		COUNT*	$$/INIT
		BANK	17
		SETLOC	DAPS7
		BANK

		EBANK=	BZERO

; ============================================================================
; TRANSITION: TVC Initialization Entry and Memory Clearing
;
; Before the Service Propulsion System (SPS) main engine can fire for major
; maneuvers like Translunar Injection or Lunar Orbit Insertion, the Thrust
; Vector Control system must initialize its gimbal actuators and control
; parameters. This section begins that process by clearing TVC variables.
; ============================================================================

; TVCDAPON - Primary TVC DAP initialization entry point
; Called by DOTVCON in program P40 when preparing for SPS engine burns.
; This routine sets up all control gains, timing parameters, and filter
; variables needed for the digital autopilot to steer the main engine.

TVCDAPON	LXCH	BANKRUPT	# T5 RUPT ARRIVAL (CALL BY DOTVCON - P40)
		EXTEND			# SAVE Q REQUIRED IN RESTARTS (MRCLEAN AND
		QXCH	QRUPT		#	TVCINIT4 ARE ENTRIES)

; MRCLEAN - Memory clearing routine for TVC erasable variables
; Zeroes all TVC DAP variables before initialization to ensure clean state.
; This restart entry point allows recovery from program interruptions without
; corrupting partially-initialized control parameters.

MRCLEAN		CAF	NZERO		# NUMBER TO ZERO, LESS ONE  (MUST BE ODD)
					#	TVC RESTARTS ENTER HERE  (NEW BANK)
; Memory clearing loop - zeros TVC control variables in pairs
; Uses DXCH to clear two consecutive memory locations efficiently
; CNTR counts down from NZERO to zero, clearing OMEGAYC and subsequent variables

	+1	CCS	A
		TS	CNTR
		CAF	ZERO
		TS	L
		INDEX	CNTR		# Indexed addressing to clear variable pairs
		DXCH	OMEGAYC		# FIRST (LAST) TWO LOCATIONS
		CCS	CNTR		# Decrement counter and test for completion
		TCF	MRCLEAN +1	# Continue loop if more variables to clear
# Page 938
; Memory clearing complete - schedule next initialization phase
; Sets up T5 interrupt to continue initialization at TVCINIT1

		EXTEND			# SET UP ANOTHER T5 RUPT TO CONTINUE
		DCA	INITLOC2	#	INITIALIZATION AT TVCINIT1
		DXCH	T5LOC		# THE PHSCHK2 ENTRY  (REDOTVC) AT TVCDAPON
		CAF	POSMAX		#	+3 IS IN ANOTHER BANK. MUST RESET
		TS	TIME5		#	BBCON TOO (FULL 2CADR), FOR THAT
ENDMRC		TCF	RESUME		#	ENTRY.

; ============================================================================
; TRANSITION: From memory clearing to mass properties calculation
;
; With TVC DAP variables cleared and initialized to zero, the system now
; calls MASSPROP to compute the vehicle's moment of inertia (IXX) and average
; moment (IAVG). These values are critical for autopilot gain calculations
; that adapt to changing vehicle mass as propellant is consumed during SPS burns.
; During Apollo 11's translunar injection and lunar orbit insertion burns,
; accurate mass properties enabled stable attitude control throughout engine firing.
; ============================================================================

TVCINIT1	LXCH	BANKRUPT
		EXTEND
		QXCH	QRUPT
; TVCINIT1 entry point - Second initialization phase following memory clearing
; Computes vehicle mass properties and loads configuration-dependent parameters

		TC	IBNKCALL	# UPDATE IXX, IAVG/TLX FOR DAP GAINS (R03
		CADR	MASSPROP	#	OR NOUNS 46 AND 47 MUST BE CORRECT)
; Call MASSPROP subroutine to calculate moment of inertia about pitch/yaw axes
; (IXX) and average moment (IAVG). These depend on CSM mass, LM mass (if docked),
; and current vehicle configuration entered via DSKY Nouns 46 and 47.

; Calculate propellant mass loss during first 10 seconds of SPS burn
; The Service Propulsion System consumes ~10 kg/sec during main engine burns.
; This calculation prepares for adaptive gain scheduling as vehicle mass decreases.
		CAE	EMDOT		# SPS FLOW RATE, SCALED B+3 KG/CS
		EXTEND
		MP	ONETHOU
		TS	TENMDOT		# 10-SEC MASS LOSS B+16 KG
		COM
		AD	CSMMASS
		TS	MASSTMP		# DECREMENT FOR FIRST 10 SEC OF BURN
; EMDOT (M-dot) = propellant flow rate in kg/centisecond, scaled 2^3
; TENMDOT = mass consumed in 10 seconds, used for periodic gain updates
; MASSTMP = projected vehicle mass after 10 seconds of burn

; Determine vehicle configuration: CSM-only vs CSM/LM docked
; Apollo 11 flew CSM-only configuration after LM separation for trans-earth coast.
; Filter coefficients differ between configurations due to mass/inertia differences.
		CAE	DAPDATR1	# CHECK LEM-ON/OFF
		MASK	BIT14
		CCS	A
		CAF	BIT1		# LEM-ON (BIT1)
		TS	CNTR		# LEM-OFF (ZERO)
; DAPDATR1 bit 14 indicates LM docked (1) or separated (0)
; CNTR used as index: 0 = CSM-only coefficients, 1 = CSM/LM coefficients

		INDEX	CNTR		# LOAD THE FILTER COEFFICIENTS
		CAF	CSMCFADR
		TS	COEFFADR
		TC	LOADCOEF
; Select appropriate DAP filter coefficient table based on vehicle configuration
; CSMCFADR points to CSM-only coefficients, CSMCFADR+1 to CSM/LM coefficients
; LOADCOEF subroutine transfers coefficient block to active TVC variables

		INDEX	CNTR		# PICK UP LM-OFF,-ON KTLX/I
		CAE	EKTLX/I		# SCALED AT 1/(8 ASCREV) OF ACTUAL VALUE
		TS	KTLX/I
; Load KTLX/I control gain appropriate for current vehicle configuration
; This gain relates engine gimbal torque to vehicle angular acceleration
; Scaled at 1/(8 * 360°) to match AGC fixed-point arithmetic conventions

		TCR	S40.15		# COMPUTE 1/CONACC , VARK
; Call S40.15 subroutine to compute derived gains:
; 1/CONACC = reciprocal of constant acceleration (for steering law)
; VARK = variable gain K based on mass properties and engine characteristics

; ============================================================================
; TVCINIT2 - Filter Period and Timing Configuration
;
; Configures TVC DAP filter update period based on vehicle configuration.
; CSM-only uses faster 40ms update (higher bandwidth) while CSM/LM uses 80ms
; (lower bandwidth needed for higher inertia). Filter period affects control
; loop stability and gimbal actuator response during SPS burns.
; ============================================================================

TVCINIT2	CS	CNTR		# PICK LM-OFF,-ON VALUE FOR FILTER PERIOD
		INDEX	A		# DETERMINATION:
		CAF	BIT2		# 	BIT2 FOR CSM ONLY 40MS FILTER
		TS	KPRIMEDT	# 	BIT3 FOR CSM/LM 80MS FILTER
; Set KPRIMEDT (steering gain * delta-T) based on vehicle configuration
; BIT2 (octal 2) = 40ms period for lighter CSM-only configuration
; BIT3 (octal 4) = 80ms period for heavier CSM/LM docked configuration
; This gain-time product is used in discrete-time steering law computation

		COM			# PREPARE T5TVCDT
		AD	POSMAX
		AD	BIT1
		TS	T5TVCDT
; Compute T5TVCDT = time interval for T5 interrupt servicing TVC executive
; POSMAX = +37777 (octal), largest positive 15-bit value
; Computation creates proper T5 timer delta based on selected filter period

		CS	BIT15		# RESET SWTOVER FLAG
# Page 939
		MASK	FLAGWRD9
		TS	FLAGWRD9
; Clear SWTOVER (switchover) flag in FLAGWRD9 bit 15
; This flag controls transition from high-bandwidth to low-bandwidth filtering
; during SPS burn. Reset ensures high-bandwidth mode at burn start.

		INDEX	CNTR		# PICK UP LEM-OFF,-ON KPRIME
		CAE	EKPRIME		#	SCALED (100 PI)/16
		EXTEND
		MP	KPRIMEDT	# (TVCDT/2, SC.AT B+14CS)
		LXCH	A		#	SC.AT PI/8	(DIMENSIONLESS)
		DXCH	KPRIMEDT
; Compute KPRIMEDT = KPRIME * dt steering gain for discrete-time control law
; EKPRIME = primary steering gain scaled at (100*pi)/16 radians/second
; Multiplication by KPRIMEDT (sample period) produces dimensionless gain
; Double-precision result (KPRIMEDT, KPRIMEDT+1) scaled at pi/8 for precision

		INDEX	CNTR		# PICK UP LEM-OFF,-ON REPFRAC
		CAE	EREPFRAC
		TS	REPFRAC
; Load REPFRAC (repair fraction) for configuration-specific filter initialization
; This dimensionless parameter controls first-pass filter variable computation
; Different values for CSM-only vs CSM/LM to match configuration dynamics

		INDEX	CNTR		# PICK UP ONE-SHOT CORRECTION TIME
		CAF	TCORR
		TS	CNTR
; Load TCORR = correction time constant for one-shot filter update
; CNTR now holds timing value for single-pass correction computation
; Configuration-dependent to match vehicle response characteristics

		CAF	NEGONE		# PREVENT STROKE TEST UNTIL CALLED
		TS	STRKTIME
; Set STRKTIME = -1 to inhibit gimbal actuator stroke test during initialization
; Stroke test verifies full gimbal travel range but must be explicitly commanded
; Prevents uncommanded gimbal motion during TVC startup sequence

		CAF	NINETEEN	# SET VCNTR FOR VARIABLE-GAIN UPDATES IN
		TS	VCNTR		#	10 SECONDS (TVCEXEC 1/2 SEC RATE)
		TS	V97VCNTR	# FOR ENGFAIL (R41) LOGIC
; Initialize counters for periodic gain update scheduling
; VCNTR = 19 passes * 0.5 sec/pass = 9.5 seconds until first MASSPROP update
; V97VCNTR = separate counter for engine failure detection logic (R41 program)
; Gain updates compensate for changing vehicle mass as propellant is consumed

; ============================================================================
; TVCINIT3 - GIMBAL TRIM OFFSET INITIALIZATION
;
; Load gimbal actuator trim offsets from pad-loaded calibration values into
; active control variables. These offsets compensate for engine/gimbal
; mounting misalignment. Initialize offset trackers, command trackers, and
; filter states for both pitch and yaw axes.
; ============================================================================

TVCINIT3	CAE	PACTOFF		# TRIM VALUES TO TRIM-TRACKERS, OUTPUT
		TS	PDELOFF		#	TRACKERS, OFFSET-UPDATES, AND
		TS	PCMD		#	OFFSET-TRACKER FILTERS
		TS	DELPBAR		#	NOTE, LO-ORDER DELOFF,DELBAR ZEROED
; Pitch axis trim initialization: Load PACTOFF (actuator offset from pad load)
; Copy to PDELOFF (offset tracker), PCMD (command tracker), DELPBAR (filter state)
; These ensure gimbal command starts from calibrated null position
; Compensates for engine mounting bias to achieve true thrust through CG

		CAE	YACTOFF
		TS	YDELOFF
		TS	YCMD
		TS	DELYBAR
; Yaw axis trim initialization: Load YACTOFF (actuator offset from pad load)
; Copy to YDELOFF (offset tracker), YCMD (command tracker), DELYBAR (filter state)
; Symmetric process to pitch axis for perpendicular gimbal control plane

; ============================================================================
; ATTINIT - ATTITUDE ERROR INITIALIZATION
;
; Configure initial attitude error state based on vehicle configuration.
; CSM-only and CSM/LM configurations require different initialization due to
; different moments of inertia and control authority. Decision based on
; DAPDATR1 BIT13 configuration flag.
; ============================================================================

ATTINIT		CAE	DAPDATR1	# ATTITUDE-ERROR INITIALIZATION LOGIC
		MASK	BIT13		#	TEST FOR CSM OR CSM/LM
		EXTEND
		BZF	NEEDLEIN	#	BYPASS INITIALIZATION FOR CSM/LM
; Test BIT13 of DAPDATR1 configuration word for vehicle configuration
; BIT13 = 0: CSM/LM docked configuration, skip attitude initialization
; BIT13 = 1: CSM-only configuration, requires attitude error initialization
; Docked configuration inherits attitude state from pre-separation dynamics

		CAF	BIT1		#	SET UP TEMPORARY COUNTER
	+5	TS	TTMP1
; Initialize TTMP1 = 1 as loop counter to process both yaw (index 1) and pitch (index 0)
; Loop iterates twice to limit attitude errors in both axes

		INDEX	TTMP1
		CA	ERRBTMP		# ERRBTMP CONTAINS RCS ATTITUDE ERRORS
		EXTEND			#	ERRORY & ERRORZ (P40 AT DOTVCON)
		MP	1/ATTLIM	# .007325(ERROR) = 0 IF ERROR < 1.5 DEG
		EXTEND
		BZF	+8D		#	|ERROR| LESS THAN 1.5 DEG
; Load attitude error from ERRBTMP array (set by P40 DOTVCON from RCS DAP state)
; Multiply by 1/ATTLIM = 0.007325 to test magnitude threshold
; Result = 0 if |error| < 1.5 degrees (attitude close to desired)
; Branch to +8D if error within acceptable limit (no limiting required)

		EXTEND
# Page 940
		BZMF	+3		#	|ERROR| > 1.5 DEG, AND NEG
		CA	ATTLIM		#	|ERROR| > 1.5 DEG, AND POS
		TCF	+2
	+3	CS	ATTLIM
	+2	INDEX	TTMP1
		TS	ERRBTMP
; Large attitude error limiting logic (|error| > 1.5 degrees):
; If error positive: limit to +ATTLIM (maximum positive error)
; If error negative: limit to -ATTLIM (maximum negative error)
; Store limited error back to ERRBTMP array
; Prevents excessive initial gimbal commands that could cause instability

	+8	CCS 	TTMP1		#	TEST TEMPORARY COUNTER
		TCF	ATTINIT +5	#	BACK TO REPEAT FOR PITCH ERROR
; Decrement loop counter TTMP1
; If non-zero (first pass with yaw complete), branch back to process pitch axis
; Second pass completes with TTMP1 = 0, falls through to load final values

		CA	ERRBTMP		# ERRORS ESTABLISHED AND LIMITED
		TS	PERRB
		CA	ERRBTMP +1
		TS	YERRB
; Load limited attitude errors into TVC DAP error variables
; PERRB = limited pitch error (ERRBTMP)
; YERRB = limited yaw error (ERRBTMP+1)
; These become initial conditions for TVC body-axis error state

; ============================================================================
; NEEDLEIN - TVC DAP NEEDLES SETUP CALL
;
; Initialize DSKY display needles for TVC monitoring by calling NEEDLER
; subroutine. Set initialization flag (RCSFLAGS BIT3) to indicate first-pass
; setup for display configuration.
; ============================================================================

NEEDLEIN	CS	RCSFLAGS	# SET BIT 3 FOR INITIALIZATION PASS AND GO
		MASK	BIT3		# 	TO NEEDLER.  WILL CLEAR FOR TVC DAP
		ADS	RCSFLAGS	# 	(RETURNS AFTER CADR)
; Set RCSFLAGS BIT3 = 1 to signal NEEDLER this is initialization pass
; NEEDLER uses this flag to perform first-time setup of DSKY needle displays
; Subsequent TVC DAP passes will find BIT3 clear for normal display updates

		TC	IBNKCALL
		CADR	NEEDLER
; Cross-bank call to NEEDLER subroutine for display interface initialization
; NEEDLER configures DSKY pitch/yaw attitude error needles for TVC monitoring
; Returns via CADR linkage to continue initialization sequence

; ==============================================================================
; TRANSITION: From initialization to operational state
;
; With all TVC parameters configured and displays set up, the system now
; completes initialization by scheduling the main control loop and preparing
; the roll autopilot. The initialization sequence ends by scheduling CDU rate
; processing and returning control to the executive scheduler.
; ==============================================================================

; TVCINIT4 - Final initialization phase and operational scheduling
; Marks completion of primary TVC initialization sequence and prepares for
; main TVC control loop execution. Sets phase flag for restart protection,
; initializes roll DAP, and schedules TVCEXECUTIVE for first control cycle.

TVCINIT4	CAF	ZERO		# SET TVCPHASE TO INDICATE TVCDAPON-THRU-
		TS	TVCPHASE	#	NEEDLEIN INITIALIZATION FINISHED.
					#	(POSSIBLE TVC-RESTART ENTRY)
; Phase flag set to zero indicates initialization complete. If power transient
; causes restart, this flag allows restart logic to skip completed steps and
; resume at appropriate point in sequence

; Prepare roll axis digital autopilot for operation
		CAE	CDUX		# PREPARE ROLL DAP
		TS	OGANOW		# Store current roll gimbal angle as reference
; Roll DAP requires initial attitude reference. CDUX contains current roll
; gimbal angle from IMU. Stored in OGANOW for roll autopilot initialization.

; Handle mass property updates carefully to avoid conflicts with engine state
		CAF	BIT13		# IF ENGINE IS ALREADY OFF, ENGINOFF HAS
		EXTEND			#	ALREADY ESTABLISHED THE POST-BURN
		RAND	DSALMOUT	#	CSMMASS (MASSBACK DOES IT). DONT
		EXTEND			# 	TOUCH CSMMASS.  IF ENGINE IS ON,
		BZF	+3		#	THEN ITS OK TO DO THE COPYCYCLE
					#	EVEN BURNS LESS THAN 0.4 SEC ARE AOK
; Check engine on/off bit (BIT13) in discretes output register (DSALMOUT)
; If engine off: mass already updated by ENGINOFF routine, skip copy
; If engine on: safe to copy mass from MASSTMP to CSMMASS even for short burns

		CAE	MASSTMP		# COPYCYCLE
		TS	CSMMASS		# Update CSM mass with computed value
; Copy mass from temporary storage to active CSM mass variable for TVC use
; Only executed if engine currently firing (BZF skipped the +3 offset)

; Schedule first execution of main TVC control loop
	+3	CAF	.5SEC		# CALL TVCEXECUTIVE (ROLLDAP CALL, ETC)
		TC	WAITLIST	# Schedule via waitlist (timer-driven tasks)
		EBANK=	BZERO
		2CADR	TVCEXEC		# Entry point for TVC main control loop
; TVCEXEC scheduled to run in 0.5 seconds via WAITLIST timer mechanism
; This delay allows all initialization to stabilize before control loop starts
; First TVCEXEC pass will initialize roll DAP, then disable itself to let roll
; autopilot run independently while TVC handles pitch/yaw control

; Schedule CDU rate initialization for gyro rate processing
		EXTEND			# CALL FOR DAPINIT
		DCA	DAPINIT5	# Load 2CADR (bank + address) for DAPINIT
		DXCH	T5LOC		# Store in T5 rupt location for scheduling
		CAE	T5TVCDT		# (ALLOW TIME FOR RESTART COMPUTATIONS)
		TS	TIME5		# Set T5 timer for DAPINIT execution
; DAPINIT processes IMU CDU (Coupling Data Unit) angles to compute vehicle
; angular rates. T5 timer scheduled to allow restart computations time to
; complete if power was lost during initialization. DAPINIT provides gyro
; rates to digital autopilot for stabilization and control.

# Page 941
; Initialization complete - return control to executive scheduler
ENDTVCIN	TCF	RESUME		# Exit via RESUME to restore pre-interrupt state
; All TVC initialization complete. RESUME restores registers and returns
; control to whatever task was interrupted when T5 called TVCDAPON.
; TVC now operational with TVCEXEC scheduled and gyro rates being processed.

; ==============================================================================
; TRANSITION: From high-bandwidth to low-bandwidth control
;
; During long burns, TVC switches from high-bandwidth (responsive) to low-
; bandwidth (stable) control coefficients to optimize fuel usage and reduce
; control activity. This transition occurs via crew-initiated V46 or automatic
; timing, typically after initial acceleration phase when vehicle dynamics
; stabilize and precise rapid response is less critical than smooth control.
; ==============================================================================

; PRESWTCH - Bandwidth switchover entry point from crew verb 46
; Crew can manually command switch from high-bandwidth to low-bandwidth TVC
; coefficients during SPS burns when vehicle mass and dynamics have stabilized.

PRESWTCH	TCR	SWICHOVR	# ENTRY FROM V46
; Transfer control to switchover routine, return address saved in Q register

		TC	POSTJUMP	# THIS PROVIDES AN EXIT FROM SWITCH-OVER
		CADR	PINBRNCH	#	(PINBRNCH DOES A RELINT)
; Exit switchover via POSTJUMP to PINBRNCH (pinball branch) which re-enables
; interrupts (RELINT) and returns control to display interface processing

; SWICHOVR - TVC bandwidth coefficient switchover routine
; Transitions from high-bandwidth (HB) to low-bandwidth (LB) filter coefficients
; and control gains. This reduces control activity and propellant usage during
; stable portions of long burns while maintaining adequate attitude control.
; Restart-protected to handle power transients during coefficient loading.

SWICHOVR	INHINT			# Disable interrupts during coefficient switch
		CA	TVCPHASE	# SAVE TVCPHASE
		TS	PHASETMP	# Store current phase in temporary location
		CS	BIT2		# SET TVCPHASE = -2 (INDICATES SWITCH-OVER
		TS	TVCPHASE	#	TO RESTART LOGIC)
; Phase flag set to -2 signals restart logic that switchover is in progress
; If power lost during coefficient loading, restart can resume at correct point

	+5	EXTEND			# SAVE Q FOR RETURN (RESTART ENTRY POINT,
		QXCH	RTRNLOC		#	TVCPHASE AND PHASETMP ALREADY SET)
; Save return address in RTRNLOC for exit after switchover complete
; Restart entry point (+5) skips phase save since already set by original call

; Zero filter storage locations before loading new low-bandwidth coefficients
; Filter state must be cleared when switching control laws to prevent transients
		CAF	NZEROJR		# ZEROING LOOP FOR FILTER STORAGE LOCS
	+8	TS	CNTRTMP		# Initialize counter for zeroing loop

; MCLEANJR - "Junior" memory cleaning loop (smaller than MRCLEAN)
; Clears filter state variables and temporary storage used by previous
; high-bandwidth control law before loading low-bandwidth coefficients
MCLEANJR	CA	ZERO		# Load zero to clear memory
		TS	L		# Zero both A and L registers
		INDEX	CNTRTMP
		DXCH	PTMP1 -1	# Clear double-word at indexed location
; Indexed double-exchange zeros two consecutive memory locations per iteration
; Starting at PTMP1-1, clearing filter temporary storage and state variables
		CCS	CNTRTMP		# Decrement counter
		CCS	A		# Test if more locations to clear
		TCF	SWICHOVR +8D	# Continue loop if count not exhausted
; Loop continues until all filter storage locations cleared (NZEROJR iterations)

; Set downlink flag to indicate switchover has occurred
		CS	FLAGWRD9	# SET SWITCHOVER FLAG FOR DOWNLINK
		MASK	BIT15		# Isolate switchover status bit
		ADS	FLAGWRD9	# Add to flag word (toggle bit 15)
; BIT15 of FLAGWRD9 telemetered to ground controllers to confirm coefficient
; switch completed. Mission Control monitors this to verify TVC configuration.

; Load low-bandwidth control gains for all TVC control loops
; Low-bandwidth gains reduce control authority, smoothing response and
; conserving RCS/gimbal propellant during stable flight conditions

		CAE	EKTLX/I +2	# LOW BANDWIDTH GAINS 	- DAP
		TS	KTLX/I		# Load LB lateral translation gain
; KTLX/I controls lateral (pitch/yaw) thrust vector deflection response
; Lower gain means slower, smoother gimbal movements during stable burn
		TCR	S40.15 	+7	# Compute steering gain via utility routine
; S40.15 computes scaled control gains using fixed-point arithmetic
; Entry at +7 uses low-bandwidth parameters for KPRIMEDT calculation

		CAF	FKPRIMDT	#			- STEERING
		TS	KPRIMEDT	# Load LB primary steering gain
; KPRIMEDT is main steering loop gain controlling attitude error correction
; Lower value reduces aggressive maneuvering, providing smooth stable control

		CAF	FREPFRAC	#			- TMC LOOP
		TS	REPFRAC		# Load LB TMC (Thrust Magnitude Control) loop gain
; REPFRAC controls engine gimbal response to thrust magnitude errors
; Low-bandwidth setting reduces oscillations in thrust vector alignment

; Update gimbal trim offset estimates with accumulated values
; During high-bandwidth operation, DELPBAR/DELYBAR track average gimbal
; deflections needed to maintain attitude. These become baseline offsets
; for low-bandwidth control, improving accuracy and reducing steady-state error.
		EXTEND			# UPDATE TRIM ESTIMATES
		DCA	DELPBAR		# Load pitch trim accumulator (double precision)
		DXCH	PDELOFF		# Store as new pitch offset baseline
		EXTEND
		DCA	DELYBAR		# Load yaw trim accumulator (double precision)
		DXCH	YDELOFF		# Store as new yaw offset baseline
; Trim offsets compensate for center-of-gravity shifts and mass asymmetries
; Carrying forward HB-learned trim improves LB control loop initialization

; Load low-bandwidth filter coefficients from erasable memory
		CA	LBCFADR		# Address of LB coefficient table
# Page 942
		TS	COEFFADR	# Store coefficient address for LOADCOEF
		TC	LOADCOEF	# Call coefficient loading subroutine
; LOADCOEF transfers filter coefficients from erasable storage to active
; DAP filter memory locations. LB coefficients implement different frequency
; response optimized for stable cruise rather than rapid maneuvering.

; Restore TVC phase flag to pre-switchover value
		CAE	PHASETMP	# RESTORE TVCPHASE
		TS	TVCPHASE	# Phase flag now reflects normal operation
; Phase restored from temporary storage. Restart logic no longer sees
; switchover-in-progress state. System ready for normal LB operation.

		TC	RTRNLOC		# BACK TO PRESWTCH OR TVCRESTARTS
; Return via saved address: either to PRESWTCH (crew-initiated V46) or
; TVCRESTARTS (restart recovery). Interrupts re-enabled at return destination.

; ==============================================================================
; LOADCOEF - Filter coefficient loading subroutine
;
; Transfers DAP filter coefficients from source table to active filter memory.
; Source address varies: erasable memory (HB pad loads), fixed memory (LB/CSM).
; Coefficients define frequency response of TVC digital filters controlling
; pitch/yaw gimbal dynamics. Different coefficient sets optimize for different
; flight phases: high-bandwidth for rapid response during critical maneuvers,
; low-bandwidth for stable cruise minimizing propellant consumption.
; ==============================================================================

LOADCOEF	EXTEND			# LOAD DAP FILTER COEFFICIENTS
		INDEX	COEFFADR	#   FROM: ERASABLE FOR CSM/LM HB
		DCA	0		#         FIXED    FOR CSM/LM LB
		DXCH	N10		#         FIXED    FOR CSM
; First coefficient pair (N10, N11/2) - numerator coefficients for filter 1
; Indexed load allows flexible source: HBCFADR, LBCFADR, or CSMCFADR

		EXTEND			# NOTE: FOR CSM/LM, NORMAL COEFFICIENT
		INDEX	COEFFADR	# LOAD WILL BE HIGH BANDWIDTH PAD LOAD
		DCA	2		# ERASABLES. DURING CSM/LM SWITCHOVER,
		DXCH	N10 +2		# THIS LOGIC IS USED TO LOAD LOW BANDWIDTH
					# COEFFICIENTS FROM FIXED MEMORY.
; Second coefficient pair (N12, D11/2) - numerator/denominator for filter 1
; HB coefficients pad-loaded to erasable for mission-specific tuning
; LB coefficients stored in fixed memory as standard configuration

		EXTEND
		INDEX	COEFFADR
		DCA	4		# Third pair (D12, N20)
		DXCH	N10 +4		# Denominator coeff D12, numerator N20 (filter 2)

		EXTEND
		INDEX	COEFFADR
		DCA	6		# Fourth pair (N21/2, N22)
		DXCH	N10 +6		# Numerator coefficients for filter 2

		EXTEND
		INDEX	COEFFADR
		DCA	8D		# Fifth pair (D21/2, D22)
		DXCH	N10 +8D		# Denominator coefficients for filter 2

		EXTEND
		INDEX	COEFFADR
		DCA	10D		# Sixth pair (N30, N31/2)
		DXCH	N10 +10D	# Numerator coefficients for filter 3

		EXTEND
		INDEX	COEFFADR
		DCA	12D		# Seventh pair (N32, D31/2)
		DXCH	N10 +12D	# Numerator/denominator for filter 3

		INDEX	COEFFADR
		CA	14D		# Final coefficient D32
		TS	N10 +14D	# Denominator coefficient for filter 3
; 15 coefficients total (7 double-words + 1 single) define three cascaded
; second-order digital filters implementing TVC frequency response shaping

		TC	Q		# Return to caller via Q register

; ==============================================================================
; S40.15 - TVC gain computation utility (two entry points)
;
; Computes critical DAP gains using spacecraft mass properties and control
; parameters. Two entry points serve different initialization contexts:
;   Entry +0: Main entry computing 1/CONACC from moment of inertia (IXX)
;   Entry +7: Alternate entry computing VARK from lateral translation gain
; Both gains essential for TVC loop stability and performance.
; ==============================================================================

# Page 943
S40.15		CAE	IXX		# GAIN COMPUTATIONS (1/CONACC, VARK)
		EXTEND			# ENTERED FROM TVCINITIALIZE AND TVCEXEC
		MP	2PI/M		#	2PI/M SCALED 1/(B+8 N M)
; Compute 1/CONACC (conical acceleration gain inverse) from spacecraft moment
; of inertia IXX. 2PI/M is constant relating angular to conical motion.
; IXX updated by MASSPROP based on current CSM mass and fuel consumption.

		DDOUBL			#	IXX   SCALED B+20 KG-MSQ
		DDOUBL			# Triple doubling multiplies by 8
		DDOUBL			# Adjusts scaling for gain computation
		TS	1/CONACC	#	     SCALED B+9 SEC-SQ/REV
; 1/CONACC controls relationship between attitude error and gimbal deflection
; Higher spacecraft inertia (heavy CSM) requires larger gimbal authority
; Result scaled as B+9 seconds²/revolution for steering loop arithmetic

	+7	CAE	KTLX/I		# ENTRY FROM CSM/LM V46 SWITCH-OVER
		EXTEND			#            SCALED (B+3 ASCREV)  1/SECSQ
		MP	IAVG/TLX	#            SCALED B+2 SECSQ
; Alternate entry point: compute VARK from lateral translation gain KTLX/I
; IAVG/TLX is average moment of inertia for lateral (pitch/yaw) axes
; This entry used during switchover when KTLX/I just loaded with LB value

		DDOUBL			# Double scaling adjustment
		DDOUBL			# Multiply by 4 total
		TS	VARK		#            SCALED (B+3 ASCREV)
; VARK (variable gain K) adjusts filter response based on control authority
; Links translation gain to filter dynamics for consistent loop behavior
; Used by TVCEXECUTIVE to scale attitude error corrections

		TC	Q		# Return to caller via Q register

; ==============================================================================
; CSM-ONLY FILTER COEFFICIENTS (CSMN10)
;
; Special coefficient set for Command/Service Module solo operations (LM
; jettisoned). CSM-only mass distribution and control characteristics differ
; significantly from docked CSM/LM configuration, requiring dedicated filter
; tuning. Used post-transposition/docking or after LM separation for return.
; Only two filter stages defined (no third stage N30-D32 coefficients needed).
; ==============================================================================

CSMN10		DEC	.99999		# N10	CSM ONLY FILTER COEFFICIENTS
		DEC	-.2549		# N11/2 - First filter numerator coefficients
		DEC	.0588		# N12
		DEC	-.7620		# D11/2 - First filter denominator coefficients
		DEC	.7450		# D12

		DEC	.99999		# N20   - Second filter numerator coefficients
		DEC	-.4852		# N21/2
		DEC	0		# N22   - Zero coefficient (simplified response)
		DEC	-.2692		# D21/2 - Second filter denominator coefficients
		DEC	0		# D22   - Zero coefficient
; CSM filter emphasizes faster response with reduced damping since CSM-only
; configuration has higher thrust-to-mass ratio and different structural modes

; ==============================================================================
; LOW-BANDWIDTH FILTER COEFFICIENTS (LBN10)
;
; Used during stable cruise phases after transitioning from high-bandwidth
; control. LB coefficients implement gentler frequency response reducing
; gimbal activity and RCS propellant consumption during non-critical flight.
; Three complete filter stages provide comprehensive frequency shaping.
; Coefficients computed to maintain stability margins with reduced control
; authority, optimized for fuel conservation over rapid response.
; ==============================================================================

LBN10		DEC	+.99999		# N10	LOW BANDWIDTH FILTER COEFFICIENTS
		DEC	-.3285		# N11/2 - First stage numerator
		DEC	-.3301		# N12
		DEC	-.9101		# D11/2 - First stage denominator (heavier damping)
		DEC	+.8460		# D12

		DEC	+.03125		# N20   - Second stage (reduced gain)
		DEC	0		# N21/2 - Zero coefficients simplify computation
		DEC	0		# N22
		DEC	-.9101		# D21/2 - Second stage denominator
		DEC	+.8460		# D22

		DEC	+.5000		# N30   - Third stage numerator
		DEC	-.47115		# N31/2
		DEC	+.4749		# N32
		DEC	-.9558		# D31/2 - Third stage denominator (tight poles)
		DEC	+.9372		# D32
; Three cascaded second-order sections provide 6th-order filter response
; Poles/zeros positioned to attenuate structural resonances and noise while
; maintaining adequate phase margin for stable closed-loop operation

; ==============================================================================
; COEFFICIENT TABLE ADDRESS POINTERS
; ==============================================================================

CSMCFADR	GENADR	CSMN10		# CSM ONLY COEFFICIENTS ADDRESS
HBCFADR		GENADR	HBN10		# HIGH BANDWIDTH COEFFICIENTS ADDRESS
# Page 944
LBCFADR		GENADR	LBN10		# LOW BANDWIDTH COEFFICIENTS ADDRESS
; GENADR generates both bank and offset for cross-bank addressing
; LOADCOEF uses these addresses to locate coefficient tables in memory
; HBN10 defined elsewhere (erasable) for pad-loaded mission-specific tuning

; ==============================================================================
; MEMORY CLEARING LOOP COUNTERS
; ==============================================================================

NZERO		DEC	51		# MUST BE ODD FOR MRCLEAN
; MRCLEAN clears 52 locations (NZERO+1) using CCS count-down loop
; Odd value required because CCS loop tests A register after decrement

NZEROJR		DEC	23		# MUST BE ODD FOR MCLEANJR
; MCLEANJR (junior memory clear) clears 24 locations for partial initialization
; Used during restart sequences when full MRCLEAN unnecessary

; ==============================================================================
; ATTITUDE ERROR THRESHOLD CONSTANTS
; ==============================================================================

ATTLIM		DEC	0.00833		# INITIAL ATTITUDE ERROR LIMIT (1.5 DEG)
; Attitude error limit = 1.5 degrees scaled as fractional revolution (0.00833)
; During TVC initialization, attitude errors exceeding this threshold indicate
; spacecraft not properly stabilized, potentially delaying SPS burn initiation

1/ATTLIM	DEC	0.007325	# .007325(ERROR) = 0 IF ERROR < 1.5 DEG
; Reciprocal threshold for computational efficiency in error checking
; Multiplication by 1/ATTLIM faster than division by ATTLIM
; Result near zero when attitude error below 1.5°, enabling threshold detection

; ==============================================================================
; TVC TIMING CORRECTION CONSTANTS
; ==============================================================================

TCORR		OCT	00005		# CSM
	+1	OCT	00000		# CSM/LM (HB,LB)
; Timing correction factors compensating for TVC loop computational delays
; CSM (solo) requires 5-count correction due to different control bandwidth
; CSM/LM (docked) uses 0-count correction (different inertial characteristics)

; ==============================================================================
; LOW-BANDWIDTH CONTROL GAIN CONSTANTS
; ==============================================================================

FKPRIMDT	DEC	.0102		# CSM/LM (LB), (.05 X .08) SCALED AT PI/8
; Fixed low-bandwidth primary steering gain KPRIMEDT = 0.0102 scaled at π/8
; Product of 0.05 (base gain) × 0.08 (bandwidth factor) × π/8 (angle scaling)
; Loaded during SWICHOVR transition from high-bandwidth to low-bandwidth mode

FREPFRAC	DEC	.0375 B-2	# CSM/LM (LB), 0.0375 SCALED AT B+2
; Fixed low-bandwidth TMC (Thrust Magnitude Control) loop gain = 0.0375
; Scaled at 2^-2 (B-2) for proper numerical range in gimbal control arithmetic
; Reduced from HB value to smooth gimbal response during stable flight

; ==============================================================================
; MATHEMATICAL AND PHYSICAL CONSTANTS
; ==============================================================================

NINETEEN	=	VD1
; Alias: NINETEEN references VD1 location (value 19) for readability in code
; Used in loop counters and array indexing where symbolic name clarifies intent

2PI/M		DEC	.00331017 B+8	# 2PI/M, SCALED AT 1/(B+8 N-M)
; Constant 2π/M relating angular acceleration to moment (torque)
; M = moment arm, scaled appropriately for spacecraft dynamics computations
; Used in S40.15 to compute 1/CONACC gain from moment of inertia IXX
; Scaling 2^8 / (Newton-meters) matches AGC fixed-point arithmetic conventions

ONETHOU		DEC	1000 B-13	# KG/CS B3 TO KG/10SEC B16 CONVERSION
; Unit conversion constant: 1000 scaled at 2^-13 for mass flow rate conversion
; Converts kg/centisecond (B+3) to kg/10seconds (B+16) for propellant consumption
; Factor 1000 accounts for: (10 sec / 1 cs) × (100 cs / 1 sec) = 1000

; ==============================================================================
; SUBROUTINE ADDRESS POINTERS (2CADR FORMAT)
; ==============================================================================

		EBANK=	BZERO
DAPINIT5	2CADR	DAPINIT
; 2CADR pointer to DAPINIT subroutine (DAP initialization)
; Called via T5 interrupt from TVCINIT4 to initialize CDU rate processing
; EBANK=BZERO specifies erasable bank context for cross-bank subroutine call

		EBANK=	BZERO
INITLOC2	2CADR	TVCINIT1
; 2CADR pointer to TVCINIT1 continuation entry point
; Used by MRCLEAN to schedule next initialization phase via T5 waitlist
; Allows memory clearing and initialization to span multiple interrupt cycles

