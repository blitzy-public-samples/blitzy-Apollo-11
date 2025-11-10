# Copyright:	Public domain.
# Filename:	P70-P71.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	829-837
# Mod history:	2009-05-23 HG	Transcribed from page images.
#		2009-06-05 RSB	Fixed a typo.
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
; FILE: P70-P71.agc
; MODULE: Abort Programs
; MISSION PHASE: descent/landing/ascent
;
; TL;DR: Implements P70 and P71 abort programs for emergency ascent from the
;        lunar surface or during powered descent. P70 handles abort during
;        powered descent (engine failure, guidance failure, crew-initiated),
;        while P71 handles abort during final approach and landing phase.
;        Both programs immediately transition to emergency ascent guidance
;        for return to orbit without preplanned rendezvous targeting.
;
; COMMENT-ONLY READERS: These are the emergency procedures that would have
;        been used if Armstrong needed to abort the landing. Read to understand
;        the safety systems protecting the crew during descent and landing.
; CODE-ALONG READERS: Study abort detection logic, LM staging sequence, and
;        emergency ascent trajectory initialization. Note integration with
;        ALARM_AND_ABORT.agc and transition to ASCENT_GUIDANCE.agc.
; ============================================================================

# Page 829
		BANK	21
		SETLOC	R11
		BANK

		EBANK=	DVCNTR
		COUNT*	$$/R11

; ============================================================================
; TRANSITION: R10/R11 Routine with Abort Monitoring
;
; This section contains the R10/R11 periodic display update task that runs
; during powered descent and landing. Embedded within this routine is the
; critical abort monitoring logic that continuously checks for crew-initiated
; or automatic abort conditions. If the LETABORT flag is set (meaning abort
; is legal at this mission phase), the code monitors both the ABORT push
; button and the ABORT STAGE button on the LM control panel.
;
; During Apollo 11's descent on July 20, 1969, this code continuously
; monitored for abort conditions but was never triggered. Armstrong and
; Aldrin proceeded to successful landing without needing P70 or P71.
; ============================================================================

; R10,R11 - PERIODIC DISPLAY UPDATE ROUTINE
; This routine updates the LM landing displays and checks for abort
; conditions. It runs periodically during descent and landing phases.
; If LETABORT flag is set, it monitors both abort buttons continuously.

R10,R11		CS	FLAGWRD7	# IS SERVICER STILL RUNNING?
		MASK	AVEGFBIT
		CCS	A
		TCF	TASKOVER	# LET AVGEND TAKE CARE OF GROUP 2.
		CCS	PIPCTR
		TCF	+2
		TCF	LRHTASK		# LAST PASS. CALL LRHTASK.
 +2		TS	PIPCTR1

PIPCTR1		=	LADQSAVE
PIPCTR		=	PHSPRDT2
		CAF	OCT31
		TC	TWIDDLE
		ADRES	R10,R11
R10,R11A	CS	IMODES33	# IF LAMP TEST, DO NOT CHANGE LR LITES.
		MASK	BIT1
		EXTEND
		BZF	10,11

FLASHH?		MASK	FLGWRD11	# C(A) = 1 = HFLASH BIT
		EXTEND
		BZF	FLASHV?		# H FLASH OFF, SO LEAVE ALONE

		CA	HLITE
		TS	L
		TC	FLIP		# FLIP H LITE

FLASHV?		CA	VFLSHBIT	# VLASHBIT MUST BE BIT 2.
		MASK	FLGWRD11
		EXTEND
		BZF	10,11		# V FLASH OFF

		CA	VLITE
		TS	L
		TC	FLIP		# FLIP V LITE

; ABORT MONITORING LOGIC
; The following section checks if abort is currently legal (LETABORT flag).
; If abort is legal and we're not already in P70 or P71, the code monitors
; two physical buttons on the LM control panel:
;   - ABORT STAGE button (CHAN30 BIT4): Stages LM immediately (P71)
;   - ABORT button (CHAN30 BIT1): Initiates abort sequence (P70)

10,11		CA	FLAGWRD9	# IS THE LETABORT FLAG SET ?
		MASK	LETABBIT
		EXTEND
		BZF	LANDISP		# NO.  PROCEED TO R10.

; Abort is legal at this mission phase. Check if already in abort program.

P71NOW?		CS	MODREG		# YES.  ARE WE IN P71 NOW?
# Page 830
		AD	1DEC71
		EXTEND
		BZF	LANDISP		# YES.  PROCEED TO R10.

; Not in P71. Check ABORT STAGE button (most urgent abort - low altitude).
; ABORT STAGE button bypasses normal abort sequence and immediately separates
; the descent stage and ignites the ascent engine. Used during final approach
; or on the surface if immediate departure is needed.

		EXTEND			# NO.  IS AN ABORT STAGE COMMANDED?
		READ	CHAN30		# Read LM control panel discrete inputs
		COM			# Complement (buttons are active low)
		TS	L		# Save complemented value for later check
		MASK	BIT4		# Isolate ABORT STAGE button (bit 4)
		CCS	A
		TCF	P71A		# YES. ABORT STAGE BUTTON PRESSED - GO TO P71.

; ABORT STAGE button not pressed. Check if already in P70.

P70NOW?		CS	MODREG		# NO.  ARE WE IN P70 NOW?
		AD	1DEC70
		EXTEND
		BZF	LANDISP		# YES.  PROCEED TO R10.

; Not in P70 either. Check ABORT button (standard abort during descent).
; ABORT button initiates normal abort sequence with proper staging logic.
; Used when crew determines landing is unsafe but staging can be controlled.

		CA	L		# NO.  IS AN ABORT COMMANDED?
		MASK	BIT1		# Isolate ABORT button (bit 1)
		CCS	A
		TCF	P70A		# YES. ABORT BUTTON PRESSED - GO TO P70.
		TCF	LANDISP		# NO.  PROCEED TO R10.

		COUNT*	$$/P70

; ============================================================================
; P70 AND P71 ENTRY POINTS
;
; P70: Standard abort during powered descent. Used when crew determines
;      landing is not safe but conditions allow controlled staging sequence.
;      Checks LEGAL? to verify abort is permitted at current mission phase.
;
; P71: Emergency abort stage. Used during final approach or landing when
;      immediate separation and ascent is required. Also checks LEGAL? but
;      sets different mode flag (Q=2) to indicate staging abort vs standard.
;
; Both programs transfer control to CONTABRT routine via DTCB (Display Task
; Changer Bank call) to execute abort sequence in proper bank.
; ============================================================================

P70		TC	LEGAL?		# Check if abort is legal at this phase
P70A		CS	ZERO		# Q = 0 for P70 (standard abort)
		TCF	+3
P71		TC	LEGAL?		# Check if abort is legal at this phase
P71A		CAF	TWO		# Q = 2 for P71 (abort stage)
 +3		TS	Q		# Store abort mode indicator (0=P70, 2=P71)
 		INHINT			# Inhibit interrupts during mode change
		EXTEND
		DCA	CNTABTAD	# Load address of CONTABRT routine
		DTCB			# Display Task Changer Bank call

		EBANK=	DVCNTR
CNTABTAD	2CADR	CONTABRT	# Address of abort continuation routine

1DEC70		DEC	70		# P70 program number
1DEC71		DEC	71		# P71 program number

		BANK	05
		SETLOC	ABORTS1
		BANK
		COUNT*	$$/P70

; ============================================================================
; CONTABRT - ABORT CONTINUATION ROUTINE
;
; This routine is called after P70 or P71 has been initiated. It sets up
; the basic interrupt structure for abort processing, then transfers to
; ABRTJASK to configure the spacecraft systems for emergency ascent.
; ============================================================================

CONTABRT	CAF	ABRTJADR	# Load address of abort job
		TS	BRUPT		# Set up as restart job
		RESUME			# Exit interrupt, job will run
# Page 831

ABRTJADR	TCF	ABRTJASK

; ============================================================================
; ABRTJASK - ABORT JOB TASK
;
; This task executes immediately after abort is initiated. It performs
; critical system reconfiguration for emergency ascent:
;
; 1. Sets program mode register to P70 or P71
; 2. Configures phase table for abort sequence
; 3. Sets APSFLAG if P71 (ascent engine needed immediately)
; 4. Turns off DAP ullage and drift flags
; 5. Sets ENGONFLG to indicate engine-on state
; 6. Terminates landing radar updates (R12)
; 7. Suppresses cross-pointer display outputs
; 8. Initializes event timer for downlink telemetry
;
; After configuration, control passes to abort guidance initialization.
; ============================================================================

ABRTJASK	CAF	OCTAL27		# Phase value for abort (-27 octal)
		AD	Q		# Add abort mode (0=P70, 2=P71)
		TS	L
		COM
		DXCH	-PHASE4		# Set phase 4 for abort sequence
		INDEX	Q		# Index by abort mode
		CAF	MODE70		# Get P70 or P71 mode number
		TS	MODREG		# Set mode register (display shows P70/P71)

		TS	DISPDEX		# INSURE DISPDEX IS POSITIVE.

; Set APSFLAG if this is P71 (abort stage).
; APSFLAG indicates Ascent Propulsion System will be used.
; For P71, staging occurs immediately. For P70, staging follows abort logic.

		CCS	Q		# SET APSFLAG IF P71.
		CS	FLGWRD10	# SET APSFLAG PRIOR TO THE ENEMA.
		MASK	APSFLBIT	# Isolate APS flag bit
		ADS	FLGWRD10	# Set the flag
		CS	DAPBITS		# DAPBITS = OCT 640 = BITS 6, 8, 9
		MASK	DAPBOOLS	# (TURN OFF: ULLAGE, DRIFT, AND XOVINHIB )
		TS	DAPBOOLS	# Clear DAP control bits for abort

; Set ENGONFLG to indicate engine is on (or will be immediately).
; This prevents normal engine shutdown logic from interfering with abort.

		CS	FLAGWRD5	# SET ENGONFLG.
		MASK	ENGONBIT
		ADS	FLAGWRD5

; Command engine on if armed. This ensures the ascent engine (or descent
; engine if still in use) is running for abort trajectory.

		CS	PRIO30		# INSURE THAT THE ENGINE IS ON, IF ARMED.
		EXTEND
		RAND	DSALMOUT	# Read discrete alarm output channel
		AD	BIT13		# Add engine-on command bit
		EXTEND
		WRITE	DSALMOUT	# Write to engine control channel

; Terminate R12 (landing radar updates). During abort ascent, landing radar
; data is no longer needed and could provide invalid readings.

		CAF	LRBYBIT		# TERMINATE R12.
		TS	FLGWRD11	# Set landing radar bypass flag

; Suppress cross-pointer display outputs. During abort, crew attention shifts
; from landing displays to ascent trajectory and fuel management displays.

		CS	FLAGWRD0	# SET R10FLAG TO SUPPRESS OUTPUTS TO THE
		MASK	R10FLBIT	# CROSS-POINTER DISPLAY.
		ADS	FLAGWRD0	# THE FOLLOWING ENEMA WILL REMOVE THE
					# DISPLAY INERTIAL DATA OUTBIT.

		TC	CLRADMOD	# INSURE RADMODES PROPERLY SET FOR R29.

; Load current time into TEVENT for downlink telemetry.
; Ground control will receive abort initiation time to coordinate emergency
; procedures and contingency planning.

		EXTEND			# LOAD TEVENT FOR THE DOWNLINK.
		DCA	TIME2		# Get current mission time
		DXCH	TEVENT		# Store as event time for telemetry

; Set up average-g exit address for abort guidance integration.

		EXTEND
		DCA	SVEXITAD
		DXCH	AVGEXIT

# Page 832

; Clear phase tables for programs that are no longer needed during abort.
; Phase 1, 3, and 6 are set to zero (inactive) since landing guidance,
; throttle control, and display tasks are terminated for abort ascent.

		EXTEND
		DCA	NEG0
		DXCH	-PHASE1		# Clear phase 1 (landing guidance)

		EXTEND
		DCA	NEG0
		DXCH	-PHASE3		# Clear phase 3 (throttle control)

		EXTEND
		DCA	NEG0
		DXCH	-PHASE6		# Clear phase 6 (display tasks)

; Set up phase 4 spot 3 for GOABORT routine.
; GOABORT will initiate the ascent guidance and engine control for
; emergency return to orbit. Phase value -4.3 indicates abort sequence.

		CAF	THREE		# SET UP 4.3SPOT FOR GOABORT
		TS	L
		COM
		DXCH	-PHASE4		# Phase 4 spot 3 = GOABORT entry

; Schedule DAP (Digital Autopilot) idler to run in 40 milliseconds.
; This ensures attitude control is active for abort trajectory execution.

		CAF	OCT37774	# SET T5RUPT TO CALL DAPIDLER IN
		TS	TIME5		# 40 MILLISECONDS.

; Jump to ENEMA to terminate all existing display programs and major mode verbs.
; ENEMA (Display Program Killer) clears the verb/noun state machine and removes
; any landing-related displays, preparing DSKY for abort displays. After ENEMA
; completes, control passes to GOABORT for ascent guidance initialization.

		TC	POSTJUMP	# Transfer to display program terminator
		CADR	ENEMA		# ENEMA address in bank/location format

; ============================================================================
; ABORT PROGRAM CONSTANTS AND DATA
; ============================================================================

		EBANK=	DVCNTR
SVEXITAD	2CADR	SERVEXIT	# Service exit address for average-g

MODE70		DEC	70		# P70 mode number (standard abort)
OCTAL27		OCT	27		# Phase value for abort configuration
MODE71		DEC	71		# P71 mode number (abort stage)

DAPBITS		OCT	00640		# DAP control bits: 6,8,9 = ullage/drift/xovinhib

		BANK	32
		SETLOC	ABORTS
		BANK

		COUNT*	$$/P70

; ============================================================================
; GOABORT - ABORT GUIDANCE INITIALIZATION
;
; This routine is executed after ENEMA completes. It initializes the ascent
; guidance system for emergency return to orbit. GOABORT configures the
; guidance computer and digital autopilot for abort trajectory:
;
; 1. Initializes CDUW (Cross-Range Desired, Up-Range Desired, Weight)
; 2. Sets up descent velocity counter (DVCNTR) for guidance monitoring
; 3. Configures RCS jet control flags for abort maneuvering
; 4. Enables 4-jet translation capability for maximum control authority
; 5. Determines if P70 (abort with descent stage) or P71 (staged abort)
;
; After initialization, control transfers to ascent guidance (ASTNCLOK) to
; begin computing abort trajectory to achieve safe orbit for CSM rendezvous.
; ============================================================================

GOABORT		TC	INTPRET		# Enter interpretive mode
		CALL			# Initialize guidance parameters:
			INITCDUW	# Cross-range, up-range, and weight
		EXIT			# Return to native AGC code

; Set descent velocity counter to 4 for guidance computation cycles.

		CAF	FOUR
		TS	DVCNTR		# Descent velocity counter

; Load WHICH register to control guidance mode selection.

		CAF	WHICHADR
		TS	WHICH		# Guidance mode indicator

; Clear RCS control flags to prepare for abort attitude maneuvering.
; These flags control jet firing logic during abort trajectory.

		TC	DOWNFLAG	# Clear RCS jet control flag
		ADRES	FLRCS
# Page 833
		TC	DOWNFLAG	# Clear undispersed flag
		ADRES	FLUNDISP

		TC	DOWNFLAG	# Clear idle flag (autopilot active)
		ADRES	IDLEFLAG

; Enable 4-jet translation capability for maximum control authority during
; abort. This provides strongest possible attitude control for emergency ascent.

		TC	UPFLAG		# INSURE 4-JET TRANSLATION CAPABILITY.
		ADRES	ACC4-2FL	# 4-jet versus 2-jet translation flag

; Check if current mode is P70 or P71 to determine abort trajectory type.
; P70: Standard abort with descent stage still attached
; P71: Abort stage (descent stage already jettisoned, APS firing)

		TC	CHECKMM		# Check major mode register
70DEC		DEC	70		# Is mode P70?
		TCF	P71RET		# If not P70, must be P71

; ============================================================================
; P70INIT - P70 ABORT INITIALIZATION
;
; This section initializes guidance parameters specifically for P70 abort.
; P70 assumes the descent stage is still attached, so it must account for:
;
; - Descent Propulsion System (DPS) mass flow rate
; - Combined LM mass (ascent + descent stages)
; - Throttle-up burn time using DPS
; - Delta-V capability with current fuel state
; - Thrust acceleration profile for DPS
;
; These calculations determine if the abort can reach safe orbit with the
; descent engine, or if staging to APS is required during the abort sequence.
; ============================================================================

P70INIT		TC	INTPRET		# Enter interpretive mode for guidance math
		CALL			# Compute time-to-go for abort trajectory
			TGOCOMP		# Time remaining to abort orbit insertion

; Calculate TBUP (Throttle Burn-Up time) = (MDOTDPS * 16) / MASS
; This determines how long DPS can burn at current throttle setting.
; MDOTDPS = Descent engine mass flow rate (scaled)
; Shift left 4 decimal places (multiply by 16) for unit conversion.

		DLOAD	SL		# Load DPS mass flow rate
			MDOTDPS		# Descent Propulsion System fuel flow
			4D		# Shift left 4 (multiply by 16)
		BDDV			# Big divide operation
			MASS		# Current LM total mass
		STODL	TBUP		# Store throttle burn-up time
			MASS		# Reload current mass

; Calculate 1/DV (inverse delta-velocity) parameters for ascent guidance.
; 1/DV1, 1/DV2, 1/DV3 represent velocity-to-fuel ratios for guidance phases.
; K(1/DV) is constant relating mass to achievable velocity change.

		DDV	SR1		# Divide and shift right 1 bit
			K(1/DV)		# Delta-V constant (exhaust velocity)
		STORE	1/DV1		# Store for guidance phase 1
		STORE	1/DV2		# Store for guidance phase 2
		STORE	1/DV3		# Store for guidance phase 3

; Calculate AT (Acceleration Time constant) = 1/DV / K(AT)
; This determines thrust acceleration profile during abort ascent.

		BDDV			# Big divide
			K(AT)		# Acceleration time constant
		STODL	AT		# Store acceleration time
			DTDECAY		# Load decay time constant

; Calculate TTO (Time To Orbit) = -DTDECAY * 2048
; DTDECAY is engine thrust decay time. Complement and scale for guidance.

		DCOMP	SL		# Complement (negate) and shift left
			11D		# Shift 11 decimal places (multiply by 2048)
		STORE	TTO		# Store time-to-orbit estimate

; Initialize VE (exhaust velocity) for DPS engine.
; DPSVEX is DPS effective exhaust velocity constant.
; Complement and shift right to get proper scaling for guidance equations.

		SLOAD	DCOMP		# Load and complement
			DPSVEX		# DPS exhaust velocity constant
		SR2			# Shift right 2 bits (divide by 4)
		STORE	VE		# INITIALIZE DPS EXHAUST VELOCITY

; Set FLAP flag and call common abort initialization.
; FLAP flag indicates abort in progress for other AGC routines.

		SET	CALL		# Set flag bit
			FLAP		# Flight path flag (abort active)
			COMMINIT	# Common initialization subroutine

; For P70 (DPS still attached), use index 0 for DPS coefficient set.
; The BOTHPOLY routine uses polynomial coefficients to compute abort trajectory
; based on engine type (DPS or APS). Index X1 selects coefficient table.

		AXC,1	GOTO		# RETURN HERE IN P70, SET X1 FOR DPS COEFF.
			0D		# Index 0 = DPS coefficient table
			BOTHPOLY	# Jump to polynomial computation

; ============================================================================
; INJTARG - P71 ABORT INJECTION TARGETING
;
; Entry point for P71 abort (staged ascent with APS). Since descent stage is
; jettisoned, use APS (Ascent Propulsion System) coefficient set at index 8.
; ============================================================================

INJTARG		AXC,1			# RETURN HERE IN P71, SET X1 FOR APS COEFF
			8D		# Index 8 = APS coefficient table

; ============================================================================
; BOTHPOLY - ABORT TRAJECTORY POLYNOMIAL COMPUTATION
;
; Computes desired vertical velocity (ZDOTD) using 4th-order polynomial:
; ZDOTD = A + TGO*(B + TGO*(C + TGO*D))
;
; Coefficients (A,B,C,D) from ABTCOF table vary by engine type:
; - Index 0-7: DPS (Descent Propulsion System) coefficients for P70
; - Index 8-15: APS (Ascent Propulsion System) coefficients for P71
;
; TGO (Time-to-Go) is computed by TGOCOMP and represents seconds remaining
; until abort orbit insertion. Polynomial gives optimal vertical velocity
; profile for fuel-efficient abort trajectory to safe orbit.
; ============================================================================

BOTHPOLY	DLOAD*	DMP		# TGO D: Load D coefficient, multiply by TGO
			ABTCOF,1	# Coefficient D from indexed table
			TGO		# Time-to-go for abort trajectory
# Page 834
		DAD*	DMP		# TGO(C+TGO D): Add C, multiply by TGO
			ABTCOF +2,1	# Coefficient C from indexed table
			TGO		# Time-to-go
		DAD*	DMP		# TGO(B+TGO(C+TGO D)): Add B, multiply by TGO
			ABTCOF +4,1	# Coefficient B from indexed table
			TGO		# Time-to-go
		DAD*			# A+TGO(B+TGO(C+TGO D)): Add A (final result)
			ABTCOF +6,1	# Coefficient A from indexed table
		STORE	ZDOTD		# STORE TENTATIVELY IN ZDOTD

; Check computed ZDOTD against minimum acceptable vertical velocity.
; If below minimum, use minimum value to ensure abort reaches safe altitude.
; This prevents abort trajectories with insufficient vertical rise.

		DSU	BPL		# CHECK AGAINST MINIMUM (subtract, branch if plus)
			VMIN		# Minimum acceptable vertical velocity
			UPRATE		# IF BIG ENOUGH, LEAVE ZDOTD AS IS .

; ZDOTD is too small - replace with minimum value to ensure safe abort.
; This guarantees sufficient altitude gain during emergency ascent.

		DLOAD			# Load minimum velocity
			VMIN		# Minimum vertical velocity threshold
		STORE	ZDOTD		# IF TOO SMALL, REPLACE WITH MINIMUM.

; ============================================================================
; UPRATE - INITIALIZE RADIAL RATE AND COMPUTE CROSSRANGE
;
; Now that vertical velocity ZDOTD is validated, initialize other abort
; trajectory parameters:
; - RDOTD: Desired radial velocity rate (from abort table)
; - Y: Crossrange position (computed by YCOMP)
; - YCO: Crossrange correction for trajectory shaping
; - XRANGE: Downrange distance for crew situational awareness
; ============================================================================

UPRATE		DLOAD			# Load abort radial rate
			ABTRDOT		# Radial velocity component from table
		STCALL	RDOTD		# INITIALIZE RDOTD and call YCOMP
			YCOMP		# COMPUTE Y (crossrange position)

; Compute YCO (crossrange correction) to shape abort trajectory.
; If crossrange angle exceeds limit (YLIM), apply correction.
; Otherwise leave YCO at zero (no crossrange correction needed).

		ABS	DSU		# Absolute value of Y, subtract limit
			YLIM		# /Y/-DYMAX (crossrange limit check)
		BMN	SIGN		# IF <0, XR<.5DEG, LEAVE YCO AT 0
			YOK		# If within limits, skip correction
			Y		# Fix sign of deficit using Y's sign
		STORE	YCO		# Store crossrange correction

; Compute XRANGE (downrange distance) for crew display.
; XRANGE = (YCO - Y) / 32 provides distance estimate.
; This gives astronauts situational awareness of abort trajectory.

YOK		DLOAD	DSU		# Load YCO
			YCO		# Crossrange correction
			Y		# COMPUTE XRANGE IN CASE ASTRONAUT WANTS
		SR			# Shift right (divide)
			5D		# Divide by 32 for display scaling
		STORE	XRANGE		# TO LOOK (store for crew reference)

; ============================================================================
; UPTHROT - THROTTLE UP AND CONFIGURE FOR ASCENT
;
; With abort trajectory computed, now configure systems for ascent:
; 1. Set FLVR flag (flight phase flag for abort/ascent)
; 2. Set ROTFLAG (enable rotation to abort attitude)
; 3. Throttle up engine (via THROTUP routine)
; 4. Verify panel switches (via P40AUTO)
; 5. Configure servicer to call ascent guidance (ATMAG)
; ============================================================================

UPTHROT		SET	EXIT		# Set flag and exit interpretive mode
			FLVR		# Flight phase flag (abort in progress)

; Set ROTFLAG to enable rotation to abort attitude.
; This allows autopilot to orient LM for vertical ascent.

		TC	UPFLAG		# SET ROTFLAG (flag setting routine)
		ADRES	ROTFLAG		# Address of rotation enable flag

; First throttle up call - initial engine command.
; THROTUP increases engine thrust to begin ascent.

		TC	THROTUP		# Throttle up ascent engine

; Set restart protection for this abort phase.

		TC	PHASCHNG	# Phase change for restart protection
		OCT	04024		# Phase code for abort guidance

; Verify panel switches are properly configured for abort.
; P40AUTO checks engine arming switches and abort staging switches.
; Crew must have ABORT STAGE button guarded (not pressed yet for P70).

 -3		TC	BANKCALL	# VERIFY THAT THE PANEL SWITCHES
		CADR	P40AUTO		# ARE PROPERLY SET (cross-bank call)

; Second throttle up call - ensure engine at full thrust.
; Redundant call provides safety margin for critical abort.

		TC	THROTUP		# Throttle up again (redundancy)

; Configure servicer routine to repeatedly call ascent guidance.
; ATMAGAD is address of ATMAG guidance routine.
; AVGEXIT is the servicer exit address - guidance runs in loop.
; This is the main guidance loop that steers abort to safe orbit.

UPTHROT1	EXTEND			# SET SERVICER TO CALL ASCENT GUIDANCE
		DCA	ATMAGAD		# Load double-word ATMAG address
		DXCH	AVGEXIT		# Store in AVGEXIT (servicer exit point)
# Page 835

; ============================================================================
; GRP4OFF - TERMINATE GROUP 4 RESTART PROTECTION
;
; Abort guidance initialization complete. Terminate group 4 restart
; protection and end this job. Guidance loop now runs via servicer.
; ============================================================================

GRP4OFF		TC	PHASCHNG	# TERMINATE USE OF GROUP 4
		OCT	00004		# Group 4 termination code

		TCF	ENDOFJOB	# End this job (guidance continues via servicer)

; ============================================================================
; P71RET - RETURN FROM P71 STAGED ABORT
;
; Called after descent stage separation and ascent stage ignition.
; This routine:
; 1. Clears LETABORT flag (abort decision complete, now in ascent)
; 2. Sets DVMON threshold for ascent ΔV monitoring
; 3. Computes time-to-go (TGO) for ascent guidance
; 4. Continues to UPTHROT1 to enter ascent guidance loop
;
; P71 staging sequence: Crew presses ABORT STAGE → descent stage separates →
; explosive bolts fire → ascent engine ignites → P71RET configures for ascent
; ============================================================================

P71RET		TC	DOWNFLAG	# Clear abort flag
		ADRES	LETABORT	# LETABORT=0 (abort complete, in ascent)

; Set ΔV monitoring threshold for ascent phase.
; THRESH2 is ascent-phase threshold (different from descent).
; DVTHRUSH monitors guidance steering errors.

		CAF	THRESH2		# SET DVMON THRESHOLD TO THE ASCENT VALUE
		TS	DVTHRUSH	# Store ascent threshold

; Compute TGO (time-to-go) for ascent guidance.
; If FLAP=0: TGO = current time - TIG (ignition time)
; If FLAP=1: TGO = 2 * TGO (double the time-to-go for extended burn)

		TC	INTPRET		# Enter interpretive mode
		BON	CALL		# Branch on flag
			FLAP		# FLAP flag check
			OLDTIME		# If FLAP=1, go to OLDTIME
			TGOCOMP		# IF FLAP=0, TGO=T-TIG (compute from ignition)
		SSP	GOTO		# Set pointer and go
			QPRET		# Return address pointer
		CADR	INJTARG		# Return to INJTARG
			P12INIT		# WILL EXIT P12INIT TO INJTARG

; OLDTIME path: FLAP=1, use doubled TGO value.

OLDTIME		DLOAD	SL1		# IF FLAP=1, TGO=2*TGO (load and shift left)
			TGO		# Current time-to-go
		STCALL	TGO1		# Store in TGO1 and call
			P12INIT		# Initialize P12 ascent parameters

; Exit interpretive mode and restore TGO, then continue to guidance loop.

		EXIT			# Return to native AGC code
		TC	PHASCHNG	# Set restart protection
		OCT	04024		# Phase code

		EXTEND			# Extended instruction
		DCA	TGO1		# Load double-word TGO1
		DXCH	TGO		# Store in TGO
		TCF	UPTHROT1 -3	# Jump to guidance loop setup

TGO1		=	VGBODY		# TGO1 uses VGBODY storage location
# *************************************************************************

		BANK	21
		SETLOC	R11
		BANK

		COUNT*	$$/P70

; ============================================================================
; LEGAL? - ABORT LEGALITY CHECK
;
; Called when crew requests abort (presses ABORT or ABORT STAGE button).
; Verifies abort conditions are legal before allowing abort initiation:
; 1. Requested program not already running (avoid duplicate abort calls)
; 2. LETABORT flag set (abort window enabled - descent/landing only)
; 3. Servicer running (guidance system operational)
;
; If any check fails, displays operator error and rejects abort request.
; This prevents aborts during inappropriate mission phases (e.g., during
; nominal P12 ascent, during trans-lunar coast, etc.)
; ============================================================================

LEGAL?		CS	MMNUMBER	# IS THE DESIRED PGM ALREADY IN PROGRESS?
		AD	MODREG		# Compare requested program to current
		EXTEND			# Extended instruction
		BZF	ABORTALM	# If equal, abort request invalid

; Check if abort window is open (LETABORT flag set during descent/landing).

		CS	FLAGWRD9	# ARE THE ABORTS ENABLED?
		MASK	LETABBIT	# Check LETABORT flag bit
		CCS	A		# Check sign
# Page 836
		TCF	ABORTALM	# If not set, abort not allowed

; Check if servicer is running (guidance operational).
; Servicer must be active for abort guidance to function.

		CA	FLAGWRD7	# IS SERVICER ON THE AIR?
		MASK	AVEGFBIT	# Check AVEGFBIT (servicer active flag)
		CCS	A		# Check sign
		TC	Q		# YES. ALL IS WELL (return, abort legal)

; Abort request illegal. Display operator error and return to PINBRNCH.

ABORTALM	TC	FALTON		# Flash operator error light
		TC	RELDSP		# Release display
		TC	POSTJUMP	# Post jump to pinball
		CADR	PINBRNCH	# Return to pinball branch

		BANK	32		# Switch to bank 32
		SETLOC	ABORTS		# Set location counter (ABORTS section)
		BANK

		COUNT*	$$/P70

# ************************************************************************

; ============================================================================
; TGOCOMP - TIME-TO-GO COMPUTATION (Alternate Implementation)
;
; Computes TGO (time-to-go) for abort guidance from current mission time.
; TGO = (Current Time - TIG) * 2048 (scaled for guidance equations).
; This version uses LOADTIME to get current mission time from AGC clock.
;
; Called from P71RET when FLAP=0 (immediate abort, not staged abort).
; Provides initial TGO for ascent guidance steering computations.
; ============================================================================

TGOCOMP		RTB	DSU		# Return to basic, double subtract
			LOADTIME	# Load current mission time
			TIG		# Subtract ignition time
		SL			# Shift left (scale up)
			11D		# Shift 11 positions (multiply by 2048)
		STORE	TGO		# Store result in TGO
		RVQ			# Return via Q register

# ************************************************************************

; ============================================================================
; THROTUP - THROTTLE UP ENGINE (Simplified Implementation)
;
; Commands ascent engine to throttle up for abort.
; Direct engine control via output channels (not through COMMINGL).
;
; Sequence:
; 1. Load BIT13 into THRUST variable (throttle command magnitude)
; 2. Set BIT4 in CHAN14 (engine start/run command to hardware)
;
; This simplified version provides immediate engine response for abort.
; BIT4 in CHAN14 is the APS (Ascent Propulsion System) enable signal.
; THRUST value controls throttle percentage through engine interface.
; ============================================================================

THROTUP		CAF	BIT13		# Load throttle command constant
		TS	THRUST		# Store in THRUST variable
		CAF	BIT4		# Load engine enable bit
		EXTEND			# Extended instruction
		WOR	CHAN14		# Write OR to channel 14 (engine control)
		TC	Q		# Return to caller

# ************************************************************************

; ============================================================================
; CONSTANTS FOR ABORT PROGRAMS
;
; Various constants used throughout P70/P71 abort logic.
; All constants scaled appropriately for AGC fixed-point arithmetic.
; ============================================================================

10SECS		2DEC	1000		# 10 seconds in centiseconds (10 * 100)

; HINJECT: Target orbital insertion altitude for abort ascent.
; 60,000 feet = 18,288 meters, scaled by 2^-24 for fixed-point representation.
; This represents the minimum safe altitude for orbital insertion after abort,
; providing clearance over lunar terrain and establishing stable orbit.

HINJECT		2DEC	18288 B-24	# 60,000 FEET EXPRESSED IN METERS.

; (TGO)A: Nominal time-to-go constant for abort trajectory initialization.
; 370 seconds (scaled by 2^-17) = typical ascent duration to orbital insertion.
; Used in BOTHPOLY routine for initial trajectory polynomial computation.

(TGO)A		2DEC	37000 B-17	# Nominal TGO for abort ascent

; K(AT): Scaling constant for acceleration-time product calculations.
; Value 0.02 used in guidance equation scaling throughout abort computations.
; Converts between different coordinate frame representations.

K(AT)		2DEC	.02		# SCALING CONSTANT

; WHICHADR: Remainder address pointer to ABRTABLE.
; Used for indexed access to abort trajectory parameter tables.

WHICHADR	REMADR	ABRTABLE	# Abort table address

# ************************************************************************
# Page 837

; ============================================================================
; ROUTINE ADDRESSES AND POINTERS
;
; 2CADR (two-complement address) pointers for cross-bank routine calls.
; ADRES (address) pointers for same-bank routine references.
; EBANK setting ensures proper erasable memory bank for data access.
; ============================================================================

		EBANK=	DVCNTR		# Erasable bank for DVCNTR variables

; ATMAGAD: Address of ATMAG (atmosphere guidance) routine.
; ATMAG is the main ascent guidance loop called continuously during abort.
; Computes steering commands, monitors trajectory, and updates displays.

ATMAGAD		2CADR	ATMAG		# Ascent guidance routine address

; ORBMANAD: Address of ORBMANUV (orbital maneuver) routine.
; Used for post-abort orbit correction maneuvers if needed.

ORBMANAD	ADRES	ORBMANUV	# Orbital maneuver routine address
