# Copyright:	Public domain.
# Filename:	LUNAR_LANDING_GUIDANCE_EQUATIONS.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	HARTMUTH GUTSCHE <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	798-828
# Mod history:	2009-05-23 HG	Transcribed from page images.
#		2009-06-05 RSB	Fixed a goofy thing that was apparently
#				legal in GAP but not in yaYUL.  Eliminated
#				a couple of lines of code that shouldn't
#				have survived from Luminary 131 to here.
#		2009-06-07 RSB	Fixed a typo.
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

# Page 798

; ============================================================================
; FILE: LUNAR_LANDING_GUIDANCE_EQUATIONS.agc
; MODULE: Lunar Landing Guidance
; MISSION PHASE: descent/landing
;
; TL;DR: Computes fuel-optimal descent trajectory guidance equations from 50,000
;        feet to lunar surface. Implements gravity turn, constant deceleration,
;        and visibility phases. Generates thrust magnitude and direction commands
;        for throttle control and attitude autopilot during powered descent.
;
; COMMENT-ONLY READERS: This is the mathematics behind the automated descent that
;        brought Eagle safely to the Sea of Tranquility landing site.
; CODE-ALONG READERS: Study trajectory optimization algorithms, fixed-point
;        scaled arithmetic for position/velocity vectors, and integration with
;        throttle/attitude control systems.
; ============================================================================

		EBANK=	E2DPS
		COUNT*	$$/F2DPS

; ============================================================================
; LUNAR LANDING FLIGHT SEQUENCE TABLES
; ============================================================================
;
; The descent from 50,000 feet to touchdown is divided into distinct phases,
; each requiring different guidance strategies. The computer automatically
; transitions between phases based on altitude and velocity conditions.
;
; WCHPHASE Register Controls Current Guidance Phase:
;   -1 = IGNALG    (Ignition Algorithm - initial descent engine start)
;    0 = BRAKQUAD  (Braking Phase - high-altitude deceleration from 50K ft)
;    1 = APPRQUAD  (Approach Phase - visibility window for landing site selection)
;    2 = VERTICAL  (Vertical Descent - final low-altitude vertical landing)
;
; Each phase has associated routines for initialization, pre-guidance computation,
; main guidance equations, post-guidance computation, window vectors, displays,
; and alarm handling. These tables use WCHPHASE as an index to vector to the
; appropriate routine for the current descent phase.
;
; Historical Context: During Apollo 11's descent on July 20, 1969, these guidance
; equations executed continuously from powered descent initiation at 102:33 MET
; through touchdown at 102:45:40 MET, computing updated thrust commands every 2
; seconds to guide Eagle to the lunar surface.
; ============================================================================

# ********************************************************
# LUNAR LANDING FLIGHT SEQUENCE TABLES
# ********************************************************

# FLIGHT SEQUENCE TABLES ARE ARRANGED BY FUNCTION.  THEY ARE REFERENCED USING AS AN INDEX THE REGISTER WCHPHASE:
#	WCHPHASE = -1 ---> IGNALG
#	WCHPHASE =  0 ---> BRAKQUAD
#	WCHPHASE =  1 ---> APPRQUAD
#	WCHPHASE =  2 ---> VERTICAL

#*********************************************************

; ============================================================================
; PHASE INITIALIZATION ROUTINES
; ============================================================================
; When the guidance computer detects conditions requiring a phase transition
; (e.g., altitude threshold crossed, velocity target achieved), it vectors
; through this table to initialize the new guidance phase.
; ============================================================================

# ROUTINES FOR STARTING NEW GUIDANCE PHASES:

		TCF	TTFINCR		# IGNALG
NEWPHASE	TCF	TTFINCR		# BRAKQUAD
		TCF	STARTP64	# APPRQUAD (P64 - Approach phase with visibility)
		TCF	P65START	# VERTICAL (P65 - Final vertical descent)

; ============================================================================
; PRE-GUIDANCE POSITION AND VELOCITY COMPUTATIONS
; ============================================================================
; Before computing guidance commands, the computer must calculate position
; relative to landing site (RG) and velocity-to-be-gained (VG). These vectors
; are fundamental inputs to the fuel-optimal guidance law.
;
; REDESIG routine (Approach phase): Allows crew or computer to select new
; landing site coordinates if original site appears unsuitable (e.g., boulder
; field detected). Armstrong used semi-manual control during this phase to
; select the final Apollo 11 landing site.
; ============================================================================

# PRE-GUIDANCE COMPUTATIONS:

		TCF	CALCRGVG	# IGNALG
PREGUIDE	TCF	RGVGCALC	# BRAKQUAD
		TCF	REDESIG		# APPRQUAD (Landing site redesignation)
		TCF	RGVGCALC	# VERTICAL

; ============================================================================
; MAIN GUIDANCE EQUATION COMPUTATIONS
; ============================================================================
; TTF/8CL: Time-To-Go divided by 8, Closed Loop guidance law
; This implements the core fuel-optimal guidance algorithm derived from
; calculus of variations. Computes thrust acceleration magnitude and direction
; to achieve desired velocity at predicted landing time while minimizing fuel
; consumption. Updated every 2 seconds during descent.
;
; VERTGUID: Specialized guidance for final vertical descent phase where
; horizontal velocity is nulled and only vertical descent rate is controlled.
; ============================================================================

# GUIDANCE EQUATIONS:

		TCF	TTF/8CL		# IGNALG
WHATGUID	TCF	TTF/8CL		# BRAKQUAD (Fuel-optimal guidance)
		TCF	TTF/8CL		# APPRQUAD (Continues optimal guidance)
		TCF	VERTGUID	# VERTICAL (Vertical-only guidance)

; ============================================================================
; POST-GUIDANCE COMPUTATIONS
; ============================================================================
; CGCALC: Compute commanded acceleration vector (thrust magnitude and direction)
; from guidance solution. Output scaled for throttle control and autopilot.
;
; STEER?: Determines if attitude autopilot should execute steering commands
; or maintain current attitude during final vertical descent.
; ============================================================================

# POST GUIDANCE EQUATION COMPUTATIONS:

		TCF	CGCALC		# IGNALG
AFTRGUID	TCF	CGCALC		# BRAKQUAD
		TCF	CGCALC		# APPRQUAD
		TCF	STEER?		# VERTICAL

# Page 799

; ============================================================================
; WINDOW VECTOR EXIT CONDITION CHECKS
; ============================================================================
; These routines monitor altitude, velocity, and other parameters to determine
; when current guidance phase should terminate and next phase should begin.
; Exit conditions ensure smooth phase transitions without discontinuities in
; commanded thrust or attitude.
; ============================================================================

# WINDOW VECTOR COMPUTATIONS:

		TCF	EXGSUB		# IGNALG
WHATEXIT	TCF	EXBRAK		# BRAKQUAD (Exit braking when approach altitude reached)
		TCF	EXNORM		# APPRQUAD (Exit approach when vertical descent altitude reached)

; ============================================================================
; CREW DISPLAY UPDATE ROUTINES
; ============================================================================
; During descent, the DSKY continuously displays altitude, altitude rate,
; horizontal and vertical velocity to the crew. These routines format the
; navigation state data for display. Buzz Aldrin monitored these readouts
; and called them out to Armstrong during Apollo 11's final approach.
; ============================================================================

# DISPLAY ROUTINES:

WHATDISP	TCF	P63DISPS	# BRAKQUAD (P63 displays: altitude, velocity)
		TCF	P64DISPS	# APPRQUAD (P64 displays: add landing site data)
		TCF	VERTDISP	# VERTICAL (P66 displays: descent rate, throttle)

; ============================================================================
; TIME-TO-GO COMPUTATION ALARM HANDLING
; ============================================================================
; If TTF (Time-To-Go) computation fails to converge or produces unreasonable
; values, these alarm routines are invoked. Program alarm 1406 indicates
; guidance computation anomaly requiring crew attention.
; ============================================================================

# ALARM ROUTINE FOR TTF COMPUTATION:

		TCF	1406P00		# IGNALG
WHATALM		TCF	1406ALM		# BRAKQUAD
		TCF	1406ALM		# APPRQUAD

; ============================================================================
; TARGET PARAMETER TABLE INDICES
; ============================================================================
; Different descent phases use different target parameter sets. These indices
; reference the appropriate parameter tables for landing site coordinates,
; desired final velocity, and altitude thresholds.
; ============================================================================

# INDICES FOR REFERENCING TARGET PARAMETERS

		OCT	0		# IGNALG
TARGTDEX	OCT	0		# BRAKQUAD
		OCT	34		# APPRQUAD (Offset for approach phase target parameters)

; ============================================================================
; MAIN GUIDANCE ENTRY POINTS
; ============================================================================
; Two primary entry points into the guidance computation system:
;
; ?GUIDSUB: Ignition algorithm entry for initial descent engine start.
;           Executes N=3 passes of quadratic guidance to establish stable
;           descent trajectory immediately after powered descent initiation.
;           Called during transition from free-fall coast to powered descent.
;
; LUNLAND:  Normal entry point called by SERVOUT every 2 seconds during
;           powered descent. Computes updated guidance commands based on
;           current position, velocity, and landing site target. This is
;           the main computational heartbeat of the lunar landing.
;
; Historical Context: From PDI at 102:33 MET through touchdown at 102:45:40,
; LUNLAND executed approximately 380 times, continuously refining the descent
; trajectory as Eagle descended toward the Sea of Tranquility.
; ============================================================================

#************************************************************************
# ENTRY POINTS:  ?GUIDSUB FOR THE IGNITION ALGORITHM, LUNLAND FOR SERVOUT
#************************************************************************

; Ignition Algorithm Entry - Initial Descent Stabilization
; Performs multiple guidance passes to establish smooth transition from
; orbital coast to powered descent under thrust.

# IGNITION ALGORITHM ENTRY:  DELIVERS N PASSES OF QUADRATIC GUIDANCE

?GUIDSUB	EXIT			; Exit interpreter mode to native AGC code
		CAF	TWO		# N = 3 (Load constant TWO for 3 passes)
		TS	NGUIDSUB	; Store number of guidance sub-iterations
		TCF	GUILDRET +2	; Transfer control to guidance computation

GUIDSUB		TS	NGUIDSUB	# ON SUCCEEDING PASSES SKIP TTFINCR
		TCF	CALCRGVG	; Calculate position (RG) and velocity (VG) vectors

; ============================================================================
; LUNLAND - Primary Guidance Computation Entry Point
; ============================================================================
; Called by SERVOUT task every 2 seconds during powered descent. This routine:
; 1. Sets restart protection (phase change) to preserve guidance state
; 2. Elevates task priority to 21 (higher than background servicer tasks)
; 3. Protects critical guidance computations from lower-priority interrupts
;
; The phase change mechanism ensures that if a restart occurs mid-computation
; (e.g., due to program alarm or power transient), the guidance calculation
; can resume without loss of state. Critical during 1202 alarm events.
; ============================================================================

# NORMAL ENTRY:  CONTROL COMES HERE FROM SERVOUT

LUNLAND		TC	PHASCHNG	; Initiate phase change for restart protection
		OCT	00035		# GROUP 5:  RETAIN ONLY PIPA TASK (IMU data collection)
		TC	PHASCHNG	; Second phase change to set priority
		OCT	05023		# GROUP 3:  PROTECT GUIDANCE WITH PRIO 21
		OCT	21000		#	JUST HIGHER THAN SERVICER'S PRIORITY

# Page 800

; ============================================================================
; GUILDENSTERN - AUTO-MODES MONITOR (R13)
; ============================================================================
; Monitors crew manual control inputs and automatically selects appropriate
; guidance program based on discrete switch positions:
;
; Manual Throttle Switch:
;   - Switch ON (un-auto-throttle discrete):  Select P67 (manual throttle mode)
;   - Switch OFF (auto-throttle):             Select P66 (automatic throttle)
;
; Attitude Hold Switch:
;   - Switch ON: Select P66 (rate-of-descent control) unless already in P67
;
; Program Modes:
;   P66: Automatic guidance with crew-adjustable descent rate (ROD control)
;   P67: Full manual throttle control, computer provides guidance recommendations
;
; Crew Context: During Apollo 11 final approach, Armstrong used semi-manual
; attitude control while the computer maintained automatic throttle. This
; monitor ensured seamless transitions between auto and manual control modes.
; ============================================================================

#*******************************************************************
# GUILDENSTERN:  AUTO-MODES MONITOR (R13)
#*******************************************************************

		COUNT*	$$/R13

# HERE IS THE PHILOSOPHY OF GUILDENSTERN:	ON EVERY APPEARANCE OR DISAPPEARANCE OF THE MANUAL THROTTLE
# DISCRETE TO SELECT P67 OR P66 RESPECTIVELY:   ON EVERY APPEARANCE OF THE ATTITUDE-HOLD DISCRETE TO SELECT P66
# UNLESS THE CURRENT PROGRAM IS P67 IN WHICH CASE THERE IS NO CHANGE

; Check Manual Throttle Discrete Status
; Reads I/O channel 30, bit 5 to detect crew throttle switch position.
; If manual throttle engaged, transition to P67 for crew control.

GUILDEN		EXTEND			# IS UN-AUTO-THROTTLE DISCRETE PRESENT?
# STERN					# RSB 2009: Not originally a comment.
 		READ CHAN30		; Read channel 30 (crew control discretes)
		MASK	BIT5		; Isolate bit 5 (manual throttle discrete)
 		CCS	A		; Check if bit is set
 		TCF	STARTP67	# YES - Manual throttle requested, start P67
P67NOW?		TC	CHECKMM		# NO:  ARE WE IN P67 NOW?
		DEC	67		; Check if major mode is 67
		TCF	STABL?		# NO - Check attitude hold discrete
		
; ============================================================================
; STARTP66 - Initialize Rate-of-Descent Control Mode
; ============================================================================
; Transitions to P66 automatic guidance with crew-adjustable descent rate.
; Preserves current altitude rate as initial desired rate, allowing crew
; to smoothly adjust descent velocity using ROD control during final approach.
;
; This mode was available but not extensively used during Apollo 11 landing,
; as Armstrong preferred more direct attitude control during site selection.
; ============================================================================

STARTP66	TC	FASTCHNG	# YES - Fast phase change for mode transition
		TC	NEWMODEX	; Set new major mode
DEC66		DEC	66		; Major mode 66
		EXTEND
		DCA	HDOTDISP	# SET DESIRED ALTITUDE RATE = CURRENT
		DXCH	VDGVERT		# 	ALTITUDE RATE (smooth transition)
		
; P66 Initialization - Configure Bias Terms and ROD Control
; Sets up rate-of-descent control parameters and initializes guidance
; bias terms for smooth transition from previous guidance mode.

STRTP66A	TC	INTPRET		; Enter interpretive mode for vector operations
		SLOAD	PUSH
			PBIASZ
		SLOAD	PUSH
			PBIASY
		SLOAD	VDEF
			PBIASX
		VXSC	SET
			BIASFACT
			RODFLAG
		STOVL	VBIAS
			TEMX
		VCOMP
		STOVL	OLDPIPAX
			ZEROVECS
		STODL	DELVROD
			RODSCALE
		STODL	RODSCAL1
			PIPTIME
		STORE	LASTTPIP
		EXIT
		CAF	ZERO
		TS	FCOLD
		TS	FWEIGHT
		TS	FWEIGHT +1
VRTSTART	TS	WCHVERT
# Page 801
		CAF	TWO		# WCHPHASE = 2 ---> VERTICAL: P65,P66,P67
		TS	WCHPHOLD
		TS	WCHPHASE
		TC	BANKCALL	# TEMPORARY, I HOPE HOPE HOPE
		CADR	STOPRATE	# TEMPORARY, I HOPE HOPE HOPE
		TC	DOWNFLAG	# PERMIT X-AXIS OVERRIDE
		ADRES	XOVINFLG	; Clear X-axis override flag
		TC	DOWNFLAG	; Clear redesignation flag
		ADRES	REDFLAG		; Indicates no landing site redesignation
		TCF	VERTGUID	; Continue to vertical guidance computation

; ============================================================================
; STARTP67 - Initialize Manual Throttle Mode
; ============================================================================
; Crew has selected manual throttle control. Computer provides guidance
; recommendations but throttle is under direct crew control via Thrust/
; Translation Controller. Armstrong used this mode during final site
; selection when computer-targeted site proved unsuitable (boulder field).
; ============================================================================

STARTP67	TC	NEWMODEX	# NO HARM IN "STARTING" P67 OVER AND OVER
		DEC	67		# SO NO NEED FOR A FASTCHNG AND NO NEED
		CAF	ZERO		# TO SEE IF ALREADY IN P67.
		TS	RODCOUNT	; Clear rate-of-descent counter
		CAF	TEN		; Set countdown value
		TCF	VRTSTART	; Initialize vertical guidance parameters

; Check Attitude Hold Discrete
; Verifies if crew has attitude hold switch active. If attitude hold is
; disengaged (switch released), transition to P66 rate-of-descent mode.

STABL?		CAF	BIT13		# IS UN-ATTITUDE-HOLD DISCRETE PRESENT?
		EXTEND
		RAND	CHAN31		; Read channel 31 (attitude discrete)
		CCS	A		; Check if bit is set
		TCF	GUILDRET	# YES ALL'S WELL - continue current mode

; Check Current Mode and ROD Switch Status
; Determines if already in P66 and checks rate-of-descent switch activity.
; ROD switch allows crew to adjust descent rate in P66 mode.

P66NOW?		CS	MODREG		; Load complement of mode register
		AD	DEC66		; Add 66 to check if in P66
		EXTEND
		BZF	RESTART?	; Zero = already in P66, check restart

		CA	RODCOUNT	# NO. HAS THE ROD SWITCH BEEN "CLICKED"?
		EXTEND
		BZF	GUILDRET	# NO. CONTINUE WITH AUTOMATIC LANDING
		TCF	STARTP66	# YES. SWITCH INTO THE ROD MODE.

; Restart Check
; After program alarm restart, reinitialize P66 but preserve crew-set
; desired descent rate (VDGVERT). Critical for maintaining continuity
; during 1202 alarm events - crew settings are not lost.

RESTART?	CA	FLAGWRD1	# HAS THERE BEEN A RESTART?
		MASK	RODFLBIT	; Check restart flag bit
		EXTEND
		BZF	STRTP66A	# YES.  REINITIALIZE BUT LEAVE VDGVERT AS
					#	IS (preserve crew-set descent rate).

		TCF	VERTGUID	# NO: CONTINUE WITH R.O.D. (normal operation)

; ============================================================================
; GUIDANCE PASS INITIALIZATION
; ============================================================================
; Initializes time references and state variables for current guidance cycle.
; Called every 2 seconds by SERVOUT during powered descent.
;
; Key initializations:
;   - Update PIPA time stamps (inertial measurement timing)
;   - Save previous TTF/8 (time-to-go divided by 8) for derivatives
;   - Clear ROD counter if returning from mode check
;   - Check for first-pass flag to determine initialization path
; ============================================================================

# *******************************************************************************
# INITIALIZATION FOR THIS PASS
# *******************************************************************************

		COUNT*	$$/F2DPS

GUILDRET	CAF	ZERO		; Clear accumulator
		TS	RODCOUNT	; Reset rate-of-descent switch counter

# Page 802
; Save Previous PIPA Time for Delta-T Calculations
; PIPA (Pulsed Integrating Pendulous Accelerometer) timing is critical for
; accurate velocity integration. Time differences drive dead-reckoning
; navigation between radar updates.

 +2		EXTEND
 		DCA	TPIP		; Load current PIPA time (double precision)
		DXCH	TPIPOLD		; Store as "old" time for next pass

		TC	FASTCHNG	; Fast phase change for time updates

		EXTEND
		DCA	PIPTIME1	; Load latest PIPA sample time
		DXCH	TPIP		; Update current PIPA time

; Save Previous Time-to-Go for Rate Calculations
; TTF/8 (time-to-go divided by 8) is scaled for computational precision.
; Saving previous value enables computation of time-to-go rate (TTF dot).

		EXTEND
		DCA	TTF/8		; Load current time-to-go / 8
		DXCH	TTF/8TMP	; Store as temporary for computations

; Check First Pass Flag
; FLPASS0 indicates whether this is first guidance pass after phase change.
; First pass requires phase initialization, subsequent passes continue.

		CCS	FLPASS0		; Check first-pass flag
		TCF	TTFINCR		; Not first pass, increment TTF

BRSPOT1		INDEX	WCHPHASE	; Use phase index to select initialization
		TCF	NEWPHASE	; Branch to phase-specific start routine

; ============================================================================
; PHASE INITIALIZATION ROUTINES
; ============================================================================
; Each descent phase (Braking, Approach, Vertical) requires specific
; initialization when entered. These routines set major mode, configure
; flags, and establish phase-specific parameters.
; ============================================================================

# ******************************************************************
# ROUTINES TO START NEW PHASES
# ******************************************************************

; P65 Start - Vertical Descent Phase Initialization
; Final phase before touchdown. Vertical descent with minimal horizontal
; velocity, crew monitoring for smooth touchdown at < 3 ft/sec.

P65START	TC	NEWMODEX	; Set new major mode
		DEC	65		; Major mode 65
		CS	TWO		; Load -2
		TS	WCHVERT		; Set vertical phase indicator
		TC	DOWNFLAG	# PERMIT X-AXIS OVERRIDE
		ADRES	XOVINFLG	; Allow crew X-axis attitude override
		TCF	TTFINCR		; Continue to TTF increment

; P64 Start - Approach Phase Initialization  
; Transition from braking to final approach. Visibility phase for crew
; terrain observation and landing site evaluation. Armstrong used this
; phase to identify unsuitable computer-targeted site and select alternate.

STARTP64	TC	NEWMODEX	; Set new major mode
		DEC	64		; Major mode 64 (Approach phase)
		CA	DELTTFAP	# AUGMENT TTF/8
		ADS	TTF/8TMP	; Add approach phase TTF adjustment
		CA	BIT12		# ENABLE RUPT10
		EXTEND
		WOR	CHAN13		; Write OR to channel 13 (enable interrupt 10)
		TC	DOWNFLAG	# INITIALIZE REDESIGNATION FLAG
		ADRES	REDFLAG		; Clear redesignation flag (no site change yet)


#		(CONTINUE TO TTFINCR)

# *********************************************************************************
# INCREMENT TTF/8, UPDATE LAND FOR LUNAR ROTATION, DO OTHER USEFUL THINGS
# *********************************************************************************
#
#	TTFINCR COMPUTATIONS ARE AS FOLLOWS --
# Page 803
#		TTF/8 UPDATED FOR TIME SINCE LAST PASS:
; ============================================================================
; TTFINCR - Time-to-Go Increment and Landing Site Update
; ============================================================================
; Updates time-to-go (TTF/8) by adding elapsed time since last guidance pass.
; Compensates landing site vector for lunar rotation during descent.
; Computes slant range to landing site for crew display.
;
; Equations computed:
;   TTF/8 = TTF/8 + (TPIP - TPIPOLD)/8
;   
;   Landing site rotated for lunar motion:
;   LAND = |LAND| * UNIT(LAND - LAND(TPIP - TPIPOLD) * WM)
;   where WM is lunar rotation rate vector
;   
;   Slant range for display:
;   RANGEDSP = |LAND - R|
;
; Lunar Rotation Correction: Moon rotates ~0.5 degrees/hour. During 12-minute
; descent from 50,000 feet, landing site moves ~2.5 km eastward relative to
; inertial space. This correction keeps target aligned with rotating surface.
; ============================================================================

#			TTF/8 = TTF/8 + (TPIP - TPIPOLD)/8
#		LANDING SITE VECTOR UPDATED FOR LUNAR ROTATION:
#			____               ____   ____                   __
#			LAND = /LAND/ UNIT(LAND - LAND(TPIP - TPIPOLD) * WM)
#		SLANT RANGE TO LANDING SITE, FOR DISPLAY:
#			                 ____   _
#			RANGEDSP = ABVAL(LAND - R)

TTFINCR		TC	INTPRET		; Enter interpreter for vector math
		DLOAD	DSU		; Load double precision, subtract
			TPIP		; Current PIPA time
			TPIPOLD		; Previous PIPA time
		SLR	PUSH		# SHIFT SCALES DELTA TIME TO 2(17) CSECS
			11D		; Shift right 11, scale to 2^17 centiseconds
		VXSC	VXV		; Vector scale by delta-t, cross product
			LAND		; Landing site position vector
			WM		; Lunar rotation rate vector (rad/sec)
		BVSU	RTB		; Back vector subtract, round to buffer
			LAND		; From original landing site vector
			NORMUNIT	; Normalize to unit vector
		VXSC	VSL1		; Vector scale, shift left 1
			/LAND/		; Landing site magnitude (radius)
		STODL	LANDTEMP	; Store rotated landing site, load delta-t
		EXIT			; Exit interpreter mode

; Update Time-to-Go with Elapsed Time
; TTF/8TMP contains previous TTF/8. Add delta-t/8 to account for time
; consumed during this guidance pass. Result is updated TTF/8.

		DXCH	MPAC		; Delta-t/8 is in MPAC from STODL
		DAS	TTF/8TMP	# NOW HAVE INCREMENTED TTF/8 IN TTF/8TMP

		TC	FASTCHNG	; Fast phase change for time update

		EXTEND
		DCA	TTF/8TMP	; Load updated time-to-go / 8
		DXCH	TTF/8		; Store as current TTF/8

; Copy Rotated Landing Site to Primary Location
; Three double-precision words (6 components) define landing site vector
; in inertial coordinates. Transfer from temporary to primary storage.

		EXTEND
		DCA	LANDTEMP	; X, Y components
		DXCH	LAND
		EXTEND
		DCA	LANDTEMP +2	; Z component and padding
		DXCH	LAND     +2
		EXTEND
		DCA	LANDTEMP +4	; Additional vector data
		DXCH	LAND	 +4

# Page 804
; Compute Slant Range Display
; TDISPSET calculates |LAND - R| for crew display showing distance to
; landing site. Updated every guidance pass for real-time situational
; awareness during descent.

		TC	TDISPSET	; Calculate and display slant range
		TC	FASTCHNG	# SINCE REDESIG MAY CHANGE LANDTEMP

; Branch to Pre-Guidance Computations
; Different guidance phases require different preliminary calculations
; before main guidance equations. Index by WCHPHASE to select routine.

BRSPOT2		INDEX	WCHPHASE	; Use phase index
		TCF	PREGUIDE	; Branch to phase-specific pre-guidance

# *********************************************************************
# LANDING SITE PERTURBATION EQUATIONS
# *********************************************************************

; ============================================================================
; LANDING SITE REDESIGNATION LOGIC
;
; This is the code that enabled Neil Armstrong to manually adjust the landing
; target when he saw that the automatic guidance was taking Eagle toward a
; boulder-strewn crater. By using the hand controller, Armstrong could shift
; the landing site designation in azimuth and elevation.
;
; During Apollo 11's actual descent on July 20, 1969, Armstrong used this
; capability extensively during the approach phase when he realized the
; original target was unsuitable. This manual override capability was crucial
; to the mission's success, though it contributed to the critically low fuel
; state at touchdown.
;
; The redesignation is accomplished by computing a new target vector LANDTEMP
; from the current landing site vector LAND, modified by crew inputs ELINCR
; (elevation increment) and AZINCR (azimuth increment) applied in the local
; vertical reference frame.
; ============================================================================

REDESIG		CA	FLAGWRD6	# IS REDFLAG SET?
		MASK	REDFLBIT
		EXTEND
		BZF	RGVGCALC	# NO:  SKIP REDESIGNATION LOGIC

; Check if redesignation timer TREDES has expired.
		CA	TREDES		# YES:  HAS TREDES REACHED ZERO?
		EXTEND
		BZF	RGVGCALC	# YES:  SKIP REDESIGNATION LOGIC

; Redesignation is active. Transfer crew-commanded increments from holding
; registers to working registers. ELINCR1/AZINCR1 accumulate hand controller
; inputs during guidance passes; they're transferred here atomically.
		INHINT
		CA	ELINCR1
		TS	ELINCR
		CA	AZINCR1
		TS	AZINCR
		TC	FASTCHNG

; Clear the accumulator registers for the next guidance cycle.
		CA	ZERO
		TS	ELINCR1
		TS	AZINCR1
		TS	ELINCR	+1
		TS	AZINCR  +1

; Initialize interpreter pushdown list for vector computation.
		CA	FIXLOC		# SET PD TO 0
		TS	PUSHLOC

; Enter interpretive mode to perform vector mathematics for redesignation.
; The algorithm constructs a local coordinate system at the LM's current
; position, then applies the crew's azimuth and elevation corrections to
; compute a new landing site vector LANDTEMP.
		TC	INTPRET
		VLOAD	VSU
			LAND		; Load landing site vector (LAND)
			R		; Subtract current position (R)
		RTB	PUSH		#                 ____   _
			NORMUNIT	# PUSH DOWN UNIT (LAND - R)
					; This gives the direction from LM to target
		VXV	VSL1
			YNBPIP		; Cross product with Y-axis of navigation
					; base (YNBPIP) creates an azimuth reference
					;                    ___        ____   _
		VXSC	PDDL		# PUSH DOWN - ELINCR(YNB * UNIT(LAND - R))
			ELINCR		; Scale by elevation increment from hand controller
			AZINCR		; Load azimuth increment
		VXSC	VSU		; Scale YNBPIP by azimuth increment
			YNBPIP		; and subtract to form azimuth correction vector
		VAD	PUSH		; Add both corrections together
					; RESULTING VECTOR IS 1/2 REAL SIZE

# Page 805

; Safety check: Prevent redesignation from moving the target too close to
; the lunar horizon, which would create an unsafe shallow approach angle.
; DEPRCRIT is the minimum allowable depression angle.
		DLOAD	DSU		# MAKE SURE REDESIGNATION IS NOT
			0		# 	TOO CLOSE TO THE HORIZON.
			DEPRCRIT	; Critical depression angle limit
		BMN	DLOAD		; If result is negative (too shallow)
			REDES1		; continue to REDES1
			DEPRCRIT	; else load critical angle
		STORE	0		; and limit the redesignation

; Compute the final redesignated landing site vector LANDTEMP.
; This applies the crew's corrections while maintaining proper scaling
; and ensuring the target remains on the lunar surface at radius /LAND/.
REDES1		DLOAD	DSU		; Load LAND vector magnitude
			LAND
			R		; Subtract current position vector magnitude
		DDV	VXSC		; Divide by depression angle constraint
			0		; Scale the correction vector by this ratio
		VAD	UNIT		; Add to current position vector R
			R		; and normalize to unit length
		VXSC	VSL1		; Scale to lunar surface radius /LAND/
			/LAND/		; (left shift 1 compensates for earlier half-size)
		STORE	LANDTEMP	; Store as temporary redesignated landing site
		EXIT			# LOOKANGL WILL BE COMPUTED AT RGVGCALC

; Transfer the redesignated landing site from LANDTEMP to LAND.
; This is a six-word double-precision vector transfer (3 components).
; FASTCHNG marks this as a high-priority restart-protected operation.
		TC	FASTCHNG

		EXTEND			; Transfer X component
		DCA	LANDTEMP
		DXCH	LAND
		EXTEND			; Transfer Y component
		DCA	LANDTEMP +2
		DXCH	LAND +2
		EXTEND			; Transfer Z component
		DCA	LANDTEMP +4
		DXCH	LAND +4

; Landing site redesignation complete. Continue to state computation.
		TCF	RGVGCALC

; ============================================================================
; TRANSITION: From Landing Site Redesignation to State Computation
;
; The landing site vector LAND now reflects any manual adjustments made by
; the crew. The guidance equations operate in a specialized coordinate system
; called "guidance coordinates" where the landing site is the origin and axes
; are aligned for optimal descent trajectory computations.
;
; RGVGCALC transforms the spacecraft state (position R, velocity V) from the
; inertial reference frame into guidance coordinates, accounting for lunar
; rotation. This transformation is fundamental to all subsequent guidance
; computations during the powered descent.
; ============================================================================

# *********************************************************************
# COMPUTE STATE IN GUIDANCE COORDINATES
# *********************************************************************
#
#	RGVGCALC COMPUTATIONS ARE AS FOLLOWS:--
#	VELOCITY RELATIVE TO THE SURFACE:
#		_______   _   _   __
#		ANGTERM = V + R * WM
#	STATE IN GUIDANCE COORDINATES:
#		___   *   _   ____
#		RGU = CG (R - LAND)
#		___   *   _   __   _
#		VGU = CG (V - WM * R)
# Page 806 actually starts one line earlier but that would separate the markers from their variables
#
#	HORIZONTAL VELOCITY FOR DISPLAY
#
#		VHORIZ = 8 ABVAL (0, VG , VG )
#		                       2    1
# 	DEPRESSION ANGLE FOR DISPLAY:
#		                       _   ____  ______
#		LOOKANGL = ARCSIN(UNIT(R - LAND).XMBPIP)

; Entry point during IGNALG phase (pre-ignition) to initialize velocity vector.
; This combines the integrated velocity from the IMU (VATT1) with a trim
; correction term (UNFC/2) to account for small velocity errors before descent
; engine ignition. The velocity is rotated from platform coordinates to the
; reference coordinate system using REFSMMAT.
CALCRGVG	TC	INTPRET		# IN IGNALG, COMPUTE V FROM INTEGRATION
		VLOAD	MXV		#	OUTPUT AND TRIM CORRECTION TERM
			VATT1		# Load velocity from IMU integration output
			REFSMMAT	; Rotate to reference coordinate system
		VSR1	VAD		; Right shift 1 (divide by 2) for scaling
			UNFC/2		; Add trim velocity correction
		STORE	V		; Store as current velocity vector
		EXIT

; Main entry point for state transformation into guidance coordinates.
; This routine is called every guidance cycle (typically every 2 seconds)
; to update the spacecraft state relative to the rotating lunar surface.
;
; First, compute velocity relative to the lunar surface by accounting for
; the Moon's rotation. ANGTERM = V + (R × WM) where WM is lunar angular
; velocity vector. This gives inertial velocity minus the velocity due to
; the Moon's rotation at position R.
RGVGCALC	TC	INTPRET		# ENTER HERE TO RECOMPUTE RG AND VG
		VLOAD	VXV		; Load position vector R
			R		; Cross product with lunar angular velocity
			WM		; (R × WM) gives surface velocity at position R
		VAD	VSR2		# Add to inertial velocity V
			V		; RESCALE TO UNITS OF 2(9) M/CS
		STORE	ANGTERM		; Store as surface-relative velocity
; Transform surface-relative velocity into guidance coordinates by
; multiplying by the guidance coordinate transformation matrix CG.
; VGU = CG * ANGTERM
		MXV
			CG		# NO SHIFT SINCE ANGTERM IS DOUBLE SIZED
		STORE	VGU		; Store velocity in guidance coordinates

; Compute horizontal velocity magnitude for crew display during P65 vertical
; descent phase. This is the magnitude of the velocity components in the
; guidance coordinate horizontal plane (VG2, VG1), ignoring vertical component.
		PDDL	VDEF		# FORM (0,VG ,VG ) IN UNITS OF 2(10) M/CS
			ZEROVECS	# Load zero for vertical component
				#           2   1
		ABVAL	SL3		; Absolute value (magnitude) of horizontal vector
		STOVL	VHORIZ		# Store horizontal velocity for display
				# VHORIZ FOR DISPLAY DURING P65.
; Compute position in guidance coordinates: RGU = CG * (R - LAND)
; This transforms position relative to the landing site into the guidance
; coordinate system where vertical is along local gravity and horizontal
; components are aligned with the descent trajectory plane.
			R		# Load current position vector
		VSU	PUSH		# Subtract landing site: (R - LAND)
			LAND		# PUSH DOWN R - LAND for later use
		MXV	VSL1		; Transform to guidance coordinates
			CG		; Left shift 1 for proper scaling
		STORE	RGU		; Store position in guidance coordinates
		ABVAL			; Compute slant range magnitude
		STOVL	RANGEDSP	; Store range for display during P64

; Compute depression angle (look angle) for crew visibility assessment.
; This is the angle between the spacecraft-to-landing-site line-of-sight
; vector and the local horizontal plane. During Apollo 11, Armstrong used
; this information to visually acquire the landing site during approach.
		RTB	DOT		# Normalize (R - LAND) unit vector
			NORMUNIT	; Convert to unit vector
			XNBPIP		; Dot with body-mounted X-axis (points down)
				# NOW IN MPAC IS SINE(LOOKANGL)/4
		EXIT			; Return to basic AGC code

; Reset interpreter stack pointer after vector operations.
		CA	FIXLOC		# RESET PUSH DOWN POINTER
		TS	PUSHLOC

# Page 807
; Convert the sine value to actual depression angle in degrees.
; The arcsine function returns the angle whose sine is the dot product
; computed above. This angle tells the crew how far below horizontal
; they must look to see the landing site.
		CA	MPAC		# Load sine(LOOKANGL)/4 from interpreter
		DOUBLE			; Scale adjustment for arcsine input
		TC	BANKCALL	; Call arcsine subroutine
		CADR	SPARCSIN -1	; Single precision arcsine
		AD	1/2DEG		; Add 0.5 degrees for rounding
		EXTEND
		MP	180DEGS		; Convert from scaled units to degrees
		TS	LOOKANGL	# Store look angle for P64 display
				# LOOKANGL FOR DISPLAY DURING P64

; Branch to appropriate guidance equation based on current descent phase.
; WCHPHASE index selects: IGNALG(-1), BRAKQUAD(0), APPRQUAD(1), VERTICAL(2)
BRSPOT3		INDEX	WCHPHASE
		TCF	WHATGUID	; Jump to phase-specific guidance routine

; ============================================================================
; TRANSITION: From State Computation to Time-To-Go Calculation
;
; The spacecraft state is now expressed in guidance coordinates (RGU, VGU)
; relative to the rotating lunar surface. The next critical computation is
; Time-To-Go (TTF), which predicts how many seconds remain until landing.
;
; TTF drives the entire guidance strategy: it determines when to transition
; between descent phases, shapes the optimal fuel consumption profile, and
; provides the crew with countdown information. The guidance equations solve
; for TTF by finding the root of a polynomial that relates current state to
; desired landing conditions (zero velocity at zero altitude).
;
; This computation executes every guidance cycle, continuously updating the
; predicted landing time as the spacecraft descends.
; ============================================================================

# **************************************************************************
# TTF/8 COMPUTATION
# **************************************************************************

; Entry point for Time-To-Go calculation. TTF is divided by 8 for numerical
; scaling reasons - the actual time-to-go is TTF = 8 * (TTF/8).
; This routine builds polynomial coefficients and calls the root finder.
TTF/8CL		TC	INTPRETX	; Enter interpreter (no variable setup)
; Build polynomial coefficients A(0) through A(3) for the TTF equation.
; The polynomial relates vertical position and velocity to time-to-go:
; A(3)*t³ + A(2)*t² + A(1)*t + A(0) = 0, where t = TTF/8
; Coefficients include current state (RGU, VGU) and desired state (RDG, VDG)
; from target tables indexed by TARGTDEX.
		DLOAD*			; Load indexed jerk target parameter
			JDG2TTF,1	; Jerk (rate of change of acceleration)
		STODL*	TABLTTF +6	# A(3) = 8 JDG₂ TO TABLTTF (cubic term)
			ADG2TTF,1	; Load indexed acceleration target parameter
		STODL	TABLTTF +4	# A(2) = 6 ADG₂ TO TABLTTF (quadratic term)
			VGU 	+4	; Load VGU₂ (vertical velocity component)
		DMP	DAD*		; Multiply by 3/4
			3/4DP		; Scaling factor
			VDG2TTF,1	; Add indexed velocity target parameter
		STODL*	TABLTTF +2	# A(1) = (6*VGU₂ + 18*VDG₂)/8 (linear term)
			RDG +4,1	; Load indexed position target parameter
		DSU	DMP		; Subtract current vertical position
			RGU +4		; RGU₂ (vertical position component)
			3/8DP		; Scale by 3/8
		STORE	TABLTTF		# A(0) = -24*(RGU₂ - RDG₂)/64 (constant term)
		EXIT			; Return to basic code for root finding

; Set up root finding precision and call polynomial root solver.
; The ROOTPSRS routine iteratively solves for the root of the polynomial
; using Newton-Raphson method. Initial guess is previous TTF/8 value.
		CA	BIT8		; Load precision specification
		TS	TABLTTF +10	# FRACTIONAL PRECISION FOR TTF TO TABLE
					; BIT8 = 0.0078125 fractional precision

		EXTEND			; Prepare for double precision operation
		DCA	TTF/8		; Load previous TTF/8 as initial guess
		DXCH	MPAC		# Transfer initial guess to MPAC for ROOTPSRS
		CAF	TWO		# Polynomial degree minus one (cubic = 3-1)
		TS	L		; Store in L register
		CAF	TABLTTFL	; Load address of coefficient table
		TC	ROOTPSRS	# Call root finder - returns TTF/8 in MPAC
					# YIELDS TTF/8 IN MPAC
; If root finding fails (no convergence), branch to alarm routine.
; Otherwise continue to store computed TTF/8 value.
		INDEX	WCHPHASE	; Index by current descent phase
		TCF	WHATALM		; Jump to phase-specific alarm handler

; Root finder converged successfully. Store the computed TTF/8 value.
		EXTEND			# GOOD RETURN (no alarm triggered)
		DCA	MPAC		# Fetch TTF/8 from MPAC (double precision)
		DXCH	TTF/8		# Store corrected TTF/8 for guidance use
					# This value predicts seconds-to-landing / 8

# Page 808
; Update crew display with new time-to-go information.
; TDISPSET converts TTF/8 to actual time and formats for DSKY display,
; giving Armstrong and Aldrin continuous countdown information.
		TC	TDISPSET

# 		(CONTINUE TO QUADGUID)

; ============================================================================
; TRANSITION: From Time-To-Go to Commanded Acceleration
;
; With TTF (time-to-go) computed, the guidance system now calculates the
; commanded acceleration vector (ACG) needed to reach the landing target.
; This is the heart of the powered descent guidance law.
;
; The guidance equation computes a three-dimensional acceleration command that
; will null out position and velocity errors by the predicted landing time.
; It's a proportional-derivative (PD) controller where:
;   - Position error drives acceleration quadratically (1/TTF²)
;   - Velocity error drives acceleration linearly (1/TTF)
;   - Desired acceleration (ADG) provides feedforward compensation
;
; During Apollo 11's descent, this equation executed every 2 seconds, updating
; the thrust commands that kept Eagle on course toward Tranquility Base.
; ============================================================================

# *********************************************************************************
# MAIN GUIDANCE EQUATION
# *********************************************************************************
#
#	AS PUBLISHED --
#		              ___   __       ___   __
#		___   ___   6(VDG + VG)   12(RDG - RG)
#		ACG = ADG + ----------- + ------------
#		                TTF        (TTF)(TTF)
#
#	AS HERE PROGRAMMED --
#		             ___   __
#		      3 (1/4(RDG - RG)   ___   __)
#		      - (------------- + VDG + VG)
#		___   4 (    TTF/8               )   ___
#		ACG = ---------------------------- + ADG
#		                  TTF/8
#
# The programmed form uses TTF/8 instead of TTF for better numerical scaling
# within AGC's 15-bit signed word length. Position (RG, RDG) and velocity
# (VG, VDG) are vectors in guidance coordinates (radial, downrange, crossrange).

; Guidance equation entry point. Computes coefficients accounting for
; guidance-to-engine response lag (LEADTIME). The engine doesn't respond
; instantaneously to thrust commands, so guidance compensates by commanding
; slightly ahead. This "lead compensation" improves closed-loop stability.
QUADGUID	CS	TTF/8		; Negate TTF/8
		AD	LEADTIME	# LEADTIME IS A NEGATIVE NUMBER (typically -0.5 sec)
		AD	POSMAX		# Add POSMAX (largest positive number) to safeguard
		TS	L		# Store in L, forcing -TTF*LEADTIME >= 0
					# This prevents division overflow in next steps
		CS	L		; Negate to get TTF/8 - LEADTIME
		AD	L		; Add back: forms 2*(TTF/8 - LEADTIME)
		ZL			; Zero L register for double precision divide
		EXTEND
		DV	TTF/8		; Divide by TTF/8 to get ratio
		TS	BUF		# Store -RATIO (negative of lag-diminished TTF/TTF)
					# BUF = -(TTF - LEADTIME)/TTF
		EXTEND
		SQUARE			; Square the ratio
		TS	BUF +1		; BUF+1 = RATIO²
		AD	BUF		; Add RATIO: RATIO² + RATIO
		XCH	BUF +1		# Swap: BUF+1 now has RATIO² - RATIO
		AD	BUF +1		; Add: 2*(RATIO² - RATIO)
		TS	MPAC		# COEFFICIENT FOR VGU TERM (current velocity)
		AD	BUF +1		; Add RATIO² - RATIO again
		INDEX	FIXLOC
		TS	26D		# COEFFICIENT FOR RDG-RGU TERM (position error)
		AD	BUF +1		; Add RATIO² - RATIO again
		INDEX	FIXLOC
		TS	28D		# COEFFICIENT FOR VDG TERM (desired velocity)
		AD	BUF		; Add RATIO term
		AD	POSMAX		; Add POSMAX for scaling
# Page 809
		AD	BUF +1		; Continue building final coefficient
		AD	BUF +1
		INDEX	FIXLOC
		TS	30D		# COEFFICIENT FOR ADG TERM (desired acceleration)
; These coefficients will weight the guidance equation terms to compute ACG.

; Zero MODE register and enter interpreter to compute guidance acceleration.
		CAF	ZERO
		TS	MODE		; MODE controls guidance computation options

; Compute commanded acceleration vector ACG using the guidance equation:
; ACG = 3/4 * [ (coef26*(RDG-RGU)/TTF/8 + coef28*VDG + coef*VGU) / TTF/8 ] + coef30*ADG
; where coef26, coef28, coef30 are the coefficients computed above.
; This implements the "programmed form" of the guidance equation.
		TC	INTPRETX	; Enter interpreter mode
		VXSC	PDDL		; VGU * coef(MPAC), push result, load 28D
			VGU		; Current velocity in guidance coords
			28D		; Coefficient for VDG term
		VXSC*	PDVL*		; VDG * 28D, push result, load RDG
			VDG,1		; Desired velocity (indexed by TARGTDEX)
			RDG,1		; Desired position (indexed by TARGTDEX)
		VSU	V/SC		; RDG - RGU, then divide by TTF/8
			RGU		; Current position in guidance coords
			TTF/8		; Time-to-go divided by 8
		VSR2	VXSC		; Shift right 2 (divide by 4), multiply by 26D
			26D		; Coefficient for position error term
		VAD	VAD		; Add VDG term, then add VGU term from stack
		V/SC	VXSC		; Divide by TTF/8, multiply by 3/4
			TTF/8		; Time-to-go divided by 8
			3/4DP		; 0.75 scaling factor
		PDDL	VXSC*		; Push result, load 30D, multiply ADG by coef
			30D		; Coefficient for ADG term
			ADG,1		; Desired acceleration (indexed by TARGTDEX)
		VAD			; Add feedforward ADG term to complete ACG
; ACG (commanded acceleration in guidance coordinates) now in MPAC.

; ============================================================================
; TRANSITION: From Guidance Acceleration to Autopilot Commands
;
; The commanded acceleration ACG is computed in guidance coordinates (radial,
; downrange, crossrange relative to rotating lunar surface). To command the
; spacecraft autopilot, this must be transformed to stable member (IMU)
; coordinates where the Digital Autopilot (DAP) and throttle control operate.
;
; The transformation matrix CG (guidance-to-stable) performs this rotation.
; The result is the Autopilot Flight Control Command (AFC), which contains:
;   - Thrust magnitude command (for throttle control)
;   - Thrust direction command (for attitude control via RCS or gimbal)
;
; During Apollo 11, this AFC computation executed every 2 seconds, providing
; continuous steering commands that kept the descent engine pointed in the
; optimal direction while maintaining proper thrust level.
; ============================================================================

; Transform guidance acceleration to stable member coordinates and compute
; thrust command accounting for gravity. VERGUID (vertical guidance) also
; enters here for common AFC computation.
AFCCALC1	VXM	VSL1		# VERGUID COMES HERE (alternate entry point)
			CG		; Guidance-to-stable transformation matrix
				; VXM: vector-matrix multiply (ACG * CG)
				; VSL1: shift left 1 bit (scale by 2)
		PDVL	V/SC		; Push transformed ACG, load gravity vector
			GDT/2		; Gravity acceleration scaled by 2
			GSCALE		; Gravity scaling factor for units
		BVSU	STADR		; Subtract gravity from commanded acceleration
				; (spacecraft must thrust against gravity)
		STORE	UNFC/2		# Store unnormalized flight control command
				# UNFC/2 in stable member coords (ft/sec²)
				# NEED NOT BE UNITIZED (magnitude preserved)
		ABVAL			; Compute absolute value (magnitude)

; The magnitude of UNFC/2 (AFC) provides total commanded acceleration which
; drives the throttle control routine. Store this critical value for engine.
AFCCALC2	STODL	/AFC/		# Store MAGNITUDE OF AFC FOR THROTTLE
					# /AFC/ = |UNFC/2| in pings/cs² scaled
			UNFC/2		# Load VERTICAL COMPONENT (altitude axis)
; Compute maximum available horizontal acceleration by checking the constraint:
;     AMAXHORIZ = SQRT(ATOTAL² - A₁² - A₀²)
; where ATOTAL = HIGHESTF/MASS (maximum engine thrust capability),
; A₁ = vertical acceleration, A₀ = out-of-plane acceleration.
; This computation prevents commanded acceleration from exceeding engine limits.
		DSQ	PDDL		; Square vertical, push to stack
			UNFC/2 +2	# Load OUT-OF-PLANE component
		DSQ	PDDL		; Square out-of-plane, push to stack
			HIGHESTF	; Maximum engine thrust force (Newtons)
		DDV	DSQ		; Divide by mass, square result
			MASS		#   ATOTAL² = (HIGHESTF/MASS)²
		DSU	DSU		# Subtract A₁² and A₀² from ATOTAL²
					# Result = ATOTAL² - A₁² - A₀²
		BPL	DLOAD		# If positive, take square root
			AFCCALC3	# Branch to SQRT computation
			ZEROVECS	; Else load zero (no horizontal capability)
; Maximum horizontal acceleration computed. Add to current horizontal
; component and limit if necessary to prevent over-commanding the engine.
AFCCALC3	SQRT	DAD		; SQRT(ATOTAL² - A₁² - A₀²) + horizontal accel
			UNFC/2 +4	# Add to downrange acceleration component
# Page 810
; Check if total horizontal acceleration exceeds available capability.
; If result is positive, limit is not exceeded; if negative, clamp to zero.
		BPL	BDSU		; Branch if positive (within limits)
			AFCCLEND	; Skip to end if within capability
			UNFC/2 +4	; Else subtract to compute excess
		STORE	UNFC/2 +4	# Limit horizontal to maximum available
					# (This clamps commanded acceleration to engine capability)
AFCCLEND	EXIT			; Return to basic code from interpreter
		TC	FASTCHNG	; Mark erasable memory as fast-changing

; Phase switching logic: prepare to transition between descent phases based
; on altitude, velocity, and time-to-go criteria. WCHPHASE index determines
; current guidance phase (BRAKQUAD, APPRQUAD, or VERTICAL).
		CA	WCHPHASE	# Load current phase index
		TS	WCHPHOLD	# PREPARE FOR PHASE SWITCHING LOGIC
		INCR	FLPASS0		# INCREMENT guidance cycle PASS COUNTER

; Jump to post-guidance computations for current phase. AFTRGUID table
; contains phase-specific routines: CGCALC for most phases, STEER? for vertical.
BRSPOT4		INDEX	WCHPHASE	; Index by current phase
		TCF	AFTRGUID	; Jump to phase-specific continuation

; ============================================================================
; TRANSITION: From Acceleration Command to Coordinate Transformation
;
; The guidance equations have computed the commanded acceleration vector AFC
; in guidance coordinates (rotating with lunar surface). To command the actual
; spacecraft attitude and engine throttle, AFC must be transformed to stable
; member (IMU) coordinates where the autopilot operates.
;
; CGCALC computes the transformation matrix CG (Coordinate transformation from
; Guidance to stable member) using time-varying parameters that account for
; lunar rotation during descent. This ensures commanded thrust vectors point
; in the correct inertial direction despite the rotating guidance frame.
; ============================================================================

# ***********************************************************************
# ERECT GUIDANCE-STABLE MEMBER TRANSFORMATION MATRIX
# ***********************************************************************

; Compute time-dependent transformation matrix elements. The transformation
; accounts for lunar rotation during descent, which causes guidance coordinates
; to rotate relative to inertial (stable member) coordinates.
; Transformation rate parameters TCGFBRAK and TCGIBRAK define rotation rate.
CGCALC		CAF	EBANK5		; Switch to EBANK 5
		TS	EBANK		; for access to transformation parameters
		EBANK=	TCGIBRAK	; Assembler notation for EBANK
		EXTEND			; Enable extended addressing
		INDEX	WCHPHASE	; Index by current descent phase
		INDEX	TARGTDEX	; Index by target parameter set
		DCA	TCGFBRAK	; Load TCGFBRAK (transformation rate parameter)
					# Double precision constant
		INCR	BBANK		; Increment bank twice to access
		INCR	BBANK		; next parameter bank
		EBANK=	TTF/8		; Switch back to TTF/8 bank
; Compute time-varying transformation parameters:
; Result = TCGFBRAK + TTF/8 (accounts for time remaining to landing)
		AD	TTF/8		; Add TTF/8 to low-order word
		XCH	L		; Exchange with L register
		AD	TTF/8		; Add TTF/8 to high-order word
; Check if transformation time parameter has reached the transition point
; for phase switching. Double-precision sign check via cascaded CCS instructions.
		CCS	A		; Check sign of high-order word
		CCS	L		; If positive, check low-order word
		TCF	EXTLOGIC	; Both positive: proceed to exit logic
		TCF	EXTLOGIC	; High positive, low any: proceed
		NOOP			; High zero: no-op (continue to interpreter)

; Enter interpreter to build the CG transformation matrix using vector operations.
; The matrix transforms commanded acceleration from guidance coordinates (rotating
; with lunar surface) to stable member (IMU) coordinates (inertial).
		TC	INTPRETX	; Enter interpretive mode
		VLOAD	UNIT		; Load landing site vector LAND
			LAND		; and normalize to unit vector
		STODL	CG		; Store as first row of CG matrix
			TTF/8		; Load TTF/8 (time-to-go / 8)
; Compute angular correction term that accounts for lunar rotation during descent.
; GAINBRAK is the "mysterious number" - a gain constant tuned empirically during
; Apollo guidance development to optimize landing accuracy.
		DMP*	VXSC		; Multiply TTF/8 by GAINBRAK (indexed by phase)
			GAINBRAK,1	# NUMERO MYSTERIOSO (gain constant)
			ANGTERM		; Scale by ANGTERM (angular rate vector)
		VAD			; Add result to landing site vector
			LAND		; LAND + (TTF/8 * GAINBRAK * ANGTERM)
		VSU	RTB		; Subtract current position vector R
			R		; Result: desired position offset
			NORMUNIT	; Normalize to unit vector
# Page 811
; Complete the transformation matrix CG using vector cross products to form
; an orthonormal basis. CG transforms from guidance to stable member coordinates.
		VXV	RTB		; Cross product: (result) × LAND
			LAND		; Creates vector perpendicular to both
			NORMUNIT	; Normalize to unit vector
		STOVL	CG +6		# Store as SECOND ROW of CG matrix
			CG		; Reload first row of CG matrix
; Compute third row of CG matrix to complete orthonormal triad.
; Third row = first row × second row (right-handed coordinate system).
		VXV	VSL1		; Cross product CG × (CG+6), shift left 1
			CG +6		; Complete orthogonal triad
		STORE	CG +14		# Store as THIRD ROW of CG matrix
					# CG matrix is now complete 3×3 transformation
		EXIT			; Return to basic AGC code

; ============================================================================
; TRANSITION: From CG Matrix Computation to Exit Decision Logic
;
; The transformation matrix CG is complete. Now determine:
;   (1) Whether to switch to the next descent phase (BRAKQUAD → APPRQUAD → VERTICAL)
;   (2) Which exit routine to use for window vector computation and display updates
;
; Phase switching occurs when TTF/8 reaches the phase-specific threshold TENDBRAK.
; This implements the descent profile: high-altitude braking, approach with visibility
; for terrain assessment, and final vertical descent for touchdown.
; ============================================================================

#		(CONTINUE TO EXTLOGIC)
#
# ***********************************************************************
# PREPARE TO EXIT
# ***********************************************************************
#
# DECIDE (1) HOW TO EXIT, AND (2) WHETHER TO SWITCH PHASES
#
; Exit logic determines phase transition timing. Each descent phase has a target
; end time TENDBRAK. When TTF/8 (time-to-go) reaches TENDBRAK, switch to next phase.
; WCHPHASE indexing: 0 = BRAKQUAD (braking), 1 = APPRQUAD (approach), 2 = VERTICAL
EXTLOGIC	INDEX	WCHPHASE	# Index to phase-specific end time
		CA	TENDBRAK	# Load TENDBRAK for current phase
					# WCHPHASE = 1   APPRQUAD
					# WCHPHASE = 0   BRAKQUAD
		AD	TTF/8		; Add TTF/8 to check if phase end reached

; Check if it's time to switch to the next phase. BZMF branches if result ≤ 0,
; meaning TTF/8 has reached or passed the phase end threshold.
EXSPOT1		EXTEND			; Enable extended instruction
		INDEX	WCHPHASE	; Index by current phase
		BZMF	WHATEXIT	; Branch if phase complete (TTF/8 ≤ TENDBRAK)
					; Stay in current phase

; Phase transition point reached. Advance to next phase and reset pass counter.
		TC	FASTCHNG	; Mark erasables as fast-changing

		CA	WCHPHOLD	; Load saved phase index
		AD	ONE		; Increment to next phase
		TS	WCHPHASE	; Store new phase: BRAKQUAD(0) → APPRQUAD(1) → VERTICAL(2)
		CA	ZERO		; Zero accumulator
		TS	FLPASS0		# RESET guidance cycle PASS COUNTER for new phase

; Exit via phase-specific routine. WHATEXIT table contains exit addresses
; for each phase (EXGSUB, EXBRAK, EXNORM).
		INDEX	WCHPHOLD	; Index by previous phase
		TCF	WHATEXIT	; Transfer to phase-specific exit routine

; ============================================================================
; EXIT ROUTINES FROM LANDING GUIDANCE
; ============================================================================
; Three exit paths handle different descent phases:
;
; 1. EXGSUB - Exit from ignition algorithm (IGNALG phase)
;    Computes trim velocity correction for initial engine startup
;
; 2. EXBRAK - Exit during braking phase
;    Uses UNIT(R) as the window pointing vector for high-altitude braking
;
; 3. EXNORM - Exit during approach and later phases
;    Computes window vector pointing from current position toward landing site
;    Checks visibility constraints to keep landing site in crew's field of view
;
; EXOVFLOW handles guidance overflow errors non-abortively (alarm 01410)
; ============================================================================

# ***********************************************************************
# ROUTINES FOR EXITING FROM LANDING GUIDANCE
# ***********************************************************************
#
# 1.	EXGSUB IS THE RETURN WHEN GUIDSUB IS CALLED BY THE IGNITION ALGORITHM.
# 2.	EXBRAK IN THE EXIT USED DURING THE BRAKING PHASE.  IN THIS CASE UNIT(R) IS THE WINDOW POINTING VECTOR.
# 3.	EXNORM IS THE EXIT USED AT OTHER TIMES DURING THE BURN.
# (EXOVFLOW IS A SUBROUTINE OF EXBRAK AND EXNORM CALLED WHEN OVERFLOW OCCURRED ANYWHERE IN GUIDANCE.)

; EXGSUB: Exit for ignition algorithm phase (WCHPHASE = -1)
; Computes trim velocity correction term to compensate for engine startup transients.
; This ensures smooth transition from ignition to main guidance loop.
EXGSUB		TC	INTPRET		# COMPUTE TRIM VELOCITY CORRECTION TERM.
# Page 812
		VLOAD	RTB		; Load UNFC/2 (commanded thrust direction)
			UNFC/2		; and normalize to unit vector
			NORMUNIT
		VXSC	VXSC		; Scale by ZOOMTIME (ignition ramp time)
			ZOOMTIME	; and TRIMACCL (trim acceleration)
			TRIMACCL	; to compute velocity correction
		STORE	UNFC/2		; Store updated thrust direction command
		EXIT			; Return to basic AGC code

; Check if guidance subroutine needs another iteration
		CCS	NGUIDSUB	; Test guidance subroutine counter
		TCF	GUIDSUB		; If positive, return to GUIDSUB for another pass
		CCS	NIGNLOOP	; Test ignition loop counter
		TCF	+3		; If positive, continue to DDUMCALC
		TC	ALARM		; If zero, sound alarm
		OCT	01412		; Alarm 01412: Ignition loop count exhausted

 +3		TC	POSTJUMP	; Jump to next phase
 		CADR	DDUMCALC	; Continue with delta-altitude dummy calculation

; EXBRAK: Exit for braking phase (WCHPHASE = 0)
; During braking, the window vector points radially upward (UNIT(R)).
; This allows crew to observe the Moon's horizon and assess descent progress.
; At high altitude, the landing site is not yet visible.
EXBRAK		TC	INTPRET		; Enter interpretive mode
		VLOAD			; Load UNIT/R/ (unit position vector)
			UNIT/R/		; Points radially from lunar center
		STORE	UNWC/2		; Store as window vector UNWC/2 (half-unit scaled)
		EXIT			; Return to basic AGC code
		TCF	STEER?		; Continue to steering decision logic

; EXNORM: Exit for approach and vertical phases (WCHPHASE = 1, 2)
; Computes window vector pointing toward landing site, keeping it in crew's view.
; Blends two vectors to ensure landing site visibility while avoiding gimbal lock:
;   - UNIT(LAND - R): Points from spacecraft toward landing site
;   - CG +14: Down axis in guidance coordinates (local vertical)
; The blend coefficients depend on projection limits PROJMIN and PROJMAX.
EXNORM		TC	INTPRET		; Enter interpretive mode
		VLOAD	VSU		; Load LAND (target landing site)
			LAND		; Subtract R (current position)
			R		; Result: vector from spacecraft to landing site
		RTB			; Normalize to unit vector
			NORMUNIT
		STORE	UNWC/2		# Store UNIT(LAND - R) as tentative window vector
		VXV	DOT		; Cross product with XNBPIP (body X-axis)
			XNBPIP		; then dot with CG +6 (guidance Y-axis)
			CG +6		; Computes projection of window vector
		EXIT			# Result in MPAC, scaled 1/8 real size

; Check if projection is within visibility cone limits.
; If projection exceeds PROJMAX or is below PROJMIN, blend with CG +14 (down axis).
		CS	MPAC		# Load negative of projection
		AD	PROJMAX		; Add PROJMAX
		AD	POSMAX		; Add POSMAX (0x3FFF, positive max)
		TS	BUF		; Store result
		CS	BUF		; Complement
		ADS	BUF		# Double precision clear if negative
					# BUF = 0 if PROJMAX - PROJ < 0 (outside cone)

		CS	PROJMIN		# Load negative of PROJMIN
		AD	MPAC		; Add projection
		AD	POSMAX		; Add POSMAX
		TS	BUF +1		; Store result
		CS	BUF +1		; Complement
# Page 813
		ADS	BUF +1		# BUF +1 = 0 if PROJ - PROJMIN < 0 (below limit)

; Compute blended window vector:
; UNWC/2 = BUF × CG +14 + BUF +1 × UNIT(LAND - R)
; This forms weighted combination ensuring visibility constraints satisfied.
		CAF	FOUR		; Loop counter: 4, 2, 0 for X, Y, Z components
UNWCLOOP	MASK	SIX		; Mask to get component index
		TS	Q		; Save index in Q
		CA	EBANK5		; Switch to EBANK containing CG matrix
		TS	EBANK
		EBANK=	CG		; Set EBANK annotation
		CA	BUF		; Load CG +14 coefficient
		EXTEND
		INDEX	Q		; Index by component
		MP	CG +14		; Multiply by CG +14 component
		INCR	BBANK		; Switch back to UNWC/2 bank
		EBANK=	UNWC/2
		INDEX	Q		; Index by component
		DXCH	UNWC/2		; Store first term
		EXTEND
		MP	BUF +1		; Multiply by UNIT(LAND - R) coefficient
		INDEX	Q		; Index by component
		DAS	UNWC/2		; Add second term (double precision add)
		CCS	Q		; Decrement component index
		TCF	UNWCLOOP	; Loop for next component (Y, then Z)

		INCR	BBANK		; Restore bank register
		EBANK=	PIF

; ============================================================================
; STEERING DECISION AND AUTOPILOT OUTPUT
; ============================================================================
; After guidance computation, decide whether to send commands to autopilot.
; Check steering enable flag and overflow conditions before calling:
;   - THROTTLE: Update descent engine throttle setting
;   - FINDCDUW: Compute commanded attitude (CDUX, CDUY, CDUZ gimbal angles)
;   - STOPRATE: Zero spacecraft rotation rates if in attitude-hold
; ============================================================================

; STEER?: Check if steering is enabled (STEERSW flag)
; If steering disabled (flag down), skip throttle/attitude commands and go to RATESTOP
STEER?		CA	FLAGWRD2	# Load flag word 2
		MASK	STEERBIT	# Test STEERSW bit
		EXTEND
		BZF	RATESTOP	; If steering off, skip guidance outputs

; EXVERT: Check for guidance overflow condition
; If overflow detected anywhere in guidance computations (OVFIND set),
; sound non-abortive alarm and skip throttle/attitude commands.
EXVERT		CA	OVFIND		# Load overflow indicator
		EXTEND			# If OVFIND != 0, overflow occurred
		BZF	+13		; If no overflow, skip to GDUMP1

; EXOVFLOW: Handle guidance overflow by sounding alarm
; Alarm 01410 is non-abortive: mission can continue but crew is alerted.
; This protects against erroneous commands from corrupted guidance data.
EXOVFLOW	TC	ALARM		# Sound non-abortive alarm
		OCT	01410		; Alarm code 01410: Guidance overflow

; RATESTOP: Check if spacecraft is in attitude-hold mode
; If in attitude-hold (BIT13 of CHAN31 = 0), skip to displays.
; Otherwise, command autopilot to stop rotation rates.
RATESTOP	CAF	BIT13		# Load attitude-hold test bit
		EXTEND
		RAND	CHAN31		# Read channel 31 (mode status)
		EXTEND
		BZF	DISPEXIT	# If in attitude-hold, go to displays

		TC	BANKCALL	# Not in attitude-hold: stop rates
		CADR	STOPRATE	; Call STOPRATE to zero rotation rates

		TCF	DISPEXIT	; Continue to displays

; GDUMP1: Normal exit with steering commands
; Update throttle setting and compute commanded attitude for autopilot.
GDUMP1		TC	THROTTLE	; Call THROTTLE to update engine setting
# Page 814
		TC	INTPRET		; Enter interpretive mode
		CALL			; Call attitude computation
			FINDCDUW -2	; FINDCDUW computes CDUX/Y/Z gimbal commands
		EXIT			; Return to basic AGC code

# 		(CONTINUE TO DISPEXIT)

# ***********************************************************************
# GUIDANCE LOOP DISPLAYS
# ***********************************************************************

; ============================================================================
; DISPEXIT: Guidance Loop Display Management
; ============================================================================
; Complete guidance cycle by updating DSKY displays showing descent status.
; Phase-specific displays:
;   P63 (BRAKQUAD): V06N63 displays altitude, altitude rate, forward velocity
;   P64 (APPRQUAD): V06N64 displays LPD angle, altitude, altitude rate (with
;                   redesignation logic allowing crew to adjust landing site)
;   P65/66/67 (VERTICAL): V06N60 displays altitude, altitude rate, fuel
; ============================================================================

; DISPEXIT: Exit point for guidance loop with display updates
; Kill GROUP 3 restart protection for displays (will be restored next cycle).
DISPEXIT	EXTEND			# Enable extended instruction
		DCA	NEG0		# Load negative zero (phase table kill code)
		DXCH	-PHASE3		# Store to kill GROUP 3 restart protection

; Check FLUNDISP flag: if set, skip display this pass (e.g., during phase transitions)
 +3		CS	FLAGWRD8	# Load complement of flag word 8
 		MASK	FLUNDBIT	# Test FLUNDISP bit
		EXTEND
		BZF	ENDLLJOB	# If FLUNDISP set, skip display and end job

		INDEX	WCHPHOLD	# Index by current phase (saved in WCHPHOLD)
		TCF	WHATDISP	; Transfer to phase-specific display routine

; Return point from FINDCDUW subroutine (called at GDUMP1 -2)
-2		TC	PHASCHNG	# Kill GROUP 5 restart protection
		OCT	00035		; Phase change code

; P63DISPS: Braking phase display (WCHPHASE = 0)
; V06N63: Verb 06 (display decimal), Noun 63 (altitude, altitude rate, velocity)
P63DISPS	CAF	V06N63		; Load display code V06N63
DISPCOMN	TC	BANKCALL	; Call display routine
		CADR	REGODSPR	; REGODSPR (registers for display)

ENDLLJOB	TCF	ENDOFJOB	; End guidance job, return to executive

; P64DISPS: Approach phase display with redesignation logic (WCHPHASE = 1)
; Allows crew to adjust landing site using ROD (rate-of-descent) redesignation.
P64DISPS	CA	TREDES		# Load redesignation time remaining counter
		EXTEND
		BZF	RED-OVER	# If TREDES = 0, redesignation time expired

		CS	FLAGWRD6	# Load complement of flag word 6
		MASK	REDFLBIT	# Test REDFLAG (redesignation enabled)
		EXTEND
		BZF	REDES-OK	# If REDFLAG set, do static display

		CAF	V06N64		# REDFLAG not set: use flashing display
		TC	BANKCALL	; to prompt crew for redesignation
		CADR	REFLASHR	; REFLASHR (flashing display routine)
		TCF	GOTOPOOH	# TERMINATE: Abort to P00 (Program 00)
		TCF	P64CEED		# PROCEED: Enable redesignation
		TCF	P64DISPS	# RECYCLE: Redisplay
# Page 815
		TCF	ENDLLJOB	; End job

; P64CEED: Crew proceeded with redesignation enable
; Zero increments and set REDFLAG to allow landing site adjustments.
P64CEED		CAF	ZERO		; Zero accumulator
		TS	ELINCR1		; Clear elevation increment
		TS	AZINCR1		; Clear azimuth increment

		TC	UPFLAG		# Set REDFLAG
		ADRES	REDFLAG		; Enable redesignation logic

		TCF	ENDOFJOB	; End job

; RED-OVER: Redesignation time expired, clear REDFLAG and do static display
RED-OVER	TC	DOWNFLAG	; Clear REDFLAG
		ADRES	REDFLAG		; Redesignation window closed
REDES-OK	CAF	V06N64		; Load display code V06N64
		TCF	DISPCOMN	; Do static (non-flashing) display

; VERTDISP: Vertical descent phase display (WCHPHASE = 2)
; V06N60: Altitude, altitude rate, fuel remaining
VERTDISP	CAF	V06N60		; Load display code V06N60
		TCF	DISPCOMN	; Common display routine


; ============================================================================
; TRANSITION: From Approach Phase to Vertical Descent
;
; With the LM positioned above the landing site at low altitude, the guidance
; computer transitions to vertical descent mode. This is the final phase before
; touchdown, where the crew has maximum control over descent rate while the
; computer maintains horizontal position stability.
;
; Three vertical descent programs are available:
;   P65 - Automatic vertical descent (rarely used operationally)
;   P66 - Manual rate-of-descent control (most common for final approach)
;   P67 - Automatic landing from low altitude (not used on Apollo 11)
;
; During Apollo 11, Armstrong manually controlled descent rate in P66 mode
; while searching for a safe landing site among the boulders in the Sea
; of Tranquility. This phase consumed precious fuel, leaving only ~25 seconds
; of propellant remaining at touchdown.
; ============================================================================

# **************************************************************************
# GUIDANCE FOR P65
# **************************************************************************

; VERTGUID - Vertical Guidance Phase Router
; Selects appropriate vertical guidance algorithm based on WCHVERT flag:
;   WCHVERT > 0 ---> P67VERT (automatic landing)
;   WCHVERT = 0 ---> P66VERT (manual rate-of-descent)
;   WCHVERT < 0 ---> P65VERT (automatic vertical descent)

VERTGUID	CCS	WCHVERT		; Test vertical phase indicator
		TCF	P67VERT		; Positive non-zero ---> P67 automatic landing
		TCF	P66VERT		; +0 ---> P66 manual ROD control
#
# 	THE P65 GUIDANCE EQUATION IS AS FOLLOWS --
#		      ____   ___
#		      V2FG - VGU
#		ACG = ----------
#		        TAUVERT
;
; P65VERT - Automatic Vertical Descent Guidance
; 
; Computes commanded acceleration to null horizontal velocity while
; descending vertically. Rarely used operationally - crews preferred
; manual control (P66) for final approach.
;
; Guidance equation: ACG = (V2FG - VGU) / TAUVERT
;   where:
;     V2FG = Final velocity goal (near zero horizontal, slow vertical)
;     VGU = Current velocity estimate
;     TAUVERT = Vertical guidance time constant
;     ACG = Commanded acceleration vector
;
; This equation generates thrust commands to drive velocity error to zero
; over the time constant TAUVERT, providing smooth nulling of horizontal
; drift while maintaining controlled vertical descent rate.

P65VERT		TC	INTPRET		; Enter interpretive mode for vector math
		VLOAD	VSU		; Load V2FG vector, subtract VGU
			V2FG		; Final velocity goal vector
			VGU		; Current velocity estimate
		V/SC	GOTO		; Divide vector by scalar, branch
			TAUVERT		; Time constant for velocity nulling
			AFCCALC1	; Continue to acceleration command calc
# Page 816
# **********************************************************
# GUIDANCE FOR P66
# **********************************************************

; P66VERT - Manual Rate-of-Descent (ROD) Control Entry Point
;
; This is the mode Armstrong used during Apollo 11's final approach to the
; lunar surface. The crew controls descent rate with the thrust/translation
; controller (joystick), while the computer automatically maintains horizontal
; position above the landing site.
;
; P66 implements a two-component guidance law:
;   1. Automatic horizontal position-keeping to null lateral drift
;   2. Manual vertical rate control via crew joystick input
;
; During Apollo 11, as Armstrong searched for a boulder-free landing site,
; P66's horizontal position-keeping allowed him to focus solely on vertical
; rate control. This human-computer partnership was critical to achieving
; a safe touchdown with minimal fuel remaining.

P66VERT		TC	POSTJUMP	; Jump to P66VERTA (in different memory bank)
		CADR	P66VERTA	; Cross-bank address of P66 vertical guidance

; P67VERT - Automatic Landing From Low Altitude
;
; P67 provides fully automatic landing capability from low altitude (typically
; below 5000 feet). Not used operationally on Apollo 11 - Armstrong maintained
; manual control through touchdown. This mode was available as a backup if the
; crew needed to relinquish control to the computer for automatic landing.
;
; P67 computes velocity error and generates thrust commands to guide the LM
; to a soft touchdown at the pre-selected landing site. The guidance law drives
; both horizontal and vertical velocity to near-zero at ground contact.

P67VERT		TC	PHASCHNG	; Terminate restart group 3
		OCT	00003		; Group 3 phase change code

		TC	INTPRET		; Enter interpretive mode for vector math
		VLOAD	GOTO		; Load current velocity vector
			V		; Actual velocity from navigation state
			VHORCOMP	; Extract horizontal component for guidance

		SETLOC	P66LOC
		BANK
		COUNT*	$$/F2DPS

; ============================================================================
; RODTASK - RATE OF DESCENT TASK SCHEDULER
;
; Schedules RODCOMP to run at priority 22 for processing rate-of-descent
; commands from the crew during P66 manual descent mode.
; ============================================================================

RODTASK		CAF	PRIO22
		TC	FINDVAC
		EBANK=	DVCNTR
		2CADR	RODCOMP

		TCF	TASKOVER

; ============================================================================
; P66VERTA - P66 MANUAL DESCENT CONTROL (MAIN IMPLEMENTATION)
;
; This is the actual implementation of P66 manual descent, whereas P66VERT
; (line 1577) was merely a transfer point. P66 allows the crew to control
; rate of descent using the hand controller while the AGC maintains attitude
; and horizontal velocity nulling.
;
; MISSION CONTEXT: During Apollo 11, Armstrong used P64 semi-automatic mode
; for the approach phase. P66 was available if more direct manual control
; became necessary to avoid unsuitable terrain or respond to emergencies.
;
; CONTROL PHILOSOPHY:
; - Crew commands rate-of-descent (ROD) via hand controller (+Z/-Z axis)
; - AGC samples controller input and updates VDGVERT (desired vertical velocity)
; - AGC computes thrust magnitude to achieve commanded ROD
; - AGC automatically nulls horizontal velocity to maintain position
; - AGC maintains attitude for crew visibility and S-band antenna pointing
;
; The routine sets up periodic RODTASK execution (every 1 second) to process
; crew commands and compute thrust adjustments via RODCOMP.
; ============================================================================

P66VERTA	TC	PHASCHNG	# TERMINATE GROUP 3.
		OCT	00003

		CAF	1SEC		# Schedule RODTASK to execute every 1 second
		TC	TWIDDLE
		ADRES	RODTASK

; ============================================================================
; RODCOMP - RATE OF DESCENT COMPUTATION AND THRUST COMMAND GENERATION
;
; This routine is the heart of P66 manual descent control. It executes once
; per second (scheduled by RODTASK) to:
; 1. Read crew's rate-of-descent command from hand controller (via RODCOUNT)
; 2. Update desired vertical velocity VDGVERT based on crew command
; 3. Read current PIPA accelerometer data to measure actual acceleration
; 4. Compute velocity and position updates from accelerometer data
; 5. Compute thrust acceleration needed to achieve desired ROD
; 6. Compute horizontal velocity component and null it
; 7. Generate thrust magnitude command /AFC/ (thrust acceleration command)
; 8. Call THROTTLE routine to execute engine throttle command
;
; ACCELEROMETER DATA PROCESSING:
; The AGC has no direct velocity or altitude sensors during powered flight.
; Instead, it integrates PIPA (Pulsed Integrating Pendulous Accelerometer)
; readings to compute velocity changes. This routine:
; - Saves previous PIPA readings (OLDPIPAX, OLDPIPAY, OLDPIPAZ)
; - Reads current PIPA values (PIPAX, PIPAY, PIPAZ)
; - Computes delta-V from PIPA pulse counts scaled by KPIP1
; - Updates velocity vector V by integrating acceleration over time
; - Corrects for gravity acceleration GDT/2 and velocity bias VBIAS
;
; SCALING: PIPA pulses scaled by KPIP1 = 5.85 cm/sec per pulse
;          Velocities in meters/centisecond * 2^(-7)
;          Accelerations in meters/centisecond^2
; ============================================================================

RODCOMP		INHINT			# Disable interrupts during PIPA snapshot
		
		; Process crew's rate-of-descent command from hand controller.
		; RODCOUNT accumulates controller pulses; scaling converts to velocity.
		CAF	ZERO
		XCH	RODCOUNT	# Fetch and clear RODCOUNT atomically
		EXTEND
		MP	RODSCAL1	# Scale controller pulses to velocity units
		DAS	VDGVERT		# UPDATE DESIRED ALTITUDE RATE (add to current)
		
		; Snapshot current PIPA accelerometer readings for delta-V computation.
		; PIPA (Pulsed Integrating Pendulous Accelerometer) provides acceleration
		; measurements as pulse counts. We save previous readings (OLDPIPAx) and
		; current readings (PIPAx) to compute acceleration over the interval.
		
		EXTEND			# SET OLDPIPAX,Y,Z = PIPAX,Y,Z
		DCA	PIPAX		# Fetch PIPAX (high) and PIPAY (low) together
		DXCH	OLDPIPAX	# Store in OLDPIPAX, OLDPIPAY; fetch old values
		DXCH	RUPTREG1	# Store old values in RUPTREG1,2 for later use
		CA	PIPAZ		# Fetch PIPAZ (Z-axis accelerometer)
		XCH	OLDPIPAZ	# Store in OLDPIPAZ; fetch old value
		XCH	RUPTREG3	# Store old value in RUPTREG3
		
		; Snapshot mission elapsed time for this PIPA reading.
		; Time synchronization is critical for accurate velocity integration.
		EXTEND			# SNAPSHOT TIME OF PIPA READING.
		DCA	TIME2		# TIME2 = current mission time in centiseconds
# Page 817
		DXCH	THISTPIP	# THISTPIP = time of this PIPA reading
		
		; Compute total accumulated PIPA counts for velocity integration.
		; MPAC will hold total pulse counts in X, Y, Z axes.
		; PIPATMPX/Y/Z contain residual pulses from previous incomplete interval.
		
		CA	OLDPIPAX
		AD	PIPATMPX	# Add residual X-axis pulses
		TS	MPAC		# MPAC(X) = PIPAX + PIPATMPX
		CA	OLDPIPAY
		AD	PIPATMPY	# Add residual Y-axis pulses
		TS	MPAC +3		# MPAC(Y) = PIPAY + PIPATMPY
		CA	OLDPIPAZ
		AD	PIPATMPZ	# Add residual Z-axis pulses
		TS	MPAC +5		# MPAC(Z) = PIPAZ + PIPATMPZ
		
		; Compute delta-V from PIPA pulse count differences.
		; DELVROD = change in velocity from last reading to this reading.
		; This delta-V will be scaled and integrated into velocity vector V.
		
		CS	OLDPIPAX	# Negate OLDPIPAX (complement and add)
		AD	TEMX		# Add current TEMX (updated by READACCS interrupt)
		AD	RUPTREG1	# Add previous OLDPIPAX saved earlier
		TS	DELVROD		# DELVROD(X) = delta pulses in X axis
		CS	OLDPIPAY	# Negate OLDPIPAY
		AD	TEMY		# Add current TEMY
		AD	RUPTREG2	# Add previous OLDPIPAY
		TS	DELVROD +2	# DELVROD(Y) = delta pulses in Y axis
		CS	OLDPIPAZ	# Negate OLDPIPAZ
		AD	TEMZ		# Add current TEMZ
		AD	RUPTREG3	# Add previous OLDPIPAZ
		TS	DELVROD +4	# DELVROD(Z) = delta pulses in Z axis
		
		; Initialize low-order components and prepare for interpretive sequence.
		
		CAF	ZERO
		TS	MPAC +1		# ZERO LO-ORDER MPAC COMPONENTS (double precision)
		TS	MPAC +4
		TS	MPAC +6
		TS	TEMX		# ZERO TEMX, TEMY, AND TEMZ SO WE WILL
		TS	TEMY		#	KNOW WHEN READACCS CHANGES THEM.
		TS	TEMZ		# (READACCS is interrupt routine updating these)
		CS	ONE
		TS	MODE		# Set MODE = -1 to indicate guidance active
		TC	INTPRET		# Enter interpretive mode for vector computations
		
		; ========================================================================
		; VELOCITY INTEGRATION FROM PIPA ACCELEROMETER DATA
		;
		; This interpretive sequence integrates PIPA pulse counts to update the
		; velocity vector V. The computation accounts for:
		; 1. Scaling PIPA pulses to velocity units via KPIP1
		; 2. Computing time interval since last PIPA reading (THISTPIP - PIPTIME)
		; 3. Correcting for gravity acceleration GDT/2
		; 4. Removing velocity bias VBIAS from previous computation
		; 5. Integrating to form updated velocity vector
		;
		; INTERPRETIVE STACK MANAGEMENT:
		; Numbers in parentheses (6), (8), (0) indicate stack depth changes.
		; Vector operations push/pop data on the interpretive stack (MPAC).
		; ========================================================================
		
ITRPNT1		VXSC	PDDL		# SCALE MPAC TO M/CS *2(-7) AND PUSH 	(6)
			KPIP1		# KPIP1 = 5.85 cm/sec per PIPA pulse
			THISTPIP	# Load time of this PIPA reading
		DSU			# Subtract (double precision)
			PIPTIME		# Time of previous PIPA reading
		STORE	30D		# 30-31D CONTAINS TIME IN CS SINCE PIPTIME (delta-t)
		DDV	PDVL		# Divide delta-t by 4 seconds, push result	(8)
			4SEC(28)	# 4 seconds scaled by 2^28
			GDT/2		# Load gravity acceleration / 2
		VSU	VXSC		# Subtract VBIAS, multiply by time factor	(6)
			VBIAS		# Velocity bias from last computation
		VSL2	VAD		# Shift left 2 bits (scale), add to velocity
			V		# Current velocity vector V
		VAD	STADR		# Add acceleration correction			(0)
		STOVL	24D		# STORE UPDATED VELOCITY IN 24-29D
# Page 818
		
		; ========================================================================
		; ALTITUDE RATE AND ALTITUDE COMPUTATION
		;
		; Computes vertical velocity (HDOTDISP) by projecting velocity vector
		; onto unit radius vector (local vertical direction). This is displayed
		; to crew as rate-of-descent on DSKY Noun 63.
		;
		; Also computes predicted altitude (HCALC1) accounting for:
		; - Current altitude rate HDOTDISP
		; - Time interval projection (delta-t)
		; - Landing site reference altitude /LAND/
		; ========================================================================
		
			R		# Load position vector R
		UNIT			# Normalize to unit vector (local vertical)
		STORE	14D		# Store unit R in 14-17D
		DOT	SL1		# Dot product with velocity, shift left 1 bit
			24D		# Updated velocity from 24-29D
		STODL	HDOTDISP	# HDOTDISP = vertical velocity (descent rate)
					# (negative = descending, displayed Noun 63)
			30D		# Load delta-t (time interval)
		SL	DMP		# Shift left 11 bits, multiply
			11D		# Scale shift constant
			HDOTDISP	# Multiply by altitude rate
		DAD	DSU		# Add 36D (previous altitude calc), subtract
			36D		# Previous computation term
			/LAND/		# Landing site reference altitude
		STODL	HCALC1		# HCALC1 = predicted altitude above landing site
					# (displayed to crew via Noun 63)
		
		; ========================================================================
		; HORIZONTAL VELOCITY AND GUIDANCE COMPUTATIONS
		;
		; Computes guidance commands to null horizontal velocity and achieve
		; desired vertical velocity VDGVERT. Uses TAUROD time constant to
		; generate smooth throttle and attitude commands.
		; ========================================================================
		
			HDOTDISP	# Load vertical velocity
		BDSU	DDV		# Subtract desired vertical velocity, divide
			VDGVERT		# VDGVERT = desired descent rate (vertical)
			TAUROD		# TAUROD = guidance time constant
		PDVL	ABVAL		# Push result, load vector			(2)
			GDT/2		# Gravity acceleration / 2
		DDV	SR2		# Divide by GSCALE, shift right 2 bits
			GSCALE		# Scaling constant for gravity
		STORE	20D		# Store scaled gravity magnitude at 20D
		DAD			# Add to previous computation		(0)
		PDVL	CALL		# Push result, load vector, call subroutine	(2)
			UNITX		# Unit X vector in navigation base
			CDU*NBSM	# Transform to stable member coordinates
		DOT			# Dot product with unit radius vector
			14D		# Unit R from 14-17D
		STORE	22D		# Store result at 22D
		BDDV	STADR		# Binary divide, store and add		(0)
		STOVL	/AFC/		# Store in /AFC/ (acceleration from computer)
		
		; ========================================================================
		; VELOCITY BIAS UPDATE AND THRUST MAGNITUDE COMPUTATION
		;
		; Updates VBIAS for next computation cycle, then computes required
		; thrust magnitude accounting for:
		; 1. Current vehicle mass (propellant depletion)
		; 2. Lunar gravity compensation
		; 3. Desired acceleration to null velocity errors
		; 4. Engine throttle limits (min/max thrust constraints)
		; ========================================================================
		
			DELVROD		# Load delta-V from PIPA (raw pulse counts)
		VXSC	VAD		# Scale by KPIP1, add to VBIAS
			KPIP1		# KPIP1 = velocity scaling constant
			VBIAS		# Previous velocity bias
		ABVAL	PDDL		# Compute magnitude, push result		(2)
			THISTPIP	# Load current PIPA time
		DSU	PDDL		# Subtract last PIPA time, push		(4)
			LASTTPIP	# Previous PIPA reading time
			THISTPIP	# Current PIPA time again
		STODL	LASTTPIP	# Store as new LASTTPIP			(2)
		DDV	BDDV		# Divide by shift factor, divide again	(0)
			SHFTFACT	# Scaling shift factor
		PDDL	DMP		# Push result, load and multiply		(2)
			FWEIGHT		# Effective weight constant
			BIT1H		# Bit 1 high-order constant
		DDV	DDV		# Divide by mass, then by scale factor
			MASS		# Current vehicle mass (decreasing as fuel burns)
			SCALEFAC	# Scaling factor for force computation
# Page 819
		
		; Compute total commanded acceleration including gravity correction,
		; lag compensation, and velocity error nulling.
		
		DAD	PDDL		# Add to top of stack, push result		(4)
			0D		# Value at stack position 0
			20D		# Load scaled gravity from 20D
		DDV	DSU		# Divide, subtract				(2)
			22D		# Value at 22D
		DMP	DAD		# Multiply by lag time constant, add to /AFC/
			LAG/TAU		# Lag/time-constant ratio for smooth response
			/AFC/		# Commanded acceleration from computer
		
		; ========================================================================
		; THRUST LIMIT CHECKING
		;
		; Ensures commanded thrust remains within descent engine throttle range.
		; During Apollo 11 descent, engine could throttle from 10% to 60% in
		; braking phase, then 60% to 100% in final approach. This logic prevents
		; commands outside physical engine capability.
		; ========================================================================
		
		PDDL	DDV		# Push result, load and divide			(4)
			MAXFORCE	# Maximum thrust force (Newton)
			MASS		# Divide by current mass for max accel
		PDDL	DDV		# Push max limit, load min force		(6)
			MINFORCE	# Minimum thrust force (Newton)
			MASS		# Divide by current mass for min accel
		PUSH	BDSU		# Push min limit, subtract from commanded	(8)
			2D		# Commanded acceleration at stack 2D
		BMN	DLOAD		# Branch if negative (below minimum)		(6)
			AFCSPOT		# Jump to AFCSPOT to apply limit
		DLOAD	PUSH		# Load commanded value, push			(6)
		BDSU	BPL		# Subtract from max limit, branch if positive
			2D		# (commanded exceeds max)
			AFCSPOT		# Jump to AFCSPOT to apply limit
		DLOAD			# Load limited value				(4)
AFCSPOT		DLOAD			# Load appropriate limit value (2), (4), or (6)
		SETPD			# Set push-down pointer to 2D			(2)
			2D
		STODL	/AFC/		# Store limited acceleration command		(0)
		
		; ========================================================================
		; EXIT TO THROTTLE CONTROL
		;
		; Exits interpretive mode and calls THROTTLE routine to translate
		; commanded acceleration /AFC/ into physical engine throttle setting.
		; This is where guidance equations connect to actual hardware control.
		; ========================================================================
		
ITRPNT2		EXIT			# Exit interpretive mode to native AGC code
		DXCH	MPAC		# Transfer result from MPAC (double-precision)
		TC	BANKCALL	# Cross-bank subroutine call
		CADR	THROTTLE +3	# Call THROTTLE_CONTROL_ROUTINES.agc at entry +3
		
		; ========================================================================
		; HORIZONTAL VELOCITY COMPUTATION (VHORCOMP)
		;
		; Computes magnitude of horizontal velocity component for display to crew
		; via DSKY Noun 60. This is critical information during final approach,
		; allowing Armstrong to assess drift and select landing site.
		;
		; Horizontal velocity = Total velocity - Vertical velocity component
		; ========================================================================
		
		TC	INTPRET		# Re-enter interpretive mode
		VLOAD			# Load updated velocity vector
			24D		# Velocity stored at 24-29D
VHORCOMP	VSL2	VAD		# Shift left 2 bits, add delta-V correction
			DELVS		# Delta-V correction from steering
		VSR2	PDVL		# Shift right 2 bits, push, load position vector
			R		# Position vector R
		UNIT	VXSC		# Normalize to unit vector, multiply by scalar
			HDOTDISP	# Vertical velocity (descent rate)
		VSL1	BVSU		# Shift left 1 bit, subtract from total velocity
		ABVAL			# Compute absolute value (magnitude)
		STORE	VHORIZ		# VHORIZ = horizontal velocity magnitude
					# (displayed to crew via Noun 60)
		EXIT			# Exit interpretive mode
		
		; Display horizontal velocity to crew on DSKY without triggering restart
		; phase change. During Apollo 11 final approach, Armstrong monitored this
		; value to assess lateral drift and select final touchdown point.
		
		TC	BANKCALL	# Call display routine
		CADR	DISPEXIT +3	# Entry point avoids phase change logic

BIT1H		OCT	00001
SHFTFACT	2DEC	1 B-17
# Page 820
BIASFACT	2DEC	655.36 B-28

# *********************************************************************************
# REDESIGNATOR TRAP
# *********************************************************************************

; ========================================================================
; PITFALL - REDESIGNATOR INTERRUPT TRAP
;
; COMMENT-ONLY READERS:
; During the final approach phase, Commander Armstrong could use a hand
; controller with four buttons (+AZ, -AZ, +EL, -EL) to adjust the targeted
; landing site. This interrupt trap captures those button presses and
; schedules the REDESMON task to process the landing site redesignation.
;
; CODE-ALONG READERS:
; This is a hardware interrupt handler triggered by crew input on channel 31.
; Saves interrupt context (BANKRUPT, QRUPT), verifies we're in P64 approach
; phase, reads the redesignator button states (4 bits), and schedules a
; REDESMON task with 5-second delay to debounce and accumulate button presses.
; The ELVIRA register holds button state bits; ZERLINA is the debounce counter.
; ========================================================================

		BANK	11
		SETLOC	F2DPS*11
		BANK

		COUNT*	$$/F2DPS

PITFALL		XCH	BANKRUPT	; Save interrupted bank for restoration
		EXTEND
		QXCH	QRUPT		; Save interrupted return address

		TC	CHECKMM		# IF NOT IN P64, NO REASON TO CONTINUE
		DEC	64		; Check for major mode 64 (approach phase)
		TCF	RESUME		; Not in P64: ignore redesignation, resume

		; We are in P64: read redesignator button states and schedule monitor
		
		EXTEND
		READ	CHAN31		; Read channel 31 (crew input bits)
		COM			; Complement (buttons are active-low)
		MASK	ALL4BITS	; Isolate 4 redesignator bits (bits 1,2,5,6)
		TS	ELVIRA		; Store button state in ELVIRA register
		CAF	TWO
		TS	ZERLINA		; Initialize ZERLINA debounce counter to 2
		CAF	FIVE
		TC	TWIDDLE		; Schedule task with 5-second delay
		ADRES	REDESMON	; Task address: REDESMON (redesignation monitor)
		TCF	RESUME		; Return from interrupt

; ========================================================================
# REDESIGNATOR MONITOR (INITIATED BY PITFALL)
;
; COMMENT-ONLY READERS:
; This task monitors the redesignator buttons over time to accumulate the
; crew's desired landing site adjustment. It debounces button presses
; (ignoring noise), counts button press duration, then applies azimuth and
; elevation increments to shift the targeted landing point. During Apollo 11
; final approach, Armstrong used these controls to steer away from a boulder
; field toward a smoother landing site.
;
; CODE-ALONG READERS:
; REDESMON is a scheduled task (not interrupt) that runs every 0.5 seconds
; monitoring button state stability. ELVIRA holds current button bits;
; ZERLINA is a 2-pass debounce counter. If no bits detected for 2 passes,
; assumes button release and processes accumulated presses by updating
; AZINCR1 (azimuth increment) and ELINCR1 (elevation increment) which the
; guidance equations will apply on next cycle. Each button press contributes:
; AZ: ±2 degrees, EL: ±0.5 degrees, scaled as fractional revolutions.
; ========================================================================

PREMON1		TS	ZERLINA		; Diminish ZERLINA counter, continue monitoring
PREMON2		CAF	SEVEN
		TC	VARDELAY	; Schedule next check in 0.5 seconds
		
REDESMON	EXTEND
		READ	31		; Read channel 31 button states
		COM			; Complement (active-low buttons)
		MASK	ALL4BITS	; Isolate 4 redesignator bits
		XCH	ELVIRA		; Exchange with ELVIRA (new→ELVIRA, old→A)
		TS	L		; Store previous ELVIRA state in L register
		CCS	ELVIRA		# DO ANY BITS APPEAR THIS PASS?
		TCF	PREMON2		# Y:	CONTINUE MONITOR (buttons still pressed)

		; No buttons detected this pass
		CCS	L		# N:	ANY LAST PASS?
		TCF	COUNT'EM	#	Y: 	COUNT 'EM, RESET RUPT, TERMINATE
					#	(buttons were pressed, now released)
# Page 821
		CCS	ZERLINA		#	N: 	HAS ZERLINA REACHED ZERO YET?
		TCF	PREMON1		#		N:	DIMINISH ZERLINA, CONTINUE
					#		(debounce: wait for stable zero)
RESETRPT	CAF	BIT12		#		Y:	RESET RUPT. TERMINATE
		EXTEND			; Reset the interrupt enable bit
		WOR	CHAN13		; Write-OR to channel 13 (re-enable interrupt)
		TCF	TASKOVER	; Terminate task

; ========================================================================
; COUNT'EM - Process button counts and update landing site increments
; ========================================================================
COUNT'EM	CAF	BIT13		# ARE WE IN ATTITUDE-HOLD?
		EXTEND
		RAND	CHAN31		; Read channel 31, mask with BIT13
		EXTEND
		BZF	RESETRPT	# YES: SKIP REDESIGNATION LOGIC.
					; (Attitude-hold disables redesignation)

		CA	L		# NO. (L contains button bits from earlier)
		MASK 	-AZBIT		; Check -AZ button (bit 6)
		CCS	A		; Was -AZ pressed?
-AZ		CS	AZEACH		; Yes: subtract 2 degrees (complement of .03491)
		ADS	AZINCR1		; Add to azimuth increment (double-add subtracts)
		CA	L		; Reload button bits
		MASK	+AZBIT		; Check +AZ button (bit 5)
		CCS	A		; Was +AZ pressed?
+AZ		CA	AZEACH		; Yes: add 2 degrees (.03491 revolutions)
		ADS	AZINCR1		; Add to azimuth increment
		CA	L		; Reload button bits
		MASK	-ELBIT		; Check -EL button (bit 1, +PITCH)
		CCS	A		; Was -EL pressed?
-EL		CS	ELEACH		; Yes: subtract 0.5 degrees (complement of .00873)
		ADS	ELINCR1		; Add to elevation increment
		CA	L		; Reload button bits
		MASK	+ELBIT		; Check +EL button (bit 2, -PITCH)
		CCS	A		; Was +EL pressed?
+EL		CA	ELEACH		; Yes: add 0.5 degrees (.00873 revolutions)
		ADS	ELINCR1		; Add to elevation increment
		TCF	RESETRPT	; Finished: reset interrupt and terminate

# THESE EQUIVALENCES ARE BASED ON GSOP CHAPTER 4, REVISION 16 OF P64LM

; Redesignator button bit definitions (channel 31 input bits)
; Based on Guidance System Operations Plan (GSOP) Chapter 4, Revision 16
+ELBIT		=	BIT2		# -PITCH (bit 2: positive elevation = pitch down)
-ELBIT		=	BIT1		# +PITCH (bit 1: negative elevation = pitch up)
+AZBIT		=	BIT5		; Positive azimuth (right)
-AZBIT		=	BIT6		; Negative azimuth (left)

# Page 822
ALL4BITS	OCT	00063		; Combined mask for all 4 redesignator bits
AZEACH		DEC	.03491		# 2 DEGREES (azimuth increment per button press)
ELEACH		DEC	.00873		# 1/2 DEGREE (elevation increment per button press)
					; Both scaled as fractional revolutions

# ****************************************************************
# R.O.D. TRAP
# ****************************************************************
; RODTRAP processes Rate-Of-Descent (ROD) switch interrupts from the
; Thrust/Translation Controller. The commander can adjust descent rate
; during final approach by clicking the ROD switch up or down.

		BANK	20
		SETLOC	RODTRAP
		BANK
		COUNT*	$$/F2DPS	# ************************

; ========================================================================
; DESCBITS - Process ROD switch inputs
; ========================================================================
; Entry: A register contains bit 7 or bit 6 from channel 16
;   Bit 7 = ROD DOWN (increase descent rate, negative increment)
;   Bit 6 = ROD UP (decrease descent rate, positive increment)
; Updates RODCOUNT which accumulates commander's desired ROD adjustments

DESCBITS	MASK	BIT7		# COME HERE FROM MARKRUPT CODING WITH BIT
		CCS	A		#	7 OR 6 OF CHANNEL 16 IN A; BIT 7 MEANS
		CS	TWO		#	- RATE INCREMENT, BIT 6 + INCREMENT.
		AD	ONE		; Convert to +1 or -2, then add 1 = +2 or -1
		ADS	RODCOUNT	; Add to ROD count (double-add accumulates)
		TCF	RESUME		# TRAP IS RESET WHEN SWITCH IS RELEASED
					; (Return from interrupt)

		BANK	31
		SETLOC	F2DPS*31
		BANK

		COUNT*	$$/F2DPS

# ***********************************************************************************
# DOUBLE PRECISION ROOT FINDER SUBROUTINE (BY ALLAN KLUMPP)
# ***********************************************************************************
; ROOTPSRS is a general-purpose Newton's method root finder used throughout the
; lunar landing guidance. In P63/P64, it solves for time-to-go (TTF) by finding
; roots of trajectory polynomial equations. This mathematical engine enabled the
; AGC to compute fuel-optimal descent trajectories in real-time.
;
; Historical note: Allan Klumpp developed this subroutine at MIT Instrumentation
; Laboratory as part of the Apollo guidance algorithms. This same code executed
; during Armstrong and Aldrin's descent, continuously updating trajectory solutions.
#
#	                                               N        N-1
#	ROOTPSRS FINDS ONE ROOT OF THE POWER SERIES A X  + A   X    + ... + A X + A
#	                                             N      N-1              1     0
# USING NEWTON'S METHOD STARTING WITH AN INITIAL GUESS FOR THE ROOT.  THE ENTERING DATA MUST BE AS FOLLOWS:
#	A	SP	LOC-3		ADRES FOR REFERENCING PWR COF TABL
#	L	SP	N-1		N IS THE DEGREE OF THE POWER SERIES
#	MPAC	DP	X		INITIAL GUESS FOR ROOT
#
#	LOC-2N	DP	A(0)
#		...
#	LOC	DP	A(N)
#	LOC+2	SP	PRECROOT	 PREC RQD OF ROOT (AS FRACT OF 1ST GUESS)
#
# Page 823
# THE DP RESULT IS LEFT IN MPAC UPON EXIT, AND A SP COUNT OF THE ITERATIONS TO CONVERGENCE IS LEFT IN MPAC+2.
# RETURN IS NORMALLY TO LOC(TC ROOTPSRS)+3.  IF ROOTPSRS FAILS TO CONVERGE TO IN 8 PASSES, RETURN IS TO LOC+1 AND
# OUTPUTS ARE NOT TO BE TRUSTED.
#
# PRECAUTION:  ROOTPSRS MAKES NO CHECKS FOR OVERFLOW OR FOR IMPROPER USAGE.  IMPROPER USAGE COULD
# PRECLUDE CONVERGENCE OR REQUIRE EXCESSIVE ITERATIONS.  AS A SPECIFIC EXAMPLE, ROOTPSRS FORMS A DERIVATIVE
# COEFFICIENT TABLE BY MULTIPLYING EACH A(I) BY I, WHERE I RANGES FROM 1 TO N.  IF AN ELEMENT OF THE DERIVATIVE
# COEFFICIENT TABLE = 1 OR >1 IN MAGNITUDE, ONLY THE EXCESS IS RETAINED.  ROOTPSRS MAY CONVERGE ON THE CORRECT
# ROOT NONETHELESS, BUT IT MAY TAKE AN EXCESSIVE NUMBER OF ITERATIONS.  THEREFORE THE USER SHOULD RECOGNIZE:
#	1.  USER'S RESPONSIBILITY TO ASSUR THAT I X A(I) < 1 IN MAGNITUDE FOR ALL I.
#	2.  USER'S RESPONSIBILITY TO ASSURE OVERFLOW WILL NOT OCCUR IN EVALUATING EITHER THE RESIDUAL OR THE DERIVATIVE
#	    POWER SERIES.  THIS OVERFLOW WOULD BE PRODUCED BY SUBROUTINE POWRSERS, CALLED BY ROOTPSRS, AND MIGHT NOT
#	    PRECLUDE EVENTUAL CONVERGENCE.
#	3.  AT PRESENT, ERASABLE LOCATIONS ARE RESERVED ONLY FOR N UP TO 5.  AN N IN EXCESS OF 5 WILL PRODUCE CHAOS.
#	    ALL ERASABLES USED BY ROOTPSRS ARE UNSWITCHED LOCATED IN THE REGION FROM MPAC-33 OCT TO MPAC+7.
#	4.  THE ITERATION COUNT RETURNED IN MPAC+2 MAY BE USED TO DETECT ABNORMAL PERFORMANCE.

; ========================================================================
; ROOTPSRS Entry Point - Initialize for Newton's Method Iteration
; ========================================================================
; This root finder is critical to landing guidance. During Apollo 11's descent,
; it executed every 2 seconds to solve for time-to-go (TTF), ensuring the LM
; followed the computed fuel-optimal trajectory to the landing site.
;
; Algorithm: Newton's method iterates X_new = X_old - F(X)/F'(X)
;   where F(X) is the polynomial and F'(X) is its derivative
;
; Convergence criterion: |X_new - X_old| < PRECROOT * |initial_guess|
; Maximum iterations: 8 passes (returns to LOC+1 if fails to converge)

					# STORE ENTERING DATA, INITIALIZE ERASABLES
; Store return address and set up pointers to coefficient tables
ROOTPSRS	EXTEND
		QXCH	RETROOT		# RETURN ADRES (save in RETROOT for later)
		TS	PWRPTR		# PWR TABLE POINTER (A register = table address)
		DXCH	MPAC +3		# PWR TABLE ADRES, N-1 (fetch table address and degree)
		CA	DERTABLL	; Get address of derivative coefficient table
		TS	DERPTR		# DER TABL POINTER (derivative table in erasable)
		TS	MPAC +5		# DER TABL ADRES (also store in MPAC for reference)
		CCS	MPAC +4		# NO POWER SERIES DEGREE 1 OR LESS (check N-1)
		TS	MPAC +6		# N-2 (degree minus 2 for derivative table)
		CA	ZERO		# MODE USED AS ITERATION COUNTER.  MODE
		TS	MODE		# MUST BE POS SO ABS WON'T COMP MPAC+3 ETC.
					; Initialize iteration counter to zero

					# COMPUTE CRITERION TO STOP ITERATING
; Convergence Criterion Calculation:
; DXCRIT = |initial_root_estimate| * PRECROOT (precision requirement)
; This sets the stopping criterion for Newton iteration. The convergence test
; checks if |correction| < DXCRIT. Using a relative criterion (proportional
; to root magnitude) ensures appropriate precision across different scales.
; For Apollo 11 descent, this criterion balanced computational speed with
; the accuracy needed for thrust vector computations.
		EXTEND
		DCA	MPAC		# FETCH ROOT GUESS, KEEPING IT IN MPAC
		DXCH	ROOTPS		# AND IN ROOTPS
		INDEX	MPAC +3		# PWR TABLE ADRES
		CA	5		# PRECROOT TO A
		TC	SHORTMP		# YIELDS DP PRODUCT IN MPAC
		TC	USPRCADR
		CADR	ABS		# YIELDS ABVAL OF CRITERION ON DX IN MPAC
		DXCH	MPAC
		DXCH	DXCRIT		# CRITERION

					# SET UP DER COF TABL
; Derivative Coefficient Table Construction:
; Constructs derivative coefficients for Newton's method from polynomial
; coefficients. For polynomial P(x) = a(n)*x^n + ... + a(1)*x + a(0),
; derivative P'(x) = n*a(n)*x^(n-1) + ... + 2*a(2)*x + a(1).
; This loop computes i*a(i) for each term and stores in DERIVTAB.
; Loop executes (N-1) times, processing coefficients from highest to lowest.
# Page 824
		EXTEND
		INDEX	PWRPTR
		DCA	3
		DXCH	MPAC		# A(N) TO MPAC

		CA	MPAC +4		# N-1 TO A

DERCLOOP	TS	PWRCNT		# LOOP COUNTER
		AD	ONE
		TC	DMPNSUB		# YIELDS DERCOF = I X A(I) IN MPAC
		EXTEND
		INDEX	PWRPTR
		DCA	1
		DXCH	MPAC		# (I-1) TO MPAC, FETCHING DERCOF
		INDEX	DERPTR
		DXCH	3		# DERCOF TO DER TABLE
		CS	TWO
		ADS	PWRPTR		# DECREMENT PWR POINTER
		CS	TWO
		ADS	DERPTR		# DECREMENT DER POINTER
		CCS	PWRCNT
		TCF	DERCLOOP

					# CONVERGE ON ROOT
; Main Newton-Raphson Iteration Loop:
; Implements the classic Newton iteration formula: x(new) = x(old) - f(x)/f'(x)
; Each pass through ROOTLOOP performs one complete iteration:
;   1. Evaluate derivative P'(x) at current root estimate
;   2. Evaluate polynomial P(x) at current root (residual)
;   3. Compute correction: DX = -P(x)/P'(x)
;   4. Apply correction: new_root = old_root + DX
;   5. Check convergence: |DX| < DXCRIT
; During Apollo 11 descent, this loop typically converged in 3-5 iterations
; for the time-to-go calculations that were critical to fuel management.
ROOTLOOP	EXTEND
		DCA	ROOTPS		# FETCH CURRENT ROOT
		DXCH	MPAC		# LEAVE IN MPAC
		EXTEND
		DCA	MPAC +5		# LOAD A, L WITH DER TABL ADRES, N-2
		TC	POWRSERS	# YIELDS DERIVATIVE IN MPAC

		EXTEND
		DCA	ROOTPS
		DXCH	MPAC		# CURRENT ROOT TO MPAC, FETCHING DERIVATIVE
		DXCH	BUF		# LEAVE DERIVATIVE IN BUF AS DIVISOR
		EXTEND
		DCA	MPAC +3		# LOAD A, L WITH PWR TABL ADRES, N-1
		TC	POWRSERS	# YIELDS RESIDUAL IN MPAC

		TC	USPRCADR
		CADR	DDV/BDDV	# YIELDS -DX IN MPAC

		EXTEND
		DCS	MPAC		# FETCH DX, LEAVING -DX IN MPAC
		DAS	ROOTPS		# CORRECTED ROOT NOW IN ROOTPS

		TC	USPRCADR
		CADR	ABS		# YIELDS ABS(DX) IN MPAC
		EXTEND
# Page 825
		DCS	DXCRIT
		DAS	MPAC		# ABS(DX)-ABS(DXCRIT) IN MPAC
; Convergence Test and Iteration Control:
; Tests if |correction| < |criterion| to determine convergence.
; Also enforces maximum iteration limit of 8 passes (per Alan Klumpp's
; specification). If iterations exceed limit, returns via LOC+1 (BADROOT)
; signaling failure. This safeguard prevents infinite loops during descent
; when computational time is critical. The 8-iteration limit was chosen
; to balance convergence reliability with worst-case execution time.
		CA	MODE
		MASK	BIT4		# KLUMPP SAYS GIVE UP AFTER EIGHT PASSES
		CCS	A
BADROOT		TC	RETROOT

		INCR	MODE		# INCREMENT ITERATION COUNTER
		CCS	MPAC		# TEST HI ORDER DX
		TCF	ROOTLOOP
		TCF	TESTLODX
		TCF	ROOTSTOR
TESTLODX	CCS	MPAC +1		# TEST LO ORDER DX
		TCF	ROOTLOOP
		TCF	ROOTSTOR
		TCF	ROOTSTOR
; Successful Convergence - Store Results and Return:
; Convergence achieved: |correction| < criterion. Store converged root in MPAC
; along with iteration count in MPAC+2. Return via LOC+2 (good root found).
; This return convention allows calling code to branch based on success/failure.
ROOTSTOR	DXCH	ROOTPS
		DXCH	MPAC
		CA	MODE
		TS	MPAC +2		# STORE SP ITERATION COUNT IN MPAC+2
		INDEX	RETROOT
		TCF	2

DERTABLL	ADRES	DERCOFN -3

# ****************************************************************************
# TRASHY LITTLE SUBROUTINES
# ****************************************************************************

; ============================================================================
; TRANSITION: From Complex Guidance Mathematics to Utility Subroutines
;
; The guidance equations and root-finding computations above represent the
; sophisticated mathematics that made pinpoint lunar landings possible. The
; following utility subroutines provide essential support functions: setting
; up interpreter parameters, converting time displays, and handling velocity
; limit checks. Though small in size, these "trashy little subroutines"
; (as the original NASA programmers affectionately named them) were critical
; to the operational success of the guidance system during Apollo 11's descent.
; ============================================================================

; INTPRETX - Interpreter Entry with Target Index Setup:
; Sets up X1 register with target parameter index before entering interpretive
; mode. The target index depends on current guidance phase (WCHPHASE) and
; references TARGTDEX table to select appropriate landing site parameters.
INTPRETX	INDEX	WCHPHASE	# SET X1 ON THE WAY TO THE INTERPRETER
		CS	TARGTDEX
		INDEX	FIXLOC
		TS	X1
		TCF	INTPRET

; TDISPSET - Time Display Setup and Redesignation Time Computation:
; Prepares time-to-go display (TTFDISP) for crew DSKY presentation and computes
; time remaining for landing site redesignation (TREDES). The redesignation
; capability allows crew to adjust landing target during approach phase.
; During Apollo 11, Armstrong used manual redesignation to avoid boulder field.
; TREDES countdown ensures redesignation window closes at appropriate altitude.
TDISPSET	CA	TTF/8
		EXTEND
		MP	TSCALINV
		DXCH	TTFDISP

		CA	EBANK5		# TREDES BECOMES ZERO TWO PASSES
		TS	EBANK		#	BEFORE TCGFAPPR IS REACHED
		EBANK=	TCGFAPPR
		CA	TCGFAPPR
		INCR	BBANK
		INCR	BBANK
		EBANK=	TTF/8
# Page 826
		AD	TTF/8
		EXTEND
		MP	TREDESCL
		AD	-DEC103
		AD	NEGMAX
		TS	L
		CS	L
		AD	L
		AD	+DEC99
		AD	POSMAX
		TS	TREDES
		CS	TREDES
		ADS	TREDES
		TC	Q

; Program Alarm 01406 - TTF Computation Failure:
; Handles alarm condition when time-to-fall (TTF) computation fails due to
; numerical issues (negative discriminant in quadratic solution). Two variants:
; 1406P00 - Critical failure requiring program abort (POODOO)
; 1406ALM - Non-critical alarm allowing continued descent (transitions to RATESTOP)
; The RATESTOP mode provides backup control using attitude rate damping when
; primary guidance is unavailable, ensuring crew maintains vehicle control.
1406P00		TC	POODOO
		OCT	01406
1406ALM		TC	ALARM
		OCT	01406
		TCF	RATESTOP

# *********************************************************************
# SPECIALIZED "PHASCHNG" SUBROUTINE
# *********************************************************************

; FASTCHNG - Fast Phase Change for Restart Protection:
; Specialized phase change routine optimized for speed during time-critical
; guidance computations. Updates restart phase table (PHSNAME3) with minimal
; overhead. During descent, restart protection ensures that if power transient
; or hardware issue triggers AGC restart, guidance can resume from known state
; rather than starting over. This capability was crucial during Apollo 11 when
; 1202 alarms occurred but descent continued safely due to restart protection.
		EBANK=	PHSNAME2
FASTCHNG	CA	EBANK3		# SPECIALIZED 'PHASCHNG' ROUTINE
		XCH	EBANK
		DXCH	L
		TS	PHSNAME3
		LXCH	EBANK
		EBANK=	E2DPS
		TC	A

# *************************************************************************************
# PARAMETER TABLE INDIRECT ADDRESSES
# *************************************************************************************

; Indirect Address Aliases for Guidance Parameters:
; These equates provide mnemonic aliases for guidance target parameters used
; throughout landing trajectory computations. The "DG" suffix indicates "desired
; guidance" values representing landing site objectives. The "2TTF" variants
; reference parameters specifically used in time-to-fall (TTF) calculations.
; Position (R), velocity (V), acceleration (A), and jerk (J) targets are
; accessed through these aliases for code clarity and maintainability.
RDG		=	RBRFG
VDG		=	VBRFG
ADG		=	ABRFG
VDG2TTF		=	VBRFG*
ADG2TTF		=	ABRFG*
JDG2TTF		=	JBRFG*

# *************************************************************************************
# LUNAR LANDING CONSTANTS
# *************************************************************************************

; Landing Guidance Physical and Computational Constants:
; These constants define critical parameters for descent trajectory computations,
; display scaling, and operational limits. TTFSCALE and TSCALINV convert between
; internal fixed-point representation and physical time units. PROJMAX and PROJMIN
; define acceptable landing site slope angles (15-25 degrees) ensuring terrain
; within safe limits. V06N63/V06N64/V06N60 specify DSKY verb/noun combinations
; for crew displays during P63 (braking), P64 (approach), P65/P66/P67 (landing).
# Page 827
TABLTTFL	ADRES	TABLTTF +3	# ADDRESS FOR REFERENCING TTF TABLE
TTFSCALE	=	BIT12
TSCALINV	=	BIT4
-DEC103		DEC	-103
+DEC99		DEC	+99
TREDESCL	DEC	-.08
180DEGS		DEC	+180
1/2DEG		DEC	+.00278
PROJMAX		DEC	.42262 B-3	# SIN(25')/8 TO COMPARE WITH PROJ
PROJMIN		DEC	.25882 B-3	# SIN(15')/8 TO COMPARE WITH PROJ
V06N63		VN	0663		# P63
V06N64		VN	0664		# P64
V06N60		VN	0660		# P65, P66, P67

		BANK	22
		SETLOC	LANDCNST
		BANK
		COUNT*	$$/F2DPS

; Additional Landing Constants (Bank 22 Memory Section):
; HIGHESTF - Maximum time-to-fall computation limit (prevents unrealistic trajectories)
; GSCALE - Lunar gravitational acceleration scaling factor (100 cm/sec² at 2^-11 scale)
; 3/8DP, 3/4DP - Fractional constants for guidance equation interpolations
; DEPRCRIT - Depression angle criterion (-0.02 rad ≈ -1.15°) for landing site
;            slope assessment, ensuring terrain not excessively downward sloping
HIGHESTF	2DEC	4.34546769 B-12
GSCALE		2DEC	100 B-11
3/8DP		2DEC	.375
3/4DP		2DEC	.750
DEPRCRIT	2DEC	-.02 B-1

# Page 828
# **************************************************************************
# **************************************************************************
