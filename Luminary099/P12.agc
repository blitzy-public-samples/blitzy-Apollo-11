# Copyright:	Public domain.
# Filename:	P12.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	838-842
# Mod history:	2009-05-23 HG	Transcribed from page images.
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
; FILE: P12.agc
; MODULE: Powered Ascent Program
; MISSION PHASE: ascent
;
; TL;DR: Implements the P12 powered ascent program managing the Lunar Module's
;        departure from the lunar surface. Coordinates APS (Ascent Propulsion
;        System) ignition through BURN_BABY_BURN, executes closed-loop ascent
;        guidance via ASCENT_GUIDANCE, and achieves orbital insertion for 
;        rendezvous with the Command Module. Critical program enabling Eagle's
;        return from the lunar surface on July 21, 1969.
;
; COMMENT-ONLY READERS: This is the moment of departure. Follow the comments
;        to experience Armstrong and Aldrin's ascent from the Moon as P12
;        guides Eagle into orbit for the historic rendezvous with Columbia.
; CODE-ALONG READERS: Study how P12 initializes ascent parameters, rotates
;        state vectors to ignition time, computes tipover velocities, and
;        interfaces with ASCENT_GUIDANCE for closed-loop trajectory control.
; ============================================================================

# Page 838
		BANK	24
		SETLOC	P12
		BANK

		EBANK=	DVCNTR
		COUNT*	$$/P12

; ============================================================================
; P12LM - POWERED ASCENT PROGRAM ENTRY POINT
;
; The crew initiates P12 by entering VERB 37 ENTER on the DSKY. During Apollo
; 11's historic ascent on July 21, 1969 at mission elapsed time 124:22:00,
; Neil Armstrong spoke: "Nine, eight, seven, six, five, first stage engine on
; ascent. Proceed." This routine prepares the Lunar Module for the 7-minute
; powered ascent that will place Eagle in a 9x46 nautical mile orbit for
; rendezvous with Michael Collins in Columbia.
; ============================================================================

P12LM		TC	PHASCHNG
		OCT	04024

; Verify IMU (Inertial Measurement Unit) is operational and properly aligned.
; The platform must be in the correct orientation for ascent guidance to
; compute accurate attitude commands during the burn.

		TC	BANKCALL
		CADR	R02BOTH		# CHECK THE STATUS OF THE IMU.

; ============================================================================
; ASCENT CONFIGURATION - FLAG INITIALIZATION
;
; Configure software flags for powered ascent operations. These flags control
; RCS jet selection, display outputs, radar modes, and program transitions
; throughout the ascent burn and subsequent orbital operations.
; ============================================================================

; Set MUNFLAG indicating the LM is on the lunar surface preparing for ascent.
; This flag affects navigation updates and guidance computations.

		TC	UPFLAG
		ADRES	MUNFLAG

; Enable 4-jet RCS translation capability for ascent attitude control.
; During ascent, all four RCS jet quads are available for translation maneuvers
; providing maximum control authority.

		TC	UPFLAG		# INSURE 4-JET TRANSLATION CAPABILITY.
		ADRES	ACC4-2FL

; Disable cross-pointer outputs from R10 to prevent conflicting display data
; during the critical ascent phase when crew attention is focused on ascent
; displays and DSKY readouts.

		TC	UPFLAG		# PREVENT R10 FROM ISSUING CROSS-POINTER
		ADRES	R10FLAG		# OUTPUTS.

; Initialize radar modes for R29 downlink telemetry routine. Clears previous
; radar configuration from descent operations.

		TC	CLRADMOD	# INITIALIZE RADMODES FOR R29.

; Clear rendezvous flag used by P22. P12 will set up initial orbit, then
; subsequent rendezvous programs (P20-P25) will handle the actual rendezvous
; navigation and targeting toward the Command Module.

		TC	DOWNFLAG	# CLEAR RENDEZVOUS FLAG FOR P22
		ADRES	RNDVZFLG

; Initialize DVMON (Delta-V Monitor) thresholds and counters. DVMON tracks
; velocity changes during the burn to detect engine performance anomalies.
; THRESH2 sets the sensitivity for detecting thrust variations.

		CAF	THRESH2		# INITIALIZE DVMON
		TS	DVTHRUSH
		CAF	FOUR
		TS	DVCNTR

; Clear tracking mark counter showing R29 downlink data not yet available.
; Telemetry data will be updated once ascent trajectory is established.

		CA	ZERO
		TS	TRKMKCNT	# SHOW THAT R29 DOWNLINK DATA ISN'T READY.

; ============================================================================
; CREW INTERFACE - TIME OF IGNITION (TIG) CONFIRMATION
;
; Display TIG to the crew using VERB 06 NOUN 33. The DSKY shows the planned
; ignition time in mission elapsed time format (hours:minutes:seconds).
; Armstrong and Aldrin can PROCEED to accept the TIG or ENTER to modify it.
; During Apollo 11, TIG was 124:22:00 MET - exactly 21 hours and 36 minutes
; after Eagle's landing, as planned for optimal rendezvous geometry.
; ============================================================================

		CAF	V06N33A
		TC	BANKCALL	# FLASH TIG
		CADR	GOFLASH
		TCF	GOTOPOOH	# TERMINATE - Return to POO (No program)
		TCF	+2		# PROCEED - Accept TIG and continue
		TCF	-5		# ENTER - Load new TIG value

		TC	PHASCHNG
		OCT	04024

; ============================================================================
; GUIDANCE INITIALIZATION AND STATE VECTOR SETUP
;
; Prepare for ascent guidance by initializing lunar rotation rate (WM),
; landing site radius (/LAND/), and ascent engine parameters. These
; initializations establish the reference frame for trajectory computations
; and set up engine performance data for thrust modeling.
; ============================================================================

		TC	INTPRET
		CALL			# INITIALZE WM AND /LAND/
			GUIDINIT	# Moon rotation rate and landing site radius
		SET	CALL
			FLPI		# Set powered flight indicator flag
			P12INIT		# Initialize ascent engine data (APS parameters)

# Page 839
; ============================================================================
; STATE VECTOR ROTATION TO IGNITION TIME
;
; The guidance computer must know the Lunar Module's exact position and
; velocity at the moment of ignition. This section rotates the current state
; vectors forward to TIG using precision trajectory integration. For Apollo 11,
; this predicted Eagle's state at 124:22:00 MET with accuracy sufficient for
; the rendezvous that would follow 3.5 hours later.
; ============================================================================

P12LMB		DLOAD
			(TGO)A		# SET TGO TO AN INITIAL NOMINAL VALUE.
		STODL	TGO		# Time-to-go initialized for guidance
			TIG		# Load time of ignition
		STCALL	TDEC1		# Store as integration target time
			LEMPREC		# ROTATE THE STATE VECTORS TO THE
					# IGNITION TIME.

; Transform velocity vector from navigation coordinates to stable member
; coordinates using REFSMMAT (reference stable member matrix). Scale result
; by 2^-7 for proper velocity representation (meters/centisecond).

		VLOAD	MXV		
			VATT		# Velocity at TIG from integration
			REFSMMAT	# Stable member coordinate transformation
		VSL1			# Scale for 2^-7 meters/centisecond
		STOVL	V1S		# COMPUTE V1S = VEL(TIG)*2(-7)M/CS.

; Transform position vector using REFSMMAT and scale by 2^-24 for proper
; position representation (meters). This gives the LM's location on the lunar
; surface at ignition time.

			RATT		# Position at TIG from integration  
		MXV	VSL6		# Transform and scale for 2^-24 meters
			REFSMMAT
		STCALL	R		# COMPUTE R = POS(TIG)*2(-24)M.
			MUNGRAV		# COMPUTE GDT1/2(TIG)*2(-7)M/CS.
					# (Gravity at TIG divided by 2)

; Compute unit vector along position radius for YCOMP (vertical reference).
; During powered ascent, the guidance uses local vertical as one coordinate
; axis for defining the desired flight path angle and cross-range steering.

		VLOAD	UNIT		# Load position vector R
			R
		STCALL	UNIT/R/		# COMPUTE UNIT/R/ FOR YCOMP.
			YCOMP		# Compute Y-axis component for guidance

; Prepare scaled gravity vector for guidance computations. Right shift by
; 5 decimal places and complement to get proper sign convention.

		SR	DCOMP		# Scale and complement result
			5D
		STODL	XRANGE		# INITIALIZE XRANGE FOR NOUN 76.

; Initialize desired injection velocity (VINJNOM) and radial velocity rate
; (RDOTDNOM) for the target orbit. These nominal values define the insertion
; conditions that will result in the planned 9x46 nautical mile orbit for
; Apollo 11's rendezvous with Columbia.

			VINJNOM		# Nominal injection velocity
		STODL	ZDOTD		# Store as desired out-of-plane rate
			RDOTDNOM	# Nominal radial velocity rate
		STORE	RDOTD		# Store as desired radial rate
		EXIT

		TC	PHASCHNG
		OCT	04024

; ============================================================================
; CREW TARGETING INTERFACE - CROSS-RANGE AND APOAPSIS DISPLAY
;
; Display cross-range distance and apoapsis altitude using VERB 06 NOUN 76.
; Cross-range (XRANGE) allows the crew to adjust the insertion orbit's plane
; to optimize rendezvous geometry. Apoapsis defines the high point of the
; insertion orbit. The crew can PROCEED to accept values or ENTER to modify
; the targeting parameters for different rendezvous strategies.
; ============================================================================

NEWLOAD		CAF	V06N76		# FLASH CROSS-RANGE, AND APOLUNE VALUES.
		TC	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH	# TERMINATE - Return to POO
		TCF	+2		# PROCEED - Accept targeting and continue
		TCF	NEWLOAD		# ENTER NEW DATA - Re-display for changes

; Store P12 address for program identification during restart protection.
; WHICH variable identifies the active major program for executive scheduler.

		CAF	P12ADRES
		TS	WHICH

		TC	PHASCHNG
		OCT	04024

; ============================================================================
; TIPOVER MANEUVER INITIALIZATION
;
; The ascent begins with a brief vertical rise phase, then the LM "tips over"
; to begin building horizontal velocity for orbital insertion. This section
; computes the initial velocity at tipover by adding 49 feet/second (actually
; 57 FPS as the comment notes - the constant 49FPS is scaled) in the vertical
; direction to the ignition velocity. During Apollo 11, this pitchover began
; about 10 seconds after ignition, transitioning from vertical climb to the
; trajectory arc that would carry Eagle into orbit.
; ============================================================================

		TC	INTPRET
		DLOAD	SL		# Load cross-range distance
			XRANGE
			5D		# Scale by 2^5
		DAD			# Add to Y coordinate
# Page 840
			Y
		STOVL	YCO		# Store computed Y coordinate
			UNIT/R/		# Load unit radius vector

; Compute tipover velocity by adding vertical component to ignition velocity.
; V(TIPOVER) = V(IGN) + 49FPS (actually 57 FPS) * UNIT/R/
; This establishes the initial velocity state for the ascent guidance loop.

		VXSC	VAD		# Vector scale and add
			49FPS		# Constant for vertical velocity increment
			V1S		# Ignition velocity vector
		STORE	V		# V(TIPOVER) = V(IGN) + 57FPS (UNIT/R/)

; Compute radial velocity component (RDOT) by dot product with unit radius.
; This is the rate of altitude change, critical for guidance to monitor
; during the vertical rise and pitchover phases.

		DOT	SL1		# Dot product, scale left 1
			UNIT/R/
		STOVL	RDOT		# RDOT * 2(-7)

; Compute Z-axis orientation by cross product of unit radius with Q-axis.
; This defines the out-of-plane direction for the coordinate system used
; during ascent guidance computations.

			UNIT/R/
		VXV	UNIT		# Cross product and normalize to unit vector
			QAXIS		# Body-fixed axis
		STORE	ZAXIS1		# Store Z-axis for guidance reference frame

; ============================================================================
; TRANSFER TO ASCENT GUIDANCE
;
; With all initialization complete, P12 transfers control to the ASCENT
; routine in ASCENT_GUIDANCE.agc. This is the closed-loop guidance algorithm
; that will compute thrust vector commands throughout the burn, adjusting
; attitude to achieve the desired insertion orbit. During Apollo 11's ascent,
; this guidance ran continuously for approximately 7 minutes, placing Eagle
; into the planned orbit with errors of only a few feet per second.
; ============================================================================

		SETGO			# Set flag and go to ASCENT
			FLVR		# Set vehicle rate flag
			ASCENT		# Jump to ascent guidance main loop

; ============================================================================
; RETURN FROM ASCENT GUIDANCE - CUTOFF ATTITUDE COMPUTATION
;
; When ascent guidance completes (at velocity cutoff), control returns here
; to compute final pitch and yaw angles. These attitude angles define the
; LM's orientation at orbital insertion. The computed attitude is then
; displayed to the crew via DSKY for verification that the insertion was
; successful. For Apollo 11, this occurred at 124:29:00 MET when Eagle
; reached orbital velocity of approximately 5,545 feet per second.
; ============================================================================

P12RET		DLOAD			# Return point from ASCENT guidance
			ATP		# ATP(2)*2(18)
		DSQ	PDDL		# Square and push to stack
			ATY		# ATY(2)*2(18)
		DSQ	DAD		# Square ATY and add to ATP^2
		BZE	SQRT		# Branch if zero, else take square root
			YAWDUN		# Skip yaw calculation if zero

; Compute yaw angle using ARCSIN of ATY divided by magnitude.
; Yaw represents rotation about the vertical axis, indicating any
; out-of-plane velocity component at insertion.

		SL1	BDDV		# Scale and divide
			ATY
		ARCSIN			# Compute arc sine for yaw angle
YAWDUN		STOVL	YAW		# Store yaw angle (zero if skipped)
			UNFC/2		# Load thrust direction unit vector

; Compute pitch angle using ARCCOS of dot product with unit radius.
; Pitch represents the angle from local vertical, indicating the
; flight path angle at insertion. Complement the result for proper
; sign convention (negative pitch = nose down relative to horizon).

		UNIT	DOT		# Normalize and dot product
			UNIT/R/		# With unit radius vector
		SL1	ARCCOS		# Scale and compute arc cosine
		DCOMP			# Complement for sign convention
		STORE	PITCH		# Store pitch angle
		EXIT
		TC	PHASCHNG
		OCT	04024

		TC	DOWNFLAG
		ADRES	FLPI

		INHINT
		TC	IBNKCALL
		CADR	PFLITEDB
		RELINT

		TC	POSTJUMP
		CADR	BURNBABY

; ============================================================================
; P12INIT - ENGINE AND TARGET INITIALIZATION SUBROUTINE
;
; This subroutine initializes critical engine performance parameters and
; target orbital parameters for the ascent. Called during P12 setup and
; also reused by abort program P71. Loads engine thrust characteristics,
; exhaust velocity, and nominal target values for the insertion orbit.
;
; COMMENT-ONLY READERS: Behind the scenes, the computer loads the APS engine
; specifications - how much thrust it produces, how fast it burns fuel, and
; what orbit the LM should reach. These were pre-calculated values tested
; during mission planning and loaded before launch.
;
; CODE-ALONG READERS: Initializes 1/DV (inverse delta-velocity), AT (thrust
; acceleration), TBUP (burn-up time), TTO (time-to-go), and VE (exhaust
; velocity) from fixed constants. These parameters define the APS engine
; performance model used by ascent guidance equations.
; ============================================================================

P12INIT		DLOAD			# INITIALIZE ENGINE DATA.  USED FOR P12 AND
# Page 841
			(1/DV)A		# P71.
; Load inverse delta-velocity parameter (1/DV) into three storage locations.
; This parameter represents the reciprocal of velocity change capability
; and is used in ascent guidance trajectory computations.

		STORE	1/DV3
		STORE	1/DV2
		STODL	1/DV1
			(AT)A

; Load thrust acceleration (AT) - the acceleration produced by the APS
; engine. For the LM ascent stage, this was approximately 0.5 G initially,
; increasing as propellant mass decreased during the burn.

		STODL	AT
			(TBUP)A

; Load burn-up time (TBUP) - the time required to consume all APS propellant
; at maximum thrust. For Apollo 11, the APS carried enough fuel for
; approximately 7 minutes of powered flight with reserves.

		STODL	TBUP
			ATDECAY

; Compute time-to-go (TTO) initialization value from thrust decay parameter.
; The DCOMP inverts the sign, and SL shifts left 11 bits for proper scaling.

		DCOMP	SL
			11D
		STORE	TTO

; Load exhaust velocity (VE) from APS exhaust velocity constant (APSVEX).
; Exhaust velocity determines propellant efficiency. The APS used Aerozine 50
; fuel with nitrogen tetroxide oxidizer, producing approximately 10,000 ft/sec
; exhaust velocity.

		SLOAD	DCOMP
			APSVEX
		SR2
		STORE	VE

; Check FLAP flag and branch to COMMINIT for common initialization.
; If FLAP is set, return immediately; otherwise continue to target setup.

		BOFF	RVQ
			FLAP
			COMMINIT
; ============================================================================
; COMMINIT - COMMON TARGET INITIALIZATION
;
; Initializes target orbital parameters for ascent. Used by P12 (normal
; ascent), P70 (pre-planned abort), and P71 (contingency abort if not
; following P70). Sets target radius (RCO) by adding injection altitude
; (HINJECT) to lunar surface radius (/LAND/), zeros cross-range and
; out-of-plane components, and computes the orbital plane (QAXIS) from
; the CSM's position and velocity vectors.
;
; For Apollo 11, HINJECT was set to achieve a 9x46 nautical mile orbit,
; placing Eagle in an elliptical orbit with perilune at approximately
; 9 NM altitude for rendezvous with Columbia in the 60 NM circular orbit.
; ============================================================================

COMMINIT	DLOAD	DAD		# INITIALIZE TARGET DATA.  USED BY P12, P70
			HINJECT		# AND P71 IF IT DOES NOT FOLLOW P70.
			/LAND/

; Compute target orbital radius (RCO) = injection altitude + lunar radius.
; This defines the apoapsis of the insertion orbit.

		STODL	RCO
			HI6ZEROS

; Zero out cross-range and out-of-plane target components. TXO (x-component),
; YCO (y-component), RDOTD (radial velocity desired), and YDOTD (y-velocity
; desired) are all set to zero for a standard coplanar ascent.

		STORE	TXO
		STORE	YCO
		STORE	RDOTD
		STOVL	YDOTD
			VRECTCSM

; Compute orbital plane unit vector (QAXIS) from CSM state vectors.
; Cross product of CSM velocity and position defines the orbit normal,
; which becomes the target plane for LM ascent. This ensures the LM
; reaches the same orbital plane as the CSM for efficient rendezvous.

		VXV	MXV		# VRECTCSM x RRECTCSM -> orbit normal
			RRECTCSM
			REFSMMAT
		UNIT			# Normalize to unit vector
		STORE	QAXIS
		RVQ

; P12ADRES provides the address of P12TABLE for table lookups during
; ascent guidance processing. This address is stored in WHICH to direct
; the guidance routines to the correct parameter table.

P12ADRES	REMADR	P12TABLE

		SETLOC	P12A
		BANK
		COUNT*	$$/P12

; ============================================================================
; GUIDINIT - GUIDANCE INITIALIZATION SUBROUTINE
;
; Initializes fundamental guidance parameters needed before ascent begins.
; Computes the lunar rotation rate vector (WM) in the guidance reference
; frame and stores the lunar surface radius (/LAND/) for use in altitude
; calculations. Called early in P12 setup to establish the reference frame
; for all subsequent guidance computations.
;
; COMMENT-ONLY READERS: Before launch, the computer must know exactly how
; the Moon is rotating and how far the surface is from the Moon's center.
; This routine performs those calculations so the guidance equations can
; properly account for the Moon's rotation during the 7-minute ascent.
;
; CODE-ALONG READERS: Loads current time, computes Moon's rotation vector
; (WM) by rotating UNITZ by REFSMMAT and scaling by MOONRATE, and computes
; lunar surface radius magnitude (/LAND/) from current position (RLS).
; ============================================================================

GUIDINIT	STQ	SETPD
			TEMPR60		# Save return address
			0D		# Set push-down pointer to zero

; Load unit Z vector (UNITZ) representing Moon's rotation axis and push
; to stack. Load current mission time and push.

		VLOAD	PUSH
			UNITZ
		RTB	PUSH
			LOADTIME

; Convert current time to position via RP-TO-R routine, computing the
; transformation from inertial to rotating coordinates.

		CALL
			RP-TO-R
# Page 842

; Transform Z-axis by REFSMMAT reference frame matrix and scale by lunar
; rotation rate (MOONRATE). Result is WM - the Moon's angular velocity
; vector in the guidance coordinate system. This accounts for the Moon's
; rotation during ascent, which affects the inertial velocity required
; to achieve a given orbit.

		MXV	VXSC
			REFSMMAT
			MOONRATE
		STOVL	WM		# Store lunar rotation vector
			RLS

; Compute magnitude of landing site position vector (RLS) to obtain lunar
; surface radius at the landing location. The Moon is not perfectly spherical,
; so this local radius value ensures accurate altitude calculations during
; ascent. Scale left 3 bits for proper units.

		ABVAL	SL3
		STCALL	/LAND/		# Store lunar surface radius
			TEMPR60		# Return to caller

; ============================================================================
; ASCENT GUIDANCE CONSTANTS
;
; These constants define nominal trajectory parameters for lunar ascent:
; ============================================================================

; 49FPS - Expected radial velocity at tipover maneuver completion.
; At tipover (vertical rise complete), the LM should have approximately
; 49 feet per second outward velocity before beginning the pitchover
; toward orbital insertion. This vertical velocity was achieved during
; the first ~10 seconds of ascent.

49FPS		2DEC	.149352 B-6	# EXPECTED RDOT AT TIPOVER

; VINJNOM - Nominal injection velocity for 30 NM apoapsis orbit.
; Target velocity of 5,509.5 feet per second achieves a 9x30 nautical mile
; orbit with 19.5 fps radial velocity component. These values were baseline
; targets; actual Apollo 11 ascent achieved 9x46 NM orbit for optimal
; rendezvous geometry with Columbia.

VINJNOM		2DEC	16.7924 B-7	# 5509.5 FPS(APO=30NM WITH RDOT=19.5FPS)

; RDOTDNOM - Nominal radial velocity desired at insertion.
; 19.5 feet per second radial velocity at insertion creates the desired
; elliptical orbit shape. Positive RDOT means moving away from lunar center,
; defining the orbit's eccentricity.

RDOTDNOM	2DEC	.059436 B-7	# 19.5 FPS
