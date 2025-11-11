# Copyright:	Public domain.
# Filename:	ASCENT_GUIDANCE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	843-856
# Mod history:	2009-05-23 HG	Transcribed from page images.
#		2009-06-05 RSB	Fixed a couple of typos.
#		2009-06-07 RSB	Corrected a typo.
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
; FILE: ASCENT_GUIDANCE.agc
; MODULE: Ascent Guidance
; MISSION PHASE: ascent/rendezvous
;
; TL;DR: Implements lunar surface ascent trajectory guidance from liftoff to
;        orbital insertion. Manages vertical rise phase, pitchover sequence,
;        and closed-loop guidance for rendezvous targeting. Computes thrust
;        direction commands for APS (Ascent Propulsion System) and targets
;        insertion parameters for Command Module rendezvous.
;
; COMMENT-ONLY READERS: Follow Eagle's historic ascent from Tranquility Base
;        on July 21, 1969. This is the guidance that brought Armstrong and
;        Aldrin back to orbit after humanity's first moonwalk.
; CODE-ALONG READERS: Study orbital insertion guidance algorithms, velocity-
;        to-be-gained calculations, and closed-loop thrust vector control.
; ============================================================================

; ============================================================================
; HISTORICAL CONTEXT - APOLLO 11 ASCENT
;
; On July 21, 1969 at mission elapsed time 124:22:00, after spending 21 hours
; and 36 minutes on the lunar surface, Neil Armstrong and Buzz Aldrin fired
; the Ascent Propulsion System to lift Eagle off the Moon. This guidance code
; successfully inserted the Lunar Module into a 9 x 46 nautical mile orbit,
; setting up the rendezvous with Michael Collins in Columbia.
;
; The ascent consisted of three phases:
; 1. VERTICAL RISE: ~10 seconds straight up from the surface to clear terrain
; 2. PITCHOVER: Gradual rotation toward horizontal orbital trajectory
; 3. INSERTION GUIDANCE: Closed-loop targeting to achieve rendezvous orbit
;
; Unlike descent (which Armstrong flew semi-manually), ascent was fully
; automatic. The guidance computer controlled attitude and monitored velocity,
; shutting down the APS engine precisely when orbital parameters were achieved.
; ============================================================================

# Page 843
		BANK	34
		SETLOC	ASCFILT
		BANK

		EBANK=	DVCNTR

		COUNT*	$$/ASENT

; ============================================================================
; ATMAG - ASCENT TRAJECTORY MAGNITUDE COMPUTATION
;
; Entry point for ascent guidance initialization and trajectory setup.
; Called by P12 (Powered Ascent Program) after APS ignition.
;
; This routine computes the initial conditions for ascent guidance, including
; acceleration magnitudes, time-to-go estimates, and velocity parameters that
; will guide the Lunar Module from the surface to orbital insertion.
;
; COMMENT-ONLY READERS: The ascent engine has ignited. Eagle is beginning its
; climb from the lunar surface. This guidance computation sets up the
; trajectory that will carry the astronauts back to orbit for rendezvous.
;
; CODE-ALONG READERS: Initializes guidance parameters using interpretive
; language. Computes delta-V ratios and time-to-burn-up (TBUP) for the
; closed-loop guidance algorithm that follows.
; ============================================================================

ATMAG		TC	PHASCHNG
		OCT	00035
		TC	INTPRET		; Enter interpretive mode for vector/matrix operations
		
; Check if RCS (Reaction Control System) is failed. If so, branch to ASCENT.
		BON
			FLRCS		; RCS failure flag
			ASCENT
		
; Compute delta-V convergence check: ABDVCONV - MINABDV
; ABDVCONV = Absolute delta-V for orbital insertion convergence
; MINABDV = Minimum allowable delta-V threshold
		DLOAD	DSU
			ABDVCONV
			MINABDV
		
; If result negative (delta-V insufficient), terminate ascent guidance.
; Clear SURFFLAG to indicate vehicle has left surface.
		BMN	CLEAR
			ASCTERM4	; Branch to ascent termination
			SURFFLAG	; Surface contact flag (cleared = airborne)
		
; Clear rendezvous flag and load constant for computation.
		CLEAR	SLOAD
			RENDWFLG	; Rendezvous display flag
			BIT3H		; Constant = octal 4 (used in scaling)
		
; Divide by ABDVCONV to normalize delta-V components.
		DDV	EXIT
			ABDVCONV
		
; Store normalized delta-V ratios in memory locations 1/DV0 through 1/DV3.
; These represent the inverse delta-V components for each axis, used to
; compute guidance steering commands during powered ascent.
		DXCH	MPAC		; Exchange MPAC (interpretive accumulator) with memory
		DXCH	1/DV3		; Store in 1/DV3, retrieve previous value
		DXCH	1/DV2		; Cascade through delta-V component storage
		DXCH	1/DV1
		DXCH	1/DV0		; Final storage of normalized component
		
		TC	INTPRET		; Re-enter interpretive mode
		
; Sum all four delta-V components: 1/DV0 + 1/DV1 + 1/DV2 + 1/DV3
; This computes the total inverse delta-V magnitude for trajectory shaping.
		DLOAD	DAD
			1/DV0
			1/DV1
		DAD	DAD
			1/DV2
			1/DV3
		
; Multiply by VE (exhaust velocity) and time scaling factor.
; Result contributes to TBUP (Time-to-Burn-Up) calculation.
		DMP	DMP
			VE		; Exhaust velocity of APS engine
			2SEC(9)		; Time scaling constant
		
; Shift left 3 bits (multiply by 8) and push to stack.
; Load current TBUP estimate.
		SL3	PDDL
			TBUP		; Time remaining until propellant depletion
		
; Refine TBUP estimate by adding correction term and subtracting time offset.
		SR1	DAD		; Shift right 1, add correction
		DSU
			6SEC(18)	; 6-second time constant
		STODL	TBUP		; Store updated time-to-burn-up
		
; Compute average thrust acceleration: AT = VE / (2 * TBUP)
; This represents the mean acceleration during remaining ascent.
			VE
		SR1	DDV		; Divide exhaust velocity by twice the burn time
			TBUP
		STCALL	AT		; Store thrust acceleration, call main guidance
# Page 844
			ASCENT
BIT3H		OCT	4		; Constant for scaling computations

# Page 845
		BANK	30
		SETLOC	ASENT
		BANK
		COUNT*	$$/ASENT

; ============================================================================
; TRANSITION: From initialization to main ascent guidance loop
;
; The setup phase is complete. Eagle's guidance computer now enters the main
; ascent guidance routine that will continuously compute the required thrust
; direction to achieve orbital insertion. This closed-loop algorithm runs
; repeatedly during the ascent, adjusting the spacecraft's attitude to follow
; the optimal trajectory toward rendezvous with Columbia.
;
; During Apollo 11's actual ascent, this guidance loop executed approximately
; every 2 seconds, steering the Lunar Module through the pitchover maneuver
; and into the targeted insertion orbit.
; ============================================================================

; ============================================================================
; ASCENT - MAIN ASCENT GUIDANCE COMPUTATION ROUTINE
;
; Core closed-loop guidance algorithm for lunar ascent. Computes the required
; thrust vector to achieve targeted orbital insertion parameters for Command
; Module rendezvous.
;
; ALGORITHM OVERVIEW:
; 1. Compute current position magnitude and direction vectors
; 2. Calculate vertical (Z-axis) and horizontal (L-axis) components
; 3. Determine effective gravity and required accelerations
; 4. Compute steering commands to null velocity-to-be-gained errors
; 5. Generate thrust vector direction for autopilot
;
; Called repeatedly during powered ascent until cutoff conditions met.
; ============================================================================

ASCENT		VLOAD	ABVAL		; Load position vector R, compute absolute magnitude
			R		; Current position vector from lunar center
		STOVL	/R/MAG		; Store radius magnitude
			ZAXIS1		; Load local vertical reference (Z-axis)
		
; Compute vertical velocity component: ZDOT = Z-axis dot V
; This represents the rate of altitude gain during ascent.
		DOT	SL1		; Dot product with velocity, shift left 1
			V		; Current velocity vector - result: Z.V = ZDOT*2(-8)
		STOVL	ZDOT		; Store vertical velocity component (scaled 2^-7)
			ZAXIS1
		
; Compute lateral axis: LAXIS = Z-axis cross UNIT/R/
; This defines the direction perpendicular to both vertical and radial.
		VXV	VSL1		; Vector cross product, shift left 1
			UNIT/R/		; Unit radial vector - result: Z X UR = LAXIS*2(-2)
		STORE	LAXIS		; Store lateral axis vector (scaled 2^-1)
		
; Compute out-of-plane velocity component: YDOT = L-axis dot V
; This represents velocity perpendicular to the ascent plane.
		DOT	SL1		; Dot product with velocity, shift left 1
			V		# L.V = YDOT*2(-8)
		STCALL	YDOT		; Store lateral velocity, call YCOMP subroutine
			YCOMP		; Computes Y-axis component of reference frame
		
; ============================================================================
; EFFECTIVE GRAVITY AND ANGULAR MOMENTUM CALCULATION
;
; Computes GEFF (effective gravity) which accounts for both gravitational
; acceleration and centrifugal effects. This is the net acceleration the
; guidance system must overcome to achieve the desired trajectory.
; ============================================================================

; Load half-timestep gravity increment computed by integration routines.
		VLOAD			; Load gravity delta vector
			GDT1/2		# GDT1/2 scaled 2^-7 meters/centisecond
		V/SC	DOT		; Divide by 2 seconds, dot with radial
			2SEC(18)	; Time constant for scaling
			UNIT/R/		# Dot product: G.UR gives radial gravity component
		
; Compute angular momentum per unit mass: H = R cross V
; Angular momentum magnitude determines orbital characteristics.
		PDVL	VXV		; Push radial gravity to stack, load unit radial
			UNIT/R/		# Load UNIT/R/ scaled 2^-1
			V		# UR X V = angular momentum per radius
		
; Compute H^2/R^3 term (centrifugal acceleration contribution).
		VSQ	DDV		; Square angular momentum vector, divide by R
			/R/MAG		# H^2/R^2 scaled 2^-16, then divide by R
		
; Combine radial gravity and centrifugal term for effective gravity.
		SL1	DAD		; Shift left 1, add radial gravity from stack
		STADR			; Add address offset
		STODL	GEFF		; Store effective gravity (scaled 2^10 m/cs^2)
		
; ============================================================================
; VELOCITY-TO-BE-GAINED COMPUTATION
;
; Computes differences between desired and actual velocity components.
; These errors drive the guidance steering commands. ZDOT = vertical (altitude),
; YDOT = out-of-plane (crossrange), RDOT = radial (downrange).
; ============================================================================

; Vertical velocity error: desired minus actual.
			ZDOTD		; Load desired vertical velocity
		DSU			; Subtract actual vertical velocity
			ZDOT
		STORE	DZDOT		; Store vertical velocity error (scaled 2^7 m/cs)
		
; Scale vertical error by Z-axis vector to get component in inertial frame.
		VXSC	PDDL		; Vector times scalar, push result, load next
			ZAXIS1		; Scale DZDOT by Z-axis unit vector
			YDOTD		; Load desired out-of-plane velocity
		
; Out-of-plane (crossrange) velocity error: desired minus actual.
		DSU			; Subtract actual from desired
			YDOT
		STORE	DYDOT		; Store lateral velocity error (scaled 2^7 m/cs)
		
; Scale lateral error by L-axis vector.
		VXSC	PDDL		; Vector times scalar, push result
			LAXIS		; Scale DYDOT by lateral axis unit vector
			RDOTD		; Load desired radial velocity
# Page 846
		
; Radial (downrange) velocity error: desired minus actual.
		DSU			; Subtract actual from desired
			RDOT
		STORE	DRDOT		; Store radial velocity error (scaled 2^7 m/cs)
		
; Construct total velocity-to-be-gained vector: VG = DRDOT*R + DYDOT*L + DZDOT*Z
; This vector points in the direction the vehicle must accelerate to achieve
; the desired orbital insertion conditions.
		VXSC	VAD		; Scale DRDOT by unit radial, add to previous
			UNIT/R/		; Radial component
		VAD	VSL1		; Add all three components, shift left 1
		STADR			; Add address offset
		STORE	VGVECT		; Store velocity-to-be-gained vector
		
; ============================================================================
; GRAVITY COMPENSATION AND TGO (TIME-TO-GO) COMPUTATION
;
; Adjusts velocity-to-be-gained for gravity effects during the remaining burn.
; Computes time-to-go (TGO) until engine cutoff at orbital insertion.
; Different computations used depending on whether main APS engine or RCS
; thrusters are providing thrust.
; ============================================================================

; Compensate VG vector for gravity that will act during remaining burn time.
; Gravity will accelerate the vehicle, so subtract this from required VG.
		DLOAD	DMP		; Load time-to-go, multiply by effective gravity
			TGO		# Current TGO estimate (seconds)
			GEFF		; Effective gravity (m/cs^2)
		VXSC	VSL1		; Scale by radial direction, shift left 1
			UNIT/R/		# Gravity compensation vector: TGO * GEFF * UR
		BVSU			; Subtract from velocity-to-be-gained
			VGVECT		# VG compensated for gravity during remaining burn
		STORE	VGVECT		; Store corrected VG (used for downlink telemetry)
		
; Transform VG vector to body coordinates for crew display (Noun 85).
; This shows astronauts the thrust direction in their reference frame.
		MXV	VSL1		; Matrix multiply by navigation base to body matrix
			XNBPIP		; Navigation-to-body transformation matrix
		STOVL	VGBODY		; Store body-frame VG for DSKY display
			VGVECT		; Reload inertial-frame VG
		
; Check which propulsion system is active: main APS engine or RCS thrusters.
		ABVAL	BOFF		; Compute magnitude of VGVECT
			FLRCS		; Check RCS flag: if clear, use main engine
			MAINENG		; Branch to main engine TGO calculation

; RCS THRUSTER MODE: Simple TGO calculation.
; RCS provides constant thrust, so TGO = VG / (thrust acceleration).
		DDV			; Divide VG magnitude by RCS thrust acceleration
			AT/RCS		; RCS thrust acceleration constant
		STCALL	TGO		; Store time-to-go, call termination check
			ASCTERM2	; Check for ascent guidance termination

; ============================================================================
; MAIN ENGINE (APS) MODE: Rocket Equation TGO Calculation
;
; Main Ascent Propulsion System uses rocket equation because thrust is high
; and mass decreases significantly during burn. TGO computed from:
; TGO = TBUP * VG * (1 - KT * VG/VE) / VE - TTO
; where TBUP = burn-up time, VE = exhaust velocity, KT = gravity constant,
; TTO = tailoff correction for engine shutdown transient.
; ============================================================================

MAINENG		DDV	PUSH		; Divide VG by exhaust velocity: VG/VE (dimensionless)
			VE		; Exhaust velocity of APS engine
		
; Compute correction factor: (1 - KT * VG/VE)
; This accounts for changing mass and gravity effects.
		DMP	BDSU		; Multiply by KT1, subtract from near-one constant
			KT1		; Gravity parameter (KT - 0.25)
			NEARONE		; Constant ≈ 1.0
		
; Complete TGO calculation using rocket equation.
		DMP	DMP		; Multiply by TBUP and original VG/VE
			TBUP		; Time to burn up remaining propellant
					; Result: TBUP * VG * (1-KT*VG/VE) / VE = TGO
		
; Subtract tailoff time: brief period after shutdown command when engine
; still produces thrust as combustion chamber depressurizes.
		DSU			; Subtract tailoff compensation
			TTO		; Tailoff time constant
		STORE	TGO		; Store computed time-to-go until cutoff

; Prepare TGO in scaled format for subsequent calculations.
		SR	DCOMP		; Shift right 11, complement (negate)
			11D
		STODL	TTOGO		; Store negative TGO scaled 2^-28 centiseconds
			TGO		; Reload TGO for comparison checks

; ============================================================================
; ENGINE SHUTDOWN LOGIC AND GUIDANCE PHASE TRANSITIONS
;
; Checks if time-to-go is less than 4 seconds. If so, and if not in idle mode,
; commands engine shutdown (ENGOFF). Otherwise proceeds to check which guidance
; phase should be active based on remaining time.
; ============================================================================

; Check if in idle mode (no active engine). If so, skip 4-second test.
		BON	DSU		; Branch on IDLEFLAG, subtract 4 seconds
			IDLEFLAG	; If IDLEFLAG set, skip to T2TEST
			T2TEST
			4SEC(17)	; TGO - 4 seconds (scaled 2^-17 cs)

; If TGO < 4 seconds and engine is running, shut down for orbital insertion.
		BMN			; Branch if minus (TGO < 4 sec)
			ENGOFF		; Initiate engine shutdown sequence

; ============================================================================
; T2TEST - TIME-BASED GUIDANCE MODE SWITCHING
;
; Compares TGO against threshold T2A to determine whether to use component-wise
; guidance (early in ascent) or integrated guidance with rate damping (near
; orbital insertion). This switching provides optimal trajectory shaping
; throughout the ascent profile.
; ============================================================================

T2TEST		DLOAD			; Load time-to-go
			TGO
		DSU	BMN		; Subtract T2A threshold, branch if negative
# Page 847
			T2A		; Time threshold for guidance mode switch
			CMPONENT	; If TGO < T2A, use component guidance

; ============================================================================
; RATE DAMPING PARAMETER CALCULATION (D12)
;
; Computes D12, a time-based damping parameter that shapes the guidance
; response during final approach to orbital insertion. This parameter is
; derived from the rocket equation and burn time relationships.
; ============================================================================

; Calculate 1 - TGO/TBUP (burn fraction remaining)
		DLOAD	DSU		; Load TBUP, subtract TGO
			TBUP		; Time to burn up propellant
			TGO
		DDV	CALL		; Divide by TBUP: (TBUP-TGO)/TBUP = 1-TGO/TBUP
			TBUP
			LOGSUB		; Compute natural logarithm via subroutine

; Compute -TGO/L where L is the logarithm just calculated
		SL	PUSH		; Shift left 5, push -L to stack
			5		; -L now in PDL(0), stack depth (2)
		BDDV	BDSU		; Divide TGO by -L, then add TBUP
			TGO		; -TGO/L
			TBUP		; TBUP + TGO/L = D12 scaled 2^-17

; Store D12 and check if rate calculations should be skipped
		PUSH	BON		; Store D12 in PDL(2), stack depth (4)
			FLPC		; Check if PC (powered coast?) flag is set
			NORATES		; If FLPC=1, skip to NORATES (B=0, D=0)

; Check if TGO is less than T3 threshold to activate rate calculations
		DLOAD	DSU		; Load TGO, subtract T3
			TGO
			T3		; T3 threshold for rate activation
		BPL	SET		; Branch if plus (TGO >= T3)
			RATES		; TGO >= T3: compute full rate terms
			FLPC		; Set FLPC flag for future passes

; ============================================================================
; NORATES PATH - ZERO RATE GAINS
;
; When FLPC flag is set or before T3 threshold, rate damping gains are zeroed.
; Guidance uses position errors only, without velocity feedback. This prevents
; excessive steering during early ascent when rates would be destabilizing.
; ============================================================================

NORATES		DLOAD			; Load zeros
			HI6ZEROS
		STORE	PRATE		; Store zero to PRATE (radial rate gain = 0)
		STORE	YRATE		; Store zero to YRATE (lateral rate gain = 0)
		GOTO			; Skip rate calculations
			CONST		; Continue to constant computation

; ============================================================================
; RATES CALCULATION - POSITION AND VELOCITY ERROR RATE TERMS
;
; Computes rate damping gains based on position and velocity errors relative
; to desired trajectory. These gains provide stability during final approach
; to orbital insertion velocity and altitude. The calculations involve multiple
; intermediate parameters (D21, E) derived from time-to-go relationships.
; ============================================================================

RATES		DLOAD	DSU		; Load TGO, subtract D12
			TGO
			02D		; D21 = TGO - D12, scaled 2^-17
		PUSH	SL1		; Store D21 in PDL(4), shift left 1, stack (6)
		BDSU	SL3		; Compute (TGO/2 - D21), shift left 3
			TGO		; E = (TGO/2 - D21) scaled 2^-13, stack (8)

; ============================================================================
; RADIAL POSITION ERROR COMPUTATION (DR)
;
; Calculates the radial position error: difference between current radius plus
; predicted change (R + RDOT*TGO) and the desired cutoff radius (RCO).
; This error drives guidance corrections in the radial direction.
; ============================================================================

		PDDL	DMP		; Push E to PDL(6), load TGO
			TGO
			RDOT		; RDOT * TGO, predict radial change
		DAD	DSU		; Add current radius, subtract target
			/R/MAG		; R + RDOT*TGO (predicted radius at cutoff)
			RCO		; Subtract RCO (desired cutoff radius)
					; MPAC = -DR (negative radial error) scaled 2^-24

; ============================================================================
; RADIAL RATE GAIN CALCULATION (PRATE)
;
; Computes radial rate feedback gain using position error (DR) and the second
; derivative of radius (DRDOT). This gain shapes the guidance response to
; radial velocity errors, ensuring smooth approach to target orbit altitude.
; ============================================================================

		PDDL	DMP		; Push -DR to PDL(8), stack (10), load DRDOT
			DRDOT		; Second derivative of radius
			04D		; Multiply by D21: D21*DRDOT scaled 2^-24
		DAD	SL2		; Add -DR: (D21*DRDOT - DR) scaled 2^-22, stack (8)
		DDV	DDV		; Divide by E, then divide by TGO
			06D		; (D21*DRDOT - DR)/E scaled 2^-9
			TGO		; Final result scaled appropriately
		STORE	PRATE		; Store radial rate gain B scaled 2^8
		BMN	DLOAD		; Branch if minus (B<0): Check magnitude
			CHKBMAG		; B<0 acceptable, verify not too large
# Page 848
			HI6ZEROS		; B>0 not permitted (would oppose ascent)
		STCALL	PRATE		; Store zero to PRATE, skip to PROK
			PROK		; Continue with acceptable B value

; ============================================================================
; RADIAL RATE GAIN MAGNITUDE LIMIT CHECK (CHKBMAG)
;
; Validates that the radial rate gain B is within acceptable limits when
; negative. Computes B/TAU (rate gain normalized by time constant) and clamps
; to PRLIMIT if the magnitude is excessive. This prevents over-aggressive
; guidance commands that could destabilize the trajectory.
; ============================================================================

CHKBMAG		SR4	DDV		; Shift B right 4 bits, divide by TBUP
			TBUP		; (B / TAU) scaled 2^21
		DSU	BPL		; Subtract limit, branch if within range
			PRLIMIT		; Maximum allowable (B/TAU) = 2^21
			PROK		; B magnitude acceptable, continue
		DLOAD	DMP		; B too large: Load limit and scale
			PRLIMIT		; Maximum permissible B/TAU value
			TBUP		; BMAX = PRLIMIT * TBUP scaled 2^4
		SL4			; Shift left 4: BMAX scaled 2^8
		STORE	PRATE		; Store clamped B value to PRATE

; ============================================================================
; LATERAL RATE GAIN CALCULATION (YRATE)
;
; Computes the lateral (out-of-plane) rate feedback gain D using position
; error (DY) and acceleration (DYDOT). Lateral guidance keeps the vehicle
; aligned with the orbital plane while correcting for any cross-track errors
; accumulated during ascent. This is critical for precise rendezvous targeting.
; ============================================================================

PROK		DLOAD			; Load time-to-go for lateral error calculation
			TGO
		DMP	DAD		# YDOT TGO
			YDOT		; Lateral velocity * TGO = predicted change
			Y		# Y + YDOT TGO
		DSU	PDDL		# Y + YDOT TGO - YCO
			YCO		# MPAC = - DY*(-24.) IN PDL(8)	(10)
			DYDOT		; Second derivative of lateral position
		DMP	DAD		# D21 DYDOT - DY		(8)
			04D		; Multiply by D21 from PDL(4)
		SL2	DDV		# (D21 DYDOT - DY)/E*2(-9)
		DDV	SETPD		# (D21 DYDOT - DY)/E TGO*2(8)
			TGO		#	= D*2(8)
			04		; Reset PDL pointer to position 4
		STORE	YRATE		; Store lateral rate gain D scaled 2^8

; ============================================================================
; GUIDANCE CONSTANT COMPUTATION (CONST)
;
; Computes the two fundamental guidance constants (PCONS and YCONS) used in
; the steering equations. These constants incorporate rate gains, position
; errors, and current rates to form the basis for thrust vector commands:
;   PCONS (A) = -DRDOT/L - D12*B  (radial component constant)
;   YCONS (C) = -DYDOT/L - D12*D  (lateral component constant)
; Where D12 is a time-dependent coefficient from PDL, B is radial rate gain,
; and D is lateral rate gain computed earlier.
; ============================================================================

CONST		DLOAD	DMP		# LOAD B*2(8)
			PRATE		# B D12*2(-9)
			02D		; Multiply by D12 coefficient from PDL(2)
		PDDL	DDV		# D12 B IN PDL(4)	(6)
			DRDOT		# LOAD DRDOT*2(-7)
			00D		# -DRDOT/L*2(-7)
		SR2	DSU		# (-DRDOT/L-D12 B)=A*2(-9)	(4)
		STADR			; Store address for later reference
		STODL	PCONS		; Store radial guidance constant A
			YRATE		# D*2(8)
		DMP	PDDL		# D12 D,EXCH WITH -L IN PDL(0)	(2,2)
		BDDV	SR2		# -DYDOT/L*2(-9)
			DYDOT		; Lateral acceleration derivative
		DSU			# (-DYDOT/L-D12 D)=C*2(-9)
			00D		; Subtract D12*D term
		STORE	YCONS		; Store lateral guidance constant C

; ============================================================================
; THRUST COMPONENT COMPUTATION (CMPONENT)
;
; Applies the guidance constants to compute commanded thrust acceleration
; components. The guidance law is:
;   ATR = (A + B*(T-T0))/TBUP - GEFF  (radial thrust component)
;   ATY = (C + D*(T-T0))/TBUP         (lateral thrust component)
; These components are then combined with the local vertical (UNIT/R/) and
; local horizontal (LAXIS) unit vectors to form the commanded acceleration
; vector AH, which determines the vehicle's thrust attitude. The magnitude
; of AH is constrained by available thrust and vehicle mass.
; ============================================================================

CMPONENT	SETPD	DLOAD		; Reset PDL pointer, begin component calc
			00D
 			100CS
 		DMP
			PRATE		# B(T-T0)*2(-9)
		DAD	DDV		# (A+B(T-T0))*2(-9)
# Page 849
			PCONS		# (A+B(T-T0))/TBUP*2(8)
			TBUP		; Divide by time constant (TBUP)
		SL1	DSU		; Shift left, subtract gravity
			GEFF		# ATR*2(9)
		STODL	ATR		; Store commanded radial thrust component
			100CS		; Reload time since ignition for lateral calc
		DMP	DAD		; Compute lateral thrust component
			YRATE		; Multiply by lateral rate gain D
			YCONS		# (C+D(T-T0))*2(-9)
		DDV	SL1		; Divide by time constant
			TBUP		; Normalize to thrust capability
		STORE	ATY		# ATY*2(9)
		VXSC	PDDL		# ATY UY*2(8)		(6)
			LAXIS		; Scale lateral unit vector by ATY
			ATR		; Prepare radial component
		VXSC	VAD		; Scale radial unit vector, add to lateral
			UNIT/R/		; Local vertical (radial) unit vector
		VSL1	PUSH		# AH*2(9) IN PDL(0)	(6)
		ABVAL	PDDL		# AH(2) IN PDL(34)
			AT		# AHMAG IN PDL(6)	(8)
		DSQ	DSU		# (AT(2)-AH(2))*2(18)
			34D		# =ATP2*2(18)
		PDDL	PUSH		#			(12)
			AT		; Square of total acceleration magnitude
		DSQ	DSU		# (AT(2)KR(2)-AH(2))*2(18)	(10)
			34D		# =ATP3*2(18)
		BMN	DLOAD		# IF ATP3 NEG,GO TO NO-ATP
			NO-ATP		# LOAD ATP2, IF ATP3 POS
			8D		; If negative, cannot achieve desired AH
		SQRT	GOTO		# ATP*2(9)
			AIMER		; Proceed with normal thrust aiming

; ============================================================================
; TRANSITION: From Normal Guidance to Thrust Shortfall Handling
;
; The ascent guidance equations compute desired acceleration components. When
; the commanded horizontal acceleration AH exceeds available thrust capability,
; the guidance enters NO-ATP mode. This occurs during high-demand phases when
; guidance wants more thrust than the APS engine can deliver. Rather than
; failing, the guidance scales down the commanded vector to match available
; thrust, ensuring maximum acceleration is always applied in the best direction.
; ============================================================================

NO-ATP		DLOAD	BDDV		# KR AT/AH = KH		(8)
			6D		; Load acceleration magnitude from PDL
		VXSC			# KH AG*2(9)
			00D		; Scale commanded vector by capability ratio
		STODL	00D		# STORE NEW AH IN PDL(0)
			HI6ZEROS	; Load zero for ATP (no parallel component)

; ============================================================================
; TRANSITION: From Thrust Components to Final Thrust Vector
;
; Having computed the horizontal guidance component AH and parallel component
; ATP, this section constructs the final commanded thrust unit vector UNFC/2.
; This vector represents the direction in which the Lunar Module should point
; during ascent. The vertical component ATP is signed based on vertical velocity
; (DZDOT) to ensure proper thrust direction relative to gravity and velocity.
; ============================================================================

AIMER		SIGN		; Transfer sign based on vertical rate
			DZDOT		; Sign of vertical velocity component
		STORE	ATP		; Store parallel thrust component ATP
		VXSC		; Scale local vertical by ATP
			ZAXIS1		# ATP ZAXIS *2(8).
		VSL1	VAD		# AT*2(0)
			00D		; Add horizontal component from PDL
		STORE	UNFC/2		# WILL BE OVERWRITTEN IF IN VERT. RISE.
		SETPD	BON		; Reset PDL, check for P12 return
			00D		; Initialize PDL pointer
			FLPI		; Flag for P12 integration mode
			P12RET		; Return to P12 if flag set
		BON		; Check vertical rise flag
# Page 850
			FLVR		; Flag indicating vertical rise phase
			CHECKALT	; Branch to altitude check for vert rise
; ============================================================================
; MAINLINE GUIDANCE COMPUTATION
;
; This is the primary guidance loop for powered ascent after vertical rise.
; Computes the commanded thrust unit vector UNWC/2 (downward pointing) and
; checks for ascent termination conditions. During normal ascent, the vehicle
; follows this guidance to achieve the velocity and position required for
; orbital insertion and rendezvous with the Command Module.
; ============================================================================

MAINLINE	VLOAD	VCOMP		; Load radial unit vector, complement it
			UNIT/R/		; Local vertical direction (upward)
		STODL	UNWC/2		; Store downward thrust direction (negative)
			TXO		; Load time of ignition (TXO)
		DSU	BPL		; Compute time since ignition
			PIPTIME		; Current mission elapsed time
			ASCTERM		; If result positive, terminate ascent
		BON		; Check rotation flag
			ROTFLAG		; Is attitude rotation maneuver active?
			ANG1CHEK	; Branch to angle check if rotating
CLRXFLAG	CLEAR	CLEAR		; Clear flags for normal ascent operation
			NOR29FLG	# START r29 IN ASCENT PHASE.
			XOVINFLG	# ALLOW X-AXIS OVERRIDE

; ============================================================================
; ASCENT TERMINATION CHECK (ASCTERM)
;
; Monitors ascent progress and manages guidance termination conditions.
; Checks RCS trimming mode, engine failure status, and displays crew
; information (V06N63) showing velocity-to-be-gained and time-to-go.
; During Apollo 11 Eagle's ascent on July 21, 1969, this section monitored
; the LM's climb from Tranquility Base to a 9x46 nautical mile orbit,
; preparing for rendezvous with Michael Collins in Columbia.
; ============================================================================

ASCTERM		EXIT			; Exit interpreter to native AGC code
		CA	FLAGWRD9	; Check RCS trimming mode flag
		MASK	FLRCSBIT	; Isolate RCS flag bit
		CCS	A		; Test flag state
		TCF	ASCTERM3	; If RCS trimming, skip displays
		TC	INTPRET		; Enter interpreter for guidance computation
		CALL			; Call guidance interface routine
			FINDCDUW -2	; Update DAP with guidance commands
ASCTERM1	EXIT			; Exit interpreter for flag checks
 +1		CA	FLAGWRD9	# INSURE THAT THE NOUN 63 DISPLAY IS
 		MASK	FLRCSBIT	# BYPASSED IF WE ARE IN THE RCS TRIMMING
		CCS	A		# MODE OF OPERATION
		TCF	ASCTERM3	; Skip display if RCS trimming active
		CA	FLAGWRD8	# BYPASS DISPLAYS IF ENGINE FAILURE IS
		MASK	FLUNDBIT	# INDICATED.
		CCS	A		; Check engine failure flag
		TCF	ASCTERM3	; Skip display if engine failed
		CAF	V06N63*		; Load display verb/noun code V06N63
		TC	BANKCALL	; Call display routine
		CADR	GODSPR		; Display velocity-to-be-gained, time-to-go
		TCF	ASCTERM3	; Continue to termination logic
ASCTERM2	EXIT			; Alternate entry point (unused)
ASCTERM3	TCF	ENDOFJOB	; Terminate ascent guidance job
ASCTERM4	EXIT			; Entry for zero DAP errors
		INHINT			; Inhibit interrupts during DAP update
		TC	IBNKCALL	# NO GUIDANCE THIS CYCLE -- HENCE ZERO
		CADR	ZATTEROR	# THE DAP COMMANDED ERRORS.
		TCF	ASCTERM1 +1	; Return to flag check and display

; ============================================================================
; ALTITUDE CHECK FOR VERTICAL RISE EXIT
;
; During the initial vertical rise phase (first ~10 seconds after liftoff),
; checks if the LM has climbed above 25,000 feet. Below this altitude,
; maintains vertical ascent to clear terrain and maximize altitude gain.
; Above 25K feet, initiates pitchover maneuver to build horizontal velocity.
; ============================================================================

CHECKALT	DLOAD	DSU		; Load current radius magnitude
			/R/MAG		; Distance from lunar center
			/LAND/		; Subtract landing site radius
		DSU	BMN		# IF H LT 25K CHECK Z AXIS ORIENTATION
			25KFT		; Subtract 25,000 foot threshold
			CHECKYAW	; If below 25K, check yaw alignment
# Page 851

; ============================================================================
; VERTICAL RISE EXIT LOGIC (EXITVR)
;
; Manages the transition from vertical rise phase to pitchover/orbital
; insertion guidance. Clears vertical rise flag (FLVR) and rotation flag
; (ROTFLAG), then schedules the next guidance cycle 10 seconds later.
; This section executes during the critical transition when the LM pivots
; from straight-up climb to horizontal velocity building for orbit.
; ============================================================================

EXITVR		CLEAR	BON		; Clear vertical rise flag
			FLVR		; Vertical rise complete
			ROTFLAG		; Check rotation flag
			MAINLINE	; If set, continue to main guidance
		DLOAD	DAD		; Load current PIPA time
			PIPTIME		; Time from IMU accelerometers
			10SECS		; Add 10 second delay
		STCALL	TXO		; Store as next guidance time
			MAINLINE	; Call main guidance loop
EXITVR1		CLRGO			; Clear and go alternate entry
			ROTFLAG		; Clear rotation flag
			EXITVR		; Return to vertical rise exit

		SETLOC	ASENT1
		BANK
		COUNT*	$$/ASENT

; ============================================================================
; ANGLE CHECK 1 (ANG1CHEK)
;
; Verifies vehicle attitude alignment during ascent by checking angular
; relationships between thrust vector (UNFC/2), body axes (XNBPIP), and
; local vertical (UNIT/R/). Compares dot products against threshold angles
; (COSTHET1, COSTHET2) to ensure the LM maintains proper orientation
; during the climb from lunar surface to orbital insertion.
; Misalignment beyond thresholds triggers rotation flag for attitude correction.
; ============================================================================

ANG1CHEK	VLOAD	DOT		; Load commanded thrust vector
			UNFC/2		; Half of commanded acceleration
			XNBPIP		; Body X-axis in stable member coords
		DSU	BPL		; Subtract threshold cosine
			COSTHET1	; First angle threshold
			OFFROT		; If positive (aligned), disable rotation
		VLOAD	DOT		; Load body X-axis
			XNBPIP		; Navigation base X-axis
			UNIT/R/		; Local vertical unit vector
		DSU	BMN		; Subtract second threshold
			COSTHET2	; Second angle threshold
			KEEPVR1		; If negative (misaligned), keep rotating
OFFROT		CLRGO			; Clear rotation flag and continue
			ROTFLAG		; Disable rotation corrections
			CLRXFLAG	; Branch to clear X-axis flag

		BANK	7
		SETLOC	ASENT2
		BANK
		COUNT*	$$/ASENT

SETXFLAG	=	CHECKYAW	; Alternate entry label

; ============================================================================
; YAW ALIGNMENT CHECK (CHECKYAW)
;
; Verifies the LM's yaw orientation during vertical rise by comparing the
; body Y-axis (YNBPIP) against the computed ascent plane. Constructs the
; desired ascent direction from lateral (ATY*LAXIS) and vertical (ATP*ZAXIS1)
; components, then checks if Y-axis misalignment exceeds 5 degrees (SIN5DEG).
; Prohibits X-axis override (XOVINFLG) during this critical alignment check.
; If yaw error is within tolerance, continues vertical rise; otherwise,
; maintains rotation flag for attitude correction via RCS thrusters.
; ============================================================================

CHECKYAW	SET			; Set X-axis override inhibit flag
			XOVINFLG	# PROHIBIT X-AXIS OVERRIDE
		DLOAD	VXSC		; Load lateral acceleration component
			ATY		; Lateral guidance command
			LAXIS		; Lateral axis unit vector
		PDDL	VXSC		; Push to stack, load vertical component
			ATP		; Vertical thrust component
			ZAXIS1		; Local vertical axis
		VAD	UNIT		; Add vectors and normalize
		PUSH	DOT		; Push result, compute dot product
# Page 852
			YNBPIP		; Body Y-axis in stable member
		ABS	DSU		; Absolute value, subtract threshold
			SIN5DEG		; Sine of 5 degrees (yaw tolerance)
		BPL	DLOAD		; If within tolerance, branch
			KEEPVR		; Continue vertical rise
			RDOT		; Load radial velocity component
		DSU	BPL		; Subtract threshold velocity
			40FPS		; 40 feet per second upward rate
			EXITVR1		; If above 40 fps, exit vertical rise
		GOTO			; Otherwise continue
			KEEPVR		; Maintain vertical rise mode

		BANK	5
		SETLOC	ASENT3
		BANK
		COUNT*	$$/ASENT

; Vertical rise exit thresholds:
SIN5DEG		2DEC	0.08716 B-2	; Sine of 5° yaw tolerance (scaled B-2)
40FPS		2DEC	0.12192 B-7	; 40 ft/sec radial velocity (scaled B-7)

		BANK	14
		SETLOC	ASENT4
		BANK
		COUNT*	$$/ASENT

; ============================================================================
; VERTICAL RISE CONTINUATION (KEEPVR)
;
; Maintains pure vertical ascent by commanding thrust straight up (anti-radial
; direction). Recalls the line-of-sight vector from pushlist stack and stores
; as downward command (UNWC/2), then loads local vertical (UNIT/R/) as the
; normalized thrust command (UNFC/2). This keeps the LM climbing straight up
; to clear terrain and gain altitude before initiating pitchover for orbital
; insertion. Used during first ~10 seconds after liftoff from Tranquility Base.
; ============================================================================

KEEPVR		VLOAD	STADR		# RECALL LOSVEC FROM PUSHLIST
		STORE	UNWC/2		; Store line-of-sight as down command
KEEPVR1		VLOAD			; Load local vertical vector
			UNIT/R/		; Unit radial vector (up direction)
		STCALL	UNFC/2		; Store as thrust command, return
			ASCTERM		; Back to termination check

; ============================================================================
; ENGINE SHUTOFF TIMING (ENGOFF)
;
; Calculates precise engine cutoff timing for orbital insertion. Computes
; time remaining until desired velocity achieved (current time - PIPTIME + TTOGO),
; then schedules engine shutdown with 1-bit (10ms) precision. Critical for
; achieving correct insertion orbit for rendezvous with Columbia. Sets up
; WAITLIST task to execute ENGOFF1 at computed shutoff time. Disables delta-V
; monitor (IDLEFLAG) during final shutdown sequence.
;
; During Apollo 11 ascent on July 21, 1969, this routine calculated the
; precise APS cutoff time to insert Eagle into the 9x46 nautical mile orbit
; that enabled successful rendezvous with Michael Collins in the Command Module.
; ============================================================================

ENGOFF		RTB			; Return to basic (exit interpreter)
			LOADTIME	; Load current mission time
		DSU	DAD		; Subtract PIPA time, add time-to-go
			PIPTIME		; Last accelerometer read time
			TTOGO		; Time-to-go for orbital insertion
		DCOMP	EXIT		; Complement result, exit interpreter
		TC	TPAGREE		# FORCE SIGN AGREEMENT ON MPAC, MPAC +1.
		CAF	EBANK7		; Load erasable bank 7 address
		TS	EBANK		; Switch to EBANK 7
		EBANK=	TGO		; Declare EBANK for TGO variable
		INHINT			; Inhibit interrupts for critical section
		CCS	MPAC +1		; Check clear and skip on time delta
		TCF	+3		# C(A) = DT - 1 BIT (positive, >1 bit)
		TCF	+2		# C(A) = 0 (zero case)
		CAF	ZERO		# C(A) = 0 (negative became 0)
		AD	BIT1		# C(A) = 1 BIT OR DT (add minimum time)
# Page 853
		TS	ENGOFFDT	; Store delta-time for engine shutoff
; Schedule ENGOFF1 on WAITLIST for precise engine shutoff timing:
		TC	TWIDDLE		; Call WAITLIST task scheduler
		ADRES	ENGOFF1		; Address of shutdown task to execute
		TC	PHASCHNG	; Phase change for restart protection
		OCT	47014		; Phase table configuration
		-GENADR	ENGOFFDT	; Restart protection address
		EBANK=	TGO		; EBANK declaration
		2CADR	ENGOFF1		; Two-word address (bank + address)

		TC	INTPRET		; Return to interpretive mode
		SET	GOTO		; Set flag and branch
			IDLEFLAG	# DISABLE DELTA-V MONITOR during final shutdown
			T2TEST		; Continue with termination logic

; ============================================================================
; ENGINE SHUTOFF EXECUTION (ENGOFF1)
;
; WAITLIST task executed at computed shutoff time. Commands APS engine
; shutdown via ENGINOF2 routine, then sets up high-priority job (PRIO17)
; to execute postburn CUTOFF logic. Phase change protects restart capability.
;
; This is the moment of orbital insertion - when Eagle's ascent engine
; cuts off and the LM enters orbit for rendezvous with Columbia.
; ============================================================================

ENGOFF1		TC	IBNKCALL	# SHUT OFF THE ENGINE at computed time
		CADR	ENGINOF2	; Cross-bank call to engine shutoff

; Set up postburn guidance job:
		CAF	PRIO17		# Priority 17 job (high priority)
		TC	FINDVAC		# Find available core set for job
		EBANK=	WHICH		; EBANK for job variables
		2CADR	CUTOFF		; Execute postburn CUTOFF logic

; Restart protection for CUTOFF job:
		TC	PHASCHNG	; Phase change routine
		OCT 	07024		; Phase table code
		OCT	17000		; Job configuration
		EBANK=	TGO		; EBANK declaration
		2CADR	CUTOFF		; Restart address for CUTOFF

		TCF	TASKOVER	; Task complete, return to scheduler

; ============================================================================
; POSTBURN CUTOFF LOGIC
;
; Executes after APS engine shutdown. Sets FLRCS flag to enable RCS for
; attitude control, displays V16N63 to crew showing final orbit parameters.
; Crew can PROCEED to continue (CUTOFF1) or ENTER to terminate (TERMASC).
;
; V16N63 displays: apogee altitude, perigee altitude, time of free fall.
; This confirms successful orbital insertion for rendezvous with Columbia.
; ============================================================================

CUTOFF		TC	UPFLAG		# SET FLRCS FLAG (enable RCS control)
		ADRES	FLRCS		; Flag address in erasable memory

; Display final orbit parameters to crew:
 -5		CAF	V16N63		; Verb 16 Noun 63 (flash display)
 		TC	BANKCALL	; Cross-bank call
		CADR	GOFLASH		; Display and wait for crew response
		TCF	+3		; ENTER pressed - terminate ascent
		TCF	CUTOFF1		; PROCEED pressed - continue with CUTOFF1
		TCF	-5		; RECYCLE pressed - redisplay

; ENTER response - jump to terminal ascent:
 +3		TC	POSTJUMP	; Cross-bank jump
 		CADR	TERMASC		; Terminate ascent guidance

; ============================================================================
; CUTOFF1 - ATTITUDE ERROR RESET AND DEADBAND ADJUSTMENT
;
; Crew selected PROCEED from V16N63 display. Zero attitude errors before
; reducing deadband, then reduce to minimum deadband for fine attitude control
; in orbit. This prepares LM for rendezvous maneuvers with tight attitude hold.
; ============================================================================

CUTOFF1		INHINT			; Inhibit interrupts
		TC	IBNKCALL	# ZERO ATTITUDE ERRORS before changing deadband
		CADR	ZATTEROR	; Zero all three-axis attitude errors
		TC	IBNKCALL	; Cross-bank call
		CADR	SETMINDB	; Set minimum deadband for precise control
		TC	POSTJUMP	; Jump to next phase
		CADR	CUTOFF2		; Continue with CUTOFF2 display
# Page 854

V16N63		VN	1663		; Display constant: V16N63
		BANK	30		; Switch to bank 30
		SETLOC	ASENT5		; Set location counter
		BANK			; Bank directive
		COUNT*	$$/ASENT	; Instruction counter

; ============================================================================
; CUTOFF2 - FINAL ORBIT CONFIRMATION DISPLAY
;
; Display V16N85C to crew - final orbital parameters after APS cutoff.
; Shows altitude, altitude rate, crossrange. Crew verifies successful
; insertion into rendezvous orbit. PROCEED restores astronaut-desired
; deadband and continues. ENTER terminates ascent guidance immediately.
; ============================================================================

CUTOFF2		TC	PHASCHNG	; Phase change for restart protection
		OCT	04024		; Phase table code

; Display final orbit state to crew:
		CAF	V16N85C		; Verb 16 Noun 85 (flash display)
		TC	BANKCALL	; Cross-bank call
		CADR	GOFLASH		; Display and wait for response
		TCF	TERMASC		; ENTER - terminate ascent guidance
		TCF	+2		# PROCEED - restore deadband and continue
		TCF	CUTOFF2		; RECYCLE - redisplay

; ============================================================================
; PROCEED response path - restore astronaut-selected deadband:
; ============================================================================

		INHINT			; Inhibit interrupts for deadband change
		TC	IBNKCALL	; Cross-bank call
		CADR	RESTORDB	; Restore deadband to astronaut's setting
		TCF	GOTOPOOH	; Return to POO program (idle)

; ============================================================================
; TERMASC - TERMINAL ASCENT GUIDANCE
;
; Entered when crew presses ENTER during postburn displays, indicating
; desire to terminate ascent guidance immediately. Restores deadband to
; astronaut-selected value, clears LETABORT flag to disallow further aborts
; (ascent complete), and returns to POO idle program.
;
; Historical note: After successful orbital insertion, Eagle was in a
; 9 x 46 nautical mile orbit. This routine marks completion of powered
; ascent from lunar surface - all subsequent maneuvers handled by
; rendezvous programs P20-P25.
; ============================================================================

TERMASC		TC	PHASCHNG	; Phase change for restart
		OCT	04024		; Phase table code

		INHINT			# RESTORE DEADBAND desired by astronaut
		TC	IBNKCALL	; Cross-bank call
		CADR	RESTORDB	; Restore astronaut's deadband setting
		TC	DOWNFLAG	# DISALLOW ABORTS AT THIS TIME (ascent complete)
		ADRES	LETABORT	; Clear abort-allowed flag
		TCF	GOTOPOOH	; Return to Program POO (idle)

V16N85C		VN	1685		; Display constant: V16N85

		BANK 27		; Switch to bank 27
		SETLOC	ASENT1	; Set location counter
		BANK		; Bank directive
		COUNT* $$/ASENT	; Instruction counter

; ============================================================================
; YCOMP - CROSSRANGE (Y-AXIS) COMPUTATION
;
; Computes crossrange distance Y by projecting unit radius vector onto
; Q-axis (perpendicular to desired orbital plane), then scaling by RCO
; (radius of circular orbit). Used to determine lateral deviation from
; nominal ascent trajectory for guidance corrections.
;
; Returns Y in meters scaled appropriately for guidance calculations.
; ============================================================================

YCOMP		VLOAD	DOT		; Load UNIT/R/, dot product with QAXIS
			UNIT/R/		; Unit radius vector
			QAXIS		; Q-axis (perpendicular to orbit plane)
		SL2	DMP		; Shift left 2, multiply
			RCO		; Radius of circular orbit
		STORE	Y		; Store crossrange distance
		RVQ			; Return via Q register

		BANK	30		; Switch to bank 30
		SETLOC	ASENT		; Set location counter
		BANK			; Bank directive
# Page 855

; ============================================================================
; ASCENT GUIDANCE CONSTANTS
;
; Time constants, scaling factors, and display codes used throughout
; ascent guidance computations. Time values scaled for AGC fixed-point
; arithmetic (B-17, B-18, B-9 indicate binary scaling factors).
; ============================================================================

100CS		EQUALS	2SEC(18)	; 100 centiseconds scaled B-18
T2A		EQUALS	2SEC(17)	; 2 seconds scaled B-17
4SEC(17)	2DEC	400 B-17	; 4 seconds scaled B-17
2SEC(17)	2DEC	200 B-17	; 2 seconds scaled B-17
T3		2DEC	1000 B-17	; 10 seconds scaled B-17
6SEC(18)	2DEC	600 B-18	; 6 seconds scaled B-18
BIT4H		OCT	10		; Bit 4 high (octal 10 = binary 00001000)
2SEC(9)		2DEC	200 B-9		; 2 seconds scaled B-9

; DSKY display verb/noun codes:
V06N63*		VN	0663		; Verb 06 Noun 63 (display decimal)
V06N76		VN	0676		; Verb 06 Noun 76 (display decimal)
V06N33A		VN	0633		; Verb 06 Noun 33 (display decimal)

		BANK	33		; Switch to bank 33
		SETLOC	ASENT6		; Set location counter
		BANK			; Bank directive
		COUNT*	$$/ASENT	; Instruction counter

; Additional constants for ascent guidance:
KT1		2DEC	0.5000		; Gain constant 0.5
PRLIMIT		2DEC	-.0639		# (B/TBUP)MIN = -.1 ft/sec(-3) pitch rate limit
MINABDV		2DEC	.0356 B-5	# 10 PERCENT BIGGER THAN GRAVITY (abort criterion)
1/DV0		=	MASS1		; Reciprocal delta-V alias for mass constant

# Page 856

; ============================================================================
; THE LOGARITHM SUBROUTINE - NATURAL LOGARITHM COMPUTATION
;
; Computes -LOG(X) using polynomial approximation and normalization.
; Used in ascent guidance for velocity-to-be-gained calculations involving
; exponential trajectory equations (Tsiolkovsky rocket equation).
;
; INPUT:  X in MPAC (must be positive)
; OUTPUT: -LOG(X) in MPAC (negative natural logarithm)
;
; Algorithm:
; 1. Normalize input X to range near 1.0
; 2. Compute polynomial approximation of log(normalized value)
; 3. Add correction term for normalization: log2/32 * exponent
; 4. Return negative of result (note: subroutine returns -LOG, not +LOG)
;
; The polynomial uses 8 coefficients for accuracy across input range.
; ============================================================================

		BANK	24		; Switch to bank 24
		SETLOC	FLOGSUB		; Set location to FLOGSUB area
		BANK			; Bank directive

# INPUT ..... X IN MPAC
# OUTPUT ..... -LOG(X) IN MPAC

LOGSUB		NORM	BDSU		; Normalize X, store exponent in MPAC+6
			MPAC +6		; Exponent storage location
			NEARONE		; Constant near 1.0 for normalization
		EXIT			; Exit interpretive mode
		
		; Evaluate 6th-degree polynomial approximation:
		TC	POLY		; Call polynomial evaluation routine
		DEC	6		; Polynomial degree (6 = seven coefficients)
		
		; Polynomial coefficients (optimized for log approximation):
		2DEC	.0000000060	; Coefficient 0 (constant term)
		2DEC	-.0312514377	; Coefficient 1
		2DEC	-.0155686771	; Coefficient 2
		2DEC	-.0112502068	; Coefficient 3
		2DEC	-.0018545108	; Coefficient 4
		2DEC	-.0286607906	; Coefficient 5
		2DEC	.0385598563	; Coefficient 6
		2DEC	-.0419361902	; Coefficient 7

		; Add correction for normalization exponent:
		CAF	ZERO		; Clear accumulator
		TS	MPAC +2		; Clear MPAC+2
		EXTEND			; Extended instruction follows
		DCA	CLOG2/32	; Load log(2)/32 constant (double precision)
		DXCH	MPAC		; Exchange with MPAC
		DXCH	BUF +1		; Save to buffer
		CA	MPAC +6		; Load normalization exponent
		TC	SHORTMP		; Short multiply (exponent * log2/32)
		DXCH	MPAC +1		; Load result
		DXCH	MPAC		; Move to MPAC
		DXCH	BUF +1		; Restore buffer
		DAS	MPAC		; Double-precision add to MPAC
		
		; Return negative of logarithm:
		TC	INTPRET		; Enter interpretive mode
		DCOMP	RVQ		; Double complement (negate) and return

; Constant: Natural log of 2 divided by 32 (for normalization correction)
CLOG2/32	2DEC	.0216608494	; ln(2)/32 = 0.693147.../32 ≈ 0.0216608494

