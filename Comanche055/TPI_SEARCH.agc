# Copyright:	Public domain.
# Filename:	TPI_SEARCH.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	551-561
# Mod history:	2009-05-15 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-20 RSB	Corrections:  On p. 551, "SETLOC P17S" -> P17S1.
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
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  10:28 APR. 1, 1969
#
#	This AGC program shall also be referred to as
#			Colossus 2A

# Page 551
# TPI SEARCH

; ============================================================================
; FILE: TPI_SEARCH.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: rendezvous
;
; TL;DR: Terminal Phase Initiation search algorithms computing optimal timing
;        for final rendezvous approach maneuver. Calculates relative phase angles
;        between Command Module and Lunar Module to determine when LM should
;        initiate terminal approach for docking. Used during Apollo 11 rendezvous
;        in lunar orbit after Eagle's ascent from surface.
;
; COMMENT-ONLY READERS: This program calculated when the Lunar Module should
;        begin its final approach to dock with the Command Module.
; CODE-ALONG READERS: Study relative orbital mechanics, phase angle computation,
;        and Lambert targeting for rendezvous optimization.
; ============================================================================

# PROGRAM DESCRIPTION S17.1 AND S17.2

# FUNCTIONAL DESCRIPTION
;
; RENDEZVOUS CONTEXT:
; After the Lunar Module ascends from the lunar surface, it must rendezvous and
; dock with the Command Module orbiting overhead. The Terminal Phase Initiation (TPI)
; maneuver begins the final approach phase. This routine finds the optimal TPI timing
; that minimizes fuel consumption while maintaining safe orbital altitudes.
;
; During Apollo 11's historic rendezvous on July 21, 1969, after Eagle's ascent,
; this algorithm calculated when Eagle should fire its thrusters to begin the
; final approach toward Columbia in lunar orbit.

# 	THE TPI SEARCH ROUTINE DETERMINES THE MINIMUM TOTAL VELOCITY TRANSFER TRAJECTORY FROM A GIVEN TPI
# MANEUVER TIME WITHIN THE CONSTRAINT OF A SAFE PERICENTER.  THIS VELOCITY IS THE SUM OF THE IMPULSIVE VELOCITIES
# FOR THE TPI AND TPF MANEUVERS.
# 	THE S17.1 ROUTINE EXTRAPOLATES THE STATE VECTORS OF BOTH VEHICLES TO THE TPI TIME AND COMPUTES THE
# RELATIVE PHASE ANGLE BETWEEN THE VEHICLES, THE ALTITUDE DIFFERENCE(I.E. THE MAGNITUDE DIFFERENCE OF THE
# POSITION VECTORS) AND SELECTS A SEARCH SECTOR BASED ON THE SIGN OF THE ALTITUDE DIFFERENCE.
;
; S17.1 PHASE - INITIAL GEOMETRY ANALYSIS:
; This first phase projects both spacecraft positions forward to the proposed TPI time.
; It then computes how far apart they are angularly around the Moon (phase angle)
; and which vehicle is higher. The altitude difference determines whether the LM
; needs to "catch up from below" or "descend from above" - this selects which
; search sector to use for finding the optimal trajectory.
# 	THE S17.2 ROUTINE FURTHER DEFINES THE SEARCH SECTOR BY COMPUTING ANGULAR LIMITS AND USES THE TIME THETA
# SUBROUTINE TO COMPUTE THE SEARCH START AND END TIMES.  THE SEARCH IS THEN MADE IN AN ITERATIVE LOOP USING THE
# LAMBERT SUBROUTINE TO COMPUTE THE VELOCITIES REQUIRED AT TPI TIME AND AT TPF TIME.  EXIT FROM THE SEARCH LOOP
# IS MADE WHEN SOLUTION CRITERIA ARE MET (NORMAL EXIT) OR AS SOON AS IT IS EVIDENT THAT NO SOLUTION EXISTS IN
# THE SECTOR SEARCHED.
;
; S17.2 PHASE - ITERATIVE TRAJECTORY SEARCH:
; This phase systematically searches through possible Terminal Phase Final (TPF)
; times - when the LM would arrive at the CM's position. For each candidate TPF time,
; the Lambert subroutine (from CONIC_SUBROUTINES.agc) solves the two-point boundary
; value problem: what velocities are needed at TPI and TPF to connect the positions?
;
; The search finds the solution requiring minimum total velocity change (delta-V)
; while ensuring the transfer trajectory never dips below safe pericenter altitude.
; If the LM's path would graze the lunar surface, that solution is rejected.

# CALLING SEQUENCE

# 	BOTH ROUTINES ARE CALLED IN INTERPRETIVE CODE AND RETURN VIA QPRET.  S17.1 HAS ONLY A NORMAL EXIT.
# S17.2 RETURNS VIA QPRET FOR NORMAL EXIT AND TO ALARUMS FOR ERROR EXIT.
# SUBROUTINES CALLED
#
#	CSMCONIC
#	LEMCONIC
#	TIMETHET
#	INITVEL

		BANK	36
		SETLOC	P17S1
		BANK

		COUNT	36/TPI

		EBANK=	RACT3

# 	**** TEMPORARY ****
;
; SAFETY CONSTRAINTS AND SEARCH PARAMETERS:
; These constants define minimum safe altitudes and search timing parameters.
; The pericenter (lowest point) of any transfer trajectory must stay above these
; minimums to avoid collision with the celestial body's surface.

HPE		2DEC	157420.0 B-29		# EARTH'S MIN. PERICENTER ALTITUDE 85 N.M.
						; Earth safety margin: 85 nautical miles
HPL		2DEC	10668.0213 B-29		# MOON:S MIN. PERICENTER ALTITUDE 35000FT
						; Lunar safety margin: 35,000 feet (about 10.7 km)
						; Critical for Apollo 11 rendezvous trajectories

CDSEC		2DEC	40000			; Close distance sector: 40,000 feet
						; Search range when vehicles are nearby
CLSEC		2DEC	15000			; Close sector: 15,000 feet
						; Tighter proximity threshold
PIINVERS	2DEC	.3183098862		; 1/π = 0.3183098862
						; Used for angular calculations and conversions
SEC1THET	2DEC	.1944444444		; Sector 1 theta: 0.1944444444 (about 70 degrees)
						; Angular limit for first search sector

# Page 552
SEC2THET	2DEC	.9166666667		; Sector 2 theta: 0.9166666667 (about 330 degrees)
						; Angular limit for second search sector
MANYFEET	2DEC	-1.0 B-2		; Altitude scale factor: -1.0 * 2^-2 = -0.25
						; Used for altitude difference calculations
LIMVEL		2DEC	.6096 E-2 B-7		# 2FPS
						; Velocity limit: 2 feet per second
						; Minimum delta-V threshold for solution acceptance
DFTMOON		2DEC	.1524 E3 B-29		# 500 FEET
						; 500 feet above lunar surface
						; Additional safety margin for Moon proximity
DP-.002		2DEC	0.002			; Small epsilon value 0.002
						; Used for numerical convergence testing

		SETLOC	P17S
		BANK
;
; ============================================================================
; TRANSITION: From constant definitions to S17.1 routine
;
; The TPI search now begins. After Eagle's ascent from the lunar surface,
; Columbia (the Command Module) continues orbiting while Eagle maneuvers to
; rendezvous. This routine calculates the precise moment when Eagle should
; initiate its final approach - the Terminal Phase Initiation (TPI).
;
; The first step: project both vehicles' positions forward to the proposed
; TPI time and analyze their relative geometry (phase angle and altitude).
; ============================================================================
;
S17.1		STQ	DLOAD			; S17.1: Initial geometry analysis
			NORMEX		; Save return address for normal exit
			TTPI		; Load proposed TPI time
		STCALL	TDEC1			; Store as extrapolation target time
			LEMCONIC	; Advance PASSIVE vehicle (LM) to TPI time
						; Uses conic orbital propagation
		CALL
			LEMSTORE	; Store LM state vector (position/velocity)
		DLOAD
			TTPI		; Load TPI time again
		STCALL	TDEC1			# ADVANCE ACTIVE VEHICLE TO TPI
			CSMCONIC	; Now propagate CSM (active vehicle) to TPI time
					; At this point, both vehicles are at TPI time
		CALL
			CSMSTORE	; Store CSM state vector
;
; GEOMETRY CALCULATION:
; With both vehicles at TPI time, now compute relative geometry:
; 1. Calculate altitude difference (which vehicle is higher)
; 2. Compute phase angle (how far apart angularly around Moon)
; These determine which search sector to use for finding optimal trajectory.
;
		VLOAD
			RACT3		; Load active (CSM) position vector
		ABVAL	PDVL			# /RA/ 0D			PL 2D
					; Get magnitude (radius from Moon center)
			RPASS3		; Load passive (LM) position vector
		UNIT	PDDL			# UNIT RP 0D			PL 6D
					; Normalize to unit vector
		BDSU	SET			; Subtract magnitudes: /RP/ - /RA/
			36D			# /RP/ -/RA/
			KFLAG			# OFF = +
					; KFLAG indicates altitude relationship
		BMN	CLEAR			; Branch if magnitude negative (RP < RA)
			+2		; Skip KFLAG clear if positive
			KFLAG			# ON = -
					; KFLAG now indicates which vehicle higher:
					; KFLAG clear = passive (LM) above active (CSM)
					; KFLAG set = passive (LM) below active (CSM)
		STOVL	DELHITE		; Store altitude difference in DELHITE
			0D		; Reload unit passive position from pushlist
		VXV	UNIT			; Cross product: unit RP × velocity RP
			VPASS3		; Forms normal to passive vehicle orbit plane
		STOVL	E2			# ALMOST IT SAVE FOR 17.2
					; Store normal vector E2 (needed for S17.2)
			RACT3		; Reload active vehicle position
		PUSH	VPROJ			; Save active position, then project it
			E2		; Project onto plane normal to E2
					; This removes out-of-plane component
		VSL2	BVSU			# RPA
					; Scale and subtract from original
					; Creates in-plane projection of active position
		UNIT	DOT			; Normalize and dot with unit passive
			0D		; Dot product with unit passive position
					; Result = cos(in-plane angle between vehicles)
		SL1 	ACOS			; Scale and compute arccos
					; Gives in-plane central angle
		PDVL				; Push angle to stack, reload for sign check
		VXV	DOT			; Cross product for determining quadrant
# Page 553
			RACT3		; Cross active position with...
			E2		; ...orbit normal E2
					; Triple product determines quadrant
		PDDL	SIGN			; Push result, load angle, apply sign
		STADR				; Magnitude from pushlist 
		STODL	THETZERO		# CENTRAL ANGLE
					; Store signed central phase angle THETZERO
					; Positive = active vehicle ahead in orbit
					; Negative = active vehicle behind
					; This angle determines rendezvous geometry
			X1		; Restore index register
		STCALL	XRS			# SAVE INDICES FOR FURTHER USE
			NORMEX			#   += ACTIVE AHEAD  -= ACTIVE BEHIND
;
; S17.1 COMPLETE:
; Phase angle THETZERO computed, altitude difference in DELHITE stored.
; Search sector selected via KFLAG based on altitude relationship.
; Normal return to calling program via NORMEX.
;
; ============================================================================
; TRANSITION: From S17.1 geometry calculation to S17.2 search execution
;
; S17.1 has determined the relative positions and phase angle between the
; two vehicles at TPI time. Now S17.2 takes over to actually search for the
; optimal trajectory. It computes angular search limits based on geometry,
; then iterates through possible transfer times, using Lambert targeting
; to evaluate each candidate. The goal: find minimum total ΔV for TPI+TPF
; while maintaining safe pericenter altitude throughout the transfer.
; ============================================================================
;
; S17.2 - TPI SEARCH ITERATION ROUTINE
;
; This routine performs the iterative search for the optimal TPI trajectory.
; For Apollo 11 rendezvous (July 21, 1969), this calculated when Eagle should
; fire its engine to begin the final approach to Columbia after ascending from
; the lunar surface. The search balances fuel efficiency with safety constraints.
;
S17.2		STQ	VLOAD			# COMPUTE SEARCH SECTOR LIMITS
			QTEMP		; Save return address
			RACT3		; Load active vehicle position
		UNIT	DOT			; Normalize active position, dot with E2
			E2		; Measures orbit plane alignment
		ABS	SQRT			; Take square root of absolute value
		SL1	DAD			; Scale result (×2) and add margin
			DP-.002			# ADD .002 RADIANS TO IT
					; Safety margin: 0.002 radians ≈ 0.11°
					; Ensures search doesn't hit singularities
		BON	DCOMP			# GIVES CORRECT SINE, COSINE MUST BE
			KFLAG			# COMP. ADD .5 FOR ANGLE
			+1		; Branch based on altitude relationship
					; Complement angle if passive below active
# PHI(0)=180-(-(THETAZERO +K5IT)), PHI(I)=180-(-THETAZERO+K2IT))
# SIN(180-ALPHA)=SIN(ALPHA) ETC
;
; COMPUTE INITIAL AND FINAL SEARCH ANGLES:
; PHI(0) = start angle of search arc
; PHI(I) = end angle of search arc
; Using transformation: φ = 180° - (-θ + K·IT)
; where K depends on search sector (K2 or K5 coefficient)
;
		DMP	SETPD			; Multiply by 1/π for cycle conversion
			PIINVERS		# REVOLUTIONARY HERES TWO IT
			0D		; Set pushdown list pointer to 0D
		PUSH	DSU			; Push result, then subtract
			THETZERO	; Central phase angle θ₀
		STORE	IT			# PHI(I) , -(THETZERO + K2IT)
					; Store as IT: intermediate angle value
					; PHI(I) represents end of search arc
		PDDL	PUSH			; Push to list, load, push again
		SR1	DAD			; Shift right 1 (÷2), then add
		DAD	PUSH			# PHI(0) , -(THETZERO + K5IT)
					; Compute start angle PHI(0)
					; Different coefficient K5 defines sector start
		SIN	SET			; Compute sine of start angle
			RVSW		; Set reverse flag for iteration direction
		STODL	SNTH			; Store sin(θ) for transfer calculation
		COS	BMN			; Compute cosine of start angle
			+2		; Branch if minus (negative cosine)
		DCOMP				; Double complement if needed
		STODL	CSTH			; Store cos(θ) for transfer calculation
			XRS		; Load scale index XRS
		STOVL	X1			; Store in X1 for vector scaling
			RPASS3		; Load passive vehicle position vector
		VSR*				; Variable shift right by index
			0,2		; Shift amount from X1 (proper scaling)
		STOVL	RVEC			; Store scaled position as RVEC
					; RVEC = passive vehicle position at start angle
			VPASS3		; Load passive vehicle velocity vector
		VSR*				; Variable shift right by same index
			0,2		; Maintain consistent scaling between R and V
		STCALL	VVEC			; Store scaled velocity as VVEC
			TIMETHET	; Call time-theta subroutine
;
; TIMETHET SUBROUTINE:
; Computes the time required to reach a specific true anomaly (angle)
; in the orbit, given position, velocity, and target angle.
; Returns: T = time to reach PHI(0) from current state
;
		DLOAD				# SAVE START TIME AND GET END TIME
			T		; Load computed time from TIMETHET
# Page 554
		STORE	TF			; Store as final search time
		STODL	TFO			; Store as TFO (search end boundary)
			IT		; Reload IT (end angle PHI(I))
;
; COMPUTE END TIME OF SEARCH:
; Now calculate the time to reach PHI(I), the end of the search arc.
; This defines the time window within which to search for optimal TPI.
;
		PUSH	SIN			; Push IT, then compute sine
		STODL	SNTH			; Store sin(θ) for end angle
		COS	BMN			; Compute cosine of end angle
			+2		; Branch if negative
		DCOMP				; Complement if needed for proper quadrant
		STORE	CSTH			; Store cos(θ) for end angle
		LXA,1	CALL			; Load X1 with scale index
			XRS		; XRS contains vector scaling factor
			TIMETHET	; Call time-theta for end angle
					; Returns T = time to reach PHI(I)
# INITIALIZE LOOP
;
; SEARCH LOOP INITIALIZATION:
; Set up iteration to find minimum ΔV trajectory within search window.
; The loop will step through angles from TFO to TFI, computing Lambert
; solutions and comparing total ΔV (TPI burn + TPF burn) to find optimum.
;
		DLOAD	CLEAR			; Load end time T
			T		; Time to reach PHI(I) from TIMETHET
			ITSWICH		; Clear iteration switch flag
					; ITSWICH tracks search loop state
		STODL	TFI			# SAVE TIME FOR LOOP TEST
					; TFI = final iteration boundary time
			DPPOSMAX	; Load maximum positive double precision
		STODL	DELVEE			; Initialize DELVEE to maximum value
					; DELVEE tracks best ΔV found so far
					; Start with worst case, improve iteratively
			MANYFEET	; Load altitude constant (-1.0 scaled)
		STODL	HP			; Initialize HP (pericenter altitude)
					; HP tracks minimum altitude of trajectory
			SEC1THET	# 70 DEGREES
					; Sector 1 angular limit (70° = 0.1944 rev)
		BON	DLOAD			; Branch based on KFLAG
			KFLAG		; Test altitude relationship flag
			+2		; Skip if flag ON (Sector 2)
			SEC2THET	# 330 DEGREES
					; Sector 2 angular limit (330° = 0.9167 rev)
					; Different sector = different search window
		STCALL	THETL			; Store angular limit as THETL
			CONCAUL		; Call conic trajectory computation
;
; CONCAUL SUBROUTINE:
; Extrapolates vehicle state vectors to specific time using conic equations.
; Prepares for Lambert trajectory calculation between two orbital states.
;
BIS		DLOAD	SR1		; Back from CONCAUL, prepare for angle calc
			CSTH		; Load cosine of transfer angle
		STODL	COSTH			; Store cosine(θ/2) scaled properly
			SNTH		; Load sine of transfer angle
		SR1				; Shift right 1 (divide by 2)
		STCALL	SINTH			# GET 4 QUADRANT THETA
					; Store sine(θ/2) for Lambert calculation
			ARCTRIG		; Call arctangent to get full angle
;
; COMPUTE TRANSFER ANGLE AND CHECK SEARCH BOUNDS:
; Transfer angle θ defines the arc traveled from TPI to TPF.
; Must verify this angle is within the valid search sector.
;
		BPL	DAD			; Branch if plus (0 to 180°)
			+2		; Skip adjustment if already positive
			DPPOSMAX	# PUT THETA BETWEEN 0,1
					; Add full cycle to make positive (0-360°)
		BDSU	PDDL			; Subtract angular limit, push result
			THETL		; THETL is sector boundary angle
					; Result: θ - THETL (deviation from limit)
			TF		; Load current iteration time
		DSU	SIGN			# FAST TIMES
					; Subtract final iteration boundary
			TFI		; TFI = search end time
					; Tests if search window exhausted
		BMN				; Branch if minus (still in search window)
			RNGETEST	# TIME MUST HAVE A STOP
					; Continue to range test if more time remains
;
; If we reach here, search window is exhausted. Continue to test current
; solution before potentially exiting or refining search parameters.
;
# ADVANCE PASSIVE FOR TARGET VECTOR
;
; CONCAUL ROUTINE:
; Extrapolates passive vehicle state vector forward in time to the
; current search iteration point. This gives the target position/velocity
; for the Lambert trajectory solution.
;
CONCAUL		DLOAD				; Load TPI time (Terminal Phase Init)
			TTPI		; Base reference time for maneuver
		DAD	BON			; Add current iteration offset time
			TF		; TF = time offset within search window
# Page 555
			AVFLAG		; Test which vehicle is active
			ADVCSM		; Branch if CSM is active vehicle
;
; LM IS ACTIVE, CSM IS PASSIVE:
; Advance CSM (Command Module) state vector to target time.
;
		STCALL	TDEC1			; Store target time as TDEC1
			LEMCONIC	; Call LM conic propagator
					; Propagates CSM using Keplerian orbit
		GOTO				; Continue to junction point
			JUNCT3		; Common path after state propagation
;
; CSM IS ACTIVE, LM IS PASSIVE:
; Advance LM (Lunar Module) state vector to target time.
;
ADVCSM		STCALL	TDEC1			; Store target time as TDEC1
			CSMCONIC	; Call CSM conic propagator
					; Propagates LM using Keplerian orbit
# SAVE BACK VALUES OF HP AND DELVEE
;
; JUNCT3 - COMMON STATE VECTOR STORAGE:
; After propagating the passive vehicle forward, save current state vectors
; and orbital parameters as backup values. These are retained to enable
; interpolation if the search overshoots the optimal solution.
;
JUNCT3		VLOAD				; Load passive vehicle velocity
			VATT		; Current passive velocity vector
		STOVL	VPASS4			; Save as VPASS4 for Lambert target
			RATT		; Load passive position vector
		STORE	RPASS4			; Save as RPASS4 for Lambert target
		STODL	RTARG			; Also store as general target position
			TF		; Load current time offset from TPI
		STODL	DELLT4			; Save as delta-time parameter
			HP		; Load current pericenter altitude
		STODL	HPO			; Save as previous pericenter (HPO)
			DELVEE		; Load current total delta-V magnitude
		STODL	DELVEO			; Save as previous delta-V (DELVEO)
# PREPARE FOR LAMBERT
;
; LAMBERT SETUP:
; Prepare initial conditions for the INITVEL Lambert targeting subroutine.
; The Lambert problem solves for required velocities at TPI and TPF (Terminal
; Phase Final) given two positions and a transfer time.
;
			TTPI		; Load TPI time
		STODL	INTIME			; Store as initial time for Lambert
			XRS		; Load body selection flag
		STODL	RTX1			; Store body parameter (Earth/Moon)
			HI6ZEROS	; Load precision tolerance
		SETPD	PDDL			; Initialize push-down stack pointer
			0D		; Set to stack base (0D)
			EPSFOUR		; Load convergence epsilon (tolerance)
		PDVL				; Push epsilon onto stack
			RACT3		; Load active vehicle position vector
		STOVL	RINIT			; Store as initial position for Lambert
			VACT3		; Load active vehicle velocity
		STCALL	VINIT			; Store as initial velocity for Lambert
			INITVEL		; Call Lambert targeting subroutine
;				; Returns VTPRIME (TPI velocity required)
;				; and VIPRIME (TPF velocity required)
# COMPUTE H ET CETERA
;
; DELTA-V COMPUTATION:
; Calculate the velocity changes required at TPI and TPF. The total delta-V
; is the sum of these two impulsive maneuvers. This is the cost function
; that the search algorithm minimizes.
;
		VLOAD	VSU			; Load Lambert solution TPI velocity
			VTPRIME		; Required velocity at TPI
			VPASS4		; Minus actual passive vehicle velocity
		ABVAL	PUSH			; Compute magnitude, push on stack
		STOVL	RELDELV			; Save as relative delta-V |V2-VP|
			DELVEET3	; Load active vehicle delta-V (V1-VA)
		ABVAL				; Compute magnitude |V1-VA|
		STORE	MAGVTPI			; Save as TPI burn magnitude
		DAD	STADR			; Add the two delta-V magnitudes
		STODL	DELVEE			; Store total delta-V for this iteration
			XRS		; Load body parameter flag
		STOVL	X1			; Store for index calculations
			VIPRIME		; Load Lambert TPF velocity solution
# Page 556
;
; PERICENTER CALCULATION:
; Call the PERIAPO subroutine to compute orbital elements from the transfer
; trajectory state vectors, yielding the pericenter altitude HP.
;
		VSR*				; Shift velocity vector right by X2
			0,2		; Dynamic shift count
		STOVL	VVEC			; Store as velocity input vector
			RACT3		; Load active vehicle position
		VSR*				; Shift position vector right by X2
			0,2		; Dynamic shift count
		STCALL	RVEC			; Store as position input vector
			PERIAPO		; Call pericenter/apocenter calculator
;				; Returns HP (pericenter altitude)
		LXA,2	DLOAD			; Load index from X2, load result
			XRS	+1	; Body parameter (Earth/Moon index)
		SL*				; Shift left by indexed amount
			0,2		; Scale for proper units
		STORE	HP			; Save pericenter altitude
# ITSWICH DENOTES INTERPOLATION--SOLUTION ACCEPTANCE IS FORCED
;
; PERICENTER SAFETY CHECKS:
; Verify that the transfer orbit maintains safe altitude above the body.
; Mission rules required minimum clearance (35,000 feet above Moon, 85 NM
; above Earth) to prevent collision with terrain or mountains.
;
; If ITSWICH flag is set, skip safety checks and accept solution (forced
; by interpolation logic indicating we're refining a previously-found answer).
;
		BON	DLOAD			; Branch on ITSWICH flag
			ITSWICH		; Interpolation switch
			ENDEN		; Jump to acceptance (skip checks)
			HPERMIN		; Load minimum safe pericenter
		DSU	BMN			; Subtract computed HP from minimum
			HP		; Current pericenter altitude
			HALFSAFE	; Branch if HP < minimum (unsafe!)
;
; PERICENTER IS SAFE - CHECK IF IMPROVING:
;
		PDDL	DSU			# WAS PERICENTER ALT SAFE
			HPERMIN		; Load minimum safe altitude
			HPO		; Subtract previous pericenter
		BMN	DSU			# (HPLIM-HPO)-(HPLIM-HP)=HP-HPO
			INTERP		# SOLUTION AT HAND
;				; Previous HP was unsafe, jump to interp
		BMN	DLOAD			; If HP-HPO < 0 (getting worse!)
			ALARUMS		# IT'S GETTING WORSE - SOUND THE ALARM
;				; No safe solution exists in sector
			CDSEC		; Load coarse delta search increment
;
; JUNCT1 - SEARCH STEP SIZE CONTROL:
; Determines the direction and magnitude of the next search iteration step.
; KFLAG controls search direction: OFF = positive (forward), ON = negative.
; This implements the iterative search through the transfer angle sector.
;
JUNCT1		BOFF	DCOMP			# OFF IS PLUS ON IS MINUS
			KFLAG		; Test search direction flag
			+1		; Skip complement if flag OFF
		STORE	DELTEE			; Save time step increment
;
; JUNCT2 - ADVANCE SEARCH TIME:
; Add the time increment DELTEE to the current search point TF, then
; recycle back to the beginning of the search loop to test the next point.
;
JUNCT2		DLOAD	DAD			; Load time increment
			DELTEE		; Delta-time for this iteration
			TF		; Add to current search time offset
		STCALL	TF			; Update TF to next search point
			BIS		# RECYCLE
;				; Return to beginning of search loop
;
; INTERP - INTERPOLATION REFINEMENT:
; When we've bracketed the minimum delta-V point, this routine performs
; interpolation to find the precise time at which the minimum occurs.
; Uses the difference in pericenter altitude (HP-HPO) to compute a refined
; time step that moves toward the optimal solution.
;
INTERP		SET	DSU			# HP-HPO
			ITSWICH		; Set interpolation switch flag
;				; Indicates we're in refinement mode
		NORM	PDDL			; Normalize (HP - HPO)
			X1		; Store exponent in X1
			DFTMOON		; Load Moon distance threshold (500 ft)
		DAD	DSU			; Add minimum pericenter altitude
			HPERMIN		; Subtract current pericenter HP
			HP		; Computes adjusted pericenter difference
		NORM	SR1			; Normalize result
			X2		; Store exponent in X2
		XSU,2	DDV			; Index subtract, then divide
			X1		; Compute ratio of normalized values
# Page 557
		DMP	SR*			; Multiply by time step
			DELTEE		; Current delta-time increment
			0 	-1,2	; Shift right by (X2-1)
		STCALL	DELTEE			; Store refined time increment
			JUNCT2		; Return to search loop with new step
;
; HALFSAFE - SAFE PERICENTER CONVERGENCE CHECK:
; Tests whether the search is converging toward a safe solution. Checks
; if the delta-V change between iterations is small enough to indicate
; convergence, and evaluates the pericenter altitude trend to determine
; if we should continue searching, interpolate, or terminate successfully.
;
HALFSAFE	PDDL	DSU			# SAVE HP-HPLIM FOR POSSIBLE
			DELVEE		; Push current delta-V onto stack
			DELVEO		# SAVE THIS TOO
;				; Compute DELVEE - DELVEO
		PUSH	ABS			; Push difference, then absolute value
		DSU	BMN			; Subtract velocity convergence limit
			LIMVEL		; Limit is 2 ft/sec (0.6096 m/s)
			ENDEN		; Branch if converged (success exit)
;				; Delta-V change is small enough
		DLOAD	DSU			; Load minimum pericenter altitude
			HPERMIN		; Subtract previous pericenter
			HPO		; Computes HPERMIN - HPO
		PDDL				; Push result onto stack
		BMN	DLOAD			; Branch if getting worse
			LRGRDVO		; Continue large reduction search
		BPL	DLOAD			; Branch if improving
			INTERP		; Interpolate to find minimum
			DELTEE		; Load current time step
		SR1	DCOMP			; Shift right 1, then complement
;				; Reverses and halves the step size
		STCALL	DELTEE			; Store adjusted step
			JUNCT2		; Return to search loop
;
; LRGRDVO - LARGE REDUCTION VELOCITY TEST:
; Reached when pericenter is getting worse (HPERMIN - HPO is negative).
; This handles the case where we're making large changes in the search
; but haven't found a safe solution yet. Tests the loaded value and either
; continues the search with a different step size or transitions back to
; the main search loop.
;
LRGRDVO		DLOAD			; Load value from stack or accumulator
		BMN	DLOAD		; If minus, branch; else load CLSEC
			JUNCT2		; Return to search loop (negative case)
			CLSEC		; Load close sector time constant (15000)
		GOTO			; Unconditional branch
			JUNCT1		; Go to JUNCT1 to set new step size
;
; ============================================================================
; RNGETEST - SEARCH TIME EXPIRATION CHECK:
; When the search loop exhausts its time range, this routine determines
; if a valid solution was found. Tests whether the saved pericenter (HP)
; is above the minimum safe pericenter (HPERMIN). If safe pericenter was
; achieved, accepts the solution and stores the time. If no safe solution
; found, branches to ALARUMS for error handling.
; ============================================================================
;
# TIME RAN OUT ASSUME SOLUTION IF SAVE PERICENTER
RNGETEST	DLOAD	DSU		; Load HP (saved pericenter altitude)
			HP		; Subtract minimum safe pericenter
			HPERMIN		; Result: HP - HPERMIN
		BMN	DLOAD		; If negative (unsafe), branch to alarm
			ALARUMS		; No safe solution found - abort
			TF		; Load TF (final search time)
		DSU			; Subtract step size
			DELTEE		; Result: TF - DELTEE
		STORE	TF			; Store adjusted time as solution time
;
; ============================================================================
; ENDEN - SEARCH COMPLETION AND FINAL PARAMETER COMPUTATION:
; After finding a valid TPI solution, computes final geometric parameters
; including the transfer angle (omega-t) and determines the sign relationships
; between velocity vectors. This data is required for proper trajectory
; setup in subsequent rendezvous programs.
; ============================================================================
;
ENDEN		VLOAD			; Load VTPRIME (velocity at TPF)
			VTPRIME		; Required velocity vector at TPF time
		DOT	PDDL			# Compute SG2 (dot product magnitude)
			RPASS4		; Dot with position at TPF
			RELDELV		; Push result, load relative delta-V
		SIGN	STADR			# Transfer sign: SIGN(RELDELV)=SIGN(SG2)
		STCALL	RELDELV		; Store signed relative delta-V
			TRANSANG		; Call TRANSANG to compute central angle
;
; Sign determination for transfer geometry:
; Computes SG1 (dot product of position at TPI with velocity at TPI) and
; determines sign relationships between SG1 and SG2. This determines the
; geometry of the transfer trajectory and affects the NN1 parameter used
; in later computations. The logic handles both prograde and retrograde
; transfer cases.
;
		VLOAD	DOT		; Load RACT3 (position at TPI)
			RACT3		; Position vector at TPI time
# Page 558
			VIPRIME		; Dot with VIPRIME (velocity at TPI) = SG1
		SIGN	BPL		; Transfer sign from RELDELV, test
			RELDELV		; If positive: SG1 = SG2 (same sign)
			USEKAY		; Branch if SG1 has same sign as SG2
;
; Case: SG1 and SG2 have opposite signs (SG2 - SG1 has sign of SG2):
; Load DECTWO (value 2), complement it to get -2, apply sign from RELDELV.
; This computes the NN1 parameter for the opposite-sign case.
;
		SLOAD	DCOMP		; Load DECTWO (2), complement to -2
			DECTWO		; Result: -2
		SIGN	BPL		; Apply sign from RELDELV
			RELDELV		; Transfer sign of relative delta-V
			NEXUS		; If positive, go to NEXUS
		DCOMP	GOTO		; Complement again (make positive)
			USEKAY	+4	; Skip to computation continuation
;
; USEKAY - Same-sign case processing:
; When SG1 and SG2 have the same sign, process based on KFLAG setting.
; KFLAG indicates K-parameter selection for the Lambert solution.
;
USEKAY		SLOAD	BON		; Load DECTWO (value 2)
			DECTWO		; Standard value
			KFLAG		; Test K-flag setting
			NEXUS		; If K-flag set, use value as-is
		DSU			; K-flag clear: subtract P21ONENN
			P21ONENN	; Adjust NN1 for K-flag clear case
NEXUS		STODL	NN1		; Store final NN1 parameter
			HP		; Load HP (pericenter altitude)
		STCALL	POSTTPI		; Store in POSTTPI variable
			QTEMP		; Return via saved address in QTEMP
;
; ============================================================================
; TRANSITION: From TPI search completion to orbital mechanics utilities
;
; The TPI search has determined the optimal maneuver times and delta-V
; requirements. The following TRANSANG subroutine provides the fundamental
; orbital mechanics calculation needed during the search: computing the
; central angle (angular displacement) of the passive vehicle during the
; transfer trajectory. This angle is critical for determining the geometric
; relationship between the vehicles at TPI and TPF.
; ============================================================================
;
		BANK	07
		SETLOC	XANG
		BANK
		COUNT	07/XANG

# CENTRAL ANGLE SUBROUTINE
# 	THIS SUBROUTINE COMPUTES THE CENTRAL ANGLE OF TRAVEL OF THE
# PASSIVE VEHICLE DURING THE TRANSFER.
;
; ============================================================================
; TRANSANG - CENTRAL ANGLE COMPUTATION SUBROUTINE:
;
; Computes the central angle (angular displacement) traveled by the passive
; vehicle during the transfer time from TPI to TPF. Uses classical orbital
; mechanics formulas based on the vis-viva equation and Kepler's third law.
;
; The computation follows these orbital mechanics steps:
; 1. Compute velocity squared: V² = VVEC dot VVEC
; 2. Compute specific orbital energy: ε = V²/2 - μ/R
; 3. Compute semi-major axis: a = -μ/(2ε) = R/(2 - RV²/μ)
; 4. Compute mean motion: n = √(μ/a³)
; 5. Compute central angle: θ = n × TF (converted to revolutions)
;
; This algorithm works for elliptical orbits (typical rendezvous case).
; The passive vehicle (Command Module) maintains its orbital trajectory
; while the active vehicle (Lunar Module after ascent) performs the
; transfer maneuvers.
;
; INPUTS:
;   VPASS4 - Passive vehicle velocity vector at transfer time
;   RPASS4 - Passive vehicle position vector at transfer time
;   TF     - Transfer time duration (TPI to TPF)
;   MUTABLE - Contains μ (gravitational parameter) and scaling indices
;   XRS    - Index registers for scaling control
;
; OUTPUTS:
;   CENTANG - Central angle traveled (in revolutions, B-0 scaling)
;
; RETURNS: Via SUBEXIT (saved return address)
; ============================================================================
;
TRANSANG	STQ	SETPD		; Save return address in SUBEXIT
			SUBEXIT		; Store Q register for return
			0		; Set push-down pointer to 0
;
; Index register setup for Earth/Moon scaling control:
; XRS contains scaling indices that differ for Earth vs Moon orbits.
; X1 controls μ (gravitational parameter) scaling
; X2 controls position/velocity vector scaling
;
		LXA,1	LXA,2		; Load index registers from XRS
			XRS		; X1 ← XRS (for μ scaling)
			XRS	+1	; X2 ← XRS+1 (for R/V scaling)
;
; Step 1: Load and scale passive vehicle velocity vector
; VPASS4 contains the passive vehicle's velocity at the transfer time.
; Vector scaling depends on whether orbiting Earth or Moon (via X2).
;
		VLOAD	VSR*		; Load velocity vector
			VPASS4		; Passive vehicle velocity
			0,2		; Scale right by amount in X2
		STODL*	VVEC		; Store scaled velocity in VVEC
			MUTABLE	+2,1	; Load √μ with scaling via X1
		PDVL	VSR*		; Push √μ to stack			00D
			RPASS4		; Load position vector
			0,2		; Scale right by amount in X2
;
; Step 2: Compute position magnitude |R|
; The magnitude is needed for the vis-viva equation: V² = μ(2/R - 1/a)
; Position scaling: +29 for Earth orbit, +27 for Moon orbit
;
		ABVAL	PDDL*		; |R| = magnitude of position		02D
			MUTABLE,1	; Load 1/μ with scaling via X1
		PDVL	VSQ		; Push 1/μ to stack			04D
			VVEC		; Load velocity vector
;
; Step 3: Compute V² (velocity squared)
; VSQ computes the dot product of velocity with itself.
; This is the kinetic energy term (per unit mass) in the vis-viva equation.
;
		NORM	DMPR		; Normalize V², multiply by value at 02D
			X1		; Store normalization shift in X1
# Page 559
;
; Step 4: Compute RV²/μ
; This forms the dimensionless parameter needed for the vis-viva equation.
; The result represents the ratio of kinetic energy to gravitational
; potential energy at the current orbital position.
;
		DMP	SRR*		; Multiply by value at 02D (|R|)
			02D		; |R| from position magnitude
			0 	-3,1	; Scale right with adjustment via X1
;
; Step 5: Compute (2 - RV²/μ)
; This is the denominator in the semi-major axis formula: a = R/(2 - RV²/μ)
; For an elliptical orbit, this value is positive (RV²/μ < 2)
; For a parabolic orbit, this would be zero (RV²/μ = 2)
; For a hyperbolic orbit, this would be negative (RV²/μ > 2)
;
		BDSU			; 2 - RV²/μ (subtract from constant)
			D1/32		; Constant 2.0 scaled appropriately
		NORM	PDDL		; Normalize and push to stack
			X1		; Store shift in X1	(2 - RV²/μ) (+6-N)
;
; Step 6: Compute semi-major axis a = R/(2 - RV²/μ)
; The semi-major axis is the fundamental orbital parameter that defines
; the size of the ellipse. It appears in Kepler's third law relating
; orbital period to the central body's gravitational parameter.
; Scaling: +30 for Earth, +28 for Moon (magnitude of R)
;
		SR1R	DDV		; Shift |R| right 1, divide by (2-RV²/μ)
		SL*	PUSH		; Scale left, push: a = R/(2-RV²/μ)	02D
			0 	-5,1	; Final scaling: +29 Earth, +27 Moon
;
; Step 7: Compute √a (square root of semi-major axis)
; This intermediate value is needed for computing a³/² in Kepler's formula
; for mean motion: n = √(μ/a³) = √μ / a³/²
;
		SR1	SQRT		; Shift right 1, compute square root
		DMP			; Multiply (computes a³/² = a × √a)
		NORM	PDDL		; Normalize a³/², push to stack		00D
			X1		; Store normalization shift
;
; Step 8: Compute mean motion n = √(μ/a³) = √μ / a³/²
; The mean motion is the angular velocity (radians per unit time) of an
; orbiting body assuming uniform circular motion with the same period as
; the actual elliptical orbit. This comes from Kepler's third law:
; T² = (4π²/μ) a³, which gives n = 2π/T = √(μ/a³)
;
		SR1	DDV		; Shift √μ right 1, divide by a³/²
;
; Step 9: Compute angular displacement θ = n × TF
; Multiply mean motion by transfer time to get the angle traveled.
; This gives the central angle in radians (before conversion to revolutions).
;
		DMP	SL*		; Multiply by TF (transfer time)
			TF		; Time from TPI to TPF
			0,1		; Scale left with adjustment
;
; Step 10: Convert to revolutions
; Divide by 2π to convert radians to revolutions (full circles).
; This is the natural unit for the TPI search algorithms since they work
; with phase angles and orbital geometry expressed in revolutions.
;
		PDDL	NORM		; Push result, normalize 2π constant
			2PISC		; Load 2π scaled constant
			X1		; Store normalization shift
		PDDL	DDV		; Push 2π, divide angle by 2π
		SL*			; Scale left with adjustment
			0 	-3,1	; Final scaling adjustment via X1
;
; RESULT: CENTANG contains the central angle in revolutions (B-0 scaling)
; This represents how far the passive vehicle travels around its orbit
; during the transfer time from TPI to TPF.
;
		STCALL	CENTANG		; Store final central angle
			SUBEXIT		; Return via saved address
		BANK	35
		SETLOC	P17S1
		BANK
		COUNT	35/P17

# TPI SEARCH DISPLAY ROUTNE

; ============================================================================
; TPI SEARCH DISPLAY ROUTINE (P17/P77)
;
; These programs provide the crew interface for the TPI search algorithms.
; P17 is used when the active vehicle is the CSM (Command Module).
; P77 is used when the active vehicle is the LM (Lunar Module).
;
; During Apollo 11's rendezvous after Eagle's ascent from the lunar surface,
; the crew would have used P77 since the LM was performing the rendezvous
; maneuvers to dock with Columbia.
;
; CREW INTERACTION SEQUENCE:
; 1. V06N37: Display TPI time (TTPI) for crew review/modification
; 2. V06N72: Display phase angle (PHI), altitude difference (DELTA H), 
;            and search option K
; 3. V06N58: Display computed ΔV at TPI, ΔV at TPF, and pericenter altitude
; 4. V06N55: Display pericenter safety code and central angle
;
; The crew can recycle the program with new TPI times or search options
; until they find a satisfactory solution.
; ============================================================================

;
; P17 ENTRY: Active vehicle is CSM (Command Module)
; Sets the AVFLAG to indicate CSM is performing the rendezvous maneuvers.
;
P17		TC	AVFLAGA			# AVFLAG = CSM , SET TRACK + UPDATE FLAGS
		TC	P17.1			; Continue to common display logic
;
; P77 ENTRY: Active vehicle is LM (Lunar Module)
; Sets the AVFLAG to indicate LM is performing the rendezvous maneuvers.
; This was the configuration used during Apollo 11's lunar orbit rendezvous,
; where Eagle (LM) maneuvered to dock with Columbia (CSM).
;
P77		TC	AVFLAGP			# AVFLAG = LEM , SET TRACK + UPDATE FLAGS
;
; COMMON DISPLAY SEQUENCE
; Both P17 and P77 converge here to display TPI search parameters and results.
;
P17.1		TC	P20FLGON		; Set tracking and update flags
;
; STEP 1: Display TPI time (TTPI) to crew
; V06N37 displays the proposed time for Terminal Phase Initiation.
; The crew can accept this time (PROCEED) or enter a new TPI time (ENTER).
;
		CAF	V06N37			; Load verb 06, noun 37
		TC	VNPOOH			; Display and wait for crew response
;
; STEP 2: Execute S17.1 to extrapolate state vectors to TTPI
; This updates the CSM and LM position/velocity to the proposed TPI time
; and computes initial search parameters (phase angle, altitude difference).
;
		TC	INTPRET			; Enter interpretive mode
		CLEAR	CALL			; Clear update flag
			UPDATFLG
			S17.1			; Call S17.1 routine to setup search
;
; STEP 3: Determine search option display value
; The KFLAG indicates which search sector was selected:
;   KFLAG OFF: K positive (LM is below CSM), display DELTA H = 2
;   KFLAG ON:  K negative (LM is above CSM), display DELTA H = 1
;
		SET	AXT,1			; Set update flag, load index register
			UPDATFLG
		DEC	2			# DELTA H = 2	K POSITIVE , KFLAG OFF
		BOFF	AXT,1			; Branch if KFLAG is off
			KFLAG
			+2			; Skip next instruction if off
		DEC	1			# DELTA H = 1	K NEGATIVE , KFLAG ON
# Page 560
		SXA,1	EXIT			; Store X1 to OPTION2, exit interpretive
			OPTION2
;
; STEP 4: Display search parameters to crew
; V06N72 displays:
;   R1: PHI (phase angle between vehicles in revolutions)
;   R2: DELTA H (altitude difference indicator: 1 or 2)
;   R3: Search option K (sector selection)
; Crew can modify these values to explore different search options.
;
		CAF	V06N72			# DISPLAY PHI , DELTA H , SEARCH OPTION K
		TC	VNCOMP17		; Display with recycle capability
;
; STEP 5: Process crew input from V06N72 display
; After the crew reviews (or modifies) the search parameters, process the
; selected option and set up the appropriate search sector.
;
		TC	INTPRET			; Return to interpretive mode
		CLEAR	SET			; Clear update flag, set KFLAG initially
			UPDATFLG
			KFLAG			; Assume K negative (option 1)
;
; Determine KFLAG setting based on OPTION2 value:
; If OPTION2 = 1 (K negative): KFLAG ON (set above)
; If OPTION2 = 2 (K positive): KFLAG OFF (cleared below)
;
		SLOAD	DSU			; Load OPTION2, subtract 1
			OPTION2			# RESET KFLAG ON FOR OPTION =1
			P21ONENN		#	     OFF FOR OPTION =2
		BHIZ	CLEAR			; Branch if zero (option=1), else clear
			+2			; Skip KFLAG clear if option=1
			KFLAG			; Clear KFLAG for option=2
;
; STEP 6: Set minimum pericenter altitude based on central body
; XRS indicates the sphere of influence:
;   XRS = 0: Moon (use HPL = 35000 ft minimum pericenter)
;   XRS = 1: Earth (use HPE = 85 n.mi. minimum pericenter)
; This safety constraint ensures the transfer orbit doesn't come dangerously
; close to the surface of the central body.
;
		SLOAD	BHIZ			; Load XRS flag
			XRS	+1		; XRS with offset indicator
			+4			; Branch if zero (Moon)
		DLOAD	GOTO			; Earth case: load HPL
			HPL			; Moon pericenter (35000 ft)
			P17.2			; Continue to S17.2 call
		DLOAD				; Moon case: load HPE
			HPE			; Earth pericenter (85 n.mi.)
P17.2		STCALL	HPERMIN			; Store minimum pericenter
			S17.2			; Execute S17.2 search routine
;
; STEP 7: Process S17.2 search results
; The S17.2 routine has completed the TPI search and computed the optimal
; transfer trajectory. The search has determined the delta-V values for TPI
; (Terminal Phase Initiation) and TPF (Terminal Phase Finalization), along
; with the pericenter altitude of the transfer orbit.
;
		SET	EXIT			; Set update flag, exit to basic
			UPDATFLG		; Indicate data has been updated
;
; STEP 8: Display delta-V and altitude results (V06N58)
; V06 is "Display Decimal" verb
; N58 is noun for "TPI delta-V, TPF delta-V, and altitude difference"
; This shows the crew:
;   R1: Delta-V for TPI maneuver (feet/sec)
;   R2: Delta-V for TPF maneuver (feet/sec)
;   R3: Altitude difference between vehicles (feet)
;
P17.3		CAF	V06N58			# DISPLAY DELTA VTPI , DELTA VTPF , AND H
		TC	VNCOMP17		; Display with recycle capability
;
; STEP 9: Display pericenter code and central angle (V06N55)
; V06 is "Display Decimal" verb
; N55 is noun for "Pericenter altitude code and central angle"
; This shows the crew:
;   R1: Pericenter altitude code (indicates safety margin)
;   R2: Central angle of transfer (degrees or revolutions)
; The pericenter code informs the crew whether the transfer trajectory
; passes safely above the central body's surface (critical safety check).
;
		CAF	V06N55			# DISPLAY PERICENTER CODE AND CENTRAL ANG.
		TC	BANKCALL		; Cross-bank call to display routine
		CADR	GOFLASHR		; Flash display, wait for crew response
;
; STEP 10: Process crew response to final display
; Crew has three options via DSKY input:
;   PROCEED (V33): Accept solution, terminate program
;   ENTER: Also accept solution, terminate program  
;   RECYCLE: Return to P17.1 for new search with different parameters
;
		TC	GOTOPOOH		# TERMINATE PROGRAM
		TC	GOTOPOOH		# END PROGRAM
		TC	P17.1			# RECYCLE WITH NEW TTPI OR SEARCH OPTION
;
; If crew chooses to recycle, blank R2 of the previous display and
; return to the beginning of P17 for a new TPI time or search option.
;
		CAF	TWO			# BLANK R2
		TC	BLANKET			; Blank specified display register
		TCF	ENDOFJOB		; End this job, return to waitlist
		EBANK=	RTRN
;
; ===========================================================================
; VNCOMP17 - Display and Update Subroutine for P17/P77
; ===========================================================================
;
; This subroutine handles DSKY display and crew input processing for the TPI
; search programs P17 and P77. It displays the verb/noun combination passed
; in the accumulator, waits for crew response via flashing display, and
; handles three possible crew actions: TERMINATE, PROCEED, or RECYCLE.
;
; Entry: Accumulator contains verb/noun code to display
;        Calling sequence uses TCR or TC to VNCOMP17
; Exit: Returns to caller based on crew response:
;       TERMINATE: Aborts program (illegal redisplay)
;       PROCEED: Returns to caller via QSAVED
;       RECYCLE: Returns to P17.1 for new computation (if MPAC option set)
;               Returns to VNCOMP17+3 for different search option
;
; During Apollo 11's rendezvous on July 21, 1969, the crew used this routine
; to review TPI search results. When the DSKY displayed the computed delta-V
; values, Armstrong and Aldrin could PROCEED to accept the solution, RECYCLE
; to try a different TPI time, or TERMINATE if the results were unsatisfactory.
;
VNCOMP17	EXTEND			; Extend next instruction
		QXCH	QSAVED		; Save return address from Q register
		TS	VERBNOUN	; Store verb/noun code from accumulator
		CA	VERBNOUN	; Load verb/noun code back to accumulator
		TCR	BANKCALL	; Call display routine with return
		CADR	GOFLASH		; Flash display, wait for crew input
		TC	-3		; TERMINATE ILLEGAL REDISPLAY
		TC	QSAVED		; PROCEED - return to caller
;
; Recycle Path: Crew has selected recycle option
; Check MPAC to determine which type of recycle:
; - If MPAC bit 6 is clear: Recycle with new TPI time (return to P17.1)
; - If MPAC bit 6 is set: Try different search option (return to VNCOMP17+3)
;
		CS	MPAC		; Load complement of MPAC (option flags)
		AD	BIT6		; Add bit 6 (test for search option bit)
		EXTEND			; Extend next instruction
# Page 561
		BZF	P17.1		; If zero, new TPI time - restart search
		TC	VNCOMP17 +3	; If non-zero, try different search option
;
; ===========================================================================
; ALARUMS - Error Exit for TPI Search Failure
; ===========================================================================
;
; This routine handles the error condition when the TPI search algorithm
; cannot find a safe solution. The most common failure mode is inability
; to find a trajectory that maintains safe pericenter altitude during the
; transfer orbit between TPI and TPF maneuvers.
;
; Entry: Called when search algorithm detects no valid solution exists
; Exit: Displays alarm code 00124 to crew, then terminates program
;
; Program Alarm 00124: "No safe pericenter in this sector"
; This alarm indicates the search algorithm examined the entire search sector
; but could not find a TPI/TPF maneuver combination that keeps the transfer
; orbit above minimum safe pericenter altitude (85 nm for Earth, 35,000 ft
; for Moon). The crew must select a different TPI time or accept that
; rendezvous is not feasible from current orbital geometry.
;
; Historical Context: During Apollo rendezvous operations, this alarm would
; require the crew to consult with Mission Control to determine alternate
; rendezvous timing or activate backup rendezvous procedures.
;
ALARUMS		SET	EXIT		; Set update flag and exit interpretive
			UPDATFLG	; Flag indicates state vectors updated
		TC	ALARM		; Issue program alarm to crew
		OCT	00124		; Alarm code: NO SAFE PERICENTER IN THIS SECTOR
;
; After displaying the alarm, show V05N09 to allow crew to review the
; problematic parameters before terminating the program.
;
		CAF	V05N09		; Load verb 05 noun 09 (display data)
		TC	VNCOMP17	; Call display routine
		TC	GOTOPOOH	; PROCEED ILLEGAL - TERMINATE PROGRAM
;
; ===========================================================================
; Verb/Noun Code Definitions
; ===========================================================================
;
; V06N72: Verb 06 (Display Decimal) Noun 72 (Program Number)
; Used to display program identification during TPI search operations.
;
V06N72		VN	0672		; Verb 06, Noun 72 definition
