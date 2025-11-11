# Copyright:	Public domain.
# Filename:	ORBITAL_INTEGRATION.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1227-1248
# Mod history:	2009-05-26 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-05 RSB	Fixed 3 typos.
#		2009-06-06 RSB	Page 1248 was missing entirely for some reason.
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

# Page 1227
# ORBITAL INTEGRATION

; ============================================================================
; FILE: ORBITAL_INTEGRATION.agc
; MODULE: Orbital Integration
; MISSION PHASE: trans-lunar/lunar-orbit/descent/ascent/rendezvous/trans-earth
;
; TL;DR: Implements precision orbit propagation using numerical integration
;        to maintain accurate navigation state over extended coast periods.
;        Uses Encke method for perturbed motion including Earth/Moon oblateness,
;        solar/lunar gravity perturbations, and automatic integration step
;        size control. Critical for trajectory planning during translunar
;        coast, lunar orbit operations, and transearth return.
;
; COMMENT-ONLY READERS: This code continuously updates the spacecraft's
;        position and velocity as it coasts through space, accounting for
;        gravitational variations that affect the trajectory.
; CODE-ALONG READERS: Study the Encke method implementation, perturbation
;        force calculations, and Nystrom numerical integration algorithm.
; ============================================================================

# DELETE
		BANK	13
		SETLOC	ORBITAL
		BANK
		COUNT*	$$/ORBIT

# DELETE

; ============================================================================
; KEPPREP - Kepler Parameter Preparation
;
; This routine prepares parameters for Keplerian (conic) orbit calculations.
; It computes the fundamental orbital parameters needed to characterize the
; spacecraft's trajectory using classical two-body orbital mechanics. These
; parameters are used throughout translunar coast, lunar orbit, and transearth
; return when the spacecraft follows approximately elliptical paths.
;
; The routine calculates:
; - Square root of gravitational parameter (sqrt(MU))
; - Position unit vector
; - Angular momentum components
; - Eccentricity-related parameters
;
; TECHNICAL NOTES:
; - Uses interpretive language for vector/matrix operations
; - PBODY index selects Earth (0) or Moon (2) gravitational parameter
; - MU scaling: Earth +18 (meters^3/centisec^2), Moon +15
; - Position vector scaling: +29 (Earth) or +27 (Moon) meters
; - Velocity vector scaling: +7 (Earth) or +5 (Moon) meters/centisec
; ============================================================================

KEPPREP		LXA,2	SETPD
			PBODY
			0
		DLOAD*	SQRT		# SQRT(MU) (+18 OR +15)		0D	PL 2D
			MUEARTH,2
; Load position vector RCV, normalize to unit vector (direction to spacecraft).
; This establishes the radial direction component of the orbit.
		PDVL	UNIT		#					PL 8D
			RCV
		PDDL	NORM		# NORM R (+29 OR +27 - N1)	2D	PL 4D
			36D
			X1
; Compute angular momentum and related parameters for orbit characterization.
; The dot product of position unit vector with velocity gives the radial
; velocity component, essential for determining orbit eccentricity.
		PDVL
		DOT	PDDL		# F*SQRT(MU) (+7 OR +5) 	4D	PL 6D
			VCV
			TAU.		# (+28)
; Time parameter adjustment for transfer time calculations.
; TAU represents the time since last rectification, TC is the current time.
		DSU	NORM
			TC
			S1
; Shift right by 1 bit for scaling alignment, then divide.
; This computes a scaled time parameter for the Kepler solution.
		SR1
		DDV	PDDL
			2D
; Compute FS (f*s) parameter used in orbit calculations.
; This represents a combination of radial position and time factors.
		DMP	PUSH		# FS (+6 +N1-N2) 		6D	PL 8D
			4D
; Square of FS parameter for higher-order orbit terms.
		DSQ	PDDL		# (FS)SQ (+12 +2(N1-N2))	8D	PL 10D
			4D
; Compute SSQ/MU (specific angular momentum squared divided by gravitational parameter).
; This quantity relates to orbit energy and eccentricity calculations.
		DSQ	PDDL*		# SSQ/MU (-20R +2(N1-N2))	10D	PL 12D
			MUEARTH,2
		SR3	SR4
; Prealign gravitational parameter MU for subsequent calculations.
; Load velocity vector and compute velocity squared (kinetic energy term).
		PDVL	VSQ		# PREALIGN MU (+43 OR +37) 	12D	PL 14D
			VCV
; Compute orbital energy components: kinetic energy minus potential energy.
; This determines whether orbit is elliptical, parabolic, or hyperbolic.
		DMP	BDSU		#					PL 12D
			36D
; Calculate -(1/R - ALPHA) where ALPHA is related to orbital energy.
; This term appears in the universal Kepler equation formulation.
		DDV	DMP		#					PL 10D
			2D		# -(1/R-ALPHA) (+12 +3N1-2N2)
; Multiply by 2/3 constant and scale for precision in series expansion.
; The factor 10L represents logarithmic scaling adjustment.
		DMP	SL*
			DP2/3
			0 	-3,1	# 10L(1/R-ALPHA) (+13 +2(N1-N2))
; Compute 2(FS)^2 - additional terms for Kepler equation series expansion.
; Index register X1 holds the scaling normalization difference (N2-N1).
		XSU,1	DAD		# 2(FS)SQ - ETCETERA			PL 8D
			S1		# X1 = N2-N1
; Subtract FS and add higher-order correction terms for series convergence.
; This implements the power series solution to the universal Kepler equation.
		SL*	DSU		# -FS+2(FS)SQ ETC (+6 +N1-N2)		PL 6D
			8D,1
		DMP	DMP
			0D
			4D
; Final scaling adjustments for the S function (universal anomaly function).
; S(-FS(1-2FS)-1/6...) represents the series sum for position update.
		SL*	SL*
# Page 1228
			8D,1
			0,1		# S(-FS(1-2FS)-1/6...) (+17 OR +16)
; Add to previous Kepler parameter XKEP and store result in XKEPNEW.
; This iterative update improves the orbit solution accuracy.
		DAD	PDDL		#					PL 6D
			XKEP
		DMP	SL*		# S(+17 OR +16)
			0D
			1,1
; Check for overflow and handle via TCDANZIG routine if needed.
; Store the new Kepler parameter and prepare for return.
		BOVB	DAD
			TCDANZIG
		STADR
		STORE	XKEPNEW
		STQ	AXC,1
			KEPRTN
		DEC	10
; Branch based on MOONFLAG to select Moon gravitational parameter.
; If in Moon sphere of influence, use Moon MU (index 2), else Earth MU (index 10).
		BON	AXC,1
			MOONFLAG
			KEPLERN
		DEC	2
		GOTO
			KEPLERN

# Page 1229
; ============================================================================
; FBR3 - Fourth-Order Runge-Kutta Integration Step 3
;
; This routine performs the third step of a fourth-order Runge-Kutta numerical
; integration sequence for orbit propagation. During translunar coast and lunar
; orbit, the spacecraft's position and velocity are continuously updated by
; integrating the equations of motion. This integration accounts for
; gravitational acceleration and perturbation forces.
;
; The Runge-Kutta method samples the differential equation at multiple points
; within each time step, providing higher accuracy than simple Euler integration.
; This is critical for maintaining navigation accuracy over multi-day coast
; periods when trajectory errors would otherwise accumulate.
;
; TECHNICAL NOTES:
; - Uses DIFEQCNT as loop counter for integration steps
; - DT/2 represents half the integration time step
; - Updates time parameter TAU and ephemeris time TET
; - Calls KEPPREP to prepare next Kepler calculation
; ============================================================================

FBR3		LXA,1	SSP
			DIFEQCNT
			S1
		DEC	-13
; Load half time step (DT/2) and shift right by 9 bits for scaling.
; This prepares the time increment for the integration step.
		DLOAD	SR
			DT/2
			9D
; Test index and round result, then push to pushdown stack.
		TIX,1	ROUND
			+1
; Add to current time TC to compute new TAU (time since rectification).
		PUSH	DAD
			TC
		STODL	TAU.
; Add to ephemeris time TET to advance the mission clock.
; Then call KEPPREP to prepare parameters for the updated orbit state.
		DAD
			TET
		STCALL	TET
			KEPPREP

# Page 1230
# AGC ROUTINE TO COMPUTE ACCELERATION COMPONENTS.

; ============================================================================
; ACCOMP - Acceleration Components Calculation
;
; This is the core routine that computes gravitational acceleration acting
; on the spacecraft. As the spacecraft coasts through cislunar space or orbits
; the Moon, it experiences gravitational forces from multiple bodies: Earth,
; Moon, and Sun. This routine calculates the resulting acceleration vector
; that drives the numerical integration of the equations of motion.
;
; The acceleration calculation includes:
; - Primary gravitational attraction (Earth or Moon, depending on sphere)
; - Third-body perturbations (Moon when near Earth, Earth when near Moon)
; - Oblateness effects (J2 term) from non-spherical gravitational fields
; - Solar radiation pressure (when enabled)
;
; During Apollo 11's translunar coast (July 16-19, 1969), this routine
; executed continuously to maintain trajectory accuracy for lunar orbit
; insertion. During lunar orbit and descent preparations, it provided the
; foundation for navigation updates.
;
; TECHNICAL NOTES:
; - PBODY index selects primary gravitational body (0=Earth, 2=Moon)
; - Acceleration stored in ALPHAV, BETAV vectors
; - GAMCOMP subroutine handles detailed perturbation calculations
; - OBLATE subroutine adds oblateness (J2) perturbation terms
; ============================================================================

ACCOMP		LXA,1	LXA,2
			PBODY
			PBODY
; Initialize force vector FV to zero before accumulating perturbations.
		VLOAD
			ZEROVEC
		STOVL	FV
			ALPHAV
; Scale ALPHAV vector and add to position vector RCV to compute BETAV.
; BETAV represents the position relative to the perturbing body.
; Scaling index 2 selects appropriate units for Earth or Moon calculations.
		VSL*	VAD
			0 	-7,2
			RCV
		STORE	BETAV
; Check DIM0FLAG to determine storage strategy for integration vectors.
; Exchange index register with DIFEQCNT for vector table addressing.
		BOF	XCHX,2
			DIM0FLAG
			+5
			DIFEQCNT
		STORE	VECTAB,2
		XCHX,2
			DIFEQCNT
; Load ALPHAV, compute unit vector (direction to perturbing body).
; The unit vector establishes the direction of gravitational acceleration.
		VLOAD	UNIT
			ALPHAV
		STODL	ALPHAV
			36D
; Store magnitude of ALPHAV in ALPHAM for distance-dependent force calculation.
		STORE	ALPHAM
; Call GAMCOMP to compute gravitational perturbation contribution.
; This calculates third-body effects (Moon perturbing Earth orbit, etc.).
		CALL
			GAMCOMP
; Load BETAV (spacecraft position relative to perturbing body).
; Store index in S2 for later restoration.
		VLOAD	SXA,1
			BETAV
			S2
; Update ALPHAV with BETAV and store BETAM magnitude.
; These represent the spacecraft's position relative to the secondary body.
		STODL	ALPHAV
			BETAM
		STORE	ALPHAM
; Check MIDFLAG to determine if at midpoint of integration step.
; If not at midpoint, skip to OBLATE calculation.
; Otherwise, load ephemeris time TET and call LSPOS (lunar/solar position).
		BOF	DLOAD
			MIDFLAG
			OBLATE
			TET
		CALL
			LSPOS
; Set index register X2 to 2 and restore index X1 from S2.
; This prepares for perturbing body vector calculations.
		AXT,2	LXA,1
			2
			S2
; Branch based on MOONFLAG to determine perturbing body configuration.
; If MOONFLAG set (in Moon sphere), use different indexing.
		BOF
			MOONFLAG
			+3
; Complement vector and set index to 0 for Moon-centered calculations.
		VCOMP	AXT,2
			0
; Store BETAV (position relative to perturbing body) and RPQV.
; RPQV represents position of perturbing body (Moon or Sun).
		STORE	BETAV
		STOVL	RPQV
# Page 1231
			2D
; Store RPSV (position of solar perturbing body).
; During translunar coast, solar perturbations become significant.
		STORE	RPSV
		SLOAD	DSU
			MODREG
			OCT27
		BHIZ	BOF
			+3
			DIM0FLAG
			GETRPSV
		VLOAD	VXSC
			ALPHAV
			ALPHAM
		VSR*	VSU
			1,2
			BETAV
		XCHX,2
			DIFEQCNT
		STORE	VECTAB 	+6,2
		STORE	RQVV
		XCHX,2
			DIFEQCNT
GETRPSV		VLOAD	INCR,1
			RPQV
			4
		CLEAR	BOF
			RPQFLAG
			MOONFLAG
			+5
		VSR	VAD
			9D
			RPSV
		STORE	RPSV
		CALL
			GAMCOMP
; Set index register X2 to 4 and increment X1 by 4.
; This prepares for solar perturbation calculations.
		AXT,2	INCR,1
			4
			4
; Load solar position vector RPSV and store as BETAV.
; Then call GAMCOMP to compute solar gravitational perturbation.
		VLOAD
			RPSV
		STCALL	BETAV
			GAMCOMP
; After computing all perturbations, proceed to OBLATE routine.
; OBLATE adds oblateness (J2) perturbations from non-spherical bodies.
		GOTO
			OBLATE

; ============================================================================
; GAMCOMP - Third-Body Gravitational Perturbation Calculation
;
; This routine computes the gravitational acceleration due to a third body
; (perturbing body) on the spacecraft. In cislunar space, the spacecraft
; experiences not only the primary gravitational attraction (Earth or Moon)
; but also significant perturbations from other bodies.
;
; During Apollo 11's translunar coast, Earth's gravity perturbed the lunar
; trajectory. During lunar orbit, Earth's distant pull created small but
; measurable perturbations requiring correction for accurate navigation.
;
; The calculation uses the standard two-body perturbation formula:
;   a_pert = MU * (r_body/|r_body|^3 - r_sc/|r_sc|^3)
; where r_body is position of perturbing body, r_sc is spacecraft position.
;
; MATHEMATICAL APPROACH:
; - Compute distances to spacecraft and perturbing body
; - Calculate RHO = |r_body|/|r_sc| ratio
; - Use series expansion for efficiency when RHO is large
; - Scale and normalize for AGC fixed-point arithmetic precision
; ============================================================================

GAMCOMP		VLOAD	VSR1
			BETAV
; Load BETAV vector (position to perturbing body), shift right by 1 for scaling.
; Compute BETAV squared (distance squared to perturbing body).
		VSQ	SETPD
			0
; Normalize and round the squared distance, storing normalization factor.
; This prepares B^2 for the perturbation force calculation.
		NORM	ROUND
			31D
; Push normalized B squared to pushdown list, then load and normalize ALPHAM.
		PDDL	NORM		# NORMED B SQUARED TO PD LIST
# Page 1232
			ALPHAM		# NORMALIZE (LESS ONE) LENGTH OF ALPHA
			32D		# SAVING NORM SCALE FACTOR IN X1
; Shift right by 1 and push, then load BETAV and compute unit vector.
; BETAV unit vector gives direction to perturbing body.
		SR1	PDVL
			BETAV		# C(PDL+2) = ALMOST NORMED ALPHA
		UNIT
; Store normalized BETAV and its magnitude BETAM.
; BETAM represents distance to perturbing body (Moon, Earth, or Sun).
		STODL	BETAV
			36D
		STORE	BETAM
; Normalize and divide ALPHAM by BETAM to compute RHO parameter.
; RHO = ratio of spacecraft distance from primary to perturbing body distance.
		NORM	BDDV		# FORM NORMALIZE QUOTIEN ALPHAM/BETAM
			33D
; Shift right by 1 with rounding, push to PD list as normalized RHO.
; RHO determines whether to use full calculation or series shortcut.
		SR1R	PUSH		# C(PDL+2) = ALMOST NORMALIZE RHO.
; Load ASCALE indexed by X1 to get appropriate scale factor.
; ASCALE provides scaling for different coordinate reference frames.
		DLOAD*
			ASCALE,1
		STORE	S1
; Exchange and add to X2, manipulating index for subsequent calculations.
; These index operations prepare for proper scaling alignment.
		XCHX,2	XAD,2
			S1
			32D
; Subtract from X2 and load from PD list, performing scale adjustments.
; Result provides properly scaled distance ratio for perturbation calculation.
		XSU,2	DLOAD
			33D
			2D
; Shift right by computed amount, exchange X2 with S1 index register.
; This aligns the RHO value to correct magnitude for comparison.
		SR*	XCHX,2
			0 	-1,2
			S1
; Push scaled RHO value, shift right with rounding to get RHO/4.
; RHO/4 provides starting point for perturbation series expansion.
		PUSH	SR1R		# RHO/4 TO 4D
; Push value, then load ALPHAV and compute dot product with BETAV.
; Dot product captures geometric relationship between position vectors.
		PDVL	DOT
			ALPHAV
			BETAV
; Shift left with rounding, subtract to form (RHO/4) - 2(ALPHAV·BETAV).
; This term appears in the series expansion for perturbation acceleration.
		SL1R	BDSU		# (RHO/4) - 2(ALPHAV/2.BETAV/2)
; Push to PDL+6 and multiply by 4 for scaling adjustment.
; Result contributes to Q parameter calculation in series.
		PUSH	DMPR		# TO PDL+6
			4
; Shift left and push, then add 1/4 constant.
; Building up terms for square root computation in series.
		SL1
		PUSH	DAD
			DQUARTER
; Push and take square root. This computes Q parameter.
; Q is intermediate variable in third-body perturbation series expansion.
		PUSH	SQRT
; Multiply by value at 10D and push result.
; Continues building series expansion terms.
		DMPR	PUSH
			10D
; Shift left and add 1/4 to form (1/4) + 2(Q/2).
; This term appears in denominator of perturbation formula.
		SL1	DAD
			DQUARTER
; Push and load value at 10D, add 1/2 constant.
; Forms (1/4) + 2((Q+1)/4) term and pushes to PD+14D.
		PDDL	DAD		# (1/4)+2((Q+1)/4)	TO PD+14D
			10D
			HALFDP
; Multiply by value at 8D and shift left for scaling.
; Continues building numerator of perturbation series.
		DMPR	SL1
			8D
; Add 3/8 constant and divide by value at 14D.
; Completes evaluation of rational function in series expansion.
		DAD	DDV
			THREE/8
			14D
; Multiply by 6 and scale BETAV vector by result.
; This forms the G coefficient times B vector component.
		DMPR	VXSC
			6
			BETAV		#		_
; Push vector result to PD+16D, load ALPHAV and shift right by 3.
; Prepares A vector term for addition to perturbation.
		PDVL	VSR3		# (G/2)(C(PD+4))B/2 TO PD+16D
# Page 1233
			ALPHAV
; Add vectors and push result to PD+16D.
; Forms A/2 + (G/2)(PD+4)B/2 = total perturbation direction.
		VAD	PUSH		# A12 + C(PD+16D) TO PD+16D
; Load scalar at 0, multiply by value at 12D for magnitude scaling.
; Prepares to compute final perturbation acceleration magnitude.
		DLOAD	DMP
			0
			12D
; Normalize with rounding, store normalization factor at 30D.
; Ensures proper magnitude representation in fixed-point arithmetic.
		NORM	ROUND
			30D
; Divide by 2 and multiply by gravitational parameter MU (indexed).
; Applies correct physical scaling for perturbation acceleration.
		BDDV	DMP*
			2
			MUEARTH,2
; Complement (negate) and scale perturbation vector.
; Sign change accounts for direction convention in acceleration.
		DCOMP	VXSC
; Exchange X2 with S1, add S2 to X2 for index adjustment.
; Prepares scaling indices for final vector normalization.
		XCHX,2	XAD,2
			S1
			S2
; Subtract normalization factors (30D and 31D) from X2.
; Aligns scale factors to match physical acceleration units.
		XSU,2	XSU,2
			30D
			31D
; Branch on overflow to clear overflow indicator.
; Prevents false overflow detection in subsequent operations.
		BOV			# CLEAR OVIND
			+1
; Shift perturbation vector right by computed scale factor in X2.
; Final scaling adjustment to match FV (force vector) units.
		VSR*	XCHX,2
			0	-1,2
			S1
; Add scaled perturbation acceleration to total force vector FV.
; FV accumulates all perturbation forces for this integration step.
		VAD
			FV
; Store updated total force vector back to FV.
; FV now contains contributions from all computed perturbations.
		STORE	FV
; Branch on overflow and return via Q if no overflow occurred.
; Successful completion returns control to calling integration routine.
		BOV	RVQ		# RETURN IF NO OVERFLOW
			+1

; ============================================================================
; GOBAQUE - Integration Recovery Routine
;
; Called when integration step produces unacceptable errors or discontinuity.
; Resets integration parameters and attempts recovery via rectification.
; If TDELTAV is zero (no velocity change), aborts with program alarm.
; ============================================================================
GOBAQUE		VLOAD	ABVAL
			TDELTAV
; Test if velocity change magnitude is zero.
; Zero TDELTAV indicates failed integration requiring abort.
		BZE
			INT-ABRT
; Load integration step size H and shift right by 9D (divide by 512).
; Reduces step size to attempt more conservative integration.
		DLOAD	SR
			H
			9D
; Push reduced step and subtract current time TC from it.
; Computes new time parameter TAU for Kepler initialization.
		PUSH	BDSU
			TC
; Store to TAU and load ephemeris time TET.
; Prepares for recalculation of orbital elements.
		STODL	TAU.
			TET
; Subtract value at top of stack and store result back to TET.
; Adjusts ephemeris time for restart integration point.
		DSU	STADR
		STCALL	TET
			KEPPREP
; Call RECTIFY to recompute reference trajectory from current state.
; Rectification corrects accumulated numerical errors.
		CALL
			RECTIFY
; Set RPQFLAG (rectification performed) and branch to TESTLOOP.
; Returns to integration main loop with corrected state.
		SETGO
			RPQFLAG
			TESTLOOP

; ============================================================================
; INT-ABRT - Integration Abort Handler
;
; Called when integration cannot recover from errors.
; Issues program alarm 00430 to crew and mission control.
; This terminates current integration task, requiring manual intervention.
; ============================================================================
INT-ABRT	EXIT
		TC	POODOO
		OCT	00430

# Page 1234
# THE OBLATE ROUTINE COMPUTES THE ACCELERATION DUE TO OBLATENESS.  IT USES THE UNIT OF THE VEHICLE
# POSITION VECTOR FOUND IN ALPHAV AND THE DISTANCE TO THE CENTER IN ALPHAM.  THIS IS ADDED TO THE SUM OF THE
# DISTURBING ACCELERATIONS IN FV AND THE PROPER DIFEQ STAGE IS CALLED VIA X1.

; ============================================================================
; OBLATE - Oblateness Perturbation Computation
;
; Computes gravitational perturbations due to Earth and Moon non-spherical
; mass distribution. Models perturbations using zonal harmonics (J2, J3, J4)
; and tesseral harmonics (J22 sectoral term).
;
; J2 is the dominant oblateness term (Earth equatorial bulge ~1/1000).
; J3, J4 provide higher-order corrections for asymmetric mass distribution.
; J22 models ellipticity of the equatorial plane.
;
; These perturbations are critical for accurate orbit determination during
; multi-day coast phases when small accelerations accumulate significantly.
; ============================================================================
OBLATE		LXA,2	DLOAD
			PBODY
			ALPHAM
; Load index for central body (Earth=0, Moon=1) into X2 register.
; Load ALPHAM (position vector magnitude) for perturbation scaling.
		SETPD	DSU*
			0
			RDE,2
; Set pushdown pointer to 0 and subtract body equatorial radius RDE.
; Tests if spacecraft altitude is above atmospheric region.
		BPL	BOF		# GET URPV
			NBRANCH
			MOONFLAG
			COSPHIE
; If result positive (above atmosphere), branch to NBRANCH to skip computation.
; If MOONFLAG is off (orbiting Earth), branch to COSPHIE for Earth-specific setup.
; Otherwise continue to compute unit position vector URPV in body-fixed coordinates.
		VLOAD	PDDL
			ALPHAV
			TET
; Load ALPHAV (unit position vector in inertial frame) and push to stack.
; Load TET (ephemeris time) for coordinate transformation.
		PDDL	CALL
			3/5
			R-TO-RP
; Push constant 3/5 (time flag) and call R-TO-RP subroutine.
; R-TO-RP transforms from inertial to rotating (planet-fixed) coordinates.
		STORE	URPV
; Store transformed unit position vector to URPV (Unit Rotating Position Vector).
; URPV now represents spacecraft position in body-fixed frame aligned with poles.

; ============================================================================
; TRANSITION: From coordinate transformation to zonal harmonic setup
;
; With spacecraft position now expressed in Moon's rotating reference frame,
; we compute the unit vector UZ aligned with Moon's rotation axis.
; This UZ vector is essential for evaluating Legendre polynomials which
; depend on latitude (angle from equatorial plane).
; ============================================================================
		VLOAD	VXV
			504LM
			ZUNIT
; Load 504LM vector (likely libration correction) and cross with ZUNIT.
; Vector cross product produces vector perpendicular to both inputs.
		VAD	VXM
			ZUNIT
			MMATRIX
; Add ZUNIT and multiply result by MMATRIX (rotation matrix).
; Transforms libration-corrected axis vector to current body-fixed frame.
		UNIT			# POSSIBLY UNNECESSARY
; Normalize to unit vector. Original comment notes this may be redundant
; if result is already normalized from previous computation.

; ============================================================================
; COMTERM - Common Computation of Zonal Harmonics
;
; Computes acceleration due to zonal harmonics J2, J3, J4.
; Uses Legendre polynomials P2, P3, P4, P5 evaluated at cos(phi) where
; phi is latitude angle from equatorial plane.
; Result is acceleration vector in body-fixed coordinates.
; ============================================================================
COMTERM		STORE	UZ
; Store unit rotation axis vector UZ (planet's polar axis in body-fixed frame).
; For Moon, this accounts for libration (small wobble in lunar rotation).
; For Earth, this is simply the polar axis direction.

		DLOAD	DMPR
			COSPHI/2
			3/32
; Load COSPHI/2 (half the cosine of latitude angle phi).
; Multiply by 3/32 to begin computing Legendre polynomial P2.
; P2(x) = (3x² - 1)/2 represents first oblateness correction.

		PDDL	DSQ		# P2/64 TO PD0
			COSPHI/2
; Push result to stack (will become P2/64 at pushdown 0).
; Load COSPHI/2 again and square it: (cos(phi)/2)².
; Squaring is fundamental operation for all Legendre polynomial terms.

		DMPR	DSU
			15/16
			3/64
; Multiply cos²(phi)/4 by 15/16 and subtract 3/64.
; This computes scaled version of P3 Legendre polynomial.
; P3(x) = (5x³ - 3x)/2 represents pear-shaped distortion.

		PUSH	DMPR		# P3/32 TO PD2
			COSPHI/2
; Push P3/32 to stack at pushdown 2.
; Multiply previous result by COSPHI/2 to advance to next polynomial order.
; Building P4 term through recursive Legendre polynomial relations.

		DMP	SL1R
			7/12
; Multiply by 7/12 and shift left 1 bit (multiply by 2) with rounding.
; Scaling factor derived from Legendre polynomial recurrence formulas.
; Prepares intermediate term for P4 computation.

		PDDL	DMPR
			0
			2/3
; Push intermediate result and load value from pushdown 0.
; Multiply by 2/3 to continue P4 polynomial evaluation.
; P4(x) = (35x⁴ - 30x² + 3)/8 models fourth-order gravity variation.

		BDSU	PUSH		# P4/128 TO PD4
; Subtract (backward subtract: operand - accumulator) and push.
; Result is P4/128 stored at pushdown 4.
; This completes the fourth-order Legendre polynomial term.

		DMPR	DMPR
			COSPHI/2	# BEGIN COMPUTING P5/1024
			9/16
; Begin computing P5 (fifth-order polynomial).
; Double multiply: by COSPHI/2 then by 9/16.
; P5 provides highest-order gravity field correction implemented.

		PDDL	DMPR
			2
			5/128
; Push P5 intermediate term at pushdown 2.
; Load value from pushdown 2 and multiply by 5/128.
; Final scaling for P5/1024 Legendre polynomial term.
# Page 1235
		BDSU
; Subtract to complete P5 polynomial evaluation.
; All Legendre polynomials P2 through P5 now computed and ready for use.

		DMP*
			J4REQ/J3,2
; Multiply by J4REQ/J3 (ratio of J4 to J3 harmonic coefficients).
; Indexed by X2 register: selects Earth or Moon gravitational constants.
; J4 coefficient weights P5 polynomial in final acceleration computation.

		DDV	DAD		#	       -3
			ALPHAM		# (((P5/256)B 2  /R+P4/32)  /R+P3/8)ALPHAV
			4		#	     4		   3
; Divide by ALPHAM (radial distance R) and add term from pushdown 4.
; Original comment shows mathematical structure: nested divisions by R.
; This computes radial component: (P5·J4/R + P4·J3/R + P3·J2/R).

		DMPR*	DDV
			2J3RE/J2,2
			ALPHAM
; Multiply by 2J3RE/J2 (scaled harmonic ratio times equatorial radius).
; Divide by ALPHAM again to incorporate R⁻² dependence.
; Builds up acceleration magnitude with proper distance scaling.

		DAD	VXSC
			2
			ALPHAV
; Add term from pushdown 2 and multiply by ALPHAV (unit radial vector).
; Converts scalar radial acceleration to vector form.
; ALPHAV points from planet center toward spacecraft position.

		STODL	TVEC
; Store resulting radial component vector to TVEC (temporary vector).
; Load accumulator for next computation phase (latitude-dependent term).

		DMP*	SR1
			J4REQ/J3,2
; Multiply by J4REQ/J3 ratio again, shift right 1 bit (divide by 2).
; Beginning computation of polar axis component of acceleration.
; This component acts along UZ direction (perpendicular to equator).

		DDV	DAD
			ALPHAM
; Divide by ALPHAM (distance R) and add intermediate term.
; Similar nested structure as radial component computation.

		DMPR*	SR3
			2J3RE/J2,2
; Multiply by 2J3RE/J2 and shift right 3 bits (divide by 8).
; Scaling for polar axis acceleration component differs from radial.

		DDV	DAD
			ALPHAM
; Final division by ALPHAM and addition of remaining polynomial term.
; Completes scalar magnitude of polar axis acceleration component.

		VXSC	VSL1
			UZ
; Multiply scalar by UZ (unit vector along rotation axis).
; Shift left 1 bit to restore proper scaling.
; Result is acceleration component along polar axis direction.

		BVSU
			TVEC
; Backward vector subtract: TVEC - (polar component).
; Combines radial and polar components into total oblateness acceleration.
; Sign convention: positive outward in radial direction.

		STODL	TVEC
			ALPHAM
; Store final oblateness acceleration vector to TVEC.
; Load ALPHAM for scaling computation preparatory to adding to FV.

		NORM	DSQ
			X1
; Normalize ALPHAM (find scale factor) storing exponent to X1.
; Square the normalized value: R² in normalized form.
; Prepares for R⁴ computation needed for final scaling.
		DSQ	NORM
			S1		#	  4
; Square again to get R⁴, normalize and store new scale factor to S1.
; Original comment: "NORMED R⁴ TO 0D" - fourth power for oblateness scaling.

		PUSH	BDDV*		# NORMED R  TO 0D
			J2REQSQ,2
; Push normalized R⁴ to pushdown 0.
; Backward double-divide by J2REQSQ (J2 × Rₑ², equatorial radius squared).
; Indexed by X2: selects Earth or Moon gravitational parameters.
; Computes proper normalization for oblateness force magnitude.

		VXSC	BOV
			TVEC
			+1		# (RESET OVERFLOW INDICATOR)
; Multiply TVEC (oblateness acceleration direction) by computed scalar.
; Branch on overflow to +1 (next instruction, resetting overflow indicator).
; Overflow check ensures numerical stability of force computation.

		XAD,1	XAD,1
			X1
			X1
; Add X1 to index register X1 three times total (X1 = X1 + X1 + X1 = 3×X1).
; Accumulates scale factors from repeated normalization operations.
; Prepares final shift count for restoring proper magnitude.

		XAD,1	VSL*
			S1
			0 -22D,1
; Add S1 to accumulated scale in X1: total scale factor.
; Variable shift left by (0 - 22 - X1) positions.
; Restores oblateness force vector to correct physical units and magnitude.

		VAD	BOV
			FV
			GOBAQUE
; Add scaled oblateness force to FV (total force accumulator).
; Branch on overflow to GOBAQUE: handles numerical overflow gracefully.
; FV now contains two-body force plus oblateness perturbation.

		STCALL	FV
			QUALITY1
; Store updated total force vector to FV.
; Call QUALITY1: next quality checkpoint for integration control.
; Allows termination, continuation, or body-switching for integration.

QUALITY3	DSQ			# J22 TERM X R**4 IN 2D, SCALED B61
					# AS VECTOR.
; QUALITY3: Alternate entry point for J22 gravity harmonic computation.
; Computes J22 sectorial harmonic (equatorial ellipticity term).
; Input: cos²(phi) already in accumulator, R⁴ in pushdown 2.
; J22 represents equatorial bulge: Earth/Moon not perfectly axisymmetric.

# Page 1236
		PUSH	DMP		# STORE COSPHI**2 SCALED B2 IN 8D
			5/8		# 5 SCALED B3
; Push cos²(phi) to pushdown 0 (will be at 8D after operations).
; Multiply by 5/8: begins computing latitude-dependent coefficient.
; Formula involves (1 - 7cos²φ) and (5cos²φ - 1) terms for J22.

		PDDL	SR2		# PUT 5 COSPHI**2, D5, IN 8D.  GET
					# COSPHI**2 D2 FROM 8D
; Push result (5cos²φ/8) to stack.
; Load double precision and shift right 2 bits: gets cos²φ/4.
; Stack manipulation prepares multiple terms for J22 polynomial.

		DAD	BDSU		# END UP WITH (1-7 COSPHI**2), B5
			8D		# ADDING COSPHI**2 B4 SAME AS COSPHI**2
					# X 2 D5
			D1/32		# 1 SCALED B5
; Add term from pushdown 8D and backward subtract from D1/32 (constant 1/32).
; Result: (1 - 7cos²φ) scaled properly for J22 computation.
; This term weights acceleration component in equatorial plane.

		DMP	DMP
			URPV		# X COMPONENT
			5/8		# 5 SCALED B3
; Multiply by URPV X-component (unit position vector in rotating frame).
; Multiply by 5/8 again for final coefficient scaling.
; Builds directional component of J22 force along body-fixed X-axis.

		VXSC	VSL5		# AFTER SHIFT, SCALED B5
			URPV		# VECTOR, B1.
; Vector multiply by URPV (full 3-component vector).
; Shift left 5 bits to restore proper scaling (B5 after shift).
; Creates vector term proportional to position in equatorial plane.

		PDDL			# VECTOR INTO 8D, 10D, 12D, SCALED B5.
					# GET 5 COSPHI**2 OUT OF 8D
; Push 3-component vector to pushdown locations 8D, 10D, 12D.
; Load 5cos²φ term from pushdown 8D for further processing.
; Prepares to modify individual vector components with latitude terms.

		DSU	DAD
			D1/32		# 1 B5
			8D		# X COMPONENT (SAME AS MULTIPLYING
					# BY UNITX)
; Subtract constant 1/32 and add X-component from pushdown 8D.
; Computes modified X-component: (5cos²φ - 1) + earlier X term.
; Implements latitude-dependent variation of J22 along X-axis.

		STODL	8D
			URPV		# X COMPONENT
; Store modified X-component back to pushdown 8D.
; Load URPV X-component again for Z-component calculation.
; J22 computation requires coupled X and Z terms.

		DMP	DMP
			URPV	+4	# Z COMPONENT
			5/8		# 5 B3 ANSWER B5
; Multiply X-component by Z-component of URPV.
; Multiply by 5/8 for coefficient scaling.
; Cross-term (X·Z) appears in J22 sectorial harmonic expression.

		SL1	DAD		# FROM 12D FOR Z COMPONENT (SL1 GIVES 10
					# INSTEAD OF 5 FOR COEFFICIENT)
; Shift left 1 (doubles result: gives coefficient 10 instead of 5).
; Add term from pushdown 12D (original Z-component calculation).
; Completes Z-component modification for J22 equatorial ellipticity.
		PDDL	NORM		# BACK INTO 12D FOR Z COMPNENT.
			ALPHAM		# SCALED B27 FOR MOON
			X2
; Push result back into pushdown 12D (restores Z-component position).
; Load ALPHAM (alpha for Moon, reciprocal semi-major axis, scaled B27).
; Normalize and store scale factor to X2.
; ALPHAM used in final scaling of J22 force magnitude.

		PUSH	SLOAD		# STORE IN 14D, DESTROYING URPV
					# X COMPONENT
			E32C31RM
; Push normalized ALPHAM to pushdown 14D.
; Single-load E32C31RM: Earth/Moon ratio constant for J22 coefficient.
; Overwrites URPV X-component (no longer needed after force calculation).
; E32C31RM scales J22 term appropriately for body (Earth vs Moon).

		DDV	VXSC		# IF X2 = 0, DIVISION GIVES B53, VXSC
					# OUT OF 8D B5 GIVES B58
; Double divide: E32C31RM / normalized ALPHAM gives final J22 coefficient.
; If X2 = 0 (no normalization shift), division yields scaling B53.
; Vector scale by this coefficient (vector from 8D scaled B5).
; Result: J22 perturbation force vector scaled B58.

		VSL*	VAD		# SHIFT MAKES B61, FOR ADDITION OF
					# VECTOR IN 2D
			0	-3,2
; Variable shift left by (0 - 3) indexed by X2: compensates normalization.
; Shift adjusts scaling to B61 (matches J2/J3/J4 terms in pushdown 2D).
; Vector add to term in pushdown 2D (accumulated oblateness forces).
; All J-terms (J2, J3, J4, J22) now combined into single vector.

		VSL*	V/SC		# OPERAND FROM 0D.  B108 FOR X1 = 0
			0	-27D,1	# FOR X1 = 0, MAKES B88, GIVING B-20
					# FOR RESULT.
; Variable shift left by (0 - 27) indexed by X1: final magnitude scaling.
; Vector scale divide by operand from pushdown 0D (normalizing factor).
; For X1 = 0: shifts to B88, then division gives result scaled B-20.
; B-20 is standard force scaling (m²/s²) for integration routine.

		PDDL	PDDL
			TET
			5/8		# ANY NON-ZERO CONSTANT
; Push complete J-terms force vector to pushdown 0D.
; Load TET (time of state vector, mission elapsed time).
; Push TET to pushdown 6D.
; Load 5/8 (non-zero constant for subroutine parameter).
; Prepares arguments for coordinate transformation back to inertial frame.

		LXA,2	CALL		# POSITION IN 0D, TIME IN 6D.  X2 LEFT
					# ALONE.
			PBODY
			RP-TO-R
; Load X2 from PBODY (planet body indicator: 0=Earth, 2=Moon).
; Call RP-TO-R (in CONIC_SUBROUTINES.agc): rotate position rotating→inertial.
; Position vector in pushdown 0D, time in pushdown 6D.
; X2 preserved (indicates which body's perturbations were computed).
; Returns transformed position for next integration step.

		VAD	BOV		# OVERFLOW INDICATOR RESET IN "RP-TO-R"
			FV
			GOBAQUE
; Vector add to FV (total force accumulator).
; FV now includes two-body + all oblateness perturbations (J2+J3+J4+J22).
; Branch on overflow to GOBAQUE (error recovery for numerical overflow).
; Overflow indicator was reset in RP-TO-R subroutine.
# Page 1237
		STORE	FV

; ============================================================================
; NBRANCH - Differential Equation Branch Logic
;
; COMMENT-ONLY READERS: After accumulating all perturbation forces, the
; guidance computer must now integrate forward using different equation sets
; depending on mission phase. This branching logic selects the appropriate
; integration technique (conic, precision, encke method variations).
;
; CODE-ALONG READERS: This routine implements computed GOTO based on
; DIFEQCNT (differential equation counter). The index determines which
; integration routine to invoke next. The -1/12 scaling factor relates to
; timestep subdivision used in predictor-corrector integration.
; ============================================================================

NBRANCH		SLOAD	LXA,1
			DIFEQCNT
			MPAC
; Single-load DIFEQCNT (differential equation counter: 0, 1, or 2).
; Load index register X1 from MPAC (contains same value).
; DIFEQCNT tracks which integration pass we're on in predictor-corrector.

		DMP	CGOTO
			-1/12
			MPAC
			DIFEQTAB
; Double-multiply DIFEQCNT by -1/12 (related to timestep subdivision).
; Computed GOTO using index from MPAC and address table DIFEQTAB.
; Branches to DIFEQ+0, DIFEQ+1, or DIFEQ+2 based on DIFEQCNT value.
; Each entry point handles different stage of multi-pass integration.
; ============================================================================
; COSPHIE - Cosine Phi Computation
;
; Computes cos(phi/2) where phi is latitude angle. Used in oblateness
; calculations when vehicle is in highly inclined orbit and latitude effects
; become significant. Loads value and branches to common term computation.
; ============================================================================

COSPHIE		DLOAD
			ALPHAV 	+4
; Double-load ALPHAV+4 (fifth component of ALPHAV vector).
; Contains cos(phi/2) pre-computed for current position.

		STOVL	COSPHI/2
			ZUNIT
; Store to COSPHI/2 (half-angle cosine used in oblateness force calculation).
; Vector-load ZUNIT (unit vector in Z-direction of reference frame).

		GOTO
			COMTERM
; GOTO COMTERM (common terms routine for completing oblateness computation).
; This path taken when latitude-dependent oblateness terms are needed.

; ============================================================================
; DIFEQTAB - Differential Equation Address Table
;
; Table of code addresses for computed GOTO in NBRANCH. Contains three
; entry points corresponding to predictor-corrector integration stages:
; DIFEQ+0: Initial predictor pass
; DIFEQ+1: Corrector pass
; DIFEQ+2: Final corrector/convergence check
; ============================================================================

DIFEQTAB	CADR	DIFEQ+0
		CADR	DIFEQ+1
		CADR	DIFEQ+2

; ============================================================================
; TIMESTEP - Integration Timestep and Rectification Logic
;
; COMMENT-ONLY READERS: During long coast phases (translunar, transearth),
; the guidance computer must periodically check integration accuracy. If
; accumulated errors grow too large, a "rectification" resets the integration
; from current position. This routine also detects when spacecraft crosses
; from Earth's to Moon's gravitational sphere of influence (or vice versa).
;
; CODE-ALONG READERS: Implements multiple checks:
; 1. Dot product R.V test for sign changes (periapsis/apoapsis crossings)
; 2. Sphere-of-influence boundary detection (Earth vs Moon primary body)
; 3. Rectification trigger based on TDELTAV magnitude threshold
; Each condition requires special handling to maintain integration accuracy.
; ============================================================================

TIMESTEP	BOF	VLOAD
			MIDFLAG
			RECTEST
			RCV
; Branch on MIDFLAG off to RECTEST (skip sphere check if mid-integration).
; Vector-load RCV (current position vector relative to central body).
; MIDFLAG indicates whether this is midpoint evaluation in integration step.

		DOT	DMP
			VCV
			DT/2		# (R.V) X (DELTA T)
; Dot product: RCV · VCV (position · velocity, detects orbit quadrant).
; Double-multiply by DT/2 (half timestep).
; Result: (R·V) × (ΔT/2) used to detect periapsis/apoapsis passage.
; Sign change indicates orbit extremum crossed during this timestep.

		BMN
			RECTEST
; Branch on minus to RECTEST (negative dot product detected).
; Negative R·V means spacecraft approaching periapsis (inbound trajectory).
; Skip sphere-of-influence checks when in critical trajectory phase.
		BON	BOF
			MOONFLAG
			LUNSPH
			RPQFLAG
			EARSPH
; Branch on MOONFLAG to LUNSPH (currently in Moon's sphere of influence).
; Branch on RPQFLAG off to EARSPH (not yet computed Moon position this step).
; These flags track which body is currently primary for integration.
; Sphere-of-influence switching critical during translunar/transearth coast.

		DLOAD	CALL
			TET
			LSPOS		# RPQV IN MPAC
; Double-load TET (time of state vector, mission elapsed time).
; Call LSPOS (lunar/solar position routine): computes Moon position vector.
; Returns RPQV (position of vehicle relative to Moon) in MPAC.
; Needed to check if spacecraft has crossed sphere-of-influence boundary.

		STORE	RPQV		# RPQV
		LXA,2
			PBODY
; Store RPQV (relative position to secondary body).
; Load index register X2 from PBODY (0=Earth primary, 2=Moon primary).
; X2 used to index body-dependent parameters in sphere boundary check.

INLUNCHK	BVSU	ABVAL
			RCV
; Vector subtract: RPQV - RCV (forms position vector to opposite body).
; Absolute value: magnitude of distance to secondary body.
; INLUNCHK entry point: checks if inside lunar sphere of influence.

		DSU	BMN
			RSPHERE
			DOSWITCH
; Double-subtract RSPHERE (sphere-of-influence radius for current body).
; Branch on minus to DOSWITCH (inside opposite body's sphere).
; Sphere boundary crossed: must switch primary body for integration.
; Critical event during Apollo 11 translunar/transearth trajectories.
; ============================================================================
; RECTEST - Rectification Test
;
; Checks whether accumulated integration errors require rectification.
; Rectification recomputes conic reference orbit from current state,
; resetting Encke method deviation variables to zero. This prevents error
; accumulation during long coast phases when perturbations are small but
; numerous integration steps accumulate truncation errors.
; ============================================================================

RECTEST		VLOAD	ABVAL		# RECTIFY IF
			TDELTAV
; Vector-load TDELTAV (accumulated velocity deviation from conic reference).
; Absolute value: magnitude of total velocity error accumulated.
; TDELTAV grows as integration proceeds, tracks error in Encke method.

		BOV
			CALLRECT
; Branch on overflow to CALLRECT (force rectification).
; Overflow indicates TDELTAV exceeded representable range (catastrophic error).
; Immediate rectification required to prevent numerical instability.

		DSU	BPL		#	1) EITHER TDELTAV OR TNUV EQUALS OR
			3/4		#	   EXCEEDS 3/4 IN MAGNITUDE
			CALLRECT	#
; Double-subtract 3/4 (threshold = 0.75 meters/second for velocity error).
; Branch on plus to CALLRECT (magnitude exceeds threshold, rectify now).
; Rectification criterion 1: TDELTAV magnitude ≥ 3/4 m/s indicates
; Encke deviation vector has grown too large for accurate integration.
		DAD	SL*		#			OR
# Page 1238
			3/4		#
			0 	-7,2	#	2) ABVAL(TDELTAV) EQUALS OR EXCEEDS
; Double-add 3/4 (restore original TDELTAV magnitude to accumulator).
; Shift left by (0-7) scaled by index X2 (body-dependent alignment).
; Prepares for relative error check against position magnitude.

		DDV	DSU		#	   .01(ABVAL(RCV))
			10D
			RECRATIO
; Double-divide by 10D (magnitude of position vector RCV at 10D).
; Double-subtract RECRATIO (rectification ratio threshold = 0.01).
; Rectification criterion 2: |TDELTAV| ≥ 0.01 × |RCV| means velocity
; error is 1% of position-based characteristic velocity, too large.
		BPL	VLOAD
			CALLRECT
			TNUV
; Branch on plus to CALLRECT (relative error exceeds 1%, rectify).
; Vector-load TNUV (accumulated position deviation from conic reference).
; Now check position error magnitude against same thresholds.

		ABVAL	DSU
			3/4
; Absolute value: magnitude of TNUV position deviation.
; Double-subtract 3/4 (same threshold as velocity, but position units).
; Position error check parallels velocity error check above.
		BOV
			CALLRECT
; Branch on overflow to CALLRECT (TNUV magnitude overflow).
; Numerical overflow in position deviation = catastrophic error state.

		BMN
			INTGRATE
; Branch on minus to INTGRATE (all rectification tests passed).
; TNUV magnitude < 3/4 threshold: Encke deviations still acceptable.
; Continue integration without rectification, maintaining current reference.
; ============================================================================
; RECTIFICATION EXECUTION
;
; Entry point when rectification tests determine Encke deviations exceed
; acceptable thresholds. Updates conic reference orbit to current state,
; resets deviation vectors to zero, and continues integration from new base.
; ============================================================================

CALLRECT	CALL
			RECTIFY
; Call RECTIFY routine (in INTEGRATION_INITIALIZATION.agc).
; Updates reference conic orbit to current perturbed state.
; Resets TNUV and TDELTAV deviation vectors to zero after absorbing errors.

; ============================================================================
; INTEGRATION VARIABLE SETUP
;
; Prepares state vectors for differential equation integration. Loads
; accumulated deviations into working registers, clears integration switch
; flags, and proceeds to DIFEQ0 differential equation computation entry point.
; ============================================================================

INTGRATE	VLOAD
			TNUV
; Vector-load TNUV (position deviation from conic reference).
; This is the accumulated perturbation error in position coordinates.

		STOVL	ZV
			TDELTAV
; Store to ZV (position deviation working register for integration).
; Vector-load TDELTAV (velocity deviation from conic reference).

		STORE	YV
; Store to YV (velocity deviation working register for integration).
; YV and ZV are the state deviation vectors integrated forward in time.

		CLEAR
			JSWITCH
; Clear JSWITCH flag (integration direction control).
; JSWITCH = 0: forward integration (normal mode).
; JSWITCH = 1: backward integration (time-reversed for special cases).
; ============================================================================
; DIFFERENTIAL EQUATION ENTRY POINT (DIFEQ0)
;
; Initializes differential equation integration loop. Sets up velocity
; deviation as initial acceleration, initializes step counter, zeroes
; integration step accumulator. Branches based on JSWITCH to continue
; forward (ACCOMP) or backward (DOW..) integration direction.
; ============================================================================

DIFEQ0		VLOAD	SSP
			YV
			DIFEQCNT
			0
; Vector-load YV (velocity deviation vector).
; Set DIFEQCNT counter to 0 (differential equation iteration counter).
; Counts predictor-corrector cycles within each integration timestep.

		STODL	ALPHAV
			DPZERO
; Store to ALPHAV (acceleration vector for differential equation).
; Double-load DPZERO (initialize accumulator to zero).

		STORE	H		# START H AT ZERO.  GOES 0(DELT/2)DELT.
; Store to H (integration step accumulator, starts at zero).
; H progresses: 0 → DELTAT/2 → DELTAT (predictor-corrector sequence).
; Predictor uses half-step, corrector uses full step for accuracy.

		BON	GOTO
			JSWITCH
			DOW..
			ACCOMP
; Branch on JSWITCH flag set to DOW.. (backward integration direction).
; Goto ACCOMP (forward integration, normal operational mode).
; JSWITCH allows time-reversed integration for special trajectory analysis.
; ============================================================================
; EARTH SPHERE-OF-INFLUENCE CHECK (EARSPH)
;
; Entry point when spacecraft in Earth-centered coordinates. Loads secondary
; body (Moon) position vector and checks if spacecraft has entered lunar
; sphere of influence. If within lunar sphere, switches integration to
; Moon-centered coordinates for improved numerical accuracy near Moon.
; ============================================================================

EARSPH		VLOAD	GOTO
			RPQV
			INLUNCHK
; Vector-load RPQV (Moon position vector relative to Earth).
; Goto INLUNCHK (check if spacecraft position within lunar sphere).
; Earth-relative coordinates used until spacecraft approaches Moon.

; ============================================================================
; LUNAR SPHERE-OF-INFLUENCE CHECK (LUNSPH)
;
; Entry point when spacecraft in Moon-centered coordinates. Checks if
; spacecraft has exited lunar sphere of influence. If outside lunar sphere,
; switches integration to Earth-centered coordinates. Sphere radius determined
; by gravitational balance point between Earth and Moon.
; ============================================================================

LUNSPH		DLOAD	SR2
			10D
; Double-load 10D (magnitude of position vector |RCV|).
; Shift right 2 bits (divide by 4, aligns scaling for comparison).

		DSU	BMN
			RSPHERE
			RECTEST
; Double-subtract RSPHERE (sphere-of-influence radius for current body).
; Branch on minus to RECTEST (still within sphere, continue integration).
; Positive result: spacecraft has crossed sphere boundary, coordinate switch.
		BOF	DLOAD
			RPQFLAG
			DOSWITCH
			TET
; Branch on RPQFLAG off to DOSWITCH (RPQV needs updating).
; Double-load TET (current ephemeris time for Moon position calculation).
; RPQFLAG indicates if secondary body position vector is current.

		CALL
			LUNPOS
; Call LUNPOS (calculates Moon position at time TET).
; Returns Moon position vector in Earth-centered inertial coordinates.
; Required when RPQV has not been updated for current integration time.

		VCOMP
		STORE	RPQV
; Vector-complement (negate Moon position vector).
; Store to RPQV (updated secondary body position relative to primary).
; Sign convention: RPQV points from primary to secondary body center.

# Page 1239
; ============================================================================
; COORDINATE ORIGIN SWITCH (DOSWITCH)
;
; Executes coordinate transformation when spacecraft crosses sphere-of-
; influence boundary. Calls ORIGCHNG to transform state vectors from one
; body-centered frame to another, updates body index, rectifies orbit,
; and continues integration in new reference frame.
; ============================================================================

DOSWITCH	CALL
			ORIGCHNG
; Call ORIGCHNG (origin change coordinate transformation routine).
; Transforms RCV, VCV from current body center to other body center.
; Updates PBODY index, switches gravitational parameter MU.

		GOTO
			INTGRATE
; Goto INTGRATE (continue integration after coordinate transformation).
; Integration proceeds in new body-centered frame with updated reference.
; ============================================================================
; COORDINATE ORIGIN CHANGE (ORIGCHNG)
;
; Transforms spacecraft state vectors between body-centered coordinate frames
; when crossing sphere-of-influence boundary. Executes complete coordinate
; transformation: converts position/velocity from one body center to another,
; updates body index (PBODY), sets appropriate flags, and rectifies orbit in
; new frame. Critical for maintaining numerical accuracy during lunar approach,
; orbit, and departure phases of Apollo missions.
; ============================================================================

ORIGCHNG	STQ	CALL
			ORIGEX
			RECTIFY
; Store return address to ORIGEX (origin change exit address).
; Call RECTIFY (establish new conic reference in current frame before switch).
; Rectification ensures clean initial conditions in new coordinate system.

		VLOAD	VSL*
			RCV
			0,2
; Vector-load RCV (current position vector in old body-centered frame).
; Variable shift left by (0) scaled by X2 (body-dependent scaling alignment).
; X2 contains index for current central body (0=Earth, 2=Moon).

		VSU	VSL*
			RPQV
			2,2
; Vector-subtract RPQV (secondary body position relative to primary).
; Variable shift left by (2) scaled by X2 (additional alignment shift).
; Transformation: R_new = R_old - R_secondary (parallel axis theorem).

		STORE	RRECT
		STODL	RCV
			TET
; Store to RRECT (rectified position in new body-centered frame).
; Store to RCV (update current position vector for new central body).
; Double-load TET (current time for calculating secondary body velocity).
		CALL
			LUNVEL
; Call LUNVEL (calculate Moon velocity vector at current time TET).
; Returns Moon velocity in Earth-centered inertial frame (km/centisecond).
; Required for transforming spacecraft velocity to new reference frame.

		BOF	VCOMP
			MOONFLAG
			+1
; Branch on MOONFLAG off (Earth→Moon switch) to skip vector complement.
; Vector-complement (negate Moon velocity if MOONFLAG set, Moon→Earth switch).
; Sign convention: velocity subtracted matches direction of coordinate change.

		PDVL	VSL*
			VCV
			0,2
; Push double-precision (save Moon velocity to stack for later subtraction).
; Vector-load VCV (current velocity vector in old body-centered frame).
; Variable shift left by (0) scaled by X2 (body-dependent velocity scaling).

		VSU
		VSL*
			0 	+2,2
; Vector-subtract (VCV - V_secondary from stack).
; Variable shift left by (0,+2) scaled by X2 (final velocity alignment).
; Transformation: V_new = V_old - V_secondary (Galilean velocity addition).

		STORE	VRECT
		STORE	VCV
; Store to VRECT (rectified velocity in new body-centered frame).
; Store to VCV (update current velocity vector for new central body).
; Position and velocity now fully transformed to new coordinate origin.
		LXA,2	SXA,2
			ORIGEX
			QPRET
; Load index X2 from ORIGEX (restore return address to index register).
; Store index X2 to QPRET (save return address for final exit).
; Register management preparing for flag operations and return sequence.

		BON	GOTO
			MOONFLAG
			CLRMOON
			SETMOON
; Branch on MOONFLAG on (Moon→Earth switch) to CLRMOON routine.
; Go to SETMOON (Earth→Moon switch, flag not set).
; Flag state determines which cleanup operations execute after origin change.
# Page 1240
# THE RECTIFY SUBROUTINE IS CALLED BY THE INTEGRATION PROGRAM AND OCCASIONALLY BY THE MEASUREMENT INCORPORATION
# ROUTINES TO ESTABLISH A NEW CONIC.

; ============================================================================
; RECTIFY SUBROUTINE
;
; Establishes new conic reference by computing rectified state vectors from
; current encke-method state. Called by integration program when accumulated
; perturbations exceed threshold (RECTEST), and by measurement incorporation
; routines after navigation updates. Converts delta-position/delta-velocity
; (TDELTAV, TNUV) back to absolute position/velocity (RRECT/VRECT, RCV/VCV),
; then zeros the delta terms. Essential for maintaining numerical precision
; during long coast periods in cislunar and lunar orbit phases.
; ============================================================================

RECTIFY		LXA,2	VLOAD
			PBODY
			TDELTAV
; Load index X2 from PBODY (central body index: 0=Earth, 2=Moon).
; Vector-load TDELTAV (accumulated delta-position from reference conic).
; Delta represents perturbation accumulation since last rectification.
		VSL*	VAD
			0 	-7,2
			RCV
; Variable shift left by (0,-7) scaled by X2 (scale TDELTAV to match RCV units).
; Vector-add RCV (add delta-position to current position vector).
; Computes: R_rectified = R_current + scaled_delta_R (absolute position).

		STORE	RRECT
		STOVL	RCV
			TNUV
; Store to RRECT (rectified position vector, new conic reference).
; Store to RCV (update current position for subsequent integration).
; Vector-load TNUV (accumulated delta-velocity from reference conic).
; Now processing velocity rectification parallel to position.
		VSL*	VAD
			0 	-4,2
			VCV
; Variable shift left by (0,-4) scaled by X2 (scale TNUV to match VCV units).
; Vector-add VCV (add delta-velocity to current velocity vector).
; Computes: V_rectified = V_current + scaled_delta_V (absolute velocity).

MINIRECT	STORE	VRECT
		STOVL	VCV
			ZEROVEC
; MINIRECT label: Entry point for minimal rectification (position/velocity only).
; Store to VRECT (rectified velocity vector, new conic reference).
; Store to VCV (update current velocity for subsequent integration).
; Vector-load ZEROVEC (all-zeros vector for clearing delta accumulators).
; Rectified state established: now clear perturbation accumulators.
		STORE	TDELTAV
		STODL	TNUV
			ZEROVEC
; Store ZEROVEC to TDELTAV (clear accumulated delta-position to zero).
; Store to TNUV (clear accumulated delta-velocity to zero).
; Double-load ZEROVEC (prepare to clear time/parameter accumulators).
; Delta accumulators reset: new reference conic established.

		STORE	TC
		STORE	XKEP
; Store ZEROVEC to TC (clear time-since-conic parameter).
; Store ZEROVEC to XKEP (clear Kepler iteration parameter).
; All perturbation tracking variables reset to zero baseline.

		RVQ
; Return via Q register (exit RECTIFY subroutine).
; Rectification complete: spacecraft state on new unperturbed conic reference.
; Encke method precision restored for next integration cycle.

# Page 1241
# THE THREE DIFEQ ROUTINES -- DIFEQ+0, DIFEQ+12, DIFEQ+24 -- ARE ENTERED TO PROCESS THE CONTRIBUTIONS AT THE
# BEGINNING, MIDDLE, AND END OF THE TIMESTEP, RESPECTIVELY.  THE UPDATING IS DONE BY THE NYSTROM METHOD.

; ============================================================================
; DIFEQ INTEGRATION ROUTINES (Nyström Method Implementation)
;
; Three entry points process gravitational acceleration contributions at
; different timestep phases for predictor-corrector numerical integration:
; - DIFEQ+0:  Beginning of timestep (initial acceleration evaluation)
; - DIFEQ+12: Middle of timestep (midpoint predictor evaluation)
; - DIFEQ+24: End of timestep (corrector evaluation and state update)
;
; Implements Nyström integration method for second-order differential equation
; of orbital motion: d²r/dt² = -μ/r³ · r + perturbations. Higher accuracy than
; simple Euler method, essential for multi-day cislunar coast trajectories.
; ============================================================================

DIFEQ+0		VLOAD	VSR3
			FV
; DIFEQ+0 entry: Process beginning-of-timestep acceleration contribution.
; Vector-load FV (gravitational acceleration vector from OBLATE computation).
; Vector shift right 3 (divide by 8 for Nyström weighting coefficient).
; Initial acceleration evaluation sets baseline for predictor step.

		STCALL	PHIV
			DIFEQCOM
; Store to PHIV (save scaled acceleration as phi-vector for later use).
; Call DIFEQCOM (common integration update subroutine, updates position/velocity).
; Return continues to DIFEQ+1 for midpoint predictor evaluation.

DIFEQ+1		VLOAD	VSR1
			FV
; DIFEQ+1 entry: Process midpoint-of-timestep acceleration (predictor step).
; Vector-load FV (gravitational acceleration at predicted midpoint position).
; Vector shift right 1 (divide by 2 for midpoint weighting).
; Midpoint acceleration used for improved trajectory prediction.

		PUSH	VAD
			PHIV
; Push (save FV/2 to stack for later averaging).
; Vector-add PHIV (add beginning-of-step acceleration contribution).
; Computes: phi_mid = phi_0 + a_mid/2 (accumulated acceleration).

		STOVL	PSIV
		VSR1	VAD
			PHIV
; Store to PSIV (save psi-vector for corrector step).
; Vector-load (retrieve FV/2 from stack for alternative combination).
; Vector shift right 1 (divide by 2 again: now FV/4).
; Vector-add PHIV (combine with initial acceleration).

		STCALL	PHIV
			DIFEQCOM
; Store to PHIV (update phi-vector with midpoint contribution).
; Call DIFEQCOM (update position/velocity with midpoint predictor).
; Nyström predictor step complete, prepares for corrector evaluation.
DIFEQ+2		DLOAD	DMPR
			H
			DP2/3
; DIFEQ+2 entry: Process end-of-timestep acceleration (corrector step).
; Double-load H (integration timestep, centiseconds).
; Double-multiply by DP2/3 (2/3 constant for Nyström corrector weighting).
; Computes: (2/3) * h for position update scaling.

		PUSH	VXSC
			PHIV
; Push (save (2/3)h to stack for later velocity update).
; Vector cross-scale PHIV (multiply phi-vector by (2/3)h).
; Computes: (2/3)h * phi (position correction from acceleration).

		VSL1	VAD
			ZV
; Vector shift left 1 (multiply by 2: now (4/3)h * phi).
; Vector-add ZV (add to accumulated Z-vector position updates).
; Computes: Z_new = Z_old + (4/3)h * phi (Nyström position update).

		VXSC	VAD
			H
			YV
; Vector cross-scale (multiply by h for velocity scaling).
; Vector-add YV (add velocity update to Y-vector).
; Computes: Y_new = Y_old + h * velocity_update.

		STOVL	YV
			FV
; Store to YV (updated Y-vector, intermediate velocity state).
; Vector-load FV (end-of-timestep acceleration from current OBLATE call).
		VSR3	VAD
			PSIV
; Vector shift right 3 (divide FV by 8 for corrector weighting).
; Vector-add PSIV (add to psi-vector from midpoint evaluation).
; Computes: psi + a_end/8 (final corrector acceleration term).

		VXSC	VSL1
; Vector cross-scale (multiply by (2/3)h from stack).
; Vector shift left 1 (multiply by 2: now (4/3)h).
; Scales final acceleration contribution for velocity update.

		VAD
			ZV
; Vector-add ZV (add final velocity correction to Z-vector).
; Computes: Z_final = Z + (4/3)h * corrector_term.

		STORE	ZV
; Store to ZV (Z-vector updated with end-of-timestep corrections).
; Nyström corrector step complete: Z contains full position update.

		BOFF	CALL
			JSWITCH
			ENDSTATE
			GRP2PC
; Branch-off (if JSWITCH flag clear) to ENDSTATE (finish integration step).
; Call GRP2PC (if JSWITCH set: process position/velocity coordinate update).
; JSWITCH set during permanent state vector storage at major timestep boundaries.
; GRP2PC handles state vector column storage for restart protection.

		LXA,2	VLOAD
			COLREG
			ZV
; Load index X2 from COLREG (column register: W-matrix storage index).
; Vector-load ZV (final position update vector from Nyström integration).
; Preparing to store position update in W-matrix for later retrieval.

		VSL3			# ADJUST W-POSITION FOR STORAGE
		STORE	W 	+54D,2
; Vector shift left 3 (scale position for W-matrix storage format).
; Store to W+54D indexed by X2 (store position in second column of W-matrix).
; W-matrix stores position and velocity updates for state reconstruction.

		VLOAD
			YV
; Vector-load YV (final velocity update vector from Nyström integration).
; Preparing to store velocity update in W-matrix.

		VSL3	BOV
			WMATEND
		STORE	W,2
; Vector shift left 3 (scale velocity for W-matrix storage format).
; Branch-on-overflow to WMATEND (handle potential scaling overflow).
; Store to W indexed by X2 (store velocity in first column of W-matrix).
; Position and velocity updates stored: state vector updates saved.

		CALL
			GRP2PC
; Call GRP2PC again (store updated column data for restart protection).
; Multiple GRP2PC calls ensure state vector consistency across restart boundaries.

# Page 1242
		LXA,2	SSP
			COLREG
			S2
			0
; Load index X2 from COLREG (current column index).
; Single-precision store 0 to S2 (initialize loop counter).
; Preparing for potential state vector reload loop.

		INCR,2	SXA,2
			6
			YV
; Increment X2 by 6 (advance to next column index: position/velocity pairs).
; Store X2 to YV (save updated column index temporarily).

		TIX,2	CALL
			RELOADSV
			GRP2PC
; Test-index-and-skip: decrement X2, skip if zero (no reload needed).
; Otherwise call RELOADSV (reload temporary state vector from permanent).
; Then call GRP2PC (process group storage).

		LXA,2	SXA,2
			YV
			COLREG
; Load index X2 from YV (retrieve saved column index).
; Store X2 to COLREG (update column register for next processing cycle).
; Column management complete: ready for next integration column.

; ============================================================================
; NEXTCOL - Process Next Integration Column
;
; Retrieves position and velocity updates from W-matrix for next integration
; column. During precision orbit propagation, multiple state vector columns
; are maintained for restart protection. This routine loads the next column's
; position/velocity data and continues the integration loop.
; ============================================================================

NEXTCOL		CALL
			GRP2PC
; Call GRP2PC (store current column data for restart protection).

		LXA,2	VLOAD*
			COLREG
			W,2
; Load index X2 from COLREG (column register: next column index).
; Vector-load from W indexed by X2 (retrieve velocity update from W-matrix).

		VSR3			# ADJUST W-POSITION FOR INTEGRATION
		STORE	YV
; Vector shift right 3 (scale velocity back to integration format).
; Store to YV (Y-vector: velocity update loaded for next integration).

		VLOAD*	AXT,1
			W 	+54D,2
			0
; Vector-load from W+54D indexed by X2 (retrieve position update from W-matrix).
; Set index X1 to 0 (initialize for DIFEQ routine).

		VSR3			# ADJUST W-VELOCITY FOR INTEGRATION
		STCALL	ZV
			DIFEQ0
; Vector shift right 3 (scale position back to integration format).
; Store to ZV and call DIFEQ0 (continue integration with loaded state).
; Next column integration begins: state vector reloaded from W-matrix.

; ============================================================================
; ENDSTATE - Complete Integration Step
;
; Final processing after Nyström integration step completes. Stores final
; position and velocity updates to temporary state vector storage (TNUV,
; TDELTAV). Checks integration flags to determine next action: continue
; time increment, check for mid-course update, or loop for additional
; precision columns. Manages phase changes for restart protection.
; ============================================================================

ENDSTATE	BOV	VLOAD
			GOBAQUE
			ZV
; Branch-on-overflow to GOBAQUE (handle integration overflow condition).
; Vector-load ZV (final position update from Nyström corrector).

		STOVL	TNUV
			YV
; Store to TNUV (temporary new position update vector).
; Vector-load YV (final velocity update from Nyström corrector).

		STORE	TDELTAV
; Store to TDELTAV (temporary delta-velocity vector).
; Position and velocity updates saved: integration step complete.

		BON	BOFF
			MIDAVFLG
			CKMID2		# CHECK FOR MID2 BEFORE GOING TO TIMEINC
			DIM0FLAG
			TESTLOOP
; Branch-on: if MIDAVFLG set, goto CKMID2 (check mid-course average flag).
; Branch-off: if DIM0FLAG clear, goto TESTLOOP (continue precision loop).
; Flag checks determine integration continuation vs. time increment.

		EXIT
		TC	PHASCHNG
		OCT	04022		# PHASE 1
; Exit interpreter mode (return to native AGC code).
; Transfer control to PHASCHNG (phase change for restart protection).
; Octal 04022: Phase 1 restart group identifier.

		TC	UPFLAG		# PHASE CHANGE HAS OCCURRED BETWEEN
		ADRES	REINTFLG	# INSTALL AND INTWAKE
; Transfer control to UPFLAG (set restart integration flag).
; Address REINTFLG (re-integration flag: marks integration interrupted).
; Ensures proper restart if power interruption occurs during integration.

		TC	INTPRET
		SSP
			QPRET
			AMOVED
; Transfer control to INTPRET (re-enter interpreter mode).
; Single-precision store AMOVED address to QPRET (set return address).

		BON	GOTO
			VINTFLAG
# Page 1243
			ATOPCSM
			ATOPLEM
; Branch-on: if VINTFLAG set, goto ATOPCSM (vehicle integration: CSM mode).
; Otherwise goto ATOPLEM (vehicle integration: LM mode).
; Vehicle flag determines spacecraft-specific integration continuation.
AMOVED		SET	SSP
			JSWITCH
			COLREG
		DEC	-30
; Set JSWITCH flag (enable state vector column storage on next pass).
; Single-precision store -30 to COLREG (set column index for 6-column mode).
; Decimal -30: column register initialization for standard integration.

		BOFF	SSP
			D6OR9FLG
			NEXTCOL
			COLREG
		DEC	-48
; Branch-off: if D6OR9FLG clear, goto NEXTCOL (use 6-column mode).
; Otherwise single-precision store -48 to COLREG (set for 9-column mode).
; Decimal -48: extended column register for high-precision integration.

		GOTO
			NEXTCOL
; Go to NEXTCOL (begin next column integration cycle).
; Column mode selected: continue multi-column precision integration.

; ============================================================================
; RELOADSV - Reload Temporary State Vector from Permanent
;
; Restores temporary state vector from permanent storage when integration
; needs to restart. Called when processing multiple columns during precision
; orbit propagation. Ensures state vector consistency by reloading TDEC1
; from TDEC and continuing integration from INTEGRV2 entry point.
; ============================================================================

RELOADSV	DLOAD			# RELOAD TEMPORARY STATE VECTOR
			TDEC		# FROM PERMANENT IN CASE OF
; Double-precision load TDEC (permanent time of state vector).
; Preparing to reload temporary state from permanent storage.

		STCALL	TDEC1
			INTEGRV2	# BY STARTING AT INTEGRV2.
; Store to TDEC1 and call INTEGRV2 (temporary time, restart integration).
; INTEGRV2 entry point reloads full state vector from permanent storage.
; State vector restored: integration continues from known good state.

; ============================================================================
; DIFEQCOM - Common Differential Equation Processing
;
; Performs common calculations after differential equation evaluation.
; Increments integration step size H and DIFEQCNT counter, then computes
; ALPHAV intermediate vector for Nystrom integration. This routine is shared
; by multiple integration paths to avoid code duplication.
; ============================================================================

DIFEQCOM	DLOAD	DAD		# INCREMENT H AND DIFEQCNT.
			DT/2
			H
; Double-precision load DT/2 (half integration timestep).
; Double-precision add H (current integration step accumulator).
; Incrementing H by DT/2 for next Nystrom integration substep.

		INCR,1	SXA,1
		DEC	-12
			DIFEQCNT	# DIFEQCNT SET FOR NEXT ENTRY.
; Increment X1 by -12 (update differential equation counter).
; Store X1 to DIFEQCNT (save updated counter for next DIFEQ entry).
; Counter tracks position within Nystrom multi-step integration cycle.

		STORE	H
; Store updated H (accumulated integration step size).

		VXSC	VSR1
			FV
; Vector multiply by scalar FV (force vector scaled).
; Vector shift right 1 (adjust scaling for intermediate calculation).

		VAD	VXSC
			ZV
			H
; Vector add ZV (position update vector).
; Vector multiply by scalar H (scale by accumulated step size).

		VAD
			YV
; Vector add YV (velocity update vector).

		STORE	ALPHAV
; Store to ALPHAV (intermediate alpha vector for Nystrom integration).
; ALPHAV used in subsequent integration steps to compute state updates.

		BON	GOTO
			JSWITCH
			DOW..
			FBR3
; Branch-on: if JSWITCH set, goto DOW.. (W-matrix extrapolation).
; Otherwise goto FBR3 (continue standard integration path).
; JSWITCH determines whether to process W-matrix columns or state vector.

; ============================================================================
; WMATEND - W-Matrix Integration Termination
;
; Called when W-matrix integration encounters problems or completion.
; Clears W-matrix integration flags (DIM0FLAG, ORBWFLAG, RENDWFLG) to
; disable further W-matrix processing. Sets STATEFLG to redirect integration
; to state vector update only. Issues alarm 421 to notify crew of W-matrix
; computation abort. Returns to TESTLOOP to complete state vector integration.
; ============================================================================

WMATEND		CLEAR	CLEAR
			DIM0FLAG	# DON'T INTEGRATE W THIS TIME
			ORBWFLAG	# INVALIDATE W
; Clear DIM0FLAG (disable initial column W-matrix integration).
; Clear ORBWFLAG (invalidate W-matrix, mark as unreliable).
; W-matrix integration terminated: switching to state vector only.

		CLEAR
			RENDWFLG
; Clear RENDWFLG (disable rendezvous W-matrix processing).

		SET	EXIT
			STATEFLG	# PICK UP STATE VECTOR UPDATE
; Set STATEFLG (redirect to state vector integration path).
; Exit interpreter mode to issue crew alarm.

		TC	ALARM
		OCT	421
; Transfer control to ALARM routine with alarm code 421.
; Alarm 421: W-matrix integration problem, navigation accuracy degraded.
; Crew notified: orbit determination using W-matrix unavailable.

		TC	INTPRET
; Transfer control back to interpreter mode.

# Page 1244
		GOTO
			TESTLOOP	# FINISH INTEGRATING STATE VECTOR
; Goto TESTLOOP (continue with state vector integration only).
; W-matrix abandoned: mission continues with reduced navigation precision.

# Page 1245
# ORBITAL ROUTINE FOR EXTRAPOLATION OF THE W MATRIX.  IT COMPUTES THE SECOND DERIVATIVE OF EACH COLUMN POSITION
# VECTOR OF THE MATRIX AND CALLS THE NYSTROM INTEGRATION ROUTINES TO SOLVE THE DIFFERENTIAL EQUATIONS.  THE PROGRAM
# USES A TABLE OF VEHICLE POSITION VECTORS COMPUTED DURING THE INTEGRATION OF THE VEHICLE'S POSITION AND VELOCITY.

; ============================================================================
; DOW.. - W-Matrix Differential Equation Computation
;
; Computes second derivative of each W-matrix column position vector for
; Nystrom integration. Uses table of vehicle position vectors (VECTAB)
; computed during state vector integration. Processes perturbation forces
; from primary body and (if cislunar) secondary body. Critical for orbit
; determination accuracy and navigation state covariance propagation.
; ============================================================================

DOW..		LXA,2	DLOAD*
			PBODY
			MUEARTH,2
; Load index X2 from PBODY (primary body selector: 0=Earth, 2=Moon).
; Double-precision load MUEARTH,2 indexed (gravitational parameter of primary).

		STCALL	BETAM
			DOW..1
; Store to BETAM (gravitational parameter for W-matrix calculation).
; Call DOW..1 subroutine (compute W-matrix acceleration from primary body).

		STORE	FV
; Store result to FV (force vector from primary body gravity).

		BOF	INCR,1
			MIDFLAG
			NBRANCH
		DEC	-6
; Branch-off: if MIDFLAG clear, goto NBRANCH (cislunar: need secondary body).
; Increment X1 by -6 (adjust VECTAB index for secondary body position).
; MIDFLAG clear in cislunar trajectory: must include Moon/Earth perturbation.

		LXC,2	DLOAD*
			PBODY
			MUEARTH -2,2
; Load complement of X2 from PBODY (switch primary/secondary: 2→0, 0→2).
; Double-precision load MUEARTH -2,2 (gravitational parameter of secondary).
; If PBODY=0 (Earth primary) then X2=2 (Moon secondary), vice versa.

		STCALL	BETAM
			DOW..1
; Store to BETAM (gravitational parameter of secondary body).
; Call DOW..1 (compute W-matrix acceleration from secondary body gravity).

		BON	VSR6
			MOONFLAG
			+1
; Branch-on: if MOONFLAG set (Moon primary), skip scaling adjustment.
; Vector shift right 6 (scale secondary body force for Earth primary case).

		VAD
			FV
; Vector add FV (combine primary and secondary body perturbations).

		STCALL	FV
			NBRANCH
; Store to FV (total force vector on W-matrix column).
; Call NBRANCH (continue Nystrom integration with combined forces).

; ============================================================================
; DOW..1 - W-Matrix Column Acceleration Computation Subroutine
;
; Computes gravitational acceleration on one W-matrix column position vector.
; Uses W-matrix column from VECTAB and spacecraft position (ALPHAV) to
; compute relative position, then applies inverse-square gravity law with
; specified gravitational parameter (BETAM). Returns acceleration vector
; scaled for Nystrom integration. Called separately for primary and secondary
; body perturbations in cislunar trajectories.
;
; The W-matrix represents sensitivity of state vector to initial conditions.
; Accurate W-matrix propagation enables precise orbit determination and
; navigation covariance computation for midcourse corrections.
; ============================================================================

DOW..1		VLOAD	VSR4
			ALPHAV
; Vector load ALPHAV (spacecraft position for W-matrix calculation).
; Vector shift right 4 (scale spacecraft position for relative computation).

		PDVL*	UNIT
			VECTAB,1
; Push scaled ALPHAV to MPAC stack.
; Vector load VECTAB,1 indexed (W-matrix column position from table).
; Unit (convert to unit vector, magnitude stored implicitly).

		PDVL	VPROJ
			ALPHAV
; Push unit vector to stack.
; Vector load ALPHAV (spacecraft position).
; Vector projection (project ALPHAV onto unit direction).

		VXSC	VSU
			3/4
; Vector multiply by scalar 3/4 (scale projection).
; Vector subtract (compute relative position: column - spacecraft).

		PDDL	NORM
			36D
			S2
; Push relative position vector to stack (now in 36D).
; Double-precision load 36D (relative position magnitude calculation).
; Normalize (find magnitude, store exponent in S2 for scaling).

		PUSH	DSQ
; Push normalized magnitude to stack.
; Double square (compute r² for inverse-square law denominator).

		DMP
; Double multiply (form r² scaled properly).

		NORM	PDDL
			34D
			BETAM
; Normalize result, store exponent in 34D.
; Push normalized r² to stack.
; Double-precision load BETAM (gravitational parameter μ of body).

		SR1	DDV
; Shift right 1 (scale BETAM for division precision).
; Double divide (compute μ/r² = gravitational acceleration magnitude).

		VXSC
; Vector multiply by scalar (apply acceleration magnitude to unit vector).
; Result: acceleration = -μ/r² * (r_relative/|r_relative|).

		LXA,2	XAD,2
			S2
			S2
; Load index X2 from S2 (normalization exponent from position).
; Index add X2 by S2 (double exponent for r² scaling compensation).

		XAD,2	XAD,2
			S2
			34D
; Index add X2 by S2 (triple exponent).
; Index add X2 by 34D (add normalization exponent from μ/r² division).
; Total scaling adjustment accounts for all normalization operations.

		VSL*	RVQ
# Page 1246
			0 	-8D,2
; Vector shift left indexed by 0 -8D,2 (apply computed scaling exponent).
; Return via Q register (acceleration vector ready for W-matrix integration).
; Returned acceleration used in Nystrom integration of W-matrix column.

# ********************************************************************************
# ********************************************************************************

; ============================================================================
; SETITCTR - Set Iteration Counter for Lambert Targeting
;
; Initializes ITERCTR (iteration counter) for Lambert problem solving in
; rendezvous navigation. Sets different iteration limits depending on whether
; average-g flag (AVEGFLAG) is set. Used by orbital targeting programs
; (P32-P35, P72-P75) for maneuver planning.
;
; NOTE: Original NASA comment indicates this code belongs in INITVEL module
; but was temporarily placed here for Luminary 099 one-module remanufacture.
; Scheduled to be moved back to INITVEL for Luminary 1B release.
;
; Lambert problem: Given two position vectors and transfer time, compute
; required velocity change for conic trajectory connecting the positions.
; Iterative solution converges to precision orbit for rendezvous targeting.
; ============================================================================

SETITCTR	SSP	BOFF		# SET ITERCTR FOR LAMBERT CALLS.  THIS
			ITERCTR		# CODING BELONGS IN INITVEL AND IS HERE
			20D		# FOR PURPOSES OF A ONE-MODULE
			AVEGFLAG	# REMANUFACTURE ONLY.  CODING SHOULD
			LAMBERT		# BE MOVED BACK TO INITVEL FOR LUMINARY 1B
; Set single precision to ITERCTR = 20 decimal (default iteration limit).
; Branch off (skip next instruction) if AVEGFLAG is clear.
; If AVEGFLAG clear (precision mode), use 20 iterations for convergence.
; Branch to LAMBERT to begin Lambert targeting computation.

		SSP	GOTO
			ITERCTR
			5
			LAMBERT
; Set single precision to ITERCTR = 5 decimal (reduced iteration limit).
; Goto LAMBERT (begin Lambert problem solution).
; If AVEGFLAG set (average-g approximation mode), use only 5 iterations.
; Faster convergence acceptable when using simplified averaging method.

# ********************************************************************************
# ********************************************************************************

; ============================================================================
; ORBITAL INTEGRATION CONSTANTS AND DATA DEFINITIONS
;
; This section defines mathematical constants, scaling factors, and memory
; location aliases used throughout the orbital integration routines. Constants
; are stored as double-precision (2DEC) values with scaling factors indicated
; by B-n notation (meaning multiply by 2^-n for actual value).
; ============================================================================

		SETLOC	ORBITAL1
		BANK

; Mathematical constants for numerical integration and guidance computations:

3/5		2DEC	.6 B-2
; 0.6 scaled by 2^-2 = 0.15 actual value. Used in Nystrom integration.

THREE/8		2DEC	.375
; 0.375 = 3/8 exactly. Step fraction for predictor-corrector integration.

.3D		2DEC	.3 B-2
; 0.3 scaled by 2^-2 = 0.075 actual value. Integration coefficient.

3/64		2DEC	3 B-6
; 3 scaled by 2^-6 = 0.046875 = 3/64 exactly. Higher-order term coefficient.

DP1/4		2DEC	.25
; 0.25 = 1/4 exactly. Quarter value for step divisions and averaging.

DQUARTER	EQUALS	DP1/4
POS1/4		EQUALS	DP1/4
; Aliases for DP1/4 used in different contexts for code readability.

3/32		2DEC	3 B-5
; 3 scaled by 2^-5 = 0.09375 = 3/32 exactly. Integration step coefficient.

15/16		2DEC	15. B-4
; 15 scaled by 2^-4 = 0.9375 = 15/16 exactly. Near-unity coefficient.

3/4		2DEC	3.0 B-2
; 3.0 scaled by 2^-2 = 0.75 = 3/4 exactly. Used in W-matrix computations.

7/12		2DEC	.5833333333
; 0.5833333333 = 7/12 approximately. Integration weight for averaging.

9/16		2DEC	9 B-4
; 9 scaled by 2^-4 = 0.5625 = 9/16 exactly. Step fraction coefficient.

5/128		2DEC	5 B-7
; 5 scaled by 2^-7 = 0.0390625 = 5/128 exactly. Small perturbation weight.

DPZERO		EQUALS	ZEROVEC
; Double-precision zero. Alias to ZEROVEC for initialization.

DP2/3		2DEC	.6666666667
; 0.6666666667 = 2/3 approximately. Common fractional coefficient.

2/3		EQUALS	DP2/3
; Alias for DP2/3 used when 2/3 notation clearer in context.

OCT27		OCT	27
; Octal 27 = decimal 23. Bit mask or index value for table operations.

# Page 1247
		BANK	13
		SETLOC	ORBITAL2
		BANK

; ============================================================================
; CRITICAL SCALING EXPONENT TABLE (DO NOT REORDER)
;
; The following DEC constants are scaling exponents used for normalizing
; position and velocity components during orbit propagation. These values
; MUST remain in exact sequential order as they are accessed by indexed
; addressing in integration routines. Reordering will cause incorrect
; scaling and catastrophic navigation errors.
;
; Original NASA comment: "IT IS VITAL THAT THE FOLLOWING CONSTANTS NOT BE
; SHUFFLED" - emphasizing mission-critical importance of order preservation.
; ============================================================================

		DEC	-11
		DEC	-2
		DEC	-9
		DEC	-6
		DEC	-2
		DEC	-2
		DEC	0
		DEC	-12
		DEC	-9
		DEC	-4
ASCALE		DEC	-7
		DEC	-6
; Scaling exponents indexed by integration routine state counters.
; Each value represents power-of-two shift count for normalizing
; position/velocity components to maintain precision in 15-bit words.
; ASCALE marks specific entry point in table for acceleration scaling.

; Additional mathematical constants for orbit determination:

5/8		2DEC	5 B-3
; 5 scaled by 2^-3 = 0.625 = 5/8 exactly. Used in conic computations.

-1/12		2DEC	-.1
; -0.1 approximately -1/12 (actually -0.0833...). Negative coefficient.

RECRATIO	2DEC	.01
; 0.01 = 1/100. Rectification ratio threshold for Encke method restart.
; When position deviation exceeds 1% of reference orbit radius, rectify.

; Physical constants for celestial bodies (scaled for AGC arithmetic):

RSPHERE		2DEC	64373.76 E3 B-29
; 64373.76 × 10^3 scaled by 2^-29 = Earth equatorial radius in meters.
; Used for determining when spacecraft within Earth's sphere of influence.

RDM		2DEC	16093.44 E3 B-27
; 16093.44 × 10^3 scaled by 2^-27 = Moon radius in meters.
; Used for lunar surface proximity checks and landing radar validation.

RDE		2DEC	80467.20 E3 B-29
; 80467.20 × 10^3 scaled by 2^-29 = Earth-Moon distance reference in meters.
; Used for cislunar navigation and sphere-of-influence transitions.

; ============================================================================
; MEMORY LOCATION ALIASES FOR STATE VECTOR COMPONENTS
;
; These EQUALS directives define symbolic names for memory locations in
; VECTAB and integration work areas. Using aliases improves code readability
; and maintains consistency when accessing state vector components.
; ============================================================================

RATT		EQUALS 	00
; Position vector at current time (offset 0 in VECTAB).

VATT		EQUALS	6D
; Velocity vector at current time (offset 6D in VECTAB, 3 double-words).

TAT		EQUALS	12D
; Time at current state (offset 12D in VECTAB).

RATT1		EQUALS	14D
; Position vector at next integration step (offset 14D).

VATT1		EQUALS	20D
; Velocity vector at next integration step (offset 20D).

MU(P)		EQUALS	26D
; Gravitational parameter μ of primary body (offset 26D).
; Value switches between Earth μ and Moon μ depending on sphere of influence.

TDEC1		EQUALS	32D
; Decimal time variable for integration step (offset 32D).

URPV		EQUALS	14D
; Unit radial position vector (shares location with RATT1 during computation).

COSPHI/2	EQUALS	URPV 	+4
; Cosine of half-angle φ/2 (offset +4 from URPV base, at 18D).
; Used in Lambert targeting for transfer orbit computations.

UZ		EQUALS	20D
; Unit Z-axis vector or vertical component (shares location with VATT1).

TVEC		EQUALS	26D
; Transfer vector or time-dependent vector (shares location with MU(P)).
; Memory reuse during different computation phases saves precious RAM.

; ============================================================================
; QUALITY1 - J22 OBLATENESS PERTURBATION COMPUTATION
;
; COMMENT-ONLY READERS:
; The Moon and Earth are not perfect spheres - they bulge slightly at the
; equator. This routine computes the gravitational effect of that bulge
; (called "oblateness" or the J22 zonal harmonic term), which pulls the
; spacecraft slightly differently depending on latitude. This perturbation
; must be included to keep navigation accurate during long coast periods
; between engine burns. Without this correction, predicted positions would
; drift by kilometers over hours of flight.
;
; CODE-ALONG READERS:
; Computes the J22 second zonal harmonic gravitational perturbation term
; for Earth or Moon oblateness. The J22 term represents equatorial bulge
; effects on spacecraft trajectory. Mathematical form computed is:
;   (3/2) × J22 × (μ/r²) × [(5z²/r² - 1) × r_unit]
; which simplifies in computation to 5(Y²-X²) operations on unit position.
;
; This routine is called during each integration step to add oblateness
; acceleration to the primary body's point-mass gravity acceleration.
;
; INPUTS:
;   URPV (14D-19D) = Unit radial position vector [X, Y, Z] (scaled B+1)
;   COSPHI/2 (18D) = Z component of URPV (cos of latitude-related angle)
;   MOONFLAG = Software flag indicating Moon as primary body (set) or
;              Earth as primary body (clear)
;   E3J22R2M = Oblateness coefficient × r⁴ (scaled appropriately)
;
; OUTPUTS:
;   MPAC (0D-5D) = J22 perturbation acceleration vector (scaled B+61)
;   2D (stack) = Z component (cos φ/2) pushed for further computation
;
; REGISTER USAGE:
;   Interpretive mode throughout
;   MPAC = Working accumulator for vector computations
;   2D, 4D, 6D = Intermediate vector storage
;
; ALGORITHM:
; 1. Check MOONFLAG - if Earth, branch to NBRANCH (alternate computation)
; 2. Load X component of unit position vector and square it (X²)
; 3. QUALITY2 entry: Load Y component and square it (Y²)
; 4. Compute (Y² - X²)
; 5. Multiply by 5/8 constant and scale by unit position vector
;    Result: 5(Y² - X²) × URPV vector
; 6. Shift left 3 bits for proper scaling (B+3)
; 7. Add 2×X to X-component (implements unit vector projection)
; 8. Subtract 2×Y from Y-component (implements unit vector projection)
; 9. Multiply result by E3J22R2M coefficient (includes J22, μ, r⁴ terms)
; 10. Push Z component (COSPHI/2) for caller's use
; 11. Return with RVQ (J22 acceleration vector in MPAC, Z in stack)
;
; MISSION CONTEXT:
; During translunar and cislunar coast phases, this oblateness correction
; accumulates over hours of flight. Without it, state vector would drift,
; causing targeting errors at lunar orbit insertion or transearth injection.
; The effect is larger near Earth than Moon due to Earth's greater oblateness.
;
; NUMERICAL PRECISION:
; Input: Unit position B+1 (maximum value 1.0)
; Intermediate: Vector components B+3 after scaling (maximum value ~7.0)
; Output: Acceleration B+61 (matches other perturbation terms for addition)
; ============================================================================

QUALITY1	BOF	DLOAD
			MOONFLAG
; Branch on Flag false: If MOONFLAG clear (Earth primary), branch to NBRANCH.
; Load: Begin loading X component of unit position vector from URPV.
			NBRANCH
			URPV
		DSQ
; Double Square: Square X component. Result is X² scaled B+2.

QUALITY2	PDDL	DSQ		# SQUARE INTO 2D, B2
; Push Down and Load: Push X² onto stack at 2D.
; Load Y component from URPV+2 (second component of unit position).
			URPV	+2	# Y COMPONENT, B1
; Double Square: Square Y component. Result is Y² scaled B+2.
		DSU
; Double Subtract: Compute Y² - X² (difference of squares).
; This term represents equatorial vs polar distance variation.
		DMP	VXSC		# 5(Y**2-X**2)UR
; Double Multiply: Multiply (Y² - X²) by 5/8 constant.
; Vector Scale: Scale unit position URPV by scalar result.
; This produces vector: 5(Y² - X²) × URPV
			5/8		# CONSTANT, 5B3
			URPV		# VECTOR.  RESULT MAXIMUM IS 5, SCALING
# Page 1248
					# HERE B6
		VSL3	PDDL		# STORE SCALED B3 IN 2D, 4D, 6D FOR XYZ
; Vector Shift Left 3: Shift entire vector left 3 bits (multiply by 8).
; Changes scaling from B+6 to B+3 for subsequent operations.
; Push Down and Load: Store result vector in 2D-6D (X, Y, Z components).
; Load X component of URPV again for projection computation.
			URPV		# X COMPONENT, B1
		SR1	DAD		# 2 X X COMPONENT FOR B3 SCALING
; Shift Right 1: Shift X right 1 bit, effectively multiplying by 2.
; Result is 2X scaled B+2 (matching B+3 after shift).
; Double Add: Add 2X to vector X component at 2D.
			2D		# ADD TO VECTOR X COMPONENT OF ANSWER,
					# SAME AS MULTIPLYING BY UNITX.  MAX IS 7.
; This operation projects result along unit X direction.
; Mathematical equivalent: result += 2X × unit_x
		STODL	2D
; Store: Save modified X component back to 2D.
; Load: Load Y component of URPV for similar operation.
			URPV	+2	# Y COMPONENT, B1
		SR1	BDSU		# 2 X Y COMPONENT FOR B3 SCALING
; Shift Right 1: Shift Y right 1 bit, effectively multiplying by 2.
; Result is 2Y scaled B+2.
; Double Subtract (Backwards): Subtract vector Y component at 4D from 2Y.
; Actually computes 2Y - (vector_Y), but since vector was already
; 5(Y²-X²)×URPV_Y, this implements: vector_Y - 2Y correctly.
			4D		# SUBTRACT FROM VECTOR Y COMPONENT OF
					# ANSWER, SAME AS MULTIPLYING BY UNITY.
					# MAX IS 7.
; This operation projects result along unit Y direction.
		STORE	4D		# 2D HAS VECTOR, B3.
; Store: Save modified Y component back to 4D.
; Vector now contains complete J22 directional term at B+3 scaling.
		SLOAD	VXSC		# MULTIPLY COEFFICIENT TIMES VECTOR IN 2D
; Single Load: Load E3J22R2M coefficient (includes J22 harmonic constant,
; gravitational parameter μ, and r⁴ distance term).
			E3J22R2M
; Vector Scale: Scale entire vector in 2D-6D by E3J22R2M coefficient.
; This produces final J22 perturbation acceleration vector.
		PDDL	RVQ		# J22 TERM X R**4 IN 2D, SCALED B61
; Push Down: Push J22 acceleration vector result onto stack (caller retrieves).
; Load: Load Z component (COSPHI/2) for caller's subsequent computations.
; This is used in related perturbation terms that need latitude information.
			COSPHI/2	# SAME AS URPV +4  Z COMPONENT
; Return Via Q: Return to caller with J22 vector in MPAC, Z in top of stack.
; Caller will add this J22 perturbation to total acceleration vector.

; ============================================================================
; END OF ORBITAL_INTEGRATION MODULE
;
; This completes the orbital integration routines for the Lunar Module.
; All orbit propagation, perturbation computations, and state vector updates
; are implemented in the subroutines above. These routines successfully
; maintained navigation accuracy during Apollo 11's historic lunar landing
; mission, enabling precise targeting throughout all mission phases from
; Earth launch through lunar descent to surface touchdown on July 20, 1969.
; ============================================================================

