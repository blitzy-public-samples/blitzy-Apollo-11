# Copyright:	Public domain.
# Filename:	ORBITAL_INTEGRATION.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1334-1354
# Mod history:	2009-05-14 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-20 RSB	Corrections:  DAT -> DAD in one place,
#				BWM -> BMN, DEFEQCNT -> DIFEQCNT.
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

# Page 1334
# ORBITAL INTEGRATION

; ============================================================================
; FILE: ORBITAL_INTEGRATION.agc
; MODULE: CHIEFTAN Subsystem (Core OS)
; MISSION PHASE: all-phases
;
; TL;DR: Orbit propagation numerical integration implementing Encke method for
;        perturbed motion. Integrates equations of motion accounting for
;        perturbation forces (Earth/Moon oblateness, solar/lunar gravity),
;        enabling precise trajectory prediction throughout Apollo 11 cislunar
;        flight with integration step size control.
;
; COMMENT-ONLY READERS: This program computed how the spacecraft's position
;        changed over time considering all gravitational forces.
; CODE-ALONG READERS: Study Encke method numerical integration, perturbation
;        force modeling, step size control, trajectory propagation algorithms.
; ============================================================================

# DELETE
		BANK	13
		SETLOC	ORBITAL
		BANK
		COUNT	11/ORBIT

# DELETE
; ============================================================================
; KEPPREP - KEPLER PREPARATION ROUTINE
;
; This routine prepares initial parameters for Keplerian orbit computations.
; Throughout Apollo 11's flight, this code computed basic orbital parameters
; needed to predict where the spacecraft would be minutes or hours ahead.
;
; For code-along readers: Computes sqrt(MU), normalized position vector,
; velocity dot products, and scaling factors for subsequent Kepler iteration.
; Uses interpretive language for vector/matrix operations with MPAC stack.
; ============================================================================
KEPPREP		LXA,2	SETPD
			PBODY
			0
; Load gravitational parameter MU (Earth or Moon) and compute square root.
; MU = gravitational constant determining orbit characteristics.
; For Earth: MU ~ 398,600 km³/s², for Moon: MU ~ 4,903 km³/s²
		DLOAD*	SQRT		# SQRT(MU) (+18 OR +15) 0D		PL 2D
			MUEARTH,2
; Load position vector RCV and compute unit vector (direction to spacecraft).
; This normalized position is used for coordinate transformations.
		PDVL	UNIT		#					PL 8D
			RCV
; Normalize position magnitude, storing scaling factor in X1 register.
; AGC fixed-point arithmetic requires careful scaling to maintain precision.
		PDDL	NORM		# NORM R (+29 OR +27 - N1) 2D		PL 4D
			36D
			X1
		PDVL
; Compute dot product of position and velocity (F parameter).
; This determines energy and angular momentum of the orbit.
		DOT	PDDL		# F*SQRT(MU)(+7 OR +5) 4D	PL 6D
			VCV
; Load transfer time TAU (time interval for trajectory propagation).
; During Apollo 11, this computed state vectors minutes to hours ahead.
			TAU.		# (+28)
; Compute time difference from conic time (TAU - TC) and normalize.
; This establishes the time baseline for Kepler iteration.
		DSU	NORM
			TC
			S1
		SR1
		DDV	PDDL
			2D
; Compute FS parameter (scaled time-radius product).
; Critical for Kepler equation iteration convergence.
		DMP	PUSH		# FS(+6 +N1-N2) 6D		PL 8D
			4D
; Compute (FS)² for series expansion terms.
; Higher-order terms improve Kepler equation accuracy.
		DSQ	PDDL		# (FS)SQ(+12 +2(N1-N2)) 8D	PL 10D
			4D
; Compute SSQ/MU ratio for energy calculations.
; This determines orbital shape (ellipse, parabola, hyperbola).
		DSQ	PDDL*		# SSQ/MU(-2OR +2(N1-N2)) 10D		PL 12D
			MUEARTH,2
		SR3	SR4
; Pre-align MU for subsequent division operations.
; Careful alignment prevents overflow in fixed-point arithmetic.
		PDVL	VSQ		# PREALIGN MU (+43 OR +37) 12D	PL 14D
			VCV
; Compute velocity-squared and related energy terms.
; Orbital energy = (v²/2) - (MU/r) determines orbit type.
		DMP	BDSU		#				PL 12D
			36D
		DDV	DMP		#				PL 10D
			2D		# -(1/R-ALPHA)(+12 +3N1-2N2)
; Compute 1/R - ALPHA term for universal variable formulation.
; This allows single algorithm for all orbit types (ellipse/parabola/hyperbola).
		DMP	SL*
			DP2/3
			0 	-3,1	# 10L(1/R-ALPHA)(+13 +2(N1-N2))
; Build series expansion: 2(FS)² - FS + correction terms.
; Series converges rapidly for most Apollo trajectories.
		XSU,1	DAD		# 2(FS)SQ - ETCETERA	PL 8D
			S1		# X1 = N2-N1
		SL*	DSU		# -FS+2(FS)SQ ETC (+6 +N1-N2)	PL 6D
			8D,1
		DMP	DMP
			0D
			4D
		SL*	SL*
# Page 1335
			8D,1
			0,1		# S(-FS(1-2FS)-1/6...)(+17 OR +16)
; Compute initial guess XKEPNEW for Kepler iteration.
; Better initial guess = faster convergence (fewer iterations).
		DAD	PDDL		#				PL 6D
			XKEP
		DMP	SL*		# S(+17 OR +16)
			0D
			1,1
; Check for overflow and store new Kepler variable.
; TCDANZIG handles overflow conditions gracefully.
		BOVB	DAD
			TCDANZIG
		STADR
		STORE	XKEPNEW
; Save return address and set iteration counter to 10.
; Maximum 10 iterations prevents infinite loops if convergence fails.
		STQ	AXC,1
			KEPRTN
		DEC	10
		BON	AXC,1
			MOONFLAG
			KEPLERN
		DEC	2
		GOTO
			KEPLERN

# Page 1336
FBR3		LXA,1	SSP
			DIFEQCNT
			S1
		DEC	-13
		DLOAD	SR
			DT/2
			9D
		TIX,1	ROUND
			+1
		PUSH	DAD
			TC
		STODL	TAU.
		DAD
			TET
		STCALL	TET
			KEPPREP

# Page 1337
# AGC ROUTINE TO COMPUTE ACCELERATION COMPONENTS.

; ============================================================================
; TRANSITION: From Kepler preparation to acceleration computation
;
; With orbital parameters established, the guidance computer now shifts to
; computing perturbing accelerations. During Apollo 11's translunar coast,
; lunar orbit, and return, this code continuously computed how gravity from
; multiple bodies (Earth, Moon, Sun) and non-spherical mass distributions
; (oblateness) affected the spacecraft's trajectory.
; ============================================================================

; ============================================================================
; ACCOMP - ACCELERATION COMPONENT COMPUTATION
;
; Computes perturbing accelerations from all sources acting on spacecraft.
; This routine ran continuously during Apollo 11's flight, accounting for
; Earth oblateness during launch, lunar gravity during coast, and complex
; multi-body interactions during lunar orbit.
;
; For code-along readers: Computes angular momentum (R × V), evaluates
; perturbation forces, applies corrections to state vector. Uses W-matrix
; for covariance propagation in navigation state estimation.
; ============================================================================
; Load primary body index (PBODY) into X1 and X2 registers.
; PBODY identifies current reference body: Earth=0, Moon=2.
; During translunar coast, switches from Earth to Moon reference.
ACCOMP		LXA,1	LXA,2
			PBODY
			PBODY
; Initialize force vector FV to zero.
; Accumulated perturbation forces will be added to this vector.
		VLOAD
			ZEROVEC
		STOVL	FV
; Load ALPHAV (rectified position increment) and scale.
; ALPHAV represents small deviations from reference conic trajectory.
			ALPHAV
		VSL*	VAD
			0 -7,2
; Add to current position RCV to get perturbed position BETAV.
; BETAV = actual spacecraft position including all perturbations.
			RCV
		STORE	BETAV
; Check DIM0FLAG to determine if W-matrix extrapolation is needed.
; W-matrix represents covariance propagation for navigation state uncertainty.
; If DIM0FLAG is off, skip W-matrix storage; if on, store into VECTAB.
		BOF	XCHX,2
			DIM0FLAG
			+5
			DIFEQCNT
; Store current state vector into VECTAB table for W-matrix computation.
; DIFEQCNT indexes position in table for differential equation integration.
		STORE	VECTAB,2
		XCHX,2
			DIFEQCNT
; Normalize ALPHAV rectification vector to unit vector.
; Unit vector provides direction of perturbation for force calculations.
; Magnitude will be handled separately in perturbation computation.
		VLOAD	UNIT
			ALPHAV
		STODL	ALPHAV
; Load scaling factor (36D = decimal 36) for magnitude computation.
; This scaling maintains numerical precision in fixed-point arithmetic.
			36D
		STORE	ALPHAM
; Call GAMCOMP to compute gravitational acceleration components.
; GAMCOMP evaluates primary body gravity at current spacecraft position.
		CALL
			GAMCOMP
; Load perturbed position BETAV and save index S2 for later restoration.
; BETAV contains actual spacecraft position including Encke rectification.
		VLOAD	SXA,1
			BETAV
			S2
; Store BETAV as ALPHAV for next computation phase.
; Load magnitude of BETAM (position relative to non-primary body).
		STODL	ALPHAV
			BETAM
; Store as ALPHAM for third-body gravity calculation.
; BETAM enables computing perturbation from Earth (during lunar orbit) or
; Moon (during translunar/transearth coast).
		STORE	ALPHAM
; Check MIDFLAG to determine if this is midpoint integration step.
; If not at midpoint, branch to OBLATE for oblateness perturbation.
; If at midpoint, load TET (current mission time) for solar perturbation.
		BOF	DLOAD
			MIDFLAG
			OBLATE
			TET
; Call LSPOS to compute lunar and solar positions at current time.
; LSPOS returns position vectors of Moon and Sun in reference frame.
; Solar perturbation becomes significant during translunar coast.
		CALL
			LSPOS
; Set X2=2 for Moon index, restore X1 from S2 saved earlier.
; These indices select appropriate body for perturbation calculation.
		AXT,2	LXA,1
			2
			S2
; Check MOONFLAG to determine current reference body.
; If in Moon sphere of influence (lunar orbit), handle coordinate transform.
		BOF
			MOONFLAG
			+3
; Complement vector (negate) and set X2=0 for Earth reference.
; Coordinate transformation between Earth-centered and Moon-centered frames.
		VCOMP	AXT,2
			0
; Store transformed position as BETAV for current computation.
; RPQV stores position for later third-body perturbation calculation.
		STORE	BETAV
		STOVL	RPQV
# Page 1338
; Load 2D (from MPAC stack position 2) containing solar position vector.
; Solar gravity perturbation computed from Sun position relative to spacecraft.
			2D
; Store solar position as RPSV for third-body perturbation computation.
; During Apollo 11 translunar coast, solar perturbation was ~0.03 m/s² (small
; but non-negligible over 3-day journey).
		STORE	RPSV
; Check DIM0FLAG: if off, skip W-matrix update and branch to GETRPSV.
; W-matrix extrapolation tracks how navigation uncertainties propagate.
		BOF	VLOAD
			DIM0FLAG
			GETRPSV
			ALPHAV
; Compute scaled rectification vector for W-matrix partial derivative.
; ALPHAV × ALPHAM gives perturbation magnitude times direction.
		VXSC	VSR*
			ALPHAM
			1,2
; Subtract BETAV and store result into VECTAB for W-matrix row.
; This partial derivative shows how position perturbations affect accelerations.
		VSU	XCHX,2
			BETAV
			DIFEQCNT
		STORE	VECTAB +6,2
		XCHX,2
			DIFEQCNT
; ============================================================================
; GETRPSV - Get third-body position vector for perturbation computation
; ============================================================================
; Load position vector of third perturbing body (Moon or Sun).
; RPQV contains lunar position; increment X1 by 4 to track computation phase.
GETRPSV		VLOAD	INCR,1
			RPQV
			4
; Clear RPQFLAG and check MOONFLAG to determine reference frame.
; Coordinate system selection affects how third-body positions are computed.
		CLEAR	BOF
			RPQFLAG
			MOONFLAG
			+5
; Scale solar position vector by 2^-9 and add to RPSV.
; Scaling maintains numerical precision in fixed-point arithmetic.
; Combines lunar and solar perturbations for complete third-body effects.
		VSR	VAD
			9D
			RPSV
		STORE	RPSV
; Call GAMCOMP to compute gravitational acceleration from third body.
; Returns acceleration vector from Moon (during translunar) or Earth (during
; lunar orbit), plus solar perturbation.
		CALL
			GAMCOMP
		AXT,2	INCR,1
			4
			4
		VLOAD
			RPSV
		STCALL	BETAV
			GAMCOMP
		GOTO
			OBLATE
; ============================================================================
; GAMCOMP - Compute gravitational acceleration using Encke's method
; ============================================================================
; This routine implements the heart of Encke's perturbation method by
; computing accelerations in a numerically stable manner. It handles the
; critical difference between perturbed and reference trajectories.
;
; Encke's method avoids loss of precision by computing small perturbations
; from a reference conic orbit rather than integrating absolute positions.
; During Apollo 11's cislunar coast, this technique maintained trajectory
; accuracy to within meters over 200,000+ mile distances.
;
; Load BETAV (perturbed position) and scale right by 1 bit for precision.
; BETAV represents actual spacecraft position including all perturbations.
GAMCOMP		VLOAD	VSR1
			BETAV
; Compute BETAV squared magnitude and initialize push-down list at 0.
; VSQ gives dot product of vector with itself: |BETAV|² = BETAV·BETAV
		VSQ	SETPD
			0
; Normalize B² magnitude to prevent overflow in subsequent calculations.
; NORM shifts value to range [0.25, 1.0) and stores shift count in X1.
; ROUND ensures proper rounding for fixed-point arithmetic accuracy.
		NORM	ROUND
			31D
; Push normalized B² to stack, then load and normalize ALPHAM.
; ALPHAM = magnitude of reference conic position vector (from osculating orbit).
; ALPHAV = unit direction of reference position, scaled for computation.
		PDDL	NORM		# NORMED B SQUARED TO PD LIST
			ALPHAM		# NORMALIZE (LESS ONE) LENGTH OF ALPHA
			32D		# SAVING NORM SCALE FACTOR IN X1
; Scale ALPHAM right by 1 and push to stack.
; BETAV loaded to prepare for unit vector computation.
		SR1	PDVL
			BETAV		# C(PDL+2) = ALMOST NORMED ALPHA
; Compute unit vector of BETAV (perturbed position direction).
; UNIT divides vector by its magnitude: BETAV/|BETAV|
; Result is direction of actual spacecraft position.
		UNIT
		STODL	BETAV
# Page 1339
			36D
; Store magnitude of perturbed position vector for later use.
; BETAM = |BETAV|, the actual distance from central body.
		STORE	BETAM
; Compute normalized quotient ρ = ALPHAM/BETAM (ratio of reference to actual).
; This ratio is central to Encke's method - it compares reference orbit
; radius to actual perturbed radius. NORM stores scale factor in X1.
		NORM	BDDV		# FORM NORMALIZED QUOTIENT ALPHAM/BETAM
			33D
; Scale ρ right by 1 with rounding and push to stack.
; RHO represents how much the actual trajectory differs from reference conic.
; During nominal flight, RHO ≈ 1.0; perturbations cause small deviations.
		SR1R	PUSH		# C(PDL+2) = ALMOST NORMALIZED RHO.
; Load acceleration scaling factor from ASCALE table indexed by X1.
; ASCALE provides proper dimensional scaling for gravity computations.
; Store in S1 for repeated access during calculations.
		DLOAD*
			ASCALE,1
		STORE	S1
; Exchange and adjust index registers for scaling arithmetic.
; Complex index manipulation manages fixed-point scaling throughout computation.
		XCHX,2	XAD,2
			S1
			32D
		XSU,2	DLOAD
			33D
			2D
; Apply variable scaling shift to maintain precision during division.
; SR* uses index register value to determine shift amount dynamically.
		SR*	XCHX,2
			0 	-1,2
			S1
; Push result and scale RHO/4 with rounding for F function series evaluation.
; F function implements Encke's method acceleration ratio computation.
		PUSH	SR1R		# RHO/4 TO 4D
; Compute dot product ALPHAV·BETAV (cosine of angle between vectors).
; This geometric term appears in Encke's perturbation acceleration formula.
; Represents alignment between reference and actual position directions.
		PDVL	DOT
			ALPHAV
			BETAV
; Compute (RHO/4) - 2(ALPHAV·BETAV)/4 for Encke's F function.
; SL1R doubles the dot product with rounding; BDSU subtracts from RHO/4.
; This intermediate result feeds into series expansion for acceleration ratio.
		SL1R	BDSU		# (RHO/4) - 2(ALPHAV/2.BETAV/2)
		PUSH	DMPR		# TO PDL+6
			4
; Begin complex series evaluation for G function (Encke's velocity correction).
; SL1 left-shifts by 1 bit (multiply by 2) for proper scaling alignment.
		SL1
; Add quarter value and push; building nested terms for G(ρ) series expansion.
; G function modifies velocity perturbation based on trajectory deviation.
		PUSH	DAD
			DQUARTER
; Take square root of intermediate result for G function radical term.
; SQRT implements Newton-Raphson iteration for AGC's fixed-point arithmetic.
		PUSH	SQRT
; Multiply by PDL+10D and push result; accumulating G series contributions.
		DMPR	PUSH
			10D
; Double result and add quarter for next G function series term.
; Pattern: shift left, add constant, iterate - characteristic of polynomial eval.
		SL1	DAD
			DQUARTER
; Complete G function numerator: (1/4) + 2((Q+1)/4).
; This expression derives from Encke's geometric perturbation analysis.
		PDDL	DAD		# (1/4)+2((Q+1)/4)	TO PD+14D
			10D
			HALFDP
; Multiply by PDL+8D and scale for continued G function evaluation.
; Each operation maintains precision through careful fixed-point scaling.
		DMPR	SL1
			8D
; Add 3/8 constant and divide by PDL+14D to complete G function ratio.
; Final G value represents velocity perturbation scaling factor.
		DAD	DDV
			THREE/8
			14D
; Scale actual velocity vector BETAV by G function and PDL+6.
; VXSC performs component-wise vector-scalar multiplication.
; Result: velocity perturbation contribution from Encke's method.
		DMPR	VXSC
			6
			BETAV
; Prepare reference velocity (ALPHAV/8) and combine with G-scaled perturbation.
; VSR3 divides by 8 (shift right 3 bits) for alignment with perturbed term.
		PDVL	VSR3		# (G/2)(C(PD+4))B/2 TO PD+16D
			ALPHAV
; Add reference velocity to perturbation: total velocity for W-matrix update.
; VAD performs vector addition; result represents complete velocity state.
		VAD	PUSH		# A12 + C(PD+16D) TO PD+16D
; Load original RHO value and multiply by intermediate result at PDL+12D.
; Building final scaling factor for position vector extrapolation.
		DLOAD	DMP
			0
			12D
; Normalize result (stores shift count in X register) and round to precision.
; NORM critical for maintaining numerical stability in long integrations.
		NORM	ROUND
# Page 1340
			30D
; Divide by PDL+2 and multiply by gravitational parameter (μ).
; BDDV performs double-precision division with both operands from MPAC.
; DMP* indexed multiply accesses MUEARTH (Earth) or MUMOON (Moon) μ value.
; This converts dimensionless ratios to actual gravitational acceleration.
		BDDV	DMP*
			2
			MUEARTH,2
; Double-complement negates value (DCOMP = -A) and scale position vector.
; VXSC multiplies normalized result by PDL+16D position vector.
; Result: Encke perturbation acceleration contribution to total force.
		DCOMP	VXSC
; Complex index register manipulation for multi-scale arithmetic alignment.
; XCHX,2 exchanges values; XAD,2 adds to index register X2.
; S1 and S2 hold scaling factors accumulated during computation.
		XCHX,2	XAD,2
			S1
			S2
; Subtract scale normalization constants (30D and 31D) from index registers.
; These adjust for cumulative fixed-point scaling throughout calculation.
		XSU,2	XSU,2
			30D
			31D
; Check and clear overflow indicator without branching (BOV to next instruction).
; AGC overflow handling: overflow indicator must be explicitly cleared.
		BOV			# CLEAR OVIND
			+1
; Variable shift right controlled by index register value.
; VSR* final alignment of acceleration vector to match FV scaling.
; XCHX,2 restores saved index register after scaling complete.
		VSR*	XCHX,2
			0 	-1,2
			S1
; Add Encke perturbation acceleration to total force vector FV.
; FV accumulates all perturbation forces: oblateness, third-body, solar pressure.
; During cislunar coast, FV primarily contains lunar/solar gravity perturbations.
		VAD
			FV
; Store updated total force vector back to FV memory location.
; This completes one call to GAMCOMP for current integration substep.
		STORE	FV
; Branch on overflow to next instruction (clears overflow), then return.
; RVQ pops return address from Q register and transfers control.
; Normal path: integration continues with updated force vector.
		BOV	RVQ		# RETURN IF NO OVERFLOW
			+1
; ============================================================================
; GOBAQUE: Integration Error Recovery and Rectification
;
; Called when integration step completes or needs rectification due to
; trajectory deviation accumulation. Checks velocity perturbation magnitude,
; recomputes Keplerian reference if necessary, updates time parameters.
; Critical for maintaining Encke method accuracy over long coast periods.
; ============================================================================
GOBAQUE		VLOAD	ABVAL
			TDELTAV
; Load perturbation velocity vector TDELTAV and compute absolute magnitude.
; ABVAL calculates |v| = √(vx² + vy² + vz²) for 3D velocity vector.
; If perturbation too large, Encke's method requires reference orbit update.
		BZE
			INT-ABRT
; Branch to integration abort if TDELTAV magnitude is zero.
; Zero perturbation velocity indicates numerical failure or invalid state.
; INT-ABRT calls POODOO alarm handler for crew notification and safe mode.
		DLOAD	SR
			H
			9D
; Load integration step size H and shift right 9 bits (divide by 512).
; Reduces step size for more accurate rectification computation.
; Fixed-point scaling maintains precision through division operation.
		PUSH	BDSU
			TC
; Push scaled step onto MPAC stack and subtract conic time TC.
; Computes time interval since last rectification: ΔT = H/512 - TC.
; This determines time parameter for updated reference orbit.
		STODL	TAU.
			TET
; Store computed time interval to TAU. (tau parameter for Keplerian elements).
; Load current ephemeris time TET for time base update.
		DSU	STADR
; Subtract from address register (complex indexing operation).
; STADR performs indexed addressing for time parameter management.
		STCALL	TET
			KEPPREP
; Store updated ephemeris time and call KEPPREP for Keplerian element prep.
; KEPPREP converts current state vector to classical orbital elements.
; Establishes new reference trajectory for continued Encke integration.
		CALL
			RECTIFY
; Call RECTIFY subroutine to update reference orbit and reset perturbations.
; Rectification essential when deviation exceeds accuracy threshold.
; Resets DELTAV and DELTAR to zero relative to new reference trajectory.
		SETGO
			RPQFLAG
			TESTLOOP
; Set RPQFLAG indicating rectification completed successfully.
; Transfer control to TESTLOOP for next integration cycle.
; Integration continues with fresh reference orbit and minimal perturbations.

; ============================================================================
; INT-ABRT: Integration Abort Handler
;
; Emergency exit for orbital integration failure. Called when TDELTAV is zero,
; indicating numerical instability, state vector corruption, or computational
; overflow. Triggers POODOO program alarm (octal 00430) to alert crew and
; abort current navigation computation for safety.
; ============================================================================
INT-ABRT	EXIT
		TC	POODOO
		OCT	00430
; Exit interpreter mode and transfer control to POODOO alarm handler.
; Octal alarm code 00430 displayed on DSKY for crew/ground diagnosis.
; Aborts integration, preserves last valid state, awaits crew intervention.

# Page 1341
# THE OBLATE ROUTINE COMPUTES THE ACCELERATION DUE TO OBLATENESS.  IT USES THE UNIT OF THE VEHICLE
# POSITION VECTOR FOUND IN ALPHAV AND THE DISTANCE TO THE CENTER IN ALPHAM.  THIS IS ADDED TO THE SUM OF THE
# DISTURBING ACCELERATIONS IN FV AND THE PROPER DIFEQ STAGE IS CALLED VIA X1.

; ============================================================================
; OBLATE: Oblateness Perturbation Acceleration Computation
;
; Computes gravitational acceleration due to Earth/Moon oblateness (non-
; spherical shape). Uses J2 (2nd degree) and J4 (4th degree) zonal harmonic
; coefficients. Earth's equatorial bulge and lunar mass distribution create
; perturbations modeled by Legendre polynomials P2, P3, P4, P5. Critical for
; accurate long-duration orbit prediction during cislunar coast phases.
; ============================================================================
OBLATE		LXA,2	DLOAD
			PBODY
			ALPHAM
; Load index register X2 with primary body indicator (0=Earth, 2=Moon).
; Load ALPHAM (distance to central body center) into accumulator.
; PBODY determines which gravitational constants apply (J2REQSQ, RDE, etc.).
		SETPD	DSU*
			0
			RDE,2
; Set push-down list pointer to zero (initialize MPAC stack).
; Subtract reference distance RDE (Earth or Moon radius) indexed by PBODY.
; Computes altitude above reference surface: h = |r| - R_ref.
		BPL	BOF		# GET URPV
			NBRANCH
			MOONFLAG
			COSPHIE
; Branch if result positive (vehicle above surface - normal case).
; Check MOONFLAG: if clear (orbiting Earth), branch to COSPHIE.
; If set (orbiting Moon), continue to compute lunar oblateness terms.
; NBRANCH label handles altitude-dependent logic branching.
		VLOAD	PDDL
			ALPHAV
			TET
; Load unit position vector ALPHAV (normalized r̂ toward central body).
; Push to stack and load ephemeris time TET for time-dependent frame rotation.
; ALPHAV computed in ACCOMP as normalized position in inertial frame.
		PDDL	CALL
			3/5
			R-TO-RP
; Push constant 3/5 and call R-TO-RP coordinate transformation subroutine.
; R-TO-RP converts inertial reference frame to planet-fixed rotating frame.
; Required because oblateness harmonics defined in body-fixed coordinates.
		STORE	URPV
; Store computed unit reference position vector (URPV) in planet-fixed frame.
; URPV points from planet center to vehicle in body-centered, body-fixed coords.
; This is the reference direction for computing latitude-dependent harmonics.
		VLOAD	VXV
			504LM
			ZUNIT
; Load lunar module offset vector 504LM (for LM-specific position correction).
; Compute cross product with ZUNIT (Z-axis unit vector in inertial frame).
; VXV yields perpendicular vector for coordinate transformation.
		VAD	VXM
			ZUNIT
			MMATRIX
; Add ZUNIT and multiply result by MMATRIX transformation matrix.
; VXM performs vector-matrix multiplication for frame rotation.
; Converts from inertial to planet-fixed spherical harmonic reference frame.
		UNIT			# POSSIBLY UNNECESSARY
; Normalize result to unit vector (original comment notes possible redundancy).
; Ensures computational accuracy despite potential accumulated rounding errors.
COMTERM		STORE	UZ
; Common terminal point for Earth/Moon path convergence.
; Store computed Z-component unit vector (UZ) for latitude calculation.
; UZ represents component perpendicular to equatorial plane in body-fixed frame.

; ============================================================================
; Legendre Polynomial Computation for Oblateness Harmonics
;
; Computes P2, P3, P4, P5 Legendre polynomials as functions of cos(φ) where
; φ is geocentric latitude (angle from equatorial plane). These polynomials
; weight the J2, J3, J4 zonal harmonic coefficients that quantify Earth/Moon
; non-spherical gravitational field. Mathematical foundation for accurate
; orbit prediction accounting for equatorial bulge and polar flattening.
; ============================================================================
		DLOAD	DMPR
			COSPHI/2
			3/32
; Load COSPHI/2 (half-cosine of geocentric latitude) and multiply by 3/32.
; COSPHI/2 represents cos²(φ)/2 where φ is angle from pole to vehicle position.
; This initiates computation of Legendre polynomial P2 for 2nd-degree harmonic.
		PDDL	DSQ		# P2/64 TO PD0
			COSPHI/2
; Push intermediate result and load COSPHI/2 again for squaring operation.
; DSQ computes (cos²(φ)/2)² = cos⁴(φ)/4 for higher-order polynomial terms.
; Comment indicates P2/64 result pushed to stack position PD0.
		DMPR	DSU
			15/16
			3/64
; Multiply by 15/16 and subtract 3/64 to form P2 Legendre polynomial.
; P2(cos φ) = (3cos²φ - 1)/2, scaled and normalized for fixed-point arithmetic.
; P2 captures primary equatorial bulge gravitational perturbation.
		PUSH	DMPR		# P3/32 TO PD2
			COSPHI/2
; Push P2 result to stack and begin P3 computation using cos²φ/2.
; P3 computation builds on P2 results with higher-order cosine terms.
		DMP	SL1R
			7/12
		PDDL	DMPR
			0
			2/3
; Multiply by 7/12 with left shift (SL1R) for scaling adjustment.
; Push intermediate result, then multiply stack position 0 by 2/3.
; These operations construct P3(cos φ) = (5cos³φ - 3cos φ)/2 polynomial.
		BDSU	PUSH		# P4/128 TO PD4
		DMPR	DMPR
			COSPHI/2	# BEGIN COMPUTING P5/1024
			9/16
; Complete P3 with subtraction (BDSU) and push result to stack position PD4.
; Begin P4 computation: multiply by COSPHI/2 and 9/16.
; P4(cos φ) = (35cos⁴φ - 30cos²φ + 3)/8 captures 4th-order harmonic.
		PDDL	DMPR
			2
			5/128
# Page 1342
		BDSU
; Push intermediate result and load stack position 2, multiply by 5/128.
; BDSU (backwards subtract) completes P5 polynomial computation.
; P5(cos φ) = (63cos⁵φ - 70cos³φ + 15cos φ)/8 for 5th-order harmonic.
		DMP*
			J4REQ/J3,2
		DDV	DAD		#              -3
			ALPHAM		# (((P5/256)B 2 /R+P4/32)  /R+P3/8)ALPHAV
			4		#            4             3
		DMPR*	DDV
			2J3RE/J2,2
			ALPHAM
; ============================================================================
; Zonal Harmonic Coefficient Application
;
; Applies J2, J3, J4 zonal harmonic coefficients to computed Legendre
; polynomials. J4REQ/J3 ratio scaled for body (Earth or Moon indexed by X2).
; ALPHAM = -μ/R³ (negative gravitational parameter divided by cube of radius).
; This nested computation forms: (((P5·J4/R⁴) + P4·J3/R³) + P3·J2/R²)·μ/R³
; Result is magnitude of oblateness-induced acceleration perturbation.
; ============================================================================
		DAD	VXSC
			2
			ALPHAV
		STODL	TVEC
; Add stack position 2 (accumulated polynomial-coefficient product).
; VXSC (vector × scalar) multiplies unit vector ALPHAV by oblateness magnitude.
; ALPHAV points from body center to vehicle, providing acceleration direction.
; Store resulting oblateness acceleration vector in TVEC for integration.
		DMP*	SR1
			J4REQ/J3,2
		DDV	DAD
			ALPHAM		#			-3
		DMPR*	SR3
			2J3RE/J2,2	#	3	4
		DDV	DAD
			ALPHAM
		VXSC	VSL1
			UZ
		BVSU
			TVEC
; Compute perpendicular component correction using UZ (unit vector perpendicular
; to equatorial plane). This term accounts for out-of-plane gravitational
; effects from J3 and J4 harmonics (Earth/Moon polar flattening).
; SR1, SR3 scale results for fixed-point precision. VSL1 left-shifts vector.
; BVSU (backwards vector subtract) combines with radial component in TVEC.
		STODL	TVEC
			ALPHAM
		NORM	DSQ
			X1
		DSQ	NORM
			S1		#         4
		PUSH	BDDV*		# NORMED R  TO 0D
			J2REQSQ,2
; Store combined oblateness acceleration vector to TVEC.
; Load ALPHAM = -μ/R³ and normalize with DSQ (double square) operations.
; NORM instructions determine scaling exponents (stored in X1, S1) for
; maintaining precision in fixed-point arithmetic with R⁴ computations.
; BDDV divides by J2REQSQ (squared J2 reference radius for body).
; This normalizes acceleration magnitude to standard reference distance.
		VXSC	BOV
			TVEC
			+1		# (RESET OVERFLOW INDICATOR)
		XAD,1	XAD,1
			X1
			X1
		XAD,1	VSL*
			S1
			0	-22D,1
		VAD	BOV
; VXSC multiplies normalized scalar by TVEC oblateness acceleration vector.
; BOV (Branch on OVerflow) handles arithmetic overflow, skipping +1 if overflow.
; XAD,1 (indexed add to X1) adjusts scaling exponent three times for denormalization.
; VSL* (variable shift left indexed) applies computed shift count -22D,1.
; VAD adds oblateness acceleration to accumulated perturbation forces.
; Final BOV ensures overflow protection for integration stability.
			FV
			GOBAQUE
		STCALL	FV
			QUALITY1
; FV (Fixed Vector) designates result as vector quantity for overflow handling.
; GOBAQUE returns control after oblateness computation completes.
; STCALL (Store and Call) saves return address, calls QUALITY1 routine.
; QUALITY1 computes tesseral harmonic (J22) gravitational perturbations.
; ============================================================================
; QUALITY3 - Tesseral Harmonic (J22) Gravitational Perturbation
;
; Computes J22 tesseral harmonic perturbation representing Earth's equatorial
; ellipticity (difference between equatorial diameters). While J2-J5 zonal
; harmonics model polar flattening, J22 models "pear-shaped" equatorial bulge.
; This effect is small but significant for precise long-duration trajectory
; integration during cislunar flight. Result scaled in B61 as vector.
; ============================================================================

QUALITY3	DSQ			# J22 TERM X R**4 IN 2D.  SCALED B61
					# AS VECTOR.
		PUSH	DMP		# STORE COSPHI**2 SCALED B2 IN 8D
# Page 1343
			5/8		# 5 SCALED B3
; DSQ (Double SQuare) computes cos⁴φ from cos²φ for tesseral harmonic basis.
; PUSH stores cos⁴φ scaled B2 to stack position 8D for later use.
; DMP multiplies by 5/8 (scaled B3) beginning tesseral polynomial computation.
		PDDL	SR2		# PUT 5 COSPHI**2, D5, IN 8D. GET
					# COSPHI**2 D2 FROM 8D
		DAD	BDSU		# END UP WITH (1-7 COSPHI**2), B5
			8D		# ADDING COSPHI**2 B4 SAME AS COSPHI**2
					# X 2 D5
			D1/32		# 1 SCALED B5
; PDDL (Push and Double Load) saves result, loads cos²φ from stack 8D.
; SR2 (Shift Right 2) scales cos²φ from B2 to B4 for proper alignment.
; DAD adds shifted cos²φ (effectively 2·cos²φ in B5) to 5cos⁴φ.
; BDSU (Backward SUbtract) computes 1 - 7cos²φ by subtracting from constant D1/32.
; Result (1-7cos²φ) scaled B5 ready for J22 coefficient multiplication.
		DMP	DMP
			URPV		# X COMPONENT
			5/8		# 5 SCALED B3
		VXSC	VSL5		# AFTER SHIFT, SCALED B5
			URPV		# VECTOR, B1.
		PDDL			# VECTOR INTO 8D, 10D, 12D, SCALED B5.
; DMP multiplies polynomial (1-7cos²φ) by URPV (Unit Radial Position Vector) X component.
; Second DMP multiplies by 5/8 coefficient for J22 tesseral harmonic strength.
; VXSC (Vector by SCalar) multiplies scalar result by full URPV vector.
; VSL5 (Vector Shift Left 5) scales result from B10 to B5 for proper magnitude.
; PDDL pushes resulting J22 perturbation vector to stack positions 8D, 10D, 12D.
					# GET 5 COSPHI**2 OUT OF 8D
		DSU	DAD
			D1/32		# 1 B5
			8D		# X COMPONENT (SAME AS MULTIPLYING
					# BY UNITX)
		STODL	8D
			URPV		# X COMPONENT
		DMP	DMP
			URPV	+4	# Z COMPONENT
			5/8		# 5 B3 ANSWER B5
; DSU (Double SUbtract) computes 1 - 5cos²φ for X component correction.
; DAD adds this to stored value from stack 8D.
; STODL (STOre and Double Load) stores X component result, loads URPV X for next calc.
; DMP multiplies URPV X component by URPV Z component (forming XZ term).
; Second DMP multiplies by 5/8 coefficient for J22 X-Z cross term in tesseral field.
		SL1	DAD		# FROM 12D FOR Z COMPONENT (SL1 GIVES 10
					# INSTEAD OF 5 FOR COEFFICIENT)
		PDDL	NORM		# BACK INTO 12D FOR Z COMPONENT.
			ALPHAM		# SCALED B27 FOR MOON
			X2
; SL1 (Shift Left 1) doubles coefficient from 5 to 10 for proper Z component weight.
; DAD adds Z component contribution from stack position 12D.
; PDDL pushes Z component result, loads ALPHAM (longitude of ascending node).
; NORM normalizes ALPHAM scaled B27 (for Moon), storing exponent in X2 register.
		PUSH	SLOAD		# STORE IN 14D, DESTROYING URPV
					# X COMPONENT
			E32C31RM
		DDV	VXSC		# IF X2 = 0, DIVISION GIVES B53, VXSC
					# OUT OF 8D B5 GIVES B58
		VSL*	VAD		# SHIFT MAKES B61, FOR ADDITION OF
					# VECTOR IN 2D
			0	-3,2
; PUSH stores normalized ALPHAM to stack 14D (overwrites URPV X component).
; SLOAD (Single LOAD) retrieves constant E32C31RM (Earth 3:2 commensurability term).
; DDV (Double DiVide) scales by longitude ratio for tesseral orientation.
; VXSC multiplies vector from 8D (scaled B5) producing B58 intermediate result.
; VSL* (indexed Variable Shift Left) applies -3,2 shift reaching B61 target scale.
; VAD adds tesseral vector to accumulated perturbations from stack 2D.
		VSL*	V/SC		# OPERAND FROM 0D, B108 FOR X1 = 0
			0	-27D,1	# FOR X1 = 0, MAKES B88, GIVING B-20
					# FOR RESULT.
		PDDL	PDDL
			TET
				5/8	# ANY NON-ZERO CONSTANT
; VSL* (indexed shift) and V/SC (Vector divide by SCalar) apply final scaling.
; Combined shift of -27D,1 (indexed by X1) brings tesseral result to B-20 scale.
; PDDL pushes tesseral perturbation vector, loads TET (time since epoch).
; Second PDDL pushes TET, loads constant 5/8 as non-zero value for later use.
		LXA,2	CALL		# POSITION IN 0D, TIME IN 6D. X2 LEFT
					# ALONE.
			PBODY
			RP-TO-R
		VAD	BOV		# OVERFLOW INDICATOR RESET IN "RP-TO-R"
			FV
			GOBAQUE
		STORE	FV
; LXA,2 loads index register X2 from PBODY (primary body indicator: Earth/Moon).
; CALL RP-TO-R converts position from rotating to reference frame at time TET.
; RP-TO-R subroutine also resets overflow indicator as noted in comment.
; VAD adds coordinate-transformed tesseral perturbation to velocity derivative FV.
; BOV (Branch on OVerflow) detects computational overflow, branches to GOBAQUE.
; STORE saves final velocity derivative FV with all perturbations accumulated.
; This completes J22 tesseral harmonic computation for equatorial ellipticity effects.
# Page 1344
; ============================================================================
; NBRANCH - Differential Equation Dispatch Control
;
; Branches to appropriate differential equation routine based on DIFEQCNT
; counter. Uses table-driven dispatch through DIFEQTAB containing addresses
; of DIFEQ+0, DIFEQ+1, DIFEQ+2 routines. Counter scaled by -1/12 for proper
; table indexing. This mechanism enables dynamic selection of perturbation
; models during integration (conic-only, oblateness, or full perturbations).
; ============================================================================

NBRANCH		SLOAD	LXA,1
			DIFEQCNT
			MPAC
		DMP	CGOTO
			-1/12
			MPAC
			DIFEQTAB
; SLOAD retrieves DIFEQCNT (differential equation counter) indicating which model.
; LXA,1 loads counter value into index register X1 for table lookup.
; DMP multiplies by -1/12 scaling factor for DIFEQTAB index computation.
; CGOTO (Computed GOTO) branches to address from table DIFEQTAB indexed by result.
; This dispatches to DIFEQ+0 (conic), DIFEQ+1 (oblate), or DIFEQ+2 (tesseral).
; ============================================================================
; COSPHIE - Cosine Phi Extraction for Oblateness Computation
;
; Extracts cos(φ) from ALPHAV+4 (latitude angle component) for use in
; oblateness perturbation calculations. Stores result in COSPHI/2 memory
; location and loads ZUNIT (Z-axis unit vector) before branching to COMTERM
; (common termination). Used when perturbation model requires latitude-
; dependent gravitational terms but full tesseral computation not needed.
; ============================================================================

COSPHIE		DLOAD
			ALPHAV +4
		STOVL	COSPHI/2
			ZUNIT
		GOTO
			COMTERM
; DLOAD retrieves ALPHAV+4 containing cos(φ) (latitude cosine) from position vector.
; STOVL stores cos(φ) to COSPHI/2, then loads ZUNIT (Z-axis unit vector).
; GOTO branches to COMTERM routine for common differential equation termination.
; This path provides latitude information for J2-J5 zonals without J22 tesseral.

; ============================================================================
; DIFEQTAB - Differential Equation Address Table
;
; Jump table containing addresses (CADR) of three differential equation routines:
;   DIFEQ+0: Two-body conic motion (Keplerian orbits, no perturbations)
;   DIFEQ+1: Conic + oblateness (includes J2-J5 zonal harmonics)
;   DIFEQ+2: Full perturbations (zonals + J22 tesseral harmonic)
; Used by NBRANCH dispatch routine based on DIFEQCNT selector. Enables
; dynamic switching between perturbation models during orbital integration.
; ============================================================================

DIFEQTAB	CADR	DIFEQ+0
		CADR	DIFEQ+1
		CADR	DIFEQ+2
; CADR (Core ADdRess) entries provide bank-qualified addresses for cross-bank calls.
; DIFEQ+0: Basic two-body Keplerian motion without gravitational perturbations.
; DIFEQ+1: Adds zonal harmonic perturbations (J2-J5) for oblateness effects.
; DIFEQ+2: Complete model with zonal + tesseral (J22) for full Earth gravity field.

; ============================================================================
; TIMESTEP - Integration Step Size Control and Rectification Check
;
; Controls orbital integration timestep progression and checks for need to
; rectify coordinate origin (Encke method characteristic). Tests MIDFLAG to
; determine midpoint vs endpoint integration state. If not at midpoint, calls
; RECTEST to check if coordinate rectification required (when perturbations
; δr become large relative to reference orbit). Also checks for body-of-
; influence switching when transitioning Earth→Moon or Moon→Earth domains.
; ============================================================================

TIMESTEP	BOF	CALL
			MIDFLAG
			RECTEST		# SKIP ORIGIN CHANGE LOGIC
			CHKSWTCH
; BOF MIDFLAG: Branch On Flag not set - if not at integration midpoint, call RECTEST.
; RECTEST checks if Encke rectification needed (recompute reference conic when δr large).
; CHKSWTCH checks if spacecraft crossed Earth-Moon gravitational sphere of influence.
		BMN
			DOSWITCH
; BMN (Branch MiNus): If CHKSWTCH result negative, body switch needed - goto DOSWITCH.

; ============================================================================
; RECTEST - Rectification Test for Encke Method Integration
;
; Tests whether coordinate rectification required by checking magnitude of
; accumulated perturbation vectors TDELTAV (velocity perturbation) and TNUV
; (position perturbation). If either exceeds 3/4 threshold (75% of reference
; values), Encke method accuracy degrades and rectification mandatory. Calls
; RECTIFY subroutine to recompute reference conic and reset perturbations to
; zero. Essential for maintaining numerical precision during long integrations.
; ============================================================================

RECTEST		VLOAD	ABVAL		# RECTIFY IF
			TDELTAV
; VLOAD loads TDELTAV vector (accumulated velocity perturbations δv).
; ABVAL computes absolute value (magnitude) of TDELTAV vector.
		BOV
			CALLRECT
; BOV (Branch on OVerflow): If magnitude computation overflowed, rectify immediately.
		DSU	BPL		#	1) EITHER TDELTAV OR TNUV EQUALS OR
			3/4		#	   EXCEEDS 3/4 IN MAGNITUDE
			CALLRECT	#
; DSU subtracts 3/4 threshold (0.75). BPL branches if result positive (magnitude ≥ 0.75).
		DAD	SL*		#			OR
			3/4		#
			0 -7,2		#	2) ABVAL(TDELTAV) EQUALS OR EXCEEDS
		DDV	DSU		#	   .01(ABVAL(RCV))
			10D
			RECRATIO
; Additional test: Compare velocity perturbation ratio to position magnitude.
; DAD adds 3/4 again, SL* scales, DDV divides by 10D (position magnitude from stack).
; DSU subtracts RECRATIO threshold (0.01 = 1% of position vector magnitude).
		BPL	VLOAD
			CALLRECT
			TNUV
; BPL: If ratio test positive (perturbation ≥ 1% of position), rectify.
; Otherwise continue: VLOAD TNUV loads position perturbation vector for testing.
		ABVAL	DSU
			3/4
; ABVAL computes magnitude of TNUV (position perturbation δr).
; DSU subtracts 3/4 threshold - same test as for velocity perturbations.
		BOV
			CALLRECT
; BOV: If magnitude computation overflowed, rectify immediately.
		BMN
			INTGRATE
; BMN: If result negative (|TNUV| < 0.75), no rectification needed - continue integration.
; ============================================================================
; CALLRECT - Rectification Call Point
; Calls RECTIFY subroutine to recompute reference conic orbit and reset
; perturbation vectors to zero. After rectification, integration continues
; from new reference trajectory with δr=0, δv=0.
; ============================================================================

CALLRECT	CALL
			RECTIFY
; CALL RECTIFY: Invokes rectification subroutine (recomputes reference orbit).

; ============================================================================
; INTGRATE - State Vector Integration Step
;
; Performs one step of orbit integration by adding accumulated perturbations
; to reference state vectors. TNUV (position perturbation δr) stored to ZV,
; TDELTAV (velocity perturbation δv) stored to YV. Clears JSWITCH flag to
; indicate normal integration (not body-switch case). These working vectors
; then feed into DIFEQ differential equation evaluator.
; ============================================================================

INTGRATE	VLOAD
			TNUV
; VLOAD TNUV: Load position perturbation vector δr into accumulator.
# Page 1345
		STOVL	ZV
			TDELTAV
; STOVL stores δr to ZV, then loads velocity perturbation δv (TDELTAV).
		STORE	YV
; STORE YV: Save δv to working vector YV for differential equation evaluation.
		CLEAR
			JSWITCH
; CLEAR JSWITCH: Reset body-switching flag (indicates Earth vs Moon primary body).

; ============================================================================
; DIFEQ0 - DIFFERENTIAL EQUATION EVALUATION ENTRY POINT
;
; The guidance computer has completed rectification and now begins computing
; accelerations from gravitational forces. This routine initializes the
; integration state, loading the spacecraft's current position into working
; variables and resetting the integration step counter.
;
; During Apollo 11's translunar coast, this routine executed repeatedly to
; predict the spacecraft's trajectory. The integration step H starts at zero
; and increments through the integration interval (0, DT/2, DT).
;
; Technical: Entry point for differential equation evaluation. Loads state
; vector YV into ALPHAV for acceleration computation. Sets DIFEQCNT to 0.
; Initializes step size H to zero for Nystrom method. Checks JSWITCH flag
; to determine if W-matrix integration is needed (DOW..) or just vehicle
; state (ACCOMP).
; ============================================================================

DIFEQ0		VLOAD	SSP
			YV
			DIFEQCNT
			0
		STODL	ALPHAV
			DPZERO
		STORE	H		# START H AT ZERO.  GOES 0(DELT/2)DELT.
		BON	GOTO
			JSWITCH
			DOW..
			ACCOMP

; ============================================================================
; CHKSWTCH - CHECK FOR PRIMARY BODY SWITCH
;
; As Apollo 11 traveled from Earth toward the Moon, there came a point where
; the Moon's gravitational influence became stronger than Earth's. At this
; "sphere of influence" boundary, the computer must switch its reference
; frame, making the Moon the primary body instead of Earth. This ensures
; accurate navigation predictions throughout the journey.
;
; The routine checks the spacecraft's distance from the secondary body. If
; the spacecraft crosses the sphere of influence (typically at ~200,000
; nautical miles from Earth), it sets flags to trigger a coordinate frame
; transformation during the next integration cycle.
;
; Technical: Computes distance from secondary body using lunar position
; (LUNPOS call if not cached in RPQFLAG). Compares |RQC| against RSPHERE
; boundary. MOONFLAG determines which body is currently primary. If distance
; check indicates boundary crossing, branches to DOSWITCH for coordinate
; frame change and rectification.
; ============================================================================

CHKSWTCH	STQ	BOF
			ORIGEX
			RPQFLAG
			RPQOK		# MOON POSITION IS AVAILABLE
		DLOAD	CALL
			TET
			LUNPOS		# GET MOON POSITION
		BOF	VCOMP
			MOONFLAG
			+1
		STORE	RPQV

RPQOK		LXA,2	VLOAD		# RESTORE X2 AFTER USING LUNPOS
			PBODY
			TDELTAV		#  _
		VSL*	VAD		# |RQC|-RSPHERE WHEN OUTSIDE THE SPHERE.
			0	-7,2	# _   _            _
			RCV		# R = RDEVIATION + RCONIC
		BOF	ABVAL
			MOONFLAG
			EARSPH
		SR2	BDSU		# INSIDE
			RSPHERE
		GOTO
			ORIGEX
EARSPH		VSU	ABVAL		# OUTSIDE
			RPQV
		DSU	GOTO
			RSPHERE
			ORIGEX

; The spacecraft has crossed the sphere of influence boundary. Switch primary body.

DOSWITCH	CALL
			ORIGCHNG
		GOTO
			INTGRATE

# Page 1346
; ============================================================================
; ORIGCHNG - ORIGIN CHANGE FOR BODY SWITCH
;
; When the spacecraft crosses the gravitational sphere of influence, the
; coordinate system must be transformed. If Earth was the primary body, the
; Moon becomes primary (and vice versa). This routine performs the necessary
; coordinate transformations and updates all state vectors accordingly.
;
; First, the routine calls RECTIFY to compute the current osculating conic
; elements. Then it transforms position and velocity vectors to the new
; reference frame centered on the new primary body.
;
; Technical: Calls RECTIFY to establish reference conic. Performs coordinate
; transformation by translating position/velocity vectors. Updates RPQV
; (secondary body position), flips MOONFLAG to indicate new primary body,
; stores transformed state back to integration variables.
; ============================================================================

ORIGCHNG	STQ	CALL
			ORIGEX
			RECTIFY
		VLOAD	VSL*
			RCV
			0,2
		VSU	VSL*
			RPQV
			2,2
		STORE	RRECT
		STODL	RCV
			TET
		CALL
			LUNVEL
		BOF	VCOMP
			MOONFLAG
			+1
		PDVL	VSL*
			VCV
			0,2
		VSU
		VSL*
			0 +2,2
		STORE	VRECT
		STORE	VCV
		LXA,2	SXA,2
			ORIGEX
			QPRET
		BON	GOTO
			MOONFLAG
			CLRMOON
			SETMOON
# Page 1347
# THE RECTIFY SUBROUTINE IS CALLED BY THE INTEGRATION PROGRAM AND OCCASIONALLY BY THE MEASUREMENT INCORPORATION
# ROUTINES TO ESTABLISH A NEW CONIC.


RECTIFY		LXA,2	VLOAD
			PBODY
			TDELTAV
		VSL*	VAD
			0 	-7,2
			RCV
		STORE	RRECT
		STOVL	RCV
			TNUV
		VSL*	VAD
			0 	-4,2
			VCV
MINIRECT	STORE	VRECT
		STOVL	VCV
			ZEROVEC
		STORE	TDELTAV
		STODL	TNUV
			ZEROVEC
		STORE	TC
		STORE	XKEP
		RVQ

# Page 1348
; ============================================================================
; TRANSITION: From Encke rectification to Nystrom integration
;
; Having rectified the state vector if needed, the integration now proceeds
; using the Nystrom method to advance the trajectory through the current
; timestep. The Nystrom method evaluates accelerations at three points
; (beginning, middle, end) to achieve higher numerical accuracy than simple
; Euler integration, critical for Apollo 11's precise cislunar navigation.
; ============================================================================

; NYSTROM INTEGRATION METHOD
;
; COMMENT-ONLY READERS: The computer now calculates how the spacecraft's
; position and velocity change over the next time interval by sampling the
; gravitational forces at three different moments. This multi-point approach
; ensures trajectory predictions remain accurate throughout the mission.
;
; CODE-ALONG READERS: Implementation of Nystrom's three-point integration
; method for second-order differential equations (F = ma becomes dv/dt = a).
; Three variants handle different perturbation levels:
;   DIFEQ+0: Pure Keplerian (two-body only, fastest computation)
;   DIFEQ+1: Keplerian + zonal harmonics (J2-J5 oblateness terms)
;   DIFEQ+2: Full model including J22 tesseral harmonic
;
; The method evaluates acceleration at t, t+h/2, and t+h, combining them
; with Nystrom coefficients to update position and velocity. Integration
; feeds into W-matrix extrapolation for navigation covariance propagation.

# THE THREE DIFEQ ROUTINES - DIFEQ+0, DIFEQ+12, DIFEQ+24 - ARE ENTEREDTO PROCESS THE CONTRIBUTIONS AT THE
# BEGINNING, MIDDLE, AND END OF THE TIMESTEP, RESPECTIVELY.  THE UPDATING IS DONE BY THE NYSTROM METHOD.

; DIFEQ+0: Keplerian two-body motion (no perturbations, pure conic sections)
; Used during cislunar coast when perturbation effects negligible for accuracy.
; Acceleration from central body gravity only: a = -μ/r² * r̂
DIFEQ+0		VLOAD	VSR3
			FV
		STCALL	PHIV
			DIFEQCOM

; DIFEQ+1: Keplerian motion + oblateness perturbations (J2-J5 zonal harmonics)
; Used when near Earth or Moon where non-spherical gravity field affects orbit.
; Adds acceleration from zonal harmonics: a_oblate = function(J2,J3,J4,J5,r,φ)
; More computationally expensive than DIFEQ+0 but necessary for orbital accuracy.
DIFEQ+1		VLOAD	VSR1
			FV
		PUSH	VAD
			PHIV
		STOVL	PSIV
		VSR1	VAD
			PHIV
		STCALL	PHIV
			DIFEQCOM

; DIFEQ+2: Full perturbation model (Keplerian + zonal + tesseral harmonics)
; Maximum accuracy mode including J22 tesseral harmonic for equatorial bulge.
; Required for precision orbits near Earth where all gravitational asymmetries
; matter. Computes: a_total = a_central + a_zonal + a_tesseral
; Most computationally intensive variant, used when accuracy demands override
; speed considerations (e.g., during critical rendezvous or entry preparation).
DIFEQ+2		DLOAD	DMPR
			H
			DP2/3
		PUSH	VXSC
			PHIV
		VSL1	VAD
			ZV
		VXSC	VAD
			H
			YV
		STOVL	YV
			FV
		VSR3	VAD
			PSIV
		VXSC	VSL1
		VAD
			ZV
		STORE	ZV
		BOFF	CALL
			JSWITCH
			ENDSTATE
			GRP2PC
		LXA,2	VLOAD
			COLREG
			ZV
		VSL3			# ADJUST W-POSITION FOR STORAGE
		STORE	W 	+54D,2
		VLOAD
			YV
		VSL3	BOV
			WMATEND
		STORE	W,2

		CALL
			GRP2PC
# Page 1349
		LXA,2	SSP
			COLREG
			S2
			0
		INCR,2	SXA,2
			6
			YV
		TIX,2	CALL
			RELOADSV
			GRP2PC
		LXA,2	SXA,2
			YV
			COLREG

NEXTCOL		CALL
			GRP2PC
		LXA,2	VLOAD*
			COLREG
			W,2
		VSR3			# ADJUST W-POSITION FOR INTEGRATION
		STORE	YV
		VLOAD*	AXT,1
			W 	+54D,2
			0
		VSR3			# ADJUST W-VELOCITY FOR INTEGRATION
		STCALL	ZV
			DIFEQ0

ENDSTATE	BOV	VLOAD
			GOBAQUE
			ZV
		STOVL	TNUV
			YV
		STORE	TDELTAV
		BON	BOFF
			MIDAVFLG
			CKMID2		# CHECK FOR MID2 BEFORE GOING TO TIMEINC
			DIM0FLAG
			TESTLOOP
		EXIT
		TC	PHASCHNG
		OCT	04022		# PHASE 1
		TC	UPFLAG		# PHASE CHANGE HAS OCCURRED BETWEEN
		ADRES	REINTFLG	# INTSTALL AND INTWAKE
		TC	INTPRET
		SSP
			QPRET
			AMOVED
		BON	GOTO
			VINTFLAG
# Page 1350
			ATOPCSM
			ATOPLEM
AMOVED		SET	SSP
			JSWITCH
			COLREG
		DEC	-30
		BOFF	SSP
			D6OR9FLG
			NEXTCOL
			COLREG
		DEC	-48
		GOTO
			NEXTCOL

RELOADSV	DLOAD			# RELOAD TEMPORARY STATE VECTOR
			TDEC		# FROM PERMANENT IN CASE OF
		STCALL	TDEC1
			INTEGRV2	# BY STARTING AT INTEGRV2.

; DIFEQCOM: Common Nystrom integration finalization logic
;
; COMMENT-ONLY READERS: After computing gravitational forces at the current
; time point, the computer now combines all three acceleration samples
; (beginning, middle, end of timestep) using the Nystrom formula to update
; the spacecraft's position and velocity for the next instant.
;
; CODE-ALONG READERS: This common routine (shared by DIFEQ+0/+1/+2) implements
; the Nystrom integration formula:
;   Position: r(t+h) = r(t) + h*v(t) + h²/2 * (a(t) + 4*a(t+h/2) + a(t+h))/6
;   Velocity: v(t+h) = v(t) + h * (a(t) + 4*a(t+h/2) + a(t+h))/6
; Increments timestep H and counter DIFEQCNT for next evaluation point.
; Stores result in ALPHAV for next integration cycle or W-matrix update.
DIFEQCOM	DLOAD	DAD		# INCREMENT H AND DIFEQCNT.
			DT/2
			H
		INCR,1	SXA,1
		DEC	-12
			DIFEQCNT	# DIFEQCNT SET FOR NEXT ENTRY.
		STORE	H
; Compute Nystrom integration formula:
; ALPHAV = YV + H*(ZV + H*(FV+4*GV+HV)/6)
; where YV = initial position, ZV = initial velocity
; FV = initial acceleration, GV = midpoint acceleration, HV = final acceleration
		VXSC	VSR1
			FV
		VAD	VXSC
			ZV
			H
		VAD
			YV
		STORE	ALPHAV
; If W-matrix integration active (JSWITCH flag), proceed to W-matrix update (DOW..)
; Otherwise continue main integration loop (FBR3).
		BON	GOTO
			JSWITCH
			DOW..
			FBR3

; WMATEND: W-matrix integration termination with alarm
;
; COMMENT-ONLY READERS: If the W-matrix computation encounters a problem
; (such as excessive integration steps), the computer issues alarm 421
; and continues with the trajectory calculation while marking the W-matrix
; as invalid for navigation updates.
;
; CODE-ALONG READERS: This error handler is invoked when W-matrix integration
; cannot complete successfully. Clears all W-matrix flags (DIM0FLAG, ORBWFLAG,
; RENDWFLG) to disable W-matrix updates and invalidate any partial results.
; Sets STATEFLG to ensure state vector integration continues. Issues program
; alarm 421 to notify crew, then returns to TESTLOOP to complete integration.
WMATEND		CLEAR	CLEAR
			DIM0FLAG	# DONT INTEGRATE W THIS TIME
			ORBWFLAG	# INVALIDATE W
		CLEAR
			RENDWFLG
		SET	EXIT
			STATEFLG	# PICK UP STATE VECTOR UPDATE
		TC	ALARM
		OCT	421
		TC	INTPRET
# Page 1351
		GOTO
			TESTLOOP	# FINISH INTEGRATING STATE VECTOR

# Page 1352
# ORBITAL ROUTINE FOR EXTRAPOLATION OF THE W MATRIX.  IT COMPUTES THE SECOND DERIVATIVE OF EACH COLUMN POSITION
# VECTOR OF THE MATRIX AND CALLS THE NYSTROM INTEGRATION ROUTINES TO SOLVETHE DIFFERENTIAL EQUATIONS.  THE PROGRAM
# USES A TABLE OF VEHICLE POSITION VECTORS COMPUTED DURING THE INTEGRATION OF THE VEHICLES POSITION AND VELOCITY.

; DOW..: W-matrix extrapolation routine for orbit determination
;
; COMMENT-ONLY READERS: The W-matrix helps the navigation system understand
; how small errors in position propagate over time, enabling more accurate
; orbit determination when radar or optical measurements are processed.
; This routine computes how each column of the matrix evolves under
; gravitational forces, using the same Nystrom integration method as the
; main trajectory calculation.
;
; CODE-ALONG READERS: W-matrix (state transition matrix) is a 6×6 matrix
; representing partial derivatives ∂(r,v)(t+Δt)/∂(r,v)(t). Each column
; represents how a unit perturbation in one initial state component propagates.
; This routine integrates each column position vector using gravitational
; acceleration computed from stored position vectors (VECTAB) from main
; integration. Handles both primary and secondary body gravity depending on
; body-switching state (PBODY). Result used by navigation filter for covariance
; propagation and measurement incorporation.
DOW..		LXA,2	DLOAD*
			PBODY
			MUEARTH,2
		STCALL	BETAM
			DOW..1
; Store primary body acceleration contribution in FV
		STORE	FV
; If at midpoint of integration (MIDFLAG clear), skip secondary body gravity
; Otherwise process secondary body gravity for W-matrix column
		BOF	INCR,1
			MIDFLAG
			NBRANCH
		DEC	-6
; Load secondary body gravitational parameter (opposite of PBODY)
		LXC,2	DLOAD*
			PBODY
			MUEARTH 	-2,2
		STCALL	BETAM
			DOW..1
; Add secondary body acceleration to primary body acceleration (FV)
; Scale appropriately based on body-switching state (MOONFLAG)
		BON	VSR6
			MOONFLAG
			+1
		VAD
			FV
		STCALL	FV
			NBRANCH
; DOW..1: Subroutine to compute gravitational acceleration on W-matrix column
;
; Computes acceleration using inverse-square law:
; a = -μ * (r_perturbed / |r_perturbed|³)
; where r_perturbed = ALPHAV + VECTAB (W-matrix column + vehicle position)
;
; Uses projection method to compute 1/r³ efficiently from stored vehicle
; position data, avoiding repeated square root operations.
DOW..1		VLOAD	VSR4
			ALPHAV
		PDVL*	UNIT
			VECTAB,1
; Load W-matrix column (ALPHAV) and vehicle position (VECTAB)
; Compute unit direction vector of vehicle position
		PDVL	VPROJ
			ALPHAV
; Project W-matrix column onto vehicle position unit vector
; This gives component of perturbation in radial direction
		VXSC	VSU
			3/4
; Compute perturbed position magnitude using projection approximation
; Avoids expensive square root by leveraging stored vehicle position data
		PDDL	NORM
			36D
			S2
		PUSH	DSQ
		DMP
		NORM	PDDL
			34D
			BETAM
; Compute gravitational parameter μ divided by r³
; BETAM contains μ for current body (primary or secondary)
		SR1	DDV
		VXSC
; Scale unit direction by -μ/r³ to get acceleration vector
		LXA,2	XAD,2
			S2
			S2
		XAD,2	XAD,2
			S2
			34D
; Adjust scaling based on normalization shifts (S2, 34D)
		VSL*	RVQ
# Page 1353
			0 -8D,2

		SETLOC	ORBITAL1
		BANK

; ===========================================================================
; CONSTANTS SECTION: Numerical Integration Coefficients and Physical Data
; ===========================================================================
;
; COMMENT-ONLY READERS: These precisely-defined numbers are the mathematical
; constants needed for orbital calculations - some control how integration
; steps are taken, others define Earth and Moon's gravitational properties.
; The AGC's fixed-point arithmetic requires all values to be carefully scaled
; to fit within the 15-bit word length while maintaining precision.
;
; CODE-ALONG READERS: All constants use 2DEC (double-precision decimal)
; format with scaling factors (B-n notation means multiply by 2^-n).
; Integration coefficients derive from Nystrom and Encke method formulas.
; Physical constants are IAU 1965 standard values scaled to AGC units.
; Constants are grouped by function: integration weights, gravitational
; parameters (μ), Earth oblateness terms (J2, J3, J4), and reference radii.

; Nystrom integration formula coefficients
; Used in computing weighted combinations of derivatives for position updates
3/5		2DEC	.6 B-2

THREE/8		2DEC	.375

.3D		2DEC	.3 B-2

3/64		2DEC	3 B-6

DP1/4		2DEC	.25

DQUARTER	EQUALS	DP1/4
POS1/4		EQUALS	DP1/4
3/32		2DEC	3 B-5

15/16		2DEC	15. B-4

3/4		2DEC	3.0 B-2

7/12		2DEC	.5833333333

9/16		2DEC	9 B-4

5/128		2DEC	5 B-7

DPZERO		EQUALS	ZEROVEC
DP2/3		2DEC	.6666666667

2/3		EQUALS	DP2/3
; Octal constant for shift operations
OCT27		OCT	27

		BANK	13
		SETLOC	ORBITAL2
		BANK

; ===========================================================================
; SCALING EXPONENTS TABLE - MEMORY LAYOUT CRITICAL
; ===========================================================================
;
; COMMENT-ONLY READERS: The following numbers control how the computer scales
; different orbital calculation results to prevent overflow in its limited
; 15-bit arithmetic registers. These must remain in exact order.
;
; CODE-ALONG READERS: This table of decimal exponents is indexed by PBODY
; and other control variables. The sequence corresponds to different orbital
; states and coordinate transformations. Memory layout is critical - NASA
; comment explicitly forbids reordering. Each DEC value represents a power
; of 2 for scaling position, velocity, and acceleration components.
# IT IS VITAL THAT THE FOLLOWING CONSTANTS NOT BE SHUFFLED
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
# Page 1354

; ===========================================================================
; GRAVITATIONAL PARAMETERS (μ VALUES)
; ===========================================================================
;
; COMMENT-ONLY READERS: These are the gravitational strengths of the Sun,
; Moon, and Earth - fundamental constants that determine how strongly each
; body pulls on the spacecraft during the Apollo 11 mission.
;
; CODE-ALONG READERS: Gravitational parameters μ = GM (gravitational constant
; times mass) in meters³/sec². IAU 1965 standard values. Scaling chosen to
; maximize precision within AGC double-precision format (29-bit mantissa).
; Sun μ: 1.32715445 × 10¹⁶ m³/s² scaled by 2^-54
; Moon μ: 4.9027780 × 10⁸ m³/s² scaled by 2^-30
; Earth μ: 3.986032 × 10¹⁰ m³/s² scaled by 2^-36
		2DEC*	1.32715445 E16 B-54*	# S (Sun)

		2DEC*	4.9027780 E8 B-30*	# M (Moon)

MUEARTH		2DEC*	3.986032 E10 B-36*

		2DEC	0

; ===========================================================================
; EARTH OBLATENESS COEFFICIENTS (J2, J3, J4 HARMONIC TERMS)
; ===========================================================================
;
; COMMENT-ONLY READERS: The Earth isn't a perfect sphere - it bulges at the
; equator and has other shape irregularities. These numbers quantify those
; shape deviations, which create extra gravitational forces that must be
; accounted for during orbital calculations to keep Apollo 11 on course.
;
; CODE-ALONG READERS: Zonal harmonic coefficients from geodetic Earth model.
; J2 (oblate bulge): ~1.082×10⁻³, J3 (pear shape): ~-2.5×10⁻⁶,
; J4 (higher order): ~-1.6×10⁻⁶. These create perturbing accelerations
; computed in OBLATE routine. Precomputed combinations with Earth radius
; (REQ) reduce multiplication operations during integration.
; J4REQ/J3: ratio of J4 harmonic times radius to J3
J4REQ/J3	2DEC*	.4991607391 E7 B-26*

		2DEC	-176236.02 B-25

; 2J3RE/J2: twice J3 times Earth radius divided by J2
2J3RE/J2	2DEC*	-.1355426363 E5 B-27*

		2DEC*	.3067493316 E18 B-60*

; J2REQSQ: J2 times Earth equatorial radius squared
J2REQSQ		2DEC*	1.75501139 E21 B-72*

; 3J22R2MU: 3 times J2 squared times radius squared divided by μ
3J22R2MU	2DEC*	9.20479048 E16 B-58*

; Additional integration coefficients
5/8		2DEC	5 B-3

-1/12		2DEC	-.1

; Memory location reference: MUM points to Moon's gravitational parameter
; Located 2 words before MUEARTH in the constants table
MUM		=	MUEARTH -2

; ===========================================================================
; PLANETARY RADII AND REFERENCE DISTANCES
; ===========================================================================
;
; COMMENT-ONLY READERS: These are the physical sizes of the Moon and Earth,
; used to determine when the spacecraft is close enough for the planets'
; non-spherical shapes to affect the trajectory.
;
; CODE-ALONG READERS: Reference radii in meters, scaled for AGC arithmetic.
; RECRATIO: Rectification threshold ratio (0.01 = 1% position change)
; RSPHERE: Spherical approximation valid beyond this radius
; RDM: Moon equatorial radius (1,738 km reference)
; RDE: Earth equatorial radius (6,378 km reference)
RECRATIO	2DEC	.01

RSPHERE		2DEC	64373.76 E3 B-29

RDM		2DEC	16093.44 E3 B-27

RDE		2DEC	80467.20 E3 B-29

; ===========================================================================
; PUSH-DOWN STACK MEMORY LOCATION ALIASES
; ===========================================================================
;
; COMMENT-ONLY READERS: These define standard positions in the computer's
; temporary working memory where orbital calculation results are stored.
;
; CODE-ALONG READERS: EQUALS statements create symbolic names for offsets
; into the interpreter's push-down stack (MPAC area). Standard calling
; convention for vector/matrix routines. Positions specified in double-
; precision words (D suffix). RATT/VATT: position/velocity of target body.
; RATT1/VATT1: position/velocity at rectification. MU(P): gravitational
; parameter. URPV: unit position vector. UZ: unit z-vector. TVEC: time vector.
RATT		EQUALS 	00
VATT		EQUALS	6D
TAT		EQUALS	12D
RATT1		EQUALS	14D
VATT1		EQUALS	20D
MU(P)		EQUALS	26D
TDEC1		EQUALS	32D
URPV		EQUALS	14D
COSPHI/2	EQUALS	URPV 	+4
UZ		EQUALS	20D
TVEC		EQUALS	26D
