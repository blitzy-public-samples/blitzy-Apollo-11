# Copyright:	Public domain.
# Filename:	P37_P70.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim.lawton@gmail.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	890-933
# Mod history:	2009-05-11 JVL	Adapted from the Colossus249/ file
#				of the same name, using Comanche055 page
#				images.
#		2009-05-20 RSB	Added missing label V2T179.  Fixed POODOO -> POODOO.
#		2009-05-23 RSB	In RTD18, corrected a STOVL DELVLVC to
#				STODL DELVLVC and a STODL 02D to STORE 02D.
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
; FILE: P37_P70.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: trans-earth
;
; TL;DR: Return to Earth targeting program (P37) and Transearth Injection
;        planning (P70). Computes burn parameters for departing lunar orbit
;        and establishing Earth return trajectory. Critical for Apollo 11's
;        journey home after lunar operations, calculating precise TEI
;        (Transearth Injection) maneuver executed July 21, 1969.
;
; COMMENT-ONLY READERS: This program calculated the engine burn that sent
;        Apollo 11 back to Earth from lunar orbit after the successful Moon
;        landing mission. Follow the comments to understand how the guidance
;        computer planned the return trajectory targeting Earth's atmospheric
;        entry corridor.
; CODE-ALONG READERS: Study Earth return trajectory optimization, atmospheric
;        entry corridor targeting constraints, TEI burn parameter computation,
;        and iterative velocity vector calculations that balance fuel efficiency
;        with reentry safety margins.
; ============================================================================

# Page 890
		BANK	31
		SETLOC	RTE1
		BANK

		EBANK=	RTEDVD
		COUNT	31/P37

; ============================================================================
; TRANSITION: Transearth Injection Planning
;
; After completing lunar operations, the Command Module must execute a precise
; engine burn to depart lunar orbit and return to Earth. This program computes
; the Transearth Injection (TEI) burn parameters that will place the spacecraft
; on a trajectory intersecting Earth's atmospheric entry corridor - a narrow
; window just 27 nautical miles high. For Apollo 11, this burn occurred on
; July 21, 1969, sending Columbia and its crew home after humanity's first
; lunar landing mission.
; ============================================================================

# PROGRAM DESCRIPTION:  P37, RETURN TO EARTH
#
# DESCRIPTION
#	A RETURN TO EARTH TRAJECTORY IS COMPUTED PROVIDED THE CSM IS OUTSIDE THE LUNAR SPHERE OF INFLUENCE AT THE
#	TIME OF IGNITION.  INITIALLY A CONIC TRAJECTORY IS DETERMINED AND RESULTING IGNITION AND REENTRY PARAMETERS ARE
# 	DISPLAYED TO THE ASTRONAUT.  THEN IF THE ASTRONAUT SO DESIRES, A PRECISION TRAJECTORY IS DETERMINED WITH THE
# 	RESULTING IGNITION AND REENTRY PARAMETERS DISPLAYED.  UPON FINAL ACCEPTANCE BY THE ASTRONAUT, THE PROGRAM
# 	COMPUTES AND STORES THE TARGET PARAMETERS FOR RETURN TO EARTH FOR USE BY SPS PROGRAM (P40) OR RCS PROGRAM (P41).

;
; OPERATIONAL CONTEXT - Return to Earth Trajectory Planning:
;
; After lunar orbit operations are complete, the crew initiates P37 to plan
; the Transearth Injection (TEI) burn. The program operates in two phases:
;
; 1. CONIC SOLUTION: Computes a quick two-body trajectory assuming only
;    Earth's gravity. This provides initial burn parameters displayed to
;    the crew for evaluation within seconds.
;
; 2. PRECISION SOLUTION: If crew accepts the conic solution, P37 computes
;    a high-fidelity trajectory accounting for lunar gravity, Earth
;    oblateness, and solar perturbations. This precision integration
;    ensures accurate targeting of Earth's narrow entry corridor.
;
; The computed velocity change (DELVLVC) specifies the burn magnitude and
; direction required to achieve the desired flight path angle (GAMMAEI)
; at 400,000 feet altitude - the official entry interface. The entry angle
; must be shallow enough to avoid excessive deceleration forces, yet steep
; enough to prevent the spacecraft from skipping back out of the atmosphere.
;
; For Apollo 11's return on July 21, 1969, this program calculated the TEI
; burn that brought Armstrong, Aldrin, and Collins safely home, achieving
; Pacific Ocean splashdown on July 24.
;
#
# CALLING SEQUENCE
#	L	TC	P37
#
# SUBROUTINES CALLED
#	PREC100
#		V2T100
#		RTENCK2
#		RTENCK3
#		TIMERAD
#		PARAM
#	V2T100
#		GAMDV10
#		XT1LIM
#		DVCALC
#	RTENCK1
#		INTSTALL
#		INTEGRVS
#	RTEVN
#		RTEDISP
#		TMRAD100
#		AUGEKUGL
#		LAT-LONG
#	TMRAD100
#		TIMERAD
#	INVC100
#		CSMPREC
#	GETERAD
#	TIMETHET
#	P370ALRM
#	VN1645
#	POLY
#
# ERASABLE INITIALIZATION REQUIRED
#	CSM STATE VECTOR
# Page 891
#	NJETSFLG	NUMBER OF JETS IF THE RCS PROPULSION SYSTEM SELECTED	STATE FLAG	0=4 JETS  1=2 JETS
#
# ASTRONAUT INPUT
#	SPRTETIG	TIME OF IGNITION (OVERLAYS TIG)				DP	B28	CS
#	VPRED		DESIRED CHANGE IN VELOCITY AT TIG(PROGRM COMPUTED IF 0)	DP	B7	METERS/CS
#	GAMMAEI		DESIRED FLIGHT PATH ANGLE AT REENTRY (COMPUTED IF 0)	DP	B0	REVS + ABOVE HORIZ.
#	OPTION2		PROPULSION SYSTEM OPTION				SP	B14	1=SPS, 2=RCS
#
# OUTPUT
#    CONIC OR PRECISION TRAJECTORY DISPLAY
#	VPRED	 	VELOCITY MAGNITUDE AT 400,000 FT. ENTRY ALTITUDE	DP	B7	METERS/CS
#	T3TOT4		TRANSIT TIME TO 400,000 FT. ENTRY ALTITUDE		DP	B28	CS
#	GAMMAEI		FLIGHT PATH ANGLE AT 400,00 FT. ENTRY ALTITUDE		DP	B0	REVS + ABOVE HORIZON
#	DELVLVC		INITIAL VELOCITY CHANGE VECTOR IN LOCAL VERTICAL COORD.	VECTOR	B7	METERS/CS
#	LAT(SPL)	LATITUDE OF THE LANDING SITE				DP	B0	REVS
#	LNG(SPL)	LONGITUDE OF THE LANDING SITE				DP	B0	REVS
#    TARGETING COMPUTATION DISPLAY
#	TIG		RECOMPUTED TIG BASED ON THRUST OPTION			DP	B28	CS
#	TTOGO		TIME FROM TIG						DP	B28	CS
#	+MGA		POSITIVE MIDDLE GIMBAL ANGLE				DP	B0	REVS -.02 IF REFSMFLG=0
#    THRUST PROGRAM COMMUNICATION
#	XDELVFLG	EXTERNAL DELTA V FLAG					STATE	FLAG	SET 0 FOR LAMBERT AIMPT
#	NORMSW		LAMBERT AIMPT ROTATION SWITCH				STATE	FLAG	SET 0 FOR NO ROTATION
#	ECSTEER		CROSS PRODUCT STEERING CONSTANT				SP	B2	SET 1
#	RTARG		CONICALLY INTEGRATED REENTRY POSITION VECTOR		VECTOR	B29	METERS
#	TPASS4		REENTRY TIME						DP	B28	CS

;
; P37 MAIN ENTRY POINT - Return to Earth Program
;
; The crew initiates P37 by entering VERB 37 ENTER on the DSKY when ready
; to plan the Transearth Injection (TEI) burn. This program computes the
; velocity change required to depart lunar orbit and return safely to Earth,
; targeting the narrow atmospheric entry corridor.
;
; For Apollo 11 on July 21, 1969, this program calculated the TEI burn that
; brought Armstrong, Aldrin, and Collins home after completing humanity's
; first lunar landing mission. The burn placed Columbia on a precise trajectory
; for Pacific Ocean splashdown three days later on July 24.
;
; The program is NOT RESTARTABLE - if interrupted, it must be re-initiated.
;
P37		TC	PHASCHNG	# P37 IS NOT RESTARTABLE
		OCT	4

;
; Initialize trajectory computation parameters. Zero out predicted velocity
; and flight path angle - these will either be computed by the program or
; entered by the crew via DSKY.
;
		TC	INTPRET		; Enter interpretive mode
		AXT,1	SXA,1		; Set index register X1
		OCT	04000		; Initialize to 04000
			ECSTEER		; Store in ECSTEER location
		DLOAD			; Load double precision
			ZEROVECS	; Zero vector constant
		STORE	VPRED		; Clear predicted velocity magnitude
		STORE	GAMMAEI		; Clear entry flight path angle
		EXIT			; Return to native AGC mode
;
; CREW INPUT SEQUENCE
;
; The program prompts the crew for two key parameters via DSKY:
; 1. Time of Ignition (TIG) - when to execute the TEI burn
; 2. Desired reentry angle and velocity change (if non-zero)
;
; For Apollo 11's TEI on July 21, the crew confirmed the computed TIG
; that would achieve proper Pacific Ocean splashdown targeting.
;
		CAF	V6N33RTE	# INPUT TIG	STORED IN SPRTETIG
		TCR	P370GOF		#		OVERLAYED WITH TIG
		TCF	-2		# DISPLAY NEW DATA
		CAF	V6N60RTE	# INPUT REENTRY ANGLE IN GAMMAEI
		TCR	P37GFRB1	#	AND DESIRED DELTA V IN RTEDVD
		TCF	-2		# DISPLAY NEW DATA
;
; ============================================================================
; TRAJECTORY COMPUTATION SECTION
;
; With crew inputs confirmed, P37 now computes the return trajectory.
; This section initializes the conic trajectory solution - a two-body
; approximation assuming only Earth's gravity. The conic solution provides
; quick initial results for crew review before proceeding to the more
; computationally intensive precision trajectory integration.
;
; The computation determines the velocity change vector (DELVLVC) required
; to achieve the desired entry conditions at 400,000 feet altitude.
; ============================================================================
;
RTE299		TC	INTPRET		; Re-enter interpretive mode
		SSP	DLOAD		; Set push location, load double
			OVFIND		; Overflow indicator
			0		; Initialize to zero
			VPRED		; Load predicted velocity
# Page 892
		STODL	RTEDVD		; Store as RTE delta-V desired
			GAMMAEI		; Load entry flight path angle
		STODL	RTEGAM2D	; Store as RTE gamma desired
			1RTEB13		; Load constant 1/B13
		STODL	CONICX1		; Store in conic X1 parameter
			C4RTE		; Load C4 constant for RTE
		STCALL	MAMAX1		; Store max iterations count
			INVC100		; GET R(T1)/,V(T1)/,UR1/,UH/
;
; Check if computation requires high-precision "slow" mode based on
; desired delta-V magnitude and spacecraft position. SLOWFLG is set
; when near Earth's sphere of influence requiring more careful calculation.
;
		CLEAR	DLOAD		; Clear slow computation flag
			SLOWFLG		; Normal speed trajectory mode
			RTEDVD		; Load desired delta-V
		BPL	ABS
			RTE317
		STORE	RTEDVD
		DLOAD	DSU
			R(T1)
			K1RTE
		BMN	SET
			RTE317
			SLOWFLG
RTE317		DLOAD	EXIT		; Load position at T1
			R(T1)		; Spacecraft position magnitude
;
; Compute maximum iteration count as function of distance from Earth.
; Uses 3rd-degree polynomial: MAMAX2 = C0 + C1*R + C2*R^2 + C3*R^3
; Farther from Earth requires fewer iterations to converge on solution.
;
		TC	POLY		; Polynomial evaluation subroutine
		DEC	2		; Polynomial degree 3 (index 2)
		2DEC	181000434. B-31	; C0 constant term
		2DEC	1.50785145 B-2	; C1 linear coefficient
		2DEC*	-6.49993057 E-9 B27*	; C2 quadratic coefficient
		2DEC*	9.76938926 E-18 B56*	; C3 cubic coefficient
		TC	INTPRET
		SL1
		STODL	MAMAX2		# C0+C1*R+C2*R**2+C3*R**3=MAMAX2 B30
			M9RTEB28
		STODL	NN1A
			K2RTE
RTE320		STODL	RCON		# RCON=K2
			RTEGAM2D	; Load desired entry angle
;
; ENTRY ANGLE COMPUTATION BRANCH
;
; If crew input non-zero gamma (flight path angle at 400K ft altitude),
; compute X(T2) as cotangent of that angle. Otherwise, use default values
; based on spacecraft distance from Earth to ensure entry corridor targeting.
;
		BZE	BDSU		; Branch if zero (no gamma input)
			RTE340		# GOTORTE340 IF REENTRY ANGLE NOT INPUT
			1RTEB2		; Subtract bias constant
		PUSH	COS		#					PL02D
		PDDL	SIN
		BDDV	STADR		#					PL00D
		STCALL	X(T2)		# X(T2)=COT(GAM2D)			B0
			RTE360		; Jump to V2T100 calculation
;
; DEFAULT ENTRY ANGLE SELECTION (no crew input)
;
; Choose X(T2) parameter based on spacecraft position relative to K1RTE
; threshold. Near Earth (R < K1) uses steeper approach (K3), while farther
; out (R >= K1) uses shallower approach (K4). This ensures the trajectory
; stays within the narrow atmospheric entry corridor for safe deceleration.
;
RTE340		DLOAD	DSU		; Load position at T1
			R(T1)		; Current spacecraft distance
# Page 893
			K1RTE		; Subtract position threshold
		BMN	DLOAD		; Branch if R(T1) < K1RTE
			RTE350		; Use K3 for closer approach
			K4RTE		; Load K4 constant (shallower)
		STCALL	X(T2)		# X(T2)=K4
			RTE360		; Jump to V2T100 calculation
RTE350		DLOAD			; Closer to Earth path
			K3RTE		; Load K3 constant (steeper)
		STORE	X(T2)		# X(T2)=K3
RTE360		CALL			; Call velocity computation
			V2T100		; Compute V2 and T2 from X(T2)
;
; V2T100 returns with overflow indicator in OVFIND. Zero indicates successful
; convergence on entry trajectory solution. Non-zero triggers program alarm.
;
		BZE	GOTO		; Branch if successful (OVFIND=0)
			RTE367		; Continue with solution
			RTEALRM		; Jump to alarm handler
RTE367		VLOAD
			R(T1)/
		STODL	RVEC
			RCON
		STOVL	RDESIRED
			V2(T1)/
		STCALL	VVEC
			TMRAD100
		DAD
			T1
		STODL	T2		; Store time at entry (400K ft)
			RTEGAM2D	; Load desired gamma
;
; ENTRY ANGLE REFINEMENT
;
; If no gamma specified by crew (zero), compute refined X(T2) from velocity
; at entry using polynomial fit. Otherwise use the X(T2) already computed
; from crew's specified gamma angle.
;
		BZE	GOTO		; Branch if gamma not specified
			RTE369		; Compute X(T2) from velocity
			RTE372		; Use existing X(T2)
RTE369		VLOAD	ABVAL		; Load velocity vector at T2
			V(T2)/		; Entry velocity magnitude
		EXIT			; Exit interpreter for POLY
;
; Polynomial computes X(T2) from entry velocity V2:
; X(T2) = D1 + D2*V2 + D3*V2^2 + D4*V2^3
; This empirical relationship ensures trajectory stays in entry corridor.
;
		TC	POLY		; Polynomial evaluation
		DEC	2		; Degree 3
		2DEC	0		; D1 = 0
		2DEC	-4.8760771 E-2 B4	; D2 coefficient
		2DEC	4.5419476 E-4 B11	; D3 coefficient
		2DEC	-1.4317675 E-6 B18	; D4 coefficient

		TC	INTPRET		; Re-enter interpreter
		DAD			; Add offset
			RTED1		; D1 constant offset
		SL3	GOTO		# X(T2),=D1+D2V2+D3V2**2+D4V2**3
			RTE373		; Continue with computed X(T2)
RTE372		DLOAD			# X(T2),=X(T2)
			X(T2)		; Use crew-specified gamma's X(T2)
RTE373		DSU	PUSH		# X(T2)ERR				B0 PL02D
# Page 894
			X(T2)		; Previous X(T2) value
;
; CONVERGENCE CHECK AND REENTRY PARAMETERS
;
; Compute error in X(T2) by comparing new computed value with previous
; iteration. This X(T2)ERR is pushed to stack for convergence testing.
;
; Next, compute reentry corridor parameters from position at entry point
; to verify trajectory will safely decelerate the spacecraft through
; Earth's atmosphere to landing.
;
		VLOAD	UNIT		; Load position at entry
			R(T2)/		#					B58
		STCALL	ALPHAV		; Store unit position vector
			GETERAD		; Get Earth radius at entry latitude
		DAD			; Add offset constant
			E3RTE		; E3 bias value
		PUSH	DSU		# RCON,=(E1/1+E2BETA11)**.5)+E3 	B29 PL04D
			RCON		; Subtract previous RCON
;
; TEST FOR CONVERGENCE
;
; Two convergence criteria must be satisfied:
; 1. Change in RCON must be less than EPC2RTE threshold
; 2. X(T2) error must be less than EPC3RTE threshold
;
; If both criteria met, trajectory has converged to valid Earth return path.
; Otherwise, iterate again with refined parameters (up to maximum iterations).
;
		ABS	DSU		; Absolute RCON change
			EPC2RTE		; Convergence threshold 1
		BMN	GOTO		; Branch if converged
			RTE374		; Check second criterion
			RTE375		; Not converged, continue iteration
RTE374		DLOAD	ABS		; Check X(T2) error magnitude
			00D		; X(T2)ERR from stack
		DSU	BMN		; Compare to threshold
			EPC3RTE		; Convergence threshold 2
			P37E		; Both criteria met - CONVERGED!
RTE375		DLOAD	DAD		; Not converged yet
			NN1A		; Load iteration counter
			1RTEB28		; Increment by 1
;
; ITERATION LIMIT CHECK
;
; Increment iteration counter and verify we haven't exceeded maximum allowed
; iterations (MAMAX2 computed earlier). If too many iterations, trajectory
; computation has failed to converge and program alarm is triggered.
;
		BMN	SLOAD		; Branch if counter negative (OK)
			RTE380		; Continue iteration
			OCT605		; Load alarm code 605
		GOTO			; Exceeded iteration limit!
			RTEALRM		# TOO MANY ITERATIONS
RTE380		STORE	NN1A		; Store updated counter
		DSU	BZE		; Check if counter = -8
			M8RTEB28	; Constant -8
			RTE385		; Use slow precision mode
		DLOAD	DSU		; Calculate refined adjustment
			00D		; Current X(T2) error
			DRCON		; Subtract previous delta RCON
;
; NORMAL MODE: RICHARDSON EXTRAPOLATION
;
; Use Richardson extrapolation to accelerate convergence. This computes
; a more sophisticated adjustment DX(T2) based on rate of change between
; iterations, allowing faster approach to solution.
;
; Compute: DX(T2) = X(T2)ERR * (Z2/Z1) where
;   Z1 = X(T2)ERR - previous X(T2)ERR
;   Z2 = X(T2)PRI - X(T2)
;
		NORM	PDDL		# X(T2)ERR-X(T2)ERR,=Z1			PL06D
			X1		; Normalization shift count
			RPRE'		; Previous X(T2) value
		DSU	DDV		# X(T2)PRI-X(T2)=Z2			PL04D
			X(T2)		; Current X(T2)
		DMP	SL*		# DX(T2)=X(T2)ERR(Z2/Z1)
			00D		; Multiply by X(T2) error
			0,1		; Shift by normalization count
		GOTO
			RTE390		; Continue with adjustment
;
; SLOW MODE: SIMPLE ERROR ADJUSTMENT
;
; After 8 iterations, switch to simple adjustment to ensure convergence
; even for difficult cases. Just use X(T2) error directly as adjustment.
;
RTE385		DLOAD			# DX(T2)=X(T2)ERR
			00D		; Use X(T2) error as-is
RTE390		STODL	16D		# DX(T2)				PL02D
		STADR			; Store address for later
		STODL	RCON		# RCON=RCON,
		BOV			; Check for overflow
# Page 895
			RTE360		; Overflow - restart iteration
;
; UPDATE ITERATION PARAMETERS
;
; Store current values for next iteration's comparison:
; - DRCON stores current RCON change for Richardson extrapolation
; - RPRE' stores current X(T2) for next iteration's rate calculation
;
; Then update X(T2) with the computed adjustment and reiterate.
;
		STODL	DRCON		# X(T2)ERR,=X(T2)ERR
			X(T2)		; Current X(T2)
		STODL	RPRE'		# X(T2)PRI=X(T2)
			16D		; Load DX(T2) adjustment
		DAD			; Add adjustment to X(T2)
			X(T2)		; Current value
		STCALL	X(T2)		# X(T2)=X(T2)+DX(T2)
			RTE360		# REITERATE - loop back
;
; ============================================================================
; TRANSITION: From Iterative Convergence to Display and Precision Computation
;
; The conic trajectory has converged to a valid Earth return solution.
; Display the initial conic results to crew, then compute high-precision
; trajectory accounting for Earth oblateness and perturbations for final
; verification before committing to the burn.
; ============================================================================
;
P37E		CALL			# DISPLAY CONIC SOLUTION
			RTEVN		; Display velocity, time, gamma
;
; ENTRY GEOMETRY DETERMINATION
;
; Determine whether spacecraft will enter atmosphere near apogee or perigee
; of return trajectory. This affects the sign of PHI2 angle used in
; precision trajectory integration.
;
; Test: PCON*BETA1 compared to RCON
;   If difference near zero: entry at perigee
;   Otherwise: entry at apogee
;
RTE505		DLOAD	DMP		; Compute PCON * BETA1
			PCON		; Perigee conic parameter
			BETA1		; Beta angle parameter
		BDSU	BZE		; Subtract RCON and test
			RCON		; Reference conic parameter
			RTE510		; Zero - entry at perigee
		BMN	DLOAD		; Negative - entry at perigee
			RTE510		; Perigee entry path
			1RTEB2		; Load +1 constant
		GOTO			# ENTRY NEAR APOGEE
			RTE515		; Continue with positive PHI2
RTE510		DLOAD	DCOMP		# ENTRY NEAR PERIGEE
			1RTEB2		; Load +1 and complement to -1
;
; PRECISION TRAJECTORY COMPUTATION
;
; Now compute high-precision trajectory accounting for:
; - Earth oblateness (J2 gravitational harmonic)
; - Lunar and solar gravitational perturbations
; - Numerical integration accuracy
;
; This refines the conic solution to verify trajectory stays within
; entry corridor with actual perturbing forces included.
;
RTE515		STCALL	PHI2		; Store PHI2 entry geometry flag
			PREC100		# PRECISION TRAJECTORY COMPUTATION
RTE625		BZE
			P37G
RTEALRM		CALL
			P370ALRM
		EXIT
		TCF	P37		# RECYCLE AFTER ALARM DISPLAY

# RETURN TO EARTH DISPLAY SUBROUTINE

RTEVN		STQ	CALL
			VNSTORE
			RTEDISP		# DISPLAY PREPARATION
		EXIT
		CAF	V6N61RTE	# LATITUDE,LONGITUDE,BLANK
		TCR	P370GOFR	#   IN LAT(SPL),LNG(SPL),-
		CAF	FOUR
		TCR	37BLANK +1
		TCF	+5
		TCF	P37		# RECYCLE
		CAF	V6N39RTE	# T21 HRS,MIN,SEC IN T3TOT4
		TCR	P370GOF
		TCF	P37		# RECYCLE
		CAF	V6N60RTE	# DISPLAY BLANK,V(T2),FPA2
		TCR	P37GFRB1	#   IN -,VPRED,GAMMAEI
# Page 896
		TCF	P37		# RECYCLE
		CAF	V6N81RTE	# DISPLAY DELTA V (LV) IN DELVLVC
		TCR	P370GOF
		TCF	P37		# RECYCLE
		TCR	INTPRET
		GOTO
			VNSTORE

# PRECISION DISPLAY, TARGETING COMPUTATION AND RTE END PROCESSING

P37G		CALL
			RTEVN
		EXIT
P37N		CAF	SEVEN
		TS	OPTION1
		CAF	ONE
		TS	OPTION2
		CAF	V4N06RTE	# DISPLAY RCS OR SPS OPTION  SPS ASSUMED
		TCR	P370GOF
		TCF	-2		# RECYCLE
		TC	INTPRET		# PROCEED
		SETPD	SLOAD
			00D
			OPTION2
		DSU	BZE
			1RTEB13
			P37Q
		SLOAD	NORM		# SPS
			EMDOT
			X1
		PDDL	GOTO
			VCSPS
			P37T
P37Q		DLOAD	BON		# RCS
			MDOTRCS
			NJETSFLG
			P37R
		SL1
P37R		SL1
		NORM	PDDL
			X1
			VCRCS
P37T		PDDL	DDV		# DV/VC			B7 -B5 = B2 	PL02D
			DV
		EXIT
		TC	POLY
		DEC	1
		2DEC	5.66240507 E-4 B-3
		2DEC	9.79487897 E-1 B-1
# Page 897
		2DEC	-.388281955 B1
		TC	INTPRET
		PUSH	SLOAD		# (1-E)**(-DV/VC)=A		B3 	PL04D
			WEIGHT/G
		DMP	DDV		# DTB=(M0/MDOT)A	B16+B3-B3=B16 	PL00D
		SL*	DMP
			0 -12D,1
			CSUBT
		BDSU
			T1
		STORE	TIG		# TIG=T1-CT*DTB			B28
		EXIT
		CAF	V6N33RTE	# DISPLAY BIASED TIG
		TCR	P370GOF
		TCF	-2
		CAF	ZERO
		TS	VHFCNT
		TS	TRKMKCNT
		TC	INTPRET
		CALL			# CONICALLY INTEGRATE FROM R1,V1 OVER T12
			RTENCK1
		VLOAD	UNIT		#					PL00D
			R(T2)/
		PDVL	VXSC		# UR2				B1 	PL06D
			UR1/
			MCOS7.5
		PDVL	VXSC		# -UR1(COS7.5)			B1 	PL12D
			UH/
			MSIN7.5
		VAD	DOT		# K/=-UR1(COS7.5)-UH(SIN7.5)	B2 	PL00D
		DAD	BMN
			MCOS22.5
			P37W
		VLOAD	DOT		# K/ . UR2 GR COS22.5
			UH/
			R(T2)/
		BMN	DLOAD
			P37U
			THETA165
		PUSH	GOTO
			P37V
P37U		DLOAD	PUSH
			THETA210
P37V		SIN
		STODL	SNTH
		COS	CLEAR
			RVSW
		STOVL	CSTH
			R(T1)/
# Page 898
		STOVL	RVEC
			V2(T1)/
		STCALL	VVEC
			TIMETHET
P37W		CLEAR	CLEAR
			XDELVFLG
			NORMSW
		SET	VLOAD
			FINALFLG
		STADR
		STODL	RTARG
			T
		DAD
			T1
		STOVL	TPASS4
			V2(T1)/
		VSU
			V(T1)/
		STCALL	DELVSIN
			VN1645
		GOTO
			P37W

# SUBROUTINE TO GO TO GOFLASHR AND BLANK R1

P37GFRB1	EXTEND
		QXCH	SPRTEX
		TCR	P370GOFR
37BLANK		CAF	ONE
		TCR	BLANKET
		TCF	ENDOFJOB
		TC	SPRTEX		# RECYCLE
		TCF	P37PROC		# PROCEED

# SUBROUTINE TO GO TO GOFLASHR

P370GOFR	EXTEND
		QXCH	RTENCKEX
		TCR	BANKCALL
		CADR	GOFLASHR
		TCF	GOTOPOOH	# TERMINATE
		TCF	+3
		TCF	+4
		TC	RTENCKEX	# IMMEDIATE RETURN
		INDEX	RTENCKEX	# PROCEED
		TCF	0 +4
		INDEX	RTENCKEX	# RECYCLE
		TCF	0 +3

# SUBROUTINE TO GO TO GOFLASH

# Page 899
P370GOF		EXTEND
		QXCH	SPRTEX
		TCR	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH
		TCF	+2
		TC	SPRTEX
P37PROC		INDEX	SPRTEX
		TCF	0 +1
V6N33RTE	VN	0633
V4N06RTE	VN	0406
V6N61RTE	VN	0661
V6N39RTE	VN	0639
V6N60RTE	VN	0660
V6N81RTE	VN	0681
		BANK	32
		SETLOC	RTE
		BANK
		COUNT	32/RTE

# Page 900
# ALARM DISPLAY SUBROUTINE

P370ALRM	STQ	EXIT
			SPRTEX
		CA	MPAC
		TC	VARALARM
		CAF	V5N09RTE
		TC	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH
		TCF	-4
		TC	INTPRET
		GOTO
			SPRTEX
V5N09RTE	VN	0509

# Page 901
# TIME RADIUS CALLING SUBROUTINE
#
# INPUT
#	RVEC		INITIAL POSITION VECTOR					VECTOR	B29	METERS
#	VVEC		INITIAL VELOCITY VECTOR					VECTOR	B7	METERS/CS
#	RDESIRED	FINAL RADIUS FOR WHICH TRANSFER TIME IS TO BE COMPUTED	DP	B29	METERS
#	CONICX1		X1 SETTING FOR CONIC SUBROUTINES  -2=EARTH		SP	B14
#
# OUTPUT
#	R(T2)/		FINAL POSITION VECTOR					VECTOR	B29 	METERS
#	V(T2)/		FINAL VELOCITY VECTOR					VECTOR	B7	METERS/CS
#	T12		TRANSFER TIME TO FINAL RADIUS				DP	B28	CS

TMRAD100	STQ	CLEAR
			RTENCKEX
			RVSW
		AXC,2	SXA,2
		OCT	20000
			SGNRDOT
		LXC,1	CALL
			CONICX1
			TIMERAD
		STOVL	V(T2)/							PL00D
		STADR
		STODL	R(T2)/
			T
		STCALL	T12
			RTENCKEX

# Page 902
# DISPLAY CALCULATION SUBROUTINE
#
# DESCRIPTION
#	OUTPUT FOR DISPLAY IS CONVERTED TO PROPER UNITS AND PLACED IN OUTPUT STORAGE REGISTERS.  LANDING SITE
#	COMPUTATION FOR DETERMINING LANDING SITE LATITUDE AND LONGITUDE IS INCLUDED IN THE ROUTINE.
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		RTEDISP
#
# SUBROUTINES CALLED
#	TMRAD100
#	AUGEKUGL
#	LAT-LONG
#
# ERASABLE INITIALIZATION REQUIRED
#    PUSHLIST
#	NONE
#    MPAC
#	NONE
#    OTHER
#	R(T2)/		FINAL POSITION VECTOR					VECTOR	B29	METERS
#	V(T2)/		FINAL VELOCITY VECTOR					VECTOR	B7	METERS/CS
#	T2		FINAL TIME						DP	B28	CS
#	V2(T1)/		POST IMPULSE INITIAL VELOCITY VECTOR			VECTOR	B7	METERS/CS
#	V(T1)/		INITIAL VELOCITY VECTOR					VECTOR	B7
#	UR1/		UNIT INITIAL VECTOR					VECTOR	B1
#	UH/		UNIT HORIZONTAL VECTOR					VECTOR	B1
#
# OUTPUT
#	VPRED		VELOCITY MAGNITUDE AT 400,000 FT. ENTRY ALTITUDE	DP	B7	METERS/CS
#	T3TOT4		TRANSIT TIME TO 400,000 FT. ENTRY ALTITUDE		DP	B28	CS
#	GAMMAEI		FLIGHT PATH ANGLE AT 400,000 FT. ENTRY ALTITUDE		DP	B0	REVS + ABOVE HORIZ
#	DELVLVC		INITIAL VELOCITY CHANGE VECTOR IN LOCAL VERTICAL COORD.	VECTOR	B7	METERS/CS
#	LAT(SPL)	LATITUDE OF THE LANDING SITE				DP	B0	REVS
#	LNG(SPL)	LONGITUDE OF THE LANDING SITE				DP	B0	REVS

RTEDISP		STQ	VLOAD		# DISPLAY
			SPRTEX
			V(T2)/
		UNIT	PDDL
			36D
		STODL	VPRED		# V(T2)
			T2
		DSU
			SPRTETIG
		STOVL	T3TOT4		# T21
			R(T2)/
		UNIT	DOT
		SL1

# Page 903
		ARCCOS	BDSU
			1RTEB2
		STOVL	GAMMAEI		# FLIGHT PATH ANGLE T2
			V2(T1)/
		VSU	PUSH
			V(T1)/
		DOT	DCOMP
			UR1/
		PDVL	PUSH
		DLOAD	PDVL
			ZERORTE
		DOT	VDEF
			UH/
		VSL1
		STODL	DELVLVC
			DELVLVC
		BOFF	DCOMP
			RETROFLG
			RTD18
		STORE	DELVLVC		# NEGATE X COMPONENT, RETROGRADE
RTD18		VLOAD	ABVAL
			DELVLVC
		STOVL	VGDISP
			R(T2)/
		STORE	RVEC		# ***** LANDING SITE COMPUTATION *****
		ABVAL	DSU
			30480RTE
		STOVL	RDESIRED
			V(T2)/
		STCALL	VVEC
			TMRAD100	# R3,V3,T23 FROM TIMERAD
		VLOAD	UNIT
			R(T2)/
		PDVL	UNIT		# UR3					PL06D
			V(T2)/
		DOT	SL1		# GAMMAE=ARCSIN(UR3 . UV3)		PL00D
		ARCSIN	PDDL		# V(T3)					PL02D
			36D
		PDDL	ABS
		PUSH	CALL		# /GAMMAE/				PL04D
			AUGEKUGL	# PHIE					PL06D
		DAD	DAD
			T12		# T23
			T2
		STORE	02D		# T(LS)=T2&T23&TE
		SLOAD	BZE
			P37RANGE
			RTD22
		STORE	04D		# OVERRIDE RANGE (PCR 261)
RTD22		DLOAD	SIN

# Page 904
			04D
		STODL	LNG(SPL)	# LNG(SPL)=SIN(PHIE)			PL04D
		COS
		STORE	LAT(SPL)	# LAT(SPL)=COS(PHIE)
		VLOAD	UNIT
			R(T2)/
		PUSH	PUSH
		PDVL	UNIT		#					PL22D
			V(T2)/
		PDVL	VXV
		VXV	UNIT		# UH3=UNIT(UR3 X UV3 X UR3)		PL10D
		VXSC	PDVL
			LNG(SPL)
		VXSC	VAD		#					PL04D
			LAT(SPL)
		CLEAR	CLEAR		# T(LS) IN MPAC
			ERADFLAG
			LUNAFLAG
		STODL	ALPHAV		# ALPHAV=UR3(COSPHIE)+UH3(SINPHIE) 	PL02D
		CALL
			LAT-LONG
		DLOAD
			LAT
		STODL	LAT(SPL)	# LATITUDE LANDING SITE  *****
			LONG
		STCALL	LNG(SPL)	# LONGITUDE LANDING SITE *****
			SPRTEX
		COUNT*	$$/RTE

# Page 905
# INITIAL VECTOR SUBROUTINE
#
# DESCRIPTION
#	A PRECISION INTEGRATION OF THE STATE VECTOR TO THE TIME OF IGNITION IS PERFORMED. PRECOMPUTATIONS OCCUR.
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		INVC100
#
# NORMAL EXIT MODE
#	AT L+2 OF CALLING SEQUENCE WITH MPAC = 0
#
# ALARM EXIT MODE
#	AT L+2 OF CALLING SEQUENCE WITH MPAC = OCTAL 612 FOR STATE VECTOR IN MOONS SPHERE OF INFLUENCE
#
# SUBROUTINES CALLED
#	CSMPREC
#
# ERASABLE INITIALIZATION REQUIRED
#    PUSHLIST
#	NONE
#    MPAC
#	NONE
#    OTHER
#	SPRTETIG	TIME OF IGNITION					DP	B28	CS
#	CSM STATE VECTOR
#
# OUTPUT
#	R(T1)/		INITIAL POSITION VECTOR AT TIG				VECTOR	B29	METERS
#	V(T1)/		INITIAL VELOCITY VECTOR AT TIG				VECTOR	B7	METERS/CS
#	T1		INITIAL VECTOR TIME (TIG)				DP	B28	CS
#	UR1/		UNIT INITIAL VECTOR					VECTOR	B1
#	UH/		UNIT HORIZONTAL VECTOR					VECTOR	B1
#	CFPA		COSINE OF INITIAL FLIGHT PATH ANGLE			DP	B1

INVC100		STQ	DLOAD
			SPRTEX
			SPRTETIG
		STCALL	TDEC1
			CSMPREC		# PRECISION INTEGRATION  R0,V0 TO R1,V1
		VLOAD	SXA,2
			RATT
			P(T1)
		STOVL	R(T1)/
			VATT
		STODL	V(T1)/
			TAT
		STORE	T1
		SLOAD	BZE
			P(T1)
# Page 906
			INVC109
INVC107		SLOAD	GOTO
			OCT612
			RTEALRM		# R1,V1 NOT IN PROPER SPHERE OF INFLUENCE
INVC109		VLOAD	UNIT
			R(T1)/
		STODL	UR1/		# UR1/					B1
			36D
		STOVL	R(T1)		# R(T1)					B29
			V(T1)/
		UNIT
		STORE	UV1/
		DOT	SL1
			UR1/
		STORE	CFPA		# CFPA					B1
		ABS	DSU
			EPC1RTE
		BMN	DLOAD
			INVC115		# NOT NEAR RECTILINEAR
			1RTEB2
		PDDL	PUSH
			ZERORTE
		VDEF	PUSH		# N/ = (0,0,1)
		GOTO
			INVC120
INVC115		VLOAD	VXV
			UR1/
			UV1/
		PUSH			# N/ = UR X UV				B2
INVC120		CLEAR	DLOAD
			RETROFLG
		PUSH	BPL
			INVC125
		VLOAD	VCOMP		# RETROGRADE ORBIT
		PUSH	SET
			RETROFLG
INVC125		VLOAD
		VXV	UNIT
			UR1/
		STORE	UH/		# UH/					B1
		GOTO
			SPRTEX

# Page 907
# PRECISION TRAJECTORY COMPUTATION SUBROUTINE
#
# DESCRIPTION
#	A NUMERICALLY INTEGRATED TRAJECTORY IS GENERATED WHICH FOR THE RETURN TO EARTH PROBLEM SATISFIES THE REENTRY
#	CONSTRAINTS (RCON AND X(T2)) ACHIEVED BY THE INITIAL CONIC TRAJECTORY AND MEETS THE DVD REQUIREMENT AS CLOSELY
#	AS POSSIBLE.
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		PREC100
#
# NORMAL EXIT MODE
#	AT L+2 OF CALLING SEQUENCE WITH MPAC = 0
#
# ALARM EXIT MODE
#	AT L+2 OF CALLING SEQUENCE WITH MPAC =
#		OCTAL 605	FOR EXCESS ITERATIONS
#		OCTAL 613	FOR REENTRY ANGLE OUT OF LIMITS
#
# SUBROUTINES CALLED
#	INTSTALL
#	RTENCK2
#	RTENCK3
#	TIMERAD
#	PARAM
#	V2T100
#
# ERASABLE INITIALIZATION REQUIRED
#    PUSHLIST
#	NONE
#    MPAC
#	NONE
#    OTHER
#	R(T1)/		INITIAL POSITION VECTOR					VECTOR	B29/B27	METERS
#	V2(T1)/		POST IMPULSE INITIAL VELOCITY VECTOR			VECTOR	B7/B5	METERS/CS
#	V(T1)/		INITIAL VELOCITY VECTOR					VECTOR	B7/B5	METERS/CS
#	T1		INITIAL VECTOR TIME					DP	B28	CS
#	T12		INITIAL TO FINAL POSITION TIME				DP	B28 	CS
#	RCON		CONIC FINAL RADIUS					DP	B29/B27	METERS
#	R(T1)		MAGNITUDE OF INITIAL POSITION VECTOR			DP	B29/B27	METERS
#	X(T2)		COTANGENT OF FINAL FLIGHT PATH ANGLE			DP	B0
#	X(T1)		COTANGENT OF INITIAL FLIGHT PATH ANGLE			DP	B5
#	RTEDVD		DELTA VELOCITY DESIRED					DP	B7/B5	METERS/CS
#	MAMAX1		MAJOR AXIS LIMIT FOR LOWER BOUND ON GAMDV ITERATOR	DP	B30/B28	METERS
#	MAMAX2		MAJOR AXIS LIMIT FOR UPPER BOUND ON GAMDV ITERATOR	DP	B30/B28	METERS
#	UR1/		UNIT INITIAL VECTOR					VECTOR	B1
#	UH/		UNIT HORIZONTAL VECTOR					VECTOR	B1
#	BETA1		1+X(T2)**2						DP	B1
#	PHI2		PERIGEE OR APOGEE INDICATOR				DP	B2	-1 PERIGEE, +1 APOGEE
#
# Page 908
#
# OUTPUT
#    	V2(T1)/		POST IMPULSE INITIAL VELOCITY VECTOR			VECTOR	B7	METERS/CS
#	R(T2)/		FINAL POSITION VECTOR					VECTOR	B29	METERS
#	V(T2)/		FINAL VELOCITY VECTOR					VECTOR	B7	METERS/CS
#	T2		FINAL TIME						DP	B28	CENTISECONDS
#
# DEBRIS
#	RD		FINAL R DESIRED						DP	B29/B27	METERS
#	R/APRE		R/A							DP	B6
#	P/RPRE		P/R							DP	B4
#	RPRE		MAGNITUDE OF R(T2)/					DP	B29/B27	METERS
#	X(T2)PRE	COTANGENT OF GAMMA2					DP	B0
#	DT12		CORRECTION TO FINAL TIME T2				DP	B28	CENTISECONDS
#	RCON		FINAL RADIUS						DP	B29/B27	METERS
#	DRCON		DELTA RCON						DP	B29/B27	METERS

PREC100		STQ	DLOAD
			SPRTEX
			10RTE
		STODL	NN1A
			RCON
		STORE	RD
PREC120		DLOAD
			2RTEB1
		STODL	DT21PR		# DT21PR = POSMAX
			M15RTE
		STCALL	NN2
			RTENCK3
PREC125		CALL
			PARAM
		DLOAD
			P
		STODL	P/RPRE
			R1A
		STODL	R/APRE
			R1
		STODL	RPRE
			COGA
		SL
			5
		STORE	X(T2)PRE
		DCOMP	DAD
			X(T2)
		ABS	DSU
			EPC4RTE
		BOV	BMN
			PREC130
			PREC175

# DESIRED REENTRY ANGLE NOT ACHIEVED

# Page 909
PREC130		DLOAD	BMN
			NN2
			PREC140
PREC132		SLOAD	GOTO		# TOO MANY ITERATIONS
			OCT605		#	EXIT WITH ALARM
			PRECX

# DETERMINE RADIUS AT WHICH THE DESIRED REENTRY ANGLE WILL BE ACHIEVED

PREC140		DLOAD	BZE
			NN1A
			PREC162
PREC150		DLOAD	SL2						B2
			P/RPRE
		DMP	SL1		# BETA2=BETA1*P/R		B2	PL02
			BETA1
		PUSH	DLOAD
			R/APRE
		SL4	DMP
			00D
		BDSU	BMN		# BETA3=1-BETA2*R/A
			1RTEB4
			PREC160
PREC155		SL2	SQRT
		DMP	BDSU
			PHI2
			1RTEB3
		NORM	PDDL
			X1
		SR1	DDV		# BETA4=BETA2/(1-PHI2*SQRT(BETA3))
		SL*	GOTO						B1
			0	-1,1
			PREC165
PREC160		DLOAD	NORM
			R/APRE
			X1
		BDDV	SL*						B1
			1RTEB1
			0	-6,1
		GOTO
			PREC165
PREC162		DLOAD	NORM
			RPRE
			X1
		BDDV	SL*		# BETA4=RD/RPRE			B1
			RD
			0 -1,1
PREC165		SETPD	PUSH
			0
		DSU	DCOMP
# Page 910
			1RTEB1
		STORE	BETA12
		BMN	DLOAD
			PREC168
			X(T2)PRE
		BMN	DLOAD
			PREC167
			BETA12
		DCOMP
		STORE	BETA12
PREC167		DLOAD
			BETA12
PREC168		ABS	DSU
			EPC6RTE
		BMN	DLOAD
			PREC175
		DMP	SL1
			RPRE
		PUSH			# RF = NEW RADIUS
PREC170		DLOAD	DAD
			NN2
			1RTEB28
		STORE	NN2
		VLOAD	SET
			R(T2)/
			RVSW
		STOVL	RVEC
			V(T2)/
		SIGN
			BETA12
		STODL	VVEC
			1RTEB1
		SIGN	DCOMP
			BETA12
		LXA,2	DLOAD
			MPAC
		LXC,1	SXA,2
			CONICX1
			SGNRDOT
		STCALL	RDESIRED	# COMPUTED DT12 (CORRECTION TO TIME OF
			TIMERAD		#	NEW RADIUS)
		DLOAD	SIGN
			T
			BETA12
		PDDL	NORM		# DT21=(PHI4)DT21			PL02D
			DT21PR
			X1
		BDDV	SL*
			00D
			0 -3,1
# Page 911
		PUSH	BMN		# BETA13=(DT21)/(DT21PR)	R3 	PL04D
			PREC172
		DLOAD	PDDL		# BETA14=1			B0 	PL04D
			2RTEB1
		GOTO
			PREC173
PREC172		DLOAD	PDDL		# BETA14=.6			B0 	PL04D
			M.6RTE
PREC173		DDV	DSU
			02D
			1RTEB3
		BMN	DLOAD
			PREC174
		DMP
			DT21PR
		STORE	00D		# DT21=(BETA14)DT21PR		B28
PREC174		DLOAD	PUSH
			00D
		STCALL	DT21PR
			RTENCK2
		GOTO
			PREC125
PREC175		DLOAD	DSU
			RPRE
			RD
		PUSH	ABS		# RPRE-RD = RERR
		DSU	BMN
			EPC7RTE
			PREC220

# DESIRED RADIUS HAS NOT BEEN ACHIEVED

		DLOAD	BZE
			NN1A
			PREC132		# TOO MANY ITERATIONS
		DSU	BZE
			10RTE
			PREC207
PREC205		DLOAD	DSU		# NOT FIRST PASS OF ITERATION
			RPRE'
			RPRE		# RPRE'-RPRE			B29/B27
		NORM	BDDV
			X2
			DRCON
		SL*	PUSH		# DRCON/(RPRE'-RPRE)=S		B2
			0 -2,2
		DAD	BOV		# S GR +4 OR LS -4
			1RTEB1
			PREC205M
		ABS	DSU
# Page 912
			1RTEB1
		BMN
			PREC206
PREC205M	DLOAD	DCOMP		# S GR 0 OR LS -4
			2RTEB1
		PDDL			# S=-4				B2
PREC206		DLOAD	DMP
		SL2
		STORE	DRCON		# DRCON=S(RERR)			B29
		DAD
			RCON
		STORE	RCON		# RCON+DRCON=RCON
		GOTO
			PREC210
PREC207		DLOAD	DSQ		# FIRST PASS OF ITERATION
			RD
		NORM	SR1
			X1
		PDDL	NORM
			RPRE
			X2
		XSU,1	BDDV
			X2
		SR*
			0 -1,1
		STORE	RCON		# RD**2/RPRE=RCON
		DSU
			RD
		STORE	DRCON		# RCON-RD=DRCON
PREC210		DLOAD			# PREPARE FOR NEXT ITERATION
			RPRE
		STODL	RPRE'
			NN1A
		DSU
			1RTEB28
		STCALL	NN1A
			V2T100
		BHIZ	GOTO
			PREC120
			PRECX

# DESIRED RADIUS ACHIEVED

		SETLOC	RTE2
		BANK
PREC220		DLOAD	DSU
			X(T2)
			X(T2)PRE
		ABS	DSU
			EPC8RTE
# Page 913
		BMN	SLOAD
			PREC225
			OCT613
		GOTO
			PRECX		# IF REENTRY ANGLE OUT OF LIMITS

EPC8RTE		2DEC	.002

OCT613		OCT	613

# DESIRED FINAL ANGLE HAS BEEN REACHED.

		SETLOC	RTE
		BANK
PREC225		DLOAD
			ZERORTE
PRECX		GOTO
			SPRTEX

# Page 914
# INTEGRATION CALLING SUBROUTINE
#
# DESCRIPTION
#	PERFORMS CONIC AND PRECISION INTEGRATIONS USING SUBROUTINE INTEGRVS.  THERE ARE THREE ENTRANCES (RTENCK1,
#	RTENCK2, AND RTENCK3) FOR DIFFERENT SOURCES OF INPUT AND DIFFERENT OPTIONS.  THERE IS A COMMON SET OF OUTPUT
# 	WHICH INCLUDES SET UP OF INPUT FOR THE PARAM SUBROUTINE.
#
# RTENCK1 (CONIC INTEGRATION)
#
#    CALLING SEQUENCE
#	L	CALL
#	L+1		RTENCK1
#
#    ERASABLE INITIALIZATION REQUIRED
#	SAME AS FOR THE RTENCK3 ENTRANCE
#
# RTENCK2 (PRECISION INTEGRATION)
#
#    CALLING SEQUENCE
#	L	CALL
#	L+1		RTENCK2
#
#    ERASABLE INITIALIZATION REQUIRED
#	PUSHLIST
#	    PUSHLOC-2	INTEGRATION TIME DT12 (CORRECTION TO T2)		DP	B28	CS
#	OTHER
#	    R(T2)/	FINAL POSITION VECTOR					VECTOR	B29	METERS
#	    V(T2)/	FINAL VELOCITY VECTOR					VECTOR	B7	METERS/CS
#	    T2		FINAL TIME						DP	B28	CS
#
# RTENCK3 (PRECISION INTEGRATION)
#
#    CALLING SEQUENCE
#	L	CALL
#	L+1		RTENCK3
#
#    ERASABLE INITIALIZATION REQUIRED
#	R(T1)/		INITIAL POSITION VECTOR					VECTOR	B29	METERS
#	V2(T1)/		POST IMPULSE INITIAL VELOCITY VECTOR			VECTOR	B7	M/CS
#	T1		INITIAL VECTOR TIME					DP	B28	CS
#	T2		FINAL TIME						DP	B28	CS
#
# EXIT MODE
#	AT L+2 OF CALLING SEQUENCE
#
# SUBROUTINES CALLED
#	INTSTALL
#	INTEGRVS
#
# OUTPUT
#    PUSHLIST
# Page 915
#	PUSHLOC-6	FINAL POSITION VECTOR R(T2)/				VECTOR	B29	METERS
#	X1		CONICS MUTABLE ENTRY FOR EARTH (-2)			SP	B14
#    MPAC
#			FINAL VELOCITY VECTOR V(T2)/				VECTOR	B7	M/CS
#    OTHER
#	R(T2)/		AS IN PUSHLIST
#	V(T2)/		AS IN MPAC
#	T2		FINAL TIME						DP	B28	CS

		SETLOC	RTE3
		BANK
RTENCK1		STQ	CALL
			RTENCKEX
			INTSTALL
		VLOAD	SET
			R(T1)/
			INTYPFLG
		GOTO
			RTENCK3B

RTENCK2		STQ	CALL
			RTENCKEX
			INTSTALL
		CLEAR	VLOAD
			INTYPFLG
			R(T2)/
		STOVL	RCV
			V(T2)/
		STODL	VCV
			T2
		STORE	TET
		DAD
		GOTO
			RTENCK3D

RTENCK3		STQ	CALL
			RTENCKEX
			INTSTALL
RTENCK3A	VLOAD	CLEAR
			R(T1)/
			INTYPFLG
RTENCK3B	STOVL	RCV
			V2(T1)/
		STODL	VCV
			T1
		STODL	TET
			T2
# Page 916
RTENCK3D	STORE	TDEC1
		CLEAR	CALL
			MOONFLAG
			INTEGRVS
		VLOAD
			RATT
		STORE	R(T2)/
		PDDL	LXC,1
			TAT
			CONICX1
		STOVL	T2
			VATT
		STORE	V(T2)/
		GOTO
			RTENCKEX
		SETLOC	RTE
		BANK

# Page 917
# V2(T1) COMPUTATION SUBROUTINE
#
# DESCRIPTION
#	A POST IMPULSE VELOCITY VECTOR (V2(T1)) IS COMPUTED WHICH EITHER
#	(1)	MEETS THE INPUT VELOCITY CHANGE DESIRED (RTEDVD) IN A MINIMUM TIME	OR
#	(2)	IF A VELOCITY CHANGE ISN'T SPECIFIED (RTEDVD = 0), A V2(T1) IS COMPUTED WHICH MINIMIZES THE IMPULSE (DV)
#		AND CONSEQUENTLY FUEL.
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		V2T100
#
# NORMAL EXIT MODE
#	AT L+2 OF CALLING SEQUENCE WITH MPAC = 0
#
# ALARM EXIT MODE
#	AT L+2 OF CALLING SEQUENCE WITH MPAC = OCTAL 605 FOR EXCESS ITERATIONS.
#
# SUBROUTINES CALLED
#	GAMDV10
#	XT1LIM
#	DVCALC
#
# ERASABLE INITIALIZATION REQUIRED
#    PUSHLIST
#	NONE
#    MPAC
#	NONE
#    OTHER
#	R(T1)		MAGNITUDE OF INITIAL POSITION VECTOR			DP	B29/B27	METERS
#	RCON		MAGNITUDE OF FINAL POSITION VECTOR			DP	B29/B27	METERS
#	V(T1)/		INITIAL VELOCITY VECTOR					VECTOR	B7/B5	METERS/CS
#	RTEDVD		DELTA VELOCITY DESIRED					DP	B7/B5	METERS/CS
#	UR1/		UNIT INITIAL VECTOR					VECTOR	B1
#	UH/		UNIT HORIZONTAL VECTOR					VECTOR	B1
#	X(T2)		COTANGENT OF FINAL FLIGHT PATH ANGLE			DP	B0
#	X(T1)		COTANGENT OF INITIAL FLIGHT PATH ANGLE (INPUT FOR PREC)	DP	B5
#	CFPA		COSINE OF INITIAL FLIGHT PATH ANGLE			DP	B1
#	MAMAX1		MAJOR AXIS LIMIT FOR LOWER BOUND ON GAMDV ITERATOR	DP	B30/B28	METERS
#	MAMAX2		MAJOR AXIS LIMIT FOR UPPER BOUND ON GAMDV ITERATOR	DP	B30/B28	METERS
#	PHI2		REENTRY NEAR PERIGEE OR APOGEE INDICATE (RTE ONLY)	DP	B2	-1 PERIGEE, +1 APOGEE
#	N1		CONIC OR PRECISION ITERATION OPERATOR			DP	B28	NEGATIVE CONIC, PLUS PREC
#
# OUTPUT
#	V2(T1)/		POST IMPULSE INITIAL VELOCITY VECTOR			VECTOR	B7/B5	METERS/CS
#	DV		INITIAL VELOCITY CHANGE					DP	B7/B5	METERS/CS
#	X(T1)		COTANGENT OF INITIAL FLIGHT PATH ANGLE (POST IMPULSE)	DP	B5
#	PCON		SEMI-LATUS RECTUM					DP	B28/B26	METERS
#	BETA1		1+X(T2)**2						DP	B1
#
# Page 918
#
# DEBRIS
#    PUSHLIST
#	00D		X(T1),,=PREVIOUS PRECISION X(T1)			DP	B5
#	02D		THETA1=BETA5*LAMBDA-1					TP	B17
#	05D		THETA2=2*R(T1)*(LAMBDA-1)				TP	B38/B36
#	08D		THETA3=MU**.5/R(T1)					DP	B-4/B-5
#	10D		X(T1)MIN=LOWER BOUND ON X(T1) IN GAMDV ITERATOR		DP	B5
#	12D		DX(T1)MAX=MAXIMUM DELTA X(T1)				DP	B5
#	14D		X(T1)MAX=UPPER BOUND ON X(T1) IN GAMDV ITERATOR		DP	B5
#	16D		DX(T1)=ITERATOR INCREMENT				DP	B5
#	31D		GAMDV10 SUBROUTINE RETURN ADDRESS
#	32D		DVCALC SUBROUTINE RETURN ADDRESS
#	33D		V2T100 SUBROUTINE RETURN ADDRESS

V2T100		STQ	DLOAD
			33D
			RCON
		BMN	DSU		# ABORT IF RCON NEGATIVE
			V2TERROR
			R(T1)
		BMN
			V2T101
V2TERROR	EXIT			#	OR IF LAMBDA LESS THAN ONE
		TC	POODOO		# NO SOLUTION IF LAMBDA LESS THAN 1
		OCT	00610
V2T101		SETPD	CLEAR
			0		#					PL00D
			F2RTE
		DLOAD	NORM
			RCON
			X1
		PDDL	NORM
			R(T1)
			S1
		STORE	10D
		SR1	DDV		# R1/RCON = LAMBDA		B1
		XSU,1	PDDL		#					PL02D
			S1
			X(T2)
		DSQ
		SR1	DAD
			1RTEB1
		STORE	BETA1		# 1+X(T2)**2 = BETA1		B1
		DMP
			00D
		STORE	28D		# BETAI*LAMBDA = BETA5
		DMP	SL*
			00D
			0 -7,1
		SL*	DSU
# Page 919
			0 -7,1
			1RTEB17
		RTB	PDDL		# BETA5*LAMBDA-1 = THETA1	B17	PL05D
			TPMODE
			1RTEB1
		SR*	DCOMP
			0,1
		DAD	DMP
			00D
			R(T1)
		SL*	RTB
			0 -7D,1
			TPMODE
		PDDL			# 2*R(T1)*(LAMBDA-1)=THETA2	B38/B36 PL08D
			RTMURTE
		NORM	SR1
			X2
		XSU,2	DDV
			S1
			10D
		SR*	PDDL		# MU**.5/R(T1)=THETA3		B-4/B-5 PL10D
			6,2
			MAMAX1
		PUSH	PUSH		# MAMAX1=MA
		CALL
			XT1LIM
		DCOMP	PUSH		# X(T1)MIN			B5 	PL12D
		DCOMP	SR4
		PDDL	PUSH		# DX(T1)MAX			B5 	PL14D
			MAMAX2
		PUSH	CALL
			XT1LIM
		PDDL	BMN		# X(T1)MAX			B5 	PL16D
			NN1A
			V2T102
		GOTO
			V2T110

# PROCEED HERE IF NOT PRECISION COMPUTATION

V2T102		DLOAD
			RTEDVD
		BZE	GOTO
			V2T105
			V2T140
V2T105		DLOAD	BMN
			CFPA
			V2T140
		GOTO
			V2T145
# Page 920
# DURING A PRECISION TRAJECTORY ITERATION CONSTRAIN THE INDEPENDENT
# VARIABLE TO INSURE THAT ALL CONICS PASS THROUGH RCON ON THE SAME PASS
# THROUGH X(T2)

V2T110		DLOAD	RTB
			1RTEB17
			TPMODE
		DCOMP	PDDL		# -1				B17 	PL19D
			2RTEB1
		SR*	DSU
			0,1
			00D
		DMP	SL*
			28D
			0 -7,1
		SL*	TAD
			0 -7,1
		RTB	PDDL		# BETA5(2-LAMBDA)-1=BETA6	B17 	PL19D
			TPMODE
			X(T1)
		STORE	00D		# X(T1),,			B5
		TLOAD			#					PL16D
		BMN	BZE
			V2T115
			V2T115
		SL	GOTO
			7
			V2T120
V2T115		DLOAD	BMN
			PHI2
			V2T125
		DCOMP
		STODL	PHI2
			10RTE
		STORE	NN1A
		GOTO
			V2T125
V2T120		SQRT	RTB
			DPMODE
		PDDL	BMN		# BETA6**.5=X(T1)LIM		B5 	PL18D
			PHI2
			V2T130
		DLOAD	STADR
		STORE	14D		# X(T1)LIM = X(T1)MAX
		DCOMP
		STORE	10D		# -X(T1)LIM = X(T1)MIN
V2T125		DLOAD	BZE
			X(T1)
			V2T140
		BMN	GOTO
# Page 921
			V2T140
			V2T145
V2T130		DLOAD	BZE
			X(T1)
			V2T135
		BMN	DLOAD		#					PL16D
			V2T135
		STADR
		STORE	10D		# X(T1)LIM = X(T1)MIN
		GOTO
			V2T145
V2T135		DLOAD	DCOMP		#					PL16D
		STADR
		STORE	14D		# -X(T1)LIM = X(T1)MAX
V2T140		DLOAD
			10D
		STODL	X(T1)		# X(T1)MIN = X(T1)
			12D
		PUSH	GOTO		# DX(T1)MAX = DX(T1)			PL18D
			V2T150
V2T145		DLOAD
			14D
		STODL	X(T1)		# X(T1)MAX = X(T1)
			12D
		DCOMP	PUSH		# -DX(T1)MAX = DX(T1)			PL18D
V2T150		CALL			# GOTO X(T1)-DV ITERATOR
			GAMDV10
		DLOAD	BZE		# EXIT IF MINIMUM FUEL MODE
			RTEDVD
			V2T1X

# CONTINUE IF TIME CRITICAL MODE

		DSU	BMN
			DV
			V2T155
		GOTO
			V2T175
V2T155		DLOAD	BMN
			NN1A
			V2T160
		GOTO
			V2T185

# CONIC TRAJECTORY COMPUTATION

V2T160		DLOAD	BZE
			X(T1)
			V2T165
		BMN	GOTO
# Page 922
			V2T165
			V2T300
V2T165		DLOAD	BZE
			CFPA
			V2T300
		BMN	DLOAD
			V2T300
			14D
		STODL	X(T1)		# X(T1)MAX=X(T1)
			12D
		DCOMP
		STCALL	16D		# -DX(T1)MAX=DX(T1)
			GAMDV10
		DLOAD	DSU
			RTEDVD
			DV
		BMN
			V2T300
V2T175		SET	DLOAD
			F2RTE
			X(T1)
		BOFF
			SLOWFLG
			V2T177
		STODL	10D		# X(T1)MIN
			12D		# DX(T1)MAX
		GOTO
			V2T179
V2T177		STODL	14D
			12D
		DCOMP
V2T179		STCALL	16D		# DX(T1)
			GAMDV10
		DLOAD	BMN
			NN1A
			V2T300

# PREVENT A LARGE CHANGE IN INDEPENDENT VARIABLE DURING AN ITERATION FOR A
# PRECISION TRAJECTORY

V2T185		DLOAD	DSU
			X(T1)
			00D
		ABS	PDDL		# /X(T1)-X(T1),,/ = BETA7
			12D
		SL1	BDSU
		BMN	DLOAD
			V2T300
			00D		# CONTINUE IF BETA7 LARGER THAN 2DX(T1)MAX
		STORE	X(T1)		# X(T1),, = X(T1)
# Page 923
		DSU	BMN
			14D
			V2T195
		DLOAD
			14D
		STORE	X(T1)		# X(T1)MAX = X(T1)
		GOTO
			V2T205
V2T195		DLOAD	DSU
			X(T1)
			10D
		BMN	GOTO
			V2T200
			V2T205
V2T200		DLOAD
			10D
		STORE	X(T1)		# X(T1)MIN = X(T1)
V2T205		CALL
			DVCALC
V2T300		DLOAD
			ZERORTE
V2T1X		GOTO
			33D

# Page 924
# X(T1)-DV ITERATOR SUBROUTINE
#
# DESCRIPTION
#	COMPUTES A POST IMPULSE VELOCITY VECTOR (V2(T1)) WHICH REQUIRES A MINIMUM DV.
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		GAMDV10
#
# NORMAL EXIT MODE
#	AT L+2 OF CALLING SEQUENCE
#
# ALARM EXIT MODE
#	AT V2T1X WITH MPAC = OCTAL 605 FOR EXCESS ITERATIONS
#
# SUBROUTINES CALLED
#	DVCALC
#
# ERASABLE INITIALIZATION REQUIRED
#    PUSHLIST
#	02D		THETA1=BETA5*LAMBDA-1					TP	B17
#	05D		THETA2=2*R(T1)*(LAMBDA-1)				TP	B38/B36
#	08D		THETA3=MU**.5/R(T1)					DP	B-4/B-5
#	10D		X(T1)MIN=LOWER BOUND ON INDEPENDENT VARIABLE X(T1)	DP	B5
#	12D		DX(T1)MAX=MAXIMUM DX(T1)				DP	B5
#	14D		X(T1)MAX=UPPER BOUND ON INDEPENDENT VARIABLE X(T1)	DP	B5
#	16D		DX(T1)=ITERATOR INCREMENT				DP	B5
#    MPAC
#	NONE
#    OTHER
#	V(T1)/		INITIAL VELOCITY VECTOR					VECTOR	B7/B5	METERS/CS
#	RTEDVD		DELTA VELOCITY DESIRED					DP	B7/B5	METERS/CS
#	UR1/		UNIT INITIAL VECTOR					VECTOR	B1
#	UH/		UNIT HORIZONTAL VECTOR					VECTOR	B1
#	X(T1)		COTANGENT OF INITIAL FLIGHT PATH ANGLE (FROM VERTICAL)	DP	B5
#	F2RTE		TIME CRITICAL OR MINIMUM FUEL MODE INDICATOR		STATE AREA	0 MIN. FUEL, 1 MIN. TIME
#
# OUTPUT
#	V2(T1)/		POST IMPULSE INITIAL VELOCITY VECTOR			VECTOR	B7/B5	METERS/CS
#	DV		INITIAL VELOCITY CHANGE					DP	B7/B5	METERS/CS
#	X(T1)		COTANGENT OF INITIAL FPA MEASURED FROM VERTICAL		DP	B5
#	PCON		SEMI-LATUS RECTUM					DP	B28/B26	METERS
#
# DEBRIS
#    PUSHLIST
#	00D		X(T1),,
#	02D		THETA1
#	05D		THETA2
#	08D		THETA3
#	10D		X(T1)MIN
#	12D		DX(T1)MAX
# Page 925
#	14D		X(T1)MAX
#	16D		DX(T1)
#	22D		DV,=PREVIOUS DV						DP	B7/B5
#	24D		BETA9=X(T1)+1.1DX(T1)					DP	B5
#	31D		GAMDV10 SUBROUTINE RETURN ADDRESS
#	32D		DVCALC SUBROUTINE RETURN ADDRESS
#	33D		V2T100 SUBROUTINE RETURN ADDRESS

; ============================================================================
; GAMDV10: FLIGHT PATH ANGLE ITERATOR
;
; This iterator finds the optimal initial flight path angle (X(T1)) that 
; minimizes either fuel consumption or mission time for the TEI burn. For 
; Apollo 11's return journey, this algorithm computed the precise burn 
; attitude that would send Columbia on the fastest path home while staying 
; within the Earth entry corridor.
;
; The iterator uses a modified Newton-Raphson method to find X(T1) by:
; 1. Computing DV for current X(T1)
; 2. Comparing to desired DV or checking convergence
; 3. Adjusting X(T1) by step size DX(T1)
; 4. Repeating until convergence or iteration limit
;
; MINIMUM FUEL mode (F2RTE=0): Finds X(T1) that achieves desired DV exactly
; MINIMUM TIME mode (F2RTE=1): Finds X(T1) that maximizes velocity change
; ============================================================================

GAMDV10		STQ			; Store return address
			31D		; At location 31D in pushlist
		SETPD	CALL		; Initialize pushlist pointer
			18D		; PL18D
			DVCALC		; Compute initial DV for current X(T1)
;
; ITERATOR INITIALIZATION
;
; Check if X(T1) bounds are sufficiently separated to allow iteration.
; BETA8 = X(T1)MAX - X(T1)MIN represents the search space width.
;
		DLOAD	DSU		; Compute search space width
			14D		; X(T1)MAX
			10D		; X(T1)MIN
		BOV			; Branch if overflow
			GAMDV20		; Continue with current step
		PUSH	DSU		; BETA8 = X(T1)MAX - X(T1)MIN (PL20D)
			EPC9RTE		; BETA8 - convergence tolerance
		BMN	DLOAD		; If BETA8 < tolerance
			GAMDVX		; Bounds too close, exit
			18D		; Reload BETA8
		DSU	BMN		; BETA8 - DX(T1)MAX
			12D		; Maximum step size
			GAMDV15		; Step exceeds bounds, reduce
		SETPD	GOTO		; Keep current step size
			18D		; Reset pushlist pointer
			GAMDV20		; Begin iteration
GAMDV15		DLOAD			; Reduce step size (PL18D)
		SIGN	SR1		; BETA8 * SIGN(DX(T1)) / 2
			16D		; Preserve sign of DX(T1)
		STORE	16D		; Store reduced step: DX(T1) = BETA8/2
;
; MAIN ITERATION LOOP
;
; Iterates up to 144 times to find optimal X(T1). For Apollo 11's TEI burn
; on July 21, 1969, this loop typically converged in 10-20 iterations,
; computing the flight path angle that would bring Columbia safely home.
;
GAMDV20		DLOAD			; Initialize iteration counter
			M144RTE		; Load -144 (max iterations)
		STORE	NN2		; Store in counter
GAMDV25		DLOAD	DAD		; Increment iteration counter
			NN2		; Current count
			1RTEB28		; Add 1
		BMN	SLOAD		; If still negative (iterations remain)
			GAMDV30		; Continue iteration
			OCT605		; Load alarm code 605
		GOTO			; Exit with alarm
			V2T1X		; Excess iterations alarm
GAMDV30		STORE	NN2		; Update counter: NN2 = NN2 + 1
;
; ITERATION STEP: Adjust X(T1) and recompute DV
;
; Save current values, increment X(T1) by step size DX(T1), and recompute
; the required velocity change to see if we're converging to the solution.
;
		DLOAD	PDDL		; Save previous X(T1) (PL20D)
			X(T1)		; Current flight path angle
			DV		; Save previous DV (PL22D)
		PDDL	DAD		; Previous DV stored
			X(T1)		; Reload X(T1)
			16D		; Add step size DX(T1)
# Page 926
		STCALL	X(T1)		; Store updated X(T1) = X(T1) + DX(T1)
			DVCALC		; Recompute DV for new X(T1)
;
; MODE BRANCH: Check if fuel-critical or time-critical mode
;
		BON	DLOAD		; Branch on F2RTE flag
			F2RTE		; If time-critical mode
			GAMDV35		; Process time-critical logic
			DV		; Load new DV (fuel-critical)
		DSU	BMN		; DV - DV_previous
			20D		; Previous DV from pushlist
			GAMDV33		; If DV decreased, continue
;
; FUEL CRITICAL MODE: DV increased - reduce step size and try again
;
GAMDV32		DLOAD	DCOMP		; Reverse direction
			16D		; Load DX(T1)
		SR1			; Halve the step size
		STORE	16D		; Store reduced DX(T1)
GAMDV33		SETPD	GOTO		; Reset pushlist pointer
			18D		; PL18D
			GAMDV50		; Continue iteration
;
; ============================================================================
; TIME CRITICAL MODE
;
; In time-critical mode (F2RTE=1), we seek the X(T1) that maximizes DV
; (minimum flight time). This mode iterates to find the trajectory that gets
; Apollo 11 home fastest while staying within safe entry corridor limits.
; ============================================================================
;
GAMDV35		DLOAD	DSU		; Compute DV error
			RTEDVD		; Desired DV (target)
			DV		; Current computed DV
		PDDL	PUSH		; DVERR = DVD - DV (PL22D, PL24D)
;
; CONVERGENCE CHECK: Compare change in DV to tolerance
;
GAMDV40		DLOAD	ABS		; |DV_previous| from pushlist PL20D
			20D		; Previous DV
		DSU	BMN		; |DV_prev| - epsilon
			EPC10RTE	; Convergence tolerance
			GAMDVX		; Converged - exit with solution
;
; COMPUTE STEP SIZE: Use Newton-Raphson derivative approximation
;
GAMDV45		BOVB	DLOAD		; Clear overflow flag
			TCDANZIG	; Ensure OVFIND is 0
		BDSU	NORM		; DV - DV_previous
			DV		; Current DV
			X2		; Store shift count
		PDDL			; Delta DV stored (PL22D)
		NORM	SR1		; Normalize DVERR
			X1		; Store shift count for DVERR
		DDV	PDDL		; DVERR / (DV - DV_prev)
		BDSU	DMP		; (X(T1)_prev - X(T1)) * ratio
			X(T1)		; Current X(T1)
		XSU,1			; Denormalize
			X2		; Using X2 shift count
		STORE	16D		; Preserve sign if overflow
		SR*	BOV		; Right shift with X1
			0 -1,1		; Shift amount from X1
			GAMDV47		; Handle overflow case
		STORE	16D		; New step: DX(T1) = (X-X_prev)*DVERR/(DV-DV_prev)
		ABS	DSU		; Check if step within max
			12D		; Maximum step size
		BMN			; If step < max
			GAMDV50		; Accept step, continue
# Page 927
;
; LIMIT STEP SIZE: Cap to maximum allowed step
;
GAMDV47		DLOAD	SIGN		; Load max step
			12D		; Maximum DX(T1)
			16D		; With sign from computed step
		STORE	16D		; DX(T1) = DX(T1)_MAX * sign(DX_computed)
;
; ============================================================================
; CHECK TO KEEP INDEPENDENT VARIABLE IN BOUNDS
;
; Ensures next iteration's X(T1) stays within valid physical limits computed
; by XT1LIM subroutine. This prevents trajectory solutions that would violate
; entry corridor constraints or spacecraft limitations.
; ============================================================================
;
GAMDV50		DLOAD	DMP		; Compute lookahead value
			16D		; Current step DX(T1)
			1.1RTEB1	; 1.1 multiplier
		SL1	DAD		; Scale and add
			X(T1)		; Current X(T1)
		STORE	24D		; BETA9 = X(T1) + 1.1*DX(T1)
		DSU	BMN		; Compare to upper bound
			14D		; X(T1) upper limit
			GAMDV55		; Within bounds, continue
		DLOAD	DSU		; Exceeded upper bound
			14D		; Upper limit
			X(T1)		; Current X(T1)
		SR1			; Halve the difference
		STCALL	16D		; DX(T1) = (X(T1)_MAX - X(T1)) / 2
			GAMDV65		; Check minimum bound
;
; CHECK LOWER BOUND: Ensure we don't go below X(T1) minimum
;
GAMDV55		DLOAD	DSU		; Load lookahead value
			24D		; BETA9 = X(T1) + 1.1*DX(T1)
			10D		; X(T1) lower limit
		BMN	GOTO		; If below minimum
			GAMDV60		; Reduce step to stay above min
			GAMDV65		; Within bounds, check convergence
;
GAMDV60		DLOAD	DSU		; Exceeded lower bound
			10D		; Lower limit
			X(T1)		; Current X(T1)
		SR1			; Halve the difference
		STORE	16D		; DX(T1) = (X(T1)_MIN - X(T1)) / 2
;
; FINAL CONVERGENCE CHECK: Exit if step size negligible
;
GAMDV65		DLOAD	ABS		; Check if step size tiny
			16D		; |DX(T1)|
		DSU	BMN		; Compare to minimum step tolerance
			EPC9RTE		; Convergence epsilon
			GAMDVX		; Converged - exit iterator
		GOTO			; Not converged yet
			GAMDV25		; Continue next iteration
;
; ITERATOR EXIT: Return to caller with converged solution
;
GAMDVX		GOTO			; Exit iterator
			31D		; Return via pushlist location

# Page 928
# DV CALCULATION SUBROUTINE
#
# INPUT
#    PUSHLIST
#	02D		THETA1=BETA5*LAMBDA-1					TP	B17
#	05D		THETA2=2*R(T1)*(LAMBDA-1)				TP	B38/B36
#	08D		THETA3=MU**.5/R(T1)					DP	B-4/B-5
#    OTHER
#	X(T1)		COTANGENT OF POST IMPULSE INITIAL FLIGHT PATH ANGLE	DP	B5
#	V(T1)/		INITIAL VELOCITY VECTOR (PRE IMPULSE)			VECTOR	B7/B5	METERS/CS
#	UR1/		UNIT INITIAL VECTOR					VECTOR	B1
#	UH/		UNIT HORIZONTAL VECTOR					VECTOR	B1
#
# OUTPUT
#	V2(T1)/		POST IMPULSE INITIAL VELOCITY VECTOR			VECTOR	B7/B5	METERS/CS
#	DV		INITIAL VELOCITY CHANGE					DP	B7/B5	METERS/CS
#	PCON		SEMI-LATUS RECTUM					DP	B28/B26	METERS
#
# DEBRIS
#	28D		THETA3*PCON**.5						DP	B10/B8-N1
#	C(PUSHLOC)	THETA3(PCON**.5)*X(T1)*UR1/				VECTOR	B7/B5
#	32D		DVCALC SUBROUTINE RETURN ADDRESS
#	X1		NORMALIZATION FACTOR FOR VALUE IN 28D
#
# PUSHLOC IS RESTORED TO ITS ENTRANCE VALUE UPON EXITING DVCALC

; ============================================================================
; DVCALC: VELOCITY CHANGE CALCULATION
;
; This subroutine computes the required velocity change (DV) to achieve a 
; specified trajectory. For Apollo 11's TEI burn on July 21, 1969, this 
; calculation determined the exact velocity change needed to send Columbia 
; from lunar orbit back to Earth along the optimal return path.
;
; The calculation uses orbital mechanics to compute:
; 1. PCON (semi-latus rectum) - defines the conic section shape
; 2. V2(T1)/ - post-impulse velocity vector after the burn
; 3. DV - magnitude of velocity change required
;
; This is called repeatedly by GAMDV10 iterator to find the optimal burn.
; ============================================================================

DVCALC		STQ	DLOAD		; Save return address, load X(T1)
			32D		; Return address stored in 32D
			X(T1)		; X(T1) = cot(gamma_post_impulse)
;
; COMPUTE SEMI-LATUS RECTUM (PCON)
;
; The semi-latus rectum p defines the shape of the conic section (orbit).
; For Apollo 11's TEI trajectory, this determines whether the return path
; is hyperbolic (escape) or elliptical (Earth return).
;
; Formula: PCON = THETA2 / (THETA1 - X(T1)^2)
; where THETA1 = BETA5*LAMBDA - 1, THETA2 = 2*R(T1)*(LAMBDA - 1)
;
		DSQ	SR		; X(T1)^2, shift right
			7		; Scale adjustment
		DCOMP	TAD		; Complement and add
			02D		; Add THETA1 from pushlist 02D
		NORM	PUSH		; Normalize and push to stack
			X1		; Store normalization factor in X1
		TLOAD	NORM		; Load triple precision value
			05D		; THETA2 from pushlist 05D
			X2		; Normalization factor in X2
		RTB	SR1		; Return to basic mode, shift right 1
			DPMODE		; Double precision mode
		XSU,2	DDV		; Subtract X2 from X1, then divide
			X1		; Use normalization from X1
		SR*			; Shift right by variable amount
			6,2		; Shift count from X2
		STORE	PCON		# THETA2/(THETA1-X(T1)**2)=PCON	B28/26
;
; COMPUTE POST-IMPULSE VELOCITY V2(T1)/ 
;
; This section computes the velocity vector after the TEI burn. The velocity
; is composed of two components:
; 1. Radial component: THETA3 * sqrt(PCON) * X(T1) * UR1/ (direction of R)
; 2. Horizontal component: THETA3 * sqrt(PCON) * UH/ (perpendicular to R)
;
; where THETA3 = sqrt(MU) / R(T1)
;
		SQRT	DMP		; sqrt(PCON) * THETA3
			08D		; THETA3 = sqrt(MU) / R(T1)
		NORM			; Normalize result
			X1		; Store norm factor in X1
		STODL	28D		# THETA3*PCON**.5		B10/B8 -N1
# Page 929
			X(T1)		; Load X(T1) again
		NORM	VXSC		; Normalize and vector scale
			X2		; Normalization factor
			UR1/		# X(T1)*UR1/			B5+B1 -N2
		XAD,2	VXSC		; Exchange and add, then vector scale
			X1		; Use X1 normalization
			28D		; Multiply by THETA3*sqrt(PCON)
		VSR*	PDVL		# THETA3(PCON**.5)X(T1)*UR1/	B7/B5
			0 -9D,2		# Shift for proper scaling (radial part)
			UH/		; Load horizontal unit vector
		VXSC	VSR*		# THETA3(PCON**.5)UH/		B7/B5
			28D		; Multiply by THETA3*sqrt(PCON)
			0 -4,1		# Shift for proper scaling (horiz part)
;
; V2(T1)/ = (radial component) + (horizontal component)
; This is the velocity immediately after the TEI burn.
;
		VAD	STADR		; Vector add and store address
		STORE	V2(T1)/		# V2(T1)/			B7/B5
;
; COMPUTE DELTA-V MAGNITUDE
;
; DV = |V2(T1)/ - V(T1)/| = magnitude of velocity change required
; For Apollo 11, this was approximately 3,100 ft/s for the TEI burn.
;
		VSU	ABVAL		; Vector subtract, absolute value
			V(T1)/		; Pre-impulse velocity
		STORE	DV		# ABVAL(V2(T1)/-V1(T)/)=DV	B7/B5
		GOTO			; Return to caller
			32D		; Return address from entry

# Page 930
# SUBROUTINE TO COMPUTE BOUNDS ON INDEPENDENT VARIABLE X(T1)
#
# INPUT
#    PUSHLIST
#	PUSHLOC -4	MAJOR AXIS (MA)						DP	B30/B28
#	PUSHLOC -2	MAJOR AXIS (MA) AGAIN					DP	B30/B28
#	28D		BETA5=LAMBDA*BETA1					DP	B9
#    OTHER
#	RCON									DP	B29/B27
#	R(T1)									DP	B29/B27
#
# OUTPUT
#    MPAC
#	X(T1)LIM	LIMIT ON INDEPENDENT VARIABLE X(T1)			DP	B5
#
# DEBRIS
#    PUSHLIST
#	C(PUSHLOC)	MA-RCON							DP	(B30/28)-N1
#	C(PUSHLOC) +2	MA							DP	B30/B28
#	X1		NORMALIZATION FACTOR FOR MA-RCON
#	20D		XT1LIM SUBROUTINE RETURN ADDRESS
#
# PUSHLOC IS RESTORED TO ITS ENTRANCE VALUE UPON EXITING XT1LIM
;
; ============================================================================
; SUBROUTINE: XT1LIM - Compute Bounds on X(T1)
;
; This subroutine computes limits on the independent variable X(T1) to ensure
; the trajectory solution remains physically realizable. For the Apollo 11
; TEI burn, this prevents selection of impossible trajectories that would
; miss the atmospheric entry corridor.
;
; The limit computation ensures:
; 1. The trajectory intersects the entry interface altitude (400,000 ft)
; 2. The post-burn trajectory is within acceptable bounds
; 3. The flight path angle at entry is achievable
;
; Formula: X(T1)LIM = BETA5 * sqrt[(MA - RCON) / (MA - R(T1))]
; where MA is the major axis of the return orbit
; ============================================================================

XT1LIM		STQ	DLOAD		; Save return, load RCON
			20D		; Return address
			RCON		; Load radius constant
		SR1	BDSU		; Shift right 1, subtract from MA
		NORM	PDDL		# MA-RCON			B30-N1
			X2		; Normalization factor
		PDDL	SR1		; Push and load R(T1)
			R(T1)		; Current radius at time T1
		BDSU	DDV		; Subtract from MA, divide
		SL*	DMP		; Shift left, multiply
			0	-3,2	; Scale adjustment
			28D		; BETA5 from 28D
;
; COMPUTE BETA10 = BETA5 * (MA - R(T1)) / (MA - RCON) - 1
; This intermediate value determines the bounds on X(T1).
;
		SL*	DSU		# BETA10=BETA5(MA-RT)/(MA-RC)-1	B11
			0	-6,1	; Scale shift
			1RTEB25 +1	# 1.0				B-11
		SL1	BOV		; Shift left 1, branch on overflow
			XT1LIM2		; Handle overflow case
;
; CHECK BETA10 SIGN AND SELECT APPROPRIATE ACTION
; If BETA10 is negative, X(T1)LIM = 0 (no valid solution)
; If BETA10 is positive, X(T1)LIM = sqrt(BETA10)
;
		BMN	GOTO		; Branch if minus (negative)
			XT1LIM5		; Go to zero case
			XT1LIM3		; Continue to sqrt
;
; OVERFLOW HANDLING: If BETA10 overflows, set to maximum positive value
;
XT1LIM2		DLOAD			# BETA10=POSMAX IF OVERFLOW
			2RTEB1		; Load maximum positive value
;
; NORMAL CASE: X(T1)LIM = sqrt(BETA10)
;
XT1LIM3		SQRT	GOTO		# X(T1)=SQRT(BETA10)
			XT1LIMX		; Exit subroutine
;
; NEGATIVE BETA10: Set X(T1)LIM = 0 (physically impossible trajectory)
;
XT1LIM5		DLOAD			; Load zero
			ZERORTE		; Zero constant
XT1LIMX		GOTO			; Return to caller
			20D		; Return address

# Page 931
# CONSTANTS FOR THE P37 AND P70 PROGRAMS AND SUBROUTINES
;
; ============================================================================
; CONSTANTS SECTION: P37/P70 Return to Earth Programs
;
; This section defines all constants used in the Return to Earth (RTE)
; targeting calculations. These constants support:
; - Conic and precision trajectory computations
; - Entry corridor targeting (400,000 ft altitude)
; - Delta-V calculations for TEI burn
; - Propulsion system parameters (SPS and RCS)
; - Iteration convergence tolerances
; ============================================================================

		BANK	36
		SETLOC	RTECON1
		BANK
;
; SCALING CONSTANTS - UNITY VALUES AT VARIOUS BINARY SCALES
;
; These constants provide 1.0 at different binary scaling factors for
; fixed-point arithmetic. The AGC lacks floating-point hardware, so all
; calculations use scaled integer arithmetic.
;
1RTEB1		2DEC	1. B-1		; 1.0 scaled by 2^-1 = 0.5
1RTEB2		2DEC	1. B-2		; 1.0 scaled by 2^-2 = 0.25
1RTEB3		2DEC	1. B-3		; 1.0 scaled by 2^-3
1RTEB4		2DEC	1. B-4		; 1.0 scaled by 2^-4
1RTEB10		2DEC	1. B-10		; 1.0 scaled by 2^-10
1RTEB12		2DEC	1. B-12		; 1.0 scaled by 2^-12
1RTEB13		2DEC	1. B-13		; 1.0 scaled by 2^-13
1RTEB17		2DEC	1. B-17		; 1.0 scaled by 2^-17
1RTEB25		2DEC	1. B-25		; 1.0 scaled by 2^-25
#					* * B25 AND B28 MUST BE CONSECUTIVE * *
1RTEB28		2DEC	1. B-28		; 1.0 scaled by 2^-28
;
; BASIC CONSTANTS
;
ZERORTE		2DEC	0		; Zero constant for initialization
M144RTE		2DEC	-144. B-28	; -144 (used in trajectory calcs)
M15RTE		2DEC	-15		; -15 (iteration limit check)
10RTE		2DEC	10		; 10 (iteration counter)
M.6RTE		2DEC	-.6		; -0.6 (damping factor)
1.1RTEB1	2DEC	1.1 B-1		; 1.1 scaled by 2^-1 = 0.55
M6RTEB28	2DEC	-6		; -6 (computation constant)
2RTEB1		2OCT	3777737777	; Maximum positive value (POSMAX)
M9RTEB28	2DEC	-9		; -9 (computation constant)
M8RTEB28	2DEC	-8		; -8 (computation constant)
;
; EARTH REENTRY ALTITUDE: 400,000 feet = 121,920 meters
;
; This is the atmospheric entry interface altitude where reentry parameters
; (velocity, flight path angle, latitude/longitude) are predicted and displayed.
; For Apollo 11's return on July 24, 1969, this altitude defined the entry
; corridor targeting point.
;
30480RTE	2DEC	30480. B-29	; 400,000 ft in meters scaled B-29
;
; PROPULSION SYSTEM EXHAUST VELOCITY CONSTANTS
;
; These constants define the effective exhaust velocities for the Service
; Propulsion System (SPS) and Reaction Control System (RCS), critical for
; delta-V and mass calculations during TEI burn planning.
;
VCSPS		2DEC	31.510396 B-5	# (SEE 2VEXHUST)
					; SPS exhaust velocity: ~3,151 m/s
					; Used for SPS delta-V calculations
# Page 932
VCRCS		2DEC	27.0664 B-5	; RCS exhaust velocity: ~2,707 m/s
					; Used for RCS delta-V calculations
MDOTRCS		2DEC	.0016375 B-3	; RCS mass flow rate (kg/cs)
					; Used for RCS burn duration
;
; COMPUTATION CONTROL CONSTANTS
;
CSUBT		2DEC	.5		; Subtraction constant (0.5)
					; Used in trajectory iteration
;
; DISPLAY AND FLAGWORD BIT PATTERNS
;
OCT605		OCT	00605		; Octal constant for display control
OCT612		OCT	00612		; Octal constant for flag settings
;
; TRIGONOMETRIC CONSTANTS FOR ENTRY ANGLE COMPUTATIONS
;
; These support entry corridor angle constraints and atmospheric entry
; targeting calculations. Entry angles are critical for Apollo 11's
; safe return - too steep causes excessive G-forces, too shallow causes skip-out.
;
MCOS7.5		2DEC	-.99144486	; -cos(7.5°) for entry angle limits
MSIN7.5		2DEC	-.13052619	; -sin(7.5°) for entry angle limits
MCOS22.5	2DEC	-.92387953 B-2	; -cos(22.5°) for steering limits
;
; ANGLE CONSTANTS IN REVOLUTIONS
;
THETA165	2DEC	.4583333333	; 165° = 0.458333... revolutions
					; Entry corridor angle reference
THETA210	2DEC	.5833333333	; 210° = 0.583333... revolutions
					; Entry corridor angle reference
;
; PRECISION TRAJECTORY ITERATION TOLERANCES (EPC = EPSILON CONSTANTS)
;
; These control convergence criteria for the precision trajectory integration.
; Tighter tolerances ensure accurate Earth return targeting.
;
EPC1RTE		2DEC	.99966 B-1	; Position tolerance: ~0.99966
EPC2RTE		2DEC	100. B-29	; Distance tolerance: 100 meters
EPC3RTE		2DEC	.001		; Velocity tolerance: 0.1%
EPC4RTE		2DEC	.00001		; Time tolerance: 10 microseconds
EPC5RTE		2DEC	.01 B-6		; Angular tolerance
EPC6RTE		2DEC	.000007 B-1	; Flight path angle tolerance
EPC7RTE		2DEC	1000. B-29	; Altitude tolerance: 1 km
EPC9RTE		2DEC	1. B-25		; Normalized tolerance
EPC10RTE	2DEC	.0001 B-7	; Delta-V tolerance

		BANK	35
		SETLOC	RTECON1
		BANK
;
; EARTH GRAVITATIONAL AND ATMOSPHERIC CONSTANTS
;
; C4RTE: Fourth-order gravity harmonic coefficient for oblate Earth effects
; K1RTE, K2RTE: Atmospheric density model parameters
; K3RTE, K4RTE: Entry corridor constraint coefficients
;
C4RTE		2DEC	-6.986643 E7 B-30	; J4 harmonic coefficient
						; Earth oblateness correction
K1RTE		2DEC	7. E6 B-29		; Atmospheric model param
						; Density scale height
K2RTE		2DEC	6495000. B-29		; Earth radius + atmosphere
						; Reference altitude (meters)
K3RTE		2DEC	-.06105			; Entry corridor coefficient
						; Shallow entry limit
K4RTE		2DEC	-.10453			; Entry corridor coefficient
						; Steep entry limit
;
; EARTH GRAVITATIONAL PARAMETER FOR RETURN TRAJECTORY
;
RTMURTE		2DEC	199650.501 B-18		; sqrt(MU_Earth) scaled
						; Used in time-of-flight
						; calculations for TEI
# Page 933
;
; ENTRY INTERFACE ALTITUDE (DUPLICATE DEFINITION)
;
E3RTE		2DEC	121920. B-29		; 400,000 ft = 121,920 meters
						; Entry interface altitude
						; Matches 30480RTE definition

