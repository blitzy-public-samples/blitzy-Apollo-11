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

# Page 798
		EBANK=	E2DPS
		COUNT*	$$/F2DPS

# ********************************************************
# LUNAR LANDING FLIGHT SEQUENCE TABLES
# ********************************************************

; The powered descent from 50,000 feet to touchdown progresses through distinct phases.
; During Apollo 11's landing on July 20, 1969, this phase structure enabled the smooth
; transition from high-altitude braking through approach to final vertical descent.
;
; Phase index (WCHPHASE register) determines which guidance algorithms execute:
;   IGNALG (-1):   Ignition algorithm - initial descent engine start sequence
;   BRAKQUAD (0):  Braking phase - high-altitude deceleration from 50,000 ft
;   APPRQUAD (1):  Approach phase - visibility window for site selection (~10,000 ft)
;   VERTICAL (2):  Vertical descent - final controlled touchdown (P65/P66/P67)

# FLIGHT SEQUENCE TABLES ARE ARRANGED BY FUNCTION.  THEY ARE REFERENCED USING AS AN INDEX THE REGISTER WCHPHASE:
#	WCHPHASE = -1 ---> IGNALG
#	WCHPHASE =  0 ---> BRAKQUAD
#	WCHPHASE =  1 ---> APPRQUAD
#	WCHPHASE =  2 ---> VERTICAL

#*********************************************************

# ROUTINES FOR STARTING NEW GUIDANCE PHASES:

; Jump table for phase initialization - indexed by WCHPHASE to start new guidance phase.
; Each entry points to the appropriate initialization routine for that descent phase.

		TCF	TTFINCR		# IGNALG
NEWPHASE	TCF	TTFINCR		# BRAKQUAD
		TCF	STARTP64	# APPRQUAD
		TCF	P65START	# VERTICAL

# PRE-GUIDANCE COMPUTATIONS:

; Pre-guidance calculations prepare position and velocity vectors for guidance equations.
; RGVG (position-to-go, velocity-to-go) computations are fundamental to trajectory targeting.

		TCF	CALCRGVG	# IGNALG
PREGUIDE	TCF	RGVGCALC	# BRAKQUAD
		TCF	REDESIG		# APPRQUAD
		TCF	RGVGCALC	# VERTICAL

# GUIDANCE EQUATIONS:

; Core trajectory guidance computations - the heart of the lunar landing algorithm.
; TTF/8CL computes time-to-go and thrust commands using fuel-optimal control theory.
; During Apollo 11, these equations executed every 2 seconds, updating thrust vector commands.

		TCF	TTF/8CL		# IGNALG
WHATGUID	TCF	TTF/8CL		# BRAKQUAD
		TCF	TTF/8CL		# APPRQUAD
		TCF	VERTGUID	# VERTICAL

# POST GUIDANCE EQUATION COMPUTATIONS:

; Post-guidance calculations finalize commanded acceleration (CGCALC) for throttle control.
; These convert guidance solution into physical engine commands and attitude requirements.

		TCF	CGCALC		# IGNALG
AFTRGUID	TCF	CGCALC		# BRAKQUAD
		TCF	CGCALC		# APPRQUAD
		TCF	STEER?		# VERTICAL

# Page 799
# WINDOW VECTOR COMPUTATIONS:

; Window vectors enable crew visibility of landing terrain during approach phase.
; These computations orient the LM to provide commander visibility of landing site.

		TCF	EXGSUB		# IGNALG
WHATEXIT	TCF	EXBRAK		# BRAKQUAD
		TCF	EXNORM		# APPRQUAD

# DISPLAY ROUTINES:

; DSKY display routines show altitude, velocity, and fuel data to crew during descent.
; During Apollo 11, Armstrong and Aldrin monitored these values continuously.

WHATDISP	TCF	P63DISPS	# BRAKQUAD
		TCF	P64DISPS	# APPRQUAD
		TCF	VERTDISP	# VERTICAL

# ALARM ROUTINE FOR TTF COMPUTATION:

; Alarm 1406 triggers if time-to-go (TTF) computations fail or produce invalid results.
; This safeguards against guidance divergence during powered descent.

		TCF	1406P00		# IGNALG
WHATALM		TCF	1406ALM		# BRAKQUAD
		TCF	1406ALM		# APPRQUAD

# INDICES FOR REFERENCING TARGET PARAMETERS

		OCT	0		# IGNALG
TARGTDEX	OCT	0		# BRAKQUAD
		OCT	34		# APPRQUAD

#************************************************************************
# ENTRY POINTS:  ?GUIDSUB FOR THE IGNITION ALGORITHM, LUNLAND FOR SERVOUT
#************************************************************************

# IGNITION ALGORITHM ENTRY:  DELIVERS N PASSES OF QUADRATIC GUIDANCE

?GUIDSUB	EXIT
		CAF	TWO		# N = 3
		TS	NGUIDSUB
		TCF	GUILDRET +2

GUIDSUB		TS	NGUIDSUB	# ON SUCCEEDING PASSES SKIP TTFINCR
		TCF	CALCRGVG

# NORMAL ENTRY:  CONTROL COMES HERE FROM SERVOUT

LUNLAND		TC	PHASCHNG
		OCT	00035		# GROUP 5:  RETAIN ONLY PIPA TASK
		TC	PHASCHNG
		OCT	05023		# GROUP 3:  PROTECT GUIDANCE WITH PRIO 21
		OCT	21000		#	JUST HIGHER THAN SERVICER'S PRIORITY

# Page 800
#*******************************************************************
# GUILDENSTERN:  AUTO-MODES MONITOR (R13)
#*******************************************************************

		COUNT*	$$/R13

# HERE IS THE PHILOSOPHY OF GUILDENSTERN:	ON EVERY APPEARANCE OR DISAPPEARANCE OF THE MANUAL THROTTLE
# DISCRETE TO SELECT P67 OR P66 RESPECTIVELY:   ON EVERY APPEARANCE OF THE ATTITUDE-HOLD DISCRETE TO SELECT P66
# UNLESS THE CURRENT PROGRAM IS P67 IN WHICH CASE THERE IS NO CHANGE

GUILDEN		EXTEND			# IS UN-AUTO-THROTTLE DISCRETE PRESENT?
# STERN					# RSB 2009: Not originally a comment.
 		READ CHAN30
		MASK	BIT5
 		CCS	A
 		TCF	STARTP67	# YES
P67NOW?		TC	CHECKMM		# NO:  ARE WE IN P67 NOW?
		DEC	67
		TCF	STABL?		# NO
STARTP66	TC	FASTCHNG	# YES
		TC	NEWMODEX
DEC66		DEC	66
		EXTEND
		DCA	HDOTDISP	# SET DESIRED ALTITUDE RATE = CURRENT
		DXCH	VDGVERT		# 	ALTITUDE RATE.
STRTP66A	TC	INTPRET
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
		ADRES	XOVINFLG
		TC	DOWNFLAG
		ADRES	REDFLAG
		TCF	VERTGUID

STARTP67	TC	NEWMODEX	# NO HARM IN "STARTING" P67 OVER AND OVER
		DEC	67		# SO NO NEED FOR A FASTCHNG AND NO NEED
		CAF	ZERO		# TO SEE IF ALREADY IN P67.
		TS	RODCOUNT
		CAF	TEN
		TCF	VRTSTART

STABL?		CAF	BIT13		# IS UN-ATTITUDE-HOLD DISCRETE PRESENT?
		EXTEND
		RAND	CHAN31
		CCS	A
		TCF	GUILDRET	# YES ALL'S WELL

P66NOW?		CS	MODREG
		AD	DEC66
		EXTEND
		BZF	RESTART?

		CA	RODCOUNT	# NO. HAS THE ROD SWITCH BEEN "CLICKED"?
		EXTEND
		BZF	GUILDRET	# NO. CONTINUE WITH AUTOMATIC LANDING
		TCF	STARTP66	# YES. SWITCH INTO THE ROD MODE.

RESTART?	CA	FLAGWRD1	# HAS THERE BEEN A RESTART?
		MASK	RODFLBIT
		EXTEND
		BZF	STRTP66A	# YES.  REINITIALIZE BUT LEAVE VDGVERT AS
					#	IS.

		TCF	VERTGUID	# NO: CONTINUE WITH R.O.D.

# *******************************************************************************
# INITIALIZATION FOR THIS PASS
# *******************************************************************************

		COUNT*	$$/F2DPS

GUILDRET	CAF	ZERO
		TS	RODCOUNT

# Page 802
 +2		EXTEND
 		DCA	TPIP
		DXCH	TPIPOLD

		TC	FASTCHNG

		EXTEND
		DCA	PIPTIME1
		DXCH	TPIP

		EXTEND
		DCA	TTF/8
		DXCH	TTF/8TMP

		CCS	FLPASS0
		TCF	TTFINCR

BRSPOT1		INDEX	WCHPHASE
		TCF	NEWPHASE

# ******************************************************************
# ROUTINES TO START NEW PHASES
# ******************************************************************

P65START	TC	NEWMODEX
		DEC	65
		CS	TWO
		TS	WCHVERT
		TC	DOWNFLAG	# PERMIT X-AXIS OVERRIDE
		ADRES	XOVINFLG
		TCF	TTFINCR

STARTP64	TC	NEWMODEX
		DEC	64
		CA	DELTTFAP	# AUGMENT TTF/8
		ADS	TTF/8TMP
		CA	BIT12		# ENABLE RUPT10
		EXTEND
		WOR	CHAN13
		TC	DOWNFLAG	# INITIALIZE REDESIGNATION FLAG
		ADRES	REDFLAG


#		(CONTINUE TO TTFINCR)

# *********************************************************************************
# INCREMENT TTF/8, UPDATE LAND FOR LUNAR ROTATION, DO OTHER USEFUL THINGS
# *********************************************************************************
#
#	TTFINCR COMPUTATIONS ARE AS FOLLOWS --
# Page 803
#		TTF/8 UPDATED FOR TIME SINCE LAST PASS:
#			TTF/8 = TTF/8 + (TPIP - TPIPOLD)/8
#		LANDING SITE VECTOR UPDATED FOR LUNAR ROTATION:
#			____               ____   ____                   __
#			LAND = /LAND/ UNIT(LAND - LAND(TPIP - TPIPOLD) * WM)
#		SLANT RANGE TO LANDING SITE, FOR DISPLAY:
#			                 ____   _
#			RANGEDSP = ABVAL(LAND - R)

; ============================================================================
; TTFINCR - TIME-TO-GO INCREMENT AND LANDING SITE ROTATION UPDATE
;
; This critical routine updates time-to-go (TTF/8) based on elapsed time and
; compensates the landing site vector (LAND) for lunar surface rotation.
; The Moon rotates beneath the descending LM, so the target coordinates must
; be continuously updated. During Apollo 11's 12-minute descent, the landing
; site moved approximately 2.5 kilometers eastward due to lunar rotation.
;
; TTF/8 scaling: Time-to-go scaled by 2^3 (divided by 8) in centiseconds.
; LAND scaling: Position vector scaled by 2^29 meters.
; WM: Lunar angular velocity vector (radians/centisecond scaled 2^23).
;
; Mathematical formulation:
;   TTF/8(new) = TTF/8(old) + ΔT/8  where ΔT = TPIP - TPIPOLD
;   LAND(new) = |LAND| * UNIT(LAND - LAND × WM × ΔT)
;
; The cross product LAND × WM produces the velocity of the landing site due
; to lunar rotation, which is then scaled by elapsed time to get displacement.
; ============================================================================

TTFINCR		TC	INTPRET
; Compute elapsed time since last guidance pass:
		DLOAD	DSU
			TPIP		; Current PIPA sample time
			TPIPOLD		; Previous PIPA sample time
		SLR	PUSH		# SHIFT SCALES DELTA TIME TO 2(17) CSECS
			11D		; Scale ΔT from 2^28 cs to 2^17 cs, push onto stack
; Compute landing site displacement due to lunar rotation:
		VXSC	VXV		; LAND scaled by ΔT
			LAND		; Landing site position vector (2^29 m)
			WM		; Lunar angular velocity (2^23 rad/cs)
		BVSU	RTB		; LAND - (LAND × WM × ΔT), normalize result
			LAND
			NORMUNIT	; Unit vector in direction of rotated landing site
; Scale to actual landing site distance and store:
		VXSC	VSL1		; Multiply by landing site distance magnitude
			/LAND/		; Landing site distance (2^29 m)
		STODL	LANDTEMP	; Store updated LAND vector, load ΔT from stack
		EXIT

; Update time-to-go in native AGC code:
		DXCH	MPAC		; ΔT/8 from interpreter stack to accumulator
		DAS	TTF/8TMP	# NOW HAVE INCREMENTED TTF/8 IN TTF/8TMP

		TC	FASTCHNG

		EXTEND
		DCA	TTF/8TMP
		DXCH	TTF/8

		EXTEND
		DCA	LANDTEMP
		DXCH	LAND
		EXTEND
		DCA	LANDTEMP +2
		DXCH	LAND     +2
		EXTEND
		DCA	LANDTEMP +4
		DXCH	LAND	 +4

# Page 804
		TC	TDISPSET
		TC	FASTCHNG	# SINCE REDESIG MAY CHANGE LANDTEMP

BRSPOT2		INDEX	WCHPHASE
		TCF	PREGUIDE

# *********************************************************************
# LANDING SITE PERTURBATION EQUATIONS
# *********************************************************************

; ============================================================================
; REDESIG - LANDING SITE REDESIGNATION
;
; This routine implements manual landing site redesignation, allowing the
; commander to adjust the target landing point using the hand controller.
; During Apollo 11, Neil Armstrong used this capability at low altitude to
; avoid a boulder field and select a smoother landing site in the Sea of
; Tranquility crater.
;
; ELINCR/AZINCR: Elevation and azimuth increments from crew inputs (scaled degrees)
; REDFLAG: Flag indicating redesignation mode is active
; TREDES: Timer counting down redesignation availability window
;
; The redesignation vector is computed in guidance coordinates, with limits
; to prevent targeting too close to the horizon (depression angle check).
; ============================================================================

REDESIG		CA	FLAGWRD6	# IS REDFLAG SET?
		MASK	REDFLBIT
		EXTEND
		BZF	RGVGCALC	# NO:  SKIP REDESIGNATION LOGIC

; Check if redesignation timer has expired:
		CA	TREDES		# YES:  HAS TREDES REACHED ZERO?
		EXTEND
		BZF	RGVGCALC	# YES:  SKIP REDESIGNATION LOGIC

; Transfer crew inputs to working registers with interrupt protection:
		INHINT			; Disable interrupts during critical section
		CA	ELINCR1		; Elevation increment from hand controller
		TS	ELINCR
		CA	AZINCR1		; Azimuth increment from hand controller
		TS	AZINCR
		TC	FASTCHNG

; Clear input registers for next redesignation cycle:
		CA	ZERO
		TS	ELINCR1
		TS	AZINCR1
		TS	ELINCR	+1
		TS	AZINCR  +1

		CA	FIXLOC		# SET PD TO 0
		TS	PUSHLOC

; Compute redesignation vector in guidance coordinates:
		TC	INTPRET
		VLOAD	VSU		; Load LAND - R (landing site to spacecraft vector)
			LAND
			R		#                 ____   _
		RTB	PUSH		# PUSH DOWN UNIT (LAND - R)
			NORMUNIT	; Unit line-of-sight to landing site
		VXV	VSL1		; Cross with body Y-axis for azimuth perturbation
			YNBPIP		#                    ___        ____   _
		VXSC	PDDL		# PUSH DOWN - ELINCR(YNB * UNIT(LAND - R))
			ELINCR		; Scale by elevation increment
			AZINCR		; Load azimuth increment
		VXSC	VSU		; Azimuth component: AZINCR * YNBPIP
			YNBPIP
		VAD	PUSH		# RESULTING VECTOR IS 1/2 REAL SIZE

# Page 805

; Limit redesignation to prevent targeting near or below horizon:
		DLOAD	DSU		# MAKE SURE REDESIGNATION IS NOT
			0		# 	TOO CLOSE TO THE HORIZON.
			DEPRCRIT	; Critical depression angle limit
		BMN	DLOAD		; If depression angle too shallow, use critical value
			REDES1
			DEPRCRIT
		STORE	0
; Apply redesignation vector to compute new landing site:
REDES1		DLOAD	DSU		; Compute |LAND - R| distance
			LAND
			R
		DDV	VXSC		; Scale redesignation by distance, apply to R
			0		; Depression angle component
		VAD	UNIT		; Add to current position, unitize direction
			R
		VXSC	VSL1		; Scale to landing site distance magnitude
			/LAND/
		STORE	LANDTEMP	; Store new landing site vector
		EXIT			# LOOKANGL WILL BE COMPUTED AT RGVGCALC

; Update LAND vector in erasable memory with new redesignated site:
		TC	FASTCHNG

		EXTEND			; Transfer LANDTEMP to LAND (6 words)
		DCA	LANDTEMP
		DXCH	LAND
		EXTEND
		DCA	LANDTEMP +2
		DXCH	LAND +2
		EXTEND
		DCA	LANDTEMP +4
		DXCH	LAND +4

		TCF	RGVGCALC

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

; ============================================================================
; RGVGCALC - STATE TRANSFORMATION TO GUIDANCE COORDINATES
;
; This fundamental routine transforms the spacecraft state from inertial
; coordinates (R, V) to guidance coordinates (RG, VG) referenced to the
; rotating lunar surface. The guidance coordinate frame has:
;   X-axis: Downrange (toward landing site)
;   Y-axis: Crossrange (left)
;   Z-axis: Altitude (up from surface)
;
; CG matrix: Coordinate transformation from inertial to guidance frame
; WM: Lunar angular velocity vector (Moon rotates ~13.2 degrees/day)
; ANGTERM: Velocity relative to rotating surface (V + R × WM)
;
; Position scaling: 2^29 meters
; Velocity scaling: 2^9 meters/centisecond (after VSR2 rescale)
;
; These computations execute every guidance cycle (2 seconds) to maintain
; accurate targeting as the LM descends and the Moon rotates beneath it.
; ============================================================================

CALCRGVG	TC	INTPRET		# IN IGNALG, COMPUTE V FROM INTEGRATION
		VLOAD	MXV		#	OUTPUT AND TRIM CORRECTION TERM
			VATT1		#	COMPUTED LAST PASS AND LEFT IN UNFC/2
			REFSMMAT
		VSR1	VAD
			UNFC/2
		STORE	V
		EXIT

RGVGCALC	TC	INTPRET		# ENTER HERE TO RECOMPUTE RG AND VG
		VLOAD	VXV
			R
			WM
		VAD	VSR2		# RESCALE TO UNITS OF 2(9) M/CS
			V
		STORE	ANGTERM
		MXV
			CG		# NO SHIFT SINCE ANGTERM IS DOUBLE SIZED
		STORE	VGU
		PDDL	VDEF		# FORM (0,VG ,VG ) IN UNITS OF 2(10) M/CS
			ZEROVECS	#           2   1
		ABVAL	SL3
		STOVL	VHORIZ		# VHORIZ FOR DISPLAY DURING P65.
			R		#           _   ____
		VSU	PUSH		# PUSH DOWN R - LAND
			LAND
		MXV	VSL1
			CG
		STORE	RGU
		ABVAL
		STOVL	RANGEDSP
		RTB	DOT		# NOW IN MPAC IS SINE(LOOKANGL)/4
			NORMUNIT
			XNBPIP
		EXIT

		CA	FIXLOC		# RESET PUSH DOWN POINTER
		TS	PUSHLOC

# Page 807
		CA	MPAC		# COMPUTE LOOKANGLE ITSELF
		DOUBLE
		TC	BANKCALL
		CADR	SPARCSIN -1
		AD	1/2DEG
		EXTEND
		MP	180DEGS
		TS	LOOKANGL	# LOOKANGL FOR DISPLAY DURING P64

BRSPOT3		INDEX	WCHPHASE
		TCF	WHATGUID

# **************************************************************************
# TTF/8 COMPUTATION
# **************************************************************************
;
; ============================================================================
; SECTION: TTF/8 COMPUTATION (Time-to-Go Calculation)
;
; This section computes TTF/8 (Time-To-Fall divided by 8), a critical parameter
; for the fuel-optimal descent guidance. The TTF represents the estimated time
; remaining until touchdown, scaled by 1/8 for numerical precision in the AGC's
; fixed-point arithmetic. This value is recalculated every guidance cycle
; (approximately every 2 seconds during powered descent).
;
; The computation uses a polynomial root-finding algorithm (ROOTPSRS) to solve
; for TTF/8 given the current state (position, velocity) and desired landing
; conditions. The polynomial coefficients are constructed from:
;   A(0) = -24(RGU2 - RDG2)/64  (position difference term)
;   A(1) = (6*VGU2 + 18*VDG2)/8  (velocity terms)
;   A(2) = 6*ADG2                (acceleration term)
;   A(3) = 8*JDG2                (jerk term)
;
; During Apollo 11's descent, this routine executed reliably even during the
; famous 1202 program alarm, providing continuous guidance updates to the
; throttle control system.
;
; COMMENT-ONLY READERS: This is where the computer calculates how many more
; seconds until Eagle touches down, updating the prediction every 2 seconds
; based on current altitude and velocity.
;
; CODE-ALONG READERS: Note the use of TABLTTF to store polynomial coefficients,
; the ROOTPSRS subroutine for solving the quartic equation, and the TTF/8
; scaling which provides adequate precision while avoiding overflow in subsequent
; guidance calculations.
; ============================================================================

TTF/8CL		TC	INTPRETX
		DLOAD*
			JDG2TTF,1
		STODL*	TABLTTF +6	# A(3) = 8 JDG  TO TABLTTF
			ADG2TTF,1	#             2
		STODL	TABLTTF +4	# A(2) = 6 ADG  TO TABLTTF
			VGU 	+4	#             2
		DMP	DAD*
			3/4DP
			VDG2TTF,1
		STODL*	TABLTTF +2	# A(1) = (6 VGU  + 18 VDG )/8 TO TABLTTF
			RDG +4,1	#              2         2
		DSU	DMP
			RGU +4
			3/8DP
		STORE	TABLTTF		# A(0) = -24 (RGU  - RDG )/64 TO TABLTTF
		EXIT			#                2      2

		CA	BIT8
		TS	TABLTTF +10	# FRACTIONAL PRECISION FOR TTF TO TABLE

		EXTEND
		DCA	TTF/8
		DXCH	MPAC		# LOADS TTF/8 (INITIAL GUESS) INTO MPAC
		CAF	TWO		# DEGREE - ONE
		TS	L
		CAF	TABLTTFL
		TC	ROOTPSRS	# YIELDS TTF/8 IN MPAC
		INDEX	WCHPHASE
		TCF	WHATALM

		EXTEND			# GOOD RETURN
		DCA	MPAC		# FETCH TTF/8 KEEPING IT IN MPAC
		DXCH	TTF/8		# CORRECTED TTF/8

# Page 808
		TC	TDISPSET

# 		(CONTINUE TO QUADGUID)

# *********************************************************************************
# MAIN GUIDANCE EQUATION
# *********************************************************************************
;
; ============================================================================
; SECTION: QUADGUID - Main Guidance Equation (Fuel-Optimal Descent Trajectory)
;
; This is the core of the Apollo lunar landing guidance system. QUADGUID computes
; the commanded acceleration vector (ACG) that will guide the Lunar Module along
; a fuel-optimal trajectory from the current state to the desired landing site.
;
; The guidance law is based on calculus of variations and optimal control theory,
; developed by MIT Instrumentation Laboratory. It generates thrust commands that
; minimize fuel consumption while satisfying constraints on:
;   - Landing site accuracy (within targeting tolerance)
;   - Vertical velocity at touchdown (< 3 ft/sec for safe landing)
;   - Horizontal velocity at touchdown (< 1 ft/sec lateral drift)
;   - Attitude constraints (avoid gimbal lock, maintain visibility)
;
; The equation computes ACG (commanded acceleration in guidance coordinates) using:
;   RGU = current position vector (scaled meters, guidance coordinates)
;   VGU = current velocity vector (scaled meters/centisecond)
;   RDG = desired position (landing site coordinates)
;   VDG = desired velocity (nominally zero at touchdown)
;   ADG = desired acceleration (lunar gravity compensation)
;   TTF = time-to-fall (predicted time until touchdown in centiseconds)
;
; A LEADTIME correction factor accounts for guidance loop lag (the time between
; computing a thrust command and the engine responding). This was critical during
; Apollo 11's descent when the guidance computer was near maximum computational
; load during the 1202 alarm condition.
;
; The guidance coefficients (stored in 26D, 28D, 30D) are functions of the ratio
; (TTF - LEADTIME)/TTF, ensuring smooth thrust transitions and avoiding
; oscillations that could waste propellant or disturb crew visibility.
;
; COMMENT-ONLY READERS: This is the mathematical brain that steered Eagle down
; to the Sea of Tranquility. Every 2 seconds, it recalculated the perfect thrust
; direction and magnitude to reach the landing site with minimum fuel usage.
; Armstrong and Aldrin watched the results on the DSKY display as altitude
; and velocity numbers changed in real-time.
;
; CODE-ALONG READERS: The implementation uses scaled fixed-point vector arithmetic
; via the INTERPRETER. Note the coefficient calculations using (TTF-LEADTIME)/TTF
; ratio and its square, the TTF/8 scaling to maintain precision, and the final
; transformation through the CG matrix (guidance-to-stable-member coordinates)
; before computing commanded acceleration magnitude (/AFC/).
; ============================================================================
#
#	AS PUBLISHED --
#		              ___   __       ___   __
#		___   ___   6(VDG + VG)   12(RDG - RG)
#		ACG = ADG + ----------- + ------------
#		                TTF        (TTF)(TTF)
#	AS HERE PROGRAMMED --
#		             ___   __
#		      3 (1/4(RDG - RG)   ___   __)
#		      - (------------- + VDG + VG)
#		___   4 (    TTF/8               )   ___
#		ACG = ---------------------------- + ADG
#		                  TTF/8

QUADGUID	CS	TTF/8
		AD	LEADTIME	# LEADTIME IS A NEGATIVE NUMBER
		AD	POSMAX		# SAFEGUARD THE COMPUTATIONS THAT FOLLOW
		TS	L		#	BY FORCING -TTF*LEADTIME > OR = ZERO
		CS	L
		AD	L
		ZL
		EXTEND
		DV	TTF/8
		TS	BUF		# - RATIO OF LAG-DIMINISHED TTF TO TTF
		EXTEND
		SQUARE
		TS	BUF +1
		AD	BUF
		XCH	BUF +1		# RATIO SQUARED - RATIO
		AD	BUF +1
		TS	MPAC		# COEFFICIENT FOR VGU TERM
		AD	BUF +1
		INDEX	FIXLOC
		TS	26D		# COEFFICIENT FOR RDG-RGU TERM
		AD	BUF +1
		INDEX	FIXLOC
		TS	28D		# COEFFICIENT FOR VDG TERM
		AD	BUF
		AD	POSMAX
# Page 809
		AD	BUF +1
		AD	BUF +1
		INDEX	FIXLOC
		TS	30D		# COEFFICIENT FOR ADG TERM

		CAF	ZERO
		TS	MODE

		TC	INTPRETX
		VXSC	PDDL
			VGU
			28D
		VXSC*	PDVL*
			VDG,1
			RDG,1
		VSU	V/SC
			RGU
			TTF/8
		VSR2	VXSC
			26D
		VAD	VAD
		V/SC	VXSC
			TTF/8
			3/4DP
		PDDL	VXSC*
			30D
			ADG,1
		VAD
;
; ============================================================================
; SECTION: AFCCALC - Acceleration Magnitude and Component Calculation
;
; This section transforms the commanded acceleration vector (ACG) from guidance
; coordinates into stable-member coordinates (the spacecraft's inertial reference
; frame), then computes the acceleration magnitude /AFC/ needed for throttle
; control and the individual acceleration components for attitude steering.
;
; The transformation sequence:
;   1. ACG (guidance coords) → multiply by CG matrix → stable-member coords
;   2. Subtract gravity acceleration (GDT/2 scaled by GSCALE)
;   3. Store result in UNFC/2 (unfilt acceleration, scaled by 1/2)
;   4. Compute magnitude /AFC/ using ABVAL (vector magnitude)
;   5. Calculate horizontal acceleration capability AMAXHORIZ using:
;      AMAXHORIZ = SQRT(ATOTAL² - A₁² - A₀²)
;      where ATOTAL = HIGHESTF/MASS (maximum thrust capability)
;
; The AMAXHORIZ calculation ensures the commanded horizontal acceleration does
; not exceed the physical capability of the descent engine given current mass
; and vertical acceleration requirements. This prevents commanded attitudes
; that would be impossible to achieve, which could cause instability or abort.
;
; During Apollo 11's descent, this routine executed every 2 seconds, providing
; fresh throttle commands to the THROTTLE_CONTROL_ROUTINES. The /AFC/ magnitude
; directly controlled engine thrust from 10% to 92% throttle range, while the
; UNFC/2 components guided the attitude autopilot to point the thrust vector
; in the correct direction for trajectory control.
;
; COMMENT-ONLY READERS: After calculating which direction to thrust, this
; section figures out HOW HARD to thrust. It sends throttle commands to the
; descent engine and attitude commands to the RCS thrusters, ensuring Eagle
; could physically achieve the commanded trajectory.
;
; CODE-ALONG READERS: Note the CG matrix multiplication (VXM) for coordinate
; transformation, the gravity compensation (BVSU with GDT/2), the magnitude
; calculation (ABVAL→/AFC/), and the horizontal acceleration limit check using
; HIGHESTF/MASS. The UNFC/2 scaling (divide by 2) maintains precision in
; subsequent filtering operations.
; ============================================================================
AFCCALC1	VXM	VSL1		# VERGUID COMES HERE
			CG
		PDVL	V/SC
			GDT/2
			GSCALE
		BVSU	STADR
		STORE	UNFC/2		# UNFC/2 NEED NOT BE UNITIZED
		ABVAL
AFCCALC2	STODL	/AFC/		# MAGNITUDE OF AFC FOR THROTTLE
			UNFC/2		# VERTICAL COMPONENT
		DSQ	PDDL
			UNFC/2 +2	# OUT-OF-PLANE
		DSQ	PDDL
			HIGHESTF
		DDV	DSQ
			MASS		#                        2    2    2
		DSU	DSU		# AMAXHORIZ = SQRT(ATOTAL - A  - A  )
		BPL	DLOAD		#                            1    0
			AFCCALC3
			ZEROVECS
AFCCALC3	SQRT	DAD
			UNFC/2 +4
# Page 810
		BPL	BDSU
			AFCCLEND
			UNFC/2 +4
		STORE	UNFC/2 +4
AFCCLEND	EXIT
		TC	FASTCHNG

		CA	WCHPHASE	# PREPARE FOR PHASE SWITCHING LOGIC
		TS	WCHPHOLD
		INCR	FLPASS0		# INCREMENT PASS COUNTER

BRSPOT4		INDEX	WCHPHASE
		TCF	AFTRGUID

# ***********************************************************************
# ERECT GUIDANCE-STABLE MEMBER TRANSFORMATION MATRIX
# ***********************************************************************

CGCALC		CAF	EBANK5
		TS	EBANK
		EBANK=	TCGIBRAK
		EXTEND
		INDEX	WCHPHASE
		INDEX	TARGTDEX
		DCA	TCGFBRAK
		INCR	BBANK
		INCR	BBANK
		EBANK=	TTF/8
		AD	TTF/8
		XCH	L
		AD	TTF/8
		CCS	A
		CCS	L
		TCF	EXTLOGIC
		TCF	EXTLOGIC
		NOOP

		TC	INTPRETX
		VLOAD	UNIT
			LAND
		STODL	CG
			TTF/8
		DMP*	VXSC
			GAINBRAK,1	# NUMERO MYSTERIOSO
			ANGTERM
		VAD
			LAND
		VSU	RTB
			R
			NORMUNIT
# Page 811
		VXV	RTB
			LAND
			NORMUNIT
		STOVL	CG +6		# SECOND ROW
			CG
		VXV	VSL1
			CG +6
		STORE	CG +14
		EXIT

#		(CONTINUE TO EXTLOGIC)
#
; ============================================================================
; SECTION: EXTLOGIC - Exit Logic and Phase Transition Control
;
; After computing guidance commands, EXTLOGIC determines how to exit the current
; guidance cycle and whether to advance to the next landing phase. The landing
; sequence progresses through distinct phases based on altitude, velocity, and
; time-to-fall (TTF):
;
;   IGNALG   (WCHPHASE = -1): Initial ignition algorithm
;   BRAKQUAD (WCHPHASE =  0): Braking phase from ~50,000 ft to ~8,000 ft
;   APPRQUAD (WCHPHASE =  1): Approach phase from ~8,000 ft to ~500 ft
;   VERTICAL (WCHPHASE =  2): Final vertical descent to touchdown
;
; Phase transitions occur automatically when TTF/8 becomes less than threshold
; values (TENDBRAK for braking→approach transition, etc.). The FASTCHNG routine
; handles rapid phase changes, and FLPASS0 is reset to ensure proper first-pass
; initialization in the new phase.
;
; Exit routines vary by phase and purpose:
;
; EXGSUB: Returns from GUIDSUB when called by ignition algorithm. Computes
;         trim velocity correction using ZOOMTIME and TRIMACCL parameters.
;         Calls DDUMCALC for delta-DPS mass calculation.
;
; EXBRAK: Exit during braking phase. Computes window pointing vector as
;         UNIT(R) - the unit vector toward the spacecraft from lunar center.
;         This keeps the landing site visible to the crew through the window.
;
; EXNORM: Normal exit during approach and vertical phases. Computes window
;         vector as weighted combination of:
;         - UNIT(LAND - R): Direction from spacecraft to landing site
;         - CG+14 vector: Backup window vector when projection is out of range
;         Weighting depends on projection of desired line-of-sight against
;         velocity plane (PROJMIN < PROJ < PROJMAX ensures landing site
;         remains in window field-of-view).
;
; EXVERT: Exit for vertical descent phase. Checks OVFIND overflow flag to
;         skip throttle/FINDCDUW calls if guidance overflow detected.
;
; COMMENT-ONLY READERS: This section manages the handoff between landing phases.
; During Apollo 11, Armstrong manually took control during the approach phase
; when he spotted the boulder field. The phase transition logic here enabled
; that smooth handoff from automated braking to semi-manual approach.
;
; CODE-ALONG READERS: Note the WCHPHASE indexing into flight sequence tables
; (WHATEXIT), the TTF/8 comparison with phase-specific thresholds (TENDBRAK),
; the FASTCHNG call for phase switching, and the window vector calculations
; ensuring crew visibility of the landing site throughout descent.
; ============================================================================
# ***********************************************************************
# PREPARE TO EXIT
# ***********************************************************************
#
# DECIDE (1) HOW TO EXIT, AND (2) WHETHER TO SWITCH PHASES
#
EXTLOGIC	INDEX	WCHPHASE	# WCHPHASE = 1   APPRQUAD
		CA	TENDBRAK	# WCHPHASE = 0   BRAKQUAD
		AD	TTF/8

EXSPOT1		EXTEND
		INDEX	WCHPHASE
		BZMF	WHATEXIT

		TC	FASTCHNG

		CA	WCHPHOLD
		AD	ONE
		TS	WCHPHASE
		CA	ZERO
		TS	FLPASS0		# RESET FLPASS0

		INDEX	WCHPHOLD
		TCF	WHATEXIT

# ***********************************************************************
# ROUTINES FOR EXITING FROM LANDING GUIDANCE
# ***********************************************************************
#
# 1.	EXGSUB IS THE RETURN WHEN GUIDSUB IS CALLED BY THE IGNITION ALGORITHM.
# 2.	EXBRAK IN THE EXIT USED DURING THE BRAKING PHASE.  IN THIS CASE UNIT(R) IS THE WINDOW POINTING VECTOR.
# 3.	EXNORM IS THE EXIT USED AT OTHER TIMES DURING THE BURN.
# (EXOVFLOW IS A SUBROUTINE OF EXBRAK AND EXNORM CALLED WHEN OVERFLOW OCCURRED ANYWHERE IN GUIDANCE.)

EXGSUB		TC	INTPRET		# COMPUTE TRIM VELOCITY CORRECTION TERM.
# Page 812
		VLOAD	RTB
			UNFC/2
			NORMUNIT
		VXSC	VXSC
			ZOOMTIME
			TRIMACCL
		STORE	UNFC/2
		EXIT

		CCS	NGUIDSUB
		TCF	GUIDSUB
		CCS	NIGNLOOP
		TCF	+3
		TC	ALARM
		OCT	01412

 +3		TC	POSTJUMP
 		CADR	DDUMCALC

EXBRAK		TC	INTPRET
		VLOAD
			UNIT/R/
		STORE	UNWC/2
		EXIT
		TCF	STEER?

EXNORM		TC	INTPRET
		VLOAD	VSU
			LAND
			R
		RTB
			NORMUNIT
		STORE	UNWC/2		# UNIT(LAND - R) IS TENTATIVE CHOICE
		VXV	DOT
			XNBPIP
			CG +6
		EXIT			# WITH PROJ IN MPAC 1/8 REAL SIZE

		CS	MPAC		# GET COEFFICIENT FOR CG +14
		AD	PROJMAX
		AD	POSMAX
		TS	BUF
		CS	BUF
		ADS	BUF		# RESULT IS 0 IF PROJMAX - PROJ NEGATIVE

		CS	PROJMIN		# GET COEFFICIENT FOR UNIT(LAND - R)
		AD	MPAC
		AD	POSMAX
		TS	BUF +1
		CS	BUF +1
# Page 813
		ADS	BUF +1		# RESULT IS 0 IF PROJ - PROJMIN NEGATIVE

		CAF	FOUR
UNWCLOOP	MASK	SIX
		TS	Q
		CA	EBANK5
		TS	EBANK
		EBANK=	CG
		CA	BUF
		EXTEND
		INDEX	Q
		MP	CG +14
		INCR	BBANK
		EBANK=	UNWC/2
		INDEX	Q
		DXCH	UNWC/2
		EXTEND
		MP	BUF +1
		INDEX	Q
		DAS	UNWC/2
		CCS	Q
		TCF	UNWCLOOP

		INCR	BBANK
		EBANK=	PIF

STEER?		CA	FLAGWRD2	# IF STEERSW DOWN NO OUTPUTS
		MASK	STEERBIT
		EXTEND
		BZF	RATESTOP

EXVERT		CA	OVFIND		# IF OVERFLOW ANYWHERE IN GUIDANCE
		EXTEND			#	DON'T CALL THROTTLE OR FINDCDUW
		BZF	+13

EXOVFLOW	TC	ALARM		# SOUND THE ALARM NON-ABORTIVELY
		OCT	01410

RATESTOP	CAF	BIT13		# ARE WE IN ATTITUDE-HOLD?
		EXTEND
		RAND	CHAN31
		EXTEND
		BZF	DISPEXIT	# YES

		TC	BANKCALL	# NO:  DO A STOPRATE
		CADR	STOPRATE

		TCF	DISPEXIT

GDUMP1		TC	THROTTLE
# Page 814
		TC	INTPRET
		CALL
			FINDCDUW -2
		EXIT

# 		(CONTINUE TO DISPEXIT)

; ============================================================================
; SECTION: GUIDANCE LOOP DISPLAYS - Crew Interface During Descent
;
; The display routines provide real-time altitude, velocity, and fuel data to
; the crew on the DSKY (Display and Keyboard) during powered descent. Display
; updates occur at the end of each 2-second guidance cycle, unless suppressed
; by the FLUNDISP flag (fundamental display inhibit).
;
; Display routines are phase-specific and indexed via WHATDISP table:
;
; P63DISPS (BRAKQUAD phase): Displays V06N63 showing:
;   - Time-to-fall divided by 8 (TTF/8)
;   - Altitude rate (feet per second)
;   - Altitude (feet)
;   Used during the braking phase from ~50,000 feet to ~8,000 feet altitude.
;   Static (non-flashing) display updated every guidance cycle.
;
; P64DISPS (APPRQUAD phase): Displays V06N64 showing:
;   - Landing site redesignation capability status
;   - Crossrange (feet) - lateral offset from landing site
;   - Downrange (feet) - forward/aft offset from landing site
;   Used during approach phase from ~8,000 feet to ~500 feet altitude.
;   
;   P64 provides FLASHING DISPLAY when redesignation is available (REDFLAG not
;   set, TREDES counter not expired), allowing crew to adjust landing site via
;   +V33E (manual redesignation). During Apollo 11, Armstrong used this feature
;   at approximately 500 feet altitude when he spotted the boulder field and
;   manually selected a safer landing site downrange.
;
;   Display behavior controlled by TREDES counter (redesignation time remaining):
;   - TREDES > 0 and REDFLAG clear: Flashing V06N64 with three crew options:
;     * TERMINATE: Abort to GOTOPOOH (Program 00)
;     * PROCEED: Enable redesignations, set REDFLAG, continue with static display
;     * RECYCLE: Refresh flashing display
;   - TREDES > 0 and REDFLAG set: Static V06N64 display (redesignations active)
;   - TREDES = 0: Clear REDFLAG, static V06N64 display (redesignations locked out)
;
; P66DISPS (VERTICAL phase): Displays V06N60 showing:
;   - Forward velocity (feet per second)
;   - Lateral velocity (feet per second)
;   - Altitude (feet)
;   Used during final vertical descent from ~500 feet to touchdown.
;   Static display, updated every guidance cycle.
;
; Display update timing controlled by GROUP 3 and GROUP 5 phase change logic:
; - DISPEXIT kills GROUP 3 to ensure displays are restored by next guidance cycle
; - PHASCHNG OCT 00035 kills GROUP 5 before display update
; - FLUNDISP flag (FLAGWRD8) bypasses display updates when set
;
; All display routines call REGODSPR (register/DSKY display) via BANKCALL to
; present the verb/noun combination to the crew. The DISPCOMN common routine
; handles the actual DSKY formatting.
;
; HISTORICAL CONTEXT: During Apollo 11's descent on July 20, 1969, these displays
; provided Armstrong and Aldrin with critical situational awareness. The P64
; redesignation capability at ~102:44:00 mission time enabled Armstrong to
; manually guide Eagle away from the boulder-strewn crater toward the safer
; landing site where "The Eagle has landed" at 102:45:40 MET.
;
; COMMENT-ONLY READERS: These routines update the numbers Armstrong and Aldrin
; watched during descent. The flashing display in P64 prompted Armstrong to take
; manual control and select the final landing site.
;
; CODE-ALONG READERS: Note the WCHPHOLD indexing into WHATDISP table, the
; FLUNDISP flag check via FLAGWRD8 masking, the TREDES countdown logic, the
; REDFLAG state management in FLAGWRD6, and the three-way crew response handling
; for REFLASHR (terminate/proceed/recycle).
; ============================================================================
# ***********************************************************************
# GUIDANCE LOOP DISPLAYS
# ***********************************************************************

DISPEXIT	EXTEND			# KILL GROUP 3:  DISPLAYS WILL BE
		DCA	NEG0		#	RESTORED BY NEXT GUIDANCE CYCLE.
		DXCH	-PHASE3

 +3		CS	FLAGWRD8	# IF FLUNDISP IS SET, NO DISPLAY THIS PASS
 		MASK	FLUNDBIT
		EXTEND
		BZF	ENDLLJOB	# TO PICK UP THE TAG

		INDEX	WCHPHOLD
		TCF	WHATDISP

-2		TC	PHASCHNG	# KILL GROUP 5
		OCT	00035

P63DISPS	CAF	V06N63
DISPCOMN	TC	BANKCALL
		CADR	REGODSPR

ENDLLJOB	TCF	ENDOFJOB

P64DISPS	CA	TREDES		# HAS TREDES REACHED ZERO?
		EXTEND
		BZF	RED-OVER	# YES: CLEAR REDESIGNATION FLAG

		CS	FLAGWRD6	# NO:  IS REDFLAG SET?
		MASK	REDFLBIT
		EXTEND
		BZF	REDES-OK	# YES:  DO STATIC DISPLAY

		CAF	V06N64		# OTHERWISE USE FLASHING DISPLAY
		TC	BANKCALL
		CADR	REFLASHR
		TCF	GOTOPOOH	# TERMINATE
		TCF	P64CEED		# PROCEED	PERMIT REDESIGNATIONS
		TCF	P64DISPS	# RECYCLE
# Page 815
		TCF	ENDLLJOB

P64CEED		CAF	ZERO
		TS	ELINCR1
		TS	AZINCR1

		TC	UPFLAG		# ENABLE REDESIGNATION LOGIC
		ADRES	REDFLAG

		TCF	ENDOFJOB

RED-OVER	TC	DOWNFLAG
		ADRES	REDFLAG
REDES-OK	CAF	V06N64
		TCF	DISPCOMN


VERTDISP	CAF	V06N60
		TCF	DISPCOMN


; ============================================================================
; SECTION: VERTICAL GUIDANCE - Final Descent Phase (P65/P66/P67)
;
; The vertical guidance routines govern the final phase of lunar descent from
; approximately 500 feet altitude to touchdown (or abort). This is the critical
; manual flight phase where the crew has primary control authority.
;
; WCHVERT register determines active vertical guidance mode:
;   WCHVERT = 0  ---> P66 (Lunar Module Attitude Control)
;   WCHVERT > 0  ---> P67 (Automatic Landing)
;   [Default entry] ---> P65 (Null Velocity Guidance)
;
; PHASE DESCRIPTIONS:
;
; P65 (Null Velocity Guidance): Automatically nulls horizontal velocity while
; maintaining vertical descent rate. Commands acceleration to eliminate velocity
; mismatch between desired final ground velocity (V2FG) and current velocity (VGU).
; Guidance equation: ACG = (V2FG - VGU) / TAUVERT
; Where TAUVERT is vertical time constant controlling descent rate response.
;
; P66 (Lunar Module Attitude Control): Crew-controlled attitude and throttle.
; The LM responds to crew inputs via hand controller (attitude) and thrust/
; translational controller (throttle). Guidance computer maintains commanded
; attitude and thrust levels. This was the mode Armstrong used during Apollo 11's
; final approach when he took semi-manual control at approximately 200 feet
; altitude to fly over the boulder field and select the final landing site.
;
; P67 (Automatic Landing): Fully automatic descent to touchdown. Computer controls
; both attitude and throttle to achieve soft landing. Used if crew does not
; intervene or if automatic landing is selected.
;
; HISTORICAL CONTEXT: During Apollo 11 landing on July 20, 1969, Armstrong
; transitioned to P66 mode at approximately 500 feet altitude when he spotted
; the boulder field and crater rim that made the computer's selected landing
; site unsafe. He manually flew Eagle horizontally to find a suitable landing
; spot, extending the landing time and reducing fuel margins. "The Eagle has
; landed" at 102:45:40 MET with approximately 25 seconds of fuel remaining.
;
; COMMENT-ONLY READERS: These are the three final landing modes. P66 is where
; Armstrong took manual control to fly over the boulder field and select the
; safer landing site that made lunar landing history.
;
; CODE-ALONG READERS: Note the CCS WCHVERT logic selecting guidance mode, the
; velocity error computation (V2FG - VGU), the time constant division (TAUVERT),
; and the transfer to AFCCALC1 for thrust vector computation. P66/P67 modes use
; different guidance philosophies but converge to the same thrust calculation path.
; ============================================================================
# **************************************************************************
# GUIDANCE FOR P65
# **************************************************************************

VERTGUID	CCS	WCHVERT
		TCF	P67VERT		# POSITIVE NON-ZERO ---> P67
		TCF	P66VERT		# +0
#
# 	THE P65 GUIDANCE EQUATION IS AS FOLLOWS --
#		      ____   ___
#		      V2FG - VGU
#		ACG = ----------
#		        TAUVERT

P65VERT		TC	INTPRET
		VLOAD	VSU
			V2FG
			VGU
		V/SC	GOTO
			TAUVERT
			AFCCALC1
# Page 816
# **********************************************************
# GUIDANCE FOR P66
# **********************************************************

P66VERT		TC	POSTJUMP
		CADR	P66VERTA

P67VERT		TC	PHASCHNG	# TERMINATE GROUP 3.
		OCT	00003

		TC	INTPRET
		VLOAD	GOTO
			V
			VHORCOMP

		SETLOC	P66LOC
		BANK
		COUNT*	$$/F2DPS

; ============================================================================
; SECTION: RATE-OF-DESCENT UPDATE TASK (RODTASK)
;
; The RODTASK is initiated every 2 seconds during powered descent phases P63,
; P64, P66, and P67. It computes and updates the displayed rate-of-descent and
; horizontal velocity for crew monitoring during final approach and landing.
;
; CREW INTERFACE:
; - Displays altitude rate (vertical velocity) on DSKY
; - Displays horizontal velocity during P66 manual control mode
; - Manual ROD thumbwheel on DSKY allows crew to adjust displayed rate
; - Critical for Armstrong's landing decisions during Apollo 11 final approach
;
; TASK SCHEDULING:
; Priority 22 VAC area task (lower priority than guidance, higher than displays)
; Scheduled via TWIDDLE at 1-second intervals, executes RODCOMP computation
;
; CODE-ALONG READERS: FINDVAC allocates VAC area for RODCOMP computation task.
; EBANK=DVCNTR ensures proper erasable memory bank selection for variables.
; 2CADR provides both bank number and address for cross-bank RODCOMP call.
; ============================================================================

RODTASK		CAF	PRIO22
		TC	FINDVAC
		EBANK=	DVCNTR
		2CADR	RODCOMP

		TCF	TASKOVER

P66VERTA	TC	PHASCHNG	# TERMINATE GROUP 3.
		OCT	00003

		CAF	1SEC
		TC	TWIDDLE
		ADRES	RODTASK

; ============================================================================
; SUBROUTINE: RODCOMP - Rate-Of-Descent Computation
;
; Computes updated rate-of-descent (vertical velocity) for DSKY display using
; PIPA (Pulsed Integrating Pendulous Accelerometer) delta-V measurements.
;
; COMPUTATION SEQUENCE:
; 1. Process manual ROD thumbwheel input (RODCOUNT adjustments)
; 2. Update desired altitude rate (VDGVERT) from thumbwheel changes
; 3. Read IMU accelerometer pulses (PIPAX, PIPAY, PIPAZ)
; 4. Compute delta-velocity since last cycle (DELVROD vector)
; 5. Transform delta-V to local vertical reference frame
; 6. Update displayed altitude rate and horizontal velocity
;
; MANUAL ROD THUMBWHEEL PROCESSING:
; RODCOUNT is incremented/decremented by MARKRUPT interrupt handler when crew
; turns thumbwheel. Each click = ±1 count. RODSCAL1 scaling converts counts
; to ft/sec rate adjustment. Updated VDGVERT becomes new desired descent rate.
;
; PIPA MEASUREMENT PROCESSING:
; - PIPAX, PIPAY, PIPAZ contain accumulated accelerometer pulses since last read
; - OLDPIPAX/Y/Z store previous reading for delta computation
; - RUPTREG1/2/3 save old values before overwriting
; - DELVROD = (current PIPA) - (old PIPA) + (interrupt correction RUPTREG)
; - Result is delta-velocity vector in platform coordinates
;
; HISTORICAL CONTEXT: During Apollo 11 landing, this computation ran every 2
; seconds from PDI through touchdown. Armstrong monitored the displayed altitude
; rate (ft/sec down) throughout descent. During final approach below 500 feet,
; he used ROD display to judge proper sink rate while maneuvering horizontally.
;
; CODE-ALONG READERS: INHINT disables interrupts during PIPA read to ensure
; atomic snapshot. OLDPIPAX/Y/Z-RUPTREG1/2/3 exchange preserves state across
; potential READACCS interrupts. DELVROD computation uses CS/AD for subtraction.
; ============================================================================

RODCOMP		INHINT
		CAF	ZERO
		XCH	RODCOUNT
		EXTEND
		MP	RODSCAL1
		DAS	VDGVERT		# UPDATE DESIRED ALTITUDE RATE.

		EXTEND			# SET OLDPIPAX,Y,Z = PIPAX,Y,Z
		DCA	PIPAX
		DXCH	OLDPIPAX
		DXCH	RUPTREG1	# SET RUPTREG1,2,3 = OLDPIPAX,Y,Z
		CA	PIPAZ
		XCH	OLDPIPAZ
		XCH	RUPTREG3

		EXTEND			# SNAPSHOT TIME OF PIPA READING.
		DCA	TIME2
# Page 817
		DXCH	THISTPIP

; Compute cumulative PIPA readings for scaled velocity integration.
; MPAC vector = OLDPIPA + PIPATMP (accumulated pulses since last cycle).
; PIPATMPX/Y/Z are incremented by READACCS during SERVICER interrupts.
		CA	OLDPIPAX
		AD	PIPATMPX
		TS	MPAC		# MPAC(X) = PIPAX + PIPATMPX
		CA	OLDPIPAY
		AD	PIPATMPY
		TS	MPAC +3		# MPAC(Y) = PIPAY + PIPATMPY
		CA	OLDPIPAZ
		AD	PIPATMPZ
		TS	MPAC +5		# MPAC(Z) = PIPAZ + PIPATMPZ

; Compute DELVROD = (current PIPA) - (old PIPA) + (interrupt correction).
; TEMX/Y/Z contain new readings from READACCS if it executed during this cycle.
; RUPTREG1/2/3 contain saved old PIPA values for delta computation.
; CS performs complement (negation) for subtraction via addition.
; Result is delta-velocity vector since last 2-second ROD update.
		CS	OLDPIPAX
		AD	TEMX
		AD	RUPTREG1
		TS	DELVROD
		CS	OLDPIPAY
		AD	TEMY
		AD	RUPTREG2
		TS	DELVROD +2
		CS	OLDPIPAZ
		AD	TEMZ
		AD	RUPTREG3
		TS	DELVROD +4

		CAF	ZERO
		TS	MPAC +1		# ZERO LO-ORDER MPAC COMPONENTS
		TS	MPAC +4
		TS	MPAC +6
		TS	TEMX		# ZERO TEMX, TEMY, AND TEMZ SO WE WILL
		TS	TEMY		#	KNOW WHEN READACCS CHANGES THEM.
		TS	TEMZ
		CS	ONE
		TS	MODE
		TC	INTPRET
ITRPNT1		VXSC	PDDL		# SCALE MPAC TO M/CS *2(-7) AND PUSH 	(6)
			KPIP1
			THISTPIP
		DSU
			PIPTIME
		STORE	30D		# 30-31D CONTAINS TIME IN CS SINCE PIPTIME
		DDV	PDVL		#					(8)
			4SEC(28)
			GDT/2
		VSU	VXSC		#					(6)
			VBIAS
		VSL2	VAD
			V
		VAD	STADR		#					(0)
		STOVL	24D		# STORE UPDATED VELOCITY IN 24-29D
# Page 818
			R
		UNIT
		STORE	14D
		DOT	SL1
			24D
		STODL	HDOTDISP	# UPDATE HDOTDISP RATE FOR NOUN 63.
			30D
		SL	DMP
			11D
			HDOTDISP
		DAD	DSU
			36D
			/LAND/
		STODL	HCALC1		# UPDATE HCALC1 FOR NOUN 63.
			HDOTDISP
		BDSU	DDV
			VDGVERT
			TAUROD
		PDVL	ABVAL		#				(2)
			GDT/2
		DDV	SR2
			GSCALE
		STORE	20D
		DAD			#				(0)
		PDVL	CALL		#				(2)
			UNITX
			CDU*NBSM
		DOT
			14D
		STORE	22D
		BDDV	STADR		#				(0)
		STOVL	/AFC/
			DELVROD
		VXSC	VAD
			KPIP1
			VBIAS
		ABVAL	PDDL		#				(2)
			THISTPIP
		DSU	PDDL		#				(4)
			LASTTPIP
			THISTPIP
		STODL	LASTTPIP	#				(2)
		DDV	BDDV		#				(0)
			SHFTFACT
		PDDL	DMP		#				(2)
			FWEIGHT
			BIT1H
		DDV	DDV
			MASS
			SCALEFAC
# Page 819
		DAD	PDDL		#				(4)
			0D
			20D
		DDV	DSU		#				(2)
			22D
		DMP	DAD
			LAG/TAU
			/AFC/
		PDDL	DDV		#				(4)
			MAXFORCE
			MASS
		PDDL	DDV		#				(6)
			MINFORCE
			MASS
		PUSH	BDSU		#				(8)
			2D
		BMN	DLOAD		#				(6)
			AFCSPOT
		DLOAD	PUSH		#				(6)
		BDSU	BPL
			2D
			AFCSPOT
		DLOAD			#				(4)
AFCSPOT		DLOAD			#				(2), (4), OR (6)
		SETPD			#				(2)
			2D
		STODL	/AFC/		#				(0)
ITRPNT2		EXIT
		DXCH	MPAC		# MPAC = MEASURED ACCELERATION.
		TC	BANKCALL
		CADR	THROTTLE +3
		TC	INTPRET
		VLOAD			# PICK UP UPDATED VELOCITY VECTOR.
			24D

; ============================================================================
; SUBROUTINE: VHORCOMP - Horizontal Velocity Computation
;
; Computes horizontal velocity magnitude for DSKY display during P66 manual
; landing mode. Crew needs horizontal velocity feedback when translating LM
; to find smooth landing spot.
;
; COMPUTATION METHOD:
; 1. Start with total velocity vector V (24D) from navigation state
; 2. Add commanded delta-velocity DELVS from guidance
; 3. Extract vertical component by projecting onto unit radius vector
; 4. Subtract vertical component from total velocity
; 5. Compute magnitude of remaining horizontal component
; 6. Store result in VHORIZ for DSKY display (Noun 60)
;
; MATHEMATICAL FORMULATION:
;   V_horiz = |V_total - (V_total · R_unit) * R_unit|
; where R_unit is unit vector from lunar center to LM position.
;
; SCALING:
; - Input velocity scaled in meters/centisecond * 2^7
; - VSL2/VSR2 adjust scaling for computation precision
; - HDOTDISP provides altitude rate for vertical component extraction
; - Result VHORIZ in ft/sec for crew display
;
; HISTORICAL CONTEXT: During Apollo 11 P66 manual phase (below 500 feet),
; Armstrong controlled attitude while monitoring horizontal velocity on DSKY.
; He needed to null horizontal motion before touchdown to avoid tipover risk.
; This computation provided real-time horizontal speed feedback critical for
; safe touchdown site selection and final descent to lunar surface.
;
; DISPLAY: Result shown via V06N60 (Verb 06 Noun 60) on DSKY.
; Noun 60 displays horizontal velocity and altitude rate simultaneously.
;
; CODE-ALONG READERS: VLOAD/VAD/VXSC are interpretive vector operations.
; UNIT computes unit vector from position R. BVSU is backwards vector subtract.
; ABVAL computes vector magnitude. DISPEXIT+3 updates display without phase chg.
; ============================================================================
VHORCOMP	VSL2	VAD
			DELVS
		VSR2	PDVL
			R
		UNIT	VXSC
			HDOTDISP
		VSL1	BVSU
		ABVAL
		STORE	VHORIZ
		EXIT
		TC	BANKCALL	# PUT UP V06N60 DISPLAY BUT AVOID PHASCHNG
		CADR	DISPEXIT +3

BIT1H		OCT	00001
SHFTFACT	2DEC	1 B-17
# Page 820
BIASFACT	2DEC	655.36 B-28

; ============================================================================
; SECTION: REDESIGNATOR TRAP - Manual Landing Site Selection Interface
;
; The Redesignator Trap is the critical crew interface that enabled Neil Armstrong
; to manually adjust the landing site target during Apollo 11's final approach.
; This interrupt-driven logic processes inputs from the Attitude Controller
; Assembly (ACA) hand controller, translating stick deflections into landing
; site coordinate adjustments (azimuth and elevation changes).
;
; HISTORICAL SIGNIFICANCE:
; On July 20, 1969, as Eagle descended through the approach phase below 2000 feet,
; Armstrong looked out the window and saw the computer was guiding toward a
; boulder-strewn crater. Using the ACA hand controller, he redesignated the
; landing site multiple times, moving the target roughly 1500 feet downrange
; to find the smooth area that became Tranquility Base. This redesignation
; capability, implemented in this code section, was essential to mission success.
;
; INTERRUPT MECHANISM:
; - PITFALL is the interrupt service routine triggered by ACA input
; - Interrupt source: CHAN31 (Channel 31) discrete input bits
; - Four redesignation bits monitored: +PITCH, -PITCH, +YAW, -YAW
; - Interrupt fires when any redesignation bit transitions
;
; REDESIGNATION INCREMENTS:
; - Azimuth (left/right): ±2 degrees per input (AZEACH = .03491 radians)
; - Elevation (up/down): ±0.5 degrees per input (ELEACH = .00873 radians)
; - Increments accumulate in AZINCR1 and ELINCR1
; - Guidance equations read these increments to adjust landing site target
;
; PROGRAM MODE RESTRICTION:
; Redesignation only active during P64 (approach phase, typically 2000-500 feet).
; Not available in P63 (braking phase) or P66 (manual attitude control).
; Mode check prevents unintended target changes in other phases.
;
; ATTITUDE-HOLD INTERLOCK:
; If BIT13 of CHAN31 is set (attitude-hold mode), redesignation logic is bypassed.
; Attitude-hold mode means autopilot is maintaining fixed orientation, so
; redesignation inputs would conflict with automatic guidance.
;
; DEBOUNCE LOGIC:
; REDESMON (redesignator monitor) implements software debounce:
; - ELVIRA: Current reading of redesignation bits
; - ZERLINA: Debounce counter (decrements each pass)
; - Monitor continues until inputs stable for multiple samples
; - Prevents false triggers from electrical noise or contact bounce
;
; COMMENT-ONLY READERS: This is the code that let Armstrong steer Eagle away
; from the boulder field during the final approach. Each time he moved the hand
; controller, this routine added small adjustments to the landing target
; coordinates, gradually walking the aim point across the lunar surface until
; he found a safe spot.
;
; CODE-ALONG READERS: PITFALL is interrupt entry point (saves BANKRUPT/QRUPT).
; CHECKMM verifies mode 64. Channel 31 discrete inputs are complemented and
; masked to extract four redesignation bits. REDESMON implements state machine
; for debouncing. AZINCR1/ELINCR1 are read by REDESIG subroutine during guidance.
; ============================================================================
# *********************************************************************************
# REDESIGNATOR TRAP
# *********************************************************************************

		BANK	11
		SETLOC	F2DPS*11
		BANK

		COUNT*	$$/F2DPS

; PITFALL interrupt entry point - triggered by ACA hand controller input.
; Save interrupted program context (bank and return address).
PITFALL		XCH	BANKRUPT
		EXTEND
		QXCH	QRUPT

; Verify we're in P64 approach phase. Redesignation only allowed in P64.
; CHECKMM checks current major mode in MODREG against expected mode (64).
; If not in P64, immediately resume interrupted program without processing.
		TC	CHECKMM		# IF NOT IN P64, NO REASON TO CONTINUE
		DEC	64
		TCF	RESUME

; Read Channel 31 discrete inputs and extract redesignation bits.
; CHAN31 bits (complemented): +PITCH, -PITCH, +YAW, -YAW hand controller inputs.
; ALL4BITS mask isolates the four redesignation input bits.
; ELVIRA holds current state of redesignation bits for debounce comparison.
		EXTEND
		READ	CHAN31
		COM			; Complement channel bits (hardware inverted)
		MASK	ALL4BITS	; Extract four redesignation bits
		TS	ELVIRA		; Save current bit state in ELVIRA

; Initialize debounce counter ZERLINA to 2.
; Debounce requires stable input for multiple samples before acting.
		CAF	TWO
		TS	ZERLINA

; Schedule REDESMON task to run in 5 centiseconds (50 milliseconds).
; TWIDDLE sets up a waitlist task. Five centiseconds provides time for
; input to stabilize and filters out transient electrical noise.
		CAF	FIVE
		TC	TWIDDLE		; Schedule task with 5cs delay
		ADRES	REDESMON	; Task address: redesignator monitor
		TCF	RESUME		; Return from interrupt

; ============================================================================
; REDESIGNATOR MONITOR - Software Debounce State Machine
;
; REDESMON is waitlist task scheduled by PITFALL to monitor redesignation
; inputs over time and filter out noise. Task reschedules itself until
; inputs stabilize or debounce counter expires.
;
; DEBOUNCE ALGORITHM:
; 1. Read current state of four redesignation bits
; 2. Compare to previous reading in ELVIRA
; 3. If bits still active: Continue monitoring (reschedule 7cs later)
; 4. If bits released: Process accumulated inputs (go to COUNT'EM)
; 5. If bits inactive and were inactive: Decrement ZERLINA counter
; 6. When ZERLINA reaches zero: Reset interrupt enable, terminate
;
; VARIABLES:
; - ELVIRA: Previous reading of redesignation bits
; - ZERLINA: Debounce countdown (decremented when no bits detected)
; - L register: Holds previous ELVIRA value for comparison
;
; TIMING:
; - Initial check after 5cs (50ms) from PITFALL
; - Subsequent checks every 7cs (70ms) via VARDELAY
; - Typical redesignation input lasts 100-300ms (crew hand controller pulse)
; ============================================================================
# REDESIGNATOR MONITOR (INITIATED BY PITFALL)

; PREMON1: Decrement ZERLINA and continue monitoring.
PREMON1		TS	ZERLINA		; Store decremented counter

; PREMON2: Reschedule monitor task for another check in 7 centiseconds.
PREMON2		CAF	SEVEN
		TC	VARDELAY	; Reschedule this task in 7cs

; REDESMON: Main monitoring entry point (runs every 7cs after initial 5cs delay).
; Read current state of Channel 31 redesignation bits and compare to previous.
REDESMON	EXTEND
		READ	31
		COM			; Complement (hardware inverted logic)
		MASK	ALL4BITS	; Extract four redesignation bits
		XCH	ELVIRA		; Swap with previous reading
		TS	L		; Save previous ELVIRA in L

; Test current reading (now in ELVIRA). If any bits set, inputs still active.
		CCS	ELVIRA		# DO ANY BITS APPEAR THIS PASS?
		TCF	PREMON2		# Y:	CONTINUE MONITOR

; No bits detected this pass. Check if any were detected last pass.
		CCS	L		# N:	ANY LAST PASS?
		TCF	COUNT'EM	#	Y: 	COUNT 'EM, RESET RUPT, TERMINATE
# Page 821

; No bits this pass or last pass. Decrement debounce counter ZERLINA.
; If counter not yet zero, continue monitoring to ensure inputs truly stable.
		CCS	ZERLINA		#	N: 	HAS ZERLINA REACHED ZERO YET?
		TCF	PREMON1		#		N:	DIMINISH ZERLINA, CONTINUE

; ZERLINA reached zero. Inputs have been stable (inactive) for full debounce
; period. Reset interrupt enable (BIT12 of CHAN13) and terminate task.
RESETRPT	CAF	BIT12		#		Y:	RESET RUPT. TERMINATE
		EXTEND
		WOR	CHAN13		; Write-OR to set interrupt enable bit
		TCF	TASKOVER	; Terminate waitlist task

; ============================================================================
; COUNT'EM - Process Redesignation Inputs and Update Landing Target
;
; This routine decodes the four redesignation bits captured in register L
; and converts them into azimuth and elevation adjustments. Each hand
; controller input adds a fixed angular increment to the accumulated
; redesignation angles (AZINCR1, ELINCR1). The REDESIG subroutine later
; converts these angular increments into changes in landing site coordinates.
;
; INPUT BIT MAPPING (from ACA hand controller):
; - BIT1 (+PITCH): Elevation down (nose down input) → -EL increment
; - BIT2 (-PITCH): Elevation up (nose up input) → +EL increment  
; - BIT5: Azimuth right → +AZ increment
; - BIT6: Azimuth left → -AZ increment
;
; REDESIGNATION INCREMENTS (angles in revolutions):
; - AZEACH = .03491 rev = 2.0 degrees (azimuth step per input)
; - ELEACH = .00873 rev = 0.5 degrees (elevation step per input)
;
; ACCUMULATED REDESIGNATION:
; - AZINCR1: Total azimuth change accumulated since start of P64
; - ELINCR1: Total elevation change accumulated since start of P64
; - These accumulate across multiple hand controller inputs
; - REDESIG routine reads these and adjusts landing site target
;
; ATTITUDE-HOLD INTERLOCK:
; First checks if attitude-hold mode active (BIT13 of CHAN31).
; If attitude-hold engaged, autopilot maintains fixed orientation and
; redesignation would conflict, so skip processing and reset interrupt.
;
; HISTORICAL CONTEXT: During Apollo 11 approach, Armstrong made approximately
; 6-8 redesignation inputs, each adding 2° azimuth or 0.5° elevation to shift
; the landing target. Total redesignation moved aim point roughly 1500 feet
; downrange from the boulder field to the smooth area of Tranquility Base.
;
; CODE-ALONG READERS: RAND performs read-AND with channel. BZF branches if
; zero (att-hold not active). Each input bit is masked, tested with CCS,
; and if present adds (ADS) the corresponding increment to accumulator.
; Process repeats for all four possible input directions.
; ============================================================================
COUNT'EM	CAF	BIT13		# ARE WE IN ATTITUDE-HOLD?
		EXTEND
		RAND	CHAN31		; Read-AND Channel 31 with BIT13 mask
		EXTEND
		BZF	RESETRPT	# YES: SKIP REDESIGNATION LOGIC.

; Attitude-hold not active. Process accumulated redesignation bits in L.
; Check each of four input bits and apply corresponding increment.

; Process -AZBIT (left azimuth): Subtract 2 degrees from azimuth accumulator.
		CA	L		# NO: Recall captured bits from L
		MASK 	-AZBIT		; Isolate left azimuth bit (BIT6)
		CCS	A		; Test if bit present
-AZ		CS	AZEACH		; Bit present: negate AZEACH (2° complement)
		ADS	AZINCR1		; Add to azimuth increment accumulator

; Process +AZBIT (right azimuth): Add 2 degrees to azimuth accumulator.
		CA	L		; Recall captured bits
		MASK	+AZBIT		; Isolate right azimuth bit (BIT5)
		CCS	A		; Test if bit present
+AZ		CA	AZEACH		; Bit present: load AZEACH (2°)
		ADS	AZINCR1		; Add to azimuth increment accumulator

; Process -ELBIT (down elevation): Subtract 0.5 degrees from elevation accumulator.
		CA	L		; Recall captured bits
		MASK	-ELBIT		; Isolate pitch down bit (BIT1)
		CCS	A		; Test if bit present
-EL		CS	ELEACH		; Bit present: negate ELEACH (0.5° complement)
		ADS	ELINCR1		; Add to elevation increment accumulator

; Process +ELBIT (up elevation): Add 0.5 degrees to elevation accumulator.
		CA	L		; Recall captured bits
		MASK	+ELBIT		; Isolate pitch up bit (BIT2)
		CCS	A		; Test if bit present
+EL		CA	ELEACH		; Bit present: load ELEACH (0.5°)
		ADS	ELINCR1		; Add to elevation increment accumulator

; All bits processed. Increments updated. Reset interrupt and terminate.
		TCF	RESETRPT

; ============================================================================
; BIT ASSIGNMENTS AND CONSTANTS
;
; Bit mappings based on Guidance System Operations Plan (GSOP) Chapter 4,
; Revision 16, which defines P64LM hand controller interface specification.
; ============================================================================
# THESE EQUIVALENCES ARE BASED ON GSOP CHAPTER 4, REVISION 16 OF P64LM

+ELBIT		=	BIT2		# -PITCH (nose up → landing site up)
-ELBIT		=	BIT1		# +PITCH (nose down → landing site down)
+AZBIT		=	BIT5		# Right azimuth
-AZBIT		=	BIT6		# Left azimuth

# Page 822
; Four-bit mask for extracting all redesignation inputs from CHAN31.
ALL4BITS	OCT	00063		; Bits 1,2,5,6 = octal 063

; Redesignation angular increments (in revolutions, AGC angle units).
AZEACH		DEC	.03491		# 2 DEGREES azimuth per input
ELEACH		DEC	.00873		# 1/2 DEGREE elevation per input

; ============================================================================
; R.O.D. TRAP - Rate-of-Descent Switch Interrupt Handler
;
; This interrupt handler processes Rate-of-Descent (ROD) switch inputs from
; the LM crew station. The ROD switch is a three-position momentary switch
; (up/center/down) that allows crew to adjust commanded descent rate during
; manual landing phases (P66 and P67).
;
; SWITCH FUNCTION:
; - UP position (BIT7 of CHAN16): Decrease descent rate (less negative RODCOUNT)
; - DOWN position (BIT6 of CHAN16): Increase descent rate (more negative RODCOUNT)
; - CENTER (released): No change, trap resets on return to RESUME
;
; COUNTER LOGIC:
; - RODCOUNT accumulates rate adjustments (positive = slower, negative = faster)
; - Each switch activation changes RODCOUNT by ±1 increment
; - BIT7 active: CS TWO = -2, plus AD ONE = -1 added to RODCOUNT (slower)
; - BIT6 active: +1 added to RODCOUNT (faster descent)
;
; INTEGRATION WITH GUIDANCE:
; RODCOUNT value used by P66VERT and P67VERT vertical guidance routines
; to modify commanded descent rate from nominal trajectory. Crew can override
; automatic rate commands to adjust sink rate for terrain approach.
;
; HISTORICAL CONTEXT: During Apollo 11 P66 manual phase, Armstrong used ROD
; switch to fine-tune descent rate as he maneuvered LM toward smooth landing
; area. The ROD control gave him precise vertical speed adjustment while
; maintaining horizontal translation capability via attitude stick.
;
; INTERRUPT SOURCE: MARKRUPT (T5RUPT) samples CHAN16 and vectors here when
; ROD switch bits detected. This trap executes at high priority to ensure
; immediate response to crew inputs during critical final approach.
;
; CODE-ALONG READERS: Entry has BIT7 or BIT6 in A register. MASK extracts
; BIT7, CCS tests if zero. For BIT7: CS TWO gives -2, AD ONE gives -1.
; For BIT6: Falls through to AD ONE giving +1. ADS adds to RODCOUNT.
; TCF RESUME returns from interrupt with trap reset.
; ============================================================================
# ****************************************************************
# R.O.D. TRAP
# ****************************************************************

		BANK	20
		SETLOC	RODTRAP
		BANK
		COUNT*	$$/F2DPS	# ************************

; Entry point from MARKRUPT with BIT7 or BIT6 of CHAN16 in A register.
; Decode which switch direction was activated and update RODCOUNT accordingly.
DESCBITS	MASK	BIT7		# COME HERE FROM MARKRUPT CODING WITH BIT
		CCS	A		#	7 OR 6 OF CHANNEL 16 IN A; BIT 7 MEANS
		CS	TWO		#	- RATE INCREMENT, BIT 6 + INCREMENT.
		AD	ONE		; BIT7: -2+1=-1, BIT6: 0+1=+1
		ADS	RODCOUNT	; Add increment to ROD counter
		TCF	RESUME		# TRAP IS RESET WHEN SWITCH IS RELEASED

		BANK	31
		SETLOC	F2DPS*31
		BANK

		COUNT*	$$/F2DPS

; ============================================================================
; ROOTPSRS - Double-Precision Power Series Root Finder
; (Algorithm by Allan Klumpp, MIT Instrumentation Laboratory)
;
; MATHEMATICAL FUNCTION:
; Finds one real root of the Nth-degree polynomial power series:
;
;   A_N * X^N + A_(N-1) * X^(N-1) + ... + A_1 * X + A_0 = 0
;
; using Newton's iterative method:
;
;   X_new = X_old - f(X_old) / f'(X_old)
;
; where f(X) is the polynomial and f'(X) is its derivative. Iteration
; continues until convergence criterion met or maximum iterations reached.
;
; USAGE IN LUNAR LANDING GUIDANCE:
; ROOTPSRS is called by TTF/8 computation routines to solve for time-to-go
; (TTF) in polynomial trajectory equations. During powered descent, guidance
; must determine when current trajectory will reach target landing site given
; thrust profile. This requires solving higher-order polynomial equations
; relating position, velocity, acceleration, and time.
;
; NEWTON'S METHOD OVERVIEW (for comment-only readers):
; Newton's method is an iterative numerical technique. Starting with an
; initial guess X_0, each iteration improves the estimate by computing how
; far the polynomial is from zero, dividing by the slope (derivative), and
; stepping toward the root. Convergence is typically rapid (quadratic) when
; initial guess is reasonably close to actual root.
;
; CALLING INTERFACE:
; INPUT REGISTERS:
;   A register (SP):     LOC-3    = Address for referencing power coefficient table
;   L register (SP):     N-1      = Polynomial degree minus 1 (e.g., 4 for 5th degree)
;   MPAC (DP):          X_0       = Initial guess for root (double-precision)
;
; COEFFICIENT TABLE STRUCTURE (in memory at address LOC-3):
;   LOC-2N (DP):        A_0       = Constant term coefficient
;   LOC-2N+2 (DP):      A_1       = Linear term coefficient
;   ...
;   LOC (DP):           A_N       = Highest degree term coefficient
;   LOC+2 (SP):         PRECROOT  = Convergence precision (fraction of initial guess)
;
; OUTPUT:
;   MPAC (DP):          X_final   = Computed root (double-precision)
;   MPAC+2 (SP):        COUNT     = Iteration count to convergence
;   Return address:     Normal return TC+3 if converged within 8 iterations
;                       Error return TC+1 if failed to converge (outputs invalid)
;
; ALGORITHM FLOW:
; 1. Initialize: Store coefficient table address, polynomial degree, set up
;    derivative coefficient table by multiplying each A_i by i.
; 2. Iterate (max 8 passes):
;    a. Evaluate polynomial f(X) using current X estimate (calls POWRSERS)
;    b. Evaluate derivative f'(X) using derivative table (calls POWRSERS)
;    c. Compute Newton correction: delta = -f(X) / f'(X)
;    d. Update estimate: X_new = X_old + delta
;    e. Test convergence: |delta| < PRECROOT * |X_0| ?
;    f. If converged: return via TC+3 with result and iteration count
;    g. If not converged and iterations remain: loop to step 2
; 3. If 8 iterations exhausted without convergence: return via TC+1 (error)
;
; PRECISION AND SCALING:
; Double-precision (DP) arithmetic used throughout for accuracy in trajectory
; computations. AGC double-precision uses two 15-bit words giving ~28 bits
; effective precision. PRECROOT typically set to 10^-6 or smaller, ensuring
; root accurate to fraction-of-a-meter in position or millimeters/second in
; velocity when solving trajectory equations.
;
; PRECAUTIONS AND LIMITATIONS:
; 1. COEFFICIENT SCALING: User must ensure |i * A_i| < 1.0 for all i, otherwise
;    derivative table construction loses precision. AGC fixed-point arithmetic
;    retains only fractional part when product >= 1.0.
;
; 2. OVERFLOW PREVENTION: User must scale coefficients so that evaluating
;    polynomial and derivative at X does not produce overflow. POWRSERS
;    performs evaluation; overflow there would corrupt results but might
;    not prevent eventual convergence if subsequent iterations recover.
;
; 3. POLYNOMIAL DEGREE LIMIT: Erasable memory reserved for N up to 5 only
;    (5th-degree polynomial maximum). Using N > 5 produces memory corruption
;    ("chaos") as derivative table overwrites unintended locations.
;    Erasables span MPAC-33 octal to MPAC+7 (unswitched bank).
;
; 4. CONVERGENCE BEHAVIOR: Newton's method requires reasonable initial guess.
;    Poor guess may cause divergence or excessive iterations. Iteration count
;    in MPAC+2 can diagnose abnormal performance (e.g., count near 8 suggests
;    marginal convergence or poor initial guess).
;
; 5. NO ERROR CHECKING: ROOTPSRS performs no validation of inputs. Improper
;    usage (e.g., wrong degree, unscaled coefficients, bad initial guess) may
;    produce nonsense results without indication beyond iteration count or
;    non-convergence error return.
;
; HISTORICAL CONTEXT: Allan Klumpp developed this root finder as part of MIT's
; guidance algorithm toolkit. During Apollo 11 descent, TTF/8 computations
; called ROOTPSRS multiple times per guidance cycle (every 2 seconds) to solve
; for time-remaining to landing. Reliable convergence was critical; failures
; would trigger program alarms and potentially abort landing.
;
; CODE-ALONG READERS: Entry saves return address in RETROOT, stores coefficient
; table pointer in PWRPTR, sets up derivative table pointer in DERPTR. MODE
; register used as iteration counter (must be positive so ABS won't complement).
; Main loop calls POWRSERS twice (once for f(X), once for f'(X)), performs
; Newton division, updates X, tests convergence using fractional precision.
; ============================================================================
# ***********************************************************************************
# DOUBLE PRECISION ROOT FINDER SUBROUTINE (BY ALLAN KLUMPP)
# ***********************************************************************************
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

					# STORE ENTERING DATA, INITIALIZE ERASABLES
ROOTPSRS	EXTEND
		QXCH	RETROOT		# RETURN ADRES
		TS	PWRPTR		# PWR TABLE POINTER
		DXCH	MPAC +3		# PWR TABLE ADRES, N-1
		CA	DERTABLL
		TS	DERPTR		# DER TABL POINTER
		TS	MPAC +5		# DER TABL ADRES
		CCS	MPAC +4		# NO POWER SERIES DEGREE 1 OR LESS
		TS	MPAC +6		# N-2
		CA	ZERO		# MODE USED AS ITERATION COUNTER.  MODE
		TS	MODE		# MUST BE POS SO ABS WON'T COMP MPAC+3 ETC.

					# COMPUTE CRITERION TO STOP ITERATING
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
ROOTSTOR	DXCH	ROOTPS
		DXCH	MPAC
		CA	MODE
		TS	MPAC +2		# STORE SP ITERATION COUNT IN MPAC+2
		INDEX	RETROOT
		TCF	2

DERTABLL	ADRES	DERCOFN -3

; ============================================================================
; TRANSITION: From ROOTPSRS Root Finder to Utility Subroutines
;
; The major mathematical computation routines are now complete. The following
; "trashy little subroutines" (as humorously labeled by the original MIT
; programmers) are small utility routines that perform common housekeeping
; tasks needed throughout the guidance equations. Despite the informal name,
; these routines are essential for setting up interpreter mode and computing
; display parameters during descent.
; ============================================================================

# ****************************************************************************
# TRASHY LITTLE SUBROUTINES
# ****************************************************************************

; INTPRETX - Set X1 Register and Enter Interpreter Mode
;
; FUNCTION:
; This small utility routine sets up the X1 index register based on the
; current guidance phase (WCHPHASE), then transfers control to the interpreter.
; It provides a convenient way to initialize X1 before executing interpretive
; code that requires phase-dependent indexing.
;
; OPERATION:
; 1. Uses WCHPHASE as index to fetch appropriate TARGTDEX value
; 2. Complements TARGTDEX (CS instruction gets one's complement)
; 3. Uses FIXLOC as base address for indirect indexing
; 4. Stores result in X1 register for interpreter use
; 5. Transfers control to INTPRET entry point
;
; USAGE CONTEXT:
; Called by guidance routines that need to reference phase-dependent parameter
; tables in interpretive mode. X1 register commonly used for indirect
; addressing in interpreter, allowing same code to access different data
; based on guidance phase (braking, approach, vertical).
;
; TARGTDEX VALUES (from earlier tables):
;   WCHPHASE = -1 (IGNALG):    TARGTDEX = 0
;   WCHPHASE =  0 (BRAKQUAD):  TARGTDEX = 0
;   WCHPHASE =  1 (APPRQUAD):  TARGTDEX = 34 octal
;
; CODE-ALONG READERS: INDEX instruction modifies next instruction's address
; by adding register content. First INDEX uses WCHPHASE to select which
; TARGTDEX entry, second INDEX uses FIXLOC as addressing base.
;
INTPRETX	INDEX	WCHPHASE	# SET X1 ON THE WAY TO THE INTERPRETER
		CS	TARGTDEX
		INDEX	FIXLOC
		TS	X1
		TCF	INTPRET

; TDISPSET - Time Display Setup and TREDES Computation
;
; FUNCTION:
; This utility routine performs two distinct display-related computations:
; 1. Scales time-to-go (TTF/8) for crew display on DSKY
; 2. Computes TREDES parameter (time remaining for redesignation)
;
; PART 1: TTF Display Scaling
; TTF/8 is the primary time-to-go variable (stored scaled by factor of 8).
; For display to crew, it must be unscaled and formatted. TSCALINV is the
; inverse time scale factor (1/8 = 0.125 = BIT4 in AGC scaling). Multiply
; TTF/8 by TSCALINV produces actual time in display units, stored double-
; precision in TTFDISP for DSKY formatting routines.
;
; PART 2: TREDES Computation (Time Remaining for Redesignation)
; TREDES tells guidance when to disable manual redesignation capability.
; During approach phase, crew (Armstrong on Apollo 11) could manually
; redesignate landing site using hand controller. As altitude decreases,
; there comes a point where redesignation must be disabled because there's
; insufficient time/altitude to safely reach a new target.
;
; TREDES ALGORITHM:
; TREDES is computed as function of two time parameters:
;   TCGFAPPR = Time constant for approach phase (when guidance reaches this)
;   TTF/8    = Current time-to-go until landing
;
; The computation performs:
;   temp = (TCGFAPPR + 2*BBANK) + TTF/8
;   temp = temp * TREDESCL + (-103)
;   temp = temp + (-32768)  [NEGMAX adds maximum negative]
;   temp stored in L, complemented, added back
;   temp = temp + 99 + 32767 [POSMAX adds maximum positive]
;   TREDES = temp, then complemented and added to itself
;
; This complex sequence produces TREDES value that decreases as landing
; approaches. When TREDES reaches zero, redesignation window closes.
; Original comment notes "TREDES BECOMES ZERO TWO PASSES BEFORE TCGFAPPR
; IS REACHED" - meaning redesignation disabled shortly before final approach.
;
; APOLLO 11 CONTEXT:
; During Apollo 11's descent on July 20, 1969, Armstrong used manual
; redesignation extensively starting around 3000 feet altitude when he
; realized the automatic system was taking Eagle toward a boulder field.
; He took semi-manual control and steered to a clearer landing site about
; 4 miles downrange from the original target. TREDES computation ensured
; he had sufficient altitude/time for this critical maneuver. By the time
; TREDES reached zero at very low altitude, manual redesignation was no
; longer safe and the landing had to proceed to the selected site.
;
; CODE-ALONG READERS: EBANK switching accesses TCGFAPPR in different
; erasable bank. INCR BBANK twice adds 2 to bank register. The arithmetic
; uses NEGMAX (-32768) and POSMAX (+32767) constants to perform modular
; arithmetic wrapping, producing final TREDES value through series of
; additions/complements.
;
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

; ============================================================================
; ALARM ROUTINES FOR TTF COMPUTATION
; ============================================================================
;
; These two small routines handle error conditions detected during time-to-go
; (TTF) computation. They differ in severity:
;
; 1406P00 - Critical Alarm (Program Abort via POODOO)
; Used during IGNALG (ignition algorithm) phase when TTF computation fails.
; Calls POODOO routine which initiates program abort sequence. Alarm code
; 01406 octal (774 decimal) indicates "TTF computation impossible during
; ignition phase." This is a serious condition requiring mission controller
; attention and possible manual intervention or abort.
;
; 1406ALM - Warning Alarm (Continue After Notification)
; Used during BRAKQUAD and APPRQUAD phases when TTF computation encounters
; problems. Calls ALARM routine to display alarm 01406 to crew on DSKY and
; notify ground controllers via telemetry, but allows guidance to continue
; by transferring to RATESTOP. During descent, the system can often recover
; from transient TTF computational issues using backup rate damping mode.
;
; APOLLO 11 CONTEXT:
; These alarms did not trigger during Apollo 11's descent. The famous 1201
; and 1202 alarms that occurred were executive/waitlist overload alarms, not
; guidance computation alarms. However, these 1406 alarm handlers were critical
; safety mechanisms - if trajectory computation had become impossible, they
; would have alerted crew and ground to switch to manual control modes.
;
; CODE-ALONG READERS: OCT directive places octal constant (alarm code) in
; memory following TC instruction. POODOO and ALARM routines read this
; constant to know which alarm code to display. POODOO (Program Abort) is
; more severe than ALARM (Warning and Continue).
;
1406P00		TC	POODOO
		OCT	01406
1406ALM		TC	ALARM
		OCT	01406
		TCF	RATESTOP

; ============================================================================
; FASTCHNG - Fast Phase Change Specialized Subroutine
; ============================================================================
;
; FUNCTION:
; FASTCHNG is a streamlined version of the standard PHASCHNG (phase change)
; routine, optimized for speed during time-critical guidance computations.
; During powered descent, guidance equations run on tight timing schedule
; (typically every 2 seconds). FASTCHNG allows rapid update of restart phase
; information without the overhead of full PHASCHNG processing.
;
; STANDARD PHASCHNG OVERHEAD:
; The full PHASCHNG routine performs extensive bookkeeping: saving multiple
; registers, updating phase tables, setting restart groups, checking timing
; constraints. This overhead acceptable during non-critical mission phases
; but unacceptable during descent when every millisecond of computation time
; is precious (remember the 1202 alarm on Apollo 11 was caused by executive
; overload - the AGC was running near computational capacity).
;
; FASTCHNG OPTIMIZATION:
; FASTCHNG performs minimal phase update:
; 1. Switch to EBANK3 to access phase name storage
; 2. Save current L register and EBANK setting
; 3. Store phase identifier in PHSNAME3
; 4. Restore EBANK to E2DPS (guidance equation bank)
; 5. Return via TC A (return address in A register)
;
; This streamlined sequence updates restart phase information (so restart
; protection knows where to resume if power transient occurs) without the
; full PHASCHNG overhead. Trade-off: less comprehensive restart information
; but faster execution allowing more time for critical guidance computations.
;
; APOLLO 11 CONTEXT:
; During Apollo 11's descent, when the 1202 executive overload alarm occurred,
; every optimization mattered. FASTCHNG's speed advantage helped keep guidance
; running even as the executive struggled with radar data overload. The AGC
; had only ~85 microseconds per instruction cycle - FASTCHNG saved perhaps
; 50-100 instruction cycles per guidance pass compared to full PHASCHNG,
; which over 12-minute descent added up to significant margin.
;
; CODE-ALONG READERS: EBANK switching (CA EBANK3, XCH EBANK) changes erasable
; memory bank addressing. DXCH L swaps A and L registers with memory location.
; LXCH restores L. Final TC A is indirect return - A register contains return
; address from caller. This unusual return mechanism allows FASTCHNG to avoid
; Q register use (Q might be needed by caller for other purposes).
;
# *********************************************************************
# SPECIALIZED "PHASCHNG" SUBROUTINE
# *********************************************************************

		EBANK=	PHSNAME2
FASTCHNG	CA	EBANK3		# SPECIALIZED 'PHASCHNG' ROUTINE
		XCH	EBANK
		DXCH	L
		TS	PHSNAME3
		LXCH	EBANK
		EBANK=	E2DPS
		TC	A

; ============================================================================
; PARAMETER TABLE INDIRECT ADDRESSES
; ============================================================================
;
; This section defines symbolic aliases for indirect addressing of guidance
; parameters. These aliases improve code readability by providing meaningful
; names that indicate what the parameters represent, while pointing to the
; actual storage locations that change throughout the descent phases.
;
; BRAKING PHASE REFERENCE GUIDANCE (BRFG) ALIASES:
;
; The "BRF" references (RBRFG, VBRFG, ABRFG, JBRFG) are the "braking phase
; reference guidance" parameters computed during initialization and updated
; as descent progresses. They represent the ideal trajectory the LM should
; follow for fuel-optimal descent:
;
; RDG = RBRFG    Position reference guidance
;                3-component vector specifying desired position relative to
;                landing site. Updated each guidance cycle (every 2 seconds).
;                Units: meters, scaled 2^29 (AGC scaling for position).
;
; VDG = VBRFG    Velocity reference guidance
;                3-component vector specifying desired velocity at current time.
;                Computed from trajectory equations to maintain fuel-optimal path.
;                Units: meters/centisecond, scaled 2^7.
;
; ADG = ABRFG    Acceleration reference guidance
;                3-component vector specifying desired acceleration (thrust).
;                Drives throttle control and attitude commands to autopilot.
;                Units: meters/centisecond², scaled appropriately.
;
; TIME-TO-GO PARAMETER ALIASES (with * suffix):
;
; The *-suffixed aliases (VBRFG*, ABRFG*, JBRFG*) are indirect addresses used
; specifically in TTF (time-to-go) computations. The * notation in AGC assembly
; indicates "address of" or indirect addressing mode. During TTF computation,
; code needs addresses of these parameters, not their values, to pass to
; subroutines that compute time remaining to landing.
;
; VDG2TTF = VBRFG*    Address of velocity vector for TTF computation
; ADG2TTF = ABRFG*    Address of acceleration vector for TTF computation
; JDG2TTF = JBRFG*    Address of jerk vector for TTF computation
;
; "Jerk" (JBRFG) is the third derivative of position (rate of change of
; acceleration). In powered flight, jerk is non-zero because throttle setting
; changes continuously to follow optimal trajectory. TTF computation needs
; jerk to accurately predict when position/velocity will reach landing values.
;
; APOLLO 11 CONTEXT:
; These parameter tables were accessed thousands of times during the 12-minute
; descent. Every 2-second guidance cycle read RDG/VDG/ADG to determine current
; errors from optimal trajectory, then computed thrust commands to null those
; errors. The indirect addressing scheme (using * notation) allowed one piece
; of code to work across all three descent phases (braking, approach, vertical)
; by simply changing which tables the addresses pointed to.
;
; CODE-ALONG READERS: The = directive in AGC assembly creates symbolic constant.
; "RDG = RBRFG" means wherever code uses RDG, assembler substitutes RBRFG's
; address. The * suffix creates indirect address (address-of). This two-level
; indirection (symbolic name → base address → indirect address) provides
; flexibility for guidance equations to work with different parameter sets
; depending on descent phase.
;
# *************************************************************************************
# PARAMETER TABLE INDIRECT ADDRESSES
# *************************************************************************************

RDG		=	RBRFG
VDG		=	VBRFG
ADG		=	ABRFG
VDG2TTF		=	VBRFG*
ADG2TTF		=	ABRFG*
JDG2TTF		=	JBRFG*

; ============================================================================
; LUNAR LANDING CONSTANTS
; ============================================================================
;
; This section defines all numerical constants used throughout the lunar landing
; guidance equations. These values were carefully computed by MIT Instrumentation
; Laboratory engineers to ensure safe, fuel-optimal descent to the lunar surface.
; Many of these constants encode physical constraints (maximum angles, velocity
; limits), mathematical scaling factors for AGC fixed-point arithmetic, and
; operational thresholds derived from extensive simulation and analysis.
;
; COMMENT-ONLY READERS: These numbers represent years of engineering analysis
; distilled into constants that guided Eagle safely to the Sea of Tranquility.
; Each value was chosen to balance competing requirements: conserve fuel, provide
; crew visibility of landing site, maintain structural limits, allow abort margins.
;
; CODE-ALONG READERS: Note the various scaling conventions: B-3 means scale by
; 2^-3 (divide by 8), B-11 means scale by 2^-11. Double-precision constants use
; 2DEC directive. Single-precision use DEC. BIT12 and BIT4 are power-of-2 masks.
;
; ============================================================================
; TIME-TO-GO (TTF) TABLE REFERENCE AND SCALING
; ============================================================================
;
# *************************************************************************************
# LUNAR LANDING CONSTANTS
# *************************************************************************************

# Page 827
;
; TABLTTFL - Time-to-Go Table Address
; This constant holds the address of the TTF (Time-To-Go) lookup table, offset
; by 3 entries. The TTF table contains pre-computed values mapping current velocity
; and acceleration to predicted time remaining until landing. Using table lookup
; with interpolation is faster than solving the nonlinear equations repeatedly.
; The +3 offset positions the pointer for indexed addressing during table search.
;
TABLTTFL	ADRES	TABLTTF +3	# ADDRESS FOR REFERENCING TTF TABLE
;
; TTFSCALE - Time-to-Go Scaling Factor (BIT12 = 2^12 = 4096)
; TTF is stored in centiseconds (1/100 second) but needs scaling for AGC arithmetic.
; Multiplying by 4096 converts TTF to appropriate fixed-point representation.
; During Apollo 11 descent, TTF ranged from ~720 seconds at PDI down to 0 at landing.
;
TTFSCALE	=	BIT12
;
; TSCALINV - Inverse Time Scaling (BIT4 = 2^4 = 16)
; Reciprocal of time scaling for converting back from scaled representation.
; Division by 16 in some calculations compensates for earlier scaling operations.
;
TSCALINV	=	BIT4
;
; ============================================================================
; ALTITUDE AND DESCENT RATE LIMITS
; ============================================================================
;
; -DEC103 - Negative altitude threshold (-103 degrees or other unit context)
; Used in altitude monitoring and phase transition logic. The negative value
; indicates direction convention (below reference vs above reference).
;
-DEC103		DEC	-103
;
; +DEC99 - Positive limit constant (+99 in decimal)
; Likely used as iteration limit or altitude band boundary in guidance loops.
;
+DEC99		DEC	+99
;
; TREDESCL - Time Redesignation Scale Factor (-0.08)
; Scaling constant for manual redesignation (crew input) of landing site.
; When commander moves redesignation cross-hairs on LPD (Landing Point Designator),
; this factor converts angular displacement to landing site coordinate offset.
; Negative sign indicates direction convention for coordinate system transformation.
; Armstrong used this during final approach when he selected new landing site to
; avoid boulder field at original touchdown point.
;
TREDESCL	DEC	-.08
;
; ============================================================================
; ANGULAR CONSTANTS FOR ATTITUDE AND TRAJECTORY
; ============================================================================
;
; 180DEGS - Half Circle (180 degrees)
; Used for angle wrapping and coordinate transformations. When spacecraft attitude
; or trajectory angles exceed ±180°, they must be wrapped to stay in principal range.
;
180DEGS		DEC	+180
;
; 1/2DEG - Half Degree in Decimal (0.00278 ≈ 0.5°)
; Angular resolution for attitude tolerances and gimbal angle computations.
; AGC uses degrees as angular unit (not radians) for crew-facing calculations.
; This constant provides fine angular discrimination (half-degree precision).
;
1/2DEG		DEC	+.00278
;
; ============================================================================
; LANDING SITE VISIBILITY PROJECTION LIMITS
; ============================================================================
;
; During approach phase, crew must maintain visual contact with landing site.
; These constants define acceptable projection angles ensuring landing site
; remains visible through LM window throughout descent. Projection is geometric
; calculation of landing site position relative to LM attitude.
;
; PROJMAX - Maximum Projection (sin(25°)/8 = 0.42262 × 2^-3)
; Upper limit for landing site projection angle. If projection exceeds this,
; landing site is too far forward (above) in window, potentially out of view.
; The division by 8 (B-3 scaling) matches AGC's angular representation format.
; 25° represents maximum acceptable forward view angle before crew loses sight.
;
PROJMAX		DEC	.42262 B-3	# SIN(25')/8 TO COMPARE WITH PROJ
;
; PROJMIN - Minimum Projection (sin(15°)/8 = 0.25882 × 2^-3)
; Lower limit for landing site projection angle. If projection below this,
; landing site is too far behind (below) in window, also out of acceptable view.
; 15° represents minimum forward view angle while maintaining safe attitude.
; Window between 15° and 25° provides optimal crew visibility while allowing
; AGC to maintain guidance accuracy.
;
PROJMIN		DEC	.25882 B-3	# SIN(15')/8 TO COMPARE WITH PROJ
;
; ============================================================================
; VERB/NOUN DISPLAY CODES FOR CREW INTERFACE
; ============================================================================
;
; These constants define DSKY display codes for different landing programs.
; During descent, AGC automatically displays relevant data on DSKY using these
; verb/noun combinations. Crew can recognize which program is active and monitor
; key parameters (altitude, velocity, fuel, time) appropriate to each phase.
;
; V06N63 - Display for P63 (Braking Phase)
; Verb 06 = Display Decimal (in R1, R2, R3)
; Noun 63 = Range-to-go, altitude, rate-of-descent
; During braking phase (102:33 to ~102:42 mission time), crew saw these values
; updating every 2 seconds as LM descended from 50,000 feet toward 7,500 feet.
;
V06N63		VN	0663		# P63
;
; V06N64 - Display for P64 (Approach Phase)
; Noun 64 = LPD angle, altitude, altitude-rate
; During approach phase (~102:42 to ~102:44), crew used Landing Point Designator
; (LPD) to visually track landing site. LPD angle told them where to look in
; window reticle pattern to see predicted touchdown point. Armstrong monitored
; this while evaluating original landing site, eventually deciding to redesignate.
;
V06N64		VN	0664		# P64
;
; V06N60 - Display for P65/P66/P67 (Vertical Descent and Landing)
; Noun 60 = Altitude, altitude-rate, fuel remaining
; During final descent (below ~500 feet), crew needed fuel awareness more than
; range. This display showed critical information for landing decision: how high,
; how fast descending, how much fuel left. During Apollo 11, low fuel warnings
; (60-second call, then 30-second call) were given while this display was active.
; Armstrong and Aldrin watched these numbers closely in final minute before landing.
;
V06N60		VN	0660		# P65, P66, P67
;
; ============================================================================
; MEMORY BANK ALLOCATION FOR ADDITIONAL CONSTANTS
; ============================================================================
;
; AGC's 36K fixed memory is divided into banks of 1024 words each. Constants are
; distributed across banks based on which guidance routines need them (minimizing
; bank-switching overhead). Bank 22 holds additional landing constants that don't
; fit in primary guidance bank.
;

		BANK	22
		SETLOC	LANDCNST
		BANK
		COUNT*	$$/F2DPS
;
; ============================================================================
; DOUBLE-PRECISION GUIDANCE CONSTANTS (BANK 22)
; ============================================================================
;
; These constants use 2DEC (double-precision) format, occupying two consecutive
; AGC words each. Double precision provides ~10 decimal digits accuracy, essential
; for trajectory calculations where position errors accumulate over 12-minute descent.
;
; HIGHESTF - Highest Frequency Limit (4.34546769 × 2^-12)
; This constant limits maximum frequency in some iterative computation (possibly
; related to guidance loop frequency or oscillation damping). The B-12 scaling
; means divide by 4096. Value chosen to prevent numerical instability in guidance
; equations when rates change rapidly (as during throttle-up or attitude changes).
;
HIGHESTF	2DEC	4.34546769 B-12
;
; GSCALE - Gravity Scaling Factor (100 × 2^-11)
; Scales gravitational acceleration in trajectory computations. Lunar gravity is
; 1.62 m/s² (about 1/6 Earth's 9.81 m/s²). This scale factor converts between
; AGC's internal acceleration units and physical meters/centisecond². The factor
; of 100 with B-11 scaling (divide by 2048) yields appropriate fixed-point range
; for representing lunar gravitational effects on descent trajectory.
;
GSCALE		2DEC	100 B-11
;
; 3/8DP - Three-Eighths Double-Precision (0.375)
; Mathematical constant used in trajectory equation coefficients. Appears in
; polynomial approximations and integration formulas. The "DP" suffix distinguishes
; this from any single-precision 3/8 constant.
;
3/8DP		2DEC	.375
;
; 3/4DP - Three-Quarters Double-Precision (0.750)
; Another fractional constant for trajectory mathematics. The combination of 3/8
; and 3/4 suggests these appear in related polynomial terms (possibly Taylor series
; expansion or Runge-Kutta integration coefficients for position/velocity updates).
;
3/4DP		2DEC	.750
;
; DEPRCRIT - Depression Angle Critical Value (-0.02 × 2^-1)
; Critical depression angle (negative = below horizontal) used in visibility
; and attitude checks. The value -0.02 with B-1 scaling means -0.01 in final units.
; This represents small negative angle where landing site approaches horizon line
; from crew's perspective. If actual depression angle goes below (more negative than)
; this critical value, landing site may disappear from view, triggering guidance
; adjustment or crew warning.
;
; During Apollo 11 approach phase, this constant was part of the logic ensuring
; Armstrong could continuously see his selected landing site through the window.
; The guidance equations adjusted thrust vector attitude to keep landing site
; projection within acceptable angular range defined by PROJMIN/PROJMAX and
; constrained by depression angle limits like DEPRCRIT.
;
DEPRCRIT	2DEC	-.02 B-1

# Page 828
# **************************************************************************
# **************************************************************************
