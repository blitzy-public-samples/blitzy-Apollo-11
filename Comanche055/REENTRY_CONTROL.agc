# Copyright:	Public domain.
# Filename:	REENTRY_CONTROL.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	844-882
# Mod history:	2009-05-08 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-23 RSB	In a couple of 2OCT statements, removed the
#				space between the first and second octal words.
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

; ============================================================================
; FILE: REENTRY_CONTROL.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: re-entry
;
; TL;DR: Atmospheric reentry guidance implementing lift vector control for
;        range and crossrange targeting. Computes bank angle modulation commands
;        to steer Command Module through entry corridor to Pacific Ocean
;        splashdown point. Apollo 11 execution July 24, 1969 achieved precise
;        landing near recovery ships.
;
; COMMENT-ONLY READERS: This program steered Apollo 11 through reentry to land
;        at the planned splashdown point in the Pacific Ocean.
; CODE-ALONG READERS: Study lift vector steering algorithms, bank angle control
;        logic, range/crossrange error correction, and entry corridor management.
; ============================================================================

# Page 844
# ENTRY INITIALIZATION ROUTINE
# ----------------------------

; ============================================================================
; ENTRY INITIALIZATION
;
; The Command Module has separated from the Service Module and is oriented
; for atmospheric entry. This routine prepares the guidance computer for the
; critical reentry phase that will guide the spacecraft through Earth's
; atmosphere to a precise splashdown in the Pacific Ocean.
;
; For Apollo 11, this sequence began on July 24, 1969, as the crew prepared
; for their return to Earth after the historic lunar landing mission.
; ============================================================================

		BANK	25
		SETLOC	REENTRY
		BANK

		COUNT*	$$/ENTRY
		EBANK=	RTINIT

EBENTRY		=	EBANK7
EBAOG		EQUALS	EBANK6
NTRYPRIO	EQUALS	PRIO20		# (SERVICER)
CM/FLAGS	EQUALS	STATE +6

; STARTENT - Entry Initialization Routine
; Called from CM/POSE program (P61-P67 entry programs).
; Initializes all entry guidance parameters, control flags, and target data.

STARTENT	EXIT			# MM = 63

; Entry Control Flags Initialization
; The CM/FLAGS word controls the entire reentry sequence through a set of
; software switches. These flags track the spacecraft's progress through
; each phase of atmospheric entry and enable/disable specific control modes.
;
					# COME HERE FROM CM/POSE.  RESTARTED IN CM/POSE.
		CS	ENTMASK		# INITIALIZE ALL SWITCHES TO ZERO
					# EXCEPT LATSW, ENTRYDSP, AND GONEPAST.
					# GONEBY 112D BIT8 FLAG7, SELF-INITIALIZING.
		INHINT
		MASK	CM/FLAGS
					# ENTRYDSP = 92D B13
					# GONEPAST=95D B10.	RELVELSW=96D B9
					# EGSW = 97D B8		NOSWITCH = 98D B7
					# HIND=99D B6		INRLSW=100D B5
					# LATSW=101D B4		.05GSW=102D B3

; Set initial control flags for entry guidance:
; ENTRYDSP - Enable entry display on DSKY for crew monitoring
; LATSW - Lateral (crossrange) control switch
; GONEPAST - Flag indicating passage of range prediction point

		AD	ENTRYSW		# SET ENTRYDSP, LATSW, GONEPAST.
		TS	CM/FLAGS

		RELINT

; Entry Guidance Parameter Initialization
; Load nominal entry parameters from pad-loaded constants (LODPAD, LADPAD)
; and compute derived values for lift-to-drag ratio control.
;
; These parameters define the spacecraft's lift vector control authority
; and the target corridor for atmospheric entry. The lift-to-drag ratio (L/D)
; determines how aggressively the spacecraft can maneuver to correct range
; and crossrange errors.

		TC	INTPRET

		SLOAD
			LODPAD		# Lift-over-drag (L/D) initial value
		STORE	LOD

		SLOAD
			LADPAD		# Lateral acceleration derivative
		STORE	LAD

; Compute minimum L/D for control corridor:
; L/DCMINR = LAD * COS(15°)
; This defines the minimum lift-to-drag ratio before control authority is lost.

		DMP			# L/DCMINR = LAD COS(15)
			COS15
		STODL	L/DCMINR
			LATSLOPE
		DMP	SR1		# KLAT = LAD/24
			LAD
# Page 845
		STODL	KLAT		# Lateral control gain
			Q7F
		STODL	Q7		# Q7 = Q7F (heating rate limit)
			NEARONE		# 1.0 -1BIT
		STODL	FACTOR		# Range correction factor
			LAD
		SIGN	DCOMP
			HEADSUP		# MAY BE NOISE FOR DISPLAY P61
		STCALL	L/D		# L/D = - LAD SGN(HEADSUP)

			STARTEN1	# RETURN VIA GOTOADDR

; Compute lateral control parameters and roll direction:
; Calculate the cross product of velocity and position to determine the
; lateral acceleration direction needed for crossrange control.

		VLOAD	VXV
			VN		# (-7) M/CS
			UNITR		# .5 UNIT		REF COORDS
		UNIT	DOT
			RT		# RT/2 TARGET VECTOR	REF COORDS
		STORE	LATANG		# LATANG = UNI.RT /4
		DCOMP	RTB
			SIGNMPAC
		STODL	K2ROLL		# K2ROLL = -SGN(LATANG)

; Compute Q2 parameter for range control:
; Q2 = -1152 + 500*LAD
; This parameter determines the sensitivity of range correction.

			LAD
		DMP	DAD
			Q21
			Q22
		STORE	Q2		# Q2 = -1152 + 500 LAD

; Set up the guidance phase sequence:
; GOTOADDR will be set to INITROLL for the first entry guidance pass.
; This begins the active guidance sequence after initialization.

		SSP	SSP
			GOTOADDR	# SET SELECTOR FOR INITIAL PASS
			INITROLL
			POSEXIT
			SCALEPOP	# SET CM/POSE TO CONTINUE AT SCALEPOP

		RTB
			SERVNOUT	# OMIT INITIAL DISPLAY, SINCE 1ST GUESSBAD

; ============================================================================
; STARTEN1 - Calculate Initial Target Vector
;
; Computes the initial target vector (RTINIT) pointing to the splashdown
; location in the Pacific Ocean. For Apollo 11, this was carefully calculated
; to place the Command Module near recovery ships waiting at the planned
; recovery zone.
;
; The routine converts latitude/longitude coordinates into a reference-frame
; unit vector and calculates the initial range angle from current position
; to target.
; ============================================================================

# CALCULATE THE INITIAL TARGET VECTOR: RTINIT, ALSO RTEAST, RTNORM AND RT.  ALL ARE .5 UNIT AND IN
# REFERENCE COORDINATES.

STARTEN1	STQ	VLOAD
			GOTOADDR
			LAT(SPL)	# TARGET COORDINATES
		CLEAR	CLEAR		# DO CALL USING PAD RADIUS.  WILL UNIT IT.
			ERADFLAG	# ANYWAY.
			LUNAFLAG
		STODL	LAT
			3ZEROS

; Set target altitude to zero (sea level) for splashdown point.

		STODL	LAT +4		# SET ALT=0.
			PIPTIME		# ESTABLISH RTINIT AT TIME OF PRESENT
# Page 846
					# RN AND VN.
		STCALL	TIME/RTO	# SAVE TIME BASE OF RTINIT.
			LALOTORV	# C(MPAC) =TIME  (PIPTIME)
		UNIT			# ANSWER IN ALPHAV ALSO
		STODL	RTINIT		# .5 UNIT TARGET		REF COORDS
			500SEC		# NOMINAL ENTRY TIME FOR P63
					# TIME/RTO = PIPTIME, STILL.
		STCALL	DTEAROT		# INITIALIZE EARROT
			EARROT1		# GET RT

; Calculate initial range angle (THETAH) from current position to target.
; This represents the great-circle arc distance expressed as an angle.

		DOT	SL1
			UNITR		# RT/2 IN MPAC
		ACOS
		STCALL	THETAH		# RANGE ANGLE /360
			GOTOADDR	# RETURN TO CALLER

500SEC		2DEC	50000 B-28	# CS

ENTMASK		OCT	11774
ENTRYSW		OCT	11010		# ENTRYDSP B13, GONEPAST B10, LATSW B4
# Page 847

; ============================================================================
; SCALEPOP - Entry Guidance Main Loop
;
; This is the main entry point for periodic execution of the reentry guidance
; algorithm. Called repeatedly by the servicer during atmospheric entry to
; update target vectors and transition between guidance phases.
;
; The routine updates the target vector for Earth rotation, then jumps to
; the appropriate reentry phase based on the current flight regime.
; ============================================================================

SCALEPOP	CALL
			TARGETNG	# Update target for Earth rotation

		EXIT

REFAZE10	TC	PHASCHNG
		OCT	10035		# SERVICER 5.3 RESTART AT REFAZE10

		TC	INTPRET

; ============================================================================
; REENTRY GUIDANCE PHASE SEQUENCER
;
; The reentry guidance proceeds through five distinct phases:
;
; 1. INITROLL - Initial roll orientation to proper bank angle
; 2. HUNTEST - "Hunt" for proper lift vector to achieve target range
; 3. UPCONTRL - Lift-up control to manage heating and deceleration
; 4. KEP2 - Kepler (ballistic) phase monitoring
; 5. PREDICT3 - Final phase guidance and chute deployment preparation
;
; GOTOADDR selector determines which phase executes on each guidance cycle.
; The phases automatically transition based on velocity, altitude, and
; range-to-target criteria.
; ============================================================================

# JUMP TO PARTICULAR RE-ENTRY PHASE:
#SEQUENCE
		GOTO
			GOTOADDR

# GOTOADDR CONTAINS THE ADDRESS OF THE ROLL COMMAND EQUATIONS TO THE CURRENT PHASE OF
# RE-ENTRY.  SEQUENCING IS AS FOLLOWS:
#
# INITROLL	ADDRESS IS SET HERE INITIALLY.  HOLDS INITIAL ROLL ATTITUDE UNTIL  KAT  IS EXCEEDED.  THEN HOLDS NEW ROLL
#		ATTITUDE UNTIL  VRTHRESH  IS EXCEEDED.  THEN BRANCHES TO
#
# HUNTEST	THIS SECTION CHECKS TO SEE IF THE PREDICTED RANGE AT NOMINAL   L/D FROM PRESENT CONDITIONS IS LESS
#		THAN THE DESIRED RANGE.
#			IF NOT --- A ROLL COMMAND IS GENERATED BY THE CONSTANT DRAG CONTROLLER.
#			IF SO  --- CONTROL AND GOTOADDR ARE SET TO UPCONTRL.
#		USUALLY NO ITERATION IS INVOLVED EXCEPT IF THE RANGE DESIRED IS TOO LONG ON THE FIRST PASS THROUGH
#		HUNTEST.
#
# UPCONTRL	CONTROLS ROLL DURING THE SUPER-CIRCULAR PHASE.  UPCONTRL IS TERMINATED EITHER
#			(A) WHEN THE DRAG (AS MEASURED BY THE PIPAS) FALLS BELOW Q7, OR
#			(B) IF RDOT IS NEGATIVE AND REFERENCE VL EXCEEDS V.
#		IN CASE (A),  GOTOADDR  IS SET TO  KEP2  AND IN CASE (B), TO  PREDICT3  SKIPPING THE KEPLER PHASE OF
#		ENTRY.
#
# KEP2		GOTOADDR IS SET HERE DURING THE KEPLER PHASE TO MONITOR DRAG.  THE SPACECRAFT IS INSTANTANEOUSLY
#		TRIMMED IN PITCH AND YAW TO THE COMPUTED RELATIVE VELOCITY.  THE LAST COMPUTED ROLL ANGLE IS MAINTAINED.
#		WHEN THE MEASURED DRAG EXCEEDS Q7 +0.5,  GOTOADDR  IS SET TO
#
# PREDICT3	THIS CONTROLS THE FINAL SUB-ORBITAL PHASE. ROLL COMMANDS CEASE
#		WHEN  V  IS LESS THAN  VQUIT .  AN EXIT IS MADE TO
#
# P67.1		THE LAST COMPUTED ROLL ANGLE IS MAINTAINED.  RATE DAMPING IS DONE IN PITCH AND YAW.  PRESENT LATITUDE
#		AND LONGITUDE ARE COMPUTED FOR DISPLAY.
#		ENTRY IS TERMINATED WHEN DISKY RESPONSE IS MADE TO TO THIS FINAL FLASHING DISPLAY.

# Page 848
# PROCESS AVERAGE G OUTPUT...SCALE IT AND GET INPUT DATA

# * START  TARGETING ...

		EBANK=	RTINIT

					# TARGETNG IS CALLED BY P61, FROM GROUP 4.
					# TARGETNG IS CALLED BY ENTRY, FROM GROUP 5.

					# ALL MM COME HERE.
TARGETNG	BOFF	VLOAD		# ENTER WITH PROPER EB FROM CM/POSE(TEST)
			RELVELSW	# RELVELSW = 96D BIT9
			GETVEL		# WANT INERTIAL VEL.  GO GET IT.
			-VREL		# NEW V IS RELATIVE, CONTINUE

		VCOMP	GOTO		# (VREL) = (V) + KWE UNITR*UNITW
			GETUNITV -1	# - VREL WAS LEFT BY CM/POSE

GETVEL		VLOAD	VXSC		# INERTIAL V WANTED
			VN		# KVSCALE = (12800 / .3048) / 2VS
			KVSCALE		# KVSCALE = .81491944
		STORE	VEL		# V/2 VS

GETUNITV	UNIT	STQ
			60GENRET
		STODL	UNITV
			34D
		STORE	VSQUARE		# VSQ/4

		DSU			# LEQ = VSQUARE - 1
			FOURTH		# 4 G-S FULL SCALE
		STODL	LEQ		# LEQ/4

			36D
		STOVL	V		# V/2 VS = VEL/2 VS

			VEL
		DOT	SL1		# RDOT= V.UNITR
			UNITR
		STOVL	RDOT		# RDOT /2 VS

			DELV		# PIPA COUNTS IN PLATFORM COORDS.
		ABVAL	DMP
			KASCALE
		SL1	BZE
			SETMIND
DSTORE		STOVL	D		# ACCELERATION USED TO APPROX DRAG
			VEL
		VXV	UNIT		# UNI = UNIT(V*R)
# Page 849
			UNITR
		STORE	UNI		# .5 UNI		REF COORDS.

		BOFF	DLOAD
			RELVELSW
			GETETA
			3ZEROS
UPDATERT	DSU	DAD		# PIPTIME-TIME/RTO =ELAPSED TIME SINCE
					# RTINIT WAS ESTABLISHED.
			TIME/RTO
			PIPTIME
		STCALL	DTEAROT		# GET PREDICTED TARGET VECTOR RT

			EARROT2
		DOT	SETPD		# SINCE (RT) UNIT VECT, THIS IS 1/4 MAX
			UNI		# LATANG = RT.UNI
			0
		STOVL	LATANG		# LATANG = MAC LATANG / 4

			RT
		CLEAR
			GONEBY		# SHOW HAVE NOT GONE PAST TARGET.
		VXV	DOT		# IF RT*UNITR.UNI NEG, GONEBY=1
			UNITR		# GONEPAST IS CONDITIONAL SW SET IN
			UNI		# FINAL PHASE.
		BPL	SET
			+2
			GONEBY		# SHOW HAVE GONE PAST TARGET.

		VLOAD
			RT
GETANGLE	DOT	DSU		# THETA = ARCCOS(RT.UNITR)
			UNITR
			NEAR1/4		# TO IMPROVE ACCURACY, CALC RANGE BY
		BPL	DAD		# TINYTHET IF HIGH ORDER PART OF
			TINYTHET	# ARCCOS ARGUMENT IS ZERO
			NEAR1/4
		SL1	ACOS
THETDONE	STORE	THETAH		# THETAH/360
					# HI WORD, LO BIT =1.32 NM=360 60/16384

		BON	DCOMP
			GONEBY		# =1 IF HAVE GONE PAST TARGET.
					# (SIGN MAY BECOME ERRATIC VERY NEAR
					# TARGET DUE TO LOSS OF PRECISION.)
			+1
		STODL	RTGON67		# RANGE ERROR:  NEG IF WILL FALL SHORT.

			D
		DSU	BMN
# Page 850
			.05G
			NO.05G
		SET	VLOAD
			.05GSW
			DELVREF
		PUSH	DOT
			UXA/2
		SL1	DSQ
		PDVL	VSQ		# EXCHANGE WITH PDL.
		DSU	DDV
			0
		BOV	SQRT
			NOLDCALC	# OVFL LAST CLEARED IN EARROT2 ABOVE.
		STORE	L/DCALC

NOLDCALC	GOTO
			60GENRET

NO.05G		CLEAR	GOTO		# THIS WAY FOR DAP. (MAY INTERRUPT)
			.05GSW		# .05GSW = 102D B3
			NOLDCALC	# KEEP SINGLE EXIT FOR TARGETNG

# Page 851
# SUBROUTINES CALLED BY SCALEPOP (TARGETING):

		BANK	26
		SETLOC	REENTRY1
		BANK

		COUNT*	$$/ENTRY

; ============================================================================
; GETETA - Drag Acceleration Update and Range Angle Computation
;
; This routine updates the drag acceleration estimate D based on current
; velocity and altitude rate. The drag term is used to predict future
; trajectory behavior for guidance commands.
;
; During Apollo 11's reentry on July 24, 1969, this computation executed
; continuously to track the changing atmospheric drag as the Command Module
; descended through the entry corridor toward the Pacific splashdown point.
;
; The routine also computes ETA (range angle to target) differently depending
; on flight phase - using inertial velocity early in entry, then switching
; to velocity relative to rotating Earth for final guidance accuracy.
; ============================================================================

GETETA		DLOAD	DDV		# D = D +D(-RDOT/HS -2D/V)  DT/2
					# DT/2 = 2/2 =1
			RDOT
			-HSCALED
		PDDL	DMP
			D
			-KSCALE
		DDV	DAD
			V
					# -RDOT/HS FROM PDL.
		DMP	DAD
			D
			D
		STORE	D

		BON	DLOAD		# EGSW INDICATES FINAL PHASE.
			EGSW
			SUBETA
			THETAH
		DMP	GOTO
			KTETA		# = 1000X2PI/(2)E14 163.84
			UPDATERT

; ----------------------------------------------------------------------------
; SUBETA - Switch from Inertial to Relative Velocity Reference
;
; During early reentry, guidance uses inertial velocity. As the spacecraft
; slows and approaches the target, Earth's rotation becomes significant.
; This section switches to velocity relative to the rotating Earth for
; improved targeting accuracy in the final phase.
; ----------------------------------------------------------------------------

SUBETA		DLOAD	DSU		# SWITCH FROM INERTIAL TO RELATIVE VEL.
			V
			VMIN
		BPL	SET
			SUBETA2
			RELVELSW

SUBETA2		DLOAD	DMP

			THETAH
			KT1		# KT1 = KT
		DDV	GOTO
			V		# KT = RE(2 PI)/2 VS 16384 163.84/ 2 VSAT
			UPDATERT

SETMIND		DLOAD	GOTO
			1BITDP
			DSTORE

# Page 852
TINYTHET	DSU	ABS		# ENTER WITH X-.249
			1BITDP +1	# GET 1/4 - MPAC
		SL	SQRT		# SCALE UP BEFORE SQRT
			13D		# HAS FACTOR FOR UP SCALING
		DMP	GOTO
			KACOS
			THETDONE

# Page 853
# * START	INITIAL ROLL ...

		BANK	25
		SETLOC	REENTRY
		BANK

		COUNT*	$$/ENTRY

; ============================================================================
; PHASE 1: INITROLL - Initial Roll Orientation for Entry
;
; This is the first active guidance phase after entry interface. The spacecraft
; must establish the correct bank angle to generate lift in the proper direction
; for reaching the target landing point.
;
; The routine computes a bank angle acceleration command (KA) based on the
; current range error (LEQ). The spacecraft rolls to align its lift vector
; to null out cross-range and down-range errors simultaneously.
;
; During Apollo 11's entry, this phase established the initial bank orientation
; as the Command Module began feeling atmospheric drag above 400,000 feet altitude.
; The guidance transitioned to HUNTEST phase once drag exceeded 0.05g.
; ============================================================================

					# MM = 63 , 64 ..
INITROLL	BON	BOFF		# IF D- .05G NEG, GO TO LIMITL/D
			INRLSW
			INITRL1
			.05GSW
			LIMITL/D

; Bank angle acceleration command computation. KA determines how fast the
; spacecraft should change its bank angle to correct range errors. The
; cubic relationship (LEQ^3) provides strong correction for large errors
; while avoiding oscillation near the target.

					# MM = 64, NOW
					#	      3
					# KA = KA1 LEQ  + KA2
		DLOAD	DSQ		# Load range error and square it
			LEQ		# LEQ = range error to target
		DMP	DDV
			LEQ
			1/KA1		# = 25 /(64  1.8)
		DAD	RTB
			KA2		# = .2
			P64		# ROLLC		VI		RDOT
					# XXX.XX DEG	XXXXX. FPS	XXXXX. FPS
		STORE	KAT

		DSU	BMN
			KALIM
			+4
		DLOAD
			KALIM
		STORE	KAT
		DLOAD	DSU		# IF V-VFINAL1 NEG, GO TO FINAL PHASE.
			V
			VFINAL1
		CLEAR	BPL		# (CAN'T CLEAR INRLSW AFTER HERE: RESTARTS)
			GONEPAST	# GONEPAST WAS INITIALLY SET=1 TO FORCE
					# ROLLC TO REMAIN AS DEFINED BY HEADSUP
					# UNTIL START OF P64.  (UNTIL D > .05G)
			D0EQ
		SSP	GOTO
			GOTOADDR
			KEP2		# AND IDLE UNTIL D > 0.2 G.  (NO P66 HERE)
			INROLOUT	# GO TO LIMITL/D AFTER SETTING INRLSW.

; ----------------------------------------------------------------------------
; D0 Drag Level Computation
;
; D0 represents the target drag level for the UPCONTRL phase. This value
; is computed based on range error to ensure the spacecraft dissipates the
; correct amount of energy to reach the target. Larger range errors require
; higher drag (lift-down orientation) to shorten the trajectory.
; ----------------------------------------------------------------------------

D0EQ		DLOAD	DMP		# D0 = KA3 LEQ + KA4
# Page 854
			LEQ		# Range error
			KA3		# Drag-to-range sensitivity
		DAD
			KA4
		STORE	D0		# D0/805
		BDDV	BOV
			C001		# (-4/25 G) B-8
			+1		# CLEAR OVFIND, IF ON.
		STODL	C/D0		# (-4/D0) B-8
			LAD		# IF V-VFINAL +K(RDOT/V)CUBED POS,L/D=-LAD
		STODL	L/D
			RDOT
		DDV	PUSH
			V
		DSQ	DMP
		DDV	DSU
			1/K44
			VFINAL
					#		    3
					# V-VFINAL +(RDOT/V)  / K44	OVFL $

		DAD	BOV
			V
			INROLOUT	# GO TO LIMITL/D AFTER SETTING INRLSW.
		BMN	DLOAD
			INROLOUT	# GO TO LIMITL/D AFTER SETTING INRLSW.
			LAD		# Load desired L/D
		DCOMP
		STORE	L/D		# Store negative LAD as initial L/D

; ----------------------------------------------------------------------------
; INITROLL Phase Completion and Transition Logic
;
; INROLOUT marks the end of pre-0.05g guidance (before drag becomes significant).
; Sets INRLSW flag for restart protection, then proceeds to check if guidance
; should transition from INITROLL to HUNTEST phase.
; ----------------------------------------------------------------------------

					# SET INRLSW AT END FOR RESTART PROTECTION
INROLOUT	BOFSET			# END OF PRE .05G PATH OF INITROLL.
			INRLSW		# SWITCH IS ZERO INITIALLY.
			LIMITL/D	# (GO TO)

; ----------------------------------------------------------------------------
; Transition Tests: INITROLL to HUNTEST
;
; KATEST checks if drag level has reached the threshold (KAT) where active
; hunting for the lift vector should begin. If drag is still below KAT,
; remains in constant-drag (CONSTD) mode.
;
; INITRL1 checks the altitude rate (RDOT) to determine if the spacecraft has
; reached the proper flight regime for HUNTEST. When RDOT + VRCONT becomes
; positive, the spacecraft has descended far enough into the atmosphere that
; precise lift vector control is required to reach the target.
; ----------------------------------------------------------------------------

KATEST		DLOAD	DSU		# IF KAT - D POS, GO TO CONSTD
			KAT		# Drag threshold for hunt initiation
			D		# Current drag level
		BPL	GOTO		# IF POS, OUT WITH COMMAND VIA LIMITL/D
			LIMITL/D
			CONSTD

INITRL1		DLOAD	DAD		# IF RDOT + VRCONT POS, GO TO HUNTEST
			RDOT		# Altitude rate (negative when descending)
			VRCONT		# Velocity threshold for hunt phase
		BMN	CALL		# If negative, not ready for hunt yet
			KATEST

			FOREHUNT	# Initialize HUNTEST phase

# Page 855
# * START	HUNT TEST ..

; ============================================================================
; PHASE 2: HUNTEST - Lift Vector Hunting for Range Targeting
;
; This is the primary entry guidance phase where the spacecraft actively
; adjusts its bank angle to "hunt" for the optimal lift vector that will
; achieve the target range. The algorithm computes predicted range based on
; current velocity, drag, and lift-to-drag ratio, then commands bank angle
; changes to null range errors.
;
; The routine computes:
; - A0, A1: Drag acceleration levels for range prediction
; - VL: Velocity at which lift-up control should begin
; - GAMMAL: Flight path angle for optimal trajectory
; - Range prediction to target
; - Bank angle commands to achieve predicted range
;
; During Apollo 11's entry on July 24, 1969, HUNTEST guided the Command Module
; through the critical heating phase, modulating bank angle to target the
; Pacific Ocean splashdown point near the recovery ships.
;
; The phase continues until velocity drops below VLMIN, then transitions to
; UPCONTRL for final energy management.
; ============================================================================

					# MM = 64
		SSP			# INITIALIZE HUNTEST ON FIRST PASS
			GOTOADDR
			HUNTEST		# MUST GO AFTER FOREHUNT FOR RESTARTS.

HUNTEST		DLOAD
			D		# Current drag level
		STODL	A1		# A1/805 = A1/25G

			LAD
		STODL	TEM1B
			RDOT
		BMN	DLOAD		# IF RDOT NEG,TEM1B=LAD, OTHERWISE = LEWD
			A0CALC
			LEWD		# Use lift-down L/D for positive RDOT
		STODL	TEM1B

; ----------------------------------------------------------------------------
; A0 Drag Acceleration Computation
;
; A0 represents the average drag acceleration expected during the remaining
; trajectory. This value is critical for range prediction accuracy.
;
; The computation accounts for:
; - V1: Effective velocity including altitude rate effects
; - Altitude rate contribution to energy dissipation
; - Current drag level and atmospheric density
;
; A0 is used in subsequent range prediction calculations to determine how
; far the spacecraft will travel before reaching target velocity.
; ----------------------------------------------------------------------------

			RDOT		# Altitude rate
A0CALC		DDV	DAD		# V1 = V + RDOT/TEM1B
			TEM1B		# L/D ratio (LAD or LEWD)
			V		# Current velocity
		STODL	V1		# V1/2 VS - effective velocity

			RDOT
		DSQ	DDV		# A0=(V1/V)SQ(D+RDOT SQ/(TEM1B 2 C1 HS)
			TEM1B		# Square of L/D
		DDV	DAD		# Altitude rate contribution
			2C1HS		# Atmospheric constant
			D		# Current drag
		DMP	DMP		# Scale by velocity ratio squared
			V1
			V1
		DDV
			VSQUARE		# Normalize by velocity squared
		STODL	A0		# A0/805 = A0/25G - average drag level

; ----------------------------------------------------------------------------
; A1 Assignment and V1 Adjustment
;
; If RDOT is positive (climbing), A1 is set equal to A0 for consistency.
; If L/D is negative (lift-down configuration), V1 is reduced by VQUIT (1000 fps)
; to account for the different trajectory characteristics during lift-down flight.
; ----------------------------------------------------------------------------

			RDOT
		BPL	DLOAD		# If climbing (RDOT positive)
			V1LEAD
			A0
		STORE	A1		# A1/25G - set A1 = A0

V1LEAD		DLOAD	BPL		# IF L/D NEG, V1=V1 - 1000
			L/D		# Current lift-to-drag ratio
			HUNTEST1	# If lift-up, skip adjustment

		DLOAD	DSU		# Lift-down case
			V1
# Page 856
			VQUIT		# 1000 fps adjustment
		STORE	V1		# Adjusted V1 for lift-down

; ----------------------------------------------------------------------------
; VL (Lift Velocity) Computation
;
; VL is the target velocity at which the UPCONTRL phase should begin. This
; velocity defines the transition point from range-hunting to lift-up control.
;
; The calculation involves:
; - ALP: Atmospheric density parameter
; - FACT1, FACT2: Intermediate factors for the VL equation
; - Q7: Heat load factor determining when to start lift-up
;
; VL represents the velocity at which continued lift-down would violate
; heating constraints, requiring transition to lift-up for thermal protection.
; ----------------------------------------------------------------------------

HUNTEST1	DLOAD	DMP		# ALP = 2 C1 HS A0/LEWD V1 V1
			A0		# Average drag level
			2C1HS		# Atmospheric constant
		DDV	SETPD
			V1		# Effective velocity
			0
		DDV	DDV
			V1
			LEWD		# Lift-down L/D
		STORE	ALP		# Atmospheric density parameter

		BDSU	BDDV		# FACT1 = V1 / (1 - ALP)
			BARELY1		# Constant: 1.0 - 1 bit
			V1
		STODL	FACT1		# FACT1 / 2VS

			ALP
		DSU	DMP		# FACT2 = ALP(ALP - 1) / A0
			BARELY1		# (ALP - 1)
			ALP
		DDV
			A0		# Normalize by drag level
		STORE	FACT2		# FACT2 (25G)

		DMP	DAD		# VL = FACT1 (1 - SQRT(Q7 FACT2 + ALP))
			Q7		# Q7 / 805 = Q7 / 25G - heat load factor
			ALP		# VL=FACT1 (1-SQRT(Q7 FACT2 +ALP) )
		SQRT	BDSU		# Square root of heat load term
			BARELY1		# (1 - SQRT(...))
		DMP			# Multiply by FACT1
			FACT1
		STORE	VL		# VL / 2 VS - target velocity for UPCONTRL

; ----------------------------------------------------------------------------
; GAMMAL1 Initial Calculation
;
; GAMMAL1 = LEWD * (V1-VL)/VL
;
; This initial value of GAMMAL1 represents the bank angle command that would
; be required if we transitioned directly to UPCONTRL at velocity VL.
; It will be refined later in the HUNTEST logic.
; ----------------------------------------------------------------------------

		BDSU	DMP		# GAMMAL1 = LEWD (V1-VL)/VL
			V1		# Current effective velocity
			LEWD		# Lift-down L/D
		DDV
			VL		# Target lift velocity
		STODL	GAMMAL1		# GAMMAL1 USED IN UPCONTROL

; ----------------------------------------------------------------------------
; VL Validity Check
;
; If VL is less than VLMIN (minimum acceptable velocity for lift transition),
; the trajectory is approaching final phase conditions. Branch to PREFINAL
; to begin final descent control.
; ----------------------------------------------------------------------------

					# GAMMAL1 = PDL 22D.
			VL
		DSU	BMN		# IF VL-VLMIN NEG, GO TO PREFINAL
			VLMIN		# Minimum lift velocity threshold
			PREFINAL	# Branch to final phase

; ----------------------------------------------------------------------------
; VBARS and Range Hunting Setup
;
; VBARS = VL^2 is the reference velocity squared for range calculations.
; DVL represents the velocity increment over which range hunting occurs.
; VS1 is set to either VSAT (saturated velocity) or V1, depending on which
; is smaller, to determine the range hunting interval.
; ----------------------------------------------------------------------------

		DLOAD	DSQ		# Square VL for VBARS
# Page 857
			VL
		STODL	VBARS		# VBARS / 4 VS VS - reference velocity squared

			HALVE		# HALVE = VSAT (saturated velocity)
		DSU	BMN		# IF VSAT-VL NEG, GO TO CONSTD
			VL
			BECONSTD	# SET MODE=HUNTEST, CONTINUE IN CONSTD
		STODL	DVL		# DVL / 2VS - initial velocity increment

			HALVE		# VSAT
		STORE	VS1		# VS1 = VSAT initially

		DSU	BMN		# IF V1 GREATER THAN VSAT, GO ON
			V1		# Compare VSAT with V1
			GETDHOOK	# V1 <= VSAT, use VSAT for VS1
		BDSU			# V1 > VSAT case
			DVL
		STODL	DVL		# DVL = DVL - (VSAT-V1) = V1 - VL
			V1
		STORE	VS1		# VS1 = V1, IN THIS CASE

; ----------------------------------------------------------------------------
; DHOOK Computation
;
; DHOOK is a drag parameter used in the range prediction equations.
; DHOOK = ((1 - VS1/FACT1)^2 - ALP) / FACT2
;
; This parameter characterizes the atmospheric drag effects over the
; range-hunting velocity interval.
;
; AHOOKDV is derived from DHOOK and Q7, used in the GAMMAL calculation.
; ----------------------------------------------------------------------------

GETDHOOK	DLOAD	CALL		# DHOOK=((1-VS1/FACT1) SQ -ALP)/FACT2
			VS1		# VS1 / 2 VS
			DHOOKYQ7	# GO CALC DHOOK - subroutine
		STORE	DHOOK		# DHOOK / 25G

		SR	DDV		# CHOOK calculation
			6		# Shift right 6 bits
			Q7		# Heat load factor
		DSU
			CHOOK		# = .25/16 = (-6)
		STORE	AHOOKDV		# AHOOKDV parameter for GAMMAL

; ----------------------------------------------------------------------------
; GAMMAL Computation for Range Hunting
;
; GAMMAL is the bank angle command computed for range control during HUNTEST.
;
; GAMMAL = GAMMAL1 - (CH1 * DVL^2 * (1+AHOOK*DVL)) / (DHOOK * VBARS)
;
; This equation adjusts the initial GAMMAL1 to account for the predicted
; range error over the velocity increment DVL. The result determines the
; bank angle needed to zero the range error at the target velocity VL.
;
; If GAMMAL is negative (lift-up required), branch to NEGAMA for handling.
; ----------------------------------------------------------------------------

		DAD	DMP		# GAMMAL= GAMMAL1-CH1 DVL SQ(1+AHOOK DVL)
			1/16TH		# Constant offset
			CH1		# Range control coefficient
		DMP	DMP		# Multiply by DVL squared
			DVL
			DVL
		DDV	DDV		# Normalize by DHOOK and VBARS
			DHOOK		# Drag parameter
			VBARS		# Velocity squared reference
		BDSU	BMN		# Subtract from GAMMAL1
			GAMMAL1
			NEGAMA		# If negative, handle lift-up case
HUNTEST3	STORE	GAMMAL		# Store computed bank angle

; ----------------------------------------------------------------------------
; GAMMAL1 Update for Continuity
;
; Update GAMMAL1 with a weighted average:
; GAMMAL1 = GAMMAL1 + Q19 * (GAMMAL - GAMMAL1)
;
; This provides smooth transition between successive HUNTEST cycles,
; preventing abrupt bank angle changes.
; ----------------------------------------------------------------------------

		DSU			# GAMMAL1=GAMMAL1 +Q19 (GAMMAL-GAMMAL1)
			GAMMAL1		# Difference (GAMMAL - GAMMAL1)
		DMP	DAD		# Weighted update
# Page 858
			Q19		# Filter constant
			GAMMAL1		# Add to old GAMMAL1
		STODL	GAMMAL1		# Store updated GAMMAL1
			GAMMAL		# Load GAMMAL for next section

# Page 859
; ============================================================================
; RANGE PREDICTION CALCULATIONS
;
; This section computes the predicted range components for the entry trajectory.
; Three range components are calculated:
;   - ASKEP: Ballistic range (Kepler arc range)
;   - ASP1: Final phase range
;   - ASPUP: Upcontrol phase range
;
; These components are summed to predict total range to target, enabling
; closed-loop range control during entry.
; ============================================================================

# *START	RANGE PREDICTION ...
					# C(MPAC) = GAMMAL

; ----------------------------------------------------------------------------
; COSG and Energy Parameter (E) Computation
;
; COSG = 1 - GAMMAL^2/2 (truncated series approximation of cos(GAMMAL))
; This approximation is valid for small bank angles typical during entry.
;
; E = SQRT(1 + VBARS(VBARS-1)*COSG^2/4 + C1/16)
; E represents the energy parameter used in the ballistic range equation.
; ----------------------------------------------------------------------------

RANGER		DSQ	SR2		# COSG = 1-GAMMAL SQ/2, TRUNCATED SERIES
		BDSU			# 1 - (GAMMAL^2 / 4)
			HALVE		# Constant: 1/2
		STODL	COSG/2		# Store COSG/2
			VBARS		# E=SQRT(1+VBARS........

		DSU	DMP		# (VBARS - 1/2) * VBARS
			HALVE
			VBARS
		DMP	DMP		# * COSG/2 * COSG/2
			COSG/2
			COSG/2
		SL2	DAD		# Scale and add C1/16
			C1/16		# C1/16 = 1/16
		SQRT	PDDL		# E/4 INTO PDL - energy parameter

; ----------------------------------------------------------------------------
; Ballistic Range (ASKEP) Calculation
;
; ASKEP = 2 * ARCSIN(VBARS * COSG * sin(GAMMAL) / E)
;
; This represents the Kepler arc range - the distance traveled during
; ballistic flight phases when lift is minimal. Based on classical
; orbital mechanics, treating the entry as a perturbed elliptical arc.
; ----------------------------------------------------------------------------

			VBARS		# Velocity ratio parameter
		DMP	DMP		# ASKEP/2 = ARCSIN(VBARS COSG SING/E)
			COSG/2		# Cosine approximation
			GAMMAL		# Bank angle (approximates sine)
		DDV	ASIN		# Divide by E, compute arcsine
		SL1	PUSH		# ASKEP TO PDL 0.
		STODL	ASKEP		# BALLISTIC RANGE	ASKEP/2PI

; ----------------------------------------------------------------------------
; Final Phase Range (ASP1) Calculation
;
; ASP1 = Q2 + Q3*VL
;
; This represents the predicted range during the final constant-drag phase
; where lift velocity (VL) effects dominate. Q2 and Q3 are trajectory
; constants derived from entry corridor analysis.
; ----------------------------------------------------------------------------

					# FOR TM, STORE RANGE COMPONENTS OVERLAPPING (SP)
			VL		# Lift velocity parameter
		DMP	DAD		# ASP1 = Q2 + Q3 VL
			Q3		# Final phase coefficient
			Q2		# Final phase constant
		STORE	ASP1		# FINAL PHASE RANGE	ASP1/2 PI

; ----------------------------------------------------------------------------
; Upcontrol Phase Range (ASPUP) Calculation
;
; ASPUP = -C12 * LOG(V1^2 * Q7 / (VBARS * A0)) / GAMMAL1
;
; This represents the range traveled during the upcontrol (lift-up) phase
; of entry. The logarithmic term accounts for exponential energy dissipation
; during atmospheric braking. GAMMAL1 provides the effective bank angle
; during this phase.
; ----------------------------------------------------------------------------

		PDDL	DSQ		# ASP1 TO PDL 2.
			V1		# Effective velocity
					#		    2
					# ASPUP= -C12 LOG(V1 Q7/VBARS A0)/GAMMAL1
		DMP	DDV		# V1^2 * Q7
			Q7		# Trajectory parameter
			VBARS		# / VBARS
		DDV	CALL		# / A0, then compute LOG
			A0		# Average drag
			LOG		# RETURN WITH -LOG IN MPAC

		DMP	DDV		# * C12 / GAMMAL1
			C12		# Range scaling constant
			GAMMAL1		# Filtered bank angle
		STORE	ASPUP		# UP PHASE RANGE	ASPUP / 2 PI

; ----------------------------------------------------------------------------
; Downcontrol Phase Range (ASPDWN) Calculation
;
; ASPDWN = KC3 * RDOT * V / (A0 * LAD)
;
; This represents the range traveled during the pull-out from the downcontrol
; (lift-down) phase. When altitude rate (RDOT) is negative (descending),
; this term accounts for the range covered while reversing vertical motion
; back to level flight.
; ----------------------------------------------------------------------------

# Page 860
		PDDL	DMP		# ASPUP TO PDL 4.
			KC3		# KC3 = -4 VS VS / 2 PI 805 RE
					# ASPDWN = KC3 RDOT V / A0
			RDOT		# Altitude rate
		DMP	DDV		# * V / A0
			V		# Current velocity
			A0		# Average drag
		DDV	PUSH		# ASPDWN TO PDL 6.
			LAD		# Lift-up L/D ratio
		STODL	ASPDWN		# RANGE TO PULL OUT	ASPDWN /2 PI

; ----------------------------------------------------------------------------
; Bank Angle Correction Range (ASP3) Calculation
;
; ASP3 = Q5 * (Q6 - GAMMAL)
;
; This term corrects the range prediction for deviations of the current
; bank angle (GAMMAL) from the nominal trajectory assumption (Q6).
; Provides first-order feedback correction for bank angle errors.
; ----------------------------------------------------------------------------

			Q6		# Nominal bank angle parameter
		DSU	DMP		# ASP3 = Q5(Q6-GAMMAL)
			GAMMAL		# Current commanded bank angle
			Q5		# Sensitivity coefficient
		STOVL	ASP3		# GAMMA CORRECTION	ASP3/2PI

; ----------------------------------------------------------------------------
; Total Range Summation and Error Computation
;
; The total predicted range (ASP) is the sum of all range components:
; ASP = ASKEP + ASP1 + ASPUP + ASPDWN + ASP3
;
; DIFF = ASP - THETAH represents the range error (predicted minus target).
;
; During Apollo 11 entry on July 24, 1969, this range prediction algorithm
; continuously compared predicted landing point to the recovery ship location
; in the Pacific Ocean, adjusting bank angle to null the error and achieve
; precise splashdown targeting.
; ----------------------------------------------------------------------------

			ASKEP		# GET HI-WD AND
		STODL	ASPS(TM)	# SAVE HI-WORD OF ASP'S FOR TM.

			ASP3		# Start with gamma correction
		DAD	DAD		# Add range components from PDL stack
					# ASPDWN FROM PDL 6.
					# ASPUP FROM PDL 4.
		DAD	DAD		# Continue summing
					# ASP1 FROM PDL 2.
					# ASKEP FROM PDL 0.
		DSU	BOVB		# CLEAR OVFIND.
			THETAH		# Subtract target range
			TCDANZIG	# Overflow protection
		STORE	DIFF		# DIFF = (ASP-THETAH) / 2 PI
					# ASP=ASKEP+ASP1+ASPUP+ASP3+ASPDWN = TOTAL RANGE

; ----------------------------------------------------------------------------
; Range Error Convergence Check
;
; If |DIFF| < 25 NM, range error is sufficiently small to transition to
; UPSY (constant drag phase). Otherwise, continue range hunting by adjusting
; the lift-down L/D parameter (LEWD) to drive range error toward zero.
;
; The HIND flag indicates whether the vehicle is in the "high altitude"
; region where LEWD adjustments have different characteristics.
; ----------------------------------------------------------------------------

		ABS	DSU		# IF ABS(THETAH-ASP) -25NM NEG, GO TO UPSY
			25NM		# 25 nautical mile convergence threshold
		BMN	BON		# Branch if converged or if HIND set
			GOTOUPSY	# Exit to UPSY phase (converged)
			HIND		# High altitude indicator flag
			GETLEWD		# Special LEWD update path

; ----------------------------------------------------------------------------
; L/D Ratio Update Logic (Range Error Correction)
;
; The lift-down L/D parameter (LEWD) is adjusted using a secant method to
; drive the range error (DIFF) to zero. The correction increment (DLEWD) is
; computed based on the current and previous range errors:
;
;   DLEWD_new = DLEWD_old * (DIFF / (DIFFOLD - DIFF))
;
; This implements closed-loop range control: if range is short (DIFF < 0),
; LEWD increases to provide more lift-down, extending the range. If range
; is long (DIFF > 0), LEWD decreases.
; ----------------------------------------------------------------------------

		DLOAD	BPL		# Load DIFF, branch on sign
			DIFF		# Current range error
			DCONSTD		# EVENTUALLY SETS MODE = HUNTEST.

GETDLEWD	DLOAD	DMP		# Secant method update
					# DLEWD = DLEWD (DIFF/(DIFFOLD-DIFF))
			DLEWD		# Previous correction increment
			DIFF		# Current error
		PDDL	DSU		# Push to PDL, compute denominator
			DIFFOLD		# Previous error
			DIFF		# Current error
# Page 861
		BDDV			# Divide with underflow protection
LWDSTORE	STADR			# Common return point
		STORE	DLEWD		# Store updated correction
		DAD	BMN		# IF LEWD+DLEWD NEG, DLEWD=-LEWD/2
			LEWD		# Current L/D parameter
			LEWDPTR		# Handle negative result
		BOV			# Check for overflow
			LEWDOVFL	# Overflow handler
		STORE	LEWD		# Update LEWD with correction

; ============================================================================
; SIDETRAK - Restart Priority Management and Iteration Setup
;
; This section manages restart protection priorities during the hunt iteration.
; The restart system ensures that if a power transient occurs, the program can
; recover and continue from a safe state.
;
; The routine:
;   1. Drops restart priority for this iteration (Group 5) below Group 4
;   2. Sets up restart to return to PRE-HUNT phase if interrupted
;   3. Sets GOTOADDR to ADENDEXT to prevent multiple passes overlapping
;   4. Saves current error as DIFFOLD for next iteration
;   5. Restores Q7 to Q7F and returns to HUNTEST
;
; The HIND flag is set to indicate this high-altitude path was taken.
; ============================================================================

SIDETRAK	EXIT			# Exit interpreter mode

		CA	EBENTRY		# Set EBANK for entry variables
		TS	EBANK

		CA	PRIO16		# DROP GRP 5 RESTART PRIO TO 1 LESS THAN
		TS	PHSPRDT5	# GRP 4.

		TC	PHASCHNG	# Change restart phase
		OCT	00474		# RESTART GRP 4 AT PRE-HUNT.
					# FORCE RESTART TO PICK UP IN GRP 4:
					# USE PRIO 17 FOR GRP 4 (< SERVICER PRIO)
		CA	PRIO16		# CONTINUE GRP 5 AT LOWER PRIO THAN EITHER
					# GRP 4 OR SERVICER.
		TC	PRIOCHNG	# Change priority

		CAF	ADENDEXT	# SIDETRACK NEXT PASS UNTIL THIS ONE DONE.
		TS	GOTOADDR	# ONLY AFTER RESTART IS LEFT AFTER DETOUR.

		TC	INTPRET		# Return to interpreter mode

		DLOAD	SET		# Load current error and set flag
			DIFF		# Current range error
			HIND		# High altitude indicator flag
		STODL	DIFFOLD		# DIFFOLD / 2 PI - save for next iteration

			Q7F		# Final Q7 value
		STCALL	Q7		# Q7 / 805 FPSS - restore and call
			HUNTEST		# (GO TO) - return to hunt

; ----------------------------------------------------------------------------
; LEWDOVFL - L/D Overflow Handler
;
; This routine handles arithmetic overflow during LEWD calculation. If the
; computed LEWD exceeds reasonable bounds, it is clamped to NEARONE
; (approximately 1.0, which is 1.0 - 1 bit). This prevents divergence and
; maintains stability in the range control loop.
;
; Control then passes to DCONSTD phase, which will also set MODE = HUNTEST
; for the next iteration.
; ----------------------------------------------------------------------------

LEWDOVFL	DLOAD			# Load overflow clamp value
			NEARONE		# 1.0 - 1 bit
		STCALL	LEWD		# Set LEWD to safe maximum
			DCONSTD		# (GO TO)  ALSO WILL SET MODE = HUNTEST

; ----------------------------------------------------------------------------
; LEWDPTR - Negative LEWD Handler
;
; This routine handles the case where the updated LEWD becomes negative,
; which is physically impossible (lift-down ratio must be positive).
;
; Recovery action:
;   DLEWD = -LEWD/2  (reverse the correction by half)
;
; The shift right (SR1) divides LEWD by 2, then DCOMP negates it.
; This conservative approach prevents oscillation while maintaining a
; positive correction direction. Control passes to LWDSTORE to apply the
; corrected increment.
; ----------------------------------------------------------------------------

LEWDPTR		DLOAD	SR1		# Load LEWD and divide by 2
			LEWD		# Current L/D value
		DCOMP	GOTO		# Negate and branch
			LWDSTORE	# Store corrected increment

# Page 862

; ============================================================================
; NEGAMA - Velocity Loss Calculation (Part of HUNTEST)
;
; This routine computes the velocity loss (DEL VL) during the drag prediction
; interval. It is called from HUNTEST when GAMMAL (drag coefficient) is
; negative, indicating the spacecraft is experiencing significant atmospheric
; drag.
;
; The calculation implements the equation:
;
;   DEL VL = (GAMMAL * VL/3) / (LEWD/3 - DVL * (2/3 + AHOOKDV) * (CH1 * GS/DHOOK * VL))
;
; Where:
;   GAMMAL = drag acceleration coefficient (negative)
;   VL = current velocity
;   LEWD = lift-down L/D ratio
;   DVL = drag derivative parameter
;   AHOOKDV = atmospheric density derivative parameter
;   CH1 = Chapman function coefficient
;   GS = gravitational acceleration at surface
;   DHOOK = density scale height
;
; After computing velocity loss, the routine updates VL and calculates the
; new Q7 parameter for the next prediction interval.
; ============================================================================

# NEGAMA IS PART OF HUNTEST ...
NEGAMA		DMP	DMP		# ENTER WITH GAMMAL IN MPAC
			VL		# Multiply by velocity
			1/3RD		# Multiply by 1/3
		PDDL	DMP		# PUSH GAMMAL VL/3 to PDL stack
			LEWD		# Load lift-down L/D
			1/3RD		# Multiply by 1/3
		PDDL	DAD		# PUSH LEWD/3 to stack
			AHOOKDV		# Load density derivative
			1/24TH		# Add 1/24 (yields 2/3 + AHOOKDV)
		DMP	DMP		# DEL VL = (GAMMAL VL/3)/(LEWD/3-DVL
			DVL		# (2/3 + AHOOKDV)(CH1 GS/DHOOK VL))
			CH1		# Chapman coefficient
		DDV	DDV		# Divide by DHOOK, then by VL
			DHOOK		# Density scale height
			VL		# Current velocity
		BDSU	BDDV		# Subtract from top of stack (LEWD/3)
					# LEWD/3
					# GAMMAL VL /3
		DAD			# Add velocity back
			VL		# Current velocity
		STCALL	VL		# VL/2 VS - store updated velocity

			DHOOKYQ7	# GO CALC Q7
					# Q7=((1-VL/FACT1)SQ - ALP)/FACT2
		STODL	Q7		# Q7 / 25G - store deceleration parameter

			VL
		DSQ			# Square velocity
		STODL	VBARS		# VBARS / 4 VS VS - store velocity squared

			3ZEROS		# Load zero
		GOTO			# SET GAMMAL = 0
			HUNTEST3	# Continue hunt with zero drag

; ----------------------------------------------------------------------------
; DHOOKYQ7 - Subroutine to Calculate DHOOK or Q7
;
; This subroutine computes either the density scale height (DHOOK) or the
; deceleration parameter (Q7), depending on context. Both use the same
; mathematical form:
;
;   Result = ((1 - VL/FACT1)² - ALP) / FACT2
;
; Where:
;   VL = current velocity (in MPAC on entry, shifted right by 1)
;   FACT1 = velocity scaling factor
;   HALVE = 0.5 (for (1 - VL/FACT1) calculation)
;   ALP = atmospheric model parameter
;   FACT2 = result scaling factor
;
; The computation is:
;   1. Divide VL by 2*FACT1
;   2. Subtract from 0.5 and multiply by 2 to get (1 - VL/FACT1)
;   3. Square the result
;   4. Subtract ALP
;   5. Divide by FACT2
;
; This is a core atmospheric model calculation used throughout the entry
; guidance algorithm.
; ----------------------------------------------------------------------------

DHOOKYQ7	SR1	DDV		# SUBROUTINE TO CALC DHOOK OR Q7)
			FACT1		# Divide by 2*FACT1
		BDSU	SL1		# Subtract from 0.5, multiply by 2
			HALVE		# 0.5 constant
		DSQ	DSU		# Square and subtract ALP
			ALP		# Atmospheric parameter
		DDV	RVQ		# Divide by FACT2 and return
			FACT2		# Scaling factor

# Page 863

; ============================================================================
; PRE-HUNT - Restart Recovery for HUNTEST
;
; This restart handler is invoked if a power transient occurs during HUNTEST
; execution, specifically after the program has been side-tracked at SIDETRAK.
; The restart system picks up execution here in Group 4.
;
; The routine:
;   1. Clears the HIND (high-altitude) flag
;   2. Calls FOREHUNT to re-initialize HUNTEST parameters
;   3. Returns to HUNTEST to continue the hunt iteration
;
; This ensures that even if the computer experiences a power glitch during
; the critical entry phase, the guidance algorithm can recover and continue
; safely from a known state.
; ============================================================================

					# COME TO PRE-HUNT WHEN RESTART OCCURS AFTER
					# HUNTEST IS SIDE-TRACKED AT SIDETRAK.
					# PICK UP IN GROUP 4.

PRE-HUNT	TC	INTPRET		# Enter interpreter mode
		CLEAR	CALL		# Clear flag and call initialization
			HIND		# HIND	99D BIT 6 FLAG 6 - high altitude flag
			FOREHUNT	# RE-INITIALIZE HUNTEST AFTER RE-START.
		GOTO			# Continue to main hunt loop
			HUNTEST		# Resume hunt iteration

; ----------------------------------------------------------------------------
; FOREHUNT - HUNTEST Initialization Subroutine
;
; This routine initializes (or re-initializes after restart) the HUNTEST
; iteration parameters:
;
;   DIFFOLD = 0    (previous range error, zero for first iteration)
;   DLEWD = DLEWD0 (initial L/D increment)
;
; These values set up the first iteration of the Newton-Raphson hunt for
; the optimal L/D ratio to achieve the target range.
; ----------------------------------------------------------------------------

FOREHUNT	DLOAD			# INITIALIZE HUNTEST.
			3ZEROS		# Load zero constant
		STODL	DIFFOLD		# Clear previous error
			DLEWD0		# Load initial L/D increment
		STODL	DLEWD		# Store initial L/D increment
			LEWD1		# Load initial L/D value
		STORE	LEWD		# Store initial L/D
		RVQ			# Return to caller

ADENDEXT	CADR	ENDEXIT		# Address for ENDEXIT routine

# Page 864

; ============================================================================
; TRANSITION: From HUNTEST (Range Prediction) to UPCONTRL (Upward Control)
;
; At this point in the entry trajectory, the Command Module has completed
; the initial hunt for optimal L/D ratio and has established the entry
; corridor parameters. The guidance now transitions from range prediction
; to active lift vector control.
;
; UPCONTRL (Upward Control) manages the entry trajectory when the spacecraft
; is flying with lift vector pointed upward to increase range. This phase
; prevents the spacecraft from descending too rapidly and exceeding g-load
; or heating limits.
;
; The transition occurs through P65 display program, which shows the crew
; the current guidance state and allows manual intervention if needed.
; ============================================================================

# * START	UP CONTROL ...
					# MM = 65
GOTOUPSY	RTB			# END OF HUNTEST
			P65		# HUNTEST USE OF GRP4 IS DISABLED BY P65
					# USE FOR DISPLAY.
					# SET MODE = UPCONTRL.
					# RETURN FROM P65 DIRECTLY TO UPCONTRL
					# VIA THE GOTOADDR AT REFAZE10.

; ----------------------------------------------------------------------------
; UPCONTRL - Upward Control Guidance Phase
;
; This phase controls the entry when lift vector is pointed upward (L/D > 0).
; The guidance logic:
;
;   1. If altitude rate (D) exceeds threshold C21 (140 kft?), set NOSWITCH
;      flag to suppress lateral (crossrange) guidance switching
;
;   2. If velocity (V) exceeds V1 threshold, transition to DOWNCNTL
;      (downward control phase)
;
;   3. If altitude rate (D) is less than Q7 parameter, go to KEP
;      (constant drag phase)
;
; This multi-threshold logic ensures smooth transitions between entry phases
; while maintaining safe flight within the entry corridor.
; ----------------------------------------------------------------------------

UPCONTRL	DLOAD	DSU		# IF D-140 POS, NOSWITCH =1
			D		# (SUPPRESS LATERAL SWITCH)
			C21		# C21 = threshold altitude rate (~140 kft)
		BMN	SET		# Branch if negative, otherwise set flag
			+2		# Skip flag setting
			NOSWITCH	# Suppress lateral switching flag

		DLOAD	DSU		# IF V-V1 POS, GO TO DOWN CONTROL.
			V		# Current velocity
			V1		# Velocity threshold for mode transition
		BPL	DLOAD		# Branch if positive (V > V1)
			DOWNCNTL	# Go to downward control phase
			D		# Reload altitude rate
		DSU	BMN		# IF D- Q7 NEG, GO TO KEP
			Q7		# Deceleration parameter threshold
			KEP		# Transition to Kepler (ballistic) phase
		DLOAD	BPL		# IF RDOT NEG, DO VLTEST
			RDOT		# Altitude rate (vertical velocity)
			CONT1		# If positive, skip velocity loss test

; During the early part of the upward control phase, if the altitude rate
; (RDOT) is negative (descending), the guidance performs a velocity loss test
; to determine if the spacecraft has decelerated sufficiently to transition
; to the final entry phase.

VLTEST		DLOAD	DSU		# IF V-VL-C18 NEG,EGSW=1,MODE=PREDICT3
			V		# Current velocity
			VL		# Target velocity at current range
		DSU	BMN		# Subtract threshold margin C18
			C18		# Velocity margin for final phase
			PREFINAL	# Transition to final phase if within margin

; If altitude rate is positive (climbing), the guidance computes the L/D
; command to manage the trajectory. For very high altitude rates (D > A0),
; the guidance simply uses maximum L/D (LAD) to pull the spacecraft up.
; For moderate altitude rates, a more refined calculation is performed.

CONT1		DLOAD			# IF D-A0 POS, L/D = LAD, GO TO LIMITL/D
			D		# Current altitude rate
		DSU	BMN		# Subtract threshold A0
			A0		# High altitude rate threshold
			CONT3		# Use refined calculation
		DLOAD	GOTO		# Use maximum L/D
			LAD		# Load design L/D value
			STOREL/D	# Store and continue to limit checks

; For moderate altitude rates, compute a reference velocity VREF that
; represents the desired velocity profile. This uses a nonlinear function
; involving square root to provide smooth trajectory shaping.
; VREF = FACT1 * (1 - sqrt(FACT2 * D + ALP))

CONT3		DLOAD	DMP		# VREF=FACT1(1-SQRT(FACT2 D + ALP))
			D		# Current altitude rate
			FACT2		# Scaling factor for D
# Page 865
		DAD	SQRT		# Add ALP and take square root
			ALP		# Offset parameter
		BDSU	DMP		# Compute (1 - sqrt(...))
			BARELY1		# Constant approximately 1.0
			FACT1		# Multiply by scaling factor
		STORE	VREF		# Store reference velocity (VREF / 2VS)

; The reference velocity VREF represents the velocity excess that needs to
; be dissipated. The guidance now computes a reference altitude rate RDOTREF
; that will achieve the desired velocity profile by multiplying the velocity
; error (V1 - VREF) by the current L/D ratio.

		BDSU	DMP		# RDOTREF = LEWD(V1-VREF)
			V1		# Target velocity parameter
			LEWD		# Current lift-to-drag ratio
		STODL	RDOTREF		# Store reference altitude rate (RDOTREF / 2VS)

; The guidance checks if VSAT (saturation velocity) exceeds VREF. If VREF
; is greater than VSAT, the trajectory is saturated and special handling
; is needed. This prevents the guidance from commanding unrealistic
; deceleration rates.

			VS1		# Load saturation velocity VSAT
		DSU	BMN		# IF VSAT-VREF NEG, GO TO CONTINU2
			VREF		# Subtract reference velocity
			CONTINU2	# Handle saturated case

; For non-saturated case, compute a correction to RDOTREF based on the
; difference between VSAT and VREF. This provides smoother trajectory
; shaping when approaching velocity saturation.

		PUSH	PUSH		# VS1-VREF  TO PDL TWICE (save for calculation)

; Compute RDHOOK, a correction term to RDOTREF that handles the transition
; to velocity saturation smoothly. The formula is:
; RDHOOK = CHI * (1 + DV * AHOOKDV/DVL) * DV * DV / (DHOOK * VREF)
; where DV = (VS1-VREF), the velocity excess above reference.

		DMP	DDV		# RDHOOK=CHI(1+DV AHOOKDV/DVL) DV DV
			AHOOKDV		#	/DHOOK VREF
			DVL		# WHERE  DV = (VS1-VREF)
		DAD	DMP		# Add 1/16 and multiply by CHI
			1/16TH		# Constant (represents 1.0 scaled)
			CH1		# CHI parameter (hook gain)
		DMP	DMP		# Multiply by (VS1-VREF) twice
					# VS1-VREF  FROM PDL TWICE.
		DDV			# Divide by DHOOK
			DHOOK		# Hook parameter
		DDV	BDSU		# Divide by VREF, then subtract from RDOTREF
			VREF		# Reference velocity
			RDOTREF		# C(RDOTREF)= LEWD (V1-VREF)
		STORE	RDOTREF		# RDOTREF = RDOTREF - RDHOOK

; The CONTINU2 entry point is used when VSAT-VREF is negative (saturated).
; Now compute the actual altitude rate error (D - RDOTREF) and determine
; the required L/D command to correct this error.

CONTINU2	DLOAD	DSU		# Check if D is below Q7MIN threshold
			D		# Current altitude rate
			Q7MIN		# Minimum Q7 threshold
		BOVB	BMN		# Branch on minus if D < Q7MIN
			TCDANZIG	# CLEAR OVFL IND, IF ON.
			UPCNTRL3	# Skip Q7 update logic

; When altitude rate is high enough (D >= Q7MIN), update the Q7 gain
; parameter based on the difference between A1 and the current Q7 value.
; This provides adaptive gain adjustment as the trajectory evolves.

		DLOAD	DSU		# Compute A1 - Q7
			A1		# Gain parameter A1
			Q7		# Current Q7 gain
		PDDL	DSU		# Push (A1-Q7), then compute D-Q7
			D		# Current altitude rate
			Q7		# Current Q7 gain
		DDV	STADR		# Divide: FACTOR = (A1-Q7)/(D-Q7)
		STORE	FACTOR		# Store updated gain factor (FACTOR / 25G)

; The FACTOR parameter represents the adaptive gain for the altitude rate
; feedback loop. It adjusts automatically based on the trajectory state.

# Page 866
# SKIPPER
; The UPCNTRL3 section computes the final L/D command using a two-term
; feedback law that combines altitude rate error and velocity error:
; DELTA L/D = -((RDOT-RDOTREF)*F1/KB1 + (V-VREF))*F1/KB2
; where F1 = FACTOR (adaptive gain).

UPCNTRL3	DLOAD			# Load current altitude rate
			RDOT		# RDOT (scaled altitude rate)
		DSU	DMP		# Compute altitude rate error and scale by FACTOR
			RDOTREF		# Reference altitude rate
			FACTOR		# Adaptive gain factor
		DDV	DAD		# Divide by KB1, add current velocity
			1/KB1		# Gain constant KB1
			V		# Current velocity
		DSU	DMP		# Subtract VREF, multiply by FACTOR
			VREF		# Reference velocity
			FACTOR		# Adaptive gain factor
		DDV	PUSH		# Divide by -KB2, push DELTA L/D to PDL

			-1/KB2		# DELTA L/D INTO PDL (negative gain KB2)

; Nonlinear limiting circuit prevents excessive L/D commands during high
; error conditions. For large DELTA L/D values, apply compression to reduce
; command magnitude and prevent control saturation.

		BOV	ABS		# NONLINEAR CIRCUIT FOR REDUCING HIGH GAIN
			GOMAXL/D	# On overflow, use maximum L/D
		DSU	BMN		# Check if |DELTA L/D| < PT1/16
			PT1/16		# Threshold for nonlinear limiting
			NEXT1		# Small error, no limiting needed
		DMP	DAD		# Apply compression: 0.1*excess + PT1/16
			POINT1		# Compression factor 0.1
			PT1/16		# Threshold offset
		SIGN	PUSH		# Attach original sign, push compressed value

; For small DELTA L/D (below threshold), proceed with linear control law.

NEXT1		DLOAD	SL4		# Scale DELTA L/D by 16
					# DELTA L/D FROM PDL.

		DAD			# Add base L/D command (LEWD)
			LEWD		# Base lift-to-drag ratio command

; The NEGTESTS section implements special logic to handle negative L/D
; commands during entry. When altitude rate is positive (D-C20 > 0),
; the vehicle is climbing, so LATSW is cleared to enforce roll-over-top
; regardless of bank angle. If L/D is negative in this case, it is set
; to zero to prevent unphysical commands.

NEGTESTS	BOV	PUSH		# L/D TO PDL FOR USE IN NEGTESTS.
			GOMAXL/D	# On overflow, use maximum L/D
		STODL	L/D		# Store computed L/D command
					# IF D-C20 POS, LATSW =0
					# AND IF L/D NEG, L/D = 0.
			D		# Current altitude rate
		DSU	BMN		# Check if D < C20 (descending)
			C20		# Altitude rate threshold
			LIMITL/D	# Descending case, skip LATSW clear
		CLEAR	DLOAD		# Climbing case: clear LATSW
			LATSW		# =21D.  ROLL OVER TOP, REGARDLESS.
					# L/D FROM PDL.
		BPL	DLOAD		# If L/D positive, keep it
			LIMITL/D	# Skip zeroing
			3ZEROS		# L/D negative: set to zero
		STCALL	L/D		# Store final L/D command
			LIMITL/D	# (GO TO) Proceed to limiting/saturation logic

# Page 867
; ============================================================================
; TRANSITION: From UPCONTRL to constant-drag guidance
;
; The CONSTD section implements constant-drag (constant deceleration)
; guidance used during certain entry phases. Multiple entry points exist
; for different prior phases (RANGER, HUNTEST). This phase maintains a
; constant drag level to provide predictable deceleration.
; ============================================================================

DCONSTD		DLOAD			# TWO RANGER ENTRIES TO CONSTD HERE
			DIFF		# Range difference from previous pass
					# SAVE OLD VALUE OF DIFF FOR NEXT PASS.
		STODL	DIFFOLD		# DIFFOLD / 2 PI (save for prediction)

			Q7F		# Reset Q7 to initial value
		STORE	Q7		# Q7 = Q7F (initial gain parameter)

BECONSTD	SSP	RTB		# A HUNTEST ENTRY INTO CONSTD.
			GOTOADDR	# RESET MODE TO HUNTEST
			HUNTEST		# Set return mode to HUNTEST
			KILLGRP4	# DEACTIVATE GRP4 FROM HUNTEST.

; The CONSTD guidance law computes L/D to maintain constant drag coefficient
; by balancing altitude rate feedback with drag error feedback. Used when
; constant deceleration is desired to manage energy and range predictably.

CONSTD		BOVB			# Main constant-drag guidance entry
			TCDANZIG	# CLEAR OVF IND IF ON.

; Compute reference altitude rate: RDOTREF = -2*HS*D0/V
; This sets desired vertical velocity for constant-drag flight.
		DLOAD	DMP		# LEQ * C/D0
			LEQ		# Equilibrium L/D
			C/D0		# C/D0 = -4/D0 B-8 (drag coefficient constant)
		PDDL	DMP		# LEQ C/D0 INTO PDL (first term of L/D command)
			2HS		# 2HS / 4 VS VS (scale height parameter)
			D0		# D0 = reference drag level (805 ft/sec²)
		DDV	DAD		# RDOTREF = -2 HS D0/V (divide by velocity)
			V		# Current velocity
			RDOT		# Add to current altitude rate
		DMP	DAD		# K2D * (RDOT - RDOTREF) (altitude rate error feedback)
			K2D		# Drag rate gain (typ. 0.02)
					# C/D0 LEQ + K2D(RDOT-RDOTREF) INTO PD
		PDDL			# Push sum to stack
			D0		# D0 /805 (reference drag, will be DREF)

; CONSTD1 is common exit point computing final L/D from drag error.
; L/D = K1D*(DREF - D) + [K2D term from stack]

CONSTD1		BDSU			# ENTER WITH DREF IN MPAC
			D		# Current drag level (ft/sec²)
		DMP	DAD		# K1D * (DREF - D) (drag error feedback)
			K1D		# Drag error gain (typ. 1.0)
					# K2D TERM FROM PUSH
		SL	GOTO		# Scale by 256
			8D		# Shift left 8 bits
			NEGTESTS	# (GO TO) Apply limiting and sign tests

; ============================================================================
; TRANSITION: From constant-drag to downward control
;
; DOWNCNTL implements the initial descent phase of entry guidance, typically
; used at high velocity when descending from high altitude. Computes a
; reference drag profile based on velocity error and desired L/D, ensuring
; proper energy management for the upcoming equilibrium glide phase.
; ============================================================================

DOWNCNTL	BOVB			# INITIAL PART OF UPCONTROL.
			TCDANZIG	# CLEAR OVFIND, IF ON.

; Compute reference altitude rate: RDTR = LAD * (V1 - V)
; This ties vertical velocity to velocity error, guiding toward V1 target.
		DLOAD	SR		# LAD (desired L/D) scaled
			LAD		# Target lift-to-drag ratio
			8D		# Scale factor
		PDDL	DSU		# RDTR = LAD(V1-V) computation starts
			V		# Current velocity
			V1		# Target velocity for equilibrium glide
		DMP	DAD		# (V1 - V) * LAD
			LAD		# Multiply by LAD
# Page 868
			RDOT		# Add current altitude rate
		DMP	DAD		# K2D * (RDOT - RDTR) (altitude rate error term)
			K2D		# Altitude rate gain
					# PUSH UP LAD.
		PDDL	DSU		# LAD + K2D(RDOT-RDTR) INTO PD (saved for later)
			V1		# Target velocity
			V		# Current velocity
		DSQ	DMP		# (V1 - V)² * LAD (velocity error squared term)
			LAD		# Multiply by LAD
		DDV	PDDL		# (V1-V)² LAD / (2 C1 HS) INTO PD
			2C1HS		# 2 * C1 * HS (scale constant)
			V1		# Target velocity
		DSQ	DDV		# V1² / V² (velocity ratio squared)
			VSQUARE		# Current velocity squared
		BDDV	DSU		# DREF = (V/V1)² A0 - [(V1-V)² LAD / (2 C1 HS)]
			A0		# Initial drag parameter
					# PUSH UP HERE (subtract velocity error term)
		GOTO			# C(MPAC) = DREF (reference drag for CONSTD1)
			CONSTD1		# Use common drag-error feedback path

					#              2           2
					# DREF = (V/V1)  A0 -(V-V1)  LAD/2 C1 HS
; This DREF formulation provides velocity-squared drag scheduling with
; a correction term based on velocity error, guiding the vehicle to the
; target velocity V1 while managing drag profile for range control.
# Page 869
; ============================================================================
; TRANSITION: From active guidance to ballistic (Kepler) phase
;
; The KEP (Kepler) ballistic phase is entered when the vehicle reaches very
; low deceleration levels where aerodynamic control is minimal. The vehicle
; flies a ballistic trajectory with minimal roll commands, maintaining trim
; attitude until drag increases sufficiently to enter the final phase.
; During Apollo 11's reentry, this phase ensured proper attitude control
; during the shallow portion of the trajectory.
; ============================================================================

# * START	BALLISTIC PHASE ...
					# MM = 66	UPCONTRL ENTRY INTO KEP2.
KEP		RTB	SSP		# Enter Kepler ballistic phase
			P66		# DISPLAY TRIM GIMBAL ANGLE VALUES.
			GOTOADDR	# SET GOTOADDR TO KEPLER PHASE.
			KEP2		# Set restart address to KEP2

; KEP2 is the main ballistic phase loop. The vehicle flies essentially
; ballistic with minimal aerodynamic forces. Roll commands are zeroed at
; very low drag levels (.05G threshold) to maintain trim attitude.
					# KEP2 CAN ALSO BE STARTED UP DIRECTLY FROM INITROLL
					# IN P64.  PROGRAM WILL IDLE IN P64 UNTIL D EXCEEDS
					# .2 G BEFORE GOING ON TO P67.

KEP2		DLOAD	DSU		# IF Q7F+KDMIN -D NEG, GO TO FINAL PHASE.
			Q7FKDMIN	# (Q7F + KDMIN)/805 (final phase entry threshold)
			D		# Current drag level
		BMN	TLOAD		# If drag exceeds threshold, enter final phase
			PREFINAL	# Go to final phase setup
					# SET ROLLHOLD = ROLLC, IN CASE CMDAPMOD
			ROLLC		# = +1 EVER ENTERED. (current roll command)
		BON	TLOAD		# IF D > .05G, KEEP PRESENT ROLL COMMAND.
			.05GSW		# .05G switch flag (21D bit3)
			+2		# Skip zeroing if above .05G
			3ZEROS		# SET ROLLC & ROLLHOLD =0. (very low drag)
+2		STCALL	ROLLC		# (SP ROLLHOLD FOLLOWS DP ROLLC)
			P62.3		# CALC DESIRED GIMBAL ANGLES AT PRESENT
					# RN, VN TO YIELD TRIM ATTITUDE.
					# AVAILABLE IN CPHI'S FOR N22.

# Page 870
; ============================================================================
; TRANSITION: From ballistic phase to final phase
;
; PREFINAL initiates the final entry phase (P67) when drag increases
; sufficiently. This is the terminal guidance phase leading to parachute
; deployment altitude. The vehicle uses final range control to guide
; precisely to the target splashdown point.
; ============================================================================

# START FINAL PHASE ...
					# MM = 67
PREFINAL	SSP	RTB		# Setup for final phase
			GOTOADDR	# RESTART PROTECT: RESET GOTOADDR IF CAME
			PREFINAL	# FROM HUNTEST. (set restart address)
			P67		# DISABLES GRP4.  FINE IF FROM HUNTEST.BUT
					# MAY ALSO REMOVE RESTART PROTECTION OF
					# N69 (P65). (display program)
					# ROLLC		XRNGERR		DNRNGERR
					# XXX.XX DEG	XXXX.X NM	XXXX.X NM

		SET	SSP
			EGSW
			GOTOADDR
			PREDICT3

; ============================================================================
; PREDICT3: Final Phase Guidance Computation (P67)
;
; This is the final phase of Apollo 11's reentry guidance, executing from
; high velocities down to subsonic speeds. The routine performs table-driven
; trajectory predictions using the VREFER reference table, which contains
; 13 velocity points (from 36,000 fps down to 1,000 fps) and 6 associated
; trajectory functions. Linear interpolation provides predicted values for
; range derivatives and accelerations based on current velocity.
;
; The guidance law computes bank angle commands (ROLLC) to null range error
; (XRNGERR) and downrange error (DNRNGERR). Steering continues until
; velocity drops below VQUIT (approximately 1000 fps), at which point
; Apollo 11 enters unguided ballistic descent to splashdown.
;
; Apollo 11 Context: This phase executed from approximately 400,000 feet
; altitude through final descent, steering the Command Module Columbia to
; its splashdown point in the Pacific Ocean on July 24, 1969. The precision
; of this guidance achieved landing within visual range of recovery ships.
; ============================================================================

; Velocity check: If current velocity has dropped below VQUIT (~1000 fps),
; reentry guidance is complete and steering commands cease. At this point
; Apollo 11 continues ballistic descent to splashdown with no further
; bank angle modulation.

PREDICT3	DLOAD	DSU		# IF V-VQUIT NEG, STOP STEERING
			V		# Current inertial velocity
			VQUIT		# Cutoff velocity (scaled 51532.39 fps)
		BMN	EXIT		# If V < VQUIT, terminate guidance
			STEEROFF	# Branch to steering shutdown

; Table search initialization: The VREFER table contains 13 velocity entries
; in descending order (highest velocity at table end). Search backward from
; highest velocity to bracket current velocity between two table entries.

		CA	EBENTRY		# PRECAUTIONARY.
		TS	EBANK		# Set extended memory bank

		CA	TWELVE		# Start at table entry 12 (highest V)
BACK		TS	JJ		# JJ = table index counter

; Binary search loop: Decrement through table until VREF < V, identifying
; bracketing entries for interpolation.

		CS	V		# -V (complement of velocity)
		INDEX	JJ		# Indexed addressing
		AD	VREFER		# VREF - V, HIGHEST VREF AT END OF TABLE.
		CCS	A		# IF VREF-V POS LOOP BACK
		CCS	JJ		# DECREMENT JJ, JJ CANNOT BE ZERO
		TCF	BACK		# Continue search at next lower table entry

; Interpolation fraction computation: Now that bracketing table entries are
; identified (at index JJ and JJ+1), compute the fractional position of
; current velocity within this interval. GRAD represents the interpolation
; weight: GRAD=0 means use entry JJ, GRAD=1 means use entry JJ+1.

		AD	ONE		# A now contains V-VREF (positive)
		TS	TEM1B		# V-VREF IN TEM1B (MUST BE POSITIVE NUM)

		INDEX	JJ		# Access VREFER[JJ]
		CS	VREFER		# -VREF(K)
		INDEX	JJ		# Access VREFER[JJ+1]
		AD	VREFER +1	# V(K+1) - V(K)			(POS NUM)
		XCH	TEM1B		# Swap: A now V-VREF, TEM1B=delta V
		ZL			# Clear L register for division
		EXTEND			# Extended instruction follows
		DV	TEM1B		# Divide (V-VREF) by (VK+1-VK)
		TS	GRAD		# GRAD = (V-VREF)/(VK+1 - VK)	(POS NUM

; Interpolate all 6 trajectory functions: Using computed GRAD factor,
; interpolate between table entries for all functions (F0-F5):
; F0: DREF (downrange reference), F1: drange/dalpha, F2: drange/drdot,
; F3: RDOTREF (range-rate reference), F4: dalpha/dV, F5: RTOGO.
; Loop processes each function using index MM (5 down to 0).

		CAF	FIVE		# Initialize function counter

# Page 871
; Interpolation loop: For each function (MM=5 down to 0), compute interpolated
; value using: FX[MM] = VREFER[JJ,MM] + GRAD * (VREFER[JJ+1,MM] - VREFER[JJ,MM])
; The VREFER table is organized with 13 rows (velocity points) and 6 columns
; (trajectory functions F0-F5). After this loop, FX array contains all
; interpolated values at current velocity for use in guidance calculations.

BACK2		TS	MM		# MM = function index (5,4,3,2,1,0)
		CAF	THIRTEEN	# Offset to next table row
		ADS	JJ		# Advance JJ to next function column
		INDEX	A		# Indexed by row offset
		CS	VREFER		# -X(K) from table
		INDEX	JJ		# Access JJ+1 entry
		AD	VREFER	 +1	# X(K+1) - X(K) = delta for this function
		EXTEND
		MP	GRAD		# GRAD * delta
		INDEX	JJ		# Access JJ entry
		AD	VREFER		# Add base value X(K)
		INDEX	MM		# Store in FX array
		TS	FX		# FX = AK + GRAD (AK+1 - AK)
		CCS	MM		# Decrement and test function counter
		TCF	BACK2		# Loop back for next function

; Final prediction angle calculation: Compute PREDANG using interpolated
; trajectory functions. PREDANG represents the predicted bank angle needed
; to null range errors. The calculation combines:
; - Downrange error: F1 * (D - DREF) where D=current range, DREF=reference
; - Range-rate error: F2 * (RDOTREF - RDOT) scaled by 8 for this phase
; - Remaining range: F4 (RTOGO) added to error terms
; Result is the predicted angle correction needed for precision targeting.

		XCH	FX 	+1	# ZERO FX +1 AND GET DREFR
		AD	D		# D - DREF (downrange error)
		EXTEND
		MP	FX	+5	# F1 (drange/dalpha sensitivity)
		DXCH	MPAC		# MPAC = F1(D-DREF)

		EXTEND
		DCS	RDOT		# FORM RDOTREF - RDOT
		DDOUBL			# Scale up by 8 for final phase
		DDOUBL			# (compensates for velocity regime)
		DDOUBL			# SCALE UP BY 8 FOR THIS PHASE.
		AD	FX 	+3	# RDOTREF (reference range-rate)
		EXTEND
		MP	FX 	+4	# F2 (drange/drdot sensitivity)
		AD	FX	+2	# RTOGO (range to go)
		DAS	MPAC		# ADD F2(DADV1-DADVR)
		CA	MPAC		# Retrieve result
		TS	PREDANG		# Store predicted angle
					# L/D = LOD + (THETA- PREDANG)/ Y
; Return to interpretive mode for final L/D computation and limit checking.

		TC	INTPRET

; Downrange error computation: Calculate DNRNGERR = (PREDANG - THETA) / 360
; which represents the angular error in predicted vs actual trajectory.
; Special handling for GONEPAST and GONEBY flags ensures proper behavior when
; passing over or going beyond target point during Apollo 11's final descent.

		SR3	DSU		# Shift right 3, subtract
			THETAH		# Current heading angle
		BON	BOFF		# Branch on flag combinations
			GONEPAST	# If already past target
			GONEGLAD	# Use maximum negative L/D
			GONEBY		# Check if going by target
			HAVDNRNG	# Normal calculation path
		DLOAD	SET		# SET GONEPAST IF GONEBY SET & LATCH IN-PLACE
			MAXRNG		# DISPLAY = 9999.9 IF GONEBY
			GONEPAST	# Set flag to indicate past target
		STCALL	DNRNGERR	# Store error, proceed to limit
			GONEGLAD	# Maximum negative L/D handling

HAVDNRNG	STORE	DNRNGERR	# = (PREDANG - THETA) /360
# Page 872
; Lift-to-drag ratio calculation: Compute L/D = LOD + (THETA - PREDANG) / Y
; where Y = FX (drange/dL/D sensitivity). This is the core steering law that
; modulates bank angle to null range errors. Overflow protection ensures L/D
; stays within vehicle capability limits (approximately -0.5 to +0.5).

		DCOMP			# FALL SHORT IF NEG, OVERSHOOT IF POS
		BOVB	DDV		# Branch on overflow, divide
			TCDANZIG	# CLEAR OVFIND IF ON.
			FX		# FX= DRANGE/D L/D = Y
		SL	BOV		# Shift left 5 bits, check overflow
			5		# Amplify sensitivity
			GOMAXL/D	# If overflow, use maximum L/D
		DAD	BOV		# Add to nominal L/D
			LOD		# Base lift-to-drag ratio
			GOMAXL/D	# If overflow, use maximum L/D
		STCALL	L/D		# Store final L/D command
			GLIMITER	# (GO TO) Apply G-load limits

# GONEGLAD AND GOPOSMAX ENTRY POINTS FOR GLIMITER ...

; Special entry points for maximum L/D conditions:
;
; GONEGLAD: Used when spacecraft has gone past target point. Sets L/D to
; maximum negative value (-LAD) to generate maximum lift-down, attempting
; to bring the vehicle back toward the target. Apollo 11 context: If Columbia
; had overshot the intended splashdown point, this would command maximum
; downward lift to extend range and return toward the Pacific recovery area.
;
; GOMAXL/D: Used when computed L/D exceeds vehicle capability limits. Sets
; L/D = ±LAD (maximum lift-to-drag ratio) with sign determined by current
; MPAC value, ensuring commanded lift stays within CM aerodynamic limits.

GONEGLAD	DLOAD			# SET L/D = -LAD
			GONEGLAD	# (ANY NEGATIVE NUMBER WILL DO)

GOMAXL/D	RTB	DMP		# L/D = LAD SIGN(MPAC)
			SIGNMPAC	# Extract sign from MPAC
			LAD		# Maximum L/D magnitude
		STORE	L/D		# AND FALL INTO GLIMITER SECTION

; ============================================================================
; GLIMITER: G-Load Limiting Logic
;
; This section ensures commanded L/D does not generate excessive g-loads on
; the crew. The algorithm checks current deceleration (D) against maximum
; allowable g-load (GMAX), computing a velocity-dependent limit (XLIM) that
; constrains lift modulation. The equations implement:
;
; If (GMAX/2 - D) > 0: Continue to LIMITL/D (moderate g regime)
; If (GMAX - D) < 0: Set L/D = +LAD (high g regime, use positive lift)
; Otherwise: Compute XLIM = SQRT[2HS(GMAX-D)(LEQ/GMAX+LAD) + (2HSGMAX/V)²]
;            Check if RDOT + XLIM > 0 to determine if L/D limit applies
;
; Apollo 11 Context: During reentry, Columbia experienced peak deceleration
; near 6.5 g's. This limiter ensured Armstrong, Aldrin, and Collins were not
; subjected to g-loads exceeding the spacecraft's 7.5g structural limit or
; crew tolerance limits. The velocity-dependent XLIM calculation accounts for
; the rapidly changing flight regime during atmospheric entry.
; ============================================================================

GLIMITER	DLOAD	DSU		# IF GMAX/2-D POS, GO TO LIMITL/D
			GMAX/2		# Half of maximum g-load limit
			D		# Current deceleration
		BPL	DAD		# IF GMAX  -D NEG, GO TO GOPOSLAD
			LIMITL/D	# Branch to L/D limiting logic
			GMAX/2		# Add back to check full GMAX
		BMN	DMP		# Branch if (GMAX - D) negative
			GOPOSLAD	# High g-load: use positive LAD
			2HS		# 2 * h-dot scale factor
		PDDL	DMP		# 2HS(GMAX-D) INTO PD
			LEQ		# Equilibrium glide L/D
			1/GMAX		# Inverse of max g-load
		DAD	DMP		# Add LAD and multiply
			LAD		# Maximum L/D
		PDDL	DDV		# 2HS(GMAX-D) (LEQ/GMAX+LAD) INTO PD
			2HSGMXSQ	# (2 * h-dot * GMAX)²
			VSQUARE		# V² (current velocity squared)
		DAD	SQRT		# XLIM = SQRT(PD+(2HSGMAX/V)SQ)
		DAD	BPL		# IF RDOT+XLIM POS, GO TO LIMITL/D
			RDOT		# Range rate (radial velocity)
			LIMITL/D	# Apply L/D limit

; GOPOSLAD: High G-Load Override
;
; Reached when deceleration exceeds maximum allowable (GMAX - D < 0). Sets
; L/D to maximum positive value (+LAD), generating maximum lift-up to reduce
; deceleration and bring g-loads back within limits. This is a safety override
; that prioritizes crew and vehicle protection over trajectory accuracy.

GOPOSLAD	DLOAD			# L/D = LAD
			LAD		# Maximum positive L/D
STOREL/D	STORE	L/D		# Store L/D and continue to LIMITL/D

# Page 873
; ============================================================================
; LIMITL/D: Roll Command Computation
;
; This section computes the bank angle (roll) command ROLLC that the CM Entry
; Digital Autopilot uses to orient the vehicle's lift vector. The algorithm
; implements lateral (crossrange) control by modulating bank angle based on
; lateral position error (LATANG) relative to the ground track.
;
; The computation involves:
; 1. Store current L/D as L/D1 for roll calculation
; 2. If GONEPAST flag set, skip lateral control (past target point)
; 3. Compute Y = KLAT*V² + LATBIAS (velocity-dependent lateral threshold)
; 4. Compare |L/D| against minimum L/D for crossrange control (L/DCMINR)
; 5. Determine if roll reversal needed based on LATANG and K2ROLL sign
; 6. Compute ROLLC = ACOS(L/D1/LAD) with sign from K2ROLL
;
; Apollo 11 Context: This logic kept Columbia on the narrow entry corridor
; leading to the Pacific splashdown point. Bank angle modulation allowed the
; CM to "fly" sideways during reentry, correcting for atmospheric variations
; and navigation errors. The crew experienced periodic roll reversals as the
; guidance computer adjusted the flight path.
; ============================================================================

LIMITL/D	DLOAD			# Load L/D into MPAC
			L/D		# Current lift-to-drag command
		STODL	L/D1		# Save for roll calculation
			VSQUARE		# Load velocity squared

		BON			# NO LATERAL CONTROL IF PAST TARGET
			GONEPAST	# Flag: spacecraft past target
			L355		# Skip to roll computation
		DMP	DAD		# Y= KLAT VSQUARE + LATBIAS
			KLAT		# Lateral gain (scaled by velocity)
			LATBIAS		# Lateral bias term
					# Y INTO PD

; Lateral Control Logic:
;
; Check if L/D is large enough to enable crossrange control. If |L/D| is less
; than L/DCMINR (minimum L/D for crossrange maneuvers), skip the Y/2 reduction
; and proceed directly to roll reversal check. This prevents excessive roll
; activity at low L/D where lateral control effectiveness is minimal.

L350		PDDL	ABS		# IF ABS(L/D)-L/DCMINR NEG, GO TO L353
			L/D		# Current L/D magnitude
		DSU	BMN		# Check against minimum
			L/DCMINR	# Minimum L/D for lateral control
			L353		# Skip Y reduction if L/D too small
		DLOAD	SIGN		# IF K2ROLL LATANG NEG, GO TO L357
			LATANG		# Lateral angle error
			K2ROLL		# Roll direction indicator
		BMN	DLOAD		# Check sign compatibility
			L357		# Different signs: go to L357
		SR1	PUSH		# Y = Y/2 (reduce threshold)

; Roll Reversal Decision Logic (L353):
;
; Determines if the spacecraft should reverse its bank angle to correct
; crossrange error. Compares LATANG*SIGN(K2ROLL) against the threshold Y.
; If the spacecraft is displaced far enough from the desired ground track
; (|LATANG*SIGN(K2ROLL) - Y| > 0) and no recent switch has occurred, reverse
; K2ROLL to flip the bank angle and steer back toward the nominal trajectory.
;
; The NOSWITCH flag prevents rapid oscillations by inhibiting reversals for
; a period after each switch. This hysteresis ensures stable flight control.

L353		DLOAD	SIGN		# IF LATANG SIGN(K2ROLL)-Y POS, SWITCH
			LATANG		# Lateral position error
			K2ROLL		# Current roll direction
		DSU			# Subtract threshold
		BMN	DLOAD		# If negative, no switch needed
			L355		# Go to roll computation
			K2ROLL		# Load current K2ROLL
		BONCLR	DCOMP		# IF NOSWITCH =1, K2ROLL= K2ROLL
			NOSWITCH	# Flag prevents rapid reversals
			L355		# Skip reversal if recently switched
		STORE	K2ROLL		# K2ROLL = -K2ROLL (reverse direction)

; Roll Command Computation (L355):
;
; Computes bank angle command ROLLC from the lift-to-drag ratio using the
; relation: ROLLC = ±ACOS(L/D1 / LAD)
;
; The arccosine function converts the normalized L/D (ranging from -LAD to
; +LAD) into a bank angle (ranging from 0° to 180°). The sign from K2ROLL
; determines the roll direction: positive K2ROLL gives positive bank angle
; (lift vector tilted right), negative K2ROLL gives negative bank angle
; (lift vector tilted left). The SR1 (shift right 1) scales the ratio before
; the ACOS operation.
;
; The NOSWITCH flag is cleared here, allowing future roll reversals once the
; current bank angle command is established. Apollo 11 executed multiple roll
; reversals during reentry as this logic adjusted Columbia's flight path to
; maintain the narrow entry corridor.

L355		DLOAD	DDV		# ROLLC = ACOS( (L/D1) / LAD)
			L/D1		# Saved L/D from earlier
			LAD		# Maximum L/D magnitude
					# MPAC SET TO +-1 IF OVERFLOW***
		SR1	ACOS		# Compute arccosine of scaled ratio
		SIGN	CLEAR		# Apply sign from K2ROLL direction
			K2ROLL		# Roll direction indicator
			NOSWITCH	# Allow future roll reversals
		STORE	ROLLC		# Final bank angle command (radians)

; ============================================================================
; ENDEXIT: Exit from Interpretive Mode
;
; Transitions from interpretive (virtual machine) mode back to native AGC
; assembly code. The computed L/D and ROLLC values are now available for use
; by the entry autopilot and display routines.
; ============================================================================

ENDEXIT		EXIT			# Return to native AGC code

; ============================================================================
; OVERNOUT: Entry Display Processing
;
; Manages crew displays during atmospheric entry. The ENTRYDSP flag (bit 13 of
; CM/FLAGS) controls whether displays are updated. If displays are active, the
; routine calls REGODSPR to show entry parameters to the crew on the DSKY.
;
; Apollo 11 Context: During Columbia's reentry on July 24, 1969, the crew
; monitored altitude, velocity, range-to-go, and predicted landing coordinates
; on these displays. The displays helped Armstrong, Aldrin, and Collins verify
; the computer's guidance was steering them toward the recovery ships in the
; Pacific Ocean.
; ============================================================================

OVERNOUT	CA	BIT13		# ENTRYDSP =92D B13
		MASK	CM/FLAGS	# Check display enable flag
		EXTEND
		BZF	NODISKY		# OMIT DISPLAY if flag clear
# Page 874
		CA	ENTRYVN		# ALL ENTRY DISPLAYS ARE DONE HERE.
		TC	BANKCALL	# Cross-bank call
		CADR	REGODSPR	# Display routine (no abort if DSKY busy)

; NODISKY: Skip Display Processing
;
; Control passes here when displays are disabled or after successful display
; update. Inhibits interrupts and checks for job conflicts before continuing
; to service exit.

NODISKY		INHINT			# Inhibit interrupts
		CCS	NEWJOB		# Check for pending job conflicts
		TC	CHANG1		# Handle job conflict if present
		
; SERVNOUT: Service Exit Point
;
; Normal exit from entry guidance computation cycle. This routine can also be
; entered directly from P67.3 (final entry phase). Transfers control to
; SERVEXIT which ends the AVERAGEG job and returns control to the executive
; scheduler for the next guidance cycle.

SERVNOUT	TC	POSTJUMP	# Jump to service exit
		CADR	SERVEXIT	# End AVERAGEG job via ENDOFJOB

# Page 875
# DISPLAY WHEN V IS LESS THAN VQUIT.

; ============================================================================
; STEEROFF: End of Active Steering Phase
;
; Entered when spacecraft velocity drops below VQUIT threshold, indicating
; that active lift vector steering is no longer effective. At this point,
; the Command Module is descending on a ballistic trajectory toward splashdown.
;
; This routine initiates P67.1 (final entry display phase) which shows the
; crew the predicted landing coordinates and range-to-go. The displays update
; as the CM descends under parachutes.
;
; Apollo 11 Context: Columbia reached this phase at approximately 24,000 feet
; altitude during the final minutes before parachute deployment. The crew
; verified their splashdown point would be near the recovery ships USS Hornet
; in the Pacific Ocean.
;
; Display Format:
;   RTOGO    LAT      LONG
;   XXXX.X   XXX.XX   XXX.XX
;   (NM)     (DEG)    (DEG)
; ============================================================================

STEEROFF	EXIT			# Exit interpretive mode
		CA	EBENTRY		# Set up E-bank
		TS	EBANK		# Precautionary bank setup

		CA	PRIO16		# Priority 16 (2 less than NTRYPRIO)
		TC	NOVAC		# Create new job
		EBANK=	AOG		# E-bank for P67
		2CADR	P67.1		# Start P67.1 display routine

					# Display format:
					# RTOGO		LAT		LONG
					# XXXX.X NM	XXX.XX DEG	XXX.XX DEG

		TC	2PHSCHNG	# Set up restart protection
		OCT	00414		# Phase 4.41: Restart for P67.1 display
		OCT	10035		# Phase 5.3: Servicer restart

		CA	P67.2CAD	# Set continuation address
		TS	GOTOADDR	# For lat/long computation next cycle

		TC	INTPRET		# Enter interpretive mode
		GOTO			# Continue to P67.2
P67.2CAD		P67.2		# Lat/long computation this time

; L357: Minimum L/D Override
;
; Special case handler that forces L/D to its minimum reference value
; (L/DCMINR) while preserving the original sign. This is used when the
; computed L/D would be excessively small, ensuring the guidance maintains
; at least minimum control authority. After setting L/D1 to this minimum
; value, control transfers to L355 to compute the corresponding ROLLC.

L357		DLOAD	SIGN		# L/D1 = L/DCMINR × SIGN(L/D)
			L/DCMINR	# Minimum L/D reference value
			L/D		# Original L/D (for sign only)
		STCALL	L/D1		# Store as L/D1
			L355		# Go compute ROLLC from this L/D

# Page 876
# TABLE USED FOR SUB-ORBITAL REFERENCE TRAJECTORY CONTROL.

; ============================================================================
; VREFER: Reference Trajectory Velocity Table
;
; This table defines the sub-orbital reference trajectory used during entry
; guidance. It contains velocity points (independent variable) followed by
; six 13-point functions of velocity. These functions describe the nominal
; trajectory parameters (altitude, range, flight path angle, etc.) that the
; guidance algorithm uses to compute steering commands.
;
; The velocity values are scaled as V/51532.3946 (approximately V/orbital_vel).
; The guidance compares actual state against these reference values to compute
; range and crossrange errors, which drive the L/D and bank angle commands.
;
; The table covers the velocity regime from initial entry interface (~400,000
; feet altitude, ~36,000 ft/sec) down to VQUIT (~2,000 ft/sec) where active
; steering ends and ballistic descent begins.
; ============================================================================

; --- Independent Variable: Velocity (13 reference points) ---
; Scaled as V/51532.3946 (where 51532.3946 ≈ 2×VS, VS = satellite velocity)
; Values span from low-speed entry to near-orbital velocity

VREFER		DEC	.019288		# 994 fps (low-speed terminal phase)
		DEC	.040809		# 2103 fps
		DEC	.076107		# 3923 fps (subsonic to supersonic transition)
		DEC	.122156		# 6296 fps
		DEC	.165546		# 8532 fps (heating peak region)
		DEC	.196012		# 10101 fps
		DEC	.271945		# 14014 fps
		DEC	.309533		# 15951 fps
		DEC	.356222		# 18359 fps (high-speed regime)
		DEC	.404192		# 20830 fps
		DEC	.448067		# 23091 fps
		DEC	.456023		# 23501 fps
		DEC	.67918		# 35001 fps (HIGHVELOCITY FOR SAFETY)

; --- Function Table 1: DRANGE/DA (13 points) ---
; Range sensitivity to drag acceleration. Represents how much the downrange
; distance changes per unit change in drag acceleration. Used to correct range
; errors by adjusting lift vector angle (which modulates drag via bank angle).
; Scaled as DRDA/(2700/805) where 2700/805 = 3.354 (drag normalization factor)
; Negative values indicate that increased drag reduces remaining range.

		DEC	-.010337	# DRANGE/DA	SCALED DRDA/(2700/805)
		DEC	-.016550
		DEC	-.026935
		DEC	-.042039
		DEC	-.058974
		DEC	-.070721
		DEC	-.098538
		DEC	-.107482
		DEC	-.147762
		DEC	-.193289
		DEC	-.602557
		DEC	-.99999
		DEC	-.99999		# Clamp value at high velocities

; --- Function Table 2: -DRANGE/DRDOT (13 points) ---
; Negative of range sensitivity to altitude rate. Represents how much downrange
; distance changes per unit change in descent rate (RDOT = dR/dt, altitude rate).
; Positive descent rate (climbing) increases range, so sensitivity is negative.
; Scaled as ((2VS/8 2700) DR/DRDOT) B-3 for computational efficiency.
; Used to predict range changes due to vertical velocity variations.

		DEC	-.0478599 B-3	# -DRANGE/DRDOT
		DEC	-.0683663 B-3	# SCALED ((2VS/8 2700) DR/DRDOT)
		DEC	-.1343468 B-3
		DEC	-.2759846 B-3
		DEC	-.4731437 B-3
		DEC	-.6472087 B-3
		DEC	-1.171693 B-3
		DEC	-1.466382 B-3
		DEC	-1.905171 B-3
		DEC	-2.547990 B-3
		DEC	-4.151220 B-3
		DEC	-5.813617 B-3
		DEC	-5.813617 B-3	# Clamp value at high velocities

; --- Function Table 3: RDOTREF (13 points) ---
; Reference altitude rate (descent rate) as a function of velocity. Represents
; the expected vertical velocity for a nominal entry trajectory. The guidance
; compares actual RDOT against this reference to detect deviations from the
; planned descent profile. Scaled as (8 RDT/2VS) B3 where VS = satellite velocity.
; Negative values indicate descent (downward vertical velocity).

# Page 877
		DEC	-.0134001  B3	# RDOTREF	SCALED (8 RDT/2VS)
		DEC	-.013947   B3
		DEC	-.013462   B3
		DEC	-.011813   B3
		DEC	-.0095631  B3
		DEC	-.00806946 B3
		DEC	-.006828   B3
		DEC	-.00806946 B3
		DEC	-.0109791  B3
		DEC	-.0151498  B3
		DEC	-.0179817  B3
		DEC	-.0159061  B3
		DEC	-.0159061  B3	# Clamp value for low velocities

; --- Function Table 4: RANGE TO GO (13 points) ---
; Nominal downrange distance remaining to target as a function of velocity.
; Provides reference trajectory for how much horizontal distance should remain
; at each velocity point. Guidance computes actual range-to-go and compares
; against this reference to determine if spacecraft is overshooting or
; undershooting the target. Scaled as RTOGO/2700 where 2700 NM is a range
; normalization factor (approximately half Earth's circumference).

		DEC	.0008067	# RANGE TO GO SCALED RTOGO/2700
		DEC	.0032963	#	8.9
		DEC	.0081852	#	22.1
		DEC	.017148
		DEC	.027926
		DEC	.037
		DEC	.063296
		DEC	.077889
		DEC	.098815
		DEC	.127519
		DEC	.186963
		DEC	.238148
		DEC	.294185185	# 794 NM remaining at high velocity

; --- Function Table 5: -AREF/805 (13 points) ---
; Negative reference drag acceleration. Represents expected drag deceleration
; at each velocity point for a nominal entry trajectory. The value 805 is a
; scaling factor (805 fps² ≈ 25 m/s² ≈ 2.5g) used to normalize drag acceleration
; into a convenient computational range. Guidance compares actual sensed drag
; against this reference to detect atmospheric density variations or trajectory
; deviations. Negative because drag opposes velocity direction.

		DEC	-.051099	# -AREF/805
		DEC	-.074534
		DEC	-.101242
		DEC	-.116646
		DEC	-.122360
		DEC	-.127081
		DEC	-.147453
		DEC	-.155528
		DEC	-.149565
		DEC	-.118509
		DEC	-.034907
		DEC	-.007950
		DEC	-.007950	# Clamp value at low velocities

; --- Function Table 6: DRANGE/D L/D (13 points) ---
; Range sensitivity to lift-to-drag ratio. Represents how much downrange
; distance changes per unit change in L/D. This is the primary control parameter
; for entry guidance: by modulating bank angle, the guidance varies the effective
; L/D to correct range errors. Positive values indicate that increasing L/D
; (more lift, less drag) increases remaining range by generating more "skip"
; and reducing descent rate. Scaled as Y/2700 where Y is the sensitivity
; coefficient and 2700 NM is the range normalization factor.

# Page 878
		DEC	.004491		# DRANGE/D L/D SCALED Y/2700
		DEC	.008081
		DEC	.016030
		DEC	.035815
		DEC	.069422
		DEC	.104519
		DEC	.122
		DEC	.172407
		DEC	.252852
		DEC	.363148
		DEC	.512963
		DEC	.558519
		DEC	.558519		# END OF STORED REFERENCE

; ============================================================================
; TRANSITION: From Reference Trajectory Tables to Entry Constants
;
; The VREFER tables above define the nominal entry trajectory as six functions
; of velocity. The guidance algorithms use these reference curves to compute
; predicted trajectory behavior and generate corrective commands. Below are the
; physical constants, scaling factors, and computational parameters used
; throughout the entry guidance calculations.
; ============================================================================

# Page 879
# REENTRY CONSTANTS.
;
; This section contains the fundamental constants used by atmospheric entry
; guidance and control. These include:
;   - Physical constants (Earth radius, satellite velocity, gravitational acceleration)
;   - Scaling factors for computational efficiency (normalized velocity, range, drag)
;   - Guidance gains and limits (maximum g-load, minimum L/D, bank angle limits)
;   - Computational thresholds (convergence criteria, phase transition points)
;
; Constants are carefully scaled to fit within AGC's 15-bit signed arithmetic
; while maintaining numerical precision throughout the 2700 NM entry trajectory.

; --- Symbolic Constants (defined by EQUALS directive) ---

DEC15		=	LOW4		# Decimal 15 for bit masking operations
#GAMMAL1	=	22D		# (Commented out) Flight path angle limit

; --- Range Error Saturation ---
; Maximum range error value used when spacecraft has passed the target (GONEPAST=1).
; Setting DNRNGERR to 9999.9 NM effectively saturates the range error computation,
; preventing numerical overflow in guidance calculations after target flyby.

MAXRNG		2OCT	1663106755	# DNRNGERR = 9999.9 NM if GONEPAST=1

		BANK	26
		SETLOC	REENTRY1
		BANK

		COUNT*	$$/ENTRY

; --- Common Mathematical Constants ---

BARELY1		=	NEARONE		# 1.0 - 1bit (shared with DISK, DANCE routines)
#1BITDP					# (Commented) 1-bit DP value (defined in VECPOINT)

; --- Fractional Constants ---

1/12TH		DEC	.083333		# 1/12 (high word, pairs with 1/3 below for DP)
1/3RD		2DEC	.3333333333	# 1/3 double-precision

1/16TH		=	DP2(-4)		# 1/16 = 2^(-4) double-precision

; --- Fundamental Physical Constants ---
; VS = VSAT = 25766.1973 ft/sec (satellite/circular orbital velocity at entry altitude)
; RE = 21,202,900 feet (Earth radius to entry interface, ~400,000 ft altitude)
;
; These reference values are used throughout entry guidance for velocity and
; range normalization. Entry velocities are scaled relative to 2×VS (≈51532 fps),
; and ranges are scaled relative to half Earth's circumference (2700 NM).

; --- Lift-to-Drag Ratio Bounds and Increments ---

LEWD1		2DEC	.15		# L/D initialization value (0.15)

POINT1		2DEC	.1		# Generic 0.1 constant

POINT2		2DEC	.2		# Generic 0.2 constant (step size, threshold)

DLEWD0		2DEC	-.05		# L/D decrement (-0.05) for range correction

GMAX/2		2DEC	.16		# 8 GS / 2

3ZEROS		EQUALS	HI6ZEROS
NEAR1/4		2OCT	0777700000	# 1/4 LESS 1 BIT IN UPPER PART.

C18		2DEC	.0097026346	# 500/2VS

Q7FKDMIN	2DEC	.0080745342	# 6.5/805  (Q7F +KDMIN) = 6 + .5)

C1/16		=	DP2(-4)

; --- Q-Series Computational Constants ---
; The Q-constants are scaling factors, gains, and thresholds used throughout
; entry guidance calculations. Most are normalized relative to the reference
; velocity 2×VS (51532 fps) and reference range (21600 NM = half circumference).

Q3		2DEC	.167003132	# Velocity threshold: 0.07×2VS/21600 scaling

# Page 880
Q5		2DEC	.326388889	# Range normalization: 0.3×23500/21600

Q6		2DEC	.0349		# Bank angle increment: 2° ≈ 820/23500 rad

Q7F		2DEC	.0074534161	# Drag scaling: 6/805 (Q7 fixed memory value)

Q19		=	HALVE		# Q19 = 0.5 constant

Q21		2DEC	.0231481481	# Velocity scaling: 500/21600

Q22		2DEC	-.053333333	# Negative coefficient: -1152/21600

; --- Velocity Limits ---

VLMIN		2DEC	.34929485	# Minimum velocity threshold: 18000/(2×VS)
					# Below this velocity (~18000 fps), certain
					# guidance modes transition or are disabled

VMIN		=	FOURTH		# Minimum normalized velocity: (VS/2)/(2VS) = 0.25
					# Corresponds to VS/2 = 12883 fps

; --- C-Series Coefficients ---

C12		2DEC	.00684572901	# Trajectory coefficient: 32×28500/(RE×2π)
					# Used in range and drag computations

1/KB1		2DEC	.29411765	# 1 / 3.4

-1/KB2		2DEC	-.0057074322 B4	# = -1/(.0034 2 VS) EXP +4

VQUIT		2DEC	.019405269	# 1000 /2VS

C20		2DEC	.21739130	# (175 FPSS) LIFT UP IF ABOVE C20

C21		2DEC	.17391304	# 140/805

25NM		2DEC	.0011574074	# 25/21600	(25 NAUT MILES)

K1D		2DEC	.0314453125	# =C16 805/256 = .01 805/256

K2D		2DEC	-.201298418	# -C17 2VS/256 = -.001 2VS/256

KVSCALE		2DEC	.81491944	# 12800/(2 VS .3048)

KASCALE		2DEC	.97657358	# 5.85 16384/(4 .3048 100 805)

KTETA		2DEC*	.383495203 E2 B-14* # 1000 2PI/16384(163.84)

KT1		2DEC*	.157788327 E 2 B-14* # RE(2PI)/2 VS(16384) 163.84

.05G		2DEC	.002		# .05/25

LATBIAS		2DEC	.00003		# APPRX .5 NM/ 4(21600/2 PI)

KWE		2DEC	.120056652 B-1

KACOS		2DEC	.004973592	# 1/32(2PI)

CHOOK		2DEC	1 B-6		# .25/16
# Page 881
1/24TH		2DEC	.0833333333 B-1

CH1		2DEC	.32 B1		# 16 CH1/25 = 16 (1) /25

KC3		2DEC	-.0247622232	# -(4 VS VS/ 2 PI 805 RE)

VRCONT		2DEC	.0135836886	# 700/2 VSAT

HALVE		EQUALS	HIDPHALF
FOURTH		EQUALS	HIDP1/4

1/GMAX		EQUALS	HALVE		# 4/GMAX = 4 / 8
2HS		2DEC	.0172786611	# 2 28500 25 32.2/(4 VS VS)

2HSGMXSQ	2DEC	.0000305717	# (2 28500 8 32.2/ 4 VS VS)SQ

C001		2DEC	-.000625	# -(4/25)/256	LEQ/D0 CONST

POINT8		2DEC	.8

2C1HS		2DEC	.0215983264	# 2 1.25 28500 805/(2 VS)SQ

; --- Precision and Step Size Constants ---

PT1/16		2DEC	.1 B-4		# 0.1 scaled by 2^(-4) = 0.00625
					# Used as small increment in iterative computations

; --- Trajectory Scaling and Velocity Constants ---

1/K44		2DEC	.00260929464	# Inverse trajectory coefficient: 2VS/19749550
					# Used in range and velocity computations

VFINAL		2DEC	.51618016	# Final approach velocity: 26600/(2VS)
					# Target velocity for final entry phase (~26600 fps)

VFINAL1		2DEC	.523942273	# Alternate final velocity: 27000/(2VS)
					# Slightly higher threshold (~27000 fps)

; --- KA-Series Drag Acceleration Constants ---
; These constants are used in drag acceleration modeling and G-load limiting.
; The KA series provides scaling factors for converting sensed accelerations
; into guidance-usable values and establishing G-load protection limits.

1/KA1		2DEC	.30048077	# Inverse gain: 25/(1.3×64)
					# Drag acceleration scaling factor

KA2		2DEC	.008		# Small gain: 0.2/25
					# Fine adjustment coefficient

KA3		2DEC	.44720497	# Drag reference: 90×√4/805
					# Combined scaling for drag magnitude

KA4		2DEC	.049689441	# Drag normalization: 40/805
					# Standard drag scaling (≈0.05)

KALIM		2DEC	.06		# Drag acceleration limit: 1.5/25
					# Maximum allowable normalized drag (~0.06)

; --- Final Constants ---

Q7MIN		=	KA4		# Minimum Q7 value equals KA4 = 40/805
					# Lower bound on drag acceleration scaling

-HSCALED	2DEC	-.55305018	# Negative altitude scale: -28500/(2VS)
					# Used in altitude rate computations

-KSCALE		2DEC	-.0312424837	# Negative drag scale: -805/VS
					# Inverse drag normalization (negative)

COS15		2DEC	.965		# cos(15°) ≈ 0.9659
					# Used in lateral range computations (L/DCMINR)

LATSLOPE	EQUALS	1/12TH		# Lateral slope constant = 1/12
					# Ratio for cross-range to downrange conversion

; ============================================================================
; END OF REENTRY_CONTROL.AGC
;
; This file has implemented the complete atmospheric entry guidance system
; for the Apollo Command Module. The algorithms computed lift vector steering
; commands that guided Apollo 11 through Earth's atmosphere on July 24, 1969,
; bringing Armstrong, Aldrin, and Collins safely home to a precise splashdown
; in the Pacific Ocean near the recovery ships.
;
; The entry corridor was narrow - too steep and the spacecraft would burn up
; from excessive heating, too shallow and it would skip back into space. This
; guidance system successfully threaded that needle by continuously adjusting
; the bank angle to modulate lift, steering the Command Module to the target
; landing point while keeping G-loads within crew tolerance and protecting
; the heat shield from excessive temperatures.
;
; Columbia's entry marked the successful conclusion of humanity's first journey
; to another world.
; ============================================================================

# ... END OF RE-ENTRY CONSTANTS ...

