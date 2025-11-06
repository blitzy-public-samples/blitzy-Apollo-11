# Copyright:    Public domain.
# Filename:     STABLE_ORBIT.agc
# Purpose:      Part of the source code for Colossus 2A, AKA Comanche 055.
#               It is part of the source code for the Command Module's (CM)
#               Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:    yaYUL
# Contact:      Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:      www.ibiblio.org/apollo.
# Pages:	525-532
# Mod history:  2009-05-10 HG    Started adapting from the Colossus249/ file
#                of the same name, using Comanche055 page
#                images 0525.jpg - 0532.jpg.
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
; FILE: STABLE_ORBIT.agc
; MODULE: COMEKISS Subsystem (Orbital Navigation)
; MISSION PHASE: earth-orbit/lunar-orbit
;
; TL;DR: Stable orbit rendezvous programs (P38/P78/P39/P79) calculating
;        delta-V for transfer trajectories intercepting passive vehicle orbit
;        and midcourse corrections. Used during Apollo 11 Earth parking orbit
;        and lunar orbit coast phases for rendezvous trajectory planning and
;        orbit stability monitoring.
;
; COMMENT-ONLY READERS: These programs planned rendezvous maneuvers by
;        calculating when and how much to fire engines to intercept another
;        spacecraft in a stable orbit.
; CODE-ALONG READERS: Study Lambert problem solution for rendezvous targeting,
;        delta-V computation algorithms, orbit stability criteria, and DSKY
;        crew interaction sequences for rendezvous planning.
; ============================================================================

# Page 525
; ============================================================================
; STABLE ORBIT RENDEZVOUS PROGRAMS (P38/P78 and P39/P79)
;
; COMMENT-ONLY READERS: During rendezvous operations, one spacecraft (the
; "active vehicle") needs to maneuver to meet another spacecraft (the "passive
; vehicle") that is orbiting stably. These programs calculate exactly when to
; fire engines and how much thrust to apply to achieve rendezvous.
;
; CODE-ALONG READERS: Implements stable orbit rendezvous using two-burn
; transfer sequence. SOI (Stable Orbit Insertion) places active vehicle on
; intercept trajectory. SOR (Stable Orbit Rendezvous) circularizes orbit
; to match passive vehicle. P38/P78 select which vehicle is active.
; P39/P79 provide midcourse corrections between burns.
; ============================================================================

# STABLE ORBIT RENDEZVOUS PROGRAMS (P38 AND P78)
#
# MOD NO -1		LOG SECTION - STABLE ORBIT - P38-P39
# MOD BY RUDNICKI.S	DATE 25JAN68
#
# FUNCTIONAL DESCRIPTION
#
#	P38 AND P78 CALCULATE THE REQUIRED DELTA V AND OTHER INITIAL
#	CONDITIONS REQUIRED BY THE AGC TO (1) PUT THE ACTIVE VEHICLE
#	ON A TRANSFER TRAJECTORY THAT INTERCEPTS THE PASSIVE VEHICLE
#	ORBIT A GIVEN DISTANCE, DELTA R, EITHER AHEAD OF OR BEHIND THE
#	PASSIVE VEHICLE AND (2) ACTUALLY PLACE THE ACTIVE VEHICLE IN THE
#	PASSIVE VEHICLE ORBIT WITH A DELTA R SEPARATION BETWEEN THE TWO
#	VEHICLES
#
; ============================================================================
; CREW PROCEDURES - DSKY INTERACTION
;
; COMMENT-ONLY READERS: The crew initiates rendezvous planning by entering
; verb 37 (change program) on the DSKY keyboard, followed by either program
; 38 or 78 depending on which spacecraft will perform the maneuvers.
;
; CODE-ALONG READERS: DSKY verb/noun interface for rendezvous targeting.
; V37E38E: This vehicle (Command Module) performs active maneuvers
; V37E78E: Other vehicle (Lunar Module) performs active maneuvers
; Program prompts for TIG (Time of Ignition), orbital angles, separation
; distance, and maneuver option selection through DSKY numerical displays.
; ============================================================================

# CALLING SEQUENCE
#
#	ASTRONAUT REQUEST THRU DSKY
#
#	V37E38E		IF THIS VEHICLE IS ACTIVE VEHICLE
#	V37E78E		IF OTHER VEHICLE IS ACTIVE VEHICLE
#
# INPUT
#
#	(1)	SOI MANEUVER
#
#		(A)  TIG	TIME OF SOI MANEUVER
#		(B)  CENTANG	ORBITAL CENTRAL ANGLE OF THE PASSIVE VEHICLE
#				DURING TRANSFER FROM TIG TO TIME OF INTERCEPT
#		(C)  DELTAR	THE DESIRED SEPARATION OF THE TWO VEHICLES
#				SPECIFIED AS A DISTANCE ALONG THE PASSIVE VEHICLE
#				ORBIT
#		(D)  OPTION	EQUALS 1 FOR SOI

; COMMENT-ONLY READERS: For the first burn (SOI), the crew enters the planned
; ignition time, how far the target spacecraft will travel during the transfer,
; the desired final separation distance, and selects option 1.
;
; CODE-ALONG READERS: SOI (Stable Orbit Insertion) input parameters:
; TIG scaled in centiseconds, CENTANG in revolutions (±0.5 = ±180°),
; DELTAR in nautical miles scaled by 2^-24, OPTION flag distinguishes
; SOI (value 1) from SOR (value 2) maneuver calculations.
#
;
; COMMENT-ONLY READERS: For the second burn (SOR), the crew enters the new
; ignition time and selects option 2. The computer remembers parameters from
; the first burn and calculates the circularization maneuver needed to match
; the target spacecraft's orbit exactly.
;
; CODE-ALONG READERS: SOR (Stable Orbit Rendezvous) uses saved state from SOI.
; DELTTIME and TINT preserved in memory from SOI calculation. SOR recalculates
; based on actual achieved trajectory after SOI burn execution, allowing
; corrections for off-nominal SOI performance.
;
#	(2)	SOR MANEUVER
#
#		(A)  TIG	TIME OF SOR MANEUVER
#		(B)  CENTANG	AN OPTIONAL RESPECIFICATION OF 1 (B) ABOVE
#		(C)  OPTION	EQUALS 2 FOR SOR
#		(D)  DELTTIME	THE TIME REQUIRED TO TRAVERSE DELTA R WHEN
#				TRAVELING AT A VELOCITY EQUAL TO THE HORIZONTAL
#				VELOCITY OF THE PASSIVE VEHICLE - SAVED FROM
#				SOI PHASE
#		(E)  TINT	TIME OF INTERCEPT (SOI) - SAVED FROM SOI PHASE
#
; ============================================================================
; COMPUTED OUTPUTS DISPLAYED TO CREW
;
; COMMENT-ONLY READERS: After computing the rendezvous trajectory, the computer
; displays critical information to the crew: when to start the burn countdown,
; how much velocity change is needed, what the orbital altitude will be after
; the burn, and the spacecraft attitude required for the maneuver.
;
; CODE-ALONG READERS: Output parameters formatted for DSKY display using
; standard verb/noun combinations. TTOGO provides burn countdown timer.
; DELVTPI magnitude shown in feet per second. POSTTPI altitude in nautical
; miles. +MGA gimbal angle in degrees for IMU alignment verification.
; DELVLVC components allow crew to monitor three-axis velocity change.
; ============================================================================

# OUTPUT
#
#	(1)  TRKMKCNT	NUMBER OF MARKS
#	(2)  TTOGO	TIME TO GO
#	(3)  +MGA	MIDDLE GIMBAL ANGLE
# Page 526
#	(4)  DSPTEM1	TIME OF INTERCEPT OF PASSIVE VEHICLE ORBIT
#			(FOR SOI ONLY)
#	(5)  POSTTPI	PERIGEE ALTITUDE OF ACTIVE VEHICLE ORBIT AFTER
#			THE SOI (SOR) MANEUVER
#	(6)  DELVTPI	MAGNITUDE OF DELTA V AT SOI (SOR) TIME
#	(7)  DELVTPF	MAGNITUDE OF DELTA V AT INTERCEPT TIME
#	(8)  DELVLVC	DELTA VELOCITY AT SOI (AND SOR) - LOCAL VERTICAL
#			COORDINATES
#
# SUBROUTINES USED
#
#	AVFLAGA
#	AVFLAGP
#	VNDSPLY
#	BANKCALL
#	GOFLASHR
#	GOTOPOOH
#	BLANKET
#	ENDOFJOB
#	PREC/TT
#	SELECTMU
#	INTRPVP
#	MAINRTNE

; ============================================================================
; PROGRAM ENTRY POINTS - P38/P78 STABLE ORBIT RENDEZVOUS
;
; COMMENT-ONLY READERS: Programs P38 and P78 calculate rendezvous maneuvers.
; P38 is used when this spacecraft (Command Module) performs the burns. P78
; is used when the other spacecraft (Lunar Module) is the active vehicle.
; The crew enters the program by keying V37E38E or V37E78E on the DSKY.
;
; CODE-ALONG READERS: Programs reside in memory bank 04. AVFLAGA sets active
; vehicle flag for this vehicle. AVFLAGP sets flag for other vehicle active.
; P20FLGON enables update and tracking flags. EBANK points to SUBEXIT for
; erasable memory bank context.
; ============================================================================

		BANK	04
		SETLOC	STBLEORB
		BANK

		EBANK=	SUBEXIT
		COUNT*	$$/P3879

P38		TC	AVFLAGA         # THIS VEHICLE ACTIVE
		TC      +2
P78		TC	AVFLAGP         # OTHER VEHICLE ACTIVE
		
; The spacecraft has entered stable orbit rendezvous planning mode.
; The computer now prompts the crew to enter time of ignition (TIG)
; and central angle - the orbital angle the passive vehicle travels
; during the rendezvous transfer trajectory.

		TC      P20FLGON        # SET UPDATFLG, TRACKFLG
		CAF	V06N33SR	# DISPLAY TIG
		TC	VNDSPLY
		CAF	V06N55SR	# DISPLAY CENTANG
		TCR	BANKCALL
		CADR	GOFLASHR
		TCF	GOTOPOOH	# TERMINATE
		TCF	+5		# PROCEED
		TCF	-5		# RECYCLE
		CAF	THREE		# IMMEDIATE RETURN - BLANK R1, R2
		TCR	BLANKET
		TCF	ENDOFJOB
		CAF	FIVE
		TS	OPTION1
		CAF	ONE
		TS	OPTION2		# OPTION CODE IS SET TO 1

; ============================================================================
; MANEUVER TYPE SELECTION - SOI OR SOR
;
; COMMENT-ONLY READERS: The crew now selects which maneuver to plan:
; Option 1 = SOI (Stable Orbit Insertion) - the first burn that puts the
; spacecraft on an intercept trajectory toward the target orbit.
; Option 2 = SOR (Stable Orbit Rendezvous) - the second burn that circularizes
; the orbit to match the passive vehicle's orbit for final rendezvous.
;
; CODE-ALONG READERS: OPTION2 initialized to 1 for SOI by default. V04N06
; displays option code on DSKY R1 for crew confirmation or modification.
; GOFLASHR flashes display waiting for crew response (PROCEED/RECYCLE/TERMINATE).
; BIT3 used to blank R3 register on immediate return from display.
; ============================================================================

# Page 527
		CAF	V04N06SR	# DISPLAY OPTION CODE - 1 = SOI, 2 = SOR
		TCR	BANKCALL
		CADR	GOFLASHR
		TCF	GOTOPOOH	# TERMINATE
		TCF	+5		# PROCEED
		TCF	-5		# RECYCLE
		CAF	BIT3		# IMMEDIATE RETURN - BLANK R3
		TCR	BLANKET
		TCF	ENDOFJOB
; ============================================================================
; TRANSITION: From crew input to computational processing
;
; The computer has received all required inputs from the crew. It now switches
; to interpretive mode for high-precision vector and trajectory calculations.
; The option selected (SOI or SOR) determines which computational path to follow.
; ============================================================================

		TC	INTPRET
		
; Interpretive mode activated for double-precision vector mathematics.
; Initialize loop counter NN=2 for iteration control. Load OPTION2 and
; shift right 1 bit to test for zero (SOI=1, SOR=2).

		SSP
		        NN
		        2
		SLOAD   SR1
			OPTION2
		BHIZ	DLOAD
			OPTN1
			TINT

; For SOR (Option 2): Load time of intercept from SOI phase.
; This was saved during the previous SOI calculation and represents
; when the spacecraft will reach the passive vehicle's orbital plane.

		STORE	TINTSOI		# STORE FOR SOR PHASE
		CLRGO
			OPTNSW		# OPTNSW; ON = SOI, OFF = SOR
			JUNCTN1

; For SOI (Option 1): Calculate time of intercept by adding time of
; free fall (PREC/TT) to the ignition time. This determines when the
; spacecraft will intersect the passive vehicle's orbit after the burn.

OPTN1		SET	CLEAR		# SOI
			OPTNSW
			UPDATFLG
		CALL
			PREC/TT
		SET	DAD
			UPDATFLG
			TIG
		STORE	TINT		# TI = TIG + TF
		STORE	DSPTEM1		# FOR DISPLAY
		EXIT
; The computer displays Delta-R, the desired separation distance between
; the two spacecraft along the passive vehicle's orbit after rendezvous.
; This value was entered earlier by the crew during initial setup.

		CAF	V06N57SR	# DISPLAY DELTA R
		TCR	BANKCALL
		CADR	GOFLASHR
		TCF	GOTOPOOH	# TERMINATE
		TCF	+5		# PROCEED
		TCF	-5		# RECYCLE
		CAF	SIX		# IMMEDIATE RETURN - BLANK R2, R3
		TCR	BLANKET
		TCF	ENDOFJOB

; Display the calculated time of intercept. This is when the spacecraft
; trajectory will cross the passive vehicle's orbital plane if the
; computed burn is executed at TIG.

		CAF	V06N34SR	# DISPLAY TIME OF INTERCEPT
		TC	VNDSPLY
		TC	INTPRET

; ============================================================================
; MAIN CALCULATION JUNCTION - ORBITAL MECHANICS PROCESSING
;
; COMMENT-ONLY READERS: With all inputs confirmed, the computer begins the
; complex orbital calculations. It determines the current position of both
; spacecraft, predicts where they will be at intercept time, and calculates
; the precise velocity change needed to achieve rendezvous.
;
; CODE-ALONG READERS: SELECTMU selects gravitational parameter (Earth or Moon).
; PREC/TT computes precision time of free fall for transfer trajectory.
; Calculations performed in double-precision interpretive mode for accuracy
; required in orbital mechanics (position errors < 1 meter).
; ============================================================================

JUNCTN1		CLEAR	CALL
			P39/79SW
			SELECTMU	# SELECT MU, CLEAR FINALFLG, GO TO VN1645
RECYCLE		CALL
			PREC/TT
# Page 528

; ============================================================================
; PASSIVE VEHICLE STATE VECTOR COMPUTATION
;
; COMMENT-ONLY READERS: The computer calculates where the passive vehicle
; (the target spacecraft) will be at intercept time. It propagates the
; passive vehicle's orbit forward from current time to the predicted
; intercept, accounting for orbital motion around Earth or Moon.
;
; CODE-ALONG READERS: Branch based on OPTNSW flag (SOI vs SOR). INTRPVP
; performs precision integration of passive vehicle state vector to TINT.
; RATT = passive vehicle position vector at intercept time (meters, scaled).
; VATT = passive vehicle velocity vector at intercept time (m/s, scaled).
; ============================================================================

		BOFF	DLOAD
			OPTNSW
			OPTN2
			TINT
		STCALL	TDEC1		# PRECISION UPDATE PASSIVE VEHICLE TO
			INTRPVP		#	INTERCEPT TIME

; Compute unit vector of passive vehicle position: RP/(RP) magnitude.
; This defines the radial direction from central body to passive vehicle.
; Cross product VP x RP gives angular momentum vector perpendicular to
; orbital plane. Used to calculate traverse time for Delta-R distance.

		VLOAD	UNIT
			RATT		# RP/(RP)
		PDVL	VXV
			VATT
		ABVAL	NORM		# (VP X RP/(RP))
			X1
		PDDL	DDV
			DELTAR
		SL*			# DELTA R / (VP X RP/RP)
			0 -7,1

; Calculate Delta-Time: time required to traverse Delta-R distance along
; passive vehicle orbit at current velocity. Delta-T = Delta-R / V_tangential.
; This determines phasing offset between active and passive vehicles.

		STCALL	DELTTIME	# DELTA T = (RP) DELTA R / (VP X RP)
			JUNCTN2

; For SOR option: Add time of free fall to previous intercept time.
; Refines intercept calculation based on trajectory updates.

OPTN2		DLOAD	DAD
			TINTSOI
			T
		STORE	TINT		# TI = TI + TF

; Calculate target time for maneuver: intercept time minus Delta-Time.
; This is when the active vehicle must reach the intercept point to
; achieve the desired separation distance from the passive vehicle.

JUNCTN2		DLOAD	DSU
			TINT
			DELTTIME
		STORE	TARGTIME	# TT = TI - DELTA T

# .... MAINRTNE ....
# SUBROUTINES USED
#
#	S3435.25
#	PERIAPO1
#	SHIFTR1
#	VNDSPLY
#	BANKCALL
#	GOFLASH
#	GOTOPOOH
#	VN1645

; ============================================================================
; MAIN TRAJECTORY COMPUTATION ROUTINE
;
; COMMENT-ONLY READERS: Now the computer performs the actual trajectory
; calculations. It determines the velocity change (Delta-V) needed at the
; time of ignition (TIG) to place the spacecraft on an intercept path,
; and calculates the final velocity change at intercept to match orbits
; with the passive vehicle. The results are displayed for crew review.
;
; CODE-ALONG READERS: INTRPVP propagates passive vehicle state to target
; time. S3435.25 computes Lambert solution for transfer trajectory from
; active vehicle position at TIG to passive vehicle position at target time.
; Result vectors: DELVEET3 = Delta-V at TIG, VPASS4 = passive velocity,
; VTPRIME = active velocity after maneuver, all in local vertical coords.
; ============================================================================

MAINRTNE	STCALL	TDEC1		# PRECISION UPDATE PASSIVE VEHICLE TO
			INTRPVP		#	TARGET TIME
		DLOAD
			TIG
		STORE	INTIME
		SSP	VLOAD
			SUBEXIT
			TEST3979
			RATT
		CALL
			S3435.25

; Branch logic based on program mode flags. P39/79 midcourse programs
; have different display requirements than main P38/78 programs.
; FINALFLG indicates if this is final computation or iterative refinement.

TEST3979	BOFF	BON
# Page 529
			P39/79SW
			MAINRTN1
			FINALFLG
			P39P79
		SET
			UPDATFLG
P39P79		EXIT
		TC	DSPLY81		# FOR P39 AND P79

; Compute Delta-V magnitudes for crew display:
; DELVTPI = magnitude of initial velocity change at TIG
; DELVTPF = magnitude of final velocity change at intercept
; These values critical for fuel budget planning.

MAINRTN1	VLOAD	ABVAL
			DELVEET3
		STOVL	DELVTPI		# DELTA V
			VPASS4
		VSU	ABVAL
			VTPRIME
		STOVL	DELVTPF		# DELTA V (FINAL) = V'T - VT
			RACT3

; Calculate perigee altitude of post-maneuver orbit. This ensures the
; transfer trajectory does not intersect the central body (Earth or Moon).
; PERIAPO1 finds lowest point of orbit, SHIFTR1 scales to display units.

		PDVL	CALL
			VIPRIME
			PERIAPO1	# GET PERIGEE ALTITUDE
		CALL
			SHIFTR1
		STORE	POSTTPI
		BON	SET
			FINALFLG
			DSPLY58
			UPDATFLG

; Display computed parameters to crew via DSKY for evaluation:
; V06N58: Perigee altitude (HP), initial Delta-V, final Delta-V
; V06N81: Delta-V components in local vertical coordinates
; Crew can proceed with burn, recycle for new parameters, or terminate.

DSPLY58		EXIT
		CAF	V06N58SR	# DISPLAY HP, DELTA V, DELTA V (FINAL)
		TC	VNDSPLY
DSPLY81		CAF	V06N81SR	# DISPLAY DELTA V (LV)
		TC	VNDSPLY
		TC	INTPRET
		CLEAR	VLOAD
			XDELVFLG
			DELVEET3
		STCALL	DELVSIN
			VN1645		# DISPLAY TRKMKCNT, TTOGO, +MGA
		BON	GOTO
			P39/79SW
			P39/P79B
			RECYCLE

; ============================================================================
; TRANSITION: From P38/P78 Initial Rendezvous to P39/P79 Midcourse Correction
;
; After the SOI maneuver places the active vehicle on a transfer trajectory,
; tracking data may reveal small errors requiring midcourse correction. The
; P39/P79 programs compute a correction burn to refine the rendezvous before
; the final SOR maneuver. This improves accuracy and conserves fuel.
; ============================================================================

# STABLE ORBIT MIDCOURSE PROGRAM (P39 AND P79)
#
# MOD NO -1		LOG SECTION - STABLE ORBIT - P38-P39
# MOD BY RUDNICKI, S	DATE 25JAN68
#
# FUNCTIONAL DESCRIPTION
#
#	P39 AND P79 CALCULATE THE REQUIRED DELTA V AND OTHER INITIAL
#	CONDITIONS REQUIRED BY THE AGC TO MAKE A MIDCOURSE CORRECTION
# Page 530
#	MANEUVER AFTER COMPLETING THE SOI MANEUVER BUT BEFORE MAKING
#	THE SOR MANEUVER
#
# CALLING SEQUENCE
#
#	ASTRONAUT REQUEST THRU DSKY
#
#	V37E39E		IF THIS VEHICLE IS ACTIVE VEHICLE
#	V37E79E		IF OTHER VEHICLE IS ACTIVE VEHICLE
#
# INPUT
#
#	(1)  TPASS4	TIME OF INTERCEPT - SAVED FROM P38/P78
#	(2)  TARGTIME	TIME THAT PASSIVE VEHICLE IS AT INTERCEPT POINT -
#			SAVED FROM P38/P78
#
# OUTPUT
#
#	(1)  TRKMKCNT	NUMBER OF MARKS
#	(2)  TTOGO	TIME TO GO
#	(3)  +MGA	MIDDLE GIMBAL ANGLE
#	(4)  DELVLVC	DELTA VELOCITY AT MID - LOCAL VERTICAL COORDINATES
#
# SUBROUTINES USED
#
#	AVFLAGA
#	AVFLAGP
#	LOADTIME
#	SELECTMU
#	PRECSET
#	S34/35.1
#	MAINRTNE

; ============================================================================
; P39/P79 MIDCOURSE CORRECTION ENTRY POINTS
;
; COMMENT-ONLY READERS: Programs P39 and P79 handle midcourse trajectory
; adjustments between the initial and final rendezvous burns. The crew
; initiates these programs through the DSKY after tracking confirms the
; need for correction. The computer recalculates the required burn using
; updated position data.
;
; CODE-ALONG READERS: Similar structure to P38/P78 entry. P39/79SW flag
; distinguishes midcourse mode from initial/final modes. P20FLGON enables
; tracking and update flags. PRECSET propagates both vehicle state vectors
; to TIG with precision integration.
; ============================================================================

P39		TC	AVFLAGA		# THIS VEHICLE ACTIVE
		EXTEND
		DCA	ATIGINC
		TC	P39/P79A
P79		TC	AVFLAGP		# OTHER VEHICLE ACTIVE
		EXTEND
		DCA	PTIGINC
P39/P79A	DXCH	KT		# TIME TO PREPARE FOR BURN
		TC	P20FLGON	# SET UPDATFLG, TRACKFLG
		TC	INTPRET
		SET	CALL
			P39/79SW
			SELECTMU	# SELECT MU, CLEAR FINALFLG, GO TO VN1645

; Load current mission time and add preparation interval to establish TIG.
; Midcourse burns typically smaller than initial/final burns, allowing
; shorter preparation time. Precision state vector updates account for
; orbital motion since previous maneuver.

P39/P79B	RTB	DAD
			LOADTIME
			KT
		STORE	TIG		# TIG = T (PRESENT) + PREPARATION TIME
# Page531
		STCALL	TDEC1		# PRECISION UPDATE ACTIVE AND PASSIVE
			PRECSET		#	VEHICLES TO TIG
		CALL
			S34/35.1	# GET UNIT NORMAL
		DLOAD	GOTO
			TARGTIME
			MAINRTNE	# CALCULATE DELTA V AND DELTA V (LV)

; ============================================================================
; PREC/TT - PRECISION STATE UPDATE AND TRANSFER TIME COMPUTATION
;
; COMMENT-ONLY READERS: This subroutine updates both spacecraft positions
; to the burn time and calculates how long the transfer trajectory will
; take. This ensures the computer uses current orbital data rather than
; outdated positions, improving rendezvous accuracy.
;
; CODE-ALONG READERS: PRECSET performs precision integration of both vehicle
; state vectors to TIG. RPASS3/VPASS3 contain passive vehicle position and
; velocity vectors, scaled for interpretive processing. VSR* performs variable
; right-shift for scaling adjustment. TIMETHET computes transfer time based
; on central angle traversed. S34/35.1 calculates orbit plane normal vector.
; CSTH/SNTH store cosine/sine of central angle for trajectory calculations.
; ============================================================================

# .... PREC/TT ....
# SUBROUTINES USED
#
#	PRECSET
#	TIMETHET
#	S34/35.1

PREC/TT		STQ	DLOAD
			RTRN
			TIG
		STCALL	TDEC1		# PRECISION UPDATE ACTIVE AND PASSIVE
			PRECSET		#	VEHICLES TO TIG
		VLOAD	VSR*
			RPASS3
			0,2
		STODL	RVEC
			CENTANG
		PUSH	COS
		STODL	CSTH
		SIN	SET
			RVSW
		STOVL	SNTH
			VPASS3
		VSR*
			0,2
		STCALL	VVEC		# GET TRANSFER TIME BASED ON CENTANG OF
			TIMETHET	#	PASSIVE VEHICLE
		CALL
			S34/35.1	# GET UNIT NORMAL
		DLOAD	GOTO
			T
			RTRN
; ============================================================================
; INTRPVP - PASSIVE VEHICLE PRECISION STATE VECTOR UPDATE
;
; COMMENT-ONLY READERS: This routine updates the other spacecraft's position
; and velocity to a specific time. If the Command Module is performing the
; rendezvous maneuvers, this updates the Lunar Module's position. If the LM
; is active, it updates the CM's position. Accurate tracking of both vehicles
; is essential for safe rendezvous.
;
; CODE-ALONG READERS: AVFLAG bit determines active vehicle. BOFF branches
; if flag is clear (other vehicle active). CSMPREC integrates CSM state
; vector forward to TDEC1 time. LEMPREC integrates LM state vector. Both
; use precision orbital integration accounting for gravitational perturbations.
; Return address saved in RTRN for subroutine linkage.
; ============================================================================

# .... INTRPVP ....
# SUBROUTINES USED
#
#	CSMPREC
#	LEMPREC

INTRPVP		STQ	BOFF		# PRECISION UPDATE PASSIVE VEHICLE TO
			RTRN		#	TDEC1
			AVFLAG
			OTHERV
		CALL
# Page 532
			CSMPREC
		GOTO
			RTRN
OTHERV		CALL
			LEMPREC
		GOTO
			RTRN
; ============================================================================
; VNDSPLY - VERB/NOUN DISPLAY ROUTINE WITH FLASH
;
; COMMENT-ONLY READERS: This routine displays information on the DSKY and
; flashes the display to get the crew's attention. The astronaut can respond
; by pressing PROCEED to continue, pressing TERMINATE to abort the program,
; or pressing RECYCLE to refresh the display. This is how the computer
; communicates rendezvous data to the crew during P38/P78/P39/P79 operations.
;
; CODE-ALONG READERS: QXCH saves return address in RTRN. VERBNOUN holds the
; verb/noun code for the display format. BANKCALL to GOFLASH initiates the
; flashing display and waits for crew input. TCF GOTOPOOH terminates program
; if crew presses V34 (TERMINATE). TC RTRN returns on PROCEED. TCF -5 loops
; back to refresh display on RECYCLE. Verb/noun definitions below specify
; display formats used throughout stable orbit programs.
; ============================================================================

# .... VNDSPLY ....
# SUBROUTINES USED
#
#	BANKCALL
#	GOFLASH
#	GOTOPOOH

VNDSPLY		EXTEND			# FLASH DISPLAY
		QXCH	RTRN
		TS	VERBNOUN
		CA	VERBNOUN
		TCR	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH	# TERMINATE
		TC	RTRN		# PROCEED
		TCF	-5		# RECYCLE

; ============================================================================
; VERB/NOUN DISPLAY FORMAT DEFINITIONS
;
; COMMENT-ONLY READERS: These codes define what information appears on the
; DSKY display during rendezvous operations:
; - V06N33: Display time (hours, minutes, seconds)
; - V06N55: Display time of intercept
; - V04N06: Display time to ignition (flashing)
; - V06N57: Display option code
; - V06N34: Display central angle
; - V06N58: Display delta-R (separation distance)
; - V06N81: Display apogee, perigee, and delta-V magnitude
;
; CODE-ALONG READERS: VN pseudo-op defines verb/noun pair as single word.
; First two digits specify verb (V06 = display decimal, V04 = display with
; monitor). Last two digits specify noun (data type). These constants are
; loaded into VERBNOUN and passed to display interface routines.
; ============================================================================

V06N33SR	VN	0633
V06N55SR	VN	0655
V04N06SR	VN	0406
V06N57SR	VN	0657
V06N34SR	VN	0634
V06N58SR	VN	0658
V06N81SR	VN	0681

# *** END OF COMEKISS.020 ***
