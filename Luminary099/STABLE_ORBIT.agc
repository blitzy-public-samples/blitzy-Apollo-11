# Copyright:	Public domain.
# Filename:	STABLE_ORBIT.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	723-730
# Mod history:	2009-05-19 RSB	Adapted from the corresponding
#				Colossus249 file (there being no corresponding
#				Luminary131 source-code file), using page
#				images from Luminary 1A.
#		2009-06-07 RSB	Eliminated an extraneous instruction.
#		2011-01-06 JL	Fixed pseudo-label indentation.
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
; FILE: STABLE_ORBIT.agc
; MODULE: Rendezvous Navigation and Orbital Operations
; MISSION PHASE: rendezvous/lunar-orbit
;
; TL;DR: Implements stable orbit rendezvous programs (P38/P78) and midcourse
;        correction programs (P39/P79) that compute delta V maneuvers required
;        to establish transfer trajectories and achieve precise orbital
;        separation between spacecraft during rendezvous operations. Monitors
;        orbital stability, predicts orbit decay from perturbations, and
;        calculates midcourse corrections to maintain target trajectories.
;
; COMMENT-ONLY READERS: This code calculates the velocity changes needed to
;        move from one spacecraft orbit to intercept another spacecraft during
;        rendezvous. Follow the delta V calculations and intercept geometry.
; CODE-ALONG READERS: Study the Lambert targeting algorithms, conic orbit
;        propagation, and perturbation analysis. Note the SOI (Stable Orbit
;        Insertion) and SOR (Stable Orbit Rendezvous) maneuver calculations.
; ============================================================================

# Page 723
; ============================================================================
; STABLE ORBIT RENDEZVOUS PROGRAMS (P38 AND P78)
;
; During Apollo rendezvous operations, one spacecraft must transfer from its
; current orbit to intercept another spacecraft's orbit at a precise location.
; These programs calculate the engine burns needed to accomplish this maneuver.
;
; P38 is used when this vehicle (the Lunar Module) is the active vehicle
; performing the rendezvous. P78 is used when the other vehicle (the Command
; Module) is active and this vehicle is passive.
;
; The rendezvous consists of two maneuvers:
; 1. SOI (Stable Orbit Insertion) - transfers from current orbit to intercept
;    the target orbit at a specified distance ahead or behind the target
; 2. SOR (Stable Orbit Rendezvous) - circularizes into the target orbit with
;    the desired separation distance maintained between vehicles
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
; CREW PROCEDURES
;
; The astronaut initiates these programs through the DSKY (Display and
; Keyboard) using verb-noun sequences:
;
; V37E38E - Start Program 38 if the LM is performing the rendezvous maneuver
; V37E78E - Start Program 78 if the CM is performing the rendezvous maneuver
;
; The crew must provide timing, geometry, and targeting information which the
; program uses to compute the required velocity changes (delta V).
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
#				DURING THE TRANSFER FROM TIG TO TIME OF INTERCEPT
#		(C)  DELTAR	THE DESIRED SEPARATION OF THE TWO VEHICLES
#				SPECIFIED AS A DISTANCE ALONG THE PASSIVE VEHICLE
#				ORBIT
#		(D)  OPTION	EQUALS 1 FOR SOI
#
;
; SOI MANEUVER (Stable Orbit Insertion):
; The first burn transfers the active vehicle from its current orbit onto a
; trajectory that will intercept the passive vehicle's orbit. The crew specifies
; when to perform the burn (TIG), how far the passive vehicle will travel during
; the transfer (CENTANG), and the desired separation distance (DELTAR).
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
;
; SOR MANEUVER (Stable Orbit Rendezvous):
; The second burn circularizes the active vehicle into the passive vehicle's
; orbit, maintaining the planned separation distance. This burn occurs at the
; intercept point. Parameters from the SOI calculation (travel time DELTTIME
; and intercept time TINT) are reused to ensure consistent geometry.
;
# OUTPUT
#
#	(1)  TRKMKCNT	NUMBER OF MARKS
#	(2)  TTOGO	TIME TO GO
#	(3)  +MGA	MIDDLE GIMBAL ANGLE
# Page 724
#	(4)  DSPTEM1	TIME OF INTERCEPT OF PASSIVE VEHICLE ORBIT
#			(FOR SOI ONLY)
#	(5)  POSTTPI	PERIGEE ALTITUDE OF ACTIVE VEHICLE ORBIT AFTER
#			THE SOI (SOR) MANEUVER
#	(6)  DELVTPI	MAGNITUDE OF DELTA V AT SOI (SOR) TIME
#	(7)  DELVTPF	MAGNITUDE OF DELTA V AT INTERCEPT TIME
#	(8)  DELVLVC	DELTA VELOCITY AT SOI (AND SOR) - LOCAL VERTICAL
#			COORDINATES
;
; PROGRAM OUTPUTS TO CREW:
; The programs compute and display critical maneuver parameters:
; - DELVTPI: The total velocity change magnitude required at burn time
; - DELVLVC: The delta V vector components in local vertical coordinates
;   (radial, downrange, and out-of-plane directions relative to orbit)
; - POSTTPI: Resulting perigee altitude after the maneuver (crew safety check)
; - DSPTEM1: Predicted intercept time (SOI phase only)
; - TTOGO: Countdown timer to maneuver ignition
; - +MGA: Spacecraft attitude (middle gimbal angle) for proper burn orientation
;
#
# SUBROUTINE USED
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

		BANK	04
		SETLOC	STBLEORB
		BANK

		EBANK=	SUBEXIT
		COUNT*	$$/P3879

; ============================================================================
; TRANSITION: Program P38/P78 Entry Points
;
; The crew has initiated a stable orbit rendezvous calculation through the
; DSKY (V37E38E for P38 or V37E78E for P78). The AGC now determines which
; spacecraft is the active vehicle performing the maneuver and which is the
; passive target vehicle being intercepted. P38 assumes this LM is active;
; P78 assumes the CSM is active and this LM is passive.
; ============================================================================

P38		TC	BANKCALL
		CADR	AVFLAGA		# THIS VEHICLE ACTIVE
; Call AVFLAGA to set active vehicle flags for this LM. This establishes
; which spacecraft performs the delta V burn and which is intercepted.

		TC	+3
P78		TC	BANKCALL
		CADR	AVFLAGP		# OTHER VEHICLE ACTIVE
; Call AVFLAGP to set passive vehicle flags for this LM. The CSM will perform
; the delta V burn and intercept this LM at the target separation distance.

		TC	BANKCALL
		CADR	P20FLGON	# SET UPDATFLG, TRACKFLG
; Enable navigation state update flag and tracking flag. These flags indicate
; the state vectors should be updated and the rendezvous tracking is active.
; The crew will now be prompted to enter rendezvous parameters through the DSKY.
; First, they must provide the time of ignition (TIG) and the central angle
; that the passive vehicle will traverse during the transfer.

		CAF	DECTWO
		TS	NN
; Set noun number NN to 2 for upcoming display sequence.

		CAF	V06N33SR	# DISPLAY TIG
		TC	VNDSPLY
; Display Verb 06 Noun 33: Request crew input for Time of Ignition (TIG).
; The crew enters the mission elapsed time when the SOI maneuver will occur.

		CAF	V06N55SR	# DISPLAY CENTANG
		TCR	BANKCALL
		CADR	GOFLASHR
; Display Verb 06 Noun 55: Request crew input for Central Angle (CENTANG).
; This is the orbital angle the passive vehicle traverses from TIG to intercept.
; The display flashes, awaiting crew input via the DSKY keypad.

		TCF	GOTOPOOH	# TERMINATE
; If crew presses TERMINATE, abort the program and return to POO (idle).

		TCF	+5		# PROCEED
; If crew presses PROCEED, accept the entered values and continue.

		TCF	-5		# RECYCLE
; If crew presses RECYCLE, return to previous display for data re-entry.

		CAF	THREE		# IMMEDIATE RETURN - BLANK R1, R2
		TCR	BLANKET
; Blank display registers R1 and R2 on immediate return from the routine.
# Page 725
		TCF	ENDOFJOB

; ============================================================================
; TRANSITION: Maneuver Option Selection
;
; The crew must now select which type of stable orbit maneuver to perform:
; Option 1 = SOI (Stable Orbit Insertion) - establishes transfer trajectory
; Option 2 = SOR (Stable Orbit Rendezvous) - completes orbit insertion
; The option code is initially set to 1 and can be changed by the crew.
; ============================================================================

		CAF	FIVE
		TS	OPTION1
; Initialize OPTION1 register to 5 (display format control).

		CAF	ONE
		TS	OPTION2		# OPTION CODE IS SET TO 1
; Set default OPTION2 to 1, indicating SOI (Stable Orbit Insertion) maneuver.
; The crew can change this to 2 for SOR (Stable Orbit Rendezvous).

		CAF	V04N06SR	# DISPLAY OPTION CODE - 1 = SOI, 2 = SOR
		TCR	BANKCALL
		CADR	GOFLASHR
; Display Verb 04 Noun 06: Show current option code (default = 1).
; Crew can change to 2 if performing SOR instead of SOI.
; V04 displays data but allows crew modification via PROCEED.

		TCF	GOTOPOOH	# TERMINATE
; Crew pressed TERMINATE - abort program and return to idle.

		TCF	+5		# PROCEED
; Crew pressed PROCEED - accept the option selection and continue.

		TCF	-5		# RECYCLE
; Crew pressed RECYCLE - return to option display for re-entry.

		CAF	BIT3		# IMMEDIATE RETURN - BLANK R3
		TCR	BLANKET
; Blank display register R3 on immediate return.

		TCF	ENDOFJOB

; The crew has now selected the option. The AGC processes this selection to
; determine the appropriate computation path: SOI or SOR.

		TC	INTPRET
; Enter interpreter mode for option processing calculations.

		SLOAD	SR1
			OPTION2
; Load OPTION2 value (1 or 2) and shift right 1 bit (divide by 2).
; This converts: Option 1 (SOI) → 0, Option 2 (SOR) → 1

		BHIZ	DLOAD
			OPTN1
			TINT
; Branch if result is zero (Option 1/SOI selected) to OPTN1.
; Otherwise (Option 2/SOR selected), load TINT (time of intercept).

		STORE	TINTSOI		# STORE FOR SOR PHASE
; Store TINT as TINTSOI for SOR calculations. SOR uses the intercept time
; computed during the previous SOI phase to calculate final orbit insertion.

		CLRGO
			OPTNSW		# OPTNSW: ON = SOI, OFF = SOR
			JUNCTN1
; Clear OPTNSW flag (OFF = SOR phase) and proceed to junction point.
; OPTNSW tracks which phase is being computed: ON=SOI, OFF=SOR.

OPTN1		SET	CLEAR		# SOI
			OPTNSW
			UPDATFLG
; Option 1 (SOI) was selected. Set OPTNSW flag (ON = SOI phase).
; Clear UPDATFLG to prevent state vector updates during computation.

		CALL
			PREC/TT
; Call PREC/TT routine to compute precision time-to-go calculations and
; trajectory propagation for the selected maneuver option.
		DAD	SET
			TIG
			UPDATFLG
; Add TIG (Time of Ignition) to the computed transfer time from PREC/TT.
; Set UPDATFLG to enable state vector updates for trajectory propagation.

		STORE	TINT		# TI = TIG + TF
; Store result as TINT (Time of Intercept). This is when the active vehicle
; will intercept the passive vehicle's orbit after the transfer maneuver.

		EXIT
; Exit interpreter mode to perform DSKY display operations.

; ============================================================================
; TRANSITION: Crew Input for Delta R and Intercept Time Display
;
; The AGC now prompts the crew to enter DELTA R (desired separation distance
; between vehicles along the passive vehicle's orbit). For SOI maneuvers,
; the computed time of intercept is also displayed for crew awareness.
; ============================================================================

		CAF	V06N57SR	# DISPLAY DELTA R
		TCR	BANKCALL
		CADR	GOFLASHR
; Display Verb 06 Noun 57: Request crew input for DELTA R.
; DELTA R specifies the desired along-track separation distance between the
; active and passive vehicles, measured along the passive vehicle's orbit.

		TCF	GOTOPOOH	# TERMINATE
		TCF	+5		# PROCEED
		TCF	-5		# RECYCLE
; Standard TERMINATE/PROCEED/RECYCLE handling for crew input.

		CAF	SIX		# IMMEDIATE RETURN - BLANK R2, R3
		TCR	BLANKET
; Blank display registers R2 and R3 on immediate return.

		TCF	ENDOFJOB
 +5		EXTEND
		DCA	TINT
		DXCH	DSPTEM1		# FOR DISPLAY
; Load TINT (Time of Intercept) into DSPTEM1 for display to crew.

		CAF	V06N34SR	# DISPLAY TIME OF INTERCEPT
		TC	VNDSPLY
; Display Verb 06 Noun 34: Show the computed Time of Intercept (TINT).
; This informs the crew when the active vehicle will reach the passive
; vehicle's orbital path after executing the SOI maneuver.
		TC	INTPRET
; Re-enter interpreter mode for main trajectory computations.

JUNCTN1		CLEAR	CALL
# Page 726
			P39/79SW
			SELECTMU	# SELECT MU, CLEAR FINALFLG, GO TO VN1645
; Junction point where both SOI and SOR paths converge. Clear P39/79SW flag
; to indicate this is not a midcourse correction program (P39/P79).
; Call SELECTMU to select appropriate gravitational parameter (Earth or Moon)
; based on current sphere of influence, and initialize computation flags.

; ============================================================================
; TRANSITION: Main Trajectory Computation
;
; With all parameters entered, the AGC now computes the required delta-V
; and orbital elements for the stable orbit maneuver. The calculation differs
; slightly between SOI (first insertion) and SOR (final rendezvous) phases.
; Both require precise propagation of the passive vehicle's state to the
; intercept time.
; ============================================================================

RECYCLE		CALL
			PREC/TT
; Call PREC/TT again to recompute precision time-to-go with updated parameters.
; This ensures accurate trajectory propagation for the current orbital geometry.

		BOFF	DLOAD
			OPTNSW
			OPTN2
			TINT
; Check OPTNSW flag: if OFF (SOR phase), branch to OPTN2.
; If ON (SOI phase), load TINT (Time of Intercept) for passive vehicle update.

		STCALL	TDEC1		# PRECISION UPDATE PASSIVE VEHICLE TO
			INTRPVP		# 	INTERCEPT TIME
; Store TINT in TDEC1 and call INTRPVP (Interpret Passive Vehicle Position).
; This propagates the passive vehicle's state vector from current time to the
; intercept time, accounting for orbital perturbations and gravitational effects.
		VLOAD	UNIT
			RATT		# RP/(RP)
; Load passive vehicle position vector RATT and convert to unit vector.
; This gives the direction from Moon (or Earth) center to the passive vehicle.

		PDVL	VXV
			VATT
; Push unit position vector to stack, then load passive vehicle velocity VATT.
; Compute cross product: VP × (RP/|RP|), giving the out-of-plane component.

		ABVAL	NORM		# (VP X RP/(RP))
			X1
; Compute absolute value (magnitude) of the cross product.
; Normalize and store scaling exponent in X1 register.

		PDDL	DDV
			DELTAR
; Push normalized cross product magnitude to stack, then load DELTA R.
; Divide DELTA R by the cross product magnitude.

		SL*			# DELTA R / (VP X RP/RP)
			0 	-7,1
; Scale result using exponent from X1 register.
; This computes the time required to traverse DELTA R at the passive vehicle's
; horizontal velocity component.

		STCALL	DELTTIME	# DELTA T = (RP) DELTA R / (VP X RP)
			JUNCTN2
; Store result as DELTTIME (the time to traverse the separation distance).
; Proceed to JUNCTN2 to compute final target time.

OPTN2		DLOAD	DAD
			TINTSOI
			T
; SOR path: Load TINTSOI (intercept time saved from SOI phase) and add
; the time-to-go (T) computed by PREC/TT.

		STORE	TINT		# TI = TI + TF
; Store updated TINT for SOR maneuver calculations.
; This represents when the SOR burn should occur to achieve rendezvous.

JUNCTN2		DLOAD	DSU
			TINT
			DELTTIME
; Load TINT (time of intercept) and subtract DELTTIME (time to traverse DELTA R).
; This computes when the active vehicle must arrive at the intercept point to
; achieve the desired along-track separation from the passive vehicle.

		STORE	TARGTIME	# TT = TI - DELTA T
; Store result as TARGTIME (target time for the active vehicle).
; This is the key parameter: the active vehicle must reach the intercept point
; at TARGTIME, arriving DELTTIME before the passive vehicle, resulting in the
; crew-specified DELTA R separation distance along the orbital path.

# .... MAINRTNE ....
# SUBROUTINES USED:
#
#	S3435.25
#	PERIAPO1
#	SHIFTR1
#	VNDSPLY
#	BANKCALL
#	GOFLASH
#	GOTOPOOH
#	VN1645

MAINRTNE	STCALL	TDEC1		# PRECISION UPDATE PASSIVE VEHICLE TO
			INTRPVP		#	TARGET TIME
		DLOAD
			TIG
		STORE	INTIME
		SSP	VLOAD
			SUBEXIT
			TEST3979
# Page 727
			RATT
		CALL
			S3435.25
TEST3979	BOFF	BON
			P39/79SW
			MAINRTN1
			FINALFLG
			P39P79
		SET
			UPDATFLG
P39P79		EXIT
		TC	DSPLY81		# FOR P39 AND P79
MAINRTN1	VLOAD	ABVAL
			DELVEET3
		STOVL	DELVTPI		# DELTA V
			VPASS4
		VSU	ABVAL
			VTPRIME
		STOVL	DELVTPF		# DELTA V (FINAL) = V'T - VT
			RACT3
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

# STABLE ORBIT MIDCOURSE PROGRAM (P39 AND P79)
#
# MOD NO -1		LOG SECTION - STABLE ORBIT - P38-P39
# MOD BY RUDNICKI.S	DATE 25JAN68
#
# Page 728
# FUNCTIONAL DESCRIPTION
#
#	P39 AND P79 CALCULATE THE REQUIRED DELTA V AND OTHER INITIAL
#	CONDITIONS REQUIRED BY THE AGC TO MAKE A MIDCOURSE CORRECTION
#	MANEUVER AFTER COMPLETING THE SOI MANEUVER BUT BEFORE MAKING
#	THE SOR MANEUVER.
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
#	(1)  TRKMKCNT	NUMBER OF MARKS.
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

P39		TC	BANKCALL
		CADR	AVFLAGA		# THIS VEHICLE ACTIVE
		EXTEND
		DCA	ATIGINC
		TC	P39/P79A
P79		TC	BANKCALL
		CADR	AVFLAGP		# OTHER VEHICLE ACTIVE
		EXTEND
		DCA	PTIGINC
P39/P79A	DXCH	KT		# TIME TO PREPARE FOR BURN
		TC	BANKCALL
		CADR	P20FLGON	# SET UPDATFLG, TRACKFLG
		TC	INTPRET
# Page 729
		SET	CALL
			P39/79SW
			SELECTMU	# SELECT MU, CLEAR FINALFLG, GO TO VN1645
P39/P79B	RTB	DAD
			LOADTIME
			KT
		STORE	TIG		# TIG = T (PRESENT) + PREPARATION TIME
		STCALL	TDEC1		# PRECISION UPDATE ACTIVE AND PASSIVE
			PRECSET		# 	VEHICLES TO TIG
		CALL
			S34/35.1	# GET UNIT NORMAL
		DLOAD	GOTO
			TARGTIME
			MAINRTNE	# CALCULATE DELTA V AND DELTA V (LV)

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

# .... INTRPVP ....
# SUBROUTINES USED
#
#	CSMPREC
# Page 730
#	LEMPREC

INTRPVP		STQ	BOFF		# PRECISION UPDATE PASSIVE VEHICLE TO
			RTRN		#	TDEC1
			AVFLAG
			OTHERV
		CALL
			CSMPREC
		GOTO
			RTRN
OTHERV		CALL
			LEMPREC
		GOTO
			RTRN

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
V06N33SR	VN	0633
V06N55SR	VN	0655
V04N06SR	VN	0406
V06N57SR	VN	0657
V06N34SR	VN	0634
V06N58SR	VN	0658
V06N81SR	VN	0681
DECTWO		OCT	2

# *** END OF KISSING  .050 ***


