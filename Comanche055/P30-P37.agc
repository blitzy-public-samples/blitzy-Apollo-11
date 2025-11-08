# Copyright:	Public domain.
# Filename:	P30-P37.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	635-648
# Mod history:	2009-05-10 RSB	Adapted from the Colossus249/ file
#				of the same name, using Comanche055 page
#				images.
#		2009-05-20 RSB	Corrected BDV -> BOV.
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
; FILE: P30-P37.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth
;
; TL;DR: External ΔV program suite for burn targeting and maneuver planning
;        including CSI (Coelliptic Sequence Initiation), CDH (Constant Delta
;        Height), and TPI (Terminal Phase Initiation) targeting for rendezvous.
;        Computes required velocity changes for orbit modifications throughout
;        Apollo 11 mission phases including translunar injection corrections,
;        lunar orbit insertion adjustments, and rendezvous maneuvers.
;
; COMMENT-ONLY READERS: These programs calculated the engine burns needed to
;        change orbits and approach other spacecraft during the mission. The
;        computer determined how much to fire the engines and in what direction
;        to achieve the desired orbital changes for rendezvous and navigation.
; CODE-ALONG READERS: Study ΔV targeting algorithms, Lambert problem solutions,
;        two-point boundary value problems for orbital transfers, and conic
;        subroutine integration for trajectory calculations.
; ============================================================================

# Page 635
		BANK	32

		SETLOC	P30S1
		BANK

		EBANK=	+MGA

		COUNT	35/P34

; ============================================================================
; DISPLAY AND TIME-TO-GO COMPUTATION ROUTINES
;
; These routines manage DSKY displays for burn parameters and compute the
; time remaining until Time of Ignition (TIG) for maneuvers. During Apollo 11
; mission operations, crew used these displays to monitor upcoming engine burns
; for orbit changes and rendezvous maneuvers.
; ============================================================================

; DISPMGA - Display MGA (Middle Gimbal Angle) and return to calling routine
; Used in P30 to show spacecraft attitude for the planned burn.
; Exits interpreter mode, computes time-to-go, then returns.
DISPMGA		STQ	EXIT		# USED IN P30

			RGEXIT
		TC	COMPTGO

; DISP45 - Display Verb 16 Noun 45 (apogee/perigee/delta-V time)
; This display shows the astronaut the results of trajectory calculations:
; orbital apogee altitude, perigee altitude, and time of delta-V application.
; The display flashes, allowing crew to proceed, terminate, or recycle.
DISP45		CAF	V16N45
		TC	BANKCALL
		CADR	GOFLASHR
		TC	GOTOPOOH	; Terminate: Return to POO (no program)
		TC	END45		; Proceed: Accept values and continue
		TC	DISP45		; Recycle: Display again
P30PHSI		TC	PHASCHNG
		OCT	14
		TCR	ENDOFJOB
END45		TC	INTPRET
		CLEAR	GOTO
			TIMRFLAG
			RGEXIT

; COMPTGO - Compute Time-To-Go until TIG (Time of Ignition)
; This routine sets up a continuous countdown display showing the astronaut
; how many minutes and seconds remain until the next engine burn. The CLOKTASK
; (clock task) runs every second to update the displayed countdown.
; During Apollo 11 rendezvous operations, this countdown was critical for
; crew situational awareness before major maneuvers.
COMPTGO		EXTEND			# USED TO COMPUTE TTOGO
		QXCH	PHSPRDT6	# ** GROUP 6 TEMPORARY USED .. BEWARE **

		TC	UPFLAG		# SET TIMRFLAG
		ADRES	TIMRFLAG	# BIT 11 FLAG 7 - indicates countdown active
		CAF	ZERO
		TS	NVWORD1

		CAF	ONE		; Schedule CLOKTASK to run every second
		TC	WAITLIST
		EBANK=	TIG
		2CADR	CLOKTASK

		TC	2PHSCHNG
		OCT	40036		# 6.3SPOT FOR CLOKTASK
		OCT	05024		# GROUP 4 CONTINUES HERE
		OCT	13000

		TC	PHSPRDT6
# Page 636
; ============================================================================
; PROGRAM DESCRIPTIONS: P30 AND P31
;
; These are two of the key mission programs for orbital maneuvering during
; Apollo 11. Both programs compute engine burn parameters but use different
; input methods and targeting strategies.
; ============================================================================

# PROGRAM DESCRIPTION P30	DATE 3-6-67

# MOD.I BY S. ZELDIN-  TO ADD P31 AND AD APT P30 FOR P31 USE.	22DEC67
# FUNCTIONAL DESCRIPTION
; P30: EXTERNAL DELTA-V TARGETING PROGRAM
; 
; This program allows the astronaut to manually specify the exact velocity
; change (delta-V) vector they want to apply at a given time. The computer
; then calculates and displays the resulting orbit after that burn.
;
; Inputs from astronaut via DSKY:
;   - TIG (Time of Ignition): When the burn should occur
;   - DELV(LV): Desired velocity change in local vertical coordinates
;
; Outputs displayed to astronaut:
;   - Apogee altitude (highest point of resulting orbit)
;   - Perigee altitude (lowest point of resulting orbit)  
;   - DELV(MAG): Magnitude of velocity change (burn duration indicator)
;   - MGA: Middle Gimbal Angle (spacecraft attitude for burn)
;
; Mission usage: P30 was used when Mission Control uplinked specific delta-V
; values for trajectory corrections during translunar coast and lunar orbit.
# +30(EXTERNAL DELTA-V TARGETTING PROGRAM)
#	ACCEPTS ASTRONAUT INPUTS OF TIG,DELV(LV) AND COMPUTES,FOR DISPLAY,
#	APOGEE,PERIGEE,DELV(MAG),MGA ASSOCIATED WITH DESIRED MANEUVER

; P31: GENERAL LAMBERT AIMPOINT GUIDANCE
;
; This program solves the Lambert problem: given a starting position, target
; position, and transfer time, compute the required velocity to reach the
; target. This is the fundamental two-point boundary value problem for
; orbital rendezvous.
;
; Constraint: The angle between target vector and position vector at TIG
; must not be 165-195 degrees (near-backflip geometry causes solution issues).
;
; Inputs (stored by other programs + astronaut entry):
;   - Offset target position (B+29): Where to aim relative to target
;   - Delta-T transfer: How long the transfer should take
;   - TIG (from astronaut): When to begin the transfer
;
; Outputs displayed to astronaut:
;   - Required velocity for maneuver (solved from Lambert problem)
;   - Apogee altitude of transfer orbit
;   - Perigee altitude of transfer orbit
;   - DELV(MAG): Magnitude of required velocity change
;   - MGA: Middle Gimbal Angle for burn attitude
;
; Mission usage: P31 was used during rendezvous planning to compute the burns
; needed for CSI (Coelliptic Sequence Initiation), CDH (Constant Delta Height),
; and TPI (Terminal Phase Initiation) maneuvers during lunar orbit rendezvous.
# P31(GENERAL LAMBERT AIMPOINT GUIDANCE)
# A GROUND RULE FOR P31 IS THE ANGLE BETWEEN THE TARGET VECTOR AND
# POSITION VECTOR AT TIG IS NOT 165-195 DEGREES APART
#	BASED ON STORED INPUT OF OFFSET TARGET(B+29) AND DELTA T TRANS,AND
#	ASTRONAUT ENTRY OF TIG,P31 COMPUTES REQUIRED VELOCITY FOR MANEUVER
#	AND,FOR DISPLAY,APOGEE,PERIGEE,DELV(7AG),+MGA ASSOCIATED WITH
#	DESIRED MANEUVER

; Subroutine architecture:
; - S30.1: Used by P30 to compute orbital parameters from given delta-V
; - S31.1: Used by P31 to solve Lambert problem and compute required delta-V
; - Both programs share display routines for TIG, apogee, perigee, MGA
# THE FOLLOWING SUBROUTINES ARE USED IN P30 AND P31
#	S30.1 (P30 ONLY)
#	S31.1 (P31 ONLY)
; Display subroutines shared by P30 and P31:
;   - P30/P31: Displays TIG (Time of Ignition) to astronaut
;   - CNTUP30: Displays DELV(LV) - velocity change in local vertical coords
;   - PARAM30: Displays apogee, perigee, delta-V magnitude, MGA (middle gimbal
;              angle), time from TIG, and marks since last thrusting maneuver
#	P30/P31 - DISPLAYS TIG
#	CNTUP30 - DISPLAYS DELV(LV)
#	PARAM30 - DISPLAYS APOGEE,PERIGEE,DELV(MAG),MGA,TIME FROM TIG,
#		  MARKS SINCE LAST THRUSTING MANEUVER

# CALLING SEQUENCE VIA JOB FROM V37

# EXIT VIA V37 OR GOTOPOOH

; Output variables set by these programs for powered flight routines:
;   VTIG: Velocity at Time of Ignition
;   RTIG: Position at Time of Ignition  
;   DELVSIN: Delta-V in stable member coordinates
;   VGDISP: Velocity to be gained (for displays)
;   RTARG: Target radius vector (for P31 Lambert targeting)
;   TPASS4: Transfer time (for P31 Lambert targeting)
# OUTPUT FOR POWERED FLIGHT
#	VTIG	X
#	RTIG	 XSEE S30.1
#	DELVSIN	 X
#	VGDISP
#	RTARG	X
#	TPASS4	 X SEE S31.1
#		X


		COUNT	35/P30

; ============================================================================
; P30 MAIN PROGRAM ENTRY
;
; External delta-V targeting: Astronaut specifies desired velocity change,
; computer calculates resulting orbit. Used when Mission Control provides
; specific delta-V vectors for trajectory corrections.
; ============================================================================
P30		TC	P30/P31		; Get TIG from astronaut (shared routine)
		TC	CNTNUP30	; Get delta-V components from astronaut
		TC	DOWNFLAG	; RESET UPDATFLG
		ADRES	UPDATFLG	; BIT 7  FLAG 1
		TC	INTPRET		; Enter interpreter for trajectory calculations
		CALL
			S30.1		; Compute apogee/perigee from given delta-V
		EXIT
		TC	PARAM30		; Display results to astronaut
		TC	UPFLAG
# Page 637
		ADRES	XDELVFLG	; SET XDELVFLG BIT 8 FLAG 2 - external delta-V
		TCF	GOTOPOOH	; Return to POO (no program)

; ============================================================================
; P31 MAIN PROGRAM ENTRY
;
; Lambert aimpoint guidance: Astronaut specifies target and time, computer
; solves for required velocity change. Used for rendezvous maneuver planning
; (CSI, CDH, TPI burns) during lunar orbit operations.
; ============================================================================
P31		TC	P30/P31		; Get TIG from astronaut (shared routine)
		TC	DOWNFLAG
		ADRES	UPDATFLG	; RESET UPDATFLG BIT 7 FLAG 1
		TC	DOWNFLAG
		ADRES	NORMSW		; RESET NORMSW BIT 10 FLAG 7
		TC	INTPRET		; Enter interpreter for Lambert problem
		CALL
			S31.1		; Solve Lambert problem for required delta-V
		EXIT
		TC	CNTNUP30	; Display computed delta-V to astronaut
		TC	PARAM30		; Display apogee/perigee results
		TC	DOWNFLAG
		ADRES	XDELVFLG	; BIT 8 FLAG 2
		TCF	GOTOPOOH	; Return to POO (no program)

; ============================================================================
; P30/P31: SHARED SUBROUTINE - TIG DISPLAY AND INPUT
;
; This subroutine prompts the astronaut to enter the Time of Ignition (TIG)
; via the DSKY using Verb 06 Noun 33. Both P30 and P31 use this routine since
; both require the astronaut to specify when the burn should occur.
; ============================================================================
P30/P31		XCH	Q		; Save return address
		TS	P30/31RT
		TC	UPFLAG
		ADRES	UPDATFLG	; SET UPDATFLG BIT 7 FLAG 1
		TC	UPFLAG
		ADRES	TRACKFLG	; SET TRACKFLG BIT 5 FLAG 1
		CAF	V06N33		; Display V06N33: "Please load" TIG
		TC	BANKCALL	; Call display routine
		CADR	GOFLASHR
		TCF	GOTOPOOH	; Terminate pressed - exit program
		TC	P30/31RT	; Proceed pressed - return to caller
		TCF	P30/P31 +4	; Enter pressed - re-display
		TC	PHASCHNG	; Phase change for restart protection
		OCT	00014
		TC	ENDOFJOB

; ============================================================================
; CNTNUP30: DISPLAY AND INPUT DELTA-V COMPONENTS
;
; This routine prompts the astronaut to enter the desired delta-V in local
; vertical coordinates via DSKY using Verb 06 Noun 81. P30 uses this to
; get the delta-V from the astronaut; P31 uses it to display the computed
; delta-V after solving the Lambert problem.
; ============================================================================
CNTNUP30	XCH	Q		; Save return address
		TS	P30/RET
		CAF	V06N81		; Display V06N81: Delta-V in local vertical
		TC	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH	; Terminate - exit program
		TC	P30/RET		; Proceed - return to caller
		TCF	CNTNUP30 +2	; Enter - re-display

; ============================================================================
; PARAM30: DISPLAY BURN PARAMETERS
;
; This routine displays the computed burn parameters to the astronaut using
; Verb 06 Noun 42: apogee altitude, perigee altitude, and delta-V magnitude.
; These values allow the astronaut to verify the computed maneuver will
; achieve the desired orbit before committing to the burn.
; ============================================================================
PARAM30		XCH	Q		; Save return address
		TS	P30/31RT
		CAF	V06N42		; Display V06N42: Apogee, Perigee, Delta-V mag
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH	; Terminate - exit program
		TCF	REFTEST		; Proceed - continue to MGA display
# Page 638
		TCF	PARAM30 +2	; Enter - re-display

; ============================================================================
; REFTEST: CHECK REFERENCE COORDINATE SYSTEM STATUS
;
; This routine checks if the REFSMMAT (Reference to Stable Member Matrix) has
; been defined. If so, it computes and displays the Middle Gimbal Angle (MGA)
; which tells the astronaut what spacecraft attitude is needed for the burn.
; If REFSMMAT is not set, it displays -00001 as an indicator.
; ============================================================================
REFTEST		CAF	BIT13		; Check REFSMFLAG (bit 13 of STATE+3)
		MASK	STATE +3	; REFSMFLAG
		EXTEND
		BZF	NOTSET		; REFSMFLAG = 0, branch to NOTSET
		TC	INTPRET		; Enter interpreter
		VLOAD	PUSH		; Load delta-V vector
			DELVSIN
		CALL
			GET+MGA		; Compute Middle Gimbal Angle
		GOTO
			FLASHMGA	; Display MGA to astronaut
NOTSET		EXTEND			; REFSMMAT not defined
		DCS	MARSDP		; Load -00001 indicator
		DXCH	+MGA		; Store -00001 in +MGA, +MGA+1
		TC	INTPRET
FLASHMGA	CALL
			DISPMGA		; Display MGA (or -00001) to astronaut
		EXIT
		TC	P30/31RT	; Return to caller
MARSDP		OCT	00000		; Constant -00001 in double precision
		OCT	35100		; Low order: 0.01 degrees resolution
					# ( .01 ) DEGREES IN THE LOW ORDER REGISTE

V06N33		VN	0633
V06N42		VN	0642
V16N35		VN	1635
V06N45		VN	0645

# Page 639
# PROGRAM DESCRIPTION S30.1	DATE 9NOV66

# MOD NO 1			LOG SECTION P30,P37
# MOD BY RAMA AIYAWAR **
# MOD.2 BY S.ZELDIN - TO CORRECT MOD.1 FOR COLOSSUS		29DEC67
# FUNCTIONAL DESCRIPTION
#	BASED ON STORED TARGET PARAMETERS(R OF IGNITION(RTIG),V OF
# IGNITION(VTIG),TIME OF IGNITION(TIG),DELV(LV),COMPUTE PERIGEE ALTITUDE
# A+OGEE ALTITUDE AND DELTA-V REQUIRED IN REF. COORDS.(DELVSIN)
# CALLING SEQUENCE
#	L	CALL
#	L+1		S30.1
# NORMAL EXIT MODE
#	AT L+2 OR CALLING SEQUENCE (GOTO L+2)
# SUBROUTINES CALLED
#	THISPREC
#	PERIAPO
# ALARM OR ABORT EXIT MODES
#		NONE
# ERASABLE INITIALIZATION REQUIRED
#	TIG		TIME OF IGNITION	DP	B28CS
#	DELVSLV		SPECIFIED DELTA-V IN LOCAL VERT.
#			COORDS. OF ACTIVE VEHICLE AT
#	TIME OF IGNITION			VCT.	B+7M/CS
#
# OUTPUT
#	RTIG		POSITION AT TIG		VCT. 	B+29M
#	VTIG		VELOCITY AT TIG		VCT. 	B+7M
#	HAPO		APOGEE ALT.		DP 	B+29M
#	HPER		PERIGEE ALT.		DP 	B+29M
#	DELVSIN		DELVSLV IN REF COORDS	VCT. 	B+7M/CS
#	VGDISP		MAG. OF DELVSIN		DP 	B+7M/CS
# DEBRIS	QTEMP	TEMP. ERASABLE
#		QPRET,MPAC
#		PUSHLIST

		SETLOC	P30S1A
		BANK

		COUNT	35/S30S

; ============================================================================
; SUBROUTINE: S30.1 - EXTERNAL DELTA-V COMPUTATION
;
; This subroutine computes the resulting orbit parameters after applying
; a specified velocity change (ΔV) at a given time. Used by P30 to determine
; the outcome of astronaut-specified maneuvers.
;
; The crew inputs the desired ΔV in local vertical coordinates (forward,
; left, up relative to the spacecraft orientation). S30.1 converts this to
; inertial reference coordinates, predicts the spacecraft state at the time
; of ignition (TIG), applies the ΔV, and calculates the resulting apogee
; and perigee altitudes of the new orbit.
;
; This was critical during Apollo 11 for planning trajectory correction burns
; during translunar coast and for verifying orbital characteristics after
; lunar orbit insertion and rendezvous maneuvers.
; ============================================================================

S30.1		STQ	DLOAD
			QTEMP
			TIG		# TIME IGNITION SCALED AT 2(+28)CS
		STCALL	TDEC1
			THISPREC	# ENCKE ROUTINE FOR
; Call THISPREC to propagate current state vector forward to time of ignition.
; THISPREC uses precise Encke orbital integration method accounting for
; gravitational perturbations. Returns updated position (RATT) and velocity
; (VATT) at TIG. Source: CONIC_SUBROUTINES.agc

		VLOAD	SXA,2
			VATT
			RTX2
		STOVL	VTIG
; Store predicted velocity at TIG in VTIG (scaled B+7 meters/centisecond).
; Save index register X2 for coordinate transformation matrix reference.
# Page 640
			RATT
; Store predicted position at TIG in RTIG (scaled B+29 meters). Also store
; in RACT3 for coordinate frame calculations. The RATT vector comes from
; the previous PRECSET/RVBOTH orbit integration, representing where the
; spacecraft will be at the planned Time of Ignition.

		STORE	RTIG
		STORE	RACT3
		VXV	UNIT
			VTIG
		STCALL	UNRM
			LOMAT
; Compute unit normal vector (UNRM) by taking cross product of position and
; velocity vectors. This defines the orbital plane orientation. Call LOMAT
; to construct the local orientation matrix that transforms between spacecraft-
; relative (forward/left/up) and inertial reference coordinates. During Apollo
; 11, this allowed crew to specify ΔV in intuitive "forward/left/up" terms
; while AGC computed actual inertial space velocity changes.
		VLOAD	VXM
			DELVSLV
			0
		VSL1	SXA,1
			RTX1
		STORE	DELVSIN
; Transform astronaut-specified ΔV from local vertical coordinates (DELVSLV)
; to inertial reference coordinates (DELVSIN) using the local orientation
; matrix. The crew inputs ΔV relative to the spacecraft (forward, left, up),
; but trajectory calculations require velocities in inertial frame. VSL1
; shifts left 1 bit for proper scaling. Both scaled B+7 meters/centisecond.
		ABVAL
		STOVL	VGDISP		# MAG DELV
			RTIG
		PDVL	VAD
			DELVSIN
			VTIG
; Compute magnitude of ΔV for crew display (VGDISP). This single number tells
; the astronauts how much total velocity change is required for the maneuver.
; Push position vector RTIG onto stack, then add DELVSIN to pre-burn velocity
; VTIG to calculate post-burn velocity vector for orbital analysis.
		CALL
			PERIAPO1
; Call PERIAPO1 to compute apogee and perigee altitudes of the resulting orbit.
; PERIAPO1 is a conic subroutine that solves Kepler's two-body orbital equations
; given the post-burn position and velocity vectors. Returns orbital parameters
; including apogee radius (4D stack location) and perigee radius (2D stack).
; Source: CONIC_SUBROUTINES.agc

		CALL
			SHIFTR1
		CALL
			MAXCHK
		STODL	HPER		# PERIGEE ALT B+29
			4D
		CALL
			SHIFTR1
		CALL
			MAXCHK
		STCALL	HAPO		# APOGEE ALT B+29
			QTEMP
; SHIFTR1 converts orbital radii to altitudes by subtracting planetary radius
; and scaling to proper display format. MAXCHK limits altitude values to
; prevent DSKY display overflow (clamps to maximum displayable value). Store
; perigee altitude in HPER and apogee altitude in HAPO, both scaled B+29 meters.
; During Apollo 11, these values informed mission control and crew whether
; proposed maneuvers would achieve desired orbital characteristics. Return to
; caller using address saved in QTEMP.

# Page 641
; ============================================================================
; SUBROUTINE: S31.1 (Lambert Targeting for P31)
;
; Computes required ΔV to reach a specified target position (RTARG) at a
; specified time offset (DELLT4) from TIG. This is a two-point boundary
; value problem solver using Lambert's theorem: given position at TIG,
; target position, and transfer time, compute the required velocity vector.
; Used in P31 (General Lambert Aimpoint Guidance) for rendezvous targeting,
; trans-lunar/trans-earth injection planning, and trajectory correction burns.
;
; During Apollo 11's trans-lunar coast, these calculations determined the
; trajectory corrections needed to reach the Moon. For rendezvous operations,
; S31.1 computed velocity changes to intercept Columbia from Eagle.
; ============================================================================

# S31.1 PROGRAM DESCRIPTION		28DEC67
# MOD.1 BY S.ZELDIN

# S31.1 COMPUTES DELV IN REF AND LV COORDS,MAG OF DELV,INTERCEPT TIME,
# APOGEE AND PERIGEE ALT FOR REQUIRED MANEUVER

# CALLING SEQUENCE
#	L	CALL
#	L+1		S31.1

# NORMAL EXIT MODE
#	AT L +2 OF CALLING SEQUENCE(GOTO L+2)
# SUBROUTINES CALLED
#	AGAIN
#	PERIAPO1
#	SHIFTR1
#	MIDGIM
# NO ALARM OR ABORT MODES
# INPUT
#	DELLT4		DP	+28
#	TIG		DP     +28
#	RTARG		VCT	+29
# OUTPUT
#	DELVLVC		VCT	+7
#	VGDISP		DP	+7
#	HAPO		DP	+29
#	HPER		DP	+29
#	TPASS4		DP	+28
# DEBRIS - QTEMP

S31.1		STQ	DLOAD
			QTEMP
			TIG
; Lambert targeting solution begins. Save return address in QTEMP. Load TIG
; (Time of Ignition) which marks the start of the transfer trajectory.
		STCALL	TDEC1
			AGAIN		# RETURNS RTX2,RTX1,RATT,VATT,VIPRIME
; Store TIG in TDEC1 and call AGAIN subroutine, which performs Lambert targeting
; computation. AGAIN solves the two-point boundary value problem: given current
; position (RATT), target position (RTARG), and time-of-flight (DELLT4), compute
; the required initial velocity (VIPRIME) to reach target. Also returns updated
; position/velocity at TIG. This is TIME_OF_FREE_FALL.agc Lambert solver.
		VLOAD	PDVL		# DELUEET3
			RTIG
			VIPRIME
; Load position vector RTIG and push to stack, then load computed velocity
; VIPRIME (the Lambert solution velocity). These define the state vector at
; TIG that will achieve the desired intercept trajectory.

		CALL
			PERIAPO1
; Call PERIAPO1 (from CONIC_SUBROUTINES.agc) to compute resulting orbit's
; apogee and perigee. This verifies the Lambert solution produces a valid
; orbit and provides orbital parameters for crew/ground display. Critical
; for ensuring trajectory safety margins.

		CALL
			SHIFTR1
		CALL
			MAXCHK
		STODL	HPER		# B29
			4D
		CALL
			SHIFTR1
		CALL
			MAXCHK
		STOVL	HAPO		# B29
; Convert orbital radii to altitudes (SHIFTR1) and apply display limits
; (MAXCHK). Store perigee altitude in HPER and apogee altitude in HAPO,
; both scaled B+29 meters. During Apollo 11's trajectory corrections,
; these values confirmed burns would not result in collision course or
; unsafe lunar approach trajectory.
# Page 642
			DELVEET3
		STORE	0
		SET	CALL
			AVFLAG
			MIDGIM		# GET DELVLVC B7 FORDISPLAY
; Load DELVEET3 (the required ΔV in inertial coordinates, computed as difference
; between Lambert solution velocity VIPRIME and current velocity VATT). Store
; at location 0 for MIDGIM processing. Set AVFLAG and call MIDGIM to transform
; ΔV from inertial coordinates to local vertical coordinates (DELVLVC). The crew
; needs ΔV in forward/left/up format for manual backup or monitoring, scaled B+7.

		ABVAL
		STODL	VGDISP		# B+7 FOR DISPLAY
; Compute magnitude of ΔV vector (ABVAL) and store in VGDISP for crew display.
; This single number tells astronauts total velocity change required. During
; Apollo 11 mid-course corrections, typically showed values of a few meters/sec.

			DELLT4
		DAD
			TIG
		STCALL	TPASS4		# FOR S40.1
			QTEMP
; Compute intercept time by adding transfer time (DELLT4) to TIG. Store result
; in TPASS4 for subsequent program use (S40.1 time-to-go display). Return to
; caller using address saved in QTEMP. The Lambert solution is now complete:
; crew has ΔV magnitude, direction, timing, and resulting orbital parameters.

# Page 643
# SUBROUTINE NAME:	DELRSPL		(CONTINUATION OF V 82 IN CSM IF P11 ACTI
# TRANSFERRED COMPLETELY FROM SUNDISK, P30S REV 33.  9 SEPT 67.
# MOD NO: 0		MOD BY: ZELDIN		DATE:
# MOD NO: 1		MOD BY: RR BAIRNSFATHER	DATE:  11 APR 67
# MOD NO: 2		MOD BY: RR BAIRNSFATHER	DATE:  12 MAY 67	ADD UR.RT CALC WHEN BELOW 300K FT
# MOD NO: 2.1		MOD BY: RR BAIRNSFATHER	DATE: 5 JULY 67		FIX ERROR ON MOD. 2.
# MOD NO: 3		MOD BY: RR BAIRNSFATHER	DATE:  12 JUL 67	CHANGE SIGN OF DISPLAYED ERROR.
# MOD 4			MOD BY  S.ZELDIN	DATE  3 APRIL 68	CHANGE EQUATIONS FOR L/D=.18 WHICH REPLA
# FUNCTION:		CALCULATE (FOR DISPLAY ON CALL) AN APPROXIMATE MEASURE OF IN-PLANE SPLASH DOWN
#			ERROR.  IF THE FREE-FALL TRANSFER ANGLE TO 300K FT ABOVE PAD RADIUS IS POSITIVE:
#			SPLASH ERROR= -RANGE TO TARGET + FREE-FALL TRANSFER ANGLE + ESTIMATED ENTRY ANGLE.
#			THE TARGET LOCATION AT ESTIMATED TIME OF IMPACT IS USED.  IF THE FREE-FALL TRANSFER
#			ANGLE IS NEGATIVE:  SPASH ERROR= -RANGE TO TARGET
#			THE PRESENT TARGET LOCATION IS USED.
# CALLING SEQUENCE	CALLED AFTER SR30.1 IF IN CSM AND IF P11 OPERATING (UNDER CONTROL OF V82)
# SUBROUTINES CALLED:  VGAMCALC, TFF/TRIG, LALOTORV.
# EXIT:			RETURN DIRECTLY TO V 82 PROG. AT SPLRET
# ERASABLE INITIALIZATION  LEFT BY SR30.1 AND V82GON1
# OUTPUT:	RSP-RREC RANGE IN REVOLUTIONS.  			DSKY DISPLAY IN N. MI.
# DEBRIS:	QPRET, PDL0 ...PDL7 ,PDL10

# 		THETA(1)

		BANK	32
		SETLOC	DELRSPL1
		BANK
		COUNT*	$$/P30		# PROGRAMS: P30 EXTERNAL DELTA V

; ============================================================================
; DELRSPL - Splashdown Error Calculation for Reentry Planning
;
; COMMENT-ONLY READERS: This routine helped mission planners ensure the
; Command Module would splash down in the correct ocean recovery area after
; returning from the Moon. It calculated how far off-target the spacecraft
; would land if the proposed reentry trajectory was executed. For Apollo 11,
; accurate splashdown prediction was critical to position Navy recovery
; ships in the Pacific Ocean on July 24, 1969.
;
; CODE-ALONG READERS: This subroutine computes an approximate measure of
; in-plane splashdown error by checking altitude constraints (must be
; below 300,000 feet), calling VGAMCALC for reentry angle calculations,
; using TFF/TRIG for time-of-free-fall computations, and calculating
; angular separation between predicted and target landing sites.
; Input: 8D register contains altitude parameter
; Output: Angular splashdown error measure
; ============================================================================

; Altitude Constraint Check
; The spacecraft must be below 300,000 feet altitude for valid reentry
; trajectory calculation. This altitude represents the entry interface
; where aerodynamic forces become significant.

DELRSPL		STORE	8D
		BPL	DSU
			CANTDO		# GONE PAST 300K FT ALT
			1BITDP
		BOV	CALL
			CANTDO		# POSMAX INDICATES NO 300K FT SOLUTION.
			VGAMCALC	# +GAMMA(REV) IN PMAC,V300 MAG(B-7)=PDL 0
; Reentry Trajectory Time Calculation
; TFF/TRIG computes time-of-free-fall for the reentry trajectory,
; accounting for gravitational forces and atmospheric entry dynamics.
; AUGEKUGL then converts target latitude/longitude to position vector.

		PUSH	CALL
			TFF/TRIG
		CALL
			AUGEKUGL
; Angular Splashdown Error Computation
; Calculate the angular separation between predicted and target splashdown
; locations. ACOS converts the dot product (CDELF/2) to an angle, which
; represents the arc distance between the two splashdown points on Earth's
; surface. This angle is stored in THETA(1) for error display.

		PDDL	ACOS		# T ENTRY PDL 6
			CDELF/2
		DAD
			4
GETARG		STOVL	THETA(1)
			LAT(SPL)
		STODL	LAT
			HI6ZEROS
		STODL	ALT		# ALT=0 = LAT +4
			PIPTIME
# Page 644
; Target Position Vector Conversion
; Convert target splashdown latitude/longitude (LAT(SPL)) to a position
; vector using LALOTORV. This allows angular separation calculation by
; taking the dot product of unit vectors. The time adjustment accounts
; for when the calculation was initiated versus current mission time.

		BON	DLOAD
			V37FLAG
			+2
			TSTART82
		DSU	DAD
			8D
		CLEAR	CALL
			ERADFLAG
			LALOTORV	# R RECOV. IN ALPHAV AND MPAC

; Final Splashdown Error Calculation
; Compute unit vectors for both predicted and target positions (RONE is
; the target recovery position). The dot product gives the cosine of the
; angular separation. ARCCOS converts this to the actual angle.
; ERROR = THETA(estimated) - THETA(target)
; Negative error: spacecraft will fall short of target (undershoot)
; Positive error: spacecraft will overshoot target beyond recovery zone
; This error value helps mission control adjust the reentry trajectory
; to ensure splashdown within the designated recovery area.

		UNIT	PDVL
			RONE
		UNIT	DOT
		SL1	ARCCOS
		BDSU			# ERROR = THETA EST - THETA TARG
					# NEGATIVE NUMBER SIGNIFIES THAT WILL FALL SHORT.
					# POSITIVE NUMBER SIGNIFIES THAT WILL OVERSHOOT.
			THETA(1)
DELRDONE	STCALL	RSP-RREC	# DOWNRANGE RECOVERY RANGE ERROR	/360
			INTWAKE0
		CALL
			SPLRET
CANTDO		DLOAD	PDDL		# INITIALIZE ERASE TO DOT TARGET AND UR
					# FOR RANGE ANGLE.
			HIDPHALF	# TO PDL 0 FOR DEN IN DDV.
			HI6ZEROS
		PUSH			# ZERO TO PDL 2 FOR PHI ENTRY
		STCALL	8D
			GETARG		# GO SET RSP-RREC =0

; ============================================================================
; AUGEKUGL - Entry Trajectory K1/K2 Coefficient Calculator
;
; COMMENT-ONLY READERS: When Apollo 11 prepared for Earth reentry, this
; routine calculated critical trajectory shape coefficients (K1 and K2)
; that determined how steeply the Command Module would dive through the
; atmosphere. Too shallow and the capsule would skip back into space; too
; steep and the crew would experience crushing deceleration forces. These
; coefficients were essential for safe atmospheric entry planning.
;
; CODE-ALONG READERS: AUGEKUGL performs velocity-based table lookup and
; interpolation to compute K1 and K2 trajectory shape coefficients for
; reentry guidance. Uses indexed comparison against velocity thresholds
; (21,000 fps, 3,000 fps, 4,000 fps, 400 fps) to select appropriate
; coefficient ranges from CK1K2 and YK1K2 tables. Also calculates PHI
; entry angle parameter with maximum constraint checking (2000 NM).
; Called from DELRSPL during transearth trajectory planning.
; ============================================================================

AUGEKUGL	VLOAD
			X1CON -2
		STODL	X1 -2
			0
; --- Velocity Range Classification ---
; Compare entry velocity against four critical thresholds to select
; appropriate K1/K2 coefficient range from lookup tables. Each velocity
; range represents different atmospheric entry corridor characteristics.
; V(21K) = 21,000 fps: High-speed direct entry threshold
; V(3K) = 3,000 fps: Mid-range entry velocity
; V(4K) = 4,000 fps: Moderate entry velocity  
; V(400) = 400 fps: Low-speed approach threshold
		DSU	BMN			# Compare against 21,000 fps threshold
			V(21K)
			LOOPSET
		XSU,1	XCHX,2
			S1
			X1
		XCHX,2	DSU
			S1
			V(3K)
		BMN	XCHX,2
			LOOPSET
			S1
		DSU	BMN
			V(4K)
			LOOPSET
		XCHX,2	XCHX,2
# Page 645
			S1
			X1
		DSU	BMN
			V(400)
			LOOPSET
		SXA,1
			S1
LOOPSET		INCR,1	GOTO
		DEC	1
			K1K2LOOP
; --- K1 and K2 Coefficient Calculation ---
; Using selected velocity range index, interpolate K1 and K2 values from
; coefficient tables. These coefficients define entry trajectory curvature:
; K1 affects lift-to-drag ratio modulation during entry
; K2 determines bank angle profile for range and crossrange control
; Linear interpolation formula: K = Y * (V - Vref) + C
; where Y and C values come from indexed YK1K2 and CK1K2 tables
K2CALC		SXA,1
			S1
K1K2LOOP	DLOAD	DSU*
			0
			V(32K) +1,1		# Subtract reference velocity
		DMP*	DAD*
			YK1K2 +1,1		# Multiply by slope coefficient
			CK1K2 +1,1		# Add constant offset
		PDDL	TIX,1
			2
			K2CALC
		DSU	BDDV
		PUSH	BOV
			MAXPHI
		BMN	DSU
			MAXPHI
			MAXPHIC
		BPL
			MAXPHI
; --- PHI Entry Angle Calculation ---
; Compute PHI, the flight path angle at entry interface (400,000 feet
; altitude). PHI determines initial entry trajectory steepness. Apollo 11
; targeted approximately -6.5 degrees for safe corridor entry. Different
; velocity regimes (above/below 26,000 fps) use different calculation
; methods due to varying atmospheric interaction physics.
PHICALC		DLOAD	DSU		# PHI ENTRY PDL 4D
			0
			V(26K)			# Compare velocity to 26,000 fps
		BPL	DLOAD
			TGR26			# Use high-velocity formula
			TLESS26			# Use low-velocity formula
		DDV
			0
TENT		DMP	RVQ			# Return PHI value to caller
			4D
TGR26		DLOAD	GOTO
			TGR26CON
			TENT

; --- PHI Maximum Constraint Handler ---
; If calculated PHI exceeds safe limits, apply maximum constraint of
; 2000 nautical miles splashdown range equivalent (about 6.49 degrees
; entry angle). Prevents trajectory calculations from producing unsafe
; entry corridors that could endanger crew or miss recovery area.
MAXPHI		DLOAD	PDDL
			MAXPHIC
		GOTO
			PHICALC
MAXPHIC		2DEC	.09259298	# 2000 NM FOR MAXIMUM PHI ENTRY

# Page 646

		COUNT*	$$/P30

; ============================================================================
; K1/K2 Coefficient Lookup Tables
;
; These tables contain trajectory shape coefficients derived from extensive
; atmospheric entry simulations and flight test data. Each row corresponds
; to specific velocity ranges determined by AUGEKUGL's comparison logic.
; Tables are indexed - DO NOT reorder entries without updating index logic.
;
; V(32K) table: Reference velocities for interpolation (fps)
; YK1K2 table: Slope coefficients for linear interpolation
; CK1K2 table: Constant offsets for coefficient calculation
; ============================================================================
					# 		BELOW
					# <<<< TABLE IS INDEXED. KEEP IN ORDER >>>>

		2DEC	7.07304526 E-4	# 5500

		2DEC	3.08641975 E-4	# 2400

		2DEC	3.08641975 E-4	# 2400

		2DEC	-8.8888888 E-3	# -3.2

		2DEC	2.7777777 E-3	# 1

CK1K2		2DEC	6.6666666 E-3	# 2.4

		2DEC	0		# 0

		2DEC*	-1.86909989 E-5 B7* 	# -.443

		2DEC	0

		2DEC*	1.11639691 E-3 B7*	# .001225

		2DEC*	9.56911636 E-4 B7*	# .00105

YK1K2		2DEC*	2.59733157 E-4 B7*	# 	.000285

V(400)		2DEC	1.2192 B-7

V(28K)		2DEC	85.344 B-7

V(3K)		2DEC	9.144 B-7

V(24K)		2DEC	73.152 B-7

		2DEC	85.344 B-7

V(32K)		2DEC	97.536 B-7

V(4K)		2DEC	12.192 B-7

V(21K)		2DEC	64.000 B-7

TLESS26		2DEC*	5.70146688 E7 B-35*	# 8660PHI/V

TGR26CON	2DEC	7.2 E5 B-28	# PHI/3

V(26K)		2DEC	79.248 B-7	# 26000

# Page 647

X1CON		DEC	10

		DEC	8

		DEC	6
					# <<<< TABLE IS INDEXED.  KEEP IN ORDER >>>>
					#		ABOVE
# Page 648
# ***** AVFLAGA/P *****

; ============================================================================
; AVFLAGA/P - Active Vehicle Flag Management
;
; COMMENT-ONLY READERS: During Apollo 11's mission, the spacecraft consisted
; of two vehicles: the Command/Service Module (CSM) "Columbia" and the Lunar
; Module (LM) "Eagle". These routines told the guidance computer which
; vehicle was currently performing maneuvers. This was critical because the
; two vehicles had different masses, engine capabilities, and control systems.
; When Armstrong and Aldrin piloted Eagle to the lunar surface, the computer
; needed to know it was controlling the LM, not the CSM.
;
; CODE-ALONG READERS: AVFLAGA and AVFLAGP manage the AVFLAG (Active Vehicle
; Flag, bit 5 of flag word 2) to identify which spacecraft is executing
; guidance programs. AVFLAGA clears the flag and sets ECSTEER=1 for CSM mode
; (Command/Service Module). AVFLAGP sets the flag for LM mode (Lunar Module).
; Flag state determines which mass properties, engine parameters, and control
; gains are used throughout guidance and navigation computations.
; ============================================================================

# SUBROUTINES USED

#	UPFLAG
#	DOWNFLAG
		SETLOC	P30SUBS
		BANK
		EBANK=	SUBEXIT
; --- AVFLAGA: Set Active Vehicle to Command/Service Module ---
; Clears AVFLAG to indicate CSM is the active vehicle for maneuver
; execution. Also initializes ECSTEER steering parameter to 1 for CSM
; control system configuration. Used during translunar, lunar orbit, and
; transearth mission phases when Columbia was the active spacecraft.
AVFLAGA		EXTEND			# AVFLAG = CSM
		QXCH	SUBEXIT		; Save return address from Q register
		TC	DOWNFLAG	; Clear AVFLAG (bit 5 of flag word 2)
		ADRES	AVFLAG		# BIT 5 FLAG 2
; After clearing AVFLAG, the computer now knows the Command/Service Module
; is the active vehicle. Next, set the ECSTEER parameter to 1 to configure
; external conics steering for CSM mass properties and control characteristics.
		CAF	EBANK7		; Switch to memory bank 7
		TS	EBANK		; to access ECSTEER variable
		EBANK=	ECSTEER
		CAF	BIT13		; Load constant 1 (BIT13 scaled)
		TS	ECSTEER		# SET ECSTEER = 1
; Restore memory bank and return to caller.
		CAF	EBANK4		; Switch back to bank 4
		TS	EBANK		; for SUBEXIT access
		EBANK=	SUBEXIT
		TC	SUBEXIT		; Return to caller via saved address
; --- AVFLAGP: Set Active Vehicle to Lunar Module ---
; Sets AVFLAG to indicate LM is the active vehicle for maneuver execution.
; Used during LM separation, descent, landing, surface operations, ascent, and
; rendezvous phases when Eagle was the active spacecraft. During Apollo 11,
; this was set when Armstrong and Aldrin piloted Eagle to the Moon's surface.
AVFLAGP		EXTEND			# AVFLAG = LEM
		QXCH	SUBEXIT		; Save return address from Q register
		TC	UPFLAG		; Set AVFLAG (bit 5 of flag word 2)
		ADRES	AVFLAG		# BIT 5 FLAG 2
; With AVFLAG set, guidance computations now use LM mass properties (lighter
; than CSM), LM engine parameters (descent/ascent engines), and LM-specific
; control gains for attitude control and navigation algorithms.
		TC	SUBEXIT		; Return to caller via saved address
; ============================================================================
; P20FLGON - Enable Rendezvous Navigation Tracking and Update Flags
;
; COMMENT-ONLY READERS: During rendezvous operations when the Lunar Module
; Eagle was approaching the Command Module Columbia in lunar orbit, the
; guidance computer needed to continuously track the relative position between
; the two spacecraft. This routine enabled two critical capabilities: tracking
; the other spacecraft's position using radar or optical sightings, and
; automatically updating the navigation state with new measurements to improve
; accuracy as the two vehicles approached for docking.
;
; CODE-ALONG READERS: P20FLGON enables TRACKFLG and UPDATFLG for rendezvous
; navigation programs (P20-P25). TRACKFLG enables continuous tracking of the
; target vehicle, while UPDATFLG enables automatic incorporation of tracking
; measurements into the navigation state vector. These flags coordinate radar
; data processing, optical sighting processing, and state vector updates
; throughout the rendezvous sequence.
; ============================================================================
P20FLGON	EXTEND
		QXCH	SUBEXIT		; Save return address from Q register
		TC	UPFLAG		; Set TRACKFLG to enable target tracking
		ADRES	TRACKFLG	; (rendezvous radar or optical tracking)
		TC	UPFLAG		; Set UPDATFLG to enable automatic
		ADRES	UPDATFLG	; navigation state updates from measurements
; With both flags set, the P20-P25 rendezvous programs can now continuously
; track the target spacecraft and automatically refine navigation accuracy
; as relative position measurements are acquired. Critical during Apollo 11's
; rendezvous when Eagle ascended from the lunar surface to dock with Columbia.
		TC	SUBEXIT		# DP	B4
