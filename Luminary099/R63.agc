# Copyright:	Public domain.
# Filename:	R63.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	338-341
# Mod history:	2009-05-16 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
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
; FILE: R63.agc
; MODULE: LM Display and Monitoring Routines
; MISSION PHASE: lunar-orbit/rendezvous
;
; TL;DR: Implements Verb 89 routine (R63) which calculates and displays FDAI
;        ball angles required to point the Lunar Module's +X or +Z axis
;        toward the Command/Service Module during rendezvous operations. This
;        crew-initiated routine enables visual alignment confirmation before
;        executing automated maneuvers for docking preparation.
;
; COMMENT-ONLY READERS: This routine helps astronauts visually align the LM
;        with the CSM during rendezvous by computing the required spacecraft
;        attitude and displaying it on the FDAI (Flight Director Attitude
;        Indicator) ball.
; CODE-ALONG READERS: Study the integration of state vector updates, line-of-
;        sight vector computation, coordinate transformations via VECPOINT,
;        and DSKY display interface for crew-interactive attitude targeting.
; ============================================================================

# Page 338
; ============================================================================
; RENDEZVOUS ALIGNMENT DISPLAY ROUTINE (V89CALL / R63)
;
; During rendezvous operations in lunar orbit, the crew needs to visually
; confirm the LM's orientation relative to the CSM before executing docking
; maneuvers. This routine computes the precise spacecraft attitude required
; to point either the LM's docking axis (+Z) or approach axis (+X) directly
; at the Command Module, then displays the required FDAI ball angles.
;
; The astronauts initiate this routine by keying "V 89 E" (Verb 89 Enter)
; on the DSKY while in Program 00 (P00) standby mode. The AGC calculates
; the line-of-sight vector from LM to CSM, determines required gimbal angles,
; and offers the option to automatically maneuver to the computed attitude.
; ============================================================================

# SUBROUTINE NAME:	V89CALL
# MOD NO:	0			DATE:		9 JAN 1968
# MOD BY:	DIGITAL DEVEL GROUP	LOG SECTION:	R63
#
# FUNCTIONAL DESCRIPTION:
#
# CALLED BY VERB 89 ENTER DURING P00.  PRIO 10 USED.  CALCULATES AND
# DISPLAYS FINAL FDAI BALL ANGLES TO POINT LM +X OR +Z AXIS AT CSM.
#
# 1. KEY IN V 89 E ONLY IF IN PROG 00.  IF NOT IN P00, OPERATOR ERROR AND
# EXIT R63, OTHERWISE CONTINUE.
#
# 2. IF IN P00, DO IMU STATUS CHECK ROUTINE (R02BOTH).  IF IMU ON AND ITS
# ORIENTATION KNOWN TO LGC, CONTINUE.
#
# 3. FLASH DISPLAY V 04 N 06.  R2 INDICATES WHICH SPACECRAFT AXIS IS TO
# BE POINTED AT CSM.  INITIAL CHOICE IS PREFERRED (+Z) AXIS (R2=1).
# ASTRONAUT CAN CHANGE TO (+X) AXIS (R2 NOT = 1) BY V 22 E 2 E.  CONTINUE
# AFTER KEYING IN PROCEED.
#
# 4. BOTH VEHICLE STATE VECTORS UPDATED BY CONIC EQS.
#
# 5. HALF MAGNITUDE UNIT LOS VECTOR (IN STABLE MEMBER COORDINATES) AND
# HALF MAGNITUDE UNIT SPACECRAFT AXIS VECTOR (IN BODY COORDINATES)
# PREPARED FOR VECPOINT.
#
# 6. GIMBAL ANGLES FROM VECPOINT TRANSFORMED INTO FDAI BALL ANGLES BY
# BALLANGS.  FLASH DISPLAY V 06 N 18 AND AWAIT RESPONSE.
#
# 7. RECYCLE - RETURN TO STEP 4.
#    TERMINATE - EXIT R63.
#    PROCEED - RESET 3AXISFLG AND CALL R60LEM FOR ATTITUDE MANEUVER.
#
# CALLING SEQUENCE:	V 89 E.
#
# SUBROUTINES CALLED:	CHKPOOH, R02BOTH, GOXDSPF, CSMCONIC, LEMCONIC,
#			VECPOINT, BALLANGS, R60LEM.
#
# NORMAL EXIT MODES: 	TC ENDEXT
#
# ALARMS:	1. OPERATOR ERROR IF NOT IN P00.
#		2. PROGRAM ALARM IF IMU IS OFF.
#		3. PROGRAM ALARM IF IMU ORIENTATION IS UNKNOWN.
#
# OUTPUT:	NONE
#
# ERASABLE INITIALIZATION REQUIRED:  NONE
#
# DEBRIS:	OPTION1, +1, TDEC1, PDINTVSM, SCAXIS, CPHI, CTHETA, CPSI,
# Page 339
#		3AXISFLG.

		EBANK=	RONE
		BANK	32
		SETLOC	BAWLANGS
		BANK

		COUNT*	$$/R63

; ============================================================================
; V89CALL - MAIN ENTRY POINT FOR RENDEZVOUS ALIGNMENT DISPLAY
;
; The crew has keyed "V 89 E" on the DSKY to request LM-to-CSM alignment
; angles. This routine begins by verifying the IMU (Inertial Measurement
; Unit) is operational and properly aligned, then prompts the astronaut to
; select which spacecraft axis should point toward the CSM.
; ============================================================================

V89CALL		TC	BANKCALL	# IMU STATUS CHECK.  RETURNS IF ORIENTATION
		CADR	R02BOTH		# KNOWN.  ALARMS IF NOT.

; The astronaut can select either the +Z axis (preferred docking axis) or
; the +X axis (alternate approach axis) for CSM alignment. This choice is
; displayed via DSKY and can be modified before proceeding.

		CAF	THREE		# ALLOW ASTRONAUT TO SELECT DESIRED
		TS	OPTIONX		# TRACKING ATTITUDE AXIS.
		CAF	ONE
		TS	OPTIONX  +1	# OPTIONX+1 = 1 INITIALLY (Z AXIS DEFAULT)

; Flash Display V 04 N 12 prompts crew to review/modify axis selection.
; R2 display shows current selection: 1 for +Z axis, 2 for +X axis.
; Crew can change via V 22 E 2 E, then press PROCEED to continue.

		CAF	VB04N12		# V 04 N 12
		TC	BANKCALL
		CADR	GOFLASH		# FLASH DISPLAY AND WAIT FOR RESPONSE
		TC	ENDEXT		# TERMINATE - CREW PRESSED V 34
		TC 	+2		# PROCEED - CREW PRESSED V 33
		TC	-5		# DATA IN - CREW ENTERED NEW VALUE VIA V 22
					# OPTIONX+1 = 1 FOR Z AXIS, 2 FOR X AXIS
; ============================================================================
; V89RECL - RECYCLE ENTRY POINT FOR UPDATED ALIGNMENT COMPUTATION
;
; Both spacecraft are moving in lunar orbit. To compute accurate alignment
; angles, the AGC must first update both the CSM and LM state vectors to
; a common time slightly in the future (current time + 1 minute), then
; calculate the line-of-sight vector from LM to CSM in stable member
; coordinates for the VECPOINT attitude targeting algorithm.
; ============================================================================

V89RECL		TC	INTPRET		# ENTER INTERPRETIVE MODE FOR VECTOR MATH

; Compute target time as current time + 1 minute for state vector updates.
; This forward projection accounts for the time required to complete the
; computation and crew response, providing more accurate targeting.

		RTB	DAD
			LOADTIME	# READ PRESENT TIME INTO MPAC
			DP1MIN		# ADD 1 MINUTE (6000 CENTISECONDS)
		STORE	TSTART82	# SAVE TIME FOR LEMCONIC CALL
		STCALL	TDEC1		# STORE TIME FOR CSMCONIC CALL
			CSMCONIC	# UPDATE CSM STATE VECTOR TO TARGET TIME

; CSM position vector now in RATT (Reference Attitude). Save for later use
; in line-of-sight computation after LM state vector is similarly updated.

		VLOAD			# LOAD CSM POSITION VECTOR
			RATT
		STODL	RONE		# SAVE CSM POSITION IN RONE FOR LOS CALC
			TSTART82	# RETRIEVE TARGET TIME
		STCALL	TDEC1		# STORE TIME FOR LEMCONIC CALL
			LEMCONIC	# UPDATE LM STATE VECTOR TO TARGET TIME

; Compute line-of-sight vector: CSM position - LM position = vector from
; LM to CSM. Transform from reference coordinates to stable member (platform)
; coordinates and normalize to unit vector for VECPOINT.

		VLOAD	VSU		# CSM POSITION - LM POSITION = LOS VECTOR
			RONE		# RETRIEVE SAVED CSM POSITION
			RATT		# SUBTRACT LM POSITION (NOW IN RATT)
		MXV	RTB		# MULTIPLY BY REFSMMAT TO TRANSFORM LOS
			REFSMMAT	# FROM REFERENCE TO STABLE MEMBER COORD
			NORMUNIT	# NORMALIZE TO UNIT VECTOR (MAGNITUDE 0.5)
		STORE	POINTVSM	# STORE LOS VECTOR FOR VECPOINT CALCULATION
		EXIT			# RETURN TO NATIVE AGC CODE
; Determine which spacecraft axis the crew selected for CSM alignment.
; OPTIONX+1 = 1 indicates +Z axis (docking axis), value 2 indicates +X axis.
; Branch to appropriate axis setup routine.

		CS	OPTIONX +1	# COMPLEMENT OF SELECTION (1 OR 2)
		AD	ONE		# SUBTRACT 1: RESULT 0 FOR Z, -1 FOR X
		EXTEND
		BZF	ALINEZ		# BRANCH IF ZERO (Z AXIS SELECTED)

; ============================================================================
; X AXIS ALIGNMENT PATH
;
; The crew has selected the LM +X axis (approach axis) to point at the CSM.
; Load the unit vector representing +X in body coordinates: (0.5, 0, 0).
; The 0.5 scaling is used because AGC unit vectors have magnitude 0.5 rather
; than 1.0, a convention that simplifies fixed-point arithmetic operations.
; ============================================================================

ALINEX		TC	INTPRET		# ENTER INTERPRETIVE MODE
		VLOAD
			UNITX		# LOAD +X UNIT VECTOR: (0.5, 0, 0)

# Page 340
; ============================================================================
; V89CALL1 - COMMON PATH FOR ATTITUDE COMPUTATION AND DISPLAY
;
; With the line-of-sight vector (POINTVSM) and selected spacecraft axis
; (SCAXIS) now defined, VECPOINT computes the gimbal angles required to
; align the chosen axis with the LOS vector. BALLANGS then transforms these
; gimbal angles into FDAI ball angles for crew display and manual reference.
; ============================================================================

V89CALL1	STCALL	SCAXIS		# STORE SELECTED AXIS VECTOR IN SCAXIS
			VECPOINT	# COMPUTE GIMBAL ANGLES (OG, IG, MG)
		STORE	CPHI		# STORE GIMBAL ANGLES (OUTER, INNER, MIDDLE)
		EXIT			# RETURN TO NATIVE CODE

; Transform gimbal angles to FDAI ball angles for display on the Flight
; Director Attitude Indicator. The FDAI ball shows spacecraft attitude
; relative to the stable member platform, enabling crew to manually fly
; the LM to the computed attitude or verify automatic maneuver progress.

		TC	BANKCALL
		CADR	BALLANGS	# CONVERT TO FDAI ANGLES IN FDAIX, Y, Z

; Flash Display V 06 N 18 shows the computed FDAI ball angles to the crew.
; Noun 18 displays three angles (X, Y, Z) representing pitch, yaw, and roll.
; Crew responses: TERMINATE exits, PROCEED initiates automatic maneuver,
; RECYCLE recomputes with updated state vectors.

		CAF	VB06N18		# V 06 N 18
		TC	BANKCALL
		CADR	GOFLASH		# FLASH DISPLAY, WAIT FOR RESPONSE
		TC	ENDEXT		# TERMINATE - CREW ABORTS R63
		TC	+2		# PROCEED - INITIATE AUTO MANEUVER
		TC	V89RECL		# RECYCLE - RECOMPUTE WITH FRESH VECTORS

; Crew has pressed PROCEED to initiate automatic attitude maneuver to the
; computed alignment. R60LEM performs the LM maneuver using RCS thrusters
; to rotate the spacecraft to the desired attitude for rendezvous operations.

		TC	DOWNFLAG	# RESET 3 AXIS FLAG FOR MANEUVER
		ADRES	3AXISFLG	# CLEAR BIT6 OF FLAG WORD 5
		TC	BANKCALL	# CALL LM AUTOMATIC MANEUVER ROUTINE
		CADR	R60LEM		# EXECUTE ATTITUDE MANEUVER TO ALIGN WITH CSM
		TCF	ENDEXT		# MANEUVER COMPLETE, EXIT R63

; ============================================================================
; Z AXIS ALIGNMENT PATH
;
; The crew has selected the LM +Z axis (preferred docking axis) to point at
; the CSM. Load the unit vector representing +Z in body coordinates: (0, 0, 0.5).
; The +Z axis is the primary docking interface and provides the preferred
; orientation for final rendezvous approach and docking operations.
; ============================================================================

ALINEZ		TC	INTPRET		# ENTER INTERPRETIVE MODE FOR Z AXIS
		VLOAD	GOTO
			UNITZ		# LOAD +Z UNIT VECTOR: (0, 0, 0.5)
			V89CALL1	# PROCEED TO COMMON COMPUTATION PATH

; ============================================================================
; DISPLAY VERB/NOUN CODES AND TIME CONSTANTS
;
; These constants define the DSKY display codes used by R63 for crew
; interaction and the time interval for state vector propagation.
; ============================================================================

VB04N12		VN	412		# V 04 N 12: FLASH DISPLAY FOR AXIS SELECTION
VB06N18		VN	0618		# V 06 N 18: FLASH DISPLAY FOR FDAI BALL ANGLES

# Page 341

; Time interval constant: 60 seconds (6000 centiseconds) used to propagate
; both spacecraft state vectors forward in time before computing line-of-sight.
; This ensures the alignment solution accounts for relative motion during the
; time required for crew response and maneuver execution.

DP1MIN		2DEC	6000		# 60 SECONDS IN CENTISECONDS (1 MIN)

