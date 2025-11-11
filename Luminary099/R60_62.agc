# Copyright:	Public domain.
# Filename:	R60_R62.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	472-485
# Mod history:	2009-05-17 RSB	Adapted from the corresponding
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
; FILE: R60_62.agc
; MODULE: Attitude Maneuver and Display Routines
; MISSION PHASE: rendezvous/ascent/lunar-orbit
;
; TL;DR: R60 automates spacecraft attitude maneuvers to point any specified
;        spacecraft axis (such as rendezvous radar antenna, AOT telescope, or
;        thrust vector) in a desired direction. R62 provides manual maneuver
;        initiation via keyboard (V49). These routines enable crew to orient
;        the Lunar Module for critical operations including rendezvous radar
;        tracking of the Command Module during Eagle's ascent from the lunar
;        surface and subsequent rendezvous with Columbia in lunar orbit.
;
; COMMENT-ONLY READERS: This code controlled Eagle's orientation during the
;        historic rendezvous with Columbia after leaving the Moon. Follow the
;        comments to understand how the computer automatically maneuvered the
;        spacecraft to track Michael Collins' Command Module during approach.
;
; CODE-ALONG READERS: Study the integration of attitude computation (VECPOINT),
;        gimbal angle display conversion (BALLANGS), and automatic maneuver
;        execution (GOMANUR). Note the gimbal lock avoidance logic and the
;        cross-product rotation mathematics for attitude targeting.
; ============================================================================

# Page 472
# MOD NO: 0			DATE: 1 MAY 1968
# MOD BY: DIGITAL DEVEL GROUP	LOG SECTION R60,R62
#
# FUNCTIONAL DESCRIPTION:
#
# CALLED AS A GENERAL SUBROUTINE TO MANEUVER THE LM TO A SPECIFIED
# ATTITUDE.
#
# 1. IF THE 3-AXIS FLAG IS NOT SET THE FINAL CDU ANGLES ARE
# CALCULATED (VECPOINT).
#
# 2. THE FDAI BALL ANGLES (NOUN 18) ARE CALCULATED (BALLANGS).
#
# 3. REQUEST FLASHING DISPLAY V50 N18 PLEASE PERFORM AUTO MANEUVER.
#
# 4. IF PRIORITY DISPLAY FLAG IS SET DO A PHASECHANGE. THEN AWAIT
# ASTRONAUT RESPONSE.
#
# 5. DISPLAY RESPONSE RETURNS:
#
#     A. ENTER - RESET 3-AXIS FLAG AND RETURN TO CLIENT.
#
#     B. TERMINATE - IF IN P00 GO TO STEP 5A. OTHERWISE CHECK IF R61 IS
#	 THE CALLING PROGRAM. IF IN R61 AN EXIT IS MADE TO GOTOV56. IF
#	 NOT IN R61 AN EXIT IS DONE VIA GOTOPOOH.
#
#     C. PROCEED - CONTINUE WITH PROGRAM AT STEP 6.
#
# 6. IF THE 3-AXISFLAG IS NOT SET, THE FINAL CDU ANGLES ARE CALCULATED
#    (VECPOINT).
#
# 7. THE FDAI BALL ANGLES (NOUN 18) ARE CALCULATED (BALLANGS).
#
# 8. IF THE G+N SWITCH IS NOT SET GO BACK TO STEP 3.
#
# 9. IF THE AUTO SWITCH IS NOT SET GO BACK TO STEP 3.
#
# 10. NONFLASHING DISPLAY V06N18 (FDAI ANGLES).
#
# 11. DO A PHASECHANGE.
#
# 12. DO A MANEUVER CALCULATION AND ICDU DRIVE ROUTINE TO ACHIEVE FINAL
#
#     GIMBAL ANGLES (GOMANUR).
# 13. AT END OF MANEUVER GO TO STEP 3.
#
#	   IF SATISFACTORY MANEUVER STEP 5A EXITS R60.
#	   FOR FURTHER ADJUSTMENT OF THE VEHICLE ATTITUDE ABOUT THE
#	   DESIRED VECTOR, THE ROUTINE MAY BE PERFORMED AGAIN STARTING AT
# Page 473
#	   STEP 5C.
#
# CALLING SEQUENCE:  TC BANKCALL
#		     CADR R60LEM
#
# ERASABLE INITIALIZATION REQUIRED : SCAXIS, POINTVSM (FOR VECPOINT)
#				     3AXISFLG.
#
# SUBROUTINES CALLED: VECPOINT, BALLANGS, GOPERF2R, LINUS, GODSPER,
#		     GOMANUR, DOWNFLAG, PHASCHNG, UPFLAG
#
# NORMAL EXIT MODES: CAE TEMPR60   (CALLERS RETURN ADDRESS)
#		     TC  BANKJUMP
#
# ALARMS: NONE
#
# OUTPUT: NONE
#
# DEBRIS: CPHI, CTHETA, CPSI, 3AXISFLG, TBASE2

; ============================================================================
; TRANSITION: From program description to R60LEM implementation
;
; The Lunar Module now requires precise attitude control to point its
; rendezvous radar antenna toward the Command Module during ascent and
; approach. This automatic maneuver routine was crucial during Apollo 11's
; rendezvous on July 21, 1969, when Eagle tracked Columbia after lifting
; off from the lunar surface. The routine computes required gimbal angles,
; displays them to the crew on the FDAI ball, and executes the maneuver
; using the digital autopilot and RCS thrusters.
; ============================================================================

		BANK	34
		SETLOC	MANUVER
		BANK

		EBANK=	TEMPR60

		COUNT*	$$/R06

; R60LEM - Main Entry Point for Automatic Attitude Maneuver
;
; COMMENT-ONLY READERS: When the crew needs to point the spacecraft in a
; specific direction (such as aiming the rendezvous radar at the Command
; Module during approach), they call this routine. The computer calculates
; the required spacecraft orientation and displays it on the Flight Director
; Attitude Indicator (FDAI) ball. The crew can then choose to let the
; computer automatically maneuver to that attitude, or manually fly there.
;
; CODE-ALONG READERS: Entry via BANKCALL with return address in TEMPR60.
; Routine checks 3AXISFLG to determine if final gimbal angles are already
; computed or need calculation via VECPOINT. MAKECADR constructs a bank-
; relative address for return linkage across memory banks.

R60LEM		TC	MAKECADR
		TS	TEMPR60		; Save caller's return address for exit

; Main maneuver computation and display loop begins here. The LM orientation
; is calculated to point a specified spacecraft axis (stored in SCAXIS) toward
; a desired direction vector (stored in POINTVSM). During Apollo 11's ascent,
; this pointed the rendezvous radar antenna at Columbia's position in orbit.

REDOMANN	CAF	3AXISBIT
		MASK	FLAGWRD5	; IS 3-AXIS FLAG SET
		CCS	A
		TCF	TOBALL		; YES - Final angles already computed
		TC	INTPRET		; Enter interpretive mode for vector math
		CALL
			VECPOINT	; Compute final gimbal angles (CPHI,CTHETA,CPSI)
		STORE	CPHI		; Store final angles for FDAI display conversion
		EXIT

; Convert computed gimbal angles to FDAI ball angles for crew display.
; The FDAI (Flight Director Attitude Indicator) shows the spacecraft's
; orientation using an eight-ball display on the instrument panel. During
; rendezvous, Armstrong and Aldrin monitored these angles to ensure the
; rendezvous radar was tracking Columbia correctly.

TOBALL		TC	BANKCALL
		CADR	BALLANGS	; Convert gimbal angles to FDAI ball angles
TOBALLA		CAF	V06N18
		TC	BANKCALL
		CADR	GOPERF2R	; Display V06N18: Request AUTO MANEUVER
		TC	R61TEST		; Check if called from R61
		TC	REDOMANC	; PROCEED - Execute maneuver
		TC	ENDMANU1	; ENTER - Maneuver complete, exit R60
# Page 474
		TC	CHKLINUS	; Check for priority displays interrupting
		TC	ENDOFJOB

; REDOMANC - Recompute Maneuver and Check AUTO Mode
;
; COMMENT-ONLY READERS: After the crew presses PROCEED, the computer
; recalculates the required attitude (in case the target has moved, as
; Columbia would be continuously orbiting). It then checks if the AUTO
; switch is enabled. If AUTO mode is on, the computer takes control of
; the RCS thrusters and rotates the spacecraft automatically. If not,
; the display returns to request the crew enable AUTO mode.
;
; CODE-ALONG READERS: Recalculates final angles if 3AXISFLG not set,
; converts to FDAI angles, then checks G+N and AUTO switches. If AUTO
; not engaged, loops back to TOBALLA to re-request maneuver. If AUTO
; is engaged, proceeds to AUTOMANV for automatic RCS control.

REDOMANC	CAF	3AXISBIT
		MASK	FLAGWRD5	; IS 3-AXIS FLAG SET
		CCS	A
		TCF	TOBALLC		; YES - Skip angle recalculation
		TC	INTPRET		; Enter interpretive mode
		CALL
			VECPOINT	; Recompute final gimbal angles
		STORE	CPHI		; Store updated angles
		EXIT

TOBALLC		TC	BANKCALL
		CADR	BALLANGS	; Convert to FDAI ball angles
		TC	G+N,AUTO	; Verify AUTO mode engaged
		CCS	A
		TCF	TOBALLA		; AUTO not set - request AUTO again

; Automatic maneuver execution begins. The digital autopilot takes control
; of the RCS thrusters to rotate the Lunar Module to the computed attitude.
; During Apollo 11's rendezvous, this automated rotation ensured the
; rendezvous radar antenna remained locked on Columbia's position while
; Eagle maneuvered for docking approach.

AUTOMANV	CAF	V06N18		; Display FDAI angles (static, non-flashing)
		TC	BANKCALL
		CADR	GODSPR		; Display during automatic maneuver
		TC	CHKLINUS	; Check for priority display interrupts

STARTMNV	TC	BANKCALL	; Execute automatic maneuver
		CADR	GOMANUR		; KALCMANU steering with RCS autopilot

; Maneuver complete. The spacecraft has reached the desired attitude.
; Control returns to the main display loop to allow crew to verify the
; final orientation or initiate another adjustment if needed.

ENDMANUV	TCF	TOBALLA		; Maneuver complete - return to display
ENDMANU1	TC	DOWNFLAG	; Clear 3-AXIS flag on exit
		ADRES	3AXISFLG
		CAE	TEMPR60		; Retrieve caller's return address
		TC	BANKJUMP	; Return to calling program

; CHKLINUS - Check for Priority Display Interrupts
;
; COMMENT-ONLY READERS: During a maneuver, higher-priority displays can
; interrupt the normal sequence (such as urgent alarms or critical navigation
; updates). This routine checks if such an interrupt has occurred and handles
; the transition gracefully, ensuring the maneuver can resume after the
; priority display is acknowledged by the crew.
;
; CODE-ALONG READERS: Checks PDSPFBIT (Priority Display Flag) in FLAGWRD4.
; If set, saves return address in MPAC+2, computes restart location from
; BUF2-3 for phase change restart protection, calls LINUS to set display
; bits, then returns to saved address. Priority display system ensures
; critical information reaches crew even during automated sequences.

CHKLINUS	CS	FLAGWRD4
		MASK	PDSPFBIT	; IS PRIORITY DISPLAY FLAG SET?
		CCS	A
		TC	Q		; NO - Return to caller
		CA	Q
		TS	MPAC +2		; Save return address for post-display resume
		CS	THREE		; Compute restart protection address
		AD	BUF2		; BUF2 holds Q of last display routine
		TS	TBASE2		; Store for restart if power interruption

		TC	PHASCHNG
		OCT	00132		; Phase change for restart protection

		CAF	BIT7
		TC	LINUS		; Service priority display interrupt
		TC	MPAC +2		; Return to saved address after display

# Page 475

; RELINUS - Return from Priority Display (LINUS)
;
; After a priority display has been serviced, control returns here to
; restore the original job priority and determine whether to continue
; the R60 maneuver. The routine checks the TRACKFLAG to ensure the
; rendezvous tracking is still active before resuming automated control.

RELINUS		CAF	PRIO26		; Restore original job priority level
		TC	PRIOCHNG

		CAF	TRACKBIT	; Check if tracking still active
		MASK	FLAGWRD1
		CCS	A
		TCF	RER60		; TRACKFLAG set - resume R60

		CAF	RNDVZBIT	; IS IT P20 (rendezvous navigation)?
		MASK	FLAGWRD0
		CCS	A
		TC	+4		; YES - P20 active
		TC	PHASCHNG	; NO - Must be P25, set restart at 2.11
		OCT	40112

		TC	ENDOFJOB

		TC	PHASCHNG	; P20 active - set restart at 2.7
		OCT	40072

		TC 	ENDOFJOB

; RER60 - Re-enter R60 After Priority Display or Restart
;
; This entry point resumes R60 operation after a priority display interrupt
; or system restart. Sets the priority display flag and jumps to the saved
; restart location (TBASE2) to continue the maneuver sequence.

RER60		TC	UPFLAG		; Set priority display flag after restart
		ADRES	PDSPFLAG

		TC	TBASE2		; Jump to saved restart location

; R61TEST - Check Calling Program Context
;
; COMMENT-ONLY READERS: This routine determines which program called R60
; and selects the appropriate exit path. If called from program P00 (the
; idle program), it assumes the crew manually requested the maneuver via
; verb V49. If called from R61 within P20 or P25 rendezvous programs, it
; exits to the proper rendezvous navigation sequence.
;
; CODE-ALONG READERS: Checks MODREG for P00 (idle). If P00, returns via
; ENDMANU1. Otherwise checks PDSPFBIT to determine if R61 is active. If
; R61 active (P20/P25 rendezvous), exits via GOTOV56. If not R61, exits
; via GOTOPOOH to terminate program cleanly.

R61TEST		CA	MODREG		; Check current program mode
		EXTEND
		BZF	ENDMANU1	; P00 - Manual V49/V89 call, normal exit

		CA	FLAGWRD4	; Check if called from R61 (P20/P25)
		MASK	PDSPFBIT
		EXTEND
		BZF	GOTOPOOH	; Not R61 - terminate via POOH
		TC	GOTOV56		; R61 active - return to rendezvous sequence

; Flag and Display Constant Definitions

BIT14+7		OCT	20100		; Combined bit mask for specific flag tests
OCT203		OCT	203		; Constant for display/calculation routines
V06N18		VN	0618		; Verb 06 Noun 18 - Display FDAI angles

; ============================================================================
; G+N,AUTO - Check Guidance/Navigation and AUTO Mode Switches
; ============================================================================
;
; COMMENT-ONLY READERS: Before allowing the computer to automatically fire
; RCS thrusters to maneuver the spacecraft, this routine verifies that the
; crew has placed both mode switches in the correct positions. First it
; checks that the Guidance and Navigation (G+N) switch is engaged, giving
; the computer control authority. Then it verifies the AUTO switch is set,
; authorizing automatic thruster firing. This dual safety check ensures
; the astronauts retain ultimate control - they must explicitly authorize
; automated maneuvers.
;
; During Apollo 11's rendezvous with Columbia after Eagle's ascent from the
; lunar surface, Armstrong and Aldrin enabled both G+N and AUTO modes to
; allow the computer to execute precise attitude changes for rendezvous
; radar tracking, while maintaining ability to take manual control if needed.
;
; CODE-ALONG READERS: Reads hardware channel 30 (CHAN30) and tests bit 10
; for G+N switch position. If G+N not engaged, returns positive (non-zero)
; to indicate failure. If G+N engaged, falls through to ISITAUTO to check
; channel 31 (CHAN31) bit 14 for AUTO mode. Returns positive if AUTO not
; engaged, returns +0 (positive zero) if both switches set correctly.
; Calling routine uses CCS A to test: positive result triggers re-display
; requesting crew enable switches, +0 result allows maneuver to proceed.
;
# SUBROUTINE TO CHECK FOR G+N CONTROL. AUTO STABILIZATION
#
# RETURNS WITH C(A) = +  IF NOT SET FOR G+N, AUTO
# RETURNS WITH C(A) = +0 IF SWITCHES ARE SET

G+N,AUTO	EXTEND
		READ	CHAN30		; Read guidance/navigation control panel
		MASK	BIT10		; Isolate G+N switch bit
		CCS	A
		TC	Q		; NOT IN G+N - return positive (fail)
# Page 476
ISITAUTO	EXTEND			; G+N engaged - now check AUTO mode
		READ	CHAN31		; Read mode control panel
		MASK	BIT14		; Isolate AUTO switch bit
		TC	Q		; (+) = not AUTO, (+0) = both switches OK

# Page 477
; ============================================================================
; BALLANGS - Compute FDAI Ball Display Angles
; ============================================================================
;
; COMMENT-ONLY READERS: The Flight Director Attitude Indicator (FDAI), known
; as the "8-ball" display, shows the spacecraft's orientation to the crew.
; This routine converts the spacecraft's internal gimbal angles (which the
; computer uses for navigation calculations) into roll, pitch, and yaw angles
; that match how pilots naturally think about aircraft attitude. When Armstrong
; and Aldrin performed manual maneuvers during rendezvous or landing approach,
; they relied on this FDAI display to understand their spacecraft orientation.
;
; The transformation is complex because gimbal angles (outer/inner/middle)
; don't directly correspond to intuitive roll/pitch/yaw. This routine performs
; the mathematical gymnastics to present the information in pilot-friendly form.
;
; CODE-ALONG READERS: Converts gimbal angles (CPHI=OGA, CTHETA=IGA, CPSI=MGA)
; stored in spacecraft CDU (Coupling Data Unit) format into FDAI display angles
; (roll, pitch, yaw). Uses interpretive math routines for trigonometric
; transformations. Calls CD*TR*G to compute sines/cosines of gimbal angles,
; then uses ARCTAN and ARCSIN to extract Euler angles in pilot-natural sequence.
; Scaling: all angles are single-precision 2's complement scaled to half
; revolution (±0.5 = ±180°). Output stored in FDAIX/FDAIY/FDAIZ for display
; as degrees and hundredths via Noun 18 (inner gimbal angles) or Noun 19.
;
# PROGRAM DESCRIPTION BALLANGS
# MOD NO.	  LOG SECTION  R60,R62
#
# WRITTEN BY RAMA M.AIYAWAR
# FUNCTIONAL DESCRIPTION
#
# COMPUTES LM FDAI BALL DISPLAY ANGLES
# CALLING SEQUENCE
#
#		TC	BALLANGS
# NORMAL EXIT MODE
#
#		TC	BALLEXIT	# (SAVED Q)
#
# ALARM OR EXIT MODE  NIL
# SUBROUTINES CALLED
#		CD*TR*G
#		ARCTAN
#
# INPUT
#
# CPHI,CTHETA,CPSI  ARE  THE ANGLES CORRESPONDING TO AOG,AIG,AMG. THEY ARE
# SP,2S COMPLIMENT SCALED TO HALF REVOLUTION.
# OUTPUT
#
# FDAIX,FDAIY,FDAIZ ARE THE REQUIRED BALL ANGLES SCALED TO HALF REVOLUTION
# SP,2S COMPLIMENT.
# THESE ANGLES WILL BE DISPLAYED AS DEGREES AND HUNDREDTHS. IN THE ORDER  ROLL, PITCH, YAW, USING NOUNS 18 & 19.
#
# ERASABLE INITIALIZATION REQUIRED
#
# CPHI,CTHETA,CPSI EACH A SP REGISTER
# DEBRIS
#
# A,L,Q,MPAC,SINCDU,COSCDU,PUSHLIS,BALLEXIT
#
#
# NOMENCLATURE:	 CPHI, CTHETA, & CPSI REPRESENT THE OUTER, INNER, & MIDDLE GIMBAL ANGLES, RESPECTIVELY; OR
# EQUIVALENTLY, CDUX, CDUY, & CDUZ.
#
# NOTE:  ARCTAN CHECKS FOR OVERFLOW AND SHOULD BE ABLE TO HANDLE ANY SINGULARITIES.

		SETLOC	BAWLANGS
		BANK

		COUNT*	$$/BALL
BALLANGS	TC	MAKECADR		; Save return address for later
		TS	BALLEXIT
		CA	CPHI			; Outer gimbal angle (OGA = CDU X-axis)
# Page 478
		TS	CDUSPOT +4		; Store for CD*TR*G subroutine
		CA	CTHETA			; Inner gimbal angle (IGA = CDU Y-axis)
		TS	CDUSPOT			; Store in CDUSPOT base location
		CA	CPSI			; Middle gimbal angle (MGA = CDU Z-axis)
		TS	CDUSPOT +2		; Complete the gimbal angle triplet
;
; The three gimbal angles are now staged for trigonometric conversion.
; CD*TR*G will compute sines and cosines needed for the transformation.
;
		TC	INTPRET			; Enter interpretive mode for vector math
		SETPD	CALL			; Initialize pushdown stack pointer
			0D			; Start at location 0 in pushdown list
			CD*TR*G			; Compute sin/cos of all three gimbal angles

;
; YAW ANGLE COMPUTATION: Transform gimbal angles to pilot's yaw reference.
; Uses the relationship: Yaw = arcsin(-sin(OGA) * cos(MGA))
;
		DLOAD	DMP			; Load sin(OGA)
			SINCDUX			# SIN (OGA) = sin of outer gimbal
			COSCDUZ			# COS (MGA) = cos of middle gimbal
;
		SL1	DCOMP			# SCALE and negate: -sin(OGA)*cos(MGA)
		ARCSIN	PDDL			# YAW = ARCSIN(-SXCZ) pushed to 0 in PD list
			SINCDUZ			; Load sin(MGA) for next calculation
		STODL	SINTH			# Store sin of computed angle (SINTH = 18D in PD)
			COSCDUZ			; Load cos(MGA) for roll computation
;
; ROLL ANGLE COMPUTATION: Extract roll from gimbal geometry.
; Uses: Roll = arctan(sin(MGA) / [cos(MGA) * cos(OGA)])
;
		DMP	SL1			# RESCALE: cos(MGA) * cos(OGA)
			COSCDUX			; Multiply by cos(OGA)
		STCALL	COSTH			# Store result (COSTH = 16D in PD), call ARCTAN
			ARCTAN			; Compute arctangent for roll angle
		PDDL	DMP			# ROLL = ARCTAN(SZ/CZCX) pushed to 2 in PD
			SINCDUZ			; sin(MGA) for pitch computation
			SINCDUX			; sin(OGA) for cross-products
;
; PITCH ANGLE COMPUTATION: Final transformation for pilot's pitch reference.
; This involves complex cross-products of gimbal sines and cosines.
;
		SL2	PUSH			# sin(OGA)*sin(MGA) scaled, push to 4 in PD
		DMP	PDDL			# Multiply by cos(IGA), push to 4 in PD
			COSCDUY			; cos(IGA) for Y-axis component
		DMP	PDDL			# Multiply by sin(IGA), push to 6 in PD
			SINCDUY			; sin(IGA) for cross-coupling term
			COSCDUX			; cos(OGA) for next computation
		DMP	SL1			# cos(OGA) * cos(IGA)
			COSCDUY			; Multiply by cos(IGA)
		DSU	STADR			# Subtract value from 6 in PD stack
		STODL	COSTH			# COSTH = cos(OGA)*cos(IGA) - sin(OGA)*sin(MGA)*sin(IGA)
			SINCDUY			; sin(IGA) for final term
		DMP	SL1			; Rescale
			COSCDUX			# cos(OGA) * sin(IGA)
		DAD	STADR			# Add value from 4 in PD stack
		STCALL	SINTH			# SINTH = cos(OGA)*sin(IGA) + sin(OGA)*sin(MGA)*cos(IGA)
			ARCTAN			# RETURNS WITH D(MPAC) = PITCH
;
; The three angles (roll, pitch, yaw) are now computed in pilot-natural form.
; Pack them into a vector for storage in FDAI display registers.
;
		PDDL	VDEF			# PITCH pushed to 2 in PD, vector defined from stack
		RTB				# Round-trip conversion through vector routines
			V1STO2S			; Convert single-precision vector to scaled format
		STORE	FDAIX			# Store as FDAI X, Y, Z (roll, pitch, yaw)
		EXIT				; Return to basic AGC mode
;
; The FDAI display now shows the spacecraft orientation in intuitive form.
; Crew can monitor roll, pitch, yaw during manual attitude adjustments.
;
ENDBALL		CA	BALLEXIT		; Retrieve saved return address

# Page 479
		TC	BANKJUMP		; Cross-bank return to calling program

# Page 480
# PROGRAM DESCRIPTION - VECPOINT
#
#
#	   THIS INTERPRETIVE SUBROUTINE MAY BE USED TO POINT A SPACECRAFT AXIS IN A DESIRED DIRECTION.  THE AXIS
# TO BE POINTED MUST APPEAR AS A HALF UNIT DOUBLE PRECISION VECTOR IN SUCCESSIVE LOCATIONS OF ERASABLE MEMORY
# BEGINNING WITH THE LOCATION CALLED SCAXIS.  THE COMPONENTS OF THIS VECTOR ARE GIVEN IN SPACECRAFT COORDINATES.
# THE DIRECTION IN WHICH THIS AXIS IS TO BE POINTED MUST APPEAR AS A HALF UNIT DOUBLE PRECISION VECTOR IN
# SUCCESSIVE LOCATIONS OF ERASABLE MEMORY BEGINNING WITH THE ADDRESS CALLED POINTVSM.  THE COMPONENTS OF THIS
# VECTOR ARE GIVEN IN STABLE MEMBER COORDINATES.  WITH THIS INFORMATION VECPOINT COMPUTES A SET OF THREE GIMBAL
# ANGLES (2S COMPLEMENT) CORESPONDING TO THE CROSS-PRODUCT ROTATION BETWE EN SCAXIS AND POINTVSM AND STORES THEM
# IN T(MPAC) BEFORE RETURNING TO THE CALLER.
#	   THIS ROTATION, HOWEVER, MAY BRING THE S/C INTO GIMBAL LOCK.  WHEN POINTING A VECTOR IN THE Y-Z PLANE,
# THE TRANSPONDER AXIS, OR THE AOT FOR THE LEM, THE PROGRAM WILL CORRECT THIS PROBLEM BY ROTATING THE CROSS-
# PRODUCT ATTITUDE ABOUT POINTVSM BY A FIXED AMOUNT SUFFICIENT TO ROTATE THE DESIRED S/C ATTITUDE OUT OF GIMBAL
# LOCK.  IF THE AXIS TO BE POINTED IS MORE THAN 40.6 DEGREES BUT LESS THAN 60.5 DEG FROM THE +X (OR-X) AXIS,
# THE ADDITIONAL ROTATION TO AVOID GIMAL LOCK IS 35 DEGREES.  IF THE AXIS IS MORE THAN 60.5 DEGEES FROM +X (OR -X)
# THE ADDITIONAL ROTATION IS 35 DEGREES.  THE GIMBAL ANGLES CORRESPONDING TO THIS ATTITUDE ARE THEN COMPUTED AND
# STORED AS 2S COMPLIMENT ANGLES IN T(MPAC) BEFORE RETURNING TO THE CALLER.
#	   WHEN POINTING THE X-AXIS, OR THE THRUST VECTOR, OR ANY VECTOR WITHIN 40.6 DEG OF THE X-AXIS, VECPOINT
# CANNOT CORRECT FOR A CROSS-PRODUCT ROTATION INTO GIMBAL LOCK.  IN THIS CASE A PLATFORM REALIGNMENT WOULD BE
# REQUIRED TO POINT THE VECTOR IN THE DESIRED DIRECTION.  AT PRESENT NO INDICATION IS GIVEN FOR THIS SITUATION
# EXCEPT THAT THE FINAL MIDDLE GIMBAL ANGLE IN MPAC +2 IS GREATER THAN 59 DEGREES.
#
#	   CALLING SEQUENCE -
#	       1) LOAD SCAXIS, POINTVSM
#	       2) CALL
#		       VECPOINT
#
#	   RETURNS WITH
#
#	       1) DESIRED OUTER  GIMBAL ANGLE IN MPAC
#	       2) DESIRED INNER  GIMBAL ANGLE IN MPAC +1
#	       3) DESIRED MIDDLE GIMBAL ANGLE IN MPAC +2
#
#	   ERASABLES USED -
#
#	       1) SCAXIS	   6
#	       2) POINTVSM	   6
#	       3) MIS		  18
#	       4) DEL		  18
#	       5) COF		   6
#	       6) VECQTEMP	   1
#	       7) ALL OF VAC AREA 43
#
#			TOTAL	  99

		SETLOC	VECPT
		BANK
# Page 481
		COUNT*	$$/VECPT

		EBANK=	BCDU

;
; VECPOINT - VECTOR POINTING COMPUTATION WITH GIMBAL LOCK AVOIDANCE
;
; This critical routine computes the gimbal angles required to point a
; spacecraft axis (SCAXIS) in a desired direction (POINTVSM) while avoiding
; gimbal lock configurations. Used throughout rendezvous operations when the
; Lunar Module must orient for radar tracking, docking, or engine burns.
;
; The routine implements sophisticated logic to detect potential gimbal lock
; situations (middle gimbal near 90 degrees) and automatically adds a rotation
; about the pointing axis to maintain controllable gimbal geometry.
;
VECPNT1		STQ	BOV		# THIS ENTRY USES DESIRED CDUS
			VECQTEMP	# NOT PRESENT-ENTER WITH CDUD'S IN MPAC
			VECPNT2		; Skip reading current CDU angles
;
VECPNT2		AXC,2	GOTO		; Index to MIS storage location
			MIS		; Stable member coordinate system matrix
			STORANG		; Jump to angle storage routine
;
; VECPOINT - Main entry point for vector pointing computation
; Input: SCAXIS (spacecraft axis to point), POINTVSM (target direction)
; Output: Desired gimbal angles in MPAC (outer, inner, middle)
;
VECPOINT	STQ	BOV		# Save return address for later exit
			VECQTEMP	; Return queue temporary storage
			VECLEAR		# Clear overflow indicator if set
;
; Initialize computation by reading current spacecraft orientation from IMU.
; The present CDU (gimbal) angles define the current stable member orientation
; relative to the spacecraft body axes.
;
VECLEAR		AXC,2	RTB		; Set index register and execute subroutine
			MIS		# Read the present CDU angles and
			READCDUK	# store them in PD list at 25, 26, 27
STORANG		STCALL	25D		; Store angles at pushdown location 25
			CDUTODCM	# Convert CDU angles to direction cosine matrix
				; Result: S/C axes to stable member axes (MIS)
;
; CROSS-PRODUCT COMPUTATION: Determine rotation axis and angle
;
; Transform the desired pointing direction (POINTVSM) from stable member
; coordinates into initial spacecraft axes. This gives us the final target
; vector VF in the same coordinate system as the initial vector VI (SCAXIS).
;
		VLOAD	VXM		; Load target direction vector
			POINTVSM	# RESOLVE THE POINTING DIRECTION VF INTO
			MIS		# INITIAL S/C AXES ( VF = POINTVSM)
		UNIT			; Normalize to unit vector
		STORE	28D		; Store VF at pushdown location 28-33
					# PD 28 29 30 31 32 33 holds final direction
;
; Compute the cross product VF x VI to find the rotation axis perpendicular
; to both vectors. The magnitude of this cross product determines the sine
; of the rotation angle needed. If the vectors are parallel (or antiparallel),
; the cross product will be near zero, requiring special handling.
;
		VXV	UNIT		# Take the cross product VF x VI
			SCAXIS		# where VI = SCAXIS (initial pointing axis)
		BOV	VCOMP		; Branch on overflow to parallel-vector handler
			PICKAXIS	; Vectors are (anti)parallel, need special logic
		STODL	COF		# Store rotation axis; check magnitude
			36D		# of cross product (sine of angle)
		DSU	BMN		# Subtract minimum threshold
			DPB-14		# B-14 threshold (very small angle)
			PICKAXIS	# If magnitude too small, vectors are parallel
;
; Valid cross product found. Now compute the rotation angle between VI and VF
; by taking the dot product (gives cosine) and then ARCCOS to get the angle.
; This angle, combined with the rotation axis from the cross product, defines
; the required spacecraft rotation.
;
		VLOAD	DOT		# Load SCAXIS and dot with final direction
			SCAXIS		; VI vector
			28D		; VF vector from pushdown
		SL1	ARCCOS		; Scale and compute arccos for rotation angle
;
; ============================================================================
; COMPUTE ROTATION MATRIX AND CHECK FOR GIMBAL LOCK
;
; Having determined the rotation angle and axis, we now compute the full
; transformation matrix from initial to final spacecraft orientation. Then
; we must check if this desired attitude would place the gimbals near the
; lock position (middle gimbal near ±90 degrees).
; ============================================================================
;
COMPMATX	CALL			# Now compute the transformation from
			DELCOMP		# final S/C axes to initial S/C axes (MFI)
;
; Compute the complete transformation matrix from final spacecraft axes
; to stable member (reference) coordinates: MFS = MIS · MFI
; This matrix reveals what gimbal angles would be required.
;
		AXC,1	AXC,2		; Set up matrix pointers
			MIS		# COMPUTE THE TRANSFORMATION FROM FINAL
			KEL		# S/C AXES TO STABLE MEMBER AXES
		CALL			# MFS = MIS · MFI
			MXM3		# Matrix multiply (result in PD list)
;
; GIMBAL LOCK DETECTION: Check if desired attitude is near gimbal lock
;
; Element MFS6 = sin(middle gimbal angle). If |sin(ψ)| > sin(59°),
; the middle gimbal is too close to ±90° (gimbal lock configuration).
; In this case, we must add a rotation about the pointing axis to move
; away from the singular gimbal geometry.
;
		DLOAD	ABS		; Load and take absolute value
			6		# MFS6 = sin(CPSI) at scale 2
		DSU	BMN		; Subtract gimbal lock threshold
			SINGIMLC	# = sin(59 degrees) at scale 2
			FINDGIMB	# |ψ| < 59°: attitude is safe, proceed
# Page 482
					# I.E. DESIRED ATTITUDE NOT IN GIMBAL LOCK

;
; Gimbal lock detected! Now check if we're attempting to point the thrust
; axis (Z-axis, SCAXIS component) toward gimbal lock. If the Z-component
; is large, we're trying to point along the gimbal axis—a problematic case.
;
		DLOAD	ABS		# Check Z-component of SCAXIS
			SCAXIS		# The thrust axis (typically Z-axis)
		DSU	BPL		; Subtract threshold
			SINVEC1		# sin(49.4 degrees) at scale 2
			FINDGIMB	# Z-component small: safe to proceed
;
; We are attempting to point the thrust axis into gimbal lock. This is
; a dangerous situation where attitude control becomes unstable. The code
; could abort here, but instead adds a corrective rotation.
;
;
; Store the MFS matrix (currently in pushdown list) into MIS for use in
; computing the corrective rotation. We need to determine which direction
; to rotate about the SCAXIS to most efficiently escape gimbal lock.
;
		VLOAD			# Load matrix elements from pushdown
		STADR			; Get address of data
		STOVL	MIS +12D	; Store third row of MFS matrix
		STADR			# Get next element address
		STOVL	MIS +6		; Store second row of MFS matrix
		STADR			; Get next element address
		STOVL	MIS		; Store first row of MFS matrix
			MIS +6		# Load inner gimbal axis in final S/C axes
;
; Determine rotation direction: The inner gimbal axis direction tells us
; which way to rotate about SCAXIS. We choose +SCAXIS or -SCAXIS to give
; the shortest rotation out of gimbal lock.
;
		BPL	VCOMP		# If IG axis positive, complement
			IGSAMEX		# Otherwise use SCAXIS directly

;
; IGSAMEX - Determine optimal rotation direction about SCAXIS
;
; Compute cross product (inner gimbal axis × SCAXIS) and check its dot
; product with final X-axis. This determines the shortest rotation path
; out of gimbal lock.
;
IGSAMEX		VXV	BMN		# Find the shortest way of rotating the
			SCAXIS		# S/C out of gimbal lock by a rotation
			U=SCAXIS	# about ±SCAXIS. If (IG·sign(MFS3)
					# × SCAXIS · XF) < 0, U = +SCAXIS
					# Otherwise U = -SCAXIS
;
; Result negative: Rotate about -SCAXIS (negative direction)
;
		VLOAD	VCOMP		; Load SCAXIS and complement it
			SCAXIS		; Gives -SCAXIS
		STCALL	COF		# Store rotation axis (about -SCAXIS)
			CHEKAXIS	; Check which angle to apply
;
; Result positive: Rotate about +SCAXIS (positive direction)
;
U=SCAXIS	VLOAD			; Load SCAXIS directly
			SCAXIS		; No sign change needed
		STORE	COF		# Store rotation axis (about +SCAXIS)
;
; ============================================================================
; CHEKAXIS - Select corrective rotation angle based on pointing vector
;
; The amount of rotation needed to escape gimbal lock depends on which
; sensor or system is being pointed. AOT (Alignment Optical Telescope)
; requires 50° rotation, while transponder or other Y/Z plane vectors
; need only 35° rotation. This ensures we move far enough from gimbal
; lock while minimizing attitude disturbance.
; ============================================================================
;
CHEKAXIS	DLOAD	ABS		; Load and take absolute value
			SCAXIS		# Check Z-component of SCAXIS
		DSU	BPL		; Subtract AOT threshold
			SINVEC2		# sin(29.5 degrees) at scale 2
			PICKANG1	# |Z| > sin(29.5°): pointing AOT, use 50°
;
; Not pointing AOT. Must be pointing transponder antenna or vector in Y-Z
; plane. Use 35° corrective rotation (smaller angle sufficient).
;
		DLOAD	GOTO		# Load 35° rotation angle
			VECANG2		# VECANG2 = 35 degrees at $360 scale
			COMPMFSN	# Apply this rotation to escape lock
;
; Pointing AOT: Use 50° corrective rotation about ±SCAXIS
;
PICKANG1	DLOAD			; Load 50° rotation angle
			VECANG1		# VECANG1 = 50 degrees at $360 scale
;
; Compute the modified orientation matrix with corrective rotation applied
;
COMPMFSN	CALL			; Compute rotation about SCAXIS axis
			DELCOMP		# Build rotation matrix for escape angle
		AXC,1	AXC,2		# Set up matrix multiplication pointers
			MIS		; Current orientation matrix
			KEL		; Matrix multiply buffer
		CALL			# Compute the new transformation from
			MXM3		# desired S/C axes to stable member axes
					# This matrix aligns VI with VF while
# Page 483
					# avoiding gimbal lock configuration
;
; ============================================================================
; FINDGIMB - Extract gimbal angles from orientation matrix
;
; Convert the computed orientation matrix (stable member to spacecraft axes)
; into the three gimbal angles (inner, middle, outer) that the CDUs
; (Coupling Data Units) must drive to. This is the final output of VECPOINT:
; the commanded CDU angles that will achieve the desired pointing.
; ============================================================================
;
FINDGIMB	AXC,1	CALL		; Index pointer = 0
			0		# Extract the commanded CDU angles from
			DCMTOCDU	# orientation matrix (DCM to CDU conversion)
		RTB	SETPD		; Convert angle format
			V1STO2S		# Convert to 2's complement format
			0		; Set pushdown pointer to base
		GOTO			; Mission complete
			VECQTEMP	# Return to calling program with CDU angles
;
; ============================================================================
; PICKAXIS - Handle special case: VF parallel or antiparallel to VI
;
; When the cross product VF × VI = 0, the vectors are either parallel
; (same direction) or antiparallel (opposite directions). This requires
; special handling:
;   - If VF = VI (parallel): No rotation needed, use current CDU angles
;   - If VF = -VI (antiparallel): 180° rotation required about perpendicular
;
; This is a degenerate case where the standard rotation axis calculation
; fails (zero-length rotation axis vector).
; ============================================================================
;
PICKAXIS	VLOAD	DOT		# Cross product was zero: Check dot product
			28D		; Load VF from pushdown location
			SCAXIS		; Dot with VI (in SCAXIS)
		BMN	TLOAD		; Negative dot product: VF antiparallel to VI
			ROT180		# Go handle 180° rotation case
			25D		; Positive: VF parallel to VI
		GOTO			# VF = VI, no rotation needed
			VECQTEMP	# Return with current CDU angles unchanged

		BANK	35
		SETLOC	MANUVER1
		BANK
;
; ============================================================================
; ROT180 - Construct 180° rotation for antiparallel vectors
;
; When VF and VI point in opposite directions (antiparallel), a 180° rotation
; is needed. But we must choose a rotation axis perpendicular to VI. This
; routine constructs an appropriate axis by finding a vector in the plane
; of the initial X-axis and stable member Y-axis, perpendicular to VI.
;
; If this fails (vectors too small), falls back to using the X-axis directly.
; ============================================================================
;
ROT180		VLOAD	VXV		# VF antiparallel to VI: 180° rotation needed
			MIS 	+6	# Load Y stable member axis from matrix
			HIDPHALF	# X-axis (initial spacecraft orientation)
		UNIT	VXV		# Compute Y(SM) × X(I), then normalize
			SCAXIS		# Compute VI × unit(Y(SM) × X(I))
		UNIT	BOV		# Normalize to get perpendicular axis
			PICKX		# Overflow: vectors aligned, use X-axis
		STODL	COF		; Store rotation axis in COF
			36D		# Load magnitude of this vector
		DSU	BMN		# Check magnitude adequacy
			DPB-14		# Compare against 2^-14 threshold
			PICKX		# Too small: Use X-axis instead
;
; Rotation axis is adequate. Use it for 180° rotation.
;
		VLOAD			; Reload the computed axis
			COF
XROT		STODL	COF		; Store rotation axis (180° about this axis)
			HIDPHALF	; Load 180° angle constant
		GOTO			; Proceed to matrix computation
			COMPMATX	# Build rotation matrix for 180° turn
;
; Fallback: Use spacecraft X-axis as rotation axis for 180° turn
;
PICKX		VLOAD	GOTO		# Use X-axis in this degenerate case
			HIDPHALF	; X-axis unit vector
			XROT		; Apply 180° rotation about X-axis
;
; ============================================================================
; VECPOINT Constants - Gimbal lock detection and escape thresholds
;
; These constants define the critical angles for detecting when the spacecraft
; is approaching gimbal lock and the corrective rotations needed to escape it.
; ============================================================================
;
; Gimbal lock detection threshold: Middle gimbal angle > 59°
;
SINGIMLC	2DEC	.4285836003	# sin(59°) at scale 2
					# Detection threshold for gimbal lock
;
; Thrust axis pointing threshold: Determines if pointing toward gimbal lock
;
SINVEC1		2DEC	.3796356537	# sin(49.4°) at scale 2
					# Threshold for thrust axis check
;
; AOT pointing detection threshold: Determines corrective rotation angle
;
SINVEC2		2DEC	.2462117800	# sin(29.5°) at scale 2
					# AOT vs transponder discrimination
;
; Corrective rotation angles for escaping gimbal lock
;
VECANG1		2DEC	.1388888889	# 50 degrees at $360 scale
					# Rotation for AOT pointing case
# Page 484
VECANG2		2DEC	.09722222222	# 35 degrees at $360 scale
					# Rotation for transponder/Y-Z vectors
;
; Magnitude threshold for vector validity checking
;
1BITDP		OCT	0		# Keep this before DPB-14
DPB-14		OCT	00001		# 2^-14 threshold for vector magnitude
		OCT	00000		# Minimum acceptable vector size

# Page 485
;
; ============================================================================
; R62 - Manual Attitude Maneuver via DSKY (Verb 49)
;
; ROUTINE FOR INITIATING AUTOMATIC MANEUVER VIA KEYBOARD (V49)
;
; R62 allows the crew to manually command the LM to a specific attitude by
; entering desired gimbal angles through the DSKY. This is used when the
; crew needs direct control of spacecraft orientation, such as:
;   - Setting up for visual observations through windows
;   - Positioning high-gain antenna for Earth communications
;   - Manual attitude adjustments before rendezvous maneuvers
;
; CALLING SEQUENCE: V49E (Verb 49, ENTER) from DSKY
;
; CREW PROCEDURE:
;   1. Verb 49 ENTER invokes R62
;   2. Display flashes V06N22 showing current ICDU (gimbal) angles
;   3. PROCEED: Initiate maneuver to displayed angles
;   4. ENTER: Allows crew to load new desired angles, then re-flash
;   5. TERMINATE: Abort and return to program
;
; HISTORICAL CONTEXT:
; During Apollo 11's approach to Columbia after lunar ascent, Aldrin and
; Armstrong used manual attitude commands to maintain visual contact with
; the Command Module while the computer handled rendezvous navigation.
; ============================================================================
;

		BANK	34
		SETLOC	R62
		BANK
		EBANK=	BCDU		; Extended bank contains gimbal angles

		COUNT*	$$/R62

R62DISP		EQUALS	R62FLASH	; Entry point alias

;
; Display current ICDU angles and await crew response
;
R62FLASH	CAF	V06N22		# Flash display: V06N22
		TC	BANKCALL	# Verb 06 (display decimal)
		CADR	GOFLASH		# Noun 22 (ICDU angles: OG, MG, IG)
		TCF	ENDEXT		# Crew response: TERMINATE - abort R62
		TCF	GOMOVE		# Crew response: PROCEED - execute maneuver
		TCF	R62FLASH	# Crew response: ENTER - reload angles
;
; Crew has entered PROCEED or loaded new angles
; Astronaut may load new ICDU angles at this point via DSKY numeric entry
;
GOMOVE		TC	UPFLAG		# Set 3-axis maneuver flag
		ADRES	3AXISFLG	# Enables full 3-axis attitude control

		TC	BANKCALL	; Execute automatic maneuver
		CADR	R60LEM		# Call R60LEM to perform attitude change
		TCF	ENDEXT		# Maneuver complete: End R62 and return
