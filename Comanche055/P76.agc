# Copyright:    Public domain.
# Filename:     P76.agc
# Purpose:      Part of the source code for Colossus 2A, AKA Comanche 055.
#               It is part of the source code for the Command Module's (CM)
#               Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:    yaYUL
# Contact:      Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:      www.ibiblio.org/apollo.
# Pages:	pp  511-513
# Mod history:  2009-05-08 HG    Adapting from the Luminary131/ file
#               of the same name, using Comanche055 page
#               images 0511.jpg - 0513.jpg.
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
; FILE: P76.agc
; MODULE: COMEKISS Subsystem (Orbital Navigation)
; MISSION PHASE: all-phases
;
; TL;DR: Target ΔV program computing required velocity changes for external
;        maneuver planning. Calculates burn magnitude and direction from ground
;        uplink data or crew input, integrating with mission planning timeline.
;        Used throughout Apollo 11 for maneuver verification and planning.
;
; COMMENT-ONLY READERS: This program calculated velocity changes needed for
;        maneuvers planned by Mission Control or adjusted by the crew.
; CODE-ALONG READERS: Study ΔV computation from target parameters, ground
;        uplink integration, and burn parameter display formatting.
; ============================================================================

# Page 511
# 1)	PROGRAM NAME - TARGET DELTA V PROGRAM (P76).
# 2)	FUNCTIONAL DESCRIPTION - UPON ENTRY BY ASTRONAUT ACTION, P76 FLASHES DSKY REQUESTS TO THE ASTRONAUT
#	TO PROVIDE VIA DSKY (1) THE DELTA V TO BE APPLIED TO THE OTHER VEHICLE STATE VECTOR AND (2) THE
#	TIME (TIG) AT WHICH THE OTHER VEHICLE VELOCITY WAS CHANGED BY EXECUTION OF A THRUSTING MANEUVER. THE
#	OTHER VEHICLE STATE VECTOR IS INTEGRATED TO TIG AND UPDATED BY THE ADDITION OF DELTA V (DELTA V HAVING
#	BEEN TRANSFORMED FROM LV TO REF COSYS).  USING INTEGRVS, THE PROGRAM THEN INTEGRATES THE OTHER
#	VEHICLE STATE VECTOR TO THE STATE VECTOR OF THIS VEHICLE, THUS INSURING THAT THE W-MATRIX AND BOTH VEHICLE
#	STATES CORRESPOND TO THE SAME TIME.
# 3)	ERASABLE INITIALIZATION REQUIRED - NONE.
# 4)	CALLING SEQUENCES AND EXIT MODES - CALLED BY ASTRONAUT REQUEST THRU DSKY V 37 E 76 E.
#	EXITS BY TCF ENDOFJOB.
# 5)	OUTPUT - OTHER VEHICLE STATE VECTOR INTEGRATED TO TIG AND INCREMENTED BY DELTA V IN REF COSYS.
#	THE PUSHLIST CONTAINS THE MATRIX BY WHICH THE INPUT DELTA V MUST BE POST-MULTIPLIED TO CONVERT FROM LV
#	TO REF COSYS.
# 6)	DEBRIS - OTHER VEHICLE STATE VECTOR.
# 7)	SUBROUTINES CALLED - BANKCALL,GOXDSPF,CSMPREC (OR LEMPREC),ATOPCSM (OR ATOPLEM),INTSTALL,INTWAKE, PHASCHNG
#	INTPRET, INTEGRVS, AND MINIRECT.
# 8)	FLAG USE - MOONFLAG,CMOONFLG,INTYPFLG,RASFLAG, AND MARKCTR.

		BANK	30
		SETLOC	P76LOC
		BANK

		COUNT*	$$/P76

		EBANK=	TIG

; ============================================================================
; PROGRAM P76 - TARGET DELTA V PROGRAM
;
; This program enables the crew or Mission Control to update the other
; vehicle's state vector with a planned velocity change (ΔV). During Apollo 11,
; this capability allowed coordination between the Command Module (Columbia)
; and Lunar Module (Eagle) for rendezvous planning and maneuver verification.
;
; The program requests two inputs via DSKY:
; 1. Delta V vector components (magnitude and direction of velocity change)
; 2. TIG (Time of Ignition) when the maneuver occurred or will occur
;
; P76 then transforms the ΔV from local vertical coordinates to the reference
; coordinate system, integrates the other vehicle's state vector to TIG,
; applies the velocity change, and integrates both vehicles to a common time.
; ============================================================================

P76		TC	UPFLAG
		ADRES	TRACKFLG

; ============================================================================
; CREW INPUT SEQUENCE - DELTA V VECTOR
;
; The program begins by flashing Verb 06 Noun 84 on the DSKY, displaying the
; last delta V that was entered. This allows the crew to verify or modify the
; velocity change vector.
;
; Crew actions:
; - ENTER: Accept displayed values and proceed to next input
; - Keyboard entry: Modify delta V components (3 values: X, Y, Z in local
;   vertical coordinate system scaled in feet/second)
; - PROCEED: Confirm entry and continue
;
; The delta V values can come from Mission Control via uplink (processed by
; UPDATE_PROGRAM) or be manually entered by the crew based on flight plan.
; ============================================================================

		CAF	V06N84          # FLASH LAST DELTA V.
		TC      BANKCALL        # AND WAIT FOR KEYBOARD ACTION.
	        CADR    GOFLASH
		TCF     ENDP76		; TERMINATE if crew presses out
		TC	+2		# PROCEED
		TC	-5		# STORE DATA AND REPEAT FLASHING

; ============================================================================
; CREW INPUT SEQUENCE - TIME OF IGNITION (TIG)
;
; Next, the program flashes Verb 06 Noun 33, displaying the last TIG (Time of
; Ignition). The crew verifies or enters when the other vehicle executed or
; will execute its velocity change maneuver.
;
; TIG is specified as mission elapsed time in hours, minutes, and seconds,
; which the AGC stores internally as centiseconds scaled by 2^28.
;
; Accurate TIG is critical: the program integrates the other vehicle's
; trajectory to this exact time before applying the delta V, ensuring the
; updated state vector correctly reflects the maneuver timing.
; ============================================================================

		CAF	V06N84 +1	# FLASH VERB 06 NOUN 33, DISPLAY LAST TIG,
		TC	BANKCALL	# AND WAIT FOR KEYBOARD ACTION.
		CADR	GOFLASH
		TCF	ENDP76		; TERMINATE if crew presses out
		TC	+2		; PROCEED confirmed
		TC	-5		; STORE DATA AND REPEAT FLASHING
; ============================================================================
; COORDINATE TRANSFORMATION PREPARATION
;
; With delta V and TIG entered, the program now integrates the other vehicle's
; state vector (position and velocity) to the specified TIG. This ensures the
; velocity change is applied at the correct point in the other vehicle's orbit.
;
; OTHPREC subroutine handles precision integration, accounting for:
; - Earth or lunar gravitational field (depending on MOONFLAG)
; - Orbital perturbations
; - Integration from current time to TIG
;
; After integration, RATT contains the other vehicle's position at TIG,
; and VATT contains its velocity at TIG, both in the reference coordinate
; system (Earth-centered inertial or Moon-centered inertial).
; ============================================================================

		TC	INTPRET		# RETURN TO INTERPRETIVE CODE
		DLOAD	                # SET D(MPAC)=TIG IN CSEC B28
			TIG
		STCALL	TDEC1		# SET TDEC1=TIG FOR ORBITAL INTEGRATION
			OTHPREC		; Integrate other vehicle to TIG

; ============================================================================
; COORDINATE TRANSFORMATION MATRIX COMPUTATION (COMPMAT)
;
; This section constructs a transformation matrix to convert the delta V from
; Local Vertical (LV) coordinates to Reference (REF) coordinates.
;
; Local Vertical coordinate system at TIG:
; - X-axis: Along velocity vector (direction of orbital motion)
; - Y-axis: Perpendicular to orbit plane (along angular momentum vector)
; - Z-axis: Radially outward from central body
;
; The transformation matrix is built from three orthogonal unit vectors:
; 1. U(-R): Unit vector radially inward (negative of position vector)
; 2. U(V×R): Unit vector perpendicular to orbit plane
; 3. U((R×V)×R): Unit vector along velocity direction
;
; These three vectors form a right-handed orthonormal basis stored in
; locations 12D, 18D, and 24D, creating the transformation matrix.
; ============================================================================

COMPMAT		VLOAD	UNIT		; Load position vector at TIG
			RATT		; RATT = other vehicle position
# Page 512
		VCOMP			# U(-R): Complement to get inward direction
		STORE	24D		; Store radially inward unit vector to 24D

; Construct perpendicular to orbital plane: U(V × R)
; This is the angular momentum direction (orbit normal)
		VXV	UNIT		; Cross product: U(-R) × V
			VATT		; VATT = other vehicle velocity at TIG
		STORE	18D		; Store orbit normal unit vector to 18D

; Construct along-track direction: U((R × V) × R)
; This is perpendicular to both the radial and orbit normal directions,
; pointing along the velocity vector direction
		VXV	UNIT		; Cross product: U(V×R) × U(-R)
			24D
		STOVL	12D		; Store along-track unit vector to 12D
			DELVOV		; Load crew-entered delta V (in LV coords)

; ============================================================================
; DELTA V TRANSFORMATION AND APPLICATION
;
; Transform the delta V from Local Vertical coordinates (crew input) to
; Reference coordinates using the transformation matrix just computed.
;
; The matrix multiplication (VXM) post-multiplies the delta V vector by the
; transformation matrix, converting from the local vertical system (defined
; by the other vehicle's position and velocity at TIG) to the reference
; inertial system.
;
; After transformation, the delta V is added to the other vehicle's velocity
; at TIG, producing the updated velocity after the maneuver. This updated
; state vector (position unchanged, velocity incremented) represents the
; other vehicle's trajectory following the planned or executed burn.
; ============================================================================

		VXM	VSL1		; Transform: DELVOV × Matrix, shift left 1
			12D		; Matrix in 12D, 18D, 24D
		VAD			; Add transformed delta V to velocity
			VATT		; Current velocity at TIG
		STORE	6		; V(PD6) = VATT + DELTA V (updated velocity)
; ============================================================================
; STATE VECTOR UPDATE AND INTEGRATION TO COMMON TIME
;
; Now that the delta V has been applied to the other vehicle's velocity at
; TIG, both vehicles' state vectors must be integrated to a common time to
; maintain consistency for rendezvous navigation and tracking.
;
; INTSTALL prevents other programs from interfering with the orbital
; integration routines during this critical update sequence.
;
; The updated state vector (position at TIG, velocity at TIG plus delta V)
; is stored in the conic integration variables (RCV, VCV) with the time
; stored in TET. The program then integrates this updated state to the
; time of this vehicle's state vector (TETTHIS).
; ============================================================================

		CALL			# PREVENT WOULD-BE USER OF ORBITAL
			INTSTALL	; INTEG FROM INTERFERING WITH UPDATING
		CALL
			P76SUB1		; Set or clear MOONFLAG based on X2
		VLOAD	VSR*		; Load updated velocity from PD6
			6
			0,2		; Shift right by X2 for proper scaling
		STOVL	VCV		; Store in conic velocity variable
			RATT		; Load position at TIG
		VSR*			; Shift right for proper scaling
			0,2
		STODL	RCV		; Store in conic position variable
			TIG		; Load TIG
		STORE	TET		; Store as epoch time for integration
		CLEAR	DLOAD		; Clear integration type flag
			INTYPFLG	; (precision integration mode)
			TETTHIS		; Load this vehicle's state vector time

; Integrate the other vehicle's updated state from TIG to this vehicle's time
INTOTHIS	STCALL	TDEC1		; Set target time for integration
			INTEGRVS	; Call precision integrator (INTEGRVS)
; ============================================================================
; REVERSE INTEGRATION - THIS VEHICLE TO COMMON TIME
;
; The other vehicle has been integrated forward to this vehicle's time.
; Now integrate this vehicle backward to ensure both state vectors
; correspond to the exact same time, maintaining the W-matrix consistency
; required for accurate rendezvous navigation.
;
; RATT1/VATT1 contain this vehicle's state, which is temporarily stored
; in the integration variables and processed through MINIRECT to ensure
; proper coordinate system alignment.
; ============================================================================

		CALL
			INTSTALL	; Prevent integration interference
		CALL
		        P76SUB1         ; SET/CLEAR MOONFLAG based on X2
		VLOAD			; Load this vehicle's position
			RATT1
		STORE	RRECT		; Store in rectangular coordinates
		STODL	RCV		; Store for integration
			TAT		; Load this vehicle's time
		STOVL	TET		; Store as epoch time
			VATT1		; Load this vehicle's velocity
		CALL
			MINIRECT	; Mini-rectification for coordinate alignment
		EXIT
		TC	PHASCHNG	; Phase change for restart protection
		OCT	04024		; Phase table entry
# Page 513

; ============================================================================
; FINALIZATION AND EXIT SEQUENCE
;
; With both vehicles' state vectors now at a common time and the other
; vehicle's state updated with the delta V, the program completes by:
;
; 1. Setting REINTFLG to indicate re-integration is complete
; 2. Calling ATOPOTH to finalize the other vehicle's state vector
; 3. Releasing the integration system (INTWAKE1) for other programs
; 4. Clearing tracking flags and buffers
;
; The program then returns control to the keyboard/display system (GOTOPOOH),
; allowing the crew to proceed with mission activities using the updated
; state vector information for navigation and maneuver planning.
; ============================================================================

		TC	UPFLAG		; Set re-integration complete flag
		ADRES	REINTFLG

		TC	INTPRET		; Return to interpretive code
		CALL
			ATOPOTH		; Finalize other vehicle state vector
		SSP     EXIT		; Store skip parameter and exit
		        QPRET		; Return address
		        OUT		; Jump to OUT label
		TC      BANKCALL        ; PERMIT USE OF ORBITAL INTEGRATION
		CADR    INTWAKE1	; Release integration system lock
OUT		EXIT

; ============================================================================
; PROGRAM TERMINATION - ENDP76
;
; Clean up tracking system state before returning to keyboard/display.
; This ensures no residual tracking or mark data interferes with subsequent
; programs. During Apollo 11, proper cleanup was essential to prevent
; spurious radar tracking data from affecting rendezvous computations.
; ============================================================================

ENDP76		CAF	ZERO
		TS	MARKCTR		; CLEAR RR TRACKING MARK COUNTER
		TS      VHFCNT		; Clear VHF ranging counter

		CAF     NEGONE
		TS      MRKBUF2         ; INVALIDATE MARK BUFFER

		TCF	GOTOPOOH	; Return to keyboard/display control

V06N84		NV	0684		; Verb 06 Noun 84 (display and flash)
		NV	0633		; Verb 06 Noun 33 (display and flash)

; ============================================================================
; SUBROUTINE P76SUB1 - MOON FLAG MANAGEMENT
;
; This interpretive subroutine determines whether orbital integration should
; use Earth-centered or Moon-centered gravitational models. The decision is
; based on the value in index register X2, which indicates the magnitude
; scaling applied to position and velocity vectors.
;
; X2 values and their meaning:
; - X2 = 0: Use Earth-centered coordinates (MOONFLAG cleared)
; - X2 = 2: Use Moon-centered coordinates (MOONFLAG set)
;
; The value in X2 corresponds to the right-shift amount applied to state
; vectors, indicating whether cislunar distances (Earth scale) or lunar
; vicinity distances (Moon scale) are being used. During Apollo 11's journey,
; this automatic switching ensured accurate trajectory integration as the
; spacecraft transitioned between Earth and Moon gravitational spheres.
;
; Called from: Main P76 integration sequence (twice)
; Returns via: RVQ (interpretive return via Q)
; ============================================================================

P76SUB1		CLEAR   SLOAD		; Clear MOONFLAG and load X2
			MOONFLAG	; Start with Earth-centered assumption
                        X2		; Load scaling index register
                BHIZ    SET             ; Branch if X2=0, else set MOONFLAG
                        +2              ; Skip next instruction if X2=0
                        MOONFLAG	; Set for Moon-centered integration
                RVQ			; Return via Q register

