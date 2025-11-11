# Copyright:	Public domain.
# Filename:	P76.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	709-711
# Mod history:	2009-05-19 RSB	Adapted from the corresponding
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
; FILE: P76.agc
; MODULE: Orbital Navigation Programs
; MISSION PHASE: lunar-orbit/rendezvous
;
; TL;DR: Implements Target Delta-V Program (P76) for computing and applying
;        velocity changes to the other vehicle's state vector (Command Module
;        when executed from LM, or vice versa). Accepts crew input of delta-V
;        magnitude and time of ignition (TIG), transforms the velocity change
;        from local vertical coordinates to reference coordinates, integrates
;        both vehicle states to common time for accurate relative navigation,
;        and updates the other vehicle's trajectory for rendezvous planning.
;
; COMMENT-ONLY READERS: This program manages tracking the other spacecraft's
;        maneuvers during rendezvous operations, allowing the crew to update
;        navigation state when the other vehicle performs a burn.
; CODE-ALONG READERS: Study coordinate transformation matrices (LV to REF),
;        orbital integration calls, and DSKY display interface for crew input.
; ============================================================================

# Page 709
; ORIGINAL NASA PROGRAM DESCRIPTION FOR P76 (TARGET DELTA V PROGRAM):
;
; 1) PROGRAM NAME - TARGET DELTA V PROGRAM (P76).
; 2) FUNCTIONAL DESCRIPTION - UPON ENTRY BY ASTRONAUT ACTION, P76 FLASHES DSKY REQUESTS TO THE ASTRONAUT
;    TO PROVIDE VIA DSKY (1) THE DELTA V TO BE APPLIED TO THE OTHER VEHICLE STATE VECTOR AND (2) THE
;    TIME (TIG) AT WHICH THE OTHER VEHICLE VELOCITY WAS CHANGED BY EXECUTION OF A THRUSTING MANEUVER. THE
;    OTHER VEHICLE STATE VECTOR IS INTEGRATED TO TIG AND UPDATED BY THE ADDITION OF DELTA V (DELTA V HAVING
;    BEEN TRANSFORMED FROM LV TO REF COSYS).  USING INTEGRVS, THE PROGRAM THEN INTEGRATES THE OTHER
;    VEHICLE STATE VECTOR TO THE STATE VECTOR OF THIS VEHICLE, THUS INSURING THAT THE W-MATRIX AND BOTH VEHICLE
;    STATES CORRESPOND TO THE SAME TIME.
; 3) ERASABLE INIITIALIZATION REQUIRED - NONE.
; 4) CALLING SEQUENCES AND EXIT MODES - CALLED BY ASTRONAUT REQUEST THRU DSKY V 37 E 76 E.
;    EXITS BY TCF ENDOFJOB.
; 5) OUTPUT -- OTHER VEHICLE STATE VECTOR INTEGRATED TO TIG AND INCREMENTED BY DELTA V IN REF COSYS.
;    THE PUSHLIST CONTAINS THE MATRIX BY WHICH THE INPUT DELTA V MUST BE POST-MULTIPLIED TO CONVERT FROM LV
;    TO REF COSYS.
; 6) DEBRIS - OTHER VEHICLE STATE VECTOR.
; 7) SUBROUTINES CALLED - BANKCALL, GOXDSPF, CSMPREC (OR LEMPREC), ATOPCSM (OR ATOPLEM), INTSTALL, INTWAKE, PHASCHNG
;    INTPRET, INTEGRVS, AND MINIRECT.
; 8) FLAG USE - MOONFLAG, CMOONFLG, INTYPFLG, RASFLAG, AND MARKCTR.
;
; ============================================================================
; MISSION CONTEXT: TARGET DELTA-V PROGRAM
;
; During rendezvous operations, the two spacecraft (Lunar Module and Command
; Module) maintain independent navigation state vectors. When one vehicle
; performs a thrusting maneuver, the other vehicle must update its tracking
; data to accurately predict the relative motion for rendezvous planning.
;
; P76 allows the crew to input the delta-V (velocity change) that the other
; vehicle just executed, along with the time of ignition (TIG). The program
; then updates the other vehicle's state vector by:
;   1. Integrating the other vehicle's trajectory to the maneuver time
;   2. Transforming the delta-V from local vertical to reference coordinates
;   3. Applying the velocity change to the state vector
;   4. Integrating both vehicles to a common time for synchronized tracking
;
; This program would be used during Apollo 11's rendezvous phase after Eagle
; ascended from the lunar surface and was maneuvering to dock with Columbia.
; ============================================================================

		BANK	30
		SETLOC	P76LOC
		BANK

		COUNT*	$$/P76

		EBANK=	TIG

; ============================================================================
; P76 PROGRAM ENTRY POINT
;
; The crew initiates this program by entering V37 E 76 E on the DSKY
; (Verb 37 = change program, followed by program number 76).
; ============================================================================

P76		TC	UPFLAG		; Set tracking flag to indicate we are
		ADRES	TRACKFLG	; actively tracking the other vehicle's state

; Initialize delta-V storage from last computed value.
; This provides a default starting point for crew input.

		TC	INTPRET		; Enter interpretive mode for vector operations
		VLOAD			; Load vector from DELVLVC (last delta-V in
			DELVLVC		; local vertical coordinates)
		STORE	DELVOV		; Store in DELVOV (delta-V for other vehicle)
		EXIT			; Return to native AGC code

; ============================================================================
; CREW INPUT PHASE: DELTA-V AND TIME OF IGNITION
;
; The program now displays two flashing requests to the crew on the DSKY,
; requesting input of:
;   1. Delta-V magnitude and components (Noun 84)
;   2. Time of ignition - TIG (Noun 33)
;
; The crew can respond with:
;   - TERMINATE (V34): Abort program and return to POO (idle program)
;   - PROCEED (V33): Accept displayed value and continue
;   - ENTER: Input new value and continue
; ============================================================================

		CAF	V06N84		; V06N84 = Verb 06 (display decimal),
					; Noun 84 (delta-V vector in ft/sec)
		TC	BANKCALL	; Flash display and wait for crew response
		CADR	GOFLASH		; GOFLASH handles flashing display logic
		TCF	ENDP76		; TERMINATE response: exit program
		TC	+2		; PROCEED response: accept value, continue
		TC	-5		; ENTER response: store new data, repeat flash
		
		CAF	V06N84 +1	; V06N33 = Verb 06 (display decimal),
					; Noun 33 (time of event in hrs:min:sec)
		TC	BANKCALL	; Flash TIG display and wait for crew response
		CADR	GOFLASH
		TCF	ENDP76		; TERMINATE: exit program
		TC	+2		; PROCEED: accept TIG, continue
		TC	-5		; ENTER: store new TIG, repeat flash
		
		TC	INTPRET		; Crew input complete, return to interpretive
					; code for trajectory computations
# Page 710
; ============================================================================
; ORBITAL INTEGRATION TO MANEUVER TIME
;
; The other vehicle's state vector must be integrated (propagated) forward
; to the time of ignition (TIG) so we can apply the delta-V at the correct
; point in its trajectory. This accounts for orbital motion during the coast
; phase before the burn.
; ============================================================================

		DLOAD			; Load TIG into MPAC (interpretive accumulator)
			TIG		; TIG in centiseconds, scaled by 2^28
		STCALL	TDEC1		; Store in TDEC1 for integration routines
			OTHPREC		; Call OTHPREC to integrate other vehicle
					; state to TIG (calls LEMPREC or CSMPREC
					; depending on which vehicle we're tracking)

; ============================================================================
; COORDINATE TRANSFORMATION MATRIX CONSTRUCTION
;
; The crew inputs delta-V in "local vertical" coordinates aligned with the
; other vehicle's orbital plane:
;   - Radial: along position vector (up/down)
;   - In-plane: perpendicular to R in orbital plane (forward/back) 
;   - Out-of-plane: perpendicular to orbital plane (normal)
;
; We must transform this to the reference coordinate system (inertial frame)
; used for state vector propagation. The transformation matrix is built from
; the other vehicle's position (R) and velocity (V) vectors at TIG.
;
; Matrix columns (stored in reverse order in pushdown list):
;   Column 1 (12D): U((RxV)xR) = radial direction, unit vector along R
;   Column 2 (18D): U(VxR) = in-plane direction, perpendicular to R
;   Column 3 (24D): U(-R) = out-of-plane direction (actually should be RxV)
;
; Note: Comments in original code suggest column 3 should be U(RxV), but
; implementation uses U(-R). This may be a historical artifact or alternate
; convention for local vertical coordinates.
; ============================================================================

COMPMAT		VLOAD	UNIT		; Load position vector RATT (at TIG)
			RATT		; and convert to unit vector
		VCOMP			; Complement to get -R direction
		STORE	24D		; Store U(-R) in pushdown location 24D
		
		VXV	UNIT		; U(-R) cross V = U(VxR), perpendicular
			VATT		; to both velocity and -R (in-plane direction)
		STORE	18D		; Store U(VxR) in pushdown location 18D
		
		VXV	UNIT		; U(VxR) cross U(-R) = U((VxR)x(-R))
			24D		; This completes the orthogonal triad
		STOVL	12D		; Store radial direction unit vector at 12D
			DELVOV		; Load crew-input delta-V vector (LV coords)

; Apply coordinate transformation and add to velocity state vector

		VXM	VSL1		; Multiply delta-V by transformation matrix
			12D		; (VXM: Vector X Matrix, VSL1: shift left 1)
					; Result is delta-V in reference coordinates
		VAD			; Add transformed delta-V to velocity at TIG
			VATT		; VATT = other vehicle velocity at TIG
		STORE	6		; Store updated velocity in pushdown loc 6
					; V(new) = V(TIG) + delta-V(REF)
		
; ============================================================================
; STATE VECTOR UPDATE AND INTEGRATION TO COMMON TIME
;
; With the delta-V successfully transformed to reference coordinates and
; added to the other vehicle's velocity, we now update the permanent state
; vector storage and integrate both vehicles to a common time. This ensures
; synchronized relative navigation data for rendezvous operations.
; ============================================================================

		CALL			; Stall orbital integration routines to
			INTSTALL	; prevent interference with state updates
					; (INTSTALL inhibits background integration)
		CALL
			P76SUB1		; Determine sphere of influence (Earth/Moon)
					; Sets scaling index in X2 register based on
					; MOONFLAG and CMOONFLG status

; Update other vehicle state vector with scaled position and velocity

		VLOAD	VSR*		; Load updated velocity from pushdown loc 6
			6		; (velocity at TIG + transformed delta-V)
			0,2		; Scale right by amount in X2 (0 or 2 bits)
		STOVL	VCV		; Store scaled velocity in VCV (other vehicle)
			RATT		; Load position at TIG (unchanged by delta-V
					; since delta-V is instantaneous velocity change)
		VSR*			; Scale position vector by same amount
			0,2		; (scaling depends on sphere of influence)
		STODL	RCV		; Store scaled position in RCV (other vehicle)
			TIG		; Load time of ignition
		STORE	TET		; Store as TET (time of state vector)

; Integrate other vehicle from TIG forward to this vehicle's state vector time

		CLEAR	DLOAD		; Clear integration type flag (precision mode)
			INTYPFLG	; INTYPFLG clear = precision integration
			TETTHIS		; Load time of THIS vehicle's state vector
INTOTHIS	STCALL	TDEC1		; Store as target integration time TDEC1
			INTEGRVS	; Call INTEGRVS to integrate other vehicle
					; from TIG (in TET) to TETTHIS (target time)
					; Result stored in RATT1/VATT1 temporaries

; Transfer integrated state to permanent storage locations

		CALL			; Re-stall integration for safe updates
			INTSTALL	; to permanent state vector storage
		VLOAD			; Load integrated position from RATT1
			RATT1		; (temporary integration result storage)
		STORE	RRECT		; Store in RRECT (permanent rectified position)
		STODL	RCV		; Also store in RCV (other vehicle position)
			TAT		; Load time at end of integration (actual time)
		STOVL	TET		; Store as TET (time of state vector)
			VATT1		; Load integrated velocity from VATT1
		CALL
			MINIRECT	; Convert state to proper coordinate format
					; and update permanent storage locations
# Page 711

; ============================================================================
; RESTART PROTECTION AND INTEGRATION WAKE-UP
;
; After successfully updating the other vehicle's state, we must set up
; restart protection (in case of power transient) and re-enable orbital
; integration routines that were stalled earlier. This ensures background
; integration can resume and maintains program restartability.
; ============================================================================

		EXIT			; Return to native AGC code from interpreter
		TC	PHASCHNG	; Establish restart phase change point
		OCT	04024		; Phase change code (Group 4, Phase 024)
					; Enables restart recovery if power lost

		TC	UPFLAG		; Set reintegration flag to signal
		ADRES	REINTFLG	; that orbital integration needs updating
					; (REINTFLG triggers background recalculation)

; Convert coordinate system and enable orbital integration

		TC	INTPRET		; Return to interpretive code for vector ops
		CALL
			ATOPOTH		; Convert coordinates "A to Other"
					; (transforms between this vehicle and other
					; vehicle coordinate representations)
		SSP	EXIT		; Store single precision: set QPRET register
			QPRET		; to return address OUT (program exit point)
			OUT		; Return address for subroutine linkage
		TC	BANKCALL	; Cross-bank call to wake integration
		CADR	INTWAKE1	; INTWAKE1: re-enables orbital integration
					; (previously stalled by INTSTALL calls)
					; Permits background integration to resume

; ============================================================================
; PROGRAM EXIT SEQUENCE
;
; P76 terminates after updating the other vehicle's state vector and
; re-enabling integration. Control returns to astronaut via POOH entry,
; allowing selection of next program or display mode.
; ============================================================================

OUT		EXIT			; Ensure in native code mode
ENDP76		CAF	ZERO		; Load zero into accumulator
		TS	MARKCTR		; Clear rendezvous radar tracking mark counter
					; (resets RR mark accumulation for next use)
		TCF	GOTOPOOH	; Transfer control to POOH job dispatcher
					; Displays PROG light, awaits crew input

; ============================================================================
; DSKY DISPLAY CONSTANTS FOR P76
;
; Verb/Noun combinations for crew data input during P76 execution.
; ============================================================================

V06N84		NV	0684		; Verb 06 Noun 84: Display for decimal
					; "Load component 1, 2, 3" - used for ΔV
					; input (three velocity components in fps)
		NV	0633		; Verb 06 Noun 33: Display for octal
					; "Time of event" - used for TIG input
					; (time of ignition in hours:min:sec)

; ============================================================================
; P76SUB1: SPHERE OF INFLUENCE DETERMINATION SUBROUTINE
;
; Determines whether this vehicle and the other vehicle are in Earth's or
; Moon's gravitational sphere of influence, and sets the X2 index register
; to the appropriate scaling factor (0 or 2 bit positions) for state vector
; storage. This scaling maintains numerical precision within the AGC's
; 15-bit word length while representing cislunar distances.
;
; SPHERE OF INFLUENCE LOGIC:
; - If MOONFLAG set: Moon is sphere of influence (lunar orbit operations)
; - If CMOONFLG set: CM permanent state is in lunar sphere (affects scaling)
; - X2 = 2 for Earth sphere (positions scaled larger, ~meters)
; - X2 = 0 for Moon sphere (positions scaled smaller for lunar distances)
;
; This routine is critical during rendezvous when vehicles may be in
; different spheres (e.g., during translunar or transearth coast).
; ============================================================================

P76SUB1		AXT,2	SET		; Execute two operations atomically:
			2		; 1) Set X2 index register to 2 (Earth scaling)
			MOONFLAG	; 2) Set MOONFLAG (assume Moon is sphere)
		BON	AXT,2		; Branch on flag: if CMOONFLG is set (ON)
			CMOONFLG	; Check if CM state in lunar sphere
			QPRET		; If set: return immediately (X2=2, correct)
			0		; If clear: set X2 to 0 (Moon sphere scaling)
		CLEAR	RVQ		; Clear MOONFLAG (Earth is actual sphere)
			MOONFLAG	; Return via Q register (RVQ = return)
