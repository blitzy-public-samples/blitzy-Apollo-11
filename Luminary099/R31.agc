# Copyright:	Public domain.
# Filename:	R31.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	703-708
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
; FILE: R31.agc
; MODULE: Rendezvous Navigation Programs
; MISSION PHASE: lunar-orbit/ascent/rendezvous
;
; TL;DR: R31 is the display routine called by Verb 83 that computes and 
;        displays relative orbital parameters between the Lunar Module and
;        Command Module during rendezvous operations. Calculates range,
;        range rate, and angular position (theta) between the two spacecraft
;        for crew monitoring during orbital rendezvous phases.
;
; COMMENT-ONLY READERS: This routine helps the crew monitor the distance and
;        closing velocity between Eagle and Columbia during rendezvous. The
;        displayed values updated continuously to track rendezvous progress.
; CODE-ALONG READERS: Study flag-based computation path selection (Average G,
;        Surface, Moon flags), conic vs precision integration, and coordinate
;        transformation for relative geometry calculations.
; ============================================================================

# Page 703
		BANK	40
		SETLOC	R31LOC
		BANK

		COUNT*	$$/R31

; ============================================================================
; R31CALL - VERB 83 ENTRY POINT
;
; When the crew enters Verb 83, this routine initiates the continuous
; display of relative navigation parameters between the LM and CSM.
; During Apollo 11's rendezvous after lunar ascent, Aldrin and Armstrong
; used these displays to monitor their approach to Collins in the
; Command Module Columbia.
; ============================================================================

R31CALL		CAF	PRIO3
		TC	FINDVAC		; Find vacant core set for V83CALL job
		EBANK=	SUBEXIT
		2CADR	V83CALL		; Start main V83 computation routine

; Display delay loop waits for display system to be ready for output.
; Checks BIT12 of EXTVBACT to determine if extended verb is still active.
; 100 centiseconds = 1 second delay between display updates.

DSPDELAY	TC	FIXDELAY	; Fixed delay routine
		DEC	100		; Wait 100 centiseconds (1 second)
		CA	EXTVBACT	; Load extended verb activity word
		MASK	BIT12		; Check if display update requested
		EXTEND
		BZF	DSPDELAY	; If not set, continue waiting

; Display is ready - schedule the display update task.

		CAF	PRIO5		; Priority 5 for display task
		TC	NOVAC		; Schedule new job (no VAC area needed)
		EBANK=	TSTRT
		2CADR	DISPN5X		; Display N5X routine

		TCF	TASKOVER	; Terminate this task

		BANK	37
		SETLOC	R31
		BANK
		COUNT*	$$/R31

; ============================================================================
; DISPN5X - DISPLAY RENDEZVOUS PARAMETERS
;
; Displays V16N54 showing relative position parameters:
; R1 = RANGE (distance between spacecraft in nautical miles)
; R2 = RRATE (range rate - closing velocity in feet per second)
; R3 = RTHETA (angle theta in degrees)
; Crew monitors these values to verify rendezvous trajectory.
; ============================================================================

DISPN5X		CAF	V16N54		; Verb 16 Noun 54 display code
		TC	BANKCALL	; Cross-bank call to display system
		CADR	GOMARKF		; Display interface routine
		TC	B5OFF		; Bit 5 off return (terminate request)
		TC	B5OFF		; Bit 5 off return (proceed)
		TCF	DISPN5X		; Loop - continue displaying

; ============================================================================
; V83CALL - MAIN COMPUTATION LOGIC
;
; Determines which computation path to use based on mission mode flags:
; - AVEGFBIT: Average G flag (set during powered flight phases)
; - SURFFBIT: Surface flag (set when LM is on lunar surface)
; - MOONFLAG: Moon flag (determines Earth vs Moon gravity model)
;
; The routine computes relative state vectors and converts them to
; range, range rate, and theta angle for display to the crew.
; ============================================================================

V83CALL		CS	FLAGWRD7	# TEST AVERAGE G FLAG
		MASK	AVEGFBIT	; Check if Average G guidance active
		EXTEND
		BZF	MUNG?		# ON - TEST MUNFLAG

; Average G is off - check if LM is on lunar surface.

		CS	FLAGWRD8
		MASK	SURFFBIT	; Check surface flag
		EXTEND
		BZF	ONEBASE		# ON SURFACE - BYPASS LEMPREC

; LM is not on surface - perform precision extrapolation for both vehicles.
; This path used during most of rendezvous when both vehicles are in orbit.

		TC	INTPRET		# EXTRAPOLATE BOTH STATE VECTORS
		RTB
# Page 704
			LOADTIME	; Load current mission time
		STCALL	TDEC1		; Store in TDEC1, call LEMPREC
			LEMPREC		# PRECISION BASE VECTOR FOR LM

; LEMPREC performs precision numerical integration of the LM state vector
; accounting for lunar gravity harmonics and perturbations. Returns
; position in RATT1 and velocity in VATT1 at requested time.

		VLOAD
			RATT1		; Load LM position vector
		STOVL	BASETHP		; Store as "this" vehicle (LM) position
			VATT1		; Load LM velocity vector
		STODL	BASETHV		; Store as "this" vehicle velocity
			TAT		; Load time of position/velocity

; Now compute precision state vector for CSM (the "other" vehicle).
; During Apollo 11 rendezvous, this tracked Columbia's orbital position
; while Eagle approached from below after lunar ascent.

DOCMBASE	STORE	BASETIME	# PRECISION BASE VECTOR FOR CM
		STCALL	TDEC1		; Store time, call CSMPREC
			CSMPREC		; Precision integration for CSM

		VLOAD
			RATT1		; Load CSM position vector
		STOVL	BASEOTP		; Store as "other" vehicle position
			VATT1		; Load CSM velocity vector
		STORE	BASEOTV		; Store as "other" vehicle velocity
		EXIT			; Return to native AGC instructions

; ============================================================================
; TRANSITION: From precision extrapolation to repeated computation cycle
;
; After computing initial base vectors, R31 enters a continuous update loop.
; Every display cycle (approximately 1 second), the routine recalculates
; relative parameters to provide real-time rendezvous monitoring. The crew
; watches these values change as the spacecraft approach each other.
; ============================================================================

REV83		CS	FLAGWRD7
		MASK	AVEGFBIT	; Test Average G flag again
		EXTEND
		BZF	GETRVN		# IF AVEGFLAG SET, USE RN,VN

; Average G is off - check if on surface.

		CS	FLAGWRD8
		MASK	SURFFBIT	; Test surface flag
		EXTEND
		BZF	R31SURF		# IF ON SURFACE,USE LEMAREC

; Neither Average G nor Surface flag set - use conic extrapolation.
; This is faster than precision integration, suitable for display updates.
; Uses two-body Keplerian orbital mechanics (conic section trajectories).

		TC	INTPRET		# DO CONIC EXTRAPOLATION FOR BOTH VEHICLES
		RTB
			LOADTIME	; Get current time
		STCALL	TDEC1		; Store time, initialize integration
			INTSTALL	; Set up integration parameters

; Extrapolate "this" vehicle (LM) using conic integration.

		VLOAD	CLEAR
			BASETHP		; Load LM base position
			MOONFLAG	; Clear moon flag initially
		STOVL	RCV		; Store as reference position
			BASETHV		; Load LM base velocity
		STODL	VCV		; Store as reference velocity
			BASETIME	; Load base vector time
		BOF	SET		# GET APPROPRIATE MOONFLAG SETTING
			MOONTHIS	; Branch if this vehicle is moon-centric
			+2		; Skip moon flag set
			MOONFLAG	; Set moon flag for lunar gravity
		SET
			INTYPFLG	# CONIC EXTRAP.
		STCALL	TET		; Store integration time
			INTEGRVS	# INTEGRATION --- AT LAST---

; LM state vector now extrapolated to current time in RATT/VATT.

OTHCONIC	VLOAD
# Page 705
			RATT		; LM position at current time
		STOVL	RONE		; Store as vehicle one position
			VATT		; LM velocity at current time
		STCALL	VONE		# GET SET FOR CONIC EXTRAP.,OTHER.
			INTSTALL	; Initialize for CSM integration

; Now extrapolate "other" vehicle (CSM) using same conic method.

		SET	DLOAD
			INTYPFLG	; Set conic integration flag
			TAT		; Load time at attitude (current)
OTHINT		STORE	TDEC1		; Store as integration time
		VLOAD	CLEAR
			BASEOTP		; Load CSM base position
			MOONFLAG	; Clear moon flag initially
		STOVL	RCV		; Store as reference position
			BASEOTV		; Load CSM base velocity
		STODL	VCV		; Store as reference velocity
			BASETIME	; Load base vector time
		BOF	SET
			MOONTHIS	; Branch if "other" is moon-centric
			+2		; Skip moon flag set
			MOONFLAG	; Set moon flag for lunar gravity
		STCALL	TET		; Store integration time
			INTEGRVS	; Integrate CSM state vector

; ============================================================================
; COMPDISP - COMPUTE RELATIVE GEOMETRY FOR DISPLAY
;
; With both vehicles' state vectors at the same time, now compute the
; three display parameters:
; - RANGE: Distance between spacecraft (nautical miles)
; - RRATE: Range rate - rate of change of range (feet/second)  
; - RTHETA: Angle theta measured from Z-axis in navigation base (degrees)
;
; These values provide the crew with complete relative geometry information.
; ============================================================================

COMPDISP	VLOAD	VSU
			RATT		; Load CSM position
			RONE		; Subtract LM position
		RTB	PDDL		; Get unit vector and magnitude
			NORMUNX1	# UNIT(RANGE) TO PD 0-5
			36D		; Magnitude in 36D
		SL*			# RESCALE AFTER NORMUNIT
			0,1		; Shift count from NORMUNIT
		STOVL	RANGE		# SCALED 2(29)M - store range magnitude
			VATT		; Load CSM velocity
		VSU	DOT		# (VCM- VLM).UNIT(LOS). PD=0
			VONE		; Subtract LM velocity
		SL1			# SCALED 2(7)M/CS
		STOVL	RRATE		; Store range rate (closing velocity)
			RONE		; Load LM position

; Now compute theta angle - angular position relative to navigation base.
; This involves transforming to stable member coordinates and computing
; the angle between the line-of-sight and the spacecraft Z-axis.

		UNIT	PDVL		# UNIT(R) TO PD 0-5
			UNITZ		; Unit Z vector
		CALL
			CDU*NBSM	; Transform by CDU angles to navigation base
		VXM	PUSH		# UNIT (Z)/4 TO PD 6-11
			REFSMMAT	; Reference stable member matrix
		VPROJ	VSL2		# UNIT(P)=UNIT(UZ -(UZ)PROJ(UR))
			0D		; Project onto unit range vector
		BVSU	UNIT		; Subtract and normalize
			6D
		PDVL	VXV		# UNIT(P) TO PD 12-17
			0D		# UNIT(RL)
			VONE		; LM velocity
# Page 706
		VXV	DOT		# (UR * VL)*UR . U(P)
			0D		; Unit range
			12D		; Unit P vector
		PDVL			# SIGN TO 12-13 , LOAD U(P)
		DOT	SIGN		; Determine sign of angle
			6D
			12D
		SL2	ACOS		# ARCCOS(UP.UZ(SIGN))
		STOVL	RTHETA		; Store theta angle (scaled revolutions)
			0D		; Unit range vector
		DOT	BPL		# IF UR.UZ NEG,
			6D		#   RTHETA = 1 - RTHETA
			+5		; Skip complement if positive
		DLOAD	DSU		; Load maximum positive
			DPPOSMAX	; Maximum DP value
			RTHETA		; Subtract theta
		STORE	RTHETA		; Store complemented angle
		EXIT			; Return to native instructions

; ============================================================================
; DISPLAY TERMINATION CHECK
;
; After computing and displaying the relative parameters (range, range rate,
; theta), check if the crew has terminated the Extended Verb display. If
; not terminated, loop back to REV83 to recompute with updated positions
; using faster conic extrapolation (1-second update cycle).
; ============================================================================

		CA	BIT5		; Check Extended Verb status
		MASK	EXTVBACT	; Test if display is still active
		EXTEND			# IF ANSWERED,
		BZF	ENDEXT		#	 TERMINATE
					; Crew has terminated display

; If display still active, set up for next iteration using conic
; extrapolation (much faster than precision integration for updates).

		CS	EXTVBACT	; Complement Extended Verb flag
		MASK	BIT12		; Isolate bit 12
		ADS	EXTVBACT	# SET BIT 12
		TCF	REV83		# AND START AGAIN.
					; Loop back for 1-second update

; ============================================================================
; ALTERNATE COMPUTATION PATHS
;
; The following sections handle special cases where the standard computation
; path (precision integration for both vehicles) is not appropriate:
;
; GETRVN:   Used when MUNFLAG is set (Moon) or precision data not needed
; R31SURF:  Used when LM is on the lunar surface (SURFFBIT set)
; MUNG?:    Dispatcher that tests MUNFLAG to select computation path
; ONEBASE:  Gets CSM base vector when only one vehicle needs precision
; ============================================================================

; ============================================================================
; GETRVN - Alternate State Vector Acquisition Path
;
; This routine is used when MUNFLAG is set (lunar sphere of influence) or
; when the CSM precision base vector is not needed. It directly uses the
; current LM state vector from RN,VN and CSM state from R(CSM),V(CSM)
; without performing precision integration for the CSM.
;
; COMMENT-ONLY READERS: This faster path is taken when the spacecraft are
;        close enough that simple position/velocity updates are sufficient.
; CODE-ALONG READERS: Note priority change to inhibit servicer during state
;        vector copying, then test of MUNFLAG to select reference frame.
; ============================================================================

GETRVN		CA	PRIO22		# INHIBIT SERVICER
		TC	PRIOCHNG	; Change to high priority
		TC	INTPRET		; Enter interpretive mode
		VLOAD	SETPD		; Load LM position vector
			RN		# LM STATE VECTOR IN RN,VN
			0		; Set pushdown pointer to 0
		STOVL	RONE		; Store as "vehicle one" position
			VN		; Load LM velocity vector
		STOVL	VONE		# LOAD R(CSM),V(CSM) IN CASE MUNFLAG SET
			V(CSM)		# (TO INSURE TIME COMPATABILITY)
		PDVL	PDDL		; Push CSM velocity, load position
			R(CSM)		; CSM position
			PIPTIME		; Load current time from IMU
		EXIT			; Return to native instructions

		CA	PRIO3		; Restore normal priority
		TC	PRIOCHNG	; Priority change
		TC	INTPRET		; Back to interpretive mode
		BOFF	VLOAD		; Test MUNFLAG
			MUNFLAG		; If clear, do precision for CSM
			GETRVN2		# IF MUNFLAG RESET, DO CM DELTA PRECISION
					; MUNFLAG set - in lunar sphere

; When MUNFLAG is set (lunar sphere of influence), transform CSM state
; vector from reference coordinates to relative display coordinates.
# Page 707
		VXM	VSR4		# CHANGE TO REFERENCE SYSTEM AND RESCALE
			REFSMMAT	; Reference to stable member matrix
		PDVL			# R TO PD 0-5
		VXM	VSL1		; Transform velocity vector
			REFSMMAT	; Apply same transformation
		PUSH	SETPD		# V TO PD 5-11
			0		; Reset pushdown pointer
		GOTO			; Jump to relative computation
			COMPDISP	; Compute and display parameters

; When MUNFLAG is clear (Earth sphere or trans-lunar), perform precision
; integration for the CSM (other vehicle) to get accurate relative state.

GETRVN2		CALL			; Call integration setup
			INTSTALL	; Install integration parameters
		CLEAR	GOTO		; Clear integration type flag
			INTYPFLG	# PREC EXTRAP FOR OTHER
			OTHINT		; Perform CSM precision integration

; ============================================================================
; R31SURF - Surface Operations Path
;
; This path is taken when the LM is on the lunar surface (SURFFBIT set).
; Since the LM is stationary on the surface, only the CSM state needs to
; be extrapolated. Uses LEMPREC for surface-to-inertial transformation,
; then conic extrapolation for the orbiting CSM.
;
; COMMENT-ONLY READERS: After lunar landing, Eagle sits on the surface while
;        Columbia continues orbiting. This routine tracks Columbia's position
;        for the eventual ascent and rendezvous.
; CODE-ALONG READERS: Note use of LEMPREC for surface orientation, followed
;        by OTHCONIC for simple conic propagation of the orbiting CSM.
; ============================================================================

R31SURF		TC	INTPRET		; Enter interpretive mode
		RTB			# LM IS ON SURFACE, SO PRECISION
			LOADTIME	# INTEGRATION USES PLANETARY INERTIAL
		STCALL	TDEC1		# ORIENTATION SUBROUTINE
			LEMPREC		; Get LM surface position/orientation
		GOTO			# DO CSM CONIC
			OTHCONIC	; Extrapolate CSM using conic

; ============================================================================
; MUNG? - MUNFLAG Test Dispatcher
;
; Tests MUNFLAG to determine if CSM precision base vector is needed.
; If MUNFLAG is set (lunar sphere), CSM base not needed - go to GETRVN.
; If MUNFLAG is clear (Earth sphere or trans-lunar), get CSM base via ONEBASE.
; ============================================================================

MUNG?		CS	FLAGWRD6	; Test for Moon flag
		MASK	MUNFLBIT	; Isolate MUNFLAG bit
		EXTEND			; Extended instruction follows
		BZF	GETRVN		# IF MUNFLAG SET, CSM BASE NOT NEEDED
					; Jump to simplified path

; ============================================================================
; ONEBASE - Single Precision Base Vector
;
; Gets precision base vector for CSM only (when LM is on surface or
; MUNFLAG requires CSM precision but not LM precision). Loads current
; time and jumps to DOCMBASE to compute CSM precision state vector.
; ============================================================================

ONEBASE		TC	INTPRET		# GET CSM BASE VECTOR
		RTB	GOTO		; Load time and jump
			LOADTIME	; Get current mission time
			DOCMBASE	; Compute CSM precision base

; ============================================================================
; DISPLAY FORMAT CONSTANT
;
; V16N54: Verb 16, Noun 54 - Display relative orbital parameters
;         R1: Range (nautical miles)
;         R2: Range rate (feet per second)
;         R3: Theta angle (degrees)
;
; This display format is used throughout R31 to show the crew the current
; separation distance, closing velocity, and angular position between the
; two spacecraft during rendezvous operations.
; ============================================================================

V16N54		VN	1654		; Verb 16 Noun 54 display code

# Page 708 (empty page)
