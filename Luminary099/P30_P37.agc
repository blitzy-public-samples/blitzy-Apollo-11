# Copyright:	Public domain.
# Filename:	P30_P37.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	614-617
# Mod history:	2009-05-17 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-05 RSB	Removed 4 lines of code that shouldn't
#				have survived from Luminary 131.
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
; FILE: P30_P37.agc
; MODULE: External Delta-V Programs
; MISSION PHASE: lunar-orbit/ascent/rendezvous/trans-earth
;
; TL;DR: Programs P30-P37 compute velocity changes (ΔV) needed for orbital
;        maneuvers and generate burn parameters for engine execution. Accept
;        crew inputs of ignition time and desired velocity change, propagate
;        current orbit to the ignition time, compute resulting orbit parameters
;        (apogee, perigee), and prepare for burn program execution via P40-P47.
;        Critical for rendezvous maneuvers during Eagle-Columbia reunion.
;
; COMMENT-ONLY READERS: These programs computed the precise velocity changes
;        needed for Eagle to rendezvous with Columbia after lunar ascent. Read
;        to understand how orbital maneuvers were planned and displayed to crew.
; CODE-ALONG READERS: Study the coordinate transformations from local vertical
;        to inertial frames, orbit propagation via LEMPREC, and integration
;        with PERIAPO orbital parameter computation.
; ============================================================================

# Page 614
# PROGRAM DESCRIPTION P30	DATE 3-6-67
#
# MOD.1 BY RAMA AIYAWAR
# FUNCTIONAL DESCRIPTION
#	ACCEPT ASTRONAUT INPUTS OF TIG.DELV(LV)
#	CALL IMU STATUS CHECK ROUTINE (R02)
#	DISPLAY TIME TO GO, APOGEE, PERIGEE, DELV(MAG), MGA AT IGN
#	REQUEST BURN PROGRAM
#
# CALLING SEQUENCE VIA JOB FROM V37
#
# EXIT VIA V37 CALL OR TO GOTOPOOH (V34E)
#
# SUBROUTINE CALLS-FLAGUP, PHASCHNG, BANKCALL, ENDOFJOB, GOFLASH, GOFLASHR
#		   GOPERF3R, INTPRET, BLANKET, GOTOPOOH, R02BOTH, S30.1,
#		   TIG/N35, MIDGIM, DISPMGA
#
# ERASABLE INITIALIZATION- STATE VECTOR
#
# OUTPUT-RINIT, VINIT, +MGA, VTIG, RTIG, DELVSIN, DELVSAB, DELVSLV, HAPO,
#	 HPER, TTOGO
#
# DEBRIS- A,L, MPAC, PUSHLIST

; ============================================================================
; PROGRAM P30: EXTERNAL DELTA-V TARGETING
;
; When the Lunar Module needs to perform an orbital maneuver, the crew uses
; P30 to plan the burn. During Apollo 11's rendezvous, this program computed
; the precise velocity changes for CSI (Coelliptic Sequence Initiation),
; CDH (Constant Delta Height), and TPI (Terminal Phase Initiation) maneuvers
; that brought Eagle back to Columbia.
;
; The crew enters two critical parameters via DSKY:
; 1. TIG (Time of Ignition): When the burn should occur
; 2. DELV (Delta Velocity): Desired velocity change in local vertical coords
;
; P30 then propagates the current orbit forward to TIG, transforms the
; velocity change from local vertical coordinates to inertial coordinates,
; computes the resulting orbit, and displays the apogee, perigee, and
; total ΔV magnitude to the crew for review before proceeding to the actual
; burn program (P40-P47).
; ============================================================================

		BANK	32
		SETLOC	P30S
		BANK
		EBANK=	+MGA
		COUNT*	$$/P30
		
; P30 PROGRAM ENTRY POINT
; The crew invokes P30 when planning an external ΔV maneuver. The program
; begins by setting flags to indicate that maneuver parameters are being
; updated and that the guidance system should track the target.

P30		TC	UPFLAG		# SET UPDATE FLAG
		ADRES	UPDATFLG
		TC	UPFLAG		# SET TRACK FLAG
		ADRES	TRACKFLG

; ASTRONAUT INPUT SEQUENCE
; P30 requests two key inputs from the crew via DSKY displays:
; V06N33 displays the time of ignition (TIG) for crew input
; V06N81 displays the desired ΔV in local vertical coordinates

P30N33		CAF	V06N33		# T OF IGN
		TC	VNPOOH		# RETURNS ON PROCEED, POOH ON TERMINATE
		
; After TIG is entered, request the desired velocity change.
; DELV is specified in the local vertical coordinate system centered at
; the spacecraft: X-axis = radial (up), Y-axis = downrange, Z-axis = cross-track.
; This coordinate system is intuitive for astronauts planning maneuvers.

		CAF	V06N81		# DISPLAY DELTA V (LV)
		TC	VNPOOH		#     REDISPLAY ON RECYCLE

; COMPUTE MANEUVER PARAMETERS
; With TIG and DELV known, call subroutine S30.1 to propagate the orbit
; to TIG, transform DELV to inertial coordinates, compute the resulting
; orbit parameters, and prepare for crew display.

		TC	DOWNFLAG	# RESET UPDATE FLAG
		ADRES	UPDATFLG
		TC	INTPRET
		CALL
			S30.1
		SET	EXIT
			UPDATFLG
			
; DISPLAY COMPUTED RESULTS TO CREW
; Show apogee altitude, perigee altitude, and total ΔV magnitude.
; The crew reviews these parameters to verify the maneuver will achieve
; the desired orbit before committing to the burn.
; Historical note: During Apollo 11 rendezvous, Aldrin monitored these
; displays while Armstrong prepared for each rendezvous maneuver.

PARAM30		CAF	V06N42		# DISPLAY APOGEE,PERIGEE ,DELTA V
		TC	VNPOOH
# Page 615

; TRANSITION TO BURN PROGRAM
; After crew approval of the computed parameters, P30 sets the external
; delta-V flag (XDELVFLG) to inform the burn execution programs (P40-P47)
; that they should use the externally-specified ΔV rather than computing
; their own guidance solution. Control then transfers to display the
; middle gimbal angle and prepare for burn execution.

		TC	INTPRET
		SETGO
			XDELVFLG	# FOR P40'S: EXTERNAL DELTA-V GUIDANCE.
			REVN1645	# TRKMKCNT, T60, +MGA  DISPLAY

; DSKY VERB/NOUN CODES USED BY P30
; V06N33: Verb 06 (Display Decimal) with Noun 33 (Time of Event)
; V06N42: Verb 06 (Display Decimal) with Noun 42 (Apogee/Perigee/Delta-V)
; V06N81: Verb 06 (Display Decimal) with Noun 81 (Delta-V Local Vertical)

V06N33		VN	0633
V06N42		VN	0642

# Page 616
# PROGRAM DESCRIPTION S30.1	DATE 9NOV66
# MOD NO 1			LOG SECTION P30,P37
# MOD BY RAMA AIYAWAR **
# FUNCTIONAL DESCRIPTION
#	BASED ON STORED TARGET PARAMETERS(R OF IGNITION(RTIG),V OF
#	IGNITION(VTIG),TIME OF IGNITION (TIG)),COMPUTE PERIGEE ALTITUDE
#	APOGEE ALTITUDE AND DELTAV REQUIRED(DELVSIN).
# CALLING SEQUENCE
#	L	CALL
#	L+1		s30.1
# NORMAL EXIT MODE
#	AT L+2 OR CALLING SEQUENCE (GOTO L+2)
# SUBROUTINES CALLED
#	LEMPREC
#	PERIAPO
# ALARM OR ABORT EXIT MODES
#	NONE
# ERASABLE INITIALIZATION REQUIRED
#	TIG		TIME OF IGNITION	DP B28CS
#	DELVSLV		SPECIFIED DELTA-V IN LOCAL VERT.
#			COORDS. OF ACTIVE VEHICLE AT
#			TIME OF IGNITION	VECTOR 	B+7 METERS/CS
#
# OUTPUT
#	RTIG		POSITION AT TIG		VECTOR 	B+29 METERS
#	VTIG		VELOCITY AT TIG		VECTOR 	B+29 METERS/CS
#	PDL 4D		APOGEE ALTITUDE		DP 	B+29 M ,  B+27 METERS.
#	HAPO		APOGEE ALTITUDE		DP 	B+29 METERS
#	PDL 8D		PERIGEE ALTITUDE	DP 	B+29 M ,  B+27 METERS.
#	HPER		PERIGEE ALTITUDE	DP 	B+29 METERS
#	DELVSIN		SPECIFIED DELTA-V IN INTERTIAL
#			COORD. OF ACTIVE VEHICLE AT
#			TIME OF IGNITION	VECTOR 	B+7 METERS/CS
#	DELVSAB		MAG. OF DELVSIN		VECTOR 	B+7 METERS/CS
#
# DEBRIS	QTEMP	TEMP. ERASABLE
#		QPRET,MPAC
#		PUSHLIST

; ============================================================================
; SUBROUTINE S30.1: MANEUVER PARAMETER COMPUTATION
;
; S30.1 performs the mathematical core of P30's maneuver planning:
;
; 1. ORBIT PROPAGATION: Uses LEMPREC (Encke precision integrator) to
;    propagate the current state vector from present time to TIG, accounting
;    for lunar gravity and orbital perturbations. This gives RTIG (position)
;    and VTIG (velocity) at the planned ignition time.
;
; 2. COORDINATE TRANSFORMATION: Transforms the crew-entered ΔV from local
;    vertical coordinates (intuitive for astronauts) to inertial coordinates
;    (needed for orbit computation). Local vertical frame is defined by:
;    - X-axis (ZRF): Radial direction (toward/away from Moon center)
;    - Y-axis (XRF): Cross-product of velocity and position (orbit normal)
;    - Z-axis (YRF): Completes right-handed system (roughly downrange)
;
; 3. ORBIT ANALYSIS: Computes the resulting orbit after applying ΔV by
;    calling PERIAPO, which determines apogee and perigee altitudes using
;    the vis-viva equation and orbital energy analysis.
;
; All position vectors scaled at 2^29 meters (~1.862 nanometers per bit).
; All velocity vectors scaled at 2^7 meters/centisecond (~0.78 cm/s per bit).
; These scaling factors maximize precision within AGC's 15-bit word length.
; ============================================================================

		SETLOC	P30S1
		BANK

		COUNT*	$$/S30S

; S30.1 ENTRY POINT: COMPUTE ORBIT AT TIG
; The first task is to determine where the spacecraft will be at the planned
; ignition time. This requires propagating the current orbit forward in time
; using precision numerical integration.

S30.1		STQ	DLOAD
			QTEMP
			TIG		# TIME IGNITION SCALED AT 2(+28)CS
		STCALL	TDEC1
			LEMPREC		# ENCKE ROUTINE FOR LEM
			
; LEMPREC is the Encke precision orbit propagator, which integrates the
; equations of motion accounting for lunar gravity. It propagates from the
; current time to TIG, computing RATT (position at time) and VATT (velocity
; at time). The Encke method maintains precision by integrating perturbations
; from a reference conic orbit rather than the full equations.

; ============================================================================
; COORDINATE FRAME CONSTRUCTION
; The crew entered ΔV in local vertical (LV) coordinates, which are intuitive
; for spacecraft operations. Now we must transform this to inertial coordinates
; for orbit computation. The local vertical frame is defined by three unit
; vectors:
;   ZRF (Z-axis, Radial): Points from Moon center toward spacecraft
;   YRF (Y-axis, Downrange): Perpendicular to orbit plane
;   XRF (X-axis, Cross-track): Completes right-handed coordinate system
; ============================================================================

		VLOAD	SXA,2
# Page 617
			RATT
			RTX2
		STORE	RTIG		# RADIUS VECTOR AT IGNITION TIME
		
; UNIT normalizes RTIG to unit length, creating the radial unit vector.
; VCOMP reverses its direction (toward Moon center, not away from).
; This becomes ZRF, stored temporarily in DELVSIN.
		UNIT	VCOMP
		STOVL	DELVSIN		# ZRF/LV IN DELVSIN SCALED AT 2
			VATT		# VELOCITY VECTOR AT TIG, SCALED 2(7) M/CS
		STORE	VTIG
		
; Compute XRF: velocity cross position gives orbit normal vector.
; The cross product V × R produces a vector perpendicular to the orbital
; plane, pointing in the direction of orbital angular momentum.
		VXV	UNIT
			RTIG
		SETPD	SXA,1
			0
			RTX1
			
; Store XRF on push-down list (PDL) at location 0.
; Now compute YRF: XRF × ZRF completes the right-handed coordinate system.
; YRF points roughly in the velocity direction (downrange).
		PUSH	VXV		# YRF/LV PDL 0  SCALED AT 2
			DELVSIN
			
; Left shift 1 bit to restore proper scaling after cross product.
; Push YRF and ZRF onto PDL to build the transformation matrix.
		VSL1	PDVL
		PDVL	PDVL		# YRF/LV PDL 6 SCALED AT 2
			DELVSIN		# ZRF/LV PDL 12D SCALED AT 2
			DELVSLV
			
; ============================================================================
; TRANSFORM ΔV FROM LOCAL VERTICAL TO INERTIAL COORDINATES
; The crew entered ΔV in local vertical coordinates (easier to visualize).
; Now transform to inertial coordinates using the rotation matrix built
; from XRF, YRF, ZRF. VXM (vector times matrix) performs the transformation.
; ============================================================================
			
		VXM	VSL1
			0
		STORE	DELVSIN		# DELTAV IN INERT. COOR. SCALED TO B+7M/CS
		
; DELVSIN now contains the ΔV in inertial coordinates. This is the actual
; velocity change that will be applied at TIG. Compute its magnitude for
; display to the crew so they can verify the maneuver size.
		ABVAL
		STOVL	DELVSAB		# DELTA V MAG.
			RTIG		# (FOR PERIAPO)
			
; ============================================================================
; COMPUTE RESULTING ORBIT PARAMETERS
; With ΔV known, compute the orbit that will result after the burn. The crew
; needs to see the apogee and perigee altitudes to verify the maneuver will
; achieve the desired orbit. PERIAPO1 computes orbital elements from state.
; ============================================================================
			
; Compute the velocity after the burn: VREQUIRED = VTIG + DELVSIN
; This is the velocity the spacecraft will have immediately after ignition.
		PDVL	VAD		# VREQUIRED = VTIG + DELVSIN (FOR PERIAPO)
			VTIG
			DELVSIN
			
; Call PERIAPO1 to compute apogee and perigee from position and velocity.
; PERIAPO1 uses the vis-viva equation and conservation of angular momentum
; to determine the orbital elements without full integration.
		CALL
			PERIAPO1
			
; Process perigee altitude for display. SHIFTR1 rescales the value if needed
; to fit display format. MAXCHK limits the display to 9999.9 nautical miles
; to prevent overflow on the DSKY.
		CALL
			SHIFTR1		# RESCALE IF NEEDED
		CALL			# LIMIT DISPLAY TO 9999.9 N. MI.
			MAXCHK
		STODL	HPER		# PERIGEE ALT 2(29) METERS, FOR DISPLAY
			4D
			
; Process apogee altitude for display using the same rescaling and limiting.
		CALL
			SHIFTR1		# RESCALE IF NEEDED
		CALL			# LIMIT DISPLAY TO 9999.9 N. MI.
			MAXCHK
		STCALL	HAPO		# APOGEE  ALT 2(29) METERS, FOR DISPLAY
			QTEMP

