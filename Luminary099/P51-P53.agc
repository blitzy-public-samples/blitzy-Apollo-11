# Copyright:	Public domain.
# Filename:	P51-P53.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	926-983
# Mod history:	2009-05-31 HG	Transcribed from page images.
#		2009-06-07 RSB	Corrected a typo.
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
; FILE: P51-P53.agc
; MODULE: IMU Alignment Programs
; MISSION PHASE: lunar-orbit/descent/landing/ascent
;
; TL;DR: Inertial Measurement Unit (IMU) alignment programs that establish
;        or verify the IMU platform orientation relative to known inertial
;        reference frames using star sightings. Contains PROG52 (main IMU
;        alignment with multiple orientation options), P51 (IMU orientation
;        determination), and P57 (lunar surface alignment). Regular IMU
;        alignments were critical throughout Apollo 11's lunar orbit
;        operations to maintain navigation accuracy and compensate for
;        gyro drift accumulated during the mission.
;
; COMMENT-ONLY READERS: This file contains the star sighting and alignment
;        procedures that astronauts used to ensure the navigation system
;        knew exactly which direction the spacecraft was pointing. Read to
;        understand how Aldrin and Armstrong maintained navigation accuracy.
; CODE-ALONG READERS: Study the coordinate transformations, gimbal angle
;        computations, and star catalog usage. Note integration with AOT
;        (Alignment Optical Telescope) hardware and IMU compensation routines.
; ============================================================================

# Page 926
; ============================================================================
; PROGRAM NAME: PROG52 (IMU Realignment Program)
;
; MISSION CONTEXT:
; During lunar orbit operations, the Inertial Measurement Unit (IMU) 
; accumulates small errors due to gyro drift. PROG52 allows the crew to 
; realign the IMU to a known orientation using star sightings through the
; Alignment Optical Telescope (AOT). This was essential for maintaining
; navigation accuracy throughout Apollo 11's mission, especially before
; critical maneuvers like the descent orbit insertion and lunar landing.
;
; COMMENT-ONLY READERS:
; Think of the IMU as the spacecraft's sense of orientation in space. Over
; time, tiny errors accumulate (gyro drift). The astronauts use this program
; to "recalibrate" the IMU by sighting known stars through a telescope,
; essentially asking "Where are we pointing now?" and comparing that to
; "Where should known stars appear?" This corrects accumulated errors.
;
; CODE-ALONG READERS:
; PROG52 orchestrates a complete IMU realignment sequence: orientation
; selection → gimbal angle computation → coarse alignment → star selection
; → fine alignment using star sightings. Study the coordinate transformation
; logic and integration with IMU hardware control routines.
; ============================================================================

# PROGRAM NAME -- PROG52			DATE -- JAN 9, 1967
# MOD NO -- 0					LOG SECTION -- P51-P53
# MODIFICATION BY -- LONSKE			ASSEMBLY -- SUNDANCE REV 46
#
# FUNCTIONAL DESCRIPTION --
#
#	ALIGNS THE IMU TO ONE OF THREE ORIENTATIONS SELECTED BY THE ASTRONAUT.  THE PRESENT IMU ORIENTATION IS KNOWN
#	AND IS STORED IN REFSMMAT.  THE THREE POSSIBLE ORIENTATIONS MAY BE:
#
#	(A)	PREFERRED ORIENTATION
#
#		AN OPTIMUM ORIENTATION FOR A PREVIOUSLY CALCULATED MANEUVER.  THIS ORIENTATION MUST BE CALCULATED AND
#		STORED BY A PREVIOUSLY SELECTED PROGRAM.
#
#	(B)	NOMINAL ORIENTATION
#
#		X   = UNIT ( R )
#		-SM
#
#		Y   = UNIT (V X R)
#		 SM
#
#		Z   = UNIT (X   X Y  )
#		 SM          SM    SM
#
#		WHERE:
#
#		R = THE GEOCENTRIC RADIUS VECTOR AT TIME T(ALIGN) SELECTED BY THE ASTRONAUT
#		-
#
#		V = THE INERTIAL VELOCITY VECTOR AT TIME T(ALIGN) SELECTED BY THE ASTRONAUT
#		-
#
#	(C)	RERSMMAT ORIENTATION
#
#	(D)	LANDING SITE -- THIS IS NOT AVAILABLE IN SUNDANCE
#
#	THIS SELECTION CORRECTS THE PRESENT IMU ORIENTATION.  THE PRESENT ORIENTATION DIFFERS FROM THAT TO WHICH IT
#	WAS LAST ALIGNED ONLY DUE TO GYRO DRIFT (I.E., NEITHER GIMBAL LOCK NOR IMU POWER INTERRUPTION HAS OCCURRED
#	SINCE THE LAST ALIGNMENT).
#
#	AFTER A IMU ORIENTATION HAS BEEN SELECTED ROUTINE S52.2 IS OPERATED TO COMPUTE THE GIMBAL ANGLES USING THE
#	NEW ORIENTATION AND THE PRESENT VEHICLE ATTITUDE.  CAL52A THEN USES THESE ANGLES, STORED IN THETAD,+1,+2, TO
#	COARSE ALIGN THE IMU.  THE STARS SELECTION ROUTINE, R56, IS THEN OPERATED.  IF 2 STARS ARE NOT AVAILABLE AN ALARM
#	IS FLASHED TO NOTIFY THE ASTRONAUT.  AT THIS POINT THE ASTRONAUT WILL MANEUVER THE VEHICLE AND SELECT 2 STARS
# 	EITHER MANUALLY OR AUTOMATICALLY.  AFTER 2 STARS HAVE BEEN SELECTED THE IMU IS FINE ALIGNED USING ROUTINE R51.  IF
# 	THE RENDEZVOUS NAVIGATION PROCESS IS OPERATING (INDICATED BY RNDVZFLG) P20 IS DISPLAYED.  OTHERWISE P00 IS
#	REQUESTED.
#
# CALLING SEQUENCE --
#
# Page 927
#	THE PROGRAM IS CALLED BY THE ASTRONAUT BY DSKY ENTRY.
#
# SUBROUTINES CALLED --
#
#	1. FLAGDOWN		 7. S52.2		13. NEWMODEX
#	2. R02BOTH		 8. CAL53A		14. PRIOLARM
#	3. GOPERF4		 9. FLAGUP
#	4. MATMOVE		10. R56
#	5. GOFLASH		11. R51
#	6. S52.3		12. GOPERF3
#
# NORMAL EXIT MODES --
#
#	EXITS TO ENDOFJOB
#
# ALARM OR ABORT EXIT MODES --
#
#	NONE
#
# OUTPUT --
#
#	THE FOLLOWING MAY BE FLASHED ON THE DSKY
#		1. IMU ORIENTATION CODE
#		2. ALARM CODE 215 -- PREFERRED IMU ORIENTATION NOT SPECIFIED
#		3. TIME OF NEXT IGNITION
#		4. GIMBAL ANGLES
#		5. ALARM CODE 405 -- TWO STARS NOT AVAILABLE
#		6. PLEASE PERFORM P00
#	THE MODE DISPLAY MAY BE CHANGED TO 20
#
# ERASABLE INITIALIZATION REQUIRED --
#
#	PFRATFLG SHOULD BE SET IF A PREFERRED ORIENTATION HAS BEEN COMPUTED.  IF IT HAS BEEN COMPUTED IT IS STORED IN
#	XSMD, YSMD, ZSMD.
#
#	RNDVZFLG INDICATES WHETHER THE RENDEZVOUS NAVIGATION PROCESS IS OPERATING.
#
# DEBRIS --
#
#	WORK AREA

		BANK	33
		SETLOC	P50S
		BANK

		EBANK=	BESTI
		COUNT*	$$/P52
;
; ============================================================================
; TRANSITION: IMU Alignment Program Entry
;
; The Inertial Measurement Unit (IMU) is the spacecraft's primary navigation
; sensor, containing gyroscopes and accelerometers that track attitude and
; velocity. Over time, gyroscope drift causes the IMU platform orientation
; to become misaligned from the true inertial reference frame. PROG52
; allows the crew to realign the IMU using star sightings through the
; Alignment Optical Telescope (AOT). During Apollo 11's mission, Armstrong
; and Aldrin performed regular IMU alignments in lunar orbit to maintain
; the navigation accuracy needed for the precise lunar landing.
; ============================================================================
;
; PROG52: Main IMU Alignment Program
; This program guides the crew through selecting an IMU orientation,
; computing gimbal angles, coarse aligning the platform, selecting stars,
; and performing fine alignment through optical star sightings.
;
PROG52		TC	BANKCALL
		CADR	R02BOTH		# IMU STATUS CHECK
;
; The crew begins by checking if a preferred IMU orientation has been
; pre-computed by a previous program (such as a maneuver program that
; calculated an optimal platform orientation for the planned burn).
;
		CAF	PFRATBIT
		MASK	FLAGWRD2	# IS PFRATFLG SET?
		CCS	A
# Page 928
		TC	P52A		# YES
		CAF	BIT2		# NO
		TC	P52A +1
;
; Display V04N06 to crew: Flash option code and orientation code.
; The crew selects from multiple alignment options:
;   Option 1: Preferred orientation (pre-computed by another program)
;   Option 2: Nominal orientation (based on current position and velocity)
;   Option 3: REFSMMAT orientation (retain current reference matrix)
;   Option 4: Landing site orientation (for lunar surface operations)
;
P52A		CAF	BIT1
		TS	OPTION2
P52B		CAF	BIT1
		TC	BANKCALL	# FLASH OPTION CODE AND ORIENTATION CODE
		CADR	GOPERF4R	# FLASH V04N06
		TC	GOTOPOOH
		TCF	+5		# V33 -- PROCEED
		TC	P52B		# NEW CODE -- NEW ORIENTATION CODE INPUT
		TC	PHASCHNG	# DISPLAY RETURN
		OCT	00014
		TC	ENDOFJOB

;
; Branch to appropriate alignment routine based on crew's option selection.
; Each option establishes a different reference orientation for the IMU.
;
		CA	OPTION2
		MASK	THREE
		INDEX	A
		TC	+1
		TCF	OPT4		# OPTION 4 LANDING SITE
		TCF	P52H		# OPTION 1 PREFERRED
		TCF	P52T		# OPTION 2 NOMINAL
;
; Option 3: Use existing REFSMMAT (REFerence Stable Member MATrix).
; This option retains the current IMU orientation but performs a fine
; alignment to correct for accumulated gyro drift.
;
P52E		TC	INTPRET		# OPTION 3 REFSMMAT
		GOTO
			P52F		# GO DO R51

OPT4		EXTEND
		DCA	TLAND		# IF OPTION 4 DISPLAY TLAND
		TCF	P52T +2

P52T		EXTEND
		DCA	NEG0
		DXCH	DSPTEM1
		CAF	V06N34*
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	+2
		TC	-5
		DXCH	DSPTEM1
		EXTEND
		BZMF	+2		# IF TIME ZERO OR NEG USE TIME2
		TCF	+3
		EXTEND
		DCA	TIME2
		DXCH	TALIGN
P52V		CA	OPTION2
		MASK	BIT2
		CCS	A
		TC	P52W
# Page 929
		TC	INTPRET		# OPTION 4 -- GET LS ORIENTATION
		GOTO
			P52LS

# Page 930
; ============================================================================
; P52W - Compute Nominal IMU Orientation
;
; Computes nominal (LVLH-type) IMU orientation based on state vector at
; alignment time TALIGN. Nominal orientation aligns IMU axes with the
; local vertical/local horizontal reference frame, where X-axis points
; radially outward (local vertical), Y-axis points in orbit velocity
; direction (forward), and Z-axis completes right-hand system (cross-track).
; This provides intuitive orientation for orbital operations.
;
; INPUTS:
;   TALIGN = Time for IMU alignment (GET, ground elapsed time)
;   RN = Position state vector at TALIGN
;   VN = Velocity state vector at TALIGN
;
; OUTPUTS:
;   XSMD, YSMD, ZSMD = Desired stable member axes (nominal orientation)
;
; CALLED BY: P52A when crew selects Option 2 (Nominal orientation)
;
; HISTORICAL NOTE: During Apollo 11 lunar orbit operations, nominal
; orientation was frequently used for routine IMU realignments, providing
; orientation natural for orbital maneuvering and rendezvous operations.
; ============================================================================

P52W		TC	INTPRET		; Enter interpretive mode
		DLOAD	CALL		; PICK UP ALIGN TIME
			TALIGN		; Load alignment time from crew entry
			S52.3		; COMPUTE NOMINAL IMU ORIENTATION
					; S52.3 computes XSMD = UNIT(R)
					; YSMD = UNIT(V x R), ZSMD = UNIT(X x Y)
					; Classic LVLH frame for orbital ops

; ============================================================================
; P52D - Compute Desired Gimbal Angles from Orientation
;
; Converts desired stable member orientation (XSMD, YSMD, ZSMD) into gimbal
; angles (THETAD) that will physically position the IMU platform to achieve
; that orientation. Reads current vehicle attitude from IMU CDUs and computes
; gimbal commands that account for both desired platform orientation and
; current spacecraft attitude. These angles are displayed to crew for
; verification before commanding physical platform motion.
;
; FUNCTION SEQUENCE:
; 1. Compute desired IMU orientation (XSMD, YSMD, ZSMD) relative to inertial
;    reference frame
; 2. Read current vehicle attitude (gimbal angles from CDUs)
; 3. Compute gimbal angle commands that achieve desired orientation
; 4. Display angles to crew for verification
; 5. Await crew decision: Proceed/Recycle/Terminate
;
; OUTPUTS:
;   THETAD, THETAD+1, THETAD+2 = Desired gimbal angles (CDU angle units)
;       Inner gimbal (roll), Middle gimbal (pitch), Outer gimbal (yaw)
;
; DISPLAY: V06N22 shows three gimbal angles in degrees
;   DSPTEM1 = Outer gimbal angle (OGA)
;   DSPTEM1+1 = Middle gimbal angle (MGA)  
;   DSPTEM1+2 = Inner gimbal angle (IGA)
;
; CREW RESPONSES:
;   V33 (Proceed) = Accept gimbal angles, continue to coarse alignment
;   V32 (Resequence) = Recycle alignment, recompute angles
;   V34 (Terminate) = Abort alignment program
;
; HISTORICAL NOTE: During Apollo 11, crew monitored these angles to ensure
; coarse alignment would not cause gimbal lock (MGA near ±90 degrees).
; ============================================================================

P52D		CALL			; READ VEHICLE ATTITUDE AND
			S52.2		; COMPUTE GIMBAL ANGLES
					; S52.2 combines desired REFSMMAT with
					; current vehicle CDU angles to compute
					; gimbal commands (THETAD,+1,+2)
		EXIT			; Exit interpretive mode for DSKY display
		CAF	V06N22		; Load V06N22 (display gimbal angles)
		TC	BANKCALL	; DISPLAY GIMBAL ANGLES
		CADR	GOFLASH		; Flash display, await crew response
					; Crew verifies angles acceptable
					; (especially checks for gimbal lock risk)
		TC	GOTOPOOH	; V34 -- TERMINATE, abort alignment
		TCF	COARSTYP	; V33 -- PROCEED, continue to coarse align
					; (offers choice of normal vs gyro torque)

; ============================================================================
; P52H - Recompute Gimbal Angles
;
; Re-entry point for alignment cycle after crew requests angle recomputation
; (V32 resequence). Returns to P52D to recalculate gimbal angles, typically
; used when vehicle attitude has changed or crew wants to verify updated
; angle computation. Common during orbital maneuvering when attitude drifts
; during alignment setup phase.
;
; HISTORICAL NOTE: During Apollo 11, this resequence option allowed crew
; to update gimbal angle calculations if Columbia or Eagle's attitude
; drifted between initial calculation and crew readiness to proceed with
; physical platform alignment.
; ============================================================================

P52H		TC	INTPRET		; Enter interpretive mode
		GOTO			; Branch to angle computation
			P52D		; Recalculate gimbal angles with
					; current vehicle attitude

; ============================================================================
; REGCOARS - Perform Regular Coarse and Fine Alignment
;
; Executes standard mechanical coarse alignment sequence (normal coarse).
; This is the primary alignment method, using IMU gimbal motors to physically
; rotate the stable member platform to desired orientation. After coarse
; alignment completes, program continues to star sighting and fine alignment
; phases. Contrasts with gyro torque coarse which precesses gyros without
; mechanical gimbal motion.
;
; SEQUENCE:
; 1. Call CAL53A to drive gimbals to THETAD angles (coarse + fine)
; 2. Set REFSMFLG indicating valid REFSMMAT established
; 3. Clear PFRATFLG (preparation for attitude reference)
; 4. Call R51 for star sighting and fine alignment computation
;
; INPUTS:
;   THETAD, THETAD+1, THETAD+2 = Desired gimbal angles from S52.2
;
; CALLED BY: COARSTYP after crew selects V33 for normal coarse alignment
;
; HISTORICAL NOTE: During Apollo 11 lunar orbit, this routine executed
; multiple times as IMU required periodic realignment to compensate for
; accumulated gyro drift. The mechanical platform motion was visible to
; crew as gimbals repositioned (audible click/whir of gimbal motors).
; ============================================================================

REGCOARS	TC	INTPRET		; Enter interpretive mode
		CALL			; DO COARSE ALIGN
			CAL53A		; Perform coarse + fine mechanical alignment
					; CAL53A drives gimbals to THETAD position
					; using both coarse (fast) and fine (precise)
					; gimbal drive sequences

; ============================================================================
; COARSRET - Coarse Alignment Completion and Flag Management
;
; Common return point after coarse alignment (both mechanical and gyro torque
; methods). Sets REFSMFLG to indicate valid REFSMMAT now established in the
; AGC, and clears PFRATFLG to prepare for attitude reference mode. These
; flags control navigation state validity and coordinate frame usage throughout
; the flight software.
;
; FLAGS MANAGED:
;   REFSMFLG (SET) = REFSMMAT is now valid and IMU is aligned to it
;       All navigation computations can now reference this stable member
;       orientation matrix. Flight programs check this flag before using
;       IMU data for guidance and control.
;
;   PFRATFLG (CLEAR) = Preferred attitude reference flag cleared
;       Resets attitude reference mode to default. During P52 alignment,
;       this flag ensures attitude references use newly established
;       REFSMMAT orientation.
;
; After flag management, continues to P52F for star sighting and fine
; alignment refinement phase.
; ============================================================================

COARSRET	SET	CLEAR		; Set REFSMFLG, clear PFRATFLG
			REFSMFLG	; REFSMMAT now valid for navigation
			PFRATFLG	; Clear preferred attitude mode

; ============================================================================
; P52F - Initiate Star Sighting and Fine Alignment
;
; Calls R51 fine alignment routine which guides crew through star sighting
; process. R51 displays selected star identification (from R56 star selection),
; manages optical telescope marks, computes platform misalignment from star
; observations, and computes gyro torque commands to null out alignment errors.
; This fine alignment phase brings IMU orientation from coarse alignment
; accuracy (~degrees) to navigation-quality accuracy (~arc-seconds).
;
; SEQUENCE IN R51:
; 1. Display first star ID for crew to sight in telescope
; 2. Accept crew mark (V25 when star centered in reticle)
; 3. Display second star ID
; 4. Accept second crew mark
; 5. Compute platform tilt errors from star measurements
; 6. Compute gyro torque angles to null errors
; 7. Apply torque pulses to gyros
; 8. Update REFSMMAT to reflect corrected orientation
;
; HISTORICAL NOTE: During Apollo 11, Buzz Aldrin performed star sightings
; through the Alignment Optical Telescope (AOT) while Eagle was in lunar
; orbit. The two-star sightings typically achieved alignment accuracy of
; better than 1 arc-minute, essential for precise lunar landing navigation.
; ============================================================================

P52F		CALL			; Call fine alignment routine
			R51		; R51 manages star sighting process
					; and computes alignment corrections

; ============================================================================
; P52OUT - P52 Alignment Program Exit Point
;
; Normal completion point for P52 alignment program. Returns control to
; P00 (crew idle program) or P20 (rendezvous navigation) depending on
; RNDVZFLG state. Exits interpretive mode and transfers to GOTOPOOH
; which handles program termination and display of appropriate next
; program number.
;
; EXIT PATHS:
;   If RNDVZFLG clear: Return to P00 (crew awaits next program selection)
;   If RNDVZFLG set: Return to P20 (rendezvous navigation continues)
;
; HISTORICAL NOTE: After successful IMU alignment in lunar orbit, Eagle
; typically returned to P00 awaiting crew selection of next mission program
; (P63 for landing, P12 for ascent, etc.). The newly aligned IMU provided
; accurate inertial reference for all subsequent navigation and guidance.
; ============================================================================

P52OUT		EXIT			; Exit interpretive mode
		TC	GOTOPOOH	; Terminate P52, return to appropriate
					; program (P00 or P20)
VB05N09		=	V05N09
V06N34*		VN	634

# Page 931
# CHECK FOR GYRO TORQUE COARSE ALIGNMENT
;
; ============================================================================
; TRANSITION: Coarse Alignment Method Selection
;
; The IMU alignment process has two phases: coarse and fine. Coarse alignment
; drives the IMU gimbals to approximately the desired orientation (within a
; few degrees). Fine alignment uses star sightings to refine the orientation
; to within a few arc-seconds. The crew can choose between normal coarse
; alignment (mechanical gimbal rotation) and gyro torque coarse alignment
; (torquing the gyros to precess the platform). During Apollo 11, normal
; coarse alignment was typically used for routine realignments in lunar orbit.
; ============================================================================
;
; Display V50N25 to crew: Flash coarse alignment option.
; V33 (Proceed) = Normal mechanical coarse alignment
; V32 (Resequence) = Gyro torque coarse alignment (for small corrections)
; V34 (Terminate) = Abort alignment program
;
COARSTYP	CAF	OCT13
		TC	BANKCALL	# DISPLAY V 50N25 WITH COARSE ALIGN OPTION
		CADR	GOPERF1
		TCF	GOTOPOOH	# V34 -- TERMIN&OE
		TCF	REGCOARS	# V33 -- NORMAL COARSE
;
; Gyro torque coarse: Transform desired stable member orientation (XSMD,
; YSMD, ZSMD) from inertial frame to current stable member frame using
; REFSMMAT. Store result in XDC, YDC, ZDC for gyro torquing routine.
;
		TC	INTPRET		# V32 -- GYRO TORQUE COARSE
		VLOAD	MXV
			XSMD		# GET SM(DESIRED) WRT SM(PRESENT)
			REFSMMAT
		UNIT
		STOVL	XDC
			YSMD
		MXV	UNIT
			REFSMMAT
		STOVL	YDC
			ZSMD
		MXV	UNIT
			REFSMMAT
		STCALL	ZDC
			GYCOARS
		GOTO
			P52OUT
OCT13		OCT	13

# Page 932
# COMPUTE LANDING ORIENTATION FOR OPTION 4
;
; ============================================================================
; TRANSITION: Landing Site Alignment (Option 4)
;
; For lunar surface operations, the IMU needs to be aligned with the local
; landing site coordinate frame rather than an inertial reference. This
; option computes an IMU orientation where the X-axis points radially up
; from the lunar surface, useful for measuring gravity and controlling
; attitude during surface activities. During Apollo 11, this alignment
; would be used after Eagle landed on the Sea of Tranquility to support
; surface experiments and prepare for ascent.
; ============================================================================
;
; Compute landing site orientation based on landing site position vector
; (RLS) stored in Moon-Fixed (MF) reference frame. Transform to inertial
; reference frame and compute latitude, longitude, and altitude.
;
P52LS		SET	CLEAR		# GET LANDING SITE ORIENTATION
			LUNAFLAG
			ERADFLAG	# TO PICK UP RLS
		SETPD	VLOAD
			0
			RLS		# PICK UP LANDING SITE VEC IN MF
		PDDL	PUSH		# RLS PD 0-5
			TALIGN
		STCALL	TLAND		# JAM ALIGN TIME IN TLAND FOR OPTION 4
			RP-TO-R		# TRANS RLS TO REF
		VSR2
		STODL	ALPHAV		# INPUT TO LAT-LONG
			TALIGN
		CALL
			N89DISP
;
; Compute landing site orientation: XSMD points radially outward from
; lunar center through landing site (local vertical). YSMD and ZSMD
; define horizontal plane perpendicular to local gravity vector.
;
		VLOAD	UNIT		# COMPUTE LANDING SITE ORIENT (XSMD)
			ALPHAV
		STCALL	XSMD
			LSORIENT
		GOTO
			P52D		# NOW GO COMPUTE GIMBAL ANGLES.

# Page 933
# SUBROUTINE TO CALCULATE AND DISPLAY THE LUNAR LANDING SITE
;
; This routine converts the landing site position from reference coordinates
; to latitude, longitude, and altitude, then displays these geodetic
; coordinates to the crew on the DSKY for verification.
;
		SETLOC	P50S1
		BANK
		EBANK=	XSM
;
; N89DISP: Compute and store landing site latitude, longitude, and altitude.
; Calls LAT-LONG subroutine to convert Cartesian position to geodetic coords.
; Longitude is scaled by 2 (LONG/2) for DSKY display format.
;
N89DISP		STQ
			QMAJ
		STCALL	GDT/2 +4	# TEMP STORE TIME
			LAT-LONG
		DLOAD	SR1
			LONG
		STODL	LANDLONG
			ALT
		STODL	LANDALT
			LAT
		STODL	LANDLAT
		EXIT
;
; Display V06N89 to crew: Flash latitude, longitude/2, and altitude of
; landing site. Crew can verify coordinates match expected landing zone
; (for Apollo 11, Sea of Tranquility at approximately 0.67°N, 23.5°E).
; V33 (Proceed) = Accept displayed coordinates
; V32 (Resequence) or ENTR = Display again and/or load new coordinates
; V34 (Terminate) = Abort program
;
LSDISP		CAF	V06N89*		# DISPLAY LAT,LONG/2,ALT
		TC	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH	# V34 -- TERMINATE -- EXIT P57
		TCF	+2		# V33 -- PROCEED -- ACCEPT LS DATA
		TCF	LSDISP		# V32 OR E -- LOOK AGAIN AND/OR LOAD NEW LS

		TC	INTPRET
		DLOAD	SL1
			LANDLONG
		STODL	LONG
			LANDALT
		STODL	ALT
			LANDLAT
		STODL	LAT
			GDT/2 +4	# PICK UP TIME
		CALL			# GET RLS BACK FROM LAT,LONG,ALT
			LALOTORV	# RLS B-29 IN MPAC AND ALPHAV
		GOTO
			QMAJ
V06N89*		VN	689

# Page 934
;
; ============================================================================
; SUBROUTINE: S50 (LOCSAM) - Celestial Body Position Computations
;
; This subroutine computes the positions and angular sizes of celestial
; bodies (Sun, Earth, Moon) as seen from the Lunar Module. These calculations
; are used by the star selection routine (R56/PICAPAR) to determine which
; stars are visible and not blocked by bright bodies during IMU alignment.
;
; During Apollo 11's lunar orbit operations, accurate star visibility
; predictions were essential for planning IMU alignments. The crew needed
; to know which stars would be visible through the Alignment Optical
; Telescope (AOT) without interference from the Sun, Earth, or Moon's
; bright light washing out the star field.
;
; COMMENT-ONLY READERS: This routine calculates where the Sun, Earth, and
; Moon are located relative to the spacecraft, so the computer can predict
; which stars will be visible for alignment sightings.
;
; CODE-ALONG READERS: Study the vector computations for unit vectors
; toward each celestial body and the angular diameter calculations using
; ARCSIN. Note the use of mean Earth-Moon distance (384,402 km) and
; Earth equatorial radius (6,378.166 km) constants.
; ============================================================================
;
# NAME -- S50 ALIAS LOCSAM
# BY
# VINCENT
#
# FUNCTION -- COMPUTE INPUTS FOR PICAPAR AND PLANET
#
#	DEFINE
#
#	U   = UNIT( SUN WRT EARTH )
#	 ES
#
#	U   = UNIT( MOON WRT EARTH )
#	 EM
#
#	R   = POSITION VECTOR OF LEM
#	 L
#
#	R   = MEAN DISTANCE (384402KM) BETWEEN EARTH AND MOON
#	 EM
#
#	P   = RATIO R  /(DISTANCE SUN TO EARTH) > .00257125
#	             EM
#
#	R   = EQUATORIAL RADIUS (6378.166KM) OF EARTH
#	 E
#
#	LOCSAM COMPUTES IN EARTH INFLUENCE
#
#	VSUN = U
#	        ES
#
#	VEARTH = -UNIT( R  )
#	                 L
#
#	VMOON = UNIT(R  .U   - R  )
#	              EM  EM    L
#
#	CSUN = COS 90
#
#	CEARTH = COS(5 + ARCSIN(R /MAG(R )))
#	                         E      L
#
#	CMOON	= COS 5
#
# INPUT -- TIME IN MPAC
#
# OUTPUT -- LISTED ABOVE
#
# SUBROUTINES -- LSPOS, LEMPREC
#
# DEBRIS -- VAC AREA, TSIGHT

# Page 935
		COUNT*	$$/LOSAM

S50		= 	LOCSAM

; LOCSAM computes unit vectors toward celestial bodies (Sun, Earth, Moon)
; and their angular half-cone sizes as seen from spacecraft position.
; Branches to EARTCNTR for Earth influence sphere operations or MOONCNTR
; for Moon influence sphere operations.
;
LOCSAM		STQ			; Save return address
			QMIN
		STCALL	TSIGHT		; Store time for ephemeris computation
			LSPOS		; Compute Sun/Moon positions (VSUN, VMOON)
		DLOAD			; Load time from TSIGHT
			TSIGHT
		STCALL	TDEC1		; Store as TDEC1 for precision calculations
			LEMPREC		; Compute LEM position and velocity vectors
		SSP	TIX,2		; Set S2=0, test X2 counter
			S2		; S2 used as branch selector
			0
			MOONCNTR	; If X2=0 (Moon sphere), branch to MOONCNTR

; ============================================================================
; EARTCNTR - Earth Influence Sphere Celestial Body Calculations
;
; When spacecraft is in Earth's gravitational influence sphere (cislunar
; space closer to Earth than to Moon), compute unit vectors toward Sun,
; Earth, and Moon from spacecraft perspective. Also compute angular sizes
; (cosines of half-cone angles) for each body to support star occulting
; calculations.
;
; OUTPUTS:
;   VMOON = Unit vector toward Moon
;   VEARTH = Unit vector toward Earth (negative of position vector)
;   VSUN = Unit vector toward Sun (from LSPOS)
;   CMOON = Cosine of Moon half-cone angle (5 degrees baseline)
;   CEARTH = Cosine of Earth half-cone angle (5 deg + arcsin(R_E/distance))
;   CSUN = Cosine of Sun half-cone angle (60 degrees = 0.5 radians)
; ============================================================================
;
EARTCNTR	VLOAD	VXSC		; Compute Moon position in reference frame
			VMOON		; Unit vector toward Moon from Earth
			RSUBEM		; Mean Earth-Moon distance (384,402 km)
		VSL1	VSU		; Scale left 1 bit, subtract spacecraft pos
			RATT		; Spacecraft position vector from Earth
		UNIT			; Normalize to unit vector
		STOVL	VMOON		; Store unit vector toward Moon
			RATT		; Load spacecraft position
		UNIT	VCOMP		; Normalize and complement (reverse direction)
		STODL	VEARTH		; Store unit vector toward Earth
			RSUBE		; Earth equatorial radius (6378.166 km)
		CALL			; Compute angular size of Earth
			OCCOS		; OCCOS: computes cos(5° + arcsin(R/distance))
		STODL	CEARTH		; Store Earth half-cone cosine
			CSS5		; Constant cos(5°)/4
		STCALL	CMOON		; Store Moon half-cone cosine (fixed 5°)
			ENDSAM		; Continue to common exit point

; ============================================================================
; MOONCNTR - Moon Influence Sphere Celestial Body Calculations
;
; When spacecraft is in Moon's gravitational influence sphere (near-lunar
; space closer to Moon than to Earth), compute unit vectors toward Sun,
; Earth, and Moon from spacecraft perspective. Coordinate transformations
; differ from EARTCNTR because reference frame origin is effectively at Moon
; rather than Earth.
;
; During Apollo 11 lunar orbit operations, Eagle was in Moon influence sphere.
; Star sightings for P52 IMU alignment required accurate knowledge of where
; Sun, Earth, and Moon appeared to avoid selecting stars close to these bright
; bodies which would degrade optical measurements.
;
; COMPUTATION SEQUENCE:
; 1. Transform Sun position from Earth-centered to Moon-centered coordinates
; 2. Compute unit vectors toward all three bodies from spacecraft
; 3. Compute angular half-cone sizes accounting for distance variations
;
; OUTPUTS:
;   VSUN = Unit vector toward Sun (Moon-centered coordinates)
;   VEARTH = Unit vector toward Earth from spacecraft
;   VMOON = Unit vector toward Moon from spacecraft
;   CMOON = Cosine of Moon half-cone angle (arcsin(R_moon/distance))
;   CEARTH = Cosine of Earth half-cone angle (fixed 5 degrees)
;   CSUN = Cosine of Sun half-cone angle (60 degrees = 0.5 radians)
; ============================================================================
;
MOONCNTR	VLOAD	VXSC		; Compute Sun position in Moon frame
			VMOON		; Unit vector Moon-to-Earth
			ROE		; Earth-orbit radius around Sun
		BVSU	UNIT		; Vector subtract from VSUN, normalize
			VSUN		; Sun position (Earth-centered)
		STOVL	VSUN		; Store unit Sun vector (Moon-centered)
			VMOON		; Load Moon unit vector
		VXSC	VAD		; Scale by Earth-Moon distance
			RSUBEM		; Mean Earth-Moon distance
			RATT		; Add spacecraft position from Moon
		UNIT	VCOMP		; Normalize, complement for Earth direction
		STOVL	VEARTH		; Store unit vector toward Earth
			RATT		; Load spacecraft position from Moon
		UNIT	VCOMP		; Normalize, complement for Moon direction
		STODL	VMOON		; Store unit vector toward Moon
			RSUBM		; Moon equatorial radius (1738.2 km)
		CALL			; Compute angular size of Moon
			OCCOS		; OCCOS: cos(5° + arcsin(R/distance))
# Page 936
		STODL	CMOON		; Store Moon half-cone cosine
			CSS5		; Constant cos(5°)/4
		STORE	CEARTH		; Store Earth half-cone (fixed 5°)

; ============================================================================
; ENDSAM - Common Exit Point for LOCSAM Routine
;
; Final step for both EARTCNTR and MOONCNTR paths: store Sun angular size
; and return to caller. Sun half-cone angle is constant (approx 60 degrees
; = 0.5 radians = 30' arc) since Sun distance varies negligibly during
; cislunar or lunar operations compared to Earth-Moon distances.
;
; All three unit vectors (VSUN, VEARTH, VMOON) and three angular sizes
; (CSUN, CEARTH, CMOON) are now available for star selection routine R56
; which must avoid selecting stars within these exclusion cones.
; ============================================================================
;
ENDSAM		DLOAD			; Load Sun half-cone cosine
			CSSUN		; Constant cos(60°) ≈ 0.5
		STORE	CSUN		; Store for star occulting checks
		GOTO			; Return to caller
			QMIN		; Saved return address

; ============================================================================
; OCCOS - Occultation Cosine Calculator
;
; Computes cosine of angular half-cone for celestial body occulting
; calculations. Given body radius and spacecraft distance, calculates
; the half-angle of the circular cone within which stars would be blocked
; or optically contaminated by body brightness.
;
; ALGORITHM:
;   half_cone_angle = 5° + arcsin(radius / distance)
;
; The 5° buffer zone accounts for:
; - Atmospheric refraction (Earth)
; - Albedo light scattering from body surface
; - Optical telescope field-of-view limitations
; - Crew visual acquisition margin
;
; INPUT:
;   MPAC = Body radius (meters, scaled appropriately)
;   36D = Distance to body (from spacecraft position magnitude)
;
; OUTPUT:
;   MPAC = cos(half_cone_angle) scaled and right-shifted
;
; USAGE:
;   Called by EARTCNTR and MOONCNTR to compute exclusion zones for Earth
;   and Moon. Star selection routine R56 uses these cosines to reject
;   candidate stars too close to bright bodies.
;
; HISTORICAL NOTE:
;   During Apollo 11 lunar orbit, Moon's angular radius was approximately
;   15 degrees as seen from 60 nautical mile orbit altitude. The 5-degree
;   buffer gave 20-degree total exclusion cone, ensuring star sightings
;   through AOT were not contaminated by lunar surface brightness or
;   earthshine reflection.
; ============================================================================
;
OCCOS		DDV	SR1		; Divide radius by distance, shift right 1
			36D		; Distance to body (from RATT magnitude)
		ASIN	DAD		; Arcsine of ratio, add buffer margin
			5DEGREES	; 5-degree buffer zone constant
		COS	SR1		; Cosine of total angle, shift right 1
		RVQ			; Return to caller
CEARTH		=	14D
CSUN		=	16D
CMOON		=	18D
CSS5		2DEC	.2490475	# (COS 5)/4
CSSUN		2DEC	.125		# (COS 60)/4
5DEGREES	2DEC	.013888889	# SCALED IN REVS

# Page 937
;
; ============================================================================
; SUBROUTINE: R56 (PICAPAR) - Automatic Star Pair Selection
;
; This routine automatically selects the optimal pair of stars for IMU
; alignment by testing all stars in the onboard star catalog for visibility
; and geometric suitability. The selection process ensures the chosen stars
; are not occulted (blocked) by the Sun, Earth, or Moon, have good angular
; separation (50-100 degrees), and lie within the current field of view of
; the Alignment Optical Telescope (AOT).
;
; During Apollo 11's lunar orbit operations, R56 automated the tedious task
; of manually checking which stars were visible and suitably positioned for
; alignment. The routine would identify the best star pair, allowing the
; crew to proceed directly to sighting rather than spending time searching
; for suitable stars.
;
; Star Selection Criteria:
; 1. Not occulted by Earth, Sun, or Moon (uses angular diameter checks)
; 2. Angular separation between 50° and 100° (optimal for alignment accuracy)
; 3. Both stars within 50° of shaft axis (AOT field of view)
; 4. Maximum separation among qualifying pairs (better alignment geometry)
;
; COMMENT-ONLY READERS: This routine is the "autopilot" for star selection.
; Instead of the crew manually picking stars from a list, the computer
; searches through all 37 catalog stars, checks which ones aren't blocked
; by bright bodies, and picks the best pair for the current spacecraft
; attitude. If no suitable pair is found, it alerts the crew to maneuver
; the spacecraft or manually select stars.
;
; CODE-ALONG READERS: Study the nested loop structure testing star pairs,
; the occultation tests using dot products with celestial body unit vectors,
; the angular separation checks, and the field-of-view tests against SAX.
; Note the use of X1 register for star catalog indexing (37 stars × 6 words
; per star) and the BESTI/BESTJ output containing star numbers.
; ============================================================================
;
# PROGRAM NAME -- R56		DATE: DEC 20 66
# MOD 1				LOG SECTION: P51-P53
#				ASSEMBLY:  SUNDISK REV4D
# BY KEN VINCENT
#
# FUNCTION
#	THIS PROGRAM READS THE IMU-CDUS AND COMPUTES THE VEHICLE ORIENTATION
#	WITH RESPECT TO INERTIAL SPACE.  IT THEN COMPUTES THE SHAFT AXIS (SAX)
#	WITH RESPECT TO REFERENCE INTERTIAL.  EACH STAR IN THE CATALOG IS TESTED
# 	TO DETERMIN IF IT IS OCCULTED BY EITHER EARTH, SUN OR MOON.  IF A
# 	STAR IS NOT OCCULTED THEN IT IS PAIRED WITH ALL STARS OF LOWER INDEX.
# 	THE PAIRED STAR IS TESTED FOR OCCULTATION.  PAIRS OF STARS THAT PASS
#	THE OCCULTATION TESTS ARE TESTED FOR GOOD SEPARATION.  A PAIR OF STARS
#	HAVE GOOD SEPARATION IF THE ANGLE BETWEEN THEM IS LESS THAN 100 DEGREES
#	AND MORE THAN 50 DEGREES.  THOSE PAIRS WITH GOOD SEPARATION
#	ARE THEN TESTED TO SEE IF THEY LIE IN CURRENT FIELD OF VIEW.  (WITHIN
#	50 DEGREES OF SAX).  THE PAIR WITH MAX SEPARATION IS CHOSEN FROM
#	THOSE WITH GOOD SEPARATION, AND IN FIELD OF VIEW.
#
# CALLING SEQUENCE
#	L	TC	BANKCALL
#	L+1	CADR	R56
#	L+2	ERROR RETURN -- NO STARS IN FIELD OF VIEW
#	L+3	NORMAL RETURN
#
# OUTPUT
#	BESTI, BESTJ -- SINGLE PREC, INTEGERS, STAR NUMBERS TIMES 6
#	VFLAG -- FLAG BIT SET IMPLIES NO STARS IN FIELD OF VIEW
#
# INITIALIZATION
#	1)	A CALL TO LOCSAM MUST BE MADE
#
# DEBRIS
#	WORKAREA
#	X,Y,ZNB
#	SINCDU, COSCDU
#	STARAD -- STAR +5

R56		=	PICAPAR
		COUNT*	$$/R56
PICAPAR		TC	MAKECADR
		TS	QMIN
		TC	INTPRET
		CALL
			CDUTRIG
		CALL
			CALCSMSC
		SETPD
			0
		SET	DLOAD		# VFLAG = 1
			VFLAG
# Page 938
			DPZERO
		STOVL	BESTI
			XNB
		VXSC	PDVL
			HALFDP
			ZNB
		AXT,1	VXSC
			228D		# X1 = 37 X 6 + 6
			HALFDP
		VAD
		VXM	UNIT
			REFSMMAT
		STORE	SAX		# SAX = SHAFT AXIS
		SSP	SSP		# S1 = S2 = 6
			S1
			6
			S2
			6
PIC1		TIX,1	GOTO		# MAJOR STAR
			PIC2
			PICEND
PIC2		VLOAD*	DOT
			CATLOG,1
			SAX
		DSU	BMN
			CSS33
			PIC1
		LXA,2
			X1
PIC3		TIX,2	GOTO
			PIC4
			PIC1
PIC4		VLOAD*	DOT
			CATLOG,2
			SAX
		DSU	BMN
			CSS33
			PIC3
		VLOAD*	DOT*
			CATLOG,1
			CATLOG,2
		DSU	BPL
			CSS40
			PIC3
		VLOAD*	CALL
			CATLOG,1
			OCCULT
		BON
			CULTFLAG
			PIC1
# Page 939
		VLOAD*	CALL
			CATLOG,2
			OCCULT
		BON
			CULTFLAG
			PIC3
STRATGY		BONCLR
			VFLAG
			NEWPAR
		XCHX,1	XCHX,2
			BESTI
			BESTJ
STRAT		VLOAD*	DOT*
			CATLOG,1
			CATLOG,2
		PUSH	BOFINV
			VFLAG
			STRAT -3
		DLOAD	DSU
		BPL
			PIC3
NEWPAR		SXA,1	SXA,2
			BESTI
			BESTJ
		GOTO
			PIC3
OCCULT		MXV	BVSU
			CULTRIX
			CSS
		BZE
			CULTED
		BMN	SIGN
			CULTED
			MPAC +3
		BMN	SIGN
			CULTED
			MPAC +5
		BMN	CLRGO
			CULTED
			CULTFLAG
			QPRET
CULTED		SETGO
			CULTFLAG
			QPRET
CSS		= 	CEARTH
CSS40		2DEC	.16070		# COS 50 / 4
CSS33		2DEC	.16070		# COS 50 / 4
PICEND		BOFF	EXIT

# Page 940
			VFLAG
			PICGXT
		TC	PICBXT
PICGXT		LXA,1	LXA,2
			BESTI
			BESTJ
		VLOAD	DOT*
			SAX
			CATLOG,1
		PDVL	DOT*
			SAX
			CATLOG,2
		DSU
		BPL	SXA,1
			PICNSWP
			BESTJ
		SXA,2
			BESTI
PICNSWP		EXIT
		INCR	QMIN
PICBXT		CA	QMIN
		TC	SWCALL
VPD		= 	0D
V0		=	6D
V1		= 	12D
V2		=	18D
V3		=	24D
DP0		=	30D
DP1		=	32D

# Page 941
;
; ============================================================================
; SUBROUTINE: R51 - Fine Alignment of IMU Stable Member
;
; R51 performs the fine alignment of the IMU stable member to the desired
; orientation defined by REFSMMAT (Reference Stable Member Matrix). This
; routine brings the IMU platform to precise alignment using star sightings
; through the Alignment Optical Telescope (AOT), following a coarse alignment.
;
; The fine alignment process involves:
; 1. Selecting optimal star pairs (via LOCSAM and R56/PICAPAR)
; 2. Crew sighting of first star through AOT (via R52 and AOTMARK)
; 3. Recording star position and computing alignment errors
; 4. Crew sighting of second star
; 5. Calculating gyro torquing angles to correct platform orientation
; 6. Issuing fine torque pulses to IMU gyros
;
; During Apollo 11's lunar orbit, R51 was executed multiple times to maintain
; IMU accuracy. Each alignment took approximately 5-10 minutes as the crew
; maneuvered the LM to point the AOT at selected stars, marked each star's
; position, and waited for the AGC to compute and apply corrective torques.
; The precision alignment was essential for accurate landing site targeting
; and ascent trajectory calculations.
;
; COMMENT-ONLY READERS: This is the core alignment routine. After the computer
; selects two suitable stars, it guides the astronauts through sighting each
; star in the telescope. Based on where the stars appear versus where they
; should appear, the computer calculates exactly how much the navigation
; platform has drifted and issues commands to the gyros to correct the drift.
;
; CODE-ALONG READERS: Study the integration of multiple subsystems - LOCSAM
; for celestial body positions, R56 for star selection, R52 for AOT sighting
; management, AOTMARK for mark processing, and R53-R55 for torquing angle
; calculations. Note the phase changes for restart protection and the STARIND
; counter tracking which star is being sighted (0=first, 1=second).
; ============================================================================
;
# NAME -- R51	FINE ALIGN
# FUNCTION -- TO ALIGN THE STABLE MEMBER TO REFSSMAT
# CALLING SEQ -- CALL R51
# INPUT -- REFSMMAT
# OUTPUT -- GYRO TORQUE PULSES
# SUBROUTINES -- LOCSAM, PICAPAR, R52, R53, R54, R55

		COUNT*	$$/R51
; ============================================================================
; R51 - FINE IMU ALIGNMENT USING STAR SIGHTINGS
;
; Primary routine for precise IMU platform alignment using optical star
; sightings through the Alignment Optical Telescope (AOT). Implements
; two-star alignment procedure that determines platform misalignment and
; commands gyro torques to correct orientation. This is the core alignment
; routine called after coarse alignment establishes approximate orientation.
;
; ALIGNMENT PROCEDURE:
; 1. Display time-to-sight for crew preparation (V05N09)
; 2. Crew maneuvers spacecraft to acquire first star in AOT
; 3. Crew marks star position when properly centered (MARK button)
; 4. Process first star sighting data
; 5. Crew maneuvers to acquire second star
; 6. Crew marks second star position
; 7. Compute platform misalignment from two-star geometry
; 8. Issue gyro torque commands to correct misalignment
; 9. Update REFSMMAT to reflect new alignment
;
; Historical Context: During lunar orbit operations, regular IMU alignments
; maintained navigation accuracy. Star sightings provided absolute inertial
; reference, correcting accumulated gyro drift. Two-star method provides
; complete 3-axis orientation determination.
; ============================================================================

R51		STQ			; Store return address
			QMAJ		; QMAJ = calling program return point
R51.1		EXIT			; Exit interpreter mode
		TC	PHASCHNG		; Phase change for restart protection
		OCT	04024			; Phase change code

; ============================================================================
; R51 DISPLAY AND INITIALIZATION
;
; Displays program mode to crew and initializes alignment sequence timing.
; Computes time-to-sight by adding current time to TSIGHT1 offset, then
; calls star selection routine R56 to determine which stars are available
; for crew to sight through AOT given current spacecraft attitude.
; ============================================================================

R51C		CAF	OCT15			; Load verb 05 code for display
						; Announces alignment program active
		TC	BANKCALL		; Bank call to display handler
		CADR	GOPERF1			; GOPERF1 processes program display
		TC	GOTOPOOH		; Crew terminated - exit to P00
		TC	+2		# V33E	; Crew entered V33 (proceed without data)
		TC	R51E		# ENTER	; Crew pressed ENTER - proceed
		
		TC	INTPRET			; Enter interpretive mode for timing calc
		RTB	DAD			; Load time, add offset
			LOADTIME		; RTB LOADTIME gets current mission time
			TSIGHT1			; TSIGHT1 = time offset for first sighting
						; Computes when crew should sight star
		CALL				; Call subroutine
			LOCSAM			; LOCSAM processes sighting time
						; Prepares timing for crew operations
		EXIT				; Return to native mode

		TC	BANKCALL		; Call star selection routine
		CADR	R56			; R56 selects optimal stars for sighting
						; Determines which stars visible in AOT
						; given current spacecraft attitude
		TC	R51I			; Continue to alarm check
; ============================================================================
; R51 ALARM AND TIME DISPLAY
;
; Issues alarm 405 if star selection encountered problems (e.g., no suitable
; stars available for current attitude). Displays time-to-sight (V05N09) to
; inform crew when to begin star sighting sequence. Crew can maneuver
; spacecraft to acquire star or recycle program if timing unsatisfactory.
;
; Alarm 405: Star availability problem - crew may need attitude change
; V05N09: Displays time for star sighting operation
; ============================================================================

R51F		TC	R51E			; Jump to initialization
R51I		TC	ALARM			; Issue program alarm
		OCT	405			; Alarm 405: Star selection issue
						; Indicates star visibility problem
						; Crew may need to adjust attitude
		CAF	VB05N09			; Load verb 05, noun 09 code
						; V05N09 displays time parameter
		TC	BANKCALL		; Bank call to display routine
		CADR	GOFLASH			; GOFLASH displays and waits for response
						; Shows time for crew to sight star
		TC	GOTOPOOH		; Crew terminated - exit to P00
		TC	R51E			; Crew proceeded - continue alignment
		TC	R51C			; Crew recycled - restart from beginning
; ============================================================================
; STAR SIGHTING INITIALIZATION
;
; Initializes star counter (STARIND) to track sighting sequence. STARIND=0
; indicates first star, STARIND=1 indicates second star. Two-star method
; provides complete 3-axis platform orientation determination. Single star
; constrains only 2 axes, leaving rotation about star line-of-sight unknown.
; ============================================================================

R51E		CAF	ZERO			; Clear accumulator
		TS	STARIND			; STARIND = 0 (first star)
						; Tracks which star being sighted
R51.2		TC	INTPRET			; Enter interpretive mode
						; Prepare for alignment computations
R51.3		EXIT				; Return to native mode
						; Star sighting complete
		TC	PHASCHNG		; Phase change for restart protection
		OCT	04024			; Phase code

; ============================================================================
; STAR SIGHTING SEQUENCE
;
; Executes crew star sighting through AOT. R52 prepares for mark acquisition,
; AOTMARK captures crew's mark when star properly centered in AOT reticle.
; Crew presses MARK button when star centered, capturing spacecraft attitude
; and star line-of-sight vector at that instant. OPTSTALL waits for mark.
;
; Historical: Crew had to carefully center star in AOT reticle cross-hairs
; and press MARK at exact centering moment. Accuracy of alignment depended
; on crew's precision in marking star positions.
; ============================================================================

		TC	INTPRET			; Enter interpretive mode
		CALL				; Call sighting preparation
			R52		# AOP WILL MAKE CALLS TO SIGHTING
						; R52 prepares for star mark
		EXIT				; Return to native mode
		TC	BANKCALL		; Call mark acquisition routine
# Page 942
		CADR	AOTMARK			; AOTMARK waits for crew MARK button
						; Captures star sighting data
		TC	BANKCALL		; Call stall routine
		CADR	OPTSTALL		; OPTSTALL waits for mark completion
		TC	CURTAINS		; Error exit if mark failed
; ============================================================================
; STAR SIGHTING COMPLETION CHECK
;
; Checks STARIND to determine if first or second star just sighted. If
; STARIND=0 (first star), proceeds to R51.4 to save data and prepare for
; second star. If STARIND=1 (second star), saves second star vector and
; proceeds to alignment computation using both star measurements.
; ============================================================================

		CCS	STARIND			; Check star index (0 or 1)
		TCF	+2			; STARIND was 1 (second star) - continue
		TC	R51.4			; STARIND was 0 (first star) - save and loop
		
		TC	INTPRET			; Enter interpretive mode
		VLOAD				; Load vector
			STARAD +6		; STARAD+6 = second star unit vector
						; Measured line-of-sight to star
		STORE	STARSAV2		; Save second star measurement
						; STARSAV2 holds star 2 data
		EXIT				; Return to native mode
		TC	PHASCHNG		; Phase change for restart protection
		OCT	04024			; Phase code

; ============================================================================
; TWO-STAR ALIGNMENT COMPUTATION
;
; Computes platform misalignment from two star measurements and commands
; corrective gyro torques. Process:
; 1. PLANET computes true star positions in inertial frame at TSIGHT time
; 2. Transform star vectors to platform frame using current REFSMMAT
; 3. R54 validates star data quality and geometry
; 4. AXISGEN computes platform misalignment from star measurement errors
; 5. R55 issues gyro torque commands to correct misalignment
; 6. Updates REFSMMAT to reflect corrected orientation
;
; Two-star method provides complete 3-axis determination. Star separation
; angle must be adequate (typically >30°) for good geometry. Platform
; misalignment appears as difference between measured and computed star
; line-of-sight vectors.
; ============================================================================

		TC	INTPRET			; Enter interpretive mode
		DLOAD	CALL			; Load time, compute star positions
			TSIGHT			; TSIGHT = time of star sighting
			PLANET			; PLANET computes true star positions
						; Accounts for precession, proper motion
		MXV	UNIT			; Transform to platform frame, normalize
			REFSMMAT		; REFSMMAT = platform orientation matrix
						; Transforms inertial to platform frame
		STOVL	STARAD +6		; Store computed star 2 platform vector
			PLANVEC			; PLANVEC = computed star 1 position
		MXV	UNIT			; Transform star 1 to platform, normalize
			REFSMMAT		; Apply platform transformation
		STOVL	STARAD			; Store computed star 1 platform vector
			STARSAV1		; STARSAV1 = measured star 1 vector
		STOVL	6D			; Store in working memory
			STARSAV2		; STARSAV2 = measured star 2 vector
		STCALL	12D			; Store in working memory, call test
			R54		# STAR DATA TEST
						; R54 validates star measurement quality
						; Checks star separation angle geometry
		BOFF	CALL			; Branch if flag off, then call
			FREEFLAG		; FREEFLAG controls alignment mode
			R51K			; Skip AXISGEN if flag set
			AXISGEN			; AXISGEN computes misalignment angles
						; Determines gyro torque commands needed
		CALL				; Call gyro torquing routine
			R55		# GYRO TORQUE
						; R55 issues gyro torque commands
						; Corrects platform orientation
		CLEAR				; Clear flag
			PFRATFLG		; PFRATFLG = preferred attitude flag
R51K		EXIT				; Return to native mode
; ============================================================================
; R51P63 - ALIGNMENT DURING POWERED DESCENT
;
; Special entry point for P63 (lunar landing) to perform IMU alignment during
; descent phase. Displays program status, allows crew to proceed with alignment
; or return to landing program. If crew chooses alignment, executes R51 then
; returns to major mode caller (typically P63 guidance loop).
;
; Historical: Rarely used during actual descent as crew focused on landing.
; Provided backup capability to realign IMU if drift suspected during descent.
; ============================================================================

R51P63		CAF	OCT14			; Load program identifier
		TC	BANKCALL		; Bank call to display
		CADR	GOPERF1			; Display alignment available
		TC	GOTOPOOH		; Crew terminated - exit to P00
		TC	R51C			; Crew proceeded - execute alignment
		TC	INTPRET			; Enter interpretive mode
		GOTO				; Return to caller
			QMAJ			; QMAJ = major mode return address
; ============================================================================
; FIRST STAR PROCESSING - R51.4
;
; After first star sighted, saves star measurement and computes planet
; (Earth or Moon) position for gimbal angle calculations. Saves first star
; unit vector in STARSAV1, calls PLANET to get celestial body position at
; sighting time, then proceeds to second star acquisition (R51.3).
;
; Two-star method required: single star constrains only 2 IMU axes (rotation
; about star line-of-sight remains free). Second star at different angle
; resolves 3rd axis, completing 3-axis platform alignment determination.
; ============================================================================

R51.4		TC	INTPRET			; Enter interpretive mode
		VLOAD				; Load vector
# Page 943
			STARAD +6		; STARAD+6 = first star unit vector
						; Measured line-of-sight from AOT
		STORE	STARSAV1		; Save first star measurement
						; STARSAV1 preserves star 1 data
		DLOAD	CALL			; Load time, call planet routine
			TSIGHT			; TSIGHT = time of first sighting
			PLANET			; PLANET computes Earth/Moon position
						; Needed for horizon reference
		STORE	PLANVEC			; Store planet position vector
						; PLANVEC used in angle calcs
		SSP				; Set single precision
			STARIND			; STARIND = star index counter
			1			; Set to 1 (first star processed)
		GOTO				; Jump to next phase
			R51.3			; R51.3 continues to second star
TSIGHT1		2DEC	36000		# 6 MIN TO MARKING
						; Time offset = 360 seconds (6 minutes)
						; Separation between star sightings

# Page 944
# GYRO TORQUE COARSE ALGNMENT

; ============================================================================
; GYCOARS - GYRO TORQUE COARSE ALIGNMENT
;
; Performs initial coarse alignment of IMU platform to desired orientation
; using gyro torquing. Computes required gimbal angles to target orientation,
; displays gimbal monitoring to crew, then commands gyros to physically
; reorient platform. Uses large torque pulses to rapidly move platform to
; approximate alignment before fine alignment with star sightings.
;
; PROCESS:
; 1. Compute desired gimbal angles (CALCGTA)
; 2. Display gimbal angles to crew (V16N20 - monitor mode)
; 3. Issue gyro torque pulses to reorient platform (IMUPULSE)
; 4. Wait for gyros to complete movement (IMUSTALL)
; 5. Save desired REFSMMAT and initialize drift compensation
;
; Historical: Coarse alignment typically took 1-2 minutes. Crew monitored
; gimbal angles on DSKY to verify proper platform movement before proceeding
; to fine star alignment.
; ============================================================================

GYCOARS		STQ	CALL			; Store return, call angle calc
			QMAJ			; QMAJ = major mode return address
			CALCGTA			; CALCGTA computes gimbal target angles
						; Determines OGA, IGA, MGA for alignment
		CLEAR	CLEAR			; Clear flags
			DRIFTFLG		; DRIFTFLG = gyro drift flag
			REFSMFLG		; REFSMFLG = reference matrix flag
						; Cleared during coarse align phase
		EXIT				; Return to native mode

		CAF	V16N20		# MONITOR GIMBALS
						; V16 = monitor verb, N20 = gimbal angles
		TC	BANKCALL		; Bank call to display
		CADR	GODSPR			; GODSPR displays angles to crew
						; Crew monitors OGA, IGA, MGA during movement
		CA	R55CDR			; Load R55 address
		TC	BANKCALL		; Bank call to IMU control
		CADR	IMUPULSE		; IMUPULSE issues gyro torque commands
						; Physical gyros reorient platform
		TC	BANKCALL		; Bank call to stall
		CADR	IMUSTALL		; IMUSTALL waits for gyros to complete
						; Blocks until platform movement finished
		TC	CURTAINS		; Error exit if IMU stall fails
						; CURTAINS terminates program on IMU fault
		TC	PHASCHNG		; Phase change for restart protection
		OCT	04024			; Phase change code

		TC	INTPRET			; Enter interpretive mode
		AXC,1	AXC,2			; Set index registers
			XSMD			; XSMD = temporary matrix storage
			REFSMMAT		; REFSMMAT = reference matrix
		CALL			# STORE DESIRED REFSMMAT
			MATMOVE			; MATMOVE copies matrix
						; Saves desired orientation in REFSMMAT
		CLEAR	SET			; Update flags
			PFRATFLG		; PFRATFLG = preferred attitude flag
			REFSMFLG		; REFSMFLG now set (REFSMMAT valid)
		CALL				; Call initialization
			NCOARSE		# SET DRIFT AND INITIALIZE 1/PIPADT
						; NCOARSE initializes gyro drift comp
						; Sets up PIPA integration interval
		GOTO				; Jump to completion
			R51K			; R51K exits alignment sequence
V16N20		VN	1620			; Verb 16 Noun 20 code

# Page 945
# R55	GYRO TORQUE
# FUNCTION -- COMPUTE AND SEND GYRO PULSES
# CALLING SEQ -- CALL R55
# INPUT -- X,Y,ZDC -- REFSMMAT WRT PRESENT STABLE MEMBER
# OUTPUT -- GYRO PULSES
# SUBROUTINES -- CALCGTA, GOFLASH, GODSPR, IMUFINE, IMUPULSE, GOPERF1

; ============================================================================
; R55 GYRO TORQUE ROUTINE
;
; COMMENT-ONLY READERS: After computing the misalignment between the current
; IMU platform orientation and the desired orientation, R55 physically
; torques (rotates) the IMU gyroscopes to correct the alignment error.
; The gyroscopes are precision instruments that must be carefully commanded
; with discrete pulses rather than continuous signals. During Apollo 11's
; mission, these fine alignment corrections maintained navigation accuracy
; within fractions of a degree throughout the journey.
;
; CODE-ALONG READERS: R55 computes gyro torquing angles (OGC, IGC, MGC) via
; CALCGTA subroutine, displays them to crew for verification (V06N93), then
; sends discrete pulses to IMU gyros via IMUPULSE. Each pulse represents a
; precise angular increment. The routine uses IMUSTALL to wait for gyro
; torquing completion before returning. Phase changes (PHASCHNG) protect
; against restart during critical gyro torquing operations.
; ============================================================================

		COUNT*	$$/R55
R55		STQ
			QMIN
		CALL
			CALCGTA
PULSEM		EXIT
R55.1		CAF	V06N93
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	R55.2
		TC	R55RET
R55.2		TC	PHASCHNG
		OCT	00214
		CA	R55CDR
		TC	BANKCALL
		CADR	IMUPULSE
		TC	BANKCALL
		CADR	IMUSTALL
		TC	CURTAINS
		TC	PHASCHNG
		OCT	04024

R55RET		TC	INTPRET
		GOTO
			QMIN
V06N93		VN	0693
R55CDR		ECADR	OGC
R54		=	CHKSDATA

# ROUTINE NAME -- CHKSDATA		DATE -- JAN 9, 1967
# MOD NO -- 0				LOG SECTION -- P51-P53
# MODIFICATION BY -- LONSKE		ASSEMBLY --
#
# FUNCTIONAL DESCRIPTION -- CHECKS THE VALIDITY OF A PAIR OF STAR SIGHTINGS.  WHEN A PAIR OF STAR SIGHTINGS ARE MADE
# BY THE ASTRONAUT THIS ROUTINE OPERATES AND CHECKS THE OBSERVED SIGHTINGS AGAINST STORED STAR VECTORS IN THE
# COMPUTER TO INSURE A PROPER SIGHTING WAS MADE.  THE FOLLOWING COMPUTATIONS ARE PERFORMED --
#	OS1	=	OBSERVED STAR 1 VECTOR
#	OS2	=	OBSERVED STAR 2 VECTOR
#	SS1	=	STORED STAR 1 VECTOR
#	SS2 	=	STORED STAR 2 VECTOR
#	 A1	= 	ARCCOS(OS1 - OS2)
#	 A2	=	ARCCOS(SS1 - SS2)
#	  A 	=	ABS(2(A1 - A2))
# Page 946
# THE ANGULAR DIFFERENCE IS DISPLAYED FOR ASTRONAUT ACCEPTANCE.
#
# EXIT MODE --	1. FREEFLAG SET IMPLIES ASTRONAUT WANTS TO PROCEED
#		2. FREEFLAG RESET IMPLIES ASTRONAUT WANTS TO RECYCLE
#
# OUTPUT --	1. VERB 6,NOUN 3 -- DISPLAYS ANGULAR DIFFERENCE BETWEEN 2 SETS OF STARS.
#		2. STAR VECTORS FROM STAR CATALOG ARE LEFT IN 6D AND 12D.
#
# ERASABLE INITIALIZATION REQUIRED --
#		1. MARK VECTORS ARE STORED IN STARAD AND STARAD +6.
#		2. CATALOG VECTORS ARE STORED IN 6D AND 12D.
#
# DEBRIS --

; ============================================================================
; R54 ALIGN SIGHTING DATA CHECK (CHKSDATA)
;
; COMMENT-ONLY READERS: Before accepting star sightings for IMU alignment,
; the computer validates the astronaut's observations. If the astronaut
; accidentally sighted the wrong star or made a poor mark, the angular
; relationship between the two observed stars would differ from their known
; catalog positions. R54 computes this angular difference and displays it
; to the crew. A large difference (more than a few degrees) indicates a
; sighting error. During Apollo 11, this quality check prevented bad star
; sightings from corrupting the navigation system alignment.
;
; CODE-ALONG READERS: R54 computes angles between observed star pairs
; (STARAD, STARAD+6) and catalog star pairs (6D, 12D) using dot products
; and ACOS. The absolute angular difference (stored in NORMTEM1) is
; displayed via V06N05. FREEFLAG controls two-pass logic: first pass
; computes observed angle, second pass computes catalog angle and difference.
; Astronaut can PROCEED (accept sightings) or RECYCLE (reject and retry).
; ============================================================================

		COUNT*	$$/R54
CHKSDATA	STQ	SET
			QMIN
			FREEFLAG
CHKSAB		AXC,1			# SET X1 TO STORE EPHEMERIS DATA
			STARAD

CHKSB		VLOAD*	DOT*		# CAL. ANGLE THETA
			0,1
			6,1
		SL1	ACOS
		STORE	THETA
		BOFF	INVERT		# BRANCH TO CHKSD IF THIS IS 2ND PASS
			FREEFLAG
			CHKSD
			FREEFLAG	# CLEAR FREEFLAG
		AXC,1	DLOAD		# SET X1 TO MARK ANGLES
			6D
			THETA
		STORE	18D
		GOTO
			CHKSB		# RETURN TO CAL. 2ND ANGLE
CHKSD		DLOAD	DSU
			THETA
			18D
		ABS	RTB		# COMPUTE POS DIFF
			SGNAGREE
		STORE	NORMTEM1
		SET	EXIT
			FREEFLAG
		CAF	VB6N5
		TC	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH
		TC	CHKSDA		# PROCEED
		TC	INTPRET
		CLEAR	GOTO
			FREEFLAG
			QMIN
CHKSDA		TC	INTPRET
# Page 947
		GOTO
			QMIN
VB6N5		VN	605

# NAME -- CAL53A
# FUNCTION -- COMPUTE DESIRED GIMBAL ANGLES AND COARSE ALIGN IF NECESSARY
# CALLING SEQUENCE -- CALL CAL53A
# INPUT -- X,Y,ZSMD, CDUX,Y,Z
#	   DESIRED GIMBAL ANGLES -- THETAD,+1,+2
# OUTPUT -- THE IMU COORDINATES AT STORED IN REFSMMAT
# SUBROUTINES -- S52.2, IMUCOARSE, IMUFINE

; ============================================================================
; CAL53A - IMU COARSE ALIGNMENT ROUTINE
;
; Performs coarse alignment of IMU platform to desired orientation. Computes
; target gimbal angles, compares them with current gimbal positions, and
; determines if coarse gyro torquing is needed or if platform is already
; close enough for fine alignment only.
;
; LOGIC:
; 1. Recompute desired gimbal angles using current state (S52.2)
; 2. Read actual CDU (gimbal) angles from IMU hardware
; 3. Loop through all three gimbals (outer, inner, middle)
; 4. For each gimbal, compute angular difference between target and current
; 5. If ANY gimbal is more than 1 degree off target:
;    - Perform COARSE alignment (large gyro torque pulses)
;    - Initialize drift compensation (NCOARSE)
; 6. If ALL gimbals are within 1 degree:
;    - Skip coarse alignment (FINEONLY)
;    - Proceed directly to star fine alignment
; 7. Store alignment matrix in REFSMMAT
;
; The 1-degree threshold balances alignment time vs accuracy. Larger errors
; require physical platform movement via gyro torquing. Small errors can be
; compensated during fine alignment using star sightings.
;
; Historical: During lunar orbit IMU alignments, platform typically required
; full coarse alignment after long coast phases due to accumulated gyro drift.
; ============================================================================

		COUNT*	$$/R50
CAL53A		CALL			; Recompute gimbal angles
			S52.2		# MAKE ONE FINAL COMP OF GIMBAL ANGLES
					; S52.2 uses current vehicle attitude
					; and desired REFSMMAT to get angles
		RTB	SSP		; Return to basic, set pointer
			RDCDUS		# READ CDUS
					; RDCDUS reads CDU registers
					; Returns OGA, IGA, MGA in degrees
			S1		; S1 = loop counter for gimbals
			1		; Start at gimbal 1
		AXT,1	SETPD		; Initialize index and push-down
			3		; X1 = 3 (gimbal count)
			4		; PD = 4 (stack pointer)

; Loop through three gimbals checking alignment error
CALOOP		DLOAD*	SR1		; Load desired angle, shift right 1
			THETAD +3D,1	; THETAD+3 indexed by X1
					; Desired gimbal angle from computation
		PDDL*	SR1		; Push to stack, load current angle
			4,1		; CDU angle indexed (actual gimbal)
					; Current gimbal position from hardware
		DSU	ABS		; Compute difference, absolute value
					; |desired - actual| = alignment error
		PUSH	DSU		; Push error, subtract threshold
			DEGREE1		; DEGREE1 = 1 degree scaled
					; Check if error > 1 degree
		BMN	DLOAD		; Branch if minus (error < 1 deg)
			CALOOP1		; Within tolerance, check next gimbal
					; If not minus, error > 1 degree
		DSU	BPL		; Subtract 359 degrees, check sign
			DEG359		; DEG359 = 359 degrees scaled
					; Handles wrap-around case
			CALOOP1		; If plus, continue check
					; If error large, need coarse align
		EXIT			; Exit interpretive mode

		TC	PHASCHNG	; Phase change for restart protection
		OCT	04024		; Phase code

		TC	INTPRET		; Re-enter interpretive mode

; Coarse and fine alignment needed (error > 1 degree found)
COARFINE	CALL			; Perform coarse alignment
			COARSE		; COARSE issues large gyro torque pulses
					; Physically moves platform to target
		CALL			; Initialize drift compensation
			NCOARSE		; NCOARSE sets gyro drift parameters
					; Initializes 1/PIPADT for accel comp
		GOTO			; Continue to matrix storage
			FINEONLY	; Jump to common exit path

; Check next gimbal (current gimbal within 1 degree)
CALOOP1		TIX,1			; Test index and decrement
			CALOOP		; Loop back for next gimbal
					; X1--, continue until all 3 checked

; Fine alignment only (all gimbals within 1 degree)
FINEONLY	AXC,1	AXC,2		; Set index registers for matrix move
			XSM		; X1 -> XSM (source matrix)
			REFSMMAT	; X2 -> REFSMMAT (destination)
		CALL			; Store alignment matrix
			MATMOVE		; MATMOVE copies 3x3 matrix
# Page 948
		GOTO			; Return from alignment
			COARSRET	; COARSRET continues alignment sequence

; ============================================================================
; MATMOVE - MATRIX TRANSFER UTILITY
;
; Copies a 3x3 matrix (9 words) from source to destination using indexed
; addressing. Used throughout alignment routines to save/restore REFSMMAT
; and other orientation matrices.
;
; INPUT: X1 = source matrix address, X2 = destination matrix address
; OUTPUT: 9 words transferred (3 vectors of 3 components each)
;
; Matrix stored as three consecutive 3D vectors:
; - Words 0,1,2 = first row (X-axis)
; - Words 6,7,8 (6D offset) = second row (Y-axis)  
; - Words 12,13,14 (12D offset) = third row (Z-axis)
; ============================================================================

MATMOVE		VLOAD*			# TRANSFER MATRIX
			0,1		; Load first vector (row 1) from source
					; Indexed by X1
		STORE	0,2		; Store to destination indexed by X2
		VLOAD*			; Load second vector (row 2)
			6D,1		; 6D offset (double precision triplet)
		STORE	6D,2		; Store second vector
		VLOAD*			; Load third vector (row 3)
			12D,1		; 12D offset (third triplet)
		STORE	12D,2		; Store third vector
		RVQ			; Return via Q register

; Angle constants for alignment threshold checking
DEGREE1		DEC	46		# 1 DEG SCALED CDU/2
					; CDU scale: 360 deg = 2^14 = 16384
					; 1 deg = 16384/360 = 45.51, scaled /2 = 46
DEG359		DEC	16338		# 359 DEG SCALED CDU/2
					; 359 deg for wrap-around check
					; Handles gimbal angle discontinuity at 0/360

; ============================================================================
; RDCDUS - READ CDU ANGLES
;
; Reads current gimbal angles from IMU Coupling Data Units (CDUs). CDUs are
; synchro resolvers that measure actual gimbal shaft positions. Returns
; outer gimbal angle (CDUX), inner gimbal angle (CDUY), and middle gimbal
; angle (CDUZ) in scaled units.
;
; CDU SCALE: 360 degrees = 2^14 counts (16384 counts per revolution)
;            1 degree = 45.51 counts
;            Resolution = 0.022 degrees (1.32 arc-minutes)
;
; INHINT used to prevent interrupt during multi-word read, ensuring all
; three gimbal angles represent same instant in time.
; ============================================================================

RDCDUS		INHINT			# READ CDUS
					; Inhibit interrupts for atomic read
		CA	CDUX		; Read outer gimbal angle from hardware
					; CDUX = outer gimbal CDU register
		INDEX	FIXLOC		; Indexed store using FIXLOC
		TS	1		; Store to location 1 offset by FIXLOC
					; First element of CDU triplet
		CA	CDUY		; Read inner gimbal angle
					; CDUY = inner gimbal CDU register
		INDEX	FIXLOC		; Indexed store
		TS	2		; Store to location 2 offset by FIXLOC
					; Second element of CDU triplet
		CA	CDUZ		; Read middle gimbal angle
					; CDUZ = middle gimbal CDU register
		INDEX	FIXLOC		; Indexed store
		TS	3		; Store to location 3 offset by FIXLOC
					; Third element of CDU triplet
		RELINT			; Re-enable interrupts
					; CDU read complete, safe to interrupt
		TC	DANZIG		; Return via DANZIG (interpretive entry)
					; Converts CDU counts to scaled angles
		COUNT*	$$/INFLT

# Page 949
# NAME -- P51 -- IMU ORIENTATION DETERMINATION
# MOD. NO. 1	23 JAN 67				LOG SECTION -- P51-P53
# MOD BY STURLAUGSON					ASSEMBLY SUNDANCE REV56
#
# FUNCTIONAL DESCRIPTION
#	DETERMINES THE INERTIAL ORIENTATION OF THE IMU.  THE PROGRAM IS SELECTED BY DSKY ENTRY.  THE SIGHTING
#	(AOTMARK) ROUTINE IS CALLED TO COLLECT AND PROCESS MARKED-STAR DATA.  AOTMARK (R53) RETURNS THE STAR NUMBER AND THE
# 	STAR LOS VECTOR IN STARAD +6.  TWO STARS ARE THUS SIGHTED.  THE ANGLE BETWEEN THE TWO STARS IS THEN CHECKED AT
#	CHKSDATA (R54).  REFSMMAT IS THEN COMPUTED AT AXISGEN.
#
# CALLING SEQUENCE
#	THE PROGRAM IS CALLED BY THE ASTRONAUT BY DSKY ENTRY.
#
# SUBROUTINES CALLED
#	GOPERF3
#	GOPERF1
#	GODSPR
#	IMUCOARS
#	IMUFIN20
#	AOTMARK (R53)
#	CHKSDATA (R54)
#	MKRELEAS
#	AXISGEN
#	MATMOVE
#
# ALARMS
#	NONE.
#
# ERASABLE INITIALIZATION
#	IMU ZERO FLAG SHOULD BE SET.
#
# OUTPUT
#	REFSMMAT
#	REFSMFLG
#
# DEBRIS
#	WORK AREA
#	STARAD
#	STARIND
#	BESTI
#	BESTJ

; ============================================================================
; PROGRAM P51 -- IMU ORIENTATION DETERMINATION
;
; COMMENT-ONLY READERS: P51 determines the IMU's orientation when it is
; unknown (such as after IMU power-up or a potential gimbal lock condition).
; The astronaut sights two known stars through the Alignment Optical
; Telescope (AOT) while the spacecraft attitude is arbitrary. From these
; two star sightings and knowledge of the stars' true inertial positions,
; the computer calculates the IMU platform's actual orientation and creates
; a new REFSMMAT (Reference Stable Member Matrix) describing this orientation.
; This program was particularly important when the LM undocked from the
; Command Module, as it established an independent navigation reference.
;
; CODE-ALONG READERS: P51 entry checks IMU status via IMUCHK, then offers
; optional coarse align (V41) to zero gimbals. Main sequence: (1) call
; AOTMARK/R53 twice to collect two star sightings stored in STARAD, STARAD+6,
; (2) validate sighting geometry via CHKSDATA/R54, (3) compute new REFSMMAT
; via AXISGEN using two star vectors to establish coordinate frame. STARIND
; tracks first/second star. PHASCHNG protects critical sections. IMUFINE
; performs fine alignment torquing after REFSMMAT computed. Exit via ENDOFJOB.
; ============================================================================

		COUNT*	$$/P51
# Page 950
P51		TC	BANKCALL	# IS ISS ON - IF NOT, IMUCHK WILL SEND
		CADR	IMUCHK		# ALARM CODE 210 AND EXIT VIA GOTOPOOH.

		CAF	OCT15
		TC	BANKCALL
		CADR	GOPERF1
		TC	GOTOPOOH	# TERM.
		TCF	P51B		# V33
		TC	PHASCHNG
		OCT	04024

		CAF	ZERO
		TS	THETAD		# ZERO THE GIMBALS
		TS	THETAD +1
		TS	THETAD +2
		CAF	V06N22
		TC	BANKCALL
		CADR	GODSPRET
		CAF	V41K		# NOW DISPLAY COARSE ALIGN VERB 41
		TC	BANKCALL
		CADR	GODSPRET
		TC	INTPRET
		CALL
			COARSE
		EXIT
		TC	PHASCHNG
		OCT	04024
		TCF	P51 +2

P51B		TC	PHASCHNG
		OCT	00014
		TC	INTPRET
		CALL
			NCOARSE
		SSP	SETPD
			STARIND		# INDEX -- STAR 1 OR 2
			0
			0
P51C		EXIT
		TC	PHASCHNG
		OCT	04024

		TC	BANKCALL
		CADR	AOTMARK		# R53
		TC	BANKCALL
		CADR	AOTSTALL
		TC	CURTAINS
		CCS	STARIND
		TCF	P51D +1
		TC	INTPRET
# Page 951
		VLOAD
			STARAD +6
		STORE	STARSAV1
P51D		EXIT
		TC	PHASCHNG
		OCT	04024

		CCS	STARIND
		TCF	P51E
		TC	PHASCHNG
		OCT	04024

		TC	INTPRET
		DLOAD	CALL
			TSIGHT
			PLANET
		STORE	PLANVEC
		EXIT
		CAF	BIT1
		TS	STARIND
		TCF	P51C +1		# DO SECOND STAR
P51E		TC	PHASCHNG
		OCT	04024

		TC	INTPRET
		DLOAD	CALL
			TSIGHT
			PLANET
		STOVL	12D
			PLANVEC
		STOVL	6D
			STARSAV1
		STOVL	STARAD
			STARSAV2
		STCALL	STARAD +6
			CHKSDATA	# CHECK STAR ANGLES IN STARAD AND
		BON	EXIT
			FREEFLAG
			P51G
		TC	P51 +2
P51G		CALL
			AXISGEN		# COME BACK WITH REFSMMAT IN XDC
		AXC,1	AXC,2
			XDC
			REFSMMAT
		CALL
			MATMOVE
		SET	EXIT
			REFSMFLG
		TC	GOTOPOOH	# FINIS
# Page 952
V41K		VN	4100

; ============================================================================
; COARSE - Perform Coarse and Fine IMU Alignment
;
; Executes physical platform alignment sequence after desired gimbal angles
; have been computed and stored in THETAD,+1,+2 by S52.2. Performs both
; coarse alignment (rapid drive to approximate angles) and fine alignment
; (precise positioning to commanded angles). Essential step in every IMU
; alignment procedure (P51, P52, P53) to physically orient the stable
; member platform to match the computed desired orientation.
;
; SEQUENCE:
; 1. Coarse alignment - Drives gimbals rapidly to within ±5 degrees of target
;    (takes ~30 seconds, may cause gimbal motion visible to crew)
; 2. Wait for gyro stabilization after coarse motion
; 3. Fine alignment - Drives gimbals precisely to target (±1 arcminute)
;    (takes ~15 seconds, uses slower gimbal drive rates)
; 4. Wait for gyro stabilization after fine alignment
;
; INPUTS:
;   THETAD, THETAD+1, THETAD+2 = Desired gimbal angles (CDU units)
;       Computed by S52.2 from desired REFSMMAT and vehicle attitude
;
; OUTPUTS:
;   IMU gimbals physically positioned to desired orientation
;   Platform stable and ready for star sightings or navigation
;
; SUBROUTINES CALLED:
;   IMUCOARS - Performs coarse alignment gimbal drive
;   IMUSTALL - Waits for IMU to stabilize (gyros to settle)
;   IMUFINE - Performs fine alignment gimbal drive
;   CURTAINS - Exit point if alignment fails (hardware malfunction)
;
; FAILURE HANDLING:
;   If IMUCOARS or IMUFINE fail (gimbal drive errors, CDU failures),
;   IMUSTALL branches to CURTAINS which terminates alignment program
;   and alerts crew to IMU malfunction.
;
; HISTORICAL NOTE: During Apollo 11 lunar orbit, this routine executed
; multiple times for IMU realignments, physically moving the platform
; to maintain accurate orientation despite accumulated gyro drift.
; ============================================================================

COARSE		EXIT			; Exit interpretive mode for hardware control
		TC	BANKCALL	; Call coarse alignment routine
		CADR	IMUCOARS	; Drive gimbals to approximate target
					; (rapid slew, completes in ~30 seconds)
		TC	BANKCALL	; Wait for gyro stabilization
		CADR	IMUSTALL	; (gyros must settle after motion)
		TC	CURTAINS	; Failure exit if coarse align failed
					; (hardware problem or gimbal limits)
		TC	BANKCALL	; Call fine alignment routine
		CADR	IMUFINE		; Drive gimbals to precise target
					; (slow positioning, ±1 arcmin accuracy)
		TC	BANKCALL	; Wait for final stabilization
		CADR	IMUSTALL	; (platform must be motionless for marks)
		TC	CURTAINS	; Failure exit if fine align failed
		TC	INTPRET		; Return to interpretive mode
		RVQ			; Return to caller
					; Platform aligned, ready for star sighting

; ============================================================================
; NCOARSE - Initialize PIPAs Without Physical Platform Alignment
;
; Performs PIPA (accelerometer) initialization without driving IMU gimbals.
; Used when IMU platform orientation is already acceptable (no coarse/fine
; alignment needed) but accelerometer readings must be zeroed for navigation
; updates. Resets PIPA counters and initializes gravity compensation vector
; for subsequent navigation computations.
;
; FUNCTION:
; Initializes inertial navigation integration by:
; 1. Recording current time as PIPA integration start time
; 2. Zeroing all three accelerometer pulse counters (PIPAX, PIPAY, PIPAZ)
; 3. Zeroing gravity compensation vector (GCOMP)
; 4. Setting drift flag to enable gyro drift computation
;
; This establishes initial conditions for W-matrix integration and velocity
; state updates without physically moving the platform. Used in alignment
; procedures where platform orientation is already correct (e.g., after
; power-up restart with known REFSMMAT).
;
; INPUTS:
;   TIME1 = Current AGC time (centiseconds)
;
; OUTPUTS:
;   1/PIPADT = Time of last PIPA reading initialization
;              (Start point for integration interval)
;   PIPAX = X-axis accelerometer pulse count (zeroed)
;   PIPAY = Y-axis accelerometer pulse count (zeroed)
;   PIPAZ = Z-axis accelerometer pulse count (zeroed)
;   GCOMP = Gravity compensation vector (zeroed)
;   DRIFTFLG = Set to enable drift calculation
;
; HISTORICAL NOTE: During Apollo missions, NCOARSE was used when IMU
; platform was already aligned but navigation state needed reinitialization,
; such as after computer restarts or when switching between programs.
; ============================================================================

NCOARSE		EXIT			; Exit interpretive mode
		CA	TIME1		; Load current AGC time
		TS	1/PIPADT	; Store as PIPA integration start time
					; (marks beginning of integration interval)
		CS	ZERO		; Load negative zero (clears all bits)
		TS	PIPAX		; Zero X-axis accelerometer counter
		TS	PIPAY		; Zero Y-axis accelerometer counter
		TS	PIPAZ		; Zero Z-axis accelerometer counter
					; All PIPA pulse counts reset for new
					; integration period starting now
		TC	INTPRET		; Return to interpretive mode
		VLOAD			; Load zero vector
			ZEROVEC		; (all three components zero)
		STORE	GCOMP		; Zero gravity compensation vector
					; GCOMP will accumulate gravity effects
					; during powered flight guidance
		SET	RVQ		; Set drift flag and return
			DRIFTFLG	; Enable gyro drift computation
					; Navigation state ready for updates

# Page 953
# NAME -- S52.2
# FUNCTION -- COMPUTE GIMBAL ANGLES FOR DESIRED SM AND PRESENT VEHICLE
# CALL -- CALL S52.2
# INPUT -- X,Y,ZSMD
# OUTPUT -- OGC,IGC,MGC,THETAD,+1,+2
# SUBROUTINES -- CDUTRIG, CALCSMSC, MATMOVE, CALCGA

		COUNT*	$$/S52.1
; ============================================================================
; S52.2 - COMPUTE GIMBAL ANGLES FOR NEW IMU ORIENTATION
;
; After the crew has selected a desired IMU orientation (preferred, nominal,
; or REFSMMAT), this routine computes what the gimbal angles should be to
; achieve that orientation given the present vehicle attitude. These computed
; angles are stored in THETAD, THETAD+1, THETAD+2 and will be used by CAL53A
; to coarse align the IMU to the new orientation.
;
; The calculation transforms the vehicle attitude from the current IMU
; reference frame (REFSMMAT) to the new desired reference frame, determining
; the gimbal angles needed for the physical platform realignment.
;
; Technical: Reads current gimbal angles (CDUs), transforms navigation base
; coordinate system (XNB, YNB, ZNB) through REFSMMAT, unitizes vectors, and
; calls CALCGA to compute target gimbal angles.
; ============================================================================

S52.2		STQ	CALL		; Save return address, read current CDU angles
			QMAJ
			CDUTRIG
		CALL			; Calculate sine/cosine of gimbal angles
			CALCSMSC		; to get current SM to SC transformation
		AXT,1	SSP		; Initialize loop counter for 3 axis vectors
			18D		; Start at offset 18 (ZNB vector)
			S1
			6D		; Decrement by 6 each iteration
			
; Loop through XNB, YNB, ZNB (navigation base coordinate vectors) and transform
; them through REFSMMAT to get desired stable member coordinates. This expresses
; the vehicle body axes in the new IMU reference frame.

S52.2A		VLOAD*	VXM		; Load nav base vector, multiply by REFSMMAT
			XNB +18D,1	; XNB+18=ZNB, XNB+12=YNB, XNB+6=XNB
			REFSMMAT	; Transform to desired SM coordinates
		UNIT			; Normalize to unit vector
		STORE	XNB +18D,1	; Store transformed unit vector
		TIX,1			; Decrement index, loop for all 3 vectors
			S52.2A
			
; Now we have the navigation base vectors expressed in the desired stable member
; frame. Move this transformation matrix to the working location and compute the
; gimbal angles that will achieve this orientation.

S52.2.1		AXC,1	AXC,2		; Set up matrix move pointers
			XSMD		; Source: transformed NB vectors
			XSM		; Destination: working matrix
		CALL
			MATMOVE		; Copy the transformation matrix
		CALL
			CALCGA		; Compute gimbal angles (OGA, IGA, MGA)
		GOTO			; Return to caller with angles in THETAD
			QMAJ

# Page 954
# NAME -- S52.3
# FUNCTION --	XSMD= UNIT R
#		YSMD= UNIT(V X R)
#		ZSMD= UNIT(XSMD X YSMD)
# CALL --	DLOAD	CALL
#			TALIGN
#			S52.3
# INPUT --	TIME OF ALIGNMENT IN MPAC
# OUTPUT --	X,Y,ZSMD
# SUBROUTINES -- CSMCONIC

; ============================================================================
; S52.3 - COMPUTE NOMINAL IMU ORIENTATION
;
; This routine computes the "nominal" IMU orientation, which is an orbit-
; referenced coordinate system based on the vehicle's position and velocity
; at the alignment time selected by the crew.
;
; The nominal orientation is defined as:
;   X-axis (XSMD) = Unit position vector (radial direction from Moon center)
;   Y-axis (YSMD) = Unit(Velocity × Position) (orbit normal direction)
;   Z-axis (ZSMD) = Unit(X × Y) (completes right-handed system)
;
; This creates an orientation where the X-axis points toward/away from the
; Moon, the Y-axis is perpendicular to the orbit plane, and the Z-axis is
; roughly in the direction of orbital motion. This is useful for orbit
; operations where attitude relative to the local vertical is important.
;
; Historical: During lunar orbit operations, periodic IMU realignments to
; nominal orientation helped maintain navigation accuracy and provided a
; consistent attitude reference for rendezvous operations.
; ============================================================================

		COUNT*	$$/S52.3
S52.3		STQ			; Save return address
			QMAJ
		STCALL	TDEC1		; Store alignment time, compute state vector
			LEMCONIC	; Get position/velocity at TALIGN
			
; Build the nominal orientation coordinate system from orbital elements.
; RATT contains position vector, VATT contains velocity vector from LEMCONIC.

		VLOAD	UNIT		; Load position vector
			RATT
		STOVL	XSMD		; Store as X-axis (radial), load velocity
			VATT
		VXV	UNIT		; V × R, normalized = orbit normal
			RATT
		STOVL	YSMD		; Store as Y-axis (normal to orbit plane)
			XSMD
		VXV	UNIT		; X × Y completes right-handed triad
			YSMD
		STCALL	ZSMD		; Store as Z-axis, return
			QMAJ

# Page 955
# NAME -- R52 (AUTOMATIC OPTICS POSITIONING ROUTINE)
#
# FUNCTION -- POINT THE AOT APTICS AXIS BY MANEUVERING THE LEM TO A NAVIGATION
# STAR SELECTED BY ALIGNMENT PROGRAMS OR DSKY INPUT
#
# CALLING -- CALL R52
#
# INPUT -- BESTI AND BESTJ (STAR CODES TIMES 6)
#
# OUTPUT -- STAR CODE IN BITS 1-6, DETENT CODE IN BITS 7-9
# (NO CHECK IS MADE TO INSURE THE DETENT CODE TO BE VALID)
# POINTVSM-1/2 UNIT NAV STAR VEC IN SM
# SCAXIS-AOT OPTIC AXIS VEC IN NB X-Z PLANE
#
# SUBROUT -- R60LEM

; ============================================================================
; R52 - AUTOMATIC OPTICS POSITIONING ROUTINE
;
; This routine displays the selected star to the crew and then automatically
; maneuvers the Lunar Module so the Alignment Optical Telescope (AOT) is
; pointed at the star. The AOT is a fixed telescope mounted in the LM, so the
; entire spacecraft must be rotated to aim it at alignment stars.
;
; The crew sees the star identification on the DSKY (V01 N70) and can:
;   - Press V33 PROCEED to maneuver to the star
;   - Press ENTER to select a different star
;   - Press V34 to terminate
;
; Once the LM is oriented with the AOT pointing at the star, the crew can
; perform optical sightings for IMU alignment. The AOT has multiple detent
; positions (viewing angles), and this routine selects the optimal detent.
;
; For special cases (detent codes 0 or 7), the crew manually enters azimuth
; and elevation angles for COAS (Crew Optical Alignment Sight) calibration.
;
; Historical: During Apollo 11's lunar orbit operations, Aldrin used the AOT
; to sight alignment stars while Armstrong maintained spacecraft attitude.
; The automatic positioning saved time and propellant during alignment.
; ============================================================================

		COUNT*	$$/R52
R52		STQ	EXIT		; Save return address, exit interpreter
			SAVQR52
		INDEX	STARIND		; Index to first or second star
		CA	BESTI		# PICK UP STARCODE DETERMINED BY R56
		EXTEND
		MP	1/6TH		; Scale star code (stored as x6)
		AD	BIT8		# SET DETENT POSITION 2 (default 45°)
		TS	STARCODE	# SCALE AND STORE IN STARCODE

; Display the star code to the crew and wait for response.
; Star codes 1-50 identify navigation stars from the star catalog.
; The detent code (bits 7-9) specifies which AOT viewing position to use.

R52A		CAF	V01N70		; V01 = Display, N70 = Star code
		TC	BANKCALL
		CADR	GOFLASH		# DISPLAY STARCODE AND WAIT FOR RESPONSE
		TC	GOTOPOOH	# V34 -- TERMINATE
		TCF	R52B		# V33 -- PROCEED TO ORIENT LEM
		TCF	R52A		# ENTER -- SELECT NEW STARCODE -- RECYCLE

; The crew has pressed PROCEED (V33) to begin automatic maneuvering.
; Now extract the detent code to determine which AOT viewing angle to use.

R52B		TC	DOWNFLAG	; Clear 3-axis flag
		ADRES	3AXISFLG	# BIT6 OF FLAGWRD5 ZERO TO ALLOW VECPOINT
		CA	STARCODE	# GRAB DETENT CODE (bits 7-9)
		MASK	HIGH9		; Isolate upper bits
		EXTEND
		MP	BIT9		; Extract detent code (0-7)
		TS	L		# TEMP STORE DETENT

; Check for special detent codes that require manual crew input.
; Detent code 0 = COAS calibration (manual azimuth/elevation entry)
; Detent code 7 = COAS sighting (manual azimuth/elevation entry)
; Detent codes 1-6 = AOT detent positions (automatic lookup)

		EXTEND
		BZMF	GETAZEL		# CODE 0, COAS CALIBRATION

		AD	NEG7		; Check if detent code = 7
		EXTEND
		BZF	GETAZEL		# CODE 7, COAS SIGHTING

; For detent codes 1-6, look up the azimuth angle from the AOTAZ table.
; Each AOT detent position has a fixed azimuth (viewing angle around the LM).
; Elevation is fixed at 45 degrees for all standard detents.

		EBANK=	XYMARK
		CA	EBANK7		; Switch to EBANK7 for AOTAZ table access
		TS	EBANK
# Page 956
		INDEX	L		; Use detent code as table index
		CA	AOTAZ -1	# PICK UP AZ CORRESPONDING TO DETENT
		TS	L		; Save azimuth temporarily
		EBANK=	XSM
		CA	EBANK5		# CHANGE TO EBANK5 BUT DON'T DISTURB L
		TS	EBANK
		CA	BIT13		# SET ELV TO 45 DEG (BIT13 = 45° scaled)
		XCH	L		# SET C(A)=AZ, C(L)=45 DEG
		TCF	AZEL		# GO COMP OPTIC AXIS

; For COAS sightings (detent codes 0 or 7), the crew manually enters the
; azimuth and elevation angles using the DSKY. COAS (Crew Optical Alignment
; Sight) is a simple reticle sight used for backup alignment when AOT is
; unavailable or for calibration checks.

GETAZEL		CAF	V06N87		# CODE 0 OR 7, GET AZ AND EL KEY IN
		TC	BANKCALL
		CADR	GOFLASH		; V06 N87 = Load azimuth, elevation
		TC	GOTOPOOH	# V34 -- TERMINATE
		TCF	+2		# PROCEED -- CALC OPTIC AXIS
		TCF	GETAZEL		# ENTER -- RECYCLE

		EXTEND
		DCA	AZ		# PICK UP AZ AND EL IN SP 2'S COMP
		
; Now compute the optic axis vector in navigation base (NB) coordinates.
; This vector represents the direction the AOT or COAS is pointing.

AZEL		INDEX	FIXLOC		# JAM AZ AND EL IN 8 AND 9 OF VAC
		DXCH	8D		; Store in interpreter push-down list
		TC	INTPRET		; Enter interpretive mode
		CALL			# GO COMPUTE OPTIC AXIS AND STORE IN
			OANB		# SCAXIS IN NB COORDS
			
; Compute the star's unit vector in stable member (SM) coordinates.
; This involves getting current time, computing star position (PLANET routine),
; and transforming to SM coordinates using REFSMMAT.
			
		RTB	CALL		; Load time, compute star position
			LOADTIME
			PLANET		; Get star unit vector in reference frame
		MXV	UNIT		; Transform to SM coordinates
			REFSMMAT
		STORE	POINTVSM	# STORE FOR VECPOINT (desired direction)

; Now call R60LEM to compute and execute the maneuver that will align the
; optic axis (SCAXIS) with the star line-of-sight (POINTVSM).

		EXIT
		TC	BANKCALL
		CADR	R60LEM		# GO TORQUE LEM OPTIC AXIS TO STAR LOS

; If this was a COAS calibration sighting (code 0), recycle for another
; sighting. Otherwise, return to the calling routine (R51 fine alignment).

		CAF	HIGH9		# IF COAS CALIBRATION CODE 0.  RECYCLE
		MASK	STARCODE	; Extract star/detent code
		EXTEND
		BZF	R52A		; If code 0, take another sighting

		TC	INTPRET		# RETURN FROM KALCMANU
		GOTO
			SAVQR52		# RETURN TO CALLER

; Constants used by R52 and related routines.

1/6TH		DEC	.1666667	; One-sixth constant
V01N70		VN	0170		; Display verb/noun code
V06N87		VN	687		; Load verb/noun for azimuth/elevation

# Page 957
# LUNAR SURFACE STAR ACQUISITION

		BANK	15
		SETLOC	P50S
		BANK
		COUNT*	$$/R59

; ============================================================================
; R59 - LUNAR SURFACE STAR ACQUISITION
;
; This routine assists the crew in acquiring a star for IMU alignment when
; the Lunar Module is on the lunar surface. It computes which AOT detent
; position provides the best view of a selected catalog star, considering
; the LM's orientation on the surface.
;
; COMMENT-ONLY READERS: After landing on the Moon, the crew needs to realign
; the IMU periodically. This routine helps them find stars through the AOT.
;
; CODE-ALONG READERS: R59 transforms catalog star vectors to navigation base
; coordinates, then searches through six AOT detent positions to find which
; provides the best viewing angle.
; ============================================================================

R59 		CS	FLAGWRD3
		MASK	REFSMBIT	# IF REFSMMAT FLAG CLEAR BYPASS STAR ACQUIRE
		CCS	A		; Check if REFSMMAT is valid
		TCF	R59OUT		# NO REFSMMAT GO TO AOTMARK

; Request crew to select which catalog star they want to sight.
; V01 N70 displays "Please load star code" on DSKY.

		CAF	V01N70*		# SELECT STAR CODE FOR ACQUISITION
		TC	BANKCALL
		CADR	GOFLASH		; Display and wait for crew input
		TC	GOTOPOOH	# V34 -- TERMINATE
		TCF	R59A		# V33 -- PROCEED
		TCF	R59		# V32 -- RECYCLE

; Process the star code entered by the crew. Validate that it's a catalog star
; (codes 1-50) and compute the index into the star catalog table.

R59A		CS	HIGH9		# GRAB STARCODE FOR INDEX
		MASK	AOTCODE		; Extract star code bits
		EXTEND
		MP	REVCNT		# JUST 6 (multiply by 6 for table index)
		XCH	L		; Result in L register
		INDEX	STARIND
		TS	BESTI		; Save for later use
		INDEX	FIXLOC
		TS	X1		# CODE X 6 FOR CATLOG STAR INDEX
		EXTEND
		BZF	R59OUT		# BYPASS ACQUISITION IF NOT CATLOG STAR
		COM			; Check if code > 50
		AD	DEC227
		EXTEND
		BZMF	R59OUT		; If invalid star code, skip acquisition

; Look up the star's unit vector from the catalog, transform it to stable
; member coordinates using REFSMMAT, then to navigation base coordinates
; using current gimbal angles. This gives us the star direction in LM body
; coordinates.

		TC	INTPRET
		VLOAD*	MXV		; Load star vector, transform to SM
			CATLOG,1	# GRAB STAR VECTOR (reference coords)
			REFSMMAT	# TRANSFORM TO SM
		UNIT	CALL		; Normalize and transform to NB
			CDU*SMNB	; Using current gimbal angles
		STORE	STAR		# TEMP STORE STAR VEC(NB)
		EXIT

; Now search through the six AOT detent positions to find which one provides
; the best viewing angle for this star. Start with detent position 1 (azimuth
; -60 degrees). The AOT has six fixed azimuth positions: -60, -30, 0, +30,
; +60, +90 degrees, all at 45-degree elevation.

		CAF	BIT1		# INITIALIZE AZ POSITION COD TO 1 (-60)
		TS	POSCODE		; Position code counter

		EBANK=	XYMARK
INCAZ		CA	EBANK7		; Switch to EBANK7 for AOTAZ table
		TS	EBANK

# Page 958
		INDEX	POSCODE		; Use position code as index
		CA	AOTAZ -1	# PICK UP AZ CORRESPONDING TO POSCODE
		TS	L		; Save azimuth temporarily

		EBANK=	XSM
		CA	EBANK5		; Switch to EBANK5
		TS	EBANK

		CA	BIT13		# SET ELV TO 45 DEG (all detents at 45°)
		XCH	L		# SET C(A)=AZ, C(L)=45 DEG
		TS	QMIN		# STORE QMIN=AZ FOR LATER
		INDEX	FIXLOC
		DXCH	8D		# JAM AZ IN 8D, 45 DEG IN 9D FOR OANB

	; Compute the optic axis vector for this AOT detent position (azimuth and
; 45-degree elevation). The OANB routine returns the optic axis direction
; in navigation base coordinates in SCAXIS.

	TC	INTPRET
		CALL
			OANB		# GO CALC OPTIC AXIS WRT NB
		
; Calculate the angle between the star direction and the optic axis.
; This angular separation determines if the star is within the AOT's
; 30-degree field of view and how far from center it appears.

		VLOAD	DOT
			STAR		# DOT STAR WITH OA
			SCAXIS		; Dot product gives cos(angle)
		SL1	ARCCOS		; Scale and take arccos to get angle
		STORE	24D		# TEMP STORE ARCCOS(STAR.OPTAXIS)

; Check if the angular separation is less than 30 degrees. If the star
; is more than 30 degrees from the optic axis, it cannot be seen through
; the AOT at this detent position, so try the next detent.

		DSU	BPL
			DEG30		# SEE IF STAR IN AOT FIELD-OF-VIEW
			NXAX		# NOT IN FIELD -- TRY NEXT POSITION
; Star is within the 30-degree field of view. Now check if it's very close
; to the center (within 0.5 degrees). If yes, the spiral and cursor displays
; should both be zeroed since the star is already at the reticle center.

		DLOAD	DSU		# SEE IF STAR AT FIELD CENTER
			24D
			DEG.5
		BMN	DLOAD		# CALC SPIRAL AND CURSOR
			ZSPCR		# GO ZERO CURSOR AND SPIRAL
			24D		# GET SPIRAL
			
; Compute the spiral angle, which indicates how far from center the star
; appears in the AOT field of view. The spiral is scaled as revolutions.

		DMP	SL4
			3/4		# 12 SCALED AT 16
		STOVL	24D		# 12(ARCCOS(AO.STAR)) SCALED IN REVS

; Compute the cursor angle (theta), which indicates the rotational position
; where the star appears in the AOT reticle. This requires constructing an
; orthogonal coordinate system based on the optic axis and calculating the
; angular position of the star within that system.

			SCAXIS		# OA
		VXV	UNIT		; Construct first basis vector
			XUNIT		; perpendicular to optic axis
		PUSH	VXV		# OA X UNITX 	PD 0-5
			SCAXIS
		VCOMP			; Get opposite direction
		UNIT	PDVL		# UNIT(OA X (OA X UNITX))	PD 6-11
			SCAXIS		; Second basis vector perpendicular
		VXV	UNIT		; to both optic axis and first vector
			STAR
		PUSH	DOT		# 1/2(OA X STAR)	PD 12-17
			0		# DOT WITH 1/2(OA X UNITX) FOR YROT
		SL1	ARCCOS		; Compute angle in the constructed frame
		STOVL	26D		# STORE THET SCALED IN REVS
# Page 959
; Determine the sign of theta by checking the second component. If negative,
; we need to measure the angle in the opposite direction (360 - theta) to get
; the correct cursor position in the AOT reticle coordinate system.

		DOT			# UP 12-17, UP 6-11 FOR C2
		BPL	DLOAD		# IF THET NEG -- GET 360-THET
			R59D
			ABOUTONE	; Load 1.0 (representing 360 degrees)
		DSU			; Subtract theta from 360 degrees
			26D
		STORE	26D		# 360-THET SCALED IN REVS

; R59D - Format spiral and cursor angles for DSKY display
; The computed angles must be combined with the AOT detent azimuth and
; converted to half-revolution units for display to the crew.

R59D		SLOAD	SR1
			QMIN		# RESCALE AZ(N) TO REVS
		DAD	PUSH		# PUSH YROT + AZ(N) REVS
			26D		; Add cursor angle (theta) to azimuth
		RTB			; to get absolute cursor position
			1STO2S		; Convert to half-revolutions
		STODL	CURSOR		# YROT IN 1/2 REVS
			24D		# LOAD SROT IN REVS
		DAD			# 12(SEP) + YROT
		RTB			; Combine spiral angle with cursor
			1STO2S		; Convert to half-revolutions
		STORE	SPIRAL		# SROT IN 1/2 REVS
		EXIT
		TCF	79DISP		# GO DISPLAY CURSOR-SPIRAL-POS CODE

; ZSPCR - Zero spiral and cursor for centered star
; When the star is within 0.5 degrees of the optic axis center, the crew
; does not need to adjust the reticle. Display zero for both cursor and
; spiral to indicate the star is already perfectly aligned.

ZSPCR		EXIT
		CAF	ZERO		# STAR ALMOST OPTIC AXIS, ZERO CURSOR
		TS	CURSOR		# AND SPIRAL ANGLES
		TS	SPIRAL
		TCF	79DISP		; Display zeros to crew

; NXAX - Try next AOT detent position
; The star is not visible at this azimuth position. Increment to the next
; of the six AOT detent positions (separated by 60 degrees) and check again.
; If all six positions have been tried without finding the star, issue alarm.

NXAX		EXIT
		INCR	POSCODE		; Move to next detent position
		CS	POSCODE
		AD	SEVEN		; Check if tried all 6 positions
		EXTEND
		BZMF	R59ALM		# THIS STAR NOT AT ANY POSITION
		TCF	INCAZ		; Try next azimuth position

; R59ALM - Star not visible in any AOT position
; After checking all six AOT detent positions, the selected star cannot be
; seen through the alignment optical telescope. This may occur if the vehicle
; attitude is unfavorable or if the wrong star was selected. Alarm 404 alerts
; the crew to this condition.

R59ALM		TC	ALARM		# THIS STAR CAN'T BE LOCATED IN AOT FIELD
		OCT	404		; Alarm code 404
		CAF	VB05N09		# DISPLAY ALARM
		TC	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH	# VB34 -- TERMINATE
		TCF	R59OUT		# VB33 -- PROCEED, GO WITHOUT ACQUIRE
		TCF	R59		# VB32 -- RECYCLE AND TRY ANOTHER STAR

; 79DISP - Display cursor, spiral, and position code to crew
; Verb 06 Noun 79 displays three values on the DSKY:
;   R1: Cursor angle (half-revolutions) - rotational position in reticle
;   R2: Spiral angle (half-revolutions) - radial distance from center  
;   R3: Position code (1-6) - which AOT detent to use
; The crew uses these values to manually align the AOT reticle with the star.

79DISP		CAF	V06N79		# DISPLAY CURSOR, SPIRAL AND POS CODE
		TC	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH	# V34 -- TERMINATE
# Page 960
		TCF	R59E		# V33 -- PROCEED TO MARK ROUTINE
		TCF	R59		# V32 -- RECYCLE TO TOP OF R59 AGAIN

; R59E - Encode AOT detent position with star number
; After the crew confirms they can see the star at the displayed position,
; this section encodes the detent position (1-6) into bits 7-9 of AOTCODE
; while preserving the star number in bits 1-6. This encoded value tells
; AOTMARK which AOT detent the crew will use for the sighting.

R59E		CAF	SEVEN		# GET DETENT CODE CORRESPONDING TO POSCODE
		MASK	POSCODE		; Isolate detent position code
		EXTEND
		MP	BIT7		# DETEND CODE NOW IN L
		CS	HIGH9		; Clear bits 7-9
		MASK	AOTCODE		# ISOLATE STAR NO BIT 1-6
		AD	L		; Combine star number with detent code
		TS	AOTCODE		# STORE DETENT 7-9

; R59OUT - Perform AOT mark on star
; With the AOT detent position encoded, call AOTMARK to set up the star
; sighting. Then AOTSTALL waits for the crew to complete the mark. During
; Apollo 11's lunar orbit, these AOT star marks were critical for verifying
; the IMU alignment before the descent to the lunar surface.
;
; The crew would:
; 1. Rotate the LM to the computed attitude
; 2. Select the specified AOT detent (1-6) 
; 3. Adjust the reticle cursor and spiral per displayed values
; 4. Center the star in the reticle
; 5. Press MARK to record the sighting time

R59OUT		TC	BANKCALL	# GO TO AOTMARK FOR SIGHTING
		CADR	AOTMARK
		TC	BANKCALL
		CADR	AOTSTALL	# SLEEP TILL SIGHTING DONE
		TC	CURTAINS	# BADEND RETURN FROM AOTMARK
		TCF	R59RET		# RETURN TO 1 STAR OR 2STAR

; R59 Constants and DSKY codes
V01N70*		VN 	170		; Display mark data
V06N79		VN	679		; Display cursor/spiral/position
DEG30		2DEC	.083333333	# 30 DEGREES
DEG.5		2DEC	.00138888	# .5 DEGREES SCALED IN REVS.
DEG60		OCT	12525		# 60 DEG CDU SCALING
CURSOR		EQUALS	GDT/2		; Cursor angle storage
SPIRAL		EQUALS	GDT/2 +2	; Spiral angle storage
POSCODE		EQUALS	GDT/2 +4	; AOT position code storage

# Page 961
# ============================================================================
# TRANSITION: From star acquisition (R59) to celestial body vector computation
#
# Having marked stars through the AOT, the alignment process now requires
# precise reference vectors for those celestial bodies. The PLANET routine
# provides unit vectors for any celestial object: stars from the onboard
# catalog, Sun/Earth/Moon from computed ephemerides, or planets from crew
# input. These reference vectors are compared with the marked line-of-sight
# vectors to compute the IMU alignment error and refine the platform attitude.
# ============================================================================

# NAME -- 	PLANET
# FUNCTION --	TO PROVIDE THE REFERENCE VECTOR FOR THE SIGHTED CELESTIAL
#		BODY.  STARS ARE FETCHED FROM THE CATALOG, SUN, EARTH AND
#		MOON ARE COMPUTED BY LOCSAM, PLANET VECTORS ARE ENTERED
#		BY DSKY INPUT.
# CALL --	CALL
#			PLANET
# INPUT --	TIME IN MPAC
# OUTPUT --	VECTOR IN MPAC
# SUBROUTINES -- LOCSAM
# DEBRIS --	VAC, STARAD - STARAD +17

		SETLOC	P50S
		BANK
		COUNT*	$$/P51

; PLANET - Celestial body reference vector computation
; This routine provides the inertial reference vector for any celestial body
; used in IMU alignment. It handles:
;   Stars: Fetched from onboard star catalog (37 stars)
;   Sun/Earth/Moon: Computed by LOCSAM ephemeris routine
;   Planets: Manually entered by crew via DSKY
;
; The reference vector is compared with the marked line-of-sight vector from
; the AOT to compute alignment corrections. During Apollo 11's lunar orbit,
; star vectors from this routine were essential for verifying IMU accuracy
; before the descent to the lunar surface.

PLANET		STOVL	TSIGHT		; Store sighting time
			ZEROVEC		; Initialize STARAD to zero
		STORE	STARAD
		STQ	EXIT		; Save return address
			GCTR
		CS	HIGH9		; Clear bits 7-9
		MASK	AOTCODE		; Extract star number (bits 1-6)
		EXTEND
		MP	REVCNT		; Multiply by revolution count
		XCH	L		; Result to A register
		INDEX	STARIND		; Index by star indicator
		TS	BESTI		; Store to BESTI array
		CCS	A		; Check if star number > 0
		TCF	NOTPLAN		; Branch to star catalog lookup

; Manual planet entry (star code = 0)
; When the star code is zero, this prompts the crew to manually enter
; the planet's unit vector via Verb 06 Noun 88. This capability allowed
; use of planets not in the star catalog for alignment purposes.

		CAF	VNPLANV		; V06N88 - Load and display vector
		TC	BANKCALL
		CADR	GOFLASH		; Flash display, wait for crew input
		TC	-3		; V34 - recycle to GOFLASH
		TC	+2		; V33 - proceed with entered vector
		TC	-5		; V32 - recycle to top
		TC	INTPRET
		VLOAD	UNIT		; Load manually entered vector
			STARAD		; and normalize to unit vector
		GOTO
			GCTR		; Return to caller
; NOTPLAN - Star catalog or ephemeris body lookup
; Star number was non-zero, so determine if it's a catalog star or
; computed ephemeris body (Sun/Earth/Moon). The onboard star catalog
; contains 37 navigational stars with unit vectors in the Basic Reference
; Coordinate System (BRCS). Sun, Earth, and Moon vectors are computed
; by the LOCSAM ephemeris routine using the sighting time.

NOTPLAN		CS	A		; Negate star number
		AD	DEC227		; Subtract 227 (catalog threshold)
		EXTEND
		BZMF	CALSAM1		; If <= 227, compute ephemeris body
		INDEX	STARIND		; Otherwise, fetch from star catalog
		CA	BESTI		; Get star number
		INDEX	FIXLOC		; Index into catalog
		TS	X1		; Store catalog index
		TC	INTPRET
# Page 962
		VLOAD*	GOTO		; Load star vector from catalog
			CATLOG,1	; Indexed by X1
			GCTR		; Return to caller
; CALSAM1 / CALSAM - Compute Sun, Earth, or Moon vector
; For star codes > 227, compute the ephemeris position of Sun, Earth, or Moon
; at the sighting time. LOCSAM uses polynomial approximations to lunar and
; solar ephemerides, providing accuracy sufficient for IMU alignment. The
; Earth vector is simply the negative of the spacecraft position vector.
;
; During Apollo 11's translunar coast, Earth sightings verified navigation
; accuracy. In lunar orbit, star sightings were preferred for IMU alignment
; due to their fixed inertial positions and higher angular accuracy.

CALSAM1		TC	INTPRET
CALSAM		DLOAD	CALL		; Load sighting time
			TSIGHT
			LOCSAM		; Compute Sun, Earth, Moon vectors
		LXC,1	VLOAD		; Load index from STARIND
			STARIND
			VEARTH		; Load Earth vector
		STOVL	0D		; Save in temp location
			VSUN		; Load Sun vector
		STOVL	VEARTH		; Swap: put Sun in VEARTH
			0D		; Load saved Earth vector
		STORE	VSUN		; Swap: put Earth in VSUN
		DLOAD*	LXC,1		; This swap allows indexed selection
			BESTI,1
			MPAC
		VLOAD*	GOTO		; Load selected body vector
			STARAD -228D,1	; Offset for ephemeris body codes
			GCTR		; Return to caller
; PLANET Constants
DEC227		DEC	227		; Catalog threshold (stars 1-227)
VNPLANV		VN	0688		; V06N88 - manual planet vector entry
PIPSRINE	=	PIPASR +3	# EBANK NOT 4 SO DON'T LOAD PIPTIME1

# Page 963
; ============================================================================
; TRANSITION: From P51 IMU Orientation Programs to P57 Lunar Surface Alignment
;
; Having established the basic IMU alignment capability through P51-P53,
; the Lunar Module required specialized alignment procedures for surface
; operations. P57 provides gravity vector determination for precise IMU
; alignment when the LM is landed on the lunar surface. This routine
; determines the local vertical (gravity direction) using the known landing
; site position and LM attitude, enabling accurate 2-DOF alignment.
; ============================================================================

# GRAVITY VECTOR DETERMINATION ROUTINE
# BY KEN VINCENT

; ============================================================================
; GVDETER - Gravity Vector Determination for Lunar Surface Alignment
;
; COMMENT-ONLY READERS: After landing on the Moon, the LM's IMU needs precise
; alignment relative to the local vertical (direction of lunar gravity). This
; routine computes the gravity direction by combining the known landing site
; location with measurements of the LM's actual attitude on the surface.
; During Apollo 11, this enabled accurate navigation reference after the
; historic landing in the Sea of Tranquility.
;
; CODE-ALONG READERS: Computes gravity vector in navigation base (stable
; member) coordinates for P57 2-DOF alignment. Uses PIPAs (accelerometers)
; to measure gravity at two different IMU orientations (42° and 35° gimbal
; angles), then averages and transforms to navigation coordinates. The dual
; orientation technique reduces accelerometer bias errors. Output in GSAV
; and STARSAV1 provides reference for alignment quality assessment.
; ============================================================================
#
# FOR DETAILED DESCRIPTION SEE 504GSOP 5.6.3.2.5.
#
# THIS PROGRAM FINDS THE DIRECTION OF THE MOON'S GRAVITY
# WHILE THE LM IS IN THE MOON'S SURFACE.  IT WILL BE USED
# FOR LUNAR SURFACE ALIGNMENT.  THE GRAVITY VECTOR IS
# DETERMINED BY READING THE PIPAS WITH THE IMU AT TWO
# PARTICULAR ORIENTATIONS.  THE TWO READINGS ARE AVERAGED
# AND UNITIZED AND TRANSFORMED TO NB COORDINATES.  THE TWO
# ORIENTATIONS WERE CHOSEN TO REDUCE BIAS ERRORS IN THE
# READINGS.
#
# CALL --
#	TC	BANKCALL
#	CADR	GVDETER
#
# INPUTS --
#	PIPAS, CDUS
#
# OUTPUTS --
#	STARSAV1 = UNIT GRAVITY
#	GSAV     = DITTO
#	GRAVBIT  = 1
#
# SUBROUTINES --
#	PIPASR, IMUCOARS, IMUFINE, IMUSTALL, 1/PIPA, DELAYJOB, CDUTRIG,
#	*NBSM*, *SNMB*, CALCGA, GOFLASH
#
# DEBRIS --
#	VAC, SAC, STARAD, XSM, XNB, THETAD, DELV, COSCDU, SINCDU

; The LM has landed. To establish precise IMU alignment, the guidance computer
; must determine the direction of lunar gravity at the landing site. This
; measurement uses the IMU's accelerometers (PIPAs) at two carefully chosen
; orientations to average out instrument bias errors.

GVDETER		CAF	42DEG		; Load 42 degree gimbal angle
		TS	THETAD		; Store in THETAD (outer gimbal)
		COM			; Complement for middle gimbal
		TS	THETAD 	+1	; Store -42 degrees (180° rotation effect)
		CAF	35DEG		; Load 35 degree gimbal angle
		TS	THETAD	+2	; Store in THETAD+2 (inner gimbal)
		TC	INTPRET		; Enter interpreter for vector operations
		CLEAR	CALL		; Clear REFSMMAT flag
			REFSMFLG	; (not using reference coordinate system)
			LUNG		; Call LUNG subroutine for gimbal calc

; ============================================================================
; TRANSITION: From IMU Orientation Setup to Gravity Vector Computation
;
; With the IMU positioned at the first of two measurement orientations, the
; computer now calculates the expected gimbal angles that would rotate the
; stable member 180 degrees about the gravity vector. This mathematical
; preparation enables precise comparison between computed and measured gravity
; directions, forming the basis for alignment quality assessment.
; ============================================================================

# FIND GIMBAL ANGLES WHICH ROTATE SM 180 DEG ABOUT G VEC
#
# 	DEFINE G COOR SYS
#		      _
#		    [ X ]   [    UNIT G    ]
#		*   [ _ ]   [            _ ]
#		M = [ Y ] = [ UNITEZSM * X ]
#		    [ _ ]   [       _   _  ]
#		    [ Z ]   [ UNIT( X * Y )]
#
#	THEN ROTATED SM WRT PRESENT IS
#
# Page 964
#		           [ 1   0   0 ]
#		 *      *T [           ] *             *      *
#		XSM =   M  [ 0  -1   0 ] M  =  2 (X X ) - 1/2 I
#     		           [           ]           I J
#			   [ 0   0  -1 ]
#
# 	ALSO NB WRT PRES SM IS
#
#		 *      *  *
#		XNB = NBSM I
#
#	                         *    *
#	GIMBAL ANGLES = CALCGA( XSM, XNB )

; The computation now constructs coordinate transformation matrices. The GRAVEL
; loop processes three orthogonal unit vectors (X, Y, Z axes) through both the
; navigation base (XNB) and stable member (XSM) coordinate systems. This dual
; representation enables calculation of gimbal angles needed for the 180-degree
; rotation about the gravity vector.

		SETLOC	P50S
		BANK
		COUNT*	$$/P57
		AXT,1	SSP		; Initialize index registers
			18D		; X1 = 18 (loop counter)
			S1		; S1 = 6 (step size)
			6D		; X2 starts at -2
		LXC,2			; Load index register X2
			S1		; X2 = 6
			
; GRAVEL loop: Transform three unit vectors through coordinate systems
; Processes UNITX, UNITY, UNITZ sequentially (X2 counts: 6, 4, 2)
GRAVEL		VLOAD*	CALL		; Load unit vector (indexed)
			XUNIT -6,2	; XUNIT-6, XUNIT-4, XUNIT-2
			*NBSM*		; Transform to navigation base coords
					; (SIN and COS computed in LUNG)
		STORE	XNB +18D,1	; Store result in XNB matrix
					; (fills columns from right to left)
		VLOAD			; Load star vector (gravity unit)
			STAR		; Unit gravity from first measurement
		LXC,2	VXSC*		; Reload index, vector cross-scale
			X2		; Index for star component selection
			STAR +6,2	; Outer product computation
					; (UNITX vectors are backward)
		VSL2	LXC,2		; Shift left 2 bits, reload index
			X2		; Prepare for subtraction
		VSU*	INCR,2		; Vector subtract (indexed)
			XUNIT -6,2	; Subtract original unit vector
			2D		; Decrement index by 2
		STORE	XSM +18D,1	; Store in XSM matrix
					; (rotated stable member coords)
		TIX,1	CALL		; Test index, loop if not done
			GRAVEL		; Loop for next axis (3 total)
			CALCGA		; Calculate gimbal angles when done
; With gimbal angles computed, the program now determines alignment quality by
; comparing gravity measurements at both IMU orientations. The angular difference
; between the two measurements indicates gyro drift and alignment error.
		
		VLOAD	VSR1		; Load first gravity measurement
			GOUT		; (from first orientation)
		STCALL	STARAD +12D	; Store and call LUNG
			LUNG		; Recompute coordinate transformation
		VLOAD	VSR1		; Load second gravity measurement
			GOUT		; (from second orientation)
		VAD	UNIT		; Add first measurement, normalize
			STARAD +12D	; Average of two measurements
		STORE	STARSAV1	; Store averaged gravity vector
		DOT			; Dot product with original
			GSAV		; Compute angular difference
		SL1	ACOS		; Scale and take arccosine
					; Result is alignment error angle
# Page 965
		STORE	DSPTEM1		; Store for display to crew
		EXIT			; Exit interpreter mode
		TC	DOWNFLAG	; Clear FREEFLAG in case of recycle
		ADRES	FREEFLAG	; (prevents redundant processing)

; Display alignment error to crew on DSKY
; Crew verifies alignment quality before proceeding with mission operations
		CA	DISGRVER	; Load display verb/noun code
		TC	BANKCALL	; Bank call to display routine
		CADR	GOFLASH		; Flash display, await crew response
		TC	GOTOPOOH	; Terminate key pressed, exit
		TCF	PROGRAV		; VB33 -- PROCEED (crew accepts)
		TC	UPFLAG		; VB32 -- RECYCLE requested
		ADRES	FREEFLAG	; Store gravity and repeat measurement
					; (FREEFLAG indicates recycle mode)

; Crew has accepted the alignment quality. Store the averaged gravity vector
; as the reference for subsequent surface alignment operations.
PROGRAV		TC	PHASCHNG	; Phase change for restart protection
		OCT	04024		; Group 4, phase 2

		TC	INTPRET		; Enter interpreter mode
		VLOAD			; Load averaged gravity vector
			STARSAV1	; (computed from both measurements)
		STORE	GSAV		; Store as saved gravity reference
		EXIT			; Return to native AGC code
		CAF	FREEFBIT	; Check if FREEFLAG set
		MASK	FLAGWRD0	; Test flag bit
		CCS	A		; Count, compare, skip on result
		TCF	GVDETER		; FREEFLAG set: repeat measurement
		TCF	ATTCHK		; FREEFLAG clear: exit GVDETER

; ============================================================================
; SUBROUTINE: LUNG - Gimbal Positioning and Gravity Measurement
;
; This subroutine physically moves the IMU to the computed gimbal angles and
; then measures gravity using the PIPA accelerometers. The two-stage alignment
; (coarse then fine) ensures precise gimbal positioning. The 2-second
; measurement interval allows sufficient PIPA pulses to accumulate for accurate
; gravity vector determination.
; ============================================================================

LUNG		STQ	VLOAD		; Save return address
			QMIN		; Store in QMIN
			ZEROVEC		; Load zero vector
		STORE	GACC		; Initialize gravity accumulator
		EXIT			; Exit interpreter mode
		TC	PHASCHNG	; Phase change for restart
		OCT	04024		; Group 4, phase 2

; Position IMU gimbals to computed angles (stored in THETAD, THETAD+1, THETAD+2)
		TC	BANKCALL	; Coarse align first
		CADR	IMUCOARS	; Move gimbals to approximate position
		TC	BANKCALL	; Wait for gyros to stabilize
		CADR	IMUSTALL	; (prevents gimbal overshoot)
		TC	CURTAINS	; Failure exit if IMU malfunction
		TC	BANKCALL	; Fine align for precision
		CADR	IMUFINE		; Position to exact gimbal angles
		TC	BANKCALL	; Wait for stabilization again
		CADR	IMUSTALL	; (critical for measurement accuracy)
		TC	CURTAINS	; Failure exit if problems
		
; Initialize PIPA measurement system
		CA	T/2SEC		; Load 0.5 second time constant
		TS	GCTR		; Store in gravity counter
		CA	PRIO31		; High priority for time-critical
		TS	1/PIPADT	; Set PIPA read interval
		TC	BANKCALL	; Initialize IMU compensation
# Page 966
		CADR	GCOMPZER	; Zero compensation accumulators
		TC	PHASCHNG	; Phase change for measurement
		OCT	04024		; Group 4, phase 2

; Begin gravity measurement (PIPA pulse accumulation)
		TC	BANKCALL	; Initialize PIPA readings
		CADR	PIPSRINE	; (first read establishes baseline)
		TC	INTPRET		; Enter interpreter for setup
GREED		EXIT			; Exit to native mode
		CAF	2SECS		; Load 2-second interval
		TC	TWIDDLE		; Set up timed task
		ADRES	GRABGRAV	; Task: read PIPAS after 2 seconds
					; (allows sufficient pulse count)
		TC	ENDOFJOB	; Suspend until GRABGRAV task fires

; Task GRABGRAV fires after 2-second PIPA accumulation period
; Reads PIPA counters and schedules mainline job to process the data
GRABGRAV	TC	IBNKCALL	; Inter-bank call
		CADR	PIPSRINE	; Read PIPA counters into DELV
		CAF	PRIO13		; Priority 13 for mainline
		TC	FINDVAC		; Find vacant core set
		EBANK=	STARAD		; Set EBANK for ADDGRAV
		2CADR	ADDGRAV		; Schedule ADDGRAV job

		TC	TASKOVER	; Task complete

; Job ADDGRAV accumulates PIPA readings and computes final gravity vector
; Multiple readings are accumulated (controlled by GCTR) to improve accuracy
ADDGRAV		TC	BANKCALL	; Process PIPA data
		CADR	1/PIPA		; Convert pulses to velocity units
		INCR	GCTR		; Increment gravity counter
		TC	INTPRET		; Enter interpreter mode
		VLOAD	VAD		; Load delta-V vector
			DELV		; (from PIPA pulses)
			GACC		; Add to accumulated gravity
		STORE	GACC		; Store accumulated total
					; (averages multiple readings)
		SLOAD	BMN		; Load counter, branch if negative
			GCTR		; Check if more readings needed
			GREED		; Loop back for another 2-sec read
					; (continues until GCTR >= 0)
; Sufficient readings accumulated. Compute final unit gravity vector.
		VLOAD	UNIT		; Load accumulated gravity
			GACC		; Normalize to unit vector
		STCALL	STAR		; Store as star vector (gravity ref)
			CDUTRIG		; Compute CDU trig functions
					; (for coordinate transformation)
		CALL			; Transform gravity vector
			*SMNB*		; Stable member to nav base coords
		STORE	GOUT		; Store output gravity vector
		EXIT			; Exit interpreter mode
		TC	PHASCHNG	; Phase change for completion
		OCT	04024		; Group 4, phase 2

QMINEXIT	TC	INTPRET		; Return to caller
		GOTO			; Via saved return address
			QMIN		; (set at LUNG entry)
T/2SEC		DEC	-20
# Page 967
DISGRVER	VN	0604
42DEG		OCT	07357
35DEG		OCT	06211

# Page 968
# NAME -- GYROTRIM
#
# THIS PROGRAM COMPUTES AND SENDS GYRO COMMANDS WHICH CAUSE THE CDUS
# TO ATTAIN A PRESCRIBED SET OF ANGLES.  THIS ROUTINE ASSUMES THE
# VEHICLES ATTITUDE REMAINS STATIONARY DURING ITS OPERATION.
#
# CALL		CALL
#			GYROTRIM
#
# INPUT		THETAD,+1,+2 = DESIRED CDU ANGLES
#		CDUX,CDUY,CDUZ
#
# OUTPUT	GYRO TORQUE PULSES
#
# SUBROUTINES	TRG*NBSM, *NBSM*, CDUTRIG, AXISGEN, CALCGTA, IMUFINE
#		IMPULSE, IMUSTALL
#		_______  ______  ______     *          *   ___
# DEBRIS 	CDUSPOT, SINCDU, COSCDU, STARAD, VAC, XDC, OGC

		COUNT*	$$/P57
		
; After measuring gravity and computing alignment errors, this routine
; calculates the precise gyro torquing pulses needed to correct the IMU
; orientation. It transforms coordinate systems, computes the required axis
; rotations, and applies the correction pulses to the gyros. This is the final
; step in the P57 lunar surface alignment process.

GYROTRIM	STQ	DLOAD		; Save return address
			QMIN		; Store in QMIN
			THETAD		; Load desired gimbal angle
		PDDL	PDDL		; Push to stack, load next angle
			THETAD +2	; Yaw angle
			THETAD +1	; Pitch angle
		VDEF			; Define vector of gimbal angles
		STOVL	CDUSPOT		; Store CDU spot check
			XUNIT		; Load X unit vector
		CALL			; Transform to body coordinates
			TRG*NBSM	; Transpose, gyro to stable member
		STOVL	STARAD		; Store X axis direction
			YUNIT		; Load Y unit vector
		CALL			; Transform Y axis
			*NBSM*		; Nav base to stable member
		STCALL	STARAD +6	; Store Y axis direction
			CDUTRIG		; Compute CDU trig functions
		CALL			; Calculate stable member
			CALCSMSC	; to spacecraft transformation
		VLOAD			; Load X nav base vector
			XNB		; (body X axis)
		STOVL	6D		; Store at 6D location
			YNB		; Load Y nav base vector
		STCALL	12D		; Store at 12D, call next routine
			AXISGEN		; Generate axis transformation
		CALL			; Calculate gyro torquing angles
			CALCGTA		; (computes OGC output)

; Apply computed gyro torquing pulses to correct IMU orientation
JUSTTRIM	EXIT			; Exit interpreter mode
		TC	BANKCALL	; Position gimbals precisely
		CADR	IMUFINE		; Fine align to computed angles
		TC	BANKCALL	; Wait for stabilization
# Page 969
		CADR	IMUSTALL	; (gyros must be settled)
		TC	CURTAINS	; Failure exit if problems
		CA	GYRCDR		; Load gyro command address
		TC	BANKCALL	; Apply torquing pulses
		CADR	IMUPULSE	; (sends pulses to gyros)
					; This physically corrects IMU error
		TC	BANKCALL	; Wait for pulse completion
		CADR	IMUSTALL	; (gyros integrate pulses)
		TC	CURTAINS	; Failure exit if problems
		TCF	QMINEXIT	; Return to caller

GYRCDR		ECADR	OGC		; Address of gyro commands

# Page 970
# ============================================================================
# PERFORM STAR ACQUISITION AND STAR SIGHTINGS
# ============================================================================
; After initial IMU coarse alignment or gravity measurement, this routine
; acquires two star sightings through the Alignment Optical Telescope (AOT)
; to precisely determine the platform orientation. The crew visually acquires
; each star, centers it in the telescope reticle, and marks the sighting.
; The routine processes each mark to extract the star's direction vector in
; stable member coordinates, then compares this with the star's known position
; from the catalog to determine or refine the IMU alignment.

2STARS		CAF	ZERO		# Initialize star indicator
		TCF	+2		# Zero for 1st star
1STAR		CAF	BIT1		# One for 2nd star
		TS	STARIND		# Store which star we're acquiring

		TC	PHASCHNG	# Set up restart protection
		OCT	04024

		TCF	R59		# Go do star acquire and AOTMARK
					; (R59 guides crew to star location,
					; gets mark, computes STARAD vector)

; Return point after star marking completed
R59RET		CA	STARIND		# Back from surface marking
		EXTEND			; Check which star we just marked
		BZF	ASTAR		# First star marked, process it

		; Second star marked - complete the pair
		TC	PHASCHNG	# Update restart protection
		OCT 	04024

		TC	INTPRET		# Enter interpretive mode
		DLOAD	CALL		; Load time of 2nd mark
			TSIGHT		# (mark timestamp from R59)
			PLANET		; Get star's catalog position
		STCALL	VEC2		# Store 2nd catalog vector (REF)
			SURFLINE	; Proceed to alignment computation

; Process first star marking
ASTAR		TC	INTPRET		# Enter interpretive mode
		VLOAD			; Load first observed star direction
			STARAD +6	; (from AOT measurement)
		STORE	STARSAV1	# Store 1st observed star (SM coords)
		DLOAD	CALL		; Load time of 1st mark
			TSIGHT		# (mark timestamp from R59)
			PLANET		; Get star's catalog position
		STORE	VEC1		# Store 1st catalog vector (REF)
		EXIT			; Back to basic instructions
		TCF	1STAR		# Go get 2nd star sighting

# Page 971
# ============================================================================
# DO FINE OR COARSE ALIGNMENT OF IMU
# ============================================================================
; With two star sightings acquired, this routine computes the IMU alignment.
; It transforms the catalog star positions from the reference frame (inertial)
; to the desired stable member frame, then compares these with the actual
; observed star directions. The difference indicates the alignment error.
; If error is small (< 5 degrees), fine alignment via gyro torquing is used.
; If error is large, coarse alignment through gimbal repositioning is needed.

SURFLINE	SSP	AXT,2		; Set S2 = 6 (for loop control)
			S2		; Index register 2 = 12
			6		; (process both stars in loop)
			12D
WRTDESIR	VLOAD*	MXV		; Load catalog star vector (REF frame)
			VEC1 +12D,2	# Pick up VEC in REF, transform to
			XSMD		# desired stable member frame
		UNIT			; Normalize to unit vector
		STORE	STARAD +12D,2	# Store vector in SM coordinates
		VLOAD*			; Load observed star direction
			STARSAV1 +12D,2	# Pick up vector in present SM
		STORE	18D,2		# Store for comparison
		TIX,2	BON		; Loop for both stars
			WRTDESIR	; Continue loop
			INITALGN	# If initial pass (OPTION 0)
			INITBY		# bypass R54 data check

; Check quality of star sighting data
DOALIGN		CALL
			R54		# Do CHKSDATA (verify star angles)
		BOFF			; Check FREEFLAG (crew approval)
			FREEFLAG	; If not set, crew rejected data
			P57POST		# Astronaut does not like data, recycle
; Compute alignment correction angles
INITBY		CALL
			AXISGEN		# Get desired orientation wrt present
					; Computes XDC, YDC, ZDC axes
		CALL
			CALCGTA		# Get gyro torque angles
					; Computes OGC, IGC, MGC (outer, inner,
					; middle gimbal correction angles)
		EXIT			; Return to basic instructions
		
		; Display correction angles to crew (unless initial alignment)
		CAF	INITABIT	# If initial pass, bypass noun 93 display
		MASK	FLAGWRD8	; Check INITALGN flag
		CCS	A		; Is flag set?
		TCF	5DEGTEST	# Yes, skip display
		
		; Display gyro torquing angles for crew verification
		CAF	DISPGYRO	# Display gyro torque angles V 06 N93
		TC	BANKCALL	# (shows OGC, IGC, MGC to crew)
		CADR	GOFLASH		; Flash display, await crew response
		TC	GOTOPOOH	# V34 -- Terminate (crew abort)
		TCF	5DEGTEST	# V33 -- Proceed to coarse or fine align
		TCF	P57POST +1	# V32 -- Recycle, maybe re-align

; Determine if coarse or fine alignment needed based on angle magnitude
5DEGTEST	TC	INTPRET		# If angles > 5 degrees, do coarse align
		VLOAD	BOV		; Load outer gimbal correction angle
			OGC		; (OGC, IGC, MGC vector)
			SURFSUP		; Handle overflow
SURFSUP		STORE	OGCT		; Store temporarily
		V/SC	BOV		; Divide by 5 degrees
			5DEGREES	; (check if correction < 5 deg)
			COATRIM		; Large error: go to coarse alignment
		SSP	GOTO		; Small error: set QMIN and
			QMIN		; go to fine alignment
			SURFDISP
# Page 972
			JUSTTRIM	# ANGLES LESS THAN 5 DEG, DO GYRO TORQ

; ============================================================================
; FINE ALIGNMENT PATH - Update REFSMMAT and complete alignment
; ============================================================================
; The alignment error is small enough for fine alignment via gyro torquing.
; SURFDISP updates REFSMMAT to reflect the newly computed orientation, then
; proceeds to gyro torquing. For P57 option 2 (landing site alignment),
; additional processing calculates the LM's precise position on the lunar
; surface using the measured gravity vector.

SURFDISP	EXIT			; Return to basic instructions
		TC	PHASCHNG	; Set up restart protection
		OCT	04024

		TC	INTPRET		; Enter interpretive mode
		AXC,1	AXC,2		; Set up matrix move parameters
			XSMD		; Source: newly computed orientation
			REFSMMAT	; Destination: REFSMMAT
		SET	CALL		; Set REFSMMAT valid flag
			REFSMFLG	; (indicates REFSMMAT updated)
			MATMOVE		; Copy XSMD to REFSMMAT
		EXIT			; Back to basic instructions

		; Check alignment option to determine next action
		CCS	OPTION2		# IF OPTION ZERO DO FINISH
		TCF	B2F8		; Non-zero option
		TCF	P57POST +1	; Option 0, go to post-processing

; Handle non-zero alignment options (option 1 or 2)
B2F8		CAF	INITABIT	# IF INITIAL FLAG SET, RE-CYCLE.
		MASK	FLAGWRD8	; Check INITALGN flag status
		CCS	A		; Is it set?
		TCF	P57JUMP		# IT'S SET

		; Get current attitude vectors in mission frame
		TC	INTPRET		; Enter interpretive mode
		CALL
			REFMF		# GO GET ATTITUDE VEC IN MF(YNBSAV,XNBSAV)

; Display alignment result to crew for verification
P57POST		EXIT			; Back to basic instructions
		CAF	OCT14		# DISPLAY V50N25 CHK CODE 14
		TC	BANKCALL	; (alignment complete, quality check)
		CADR	GOPERF1		; Flash display, await crew response
		TCF	GOTOPOOH	# VB34 -- TERMINATE
		TCF	P57JUMP		# VB33 -- PROCEED TO RE-ALIGN

		; Check if option 2 (landing site calculation) is active
		CS	BIT2		# TEST TO SEE IF ALIGNED BY OPTION 2
		AD 	OPTION2		; Compare OPTION2 value with 2
		EXTEND
		BZF	+2		# YES -- GO CALCULATE LANDING SITE
		TCF	GOTOPOOH	# NO -- EXIT P57

; ============================================================================
; LANDING SITE POSITION CALCULATION (Option 2 only)
; ============================================================================
; Using the measured gravity vector and current LM position, compute the
; precise landing site coordinates on the lunar surface. The gravity vector
; points toward the Moon's center, allowing determination of the local
; vertical. Combined with position data, this yields accurate latitude and
; longitude for surface navigation. This was used during Apollo 11 to
; establish Eagle's precise location in the Sea of Tranquility after landing.

		TC	PHASCHNG	# RESTART PLACE
		OCT	04024

		TC	INTPRET		; Enter interpretive mode
		VLOAD	CALL		# USE GNB
			GSAV		; (measured during P57 alignment)
			CDU*NBSM	# GO TO SM COORDS
		VXM	SET		#	ON MOON SO SET LUNAFLAG
			REFSMMAT	#	G(REF) = (REFSMMAT)T (NBSM)GNB
			LUNAFLAG	; Set flag (on Moon, not Earth)
		PDVL	ABVAL		; Push to stack, load RLS vector
			RLS		; (landing site position vector)
		VXSC	STADR		; Scale by RLS magnitude
		STORE	ALPHAV		# ALPHAV = RLSMAG * G(REF)
					; (unit vector from Moon center to surface)
		CLEAR	RTB		; Clear Earth radius flag
# Page 973
			ERADFLAG	; (use lunar radius for calculations)
			LOADTIME	; Load current mission time
		CALL
			N89DISP		# SUBROUTINE TO CALC LS AND GIVE RLS BACK
					; Returns RLS in inertial coordinates
		STORE	RN		# RN=RLS B-29 = LM POSITION
		VSL2	PDDL		# R-TO-RP GETS RLS B-27 AT 0-50 IN PDLIST
			GDT/2 +4	# TIME TEMP STORED IN N89DISP
		PUSH			# TIME AT 6-7 IN PDLIST
		STCALL	PIPTIME		# PIPTIME = LM STATE TIME
			R-TO-RP		; Convert to moon-fixed coordinates
					; (accounts for lunar rotation)
		STORE	RLS		# RLS IN MOON-FIXED COORDS
					; Now available for surface navigation
		EXIT			; Back to basic instructions
		TCF	GOTOPOOH	# EXIT P57

# Page 974
# COARSE AND FINE ALIGN IMU

; ============================================================================
; COATRIM - Coarse Alignment and Gyro Trim Path
; ============================================================================
; This routine is executed when computed alignment errors exceed 5 degrees.
; Coarse alignment physically drives the IMU gimbals to new angles, then
; performs fine gyro torquing. This path is used for large initial alignment
; errors or when the IMU has been significantly disturbed since last alignment.
;
; COMMENT-ONLY READERS: When IMU error is too large for simple gyro torquing,
; the platform must be physically rotated to a new orientation. This takes
; longer but handles large misalignments.
;
; CODE-ALONG READERS: COATRIM computes new desired gimbal angles, displays
; them to the crew if initial alignment flag is set, drives gimbals via
; COARSE routine, then performs fine torquing via GYROTRIM.

COATRIM		AXC,1	AXC,2		; Set up matrix parameters
			XDC		; Source: desired orientation (XDC)
			XSM		; Destination: temp storage (XSM)
		CALL
			MATMOVE		; Copy desired orientation matrix
		CALL
			CDUTRIG		; Compute CDU trig functions
		CALL
			CALCSMSC	; Calculate stable member to spacecraft
		CALL
			CALCGA		; Calculate gimbal angles for new orientation
		BOFF	EXIT		; Check initial alignment flag
			INITALGN	# IF INITIAL ALIGNMENT DISPLAY FINAL
			CORSIT		# GIMBAL ANGLES IF COARSE ANGLES GREATER
		; Display computed gimbal angles to crew for initial alignment
		CAF	V06N22		# THAN 5 DEGREES
					; V06N22: Display OGC, IGC, MGC
					; (computed gimbal angles before coarse)
		TC	BANKCALL
		CADR	GOFLASH		; Flash display, await crew response
		TC	GOTOPOOH	; V34 -- Terminate
		TCF	+2		; V33 -- Proceed
		TCF	-5		; V32 -- Recycle display

		TC	PHASCHNG	; Restart protection
		OCT	04024

		TC	INTPRET		; Enter interpretive mode

; Perform coarse alignment and fine gyro torquing
CORSIT		CALL
			COARSE		; Drive gimbals to new angles
		CALL
			NCOARSE		; Update navigation base after coarse
		CALL
			GYROTRIM	; Perform fine gyro torquing
		GOTO
			SURFDISP	; Update REFSMMAT and complete
DISPGYRO	VN	0693		; Display verb-noun for gyro data

# Page 975
# LUNAR SURFACE IMU ALIGNMENT PROGRAM

; ============================================================================
; P57 - LUNAR SURFACE IMU ALIGNMENT PROGRAM
; ============================================================================
; This program performs IMU alignment while landed on the lunar surface.
; Unlike orbital alignments which use star sightings alone, P57 can use the
; measured gravity vector to establish the local vertical. This provides
; highly accurate determination of the LM's orientation relative to the Moon.
;
; P57 was used during Apollo 11 after landing in the Sea of Tranquility to
; align the IMU for the subsequent ascent. Accurate alignment was critical
; because navigation errors would compound during the ascent to rendezvous
; with the Command Module in lunar orbit.
;
; P57 ALIGNMENT OPTIONS:
; Option 0: Landing site orientation (nominal)
; Option 1: Preferred orientation (pre-computed for specific maneuver)
; Option 2: Invalid in P57 (used in other programs)
; Option 3: Current REFSMMAT orientation (maintain existing)
;
; COMMENT-ONLY READERS: P57 aligns the guidance platform while on the Moon's
; surface using gravity measurements and star sightings. This ensures the
; guidance system knows exactly which way the spacecraft is pointing before
; launching back to orbit.
;
; CODE-ALONG READERS: P57 displays alignment option selection (V04N06), then
; branches to appropriate orientation computation based on selected option.
; Flag checks determine whether stored attitude is valid or must be recomputed.

P57		TC	BANKCALL	# IS ISS ON -- IF NOT, IMUCHK WILL SEND
		CADR	IMUCHK		# ALARM CODE 210 AND EXIT VIA GOTOPOOH
				; Check IMU power status before proceeding

		CAF	THREE		# JAM REFSMMAT OPTION 3 FOR INITIAL DISP.
		TS	OPTION2		; (default to current REFSMMAT)

; Display option selection to crew
P57OPT		CAF	BIT1		; Set up display request
		TC	BANKCALL
		CADR	GOPERF4R	# FLASH V04N06 FOR ALIGNMENT CODE
				; V04N06: Load alignment option into R1
		TC	GOTOPOOH	# V34 TERMINATE
		TCF	ALIGNOPT	# V33 PROCEED
		TCF	P57OPT		# V32 RECYCLE

		TC	PHASCHNG	; Restart protection
		OCT	00014
		TC	ENDOFJOB	; Job complete, await crew entry

; Branch based on selected alignment option
ALIGNOPT	CA	OPTION2		; Load crew-selected option
		MASK	THREE		; Isolate option bits
		INDEX	A		; Indexed branch on option value
		TCF	+1		; Fall through to computed TCF
		TCF	TDISP		# OPTION 4 LS ORIENTATION
				; (landing site, compute from TIG)
		TCF	PACKOPTN	# OPTION 1 PREFERRED
				; (use pre-stored preferred orientation)
		TCF	P57OPT		# OPTION 2 INVALID IN P57, RECYCLE
				; (option 2 not supported, re-prompt)
		TC	INTPRET		# OPTION 3 REFSMMAT
				; (use current REFSMMAT as-is)
		AXC,1	AXC,2		# JAM REFSMMAT IN XSMD LOC
			REFSMMAT	; Source: current REFSMMAT
			XSMD		; Destination: desired orientation
		CALL
			MATMOVE		; Copy REFSMMAT to XSMD
		GOTO
			PACKOPTN -1	; Skip to option display

; Display and input alignment time for option processing
; This section handles time input for alignments that require temporal orientation
; (e.g., landing site orientation at specific TIG, or preferred orientation at maneuver time)
TDISP		TC	INTPRET		; Enter interpretive mode
		DLOAD			; Load alignment time
			TIG		# LOAD ASCENT TIME FOR DISPLAY
					; TIG: time of ignition or alignment epoch
P57A		STORE	DSPTEM1		; Store for display
		EXIT			; Return to basic instructions

; Display current alignment time to crew and allow modification
P57AA		CAF	V06N34*		# DISPLAY TALIGN, TALIGN : DSPTEM1
					; V06N34*: Display and allow load of time
		TC	BANKCALL
		CADR	GOFLASH		; Flash display, await crew response
		TCF	GOTOPOOH	# V34 -- TERMINATE
		TCF 	+2		; V33 -- PROCEED with crew input
		TCF	P57AA		# VB32 -- RECYCLE

		; Process crew time input and compute orientation
		TC	INTPRET		; Re-enter interpretive mode
		RTB	PDDL		; Load current time to stack
			LOADTIME	# PUSH CURRENT TIME AND PICK UP KEY IN
			DSPTEM1		; Load crew-entered time
# Page 976
		BZE	PDDL		; If zero, use current time
			P57C		# IF KEY IN TIME ZERO - TALIGN=CURRENT TIME
		DSU	BPL		# NOT ZERO SO EXCHANGE PD WITH DSPTEM1
			DSPTEM1		; Compute time difference
			P57C		; Branch if future time
		DLOAD	STADR		# IF KEYIN TIME GREATER THAN CURRENT TIME
		STORE	TIG		# STORE IT IN TIG
					; Update TIG with crew input
		STCALL	TALIGN		; Store alignment time
			P57D		; Proceed to orientation computation
P57C		DLOAD	STADR		; Load time from stack
		STORE	TALIGN		; Store as alignment time
					; (using current time if crew entered zero)
P57D		STCALL	TDEC1		; Store time for precision computation
			LEMPREC		# COMPUTE DESIRED IMU ORIENTATION STORE
					; Compute LM state at alignment time
		VLOAD	UNIT		# IN X,Y,ZSMD
			RATT		; Load position vector at TALIGN
					; RATT: position in reference frame
		STCALL	XSMD		; Store unit position as XSMD X-axis
			LSORIENT	; Compute landing site orientation
					; Establishes Y and Z axes for surface frame
		EXIT			; Return to basic instructions
; Pack alignment option flags for display to crew
; Prepares flag bit configuration showing IMU state and alignment options
PACKOPTN	CAF	ZERO		# PACK FLAG BITS FOR OPTION DISPLAY
		TS	OPTION1 +1	# JAM ZERO IN ALIGNMENT OPTION
					; Clear option display word
		TS	OPTION1 +2	# INITIALIZE FLAG BIT CONFIGURATION
					; Clear flag bit accumulator

		; Check REFSMMAT flag (indicates whether REFSMMAT is valid)
		CAF	REFSMBIT	; Load REFSMMAT flag bit mask
		MASK	FLAGWRD3	# REFSMFLG
					; Test if REFSMMAT has been established
		CCS	A		; Check if flag set
		CAF	BIT7		# SET
					; REFSMMAT valid: set bit 7 in display
		ADS	OPTION1 +2	# CLEAR -- JUST ZERO
					; Add to option display word

		; Check attitude flag (indicates whether vehicle attitude is known)
		CAF	ATTFLBIT	; Load attitude flag bit mask
		MASK	FLAGWRD6	# ATTFLG
					; Test if attitude has been stored
		CCS	A		; Check if flag set
		CAF	BIT4		# SET
					; Attitude known: set bit 4 in display
		ADS	OPTION1 +2	# CLEAR -- ZERO IN A
					; Add to option display word

		CAF	BIT4		; Load option code indicator
		TS	OPTION1		# JAM 00010 IN OPTION1 FOR CHECK LIST
					; Set option display format code

; Display alignment options and flags to crew
; V05N06 shows option code and current flag configuration
; Allows crew to verify alignment preconditions before proceeding
DSPOPTN		CAF	VB05N06		# DISPLAY OPTION CODE AND FLAG BITS
					; V05N06: Display R1=option, R2=flags
		TC	BANKCALL
		CADR	GOFLASH		; Flash display, await crew response
		TCF	GOTOPOOH	# VB34 -- TERMINATE
		TCF	+2		# V33 -- PROCEED
					; Crew confirms option selection
		TCF	DSPOPTN		# V32 -- RECYCLE

		; Verify alignment preconditions based on flags
		; REFSMFLG and ATTFLG must be consistent with selected option

		CAF	REFSMBIT	; Check REFSMMAT flag
		MASK	FLAGWRD3	; Load REFSMFLG from flag word 3
		CCS	A		; Test flag state
		TCF	GETLMATT	# SET, GO COMPUTE LM ATTITUDE
					; REFSMMAT valid: compute current attitude

		CAF	ATTFLBIT	# CLEAR -- CHECK ATTFLAG FOR STORED ATTITUDE.
					; REFSMMAT not valid, check attitude flag
		MASK	FLAGWRD6	; Load ATTFLG from flag word 6
		CCS	A		; Test attitude flag
		TCF	BYLMATT		# ALLFLG SET, CHK OPTION FOR GRAVITY COMP
					; Attitude stored: bypass attitude computation

		CAF	BIT2		# SEE IF OPTION 2 OR 3
					; Check if option requires stored attitude
# Page 977
		MASK	OPTION2		; Mask selected option
		CCS	A		; Test if option 2 or 3
		TCF	BYLMATT		# OPTION 2 OR 3 BUT DON'T HAVE ATTITUDE
					; Options 2/3 can proceed without current attitude

		; Alignment preconditions not met - issue alarm
		TC	ALARM		# OPTION INCONSISTENT WITH FLAGS -- ALARM 701
		OCT	701		; Alarm 701: Invalid alignment option/flag combination
					; Crew must either change option or perform required setup
		CAF	VB05N09		# DISPLAY ALARM FOR ACTION
					; V05N09: Display alarm code
		TC	BANKCALL
		CADR	GOFLASH		; Flash alarm, await crew action
		TCF	GOTOPOOH	# VB34 -- TERMINATE
					; Crew terminates alignment attempt
		TCF	DSPOPTN		# V33 -- PROCEED *********TEMPORARY
					; Allow proceed despite alarm (temporary provision)
		TCF	DSPOPTN		# VB32 -- RECYCLE TO OPTION DISPLAY V 05N06
					; Return to option display for new selection

# Page 978
# TRANSFORM VEC1,2 FROM MOON FIXED TO REF AND JAM BACK IN VEC1,2

; ============================================================================
; MFREF - Moon-Fixed to Reference Frame Transformation
;
; Transforms two alignment reference vectors (VEC1, VEC2) from Moon-fixed
; coordinates to inertial reference frame coordinates. Essential for P57
; lunar surface alignment where reference vectors are naturally defined
; in selenographic (Moon-fixed) coordinates but must be compared with
; inertially-referenced star observations.
;
; The Moon-fixed frame rotates with the lunar surface (one rotation per
; 27.3 days). The reference (inertial) frame is fixed relative to stars.
; Transformation accounts for lunar rotation at current time TSIGHT.
;
; INPUTS:
;   VEC1 = First reference vector in Moon-fixed frame (unit vector)
;   VEC2 = Second reference vector in Moon-fixed frame (unit vector)
;   Current AGC time used to compute lunar rotation angle
;
; OUTPUTS:
;   VEC1 = First reference vector in inertial reference frame
;   VEC2 = Second reference vector in inertial reference frame
;
; SUBROUTINES CALLED:
;   LOADTIME - Gets current AGC time
;   RP-TO-R - Rotating Planet to Reference coordinate transformation
;             Accounts for lunar rotation about polar axis
;
; USED BY: P57 Options 0, 1, 3 when alignment uses lunar surface geometry
; ============================================================================

MFREF		STQ	SETPD		; Store return address, initialize push-down list
			QMAJ		; QMAJ = return address for later
			0		; Set push-down pointer to stack base
		RTB			; Return to Basic (load subroutine)
			LOADTIME	; Get current AGC time
		STOVL	TSIGHT		; Store as sight time, load first vector
					; TSIGHT = transformation epoch
			VEC1		; First reference vector (Moon-fixed frame)
		PDDL	PUSH		; Push vector to stack, load time
			TSIGHT		; Sight time for transformation
		CALL			; Call transformation routine
			RP-TO-R		; Rotating Planet to Reference
					; Transforms VEC1 from Moon-fixed to inertial
					; Accounts for lunar rotation since epoch
		STOVL	VEC1		; Store transformed first vector, load second
					; VEC1 now in inertial reference frame
			VEC2		; Second reference vector (Moon-fixed frame)
		SETPD	PDDL		; Reset push-down pointer, push time
			0		; Stack base
			TSIGHT		; Same sight time for consistency
		PUSH	CALL		; Push time, call transformation
			RP-TO-R		; Transform VEC2 from Moon-fixed to inertial
					; Both vectors now in common inertial frame
		STCALL	VEC2		; Store transformed second vector, return
			QMAJ		; Return to stored address
					; VEC1, VEC2 ready for comparison with star vectors

# Page 979
# COMPUTE LM ATTITUDE IN MOON FIXED COORDINATES USING REFSMMAT AND
# STORE IN YNBSAV AND ZNBSAV.

; ============================================================================
; REFMF - Reference to Moon-Fixed Frame Transformation
;
; Computes current Lunar Module body axes (Y and Z) in Moon-fixed
; coordinates for P57 Option 3 alignment. Essential for landing site
; alignments where LM orientation must be known relative to lunar surface
; features. Combines IMU gimbal angles with REFSMMAT to determine attitude
; in inertial frame, then transforms to selenographic (Moon-fixed) frame.
;
; The transformation chain is:
; 1. Read CDU gimbal angles (stable member orientation)
; 2. Compute YNB, ZNB in stable member coordinates (CALCSMSC)
; 3. Transform to inertial reference frame using REFSMMAT
; 4. Transform from reference to Moon-fixed using R-TO-RP
;    Accounts for lunar rotation at current time TSIGHT
;
; Result stored in YNBSAV and ZNBSAV provides LM orientation relative to
; lunar surface, needed for landmark sighting geometry calculations and
; landing site alignment verification.
;
; INPUTS:
;   CDUs = IMU gimbal angles (read from hardware)
;   REFSMMAT = Reference to stable member transformation matrix
;              Defines IMU platform orientation in inertial frame
;   Current AGC time (from LOADTIME)
;
; OUTPUTS:
;   YNBSAV = LM Y-axis in Moon-fixed coordinates (unit vector)
;            Lateral axis relative to lunar surface
;   ZNBSAV = LM Z-axis in Moon-fixed coordinates (unit vector)
;            Vertical axis pointing toward/away from lunar surface
;   ATTFLAG = Set to indicate saved attitude data is valid
;   TSIGHT = Time at which attitude was computed
;
; SUBROUTINES CALLED:
;   CDUTRIG - Computes sine and cosine of CDU angles
;   LOADTIME - Gets current AGC time
;   CALCSMSC - Calculates YNB, ZNB from gimbal angles
;   R-TO-RP - Reference to Rotating Planet (Moon) transformation
;             Accounts for lunar rotation since epoch
;
; USED BY: P57 Option 3 (landing site alignment), GETLMATT entry point
;
; HISTORICAL NOTE: This routine enabled lunar surface landmark alignment
; during Apollo missions. For Apollo 11, provided LM orientation data for
; landing site identification and post-landing attitude verification.
; ============================================================================

REFMF		STQ	CALL		; Store return address, get gimbal trig values
			QMAJ		; QMAJ = return address for final exit
			CDUTRIG		# GET SIN AND COS OF CDUS
					; CDUTRIG reads IMU gimbal angles and
					; computes sine/cosine for attitude calc
		RTB	SETPD		; Return to basic, initialize push-down
			LOADTIME	; Get current AGC time for transformation
			0		; Set push-down pointer to stack base
		STCALL	TSIGHT		; Store as sight time, call attitude calc
					; TSIGHT = epoch for Moon rotation state
			CALCSMSC	# GET YNB IN SM
					; Compute Y and Z navigation base vectors
					; in stable member coordinates from gimbals
		VLOAD	VXM		; Load Y-axis vector, transform to ref frame
			YNB		; Y navigation base in stable member coords
			REFSMMAT	# YNB TO REF
					; Transform from stable member to inertial
					; reference frame using REFSMMAT matrix
		UNIT	PDDL		; Normalize to unit vector, push to stack
					; Unit vector ensures consistent scaling
			TSIGHT		; Load sight time for Moon transformation
		PUSH	CALL		; Push time onto stack, call transform
			R-TO-RP		; Reference to Rotating Planet (Moon)
					; Accounts for lunar rotation at TSIGHT
					; Transforms YNB from inertial to Moon-fixed
		STOVL	YNBSAV		# YNB TO MF
					; Store Y-axis in Moon-fixed frame
					; YNBSAV defines LM lateral orientation
					; relative to lunar surface features
			ZNB		; Load Z navigation base vector
		VXM	UNIT		; Transform to reference frame, normalize
			REFSMMAT	# ZNB TO REF
					; Z-axis from stable member to inertial
					; Z-axis nominally points toward lunar surface
		PDDL	PUSH		; Push vector to stack, load time
			TSIGHT		; Same sight time for consistency
					; All transformations at same instant
		CALL			; Call Moon transformation
			R-TO-RP		# ZNB TO MF
					; Transform Z-axis from inertial to
					; Moon-fixed frame
		STORE	ZNBSAV		; Store Z-axis in Moon-fixed frame
					; YNBSAV and ZNBSAV define complete LM
					; orientation relative to lunar surface
		SETGO			; Set flag and return to caller
			ATTFLAG		; Set ATTFLAG to indicate attitude valid
					; Flag tells other routines YNBSAV/ZNBSAV
					; contain current, usable data
			QMAJ		; Return to stored address
					; YNBSAV, ZNBSAV ready for landmark sighting

# Page 980
; ============================================================================
; TRANSITION: From option display to alignment processing
;
; After crew confirms alignment option and flags, program computes current
; LM attitude (if needed) and branches to option-specific alignment logic.
; Each option uses different reference vectors and star sighting strategies.
; ============================================================================

# BRANCH TO ALIGNMENT OPTION

; Compute current LM attitude for alignment
; Transforms stored attitude vectors from reference to middle frame
GETLMATT	TC	INTPRET		; Enter interpretive mode
		CALL
			REFMF		# GO TRANSFORM TO MF IN YNBSAV, ZNBSAV
					; REFMF: transforms reference frame vectors to middle frame
					; Results stored in YNBSAV, ZNBSAV for comparison
		EXIT			; Return to basic instructions

; Bypass attitude computation - use stored attitude
; Sets initial alignment flag and branches based on option
BYLMATT		TC	UPFLAG		# SET INITIAL ALIGN FLAG
		ADRES	INITALGN	; INITALGN flag indicates alignment in progress
					; Protects alignment state during restarts

		CAF	BIT1		; Load option test mask
		MASK	OPTION2		# SEE IF OPTION 1 OR 3
					; Options 1 and 3 require gravity vector
		CCS	A		; Test option bit
		TCF	GVDETER		# OPTION 1 OR 2, GET GRAVITY
					; Compute gravity for landing site orientation

; Check attitude flag for option 0 processing
ATTCHK		TC	PHASCHNG	; Restart protection
		OCT	04024		; Phase change code

		CAF	ATTFLBIT	# NOT 1 OR 3, CHECK ATTFLAG
					; Verify stored attitude is available
		MASK	FLAGWRD6	; Load attitude flag from flag word 6
		CCS	A		; Test attitude flag state
		TCF	P57OPT0		# GET ALIGNMENT VECS FOR OPTION 0
					; Attitude stored: use for option 0 alignment

; Option jump table - dispatch to selected alignment option
P57JUMP		TC	PHASCHNG	; Restart protection
		OCT	04024		; Phase change code

		TC	DOWNFLAG	# ATTFLG CLEAR -- RESET INTALIGN FLAG
		ADRES	INITALGN	; Clear initial alignment flag
					; Attitude not valid for this path

		CAF	THREE		; Load index mask for 4 options
		MASK	OPTION2		# BRANCH ON OPTION CODE
					; Extract 2-bit option code (0-3)
		INDEX	A		; Indexed transfer based on option
		TCF	+1		; Jump table base
		TCF	P57OPT0		# OPTION IS 0
					; Preferred orientation (pre-computed)
		TCF	P57OPT1		# OPTION IS 1
					; Landing site orientation
		TCF	P57OPT2		# OPTION IS 2
					; Two-star sighting alignment
		TCF	P57OPT3		# OPTION IS 3
					; Landing site plus one star

# Page 981
# OPTION 0, GET TWO ATTITUDE VECS

; ============================================================================
; ALIGNMENT OPTION 0: Preferred Orientation
;
; Uses pre-stored attitude vectors (YNBSAV, ZNBSAV) computed by earlier
; program as reference for alignment. This is the "preferred" orientation
; optimized for a planned maneuver. Transforms these vectors to reference
; frame and compares against current spacecraft axes from star sightings.
; ============================================================================

P57OPT0		TC	INTPRET		; Enter interpretive mode
		VLOAD			; Load vector into MPAC
			YNBSAV		# Y AND Z ATTITUDE WILL BE PUT IN REF
					; Y-axis vector from stored attitude
		STOVL	VEC1		; Store as first reference vector
			ZNBSAV		; Load Z-axis vector from stored attitude
		STCALL	VEC2		; Store as second reference vector, call CDUTRIG
			CDUTRIG		; Compute CDU sines and cosines for transformations

		CALL
			CALCSMSC	# COMPUTE SC AXIS WRT PRESENT SM
					; Calculates current spacecraft axes in stable member frame
					; Results: YNB and ZNB in present SM coordinates

		VLOAD			; Load computed Y-axis
			YNB		; Y spacecraft axis in present stable member frame
SAMETYP		STOVL	STARSAV1	# Y SC AXIS WRT PRESENT SM
					; Store as first observed vector (equivalent to star 1)
			ZNB		; Load Z spacecraft axis
		STCALL	STARSAV2	# Z SC AXIS WRT PRESENT SM
					; Store as second observed vector (equivalent to star 2)
			MFREF		# TRANSFORM VEC1,2 FROM MF TO REF
					; Transform reference vectors from middle to reference frame

		GOTO
			SURFLINE	; Continue to torquing angle computation

# OPTION 1, GET LANDING SITE AND Z-ATTITUDE VEC

; ============================================================================
; ALIGNMENT OPTION 1: Landing Site Orientation
;
; Aligns IMU using landing site position vector (from gravity computation)
; and Z-axis attitude vector. Commonly used during lunar landing preparation
; to orient platform for descent guidance. Combines known landing site
; location with stored attitude for two-vector alignment.
; ============================================================================

P57OPT1		TC	INTPRET		; Enter interpretive mode
		VLOAD	UNIT		; Load and normalize landing site vector
			RLS		# LANDING SITE VEC
					; Landing site position in reference frame (computed earlier)
		STOVL	VEC1		; Store as first reference vector (X-axis toward site)
			ZNBSAV		# Z ATTITUDE VEC
					; Stored Z-axis attitude vector
		STCALL	VEC2		; Store as second reference vector, call CDUTRIG
			CDUTRIG		; Compute CDU transformation matrices

		CALL
			CALCSMSC	# GET ZNB AXIS WRT PRES SM FOR STARSAV2
					; Compute spacecraft Z-axis in present stable member frame

		VLOAD	CALL		; Load gravity vector
			GSAV		# TRANS GSAV FROM NB TO SM FOR STARSAV1
					; Gravity vector in navigation base frame
			CDU*NBSM	; Transform from navigation base to stable member
					; Result: gravity direction in SM (toward lunar center)
		GOTO
			SAMETYP		# NOW DO SAME AS OPTION 0
					; Store results and transform to reference frame

# Page 982
# OPTION 2, GET TWO STAR SIGHTINGS

; ============================================================================
; ALIGNMENT OPTION 2: Two-Star Alignment
;
; Traditional IMU alignment using two star sightings through alignment
; optical telescope (AOT). Crew manually sights two catalog stars and
; marks their positions. Program compares sighted directions with known
; star positions to compute IMU orientation error. Most accurate alignment
; method when stars are available.
; ============================================================================

P57OPT2		TCF	2STARS		# DO SIGHTING ON 2 STARS
					; Branch to two-star sighting routine
					; Crew will sight and mark two stars from catalog

# OPTION 3, GET LANDING SITE VEC AND ONE STAR SIGHTING

; ============================================================================
; ALIGNMENT OPTION 3: Landing Site Plus One Star
;
; Hybrid alignment using landing site position vector and one star sighting.
; Useful when only one star is visible or available for sighting. Combines
; computed landing site direction (gravity-based) with single optical star
; measurement for two-vector alignment. Less accurate than two-star method
; but more accurate than landing site alone.
; ============================================================================

P57OPT3		TC	INTPRET		; Enter interpretive mode
		VLOAD	UNIT		; Load and normalize landing site vector
			RLS		# LANDING SITE VEC
					; Landing site position from gravity computation
		STORE	VEC1		; Store as first reference vector
		STOVL	VEC2		# DUMMY VEC2 FOR 2ND CATALOG STAR
					; Store placeholder for second reference (from star catalog)
			GSAV		# GRAVITY VEC NB
					; Gravity vector in navigation base frame
		CALL
			CDU*NBSM	# TRANS GSAV FROM NB TO SM FOR STARSAV1
					; Transform gravity to stable member frame
		STCALL	STARSAV1	; Store as first observed vector (gravity direction)
			MFREF		# STARSAV2 IS STORED AS 2ND OBSERVED STAR
					; Transform reference vectors to reference frame
		EXIT			; Return to basic instructions
		TCF	1STAR		# 1STAR GET VEC2, STARSAV2 GOES TO SURFLINE.
					; Branch to one-star sighting routine
					; Crew will sight and mark one catalog star

VB05N06		VN	506		; Verb-noun code for display

# Page 983
# CHECK IMODES30 TO VERIFY IMU IS ON

; ============================================================================
; IMU STATUS VERIFICATION
;
; Verifies IMU power is on before proceeding with alignment operations.
; Checks IMODES30 register BIT9 which indicates IMU operational status.
; If IMU is off, issues program alarm 210 and terminates alignment attempt.
; If IMU is on, sets IMUSE flag to indicate IMU is being actively used.
; ============================================================================

IMUCHK		CS	IMODES30	; Load complement of IMU mode register
					; IMODES30 contains IMU operational status
		MASK	BIT9		; Isolate BIT9 (IMU power on indicator)
					; BIT9 = 1 when IMU is powered on
		CCS	A		# IS IMU ON
					; Test masked bit
		TCF	+4		# YES
					; IMU is on - continue to flag setup

		TC	ALARM		# NO, SEND ALARM AND EXIT
					; IMU is off - cannot perform alignment
		OCT	210		; Program alarm 210: IMU not operating
					; Displayed to crew on DSKY
		TC	GOTOPOOH	; Terminate program and return to POO
					; Cannot proceed without operational IMU

		TC	UPFLAG		; Set IMUSE flag
		ADRES	IMUSE		# SET IMUSE FLAG
					; IMUSE indicates IMU actively used by program
					; Protects IMU during alignment operations

		TC	SWRETURN	; Return to caller via switch return
					; IMU verified and ready for alignment

		BANK	04
		SETLOC	AOTMARK2
		BANK
		COUNT*	$$/P57

; ============================================================================
; LANDING SITE ORIENTATION COMPUTATION
;
; Computes nominal orientation matrix for surface operations using current
; CSM position and velocity vectors. Constructs coordinate frame with:
;   XSMD = Pre-defined X-axis (stored)
;   ZSMD = Unit(R × (V × XSMD)) = Normal to orbit plane containing XSMD
;   YSMD = ZSMD × XSMD = Completes right-hand triad
;
; This creates stable member orientation aligned with orbit geometry,
; suitable for lunar surface operations where local vertical reference
; is desired. Used when SURFFBIT indicates spacecraft is on surface.
; ============================================================================

LSORIENT	STQ	VLOAD		; Store return address, load vector
			QMAJ	; QMAJ = return address for major mode
			RRECTCSM	; Load CSM position vector (inertial frame)
					; RRECTCSM = radius vector from lunar center

		VXV	VXV		; Vector cross product operations
			VRECTCSM	; First: R × V (angular momentum direction)
					; VRECTCSM = velocity vector (inertial frame)
			XSMD		; Second: (R × V) × XSMD
					; Projects XSMD into plane perpendicular to R × V

		UNIT			; Normalize to unit vector
					; Creates perpendicular to orbit plane
		STORE	ZSMD		; Store as Z-axis of stable member desired
					; ZSMD defines "up" direction for surface ops

		VXV	UNIT		; Vector cross product and normalize
			XSMD		; ZSMD × XSMD = Y-axis direction
					; Completes right-hand coordinate system
		STCALL	YSMD		; Store Y-axis, then return
			QMAJ		; Return to address stored in QMAJ
					; Orientation matrix (XSMD, YSMD, ZSMD) now defined

