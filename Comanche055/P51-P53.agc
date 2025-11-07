; ============================================================================
; FILE: P51-P53.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: all-phases (navigation alignment)
;
; TL;DR: IMU (Inertial Measurement Unit) alignment programs using star sighting
;        procedures. P51 implements manual optical alignment, P52 provides
;        automatic star tracking, P53 offers backup alignment mode. Essential
;        for maintaining accurate inertial platform orientation throughout
;        mission, enabling precise navigation for all Apollo 11 maneuvers.
;
; COMMENT-ONLY READERS: These programs aligned the spacecraft's navigation
;        system by sighting on stars, like sailors navigating by the stars.
;        Critical for ensuring Columbia knew exactly where it was pointed
;        during translunar coast, lunar orbit, and return to Earth.
; CODE-ALONG READERS: Study IMU platform alignment mathematics, star catalog
;        usage from STAR_TABLES.agc, REFSMMAT computation, coarse and fine
;        alignment algorithms, and alignment quality assessment procedures.
; ============================================================================
;
# Copyright:    Public domain.
# Filename:     P51-P53.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 737-784
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-12 RSB	Adapted from Colossus249 file of the same
#				name, and Comanche 055 page images.
#		2009-05-20 RSB	Corrections: SETI/PDT -> SET1/PDT,
#				GOTOPOOH -> GOTOPOOH, R33EXIT -> R53EXIT,
#				V853 -> VB53, R56A -> R56A1 (some places
#				only), added missing R56A1 label, added a
#				missing CAF in COARSTYP, corrected a SETLOC
#				from P50S to P50S3.
# 		2009-05-21 RSB	In COARFINE, a TC BANKCALL was corrected to
#				TC PHASCHNG.  In R53C, a CADR GOFLASHR was
#				corrected to CADR GOFLASH.
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#       Assemble revision 055 of AGC program Comanche by NASA
#       2021113-051.  April 1, 1969.
#
#       This AGC program shall also be referred to as Colossus 2A
#
#       Prepared by
#                       Massachusetts Institute of Technology
#                       75 Cambridge Parkway
#                       Cambridge, Massachusetts
#
#       under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; TRANSITION: IMU Alignment Program Overview
;
; The Inertial Measurement Unit (IMU) is the spacecraft's primary navigation
; sensor, containing gyroscopes and accelerometers that track orientation and
; velocity. Over time, gyroscopes drift slightly from their calibrated position.
; These programs (P51, P52, P53) realign the IMU platform by sighting on known
; stars, similar to how ancient mariners used celestial navigation. During
; Apollo 11's journey, Collins performed these alignments multiple times to
; ensure Columbia's guidance system maintained precise knowledge of spacecraft
; orientation for critical maneuvers.
; ============================================================================
;
# Page 737
# PROGRAM NAME -- PROG52			DATE -- NOV 30, 1966
# MOD NO -- 2					LOG SECTION -- P51-P53
# MODIFICATION BY -- LONSKE			ASSEMBLY -- SUNDISK REV 30
#
# FUNCTIONAL DESCRIPTION --
#
#	ALIGNS THE IMU TO ONE OF THREE ORIENTATIONS SELECTED BY THE ASTRONAUT.  THE PRESENT IMU ORIENTATION IS KNOWN
#	AND IS STORED IN REFSMMAT.  THE THREE POSSIBLE ORIENTATIONS MAY BE:
#
;
; THREE IMU ORIENTATION OPTIONS:
;
;	(A)	PREFERRED ORIENTATION
;		Used when a specific maneuver requires an optimal IMU orientation.
;		For example, before a major engine burn, the guidance program
;		calculates the best platform alignment to minimize gimbal angles
;		during the burn. This orientation is pre-computed by mission
;		programs like P40 (SPS burn targeting).
;
#	(A)	PREFERRED ORIENTATION
#
#		AN OPTIMUM ORIENTATION FOR A PREVIOUSLY CALCULATED MANEUVER.  THIS ORIENTATION MUST BE CALCULATED AND
#		STORED BY A PREVIOUSLY SELECTED PROGRAM.
#
;	(B)	NOMINAL ORIENTATION
;		A mathematically-defined orientation based on current position and
;		velocity. The coordinate system is defined such that:
;		- Z-axis points toward Earth/Moon center (local vertical)
;		- Y-axis perpendicular to orbital plane (velocity x radius)
;		- X-axis completes right-handed coordinate system
;		This "local vertical, local horizontal" frame is intuitive for
;		orbital operations and was commonly used during Apollo 11's
;		translunar and trans-earth coast phases.
;
#	(B)	NOMINAL ORIENTATION
#
#		X   = UNIT ( Y   x Z   )
#		-SM          -SM   -SM
#
#		Y   = UNIT (V X R)
#		-SM         -   -
#
#		Z   = UNIT ( -R )
#		-SM           -
#
#		WHERE:
#
#		R = THE GEOMETRIC RADIUS VECTOR AT TIME T(ALIGN) SELECTED BY THE ASTRONAUT
#		-
#
#		V = THE INERTIAL VELOCITY VECTOR AT TIME T(ALIGN) SELECTED BY THE ASTRONAUT
#		-
#
;	(C)	REFSMMAT ORIENTATION (Realignment)
;		Corrects accumulated gyro drift since last alignment without
;		changing the reference coordinate frame. Used when the current
;		REFSMMAT (REFerence to Stable Member MATrix) is still valid but
;		gyros have drifted slightly. This is the quickest alignment mode
;		because it doesn't require computing a new reference frame.
;		Collins used this mode for periodic drift corrections during
;		long coast phases when no gimbal lock or power interruption
;		had occurred.
;
#	(C)	RERSMMAT ORIENTATION
#
#		THIS SELECTION CORRECTS THE PRESENT IMU ORIENTATION.  THE PRESENT ORIENTATION DIFFERS FROM THAT TO WHICH IT
#		WAS LAST ALIGNED ONLY DUE TO GYRO DRIVE (I.E., NEITHER GIMBAL LOCK NOR IMU POWER INTERRUPT HAS OCCURRED
#		SINCE THE LAST ALIGNMENT).
#
;
; ALIGNMENT PROCEDURE SEQUENCE:
;
; 1. ORIENTATION SELECTION - Astronaut selects one of three orientation modes
;    via DSKY (Display and Keyboard) entry. Program computes target orientation.
;
; 2. COARSE ALIGNMENT - Routine S52.2 calculates gimbal angles for new
;    orientation. CAL53A commands the IMU gimbals to rotate to approximately
;    correct angles. This gets the platform "in the ballpark" (within a few
;    degrees). During Apollo 11, you could hear the gimbal motors whirring as
;    the platform rotated inside its stabilized mounting.
;
; 3. STAR SELECTION - Routine R56 searches star catalog (STAR_TABLES.agc)
;    for two suitable stars visible in sextant field of view. Stars must be
;    well-separated (good geometry) and not occulted by Earth/Moon/Sun.
;    If no suitable star pair found, program displays alarm and waits for
;    astronaut to maneuver spacecraft or manually select stars.
;
; 4. FINE ALIGNMENT - After two stars acquired, routine R51 uses precise
;    star sightings to calculate remaining orientation errors and corrects
;    them. This achieves alignment accuracy of approximately 1 arc-minute
;    (1/60 of a degree), sufficient for Apollo 11's navigation requirements.
;
; 5. PROGRAM COMPLETION - Returns to P00 (idle) or P20 (rendezvous navigation)
;    depending on mission phase. IMU now accurately aligned for next maneuver.
;
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
#	THE PROGRAM IS CALLED BY THE ASTRONAUT BY DSKY ENTRY.
# Page 738
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

; ============================================================================
; TRANSITION: From Program Documentation to Executable Code
;
; The following sections contain the actual AGC assembly code for IMU
; alignment. The program begins with PROG52 (P52), the main automatic
; alignment routine. Astronauts initiated this by entering V37E52E on the
; DSKY (Verb 37, Program 52). Michael Collins performed these alignments
; regularly during Apollo 11's journey to ensure Columbia's guidance system
; remained accurately calibrated for critical navigation and maneuvers.
; ============================================================================

P54		=	PROG52
		BANK	33
		SETLOC	P50S
		BANK

		SBANK=	LOWSUPER
		EBANK=	SAC
		COUNT	15/P52

;
; PROG52 ENTRY POINT - Automatic IMU Alignment Program
;
; The astronaut has just entered P52 via DSKY. The program begins by
; clearing tracking flags and checking IMU status to ensure the platform
; is ready for alignment. During Apollo 11, this check verified the IMU
; was in coarse align mode and not experiencing any gimbal lock conditions.
;
PROG52		TC	PHASCHNG
		OCT	00254
		TC	DOWNFLAG
		ADRES	UPDATFLG	# BIT 7 FLAG 1
# Page 739
;
; Clear navigation tracking flag to prevent interference during alignment.
;
		TC	DOWNFLAG
		ADRES	TRACKFLG	# BIT 5 FLAG 1
;
; IMU Status Check - Verify platform is operational and not in gimbal lock.
; R02BOTH routine (defined elsewhere) checks IMU health and mode status.
; If IMU is unusable, program terminates with appropriate alarm.
;
		TC	BANKCALL
		CADR	R02BOTH		# IMU STATUS CHECK
;
; Check PFRATFLG (Preferred Attitude Flag) to determine default options.
; This flag indicates whether a preferred orientation has been pre-calculated
; by another program (like P40 for an upcoming SPS burn).
;
		CAF	BIT4
		MASK	STATE +2	# IS PFRATFLG SET?
		CCS	A
		TC	P52A		# YES
		CAF	BIT2		# NO
		TC	P52A +1
P52A		CAF	BIT1
		TS	OPTION2
;
; Display alignment options to astronaut on DSKY.
; GOPERF4R flashes display showing:
; - Option Code: Current alignment mode selection
; - Orientation Code: Desired reference frame orientation
; Astronaut can accept defaults (ENTER) or modify (keypad entry).
; During Apollo 11, Collins would review these options before proceeding.
;
P52B		CAF	BIT1
		TC	BANKCALL	# FLASH OPTION CODE AND ORIENTATION CODE
		CADR	GOPERF4R
		TC	GOTOPOOH
		TC	+5
		TC	P52B		# NEW CODE -- NEW ORIENTATION CODE INPUT
;
; Job phasing and task management for restart protection.
;
		TC	PHASCHNG
		OCT	00014
		TC	ENDOFJOB
;
; ORIENTATION MODE SELECTION - Branch based on astronaut's choice:
;
; The OPTION2 variable (bits 0-1) encodes the orientation mode:
;   00 = Landing Site (L.S.) - Special mode for lunar landing preparation
;   01 = Preferred (PREF) - Use pre-calculated optimal orientation
;   10 = Nominal (NORM) - Calculate local vertical/local horizontal frame
;   11 = REFSMMAT (REF) - Realign to existing reference frame
;
; This indexed branch efficiently routes to the appropriate computation
; routine without multiple conditional tests.
;
		CA	OPTION2
		MASK	THREE
		INDEX	A
		TC	+1
		TC	P52T		# L.S.
		TC	P52J		# PREF
		TC	P52T		# NORM
		TCF	P52C		# REF
;
; ============================================================================
; TRANSITION: Time Input for Nominal and Landing Site Orientations
;
; For Nominal and Landing Site orientations, the computer needs to know when
; to compute the orientation. This section prompts the astronaut to enter
; the time for alignment (T-ALIGN). If the astronaut enters zero or presses
; ENTER without input, the current time (TIME2) is used instead. This allows
; immediate alignment or scheduling for a future maneuver.
; ============================================================================
;
P52T		EXTEND
		DCA	NEG0
		DXCH	DSPTEM1
;
; Display V06N34 - Time input request on DSKY
; Format: Hours:Minutes:Seconds from mission start
;
		CAF	V06N34
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	+2
		TC	-5
;
; Check if astronaut entered a specific time or pressed ENTER (zero input)
;
		EXTEND
		DCA	DSPTEM1
		EXTEND
		BZF	+2		# Zero entered?
		TCF	+4		# No, use astronaut's time

;
; Astronaut pressed ENTER without time input - use current mission time.
; TIME2 is the AGC's main mission elapsed time counter.
;
		EXTEND
		DCA	TIME2
		DXCH	DSPTEM1
;
; Determine whether this is Landing Site (LS) or Nominal (NOM) orientation.
; BIT2 of OPTION2 distinguishes between these two modes.
;
		CA	OPTION2
		MASK	BIT2
		CCS	A
# Page 740
		TCF	+6		# NOM
;
; LANDING SITE ORIENTATION MODE
; Computes orientation relative to intended lunar landing site.
; P52LS subroutine (defined later in this file) calculates the coordinate
; transformation from landing site coordinates to inertial reference frame.
;
		TC	INTPRET		# LS
		CALL
			P52LS
		GOTO
			P52D
;
; NOMINAL ORIENTATION MODE
; Computes local vertical/local horizontal reference frame at specified time.
; X-axis: Cross product of velocity and position (orbit normal direction)
; Y-axis: Cross product of velocity and radius (horizontal, along track)
; Z-axis: Nadir pointing (toward Earth center or Moon center)
;
		TC	INTPRET
		DLOAD
			DSPTEM1
		CALL			# COMPUTE NOMINAL IMU
			S52.3		#	ORIENTATION
;
; COMMON PATH: Compute Desired Gimbal Angles
; S52.2 routine takes the computed REFSMMAT orientation and current vehicle
; attitude, then calculates the three gimbal angles (outer, inner, middle)
; needed to achieve that orientation. These angles will drive the IMU
; platform motors during coarse alignment.
;
P52D		CALL			# READ VEHICLE ATTITUDE AND
			S52.2		#	COMPUTE GIMBAL ANGLES
		EXIT
;
; Display computed gimbal angles to astronaut on DSKY using V06N22.
; Astronaut reviews these angles before proceeding with physical alignment.
; Format: OUTER GIMBAL, INNER GIMBAL, MIDDLE GIMBAL (degrees).
; During Apollo 11, Collins would verify these looked reasonable before
; commanding the IMU to actually move to these angles.
;
		CAF	VB06N22
		TC	BANKCALL	# DISPLAY GIMBAL ANGLES
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	COARSTYP
;
; PREFERRED ORIENTATION MODE (PREF)
; Uses REFSMMAT already computed by another program. Vehicle attitude is
; read and gimbal angles computed for the existing preferred orientation.
;
P52J		TC	INTPRET		# RECYCLE: VEHICLE HAS BEEN MANEUVERED
		GOTO
			P52D
;
; ============================================================================
; TRANSITION: Coarse Alignment Phase
;
; With desired gimbal angles computed and displayed, the IMU now physically
; rotates its platform. Coarse alignment drives the platform motors to the
; computed angles with moderate precision (~1 degree accuracy). This provides
; the initial rough orientation before fine alignment using star sightings.
; ============================================================================
;
		TC	INTPRET
		CALL			# DO COARSE ALIGN
			CAL53A		#	ROUTINE
;
; Set REFSMFLG to indicate REFSMMAT is now valid and IMU is aligned to it.
;
CAL53RET	SET	EXIT
			REFSMFLG
;
; REFSMMAT CORRECTION MODE ENTRY POINT
; Astronaut selected option 3 (REF) to correct the current IMU orientation
; without computing a new reference frame. This mode assumes gyro drift only,
; no gimbal lock or power interruption since last alignment.
;
P52C		TC	PHASCHNG
		OCT	04024
;
; Display ALARM 15 and wait for astronaut response.
; This alarm alerts the crew that automatic star selection will follow.
; Astronaut can:
;   - Press V33 (PROCEED) to use automatic star selection
;   - Press ENTER to proceed to fine alignment with current stars
;
		CAF	ALRM15
		TC	BANKCALL
		CADR	GOPERF1
		TC	GOTOPOOH
		TC	+2		# V33
		TC	P52F		# E
;
; Load current time plus TSIGHT1 offset for star visibility computation.
; LOCSAM routine computes spacecraft position and attitude at sighting time.
;
		TC	INTPRET
		RTB	DAD
			LOADTIME
			TSIGHT1
		CALL
			LOCSAM
		EXIT
;
; AUTOMATIC STAR SELECTION
; PICAPAR routine selects the two best stars for fine alignment based on:
; - Star brightness and catalog availability (STAR_TABLES.agc)
; - Current spacecraft attitude and attitude constraints
; - Optical system limitations (sextant field of view)
; - Geometric dilution of precision (star separation angle)
;
P52E		TC	BANKCALL	# DO STAR SELECTION
		CADR	PICAPAR
		TC	P52I		# 2 STARS NOT AVAILABLE
;
; ============================================================================
; TRANSITION: Fine Alignment Phase
;
; Two suitable stars have been selected (either automatically or manually).
; The program now executes R51, the fine alignment routine. R51 prompts
; the astronaut to sight each star through the Command Module's sextant,
; marking when the star is centered in the field of view. The computer
; uses these precise angular measurements to calculate small corrections
; to the IMU platform orientation, achieving alignment accuracy better
; than 0.01 degrees. This precision was essential for Apollo 11's accurate
; navigation throughout the mission.
; ============================================================================
;
P52F		TC	INTPRET		# 2 STARS AVAILABLE
		CALL
			R51
ENDP50S		EXIT
		TC	GOTOPOOH
# Page 741
;
; STAR UNAVAILABLE ALARM
; Automatic star selection failed to find two suitable stars for fine alignment.
; This occurs when spacecraft attitude restricts star visibility or stars are
; too close together for good geometric accuracy. Astronaut options:
;   - PROCEED: Accept situation and proceed with R51 fine alignment anyway
;   - RECYCLE: Maneuver spacecraft to expose different stars, retry selection
;
P52I		TC	ALARM
		OCT 405
;
; Display ALARM 405 using V05N09. This flashes both the PROG and OPER ERR lights
; on the DSKY, alerting the astronaut that star selection encountered difficulty.
;
		CAF	V05N09
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	P52F		# PROCEED:  DO FINE ALIGN-R51
		TC	P52C		# RECYCLE:  VEHICLE HAS BEEN MANEUVERED
;
; Display codes defined for P51-P53 programs
;
V06N34		VN	0634
VB06N22		VN	00622
;
; COARSE ALIGNMENT TYPE SELECTION
; After coarse alignment completes, astronaut selects alignment refinement type:
;   - V34 (TERMINATE): End program, accept current coarse alignment only
;   - PROCEED: Normal fine alignment using optical star sightings
;   - V32 (RECYCLE): Use gyro torquing commands (gyro coarse mode)
;
; Gyro coarse mode applies direct torquing pulses to IMU gyroscopes rather than
; using star sightings. Used when optical alignment unavailable (stars occluded,
; optics failure, time constraints) but better accuracy than coarse align needed.
;
COARSTYP	CAF	OCT13
		TC	BANKCALL
		CADR	GOPERF1
		TCF	GOTOPOOH	# V34
		TCF	P52J	+3	#	NORMAL
;
; GYRO COARSE ALIGNMENT MODE
; Computes IMU torquing angles to achieve desired orientation without star
; sightings. Transforms stable member desired coordinates (XSMD, YSMD, ZSMD)
; through REFSMMAT to obtain desired orientation in inertial coordinates.
; Then issues gyro torquing commands to rotate IMU platform accordingly.
;
		TC	INTPRET		# GYRO COARSE
;
; Transform desired stable member X-axis through REFSMMAT to inertial frame,
; normalize to unit vector, and store in XDC (desired coordinates X-axis).
;
GYCRS		VLOAD	MXV
			XSMD
			REFSMMAT
		UNIT
		STOVL	XDC
;
; Transform desired stable member Y-axis through REFSMMAT to inertial frame,
; normalize to unit vector, and store in YDC (desired coordinates Y-axis).
;
			YSMD
		MXV	UNIT
			REFSMMAT
		STOVL	YDC
;
; Transform desired stable member Z-axis through REFSMMAT to inertial frame,
; normalize to unit vector, and store in ZDC (desired coordinates Z-axis).
;
			ZSMD
		MXV	UNIT
			REFSMMAT
		STCALL	ZDC
;
; CALCGTA routine computes gyro torquing angles needed to rotate IMU from
; current orientation to desired orientation (XDC, YDC, ZDC).
;
			CALCGTA
;
; Clear drift compensation and REFSMMAT valid flags before issuing gyro pulses.
; DRIFTFLG cleared: gyro drift compensation suspended during torquing.
; REFSMFLG cleared: REFSMMAT temporarily invalid during platform rotation.
;
		CLEAR	CLEAR
			DRIFTFLG
			REFSMFLG
		EXIT
;
; Display gyro torquing angles to astronaut using V16N20 before execution.
; Shows computed torquing pulses for outer, inner, middle gyros (gyro pulses,
; not degrees). Astronaut verifies and approves before IMU receives commands.
;
		CAF	V16N20
		TC	BANKCALL
		CADR	GODSPR
;
; Issue gyro torquing pulses to IMU. R55CDR contains the computed pulse counts
; for each of the three gyroscopes. IMUPULSE routine sends these commands to
; IMU hardware, physically rotating the stable member platform.
;
		CA	R55CDR
		TC	BANKCALL
		CADR	IMUPULSE
;
; Wait for IMU motors to complete gyro torquing commands. IMUSTALL monitors
; IMU status until platform rotation finishes and platform is stable.
;
		TC	BANKCALL
		CADR	IMUSTALL
		TC	CURTAINS
		TC	PHASCHNG
		OCT	04024
;
; Copy stable member desired coordinates (XSMD) to REFSMMAT using matrix move.
; This updates REFSMMAT to reflect the new IMU orientation achieved by gyro
; torquing. AXC,1 and AXC,2 set up index registers for MATMOVE source/dest.
;
		TC	INTPRET
		AXC,1	AXC,2
			XSMD
			REFSMMAT
		CALL
# Page 742
			MATMOVE
;
; Set flags to reflect updated IMU state:
; PFRATFLG cleared: platform not in fine align rotation mode
; REFSMFLG set: REFSMMAT now valid and represents current IMU orientation
; DRIFTFLG set: resume gyro drift compensation with updated orientation
;
		CLEAR	SET
			PFRATFLG
			REFSMFLG
		RTB	VLOAD
			SET1/PDT
			ZEROVEC
		STORE	GCOMP
		SET	GOTO
			DRIFTFLG
			R51K
V16N20		VN	1620
ALRM15		EQUALS	OCT15
		SETLOC	P50S2
		BANK
V06N89*		VN	0689
;
; ============================================================================
; SUBROUTINE: P52LS - LANDING SITE ORIENTATION COMPUTATION
;
; FUNCTION: Computes IMU orientation aligned to lunar landing site coordinates.
;           Displays landing site latitude, longitude, and altitude to astronaut
;           for verification and allows keyboard updates. Creates stable member
;           coordinate frame (XSMD, YSMD, ZSMD) with X-axis pointing at landing
;           site, Z-axis perpendicular to orbital plane, Y-axis completing
;           right-handed triad.
;
; COORDINATE FRAME DEFINITION:
;   XSMD = UNIT(RLS)              - Points at landing site from Moon center
;   YSMD = UNIT(ZSMD × XSMD)      - Completes right-handed orthogonal triad
;   ZSMD = UNIT((R × V) × RLS)    - Normal to orbital plane, projected onto
;                                    plane perpendicular to landing site vector
;
; WHERE:
;   RLS = Landing site position vector (Moon-fixed coordinates)
;   R   = CSM position vector (reference inertial coordinates)
;   V   = CSM velocity vector (reference inertial coordinates)
;
; MISSION CONTEXT:
; This orientation was used during Apollo 11 lunar orbit when the crew needed
; IMU alignment referenced to the Sea of Tranquility landing site. Aligning
; the IMU to the landing site simplified navigation computations during descent
; preparations and enabled the crew to monitor approach trajectory relative to
; the target landing area.
;
; INPUTS:
;   DSPTEM1 = Time of alignment (centi-seconds)
;   RLS     = Landing site vector in Moon-fixed coordinates
;
; OUTPUTS:
;   XSMD, YSMD, ZSMD = Stable member desired coordinates (landing site frame)
;
; SUBROUTINES CALLED:
;   RP-TO-R   = Converts Moon-fixed position to inertial reference frame
;   LAT-LONG  = Computes latitude/longitude from position vector
;   LLASRD    = Prepares lat/long/altitude for display
;   LLASRDA   = Accepts keyboard updates to landing site coordinates
;   LALOTORV  = Converts lat/long/altitude back to position vector
;   CSMPREC   = Computes CSM state vectors (position, velocity, attitude)
; ============================================================================
;
; Save return address in QMAJ, set LUNAFLAG to indicate lunar coordinate
; system (not Earth-centered). This tells coordinate conversion routines
; to use lunar radius and rotation parameters.
;
P52LS		STQ	SET
			QMAJ
			LUNAFLAG
;
; Load alignment time from DSPTEM1 (entered by astronaut earlier in PROG52)
; and store in TSIGHT for use by coordinate transformation routines.
;
		DLOAD
			DSPTEM1
		STORE	TSIGHT
;
; Load landing site vector RLS (Moon-fixed coordinates) into 0D, set ERADFLAG
; to indicate position vector requires planetary radius consideration. Store
; TSIGHT time in 6D as second parameter for RP-TO-R conversion routine.
;
		VLOAD	SET
			RLS
			ERADFLAG
		STODL	0D
			TSIGHT
		STCALL	6D
# Page 743
;
; RP-TO-R COORDINATE TRANSFORMATION
; Converts landing site position from Moon-fixed (rotating) coordinates to
; inertial reference coordinates. Accounts for lunar rotation from epoch time.
; Returns position vector in reference frame at alignment time TSIGHT.
;
			RP-TO-R
;
; Scale result by 2^-2 (divide by 4) to prevent overflow in subsequent
; computations, store in ALPHAV for latitude/longitude conversion.
;
		VSR2
		STODL	ALPHAV
			TSIGHT
;
; Convert landing site position vector (ALPHAV) to latitude and longitude.
; LAT-LONG routine computes geodetic coordinates assuming spherical Moon.
;
		CALL
			LAT-LONG
;
; Prepare latitude, longitude, altitude for display to astronaut. LLASRD
; formats these values for DSKY display (degrees, minutes for lat/long;
; nautical miles for altitude above lunar surface).
;
		CALL
			LLASRD
		EXIT
;
; LANDING SITE DISPLAY AND KEYBOARD UPDATE
; Display current landing site coordinates to astronaut using V06N89:
;   R1 = Latitude (degrees, 5 digits with sign)
;   R2 = Longitude (degrees, 5 digits with sign)  
;   R3 = Altitude (nautical miles above surface)
;
; Astronaut can:
;   - TERMINATE: Abort alignment program, return to P00
;   - PROCEED: Accept displayed values, continue to orientation computation
;   - RECYCLE: Redisplay same values (useful if display was unclear)
;   - ENTER new values: Update landing site coordinates via keyboard
;
LSDISP		CAF	V06N89*
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	+2
		TC	LSDISP
;
; Process keyboard input if astronaut entered new landing site coordinates.
; LLASRDA accepts lat/long/altitude from DSKY and validates values are within
; acceptable ranges for lunar surface positions.
;
		TC	INTPRET
		CALL
			LLASRDA
;
; Convert updated latitude, longitude, altitude back to position vector.
; LALOTORV performs inverse transformation: geodetic coordinates → Cartesian
; position vector in Moon-fixed coordinates. Result stored in updated RLS.
;
		DLOAD	CALL
			TSIGHT
			LALOTORV
;
; STABLE MEMBER COORDINATE FRAME COMPUTATION
;
; Compute XSMD: Normalize landing site position vector to unit vector.
; This becomes the X-axis of the stable member frame, pointing from Moon
; center through landing site into space above the site.
;
		VLOAD	UNIT
			ALPHAV
		STODL	XSMD
;
; Load alignment time into TDEC1 and call CSMPREC to compute CSM state vectors
; (position RATT, velocity VATT, attitude) at the alignment time. These are
; needed to compute the Z-axis of landing site frame.
;
			TSIGHT
		STCALL	TDEC1
			CSMPREC
;
; Compute ZSMD: Cross product of position and velocity (R × V) gives angular
; momentum vector, which is normal to the orbital plane. Cross this with XSMD
; to get a vector in the orbital plane perpendicular to the landing site radial.
; Normalize to unit vector. This becomes the Z-axis of the stable member frame.
;
; Mathematical breakdown:
;   R × V           = Angular momentum vector (perpendicular to orbit plane)
;   (R × V) × XSMD  = Vector in orbit plane, perpendicular to landing site
;   UNIT(...)       = Normalize to unit vector → ZSMD
;
		VLOAD	VXV
			RATT
			VATT
		VXV	UNIT
			XSMD
		STORE	ZSMD
;
; Compute YSMD: Cross product ZSMD × XSMD completes the right-handed orthogonal
; coordinate frame. Normalize to unit vector. This becomes the Y-axis of the
; stable member frame, completing the landing site orientation triad.
;
; Result: Orthogonal coordinate system (XSMD, YSMD, ZSMD) where:
;   - XSMD points at landing site
;   - ZSMD lies in orbital plane, perpendicular to landing site direction
;   - YSMD completes right-handed triad
;
		VXV	UNIT
			XSMD
		STCALL	YSMD
			QMAJ
		SETLOC	P50S1
		BANK

# NAME:		AUTOMATIC OPTICS POSITIONING ROUTINE
#
# FUNCTION:	(1) TO POINT THE STAR LOS OF THE OPTICS AT A STAR OR LANDMARK DEFINED BY THE PROGRAM OR BY DSKY INPUT.
#		(2) TO POINT THE STAR LOS OF THE OPTICS AT THE LEM DURING RENDEZVOUS TRACKING OPERATIONS.
#
# CALLING:	CALL R52
#
# INPUT:	1.  TARG1FLG AND TARG2FLG:  PRESET BY CALLER
#		2.  RNDVZFLG AND TRACKFLG:  PRESET BY CALLER
#		3.  STAR CODE:  PRESET BY CALLER.  ALSO INPUT THROUGH DSKY
#		4.  LAT, LONG, AND ALT OF LANDMARK:  INPUT THROUGH DSKY
# Page 744
#		5.  NO. OF MARKS (MARKINDX):  PRESET BY CALLER
#
# OUTPUT:	DRIVE SHAFT AND TRUNNION CDUS.
#
# SUBROUTINES:	1.  FIXDELAY		7.  CLEANDSP
#		2.  GOPERF1		8.  GODSPR
#		3.  GOFLASH		9.  REFLASHR
#		4.  R53			10. R52.2
#		5.  ALARM		11. R52.3
#		6.  SR52.1

		COUNT	15/R52

R52		STQ	CLEAR
			SAVQR52
			ADVTRK
R52VRB		EXIT
		EXTEND
		DCA	CDUT
		DXCH	DESOPTT
		TC	INTPRET
		SSP	CLEAR
			OPTIND
			0
			R53FLAG
		EXIT
R52A		TC	INTPRET
		SET	BON
			TRUNFLAG
			TARG1FLG
			R52H
		CLEAR	EXIT
			TERMIFLG
R52C		CA	SWSAMPLE	# IS OPTICS MODE IN AGC
		EXTEND
		BZMF	R52M		# MANUAL
R52D		TC	BANKCALL	# AGC
		CADR	SR52.1
		TCF	R52L		# GR 90 DEGREES
		TCF	R52J		# GR 50 DEGREES
		TC	UPFLAG		# LS 50 DEGREES
		ADRES	TRUNFLAG	# SET TRUNFLAG BIT 4 FLAG 0
R52JA		CAF	BIT10		# IS THIS A LEM
		MASK	STATE +1
		CCS	A
		TC	R52E		# YES
		CAF	BIT6		# NO, IS R53FLAG SET
		MASK	STATE
		CCS	A
		TCF	R52E		# YES
# Page 745
		CAF	V06N92		# NO
		TC	BANKCALL
		CADR	GODSPR
R52E		CA	SWSAMPLE	# IS OSS IN CMC MODE
		EXTEND
		BZMF	R52F		# NO
		CS	STATE		# YES: IS TRUNFLAG SET
		MASK	BIT4
		CCS	A
		TC	+3		# NO
		CA	PAC		# YES
		TS	DESOPTT
		CA	SAC
		TS	DESOPTS
R52F		CAF	.5SEC		# WAIT 1/2 SEC
		TC	BANKCALL
		CADR	DELAYJOB
		CAF	BIT10
		MASK	STATE +1
		CCS	A
		TCF	R52HA		# YES, LEM
		CAF	BIT15		# NO
		MASK	STATE +7	# IS TERMIFLG SET
		EXTEND
		BZF	R52C		# NO
R52Q		TC	INTPRET		# YES
		GOTO
			SAVQR52
R52H		EXIT			# LEM
R52HA		TC	BANKCALL
		CADR	R61CSM
		CA	STATE +1
		MASK	BIT5
		EXTEND			# TRACKFLG
		BZF	R52Q

		CS	STATE +1
		MASK	BIT7		# UPDATFLG
		CCS	A
		TCF	R52SYNC

R52I		CA	STATE +5
		MASK	BIT10
		CCS	A
		TC	R52D		# PRFTRKAT = 1
R52SYNC		CAF	1.8SEC		# MAKE UP FOR LOST TIME
		TCF	R52F +1

R52J		TC	DOWNFLAG	# CLEAR TRUNFLAG
		ADRES	TRUNFLAG	# BIT 4 FLAG 0
# Page 746
		TC	ALARM		# SET 407 ALARM
		OCT	407
		TC	R52JA
R52M		CAF	BIT6		# IS R53FLAG SET
		MASK	STATE
		CCS	A
		TC	R52F		# YES
		INHINT			# NO
		CAF	PRIO24
		TC	FINDVAC
		EBANK=	SAC
		2CADR	R53JOB

		RELINT
		TCF	R52F
R53JOB		TC	INTPRET
		CALL
			R53
ENDPLAC		EXIT			# INTERPRETER RETURN TO ENDOFJOB (R22 USES)
		TC	ENDOFJOB
V06N92		VN	00692
V06N89A		VN	0689
SHAXIS		2DEC	.5376381241 B-1

		2DEC	0

		2DEC	.8431766920 B-1

R52L		CAF	BIT10		# IS THIS A LEM
		MASK	STATE +1
		CCS	A
		TC	R52J		# YES
		CAF	OCT404
		TC	BANKCALL
		CADR	PRIOLARM
		TCF	TERM52		# TERMINATE
		TCF	R52F		# PROCEED
		TCF	R52F		# NO PROVISION FOR NEW DATA
		TCF	ENDOFJOB

OCT404		OCT	404
1.8SEC		DEC	180

TERM52		TC	CLEARMRK

		TC	BANKCALL	# KILL MARK SYSTEM
		CADR	MKRELEAS

# Page 747

		CAF	ZERO
		TS	OPTCADR

		TC	BANKCALL	# CLEAR OUT EXTENDED VERBS
		CADR	KLEENEX

		TC	GOTOPOOH	# NO GO TO P00

ADVORB		STQ	SET		# SETS UP ADVANCED ORBIT TRACKING
			SAVQR52
			ADVTRK
		SET	SET
			LUNAFLAG
			ERADFLAG
		GOTO
			R52VRB

# Page 748
# NAME -- S50 ALIAS LOCSAM
# NAME:	LOCSAM
#
# FUNCTION -- TO COMPUTE QUATITIGS LISTED BELOW, USED IN THE
#	      IMU ALIGNMENT PROGRAMS.
#
#	DEFINE:
#
#	RATT = POSITION VECTOR OF CM WRT PRIMARY BODY
#
#	VATT = VELOCITY VECTOR OF CM WRT PRIMARY BODY
#
#	RE = RADIUS OF EARTH
#
#	RM = RADIUS OF MOON
#
#	ECLIPOL = POLE OF ECLIPTIC SCALED BY TANGENTIAL VELOCITY OF EARTH
#		  WRT TO SUN OVER THE VELOCITY OF LIGHT
#
#	REM = POSITION OF MOON WRT EARTH
#
#	RES = POSITION OF SUN WRT EARTH
#
#	C = VELOCITY OF LIGHT
#
#		EARTH IS PRIMARY			MOON IS PRIMARY
#		        _                                       _
#		VEARTH=-1(RATT)				VEARTH=-1(REM+RATT)
#		        _				        _
#		VMOON = 1(REM-RATT)			VMOON =-1(RATT)
#		        _				        _
#		VSUN  = 1(RES)				VSUN  = 1(RES-REM)
#		              -1
#		CEARTH=COS(SIN  (RE/RATT)+5)		CEARTH=COS 5
#								      -1
#		CMOON =COS 5				CMOON =COS(SIN  CRM/RATT)+5)
#
#		CSUN  =COS 15				CSUN  =COS 15
#
#			    VEL/C = VSUN x ECLIPOL + VATT/C
#
# CALL:		DLOAD	CALL
#			DESIRED TIME
#			LOCSAM
#
# INPUTS:  	MPAC = TIME
#
# OUTPUTS:  	VEARTH, VMOON, VSUN, CEARTH, CMOON, CSUN, VEL/C
#
# SUBROUTINES:  LSPOS, CSMCONIC
#
# DEBRIS:  	VAC AREA, SEE SUBROUTINES.

# Page 749
		SETLOC	P50S1
		BANK

		COUNT*	$$/S50

LOCSAM		=	S50
S50		STQ
			QMAJ
		STCALL	TSIGHT
			LSPOS
		STOVL	VMOON
			2D
		STODL	VSUN
			TSIGHT
		STCALL	TDEC1
			CSMCONIC
		SSP	TIX,2
			S2
			0
			MOONCNTR
;
; EARTH-CENTERED COORDINATE SYSTEM BRANCH
; When spacecraft is in Earth's sphere of influence (lunar distance > halfway
; to Moon), compute relative position vectors from Earth as primary reference.
; This branch executes during translunar and transearth coast phases.
;
; Compute Moon's position relative to spacecraft by subtracting spacecraft
; position (RATT) from Moon position vector (VMOON), then normalize to unit.
;
EARTCNTR	VLOAD	VSU
			VMOON
			RATT
		UNIT
		STOVL	VMOON
;
; Normalize spacecraft position vector (RATT in Earth-centered frame) to unit
; vector and complement (negate) to get unit vector from spacecraft toward
; Earth center. This vector determines which stars are occulted by Earth.
;
			RATT
		UNIT	VCOMP
		STODL	VEARTH
;
; Load Earth's radius (6,378,166 meters scaled by 2^-29) and call OCCOS to
; compute occultation cosine for Earth. This determines the angular radius
; of Earth as seen from spacecraft position - critical for determining if
; Earth occludes any star during star sighting procedures.
;
; OCCOS computation: cos(angle) = arcsin(radius / distance) + 5 degrees margin
; The 5-degree margin accounts for atmospheric refraction and penumbra effects.
;
			RSUBE
		CALL
			OCCOS
;
; Store Earth occultation cosine in CEARTH for later star visibility tests.
; Load CSS5 constant (cos(5°)/4 for Moon at Earth distance) and store in
; CMOON - this is the default Moon occultation value when Earth-centered.
;
		STODL	CEARTH
			CSS5
		STOVL	CMOON
;
; Normalize Sun position vector to unit length for direction testing.
; Jump to ENDSAM to compute velocity aberration correction.
;
			VSUN
		UNIT
		STCALL	VSUN
			ENDSAM
;
; MOON-CENTERED COORDINATE SYSTEM BRANCH
; When spacecraft is in Moon's sphere of influence (closer to Moon than
; halfway point), compute relative position vectors from Moon as primary
; reference. This branch executes during lunar orbit and descent/ascent phases.
;
; Scale Moon position vector by 2^-9 (divide by 512) to prevent computational
; overflow when computing Sun direction relative to Moon center. The VMOON
; position vector is in meters scaled by 2^-29 (Earth-Moon distance range).
;
MOONCNTR	VLOAD	VSR8
			VMOON
		VSR1	BVSU
;
; Compute Sun's position relative to Moon center by subtracting Sun position
; (VSUN) from scaled Moon position. Normalize to unit vector for occultation
; calculations. During lunar orbit operations, Sun direction is critical for
; thermal control and lighting conditions during star sightings.
;
			VSUN
		UNIT
		STOVL	VSUN
;
; Compute Earth's position relative to spacecraft by adding Moon position
; (VMOON) to spacecraft position in Moon frame (RATT), normalize to unit,
; and complement to get direction from spacecraft toward Earth center.
; This determines which stars are occulted by Earth as seen from lunar orbit.
;
			VMOON
		VAD	UNIT
			RATT
		VCOMP
		STOVL	VEARTH
# Page 750

;
; Normalize spacecraft position (RATT in Moon-centered frame) to unit vector
; and complement to get direction from spacecraft toward Moon center. This
; vector determines which stars are occulted by Moon during lunar operations.
;
			RATT
		UNIT	VCOMP
		STODL	VMOON
;
; Load Moon's radius (1,738,090 meters scaled by 2^-29) and call OCCOS to
; compute occultation cosine for Moon. This determines the angular radius
; of Moon as seen from spacecraft position in lunar orbit - critical during
; P52 alignment when maneuvering for star sightings above lunar horizon.
;
			RSUBM
		CALL
			OCCOS
;
; Store Moon occultation cosine in CMOON for star visibility tests.
; Load CSS5 constant (cos(5°)/4 for Earth at lunar distance) and store in
; CEARTH - this is the default Earth occultation value when Moon-centered.
;
		STODL	CMOON
			CSS5
		STOVL	CEARTH
			VSUN
;
; ENDSAM - END OF STAR AVAILABILITY MATRIX COMPUTATION
;
; Compute velocity aberration correction due to spacecraft motion relative
; to inertial star field. This correction is essential for accurate star
; sighting - stars appear displaced by up to 20 arcseconds due to spacecraft
; velocity (up to 11 km/s during translunar coast). The correction ensures
; optical alignment telescope (sextant) points at true star direction.
;
; Cross product of Sun position vector (VSUN) with ecliptic pole (ECLIPOL)
; gives velocity direction perpendicular to ecliptic plane for aberration.
;
ENDSAM		VXV
			ECLIPOL
		STOVL	VEL/C
;
; Scale spacecraft velocity vector (VATT) by inverse speed of light (1/C)
; to compute velocity aberration vector. At maximum Apollo velocity of
; 11,000 m/s, aberration angle = v/c ≈ 37 microradians (7.6 arcseconds).
; Add aberration correction to preliminary velocity vector.
;
			VATT
		VXSC	VAD
			1/C
			VEL/C
		STODL	VEL/C
;
; Load Sun occultation cosine constant (CSSUN) and store in CSUN.
; Return to caller (QMAJ) with occultation data and aberration correction.
;
			CSSUN
		STCALL	CSUN
			QMAJ
;
; OCCOS - OCCULTATION COSINE COMPUTATION SUBROUTINE
;
; Computes the cosine of the occultation angle for a celestial body (Earth,
; Moon, or Sun) as seen from spacecraft position. This angle determines the
; angular radius of the body's disk plus a 5-degree safety margin for
; atmospheric effects (Earth) and penumbra (all bodies).
;
; INPUTS:
;   A register: Body radius in meters scaled by 2^-29
;   36D: Spacecraft distance from body center (computed by caller)
;
; OUTPUT:
;   A register: cos(arcsin(radius/distance) + 5°) scaled and right-shifted
;
; Mathematical sequence: arcsin(R/d) gives angular radius of body disk,
; add 5° safety margin, take cosine to get occultation test threshold,
; shift right by 1 to scale for comparison with star dot products.
;
OCCOS		DDV	SR1
			36D
		ASIN	DAD
			5DEGREES
		COS	SR1
		RVQ
		SETLOC	P50S
		BANK
RSUBM		2DEC	1738090 B-29	# MOON RADIUS IN METERS

RSUBE		2DEC	6378166 B-29

5DEGREES	2DEC	.013888889 	# SCALED IN REVS

1/C		2DEC	.000042699 B-1	# *

ECLIPOL		2DEC	0		# *

		2DEC	-.00007896 B-1	# *

		2DEC	.00018209 B-1	# *		* FOR USE BY CSM ONLY

TSIGHT1		2DEC	24000

CEARTH		=	14D
CSUN		=	16D
CMOON		=	18D
CSS5		2DEC	.2490475	# (COS 5)/4
# Page 751
CSSUN		2DEC	.24148		# (COS 15)/4

# Page 752
;
; ============================================================================
; TRANSITION: From star catalog and alignment setup to automatic star selection
;
; After establishing the spacecraft's orientation and computing which celestial
; bodies are visible, the computer now automatically selects the optimal pair
; of stars for IMU alignment. This sophisticated algorithm evaluates every
; possible star pair, checking for occultation by Earth, Sun, or Moon, testing
; angular separation for good geometry, and verifying the stars lie within
; the telescope's field of view. The best pair is then presented to the crew.
; ============================================================================
;
# PROGRAM NAME -- PICAPAR	DATE: DEC 20 66
# MOD 1				LOG SECTION: P51-P53
#				ASSEMBLY:  SUNDISK REV40
# BY KEN VINCENT
#
# FUNCTION
#	THIS PROGRAM READS THE IMU-CDUS AND COMPUTES THE VEHICLE ORIENTATION
#	WITH RESPECT TO INERTIAL SPACE.  IT THEN COMPUTES THE SHAFT AXIS (SAX)
#	WITH RESPECT TO REFERENCE INERTIAL.  EACH STAR IN THE CATALOG IS TESTED
# 	TO DETERMINE IF IT IS OCCULTED BY EITHER EARTH, SUN OR MOON.  IF A
# 	STAR IS NOT OCCULTED THEN IT IS PAIRED WITH ALL STARS OF LOWER INDEX.
# 	THE PAIRED STAR IS TESTED FOR OCCULTATION.  PAIRS OF STARS THAT PASS
#	THE OCCULTATION TESTS ARE TESTED FOR GOOD SEPARATION.  A PAIR OF STARS
#	HAVE GOOD SEPARATION IF THE ANGLE BETWEEN THEM IS LESS THAN 66 DEGREES
#	AND MORE THAN 40 DEGREES.  THOSE PAIRS WITH GOOD SEPARATION
#	ARE THEN TESTED TO SEE IF THEY LIE IN CURRENT FIELD OF VIEW.  (WITHIN
#	33 DEGREES OF SAX).  THE PAIR WITH MAX SEPARATION IS CHOSEN FROM
#	THOSE WITH GOOD SEPARATION, AND IN FIELD OF VIEW.
#
# CALLING SEQUENCE
#	L	TC	BANKCALL
#	L+1	CADR	PICAPAR
#	L+2	ERROR RETURN -- NO STARS IN FIELD OF VIEW
#	L+3	NORMAL RETURN
#
# OUTPUT
#	BESTI, BESTJ -- SINGLE PREC, INTEGERS, STAR NUMBERS TIMES 6
#	VFLAG -- FLAG BIT SET IMPLIES NO STARS IN FIELD OF VIEW
#
# INITIALIZATION
#	1)	A CALL TO LOCSAM MUST BE MADE
#	2)	VEARTH = -UNIT(R) WHERE R HAS BEEN UPDATED TO APPROXIMATE TIME OF
#		SIGHTINGS.
#
# DEBRIS
#	WORK AREA
#	X,Y,ZNB
#	SINCDU, COSCDU
#	STARAD -- STAR +5
;
; COMMENT-ONLY READERS: The computer automatically searches through all 37
; navigation stars to find the best pair for alignment. It checks each star
; to ensure it's not hidden behind Earth, Moon, or Sun, verifies the two
; stars have good angular separation (40-66 degrees apart for best geometry),
; and confirms both lie within the telescope's 33-degree field of view. This
; automation greatly simplified crew workload during Apollo 11's alignment
; procedures, particularly during busy mission phases like translunar coast.
;
; CODE-ALONG READERS: PICAPAR (PIck A PARameter) is the automatic star pair
; selection algorithm. Reads IMU CDU angles to compute spacecraft attitude
; matrix (XNB, YNB, ZNB), transforms shaft axis (SAX) to inertial coordinates,
; then systematically tests all 703 possible star pairs (37 choose 2). For
; each pair: (1) Checks occultation via OCCULT subroutine using CULTRIX
; occulting body cone matrix, (2) Computes angular separation via dot product
; and compares against 40-66 degree window (cos(66°)=0.407, cos(40°)=0.766),
; (3) Tests field-of-view constraint by checking star angles from SAX < 33°
; (cos(33°)=0.843). Selects pair with maximum separation meeting all criteria.
; Returns star indices in BESTI, BESTJ scaled by 6 for CATLOG addressing.
; Sets VFLAG if no valid pairs found. Critical for P52 automatic alignment.
;

		COUNT	14/PICAP

		SETLOC	P50S1
		BANK
PICAPAR		TC	MAKECADR
		TS	QMIN
;
; PICAPAR INITIALIZATION - COMPUTE SHAFT AXIS IN INERTIAL COORDINATES
;
; COMMENT-ONLY READERS: The computer first reads the spacecraft's current
; attitude from the IMU gyroscopes, then calculates which direction the
; telescope is pointing. This "shaft axis" direction tells the computer
; where to look for stars in the celestial sphere.
;
; CODE-ALONG READERS: Initialization sequence. Calls CDUTRIG to read IMU CDU
; angles and compute sine/cosine tables (SINCDU, COSCDU). Calls CALCSMSC to
; compute spacecraft attitude matrix XNB, YNB, ZNB transforming from navigation
; base to stable member coordinates. Sets pushdown pointer to 0. Initializes
; VFLAG=1 (assume no valid stars until proven otherwise). Clears BESTI, BESTJ
; to zero. Computes shaft axis (SAX) in inertial coordinates: SAX = REFSMMAT *
; (XNB*sin(33°) + ZNB*cos(33°)), where 33° is optics shaft angle from Z-axis.
; This transforms telescope boresight from spacecraft body coordinates through
; current REFSMMAT to inertial space. Initializes S1=S2=6 as star index counters
; (each star catalog entry occupies 6 words: 3 for unit vector + magnitude data).
;
		TC	INTPRET
		CALL
			CDUTRIG
		CALL
			CALCSMSC
# Page 753
		SETPD
			0
		SET	DLOAD		# VFLAG = 1
			VFLAG
			DPZERO
		STOVL	BESTI
			XNB
		VXSC	PDVL
			SIN33
			ZNB
		AXT,1	VXSC
			228D		# X1 = 37 X 6 + 6
			COS33
		VAD
		VXM	UNIT
			REFSMMAT
		STORE	SAX		# SAX = SHAFT AXIS
		SSP	SSP		# S1 = S2 = 6
			S1
			6
			S2
			6
;
; MAIN STAR SELECTION LOOP STRUCTURE
;
; COMMENT-ONLY READERS: The computer now systematically tests every possible
; pair of stars from the catalog. It starts with star 37, checks if it's
; visible (not hidden by Earth, Moon, or Sun), then pairs it with stars 36,
; 35, 34... all the way down to star 1. For each pair, the computer checks
; three things: (1) Both stars are visible, (2) They're separated by 40-66
; degrees (good geometry for accurate alignment), (3) Both are within the
; telescope's 33-degree field of view. The pair with the best separation is
; automatically selected for the crew to sight.
;
; CODE-ALONG READERS: Nested loop structure implements exhaustive star pair
; search. Outer loop (PIC1) iterates major star from index 228 down to 6 in
; steps of -6 (37 stars * 6 words/star = 222, plus offset = 228). For each
; major star, tests occultation via OCCULT subroutine. Inner loop (PIC3)
; iterates minor star from X1 down to 6. For each minor star: (1) Tests
; occultation, (2) Computes dot product to check separation angle between
; 66° (cos=0.2419) and 40° (cos=0.7660), (3) Tests both stars within 33°
; cone of SAX (cos(33°)=0.8432). STRATGY routine selects pair with maximum
; separation. Index registers X1, X2 point to CATLOG entries (star unit
; vectors). CULTFLAG indicates occultation status. Returns via QPRET.
;
PIC1		TIX,1	GOTO		# MAJOR STAR
			PIC2
			PICEND
;
; OCCULTATION TEST FOR MAJOR STAR
;
; CODE-ALONG READERS: PIC2 loads major star vector from CATLOG,1 (X1 index),
; calls OCCULT to test if star is behind Earth, Sun, or Moon. If CULTFLAG
; set (star occulted), branches to PIC1 for next major star. If visible,
; loads X2=X1 to begin minor star loop at same index.
;
PIC2		VLOAD*	CALL
			CATLOG,1
			OCCULT
		BON	LXA,2
			CULTFLAG
			PIC1
			X1
;
; MINOR STAR LOOP - PAIR WITH ALL LOWER-INDEX STARS
;
; CODE-ALONG READERS: PIC3 decrements X2 by 6 to next minor star. If X2
; reaches zero, all minor stars tested, return to PIC1 for next major star.
; Otherwise continue to PIC4 to test this star pair.
;
PIC3		TIX,2	GOTO
			PIC4
			PIC1
;
; STAR PAIR QUALITY TESTS
;
; COMMENT-ONLY READERS: For this candidate star pair, the computer first
; checks if the second star is also visible. Then it measures the angle
; between the two stars - they must be between 40 and 66 degrees apart.
; Too close together (less than 40°) gives poor geometry. Too far apart
; (more than 66°) makes it difficult to see both stars in sequence. If
; the pair passes these tests, the computer checks if both stars are within
; the telescope's current 33-degree field of view.
;
; CODE-ALONG READERS: PIC4 tests minor star occultation. If occulted, skip
; to PIC3. If visible, compute separation angle via dot product of unit
; vectors CATLOG,1 · CATLOG,2. Test against CSS66 = cos(66°)/4 = 0.06048
; (scaled for double precision). If separation > 66°, dot product < CSS66,
; branch negative to PIC3. Add CSS6640 = (cos(66°)-cos(40°))/4 = -0.15603.
; If result positive, separation < 40°, skip to PIC3. Only pairs with
; 40° < separation < 66° proceed to field-of-view tests.
;
PIC4		VLOAD*	CALL
			CATLOG,2
			OCCULT
		BON	VLOAD*
			CULTFLAG
			PIC3
			CATLOG,1
		DOT*	DSU
			CATLOG,2
			CSS66		# SEPARATION LESS THAN 66 DEG.
		BMN	DAD
			PIC3
			CSS6640		# SEPARATION MORE THAN 40 DEG.
		BPL
			PIC3
# Page 754
;
; FIELD OF VIEW TESTS - BOTH STARS WITHIN TELESCOPE CONE
;
; COMMENT-ONLY READERS: Now the computer checks if both stars are actually
; within the telescope's field of view at the current spacecraft attitude.
; The telescope can see 33 degrees from its center line (the shaft axis),
; forming a cone-shaped viewing area. Both stars must be inside this cone
; for the astronaut to see them through the optics. If both stars pass
; this test, the computer saves this pair as a candidate and continues
; searching for an even better pair with wider separation.
;
; CODE-ALONG READERS: Test if both stars within 33° cone of shaft axis (SAX).
; Dot product CATLOG,1 · SAX compares major star against cos(33°)/4 = 0.2108
; (CSS33 constant). If dot product < CSS33, angle > 33°, star outside cone,
; branch to PIC1 to try next major star. If major star in cone, test minor
; star CATLOG,2 · SAX against same threshold. If minor star angle > 33°,
; branch to PIC3 for next minor star. If BOTH stars in cone, this is a valid
; pair - proceed to STRATGY to evaluate if better than previous best pair.
; Otherwise return to PIC3 to continue minor star search.
;
		VLOAD*	DOT
			CATLOG,1
			SAX
		DSU	BMN		# MAJOR STAR IN CONE
			CSS33
			PIC1
		VLOAD*	DOT
			CATLOG,2
			SAX
		DSU	BPL
			CSS33
			STRATGY
		GOTO
			PIC3
;
; STRATGY - SELECT BEST STAR PAIR BASED ON MAXIMUM SEPARATION
;
; COMMENT-ONLY READERS: When the computer finds a valid star pair (both
; visible, good separation, within field of view), it compares this pair
; to the previous best candidate. The goal is to find the pair with the
; widest separation angle - better geometry means more accurate alignment.
; If this new pair is better than the old one, the computer saves it and
; keeps searching. When all star pairs have been tested, the computer has
; found the optimal pair for IMU alignment.
;
; CODE-ALONG READERS: STRATGY implements "greedy" selection algorithm for
; maximum separation. VFLAG indicates if any valid pair found yet. On first
; entry (VFLAG set), clear VFLAG and branch to NEWPAR to unconditionally
; accept first valid pair. On subsequent entries, exchange X1↔BESTI and
; X2↔BESTJ to reload previous best pair indices. Compute dot product of
; current pair (CATLOG,1 · CATLOG,2), push to stack. BOFINV means "branch
; on flag inverse" - if VFLAG clear, skip to STRAT-3. Load previous best
; separation from top of stack, subtract from current separation. If
; current separation > previous (subtraction positive), current pair is
; better - fall through to NEWPAR to save it. Otherwise branch to PIC3 to
; continue search with previous best pair still saved.
;
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
;
; NEWPAR - SAVE NEW BEST PAIR
;
; COMMENT-ONLY READERS: When the computer finds a star pair with better
; geometry (wider separation angle) than the previous best, it saves this
; new pair as the best candidate. The search continues to find an even
; better pair if one exists in the star catalog.
;
; CODE-ALONG READERS: Store X1→BESTI (major star index) and X2→BESTJ (minor
; star index). These are catalog indices scaled by 6 (each star occupies 6
; words in CATLOG). After saving the new best pair, GOTO PIC3 to continue
; searching for an even better pair. At end of search, BESTI and BESTJ will
; contain indices of the star pair with maximum separation angle.
;
NEWPAR		SXA,1	SXA,2
			BESTI
			BESTJ
		GOTO
			PIC3
;
; OCCULT - CHECK IF STAR IS OCCULTED BY EARTH OR MOON
;
; COMMENT-ONLY READERS: This subroutine checks if a star is hidden behind
; Earth or the Moon from the spacecraft's current position. If the celestial
; body blocks the line of sight to the star, the star is "occulted" and
; cannot be used for alignment. The computer marks such stars as unavailable
; and skips them in the selection process.
;
; CODE-ALONG READERS: Called as subroutine via TC BANKCALL. Transforms star
; unit vector through CULTRIX matrix (set by caller: EARTHTAB for Earth check,
; MOONTAB for Moon check). MXV rotates star vector to body-centered frame.
; BVSU subtracts CSS (=CEARTH radius in Earth radii, scaled). Result is vector
; from body surface to star. Tests all three components using BMN (branch minus)
; to determine if star direction intersects body disk. If any component test
; fails, branches to CULTED to set CULTFLAG=1 indicating occultation. Uses SIGN
; to test vector component signs. MPAC+3, MPAC+5 are Y and Z components. If all
; tests pass, star is visible - CLRGO clears CULTFLAG and returns via QPRET.
;
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
;
; CULTED - STAR IS OCCULTED
;
; COMMENT-ONLY READERS: If the computer determines a star is hidden behind
; Earth or Moon, execution reaches this label which marks the star as
; occulted and unavailable for alignment.
;
; CODE-ALONG READERS: SETGO sets CULTFLAG=1 indicating star is occulted, then
; returns to caller via QPRET. Caller (PIC2) will test CULTFLAG and skip this
; star if occulted.
;
CULTED		SETGO
# Page 755
			CULTFLAG
			QPRET
;
; ============================================================================
; PICAPAR CONSTANTS - ANGULAR THRESHOLDS AND TRIGONOMETRIC VALUES
; ============================================================================
;
; COMMENT-ONLY READERS: These mathematical constants define the geometry
; requirements for selecting good star pairs. Stars must be separated by
; at least 40 degrees but no more than 66 degrees for optimal alignment
; accuracy. The constants also include values for checking if stars fall
; within the sextant's 33-degree field of view.
;
; CODE-ALONG READERS: Constants in double-precision scaled format (2DEC).
; CSS = CEARTH (Earth radius for occultation check, defined elsewhere).
; SIN33, COS33: Sine and cosine of 33° (half-angle of 66° sextant FOV cone).
; CSS66 = COS(76°)/4: Used in dot product threshold for 66° maximum separation.
; CSS6640 = (COS(76°)-COS(30°))/4: Difference threshold for 40°-66° range test.
; CSS33 = COS(38°)/4: Used for field-of-view cone test (76°/2 = 38°).
; All angle constants scaled for interpretive vector dot product computations.
;
CSS		= 	CEARTH
SIN33		2DEC	.5376381241

COS33		2DEC	.8431756920

CSS66		2DEC	.060480472	# (COS76)/4

CSS6640		2DEC	-.15602587	# (COS76 - COS30)/4

CSS33		2DEC	.197002688	# (COS(1/2(76))/4
;
; ============================================================================
; PICEND - PICAPAR EXIT ROUTINE
; ============================================================================
;
; COMMENT-ONLY READERS: When PICAPAR completes its search for the best star
; pair, it exits through this routine. If automatic star selection succeeded
; (found good pair), the program continues to alignment. If no suitable pair
; was found, the astronaut receives an alarm and must manually select stars.
;
; CODE-ALONG READERS: BOFF tests VFLAG (set if valid pair found). If VFLAG=0
; (no valid pair), branches to PICGXT which increments QMIN alarm counter and
; continues. If VFLAG=1, falls through to PICBXT. Both paths converge at PICBXT
; which loads QMIN and calls SWCALL to return to caller. QMIN indicates whether
; manual star selection is required (QMIN>0 means automatic selection failed).
; EXIT instruction terminates interpretive mode before BOFF test.
;
PICEND		BOFF	EXIT
			VFLAG
			PICGXT
		TC	PICBXT
PICGXT		EXIT
		INCR	QMIN
PICBXT		CA	QMIN
		TC	SWCALL
#V1		= 	12D

# Page 756
# NAME -- R51	FINE ALIGN
# FUNCTION -- TO ALIGN THE STABLE MEMBER TO REFSSMAT
# CALLING SEQ -- CALL R51
# INPUT -- BESTI, BESTJ (PAIR OF STAR NO)
# OUTPUT -- GYRO TORQUE PULSES
# SUBROUTINES -- R52, R54, R55 (SXTNB, NBSM, AXISGEN)

		COUNT	14/R51

R51		EXIT
		CAF	BIT1
		TS	STARIND
		TS	MARKINDX
R51.2		TC	INTPRET
R51.3		CLEAR	CLEAR
			TARG2FLG
			TARG1FLG
		EXIT
		TC	PHASCHNG
		OCT	05024		# RESTART GR 4 FOR R52-R53
		OCT	13000
		INDEX	STARIND
		CA	BESTI
		EXTEND
		MP	1/6TH
		TS	STARCODE
R51DSP		CAF	V01N70
		TC	BANKCALL
		CADR	GOFLASHR
		TC	GOTOPOOH
		TC	+5
		TC	-5
		CAF	SIX
		TC	BLANKET
		TCF	ENDOFJOB
		TC	CHKSCODE
		TC	FALTON
		TC	R51DSP
		TC	INTPRET
		RTB	CALL
			LOADTIME
			PLANET
		SSP	LXA,1
			S1
			0
			STARIND
		TIX,1
			R51ST
		STCALL	STARSAV2	# 2ND STAR
			R51ST +1
R51ST		STORE	STARSAV1	# 1ST STAR
# Page 757
		EXIT
		CS	MODREG		# IS THIS P54
		AD 	OCT66
		EXTEND
		BZF	R51B		# YES
		TC	INTPRET
		CALL
			R52		# AOP WILL MAKE CALLS TO SIGHTING
R51A		CALL			# COMPUTE LOS IN SM FROM MARK DATA
			SXTSM
		STORE	STARSAV2
		EXIT
		TC	BANKCALL
		CADR	MKRELEAS
		TC	INTPRET
		DLOAD	CALL
			TSIGHT
			PLANET
		EXIT
		CCS	STARIND
		TC	R51.4
		TC	INTPRET
		MXV	UNIT
			REFSMMAT
		STORE	STARAD
		VLOAD
			STARSAV2
		STOVL	6D
			STARSAV1
		STOVL	12D
			PLANVEC
		STCALL	STARAD +6
			R54		# STAR DATA TEST
		BOFF	CALL
			FREEFLAG
			R51K
			AXISGEN
		CALL
			R55		# GYRO TORQUE
		CLEAR
			PFRATFLG
R51K		EXIT
		CAF	OCT14
		TC	BANKCALL
		CADR	GOPERF1
		TC	GOTOPOOH
		TC	+2		# V33
		TC	+3
		TC	BANKCALL
		CADR	P52C
# Page 758
		TC	INTPRET
		GOTO
			ENDP50S
R51.4		TC	INTPRET
		MXV	UNIT
			REFSMMAT
		STOVL	PLANVEC
			STARSAV2
		STORE	STARSAV1
		SSP
			STARIND
			0
		GOTO
			R51.3
R51B		TC	INTPRET
		CALL
			R56
		GOTO
			R51A
OCT66		OCT	00066
V01N70		VN	0170
1/6TH		DEC	.1666667

# Page 759
# NAME:		R55	GYRO TORQUE
# FUNCTION -- COMPUTE AND SEND GYRO PULSES
# CALLING SEQ -- CALL R55
# INPUT -- X,Y,ZDC -- REFSMMAT WRT PRESENT STABLE MEMBER
# OUTPUT -- GYRO PULSES
# SUBROUTINES -- CALCGTA, GOFLASH, GODSPR, IMUFINE, IMUPULSE, GOPERF1

		SETLOC	P50S
		BANK
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
		OCT	00314
		CA	R55CDR
		TC	BANKCALL
		CADR	IMUPULSE
		TC	BANKCALL
		CADR	IMUSTALL
		TC	CURTAINS
		TC	PHASCHNG
		OCT	05024
		OCT	13000
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
# Page 760
#	 A2	=	ARCCOS(SS1 - SS2)
#	  A 	=	ABS(2(A1 - A2))

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

		SETLOC	P50S1
		BANK
		COUNT*	$$/R50
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
			THETA		# COMPUTE POS DIFF
			18D
		ABS	RTB
			SGNAGREE
		STORE	NORMTEM1
		SET	EXIT
			FREEFLAG
		CAF	ZERO
		TC	BANKCALL
		CADR	CLEANDSP

		CAF	VB6N5
# Page 761
		TC	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH
		TC	CHKSDA		# PROCEED
		TC	INTPRET
		CLEAR	GOTO
			FREEFLAG
			QMIN
CHKSDA		TC	INTPRET
		GOTO
			QMIN
VB6N5		VN	605

# NAME -- CAL53A
# FUNCTION -- COARSE ALIGN THE IMU, IF NECESSARY
# CALLING SEQUENCE -- CALL CAL53A
# INPUT -- PRESENT GIMBAL ANGLES -- CDUX, CDUY, CDUZ
#	   DESIRED GIMBAL ANGLES -- THETAD,+1,+2
# OUTPUT -- THE IMU COORDINATES AT STORED IN REFSMMAT
# SUBROUTINES -- 1.IMUCOARS, 2.IMUSTALL, 3CURTAINS

		COUNT	14/R50

CAL53A		CALL
			S52.2		# MAKE FINAL COMP OF GIMBAL ANGLES
		RTB	SSP
			RDCDUS		# READ CDUS
			S1
			1
		AXT,1	SETPD
			3
			4
CALOOP		DLOAD*	SR1
			THETAD +3D,1
		PDDL*	SR1
			4,1
		DSU	ABS
		PUSH	DSU
			DEGREE1
		BMN	DLOAD
			CALOOP1
		DSU	BPL
			DEG359
			CALOOP1
COARFINE	EXIT
		TC	PHASCHNG
		OCT	04024
		TC	BANKCALL
		CADR	IMUCOARS	# PERFORM COARSE ALIGNMENT
		TC	BANKCALL
		CADR	IMUSTALL	# REQUEST MODE SWITCH
# Page 762
		TC	CURTAINS
		TC	BANKCALL
		CADR	IMUFIN20
		TC	BANKCALL
		CADR	IMUSTALL
		TC	CURTAINS	# TEST FOR MALFUNCTION
		TC	INTPRET
		RTB	VLOAD
			SET1/PDT
			ZEROVEC
		STORE	GCOMP
		SET	GOTO
			DRIFTFLG
			FINEONLY
CALOOP1		TIX,1
			CALOOP
FINEONLY	AXC,1	AXC,2
			XSM
			REFSMMAT
		CALL
			MATMOVE
		GOTO
			CAL53RET
MATMOVE		VLOAD*			# TRANSFER MATRIX
			0,1
		STORE	0,2
		VLOAD*
			6D,1
		STORE	6D,2
		VLOAD*
			12D,1
		STORE	12D,2
		RVQ
DEGREE1		DEC	46
DEG359		DEC	16338
		SETLOC	P50S
		BANK
RDCDUS		INHINT			# READ CDUS
		CA	CDUX
		INDEX	FIXLOC
		TS	1
		CA	CDUY
		INDEX	FIXLOC
		TS	2
		CA	CDUZ
		INDEX	FIXLOC
		TS	3
		RELINT
		TC	DANZIG

# Page 763
# NAME:		GIMB
#
# FUNCTION:	DETERMINE AND COMPUTE THE DESIRED GIMBAL ANGLES TO BE USED FOR COARSE ALIGNMENT.
#
# CALLING SEQUENCE:  CALL GIMB
#
# INPUT:	DESIRED IMU INERTIAL ORIENTATION VECTORS:  XSMD, YSMD, ZSMD
#
# OUTPUT:	GIMBAL ANGLES LEFT IN THETAD, +1, +2
#
# SUBROUTINES USED:  1.CDUTRIG 2.CALCSMSC 3.CALCGA

		SETLOC	P50S2
		BANK
		COUNT	14/INFLT

CALCSMSC	DLOAD	DMP
			SINCDUY
			COSCDUZ
		DCOMP
		PDDL	SR1
			SINCDUZ
		PDDL	DMP
			COSCDUY
			COSCDUZ
		VDEF	VSL1
		STORE	XNB
		DLOAD	DMP
			SINCDUX
			SINCDUZ
		SL1
		STORE	26D
		DMP
			SINCDUY
		PDDL	DMP
			COSCDUX
			COSCDUY
		DSU
		PDDL	DMP
			SINCDUX
			COSCDUZ
		DCOMP
		PDDL	DMP
			COSCDUX
			SINCDUY
		PDDL	DMP
			COSCDUY
			26D
		DAD	VDEF
		VSL1
		STORE	ZNB
		VXV	VSL1
			XNB
		STORE	YNB
		RVQ

# NAME -- P51 -- IMU ORIENTATION DETERMINATION
# MOD. NO. 2	21 DEC 66				LOG SECTION -- P51-P53
# Page 764
# MOD BY STURLAUGSON					ASSEMBLY SUNDISK REV15
#
# FUNCTIONAL DESCRIPTION
#	DETERMINES THE INERTIAL ORIENTATION OF THE IMU.  THE PROGRAM IS SELECTED BY DSKY ENTRY.  THE SIGHTING
#	ROUTINE IS CALLED TO COLLECT THE CDU COUNTERS AND SHAFT AND TRUNNION ANGLES FOR A SIGHTED STAR.  THE DATA IS
#	THEN PROCESSED AS FOLLOWS.
#
#	1.  SEXTANT ANGLES ARE COMPUTED IN TERMS OF NAVIGATIONAL BASE COORDINATES.  LET SA AND TA BE THE SHAFT AND
#	TRUNNION ANGLES, RESPECTIVELY.  THEN,
#	_
#	V  = (SIN(TA)*COS(SA), SIN(TA)*SIN(SA), COS(TA))	(A COLUMN VECTOR)
#        NB
#	THE OUTPUT IS A HALF-UNIT VECTOR STORED IN STARM.
#
#	2.  THIS VECTOR IN NAV. BASE COORDS. IS THEN TRANSFORMED TO ONE IN STABLE MEMBER COORDINATES.
#	_    T  T  T _
#	V = Q *Q *Q *V  ,	WHERE
#	     1  2  3  NB
#
#	     ( COS(IG)	 0    -SIN(IG) )
#	     (			       )					THE GIMBAL ANGLES ARE COMPUTED FROM
#	Q  = (   0	 1  	 0     ), IG= INNER GIMBAL ANGLE		THE CDU COUNTERS AT NBSM (USING AXIS-
#	 1   (			       )					ROT AND CDULOGIC)
#	     ( SIN(IG)	 0     COS(IG) )
#
#	     ( COS(MG) SIN(MG)   0     )
#	     (			       )
#	Q  = (-SIN(MG) COS(MG)   0     ), MG= MIDDLE GIMBAL ANGLE
#	 2   (                         )
#	     (   0       0       1     )
#
#	     (   1       0       0     )
#	     (                         )
#	Q  = (   0     COS(OG) SIN(OG) ), OG= OUTER GIMBAL ANGLE
#	 3   (                         )
#	     (   0    -SIN(OG) COS(OG) )
#
#	3.  THE STAR NUMBER IS SAVED AND THE SECOND STAR IS THEN SIMILARLY PROCESSED.
#
#	4.  THE ANGLE BETWEEN THE TWO STARS IS THEN CHECKED AT CKSDATA.
#
#	5.  REFSMMAT IS THEN COMPUTED AT AXISGEN AS FOLLOWS.
#		    _      _
#		LET S  AND S  BE TWO STAR VECTORS EXPRESSED IN TWO COORDINATE SYSTEMS, A AND B (BASIC AND STABLE MEMBER).
#		     1      2
# Page 765
#		DEFINE,
#		_    _
#		U  = S
#		 A    A1
#		_         _    _
#		V  = UNIT(S  x S  )
#		 A         A1   A2
#		_    _   _
#		W  = U x V
#		 A    A   A
#
#		AND,
#		_    _
#		U  = S
#		 B    B1
#		_         _    _
#		V  = UNIT(S  x S  )
#		 B         B1   B2
#		_    _   _
#		W  = U x V
#		 B    B   B
#
#		THEN
#		_        _       _       _
#		X  = U  *U + V  *V + W  *W
#		      B1  A   B1  A   B1  A
#		_        _       _       _
#		Y  = U  *U + V  *V + W  *W		(REFSMMAT)
#		      B2  A   B2  A   B2  A
#		_        _       _       _
#		Z  = U  *U + V  *V + W  *W
#		      B3  A   B3  A   B3  A
#
# THE INPUTS CONSIST OF THE FOUR HALF-UNIT VECTORS STORED AS FOLLOWS
#		_
#		S   IN 6-11 OF THE VAC AREA
#		 A1
#		_
#		S   IN 12-17 OF THE VAC AREA
#		 A2
#		_
#		S   IN STARAD
#		 B1
# Page 766	_
#		S   IN STARAD +6
#		 B2
#
# CALLING SEQUENCE:
#
#	THE PROGRAM IS CALLED BY THE ASTRONAUT BY DSKY ENTRY.
#
# SUBROUTINES CALLED:
#
#	GOPERF3
#	GOPERF1R
#	GODSPR
#	IMUCOARS
#	IMUFIN20
#	R53
#	SXTNB
#	NBSM
#	MKRELEAS
#	CHKSDATA
#	MATMOVE
#
# ALARMS
#
#	NONE
#
# ERASABLE INITIALIZATION:
#
#	IMU ZERO FLAG SHOULD BE SET.
#
# OUTPUT
#
#	REFSMMAT
#	REFSMFLG
#
# DEBRIS
#
#	WORK AREA
#	STARAD
#	STARIND
#	BESTI
#	BESTJ

		SETLOC	P50S1
		BANK
		COUNT	14/P5153

P53		EQUALS	P51
P51		CS	IMODES30
		MASK	BIT9
		CCS	A
# Page 767
		TC	P51A
		TC	ALARM
		OCT	210
		TC	GOTOPOOH
P51A		TC	BANKCALL
		CADR	R02ZERO

P51AA		CAF	PRFMSTAQ
		TC	BANKCALL
		CADR	GOPERF1
		TC	GOTOPOOH	# TERM.
		TC	P51B		# V33
		TC	PHASCHNG
		OCT	05024
		OCT	13000
		CAF	P51ZERO
		TS	THETAD		# ZERO THE GIMBALS
		TS	THETAD +1
		TS	THETAD +2
		CAF	V6N22
		TC	BANKCALL
		CADR	GODSPRET
		CAF	V41K		# NOW DISPLAY COARSE ALIGN VERB 41
		TC	BANKCALL
		CADR	GODSPRET
		TC	BANKCALL
		CADR	IMUCOARS
		TC	BANKCALL
		CADR	IMUSTALL
		TC	CURTAINS	# CAGING OR BAD END
		TC	BANKCALL	# SCHEDULE IFAILOK AND IMUFINED TASKS, IN 5
		CADR	IMUFIN20	# AND 20 SECS. DIRECT RETURN AND NO STALL,
		TC	BANKCALL	# IF CAGING, BUT T4 WILL ZERO C/A ENABLE.
		CADR	IMUSTALL	# IF PUT TO SLEEP, IMUFINED WILL WAKE US
		TC	CURTAINS	# UP.
		TC	PHASCHNG
		OCT	05024
		OCT	13000
		TCF	P51AA		# COARSE ALIGN DONE:  RECYCLE FOR FINE

# Page 768
# DO STAR SIGHTING AND COMPUTE NEW REFSMMAT
P51B		TC	PHASCHNG
		OCT	00014
		TC	INTPRET
		SSP	SETPD
			STARIND		# INDEX -- STAR 1 OR 2
			0
			0
		RTB	VLOAD
			SET1/PDT
			ZEROVEC
		STORE	GCOMP
		SET	CLEAR
			DRIFTFLG	# ENABLE T4 COMPENSATION
			TARG2FLG	# SHOW MARK IS STAR --- NOT LANDMARK
		EXIT
		CAF	BIT1
		TS	MARKINDX	# INITIALIZE FOR ONE MARK

P51C		TC	PHASCHNG
		OCT	05024
		OCT	13000
		TC	CHECKMM
		MM	53		# BACKUP PROGRAM
		TCF	P51C.1		# NOT P53
		TC	INTPRET
		CALL
			R56
		GOTO
			P51C.2
P51C.1		TC	INTPRET
		CALL
			R53		# SIGHTING ROUTINE
P51C.2		CALL			# COMPUTE LOS IN SM FROM MARK DATA
			SXTSM
		PUSH
		SLOAD	BZE
			STARIND
			P51D
		VLOAD	STADR
		STORE	STARSAV2	# DOWNLINK
		GOTO
			P51E
P51D		VLOAD	STADR
		STODL	STARSAV1
			TSIGHT
		CALL
			PLANET
		STORE	PLANVEC
# Page 769
P51E		EXIT
		TC	PHASCHNG
		OCT	05024
		OCT	13000
		TC	BANKCALL
		CADR	MKRELEAS	# ZERO MARKSTAT
		CCS	STARIND
		TCF	P51F		# STAR 2
		TC	PHASCHNG
		OCT	05024
		OCT	13000
		CAF	BIT1
		TS	STARIND
		TCF	P51C		# GO DO SECOND STAR
P51F		TC	PHASCHNG
		OCT	05024
		OCT	13000
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
		TC	P51AA
P51G		CALL
			AXISGEN		# COME BACK WITH REFSMMAT IN XDC
		AXC,1	AXC,2
			XDC
			REFSMMAT
		CLEAR	CALL
			REFSMFLG
			MATMOVE
		SET	GOTO
			REFSMFLG
			ENDP50S
PRFMSTAQ	=	OCT15
P51ZERO		=	ZERO
P51FIVE		=	FIVE
V6N22		VN	0622
V41K		VN	4100
SET1/PDT	CA	TIME1
# Page 770
		TS	1/PIPADT
		TCF	DANZIG

# Page 771
# SXTSM COMPUTES AN LOS VECTOR IN SM COORD FROM OCDU AND ICDU MARK DATA

		SETLOC	P50S3
		BANK
SXTSM		STQ
			QMAJ
		LXC,1	DLOAD*
			MARKSTAT
			0D,1
		STORE	TSIGHT
		LXC,2	SLOAD*
			STARIND
			MKDNCDR,2
		LXC,2	VLOAD*
			MPAC
			0,1
		STORE	0,2
		DLOAD*
			5,1
		STORE	5,2
		CALL
			SXTNB		# COMPUTE LOS VECTOR FROM OCDU IN MKVAC
		LXA,1	INCR,1
			MARKSTAT
			2		# INCREMENT TO BASE ADR OF ICDU
		SXA,1	CALL
			S1
			NBSM		# TRANSFORM LOS TO SM
		GOTO
			QMAJ
MKDNCDR		ECADR	MARKDOWN
		ECADR	MARK2DWN

# Page 772
# PROGRAM DESCRIPTION:  R53 -- SIGHTING MARK ROUTINE
# MOD. NO. 2						21 DEC 66
# MOD. BY STURLAUGSON
#
# FUNCTIONAL DESCRIPTION:
#
#	TO PERFORM A SATISFACTORY NUMBER OF SIGHTING MARKS FOR THE REQUESTING PROGRAM (OR ROUTINE).  SIGHTINGS
# 	CAN BE MADE ON A STAR OR LANDMARK.  WHEN THE CMC ACCEPTS A MARK IT RECORDS AND STORES 5 ANGLES (3 ICDUS AND 2
#	OCDUS) AND THE TIME OF THE MARK.
#
# CALLING SEQUENCE:
#
#	R53 IS CALLED AND RETURNS IN INTERPRETIVE CODE.  RETURN IS VIA QPRET.
#	THERE IS NO ERROR EXIT IN THIS ROUTINE ITSELF.
#
# SUBROUTINES CALLED
#
#	SXTMARK
#	OPTSTALL
#	GOFLASH
#
# ERASABLE INITIALIZATION:
#
#	TARGET FLAG -- STAR OR LANDMARK
#	MARKINDX -- NUMBER OF MARKS WANTED
#	STARIND -- INDEX TO BESTI OR BESTJ (STAR NUMBER)
#
# OUTPUT
#
#	MARKSTAT CONTAINS INDEX TO VACANT AREA WEHRE MARK DATA IS STORED
#	BESTI (INDEXED BY STARIND) CONTAINS STAR NUMBER SIGHTED.
#
# DEBRIS
#
#	MARKINDX CONTAINS NUMBER OF MARKS DESIRED

		SETLOC	RT53
		BANK

		COUNT	14/R53

R53		STQ	SET		# SET SIGHTING MARK FLAG
			R53EXIT
			R53FLAG
		EXIT
R53A		CA	MARKINDX	# NUMBER OF MARKS
		MASK	LOW3
		TC	BANKCALL
		CADR	SXTMARK
		TC	BANKCALL
		CADR	OPTSTALL
		TC	CURTAINS
		INDEX	MARKSTAT
		CCS	QPRET		# NUMBER OF MARKS ACTUALLY DONE
		TCF	R53B
		TCF	+2		# ZERO
		TCF	+1		# CCS HOLE
		CAF	ZERO		# HOUSEKEEP VAC AREA SAVE
		XCH	MARKSTAT	#	AND MARKSTAT
# Page 773
		CCS	A
		INDEX	A
		TS	0
		TCF	R53A
R53B		TC	CHECKMM
		MM	22
		TCF	+2
		TCF	R53D
		TC	CHECKMM
		MM	23
		TCF	R53C1
		TCF	R53D
R53C1		CAF	ZERO
		TC	BANKCALL
		CADR	CLEANDSP
R53C		CAF	V01N71
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH	# TERM.
		TCF	R53Z
		TC	R53C		# RECYCLE
R53Z		TC	CHKSCODE
		TC	FALTON
		TC	R53C
		CS	HIGH9
		MASK	STARCODE
		EXTEND
		MP	SIGHTSIX
		XCH	L
		INDEX	STARIND
		TS	BESTI
R53D		TC	INTPRET
R53OUT		SETGO
			TERMIFLG	# SET TERMINATE FOR R52
			R53EXIT
SIGHTSIX	=	SIX
V01N71		VN	0171

# ****** KEEP IN SAME BANK AS R51 AND R53 ********
CHKSCODE	CCS	STARCODE
		AD	NEG47
		CCS	A
		TC	Q		# SC < 0 OR SC > 50
		TCF	+2		# SC = + OR - 0
		TCF	+1		# 0 <= SC< 50
		INDEX	Q		# SC = 50
		TC	00002
NEG47		OCT	77730

# Page 774
# NAME -- S52.2
# FUNCTION -- COMPUTE GIMBAL ANGLES FOR DESIRED SM AND PRESENT VEHICLE
# CALL -- CALL S52.2
# INPUT -- X,Y,ZSMD
# OUTPUT -- OGC,IGC,MGC,THETAD,+1,+2
# SUBROUTINES -- CDUTRIG, CALCSMSC, MATMOVE, CALCGA

		SETLOC	S52/2
		BANK

		COUNT	13/S52.2
S52.2		STQ
			QMAJ
		CALL
			CDUTRIG
		CALL
			CALCSMSC
		AXT,1	SSP
			18D
			S1
			6D
S52.2A		VLOAD*	VXM
			XNB 	+18D,1
			REFSMMAT
		UNIT
		STORE	XNB 	+18D,1
		TIX,1
			S52.2A
S52.2.1		AXC,1	AXC,2
			XSMD
			XSM
		CALL
			MATMOVE
		CALL
			CALCGA
		GOTO
			QMAJ

# Page 775
# PROGRAM NAME:  SR52.1				DATE:  DEC 20 1968
# MOD 1						LOG SEC:  P51-P53
# BY KEN VINCENT				ASSEMBLY:  SUNDISK REV 40
#
# FUNCTION
#
# TARG1 AND TARG2 FLAGS ARE LOOKED AT TO DETERMINE IF THE TARGET IS THE
# LEM, STAR, OR LANDMARK.  IN CASE OF LEM OR LMK, THE PRESENT TIME PLUS
# 2 SECONDS IS SAVED IN AOPTIME (ALIAS STARAD, +1).  IF THE LEM IS
# THE TARGET THEN CONIC UPDATES OF THE CSM AND LEM ARE MADE TO
# THE TIME IN AOPTIME.  THE UNIT OF THE DIFFERENCE OF LEM AND CSM
# POSITION VECTORS BECOMES THE REFERENCE SIGHTING VECTOR USED IN THE
# COMMON PART OF THE THIS PROGRAM.
#
# IN THE CASE OF LANDMARK, THE CSM IS UPDATED CONICALLY.  THE RADIUS
# VECTOR FOR THE LANDMARK IS OBTAINED FROM LALOTORV.  BOTH OF THESE ARE
# FOUND FOR THE TIME IN AOPTIME.  THE UNIT OF THE DIFFERENCE BETWEEN
# THE LANDMARK AND CSM RADIUS VECTORS BECOMES THE REFERENCE SIGHTING
# VECTOR FOR THE COMMON PART OF THIS ROUTINE.
#
# IF A STAR IS THE TARGET, THE PROPER STAR IS OBTAINED FROM THE CATALOG
# AND THIS VECTOR BECOMES THE REFERENCE SIGHTING VECTOR.
#
# THE COMMON PART OF THIS PROGRAM TRANSFORMS THE REFERENCE SIGHTING
# VECTOR INTO STABLE MEMBER COORDINATES.  IT READS THE IMU-CDUS AND USES
# THIS DATA IN A CALL TO CALCSXA.  ON RETURN FROM CALCSXA A TEST IS
# MADE TO SEE IF THE TRUNNION ANGLE IS GREATER THAN  90DEG OR 38DEG.
# MADE TO SEE IF THE TRUNNION ANGLE IS GREATER THAN 90DEG. OR 50DEG.
#
# CALLING SEQUENCE
#
# 	L+4	RETURN WHEN SHAFT OR TRUNION NOT WITHIN 5 DEG OF DESIRED
#	L	TC	BANKCALL
#	L+1	CADR	SR52.1
#	L+2	ERROR RETURN	TRUNNION GREATER THAN 90 DEG.
#	L+3	ERROR RETURN	TRUNNION GREATER THAN 50 DEG
#	L+4	NORMAL RETURN
#
# OUTPUT
#
#	SAC:	SINGLE PREC, 2'S COMP, SCALED AT HALF REVS -- SHAFT ANGLE DESIRED.
#	PAC:	SINGLE PREC, 2'S COMP, SCALED AT EIGHTH REVS -- TRUNNION ANGLE DESIRED.
#
# INITIALIZATION
#
#	IF TARG1FLG =1 THEN TARGET IS LEM -- NO OTHER INPUT REQUIRED.
#
#	IF TARG1FLG =0 AND TARG2FLG =0 THE TARGET IS STAR, STARIND SHOULD
#	0 OR 1 DENOTING BESTI OR BESTJ RESPECTIVELY AS STAR CODE.  STAR CODES
#	ARE 6 TIMES STAR NUMBER.
#
#	IF TARG1FLG =0 AND TARG2FLG =1 THEN TARGET IS LANDMARK.  SETT ROUTINE
#	LALOTORV FOR INPUT REQUIREMENTS.  HERE FIXERAD=1 FOR CONSTANT EARTH
#	RADIUS
#
# DEBRIS
#
#	WORK AREA
#	STARAD -- STAR+5 (STAR IS DESIRED LOS IN STABLE MEMBER COORDINATES)

		COUNT*	$$/SR521
# Page 776
		SETLOC	SR52/1
		BANK

SR52.1		TC	MAKECADR
		TS	QMIN
		TC	INTPRET
		RTB	DAD
			LOADTIME
			1.3SECDP
		STORE	AOPTIME
		BON	BON
			TARG1FLG
			LEM52
			TARG2FLG
			LMK52
		GOTO
			STAR52
LEM52		DLOAD
			AOPTIME
		STCALL	TDEC1
			LEMCONIC
		VLOAD
			RATT
		GOTO
			LMKLMCOM
LMK52		BON	DLOAD
			ADVTRK
			ADVTRACK
			AOPTIME
		CALL
			LALOTORV
		VLOAD
			ALPHAV
LMKLMCOM	STODL	STAR
			AOPTIME
		STCALL	TDEC1
			CSMCONIC
		VLOAD	VSU
			STAR
			RATT
		UNIT	GOTO
			COM52
STAR52		SSP	LXA,1
			S1
			0
			STARIND
		TIX,1
			ST52ST
		VLOAD	GOTO
			STARSAV2
# Page 777
			COM52
ST52ST		VLOAD
			STARSAV1
COM52		MXV	UNIT
			REFSMMAT
		STORE	STAR
		SETPD	CALL
			0
			CDUTRIG		# COMPUTES SINES AND COSINES FOR CALCSXA
		CALL			#	NOW EXPECT TO SEE THE CDU ANGLES.
			CALCSXA
		BOFF	EXIT
			CULTFLAG
			TRUN38
		TC	SR52E1
TRUN38		DLOAD	DSU
			PAC
			38TRDEG
		BPL	DLOAD
			SR52E22
			PAC
		DSU	BPL
			20DEGSMN
			SR52E3
SR52E22		EXIT
		TC	SR52E2
SR52E3		EXIT
		INCR	QMIN
SR52E2		INCR	QMIN
SR52E1		CA	QMIN
		TC	SWCALL
38TRDEG		2DEC	.66666667	# CORRESPONDS TO 50 DEGS IN TRUNION

1.3SECDP	2DEC	130

20DEGSMN	DEC	-07199
		DEC	-0

# Page 778
# THE ADVTRACK ROUTINE IS USED TO COMPUTE AN OPTICS LOS VECTOR TO
# A POINT ON THE GROUND TRACK 60 DEGREES FORWARD OF THE LOCAL VERTICAL
# OF AN ADVANCED ORBIT A SPECIFIED NUMBER OF REVOLUTIONS FROM NOW.

		SETLOC	26P50S
		BANK
ADVTRACK	SETPD
			0
		VLOAD	PUSH		# INITIALIZE FOR RP-TO-R
			UNITZ		# UZ VEC IN PD 0-5
		RTB	PUSH		# TIME IN PD 6-7
			LOADTIME
		STCALL	AOPTIME		# TIME ALSO IN AOPTIME FOR CSMCONIC
			RP-TO-R		# GET MOON ROTATION VEC IN REF
		STODL	STAR
			AOPTIME		# PICK UP TIME
		STCALL	TDEC1		# UPDATE STATE TO TIME
			CSMCONIC
		VLOAD	VXV
			VATT
			RATT
		UNIT
		STOVL	24D		# SAVE -UNIT(VxR) FOR 2ND ROTATION
			RATT
		UNIT	VCOMP
		SETPD	PUSH		# PUSH LOS=-UNIT(RVEC) PD 0-5
			0
		EXIT
		CA	LANDMARK
		MASK	SEVEN		# GET NUMBER OF ADVANCE PERIODS
		EXTEND
		MP	BIT11		# GET N/16
		XCH	L
		INDEX	FIXLOC
		TS	30D		# TEMP STORE N/16
		TC	INTPRET
		SLOAD	DMP
			30D
			MPERIOD
		STCALL	AOPTIME		# ROTATE ANG ABOUT UR
			ROTA
		VLOAD
			24D		# PICK UP 2ND ROTATION AXIS
		STODL	STAR
			DP1/6
		DSU
			AOPTIME		# 2ND RAT ANGLE = 60 - A
		STCALL	AOPTIME
			ROTA		# GO ROTATE 2ND TIME
		VLOAD
# Page 779
			0
		STCALL	STAR		# STORE FINAL LOS IN STAR
			COM52		# RETURN TO SR52.1

;
; ROTA - ROTATION SUBROUTINE
;
; COMMENT-ONLY READERS: This mathematical routine rotates vectors in 3D space,
; essential for computing where stars will appear from different spacecraft
; orientations. Like rotating a telescope to aim at different stars.
;
; CODE-ALONG READERS: Implements Rodrigues' rotation formula to rotate a 
; line-of-sight vector (LOS) stored at location 0 about a unit rotation axis
; (UR) stored in STAR, by angle AOPTIME. The rotation formula is:
;   ROTATED = LOS*cos(A) + (UR×LOS)*sin(A) + UR*(UR·LOS)*(1-cos(A))
; Uses double-precision interpretive math for maximum accuracy. This is called
; during star pattern rotation to account for spacecraft motion during alignment.
;
ROTA		DLOAD	SIN
			AOPTIME
		PDVL	VXV		# PUSH 1/2SIN(A) PD 6-7
			STAR		#	UR VEC
			0		#	LOS
		VXSC	VSL2		# 1/2SIN(A)(URXLOS) PD 6-11
		PDVL	DOT
			STAR
			0
		VXSC	VSL2
			STAR
		PDDL	COS		# 1/2(UR . LOS)UR 12-17
			AOPTIME
		PDVL	BVSU		# PUSH 1/2COS(A) 18-19
			12D
			0
		VXSC	VSL1		# UP 18-19
		VAD	VAD		# UP 12-17 UP 6011
		UNIT	SETPD
			0
		PUSH	RVQ

DP1/6		2DEC	.16666666

MPERIOD		2DEC	.047619		# APPROX LUNAR ROT ANG IN 2HRS x 16

# Page 780
# NAME -- S52.3
# FUNCTION --	XSMD= UNIT(YSMD x ZSMD)
#		YSMD= UNIT(V X R)
#		ZSMD= UNIT(-R)
# CALL --	DLOAD	CALL
#			TALIGN
#			S52.3
# INPUT --	TIME OF ALIGNMENT IN MPAC
# OUTPUT --	X,Y,ZSMD
# SUBROUTINES -- CSMCONIC

		SETLOC	P50S2
		BANK

		COUNT	15/S52.3
;
; S52.3 - COMPUTE NOMINAL ORIENTATION (REFSMMAT)
;
; COMMENT-ONLY READERS: This calculates the ideal platform orientation based
; on where the spacecraft is and which direction it's moving. The platform 
; always knows which way to point for optimal navigation accuracy.
;
; CODE-ALONG READERS: Computes nominal REFSMMAT orientation at alignment time
; TDEC1 using current state vector (position R and velocity V). The coordinate
; system is defined as:
;   ZSMD = UNIT(-R)      [Z-axis points toward Earth/Moon center]
;   YSMD = UNIT(V × R)   [Y-axis normal to orbital plane]
;   XSMD = UNIT(Y × Z)   [X-axis completes right-hand system]
; This orientation minimizes gimbal rates during orbital flight. Calls CSMPREC
; to get precision state vector at alignment time, then builds orthonormal basis.
;
S52.3		STQ
			QMAJ
		STCALL	TDEC1
			CSMPREC
		SETPD
			0
		VLOAD	VCOMP
			RATT
		UNIT
		STOVL	ZSMD
			VATT
		VXV	UNIT
			RATT
		STORE	YSMD
		VXV	UNIT
			ZSMD
		STCALL	XSMD
			QMAJ

# Page 781
# PROGRAM DESCRIPTION:  R56 -- ALTERNATE LOS SIGHTING MARK ROUTINE
#
# FUNCTIONAL DESCRIPTION
#
#	TO PERFORM SIGHTING MARKS FOR THE BACK-UP ALIGNMENT PROGRAMS (P53,P54).  THE ASTRONAUT KNOWS THE
#	COORDINATES (OPTICS) OF THE ALTERNATE LINE OF SIGHT HE MUST USE FOR THIS ROUTINE.  WHEN THE ASTRONAUT KEYS IN
#	ENTER IN RESPONSE TO THE FLASHING V50 N25 R1-XXXXX THE CMC STORES THE THREE ICDU ANGLES AND TWO ANGLES DISPLAYED
#	IN N92.
#
# CALLING SEQUENCE
#
#	CALL
#		R56
#
# SUBROUTINES CALLED
#
#	A PORTION OF SXTMARK (VAC.AREA SEARCH)
#	GOFLASH
#	GOPERF1
#
# ERASABLE INITIALIZATION
#
#	STARIND:  INDEX TO STAR NUMBER
#
# OUTPUT
#
#	MARKSTAT:  INDEX TO VAC.AREA WHERE OUTPUT IS STORED.
#	BESTI (INDEXED BY STARIND) CONTAINS STAR NUMBER.
#	ICDU AND OCDU ANGLES IN VAC. AREA AS FOLLOWS:
#		VAC +2	CDUY
#		VAC +3	CDUS
#		VAC +4	CDUZ
#		VAC +5	CDUT
#		VAC +6	CDUX

		COUNT*	$$/R56
		SETLOC	P50S3
		BANK
;
; R56 - ALTERNATE LOS SIGHTING MARK ROUTINE
;
; COMMENT-ONLY READERS: For backup alignment (P53/P54), the crew manually
; sights on stars using the spacecraft's optical telescope. When they see
; a star aligned, they press ENTER to record the exact spacecraft attitude.
; This gives the computer two star sightings to compute IMU alignment.
;
; CODE-ALONG READERS: Backup alignment procedure when automatic star tracker
; unavailable. Displays V06N94 to show current optics angles (shaft/trunnion).
; Crew manually positions optics on known star, then presses ENTER. Routine
; records IMU gimbal angles (CDUX, CDUY, CDUZ) and optics angles (SAC, PAC)
; at mark time in VAC area indexed by MARKSTAT. Two marks on different stars
; provide sufficient data for R51 fine alignment calculation. Used during
; Apollo 11 when automatic star tracker had difficulty acquiring certain stars.
;
R56		STQ	EXIT
			R53EXIT
		CAF	V06N94B
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH	# TERM.
		TC	R56A		# PROCEED:  ANGLES OK
		TC	-5		# ENTER:  NEW ANGLES
R56A		TC	BANKCALL
		CADR	SXTMARK +2	# INHIBIR EXT VB ACT AND FIND VAC AREA

		CAF	ZERO
		TC	BANKCALL
		CADR	CLEANDSP

R56A1		CAF	VB53		# DISPLAY V53 REQUESTING ALTERNATE MARK
		TC	BANKCALL
# Page 782
		CADR	GOMARK2
		TCF	GOTOPOOH	# V34:  TERMINATE
		TCF	R56A1		# V33:  DON'T PROCEED -- JUST ENTER TO MARK
		TC	INTPRET
		DLOAD
			MRKBUF1 +3
		STODL	SAC
			MRKBUF1 +5
		STORE	PAC
		EXIT
		INHINT
		EXTEND
		DCA	TIME2
		INDEX	MARKSTAT
		DXCH	0
		CA	CDUY		# ENTER:  THIS IS A BACKUP SYSTEM MARK
		INDEX	MARKSTAT
		TS	2
		CA	SAC
		INDEX	MARKSTAT
		TS	3
		CA	CDUZ
		INDEX	MARKSTAT
		TS	4
		CA	PAC
		INDEX	MARKSTAT
		TS	5
		CA	CDUX
		INDEX	MARKSTAT
		TS	6
		RELINT
		TC	CLEARMRK	# ENABLE EXTENDED VERBS
		CAF	OCT16
		TC	BANKCALL
		CADR	GOPERF1
		TC	GOTOPOOH	# TERM.
		TCF	R56B		# PROCEED:  MARK COMPLETED
		TCF	R56A 	+2	# RECYCLE:  DO ANOTHER MARK -- LIKE REJECT.
R56B		TC	BANKCALL
		CADR	R53C1
VB53		VN	05300		# ALTERNATE MARK VERB
V06N94B		VN	00694
		SETLOC	P50S
		BANK

;
; PLANET - COMPUTE LINE-OF-SIGHT TO PLANETARY BODY
;
; COMMENT-ONLY READERS: When viewing Earth or Moon through the telescope for
; alignment, the computer must calculate exactly where that planetary body
; appears in the telescope based on the spacecraft's current position.
;
; CODE-ALONG READERS: Computes unit line-of-sight vector to specified planetary
; body (Earth or Moon) as seen from spacecraft position. Input is sighting time
; in TSIGHT. Calls LOCSAM to determine which body (based on sphere of influence)
; and compute current spacecraft position. Returns unit vector pointing from
; spacecraft toward planetary body center, accounting for spacecraft position
; relative to Earth-Moon system. Used for planetary horizon sightings.
;
PLANET		STORE	TSIGHT
		STQ	CALL
			QMIN
			LOCSAM
		VLOAD
# Page 783
			VEARTH
		STOVL	0D
			VSUN
		STOVL	VEARTH
			0D
		STORE 	VSUN
;
; NOSAM - NO SAMPLING REQUIRED PATH
;
; COMMENT-ONLY READERS: When the crew manually selects a star (not requiring
; automatic searching), this path processes that selection and prepares the
; star data for the alignment calculation.
;
; CODE-ALONG READERS: Handles case where LOCSAM not needed (crew has pre-selected
; star rather than planetary body). Decodes STARCODE to extract star catalog
; index, multiplies by 6 (SIGHTSIX) to compute offset into star table, and stores
; result in BESTI array indexed by STARIND. HIGH9 mask (octal 77600) isolates
; the 9-bit star number from STARCODE. Each star table entry is 6 words (unit
; vector components + magnitude), hence the factor-of-6 addressing calculation.
;
NOSAM		EXIT
		CS	HIGH9
		MASK	STARCODE
		EXTEND
		MP	SIGHTSIX
		XCH	L
		INDEX	STARIND
		TS	BESTI
		CCS	A
		TCF	NOTPLAN
		CAF	VNPLANV
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	+2
		TC	-5
		TC	INTPRET
		VLOAD	VXSC
			STARSAV3
			1/SQR3
		UNIT	GOTO
			CORPLAN
;
; NOTPLAN - NOT A PLANETARY BODY (STAR SIGHTING)
;
; COMMENT-ONLY READERS: After determining this is a star sighting (not Earth
; or Moon), the computer retrieves the star's position from the catalog.
;
; CODE-ALONG READERS: Star sighting path. Complements A register value from
; previous check, adds DEC227 (decimal 227) to determine if target is catalog
; star or special entry. BZMF branches to CALSAM1 if result ≤ 0 (indicates
; Earth or Moon entry). For catalog stars, retrieves offset from BESTI array
; and stores in X1 for indexed addressing into CATLOG star table. CATLOG
; contains unit vectors to 37 navigation stars in Basic Reference Coordinate
; System. These stars were carefully selected by MIT for optimal distribution
; across the celestial sphere and reliable identification characteristics.
;
NOTPLAN		CS	A
		AD	DEC227
		EXTEND
		BZMF	CALSAM1
		INDEX	STARIND
		CA	BESTI
		INDEX	FIXLOC
		TS	X1
		TC	INTPRET
		VLOAD*	GOTO
			CATLOG,1
			CORPLAN
;
; CALSAM1 - CALCULATE SAMPLED PLANETARY BODY VECTOR
;
; COMMENT-ONLY READERS: For Earth or Moon sightings, the computer retrieves
; the planetary body's position data and prepares it for the alignment
; calculation, accounting for the spacecraft's motion during the sighting.
;
; CODE-ALONG READERS: Handles planetary body (Earth or Moon) sighting case.
; Uses STARIND to index into BESTI array, then loads result into index register
; X1 via MPAC. Offset of -228D (decimal -228) maps to correct entry in STARAD
; table for planetary ephemeris data. STARAD contains unit vectors and distance
; data for Earth and Moon as viewed from spacecraft. Falls through to CORPLAN
; to apply velocity aberration correction.
;
CALSAM1		TC	INTPRET
		LXC,1	DLOAD*
			STARIND
			BESTI,1
		LXC,1	VLOAD*
			MPAC
			STARAD 	-228D,1
;
; CORPLAN - CORRECT FOR PLANETARY MOTION (VELOCITY ABERRATION)
;
; COMMENT-ONLY READERS: When the crew sights on a star or planetary body, light
; takes time to travel from the object to the spacecraft. During this time, the
; spacecraft has moved, so the computer must correct for this "aberration" effect
; to get the true direction.
;
; CODE-ALONG READERS: Applies velocity aberration correction to line-of-sight
; vector. Adds VEL/C (spacecraft velocity divided by speed of light) to the
; unit vector pointing toward target. This correction accounts for the finite
; speed of light and spacecraft motion during light travel time. Effect is
; typically small (~0.001 degrees for 8 km/s orbital velocity) but necessary
; for precise IMU alignment. After correction, normalizes result to unit vector
; via UNIT instruction, then returns via GOTO QMIN. All star catalog and
; planetary vectors require this correction for accurate navigation.
;
CORPLAN		VAD	UNIT
			VEL/C
		GOTO
# Page 784
			QMIN
DEC227		DEC	227
VNPLANV		VN	0688
1/SQR3		2DEC	.57735021


