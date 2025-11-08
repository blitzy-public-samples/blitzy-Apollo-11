# Copyright:    Public domain.
# Filename:     P11.agc
# Purpose:      Part of the source code for Colossus 2A, AKA Comanche 055.
#               It is part of the source code for the Command Module's (CM)
#               Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:    yaYUL
# Contact:      Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:      www.ibiblio.org/apollo.
# Pages:	533-550
# Mod history:  2009-05-13 HG   Started adapting from the Colossus249/ file
#                		of the same name, using Comanche055 page
#                		images 0533.jpg - 0550.jpg.
#		2009-05-20 RSB	Corrections: ERTHALT -> EARTHALT,
#				STATSW -> SATSW.
#		2009-05-23 RSB	At end of RESCALES, corrected TC 0 to TC Q.
#				Added an SBANK= prior to a 2CADR.
#		2010-08-24 JL	Fixed page numbers. Added missing comment character on p537.
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
# Page 533
# EARTH ORBIT INSERTION MONITOR PROGRAM
# *************************************

; ============================================================================
; FILE: P11.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: launch/earth-orbit
;
; TL;DR: Earth orbit insertion monitoring program verifying ascent targeting
;        accuracy and orbital parameter validation following Saturn V launch.
;        Provides crew with orbital characteristics display (velocity, altitude,
;        altitude rate) and confirms successful Earth parking orbit achievement
;        before translunar injection. Critical for Apollo 11's July 16, 1969
;        launch sequence validation.
;
; COMMENT-ONLY READERS: This program confirmed Apollo 11 successfully reached
;        Earth orbit after launch, displaying critical flight parameters to
;        the crew and computing attitude errors during the Saturn V boost phase.
; CODE-ALONG READERS: Study orbital parameter computation from state vectors,
;        Keplerian element conversion, attitude error display mathematics using
;        6th-degree polynomials, and the Saturn Takeover manual control feature.
; ============================================================================

# PROGRAM DESCRIPTION -P11-

#	MOD NO. 1
#	MOD BY ELIASSEN

; ============================================================================
; PROGRAM OVERVIEW: EARTH ORBIT INSERTION MONITOR
;
; For Apollo 11's July 16, 1969 launch, this program initiated at the moment
; of liftoff from Kennedy Space Center Launch Complex 39A. As the Saturn V
; rocket lifted the Command Module Columbia and Lunar Module Eagle toward space,
; P11 zeroed the CMC clock, computed initial navigation vectors, and began
; monitoring the ascent trajectory toward Earth parking orbit.
;
; The program served three critical functions during the boost phase:
; 1) Display real-time velocity, altitude, and altitude rate to crew
; 2) Compute and display attitude errors on FDAI needles during Saturn guidance
; 3) Provide Saturn Takeover capability for manual S-IVB stage control
;
; After approximately 12 minutes of flight, Apollo 11 achieved Earth parking
; orbit at 100.4 nautical miles altitude. P11 confirmed orbital parameters
; were within acceptable limits for the upcoming translunar injection burn.
; ============================================================================

# FUNCTIONAL DESCRIPTION

; ============================================================================
; PROGRAM INITIATION - Two Methods
; ============================================================================

#	P11 IS INITIATED BY

#		A) GYROCOMPASS PRG P02 WHEN LIFTOFF DISCRETE IS RECEIVED OR
;		   Automatic initiation: When first motion detected at Launch Complex
;		   39A, hardware discrete signal triggers P11 automatically from the
;		   gyrocompass alignment program. This was the normal Apollo 11 mode.

#		B) BACKUP THRU VERB 75 ENTER
;		   Manual backup: If automatic initiation fails, crew can manually
;		   start P11 using Verb 75. Provides redundancy for mission-critical
;		   ascent monitoring capability.

; ============================================================================
; CRITICAL PROGRAM FUNCTIONS DURING APOLLO 11 ASCENT
;
; At T-0 (9:32 AM EDT, July 16, 1969), the following sequence initiated:
; ============================================================================

#	PROGRAM WILL
#		1. ZERO CMC CLOCK AT LIFTOFF (OR UPON RECEIPT OF BACKUP)
;		   Mission time begins: The AGC clock synchronizes to the moment
;		   of first motion, establishing T+0 for all subsequent navigation
;		   and guidance computations throughout the lunar mission.
#		2. UPDATE TEPHEM TO TIME CMC CLOCK WAS ZEROED
;		   Ephemeris time correlation: Links AGC mission time to astronomical
;		   time base for accurate celestial navigation calculations during
;		   translunar coast phase.

#		3. INITIATE SERVICER AT PREREAD1
;		   Background task activation: Begins periodic updates of navigation
;		   state and display parameters throughout ascent phase. Servicer runs
;		   as low-priority job updating every 2 seconds.

#		4. CHANGE MAJOR MODE TO 11
;		   Mode display: DSKY shows "11" indicating Earth Orbit Insertion
;		   Monitor is active, confirming to crew that ascent monitoring begun.
;		   Crew sees this immediately after liftoff.

#		5. CLEAR DSKY IN CASE OF V 75
;		   Display initialization: Ensures clean display state for Noun 62
;		   orbital parameter presentation, removing any previous displays from
;		   manual backup initiation sequence.

#		6. STORE LIFTOFF IMU-CDU ANGLES FOR ATT. ERROR DISPLAY
;		   Reference attitude capture: Saves initial IMU gimbal angles at
;		   liftoff for computing attitude deviations during boost phase. These
;		   angles define the "zero error" reference for first 10 seconds.
#		7. TERMINATE GYROCOMPASSING	-  -
;		   IMU mode transition: Ends prelaunch alignment mode, switching IMU
;		   from gyrocompass (aligning to local vertical and north) to inertial
;		   reference mode for flight navigation. IMU now maintains fixed
;		   orientation relative to inertial space.

#		8. COMPUTE INITIAL VECTORS	RN, VN            -  -  -
;		   State vector initialization: Calculates initial position (RN) and
;		   velocity (VN) vectors from launch pad coordinates at Kennedy Space
;		   Center Complex 39A (28.5° N latitude, 80.6° W longitude, ~10 meters
;		   above sea level). Initial velocity includes Earth rotation component
;		   (~1,470 fps eastward at this latitude).

#		9. COMPUTE REFSMMAT FOR PRELAUNCH ALIGNMENT WHERE U ,U ,U ARE
#			-         -                                X  Y  Z
#			U =(UNIT(-R) LOCAL VERTICAL AT TIME OF LIFTOFF
#			Z
;			   Z-axis definition: Unit vector pointing up (opposite of local
;			   gravity vector R), establishing local vertical as reference.

#			-	-   -
#			U =UNIT(A), A=HOR VECTOR AT LAUNCH AZIMUTH
#			X
;			   X-axis definition: Unit vector aligned with launch azimuth.
;			   For Apollo 11, azimuth was 72° east of north to achieve 31.5°
;			   orbital inclination (compromise between fuel efficiency and
;			   lunar orbit plane accessibility).

#			-  -   -
#			U =U * U
#			 U  Z   X
;			   Y-axis definition: Cross product of Z and X axes completes
;			   right-handed coordinate system. REFSMMAT (Reference Stable Member
;			   Matrix) transforms inertial coordinates to this launch-site
;			   reference frame, enabling attitude error computation during
;			   Saturn V guidance phase.

#		10. SET REFSMMAT KNOWN FLAG
;		    Matrix validation: Confirms reference frame is established and
;		    reliable for navigation computations throughout mission. Other
;		    programs check this flag before using REFSMMAT for transformations.

#		11. SET AVGEXIT IN SERVICER TO VHHDOT TO
#		    COMPUTE AND DISPLAY NOUN 62 EVERY 2 SECONDS
;		    Crew display activation: Every 2 seconds during ascent, servicer
;		    calls VHHDOT routine to compute and display three critical orbital
;		    parameters on DSKY. Crew monitors to verify ascent profile matches
;		    expected trajectory toward parking orbit.

#		    R1	V1  - INERTIAL VELOCITY MAGNITUDE IN FPS
;		           Velocity climbs from ~1,470 fps (Earth rotation) at liftoff
;		           to approximately 25,600 fps (orbital velocity) at insertion.
;		           Crew monitors to confirm expected acceleration profile during
;		           first stage (~6,000 fps at staging), second stage (~15,000 fps
;		           at S-II cutoff), and S-IVB third stage burn to orbital speed.

#		    R2	HDOT - RATE OF CHANGE OF VEHICLE VEL IN FPS
;		           Altitude rate (vertical velocity component) shows climb speed,
;		           reaching maximum during first-stage burn, then decreasing as
;		           trajectory flattens toward horizontal orbital insertion. At
;		           Apollo 11 orbital insertion, HDOT approached zero as vehicle
;		           achieved circular parking orbit.

#		    R3	H    - VEHICLE ALTITUDE ABOVE PAD IN NM
;		           Altitude increases from 0 NM at Launch Complex 39A pad to
;		           approximately 100 NM at orbital insertion. Apollo 11 achieved
;		           100.4 NM x 98.9 NM parking orbit (nearly circular). Crew
;		           verifies altitude increasing at expected rate throughout boost.

#		12. DISPLAY BODY AXES ATT. ERRORS ON FDAI NEEDLES
;		    Attitude error presentation: Flight Director Attitude Indicator
;		    (FDAI) needles show pitch, yaw, and roll deviations from desired
;		    Saturn V guidance trajectory. These error needles provide crew
;		    immediate visual feedback on vehicle orientation accuracy during
;		    critical boost phase. Errors beyond acceptable limits would trigger
;		    crew evaluation of abort options.
#
#		    A) FROM L.O. TO RPSTART (APPROX. 0 TO +10SECS AFTER L.O.)
#		       DESIRED ATTITUDE IS AS STORED AT L.O.
;		       Vertical rise phase: First ~10 seconds after liftoff, Saturn V
;		       climbs straight up from pad to clear launch tower (398 feet tall).
;		       Desired attitude is the launch orientation captured at T-0.
;		       During Apollo 11 ascent, vehicle maintained vertical orientation
;		       during tower clearance, with FDAI needles showing minimal error.
;		       Any significant deviation would indicate vehicle drift, wind shear
;		       effects, or guidance malfunction requiring immediate crew attention.

#		    B) FROM RPSTART TO POLYSTOP(APPROX.+10 TO +133SECS AFTER LO)
#		       DESIRED ATTITUDE IS SPECIFIED BY CMC PITCH AND ROLL
#		       POLYNOMIALS DURING SATURN ROLLOUT AND PITCHOVER
;		       Programmed guidance phase: After clearing tower, Saturn V executes
;		       roll program (T+13 seconds for Apollo 11) aligning flight azimuth
;		       to 72° east of north for 31.5° orbital inclination. Pitch program
;		       begins shortly after, gradually tipping vehicle toward horizontal.
;		       CMC computes desired attitude every computation cycle using 6th-
;		       degree polynomials fitted to Saturn Instrument Unit guidance
;		       trajectory. FDAI needles display computed errors between actual
;		       IMU-sensed attitude and polynomial-specified desired attitude.
;		       This comparison allows crew to verify Saturn guidance maintaining
;		       correct ascent profile toward parking orbit insertion conditions.
# Page 534
#			THE DISPLAY IS RUN AS LOW PRIORITY JOB APPROX.
#			EVERY 1/2 SEC OR LESS AND IS DISABLED UPON OVFLO OF TIME1
;			Display update timing: Attitude error needles refresh at high
;			rate (approximately twice per second) as low-priority background
;			task. Display automatically terminates when TIME1 overflows,
;			indicating end of polynomial guidance phase (approximately T+133
;			seconds). After polynomial phase, crew transitions to monitoring
;			other mission programs for orbital insertion verification.

# SUBROUTINES CALLED
;		P11 orchestrates complex initialization and display logic by calling
;		numerous AGC operating system and navigation subroutines. Critical
;		calls include INTPRET (interpreter mode for vector math), LALOTORV
;		(latitude/longitude to position vector conversion for launch site),
;		CDUTRIG (IMU gimbal angle processing), NEEDLER (FDAI needle display
;		driver), and CALCGRA (gravity calculation). Executive calls (FINDVAC,
;		PHASCHNG, 2PHSCHNG) manage job scheduling and restart protection.
;		WAITLIST provides timer-driven task sequencing. BANKCALL/IBNKCALL
;		handle cross-bank subroutine invocation in AGC's 36K fixed memory.

#		2PHSCHNG	BANKCALL	CALCGRA		CDUTRIG		CLEANDSP	DANZIG
#		DELAYJOB	EARTHR		ENDOFJOB	FINDVAC		IBNKCALL
#		INTPRET		LALOTORV	NEEDLER		NEWMODEX	PHASCHNG
#		POSTJUMP	POWRSERS	PREREAD1	REGODSPR	S11.1
#		SERVEXIT	TASKOVER	TCDANZIG	V1STO2S		WAITLIST

# ASTRONAUT REQUESTS (IF ALTITUDE ABOVE 300,000 FT)
;		Above 300,000 feet (~57 NM), crew can request supplementary orbital
;		parameter displays beyond standard Noun 62 (velocity, H-dot, altitude).
;		These displays provide additional verification of orbital characteristics
;		during final boost to parking orbit. During Apollo 11 ascent on July 16,
;		1969, crew monitored these values to confirm successful insertion into
;		planned 100 NM circular parking orbit before translunar injection.
#
#	DSKY -
#	     MONITOR DISPLAY OF TIME TO PERIGEE R1 HOURS
#						R2 MINUTES
;		Time-to-perigee display: Shows countdown to lowest orbital point.
;		For Apollo 11's nearly-circular parking orbit (100.4 x 98.9 NM),
;		perigee occurred approximately every 88 minutes (orbital period).
;		Crew uses this to verify orbital mechanics computations and plan
;		upcoming maneuvers (translunar injection occurred at second perigee).

#	DSKY -
#	     MONITOR DISPLAY OF R1 APOGEE ALTITUDE IN NAUTICAL MILES
#				R2 PERIGEE ALTITUDE IN NAUTICAL MILES
#				R3 TFF IN MINUTES/SECS
;		Orbital characteristics display: Apogee (highest point) and perigee
;		(lowest point) altitudes define orbital ellipse shape. TFF (Time From
;		Fictitious impacting trajectory) indicates time until theoretical Earth
;		impact if all thrust ceased—a safety metric. Apollo 11's 100.4 x 98.9 NM
;		orbit had minimal eccentricity, confirming accurate S-IVB insertion.

#	IF ASTRONAUT HAS REQUESTED ANY OF THESE DISPLAYS HE MUST
# HIT PROCEED TO RETURN TO NORMAL NOUN 62 DISPLAY.
;		Display navigation: After reviewing supplementary orbital data, crew
;		presses PROCEED key to return to continuous Noun 62 monitoring (velocity,
;		altitude rate, altitude). This ensures crew maintains awareness of current
;		vehicle state rather than static orbital parameters.
# NORMAL EXIT MODE
;		Program termination: Crew exits P11 by keying VERB 37 ENTER 00 ENTER
;		on DSKY. This command changes major mode to Program 00 (idle state).
;		During Apollo 11 mission, crew terminated P11 after confirming successful
;		orbital insertion and beginning preparations for systems checkout in
;		parking orbit before translunar injection burn.

#	ASTRONAUT	VERB 37 ENTER 00 ENTER

# ALARM MODES - NONE
;		P11 generates no program alarms under normal operation. All potential
;		error conditions (IMU failures, computation overflows, etc.) handled
;		by calling subroutines which issue their own alarms if needed. P11
;		itself maintains simple monitoring role without alarm generation logic.

# ABORT EXIT MODES -
;		No specific abort exit defined for P11. If mission abort required during
;		ascent (launch escape system activation, S-IVB shutdown failure, etc.),
;		crew would transition directly to appropriate abort program (P70 for CM/SM
;		separation, contingency abort programs). P11 monitoring continues providing
;		situational awareness until superseded by abort program sequence.

# OUTPUT
;		P11 produces multiple outputs for crew displays and internal AGC state:

#	TLIFTOFF (DP)	TEPHEM (TP)
;		Time variables: TLIFTOFF (double-precision) records mission elapsed time
;		at liftoff (T-0 moment). TEPHEM (triple-precision) stores updated
;		ephemeris reference time synchronized to CMC clock zero event at liftoff.
;		These time references anchor all subsequent mission timeline computations.
;		For Apollo 11: TLIFTOFF = 0.0 at 13:32:00 UTC, July 16, 1969.

#	REFSMMAT
;		Reference to Stable Member Matrix: 3×3 rotation matrix defining
;		relationship between inertial reference frame and IMU stable member
;		(platform) orientation. P11 computes REFSMMAT aligned to launch site:
;		Z-axis points up (local vertical), X-axis points along launch azimuth
;		(72° east of north for Apollo 11's 31.5° inclination trajectory),
;		Y-axis completes right-handed coordinate system. This matrix enables
;		transformation between IMU-sensed attitude and geographic orientation.

#	DSKY DISPLAY
;		Numeric displays: Continuous Noun 62 presentation showing R1 = inertial
;		velocity magnitude (feet/second), R2 = altitude rate H-dot (feet/second),
;		R3 = altitude above pad (nautical miles). Updates every 2 seconds during
;		ascent. Apollo 11 crew monitored these values throughout 11-minute boost
;		to orbital insertion, verifying velocity approached 25,568 ft/sec (orbital
;		speed) and altitude climbed toward 100 NM target parking orbit.

#	FDAI DISPLAY
;		Attitude error needles: Flight Director Attitude Indicator mechanical
;		ball display plus error needles showing pitch, yaw, roll deviations from
;		desired Saturn V trajectory. Needles driven by NEEDLER subroutine called
;		from P11's attitude monitoring job. During Apollo 11 ascent, needles
;		confirmed vehicle maintained guidance profile within acceptable tolerances,
;		giving crew confidence in automated Saturn guidance system performance.

# ERASABLE INITIALIZATION
;		P11 requires preloaded values in AGC erasable memory (RAM) before
;		execution. Ground uplink or prelaunch initialization loads these
;		launch site and trajectory parameters. Mission Control verified all
;		initialization data correct before Apollo 11 liftoff countdown.

#	AZO, AXO, -AYO
;		Launch azimuth unit vectors: Define local horizontal direction for
;		launch trajectory alignment. For Apollo 11 at Kennedy Space Center
;		Launch Complex 39A, azimuth = 72.058° east of north, establishing
;		trajectory inclination = 31.5° for translunar orbit requirements.

#	LATITUDE
#	PADLONG
;		Launch site geodetic coordinates: LATITUDE = +28.608° N (Launch Complex
;		39A, Kennedy Space Center). PADLONG = -80.604° W (west longitude from
;		Greenwich meridian). These coordinates anchor initial position vector
;		computation via LALOTORV subroutine, converting geographic location to
;		Earth-centered inertial coordinates at liftoff instant.

#	TEPHEM
;		Ephemeris time: Initial reference time for lunar/solar position
;		computations. P11 updates TEPHEM to liftoff moment, synchronizing
;		celestial mechanics calculations to mission timeline origin.

#	PGNCSALT
;		Pad altitude: Launch Complex 39A elevation above reference geoid.
;		Initializes altitude measurement baseline for H (altitude above pad)
;		display during ascent. Apollo 11 launched from ~10 feet above sea level.

#	POLYNUM THRU POLYNUM +14D)
;		Polynomial coefficients: 6th-degree polynomial coefficients (15 values)
;		defining desired pitch and roll angles during Saturn V guidance phase.
;		Ground computed these coefficients matching Saturn Instrument Unit
;		trajectory profile. P11 evaluates polynomials each computation cycle
;		to determine desired attitude for FDAI error needle calculation.

#	RPSTART
#	POLYSTOP
;		Polynomial phase timing: RPSTART = time when roll/pitch polynomial
;		guidance begins (~T+10 seconds after tower clearance). POLYSTOP = time
;		when polynomial guidance ends (~T+133 seconds approaching S-IVB cutoff).
;		Define interval when FDAI needles reference polynomial desired attitude.
# FLAGS SET OR RESET
;		P11 modifies several AGC software flags controlling system behavior:
# Page 535
#	SET REFSMFLG
;		REFSMMAT known flag: Indicates valid REFSMMAT exists in memory. P11
;		computes launch-aligned REFSMMAT and sets this flag, enabling subsequent
;		navigation programs to use established reference frame. Other programs
;		check REFSMFLG before performing attitude transformations.

#	SET DVMON IDLE FLAG
;		Delta-V monitor idle: Signals that Delta-V monitoring (thrust integral
;		tracking) not currently active. During ascent, Saturn V guidance system
;		controls thrust; CMC monitors passively. Flag prevents CMC from attempting
;		active thrust vector control during boost phase.

#	CLEAR ERADFLAG
;		Erase radar flag: Indicates radar data should not be processed. During
;		ascent, no rendezvous or landing radar operations occur. Flag cleared
;		to prevent radar processing routines from attempting to incorporate
;		non-existent radar measurements into navigation state.

# DEBRIS
;		Temporary storage locations ("debris") used by P11 during computation
;		but not preserved across program exit. These erasable memory locations
;		may be reused by subsequent programs. Content has meaning only during
;		P11 execution and should not be referenced by other code after P11 exit.

#	LIFTTEMP
;		Temporary liftoff time storage: Intermediate variable during TLIFTOFF
;		computation and TEPHEM update. Holds time values during transformation
;		between different AGC time representations (centiseconds, double-precision).

#	POLYNUM THRU POLYNUM +7
;		Polynomial work area: Temporary storage for polynomial evaluation
;		(POWRSERS subroutine). Holds intermediate values when computing 6th-
;		degree pitch/roll polynomials. Eight locations accommodate coefficient
;		processing and power series accumulation.

#	SPOLYARG
;		Scaled polynomial argument: Time value scaled appropriately for
;		polynomial evaluation. Converted from mission elapsed time to polynomial
;		domain (typically normalized to 0-1 range over polynomial validity interval).

#	BODY1, BODY2, BODY3
;		Body axes temporary vectors: Intermediate attitude computation storage.
;		Hold transformation matrix rows during REFSMMAT calculation and attitude
;		error processing. Represent spacecraft body axes in various coordinate frames.

#	VMAG2, ALTI, HDOT
;		Navigation display temporaries: VMAG2 = squared velocity magnitude
;		(intermediate for velocity computation), ALTI = altitude temporary
;		(during scaling for display), HDOT = altitude rate temporary (during
;		numerical differentiation). Final values transferred to DSKY registers.

#	CENTRALS, CORE SET AND VAC AREAS
;		Standard AGC temporary storage: CENTRALS = A, L, Q registers and other
;		central processor state saved/restored during interrupts. CORE SET =
;		executive core set locations for job scheduling. VAC AREAS = vector
;		accumulator storage for interpreter mode vector/matrix operations.
;		These areas used throughout P11 for standard AGC computational purposes.

; ============================================================================
; TRANSITION: From Program Description to P11 Main Implementation
;
; The following sections implement the P11 Earth Orbit Insertion Monitor
; program described above. The code executes immediately after liftoff
; (when the Liftoff Discrete is received) or can be initiated manually
; via Verb 75. During Apollo 11's launch on July 16, 1969 at 13:32:00 UTC,
; this program began execution within milliseconds of the Saturn V's
; five F-1 engines igniting. It provided the crew with real-time displays
; of velocity, altitude, and altitude rate as the massive rocket accelerated
; toward Earth orbit insertion approximately 12 minutes after liftoff.
; ============================================================================

		COUNT	34/P11

BITS5-6		=	SUPER011
		BANK	42
		SETLOC	P11ONE
		BANK

		EBANK=	TEPHEM
P11		CA	EBANK3
		TS	EBANK

		EXTEND
		DCA	REP11S		# DIRECT RESTARTS TO REP11
		DXCH	-PHASE3
		CS	ZERO
		ZL
		TS	LIFTTEMP
		DXCH	-PHASE5		# INACTIVE GROUP 5, PRELAUNCH PROTECTION
P11+7		EXTEND
		DCA	REP11SA
		DXCH	TLIFTOFF

		EXTEND
		DCA	TIME2
		DXCH	LIFTTEMP	# FOR RESTARTS

		CA	ZERO
		ZL
		DXCH	TIME2
REP11A-2	DXCH	TLIFTOFF
REP11A-1	DXCH	-PHASE3		# RESET PHASE


REP11A		INHINT
		EXTEND
		DCA	TEPHEM	+1
		DXCH	TEPHEM1	+1
		CA	TEPHEM
# Page 536
		XCH	TEPHEM1

		EXTEND
		DCA	TLIFTOFF
		DAS	TEPHEM1	+1
		ADS	TEPHEM1		# CORRECTFOR OVERFLOW

		TC	PHASCHNG
		OCT	05023
		OCT	22000

		INHINT
		EXTEND
		DCA	TEPHEM1
		DXCH	TEPHEM
		CA	TEPHEM1	+2
		XCH	TEPHEM	+2

		CAF	EBDVCNT
		TS	EBANK
		EBANK=	DVCNTR
		TC	IBNKCALL
		CADR	PREREAD1	# ZERO PIPS AND INITIALIZE AVERAGEG

		TC	PHASCHNG
		OCT	05023		# CONTINUE HERE ON RESTART
		OCT	22000

		CAF	.5SEC		# START ATT ERROR DISPLAY
		TC	WAITLIST	# IN .5 SEC
		EBANK=	BODY3
		2CADR	ATERTASK

		TC	NEWMODEX	# DISPLAY MM 11
		MM	11

		TC	UPFLAG
		ADRES   NODOP01

		CA      POWDNCOD        # SWITCH TO POWERED FLIGHT DOWNLIST
		TS      DNLSTCOD

		TC      BANKCALL
		CADR	CLEANDSP	# CLEAR DSKY IN CASE OF V75

		TC	2PHSCHNG
		OCT	40514		# PROTECT ATERTASK
		OCT	00073
		CAF	EBQPLACE
# Page 537
		TS	EBANK

		EBANK=	QPLACES
		CA	P11XIT		# SET EXIT FROM PROUT IN EARTHR
		TS	QPLACES
		TC      INTPRET
		VLOAD	MXV
			THETAN
			XSM
		VSL1	VAD
			ERCOMP
		STODL	ERCOMP
			TLIFTOFF
		SSP	GOTO
			S2
		CADR	PROUT		# RETURN FROM EARTHR
			EARTHR	+3

; ============================================================================
; TRANSITION: From Initial Setup to Attitude Reference Matrix Initialization
;
; With the CMC clock zeroed and servicer initiated, the program now captures
; the initial IMU gimbal angles (CDU angles) at liftoff. These angles define
; the spacecraft's orientation relative to the stable platform at the moment
; the Saturn V begins its ascent. The stored gimbal angles (OGC, IGC, MGC)
; serve as the reference for computing attitude errors displayed on the FDAI
; needles throughout the powered flight phase. The MATRXJOB routine then
; computes the initial position and velocity vectors (RN, VN) based on the
; launch pad's geodetic coordinates at Kennedy Space Center Launch Complex 39A.
; ============================================================================

MATRXJOB	ZL	                # STORE DP GIMBAL ANGLES FOR ATTITUDE
		CA      CDUX            #       ERROR DISPLAY AFTER LIFTOFF
		DXCH    OGC
		ZL
		CA      CDUY
		DXCH    IGC
		ZL
		CA      CDUZ
		DXCH    MGC
                TC      INTPRET         #       -
		VLOAD   VSR1            # SCALE OGC B-1
			OGC
		STORE   OGC
		SSP			# ZERO RTX2
			RTX2		# FOR
			0		# EARTH
		DLOAD	PDDL
			PGNCSALT	# ALTITUDE OF PGNCS
			PADLONG		# LONGITUDE
		PDDL	VDEF
			LATITUDE	# GEODETIC LATITUDE
		STODL	LAT		# LAT,LONG,ALT ARE CONSECUTIVE
			HI6ZEROS	# TIME = 0
		CLEAR	CALL
			ERADFLAG
			LALOTORV	# CONVERT TO POSITION VECTOR IN REF.COORDS

		STCALL	RN1             #              -
			GETDOWN 	# RETURN WITH VECTOR FOR DOWN DIRECTION
		VCOMP   UNIT
		STOVL	REFSMMAT +12D	# UNITZ = UNIT(GRAV)
			RN1
		VXV	VXSC
# Page 538
			UNITW		# SCALED AT 1
			-ERTHRAT	# V = EARTHRATE X R
		VSL4			# SCALE TO 2(7) M/CS
		STOVL	VN1
			REFSMMAT +12D
		VXV	UNIT
			UNITW		# (REF3 X UNITW) = EAST
		PUSH	VXV
			REFSMMAT +12D	# (EAST X REF3) = -SOUTH
		UNIT	PDDL
			LAUNCHAZ	# COS(AZ)*SOUTH
		COS	VXSC
		STADR
		STODL	REFSMMAT	# TEMPORARY STORAGE
			LAUNCHAZ
		SIN	VXSC		# SIN(AZ)*EAST
		VAD	UNIT		# SIN(AZ)*EAST - COS(AZ)*SOUTH = REF1
			REFSMMAT
		STORE	REFSMMAT

		VXV	UNIT		# (REF1 X REF3) = -REF3
			REFSMMAT +12D
		VCOMP
		STORE	REFSMMAT +6
		DLOAD	DSU
			DPHALF		# 1/2 REV
			LAUNCHAZ
		DAD	PDDL
			AZIMUTH
			SATRLRT		# SET	SATRLRT = -SATRLRT IF
		SIGN	STADR		# (1/2REV -LAVNCHAZ +AZIMUTH) IS NEGATIVE
		STORE	SATRLRT		# FOR ROLL CALC IN FDAI ATT. ERROR DISPLAY
		SET	EXIT
			REFSMFLG	# SET REFSMMAT KNOWN FLAG

		TC	PHASCHNG
		OCT	04023

		EXTEND
		DCA	P11SCADR
		DXCH	AVGEXIT		# SET AVGEXIT

		CA	PRIO31		# 2 SECONDS AT 2(+8)
		TS	1/PIPADT

		EBANK=	RCSFLAGS
		CA	EBANK6
		TS	EBANK

		INHINT
# Page 539
		CS	ZERO
		TS	TBASE5		# RESTART READACCS 2 SECONDS AFTER LIFTOFF

		CS	TIME1
		AD	2SECS		# DO READACCS 2 SECONDS AFTER LIFTOFF

		CCS	A		# CHECK TO INSURE DT IS POSITIVE
		TCF	+3		# TIME POSITIVE
		TCF	+2		# CANNOT GET HERE
		CA	ZERO		# TIME NEGATIVE - SET TO 1
		AD	ONE		# RESTORE TIME  -  OR MAKE POSITIVE

		TC	WAITLIST
		EBANK=	AOG
		2CADR	READACCS

		TC	2PHSCHNG
		OCT	00003		# TURN OFF GROUP 3
		OCT	00025		# PROTECT NORMLIZE AND READACCS

		TC	POSTJUMP
		CADR	NORMLIZE	# DO NORMLIZE AND ENDOFJOB


		EBANK=	TEPHEM
REP11		INHINT
		CCS	PHASE5
		TC	ENDOFJOB

		CCS	LIFTTEMP
		TCF	+4
		TCF	+3
		TCF	+2
		TCF	P11+7

		CS	TLIFTOFF
		EXTEND
		BZMF	ENDREP11

		CCS	TIME2		# **TIME2 MUST BE NON-ZERO AT LIFTOFF**
		TCF	REP11A	-5	# T2,T1 NOT YET ZEROED, GO AND DO IT

		EXTEND			# T2,T1 ZEROED, SET TLIFTOFF
		DCA	LIFTTEMP
		TCF	REP11A-2

ENDREP11	EXTEND
		DCA	REP11SA
		TCF	REP11A-1
# Page 540
REP11S		2OCT	7776600011

REP11SA		2OCT	7776400013

P11XIT		GENADR	P11OUT
-ERTHRAT	2DEC*	-7.292115138 E-7 B18*	# - EARTH RATE AT 2(18)

		EBANK=	BODY3
P11SCADR	2CADR	VHHDOT

POWDNCOD        EQUALS  THREE

; ============================================================================
; TRANSITION: From Initialization Phase to Active Display Generation
;
; With liftoff confirmed, initial vectors computed, and the servicer
; established, P11 now enters its primary operational mode: providing the
; crew with continuous real-time displays of vehicle performance during
; ascent. The VHHDOT routine executes every 2 seconds to compute and
; display three critical parameters on the DSKY (Display and Keyboard):
;   R1: VI (inertial velocity magnitude in feet per second)
;   R2: HDOT (altitude rate in feet per second)  
;   R3: H (altitude above launch pad in nautical miles)
;
; During Apollo 11's ascent on July 16, 1969, astronauts Neil Armstrong,
; Buzz Aldrin, and Michael Collins monitored these displays (Noun 62) as
; confirmation that the Saturn V was performing nominally. At approximately
; T+2 minutes, velocity would read ~2,500 fps climbing toward 25,500 fps at
; Earth orbit insertion 12 minutes after liftoff.
; ============================================================================

		EBANK=	BODY3
# VHHDOT IS EXECUTED EVERY 2 SECONDS TO DISPLAY ON DSKY
#			VI INERTIAL VELOCITY MAGNITUDE
#			HDOT  RATE OF CHANGE OF ALT ABOVE L PAD RADIUS
#			H  ALTITUDE ABOVE L PAD RADIUS

VHHDOT		TC	INTPRET
		CALL			# LOAD VMAGI, ALTI,
			S11.1		# HDOT FOR DISPLAY
		EXIT
		TC      PHASCHNG
		OCT     00035

		CAF	V06N62		# DISPLAY IN R1	R2	R3
		TC	BANKCALL	#            VI	HDOT	H
		CADR	REGODSP

ATERTASK	CAF	PRIO1		# ESTABLISH JOB TO DISPLAY ATT ERRORS
		TC	FINDVAC		# COMES HERE AT L.O. + .33 SEC
		EBANK=	BODY3
		2CADR	ATERJOB

		CS	RCSFLAGS	# SET BIT3 FOR
		MASK	BIT3		# NEEDLER
		ADS	RCSFLAGS	# INITIALIZATION PASS
		TC	IBNKCALL	# AND GO
		CADR	NEEDLER		# DO IT
		CA      BIT1            # SET SW
		TS      SATSW           # FOR DISPLAY
		TC	TASKOVER
GETDOWN         STQ     SETPD
			INCORPEX
			0D
		DLOAD
			HI6ZEROS
# Page 541
		STODL   6D
		        DPHALF
		STCALL  8D
		        LALOTORV +5
# THIS SECTION PROVIDES ATTITUDE ERROR DISPLAYS TO THE FDAI DURING SONE BOOST

#					COMPUTE DESIRED PITCH W.R.T. PAD LOCAL VERTICAL AT LIFTOFF
#							2    3    4    5    6
#					PITCH = A0+A1T+A2T +A3T +A4T +A5T +A6T
#						SCALED TO 32 REVS.                                   -14
#					IF TL = TIME IN SECS FROM L.O., THEN	T = 100(TL-RPSTART)2
#					WHERE	TL GE RPSTART
#						TL LE (-POLYSTOP + RPSTART)
#					COMPUTE DESIRED ROLL WHERE ROLL EQUALS ANGLE FROM
#					LAUNCHAZ TO -Z(S/C) AS SEEN FROM X(S/C).
#					ROLL = LAUNCHAZ-AZIMUTH-.5 +SATRLRT*T	IN REV
#					SATRLRT = RATE OF ROLL IN REV/CENTI-SEC
#					T,IN CENTI-SEC,IS DEFINED AS ABOVE,INCLUSIVE OF TIME RESTRICTIONS

#					FOR SIMPLICITY, LET	P = 2*PI*PITCH
#								R = 2*PI*ROLL

#					CONSTRUCT THE TRANSFORMATION MATRIX, TSMV, GIVING DESIRED S/C AXES IN
#					TERMS OF SM COORDINATES.  LET THE RESULTING ROWS EQUAL THE VECTORS XDC,
#					YDC,AND ZDC.

#					 *     (    SIN(P)                 0               -COS(P)    )   (XDC)
#					TSMV = (-SIN(R)*COS(P)		-COS(R)		-SIN(R)*SIN(P)) = (YDC)
#					       (-COS(R)*COS(P)           SIN(R)         -COS(R)*SIN(P))   (ZDC)

#					XDC,YDC,ZDC ARE USED AS INPUT TO CALCGTA FOR THE EXTRACTION OF THE
#					EULER SET OF ANGLES WHICH WILL BRING THE SM INTO THE DESIRED
#					ORIENTATION.  THIS EULER SET, OGC, IGC, AND MGC, MAY BE IDENTIFIED
#					AS THE DESIRED CDU ANGLES.

#					(XDC)			(OGC)
#					(YDC) ---) CALCGTA ---) (IGC)
#					(ZDC)			(MGC)

#							   -
#					DEFINE THE VECTOR DELTACDU.

#					 -         (OGC)   (CDUX)
#					DELTACDU = (IGC) - (CDUY)
#					           (MGC)   (CDUZ)

#								 -		-    *    -
#					COMPUTE ATTITUDE ERRORS, A, WHERE	A = TGSC*DELTACDU
#
#					 *     (1	      SIN(CDUZ)             0    )   THE GIMBAL ANGLES
# Page 542
#	TGSC = (0	COS(CDUX)*COS(CDUZ)	SIN(CDUX)) = TO SPACECRAFT AXES
#	       (0      -SIN(CDUX)*COS(CDUZ)	COS(CDUX))   CONVERSION MATRIX

#			     -
#	THE ATTITUDE ERRORS, A, ARE STORED ONE HALF SINGLE PRECISION IN
#	THE REGISTERS AK, AK1, AK2 AS INPUT TO NEEDLER, THE FDAI ATTITUDE
#	ERROR DISPLAY ROUTINE.

; ============================================================================
; TRANSITION: From Display Updates to Attitude Error Monitoring
;
; While VHHDOT provides velocity and altitude displays every 2 seconds, the
; ATERJOB routine runs as a separate low-priority job approximately every
; 0.5 seconds to compute and display attitude errors on the Flight Director
; Attitude Indicator (FDAI) needles. This is critical during the first 133
; seconds after liftoff when the Saturn V executes its roll program and
; pitchover maneuver to align the vehicle's trajectory with the orbital
; plane. The crew uses these attitude error needles to verify that the
; vehicle is following the programmed pitch and roll profile specified by
; CMC polynomials. During Apollo 11's launch, this system confirmed proper
; vehicle attitude throughout the dynamic first stage burn and staging
; events, giving the crew confidence in the guidance system's performance.
; ============================================================================

; Attitude error display job entry point. This routine checks mission mode
; flags to determine whether attitude error computation should proceed.
; During manual Saturn steering (astronaut takeover), this job terminates
; to avoid conflicting guidance displays.

ATERJOB		CAE	FLAGWRD6	# CHECK FLAGWRD6
		MASK    OCT60000        # BITS 14, 15
		EXTEND
		BZF     +2              # OK - CONTINUE
		TC      ENDOFJOB	# SATURN STICK ON - KILL JOB

; Check spacecraft control panel switch (CHAN30 bit 10) to determine if
; astronaut has taken control of Saturn V attitude. This switch, located on
; the main display console, allows the crew to override automated guidance
; and manually command vehicle rates during the first two minutes of flight.
; The SATSW flag tracks state changes to properly reinitialize the FDAI
; needles when transitioning between automatic and manual control modes.

		CAF     BIT10           # CHECK IF S/C CONTROL
		EXTEND                  # OF SATURN PANEL
		RAND    CHAN30          # SWITCH IS ON
		EXTEND
		BZF     STRSAT          # IT IS - GO STEER
		CCS     SATSW           # IT IS NOT - WAS IT ON LAST CYCLE
		TC      ATTDISP         # NO - CONTINUE
		TC      ATRESET         # YES - REINITIALIZE NEEDLER
		TC      ATRESET         # YES - REINITIALIZE NEEDLER

; ATTDISP computes desired attitude from Saturn V pitch/roll polynomial
; profile during powered flight (RPSTART to POLYSTOP, approximately +10 to
; +133 seconds after liftoff). The vehicle executes a preprogrammed roll to
; the correct azimuth plane followed by a pitchover maneuver to achieve the
; proper trajectory for Earth orbital insertion. TIME1 is the mission timer
; in centiseconds; subtracting RPSTART gives elapsed time since maneuver
; start. During Apollo 11's ascent, this profile rolled the vehicle to the
; correct heading and gradually pitched it toward horizontal for orbit.

ATTDISP		CS	RPSTART		# PITCH/ROLL START TIME
		AD	TIME1
		EXTEND
		BZMF	NOPOLY		# IF MINUS THEN ATTITUDE HOLD
		TS	MPAC		# MPAC=TIME1-RPSTART
		TS	SPOLYARG	# SAVE FOR USE IN ROLL CALCULATION
		AD	POLYSTOP	# NEG PITCHOVER TIME IN CSECS
		EXTEND
		BZMF	+2
		TC      NOPOLY          # GO TO ATTITUDE HOLD
		CA      TIME2
		EXTEND
		BZMF    +2
		TC      NOPOLY          # GO TO ATTITUDE HOLD

; Evaluate pitch polynomial using power series. The POLYNUM coefficient count
; and COEFPOLY coefficient address define the polynomial that represents the
; vehicle's commanded pitch angle as a function of time. The POWRSERS routine
; (power series evaluator) computes pitch angle scaled to 32 revolutions,
; which is then converted to radians for trigonometric calculations. This
; polynomial was carefully designed to limit aerodynamic loads during the
; transonic phase while achieving the proper pitch angle for orbital insertion.

		CAE	POLYNUM
		TS	L
		CAF	COEFPOLY	# EVALUATE PITCH POLYNOMIAL
		TC	POWRSERS	# SCALED TO 32 REVOLUTIONS

		CA      ZERO            # RETURN WITH PITCH(32REV)
		TS	MODE		# STORED MPAC, MPAC +1
		TC	INTPRET
		SETPD	SL		# 32(PITCH(32REV))=PITCH(REV)
			0
			5
		PUSH			# LET P(RAD)=2*PI*PITCH(REV)
		GOTO
			ATTDISP1	# AROUND SETLOC
# Page 543
#				     *
#	CONSTRUCT SM TO S/C MATRIX, TSMV

		SETLOC	P11TWO
		BANK			# 36 IN COL., 34 IN DISK

		COUNT	36/P11

ATTDISP1	COS	DCOMP
		STODL	14D		# -.5*COS(P)
		SIN
		STODL	10D		# .5*SIN(P)
			ZEROVECS
		STORE	12D		# 0

; Roll angle computation combines several factors: LAUNCHAZ (launch azimuth
; angle from north to the stable member X-axis), AZIMUTH (desired final
; azimuth angle), and SATRLRT (Saturn roll rate in revolutions per centisecond).
; The roll program aligns the vehicle with the orbital plane, which for Apollo
; 11 required rolling to an azimuth of approximately 72 degrees to achieve the
; proper inclination for lunar transfer. The RLTST routine checks whether the
; roll program has completed, transitioning to pitch control only.

#	EVALUATE ROLL = LAUNCHAZ-AZIMUTH-.5+SATRLRT*T
		SLOAD	DMP
			SPOLYARG	# TIME1 - RSPSTART ,CSECS B-14.
			SATRLRT
		SL	DSU
			14D
			DPHALF
		DAD	DSU		# ASSUMING X(SM) ALONG LAUNCH AZIMUTH,
			LAUNCHAZ	# LAUNCHAZ = ANGLE FROM NORTH TO X(SM).
			AZIMUTH		# AZIMUTH = -ANGLE FROM NORTH TO Z(S/C)
		RTB			# DETERMINE IF ROLLOUT
			RLTST		# IS COMPLETED

; Matrix element computation using trigonometric functions of pitch and roll.
; The transformation matrix TSMV is built using cosine and sine values scaled
; by 0.5 for single-precision arithmetic. Matrix elements are computed as
; products of COS(R), SIN(R), COS(P), SIN(P) where R=roll angle, P=pitch angle.
; The interpreter DMP (double-precision multiply) and SL1 (shift left 1 bit)
; instructions maintain proper scaling throughout. Comments show the geometric
; interpretation of each matrix element (e.g., -.5*COS(R)*COS(P) represents
; the contribution to one component of the transformed vector).

ATTDISPR	PUSH	COS		# CONTINUE COMPUTING TSMV
		PUSH			# LET	R(RAD) = 2*PI*ROLL(REV)
		DMP	SL1
			14D
		STODL	22D		# -.5*COS(R)*COS(P)
		DCOMP
		STORE	18D		# -.5*COS(R)
		DMP	SL1
			10D
		STODL	26D		# -.5*COS(R)*SIN(P)
		SIN	PUSH
		STORE	24D		# .5*SIN(R)
		DMP	SL1
			14D
		STODL	16D		# -.5*SIN(R)*COS(P)
		DCOMP
		DMP	SL1
			10D
		STOVL	20D		# -.5*SIN(R)*SIN(P)
			10D

; Extract half-unit direction cosine vectors XDC, YDC, ZDC from the TSMV
; matrix. These three orthogonal vectors represent the spacecraft body axes
; (X, Y, Z) expressed in stable member coordinates. The UNIT instruction
; normalizes each vector to unit length (scaled by 0.5). These half-unit
; vectors become inputs to CALCGTA (calculate gimbal angles) which computes
; the desired IMU gimbal angles corresponding to the commanded spacecraft
; attitude. During Apollo 11's ascent, these vectors continuously defined the
; relationship between the inertial reference frame and the pitching/rolling
; vehicle, enabling precise attitude error computation.

#	FROM TSMV FIND THE HALF UNIT VECTORS XDC,YDC,ZDC = INPUT TO CALCGTA
# Page 544
		UNIT
		STOVL	XDC		# XDC = .5*UNIT(SIN(P),0,-COS(P))
			16D
		UNIT
		STOVL	YDC		# YDC = .5*UNIT(-SIN(R)*COS(P),-COS(R),
			22D		#			-SIN(R)*SIN(P))
		UNIT
		STCALL	ZDC		# ZDC = .5*UNIT(-COS(R)*COS(P),SIN(R),
			CALCGTA		#			-COS(R)*SIN(P))

#	CALL CALCGTA TO COMPUTE DESIRED SM ORIENTATION	OGC,IGC,AND MGC
#				 -          -   -
#	FIND DIFFERENCE VECTOR	DELTACDU = OGC-CDUX

#	ENTER HERE IF ATTITUDE HOLD

; ============================================================================
; TRANSITION: From Polynomial-Based Attitude to Attitude Hold Mode
;
; The NOPOLYM routine is entered when the vehicle is outside the polynomial
; maneuver window (either before RPSTART at ~T+10 seconds or after POLYSTOP
; at ~T+133 seconds). During attitude hold, the desired spacecraft orientation
; is frozen at the IMU gimbal angles stored at liftoff, rather than following
; the dynamic pitch/roll profile. This mode applies during the pre-launch
; vertical hold, the brief period after tower clearance before roll program
; initiation, and after polynomial completion when the vehicle maintains its
; achieved attitude through orbital insertion. The routine computes attitude
; errors by comparing current IMU gimbal angles (CDUX, CDUY, CDUZ) against
; the commanded orientation (OGC, IGC, MGC), transforming these angular
; differences into body-axis error signals for the FDAI error needles. During
; Apollo 11's ascent, this mode held the vehicle steady during critical phase
; transitions, giving the crew confidence in guidance system stability.
; ============================================================================

NOPOLYM		VLOAD	PUSH		#        OGC      IGC
			OGC		# CHANGE IGC  TO  MGC FOR COMPATIBILITY
		PUSH	CALL		#        MGC      OGC
			CDUTRIG		# WITH Y,Z,X ORDER OF CDUSPOT

; Compute DELTACDU = commanded gimbal angles minus actual gimbal angles.
; The V1STO2S routine (vector 1 store to 2s complement) converts the gimbal
; angle differences into properly scaled angular errors. OGC (outer gimbal
; commanded), IGC (inner gimbal commanded), and MGC (middle gimbal commanded)
; represent the desired IMU platform orientation computed from the attitude
; reference stored at liftoff. CDUX, CDUY, CDUZ are the actual gimbal angles
; read from the IMU Coupling Data Units. The difference vector quantifies how
; far the current spacecraft attitude has deviated from the reference attitude.

		VLOAD	RTB		#  -         DPHI     OGC-CDUX ,PD4
			2		# DELTACDU = DTHETA = IGC-CDUY ,   0
			V1STO2S		#            DPSI     MGC-CDUZ ,   2
		STOVL	BOOSTEMP
			ZEROVECS
		STOVL	0
			CDUSPOT
		RTB	RTB
			V1STO2S
			DELSTOR
		STODL	10D
			SINCDUZ

; Transform gimbal angle errors into body-axis attitude errors accounting for
; gimbal geometry. The transformation uses trigonometric functions of current
; gimbal angles (SINCDUZ, COSCDUZ, SINCDUX, COSCDUX) to resolve the platform-
; referenced errors into pitch, yaw, and roll components that correspond to
; the spacecraft body axes. This coordinate transformation is essential because
; the FDAI error needles display attitude deviations in body axes, which the
; crew interprets relative to the vehicle's orientation, not the inertial
; platform. Scaling to 2 revolutions (SR2) matches FDAI display requirements.

		DMP	SL1
			0
		DAD	SR2		# CHANGE SCALE OF AK TO 2REVS
			4
		GOTO
			ATTDISP2

		SETLOC	P11ONE
		BANK
		COUNT	34/P11

; ATTDISP2 performs the final attitude error display computation, formatting
; gimbal angle errors for FDAI needle presentation. This routine receives the
; angular error vector (DPHI, DTHETA, DPSI) and applies a complex trigonometric
; transformation to convert platform-referenced errors into body-axis errors
; that the crew can interpret. The transformation accounts for gimbal coupling
; effects—when one gimbal rotates, the orientation of the other gimbals changes
; relative to the spacecraft body, creating cross-coupling terms in the error
; transformation. The mathematical formulas (visible in original NASA comments)
; compute the effective pitch, yaw, and roll errors the crew would observe on
; the Flight Director Attitude Indicator (FDAI) error needles. During Apollo 11
; ascent, these needles showed Armstrong and Aldrin how accurately the Saturn V
; was following its commanded trajectory, with small deviations indicating normal
; guidance corrections and large deviations potentially signaling guidance problems.

ATTDISP2	STODL	16D		# 16D, .5(DPHI + DTHETA*SIN(CDUZ))
			COSCDUZ

; The following sequence computes cross-coupled gimbal error terms using the
; current IMU gimbal sines and cosines (SINCDUX, COSCDUX, SINCDUZ, COSCDUZ).
; Each computation combines the three angular errors (DPHI outer gimbal,
; DTHETA inner gimbal, DPSI middle gimbal) with trigonometric functions that
; account for how each gimbal's rotation appears when viewed in body axes.
; Results are stored in 16D, 17D, 18D as half-scaled attitude errors ready
; for FDAI display transformation in later routines.

		DMP	PUSH
			0
		DMP	SL1
			COSCDUX
		PDDL	DMP
# Page 545
			SINCDUX
			2
		DAD	SL1
		STADR
		STODL	17D		# 17D,	.5(DTHETA*COS(CDUX)*COS(CDUZ)
		DMP	SL1		#			+DPSI*SIN(CDUX))
			SINCDUX
		PDDL	DMP
			COSCDUX
			2
		DSU	SL1
		STADR
		STORE	18D		# 18D,	.5(-DTHETA*SIN(CDUX)*COS(CDUZ)
		TLOAD			#			+DPSI*COS(CDUX))
			16D

; Store the computed attitude errors in AK (pitch error), AK1 (yaw error), and
; AK2 (roll error). These three components form the complete body-axis attitude
; error vector that will drive the FDAI error needles. The AK variables are the
; final product of all the gimbal angle computations, scaling, and trigonometric
; transformations performed by the attitude display logic.

		STORE	AK		# STORE ATTITUDE ERRORS IN AK,AK1,AK2
		EXIT

; After computing body-axis attitude errors, the program checks the SATSW
; (Saturn takeover switch) to determine operational mode. Three paths exist:
; positive SATSW means display-only mode (show errors but don't apply steering),
; zero means store current errors as bias reference, and negative means active
; steering mode where errors drive attitude commands to the Saturn V guidance.

		CA      SATSW
		CCS     A               # CHK TAKEOVER STATUS
		TC      SATOUT          # POS - DISPLAY ONLY
		TC      AKLOAD          # 0     STORE BIAS

; STEERSAT applies stored bias correction to computed attitude errors, allowing
; the crew to null out known systematic errors before using error signals for
; vehicle steering. During Apollo 11 ascent, this bias correction compensated
; for known IMU alignment errors and platform drift, ensuring steering commands
; reflected true trajectory deviations rather than sensor biases. The RESCALES
; routine then scales the corrected errors to appropriate range for display and
; control system input.

STEERSAT        TC      INTPRET         # NEG   STEER L/V
		TLOAD   TAD
			BIASAK
			AK
		STORE   AK              # AKS = AKS - STORED BIAS
		EXIT
		CA      AK
		TC      RESCALES
		TS      AK
		CA      AK1
		TC      RESCALES
		TS      AK1
		CA      AK2
		TC      RESCALES
		TS      AK2
# DISPLAY ATTITUDE ERRORS ON FDAI VIA NEEDLER

; SATOUT sends the scaled attitude errors (already in AK, AK1, AK2) to the
; Flight Director Attitude Indicator (FDAI) through the NEEDLER display routine.
; During Apollo 11 ascent, this updated the FDAI error needles approximately
; every 0.5 seconds, showing Armstrong and Aldrin how accurately the Saturn V
; followed its programmed trajectory. Small deflections indicated normal guidance
; corrections; large deflections would signal trajectory problems requiring crew
; intervention or abort. The NEEDLER routine handles the hardware interface to
; the electromechanical FDAI instrument in the Command Module main display panel.

SATOUT		TC	BANKCALL
		CADR	NEEDLER

; ATERSET implements the timing control for the attitude error display cycle.
; After sending errors to NEEDLER, this routine delays 0.25 seconds (OCT31 time
; units) before returning to ATERJOB for the next display cycle. The total cycle
; time of approximately 0.56 seconds (execution time + delay) provides smooth
; needle movement on the FDAI while avoiding excessive AGC computational load.
; This update rate was fast enough for crew monitoring during dynamic ascent
; phases but slow enough to preserve AGC capacity for critical guidance computations.

ATERSET		CAF	OCT31		# DELAY .25 SEC
		TC	BANKCALL	# EXECUTION + DELAY =.56SEC APPROX
		CADR	DELAYJOB
		TC	ATERJOB		# END OF ATT ERROR DISPLAY CYCLE

; AKLOAD stores the current attitude error values as bias references for future
; steering computations. When the crew activates bias mode (SATSW = 0), this
; routine captures the current errors (AK pitch, AK1 yaw, AK2 roll) and stores
; them complemented in BIASAK, BIASAK+1, BIASAK+2. Future error computations
; subtract this stored bias, effectively "zeroing" the error needles at the
; current attitude. This allows the crew to compensate for known IMU drift or
; alignment errors, ensuring subsequent steering commands respond to true trajectory
; deviations rather than sensor biases. After storing the bias, SATSW is set
; negative (CS BIT1) to transition to active steering mode, and STEERSAT is
; called to apply the bias correction immediately.

AKLOAD          CS      AK              # STORE AKS
		TS      BIASAK          # INTO BIAS
		CS      AK1             # COMPLEMENTED
		TS      BIASAK +1
# Page 546
		CS      AK2
		TS      BIASAK +2
		CS      BIT1            # SET SW
		TS      SATSW           # TO STEER
		TC      STEERSAT        # GO STEER

; STRSAT performs initialization checking for the NEEDLER display routine. The
; first time attitude errors are displayed after liftoff, NEEDLER requires
; initialization to establish FDAI servo zero positions and configure the display
; hardware interface. STRSAT checks SATSW status: if negative or zero (BZMF),
; NEEDLER has been initialized and the routine branches to ATTDISP to continue
; normal display operations. If positive (first pass after liftoff), the routine
; falls through to ATRESET to perform initialization.

STRSAT		CA      SATSW           # CHECK IF NEEDLER
		EXTEND                  # HAS BEEN INITIALIZED
		BZMF    ATTDISP         # YES - CONTINUE

; ATRESET initializes the NEEDLER display system on first use after liftoff. It
; sets bit 3 in RCSFLAGS to signal NEEDLER initialization status, calls NEEDLER
; to establish servo zero positions and configure hardware interfaces, then delays
; 60 milliseconds (REVCNT = OCT 6) to allow the IMUERRCNTR register to zero and
; FDAI servos to settle. This initialization ensures accurate error needle display
; when the crew begins monitoring trajectory guidance during Saturn V ascent. After
; the 60ms delay, subsequent attitude display cycles proceed through the normal
; ATTDISP path without re-initialization.

ATRESET		CS      RCSFLAGS        # NO - SET
		MASK    BIT3            # INITIALIZATION SW
		ADS     RCSFLAGS        # FOR NEEDLER
		TC      BANKCALL        # AND GO
		CADR    NEEDLER         # DO IT
		CAF     REVCNT          # OCT 6
		TC      BANKCALL        # DELAY JOB
		CADR    DELAYJOB        # 60 MS -WAIT TILL IMUERRCNTR ZEROED
		CCS     SATSW          	# CHECK SW STATUS
		TC      TAKEON          # POS   STEER INIT.
		TC      +1              # 0     RETURN TO DISPLAY
		CA      BIT1            # NEG   RETURN TO DISPLAY
		TS      SATSW           # SW = DISPLAY ON
		CS      BIT9            # DISABLE
		EXTEND                  # SIVB
		WAND    CHAN12          # TAKEOVER
		TC      SATOUT          # DISPLAY

; TAKEON enables Saturn V takeover mode by setting bit 9 of channel 12, which
; signals the Saturn Instrument Unit (IU) that the AGC is assuming guidance
; control. During Apollo 11 ascent, this interface allowed the Command Module
; computer to provide steering commands to the Saturn V guidance system during
; critical trajectory phases. After enabling takeover (WOR CHAN12 sets the
; hardware bit), SATSW is zeroed to indicate NEEDLER initialization is complete
; and attitude error display can proceed normally. This routine is called during
; initialization when the crew activates Saturn takeover through the DSKY,
; establishing the AGC-to-Saturn guidance interface used throughout powered flight.

TAKEON          CAF     BIT9            # ENABLE
		EXTEND                  # SIVB
		WOR     CHAN12          # TAKEOVER
		CA      ZERO            # INDICATE NEEDLER
		TS      SATSW           # WAS INITIALIZED
		TC      SATOUT

; ============================================================================
; TRANSITION: From Saturn takeover initialization to orbital parameter display
;
; With the Saturn guidance interface established, Apollo 11 now begins the
; continuous computation of orbital parameters displayed to the crew every
; 2 seconds throughout ascent. This routine computes the three values shown
; as Noun 62 on the DSKY: velocity magnitude (V1), altitude rate HDOT (R2),
; and altitude above the launch pad (H in R3). During the July 16, 1969 launch,
; Armstrong, Aldrin, and Collins monitored these values as the Saturn V
; accelerated toward the 25,567 ft/sec orbital velocity needed for Earth orbit.
; ============================================================================

; S11.1 computes velocity magnitude, altitude rate (HDOT), and altitude (ALTI)
; for continuous crew display during ascent to Earth orbit. This interpretive
; language routine is called every 2 seconds by the VHHDOT job established at
; liftoff. Using the current navigation state vectors RN (position) and VN
; (velocity) maintained by orbital integration, it computes:
;
; COMMENT-ONLY READERS: This is the calculation behind the three numbers the
; crew watched climb during ascent - velocity increasing toward orbital speed,
; altitude rate showing climb rate, and altitude above Cape Kennedy. These
; displays confirmed to Armstrong, Aldrin, Collins, and Mission Control that
; the Saturn V was delivering Apollo 11 toward its target 103-nautical-mile
; parking orbit as planned.
;
; CODE-ALONG READERS: The routine uses interpretive language instructions for
; vector operations. VLOAD VN loads the velocity vector (scaled 2^7 meters/csec),
; ABVAL computes magnitude (square root of sum of squares of components), stored
; in VMAGI. Then RN position vector is loaded, converted to UNIT vector (dividing
; by magnitude to get direction only), and DOT product with VN gives the radial
; velocity component (velocity along the position radius). This radial velocity,
; scaled by SL1 (shift left 1 = multiply by 2), becomes HDOT altitude rate in
; feet per second. For altitude, the routine checks AMOONFLG - if clear (Earth
; operations), branches to EARTHALT which loads RPAD (Earth radius at pad), takes
; ABVAL of RN to get distance from Earth center, shifts right 2 (divide by 4 for
; scaling), subtracts 36D (Earth equatorial radius constant scaled appropriately),
; and stores result in ALTI (altitude above pad in nautical miles). During Apollo
; 11 ascent, AMOONFLG was clear, so EARTHALT path executed. If the flag were set
; (lunar operations), RLS (lunar sphere radius) would be loaded instead.

S11.1		VLOAD	ABVAL
			VN
		STOVL	VMAGI		# VI	SCALED 2(7) IN METERS/CSEC
			RN
		UNIT    DOT
		        VN
		SL1
		STODL   HDOT
		        RPAD
		BOF     VLOAD
		        AMOONFLG
		        EARTHALT
		        RLS
		ABVAL	SR2
EARTHALT         BDSU
		        36D
		STORE   ALTI
		RVQ

; DELSTOR computes the attitude error (delta CDU angles) between the current IMU
; gimbal angles (stored in MPAC, MPAC+1, MPAC+2 for outer, inner, middle gimbals)
; and the reference attitude taken at liftoff (stored in BOOSTEMP). These delta
; angles represent how far the spacecraft has deviated from its initial orientation,
; which during the first 10 seconds after liftoff should be near zero. After the
; Saturn begins its roll and pitch program, these deltas indicate the attitude
; error that the AGC displays on the FDAI (Flight Director Attitude Indicator)
; needles for the crew to monitor. The computed deltas are stored in the PDL
; (Push-Down List) at locations 0, 2, 4 (outer, inner, middle gimbal errors) for
; subsequent use by the NEEDLER routine that drives the FDAI error needles.

DELSTOR		CA	BOOSTEMP
# Page 547
		EXTEND			# STORE DELTACDU INTO PDL 0,2,4
		MSU	MPAC
		INDEX	FIXLOC
		TS	0
		CA	BOOSTEMP +1
		EXTEND
		MSU	MPAC +1
		INDEX	FIXLOC
		TS	2
		CA	BOOSTEMP +2
		EXTEND
		MSU	MPAC +2
		INDEX	FIXLOC
		TS	4
		TCF	DANZIG

; RLTST determines whether the Saturn V roll maneuver has completed. During the
; first ~10 seconds after liftoff from Pad 39A, the Saturn V performs a roll
; program to align its guidance system with the desired launch azimuth (72 degrees
; east of north for Apollo 11's trajectory toward a 31-degree inclination orbit).
; The test multiplies MPAC (current attitude parameter) by SATRLRT+1 (rollout
; rate parameter). If the signs differ (BZMF branches), the vehicle is still
; rolling. Once signs match, rollout is complete and the roll axis contribution
; to the steering polynomial is zeroed out by loading -MBDYTCTL+2 into MPAC,
; allowing the pitch program to dominate the subsequent steering commands during
; the ascent trajectory that carries Apollo 11 downrange toward orbital insertion.

RLTST		CA	MPAC		# DETERMINE IF ROLLOUT
		EXTEND			# IS COMPLETED
		MP	SATRLRT +1
		EXTEND
		BZMF	DANZIG		# UNLIKE SIGNS STILL ROLLING
		EXTEND			# ROLLOUT COMPLETED
		DCA	MBDYTCTL +2	# ZERO OUT ROLL CONTRIBUTION
		DXCH	MPAC
		TC	DANZIG

; NOPOLY handles attitude hold mode when the Saturn steering polynomials are not
; active. This occurs if the vehicle has passed the polynomial time window or if
; backup attitude hold is commanded. Sets the PDL pointer to 0 and branches to
; NOPOLYM which implements fixed attitude hold using the IMU orientation stored
; at liftoff, rather than the time-varying polynomial attitude commands.

NOPOLY		TC	INTPRET		# COMES HERE IF
		SETPD	GOTO		# ATTITUDE HOLD
			0
			NOPOLYM
COEFPOLY	ADRES	POLYLOC
V06N62		VN	0662

; RESCALES adjusts the AK steering coefficients for the hardware scaling used by
; the Saturn V autopilot. The CMC (Command Module Computer) computes steering
; commands in its internal fixed-point arithmetic scaling, but the Saturn IU
; (Instrument Unit) autopilot expects commands in a different scale. This routine
; multiplies the AK value by SATSCALE, then performs two double-shifts (DDOUBL)
; which multiply by 4, achieving the proper scaling conversion for the Saturn
; guidance interface. Returns via TC Q to the calling routine.

RESCALES        EXTEND                  # RESCALE AK S FOR
		MP      SATSCALE        # NEW HARDWARE
		DDOUBL                  # SCALING FOR
		DDOUBL                  # STEERING
		TC      Q               # SATURN
# SATURN TAKEOVER FUNCTION
# ************************

# PROGRAM DESCRIPTION

#	MOD NUMBER 1
#	MOD BY ELIASSEN

# FUNCTIONAL DESCRIPTION

#	DURING THE COASTING PHASE OF SIVB ATTACHED, THE
#	ASTRONAUT MAY REQUEST SATURN TAKEOVER THROUGH
#	EXTENDED VERB 46 (BITS 13,14 OF DAPDATR1 SET ).
#	THE CMC REGARDS RHC COMMANDS AS BODY-AXES RATE
#	COMMANDS AND IT TRANSMITS THESE TO SATURN AS DC
# Page 548
#	VOLTAGES.  THE VALUE OF THE CONSTANT RATE COMMAND
#	IS 0.5 DEG/SEC.  AN ABSENCE OF RHC ACTIVITY RE-
#	SULTS IN A ZERO RATE COMMAND.

#	THE FDAI ERROR NEEDLES WILL INDICATE THE VALUE
#	OF THE RATE COMMAND.

# CALLING SEQUENCE
#
#	DAPFIG +9D	TC	POSTJUMP
#			CADR	SATSTKON

# SUBROUTINES CALLED

#	ENDEXT
#	IBNKCALL
#	STICKCHK
#	NEEDLER
#	T5RUPT
#	RESUME

# ASTRONAUT REQUESTS

#	ENTRY	- VERB 46 ENTER
#		  (CONDITION - BITS 13, 14 OF DAPDATR1 SET)
#
#	EXIT	- VERB 48 ENTER	(FLASH V06N46)
#		  VERB 21 ENTER	AXXXX ENTER WHERE A=0 OR 1
#		  VERB 34 ENTER
#		  VERB 46 ENTER

# NORMAL EXIT MODE

#		VERB 46 ENTER	(SEE ASTRONAUT ABOVE)

# ALARM OR ABORT EXIT MODES

#	NONE

# OUTPUT

#	SATURN RATES IN CDUXCMD, CDUYCMD, CDUZCMD

# ERASABLE INITIALIZATION

#	DAPDATR1	(BITS 13,14 MUST BE SET)

# DEBRIS

#	CENTRALS
# Page 549
#	CDUXCMD, CDUYCMD, CDUZCMD

; ==============================================================================
; SATURN TAKEOVER FUNCTION - MANUAL ATTITUDE CONTROL BACKUP MODE
; ==============================================================================
;
; This section implements the "Saturn Stick" manual control mode, activated by
; the crew via Verb 46 (V46 E) on the DSKY if the automatic Saturn steering fails
; during ascent. In normal operations, the CMC computes steering commands from
; time-based polynomials (as implemented in the STEERSAT routine above). However,
; if guidance fails or the crew needs manual control, this backup system allows
; the astronauts to command pitch, yaw, and roll attitude rates directly through
; the hand controller (rotation controller, called "the stick" in pilot terminology).
;
; The manual control interface reads the astronaut's hand controller inputs from
; CHAN31 (which encodes stick deflection in three axes), translates these into
; commanded rate values from the SATRATE table, and outputs them as AK, AK1, AK2
; (roll, pitch, yaw rate commands) to the Saturn IU (Instrument Unit) autopilot.
; The NEEDLER routine then displays attitude errors on the FDAI needles so the
; crew can fly closed-loop, similar to flying an aircraft on instruments.
;
; During Apollo 11's actual launch on July 16, 1969, this backup mode was not
; needed - the automatic polynomial steering performed flawlessly through Saturn
; shutdown at 11 minutes 42 seconds mission elapsed time. However, this capability
; was critical for crew confidence, knowing they could manually steer the Saturn V
; to orbit if the guidance computer failed during the 2-minute first-stage burn or
; the subsequent S-II and S-IVB stage burns. Test astronauts validated this mode
; extensively in the Cape Kennedy simulator before committing to flight.

		BANK	43
		SETLOC	EXTVERBS
		BANK

		COUNT	23/STTKE

; SATSTKON is the Saturn Stick-On routine, invoked by the crew via Verb 46 Enter
; (V46 E) on the DSKY to enable manual attitude control. This routine:
; 1. Sets up the T5 interrupt location (T5LOC) to point to REDOSAT initialization
; 2. Sets TIME5 to POSMAX (maximum delay) to trigger immediately
; 3. Sets FLAGWRD6 bits 15,14 to indicate Saturn stick control is active
; 4. Calls ZEROJET to zero all RCS jet command channels and disable T6 clock
; 5. Returns via GOPIN (because we entered via V46 extended verb)
;
; After initialization, control transfers to REDOSAT which sets up the cyclic
; SATSTICK task that runs every 100 milliseconds to read stick inputs and output
; rate commands. The crew would execute V46 E only if automatic guidance failed,
; giving them direct manual control to fly the Saturn V to orbital velocity using
; the hand controller and FDAI (similar to flying an aircraft on instruments).

SATSTKON	EXTEND
		DCA	2REDOSAT
		INHINT
		DXCH	T5LOC
		CAF	POSMAX
		TS	TIME5
		CS	FLAGWRD6	# TURN ON BITS 15,14 OF
		MASK	RELTAB11	# FLAGWRD6
		ADS	FLAGWRD6	#	SATSTICK CONTROL OF T5
		TC	IBNKCALL	# ZERO JET CHANNELS IN 14 MS AND THEN
		CADR	ZEROJET		# LEAVE THE T6 CLOCK DISABLED
		RELINT
		TC	GOPIN		# EXIT THUS BECAUSE WE CAME VIA V46

		EBANK=	BODY3
2REDOSAT	2CADR	REDOSAT


		SBANK=  LOWSUPER
		BANK	32
		SETLOC	P11FOUR
		BANK

; REDOSAT re-initializes the Saturn manual control system. This routine is executed
; both when the crew first activates manual mode via V46 and also if a restart
; occurs while in manual control mode (the "ALSO COMES HERE FOR RESTARTS" comment
; indicates restart protection). The routine:
; 1. Saves the interrupt bank register and return address (standard T5RUPT entry)
; 2. Sets RCSFLAGS bit 3 for NEEDLER initialization (enables FDAI needle display)
; 3. Calls NEEDLER to disable IMU error counter accumulation (switches to rate mode)
; 4. Sets CHAN12 bit 9 to enable S-IVB stage manual takeover in the IU autopilot
; 5. Sets up the T5 cyclic interrupt to call SATSTICK every 100 milliseconds
; 6. Returns via RESUME to restore interrupted task context
;
; The SIVB (S-IVB stage) takeover enable bit tells the Saturn IU that the CMC is
; commanding rate values, not attitude errors, so the IU autopilot should drive
; engine gimbals and RCS thrusters to achieve the commanded rates. This is the
; interface that allows astronaut stick inputs to control the massive Saturn V.

REDOSAT		LXCH	BANKRUPT	# ALSO COMES HERE FOR RESTARTS
		EXTEND
		QXCH	QRUPT
		CS	RCSFLAGS	# TURN ON BIT3 OF RCSFLAGX
		MASK	BIT3		# FOR
		ADS	RCSFLAGS	# NEEDLER INITIALIZATION
		TC	IBNKCALL
		CADR	NEEDLER		# DISABLE IMU ERR COUNTERS ETC.
		CAF	BIT9		# SIVB
		EXTEND			# TAKEOVER
		WOR	CHAN12		# ENABLE
		EXTEND			# SET UP T5 CYCLE
		DCA	2SATSTCK
		DXCH	T5LOC
		CAF	100MST5		# IN 100 MSECS
		TS	TIME5
		TCF	RESUME		# END OF SATURN STICK INITIALIZATION

; SATSTICK is the cyclic task executed every 100 milliseconds (10 Hz update rate)
; while in manual control mode. This routine reads the astronaut's hand controller
; position from CHAN31, determines which rate table entries to use based on stick
; deflection direction and magnitude, and outputs the commanded rates to AK (roll),
; AK1 (pitch), and AK2 (yaw) for the Saturn IU autopilot to execute. The routine:
;
; 1. Saves interrupt context (BANKRUPT, QRUPT) - standard T5RUPT entry sequence
; 2. Re-arms the T5 interrupt to fire again in 100 msec (maintains 10 Hz cycle)
; 3. Reads CHAN31 and exclusive-ORs with STIKBITS to detect stick deflections
; 4. Calls STICKCHK to set rate indices (RMANNDX, PMANNDX, YMANNDX) based on
;    stick position, determining which entries in the SATRATE table to use
; 5. Loads SATRATE values indexed by RMANNDX → AK (roll rate command)
; 6. Loads SATRATE values indexed by PMANNDX → AK1 (pitch rate command)  
; 7. Loads SATRATE values indexed by YMANNDX → AK2 (yaw rate command)
; 8. Calls NEEDLER to output AK/AK1/AK2 to Saturn IU and update FDAI needles
; 9. Returns via RESUME to the interrupted task
;
; The SATRATE table contains pre-computed rate values corresponding to different
; stick deflection levels (null, small, medium, large deflection in each axis).
; The crew can thus command smooth rate variations by deflecting the hand controller,
; with the FDAI needles showing attitude errors so they can fly closed-loop. The
; 10 Hz update rate (100 millisecond cycle) provides responsive manual control
; while not overloading the AGC or the Saturn IU autopilot control loops.

#	THIS SECTION IS EXECUTED EVERY 100 MSECS
# Page 550
SATSTICK	LXCH	BANKRUPT
		EXTEND
		QXCH	QRUPT

		CAF	2SATSTCK	# SET UP RUPT
		TS	T5LOC		# LO ORDER LOC SET
		CAF	100MST5		# 100 MSECS
		TS	TIME5
		CAF	STIKBITS
		EXTEND
		RXOR	CHAN31		# CHECK IF MAN ROT BITS SAME
		MASK	STIKBITS
		TC	IBNKCALL	# SET RATE INDICES
		CADR	STICKCHK	# FOR PITCH YAW AND ROLL

		INDEX	RMANNDX		# SET SATURN RATES
		CA	SATRATE
		TS	AK		#		ROLL
		INDEX	PMANNDX
		CA	SATRATE
		TS	AK1		#		PITCH
		INDEX	YMANNDX
		CA	SATRATE
		TS	AK2		#		YAW

		TC	IBNKCALL	# FOR SATURN INTERFACE AND FDAI DISPLAY
		CADR	NEEDLER
		TCF	RESUME		# END OF SATURN STICK CONTROL

STIKBITS	OCT	00077
100MST5		DEC	16374
		EBANK=	BODY3
2SATSTCK	2CADR	SATSTICK
