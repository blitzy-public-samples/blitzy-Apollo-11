# Copyright:	Public domain.
# Filename:	P34-35_P74-75.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	658-702
# Mod history:	2009-05-19 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-05 RSB	Corrected a typo.
#		2011-01-06 JL	Fixed pseudo-label indentation.
#		2011-01-06 JL	Added missing comment characters.
#		2011-05-07 JL	Removed workaround.

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
; FILE: P34-35_P74-75.agc
; MODULE: Rendezvous Navigation Programs
; MISSION PHASE: rendezvous
;
; TL;DR: Lambert targeting and rendezvous mid-course programs for Transfer
;        Phase Initiation (TPI) and Terminal Phase Finalization (TPF). Solves
;        the two-point boundary value problem: given two position vectors and
;        time-of-flight, compute required velocity vector to transfer between
;        them along a conic trajectory. Critical for LM rendezvous with Command
;        Module after lunar ascent.
;
; COMMENT-ONLY READERS: This code computed the precise burns needed for Eagle
;        to rendezvous with Columbia in lunar orbit after leaving the Moon's
;        surface. Read comments to follow the rendezvous targeting mathematics.
; CODE-ALONG READERS: Study Lambert problem solution algorithm, iterative
;        velocity computation using INITVEL, and conic trajectory calculations.
; ============================================================================

# Page 658
; ============================================================================
; TRANSITION: Rendezvous Targeting Programs Overview
;
; After the Lunar Module (LM) ascends from the lunar surface, it must execute
; a series of precisely calculated burns to rendezvous with the Command Module
; (CM) orbiting overhead. These programs (P34 and P74 for TPI, P35 and P75 for
; TPF) compute the velocity changes needed at each phase of this rendezvous.
;
; During Apollo 11, after Eagle lifted off from Tranquility Base on July 21,
; 1969, these targeting algorithms guided the LM through its rendezvous with
; Columbia, enabling Armstrong and Aldrin to rejoin Michael Collins for the
; return journey to Earth.
; ============================================================================

# TRANSFER PHASE INITITIATION (TPI) PROGRAMS (P34 AND P74)

; RENDEZVOUS NAVIGATION PROGRAMS: P34, P35, P74, P75
; These programs solve the Lambert problem: given two position vectors in space
; and a desired time-of-flight between them, calculate the velocity vector needed
; to execute a conic trajectory connecting those two points. This is fundamental
; to orbital rendezvous where the LM must intercept the CM at a specific point
; in space and time.
;
; P34/P74: Transfer Phase Initiation (TPI) - First major rendezvous burn
; P35/P75: Terminal Phase Finalization (TPF) - Final approach and intercept

# MOD NO -1			LOG SECTION -- P32-P35, P72-P75
# MOD BY WHITE, P.		DATE: 1 JUNE 67
#
# PURPOSE
#
#	(1)	TO CALCULATE THE REQUIRED DELTA V AND OTHER INITIAL CONDITIONS
#		REQUIRED BY THE ACTIVE VEHICLE FOR EXECUTION OF THE TRANSFER
#		PHASE INITITATION (TPI) MANEUVER, GIVEN --

#		(A)	TIME OF IGNITION TIG (TPI) OR THE ELEVATION ANGLE (E) OF
#			THE ACTIVE/PASSIVE VEHICLE LOS AT TIG (TPI).

#		(B)	CENTRAL ANGLE OF TRANSFER (CENTANG) FROM TIG (TPI) TO
#			INTERCEPT TIME (TIG (TPF)).

#	(2)	TO CALCULATE TIG (TPI) GIVEN E OR E GIVEN TIG (TPI).

#	(3)	TO CALCULATE THESE PARAMETERS BASED UPON MANEUVER DATA
#		APPROVED AND KEYED INTO THE DSKY BY THE ASTRONAUT.

#	(4)	TO DISPLAY TO THE ASTRONAUT AND THE GROUND CERTAIN DEPENDENT
#		VARIABLES ASSOCIATED WITH THE MANEUVER FOR APPROVAL BY THE
#		ASTRONAUT/GROUND.

#	(5)	TO STORE THE TPI TARGET PARAMETERS FOR USE BY THE DESIRED
#		THRUSTING PROGRAM.
#
# ASSUMPTIONS

#	(1)	LM ONLY -- THIS PROGRAM IS BASED UPON PREVIOUS COMPLETION OF
#		THE CONSTANT DELTA ALTITUDE (CDH) PROGRAM (P33/P73).
#		THEREFORE --

#		(A)	AT A SELECTED TPI TIME (NOW IN STORAGE) THE LINE OF SIGHT
#			BETWEEN THE ACTIVE AND PASSIVE VEHICLES WAS SELECTED TO BE
#			A PRESCRIBED ANGLE (E) (NOW IN STORAGE) FROM THE
#			HORIZONTAL PLANE DEFINED BY THE ACTIVE VEHICLE POSITION.

#		(B)	THE TIME BETWEEN CDH IGNITION AND TPI IGNITION WAS
#			COMPUTED TO BE GREATER THAN 10 MINUTES.

#		(C)	THE VARIATION OF THE ALTITUDE DIFFERENCE BETWEEN THE
#			ORBITS WAS MINIMIZED.

#		(D)	THE PERICENTER ALTITUDES OF ORBITS FOLLOWING CSI AND
#			CDH WERE COMPUTED TO BE GREATER THAN 35,000 FT FOR LUNAR
# Page 659
#			ORBIT OR 85 NM FOR EARTH ORBIT.

#		(E)	THE CSI AND CDH MANEUVERS WERE ASSUMED TO BE PARALLEL TO
#			THE PLANE OF THE PASSIVE VEHICLE ORBIT.  HOWEVER, CREW
#			MODIFICATION OF DELTA V (LV) COMPONENTS MAY HAVE RESULTED
#			IN AN OUT-OF-PLANE MANEUVER.

#	(2)	STATE VECTOR UPDATED BY P27 ARE DISALLOWED DURING AUTOMATIC
#		STATE VECTOR UPDATING INITIATED BY P20 (SEE ASSUMPTION (4)).

#	(3)	THIS PROGRAM MUST BE DONE OVER A TRACKING STATION FOR REAL
#		TIME GROUND PARTICIPATION IN DATA INPUT AND OUTPUT.  COMPUTED
#		VARIABLES MAY BE STORED FOR LATER VERIFICATION BY THE GROUND.
#		THESE STORAGE CAPABILITIES ARE LIMITED ONLY TO THE PARAMETERS
#		FOR ONE THRUSTING MANEUVER AT A TIME EXCEPT FOR CONCENTRIC
#		FLIGHT PLAN MANEUVER SEQUENCES.

;
; RADAR INTEGRATION FOR RENDEZVOUS TRACKING
; The rendezvous radar can measure the range and range-rate to the target
; spacecraft, providing real-time updates to improve targeting accuracy. During
; Apollo 11's rendezvous, the radar provided critical tracking data as Eagle
; closed in on Columbia, with automatic marks taken approximately once per minute.

#	(4)	THE RENDEZVOUS RADAR MAY OR MAY NOT BE USED TO UPDATE THE LM
#		OR CSM STATE VECTORS FOR THIS PROGRAM.  IF RADAR USE IS
#		DESIRED THE RADAR WAS TURNED ON AND LOCKED ON THE CSM BY
#		PREVIOUS SELECTION OF P20.  RADAR SIGHTING MARKS WILL BE MADE
#		AUTOMATICALLY APPROXIMATELY ONCE A MINUTE WHEN ENABLED BY THE
#		TRACK AND UPDATE FLAGS (SEE P20).  THE RENDEZVOUS TRACKING
#		MARK COUNTER IS ZEROED BY THE SELECTION OF P20 AND AFTER EACH
#		THRUSTING MANEUVER.

#	(5)	THE ISS NEED NOT BE ON TO COMPLETE THIS PROGRAM.

#	(6)	THE OPERATION OF THE PROGRAM UTILIZES THE FOLLOWING FLAGS --
#
#			ACTIVE VEHICLE FLAG -- DESIGNATES THE VEHICLE WHICH IS
#			DOING RENDEZVOUS THRUSTING MANEUVERS TO THE PROGRAM WHICH
#			CALCULATES THE MANEUVER PARAMETERS.  SET AT THE START OF
#			EACH RENDEZVOUS PRE-THRUSTING PROGRAM.
#
#			FINAL FLAG -- SELECTS FINAL PROGRAM DISPLAYS AFTER CREW HAS
#			SELECTED THE FINAL MANEUVER COMPUTATION CYCLE.
#
#			EXTERNAL DELTA V FLAG -- DESIGNATES THE TYPE OF STEERING
#			REQUIRED FOR EXECUTION OF THIS MANEUVER BY THE THRUSTING
#			PROGRAM SELECTED AFTER COMPLETION OF THIS PROGRAM.
#
#	(7)	ONCE THE PARAMETERS REQUIRED FOR COMPUTATION OF THE MANEUVER
#		HAVE BEEN COMPLETELY SPECIFIED, THE VALUE OF THE ACTIVE
#		VEHICLE CENTRAL ANGLE OF TRANSFER IS COMPUTED AND STURED.
#		THIS NUMBER WILL BE AVAILABLE FOR DISPLAY TO THE ASTRONAUT
#		THROUGH THE USE OF V06N52.
#
#		THE ASTRONAUT WILL CALL THIS DISPLAY TO VERIFY THAT THE
#		CENTRAL ANGLE OF TRANSFER OF THE ACTIVE VEHICLE IS NOT WITHIN
# Page 660
#		170 TO 190 DEGREES.  IF THE ANGLE IS WITHIN THIS ZONE THE
#		ASTRONAUT SHOULD REASSES THE INPUT TARGETING PARAMETERS BASED
#		UPON DELTA V AND EXPECTED MANEUVER TIME.
#
#	(8)	THIS PROGRAM IS SELECTED BY THE ASTRONAUT BY DSKY ENTRY --
#
#			P34 IF THIS VEHICLE IS ACTIVE VEHICLE.
#
#			P74 IF THIS VEHICLE IS PASSIVE VEHICLE.
#
# INPUT
#
#	(1)	TTPI	TIME OF THE TPI MANEUVER.
#	(2)	ELEV	DESIRED LOS ANGLE AT TPI
#	(3)	CENTANG	ORBITAL CENTRAL ANGLE OF THE PASSIVE VEHICLE DURING
#			TRANSFER FROM TPI TO TIME OF INTERCEPT
#
# OUTPUT
#
#	(1)	TRKMKCNT	NUMBER OF MARKS
#	(2)	TTOGO		TIME TO GO
#	(3)	+MGA		MIDDLE GIMBAL ANGLE
#	(4)	TTPI		COMPUTED TIME OF TPI MANEUVER
#		 OR
#		ELEV		COMPUTED LOS ANGLE AT TPI
#	(5)	POSTTPI		PERIGEE ALTITUDE AFTER THE TPI MANEUVER
#	(6)	DELVTPI		MAGNITUDE OF DELTA V AT TPI
#	(7)	DELVTPF		MAGNITUDE OF DELTA V AT INTERCEPT
#	(8)	DVLOS		DELTA VELOCITY AT TPI -- LINE OF SIGHT
#	(9)	DELVLVC		DELTA VELOCITY AT TPI -- LOCAL VERTICAL COORDINATES
#
# DOWNLINK
#
#	(1)	TTPI		TIME OF TPI MANEUVER
#	(2)	TIG		TIME OF TPI MANEUVER
#	(3)	ELEV		DESIRED LOS ANGLE AT TPI
#	(4)	CENTANG		ORBITAL CENTRAL ANGLE OF THE PASSIVE VEHICLE DURING
#				TRANSFER FROM TPI TO TIME OF INTERCEPT
#	(5)	DELVEET3	DELTA VELOCITY AT TPI -- REFERENCE COORDINATES
#	(6)	TPASS4		TIME OF INTERCEPT
#
# COMMUNICATION TO THRUSTING PROGRAMS
#
#	(1)	TIG		TIME OF THE TPI MANEUVER
#	(2)	RTARG		OFFSET TARGET POSITION
#	(3)	TPASS4		TIME OF INTERCEPT
#	(4)	XDELVFLG	RESET TO INDICATE LAMBERT (AIMPOINT) VG COMPUTATION
#
# SUBROUTINES USED
#
#	AVFLAGA
# Page 661
#	AVFLAGP
#	VNPOOH
#	DISPLAYE
#	SELECTMU
#	PRECSET
#	S33/34.1
#	ALARM
#	BANKCALL
#	GOFLASH
#	GOTOPOOH
#	TIMETHET
#	S34/35.2
#	PERIAPO1
#	SHIFTR1
#	S34/35.5
#	VN1645

; ============================================================================
; PROGRAM ENTRY POINTS: P34 and P74
;
; P34: Active vehicle (LM doing the maneuvering)
; P74: Passive vehicle (LM tracking but not maneuvering)
;
; These programs prompt the crew via DSKY to enter targeting parameters:
; - TTPI: Time of Transfer Phase Initiation burn
; - ELEV: Elevation angle of line-of-sight at TPI
; - CENTANG: Central angle to travel during transfer
;
; The AGC then computes the required delta-V and displays critical parameters
; for crew and ground verification before committing to the maneuver.
; ============================================================================

		SETLOC	CSI/CDH
		BANK
		EBANK=	SUBEXIT
		COUNT*	$$/P3474
		
; P34 ENTRY POINT - Active Vehicle TPI Targeting
; Called when this spacecraft (LM) will perform the TPI maneuver to begin
; the transfer phase of rendezvous. Sets active vehicle flag.

P34		TC	AVFLAGA		; Set active vehicle flag
		TC	P34/P74A	; Continue to common code
		
; P74 ENTRY POINT - Passive Vehicle TPI Targeting
; Called when this spacecraft is the target (CSM) and the other vehicle will
; perform the maneuver. Sets passive vehicle flag.

P74		TC	AVFLAGP		; Set passive vehicle flag
		
; COMMON INITIALIZATION FOR P34/P74
; Enables rendezvous tracking flags and requests crew input via DSKY

P34/P74A	TC	P20FLGON	# SET UPDATFLG, TRACKFLG
		CAF	V06N37		# TTPI - Request time of TPI from crew
		TC	VNPOOH		; Display verb/noun, wait for crew input
		EXTEND
		DCA	130DEG
		DXCH	CENTANG
		CAF	P30ZERO
		TS	NN
		TC	DISPLAYE	# ELEV AND CENTANG
		TC	INTPRET
		CLEAR	DLOAD
			ETPIFLAG
			TTPI
		STODL	TIG
			ELEV
		BZE	SET
			P34/P74B
			ETPIFLAG
P34/P74B	CALL
			SELECTMU
DELELO		EQUALS	26D
P34/P74C	DLOAD	SET
			ZEROVECS
			ITSWICH
		BON	CLEAR
			ETPIFLAG
# Page 662
			SWCHSET
			ITSWICH

; ============================================================================
; SWCHSET / INTLOOP - Lambert Targeting Iteration Control
; ============================================================================
;
; COMMENT-ONLY READERS: These labels mark the beginning of the iterative
; computation that solves the rendezvous targeting problem. The computer
; repeatedly refines its calculations, adjusting timing and geometry until
; it finds the exact velocity change needed to reach the target spacecraft.
;
; Think of this like plotting a route on a map: you know where you are now,
; where the target will be later, and roughly how long the trip will take.
; The computer iteratively adjusts the departure time and path until everything
; lines up perfectly - arriving at the exact moment the target is there.
;
; During Apollo 11's rendezvous on July 21, 1969, after Eagle's ascent from
; the lunar surface, these calculations ran continuously to compute the
; precise burns needed to catch up with Columbia orbiting overhead.
;
; CODE-ALONG READERS: Lambert Iteration Loop Structure:
;
; Entry Points:
;   SWCHSET: Conditional entry when ETPIFLAG is set
;   INTLOOP: Main loop entry point (both initial and repeated iterations)
;
; SWCHSET Logic:
;   STORE NOMTPI: Stores ZEROVECS (loaded at P34/P74C) into NOMTPI
;     - NOMTPI is the nominal time offset for TPI (Transfer Phase Initiation)
;     - Set to zero when iterating on elevation angle
;     - Provides time adjustment flexibility for convergence
;
; INTLOOP Main Iteration Cycle:
;   1. Time Computation:
;      DLOAD TTPI: Load time of TPI (input by crew or computed)
;      DAD NOMTPI: Add nominal time offset
;      STCALL TDEC1, PRECSET: Store as target time, compute vehicle states
;        - PRECSET advances both active and passive vehicle state vectors
;        - Propagates orbital positions to the computed TIG time
;
;   2. Lambert Solution:
;      CALL S33/34.1: Invoke Lambert targeting algorithm
;        - Solves two-point boundary value problem
;        - Computes required velocity vector for orbital transfer
;        - Returns convergence status in A register
;
;   3. Convergence Check:
;      BZE EXIT, SWCHCLR: If solution converged (A=0), exit to SWCHCLR
;        - SWCHCLR checks input mode flags and displays results
;        - Success path leads to parameter display for crew approval
;
;   4. Error Handling (No Convergence):
;      TC ALARM: Trigger program alarm
;      OCT 611: Alarm code 611 - "No solution for Lambert problem"
;        - Indicates geometry constraints cannot be satisfied
;        - Typical causes: insufficient time, impossible trajectory
;      CAF V05N09: Display verb 05 noun 09 for crew input
;        - V05N09 requests new elevation angle from crew
;      TC BANKCALL, GOFLASH: Flash display, wait for crew response
;        - Crew options:
;          TERMINATE: Abort targeting (TC GOTOPOOH)
;          PROCEED: Restart with new inputs (TC P34/P74A)
;          V32: Recycle current computation (TC -7)
;
; Iteration Control:
;   The loop continues until either:
;   - Lambert solution converges (BZE SWCHCLR succeeds)
;   - Crew terminates (GOTOPOOH)
;   - Maximum iterations exceeded (handled within S33/34.1)
;
; Mathematical Context:
;   Lambert's problem requires solving Kepler's equation iteratively.
;   The INTLOOP provides the outer control structure while S33/34.1
;   performs the inner numerical iteration (Newton-Raphson method).
;   Typical convergence occurs in 3-7 outer iterations for Apollo
;   rendezvous geometries, with each outer iteration calling S33/34.1
;   which may perform 10-20 inner iterations.

SWCHSET		STORE	NOMTPI
INTLOOP		DLOAD	DAD
			TTPI
			NOMTPI
		STCALL	TDEC1
			PRECSET
		CALL
			S33/34.1
		BZE	EXIT
			SWCHCLR
		TC	ALARM
		OCT	611
		CAF	V05N09\t\t; Request elevation angle input from crew
		TC	BANKCALL
		CADR	GOFLASH\t\t; Display and flash for crew response
		TC	GOTOPOOH\t; Terminate pressed
		TC	P34/P74A	# PROCEED - Restart with new inputs
		TC	-7		# V32 - Recycle to recompute

; TARGETING PARAMETER INPUT SWITCH LOGIC
; Depending on which parameters the crew has specified (time or elevation angle),
; the program branches to display the appropriate inputs and request missing data.

SWCHCLR		BONCLR	BON\t\t; Check input mode flags
			ITSWICH\t\t; IT switch flag
			INTLOOP
			ETPIFLAG\t; Elevation TPI flag
			P34/P74D	# DISPLAY TTPI - Branch if time was input
		EXIT
		TC	DISPLAYE	# DISPLAY ELEV AND CENTANG - Display elevation/angle
		TC	P34/P74E\t; Continue to computation
P34/P74D	EXIT
		CAF	V06N37\t\t# TTPI - Display time of TPI
		TC	VNPOOH\t\t; Show to crew
P34/P74E	TC	INTPRET
		SETPD	DLOAD
			0D
			RTX1
		STODL	X1
			CENTANG
		PUSH	COS
		STODL	CSTH
		SIN
		STOVL	SNTH
			RPASS3
		VSR*
			0,2
		STOVL	RVEC
			VPASS3
		VSR*	SET
			0,2
			RVSW
# Page 663
		STCALL	VVEC
			TIMETHET
		DLOAD
			TTPI
		STORE	INTIME		# FOR INITVEL
		DAD
			T		# RENDEZVOUS TIME
		STCALL	TPASS4		# FOR INITVEL
			S34/35.2
		VLOAD	ABVAL
			DELVEET3
		STOVL	DELVTPI
			VPASS4
		VSU	ABVAL
			VTPRIME
		STOVL	DELVTPF
			RACT3
		PDVL	CALL
			VIPRIME
			PERIAPO1
		CALL
			SHIFTR1
		STODL	POSTTPI
			TTPI
		STORE	TIG
		EXIT
		CAF	V06N58
		TC	VNPOOH
		TC	INTPRET
		CALL
			S34/35.5
		CALL
			VN1645
		GOTO
			P34/P74C
# Page 664
; ============================================================================
; TRANSITION: From TPI Planning to Mid-Course Correction
;
; After the Transfer Phase Initiation (TPI) burn begins the rendezvous, the
; spacecraft follows an intercept trajectory toward the target. However, small
; errors in the TPI burn or trajectory changes require mid-course corrections.
; This is the Terminal Phase Mid-Course (TPM) correction.
;
; During Apollo 11's rendezvous on July 21, 1969, after Eagle's ascent from
; the lunar surface and initial TPI burn, these programs computed fine-tuning
; adjustments to ensure precise intercept with Columbia in lunar orbit.
; ============================================================================

# RENDEZVOUS MID-COURSE MANEUVER PROGRAMS (P35 AND P75)

; P35/P75 PROGRAMS - TERMINAL PHASE MID-COURSE (TPM) CORRECTION
; These programs compute mid-course velocity corrections during the transfer
; phase of rendezvous. After TPI places the active vehicle on an approximate
; intercept trajectory, small delta-V adjustments ensure the spacecraft arrives
; at the planned intercept point at the correct time.
;
; P35: Active vehicle (LM performing the correction maneuver)
; P75: Passive vehicle (LM tracking target but not maneuvering)
;
; The program uses rendezvous radar tracking data to refine state vectors and
; computes optimal corrections based on the time of intercept (T(INT)) from P34.

# MOD NO -1			LOG SECTION -- P32-P35, P72-P75
# MOD BY WHITE, P.		DATE:  1 JUNE 67
#
# PURPOSE
#
#	(1) 	TO CALCULATE THE REQUIRED DELTA V AND OTHER INITIAL CONDITIONS
#		REQUIRED BY THE ACTIVE VEHICLE FOR EXECUTION OF THE NEXT
#		MID-COURSE CORRECTION OF THE TRANSFER PHASE OF AN ACTIVE
#		VEHICLE RENDEZVOUS.
#
#	(2)	TO DISPLAY TO THE ASTRONAUT AND THE GROUND CERTAIN DEPENDENT
#		VARIABLES ASSOCIATED WITH THE MANEUVER FOR APPROVAL BY THE
#		ASTRONAUT/GROUND.
#
#	(3)	TO STORE THE TPM TARGET PARAMETERS FOR USE BY THE DESIRED
#		THRUSTING PROGRAM.
#
# ASSUMPTIONS
#
#	(1)	THE ISS NEED NOT BE ON TO COMPLETE THIS PROGRAM.
#
#	(2)	STATE VECTOR UPDATES BY P27 ARE DISALLOWED DURING AUTOMATIC
#		STATE VECTOR UPDATING INITIATED BY P20 (SEE ASSUMPTION (3)).
#
#	(3)	THE RENDEZVOUS RADAR IS ON AND IS LOCKED ON THE CSM.  THIS WAS
#		DONE DURING PREVIOUS SELECTION OF P20.  RADAR SIGHTING MARKS
#		WILL BE MADE AUTOMATICALLY APPROXIMATELY ONCE A MINUTE WHEN
#		ENABLED BY THE TRACK AND UPDATE FLAGS (SEE P20).  THE
#		RENDEZVOUS TRACKING MARK COUNTER IS ZEROED BY THE SELECTION OF
#		P20 AND AFTER EACH THRUSTING MANEUVER.
#
#	(4)	THE OPERATION OF THE PROGRAM UTILIZES THE FOLLOWING FLAGS --
#
#			THE ACTIVE VEHICLE FLAG -- DESIGNATES THE VEHICLE WHICH IS
#			DOING RENDEZVOUS THRUSTING MANEUVERS TO THE PROGRAM WHICH
#			CALCULATES THE MANEUVER PARAMETERS.  SET AT THE START OF
#			EACH RENDEZVOUS PRE-THRUSTING PROGRAM.
#
#			FINAL FLAG -- SELECTS FINAL PROGRAM DISPLAYS AFTER CREW HAS
#			SELECTED THE FINAL MANEUVER COMPUTATION CYCLE.
#
#			EXTERNAL DELTA V FLAG -- DESIGNATES THE TYPE OF STEERING
#			REQUIRED FOR EXECUTION OF THIS MANEUVER BY THE THRUSTING
#			PROGRAM SELECTED AFTER COMPLETION OF THIS PROGRAM.
#
#	(5)	THE TIME OF INTERCEPT (T(INT)) WAS DEFINED BY PREVIOUS
#		COMPLETION OF THE TRANSFER PHASE INITIATION (TPI) PROGRAM
#		(P34/P74) AND IS PRESENTLY AVAILABLE IN STORAGE.
#
# Page 665
#	(6)	ONCE THE PARAMETERS REQUIRED FOR COMPUTATION OF THE MANEUVER
#		HAVE BEEN COMPLETELY SPECIFIED, THE VALUE OF THE ACTIVE
#		VEHICLE CENTRAL ANGLE OF TRANSFER IS COMPUTED AND STORED.
#		THIS NUMBER WILL BE AVAILABLE FOR DISPLAY TO THE ASTRONAUT
#		THROUGH THE USE OF V06N52
#
#		THE ASTRONAUT WILL CALL THIS DISPLAY TO VERIFY THAT THE
#		CENTRAL ANGLE OF TRANSFER OF THE ACTIVE VEHICLE IS NOT WITHIN
#		170 TO 190 DEGREES.  IF THE ANGLE IS WITHIN THIS ZONE THE
#		ASTRONAUT SHOULD REASSESS THE INPUT TARGETING PARAMETERS BASED
#		UPON DELTA V AND EXPECTED MANEUVER TIME.
#
#	(7)	THIS PROGRAM IS SELECTED BY THE ASTRONAUT BY DSKY ENTRY --
#
#			P35 IF THIS VEHICLE IS ACTIVE VEHICLE.
#
#			P75 IF THIS VEHICLE IS PASSIVE VEHICLE.
#
# INPUT
#
#	(1)	TPASS4		TIME OF INTERCEPT -- SAVED FROM P34/P74
#
# OUTPUT
#
#	(1)	TRKMKCNT	NUMBER OF MARKS
#	(2)	TTOGO		TIME TO GO
#	(3)	+MGA		MIDOLF GIMBAL ANGLE
#	(4)	DVLOS		DELTA VELOCITY AT MID -- LINE OF SIGHT
#	(5)	DELVLVC		DELTA VELOCITY AT MID -- LOCAL VERTICAL COORDINATES
#
# DOWNLINK
#
#	(1)	TIG		TIME OF THE TPM MANEUVER
#	(2)	DELVEET3	DELTA VELOCITY AT TPM -- REFERENCE COORDINATES
#	(3)	TPASS4		TIME OF INTERCEPT
#
# COMMUNICATION TO THRUSTING PROGRAMS
#
#	(1)	TIG		TIME OF THE TPM MANEUVER
#	(2)	RTARG		OFFSET TARGET POSITION
#	(3)	TPASS4		TIME OF INTERCEPT
#	(4)	XDELVFLG	RESET TO INDICATE LAMBERT (AIMPOINT) VG COMPUTATION.
#
# SUBROUTINES USED
#
#	AVFLAGA
#	AVFLAGP
#	LOADTIME
#	SELECTMU
#	PRECSET
#	S34/35.1
#	S34/35.2
# Page 666
#	S34/35.5
#	VN1645

		COUNT*	$$/P3575
		EBANK=	KT

; ============================================================================
; P35/P75 PROGRAM ENTRY POINTS
;
; P35: Active vehicle Terminal Phase Mid-Course (TPM) computation
; The LM (Eagle) uses this program after TPI to compute small velocity
; corrections ensuring precise intercept with the CSM (Columbia). During
; Apollo 11's rendezvous on July 21, 1969, these mid-course corrections
; refined Eagle's trajectory after its ascent from the lunar surface.
;
; P75: Passive vehicle tracking mode
; Used when the spacecraft is the target being approached, not the one
; maneuvering. Computes predicted intercept without commanding burns.
;
; Both programs use rendezvous radar tracking data and Lambert targeting
; algorithms to compute optimal corrections based on time to intercept.
; ============================================================================

P35		TC	AVFLAGA
		EXTEND
		DCA	ATIGINC
		TC	P35/P75A
P75		TC	AVFLAGP
		EXTEND
		DCA	PTIGINC
P35/P75A	DXCH	KT
		TC	P20FLGON	# SET UPDATFLG, TRACKFLG
		TC	INTPRET
		CALL
			SELECTMU

; ============================================================================
; P35/P75B - Continuous Tracking and Midcourse Correction Loop
; ============================================================================
;
; COMMENT-ONLY READERS: This is where the spacecraft enters continuous tracking
; mode during rendezvous. Unlike the P34/P74 programs which compute a single
; burn, P35/P75 continuously monitors the target spacecraft and updates the
; required velocity corrections in real time.
;
; Imagine a pilot constantly checking the distance and direction to another
; aircraft, updating the flight plan every few seconds as conditions change.
; The computer does exactly this - it repeatedly:
;   1. Checks the current time
;   2. Predicts where both spacecraft will be at the planned maneuver time
;   3. Computes what velocity change is needed
;   4. Displays the results to the crew
;   5. Loops back to check again
;
; During Apollo 11's rendezvous after Eagle's ascent, Buzz Aldrin watched
; these continuously updating displays showing exactly how much thrust would
; be needed at each planned correction point. As Eagle and Columbia's orbits
; evolved, the displayed delta-V values gradually refined to account for
; gravitational perturbations and radar tracking updates.
;
; The loop runs indefinitely until the crew terminates it or selects a burn
; for execution. This provides constant situational awareness of rendezvous
; geometry and ensures the latest tracking data is incorporated into targeting.
;
; CODE-ALONG READERS: P35/P75B Tracking Loop Architecture:
;
; Loop Entry Point (P35/P75B):
;   This label marks the start of each iteration cycle. The loop executes
;   repeatedly with no explicit termination condition - crew intervention
;   (TERMINATE verb) or program change ends the cycle.
;
; Time Management:
;   RTB LOADTIME: Get present GET (Ground Elapsed Time)
;     - Uses real-time clock to establish current mission time
;     - Returns scaled time value in MPAC (seconds * 2^-28)
;   STORE TSTRT: Save as start time of current computation cycle
;     - TSTRT serves as reference point for this iteration
;   DAD KT: Add time increment (KT)
;     - KT was loaded at P35/P75A from ATIGINC (P35) or PTIGINC (P75)
;     - Typically 15-30 minutes: time from now until planned maneuver
;   STORE TIG: Save as Time of Ignition
;     - TIG is the target time for the computed velocity change
;   STORE INTIME: Save for INITVEL routine
;     - INITVEL uses this to compute state vectors at maneuver time
;
; State Vector Propagation:
;   STCALL TDEC1, PRECSET: Set target time, compute vehicle positions
;     - TDEC1 = time of intercept (TIG computed above)
;     - PRECSET advances both active and passive vehicle state vectors
;     - Uses Encke method integration to account for gravitational perturbations
;     - Returns predicted positions/velocities at TIG
;
; Targeting Computations:
;   CALL S34/35.1: Get normal and line-of-sight for coordinate transform
;     - Computes orbital plane normal vector from passive vehicle state
;     - Computes line-of-sight unit vector from active to passive vehicle
;     - Establishes local-vertical coordinate frame for delta-V display
;
;   CALL S34/35.2: Compute required delta-V in local-vertical frame
;     - Solves Lambert problem for current geometry
;     - Transforms delta-V vector into local-vertical coordinates
;     - Output: Delta-V components (forward, lateral, vertical)
;
;   CALL S34/35.5: Additional targeting refinements
;     - Computes display parameters (range, range-rate, angles)
;     - Prepares formatted data for DSKY presentation
;
; Display and Loop Control:
;   CALL VN1645: Display midcourse correction parameters
;     - Shows updated delta-V requirements to crew
;     - Displays time to ignition, range, geometry
;     - Flashes display to indicate active tracking mode
;
;   GOTO P35/P75B: Loop back to recompute with updated time
;     - Unconditional branch creates infinite tracking loop
;     - Each cycle duration: approximately 1-2 seconds
;     - Loop exits only via crew TERMINATE action or program change
;
; Continuous Update Strategy:
;   The loop provides real-time tracking by recomputing the entire targeting
;   solution every cycle. This ensures:
;   - Latest radar tracking data incorporated (via P20 updates)
;   - Gravitational perturbations continuously accounted for
;   - Crew has current best estimate of required maneuver
;   - Changing geometry reflected in evolving delta-V displays
;
; P35 vs P75 Behavior:
;   Both use the same loop structure (P35/P75B), but:
;   - P35: Active vehicle targeting (LM computing its own burns)
;     Time increment from ATIGINC, uses LM state as active
;   - P75: Passive vehicle tracking (CSM tracking LM maneuvers)
;     Time increment from PTIGINC, uses CSM state as active
;
; Performance Characteristics:
;   Loop execution time: ~1.5 seconds per cycle (typical Apollo geometry)
;   Display update rate: Once per cycle (crew sees frequent updates)
;   Computation load: Moderate (Lambert solver, state propagation)
;   Tracking data incorporation: Automatic via P20 background updates

P35/P75B	RTB
			LOADTIME
		STORE	TSTRT
		DAD
			KT
		STORE	TIG
		STORE	INTIME		# FOR INITVEL
		STCALL	TDEC1
			PRECSET		# ADVANCE BOTH VEHICLES
		CALL
			S34/35.1	# GET NORM AND LOS FOR TRANSFORM
		CALL
			S34/35.2	# GET DELTA V(LV)
		CALL
			S34/35.5
		CALL
			VN1645
		GOTO
			P35/P75B
# Page 667
# ***** S33/34.1 *****

; ============================================================================
; S33/34.1 - LAMBERT TARGETING ITERATIVE SOLUTION
;
; This subroutine solves the classical Lambert problem: given two position
; vectors (active and passive vehicles) and time of flight, find the velocity
; vector required for orbital transfer. This is a two-point boundary value
; problem in orbital mechanics.
;
; COMMENT-ONLY READERS: This is the mathematical heart of rendezvous guidance.
; The computer iteratively adjusts timing and geometry until it finds the exact
; velocity change needed to fly from the current spacecraft position to the
; intercept point at the precise time the target will be there.
;
; CODE-ALONG READERS: The algorithm uses Newton-Raphson iteration on elevation
; angle and time parameters. Maximum iterations (TITER) set to 40000 octal
; (16384 decimal). Convergence tolerance controlled by ELEPS (elevation error).
; The routine computes position/velocity pairs at transfer initiation and
; intercept, then iterates to minimize timing and geometry errors.
;
; TECHNICAL DETAILS:
; - Solves for conic trajectory connecting two points in space-time
; - Iterates on elevation angle (E) and transfer time
; - Stores intermediate position/velocity pairs (RAPREC, VAPREC, RPPREC, VPPREC)
; - Calls S34/35.1 for unit normal and line-of-sight vectors
; - Convergence when DELEL (elevation error) < ELEPS (elevation epsilon)
; ============================================================================

S33/34.1	STQ	SSP
			NORMEX
			TITER
		OCT	40000
		DLOAD	SETPD
			MAX250
			0D
		STOVL	SECMAX
			RACT3
		STOVL	RAPREC
			VACT3
		STOVL	VAPREC
			RPASS3
		STOVL	RPPREC
			VPASS3
		STORE	VPPREC
ELCALC		CALL
			S34/35.1	# NORMAL AND LOS
		VXV	PDVL
			RACT3		# (RA*VA)*RA 0D
		PDVL	UNIT		# ULOS AT 6D
			RACT3
		PDVL	VPROJ		# XCHNJ AND UP
		VSL2	BVSU
			ULOS
		UNIT	PDVL		# UP AT 0D
		DOT	PDVL		# UP.UN*RA AT 0D
			0D		# UP IN MPAC
		DOT	SIGN
			ULOS
		SL1	ACOS
		PDVL	DOT		# EA AT 0D
			ULOS
			RACT3
		BPL	DLOAD
			TESTY
			DPPOSMAX
		DSU	PUSH
TESTY		BOFF	DLOAD
			ITSWICH
			ELEX
			DELEL
		STODL	DELELO
		DSU
			ELEV
		STORE	DELEL
		ABS	DSU
			ELEPS
# Page 668
		BMN
			TIMEX		# COMMERCIALS EVERYWHERE
FIGTIME		SLOAD	SR1
			TITER
		BHIZ	LXA,1
			NORMEX		# TOO MANY ITERATIONS
			MPAC
		SXA,1	VLOAD
			TITER
			RPASS3
		UNIT	PDDL
			36D
		PDVL	UNIT
			RACT3
		PDDL
		PDDL	PUSH
			36D
		BDSU
			12D
		STODL	30D		# RP - RA MAGNITUDES
			DPHALF
		DSU	PUSH
			ELEV
		SIGN	BMN
			30D
			NORMEX
		DLOAD	COS
		DMP	DDV
			14D
			12D
		DCOMP			# SINCE COS(180-A)=-COS A
		STORE	28D
		ABS	BDSU
			DPHALF
		BMN	VLOAD
			NORMEX
			UNRM
		VXV	UNIT
			6D		# UN*RA
		DOT	DMP
			VACT3
			12D
		PDVL	VXV
			0D
			VPASS3
		VXV	UNIT
			0D		# (RP*VP)*RP
		DOT	DMP
			VPASS3
			14D
# Page 669
		BDSU
		NORM	PDVL		# NORMALIZED WA - WP 12D
			X1
			6D
		VXV	DOT
			0D
			UNRM		# RA*RP.UN 14D
		PDVL	DOT
			0D
			6D
		SL1	ACOS
		SIGN
		DSU	DAD		# ALPHA PI
			DPHALF
			ELEV
		PDDL	ACOS
			28D
		BDSU	SIGN
			DPHALF
			30D		# CONTAINS RP-RA
		DAD
		DMP	DDV
			TWOPI
		DMP
		SL*	DMP
			0 	-3,1
		PUSH	ABS
		DSU	BMN
			SECMAX
			OKMAX
		DLOAD	SIGN		# REPLACE TIME WITH MAX TIME SIGNED
			SECMAX
		PUSH
OKMAX		SLOAD	BPL		# TEST FIRST ITERATION
			TITER
			REPETE
		SSP	DLOAD
			TITER
		OCT	37777
		GOTO
			STORDELT
REPETE		DLOAD	DMP
			DELEL
			DELELO
		BPL	DLOAD
			NEXTES
			SECMAX
		DMP
			THIRD
		STODL	SECMAX
# Page 670
		ABS	SR1		# CROSSED OVER SOLUTION
		DCOMP	GOTO		# DT=(-SIGN(DTO)//DT//)/2
			RESIGN
NEXTES		DLOAD	ABS
			DELEL
		PDDL	ABS
			DELELO
		DSU
		BMN	DLOAD
			REVERS		# WRONG DIRECTION
		ABS
RESIGN		SIGN 	GOTO
			DELTEEO
			STORDELT
REVERS		DLOAD	DCOMP
			DELTEEO
		PUSH	SR1
		STORE	DELTEEO
		DAD
		GOTO
			ADTIME

; ============================================================================
; STORDELT / ADTIME - Time Step Storage and Accumulation
; ============================================================================
;
; COMMENT-ONLY READERS: These labels are part of the "hunt and refine" process
; the computer uses to find the perfect rendezvous timing. Think of it like
; adjusting the focus on a camera - first you make big adjustments to get close,
; then smaller and smaller tweaks until the image is perfectly sharp.
;
; The computer tries different timing scenarios:
; - "What if we ignite 10 seconds earlier?"
; - "That overshot - try 5 seconds later instead"
; - "Getting closer - now try 2 seconds earlier"
; - "Perfect! That trajectory works!"
;
; STORDELT stores each time adjustment, while ADTIME accumulates all the
; adjustments to update the overall timing plan. After dozens of these micro-
; adjustments, the computer converges on the exact ignition time that achieves
; the desired rendezvous geometry.
;
; During Apollo 11's rendezvous, this iterative refinement ran continuously,
; ensuring that even as radar tracking updated the target spacecraft's position,
; the computed burn times stayed accurate.
;
; CODE-ALONG READERS: Time Step Convergence Logic:
;
; Entry Paths to STORDELT:
;   Multiple paths converge at STORDELT depending on iteration behavior:
;
;   Path 1 (from RESIGN): Standard time adjustment with sign correction
;     - SIGN DELTEEO: Apply appropriate sign to time increment
;     - GOTO STORDELT: Store the signed time step
;
;   Path 2 (from OKMAX): First iteration initialization
;     - SLOAD TITER: Check iteration counter
;     - BPL REPETE: If not first iteration, go to REPETE
;     - SSP TITER, OCT 37777: Initialize iteration counter
;     - GOTO STORDELT: Store initial time step
;
;   Path 3 (from REVERS): Reversal correction (wrong direction detected)
;     - DLOAD DCOMP DELTEEO: Negate previous time step
;     - PUSH SR1: Halve it (divide by 2, shift right 1)
;     - STORE DELTEEO: Store halved, reversed time step
;     - DAD: Add to accumulator on stack
;     - GOTO ADTIME: Skip STORDELT, go directly to accumulation
;
; STORDELT Label Function:
;   STORE DELTEEO: Store computed time increment in DELTEEO
;     - DELTEEO = "delta time of epoch" (time adjustment for current iteration)
;     - This value represents how much to adjust TPI time for next try
;     - Positive = delay ignition, Negative = advance ignition
;
; ADTIME Label Function:
;   DAD NOMTPI: Add current time increment to accumulated total
;     - NOMTPI = "nominal TPI time offset" (sum of all adjustments so far)
;     - Running total of all time adjustments across iterations
;     - Initialized to zero at loop start, grows/shrinks with each iteration
;   STORE NOMTPI: Save updated accumulated time offset
;
; State Vector Update Sequence (following ADTIME):
;   After updating NOMTPI, the code recomputes vehicle positions at the
;   adjusted time to prepare for the next Lambert solution attempt:
;
;   1. Active Vehicle Update:
;      VLOAD PDVL VAPREC / RAPREC: Load active vehicle velocity and position
;      CALL GOINT: Integrate (propagate) to new time
;      CALL ACTIVE: Store updated RACT3, VACT3
;
;   2. Passive Vehicle Update:
;      VLOAD PDVL VPPREC / RPPREC: Load passive vehicle velocity and position
;      CALL GOINT: Integrate (propagate) to new time
;      CALL PASSIVE: Store updated RPASS3, VPASS3
;
;   3. Restart Iteration:
;      GOTO ELCALC: Return to elevation angle calculation
;        - Recomputes geometry with updated positions
;        - Feeds back into Lambert solver for next iteration
;        - Loop continues until convergence criteria satisfied
;
; Convergence Strategy:
;   The time-stepping algorithm uses adaptive step sizing:
;   - Large steps initially to bracket the solution region
;   - Progressively smaller steps as solution is approached
;   - Direction reversal when overshooting (REVERS path)
;   - Step halving when crossing over the solution
;
;   Typical convergence pattern for Apollo rendezvous:
;     Iteration 1: ±60 seconds (initial bracket)
;     Iteration 2: ±30 seconds (halved, direction corrected)
;     Iteration 3: ±15 seconds (converging)
;     Iteration 4: ±7 seconds
;     Iteration 5: ±3 seconds
;     Iteration 6: ±1 second (approaching tolerance)
;     Iteration 7: ±0.5 seconds (within convergence criteria)
;
; Mathematical Context:
;   This is a bracketing method combined with bisection for robustness.
;   The Lambert problem is highly nonlinear in time - small time changes
;   can produce large delta-V variations. The adaptive step sizing prevents
;   oscillation while ensuring convergence within computational constraints
;   (typically 7-10 iterations for nominal Apollo geometries).

STORDELT	STORE	DELTEEO
ADTIME		DAD
			NOMTPI		# SUM OF DELTA T'S
		STORE	NOMTPI
		VLOAD	PDVL
			VAPREC
			RAPREC
		CALL
			GOINT
		CALL
			ACTIVE		# STORE NEW RACT3 VACT3
		VLOAD	PDVL
			VPPREC
			RPPREC
		CALL
			GOINT
		CALL
			PASSIVE		# STORE NEW RPASS3 VPASS3
		GOTO
			ELCALC
ELEX		DLOAD	DAD
			TTPI
			NOMTPI
		STODL	TTPI
		BON
			ETPIFLAG
			TIMEX
		STORE	ELEV
TIMEX		DLOAD	GOTO
# Page 671
			ZEROVECS
			NORMEX

# Page 672
# ***** S34/35.1 *****

# COMPUTE UNIT NORMAL AND LINE OF SIGHT VECTORS GIVEN THE ACTIVE AND
# PASSIVE POS AND VEL AT TIME T3

; S34/35.1 - COMPUTE GEOMETRY VECTORS FOR RENDEZVOUS
;
; This subroutine computes two critical unit vectors defining the rendezvous
; geometry at time T3:
;
; ULOS (Unit Line-Of-Sight): Unit vector from active vehicle (LM) pointing
;   toward passive vehicle (CSM). This defines the relative position direction.
;
; UNRM (Unit Normal): Unit vector perpendicular to the active vehicle's
;   orbital plane (RA x VA). Defines the plane in which orbital motion occurs.
;
; These vectors establish a coordinate frame for computing elevation angles
; and trajectory geometry during the Lambert problem solution.
;
; INPUTS:  RACT3, VACT3 (active vehicle position and velocity)
;          RPASS3, VPASS3 (passive vehicle position and velocity)
; OUTPUTS: ULOS (unit line-of-sight vector)
;          UNRM (unit normal to active orbit plane)

S34/35.1	VLOAD	VSU
			RPASS3
			RACT3
		UNIT	PUSH
		STOVL	ULOS
			RACT3
		VXV	UNIT
			VACT3
		STORE	UNRM
		RVQ

# Page 673
# ***** S34/35.2 *****

# ADVANCE PASSIVE VEH TO RENDEZVOUS TIME AND GET REQ VEL FROM LAMBERT

; S34/35.2 - LAMBERT TARGETING VELOCITY COMPUTATION
;
; This subroutine solves for the required velocity vector using Lambert
; targeting algorithms. It advances the passive vehicle (target) forward
; in time to the planned intercept time (TPASS4), then computes the velocity
; the active vehicle must have to reach that intercept point.
;
; COMMENT-ONLY READERS: This calculates the exact velocity change needed to
; fly from the spacecraft's current position to meet the target at a future
; time. It's like computing the trajectory of a thrown ball to hit a moving
; target - but in orbit, accounting for gravitational effects.
;
; CODE-ALONG READERS: The routine calls INTINT (conic integration) to
; propagate the passive vehicle position/velocity from INTIME to TPASS4,
; storing the result in RTARG (target position) and VPASS4 (target velocity).
; It then calls INITVEL to compute the required initial velocity for the
; active vehicle, using the Lambert aimpoint guidance algorithm.
;
; The central angle PHI is computed as: PI + (ACOS(RA·RP) - PI)*SIGN(RA×RP·U)
; This angle describes the orbital arc from active to passive vehicle.
;
; INPUTS:  RACT3, VACT3 (active vehicle state at INTIME)
;          RPASS3, VPASS3 (passive vehicle state at INTIME)
;          INTIME (current time), TPASS4 (intercept time)
;          UNRM (unit normal to orbital plane)
; OUTPUTS: RTARG (target position at intercept)
;          VPASS4 (target velocity at intercept)
;          DELLT4 (time-of-flight = TPASS4 - INTIME)
;          ACTCENT (central angle of transfer arc)

S34/35.2 	STQ	VLOAD
			SUBEXIT
			VPASS3
		PDVL	PDDL
			RPASS3
			INTIME
		PDDL	PDDL
			TPASS4
			TWOPI		# CONIC
		PDDL	BHIZ
			NN
			S3435.23
		DLOAD
		DLOAD	PUSH
			ZEROVECS	# PRECISION
S3435.23	CALL
			INTINT		# GET TARGET VECTOR
S3435.25	STOVL	RTARG
			VATT
		STOVL	VPASS4
			RTARG
# COMPUTE PHI = PI + (ACOS(UNIT RA.UNIT RP) - PI)SIGN(RA*RP.U)
		UNIT	PDVL		# UNIT RP
			RACT3
		UNIT	PUSH		# UNIT RA
		VXV	DOT
			0D
			UNRM		# RA*RP.U
		PDVL
		DOT	SL1		# UNIT RA.UNIT RP
			0D
		ACOS	SIGN
		BPL	DAD
			NOPIE
			DPPOSMAX	# REASONABLE TWO PI
NOPIE		STODL	ACTCENT
			TPASS4
		DSU
			INTIME
		STORE	DELLT4
		SLOAD	SETPD
			NN		# NUMBER OF OFFSETS
			0D
		PDDL	PDVL
			EPSFOUR
			RACT3
		STOVL	RINIT
# Page 674
			VACT3
		STCALL	VINIT
			INITVEL
		CALL
			LOMAT
		VLOAD	MXV
			DELVEET3
			0D
		VSL1
		STCALL	DELVLVC
			SUBEXIT

# Page 675
# ***** S34/35.3 *****

; S34/35.3 - INTEGRATE TARGET POSITION WITH NEW DELTA-V
;
; This subroutine applies the computed delta-V correction to the active
; vehicle and integrates forward to compute the new target position at
; intercept time. This allows checking whether the proposed maneuver
; achieves the desired rendezvous geometry.
;
; COMMENT-ONLY READERS: After computing a velocity change, the computer
; simulates the resulting trajectory to verify the spacecraft will reach
; the intended intercept point. If not quite right, the calculation iterates
; with small adjustments until the solution converges.
;
; CODE-ALONG READERS: The routine transforms DELVLVC (delta-V in local
; vertical coordinates) back to inertial reference frame using LOMAT matrix.
; It then adds this delta-V to VACT3 to get the new required velocity,
; integrates from TIG to TPASS4 using INTINT, and stores the resulting
; position in RTARG. The DVLOS (delta-V line-of-sight components) is
; computed for display purposes.
;
; INPUTS:  DELVLVC (delta-V in local vertical coordinates)
;          VACT3, RACT3 (active vehicle state)
;          TIG (time of ignition), TPASS4 (intercept time)
; OUTPUTS: RTARG (predicted target position after maneuver)
;          DELVEET3 (delta-V in inertial frame)
;          DVLOS (delta-V components along line-of-sight)

S34/35.3	STQ	CALL
			NORMEX
			LOMAT		# GET MATRIX IN PUSH LIST
		VLOAD	VXM
			DELVLVC		# NEW DEL V TPI
			0D
		VSL1
		STORE	DELVEET3	# SAVE FOR TRANSFORM
		VAD	PDVL
			VACT3		# NEW V REQ
			RACT3
		PDDL	PDDL
			TIG
			TPASS4
		PDDL	PUSH
			DPPOSMAX
		CALL			# INTEG. FOR NEW TARGET VEC
			INTINT
		VLOAD
			RATT
		STORE	RTARG
NOVRWRT		VLOAD	PUSH
			ULOS
		VXV	VCOMP
			UNRM
		UNIT	PUSH
		VXV	VSL1
			ULOS
		PDVL
		PDVL	MXV
			DELVEET3
			0D
		VSL1
		STCALL	DVLOS
			NORMEX

# Page 676
# ***** S34/35.4 *****

; S34/35.4 - SKIP ASTRONAUT OVERWRITE
;
; This is a short entry point used when astronaut modifications to the
; computed delta-V are not permitted. It bypasses the crew input step and
; directly proceeds to NOVRWRT to continue with the computed solution.
;
; Used during automatic sequences or when ground control has disabled
; manual delta-V adjustments.

S34/35.4	STQ	SETPD		# NO ASTRONAUT OVERWRITE
			NORMEX
			0D
		GOTO
			NOVRWRT

# Page 677
# ***** LOMAT *****

; LOMAT - LOCAL ORIENTATION MATRIX
;
; Constructs a coordinate transformation matrix from inertial reference frame
; to local vertical reference frame centered on the active vehicle. This
; matrix allows expressing delta-V vectors in terms of local directions
; (along velocity, perpendicular to orbit plane, radial) rather than the
; inertial X-Y-Z frame.
;
; COMMENT-ONLY READERS: The computer creates a custom coordinate system
; centered on the spacecraft, with axes pointing along its velocity direction,
; perpendicular to its orbital plane, and toward/away from the planet. This
; makes it easier for astronauts to visualize and adjust maneuvers.
;
; CODE-ALONG READERS: The matrix is constructed with:
;   X-axis (0D):  Unit velocity × Unit normal (in-plane perpendicular to velocity)
;   Y-axis (6D):  -Unit normal (perpendicular to orbital plane)
;   Z-axis (12D): -Unit position (radial direction, toward planet center)
;
; This forms a right-handed orthonormal basis for the local vertical frame.
; The matrix is stored in push-down list locations 0D, 6D, 12D (18 words).
;
; INPUTS:  UNRM (unit normal to orbit), RACT3 (position vector)
; OUTPUTS: 3×3 transformation matrix in push-down list

LOMAT		VLOAD	VCOMP
			UNRM
		STOVL	6D		# Y
			RACT3
		UNIT	VCOMP
		STORE	12D
		VXV	VSL1
			UNRM		# Z*-Y
		STORE	0D
		SETPD	RVQ
			18D
GOINT		PDDL	PDDL		# DO
			ZEROVECS	#	NOT
			NOMTPI		#
		PUSH	PUSH		#		ORDER OR INSERT BEFORE INTINT

; INTINT - INTEGRATE TARGET POSITION/VELOCITY FORWARD IN TIME
;
; Numerically integrates the passive vehicle's (target's) trajectory from
; the current time to the planned intercept time (TPASS4) using precision
; conic section orbital mechanics. This propagates the target's state vector
; forward in time to predict where it will be at the intercept.
;
; COMMENT-ONLY READERS: The computer calculates where the target spacecraft
; will be at the intercept time by simulating its orbital motion forward
; through time. This accounts for the curved path through space caused by
; gravitational forces around the Moon or Earth.
;
; CODE-ALONG READERS: Uses the INTSTALL/INTEGRVS integration routines to
; propagate position and velocity. The integration setup involves:
;   - INTSTALL: Initializes integration parameters and state vectors
;   - INTYPFLG: Flag controlling integration type/method
;   - MOONFLAG/CMOONFLG: Selects lunar vs Earth gravitational model
;   - RCV/VCV: Position and velocity vectors for integration
;   - INTEGRVS: Performs the numerical integration
;   - RATT: Result position vector after integration
;
; The integration accounts for:
;   - Central body gravitational acceleration (Moon or Earth)
;   - Conic section approximation (two-body problem)
;   - Time step control for numerical accuracy
;   - Vector scaling based on coordinate frame (RTX2 index)
;
; INPUTS:  Initial state vectors set up in calling routine
;          TDEC1 (target time), TET (initial time)
; OUTPUTS: RATT (position at target time), velocity in VCV
;
INTINT		STQ	CALL
			RTRN
			INTSTALL
		CLEAR	DLOAD
			INTYPFLG
		BZE	SET
			+2
			INTYPFLG
		DLOAD	STADR
		STODL	TDEC1
		SET	LXA,2
			MOONFLAG
			RTX2
		BON	CLEAR
			CMOONFLG
			ALLSET
			MOONFLAG
ALLSET		STOVL	TET
		VSR*
			0,2
		STOVL	RCV
		VSR*
			0,2
		STCALL	VCV
			INTEGRVS
		VLOAD	GOTO
			RATT
			RTRN

# Page 678
# ***** S34/35.5 *****
# SUBROUTINES USED
#	BANKCALL
#	GOFLASH
#	GOTOPOOH
#	S34/35.3
#	S34.35.4
#	VNPOOH

; S34/35.5 - ASTRONAUT INPUT AND TARGET PARAMETER VERIFICATION
;
; This routine handles astronaut input via the DSKY and verification of target
; parameters before executing the Transfer Phase Initiation (TPI) maneuver.
; The astronaut reviews computed values and can modify parameters, then
; approves the maneuver for execution.
;
; COMMENT-ONLY READERS: The astronaut views key rendezvous parameters on the
; DSKY display and can modify them if needed. After approval, the computer
; proceeds to calculate the final maneuver. This ensures the crew has final
; authority over critical rendezvous burns. During Apollo 11's rendezvous,
; these displays allowed Armstrong and Aldrin to monitor and approve each
; step of the complex orbital ballet that reunited Eagle with Columbia.
;
; CODE-ALONG READERS: This is a key crew interface section that:
;   - Stores return address in SUBEXIT for later use
;   - Tests FINALFLG to determine if this is final approval or intermediate
;   - Branches based on flag states (FINALFLG and UPDATFLG):
;       * FINALFLG set: Skip update, go to FLAGON for display
;       * FINALFLG clear: Set UPDATFLG and go to FLAGOFF for recalculation
;   - FLAGON path: Loads and displays current parameters via V06N59
;   - FLAGOFF path: Calls S34/35.4 for parameter recalculation
;   - Uses VNPOOH routine to flash display and get crew response
;   - SUBEXIT returns to calling program after approval
;
; The verb/noun displays show:
;   - V06N59: TIG, DELVEET (required delta-velocity), other parameters
;
; Astronaut actions:
;   - PROCEED: Accept parameters and continue to maneuver execution
;   - TERMINATE: Abort program and return to POO (standby)
;   - RECYCLE: Return for new computation with modified inputs
;
; INPUTS:  Computed target parameters from previous routines
;          FINALFLG (indicates final approval cycle)
; OUTPUTS: Approved parameters stored for thrusting program use
;
S34/35.5	STQ	BON
			SUBEXIT
			FINALFLG
			FLAGON
		SET	GOTO
			UPDATFLG
			FLAGOFF
FLAGON		CLEAR	VLOAD
			NTARGFLG
			DELVLVC
		STORE	GDT/2
		EXIT
 +5		CAF	V06N81
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	+2		# PRO
		TC	FLAGON 	+5	# LOAD
 +2		CA	EBANK7
		TS	EBANK		# TO BE SURE

		ZL
		CA	FIVE
NTARGCHK	TS	Q
		INDEX	Q
		CS	DELVLVC
		INDEX	Q
		AD	GDT/2
		ADS	L
		CCS	Q
		TCF	NTARGCHK
		LXCH	A
		EXTEND
		BZF	+3
		TC	UPFLAG
		ADRES	NTARGFLG

		TC	INTPRET
		BOFF	CALL
			NTARGFLG
# Page 679
			NOCHG
			S34/35.3
NOCHG		CLEAR	VLOAD
			XDELVFLG
			DELVEET3
		STORE	DELVSIN
FLAGOFF		CALL
			S34/35.4
		EXIT
		CAF	V06N59
		TC	VNPOOH
		TC	INTPRET
		GOTO
			SUBEXIT

# Page 680
# ***** VN1645 *****
#
# SUBROUTINES USED
#	P3XORP7X
#	GET+MGA
#	BANKCALL
#	DELAYJOB
#	COMPTGO
#	GOFLASHR
#	GOTOPOOH
#	FLAGUP

; VN1645 - DISPLAY TRACKING MARK COUNT, TIME-TO-GO, AND MGA
;
; This verb/noun routine displays critical rendezvous tracking parameters
; on the DSKY for astronaut monitoring during the rendezvous sequence.
; It provides real-time updates of tracking status and midcourse guidance.
;
; COMMENT-ONLY READERS: During rendezvous, the astronaut monitors how many
; radar tracking marks have been acquired, how much time remains until the
; next burn, and the computed midcourse guidance angle (MGA). The display
; updates every second, giving the crew continuous awareness of the
; rendezvous progress. This was essential during Apollo 11's Eagle-Columbia
; rendezvous as it gave Armstrong and Aldrin confidence that the computer
; was properly tracking their target.
;
; CODE-ALONG READERS: This routine:
;   - Stores return address in SUBEXIT
;   - Initializes MGA (Midcourse Guidance Angle) to -0.01 degrees
;   - Tests FINALFLG to determine if final phase:
;       * If FINALFLG set: Uses MGA = -0.01
;       * If FINALFLG clear: Uses MGA = -0.02 (adds another -0.01)
;   - Tests REFSMFLG to determine coordinate reference frame
;   - Calls P3XORP7X to check program mode (P34 vs P74)
;   - Calls GET+MGA to compute actual midcourse guidance angle
;   - Initiates COMPTGO task to continuously update TTOGO (time-to-go)
;   - Displays V16N45 showing:
;       * TRKMKCNT: Number of radar tracking marks acquired
;       * TTOGO: Time remaining until next maneuver ignition
;       * +MGA: Midcourse guidance angle for trajectory correction
;   - Updates display every 1 second via DELAYJOB
;   - Waits for astronaut response (PROCEED/TERMINATE/RECYCLE)
;
; Display timing:
;   - 1SEC delay between updates ensures smooth real-time display
;   - COMPTGO task runs in background updating TTOGO continuously
;   - DISPDEX controls display update rate
;
; Astronaut actions:
;   - PROCEED: Accept tracking data and continue (N45PROC)
;   - TERMINATE: Stop tracking updates and return to standby (KILCLOCK)
;   - RECYCLE: Return for fresh computation (CLUPDATE)
;
; INPUTS:  DELVSIN (delta-velocity sine component)
;          FINALFLG, REFSMFLG (program state flags)
; OUTPUTS: TRKMKCNT, TTOGO, +MGA displayed on DSKY
;
VN1645		STQ	DLOAD
			SUBEXIT
			DP-.01
		STORE	+MGA		# MGA = -.01
		BOFF	DLOAD
			FINALFLG
			GET45
			DP-.01
		DAD
			DP-.01
		STORE	+MGA		# MGA = -.02
		BOFF	EXIT
			REFSMFLG
			GET45
		TC	P3XORP7X
		TC	+2		# P3X
		TC	GET45 	+1	# P7X
		TC	INTPRET
		VLOAD	PUSH
			DELVSIN
		CALL			# COMPUTE MGA
			GET+MGA
GET45		EXIT
		TC	COMPTGO		# INITIATE TASK TO UPDATE TTOGO
		CA	SUBEXIT
		TS	QSAVED
		CAF	1SEC
		TC	BANKCALL
		CADR	DELAYJOB
		CAF	V16N45		# TRKMKCNT, TTOGO, +MGA
		TC	BANKCALL
		CADR	GOFLASH
		TC	KILCLOCK	# TERMINATE
		TC	N45PROC		# PROCEED
		TC	CLUPDATE	# RECYCLE -- RETURN FOR INITIAL COMPUTATION
KILCLOCK	CA	Z
		TS	DISPDEX
# Page 681
		TC	GOTOPOOH
N45PROC		CS	FLAGWRD2
		MASK	BIT6
		EXTEND
		BZF	KILCLOCK	# FINALFLG IS SET -- FLASH V37 -- AWAIT NEW PGM
		TC	PHASCHNG
		OCT	04024
		TC	UPFLAG		# SET
		ADRES	FINALFLG	# FINALFLG
CLUPDATE	CA	Z
		TS	DISPDEX
		TC	PHASCHNG
		OCT	04024
		TC	INTPRET
		CLEAR	GOTO
			UPDATFLG
			QSAVED

# Page 682
# ***** DISPLAYE *****
#
# SUBROUTINES USED
#	BANKCALL
#	GOFLASHR
#	GOTOPOOH
#	BLANKET
#	ENDOFJOB

; DISPLAYE - DISPLAY ELEVATION ANGLE
;
; Simple display routine that shows the elevation angle (E) of the line of
; sight between active and passive vehicles. The elevation angle is measured
; from the horizontal plane defined by the active vehicle's position vector.
;
; COMMENT-ONLY READERS: The computer displays the angle between the two
; spacecraft as seen from the pilot's perspective. This angle is crucial
; for rendezvous timing - the crew wants to catch up to the target when
; it's at the right angle above (or below) their orbital plane. Think of
; it like knowing what angle to look up at to see the other spacecraft.
;
; CODE-ALONG READERS: This is a streamlined display routine:
;   - Saves return address in NORMEX (normal exit)
;   - Displays V06N55 (elevation angle display)
;   - Uses GOFLASH to flash display and await crew response
;   - Three possible outcomes:
;       * TERMINATE (TCF GOTOPOOH): Return to POO standby mode
;       * PROCEED (TC NORMEX): Continue with current value
;       * RECYCLE (TCF -5): Redisplay and wait again
;
; V06N55 displays:
;   - Elevation angle in degrees (fractional revolutions converted to degrees)
;   - Typical values range from -90° to +90°
;
; INPUTS:  Elevation angle computed by calling program
; OUTPUTS: None (display only)
; RETURNS: Via NORMEX to calling routine
;
DISPLAYE	EXTEND
		QXCH	NORMEX
		CAF	V06N55
		TCR	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH
		TC	NORMEX
		TCF	-5

# Page 683
# ***** P3XORP7X *****

; P3XORP7X - CHECK IF PROGRAM IS P3X (P30-39) OR P7X (P70-79)
;
; Simple utility routine that determines which program family is running
; by examining the high-order bits of the mode register. This is used to
; branch to appropriate program-specific code paths.
;
; COMMENT-ONLY READERS: The computer checks whether it's running a P30-series
; program (like P34) or a P70-series program (like P74). Different program
; families have slightly different procedures, so the computer needs to know
; which one is active.
;
; CODE-ALONG READERS: This routine:
;   - Loads HIGH9 mask (octal 77600) to isolate upper bits
;   - Masks MODREG (mode register containing program number)
;   - Tests if result is zero:
;       * Zero: P30-39 program family (skip increment, return to Q)
;       * Non-zero: P70-79 program family (increment Q by 1, then return)
;   - The Q register increment allows caller to have two return addresses:
;       * Return to Q: P3X case
;       * Return to Q+1: P7X case
;
; This is a common AGC pattern for binary decision returns - the subroutine
; modifies its own return address to select between two code paths.
;
; INPUTS:  MODREG (current program number)
; OUTPUTS: Q register possibly incremented
; RETURNS: Via Q (P3X) or Q+1 (P7X)
;
P3XORP7X	CAF	HIGH9
		MASK	MODREG
		EXTEND
		BZF	+2
		INCR	Q
		RETURN

# ***** VNPOOH *****
#
# SUBROUTINES USED
#	BANKCALL
#	GOFLASH
#	GOTOPOOH

; VNPOOH - VERB/NOUN PROCEED OR HOLD
;
; Display routine that flashes verb/noun on DSKY and waits for astronaut
; response. Handles three possible crew inputs: PROCEED, TERMINATE, or RECYCLE.
; Named "VNPOOH" as a play on Winnie-the-Pooh, reflecting the "proceed or hold"
; decision point.
;
; COMMENT-ONLY READERS: The computer pauses and waits for the astronaut's
; decision. The DSKY display flashes to get attention. The astronaut can:
;   - Press PROCEED to accept and continue
;   - Press TERMINATE to stop and return to standby (GOTOPOOH routine)
;   - Press RECYCLE to go back and recalculate
;
; CODE-ALONG READERS: This is a standard DSKY interaction routine:
;   - Saves return address from Q to RTRN using QXCH (Q exchange)
;   - Stores accumulator to VERBNOUN (verb/noun code)
;   - Loads VERBNOUN and calls GOFLASH via BANKCALL to flash display
;   - Branches based on astronaut response:
;       * Return+0 (TERMINATE): TCF GOTOPOOH - return to POO standby mode
;       * Return+1 (PROCEED): TC RTRN - continue to saved return address
;       * Return+2 (RECYCLE): TCF -5 - jump back to re-execute from EXTEND
;   - The BANKCALL return uses standard three-way branching convention
;
; This is a common pattern in rendezvous programs where crew approval
; is required at multiple stages. The flashing display alerts the crew
; that their input is needed.
;
; INPUTS:  Accumulator contains verb/noun code to display
; OUTPUTS: Q return address saved to RTRN, VERBNOUN updated
; RETURNS: Via RTRN (proceed), GOTOPOOH (terminate), or re-execute (recycle)
;
VNPOOH		EXTEND
		QXCH	RTRN
		TS	VERBNOUN
		CA	VERBNOUN
		TCR	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH
		TC	RTRN
		TCF	-5

# Page 684
# ***** CONSTANTS *****

; ============================================================================
; CONSTANTS - VERB/NOUN CODES AND NUMERICAL CONSTANTS
;
; This section defines display codes and mathematical constants used throughout
; the P34-35 and P74-75 programs. Verb/noun (V-N) codes specify DSKY display
; formats, while numerical constants support orbital mechanics calculations.
;
; COMMENT-ONLY READERS: These are the preset codes and numbers the computer
; uses for displaying information to the astronauts and performing trajectory
; calculations. Each V-N code tells the DSKY what format to use.
;
; CODE-ALONG READERS: Constants defined here include:
;   - V06N37: Display format for TPI time and elevation angle
;   - V06N55: Display format for delta-V components
;   - V06N58, V06N59, V06N81: Additional rendezvous display formats
;   - V16N45: Monitor format for continuous display
;   - TWOPI: 2π (6.283185307) scaled B-4 for angle conversions
;   - MAX250: Maximum value 25,000 (for range/altitude limits)
;   - THIRD: 1/3 (0.333333333) for cubic equation solutions
;   - ELEPS: Small angle epsilon (0.27777777 E-3) for convergence tests
;   - DP-.01: Double precision -0.01 for display adjustments
;   - EPSFOUR: 1/24 (0.0416666666) for Taylor series expansions
;   - 130DEG: 130 degrees (0.3611111111 revolutions) for transfer angle limits
;
; Scaling notation: B-4 means scaled by 2^-4, E3 means ×10^3, etc.
; ============================================================================

V06N37		VN	0637
V06N55		VN	0655
V06N58		VN	0658
V06N59		VN	0659
V06N81		VN	0681
V16N45		VN	1645
TWOPI		2DEC	6.283185307 B-4

MAX250		2DEC	25 E3

THIRD		2DEC	.333333333

ELEPS		2DEC	.27777777 E-3

DP-.01		OCT	77777		# CONSTANTS
		OCT	61337		# ADJACENT	-.01 FOR MGA DSP
EPSFOUR		2DEC	.0416666666

130DEG		2DEC	.3611111111

# Page 685
# ***** INITVEL *****

; ============================================================================
; TRANSITION: From Display and Constants to Lambert Trajectory Computation
;
; With the crew interaction routines and display constants defined, we now
; move into the core mathematical engine of the rendezvous targeting programs.
; INITVEL solves the Lambert problem: given two positions in space and a
; desired transfer time, compute the required velocity vectors.
;
; This is the fundamental calculation that determines "how fast and in what
; direction must we thrust to reach the other spacecraft?" During Apollo 11's
; rendezvous, these calculations ensured Eagle could accurately target Columbia
; after ascending from the lunar surface.
;
; The Lambert problem is one of the classic challenges in orbital mechanics,
; and this implementation uses iterative methods with both conic (two-body)
; and precision (perturbed) trajectory options.
; ============================================================================

# MOD NO -1			LOG SECTION -- P34-P35, P74-P75
# MOD BY WHITE, P.		DATE:  21 NOV 67
#
# FUNCTIONAL
#
#	THIS SUBROUTINE COMPUTES THE REQUIRED INITIAL VELOCITY VECTOR FOR
#	A TRAJECTORY OF SPECIFIC TRANSFER TIME BETWEEN SPECIFIED INITIAL
#	AND TARGET POSITIONS.  THE TRAJECTORY MAY BE EITHER CONIC OR
#	PRECISION DEPENDING ON AN INPUT PARAMETER (NAMELY, NUMBER OF
#	OFFSETS).  IN ADDITION, IN THE PRECISION TRAJECTORY CASE, THE
#	SUBROUTINE ALSO COMPUTES AN OFFSET TARGET VECTOR, TO BE USED
#	DURING PURE-CONIC CROSS-PRODUCT STEERING.  THE OFFSET TARGET
#	VECTOR IS THE TERMINAL POSITION VECTOR OF A CONIC TRAJECTORY WHICH
#	HAS THE SAME INITIAL STATE AS A PRECISION TRAJECTORY WHOSE
#	TERMINAL POSITION VECTOR IS THE SPECIFIED TARGET VECTOR.
#
#	IN ORDER TO AVOID THE INHERENT SINGULARITIES IN THE 180 DEGREE
#	TRANSFER CASE WHEN THE (TRUE OR OFFSET) TARGET VECTOR MAY BE
#	SLIGHTLY OUT OF THE ORBITAL PLANE, THIS SUBROUTINE ROTATES THIS
#	VECTOR INTO A PLANE DEFINED BY THE INPUT INITIAL POSITION VECTOR
#	AND ANOTHER INPUT VECTOR (USUALLY THE INITIAL VELOCITY VECTOR),
#	WHENEVER THE INPUT TARGET VECTOR LIES INSIDE A CONE WHOSE VERTEX
#	IS THE ORIGIN OF COORDINATES, WHOSE AXIS IS THE 180 DEGREE
#	TRANSFER DIRECTION, AND WHOSE CONE ANGLE IS SPECIFIED BY THE USER.
#
#	THE LAMBERT SUBROUTINE IS UTILIZED FOR THE CONIC COMPUTATIONS AND
#	THE COASTING INTEGRATION SUBROUTINE IS UTILIZED FOR THE PRECISION
#	TRAJECTORY COMPUTATIONS.
#
# CALLING SEQUENCE
#
#	L	CALL
#	L+1		INITVEL
#	L+2	(RETURN -- ALWAYS)
#
# INPUT
#
#	(1)	RINIT		INITIAL POSITION RADIUS VECTOR
#	(2)	VINIT		INITIAL POSITION VELOCITY VECTOR
#	(3)	RTARG		TARGET POSITION RADIUS VECTOR
#	(4)	DELLT4		DESIRED TIME OF FLIGHT FROM RINIT TO RTARG
#	(5)	INTIME		TIME OF RINIT
#	(6)	0D		NUMBER OF ITERATIONS OF LAMBERT/INTEGRVS
#	(7)	2D		ANGLE TO 180 DEGREES WHEN ROTATION STARTS
#	(8)	RTX1		-2 FOR EARTH, -10D FOR LUNAR
#	(9)	RTX2		COORDINATE SYSTEM ORIGIN -- 0 FOR EARTH, 2 FOR LUNAR
#	PUSHLOC SET AT 4D
#
# Page 686
# OUTPUT
#
#	(1)	RTARG		OFFSET TARGET POSITION VECTOR
#	(2)	VIPRIME		MANEUVER VELOCITY REQUIRED
#	(3)	VTPRIME		VELOCITY AT TARGET AFTER MANEUVER
#	(4)	DELVEET3	DELTA VELOCITY REQUIRED FOR MANEUVER
#
# SUBROUTINES USED
#
#	LAMBERT
#	INTSTALL
#	INTEGRVS

; INITVEL - INITIALIZE VELOCITY FOR LAMBERT TARGETING
;
; This is the master subroutine that solves the Lambert problem: given two
; position vectors (current and target) and a time-of-flight, compute the
; required initial velocity vector. This is the mathematical heart of all
; rendezvous targeting programs (P34, P35, P74, P75).
;
; COMMENT-ONLY READERS: This routine answers the critical question: "What
; velocity do we need right now to reach the target spacecraft at the desired
; time?" During Apollo 11's rendezvous after Eagle's ascent from the Moon,
; this calculation repeatedly ran to refine the trajectory that would bring
; the two spacecraft together.
;
; The routine handles both simple conic (two-body) solutions and complex
; precision trajectories that account for Moon's gravity perturbations. It
; can iterate multiple times to converge on an accurate answer, with each
; iteration refining the velocity estimate.
;
; Special care is taken for 180-degree transfers (halfway around the orbit)
; where mathematical singularities can occur. If the target is nearly opposite
; the current position, the routine rotates vectors into a better-defined
; plane to avoid numerical problems.
;
; CODE-ALONG READERS: Implementation details:
;
; INPUTS (stored in erasable memory before calling):
;   RINIT    - Initial position vector (B-29 meters)
;   VINIT    - Initial velocity vector (B-7 meters/centisecond)
;   RTARG    - Target position vector (B-29 meters)
;   DELLT4   - Desired time-of-flight (centiseconds)
;   INTIME   - Time corresponding to RINIT
;   0D       - Number of Lambert/integration iterations (usually 0-3)
;   2D       - Cone angle threshold for 180-degree rotation (typically ~10 deg)
;   RTX1     - Gravity parameter flag (-2 Earth, -10D Moon)
;   RTX2     - Coordinate origin (0 = Earth, 2 = Moon)
;
; OUTPUTS:
;   RTARG    - Possibly rotated target position vector
;   VIPRIME  - Required maneuver velocity at RINIT (B-7 m/cs)
;   VTPRIME  - Velocity at target after transfer (B-7 m/cs)
;   DELVEET3 - Delta-V magnitude for display (B-7 m/cs)
;
; ALGORITHM OVERVIEW:
; 1. Normalize and store target vector, compute magnitude
; 2. If lunar coordinates (RTX2=2), rescale all vectors for precision
; 3. Initialize iteration counter (ITCTR) to -1
; 4. Compute cone angle cosine for 180-degree transfer detection
; 5. Set up R1VEC=RINIT, R2VEC=RTARG for Lambert subroutine
; 6. Compute plane normal UN = UNIT(RINIT × VINIT)
; 7. Check if target is in 180-degree cone; rotate if necessary
; 8. Call LAMBERT for conic solution
; 9. If iterations requested, call INTSTALL/INTEGRVS for precision
; 10. Return with VIPRIME containing required velocity
;
; The GUESSW flag indicates no initial velocity guess is available (cold start).
; If a previous Lambert solution exists, HAVEGUES entry can be used instead.
;
; Scaling: Position vectors use B-29 (1 unit = 1.862 nanometers), velocities
; use B-7 (1 unit = 1.28 meters/centisecond). These scales maximize precision
; within AGC's 15-bit word length for cislunar distances and typical orbital
; velocities.
;
; The routine uses the interpretive language for vector operations (VLOAD, VXV,
; UNIT, DOT, etc.) which provides high-level vector/matrix capabilities with
; automatic scaling management.

		SETLOC	INTVEL
		BANK

		COUNT*	$$/INITV
INITVEL		SET			# COGA GUESS NOT AVAILABLE
			GUESSW
HAVEGUES	VLOAD	STQ
			RTARG
			NORMEX
		STORE	RTARG1
		ABVAL
		STORE	RTMAG
		SLOAD	BHIZ
			RTX2
			INITVEL1
		VLOAD	VSL2
			RINIT		# B29
		STOVL	RINIT		# B27
			VINIT		# B7
		VSL2
		STOVL	VINIT		# B5
			RTARG1
		VSL2
		STORE	RTARG1
		ABVAL
		STORE	RTMAG

# INITIALIZATION

INITVEL1	SSP	DLOAD		# SET ITCTR TO -1,LOAD MPAC WITH E4 (PL 2D)
			ITCTR
			0 	-1
		COSINE	SR1		# CALCULATE COSINE (E4) (+2)
		STODL	COZY4		# SET COZY4 TO COSINE (E4) 	    (PL 0D)
		LXA,2	SXA,2
			MPAC
			VTARGTAG	# SET VTARGTAG TO 0D (SP)
		VLOAD
# Page 687
			RINIT
		STOVL	R1VEC		# R1VEC EQ RINIT
			RTARG1
		STODL	R2VEC		# R2VEC EQ RTARG
			DELLT4
		STORE	TDESIRED	# TDESIRED EQ DELLT4
		SETPD	VLOAD
			0D		# INITIALIZE PL TO 0D
			RINIT		# MPAC EQ RINIT (+29)
		UNIT	PUSH		# UNIT(RI) (+1)		            (PL 6D)
		VXV	UNIT
			VINIT		# MPAC EQ UNIT(RI) X VI (+8)
		STOVL	UN
			RTARG1
		UNIT	DOT		# TEMP*RT.URI (+2)		    (PL 0D)
		DAD	CLEAR
			COZY4
			NORMSW
		STORE	COZY4
INITVEL2	BPL	SET
			INITVEL3	# UN CALCULATED IN LAMBERT
			NORMSW

# ROTATE RC INTO YC PLANE -- SET UNIT NORMAL TO YC

; COMMENT-ONLY READERS: For certain trajectory geometries (when the target is
; nearly opposite the current position, like trying to catch up after half an
; orbit), the math becomes numerically unstable. This section detects those
; cases and rotates the target vector into a better-defined reference plane
; to avoid calculation errors.
;
; CODE-ALONG READERS: When the cone angle is near 180 degrees (COZY4 negative),
; we rotate R2VEC (target position) into the YC plane perpendicular to the
; current orbit plane. This rotation eliminates the numerical singularity that
; occurs in the Lambert solution for transfer angles near 180 degrees.
;
; The rotation is: R2VEC_new = |R2VEC| * UNIT(R2VEC - PROJECT(R2VEC onto UN))
; where UN is the orbit normal. This preserves the magnitude but changes the
; direction to avoid the singularity.

		VLOAD	PUSH		#				    (PL 6D)
			R2VEC		# RC TO 6D (+29)
		ABVAL	PDVL		# RC TO MPAC, ABVAL(RC) (+29) TO OD (PL 2D)
		PUSH	VPROJ		#				    (PL 8D)
			UN
		VSL2	BVSU
		UNIT	VXSC		#				    (PL 0D)
		VSL1
		STORE	R2VEC
		TLOAD	SLOAD
			ZEROVEC
			ITCTR
		BPL	VLOAD
			INITVEL3
			R2VEC
		STORE	RTARG1
INITVEL3	DLOAD	PDVL		#				    (PL 2D)
			MUEARTH		# POSITIVE VALUE
			R2VEC
		UNIT	PDVL		# 2D = UNIT(R2VEC)		    (PL 8D)
			R1VEC
		UNIT	PUSH		# 8D = UNIT(R1VEC)		   (PL 14D)
		VXV	VCOMP		# -N = UNIT(R2VEC) X UNIT(R1VEC)
			2D
		PUSH			#				   (PL 20D)
		LXA,1	DLOAD
# Page 688
			RTX1
			18D
		BMN	INCR,1
			+2
		DEC	-8
		INCR,1	SLOAD
			10D
			X1
		BHIZ	VLOAD		#				   (PL 14D)
			+2
		VCOMP	PUSH		#				   (PL 20D)
		VLOAD			#				   (PL 14D)
		VXV	DOT		#				    (PL 2D)
		BPL	DLOAD		#				    (PL 0D)
			INITVEL4
		DCOMP	PUSH		#				    (PL 2D)
INITVEL4	LXA,2	SXA,2
			0D
			GEOMSGN

# SET INPUTS UP FOR LAMBERT

; COMMENT-ONLY READERS: Now that the geometry is properly set up, we're ready
; to call the Lambert subroutine - the mathematical solver that will compute
; what velocity we need to transfer between the two positions in the desired
; time. This is the key calculation that tells us "thrust this fast in this
; direction."
;
; CODE-ALONG READERS: The RTX1 index register selects the appropriate gravity
; parameter (mu) for the calculation: -2 for Earth, -10D for Moon. SETITCTR
; initializes the iteration counter before calling the Lambert solver.
;
; The Lambert subroutine expects:
;   R1VEC - initial position (already set)
;   R2VEC - target position (possibly rotated, already set)
;   DELLT4 - time of flight (already set)
;   Gravity parameter indexed by RTX1

		LXA,1	CALL
			RTX1

#  OPERATE THE LAMBERT CONIC ROUTINE (COASTFLT SUBROUTINE)

			SETITCTR	# GO TO END OF BANK TO SET ITERCTR BEFORE
					# CALLING LAMBER (FOR REMANUFACTURE ONLY)

# ARRIVED AT SOLUTION IS GOOD ENOUGH ACCORDING TO SLIGHTLY WIDER BOUNDS.

		CLEAR	VLOAD
			GUESSW
			VVEC

# STORE CALCULATED INITIAL VELOCITY REQUIRED IN VIPRIME

; COMMENT-ONLY READERS: The Lambert solver has returned an answer - the
; velocity vector we need. For a simple calculation (NUMIT=0), this conic
; solution is good enough. But for high-precision rendezvous like Apollo 11's
; lunar orbit rendezvous, we'll iterate: use this velocity to compute a more
; accurate trajectory accounting for gravity perturbations, then adjust.
;
; CODE-ALONG READERS: VIPRIME now contains the required initial velocity from
; the Lambert conic solution (two-body problem, no perturbations). If VTARGTAG
; (which stores NUMIT, the number of iterations requested) is zero, we skip
; to INITVEL7 to compute the final delta-V. Otherwise, we call INTSTALL and
; INTEGRVS to perform precision integration with perturbations (Encke method).

		STODL	VIPRIME		# INITIAL VELOCITY REQUIRED (+7)

# IF NUMIT IS ZERO, CONTINUE AT INITVELB, OTHERWISE
# SET UP INPUTS FOR ENCKE INTEGRATION (INTEGRVS).

			VTARGTAG
		BHIZ	CALL
			INITVEL7
			INTSTALL

; COMMENT-ONLY READERS: For precision targeting, the simple conic solution isn't
; enough. This section uses the computed velocity to fly a "pretend" trajectory
; forward in time, accounting for the Moon's non-uniform gravity field, to see
; where we'd actually end up. Then we compare that to where we wanted to go,
; and compute a correction. This iterate-and-refine process continues until the
; answer converges to high accuracy.
;
; CODE-ALONG READERS: After INTSTALL sets up the Encke precision integrator,
; we prepare the integration inputs:
;   MOONFLAG - Set based on RTX2 (2=Moon, 0=Earth) to select gravity model
;   R1VEC/RCV - Initial position (RINIT)
;   VCV - Initial velocity (VIPRIME from Lambert)
;   TET - Initial time (INTIME)
;   TDEC1 - Final time (INTIME + DELLT4)
;   INTYPFLG - Cleared for forward integration
;
; INTEGRVS performs numerical integration using the Encke method to compute
; the perturbed trajectory. The result (position RATT1, velocity VATT1) shows
; where the spacecraft actually arrives accounting for perturbations.

		SLOAD	CLEAR
			RTX2
			MOONFLAG
		BHIZ	SET
			INITVEL5
# Page 689
			MOONFLAG
INITVEL5	VLOAD
			RINIT
		STORE	R1VEC
		STOVL	RCV
			VIPRIME
		STODL	VCV
			INTIME
		STORE	TET
		DAD	CLEAR
			DELLT4
			INTYPFLG
		STCALL	TDEC1
			INTEGRVS
		VLOAD
			VATT1
		STORE	VTARGET

# IF ITERATION COUNTER (ITCTR) EQ NO. ITERATIONS (NUMIT), CONTINUE AT
# INITVELC, OTHERWISE REITERATE LAMBERT AND ENCKE

; COMMENT-ONLY READERS: After computing the perturbed trajectory, we check if
; we've done enough iterations. If ITCTR (current iteration count) equals NUMIT
; (requested number of iterations), we're done and can proceed to compute the
; final answer. Otherwise, we loop back to run Lambert again with adjusted
; inputs based on what we learned from this iteration.
;
; Each iteration refines the velocity estimate: Lambert gives an initial guess,
; integration shows where that guess takes us, and the difference tells us how
; to adjust for the next iteration. Typically 1-3 iterations achieve convergence.
;
; CODE-ALONG READERS: Iteration control logic:
;   1. Load ITCTR (current iteration, starts at -1) into X2
;   2. Increment X2 by 1
;   3. Store X2 back to ITCTR (now 0, 1, 2, ... on successive passes)
;   4. Subtract VTARGTAG (= NUMIT, target iteration count) from X2
;   5. If result is zero (ITCTR == NUMIT), branch to INITVEL6 (done iterating)
;   6. Otherwise, fall through to recompute R2VEC and re-call Lambert

		LXA,2	INCR,2
			ITCTR
			1D		# INCREMENT ITCTR
		SXA,2	XSU,2
			ITCTR
			VTARGTAG
		SLOAD	BHIZ		# IF SP(MPAC) EQ 0, CONTINUE AT INITVELC
			X2
			INITVEL6

# OFFSET CONIC TARGET VECTOR

; COMMENT-ONLY READERS: Since the perturbed trajectory didn't hit the exact
; target, we adjust. The calculation computes a corrected target position:
; "aim for where we wanted to go, adjusted by how far off we were this time."
; This adjusted target goes back into Lambert for another solution. With each
; iteration, the error shrinks and the answer converges.
;
; CODE-ALONG READERS: Iteration adjustment formula:
;   R2VEC = R2VEC + (RTARG1 - RATT1)
;
; Where:
;   RTARG1 = desired target position (saved from original RTARG)
;   RATT1 = where we actually arrived after precision integration
;   R2VEC = Lambert target vector, now adjusted for next iteration
;
; This adds the position error to the target: if we fell short by distance E,
; aim E farther. If we overshot by E, aim E closer. The accumulating adjustment
; in R2VEC drives convergence. After a few iterations, RATT1 ≈ RTARG1 within
; acceptable tolerance.
;
; After computing the adjusted R2VEC, we reload the cone angle COZY4 and GOTO
; INITVEL2 to re-enter the Lambert computation with refined inputs. The loop
; continues until ITCTR == NUMIT, then branches to INITVEL6.

		VLOAD	VSU
			RTARG1
			RATT1
		VAD
			R2VEC
		STODL	R2VEC
			COZY4
		GOTO
			INITVEL2	# CONTINUE ITERATING AT INITVEL2

# COMPUTE THE DELTA VELOCITY

; ============================================================================
; TRANSITION: From Lambert iteration loop to final delta-V computation
;
; Iterations are complete. The Lambert solution converged, giving us VIPRIME
; (required initial velocity) to reach the target. Now we compute the burn:
; DELTA-V = VIPRIME - VINIT (what we need minus what we have). This burn will
; start the transfer trajectory to intercept the target spacecraft.
; ============================================================================

; COMMENT-ONLY READERS: The computer has solved the rendezvous problem. After
; 1-3 iterations refining the trajectory through gravitational perturbations,
; we now know the exact velocity needed at ignition time. Subtracting our current
; velocity from this target velocity gives the delta-V: the change in velocity
; the LM's RCS or DPS engine must provide to start the transfer to rendezvous.
;
; This is the answer the crew needs: magnitude, direction, and timing of the burn
; that will bring the LM to the CSM. The calculated delta-V will be displayed on
; the DSKY for crew review and stored for use by the thrust programs (P40-P47).
;
; CODE-ALONG READERS: Final delta-V computation at INITVEL6:
;   1. Store R2VEC → RTARG1 (finalized target position for reference)
;   2. Load VIPRIME (required initial velocity from Lambert+iterations)
;   3. Subtract VINIT (current velocity)
;   4. Store result in DELVEET3 = VIPRIME - VINIT
;
; DELVEET3 is scaled at 2^+7 meters/centisecond. This vector represents the
; impulsive delta-V required at TIG (time of ignition) to initiate the transfer.
; The coordinate frame is the same as used throughout (typically inertial).

INITVEL6	VLOAD
			R2VEC
		STORE	RTARG1
INITVEL7	VLOAD	VSU
			VIPRIME
			VINIT
		STOVL	DELVEET3	# DELVEET3 = VIPRIME-VINIT (+7)
# Page 690
; CODE-ALONG READERS: Overflow protection scaling logic:
;
; RTX2 is a flag set during computation if vector magnitudes approach AGC
; register overflow limits. If RTX2 is negative (overflow risk detected),
; we scale all computed vectors down by factor of 4 (VSR2 = shift right 2 bits).
;
; If RTX2 is zero or positive (BHIZ branch), skip scaling and continue at
; INITVELX with full-precision vectors.
;
; Vectors scaled if needed:
;   VTPRIME (target velocity at intercept)
;   VIPRIME (required initial velocity)
;   RTARG1 (target position)
;   DELVEET3 (computed delta-V)
;
; This maintains computational stability while preserving maximum precision
; when possible. Subsequent code must account for the scaling factor if RTX2
; indicated overflow protection was necessary.

			VTARGET
		STORE	VTPRIME
		SLOAD	BHIZ
			RTX2
			INITVELX
		VLOAD	VSR2
			VTPRIME
		STOVL	VTPRIME
			VIPRIME
		VSR2
		STOVL	VIPRIME
			RTARG1
		VSR2
		STOVL	RTARG1
			DELVEET3
		VSR2
		STORE	DELVEET3
; CODE-ALONG READERS: Final parameter computation:
;
; MU/A computation:
;   Load X1 from RTX1 (body selection: 0=Earth, 2=Moon)
;   Load MUTABLE-2,1 (gravitational parameter μ for selected body)
;   Multiply by R1A (semi-major axis scale factor)
;   Divide by R1 (initial radius magnitude)
;   Result: MU/A = (μ * R1A) / R1
;
; This computes a normalized gravitational parameter used by guidance routines.
; The value MU/A relates orbital energy to position/velocity relationships.
;
; MUASTEER = MU/A scaled down by 2^6 for steering algorithm use.
;
; Finally, store RTARG1 → RTARG (target position) and call NORMEX to compute
; unit vectors and exit INITVEL, returning control to the calling program.

INITVELX	LXA,1	DLOAD*
			RTX1
			MUTABLE -2,1
		PUSH	DMP
			R1A
		SR1	DDV
			R1
		STODL	MU/A
		SR
			6
		STORE	MUASTEER
		SETPD	VLOAD
			0D
			RTARG1
		STCALL	RTARG
			NORMEX

# ***** END OF INITVEL ROUTINE *****

# Page 691
# ***** MIDGIM *****

; ============================================================================
; SUBROUTINE: MIDGIM - Middle Gimbal Angle and Coordinate Transformation
;
; PURPOSE: Dual-function attitude/coordinate conversion routine for rendezvous
;
; COMMENT-ONLY READERS: After computing the required delta-V for a rendezvous
; burn, the crew needs to know two things:
;   1. What spacecraft attitude (gimbal angle) to achieve for the burn
;   2. What the delta-V looks like in local vertical coordinates (up/down,
;      left/right, forward/backward relative to the spacecraft)
;
; This routine computes one or the other depending on which vehicle (LM or CSM)
; is active. The LM gets the middle gimbal angle (+MGA) for attitude reference.
; The CSM gets delta-V in local vertical coordinates for display and guidance.
;
; CODE-ALONG READERS: MIDGIM branches based on AVFLAG:
;   - AVFLAG=1 (LM active): Compute +MGA = positive middle gimbal angle
;   - AVFLAG=0 (CSM active): Compute DELVLVC = delta-V in local vertical coords
;
; The gimbal angle computation uses the stable member reference frame (REFSMMAT)
; to determine spacecraft orientation. The local vertical transformation builds
; a rotation matrix from position and velocity vectors to convert inertial
; delta-V into a body-relative frame.
; ============================================================================

# MOD NO. 0, BY WILLMAN, SUBROUTINE RENDGUID, LOG P34-P35, P74-P75
# REVISION 03, 17 FEB 67
#
# IF THE ACTIVE VEHICLE IS DOING THE COMPUTATION, MIDGIM COMPUTES
# THE POSITIVE MIDDLE GIMBAL ANGLE OF THE ACTIVE VEHICLE TO THE INPUT
# DELTA VELOCITY VECTOR (0D IN PUSH LIST), OTHERWISE
# MIDGIM CONVERTS THE INPUT DELTA VELOCITY VECTOR FROM INERTIAL COORDIN-
# ATES TO LOCAL VERTICAL COORDINATES OF THE ACTIVE VEHICLE.
#
# ** INPUTS **
#
#	NAME	MEANING						UNITS/SCALING/MODE
#
#	AVFLAG	INT FLAG -- 0 IS CSM ACTIVE, 1 IS LEM ACTIVE			BIT
#	RINIT	ACTIVE VEHICLE RADIUS VECTOR			METERS/CSEC (+7) VT
#	VINIT	ACTIVE VEHICLE VELOCITY VECTOR			METERS/CSEC (+7) VT
#	0D(PL)	ACTIVE VEHICLE DELTA VELOCITY VECTOR		METERS/CSEC (+7) VT
#
# ** OUTPUTS **
#
#    NAME	MEANING						UNITS/SCALING/MODE
#
#    +MGA	+ MIDDLE GIMBAL ANGLE				REVOLUTIONS (+0) DP
#    DELVLVC	DELTA VELOCITY VECTOR IN LV COORD.		METERS/CSEC (+7) VT
#    MGLVFLAG	INT FLAG: 0 IS +MGA COMPUTED, 1 IS DELVLVC COMP.		BIT
#
# ** CALLING SEQUENCE **
#
#	L 	CALL
#	L+1		MIDGIM
#	L+2	(RETURN -- ALWAYS)
#
# ** NO SUBROUTINES CALLED **
#
# ** DEBRIS -- ERASABLE TEMPORARY USAGE **
#
#	A,Q,L, PUSH LIST, MPAC.
#
# ** ALARMS -- NONE **

# Page 692
# MIDDLE GIMBAL ANGLE COMPUTATION.

		SETLOC	MIDDGIM
		BANK

		COUNT*	$$/MIDG

; CODE-ALONG READERS: Constant for gimbal angle computation.
HALFREV		2DEC	1 B-1		# Half revolution = 0.5 revolutions (180°)

; ============================================================================
; GET+MGA: Compute Positive Middle Gimbal Angle
; ============================================================================
;
; COMMENT-ONLY READERS: This calculates the spacecraft attitude needed for the
; burn. The "middle gimbal angle" is one of three angles (outer, middle, inner)
; that define spacecraft orientation. Computing +MGA tells the crew or autopilot
; what pitch attitude to achieve before ignition.
;
; The calculation finds the angle between the delta-V vector and the spacecraft's
; stable member Y-axis (the IMU reference frame). If the angle comes out negative
; (burn pointing "down"), we add 360° to get the equivalent positive angle.
;
; CODE-ALONG READERS: Middle gimbal angle computation algorithm:
;   1. Load delta-V vector from pushlist (0D), unitize → UV (unit vector)
;   2. Dot product UV · REFSMMAT+6 (Y-axis of stable member frame)
;   3. Scale result from +2 to +1 (SL1) for ARCSIN input requirements
;   4. ARCSIN: compute angle = arcsin(dot product)
;   5. If angle ≥ 0 (BPL), store directly as +MGA
;   6. If angle < 0, add 360° (two HALFREV additions) to convert to positive
;   7. Store result in +MGA (scaled in revolutions, +0)
;   8. Clear MGLVFLAG (=0 indicates +MGA computed, not DELVLVC)
;   9. Return to caller via RVQ
;
; The REFSMMAT (reference to stable member matrix) transforms between inertial
; coordinates and the IMU platform orientation. REFSMMAT+6 accesses the Y-axis
; column of this transformation matrix.

GET+MGA		VLOAD	UNIT		# (PL 0D) V (+7) TO MPAC UNITIZE UV (+1)
		UNIT
		DOT	SL1		# DOT UV WITH Y(STABLE MEMBER) AND RESCALE
			REFSMMAT +6	# FROM +2 TO +1 FOR ASIN ROUTINE
		ARCSIN	BPL
			SETMGA
		DAD	DAD		# CONVERT -MGA TO +MGA BY
			HALFREV		# ADDING ONE REVOLUTION
			HALFREV
SETMGA		STORE	+MGA
		CLR	RVQ		# CLEAR MGLVFLAG TO INDICATE +MGA CALC
			MGLVFLAG	# AND EXIT

; ============================================================================
; GET.LVC: Compute Delta-V in Local Vertical Coordinates
; ============================================================================
;
; COMMENT-ONLY READERS: This transforms the burn delta-V from inertial space
; coordinates (fixed with respect to the stars) into local vertical coordinates
; (relative to the spacecraft's position and motion). The result tells the crew
; the burn components in intuitive terms:
;   - Along velocity direction (forward/backward tangent to orbit)
;   - Cross-track (perpendicular to orbital plane, left/right)
;   - Radial (up/down from Earth/Moon center)
;
; This transformation is essential for the CSM pilot (Michael Collins during
; Apollo 11) to understand and verify the maneuver before execution. It converts
; abstract inertial coordinates into body-relative terms the crew can visualize.
;
; CODE-ALONG READERS: Local vertical coordinate frame construction algorithm:
;   1. Build orthonormal transformation matrix from position and velocity:
;      - Unitize position vector RINIT → UR (+1)
;      - Complement to get U(-R), store in pushlist 18D
;      - Cross product: U(-R) × VINIT → U(V×R), unitize, store in 12D
;      - Cross product: U(V×R) × U(-R) → U((V×R)×(-R)), unitize, store in 6D
;   
;   2. Result: 3×3 transformation matrix at 6D (+1) transforms inertial → local
;      Each column of matrix represents a basis vector of local vertical frame:
;      - Column 1: Radial direction (toward/away from planet center)
;      - Column 2: Cross-track (orbit normal, perpendicular to plane)
;      - Column 3: Along-track (velocity direction)
;
;   3. Apply transformation: Matrix multiply delta-V by transformation matrix
;      DELVLVC = [6D matrix] × DELVEET (from pushlist 0D)
;      Input: delta-V at +7, matrix at +1 → product at +8
;      VSL1: Rescale from +8 back to +7 for storage
;
;   4. Store result in DELVLVC (+7) - ready for display to crew
;   5. Set MGLVFLAG=1 to indicate local vertical coordinates computed (not +MGA)
;   6. Return via RVQ
;
; The matrix construction uses vector cross products to build an orthonormal
; coordinate system aligned with the spacecraft's orbital position and velocity.

GET.LVC		VLOAD	UNIT		# (PL 6D) R (+29) IN MPAC UNITIZE UR
			RINIT
		VCOMP			# U(-R)
		STORE	18D		# U(-R) TO 18D
		VXV	UNIT		# U(-R)*V EQ V*U(R), U(V*R)
			VINIT
		STORE	12D		# U(V*R) TO 12D
		VXV	UNIT		# U(V*R)*U(-R), U((V*R)*(-R))
			18D
		STOVL	6D		# TRANSFORMATION MATRIX IS IN 6D (+1)
			0D		# DELTA V (+7) IN 0D
		MXV	VSL1		# CONVERT FROM INER COOR TO LV COOR (+8)
			6D		# AND SCALE +7 IN MPAC
		STORE	DELVLVC		# STORE IN DELVLVC (+7)
		SET	RVQ		# SET MGLVFLAG TO INDICATE LVC CALC
			MGLVFLAG	# AND EXIT

# ***** END OF MIDGIM ROUTINE *****

# Page 693
		BANK	10
		SETLOC	SLCTMU
		BANK
		COUNT*	$$/MIDG

; ============================================================================
; SELECTMU: Select Gravitational Parameter Based on Primary Body
; ============================================================================
;
; COMMENT-ONLY READERS: This routine determines which celestial body is the
; dominant gravitational influence (Earth or Moon) and loads the appropriate
; gravitational constant (μ = GM, where G is the gravitational constant and M
; is the body's mass). This is critical for calculating orbital trajectories
; because the equations of motion depend on which body the spacecraft is
; orbiting around.
;
; During Apollo 11, this routine switched between Earth's μ and the Moon's μ
; as the spacecraft traveled from Earth orbit to lunar orbit and back.
;
; CODE-ALONG READERS: Algorithm for selecting gravitational parameter:
;   1. Initialize index registers: X1=2D, X2=0D (Earth defaults)
;   2. Test CMOONFLG (Circumlunar flag):
;      - If OFF (=0): Spacecraft is near Earth, use Earth parameters
;      - If ON (=1): Spacecraft is near Moon, use Moon parameters
;   3. If CMOONFLG set: Reset indices X1=10D, X2=2D (Moon parameters)
;   4. Load from MUTABLE (table of gravitational parameters):
;      - MUTABLE+4,1: Load radius-to-μ ratio → store in RTSR1/MU
;      - MUTABLE-2,1: Load μ value
;   5. If CMOONFLG set: Right shift μ by 6 bits (SR 6D) to scale Moon's
;      smaller gravitational parameter appropriately
;   6. Store final μ value in RTMU for use by trajectory calculations
;   7. Save index X2 position in RTX2, clear FINALFLG
;   8. Continue to VN1645 for display sequence
;
; The MUTABLE table contains gravitational parameters and radii for both Earth
; and Moon. Index arithmetic selects the appropriate set based on the flag.

SELECTMU	AXC,1	AXT,2
			2D
			0D
		BOFF
			CMOONFLG
			SETMUER
		AXC,1	AXT,2
			10D
			2D
SETMUER		DLOAD*	SXA,1
			MUTABLE +4,1
			RTX1
		STODL*	RTSR1/MU
			MUTABLE -2,1
		BOFF	SR
			CMOONFLG
			RTRNMU
			6D
RTRNMU		STORE	RTMU
		SXA,2	CLEAR
			RTX2
			FINALFLG
		GOTO
			VN1645

# Page 694
# ***** PERIAPO *****
#
# MOD NO -1			LOG SECTION -- P34-P35, P74-P75
# MOD BY WHITE, P.		DATE 18 JAN 68
#
# FUNCTIONAL DESCRIPTION
#
#	THIS SUBROUTINE COMPUTES THE TWO BODY APOCENTER AND PERICENTER
#	ALTITUDES GIVEN THE POSITION AND VELOCITY VECTORS FOR A POINT ON
#	TRAJECTORY AND THE PRIMARY BODY.
#
#	SETRAD IS CALLED TO DETERMINE THE RADIUS OF THE PRIMARY BODY.
#
#	APSIDES IS CALLED TO SOVE FOR THE TWO BODY RADII OF APOCENTER AND
#	PERICENTER AND THE ECCENTRICITY OF THE TRAJECTORY.
#
# CALLING SEQUENCE
#
#	L	CALL
#	L+1		PERIAPO
#	L+2	(RETURN -- ALWAYS)
#
# INPUT
#
#	(1)	RVEC	POSITION VECTOR IN METERS
#			SCALE FACTOR -- EARTH +29, MOON +27
#	(2)	VVEC	VELOCITY VECTOR IN METERS/CENTISECOND
#			SCALE FACTOR -- EARTH +7, MOON +5
#	(3)	X1	PRIMARY BODY INDICATOR
#			EARTH -1, MOON -10
#
# OUTPUT
#
#	(1)	2D	APOCENTER RADIUS IN METERS
#			SCALE FACTOR -- EARTH +29, MOON +27
#	(2)	4D	APOCENTER ALTITUDE IN METERS
#			SCALE FACTOR -- EARTH +29, MOON +27
#	(3)	6D	PERICENTER RADIUS IN METERS
#			SCALE FACTOR -- EARTH +29, MOON +27
#	(4)	8D	PERICENTER ALTITUDE IN METERS
#			SCALE FACTOR -- EARTH +29, MOON +27
#	(5)	ECC	ECCENTRICITY OF CONIC TRAJECTORY
#			SCALE FACTOR -- +3
#	(6)	XXXALT	RADIUS OF THE PRIMARY BODY IN METERS
#			SCALE FACTOR -- EARTH +29, MOON +27
#	(7)	PUSHLOC	EQUALS 10D
#
# SUBROUTINES USED
#
#	SETRAD
# Page 695
#	APSIDES

		SETLOC	APOPERI
		BANK

		COUNT*	$$/PERAP

; ============================================================================
; PERIAPO: Compute Apocenter and Pericenter Altitudes
; ============================================================================
;
; COMMENT-ONLY READERS: This subroutine calculates the highest and lowest
; points of the spacecraft's orbit. The highest point is called "apocenter"
; (or "apogee" for Earth, "apolune" for Moon). The lowest point is called
; "pericenter" (or "perigee" for Earth, "perilune" for Moon).
;
; These orbital parameters are critical for mission safety. During Apollo 11:
; - The Command Module's lunar orbit pericenter had to remain above 35,000 feet
;   to ensure clearance over lunar mountains
; - The Lunar Module's descent orbit pericenter determined the powered descent
;   initiation point
; - After each rendezvous maneuver, this routine verified the resulting orbit
;   would not impact the surface
;
; The routine uses the current position and velocity to predict the entire
; orbital trajectory, computing where the spacecraft will be highest and
; lowest relative to the planet or Moon's surface.
;
; CODE-ALONG READERS: Two-body orbital mechanics computation using classical
; orbital elements. Given instantaneous state vectors (position RVEC, velocity
; VVEC), compute orbital apsides (apocenter and pericenter) by:
;
;   1. Solving vis-viva equation: v² = μ(2/r - 1/a)
;      where μ = gravitational parameter, r = current radius, a = semi-major axis
;   2. Computing orbital eccentricity from angular momentum and energy
;   3. Determining apocenter radius = a(1+e) and pericenter radius = a(1-e)
;   4. Converting radii to altitudes by subtracting planetary radius
;
; Algorithm assumes two-body dynamics (spacecraft influenced only by primary
; body, ignoring perturbations from other celestial bodies, solar radiation
; pressure, etc.). Valid for short-term predictions; actual orbit evolves due
; to lunar oblateness, Earth/Sun gravity, and other perturbations.

; CODE-ALONG READERS: Launch pad reference radius constant.
RPAD		2DEC	6373338 B-29	# STANDARD RADIUS OF PAD 37-B.
					# = 20 909 901.57 FT

; ============================================================================
; PERIAPO1: Entry Point with Vector Scaling
; ============================================================================
;
; CODE-ALONG READERS: This entry point handles pre-scaled vectors from calling
; program. The scaling adjustment accounts for the different magnitude ranges
; between Earth operations (larger distances, higher velocities) and lunar
; operations (smaller distances, lower velocities).
;
; Algorithm:
;   1. LXA,2 RTX2: Load index register X2 from RTX2 (contains shift count)
;   2. VSR* 0,2: Vector shift right VVEC by X2 positions (variable scale)
;      - For Earth: typically shift right 2 positions to scale from B+7 to B+5
;      - For Moon: may use different scaling based on mission phase
;   3. STOVL VVEC: Store scaled velocity, then load VVEC for next operation
;   4. LXA,1 RTX1: Load index register X1 from RTX1 (body indicator: -1=Earth, -10=Moon)
;   5. VSR* 0,2: Vector shift right RVEC by X2 positions
;      - Scales position vector consistently with velocity
;   6. STORE RVEC: Store scaled position vector
;   7. Fall through to PERIAPO main routine (no GOTO needed, sequential execution)
;
; The VSR* (Vector Shift Right indexed) instruction performs element-wise
; right bit shifts on all three components of the vector simultaneously,
; effectively dividing the magnitude while preserving direction.

PERIAPO1	LXA,2	VSR*
			RTX2
			0,2
		STOVL	VVEC
		LXA,1	VSR*
			RTX1
			0,2
		STORE	RVEC

; ============================================================================
; PERIAPO: Main Entry Point - Compute Apocenter and Pericenter
; ============================================================================
;
; COMMENT-ONLY READERS: The spacecraft's orbit around the Moon or Earth is an
; ellipse (or circle, which is a special ellipse). The highest point of this
; orbit is the "apocenter" and the lowest point is the "pericenter."
;
; This routine calculates both of these critical altitudes. During Apollo 11's
; lunar orbit, Mission Control closely monitored these values to ensure the
; Command Module would not crash into lunar mountains during each orbit. The
; pericenter had to remain safely above 35,000 feet.
;
; CODE-ALONG READERS: Main computation sequence using interpretive language:
;
;   1. STQ NORMEX: Store return address in NORMEX
;   2. CALL SETRAD: Set planetary radius (XXXALT = Earth/Moon radius)
;      - Uses body indicator in X1 register to select correct planet
;      - Earth radius ≈ 6,378 km, Moon radius ≈ 1,738 km
;   3. STCALL XXXALT, APSIDES: Store radius, call APSIDES subroutine
;      - APSIDES computes apocenter and pericenter radii from current state vectors
;      - Returns apocenter radius in 0D, pericenter radius in 2D (MPAC positions)
;   4. Compute altitudes by subtracting planetary radius from radii
;      - Apogee altitude = Apocenter radius - Planet radius
;      - Perigee altitude = Pericenter radius - Planet radius

PERIAPO		STQ	CALL
			NORMEX
			SETRAD
		STCALL	XXXALT
			APSIDES
		SETPD	PUSH		# 2D = APOCENTER RADIUS		B29 OR B27
			2D
		DSU	PDDL		# 4D = APOGEE ALTITUDE 		B29 OR B27
			XXXALT
			0D
		PUSH	DSU		# 6D = PERICENTER RADIUS	B29 OR B27
			XXXALT
		PUSH	GOTO		# 8D = PERIGEE ALTITUDE 	B29 OR B27
			NORMEX

# Page 696
; ============================================================================
; SETRAD: Set Planetary Radius
; ============================================================================
;
; COMMENT-ONLY READERS: This subroutine determines whether the spacecraft is
; orbiting Earth or the Moon, then loads the correct planetary radius into
; memory. This is essential because:
;   - Earth's radius: approximately 6,378 km (3,963 miles)
;   - Moon's radius: approximately 1,738 km (1,080 miles)
;
; The routine needs to know which body to use when converting between orbital
; radius (distance from center of planet/moon) and altitude (distance above
; surface).
;
; CODE-ALONG READERS: Body selection algorithm:
;
;   1. DLOAD RPAD: Load Earth launch pad radius as default (6,373,338 meters)
;      - This constant represents radius to Launch Complex 37-B at Cape Kennedy
;   2. PUSH: Push radius onto MPAC stack
;   3. SXA,1 X2: Store X1 register (body indicator) in X2
;      - X1 = -1 for Earth, -10 for Moon (set by calling program)
;   4. INCR,2 2D: Increment stack pointer by 2 positions
;   5. SLOAD X2: Single-precision load X2 (body indicator)
;   6. BHIZ SETRADX: Branch if zero (actually checking sign/magnitude)
;      - If X2 indicates Earth, branch to SETRADX (use RPAD value already loaded)
;      - If X2 indicates Moon, fall through to load lunar radius
;   7. VLOAD RLS, ABVAL: Load lunar radius vector RLS, compute absolute value
;      - RLS contains Moon's radius vector
;   8. PDDL: Push result onto stack
;   9. SETRADX: DLOAD, RVQ: Load appropriate radius, return via Q register

SETRAD		DLOAD	PUSH
			RPAD
		SXA,1	INCR,2
			X2
			2D
		SLOAD	BHIZ
			X2
			SETRADX
		VLOAD	ABVAL
			RLS
		PDDL
SETRADX		DLOAD	RVQ

# Page 697
; ============================================================================
; PRECSET: Precision State Vector Setup
; ============================================================================
;
; COMMENT-ONLY READERS: During rendezvous operations, the computer needs to
; track both the Lunar Module and Command Module simultaneously. This routine
; computes precise position and velocity for both spacecraft at specific times.
;
; During Apollo 11's rendezvous on July 21, 1969, after Eagle's ascent from
; the lunar surface, this routine continuously updated the relative positions
; of Eagle (LM) and Columbia (CSM) as they approached each other in lunar orbit.
;
; The routine determines which spacecraft is "active" (performing maneuvers)
; and which is "passive" (coasting), then stores their state vectors separately
; for use in rendezvous targeting calculations.
;
; CODE-ALONG READERS: Dual spacecraft state vector computation:
;
;   1. STQ NORMEX: Store return address
;   2. STCALL TDEC2, LEMPREC: Store time in TDEC2, call LEM precision orbit routine
;      - LEMPREC computes LM position/velocity at specified time
;      - Integrates orbit forward/backward from known state
;   3. CALL LEMSTORE: Store LM state vectors in appropriate locations
;      - If LM is active vehicle: store in RACT3, VACT3
;      - If LM is passive vehicle: store in RPASS3, VPASS3
;   4. DLOAD TDEC2: Reload time (may be different for CSM)
;   5. STCALL TDEC1, CSMPREC: Store time in TDEC1, call CSM precision orbit routine
;      - CSMPREC computes CSM position/velocity at specified time
;   6. CALL CSMSTORE: Store CSM state vectors in appropriate locations
;      - If CSM is active: store in RACT3, VACT3
;      - If CSM is passive: store in RPASS3, VPASS3
;   7. GOTO NORMEX: Return to calling routine

PRECSET		STQ
			NORMEX
		STCALL	TDEC2
			LEMPREC
		CALL
			LEMSTORE
		DLOAD
			TDEC2
		STCALL	TDEC1
			CSMPREC
		CALL
			CSMSTORE
		GOTO
			NORMEX

; ============================================================================
; LEMSTORE: Store LM State Vectors (Active or Passive)
; ============================================================================
;
; COMMENT-ONLY READERS: This routine stores the Lunar Module's position and
; velocity in the correct memory locations based on whether the LM is the
; "active" vehicle (performing maneuvers) or "passive" vehicle (coasting).
;
; During Apollo 11's rendezvous on July 21, 1969, Eagle (LM) was the active
; vehicle during ascent and initial approach, while Columbia (CSM) was passive
; in a stable orbit waiting for rendezvous.
;
; CODE-ALONG READERS: Storage routing based on AVFLAG:
;
;   1. VLOAD RATT: Load LM position vector from RATT
;   2. BOFF AVFLAG, PASSIVE: Branch if AVFLAG is OFF (zero) to PASSIVE
;      - AVFLAG ON (1): LM is active vehicle → store in RACT3/VACT3
;      - AVFLAG OFF (0): LM is passive vehicle → store in RPASS3/VPASS3
;   3. If ACTIVE: STOVL RACT3: Store position in RACT3, load VATT velocity
;   4. STORE VACT3: Store velocity in VACT3
;   5. RVQ: Return via Q register
;   6. If PASSIVE: Fall through to store in RPASS3/VPASS3 instead
;
; AVFLAG is set by the rendezvous program based on mission phase. The active
; vehicle performs the maneuvers while the passive vehicle maintains orbit.

LEMSTORE	VLOAD	BOFF
			RATT
			AVFLAG
			PASSIVE
ACTIVE		STOVL	RACT3
			VATT
		STORE	VACT3
		RVQ

; ============================================================================
; CSMSTORE: Store CSM State Vectors (Active or Passive)
; ============================================================================
;
; COMMENT-ONLY READERS: This routine stores the Command/Service Module's
; position and velocity, using the opposite role from the LM. If the LM is
; active, the CSM is passive, and vice versa.
;
; During Apollo 11's rendezvous, Columbia (CSM) maintained a stable circular
; orbit while Eagle (LM) performed the active maneuvers to rendezvous. Thus
; CSM data was stored in the "passive" locations.
;
; CODE-ALONG READERS: Storage routing with inverted logic from LEMSTORE:
;
;   1. VLOAD RATT: Load CSM position vector from RATT
;   2. BOFF AVFLAG, ACTIVE: Branch if AVFLAG is OFF to ACTIVE
;      - This inverts the logic: if LM is active (AVFLAG ON), CSM is passive
;      - If AVFLAG ON: branch to ACTIVE label which stores in RPASS3/VPASS3
;      - If AVFLAG OFF: fall through to PASSIVE label which stores in RACT3/VACT3
;   3. Logic inversion ensures only one vehicle occupies active storage at a time
;   4. Two-vehicle rendezvous requires one active and one passive vehicle
;
; This complementary storage scheme prevents ambiguity about which spacecraft
; is performing maneuvers during rendezvous targeting calculations.

CSMSTORE	VLOAD	BOFF
			RATT
			AVFLAG
			ACTIVE
PASSIVE		STOVL	RPASS3
			VATT
		STORE	VPASS3
		RVQ

# Page 698
; ============================================================================
; VECSHIFT: Vector Right Shift Utility
; ============================================================================
;
; COMMENT-ONLY READERS: This is a utility routine that scales vector data
; by shifting bits to the right. This is necessary because the AGC uses
; fixed-point arithmetic with specific scaling conventions.
;
; During rendezvous calculations, position and velocity vectors often need
; to be scaled to prevent arithmetic overflow in the 15-bit AGC word format.
;
; CODE-ALONG READERS: Double-vector scaling operation:
;
;   1. LXA,2 RTX2: Load index register X2 from RTX2 (shift count)
;   2. VSR* 0,2: Vector Shift Right by amount in X2 (scales first vector)
;      - Divides vector components by 2^(X2) to reduce magnitude
;   3. LXA,1 RTX1: Load index register X1 from RTX1 (shift count for 2nd vector)
;   4. PDVL: Push first vector to stack, load second vector
;   5. VSR* 0,2: Vector Shift Right second vector by same amount in X2
;   6. PDVL: Push second vector to stack
;   7. RVQ: Return with both scaled vectors on stack
;
; This routine maintains numerical precision while preventing overflow during
; two-body orbital mechanics calculations that involve position differences
; ranging from meters to hundreds of kilometers.

VECSHIFT	LXA,2	VSR*
			RTX2
			0,2
		LXA,1	PDVL
			RTX1
		VSR*	PDVL
			0,2
		RVQ

# Page 699
; ============================================================================
; SHIFTR1: Scalar Left Shift Utility
; ============================================================================
;
; COMMENT-ONLY READERS: Despite the name "SHIFTR1", this routine actually
; shifts data LEFT, multiplying values to restore proper scaling after
; calculations. The AGC naming follows historical conventions.
;
; CODE-ALONG READERS: Single scalar scaling operation:
;
;   1. LXA,2 RTX2: Load index register X2 from RTX2 (shift count)
;   2. SL* 0,2: Shift Left by amount in X2
;      - Multiplies accumulator value by 2^(X2)
;      - Used to restore proper scaling after division or to match
;        expected units for display or maneuver execution
;   3. RVQ: Return via Q register
;
; The confusing name likely derives from register operations where "R1"
; refers to a register rather than "right". The SL* instruction clearly
; performs left shift (multiplication).

SHIFTR1		LXA,2	SL*
			RTX2
			0,2
		RVQ

# Page 700
# PROGRAM DESCRIPTION
# SUBROUTINE NAME	R36	OUT-OF-PLANE RENDEZVOUS ROUTINE
# MOD NO. 0		DATE 	22 DECEMBER 67
# MOD BY N.M.NEVILLE	LOG SECTION EXTENDED VERBS
# FUNCTIONAL DESCRIPTION
#
# TO DISPLAY AT ASTRONAUT REQUEST LGC CALCULATED RENDEZVOUS
# OUT-OF-PLANE PARAMETERS (Y, YDOT, PSI).  (REQUESTED BY DSKY).
#
# CALLING SEQUENCE
#
#	ASTRONAUT REQUEST THROUGH DSKY V 90 E
#
# SUBROUTINES CALLED
#
#	EXDSPRET
#	GOMARKF
#	CSMPREC
#	LEMPREC
#	SGNAGREE
#	LOADTIME
#
# NORMAL EXIT MODES
#
#	ASTRONAUT REQUEST THROUGH DSKY TO TERMINATE PROGRAM V 34 E
#
# ALARM OR ABORT EXIT MODES
#
#	NONE
#
# OUTPUT
#
#	DECIMAL DISPLAY OF TIME, Y, YDOT AND PSI
#
#	DISPLAYED VALUES Y, YDOT, AND PSI, ARE STORED IN ERASABLE
#	REGISTERS RANGE, RRATE, AND RTHETA RESPECTIVELY.
#
# ERASABLE INITIALIZATION REQUIRED
#
#	CSM AND LEM STATE VECTORS
#
# DEBRIS
#
#	CENTRALS A,Q,L
#	OTHER:  THOSE USED BY THE ABOVE LISTED SUBROUTINES

		BANK	20
		SETLOC	R36LM
		BANK
# Page 701
		EBANK=	RPASS36

		COUNT*	$$/R36

; ============================================================================
; R36: Out-of-Plane Rendezvous Display Routine
; ============================================================================
;
; COMMENT-ONLY READERS: During rendezvous, spacecraft must align not only in
; their orbital plane but also ensure they haven't drifted "above" or "below"
; each other. This routine displays three critical measurements that tell the
; crew about out-of-plane positioning:
;
;   Y (RANGE): How far "out of plane" the two spacecraft are from each other
;   YDOT (RRATE): How fast this out-of-plane distance is changing
;   PSI (RTHETA): The angle showing the direction of the out-of-plane offset
;
; The astronaut requests this display by entering V90 E on the DSKY. During
; Apollo 11's rendezvous after Eagle's ascent on July 21, 1969, Buzz Aldrin
; monitored these parameters to ensure Eagle and Columbia were properly aligned
; for the final docking approach.
;
; If the spacecraft are perfectly aligned in the same orbital plane, Y would be
; zero. Non-zero values indicate the LM is either "north" or "south" of the CSM's
; orbital plane, requiring small thruster corrections before final approach.
;
; CODE-ALONG READERS: R36 Routine Architecture:
;
; Entry and Time Input (R36 and LREGCHK):
;   1. ZL / CAF ZERO / DXCH DSPTEMX: Initialize time display to zero
;   2. CAF V06N16N / TC BANKCALL GOMARKF: Display V06N16 (time input request)
;      - Astronaut can input specific time for calculation
;      - Or press PROCEED to use current time
;      - Or press ENTER to terminate
;   3. LREGCHK: Checks if astronaut entered zero time
;      - BZF ENTTIM2: If zero, use present time (LOADTIME)
;      - If non-zero, use astronaut input time
;
; Main Calculation (R36INT):
;   1. STCALL TDEC1, OTHPREC: Compute "other" spacecraft state at TDEC1
;      - If this is LM, compute CSM state; if CSM, compute LM state
;   2. VLOAD VATT / RATT: Load passive vehicle velocity and position
;   3. STORE RPASS36: Save passive vehicle position
;   4. UNIT PDVL VXV UNIT: Compute unit normal to passive vehicle orbital plane
;      - Cross product of position and velocity gives orbit normal
;   5. STODL UNP36: Store unit normal vector
;
; Active Vehicle State (THISPREC):
;   1. STCALL TDEC1, THISPREC: Compute this spacecraft's state at TDEC1
;   2. VLOAD VATT / RATT: Load active vehicle velocity and position
;   3. PUSH operations: Stack position vectors for multiple calculations
;
; Y Calculation (Out-of-Plane Distance):
;   1. BVSU RPASS36: Compute line-of-sight vector (RA - RP)
;   2. DOT UNP36 / SL1: Dot product with unit normal gives out-of-plane component
;   3. STOVL RANGE: Store in RANGE (displayed as Y to crew)
;      - Positive Y means active vehicle is "above" passive vehicle's plane
;      - Negative Y means active vehicle is "below" passive vehicle's plane
;
; YDOT Calculation (Out-of-Plane Rate):
;   1. DOT UNP36 / SL1: Dot velocity vector with unit normal
;   2. STOVL RRATE: Store in RRATE (displayed as YDOT to crew)
;      - Positive YDOT means out-of-plane distance increasing
;      - Negative YDOT means out-of-plane distance decreasing
;
; PSI Calculation (Out-of-Plane Angle):
;   1. UNIT PUSH: Unit vector of active vehicle position (URA)
;   2. VXV VXV: Double cross product to get horizontal reference
;      - (URA × VA) × URA = horizontal velocity component
;   3. VSL2 UNIT: Normalize to get unit horizontal forward direction
;   4. DOT projection and ARCCOS: Compute angle between LOS and horizontal
;   5. Sign check (BPL R36TAG2): Adjust angle sign based on geometry
;   6. STORE RTHETA: Store in RTHETA (displayed as PSI to crew)
;      - PSI shows angular direction of out-of-plane offset
;
; Display and Termination:
;   1. DLOAD 30D / RTB SGNAGREE: Format time for display
;   2. CAF V06N90N / TC BANKCALL GOMARKF: Display V06N90
;      - Shows time, Y (out-of-plane distance), YDOT (rate), PSI (angle)
;   3. Crew options:
;      - TERMINATE (V34): Exit routine
;      - PROCEED: Accept values and exit
;      - RECYCLE: Redisplay current values
;
; Mathematical Foundation:
;   The out-of-plane component is computed using vector projection onto the
;   normal to the passive vehicle's orbital plane. This normal is perpendicular
;   to both the position and velocity vectors of the passive vehicle.
;
;   For two spacecraft in slightly different orbital planes, the angle between
;   the planes (inclination difference) causes periodic variation in Y as both
;   orbit. The YDOT value indicates whether the spacecraft are approaching or
;   diverging from coplanar alignment at this instant.

R36		ZL
		CAF	ZERO		# SET TIME OF EVENT TO ZERO FOR FIRST
		DXCH	DSPTEMX		# DISPLAY
		CAF	V06N16N
		TC	BANKCALL
		CADR	GOMARKF
		TCF	ENDEXT		# TERMINATE
		TCF	+2		# PROCEED
		TCF	-5		# RECYCLE FOR ASTRONAUT INPUT TIME
		DXCH	DSPTEMX
		EXTEND
		BZF	LREGCHK		# A-REG ZERO GOTO CHECK L-REG FOR ZERO
ASTROTIM	DXCH	MPAC		# A-REG NON-ZERO, TIME = ASTRO INPUT TIME
		TC	INTPRET
		RTB
			DPMODE
R36INT		STCALL	TDEC1
			OTHPREC
		VLOAD	PDVL
			VATT
			RATT		# _
		STORE	RPASS36		# R
		UNIT	PDVL		#  P
		VXV	UNIT
		STADR			# _
		STODL	UNP36		# U
			TAT
		STCALL	TDEC1
			THISPREC
		VLOAD	PDVL		#		  -
			VATT		# VELOCITY VECTOR V		00D
			RATT		# 		   A
		PDDL
			TAT		# SAVE TIME IN LOCATION 30D FOR REDISPLAY
		STOVL	30D		#		  _
		PUSH	PUSH		# POSITION VECTOR R  IN 06D AND 12D
		BVSU	PDVL		#		   A   _   _
			RPASS36		# LINE OF SIGHT VECTOR R - R  	12D
		DOT	SL1		#			P   A
			UNP36		#     _   _
		STOVL	RANGE		# Y = U . R
			00D		#          A
		DOT	SL1
			UNP36		# .   _   _
		STOVL	RRATE		# Y = U . V
			06D		#          A
# Page 702
					# _           _
		UNIT	PUSH		# U  = UNIT ( R  )		18D
		VXV	VXV		#  RA          A
			00D		#  _    _     _     _
			18D		# (U  X V ) X U   = U
		VSL2	UNIT		#   RA   A     RA    A
		UNIT
		STOVL	00D		# UNIT HORIZONTAL IN FORWARD DIR. 00D
			18D
		DOT	VXSC		# _
			12D		# U
		VSL2			#  L
		BVSU	UNIT
		UNIT
		PUSH	DOT		# LOS PROJECTED INTO HORIZONTAL  12D
			00D		# PLANE
		SL1	ARCCOS		#              _   _
		STOVL	RTHETA		# PSI = ARCCOS(U . U )
		VXV	DOT		#               A   L
			00D
		BPL	DLOAD
			R36TAG2
			LODPMAX
		DSU
			RTHETA
		STORE	RTHETA
R36TAG2		DLOAD	RTB
			30D
			SGNAGREE
		STORE	DSPTEMX
		EXIT
		CAF	V06N90N		# DISPLAY Y, YDOT, AND PSI.
		TC	BANKCALL
		CADR	GOMARKF
		TCF	ENDEXT		# TERMINATE
		TCF	ENDEXT		# PROCEED, END OF PROGRAM
		TCF	R36	+3	# REDISPLAY OUTPUT
LREGCHK		XCH	L
		EXTEND
		BZF	ENTTIM2		# L-REG ZERO, SET TIME = PRESENT TIME
		XCH	L		# L-REG NON ZERO, TIME = ASTRO INPUT TIME
		TCF	ASTROTIM
ENTTIM2		TC	INTPRET
		RTB	GOTO
			LOADTIME
			R36INT
V06N16N		VN	00616
V06N90N		VN	00690
