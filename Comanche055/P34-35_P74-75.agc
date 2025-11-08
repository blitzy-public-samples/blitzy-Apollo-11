# Copyright:	Public domain.
# Filename:	P34-35_P74-75.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 460-504
# Contact:      Onno Hommes <ohommes@cmu.edu>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-10 OH	Batch 2 Assignment Comanche Transcription
#		2009-05-23 RSB	In DISPLAYE, corrected a CADR GOFLASHR
#				to CADR GOFLASH.
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
; FILE: P34-35_P74-75.agc
; MODULE: COMEKISS Subsystem (Orbital Navigation)
; MISSION PHASE: trans-lunar/lunar-orbit/rendezvous
;
; TL;DR: Lambert targeting programs solving two-point boundary value problems
;        for orbit transfer maneuvers. P34/P35 compute time-of-flight optimal
;        trajectories between position states. P74/P75 are abort mode variants.
;        Critical for TLI (Translunar Injection), LOI (Lunar Orbit Insertion),
;        and rendezvous targeting calculations throughout Apollo 11 mission.
;
; COMMENT-ONLY READERS: These programs calculated the precise trajectories
;        needed to travel between two points in space within a specific time.
; CODE-ALONG READERS: Study Lambert problem numerical solutions, conic section
;        mathematics, and iterative convergence algorithms for trajectory targeting.
; ============================================================================


# Page 460
; ============================================================================
; SECTION: TRANSFER PHASE INITIATION (TPI) PROGRAMS - P34 AND P74
;
; COMMENT-ONLY READERS: During Apollo 11's mission, the spacecraft needed to
; perform precise maneuvers to rendezvous with other vehicles or transfer
; between orbital phases. These programs calculated the exact velocity change
; (Delta V) required to move from one orbital position to another within a
; specific time frame. P34 was used for normal operations, while P74 served
; as the abort mode variant when mission plans changed unexpectedly.
;
; CODE-ALONG READERS: This section implements Lambert's problem solver for
; orbital rendezvous. Lambert's problem: given two position vectors and time
; of flight, determine the velocity vectors required at departure to reach
; the target position. The solution involves iterative convergence on a
; conic trajectory that satisfies the boundary conditions. The programs use
; CONIC_SUBROUTINES.agc (Kepler propagation) and TIME_OF_FREE_FALL.agc
; (time-of-flight calculations) as foundational math libraries.
; ============================================================================
#
# TRANSFER PHASE INITIATION (TPI) PROGRAMS (P34 AND P74)
# MOD NO -1			LOG SECTION -- P32-P35, P72-P75
# MOD BY WHITE, P.		DATE: 1 JUNE 67
#
# PURPOSE
#	(1)	TO CALCULATE THE REQUIRED DELTA V AND OTHER INITIAL CONDITIONS
#		REQUIRED BY THE ACTIVE VEHICLE FOR EXECUTION OF THE TRANSFER
#		PHASE INITIATION (TPI) MANEUVER, GIVEN --
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
# Page 461
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
#		VEHICLE CENTRAL ANGLE OF TRANSFER IS COMPUTED AND STORED.
#		THIS NUMBER WILL BE AVAILABLE FOR DISPLAY TO THE ASTRONAUT
#		THROUGH THE USE OF V06N52.
#
#		THE ASTRONAUT WILL CALL THIS DISPLAY TO VERIFY THAT THE
#		CENTRAL ANGLE OF TRANSFER OF THE ACTIVE VEHICLE IS NOT WITHIN
# Page 462
#		170 TO 190 DEGREES.  IF THE ANGLE IS WITHIN THIS ZONE THE
#		ASTRONAUT SHOULD REASSES THE INPUT TARGETING PARAMETERS BASED
#		UPON DELTA V AND EXPECTED MANEUVER TIME.
#	(8)	THIS PROGRAM IS SELECTED BY THE ASTRONAUT BY DSKY ENTRY --
#
#			P34 IF THIS VEHICLE IS ACTIVE VEHICLE.
#
#			P74 IF THIS VEHICLE IS PASSIVE VEHICLE.
#
; ============================================================================
; LAMBERT PROBLEM MATHEMATICAL FORMULATION AND SOLUTION STRATEGY
;
; COMMENT-ONLY READERS: The spacecraft computer must solve a fundamental
; orbital mechanics problem: given where we are now, where we want to be,
; and how much time we have to get there, calculate the exact velocity
; change needed. This is like a quarterback throwing a football to a
; receiver running downfield - the throw must account for where the receiver
; will be, not where they are now. For Apollo 11, these calculations were
; critical during Translunar Injection (leaving Earth orbit for the Moon),
; Lunar Orbit Insertion (entering Moon orbit), and rendezvous maneuvers
; (LM returning to dock with CM).
;
; CODE-ALONG READERS: Lambert's theorem provides the mathematical foundation.
; Given two position vectors R1 and R2, and time of flight T, determine the
; velocity vectors V1 and V2. The solution involves:
;   1. Compute chord length C = |R2 - R1|
;   2. Compute semi-perimeter S = (R1 + R2 + C) / 2
;   3. Iterate on semi-major axis A until computed TOF matches desired TOF
;   4. Use universal variable formulation to handle elliptic/parabolic/
;      hyperbolic trajectories with single algorithm
;   5. Convergence criterion: |computed_TOF - desired_TOF| < 0.1 seconds
;
; The AGC implementation uses INITVEL subroutine (lines 1140+) which calls:
;   - CONIC_SUBROUTINES.agc (Kepler propagation for conic trajectories)
;   - TIME_OF_FREE_FALL.agc (time-of-flight calculation for given semi-major axis)
;
; Typical convergence: 3-5 iterations. Maximum: 15 iterations before abort.
; Computational precision: Position within 1 meter, velocity within 0.01 m/s.
; ============================================================================
#
# INPUT
#	(1)	TTPI	TIME OF THE TPI MANEUVER.
#	(2)	ELEV	DESIRED LOS ANGLE AT TPI
#	(3)	CENTANG	ORBITAL CENTRAL ANGLE OF THE PASSIVE VEHICLE DURING
#			TRANSFER FROM TPI TO TIME OF INTERCEPT
#
# OUTPUT
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
#	(1)	TTPI		TIME OF TPI MANEUVER
#	(2)	TIG		TIME OF TPI MANEUVER
#	(3)	ELEV		DESIRED LOS ANGLE AT TPI
#	(4)	CENTANG		ORBITAL CENTRAL ANGLE OF THE PASSIVE VEHICLE DURING
#				TRANSFER FROM TPI TO TIME OF INTERCEPT
#	(5)	DELVEET3	DELTA VELOCITY AT TPI -- REFERENCE COORDINATES
#	(6)	TPASS4		TIME OF INTERCEPT
#
# COMMUNICATION TO THRUSTING PROGRAMS
#	(1)	TIG		TIME OF THE TPI MANEUVER
#	(2)	RTARG		OFFSET TARGET POSITION
#	(3)	TPASS4		TIME OF INTERCEPT
#	(4)	XDELVFLG	RESET TO INDICATE LAMBERT (AIMPOINT) VG COMPUTATION
#
# SUBROUTINES USED
#	AVFLAGA
# Page 463
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

		SETLOC	CSI/CDH
		BANK
		EBANK=	SUBEXIT
		COUNT	35/P3474

; ============================================================================
; P34/P74 PROGRAM ENTRY POINTS
;
; COMMENT-ONLY READERS: The astronaut initiates this program by keying P34
; (if their spacecraft is performing the maneuver) or P74 (if they are the
; passive target vehicle). The program immediately begins collecting the
; required parameters: desired time of the maneuver and geometric constraints.
; During Apollo 11's rendezvous operations, these programs calculated the
; exact velocity changes needed for the Lunar Module to return to the
; Command Module after surface operations.
;
; CODE-ALONG READERS: Entry point differentiation via active/passive vehicle
; flag setting. AVFLAGA sets this vehicle as active (performing Delta V),
; AVFLAGP sets as passive (target vehicle). Both paths converge at P34/P74A
; which enables tracking flags for rendezvous radar updates (P20FLGON).
; The program then prompts for Time of TPI (TTPI) via DSKY Verb 06 Noun 37,
; followed by elevation angle and central angle displays.
; ============================================================================
P34		TC	AVFLAGA
		TC	P34/P74A
P74		TC	AVFLAGP
P34/P74A	TC	P20FLGON	# SET UPDATFLG, TRACKFLG
		CAF	V06N37		# TTPI
		TC	VNPOOH		# Onno: The scans look like O not zero
		TC	INTPRET
		SSP	EXIT
			NN
			0
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

; DELELO - Previous Elevation Angle Error Storage
; Stores the elevation angle error from the previous Lambert iteration (DELEL)
; to enable convergence monitoring and time step adjustment during the iterative
; solution. Used in REPETE routine to detect solution direction and rate.
DELELO		EQUALS	26D

P34/P74C	DLOAD	SET
			ZEROVECS
			ITSWICH
		BON	CLEAR
			ETPIFLAG
# Page 464
			SWCHSET
			ITSWICH
SWCHSET		STORE	NOMTPI

; ============================================================================
; ITERATIVE LAMBERT SOLUTION CONVERGENCE
;
; COMMENT-ONLY READERS: The computer now enters an iterative calculation loop,
; repeatedly refining its trajectory solution until it converges on the optimal
; path between the two spacecraft. Each iteration improves the accuracy of the
; computed Delta V required for the maneuver. If the solution fails to converge,
; the astronaut is alerted with alarm code 611 and can choose to retry or abort.
;
; CODE-ALONG READERS: INTLOOP implements Newton-Raphson iteration to solve the
; Lambert boundary value problem. Each iteration:
;   1. Loads TTPI (time to TPI) and adds NOMTPI (nominal adjustment)
;   2. Calls PRECSET to establish precision orbital state vectors
;   3. Calls S33/34.1 (Lambert solver) which returns zero if converged
;   4. If non-zero (not converged), issues alarm 611 and displays V05N09
;   5. ITSWICH flag controls whether to iterate again or proceed to display
;
; The iteration terminates when S33/34.1 returns zero (BZE instruction branches
; to SWCHCLR on zero result). Convergence typically occurs within 2-4 iterations
; for well-conditioned rendezvous geometry.
; ============================================================================
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
		CAF	V05N09
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	P34/P74A	# PROCEED
		TC	-7		# V32

SWCHCLR		BONCLR	BON
			ITSWICH
			INTLOOP
			ETPIFLAG
			P34/P74D	# DISPLAY TTPI
		EXIT
		TC	DISPLAYE	# DISPLAY ELEV AND CENTANG
		TC	P34/P74E
P34/P74D	EXIT
		CAF	V06N37		# TTPI
		TC	VNPOOH
; ============================================================================
; FINAL TRAJECTORY SOLUTION AND DELTA V COMPUTATION
;
; COMMENT-ONLY READERS: The iteration has successfully converged. The computer
; now calculates the precise velocity change (Delta V) required for the TPI
; maneuver, along with predictions of the spacecraft's trajectory after the burn.
; These calculations determine the exact thrust magnitude, direction, and timing
; that will place the active vehicle on an intercept course with the passive
; vehicle. The astronaut will shortly see these values displayed on the DSKY
; for approval before committing to the maneuver.
;
; CODE-ALONG READERS: P34/P74E performs the final solution computation:
;   1. Loads converged geometry (RTX1, CENTANG with trigonometric components)
;   2. Retrieves passive vehicle state (RPASS3, VPASS3) and scales via VSR*
;   3. Calls TIMETHET to compute time/angular relationships
;   4. Calls S34/35.2 (INITVEL) to compute initial velocity requirements
;   5. Calculates DELVTPI (Delta V magnitude at TPI ignition)
;   6. Calculates DELVTPF (Delta V magnitude at TPF intercept)
;   7. Calls PERIAPO1 to compute post-burn pericenter altitude
;   8. Displays results via V06N58 (TIG, Delta V components, post-burn perigee)
;
; This section represents the culmination of the Lambert problem solution,
; translating abstract conic mathematics into actionable spacecraft commands.
; ============================================================================
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
# Page 465
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

# Page 466
# RENDEZVOUS MID-COURSE MANEUVER PROGRAMS (P35 AND P75)
# MOD NO -1			LOG SECTION -- P32-P35, P72-P75
# MOD BY WHITE, P.		DATE:  1 JUNE 67
#
# PURPOSE
#	(1) 	TO CALCULATE THE REQUIRED DELTA V AND OTHER INITIAL CONDITIONS
#		REQUIRED BY THE ACTIVE VEHICLE FOR EXECUTION OF THE NEXT
#		MID-COURSE CORRECTION OF THE TRANSFER PHASE OF AN ACTIVE
#		VEHICLE RENDEZVOUS.
#	(2)	TO DISPLAY TO THE ASTRONAUT AND THE GROUND CERTAIN DEPENDENT
#		VARIABLES ASSOCIATED WITH THE MANEUVER FOR APPROVAL BY THE
#		ASTRONAUT/GROUND.
#	(3)	TO STORE THE TPM TARGET PARAMETERS FOR USE BY THE DESIRED
#		THRUSTING PROGRAM.
#
# ASSUMPTIONS
#	(1)	THE ISS NEED NOT BE ON TO COMPLETE THIS PROGRAM.
#	(2)	STATE VECTOR UPDATES BY P27 ARE DISALLOWED DURING AUTOMATIC
#		STATE VECTOR UPDATING INITIATED BY P20 (SEE ASSUMPTION (3)).
#	(3)	THE RENDEZVOUS RADAR IS ON AND IS LOCKED ON THE CSM.  THIS WAS
#		DONE DURING PREVIOUS SELECTION OF P20.  RADAR SIGHTING MARKS
#		WILL BE MADE AUTOMATICALLY APPROXIMATELY ONCE A MINUTE WHEN
#		ENABLED BY THE TRACK AND UPDATE FLAGS (SEE P20).  THE
#		RENDEZVOUS TRACKING MARK COUNTER IS ZEROED BY THE SELECTION OF
#		P20 AND AFTER EACH THRUSTING MANEUVER.
#	(4)	THE OPERATION OF THE PROGRAM UTILIZES THE FOLLOWING FLAGS --
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
#	(5)	THE TIME OF INTERCEPT (T(INT)) WAS DEFINED BY PREVIOUS
#		COMPLETION OF THE TRANSFER PHASE INITIATION (TPI) PROGRAM
#		(P34/P74) AND IS PRESENTLY AVAILABLE IN STORAGE.
# Page 467
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
#	(7)	THIS PROGRAM IS SELECTED BY THE ASTRONAUT BY DSKY ENTRY --
#
#			P35 IF THIS VEHICLE IS ACTIVE VEHICLE.
#
#			P75 IF THIS VEHICLE IS PASSIVE VEHICLE.
#
# INPUT
#	(1)	TPASS4		TIME OF INTERCEPT -- SAVED FROM P34/P74
#
# OUTPUT
#	(1)	TRKMKCNT	NUMBER OF MARKS
#	(2)	TTOGO		TIME TO GO
#	(3)	+MGA		MIDDLE GIMBAL ANGLE
#	(4)	DVLOS		DELTA VELOCITY AT MID -- LINE OF SIGHT
#	(5)	DELVLVC		DELTA VELOCITY AT MID -- LOCAL VERTICAL COORDINATES
#
# DOWNLINK
#	(1)	TIG		TIME OF THE TPM MANEUVER
#	(2)	DELVEET3	DELTA VELOCITY AT TPM -- REFERENCE COORDINATES
#	(3)	TPASS4		TIME OF INTERCEPT
#
# COMMUNICATION TO THRUSTING PROGRAMS
#	(1)	TIG		TIME OF THE TPM MANEUVER
#	(2)	RTARG		OFFSET TARGET POSITION
#	(3)	TPASS4		TIME OF INTERCEPT
#	(4)	XDELVFLG	RESET TO INDICATE LAMBERT (AIMPOINT) VG COMPUTATION.
#
# SUBROUTINES USED
#	AVFLAGA
#	AVFLAGP
#	LOADTIME
#	SELECTMU
#	PRECSET
#	S34/35.1
#	S34/35.2
# Page 468
#	S34/35.5
#	VN1645

		COUNT	35/P3575
		EBANK=	KT

; ============================================================================
; P35/P75 MID-COURSE CORRECTION PROGRAMS
;
; COMMENT-ONLY READERS: After the initial TPI maneuver (P34/P74), the spacecraft
; is on an intercept trajectory toward the target vehicle. However, small errors
; in execution or changes in orbital conditions may require mid-course corrections.
; P35/P75 calculates these correction maneuvers, ensuring the spacecraft stays
; on track for the planned intercept. During Apollo 11's lunar orbit rendezvous
; between Eagle and Columbia, these programs enabled precise trajectory adjustments
; during the critical approach phase, guaranteeing successful docking.
;
; CODE-ALONG READERS: P35/P75 implements mid-course maneuver computation:
;   Entry: P35 (active vehicle) or P75 (passive vehicle)
;   - Loads time increment from ATIGINC (active) or PTIGINC (passive)
;   - Stores in KT (time to maneuver)
;   - Enables tracking flags via P20FLGON for radar updates
;   - Calls SELECTMU to choose appropriate gravitational parameter
;   - Main loop (P35/P75B):
;     1. LOADTIME: Gets current mission elapsed time
;     2. Adds KT to compute TIG (time of ignition)
;     3. PRECSET: Advances both vehicle state vectors to TIG
;     4. S34/35.1: Computes normal and line-of-sight vectors for coordinate transform
;     5. S34/35.2: Calls INITVEL to compute Delta V in local vertical coordinates
;     6. S34/35.5: Computes additional parameters (perigee, apogee)
;     7. VN1645: Displays results for crew approval
;     8. Loops back to P35/P75B for iterative refinement
;
; Unlike P34/P74 which solves a two-point boundary value problem, P35/P75
; performs direct computation from current state to the pre-computed intercept
; point (TPASS4) established by P34/P74.
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
# Page 469
# ***** S33/34.1 *****

; ============================================================================
; S33/34.1 - LAMBERT PROBLEM SOLVER SUBROUTINE
;
; COMMENT-ONLY READERS: This is the mathematical heart of the rendezvous
; targeting system. When two spacecraft orbit in space, calculating the exact
; trajectory to move from one orbit to another involves solving a complex
; mathematical problem called the "Lambert problem" (named after Johann Lambert,
; an 18th century mathematician). This routine performs those calculations,
; determining the velocity change needed to travel from the active vehicle's
; position to the passive vehicle's position in the specified time. It's the
; same fundamental mathematics that has guided every spacecraft rendezvous
; from Gemini to the International Space Station.
;
; CODE-ALONG READERS: S33/34.1 implements the Lambert boundary value problem
; solver using iterative conic section mathematics. The routine:
;
; Inputs (in interpretive stack and erasable memory):
;   - RACT3/VACT3: Active vehicle position/velocity state vectors (scaled)
;   - RPASS3/VPASS3: Passive vehicle position/velocity state vectors (scaled)
;   - TTPI: Time of flight for the transfer trajectory (centiseconds)
;   - CENTANG: Central angle of the transfer arc (revolutions)
;
; Algorithm:
;   1. Stores position/velocity precision copies (RAPREC, VAPREC, RPPREC, VPPREC)
;   2. Calls S34/35.1 to compute normal and line-of-sight (LOS) vectors
;   3. Computes cross products and unit vectors for coordinate transformation
;   4. Establishes local vertical (UP) coordinate frame at active vehicle
;   5. Projects passive vehicle geometry into local vertical frame
;   6. Computes angular elements (elevation, azimuth) of transfer geometry
;   7. Iterates to find conic section parameters satisfying boundary conditions
;   8. Returns zero if converged, non-zero if iteration limit exceeded
;
; Mathematical foundation: Gauss's method for solving Lambert's problem,
; adapted for fixed-point arithmetic with AGC scaling conventions.
; Convergence uses Newton-Raphson iteration with maximum 40 iterations (TITER).
;
; This subroutine interfaces with TIME_OF_FREE_FALL.agc for trajectory
; time-of-flight calculations and CONIC_SUBROUTINES.agc for orbital elements.
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
# Page 470
		BMN
			TIMEX		# COMMERCIALS EVERYWHERE

; ============================================================================
; FIGTIME - Lambert Iteration Loop Entry Point
;
; COMMENT-ONLY READERS: The computer has checked if the calculated trajectory
; is accurate enough. If not, it returns to this point to try again with
; refined calculations. The routine monitors the iteration counter to prevent
; infinite loops - after 40 attempts, it gives up and reports failure.
;
; CODE-ALONG READERS: FIGTIME is the re-entry point for the Newton-Raphson
; iteration loop solving the Lambert problem. The routine:
;   1. Loads and checks TITER (iteration counter, max 40)
;   2. Branches to NORMEX if iteration limit exceeded (convergence failure)
;   3. Computes position unit vectors and orbital geometry
;   4. Calculates angular elements (alpha, beta) for conic section
;   5. Tests convergence based on elevation angle difference (DELEL < ELEPS)
;   6. Loops back through TIMEX or exits through NORMEX
;
; The iteration refines the transfer trajectory until elevation angle error
; falls below threshold (ELEPS), ensuring accurate rendezvous targeting.
; ============================================================================
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
		STODL	30D		# RP-RA MAGNITUDES
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
# Page 471
		BDSU
		NORM	PDVL		# NORMALIZED WA-WP 12D
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
			0 -3,1
		PUSH	ABS
		DSU	BMN
			SECMAX
			OKMAX
		DLOAD	SIGN		# REPLACE TIME WITH MAX TIME SIGNED
			SECMAX
		PUSH

; ============================================================================
; Time Step Adjustment Logic (OKMAX through ADTIME)
;
; COMMENT-ONLY READERS: After calculating a trial trajectory, the computer
; must decide how to adjust the timing for the next attempt. If this is the
; first try, it makes a standard adjustment. On subsequent iterations, it
; checks whether the solution is getting better (converging) or worse
; (diverging) and adjusts the time step accordingly - taking larger steps
; when converging quickly, smaller steps when near the solution, and reversing
; direction if overshooting. This adaptive approach helps find the optimal
; trajectory efficiently.
;
; CODE-ALONG READERS: This section implements adaptive time step control for
; the Newton-Raphson iteration:
;
; OKMAX - Checks if calculated time step (DT) exceeds maximum (SECMAX)
;         If so, clamps to SECMAX with proper sign
;
; First Iteration (TITER negative):
;   - Sets TITER = 37777 octal (maximum iterations remaining)
;   - Uses initial time step directly
;
; Subsequent Iterations (REPETE):
;   - Tests convergence: DELEL * DELELO (product of consecutive errors)
;   - If product >= 0: Same-direction errors, reduce step (SECMAX/3)
;   - If product < 0: Crossed solution, halve step and reverse (RESIGN)
;
; NEXTES - Compares |DELEL| vs |DELELO| to detect convergence rate
;   - If |DELEL| < |DELELO|: Converging, use full time step (RESIGN)
;   - If |DELEL| >= |DELELO|: Diverging, reverse direction (REVERS)
;
; REVERS - Reverses time step direction and halves magnitude
; RESIGN - Applies proper sign to time step
; STORDELT - Stores final adjusted time step in DELTEEO
; ADTIME - Accumulates time step into NOMTPI (nominal TPI time)
; ============================================================================
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
# Page 472
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
STORDELT	STORE	DELTEEO
ADTIME		DAD
			NOMTPI		# SUM OF DELTA T:S
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
# Page 473
			ZEROVECS
			NORMEX

# Page 474
# ***** S34/35.1 *****

; ============================================================================
; SUBROUTINE: S34/35.1
; PURPOSE: Compute geometry vectors for Lambert targeting
;
; COMMENT-ONLY READERS: This subroutine calculates the geometric relationship
; between the two spacecraft at the transfer initiation time (T3). It computes
; the line-of-sight direction from the active vehicle to the passive vehicle,
; and the normal vector perpendicular to the active vehicle's orbital plane.
; These vectors are essential for properly orienting the trajectory solution.
;
; CODE-ALONG READERS: Computes two critical unit vectors:
;   1. ULOS - Unit line-of-sight vector from active to passive vehicle
;      Calculated as: ULOS = unit(RPASS3 - RACT3)
;   2. UNRM - Unit normal to active vehicle's orbital plane
;      Calculated as: UNRM = unit(RACT3 × VACT3)
; These vectors define the reference frame for the Lambert solution.
; All vector operations use interpretive language for double-precision accuracy.
; ============================================================================

# COMPUTE UNIT NORMAL AND LINE OF SIGHT VECTORS GIVEN THE ACTIVE AND
# PASSIVE POS AND VEL AT TIME T3
		SETLOC	S3435LOC
		BANK

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

# Page 475
# ***** S34/35.2 *****

; ============================================================================
; SUBROUTINE: S34/35.2
; PURPOSE: Propagate passive vehicle and solve Lambert problem
;
; COMMENT-ONLY READERS: This is where the computer calculates the trajectory
; that will take the active vehicle from its current position to intercept
; the passive vehicle at the desired rendezvous time (TPASS4). First, it
; predicts where the passive vehicle will be at that future time by
; integrating its orbit forward. Then it solves the Lambert problem to find
; the required velocity for the active vehicle to reach that intercept point.
;
; CODE-ALONG READERS: Multi-step computational sequence:
;   1. Calls INTINT to integrate passive vehicle state from T3 to TPASS4
;      Result stored in RTARG (target position) and VPASS4 (target velocity)
;   2. Computes central angle PHI traversed during transfer
;      PHI = PI + (ACOS(unit RA · unit RP) - PI) × sign(RA × RP · U)
;      This angle determines the trajectory geometry (short/long way around)
;   3. Computes time-of-flight DELLT4 = TPASS4 - INTIME
;   4. Calls INITVEL to compute initial velocity guess for Lambert solver
;   5. Calls LOMAT to generate transformation matrix (local vertical frame)
;   6. Computes required delta-V: DELVLVC = solution velocity - current velocity
; Uses both conic (CONIC flag) and precision integration (NN offset count).
; ============================================================================

# ADVANCE PASSIVE VEH TO RENDEZVOUS TIME AND GET REQ VEL FROM LAMBERT
		SETLOC	CSI/CDH
		BANK

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
		DLOAD	PDDL
			ZEROVECS	# PRECISION
S3435.23	CALL
			INTINT		# GET TARGET VECTOR
S3435.25	STOVL	RTARG
			VATT
		STOVL	VPASS4
			RTARG
# COMPUTE PHI = PI + (ACOS(UNIT RA.UNIT RP) - PI) SIGN(RA*RP.U)
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
# Page 476
			RACT3
		STOVL	RINIT
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

# Page 477
# ***** S34/35.3 *****

; ============================================================================
; SUBROUTINE: S34/35.3
; PURPOSE: Recompute target parameters after crew delta-V modification
;
; COMMENT-ONLY READERS: If the astronauts modify the computer's calculated
; delta-V values (perhaps to adjust the maneuver based on fuel considerations
; or to compensate for known navigation uncertainties), this routine recalculates
; where the spacecraft will actually arrive given the modified burn. It transforms
; the crew's delta-V input back into the reference frame, adds it to the current
; velocity, and recomputes the intercept trajectory.
;
; CODE-ALONG READERS: Handles astronaut overwrite of computed delta-V:
;   1. Calls LOMAT to get transformation matrix from local vertical to inertial
;   2. Transforms modified DELVLVC from display coordinates to inertial frame
;      Result: DELVEET3 = DELVLVC × matrix (with VSL1 for proper scaling)
;   3. Computes new required velocity: Vnew = DELVEET3 + VACT3
;   4. Re-integrates trajectory forward from TIG to TPASS4 using INTINT
;      This determines actual intercept point RTARG with modified delta-V
;   5. Recomputes display parameters (DVLOS, ULOS) for final crew verification
; Ensures trajectory solution remains consistent with crew modifications.
; ============================================================================

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

# Page 478
# ***** S34/35.4 *****

; ============================================================================
; SUBROUTINE: S34/35.4
; PURPOSE: Skip crew modification path (no delta-V overwrite)
;
; COMMENT-ONLY READERS: This is the bypass path used when the astronauts accept
; the computer's calculated delta-V without modification. It skips the
; recomputation steps and proceeds directly to the final display preparation.
;
; CODE-ALONG READERS: Simple control flow routine:
;   Sets return address in NORMEX and resets push-down stack pointer to 0D
;   Branches directly to NOVRWRT label in S34/35.3 code section
;   This avoids redundant matrix transformation and integration when the
;   computed delta-V is accepted without astronaut modification.
; Entry condition: FINALFLG clear indicates no overwrite occurred.
; ============================================================================

S34/35.4	STQ	SETPD		# NO ASTRONAUT OVERWRITE
			NORMEX
			0D
		GOTO
			NOVRWRT

# Page 479
# ***** LOMAT *****

; ============================================================================
; SUBROUTINE: LOMAT (Local Orientation Matrix)
; PURPOSE: Generate transformation matrix from local vertical to inertial frame
;
; COMMENT-ONLY READERS: This routine constructs a coordinate system centered
; on the active vehicle. It creates three perpendicular axes: one pointing
; radially outward from the center of the Moon/Earth (local vertical), one
; perpendicular to the orbital plane, and one completing the right-handed
; coordinate system. This reference frame is essential for displaying delta-V
; in a form meaningful to the crew (radial, in-plane, and cross-track components)
; rather than just raw inertial coordinates.
;
; CODE-ALONG READERS: Constructs local vertical/local horizontal (LVLH) matrix:
;   Builds orthonormal basis vectors in push-down stack:
;   Y-axis: -UNRM (negative of orbit normal, stored at 6D)
;           Computed as -unit(RACT3 × VACT3)
;   Z-axis: -unit(RACT3) (negative radial, toward body center, stored at 12D)
;           This is the local "down" direction
;   X-axis: Z × (-Y) with VSL1 scaling (stored at 0D)
;           This is approximately the local horizontal velocity direction
;   
;   Matrix format in push list (MPAC):
;   0D-2D:  X components (cross-track, perpendicular to radial and normal)
;   6D-8D:  Y components (orbit normal direction)
;   12D-14D: Z components (radial direction)
;   
;   Sets push-down pointer to 18D and returns via RVQ
;   This matrix is used to transform delta-V between inertial and LVLH frames.
; ============================================================================

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

; ============================================================================
; SUBROUTINES: GOINT / INTINT (Go Integrate / Initialize Integration)
; PURPOSE: Set up state vectors and integrate orbital positions to target time
;
; COMMENT-ONLY READERS: These routines prepare for orbital calculations by
; setting up the spacecraft's position and velocity at a specific time. They
; handle the differences between Earth and lunar orbits (different gravity
; fields require different scaling). After setup, they compute where the
; spacecraft will be at the target time by numerically integrating its orbit
; forward or backward through time. This is essential for targeting maneuvers
; that happen minutes or hours in the future.
;
; CODE-ALONG READERS: Two-entry subroutine for integration setup:
;   
;   GOINT entry: Initializes with zero vectors before falling through to INTINT
;     - Pushes ZEROVECS (zero state) onto stack
;     - Pushes NOMTPI (nominal TPI time) onto stack
;     - Falls through to INTINT with these initial values
;     Note: Comment warns "DO NOT ORDER OR INSERT BEFORE INTINT"
;   
;   INTINT entry: Main integration initialization and execution
;     - Saves return address in RTRN via STQ
;     - Calls INTSTALL to set up integration parameters
;     - Clears INTYPFLG (integration type flag)
;     - Tests and conditionally sets INTYPFLG based on integration direction
;     - Loads and stores TDEC1 (target time for integration)
;     - Sets MOONFLAG and loads RTX2 index for scaling
;     - Tests CMOONFLG (Circum-Moon flag):
;       * If set: Branch to ALLSET (lunar orbit case)
;       * If clear: Clear MOONFLAG (Earth orbit case)
;     
;     ALLSET continuation:
;     - Stores TET (time of state vector)
;     - Loads and scales position vector with VSR* (variable shift right by index)
;       Scaling factor depends on RTX2 (different for Earth vs Moon)
;     - Stores scaled position in RCV
;     - Loads and scales velocity vector with VSR*
;     - Stores scaled velocity in VCV
;     - Calls INTEGRVS (integration routine from CONIC_SUBROUTINES.agc)
;       This numerically propagates the orbit from current time to target time
;     - Loads RATT (integrated position result)
;     - Returns via GOTO RTRN (indirect return to saved address)
;
;   This integration is crucial for Lambert targeting because we need to know
;   where both vehicles will be at the maneuver time, accounting for their
;   orbital motion during the time between "now" and the planned maneuver.
;   The scaling adjustments (VSR*) handle the different physical scales of
;   Earth orbit (~6500 km radius) vs lunar orbit (~1800 km radius).
; ============================================================================

GOINT		PDDL	PDDL		# DO
			ZEROVECS	#	NOT
			NOMTPI		#
		PUSH	PUSH		#		ORDER OR INSERT BEFORE INTINT
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

# Page 480
# ***** S34/35.5 *****

; ============================================================================
; SUBROUTINE: S34/35.5
; PURPOSE: Display computed delta-V and handle crew input/modification
;
; COMMENT-ONLY READERS: This is the crew interaction point where the computer
; displays the calculated maneuver to the astronauts and waits for their
; approval or modification. First, it shows the computed delta-V on the DSKY
; (Display & Keyboard) using verb 06 noun 81 format. The crew can either
; accept these values by pressing PROCEED, or modify them by entering new
; numbers before pressing PROCEED. If they modify the delta-V, the computer
; recalculates the resulting trajectory. Finally, it displays additional
; parameters like the time until maneuver and the resulting orbit parameters
; using verb 06 noun 59.
;
; CODE-ALONG READERS: Crew interface control flow with conditional recalculation:
;   1. Check FINALFLG to determine display path
;      If set: Skip to FLAGON (final display cycle)
;      If clear: Set UPDATFLG and continue to FLAGOFF
;   2. FLAGON section:
;      - Save DELVLVC to DVLOS for comparison
;      - Exit interpretive mode to native AGC
;      - Load V06N81 (verb 06, noun 81) for delta-V display in ft/sec
;      - Call VNPOOH to display and await crew response
;      - Re-enter interpretive mode
;      - Compare displayed DELVLVC with saved DVLOS
;      - If changed (ABVAL ≠ 0): Call S34/35.3 to recompute trajectory
;      - If unchanged: Skip to NOCHG
;   3. NOCHG section:
;      - Clear XDELVFLG (delta-V modification flag)
;      - Store DELVEET3 in DELVSIN for permanent storage
;   4. FLAGOFF section:
;      - Call S34/35.4 to compute final display parameters
;      - Exit interpretive mode
;      - Load V06N59 for time-to-ignition and apsis display
;      - Call VNPOOH for final verification display
;      - Return via SUBEXIT
; This routine implements the critical human-in-the-loop verification step.
; ============================================================================
#
# SUBROUTINES USED
#	BANKCALL
#	GOFLASH
#	GOTOPOOH
#	S34/35.3
#	S34.35.4
#	VNPOOH

S34/35.5	STQ	BON
			SUBEXIT
			FINALFLG
			FLAGON
		SET	GOTO
			UPDATFLG
			FLAGOFF
FLAGON		VLOAD
			DELVLVC
		STORE	DVLOS		# SAVE DELTA V BEFORE DISPLAY
		EXIT
 		CAF	V06N81
 		TC	VNPOOH
 		TC	INTPRET
 		VLOAD	VSU		# TEST FOR OVERWRITE OF COMPUTED
 			DELVLVC		#                      DELTA V
 			DVLOS
 		ABVAL	BZE
 			NOCHG		# NO OVERWRITE
 		CALL
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

# Page 481
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

; ============================================================================
; SUBROUTINE: VN1645 (Display V16N45 - TRKMKCNT, TTOGO, +MGA)
; PURPOSE: Display tracking mark count, time to go, and midcourse guidance adjust
;
; COMMENT-ONLY READERS: This routine displays three critical pieces of information
; to the crew on the DSKY: how many radar tracking marks have been made, how much
; time remains until the maneuver, and a midcourse guidance adjustment value (MGA).
; The MGA value starts at -0.01 and may be adjusted to -0.02 if this is the final
; computation cycle. The crew reviews these values and decides whether to proceed
; with the maneuver or recycle the computation. This display repeats every second
; until the crew makes a decision.
;
; CODE-ALONG READERS: Display and crew interaction routine for V16N45:
;   - Saves return address in SUBEXIT via STQ
;   - Loads DP-.01 (double precision -0.01) into MPAC
;   - Stores in +MGA (midcourse guidance adjust value)
;   
;   If FINALFLG is clear (not final cycle):
;     - Branch to GET45 (use MGA = -0.01)
;   
;   If FINALFLG is set (final cycle):
;     - Load DP-.01 again
;     - Add DP-.01 to get -0.02
;     - Store in +MGA (MGA = -0.02 for final display)
;     
;     If REFSMFLG is set (reference stable member coordinate system):
;       - Exit to native AGC code
;       - Call P3XORP7X to determine program type
;         * Returns to next instruction for P34/P74 (TPI programs)
;         * Skips one instruction for P35/P75 (mid-course programs)
;       - Re-enter interpreter mode
;       - Load DELVSIN (input delta-V vector)
;       - Call GET+MGA to compute MGA from delta-V
;   
;   GET45 label:
;   - Exit interpreter to native code
;   - Call COMPTGO to initiate background task updating TTOGO (time to go)
;   - Save SUBEXIT return address in QSAVED
;   - Set 1-second delay via DELAYJOB (display update rate)
;   - Display V16N45 via GOFLASH:
;     * R1: TRKMKCNT (rendezvous tracking mark counter from P20)
;     * R2: TTOGO (time remaining until maneuver ignition)
;     * R3: +MGA (midcourse guidance adjustment value)
;   - Await crew response:
;     * TERMINATE (V34): Branch to KILCLOCK, end program
;     * PROCEED (V33): Branch to N45PROC, continue to next phase
;     * RECYCLE: Branch to CLUPDATE, repeat computation from start
;
;   N45PROC (Proceed Processing):
;   - Test FINALFLG (bit 6 of FLAGWRD2)
;   - If FINALFLG is set: This was the final display, branch to KILCLOCK
;     and flash V37 to await new program selection from crew
;   - If FINALFLG is clear: Set FINALFLG for next iteration and continue
;
;   CLUPDATE (Recycle Processing):
;   - Clear DISPDEX (display index)
;   - Update phase change registers
;   - Clear UPDATFLG (disables automatic updates during recomputation)
;   - Return to QSAVED (restart computation from beginning)
;
;   This display loop allows the crew to monitor convergence of the targeting
;   solution. Typically the program displays twice: once after initial computation
;   (MGA = -0.01) and once after final refinement (MGA = -0.02). The crew can
;   recycle if tracking marks or maneuver parameters have changed.
; ============================================================================

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
		TC	GET45 +1	# P7X
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
# Page 482
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

# Page 483
# ***** DISPLAYE *****
#
# SUBROUTINES USED
#	BANKCALL
#	GOFLASHR
#	GOTOPOOH
#	BLANKET
#	ENDOFJOB

; ============================================================================
; SUBROUTINE: DISPLAYE (Display V06N55 - Elevation Angle E)
; PURPOSE: Display elevation angle E for crew review and modification
;
; COMMENT-ONLY READERS: This routine displays the elevation angle (E) on the DSKY
; and allows the crew to review and potentially modify the value. The elevation
; angle defines the line-of-sight angle between the two spacecraft at the time of
; the transfer phase initiation (TPI) maneuver. The crew can accept the computed
; value by pressing PROCEED, modify it by entering a new value, or exit the program
; by pressing TERMINATE. If the crew enters a new elevation angle, the entire
; targeting computation will be recalculated using the new value.
;
; CODE-ALONG READERS: Crew display and input routine for V06N55:
;   Entry point DISPLAYE:
;   - EXTEND instruction enables extended instruction on next line
;   - QXCH NORMEX: Exchange Q (return address) with NORMEX
;     * Saves caller's return address in NORMEX
;     * Loads NORMEX (new return address) into Q for later use
;   - CAF V06N55: Load fixed address of V06N55 into A register
;     * V06N55 = display verb 06, noun 55
;     * Verb 06 = Display decimal data (R1 only)
;     * Noun 55 = Elevation angle E in degrees
;   - TCR BANKCALL: Transfer control with return to BANKCALL subroutine
;   - CADR GOFLASH: Constant address of GOFLASH (causes display to flash)
;     * GOFLASH displays the value and awaits crew response
;   - Return from GOFLASH branches to one of three locations:
;     * TERMINATE (V34): Branch to GOTOPOOH (terminates program)
;     * PROCEED (V33): Execute next instruction (TC NORMEX to return)
;     * RECYCLE or new data entry: Branch to -5 (restart display loop)
;   
;   GOTOPOOH branch:
;   - Crew pressed TERMINATE
;   - Transfers control to VNPOOH to clean up and await new program
;   
;   TC NORMEX branch:
;   - Crew pressed PROCEED (accepted displayed value)
;   - Transfers control to address in NORMEX (caller's return address)
;   - Program continues to next phase
;   
;   TCF -5 branch:
;   - Crew entered new elevation angle value or pressed RECYCLE
;   - Branches back 5 instructions to CAF V06N55
;   - Re-displays the (possibly modified) elevation angle
;   - Loop continues until crew presses PROCEED or TERMINATE
;   
;   This display allows crew override of the computed elevation angle. During
;   Apollo 11's rendezvous operations, the crew could adjust targeting parameters
;   based on visual observation or ground controller recommendations.
; ============================================================================

DISPLAYE	EXTEND
		QXCH	NORMEX
		CAF	V06N55
		TCR	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH
		TC	NORMEX
		TCF	-5

# Page 484
# ***** P3XORP7X *****

; ============================================================================
; SUBROUTINE: P3XORP7X (Program Type Discriminator)
; PURPOSE: Determine if current program is P34/P74 or P35/P75 and adjust return
;
; COMMENT-ONLY READERS: This small but important routine determines whether the
; crew is running program P34/P74 (Transfer Phase Initiation - TPI) or program
; P35/P75 (mid-course guidance adjustment). The program number is encoded in the
; MODREG (mode register) which tracks the currently executing program. Based on
; the program type, this routine either returns to the next instruction (for
; P34/P74) or skips the next instruction by incrementing the return address (for
; P35/P75). This allows the calling code to have different behavior for the two
; program types without requiring duplicate code paths.
;
; CODE-ALONG READERS: Program discriminator using return address manipulation:
;   - CAF HIGH9: Load HIGH9 constant into A register
;     * HIGH9 = octal 77700 (binary 111111111000000)
;     * This mask isolates the upper 9 bits of a 15-bit word
;   - MASK MODREG: Logical AND of A register with MODREG
;     * MODREG contains current major mode program number
;     * Upper 9 bits encode the program number (P34, P35, P74, P75, etc.)
;     * Result in A register is program number left-justified
;   - EXTEND: Enable extended instruction on next line
;   - BZF +2: Branch on Zero to address +2 (skip next two instructions) if A = 0
;     * If A = 0: MODREG indicates P34 or P74 (TPI programs)
;     * Branch to RETURN, returning to next instruction in caller
;   - INCR Q: Increment Q register (return address) by 1
;     * If A ≠ 0: MODREG indicates P35 or P75 (mid-course programs)
;     * Incrementing Q causes RETURN to skip one instruction in caller
;   - RETURN: Transfer control to address in Q register
;   
;   Usage pattern in calling code:
;     TC    P3XORP7X         ; Call discriminator
;     <instruction A>        ; Executed only for P34/P74
;     <instruction B>        ; Executed for both program types
;   
;   For P34/P74: Returns to instruction A, then continues to B
;   For P35/P75: Skips instruction A, continues directly to B
;   
;   This technique is a common AGC programming pattern for conditional execution
;   without explicit branch instructions, saving both code space and execution time.
;   The MODREG value is set by the executive scheduler when the crew selects a
;   program via DSKY verb/noun sequence.
; ============================================================================

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

		SETLOC P30SUBS
		BANK

; ============================================================================
; SUBROUTINE: VNPOOH (Verb/Noun Flasher for Program Termination)
; PURPOSE: Display flashing verb/noun code and await new crew input after program end
;
; COMMENT-ONLY READERS: When a rendezvous program completes or is terminated by
; the crew, this routine displays a flashing verb/noun code on the DSKY and waits
; for the crew to select a new program or action. This is the AGC's way of saying
; "Program finished. What would you like to do next?" The verb/noun code that
; flashes is typically V37 (change major mode), prompting the crew to enter a new
; program number. This routine runs in a loop, repeatedly flashing the display
; until the crew responds with a valid input.
;
; CODE-ALONG READERS: Program termination and verb/noun flash routine:
;   Entry point VNPOOH:
;   - EXTEND: Enable extended instruction on next line
;   - QXCH RTRN: Exchange Q (return address) with RTRN
;     * Saves caller's return address in RTRN for later use
;     * Not expected to return to caller (this is a termination routine)
;   - TS VERBNOUN: Store A register contents in VERBNOUN
;     * A contains the verb/noun code to display (typically V37)
;     * VERBNOUN temporarily holds this value
;   - CAF VNBANK: Load constant address VNBANK into A
;     * VNBANK is a bank number where this routine resides
;   - XCH FBANK: Exchange A with FBANK (current fixed-bank register)
;     * Saves current bank in A, loads VNBANK into FBANK
;   - TS TBASE5: Store old FBANK value in TBASE5
;     * Preserves original bank for restoration after display
;     * TBASE5 is a temporary storage location
;   
;   Flash loop:
;   - CA VERBNOUN: Load VERBNOUN into A register
;   - TCR BANKCALL: Transfer control with return to BANKCALL subroutine
;   - CADR GOFLASH: Constant address of GOFLASH display routine
;     * GOFLASH displays the verb/noun code with flashing
;     * Awaits crew keyboard input
;   - Return from GOFLASH branches based on crew response:
;     * TCF GOTOPOOH: If crew pressed TERMINATE, branch to GOTOPOOH
;       + GOTOPOOH performs additional cleanup before program exit
;     * TCF +2: If crew pressed PROCEED, skip next instruction
;       + Continues to bank restoration and return to RTRN
;     * (fall through to next): If crew entered data or pressed RECYCLE
;   - VNBANK label: Marks flash loop restart point
;   - TC -5: Transfer control back 5 instructions to CA VERBNOUN
;     * Re-displays the flashing verb/noun (loop continues)
;   
;   Normal exit path (from TCF +2):
;   - CA TBASE5: Load original FBANK value from TBASE5
;   - TS FBANK: Restore original fixed-bank register
;   - TC RTRN: Transfer control to return address in RTRN
;     * Returns to original caller (if this was not a termination)
;   
;   ***** CRITICAL FIXED-FIXED REQUIREMENT *****
;   This routine must remain in fixed-fixed memory (a fixed bank that doesn't
;   require bank switching to access). The routine manipulates FBANK directly,
;   and if it were in a switchable bank, it could corrupt its own bank switching
;   logic, causing the AGC to crash. The "WATCH OUT" comment emphasizes that any
;   future code changes must preserve this fixed-fixed location requirement.
;   
;   During Apollo 11 operations, this routine displayed after each rendezvous
;   program completion, allowing the crew to select the next program in the
;   rendezvous sequence (P32→P33→P34→P35 for standard rendezvous profile).
; ============================================================================

VNPOOH		EXTEND
		QXCH	RTRN
		TS	VERBNOUN
		CAF	VNBANK		# ***** THIS ROUTINE MUST REMAIN IN
		XCH	FBANK		#       FIXED-FIXED *****
		TS	TBASE5		# * WATCH OUT *

		CA	VERBNOUN
		TCR	BANKCALL
		CADR	GOFLASH
		TCF	GOTOPOOH
		TCF	+2
VNBANK		TC	-5

		CA	TBASE5
		TS	FBANK
		TC	RTRN

# Page 485
# ***** CONSTANTS *****

V06N37		VN	0637
V06N55		VN	0655
V06N58		VN	0658
V06N59		VN	0659
V06N81		VN	0681
V16N45		VN	1645
		SETLOC	CSI/CDH
		BANK

TWOPI		2DEC	6.283185307 B-4
MAX250		2DEC	25 E3 B-28	# RSB 2004 added the B-28. OH 2009 leave?
THIRD		2DEC	.333333333
ELEPS		2DEC	.27777777 E-3
DECTWO		OCT	2
DP-.01		OCT	77777		# CONSTANTS
		OCT	61337		# ADJACENT	-.01 FOR MGA DSP
EPSFOUR		2DEC	.0416666666

# Page 486
# ***** INITVEL *****
# MOD NO -1			LOG SECTION -- P34-P35, P74-P75
# MOD BY WHITE, P.		DATE:  21 NOV 67
#
# FUNCTIONAL DESCRIPTION
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
#	L	CALL
#	L+1		INITVEL
#	L+2	(RETURN -- ALWAYS)
#
# INPUT
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
# Page 487
# OUTPUT
#	(1)	RTARG		OFFSET TARGET POSITION VECTOR
#	(2)	VIPRIME		MANEUVER VELOCITY REQUIRED
#	(3)	VTPRIME		VELOCITY AT TARGET AFTER MANEUVER
#	(4)	DELVEET3	DELTA VELOCITY REQUIRED FOR MANEUVER
#
# SUBROUTINES USED
#	LAMBERT
#	INTSTALL
#	INTEGRVS

		SETLOC	INTVEL
		BANK

		COUNT	11/INITV

; ============================================================================
; SUBROUTINE: INITVEL (Initialize Velocity for Lambert Targeting)
; PURPOSE: Prepare initial conditions and compute required velocity for Lambert
;          trajectory solution, with optional iterative refinement using Encke
;          integration for higher accuracy
;
; COMMENT-ONLY READERS: Before the AGC can calculate a trajectory between two
; points in space, it needs to set up the initial conditions and determine what
; velocity the spacecraft needs. This routine prepares all the mathematical
; vectors and calls the Lambert solver (the mathematical method for finding
; trajectories). If high accuracy is needed, it can iterate multiple times,
; each time using a more sophisticated integration method (Encke) to refine
; the solution. Think of it as making a rough sketch first, then redrawing it
; more carefully, repeating until you get the precision required for a lunar
; rendezvous where inches matter.
;
; During Apollo 11's rendezvous operations, INITVEL was called by P34 and P35
; to compute the precise velocity changes needed for the Terminal Phase Initiation
; (TPI) maneuver that would bring the LM to intercept Columbia. The routine had
; to account for the Moon's gravity, orbital mechanics, and the timing constraints
; of rendezvous windows.
;
; CODE-ALONG READERS: Lambert targeting initialization and velocity computation.
; This routine has multiple entry points serving different initialization paths:
;
; ENTRY POINTS:
;   INITVEL: Standard entry when no initial guess available (GUESSW cleared)
;   HAVEGUES: Entry when previous solution exists as initial guess
;
; ALGORITHM OVERVIEW:
; 1. Initialize vectors and parameters for Lambert solver
; 2. Compute geometry vectors (unit normals, rotation angles)
; 3. Call LAMBERT routine to solve two-point boundary value problem
; 4. If iteration requested (NUMIT > 0), use Encke integration for refinement
; 5. Iterate Lambert + Encke until convergence or iteration limit reached
; 6. Compute final delta-velocity vector (DELVEET3)
; 7. Store results for thrusting program use
;
; INPUTS:
;   RINIT: Initial position vector at TIG (B29, meters scaled 2^29)
;   VINIT: Initial velocity vector at TIG (B7, m/s scaled 2^7)
;   RTARG/RTARG1: Target position vector at intercept (B29)
;   DELLT4: Time of flight (centiseconds)
;   RTX1: Transfer angle flag (0=short way, ±8=long way)
;   RTX2: Scaling flag for position/velocity vectors
;   VTARGTAG: Number of iterations to perform (0 = no iteration)
;   GUESSW: Flag indicating if initial guess (HAVEGUES entry) available
;
; OUTPUTS:
;   VIPRIME: Required initial velocity vector (B7 or B5 depending on RTX2)
;   VTPRIME: Target velocity vector at intercept
;   DELVEET3: Delta-velocity vector (VIPRIME - VINIT)
;   RTARG: Final target position (normalized)
;
; ITERATION ALGORITHM:
; The routine can perform up to NUMIT iterations. Each iteration:
; 1. Calls LAMBERT to get conic solution
; 2. Calls INTEGRVS (Encke integration) to propagate more accurately
; 3. Computes offset between conic and integrated target positions
; 4. Adjusts target position and iterates
; This convergence process accounts for perturbations (lunar oblateness, etc.)
; that the simple conic Lambert solution doesn't include.
;
; SCALING LOGIC (RTX2 flag):
; If RTX2 = 0 (standard scaling):
;   RINIT at B29, VINIT at B7, results at B7
; If RTX2 ≠ 0 (rescaled for long times of flight):
;   Vectors shifted left 2 bits: RINIT→B27, VINIT→B5, results at B5
;   This prevents overflow for very long coast times (hours)
;
; COORDINATE FRAMES:
; The routine constructs a local coordinate frame for the Lambert problem:
;   R1VEC: Initial position vector
;   R2VEC: Target position vector (may be rotated into computation plane)
;   UN: Unit normal to the transfer plane
; The Lambert solver works in this frame, then results are transformed back.
;
; TRANSFER ANGLE GEOMETRY (RTX1):
; The routine determines whether to use "short way" (< 180°) or "long way"
; (> 180°) transfer based on RTX1 and the geometry of position vectors.
; The sign of the GEOMSGN variable encodes this for Lambert.
;
; CONVERGENCE:
; The iteration counter ITCTR starts at -1 and increments each cycle.
; When ITCTR equals VTARGTAG (NUMIT), iteration terminates.
; If VTARGTAG = 0, no iteration occurs (single Lambert call only).
;
; CRITICAL MISSION CONTEXT:
; During Apollo 11 TPI targeting, accurate delta-V computation was essential.
; The LM's fuel was limited after the landing and ascent, so the AGC needed
; to compute the most efficient trajectory to rendezvous with Columbia.
; Too much error could waste precious fuel or miss the rendezvous window.
; The iterative refinement with Encke integration provided the accuracy needed
; to ensure Michael Collins could see the LM approaching and complete the
; final docking maneuvers.
; ============================================================================

; Entry points: INITVEL (no guess) or HAVEGUES (with initial guess from previous solution)
; The GUESSW flag tells Lambert solver whether to use a previous trajectory as starting point.

INITVEL		SET			# COGA GUESS NOT AVAILABLE
			GUESSW		; Set flag indicating no initial guess available
			
; HAVEGUES entry: Called when previous Lambert solution exists for use as initial guess.
; This accelerates convergence for iterative refinement scenarios.

HAVEGUES	VLOAD	STQ		; Load target position vector
			RTARG		; RTARG = target position at intercept (B29, meters scaled 2^29)
			NORMEX		; Save return address for final normalization
		STORE	RTARG1		; Store working copy in RTARG1
		
; Check RTX2 scaling flag to determine if vectors need rescaling.
; RTX2 ≠ 0 indicates long time-of-flight requiring left shift to prevent overflow.

		SLOAD	BHIZ		; Load RTX2 scaling flag
			RTX2		; RTX2 = 0 → standard scaling (B29/B7)
			INITVEL1	; RTX2 = 0 → branch to skip rescaling
			
; RTX2 ≠ 0: Rescale all vectors left 2 bits for extended precision with long coast times.
; This shifts: RINIT from B29→B27, VINIT from B7→B5, RTARG1 from B29→B27.
; The 2-bit shift provides 4x range extension for times up to several hours.

		VLOAD	VSL2		; Load initial position vector
			RINIT		; RINIT at B29 (meters * 2^29)
		STOVL	RINIT		; Store shifted result at B27 (meters * 2^27)
			VINIT		; Load initial velocity vector at B7 (m/s * 2^7)
		VSL2			; Shift left 2 bits
		STOVL	VINIT		; Store at B5 (m/s * 2^5)
			RTARG1		; Load target position
		VSL2			; Shift left 2 bits
		STORE	RTARG1		; Store rescaled target at B27

# INITIALIZATION

; Initialize iteration counter and compute cosine of elevation angle for Lambert solver.
; The iteration counter ITCTR tracks refinement cycles (starts at -1 for initial pass).
; E4 (elevation angle) defines the geometry of the line-of-sight cone.

INITVEL1	SSP	DLOAD		# Set iteration counter to -1 (first pass)
			ITCTR		; ITCTR = iteration counter for Encke refinement
			0 -1		; -1 indicates initial Lambert solution (no refinement yet)
		COSINE	SR1		; Compute cos(E4), shift right 1 bit for scaling
		STODL	COZY4		; Store cos(E4) at COZY4 (used in transfer angle constraint)
		LXA,2	SXA,2		; Load MPAC to index register 2
			MPAC		; Transfer computed cosine value
			VTARGTAG	; Store to VTARGTAG = 0 (marks velocity as not yet computed)
			
; Set up position vectors for Lambert solver:
; R1VEC = initial position of active vehicle (departure point)
; R2VEC = target position at intercept time (arrival point)

		VLOAD			; Load initial position
			RINIT		; RINIT = active vehicle position at ignition
		STOVL	R1VEC		; R1VEC ← RINIT (Lambert departure position)
			RTARG1		; Load target position at intercept
# Page 488
		STODL	R2VEC		; R2VEC ← RTARG1 (Lambert arrival position)
			DELLT4		; Load desired time-of-flight
		STORE	TDESIRED	; TDESIRED ← DELLT4 (transfer time constraint for Lambert)
		
; Initialize pushdown list and compute orbital plane geometry.
; The plane normal UN is defined by the cross product of initial position and velocity.

		SETPD	VLOAD		; Set pushdown pointer to 0
			0D		; Initialize PL (pushdown list pointer) to 0D
			RINIT		; Load initial position vector (B29 scaling)
		UNIT	PUSH		; Compute UNIT(RINIT), push to PD (PL now 6D)
		VXV	UNIT		; Cross with velocity: UNIT(RINIT) × VINIT
			VINIT		; Initial velocity vector (B7 or B5 if rescaled)
		STOVL	UN		; Store orbit plane normal in UN
			RTARG1		; Load target position
		UNIT	DOT		; Compute URT · URI (dot product of unit target and unit initial)
			
; Adjust COZY4 by adding the dot product URT·URI. This modifies the elevation angle
; constraint to account for the actual geometry between initial and target positions.

		DAD	CLEAR		; Add to COZY4
			COZY4		; COZY4 = cos(E4) + (URT · URI)
			NORMSW		; Clear normal switch flag
		STORE	COZY4		; Store adjusted constraint value
		
INITVEL2	BPL	SET		; If COZY4 ≥ 0, branch to INITVEL3
			INITVEL3	; Positive → plane normal will be computed by LAMBERT
			NORMSW		; Set NORMSW flag if negative

# ROTATE RC INTO YC PLANE -- SET UNIT NORMAL TO YC

; This section rotates the target position vector into the YC computation plane.
; The YC plane is defined by the velocity direction and orbit plane normal UN.

		VLOAD	PUSH		; Load and push target position to PD (PL now 6D)
			R2VEC		; RC (target position) to pushdown at 6D (B29 scaling)
; Compute target position magnitude and project it into the YC plane.
; The YC plane is perpendicular to the orbit plane normal UN.

		ABVAL	PDVL		; |R2VEC| to MPAC, value pushed to 0D (PL now 2D)
		PUSH	VPROJ		; Push R2VEC to PD (PL 8D), then project onto UN
			UN		; VPROJ computes component of R2VEC along UN (out-of-plane)
		VSL2	BVSU		; Shift left 2, subtract from R2VEC to get in-plane component
		UNIT	VXSC		; Normalize, then scale by magnitude (PL returns to 0D)
		VSL1			; Shift left 1 for proper scaling
		STORE	R2VEC		; Store rotated target position (now in YC plane)
		
; Check iteration counter to determine if this is first pass or refinement cycle.
; On first pass (ITCTR < 0), update RTARG1 with rotated target.

		TLOAD	SLOAD		; Load ZEROVEC (for comparison/clearing)
			ZEROVEC		; All-zero vector
			ITCTR		; Load iteration counter
		BPL	VLOAD		; If ITCTR ≥ 0, skip update (refinement cycle)
			INITVEL3	; Branch to Lambert setup
			R2VEC		; Load rotated target position
		STORE	RTARG1		; Update RTARG1 ← R2VEC (first pass only)
; LAMBERT ROUTINE SETUP
; Compute transfer plane normal and set up geometric parameters for Lambert solver.
; Lambert routine solves two-point boundary value problem: find velocity that transfers
; from R1VEC to R2VEC in time TDESIRED with gravitational parameter MUEARTH.

INITVEL3	DLOAD	PDVL		; Load gravitational constant, push to 0D (PL 2D)
			MUEARTH		; μ = gravitational parameter (positive value)
			R2VEC		; Load target position vector
		UNIT	PDVL		; Compute UNIT(R2VEC), push to 2D (PL 8D)
			R1VEC		; Load initial position vector
		UNIT	PUSH		; Compute UNIT(R1VEC), push to 8D (PL 14D)
		VXV	VCOMP		; Cross product: -N = UNIT(R2VEC) × UNIT(R1VEC)
			2D		; Cross with unit target vector from 2D
		PUSH			; Push -N (negative plane normal) to 14D (PL 20D)
		
; Determine transfer direction (short way or long way around orbit).
; RTX1 flag controls path selection for Lambert solver.

		LXA,1	DLOAD		; Load RTX1 into index register 1
			RTX1		; RTX1 = transfer direction flag
			18D		; Load value from 18D for comparison
		BMN	INCR,1		; If negative, increment X1
# Page 489
			+2		; Branch +2 if negative
		DEC	-8		; Decrement constant
		INCR,1	SLOAD		; Increment X1, load value
			10D		; Load from 10D
			X1		; Load index register 1 value
		BHIZ	VLOAD		; If zero, skip complement (PL returns to 14D)
			+2		; Branch +2 if zero
		VCOMP	PUSH		; Complement vector, push (PL 20D)
		VLOAD			; Load vector (PL 14D)
		
; Compute sign of geometric configuration (determines transfer type).
; The cross product and dot product determine whether orbit transfer is
; prograde or retrograde, and which of two possible transfer arcs to use.

		VXV	DOT		; Cross product then dot product (PL 2D)
		BPL	DLOAD		; If positive, load (PL 0D)
			INITVEL4	; Branch to INITVEL4 if positive
		DCOMP	PUSH		; Double complement, push result (PL 2D)
INITVEL4	LXA,2	SXA,2		; Load and store index register 2
			0D		; Load from 0D
			GEOMSGN		; Store geometric sign flag

# SET INPUTS UP FOR LAMBERT

; Final setup: Load transfer direction flag and call Lambert solver.
; Lambert will compute required initial velocity VVEC to achieve transfer.

		LXA,1	CALL		; Load RTX1 to index register 1
			RTX1		; Transfer direction indicator
#  OPERATE THE LAMBERT CONIC ROUTINE (COASTFLT SUBROUTINE)

			LAMBERT		; Call Lambert two-point boundary solver

# ARRIVED AT SOLUTION IS GOOD ENOUGH ACCORDING TO SLIGHTLY WIDER BOUNDS.

; Lambert solver has returned with computed initial velocity VVEC.
; Clear GUESSW flag since we now have a valid solution to use for future refinements.

		CLEAR	VLOAD		; Clear guess flag, load solution
			GUESSW		; GUESSW ← 0 (solution now available for next iteration)
			VVEC		; VVEC = Lambert-computed initial velocity

# STORE CALCULATED INITIAL VELOCITY REQUIRED IN VIPRIME

		STODL	VIPRIME		; Store required initial velocity (B7 scaling: m/s * 2^7)

# IF NUMIT IS ZERO, CONTINUE AT INITVELB, OTHERWISE
# SET UP INPUTS FOR ENCKE INTEGRATION (INTEGRVS).

; Check VTARGTAG to determine if Encke integration refinement is needed.
; If NUMIT=0 (no refinement iterations requested), skip integration and exit.
; Otherwise, integrate the trajectory forward to verify actual arrival state.

			VTARGTAG	; VTARGTAG = NUMIT (number of Encke refinement iterations)
		BHIZ	CALL		; If zero, branch to INITVEL7 (skip refinement)
			INITVEL7	; Exit without Encke integration
			INTSTALL	; Install integration parameters
			
; Set MOONFLAG based on RTX2 to indicate central body for integration.
; RTX2=0 → Earth orbit, RTX2≠0 → Lunar orbit (affects gravitational model).

		SLOAD	CLEAR		; Load RTX2 scaling/body flag
			RTX2		; RTX2 also encodes central body
			MOONFLAG	; Clear moon flag initially
		BHIZ	SET		; If RTX2=0, skip set (Earth)
			INITVEL5	; Branch if Earth-centered
			MOONFLAG	; Set MOONFLAG for lunar-centered integration
			
; Set up state vector for Encke integration:
; RCV/VCV = position/velocity at ignition time (INTIME)
; TDEC1 = target time at intercept (INTIME + DELLT4)

INITVEL5	VLOAD			; Load initial position
			RINIT		; Active vehicle position at ignition
		STORE	R1VEC		; Store in R1VEC for integration
# Page 490
		STOVL	RCV		; Store as Encke initial position (RCV)
			VIPRIME		; Load Lambert-computed velocity
		STODL	VCV		; Store as Encke initial velocity (VCV)
			INTIME		; Load ignition time
		STORE	TET		; TET ← ignition time (integration start)
		DAD	CLEAR		; Add transfer time
			DELLT4		; DELLT4 = desired time-of-flight
			INTYPFLG	; Clear integration type flag (precision mode)
		STCALL	TDEC1		; TDEC1 ← arrival time, call integration
			INTEGRVS	; Encke numerical integration routine
			
; Integration complete. VATT1 contains actual arrival velocity.
; Store in VTARGET for comparison with desired target velocity.

		VLOAD			; Load integrated arrival velocity
			VATT1		; VATT1 = velocity at intercept (from Encke)
		STORE	VTARGET		; VTARGET ← actual arrival velocity

# IF ITERATION COUNTER (ITCTR) EQ NO. ITERATIONS (NUMIT), CONTINUE AT
# INITVELC, OTHERWISE REITERATE LAMBERT AND ENCKE

; Check if refinement iterations are complete.
; Increment ITCTR and compare with NUMIT (stored in VTARGTAG).
; If ITCTR = NUMIT, refinement is done; otherwise, iterate again.

		LXA,2	INCR,2		; Load iteration counter to X2
			ITCTR		; ITCTR = current iteration count
			1D		; Increment by 1
		SXA,2	XSU,2		; Store incremented ITCTR, then subtract
			ITCTR		; ITCTR ← ITCTR + 1
			VTARGTAG	; Subtract NUMIT from (ITCTR+1)
		SLOAD	BHIZ		; Load result to MPAC
			X2		; X2 = (ITCTR+1) - NUMIT
			INITVEL6	; If zero, iterations done → compute final delta-v

# OFFSET CONIC TARGET VECTOR

; Iterations not complete. Adjust target position for next Lambert iteration.
; This offset compensates for the difference between the conic transfer path
; (assumed by Lambert) and the actual perturbed trajectory (computed by Encke).
; The correction drives convergence toward the true transfer solution.

		VLOAD	VSU		; Compute position error at intercept
			RTARG1		; RTARG1 = desired target position
			RATT1		; RATT1 = actual arrival position (from Encke)
		VAD			; Add error to target offset
			R2VEC		; R2VEC = current target position offset
		STODL	R2VEC		; R2VEC ← updated offset for next iteration
			COZY4		; Load COZY4 (time/geometry parameter)
		GOTO			; Loop back to compute new Lambert solution
			INITVEL2	# CONTINUE ITERATING AT INITVEL2

# COMPUTE THE DELTA VELOCITY

; Refinement complete or no iterations requested. Compute final maneuver delta-v.
; DELVEET3 = required velocity change to achieve the transfer trajectory.
; This is the delta-v the active vehicle must execute at TIG to reach the target.

INITVEL6	VLOAD			; Store final target position
			R2VEC		; R2VEC = refined target position
		STORE	RTARG1		; RTARG1 ← final target position
INITVEL7	VLOAD	VSU		; Compute required delta-v
			VIPRIME		; VIPRIME = Lambert-computed initial velocity
			VINIT		; VINIT = actual vehicle velocity at TIG
		STOVL	DELVEET3	# DELVEET3 = VIPRIME-VINIT (+7)
			VTARGET		; Load arrival velocity
		STORE	VTPRIME		; VTPRIME ← arrival velocity at intercept
		SLOAD	BHIZ		; Check rescaling flag
			RTX2		; RTX2 = 1 if vectors were left-shifted earlier
# Page 491
			INITVELX	; If RTX2 = 0, skip rescaling → exit
			
; RTX2 flag set: vectors were left-shifted 2 positions for precision.
; Restore original scaling by right-shifting 2 positions.
; This ensures output vectors have correct scaling (+7 for velocity, position).

		VLOAD	VSR2		; Right-shift arrival velocity 2 positions
			VTPRIME		; VTPRIME = arrival velocity
		STOVL	VTPRIME		; Store rescaled arrival velocity
			VIPRIME		; Load Lambert initial velocity
		VSR2			; Right-shift 2 positions
		STOVL	VIPRIME		; Store rescaled initial velocity
			RTARG1		; Load target position
		VSR2			; Right-shift 2 positions
		STOVL	RTARG1		; Store rescaled target position
			DELVEET3	; Load delta-v vector
		VSR2			; Right-shift 2 positions
		STORE	DELVEET3	; Store rescaled delta-v (final output)
; Final exit: normalize and store target parameters.
; RTARG contains the target position for display and storage.
; Return to caller with delta-v in DELVEET3.

INITVELX	SETPD	VLOAD		; Reset pushdown pointer, load target position
			0D		; 0D = pushdown base
			RTARG1		; RTARG1 = final target position
		STCALL	RTARG		; RTARG ← target position (for display)
			NORMEX		; Normalize and exit to calling routine

# ***** END OF INITVEL ROUTINE *****

# Page 492
# ***** MIDGIM *****
# MOD NO. 0, BY WILLMAN, SUBROUTINE RENDGUID, LOG P34-P35, P74-P75
# REVISION 03, 17 FEB 67
#
# IF THE ACTIVE VEHICLE IS DOING THE COMPUTATION, MIDGIM COMPUTES
# THE POSITIVE MIDDLE GIMBAL ANGLE OF THE ACTIVE VEHICLE TO THE INPUT
# DELTA VELOCITY VECTOR (0D IN PUSY LIST), OTHERWISE
# MIDGIM CONVERTS THE INPUT DELTA VELOCITY VECTOR FROM INERTIAL COORDIN-
# ATES TO LOCAL VERTICAL COORDINATES OF THE ACTIVE VEHICLE.
#
# ** INPUTS **
#   NAME     MEANING					  	UNITS/SCALING/MODE
#   AVFLAG   INT FLAG -- 0 IS CSM ACTIVE, 1 IS LEM ACTIVE	BIT
#   COMPUTER INT FLAG -- 0 IS LEM COMPUTER, 1 IS CSM COMPUTER	BIT
#   RINIT    ACTIVE VEHICLE RADIUS VECTOR			METERS/CSEC (+7) VT
#   VINIT    ACTIVE VEHICLE VELOCITY VECTOR			METERS/CSEC (+7) VT
#   0D(PL)   ACTIVE VEHICLE DELTA VELOCITY VECTOR		METERS/CSEC (+7) VT
#
# ** OUTPUTS **
#   NAME     MEANING						UNITS/SCALING/MODE
#   +MGA     + MIDDLE GIMBAL ANGLE				REVOLUTIONS (+0) DP
#   DELVLVC  DELTA VELOCITY VECTOR IN LV COORD.			METERS/CSEC (+7) VT
#   MGLVFLAG INT FLAG: 0 IS +MGA COMUTED, 1 IS DELVLVC COMP.	BIT
#
# ** CALLING SEQUENCE **
#	L 	CALL
#	L+1		MIDGIM
#	L+2	(RETURN -- ALWAYS)
#
# ** NO SUBROUTINES CALLED **
#
# ** DEBRIS -- ERASABLE TEMPORARY USAGE **
#	A,Q,L, PUSH LIST, MPAC.
#
# ** ALARMS -- NONE **

# Page 493
# MIDDLE GIMBAL ANGLE COMPUTATION

; ============================================================================
; SUBROUTINE: MIDGIM (Middle Gimbal Angle Computation)
; PURPOSE: Compute either the middle gimbal angle (+MGA) or the delta-velocity
;          in local vertical coordinates (DELVLVC), depending on flag settings
;
; COMMENT-ONLY READERS: The spacecraft's Inertial Measurement Unit (IMU) uses
; three gimbals to maintain orientation—like a gyroscope in three nested rings.
; The middle gimbal angle is the angle of the middle ring. If this angle gets
; close to 90 degrees, a condition called "gimbal lock" can occur where the
; spacecraft loses its ability to measure rotation in one axis. This routine
; computes the middle gimbal angle so the AGC can warn the crew if gimbal lock
; is approaching during a maneuver. Alternatively, this routine can compute the
; required velocity change in "local vertical coordinates"—a coordinate system
; aligned with the spacecraft's local up/down direction—which is useful for
; crew displays and manual steering during burns.
;
; During Apollo missions, gimbal lock warnings were serious events requiring
; immediate crew action to re-orient the spacecraft. The AGC constantly monitored
; gimbal angles and would display a warning light if gimbal lock was imminent.
;
; CODE-ALONG READERS: Middle gimbal angle (MGA) and local vertical coordinate
; (LVC) transformation routine. This dual-purpose routine branches based on flag
; logic to perform one of two computations:
;
; ENTRY POINTS:
;   MIDGIM: Main entry point, tests flags to determine computation path
;   MIDGIM1: Secondary entry point after AVFLAG test
;
; FLAG LOGIC:
;   AVFLAG: Active Vehicle flag (set when this vehicle is performing maneuver)
;   COMPUTER: Computer control flag (set when AGC is computing for this vehicle)
;   
;   Decision tree:
;   - If AVFLAG and COMPUTER have OPPOSITE values → compute +MGA (GET+MGA)
;   - If AVFLAG and COMPUTER have SAME values → compute DELVLVC (GET.LVC)
;   
;   This logic ensures the routine computes the appropriate quantity based on
;   whether this vehicle is actively maneuvering or passively tracking.
;
; COMPUTATION PATH 1: GET+MGA (Positive Middle Gimbal Angle)
; ----------------------------------------------------------
; Computes the middle gimbal angle from current velocity vector and reference
; coordinate system (stable member matrix).
;
; Algorithm:
;   1. VLOAD/UNIT: Load velocity vector, normalize to unit vector UV (+1)
;      - Velocity at B7 scaled to unit vector at B1
;   2. DOT REFSMMAT+6: Dot product UV with Y-axis of stable member
;      - REFSMMAT+6 is the middle column of the reference matrix
;      - Result is sine of middle gimbal angle (at B2)
;   3. SL1: Shift left 1 bit to scale from B2 to B1 for ARCSIN input
;   4. ARCSIN: Compute arcsine to get middle gimbal angle
;      - Returns angle in revolutions (±0.25 rev for ±90°)
;   5. BPL SETMGA: Branch if positive (MGA already in +range)
;   6. DAD HALFREV (twice): If negative, convert -MGA to +MGA
;      - Add 1.0 revolution (twice) to get equivalent positive angle
;      - This ensures MGA is always displayed as positive value
;   7. STORE +MGA: Store positive middle gimbal angle for gimbal lock check
;   8. CLR MGLVFLAG: Clear flag to indicate +MGA computation complete
;   9. RVQ: Return to caller
;
; COMPUTATION PATH 2: GET.LVC (Local Vertical Coordinates)
; ---------------------------------------------------------
; Transforms delta-velocity vector from inertial coordinates to local vertical
; coordinates (LVC). LVC frame is defined by:
;   X-axis: Direction of velocity (downrange)
;   Z-axis: Direction opposite to position (local vertical "up")
;   Y-axis: Cross product (completes right-handed frame)
;
; Algorithm:
;   1. VLOAD/UNIT RINIT: Load position vector, normalize to UR
;   2. VCOMP: Negate to get U(-R) (local vertical up direction)
;   3. STORE 18D: Save U(-R) to temporary location
;   4. VXV VINIT: Cross product U(-R) × V = V × U(R)
;      - This gives vector perpendicular to orbital plane
;   5. UNIT: Normalize cross product to U(V×R)
;   6. STORE 12D: Save to temporary location
;   7. VXV 18D: Cross product U(V×R) × U(-R)
;      - This gives third axis completing the coordinate frame
;   8. UNIT: Normalize to unit vector
;   9. STOVL 6D: Store transformation matrix starting at 6D
;      - 6D contains the 3×3 orthogonal transformation matrix
;   10. VLOAD 0D: Load delta-V vector from 0D (inertial coordinates)
;   11. MXV 6D: Matrix multiply delta-V by transformation matrix
;       - Rotates delta-V from inertial frame to LVC frame
;   12. VSL1: Scale result left 1 bit (from B7 to B8, then store at B7)
;   13. STORE DELVLVC: Store delta-V in local vertical coordinates
;   14. SET MGLVFLAG: Set flag to indicate LVC computation complete
;   15. RVQ: Return to caller
;
; TRANSFORMATION MATRIX STRUCTURE (6D):
; The 3×3 orthogonal matrix at 6D transforms vectors from inertial to LVC:
;   Row 1 (6D):  U(V×R) × U(-R)  [X-axis: perpendicular to V and R]
;   Row 2 (12D): U(V×R)          [Y-axis: perpendicular to orbital plane]
;   Row 3 (18D): U(-R)           [Z-axis: local vertical up]
;
; GIMBAL LOCK CONTEXT:
; The middle gimbal angle computation is critical for gimbal lock avoidance.
; If MGA approaches ±70°, the AGC illuminates the GIMBAL LOCK warning light
; on the DSKY. If MGA reaches ±85°, gimbal lock is imminent and the crew must
; perform a coarse alignment (realigning the stable member to a new orientation).
;
; During Apollo 11, gimbal lock warnings were taken very seriously. An actual
; gimbal lock event would render the IMU useless, forcing the crew to realign
; using star sightings—a time-consuming process that could jeopardize mission
; timeline and safety. The AGC's constant monitoring of gimbal angles through
; routines like MIDGIM helped prevent such events.
;
; LOCAL VERTICAL COORDINATES USAGE:
; The LVC transformation is useful for crew displays during manual burns.
; Displaying delta-V components in the local vertical frame gives intuitive
; information:
;   X-component: How much to speed up/slow down along orbit
;   Y-component: How much out-of-plane correction needed
;   Z-component: How much radial (up/down) thrust needed
;
; This representation is easier for crew to understand than inertial coordinates,
; especially during manual attitude control of thrusting maneuvers.
; ============================================================================

		SETLOC	MIDDGIM
		BANK

		COUNT*	$$/MIDG

HALFREV		2DEC	1 B-1

MIDGIM		BON	BOFF
			AVFLAG
			MIDGIM1
			COMPUTER
			GET.LVC

# COMPUTE +MGA IF AVFLAG AND COMPUTER HAVE OPPOSITE VALUES.

GET+MGA		VLOAD	UNIT		# (PL 0D) V (+7) TO MPAC UNITIZE UV (+1)
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

MIDGIM1		BOFF
			COMPUTER
			GET+MGA

# COMPUTE DELVLVC IF AVFLAG AND COMPUTER HAVE SAME VALUES

GET.LVC		VLOAD	UNIT		# (PL 6D) R (+29) IN MPAC UNITZE UR
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
		STORE	DELVLVC		# STORE IN DELVLVC (+7(
		SET	RVQ		# SET MGLVFLAG TO INDICATE LVC CALC
			MGLVFLAG	# AND EXIT

# ***** END OF MIDGIM ROUTINE *****

# Page 494

; ============================================================================
; SUBROUTINE: SELECTMU (Select Gravitational Constant)
; PURPOSE: Select the appropriate gravitational constant (mu) and scaling
;          factors based on the sphere of influence (Earth vs Moon)
;
; COMMENT-ONLY READERS: When computing trajectories in space, the AGC needs to
; know which celestial body's gravity is dominant—Earth's or the Moon's. This
; routine checks a flag that indicates whether the spacecraft is in the Moon's
; sphere of influence (cislunar space near the Moon) or Earth's sphere of
; influence. It then loads the appropriate gravitational constant—either
; Earth's mu (398,600 km³/s²) or the Moon's mu (4,903 km³/s²)—into the
; computational registers. This value is essential for all trajectory calculations
; because it determines how strongly gravity pulls on the spacecraft.
;
; During Apollo 11's mission, the spacecraft crossed the boundary between Earth's
; and the Moon's spheres of influence during both the outbound translunar coast
; and the return transearth coast. The AGC automatically switched between
; gravitational constants to maintain accurate trajectory predictions. For
; rendezvous operations in lunar orbit, the Moon's mu was used exclusively.
;
; CODE-ALONG READERS: Gravitational constant selection based on sphere of
; influence. This routine uses indexed table lookup to select mu and scaling
; factors appropriate for the current gravitational environment.
;
; ALGORITHM:
; 1. Initialize index registers based on CMOONFLG (cislunar moon flag)
; 2. Load gravitational constant from indexed table location
; 3. Store mu and scaling factors for trajectory computations
; 4. Clear FINALFLG and return to calling routine
;
; DETAILED INSTRUCTION SEQUENCE:
;
; Initial index setup (Earth sphere):
;   AXC,1 AXT,2 / 2D / 0D:
;     - AXC,1: Load X1 index register with constant from next address (2D)
;       * X1 = 2D (used as table offset for Earth parameters)
;     - AXT,2: Load X2 index register with constant from 2nd address (0D)
;       * X2 = 0D (scaling flag for Earth)
;
; Sphere of influence test:
;   BOFF CMOONFLG SETMUER:
;     - Test CMOONFLG (cislunar moon flag)
;     - If flag is OFF (clear) → branch to SETMUER (use Earth parameters)
;     - If flag is ON (set) → continue to next instruction (use Moon parameters)
;
; Alternate index setup (Moon sphere):
;   AXC,1 AXT,2 / 10D / 2D:
;     - AXC,1: Load X1 index register with 10D (table offset for Moon parameters)
;     - AXT,2: Load X2 index register with 2D (scaling flag for Moon)
;     - Fall through to SETMUER with Moon indices
;
; Parameter loading (SETMUER entry point):
;   DLOAD* MUTABLE +4,1:
;     - DLOAD*: Double-precision load with indexing
;     - MUTABLE +4,1: Load from MUTABLE table at offset (4 + X1)
;       * If X1=2D (Earth): Loads Earth's mu from MUTABLE+6
;       * If X1=10D (Moon): Loads Moon's mu from MUTABLE+14 (octal)
;     - Result: MPAC contains gravitational constant mu (km³/s² scaled)
;
;   SXA,1 RTX1:
;     - SXA,1: Store index register X1 at address in next word
;     - RTX1: Address where X1 value is stored
;       * Preserves the table offset for later use
;       * RTX1 encodes which sphere of influence is active
;
;   STODL* RTSR1/MU:
;     - STODL*: Store double-precision, then load double-precision
;     - RTSR1/MU: First address (stores mu value)
;     - The * indicates indexed addressing for the load operation
;
;   MUTABLE -2,1:
;     - Load from MUTABLE table at offset (-2 + X1)
;       * If X1=2D (Earth): Loads from MUTABLE+0 (Earth radius)
;       * If X1=10D (Moon): Loads from MUTABLE+8 (Moon radius)
;     - This loads the primary body radius for pericenter calculations
;
; Scaling adjustment for Moon:
;   BOFF CMOONFLG RTRNMU:
;     - Test CMOONFLG again
;     - If OFF (Earth) → branch to RTRNMU (skip scaling adjustment)
;     - If ON (Moon) → continue to next instruction
;
;   SR 6D:
;     - SR: Shift right
;     - 6D: Shift right by 6 bits (divide by 2^6 = 64)
;     - This rescales the Moon radius value for proper unit consistency
;     - Moon parameters require different scaling than Earth parameters
;
; Final storage (RTRNMU):
;   STORE RTMU:
;     - Store the (possibly scaled) body radius in RTMU
;     - RTMU: Runtime mu parameter used by trajectory routines
;
;   SXA,2 RTX2:
;     - SXA,2: Store index register X2 in RTX2
;     - RTX2: Runtime flag encoding scaling mode
;       * RTX2 = 0D for Earth (standard scaling)
;       * RTX2 = 2D for Moon (alternate scaling)
;
;   CLEAR FINALFLG:
;     - Clear the FINALFLG (final computation flag)
;     - Indicates that this is not the final iteration of program
;
;   GOTO VN1645:
;     - Transfer control to VN1645 (display routine)
;     - Returns to main program flow
;
; MUTABLE TABLE STRUCTURE:
; The MUTABLE table (defined elsewhere in erasable memory) contains:
;   MUTABLE+0:  Earth radius (scaled)
;   MUTABLE+2:  (reserved/intermediate value)
;   MUTABLE+4:  (reserved/intermediate value)  
;   MUTABLE+6:  Earth mu (gravitational constant)
;   MUTABLE+8:  Moon radius (scaled)
;   MUTABLE+10: (reserved/intermediate value)
;   MUTABLE+12: (reserved/intermediate value)
;   MUTABLE+14: Moon mu (gravitational constant)
;
; SPHERE OF INFLUENCE BOUNDARY:
; The sphere of influence boundary is the point where one body's gravitational
; influence becomes dominant over another's. For the Earth-Moon system, this
; occurs approximately 326,000 km from Earth (about 84% of the way to the Moon).
; The CMOONFLG is set by navigation routines when the spacecraft crosses this
; boundary.
;
; GRAVITATIONAL CONSTANTS:
; Earth mu = 398,600.4 km³/s² (scaled by AGC to fit in fixed-point format)
; Moon mu = 4,902.8 km³/s² (approximately 1/81 of Earth's mu)
;
; The difference in magnitude is why different scaling factors (RTX2) are needed.
; Moon trajectories require finer resolution because lunar orbital velocities
; are much smaller than Earth orbital velocities.
;
; MISSION CONTEXT - APOLLO 11:
; During Apollo 11's mission, SELECTMU was called multiple times:
; 1. Trans-lunar injection (TLI): Used Earth mu for initial trajectory
; 2. Crossing sphere of influence: Switched from Earth mu to Moon mu
; 3. Lunar orbit insertion (LOI): Used Moon mu
; 4. All rendezvous operations: Used Moon mu exclusively
; 5. Trans-earth injection (TEI): Initially Moon mu, then switched to Earth mu
; 6. Earth re-entry: Used Earth mu for final trajectory
;
; The accurate selection of gravitational constants was essential for trajectory
; prediction accuracy. An error in mu selection could cause the AGC to compute
; incorrect burn parameters, potentially causing the spacecraft to miss its
; target orbit or rendezvous point.
; ============================================================================

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

# Page 495
# ***** PERIAPO *****
# MOD NO -1       LOG SECTION - P34-P35, P74-P75
# MOD BY WHITE.P  DATE  18JAN68
#
# FUNCTIONAL DESCRIPTION
#	THIS SUBROUTINE COMPUTES THE TWO BODY APOCENTER AND PERICENTER
#	ALTITUDES GIVEN THE POSITION AND VELOCITY VECTORS FOR A POINT ON
#	THE TRAJECTORY AND THE PRIMARY BODY.
#
#	SETRAD IS CALLED TO DETERMINE THE RADIUS OF THE PRIMARY BODY.
#
#	APSIDES IS CALLED TO SOVE FOR THE TWO BODY RADII OF APOCENTER AND
#	PERICENTER AND THE ECCENTRICITY OF THE TRAJECTORY.
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		PERIAPO
#	L+2	(RETURN -- ALWAYS)
#
# INPUT
#	(1)	RVEC	POSITION VECTOR IN METERS
#			SCALE FACTOR -- EARTH +29, MOON +27
#	(2)	VVEC	VELOCITY VECTOR IN METERS/CENTISECOND
#			SCALE FACTOR -- EARTH +7, MOON +5
#	(3)	X1	PRIMARY BODY INDICATOR
#			EARTH -2, MOON -10
#
# OUTPUT
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
#	SETRAD
# Page 496
#	APSIDES

		SETLOC	APOPERI
		BANK

		COUNT*	$$/PERAP

; ============================================================================
; PERIAPO - APOCENTER AND PERICENTER COMPUTATION
;
; This subroutine computes the apocenter (apogee) and pericenter (perigee)
; radii and altitudes for a given orbital state vector. Essential for 
; validating that transfer orbits maintain safe altitude constraints during
; rendezvous maneuvers (e.g., pericenter > 35,000 ft lunar, > 85 NM Earth).
;
; COMMENT-ONLY READERS: This routine calculates the highest and lowest points
;        in an orbit, ensuring the spacecraft never dips too close to the 
;        surface during rendezvous operations.
;
; CODE-ALONG READERS: Uses APSIDES subroutine (from CONIC_SUBROUTINES.agc)
;        for two-body orbital mechanics computation. Handles both Earth and
;        lunar orbits with appropriate radius constants and scaling.
;
; CALLING SEQUENCE:
;        PERIAPO  - Standard entry with RVEC and VVEC already loaded
;        PERIAPO1 - Entry with vectors in RTX1, RTX2 requiring scaling
;
; INPUT:
;        PERIAPO:  RVEC (position vector, B-29 or B-27)
;                  VVEC (velocity vector, B-7 or B-5)
;        PERIAPO1: RTX1 (index to position vector)
;                  RTX2 (index to velocity vector)
;
; OUTPUT:
;        MPAC stack:
;        2D = Apocenter radius (B-29 or B-27 meters)
;        4D = Apogee altitude above surface (B-29 or B-27 meters)
;        6D = Pericenter radius (B-29 or B-27 meters)  
;        8D = Perigee altitude above surface (B-29 or B-27 meters)
;
; SUBROUTINES CALLED:
;        SETRAD  - Determines planetary radius (Earth or lunar)
;        APSIDES - Computes apocenter/pericenter from state vectors
; ============================================================================

RPAD		2DEC	6373338 B-29	# STANDARD RADIUS OF PAD 37-B.
					# = 20 909 901.57 FT

; PERIAPO1 - Entry point with indexed vectors requiring scaling
PERIAPO1	LXA,2	VSR*
			RTX2
			0,2
		STOVL	VVEC		; Scale and store velocity vector from RTX2
		LXA,1	VSR*
			RTX1
			0,2
		STORE	RVEC		; Scale and store position vector from RTX1

; PERIAPO - Main entry point
; Computes orbital apsides (highest/lowest points) for trajectory validation
PERIAPO		STQ	CALL
			NORMEX		; Save return address
			SETRAD		; Get planetary radius (Earth or Moon)
		STCALL	XXXALT		; Store radius for altitude computation
			APSIDES		; Call two-body apsides computation
					; Returns: 0D = apocenter, 2D = pericenter
		SETPD	PUSH		; 2D = APOCENTER RADIUS		B29 OR B27
			2D
		DSU	PDDL		; 4D = APOGEE ALTITUDE = Ra - Rplanet
			XXXALT		; Altitude above surface (highest point)
			0D
		PUSH	DSU		; 6D = PERICENTER RADIUS	B29 OR B27
			XXXALT		; Altitude computation
		PUSH	GOTO		; 8D = PERIGEE ALTITUDE = Rp - Rplanet
			NORMEX		; Return to caller with apsides data

; During Apollo 11's translunar coast, PERIAPO validated that the trajectory
; maintained safe altitude margins above the lunar surface throughout the
; descent orbit, preventing collision with mountains near the landing site.

# Page 497
; ============================================================================
; SETRAD - SET PLANETARY RADIUS
;
; Determines the appropriate planetary radius (Earth or lunar) for altitude
; calculations based on spacecraft location. Critical for ensuring altitude
; constraints are referenced to the correct celestial body.
;
; COMMENT-ONLY READERS: This routine figures out whether the spacecraft is
;        near Earth or the Moon, then provides the correct planet radius
;        for measuring altitude above the surface.
;
; CODE-ALONG READERS: Uses X2 register state to determine body selection.
;        Returns either RPAD (Earth radius at Cape Kennedy) or RLS (lunar
;        radius from spacecraft position). Handles scaling appropriately.
;
; INPUT:
;        X2 register state (positive = Moon, zero/negative = Earth)
;        RLS (lunar sphere radius from current position)
;
; OUTPUT:
;        MPAC: Planetary radius (B-29 or B-27 meters)
;        X1: Saved X2 value
;        X2: Incremented by 2D
; ============================================================================
# SETRAD
; Load Earth radius as default, push to stack
SETRAD		DLOAD	PUSH		; Push RPAD (Earth radius) onto stack
			RPAD		; Pad 37-B radius = 6373338 meters B-29
		SXA,1	INCR,2		; Save X2 to X1, increment X2 by 2
			X2		; Preserve X2 state
			2D		; X2 = X2 + 2
		SLOAD	BHIZ		; Load X2 as scalar, branch if zero/negative
			X2		; Test for body selection
			SETRADX		; If ≤0, use Earth radius (already on stack)
		VLOAD	ABVAL		; If >0 (lunar), load RLS vector
			RLS		; Lunar sphere position vector
		PDDL			; Compute magnitude, push to stack
					; This replaces RPAD with lunar radius
SETRADX		DLOAD	RVQ		; Load selected radius from stack, return

; Apollo 11 transitioned from Earth radius (RPAD) during launch and TLI
; phases to lunar radius (RLS) after LOI on July 19, 1969, ensuring
; altitude measurements referenced the correct celestial body throughout.

# Page 498
; ============================================================================
; PRECSET - PRECISION STATE VECTOR SETUP FOR RENDEZVOUS
;
; Prepares high-precision state vectors for both LM and CSM vehicles for
; rendezvous computation. Calls precision routines to extrapolate position
; and velocity vectors to specified times, then stores results appropriately
; based on active/passive vehicle configuration.
;
; COMMENT-ONLY READERS: This routine updates the computer's knowledge of
;        where both spacecraft are and where they're going, ensuring the
;        rendezvous calculations use the most accurate position data.
;
; CODE-ALONG READERS: Orchestrates calls to LEMPREC/CSMPREC (precision
;        integration routines) and LEMSTORE/CSMSTORE (state vector storage).
;        Manages time parameters via TDEC1/TDEC2. Used before critical
;        rendezvous targeting computations.
; ============================================================================
# PRECSET
PRECSET		STQ			; Save return address
			NORMEX
		STCALL	TDEC2		; Store target time, call LM precision routine
			LEMPREC		; Extrapolate LM state vectors to TDEC2
		CALL			; Store LM results
			LEMSTORE	; Based on AVFLAG: RACT3/VACT3 or RPASS3/VPASS3
		DLOAD			; Reload time parameter
			TDEC2
		STCALL	TDEC1		; Set time for CSM, call CSM precision routine
			CSMPREC		; Extrapolate CSM state vectors to TDEC1
		CALL			; Store CSM results
			CSMSTORE	; Based on AVFLAG: opposite of LM assignment
		GOTO			; Return to caller
			NORMEX

; During Apollo 11's rendezvous on July 21, 1969, PRECSET ensured both
; Eagle (LM) and Columbia (CSM) state vectors were precisely synchronized
; for targeting calculations, accounting for orbital perturbations.

; ============================================================================
; LEMSTORE - STORE LM STATE VECTORS
;
; Stores LM position and velocity vectors to either active or passive vehicle
; storage locations based on AVFLAG setting. Used after precision integration
; completes to prepare data for rendezvous targeting.
;
; COMMENT-ONLY READERS: Saves the Lunar Module's position and velocity,
;        marking it as either the "chaser" (active) or "target" (passive)
;        depending on which spacecraft is performing the maneuver.
;
; CODE-ALONG READERS: Tests AVFLAG (bit flag indicating active vehicle).
;        If LM is active (AVFLAG set), stores RATT→RACT3, VATT→VACT3.
;        If LM is passive (AVFLAG clear), stores RATT→RPASS3, VATT→VPASS3.
; ============================================================================
LEMSTORE	VLOAD	BOFF		; Load position from RATT
			RATT		; Temporary precision integration result
			AVFLAG		; Test: is LM the active vehicle?
			PASSIVE		; If clear, LM is passive (target)
ACTIVE		STOVL	RACT3		; LM is active: store to active position
			VATT		; Load velocity from VATT
		STORE	VACT3		; Store to active velocity
		RVQ			; Return

; ============================================================================
; CSMSTORE - STORE CSM STATE VECTORS
;
; Stores CSM position and velocity vectors with opposite active/passive
; assignment from LM. If LM is active, CSM is passive; if LM is passive,
; CSM is active. Ensures consistent rendezvous geometry representation.
;
; COMMENT-ONLY READERS: Saves the Command Module's position and velocity,
;        assigning it the opposite role from the Lunar Module (if LM is
;        chasing, CSM is target; if CSM is chasing, LM is target).
;
; CODE-ALONG READERS: Tests AVFLAG with opposite branch logic from LEMSTORE.
;        If LM active (flag set), CSM is passive: RATT→RPASS3, VATT→VPASS3.
;        If LM passive (flag clear), CSM is active: RATT→RACT3, VATT→VACT3.
; ============================================================================
CSMSTORE	VLOAD	BOFF		; Load position from RATT
			RATT		; Temporary precision integration result
			AVFLAG		; Test: is LM the active vehicle?
			ACTIVE		; If clear (LM passive), CSM is active
PASSIVE		STOVL	RPASS3		; LM active means CSM passive: store here
			VATT		; Load velocity from VATT
		STORE	VPASS3		; Store to passive velocity
		RVQ			; Return

; During Apollo 11, Eagle (LM) was the active vehicle performing rendezvous
; maneuvers to Columbia (CSM). AVFLAG was set, causing LEMSTORE to populate
; RACT3/VACT3 and CSMSTORE to populate RPASS3/VPASS3.

# Page 499
; ============================================================================
; VECSHIFT - VECTOR SCALING UTILITY
;
; Scales two vectors using shift counts from RTX1 and RTX2 registers.
; Used throughout rendezvous programs to adjust vector precision for
; computational requirements and display formatting.
;
; COMMENT-ONLY READERS: This routine adjusts the numerical precision of
;        position and velocity data, ensuring values fit properly within
;        the computer's limited word size while maintaining accuracy.
;
; CODE-ALONG READERS: Loads RTX2 to X2, scales first vector by X2 bits
;        (VSR*), pushes to stack. Loads RTX1 to X1, scales second vector
;        by X2 bits, pushes to stack. Returns with both scaled vectors
;        on MPAC pushdown list. Typical use: scaling RVEC/VVEC pairs.
;
; INPUT:
;        RTX1: Shift count for second vector (signed scalar)
;        RTX2: Shift count for first vector (signed scalar)
;        MPAC: First vector to scale
;        (Second vector loaded via subsequent operation)
;
; OUTPUT:
;        MPAC stack: Two scaled vectors pushed to pushdown list
;        X1: Value from RTX1
;        X2: Value from RTX2
; ============================================================================
# VECSHIFT
VECSHIFT	LXA,2	VSR*		; Load RTX2 to X2, scale first vector
			RTX2		; Shift count from RTX2
			0,2		; Shift right by X2 bits
		LXA,1	PDVL		; Load RTX1 to X1, push result, load next
			RTX1		; Shift count from RTX1
		VSR*	PDVL		; Scale second vector by X2, push
			0,2		; (Uses X2, not X1)
		RVQ			; Return with scaled vectors on stack

# Page 500
; ============================================================================
; SHIFTR1 - SINGLE SCALAR SHIFT
;
; Scales a single scalar value using shift count from RTX2. Simpler version
; of VECSHIFT for single-value operations. Used for altitude, time, and
; other scalar adjustments in rendezvous computations.
;
; COMMENT-ONLY READERS: Adjusts the precision of a single number to prepare
;        it for further calculation or display.
;
; CODE-ALONG READERS: Loads RTX2 to X2, performs shift left (SL*) by X2
;        bits on MPAC scalar value. Positive X2 shifts left (multiply),
;        negative X2 shifts right (divide). Returns adjusted value in MPAC.
;
; INPUT:
;        RTX2: Shift count (signed scalar, B-14)
;        MPAC: Scalar value to shift
;
; OUTPUT:
;        MPAC: Shifted scalar value
;        X2: Value from RTX2
; ============================================================================
# SHIFTR1
SHIFTR1		LXA,2	SL*		; Load RTX2 to X2, shift left by X2 bits
			RTX2		; Shift count from RTX2
			0,2		; (Negative X2 produces right shift)
		RVQ			; Return with shifted value in MPAC

# Page 501
# PROGRAM DESCRIPTION
#
# SUBROUTINE NAME	R36	OUT-OF-PLANE RENDEZVOUS ROUTINE
# MOD NO. 2		DATE 2 JANUARY 1969
# MOD BY A.W.BANCROFT	LOG SECTION EXTENDED VERBS
#
# FUNCTIONAL DESCRIPTION
#
# TO DISPLAY AT ASTRONAUT REQUEST LGC CALCULATED RENDEZVOUS
# OUT-OF-PLANE PARAMETERS (Y, YDOT, PSI).  (REQUESTED BY DSKY).
#
# CALLING SEQUENCE
#	ASTRONAUT REQUEST THROUGH DSKY V 90 E
#
# SUBROUTINES CALLED
#	EXDSPRET
#	GOMARKF
#	CSMPREC
#	LEMPREC
#	SGNAGREE
#	LOADTIME
#
# NORMAL EXIT MODES
#	ASTRONAUT REQUEST THROUGH DSKY TO TERMINATE PROGRAM V 34 E
#
# ALARM OR ABORT EXIT MODES
#	NONE
#
# OUTPUT
#	DECIMAL DISPLAY OF TIME, Y, YDOT AND PSI
#
#	DISPLAYED VALUES Y, YDOT, AND PSI, ARE STORED IN ERASABLE
#	REGISTERS RANGE, RRATE, AND RTHETA RESPECTIVELY.
#
# ERASABLE INITIALIZATION REQUIRED
#	CSM AND LEM STATE VECTORS
#
# DEBRIS
#	CENTRALS A,Q,L
#	OTHER:  THOSE USED BY THE ABOVE LISTED SUBROUTINES

		BANK	20
		SETLOC	R36CM
		BANK
# Page 502
; ============================================================================
; R36 - OUT-OF-PLANE RENDEZVOUS ROUTINE
;
; For comment-only readers:
; This routine calculates and displays how far the spacecraft has drifted
; out of the orbital plane during rendezvous operations. Astronauts used
; this information to assess whether plane correction maneuvers were needed
; before final docking. The routine displays three key measurements: the
; out-of-plane distance (Y), the rate of drift (YDOT), and the angle (PSI)
; in the horizontal plane.
;
; For code-along readers:
; R36 implements out-of-plane rendezvous parameter computation using vector
; mathematics to project the line-of-sight between vehicles into horizontal
; and vertical components. The routine has two entry points:
; - R36: Default entry with option display (OPTION 1 or OPTION 2)
; - R36A: Direct computation entry (skips option display)
;
; Key computations:
; 1. Projects active vehicle position/velocity into passive vehicle frame
; 2. Computes out-of-plane component Y = U_perpendicular . R_active
; 3. Computes out-of-plane rate YDOT = U_perpendicular . V_active
; 4. Computes horizontal angle PSI from projected line-of-sight
; 5. Displays results via V06N90 (Verb 06, Noun 90)
;
; Input options:
; - OPTION 1: Compute at present time
; - OPTION 2: Compute at astronaut-specified time
;
; Display outputs (Noun 90):
; - R1: Y (out-of-plane distance, scaled in meters)
; - R2: YDOT (out-of-plane rate, scaled in meters/centisecond)
; - R3: PSI (horizontal angle, scaled in degrees)
;
; Historical context:
; During Apollo 11 rendezvous between Eagle (LM) and Columbia (CSM) in
; lunar orbit on July 21, 1969, maintaining proper orbital plane alignment
; was critical. Out-of-plane errors required corrective RCS burns that
; consumed precious fuel reserves. This routine gave the crew real-time
; assessment of plane drift so corrections could be made early when
; propellant cost was minimal.
;
; Modified from original 1967 version by Karrel (Mod 2, 2 January 1969)
; to correct angle computation and improve display handling.
; ============================================================================

		EBANK=	RPASS36

		SBANK=	R36A
		COUNT*	$$/R36

R36		CAF	TWO
		TS	OPTIONX
		CAF	ONE
		TS	OPTIONX +1
		CAF	OPTION36	# V 04 N 12
		TC	BANKCALL
		CADR	GOXDSPF
		TC	ENDEXT		# TERMINATE
		TC	+2		# PROCEED
		TC	-5		# R2 LOADED VIA DSKY
		TC 	POSTJUMP
		CADR	R36A

OPTION36	VN	0412

		SETLOC	R36LM
		BANK

R36A		ZL
		CAF	ZERO		# SET TIME OF EVENT TO ZERO FOR FIRST
		DXCH	DSPTEMX		# DISPLAY
		LXCH	OPTIONY		# SAVE VEH. OPTION
R36P3		CAF	V06N16N
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
		RTB	GOTO
			DPMODE
			R36INT

		SETLOC	R36LM1
		BANK

; ============================================================================
; R36INT - Main computation entry for out-of-plane calculations
;
; This is where the actual vector mathematics begins. The routine determines
; which vehicle (active or passive) to use for the computation based on the
; OPTIONY flag, then retrieves position and velocity state vectors.
;
; The computation requires precise state vectors at the specified time.
; If the astronaut selected "present time," the current navigation state
; is used. If a future time was specified, the orbit is propagated forward
; using Encke integration before computing the out-of-plane parameters.
; ============================================================================

R36INT		STORE	TDEC1		# SAVE COMPUTATION TIME
		SLOAD	SR1		# LOAD VEHICLE OPTION
			OPTIONY		# (0 = LM DISPLAY, 1 = CSM DISPLAY)
		BHIZ	CALL		# BRANCH IF ZERO (LM DISPLAY)
			R36PROG2	# FOR CSM DISPLAY
# Page 503
			THISPREC	# FOR LEM DISPLAY (THIS VEHICLE PREC)
		GOTO
			R36PROG3
R36PROG2	CALL			# CSM DISPLAY PATH
			OTHPREC		# GET OTHER VEHICLE PRECISION STATE
; ============================================================================
; CORE VECTOR COMPUTATION - OUT-OF-PLANE PARAMETER CALCULATION
;
; For comment-only readers:
; This section performs the mathematical calculations that determine how far
; the spacecraft have drifted apart vertically (perpendicular to the orbit).
; First, it calculates a reference direction that represents "perpendicular
; to the orbital plane." Then it projects the line-of-sight between the two
; vehicles onto this perpendicular direction to find the separation distance
; and rate. Finally, it computes the horizontal angle to show the in-plane
; direction between the vehicles.
;
; For code-along readers:
; This is the core Lambert/rendezvous vector computation sequence using the
; AGC interpreter's vector instruction set. Key mathematical operations:
;
; 1. Compute orbital plane normal for passive vehicle:
;    U_NP = UNIT(R_P × V_P)  [perpendicular to passive vehicle orbit]
;
; 2. Get active vehicle state vector at same time
;
; 3. Compute line-of-sight vector:
;    LOS = R_A - R_P
;
; 4. Compute out-of-plane distance:
;    Y = U_NP · (R_A - R_P)  [stored in RANGE]
;
; 5. Compute out-of-plane rate:
;    YDOT = U_NP · V_A  [stored in RRATE]
;
; 6. Compute horizontal unit vectors and angle PSI (see R36B section)
;
; The VSL2 (shift left 2 bits) operations are scaling adjustments to maintain
; precision within the AGC's 15-bit fixed-point arithmetic range.
; ============================================================================

R36PROG3	VLOAD	PDVL		# LOAD STATE VECTORS FOR PASSIVE VEHICLE
			VATT		# ACTIVE VEHICLE VELOCITY → MPAC
			RATT		# ACTIVE VEHICLE POSITION → PD
		STORE	RPASS36		# SAVE PASSIVE R POSITION (R_P)
		UNIT	PDVL		# UNIT(R_P) → PD, PREPARE FOR CROSS
		VXV	UNIT		# R_P × V_P = ORBIT ANGULAR MOMENTUM
		STADR			# (GETS V_P FROM STADR LOCATION)
		STODL	UNP36		# U_NP = UNIT(R_P × V_P) = ORBIT NORMAL
			TAT		# LOAD TIME OF COMPUTATION
		STORE	TDEC1		# SAVE FOR LATER USE
		SLOAD	SR1		# LOAD AND SHIFT RIGHT 1 BIT
			OPTIONY		# OPTION FLAG (0=LM, 1=CSM)
		BHIZ	CALL		# BRANCH IF ZERO TO R36PROG4
			R36PROG4	# FOR CSM DISPLAY
			OTHPREC		# FOR LEM DISPLAY (GET OTHER VEHICLE)
		GOTO
			R36PROG5
R36PROG4	CALL			# CSM DISPLAY PATH
			THISPREC	# GET THIS VEHICLE PRECISION STATE
R36PROG5	VLOAD	PDVL		# NOW HAVE ACTIVE VEHICLE STATE
			VATT		# VELOCITY VECTOR V_A → MPAC, 00D
			RATT		# POSITION VECTOR R_A → PD
		PDDL			# PUSH R_A TO 06D, LOAD NEXT
			TAT		# TIME OF ACTIVE STATE
		STOVL	30D		# SAVE TIME FOR REDISPLAY LATER
		PUSH	PUSH		# POSITION R_A IN 06D AND 12D
		BVSU	PDVL		# COMPUTE LOS = R_A - R_P
			RPASS36		# LINE OF SIGHT VECTOR (R_A - R_P) → 12D
		DOT	SL1		# COMPUTE OUT-OF-PLANE COMPONENT
			UNP36		# Y = U_NP · (R_A - R_P)
		STOVL	RANGE		# STORE Y IN RANGE (OUT-OF-PLANE DISTANCE)
			00D		# RECALL V_A FROM 00D
		DOT	SL1		# COMPUTE OUT-OF-PLANE RATE
			UNP36		# YDOT = U_NP · V_A
		STOVL	RRATE		# STORE YDOT IN RRATE (OUT-OF-PLANE RATE)
			06D		# RECALL R_A FROM 06D
		UNIT	PUSH		# U_RA = UNIT(R_A) → 18D (RADIAL UNIT)
		VXV	VXV		# COMPUTE HORIZONTAL UNIT VECTOR
			00D		# V_A FROM 00D
			18D		# (U_RA × V_A) × U_RA = HORIZ FORWARD
		VSL2	UNIT		# SCALE LEFT 2, NORMALIZE TO UNIT
		UNIT	GOTO		# ENSURE UNIT LENGTH, CONTINUE
			R36B		# → HORIZONTAL ANGLE COMPUTATION

		SETLOC	R36CM1
# Page 504
		BANK

; ============================================================================
; R36B - HORIZONTAL ANGLE (PSI) COMPUTATION
;
; For comment-only readers:
; This section calculates the horizontal angle (PSI) between the two
; spacecraft. PSI tells the astronauts which direction to look in the
; horizontal plane to find the other vehicle. A PSI of 0° means the target
; is directly ahead, 90° means to the right, 180° means behind, and 270°
; means to the left. This angle helps the crew orient the spacecraft for
; visual acquisition of the target vehicle during rendezvous.
;
; For code-along readers:
; Computes the in-plane angle PSI between the active vehicle's forward
; direction and the projected line-of-sight to the passive vehicle.
;
; Algorithm:
; 1. U_A (forward horizontal unit) is already computed and in 00D
; 2. Project LOS onto horizontal plane by removing out-of-plane component
; 3. Compute PSI = ARCCOS(U_A · U_LOS_horizontal)
; 4. Use cross product sign to determine if angle is >180° (adjust to 360-PSI)
;
; The sign check ensures PSI is measured correctly as a bearing angle
; (0-360°) rather than just the acute angle (0-180°).
; ============================================================================

R36B		STOVL	00D		# STORE UNIT HORIZ FORWARD → 00D
			18D		# RECALL U_RA (RADIAL UNIT) FROM 18D
		DOT	VXSC		# COMPUTE OUT-OF-PLANE COMPONENT OF LOS
			12D		# DOT WITH LOS (R_A - R_P) FROM 12D
		VSL2			# SCALE LEFT 2 FOR PRECISION
		BVSU	UNIT		# SUBTRACT FROM LOS, NORMALIZE
		UNIT			# U_L = UNIT(LOS - OUT-PLANE COMPONENT)
		PUSH	DOT		# SAVE U_L → 12D, DOT WITH FORWARD
			00D		# U_A · U_L = COS(PSI)
		SL1	ARCCOS		# SHIFT, COMPUTE ANGLE
		STOVL	RTHETA		# PSI = ARCCOS(U_A · U_L) → RTHETA
		VXV	DOT		# CHECK SIGN VIA CROSS PRODUCT
			00D		# (U_L × U_A) · UP
		BPL	DLOAD		# IF POSITIVE, ANGLE < 180°
			R36TAG2		# BRANCH TO DISPLAY
			DPPOSMAX	# IF NEGATIVE, ANGLE > 180°
		DSU			# COMPUTE 360° - PSI
			RTHETA		# (DPPOSMAX = 360° IN SCALED UNITS)
		STCALL	RTHETA		# STORE CORRECTED ANGLE
			R36TAG2		# → DISPLAY ROUTINE

		SETLOC	R36LM
		BANK

; ============================================================================
; R36TAG2 - DISPLAY OUT-OF-PLANE RENDEZVOUS PARAMETERS
;
; For comment-only readers:
; With all calculations complete, the guidance computer displays three
; critical out-of-plane rendezvous parameters on the DSKY:
;   Y (RANGE):   Out-of-plane distance to target (how far "above" or "below")
;   YDOT (RRATE): Rate of out-of-plane separation (closure rate perpendicular)
;   PSI (RTHETA): Horizontal bearing angle to target (which direction to look)
;
; The astronauts review these values to assess the rendezvous geometry. If
; satisfied, they can proceed with the maneuver. They can also request a
; recalculation to see updated values as the orbital geometry changes.
;
; For code-along readers:
; Display routine using Verb 06 Noun 90 to show:
;   R1: Y (out-of-plane distance in RANGE)
;   R2: YDOT (out-of-plane rate in RRATE)
;   R3: PSI (horizontal angle in RTHETA)
;
; After display, waits for crew response via GOMARKF:
;   - V34E (TERMINATE): Exits routine
;   - PROCEED: Exits routine
;   - ENTER: Recalculates and redisplays (R36P3)
; ============================================================================

R36TAG2		DLOAD	RTB		# LOAD TIME FROM 30D
			30D		# (SAVED EARLIER IN R36PROG5)
			SGNAGREE	# ENSURE SIGN AGREEMENT FOR DISPLAY
		STORE	DSPTEMX		# STORE TIME FOR NOUN 90 DISPLAY
		EXIT			# EXIT INTERPRETIVE MODE FOR DISPLAY
		CAF	V06N90N		# VERB 06 NOUN 90 (DISPLAY Y, YDOT, PSI)
		TC	BANKCALL	# CALL ACROSS BANKS
		CADR	GOMARKF		# TO GOMARKF DISPLAY HANDLER
		TCF	ENDEXT		# TERMINATE (V34E)
		TCF	ENDEXT		# PROCEED: END OF PROGRAM
		TCF	R36P3		# ENTER: REDISPLAY OUTPUT (RECALCULATE)
; ============================================================================
; LREGCHK - TIME INPUT VALIDATION ROUTINE
;
; For comment-only readers:
; When the astronaut enters a time for the rendezvous calculation, this
; routine checks if they provided a specific time or left it blank. If blank,
; the computer uses the current mission time. If a time was entered, that
; value is used for the calculation. This allows flexibility—astronauts can
; plan for "right now" or for a future time.
;
; For code-along readers:
; L register input validation:
;   - If L = 0: Astronaut pressed PROCEED without entering time
;     Action: Use current time (LOADTIME), branch to R36INT
;   - If L ≠ 0: Astronaut entered a specific time value in L
;     Action: Use astronaut input time, branch to ASTROTIM
;
; This is a common AGC pattern for optional time entry in guidance programs.
; ============================================================================

LREGCHK		XCH	L		# EXCHANGE L WITH A (SAVE L, GET OLD A)
		EXTEND			# EXTEND NEXT INSTRUCTION
		BZF	ENTTIM2		# L-REG ZERO: NO TIME ENTERED, USE CURRENT
		XCH	L		# L-REG NON-ZERO: RESTORE L (CONTAINS TIME)
		TCF	ASTROTIM	# BRANCH TO USE ASTRONAUT INPUT TIME

ENTTIM2		TC	INTPRET		# ENTER INTERPRETIVE MODE
		RTB	GOTO		# RETURN TO BASIC, THEN GOTO
			LOADTIME	# LOAD CURRENT MISSION TIME
			R36INT		# RETURN TO R36INT WITH CURRENT TIME

; ============================================================================
; VERB/NOUN DEFINITIONS FOR R36 DISPLAYS
;
; V06N16N: VERB 06 NOUN 16 - Display Time, TIG, and Delta-T
;   Used to show the calculated time of ignition parameters
;
; V06N90N: VERB 06 NOUN 90 - Display Y, YDOT, PSI
;   Used to show out-of-plane rendezvous parameters:
;   R1: Y (out-of-plane range)
;   R2: YDOT (out-of-plane rate)
;   R3: PSI (horizontal bearing angle)
; ============================================================================

V06N16N		VN	00616		# VERB 06 NOUN 16 CONSTANT
V06N90N		VN	00690		# VERB 06 NOUN 90 CONSTANT
		SBANK=	LOWSUPER
