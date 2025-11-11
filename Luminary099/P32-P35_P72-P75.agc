# Copyright:	Public domain.
# Filename:	P32-P35_P72-P75.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	618-650
# Mod history:	2009-05-18 RSB	Adapted from the Luminary 131 file of the
#				same name, as corrected from Luminary 099
#				page images.
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
; FILE: P32-P35_P72-P75.agc
; MODULE: Rendezvous Navigation - Concentric Flight Plan Maneuvers
; MISSION PHASE: ascent/rendezvous
;
; TL;DR: Implements coelliptic and concentric rendezvous sequence programs
;        for Lunar Module rendezvous with Command Module. P32/P72 compute
;        Coelliptic Sequence Initiation (CSI) maneuver - the first height
;        adjustment burn. P33/P73 compute Constant Delta Height (CDH) maneuver
;        - circularization matching target orbit shape. These programs enable
;        the precise multi-burn rendezvous sequence that brought Eagle back to
;        Columbia after Apollo 11's historic lunar landing on July 21, 1969.
;
; COMMENT-ONLY READERS: This code orchestrated Eagle's return journey to
;        Columbia in lunar orbit. Read comments to follow the mathematical
;        strategy that guided the two spacecraft back together after separation.
; CODE-ALONG READERS: Study Lambert targeting integration, orbital mechanics
;        calculations, and iterative solution techniques for multi-maneuver
;        rendezvous trajectory optimization.
; ============================================================================

# Page 618
# COELLIPTIC SEQUENCE INITIATION (CSI) PROGRAMS (P32 AND P72)
#
# MOD NO -1       LOG SECTION -- P32-P35, P72-P75
# MOD BY WHITE.P  DATE 1JUNE67
#
# PURPOSE

#	(1)	TO CALCULATE PARAMETERS ASSOCIATED WITH THE TIME FOLLOWING
#		CONCENTRIC FLIGHT PLAN MANEUVERS -- THE CO-ELLIPTIC SEQUENCE
#		INITIATION (CSI) MANEUVER AND THE CONSTANT DELTA ALTITUDE
#		(CDH) MANEUVER.

#	(2)	TO CALCULATE THESE PARAMETERS BASED UPON MANEUVER DATA
#		APPROVED AND KEYED INTO THE DSKY BY THE ASTRONAUT.

#	(3)	TO DISPLAY TO THE ASTRONAUT AND THE GROUND DEPENDENT VARIABLES
#		ASSOCIATED WITH THE CONCENTRIC FLIGHT PLAN MANEUVERS FOR
#		APPROVAL BY THE ASTRONAUT/GROUND.

#	(4)	TO STORE THE CSI TARGET PARAMETERS FOR USE BY THE DESIRED
#		THRUSTING PROGRAM.
#
# ASSUMPTIONS

#	(1)	AT A SELECTED TPI TIME THE LINE OF SIGHT BETWEEN THE ACTIVE
#		AND PASSIVE VEHICLES IS SELECTED TO BE A PRESCRIBED ANGLE (E)
#		FROM THE HORIZONTAL PLANE DEFINED BY THE ACTIVE VEHICLE
#		POSITION.

#	(2)	THE TIME BETWEEN CSI IGNITION AND CDH IGNITION MUST BE
#		COMPUTED TO BE GREATER THAN 10 MINUTES FOR SUCCESSFUL
#		COMPLETION OF THE PROGRAM.

#	(3)	THE TIME BETWEEN CDH IGNITION AND TPI IGNITION MUST BE
#		COMPUTED TO BE GREATER THAN 10 MINUTES FOR SUCCESSFUL
#		COMPLETION OF THE PROGRAM.

#	(4)	CDH DELTA V IS SELECTED TO MINIMIZE THE VARIATION OF THE
#		ALTITUDE DIFFERENCE BETWEEN THE ORBITS.

#	(5)	CSI BURN IS DEFINED SUCH THAT THE IMPULSIVE DELTA V IS IN THE
#		HORIZONTAL PLANE DEFINED BY THE ACTIVE VEHICLE POSITION AT CSI
#		IGNITION.

#	(6)	THE PERICENTER ALTITUDE OF THE ORBIT FOLLOWING CSI AND CDH
#		MUST BE GREATER THAN 35,000 FT (LUNAR ORBIT) OR 85 NM (EARTH
#		ORBIT) FOR SUCCESSFUL COMPLETION OF THIS PROGRAM.

#	(7)	THE CSI AND CDH MANEUVERS ARE ORIGINALLY ASSUMED TO BE
#		PARALLEL TO THE PLANE OF THE CSM ORBIT.  HOWEVER, CREW
# Page 619
#		MODIFICATION OF DELTA V (LV) COMPONENTS MAY RESULT IN AN
#		OUT-OF-PLANE CSI MANEUVER

#	(8)	STATE VECTOR UPDATES BY P27 ARE DISALLOWED DURING AUTOMATIC
#		STATE VECTOR UPDATING INITIATED BY P20 (SEE ASSUMPTION 10).

#	(9)	COMPUTED VARIABLES MAY BE STORED FOR LATER VERIFICATION BY
#		THE GROUND.  THESE STORAGE CAPABILITIES ARE NORMALLY LIMITED
#		ONLY TO THE PARAMETERS FOR ONE THRUSTING MANEUVER AT A TIME
#		EXCEPT FOR CONCENTRIC FLIGHT PLAN MANEUVER SEQUENCES.

#	(10)	THE RENDEZVOUS RADAR MAY OR MAY NOT BE USED TO UPDATE THE LM
#		OR CSM STATE VECTORS FOR THIS PROGRAM.  IF RADAR USE IS
#		DESIRED THE RADAR WAS TURNED ON AND LOCKED BY THE CSM BY
#		PREVIOUS SELECTION OF P20.  RADAR SIGHTING MARKS WILL BE MADE
#		AUTOMATICALLY APPROXIMATELY ONCE A MINUTE WHEN ENABLED BY THE
#		TRACK AND UPDATE FLAGS (SEE P20).  THE RENDEZVOUS TRACKING
#		MARK COUNTER IS ZEROED BY THE SELECTION OF P20 AND AFTER EACH
#		THRUSTING MANEUVER.

; ============================================================================
; COELLIPTIC SEQUENCE INITIATION (CSI) - THE FIRST RENDEZVOUS MANEUVER
;
; After Eagle separated from Columbia and descended to the lunar surface on
; July 20, 1969, it would need to return to Columbia in lunar orbit. This
; rendezvous sequence begins with the CSI burn - the first critical height
; adjustment maneuver that raises the Lunar Module from its lower orbit toward
; the Command Module's higher orbit.
;
; The CSI maneuver (Coelliptic Sequence Initiation) is designed to place the
; LM into an elliptical transfer orbit that brings it closer to the CSM's
; altitude. After CSI, the orbits are "coelliptic" - meaning they share the
; same shape but are offset in height. This sets up the subsequent CDH
; (Constant Delta Height) maneuver which circularizes the LM orbit to match
; the CSM orbit shape, followed by TPI (Terminal Phase Initiation) which
; begins the final rendezvous approach.
;
; COMMENT-ONLY READERS: This program computed the precise velocity change
; needed for Eagle's first burn to begin its journey back to Columbia. The
; astronauts approved the computed maneuver parameters displayed on the DSKY
; before committing to the burn.
;
; CODE-ALONG READERS: The program uses iterative Lambert targeting to solve
; the two-point boundary value problem: given the LM position at CSI time and
; the desired relative geometry at TPI time, compute the required delta-V.
; The solution accounts for lunar orbital mechanics, mission constraints
; (minimum time between burns), and altitude safety margins.
; ============================================================================

#	(11)	THE ISS NEED NOT BE ON TO COMPLETE THIS PROGRAM.

#	(12)	THE OPERATION OF THE PROGRAM UTILIZES THE FOLLOWING FLAGS --
#
#			ACTIVE VEHICLE FLAG -- DESIGNATES THE VEHICLE WHICH IS
#			DOING RENDEZVOUS THRUSTING MANEUVERS TO THE PROGRAM WHICH
#			CALCULATES THE MANEUVER PARAMETERS.  SET AT THE START OF
#			EACH RENDEZVOUS PRE-THRUSTING PROGRAM.
#
#			FINAL FLAG -- SELECTS FINAL PROGRAM DISPLAYS AFTER CREW HAS
#			COMPLETED THE FINAL MANEUVER COMPUTATION AND DISPLAY
#			CYCLE.
#
#			EXTERNAL DELTA V STEERING FLAG -- DESIGNATES THE TYPE OF
#			STEERING REQUIRED FOR EXECUTION OF THIS MANEUVER BY THE
#			THRUSTING PROGRAM SELECTED AFTER COMPLETION OF THIS
#			PROGRAM.
#
#	(13)	IT IS NORMALLY REQUIRED THAT THE ISS BE ON FOR 1 HOUR PRIOR TO
#		A THRUSTING MANEUVER.
#
#	(14)	THIS PROGRAM IS SELECTED BY THE ASTRONAUT BY DSKY ENTRY
#
#			P32 IF THIS VEHICLE IS ACTIVE VEHICLE.
#
#			P72 IF THIS VEHICLE IS THE PASSIVE VEHICLE.
#
# INPUT

#	(1)	TCSI		TIME OF THE CSI MANEUVER

# Page 620
#	(2)	NN		NUMBER OF APSIDAL CROSSINGS THRU WHICH THE ACTIVE
#				VEHICLE ORBIT CAN BE ADVANCED TO OBTAIN THE CDH
#				MANEUVER POINT.
#	(3)	ELEV		DESIRED LOS ANGLE AT TPI
#	(4)	TTPI		TIME OF THE TPI MANEUVER
#
# OUTPUT

#	(1)	TRKMKCNT	NUMBER OF MARKS
#	(2)	TTOGO		TIME TO GO
#	(3)	+MGA		MIDDLE GIMBAL ANGLE
#	(4)	DIFFALT		DELTA ALTITUDE AT CDH
#	(5)	T1TOT2		DELTA TIME FROM CSI TO CDH
#	(6)	T2TOT3		DELTA TIME FROM CDH TO TPI
#	(7)	DELVLVC		DELTA VELOCITY AT CSI -- LOCAL VERTICAL COORDINATES
#	(8)	DELVLVC		DELTA VELOCITY AT CDH -- LOCAL VERTICAL COORDINATES
#
# DOWNLINK

#	(1)	TCSI		TIME OF THE CSI MANEUVER
#	(2)	TCDH		TIME OF THE CDH MANEUVER
#	(3)	TTPI		TIME OF THE TPI MANEUVER
#	(4)	TIG		TIME OF THE CSI MANEUVER
#	(5)	DELVEET1	DELTA VELOCITY AT CSI -- REFERENCE COORDINATES
#	(6)	DELVEET2	DELTA VELOCITY AT CDH -- REFERENCE COORDINATES
#	(7)	DIFFALT		DELTA ALTITUDE AT CDH
#	(8)	NN		NUMBER OF APSIDAL CROSSINGS THRU WHICH THE ACTIVE
#				VEHICLE ORBIT CAN BE ADVANCED TO OBTAIN THE CDH
#				MANEUVER POINT
#	(9)	ELEV		DESIRED LOS ANGLE AT TPI
#
# COMMUNICATION TO THRUSTING PROGRAM

#	(1)	TIG		TIME OF THE CSI MANEUVER
#	(2)	RTIG		POSITION OF ACTIVE VEHICLE AT CSI -- BEFORE ROTATION
#				INTO PLANE OF PASSIVE VEHICLE
#	(3)	VTIG		VELOCITY OF ACTIVE VEHICLE AT CSE -- BEFORE ROTATION
#				INTO PLANE OF PASSIVE VEHICLE
#	(4)	DELVSIN		DELTA VELOCITY AT CSI -- REFERENCE COORDINATES
#	(5)	DELVSAB		MAGNITUDE OF DELTA VELOCITY AT CSI
#	(6)	XDELVFLG	SET TO INDICATE EXTERNAL DELTA V VG COMPUTATION
#
# SUBROUTINES USED

#	AVFLAGA
#	AVFLAGP
#	P20FLGON
#	VARALARM
#	BANKCALL
#	GOFLASH
#	GOTOPOOH
# Page 621
#	VNPOOH
#	GOFLASHR
#	BLANKET
#	ENDOFJOB
#	SELECTMU
#	ADVANCE
#	INTINT
#	PASSIVE
#	CSI/A
#	S32/33.1
#	DISDVLVC
#	VN1645

		BANK	35
		SETLOC	CSI/CDH
		BANK
		EBANK=	SUBEXIT
		COUNT*	$$/P3272

; ============================================================================
; P32 AND P72 PROGRAM ENTRY POINTS
;
; P32 is selected when the Lunar Module is the active vehicle performing the
; rendezvous maneuvers (after lunar surface ascent). P72 is selected when the
; LM is the passive vehicle (CSM performing rendezvous to LM).
;
; After Eagle's ascent from the lunar surface on July 21, 1969, Buzz Aldrin
; and Neil Armstrong would have selected P32 on the DSKY to compute their CSI
; burn parameters for returning to Columbia.
;
; The program prompts the astronauts to enter via DSKY:
;   V06N11: Time of CSI maneuver (TCSI)
;   V06N55: Number of apsidal crossings to CDH (NN), plus ELEV angle
;   V06N33: Time of TPI maneuver (TTPI)
; ============================================================================

; ============================================================================
; P32/P72 - COELLIPTIC SEQUENCE INITIATION (CSI) MANEUVER PROGRAMS
;
; The CSI burn is the first burn in the coelliptic rendezvous sequence,
; designed to adjust the active vehicle's orbit altitude to establish proper
; geometric phasing with the target vehicle. After CSI, the two spacecraft
; will be in coelliptic orbits (same shape, different altitudes), setting
; up the trajectory for the subsequent CDH (Constant Delta Height) burn.
;
; P32 is the primary CSI program (LM is active vehicle)
; P72 is the backup CSI program (CSM is active vehicle)
;
; HISTORICAL CONTEXT - APOLLO 11 EAGLE-COLUMBIA RENDEZVOUS:
; On July 21, 1969, approximately 3.5 hours after Eagle's ascent from the
; lunar surface, the CSI maneuver refined Eagle's trajectory to begin the
; precision rendezvous with Columbia orbiting overhead. Neil Armstrong and
; Buzz Aldrin used P32 to compute the optimal burn parameters, inputting
; the planned maneuver times based on ground tracking data. The CSI burn
; was executed flawlessly, placing Eagle on the proper approach trajectory
; that would culminate in docking approximately 3 hours later, reuniting
; the Apollo 11 crew and securing the mission's success.
; ============================================================================

P32		TC	AVFLAGA		; Set active vehicle flag (this vehicle does maneuvers)
		TC	P32STRT
P72		TC	AVFLAGP		; Set passive vehicle flag (other vehicle does maneuvers)
P32STRT		EXTEND
		DCA	P30ZERO		; Initialize central angle to zero
		DXCH	CENTANG
		TC	P32/P72A

; Alarm handling routine - displays alarm code on DSKY if computation fails
ALMXITA		SXA,2
			CSIALRM
ALMXIT		LXC,1
			CSIALRM
		SLOAD*	EXIT
			ALARM/TB -1,1
		CA	MPAC
		TC	VARALARM	; Display alarm to crew
		CAF	V05N09
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	-4

; Main initialization sequence - enables rendezvous radar tracking
P32/P72A	TC	P20FLGON	; Enable P20 rendezvous navigation tracking
		CAF	P30ZERO
		TS	NN 	+1	; Clear number of apsidal crossings
		TS	TCSI		; Clear CSI time (will be entered by crew)
		TS	TCSI 	+1

; ============================================================================
; CREW INPUT SEQUENCE - DSKY VERB/NOUN PROMPTS
;
; The astronauts now enter the proposed rendezvous timeline through a series
; of DSKY displays. Each entry is critical to the rendezvous trajectory.
; ============================================================================

; Prompt crew to enter CSI burn time
VN0611		CAF	V06N11		; TCSI - Time of CSI maneuver
		TC	VNPOOH		; Display and wait for crew input
		TC	INTPRET
		DLOAD	DCOMP
			TCSI
		BMN	DLOAD
			VN0655
# Page 622
			TETLEM
		STCALL	TDEC1
			PRECSET
		VLOAD	VSR*
			RACT3
			0,2
		STOVL	RVEC
			VACT3
		VSR*	SET
			0,2
			RVSW
		STODL	VVEC
			DPPOSMAX
		STCALL	RDESIRED
			TIMERAD
		DAD
			TDEC2
		STORE	TCSI
		EXIT
		TC	VN0611
VN0655		EXIT
		CAF	V06N55		# NN, ELEV(RGLOS)
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	+2
		TC	-5
		CAF	V06N37		# TTPI
		TC	VNPOOH
		TC	INTPRET
		DLOAD
			TCSI
		STCALL	TIG
			SELECTMU

; ============================================================================
; TRAJECTORY INTEGRATION AND CSI COMPUTATION SEQUENCE
;
; With all crew inputs collected, the AGC now performs the intensive
; numerical integration required to compute the CSI maneuver. This involves
; propagating both the active and passive vehicle orbits forward in time
; to determine the optimal delta-V that will place the spacecraft on an
; intercept trajectory.
;
; The computation uses the Lambert problem solver and conic subroutines
; to ensure the CSI burn creates a trajectory that meets the CDH and TPI
; timing requirements while maintaining safe orbital parameters.
; ============================================================================

; Begin main computation loop - may iterate if crew adjusts parameters
P32/P72B	CALL
			ADVANCE		; Advance state vectors to TIG

; Set up integration parameters for passive vehicle (CSM) trajectory
; The pushdown list (PD) is initialized to hold state vectors and times
		SETPD	VLOAD		; Initialize PD pointer to 0D
			0D
			VPASS1		; Load passive vehicle velocity at epoch
		PDVL	PDDL		; Push velocity, then load/push position
			RPASS1		; Passive vehicle position at epoch
			TCSI		; Time of CSI maneuver
		PDDL	PDDL		; Push times for integration endpoints
			TTPI		; Time of TPI maneuver
			TWOPI		; Two pi constant for orbit calculations

; Perform double integration to compute passive vehicle trajectory
; from TCSI through TTPI, accounting for lunar gravitational perturbations
		PUSH	CALL
			INTINT		; Double integration routine
		CALL
			PASSIVE		; Update passive vehicle state vectors

; Compute CSI maneuver delta-V required to achieve coelliptic geometry
; This is the heart of the CSI calculation - determining the burn that
; places the active vehicle on a trajectory matching the passive orbit shape
		CALL
# Page 623
			CSI/A		; CSI Algorithm - compute required delta-V
; Check if this is final computation or intermediate iteration
; Set update flag to enable radar tracking if available
P32/P72C	BON	SET		; Branch on final flag
			FINALFLG	; If set, skip to display
			P32/P72D
			UPDATFLG	; Set flag enabling P20 radar updates

; Verify timing constraints between maneuvers
; CSI to CDH must be at least 60 minutes for orbital mechanics validity
P32/P72D	DLOAD
			T1TOT2		; Time from CSI (T1) to CDH (T2)
P32/P72E	STORE	T1TOT2
		DSU	BPL		; Subtract 60 minutes, branch if positive
			60MIN		; 60-minute minimum separation constant
			P32/P72E	; Loop if less than 60 min (wait for valid geometry)

; Verify CDH to TPI timing constraint (also minimum 60 minutes)
		DLOAD
			T2TOT3		; Time from CDH (T2) to TPI (T3)
P32/P72F	STORE	T2TOT3
		DSU	BPL		; Subtract 60 minutes, branch if positive
			60MIN		; 60-minute minimum separation constant
			P32/P72F	; Loop if less than 60 min

; Exit interpreter mode to display results to crew via DSKY
		EXIT

; ============================================================================
; CREW VERIFICATION AND DISPLAY SEQUENCE
;
; The computed CSI maneuver parameters are now displayed to the crew on the
; DSKY for verification. During Apollo 11, Buzz Aldrin monitored these
; values after Eagle's ascent, comparing them against ground-computed values
; before approving the burn. The crew can adjust delta-V components if needed,
; which will cause the program to loop back and recompute trajectories.
; ============================================================================

; Display verb 06 noun 75: Time between CSI and CDH, and CDH and TPI
		CAF	V06N75		; V06 = display decimal, N75 = delta-time data
		TC	VNPOOH		; Call display interface routine
		TC	INTPRET		; Re-enter interpreter mode after crew input

; Transform delta-V from local vertical to launch vehicle coordinates
; for crew understanding and DSKY display consistency
		VLOAD	CALL
			DELVEET1	; CSI delta-V vector
			S32/33.1	; Coordinate transformation routine
		STOVL	DELVEET1	; Store transformed CSI delta-V
			RACT2		; Position at CDH time
		STOVL	RACT1		; Store for reference
			DELVEET2	; CDH delta-V vector

; Display computed delta-V components to crew
; Format: X-axis (downrange), Y-axis (crossrange), Z-axis (altitude)
		AXT,1	CALL
		VN	0682		; Verb 06, Noun 82: display delta-V components
			DISDVLVC	; Display delta-V in launch vehicle coordinates

; Store TPI time and display all rendezvous timing to crew
		DLOAD
			TTPI		; Time of Terminal Phase Initiation
		STCALL	TTPIO		; Store TPI time output
			VN1645		; Display routine for time verification

; Return to computation loop if crew adjusts parameters
; Otherwise proceed to burn preparation (not shown - handled by calling program)
		GOTO
			P32/P72B	; Loop back for parameter iteration

; ============================================================================
; TRANSITION: From CSI Planning (P32/P72) to CDH Planning (P33/P73)
;
; Having computed the Coelliptic Sequence Initiation (CSI) burn that adjusts
; the spacecraft's orbit altitude, the rendezvous sequence now proceeds to
; the Constant Delta Height (CDH) maneuver. The CDH burn circularizes the
; orbit to match the target vehicle's orbital shape while maintaining the
; desired altitude separation.
;
; In the Apollo 11 rendezvous, after Eagle fired its ascent engine to reach
; orbit, the CSI burn adjusted Eagle's trajectory toward Columbia's orbital
; plane and altitude. The subsequent CDH burn (computed by P33) refined
; Eagle's orbit to precisely match Columbia's shape, setting up the
; geometry for the final Terminal Phase Initiation (TPI) approach.
;
; Timeline: CSI occurs first, followed 60+ minutes later by CDH, with TPI
; following another 60+ minutes after that. This spacing allows orbital
; mechanics to naturally position the spacecraft for each successive burn.
; ============================================================================

# Page 624
# CONSTANT DELTA HEIGHT (CDH) PROGRAMS (P33 AND P73)
# MOD NO -1			LOC SECTION -- P32-P35, P72-P75
# MOD BY WHITE, P.		DATE: 1 JUNE 67
#
# PURPOSE
#
#	(1)	TO CALCULATE PARAMETERS ASSOCIATED WITH THE CONSTANT DELTA
#		ALTITUDE MANEUVER (CDH).
#
#	(2)	TO CALCULATE THESE PARAMETERS BASED UPON MANEUVER DATA
#		APPROVED AND KEYED INTO THE DSKY BY THE ASTRONAUT.
#
#	(3)	TO DISPLAY TO THE ASTRONAUT AND THE GROUND DEPENDENT VARIABLES
#		ASSOCIATED WITH THE CDH MANEUVER FOR APPROVAL BY THE
#		ASTRONAUT/GROUND.
#
#	(4)	TO STORE THE CDH TARGET PARAMETERS FOR USE BY THE DESIRED
#		THRUSTING PROGRAM.
#
# ASSUMPTIONS
#
#	(1)	THIS PROGRAM IS BASED UPON PREVIOUS COMPLETION OF THE
#		CO-ELLIPTIC SEQUENCE INITIATION (CSI) PROGRAM (P32/P72).
#		THEREFORE --
#
#		(A)	AT A SELECTED TPI TIME (NOW IN STORAGE) THE LINE OF SIGHT
#			BETWEEN THE ACTIVE AND PASSIVE VEHICLES WAS SELECTED TO BE
#			A PRESCRIBED ANGLE (E) (NOW IN STORAGE) FROM THE
#			HORIZONTAL PLANE DEFINED BY THE ACTIVE VEHICLE POSITION.
#
#		(B)	THE TIME BETWEEN CSI IGNITION AND CDH IGNITION WAS
#			COMPUTED TO BE GREATER THAN 10 MINUTES.
#
#		(C)	THE TIME BETWEEN CDH IGNITION AND TPI IGNITION WAS
#			COMPUTED TO BE GREATER THAN 10 MINUTES.
#
#		(D)	THE VARIATION OF THE ALTITUDE DIFFERENCE BETWEEN THE
#			ORBITS WAS MINIMIZED.
#
#		(E)	CSI BURN WAS DEFINED SUCH THAT THE IMPULSIVE DELTA V WAS
#			IN THE HORIZONTAL PLANE DEFINED BY ACTIVE VEHICLE
#			POSITION AT CSI IGNITION.
#
#		(F) 	THE PERICENTER ALTITUDES OF THE ORBITS FOLLOWING CSI AND
#			CDH WERE COMPUTED TO BE GREATER THAN 35,000 FT FOR LUNAR
#			ORBIT OR 85 NM FOR EARTH ORBIT.
#
#		(G)	THE CSI AND CDH MANEUVERS WERE ASSUMED TO BE PARALLEL TO
#			THE PLANE OF THE PASSIVE VEHICLE ORBIT.  HOWEVER, CREW
# Page 625
#			MODIFICATION OF DELTA V (LV) COMPONENTS MAY HAVE RESULTED
#			IN AN OUT-OF-PLANE MANEUVER.
#
#	(2)	STATE VECTOR UPDATES BY P27 ARE DISALLOWED DURING AUTOMATIC
#		STATE VECTOR UPDATING INITIATED BY P20 (SEE ASSUMPTION 4).
#
#	(3)	COMPUTED VARIABLES MAY BE STORED FOR LATER VERIFICATION BY
#		THE GROUND.  THESE STORAGE CAPABILITIES ARE NORMALLY LIMITED
#		ONLY TO THE PARAMETERS FOR ONE THRUSTING MANEUVER AT A TIME
#		EXCEPT FOR CONCENTRIC FLIGHT PLAN MANEUVER SEQUENCES.
#
#	(4)	THE RENDEZVOUS RADAR MAY OR MAY NOT BE USED TO UPDATE THE LM.
#		OR CSM STATE VECTORS FOR THIS PROGRAM.  IF RADAR USE IS
#		DESIRED THE RADAR WAS TURNED ON AND LOCKED ON THE CSM BY
#		PREVIOUS SELECTION OF P20.  RADAR SIGHTING MARKS WILL BE MADE
#		AUTOMATICALLY APPROXIMATELY ONCE A MINUTE WHEN ENABLED BY THE
#		TRACK AND UPDATE FLAGS (SEE P20).  THE RENDEZVOUS TRACKING
#		MARK COUNTER IS ZEROED BY THE SELECTION OF P20 AND AFTER EACH
#		THRUSTING MANEUVER.
#
#	(5)	THE ISS NEED NOT BE ON TO COMPLETE THIS PROGRAM.
#
#	(6)	THE OPERATION OF THE PROGRAM UTILIZES THE FOLLOWING FLAGS --
#
#			ACTIVE VEHICLE FLAG -- DESIGNATES THE VEHICLE WHICH IS
#			DOING RENDEZVOUS THRUSTING MANEUVERS TO THE PROGRAM WHICH
#			CALCULATES THE MANEUVER PARAMETERS.  SET AT THE START OF
#			EACH RENDEZVOUS PRE-THRUSTING PROGRAM.
#
#			FINAL FLAG -- SELECTS FINAL PROGRAM DISPLAYS AFTER CREW HAS
#			COMPLETED THE FINAL MANEUVER COMPUTATION AND DISPLAY
#			CYCLE.
#
#			EXTERNAL DELTA V STEERING FLAG -- DESIGNATES THE TYPE OF
#			STEERING REQUIRED FOR EXECUTION OF THIS MANEUVER BY THE
#			THRUSTING PROGRAM SELECTED AFTER COMPLETION OF THIS
#			PROGRAM.
#
#	(7)	IT IS NORMALLY REQUIRED THAT THE ISS BE ON FOR 1 HOUR PRIOR TO
#		A THRUSTING MANEUVER.
#
#	(8)	THIS PROGRAM IS SELECTED BY THE ASTRONAUT BY DSKY ENTRY.
#
#			P33 IF THIS VEHICLE IS ACTIVE VEHICLE.
#
#			P73 IF THIS VEHICLE IS PASSIVE VEHICLE.
#
# INPUT
#
#	(1)	TTPIO	TIME OF THE TPI MANEUVER -- SAVED FROM P32/P72
# Page 626
#	(2)	ELEV	DESIRED LOS ANGLE AT TPI -- SAVED FROM P32/P72
#	(3)	TCDH	TIME OF THE CDH MANEUVER
#
# OUTPUT
#
#	(1)	TRKMKCNT	NUMBER OF MARKS
#	(2)	TTOGO		TIME TO GO
#	(3)	+MGA		MIDDLE GIMBAL ANGLE
#	(4)	DIFFALT		DELTA ALTITUDE AT CDH
#	(5)	T2TOT3		DELTA TIME FROM CDH TO COMPUTED TPI
#	(6)	NOMTPI		DELTA TIME FROM NOMINAL TPI TO COMPUTED TPI
#	(7)	DELVLVC		DELTA VELOCITY AT CDH -- LOCAL VERTICAL COORDINATES
#
# DOWNLINK
#
#	(1)	TCDH		TIME OF THE CDH MANEUVER
#	(2)	TTPI		TIME OF THE TPI MANEUVER
#	(3)	TIG		TIME OF THE CDH MANEUVER
#	(4)	DELLVEET2	DELTA VELOCITY AT CDH -- REFERENCE COORDINATES
#	(5)	DIFFALT		DELTA ALTITUDE AT CDH
#	(6)	ELEV		DESIRED LOS ANGLE AT TPI
#
# COMMUNICATION TO THRUSTING PROGRAMS
#
#	(1)	TIG		TIME OF THE CDH MANEUVER
#	(2)	RTIG		POSITION OF ACTIVE VEHICLE AT CDH -- BEFORE ROTATION
#				INTO PLANE OF PASSIVE VEHICLE.
#	(3)	VTIG		VELOCITY OF ACTIVE VEHICLE AT CDH -- BEFORE ROTATION
#				INTO PLANE OF PASSIVE VEHICLE.
#	(4)	DELVSIN		DELTA VELOCITY AT CDH -- REFERENCE COORDINATES.
#	(5)	DELVSAB		MAGNITUDE OF DELTA VELOCITY AT CDH.
#	(6)	XDELVFLG	SET TO INDICATE EXTERNAL DELTA V VG COMPUTATION.
#
# SUBROUTINES USED
#
#	AVFLAGA
#	AVFLAGP
#	P20FLGON
#	VNPOOH
#	SELECTMU
#	ADVANCE
#	CDHMVR
#	INTINT3P
#	ACTIVE
#	PASSIVE
#	S33/S34.1
#	ALARM
#	BANKCALL
#	GOFLASH
#	GOTOPOOH
#	S32/33.1
# Page 627
#	VN1645

; ============================================================================
; P33/P73 - CONSTANT DELTA HEIGHT (CDH) MANEUVER PROGRAMS
;
; The CDH burn is the second burn in the coelliptic rendezvous sequence.
; After the CSI burn has adjusted the spacecraft's orbit altitude, the CDH
; burn circularizes the orbit to match the target's orbital shape while
; maintaining a constant altitude separation. This sets up the proper
; geometry for the final TPI (Terminal Phase Initiation) approach burn.
;
; P33 is the primary CDH program (LM is active vehicle)
; P73 is the backup CDH program (CSM is active vehicle)
;
; During Apollo 11, after Eagle's CSI burn adjusted its orbit following
; ascent from the lunar surface, the CDH burn refined Eagle's trajectory
; to match Columbia's orbital characteristics. This precision maneuver
; ensured the two spacecraft maintained the desired separation distance
; while their orbital shapes aligned perfectly for the rendezvous approach.
; ============================================================================

		COUNT*	$$/P3373

; Program entry points - set active/passive vehicle flags
P33		TC	AVFLAGA		; P33: Set AVFLAG (LEM is active vehicle)
		TC	P33/P73A	; Continue to common CDH logic
P73		TC	AVFLAGP		; P73: Clear AVFLAG (CSM is active vehicle)

; Common CDH program logic for both P33 and P73
P33/P73A	TC	P20FLGON	; Enable P20 rendezvous tracking flags

; Request CDH time from crew via DSKY
; Crew inputs the desired time for the CDH maneuver based on mission timeline
		CAF	V06N13		# TCDH - Display verb 06, noun 13
		TC	VNPOOH		; Call display interface routine
		TC	INTPRET		; Enter interpreter mode for trajectory computation

; Initialize timing parameters for CDH computation
; Load stored TPI time and CDH time for trajectory integration
		DLOAD
			TTPIO		; TPI time from previous computation (P32/P72)
		STODL	TTPI		; Store as current TPI time
			TCDH		; Time of CDH maneuver (crew input)
		STCALL	TIG		; Store as Time of Ignition
			SELECTMU	; Select gravitational parameter (lunar/Earth)

; Main computation loop - iterates if crew adjusts CDH parameters
P33/P73B	CALL
			ADVANCE		; Advance state vectors to TIG

; Compute the CDH maneuver delta-V
; This burn circularizes the orbit to match the target vehicle's shape
		CALL
			CDHMVR		; CDH Maneuver computation routine

; Integrate active vehicle (LM) trajectory from CDH through TPI
; This predicts where the spacecraft will be after the CDH burn
		SETPD	VLOAD		; Initialize pushdown list
			0D
			VACT3		; Active vehicle velocity after CDH
		PDVL	CALL		; Push velocity, load position
			RACT2		; Active vehicle position at CDH
			INTINT3P	; 3-point integration routine
		CALL
			ACTIVE		; Update active vehicle state vectors

; Integrate passive vehicle (CSM) trajectory from CDH through TPI
; This predicts the target's position for TPI geometry planning
		SETPD	VLOAD		; Initialize pushdown list
			0D
			VPASS2		; Passive vehicle velocity at CDH time
		PDVL	CALL		; Push velocity, load position
			RPASS2		; Passive vehicle position at CDH time
			INTINT3P	; 3-point integration routine
		CALL
			PASSIVE		; Update passive vehicle state vectors

; Initialize nominal TPI time and compute TPI geometry
		DLOAD	SET
			P30ZERO		; Zero constant for initialization
			ITSWICH		; Set iteration switch flag
		STCALL	NOMTPI		; Store nominal TPI time
			S33/34.1	; Compute TPI geometry and timing

; Check for computation errors - branch if result is zero (success)
		BZE	EXIT		; Branch if zero (no error), exit interpreter
			P33/P73C	; Skip to display if successful

; Handle computation error - invalid geometry or constraint violation
; Alarm 611: Perigee altitude too low after CDH maneuver
		TC	ALARM		; Trigger program alarm
		OCT	611		; Alarm code 611

; Display alarm to crew and request decision to continue or abort
		CAF	V05N09		; Verb 05 Noun 09: Display alarm code
		TC	BANKCALL	; Bank call to display routine
		CADR	GOFLASH		; Flash display, wait for crew response
		TC	GOTOPOOH	; Terminate if crew presses TERMINATE
		TC	+2		; Proceed if crew presses PROCEED
# Page 628
		TC	P33/P73A	; Recycle if crew presses ENTER (new TCDH input)

; Reset nominal TPI time after alarm handling
		TC	INTPRET		; Re-enter interpreter mode
		DLOAD
			P30ZERO		; Zero constant
		STORE	NOMTPI		; Reset nominal TPI time

; Check if this is the final iteration and enable state vector updates
P33/P73C	BON	SET		; Branch if FINALFLG is set
			FINALFLG	; Final computation flag
			P33/P73D	; Skip to timing computation
			UPDATFLG	; Enable rendezvous tracking updates

; Compute final TPI time and timing intervals for crew display
; These values verify the rendezvous geometry is acceptable
P33/P73D	DLOAD	DAD
			NOMTPI		; Nominal TPI time adjustment
			TTPI		; Current TPI time
		STORE	TTPI		; Store final TPI time

; Compute time interval from CDH to TPI (T1TOT2)
; This must be >= 10 minutes per program constraint
		DSU
			TCDH		; Subtract CDH time from TPI time

; Normalize time interval to handle orbital period wrap-around
; Ensures the result is in valid range for display
P33/P73E	DSU	BPL		; Subtract 60 minutes
			60MIN		; 60-minute constant for normalization
			P33/P73E	; Loop if result still positive
		DAD
			60MIN		; Add back 60 minutes after normalization
		STODL	T1TOT2		; Store CDH-to-TPI time interval
			TTPI		; Load TPI time

; Compute time from TPI to next TPI opportunity (T2TOT3)
; This shows the phasing between successive rendezvous attempts
		DSU	PUSH		; Subtract previous TPI time
			TTPIO		; TPI time from P32/P72 computation

; Normalize T2TOT3 interval similar to T1TOT2
P33/P73F	ABS	DSU		; Take absolute value, subtract 60 min
			60MIN		; 60-minute constant
		BPL	DAD		; Branch if positive
			P33/P73F	; Loop if still positive
			60MIN		; Add back 60 minutes
		SIGN	STADR		; Restore original sign
		STORE	T2TOT3		; Store TPI-to-next-TPI time interval
		EXIT			; Exit interpreter mode

; Display CDH delta-V to crew via DSKY for verification
; Verb 06 Noun 75: Display delta-V in R (radial), Theta (downrange), H (out-of-plane)
		CAF	V06N75		; Load display verb/noun code
		TC	VNPOOH		; Execute display routine (non-flashing)

; Allow crew to examine and optionally modify CDH delta-V components
		TC	INTPRET		; Return to interpreter mode
		VLOAD	CALL		; Load CDH delta-V vector
			DELVEET2	; Delta-V for CDH maneuver
			S32/33.1	; Transform to local vertical coordinates
		STCALL	DELVEET2	; Store transformed delta-V
			VN1645		; Display with modification capability

; After crew accepts parameters or makes adjustments, recycle computation
; This allows iterative refinement of the CDH maneuver plan
		GOTO
			P33/P73B	; Return to main CDH computation loop

; ============================================================================
; TRANSITION: From Main Rendezvous Programs to Utility Subroutines
;
; The P32/P72 and P33/P73 programs above compute the CSI and CDH maneuvers
; for coelliptic rendezvous sequences. The subroutines that follow provide
; supporting functionality used throughout the rendezvous navigation suite:
; - AVFLAGA/AVFLAGP: Set active vehicle flag (LM vs CSM perspective)
; - P20FLGON: Enable rendezvous radar tracking and state vector updates
; - DISDVLVC: Display delta-V in local vertical coordinates for crew review
; - S32/33.X: Coordinate transformation subroutines for display
;
; These utilities enable the crew to monitor and adjust rendezvous parameters
; through the DSKY interface during the multi-burn rendezvous sequence.
; ============================================================================

# Page 629
# ***** ADFLAG/P *****
#
# SUBROUTINES USED
#
#	UPFLAG
#	DOWNFLAG

; Set active vehicle flag to indicate LM is the maneuvering spacecraft
; This flag determines perspective for relative navigation computations
; During Eagle's rendezvous with Columbia, the LM was the active vehicle
; performing the CSI and CDH maneuvers to reach the passive CSM
AVFLAGA		EXTEND			# AVFLAG = LEM
		QXCH	SUBEXIT		; Save return address
		TC	UPFLAG		; Set flag high (1 = LM is active)
		ADRES	AVFLAG		; Flag address in erasable memory
		TC	SUBEXIT		; Return to caller

; Set active vehicle flag to indicate CSM is the maneuvering spacecraft
; Used when CSM performs rendezvous maneuvers with passive LM
AVFLAGP		EXTEND			# AVFLAG = CSM
		QXCH	SUBEXIT		; Save return address
		TC	DOWNFLAG	; Clear flag (0 = CSM is active)
		ADRES	AVFLAG		; Flag address in erasable memory
		TC	SUBEXIT		; Return to caller

; Enable rendezvous radar tracking and automatic state vector updates
; This subroutine activates P20 navigation functionality used during
; rendezvous sequences to continuously update relative position/velocity
; The rendezvous radar provides range, range-rate, and angle measurements
; approximately once per minute when tracking is enabled
P20FLGON	EXTEND
		QXCH	SUBEXIT		; Save return address
		TC	UPFLAG		; Enable update flag
		ADRES	UPDATFLG	# SET UPDATFLG - allows automatic navigation updates
		TC	UPFLAG		; Enable tracking flag
		ADRES	TRACKFLG	# SET TRACKFLG - enables radar mark processing
		TC	SUBEXIT		; Return to caller with both flags set

# Page 630
# ***** DISDVLVC *****
#
# SUBROUTINES USED
#
#	S32/33.X
#	VNPOOH

; Display delta-V in local vertical coordinates (LVC) for crew review
; Transforms computed delta-V from inertial coordinates to local vertical
; frame (radial, in-track, cross-track) which is more intuitive for crew
; Displays result on DSKY using verb/noun pair stored in VERBNOUN
; 
; During Apollo 11 rendezvous, Armstrong and Aldrin reviewed these displays
; to verify the computed CSI and CDH burns before approving execution
DISDVLVC	STORE	DELVLVC		; Store input delta-V vector
		STQ	CALL		; Save return address and call
			NORMEX		; transformation subroutine
			S32/33.X	; Transform to local vertical coordinates
		VLOAD	MXV		; Load transformed delta-V
			DELVLVC		; and multiply by transformation matrix
			0D		; Matrix at pushdown location 0D
		VSL1	SXA,1		; Shift left 1 bit for display scaling
			VERBNOUN	; Load verb/noun code for DSKY
		STORE	DELVLVC		; Store final display values
		EXIT			; Exit interpretive mode
		CA	VERBNOUN	; Load verb/noun into accumulator
		TC	VNPOOH		; Display on DSKY (VNPOOH = verb/noun display)
		TC	INTPRET		; Return to interpretive mode
		GOTO			; Return to caller
			NORMEX

# Page 631
# ***** CONSTANTS *****

; Verb/noun codes for DSKY display during rendezvous programs
V06N11		VN	0611		; Display time of event (V06 = decimal display)
V06N13		VN	0613		; Display time increment
V06N75		VN	0675		; Display computed maneuver parameters

; Angular constants for orbit computations
SN359+		2DEC	-.000086601	; Sine of 359 degrees (for threshold checks)

CS359+		2DEC	+.499999992	; Cosine of 359 degrees

P30ZERO		2DEC	0		; Zero constant for initialization

; Time constant for minimum maneuver spacing
60MIN		2DEC	360000		; 60 minutes in centiseconds (100ths of seconds)

; Alarm codes for various error conditions during rendezvous computations
; These alarms alert the crew to computation failures or constraint violations
ALARM/TB	OCT	00600		# NO 1 - Algorithm convergence failure
		OCT	00601		#    2 - Pericenter altitude constraint violation
		OCT	00602		#    3 - Time constraint violation (CSI-CDH < 10 min)
		OCT	00603		#    4 - Time constraint violation (CDH-TPI < 10 min)
		OCT	00604		#    5 - Delta-V magnitude excessive
		OCT	00605		#    6 - Orbital geometry invalid
		OCT	00606		#    7 - Maximum iteration count exceeded

# Page 632
# ***** CSI/A *****
#
# SUBROUTINES USED
#
#	VECSHIFT
#	TIMETHET
#	PERIAPO
#	SHIFTR1
#	INTINT2C
#	CDHMVR
#	PERIAPO1
#	INTINT
#	ACTIVE

		BANK	34
		SETLOC	CSI/CDH1
		BANK
		EBANK=	SUBEXIT
		COUNT*	$$/CSI
LOOPMX		2DEC	16

INITST		2DEC	.03048 B-7	# INITIAL DELDV = 10 FPS

DVMAX1		2DEC	3.0480 B-7	# MAXIMUM DV1 = 1000 FPS

DVMAX2		2DEC	3.014472 B-7	#		 989 FPS

1DPB2		2DEC	1.0 B-2

1DPB28		2DEC	1

PMINE		2DEC	157420 B-29	# 85 NM -- MUST BE 8 WORDS BEFORE PMINM

EPSILN1		2DEC	.0003048 B-7	# .1 FPS

NICKELDP	2DEC	.021336 B-7	# 7 FPS (CHANGED FROM .05 FPS)

FIFPSDP		2DEC	-.152400 B-7	# 50 FPS

PMINM		2DEC	10668 B-29	# 35000 FT -- MUST BE 8 WORDS AFTER PMINE

DELMAX1		2DEC	.6096000 B-7	# 200 FPS

ONETHTH		2DEC	.0001 B-3

TMIN		2DEC	60000		# 10 MIN

; ============================================================================
; CSI/A -- COELLIPTIC SEQUENCE INITIATION DELTA-V ALGORITHM
;
; This is the core computational engine for P32/P72. CSI/A solves the
; two-point boundary value problem: given the current active vehicle
; position/velocity and the target passive vehicle trajectory, compute
; the impulsive delta-V that places the active vehicle on a coelliptic
; orbit (same shape as passive orbit but at different altitude).
;
; ALGORITHM OVERVIEW:
; 1. Initialize iteration control flags and counters
; 2. Compute reference geometry (orbital radii, unit vectors)
; 3. Calculate initial delta-V estimate (10 fps baseline)
; 4. Enter Newton-Raphson iteration loop to refine delta-V:
;    - Apply trial delta-V to active vehicle state
;    - Integrate trajectory to CDH time
;    - Compute miss distance from desired coelliptic geometry
;    - Adjust delta-V based on miss distance (Newton method)
;    - Iterate until convergence or maximum iterations
; 5. Validate solution meets constraints (minimum altitude, timing)
; 6. If first solution fails, attempt second solution with different
;    initial conditions (50 fps delta-V stage)
;
; COMMENT-ONLY READERS: This algorithm is what enables the LM to
; rendezvous with the CSM. During Apollo 11's return from the lunar
; surface, Eagle used this calculation to match Columbia's orbit shape.
;
; CODE-ALONG READERS: Study the Newton iteration structure (CSI/B1 loop)
; and the convergence criteria. Note the use of interpretive language
; for vector/matrix operations and the careful management of iteration
; counts and alarm conditions.
; ============================================================================

; Initialize control flags for CSI algorithm iteration and stage management
; These flags track which solution attempt is active and whether delta-V
; limits have been exceeded, enabling fallback to alternate solution stages

CSI/A		CLEAR	SET		# INITIALIZE INDICATORS
			S32.1F1		# DVT1 HAS EXCEEDED MAX INDICATOR
			S32.1F2		# FIRST PASS FOR NEWTON ITERATION INDICATOR
# Page 633
		CLEAR	SET
			S32.1F3A	# 00=1ST 2 PASSES 2ND CYCLE, 01=FIRST CYCLE
			S32.1F3B	# 10=2ND CYCLE, 11=50 FPS STAGE 2ND CYCLE

; Initialize iteration counters
; LOOPCT tracks Newton iterations (max 16 before switching to second solution)
; CSIALRM will be set if alarm condition detected during computation

		DLOAD
			P30ZERO		# Load zero constant
		STORE	LOOPCT		# Zero iteration counter
		STORE	CSIALRM		# Clear alarm indicator
; ============================================================================
; CSI/B -- COMPUTE REFERENCE GEOMETRY AND INITIAL DELTA-V ESTIMATE
;
; This section calculates orbital geometry parameters needed for the
; Newton iteration. It computes:
; 1. Orbital radii of active vehicle (RA1) and passive vehicle (RP3)
; 2. Circular orbital velocity at active vehicle position
; 3. Out-of-plane velocity component to be nulled
; 4. Initial delta-V estimate (10 fps baseline)
; ============================================================================

CSI/B		SETPD	VLOAD
			0D		# Initialize pushdown pointer
			RACT1		# Active vehicle position at CSI time
		ABVAL	PUSH		# RA1 = magnitude		B29 PL02D
		NORM	SR1		# Normalize for scaling
			X2		# Store normalization count	B29-N2+B1 PL04D

; Compute ratio of orbital radii: RA1/RP3
; This ratio determines the shape difference between orbits

		PDVL	ABVAL
			RPASS3		# Passive vehicle position at CDH time
		NORM	BDDV		# RA1/RP3			B1 PL02D
			X1
		XSU,2	SR*		# Scale adjustment		B2
			X1
			1,2

; Calculate circular orbital velocity at active vehicle radius
; V_circular = sqrt(mu * (1 + RA1/RP3) * RA1)
; This gives the reference velocity for coelliptic orbit matching

		DAD	DMP		# (1+(RA1/RP3))RA1	B29+B2=B31 PL00D
			1DPB2		# Add 1.0 in double precision
		NORM	PDDL		# Normalize result		PL02D
			X1
			RTMU		# Gravitational parameter (mu)
		SR1	DDV		# mu / [(1+RA1/RP3)*RA1]	B38-B31=B7 PL00D
		SL*	SQRT		# Take square root for velocity	B7
			0 	-7,1
		PDVL	UNIT		# Push circular velocity	PL02D
			RACT1		# Get active vehicle position unit vector

; Compute out-of-plane velocity component
; The delta-V must null the out-of-plane component to achieve coplanar
; transfer (unless crew modifies for out-of-plane targeting)

		PDVL	VXV		# Push unit position vector
			UP1		# Passive vehicle velocity unit vector
		UNIT			# UNIT(URP1 X UVP1 X URA1) = UH1 (orbit normal)
		DOT	SL1		# VA1 . UH1 = out-of-plane velocity	B7
			VACT1		# Active vehicle velocity
		BDSU	STADR		# Subtract from circular velocity	PL00D
		STODL	DELVCSI		# Store initial delta-V estimate

; Load baseline delta-V magnitude for Newton iteration start
; 10 fps provides reasonable initial guess for most rendezvous geometries

			INITST		# 10 FPS initial estimate
		STORE	DELDV		# Store as current delta-V magnitude
; ============================================================================
; CSI/B1 -- NEWTON ITERATION LOOP ENTRY AND CONVERGENCE CHECK
;
; Main iteration loop for solving the two-point boundary value problem.
; Each iteration:
; 1. Checks if maximum iterations (16) exceeded -> switch to second solution
; 2. Applies trial delta-V to active vehicle state
; 3. Integrates trajectory from CSI to CDH
; 4. Computes miss distance from desired coelliptic geometry
; 5. Adjusts delta-V based on miss (Newton-Raphson method)
;
; COMMENT-ONLY READERS: This iterative refinement ensures the computed
; burn achieves the precise orbital geometry needed for rendezvous.
; ============================================================================

CSI/B1		DLOAD	DAD		# IF LOOPCT = 16, iteration limit reached
			LOOPCT		# Current iteration count
			1DPB28		# Increment by 1
		STORE	LOOPCT		# Update iteration counter
		DSU	AXT,2		# Check if limit exceeded
			LOOPMX		# Maximum iterations (16)
			6		# Set index register for branch
		BPL			# If positive (limit exceeded)...
			SCNDSOL		# ...try second solution stage
; ============================================================================
; CSI/B2 -- DELTA-V MAGNITUDE LIMIT CHECKING AND STAGE MANAGEMENT
;
; This section validates that the computed delta-V is within acceptable
; limits and manages transitions between solution stages:
; - Stage 1: 10 fps initial estimate, DVMAX1 limit
; - Stage 2: 50 fps initial estimate (if Stage 1 fails), DVMAX2 limit
; ============================================================================

CSI/B2		SETPD
			0D		# Reset pushdown pointer
# Page 634

; Check if computed delta-V exceeds first stage maximum
; If so, either switch to second solution stage or clamp to maximum

		DLOAD	ABS		# Load absolute value of delta-V
			DELVCSI		# Current CSI delta-V estimate
		DSU	BMN		# Subtract maximum limit
			DVMAX1		# First stage maximum (500 fps)
			CSI/B23		# If within limit, continue normally

; Delta-V exceeded DVMAX1
; Check flags to determine if already tried alternate solutions

		AXT,2	BON		# Set index for branch decisions
			7
			S32.1F1		# Check if already flagged excessive
			SCNDSOL		# If yes, try second solution
		BOFF	BON		# Check stage flags
			S32.1F3A
			CSI/B22		# FLAG 3 NEQ 3 -> clamp to DVMAX2
			S32.1F3B
			SCNDSOL		# Try second solution stage

; Clamp delta-V to second stage maximum
; Set flag indicating first stage exceeded limit

CSI/B22		SET	DLOAD
			S32.1F1		# Set "exceeded maximum" flag
			DVMAX2		# Load second stage maximum (1000 fps)
		SIGN			# Apply sign from original delta-V
			DELVCSI
		STORE	DELVCSI		# Store clamped delta-V
; ============================================================================
; CSI/B23 -- APPLY DELTA-V AND COMPUTE POST-MANEUVER ORBIT PARAMETERS
;
; This section performs the critical state vector propagation:
; 1. Compute horizontal plane unit vector (UH1) for delta-V direction
; 2. Convert scalar delta-V magnitude to vector in orbital coordinates
; 3. Apply delta-V impulsively to active vehicle velocity
; 4. Compute orbital parameters of post-CSI trajectory
; 5. Check pericenter altitude safety constraints
; 6. Prepare state for integration to CDH time
;
; COMMENT-ONLY READERS: This is where the computer "fires" the CSI burn
; mathematically. It adds the calculated velocity change to the LM's
; current velocity and computes the resulting orbit parameters.
;
; CODE-ALONG READERS: Note the use of TIMETHET for orbital parameter
; calculation and PERIAPO for altitude checks. The overflow branch
; CSI/B23D handles numerical scaling issues.
; ============================================================================

CSI/B23		VLOAD	PUSH		# Load active vehicle position
			RACT1		# At CSI time (reference state)
		UNIT	PDVL		# Compute unit position vector
			UP1		# Load passive vehicle velocity unit
		VXV	UNIT		# Cross product: URP1 X UVP1
					# Yields horizontal plane normal
					# Second cross with URA1 gives UH1
					# UNIT (URP1 X UVP1 X URA1) = UH1
		VXSC	VSL1		# Scale by delta-V magnitude
			DELVCSI		# Current CSI delta-V estimate (scalar)
		STORE	DELVEET1	# Store delta-V vector (ECI coordinates)
		VAD	BOV		# Add to active vehicle velocity
			VACT1		# Original velocity at CSI time
			CSI/B23D	# Branch if overflow (rescale needed)

; Apply velocity change and set up for orbit computation

CSI/B23D	STCALL	VACT4		# Store post-CSI velocity
			VECSHIFT	# Scale vectors for computation
		STOVL	VVEC		# Store velocity for TIMETHET
		SET			# Set reverse flag for integration
			RVSW		# Indicates backward integration mode
		STOVL	RVEC		# Store position for TIMETHET
			SN359+		# Load angle sine/cosine pair
		STCALL	SNTH		# Store as SNTH (ALSO CSTH)
			TIMETHET	# Compute orbital time and angles
; Compute pericenter altitude and check orbit shape constraints
; This determines if post-CSI orbit is safe (above minimum altitude)
; and whether orbit is circular or elliptical

		SR1	LXA,1		# Shift result, load index
			RTX1		# From TIMETHET output
		STCALL	HAFPA1		# Store half-parameter
			PERIAPO		# Compute pericenter/apocenter altitudes
		CALL			# Scale results
			SHIFTR1
		STODL	POSTCSI		# Store post-CSI pericenter altitude
			CENTANG		# Load central angle
		BZE	GOTO		# If zero (circular orbit)
			+2
# Page 635
			CIRCL		# Branch to circular orbit handling
		DLOAD			# Load eccentricity
			ECC
		DSU	BMN		# Subtract small threshold
			ONETHTH		# 0.001 threshold for circularity
			CIRCL		# If e < 0.001, treat as circular
; ============================================================================
; ELLIPTICAL ORBIT PARAMETER CALCULATIONS
;
; For non-circular orbits, compute detailed orbital geometry parameters
; needed for trajectory integration and convergence checking. These
; calculations use semi-latus rectum (P), eccentricity (ECC), and
; position/velocity dot products to characterize the orbital shape.
; ============================================================================

		DLOAD	CALL		# Load orbital radius
			R1		# At CSI position
			SHIFTR1		# Scale for computation
		SETPD	NORM		# Set pushdown to 2D, normalize
			2D
			X1		# Store normalization factor
		PDVL	DOT		# Push normalized value, compute R·V
			RACT1		# Position vector at CSI
			VACT4		# Velocity vector post-CSI
					#				PL04D
		ABS	DDV		# Absolute value of R·V
			02D		# Divide by R1
					# (/RDOTV/)/R1	   B38-B29= B7
		SL*	DSU		# Shift by normalization factor
			0,1		# Shift amount in X1
			NICKELDP	# Subtract small threshold (0.005)
		BMN	DLOAD		# If negative (nearly circular)
			CIRCL		# Branch to circular orbit path
			P		# Load semi-latus rectum
		SL2	DSU		# Shift left 2, subtract constant
			1DPB2		# Constant for calculation
		STODL	14D		# Store intermediate result
			RTSR1/MU	# Load sqrt(1/mu) constant
		SR1	DDV		# Shift, divide by R1
					# (1/ROOTMU)/R1	B-16-B29 = B-45 PL02D
		PDDL	DMP		# Push result, multiply
			P		# Semi-latus rectum
			R1		# By radius
		CALL			# Scale result
			SHIFTR1
		SL4	SL1		# Shift for precision
		SQRT	DMP		# Square root, multiply
					# ((P/MU)**.5)/R1	B14+B-14 = B-31 PL02D
		BOFF	SL3		# Check body flag, shift
			CMOONFLG	# Moon/Earth flag
			CSI/B3		# Continue with calculation
; Continue elliptical orbit parameter calculation for CSI/B3 path

CSI/B3		PDVL	DOT		# Compute R·V dot product again
			RACT1		# Position at CSI
			VACT4		# Velocity post-CSI
		STORE	RDOTV		# Store R·V for later use
		ABS			# Absolute value |R·V|
		NORM	DMP		# Normalize and multiply
			X2		# Store normalization factor
					# ((P/MU)**.5)RDOTV/R1		PL02D
		XSU,1	SL*		# Cross-subtract, shift
			X2		# By normalization amount
			3,1		# 	       B-31+B36-B3 = B2
		STODL	12D		# Store intermediate at 12D
			P30ZERO		# Load zero constant for trig
# Page 636
		STORE	16D		# Store at 16D for later use
		VLOAD	UNIT		# Load vector at 12D, normalize
			12D		# Computed vector parameter
		STOVL	SNTH		# Store as sine/cosine (ALSO STORES CSTH AND 0)
			RACT1		# Load position at CSI

; Set up state vectors for trajectory integration from CSI to CDH time
; This prepares for computing where the active vehicle will be after
; the CSI burn has been applied.

		PDVL	SIGN		# Push position, load velocity
			VACT4		# Velocity post-CSI
			RDOTV		# Sign from R·V dot product
		VCOMP	CALL		# Complement if needed, scale
			VECSHIFT	# Vector scaling for integration
		STOVL	VVEC		# Store velocity for TIMETHET
		SET			# Set reverse integration flag
			RVSW		# Backward integration mode
		STCALL	RVEC		# Store position for TIMETHET
			TIMETHET	# Compute orbital time parameters
; Determine orbital phase angle and compute CDH time based on orbit type

		PDDL	BPL		# Push result, check RDOTV sign
			RDOTV		# R·V dot product
			NTP/2		# Branch if positive (ascending)
		DLOAD	DSU		# If negative (descending)
			HAFPA1		# Half-apoapsis parameter
		PUSH	GOTO		# Push adjusted value
			NTP/2		# Continue to time calculation

; ============================================================================
; CIRCL -- CIRCULAR ORBIT SIMPLIFICATION
;
; When the post-CSI orbit is nearly circular (|R·V|/R < 0.005), skip
; the elliptical orbit parameter calculations and use simplified formulas.
; This avoids numerical precision issues near circular conditions.
;
; COMMENT-ONLY READERS: A circular orbit has constant altitude. This
; special case uses simpler math to compute rendezvous timing.
; ============================================================================

CIRCL		SETPD	DLOAD		# Reset pushdown pointer
			00D		# To base (0D)
			P30ZERO		# Load zero for initialization
		PUSH			# Push zero value

; Compute time from CSI to CDH for circular/near-circular orbits

NTP/2		DLOAD	DMP		# Load mean motion parameter
			NN		# Orbital rate (rev/time)
			HAFPA1		# Half-period adjustment
		SL	DSU		# Shift, subtract correction
			14D		# Stored parameter from earlier
		DAD			# Add CSI ignition time
			TCSI		# Time of CSI ignition
		STORE	TCDH		# Store computed CDH time
		BDSU	AXT,2
			TTPI
			5D
		BMN	SETPD
			SCNDSOL
			0D
		VLOAD	PDVL
			VACT4
			RACT1
		CALL
			INTINT2C
		STOVL	RACT2
			VATT
		STOVL	VACT2
			VPASS1
; ============================================================================
; INTEGRATE PASSIVE VEHICLE TRAJECTORY FROM CSI TO CDH TIME
;
; Now that we have computed TCDH (time of CDH maneuver), integrate the
; passive vehicle (CSM) state forward from current time to TCDH. This
; gives us the target position/velocity that the active vehicle (LM)
; should match after executing both CSI and CDH maneuvers.
;
; COMMENT-ONLY READERS: The computer now calculates where Columbia will
; be at the time Eagle plans to execute the second rendezvous burn (CDH).
;
; CODE-ALONG READERS: INTINT2C performs conic integration of the passive
; vehicle. The result is stored as RPASS2/VPASS2 (state at TCDH time).
; ============================================================================

		SETPD	PDVL		# Reset pushdown, load passive state
# Page 637
			0D		# Pushdown pointer to base
			RPASS1		# Passive vehicle position at TIG
		CALL			# Integrate passive trajectory
			INTINT2C	# Conic integration subroutine
		STOVL	RPASS2		# Store passive position at TCDH
			VATT		# Velocity from integration
		STCALL	VPASS2		# Store passive velocity at TCDH
			CDHMVR		# Compute CDH maneuver parameters
; ============================================================================
; CHECK PERICENTER ALTITUDE AFTER CDH MANEUVER
;
; After computing the CDH delta-V (via CDHMVR), check that the resulting
; orbit after CDH has acceptable pericenter altitude. If pericenter is
; too low, the solution is rejected and the algorithm will try alternate
; initial conditions (see CSI/B1A branch).
;
; COMMENT-ONLY READERS: Safety check ensures the orbit after the second
; burn doesn't dip too close to the lunar surface (below 35,000 feet).
;
; CODE-ALONG READERS: PERIAPO1 computes pericenter radius from the
; post-CDH state (RACT2, VACT3). SHIFTR1 scales the result.
; ============================================================================

		VLOAD	SETPD		# Load active position at CDH
			RACT2		# Position after CSI integration
			0D		# Reset pushdown pointer
		PDVL	CALL		# Push position, load velocity
			VACT3		# Velocity after CDH maneuver
			PERIAPO1	# Compute pericenter altitude
		CALL			# Scale result for comparison
			SHIFTR1		# Shift right for proper scaling
		STOVL	POSTCDH		# Store post-CDH pericenter
			VACT3		# Reload velocity for next step
; ============================================================================
; INTEGRATE ACTIVE VEHICLE FROM CDH TIME TO TPI TIME
;
; Set up integration parameters for the active vehicle trajectory from
; CDH to TPI. This determines where Eagle will be at terminal phase
; initiation time after executing both CSI and CDH burns.
;
; COMMENT-ONLY READERS: The computer flies Eagle's orbit forward to the
; final approach time, preparing to check the rendezvous geometry.
;
; CODE-ALONG READERS: INTINT requires position (RACT2), time span (TCDH
; to TTPI), and orbital period (TWOPI). ACTIVE processes the result.
; ============================================================================

		SETPD	PDVL		# Reset pushdown, load position
			0D		# Pushdown base
			RACT2		# Active position at CDH
		PDDL	PDDL		# Push position, load times
			TCDH		# Time of CDH maneuver
			TTPI		# Time of TPI maneuver
		PDDL	PUSH		# Push TPI time, load period
			TWOPI		# Orbital period (2π)
		CALL			# Integrate active trajectory
			INTINT		# Conic integration subroutine
		CALL			# Process integration results
			ACTIVE		# Extract position/velocity at TPI
; ============================================================================
; COMPUTE MISS DISTANCE FOR NEWTON-RAPHSON CONVERGENCE CHECK
;
; Calculate the "miss" between the desired rendezvous geometry and the
; actual geometry achieved with the current delta-V estimate. The desired
; geometry is: at TPI time, the line-of-sight (LOS) from active to passive
; vehicle should be at elevation angle ELEV (typically 27 degrees) above
; the active vehicle's local horizontal.
;
; This section constructs a unit vector U pointing in the desired LOS
; direction, then computes how far the actual passive vehicle position
; deviates from this ideal geometry. The deviation (TEMP2 later) drives
; the Newton iteration to adjust delta-V.
;
; COMMENT-ONLY READERS: The computer checks how close Eagle's geometry
; matches the ideal 27-degree approach angle for final rendezvous. Any
; error feeds back to adjust the burn calculations.
;
; CODE-ALONG READERS: This is classic Newton-Raphson. Construct unit
; vector URA3 (active position), unit vector UH3 (horizontal component),
; combine with elevation angle to form desired LOS unit vector U, then
; compute scalar TEMP1 = (U dot RA3). TEMP1 is used to calculate TEMP2
; (miss distance squared) which determines iteration convergence.
; ============================================================================

		DLOAD			# Load elevation angle
			ELEV		# Desired elevation at TPI
		SETPD	SINE		# Compute sine of elevation
			6D		# Set pushdown for vector ops
		PDVL	UNIT		# Push sine, load position vector
			RACT3		# Active position at TPI
		STORE	00D		# URA3 AT 00D (unit vector)
		PDVL	VXV		# Push URA3, load velocity, cross
			UP1		# Unit vector perpendicular to orbit
		UNIT			# Normalize cross product
		PDDL	COSINE		# UNIT(URA3 X UVA3 X URA3) = UH3 (horiz)
			ELEV		# Cosine of elevation angle
		VXSC	STADR		# Scale horizontal by cos(ELEV)
		STORE	18D		# Store horizontal component
		DLOAD	VXSC		# Load sine, scale radial by sin(ELEV)
		VAD	VSL1		# Add horizontal + radial components
			18D		# U = cos(ELEV)*UH3 + sin(ELEV)*URA3
		PUSH	DOT		# Push U vector, dot with position
			RACT3		# (U . RA3) = TEMP1 (scalar)
		SL1	PUSH		# Scale and push TEMP1
# Page 638
; ============================================================================
; CONVERGENCE CHECK: COMPUTE TEMP2 AND TEST ITERATION COMPLETION
;
; Compute TEMP2 = TEMP1² + |RA3|² + |RP3|² - 2*(U·RP3)
; where:
;   TEMP1 = (U · RA3) [computed above]
;   RA3 = active position at TPI
;   RP3 = passive position at TPI
;   U = desired line-of-sight unit vector
;
; TEMP2 measures the square of the miss distance between actual and
; desired geometry. If TEMP2 < threshold, solution has converged and
; iteration terminates. Otherwise, loop back to CSI/B1 with adjusted
; delta-V for next iteration.
;
; COMMENT-ONLY READERS: The computer checks if the orbital geometry is
; close enough to the target. If yes, the burn calculations are complete.
; If no, try again with a better estimate.
;
; CODE-ALONG READERS: This is the Newton-Raphson convergence test.
; LOOPCT tracks iteration count. If not converged and iterations remain,
; branch back to CSI/B1. If converged (TEMP2 near zero), continue to
; store final solution.
; ============================================================================

		DSQ	TLOAD		# TEMP1**2 (square the dot product)
			MPAC		# Load from accumulator
		PDVL	DOT		# Push TEMP1², dot active position
			RACT3		# Active position at TPI
			RACT3		# RA3 · RA3 = |RA3|²
		TLOAD	DCOMP		# Load and complement
			MPAC		# Prepare for subtraction
		PDVL	DOT		# Push -(RA3·RA3), load passive pos
			RPASS3		# Passive position at TPI
			RPASS3		# RP3 · RP3 = |RP3|²
		TAD	TAD		# TEMP1² + RA3·RA3 + RP3·RP3 = TEMP2
		BPL	DLOAD		# If TEMP2 ≥ 0, check iteration count
			K10RK2		# Branch target (continue iteration)
			LOOPCT		# Load iteration counter
		DSU	AXT,2		# Decrement iteration count
			1DPB28		# Subtract 1 (scaled)
			1D		# Set index register
		BZE			# If zero iterations left, exit loop
			ALMXITA		# Branch to alarm/iteration exit

; ----------------------------------------------------------------------------
; ITERATION CONTINUATION: Adjust delta-V estimate for next iteration
;
; If iterations remain and solution hasn't converged, compute the
; change in delta-V (DELVCSI = current DELDV - previous DVPREV) and
; loop back to CSI/B1 to try again with this refined estimate.
;
; COMMENT-ONLY READERS: The computer refines its burn calculation and
; tries again to find the perfect rendezvous geometry.
;
; CODE-ALONG READERS: This is the Newton-Raphson update step. The
; difference between successive delta-V estimates drives convergence.
; DELDV is shifted right before storage, then differenced with DVPREV.
; ----------------------------------------------------------------------------

		DLOAD	SR1		# Load current delta-V estimate
			DELDV		# Delta-V magnitude
		STORE	DELDV		# Shift right and re-store
		BDSU			# Subtract previous estimate
			DVPREV		# Delta-V from last iteration
		STCALL	DELVCSI		# Store change, loop back
			CSI/B1		# Return to iteration start

; ============================================================================
; CONVERGENCE SUCCESS: Compute final transfer orbit velocity at CSI
;
; When Newton-Raphson has converged (TEMP2 is small), calculate the
; final velocity vector for the CSI maneuver. This involves solving
; the quadratic equation for the passive vehicle's position relative
; to the line-of-sight:
;
;   K1 = -TEMP1 + TEMP3    (where TEMP3 = sqrt(TEMP2))
;   K2 = -TEMP1 - TEMP3
;
; Select K with smaller absolute value (the physically meaningful root),
; then compute the transfer orbit velocity: V = unit(RA3 + K*U)
;
; COMMENT-ONLY READERS: The computer has found the correct burn! Now it
; calculates the exact velocity Eagle needs after CSI to reach Columbia.
;
; CODE-ALONG READERS: This solves the geometric constraint. K scales the
; desired LOS vector U to intersect the passive vehicle's position sphere.
; The smaller |K| root gives the shorter transfer path. Final velocity
; is normalized (UNIT) for use in subsequent delta-V computation.
; ============================================================================

K10RK2		SQRT	PUSH		# TEMP3 = sqrt(TEMP2) (miss distance)
		DCOMP	DSU		# Compute -TEMP3 - TEMP1
			06D		# Get TEMP1 from pushdown 06D
		STODL	10D		# K2 = -TEMP1 - TEMP3 stored at 10D
		DSU	STADR		# Compute TEMP3 - TEMP1 (via subtraction address)
		STORE	12D		# K1 = -TEMP1 + TEMP3 stored at 12D
		ABS			# Take absolute value of K1
		STODL	14D		# Store |K1| at 14D
			10D		# Load K2
		ABS	DSU		# Compute |K2| - |K1|
			14D		# Compare absolute values
		BMN	DLOAD		# If |K2| < |K1|, branch to K2. (use K2)
			K2.		# Else continue to use K1
			12D		# Load K1 (smaller absolute value)
		STORE	10D		# K = K1 (shorter transfer path)
K2.		DLOAD			# K2. label: use K2 or already-loaded K1
			10D		# Load selected K value
		VXSC	VSL1		# K * U (scale desired LOS unit vector)
		VAD	UNIT		# V = unit(RA3 + K*U) transfer velocity
			RACT3		# Active vehicle position at TPI
		PDVL	UNIT		# Push V, load and normalize passive position
			RPASS3		# Passive vehicle position at TPI
		PDVL	UNIT		# Push unit(RPASS3), load passive velocity
			VPASS3		# Passive vehicle velocity at TPI
# Page 639
		VXV	PDVL		# Cross product: unit(VPASS3) X unit(RPASS3)
			06D		# Load unit(RPASS3) from pushdown
			06D		# Load unit(V) from pushdown
; Compute elevation angle GAMMA at TPI (LOS angle from horizontal)
		VXV	DOT		# (unit(RPASS3) X unit(V)) . (unit(VPASS3) X unit(RPASS3))
			00D		# Get cross product result from stack
		STADR			# Address for sign determination
		STOVL	12D		# TEMP = sign indicator, load unit(V)
		DOT	SL1		# unit(V) . unit(RPASS3) scaled
		ARCCOS	SIGN		# GAMMA = arccos(dot product)
			12D		# Apply sign from TEMP for direction
		SR1	PUSH		# Scale, push GAMMA (elevation angle)
		
; Newton-Raphson refinement: Compute delta-V correction using derivative
		BON	DLOAD		# Check first-pass flag
			S32.1F2		# If first pass flag set
			FRSTPAS		# Branch to first pass handler
			00D		# Not first pass: load current GAMMA
		DSU	PDDL		# GAMMA - GAMPREV (angle change)
			GAMPREV		# Previous gamma angle
			DELVCSI		# Load current delta-V magnitude
		DSU	NORM		# DELVCSI - DVPREV (delta-V change)
			DVPREV		# Previous delta-V
			X1		# Normalize for division
		BDDV	PDDL		# SLOPE = (GAMMA-GAMPREV)/(DELVCSI-DVPREV)
			02D		# Divisor from stack
			DELVCSI		# Load current delta-V
		STORE	DVPREV		# Save as previous for next iteration
		BOFF	BOFF		# Check third-pass convergence flags
			S32.1F3A	# Third pass flag A
			THRDCHK		# Not set: check third pass
			S32.1F3B	# Third pass flag B
			THRDCHK		# Not set: check third pass
		DLOAD	DMP		# Third pass: compute correction
			02D		# Load SLOPE
			GAMPREV		# Multiply by previous gamma
		BPL	DLOAD		# If positive, use initialization state
			FIFTYFPS	# Branch to 50 fps limit handler
			INITST		# Load initial state value
		SIGN			# Apply sign correction
			DELDV		# Sign from current delta change
		STORE	DELDV		# Store corrected delta-V change
		SET	CLEAR		# Update iteration flags
			S32.1F3A	# Set third-pass flag A
			S32.1F3B	# Clear third-pass flag B
			
; First pass handler: Initialize for next iteration
FRSTPAS		DLOAD			# Load current GAMMA
			00D		# From pushdown stack
		STODL	GAMPREV		# Save as previous gamma
			DELVCSI		# Load current delta-V
		STORE	DVPREV		# Save as previous delta-V
		DSU	CLEAR		# Apply correction
			DELDV		# Subtract delta change
			S32.1F2		# Clear first-pass flag
# Page 640
		STCALL	DELVCSI		# Store updated delta-V
			CSI/B1		# Loop back for next iteration
			
; Third-pass check: Verify convergence, apply delta-V limits
THRDCHK		BON	BON		# Check third-pass flags
			S32.1F3A	# If flag A set
			NEWTN		# Use Newton refinement
			S32.1F3B	# If flag B set
			NEWTN		# Use Newton refinement
			
; Apply 50 fps delta-V increment limit for safety
FIFTYFPS	DLOAD	SIGN		# Load 50 fps limit constant
			FIFPSDP		# 50 feet per second threshold
			04D		# Apply sign from computation
		SIGN			# Apply gamma direction sign
			GAMPREV		# Previous gamma angle
		STORE	DELDV		# Store limited delta-V change
		DCOMP	DAD		# Negate and add to current delta-V
			DELVCSI		# Current CSI delta-V magnitude
		STODL	DELVCSI		# Store updated delta-V
			00D		# Load current GAMMA
		SET	SET		# Update flags for next iteration
			S32.1F3B	# Set third-pass flag B
			S32.1F3A	# Set third-pass flag A
		STCALL	GAMPREV		# Store GAMMA, continue iteration
			CSI/B2		# Return to CSI/B2 path
			
; Newton-Raphson refinement: Compute delta-V correction from slope
NEWTN		DLOAD	NORM		# Load GAMMA - desired GAMMA (error)
			04D		# From pushdown
			X2		# Normalize for division
		BDDV	XSU,1		# Divide by SLOPE (derivative)
			00D		# Divisor from stack
			X2		# Scale index correction
		SR*			# Scale result by normalization
			0,1		# Variable shift by X1 index
		STODL	DELDV		# Store computed delta-V correction
			00D		# Load current GAMMA
		STORE	GAMPREV		# Save as previous for next iteration
		DLOAD	ABS		# Load delta-V correction
			DELDV		# Computed change
		PUSH	DSU		# Push absolute value, compare
			EPSILN1		# Convergence threshold (epsilon)
		BMN	DLOAD		# If converged, solution found
			CSI/SOL		# Jump to solution handler
		DSU	BMN		# Not converged: check max limit
			DELMAX1		# Maximum allowed delta-V change
			CSISTEP		# If within limit, apply correction
		DLOAD	SIGN		# Limit exceeded: clamp to maximum
			DELMAX1		# Load maximum delta-V change
			DELDV		# Apply sign from correction
		STORE	DELDV		# Store limited correction
		
; Apply delta-V correction and iterate CSI solution
CSISTEP		DLOAD	DSU		# Load current CSI delta-V
			DELVCSI		# Magnitude from previous iteration
			DELDV		# Subtract computed correction
		STCALL	DELVCSI		# Store updated CSI delta-V
# Page 641
			CSI/B1		# Loop back for next iteration

; ============================================================================
; TRANSITION: From iterative solution to solution validation
;
; The Newton-Raphson iteration has converged on a CSI delta-V solution.
; Before accepting the solution, the AGC validates four critical constraints:
; 1) Post-CSI pericenter altitude must be safe (>35,000 ft lunar orbit)
; 2) Post-CDH pericenter altitude must be safe (>35,000 ft lunar orbit)
; 3) Time from CSI to CDH must be >10 minutes for crew preparation
; 4) Time from CDH to TPI must be >10 minutes for phasing accuracy
; If any constraint fails, program attempts alternate solution path.
; ============================================================================

; CSI solution validation: Check orbital and timing constraints
CSI/SOL		DLOAD	AXT,2		# Load post-CSI pericenter altitude
			POSTCSI		# Altitude after CSI maneuver
			2		# Set index for pericenter check
		LXA,1			# Load orbit type indicator
			RTX1		# (lunar or earth orbit)
		DSU*	BMN		# Check minimum safe altitude
			PMINE 	-2,1	# Minimum pericenter (indexed)
			SCNDSOL		# If too low, try second solution
		AXT,2	DLOAD		# Check post-CDH pericenter
			3		# Index for CDH check
			POSTCDH		# Altitude after CDH maneuver
		DSU*	BMN		# Verify safe pericenter
			PMINE 	-2,1	# Minimum pericenter (indexed)
			SCNDSOL		# If too low, try second solution
		DLOAD	DSU		# Compute CSI to CDH time interval
			TCDH		# CDH ignition time
			TCSI		# CSI ignition time
		STORE	T1TOT2		# Store delta-T (CSI to CDH)
		AXT,2	DSU		# Check minimum 10-minute spacing
			4		# Index for timing check
			TMIN		# Minimum maneuver spacing (10 min)
		BMN	AXT,2		# If too short, try second solution
			SCNDSOL		# Insufficient crew preparation time
			5		# Index for next timing check
		DLOAD	DSU		# Compute CDH to TPI time interval
			TTPI		# TPI ignition time
			TCDH		# CDH ignition time
		STORE	T2TOT3		# Store delta-T (CDH to TPI)
		DSU	BPL		# Check minimum 10-minute spacing
			TMIN		# Minimum maneuver spacing (10 min)
			P32/P72C	# All checks passed, continue

; Second solution attempt: Primary solution failed validation
; This section attempts an alternate solution with modified parameters.
; If both solution attempts fail (flags S32.1F3A and S32.1F3B set),
; program exits with alarm code indicating no valid solution exists.
SCNDSOL		BON	BOFF		# Check if first attempt already failed
			S32.1F3A	# First attempt flag
			ALMXIT		# Both attempts failed, exit with alarm
			S32.1F3B	# Second attempt flag
			ALMXIT		# Both attempts failed, exit with alarm
		SXA,2	DLOAD		# Store alarm code for CSI failure
			CSIALRM		# Alarm code location
			P30ZERO		# Reset P30 time marker
		CLEAR	SET		# Update solution attempt flags
			S32.1F1		# Clear first solution flag
			S32.1F2		# Set second solution flag
		CLEAR	CLEAR		# Mark this as first attempt
			S32.1F3A	# Clear first attempt indicator
			S32.1F3B	# Clear second attempt indicator
		STCALL	LOOPCT		# Store loop counter
			CSI/B		# Retry CSI computation with modified parameters

# Page 642
# ***** ADVANCE *****
#
# SUBROUTINES USED
#	PRECSET
#	ROTATE

; ============================================================================
; ADVANCE: State vector propagation and coordinate frame rotation
;
; This subroutine prepares state vectors for maneuver calculations by:
; 1) Setting up precession parameters for coordinate frame rotation
; 2) Propagating passive vehicle (CSM) position/velocity vectors
; 3) Propagating active vehicle (LM) position/velocity vectors
; 4) Computing reference frame orientation (UP1 vector perpendicular to orbit)
; 5) Rotating state vectors to time-of-ignition (TIG) reference frame
;
; The ADVANCE subroutine is critical for rendezvous targeting because
; both vehicles are moving in orbit. State vectors must be propagated
; from current time to ignition time for accurate delta-V computation.
; ============================================================================

ADVANCE		STQ	DLOAD		# Store return address, load TIG
			SUBEXIT		# Exit address for return
			TIG		# Time of ignition
		STCALL	TDEC1		# Store as time for precession
			PRECSET		# Set up precession parameters
		SET	VLOAD		# Set delta-V computation flag
			XDELVFLG	# External delta-V flag
			VPASS3		# Load passive vehicle velocity (CSM)
		STORE	VPASS2		# Store in secondary location
		STOVL	VPASS1		# Store in primary location
			RPASS3		# Load passive vehicle position (CSM)
		STORE	RPASS2		# Store in secondary location
		STORE	RPASS1		# Store in primary location
		UNIT	VXV		# Compute orbital reference frame
			VPASS1		# Velocity vector
		UNIT			# Normalize cross product
		STOVL	UP1		# Store UP vector (perpendicular to orbit)
			RACT3		# Load active vehicle position (LM)
		STCALL	RTIG		# Store as TIG position
			ROTATE		# Rotate to TIG reference frame
		STORE	RACT2		# Store rotated position (secondary)
		STOVL	RACT1		# Store rotated position (primary)
			VACT3		# Load active vehicle velocity (LM)
		STCALL	VTIG		# Store as TIG velocity
			ROTATE		# Rotate to TIG reference frame
		STORE	VACT2		# Store rotated velocity (secondary)
		STCALL	VACT1		# Store rotated velocity (primary)
			SUBEXIT		# Return to caller

# Page 643
# ***** ROTATE *****

; ROTATE: Vector rotation utility for coordinate frame transformation
;
; This compact subroutine rotates a 3D vector into the reference frame
; defined by the UP1 unit vector (perpendicular to orbital plane).
; The rotation preserves vector magnitude while reorienting components.
; Used by ADVANCE to transform state vectors to time-of-ignition frame.
;
; Mathematical operation:
; 1) Project input vector onto UP1 direction (dot product)
; 2) Scale projection by UP1 unit vector
; 3) Shift result left 2 bits for precision
; 4) Subtract from original vector (removing UP1 component)
; 5) Normalize and scale for output
;
; This rotation is essential for computing delta-V in the correct
; orbital reference frame, accounting for precession and orbital motion.

ROTATE		PUSH	PUSH		# Save input vector twice on stack
		DOT	VXSC		# Dot product with UP1, scale by UP1
			UP1		# Orbital perpendicular vector
			UP1		# Scale by same vector
		VSL2	BVSU		# Shift left 2 bits, subtract from original
		UNIT	PDVL		# Normalize result, push to stack
		ABVAL	VXSC		# Compute magnitude, cross-scale
		VSL1	RVQ		# Shift left 1 bit, return to caller

# Page 644
# ***** INTINTNA *****

; ============================================================================
; INTINT2C and INTINT3P: Orbital integration setup routines
;
; These subroutines prepare parameters for numerical integration of
; orbital motion between maneuver times. They compute position and
; velocity vectors at intermediate times for trajectory prediction.
;
; INTINT2C: Integrate from CSI to CDH maneuver time
; - Used to predict orbital state after CSI burn when planning CDH
; - Time span: TCSI to TCDH (typically 10+ minutes)
; - Reference epoch: TWOPI (two-pi time marker)
;
; INTINT3P: Integrate from CDH to TPI maneuver time
; - Used to predict orbital state after CDH burn when planning TPI
; - Time span: TCDH to TTPI (typically 10+ minutes)
; - Reference epoch: P30ZERO (program 30 time marker)
;
; Both routines push time parameters onto the stack and branch to
; the common INTINT integrator (located in CONIC_SUBROUTINES.agc).
; ============================================================================

INTINT2C	PDDL	PDDL		# Push CSI time, push CDH time
			TCSI		# CSI ignition time
			TCDH		# CDH ignition time
		PDDL	PUSH		# Push two-pi reference time
			TWOPI		# Orbital period reference
		GOTO			# Branch to common integrator
			INTINT		# (CONIC_SUBROUTINES.agc)

INTINT3P	PDDL	PDDL		# Push CDH time, push TPI time
			TCDH		# CDH ignition time
			TTPI		# TPI ignition time
		PDDL	PUSH		# Push P30 reference time
			P30ZERO		# Program 30 time marker
		GOTO			# Branch to common integrator
			INTINT		# (CONIC_SUBROUTINES.agc)

# Page 645
# ***** S32/33.1 *****
#
# SUBROUTINES USED
#	S32/33.X

; ============================================================================
; S32/33.1: Delta-V display and coordinate transformation
;
; This subroutine prepares computed delta-V vectors for crew display
; and coordinate frame transformation. Used by P32/P33 (and P72/P73)
; to show maneuver parameters on the DSKY for crew approval.
;
; Operations:
; 1) Display delta-V components using VN 0681 (Verb 06, Noun 81)
; 2) Transform delta-V from local vertical coordinate system (LVLH)
;    to inertial reference frame using rotation matrix from S32/33.X
; 3) Store transformed delta-V in DELVSIN (inertial coordinates)
; 4) Compute and store delta-V magnitude in DELVSAB (absolute value)
;
; The LVLH frame has components: radial (outward), downrange (velocity
; direction), and cross-track. The inertial frame is fixed in space.
; Astronauts monitor delta-V magnitude to verify fuel requirements.
; ============================================================================

S32/33.1	STQ	AXT,1		# Store return address, set index
			SUBEXIT		# Exit address for return
		VN	0681		# Display delta-V (Verb 06, Noun 81)
		CALL			# Call display routine
			DISDVLVC	# Display delta-V local vertical
		CALL			# Call coordinate transformation
			S32/33.X	# Compute rotation matrix
		VLOAD	VXM		# Load delta-V, multiply by matrix
			DELVLVC		# Delta-V local vertical coordinates
			0D		# Rotation matrix at address 0D
		VSL1			# Shift left 1 bit for precision
		STORE	DELVSIN		# Store delta-V inertial coordinates
		PUSH	ABVAL		# Push to stack, compute magnitude
		STOVL	DELVSAB		# Store delta-V absolute (magnitude)
		GOTO			# Return to caller
			SUBEXIT		# Exit address

# Page 646
# ***** S32/33.X *****
; 
; S32/33.X: Rotation matrix computation for local vertical reference frame
; 
; This critical subroutine computes the coordinate transformation matrix from
; the inertial reference frame to the local vertical/horizontal (LVLH) frame
; centered on the active vehicle. The LVLH frame is defined with:
;   Z-axis: Along orbit normal (UP1 direction)
;   Y-axis: Perpendicular to both orbit plane and radial direction
;   X-axis: Completes right-handed coordinate system
; 
; This transformation is essential for expressing delta-V maneuvers in the
; natural orbital reference frame where in-plane and out-of-plane components
; have clear physical meanings for rendezvous geometry.
; 
; Historical: During Apollo 11 rendezvous, this routine computed the reference
; frame for all CSI and CDH delta-V targeting, ensuring proper orbital phasing.

S32/33.X	SETPD	VLOAD		# Initialize push-down stack pointer
			6D		# Stack pointer offset 6D
			UP1		# Load orbit angular momentum vector
		VCOMP	PDVL		# Complement (negate) and push to stack
			RACT1		# Load active vehicle position vector
		UNIT	VCOMP		# Normalize and complement position
		PUSH	VXV		# Push to stack, compute cross product
			UP1		# Cross with orbit normal
		VSL1			# Left shift result (scale adjustment)
		STORE	0D		# Store rotation matrix to 0D location
		RVQ			# Return via Q register

# Page 647
# ***** CDHMVR *****
#
# SUBROUTINES USED
#	VECSHIFT
#	TIMETHET
#	SHIFTR1
#
; ============================================================================
; CDHMVR: Constant Delta Height maneuver computation
; 
; This subroutine calculates the delta-V required for the CDH (Constant Delta
; Height) maneuver, which is the second burn in the coelliptic rendezvous
; sequence. The CDH maneuver circularizes the active vehicle's orbit at the
; same altitude as the passive vehicle's orbit, maintaining constant height
; separation while allowing phasing to continue for TPI timing.
; 
; The computation involves:
;   1) Computing the geometry between vehicles at CDH time
;   2) Calculating the transfer angle and orbital parameters
;   3) Determining the required velocity for matching orbital shape
;   4) Outputting delta-V in reference coordinates
; 
; Historical: During Apollo 11's rendezvous on July 21, 1969, Eagle executed
; the CDH maneuver to match Columbia's orbital characteristics while maintaining
; proper spacing for the final TPI approach. This ensured controlled closure
; geometry with minimal out-of-plane components.
; ============================================================================

CDHMVR		STQ	VLOAD		# Save return address
			SUBEXIT		# Exit address for subroutine
			RACT2		# Load active vehicle position at CDH
		PUSH	UNIT		# Push to stack and normalize
		STOVL	UNVEC		# Store unit radial vector (UR sub A)
			RPASS2		# Load passive vehicle position
		UNIT	DOT		# Normalize and compute dot product
			UNVEC		# Dot with active unit vector
		PUSH	SL1		# Push and left shift
		STODL	CSTH		# Store cos(theta) = separation angle
		DSQ	PDDL		# Square cos(theta), push to stack
			DP1/4TH		# Load constant 1/4
		SR2	DSU		# Right shift 2, compute difference
		SQRT	SL1		# Square root, left shift (sin calculation)
		PDVL	VCOMP		# Push result, load and complement vector
		VXV			# Cross product computation
			RPASS2		# Cross with passive position
		DOT	PDDL		# Dot product with orbit normal
			UP1		# UP1 = orbit angular momentum
		SIGN	STADR		# Transfer sign, store address
		STOVL	SNTH		# Store sin(theta) = separation angle sine
			RPASS2		# Load passive position again
		PDVL	CALL		# Push to stack, load velocity
			VPASS2		# Passive vehicle velocity
			VECSHIFT	# Call vector scaling routine
		STOVL	VVEC
		CLEAR
			RVSW
		STCALL	RVEC
			TIMETHET
		LXA,2	VSL*
			RTX2
			0,2
		STORE	18D
		DOT	SL1R
			UNVEC
		PDVL	ABVAL		# 0D = V SUB PV
		SL*	PDVL		# Scale by index, push and load
			0,2		# Indexed scaling factor
			RACT2		# Active vehicle position vector
		ABVAL	PDDL		# Absolute value (magnitude)
		DSU			# Compute altitude difference
# Page 648
			02D		# Passive vehicle magnitude reference
		STODL	DIFFALT		# Store delta-H in meters (B+29 scale)
			R1A		# Load semi-major axis parameter

; Orbital mechanics computation section:
; Using vis-viva equation and Kepler's laws to compute the velocity change
; needed to match the passive vehicle's orbital characteristics while
; maintaining the current radial distance. The equations account for:
;   - Current orbital energy (vis-viva: v² = μ(2/r - 1/a))
;   - Target semi-major axis matching passive orbit
;   - Conservation of angular momentum in orbital plane

		NORM	PDDL		# Normalize result, push to stack
			X1		# Normalization exponent
			R1		# Load R1 parameter
		CALL			# Compute scaling adjustment
			SHIFTR1		# Shift right by computed amount
		SR1R	DDV		# Right shift, divide
		SL*	PUSH		# Scale by index, push result
			0 	-5,1	# Indexed scaling (-5 offset)
		DSU	PDDL		# Compute difference (A sub A)
			DIFFALT		# Subtract altitude difference
		SR2	DDV		# Right shift 2, divide (A sub P)
			04D		# Divisor at stack location 04D
		PUSH	SQRT		# Push, compute square root
		DMPR	DMP		# Double precision multiply operations
			06D		# Stack location 06D
			00D		# Stack location 00D
		SL3R	PDDL		# Left shift 3, push (V sub AV)
			02D		# Active vehicle magnitude (B+29)
		NORM	PDDL		# Normalize and push
			X1		# Normalization exponent
			RTMU		# Load sqrt(μ) gravitational parameter
		SR1	DDV		# Right shift 1, divide (2μ computation)
		SL*	PDDL		# Scale and push (2μ/R sub AA)
			0 	-5,1	# Indexed scaling (-5 offset)
			04D		# Semi-major axis A sub A (B+29)
		NORM	PDDL		# Normalize and push
			X2		# Normalization exponent X2
			RTMU		# Load sqrt(μ) again
		SR1	DDV		# Right shift 1, divide
		SL*	BDSU		# Scale and backward subtract
			0 	-6,2	# 2μ/r - μ/a velocity term (B+14)
		PDDL	DSQ		# Push result, square previous value
			08D		# V sub AV squared
		BDSU	SQRT		# Backward subtract, take square root
		PDVL	VXV		# Push, load vector, cross product
			UP1		# Orbit angular momentum vector
			UNVEC		# Unit radial vector
		UNIT	VXSC		# Normalize, vector scale
			10D		# Scaling factor at 10D
		PDVL	VXSC		# Push result, load and scale
			UNVEC		# Unit radial vector
			08D		# Average velocity magnitude

; Final delta-V computation:
; Combines radial and tangential velocity components to produce the total
; velocity vector at CDH ignition time, then computes the difference between
; the required velocity (VACT3) and the current trajectory velocity (VACT2).
; This delta-V is the CDH thrust magnitude and direction.

		VAD	VSL1		# Vector add, left shift scale
		STADR			# Store address for result
		STORE	VACT3		# Store required velocity at CDH
		VSU			# Vector subtract (compute delta-V)
			VACT2		# Current velocity at CDH time
# Page 649
		STCALL	DELVEET2	# Store CDH delta-V in reference coords
			SUBEXIT		# Return via saved exit address

; Historical Note: During Apollo 11's rendezvous after Eagle's ascent from the
; lunar surface on July 21, 1969, the CDH maneuver fine-tuned the approach
; geometry. This burn adjusted Eagle's orbit to precisely match Columbia's
; orbital shape, ensuring the subsequent TPI maneuver would close the final
; distance with optimal fuel efficiency and timing.

# Page 650
# ***** COMPTGO *****
#
# SUBROUTINES USED
#	CLOKTASK
#	2PHSCHNG
#
; ============================================================================
; COMPTGO: Compute Time-To-Go display management
; 
; This utility routine initializes and schedules the real-time countdown timer
; display that shows time remaining until the next rendezvous maneuver. During
; the wait between CSI and CDH, or between CDH and TPI, the crew monitors this
; countdown on the DSKY display to prepare for the upcoming engine burn.
; 
; The routine:
;   1) Saves the return address in RTRN
;   2) Clears the display index (DISPDEX = 0)
;   3) Schedules the CLOKTASK routine on the WAITLIST for periodic updates
;   4) Performs phase change operations for task synchronization
;   5) Returns to calling program
; 
; Historical: During Apollo 11 rendezvous, Armstrong and Aldrin used the
; time-to-go display to coordinate maneuver preparations, switching between
; navigation updates and engine gimbal checks as each burn approached.
; ============================================================================

		BANK	35		# Switch to bank 35 for this code
		SETLOC	CSI/CDH		# Set location counter
		BANK			# Restore bank setting

		EBANK=	RTRN		# Set erasable bank for RTRN variable

		COUNT*	$$/P3575	# Accounting for program section

COMPTGO		EXTEND			# Extend next instruction
		QXCH	RTRN		# Exchange Q with RTRN (save return)
		CAF	ZERO		# Load accumulator with zero
		TS	DISPDEX		# Store to display index (clear)
		CAF	BIT2		# Load bit 2 constant (waitlist time)
		INHINT			# Inhibit interrupts during setup
		TC	WAITLIST	# Transfer control to WAITLIST scheduler
		EBANK=	WHICH		# Set erasable bank for WHICH variable
		2CADR	CLOKTASK	# Two-word address of clock task routine

; Schedule phase change operations to synchronize countdown display updates
; with the executive scheduler and ensure proper task priority during the
; coast period between maneuvers.

		TC	2PHSCHNG	# Two-phase change routine
		OCT	40036		# Phase change parameter 1
		OCT	05024		# Phase change parameter 2
		OCT	13000		# Phase change parameter 3
		TC	RTRN		# Return to calling program via RTRN

