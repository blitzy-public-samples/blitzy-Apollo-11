# Copyright:	Public domain.
# Filename:	P32-P33_P72-P73.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	649-683
# Mod history:	2009-05-09 RSB	Adapted from the Luminary131/ file
#				P32-P35_P72-P75.agc and Comanche055 page
#				images.
#		2009-05-20 RSB	Corrected CSI/COM3 -> CSI/CDH3,
#				CSI/CDHI -> CSI/CDH1, CDHTAB -> CDHTAG,
#				changed a SETLOC from CSI/CDH to CSI/CDH1,
#				a SETLOC CSI/CDH1 to CSIPROG.
#		2009-05-21 RSB	Changed a P32/P72D to P32/P72E in
#				P32/P72D.  DP1/4TH changed to DP1/4 in
#				CDHMVR.
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
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  10:28 APR. 1, 1969
#
#	This AGC program shall also be referred to as
#			Colossus 2A

; ============================================================================
; FILE: P32-P33_P72-P73.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: rendezvous
;
; TL;DR: Coelliptic (P32) and concentric (P33) rendezvous sequence programs
;        with abort variants (P72/P73). Implements height adjustment maneuvers
;        establishing matching orbital planes and altitudes for rendezvous.
;        P32 creates coelliptic orbit with target, P33 circularizes for
;        concentric approach.
;
; COMMENT-ONLY READERS: These programs adjusted the spacecraft's orbit to match
;        the target vehicle's altitude and timing for final approach.
; CODE-ALONG READERS: Study coelliptic/concentric orbital mechanics, phasing
;        maneuvers, and rendezvous geometry optimization.
; ============================================================================

# Page 649
# COELLIPTIC SEQUENCE INITIATION (CSI) PROGRAMS (P32 AND P72)

; ============================================================================
; RENDEZVOUS ORBITAL MECHANICS OVERVIEW
;
; The concentric rendezvous sequence uses two key maneuvers to match orbits:
;
; 1. COELLIPTIC STRATEGY (CSI - Coelliptic Sequence Initiation):
;    The chasing spacecraft performs a burn to enter an orbit that is
;    coelliptic (shares the same ellipse shape and orientation) with the
;    target vehicle. After CSI, both vehicles travel on orbits with matching
;    shapes but at different altitudes, allowing the chaser to adjust timing.
;
; 2. CONCENTRIC STRATEGY (CDH - Constant Delta Height):
;    The chasing spacecraft performs a height adjustment burn to circularize
;    its orbit at the same altitude as the target vehicle. This establishes
;    concentric circular orbits, setting up the geometry for terminal phase
;    initiation (TPI) and final approach.
;
; P32/P72 calculate CSI parameters, P33/P73 calculate CDH parameters.
; P72/P73 are abort mode variants used when rapid rendezvous is required.
; ============================================================================

# MOD NO -1		LOG SECTION - P32-P35, P72-P75
# MOD BY WHITE.P	DATE 1JUNE67

# PURPOSE

#	 (1) TO CALCULATE PARAMETERS ASSOCIATED WITH THE FOLLOWING
#	     CONCENTRIC FLIGHT PLAN MANEUVERS - THE CO-ELLIPTIC SEQUENCE
#	     INITIATION (CSI) MANEUVER AND THE CONSTANT DELTA ALTITUDE
#	     (CDH) MANEUVER.

#	 (2) TO CALCULATE THESE PARAMETERS BASED UPON MANEUVER DATA
#	     APPROVED AND KEYED INTO THE DSKY BY THE ASTRONAUT.

#	 (3) TO DISPLAY TO THE ASTRONAUT AND THE GROUND DEPENDENT VARIABLES
#	     ASSOCIATED WITH THE CONCENTRIC FLIGHT PLAN MANEUVERS FOR
#	     APPROVAL BY THE ASTRONAUT/GROUND.

#	 (4) TO STORE THE CSI TARGET PARAMETERS FOR USE BY THE DESIRED
#	     THRUSTING PROGRAM.

# ASSUMPTIONS

#	 (1) AT A SELECTED TPI TIME THE LINE OF SIGHT BETWEEN THE ACTIVE
#	     AND PASSIVE VEHICLES IS SELECTED TO BE A PRESCRIBED ANGLE (E)
#	     FROM THE HORIZONTAL PLANE DEFINED BY THE ACTIVE VEHICLE
#	     POSITION.

#	 (2) THE TIME BETWEEN CSI IGNITION AND CDH IGNITION MUST BE
#	     COMPUTED TO BE GREATER THAN 10 MINUTES FOR SUCCESSFUL
#	     COMPLETION OF THE PROGRAM.

#	 (3) THE TIME BETWEEN CDH IGNITION AND TPI IGNITION MUST BE
#	     COMPUTED TO BE GREATER THAN 10 MINUTES FOR SUCCESSFUL
#	     COMPLETION OF THE PROGRAM.

#	 (4) CDH DELTA V IS SELECTED TO MINIMIZE THE VARIATION OF THE
#	     ALTITUDE DIFFERENCE BETWEEN THE ORBITS.

#	 (5) CSI BURN IS DEFINED SUCH THAT THE IMPULSIVE DELTA V IS IN THE
#	     HORIZONTAL PLANE DEFINED BY THE ACTIVE VEHICLE POSITION AT CSI
#	     IGNITION.

#	 (6) THE PERICENTER ALTITUDE OF THE ORBIT FOLLOWING CSI AND CDH
#	     MUST BE GREATER THAN 35,000 FT (LUNAR ORBIT) OR 85 NM (EARTH
#	     ORBIT) FOR SUCCESSFUL COMPLETION OF THIS PROGRAM.

#	 (7) THE CSI AND CDH MANEUVERS ARE ORIGINALLY ASSUMED TO BE
#	     PARALLEL TO THE PLANE OF THE CSM ORBIT.  HOWEVER, CREW
# Page 650
#	     MODIFICATION OF DELTA V (LV) COMPONENTS MAY RESULT IN AN
#	     OUT-OF-PLANE CSI MANEUVER.

#	 (8) STATE VECTOR UPDATES BY P27 ARE DISALLOWED DURING AUTOMATIC
#	     STATE VECTOR UPDATING INITIATED BY P20 (SEE ASSUMPTION 10).

#	 (9) COMPUTED VARIABLES MAY BE STORED FOR LATER VERIFICATION BY
#	     THE GROUND.  THESE STORAGE CAPABILITIES ARE NORMALLY LIMITED
#	     ONLY TO THE PARAMETERS FOR ONE THRUSTING MANEUVER AT A TIME
#	     EXCEPT FOR CONCENTRIC FLIGHT PLAN MANEUVER SEQUENCES.

#	(10) THE RENDEZVOUS RADAR MAY OR MAY NOT BE USED TO UPDATE THE LM
#	     OR CSM STATE VECTORS FOR THIS PROGRAM.  IF RADAR USE IS
#	     DESIRED THE RADAR WAS TURNED ON AND LOCKED BY THE CSM BY
#	     PREVIOUS SELECTION OF P20.  RADAR SIGHTING MARKS WILL BE MADE
#	     AUTOMATICALLY APPROXIMATELY ONCE A MINUTE WHEN ENABLED BY THE
#	     TRACK AND UPDATE FLAGS (SEE P20).  THE RENDEZVOUS TRACKING
#	     MARK COUNTER IS ZEROED BY THE SELECTION OF P20 AND AFTER EACH
#	     THRUSTING MANEUVER.

#	(11) THE ISS NEED NOT BE ON TO COMPLETE THIS PROGRAM.

#	(12) THE OPERATION OF THE PROGRAM UTILIZES THE FOLLOWING FLAGS -

#		ACTIVE VEHICLE FLAG - DESIGNATES THE VEHICLE WHICH IS
#		DOING RENDEZVOUS THRUSTING MANEUVERS TO THE PROGRAM WHICH
#		CALCULATES THE MANEUVER PARAMETERS.  SET AT THE START OF
#		EACH RENDEZVOUS PRE-THRUSTING PROGRAM.

#		FINAL FLAG - SELECTS FINAL PROGRAM DISPLAYS AFTER CREW HAS
#		COMPLETED THE FINAL MANEUVER COMPUTATION AND DISPLAY
#		CYCLE.

#		EXTERNAL DELTA V STEERING FLAG - DESIGNATES THE TYPE OF
#		STEERING REQUIRED FOR EXECUTION OF THIS MANEUVER BY THE
#		THRUSTING PROGRAM SELECTED AFTER COMPLETION OF THIS
#		PROGRAM.

#	(13) IT IS NORMALLY REQUIRED THAT THE ISS BE ON FOR 1 HOUR PRIOR TO
#	     A THRUSTING MANEUVER.

#	(14) THIS PROGRAM IS SELECTED BY THE ASTRONAUT BY DSKY ENTRY -

#		P32 IF THIS VEHICLE IS ACTIVE VEHICLE.

#		P72 IF THIS VEHICLE IS PASSIVE VEHICLE.

# INPUT

#	 (1) TCSI	TIME OF THE CSI MANEUVER
# Page 651
#	 (2) NN		NUMBER OF APSIDAL CROSSINGS THRU WHICH THE ACTIVE
#			VEHICLE ORBIT CAN BE ADVANCED TO OBTAIN THE CDH
#			MANEUVER POINT
#	 (3) ELEV	DESIRED LOS ANGLE AT TPI
#	 (4) TTPI	TIME OF THE TPI MANEUVER
# OUTPUT

#	 (1) TRKMKCNT	NUMBER OF MARKS
#	 (2) TTOGO	TIME TO GO
#	 (3) +MGA	MIDDLE GIMBAL ANGLE
#	 (4) DIFFALT	DELTA ALTITUDE AT CDH
#	 (5) T1TOT2	DELTA TIME FROM CSI TO CDH
#	 (6) T2TOT3	DELTA TIME FROM CDH TO TPI
#	 (7) DELVLVC	DELTA VELOCITY AT CSI - LOCAL VERTICAL COORDINATES
#	 (8) DELVLVC	DELTA VELOCITY AT CDH - LOCAL VERTICAL COORDINATES

# DOWNLINK

#	 (1) TCSI	TIME OF THE CSI MANEUVER
#	 (2) TCDH	TIME OF THE CDH MANEUVER
#	 (3) TTPI	TIME OF THE TPI MANEUVER
#	 (4) TIG	TIME OF THE CSI MANEUVER
#	 (5) DELVEET1	DELTA VELOCITY AT CSI - REFERENCE COORDINATES
#	 (6) DELVEET2	DELTA VELOCITY AT CDH - REFERENCE COORDINATES
#	 (7) DIFFALT	DELTA ALTITUDE AT CDH
#	 (8) NN		NUMBER OF APSIDAL CROSSINGS THRU WHICH THE ACTIVE
#			VEHICLE ORBIT CAN BE ADVANCED TO OBTAIN THE CDH
#			MANEUVER POINT
#	 (9) ELEV	DESIRED LOS ANGLE AT TPI

# COMMUNICATION TO THRUSTING PROGRAMS

#	 (1) TIG	TIME OF THE CSI MANEUVER
#	 (2) RTIG	POSITION OF ACTIVE VEHICLE AT CSI - BEFORE ROTATION
#			INTO PLANE OF PASSIVE VEHICLE
#	 (3) VTIG	VELOCITY OF ACTIVE VEHICLE AT CSE - BEFORE ROTATION
#			INTO PLANE OF PASSIVE VEHICLE
#	 (4) DELVSIN	DELTA VELOCITY AT CSI - REFERENCE COORDINATES
#	 (5) DELVSAB	MAGNITUDE OF DELTA VELOCITY AT CSI
#	 (6) XDELVFLG	SET TO INDICATE EXTERNAL DELTA V VG COMPUTATION

# SUBROUTINES USED

#	 AVFLAGA
#	 AVFLAGP
#	 P20FLGON
#	 VARALARM
#	 BANKCALL
#	 GOFLASH
#	 GOTOPOOH
# Page 652
#	 VNPOOH
#	 GOFLASHR
#	 BLANKET
#	 ENDOFJOB
#	 SELECTMU
#	 ADVANCE
#	 INTINT
#	 PASSIVE
#	 CSI/A
#	 S32/33.1
#	 DISDVLVC
#	 VN1645

; ============================================================================
; PROGRAM ENTRY POINTS AND ABORT MODE DIFFERENCES
;
; P32: NOMINAL COELLIPTIC SEQUENCE INITIATION (CSI) PROGRAM
;      - Used when this vehicle (CSM) is the active vehicle performing the
;        rendezvous maneuvers to reach the passive vehicle (LM).
;      - Sets AVFLAGA (Active Vehicle Flag) to identify CSM as chaser.
;      - Calculates CSI and CDH burns for normal rendezvous timeline.
;
; P72: ABORT MODE CSI PROGRAM FOR PASSIVE VEHICLE
;      - Used when this vehicle (CSM) is the passive target vehicle, but
;        must now become active due to LM abort or system failure.
;      - Sets AVFLAGP (Active Vehicle Passive Flag) to reverse roles.
;      - Calculates rapid rendezvous maneuvers with compressed timeline.
;      - May use different constraints to enable faster orbit matching.
;
; Both programs share common initialization through P32STRT and use identical
; computational subroutines (CSI/A, CDHMVR), but with different vehicle role
; flags affecting state vector selection and coordinate transformations.
; ============================================================================

		BANK	35
		SETLOC	CSI/CDH1
		BANK
		EBANK=	SUBEXIT
		COUNT	35/P3272

; Program entry points - P32 for nominal, P72 for abort mode
P32		TC	AVFLAGA		; Set active vehicle flag (CSM is chaser)
		TC	P32STRT
P72		TC	AVFLAGP		; Set passive-to-active flag (CSM becomes chaser)
P32STRT		TC	INTPRET
		DLOAD
			ZEROVEC
		STORE	CENTANG
		EXIT
		TC	P32/P72A
ALMXITA		SXA,2
			CSIALRM
ALMXIT		LXC,1
			CSIALRM
		SLOAD*	EXIT
			ALARM/TB -1,1
		CA	MPAC
		TC	VARALARM
		CAF	V05N09
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
		TC	-4
P32/P72A	TC	P20FLGON
		TC	INTPRET
		DLOAD
			ZEROVEC
		STORE	NN
		EXIT
		CAF	V06N11		# TCSI
		TC	VNPOOH
		CAF	V06N55		# NN. ELEV(RGL05)
# Page 653
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
P32/P72B	CALL
			ADVANCE
		SETPD	VLOAD
			0D
			VPASS1
		PDVL	PDDL
			RPASS1
			TCSI
		PDDL	PDDL
			TTPI
			2PISC
		SL2	PUSH
		CALL
			INTINT
		CALL
			PASSIVE
		CALL
			CSI/A
P32/P72C	BON	SET
			FINALFLG
			P32/P72D
			UPDATFLG
P32/P72D	DLOAD	GOTO
			T1TOT2
			P32/P72E
		SETLOC	CSI/CDH3
		BANK
P32/P72E	STORE	T1TOT2
		DSU	BPL
			60MIN
			P32/P72E
		DLOAD	GOTO
			T2TOT3
			P32/P72F
		SETLOC	CSI/CDH1
		BANK
P32/P72F	STORE	T2TOT3
		DSU	BPL
# Page 654
			60MIN
			P32/P72F
		EXIT
		CAF	V06N75
		TC	VNPOOH
		TC	INTPRET
		VLOAD	CALL
			DELVEET1
			S32/33.1
		STOVL	DELVEET1
			RACT2
		STOVL	RACT1
			DELVEET2
		AXT,1	CALL
		VN	0682
			DISDVLVC
		DLOAD
			TTPI
		STCALL	TTPIO
			VN1645
		GOTO
			P32/P72B
# Page 655
# CONSTANT DELTA HEIGHT (CDH) PROGRAMS (P33 AND P73)

# MOD NO -1		LOC SECTION - P32-P35, P72-P75
# MOD BY WHITE.P	DATE 1JUNE67
# PURPOSE

#	 (1) TO CALCULATE PARAMETERS ASSOCIATED WITH THE CONSTANT DELTA
#	     ALTITUDE MANEUVER (CDH).

#	 (2) TO CALCULATE THESE PARAMETERS BASED UPON MANEUVER DATA
#	     APPROVED AND KEYED INTO THE DSKY BY THE ASTRONAUT.

#	 (3) TO DISPLAY TO THE ASTRONAUT AND THE GROUND DEPENDENT VARIABLES
#	     ASSOCIATED WITH THE CDH MANEUVER FOR APPROVAL BY THE
#	     ASTRONAUT/GROUND.

#	 (4) TO STORE THE CDH TARGET PARAMETERS FOR USE BY THE DESIRED
#	     THRUSTING PROGRAM.

# ASSUMPTIONS

#	 (1) THIS PROGRAM IS BASED UPON PREVIOUS COMPLETION OF THE
#	     CO-ELLIPTIC SEQUENCE INITIATION (CSI) PROGRAM (P32/P72).
#	     THERFORE -

#	     (A) AT A SELECTED TPI TIME (NOW IN STORAGE) THE LINE OF SIGHT
#		 BETWEEN THE ACTIVE AND PASSIVE VEHICLES WAS SELECTED TO BE
#		 A PRESCRIBED ANGLE (E) (NOW IN STORAGE) FROM THE
#		 HORIZONTAL PLANE DEFINED BY THE ACTIVE VEHICLE POSITION.

#	     (B) THE TIME BETWEEN CSI IGNITION AND CDH IGNITION WAS
#		 COMPUTED TO BE GREATER THAN 10 MINUTES.

#	     (C) THE TIME BETWEEN CDH IGNITION AND TPI IGNITION WAS
#		 COMPUTED TO BE GREATER THAN 10 MINUTES.

#	     (D) THE VARIATION OF THE ALTITUDE DIFFERENCE BETWEEN THE
#		 ORBITS WAS MINIMIZED.

#	     (E) CSI BURN WAS DEFINED SUCH THAT THE IMPULSIVE DELTA V WAS
#		 IN THE HORIZONTAL PLANE DEFINED BY ACTIVE VEHICLE

#		 POSITION AT CSI IGNITION.

#	     (F) THE PERICENTER ALTITUDES OF THE ORBITS FOLLOWING CSI AND
#		 CDH WERE COMPUTED TO BE GREATER THAN 35,000 FT FOR LUNAR
#		 ORBIT OR 85 NM FOR EARTH ORBIT.

#	     (G) THE CSI AND CDH MANEUVERS WERE ASSUMED TO BE PARALLEL TO
#		 THE PLANE OF THE PASSIVE VEHICLE ORBIT.  HOWEVER, CREW
# Page 656
#		 MODIFICATION OF DELTA V (LV) COMPONENTS MAY HAVE RESULTED
#		 IN AN OUT-OF-PLANE MANEUVER.

#	 (2) STATE VECTOR UPDATES BY P27 ARE DISALLOWED DURING AUTOMATIC
#	     STATE VECTOR UPDATING INITIATED BY P20 (SEE ASSUMPTION 4).

#	 (3) COMPUTED VARIABLES MAY BE STORED FOR LATER VERIFICATION BY
#	     THE GROUND.  THESE STORAGE CAPABILITIES ARE NORMALLY LIMITED
#	     ONLY TO THE PARAMETERS FOR ONE THRUSTING MANEUVER AT A TIME
#	     EXCEPT FOR CONCENTRIC FLIGHT PLAN MANEUVER SEQUENCES.

#	 (4) THE RENDEZVOUS RADAR MAY OR MAY NOT BE USED TO UPDATE THE LM
#	     OR CSM STATE VECTORS FOR THIS PROGRAM.  IF RADAR USE IS
#	     DESIRED THE RADAR WAS TURNED ON AND LOCKED ON THE CSM BY
#	     PREVIOUS SELECTION OF P20.  RADAR SIGHTING MARKS WILL BE MADE
#	     AUTOMATICALLY APPROXIMATELY ONCE A MINUTE WHEN ENABLED BY THE
#	     TRACK AND UPDATE FLAGS (SEE P20).  THE RENDEZVOUS TRACKING
#	     MARK COUNTER IS ZEROED BY THE SELECTION OF P20 AND AFTER EACH
#	     THRUSTING MANEUVER.

#	 (5) THE ISS NEED NOT BE ON TO COMPLETE THIS PROGRAM.

#	 (6) THE OPERATION OF THE PROGRAM UTILIZES THE FOLLOWING FLAGS -

#		ACTIVE VEHICLE FLAG - DESIGNATES THE VEHICLE WHICH IS
#		DOING RENDEZVOUS THRUSTING MANEUVERS TO THE PROGRAM WHICH
#		CALCULATES THE MANEUVER PARAMETERS.  SET AT THE START OF
#		EACH RENDEZVOUS PRE-THRUSTING PROGRAM.

#		FINAL FLAG - SELECTS FINAL PROGRAM DISPLAYS AFTER CREW HAS
#		COMPLETED THE FINAL MANEUVER COMPUTATION AND DISPLAY
#		CYCLE.

#		EXTERNAL DELTA V STEERING FLAG - DESIGNATES THE TYPE OF
#		STEERING REQUIRED FOR EXECUTION OF THIS MANEUVER BY THE
#		THRUSTING PROGRAM SELECTED AFTER COMPLETION OF THIS
#		PROGRAM.

#	 (7) IT IS NORMALLY REQUIRED THAT THE ISS BE ON FOR 1 HOUR PRIOR TO
#	     A THRUSTING MANEUVER.

#	 (8) THIS PROGRAM IS SELECTED BY THE ASTRONAUT BY DSKY ENTRY -

#		P33 IF THIS VEHICLE IS ACTIVE VEHICLE.

#		P73 IF THIS VEHICLE IS PASSIVE VEHICLE.

# INPUT

#	 (1) TTPIO	TIME OF THE TPI MANEUVER - SAVED FROM P32/P72
# Page 657
#	 (2) ELEV	DESIRED LOS ANGLE AT TPI - SAVED FROM P32/P72
#	 (3) TCDH	TIME OF THE CDH MANEUVER

# OUTPUT

#	 (1) TRKMKCNT	NUMBER OF MARKS
#	 (2) TTOGO	TIME TO GO
#	 (3) +MGA	MIDDLE GIMBAL ANGLE
#	 (4) DIFFALT	DELTA ALTITUDE AT CDH
#	 (5) T2TOT3	DELTA TIME FROM CDH TO COMPUTED TPI
#	 (6) NOMTPI	DELTA TIME FROM NOMINAL TPI TO COMPUTED TPI
#	 (7) DELVLVC	DELTA VELOCITY AT CDH - LOCAL VERTICAL COORDINATES

# DOWNLINK

#	 (1) TCDH	TIME OF THE CDH MANEUVER
#	 (2) TTPI	TIME OF THE TPI MANEUVER
#	 (3) TIG	TIME OF THE CDH MANEUVER
#	 (4) DELLVEET2	DELTA VELOCITY AT CDH - REFERENCE COORDINATES
#	 (5) DIFFALT	DELTA ALTITUDE AT CDH
#	 (6) ELEV	DESIRED LOS ANGLE AT TPI
# COMMUNICATION TO THRUSTING PROGRAMS

#	 (1) TIG	TIME OF THE CDH MANEUVER
#	 (2) RTIG	POSITION OF ACTIVE VEHICLE AT CDH - BEFORE ROTATION
#			INTO PLANE OF PASSIVE VEHICLE
#	 (3) VTIG	VELOCITY OF ACTIVE VEHICLE AT CDH - BEFORE ROTATION
#			INTO PLANE OF PASSIVE VEHICLE
#	 (4) DELVSIN	DELTA VELOCITY AT CDH - REFERENCE COORDINATES
#	 (5) DELVSAB	MAGNITUDE OF DELTA VELOCITY AT CDH
#	 (6) XDELVFLG	SET TO INDICATE EXTERNAL DELTA V VG COMPUTATION

# SUBROUTINES USED

#	 AVFLAGA
#	 AVFLAGP
#	 P20FLGON
#	 VNPOOH
#	 SELECTMU
#	 ADVANCE
#	 CDHMVR
#	 INTINT3P
#	 ACTIVE
#	 PASSIVE
#	 S33/S34.1
#	 ALARM
#	 BANKCALL
#	 GOFLASH
#	 GOTOPOOH
#	 S32/33.1
# Page 658
#	 VN1645

; ============================================================================
; P33/P73 CONSTANT DELTA HEIGHT (CDH) PROGRAM ENTRY POINTS
;
; P33: NOMINAL CONSTANT DELTA HEIGHT (CDH) PROGRAM
;      - Follows P32 (CSI) in the rendezvous sequence when CSM is active.
;      - Calculates the CDH burn to circularize orbit at target altitude.
;      - Height adjustment maneuver minimizes altitude variation between orbits.
;      - Establishes concentric circular orbits for terminal phase initiation.
;
; P73: ABORT MODE CDH PROGRAM FOR PASSIVE VEHICLE
;      - Follows P72 (abort CSI) when CSM becomes active due to LM abort.
;      - Calculates rapid height adjustment with compressed timeline.
;      - May use different altitude constraints than nominal P33.
;
; RENDEZVOUS SEQUENCE POSITIONING:
;      Nominal: P32 (CSI) → P33 (CDH) → P34 (TPI) → final approach
;      Abort:   P72 (CSI) → P73 (CDH) → P74 (TPI) → final approach
;
; The CDH maneuver typically occurs 10+ minutes after CSI and 10+ minutes
; before TPI, allowing sufficient time for trajectory stabilization and
; crew verification of orbital parameters.
; ============================================================================

		COUNT	35/P3373

; Program entry points - P33 for nominal, P73 for abort mode
P33		TC	AVFLAGA		; Set active vehicle flag (CSM is chaser)
		TC	P33/P73A
P73		TC	AVFLAGP		; Set passive-to-active flag (CSM becomes chaser)
P33/P73A	TC	P20FLGON
		CAF	V06N13		# TCDH
		TC	VNPOOH
		TC	INTPRET
		DLOAD
			TTPIO
		STODL	TTPI
			TCDH
		STCALL	TIG
			SELECTMU
P33/P73B	CALL
			ADVANCE
		CALL
			CDHMVR
		SETPD	VLOAD
			0D
			VACT3
		PDVL	CALL
			RACT2
			INTINT3P
		CALL
			ACTIVE
		SETPD	VLOAD
			0D
			VPASS2
		PDVL	CALL
			RPASS2
			INTINT3P
		CALL
			PASSIVE
		DLOAD	SET
			ZEROVEC
			ITSWICH
		STCALL	NOMTPI
			S33/34.1
		BZE	EXIT
			P33/P73C
		TC	ALARM
		OCT	611
		CAF	V05N09
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH
# Page 659
		TC	+2
		TC	P33/P73A
		TC	INTPRET
		DLOAD
			ZEROVEC
		STCALL	NOMTPI
			P33/P73C
		SETLOC	CSI/CDH2
		BANK

P33/P73C	BON	SET
			FINALFLG
			P33/P73D
			UPDATFLG
P33/P73D	DLOAD	DAD
			NOMTPI
			TTPI
		STORE	TTPI
		DSU	GOTO
			TCDH
			P33/P73E
		SETLOC	CSI/CDH1
		BANK

P33/P73E	DSU	BPL
			60MIN
			P33/P73E
		DAD
			60MIN
		STODL	T1TOT2
			TTPI
		DSU	PUSH
			TTPIO
P33/P73F	ABS	DSU
			60MIN
		BPL	DAD
			P33/P73F
			60MIN
		SIGN	STADR
		STORE	T2TOT3
		EXIT
		CAF	V06N75
		TC	VNPOOH
		TC	INTPRET
		VLOAD	CALL
			DELVEET2
			S32/33.1
		STCALL	DELVEET2
			VN1645
		GOTO
# Page 660
			P33/P73B
# Page 661
# ..... AVFLAGA/P .....
# Page 662
# ..... DISDVLVC .....

# SUBROUTINES USED

#	 S32/33.X
#	 VNPOOH

; ============================================================================
; DISDVLVC - DISPLAY DELTA-V IN LOCAL VERTICAL COORDINATES
;
; PURPOSE:
;      Transforms delta-V from stable member (inertial) coordinates to local
;      vertical coordinates (LVC) and displays the components to the crew via
;      DSKY for review and approval.
;
; OPERATIONAL CONTEXT:
;      Orbital mechanics computations produce delta-V values that must be
;      presented to the crew in an intuitive reference frame. Local vertical
;      coordinates express the velocity change in terms meaningful for orbital
;      maneuvers:
;        - Radial component (perpendicular to surface, positive = up)
;        - In-track component (along velocity vector, positive = prograde)
;        - Cross-track component (perpendicular to orbital plane)
;
;      The crew uses these components to assess the maneuver's effect on the
;      orbit before committing to engine ignition.
;
; COORDINATE TRANSFORMATION:
;      The input delta-V (in DELVLVC initially holding inertial coordinates)
;      is transformed to local vertical coordinates using the transformation
;      matrix constructed by S32/33.X:
;
;      DELVLVC_transformed = 0D × DELVLVC_inertial
;
;      Where 0D is the 3×3 matrix mapping inertial vectors to the local
;      vertical reference frame at the spacecraft's current position.
;
; DISPLAY FORMAT:
;      The transformed delta-V is displayed via verb/noun combination stored
;      in VERBNOUN (typically V06N81 or similar), showing:
;        R1: Radial component (up/down relative to surface)
;        R2: In-track component (forward/backward along orbit)
;        R3: Cross-track component (left/right perpendicular to plane)
;
;      Crew can then:
;        - Press PROCEED to accept the displayed values
;        - Key in modified components and press ENTER to update the maneuver
;        - Press RESET to abort the program sequence
;
; SUBROUTINE CALLS:
;      S32/33.X - Constructs local vertical coordinate transformation matrix 0D
;      VNPOOH   - Verb/noun display processor for DSKY output
;
; INPUT:
;      DELVLVC - Delta-V vector (initially in inertial coordinates)
;      VERBNOUN - Packed verb/noun code for display format
;
; OUTPUT:
;      DELVLVC - Delta-V vector transformed to local vertical coordinates
;                (available for crew review and modification)
;
; REGISTER USAGE:
;      X1 - Saved/restored via SXA,1 instruction for verb/noun processing
;
; RETURN:
;      Returns via NORMEX exit, allowing calling routine to proceed based on
;      crew input (PROCEED, modified values, or abort).
;
; This routine provides essential crew situational awareness by presenting
; delta-V in orbital mechanics terms rather than spacecraft body coordinates,
; enabling informed decision-making before committing to the maneuver.
; ============================================================================

		SETLOC	CDHTAG3
		BANK

DISDVLVC	STORE	DELVLVC
		STQ	CALL
			NORMEX
			S32/33.X
		VLOAD	MXV
			DELVLVC
			0D
		VSL1	SXA,1
			VERBNOUN
		STORE	DELVLVC
		EXIT
		CA	VERBNOUN
		TC	VNPOOH
		TC	INTPRET
		GOTO
			NORMEX
		SETLOC	FFTAG12
		BANK

V06N11		VN	0611
V06N13		VN	0613
V06N75		VN	0675

V06N50		VN	0650
# Page 663
# ..... CSI/A .....

# SUBROUTINES USED

#	 VECSHIFT
#	 TIMETHET
#	 PERIAPO
#	 SHIFTR1
#	 INTINT2C
#	 CDHMVR
#	 PERIAPO1
#	 INTINT
#	 ACTIVE

		BANK	34
		SETLOC	CSIPROG
		BANK
		EBANK=	SUBEXIT
		COUNT	34/CSI

60MIN		2DEC	360000

ALARM/TB	OCT	00600		# NO 1
		OCT	00601		#    2
		OCT	00602		#    3
		OCT	00603		#    4
		OCT	00604		#    5
		OCT	00605		#    6
		OCT	00606		#    7
LOOPMX		2DEC	16

INITST		2DEC	.03048 B-7	# INITIAL DELDV = 10 FPS

DVMAX1		2DEC	3.0480 B-7	# MAXIMUM DV1 = 1000 FPS

DVMAX2		2DEC	3.014472 B-7	#                989 FPS

1DPB2		2DEC	1.08-2

1DPB28		2DEC	1

EPSILN1		2DEC	.0003048 B-7	# .1 FPS

FIFPSDP		2DEC	-.152400 B-7	# 50  FPS

DELMAX1		2DEC	.6096000 B-7	# 200 FPS

		SETLOC	CSI/CDH
		BANK
PMINE		2DEC	157420 B-29	# 84 NM. - MUST BE 8 WORDS BEFORE PMINM
# Page 664
NICKELDP	2DEC	.021336 B-7	# 7 FPS

INITST1		2DEC	.03048 B-7	# INITIAL DELDV = 10 FPS

ONETHTH		2DEC	.0001 B-3

PMINM		2DEC	10668 B-29	# 35000 FT - MUST BE 8 WORDS AFTER PMINE.

		SETLOC	CSIPROG
		BANK

; ============================================================================
; CSI/A - CO-ELLIPTIC SEQUENCE INITIATION CALCULATION ENTRY POINT
;
; This routine initializes the iterative CSI targeting computation. The CSI
; maneuver adjusts the active vehicle's orbit to be co-elliptic with the
; target, meaning both orbits have the same apogee and perigee altitudes but
; different phasing. This is the first step in the concentric rendezvous
; sequence.
;
; The calculation uses a Newton-Raphson iteration to solve for the delta-V
; magnitude that produces the desired orbital geometry. Multiple flag bits
; control the iteration state machine, tracking first/second pass status
; and handling special cases when convergence is difficult.
;
; COMPUTATIONAL APPROACH:
; - Initialize iteration counters and state flags
; - Assume initial delta-V of 10 fps (INITST)
; - Compute resulting orbit parameters
; - Iterate to converge on target geometry
; - Check constraints (minimum perigee altitude, time separation)
;
; STATE FLAGS:
; S32.1F1 = Delta-V has exceeded maximum (alarm condition)
; S32.1F2 = First pass of Newton iteration (derivative not yet available)
; S32.1F3A/B = Iteration cycle tracking (00=initial, 01=first, 10=second, 11=50fps stage)
; ============================================================================

CSI/A		CLEAR	SET		# INITIALIZE INDICATORS
			S32.1F1		# DVT1 HAS EXCEEDED MAX INDICATOR
			S32.1F2		# FIRST PASS FOR NEWTON ITERATION INDICATR
		CLEAR	SET
			S32.1F3A	# 00=1ST 2 PASSES 2ND CYCLE 01=FIRST CYCLE
			S32.1F3B	# 10=2ND CYCLE, 11=50 FPS STAGE 2ND CYCLE
		DLOAD
			ZEROVEC
		STORE	LOOPCT
		STORE	CSIALRM
; ============================================================================
; CSI/B - MAIN CSI DELTA-V COMPUTATION
;
; This section computes the required CSI delta-V based on the current iteration
; parameters. It calculates the horizontal component of the velocity change
; needed to achieve a co-elliptic orbit with the target vehicle.
;
; GEOMETRIC SETUP:
; - RACT1 = Active vehicle position at CSI time
; - RPASS3 = Passive (target) vehicle position at TPI (terminal phase initiation)
; - The delta-V is constrained to lie in the local horizontal plane
;
; CALCULATION SEQUENCE:
; 1. Compute magnitude of active position vector (RA1)
; 2. Compute ratio RA1/RP3 (active radius to passive radius at TPI)
; 3. Calculate orbital velocity using vis-viva equation variant
; 4. Compute local horizontal reference (perpendicular to radial direction)
; 5. Project current velocity onto horizontal plane
; 6. Compute required velocity change (DELVCSI)
;
; The formula implements: DELVCSI = sqrt(mu*(1+RA1/RP3)/RA1) - VA1·UH1
; where mu is gravitational parameter, UH1 is horizontal unit vector
;
; SCALING:
; B29 = 2^29 meters for position vectors
; B7 = 2^7 meters/centisecond for velocities
; ============================================================================

CSI/B		SETPD	VLOAD
			0D
			RACT1
		ABVAL	PUSH		# RA1                            B29 PL02D
		NORM	SR1
			X2		#                         B29-N2+ B1 PL04D
		PDVL	ABVAL
			RPASS3
		NORM	BDDV		# RA1/RP3                         B1 PL02D
			X1
		XSU,2	SR*		#                                 B2
			X1
			1,2
		DAD	DMP		# (1+(RA1/RP3))RA1        B29+B2=B31 PL00D
			1DPB2
		NORM	PDDL		#                                    PL02D
			X1
			RTMU
		SR1	DDV		#                        B38-B31= B7 PL00D
		SL*	SQRT		#                                 B7
			0 -7,1
		PDVL	UNIT		#                                    PL02D
			RACT1
		PDVL	VXV
			UP1
		UNIT			# UNIT(URP1 X UVP1 X URA1) = UH1
		DOT	SL1		# VA1 . UH1                       B7
			VACT1
		BDSU	STADR		#                                    PL00D
# Page 665
		STODL	DELVCSI
			INITST		# 10 FPS
		STORE	DELDV
CSI/B1		DLOAD	DAD		# IF LOOPCT = 16
			LOOPCT
			1DPB28
		STORE	LOOPCT
		DSU	AXT,2
			LOOPMX
			6
		BPL	GOTO
			SCNDSOL
			CSI/B2

		SETLOC	CSIPROG2
		BANK

CSI/B2		SETPD
			0D
		DLOAD	ABS
			DELVCSI
		DSU	BMN
			DVMAX1
			CSI/B23
		AXT,2	BON
			7
			S32.1F1
			SCNDSOL
		BOFF	BON
			S32.1F3A
			CSI/B22		# FLAG 3 NEQ 3
			S32.1F3B
			SCNDSOL
CSI/B22		SET	DLOAD
			S32.1F1
			DVMAX2
		SIGN
			DELVCSI
		STCALL	DELVCSI
			CSI/B23

		SETLOC	CSIPROG3
		BANK

CSI/B23		VLOAD	PUSH
			RACT1
		UNIT	PDVL
			UP1
		VXV	UNIT		# UNIT(URP1 X UVP1 X URA1) = UH1
		VXSC	VSL1
# Page 666
			DELVCSI
		STORE	DELVEET1
		VAD	BOV
			VACT1
			CSI/B23D
CSI/B23D	STCALL	VACT4
			VECSHIFT
		STOVL	VVEC
		SET
			RVSW
		STOVL	RVEC
			SN359+
		STCALL	SNTH		# ALSO CSTH
			TIMETHET
		SR1	LXA,1
			RTX1
		STCALL	HAFPA1
			PERIAPO
		CALL
			SHIFTR1
		STODL	POSTCSI
			CENTANG
		BZE	GOTO
			+2
			CIRCL
		DLOAD
			ECC
		DSU	BMN
			ONETHTH
			CIRCL
		DLOAD	CALL
			R1
			SHIFTR1
		SETPD	NORM
			2D
			X1
		PDVL	DOT		#                                    PL04D
			RACT1
			VACT4
		ABS	DDV
			02D		# (/RDOTV/)/R1		B36-B29= B7
		SL*	DSU
			0,1
			NICKELDP
		BMN	DLOAD
			CIRCL
			P
		SL2	DSU
			1RTEB2		# 1.B.2
		STODL	14D
# Page 667
			RTSR1/MU
		SR1	DDV		# (1/ROOTMU)/R1      B-16-B29 = B-45 PL02D
		PDDL	DMP
			P
			R1
		CALL
			SHIFTR1
		SL4	SL1
		SQRT	DMP		# ((P/MU)**.5)/R1    B14+B-45 = B-31 PL02D
		BOFF	SL3
			CMOONFLG
			CSI/B3
CSI/B3		PDVL	DOT
			RACT1
			VACT4
		STORE	RDOTV
		ABS
		NORM	DMP		# ((P/MU)**.5)RDOTV/R1               PL02D
			X2
		XSU,1	SL*		#			B-31+B36-B3 = B2
			X2
			3,1
		STODL	12D
			ZEROVECS
		STORE	16D
		VLOAD	UNIT
			12D
		STOVL	SNTH		# ALSO STORES CSTH AND 0
			RACT1
		PDVL	SIGN
			VACT4
			RDOTV
		VCOMP	CALL
			VECSHIFT
		STOVL	VVEC
		SETGO
			RVSW
			CSINEXT

SN359+		2DEC	-.000086601

CS359+		2DEC	+.499999992

		SETLOC	CSIPROG4
		BANK

CSINEXT		STCALL	RVEC
			TIMETHET
		PDDL	BPL
			RDOTV
# Page 668
			NTP/2
		DLOAD	DSU
			HAFPA1
		PUSH	GOTO
			NTP/2
CIRCL		SETPD	DLOAD
			00D
			ZEROVECS
		PUSH
NTP/2		DLOAD	DMP
			NN
			HAFPA1
		SL	DSU
			14D
		DAD
			TCSI
		STORE	TCDH
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
		SETPD	PDVL
			0D
			RPASS1
		GOTO
			CSINEXT1

		SETLOC	CSIPROG5
		BANK

CSINEXT1	CALL
			INTINT2C
		STOVL	RPASS2
			VATT
		STCALL	VPASS2
			CDHMVR
		VLOAD	SETPD
			RACT2
			0D
# Page 669
		PDVL	CALL
			VACT3
			PERIAPO1
		CALL
			SHIFTR1
		STOVL	POSTCDH
			VACT3
		SETPD	PDVL
			0D
			RACT2
		PDDL	PDDL
			TCDH
			TTPI
		PDDL	SL2
			2PISC
		PUSH	CALL
			INTINT
		CALL
			ACTIVE
		DLOAD
			ELEV
		SETPD	SINE
			6D
		PDVL	UNIT
			RACT3
		STORE	00D		# URA3 AT 00D
		PDVL	VXV		# PL14D,PL08D
			UP1
		UNIT
		PDDL	COSINE		# UNIT(URA3XUVA3XURA3) = UH3      B1 PL14D
			ELEV
		VXSC	STADR		# (COSLOS)(UH3)                   B2 PL08D
		STCALL	18D		#		PLUS
			CSINEXT2

		SETLOC	CSIPROG6
		BANK

CSINEXT2	DLOAD	VXSC		# (SINLOS)(URA3) = U              B2 PL00D
		VAD	VSL1
			18D		#                                 B1
		PUSH	DOT		#                                    PL06D
			RACT3		# (U . RA3) = TEMP1      B1 +B29=B30
		SL1	PUSH		#                                B29 PL08D
		DSQ	TLOAD		# TEMP1**2                       B58
			MPAC
		PDVL	DOT		#                                    PL11D
			RACT3
			RACT3
		TLOAD	DCOMP		# RA3.RA3
# Page 670
			MPAC
		PDVL	DOT		# RP3.RP3                        B58 PL14D
			RPASS3
			RPASS3		#                                    PL11D
		TAD	TAD		# TEMP1**2+RA3.RA3+RP3.RP3=TEMP2     PL08D
		BPL	DLOAD
			K10RK2
			LOOPCT
		DSU	AXT,2
			1DPB28
			1D
		BZE
			ALMXITA
		DLOAD	SR1
			DELDV
		STORE	DELDV
		BDSU
			DVPREV
		STCALL	DELVCSI
			CSI/B1
K10RK2		SQRT	PUSH		# TEMP3 = TEMP2**.5              B29 PL10D
		DCOMP	DSU
			06D		# -TEMP1-TEMP3 =K2 AT 10D
		STODL	10D		#                                    PL08D
		DSU	STADR		#                                    PL06D
		STORE	12D		# -TEMP1+TEMP3 =K1 AT 12D
		ABS
		STODL	14D
			10D
		ABS	DSU
			14D
		BMN	DLOAD
			K2.
			12D
		STCALL	10D		# K EQUALS K1
			K2.

		SETLOC	CSIPROG7
		BANK

; ============================================================================
; NEWTON-RAPHSON ITERATION CONVERGENCE LOGIC
;
; This section implements the heart of the Newton-Raphson iterative solver
; for the CSI targeting problem. The goal is to find the delta-V magnitude
; that produces a specific elevation angle (ELEV) at TPI time.
;
; ITERATION APPROACH:
; The program iterates on DELVCSI (delta-V magnitude) to drive GAMMA
; (computed elevation angle at TPI) to match the desired ELEV angle.
;
; GEOMETRIC CALCULATION:
; 1. Compute transfer trajectory from CSI to TPI using Kepler's equation
; 2. Calculate elevation angle GAMMA at TPI based on line-of-sight geometry
; 3. Compare GAMMA to desired ELEV
; 4. Adjust DELVCSI using Newton-Raphson: DV_new = DV_old - f(DV)/f'(DV)
;
; CONVERGENCE CRITERIA:
; - Iteration continues until GAMMA converges to ELEV within tolerance
; - Maximum iteration count prevents infinite loops
; - Special handling for difficult convergence cases (50 fps stage)
;
; The elevation angle GAMMA is computed as:
; GAMMA = SIGN(cross_product) * ARCCOS(unit_vector_dot_product)
; where the vectors relate the active and passive vehicle positions at TPI
; ============================================================================

K2.		DLOAD
			10D
		VXSC	VSL1
		VAD	UNIT		# V=RA3+KU  UNIT                  B1
			RACT3
		PDVL	UNIT		#                                    PL06D
			RPASS3
		PDVL	UNIT		#                                    PL12D
			VPASS3
		VXV	PDVL		# UVP3 X URP3                        PL18D
# Page 671
			06D
			06D
		VXV	DOT
			00D
		STADR			#                                    PL12D
		STOVL	12D		# (URP3XV).(UVP3XURP3)=TEMP          PL06D
		DOT	SL1		#                                    PL00D
		ARCCOS	SIGN
			12D		#                                 B0
		SR1	PUSH		# GAMMA=SIGN(TEMP)ARCOS(UNITV.URP3)  PL02D
		BON	DLOAD
			S32.1F2
			FRSTPAS
			00D		# NOT THE FIRST PASS OF A CYCLE
		DSU	PDDL		# GAMMA-GAMPREV                   B1 PL04D
			GAMPREV
			DELVCSI
		DSU	NORM		#                                 B7
			DVPREV
			X1
		BDDV	PDDL		# (GAM-GAMPREV)/(DV-DVPREV)   B-6+N1 PL06D
			02D		#	= SLOPE
			DELVCSI
		STORE	DVPREV
		BOFF	BOFF
			S32.1F3A
			THRDCHK
			S32.1F3B
			THRDCHK
; ============================================================================
; SPECIAL CONVERGENCE HANDLING - 50 FPS STAGE
;
; If the Newton iteration is on the second cycle (both F3A and F3B set),
; this section checks whether the solution is converging properly. If the
; computed slope and previous gamma have opposite signs, convergence is
; problematic, and the routine switches to a fixed 50 fps delta-V increment
; strategy to help drive the solution toward convergence.
;
; This handles cases where the transfer orbit geometry makes the standard
; Newton-Raphson method unstable or slow to converge.
; ============================================================================
		DLOAD	DMP
			02D
			GAMPREV
		BPL	DLOAD
			FIFTYFPS
			INITST1
		SIGN
			DELDV
		STORE	DELDV
		SET	CLEAR
			S32.1F3A
			S32.1F3B
; ============================================================================
; FRSTPAS - FIRST PASS INITIALIZATION
;
; On the first pass of the Newton iteration, no previous delta-V or gamma
; value exists, so a derivative cannot be computed. The routine initializes
; with the current gamma value and prepares for the next iteration.
;
; After the first pass, the routine has two data points (current and previous)
; and can compute the slope (d_gamma/d_deltaV) needed for Newton-Raphson.
; ============================================================================
FRSTPAS		DLOAD
			00D
		STODL	GAMPREV
			DELVCSI
		STCALL	DVPREV
			CSINEXT3

		SETLOC	CSIPROG8
		BANK
# Page 672
CSINEXT3	DSU	CLEAR
			DELDV
			S32.1F2
		STCALL	DELVCSI
			CSI/B1
THRDCHK		BON	BON
			S32.1F3A
			NEWTN
			S32.1F3B
			NEWTN
FIFTYFPS	DLOAD	SIGN
			FIFPSDP
			04D
		SIGN
			GAMPREV
		STORE	DELDV
		DCOMP	DAD
			DELVCSI
		STODL	DELVCSI
			00D
		SET	SET
			S32.1F3B
			S32.1F3A
		STCALL	GAMPREV
			CSI/B2
NEWTN		DLOAD	NORM
			04D
			X2
		BDDV	XSU,1
			00D
			X2
		SR*
			0,1
		STODL	DELDV
			00D
		STORE	GAMPREV
		DLOAD	ABS
			DELDV
		PUSH	DSU		#                                    PL08D
			EPSILN1
		BMN	DLOAD
			CSI/SOL
		DSU	BMN
			DELMAX1
			CSISTEP
		DLOAD	SIGN
			DELMAX1
			DELDV
		STORE	DELDV
CSISTEP		DLOAD	DSU
# Page 673
			DELVCSI
			DELDV
		STCALL	DELVCSI
			CSI/B1
CSI/SOL		DLOAD	AXT,2
			POSTCSI
			2
		LXA,1	GOTO
			RTX1
			CSINEXT4

		SETLOC	CSIPROG9
		BANK

CSINEXT4	DSU*	BMN
			PMINE -2,1
			SCNDSOL
		AXT,2	DLOAD
			3
			POSTCDH
		DSU*	BMN
			PMINE -2,1
			SCNDSOL
		DLOAD	DSU
			TCDH
			TCSI
		STORE	T1TOT2
		AXT,2	DSU
			4
			600SEC
		BMN	AXT,2
			SCNDSOL
			5
		DLOAD	DSU
			TTPI
			TCDH
		STORE	T2TOT3
		DSU	BPL
			600SEC
			P32/P72C
; ============================================================================
; SCNDSOL - SECOND SOLUTION ATTEMPT (ITERATION RESET)
;
; This routine is called when constraint checks fail during the CSI computation,
; such as when:
; - Perigee altitude falls below minimum safe altitude (35,000 ft lunar or 85 nm Earth)
; - Time separation between CSI and CDH is less than 10 minutes (600 seconds)
; - Time separation between CDH and TPI is less than 10 minutes (600 seconds)
;
; RESET STRATEGY:
; When the current iteration produces an unacceptable solution, rather than
; declaring complete failure, this routine resets the iteration state machine
; to its initial condition and attempts the computation again. This provides
; robustness against numerical instabilities or poor initial guesses.
;
; ALARM CONDITION:
; If both S32.1F3A and S32.1F3B are set (indicating the iteration is already
; on its second cycle), the routine exits via ALMXIT rather than attempting
; a third cycle. This prevents infinite loops when no valid solution exists.
;
; RESET OPERATIONS:
; - Clear S32.1F1 (DVT1 exceeded max flag)
; - Set S32.1F2 (first pass flag - restart iteration)
; - Clear S32.1F3A/B (cycle tracking flags)
; - Reset loop counter
; - Re-enter CSI/B to recompute from fresh state
;
; This approach reflects the spacecraft's need for solution robustness during
; critical rendezvous operations where failure to compute a valid trajectory
; could jeopardize mission success.
; ============================================================================
SCNDSOL		BON	BOFF
			S32.1F3A
			ALMXIT
			S32.1F3B
			ALMXIT
		SXA,2	DLOAD
			CSIALRM
			ZEROVECS
		CLEAR	SET
			S32.1F1
# Page 674
			S32.1F2
		CLEAR	CLEAR
			S32.1F3A
			S32.1F3B
		STCALL	LOOPCT
			CSI/B
# Page 675
# ..... ADVANCE .....

# SUBROUTINES USED

#	 PRECSET
#	 ROTATE

; ============================================================================
; ADVANCE - STATE VECTOR ADVANCEMENT AND REFERENCE FRAME SETUP
;
; PURPOSE:
;      Advances the passive and active vehicle state vectors from their current
;      time (T3) to the CDH maneuver time (TIG), establishing the orbital
;      reference frame needed for the Constant Delta Height computation.
;
; OPERATIONAL CONTEXT:
;      After computing the CSI maneuver parameters, the program must project
;      both vehicles forward in time to the CDH burn time. This routine
;      integrates the orbital motion and sets up the coordinate system for
;      computing the height adjustment delta-V.
;
; STATE VECTOR PROGRESSION:
;      Input state vectors (T3):
;        RPASS3, VPASS3 - Passive vehicle position and velocity
;        RACT3, VACT3   - Active vehicle position and velocity
;
;      Intermediate state vectors (T2 = TIG):
;        RPASS2, VPASS2 - Passive vehicle at CDH time
;        RACT2, VACT2   - Active vehicle at CDH time
;
;      Reference copies (T1):
;        RPASS1, VPASS1 - Passive vehicle reference state
;        RACT1, VACT1   - Active vehicle reference state
;
; COORDINATE SYSTEM SETUP:
;      UP1 = Unit vector perpendicular to orbital plane
;          = RPASS1 × VPASS1 / |RPASS1 × VPASS1|
;
;      This unit perpendicular vector defines the out-of-plane direction for
;      the concentric rendezvous geometry, ensuring CDH delta-V is applied
;      in the correct reference frame.
;
; ROTATION OPERATION:
;      The ROTATE subroutine is called twice to transform active vehicle
;      state vectors into the reference frame defined by UP1:
;        1. RACT3 → RACT2, RACT1 (position rotated to reference frame)
;        2. VACT3 → VACT2, VACT1 (velocity rotated to reference frame)
;
; PRECSET INITIALIZATION:
;      Sets TDEC1 = TIG (CDH burn time) to establish the epoch for coordinate
;      transformations and state vector extrapolation.
;
; XDELVFLG:
;      Set to indicate that delta-V computations are in progress, signaling
;      downstream routines to use the advanced state vectors.
;
; SUBROUTINE CALLS:
;      PRECSET - Initializes precision integration parameters for time TDEC1
;      ROTATE  - Rotates vectors into orbital reference frame defined by UP1
;
; This advancement ensures that the CDH computation operates on accurately
; propagated state vectors at the correct maneuver time, accounting for
; orbital motion during the CSI-to-CDH coast phase.
; ============================================================================

		SETLOC	CDHTAG3
		BANK

ADVANCE		STQ	DLOAD
			SUBEXIT
			TIG
		STCALL	TDEC1
			PRECSET
		SET	VLOAD
			XDELVFLG
			VPASS3
		STORE	VPASS2
		STOVL	VPASS1
			RPASS3
		STORE	RPASS2
		STORE	RPASS1
		UNIT	VXV
			VPASS1
		UNIT
		STOVL	UP1
			RACT3
		STCALL	RTIG
			ROTATE
		STORE	RACT2
		STOVL	RACT1
			VACT3
		STCALL	VTIG
			ROTATE
		STORE	VACT2
		STCALL	VACT1
			SUBEXIT
# Page 676
# ..... ROTATE .....

; ============================================================================
; ROTATE - VECTOR ROTATION INTO ORBITAL REFERENCE FRAME
;
; PURPOSE:
;      Rotates a 3D vector into the orbital reference frame defined by the
;      unit perpendicular vector UP1. This transformation is essential for
;      ensuring that rendezvous computations are performed in a consistent
;      coordinate system aligned with the orbital plane.
;
; OPERATIONAL CONTEXT:
;      During rendezvous operations, the active and passive vehicles' state
;      vectors must be expressed in a common reference frame. UP1 defines
;      the out-of-plane direction (perpendicular to the orbital plane), and
;      this routine projects vectors into coordinates aligned with that frame.
;
; MATHEMATICAL OPERATION:
;      Given input vector V and reference direction UP1:
;
;      1. Compute out-of-plane component:
;         V_perp = (V · UP1) * UP1
;         This is the projection of V onto the UP1 direction.
;
;      2. Compute in-plane component:
;         V_plane = V - V_perp
;         This removes the out-of-plane component, leaving only the
;         component lying in the orbital plane.
;
;      3. Normalize in-plane component:
;         V_unit = V_plane / |V_plane|
;         This creates a unit vector in the desired direction.
;
;      4. Scale by original magnitude:
;         V_rotated = V_unit * |V|
;         This preserves the original vector's magnitude while ensuring
;         it lies in the orbital plane.
;
; INTERPRETIVE INSTRUCTION SEQUENCE:
;      PUSH PUSH       - Save input vector twice on stack for reuse
;      DOT UP1         - Compute V · UP1 (out-of-plane projection scalar)
;      VXSC UP1        - Multiply scalar by UP1 to get V_perp vector
;      VSL2            - Shift left 2 bits (scaling adjustment)
;      BVSU            - Subtract V_perp from V to get V_plane (in-plane component)
;      UNIT            - Normalize V_plane to unit vector
;      PDVL            - Push unit vector and load original V magnitude
;      ABVAL           - Compute |V| (absolute value/magnitude)
;      VXSC            - Multiply unit vector by magnitude
;      VSL1            - Shift left 1 bit (final scaling)
;      RVQ             - Return via Q register
;
; INPUT:
;      Stack: Input vector V to be rotated (3 components)
;      UP1:   Unit perpendicular vector defining orbital plane normal
;
; OUTPUT:
;      Rotated vector with same magnitude but aligned to orbital plane
;
; CALLED BY:
;      ADVANCE - Twice per execution (for RACT and VACT transformations)
;
; This rotation ensures geometric consistency during rendezvous computations,
; particularly important when computing the CDH (Constant Delta Height) maneuver
; which must be applied in the correct orbital reference frame to achieve the
; desired altitude adjustment.
; ============================================================================

		SETLOC	CDHTAG
		BANK

ROTATE		PUSH	PUSH
		DOT	VXSC
			UP1
			UP1
		VSL2	BVSU
		UNIT	PDVL
		ABVAL	VXSC
		VSL1	RVQ
# Page 677
# ..... INTINTNA .....

; ============================================================================
; INTINT2C - INTEGRATION INITIALIZATION FOR CSI-TO-CDH INTERVAL
;
; PURPOSE:
;      Prepares parameters for numerical integration of orbital motion during
;      the coast phase between CSI (Coelliptic Sequence Initiation) and CDH
;      (Constant Delta Height) maneuvers.
;
; OPERATIONAL CONTEXT:
;      After CSI execution, the active vehicle coasts in its new orbit toward
;      the CDH maneuver point. This routine initializes the integration
;      parameters needed to accurately propagate the orbital state vectors
;      across this time interval, accounting for gravitational perturbations.
;
; PARAMETERS LOADED:
;      Stack position 1: TCSI  - Time of CSI ignition (start of interval)
;      Stack position 2: TCDH  - Time of CDH ignition (end of interval)
;      Stack position 3: TWOPI - 2π constant for angle normalization
;      Stack position 4: Pushed copy for internal use
;
; TWOPI PARAMETER:
;      The 2π constant is used during integration to normalize angular
;      parameters and handle periodic orbital elements (e.g., true anomaly,
;      argument of latitude) that wrap around at 360 degrees.
;
; GOTO INTINT:
;      Transfers control to the main integration initialization routine INTINT,
;      which uses these parameters to set up the Encke method integration for
;      precise orbit propagation during the CSI-to-CDH coast.
;
; This integration ensures that the CDH maneuver calculations operate on
; accurately propagated state vectors, accounting for orbital motion and
; perturbations during the coast phase.
; ============================================================================
;
; ============================================================================
; INTINT3P - INTEGRATION INITIALIZATION FOR CDH-TO-TPI INTERVAL
;
; PURPOSE:
;      Prepares parameters for numerical integration of orbital motion during
;      the coast phase between CDH (Constant Delta Height) and TPI (Terminal
;      Phase Initiation) maneuvers.
;
; OPERATIONAL CONTEXT:
;      After CDH execution, the active vehicle maintains constant altitude
;      difference with the target while coasting toward TPI. This routine
;      initializes integration parameters for propagating state vectors
;      through this final coast phase before terminal rendezvous begins.
;
; PARAMETERS LOADED:
;      Stack position 1: TCDH      - Time of CDH ignition (start of interval)
;      Stack position 2: TTPI      - Time of TPI ignition (end of interval)
;      Stack position 3: ZEROVECS  - Zero vector (no angle normalization)
;      Stack position 4: Pushed copy for internal use
;
; ZEROVECS PARAMETER:
;      Unlike INTINT2C which uses TWOPI for angle normalization, this routine
;      uses ZEROVECS, indicating that angular normalization is not required
;      for this integration interval. This may be because the CDH-to-TPI
;      interval is shorter or the integration method handles angles differently.
;
; GOTO INTINT:
;      Transfers control to the main integration initialization routine INTINT,
;      which sets up Encke method integration parameters appropriate for the
;      CDH-to-TPI coast phase.
;
; This integration ensures that TPI targeting computations operate on accurate
; state vectors at the terminal phase initiation point, accounting for orbital
; evolution during the final concentric rendezvous coast.
; ============================================================================

		SETLOC	CDHTAG2
		BANK

INTINT2C	PDDL	PDDL
			TCSI
			TCDH
		PDDL	PUSH
			TWOPI
		GOTO
			INTINT
INTINT3P	PDDL	PDDL
			TCDH
			TTPI
		PDDL	PUSH
			ZEROVECS
		GOTO
			INTINT
# Page 678
# ..... S32/33.1 .....

# SUBROUTINES USED
#	 S32/33.X

; ============================================================================
; S32/33.1 - DELTA-V DISPLAY AND COORDINATE TRANSFORMATION
;
; PURPOSE:
;      Displays computed delta-V components to the crew via DSKY and transforms
;      the delta-V vector from local vertical coordinates (LVC) into stable
;      member (inertial) coordinates for storage and use by the thrusting
;      program.
;
; OPERATIONAL CONTEXT:
;      After computing CSI or CDH maneuver delta-V in local vertical coordinates
;      (radial, in-track, cross-track), the crew must review and approve the
;      maneuver. This routine displays the delta-V components on the DSKY
;      (Verb 06 Noun 81) and then transforms the approved delta-V into the
;      stable member reference frame for execution.
;
; CREW INTERACTION:
;      VN 0681 - "Display Decimal (Load)" Verb 06, Noun 81
;                Displays DELVLVC components in local vertical coordinates:
;                R1: Radial component (perpendicular to orbital plane)
;                R2: In-track component (along velocity vector)
;                R3: Cross-track component (perpendicular to orbital plane)
;
;      The crew can review these values and either:
;        - Press PROCEED to accept the displayed delta-V
;        - Key in modified values and press ENTER to update DELVLVC
;        - Press RESET to abort the sequence
;
; COORDINATE TRANSFORMATION:
;      After crew approval, DELVLVC is transformed from local vertical
;      coordinates to stable member (inertial) coordinates:
;
;      DELVSIN = DELVLVC × 0D
;
;      Where 0D is the transformation matrix from LVC to inertial frame.
;      This transformation is essential because the Digital Autopilot (DAP)
;      commands engine gimbals in inertial coordinates, not local vertical.
;
; MAGNITUDE COMPUTATION:
;      DELVSAB = |DELVSIN|
;
;      The absolute value (magnitude) of the inertial delta-V is computed and
;      stored for use in burn time calculations and propellant consumption
;      estimates. This scalar value represents the total velocity change
;      regardless of direction.
;
; SUBROUTINE CALLS:
;      DISDVLVC - Display delta-V in local vertical coordinates (V06N81)
;      S32/33.X - Additional delta-V processing and validation
;
; OUTPUTS:
;      DELVSIN - Delta-V in stable member (inertial) coordinates
;      DELVSAB - Absolute magnitude of delta-V (scalar)
;
; This routine provides the critical interface between computed orbital
; mechanics (in local vertical frame) and spacecraft attitude control
; (in inertial frame), while ensuring crew awareness and approval of the
; planned maneuver.
; ============================================================================

		SETLOC	CSI/CDH
		BANK

S32/33.1	STQ	AXT,1
			SUBEXIT
		VN	0681
		CALL
			DISDVLVC
		CALL
			S32/33.X
		VLOAD	VXM
			DELVLVC
			0D
		VSL1
		STORE	DELVSIN
		PUSH	ABVAL
		STOVL	DELVSAB
		GOTO
			SUBEXIT
# Page 679
# ..... S32/33.X .....

; ============================================================================
; S32/33.X - CONSTRUCT LOCAL VERTICAL COORDINATE TRANSFORMATION MATRIX
;
; PURPOSE:
;      Builds an orthonormal coordinate transformation matrix (0D) that converts
;      vectors from local vertical coordinates (LVC) to stable member (inertial)
;      coordinates. This matrix is essential for transforming computed delta-V
;      values from the orbital reference frame into spacecraft body coordinates.
;
; OPERATIONAL CONTEXT:
;      Orbital mechanics computations naturally work in local vertical coordinates
;      where directions are defined relative to the orbital motion:
;        - Radial (perpendicular to surface)
;        - In-track (along velocity direction)
;        - Cross-track (perpendicular to orbital plane)
;
;      However, the spacecraft attitude control system and engine gimbals operate
;      in inertial (stable member) coordinates. This routine constructs the
;      rotation matrix that bridges these two reference frames.
;
; TRANSFORMATION MATRIX CONSTRUCTION:
;      The matrix 0D is built as an orthonormal basis with three column vectors:
;
;      Column 1: -RACT1_unit = Negative normalized position vector
;                This represents the "down" direction (toward Earth/Moon center)
;                in the local vertical frame.
;
;      Column 2: (-RACT1_unit) × UP1 = Cross-track direction
;                This vector is perpendicular to both the radial direction and
;                the orbital plane normal (UP1), defining the in-track direction
;                in the orbital plane.
;
;      Column 3: -UP1 = Negative orbital plane normal
;                This represents the cross-track direction, perpendicular to
;                the orbital plane.
;
; INTERPRETIVE INSTRUCTION SEQUENCE:
;      SETPD 6D        - Set push-down pointer to position 6
;      VLOAD UP1       - Load orbital plane normal vector
;      VCOMP           - Complement (negate) UP1 to get -UP1
;      PDVL RACT1      - Push -UP1 to stack and load position vector RACT1
;      UNIT            - Normalize RACT1 to unit vector
;      VCOMP           - Complement to get -RACT1_unit (radial down direction)
;      PUSH VXV UP1    - Push -RACT1_unit and compute cross product with UP1
;      VSL1            - Shift left 1 bit (scaling adjustment)
;      STORE 0D        - Store complete transformation matrix at 0D
;      RVQ             - Return via Q register
;
; MATHEMATICAL REPRESENTATION:
;      0D = [ -RACT1_unit | (-RACT1_unit × UP1) | -UP1 ]
;
;      This forms a right-handed orthonormal coordinate system aligned with
;      the local vertical reference frame at the spacecraft's current position.
;
; INPUT:
;      UP1   - Unit vector perpendicular to orbital plane (from ADVANCE)
;      RACT1 - Active vehicle position vector in inertial coordinates
;
; OUTPUT:
;      0D - 3×3 transformation matrix (stored as three column vectors)
;           Transforms vectors from LVC to inertial coordinates
;
; CALLED BY:
;      S32/33.1 - Before displaying and transforming delta-V components
;
; This transformation is fundamental to the rendezvous programs, ensuring that
; orbital mechanics computations performed in intuitive local vertical coordinates
; can be accurately executed by the spacecraft's inertial guidance system.
; ============================================================================

		SETLOC	CDHTAGS
		BANK

S32/33.X	SETPD	VLOAD
			6D
			UP1
		VCOMP	PDVL
			RACT1
		UNIT	VCOMP
		PUSH	VXV
			UP1
		VSL1
		STORE	0D
		RVQ
# Page 680
# ..... CDHMVR .....

# SUBROUTINES USED

#	 VECSHIFT
#	 TIMETHET
#	 SHIFTR1

; ============================================================================
; CDHMVR - CONSTANT DELTA HEIGHT MANEUVER COMPUTATION
;
; PURPOSE:
;      Calculates the delta-V required for the CDH (Constant Delta Height)
;      maneuver that establishes concentric circular orbits with the target
;      vehicle. This height adjustment burn minimizes altitude variation
;      between the active and passive vehicles' orbits.
;
; HEIGHT ADJUSTMENT COMPUTATION STRATEGY:
;      1. Compute current altitude difference (DIFFALT) between vehicles
;      2. Calculate semi-major axis (A SUB A) of active vehicle's orbit
;      3. Determine semi-major axis (A SUB P) of desired transfer orbit
;      4. Compute required velocity change to match target altitude
;      5. Generate delta-V vector (DELVEET2) in reference coordinates
;
; ORBITAL MECHANICS PRINCIPLES:
;      - CDH burn circularizes the active vehicle's orbit at target altitude
;      - Uses vis-viva equation: V = SQRT(MU(2/R - 1/A))
;      - Delta-V applied perpendicular to radius vector for height change
;      - Minimizes fuel consumption by optimizing transfer trajectory
;      - Establishes coplanar concentric orbits for terminal phase initiation
;
; ALTITUDE DIFFERENCE CALCULATION:
;      DIFFALT = |RACT2| - |RPASS2|
;      Where RACT2 = active vehicle position at CDH time
;            RPASS2 = passive vehicle position at CDH time
;      Result scaled B+29 (meters)
;
; TRANSFER ORBIT PARAMETERS:
;      - A SUB A: Semi-major axis of active vehicle's current orbit
;      - A SUB P: Semi-major axis of desired circular orbit at target altitude
;      - The CDH burn velocity change is computed to bridge these orbits
;
; INPUT:
;      RACT2 - Active vehicle position vector at CDH time
;      VACT2 - Active vehicle velocity vector at CDH time
;      RPASS2 - Passive vehicle position vector at CDH time
;      VPASS2 - Passive vehicle velocity vector at CDH time
;      UP1 - Unit perpendicular vector defining orbital plane
;
; OUTPUT:
;      DELVEET2 - Delta-V vector for CDH burn (reference coordinates)
;      VACT3 - Active vehicle velocity after CDH burn
;      DIFFALT - Current altitude difference between vehicles (meters, B+29)
;
; SUBROUTINES CALLED:
;      VECSHIFT - Coordinate transformation for position/velocity vectors
;      TIMETHET - Time-angle computation for orbital mechanics
;      SHIFTR1 - Bit shift normalization for precision maintenance
;
; RENDEZVOUS CONTEXT:
;      The CDH maneuver typically occurs 10+ minutes after CSI and establishes
;      the geometry for TPI (Terminal Phase Initiation). For Apollo missions,
;      this was a critical step in LM-CSM rendezvous after lunar ascent.
; ============================================================================

		SETLOC	CDHTAG
		BANK

; ============================================================================
; CDHMVR COMPUTATION - ALTITUDE DIFFERENCE AND TRANSFER GEOMETRY
;
; This section calculates the altitude difference between vehicles and
; establishes the geometric parameters for the CDH height adjustment burn.
; ============================================================================

CDHMVR		STQ	VLOAD		; Save return address, load active vehicle position
			SUBEXIT		; Return address storage
			RACT2		; Active vehicle position vector (B+29 meters)
		PUSH	UNIT		; Push RACT2, compute unit vector
		STOVL	UNVEC		; Store as UNVEC (unit radial vector for active), load RPASS2

; ============================================================================
; ALTITUDE DIFFERENCE GEOMETRY CALCULATION
;
; Computing the cosine (CSTH) and sine (SNTH) of the angle between the
; active and passive vehicle position vectors. This angle determines the
; geometric relationship between the two vehicles' orbital positions.
; ============================================================================

			RPASS2		; Passive vehicle position vector (B+29 meters)
		UNIT	DOT		; Compute unit vector of RPASS2, dot with UNVEC
			UNVEC		; Dot product gives cosine of angle between positions
		PUSH	SL1		; Push result, shift left 1 bit for precision
		STODL	CSTH		; Store as CSTH (cosine theta), load for further computation
		DSQ	PDDL		; Square CSTH, push to stack, load constant
			DP1/4		; Load 1/4 constant
		SR2	DSU		; Shift right 2 (divide by 4), subtract from squared cosine
		SQRT	SL1		; Take square root: SQRT(CSTH^2 - 1/4), shift left 1
		PDVL	VCOMP		; Push sine component, load and complement vector
		VXV			; Vector cross product computation
			RPASS2		; Cross VCOMP with RPASS2
		DOT	PDDL		; Dot product with UP1, push result
			UP1		; Unit perpendicular vector (orbital plane normal)
		SIGN	STADR		; Apply sign correction based on orbital geometry
		STOVL	SNTH		; Store as SNTH (sine theta), load RPASS2

; ============================================================================
; PASSIVE VEHICLE STATE VECTOR COORDINATE TRANSFORMATION
;
; Using VECSHIFT to transform passive vehicle state vectors into the
; coordinate frame needed for CDH delta-V calculation.
; ============================================================================

			RPASS2		; Passive vehicle position
		PDVL	CALL		; Push RPASS2, load VPASS2, call subroutine
			VPASS2		; Passive vehicle velocity vector
			VECSHIFT	; Transform to appropriate reference frame
		STOVL	VVEC		; Store transformed velocity vector
		CLEAR			; Clear RVSW flag
			RVSW		; Reverse switch flag (controls computation direction)
		STCALL	RVEC		; Store transformed position, call TIMETHET
			TIMETHET	; Compute time-angle relationship for orbital transfer
; ============================================================================
; ALTITUDE DIFFERENCE CALCULATION (DIFFALT)
;
; Computing the actual altitude difference between the active and passive
; vehicles. This value determines the magnitude of the CDH burn required
; to circularize the orbit at the target altitude.
;
; DIFFALT = |RACT2| - |RPASS2|  (in meters, scaled B+29)
; ============================================================================

		LXA,2	VSL*		; Load index register X2, variable shift left
			RTX2		; Return from TIMETHET stored in RTX2
			0,2		; Shift amount from X2
		STORE	18D		; Store result at 18D
		DOT	SL1R		; Dot product with UNVEC, shift left 1 with round
			UNVEC		; Unit vector of active vehicle radial direction
		PDVL	ABVAL		; Push result, load vector, compute absolute magnitude
					# 0D = V SUB PV (passive vehicle velocity magnitude)
		SL*	PDVL		; Shift left by variable amount, push, load vector
			0,2		; Shift amount from X2
# Page 681
			RACT2		; Active vehicle position vector
		ABVAL	PDDL		; Absolute magnitude (radial distance), push
					# 2D = LENGTH OF R SUB A (active vehicle radius)
		DSU			; Double precision subtract
			02D		; Subtract passive vehicle radius from active radius
		STODL	DIFFALT		; Store altitude difference (DELTA H IN METERS, B+29)

; ============================================================================
; SEMI-MAJOR AXIS CALCULATION (A SUB A)
;
; Computing the semi-major axis of the active vehicle's current orbit using
; the vis-viva equation: A = 1/(2/R - V²/MU)
; Where: R = orbital radius, V = orbital velocity, MU = gravitational parameter
;
; This determines the energy of the current orbit, which must be modified
; by the CDH burn to achieve the desired circular orbit at target altitude.
; ============================================================================

			R1A		; Load R1A (radius component for vis-viva)
		NORM	PDDL		; Normalize for precision, push, load R1
					# 2 - R V**/MU computation (vis-viva denominator)
			X1		; Normalization shift count stored in X1
			R1		; Radius value R1
		CALL			; Call precision shift routine
			SHIFTR1		; Maintain computational precision during division
		SR1R	DDV		; Shift right 1 with round, double precision divide
		SL*	PUSH		; Shift left by computed amount, push result
			0 -5,1		; Shift determined by X1 value minus 5
		DSU	PDDL		; Double subtract, push result
					# A SUB A (semi-major axis B+29, stored at 04D)

; ============================================================================
; TARGET ORBIT SEMI-MAJOR AXIS CALCULATION (A SUB P)
;
; Computing the semi-major axis of the desired circular orbit that matches
; the passive vehicle's altitude. This is the target orbit the CDH burn
; will achieve.
;
; A SUB P = A SUB A - (DIFFALT/2)
; Where DIFFALT/2 represents half the current altitude difference
; ============================================================================

			DIFFALT		; Load altitude difference between vehicles
		SR2	DDV		; Shift right 2 (divide by 4), double divide
					# A SUB P (target semi-major axis) B+31
			04D		; Divide by A SUB A stored at 04D
					#                                B+2
		PUSH	SQRT		; Push result, compute square root
					# A SUB P/A SUB A (orbit ratio)  06D
		DMPR	DMP		; Double precision multiply
			06D		; Multiply orbit ratio by itself
			00D		; Multiply by passive vehicle velocity component

; ============================================================================
; VELOCITY CHANGE COMPUTATION USING VIS-VIVA EQUATION
;
; The vis-viva equation relates orbital velocity to position and energy:
; V² = MU(2/R - 1/A)
; Where: V = orbital velocity, MU = gravitational parameter
;        R = current radius, A = semi-major axis
;
; This section computes the required velocity change (Delta-V) to transition
; from the current orbit (A SUB A) to the target circular orbit (A SUB P).
; The Delta-V is applied perpendicular to the position vector to achieve
; the height change with minimum fuel consumption.
; ============================================================================

		SL3R	PDDL		; Shift left 3 with round, push, load
					# V SUB A V METERS/CS B+7 (08D)
			02D		; Load R SUB A MAGNITUDE (B+29)
		NORM	PDDL		; Normalize for precision, push
			X1		; Store normalization count in X1
			RTMU		; Load RTMU (square root of MU)
		SR1	DDV		; Shift right 1, double precision divide
					# 2MU computation (B+38)
		SL*	PDDL		; Variable shift left, push
					# 2 MU/R SUBAA (B+14, stored at 10D)
			0 -5,1		; Shift determined by X1 minus 5
			04D		; Load ASUBA (A SUB A, B+29)
		NORM	PDDL		; Normalize for precision, push
			X2		; Store normalization count in X2
			RTMU		; Load RTMU again
		SR1	DDV		; Shift right 1, double divide
		SL*	BDSU		; Variable shift left, backwards subtract
			0 -6,2		; Shift determined by X2 minus 6
					# 2U/R - U/A (B+14, velocity squared units)
		PDDL	DSQ		; Push result, load and square
					#                                10D
			08D		; Load V SUB A V (velocity component)
		BDSU	SQRT		; Backwards subtract, take square root

; ============================================================================
; DELTA-V VECTOR CONSTRUCTION (DELVEET2)
;
; Constructs the three-dimensional delta-V vector in reference coordinates
; by combining radial and perpendicular components. The burn direction is
; optimized for the coplanar height adjustment.
;
; The resulting DELVEET2 vector represents the impulsive velocity change
; required at CDH ignition to circularize the orbit at target altitude.
; ============================================================================

		PDVL	VXV		; Push delta-V magnitude, load vectors, cross product
					# SQRT(MU(2/R SUB A-1/A SUB A)-VSUBA2) (10D)
			UP1		; Load UP1 (orbital plane unit perpendicular)
			UNVEC		; Cross with UNVEC (unit radial direction)
		UNIT	VXSC		; Compute unit vector, vector scalar multiply
			10D		; Multiply by delta-V magnitude from 10D
		PDVL	VXSC		; Push perpendicular component, load radial component
			UNVEC		; Unit radial vector
			08D		; Multiply by radial velocity component
		VAD	VSL1		; Vector add (combine components), shift left 1
		STADR			; Store address for result
# Page 682
		STORE	VACT3		; Store as VACT3 (active vehicle velocity after CDH)

; ============================================================================
; FINAL DELTA-V OUTPUT
;
; Computes the CDH delta-V by subtracting the current active vehicle
; velocity from the post-burn velocity. This is the commanded thrust
; vector for the CDH maneuver.
;
; DELVEET2 = VACT3 - VACT2
;
; This vector is stored in reference coordinates and will be used by the
; thrusting program (P40/P41) to execute the CDH burn. For Apollo missions,
; this burn typically occurred 10-20 minutes after CSI and established the
; final geometry for terminal phase initiation (TPI).
; ============================================================================

		VSU			; Vector subtract
			VACT2		; Subtract current active vehicle velocity
		STCALL	DELVEET2	; Store as DELVEET2 (CDH delta-V), call return
					# DELTA VCDH - REFERENCE COORDINATES
			SUBEXIT		; Return to calling routine
# Page 683
# ..... COMPTGO .....

# SUBROUTINES USED

#	 CLOKTASK
#	 2PHSCHNG

; ============================================================================
; COMPTGO - MEMORY BANK ALLOCATION FOR P32/P33/P72/P73 PROGRAMS
;
; PURPOSE:
;      This section establishes the memory bank location and erasable memory
;      bank (EBANK) assignment for the P32/P33/P72/P73 rendezvous programs.
;      These assembler directives do not generate executable code but instead
;      instruct the yaYUL assembler where to place code and which erasable
;      memory bank to use for data access.
;
; MEMORY ARCHITECTURE CONTEXT:
;      The Apollo Guidance Computer uses a banked memory architecture to extend
;      its addressing capability beyond the limited 15-bit address space:
;
;      FIXED MEMORY (ROM): 36K words organized into banks
;        - Bank switching allows access to code beyond direct addressing
;        - SETLOC and BANK directives control code placement
;
;      ERASABLE MEMORY (RAM): 2K words organized into banks
;        - EBANK directive specifies which erasable bank is active
;        - Allows efficient access to related data structures
;
; ASSEMBLER DIRECTIVES EXPLANATION:
;
;      BANK 35
;        Specifies that the following code should be assembled into fixed
;        memory bank 35. This bank number was chosen during the memory
;        allocation phase of program development to balance code distribution
;        across available ROM banks.
;
;      SETLOC CSI/CDH
;        Sets the location counter to the CSI/CDH section, grouping all
;        CSI (Coelliptic Sequence Initiation) and CDH (Constant Delta Height)
;        related code together for logical organization and efficient bank
;        usage.
;
;      BANK
;        Re-establishes the current bank context after the SETLOC directive.
;        This ensures subsequent code is placed in the correct fixed memory
;        bank as previously specified.
;
;      EBANK= RTRN
;        Sets the erasable memory bank to the bank containing the RTRN variable.
;        During execution of routines in this section, data references will
;        default to the erasable bank where RTRN resides, allowing efficient
;        access to related rendezvous program data structures without explicit
;        bank switching overhead.
;
;        The RTRN variable is part of the rendezvous targeting data structure
;        and serves as the anchor point for EBANK assignment, ensuring all
;        related variables (delta-V components, state vectors, targeting
;        parameters) are accessible within the same erasable bank.
;
;      COUNT* $$/P3575
;        Establishes a counting label for this code section used by the
;        assembler's cross-reference generation. The label $$/P3575 associates
;        this code with the P32-P35 and P72-P75 program family for documentation
;        and debugging purposes.
;
; SUBROUTINES REFERENCED:
;      CLOKTASK - Mission elapsed time task scheduling
;      2PHSCHNG - Two-phase restart protection table update
;
;      These subroutines are called by the P32/P33/P72/P73 programs but are
;      defined elsewhere in the AGC codebase. The comments here document the
;      dependency for cross-reference purposes.
;
; OPERATIONAL SIGNIFICANCE:
;      Proper memory bank allocation is critical for the rendezvous programs:
;
;      1. PERFORMANCE: Minimizes bank switching overhead during time-critical
;         rendezvous computations. Frequent bank switches consume CPU cycles
;         and can affect guidance update rates.
;
;      2. RESTART PROTECTION: Grouping related code in the same bank simplifies
;         restart protection logic. If a power transient occurs during a
;         rendezvous maneuver, the restart system can more efficiently restore
;         program state when code is logically organized.
;
;      3. MEMORY EFFICIENCY: Careful bank allocation ensures efficient use of
;         the AGC's limited 36K words of ROM. Rendezvous programs are complex
;         and require substantial code space; proper allocation prevents bank
;         overflow and fragmentation.
;
; HISTORICAL CONTEXT:
;      During Apollo 11's rendezvous of Eagle (LM) with Columbia (CM) after
;      lunar ascent, these programs executed from bank 35, accessing targeting
;      data from the RTRN erasable bank. The bank allocation decisions made
;      during program development in 1968-1969 reflected careful optimization
;      of the AGC's constrained memory resources.
;
; This section represents the culmination of the P32/P33/P72/P73 source file,
; establishing the memory architecture foundation upon which all preceding
; rendezvous computation routines operate.
; ============================================================================

		BANK	35
		SETLOC	CSI/CDH
		BANK

		EBANK=	RTRN

		COUNT*	$$/P3575
