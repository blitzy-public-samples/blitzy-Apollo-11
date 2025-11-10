# Copyright:	Public domain.
# Filename:	P20-P25.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Pages:	0492-0613
# Mod history:	2009-05-26 OH	Transcribed from page images.
#		2009-06-05 RSB	Corrected a typos.
#		2009-06-06 RSB	Added a missing instruction, and a block
#				of 3 missing instructions.
#		2009-06-07 RSB	Fixed a misprint.
#		2009-06-07 RSB	Changed the construct "2DEC E-6 B12"
#				(which isn't legal in yaYUL) to
#				"2DEC 1.0 E-6 B12".
#		2011-05-07 JL	Removed workarounds.

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

# ============================================================================
# FILE: P20-P25.agc
# MODULE: Rendezvous Navigation and Tracking
# MISSION PHASE: lunar-orbit/ascent/rendezvous
#
# TL;DR: Complete rendezvous navigation suite managing LM-to-CSM relative
#        state estimation and targeting during lunar orbit rendezvous operations.
#        Programs P20 (rendezvous navigation) and P25 (preferred tracking
#        attitude) coordinate rendezvous radar tracking, relative navigation
#        state updates, and targeting for the three-maneuver rendezvous sequence
#        (CSI/CDH/TPI) that brought Eagle to within feet of Columbia for docking.
#
# COMMENT-ONLY READERS: This is the navigation brain of the rendezvous. After
#        Eagle's ascent from the lunar surface on July 21, 1969, these programs
#        managed the 3-hour 40-minute chase to dock with Columbia. Follow the
#        comments to understand how the AGC locked onto Columbia at 100+ miles
#        range and computed the precise maneuvers for rendezvous.
#
# CODE-ALONG READERS: Study the integration of rendezvous radar data processing,
#        Kalman filtering for navigation state updates, Lambert targeting
#        algorithms, and automatic attitude control. Note the interplay between
#        P20 (navigation), P25 (attitude), supporting routines R21-R24 (radar
#        management), and R61 (preferred tracking attitude computation).
# ============================================================================

; ============================================================================
; FILE: P20-P25.agc
; MODULE: Rendezvous Navigation Programs
; MISSION PHASE: lunar-orbit/ascent/rendezvous
;
; TL;DR: Complete rendezvous navigation suite managing LM-to-CSM relative
;        state estimation, radar tracking, and targeting computations during
;        lunar orbit operations. Programs P20-P25 coordinate rendezvous radar
;        data acquisition, orbital navigation updates, and maneuver targeting
;        calculations. Critical for Eagle-Columbia rendezvous after lunar
;        surface ascent on July 21, 1969.
;
; COMMENT-ONLY READERS: After Eagle's ascent from the lunar surface, these
;        programs guided the spacecraft through three precision maneuvers over
;        3 hours 40 minutes to rejoin Columbia. Read comments to follow the
;        automated rendezvous radar tracking and navigation that brought the
;        two spacecraft to within feet of each other for final docking.
;
; CODE-ALONG READERS: Examine the integration of rendezvous radar measurements
;        with Kalman filtering algorithms, Lambert targeting computations, and
;        relative state vector propagation. Study how P20-P25 coordinate radar
;        designation, data reading, navigation updates, and display outputs.
; ============================================================================

# Page 492
# RENDEZVOUS NAVIGATION PROGRAM 20
#
# PROGRAM DESCRIPTION
#
# 	MOD NO -- 2
# 	BY P. VOLANTE
#
# FUNCTIONAL DESCRIPTION
#
# 	THE PURPOSE OF THIS PROGRAM IS TO CONTROL THE RENDEZVOUS RADAR FROM
# 	STARTUP THROUGH ACQUISITION AND LOCKON TO THE CSM AND TO UPDATE EITHER
# 	THE LM OR CSM STATE VECTOR (AS SPECIFIED BY THE ASTRONAUT BY DSKY ENTRY)
# 	ON THE BASIS OF THE RR TRACKING DATA.
#
# CALLING SEQUENCE --
#
# 	ASTRONAUT REQUEST THROUGH DSKY V37E20E
#
# SUBROUTINES CALLED
#
#	R02BOTH	(IMU STATUS CHECK)			FLAGUP
#	GOFLASH	(PINBALL-DISPLAY)			FLAGDOWN
#	R23LEM	(MANUAL ACQUISITION)			BANKCALL
#	LS201	(LOS DETERMINATION)			TASKOVER
#	LS202	(RANGE LIMIT TEST)
#	R61LEM	(PREFERRED TRACKING ATTITUDE)
#	R21LEM	(RR DESIGNATE)				ENDOFJOB
#	R22LEM	(DATA READ)				GOPERF1
#	R31LEM	(RENDEZVOUS PARAMETER DISPLAY)
#	PRIOLARM (PRIORITY DISPLAY)
#
# NORMAL EXIT MODES --
#
# 	P20 MAY BE TERMINATED IN TWO WAYS -- ASTRONAUT SELECTION OF IDLING
#	PROGRAM (P00) BY KEYING V37E00E OR BY KEYING IN V56E
#
# ALARM OR ABORT EXIT MODES --
#
# 	RANGE GREATER THAN 400 NM DISPLAY
#
# OUTPUT
#
# 	TRKMKCNT = NO OF RENDEZVOUS TRACKING MARKS TAKEN (COUNTER)
#
# ERASABLE INITIALIZATION REQUIRED
#
# FLAGS SET + RESET
#
#	SRCHOPT, RNDVZFLG, ACMODFLG, VEHUPFLG, UPDATFLG, TRACKFLG
#
# DEBRIS
#
#	CENTRALS -- A,Q,L

# ============================================================================
# RENDEZVOUS NAVIGATION: APOLLO 11 HISTORICAL CONTEXT
#
# After Eagle's ascent from the lunar surface on July 21, 1969, at mission
# time 124:22:00, Armstrong and Aldrin faced a critical challenge: locating
# and rendezvousing with Columbia, piloted by Michael Collins, who had been
# orbiting the Moon alone for over 27 hours.
#
# P20 managed the entire rendezvous sequence:
# - Rendezvous radar acquired Columbia at approximately 100 nautical miles range
# - Radar tracked Columbia's position and velocity relative to Eagle
# - Navigation state vectors were continuously updated with radar measurements
# - Three rendezvous maneuvers were computed: CSI (Coelliptic Sequence
#   Initiation), CDH (Constant Delta Height), and TPI (Terminal Phase Initiation)
# - Final docking occurred at 128:03:00 mission time, 3 hours 41 minutes after
#   ascent, bringing the two spacecraft to within inches for successful docking
#
# The rendezvous radar, mounted on Eagle's forward equipment bay, provided
# range and range-rate measurements using a conical scanning antenna. These
# measurements were processed through Kalman filtering algorithms to refine
# the relative state vector, accounting for measurement noise and orbital
# dynamics uncertainties.
# ============================================================================

		SBANK=	LOWSUPER	# FOR LOW 2CADR'S.

		BANK	33
		SETLOC	P20S
		BANK

		EBANK=	LOSCOUNT
		COUNT*	$$/P20

# ============================================================================
# P20 PROGRAM ENTRY POINT
#
# Astronaut initiates P20 via DSKY entry: V37 E 20 E
#
# P20 begins by determining the LM's current state:
# - If on lunar surface: Enters orbital change mode (ORBCHGO) to compute CSM
#   state vector at estimated launch time for post-ascent rendezvous planning
# - If in orbit: Continues with standard rendezvous radar tracking sequence
#
# PROG22 is an alias for PROG20, allowing either P20 or P22 designation for
# the same rendezvous navigation functionality.
# ============================================================================

PROG22		=	PROG20
PROG20		TC	2PHSCHNG
		OCT	4
		OCT	05022
		OCT	26000		# PRIORITY 26

; The guidance computer first determines if the LM is sitting on the lunar
; surface or already in orbit. This check is critical because pre-ascent
; planning requires different computations than active orbital rendezvous.

		TC	LUNSFCHK	# CHECK IF ON LUNAR SURFACE
# Page 493
		TC	ORBCHGO		# YES -- LM ON SURFACE, PLAN RENDEZVOUS
		TC	PROG20A -2	# NO -- LM IN ORBIT, ACTIVE TRACKING
# ============================================================================
# ORBITAL CHANGE MODE (PRE-ASCENT RENDEZVOUS PLANNING)
#
# When the LM is on the lunar surface before ascent, this routine computes
# where the CSM will be at the estimated launch time. This allows the crew
# to verify that the launch window provides favorable rendezvous geometry.
#
# During Apollo 11, this mode was used on the lunar surface between landing
# (July 20, 102:45:40) and ascent (July 21, 124:22:00). Armstrong and Aldrin
# could verify Columbia's predicted position at their planned liftoff time.
# ============================================================================

ORBCHGO		TC	UPFLAG		# SET VEHUPFLG -- CSM STATE
		ADRES	VEHUPFLG	# VECTOR TO BE UPDATED

; The crew is prompted to confirm the assumption that the CSM will not
; perform any orbit-changing maneuvers before LM ascent. Option 2 (R2=1)
; means "CSM orbit unchanged."

		CAF	ONE		# SET R2 FOR OPTION CSM WILL NOT
		TS	OPTION2		# CHANGE PRESENT ORBIT
		CAF	OCT00012
		TC	BANKCALL	# DISPLAY ASSUMED CSM ORBIT OPTION
		CADR	GOPERF4
		TC	GOTOPOOH	# TERMINATE
		TC	ORBCHG1		# PROCEED VALUE OF ASSUMED OPTION OK
		TC	-5		# R2 LOADED THRU DSKY

; If the crew has confirmed that no CSM orbital changes are planned, the
; computer requests the estimated time of launch (TIG - Time of Ignition).

ORBCHG1		CS	P22ONE
		AD	OPTION2
		EXTEND
		BZF	PROG20A
		CAF	V06N33*
		TC	BANKCALL	# FLASH VERB-NOUN TO REQUEST ESTIMATED
		CADR	GOFLASH		# TIME OF LAUNCH
		TC	GOTOPOOH	# TERMINATE
		TC	ORBCHG2		# PROCEED VALUES OK
		TC	-5		# TIME LOADED THRU DSKY
; Once the estimated time of ignition (TIG) is entered, the computer begins
; conic extrapolation to compute where both spacecraft will be at that time.

ORBCHG2		TC	INTPRET
		GOTO
			ORBCHG3
	 	BANK	32
		SETLOC	P20S4
		BANK
		COUNT*	$$/P20

# ============================================================================
# CONIC EXTRAPOLATION TO ESTIMATED LAUNCH TIME
#
# This section performs numerical integration (using interpretive code) to
# propagate both the LM and CSM state vectors forward to the estimated launch
# time. The computation determines:
# - Where the CSM will be when the LM lifts off
# - The geometry of the rendezvous problem (plane change required, etc.)
# - Terminal conditions for the rendezvous trajectory
#
# The integration uses Encke's method for precision orbital mechanics,
# accounting for lunar gravitational perturbations.
# ============================================================================

ORBCHG3		CALL
			INTSTALL

; Load estimated launch time and prepare for LM state vector integration.
; The LM position at launch time is needed to determine the initial geometry
; of the rendezvous trajectory and compute the required plane change.

		DLOAD
			TIG
		STORE	LNCHTM
		STORE	TDEC1		# ESTIMATED LAUNCH TIME
		CLEAR	CLEAR
			VINTFLAG	# LM INTEGRATION (CLEAR=LM, SET=CSM)
			INTYPFLG	# PRECISION -- ENCKE (CLEAR=ENCKE)
		CLEAR	CLEAR
			DIM0FLAG	# NO W-MATRIX (STATE TRANSITION MATRIX)
			D6OR9FLG
		CALL
			INTEGRV		# PLANETARY INERTIAL ORIENTATION
		CALL
			GRP2PC		# CONVERT TO PREFERRED COORDINATES
		VLOAD
			RATT1
		STODL	RSUBL		# SAVE LM POSITION
			TAT
# Page 494
		STCALL	TDEC1
			INTSTALL

; Now integrate the CSM state vector to the same launch time. This determines
; where Columbia will be when Eagle lifts off. The W-matrix (state transition
; matrix) is optionally computed if the RENDWFLG flag indicates it's valid and
; needed for covariance propagation in the navigation filter.

		SET	CLEAR
			VINTFLAG	# CSM INTEGRATION (SET=CSM)
			INTYPFLG
		CLEAR	BOFF
			DIM0FLAG
			RENDWFLG	# W MATRIX VALID
			NOWMATX		# NO
		SET	SET		# YES -- SET FOR W MATRIX
			DIM0FLAG
			D6OR9FLG
NOWMATX		CALL
			INTEGRV		# CSM INTEGRATION
		CALL
			GRP2PC
		VLOAD
			VATT1
		STOVL	VSUBC		# SAVE CSM VELOCITY
			RATT1
		STORE	RSUBC		# SAVE CSM POSITION

; With both spacecraft positions known at launch time, compute the orbital
; plane change angle required for rendezvous. The plane change is the angle
; between the CSM orbital plane and the LM's position vector at launch.
; This geometry determines the out-of-plane velocity component needed.

; Compute the orbital plane geometry needed for rendezvous. The CSM orbital
; plane is defined by its position and velocity vectors. The plane change angle
; represents the angular separation between orbital planes that must be overcome.

		VXV	UNIT		# COMPUTE NORMAL TO CSM ORBITAL PLANE
			VSUBC		# NSUB1=UNIT(R(CM) CROSS V(CM))
		STOVL	20D		# SAVE NSUB1
			RSUBL		# COMPUTE ESTIMATED ORBITAL
		VXV	UNIT		# PLANE CHANGE
			20D		# UCSM = UNIT(R(LM) CROSS NSUB1)
		STOVL	UCSM
			RSUBC		# COMPUTE ANGLE BETWEEN UCSM
		UNIT	DOT		# AND RSUBC
			UCSM		# COS A = UCSM DOT UNIT (R(CM))
		SL1
		STORE	CSTH		# SAVE FOR TIME-THETA SUBROUTINE

; With the cosine of the plane change angle computed, derive the sine using
; the Pythagorean identity: sin² + cos² = 1. Both values feed the TIME-THETA
; subroutine which computes orbital transfer time and terminal conditions.

		DSQ	BDSU		# COMPUTE SINE A = SQRT(1 - COS^2 A)
			ONEB-2
		SQRT
		STOVL	SNTH		# SAVE FOR TIME-THETA SUBROUTINE
			RSUBC		# POSITION OF CSM AT EST. LAUNCH
		STOVL	RVEC		# TIME FOR TIME-THETHA	B-27
			VSUBC		# VELOCITY OF CSM AT EST. LAUNCH.
		VCOMP
		STORE	VVEC		# TIME FOR TIME THETA 	B-5
; Call TIME-THETA subroutine to solve the orbital transfer problem. Given
; the CSM state and plane change geometry, TIME-THETA computes the transfer
; trajectory, time required, and terminal velocity at rendezvous intercept.

		CLEAR	CALL
			RVSW
			TIMETHET
		VCOMP
		STORE	NEWVEL		# TERMINAL VELOCITY OF CSM
		DLOAD
			T
		STOVL	TRANSTM		# TRANSFER TIME
# Page 495

; The TIME-THETA solution provides the terminal conditions where the LM will
; intercept the CSM. Rotate the terminal velocity vector into the desired
; orbital plane to account for the out-of-plane component of the rendezvous.

			NEWVEL
		ABVAL
		STOVL	20D
			0D
		STORE	NEWPOS		# TERMINAL POSITION OF CSM
		VXV	UNIT		# COMPUTE NORMAL TO SCM ORBITAL PLANE
			RSUBL		# NSUB2 = UNIT(NEWPOS CROSS R(LM))
		VXV	UNIT		# ROTATE TERMINAL VEL INTO DESIRED
			NEWPOS		# ORBITAL PLANE
		VXSC	VSL1		# VSUBC = ABVAL(NEWVEL) $ UNIT (NSUB2
			20D
		STCALL	NCSMVEL		# NEW CSM VELOCITY
			GRP2PC
; Initialize orbital integration to propagate the CSM state vector forward
; to the computed intercept time. This provides accurate position/velocity
; at the moment when the rendezvous radar should acquire lock on Columbia.

		CALL
			INTSTALL
		DLOAD	BDSU
			TRANSTM		# LAUNCH TIME -- TRANSFER TIME
			LNCHTM
		STOVL	TET
			NEWPOS
		STORE	RCV
		STOVL	RRECT
			NCSMVEL
		STCALL	VRECT
			MINIRECT
		AXT,2	CALL
			2
			ATOPCSM
		CALL
			INTWAKE0
		EXIT
		TC	BANKCALL
		CADR	PROG20A

		BANK	24
		SETLOC	P20S
		BANK
		COUNT*	$$/P20

; ============================================================================
; TRANSITION: From orbital mechanics computation to rendezvous radar operations
;
; With the CSM intercept conditions computed, P20 now configures the system
; for radar tracking. Flag management enables navigation state updates while
; ensuring radar acquisition proceeds in automatic mode with proper monitoring.
; ============================================================================

		TC	DOWNFLAG	# RESET VEHUPFLG -- LM STATE VECTOR
		ADRES	VEHUPFLG	# TO BE UPDATED
PROG20A		TC	BANKCALL
		CADR	R02BOTH
		TC	UPFLAG
		ADRES	UPDATFLG	# SET UPDATE FLAG
		TC	UPFLAG
		ADRES	TRACKFLG	# SET TRACK FLAG
		TC	UPFLAG
		ADRES	RNDVZFLG	# SET RENDEZVOUS FLAG
		TC	DOWNFLAG
		ADRES	SRCHOPTN	# INSURE SEARCH OPTION OFF
# Page 496
		TC	DOWNFLAG	# ALSO MANUAL ACQUISITION FLAG RESET
		ADRES	ACMODFLG
		TC	DOWNFLAG	# TURN OFF R04FLAG TO ENSURE GETTING
		ADRES	R04FLAG		# ALARM 521 IF CAN'T READ RADAR
		TC	DOWNFLAG	# ENSURE R25 GIMBAL MONITOR IS ENABLED
		ADRES	NORRMON		# (RESET NORRMON FLAG)
		TC	DOWNFLAG	# RESET LOS BEING COMPUTED FLAG
		ADRES	LOSCMFLG
		TC	CLRADMOD
; Begin the rendezvous radar acquisition sequence. First, compute the line-of-
; sight (LOS) vector from the LM to the CSM based on the current state vectors.
; Then verify range is within the 400 nautical mile operational limit of the
; rendezvous radar. During Apollo 11, the radar acquired Columbia at ~100 nm.

P20LEM1		TC	PHASCHNG
		OCT	04022
		CAF	ZERO		# ZERO MARK COUNTER
		TS	MARKCTR
		TC	INTPRET		# LOS DETERMINATION ROUTINE
		RTB
			LOADTIME
		STCALL	TDEC1
			LPS20.1
		CALL
			LPS20.2		# TEST RANGE R/UTINE
		EXIT
		INDEX	MPAC
		TC	+1
		TC	P20LEMA		# NORMAL RETURN WITHIN 400 N M

; If range exceeds 400 nautical miles, the rendezvous radar cannot acquire
; the CSM. Display alarm 526 and allow the crew to recycle when closer.

526ALARM	CAF	ALRM526		# ERROR EXIT -- RANGE > 400 N. MI.
		TC	BANKCALL
		CADR	PRIOLARM
		TC	GOTOV56		# TERMINATE EXITS P20 VIA V56 CODING
		TC	-4		# PROC (ILLEGAL)
		TC	P20LEM1		# ENTER RECYCLE
		TC	ENDOFJOB

; Range is acceptable. Check if P22 lunar surface flag is set, which would
; indicate surface operations requiring different radar procedures. Then call
; R61 to compute the preferred tracking attitude that optimizes radar pointing.

P20LEMA		TC	PHASCHNG
		OCT	04022
		TC	LUNSFCHK	# CHECK LUNAR SURFACE FLAG (P22 FLAG)
		TC	P20LEMB
		TC	BANKCALL
		CADR	R61LEM		# PREFERRED TRACKING ATTITUDE ROUTINE

; ============================================================================
; TRANSITION: From initialization to radar acquisition mode selection
;
; With the CSM position computed and preferred attitude determined, P20 now
; enters the radar acquisition loop. The program continuously monitors the
; rendezvous radar mode (automatic vs manual) and coordinates with the crew
; to achieve radar lock-on to Columbia. This section handles both automatic
; designation (R21) and manual acquisition (R23) paths.
; ============================================================================

P20LEMB		TC	PHASCHNG
		OCT	05022		# RESTART AT PRIORITY 10 TO ALLOW V37
		OCT	10000		# REQUESTED PROGRAM TO RUN FIRST
		CAF	PRIO26		# RESTORE PRIORITY 26
		TC	PRIOCHNG
		CA	FLAGWRD1	# IS THE TRACK FLAG SET
		MASK	TRACKBIT
		EXTEND
		BZF	P20LEMWT	# BRANCH -- NO -- WAIT FOR IT TO BE SET

; Check if the rendezvous radar is in automatic mode by reading the AUTO MODE
; discrete from channel 33. If in AUTO, proceed to automatic acquisition (R21).
; If not in AUTO, request crew to select AUTO mode via PLEASE PERFORM display.

P20LEMB7	CAF	BIT2		# IS RR AUTO MODE DISCRETE PRESENT
		EXTEND
# Page 497
		RAND	CHAN33
		EXTEND
		BZF	P20LEMB3	# YES -- DO AUTOMATIC ACQUISITION (R21)

P20LEMB5	CS	OCT24		# RADAR NOT IN AUTO CHECK IF
; Radar not in AUTO mode. Verify we're in P20 or P22 before displaying PLEASE
; PERFORM requesting AUTO mode. If radar goes out of AUTO during tracking,
; display alarm 514 indicating radar mode problem.

		AD	MODREG		# MAJOR MODE IS 20
		EXTEND
		BZF	P20LEMB6	# BRANCH -- YES -- OKAY TO DO PLEASE PERFORM

		AD	NEG2		# ALSO CHECK FOR P22
		EXTEND
		BZF	P20LEMB6	# BRANCH -- YES OK TO DO PLEASE PERFORM
		CAF	ALRM514		# TRACK FLAG SET -- FLASH PRIORITY ALARM 514 --
		TC	BANKCALL	# RADAR GOES OUT OF AUTO MODE WHILE IN USE
		CADR	PRIOLARM
		TC	GOTOV56		# TERMINATE EXITS VIA V56
		TC	P20LEMB		# PROCEED AND ENTER BOTH GO BACK
		TC	P20LEMB		# TO CHECK AUTO MODE AGAIN
		TC	ENDOFJOB

; Display PLEASE PERFORM V50N25 requesting crew to select RR AUTO mode switch.
; Crew responses: TERMINATE ends program via V56, PROCEED rechecks AUTO mode,
; ENTER initiates manual acquisition (R23) where crew manually points radar.

P20LEMB6	CAF	OCT201		# REQUEST RR AUTO MODE SELECTION
		TC	BANKCALL
		CADR	GOPERF1
		TC	GOTOV56		# TERMINATE EXITS P20 VIA V56 CODING
		TC	P20LEMB		# PROCEED CHECKS AUTO MODE DISCRETE AGAIN
		TC	LUNSFCHK	# ENTER INDICATES MANUAL ACQUISITION (R23)
		TC	P20LEMB2	# YES -- R23 NOT ALLOWED -- TURN ON OPR ERROR
		TC	R23LEM		# NO -- DO MANUAL ACQUISITION

; Return from R23 manual acquisition. Set ACMODFLG indicating manual mode was
; used, then return to check AUTO mode again for subsequent tracking marks.

P20LEMB1	TC	UPFLAG		# RETURN FROM R23 -- LOCKON ACHIEVED
		ADRES	ACMODFLG	# SET MANUAL FLAG AND GO BACK TO CHECK
		TC	P20LEMB		# RR AUTO MODE

; R23 manual acquisition not allowed in current mode. Light operator error
; lamp on DSKY and return to AUTO mode check loop.

P20LEMB2	TC	FALTON		# TURNS ON OPERATOR ERROR LIGHT ON DSKY
		TC	P20LEMB		# AND GOES BACK TO CHECK AUTO MODE

; Radar is in AUTO mode. Check if RR CDUs (Coupling Data Units - gimbal angle
; readouts) are being zeroed during radar initialization. If zeroing complete,
; proceed with automatic acquisition. If search or manual flags are set from
; previous operation, clear them and transition to automatic mode.

P20LEMB3	CS	RADMODES	# ARE RR CDUS BEING ZEROED
		MASK	RCDU0BIT
		EXTEND
		BZF	P20LEMB4	# BRANCH -- YES -- WAIT
		CAF	BIT13-14	# IS SEARCH OR MANUAL ACQUISITION FLAG SET
		MASK	FLAGWRD2
		EXTEND
		BZF	P20LEMC3	# ZERO MEANS AUTOMATIC RR ACQUISITION
		TC	DOWNFLAG	# RESET TO AUTO MODE
		ADRES	SRCHOPTN
# Page 498
		TC	DOWNFLAG
		ADRES	ACMODFLG
		TC	P20LEMWT	# WAIT 2.5 SECONDS THEN GO TO RR DATA READ

; RR CDUs still being zeroed. Wait 2.5 seconds then recheck. Zeroing occurs
; during radar power-up and mode transitions to establish gimbal reference.

P20LEMB4	CAF	250DEC
		TC	BANKCALL	# WAIT 2.5 SECONDS WHILE RR CDUS ARE BEING
		CADR	DELAYJOB	# ZEROED -- THEN GO BACK AND CHECK AGAIN
		TC	P20LEMB3

; Begin automatic RR acquisition sequence. Load current time and call LPS20.3
; to compute radar antenna pointing angles to designate the CSM position.

P20LEMC3	TC	INTPRET
		RTB
			LOADTIME
		STCALL	TDEC1
			UPPSV

; Update state vectors to current time, then exit interpreter to check flags.

P20LEMC4	EXIT

; Main tracking loop entry point. Verify RNDVZFLG and TRACKFLG are still set,
; indicating rendezvous operations are active and radar tracking is enabled.
; If TRACKFLG is set, call R21LEM (automatic radar designation routine).
; If not set, wait 15 seconds before rechecking to allow radar lock-on time.

P20LEMC		TC	PHASCHNG
		OCT	04022
		CAE	FLAGWRD0	# IS THE RENDEZVOUS FLAG SET
		MASK	RNDVZBIT
		EXTEND
		BZF	ENDOFJOB	# NO -- EXIT P20
		CAE	FLAGWRD1	# IS TRACK FLAG SET (BIT 5 FLAGWORD 1)
		MASK	TRACKBIT
		EXTEND
		BZF	P20LEMD		# BRANCH -- TRACK FLAG NOT ON -- WAIT 15 SECONDS

; TRACKFLG is set, indicating radar lock-on achieved. Call R21LEM to command
; radar antenna angles based on computed LOS (line-of-sight) to CSM.

P20LEMF		TC	R21LEM

; After R21 commands radar angles, wait 2.5 seconds using TWIDDLE (a same-bank
; waitlist alternative) before proceeding to P20LEMC1 which will schedule R22
; radar data read job. Check TRACKFLG is still set during wait.

P20LEMWT	CAF	250DEC
		TC	TWIDDLE		# USE INSTEAD OF WAITLIST SINCE SAME BANK
		ADRES	P20LEMC1	# WAIT 2.5 SECONDS
		CAE	FLAGWRD1	# IS TRACK FLAG SET
		MASK	TRACKBIT
		EXTEND
		BZF	ENDOFJOB	# NO -- EXIT WITHOUT DOING 2.7 PHASE CHANGE
P20LMWT1	TC	PHASCHNG
		OCT	40072
		TC	ENDOFJOB

; After 2.5 second delay, check flags and schedule R22 radar data read job if
; tracking is active. RNDVZFLG must be set (rendezvous operations ongoing) and
; TRACKFLG must be set (radar lock-on achieved) before scheduling R22.

P20LEMC1	CAE	FLAGWRD0	# IS RENDEZVOUS FLAG SET
		MASK	RNDVZBIT
		EXTEND
		BZF	TASKOVER	# NO -- EXIT P20/R22
		CAE	FLAGWRD1	# IS TRACK FLAG SET
		MASK	TRACKBIT
		EXTEND
		BZF	P20LEMC2	# NO -- DON'T SCHEDULE R22 JOB
# Page 499
		CAF	PRIO26		# YES -- SCHEDULE R22 JOB (RR DATA READ)
		TC	FINDVAC
		EBANK=	LOSCOUNT
		2CADR	R22LEM42

		TC	TASKOVER

; TRACKFLG not set. Wait 15 seconds and recheck. This allows time for radar
; lock-on circuitry to establish tracking before attempting data read.

P20LEMC2	TC	FIXDELAY	# TRACK FLAG NOT SET, WAIT 15 SECONDS
		DEC	1500		# AND CHECK AGAIN

		TC	P20LEMC1

; Wait loop when TRACKFLG not initially set. Poll TRACKFLG every 15 seconds
; until radar lock-on is achieved, then schedule R21 designation job.

P20LEMD		CAF	1500DEC
		TC	TWIDDLE		# WAITLIST FOR 15 SECONDS
		ADRES	P20LEMD1
		TC	ENDOFJOB

P20LEMD1	CAE	FLAGWRD1	# IS TRACK FLAG SET
		MASK 	TRACKBIT
		CCS	A
		TCF	P20LEMD2	# YES -- SCHEDULE DESIGNATE JOB
		TC	FIXDELAY	# NO -- WAIT 15 SECONDS
		DEC	1500
		TC	P20LEMD1

; TRACKFLG is now set. Schedule priority 26 job to perform R21 (radar
; designation) followed by permanent state vector integration at P20LEMC3.

P20LEMD2	CAF	PRIO26		# SCHEDULE JOB TO DO R21
		TC	FINDVAC
		EBANK=	LOSCOUNT
		2CADR	P20LEMC3	# START AT PERM.  MEMORY INTEGRATION

		TC	TASKOVER

; Constants for P20-P25 programs.
; 250DEC = 2.5 seconds delay (in centiseconds) between R21 and R22 calls.
; ALRM526 = Alarm code for range > 400 NM.
; ALRM514 = Alarm code for radar designation failure.
; MAXTRIES = Maximum number of radar lock-on attempts (60 tries).

250DEC		DEC	250
ALRM526		OCT	00526
OCT201		OCT	00201
ALRM514		OCT	514
MAXTRIES	DEC	60
OCT00012	OCT	00012
P22ONE		OCT	00001
ONEB-2		2DEC	1.0 B-2

V06N33*		VN	0633

; ============================================================================
; UPPSV -- UPDATE PERMANENT STATE VECTORS
;
; Updates both LM and CSM state vectors to current time using orbital
; integration. Includes W-matrix (navigation covariance) integration when
; valid. Handles both vehicles: integrates one with W-matrix, then integrates
; the other without W-matrix. Vehicle selection controlled by VEHUPFLG.
; Critical for maintaining accurate navigation state during rendezvous.
; ============================================================================

UPPSV		STQ	CALL		# UPDATES PERMANENT STATE VECTORS
			LS21X		# 	TO PRESENT TIME
			INTSTALL
		CALL
# Page 500
			SETIFLGS
		BOF	SET		# IF W-MATRIX INVALID, DON'T INTEGRATE IT
			RENDWFLG
			UPPSV1
			DIM0FLAG	# SET DIM0FLAG TO INTEGRATE W-MATRIX
		BON	SET
			SURFFLAG	# IF ON LUNAR SURFACE W IS 6X6
			UPPSV5
			D6OR9FLG	# OTHERWISE 9X9

; Branch based on vehicle update flag. If VEHUPFLG clear, update CSM first
; (UPPSV3 path). If set, update LM first (UPPSV1 path). In both cases,
; integrate one vehicle with W-matrix covariance, then integrate other vehicle.

UPPSV5		BOF
			VEHUPFLG
			UPPSV3

; UPPSV1: Update LM state vector with W-matrix integration.

UPPSV1		SET
			VINTFLAG
		CALL
			INTEGRV
		CALL			# GROUP 2 PHASE CHANGE
			GRP2PC		# TO PROTECT INTEGRATION
		CALL
			INTSTALL
		DLOAD	CLEAR		# GET TETCSM TO STORE IN TDEC FOR LM INT.
			TETCSM
			VINTFLAG

; UPPSV4: Integrate other vehicle (without W-matrix). Time epoch taken from
; TDEC1, which holds the other vehicle's time.

UPPSV4		CALL			# INTEGRATE OTHER VEHICLE
			SETIFLGS	#	WITHOUT W-MATRIX
		STCALL	TDEC1
			INTEGRV
		BOFF	VLOAD
			SURFFLAG
			P20LEMC4
			RCVLEM
		VSR2
		STOVL	LMPOS
			VCVLEM
		VSR2
		STORE	LMVEL
		GOTO
			LS21X

; UPPSV3: Update CSM state vector (without W-matrix), then integrate LM.

UPPSV3		CLEAR	CALL
			VINTFLAG
			INTEGRV
		CALL
			GRP2PC
		CALL
			INTSTALL
		SET	DLOAD
			VINTFLAG
			TETLEM		# GET TETLEM TO STORE IN TDEC FOR CSM INT.
# Page 501
		GOTO
			UPPSV4
		EBANK=	LOSCOUNT
		COUNT*	$$/P22

# Page 502
# ============================================================================
# PROGRAM P25: PREFERRED TRACKING ATTITUDE (AUTOMATIC OPTICS)
# ============================================================================
#
# PROGRAM DESCRIPTION
#
#	PREFERRED TRACKING ATTITUDE PROGRAM P25
#	MOD NO -- 3
# 	BY P. VOLANTE
#
# FUNCTIONAL DESCRIPTION
#
#	THE PURPOSE OF THIS PROGRAM IS TO COMPUTE THE PREFERRED TRACKING
# 	ATTITUDE OF THE LM TO CONTINUOUSLY POINT THE LM TRTACKING BEACON AT THE
#	CSM AND TO PERFORM THE MANEUVER TO THE PREFERRED TRACKING ATTITUDE AND
# 	CONTINUOUSLY MAINTAIN THIS ATTITUDE WITHIN PRESCRIBED LIMITS.
#
# CALLING SEQUENCE --
#
#	ASTRONAUT REQUEST THROUGH DSKY V37E25E
#
# SUBROUTINES CALLED --
#
#	BANKCALL					FLAGUP
#	R02BOTH	(IMU STATUS CHECK)			ENDOFJOB
#	R61LEM	(PREF TRK ATT ROUT)			WAITLIST
#	TASKOVER					FINDVAC
#
# NORMAL EXIT MODES --
#
# 	P25 MAY BE TERMINATED IN TWO WAYS -- ASTRONAUT SELECTION OF IDLING
#	PROGRAM (P00) BY KEYING V37E00E OR BY KEYING IN V56E
#
# ALARM OR ABORT EXIT MODES --
#
#	NONE
#
# OUTPUT
#
# ERASABLE INITIALIZATION REQUIRED
#
# FLAGS SET + RESET
#
#	TRACKFLG, P25FLAG
#
# DEBRIS
#
#	NONE
#
# COMMENT-ONLY READERS: P25 automatically orients the LM to keep rendezvous
# radar pointed at CSM. Used during Apollo 11 rendezvous after Eagle's ascent.
#
# CODE-ALONG READERS: P25 computes optimal tracking attitude using relative
# geometry, commands DAP for automatic attitude maintenance.
# ============================================================================

; ============================================================================
; PROGRAM P25 - AUTOMATIC OPTICS (RENDEZVOUS RADAR TRACKING)
; ============================================================================
;
; P25 provides automatic rendezvous radar tracking of the Command Module during
; lunar orbit operations. After Eagle's ascent from the lunar surface on July 21,
; 1969, P25 managed radar tracking as Eagle climbed toward Columbia. The program
; maintains optimal tracking attitude, reads radar range/range-rate data, and
; updates navigation state vectors with measurement incorporation.
;
; This code initializes tracking mode, sets necessary flags, and begins the
; automatic tracking cycle that continues until rendezvous completion or crew
; termination via V37E00E (P00 idle program selection).

		EBANK=	LOSCOUNT
		COUNT*	$$/P25
PROG25		TC	2PHSCHNG
		OCT	4		# MAKE GROUP 4 INACTIVE (VERB 37)
		OCT	05022
		OCT	26000		# PRIORITY 26

; IMU status verification ensures inertial platform is operational before
; beginning radar tracking. During Apollo 11 rendezvous, IMU alignment was
; critical for transforming radar measurements into navigation coordinates.

		TC	BANKCALL
		CADR	R02BOTH		# IMU STATUS CHECK

; Set tracking mode flags. TRACKFLG indicates radar tracking is active.
; P25FLAG distinguishes P25 automatic optics mode from other P20-series programs.
; These flags control program flow and enable/disable various navigation features
; throughout the rendezvous sequence.

		TC	UPFLAG
		ADRES	TRACKFLG	# SET TRACK FLAG
		TC	UPFLAG
		ADRES	P25FLAG		# SET P25FLAG

; Establish restart protection. If computer restart occurs during P25 execution
; (due to power transient or alarm condition), this phase table entry enables
; recovery to continue tracking operations without crew reinitialization.

P25LEM1		TC	PHASCHNG
		OCT	04022

; P25 main control loop checks operational flags to determine tracking status.
; This flag-checking sequence verifies both P25FLAG (program active) and
; TRACKFLG (radar locked on CSM). If either flag is cleared by crew action
; or system condition, P25 pauses tracking operations.

		CAF	P25FLBIT
		MASK	STATE		# IS P25FLAG SET
		EXTEND
		BZF	ENDOFJOB
		CAF	TRACKBIT	# IS TRACKFLAG SET?
		MASK	STATE +1
		EXTEND
# Page 503
		BZF	P25LMWT1	# NO -- SKIP PHASE CHANGE AND WAIT 1 MINUTE

; When tracking is confirmed active, compute fine preferred tracking attitude.
; R65LEM calculates optimal LM orientation to keep rendezvous radar antenna
; pointed at CSM while minimizing RCS propellant usage for attitude control.
; R65CNTR = 7 requests fine attitude computation (higher precision than coarse).

		CAF	SEVEN		# CALL R65 -- FINE PREFERRED
		TS	R65CNTR
		TC	BANKCALL	# TRACKING ATTITUDE ROUTINE
		CADR	R65LEM
		TC	P25LEM1		# THEN GO CHECK FLAGS

; If tracking not yet established (TRACKFLG cleared), wait 60 seconds before
; rechecking status. This prevents excessive CPU load during radar acquisition
; phase while allowing reasonable response time for crew-initiated mode changes.

P25LEMWT	TC	PHASCHNG
		OCT	00112
P25LMWT1	CAF	60SCNDS
		TC	TWIDDLE		# WAIT ONE MINUTE THEN CHECK AGAIN
		ADRES	P25LEM2
		TC	ENDOFJOB

; After 60-second delay, restart P25 main control loop as priority 14 job.
; FINDVAC locates available executive core set for job scheduling. This
; restart mechanism allows P25 to continue checking tracking status without
; monopolizing executive resources during acquisition phase.

P25LEM2		CAF	PRIO14
		TC	FINDVAC
		EBANK=	LOSCOUNT
		2CADR	P25LEM1

		TC	TASKOVER
60SCNDS		DEC	6000		# 6000 centiseconds = 60 seconds

# Page 504
# DATA READ ROUTINE 22 (LEM)
# PROGRAM DESCRIPTION
#
#	MOD NO -- 2
#	BY P. VOLANTE
#
# FUNCTIONAL DESCRIPTION
#
#	TO PROCESS AUTOMATIC RR MARK DATA TO UPDATE THE STATE VECTOR OF EITHER
# 	LM OR CSM AS DEFINED IN THE RENDEZVOUS NAVIGATION PROGRAM (P20)
#
# CALLING SEQUENCE --
#
#	TC	BANKCALL
#	CADR	R22LEM
#
# SUBROUTINES CALLED --
#
#	LSR22.1		GOFLASH		WAITLIST
#	LSR22.2		PRIOLARM	BANKCALL
#	LSR22.3		R61LEM
#
# NORMAL EXIT MODES --
#
#	R22 WILL CONTINUE TO RECYCLE, UPDATING STATE VECTORS WITH RADAR DATA
#	UNTIL P20 CEASES TO OPERATE (RENDEZVOUS FLAG SET TO ZERO) AT WHICH TIME
#	R22 WILL TERMINATE SELF.
#
# ALARM OR ABORT EXIT MODES --
#
#	PRIORITY ALARM
#	PRIORITY ALARM 525 LOS NOT WITHIN 3 DEGREE LIMIT
#
# OUTPUT
#
#	SEE OUTPUT FROM LSR22.3
#
# ERASABLE INITIALIZATION REQUIRED
#
#	SEE LSR22.1, LSR22.2, LSR22.3
#
# FLAGS SET + RESET
#
#	NOANGFLG
#
# DEBRIS
#
#	SEE LSR22.1, LSR22.2, LSR22.3

; ============================================================================
; ROUTINE R22LEM - AUTOMATIC RADAR DATA READ AND STATE VECTOR UPDATE
; ============================================================================
;
; R22LEM is the core data processing routine for automatic rendezvous radar
; tracking. Once radar lock-on is established, R22 continuously reads range
; and range-rate measurements, processes them through navigation filters, and
; updates either the LM or CSM state vector (as selected by the crew).
;
; During Apollo 11's rendezvous on July 21, 1969, R22 processed radar marks
; as Eagle climbed from the lunar surface, providing Armstrong and Aldrin with
; continuous updates on their relative position and velocity with respect to
; Columbia. This data fed into the rendezvous targeting algorithms that computed
; the three maneuvers (CSI, CDH, TPI) bringing Eagle to docking range.
;
; R22 recycles automatically, taking measurements at regular intervals until
; the rendezvous flag (RNDVZFLG) is cleared by crew selection of P00 idle
; program or rendezvous completion.

		EBANK=	LRS22.1X
		COUNT*	$$/R22

; R22LEM entry point. Check rendezvous flag to confirm P20-series program
; is still active. If RNDVZFLG cleared, terminate R22 and exit to ENDOFJOB.

R22LEM		TC	PHASCHNG
		OCT	04022

; Flag verification sequence confirms operational conditions before processing
; radar data. RNDVZBIT (rendezvous flag) must be set indicating P20-series
; program active. TRACKBIT verifies radar lock-on established. If either flag
; cleared, R22 terminates or waits rather than processing invalid data.

		CAF	RNDVZBIT	# IS RENDEZVOUS FLAG SET?
		MASK	STATE
		EXTEND
		BZF	ENDOFJOB	# NO -- EXIT R22 AND P20
		CAF	TRACKBIT	# IS TRACKFLAG SET?
		MASK	STATE +1
		EXTEND
		BZF	R22WAIT		# NO WAIT

; Hardware discrete verification - checks rendezvous radar physical status
; via I/O channels before attempting data read. These checks ensure radar
; electronics are in proper operational mode for automatic tracking.

R22LEM12	CAF	BIT14		# IS RR AUTO TRACK ENABLE DISCRETE STILL
		EXTEND			# ON (A MONITOR REPOSITION BY R25 CLEARS IT)
		RAND	CHAN12		# Read channel 12 (radar status)
		EXTEND
		BZF	P20LEMA		# NO -- RETURN TO P20
		CAF	BIT2		# YES
		EXTEND			# IS RR AUTO MODE DISCRETE PRESENT
		RAND	CHAN33		# Read channel 33 (radar mode status)
# Page 505
		EXTEND
		BZF	+2		# YES CONTINUE
		TC	P20LEMB5	# NO -- SET IT
		CS	RADMODES	# ARE RR CDUS BEING ZEROED
		MASK	RCDU0BIT	# Check RR CDU zero flag
		EXTEND
		BZF	R22LEM42	# CDUS BEING ZEROED

; Radar data read sequence. LRS22.1 subroutine reads rendezvous radar range
; and range-rate from radar electronics, calculates line-of-sight (LOS) vector
; from radar gimbal angles, and performs data validity checks. During Apollo 11
; rendezvous, this sequence executed every few seconds, building navigation
; solution as Eagle closed on Columbia.

		TC	PHASCHNG	# IF A RESTART OCCURS, AND EXTRA RADAR
		OCT	00152		# READING IS TAKEN, SO BAD DATA ISN'T USED
		TC	BANKCALL	# YES READ DATA + CALCULATE LOS
		CADR	LRS22.1		# DATA READ SUBROUTINE
		INDEX	MPAC		# Indexed return based on status code in MPAC
		TC	+1		# Return dispatcher
		TC	R22LEM2		# NORMAL RETURN (GOOD DATA)
		TC	P20LEMC		# COULD NOT READ RADAR -- TRY TO REDESIGNATE
		CAF	ALRM525		# RR LOS NOT WITHIN 3 DEGREES (ALARM)
		TC	BANKCALL
		CADR	PRIOLARM	# Display priority alarm to crew
		TC	GOTOV56		# TERMINATE EXITS P20 VIA V56 CODING
		TC	R22LEM1		# PROC (DISPLAY DELTA THETA)
		TC	-5		# ENTER (ILLEGAL OPTION)
		TC	ENDOFJOB

; R22LEM1 - Delta Theta Display Handler
; When alarm 525 occurs (LOS not within 3 degrees), crew can select PROC
; to display delta theta (angular error between expected and actual radar
; pointing). This allows crew to assess whether error is acceptable or
; requires radar redesignation. VERB 06 NOUN 05 displays angular components.

R22LEM1		TC	PHASCHNG
		OCT	04022
		CAF	V06N05		# DISPLAY DELTA THETA
		TC	BANKCALL
		CADR	PRIODSP		# Priority display (flashing V/N)
		TC	GOTOV56		# TERMINATE EXITS P20 VIA V56 CODING
		TC	R22LEM2		# PROC (OK CONTINUE)
		TC	P20LEMC		# ENTER (RECYCLE)

; R22LEM2 - Normal Data Return Processing with Safety Checks
; Primary navigation update path after good radar data received. Performs
; multiple safety checks before updating state vectors: lunar surface flag
; (if on surface, bypass some checks), track flag verification, and radar
; boresight angle verification (RR must point within 30 degrees of LM +Z axis
; to avoid gimbal limits and ensure valid tracking geometry). During Apollo 11
; ascent rendezvous, these checks ensured radar maintained proper orientation
; as Eagle pitched over during climb to orbit.

R22LEM2		TC	PHASCHNG
		OCT	04022
		TC	LUNSFCHK	# CHECK IF ON LUNAR SURFACE (P22FLAG SET)
		TC	R22LEM3		# YES -- BYPASS FLAG CHECKS AND LRS22.2
		CA	FLAGWRD1	# IS TRACK FLAG SET
		MASK	TRACKBIT
		EXTEND
		BZF	R22WAIT		# NO -- WAIT
		TC	BANKCALL	# YES
		CADR	LRS22.2		# CHECKS RR BORESIGHT WITHIN 30 DEG OF +Z
		INDEX	MPAC		# Indexed return from LRS22.2
		TC	+1		# Return dispatcher
		TC	R22LEM3		# NORMAL RETURN (LOS WITHIN 30 OF Z-AXIS)
		TC	BANKCALL	# Boresight angle excessive
		CADR	R61LEM		# Call preferred tracking attitude routine
		TC	R22WAIT		# NOT WITHIN 30 DEG OF Z-AXIS

; R22LEM3 - State Vector Update Decision Point
; Controls whether radar data updates navigation state vectors. Checks two
; flags: NOUPFBIT (no-update flag) inhibits updates during special operations,
; UPDATBIT enables updates when crew has selected update mode via DSKY.
; If updates enabled, transfers to LSR22.3 (measurement incorporation) which
; uses Kalman filtering to blend radar measurements with current navigation
; solution. During rendezvous, each good radar mark refined relative position
; knowledge between LM and CSM.

R22LEM3		CS	FLAGWRD1	# SHOULD WE BYPASS STATE VECTOR UPDATE
		MASK	NOUPFBIT	# (IS NO UPDATE FLAG SET?)
# Page 506
		EXTEND
		BZF	R22LEM42	# BRANCH -- YES
		CA	FLAGWRD1	# IS UPDATE FLAG SET
		MASK	UPDATBIT
		EXTEND
		BZF	R22LEM42	# UPDATE FLAG NOT SET
		CAF	PRIO26		# INSURE HIGH PRIO IN RESTART
		TS	PHSPRDT2	# Set phase priority for restart protection

		TC	INTPRET		# Enter interpretive mode
		GOTO			# Transfer to measurement incorporation
			LSR22.3		# Kalman filter update routine

; R22LEM93 - Normal Exit from Measurement Incorporation
; Returns here after successful Kalman filter update (LSR22.3). Performs
; phase change to protect against restart conflicts with GRP2PC erasable
; memory, then continues to mark counter increment.

R22LEM93	EXIT			# NORMAL EXIT FROM LSR22.3
		TC	PHASCHNG	# PHASE CHANGE TO PROTECT AGAINST
		OCT	04022		# CONFLICT WITH GRP2PC ERASEABLE
		TCF	R22LEM44	# Continue to mark counter increment

; R22LEM96 - Crew Decision Point Display
; Entry point when measurement incorporation requires crew decision about
; data acceptance. Displays V06 N49 (navigation update decision display)
; asking crew to accept (PROCEED) or reject (RECYCLE) the radar measurement.
; During Apollo 11 rendezvous, crew monitored these displays to verify radar
; data quality before allowing state vector updates. N49FLAG tracks display
; response: zero = unanswered, positive = proceed, negative = recycle.

R22LEM96	EXIT
		CAF	ZERO		# SET N49FLAG = ZERO TO INDICATE
		TS	N49FLAG		# V06 N49 DISPLAY HASN'T BEEN ANSWERED
		TC	PHASCHNG
		OCT	04022		# TO PROTECT DISPLAY
		CAF	PRIO27		# PROTECT DISPLAY
		TC	NOVAC		# Schedule display job
		EBANK=	N49FLAG
		2CADR	N49DSP		# V06 N49 display routine

		TC	INTPRET		# Wait for crew response
		SLOAD			# Load N49FLAG to check response
			N49FLAG
		BZE	BMN		# LOOP TO CHECK IF FLAG
			-3		# SETTING CHANGED -- BRANCH -- NO
			R22LEM7		# PROCEED (flag positive)
		EXIT			# DISPLAY ANSERED BY RECYCLE (flag negative)
		TC	LUNSFCHK	# ARE WE ON LUNAR SURFACE
		TC	R22WAIT		# YES -- 15 SECOND DELAY
		CA	ZERO		# NO -- SET R65COUNTER = 0, DO FINE
		TC	R22LEM45	# TRACKING TAKE ANOTHER RADAR READING

; R22LEM7 - Crew Proceed Path
; Crew selected PROCEED on V06 N49 display, accepting radar measurement.
; Performs phase change and transfers to ASTOK (state vector update completion).

R22LEM7		CALL			# PROCEED
			GRP2PC		# PHASE CHANGE AND
		GOTO			# GO TO INCOPORATE DATA.
			ASTOK		# Complete state vector update

; R22LEM44 - Mark Counter Increment and Next Tracking Cycle
; Increments MARKCTR (count of radar marks successfully incorporated into
; navigation solution). During Apollo 11 rendezvous, approximately 30-40 marks
; were taken and incorporated over the 3+ hour rendezvous sequence. Each mark
; improved knowledge of relative position/velocity. After incrementing counter,
; determines wait time and preferred attitude computation based on surface flag.

R22LEM44	INCR	MARKCTR		# INCREMENT COUNT OF MARKS INCORPORATED.
		TC	LUNSFCHK	# ARE WE ON LUNAR SURFACE
		TC	R22LEM46	# YES -- WAIT 2 SECONDS
		CA	FIVE		# NOT ON LUNAR SURFACE
		TC	R22LEM45	# R65COUNTER = 5

; R22LEM42 - No-Update Path Mark Cycle
; Entry when state vector updates bypassed (NOUPFBIT set or UPDATBIT clear).
; Still takes radar marks for crew display but doesn't incorporate into
; navigation solution. Used during special operations or when crew wants
; radar data displayed but navigation locked.

R22LEM42	TC	LUNSFCHK	# CHECK IF ON LUNAR SURFACE (P22FLAG SET)
		TC	R22LEM46	# YES -- WAIT 2 SECONDS
		CA	TWO		# NO -- SET R65COUNTER = 2

; R22LEM45 - Fine Tracking Attitude Computation
; Computes refined LM attitude for optimal radar tracking geometry. R65CNTR
; controls tracking refinement: higher values (5) request more frequent attitude
; adjustments during dynamic phases, lower values (2) during stable tracking.
; Calls R65LEM (fine preferred tracking attitude routine).

R22LEM45	TS	R65CNTR		# Store R65 counter value
# Page 507
		TC	BANKCALL
		CADR	R65LEM		# FINE PREFERRED TRACKING ATTITUDE
		TC	R22LEM		# Return to R22LEM for next mark

; R22WAIT - 15-Second Delay Between Marks
; Standard delay between radar measurements when not on lunar surface.
; Provides adequate time for state vector propagation, radar data processing,
; and crew monitoring. 1500 centiseconds = 15 seconds.

R22WAIT		CAF	1500DEC		# 1500 centiseconds (15 seconds)
		TC	P20LEMWT +1	# Delay and return to mark cycle

; R22LEM46 - 2-Second Lunar Surface Delay
; Shorter delay when on lunar surface. CSM in stable orbit overhead, LM
; stationary on surface, so rapid mark rate unnecessary. 2-second delay
; adequate for radar data cycle and display updates.

R22LEM46	CAF	2SECS		# 2 seconds delay
		TC	BANKCALL	# WAIT 2 SECONDS AND TAKE ANOTHER MARK
		CADR	DELAYJOB	# Delay job scheduler
		TC	R22LEM		# Return to R22LEM for next mark

; N49DSP - Excessive Update Display Routine
; Displays V06 N49 when Kalman filter detects excessive delta-R or delta-V
; between predicted and measured state. R1 shows position error magnitude,
; R2 shows velocity error magnitude. Crew can: TERMINATE (exit P20 via V56),
; PROCEED (accept update, N49FLAG=-1), or RECYCLE (reject update, N49FLAG=+1).
; During rendezvous, large deltas could indicate radar anomalies or unexpected
; CSM maneuvers.

N49DSP		CAF	V06N49NB	# V06 N49 display code
		TC	BANKCALL	# EXCESSIVE STATE VECTOR UPDATE -- FLASH
		CADR	PRIODSP		# VERB 06 NOUN 49 R1=DELTA R, R2=DELTA V
		TC	GOTOV56		# TERMINATE -- EXIT R22 AND P20
		CS	ONE		# PROCEED -- N49FLAG = -1
		TS	N49FLAG		# RECYCLE -- N49FLAG = + VALUE
		TC	ENDOFJOB	# Job complete, await next scheduler cycle

; R22RSTRT - Restart Recovery for Radar Read
; If AGC restart occurs during radar data read, this routine provides safe
; recovery. Takes one range-rate reading which is discarded (prevents using
; potentially corrupted data from interrupted read). Then attempts fresh
; radar designation and continues normal R22 sequence. Restart protection
; critical during time-sensitive rendezvous operations.

R22RSTRT	TC	PHASCHNG	# IF A RESTART OCCURS WHILE READING RADAR
		OCT	00152		# COME HERE TO TAKE A RANGE-RATE READING
		TC	BANKCALL	# WHICH ISN'T USED TO PREVENT TAKING A BAD
		CADR	RRRDOT		# READING AND TRYING TO INCORPORATE THE
		TC	BANKCALL	# BAD DATA
		CADR	RADSTALL	# WAIT FOR READ COMPLETE
		TC	P20LEMC		# COULD NOT READ RADAR -- TRY TO REDISGNATE
		TC	R22LEM		# READ SUCCESSFUL -- CONTINUE AT R22

; Constants for R22LEM routines

ALRM525		OCT	00525		# Alarm code 525 (LOS not within limits)
V06N05		VN	00605		# Display verb 06 noun 05 (delta theta)
V06N49NB	VN	00649		# Display verb 06 noun 49 (delta R/V)
1500DEC		DEC	1500		# 1500 centiseconds (15 seconds)

; ============================================================================
; LUNSFCHK - Lunar Surface Flag Check Subroutine
;
; Closed subroutine checks whether LM is on lunar surface (SURFFLAG in
; FLAGWRD8). Returns to caller+1 if on surface, caller+2 if not on surface.
; Used throughout P20-P25 to select appropriate timing and logic paths.
; During Apollo 11's 21.5-hour surface stay, this flag was set, affecting
; radar mark timing and CSM state vector extrapolation strategies.
; ============================================================================

		COUNT* 	$$/P22
LUNSFCHK	CS	FLAGWRD8	# CHECK IF ON LUNAR SURFACE
		MASK	SURFFBIT	# IS SURFFLAG SET?
		CCS	A		# BRANCH -- P22FLAG SET
		INCR	Q		# NOT SET (skip return address)
		TC	Q		# RETURN (to caller+1 if set, +2 if not)

# Page 508
# RR DESIGNATE ROUTINE (R21LEM)
# PROGRAM DESCRIPTION
#
# 	MOD NO -- 2
#	BY P. VOLANTE
#
# FUNCTIONAL DESCRIPTION
#
# 	TO POINT THE RENDEZVOUS RADAR AT THE CSM UNTIL AUTOMATIC ACQUISITION
# 	OF THE CSM IS ACCOMPLISHED BY THE RADAR.  ROUTINE IS CALLED BY P20.
#
# CALLING SEQUENCE --
#
#	TC	BANKCALL
#	CADR	R21LEM
#
# SUBROUTINES CALLED --
#
#	FINDVAC		FLAGUP		ENDOFJOB	PRIOLARM
#	NOVAC		INTPRET		LPS20.1		PHASCHNG
#	WAITLIST	JOBSLEEP	JOBWAKE		FLAGDOWN
#	TASKOVER	BANKCALL	RADSTALL	RRDESSM
#
# NORMAL EXIT MODES
#
#	WHEN LOCK-ON IS ACHIEVED, BRANCH WILL BE TO P20 WHERE R22 (DATA READ
#	WILL BE SELECTED OR A NEED FOR A MANEUVER (BRANCH TO P20LEMA)
#
# ALARM OR ABORT EXIT MODES --
#
#	PRIORITY ALARM 503 WHEN LOCK-ON HASN'T BEEN ACHIEVED AFTER 30SECS --
#	THIS REQUIRES ASTRONAUT INTERFACE: SELECTION OF SEARCH OPTION OF
#	ACQUISITION
#
# OUTPUT
#
#	SEE LPS20.1, RRDESSM
#
# ERASABLE INITIALIZATION REQUIRED
#
#	RRTARGET, RADMODES ARE USED BY LPS20.1 AND RRDESSM
#
# FLAGS SET + RESET
#
#	LOSCMFLG	LOKONSW
#
# DEBRIS
#
#	SEE LPS20.1, RRSESSM

; ============================================================================
; TRANSITION: From R22LEM Data Read Routines to R21LEM Radar Designate
;
; Following the radar data acquisition sequence (R22LEM), the R21LEM routine
; handles the critical antenna pointing and lock-on acquisition process.
; During Apollo 11's rendezvous on July 21, 1969, R21LEM commanded Eagle's
; rendezvous radar antenna to track Columbia's position. The routine computed
; line-of-sight (LOS) vectors from current navigation state, commanded antenna
; angles, and managed the lock-on acquisition sequence. When antenna pointing
; achieved lock-on, R21LEM exited successfully, enabling R22LEM to begin
; taking radar measurements. If LOS fell outside radar coverage, the routine
; requested crew maneuvers to bring Columbia within the radar's field of view.
; ============================================================================

; R21LEM - Rendezvous Radar Designate Main Entry
; This routine manages the complete rendezvous radar designation sequence:
; 1) Disables RR self-track mode, 2) Checks lunar surface flag, 3) Commands
; antenna to appropriate mode/position, 4) Computes line-of-sight to CSM,
; 5) Designates radar to computed LOS, 6) Waits for lock-on acquisition.
; During Apollo 11 rendezvous, radar first acquired Columbia at approximately
; 100 nautical miles range, establishing the navigation baseline for the
; three-burn rendezvous sequence (CSI, CDH, TPI).

		EBANK=	LOSCOUNT
		COUNT*	$$/R21
R21LEM		CS	BIT14		# REMOVE RR SELF TRACK ENABLE
		EXTEND
		WAND	CHAN12
		TC	LUNSFCHK
		TC	R21LEM5
		CAF	ZERO		# COMMAND ANTENNA TO MODE CENTER
		TS	TANG		# IF NOT ON SURFACE -- MODE 1 -- (T=0,S=0)
		TS	TANG +1
		TC	R21LEM6
R21LEM5		CAF	BIT12
		MASK	RADMODES
		CCS	A
		TC	R21LEM10
		CAF	BIT15
		TS	TANG
		CS	HALF
		TS	TANG +1

# Page 509
R21LEM6		TC	DOWNFLAG
		ADRES	LOKONSW
		TC	BANKCALL
		CADR	RRDESNB
		TC	+1
		TC	BANKCALL
		CADR	RADSTALL
		TC	R21-503		# BAD RETURN FROM DESIGNATE -- ISSUE ALARM
R21LEM10	TC	UPFLAG
		ADRES	LOSCMFLG	# EVERY FOURTH PASS THRU DODES
		CAF	MAXTRIES	# ALLOW 60 PASSES (APPROX 45 SECONDS)
		TS	DESCOUNT	# TO DESIGNATE AND LOCK ON
R21LEM2		CAF	THREE
		TS	LOSCOUNT
R21LEM1		TC	INTPRET
		RTB	DAD
			LOADTIME
			HALFSEC		# EXTRAPOLATE TO PRESENT TIME + .5 SEC.
		STCALL	TDEC1		# LOS DETERMINATION ROUTINE
			LPS20.1
		EXIT
R21LEM3		TC	UPFLAG		# SET LOKONSW TO RADAR -- ON DESIRED
		ADRES	LOKONSW
		TC	DOWNFLAG
		ADRES	NORRMON
		TC	INTPRET
		CALL			# INPUT (RRTARGET UPDATED BY LPS20.1)
			RRDESSM		# DESIGNATE ROUTINE
		EXIT
		TC	R21LEM4		# LOS NOT IN MODE 2 COVERAGE
					# ON LUNAR SURFACE
		TC	P20LEMA		# VEHICLE MANEUVER REQUIRED.
		TC	BANKCALL	# NO VEHICLE MANEUVER REQUIRED
		CADR	RADSTALL	# WAIT FOR DESIGNATE COMPLETE -- LOCKON OR
		TC	+2		# BAD END -- LOCKON NOT ACHIEVED IN 60 TRIES
		TC	R21END		# EXIT ROUTINE RETURN TO P20 (LOCK-ON)
R21-503		CAF	ALRM503		# ISSUE ALARM 503
		TC	BANKCALL
		CADR	PRIOLARM
		TC	GOTOV56		# TERMINATE EXITS P20 VIA V56 CODING
		TC	R21SRCH		# PROC
		TC	P20LEMC3
		TC	ENDOFJOB
R21END		TC	DOWNFLAG
		ADRES	LOSCMFLG	# RESET LOSCMFLG
		TC	R21DISP		# PUT UP VERIFY MAIN LOBE LOCKON DISPLAY
R21SRCH		TC	PHASCHNG
		OCT	04022
		TC	R24LEM		# SEARCH ROUTINE
ALRM503		OCT	00503
# Page 510
ALRM527		OCT	527

R21LEM4		CAF	MAXTRIES	# SET UP COUNTER FOR
		TS	REPOSCNT	# 60 PASSES (APPROX 600 SECS.)
		TC	UPFLAG
		ADRES	FSPASFLG	# SET FIRST PASS FLAG
		TC	DOWNFLAG	# RESET LOS BEING
		ADRES	LOSCMFLG	# COMPUTED FLAG
		TC	INTPRET
R21LEM12	RTB
			LOADTIME
		DAD
			TENSEC		# TIME T = T + 10 SECS.
		STORE	REPOSTM		# SAVE FOR LONGCALL AND UPPSV
		STCALL	TDEC1
			LPS20.1		# COMPUTE LOS AT TIME T
		CALL
			RRDESSM
		EXIT
		TC	R21LEM13	# LOS NOT IN MODE 2 COVERAGE
		TC	ENDOFJOB	# VEHICLE MANEUVER REQUIRED
		TC	KILLTASK
		CADR	BEGDES
		TC	INTPRET
		BOF
			FSPASFLG	# FIRST PASS THRU REPOSITION
			R21LEMB		# NO -- GO TO CONTINUOUS DESIGNATE
		CLRGO
			FSPASFLG	# YES -- RESET FIRST PASS FLAG
			R21LEM7 +1
R21LEM13	CCS	REPOSCNT	# HAVE WE TRIED 60 TIMES?
		TC	R21LEM7		# NO -- ADD 10 SECS.  RECOMPUTE LOS
		TC	R21LEM11	# YES -- PUT OUT ALARM 530
R21LEM7		TS	REPOSCNT
		TC	INTPRET
		DLOAD	GOTO
			REPOSTM
			R21LEM12 +2
R21LEMB		DLOAD
			REPOSTM
		STCALL	TDEC1
			UPPSV
		EXIT
		TC	UPFLAG		# SET RADMODES BIT 15 FOR
		ADRES	CDESFLAG	# CONTINUOUS DESIGNATION
		TC	DOWNFLAG
		ADRES	LOKONSW
		TC	UPFLAG
		ADRES	NORRMON
# Page 511
		TC	BANKCALL
		CADR	RRDESNB
		TC	+1
		TC	INTPRET
		RTB	BDSU
			LOADTIME	# COMPUTE DELTA TIME
			REPOSTM		# FOR LONGCALL
		STORE	DELTATM
		EXIT
		EXTEND
		DCA	DELTATM
		TC	LONGCALL
		EBANK=	LOSCOUNT
		2CADR	R21LEM9

		TC	ENDOFJOB
R21LEM9		TC	KILLTASK
		CADR	STDESIG
		TC	CLRADMOD
		CAF	PRIO26
		TC	FINDVAC
		EBANK=	LOSCOUNT
		2CADR	R21LEM10

		TC	TASKOVER
R21LEM11	CAF	ALRM530		# ALARM 530 -- LOS NOT IN COVERAGE
		TC	BANKCALL	# AFTER TRYING TO DESIGNATE FOR
		CADR	PRIOLARM	# 600 SECS.
		TC	GOTOV56
		TC	GOTOV56
		TC	GOTOV56
		TC	ENDOFJOB
ALRM530		OCT	00530
TENSEC		2DEC	1000 B-28

HALFSEC		2DEC	50

R21DISP		TC	PHASCHNG
		OCT	04022
		CAF	V06N72PV	# FLASH V 50 N 72 -- PLEASE PERFORM RR
		TC	BANKCALL	# MAIN LOBE LOCKON VERIFICATION
		CADR	GOPERF2R
		TC	GOTOV56		# TERMINATE EXITS VIA V 56
		TC	P20LEMWT	# PROCEED CONTINUES TO R22
		TC 	-5		# ENTER ILLEGAL
		CAF	BIT7
		TC	LINUS		# SET BITS TO MAKE THIS A PRIORITY DISPLAY
		TC	ENDOFJOB

# Page 512
V06N72PV	VN	00672

# Page 513
# MANUAL ACQUISITION ROUTINE R23LEM
# PROGRAM DESCRIPTION
#
#	MOD NO -- 2
#	BY P. VOLANTE
#
# FUNCTIONAL DESCRIPTION
#
#	TO ACQUIRE THE CSM BY MANUAL OPERATION OF THE RENDEZVOUS RADAR
#
# CALLING SEQUENCE --
#
#	TC	R23LEM
#
# SUBROUTINES CALLED
#
#	BANKCALL	R61LEM
#	SETMINDB	GOPERF1
#
# NORMAL EXIT MODES --
#
#	IN RESPONSE TO THE GOPERF1,	SELECTION OF ENTER WILL RECYCLE R23
#					SELECTION OF PROC WILL CONTINUE R23
#					SELECTION OF TERM WILL TERMINATE R23 + P20
#
# ALARM OR ABORT EXIT MODES --
#
#	SEE NORMAL EXIT MODES ABOVE
#
# OUTPUT
#
#	N.A.
#
# ERASABLE INITIALIZATION REQUIRED --
#
#	ACMODFLG MUST BE SET TO 1 (MANUAL MODE)

		EBANK=	GENRET
		COUNT*	$$/R23
R23LEM		TC	UPFLAG		# SET NO ANGLE MONITOR FLAG
		ADRES	NORRMON
		INHINT
		TC	IBNKCALL	# SELECT MINIMUM DEADBAND
		CADR	SETMINDB
		RELINT
R23LEM1		CAF	BIT14		# ENABLE TRACKER
		EXTEND
		WOR	CHAN12
		CAF	OCT205
		TC	BANKCALL
		CADR	GOPERF1
		TC	R23LEM2		# TERMINATE
		TC	R23LEM11	# PROCEDE
		TC	R23LEM3		# ENTER -- DO ANOTHER MANEUVER
R23LEM11	INHINT
		TC	RRLIMCHK	# YES -- CHECK IF ANTENNA IS WITHIN LIMITS
		ADRES	CDUT
		TC	OUTOFLIM	# NOT WITHIN LIMITS
		TC	IBNKCALL	# RESTORE DEADBAND TO
		CADR	RESTORDB	# ASTRONAUT SELECTED VALUE
		RELINT
		TC	DOWNFLAG	# CLEAR NO ANGLE MONITOR FLAG
		ADRES	NORRMON
		TC	P20LEMB1	# RADAR IS LOCKED ON CONTINUE IN P20
OUTOFLIM	RELINT
# Page 514
		CAF	OCT501PV
		TC	BANKCALL	# ISSUE ALARM -- RR ANTENNA NOT WITHIN
		CADR	PRIOLARM	# LIMITS
		TC	R23LEM2		# TERMINATE -- EXIT R23 TO R00 (GO TO POOH)
		TC	OUTOFLIM +1	# PROCEED ILLEGAL
		TC	R23LEM3		# RECYCLE -- TO ANOTHER MANEUVER
		TC	ENDOFJOB
R23LEM2		TC	DOWNFLAG	# CLEAR NO ANGLE MONITOR FLAG
		ADRES	NORRMON
		TC	GOTOV56		# AND EXIT VIA V56
R23LEM3		TC	BANKCALL
		CADR	R61LEM
		TC	R23LEM1

OCT501PV	OCT	501
OCT205		OCT	205

# Page 515
# SEARCH ROUTINE R24LEM
# PROGRAM DESCRIPTION
#
#	MOD NO -- 2
#	BY P. VOLANTE
#
# FUNCTIONAL DESCRIPTION
#
#	TO ACQUIRE THE CSM BY A SEARCH PATTERN WHEN THE RENDEZVOUS RADAR HAS
#	FAILED TO ACQUIRE TEH CSM IN THE AUTOMATIC TRACKING MODE AND TO ALLOW
# 	THE ASTRONAUT TO CONFIRM THAT REACQUISITION HAS NOT BEEN IN SIDELOBE.
#
# CALLING SEQUENCE
#
#	CAF	PRIONN
#	TC	FINDVAC
#	EBANK=	DATAGOOD
#	2CADR	R24LEM
#
# SUBROUTINES CALLED
#
#	FLAGUP		FLAGDOWN	BANKCALL
#	R61LEM		GOFLASHR	FINDVAC
#	ENDOFJOB	NOVAC		LSR24.1
#
# NORMAL EXIT MODES --
#
#	ASTRONAUT RESPONSE TO DISPLAY OF OMEGA AND DATAGOOD.  HE CAN EITHER
# 	REJECT BY TERMINATING (SEARCH OPTION AND RESELECTING P20) OR ACCEPT BY
#	PROCEEDING (EXIT ROUTINE AND RETURN TO AUTO MODE IN P20)
#
# ALARM OR ABORT EXIT MODES --
#
#	SEE NORMAL EXIT MODES ABOVE
#
# OUTPUT --
#
#	SEE OUTPUT FROM LSR24.1 + R61LEM
#
# ERASABLE INITIALIZATION REQUIRED
#
#	SET INPUT FOR LSR24.1
#
# FLAGS SET + RESET
#
#	SRCHOPT, ACMODFLG

		EBANK=	DATAGOOD
		COUNT*	$$/R24
R24LEM		TC	UPFLAG
		ADRES	SRCHOPTN	# SET SRCHOPT FLAG
		TC	DOWNFLAG	# RESET LOS BEING COMPUTED FLAG TO MAKE
		ADRES	LOSCMFLG	# SURE DODES DOESN'T GO TO R21
R24LEM1		CAF	ZERO
		TS	DATAGOOD	# ZERO OUT DATA INDICATOR
		TS	OMEGAD		# ZERO OMEGA DISPLAY REGS
		TS	OMEGAD +1	# ZERO OMEGA DISPLAY REGS
R24LEM2		TC	PHASCHNG
		OCT	04022
		CAF	V16N80
		TC	BANKCALL
		CADR	PRIODSPR
		TC	GOTOV56
		TC	R24END		# PROCEED EXIT R24 TO P20LEM1

		TC	R24LEM3		# RECYCLE -- CALL R61 TO MANEUVER S/C
# Page 516
		TC	BANKCALL
		CADR	LRS24.1
R24END		TC	KILLTASK
		CADR	CALLDGCH
		TC	CLRADMOD	# CLEAR BITS 10 & 15 OF RADMODES.
		TCF	P20LEM1		# AND GO TO 400 MI. RANGE CHECK IN P20

		BLOCK	3
		SETLOC	FFTAG6
		BANK
		COUNT*	$$/R24

CLRADMOD	CS	BIT10+15
		INHINT
		MASK	RADMODES
		TS	RADMODES
		CS	BIT2		# DISABLE RR ERROR COUNTERS
		EXTEND
		WAND	CHAN12		# USER WILL RELINT

		TC	Q

BIT10+15	OCT	41000
		BANK	24
		SETLOC	P20S
		BANK
		COUNT*	$$/R24

R24LEM3		TC	PHASCHNG
		OCT	04022
		TC	KILLTASK
		CADR	CALLDGCH	# KILL WAITLIST FOR NEXT POINT IN PATTERN
		TC	CLRADMOD	# CLEAR BITS 10 + 15 OF RADMODES TO KILL
		RELINT			# HALF SECOND DESIGNATE LOOP
		CAF	.5SEC
		TC	BANKCALL	# WAIT FOR DESIGNATE LOOP TO DIE
		CADR	DELAYJOB
		TC	LUNSFCHK	# CHECK IF ON LUNAR SURFACE
		TC	R24LEM4		# YES -- DON'T DO ATTITUDE MANEUVER
		TC	BANKCALL	# CALL R61 TO DO PREFERRED TRACKING
		CADR	R61LEM		# ATTITUDE MANEUVER
R24LEM4		CAF	ZERO		# ZERO OUT RADCADR (WHICH WAS SET BY
		TS	RADCADR		# ENDRADAR WHEN DESIGNATE STOPPED) SO THAT
					# RRDESSM WILL RETURN TO CALLER
		TC	R24LEM2		# AND GO BACK TO PUT UP V16 N80 DISPLAY

V16N80		VN	01680

# Page 517
# PREFERRED TRACKING ATTITUDE ROUTINE R61LEM
# PROGRAM DESCRIPTION
#
#	MOD NO: 3		DATE: 4-11-67
#	MOD BY: P. VOLANTE, SDC
#
# FUNCTIONAL DESCRIPTION --
#
#	TO COMPUTE THE PREFERRED TRACKING ATTITUDE OF THE LM TO ENABLE RR
#	TRACKING OF THE CSM AND TO PERFORM THE MANEUVER TO THE PREFERRED
#	ATTITUDE.
#
# CALLING SEQUENCE --
#
#	TC	BANKCALL
#	CADR	R61LEM
#
# SUBROUTINES CALLED
#
#	LPS20.1		VECPOINT
#	KALCMAN3
#
# NORMAL EXIT MODES --
#
#	NORMAL RETURN IS TO CALLER + 1
#
# ALARM OR ABORT EXIT MODES --
#
#	TERMINATE P20 + R61 BY BRANCHING TO P20END IF BOTH TRACKFLAG +
#	RENDEZVOUS FLAG ARE NOT SET.
#
# OUTPUT --
#
#	SEE OUTPUT FOR LPS20.1 + ATTITUDE MANEUVER ROUTINE (R60)
#
# ERASABLE INITIALIZATION REQUIRED
#
#	GENRET USED TO SAVE Q FOR RETURN
#
# FLAGS SET + RESET
#
#	3AXISFLG
#
# DEBRIS
#
# 	SEE SUBROUTINES

		SETLOC	R61
		BANK
		EBANK=	LOSCOUNT
		COUNT*	$$/R61
R61LEM		TC	MAKECADR
		TS	GENRET
		TC	UPFLAG		# SET R61 FLAG
		ADRES	R61FLAG
		TC	R61C+L01
R65LEM		TC	MAKECADR
		TS	GENRET
		TC	DOWNFLAG	# RESET R61 FLAG
		ADRES	R61FLAG
R61C+L01	CAF	TRACKBIT	# TRACKFLAG
		MASK	STATE +1
		EXTEND
		BZF	R65WAIT		# NOT SET
R61C+L03	TC	INTPRET
		VLOAD
# Page 518
			HIUNITZ
		STORE	SCAXIS		# TRACK AXIS UNIT VECTOR
R61LEM1		RTB	DAD
			LOADTIME	# EXTRAPOLATE FORWARD TO CENTER
			3SECONDS	# SIX SECOND PERIOD.
		STCALL	TDEC1
			LPS20.1		# LOS DETERMINATION + VEH ATTITUDE
		VLOAD
			RRTARGET
		STORE	POINTVSM
		RTB	CALL		# GET DESIRED CDU'S FOR VECPNT1
			READCDUD
			VECPNT1		# COMPUTES FINAL ANGLES FROM PRESENT CDUDS
		STORE	CPHI		# STORE FINAL ANGLES -- CPHI, CTHETA, CPSI
		EXIT
		TC	PHASCHNG
		OCT	04022
		CAF	TRACKBIT	# IS TRACK FLAG SET
		MASK	FLAGWRD1
		EXTEND
		BZF	R65WAIT
		TC	BANKCALL
		CADR	G+N,AUTO	# CHECK FOR AUTO MODE
		CCS	A
		TC	R61C+L04	# NOT IN AUTO
		TC	INTPRET
		VLOAD	CALL
			RRTARGET
			CDU*SMNB
		DLOAD	DSU		# GET PHI -- ARCCOS OF Z-COMPONENT OF LOS
			MPAC +5
			COS15DEG
R61LEM2		BMN	EXIT		# BRANCH -- PHI > 15 DEGREES
			R61C+L05	# PHI GRE 10DEG
		EBANK=	CDUXD
		CAF	EBANK6
		TS	EBANK
		INHINT
		EXTEND
		DCA	CPHI
		DXCH	CDUXD
		CA	CPSI
		TS	CDUZD
		RELINT
		EBANK=	LOSCOUNT
		CAF	EBANK7
		TS	EBANK
		TC	R61C+L06
R61C+L05	EXIT
		INHINT
# Page 519
		TC	IBNKCALL
		FCADR	ZATTEROR
		TC	IBNKCALL
		FCADR	SETMINDB	# REDUCE ATTITUDE ERROR
		TC	DOWNFLAG
		ADRES	3AXISFLG
		TC	UPFLAG
		ADRES	PDSPFLAG	# SET PRIORITY DISPLAY FLAG
		TC	BANKCALL
		CADR	R60LEM
		INHINT
		TC	IBNKCALL
		FCADR	RESTORDB
		TC	PHASCHNG
		OCT	04022
		TC	DOWNFLAG
		ADRES	PDSPFLAG	# RESET PRIORITY DISPLAY FLAG
R61C+L06	CA	FLAGWRD1
		MASK	R61FLBIT
		CCS	A
		TC	R61C+L4
		CCS	R65CNTR
		TC	+2
		TC	R61C+L4		# R65CNTR = 0 - EXIT ROUTINE
		TS	R65CNTR
		CAF	06SEC
		TC	TWIDDLE
		ADRES	R61C+L2
		TC	ENDOFJOB
R61C+L2		CAF	PRIO26
		TC	FINDVAC
		EBANK=	LOSCOUNT
		2CADR	R61C+L01

		TC	TASKOVER
R61C+L04	TC	BANKCALL	# TO CONVERT ANGLES TO FDAI
		CADR	BALLANGS
		TC	R61C+L06
R61C+L4		CAE	GENRET
		TCF	BANKJUMP	# EXIT R61
R61C+L1		CAF	BIT7+9PV	# IS RENDEZVOUS OR P25FLAG SET
		MASK	STATE
		EXTEND
		BZF	ENDOFJOB	# NO -- EXIT ROUTINE AND PROGRAM.
		TC	R61C+L06	# YES EXIT ROUTINE
R65WAIT		TC	POSTJUMP
		CADR	P20LEMWT

BIT7+9PV	OCT	00500
# Page 520
COS15DEG	2DEC	0.96593 B-1

06SEC		DEC	600
PHI		EQUALS	20D
READCDUD	INHINT			# READS DESIRED CDU'S AND STORES IN
		CAF	EBANK6		# MPAC TP EXITS WITH MODE SET TO TP
		XCH	EBANK
		TS	RUPTREG1
		EBANK=	CDUXD
		CA	CDUXD
		TS	MPAC
		EXTEND
		DCA	CDUYD
		DXCH	MPAC +1
		CA	RUPTREG1
		TS	EBANK
		RELINT
		TCF	TMODE
		BLOCK	02
		SETLOC	RADARFF
		BANK

		EBANK=	LOSCOUNT
		COUNT*	$$/RRSUB

# Page 521
# THE FOLLOWING SUBROUTINE RETURNS TO CALLER +2 IF THE ABSOLUTE VALUE OF VALUE OF C(A) IS GREATER THAN THE
# NEGATIVE OF THE NUMBER AT CALLER +1.  OTHERWISE IT RETURNS TO CALLER +3.  MAY BE CALLED IN RUPT OR UNDER EXEC.

MAGSUB		EXTEND
		BZMF	+2
		TCF	+2
		COM

		INDEX	Q
		AD	0
		EXTEND
		BZMF	Q+2		# ABS(A) <= CONST GO TO L+3
		TCF	Q+1		# ABS(A) > CONST GO TO L+2

# Page 522
# PROGRAM NAME:	RRLIMCHK
#
# FUNCTIONAL DESCRIPTION:
#
#	RRLIMCHK CHECKS RR DESIRED GIMBAL ANGLES TO SEE IF THEY ARE WITHIN
# 	THE LIMITS OF THE CURRENT MODE.  INITIALLY THE DESIRED TRUNNION AND
#	SHAFT ANGLES ARE STORED IN ITEMP1 AND ITEMP2.  THE CURRENT RR
# 	ANTENNAE MODE (RADMODES BIT 12) IS CHECKED WHICH IS = 0 FOR
#	MODE 1 AND =1 FOR MODE 2.
#
#	MODE 1 -- THE TRUNNION ANGLE IS CHECKED AT MAGSUB TO SEE IF IT IS
#	BETWEEN -55 AND +55 DEGREES.  IF NOT, RETURN TO L +2.  IF WITHIN LIMITS,
#	THE SHAFT ANGLE IS CHECKED TO SEE IF IT IS BETWEEN -70 AND +59 DEGREES.
#	IF NOT, RETURN TO L +2.  IF IN LIMITS, RETURN TO L +3.
#
#	MODE 2 -- THE SHAFT ANGLE IS CHECKED AT MAGSUB TO SEE IF IT IS
#	BETWEEN -139 AND -25 DEGREES.  IF NOT, RETURN TO L +2.  IF WITHIN
#	LIMITS, THE TRUNNION ANGLE IS CHECKED TO SEE IF IT IS BETWEEN +125
# 	AND -125 (+235) DEGREES.  IF NOT, RETURN TO L +2.  IF IN LIMITS, RETURN
#	TO L +3.
#
# CALLING SEQUENCE:
#
#	L  TC  RLIMCHK (WITH INTERRUPT INHIBITED)
#	L  +1  ADRES  T,S  (DESIRED TRUNNION ANGLE ADDRESS)
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	RADMODES, MODEA, MODEB (OR DESIRED TRUNNION AND SHAFT
#	ANGLES ELSEWHERE IN CONSECUTIVE LOCATIONS -- UNSWITCHED ERASABLE OR
#	CURRENT EBANK).
#
# SUBROUTINES CALLED:	MAGSUB
#
# JOBS OR TASKS INITIATED:  NONE
#
# ALARMS:  NONE
#
# EXIT:		L + 2	(EITHER OR BOTH ANGLES NOT WITHIN LIMITS OF CURRENT MODE)
#		L + 3	(BOTH ANGLES WITHIN LIMITS OF CURRENT MODE)

RRLIMCHK	EXTEND
		INDEX	Q
		INDEX	0
		DCA	0
		INCR	Q
		DXCH	ITEMP1
		LXCH	Q		# L(CALLER +2) TO L.

		CAF	ANTENBIT	# SEE WHICH MODE RR IS IN.
		MASK	RADMODES
		CCS	A
		TCF	MODE2CHK

		CA	ITEMP1		# MODE 1 IS DEFINED AS
# Page 523
		TC	MAGSUB		# 	1. ABS(T) L 55 DEGS.
		DEC	-.30555		# 	2. ABS(S + 5.5 DEGS) L 64.5 DEGS
		TC	L		#		(SHAFT LIMITS AT +59, -70 DEGS)

		CAF	5.5DEGS
		AD	ITEMP2
		TC	MAGSUB
		DEC	-.35833		# 64.5 DEGS
		TC	L
		TC	RRLIMOK		# IN LIMITS.

MODE2CHK	CAF	82DEGS		# MODE 2 IS DEFINED AS
		AD	ITEMP2		#	1. ABS(T) G 125 DEGS.
		TC	MAGSUB		#	2. ABS(S + 82 DEGS) L 57 DEGS
		DEC	-.31667		#		(SHAFT LIMITS AT -25, -139 DEGS)
		TC	L

		CA	ITEMP1
		TC	MAGSUB
		DEC	-.69444		# 125 DEGS

RRLIMOK		INDEX	L
		TC	L		# ( = TC 1 )

5.5DEGS		DEC	.03056
82DEGS		DEC	.45556

# Page 524
# PROGRAM NAME:	SETTRKF
#
# FUNCTIONAL DESCRIPTION:
#
# 	SETTRKF UPDATES THE TRACKER FAIL LAMP ON THE DSKY.
#	INITIALLY THE LAMP TEST FLAG (IMODES33 BIT 1) IS CHECKED.
# 	IF A LAMP TEST IS IN PROGRESS, THE PROGRAM EXITS TO L +1.
#	IF NO LAMP TEST THE FOLLOWING IS CHECKED SEQUENTIALLY:
#		1) RR CDU'S BEING ZEROED, RR CDU OK, AND RR NOT IN
#		   AUTO MODE (RADMODES BITS 13, 7, 2).
#		2) LR VEL DATA FAIL AND NO LR POS DATA (RADMODES BITS
#		   8,5)
#		3) NO RR DATA (RADMODES BIT 4)
#	THE ABSENCE OF ALL THREE SIMULTANEOUSLY IN (1), THE PRESENCE OF BOTH
#	IN (2), AND THE PRESENCE OF (3) RESULTS IN EITHER THE TRACKER FAIL
#	LAMP (DSPTAB +11D BIT 8) BEING TURNED OFF OR IS LEFT OFF.  THEREFORE, THE
#	TRACKER FAIL LAMP IS TURN ON IF:
#		A) RR CDU FAILED WITH RR IN AUTO MODE AND RR CDU'S NOT BEING ZEROED
#		B) N SAMPLES OF LR DATA COULD NOT BE TAKEN IN 2N TRIES WITH
#		   EITHER THE ALT OR VEL INFORMATION
#		C) N SAMPLES OF RR DATA COULD NOT BE OBTAINED FROM 2N TRIES
#		   WITH EITHER THE AL
#
# CALLING SEQUENCE:
#
#	L	TC	SETTRKF
#
# ERASABLE INITIALIZATION REQUIRED:  IMODES33, RADMODES, DSPTAB +11D
#
# SUBROUTINES CALLED:  NONE
#
# JOBS OR TASKS INITIATED:  NONE
#
# ALARMS:  TRACKER FAIL LAMP
#
# EXIT:  L +1 (ALWAYS)

SETTRKF		CAF	BIT1		# NO ACTION IF DURING LAMP TEST
		MASK	IMODES33
		CCS	A
		TC	Q

RRTRKF		CA	BIT8
		TS	L

		CAF	13,7,2		# SEE IF CDU FAILED.
		MASK	RADMODES
		EXTEND
		BZF	TRKFLON		# CONDITION 3 ABOVE.

RRCHECK		CAF	RRDATABT	# SEE IF RR DATA FAILED.
		MASK	RADMODES
# Page 525
		CCS	A
TRKFLON		CA	L
		AD	DSPTAB +11D	# HALF ADD DESIRED AND PRESENT STATES.
		MASK	L
		EXTEND
		BZF	TCQ		# NO CHANGE.

FLIP		CA	DSPTAB +11D	# CAN'T USE LXCH DSPTAB +11D (RESTART PROB)
		EXTEND
		RXOR	LCHAN
		MASK	POSMAX
		AD	BIT15
		TS	DSPTAB +11D
		TC	Q

13,7,2		OCT	10102
ENDRMODF	EQUALS

# Page 526
# PROGRAM NAME:  RRTURNON
#
# FUNCTIONAL DESCRIPTION:
#
#	RRTURNON IS THE TURN-ON SEQUENCE WHICH, ALONG WTIH
#	RRZEROSB, ZEROES THE CDU'S AND DETERMINES THE RR MODE.
#	INITIALLY, CONTROL IS TRANSFERRED TO RRZEROSB FOR THE
#	ACTUAL TURN-ON SEQUENCE.  UPON RETURN THE PROGRAM
#	WAITS 1 SECOND BEFORE REMOVING THE TURN-ON FLAG
#	(RADMODES BIT1) SO THE REPOSITION ROUTINE WON'T
#	INITIATE PROGRAM ALARM 00501.  A CHECK IS THEN MADE
#	TO SEE IF A PROGRAM IS USING THE RR (STATE BIT 7).  IF
#	SO, THE PROGRAM EXITS TO ENDRADAR SO THAT THE RR CDU
#	FAIL FLAG (RADMODES BIT 7) CAN BE CHECKED BEFORE
#	RETURNING TO THE WAITING PROGRAM.  IF NOT, THE PROGRAM EXITS
#	TO TASKOVER.
#
# CALLING SEQUENCE:  WAITLIST TASK FROM RRAUTCHK IF THE RR POWER-ON AUTO
# BIT (CHAN 33 BIT 2) CHANGES TO 0 AND NO PROGRAM WAS USING
# THE RR (STATE BIT 7).
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	RADMODES, STATE
#
# SUBROUTINES CALLED:  RRZEROSB, FIXDELAY, TASKOVER, ENDRADAR
#
# JOBS OR TASKS INITIATED:
#
#	NONE
#
# ALARMS:  NONE (SEE RRZEROSB)
#
# EXIT:  TASKOVER, ENDRADAR (WAITING PROGRAM)

		BANK	24
		SETLOC	P20S1
		BANK

		EBANK=	LOSCOUNT
		COUNT*	$$/RSUB
RRTURNON	TC	RRZEROSB
		TC	FIXDELAY	# WAIT 1 SEC BEFORE REMOVING TURN ON FLAG
		DEC	100		# SO A MONITOR REPOSITION WON'T ALARM.
		CS	TURNONBT
		MASK	RADMODES
		TS	RADMODES
		TCF	TASKOVER
# Page 527
# PROGRAM NAME:  RRZEROSB
#
# FUNCTIONAL DESCRIPTION:
#
#	RRZEROSB IS A CLOSED SUBROUTINE TO ZERO THE RR CDU'S,
#	DETERMINE THE RR MODE, AND TURN ON THE TRACKER FAIL
#	LAMP IF REQUIRED.  INITIALLY THE RR CDU ZERO BIT (CHAN 12
#	BIT 1) IS SET.  FOLLOWING A 20 MILLISECOND WAIT, THE LGC
#	RR CDU COUNTERS (OPTY, OPTX) ARE SET = 0 AFTER
# 	WHICH THE RR CDU ZERO DISCRETE (CHAN 12 BIT 1) IS
#	REMOVED.  A 4 SECOND WAIT IS SET TO ALL THE RR CDU'S
#	TO REPEAT THE ACTUAL TRUNNION AND SHAFT ANGLES.  THE
#	RR CDU ZERO FLAG (RADMODES BIT 13) IS REMOVED.  THE
#	CONTENTS OF OPTY IS THEN CHECKED TO SEE IF THE TRUNNION
#	ANGLE IS LESS THAN 90 DEGREES.  IF NOT, BIT 12 OF
#	RADMODES IS SET = 1 TO INDICATE RR ANTENNA MODE 2.
#	IF LESS THAN 90 DEGREES, BIT 12 OF RADMODES IS SET = 0 TO
#	INDICATE RR ANTENNA MODE 1.  SETTRKF IS THEN CALLED TO
#	SEE IF THE TRACKER FAIL LAMP SHOULD BE TURNED ON.
#
# CALLING SEQUENCE:  L  TC  RRZEROSB  (FROM RRTURNON AND RRZERO)
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	RADMODES (BIT 13 SET), DSPTAB +11D
#
# SUBROUTINES CALLED:  FIXDELAY, MAGSUB, SETTRKF
#
# JOBS OR TASKS INITIATED:
#
#	NONE
#
# ALARMS:  TRAKCER FAIL
#
# EXIT:  L +1 (ALWAYS)

RRZEROSB	EXTEND
		QXCH	RRRET
		CAF	BIT1		# BIT 13 OF RADMODES MUST BE SET BEFORE
		EXTEND			# COMING HERE.
		WOR	CHAN12		# TURN ON ZERO RR CDU
		TC	FIXDELAY
		DEC	2

		CAF	ZERO
		TS	CDUT
		TS	CDUS
		CS	ONE		# REMOVE ZEROING BIT.
		EXTEND
		WAND	CHAN12
		TC	FIXDELAY
		DEC	1000		# RESET FAIL INHIBIT IN 10 SECS. -- D.281

		CS	RCDU0BIT	# REMOVE ZEROING IN PROCESS BIT
# Page 528
		MASK	RADMODES
		TS	RADMODES

		CA	CDUT
		TC	MAGSUB
		DEC	-.5
		TCF	+3		# IF MODE 2.

		CAF	ZERO
		TCF	+2
		CAF	ANTENBIT
		XCH	RADMODES
		MASK	-BIT12
		ADS	RADMODES

		TC	SETTRKF		# TRACKER LAMP MIGHT GO ON NOW.

		TC	RRRET		# DONE.

-BIT12		EQUALS	-1/8		# IN SPROOT

# Page 529
# PROGRAM NAME:  DORREPOS
#
# FUNCTIONAL DESCRIPTION:
#
#	DORREPOS IS A SEQUENCE OF TASKS TO DRIVE THE RENDEZVOUS RADAR
#	TO A SAFE POSITION.  INIITALLY SETRRECR IS CALLED WHERE THE RR
#	ERROR COUNTERS (CHAN 12 BIT 2) ARE ENABLED AND LASTYCMD
#	AND LASTXCMD SET = 0 TO INDICATE THE DIFFERENCE BETWEEN THE
#	DESIRED STATE AND PRESENT STATE OF THE COMMANDS.  THE RR
#	TURN-ON FLAG (RADMODES BIT 1) IS CHECKED AND IF NOT PRESENT,
#	PROGRAM ALARM 00501 IS REQUESTED BEFORE CONTINUING.  IN EITHER
#	CASE, FOLLOWING A 20 MILLISECOND WAIT THE PROGRAM CHECKS THE CURRENT
#	RR ANTENNA MODE (RADMODES BIT 12).  RRTONLY IS THEN CALLED
#	TO DRIVE THE TRUNNION ANGLE TO 0 DEGREES IF IN MODE 1 AND TO 180
#	DEGREES IF IN MODE 2.  UPON RETURN, THE CURRENT RR ANTENNA
#	MODE (RADMODES BIT 12) IS AGAIN CHECKED.  RRSONLY IS THEN
#	CALLED TO DRIVE THE SHAFT ANGLE TO 0 DEGREES IF IN MODE 1 AND TO
#	-90 DEGREES IF IN MODE 2.  IF DURING RRTONLY OR RRSONLY A
#	REMODE HAS BEEN REQUESTED (RADMODES BIT 14), AND ALWAYS
#	FOLLOWING COMPLETION OF RRSONLY, CONTROL IS TRANFERRED TO
#	REPOSRPT.  HERE THE REPOSITION FLAG (RADMODES BIT 11) IS
#	REMOVED.  A CHECK IS THEN MADE ON THE DESIGNATE FLAG (RADMODES
#	BIT 10).  IF PRESENT, CONTROL IS TRANSFERRED TO BEGDES.  IF NOT PRESENT
#	INDICATING NO FURTHER ANTENNA CONTROL REQUIRED, THE RR ERROR
#	COUNTER BIT (CHAN 12 BIT 2) IS REMOVED AND THE ROUTINE EXITS TO
#	TASKOVER.
#
# CALLING SEQUENCE:
#
#	WAITLIST CALL FROM RRGIMON IF TRUNNION AND SHAFT CDU ANGLES
#	NOT WITHIN LIMITS OF CURRENT MODE.
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	RADMODES
#
# SUBROUTINES CALLED
#
#	RRTONLY, RRSONLY, BEGDES (EXIT)
#
# JOBS OR TASKS INITIATED:
#
#	NONE
#
# ALARMS:  NONE
#
# EXIT:  TASKOVER, BEGDES

DORREPOS	TC	SETRRECR	# SET UP RR CDU ERROR COUNTERS.

# ALARM 501 DELETED IN DANCE 279 PER PCR 97.

		TC	FIXDELAY
		DEC	2

		CAF	ANTENBIT	# MANEUVER TRUNNION ANGLE TO NOMINAL POS.
# Page 530
		MASK RADMODES
		CCS	A
		CAF	BIT15		# 0 FOR MODE 1 AND 180 FOR MODE 2.
		TC	RRTONLY

		CAF	ANTENBIT	# NOT PUT SHAFT IN RIGHT POSITION
		MASK	RADMODES
		CCS	A
		CS	HALF		# -90 FOR MODE 2.
		TC	RRSONLY

REPOSRPT	CS	REPOSBIT	# RETURNS HERE FROM RR1AXIS IN REMODE
					# REQUESTED DURING REPOSITION.
		MASK	RADMODES	# REMOVE REPOSITION BIT.
		TS	RADMODES
		MASK	DESIGBIT	# SEE IF SOMEONE IS WAITING TO DESIGNATE.
		CCS	A
		TCF	BEGDES
		CS	BIT2		# IF NO FURTHER ANTENNA CONTROL REQUIRED,
		EXTEND			# REMOVE ERROR COUNTER ENABLE.
		WAND	CHAN12
		TCF	TASKOVER

SETRRECR	CAF	BIT2		# SET UP RR ERROR COUNTERS
		EXTEND
		RAND	CHAN12
		CCS	A		# DO NOT CLEAR LAST COMMAND IF
		TC	Q		# ERROR COUNTERS ARE ENABLED

		TS	LASTYCMD
		TS	LASTXCMD
		CAF	BIT2
		EXTEND
		WOR	CHAN12		# ENABLE RR CDU ERROR COUNTERS.
		TC	Q
# Page 531
# PROGRAM NAME:  REMODE
#
# FUNCTIONAL DESCRIPTION
#
#	REMODE IS THE GENERAL REMODING SUBROUTINE.  IT DRIVES THE
#	TRUNNION ANGLE TO 0 DEGREES IF THE CURRENT MODE IS MODE 1,
#	180 DEGREES FOR MODE 2, THEN DRIVES THE SHAFT ANGLE TO -45
#	DEGREES, AND FINALLY DRIVES THE TRUNNION ANGLE TO -130 DEGREES,
#	TO PLACE THE RR IN MODE 2, -50 DEGREES FOR MODE 1, BEFORE
#	INITIATING 2-AXIS CONTROL.  ALL REMODING IS DONE WITH SINGLE
#	AXIS ROTATIONS (RR1AXIS).  INITIALLY THE RR ANTENNA MODE FLAG
#	(RADMODES BIT 12) IS CHECKED.  CONTROL IS THEN TRANSFERRED TO
#	RRTONLY TO DRIVE THR TRUNNION ANGLE TO 0 DEGREES IF IN MODE 1
#	OR 180 DEGREES IF IN MODE 2.  RRSONLY IS THEN CALLED TO DRIVE
#	THE SHAFT ANGLE TO -45 DEGREES. THE RR ANTENNA MODE FLAG
#	(RADMODES BIT 12) IS CHECKED AGAIN.  CONTROL IS AGAIN
#	TRANSFERRED TO RRTONLY TO DRIVE THE TRUNNION ANGLE TO -130
#	DEGREES TO PLACE THE RR IN MODE 2 IF CURRENTLY IN MODE 1 OR TO
#	-50 DEGREES IF IN MODE 2 TO PLACE THE RR IN MODE 1.  RMODINV
#	IS THEN CALLED TO SET RADMODES BIT 12 TO INDICATE THE NEW
#	RR ANTENNA MODE.  THE REMODE FLAG (RADMODES BIT 14)
#	IS REMOVED TO INDICATE THAT REMODING IS COMPLETE.  THE PROGRAM
#	THEN EXITS TO STDESIG TO BEGIN 2-AXIS CONTROL.
#
# CALLIN SEQUENCE:
#
#	FROM BEGDES WHEN REMODE FLAG (RADMODES BIT 14) IS SET.
#	THIS FLAG MAY BE SET IN RRDESSM AND RRDESNB IF RRLIMCHK
#	DETERMINES THAT THE DESIRED ANGLES ARE WITHIN THE LIMITS OF THE
#	OTHER MODE.
#
# ERASABLE INIITIALIZATION REQUIRED:
#
#	RADMODES
#
# SUBROUTINES CALLED:
#
#	RRTONLY, RRSONL, RMODINV (ACTUALLY PART OF)
#
# JOBS OR TASKS INITIATED:
#
#	NONE
#
# ALARMS:  NONE
#
# EXIT:  STDESIG

REMODE		CAF	ANTENBIT	# DRIVE TRUNNION TO 0 (180)
		MASK	RADMODES	# (ERROR COUNTER ALREADY ENABLED)
		CCS	A
		CAF	BIT15
		TC	RRTONLY

		CAF	-45DEGSR
		TC	RRSONLY
# Page 532
		CS	RADMODES
		MASK	ANTENBIT
		CCS	A
		CAF	-80DEGSR	# GO TO T = -130 (-50).
		AD	-50DEGSR
		TC	RRTONLY

		CS	RADMODES
		MASK	ANTENBIT
		CCS	A
		CAF	BIT15		# GO TO T = -180 (+0).
		TC	RRTONLY

		CS	RADMODES 	# GO TO S = -90 (+0).
		MASK	ANTENBIT
		CCS	A
		CS	HALF
		TC	RRSONLY

		TC	RMODINV

		CS	REMODBIT	# END OF REMODE.
		MASK	RADMODES
		TS	RADMODES

		CAF	DESIGBIT	# WAS REMODE CALLED DURING DESIGNATE?
		MASK	RADMODES	# (BIT10 RADMODES = 1)
		EXTEND
		BZF	RGOODEND	# NO -- RETURN TO CALLER WAITING IN RADSTALL
		TC	STDESIG		# YES -- RETURN TO DESIGNATE
-45DEGSR	=	13,14,15
-50DEGSR	DEC	-.27778
-80DEGSR	DEC	-.44444

RMODINV		LXCH	RADMODES	# INVERT THE MODE STATUS.
		CAF	ANTENBIT
		EXTEND
		RXOR	LCHAN
		TS	RADMODES
		TC	Q

# Page 533
# PROGRAM NAMES:	RRTONLY, RRSONLY
#
# FUNCTIONAL DESCRIPTION:
#
# 	RRTONLY AND RRSONLY ARE SUBROUTINES FOR DOING SINGLE AXIS
#	RR MANEUVERS FOR REMODE AND REPOSITION.  IT DRIVES TO
#	WITHIN 1 DEGREE.  INITIALLY, AT RR1AX2, THE REMODE AND REPOSITION
#	FLAGS (RADMODES BITS 14, 11) ARE CHECKED.  IF BOTH EXIST,
#	THE PROGRAM EXITS TO REPOSRPT (SEE DORREPOS).  THIS INDICATES
#	THAT SOMEONE POSSIBLY REQUESTED A DESIGNATE (RADMODES BIT 10)
#	WHICH REQUIRES A REMODE (RADMODES BIT 14) AND THAT A
#	REPOSITION IS IN PROGRESS (RADMODES BIT 11).  IF NONE
# 	OR ONLY ONE OF THE FLAGS EXIST, REMODE OR REPOSITION, MAGSUB
#	IS CALLED TO SEE IF THE APPROPRIATE ANGLE IS WITHIN 1 DEGREE.  IF YES,
#	CONTROL RETURNS TO THE CALLING ROUTINE.  IF NOT, CONTROL IS
#	TRANSFERRED TO RROUT FOR SINGLE AXIS MANEUVERS WITH THE OTHER
#	ANGLE SET = 0.  FOLLOWING A .5 SECOND WAIT, THE ABOVE PROCEDURE IS
#	REPEATED.
#
# CALLING SEQUENCE:	L-1	CAF	*ANGLE*		(DESIRED ANGLE SCALED PI)
#			L	TC	RRTONLY		(TRUNNION ONLY)
#			RRSONLY				(SHAFT ONLY)
#			RRTONLY IS CALLED BY PREPOS29;
#			RRTONLY AND RRSONLY ARE CALLED BY DORREPOS AND REMODE
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	C(A) = DESIRED ANGLE, RADMODES
#
# SUBROUTINES CALLED:
#
#	FIXDELAY, REPOSRPT, MAGSUB, RROUT
#
# JOBS OR TASKS INITIATED:
#
#	NONE
#
# ALARMS:  NONE
#
# EXIT:	REPOSRPT (REMODE AND REPOSITION FLAGS PRESENT -- RADMODES
#	BITS 14, 11)
#	L+1 (ANGLE WITHIN ONE DEGREE OR RR OUT OF AUTO MODE)

RRTONLY		TS	RDES		# DESIRED TRUNNION ANGLE.
		CAF	ZERO
		TCF	RR1AXIS

RRSONLY		TS	RDES		# SHAFT COMMANDS ARE UNRESOLVED SINCE THIS
		CAF	ONE		# ROUTINE ENTERED ONLY WHEN T = 0 OR 180.

RR1AXIS		TS	RRINDEX
		EXTEND
		QXCH	RRRET
		TCF	RR1AX2

# Page 534
NXTRR1AX	TC	FIXDELAY
		DEC	50		# 2 SAMPLES PER SECOND.

RR1AX2		CS	RADMODES	# IF SOMEONE REQUESTS A DESIGNATE WHICH
		MASK	PRIO22		# REQUIRES A REMODE AND A REPOSITION IS IN
		EXTEND			# PROGRESS, INTERRUPT IT AND START THE
		BZF	REPOSRPT	# REMODE IMMEDIATELY.

		CA	RDES
		EXTEND
		INDEX	RRINDEX
		MSU	CDUT
		TS	ITEMP1		# SAVE ERROR SIGNAL.
		EXTEND
		MP	RRSPGAIN	# TRIES TO NULL .7 OF ERROR OVER NEXT .5
		TS	L
		CA	RADMODES
		MASK	AUTOMBIT
		XCH	ITEMP1		# STORE RR-OUT-OF-AUTO-MODE BIT.
		TC	MAGSUB		# SEE IF WITHIN ONE DEGREE.
		DEC	-.00555		# SCALED IN HALF-REVS.

		CCS	ITEMP1		# NO.  IF RR OUT OF AUTO MODE, EXIT.
		TC	RRRET		# RETURN TO CALLER.

		CCS	RRINDEX		# COMMAND FOR OTHER AXIS IS ZERO.
		TCF	+2		# SETTING A TO 0.
		XCH	L
		DXCH	TANG

		TC	RROUT

		TCF	NXTRR1AX	# COME BACK IN .5 SECONDS.

RRSPGAIN	DEC	.59062		# NULL .7 ERROR IN .5 SEC.

# Page 535
# PROGRAM NAME:  RROUT
#
# FUNCTIONAL DESCRIPTION:
#
#	RROUT RECEIVES RR GYRO COMMANDS IN TANG, TANG +1 IN RR
#	ERROR COUNTER SCALING.  RROUT THEN LIMITS THEM AND
#	GENERATES COMMANDS TO THE CDU TO ADJUST THE ERROR COUNTERS
#	TO THE DESIRED VALUES.  INITIALLY MAGSUB CHECKS THE MAGNITUDE OF
#	THE COMMAND (SHAFT ON 1ST PASS) TO SEE IF IT IS GREATER THAN
#	384 PULSES.  IF NOT, CONTROL IS TRANFERRED TO RROUTLIM TO
#	LIMIT THE COMMAND TO +384 OR -384 PULSES.  THE DIFFERENCE IS
#	THEN CALCULATED BETWEEN THE DESIRED STATE AND TEH PRESENT STATE OF
#	THE ERROR COUNTER AS RECORDED IN LASTYCMD AND LASTXCMD.
#	THE RESULT IS STORED IN OPTXCMD (1ST PASS) AND OPTYCMD (2ND
#	PASS).  FOLLOWING THE SECOND PASS, FOR THE TRUNNION COMMAND, THE
#	OCDUT AND OCDUS ERROR COUNTER DRIVE BITS (CHAN 14 BITS 12, 11)
#	ARE SET.  THIS PROGRAM THEN EXITS TO THE CALLING PROGRAM.
#
# CALLING SEQUENCE:
#
#	L TC RROUT (WITH RUPT INHIBITED) RROUT IS CALLED BY
#	RRTONLY, RRSONLY, AND DODES
#
# ERASABLE INITIALIZATION REQURIED:
#
#	TANG, TANG +1 (DESIRED COMMANDS), LASTYCMD, LASTXCMD
#	(1ST PASS = 0), RR ERROR COUNTER ENAGLE SET (CHAN 12 BIT 2).
#
# SUBROUTINES CALLED:
#
#	MAGSUB
#
# JOBS OR TASKS INITIATED:
#
#	NONE
#
# ALARMS:  NONE
#
# EXIT:  L+1 (ALWAYS)

RROUT		LXCH	Q		# SAVE RETURN
		CAF	ONE		# LOOP TWICE.
RROUT2		TS	ITEMP2
		INDEX	A
		CA	TANG
		TS	ITEMP1		# SAVE SIGN COMMAND FOR LIMITING.

		TC	MAGSUB		# SEE IF WITHIN LIMITS.
-RRLIMIT	DEC	-384
		TCF	RROUTLIM	# LIMIT COMMAND TO MAG OF 384.

SETRRCTR	CA	ITEMP1		# COUNT OUT DIFFERENCE BETWEEN DESIRED
		INDEX	ITEMP2		# STATE AND PRESENT STATE AS RECORDED IN
		XCH	LASTYCMD	# LASTYCMD AND LASTXCMD
		COM
# Page 536
		AD	ITEMP1
		AD	NEG0		# PREVENT +0 IN OUTCOUNTER
		INDEX	ITEMP2
		TS	CDUTCMD

		CCS	ITEMP2		# PROCESS BOTH INPUTS.
		TCF	RROUT2

		CAF	PRIO6		# ENABLE COUNTERS.
		EXTEND
		WOR	CHAN14		# PUT ON CDU DRIVES S AND T
		TC	L		# RETURN.

RROUTLIM	CCS	ITEMP1		# LIMIT COMMAND TO ABS VAL OF 384.
		CS	-RRLIMIT
		TCF	+2
		CA	-RRLIMIT
		TS	ITEMP1
		TCF	SETRRCTR +1

# Page 537
# ROUTINE TO ZERO THE RR CDUS AND DETERMINE THE ANTENNA MODE.

RRZERO		CAF	BIT11+1		# SEE IF MONITOR REPOSITION OR NOT IN AUTO
		MASK	RADMODES	# IF SO, DON'T RE-ZERO CDUS.
		CCS	A
		TCF	RADNOOP		# (IMMEDIATE TASK TO RGOODEND).

		INHINT
		CS	RCDU0BIT	# SET FLAG TO SHOW ZEROING IN PROGRESS.
		MASK	RADMODES
		AD	RCDU0BIT
		TS	RADMODES

		CAF	ONE
		TC	WAITLIST
		EBANK=	LOSCOUNT
		2CADR	RRZ2

		CS	RADMODES	# SEE IF IN AUTO MODE.
		MASK	AUTOMBIT
		CCS	A
		TCF	ROADBACK
		TC	ALARM		# AUTO DISCRETE NOT PRESENT -- TRYING
		OCT	510
ROADBACK	RELINT
		TCF	SWRETURN

RRZ2		TC	RRZEROSB	# COMMON TO TURNON AND RRZERO.
		TCF	ENDRADAR

BIT11+1		OCT	02001

# Page 538
# PROGRAM NAME:  RRDESSM
#
# FUNCTIONAL DESCRIPTION:
#
# 	THIS INTERPRETIVE ROUTINE WILL DESIGNATE, IF DESIRED ANGLES ARE
#	WITHIN THE LIMITS OF EITHER MODE, TO A LINE-OF-SIGHT (LOS) VECTOR
#	(HALF-UNIT) KNOWN WITH RESPECT TO THE STABLE MEMBER PRESENT
#	ORIENTATION.  INITIALLY THE IMU CDU'S ARE READ AND CONTROL
#	TRANSFERRED TO SMNB TO TRANSFORM THE LOS VECTOR FROM STABLE
#	MEMBER TO NAVIGATION BASE COORDINATES (SEE STG MEMO 699)
#	RRANGLES IS THEN CALLED TO CALCULATE THE RR GIMBAL ANGLES,
#	TRUNNION AND SHAFT, FOR BOT THE PRESENT AND ALTERNATE MODE.
#	RRLIMCHK IS CALLED TO SEE IF THE ANGLES CALCULATED FOR THE
#	PRESENT MODE ARE WITHIN LIMITS.  IF WITHIN LIMITS, THE RETUREN
#	LOCATION IS INCREMENTED, INASMUCH AS NO VEHICLE MANEUVER IS
#	REQUIRED, BEFORE EXITING TO STARTDES.  IF NOT WITHIN THE LIMITS OF THE
#	CURRENT MODE, TRYSWS IS CALLED.  FOLLOWING INVERTING OF THE RR
#	ANTENNA MODE FLAG (RADMODES BIT 12), RRLIMCHK IS CALLED
#	TO SEE IF THE ANGLES CALCULATED FOR THE ALTERNATE MODE ARE WITHIN
#	LIMITS.  IF YES, THE RR ANTENNA MODE FLAG IS AGAIN INVERTED,
#	THE REMODE FLAG (RADMODES BIT 14) SET, AND THE RETURN LOCATION
#	INCREMENTED, TO INDICATE NO VEHICLE MANEUVER IS REQUIRED, BEFORE
#	EXITING TO STARTDES.  IF THESE ANGLES ARE NOT WITHIN LIMITS
#	OF THE ALTERNATE MODE, THE RR ANTENNA MODE FLAG (RADMODES
#	BIT 12) IS INVERTED BEFORE RETURNING DIRECTLY TO THE CALLING PROGRAM
# 	TO INDICATE THAT A VEHICLE MANEUVER IS REQUIRED.
#
# CALLING SEQUENCE:
#
#	L	STCALL	RRTARGET	(LOS HALF-UNIT VECTOR IN SM COORDINATES)
#	L+1	RRDESM
#	L+2	BASIC			(VEHICLE MANEUVER REQUIRED)
#	L+3	BASIC			(NO VEHICLE MANEUVER REQUIRED)
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	RRTARGET, RADMODES
#
# SUBROUTINES CALLED:
#
#	READCDUS, SMNB, RRANGLES, RRLIMCHK, TRYSWS (ACTUALLY
#	PART OF), RMODINV
#
# JOBS OR TASKS INITIATED:
#
#	NONE
#
# ALARMS:  NONE
#
# EXIT:  L+2 (NEITHER SET OF ANGLES ARE WITHIN LIMITS OF RELATED MODE)
# STARTDES (DESIGNATE POSSIBLE AT PRESENT VEHICLES ATTITUDE -- RETURNS
# TO L+3 FROM STARTDES)

RRDESSM		STQ	CLEAR
			DESRET
# Page 539
			RRNBSW
		CALL			# COMPUTES SINES AND COSINES, ORDER Y Z X
			CDUTRIG
		VLOAD	CALL		# LOAD VECTOR AND CALL TRANSFORMATION
			RRTARGET
			*SMNB*

		CALL			# GET RR GIMBAL ANGLES IN PRESENT AND
			RRANGLES	# ALTERNATE MODE.
		EXIT

		INHINT
		TC	RRLIMCHK
		ADRES	MODEA		# CONFIGURATION FOR CURRENT MODE.
		TC	+3		# NOT IN CURRENT MODE
OKDESSM		INCR	DESRET		# INCREMENT SAYS NO VEHICLE MANEUVER REQ.
		TC	STARTDES	# SHOW DESIGNATE REQUIRED
		CS	FLAGWRD8
		MASK	SURFFBIT	# CHECK IF ON LUNAR SURFACE (SURFFLAG=P22F)
		EXTEND
		BZF	NORDSTAL	# BRANCH -- YES -- CANNOT DESIGNATE IN MODE 2
		TC	TRYSWS

LUNDESCH	CS	FLAGWRD8	# OVERFLOW RETURN FROM RRANGLES
		MASK	SURFFBIT	# CHECK IF ON LUNAR SURFACE
		EXTEND
		BZF	NORDSTAL	# BRANCH -- YES -- RETURN TO CALLER -- ALARM 527
		CA	STATE
		MASK	RNDVZBIT
		CCS	A		# TEST RNDVZFLG
		TC	NODESSM		# NOT ON MOON -- CALL FOR ATTITUDE MANEUVER
		TCF	ENDOFJOB	# ... BUT NOT IN R29.

# Page 540
# PROGRAM NAME:  STARTDES
#
# FUNCTIONAL DESCRIPTION:
#
#	STARTDES IS ENTERED WHEN WE ARE READY TO BEGIN DESIGNATION.
#	BIT 14 OF RADMODES IS ALREADY SET IF A REMODE IS REQUIRED.
#	AT THIS TIME, THE RR ANTENNA MAY BE IN A REPOSITON
#	OPERATION.  IN THIS CASE, IF A REMODE IS REQUIRED IT MAY HAVE
#	ALREADY BEGUN BUT IN ANY CASE THE REPOSITION WILL BE INTERRUPTED.
#	OTHERWISE, THE REPOSITION WILL BE COMPLETED BEFORE 2-AXIS
#	DESIGNATION BEGINS.  INITIALLY DESCOUNT IS SET = 60 TO INDICATE
#	THAT 30 SECONDS WILL BE ALLOWED FOR THE RR DATA GOOD INBIT
#	(CHAN 33 BIT 4) IF LOCK-ON IS DESIRED (STATE BIT 5).  BIT 10
#	OF RADMODES IS SET TO SHOW THAT A DESIGNATE IS REQUIRED.
#	THE REPOSITON FLAG (RADMODES BIT 11) IS CHECKED.  IF SET,
#	THE PROGRAM EXITS TO L+3 OF THE CALLING PROGRAM (SEE RRDESSM
#	AND RRDESNB).  THE PROGRAM WILL BEGIN DESIGNATING TO THE DESIRED
#	ANGLES FOLLOWING THE REPOSITON OR REMODE IF ONE WAS
#	REQUESTED.  IF THE REPOSITON FLAG IS NOT SET, SETRRECR IS CALLED
#	WITH SETS THE RR ERROR COUNTER ENABLE BIT (CHAN 12 BIT 2)
# 	AND SETS LASTYCMD AND LASTXCMD = 0 TO INDICATE THE
#	DIFFERENCE BETWEEN THE PRESENT AND DESIRED STATE OF THE ERROR
#	COUNTERS.  A 20 MILLISECOND WAITLIST CALL IS SET FOR BEGDES
#	AFTER WHICH THE PROGRAM EXITS TO L+3 OF TEH CALLING PROGRAM.
#
# CALLING SEQUENCE:
#
#	FROM RRDESSM AND RRDESNB WHEN ANGLES WITHIN LIMITS.
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	RADMODES, (SEE DODES)
#
# SUBROUTINES CALLED
#
#	SETRRECR, WAITLIST
#
# JOBS OR TASKS INITIATED:
#
#	BEGDES
#
# ALARMS:  NONE
#
# EXIT:	L+3 OF CALLING PROGRAM (SEE RRDESSM)
#	L+2 OF CALLING PROGRAM (SEE RRDESNB)

STARTDES	INCR	DESRET
		CS	RADMODES
		MASK	DESIGBIT
		ADS	RADMODES
		MASK	REPOSBIT	# SEE IF REPOSITIONING IN PROGRESS.
		CCS	A
		TCF	DESRETRN	# ECTR ALREADY SET UP.

		TC	SETRRECR	# SET UP ERROR COUNTERS.
# Page 541
		CAF	TWO
		TC	WAITLIST
		EBANK=	LOSCOUNT
		2CADR	BEGDES

DESRETRN	CA	RADCADR		# FIRST PASS THRU DESIGNATE
		EXTEND
		BZF	DESRTRN		# YES	SET EXIT
		TC	ENDOFJOB	# NO
DESRTRN		RELINT
		INCR	DESRET
		CA	DESRET
		TCF	BANKJUMP

NORDSTAL	CAF	ZERO		# ZERO RADCADR TO WIPE OUT ANYONE
		TS	RADCADR		# WAITING IN RADSTALL SINCE WE ARE NOW
		TCF	DESRTRN		# RETURNING TO P20 AND MAY DO NEW RADSTALL

# Page 542
# SEE IF RRDESSM CAN BE ACCOMPLISHED AFTER A REMODE.

TRYSWS		TC	RMODINV		# (NOTE RUPT INHIBIT)
		TC	RRLIMCHK	# TRY DIFFERENT MODE.
		ADRES	MODEB
		TCF	NODESSM		# VEHICLE MANEUVER REQUIRED

		TC	RMODINV		# RESET BIT12
		CAF	REMODBIT	# SET FLAG FOR REMODE.
		ADS	RADMODES

		TCF	OKDESSM

NODESSM		TC	RMODINV		# RE-INVERT MODE AND RETURN
		INCR 	DESRET		# TO CALLER +2
		TCF	NORDSTAL

MAXTRYS		DEC	60

# Page 543
# DESIGNATE TO SPECIFIC RR GIMBAL ANGLES (INDEPENDENT OF VEHICLE MOTION).  ENTER WITH DESIRED ANGLES IN
# TANG AND TANG +1.

RRDESNB		TC	MAKECADR
		TS	DESRET

		TC	DOWNFLAG	# RESET FLAG TO PREVENT DODES FROM GOING
		ADRES	LOSCMFLG	# BACK TO R21
		CA	MAXTRYS		# SET TIME LIMIT COUNTER
		TS	DESCOUNT	# FOR DESIGNATE
		INHINT			# SEE IF CURRENT MODE OK.
		TC	RRLIMNB		# DO SPECIAL V41 LIMIT CHECK
		ADRES	TANG
		TCF	TRYSWN		# SEE IF IN OTHER MODE.

OKDESNB		RELINT
		EXTEND
		DCA	TANG
		DXCH	TANGNB
		TC	INTPRET

		CALL			# GET LOS IN NB COORDS.
			RRNB
		STORE	RRTARGET

		SET	EXIT
			RRNBSW

		INHINT
		TCF	STARTDES +1
TRYSWN		TC	RMODINV		# SEE IF OTHER MODE WILL DO.
		TC	RRLIMNB		# DO SPECIAL V41 LIMIT CHECK
		ADRES	TANG
		TCF	NODESNB		# NOT POSSIBLE.

		TC	RMODINV
		CAF	REMODBIT	# CALL FOR REMODE.
		ADS	RADMODES
		TCF	OKDESNB

NODESNB		TC	RMODINV		# REINVERT MODE BIT.
		TC	ALARM		# BAD INPUT ANGLES.
		OCT	502
		TC	CLRADMOD
		TC	ENDOFJOB	# AVOID 503 ALARM.

RRLIMNB		INDEX	Q		# THIS ROUTINE IS IDENTICAL TO RRLIMCHK
		CAF	0		# EXCEPT THAT THE MODE 1 SHAFT LOWER
		INCR	Q		# LIMIT IS -85 INSTEAD OF -70 DEGREES
		EXTEND
# Page 544
		INDEX	A		# READ GIMBAL ANGLES INTO ITEMP STORAGE
		DCA	0
		DXCH	ITEMP1
		LXCH	Q		# L(CALLER +2) TO L

		CAF	ANTENBIT	# SEE WHICH MODE RR IS IN
		MASK	RADMODES
		CCS	A
		TCF	MODE2CHK	# MODE 2 CAN USE RRLIMCHK CODING
		CA	ITEMP1
		TC	MAGSUB		# MODE 1 IS DEFINED AS
		DEC	-.30555		# 1 	ABS(T) L 55 DEGS
		TC	L		# 2 	SHAFT LIMITS AT +59, -85 DEGS

		CA	ITEMP2		# LOAD SHAFT ANGLE
		EXTEND
		BZMF	NEGSHAFT	# IF NEGATIVE SHAFT ANGLE, ADD 20.5 DEGS
		AD	5.5DEGS
SHAFTLIM	TC	MAGSUB
		DEC	-.35833		# 64.5 DEGREES
		TC	L		# NOT IN LIMITS
		TC	RRLIMOK		# IN LIMITS
NEGSHAFT	AD	20.5DEGS	# MAKE NEGATIVE SHAFT LIMIT -85 DEGREES
		TCF	SHAFTLIM

20.5DEGS	DEC	.11389

# Page 545
# PROGRAM NAME:  BEGDES
#
# FUNCTIONAL DESCRIPTION:
#
#	BEGDES CHECKS VARIOUS DESIGNATE REQUESTS AND REQUESTS THE
#	ACTUAL RR DESIGNATION.  INITIALLY A CHECK IS MADE TO SEE IF A
# 	REMODE (RADMODES BIT 14) IS REQUESTED OR IN PROGRESS.  IF SO,
#	CONTROL IS TRANFERRED TO STDESIG AFTER ROUTINE REMODE IS
#	EXECUTED.  IF NO REMODE, STDESIG IS IMMEDIATELY CALLED WHERE
#	FIRST THE REPOSITION FLAG (RADMODES BIT 11) IS CHECKED.  IF
#	PRESENT, THE DESIGNATE FLAG (RADMODES BIT 10) IS REMOVED
#	AFTER WHICH THE PROGRAM EXITS TO RDBADEND.  IF THE REPOSITION
#	FLAG IS NOT PRESET, THE CONTINUOUS DESIGNATE FLAG (RADMODES
#	BIT 15) IS CHECKED.  IF PRESENT, AN EXECUTIVE CALL IS IMMEDIATELY
#	MADE FOR DODES AFTER WHICH A .5 SECOND WAIT IS INITIATED BEFORE
#	REPEATING AT STDESIG.  IF THE RR SEARCH ROUTINE (LRS24.1) IS DESIGNATING
#	TO A NEW POINT (NEWPTFLG SET) THE CURRENT DESIGNATE TASK IS TERMINATED.
#	IF CONTINUOUS DESIGNATE IS NOT WANTED, THE DESIGNATE FLAG (RADMODES
#	BIT 10) IS CHECKED.  IF NOT PRESENT, THE PROGRAM EXITS TO ENDRADAR TO
#	CHECK RR CDU FAIL BEFORE RETURNING TO THE CALLING PROGRAM.  IF DESIGNATE
#	IS STILL REQUIRED, DESCOUNT IS CHECKED TO SEE IF THE 30 SECONDS HAS
#	EXPIRED BEFORE RECEIVING THE RR DATA GOOD (CHAN 33 BIT 4)
#	SIGNAL.  IF OUT OF TIME, PROGRAM ALARM 00503 IS REQUESTED, THE
#	RR AUTO TRACKER ENABLE AND RR ERROR COUNTER ENABLE
#	(CHAN 12 BITS 14,2) BITS REMOVED, AND THE DESIGNATE FLAG
#	(RADMODES BIT 10) REMOVED BEFORE EEXITING TO RDBADEND.  IF
#	TIME HAS NOT EXPIRED, DESCOUNT IS DECREMENTED, THE
#	EXECUTIVE CALL MADE FOR DODES, AND A .5 SECOND WAIT INITIATED
#	BEFORE REPEATING THIS PROCEDURE AT STDESIG.
#
# CALLING SEQUENCE:
#
#	WAITLIST CALL FROM STARTDES
#	TCF BEGDES FROM DORREPOS
#	TC STDESIG RETURNING, FROM REMODE
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	DESCOUNT, FINDVAC
#
# JOBS OR TASKS INITIATED:  DODES
#
# ALARMS:  PROGRAM ALARM 00503 (30 SECONDS HAVE EXPIRED) WITH NO RR DATA
# GOOD (CHAN 33 BIT 4) RECEIVED WHEN LOCK-ON (STATE BIT 5) WAS REQUESTED.
#
# EXIT:  	TASKOVER (SEARCH PATTERN DESIGNATING TO NEW POINT)
# 		ENDRADAR (NO DESIGNATE -- RADMODES BIT 10)
#		RDBADEND (REPOSITION OR 30 SECONDS EXPIRED)

BEGDES		CS	RADMODES
# Page 546
		MASK	REMODBIT
		CCS	A
		TC	STDESIG
		TC	REMODE
DESLOOP		TC	FIXDELAY	# 2 SAMPLES PER SECOND.
		DEC	50

STDESIG		CAF	REPOSBIT
		MASK	RADMODES	# SEE IF GIMBAL LIMIT MONITOR HAS FOUND US
		CCS	A		# OUT OF BOUNDS.  IF SO, THIS BIT SHOWS A
		TCF	BADDES		# REPOSITION TO BE IN PROGRESS.

		CCS	RADMODES	# SEE IF CONTINUOUS DESIGNATE WANTED.
		TCF	+3		# IF SO, DON'T CHECK BIT 10 TO SEE IF IN
		TCF	+2		# LIMITS BUT GO RIGHT TO FINDVAC ENTRY.
		TCF	MOREDES +1

		CS	RADMODES	# IF NON-CONTINUOUS, SEE IF END OF
		MASK	DESIGBIT	# PROBLEM (DATA GOOD IF LOCK-ON WANTED OR
		CCS	A		# WITHIN LIMITS IF NOT).  IF SO, EXIT AFTER
		TCF	ENDRADAR	# CHECKING RR CDU FAIL.

STDESIG1	CCS	DESCOUNT	# SEE IF THE TIME LIMIT HAS EXPIRED
		TCF	MOREDES

		CS	B14+B2		# IF OUT OF TIME, REMOVE ECR ENABLE + TRKR
		EXTEND
		WAND	CHAN12
BADDES		CS	DESIGBIT	# REMOVE DESIGNATE FLAG
		MASK	RADMODES
		TS	RADMODES
		TCF	RDBADEND

MOREDES		TS	DESCOUNT
		CAF	PRIO26		# UPDATE GYRO TORQUE COMMANDS.
		TC	FINDVAC
		EBANK=	LOSCOUNT
		2CADR	DODES

		TCF	DESLOOP

B14+B2		OCT	20002

# Page 547
# PROGRAM NAME:  DODES
#
# FUNCTIONAL DESCRIPTION:
#
#	DODES CALCULATES AND REQUESTS ISSUANCE OF RR GYRO TORQUE
#	COMMANDS.  INITIALLY THE CURRENT RR CDU ANGLES ARE STORED AND
#	THE LOS HALF-UNIT VECTOR TRANSFORMED FROM STABLE MEMBER TO
#	NAVIGATION BASE COORDINATES VIA SMNB IF NECESSARY.  THE
#	SHAFT AND TRUNNION COMMANDS ARE THEN CALCULATED AS FOLLOWS:
#		+ SHAFT = LOS . (COS(S), 0, -SIN(S))  (DOT PRODUCT)
#		- TRUNNION = LOS . (SIN(T)SIN(S), COS(T), SIN(T)COS(S))
#	THE SIGN OF THE SHAFT COMMAND IS THEN REVERSED IF IN MODE 2
#	(RADMODES BIT 12) BECAUSE A RELAY IN THE RR REVERSES THE
#	POLARITY OF THE COMMAND.  AT RRSCALUP EACH COMMAND IS
#	SCALED AND IF EITHER, OR BOTH, OF THE COMMANDS IS GREATER THAN
#	.5 DEGREES, MPAC +1 IS SET POSITIVE.  IF A CONTINUOUS DESIGNATE
#	(RADMODES BIT 15) IS DESIRED AND THE SEARCH ROUTINE IS NOT OPERATING,
#	THE RR AUTO TRACKER ENABLE BIT (CHAN 12 BIT 14) IS CLEARED AND RROUT
#	CALLED TO PUT OUT THE COMMANDS PROVIDED NO REPOSITION (RADMODES BIT 11)
#	IS IN PROGRESS.  IF A CONTINUOUS DESIGNATE AND THE SEARCH ROUTINE IS
#	OPERATING (SRCHOPT FLAT SET) THE TRACK ENABLE IS NOT CLEARED.  IF NO
#	CONTINUOUS DESIGNATE AND BOTH COMMANDS ARE NOT LESS THAN .5 DEGREES AS
#	INDICATED BY MPAC +1, THE RR AUTO TRACKER ENABLE BIT (CHAN 12 BIT 14) IS
# 	CLEARED AND RROUT CALLED TO PUT OUT THE COMMANDS PROVIDED NO REPOSITON
#	(RADMODES BIT 11) IS IN PROGRESS.  IF BOTH COMMANDS ARE LESS THAN .5
#	DEGREES AS INDICATED BY MPAC+1, THE RR AUTO TRACKER ENABLE BIT
#	(CHAN 12 BIT 14) IS CLEARED AND RROUT CALLED TO PUT OUT THE
#	COMMANDS PROVIDED NO REPOSITION (RADMODES BIT 11) IS IN
#	PROGRESS.  IF BOTH COMMANDS ARE LESS THAN .5 DEGREES, THE
#	LOCK-ON FLAG (STATE BIT 5) IS CHECKED.  IF NOT PRESETN, THE
#	DESIGNATE FLAG (RADMODES BIT 10) IS CLEARED, AND ENDOFJOB
#	CALLED.  IF LOCK-ON IS DESIRED, THE RR AUTO TRACKER (CHAN 12
#	BIT 14) IS ENABLED FOLLOWED BY A CHECK OF THE RECEIPT OF THE
#	RR DATA GOOD (CHAN 33 BIT 4) SIGNAL.  IF RR DATA GOOD
#	PRESENT, THE DESIGNATE FLAG (RADMODES BIT 10) IS CLEARED,
# 	THE RR ERROR COUNTER ENABLE BIT (CHAN 12 BIT 2) IS CLEARED,
#	AND ENDOFJOB CALLED.  IF RR DATA GOOD IS NOT PRESENT, RROUT
#	IS CALLED TO PUT OUT THE COMMANDS PROVIDED NO REPOSITION
#	(RADMODES BIT 11) IS IN PROGRESS AFTER WHICH THE JOB IS TERMINATED
#	VIA ENDOFJOB.
#
# CALLING SEQUENCE:
#
#	EXECUTIVE CALL EVERY .5 SECONDS FROM BEGDES.
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	RRTARGET (HALF-UNIT LOS VECTOR IN EITHER SM OR NB COORDINATES),
#	LOKONSW (STATE BIT 5), RRNBSW (STATE BIT 6), RADMODES
#
# SUBROUTINES CALLED:
#
# 	READCDUS, SMNB, CDULOGIC, MAGSUB, RROUT
# Page 548
#
# JOBS OR TASKS INITIATED:
#
#	NONE
#
# ALARMS:  NONE
#
# EXIT:  ENDOFJOB (ALWAYS)

DODES		EXTEND
		DCA	CDUT
		DXCH	TANG

		TC	INTPRET

		SETPD	VLOAD
			0
			RRTARGET
		BON	VXSC
			RRNBSW
			DONBRD		# TARGET IN NAV-BASE COORDINATES
			MLOSV		# MULTIPLY UNIT LOS BY MAGNITUDE
		VSL1	PDVL
			LOSVEL
		VXSC	VAD		# ADD ONE SECOND RELATIVE VELOCITY TO LOS
			MCTOMS
		UNIT	CALL
			CDUTRIG
		CALL
			*SMNB*

DONBRD		STODL	32D
			TANG +1
		RTB	PUSH		# SHAFT COMMAND = V(32D).(COS(S), 0,
			CDULOGIC	#	-SIN(S)).
		SIN	PDDL		# SIN(S) TO 0 AND COS(S) TO 2.
		COS	PUSH
		DMP	PDDL
			32D
			36D
		DMP	BDSU
			0
		STADR
		STORE	TANG +1		# SHAFT COMMAND

		SLOAD	RTB
			TANG
			CDULOGIC
		PUSH	COS		# COS(T) TO 4.
		PDDL	SIN
		PUSH	DMP		# SIN(T) TO 6.
			2
# Page 549
		SL1	PDDL		# DEFINE VECTOR U =	[SIN(T)SIN(S)]
			4		#			[   COS(T)   ]
		PDDL	DMP		#			[SIN(T)COS(S)]
			6
			0
		SL1	VDEF
		DOT	EXIT		# DOT U WITH LOS TO GET TRUNNION COMMAND.
			32D

# Page 550
# AT THIS POINT WE HAVE A ROTATION VECTOR IN DISH AXES LYING IN THE TS PLANE.  CONVERT THIS TO A
# COMMANDED RATE AND ENABLE THE TRACKER IF WE ARE WITHIN .5 DEGREES OF THE TARGET.

		CS	MPAC		# DOT WAS NEGATIVE OF DESREG ANGLE.
		EXTEND
		MP	RDESGAIN	# SCALING ON INPUT ANGLE WAS 4 RADIANS.
		TS	TANG		# TRUNNION COMMAND.
		CS	RADMODES	# A RELAY IN THE RR REVERSES POLARITY OF
		MASK	BIT12		# THE SHAFT COMMANDS IN MODE 2 SO THAT A
		EXTEND			# POSITIVE TORQUE APPLIED TO THE SHAFT
		BZF	+3		# GYRO CAUSES A POSITIVE CHANGE IN THE
		CA	TANG +1		# SHAFT ANGLE.  COMPENSATE FOR THIS SWITCH
		TCF	+2		# BY CHANGING THE POLARITY OF OUR COMMAND.
 +3		CS	TANG +1
		EXTEND
		MP	RDESGAIN	# SCALING ON INPUT ANGLE WAS 4 RADIANS.
		TS	TANG +1		# SHAFT COMMAND FOR RROUT
		TC	INTPRET

		DLOAD	DMP
			2		# COS(S).
			4		# COS(T).
		SL1	PDDL		# Z COMPONENT OF URR.
		DCOMP	PDDL		# Y COMPONENT = -SIN(T)
			0		# SIN(S).
		DMP	SL1
			4		# COS(T).
		VDEF	BON		# FORM URR IN NB AXES.
			RRNBSW		# BYPASS NBSM CONVERSION IN VERB 41
			+3
		CALL
			*NBSM*		# GET URR IN SM AXES.
		DOT	EXIT
			RRTARGET	# GET COSIN OF ANGLE BETWEEN RR AND LOS

		EXTEND
		DCS	COS1/2DG
		DAS	MPAC		# DIFFERENCE OF COSINES, SCALED B-2.
		CCS	MPAC
		CA	ZERO		# IF COS ERROR BIGGER, ERROR IS SMALLER
		TCF	+2
		CA	ONE
		TS	MPAC +1		# ZERO IF RR IS POINTED OK, ONE IF NOT.
# Page 551
# SEE IF TRACKER SHOULD BE ENABLED OR DISABLED.

		CCS	RADMODES	# IF CONTINUOUS DESIGNATE WANTED, PUT OUT
		TCF	SIGNLCHK	# COMMANDS WITHOUT CHECKING MAGNITUDE OF
		TCF	SIGNLCHK	# ERROR SIGNALS
		TCF	DORROUT
SIGNLCHK	CCS	MPAC +1		# SEE IF BOTH AXES WERE WITHIN .5 DEGS.
		TCF	DGOODCHK
		CS	STATE		# IF WITHIN LIMITS AND NO LOCK-ON WANTED,
		MASK	LOKONBIT	# PROBLEM IS FINISHED.
		CCS	A
		TCF	RRDESDUN

		CAF	BIT14		# ENABLE THE TRACKER
		EXTEND
		WOR	CHAN12

DGOODCHK	CAF	BIT4		# SEE IF DATA GOOD RECEIVED YET
		EXTEND
		RAND	CHAN33
		CCS	A
		TCF	DORROUT

RRDESDUN	CS	BIT10		# WHEN PROBLEM DONE, REMOVE BIT 10 SO NEXT
		MASK	RADMODES	# WAITLIST TASK WE WILL GO TO RGOODEND.
		INHINT
		TS 	RADMODES

		TC	DOWNFLAG	# RESET LOSCMFLG TO PREENT A
		ADRES 	LOSCMFLG	# RECOMPUTATION OF LOS AFTER DATA GOOD
		CS	BIT2		# TURN OFF ENABLE RR ERROR COUNTER
		EXTEND
		WAND	CHAN12
		TCF	ENDOFJOB	# WITH ECTR DISABLED.

DORROUT		CA	FLAGWRD2	# IF BOTH LOSCMFLAG AND SEARCH FLAG ARE
		MASK	BIT12,14	# ZERO, BYPASS VELOCITY ADJUSTMENT TO LOS
		EXTEND
		BZF	NOTP20
		TC	INTPRET
		VLOAD	VXSC		# MULTIPLY UNIT LOS BY MAGNITUDE
			RRTARGET
			MLOSV
		VSL1	PUSH
		VLOAD	VXSC		# ADD .5 SEC. OF VELOCITY
			LOSVEL		# TO LOS VECTOR
			MCTOMS
		VSR1	VAD
		UNIT
		STODL	RRTARGET	# STORE VELOCITY-CORRECTED LOS (UNIT)
# Page 552
			36D
		STORE	MLOSV		# AND STORE MAGNITUDE
		EXIT
NOTP20		INHINT
		CS	RADMODES	# PUT OUT COMMAND UNLESS MONITOR
		MASK	REPOSBIT	# REPOSITION HAS TAKEN OVER
		CCS	A
		TC	RROUT

		CA	FLAGWRD2
		MASK	LOSCMBIT	# IF LOSCMFLG NOT SET, DON'T TEST
		EXTEND			# LOS COUNTER
		BZF	ENDOFJOB
		CCS	LOSCOUNT	# TEST LOS COUNTER TO SEE IF TIME TO GET
		TC	DODESEND	# A NEW LOS
		INHINT
		TC	KILLTASK	# YES -- KILL TASK WHICH SCHEDULES DODES
		CADR	DESLOOP +2
		RELINT
		CCS	NEWJOB
		TC	CHANG1
		TC	BANKCALL
		CADR	R21LEM2

DODESEND	TS	LOSCOUNT
		TC	ENDOFJOB

; ============================================================================
; RADAR CONSTANTS AND GAIN SETTINGS
;
; Following constants are used throughout radar processing routines:
; - RDESGAIN: Rendezvous radar designation gain (nulls 0.5° error in 0.5 sec)
; - COS1/2DG: Cosine of 0.5 degrees for angle comparisons
; - MCTOMS: Conversion factor from mission timer counts to milliseconds
;
; These calibration values ensure accurate radar tracking performance
; during rendezvous operations when relative motion is critical.
; ============================================================================

RDESGAIN	DEC	.53624		# TRIES TO NULL .5 ERROR IN .5 SEC.
BIT12,14	EQUALS	PRIO24		# OCT 24000
COS1/2DG	2DEC	.999961923 B-2	# COSINE OF 0.5 DEGREES.
MCTOMS		2DEC	100 B-13

; ============================================================================
; TRANSITION: From R22LEM Data Processing to Radar Read Initialization
;
; Having completed the main R22 data read and processing logic, the code
; now defines the fundamental radar reading routines. These initialization
; routines set up the hardware interface for both Landing Radar (LR) and
; Rendezvous Radar (RR) measurements. During Apollo 11's rendezvous, these
; routines coordinated the six radar measurement types: LR altitude, three
; LR velocity components, RR range, and RR range-rate.
; ============================================================================

# Page 553
# RADAR READ INITIALIZATION
#
# RADAR DATA READ BY A BANKCALL FOR THE APPROPRIATE LEAD-IN BELOW.

; RADAR MEASUREMENT ENTRY POINTS
; Each radar type has a dedicated entry point that initializes the reading
; sequence. The INITREAD routine performs common setup, then specific radar
; channels are selected via the octal code following the TC instruction.
;
; LRALT: Landing Radar altitude (single sample mode)
; LRVELZ/Y/X: Landing Radar velocity components (5-sample averaging available)
; RRRDOT: Rendezvous Radar range rate (single sample)
; RRRANGE: Rendezvous Radar range (single sample)
;
; During rendezvous, these measurements update the relative navigation state
; between LM and CSM. The RR measurements were critical when Eagle climbed
; back to Columbia after ascent from the lunar surface on July 21, 1969.

; Landing Radar altitude channel - single sample per reading
; Measures vertical distance to lunar surface during descent and landing
; Channel selection code OCT 17 (ALLREAD) activates altitude sensor
LRALT		TC	INITREAD -1	# ONE SAMPLE PER READING.
ALLREAD		OCT	17

; Landing Radar velocity Z component (vertical) - multiple samples supported
; Measures rate of descent toward lunar surface during powered descent
; Channel selection code OCT 16 activates vertical velocity beam
LRVELZ		TC	INITREAD
		OCT	16

; Landing Radar velocity Y component (lateral) - multiple samples supported
; Measures lateral motion across lunar surface during approach
; Channel selection code OCT 15 activates lateral velocity beam
LRVELY		TC	INITREAD
		OCT	15

; Landing Radar velocity X component (forward) - multiple samples supported
; Measures forward motion toward landing site during final approach
; Channel selection code OCT 14 activates forward velocity beam
LRVELX		TC	INITREAD
		OCT	14

; Rendezvous Radar range rate - single sample per reading
; Measures closing velocity between LM and CSM during rendezvous operations
; Channel selection code OCT 12 activates range-rate measurement
; Critical during Eagle-Columbia rendezvous after ascent on July 21
RRRDOT		TC	INITREAD -1
		OCT	12

; Rendezvous Radar range - single sample per reading
; Measures distance between LM and CSM during rendezvous operations
; Channel selection code OCT 11 activates range measurement
; First RR acquisition typically occurs at ~100 nautical miles range
RRRANGE		TC	INITREAD -1
		OCT	11

# LRVEL IS THE ENTRY TO THE LR VELOCITY READ ROUTINE WHEN 5 SAMPLES ARE
# WANTED.  ENTER WITH C(A)= 0,2,4 FOR LRVELZ,LRVELY,LRVELX RESP.

; LRVEL - Enhanced LR velocity read with 5-sample averaging
; Entry point when multiple samples are desired for noise reduction
; Input: A register contains index (0=LRVELZ, 2=LRVELY, 4=LRVELX)
; Five samples are collected and averaged to improve measurement accuracy
; during critical phases when velocity precision is essential
LRVEL		TS	TIMEHOLD	# STORE VBEAM INDEX HERE MOMENTARILY
		CAF	FIVE		# SPECIFY FIVE SAMPLES
		INDEX	TIMEHOLD
		TCF	LRVELZ

# Page 554
; -1 ENTRY: Single-sample mode entry point
; When only one radar sample is needed (altitude, range, range-rate),
; calling routines enter here to specify single-sample collection mode
 -1		CAF	ONE		# ENTRY TO TAKE ONLY 1 SAMPLE

; INITREAD - Common radar read initialization routine
; Sets up sampling parameters, computes timing for midpoint of sampling
; interval, and prepares for radar interrupt-driven data collection.
; Disables interrupts during setup to ensure atomic configuration.
;
; This routine is the gateway to all radar data acquisition. During
; rendezvous operations, it coordinates timing between radar hardware
; interrupts and AGC processing cycles to ensure accurate measurements.
INITREAD	INHINT

		TS	TIMEHOLD	# GET DT OF MIDPOINT OF NOMINAL SAMPLING
		EXTEND			# INTERVAL (ASSUMES NO BAD SAMPLES WILL BE
		MP	BIT3		# ENCOUNTERED).
		DXCH	TIMEHOLD

; Compute number of samples and attempt limit
; NSAMP holds the target number of good samples to collect
; SAMPLIM sets maximum attempts (currently N+1 tries for N samples)
; Note: Commented-out DOUBLE instruction would allow 2N tries for N samples
		CCS	A
		TS	NSAMP
		AD	ONE
# INSERT FOLLOWING INSTRUCTION TO GET 2N TRIES FOR N SAMPLES.
#		DOUBLE
		TS	SAMPLIM

; Read current radar data-good status bits
; These bits indicate whether radar hardware has valid measurement data
; available for reading. Different bits correspond to different radar types.
		CAF	DGBITS		# READ CURRENT VALUE OF DATA GOOD BITS.
		EXTEND
		RAND	CHAN33
		TS	OLDATAGD

; Configure radar channel selection
; First, clear all existing radar select bits in channel 13
; Then set only the bits specified by the calling routine (indexed from Q)
; This ensures clean channel selection without interference from previous state
		CS	ALLREAD
		EXTEND
		WAND	CHAN13		# REMOVE ALL RADAR BITS

		INDEX	Q
		CAF	0
		EXTEND
		WOR	CHAN13		# SET NEW RADAR BITS

; Compute timestamp for nominal sampling midpoint
; TIME2 contains current mission elapsed time
; TIMEHOLD will store the expected time at middle of sampling interval
; This timestamp is used to associate radar data with navigation state
		EXTEND
		DCA	TIME2
		DAS	TIMEHOLD	# TIME OF NOMINAL MIDPOINT

; Initialize sample accumulator to zero
; SAMPLSUM will accumulate radar measurements for averaging
; During multi-sample reads (like LR velocity with 5 samples),
; this double-precision accumulator prevents overflow
		CAF	ZERO
		TS	L
		DXCH	SAMPLSUM
		TCF	ROADBACK

; DGBITS - Data good bit mask (OCT 230 = bits 7, 6, 5)
; Used to check radar hardware status flags indicating valid measurements
DGBITS		OCT	230

# Page 555
# RADAR RUPT READER
#
# THIS ROUTINE STARTS FROM A RADARUPT.  IT READS THE DATA & LOTS MORE.

; ============================================================================
; TRANSITION: From Radar Initialization to Radar Interrupt Processing
;
; Having set up the radar measurement request, the code now enters the
; interrupt-driven data collection phase. When the radar hardware completes
; a measurement and signals RADARUPT, this handler executes to read the data,
; validate its quality, and accumulate it for navigation processing.
;
; During Apollo 11's rendezvous, RADAREAD fired repeatedly as the Rendezvous
; Radar tracked Columbia's position. Each interrupt brought fresh range or
; range-rate data that refined the relative state vector between Eagle and
; Columbia, enabling precise maneuver computation for the three rendezvous
; burns (CSI, CDH, and TPI) that brought the LM to within feet of the CSM.
; ============================================================================

		SETLOC	RADARUPT
		BANK

		COUNT*	$$/RRUPT

; ============================================================================
; RADAREAD - Radar Data Interrupt Handler
;
; This routine executes at interrupt level when radar hardware signals that
; new measurement data is available (RADARUPT interrupt). It performs:
; - Bank state preservation for safe interrupt return
; - Radar data reading from I/O channels
; - Data quality validation (data good bits, scale status, position checks)
; - Sample accumulation and averaging
; - Downlink telemetry preparation
; - Fault detection and alarm generation
;
; RADAREAD handles both Landing Radar (LR) and Rendezvous Radar (RR):
; - LR provides altitude and 3-axis velocity during descent
; - RR provides range and range-rate to CSM during rendezvous
;
; Critical for rendezvous navigation: Each RR measurement updates the relative
; position and velocity between LM and CSM, essential for computing the
; precise burns needed to bring the two spacecraft together after ascent.
; ============================================================================

RADAREAD	EXTEND			# MUST SAVE SBANK BECAUSE OF RUPT EXITS
		ROR	SUPERBNK	# VIA TASKOVER (BADEND OR GOODEND).
		TS	BANKRUPT
		EXTEND
		QXCH	QRUPT

; Read radar select bits from Channel 13 to determine which radar and
; measurement type triggered this interrupt (LR altitude, LR velocity,
; RR range, or RR range-rate). DNINDEX will be used for downlink telemetry.
		CAF	SEVEN
		EXTEND
		RAND	CHAN13
		TS	DNINDEX
		EXTEND			# IF RADAR SELECT BITS ZERO, DO NOT STORE
		BZF	TRYCOUNT	# DATA FOR DOWNLIST (ERASABLE PROBLEMS)
		CA	RNRAD
		INDEX	DNINDEX
		TS	DNRRANGE -1
; TRYCOUNT - Sample limit checking
; Verifies we haven't exceeded the allowed number of radar read attempts.
; SAMPLIM counts down from initial value; when it reaches zero, we've tried
; enough times and must decide whether to accept degraded data or abort.
TRYCOUNT	CCS	SAMPLIM
		TCF	PLENTY
		TCF	NOMORE
		TC	ALARM
		OCT	520
		TC	RESUME

; NOMORE - Sample limit exhausted
; We've run out of read attempts. Check if this situation should generate
; an alarm or be tolerated based on current program mode.
; - If LRBYPASS flag is set (R12 running), suppress alarm 521
; - If R04 (rendezvous radar self-test) is running, suppress alarm 521
; - Otherwise, P20 needs the alarm to alert crew of radar data failure
NOMORE		CA	FLGWRD11	# IS LRBYPASS SET?
		MASK	LRBYBIT
		EXTEND
		BZF	BADRAD		# NO.  R12 IS ON -- BYPASS 521 ALARM.

		CS	FLAGWRD3	# CHECK R04FLAG.
		MASK	R04FLBIT	# IF 1, R04 IS RUNNING.  DO NOT ALARM
		EXTEND
		BZF	BADRAD

		TC	ALARM		# P20 WANTS THE ALARM.
		OCT	521

; BADRAD - Radar data collection failed
; Set sample limit to -1 (indicating failure) and initiate bad-end processing.
; This path is taken when radar data quality is unacceptable or hardware
; has failed. The navigation filter will not receive an update this cycle.
BADRAD		CS	ONE
		TS	SAMPLIM
		TC	RDBADEND -2

; PLENTY - Sample limit still has attempts remaining
; Store the decremented count and continue with radar data processing
PLENTY		TS	SAMPLIM
		CAF	BIT3
		EXTEND
		RAND	CHAN13		# TO FIND OUT WHICH RADAR
		EXTEND
# Page 556
		BZF	RENDRAD

		TC	R77CHECK	# R77 QUITS HERE.

; LRPOSCHK - Landing Radar position verification
; Validates that the Landing Radar antenna is in the expected physical position
; (velocity mode vs. altitude mode) before accepting data. The LR antenna
; physically repositions between altitude and velocity measurements.
; If antenna position doesn't match commanded mode, issue alarm 522.
LRPOSCHK	CA	RADMODES	# SEE IF LR IN DESIRED POSITION
		EXTEND
		RXOR	CHAN33
		MASK	BIT6
		EXTEND
		BZF	VELCHK

		TC	ALARM
		OCT	522
		TC	BADRAD

; VELCHK - Landing Radar velocity measurement check
; Determines if this is an LR altitude read or LR velocity read by examining
; Channel 13 radar select bits. LR altitude uses a single beam aimed downward.
; LR velocity uses three beams (X, Y, Z components) to measure horizontal and
; vertical velocity components by Doppler shift.
; During Apollo 11 descent, LR velocity data was critical for ensuring proper
; descent rate management as the LM approached touchdown.
VELCHK		CAF	BIN3		# = 00003 OCT
		EXTEND
		RXOR	CHAN13		# RESET ACTIVITY BIT
		MASK	BIN3
		EXTEND
		BZF	LRHEIGHT	# TAKE A LR RANGE READING

		CAF	POSMAX
		MASK	RNRAD
		AD	LVELBIAS
		TS	L
		CAE	RNRAD
		DOUBLE
		MASK	BIT1
		DXCH	ITEMP3

; Data good check via DGCHECK subroutine
; Validates that radar hardware has asserted "data good" discrete signal.
; BIT8 in A register specifies which data good bit to check in Channel 33.
; Note: Data good isn't checked until AFTER reading data, allowing some
; radar self-tests to function even if radar doesn't assert data good.
		CAF	BIT8		# DATA GOOD ISN'T CHECKED UNTIL AFTER READ-
		TC	DGCHECK		# ING DATA SO SOME RADAR TESTS WILL WORK
					# INDEPENDENT OF DATA GOOD.

; Sample count check
; NSAMP indicates how many more samples are needed to complete averaging.
; If NSAMP > 0, continue sampling (NOEND path)
; If NSAMP = 0, we have collected enough good samples (GOODRAD path)
		CCS	NSAMP
		TC	NOEND

; GOODRAD - Successful radar data collection complete
; All required samples have been gathered with good quality.
; - Set SAMPLIM to -1 (indicating success)
; - Clear data fail flags in RADMODES for this radar type
; - Update radar indicator lights (may turn off fault lights)
; - Proceed to good-end processing which will incorporate data into nav state
GOODRAD		CS	ONE
		TS	SAMPLIM
		CS	ITEMP1		# WHEN ENOUGH GOOD DATA HAS BEEN GATHERED,
		MASK	RADMODES	# RESET DATA FAIL FLAGS FOR SETTRKF.
		TS	RADMODES
		TC	RADLITES	# LAMPS MAY GO OFF IF DATA JUST GOOD.
		TC	RGOODEND -2

; NOEND - More samples needed, continue accumulation
; Store decremented sample count and check if more read attempts available
NOEND		TS	NSAMP
RESAMPLE	CCS	SAMPLIM		# SEE IF ANY MORE TRIES SHOULD BE MADE.
		TCF	+2
		TCF	DATAFAIL	# N SAMPLES NOT AVAILABLE.
		CAF	BIT4		# RESET ACTIVITY BIT.
		EXTEND
# Page 557
		WOR	CHAN13		# RESET ACTIVITY BIT
		TC	RESUME

; ============================================================================
; LRHEIGHT - Landing Radar altitude measurement processing
; ============================================================================

; This path handles LR range (altitude) measurements during descent.
; The Landing Radar altitude beam points straight down and measures distance
; to lunar surface. During Apollo 11 descent, LR altitude data was displayed
; on the crew's altitude/velocity display and used by guidance to manage
; the descent trajectory. LR provided critical altitude information when
; below 40,000 feet where orbital navigation became less accurate.

LRHEIGHT	CAF	BIT5
		TS	ITEMP1		# (POSITION OF DATA GOOD BIT IN CHAN 33)

		CAF	BIT9
		TC	SCALECHK -1

; ============================================================================
; RENDRAD - Rendezvous Radar data processing
; ============================================================================

; This path handles RR range and range-rate measurements during rendezvous.
; Before accepting radar data, verify antenna and CDU integrity:
; - REPOSBIT: Antenna reposition limit switch (ensures antenna hasn't jammed)
; - RCDUFBIT: RR CDU failure indicator (gimbal angle readout validity)
;
; During Apollo 11 rendezvous on July 21, the RR tracked Columbia at ranges
; from ~100 nautical miles down to a few hundred feet. These measurements
; enabled precise computation of the three rendezvous burns (CSI, CDH, TPI)
; that brought Eagle to within feet of Columbia for docking.

RENDRAD		CAF	REPOSBIT	# MAKE SURE ANTENNA HAS NOT GONE OUT OF
		MASK	RADMODES	# LIMITS.
		CCS	A
		TCF	BADRAD

		CS	RADMODES	# BE SURE RR CDU HASN'T FAILED.
		MASK	RCDUFBIT
		CCS	A
		TCF	BADRAD

		CAF	BIT4		# SEE IF DATA HAS BEEN GOOD.
		TS	ITEMP1		# (POSITION OF DATA GOOD BIT IN CHAN 33)

; Determine if this is RR range or RR range-rate measurement
; RR RDOT (range-rate, Doppler velocity) doesn't require scale checking
; because it uses a different measurement principle than range.
		CAF	BIT1		# SEE IF RR RDOT.
		EXTEND
		RAND	CHAN13
		TS	Q		# FOR LATER TESTING.
		CCS	A
		TCF	+2
		TCF	RADIN		# NO SCALE CHECK FOR RR RDOT.
		CAF	BIT3
		TS	L

; ============================================================================
; SCALECHK - Radar scale factor change detection
; ============================================================================

; Radar range measurements use automatic scale switching as target distance
; changes. LR and RR both have multiple range scales (e.g., 0-2500 ft,
; 2500-25000 ft, etc.). When scale switches, accumulated samples must be
; discarded because they're at different scale factors and can't be averaged.
;
; This routine compares current scale status (Channel 33) with previous
; scale status (RADMODES) to detect mid-read scale changes. If detected,
; flag the change and restart sampling at the new scale.

SCALECHK	EXTEND
		RAND	CHAN33		# SCALE STATUS NOW
		XCH	L
		MASK	RADMODES	# SCALE STATUS BEFORE
		EXTEND
		RXOR	LCHAN		# SEE IF THEY DIFFER
		CCS	A
		TC	SCALCHNG	# THEY DIFFER.

; ============================================================================
; RADIN - Radar data input and bias correction
;
; This routine processes the raw radar measurement after scale checking:
; - Extracts measurement data from RNRAD (Channel 32 radar data register)
; - Applies bias correction for RR range-rate (Doppler measurements)
; - Prepares double-precision value for sample accumulation
;
; RR RDOT (range-rate) measurements contain a systematic bias from hardware
; electronics and antenna motion. RDOTBIAS is subtracted to correct this.
; During rendezvous, accurate range-rate is critical for computing relative
; velocity between LM and CSM - errors would compound during burn targeting.
; ============================================================================

RADIN		CAF	POSMAX
		MASK	RNRAD		# Extract measurement data (mask sign bit)
		TS	ITEMP4		# Save for later use

		CAE	RNRAD
		DOUBLE			# Convert to double-precision format
		MASK	BIT1
		TS	ITEMP3		# Lower word of DP measurement
# Page 558
		CCS	Q		# SEE IF RR RDOT.
		TCF	SCALADJ		# NO, BUT SCALE CHANGING MAY BE NEEDED.

; For RR range-rate (Doppler velocity), subtract hardware bias before
; accumulating samples. This bias correction is essential for accurate
; relative velocity determination during rendezvous radar tracking.
		EXTEND			# IF RR RANGE RATE, THROW OUT BIAS
		DCS	RDOTBIAS	# Load negative bias (2's complement)
DASAMPL		DAS	ITEMP3		# Double-precision add to sample
DGCHECK2	CA	ITEMP1		# SEE THAT DATA HAS BEEN GOOD BEFORE AND
		TC	DGCHECK +1	# AFTER TAKING SAMPLE.
		TC	GOODRAD		# If good, continue processing

; ============================================================================
; SCALCHNG - Handle radar scale factor change
;
; When radar scale switches mid-sampling (e.g., LM moving from 10000 ft to
; 2500 ft during descent, or range to CSM changing during rendezvous),
; accumulated samples at the old scale can't be averaged with new-scale data.
; This routine:
; - Updates RADMODES with new scale status
; - Preserves current data good bits for next comparison
; - Sets RNGSCFLG to notify navigation routines of scale change
; - Abandons current accumulation (goto BADRAD to restart sampling)
;
; During Apollo 11 rendezvous, RR scale switched multiple times as Eagle
; approached Columbia from ~100 nautical miles down to a few hundred feet.
; Each scale change triggered fresh sample accumulation at the new range.
; ============================================================================

SCALCHNG	LXCH	RADMODES	# Save old RADMODES in L
		AD	BIT1		# Toggle scale change indicator
		EXTEND
		RXOR	LCHAN		# Exclusive-OR with current scale status
		TS	RADMODES	# Update RADMODES with new scale
		CAF	DGBITS		# UPDATE LAST VALUE OF DATA GOOD BITS.
		EXTEND
		RAND	CHAN33		# Read current data good status
		TS	OLDATAGD	# Save for next interrupt comparison
		TC	UPFLAG		# SET RNGSCFLG
		ADRES	RNGSCFLG	# FOR LRS24.1
		TCF	BADRAD		# Discard samples, restart at new scale

# R77 MUST IGNORE DATA FAILS SO AS NOT TO DISTURB THE ASTRONAUT.

; ============================================================================
; R77CHECK - Special handling for R77 (LR test mode)
;
; R77 is a Landing Radar self-test and checkout routine that astronauts run
; during pre-descent system verification. Unlike normal radar operations, R77
; must NOT generate alarms or tracker fail lights when data quality is poor,
; because the test deliberately exercises the radar through its full range
; including marginal conditions.
;
; This routine:
; - Checks if R77 is currently active (R77FLBIT flag)
; - If R77 active: Updates LR data good status in RADMODES without alarming
; - If R77 not active: Returns to normal data quality checking
;
; During Apollo 11's descent preparation, astronauts ran R77 to verify LR
; functionality before committing to powered descent. The test confirmed that
; altitude and velocity sensors were operating properly before PDI at 102:33 MET.
; ============================================================================

R77CHECK	CS	FLAGWRD5	# Check if R77 test mode active
		MASK	R77FLBIT
		CCS	A
		TC	Q		# NOT R77 - return to normal processing
		CS	BITS5,8		# UPDATE LR DATA GOOD BITS IN RADMODES
		MASK	RADMODES	# Clear old data good bits
		TS	L
		CA	BITS5,8		# Mask for LR data good indicators
		EXTEND
		RAND	CHAN33		# Read current LR data status
		AD	L		# Combine with cleared RADMODES
		TS	RADMODES	# Update without triggering alarms
		TC	RGOODEND -2	# Exit as good data (suppress fail lamp)
BITS5,8		OCT	220		# Bits 5 & 8: LR data good indicators

# Page 559
# THE FOLLOWING ROUTINE INCORPORATES RR RANGE AND LR ALT SCALE INFORMATION AND LEAVES DATA AT LO SCALE.

; ============================================================================
; SCALADJ - Radar scale adjustment to normalize measurements
;
; Radar hardware operates in HIGH SCALE or LOW SCALE modes depending on range:
; - LR altitude: HIGH > 2500 ft, LOW < 2500 ft
; - RR range: HIGH > 25 nmi, LOW < 25 nmi
;
; This routine normalizes all measurements to LOW SCALE units for consistent
; sample accumulation and averaging. HIGH SCALE data is multiplied by scale
; factor (8 for RR, 4 for LR altitude) to convert to LOW SCALE equivalent.
;
; Without this normalization, samples taken at different scales couldn't be
; averaged together - a critical issue during descent when altitude drops
; through scale transition thresholds, or during rendezvous as range to CSM
; varies from 100+ nautical miles down to feet.
; ============================================================================

SCALADJ		CCS	L		# L HAS SCALE INBIT FOR THIS RADAR.
		TCF	+2		# ON HIGH SCALE - needs adjustment
		TCF	DGCHECK2	# On low scale - no adjustment needed

; Determine radar type (RR vs LR) from DNINDEX bit 3
		CA	DNINDEX
		MASK 	BIT3		# Bit 3 set = Landing Radar
		CCS	A
		TCF	LRSCK		# Landing Radar scale check

; Rendezvous Radar high scale to low scale conversion: multiply by 8
		DXCH	ITEMP3		# Load DP measurement
		DDOUBL			# x2
		DDOUBL			# x4
		DDOUBL			# x8 (RR scale factor)
		DXCH	ITEMP3		# Store normalized measurement

		TCF	DGCHECK2	# Continue to data good check

; ============================================================================
; LRSCK - Landing Radar Scale Check and adjustment
;
; For LR altitude, this routine:
; 1. Checks if altitude is near scale transition (2481.7 feet = HISCALIM)
; 2. Sets or clears SCABBIT flag to control scale mode selection
; 3. Converts HIGH SCALE measurement to LOW SCALE (multiply by 4)
;
; SCABBIT (scale-abort bit) prevents nuisance scale switching when altitude
; hovers near the 2500 ft threshold. During Apollo 11's descent, LR switched
; from HIGH to LOW scale as Eagle descended through 2500 feet altitude during
; the approach phase, approximately 2 minutes before landing.
; ============================================================================

LRSCK		CCS	ITEMP3		# Check sign of measurement
		TCF	+11		# Positive - continue
		CS	ITEMP4		# Negative case
		AD	HISCALIM	# Compare to scale transition limit
		EXTEND
		BZMF	+5		# Branch if at or below limit

; Altitude above HISCALIM - set SCABBIT to maintain HIGH SCALE
		CS	FLGWRD11
		MASK	SCABBIT
		ADS	FLGWRD11	# Set SCABBIT
		TCF	+4

; Altitude below HISCALIM - clear SCABBIT to enable LOW SCALE
		CS	SCABBIT
		MASK	FLGWRD11
		TS	FLGWRD11	# Clear SCABBIT

; Convert LR HIGH SCALE to LOW SCALE: multiply by 4
		EXTEND
		DCA	ITEMP3		# Load DP altitude measurement
		DDOUBL			# x2
		DDOUBL			# x4 (LR altitude scale factor)
		TCF	DASAMPL		# Continue to sample accumulation

HISCALIM	DEC	460		# 2481.7 FT (scale transition threshold)
# Page 560

; ============================================================================
; DGCHECK - Data Good verification before and after sampling
;
; Critical quality control routine that validates radar measurement integrity
; by checking that the "data good" hardware indicator remained continuously
; active during the entire sample acquisition window. This guards against:
; - Transient radar hardware glitches
; - Antenna pointing errors
; - Signal loss during CSM or LM maneuvering
; - Electromagnetic interference
;
; The routine compares:
; - OLDATAGD: Data good status BEFORE sample acquisition
; - Current Channel 33: Data good status AFTER sample acquisition
;
; If data good bit toggled off at any point during sampling, the measurement
; is discarded and RESAMPLE is attempted (up to maximum retry count).
;
; During Apollo 11 rendezvous, this routine prevented corrupted radar data
; from contaminating the navigation state vector. Eagle's radar successfully
; locked onto Columbia at approximately 100 nautical miles range, and this
; data quality gate ensured only valid measurements updated the relative
; position and velocity estimates used for rendezvous burn targeting.
; ============================================================================

DGCHECK		TS	ITEMP1		# UPDATE DATA GOOD BIT IN OLDATAGD AND
		EXTEND			# MAKE SURE IT WAS ON BEFORE AND AFTER THE
		RAND	CHAN33		# SAMPLE WAS TAKEN BEFORE RETURNING.  IF
		TS	L		# NOT, GOES TO RESAMPLE TO TRY AGAIN.  IF
		CS	ITEMP1		# MAX NUMBER OF TRIES HAS BEEN REACHED,
		MASK	OLDATAGD	# THE BIT CORRESPONDING TO THE DATA GOOD
		AD	L		# WHICH FAILED TO APPEAR IS IN ITEMP1 AND
		XCH	OLDATAGD	# CAN BE USED TO SET RADMODES WHICH VIA
		MASK	ITEMP1		# SETTRKF SETS THE TRACKER FAIL LAMP.
		AD	L
		CCS	A		# SHOULD BOTH BE ZERO (data good stable)
		TC	RESAMPLE	# Data quality failure - try again
		DXCH	ITEMP3		# IF DATA GOOD BEFORE AND AFTER, ADD TO
		DAS	SAMPLSUM	# ACCUMULATION (double-precision add)
		TC	Q		# Return to caller - good sample acquired

; ============================================================================
; DATAFAIL - Handle exhausted retry attempts with persistent bad data
;
; When maximum resample attempts have been exhausted but radar data quality
; remains poor, this routine implements graceful degradation:
; 1. Sets appropriate RADMODES failure bit (triggers TRACKER FAIL lamp)
; 2. Uses the LAST acquired sample (even though data good uncertain)
; 3. Calls RADLITES to illuminate cockpit warning indicators
; 4. Proceeds to NOMORE to complete processing with degraded data
;
; This design philosophy accepts marginal radar data rather than completely
; failing navigation updates during critical rendezvous operations. During
; Apollo 11's rendezvous, if radar tracking became intermittent, this routine
; would alert the crew (TRACKER FAIL light) while still providing the best
; available measurement rather than leaving navigation with no update at all.
;
; The astronauts could then decide whether to:
; - Continue with degraded automatic navigation
; - Switch to optical backup navigation (sextant marks on CSM)
; - Execute manual rendezvous procedures from checklist
; ============================================================================

DATAFAIL	CS	ITEMP1		# IN THE ABOVE CASE, SET RADMODES BIT
		MASK	RADMODES	# SHOWING SOME RADAR DATA FAILED
		AD	ITEMP1		# Merge failure indicator
		TS	RADMODES	# Update RADMODES with failure status

		DXCH	ITEMP3		# IF WE HAVE BEEN UNABLE TO GATHER N
		DXCH	SAMPLSUM	# SAMPLES, USE LAST ONE ONLY (DP load)
		TC	RADLITES	# Illuminate TRACKER FAIL lamp
		TCF	NOMORE		# Continue processing with degraded data
# Page 561
# THIS ROUTINE CHANGES THE LR POSITION, AND CHECKS THAT IT GOT THERE.

		SETLOC	P20S1
		BANK

		COUNT*	$$/RSUB

; ============================================================================
; LRPOS2 - Landing Radar Physical Position Change Command
;
; Commands the Landing Radar antenna to move to Position 2 and verifies
; successful positioning through hardware status monitoring.
;
; The LM Landing Radar has multiple physical antenna positions to optimize
; coverage during different descent phases:
; - Position 1: For high-altitude initial acquisition
; - Position 2: For approach and landing phase (commanded here)
;
; This routine:
; 1. Sets RADMODES to indicate desired position 2
; 2. Checks if antenna already at position 2 (via CHAN33 BIT7 status)
; 3. If not there, sends hardware command via CHAN12 BIT13
; 4. Schedules WAITLIST task to verify position after 7-second delay
; 5. Polls antenna status once per second for up to 15 seconds
; 6. Waits 2 additional seconds after confirmation for mechanical settling
;
; During Apollo 11's descent, the LR antenna was commanded to position 2
; as the LM approached the lunar surface. Proper antenna positioning was
; critical for maintaining accurate altitude and velocity measurements
; during the final approach when Armstrong manually selected the landing
; site. Mechanical antenna movement takes several seconds; this routine
; implements the complete command-verify-settle sequence.
;
; Failure modes:
; - RDBADEND: If antenna doesn't reach position 2 within 15 seconds
; - RGOODEND: Successful positioning after mechanical settling complete
; ============================================================================

LRPOS2		INHINT			# Disable interrupts for atomic operation

		CS	RADMODES
		MASK	LRPOSBIT	# SHOW DESIRED LR POSITION IS 2
		ADS	RADMODES	# Set position 2 bit in RADMODES

		CAF	BIT7
		EXTEND
		RAND	CHAN33		# SEE IF ALREADY THERE (check status)
		EXTEND
		BZF	RADNOOP		# Already at position 2 - no action needed

		CAF	BIT13
		EXTEND
		WOR	CHAN12		# COMMAND TO POSITION 2 (hardware output)
		CAF	6SECS		# START SCANNING FOR INBIT AFTER 7 SECS
		TC	WAITLIST	# Schedule verification task
		EBANK=	LOSCOUNT
		2CADR	LRPOSCAN	# Task to check antenna movement

		TC	ROADBACK	# Return to caller via standard path

LRPOSNXT	TS	SAMPLIM		# Store remaining sample count
		TC	FIXDELAY	# SCAN ONCE PER SECOND 15 TIMES MAX AFTER
		DEC	100		# INITIAL DELAY OF 7 SECONDS (1 second)

		CAF	BIT7		# SEE IF LR POS2 IS ON
		EXTEND
		RAND	CHAN33		# Read antenna position status
		EXTEND
		BZF	LASTLRDT	# IF THERE, WAIT FINAL SECOND FOR BOUNCE

		CCS	SAMPLIM		# SEE IF MAX TIME UP (15 attempts)
		TCF	LRPOSNXT	# Continue polling

		CS	BIT13		# IF TIME UP, DISABLE COMMAND AND ALARM
		EXTEND
		WAND	CHAN12		# Remove hardware command
		TCF	RDBADEND	# Exit with bad status (timeout failure)

RADNOOP		CAF	ONE		# NO FURTHER ACTION REQUESTED
		TC	WAITLIST	# Schedule immediate good completion
		EBANK=	LOSCOUNT
		2CADR	RGOODEND	# Task for successful completion
# Page 562
		TC	ROADBACK	# Return to caller

LASTLRDT	CA	2SECS		# WAIT TWO SECONDS AFTER RECEIPT OF INBIT
		TC	VARDELAY	# TO WAIT FOR ANTENNA BOUNCE TO DIE OUT

		CS	BIT13		# REMOVE COMMAND (antenna positioned)
		EXTEND
		WAND	CHAN12		# Clear hardware command bit
		TCF	RGOODEND	# Exit with good status

LRPOSCAN	CAF	FOURTEEN	# SET UP FOR 15 SAMPLES (initial entry)
		TCF	LRPOSNXT	# Begin polling loop
6SECS		DEC	600		# 6 seconds = 600 centiseconds
# Page 563
# SEQUENCES TO TERMINATE RR OPERATIONS.

; ============================================================================
; ENDRADAR - Rendezvous Radar Operations Termination
;
; Provides clean termination of RR tracking operations with status checking.
; Before ending radar processing, verifies RR CDU (Coupling Data Unit) health
; via RCDUFBIT in RADMODES. The CDU provides gimbal angle readouts for radar
; antenna pointing; CDU failure would invalidate all radar measurements.
;
; Termination paths:
; - RGOODEND: Radar data acquisition completed successfully
; - RDBADEND: Radar operations ended with failure/error condition
;
; RUPTAGN flag cleared to indicate radar interrupt no longer pending.
;
; During Apollo 11's rendezvous on July 21, 1969, after Eagle docked with
; Columbia at 128:03 mission elapsed time, this routine terminated RR tracking.
; The successful RGOODEND path confirmed that all relative navigation data
; had been properly acquired, processed, and incorporated into the state
; vector throughout the 3-hour-40-minute rendezvous sequence from ascent
; through final docking.
; ============================================================================

ENDRADAR	CAF	RCDUFBIT	# PROLOG TO CHECK RR CDU FAIL BEFORE END
		MASK	RADMODES	# Test CDU failure bit
		CCS	A		# CDU status test
		TCF	RGOODEND	# CDU failed - but exit cleanly
		TCF	RDBADEND	# CDU healthy - but some other failure
 -2		CS	ZERO		# RGOODEND WHEN NOT UNDER WAITLIST CONTROL
		TS	RUPTAGN		# Clear radar interrupt pending flag

; ============================================================================
; RGOODEND - Successful radar operations completion
;
; Terminates radar processing with successful status. All radar measurements
; acquired, quality-checked, and incorporated into navigation state.
; Jumps to GOODEND for final cleanup and return to calling program.
; ============================================================================

RGOODEND	CAF	TWO		# Set up for bank jump
		TC	POSTJUMP	# Cross-bank jump to completion routine
		CADR	GOODEND		# Successful termination handler

; ============================================================================
; RDBADEND - Failed radar operations completion
;
; Terminates radar processing with error status. Radar data quality issues,
; hardware failures, or timeout conditions prevent successful measurement
; incorporation. Jumps to BADEND for error handling and crew notification.
;
; This path illuminates cockpit warning lights (TRACKER FAIL, NO ATT, etc.)
; to alert crew that radar navigation updates have failed and backup
; procedures may be required (optical navigation, manual targeting).
; ============================================================================

 -2		CS	ZERO		# RDBADEND WHEN NOT UNDER WIATLIST
		TS	RUPTAGN		# Clear radar interrupt pending flag
RDBADEND	CAF	TWO		# Set up for bank jump
		TC	POSTJUMP	# Cross-bank jump to completion routine
		CADR	BADEND		# Error termination handler

BIN3		EQUALS	THREE

# Page 564
# PROGRAM NAME:  LPS20.1 VECTOR EXTRAPOLATION AND LOS COMPUTATION
# MOD. NO. 2	BY J.D. COYNE	SDC	DATE 12-7-66
#
# FUNCTIONAL DESCRPIPTION:
#	1)	EXTRAPOLATE THE LEM AND CSM VECTORS IN ACCORDANCE WITH THE TIME REFERRED TO IN CALLER + 1.
#	2)	COMPUTES THE LOS VECTOR TO THE CSM, CONVERTS IT TO STABLE MEMBER COORDINATES AND STORES IT IN RRTARGET.
#	3)	COMPUTES THE MAGNITUDE OF TEH LOS VECTOR AND STORES IT IN MLOSV
#
# CALLING SEQUENCE:	CALL
#				LPS20.1
#
# SUBROUTINES CALLED:
#	LEMPREC, CSMPREC
#
# NORMAL EXIT:  RETURN TO CALLER + 2.
#
# ERROR EXITS:  NONE
#
# ALARMS:  NONE
#
# OUTPUT:
#	LOS VECTOR (HALF UNIT) IN SM COORDINATES STORED IN RRTARGET
#	MAGNITUDE OF TEH LOS VECTOR (METERS SCALED B-29) STORED IN MSLOV
#	RRNBSW CLEARED.
#
# INITIALIZED ERASABLE
#	TDEC1 MUST CONTAIN THE TIME FOR EXTRAPOLATION
#	SEE ORBITAL INTEGRATION ROUTINE
#
# DEBRIS:
#	MPAC DESTROYED BY THE ROUTINE

		BANK	23
		SETLOC	P20S
		BANK
# Page 565
		COUNT*	$$/LPS20

; ============================================================================
; TRANSITION: From radar read completion to navigation state update
;
; The rendezvous radar has successfully acquired the CSM and provided
; measurements of range, range-rate, and line-of-sight angles. The following
; section processes this radar data to update either the LM or CSM state
; vector through Kalman filtering algorithms.
;
; Programs LPS20.1, LPS20.2, LRS22.1, LRS22.2, LSR22.3, and LSR22.4 form
; the measurement incorporation suite that translates raw radar measurements
; into navigation state corrections. This process was critical during Apollo
; 11's rendezvous: after Eagle's ascent on July 21, 1969, these routines
; processed rendezvous radar data to guide the LM through the three-burn
; rendezvous sequence (CSI, CDH, TPI) that brought Armstrong and Aldrin back
; to Columbia for docking 3 hours 40 minutes after liftoff.
;
; COMMENT-ONLY READERS: After radar lock-on to Columbia, the computer must
; translate raw radar data (range, closing velocity, angles) into updates
; to the spacecraft's navigational knowledge. The following routines compute
; the difference between what the radar measures and what the computer
; predicted, then adjust the position and velocity estimates accordingly.
; This continuous refinement of navigation knowledge enabled precise
; rendezvous maneuvers in the demanding environment of lunar orbit.
;
; CODE-ALONG READERS: This section implements measurement incorporation using
; extended Kalman filter techniques. For each radar measurement type (range,
; range-rate, line-of-sight angles), the routines compute:
; 1) The measurement residual (Delta Q) = measured value - predicted value
; 2) The measurement sensitivity matrix (B-vector) = partial derivatives
;    of measurement with respect to state variables
; 3) The Kalman gain and state correction through LGCUPDTE
; The W-matrix (measurement covariance) is initialized and the state
; covariance is propagated through the update. Multiple measurement types
; can be processed sequentially to refine the navigation solution.
; ============================================================================

; LPS20.1 - STATE VECTOR EXTRAPOLATION FOR RADAR MEASUREMENT TIME
; 
; This routine extrapolates both LM and CSM state vectors to the time of
; the radar measurement (TDEC1), computes the relative position (line-of-
; sight) vector, and prepares the data structures for measurement processing.
;
; The routine handles two calling scenarios:
; 1) Called from P20/P22 rendezvous navigation: extrapolates both vehicles
; 2) Called from R21 designate on lunar surface: skips LM extrapolation
;
; OUTPUTS:
;   LMPOS, LMVEL - LM position (B-29 meters) and velocity (B-7 m/cs)
;   LOSVEL - Relative velocity vector CSM-LM in stable member coordinates
;   RRTARGET - Line-of-sight unit vector to CSM in stable member coords
;   MLOSV - Magnitude of LOS vector (for 400 NM range check)
;
; During Apollo 11 rendezvous, this routine ran continuously as the radar
; tracked Columbia, providing updated relative state vectors that enabled
; precise targeting for the rendezvous burns.

; Save return address and check calling context
LPS20.1		STQ	BOFF
			LS21X		; Store return address in Q register
			LOSCMFLG	# LOSCMFLG = 0 MEANS NOT CALLED BY R21
			LMINT		# SO CALL LEMCONIC TO GET LM STATE
		BON			# IF IN R21 AND ON LUNAR SURFACE
			SURFFLAG	# DON'T CALL LEMCONIC
			CSMINT		; Skip LM extrapolation if on surface

; Extrapolate LM state vector to radar measurement time using conic integration
LMINT		CALL
			LEMCONIC	# EXTRAPOLATE LEM
		VLOAD			; Load extrapolated position vector
			RATT		; From RATT (position at time T)
		STOVL	LMPOS		# SAVE LM POSITION B-29
			VATT		; Load extrapolated velocity vector
		STODL	LMVEL		# SAVE LM VELOCITY B-7
			TAT		; Load time of extrapolation

; Extrapolate CSM state vector to same time for relative state computation
CSMINT		STCALL	TDEC1		; Store time in TDEC1 for CSM integration
			CSMCONIC	# EXTRAPOLATE CSM
; Compute relative velocity vector and transform to stable member coordinates
		VLOAD	VSU		# COMPUTE RELATIVE VELOCITY V(CSM) - V(LM)
			VATT		; CSM velocity (B-7 m/cs)
			LMVEL		; Subtract LM velocity
		MXV	VSL1		; Transform to stable member frame
			REFSMMAT	; Reference to stable member matrix
			
; Kill DODES task to prevent conflict during erasable updates
		EXIT			; Exit interpretive mode
		TC	KILLTASK	# KILL THE TASK WHICH CALLS DODES SINCE
		CADR	DESLOOP +2	# STORING INTO ERASEABLES DODES USES
		TC	INTPRET		; Re-enter interpretive mode
		
; Store relative velocity and compute line-of-sight vector (LOS)
		STOVL	LOSVEL		; Save relative velocity
			RATT		; Load CSM position
		VSU	BOFF		; Compute position difference
			LMPOS		; CSM position - LM position
			RNDVZFLG	; Check rendezvous flag
			NOTSHIFT	; Branch if not in rendezvous mode
; Handle rendezvous mode: check for overflow and apply Danzig algorithm if needed
		BOVB			; Branch on overflow
			TCDANZIG	; Danzig rescue for vector overflow
		VSL			; Shift left to normalize
			9D		; By 9 bits
			
; Compute unit line-of-sight vector and check for range overflow (>400 NM)
NOTSHIFT	UNIT	BOVB		# IF OVERFLOW, RANGE MUST BE GREATER
			526ALARM	# THAN 400 N. M.
		MXV	VSL1		; Transform LOS unit vector to stable member
			REFSMMAT	# CONVERT TO STABLE MEMBER
		STODL	RRTARGET	; Store as radar target direction
			36D		# SAVE MAGNITUDE OF LOS VECTOR FOR
		STORE	MLOSV		# VELOCITY CORRECTION IN DESIGNATE
		CLRGO			; Clear narrow-beam search flag
			RRNBSW		; RR narrow beam search switch
			LS21X		; Return to caller

# Page 566
# PROGRAM NAME:  LPS20.2  400 NM RANGE CHECK
# MOD. NO. 2	BY J.D. COYNE	SDC	DATE 12-7-66
#
# FUNCTIONAL DESCRIPTION:
#	COMPARES THE MAGNITUDE OF THE LOS VECTOR TO 400 NM.
#
# CALLING SEQUENCE:	CALL
#				LPS20.2
#
# SUBROUTINES CALLED:  NONE
#
# NORMAL EXIT:  RETURN TO CALLER +1, MPAC EQ 0 (RANGE 400NM OR LESS.)
#
# ERROR EXITS:  RETURN TO CALLER +1, MPAC EQ 1 (RANGE GREATER THAN 400NM)
#
# ALARMS:  NONE
#
# OUTPUT:  NONE
#
# INITIALIZED ERASEABLE:
#	PDL 36D MUST CONTAIN THE MAGNITUDE OF THE VECTOR
#
# DEBRIS:
#	MPAC DESTROYED BY THIS ROUTINE

		SETLOC	P20S1
		BANK
		COUNT*	$$/LPS20

; LPS20.2 - 400 NAUTICAL MILE RANGE CHECK
;
; This routine verifies that the computed range to the CSM is within the
; rendezvous radar's maximum effective range of 400 NM. Ranges beyond this
; limit indicate either loss of track or computational errors.
;
; During Apollo 11 rendezvous on July 21, 1969, the radar initially acquired
; Columbia at approximately 100 NM range. This check ensured that computed
; ranges remained reasonable throughout the rendezvous sequence.
;
; INPUTS:
;   MLOSV - Magnitude of line-of-sight vector (B-29 meters)
;
; OUTPUTS:
;   MPAC = 0 if range <= 400 NM (normal)
;   MPAC = 1 if range > 400 NM (out of range)

LPS20.2		DLOAD	DSU		; Load LOS magnitude and subtract limit
			MLOSV		# MAGNITUDE OF LOS
			FHNM		# OVER 400NM (740800 meters)
		BPL			; Branch if positive (range too far)
			TOFAR		; Handle out-of-range condition
		SLOAD	RVQ		; Load zero (range acceptable)
			ZERO/SP		; Return 0 in MPAC
TOFAR		SLOAD	RVQ		; Load one (range excessive)
			ONE/SP		; Return 1 in MPAC
ONE/SP		DEC	1		; Constant 1
# Page 567
FHNM		2DEC	740800 B-20	# 400 NAUTICAL MILES IN METERS B-20

# Page 568
# PROGRAM NAME:  LRS22.1 (DATA READ SUBROUTINE 1)
# MOD. NO.: 1		BY: P. VOLANTE  SDC		DATE:  11-15-66
#
# FUNCTIONAL DESCRIPTION:
#	1)	READS RENDEZVOUS RADAR RANGE AND RANGE-RATE, TRUNNION AND SHAFT ANGLES, THREE CDU VALUES AND TIME.  CONVERTS THIS
#		DATA AND LEAVES IT FOR THE MEASUREMENT INCORPORATION ROUTINE (LSR22.3).  CHECKS FOR THE RR DATA GOOD DISCRETE, FOR
#		RR REPOSITION AND RR CDU FAIL
#	2)	COMPARES RADAR LOS WITH LOS COMPUTED FROM STATE VECTORS TO SEE IF THEY ARE WITHIN THREE DEGREES
#
# CALLING SEQUENCE:  BANKCALL FOR LRS22.1
#
# SUBROUTINES CALLED:
#	RRDOT		LPS20.1
#	RRRANGE		BANKCALL
#	RADSTALL	CDULOGIC
#	RRNB		SMNB
#
# NORMAL EXIT:  RETURN TO CALLER+1 WITH MPAC SET TO +0
#
# ERROR EXITS:  RETURN TO CALLER+1 WITH ERROR CODE STORED IN MPAC AS FOLLOWS:
#	00001 -- ERROR EXIT 1 -- RR DATA NO GOOD (NO RR DATA GOOD DISCRETE OR RR CDU FAIL OR RR REPOSITION)
#	00002 -- ERROR EXIT 2 -- RR LOS NOT WITHIN THREE DEGREES OF LOS COMPUTED FROM STATE VECTORS
#
# ALARMS:  521 -- COULD NOT READ RADAR DATA (RR DATA GOOD DISCRETE NOT PRESENT BEFORE AND AFTER READING THE RADAR)
#	(THIS ALARM IS ISSUED BY RADARREAD SUBROUTINE WHICH IS ENTERED FROM A RADARUPT)
#
# OUTPUT:  RRLOSVEC -- THE RR LINE-OF-SIGHT VECTOR (USED BY LRS22.2) -- A HALF-UNIT VECTOR
# 	RM -- THE RR RANGE READING (TO THE CSM) DP, IN METERS SCALED BY B-29 (USED BY LRS22.2 AND LRS22.3)
#
#	ALL OF THE FOLLOWING OUTPUTS ARE USED BY LRS22.3:
#		RDOTM -- THE RR RANGE-RATE READING, DP, IN METERS PER CENTISECOND, SCALED BY B-7
#		RRTRUN -- THE RR TRUNNION ANGLE, DP, IN REVOLUTIONS, SCALED B0
#		RRSHAFT -- RR SHAFT ANGLE, DP, IN REVOLUTIONS, SCALED B0
#		AIG,AMG,AOG -- THE CDU ANGLES, THREE SP WORDS
#		MKTIME -- THE TIME OF THE RR READING, DP, IN CENTISECONDS
#
# ERASABLE INITIALIZATION REQUIRED:
#	RNRAD, THE RADAR READ COUNTER FROM WHICH IS OBTAINED:
# Page 569
#		1) RR RANGE SCALED 9.38 FT. PER BIT ON THE LOW SCALE AND 75.04 FT. PER BIT ON THE HIGH SCALE
#		2) RR RANGE RATE, SCALED .6278 FT./SEC. PER BIT
#	THE CDU ANGLES FROM CDUX, CDUY, CDUZ, AND TIME1 AND TIME2
#
# DEBRIS:  LRS22.1X, A, L, Q, PUSHLIST

		BANK	32
		SETLOC	LRS22
		BANK
		COUNT*	$$/LRS22

; ============================================================================
; LRS22.1 - RENDEZVOUS RADAR DATA READ SUBROUTINE
; ============================================================================
;
; This critical subroutine reads the rendezvous radar measurements that enable
; the LM to track the CSM during rendezvous operations. The radar provides
; both range (distance) and range-rate (closing velocity) measurements, along
; with antenna pointing angles (trunnion and shaft).
;
; APOLLO 11 RENDEZVOUS CONTEXT (July 21, 1969):
; After Eagle's ascent from the lunar surface at 17:54 UTC, the rendezvous
; radar acquired Columbia at approximately 100 nautical miles range. This
; routine was called repeatedly during the 3 hour 40 minute rendezvous chase,
; providing Armstrong and Aldrin with tracking data that guided three critical
; maneuvers:
;   - Concentric Sequence Initiation (CSI) at 128 NM
;   - Constant Delta-Height (CDH) at 88 NM  
;   - Terminal Phase Initiation (TPI) at 50 NM
;
; The routine performs two validation checks:
;   1) Verifies radar data quality (data good discrete, no reposition, CDU OK)
;   2) Compares radar LOS with computed LOS (must agree within 3 degrees)
;
; If validation passes, radar measurements are passed to LSR22.3 (measurement
; incorporation) which updates the navigation state vector using Kalman filter
; algorithms.
;
; INPUTS:
;   Hardware registers: RNRAD (radar range/range-rate counters)
;                       CDUX, CDUY, CDUZ (IMU gimbal angles)
;                       TIME1, TIME2 (mission elapsed time)
;                       CHAN33 bit 3 (radar range scale: high/low)
;
; OUTPUTS:
;   RM - Radar range to CSM (B-29 meters)
;   RDOTM - Radar range-rate (B-7 meters/centisecond)
;   RRTRUN - Radar trunnion angle (B0 revolutions)
;   RRSHAFT - Radar shaft angle (B0 revolutions)
;   AIG, AMG, AOG - IMU CDU angles at measurement time
;   MKTIME - Time of radar measurement (centiseconds)
;   RRLOSVEC - Radar line-of-sight unit vector (half-unit)
;
; ERROR CODES (returned in MPAC):
;   00000 - Success, radar data valid
;   00001 - Radar data not good (discrete absent, CDU fail, or reposition)
;   00002 - Radar LOS disagrees with computed LOS by more than 3 degrees
;
; ============================================================================

; Initialize LRS22.1: save return address and read range scale setting
; During Apollo 11 rendezvous, this routine processed radar data from Columbia
; at ranges varying from ~100 miles down to docking distance.
LRS22.1		TC	MAKECADR	; Create CADR for return address
		TS	LRS22.1X	; Save for exit jump
		TC	DOWNFLAG	; Clear range scale change flag
		ADRES	RNGSCFLG	; (prevents false scale change detection)
		INHINT			; Inhibit interrupts during hardware read
		CAF	BIT3		; Prepare to read bit 3
		EXTEND			# GET RR RANGE SCALE
		RAND	CHAN33		# FROM CHANNEL 33 BIT 3 (scale indicator)
		TS	L		; Save scale bit in L register
		CS	RRRSBIT		; Complement of range scale bit mask
		MASK	RADMODES	; Clear old scale bit in RADMODES
		AD	L		; Add new scale bit reading
		TS	RADMODES	; Update radar mode word with current scale
		RELINT			; Re-enable interrupts
; Read range-rate data from rendezvous radar
; Range-rate (closing velocity) measured by Doppler shift in radar signal
READRDOT	TC	BANKCALL	; Call range-rate read routine
		CADR	RRRDOT		# READ RANGE-RATE (ONE SAMPLE)
		TC	BANKCALL	; Wait for hardware read completion
		CADR	RADSTALL	# WAIT FOR DATA READ COMPLETION
		TCF	EREXIT1		# COULD NOT READ RADAR-ERROR EXIT 1

; Capture time-synchronized snapshot of all relevant data
; Critical: CDU angles, radar data, and time must be from same instant
		INHINT			# NO INTERRUPTS WHILE READING TIME AND CDU
		DXCH	TIMEHOLD	# SET MARK TIME EQUAL TO THE MID-POINT
		DXCH	MPAC +5		# TEMP BUFFER FOR DOWNLINK (mission control)
		DXCH	SAMPLSUM	# SAVE RANGE-RATE READING (raw radar data)
		DXCH	RDOTMSAV	; Preserve for later conversion
		EXTEND
		DCA	CDUY		# SAVE ICDU ANGLES (IMU gimbal positions)
		DXCH	MPAC +3		# TEMP BUFFER FOR DOWNLINK
		CA	CDUX		; X-axis CDU angle
		TS	MPAC +2		# TEMP BUFFER FOR DOWNLINK
		EXTEND
		DCA	TIME2		# SAVE TIME (AGC mission elapsed time)
		DXCH	MPAC		# SAVE TIME OF CDUY READINGS IN MPAC
		EXTEND
		DCA	CDUT		# SAVE TRUNNION AND SHAFT ANGLES FOR RRNB
		DXCH	TANG		; Radar antenna pointing angles
# Page 570
; Now read range data from rendezvous radar
; Range measured by radar pulse round-trip time
		RELINT			; Re-enable interrupts between reads
		TC	BANKCALL	; Call range read routine
		CADR	RRRANGE		# READ RR RANGE (ONE SAMPLE)
		TC	BANKCALL	; Wait for hardware read completion
		CADR	RADSTALL	# WAIT FOR READ COMPLETE
		TC	CHEXERR		# CHECK FOR ERRORS DURING READ (branch if error)
; Copy synchronized data snapshot for telemetry downlink
; Mission control monitors radar performance and navigation state
		INHINT			# COPY CYCLE FOR MARK DATA ON DOWNLINK
		DXCH	DNRRANGE	# RANGE, RANGE RATE (RAW DATA)
		DXCH	RANGRDOT	; Raw radar measurements
		DXCH	MPAC +5		; Time data for downlink
		DXCH	MKTIME		# MARK TIME (synchronized with radar read)
		DXCH	MPAC +3		; CDU data for downlink
		DXCH	AIG		# CDUY, CDUZ (Y and Z gimbal angles)
		EXTEND
		DCA	TANG		# PRESERVE TANG (radar antenna angles)
		DXCH	TANGNB		# TRUNNION AND SHAFT ANGLES
		CA	MPAC +2		; X-axis CDU
		TS	AOG		# CDUX (complete CDU set now saved)
; Convert all raw radar and angle data to standard navigation units
; Uses interpretive language for efficient multi-precision arithmetic
		TC	INTPRET		; Enter interpreter for vector math
		STODL	20D		# SAVE TIME OF CDU READINGS IN 20D
			RDOTMSAV	# CONVERT RDOT UNITS AND SCALING
		SL	DMPR		# START WITH READING SCALED B-28, -.6278
			14D		# FT./SECOND PER BIT (hardware conversion)
			RDOTCONV	# END WITH METERS/CENTISECOND, B-7
		STORE	RDOTM		; Converted range-rate for navigation use
		SLOAD	RTB		; Load trunnion angle
			TANG		# GET TRUNNION ANGLE (radar elevation)
			CDULOGIC	# CONVERT TO DP ONES COMP. IN REVOLUTIOINS
		STORE	RRTRUN		# AND SAVE FOR TMI ROUTINE (LSR22.3)
		SLOAD	RTB		; Load shaft angle
			TANG +1		# DITTO FOR SHAFT ANGLE (radar azimuth)
			CDULOGIC	; Convert to double-precision format
		STODL	RRSHAFT		; Save shaft angle
			SAMPLSUM	; Get raw range reading
		DMP	SL2R		# CONVERT UNITS AND SCALING DP RANGE
			RANGCONV	# PER BIT, END WITH METERS, SCALED -29
		STCALL	RM		; Store range magnitude in meters
			RRNB		# COMPUTE RADAR LOS USING RRNB (radar line-of-sight)
		STODL	RRBORSIT	# AND SAVE (radar line-of-sight in body coords)
			20D		; Get mark time
		STCALL	TDEC1		# GET STATE VECTOR LOS AT TIME OF CDU READ
			LPS20.1		; Compute expected LOS from state vectors
		EXIT			; Return to native AGC code

; Store IMU gimbal angles for coordinate transformation
; These angles define the transformation from stable member to navigation base
		CA	AIG		# STORE IMU CDU ANGLES AT MARKTIME
		TS	CDUSPOT		# IN CDUSPOT FOR TRG*SMNB (Y gimbal)
		CA	AMG		; Middle gimbal angle
		TS	CDUSPOT +2	; Save for coordinate rotation
		CA	AOG		; Outer gimbal angle
		TS	CDUSPOT +4	; Complete gimbal angle set
		TC	INTPRET		; Re-enter interpreter for comparison
# Page 571
; ============================================================================
; RADAR DATA QUALITY CHECK
; Compare actual radar line-of-sight to predicted LOS from state vectors
; If difference exceeds 3 degrees, radar may have lost lock on CSM
; ============================================================================
		VLOAD	CALL		# LOAD VECTOR AND CALL TRANSFORMATION
			RRTARGET	; Expected target LOS (from state vectors)
			TRG*SMNB	# ROTATE LOS AT MARKTIME FROM SM TO NB.
		DOT			# DOT WITH RADAR LOS TO GET ANGLE
			RRBORSIT	; Actual radar LOS (from antenna angles)
		SL1	ACOS		# BETWEEN THEM (compute angle from dot product)
		STORE	DSPTEM1		# STORE FOR POSSIBLE DISPLAY (to crew)
		DSU	BMN		# IS IT LESS THAN 3 DEGREES
			THREEDEG	; 3-degree tolerance threshold
			NORMEXIT	# YES -- NORMAL EXIT (data good)

; Radar-navigation mismatch exceeds 3 degrees - tracking error
; Possible causes: radar lost lock, wrong target, bad state vectors
		EXIT			# ERROR EXIT 2
		CAF	BIT2		# SET ERROR CODE (bit 2 = angle error)
		TS	MPAC		; Store error code for calling routine
		TCF	OUT22.1		; Return with error indication

; Normal exit - radar data valid and consistent with navigation state
NORMEXIT	EXIT			# NORMAL EXIT -- SET MPAC EQUAL ZERO
		CAF	ZERO		; Error code 0 = success
		TS	MPAC		; Return success indicator
OUT22.1		CAE	LRS22.1X	# EXIT FROM LRS22.1 (return address)
		TC	BANKJUMP	; Cross-bank return to calling routine

; Range scale change error handler
; If radar changed scale during read, data is inconsistent - retry
CHEXERR		CAE	FLAGWRD5	; Check radar status flags
		MASK	RNGSCBIT	; Isolate range scale bit
		CCS	A		# CHECK IF RANGE SCALE CHANGED
		TCF	READRDOT	# YES -- TAKE ANOTHER READING (retry cycle)

; Radar read error - hardware failure or no lock
EREXIT1		CA	BIT1		# SET ERROR CODE (bit 1 = read error)
		TS	MPAC		; Store error code
		TC	OUT22.1		; Return with error indication

; Three-degree tolerance constant for LOS comparison
THREEDEG	2DEC	.008333333	# THREE DEGREES, SCALED REVS, B0 (3/360)

RRLOSVEC	EQUALS	RRTARGET

# Page 572
# PROGRAM NAME -- LRS22.2 (DATA READ SUBROUTINE 2)
# MOD. NO.: 1		BY: P. VOLANTE  SDC		DATE: 4-11-67
#
# FUNCTIONAL DESCRIPTION:
#	(Yes, I know point #1 is missing.  It is missing from the program listing -- RSB 2003)
#	2)  CHECKS IF THE RR LOS (I.E., THE RADAR BORESIGHT VECTOR) IS WITHIN 30 DEGREES OF THE LM +Z AXIS
#
# CALLING SEQUENCE:  BANKCALL FOR LRS22.2
#
# SUBROUTINES CALLED:  G+N, AUTO, SETMAXDB
#
# NORMAL EXIT:  RETURN TO CALLER WITH MPAC SET TO +0 (VIA SWRETURN)
#
# ERROR EXIT:  RETURN TO CALLER WITH MPAC SET TO 00001 -- RADAR LOS NOT WITHIN 30 DEGREES OF LM +Z AXIS.
#
# ALARMS:  NONE
#
# ERASABLE INITIALIZATION REQUIRED:
#	RRLOSVEC -- THE RR LINE-OF-SIGHT VECTOR -- A HALF UNIT VECTOR COMPUTED BY LRS22.1
#	RM -- RR RANGE, METERS B-29, FROM LRS22.1
#	BIT 14 CHANNEL 31 -- INDICATES AUTOPILOT IS IN AUTO MODE
#
# DEBRIS -- A,L,Q,MPAC -- PUSHLIST AND PUSHLOC ARE NOT CHANGED BY THIS ROUTINE

		SETLOC	P20S
		BANK
LRS22.2		TC	MAKECADR
		TS	LRS22.1X
		TC	INTPRET
; ============================================================================
; 30-DEGREE ANTENNA CLEARANCE CHECK
; Verifies radar antenna can physically point at target without gimbal limits
; Checks angle between radar LOS and spacecraft +Z axis (antenna boresight)
; ============================================================================

					# CHECK IF RR LOS IS WITHIN 30 DEG OF
30DEGCHK	DLOAD	ACOS		# THE SPACECRAFT +Z AXIT (antenna axis)
			RRBORSIT +4	# BY TAKING ARCCOS OF Z-COMP. OF THE RR
					# LOS VECTOR, A HALF UNIT VECTOR
					# IN NAV BASE AXES)
		DSU	BMN		; Subtract 30-degree limit
			30DEG		; Antenna gimbal limit constant
			OKEXIT		# NORMAL EXIT -- WITHIN 30 DEG. (antenna can track)
		EXIT			# ERROR EXIT -- NOT WITHIN 30 DEG.
		CAF	BIT1		# SETS ERROR CODE IN MPAC (bit 1 = angle error)
		TS	MPAC		; Return error indication
		TCF	OUT22.2		; Exit with error
OKEXIT		EXIT			# NORMAL EXIT -- SET MPAC = ZERO (success)

# Page 573
		CAF	ZERO		; Error code 0 = success
		TS	MPAC		; Return success indicator
OUT22.2		CAE	LRS22.1X	; Get return address
		TC	BANKJUMP	; Cross-bank return to calling routine

; Thirty-degree tolerance constant for antenna gimbal limits
30DEG		2DEC	.083333333	# THIRTY DEGREES, SCALED REVS, B0 (30/360)

# Page 574
# PROGRAM NAME -- LSR22.3				DATE -- 29 MAY 1967
# MOD. NO 3						LOG SECTION -- P20-P25
# MOD. BY -- DANFORTH					ASSEMBLY LEMP20S REV 10
#
# FUNCTIONAL DESCRIPTION:
#	THIS ROUTINE COMPUTES THE B-VECTORS ADN DELTA Q FOR EACH OF THE QUANTITIES MEASURED BY THE RENDEZVOUS
#	RADAR.  (RANGE, RANGE RATE, SHAFT AND TRUNNION ANGLES).  THE ROUTINE CALLS THE INCORP1 AND INCORP2 ROUTINES
#	WHICH COMPUTE THE DEVIATIONS AND CORRECT THE STATE VECTOR.
#
# CALLING SEQUENCE:
#	THIS ROUTINE IS PART OF P20 RENDEZVOUS NAVIGATION FOR THE LM COMPUTER ONLY.  THE ROUTINE IS ENTERED FROM
# 	R22 LEM ONLY AND RETURNS DIRECTLY TO R22LEM FOLLOWING SUCCESSFUL INCORPORATION OF MEASURED DATA.  IF THE
#	COMPUTED STATE VECTOR DEVATIONS EXCEED THE MAXIMUM PERMITTED.  THE ROUTINE RETURNS TO R22LEM TO DISPLAY
#	THE DEVIATIONS.  IF THE ASTRONAUT ACCEPTS THE DATA R22LEM RETURNS TO LSR22.3 TO INCORPORATE THE
#	DEVIATIONS INTO THE STATE VECTOR.  IF THE ASTRONAUT REJECTS THE DEVIATIONS, NO MORE MEASUREMENTS ARE
#	PROCESSED FOR THIS MARK, I.E.,  R22LEM GETS THE NEXT MARK.
#
# SUBROUTINES CALLED:
#	WLINIT		LGCUPDTE	INTEGRV		INCORP1		ARCTAN
#	GETULC		RADARANG	INCORP2		NBSM		INTSTALL
#
# OUTPUT:
#	CORRECTED LM OR CSM STATE VECTOR (PERMANENT)
#	NUMBER OF MARKS INCORPORATED IN MARKCTR
#	MAGNITUDE OF POSITION DEVIATION (FOR DISPLAY) IN R22DISP METERS B-29
#	MAGNITUDE OF VELOCITY DEVIATION (FOR DISPLAY) IN R22DISP +2 M/CSEC B-7
#	UPDATED W-MATRIX
#
# ERASABLE INITIALIZATION REQUIRED:
#	LM AND CSM STATE VECTORS
#	W-MATRIX
#	MARK TIME IN MKTIME
#	RADAR RANGE IN RM METERS B-29
#		RANGE RATE IN RDOTM METERS/CSES B-7
#		SHAFT ANGLE IN RRSHAFT REVS. B0
#		TRUNNION ANGLE IN RRTRUN REVS. B0
#	GIMBAL ANGLES	INNER IN AIG
#			MIDDLE IN AMG
#			OUTER IN ACG
#	REFSMMAT
#	RENDWFLG
#	NOANGFLG
#	VEHUPFLG
#
# DEBRIS:
#	PUSHLIST -- ALL
#	MX, MY, MZ (VECTORS)
# Page 575
#	ULC, RXZ, SINTHETA, LGRET, RDRET, BVECTOR, W.IND, X78T
#
# ============================================================================
# LSR22.3 - STATE VECTOR CORRECTION WITH RENDEZVOUS RADAR MEASUREMENTS
#
# MISSION CONTEXT:
#	After Eagle's ascent from lunar surface on July 21, 1969, this routine
#	processed continuous rendezvous radar tracking data to refine relative
#	state knowledge between Eagle and Columbia. The rendezvous radar locked
#	onto Columbia at approximately 100 nautical miles range. LSR22.3 
#	incorporated each radar measurement (range, range-rate, shaft angle,
#	trunnion angle) into state vector corrections using Kalman filtering.
#	Accurate relative navigation enabled precise computation of three
#	rendezvous maneuvers bringing Eagle to docking 3 hours 40 minutes
#	after ascent.
#
# PROCESSING FLOW:
#	1. Integration and W-matrix initialization (INTSTALL, INTGRCAL)
#	2. Sphere of influence check (Earth vs Moon gravity model)
#	3. Measurement variance computation (based on scaling and range)
#	4. B-vector computation (measurement sensitivity partial derivatives)
#	5. DELTAQ computation (state correction for each measurement)
#	6. W-matrix update with measurement information
#	7. State vector incorporation after preset mark count
#
# TECHNICAL NOTES:
#	- Processes 4 measurement types: range, range-rate, shaft, trunnion
#	- Uses covariance matrix (W-matrix) for optimal state estimation
#	- Scaling factors vary by sphere of influence (Earth B-29, Moon B-27)
#	- Gimbal angle vectors (MX, MY, MZ) transform radar frame to body frame
#	- Mark counter TRKMKCNT tracks measurements before incorporation
#	- Deviation display allows crew to reject outlier measurements
# ============================================================================

		BANK	13
		SETLOC	P20S3
		BANK

		EBANK=	LOSCOUNT
		COUNT*	$$/LSR22
# Begin state vector correction routine. Check lunar surface flag first.
LSR22.3		CALL
			GRP2PC
		BON	SET
			SURFFLAG	# ARE WE ON LUNAR SURFACE
			LSR22.4		# YES - Branch to lunar surface navigation
			DMENFLG		# Set dimension enable flag
# Check which vehicle is being updated (LM or CSM).
# VEHUPFLG: Set = updating CSM state, Clear = updating LM state
		BOFF	CALL		# If updating LM (VEHUPFLG off)
			VEHUPFLG
			DOLEM		# Go to LM update branch
			INTSTALL	# CSM update: Install integration parameters
# CSM update path: Perform precision integration for CSM state vector
		CLEAR	CALL		# LM PRECISION INTEGRATION (for CSM update)
			VINTFLAG	# Clear velocity integration flag
			SETIFLGS	# Set integration flags
		CALL
			INTGRCAL	# Perform integration calculation
		CALL
			GRP2PC		# Group to packed format conversion
		CALL
			INTSTALL	# Install integration parameters again
# Determine if W-matrix integration is needed for CSM
		CLEAR	BOFF
			DIM0FLAG	# Clear dimension 0 flag
			RENDWFLG	# Check rendezvous W-matrix flag
			NOTWCSM		# Branch if no W-matrix integration
		SET	SET		# CSM WITH W-MATRIX INTEGRATION
			DIM0FLAG	# Set dimension 0 flag for 6-dimensional
			D6OR9FLG	# Set 6 or 9 dimension flag
# Final integration for CSM (with or without W-matrix)
NOTWCSM		SET	CLEAR
			VINTFLAG	# Set velocity integration flag
			INTYPFLG	# Clear integration type flag
		SET	CALL
			STATEFLG	# Set state vector flag
			INTGRCAL	# Perform integration calculation
		GOTO
			MARKTEST	# Go to mark test section
# ============================================================================
# LM UPDATE PATH: Integration setup when updating LM state vector
# ============================================================================
DOLEM		CALL
			INTSTALL	# Install integration parameters for LM
		SET	CALL
			VINTFLAG	# Set velocity integration flag
			SETIFLGS	# Set integration flags
		CALL
			INTGRCAL	# Perform integration calculation
# Page 576
		CALL
			GRP2PC		# Group to packed format conversion
		CALL
			INTSTALL	# Install integration parameters
# Determine if W-matrix integration is needed for LM
		CLEAR	BOFF
			DIM0FLAG	# Clear dimension 0 flag
			RENDWFLG	# Check rendezvous W-matrix flag
			NOTWLEM		# Branch if no W-matrix integration
		SET	SET		# LM WITH W-MATRIX INTEGRATION
			DIM0FLAG	# Set dimension 0 flag for 6-dimensional
			D6OR9FLG	# Set 6 or 9 dimension flag
# Final integration for LM (with or without W-matrix)
NOTWLEM		CLEAR	CLEAR
			INTYPFLG	# Clear integration type flag
			VINTFLAG	# Clear velocity integration flag
		SET	CALL
			STATEFLG	# Set state vector flag
			INTGRCAL	# Perform integration calculation
# ============================================================================
# W-MATRIX VALIDATION AND RANGE CHECK
# The W-matrix (state covariance) can be invalidated by maneuvers or large
# state changes. If invalidated, reinitialize before processing measurements.
# ============================================================================
MARKTEST	BON	CALL		# HAS W-MATRIX BEEN INVALIDATED
			RENDWFLG	# Check rendezvous W-matrix flag
			RANGEBQ		# No - proceed to range check
			WLINIT		# YES -- REINITIALIZE W-matrix
# ============================================================================
# RANGE CHECK AND SPHERE OF INFLUENCE SETUP
# Determine scaling factors based on whether we're in Earth's or Moon's
# gravitational sphere of influence. During Apollo 11 lunar orbit rendezvous,
# Moon-centered coordinates (LMOONFLG set) were used for both spacecraft.
# ============================================================================
;
; RANGEBQ: Range and Range-Rate Measurement Processing Entry Point
;
; Eagle's rendezvous radar has successfully locked onto Columbia and measured:
;   1. Range (distance between spacecraft)
;   2. Range-rate (closing velocity)
; These measurements must be processed through Kalman filtering to correct
; the state vectors of both spacecraft. During Apollo 11's rendezvous on
; July 21, 1969, this routine processed approximately 100 range measurements
; as Eagle closed from 100 miles to docking distance.
;
RANGEBQ		BON	EXIT		# DON'T CALL R65 IF ON SURFACE
			SURFFLAG	# Check if on lunar surface
			RANGEBQ1	# Skip R65 range deviation display if on surface
;
; If not on lunar surface (in free space), display range deviation to crew.
; R65 displays predicted vs. actual range on DSKY, allowing Armstrong and
; Aldrin to monitor tracking quality and detect anomalies. R65CNTR tracks
; how many times this display has been updated.
;
		CA	ZERO		# Initialize R65 counter
		TS	R65CNTR
		TC	BANKCALL	# Call range deviation display routine
		CADR	R65LEM
		TC	INTPRET		# Return to interpretive mode
;
; Sphere of Influence Setup: Determine correct scaling for position vectors
;
; The AGC must scale position vectors differently depending on whether the
; spacecraft are in Earth's or Moon's gravitational sphere of influence.
; SCALSHFT register controls this scaling:
;   SCALSHFT = 0 (Moon sphere): Positions scaled by B-27 (smaller distances)
;   SCALSHFT = 2 (Earth sphere): Positions scaled by B-29 (larger distances)
; During Apollo 11 rendezvous in lunar orbit, Moon scaling was used.
;
RANGEBQ1	AXT,2	BON		# CLEAR X2 (index register)
			0		# X2 = 0 for Moon sphere
			LMOONFLG	# IS MOON SPHERE OF INFLUENCE
			SETX2		# YES.  STORE ZERO IN SCALSHFT REGISTER
		INCR,2		# If Earth sphere, increment X2
			2		# X2 = 2 for Earth sphere
;
; Store scaling shift and convert state vectors to packed format for
; measurement processing. Group 2 packing format stores position and
; velocity compactly for efficient computation.
;
SETX2		SXA,2	CALL
			SCALSHFT	# 0 -- MOON.  2 -- EARTH.
			GRP2PC		# Group to packed conversion
# ============================================================================
# RANGE MEASUREMENT PROCESSING
# Process radar range measurement and compute state correction (DELTAQ).
# Range measurement has inherent variance (RVARMIN) representing sensor noise.
# B-vector represents sensitivity of measurement to state changes.
# ============================================================================
;
; Range Measurement Setup: Prepare for range data processing
;
; Set measurement type code to 1 (range) for DSKY display via noun 49.
; This allows crew to see which measurement type is being processed.
; During rendezvous, crew monitored noun 49 to track measurement updates.
;
		AXT,1	SXA,1		# STORE RANGE CODE (1) FOR R3 IN NOUN 49
			1		# Measurement type index (1 = range)
			WHCHREAD	# Which reading indicator for display
;
; Load minimum range measurement variance (RVARMIN) representing radar
; sensor noise. Rendezvous radar has inherent measurement uncertainty
; (typically ~50 feet RMS for range). This variance is used in Kalman
; filter gain computation to weight the measurement appropriately.
; Variance scaled from B-12 to B-40 (triple precision) for computation.
;
		SLOAD	SR		# GET SINGLE PRECISION RVARMIN (B-12)
			RVARMIN		# Minimum range measurement variance
			28D		# SHIFT TO TRIPLE PRECISION (B-40)
		RTB			# Convert to triple precision mode
			TPMODE		# AND SAVE IN 20D
		STORE	20D		# Minimum variance stored for comparison
;
; Compute B-vector (measurement sensitivity vector) for range measurement.
; B-vector points along line-of-sight from LM to CSM, representing how
; changes in relative position affect the range measurement. GETULC
; computes unit line-of-sight vector from relative position vector.
;
		CALL			# BEGIN COMPUTING THE B-VECTORS, DELTAQ
			GETULC		# Get unit line-of-sight vector (B-VECTORS FOR RANGE)
;
; B-VECTOR COMPUTATION
; The B-vector (bias vector) represents expected measurement values computed
; from current state estimates. For range measurements, B0 is the computed
; line-of-sight range from LM to CSM. B1 and B2 are set to zero for range
; (only B0 is needed). The actual radar measurement minus B0 gives the
; measurement residual that drives the Kalman filter update.
;
		BON	VCOMP		# B0, COMP. IF LM BEING CORRECTED
# Page 577
			VEHUPFLG
			+1
		STOVL	BVECTOR
			ZEROVECS
		STORE	BVECTOR +6	# B1
		STODL	BVECTOR +12D	# B2
;
; Compute measurement variance (uncertainty in this radar range measurement).
; The variance depends on range magnitude and measurement noise characteristics.
; Start with RLC (computed range in Moon-centered coordinates, B-29 scaling).
;
			36D
		SRR*	BDSU
			2,2		# SHIFT FROM EARTH/MOON SPHERE TO B-29
			RM		# RM - (MAGNITUDE RCSM-RLM)
		SLR*
			2,2		# SHIFT TO EARTH/MOON SPHERE
		STODL	DELTAQ		# EARTH B-29.  MOON B-27
			36D		# RLC B-29/B-27
;
; VARIANCE CALCULATION FOR RANGE MEASUREMENT
; Normalize RLC to determine scale, then square it. Multiply by RANGEVAR
; (the base measurement variance constant, B-12 scaling) to get variance
; scaled to this specific range. The variance increases with range because
; radar accuracy degrades at longer distances.
;
		NORM	DSQ		# NORMALIZE AND SQUARE
			X1
		DMP	SR*
			RANGEVAR	# MULTIPLY BY RANGEVAR (B12) THEN
			0 -2,1		# UNNORMALIZE
		SR*	SR*
			0,1
			0,2
		SR*	RTB
			0,2
			TPMODE
		STORE	VARIANCE	# B-40
;
; Check if computed variance is positive. Negative variance would indicate
; computational error. If negative, use the constant backup value of 20D.
; During Apollo 11 rendezvous, typical range variance was on the order of
; 0.1 nautical miles squared, corresponding to radar accuracy of ~0.3 nm.
;
		DCOMP	TAD
			20D		# B-40
		BMN	TLOAD
			QOK
			20D		# B-40
		STORE	VARIANCE
;
; Call LGCUPDTE (Lunar Guidance Computer Update) to incorporate this range
; measurement into the state vector. LGCUPDTE implements the Kalman filter
; measurement update equations using the computed B-vector, measurement
; variance, and the actual radar range measurement.
;
QOK		CALL
			LGCUPDTE

;
; ============================================================================
; RANGE-RATE MEASUREMENT PROCESSING
; Range measurement update complete. Now process range-rate (closing velocity)
; measurement. Range-rate tells us how fast the distance between Eagle and
; Columbia is changing - negative means closing (approaching), positive means
; opening (separating). During Apollo 11 rendezvous, range-rate was crucial
; for timing the final braking maneuvers.
; ============================================================================
;
		SSP	CALL
			WHCHREAD
		DEC	2		# STORE R-RATE CODE (2) FOR R3 IN NOUN 49
			GRP2PC
		CALL			# B-VECTOR, DELTAQ FOR RANGE RATE
			GETULC
;
; Retrieve RLC (computed range) and rescale for range-rate processing.
; Range-rate B-vector computation requires slightly different scaling than
; range itself. Shift from B-29/B-27 to B-23 scaling to match velocity units.
;
		PDDL	SR*		# GET RLC SCALED B-29/B-27
			36D		# AND SHIFT TO B-23
			0 -4,2
		STOVL	36D		# THEN STORE BACK IN 36D
;
; B-VECTOR FOR RANGE-RATE
; B1 (expected range-rate) is computed as the dot product of the relative
; velocity vector with the line-of-sight unit vector. If VEHUPFLG is set,
; we're updating the LM state vector, so use the velocity vector appropriately.
; B0 and B2 remain as previously computed.
;
		BON	VCOMP		# B1, COMP. IF LM BEING CORRECTED
			VEHUPFLG
			+1
		VXSC
			36D		# B1 = RLC (B-24/B-22)
# Page 578
		STOVL	BVECTOR +6
			NUVLEM
;
; RELATIVE VELOCITY COMPUTATION
; Compute the relative velocity vector VLC = VC - VL (Columbia velocity minus
; Eagle velocity). This is needed to calculate the expected range-rate as the
; component of relative velocity along the line of sight.
;
		VSR*	VAD
			6,2		# SHIFT FOR EARTH/MOON SPHERE
			VCVLEM		# EARTH B-7. MOON B-5
		PDVL	VSR*		# VL TO PD6
			NUVCSM
			6,2		# SHIFT FOR EARTH/MOON SPHERE
		VAD	VSU
			VCVCSM
		PDVL	DOT		# VC - VL = VLC TO PD6
			0
			6
;
; Compute RDOT (range-rate) as the dot product of relative velocity VLC
; with the line-of-sight unit vector. Positive RDOT means separating,
; negative means closing. During Apollo 11's final approach, RDOT was
; carefully monitored to ensure proper closing velocity for safe docking.
;
		PUSH	SRR*		# RDOT B-8/B-6 TO PD12
			2,2		# SHIFT FROM EARTH/MOON SPHERE TO B-8
;
; RANGE-RATE MEASUREMENT VARIANCE
; Compute variance for the range-rate measurement. Square RDOT and multiply
; by RATEVAR (range-rate measurement noise constant). This variance represents
; the uncertainty in the radar's velocity measurement capability.
;
		DSQ	DMPR		# RDOT**2 B-16 X RATEVAR B12
			RATEVAR
		STORE	VARIANCE
;
; Apply minimum variance limit. Very small computed variances can cause
; numerical instability in the Kalman filter. If computed variance is less
; than VVARMIN (minimum velocity variance constant), use VVARMIN instead.
; This ensures the filter doesn't over-trust noisy measurements.
;
		SLOAD	SR
			VVARMIN		# GET SINGLE PRECISION VVARMIN (B+12)
			16D		# SHIFT TO DP (B-4)
		STORE	24D		# AND SAVE IN 24D
		DSU	BMN		# IS MIN. VARIANCE > COMPUTED VARIANCE
			VARIANCE
			VOK		# BRANCH -- NO
		DLOAD			# YES -- USE MINIMUM VARIANCE
			24D
		STORE 	VARIANCE
;
; Prepare RDOT value and normalization parameters for incorporation into
; the state vector via Kalman filter update.
;
VOK		DLOAD	SR2		# RDOT (PD12) FROM B-8/B-6
		PDDL	SLR*		# TO B-10/B-8
			RDOTM		# SHIFT TO EARTH/MOON SPHERE
			0 -1,2		# B-7 TO B-10/B-8
		DSU			# Compute range rate difference
		DMPR			# Multiply by gain matrix element (36D)
			36D		# Gain term for state correction
		STOVL	DELTAQ		# Store range residual (B-33)
			0		# NOW GET B0 - load cross product result
# ============================================================================
# B-VECTOR COMPUTATION FOR RANGE MEASUREMENT
# B0 = (ULC x VLC) x ULC represents sensitivity of range measurement to
# state vector changes. Cross products compute vector perpendicular to
# line-of-sight, establishing measurement geometry for Kalman filter update.
# ============================================================================
		VXV	VXV		# (ULC X VLC) X ULC - compute B0 vector
		BON	VCOMP		# Check vehicle update flag
			VEHUPFLG	# If updating LM (flag set)
			+1		# Skip VCOMP
		VSR*			# COMP. IF LM BEING CORRECTED
			0 -2,2		# SCALED B-5 (shift depends on sphere)
		STOVL	BVECTOR		# Store normalized B-vector for range
			ZEROVECS	# Load zero vector for initialization
		STORE	20D		# ZERO OUT 20 TO 25 IN PUSHLIST
		STOVL	BVECTOR +12D	# Zero high-order B-vector component
			BVECTOR		# Reload B0 for normalization
# ============================================================================
# B-VECTOR NORMALIZATION
# Normalize B-vectors to unit magnitude to prevent numerical overflow during
# W-matrix computation. Track shift counts to properly scale final DELTAQ.
# ============================================================================
		ABVAL	NORM		# Get magnitude and normalize B0
			20D		# SHIFT COUNT IN 20D (for variance scaling)
# Page 579
# ============================================================================
# B1 VECTOR NORMALIZATION (FOR ANGLE MEASUREMENTS)
# Process second B-vector representing sensitivity of angle measurements to
# state changes. Must normalize both B0 and B1 with consistent scaling.
# ============================================================================
		VLOAD	ABVAL		# Load B1 vector
			BVECTOR +6D	# LOAD B1, GET MAGNITUDE AND NORMALIZE
		NORM	DLOAD		# Normalize and get shift count
			22D		# SHIFT COUNT IN 22D (for B1)
			22D		# FIND WHICH SHIFT IS SMALLER
		DSU	BMN		# Compare shift counts (22D - 20D)
			20D		# B0 shift count
			VOK1		# BRANCH -- B0 HAS A SMALLER SHIFT COUNT
		LXA,1	GOTO		# B1 has smaller shift, use it
			22D		# LOAD X1 WITH THE SMALLER SHIFT COUNT (B1)
			VOK2
VOK1		LXA,1			# B0 has smaller shift, use it
			20D		# Load X1 with B0 shift count
# ============================================================================
# CONSISTENT SCALING OF ALL MEASUREMENT QUANTITIES
# Apply smaller shift count to B0, B1, DELTAQ, and variance. This ensures
# all quantities maintain consistent precision and prevents overflow in
# subsequent W-matrix and state correction computations.
# ============================================================================
VOK2		VLOAD	VSL*		# THEN ADJUST B0, B1, DELTAQ AND VARIANCE
			BVECTOR		# WITH THIS SHIFT COUNT
			0,1		# Shift by smaller count (in X1)
		STOVL	BVECTOR		# Store scaled B0
			BVECTOR +6	# Load B1
		VSL*			# Scale B1 by same amount
			0,1
		STODL	BVECTOR +6	# Store scaled B1
			DELTAQ		# Load range residual
		SL*			# Scale DELTAQ by same shift
			0,1
		STORE	DELTAQ		# Store scaled residual
		DLOAD	SL*		# GET RLC AND ADJUST FOR SCALE SHIFT
			36D		# Gain matrix element
			0 -1,1		# Scale adjustment
		DSQ	DMP		# MULTIPLY RLC**2 BY VARIANCE
			VARIANCE	# Measurement variance
		SL4	RTB		# SHIFT TO CONFORM TO BVECTORS AND DELTAQ
			TPMODE		# Convert to triple precision
		STCALL	VARIANCE	# AND STORE TP VARIANCE
			LGCUPDTE	# Call state vector update routine

		CALL			# Convert from packed format
			GRP2PC
		BON	EXIT		# ARE ANGLES TO BE DONE
			SURFFLAG	# Surface flag (if set, skip angles)
			RENDEND		# NO - skip to end
# ============================================================================
# ANGLE MEASUREMENT PROCESSING (MXMYMZ)
# Compute rendezvous radar antenna gimbal angles in stable member coordinates.
# MX, MY, MZ represent antenna pointing direction which, when compared to
# computed line-of-sight, yields angle measurement residuals for navigation
# update. Critical for Columbia-Eagle rendezvous after lunar ascent.
# ============================================================================
		EBANK=	AIG
MXMYMZ		CAF	AIGBANK		# Set up extended memory bank
		TS	BBANK		# Bank containing AIG, AMG, AOG
		CA	AIG		# YES, COMPUTE	MX, MY, MZ
		TS	CDUSPOT		# Inner gimbal angle to CDUSPOT
		CA	AMG		# Middle gimbal angle
		TS	CDUSPOT +2	# Store middle gimbal
		CA 	AOG		# Outer gimbal angle
		TS	CDUSPOT +4	# GIMBAL ANGLES NOW IN CDUSPOT FOR TRG*NBSM
		TC	INTPRET		# Enter interpreter mode
# Page 580
# ============================================================================
# TRANSFORMATION OF RADAR ANTENNA AXES TO STABLE MEMBER COORDINATES
# Transform unit vectors along radar antenna axes through gimbal rotations
# (TRG*NBSM applies all three gimbal angles to navigation base) then to
# inertial reference frame via REFSMMAT. Results MX, MY, MZ represent
# actual antenna pointing direction for comparison with computed line-of-sight.
# ============================================================================
		VLOAD	CALL		# Load unit X vector
			UNITX		# [1,0,0] in radar antenna coordinates
			TRG*NBSM	# Transform through all gimbal angles
		VXM	VSL1		# Multiply by REFSMMAT
			REFSMMAT	# Reference to stable member matrix
		STOVL	MX		# Store MX (X-axis in stable member coords)
			UNITY		# Load unit Y vector [0,1,0]
		CALL			# Transform through gimbal angles
			*NBSM*		# Second transformation routine
		VXM	VSL1		# Multiply by REFSMMAT
			REFSMMAT	# Transform to stable member frame
		STOVL	MY		# Store MY (Y-axis in stable member coords)
			UNITZ		# Load unit Z vector [0,0,1]
		CALL			# Transform through gimbal angles
			*NBSM*		# Third transformation routine
		VXM	VSL1		# Multiply by REFSMMAT
			REFSMMAT	# Transform to stable member frame
# ============================================================================
# SHAFT ANGLE MEASUREMENT PROCESSING
# Compute shaft angle (radar antenna elevation) from dot products of
# line-of-sight (ULC) with transformed antenna axes. ARCTAN of projections
# yields measured shaft angle for comparison with computed angle.
# ============================================================================
SHAFTBQ		STCALL	MZ		# Store MZ, call angle computation
			RADARANG	# Compute arctangent from sine/cosine
# Shaft angle read code stored for telemetry (R3 display in Noun 49)
		SSP	CALL		# STORE SHAFT CODE (3) FOR R3 IN NOUN 49
			WHCHREAD	# Which reading indicator (shaft)
		DEC	3		# Code 3 = shaft angle measurement
			GRP2PC		# Group 2 program control
# ============================================================================
# SHAFT ANGLE DELTA-Q AND B-VECTOR COMPUTATION
# Compute shaft angle measurement residual (delta-Q) and observation geometry
# (B-vector). Shaft angle is elevation of antenna above LM body XZ-plane.
# Delta-Q = (computed shaft from ULC) - (measured shaft from RRSHAFT).
# B-vector direction determined by cross product of ULC with MY axis.
# ============================================================================
		VLOAD	DOT		# COMPUTE DELTAQ,B VECTORS FOR SHAFT ANG.
			ULC		# Unit line-of-sight vector
			MX		# Dot with X-axis in stable member
		SL1			# Scale for arctangent input
		STOVL	SINTH		# 18D - Store sine component
			ULC		# Reload unit line-of-sight
		DOT	SL1		# Dot with Z-axis
			MZ		# Z-axis in stable member frame
		STCALL	COSTH		# 16D - Store cosine component
			ARCTAN		# Compute arctangent(SINTH/COSTH)
# Compute measurement residual (computed angle - measured angle)
		BDSU	DMP		# Subtract measured shaft angle
			RRSHAFT		# Radar shaft angle from hardware
			2PI/8		# Convert to radians (360°/8 = 45° units)
		SL3R	PUSH		# Scale and push to stack
# Apply range-dependent scaling for delta-Q based on sphere of influence
		DLOAD	SL3		# Load scaling exponent
			X789		# Sphere indicator from earlier computation
		SRR*	BDSU		# SHIFT FROM -5/-3 TO B0
			0,2		# Variable shift based on sphere
		DMP	SRR*		# Multiply by range
			RXZ		# Range magnitude
			0,1		# SHIFT TO EARTH/MOON SPHERE
		STOVL	DELTAQ		# EARTH B-29.  MOON B-27 - Store residual
# ============================================================================
# B-VECTOR DIRECTION FOR SHAFT MEASUREMENT
# B-vector points in direction of maximum sensitivity to shaft angle error.
# Cross product of line-of-sight (ULC) with Y-axis (MY) produces vector
# perpendicular to radar beam in vertical plane. Unit vector required for
# observation geometry weighting in Kalman filter W-matrix.
# ============================================================================
			ULC		# Unit line-of-sight vector
		VXV	VSL1		# Cross product with Y-axis
			MY		# Y-axis in stable member frame
		UNIT			# Normalize to unit vector B0
# Check which vehicle state vector is being updated (LM or CSM)
		BOFF	VCOMP		# B0, COMP. IF CSM BEING CORRECTED
# Page 581
			VEHUPFLG	# Vehicle update flag (0=LM, 1=CSM)
			+1		# Skip complement if LM being updated
# Store B-vector components in measurement geometry array
# Format: [Bx By Bz 0 0 0 range_factor]
		STOVL	BVECTOR		# Store unit direction vector B0
			ZEROVECS	# Load zero vector
		STORE	BVECTOR +6	# Store zeros in velocity sensitivity
		STODL	BVECTOR +12D	# Store zero, then load range
			RXZ		# Range magnitude
		SR*	SRR*		# SHIFT FROM EARTH/MOON SPHERE TO B-25
			0 -2,1		# Variable shift for proper scaling
			0,2		# Additional shift based on sphere
		STORE	BVECTOR +12D	# Store range factor at end of B-vector
# ============================================================================
# SHAFT ANGLE MEASUREMENT VARIANCE COMPUTATION
# Compute total variance for shaft angle measurement including radar variance
# (SHAFTVAR) and IMU gimbal uncertainty (IMUVAR). Scale by range squared to
# get position variance, then shift for Earth/Moon sphere and store as triple
# precision. This variance is used by Kalman filter to weight measurement.
# ============================================================================
		SLOAD			# Load shaft angle variance
			SHAFTVAR	# Radar angle measurement variance rad²
		DAD	DMP		# Add IMU variance and multiply by range
			IMUVAR		# RAD**2 B12
			RXZ		# Range magnitude
		SRR*	DMP		# SHIFT TO EARTH/MOON SPHERE
			0,1		# Variable shift for sphere scaling
			RXZ		# Multiply by range again (variance ∝ range²)
		SR*	SR*		# Additional shifts for proper scaling
			0 -2,1		# Shift to B-25 scale
			0,2		# Second shift component
		SR*	RTB		# Final shift and mode conversion
			0,2		# To B-40 scale
			TPMODE		# STORE VARIANCE TRIPLE PRECISION
		STCALL	VARIANCE	# B-40, then call state vector update
			LGCUPDTE	# Incorporate measurement into state vector

# Increment radar mark counter after successful shaft angle measurement
		CALL
			GRP2PC		# Increment counter, check for mark limit
# ============================================================================
# TRUNNION ANGLE MEASUREMENT PROCESSING
# Trunnion angle is the second gimbal angle, orthogonal to shaft angle.
# Similar processing as shaft: compute angle from radar antenna geometry,
# compare with measured angle, compute B-vector direction, scale variance,
# and incorporate measurement into navigation state vector via Kalman filter.
# ============================================================================
TRUNBQ		CALL
			RADARANG	# Compute arctangent from sine/cosine
		SSP	CALL		# STORE TRUNNION CODE (4) FOR R3 IN N49
			WHCHREAD	# Which reading flag (telemetry display)
		DEC	4		# Code 4 = trunnion angle
			GRP2PC		# Update display, increment counter
# Compute trunnion angle B-vector from cross products
# B-vector points in direction of maximum measurement sensitivity
		VLOAD	VXV		# Load line-of-sight, cross with MY
			ULC		# Unit line-of-sight vector to CSM
			MY		# Transformed Y-axis of radar antenna
		VSL1	VXV		# Scale, then cross product with ULC again
			ULC		# Second cross product: (ULC × MY) × ULC
		VSL1			# Scale result to B0 (unit vector)
# Check which vehicle being updated, complement B-vector if updating CSM
		BOFF	VCOMP		# B0, COMP. IF CSM BEING CORRECTED
			VEHUPFLG	# Vehicle update flag
			+1		# Skip complement if LM
# Store trunnion B-vector with same format as shaft measurement
		STOVL	BVECTOR		# Store unit direction vector
			ZEROVECS	# Load zero vector
		STORE	BVECTOR +6	# Zero velocity sensitivity
		STODL	BVECTOR +12D	# Zero, then load range
			RXZ		# Range magnitude
# Page 582
		SR*	SRR*		# SHIFT FROM EARTH/MOON SPHERE TO B-25
			0 -2,1		# Variable shift for scaling
			0,2		# Sphere-dependent shift
		STORE	BVECTOR +14D	# Store range factor
# ============================================================================
# TRUNNION ANGLE MEASUREMENT VARIANCE COMPUTATION
# Identical process to shaft variance: combine radar and IMU uncertainties,
# scale by range squared, shift to triple precision B-40 scale. Trunnion
# variance typically similar to shaft variance for rendezvous radar.
# ============================================================================
		SLOAD			# Load trunnion angle variance
			TRUNVAR		# Radar trunnion measurement variance rad²
		DAD	DMP		# Add IMU variance, multiply by range
			IMUVAR		# IMU gimbal uncertainty
			RXZ		# Range magnitude
		SRR*	DMP		# SHIFT TO EARTH/MOON SPHERE
			0,1		# Sphere-dependent shift
			RXZ		# Multiply by range (variance ∝ range²)
		SR*	SR*		# Scale to B-25
			0 -2,1		# First shift component
			0,2		# Second shift component
		SR*	RTB		# Final shift and conversion
			0,2		# To B-40 scale
			TPMODE		# STORE VARIANCE TRIPLE PRECISION
		STODL	VARIANCE	# Store variance, load sine theta
			SINTHETA	# Sine of computed angle
# Compute measurement residual (delta-Q) for trunnion angle
# Same structure as shaft: arcsine of computed angle minus measured angle,
# scale to arc length displacement, apply coordinate transformations
		ASIN	BDSU		# SIN THETA IN PD6, compute arcsine
			RRTRUN		# Measured trunnion angle from radar
		DMP	SL3R		# Convert to full rotation units
			2PI/8		# Scale factor: 2π/8 radians
		PDDL	SL3		# Push to stack, load transformation component
			X789 +2		# Coordinate transformation parameter
		SRR*	BDSU		# SHIFT FROM -5/-3 TO B0, apply shift
			0,2		# Sphere-dependent shift factor
		DMP	SRR*		# Multiply by range to get arc displacement
			RXZ		# Range magnitude
			0,1		# Sphere shift (Earth vs Moon scaling)
		STCALL	DELTAQ		# EARTH B-29.  MOON B-27, store residual
			LGCUPDTE	# Incorporate measurement into state vector
# Increment radar mark counter after successful trunnion measurement
		CALL
			GRP2PC		# Update counter and displays
# ============================================================================
# RENDEZVOUS NAVIGATION MEASUREMENT CYCLE COMPLETION
# After shaft and trunnion angles both processed, return to main R22 loop
# to continue tracking or await next measurement cycle. During Apollo 11
# rendezvous, radar marks processed at ~once per minute as Eagle approached.
# ============================================================================
RENDEND		GOTO
			R22LEM93	# Return to main tracking loop

# ============================================================================
# LSR22.4 - LUNAR SURFACE NAVIGATION (RANGE/RANGE-RATE PROCESSING)
#
# FUNCTIONAL DESCRIPTION:
#	LSR22.4 IS THE ENTRY TO PERFORM LUNAR SURFACE NAVIGATION FOR THE LM
#	COMPUTER ONLY.  THIS ROUTINE COMPUTES THE BE-VECTORS AND DELTA Q FOR RANGE
#	AND RANGE RATE MEASURED BY THE RENDEZVOUS RADAR
#
# MISSION CONTEXT:
#	During Apollo 11 rendezvous on July 21, 1969, after Eagle's ascent from
#	the lunar surface, this routine processed rendezvous radar range and
#	range-rate measurements to Columbia. Eagle locked onto Columbia at ~100
#	nautical miles range. Range measurements accurate to ±5 feet, range-rate
#	to ±0.5 feet/second, enabling precise rendezvous navigation.
#
# SUBROUTINES CALLED:
#	INTSTALL	LGCUPDTE	INCORP1		RP-TO-R
#	INTEGRV		GETULC		INCORP2
#
# OUTPUT
#	CORRECTED CSM STATE VECTOR (PERMANENT)
#	NUMBER OF MARKS INCORPORATED IN MARKCTR
# Page 583
#	MAGNITUDE OF POSITION DEVIATION (FOR DISPLAY) IN R22 DISP METERS B-29
# 	MAGNITUDE OF VELOCITY DEVIATION (FOR DISPLAY) IN R22DISP +2 M/CSEC B-7
# 	UPDATED W-MATRIX (STATE ERROR COVARIANCE)

# ERASABLE INITIALIZATION REQUIRED
#	LM AND CSM STATE VECTORS
#	W-MATRIX (6X6 COVARIANCE MATRIX)
#	MARK TIME IN MKTIME (CENTISECONDS)
#	RADAR RANGE IN RM METERS B-29
#	RANGE RATE IN RDOTM METERS/CSEC B-7
#	VEHUPFLG (VEHICLE UPDATE FLAG: 0=LM, 1=CSM)
# ============================================================================

;
; ENTRY POINT: LSR22.4 begins measurement processing cycle
; Eagle's rendezvous radar has just captured range, range-rate, shaft angle,
; and trunnion angle measurements to Columbia. These raw measurements must now
; be processed to update both spacecraft's state vectors using Kalman filtering.
;
LSR22.4		CALL
			INTSTALL	# Install integration service routines
;
; Set flags to integrate both LM and CSM state vectors forward to the exact
; measurement time (MKTIME). This ensures that when we compute expected
; measurements from the state vectors, they correspond to the same instant
; as the actual radar measurements.
;
		SET	CLEAR		# Set up flags for state vector processing
			STATEFLG	# Indicate state vector integration needed
			VINTFLAG	# CALL TO GET LM POS + VEL IN REF COORD.
		CALL
			INTGRCAL	# Integrate state to mark time
		CALL
			GRP2PC		# Convert group 2 state vector format
		CLEAR	CALL
			DMENFLG		# SET MATRIX SIZE TO 6X6 FOR INCORP
			INTSTALL	# Reinstall for measurement processing
;
; Check if this is the first radar measurement of the tracking session.
; MARKCTR (mark counter) equals zero on first call. If so, the W-matrix
; (state error covariance matrix) must be initialized with diagonal elements
; representing initial uncertainty in relative position and velocity.
;
		DLOAD	BHIZ		# IS THIS FIRST TIME THROUGH
			MARKCTR		# Mark counter = 0 on first call
			INITWMX6	# YES, INITIALIZE 6X6 W-MATRIX
;
; NOT FIRST TIME: W-matrix already initialized, continue with measurement
; processing. Set up for second integration pass with velocity integration.
;
		CLEAR	SET
			D6OR9FLG	# Clear 6-or-9 dimension flag (use 6x6)
			DIM0FLAG	# Set dimension 0 flag
		SET	CLEAR		# Set up integration flags
			VINTFLAG	# Request velocity integration
			INTYPFLG	# Clear integration type flag
		CALL
			INTGRCAL	# Integrate state to mark time
;
; State vectors are now aligned to measurement time. Branch to RANGEBQ to
; begin processing the four radar measurements: range, range-rate, shaft
; angle (elevation), and trunnion angle (azimuth).
;
		GOTO
			RANGEBQ		# Branch to range/range-rate processing

# ============================================================================
# INITWMX6 - INITIALIZE 6x6 W-MATRIX FOR FIRST RENDEZVOUS MEASUREMENT
#
# Called on first range/range-rate measurement to initialize state error
# covariance matrix. Sets up 6x6 matrix tracking uncertainty in relative
# position and velocity between LM and CSM. Matrix elements represent
# covariance of position errors (3x3) and velocity errors (3x3).
# ============================================================================
INITWMX6	CALL
			WLINIT		# INITIALIZE W-MATRIX
		SET	CALL		# Set flags for initialization
			VINTFLAG	# Request velocity integration
			SETIFLGS	# Set integration flags
		CALL
			INTGRCAL	# Integrate to mark time
		GOTO
			RANGEBQ		# Continue to measurement processing

# ============================================================================
# INTGRCAL - INTEGRATION CALL WRAPPER
#
# Utility routine to integrate state vectors to measurement mark time (MKTIME).
# Clears RFINAL and sets up time for INTEGRV integration routine. Used to
# propagate LM and CSM states to the exact time of radar measurement.
# ============================================================================
# THIS ROUTINE CLEARS RFINAL (DP) AND CALLS INTEGRV
# Page 584
INTGRCAL	STQ	DLOAD		# Store return address, load mark time
			IGRET		# Return address
			MKTIME		# Time of radar measurement (centisecs)
		STCALL	TDEC1		# Store in integration time, call integrator
			INTEGRV		# Integrate state to TDEC1 time
		GOTO
			IGRET		# Return to caller

;
; ============================================================================
; WLINIT - W-MATRIX INITIALIZATION ROUTINE
;
; Clears all elements of the W-matrix to zero, then loads diagonal elements
; with initial variance values representing state uncertainty. The W-matrix
; is a symmetric matrix tracking position and velocity error covariances.
;
; For lunar orbit rendezvous: Uses WRENDPOS and WRENDVEL
; For lunar surface operations: Uses WSURFPOS and WSURFVEL
;
; The diagonal values represent squared uncertainty (variance) in each
; state component. As radar measurements are incorporated, these values
; decrease, indicating improved navigation accuracy.
; ============================================================================
;
		EBANK=	W
WLINIT		EXIT
;
; Switch to W-matrix bank and zero out all matrix elements using indexed loop.
; This ensures clean initialization before setting diagonal elements.
;
		CAF	WBANK
		TS	BBANK
		CAF	WSIZE
		TS	W.IND
		CAF	ZERO
		INDEX	W.IND
		TS	W
		CCS	W.IND
		TC	-5
;
; Restore bank and return to interpretive mode to load diagonal values.
;
		CAF	AIGBANK		# RESTORE EBANK 7
		TS	BBANK
		TC	INTPRET
;
; Check if on lunar surface. If so, use surface-specific uncertainty values
; (WSURFPOS, WSURFVEL) which are typically smaller than orbital values
; (WRENDPOS, WRENDVEL) because surface position is better known.
;
		BON	SLOAD		# IF ON LUNAR SURFACE, INITIALIZE WITH
			SURFFLAG	# WSURFPOS AND WSURFVEL INSTEAD OF
			WLSRFPOS	# WRENDPOS AND WRENDVEL
			WRENDPOS
		GOTO
			WPOSTORE
;
; Lunar surface branch: Load surface position uncertainty
;
WLSRFPOS	SLOAD
			WSURFPOS
;
; Store position variance in W-matrix diagonal elements (W, W+8D, W+16D).
; These correspond to X, Y, Z position uncertainty. Scaled to B-19 format.
;
WPOSTORE	SR			# SHIFT TO B-19 SCALE
			5
		STORE	W
		STORE	W +8D
		STORE	W +16D
;
; Load velocity uncertainty values (again checking surface flag)
;
		BON	SLOAD
			SURFFLAG
			WLSRFVEL
			WRENDVEL
		GOTO
			WVELSTOR
;
; Lunar surface branch: Load surface velocity uncertainty
;
WLSRFVEL	SLOAD
			WSURFVEL
;
; Store velocity variance in W-matrix diagonal elements (W+72D, W+80D, W+88D).
; These correspond to Vx, Vy, Vz velocity uncertainty components.
;
WVELSTOR	STORE	W +72D
		STORE	W +80D
		STORE	W +88D
;
; Load and store shaft angle (elevation) measurement uncertainty.
; Shaft angle uncertainty (WSHAFT) placed at W+144D diagonal position.
;
		SLOAD
# Page 585
			WSHAFT
		STORE	W +144D
;
; Load and store trunnion angle (azimuth) measurement uncertainty.
; Trunnion angle uncertainty (WTRUN) placed at W+152D diagonal position.
;
		SLOAD
			WTRUN
		STORE	W +152D
;
; W-matrix initialization complete. Set RENDWFLG to indicate W-matrix is
; valid and ready for Kalman filter measurement incorporation. Reset mark
; counter to zero to begin counting radar measurements for this tracking session.
;
		SET	SSP		# SET RENDWFLG -- W-MATRIX VALID
			RENDWFLG
			MARKCTR		# SET MARK COUNTER EQUAL ZERO
			0
		RVQ

		EBANK=	W
WBANK		BBCON	WLINIT
		EBANK=	AIG
AIGBANK		BBCON	LSR22.3

;
; ============================================================================
; GETULC - COMPUTE UNIT LINE-OF-SIGHT VECTOR
;
; Computes the relative position vector from the Lunar Module to the Command
; Module, normalizes it to a unit vector, and returns both the unit vector
; (ULC - Unit Line-of-sight CSM) and the magnitude (range between spacecraft).
;
; This calculation is fundamental to rendezvous navigation. The unit vector
; defines the line-of-sight direction from Eagle to Columbia, which should
; align with the rendezvous radar boresight axis during tracking. The magnitude
; represents the range between the two spacecraft.
;
; Inputs:
;   DELTALEM, RCVLEM: LM position state and center-of-body position
;   DELTACSM, RCVCSM: CSM position state and center-of-body position
;   SCALSHFT: Scale factor for Earth/Moon sphere-of-influence transitions
;
; Outputs:
;   ULC: Unit line-of-sight vector (pushlist and MPAC)
;   36D: Range magnitude between LM and CSM
; ============================================================================
;
GETULC		SETPD	VLOAD
			0
			DELTALEM
;
; Load scale shift factor to handle transitions between Earth and Moon
; sphere-of-influence. Position vectors scaled differently in cislunar space.
;
		LXA,2
			SCALSHFT	# LOAD X2 WITH SCALE SHIFT
;
; Compute LM absolute position: Vector shift by scale factor, then add
; center position (RCVLEM = position relative to center body).
;
		VSR*	VAD
			9D,2		# SHIFT FOR EARTH/MOON SPHERE
			RCVLEM
;
; Push LM position to stack, then compute CSM absolute position similarly.
;
		PDVL	VSR*
			DELTACSM
			9D,2		# SHIFT FOR EARTH/MOON SPHERE
		VAD	VSU
			RCVCSM
;
; Subtract LM position from CSM position to get relative position vector
; (CSM - LM). This vector points from Eagle to Columbia. Use NORMUNIT to
; normalize to unit length while preserving numerical accuracy.
;
		RTB	PUSH		# USE NORMUNIT TO PRESERVE ACCURACY
			NORMUNX1
;
; NORMUNIT returns unit vector in MPAC and magnitude shift count in X1.
; Store unit vector in ULC, load magnitude from pushdown location 36D.
;
		STODL	ULC
			36D
;
; Adjust magnitude by shifting left according to shift count from NORMUNIT.
; Store final range magnitude in 36D. Load ULC back to MPAC for return.
;
		SL*			# ADJUST MAGNITUDE FROM NORMUNIT
			0,1
		STOVL	36D		# ULC IN PD0 AND MPAC,RLC IN 36D
			ULC
		RVQ

;
; ============================================================================
; RADARANG - COMPUTE RADAR ANGLE AND RANGE GEOMETRY
;
; Computes geometric parameters for rendezvous radar measurements, including
; the sine of the elevation angle (theta) between the line-of-sight vector
; and the LM body Y-axis, and the horizontal range projection (RXZ) in the
; X-Z plane of the LM body coordinate system.
;
; These computations transform the relative position between spacecraft into
; the geometric parameters measured by the rendezvous radar. The radar measures
; range, shaft angle (elevation), and trunnion angle (azimuth) in the LM body
; frame. This routine converts the inertial line-of-sight vector into the
; body-frame geometric quantities needed for radar measurement processing.
;
; Historical Context:
; During Apollo 11's rendezvous on July 21, 1969, after Eagle's ascent from
; the lunar surface, the rendezvous radar first acquired Columbia at approximately
; 100 nautical miles range. RADARANG computed the geometric relationship between
; the spacecraft, converting the tracking data into navigation state updates
; that guided the three rendezvous maneuvers (CSI, CDH, TPI) bringing Eagle
; to within feet of Columbia for docking 3 hours 40 minutes after ascent.
;
; Mathematical Background:
; The radar measures in spherical coordinates (range, elevation, azimuth) in
; the LM body frame. SINTHETA = -ULC dot MY gives the sine of the elevation
; angle, where MY is the LM body Y-axis unit vector and ULC is the unit
; line-of-sight vector. The horizontal range RXZ = sqrt(1-sin²θ) * RLC
; represents the projection of the range onto the X-Z plane.
;
; Inputs:
;   MY: LM body Y-axis unit vector
;   (GETULC provides ULC and range RLC)
;
; Outputs:
;   ULC: Unit line-of-sight vector (in ULC and PD0)
;   RLC: Range magnitude (in PD36D)
;   SINTHETA: sin(elevation angle) (in SINTHETA and PD6)
;   RXZ: Horizontal range projection (magnitude in RXZ, shift count in X1)
; ============================================================================
;
# RADARANG
#
# THIS SUBROUTINE COMPUTS SINTHETA = -ULC DOT MY
# RXZ = (SQRT (1-SINTHETA**2))RLC
# OUTPUT
#	ULC IN ULC, PD0
# Page 586
#	RLC IN PD36D
#	SIN THETA IN SINTHETA AND PD6
#	RXZ NORM IN RXZ (N IN X1)
;
; Store return address, then call GETULC to compute unit line-of-sight vector
; and range magnitude between LM and CSM.
;
RADARANG	STQ	CALL
			RDRET
			GETULC
;
; Complement ULC (negate vector), then compute dot product with MY (LM body
; Y-axis). This gives -ULC dot MY = sin(elevation angle theta). The negative
; sign accounts for the radar coordinate system convention.
;
		VCOMP	DOT
			MY
;
; Shift left 1 bit (SL1R) to scale result, push to pushdown list at PD6,
; and store in SINTHETA. This is sin(elevation angle theta).
;
		SL1R	PUSH		# SIN THETA TO PD6
		STORE	SINTHETA
;
; Compute horizontal range projection: RXZ = sqrt(1-sin²θ) * RLC
; Square SINTHETA (DSQ), subtract from DP1/4TH (which represents 1) to get
; 1-sin²θ = cos²θ. Take square root to get cosθ.
;
		DSQ	BDSU
			DP1/4TH		# 1-(SIN THETA)**2
;
; Multiply by range RLC (at PD36D) to get horizontal range projection.
; Normalize result and store shift count in X1.
;
		SQRT	DMP
			36D
		SL1	NORM
			X1		# SET SHIFT COUNTER IN X1
		STORE	RXZ
;
; Return to caller via address stored in RDRET.
;
		GOTO			# EXIT
			RDRET
;
; ============================================================================
; LGCUPDTE - LM GUIDANCE COMPUTER STATE VECTOR UPDATE
;
; Updates the LM state vector based on navigation measurements (typically
; rendezvous radar tracking data). This routine incorporates the measurement
; residuals (DELTAX) into the state vector using INCORP1, then validates
; that the resulting position and velocity deviations are within acceptable
; limits (RMAX, VMAX) before finalizing the update.
;
; This is a critical navigation function during rendezvous operations, ensuring
; that radar measurements improve state vector knowledge without introducing
; unrealistic or erroneous updates. If deviations exceed limits, the routine
; displays the deviations to the crew for evaluation (R22DISP).
;
; Historical Context:
; During Apollo 11's rendezvous, LGCUPDTE processed rendezvous radar measurements
; every few seconds, updating Eagle's knowledge of its position and velocity
; relative to Columbia. The RMAX and VMAX limits prevented bad measurements
; from corrupting the navigation state, maintaining the accuracy needed for
; the precision rendezvous maneuvers.
;
; Inputs:
;   DELTAX: Measurement residuals (position and velocity corrections)
;   RMAX: Maximum allowable position deviation
;   VMAX: Maximum allowable velocity deviation
;   SCALSHFT: Scaling factor (0 for Moon, 2 for Earth)
;
; Outputs:
;   R22DISP: Position deviation magnitude (for crew display if out of limits)
;   R22DISP+2: Velocity deviation magnitude (for crew display if out of limits)
;   Updated state vector (via INCORP1 and INCORP2)
; ============================================================================
;
LGCUPDTE	STQ	CALL
			LGRET
			INCORP1
;
; Load velocity portion of DELTAX (at +6), compute magnitude (ABVAL).
; Load scaling factor into index register X2, shift right by scale amount.
; This scales velocity deviation for display (B-7 scale factor).
;
		VLOAD	ABVAL
			DELTAX +6
		LXA,2	SRR*
			SCALSHFT	# 0 -- MOON.  2 -- EARTH
			2,2		# SET VEL DISPLAY TO B-7
;
; Store velocity deviation magnitude in R22DISP+2. Load position portion
; of DELTAX, compute magnitude, scale for display (B-29).
;
		STOVL	R22DISP +2
			DELTAX
		ABVAL	SRR*
			2,2		# SET POS DISPLAY TO B-29
		STORE	R22DISP
;
; Check if position deviation exceeds maximum allowable (RMAX).
; Load RMAX, shift right 10 decimal places, subtract R22DISP.
; If negative (position deviation > RMAX), branch to R22LEM96 to display.
;
		SLOAD	SR
			RMAX
			10D
		DSU	BMN
			R22DISP
			R22LEM96	# GO DISPLAY
;
; Check if velocity deviation exceeds maximum allowable (VMAX).
; Load VMAX, subtract velocity deviation (R22DISP+2).
; If negative (velocity deviation > VMAX), branch to R22LEM96 to display.
;
		SLOAD	DSU
			VMAX
			R22DISP +2	# VMAX MINUS VEL. DEVIATION
		BMN
			R22LEM96	# GO DISPLAY
;
; Both position and velocity deviations are within limits. Proceed with
; state vector update by calling INCORP2 to finalize the incorporation.
;
ASTOK		CALL
			INCORP2
;
; Return to caller via address stored in LGRET.
;
		GOTO
			LGRET
IMUVAR		2DEC	E-6 B12		# RAD**2

WSIZE		DEC	161
# Page 587
2PI/8		2DEC	3.141592653 B-2

		EBANK=	LOSCOUNT
# Page 588
;
; ============================================================================
; LRS24.1 - RENDEZVOUS RADAR HEXAGONAL SEARCH PATTERN
;
; Drives the rendezvous radar through a hexagonal search pattern centered on
; the computed line-of-sight (LOS) to the CSM, attempting to acquire the
; target spacecraft. The routine continuously checks for radar lock-on
; (data good discrete) while monitoring the angle between the radar boresight
; and the LM +Z axis to ensure the spacecraft maintains a favorable attitude
; for radar tracking.
;
; Hexagonal Search Pattern:
; The search pattern consists of points arranged in a hexagonal grid around
; the predicted LOS to the CSM. This pattern efficiently scans the uncertainty
; region where the CSM might actually be located, accounting for state vector
; errors. The pattern starts at the center (predicted LOS) and spirals outward
; through successive hexagonal rings if lock-on is not achieved.
;
; Historical Context:
; During Apollo 11's rendezvous on July 21, 1969, after Eagle's ascent from
; the lunar surface, the rendezvous radar had to acquire Columbia, initially
; about 100 nautical miles away. State vector uncertainties meant the CSM might
; not be exactly where predicted. LRS24.1 systematically searched the uncertainty
; region, driving the radar antenna through the hexagonal pattern until the
; "data good" discrete indicated successful lock-on. Once locked, the radar
; began tracking Columbia continuously, providing the range and angle measurements
; that updated navigation knowledge for the three rendezvous maneuvers.
;
; Radar Boresight Angle Monitoring:
; The routine computes OMEGAD, the angle between the radar boresight (the
; direction the antenna is pointing) and the LM +Z axis. If this angle exceeds
; 30 degrees, the spacecraft's attitude is no longer favorable for radar
; operation, and the preferred tracking attitude routine (R61LEM) is called
; to maneuver the LM to a better orientation. This prevents gimbal limits
; and maintains radar performance.
;
; Search Termination:
; The search continues until either:
; 1. Radar lock-on is achieved (DATAGOOD discrete set)
; 2. The search flag (SRCHOPT) is cleared by crew or program logic
; 3. All search pattern points have been exhausted
;
; Inputs:
;   LM state vector (position, velocity)
;   CSM state vector (position, velocity)
;   REFSMMAT (reference to stable member matrix)
;   SRCHOPT flag (search option flag)
;   NSRCHPNT (search pattern point counter)
;
; Outputs:
;   DATAGOOD: Lock-on status (00000 = no lock, 11111 = locked on)
;   OMEGAD: Angle between radar boresight and LM +Z axis (degrees)
;   Radar commanded to successive search pattern points
;   Possible call to R61LEM if boresight angle exceeds limits
; ============================================================================
;
# PROGRAM NAME LRS24.1		RR SEARCH ROUTINE
# MCD NO. 0			BY P. VOLANTE, SDC		DATE 1-15-67
#
# FUNCTIONAL DESCRIPTION
#
# DRIVES THE RENDEZVOUS RADAR IN A HEXAGONAL SEARCH PATTERN ABOUT THE LOS TO THE CSM (COMPUTED FROM THE CSM AND LM
# STATE VECTORS) CHECKING FOR THE DATA GOOD DISCRETE AND MONITORING THE ANGLE BETWEEN THE RADAR BORESIGHT AND THE
# LM +Z AXIS.  IF THIS ANGLE EXCEEDS 30 DEGREES THE PREFERRED TRACKING ATTITUDE ROUTINE IS CALLED TO PERFORM AN
# ATTITUDE MANEUVER.
#
# CALLING SEQUENCE -- BANKCALL FOR LRS24.1
#
# SUBROUTINES CALLED
#
#	LEMCONIC	R61LEM
#	CSMCONIC	RRDESSM
#	JOBDELAY	FLAGDOWN
#	WAITLIST	FLAGUP
#	RRNB		BANKCALL
#
# EXIT -- TO ENDOFJOB WHEN THE SEARCH FLAG (SRCHOPT) IS NOT SET
#
# OUTPUT
#
#	DATAGOOD (SP) -- FOR DISPLAY IN R1 --	00000 INDICATES NO LOCKON
#						11111 INDICATES LOCKON ACHIEVED
#	OMEGAD (SP)   -- FOR DISPLAY IN R2 --	ANGLE BETWEEN RR BORESIGNT VECTOR AND THE SPACECRAFT +Z AXIS
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	SEARCH FLAG MUST BE SET
#	LM AND CSM STATE VECTORS AND REFSMMAT MATRIX
#
# DEBRIS
#
#	RLMSRCH		UXVECT
#	VXRLM		UYVECT
#	LOSDESRD	NSRCHPNT
#	DATAGOOD	OMEGAD
#	MPAC		PUSHLIST

		COUNT*	$$/LRS24
;
; Entry point for LRS24.1. Initialize search pattern point counter to zero,
; beginning the hexagonal search from the center (predicted LOS).
;
LRS24.1		CAF	ZERO
		TS	NSRCHPNT	# SET SEARCH PATTERN POINT COUNTER TO ZERO
;
; Check search status and enable radar auto-track mode. Issue auto-track enable
; command (BIT14) to rendezvous radar via channel 12. This places the radar
; in automatic tracking mode where it will attempt to lock onto the target.
;
CHKSRCH		CAF	BIT14		# ISSUE AUTO TRACK ENABLE TO RADAR
		EXTEND
# Page 589
		WOR	CHAN12
;
; Verify that search is still requested by checking SRCHOPT flag in FLAGWRD2.
; If the search flag has been cleared (by crew action or program logic),
; terminate the search job.
;
		CAF	SRCHOBIT	# CHECK IF SEARCH STILL REQUESTED
		MASK	FLAGWRD2	# (SRCHOPT FLAG SET)
		EXTEND
		BZF	ENDOFJOB	# NO-TERMINATE JOB

;
; Schedule waitlist task to drive radar to next search pattern point in 6 seconds.
; This gives the radar time to slew to the commanded position and attempt lock-on
; before moving to the next point. CALLDGCH is the task that will execute after
; the 6-second delay.
;
		CAF	6SECONDS	# SCHEDULE TASK TO DRIVE RADAR TO NEXT PT.
		INHINT
		TC	WAITLIST	# IN 6 SECONDS
		EBANK=	LOSCOUNT
		2CADR	CALLDGCH

		RELINT
;
; Check if radar remode (mode change) is currently in progress. REMODBIT in
; RADMODES indicates remode operation. If remode is active, wait for the
; 6-second interval to complete rather than proceeding with LOS computation.
; This prevents commanding the radar during mode transitions.
;
		CS	RADMODES	# IS REMODE IN PROGRESS
		MASK	REMODBIT
		EXTEND
		BZF	ENDOFJOB	# YES -- WAIT SIX SECONDS
;
; Enter interpretive mode to compute line-of-sight (LOS) vector from LM to CSM.
; LOS computation extrapolates both spacecraft state vectors to a common time
; (current time + 1.5 seconds) to account for computational delay and provide
; a slightly ahead prediction for radar pointing.
;
		TC	INTPRET
;
; Load current mission time and add 1.5 seconds to get prediction time.
; This accounts for the time required to complete LOS computation and command
; the radar, ensuring the computed pointing direction matches where the CSM
; will actually be when the radar slews to the commanded position.
;
		RTB	DAD		# COMPUTE LOS AT PRESENT TIME + 1.5 SEC.
			LOADTIME
			1.5SECS
;
; Store prediction time in TDEC1, then call LEMCONIC to extrapolate the LM
; state vector (position and velocity) to the prediction time. LEMCONIC uses
; the current LM state vector and integrates the orbit forward 1.5 seconds.
;
LRS24.11	STCALL	TDEC1
			LEMCONIC	# EXTRAPOLATE LM STATE VECTOR
;
; Save extrapolated LM position vector in RLMSRCH and velocity in SAVLEMV.
; These will be used to compute the relative geometry between spacecraft.
;
		VLOAD
			RATT
		STOVL	RLMSRCH		# SAVE LEM POSITION
			VATT
		STODL	SAVLEMV		# SAVE LEM VELOCITY
			TAT
;
; Store prediction time again in TDEC1, then call CSMCONIC to extrapolate
; the CSM state vector to the same time. This ensures both spacecraft positions
; are computed at the identical instant for accurate LOS determination.
;
		STCALL	TDEC1		# EXTRAPOLATE CSM STATE VECTOR
			CSMCONIC	# EXTRAPOLATE CSM STATE VECTOR
;
; Compute line-of-sight vector: LOS = R(CSM) - R(LM). This is the vector
; from the LM to the CSM in inertial coordinates. The radar will be commanded
; to point along this direction (with search pattern offset applied).
;
		VLOAD	VSU		# LOS VECTOR = R(CSM) - R(LM)
			RATT
			RLMSRCH
;
; Convert LOS vector to unit vector (magnitude = 1) and store as LOSDESRD.
; This is the desired pointing direction for the radar, before any search
; pattern offset is applied. For the center point of the search (N=0),
; the radar will point directly along this vector.
;
		UNIT
		STOVL	LOSDESRD	# STORE DESIRED LOS
;
; Compute the unit vector perpendicular to the CSM orbital plane: UNIT(V(CM) x R(CM)).
; This defines the CSM orbit normal direction and is used to establish the
; coordinate system for the search pattern. The cross product V x R gives
; the angular momentum direction (orbit normal).
;
			VATT		# COMPUTE UNIT(V(CM) CROSS R(CM))
		UNIT	VXV
			RATT
		UNIT
		STORE	VXRCM
;
; Compute relative velocity: V(CSM) - V(LM) in inertial coordinates, then
; transform to stable member (platform) coordinates via REFSMMAT. This gives
; VLC, the closing velocity between spacecraft expressed in the IMU-aligned
; coordinate system. The VSL1 shifts left one bit (multiplies by 2) for
; scaling purposes.
;
		VLOAD	VSU
			VATT
			SAVLEMV
		MXV	VSL1		# CONVERT FROM REFERENCE TO STABLE MEMBER
			REFSMMAT
		STORE	SAVLEMV		# VLC = V(CSM) - V(LM)
;
; Check search pattern point counter NSRCHPNT to determine which point in the
; hexagonal pattern we're commanding:
; - If N=0 (center point): Designate radar directly along LOS with no offset
; - If N=1 (first point of first hexagonal ring): Calculate search pattern
;   coordinate system X and Y axes based on orbital geometry
; - If N>1 (subsequent points): Rotate X-Y axes by 60 degrees to get next point
;
		SLOAD	BZE		# CHECK IF N=0
# Page 590
			NSRCHPNT
			DESGLOS		# YES -- DESIGNATE ALONG LOS
		DSU	BZE		# IS N=1
			ONEOCT		# YES -- CALCULATE X AND Y AXES OF
			CALCXY		# SEARCH PATTERN COORDINATE SYSTEM
;
; For N>1 (subsequent hexagonal search pattern points): Rotate the X-Y coordinate
; system by 60 degrees to position the next point in the hexagonal pattern.
; This implements a 2D rotation transformation using the rotation matrix:
;   UX(new) = cos(60°)·UX(old) + sin(60°)·UY(old)
;   UY(new) = -sin(60°)·UX(old) + cos(60°)·UY(old)
; Each point is separated by 60 degrees, so 6 rotations complete one ring.
;
		VLOAD			# NO -- ROTATE X-Y AXES TO NEXT SEARCH POINT
			UXVECT
		STOVL	UXVECTPR	# SAVE ORIGINAL X AND Y VECTORS
			UYVECT		# UXPRIME = ORIGINAL UX
		STORE	UYVECTPR	# UYPRIME = ORIGINAL UY
;
; Compute new UX = cos(60°)·UXPRIME + sin(60°)·UYPRIME
; First term: sin(60°)·UYPRIME stored temporarily in UXVECT
;
		VXSC
			SIN60DEG	# UX = (COS 60)UXPR + (SIN 60)UYPR
		STOVL	UXVECT
			UXVECTPR
;
; Second term: cos(60°)·UXPRIME added to first term, then normalized
;
		VXSC	VAD
			COS60DEG
			UXVECT
		UNIT
		STOVL	UXVECT
;
; Compute new UY = -sin(60°)·UXPRIME + cos(60°)·UYPRIME
; First term: sin(60°)·UXPRIME stored temporarily in UYVECT
;
			UXVECTPR	# UY = (-SIN 60)UXPR + (COS 60)UYPR
		VXSC
			SIN60DEG
		STOVL	UYVECT
			UYVECTPR
;
; Second term: cos(60°)·UYPRIME, but subtract (VSU) first term to get negative
; sign on the sin term, then normalize the result
;
		VXSC	VSU
			COS60DEG
			UYVECT
		UNIT
		STORE	UYVECT
;
; ============================================================================
; OFFCALC - OFFSET CALCULATION AND RADAR TARGET VECTOR GENERATION
; ============================================================================
;
; Compute the final radar pointing target by combining the base LOS vector
; with an offset in the Y-direction of the search pattern coordinate system.
; Offset Vector = K·UY, where K is the offset factor (distance from center)
; and UY is the Y-axis unit vector of the search pattern coordinate system.
;
; The desired radar pointing direction is:
;   Target = UNIT(LOS + K·UY)
;
; This offset moves the radar aim point away from the direct LOS to search
; surrounding space. As N increases, K increases to expand the search pattern
; to larger rings around the LOS center point.
;
OFFCALC		VXSC	VAD		# OFFSET VECTOR = K(UY)
			OFFSTFAC	# LOS VECTOR + OFFSET VECTOR DEFINES
			LOSDESRD	# DESIRED POINT IN SEARCH PATTERN
;
; Normalize the sum to unit length, then transform from reference inertial
; coordinates to stable member (IMU platform) coordinates via REFSMMAT.
; VSL1 shifts left 1 bit for proper scaling in stable member frame.
;
		UNIT	MXV
			REFSMMAT	# CONVERT TO STABLE MEMBER COORDINATES
		VSL1
;
; Store the final radar target vector in RRTARGET and the closing velocity
; in LOSVEL. These will be used by the radar designate routine to command
; the rendezvous radar gimbal angles and range rate tracking.
;
CONTDESG	STOVL	RRTARGET
			SAVLEMV
		STORE	LOSVEL
;
; Exit interpretive mode to perform radar designation in native AGC code.
;
		EXIT
		INHINT
		TC	KILLTASK	# KILL ANY PRESENTLY WAITLISTED TASK
		CADR	DESLOOP +2	# WHICH WOULD DESIGNATE TO THE LAST
					# POINT IN THE PATTERN
CONTDES2	CS	CDESBIT
		MASK	RADMODES	# SET BIT 15 OF RADMODES TO INDICATE
		AD	CDESBIT		# A CONTINUOUS DESIGNATE WANTED.
		TS	RADMODES
		TC	INTPRET

		CALL
# Page 591
			RRDESSM		# DESIGNATE RADAR TO RRTARGET VECTOR

		EXIT
		TC	LIMALARM	# LOS NOT IN MODE 2 COVERAGE (P22)
		TC	LIMALARM	# VEHICLE MANEUVER REQUIRED (P20)

					# COMPUTE OMEGA,ANGLE BETWEEN RR LOS AND
					# SPACECRAFT +Z AXIS
OMEGCALC	EXTEND
		DCA	CDUT
		DXCH	TANGNB
		TC	INTPRET
		CALL
			RRNB
		DLOAD	ACOS		# OMEGA IS ARCCOSINE OF Z-COMPONENT OF
			36D		# VECTOR COMPUTED BY RRNB (LEFT AT 32D)
		STORE	OMEGDISP	# STORE FOR DISPLAY IN R2
		EXIT
		TC	ENDOFJOB
# Page 592
# CALCULATE X AND Y VECTORS FOR SEARCH PATTERN COORDINATE SYSTEM.
;
; ============================================================================
; CALCXY - CALCULATE SEARCH PATTERN COORDINATE SYSTEM AXES
; ============================================================================
;
; For the first point (N=1) in the hexagonal search pattern, we need to
; establish the X and Y axes that define the search pattern plane perpendicular
; to the LOS (line-of-sight) direction.
;
; The coordinate system is chosen such that:
;   Z-axis = LOS direction (pointing toward CSM)
;   X-axis = perpendicular to LOS in the orbital plane direction
;   Y-axis = perpendicular to both, completing right-handed coordinate system
;
; STEP 1: Compute X-axis (UXVECT)
; The X-axis is computed as: UX = UNIT[(VLM × RLM) × LOS]
; where VLM is LM velocity vector and RLM is LM position vector.
; (VLM × RLM) gives the orbital angular momentum direction, and crossing this
; with the LOS gives a vector in the orbital plane perpendicular to LOS.
;
CALCXY		VLOAD	VXV
			VXRCM
			LOSDESRD
		UNIT
		STOVL	UXVECT		# UX = (VLM X RLM) X LOS
;
; STEP 2: Compute Y-axis (UYVECT)
; The Y-axis is computed as: UY = UNIT(LOS × UX)
; This completes the right-handed orthogonal coordinate system with Z=LOS.
;
			LOSDESRD
		VXV	UNIT
			UXVECT
		STORE	UYVECT		# UY = LOS X UX
;
; Now that we have established the X and Y axes for the search pattern,
; proceed to compute the offset for this first hexagonal point.
;
		GOTO
			OFFCALC
;
; ============================================================================
; DESGLOS - DESIGNATE ALONG LINE OF SIGHT (CENTER POINT)
; ============================================================================
;
; When N=0 (the very first search pattern point), designate the radar directly
; along the line of sight (LOS) toward the CSM with no offset. This is the
; center point of the search pattern and serves as the initial attempt to
; acquire the target before expanding the search to surrounding points.
;
; Transform the LOS vector from reference inertial coordinates to stable
; member (IMU platform) coordinates via REFSMMAT, scale left by 1 bit,
; and proceed to the continuous designate logic.
;
DESGLOS		VLOAD	MXV		# WHEN N= 0,DESIGNATE ALONG LOS
			LOSDESRD
			REFSMMAT	# CONVERT LOS FROM REFERENCE TO SM COORDS
		VSL1	GOTO
			CONTDESG

;
; ============================================================================
; CALLDGCH - SCHEDULE NEXT SEARCH PATTERN POINT
; ============================================================================
;
; Called at the completion of each search pattern designation to schedule
; the next point. Check if the rendezvous flag is still set (meaning we
; are still in rendezvous mode and should continue the search pattern).
; If not, exit the task. If yes, schedule a new job to designate the
; radar to the next point in the search pattern.
;
CALLDGCH	CAE	FLAGWRD0	# IS RENDEZVOUS FLAG SET
		MASK	RNDVZBIT
		EXTEND
		BZF	TASKOVER	# NO -- EXIT R24
;
; Rendezvous mode is still active. Schedule a priority 25 job to continue
; the search pattern by calling DATGDCHK, which checks radar data validity
; and advances to the next search point.
;
		CAF	PRIO25		# YES -- SCHEDULE JOB TO DRIVE RADAR TO NEXT
		TC	FINDVAC		# PONT IN SEARCH PATTERN
		EBANK=	RLMSRCH
		2CADR	DATGDCHK

		TC	TASKOVER
;
; ============================================================================
; DATGDCHK - CHECK RADAR DATA GOOD DISCRETE
; ============================================================================
;
; Verify that the rendezvous radar is providing valid tracking data by
; checking the data good discrete (bit 4) on channel 33. If data is good,
; the radar has achieved lock-on and we can proceed with tracking. If data
; is not good, continue the search pattern to the next point.
;
DATGDCHK	CAF	BIT4
		EXTEND			# CHECK IF DATA GOOD DISCRETE PRESENT
		RAND	CHAN33
		EXTEND
		BZF	STORE1S		# YES -- GO TO STORE 11111 FOR DISPLAY IN R1
;
; Data good discrete is NOT present, meaning the radar has not achieved
; lock-on at this search point. Continue the search pattern to the next
; point. First check if we have completed all 6 points around the circle.
;
		CS	SIX
		AD	NSRCHPNT	# IS N GREATER THAN 6
		EXTEND
		BZF	LRS24.1		# YES -- RESET N = 0 AND START AROUND AGAIN
;
; We have not completed the search circle. Increment the point counter
; and return to the search pattern logic to designate the radar to the
; next point (advancing 60 degrees around the circle from the previous point).
;
		INCR	NSRCHPNT	# NO -- SET N = N+1 AND GO TO
		TCF	CHKSRCH		# NEXT POINT IN PATTERN

;
; ============================================================================
; STORE1S - HANDLE SUCCESSFUL RADAR LOCK-ON
; ============================================================================
;
; The rendezvous radar has achieved lock-on to the Command Module! Store
; the value 11111 in DATAGOOD for display in R1, indicating successful
; acquisition. Terminate the search pattern by killing the designate task
; from the waitlist, as we no longer need to search - we have found and
; locked onto Columbia.
;
; Historical Context: During Apollo 11's rendezvous on July 21, 1969, the
; rendezvous radar acquired Columbia at approximately 100 nautical miles
; range. This lock-on enabled precise relative navigation for the three
; rendezvous maneuvers (CSI, CDH, TPI) that brought Eagle within feet of
; Columbia for docking 3 hours 40 minutes after ascent.
;
STORE1S		CAF	ALL1S		# STORE 11111 FOR DISPLAY IN R1
		TS	DATAGOOD
# Page 593
		INHINT
		TC	KILLTASK	# DELETE DESIGNATE TASK FROM
		CADR	DESLOOP +2	# WAITLIST USING KILLTASK
		TC	ENDOFJOB

;
; ============================================================================
; LIMALARM - LINE-OF-SIGHT COVERAGE ALARM
; ============================================================================
;
; Alarm 527: Line-of-sight to CSM is not within rendezvous radar mode 2
; coverage limits. This occurs when the CSM is outside the radar's field
; of view, either due to incorrect vehicle attitude or excessive range.
; In P22, this indicates the LM needs to perform a vehicle maneuver to
; bring the CSM within radar coverage. In P20, this alarm indicates the
; search pattern cannot proceed due to geometric constraints.
;
; Action: Kill the waitlist task that would schedule the next search
; pattern point, as continued searching is futile until the crew corrects
; the vehicle attitude or the CSM moves into coverage.
;
LIMALARM	TC	ALARM		# ISSUE ALARM 527 -- LOS NOT IN MODE2
		OCT	527		# COVERAGE IN P22 OR VEHICLE MANEUVER
		INHINT			# REQUIRED IN P20
		TC	KILLTASK	# KILL WAITLIST CALL FOR NEXT
		CADR	CALLDGCH	# POINT IN SEARCH PATTERN
		TC	ENDOFJOB

;
; ============================================================================
; RENDEZVOUS RADAR SEARCH PATTERN CONSTANTS
; ============================================================================
;
ALL1S		DEC	11111		# Display code for radar lock-on success
;
; 60-degree rotation constants for hexagonal search pattern:
; Each search point is rotated 60 degrees (π/3 radians) from the previous
; point around the central line-of-sight axis.
;
SIN60DEG	2DEC	.86603		# sin(60°) = √3/2 ≈ 0.86603
COS60DEG	=	DPHALF		# cos(60°) = 0.5 (reference to DPHALF constant)
;
; Vector storage locations for coordinate system computation:
;
UXVECTPR	EQUALS	12D		# Previous U_X unit vector (6 words)
UYVECTPR	EQUALS	18D		# Previous U_Y unit vector (6 words)
RLMUNIT		EQUALS	12D		# LM-to-CSM unit vector (re-use of 12D storage)
;
; Offset factor for search pattern radius:
; Tangent of 3.25 degrees defines the angular offset from the central
; line-of-sight for each search point. This creates a cone of search
; points around the estimated CSM position to account for navigation
; uncertainty in the initial state vector.
;
OFFSTFAC	2DEC	0.05678		# tan(3.25°) ≈ 0.05678
;
; Timing constants for search pattern scheduling:
;
ONEOCT		OCT	00001		# **** NOTE -- THESE TWO CONSTANTS MUST ****
3SECONDS	2DEC	300		# **** BE IN THIS ORDER BECAUSE         ****

					# **** ONEOCT NEEDS A LOWER ORDER 	****
					# **** WORD OF ZEROES			****
6SECONDS	DEC	600		# 600 centiseconds = 6 seconds
1.5SECS		2DEC	150		# 150 centiseconds = 1.5 seconds

ZERO/SP		EQUALS	HI6ZEROS

		BLOCK	02
		SETLOC	FFTAG5
		BANK
		COUNT*	$$/P20
;
; ============================================================================
; GOTOV56 - P20 TERMINATION VIA V56 (TERMINATE TRACKING)
; ============================================================================
;
; Special termination routine for P20. Instead of using the standard GOTOPOOH
; exit, P20 uses verb 56 (V56 = Terminate Tracking) which calls TRMTRACK to
; cleanly shut down the rendezvous radar tracking mode. This ensures proper
; radar system state when exiting rendezvous navigation.
;
; Crew Action: Astronaut terminates P20 by keying V56E on the DSKY.
;
GOTOV56		EXTEND			# P20 TERMINATES BY GOTOV56 INSTEAD OF
		DCA	VB56CADR	# GOTOPOOH
		TCF	SUPDXCHZ
		EBANK=	WHOCARES
VB56CADR	2CADR	TRMTRACK

# Page 594
# PROGRAM NAME: R29 	(RENDEZVOUS RADAR DESIGNATE DURING POWERED FLIGHT)
# MOD NO. 2	BY H. BLAIR-SMITH	JULY 2, 1968
#
# FUNCTIONAL DESCRIPTION:
#
#	DESIGNATES THE RENDEZVOUS RADAR TOWARD THE COMPUTES LOS TO THE CSM, WITH THE CHIEF OBJECTIVE OF OBTAINING RANGE
#	AND RANGE RATE DATA AT 2-SECOND INTERVALS FOR TRANSMISSION TO THE GROUND.  WHEN THE RR IS WITHIN .5 DEGREE OF
#	THE COMPUTED LOS, TRACKING IS ENABLED, AND DESIGNATION CONTINUES UNTIL THE DATA-GOOD DISCRETE IS RECEIVED.  AT
#	THAT POINT, DESIGNATION CEASES AND A RADAR-READING ROUTINE TAKES OVER, PREPARING A CONSISTENT SET OF DATA FOR
#	DOWN TELEMETRY.  THE SET INCLUDES RANGE, RANGE RATE, MARK TIME, TWO RR CDU ANGLES, THREE IMUCDU ANGLES, AND AN
#	INDICATOR WHICH IS 1 WHEN THE SET IS CONSISTENT AND 0 OTHERWISE.  THE INDICATOR IS IN TRKMKCNT.
#
# CALLING SEQUENCE:  BEGUN EVERY 2 SECONDS AS AN INTEGRAL PART OF SERVICER
#
# SUBROUTINES CALLED:
#
#	REMODE		RRPONLY
#	UNIT		MPACVBUF
#	QUICTRIG	AX*SR*T
#	SPSIN		SPCOS
#	SETRRECR	RROUT
#	RRRDOT		RRRANGE
#
# EXIT:  TO NOR29NOW, IN SERVICER.
#
# OUTPUT:  (ALL FOR DOWNLINK)
#
#	RM		RDOTM		(RAW)
#	AIG		AMG
#	AOG		TRKMKCNT	TRKMKCNT = 00001 IF SET IS CONSISTENT,
#	TANGNB		TANGNB +1	OTHERWISE TRKMKCNT = 00000.
#	MKTIME
# Page 595
#
# ERASABLE INITIALIZATION REQUIRED:
#
#	NOR29FLG	READRFLG		(TO 1 AND 0 BY FRESH START) (RESET NOR29FLG TO LET SERVICER RUN R29)
#	PIPTIME		RADMODES (BIT 10)	(BIT SET TO 0 BY FRESH START)
#	R(CSM)		V(CSM)
#	R		V			(PIPTIME THRU V BY AVE G IN SERVICER)
#
# DEBRIS:
#
#	RADMODES (BIT 10)
#	LOSSM		LOSVDT/4		(= RRTARGET & LOSVEL)
#	SAVECDUT	OLDESFLG		(SAVECDUT = MLOSV)
#	LOSCMFLG	READRFLG
#
# ALARMS:  NONE.
#
# COMPONENT JOBS AND TASKS:
#
#	INITIALIZING, IF RR IS FOUND TO BE IN MODE 1:  JOB R29REMOJ AND TASK REMODE:  ALWAYS: TASK PREPOS29.
#	DESIGNATING:  TASK BEGDES29 & JOB R29DODES.
#	RADAR READING:  TASK R29READ AND JOB R29RDJOB.  ALL JOBS ARE NOVAC TYPE.

		BANK	33
		SETLOC	R29/SERV
		BANK

		COUNT*	$$/r29

NR29&RDR	EQUALS	EBANK5

;
; ============================================================================
; TRANSITION: From Rendezvous Tracking (P20-P25) to Powered Flight Designate
; ============================================================================
;
; During powered flight phases (ascent from the lunar surface or major orbit
; maneuvers), the LM cannot continuously track the CSM with the rendezvous
; radar. Instead, R29 performs 2-second "snapshots" - designating the radar
; toward the computed CSM position, waiting for lock-on, reading range and
; range-rate data, then repeating the cycle. This data is critical for ground
; controllers monitoring rendezvous parameters during ascent.
;
; Historical Context: After Eagle's ascent from the lunar surface on July 21,
; 1969, R29 provided periodic range and range-rate measurements to Mission
; Control during the powered ascent phase, allowing ground flight controllers
; to verify that Eagle was on the correct trajectory toward Columbia.
;
; ============================================================================

# Page 596
# SERVICER COMES TO R29 FROM "R29?" IF NOR29FLG, READRFLG, RRREMODE, RRCDUZRO, RRREPOS, AND DISPLAY-INERTIAL-DATA
# ARE ALL RESET, AND THE RR IS IN LGC MODE (OFTEN CONFUSINGLY CALLED AUTO MODE).

;
; ============================================================================
; R29 - RENDEZVOUS RADAR DESIGNATE DURING POWERED FLIGHT
; ============================================================================
;
; Entry Point: Called every 2 seconds by SERVICER during powered flight phases
; when rendezvous radar data is needed for ground telemetry.
;
; Purpose: Designate the rendezvous radar toward the computed line-of-sight
; to the CSM, enable tracking when within 0.5 degrees, acquire lock-on, read
; a consistent set of range/range-rate/angle data, then return to SERVICER.
;
; Cycle Timing: Complete designation-lock-read cycle must finish within
; 2 seconds to maintain periodic data flow to ground controllers.
;
; First, check if designation is already in progress. If the DESIGBIT flag
; is set in RADMODES, we are already in the middle of a designation cycle,
; so skip initialization and go directly to LOS calculation.
;
R29		CS	RADMODES
		MASK	DESIGBIT
		EXTEND
		BZF	R29.LOS		# BRANCH IF DESIGNATION IS ALREADY ON.

;
; This is a new designation cycle. Initialize radar state, disable tracking,
; and prepare for designation. Interrupts are inhibited during this critical
; setup to ensure atomic state changes.
;
		INHINT
		ADS	RADMODES	# SHOW THAT DESIGNATION IS NOW ON.
;
; Remove the RR track enable discrete to prevent self-tracking during the
; high-speed designation phase. Tracking will be enabled later only when
; the radar is within 0.5 degrees of the computed LOS.
;
		CS	BIT14
		EXTEND
		WAND	CHAN12		# REMOVE RR TRACK ENABLE DISCRETE.
;
; Clear flags to show that the continuous designation loop is not active.
; During R29, we use a two-pass fast designation approach rather than the
; continuous loop used in other radar modes.
;
		CS	LOSCMBIT
		MASK	FLAGWRD2
		TS	FLAGWRD2	# CLEAR LOSCMFLG TO SHOW DES. LOOP IS OFF.
		CS	OLDESBIT
		MASK	STATE
		TS	STATE		# SHOW THAT DES. LOOP IS NOT REQUESTED.
;
; Enable the rendezvous radar error counters. These counters integrate the
; difference between commanded and actual antenna angles, driving the radar
; gimbal motors to the desired pointing direction.
;
		TC	BANKCALL
		CADR	SETRRECR	# ENABLE RR ERROR COUNTERS.
;
; Check which radar mode we are in. The RR can be in Mode 1 (automatic
; acquisition with limited gimbal freedom) or Mode 2 (full gimbal freedom).
; Mode 2 is required for R29 operation.
;
		CA	ANTENBIT
		MASK	RADMODES
		CCS	A		# TEST RR MODE BIT.
		TCF	SETPRPOS	# MODE 2.

;
; Radar is in Mode 1. We must force it into Mode 2 before designating.
; Schedule a priority 21 job to perform the remoding operation. This job
; will call RADSTALL to wait for the mode transition to complete.
;
		CA	PRIO21		# MODE 1:  MUST REMODE.
		TC	NOVAC
		EBANK=	LOSCOUNT
		2CADR	R29REM0J	# NEEDS OWN JOB TO RADSTALL IN.

;
; Clear the DESIGBIT flag before remoding (REMODE requires DESIGBIT clear),
; and set the REMODBIT flag to show remoding is in progress.
;
		CS	DESIGBIT
		MASK	RADMODES	# CLEAR DESIGNATE FLAG IN RADMODES
		TS	RADMODES	# BEFORE CALLING REMODE
		CA	REMODBIT
		ADS	RADMODES	# SHOW THAT REMODING IS ON.
		TCF	NOR29NOW	# CONTINUE SERVICER FUNCTIONS.

;
; Radar is in Mode 2. Before designating, we must preposition the trunnion
; angle to -180 degrees. This ensures the radar starts from a known position
; and can designate across the full gimbal range without hitting stops.
; Schedule a task to perform the prepositioning operation.
;
SETPRPOS	CA	ONE
		TC	WAITLIST
		EBANK=	LOSCOUNT
		2CADR	PREPOS29	# TASK TO SET TRUNNION ANGLE TO 180 DEG.

		CA	REPOSBIT
		ADS	RADMODES	# SHOW THAT REPOSITIONING IS ON.
		TCF	NOR29NOW

# Page 597
# FORCE RENDEZVOUS RADAR INTO MODE 2.

;
; ============================================================================
; R29REM0J - JOB TO REMODE RENDEZVOUS RADAR FROM MODE 1 TO MODE 2
; ============================================================================
;
; This job runs when R29 finds the radar in Mode 1 and needs to force it
; into Mode 2 for full gimbal freedom. Mode 2 is required for R29's fast
; designation approach.
;
; Process:
; 1. Schedule REMODE task (must run as a task, not inline)
; 2. Call RADSTALL to wait for remoding to complete
; 3. Exit job when remoding is done
;
; On the next SERVICER cycle, R29 will be called again and will find the
; radar in Mode 2, proceeding to prepositioning and designation.
;
R29REM0J	CA	ONE
		TC	WAITLIST
		EBANK=	LOSCOUNT
		2CADR	REMODE		# REMODE MUST RUN AS A TASK.

;
; RADSTALL suspends this job until the remoding operation completes. The
; radar hardware takes time to mechanically transition between modes.
;
		TC	BANKCALL	# WAIT FOR END OF REMODING
		CADR	RADSTALL

		TCF	ENDOFJOB	# BAD EXIT CAN'T HAPPEN.
		TCF	ENDOFJOB

# TASK TO PREPOSITION THE RR TRUNNION ANGLE TO -180 DEG.

		SETLOC	R29S1
		BANK

;
; ============================================================================
; PREPOS29 - TASK TO PREPOSITION TRUNNION TO -180 DEGREES
; ============================================================================
;
; Before each R29 designation cycle, the trunnion angle is driven to -180
; degrees (straight down relative to the LM). This provides a consistent
; starting position and ensures the radar can designate across the full
; gimbal range without hitting mechanical stops.
;
; Trunnion range: -180 to +180 degrees
; Shaft range: 0 to 360 degrees (continuous rotation)
;
; By starting at -180 degrees trunnion, the radar can designate anywhere
; in the upper hemisphere where the CSM is most likely to be located.
;
PREPOS29	CA	NEGMAX		# -180 DEG.
		TC	RRTONLY		# DRIVE TRUNNION CDU.
;
; Clear the REPOSBIT flag to show that prepositioning is complete. The
; next SERVICER cycle will proceed to LOS calculation and designation.
;
		CS	REPOSBIT	# SHOW THAT REPOSITIONING IS OFF.
		MASK	RADMODES
		TS	RADMODES
		TCF	TASKOVER

# COMPUTE THE LINE-OF-SIGHT AND LOS VELOCITY, AND PASS THEM TO THE R29DODES LOOP.

		SETLOC	R29
		BANK

;
; ============================================================================
; R29.LOS - COMPUTE LINE-OF-SIGHT VECTOR TO CSM
; ============================================================================
;
; This entry point is used when designation is already in progress or when
; remoding/prepositioning is complete. It computes the up-to-date LOS vector
; from the LM to the CSM using the current state vectors and extrapolating
; forward from the last IMU update time (PIPTIME).
;
; Calculation:
; LOS = [R(CSM) + V(CSM)*(T-PIPTIME)] - [R(LM) + V(LM)*(T-PIPTIME)]
;
; Where:
; R(CSM), V(CSM) = CSM position and velocity state vectors (stable member axes)
; R(LM), V(LM)   = LM position and velocity state vectors (stable member axes)
; T              = Current time (TIME2)
; PIPTIME        = Time of last IMU update
;
; This extrapolation compensates for the time delay since the last IMU update,
; ensuring the radar designates toward the current CSM position rather than
; an outdated position.
;
R29.LOS		EXTEND
		DCS	PIPTIME
		DXCH	MPAC
		EXTEND
		DCA	TIME2
		DAS	MPAC		# (MPAC) = T-PIPTIME, SCALED B-28.
;
; Scale the time difference to B-17 for use with velocity vectors.
; Velocity vectors are scaled in meters/centisecond, so this time scaling
; allows us to compute position change over the extrapolation interval.
;
		TS	MODE		# SET MODE TO DOUBLE PRECISION.
		CA	MPAC +1
		EXTEND
		MP	BIT12
		DXCH	MPAC		# T-PIPTIME NOW SCALED B-17.
		TC	INTPRET
# Page 598
# LOSCMFLG = 0 MEANS THAT THE DESIGNATION IS READY FOR NEW DATA.  SETTING LOSCMFLG MAKES IT GO AWAY SO SETUP29D CAN
# START IT UP WHEN THE DATA IS IN PLACE.

;
; Now in interpretive mode. Compute the extrapolated LOS vector.
;
; Step 1: Compute LOS velocity (rate of change of LOS vector)
;         LOSVEL = V(CSM) - V(LM)
;
; Step 2: Extrapolate positions forward by (T-PIPTIME):
;         R_extrap(CSM) = R(CSM) + V(CSM) * (T-PIPTIME)
;         R_extrap(LM)  = R(LM)  + V(LM)  * (T-PIPTIME)
;
; Step 3: Compute LOS vector:
;         LOSSM = R_extrap(CSM) - R_extrap(LM)
;
		PDVL	VSU		# PUSH DOWN T-PIPTIME
			V(CSM)
			V		# LOSVEL = V(CSM) - V
		PDDL	VXSC		# SWAP LOSVEL FOR T-PIPTIME, MULTIPLY THEM
		VAD	VSU		# 	AND ADD THE RESULT TO R(CSM) - R TO GET
			R(CSM)		# 	AN UP-TO-DATE LOS VECTOR IN SM AXES.
			R
;
; Check if the designation loop is currently running. If LOSCMFLG is clear
; (designation loop is off), set the flag and go set up new data. If the
; flag is set (designation loop is already running), skip data setup and
; let the loop continue using the old LOS data to avoid interrupting an
; active designation cycle mid-stream.
;
		BOFSET	EXIT		# (BOFSET DOES ITS THING INHINTED.)
			LOSCMFLG	# IF DESIGNATE LOOP IS OFF, CHANGE LOSCM-
			SETUP29D	# FLG TO ON AND GO TO SET UP NEW DATA.
		TCF	NOR29NOW	# IF DES. LOOP IS ON, LET IT USE OLD DATA.

;
; ============================================================================
; SETUP29D - STORE LOS DATA AND INITIATE DESIGNATION LOOP
; ============================================================================
;
; This section (still in interpretive mode) stores the computed LOS vector
; in LOSSM, calculates LOSVDT/4 (half-second's worth of LOS velocity for
; extrapolation), clears LOSCMFLG to signal that new data is ready, then
; exits interpretive mode to check if the designation loop needs to be started.
;
; The designation loop machinery consists of:
; - BEGDES29 task: Scheduled to run ASAP, spawns R29DODES job
; - R29DODES job: Performs one designation pass (compute, slew, check)
; - R29DLOOP task: Reschedules itself every 0.5 seconds to maintain loop
;
SETUP29D	STOVL	LOSSM		# LINE-OF-SIGHT VECTOR, STABLE MEMBER AXES
			0
;
; Calculate half-second's worth of LOS velocity: LOSVDT/4 = LOSVEL * 0.5 sec
; This is used by the designation loop to extrapolate the LOS vector during
; the ~0.5 second designation cycle.
;
		VXSC
			.5SECB17
		STORE	LOSVDT/4	# 1/2 SECOND'S WORTH OF LOS VELOCITY.
;
; Clear LOSCMFLG to signal R29DLOOP that new data is ready and can be used.
; This flag acts as a lock: when set, R29DLOOP waits for data; when clear,
; R29DLOOP proceeds with designation using the current LOSSM and LOSVDT/4.
;
		CLEAR	EXIT
			LOSCMFLG	# LET R29DLOOP USE NEW DATA.

;
; Back in native AGC code. Check if the R29 designation loop is already
; running. The OLDESBIT flag in STATE indicates whether the loop has been
; requested and is active.
;
		CS	STATE
		MASK	OLDESBIT
		EXTEND
		BZF	NOR29NOW	# BRANCH IF R29 DES. LOOP IS REQUESTED.
;
; If we reach here, the designation loop is not running. Start it now.
; Set OLDESBIT in STATE to indicate the loop is active.
;
		INHINT
		ADS	STATE		# OTHERWISE REQUEST IT NOW.

;
; Schedule BEGDES29 task to run soon. Check PIPCTR to see if we should
; offset the timing by one second (if PIPCTR is non-zero, we're in the
; middle of a PIPA reading cycle, so wait 4 centiseconds; otherwise 100).
;
		CCS	PIPCTR		# SEE IF TASK SHOULD BE OFFSET ONE SECOND.
		CS	SUPER110	# -96D +100D = 4.
		AD	1SEC		# 0 +100D = 100D.
		TC	WAITLIST
		EBANK=	LOSCOUNT
		2CADR	BEGDES29	# START BEGDES29 TASK ASAP.

		TCF	NOR29NOW	# RELINT AND CONTINUE SERVICER FUNCTIONS.

.5SECB17	2DEC	50 B-17

# Page 599
# R29 DESIGNATE JOB AND TASK MACHINERY.  TASK RECURS EVERY .5 SEC UNTIL DESIGNATE IS CALLED OFF; IT MAY WAIT FOR A
# CENTISECOND OR TWO IF IT COMES UP WHILE SETUP29D IS SUPPLYING NEW DATA.

		BANK	24
		SETLOC	P20S
		BANK

		COUNT*	$$/R29

; ============================================================================
; R29 DESIGNATION LOOP MACHINERY
;
; BEGDES29: Initiates R29DODES job at PRIO21 to perform radar designation.
;           Called by SETUP29D initially and by R29DLOOP every 0.5 seconds.
;
; R29DLOOP: Recurring task that maintains designation loop. Runs every 0.5
;           seconds, checks if designation is still active, synchronizes with
;           SETUP29D data updates, then restarts BEGDES29.
;
; Historical Context: During Apollo 11's rendezvous on July 21, this loop ran
; continuously after Eagle's ascent, designating the rendezvous radar toward
; Columbia's predicted position. Loop ran at 2 Hz (twice per second) until
; lock-on was achieved at approximately 100 miles range, then transitioned to
; R29READ for data collection.
; ============================================================================

BEGDES29	CAF	PRIO21
		TC	NOVAC		; Start high-priority job
		EBANK=	LOSVDT/4
		2CADR	R29DODES	; R29DODES job executes twice per second

; R29DLOOP maintains the 0.5-second designation cycle. It coordinates with
; SETUP29D (which updates LOS vector data) to ensure designation continues
; smoothly until lock-on is achieved.

R29DLOOP	CAF	.5SEC		; Delay 0.5 seconds (50 centiseconds)
		TC	VARDELAY

		CS	RADMODES	; Check if designation is still active
		MASK	DESIGBIT
		CCS	A
		TCF	TASKOVER	; Designation disabled - exit loop

		CS	FLAGWRD2	; Check LOSCMFLG (LOS compute in progress)
		MASK	LOSCMBIT
		EXTEND
		BZF	+3		; SETUP29D is updating data - wait briefly
		ADS	FLAGWRD2	; Set LOSCMFLG to show designation loop active
		TCF	BEGDES29	; Data ready - restart designation job

		CA	ONE		; SETUP29D busy - wait 1 centisecond
		TCF	R29DLOOP +1	; Then check again for new data
# Page 600
# R29DODES:  RR DESIGNATION LOOP FOR R29
#
# THIS ROUTINE DOES MUCH THE SAME THING AS DODES, BUT A GREAT DEAL FASTER.  IT TAKES THE NON-UNITIZED LOS VECTOR
# IN STABLE MEMBER COORDINATES (LOSSM) AND A DELTA-LOS IN SM AXES (LOSVDT/4) WHICH IS 1/2 SEC TIMES LOS VELOCITY,
# AND DEVELOPS THE SHAFT AND TRUNNION COMMANDS USING SINGLE PRECISION AS MUCH AS POSSIBLE, AND INTERPRETIVE NOT AT
# ALL.  THE UNIT(LOSM + LOSVEL * 1 SEC) IS COMPUTED IN DP AND TRANSFORMED TO NAV BASE COORDINATES IN DOUBLE PRE-
# CISION (USING SP SINES AND COSINES OF CDU ANGLES), AND THE REST IS DONE IN SP.
#
# THE FUNCTIONAL DIFFERENCE IS THAT R29DODES ALWAYS CLEARS LOSCMFLG WHEN IT ENDS, AND IT STARTS UP THE R29READ
# TASK WHEN LOCK-ON IS ACHIEVED.

		BANK	32
		SETLOC	F2DPS*32
		BANK

		COUNT*	$$/R29
		EBANK=	LOSVDT/4

; ============================================================================
; R29DODES: High-speed radar designation for R29 tracking
;
; This routine performs rapid radar designation computations (2 Hz rate) to
; continuously point the rendezvous radar at the target spacecraft. It uses
; single-precision arithmetic extensively for speed, avoiding the interpretive
; language overhead used in the slower DODES routine.
;
; Algorithm: Takes non-unit LOS vector in stable member coordinates (LOSSM)
; and LOS velocity delta (LOSVDT/4 = 0.5 sec * LOS velocity), advances the
; LOS vector by 1 second (two 0.5-sec updates), unitizes in double precision,
; transforms to navigation base coordinates, then computes radar shaft and
; trunnion angle commands in single precision.
;
; Upon achieving lock-on (DGOOD flag set), R29DODES disables designation mode
; and initiates R29READ task to begin collecting range and range-rate data for
; navigation state updates.
; ============================================================================

R29DODES	CA	ONE		; Initialize for first pass through loop
		TS	TANG		; TANG=1 indicates 1st pass
		CA	FIVE		; Loop counter (counts 5,3,1 for 3 vector components)

; Vector loop processes X, Y, Z components of LOS vector. On first pass (TANG=1),
; only loads LOSSM. On subsequent passes, advances LOSSM by velocity delta.

R29DVBEG	CCS	A		; Countdown by 2's: 5→3→1
		TS	Q		; Q holds component index (5,3,1)
		CCS	TANG		; Check if first pass
		TCF	R29DPAS1	; First pass: skip velocity advance

		EXTEND			; Subsequent passes: advance LOS by velocity
		INDEX	Q		; Index by component (X, Y, or Z)
		DCA	LOSVDT/4	; Load velocity delta (0.5 sec worth)
		INDEX	Q
		DAS	LOSSM		; Add to LOS vector (advances 0.5 seconds)

R29DPAS1	EXTEND			; Load LOS component (first or advanced)
		INDEX	Q
		DCA	LOSSM
		INDEX	Q		# MOVE CURRENT LOS (1ST PASS) OR LOS PRO-
		DXCH	MPAC +1		# JECTED 1/2 SEC AHEAD (2ND PASS).
		CCS	TANG		; Check pass indicator
		TCF	R29DVEND	; First pass: no further advance needed

		EXTEND			; Second pass: advance another 0.5 seconds
		INDEX	Q
		DCA	LOSVDT/4	; Velocity delta (0.5 sec worth)
		INDEX	Q
		DAS	MPAC +1		; Total projection: 1.0 second ahead

R29DVEND	CCS	Q		; Check component counter
		TCF	R29DVBEG	; Continue for next component (X→Y→Z)

# Page 601
; ============================================================================
; UNITIZE AND TRANSFORM LOS TO NAVIGATION BASE AXES
;
; After the vector loop completes, MPAC+1 contains either:
;   - First pass:  Current LOS vector (non-unit)
;   - Second pass: LOS vector projected 1.0 second ahead (non-unit)
;
; This section unitizes the LOS vector in double precision using the UNIT
; subroutine, then transforms it from stable member coordinates to navigation
; base coordinates using CDU angles (IMU gimbal angles). CDU angles sampled
; during first pass ensure consistent transformation even as spacecraft rotates.
; ============================================================================
# UNITIZE AND TRANSFORM TO NAV BASE AXES THE PRESENT LOS (1ST PASS) OR THE 1-SEC PROJECTED LOS (2ND PASS).

		DXCH	MPAC +1		; Move LOS vector to MPAC for unitization
		DXCH	MPAC
		CA	R29FXLOC	; Setup FIXLOC pointer for UNIT routine
		TS	FIXLOC		; Points to workspace for length, length²
		TC	USPRCADR	; Call UNIT subroutine (unitizes vector)
		CADR	UNIT		; Returns unit vector in MPAC
		TC	MPACVBUF	; Move unit(LOS) to AX*SR*T argument area

; Sample CDU angles on first pass only. Reuse same angles on second pass
; to maintain consistent coordinate frame during the 1-second projection.

		CCS	TANG		; Check pass indicator
		TCF	+2		; First pass: sample CDU angles
		TCF	GOTANGLS	; Second pass: skip to transformation
		INHINT			; Prevent CDU angle changes mid-sampling
		EXTEND
		DCA	CDUT		; Load trunnion and shaft CDU angles
		DXCH	SAVECDUT	; Save for radar commands later
		CA	CDUY		; Load Y CDU (middle gimbal)
		TS	CDUSPOT
		CA	CDUZ		; Load Z CDU (outer gimbal)
		TS	CDUSPOT +2
		CA	CDUX		; Load X CDU (inner gimbal)
		TS	CDUSPOT +4	; Stored in Y-Z-X order for AX*SR*T routine
		TC	BANKCALL
		CADR	QUICTRIG	; Compute sines and cosines of all CDU angles

; Transform unit LOS from stable member coordinates to navigation base axes.
; AX*SR*T applies the rotation matrix using CDU angles to yield ULOSNB.

GOTANGLS	CS	THREE		; Set up for 3-component vector
		TC	BANKCALL
		CADR	AX*SR*T		; ULOSNB = rotation(CDUX,Y,Z) * ULOSSM

		CCS	TANG		; Check pass indicator
		TCF	+2		; First pass: continue to angle check
		TCF	R29DPAS2	; Second pass: skip angle check, compute RR commands

# Page 602
; ============================================================================
; FIRST PASS ONLY: COMPUTE ALIGNMENT ANGLE BETWEEN LOS AND RR BORESIGHT
;
; This section computes the cosine of the angle between the current LOS
; direction and the rendezvous radar boresight axis. If the angle is less
; than approximately 0.5 degrees (cos θ ≥ cos(0.5°) ≈ 0.99996), the LOS is
; well-aligned with the radar boresight and designation mode can safely
; transition to self-tracking mode (lock-on).
;
; RR boresight vector in navigation base coordinates:
;   [sin(S)·cos(T), -sin(T), cos(S)·cos(T)]
; where S = shaft angle, T = trunnion angle
;
; Dot product with ULOSNB gives cos(angle).
; ============================================================================
# COMPUTE COSINE OF THE ANGLE BETWEEN THE PRESENT LOS AND THE RR BORESIGHT VECTOR, AND SET THE SELFTRACK ENABLE IF
# THE COSINE IS APPROXIMATELY COS(.5 DEG) OR GREATER (I.E., SMALLER ANGLE).

		INHINT			; Prevent interrupts during calculation
		TS	TANG		; Clear TANG (indicates 2nd pass next time)
		CA	SAVECDUT	; Load trunnion angle (T)
		TC	SPCOS
		TS	PUSHLOC		; PUSHLOC = cos(T)
		CS	SAVECDUT	; Load negative trunnion
		TC	SPSIN
		TS	MODE		; MODE = -sin(T)
		EXTEND
		MP	VBUF +2		; Multiply by ULOSNB_Y component
		DXCH	MPAC		; MPAC = -sin(T) · ULOSNB_Y
		CA	SAVECDUT +1	; Load shaft angle (S)
		TC	SPSIN
		TS	SAVECDUT	; Save sin(S) for later
		EXTEND
		MP	PUSHLOC		; sin(S) · cos(T)
		EXTEND
		MP	VBUF		; Multiply by ULOSNB_X component
		DAS	MPAC		; Accumulate: -sin(T)·Y + sin(S)·cos(T)·X
		CA	SAVECDUT +1	; Load shaft angle again
		TC	SPCOS
		TS	SAVECDUT +1	; SAVECDUT+1 = cos(S)
		EXTEND
		MP	PUSHLOC		; cos(S) · cos(T)
		EXTEND
		MP	VBUF +4		; Multiply by ULOSNB_Z component
		DAS	MPAC		; Complete dot product:
					; cos(angle) = sin(S)·cos(T)·X - sin(T)·Y + cos(S)·cos(T)·Z

; Note: ULOSNB stored in VBUF as half-unit vector (magnitude 0.5), so the
; dot product yields 0.5·cos(angle). Doubling gives true cos(angle).

		EXTEND
		DCA	MPAC		; Load computed cosine
TESTCOS		DAS	MPAC		; Double to get full cos(angle)
		CCS	A		; Test for positive overflow
		CA	BIT14		; Overflow: angle < ~0.5° - enable tracking
		NOOP			; Zero or negative: alignment insufficient
		EXTEND
		WOR	CHAN12		; Set bit 14: enable RR self-track mode
		RELINT			; Re-enable interrupts
		TCF	R29DVBEG -1	; Make 2nd pass (compute RR commands)

# Page 603
; ============================================================================
; SECOND PASS: COMPUTE RR SHAFT AND TRUNNION COMMANDS
;
; This section computes the radar gimbal drive commands needed to align the
; radar boresight with the 1-second-ahead projected LOS. Commands computed
; such that the boresight will null the error in 0.5 seconds (half the
; projection interval), providing smooth continuous tracking.
;
; Shaft command (rotation about LM Z-axis):
;   Raw shaft = ULOSNB' · [cos(S), 0, -sin(S)]
; Trunnion command (rotation about shaft axis):
;   Raw trunnion = ULOSNB' · [sin(S)·sin(T), cos(T), cos(S)·sin(T)]
;
; Both commands scaled by gain factor -0.53624 to achieve 0.5-second nulling.
; ============================================================================
# COMPUTE SHAFT AND TRUNNION COMMANDS TO NULL HAVE THE ERROR IN HALF A SECOND.

R29DPAS2	CA	SAVECDUT +1	; Load cos(S)
		EXTEND
		MP	VBUF		; cos(S) · ULOSNB'_X
		DXCH	TANG		; Save partial result
		CS	SAVECDUT	; Load -sin(S)
		EXTEND
		MP	VBUF +4		; -sin(S) · ULOSNB'_Z
		DAS	TANG		; Raw shaft = cos(S)·X - sin(S)·Z
		CS	MODE		; Load sin(T) (MODE was -sin(T))
		EXTEND
		MP	SAVECDUT	; sin(T) · sin(S)
		EXTEND
		MP	VBUF		; sin(T)·sin(S) · ULOSNB'_X
		DXCH	MPAC		; Save partial result
		CA	PUSHLOC		; Load cos(T)
		EXTEND
		MP	VBUF +2		; cos(T) · ULOSNB'_Y
		DAS	MPAC		; Accumulate
		CS	MODE		; Load sin(T) again
		EXTEND
		MP	SAVECDUT +1	; sin(T) · cos(S)
		EXTEND
		MP	VBUF +4		; sin(T)·cos(S) · ULOSNB'_Z
		DAS	MPAC		; Raw trunnion complete
		CA	MPAC		; Load trunnion command
		EXTEND
		MP	RR29GAIN	; Apply gain: -0.53624 (≈ -1/(2·0.93))
		XCH	TANG		; Store trunnion cmd, get raw shaft cmd
		EXTEND
		MP	RR29GAIN	; Apply same gain to shaft command
		TS	TANG +1		; Store refined shaft command

# Page 604
; ============================================================================
; CHECK RR DATA-GOOD AND TRANSITION TO LOCK-ON MODE
;
; After computing and sending gimbal commands, check if the rendezvous radar
; has achieved lock-on (data-good flag set in channel 33, bit 4). If lock-on
; confirmed, transition from designation mode to data-reading mode, where
; range, range-rate, and angle measurements are continuously processed.
;
; HISTORICAL: During Apollo 11's rendezvous, the radar locked onto Columbia
; at approximately 100 nautical miles range after several minutes of
; designation. Once lock-on occurred, continuous tracking data flowed to the
; navigation filter, enabling precise relative state estimation for the
; three rendezvous maneuvers (CSI, CDH, TPI).
; ============================================================================
# WHETHER OR NOT TRACKING WAS ENABLED THIS TIME, CHECK ON RR DATA-GOOD.  IF PRESENT, STOP DESIGNATING AND START
# READING DATA FROM THE RENDEZVOUS RADAR.

DGOOD?		CAF	BIT4		; Mask for RR data-good bit
		EXTEND
		RAND	CHAN33		; Read channel 33, isolate bit 4
		INHINT			; Prevent interrupts during transition
		EXTEND
		BZF	R29LOKON	; Branch if data-good present (bit set)

; Data-good not yet present: continue designation mode

		TC	BANKCALL
		CADR	RROUT		; Send computed shaft/trunnion commands
		TCF	END29DOD	; Exit and reschedule R29DODES

; Data-good confirmed: transition to lock-on (data-reading) mode

R29LOKON	CS	DESIGBIT	; Clear designation flag
		MASK	RADMODES
		TS	RADMODES	; Update mode: designation complete
		CS	BIT2		; Prepare to disable error counters
		EXTEND
		WAND	CHAN12		; Clear bit 2: disable RR CDU error counters
		CA	READRBIT	; Set read-radar flag
		ADS	FLAGWRD3	; Request data-reading job to start
; Schedule first data-reading task. Offset by 1 second if pulsed integrating
; pendulous accelerometer (PIPA) reading is in progress to avoid conflict.

		CCS	PIPCTR		; Check if PIPA counter active
		CS	SUPER110	; Active: offset task by 4 cs (0.04 sec)
		AD	1SEC		; Inactive: schedule at 100 cs (1.0 sec)
		TC	WAITLIST	; Add task to timer queue
		EBANK=	LOSCOUNT
		2CADR	R29READ		; Start R29READ reading task and job

; Common exit point for R29DODES job

END29DOD	CS	LOSCMBIT	; Clear LOS computation flag
		MASK	FLAGWRD2
		TS	FLAGWRD2	; Update flag word
		TCF	ENDOFJOB	; Terminate R29DODES job

; Constants and equates for R29DODES routine

R29FXLOC	ADRES	INTB15+ -34D	; FIXLOC pointer for UNIT subroutine
RR29GAIN	DEC	-.53624		; Gimbal command gain: -0.53624
LOSVDT/4	EQUALS	LOSVEL		; LOS velocity / 4 (0.5-sec increment)
LOSSM		EQUALS	RRTARGET	; LOS in stable member coordinates
SAVECDUT	EQUALS 	MLOSV		; Saved CDU trunnion/shaft angles

# Page 605
; ============================================================================
; R29READ - RENDEZVOUS RADAR DATA READING TASK
;
; Periodic task initiated by R29DODES upon radar lock-on. Schedules the
; R29RDJOB job every 2 seconds to read RR range, range-rate, and angles,
; then packages the measurements for downlink telemetry and navigation
; processing. Continues until tracking is disabled or radar loses lock.
;
; CALLED BY: WAITLIST (scheduled by R29DODES upon lock-on)
; INTERVAL: 2-second periodic task
;
; COMMENT-ONLY READERS: Once the rendezvous radar achieves lock-on, this
; routine begins continuous measurement cycles. Every 2 seconds, the computer
; reads range and range-rate, captures gimbal angles, and formats the data
; for transmission to Mission Control and for navigation updates.
;
; CODE-ALONG READERS: Task/job architecture: R29READ (WAITLIST task)
; schedules R29RDJOB (NOVAC job) to perform actual reading, then reschedules
; itself after 2-second delay. Self-terminating if READRFLG cleared.
; ============================================================================
# RR READING IS SET UP BY R29DODES WHEN IT DETECTS RR LOCK-ON

		BANK	24
		SETLOC	P20S
		BANK

		COUNT*	$$/R29

		EBANK=	LOSCOUNT

R29READ		CAF	PRIO26		; Priority 26 for reading job
		TC	NOVAC		; Schedule job via executive
		EBANK=	LOSCOUNT
		2CADR	R29RDJOB	; Start R29RDJOB to read and downlink

; Reschedule this task to run again in 2 seconds, but only if reading
; is still enabled (radar has not failed or been disabled by crew).

		CA	2SECS		; 2-second delay (200 centiseconds)
		TC	VARDELAY	; Add this task to WAITLIST again

; 2 seconds later, task resumes here. Check if reading should continue.

		CA	FLAGWRD3	; Get flag word 3
		MASK	READRBIT	; Isolate READRFLG (read-radar flag)
		CCS	A		; Test flag state
		TCF	R29READ		; Flag set: continue reading (loop)
		TCF	TASKOVER	; Flag clear: stop, wait for redesignate

; R29RDJOB: Job to read radar data and format for downlink
; Performs validity checks, reads range-rate and range, captures angles,
; packages data for telemetry.

R29RDJOB	CA	FLAGWRD3	; NOVAC job entry point
		MASK	NR29FBIT	; Check NOR29FLG (R29 termination flag)
		CCS	A		; Is R29 terminating?
		TCF	ENDRRD29	; Yes: stop reading immediately

; Check if radar still in automatic tracking mode

		CA	RADMODES	; Get radar mode flags
		MASK	AUTOMBIT	; Isolate auto-mode bit
		CCS	A		; Is bit set (NOT in auto)?
		TCF	ENDRRD29	; Yes: crew took manual control, stop

; Initiate range-rate reading. RRRDOT starts radar measurement, RADSTALL
; suspends job until data ready (typically ~100 milliseconds).

		TC	BANKCALL
		CADR	RRRDOT		; Initiate range-rate (closing velocity) read
		TC	BANKCALL
		CADR	RADSTALL	; Sleep until measurement complete
		TCF	ENDRRD29	; Return address: bad read, exit

# Page 606
; ============================================================================
; R29 RADAR READING CONTINUED - CAPTURE ANGLES AND TIME
;
; Range-rate reading succeeded. Now capture time tag and CDU angles for
; telemetry downlink. These angles enable ground reconstruction of radar
; measurements in inertial frame for navigation analysis.
;
; MPAC +0,+1: Time of reading (TIMEHOLD)
; MPAC +2,+3: RR CDU trunnion and shaft angles
; MPAC +4,+5: IMU CDU Y and Z angles
; MPAC +6:    IMU CDU X angle
; (Buffered data copied to downlink area after range reading completes)
; ============================================================================
# R29 RADAR READING CONTINUED.

		DXCH	TIMEHOLD	; Get mission elapsed time of reading
		DXCH	MPAC		; Store in MPAC buffer for downlink
		INHINT			; Prevent interrupts during angle capture
		EXTEND			; (Ensures 5 CDU angles are consistent)
		DCA	CDUT		; Get RR CDU trunnion and shaft angles
		DXCH	MPAC +2		; Store RR gimbal angles for downlink
		EXTEND
		DCA	CDUY		; Get IMU CDU Y and Z gimbal angles
		DXCH	MPAC +4		; Store IMU angles for downlink
		CA	CDUX		; Get IMU CDU X gimbal angle
		TS	MPAC +6		; Complete 7-word downlink buffer

; Initiate range reading, then copy complete measurement package to downlink
; area for telemetry transmission to Mission Control.

R29RANGE	TC	BANKCALL
		CADR	RRRANGE		; Initiate RR range (distance) reading
		TC	BANKCALL
		CADR	RADSTALL	; Sleep until measurement complete
		TCF	R29RRR?		; Return: bad read or scale change?

; Range reading succeeded. Copy MPAC buffer and range data to downlink area.
; This atomic operation ensures telemetry contains consistent measurement set.

		INHINT			; Prevent interrupts during copy cycle
		DXCH	DNRRANGE	; Range and range-rate → RM (downlink)
		DXCH	RM		; (DNRRANGE = double-word range)
		DXCH	MPAC		; Time tag → MKTIME (mark time)
		DXCH	MKTIME
		DXCH	MPAC +2		; RR CDU angles → TANGNB (tangent NB)
		DXCH	TANGNB
		DXCH	MPAC +4		; IMU CDU Y,Z angles → AIG
		DXCH	AIG
		CA	MPAC +6		; IMU CDU X angle → AOG
		TS	AOG
		CA	ONE		; Set counter to 1
		TS	TRKMKCNT	; Indicate valid tracking mark in downlink
		TCF	ENDOFJOB	; Exit R29RDJOB, await next R29READ call

; Range reading failed. Determine if failure was bad read (requires
; redesignation) or scale change (requires retry).

R29RRR?		CS	FLAGWRD5	; Get flag word 5 complement
		MASK	BIT10		; Isolate RNGSCFLG (range scale change)
		CCS	A		; Was it a scale change?
		TCF	ENDRRD29	; No: genuine bad read, exit to redesignate
		TC	DOWNFLAG	; Yes: clear scale change flag
		ADRES	RNGSCFLG
		TCF	R29RANGE	; Retry range reading immediately

; Exit reading mode due to failure or mode change. Clear flags and disable
; radar track-enable discrete.

ENDRRD29	CA	ZERO		; Load zero
		TS	TRKMKCNT	; Clear tracking mark counter
					; (Invalidates downlink telemetry data)
		TC	DOWNFLAG	; Clear read-radar flag
		ADRES	READRFLG	; (Stops R29READ periodic task)
		CS	BIT14		; Prepare to clear track-enable
		EXTEND
# Page 607
		WAND	CHAN12		; Clear bit 14: disable RR track mode
		TCF	ENDOFJOB	; Exit, await redesignation by R29DODES

# Page 608
# W-MATRIX MONITOR

; ============================================================================
; SECTION: V67 W-MATRIX MONITOR (NAVIGATION UNCERTAINTY DISPLAY)
;
; This extended verb displays navigation state uncertainty derived from the
; W-matrix (state covariance matrix). W is an 18x18 symmetric matrix tracking
; position, velocity, and IMU bias uncertainties through Kalman filtering.
;
; COMMENT-ONLY READERS: This display allowed the crew to monitor navigation
; confidence. Small uncertainty values indicate high navigation accuracy from
; recent radar marks or ground tracking. Large values indicate need for
; additional measurement updates.
;
; CODE-ALONG READERS: V67 computes RMS (root-mean-square) uncertainties by
; extracting diagonal elements from W-matrix, summing squares for position
; (3 components), velocity (3 components), and bias (3 components), taking
; square roots, and displaying via Noun 99. Values compared before/after
; display to detect crew updates via V67E.
; ============================================================================

		BANK	31
		SETLOC	VB67
		BANK
		COUNT*	$$/EXTVB

		EBANK=	WWPOS

; Entry point for V67 extended verb. Compute current W-matrix uncertainties,
; save for comparison, display on DSKY.

V67CALL		TC	INTPRET		; Enter interpretive mode
		CALL			; Compute RMS uncertainties from W
			V67WW		; Returns WWPOS, WWVEL, WWBIAS
		EXIT			; Return to native AGC code
		EXTEND			; Save present Noun 99 values for
		DCA	WWPOS		; comparison after DSKY display
		DXCH	WWBIAS +2	; (Detects if crew entered new values)
		EXTEND
		DCA	WWVEL		; Copy velocity uncertainty
		DXCH	WWBIAS +4
		EXTEND
		DCA	WWBIAS		; Copy bias uncertainty
		DXCH	WWBIAS +6	; WWBIAS+2 thru +7 = saved values
; Display RMS uncertainties using V06 N99 (display decimal with load option).
; Crew can accept displayed values (PROCEED) or enter new values (ENTER).
; Recalculation loop detects changes and sets V67FLAG if update entered.

V06N99DS	CAF	V06N99		; Load verb 06, noun 99 code
		TC	BANKCALL	; Display on DSKY
		CADR	GOXDSPF		; (Waits for crew response)
		TCF	ENDEXT		; TERMINATE: V34E or similar, exit
		TCF	V6N99PRO	; PROCEED: crew accepted, process
		TCF	V06N99DS	; RECYCLE: redisplay (shouldn't occur)

; Crew pressed PROCEED. Check if any displayed values changed (crew data entry
; via ENTER before PROCEED). Sum differences between saved and current values.

V6N99PRO	ZL			; Clear L register (accumulator)
		CA	FIVE		; Load 5 (loop through 6 words: 0-5)
N99LOOP		TS	Q		; Save loop index in Q
		INDEX	Q		; Compute difference:
		CS	WWPOS		; -WWPOS[index]
		INDEX	Q		; (current displayed value)
		AD	WWPOS +6	; +WWPOS+6[index] (saved value)
		ADS	L		; Accumulate sum of differences in L
		CCS	Q		; Decrement and test loop index
		TCF	N99LOOP		; Continue if index > 0
		LXCH	A		; Move sum to A register
		EXTEND			; Test for zero
		BZF	V06N9933	; Sum = 0: no changes, continue
		TC	UPFLAG		; Sum ≠ 0: crew entered new values
		ADRES	V67FLAG		; Set flag indicating W update needed

; Process crew input. If V67FLAG set, crew entered new W-matrix values which
; must be stored back to appropriate W-matrix locations based on mode.

V06N9933	TC	INTPRET		; Enter interpretive mode
		BON	EXIT		; Branch if V67FLAG set
			V67FLAG		; (Crew entered new values)
			+2		; Skip EXIT and continue processing
		TCF	ENDEXT		; V67FLAG clear: exit, no update needed
; Crew entered new W-matrix uncertainty values. Scale from display units
; (B-5 for feet/fps/degrees) to internal units (B+5) and store to appropriate
; W-matrix diagonal elements based on navigation mode (rendezvous or surface).

		DLOAD			; Load position uncertainty
# Page 609
			WWPOS		; (From DSKY, in feet)
		SL4	SL1		; Scale: B-5 → B+5 (shift left 5 bits)
		STODL	0D		; Store in scratch 0D, load velocity
			WWVEL		; (From DSKY, in ft/sec)
		STODL	2D		; Store velocity in 2D, load bias
			WWBIAS		; (From DSKY, in degrees)
		SL			; Scale bias: B-5 → B+5
			10D		; (10 decimal = shift left 5 bits)
		STORE	4D		; Bias now in 4D

; Determine navigation mode: rendezvous (orbital) or surface navigation.
; W-matrix elements stored at different addresses for each mode.

		BON	LXA,1		; Branch if on lunar surface
			SURFFLAG	; (P22 surface navigation mode)
			V67SURF		; Go to surface W-element addresses
			0D		; Load rendezvous position index
		SXA,1	LXA,1		; Store position to WRENDPOS index
			WRENDPOS
			2D		; Load velocity value
		SXA,1	GOTO		; Store velocity to WRENDVEL index
			WRENDVEL
			V67CLRF		; Continue to bias storage

; Surface navigation mode: Store uncertainties to surface W-matrix elements.

V67SURF		LXA,1	SXA,1		; Load and store position index
			0D
			WSURFPOS	; W surface position element
		LXA,1	SXA,1		; Load and store velocity index
			2D
			WSURFVEL	; W surface velocity element

; Store IMU bias uncertainties (same for both modes). Clear rendezvous
; downweighting flag to give full weight to radar measurements.

V67CLRF		LXA,1	SXA,1		; Store trunnion bias uncertainty
			4D
			WTRUN		; W trunnion (RR T angle) element
		SXA,1			; Store shaft bias uncertainty
			WSHAFT		; W shaft (RR S angle) element
		CLEAR	EXIT		; Clear rendezvous downweight flag
			RENDWFLG	; (Allows full Kalman gain on RR marks)
		TCF	ENDEXT		; Exit V67, return to caller
; Subroutine V67WW: Compute RMS (root-mean-square) position, velocity, and
; bias uncertainties from W-matrix diagonal elements.
;
; W-matrix is 18x18 state covariance matrix: 9 state variables (position X,Y,Z,
; velocity X,Y,Z, and 3 IMU gyro biases), each with variance and covariances.
; This routine extracts diagonal variances (uncertainty squared), sums the
; squares for position (3 terms), velocity (3 terms), and bias (3 terms),
; then takes square roots to get RMS uncertainties in feet, ft/sec, degrees.

V67WW		STQ	BOV		; Save return address in S2
			S2		; (Will return via QPRET loaded from S2)
			+1		; Trap overflow, continue at +1
		CLEAR	CALL		; Clear V67FLAG (new computation)
			V67FLAG
			INTSTALL	; Install interrupt inhibit if needed
		SSP	DLOAD		; Set S1 pointer to 6 decimal
			S1
		DEC	6		; (Loop counter for 6 components)
			ZEROVECS	; Load zero vector
		STORE	WWPOS		; Initialize position accumulator
		STORE	WWVEL		; Initialize velocity accumulator
		STORE	WWBIAS		; Initialize bias accumulator
		AXT,1			; Set index register X1 = 54 decimal
		DEC	54		; (Start at W element 54)

; Loop through W-matrix diagonal elements. W has 18 elements per row.
; Position elements: 0, 18, 36 (offset +54 from index for X,Y,Z)
; Velocity elements: 54, 72, 90 (offset +108)
; Bias elements: 108, 126, 144 (offset +162)
; Index decrements: 54, 36, 18, 0 (stepping by -18 each iteration)

NXPOSVEL	VLOAD*	VSQ		; Load vector at W+54D[X1], square it
			W +54D,1	; (Variance = uncertainty squared)
# Page 610
		GOTO			; Continue to accumulate
			ADDPOS		; (Separate bank for space)
V06N99		VN	0699		; Verb 06, Noun 99 (display code)

		SETLOC	VB67A		; Continuation bank for V67
		BANK
		COUNT*	$$/EXTVB

; Accumulate squared uncertainties for position, velocity, and bias.
; Each diagonal W-matrix element represents variance for one component.

ADDPOS		DAD			; Add position variance to accumulator
			WWPOS		; (Sum of position variances)
		STORE	WWPOS		; Update position accumulator
		VLOAD*	VSQ		; Load velocity vector, square
			W +108D,1	; (Velocity variance elements)
		DAD			; Add to velocity accumulator
			WWVEL
		STORE	WWVEL		; Update velocity accumulator
		VLOAD*	VSQ		; Load bias vector, square
			W +162D,1	; (IMU gyro bias variance elements)
		DAD			; Add to bias accumulator
			WWBIAS
		STORE	WWBIAS		; Update bias accumulator
		TIX,1	SQRT		; Decrement X1, loop if nonzero
			NXPOSVEL	; Process next W-matrix row

; Loop complete. Accumulators contain sum of variances (uncertainty squared).
; Take square roots to get RMS uncertainties, scale for display.

		SR			; Shift right: B+5 (internal) → B-5
			10D		; (Display scaling: feet, ft/sec, deg)
		STODL	WWBIAS		; Store bias RMS, load velocity sum
			WWVEL
		SQRT			; Square root of velocity variance sum
		STODL	WWVEL		; Store velocity RMS, load position
			WWPOS
		SQRT			; Square root of position variance sum
		STORE	WWPOS		; Store position RMS uncertainty
		BOV	GOTO		; Check overflow, branch if occurred
			+2		; Skip next instruction if overflow
			V67XXX		; Continue at overflow recovery
		DLOAD			; Overflow occurred: saturate to max
			DPPOSMAX	; Load maximum double precision value
		STORE	WWPOS		; Store max in all three displays
		STORE	WWVEL
		STORE	WWBIAS		; (Indicates W-matrix instability)

; Check if position uncertainty exceeds display range (99,999 feet).
; Noun 99 displays have 5-digit decimal range. Limit to displayable values.

V67XXX		DLOAD	DSU		; Load position RMS, subtract max
			WWPOS
			FT99999		; 99,999 feet display limit (B-19)
		BMN	DLOAD		; Branch if negative (within range)
			+3		; Skip limiting if in range
			FT99999		; Load display maximum
		STORE	WWPOS		; Limit position to 99,999 feet
		LXA,1	SXA,1		; Restore return address from S2
			S2		; to QPRET for interpretive return
			QPRET
# Page 611
		EXIT			; Return to native AGC code
		TC	POSTJUMP	; Post job to wake up waiting display
		CADR	INTWAKE		; (Resumes GOFLASH wait in V67CALL)

; Display limit constant: 99,999 feet (maximum 5-digit decimal display).
; Scaled B-19 for internal computation units.

FT99999		2DEC	30479 B-19	; 99999 feet at B-19 scaling

# Page 612

; ============================================================================
; SECTION: RADAR TRACKING LIGHT CONTROL ROUTINES
;
; Manages DSKY indicator lights showing rendezvous radar tracking status.
; During rendezvous operations, crew monitors VEL (velocity) and ALT (altitude)
; lights indicating radar lock-on quality for horizontal and vertical axes.
;
; COMMENT-ONLY READERS: After Eagle's ascent on July 21, 1969, these routines
; provided visual feedback to Armstrong and Aldrin as rendezvous radar acquired
; and tracked Columbia. Steady lights indicated good tracking data flowing into
; navigation. Flashing lights warned of tracking problems or data dropouts.
;
; CODE-ALONG READERS: Light control integrates with radar interrupt service.
; R12LITES called from T4RUPT checks tracking modes and calls appropriate
; light routines (HLIGHT for horizontal velocity, VLIGHT for vertical rate).
; Flash/steady state determined by FLGWRD11 flags and RADMODES bits.
; ============================================================================

		BANK	25
		SETLOC	RADARUPT
		BANK
		COUNT*	$$/RRUPT

		EBANK=	LOSCOUNT

; R12LITES: Entry point from radar service routine. Check if rendezvous radar
; tracking active, update horizontal and vertical tracking lights accordingly.

R12LITES	CA	ONE		; Load bit 1 (rendezvous mode bit)
		MASK 	IMODES33	; Test against IMODES33 flag word
		CCS	A		; Check if rendezvous radar selected
		TCF	ISWRETRN	; Yes: return immediately (mode active)

; Rendezvous radar not in use. Update both H and V tracking lights.

		TC	HLIGHT		; Update horizontal velocity light
		TC	HLIGHT -3	; (Second call processes differently)
		TCF	ISWRETRN	; Return to interrupt service

; RADLITES: Radar light dispatcher for mode-specific light control.
; Checks if rendezvous radar in designated mode, routes to appropriate handler.
; During Apollo 11 rendezvous, this determined which tracking axis lights update.

RADLITES	CA	BIT1		; Load rendezvous mode bit
		MASK	IMODES33	; Check against mode flags
		CCS	A		; Is RR mode active?
		TC	Q		; Yes: return immediately

; RR not in designated mode. Check which tracking axis to update.

		CS	BIT5		; Complement H-light bit
		AD	ITEMP1		; Add to item temp 1
		CCS	A		; Test result sign
		CS	ONE		; Negative: load -1
		TCF	VLIGHT		; Branch to vertical light handler

; Fall through to radar tracking off path.

		TCF	RRTRKF		; Radar tracking failed/off

; HLIGHT: Horizontal velocity tracking light control. Updates VEL light on DSKY
; based on radar tracking mode flags. Light steady = good lock, flashing = poor
; data quality. Called with A=0 for H-axis, A=nonzero for V-axis via entry+3.
; During rendezvous, Armstrong and Aldrin watched these lights continuously
; to verify radar was providing good relative velocity data to navigation.

HLIGHT		TS	ITEMP5		; Store A in ITEMP5 (index: 0=H, !=0=V)

		CA	HLITE		; Load horizontal light bit (BIT5)
		TS	L		; Save in L register for later use

; Check if S-band antenna in operation (SCAB mode). If yes, turn on lights.

		CA	FLGWRD11	; Load flag word 11
		MASK	SCABBIT		; Test S-band antenna bit
		CCS	A		; Is SCAB mode active?
		TCF	ONLITES		; Yes: turn on tracking lights

; SCAB not active. Check radar mode bits for horizontal/vertical tracking.

		CA	LRALTBIT	; Load landing radar altitude bit
BOTHLITS	MASK	RADMODES	; Check against radar mode word
		CCS	A		; Is this radar mode active?
		TCF	ONLITES		; Yes: turn lights on steady

; Radar mode not active for this axis. Check if light should flash (poor track).

		CA	FLGWRD11	; Load flag word 11
		INDEX	ITEMP5		; Index by 0 (HFLSHBIT) or !=0 (VFLSHBIT)
		MASK	HFLSHBIT	; Get flash bit for this axis
		CCS	A		; Is flash bit set?
		TCF	RRTRKF		; Yes: handle flashing light
# Page 613

; LITIT: Light illumination handler. Turns on tracking light (steady, not flash).
; Saves return address, calls tracking-light-on routine, then branches to cleanup.

LITIT		EXTEND		; Extended instruction follows
		QXCH	ITEMP6		; Save return address in ITEMP6
		TC	TRKFLON +1	; Call tracking light ON (+1 entry)

; Light now on. Clean up and exit radar light processing.

		EXTEND		; Extended instruction follows
		QXCH	ITEMP6		; Restore return address
		TCF	RRTRKF		; Exit to radar tracking return

; ONLITES: Force lights ON (steady illumination, clear flash bits).
; Called when radar has good lock and tracking data quality acceptable.
; Clears flash flag for this axis, then illuminates light steadily.

ONLITES		INDEX	ITEMP5		; Index by axis (0=H, !=0=V)
		CS	HFLSHBIT	; Complement flash bit for this axis
		MASK	FLGWRD11	; Clear the flash bit in flag word
		TS	FLGWRD11	; Store updated flags (flash off)

		CA	L		; Retrieve light bit from L register
		TCF	LITIT		; Branch to light illumination
; VLIGHT: Vertical rate tracking light control. Updates ALT light on DSKY
; based on vertical radar tracking mode. Entry point for V-axis light control,
; parallel to HLIGHT for H-axis. Loads V-light bit and branches to common logic.

VLIGHT		TS	ITEMP5		; Store A in ITEMP5 (V-axis index)
		CA	VLITE		; Load vertical light bit (BIT3)
		TS	L		; Save in L register for later use
		CA	BIT8		; Load bit 8 (V-axis radar mode bit)
		TCF	BOTHLITS	; Branch to common light control logic

; Light bit pattern constants for horizontal and vertical tracking indicators.
; These bits control DSKY indicator lights visible to crew during rendezvous.
; HLITE = BIT5 corresponds to VEL (velocity) light.
; VLITE = BIT3 corresponds to ALT (altitude) light.

HLITE		EQUALS	BIT5		; Horizontal velocity light bit
VLITE		EQUALS	BIT3		; Vertical altitude rate light bit

# *** END OF LEMP20S .127 ***
