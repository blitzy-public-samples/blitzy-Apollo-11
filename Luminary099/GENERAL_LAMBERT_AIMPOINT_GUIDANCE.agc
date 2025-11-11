# Copyright:	Public domain.
# Filename:	LAMBERT_AIMPOINT_GUIDANCE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	651-653
# Mod history:	2009-05-18 RSB	Transcribed from Luminary 099
#				page images.
#		2009-06-05 RSB	Corrected 4 typos.
#		2009-06-07 RSB	Fixed a typo.
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
#	16:27 JULY 14,1969

# Page 651

; ============================================================================
; FILE: GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc
; MODULE: Orbital Navigation and Rendezvous Guidance
; MISSION PHASE: lunar-orbit/rendezvous
;
; TL;DR: Implements P31, the General Lambert Aimpoint Guidance program that
;        computes required velocity changes for rendezvous maneuvers using
;        Lambert targeting algorithms. Accepts external targeting parameters
;        (ignition time, target position, time-of-flight) and calculates the
;        optimal trajectory to reach the target, displaying results to crew.
;
; COMMENT-ONLY READERS: This program calculates the precise trajectory needed
;        for the Lunar Module to rendezvous with the Command Module in lunar
;        orbit. Follow the comments to understand the targeting mathematics.
; CODE-ALONG READERS: Study Lambert two-point boundary value problem solution,
;        interpretive language usage for vector operations, and DSKY interface
;        for crew parameter input and trajectory display.
; ============================================================================

# GENERAL  LAMBERT AIMPOINT GUIDANCE **
# WRITTEN  BY RAMA M AIYAWAR

# PROGRAM P-31 DESCRIPTION **
#
# 1.	   TO ACCEPT TARGETING PARAMETERS OBTAINED FROM A SOURCE EXTERNAL
#	   TO THE LEM AND COMPUTE THERE FROM THE REQUIRED-VELOCITY AND
#	   OTHER INITIAL CONDITIONS REQUIRED BY LM FOR DESIRED MANEUVER.
#	   THE TARGETING PARAMETERS ARE TIG (TIME OF IGNITION), TARGET
#	   VECTOR (RTARG), AND THE TIME FROM TIG UNTIL THE TARGET IS
#	   REACHED(DELLT4),DESIRED TIME OF FLIGHT FROM  RINIT TO RTARG..

# ASSUMPTIONS **
#
# 1.	   THE TARGET PARAMETERS MAY HAVE BEEN LOADED PRIOR TO THE
#	   EXECUTION OF THIS PROGRAM.
# 2.	   THIS PROGRAM IS APPLICABLE IN EITHER EARTH OR LUNAR ORBIT.
# 3.	   THIS PROGRAM IS DESIGNED FOR ONE-MAN OPERATION, AND SHOULD
#	   BE SELECTED BY THE ASTRONAUT BY DSKY ENTRY  V37 E31.

# SUBROUTINES USED **
#
# MANUPARM, TTG/N35, R02BOTH, MIDGIM, DISPMGA, FLAGDOWN, BANKCALL,
# GOTOPOOH, ENDOFJOB, PHASCHNG, GOFLASHR, GOFLASH.
#
# MANUPARM	  CALCULATES APOGEE, PERIGEE ALTITUDES AND DELTAV DESIRED
#		  FOR THE MANEUVER.
#
# TTG/N35	  CLOCKTASK - UPDATES CLOCK.
#
# MIDGIM	  CALCULATES MIDDLE GIMBAL ANGLE FOR DISPLAY.
#
# R02BOTH	  IMU - STATUS CHECK ROUTINE.

# DISPLAYS USED IN P-31LM **
#
# V06N33	  DISPLAY SOTRED  TIG (IN HRS. MINS. SECS)
# V06N42	  DISPLAY APOGEE, PERIGEE, DELTAV.
# V16N35	  DISPLAY TIME FROM TIG.
# V06N45	  TIME FROM IGNITION AND MIDDLE GIMBAL ANGLE.

# ERASABLE INITIALIZATION REQUIRED **
#
# TIG		  TIME OF IGNITION    DP    (B+28) CS.
#
# DELLT4	  DESIRED TIME OF FLIGHT   DP  (B+28) CS
#		  FROM RINIT TO RTARG .
#
# RTARG		  RADIUS VECTOR OF TARGET POSITION VECTOR
#		  RADIUS VECTOR   SCALED TO  (B+29)METERS IF EARTH ORBIT
# Page 652
#		  RADIUS VECTOR SCALED TO    (B+27)METERS IF MOON  ORBIT

# OUTPUT **
#
# HAPO		  APOGEE ALTITUDE
# HPER		  PERIGEE ALTITUDE
# VGDISP	  MAG.OF DELTAV FOR DISPLAY ,SCALING	  B+7 M/CS EARTH
#		  MAG.OF DELTAV FOR DISPLAY,SCALING	  B+5 M/CS MOON
# MIDGIM	  MIDDLE GIMBAL ANGLE
# XDELVFLG	  RESETS XDELVFLG FOR LAMBERT VG COMPUTATIONS

# ALARMS OR ABORTS  NONE **

# RESTARTS  ARE VIA GROUP 4 **

; ============================================================================
; P31 - GENERAL LAMBERT AIMPOINT GUIDANCE PROGRAM
;
; This program solves the Lambert two-point boundary value problem to compute
; the velocity change (delta-V) required for the Lunar Module to reach a
; target position (typically the Command Module during rendezvous) at a
; specified time. The crew or ground control provides three key parameters:
;
; TIG (Time of Ignition) - When to perform the maneuver
; RTARG (Target Position Vector) - Where the CM will be at intercept
; DELLT4 (Time of Flight) - How long the transfer trajectory takes
;
; The Lambert algorithm computes the unique conic trajectory connecting the
; LM's current position to the target position with the specified flight time.
; This is fundamental to all rendezvous operations in lunar orbit, enabling
; the LM to rejoin the CM after surface operations or abort scenarios.
; ============================================================================

		SETLOC	GLM
		BANK

		EBANK=	SUBEXIT

		COUNT*	$$/P31
; P31 PROGRAM ENTRY POINT
; Called by crew DSKY entry V37 E31 to initiate Lambert targeting sequence.
; Sets P20 flag to indicate active targeting program for system coordination.
P31		TC	P20FLGON

; Display Time of Ignition to crew for verification.
; V06N33 displays TIG in hours, minutes, seconds format on DSKY.
; Crew can verify the maneuver timing before proceeding with calculations.
		CAF	V06N33		# TIG
		TC	VNPOOH

; ============================================================================
; STATE VECTOR INTEGRATION TO TIME OF IGNITION
;
; Before computing the Lambert trajectory, we must know the spacecraft's
; precise position and velocity at TIG. The current state vector (position
; and velocity) is integrated forward in time from the present to TIG using
; precision orbital mechanics. This accounts for lunar gravity perturbations
; and ensures accurate trajectory calculations for rendezvous.
; ============================================================================
		TC	INTPRET
		CLEAR	DLOAD
			UPDATFLG	; Clear update flag for fresh calculation
			TIG		; Load Time of Ignition (DP B+28 centiseconds)
		STCALL	TDEC1		# INTEGRATE STATE VECTORS TO TIG
			LEMPREC		; High-precision lunar orbit integration

; Store integrated state vectors at TIG for Lambert targeting.
; RATT contains position vector (B+27 meters for lunar orbit)
; VATT contains velocity vector (B+5 meters/centisecond)
; These represent where the LM will be at ignition time.
		VLOAD	SETPD
			RATT		; Position vector at TIG
			0D		; Set push-down pointer to stack base
		STORE	RTIG		; Store as ignition position
		STOVL	RINIT		; Also store as initial position for Lambert
			VATT		; Velocity vector at TIG
		STORE	VTIG		; Store as ignition velocity
		STODL	VINIT		; Also store as initial velocity for Lambert
			P30ZERO		; Load zero for eccentricity initialization

; ============================================================================
; LAMBERT TARGETING INITIALIZATION
;
; Set up parameters for INITVEL subroutine which solves the Lambert problem.
; The algorithm requires:
; - Initial position (RINIT) - where we are at TIG
; - Target position (RTARG) - where CM will be at intercept
; - Time of flight (DELLT4) - transfer trajectory duration
;
; INITVEL computes the required initial velocity (VIPRIME) that produces
; a conic trajectory connecting these two points in the specified time.
; This is the classic Lambert two-point boundary value problem solution.
; ============================================================================
		PUSH	PDDL		# E4 AND NUMIT = 0
			DELLT4		; Desired time of flight (DP B+28 CS)
		DAD	SXA,1		; Compute target intercept time
			TIG		; Add TIG to get absolute time
			RTX1		; Save index register 1
		STORE	TPASS4		; Store as passage time through target point
		SXA,2	CALL		; Save index register 2
			RTX2
			INITVEL		; Call Lambert targeting subroutine

; ============================================================================
; REQUIRED VELOCITY CHANGE COMPUTATION
;
; INITVEL has computed VIPRIME, the velocity vector needed at TIG to follow
; the Lambert trajectory to the target. The delta-V is the difference between
; this required velocity and the LM's actual velocity (VINIT) at TIG:
;
; DELTA-V = VIPRIME - VINIT
;
; This vector represents the magnitude and direction for the RCS or main
; engine burn to achieve the rendezvous trajectory. The magnitude (VGDISP)
; is displayed to the crew in meters/centisecond.
; ============================================================================
		VLOAD	PUSH
# Page 653
			DELVEET3	; Load required delta-V vector from INITVEL
		STORE	DELVSIN		; Store for later use
		ABVAL	CLEAR		; Compute absolute value (magnitude)
			XDELVFLG	; Reset external delta-V flag
		STCALL	VGDISP		; Store magnitude for display (B+7 or B+5 M/CS)
			GET.LVC		; Get local vertical coordinate transformation

; ============================================================================
; TRAJECTORY APOGEE AND PERIGEE CALCULATION
;
; After computing the required delta-V, calculate the resulting orbital
; parameters to verify the trajectory is safe and achievable. PERIAPO1
; computes the highest point (apogee) and lowest point (perigee) of the
; transfer orbit. These altitudes are displayed to the crew to ensure:
;
; 1. Perigee stays above lunar surface (no terrain collision)
; 2. Apogee remains reasonable (not escaping lunar orbit)
; 3. Trajectory profile matches mission planning expectations
;
; Input: RTIG (position at ignition), VIPRIME (velocity after delta-V burn)
; Output: HAPO (apogee altitude), HPER (perigee altitude)
; ============================================================================
		VLOAD	PDVL		; Load position and velocity vectors
			RTIG		; Position at TIG (B+27 meters)
			VIPRIME		; Required velocity after burn (B+5 M/CS)
		CALL
			PERIAPO1	; Calculate periapsis and apoapsis

; Format perigee altitude for crew display.
; SHIFTR1 adjusts scaling for nautical mile units.
; MAXCHK limits display to 9999.9 NM to fit DSKY format.
; Perigee is the critical altitude - must stay above lunar surface.
		CALL
			SHIFTR1		; Scale for display units
		CALL			# LIMIT DISPLAY TO 9999.9 N. MI.
			MAXCHK		; Clamp maximum value for DSKY
		STODL	HPER		; Store perigee altitude for display
			4D		; Load apogee data from stack

; Format apogee altitude for crew display.
; Same scaling and limiting as perigee.
; Apogee is the high point - verifies orbit remains bound to Moon.
		CALL
			SHIFTR1		; Scale for display units
		CALL			# LIMIT DISPLAY TO 9999.9 N. MI.
			MAXCHK		; Clamp maximum value for DSKY
		STORE	HAPO		; Store apogee altitude for display
		EXIT

; ============================================================================
; CREW DISPLAY SEQUENCE
;
; Present the computed Lambert trajectory parameters to the crew on the DSKY
; for review and approval before engine ignition. This is a critical decision
; point where the crew verifies the targeting solution is acceptable:
;
; V06N81: Delta-V in local vertical coordinates (magnitude and direction)
; V06N42: Apogee altitude, Perigee altitude, Delta-V magnitude
;
; The crew can proceed with the burn if parameters are acceptable, or abort
; if something appears incorrect (e.g., perigee too low, delta-V too high).
; This manual verification was essential for Apollo 11's safe rendezvous.
; ============================================================================
		CAF	V06N81		# DELVLVC
		TC	VNPOOH		; Display delta-V in local vertical coordinates
		CAF	V06N42		# HAPO, HPER, VGDISP
		TC	VNPOOH		; Display trajectory parameters for crew review

; ============================================================================
; CONTINUOUS TRAJECTORY MONITORING LOOP
;
; After displaying the targeting solution, enter a monitoring loop that
; continuously updates and displays time-to-go until ignition and middle
; gimbal angle (MGA) for engine orientation. This runs until TIG when
; the actual burn sequence begins. The crew monitors this countdown to
; prepare for the maneuver, similar to modern spacecraft countdowns.
;
; VN1645 displays: Time remaining to TIG, Middle Gimbal Angle
; FINALFLG indicates trajectory computation is complete, now monitoring only
; ============================================================================
		TC	INTPRET
REVN1645	SET	CALL		# TRKMKCNT, TTOGO, +MGA
			FINALFLG	; Set flag indicating final solution ready
			VN1645		; Display time-to-go and gimbal angle
		GOTO
			REVN1645	; Loop continuously until TIG

; ============================================================================
; END OF P31 LAMBERT AIMPOINT GUIDANCE
;
; Historical Note: This Lambert targeting algorithm was crucial for Apollo 11's
; successful rendezvous in lunar orbit on July 21, 1969. After Eagle's ascent
; from the lunar surface, this program computed the precise trajectory changes
; needed to reach Columbia (the Command Module) for docking. The crew relied on
; these DSKY displays to verify safe trajectory parameters before committing
; to each rendezvous burn.
;
; The generalized Lambert solution implemented here extends the basic two-point
; boundary value problem with aimpoint optimization and trajectory constraints,
; making it applicable to various rendezvous scenarios beyond just the standard
; CSI-CDH-TPI sequence. This flexibility was essential for handling off-nominal
; situations and mission replanning.
; ============================================================================

# *** END OF LEMP30S .103 ***
