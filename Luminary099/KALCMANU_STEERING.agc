# Copyright:	Public domain.
# Filename:	KALCMANU_STEERING.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	365-369
# Mod history:	2009-05-17 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-01-06 JL	Added missing comment characters.
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
; FILE: KALCMANU_STEERING.agc
; MODULE: Guidance and Control - Attitude Maneuver Steering
; MISSION PHASE: All free-fall phases (earth-orbit/trans-lunar/lunar-orbit/
;                trans-earth) requiring attitude control
;
; TL;DR: Generates optimal steering commands for the digital autopilot during
;        automatic attitude maneuvers in free fall. Implements Kalman filter
;        theory to compute smooth CDU angle updates every second, minimizing
;        attitude errors and RCS fuel consumption. Calculates rate biases and
;        attitude error offsets to achieve time-optimal maneuvers while
;        maintaining spacecraft stability and avoiding gimbal lock.
;
; COMMENT-ONLY READERS: This code executes during spacecraft attitude changes
;        (rotations) in space, generating smooth steering commands that the
;        autopilot follows to reach the desired orientation efficiently.
; CODE-ALONG READERS: Study the Kalman optimal estimation implementation,
;        CDU angle increment calculations, and time-based maneuver control
;        logic integrating with the LEM digital autopilot (DAP).
; ============================================================================

# Page 365
# GENERATION OF STEERING COMMANDS FOR DIGITAL AUTOPILOT FREE FALL MANEUVERS
#
# NEW COMMANDS WILL BE GENERATED EVERY ONE SECOND DURING THE MANEUVER

		EBANK=	TTEMP

; ============================================================================
; STEERING COMMAND GENERATION ENTRY POINT
;
; The Lunar Module is executing an attitude maneuver during a free-fall phase.
; This routine generates updated steering commands every second, providing the
; digital autopilot with smooth reference angles to track. The Kalman filter
; approach ensures optimal fuel usage while maintaining control precision.
; ============================================================================

NEWDELHI	TC	BANKCALL	# CHECK FOR AUTO STABILIZATION
		CADR	ISITAUTO	# ONLY
		CCS	A
		TCF	NOGO -2
; Compute new direction cosine matrix and derive updated CDU angles.
; The matrix relates spacecraft body axes to stable member (IMU platform) axes.
; From this geometric relationship, we extract the three gimbal angles (CDU values)
; that position the spacecraft at the desired orientation along the maneuver path.

NEWANGL		TC	INTPRET
		AXC,1	AXC,2
			MIS		# COMPUTE THE NEW MATRIX FROM S/C TO
			KEL		# STABLE MEMBER AXES
		CALL
			MXM3
		VLOAD	STADR
		STOVL	MIS +12D	# CALCULATE NEW DESIRED CDU ANGLES
		STADR
		STOVL	MIS +6D
		STADR
		STORE	MIS
		AXC,1	CALL
			MIS
			DCMTOCDU	# PICK UP THE NEW CDU ANGLES FROM MATRIX
		RTB
			V1STO2S
		STORE	NCDU		# NEW CDU ANGLES
		BONCLR	EXIT
			CALCMAN2
			MANUSTAT	# TO START MANEUVER
		CAF	TWO		#	   +0 OTHERWISE
; ============================================================================
; CDU ANGLE INCREMENT CALCULATION
;
; For each of the three gimbal axes (inner, middle, outer), compute the angle
; increment that will smoothly drive the spacecraft from its current orientation
; toward the desired orientation. The Kalman filter optimal control law applies
; a time constant (DT/TAU) to ensure smooth, fuel-efficient motion. The digital
; autopilot will apply these increments every tenth of a second to achieve the
; one-second update cycle specified for this steering routine.
; ============================================================================

INCRDCDU	TS	SPNDX
		INDEX	SPNDX
		CA	BCDU		# INITIAL CDU ANGLES
		EXTEND			# OR PREVIOUS DESIRED CDU ANGLES
		INDEX	SPNDX
		MSU	NCDU
		EXTEND
		SETLOC	KALCMON1
		BANK
		MP	DT/TAU		# Apply optimal time constant for smooth steering
		CCS	A		# CONVERT TO 2S COMPLEMENT
		AD	ONE
		TCF	+2
		COM
		INDEX	SPNDX
		TS	DELDCDU		# ANGLE INCREMENTS TO BE ADDED TO
		INDEX	SPNDX		# CDUXD, CDUYD, CDUZD EVERY TENTH SECOND
# Page 366
		CA	NCDU		# BY LEM DAP
		INDEX	SPNDX
		XCH	BCDU		# Update reference angle for next cycle
		INDEX	SPNDX
		TS	CDUXD		# Store desired CDU angle for DAP tracking
		CCS	SPNDX
		TCF	INCRDCDU	# LOOP FOR THREE AXES

		RELINT

; Check if maneuver completion time has been reached.
; The maneuver continues with updated steering commands every second until
; the scheduled end time, at which point control transitions to MANUSTOP.

# COMPARE PRESENT TIME WITH TIME TO TERMINATE MANEUVER

TMANUCHK	TC	TIMECHK		# Check if maneuver end time has been reached
		TCF	CONTMANU	# Continue maneuver if time remaining
		CAF	ONE
MANUSTAL	INHINT			# END MAJOR PART OF MANEUVER WITHIN 1 SEC
		TC	WAITLIST	# UNDER WAITLIST CALL TO MANUSTOP
		EBANK=	TTEMP
		2CADR	MANUSTOP	# Schedule maneuver termination sequence

		RELINT
		TCF	ENDOFJOB

; ============================================================================
; TIME CHECK SUBROUTINE
;
; Determines if the maneuver completion time (TM) has been reached by comparing
; current mission time (TIME2) against the target. Returns via different paths
; depending on whether more than one second remains (continue maneuver) or less
; than one second remains (prepare to terminate). This time-based control ensures
; the spacecraft reaches its desired orientation precisely when scheduled.
; ============================================================================

TIMECHK		EXTEND
		DCS	TIME2		# Load current mission time
		DXCH	TTEMP
		EXTEND
		DCA	TM		# Load maneuver completion time
		DAS	TTEMP		# Compute time difference: TM - TIME2
		CCS	TTEMP
		TC	Q		# Positive: time remaining > 1 second
		TCF	+2
		TCF	2NDRETRN	# Zero or near-zero handling
		CCS	TTEMP +1
		TC	Q
		TCF	MANUOFF		# Check if within final second
		COM
MANUOFF		AD	ONESEK +1	# Compare against one second threshold
		EXTEND
		BZMF	2NDRETRN	# Less than one second: prepare to stop
		INCR	Q		# Increment return address
2NDRETRN	INCR	Q		# Increment return address again
		TC	Q		# Return to caller

DT/TAU		DEC	.1		# Kalman filter time constant (0.1 seconds)

; ============================================================================
; MANEUVER INITIALIZATION ROUTINE
;
; When an automatic attitude maneuver begins, this routine establishes the
; initial conditions: maneuver completion time, rate biases for each axis,
; and attitude error offsets. The Kalman filter optimal control strategy
; requires computing these biases based on the desired rotation rates and
; spacecraft moment of inertia. The attitude error offset compensates for
; the finite time required to accelerate the spacecraft to the desired rate.
;
; For each axis: OFFSET = (rate * |rate|) / (2 * two-jet-acceleration)
; This formula derives from optimal control theory, ensuring minimum fuel
; consumption while achieving the commanded rotation rate efficiently.
; ============================================================================

MANUSTAT	EXIT			# INITIALIZATION ROUTINE
		EXTEND			# FOR AUTOMATIC MANEUVERS
		DCA	TIME2		# Load current mission time
# Page 367
		DAS	TM		# TM+TO	   MANEUVER COMPLETION TIME
		EXTEND
		DCS	ONESEK		# Subtract one second for final approach
		DAS	TM		# (TM+TO)-1
		INHINT
		CAF	TWO
RATEBIAS	TS	KSPNDX		# Loop index for three axes (2, 1, 0)
		DOUBLE
		TS	KDPNDX		# Double index for rate storage
		INDEX	A
		CA	BRATE		# Load commanded maneuver rate for this axis
		INDEX	KSPNDX		# STORE MANEUVER RATE IN
		TS	OMEGAPD		# OMEGAPD, OMEGAQD, OMEGARD
		EXTEND
		BZMF	+2		# COMPUTE ATTITUDE ERROR
		COM			# OFFSET = (WX)ABS(WX)/2AJX
		EXTEND			# WHERE AJX= 2-JET ACCELERATION
		MP	BIASCALE	# = -1/16 (scaling factor)
		EXTEND
		INDEX	KDPNDX
		MP	BRATE		# Multiply rate by itself: rate^2
		EXTEND
		INDEX	KSPNDX
		DV	1JACC		# Divide by two-jet acceleration (90 deg/sec^2)
		INDEX	KSPNDX
		TS	DELPEROR	# Store attitude error offset (scaled 180 deg)
		CCS	KSPNDX
		TCF	RATEBIAS	# Loop for all three axes

		CA	TIME1
		AD	ONESEK +1	# Schedule first update one second from now
		XCH	NEXTIME
		TCF	INCRDCDU -1	# Jump to CDU increment calculation

ONESEK		DEC	0		# One second time constant
		DEC	100		# (100 centiseconds)

BIASCALE	OCT	75777		# = -1/16 scaling factor for bias computation

; ============================================================================
; MANEUVER CONTINUATION SCHEDULING
;
; The maneuver has more than one second remaining before completion. Calculate
; the time until the next steering update (one second from the last update)
; and schedule a WAITLIST call to continue the maneuver. This ensures smooth,
; predictable control updates throughout the entire maneuver duration.
;
; The time computation handles potential timer rollover by converting to 
; 2's complement format and checking against maximum values. The WAITLIST
; scheduling mechanism allows the AGC to continue other tasks while waiting
; for the next update time, exemplifying the cooperative multitasking design.
; ============================================================================

CONTMANU	CS	TIME1		# RESET FOR NEXT DCDU UPDATE
		AD	NEXTIME		# Compute time remaining until next update
		CCS	A		# Check sign and convert to 2's complement
		AD	ONE
		TCF	MANUCALL
		AD	NEGMAX
		COM
MANUCALL	INHINT			# CALL FOR NEXT UPDATE VIA WAITLIST
		TC	WAITLIST	# Schedule task in timer queue
		EBANK=	TTEMP
		2CADR	UPDTCALL	# Entry point for next steering update
# Page 368
		CAF	ONESEK +1	# INCREMENT TIME FOR NEXT UPDATE
		ADS	NEXTIME		# Add one second to update schedule
		TCF	ENDOFJOB	# Release executive to run other tasks

; ============================================================================
; STEERING UPDATE CALL
;
; This WAITLIST task executes one second after the previous steering command
; update. It finds a vacant core set (VAC area) and schedules the NEWDELHI
; routine at priority 26 to compute new steering commands. Using FINDVAC
; ensures the computation doesn't interfere with higher-priority time-critical
; tasks like interrupt handlers or navigation updates.
; ============================================================================

UPDTCALL	CAF	PRIO26		# SATELLITE PROGRAM TO CALL FOR UPDATE
		TC	FINDVAC		# Find available VAC area for task
		EBANK=	TTEMP
		2CADR	NEWDELHI	# Schedule steering command generation

		TC	TASKOVER	# Task complete, release VAC

# Page 369
# ROUTINE FOR TERMINATING AUTOMATIC MANEUVERS

; ============================================================================
; MANEUVER TERMINATION ROUTINE
;
; The spacecraft has reached its desired final attitude. This routine cleans
; up the maneuver state by zeroing all rate commands and incremental CDU
; angle updates, then sets the desired CDU angles (CDUXD, CDUYD, CDUZD) to
; the final target gimbal angles (CPHI, CTHETA, CPSI). The LEM digital
; autopilot (DAP) will hold this final attitude using minimum RCS thruster
; firing to counteract disturbances.
;
; After automatic maneuvers during Apollo 11's mission, this routine ensured
; the LM maintained precise attitudes for critical operations like rendezvous
; radar tracking, photography of the lunar surface, and preparation for
; powered descent initiation. The Kalman steering strategy minimized fuel
; consumption throughout the maneuver, preserving RCS propellant margins.
;
; The routine restores the calling program's priority and returns control
; through SPVAC, allowing mission programs to resume normal operations with
; the spacecraft now stabilized in the commanded orientation.
; ============================================================================

MANUSTOP	CAF	ZERO		# ZERO MANEUVER RATES
		TS	DELDCDU2	# Zero roll axis incremental angle
		TS	OMEGARD		# Zero roll rate command
		TS	DELREROR	# Zero roll error offset
		TS	DELDCDU1	# Zero pitch axis incremental angle
		TS	OMEGAQD		# Zero pitch rate command
		TS	DELQEROR	# Zero pitch error offset
		CA	CPSI		# SET DESIRED GIMBAL ANGLES TO
		TS	CDUZD		# DESIRED FINAL GIMBAL ANGLES (yaw)
		CA	CTHETA		# Load final pitch angle
		TS	CDUYD		# Store as desired pitch CDU angle
ENDROLL		CA	CPHI		# Load final roll angle (NO FINAL YAW)
		TS	CDUXD		# Store as desired roll CDU angle
		CAF	ZERO
		TS	OMEGAPD		# Zero yaw rate (maneuver avoided gimbal lock)
		TS	DELDCDU		# Zero yaw incremental angle
		TS	DELPEROR	# Zero yaw error offset
GOODMANU	CA	ATTPRIO		# RESTORE USERS PRIO
		TS	NEWPRIO		# Restore calling program priority

		CA	ZERO		# ZERO ATTCADR
		DXCH	ATTCADR		# Clear attitude routine address

		TC	SPVAC		# RETURN TO USER (release VAC to caller)

		TC	TASKOVER	# End of maneuver termination task
