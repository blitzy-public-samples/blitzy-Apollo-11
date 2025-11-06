# Copyright:	Public domain.
# Filename:	GROUND_TRACKING_DETERMINATION_PROGRAM.agc
# Purpose:	Part of the source code for Comanche, build 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 456-459
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Mod history:	2009-05-07 OH	Transcription Batch 1 Assignment
#		2009-05-20 RSB	Corrected a couple of DIMOFLAG to DIM0FLAG.
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  April 1, 1969.
#
#	This AGC program shall also be referred to as Colossus 2A
#
#	Prepared by
#			Massachusetts Institute of Technology
#			75 Cambridge Parkway
#			Cambridge, Massachusetts
#
#	under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further information.
# Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: GROUND_TRACKING_DETERMINATION_PROGRAM.agc
; MODULE: COMEKISS Subsystem (Orbital Navigation)
; MISSION PHASE: all-phases
;
; TL;DR: Ground tracking determination program (P21) computes spacecraft ground
;        track position without requiring ground communication. Astronauts request
;        via DSKY to display latitude, longitude, and altitude for either CSM or
;        LM at specified times during cislunar coast and lunar orbit phases.
;
; COMMENT-ONLY READERS: This program allowed astronauts to see where their
;        spacecraft was positioned over Earth or Moon at any given time without
;        needing to radio Mission Control for tracking information.
; CODE-ALONG READERS: Study orbital state vector integration, coordinate frame
;        transformations (inertial to rotating), LAT-LONG computation algorithms,
;        and DSKY display interface for navigation data presentation.
; ============================================================================
;
; GROUND TRACKING MEASUREMENTS vs GROUND TRACK DISPLAY:
;
; This file's name "GROUND_TRACKING_DETERMINATION_PROGRAM" can be misleading.
; It does NOT process ground tracking measurements (Doppler shift, range data)
; from MSFN ground stations. That processing occurs in MEASUREMENT_INCORPORATION.agc.
;
; Instead, P21 DISPLAYS the spacecraft's ground track (latitude/longitude/altitude
; projection onto Earth or Moon surface) using navigation state already updated by
; measurement incorporation. The distinction:
;
; GROUND TRACKING (measurement processing):
;   - Doppler shift analysis from S-band communication signals reveals velocity
;   - Range measurements from ground radar reveal distance to spacecraft  
;   - MSFN (Manned Space Flight Network) stations worldwide provide tracking data
;   - MEASUREMENT_INCORPORATION.agc processes these to refine navigation state
;   - Kalman filtering combines measurements to improve position/velocity knowledge
;
; GROUND TRACK (position display - THIS PROGRAM):
;   - Reads current navigation state vector (position, velocity) from memory
;   - Integrates orbital motion forward/backward to requested time
;   - Converts inertial position to rotating body frame coordinates
;   - Computes latitude, longitude, altitude above surface
;   - Displays result on DSKY for astronaut situational awareness
;
; P21's value: Crew independence from ground communication for position knowledge.
; During trans-lunar coast, lunar orbit, and trans-earth return, P21 helps with:
; - Photography planning (knowing which surface features will be visible)
; - Communication window prediction (which ground stations will have line-of-sight)
; - Flight plan verification (confirming orbital parameters match expected values)
; - Landing site tracking (monitoring approach to Apollo 11's Sea of Tranquility target)
;
; Technical data flow: Ground stations → Measurement incorporation (Doppler/range) →
; Navigation state update → P21 reads state → Orbital integration → Coordinate
; transformation → LAT-LONG computation → DSKY display to crew.
; ============================================================================

# Page 456
; ============================================================================
; GROUND TRACKING DETERMINATION PROGRAM P21
;
; During Apollo 11's journey to the Moon and back, the crew could request this
; program to display where their spacecraft was positioned relative to the
; surface below - whether flying over Earth or orbiting the Moon. This
; eliminated the need to ask Mission Control "Where are we?" via radio.
;
; The program integrates the orbital state vector forward (or backward) in time
; to compute position and velocity at the requested moment, then transforms
; from inertial coordinates to the rotating body frame to determine the ground
; track point directly beneath the spacecraft.
; ============================================================================

# GROUND TRACKING DETERMINATION PROGRAM P21
# PROGRAM DESCRIPTION
# MOD NO - 1
# MOD BY - N.M.NEVILLE
# FUNCTIONAL DECRIPTION-
#
# TO PROVIDE THE ASTRONAUT DETAILS OF THE LM OR CSM GROUND TRACK WITHOUT
# THE NEED FOR GROUND COMMUNICATION (REQUESTED BY DSKY).
# CALLING SEQUENCE -
#
# ASTRONAUT REQUEST THROUGH DSKY V37E21E
# SUBROUTINES CALLED-
#
# GOPERF4
# GOFLASH
# THISPREC
# OTHPREC
# LAT-LONG
# NORMAL EXIT MODES-
#
# ASTRONAUT REQUEST THROUGH DSKY TO TERMINATE PROGRAM V34E
# ALARM OR ABORT EXIT MODES-
#
# NONE
# OUTPUT -
#
# OCTAL DISPLAY OF OPTION CODE AND VEHICLE WHOSE GROUND TRACK IS TO BE
# COMPUTED
#	OPTION CODE	00002
#	THIS		00001
#	OTHER		00002
# DECIMAL DISPLAY OF TIME TO BE INTEGRATED TO HOURS , MINUTES , SECONDS
# DECIMAL DISPLAY OF LAT,LONG,ALT
# ERASABLE INITIALIZATION REQUIRED
#
# AX0		2DEC	4.652459653 E-5		RADIANS		"68-69 CONSTANTS"
#
# -AY0		2DEC	2.147535898 E-5		RADIANS
#
# AZ0		2DEC	.7753206164		REVOLUTIONS
# FOR LUNAR ORBITS 504LM VECTOR IS NEEDED
#
# 504LM		2DEC	-2.700340600 E-5	RADIANS
#
# 504LM _2	2DEC	-7.514128400 E-4	RADIANS
#
# 504LM _4	2DEC	_2.553198641 E-4	RADIANS
#
# NONE
# DEBRIS

# Page 457
# CENTRALS-A,Q,L
# OTHER-THOSE USED BY THE ABOVE LISTED SUBROUTINES
# SEE LEMPREC,LAT-LONG
		SBANK=	LOWSUPER	# FOR LOW 2CADR'S.

		BANK	33
		SETLOC	P20S
		BANK

		EBANK=	P21TIME
		COUNT	24/P21

; ============================================================================
; PROGRAM 21 ENTRY POINT
;
; When the astronaut enters V37E21E on the DSKY keyboard, this routine begins.
; The program first asks which vehicle's ground track to display - this
; spacecraft (CSM) or the other vehicle (LM). During Apollo 11's mission, this
; allowed tracking both Columbia and Eagle independently while separated.
;
; Technical: OPTION2 register holds vehicle selection (1=LM, 2=CSM). GOPERF4
; displays the option and awaits crew input via DSKY. Crew can PROCEED with
; default or enter different value through R2 register.
; ============================================================================

PROG21		CAF	ONE
		TS	OPTION2		# ASSUMED VEHICLE IS LM , R2 = 00001
		CAF	BIT2		#  OPTION 2
		TC	BANKCALL
		CADR	GOPERF4
		TC	GOTOPOOH	# TERMINATE
		TC	+2		# PROCEED VALUE OF ASSUMED VEHICLE OK
		TC	-5		# R2 LOADED THROUGH DSKY
; ============================================================================
; TIME INPUT REQUEST
;
; The crew now enters the time for which they want to know the ground track
; position. For example, during translunar coast, Armstrong could enter a time
; several hours ahead to see where they would be crossing over the Moon's
; surface during approach. Times are entered in hours, minutes, and seconds.
;
; Technical: V6N34 displays verb 06 (display decimal), noun 34 (time hours/min/sec).
; GOFLASH requests crew input. Time value loaded into DSPTEM1, later transferred
; to TDEC1 for the integration routine. Crew can PROCEED, TERMINATE, or enter
; new time value via DSKY.
; ============================================================================

P21PROG1	CAF	V6N34		# LOAD DESIRED TIME OF LAT-LONG.
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH	# TERM
		TC	+2		# PROCEED VALUES OK
		TC	-5		# TIME LOADED THROUGH DSKY
		TC	INTPRET
; ============================================================================
; ORBITAL INTEGRATION SETUP
;
; The program now integrates the spacecraft's orbit to the requested time.
; Starting from the current known position and velocity, the AGC computes where
; the spacecraft will be (or was) at the specified moment by numerically
; integrating the equations of motion including gravitational effects.
;
; On first use, the program calculates a "base vector" - a reference state at
; the requested time. On subsequent cycles (pressing V32E to recycle), it reuses
; this base vector for efficiency, updating from there rather than starting over.
;
; Technical: TDEC1 holds target time for integration. INTSTALL initializes
; integration parameters. P21FLAG indicates recycle mode (reuse base vector).
; VINTFLAG selects integration type. OPTION2 determines this vehicle (0) or
; other vehicle (1). DIM0FLAG and INTYPFLG control integration precision.
; INTEGRV performs the orbital state vector integration from current state to
; target time, accounting for gravitational perturbations and body rotation.
; ============================================================================

		DLOAD
			DSPTEM1
		STCALL	TDEC1		# INTEG TO TIME SPECIFIED IN TDEC
			INTSTALL
		BON	SET
			P21FLAG
			P21CONT		# ON...RECYCLE USING BASE VECTOR
			VINTFLAG	# OFF..1ST PASS CALC BASE VECTOR
		SLOAD	SR1
			OPTION2
		BHIZ	CLEAR
			+2		# ZERO..THIS VEHICLE (CM)
			VINTFLAG	# ONE...OTHER VEHICLE(LM)
		CLEAR	CLEAR
			DIM0FLAG
			INTYPFLG	# PRECISION
		CALL
			INTEGRV		# CALCULATE
		GOTO			# .AND
			P21VSAVE	# ..SAVE BASE VECTOR
; ============================================================================
; BASE VECTOR RECYCLE LOGIC
;
; When the astronaut presses V32E to recycle (request another ground track
; calculation), the program efficiently reuses the previously calculated
; position and velocity at the target time. This saved "base vector" becomes
; the starting point for integration to any new requested time, avoiding
; redundant computation from the current vehicle state.
;
; The program also determines whether the spacecraft is in Earth or lunar
; orbit by checking P21ORIG (0=Earth, 2=Moon) and sets MOONFLAG accordingly
; to select appropriate gravitational models for integration.
;
; Technical: P21BASER and P21BASEV hold the previously calculated position and
; velocity vectors. RCV and VCV are integration working registers. TET is the
; integration epoch time. P21TIME stores the base vector timestamp. INTEGRVS
; performs integration from the saved base state using either Earth-centered
; or Moon-centered reference frame based on MOONFLAG.
; ============================================================================

P21CONT		VLOAD			# RECYCLE..INTEG FROM BASE VECTOR
			P21BASER
# Page 458
		STOVL	RCV		# ..POS
			P21BASEV
		STODL	VCV		# ..VEL
			P21TIME
		STORE	TET		# ..TIME
		CLEAR	CLEAR
			DIM0FLAG
			MOONFLAG
		SLOAD	BZE
			P21ORIG
			+3		# ZERO = EARTH
		SET			# ...2 = MOON
			MOONFLAG
		CALL
			INTEGRVS
; ============================================================================
; SAVE BASE VECTOR AND COMPUTE VELOCITY PARAMETERS
;
; After integration completes, this section saves the resulting state vector
; as the new "base vector" for future recycle operations. Additionally, it
; computes the velocity magnitude and flight path angle (gamma), which
; describe the spacecraft's trajectory geometry at the target time.
;
; Flight path angle is the angle between position vector and velocity vector.
; When gamma is positive, the spacecraft is climbing (moving away from body);
; when negative, it's descending (moving toward body). At orbital apogee or
; perigee, gamma equals zero (velocity perpendicular to position).
;
; Technical: TAT contains integration result time. RATT1 and VATT1 contain
; integrated position and velocity vectors (scaling B-29/B-27 for position,
; B-7/B-5 for velocity depending on Earth/Moon). ABVAL computes velocity
; magnitude. UNIT creates unit position vector, DOT computes dot product with
; velocity, DDV divides by velocity magnitude, ASIN computes flight path angle.
; P21GAM stores gamma in radians (range -90° to +90°). P21ORIG and P21FLAG
; saved for next cycle.
; ============================================================================

P21VSAVE	DLOAD			# SAVE CURRENT BASE VECTOR
			TAT
		STOVL	P21TIME		# ..TIME
			RATT1
		STOVL	P21BASER	# ..POS B-29 OR B-27
			VATT1
		STORE	P21BASEV	# ..VEL B-7  OR B-5
		ABVAL	SL*
			0,2
		STOVL	P21VEL		# /VEL/ FOR N73 DSP
			RATT
		UNIT	DOT
			VATT		# U(R).(V)
		DDV	ASIN		# U(R).U(V)
			P21VEL
		STORE	P21GAM		# SIN-1 U(R).U(V), -90 TO +90
		SXA,2	SET
			P21ORIG		# 0 = EARTH  2 = MOON
			P21FLAG
; ============================================================================
; GROUND TRACK DISPLAY DATA GENERATION
;
; This section converts the integrated position vector into ground track
; coordinates: latitude, longitude, and altitude. The astronaut sees these
; displayed on the DSKY to understand where the spacecraft is (or will be)
; relative to the surface below.
;
; For Earth orbits, latitude/longitude reference Earth's equator and prime
; meridian. For lunar orbits, they reference the Moon's equator and prime
; meridian. Altitude is height above the reference body's mean radius.
;
; The display uses Verb 06 Noun 43 format, showing all three coordinates
; in decimal. Latitude ranges from -90° (south pole) to +90° (north pole).
; Longitude ranges from -180° to +180° (or 0° to 360°). Altitude displays
; in units of 100 meters for convenient readability.
;
; Technical: LUNAFLAG selects Earth (clear) or Moon (set) coordinate system.
; ALPHAV receives position vector for coordinate conversion. TAT provides
; time for LAT-LONG subroutine which handles body rotation. ERADFLAG controls
; radius calculation. LAT-LONG computes geodetic coordinates. Altitude scaled
; by K.01 (divide by 100) for display units. V06N43 triggers decimal display
; of the three values via GOFLASH routine.
; ============================================================================

P21DSP		CLEAR	SLOAD		# GENERATE DISPLAY DATA
			LUNAFLAG
			X2
		BZE	SET
			+2		# 0 = EARTH
			LUNAFLAG
		VLOAD
			RATT
		STODL	ALPHAV
			TAT
		CLEAR	CALL
			ERADFLAG
			LAT-LONG
		DMP			# MPAC = ALT, METERS B-29
			K.01
		STORE	P21ALT		# ALT/100 FOR N73 DSP
# Page 459
		EXIT
		CAF	V06N43		# DISPLAY LAT,LONG,ALT
		TC	BANKCALL	# LAT,LONG = REVS B0 BOTH EARTH/MOON
		CADR	GOFLASH		# ALT = METERS B-29  BOTH EARTH/MOON
		TC	GOTOPOOH	# TERM
; ============================================================================
; RECYCLE OPERATION - V32E (PROCEED)
;
; After viewing the ground track display, the astronaut has three options:
;   V32E (PROCEED) - Recycle to compute ground track 10 minutes later
;   V33E (TERMINATE) - Exit program and return to POO (idle)
;   Enter new time - Compute ground track at a different specified time
;
; When the astronaut presses V32E to proceed, the program automatically
; advances the target time by 10 minutes (600 seconds) from the previously
; displayed time, then loops back to compute and display the new ground track.
; This allows quick successive views of the orbital ground track progression
; without manual time entry.
;
; Technical: First TC GOTOPOOH handles terminate (V33E). Second TC GOTOPOOH
; handles entry of new time. INTPRET enters interpretive mode for recycle.
; P21TIME contains previous display time. DAD adds 600SEC (10 minutes).
; Result stored in DSPTEM1. RTB returns to P21PROG1 to display and process
; the new target time, utilizing the saved base vector for efficiency.
; ============================================================================

		TC	GOTOPOOH
		TC	INTPRET		# V32E RECYCLE
		DLOAD	DAD
			P21TIME
			600SEC		# 600 SECONDS OR 10 MIN
		STORE	DSPTEM1
		RTB
			P21PROG1

; ============================================================================
; PROGRAM CONSTANTS
;
; 600SEC: Time increment for V32E recycle operation (10 minutes = 600 seconds).
;         Scaled in centiseconds (B-28 scaling: 1 unit = 0.01 seconds).
;         Value 60000 represents 600.00 seconds when properly scaled.
;         This constant determines how far ahead the ground track advances
;         each time the astronaut presses PROCEED during display.
;
; P21ONENN: Option code table for vehicle selection display (Noun 34).
;           OCT 00001 = "This vehicle" (CSM when running in CM computer)
;           OCT 00000 = Displayed to astronaut for confirmation
;           Used in GOPERF4 to show which vehicle's ground track will be computed.
;
; V06N43: DSKY display format code for ground track coordinates.
;         Verb 06 = Decimal display
;         Noun 43 = Latitude, Longitude, Altitude
;         All three values displayed simultaneously in decimal format for
;         intuitive astronaut interpretation of the spacecraft ground track.
;
; V6N34: DSKY display format code for option selection.
;        Verb 06 = Decimal display
;        Noun 34 = Option code and vehicle identifier
;        Displays option code (00002) and vehicle selection (00001 or 00002)
;        at program start for astronaut confirmation.
;
; K.01: Altitude scaling factor for display conversion. Divides altitude
;       (computed in meters) by 100 to display in units of 100 meters
;       (hectometers). This provides convenient magnitude for DSKY display
;       without excessive digits. Example: 185,000 meters altitude displays
;       as 1850 (representing 185 km).
; ============================================================================

600SEC		2DEC	60000		# 10 MIN

P21ONENN	OCT	00001		# NEEDED TO DETERMINE VEHICLE
		OCT	00000		# TO BE INTEGRATED
V06N43		VN	00643
V6N34		VN	00634
K.01		2DEC	.01

