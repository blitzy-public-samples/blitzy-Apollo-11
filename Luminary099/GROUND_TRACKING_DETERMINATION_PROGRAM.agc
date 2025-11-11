# Copyright:	Public domain.
# Filename:	GROUND_TRACKING_DETERMINATION_PROGRAM.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	654-657
# Mod history:	2009-05-18 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
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
; FILE: GROUND_TRACKING_DETERMINATION_PROGRAM.agc
; MODULE: Ground Tracking Navigation
; MISSION PHASE: lunar-orbit/trans-lunar/trans-earth
;
; TL;DR: Implements Program P21 which provides astronauts with ground track
;        position (latitude, longitude, altitude) for either the Lunar Module
;        or Command Module without requiring ground communication. Integrates
;        orbital state vector to crew-specified time and displays spacecraft
;        position over Earth or Moon surface, enabling autonomous navigation
;        awareness during cislunar coast and lunar orbit phases.
;
; COMMENT-ONLY READERS: This program allowed the Apollo 11 crew to know where
;        they were over the lunar surface during orbital operations, providing
;        navigational independence from Mission Control communications.
; CODE-ALONG READERS: Study the integration of orbital mechanics computation
;        (INTEGRV/INTEGRVS), coordinate transformation (LAT-LONG), and DSKY
;        crew interface to understand autonomous navigation capability.
; ============================================================================

# Page 654
; ============================================================================
; GROUND TRACKING DETERMINATION PROGRAM P21
;
; This program allows astronauts to determine the spacecraft's ground track
; (the path traced on the surface of Earth or Moon directly below the vehicle)
; without needing to communicate with Mission Control. During Apollo 11's
; lunar orbit operations, the crew could use P21 to see which lunar landmarks
; were passing beneath them, aiding in visual navigation and crew awareness.
; ============================================================================

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
# ASTRONAUT REQUEST TROUGH DSKY TO TERMINATE PROGRAM V34E
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
# AXO		2DEC	4.652459653 E-5		RADIANS		%68-69 CONSTANTS"
#
# -AYO		2DEC	2.147535898 E-5		RADIANS
#
# AZO		2DEC	.7753206164		REVOLUTIONS
# FOR LUNAR ORBITS 504LM VECTOR IS NEEDED
#
# 504LM		2DEC	-2.700340600 E-5	RADIANS
#
# 504LM	_2	2DEC	-7.514128400 E-4	RADIANS
#
# 504LM	_4	2DEC	_2.553198641 E-4	RADIANS
#
# NONE
# DEBRIS
#
# Page 655
# CENTRALS-A,Q,L
# OTHER-THOSE USED BY THE ABOVE LISTED SUBROUTINES
# SEE LEMPREC,LAT-LONG
; ============================================================================
; PROGRAM P21 INITIALIZATION
;
; The crew initiates this program by keying V37E21E on the DSKY. The program
; first assumes the astronaut wants to track the current vehicle (LM), but
; offers the option to track the other vehicle (CSM) instead. This was
; particularly useful during Apollo 11's lunar orbit when Columbia and Eagle
; were separated, allowing each crew to monitor both spacecraft positions.
; ============================================================================

		SBANK=	LOWSUPER	# FOR LOW 2CADR'S.

		BANK	33
		SETLOC	P20S
		BANK

		EBANK=	P21TIME
		COUNT*	$$/P21
		
; Program entry point for P21 Ground Tracking Determination.
; Sets default vehicle selection to "this vehicle" (LM when running on LM AGC).
PROG21		CAF	ONE
		TS	OPTION2		# ASSUMED VEHICLE IS LM , R2 = 00001
		
; Display vehicle selection option to crew on DSKY.
; Crew can accept default or enter 2 to track the other vehicle (CSM).
		CAF	BIT2		#  OPTION 2
		TC	BANKCALL
		CADR	GOPERF4
		TC	GOTOPOOH	# TERMINATE
		TC	+2		# PROCEED VALUE OF ASSUMED VEHICLE OK
		TC	-5		# R2 LOADED THROUGH DSKY

; ============================================================================
; TIME SELECTION AND STATE VECTOR INTEGRATION
;
; The program now asks the crew to specify the time for which they want to
; know the ground track position. The AGC will integrate the orbital state
; vector forward (or backward) to that time, then compute where the spacecraft
; will be over the surface. This allowed the Apollo 11 crew to plan ahead,
; knowing which lunar features would be visible at specific times during
; future orbits.
; ============================================================================

; Prompt crew to enter desired time for ground track computation.
; Time entered as hours:minutes:seconds on DSKY using Verb 06 Noun 34.
P21PROG1	CAF	V6N34		# LOAD DESIRED TIME OF LAT-LONG.
		TC	BANKCALL
		CADR	GOFLASH
		TC	GOTOPOOH	# TERM
		TC	+2		# PROCEED VALUES OK
		TC	-5		# TIME LOADED THROUGH DSKY
		
; Enter interpretive mode for high-level vector/matrix operations.
; The integration and coordinate transformation require double-precision
; arithmetic and vector operations provided by the interpreter.
		TC	INTPRET
		
; Load crew-specified target time from DSKY display buffer.
		DLOAD
			DSPTEM1
		STCALL	TDEC1		# INTEGRATE TO TIME SPECIFIED IN TDEC
			INTSTALL
			
; Check if this is first pass or recycle computation.
; On first pass, calls appropriate state vector routine (THISPREC or OTHPREC).
; On recycle, uses previously computed base vector for faster integration.
		BON	CLEAR
			P21FLAG
			P21CONT		# ON---RECYCLE USING BASE VECTOR
			VINTFLAG	# OFF--IST PASS CALL BASE VECTOR
			
; Determine which vehicle's state vector to use based on crew selection.
; OPTION2 = 0 means this vehicle (LM), OPTION2 = 1 means other vehicle (CSM).
		SLOAD	SR1
			OPTION2
		BHIZ	SET
			+2		# ZERO--THIS VEHICLE(LM)
			VINTFLAG	# ONE--OTHER VEHICLE(CM)
			
; Set integration flags for precision orbital propagation.
; DIM0FLAG controls coordinate system, INTYPFLG selects precision integration.
		CLEAR	CLEAR
			DIM0FLAG
			INTYPFLG	# PRECISION
			
; Call precision orbital integration routine INTEGRV.
; This propagates position and velocity from current time to target time
; using Encke method for numerical precision over long integration periods.
		CALL
			INTEGRV		# CALCULATE
		GOTO			# -AND
			P21VSAVE	# -SAVE BASE VECTOR
; ============================================================================
; RECYCLE PATH - Use Saved Base Vector for Faster Integration
;
; On subsequent ground track requests, the program can integrate from a
; previously saved "base vector" rather than starting from current time.
; This optimizes computation time when the crew wants multiple ground track
; positions during the same orbit.
; ============================================================================

P21CONT		VLOAD
			P21BASER	# RECYCLE--INTEG FROM BASE VECTOR
		STOVL	RCV		# --POS
# Page 656
			P21BASEV
		STODL	VCV		# --VEL
			P21TIME
		STORE	TET		# --TIME
			
; Determine whether integrating around Earth or Moon.
; During translunar and transearth coast, Earth origin is used.
; During lunar orbit (like Apollo 11 in Columbia/Eagle), Moon origin is used.
		CLEAR	CLEAR
			DIM0FLAG
			MOONFLAG
		SLOAD	BZE
			P21ORIG
			+3		# ZERO=EARTH
		SET			# ---2=MOON
			MOONFLAG
			
; Call short-term precision integration routine INTEGRVS.
; This uses the base vector as starting point, reducing computation time
; compared to full integration from current state.
 +3		CALL
			INTEGRVS
			
; Save the newly computed state vector as the base vector for next cycle.
; Stores time, position, and velocity for efficient recycle path entry.
P21VSAVE	DLOAD			# SAVE CURRENT BASEVECTOR
			TAT
		STOVL	P21TIME		# --TIME
			RATT1
		STOVL	P21BASER	# --POS B-29 OR B-27
			VATT1
		STORE	P21BASEV	# --VEL B-07 OR B-05
		
; ============================================================================
; VELOCITY AND FLIGHT PATH ANGLE COMPUTATION
;
; Calculate magnitude of velocity vector and flight path angle (angle between
; position and velocity vectors). Flight path angle indicates whether spacecraft
; is ascending away from or descending toward the surface. During Apollo 11's
; lunar orbit, this angle oscillated as Eagle and Columbia followed elliptical
; paths, providing crew with immediate trajectory status information.
; ============================================================================

; Compute absolute velocity magnitude (speed) for display.
; Velocity vector scaled B-7 (Earth) or B-5 (Moon) depending on MOONFLAG.
		ABVAL	SL*
			0,2
		STOVL	P21VEL		# VEL/ FOR N91 DISP
		
; Calculate flight path angle using dot product of unit vectors.
; Flight path angle = arcsin(unit(R) · unit(V))
; Positive angle means climbing away from surface, negative means descending.
			RATT
		UNIT	DOT
			VATT		#  U(R).V
		DDV	ASIN		# U(R).U(V)
			P21VEL
		STORE	P21GAM		# SIN-1 U(R).U(V) , -90 TO &90
		
; Determine display path based on vehicle selection and surface flag.
; Checks whether tracking this vehicle or other vehicle, and whether
; on surface or in flight.
		SXA,2	SLOAD
			P21ORIG		# 0=EARTH
			OPTION2
		SR1	BHIZ
			+3
		GOTO
			+4
 +3		BON
			SURFFLAG
			P21DSP
 +4		SET
			P21FLAG

; ============================================================================
; COORDINATE TRANSFORMATION TO LATITUDE/LONGITUDE/ALTITUDE
;
; The LAT-LONG subroutine transforms the inertial state vector (position and
; velocity in reference coordinates) into geographic coordinates meaningful
; to the crew: latitude, longitude, and altitude above the surface. During
; Apollo 11's lunar orbit, this allowed Armstrong, Aldrin, and Collins to
; correlate DSKY readings with visible surface features, enhancing situational
; awareness when visual navigation was needed.
; ============================================================================

P21DSP		CLEAR	SLOAD		# GENERATE DISPLAY DATA
			LUNAFLAG
			X2
			
; Set planet flag based on origin (Earth or Moon).
; LUNAFLAG clear = Earth reference, LUNAFLAG set = Moon reference.
; This determines which planetary radius and rotation rate to use.
		BZE	SET
			+2		# 0 = EARTH
			LUNAFLAG
			
; Load position vector and time for coordinate transformation.
; ALPHAV receives position vector, TAT provides time reference.
		VLOAD
			RATT
# Page 657
		STODL	ALPHAV
			TAT
			
; Call LAT-LONG subroutine to compute geographic coordinates.
; Returns: latitude (degrees north/south of equator, ±90°)
;          longitude (degrees east/west, 0°-360°)
;          altitude (meters above reference surface)
		CLEAR	CALL
			ERADFLAG
			LAT-LONG
			
; Scale altitude for display.
; Convert meters to kilometers by multiplying by 0.01 (dividing by 100).
; During Apollo 11's lunar orbit, typical altitudes were 60-110 km.
		DMP			# MPAC = ALT, METERS B-29
			K.01
		STORE	P21ALT		# ALT/100 FOR N91 DISP

; ============================================================================
; DISPLAY GROUND TRACK POSITION TO CREW
;
; Exit interpreter mode and display the computed ground track data on DSKY
; using Verb 06 Noun 43 (decimal display of latitude, longitude, altitude).
; The crew can then choose to terminate the program, or proceed to enter
; another time for a new ground track computation (recycle mode).
;
; During Apollo 11's lunar orbit, the crew used this to identify when they
; would pass over specific landing sites, monitoring their position relative
; to the Sea of Tranquility target area. This autonomous navigation capability
; was critical backup to ground tracking during loss-of-signal periods behind
; the Moon.
; ============================================================================

		EXIT
		CAF	V06N43		# DISPLAY LAT, LONG, ALT
		TC	BANKCALL	# LAT, LONG = 1/2 REVS B0
		CADR	GOFLASH		# ALT = KM B14
		TC	GOTOPOOH	# TERM
		TC	GOTOPOOH

; ============================================================================
; RECYCLE LOGIC - Compute Ground Track for Another Time
;
; If crew presses PROCEED (V32E), program increments time by 10 minutes
; (600 seconds) and returns to time display, allowing rapid calculation
; of multiple ground track positions. This was valuable during lunar orbit
; when the crew wanted to preview which surface features would pass beneath
; them at future times in the same orbit.
; ============================================================================

		TC	INTPRET		# V32E RECYCLE
		
; Increment target time by 10 minutes for next ground track prediction.
; 600 seconds chosen to provide reasonable spacing for orbital preview.
		DLOAD	DAD
			P21TIME
			600SEC		# 600 SECONDS OR 10 MIN
		STORE	DSPTEM1
		RTB
			P21PROG1

; ============================================================================
; PROGRAM CONSTANTS
;
; 600SEC:  Time increment for recycle mode (10 minutes = 600 seconds)
;          Scaled as 2DEC 60000 (representing 600.00 seconds in centiseconds)
;
; V06N43:  DSKY verb/noun code for decimal display of latitude, longitude, 
;          altitude (Verb 06 = display decimal, Noun 43 = lat/long/alt)
;
; V6N34:   DSKY verb/noun code for display of time (hours, minutes, seconds)
;
; K.01:    Scaling constant 0.01 for converting meters to hectometers
;          (altitude display scaling factor)
; ============================================================================

600SEC		2DEC	60000		# 10 MIN

V06N43		VN	00643
V6N34		VN	00634
K.01		2DEC	.01
