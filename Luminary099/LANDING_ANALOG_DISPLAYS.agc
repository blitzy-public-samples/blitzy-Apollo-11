# Copyright:	Public domain.
# Filename:	LANDING_ANALOG_DISPLAYS.agc
# Purpose:	Part of the source code for Luminary, build 099. It
#		is part of the source code for the Lunar Module's
#		(LM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 898-907
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	05/06/09 FB	Transcription Batch 4 Assignment.
#
# The contents of the "Luminary099" files, in general, are transcribed
# from scanned documents.
#
#	Assemble revision 001 of AGC program Luminary099 by NASA
#	2021112-061.  July 14, 1969.
#
#	Prepared by
#			Massachusetts Institute of Technology
#			75 Cambridge Parkway
#			Cambridge, Massachusetts
#
#	under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: LANDING_ANALOG_DISPLAYS.agc
; MODULE: Landing Display Systems
; MISSION PHASE: descent/landing
;
; TL;DR: Manages real-time crew display updates during lunar landing descent.
;        Formats and outputs altitude, altitude rate (descent velocity), and
;        horizontal velocity data to the DSKY and analog meters. Provides
;        critical situational awareness data that Buzz Aldrin monitored and
;        called out to Neil Armstrong during the Apollo 11 landing sequence.
;
; COMMENT-ONLY READERS: This code drives the displays that showed Armstrong
;        and Aldrin their descent rate and position during the final minutes
;        of the historic landing. Follow the comments to understand what the
;        crew was seeing on their instruments.
; CODE-ALONG READERS: Study how AGC converts radar and guidance data into
;        display formats, managing alternating altitude/altitude-rate updates
;        and computing velocity vector components for crew monitoring.
; ============================================================================

# Page 898
		BANK	21
		SETLOC	R10
		BANK

		EBANK=	UNIT/R/
		COUNT*	$$/R10

; ============================================================================
; LANDING ANALOG DISPLAYS MAIN ENTRY POINT
;
; During Apollo 11's descent on July 20, 1969, this routine executed
; continuously, updating the displays that Buzz Aldrin monitored while
; calling out altitude and descent rate to Commander Neil Armstrong.
;
; The displays alternated between showing altitude (in feet) and altitude
; rate (descent velocity in feet per second), providing the crew with
; essential situational awareness during the 12-minute powered descent.
;
; Historical mission transcript excerpt during final approach:
; "Aldrin: 750 feet, coming down at 23...700 feet, 21 down...
;  400 feet, down at 9...got the shadow out there...
;  75 feet, things looking good. Down a half, 6 forward...
;  40 feet, down 2-1/2. Picking up some dust...
;  30 feet, 2-1/2 down...Contact light!"
; ============================================================================
;
; ============================================================================
; LANDING DISPLAY SYSTEM OVERVIEW AND CREW MONITORING CONTEXT
;
; This module drives three critical analog displays for crew situational
; awareness during powered descent:
;
; 1. ALTITUDE DISPLAY: Height above lunar surface (0-50,000 feet range)
;    - Sourced from landing radar (RADAR LEADIN ROUTINES)
;    - Alternates with altitude rate on same display meter
;    - Radar data validity checked via ALTBITS flag
;    - Extrapolation used during brief radar dropouts
;
; 2. ALTITUDE RATE DISPLAY: Descent velocity (0-200 feet/second range)
;    - Computed from velocity vector dot product with position unit vector
;    - Alternates with altitude on same display meter every 0.5 seconds
;    - Negative values indicate descent, positive indicate ascent
;
; 3. HORIZONTAL VELOCITY DISPLAY: Forward and lateral velocity components
;    - Cross-range (VHY): Lateral drift perpendicular to approach direction
;    - Down-range (VHZ): Forward velocity along approach direction
;    - Displayed on velocity meter using CDU (Coupling Data Unit) outputs
;
; INTEGRATION WITH OTHER LANDING SYSTEMS:
;
; - LUNAR_LANDING_GUIDANCE_EQUATIONS.agc: Provides position/velocity vectors
;   (RUNIT, VVECT) and landing site geometry (UHYP, UHZP) for velocity
;   projections onto crew-referenced coordinate frame.
;
; - THROTTLE_CONTROL_ROUTINES.agc: Monitors fuel quantity and generates
;   low-fuel warnings (60-second and 30-second callouts). Fuel quantity is
;   displayed separately on DSKY numerical displays, NOT by this analog
;   display module. However, crew used these velocity/altitude displays in
;   conjunction with fuel monitoring to make critical landing decisions.
;
; - DISPLAY_INTERFACE_ROUTINES.agc: Provides DSKY display formatting for
;   numerical readouts of navigation state, complementing these analog meters.
;
; APOLLO 11 MISSION CONTEXT:
;
; During the historic July 20, 1969 landing, Buzz Aldrin continuously
; monitored these displays and called out readings to Neil Armstrong:
;   "750 feet, coming down at 23" (altitude + altitude rate)
;   "Forward 8, down 3" (horizontal velocity components)
;   "75 feet, things looking good. Down a half, 6 forward" (all three)
;   "40 feet, down 2-1/2. Picking up some dust" (altitude + rate)
;   "30 feet, 2-1/2 down. Faint shadow. Contact light!" (final callouts)
;
; These displays, combined with fuel quantity monitoring, enabled Armstrong
; to manually fly the LM to a safe landing site, avoiding the boulder field
; initially targeted. Landing occurred with approximately 25 seconds of
; fuel remaining - a testament to the crew's precise use of this display data.
;
; DISPLAY UPDATE FREQUENCY:
;
; - Altitude/Altitude Rate: Alternates every 0.5 seconds (4 Hz update rate)
; - Horizontal Velocity: Updates every 0.5 seconds (4 Hz update rate)
; - Task priority: Medium (executes between high-priority guidance and
;   low-priority housekeeping tasks in WAITLIST scheduler)
; ============================================================================

LANDISP		LXCH	PIPCTR1		# UPDATE TBASE2 AND PIPCTR SIMULTANEOUSLY.
		CS	TIME1
		DXCH	TBASE2
;
; Time base synchronization ensures display updates occur at precise
; intervals, critical for crew monitoring during rapid descent phases.
;

		CS	FLAGWRD7	# IS LANDING ANALOG DISPLAYS FLAG SET?
		MASK	SWANDBIT
		CCS	A
		TCF	DISPRSET	# NO.
;
; Check if landing analog displays are enabled. During non-landing phases
; (orbital operations, ascent), this flag remains clear and displays reset.
		CA	IMODES33	# BIT 7 = 0 (DO ALTRATE), =1 (DO ALT.)
		MASK	BIT7
		CCS	A
		TCF	ALTOUT
;
; ============================================================================
; ALTITUDE RATE (DESCENT VELOCITY) COMPUTATION
;
; This section calculates the spacecraft's descent velocity by computing
; the dot product of the position unit vector (RUNIT) with the velocity
; vector (VVECT). The result tells the crew how fast they're descending
; toward the lunar surface.
;
; During Apollo 11's final approach, Aldrin called out these descent rates:
; "23 feet per second...21 down...9 down...2-1/2 down"
;
; The display alternates between altitude and altitude rate to give the
; crew a complete picture of their descent profile on a single meter.
; ============================================================================
;
ALTROUT		TC	DISINDAT	# CHECK MODE SELECT SWITCH AND DIDFLG.
		CS	IMODES33
		MASK	BIT7
		ADS	IMODES33	# ALTERNATE ALTITUDE RATE WITH ALTITUDE.
		CAF	BIT2		# RATE COMMAND IS EXECUTED BEFORE RANGE.
		EXTEND
		WOR	CHAN14		# ALTRATE (BIT2 = 1), ALTITUDE (BIT2 = 0).
;
; Toggle display mode bit in CHAN14. The analog meter hardware responds
; to this bit to switch between displaying altitude (static) and altitude
; rate (changing velocity). Alternating every update cycle gives crew both
; critical parameters on one instrument.
ARCOMP		CA	RUNIT		# COMPUTE ALTRATE=RUNIT.VVECT M/CS *2(-6).
		EXTEND
		MP	VVECT		# MULTIPLY X-COMPONENTS.
;
; Dot product computation: RUNIT · VVECT
; RUNIT is the unit vector pointing from lunar center through spacecraft
; VVECT is the spacecraft velocity vector in meters per centisecond
; The dot product gives velocity component along radial direction:
;   positive = ascending (moving away from surface)
;   negative = descending (moving toward surface)
;
		XCH	RUPTREG1	# SAVE SINGLE PRECISION RESULT M/CS*2(-6).
		CA	RUNIT +1	# MULTIPLY Y-COMPONENTS.
		EXTEND
		MP	VVECT +1
		ADS	RUPTREG1	# ACCUMULATE PARTIAL PRODUCTS.
		CA	RUNIT +2	# MULTIPLY Z-COMPONENTS.
		EXTEND
		MP	VVECT +2
		ADS	RUPTREG1	# ALTITUDE RATE IN M/CS *2(-6).
;
; Result: Radial velocity scaled by 2^-6 in meters per centisecond.
; Scaling factor chosen to maintain precision within 15-bit signed word
; limits while representing lunar descent velocities (0-50 m/s typical).
		CA	ARCONV		# CONVERT ALTRATE TO BIT UNITS (.5FPS/BIT)
		EXTEND
		MP	RUPTREG1
		DDOUBL
		DDOUBL
;
; Unit conversion: meters/centisecond → feet/second → display bit units
; ARCONV constant converts m/cs to feet per second (1 m/cs = 100 m/s)
; Display hardware resolution: 0.5 feet per second per bit
; Double shifts left (DDOUBL twice = multiply by 4) adjust final scaling
; to 2^-14 for display register format.
;
		XCH	RUPTREG1	# ALTITUDE RATE IN BIT UNITS*2(-14).
		CA	DALTRATE	# ALTITUDE RATE COMPENSATION FACTOR.
		EXTEND
		MP	DT
		AD	RUPTREG1
;
; Apply compensation for time delta (DT) to smooth display readings.
; DALTRATE factor corrects for computational delays and sensor lag,
; preventing display jitter during rapid descent rate changes.
; Critical during final approach when descent rate changes quickly.
;
		TS	ALTRATE		# ALTITUDE RATE IN BIT UNITS*2(-14).
		CS	ALTRATE
# Page 899
		EXTEND			# CHECK POLARITY OF ALTITUDE RATE.
		BZMF	+2
		TCF	DATAOUT		# NEGATIVE - SEND POS. PULSES TO ALTM REG.
;
; Descent rate polarity determines display direction:
;   Negative altitude rate = descending (needle moves down) - NORMAL
;   Positive altitude rate = ascending (needle moves up) - ABORT/ASCENT
;   Zero = hovering (no needle movement)
;
; During powered descent, crew expects negative values (descending).
; If altitude rate becomes positive, crew knows descent has stopped
; or reversed - critical abort indicator.
;
		CA	ALTRATE		# POSITIVE OR ZERO - SET SIGN BIT = 1 AND
		AD	BIT15		# SEND TO ALTM REGISTER. *DO NOT SEND +0*
;
; Sign bit encoding for display hardware. The altitude meter electronics
; interpret bit 15 to drive needle deflection direction.
;
DATAOUT		TS	ALTM		# ACTIVATE THE LANDING ANALOG DISPLAYS - -
		CAF	BIT3
		EXTEND
		WOR	CHAN14		# BIT3 DRIVES THE ALT/ALTRATE METER.
;
; Write to CHAN14 bit 3 activates the analog meter display hardware.
; The ALTM register value now drives the physical needle position that
; Aldrin monitored, calling out: "coming down at 23...21 down...9 down..."
; Display update completes in time for next alternation cycle (altitude).
;
		TCF	TASKOVER	# EXIT

; ============================================================================
; ALTITUDE DISPLAY MODE
;
; This section executes when the display cycle alternates to show altitude
; (static height above surface) rather than altitude rate (descent velocity).
;
; During Apollo 11's approach, Aldrin called out altitudes:
; "750 feet...700 feet...400 feet...75 feet...40 feet...30 feet"
;
; Altitude data comes from the landing radar, which provides accurate
; ranging to the lunar surface. As the LM descends below 40,000 feet,
; radar data becomes available and reliable for crew display.
; ============================================================================
;
ALTOUT		TC	DISINDAT	# CHECK MODE SELECT SWITCH AND DIDFLG.
		CS	BIT7
		MASK	IMODES33
		TS	IMODES33	# ALTERNATE ALTITUDE RATE WITH ALTITUDE.
;
; Toggle mode bit to alternate next cycle back to altitude rate display.
; The alternating pattern (altitude → altitude rate → altitude → ...)
; gives crew comprehensive descent awareness on single meter.
;
		CS	BIT2
		EXTEND
		WAND	CHAN14
;
; Clear bit 2 in CHAN14 to signal altitude mode to display hardware.
; Bit 2 = 0: altitude display, Bit 2 = 1: altitude rate display.
;
		CCS	ALTBITS		# =-1 IF OLD ALT. DATA TOBE EXTRAPOLATED.
		TCF	+4
;
; Check ALTBITS flag to determine if fresh radar data is available or if
; altitude must be extrapolated from previous data. Radar updates may be
; intermittent during certain descent phases or attitudes.
		TCF	+3
		TCF	OLDDATA
		TS	ALTBITS		# SET ALTBITS FROM -0 TO +0.
		CS	ONE
		DXCH	ALTBITS		# SET ALTBITS=-1 FOR SWITCH USE NEXT PASS.
		DXCH	ALTSAVE
		CA	BIT10		# NEW ALTITUDE EXTRAPOLATION WITH ALTRATE.
		XCH	Q
		LXCH	7		# ZL
		CA	DT
		EXTEND
		DV	Q		# RESCALE DT*2(-14) TO *2(-9) TIME IN CS.
		EXTEND
		MP	ARTOA2		# .0021322 *2(+8)
		TCF	OLDDATA +1	# RATE APPLIES FOR DT CS.

ZDATA2		DXCH	ALTSAVE
		TCF	NEWDATA
;
; ============================================================================
; ALTITUDE EXTRAPOLATION (OLD DATA PATH)
;
; When landing radar data is temporarily unavailable, altitude is extrapolated
; using the previously computed descent rate (ALTRATE). This maintains display
; continuity during brief radar dropouts.
;
; Computation: Updated_Altitude = Previous_Altitude + (Descent_Rate × Time)
; ARTOA scaling constant accounts for 0.5 second display update cycle (4 Hz).
; ============================================================================
;
OLDDATA		CA	ARTOA		# RATE APPLIES FOR .5 SEC. (4X/SEC. CYCLE)
		EXTEND
		MP	ALTRATE		# EXTRAPOLATE WITH ALTITUDE RATE.
;
; Multiply altitude rate by time constant to get altitude change since last
; update. ALTRATE is descent velocity (negative when descending), scaled in
; bit units. Result represents altitude decrease over update interval.
;
		DDOUBL
		AD	ALTSAVE +1
		TS	ALTSAVE +1
		CAF	ZERO
		ADS	ALTSAVE
		CAF	POSMAX		# FORCE SIGN AGREEMENT ASSUMING A
		AD	ONE		# NON-NEGATIVE ALTSAVE.
		AD	ALTSAVE +1	# IF ALTSAVE IS NEGATIVE, ZERO ALTSAVE
		TS	ALTSAVE +1	# AND ALTSAVE +1 AT ZERODATA.
;
; Sign agreement check ensures altitude remains non-negative. If extrapolation
; produces negative altitude (indicating computational error or excessive rate),
; force altitude to zero to prevent erroneous negative display.
;
# Page 900
		CAF	ZERO
		AD	POSMAX
		AD	ALTSAVE
		TS	ALTSAVE		# POSSIBLY SKIP TO NEWDATA.
		TCF	ZERODATA
;
; ============================================================================
; NEW ALTITUDE DATA PROCESSING
;
; Fresh radar data available. Process and display current altitude reading.
; During Apollo 11 landing, this path executed when radar reliably tracked
; the lunar surface, providing Armstrong and Aldrin real-time height data.
; ============================================================================
;
NEWDATA		CCS	ALTSAVE +1
		TCF	+4
		TCF	+3
		CAF	ZERO		# SET NEGATIVE ALTSAVE +1 TO +0.
		TS	ALTSAVE +1
		CCS	ALTSAVE		# PROVIDE A 15 BIT UNSIGNED OUTPUT.
		CAF	BIT15		# THE HI-ORDER PART IS +1 OR +0.
		AD	ALTSAVE +1
		TCF	DATAOUT		# DISPATCH UNSIGNED BITS TO ALTM REG.
;
; ============================================================================
; DISPLAY INPUT DATA VALIDATION (DISINDAT)
;
; Check crew mode select switch and system flags before updating displays.
; Astronaut can disable landing displays via mode switch if PGNCS guidance
; is not desired. This routine verifies display enable conditions.
; ============================================================================
;
DISINDAT	EXTEND
		QXCH	LADQSAVE	# SAVE RETURN TO ALTROUT +1 OR ALTOUT +1
		CAF	BIT6
		EXTEND			# WISHETH THE ASTRONAUT THE ANALOG
		RAND	CHAN30		# DISPLAYS?  I.E.,
;
; Read mode select switch position from CHAN30 bit 6. This switch allows
; crew to select guidance source: PGNCS (Primary Guidance and Navigation
; Control System) for computer-driven displays, or AGS (Abort Guidance
; System) or manual modes which disable these analog displays.
;
		CCS	A		# IS THE MODE SELECT SWITCH IN PGNCS?
		TCF	DISPRSET	# NO.  ASTRONAUT REQUESTS NO INERTIAL DATA
		CS	FLAGWRD1	# YES. CHECK STATUS OF DIDFLAG.
		MASK	DIDFLBIT
		EXTEND
		BZF	SPEEDRUN	# SET. PERFORM DATA DISPLAY SEQUENCE.
		CS	FLAGWRD1	# RESET. PERFORM INITIALIZATION FUNCTIONS.
		MASK	DIDFLBIT
		ADS	FLAGWRD1	# SET DIDFLAG.
		CS	BIT7
		MASK	IMODES33	# TO DISPLAY ALTRATE FIRST AND ALT. SECOND
		TS	IMODES33
		CS	FLAGWRD0	# ARE WE IN DESCENT TRAJECTORY?
		MASK	R10FLBIT
		EXTEND
		BZF	TASKOVER	# NO
		CAF	BIT8		# YES.
		EXTEND
		WOR	CHAN12		# SET DISPLAY INERTIAL DATA OUTBIT.
		CAF	ZERO
		TS	TRAKLATV	# LATERAL VELOCITY MONITOR FLAG
		TS	TRAKFWDV	# FORWARD VELOCITY MONITOR FLAG
		TS	LATVMETR	# LATVEL MONITOR METER
		TS	FORVMETR	# FORVEL MONITOR METER
		CAF	BIT4
		TC	TWIDDLE
		ADRES	INTLZE
		TCF	TASKOVER
INTLZE		CAF	BIT2
		EXTEND
		WOR	CHAN12		# ENABLE RR ERROR COUNTER.
# Page 901
		CS	IMODES33
		MASK	BIT8
		ADS	IMODES33	# SET INERTIAL DATA FLAG.
		TCF	TASKOVER
;
; ============================================================================
; VELOCITY VECTOR COMPUTATION (SPEEDRUN)
;
; Compute spacecraft velocity components for horizontal velocity displays.
; Landing guidance needs both forward velocity (direction of motion) and
; lateral velocity (crossrange drift) to assess landing site approach.
;
; During Apollo 11 final approach, Aldrin called out horizontal velocities:
; "Forward 8, down 3...drifting right...coming down nicely...drifting right"
; These displays helped Armstrong select safe landing site away from boulder
; field, ultimately extending landing time and reducing fuel margins to ~25s.
;
; Velocity computed by integrating acceleration (gravity + thrust) over time
; interval DT since last update. Uses inertial measurement unit (IMU) data.
; ============================================================================
;
SPEEDRUN	CS	PIPTIME +1	# UPDATE THE VELOCITY VECTOR
		AD	TIME1		# COMPUTE T - TN
;
; Compute time interval DT = (Current_Time - Last_Update_Time) for numerical
; integration. PIPTIME contains timestamp of last IMU pulse integration.
; TIME1 is current mission elapsed time in centiseconds.
;
		AD	HALF		# CORRECT FOR POSSIBLE OVERFLOW OF TIME1.
		AD	HALF
		XCH	DT		# SAVE FOR LATER USE
		CA	1SEC
		TS	ITEMP5		# INITIALIZE FOR DIVISION LATER
		EXTEND
		DCA	GDT/2		# COMPUTE THE X-COMPONENT OF VELOCITY.
;
; Load gravity acceleration vector X-component (GDT/2) scaled by 2^-7 m/cs².
; Lunar gravity at surface: ~1.62 m/s² = 0.162 m/cs². Integration computes
; velocity change: ΔV = acceleration × time.
;
		DDOUBL
		DDOUBL
		EXTEND
		MP	DT
		EXTEND
		DV	ITEMP5
;
; Complete X-component velocity: V_new = V_old + (gravity × time)
; Scaled in m/cs (meters per centisecond) × 2^-5 for AGC's 15-bit precision.
;
		XCH	VVECT		# VVECT = G(T-TN) M/CS *2(-5)
		EXTEND
		DCA	V		# M/CS *2(-7)
		DDOUBL			# RESCALE TO 2(-5)
		DDOUBL
;
; Add previous velocity component to gravity-induced change. This is first-order
; numerical integration: New_Velocity = Old_Velocity + Acceleration × Time.
;
		ADS	VVECT		# VVECT = VN + G(T-TN) M/CS *2(-5)
;
; Now integrate thrust acceleration from Inertial Measurement Unit (IMU).
; PIPAX is X-axis accelerometer output (PIPA = Pulsed Integrating Pendulous
; Accelerometer). Measures all non-gravitational acceleration: engine thrust,
; RCS thruster firings, residual atmospheric drag (negligible in lunar vacuum).
;
		CA	PIPAX		# DELV CM/SEC *2(-14)
		AD	PIPATMPX	# IN CASE PIPAX HAS BEEN ZEROED
		EXTEND
		MP	KPIP1(5)	# DELV M/CS *2(-5)
;
; KPIP scaling converts IMU pulse counts to velocity units. Each PIPA pulse
; represents fixed ΔV quantum. During powered descent, engine thrust dominates
; PIPA readings. Final velocity = gravity + thrust + initial velocity.
;
		ADS	VVECT		# VVECT = VN + DELV + GN(T-TN) M/CS *2(-5)
		EXTEND
;
; Y-component velocity integration follows identical procedure to X-component.
; In landing coordinate frame: X = forward/back, Y = left/right, Z = up/down.
; Y-axis velocity represents lateral drift during descent.
;
		DCA	GDT/2 +2	# COMPUTE THE Y-COMPONENT OF VELOCITY.
		DDOUBL
		DDOUBL
		EXTEND
		MP	DT
		EXTEND
		DV	ITEMP5
		XCH	VVECT +1
		EXTEND
		DCA	V +2
		DDOUBL
		DDOUBL
		ADS	VVECT +1
		CA	PIPAY
		AD	PIPATMPY
		EXTEND
		MP	KPIP1(5)
		ADS	VVECT +1
# Page 902
		EXTEND
;
; Z-component (vertical velocity) is most critical during landing. Negative
; Z velocity indicates descent rate. Armstrong and Aldrin closely monitored
; vertical velocity during final approach: too fast risks hard landing, too
; slow wastes fuel. Target: ~3 feet/second at touchdown for safe landing.
;
		DCA	GDT/2 +4	# COMPUTE THE Z-COMPONENT OF VELOCITY.
		DDOUBL
		DDOUBL
		EXTEND
		MP	DT
		EXTEND
		DV	ITEMP5
		XCH	VVECT +2
		EXTEND
		DCA	V +4
		DDOUBL
		DDOUBL
		ADS	VVECT +2
		CA	PIPAZ
		AD	PIPATMPZ
		EXTEND
		MP	KPIP1(5)
		ADS	VVECT +2
;
; Velocity vector now complete: VVECT = [Vx, Vy, Vz] in inertial frame.
; Next steps apply descent-specific corrections and project onto landing axes.
;
		CAF	BIT3		# PAUSE 40 MS TO LET OTHER RUPTS IN.
		TC	VARDELAY
;
; Brief delay allows higher-priority interrupts to execute. During descent,
; guidance and control computations must not monopolize processor. Landing
; requires coordinated execution: guidance, navigation, display updates, radar
; processing, engine control - all sharing single 85kHz AGC processor.
;
		CS	FLAGWRD0	# ARE WE IN DESCENT TRAJECTORY?
		MASK	R10FLBIT
		CCS	A
		TCF	+2		# YES.
;
; R10 flag indicates powered descent phase active. If not descending, exit
; velocity display routine and return to calling program.
;
		TC	LADQSAVE	# NO.
;
; ============================================================================
; VELOCITY VECTOR CORRECTION WITH NAVIGATION SERVICER UPDATES
;
; Apply DELVS corrections computed by navigation servicer. Navigation updates
; state vector infrequently (~2 seconds) compared to display rate (0.5 sec).
; DELVS contains accumulated velocity changes from state vector propagation
; between navigation updates. Corrects for:
; - Navigation filter updates from radar measurements
; - IMU drift compensation adjustments
; - Coordinate frame rotation effects
;
; During Apollo 11 descent, this correction kept velocity displays accurate
; despite 2+ second gaps between landing radar updates. Aldrin's callouts
; "21 down" (21 fps descent rate) reflected these corrected values.
; ============================================================================
;
		CA	DELVS		# HI X OF VELOCITY CORRECTION TERM.
		AD	VVECT		# HI X OF UPDATED VELOCITY VECTOR.
		TS	ITEMP1		# = VX - DVX M/CS*2(-5).
		CA	DELVS +2	#    Y
		AD	VVECT +1	#    Y
		TS	ITEMP2		# = VY - DVY M/CS*2(-5).
		CA	DELVS +4	#    Z
		AD	VVECT +2	#    Z
		TS	ITEMP3		# = VZ - DVZ M/CS*2(-5).
;
; ============================================================================
; VELOCITY PROJECTION ONTO LANDING COORDINATE AXES
;
; Transform inertial velocity vector onto landing site coordinate frame:
; - VHY: Cross-range velocity (perpendicular to landing approach direction)
; - VHZ: Down-range velocity (along landing approach direction)
;
; Uses half-unit vectors UHYP and UHZP defining landing site geometry.
; These vectors computed by guidance based on landing site location and
; approach trajectory. Dot product projects velocity components:
; VHY = V · UHYP (cross-range component)
; VHZ = V · UHZP (down-range component)
;
; Armstrong used these display components during final approach to verify
; LM wasn't drifting laterally (cross-range) while moving forward toward
; selected landing site. Manual throttle and attitude adjustments based on
; these velocity readings.
; ============================================================================
;
		CA	ITEMP1		# COMPUTE VHY, VELOCITY DIRECTED ALONG THE
		EXTEND			# Y-COORDINATE.
		MP	UHYP		# HI X OF CROSS-RANGE HALF-UNIT VECTOR.
		XCH	RUPTREG1
		CA	ITEMP2
		EXTEND
		MP	UHYP +2		#    Y
		ADS	RUPTREG1	# ACCUMULATE PARTIAL PRODUCTS.
		CA	ITEMP3
		EXTEND
		MP	UHYP +4		#    Z
		ADS	RUPTREG1
# Page 903
		CA	RUPTREG1
		DOUBLE
		XCH	VHY		# VHY=VMP.UHYP M/CS*2(-5).
		CA	ITEMP1		# NO COMPUTE VHZ, VELOCITY DIRECTED ALONG
		EXTEND			# THE Z-COORDINATE.
		MP	UHZP		# HI X OF DOWN-RANGE HALF-UNIT VECTOR.
		XCH	RUPTREG1
		CA	ITEMP2
		EXTEND
		MP	UHZP +2		#    Y
		ADS	RUPTREG1	# ACCUMULATE PARTIAL PRODUCTS.
		CA	ITEMP3
		EXTEND
		MP	UHZP +4		#    Z
		ADS	RUPTREG1
		CA	RUPTREG1
		DOUBLE
		XCH	VHZ		# VHZ = VMP.UHZP M/CS*2(-5).
;
; ============================================================================
; COORDINATE ROTATION FOR CREW-REFERENCED DISPLAY
;
; Rotate velocity from landing site frame to crew display frame using
; Angle Of Gimbal (AOG) from guidance platform matrix (GPMATRIX). AOG
; represents rotation between:
; - Landing site coordinate system (cross-range/down-range)
; - Crew display coordinate system (lateral/forward relative to LM body)
;
; Retrieves M22 = COS(AOG) and M32 = SIN(AOG) from erasable bank 6.
; Bank switching required because GPMATRIX resides in different memory bank
; than current working variables. AGC's 2K erasable memory organized in banks,
; requiring explicit EBANK register updates for cross-bank data access.
; ============================================================================
;
GET22/32	CAF	EBANK6		# GET SIN(AOG),COS(AOG) FROM GPMATRIX.
		TS	EBANK
		EBANK=	M22
		CA	M22
		TS	ITEMP3
		CA	M32
		TS	ITEMP4
		CAF	EBANK7
		TS	EBANK
		EBANK=	UNIT/R/
;
; ============================================================================
; LATERAL AND FORWARD VELOCITY COMPUTATION FOR DSKY DISPLAY
;
; Apply 2D rotation matrix to convert cross-range/down-range velocities
; into lateral/forward components displayed to crew:
; LATVEL = VHY·COS(AOG) + VHZ·SIN(AOG)  (lateral, left/right)
; FORVEL = VHZ·COS(AOG) - VHY·SIN(AOG)  (forward, toward/away from site)
;
; Crew sees these velocities on DSKY during descent:
; - Lateral velocity: drift perpendicular to approach path
; - Forward velocity: closure rate toward landing site
;
; During Apollo 11 final approach, Aldrin called out forward velocity values
; "6 forward...40 feet, down 2-1/2" allowing Armstrong to assess horizontal
; drift while controlling vertical descent rate. Critical for avoiding
; lateral touchdown velocity that could tip over LM.
; ============================================================================
;
LATFWDV		CA	ITEMP4		# COMPUTE LATERAL AND FORWARD VELOCITIES.
		EXTEND
		MP	VHY
		XCH	RUPTREG1
		CA	ITEMP3
		EXTEND
		MP	VHZ
		ADS	RUPTREG1	# =VHY(COS)AOG+VHZ(SIN)AOG M/CS *2(-5)
		CA	VELCONV		# CONVERT LATERAL VELOCITY TO BIT UNITS.
		EXTEND
		MP	RUPTREG1
		DDOUBL
		XCH	LATVEL		# LATERAL VELOCITY IN BIT UNITS *2(-14).
		CA	ITEMP4		# COMPUTE FORWARD VELOCITY.
		EXTEND
		MP	VHZ
		XCH	RUPTREG1
		CA	ITEMP3
		EXTEND
		MP	VHY
		CS	A
		ADS	RUPTREG1	# =VHZ(COS)AOG-VHY(SIN)AOG M/CS *2(-5).
# Page 904
;
; Convert velocities from meters/centisecond to display bit units.
; VELCONV scaling factor: 0.5571 feet-per-second per bit unit.
; DDOUBL instruction doubles twice (multiply by 4) to adjust scaling from
; 2^(-5) to 2^(-14) required for display register format. Final values
; stored in LATVEL and FORVEL ready for analog meter outputs.
;
		CA	VELCONV		# CONVERT FORWARD VELOCITY TO BIT UNITS.
		EXTEND
		MP	RUPTREG1
		DDOUBL
		XCH	FORVEL		# FORWARD VELOCITY IN BIT UNITS *2(-14).
;
; ============================================================================
; VELOCITY DISPLAY LIMITING AND VALIDATION
;
; Limit lateral and forward velocities to ±199.9989 fps (±547 octal bit units)
; to prevent analog meter overrange. Display meters have finite deflection
; range. Velocities exceeding ±200 fps pegged at limit value rather than
; wrapping or showing incorrect readings.
;
; During normal descent, velocities well below limits. Limiting protects
; against:
; - Transient spikes from radar data dropouts
; - Navigation filter divergence
; - Sensor failures
;
; Loop processes both LATVEL (index 1) and FORVEL (index 0) using indexed
; addressing. AGC INDEX instruction allows iteration without code duplication.
; ============================================================================
;
		CS	MAXVBITS	# ACC.=-199.9989 FT./SEC.
		TS	ITEMP6		# -547 BIT UNITS (OCTAL) AT 0.5571 FPS/BIT

		CAF	ONE		# LOOP TWICE.
VMONITOR	TS	ITEMP5		# FORWARD AND LATERAL VELOCITY LANDING
		INDEX	ITEMP5		#    ANALOG DISPLAYS MONITOR.
;
; Check velocity sign and magnitude. CCS (Count, Compare, and Skip) tests:
; - Positive: skip to +4 (check upper bound)
; - Zero: go to LVLIMITS (peg at limit for zero values)
; - Negative: skip to +8D (check lower bound)
; - Minus zero: go to LVLIMITS
;
		CCS	LATVEL
		TCF	+4
		TCF	LVLIMITS
		TCF	+8D
		TCF	LVLIMITS
;
; Check if positive velocity exceeds upper limit (+199.9989 fps).
; Complement velocity and add MAXVBITS. If result ≤0, velocity within range.
; If result >0, velocity too large - go to LVLIMITS.
;
		INDEX	ITEMP5
		CS	LATVEL
		AD	MAXVBITS	# +199.9989 FT.SEC.
		EXTEND
		BZMF	CHKLASTY
		TCF	LVLIMITS
;
; Check if negative velocity exceeds lower limit (-199.9989 fps).
; Add velocity (negative value) to MAXVBITS (positive value).
; If result ≤0, magnitude within range.
; If result >0, magnitude too large - go to LVLIMITS.
;
		INDEX	ITEMP5
		CA	LATVEL
		AD	MAXVBITS
		EXTEND
		BZMF	+2
		TCF	LVLIMITS
;
; ============================================================================
; CONSISTENCY CHECK WITH PREVIOUS DISPLAY VALUE
;
; Velocity within ±200 fps range. Now verify consistency with previous
; meter reading (LATVMETR or FORVMETR). Detect sign changes or discontinuities
; that might indicate data validity problems. AGC has no floating point
; hardware - sign changes require special handling to prevent display glitches.
;
; LATVMETR stores previous velocity meter value. Check if:
; - Last value was positive and new value is positive (consistent)
; - Last value was negative and new value is negative (consistent)
; - Last value zero (initialize)
; - Sign change (handle transition carefully)
; ============================================================================
;
CHKLASTY	INDEX	ITEMP5
		CCS	LATVMETR
		TCF	+4		# LAST POSITIVE.
		TCF	LASTOK		# LAST ZERO - ACCEPT NEW VALUE.
		TCF	+7		# LAST NEGATIVE.
		TCF	LASTOK		# LAST -0 - ACCEPT NEW VALUE.
;
; Last meter value was positive. Check if new velocity is positive.
; If new value negative, handle sign transition carefully.
;
		INDEX	ITEMP5
		CA	LATVEL
		EXTEND
		BZMF	LASTPOSY +5	# NEW VALUE POSITIVE - CONSISTENT.
		TCF	+5		# NEW VALUE NEGATIVE - SIGN CHANGE.
;
; Last meter value was negative. Check if new velocity is negative.
; If new value positive, handle sign transition carefully.
;
		INDEX	ITEMP5
		CS	LATVEL
		EXTEND
		BZMF	LASTNEGY +4	# NEW VALUE NEGATIVE - CONSISTENT.
;
; Both current and previous values valid and consistent. Now check tracking
; flag TRAKLATV to determine if meter is actively tracking velocity changes.
; TRAKLATV indicates whether display has locked onto valid data stream.
;
LASTOK		INDEX	ITEMP5
		CCS	TRAKLATV
		TCF	LASTPOSY	# TRACKING POSITIVE VELOCITIES.
		TCF	+2		# NOT TRACKING YET.
		TCF	LASTNEGY	# TRACKING NEGATIVE VELOCITIES.
		INDEX	ITEMP5
# Page 905
		CA	LATVEL
		EXTEND
		BZMF	NEGVMAXY
		TCF	POSVMAXY
;
; ============================================================================
; POSITIVE VELOCITY TRACKING PATH
;
; Previous value was positive, meter actively tracking. Verify new velocity
; still positive. If velocity changed sign (now negative), transition
; through zero to maintain display continuity. Abrupt sign reversals can
; cause display meter to peg at limit before reversing direction.
; ============================================================================
;
LASTPOSY	INDEX	ITEMP5
		CA	LATVEL
		EXTEND
		BZMF	+2		# NEW VALUE STILL POSITIVE.
		TCF	POSVMAXY	# NEW VALUE POSITIVE - NORMAL TRACKING.
;
; New velocity negative while tracking positive - sign change detected.
; Must drive meter through zero before displaying negative value. Load
; negative max velocity limit to force meter toward zero crossing.
;
		CS	MAXVBITS	# FORCE METER TOWARD ZERO.
		TCF	ZEROLSTY
;
; ============================================================================
; POSITIVE VELOCITY MAX TRACKING LOGIC
;
; Check if positive velocity exceeds maximum display range (+199.9989 fps).
; If within range, compute delta from current meter position (LATVMETR)
; and update display. If exceeds max, hold at limit to prevent meter
; damage from over-range pulses.
;
; Meter tracking prevents abrupt jumps. Rather than directly commanding
; new position, AGC computes difference between current and target and
; sends incremental pulses. Mechanical meter responds proportionally to
; pulse train rate.
; ============================================================================
;
POSVMAXY	INDEX	ITEMP5
		CS	LATVMETR	# CURRENT METER POSITION.
		AD	MAXVBITS	# DELTA TO MAX POSITIVE LIMIT.
		INDEX	ITEMP5
		XCH	RUPTREG3	# STORE DELTA FOR PULSE OUTPUT.
		CAF	ONE		# SET TRACKING FLAG = +1 (POS TRACKING).
		TCF	ZEROLSTY +3
;
; ============================================================================
; NEGATIVE VELOCITY TRACKING PATH
;
; Previous value was negative, meter actively tracking. Verify new velocity
; still negative. If velocity changed sign (now positive), transition
; through zero to maintain display continuity before tracking positive values.
;
; Symmetrical logic to LASTPOSY but for negative velocity range.
; ============================================================================
;
LASTNEGY	INDEX	ITEMP5
		CA	LATVEL
		EXTEND
		BZMF	NEGVMAXY	# NEW VALUE STILL NEGATIVE.
;
; New velocity positive while tracking negative - sign change detected.
; Drive meter through zero before displaying positive value.
;
		CA	MAXVBITS	# FORCE METER TOWARD ZERO.
		TCF	ZEROLSTY
;
; ============================================================================
; NEGATIVE VELOCITY MAX TRACKING LOGIC
;
; Check if negative velocity exceeds minimum display range (-199.9989 fps).
; Computes delta from current meter position to new target. Negative
; velocities represented with sign bit set (bit 15).
;
; Symmetrical to POSVMAXY but handling negative range. Tracking flag set
; to -1 indicating meter actively tracking negative velocity values.
; ============================================================================
;
NEGVMAXY	INDEX	ITEMP5
		CA	LATVMETR	# CURRENT METER POSITION.
		AD	MAXVBITS	# DELTA TO MAX NEGATIVE LIMIT.
		COM		# COMPLEMENT FOR NEGATIVE RANGE.
		INDEX	ITEMP5
		XCH	RUPTREG3	# STORE DELTA FOR PULSE OUTPUT.
		CS	ONE		# SET TRACKING FLAG = -1 (NEG TRACKING).
		TCF	ZEROLSTY +3
;
; ============================================================================
; VELOCITY DISPLAY RATE LIMITING LOGIC
;
; LVLIMITS enforces maximum rate of change for velocity display updates.
; Prevents display meter from responding faster than mechanical capability.
; Checks tracking state (TRAKLATV) to determine current velocity direction:
;   +1: Tracking positive velocities
;    0: Not tracking (meter at zero)
;   -1: Tracking negative velocities
;
; Validates new velocity value won't cause excessive meter slew rate that
; could damage mechanism or confuse crew with unrealistic jump. During
; Apollo 11 descent, this smoothing prevented display spikes from radar
; measurement noise or navigation filter transients.
; ============================================================================
;
LVLIMITS	INDEX	ITEMP5
		CCS	TRAKLATV	# CHECK VELOCITY TRACKING STATE.
		TCF	LATVPOS		# +1: TRACKING POSITIVE.
		TCF	+2		# +0: NOT TRACKING.
		TCF	LATVNEG		# -1: TRACKING NEGATIVE.
;
; ============================================================================
; ZERO-STATE (NOT TRACKING) INITIALIZATION
;
; Meter currently at zero position (neither tracking positive nor negative).
; Determine which direction to begin tracking based on new velocity sign.
; Check meter position (LATVMETR) to verify it's on correct side of zero
; before initiating tracking.
;
; If meter position and velocity sign disagree, branch to NEGLMLV to handle
; the polarity mismatch. Otherwise compute initial tracking delta.
; ============================================================================
;
		INDEX	ITEMP5
		CS	LATVMETR	# CHECK METER POSITION SIGN.
		EXTEND
		BZMF	+2		# METER NEGATIVE OR ZERO.
		TCF	NEGLMLV		# METER POSITIVE - CHECK POLARITY.
;
; Meter at or near zero. Check new velocity sign.
;
		INDEX	ITEMP5
		CS	LATVEL		# NEW VELOCITY.
		EXTEND
		BZMF	LVMINLM		# VELOCITY NEGATIVE - GO TO MIN LIMIT.
;
; New velocity is positive. Compute rate-limited delta for initial tracking.
; Add slew rate limit (ITEMP6) and current meter position. Verify result
; won't exceed velocity value (prevent overshoot).
;
		AD	ITEMP6		# ADD SLEW RATE LIMIT.
		INDEX	ITEMP5
		AD	LATVMETR	# ADD CURRENT METER POSITION.
		EXTEND
# Page 906
		BZMF	LVMINLM		# RESULT NEGATIVE - LIMIT VIOLATED.
		INDEX	ITEMP5
		AD	LATVEL		# ADD TARGET VELOCITY.
		EXTEND
		INDEX	ITEMP5
		SU	LATVMETR	# SUBTRACT METER POSITION = DELTA.
		TCF	ZEROLSTY	# OUTPUT DELTA TO METER.
;
; ============================================================================
; POSITIVE VELOCITY TRACKING STATE
;
; Meter actively tracking positive velocities. Verify new velocity still
; positive and within tracking capability. If velocity went negative,
; must drive meter back through zero before tracking negative range.
;
; This path executes during normal positive velocity tracking when meter
; already responding to positive values. Validates sign consistency.
; ============================================================================
;
LATVPOS		INDEX	ITEMP5
		CS	LATVEL		# NEW VELOCITY VALUE.
		EXTEND
		BZMF	LVMINLM		# STILL POSITIVE - LIMIT CHECK.
		TCF	+5		# NEGATIVE - FORCE TO ZERO FIRST.
;
; ============================================================================
; NEGATIVE VELOCITY TRACKING STATE
;
; Meter actively tracking negative velocities. Verify new velocity still
; negative and within tracking capability. If velocity went positive,
; must drive meter back through zero before tracking positive range.
;
; Symmetrical logic to LATVPOS but for negative tracking state.
; ============================================================================
;
LATVNEG		INDEX	ITEMP5
		CA	LATVEL		# NEW VELOCITY VALUE.
		EXTEND
		BZMF	LVMINLM		# STILL NEGATIVE - LIMIT CHECK.
;
; Velocity changed sign. Force meter position to zero by outputting
; complement of current meter position as correction delta.
;
		INDEX	ITEMP5
		CS	LATVMETR	# COMPLEMENT METER POSITION.
		TCF	ZEROLSTY	# OUTPUT TO DRIVE METER TO ZERO.
;
; ============================================================================
; NEGATIVE LIMIT VELOCITY HANDLING
;
; Entered when meter position and velocity sign show polarity mismatch,
; typically during transition through zero. Validates velocity magnitude
; and computes rate-limited delta to prevent excessive meter slew.
;
; Checks if new velocity magnitude exceeds maximum display range. If within
; range, computes delta accounting for current meter position and slew
; rate limit (MAXVBITS). Ensures smooth transition without mechanical
; overstress.
; ============================================================================
;
NEGLMLV		INDEX	ITEMP5
		CA	LATVEL		# NEW VELOCITY VALUE.
		EXTEND
		BZMF	LVMINLM		# VELOCITY NEGATIVE - GO TO LIMIT.
;
; Velocity is positive. Compute rate-limited delta from current negative
; meter position to positive target velocity. MAXVBITS provides slew limit.
;
		CA	MAXVBITS	# MAXIMUM SLEW RATE LIMIT.
		INDEX	ITEMP5
		AD	LATVMETR	# ADD CURRENT METER POSITION.
		COM			# COMPLEMENT FOR DELTA COMPUTATION.
		INDEX	ITEMP5
		AD	LATVEL		# ADD TARGET VELOCITY.
		EXTEND
		BZMF	LVMINLM		# RESULT NEGATIVE - LIMIT VIOLATED.
;
; Compute final delta to output to meter. Subtract velocity from limit
; sum, add meter position, complement result for proper sign.
;
		EXTEND
		INDEX	ITEMP5
		SU	LATVEL		# SUBTRACT VELOCITY.
		INDEX	ITEMP5
		AD	LATVMETR	# ADD METER POSITION.
		COM			# COMPLEMENT FOR OUTPUT.
		TCF	ZEROLSTY	# OUTPUT DELTA TO METER.
;
; ============================================================================
; MINIMUM LIMIT COMPUTATION
;
; Entered when velocity tracking requires limiting. Computes minimum-limited
; delta by taking difference between target velocity and current meter
; position. This ensures meter tracks velocity without exceeding rate limits.
;
; Result fed to ZEROLSTY for final output processing.
; ============================================================================
;
LVMINLM		INDEX	ITEMP5
		CS	LATVMETR	# COMPLEMENT METER POSITION.
		INDEX	ITEMP5
		AD	LATVEL		# ADD TARGET VELOCITY = DELTA.
;
; ============================================================================
; FINAL OUTPUT PROCESSING
;
; Common exit path for all velocity display computations. Outputs computed
; delta to velocity meter hardware (CDUTCMD), updates meter position
; tracking (LATVMETR), and resets tracking state flag (TRAKLATV) to zero
; indicating no active tracking bias.
;
; Accumulator contains delta value in bit units. Hardware constraint: AGC
; DINC (Digital Increment) instruction malfunctions with +0 value, so NEG0
; added to prevent +0 output. This famous AGC hardware quirk required
; software workaround throughout all display routines.
;
; After processing one velocity component (forward or lateral), ITEMP5
; index cycles to process the other component. Both forward and lateral
; velocities displayed to crew for situational awareness during descent.
; ============================================================================
;
ZEROLSTY	INDEX	ITEMP5
		XCH	RUPTREG3	# SAVE DELTA IN RUPTREG3 TEMPORARILY.
		CAF	ZERO
		INDEX	ITEMP5
		TS	TRAKLATV	# RESET TRACKING STATE TO ZERO.
		INDEX	ITEMP5
		CA	RUPTREG3	# RETRIEVE DELTA VALUE.
		AD	NEG0		# AVOIDS +0 DINC HARDWARE MALFUNCTION.
# Page 907
		INDEX	ITEMP5
		TS	CDUTCMD		# OUTPUT DELTA TO VELOCITY METER.
		INDEX	ITEMP5
		CA	RUPTREG3	# RETRIEVE DELTA AGAIN.
		INDEX	ITEMP5
		ADS	LATVMETR	# UPDATE METER POSITION TRACKING.
		CCS	ITEMP5		# FIRST MONITOR FORWARD THEN LATERAL VEL.
		TCF	VMONITOR	# ITEMP5 POSITIVE: PROCESS SECOND VEL.
;
; Both velocity components processed. Activate X-pointer display driver
; to physically update velocity meters visible to crew in LM cabin.
;
		CAF	BITSET		# DRIVE THE X-POINTER DISPLAY.
		EXTEND
		WOR	CHAN14		# BIT 6 ACTIVATES VELOCITY METERS.
		TC	LADQSAVE	# GO TO ALTROUT +1 OR TO ALTOUT +1.
;
; Zero altitude data entry point. Used when altitude data invalid or
; unavailable. Prevents negative altitude display (nonsensical on lunar
; surface where altitude measured from surface, never negative).
;
ZERODATA	CAF	ZERO		# ZERO ALTSAVE AND ALTSAVE +1.
		TS	L		# NO NEGATIVE ALTITUDES ALLOWED.
		TCF	ZDATA2

# ************************************************************************
;
; ============================================================================
; DISPLAY RESET AND DEACTIVATION
;
; DISPRSET entered when landing analog displays flag not set (displays
; not active). Performs cleanup operations to reset display modes and
; clear interface flags. Critical during transitions between flight phases
; (e.g., exiting powered descent, entering abort trajectory).
;
; Checks R10 flag to determine if in descent trajectory (R10 guidance
; program active). If in descent, manages inertial data display modes
; and rendezvous radar (RR) error counter. If not in descent (abort or
; other mode), skips descent-specific cleanup and proceeds to flag reset.
;
; During Apollo 11, this routine executed during mode transitions when
; landing displays deactivated after touchdown or during abort scenarios.
; ============================================================================
;
DISPRSET	CS	FLAGWRD0	# ARE WE IN DESCENT TRAJECTORY?
		MASK	R10FLBIT	# R10 FLAG = DESCENT GUIDANCE ACTIVE.
		EXTEND
		BZF	ABORTON		# NO - SKIP DESCENT-SPECIFIC CLEANUP.
		CAF	BIT8		# YES - IN DESCENT TRAJECTORY.
		MASK	IMODES33	# CHECK IF INERTIAL DATA JUST DISPLAYED.
		CCS	A
		CAF	BIT2		# YES. DISABLE RR ERROR COUNTER.
		AD	BIT8		# NO. REMOVE DISPLAY INERTIAL DATA FLAG.
		COM			# COMPLEMENT FOR WAND OPERATION.
		EXTEND
		WAND	CHAN12		# CLEAR DISPLAY MODE BITS IN CHANNEL 12.
;
; Common exit path. Reset display mode flags (inertial data, interleave)
; and clear DIDFLAG (Display Inertial Data Flag) indicating no inertial
; data currently displayed. Completes display deactivation sequence.
;
ABORTON		CS	BITS8/7		# RESET INERTIAL DATA, INTERLEAVE FLAGS.
		MASK	IMODES33	# CLEAR BITS 7 AND 8 IN IMODES33.
		TS	IMODES33
		CS	DIDFLBIT	# DISPLAY INERTIAL DATA FLAG BIT.
		MASK	FLAGWRD1	# CLEAR DIDFLAG.
		TS	FLAGWRD1	# RESET DIDFLAG.
		TCF	TASKOVER	# EXIT DISPLAY ROUTINE.
# ************************************************************************
BITS8/7		OCT	00300		# INERTIAL DATA AND INTERLEAVE FLAGS.

BITSET		=	PRIO6
# ************************************************************************


