# Copyright:	Public domain.
# Filename:	AUTOMATIC_MANEUVERS.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1025-1036
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
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
; FILE: AUTOMATIC_MANEUVERS.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: all-phases
;
; TL;DR: Automated attitude maneuver sequence control implementing crew-commanded
;        spacecraft rotations and inertial hold modes. Executes predefined
;        attitude changes for thermal control, communications antenna pointing,
;        and mission timeline operations throughout Apollo 11 flight from launch
;        through splashdown.
;
; COMMENT-ONLY READERS: This program automatically rotated the spacecraft to
;        new orientations when commanded by the crew, maintaining precise 
;        attitude control during coast phases between major propulsive maneuvers.
; CODE-ALONG READERS: Study automatic maneuver sequencing with phase plane
;        switching logic, attitude error computation using gimbal angle matrix,
;        and rate command generation for RCS thruster control integration.
; ============================================================================

# Page 1025
		BANK	21
		SETLOC	DAPS3
		BANK

		COUNT	21/DAPAM

; ============================================================================
; AUTOMATIC MANEUVER CONTROL ENTRY POINT
;
; The Command Module requires periodic attitude changes throughout the mission
; for thermal control (passive thermal control "barbecue roll"), communications
; antenna pointing toward Earth, and rendezvous operations. This routine 
; determines whether the spacecraft should perform automatic attitude maneuvers
; or allow crew manual control via the rotation hand controller.
; ============================================================================

		EBANK=	KMPAC
AHFNOROT	EXTEND
		READ	CHAN31		; Read spacecraft mode control switches
		MASK	BIT14		; Check automatic rotation enable bit
		EXTEND
		BZMF	FREECONT	; If disabled, go to manual control mode
; The spacecraft checks its control mode. During translunar coast, Apollo 11
; performed slow rotation maneuvers for thermal control, preventing one side
; from overheating in direct sunlight while the other froze in shadow.
;
		CA	RCSFLAGS	# SEE IF RATE FILTER HAS BEEN INITIALIZED
		MASK	BIT14
		CCS	A		# IF SO, PROCEED WITH ATTITUDE CONTROL
		TCF	REINIT		# IF NOT, RECYCLE TO INITIALIZE FILTER
					# AUTOMATIC CONTROL YET
; Rate filtering ensures smooth spacecraft motion without oscillations that
; would waste precious RCS propellant. The filter must be initialized before
; automatic maneuvers can begin safely.
;
		EXTEND
		READ	CHAN31		; Check for attitude hold mode
		MASK	BIT13		; Isolate hold function bit
		EXTEND
		BZMF	HOLDFUNC	; Branch to hold function if requested

; ============================================================================
; TRANSITION: From control mode selection to automatic maneuver execution
;
; Once the system verifies automatic control is enabled and rate filtering is
; initialized, it determines whether to acquire a new target attitude or hold
; the current orientation. Crews used this capability throughout the mission
; for precision pointing requirements.
; ============================================================================

AUTOCONT	CA	HOLDFLAG	# IF HOLDFLAG IS +, GO TO GRABANG.
		EXTEND			# OTHERWISE, GO TO ATTHOLD.
		BZMF	ATTHOLD		; Maintain current attitude
		TCF	GRABANG		; Acquire new target attitude

# MINIMUM IMPULSE CONTROL

; ============================================================================
; MANUAL CONTROL MODE - MINIMUM IMPULSE THRUSTERS
;
; When the crew takes manual control using the rotation hand controller, the
; AGC must interpret stick deflections and command the appropriate RCS thrusters.
; To conserve propellant, the system uses minimum impulse mode - brief 14ms
; thruster pulses rather than continuous firing. The astronauts felt this as
; discrete "clicks" of attitude change rather than smooth rotation.
; ============================================================================

FREECONT	CAF	ONE
		TS	HOLDFLAG	# RESET HOLDFLAG
					# INHIBIT AUTOMATIC STEERING
; Crew manual control takes precedence over automatic systems. During Apollo 11's
; translunar coast, when not performing thermal control rolls, the crew could
; manually orient the spacecraft to observe Earth or photograph the Moon.
;
		EXTEND
		READ	CHAN32		; Read rotation hand controller inputs
		TS	L		; Save in L register
		COM			; Complement for proper sense
		MASK	MANROT		; Isolate manual rotation bits
		MASK	CHANTEMP	; Apply channel template
		LXCH	CHANTEMP	; Exchange with channel temp
		TC	STICKCHK	; Verify stick input validity
; The rotation hand controller provides three-axis control: roll, pitch, and yaw.
; Each axis deflection commands minimum impulse thruster firing (14 milliseconds)
; in the corresponding direction. Multiple pulses create smooth rotations.
;
		INDEX	RMANNDX		; Roll axis index
		CA	MINTAU		# MINTAU	+0
		TS	TAU		#		+1	+14MS MINIMUM IMPULSE
		INDEX	PMANNDX		#		+2	-14MS TIME
		CA	MINTAU		#		+3	+0
		TS	TAU1		; Pitch axis time command
		INDEX	YMANNDX		; Yaw axis index
		CA	MINTAU		; Load yaw time command
# Page 1026
		TS	TAU2		; Store yaw axis time command
		TCF	T6PROGM		; Transfer to jet selection program

; Minimum impulse time constants - 14 milliseconds (23 AGC time units)
; This brief pulse duration minimizes propellant consumption while providing
; adequate control authority for manual spacecraft maneuvering.
;
MINTAU		DEC	0		; No rotation command
		DEC	23		# = 14MS positive rotation
		DEC	-23		# = -14MS negative rotation
		DEC	0		; No rotation command

# Page 1027
# 	CALCULATION OF ATTITUDE ERRORS-
#	-    *     -      -          -
#	AK = AMGB (CDUX - THETADX) + BIAS
#
# IE	*AK *   * 1        SIN(PSI)        0	** CDUX - THETADX *    *BIAS *
#	*   *   *                               **                *    *     *
#	*AK1* = * 0   COS(PSI)COS(PHI)  SIN(PHI)** CDUY - THETADY *  + *BIAS1*
#	*   *   *                               **                *    *     *
#	*AK2*   * 0  -COS(PSI)SIN(PHI)  COS(PHI)** CDUZ - THETADZ *    *BIAS2*
#
# 	THE BIASES ARE ADDED ONLY WHILE PERFORMING AUTOMATIC MANEUVERS (ESP KALCMANU) TO PROVIDE ADDITIONAL LEAD
# AND PREVENT OVERSHOOT WHEN STARTING AN AUTOMATIC MANEUVER.  NORMALLY THE REQUIRED LEAD IS ONLY 1-2 DEGREES.
# BUT DURING HIGH RATE MANEUVERS IT CAN BE AS MUCH AS 7 DEGREES.  THE BIASES ARE COMPUTED BY KALCMANU AND REMAIN
# FIXED UNTIL THE MANEUVER IS COMPLETED AT WHICH TIME THEY ARE RESET TO ZERO.

; ============================================================================
; ATTITUDE HOLD - COMPUTING SPACECRAFT ORIENTATION ERRORS
;
; To maintain or achieve a desired spacecraft attitude, the AGC must compute
; the error between current orientation (measured by IMU gimbal angles CDUX,
; CDUY, CDUZ) and the desired target orientation (THETADX, THETADY, THETADZ).
; The rotation matrix AMGB transforms gimbal-coordinate errors into body-axis
; errors that directly correspond to roll, pitch, and yaw corrections.
;
; During Apollo 11, this calculation ran continuously during coast phases,
; computing the small attitude errors that triggered corrective RCS thruster
; firings to maintain the commanded orientation - whether that was passive
; thermal control roll, antenna pointing toward Earth, or a specific inertial
; attitude for navigation star sightings.
; ============================================================================

ATTHOLD		CA	CDUX		; Read outer gimbal angle from IMU
		EXTEND
		MSU	THETADX		; Subtract desired outer gimbal angle
		TS	ERRORX		; Store roll-axis gimbal error
; CDUX, CDUY, CDUZ are the three gimbal angles from the Inertial Measurement
; Unit (IMU). Each angle is scaled in half-revolutions (1.0 = 180 degrees).
; The target angles THETADX/Y/Z define the commanded spacecraft attitude.
;
		CA	CDUY		; Read middle gimbal angle
		EXTEND
		MSU	THETADY		; Subtract desired middle gimbal angle
		TS	T5TEMP		; Store pitch-axis gimbal error temporarily
		EXTEND
		MP	AMGB1		; Multiply by AMGB(1,2): sin(psi) term
		ADS	ERRORX		; Add contribution to roll-axis body error
; The AMGB matrix transforms gimbal-coordinate errors into body-coordinate errors.
; Body coordinates correspond directly to the spacecraft's physical axes: roll
; (X-axis along spacecraft length), pitch (Y-axis), and yaw (Z-axis). This
; transformation accounts for the current gimbal configuration.
;
		CA	T5TEMP		; Reload pitch-axis gimbal error
		EXTEND
		MP	AMGB4		; Multiply by AMGB(2,2): cos(psi)cos(phi)
		TS	ERRORY		; Initialize pitch-axis body error
		CA	T5TEMP		; Reload pitch-axis gimbal error
		EXTEND
		MP	AMGB7		; Multiply by AMGB(3,2): -cos(psi)sin(phi)
		TS	ERRORZ		; Initialize yaw-axis body error
;
		CA	CDUZ		; Read inner gimbal angle
		EXTEND
		MSU	THETADZ		; Subtract desired inner gimbal angle
		TS	T5TEMP		; Store yaw-axis gimbal error
		EXTEND
		MP	AMGB5		; Multiply by AMGB(2,3): sin(phi) term
		ADS	ERRORY		; Add contribution to pitch-axis body error
		CA	T5TEMP		; Reload yaw-axis gimbal error
		EXTEND
		MP	AMGB8		; Multiply by AMGB(3,3): cos(phi) term
		ADS	ERRORZ		; Add contribution to yaw-axis body error
; The result is a complete transformation of gimbal errors (measured in gimbal
; frame) to body errors (measured in spacecraft frame). These body-axis errors
; directly indicate which RCS thrusters must fire to correct the orientation.
;
		CS	HOLDFLAG	; Check if performing automatic maneuver
		EXTEND
# Page 1028
		BZMF	JETS		; If holding, proceed to jet selection
; During automatic maneuvers, especially high-rate maneuvers commanded by KALCMANU,
; bias terms provide additional lead to prevent overshoot. The biases anticipate
; where the spacecraft will be and command thrusters slightly ahead of time.
;
		CA	BIAS		# AD BIASES ONLY IF PERFORMING AUTOMATIC
		ADS	ERRORX		; Add roll-axis bias (typically 1-7 degrees)
		CA	BIAS1		; Load pitch-axis bias
		ADS	ERRORY		; Add to pitch error for lead compensation
		CA	BIAS2		; Load yaw-axis bias
		ADS	ERRORZ		; Add to yaw error for lead compensation
		TCF	JETS		; Proceed to jet selection logic


; ============================================================================
; HOLD FUNCTION - ATTITUDE HOLD MODE STATE LOGIC
;
; This routine determines the appropriate action based on the current state of
; the HOLDFLAG. It implements a state machine that transitions between active
; maneuvering and steady-state attitude hold. The CCS instruction tests HOLDFLAG
; and branches based on whether it's positive, zero, or negative.
; ============================================================================

HOLDFUNC	CCS	HOLDFLAG	; Count, compare, skip on HOLDFLAG
		TCF	+3		; Positive: active maneuver, skip ahead
		TCF	ATTHOLD		; Zero: already holding, compute errors
		TCF	+1		; Negative: fall through to GRABANG
; The three-way branch reflects AGC's unique CCS instruction which tests a value
; and provides different paths for positive, zero, and negative values. This
; compact logic handles multiple state transitions in minimal instructions.
;
; ============================================================================
; GRAB CURRENT ANGLES - INITIATING NEW ATTITUDE HOLD
;
; When transitioning into attitude hold mode (such as after completing a
; maneuver or when crew selects auto hold), the computer must capture the
; current spacecraft orientation and zero all rate terms. This "grab" operation
; makes the current IMU gimbal angles the new target attitude. The spacecraft
; will maintain this exact orientation using RCS thrusters.
; ============================================================================

GRABANG		CAF	ZERO		# ZERO WBODYS AND BIASES
		TS	WBODY		; Zero roll-axis angular velocity
		TS	WBODY +1	; Zero roll rate (double precision)
		TS	WBODY1		; Zero pitch-axis angular velocity
		TS	WBODY1 +1	; Zero pitch rate (double precision)
		TS	WBODY2		; Zero yaw-axis angular velocity
		TS	WBODY2 +1	; Zero yaw rate (double precision)
; The WBODY terms represent spacecraft body rates. Zeroing them indicates
; the DAP should aim for zero rotation rate - a steady hold rather than
; a continuous maneuver. During Apollo 11's coast phases, this steady-state
; control minimized propellant consumption while maintaining orientation.
;
		TS	BIAS		; Zero roll-axis maneuver bias
		TS	BIAS1		; Zero pitch-axis maneuver bias
		TS	BIAS2		; Zero yaw-axis maneuver bias
; Maneuver bias terms are only needed during active attitude changes. During
; hold mode, the spacecraft maintains current orientation without anticipatory
; lead compensation.
;
		CA	RCSFLAGS	; Check RCS control system flags
		MASK	OCT16000	; Isolate rate damping completion flag
		EXTEND			# IS RATE DAMPING COMPLETED
		BZF	ENDDAMP		# IF SO, GO TO ENDDAMP
; Rate damping is the initial phase when entering hold mode. The RCS fires to
; eliminate any residual spacecraft rotation rates before locking onto the
; target attitude. This prevents oscillation and propellant waste.
;
		CAF	ZERO		# OTHERWISE, ZERO ERRORS
		TS	ERRORX		; Zero roll-axis error
		TS	ERRORY		; Zero pitch-axis error
		TS	ERRORZ		; Zero yaw-axis error
		TCF	JETS		; Proceed with zeroed errors to jet logic
; While damping rates, the error terms are forced to zero. This prevents the
; attitude control from fighting the rate damping process. Only after rates
; are fully damped does the system begin precision attitude maintenance.
;
; ============================================================================
; END DAMPING - CAPTURING TARGET ATTITUDE
;
; Once rate damping is complete (all rotation rates near zero), the computer
; samples the current IMU gimbal angles and makes them the official target
; attitude for the hold mode. From this moment forward, any deviation from
; these angles triggers corrective thruster firing.
; ============================================================================

ENDDAMP		TS	HOLDFLAG	# SET HOLDFLAG +0
; Setting HOLDFLAG to zero indicates the system is now in steady-state hold.
; The transition from damping to holding is complete. The spacecraft orientation
; is stable and will be actively maintained by the digital autopilot.
;
		EXTEND
		DCA	CDUX		# PICK UP CDU ANGLES FOR ATTITUDE HOLD
		DXCH	THETADX		# REFERENCES
; The DCA (double precision load) and DXCH (double precision exchange) instructions
; efficiently capture both outer gimbal (CDUX) and middle gimbal (CDUY) angles
; simultaneously, storing them as the roll and pitch target attitudes.
;
		CA	CDUZ		; Load inner gimbal angle (yaw)
		TS	THETADZ		; Store as yaw-axis target
; With all three gimbal angles captured, the spacecraft now has a complete
; attitude reference. Any drift from these angles will be detected by ATTHOLD
; and corrected by the RCS jets. During Apollo 11, this hold mode maintained
; precise orientation during critical operations like navigation sightings,
; communications passes, and crew rest periods.
;
		TCF	ATTHOLD		; Begin computing attitude errors

# Page 1029
# JET SWITCHING LOGIC AND CALCULATION OF REQUIRED ROTATION COMMANDS
#
# DETERMINE THE LOCATION OF THE RATE ERROR AND THE ATTITUDE ERROR RELATIVE TO THE SWITCHING LOGIC IN THE PHASE
# PLANE.
# COMPUTE THE CHANGE IN RATE CORRESPONDING TO THE ATTITUDE ERROR NECESSARY TO DRIVE THE THE S/C INTO THE
# APPROPRIATE DEADZONE.
#
#                                     .
#   R22                          RATE . ERROR
#        WL+H                         .
# *********************************   .					***** SWITCH LINES ENCLOSING DEADZONES
#   R23  WL                        *  .
# ----------------------------------* .					----- DESIRED RATE LINES
#   R23  WL-H       -                *.
# ****************** -                .					R20, R21, R22, ETC REGIONS IN PHASE
#                   * -               .* R18      R20       R21		PLANE FOF COMPUTING DESIRED RESPONSE
#                    *                . *
#                     *-              .  *
#   R22             R24*-    R23      .   *
#                       *             .    *
#                        *            .     *
#                         + -ADB      .      * AF              ATTITUDE
#  ........................+--+---------------+--+........................
#                           AF *      .     +ADB  +             ERROR
#                               *     .            *
#                                *    .            -*
#                                 *   .             -*
#                                  *  .              -*
#                                   * .                *
#                                    *.               - *
#                                     .                - *****************
#                                     .*                -
#                                     . * --------------------------------
#                                     .  *
#                                     .   ********************************
#                                     .

#			FIG. 1	PHASE PLANE SWITCHING LOGIC


; ============================================================================
; JET SWITCHING LOGIC CONSTANTS
;
; These constants define the phase-plane switching boundaries for the RCS
; digital autopilot. The phase plane plots attitude error (horizontal axis)
; versus rate error (vertical axis). The switching logic divides this plane
; into regions, each corresponding to a specific thruster firing strategy.
;
; The deadband zones (bounded by WL±H) define acceptable error regions where
; no thruster firing occurs, conserving propellant. Outside these zones, the
; logic computes the optimal firing time to drive the spacecraft back toward
; the deadband along the minimum-fuel trajectory.
; ============================================================================

# CONSTANTS FOR JET SWITCHING LOGIC

WLH/SLOP	DEC	.00463		# = WL+H/SLOPE = .83333 DEG	$180
; Upper deadband boundary divided by slope = 0.83333 degrees
; Defines the attitude error threshold at zero rate error for entering deadband

WL-H/SLP	DEC	.00277		# = WL-H/SLOPE = .5 DEG		$180
; Lower deadband boundary divided by slope = 0.5 degrees
; Defines the inner switching line for rate damping transitions

WLH		2DEC	.0011111111	# = WL+H = 0.5 DEG/SEC		$450
; Upper rate limit: 0.5 degrees/second
; Maximum acceptable angular velocity at the deadband boundary

WLMH		2DEC	.0006666666	# = WL-H = 0.3 DEG/SEC		$450
; Lower rate limit: 0.3 degrees/second  
; Minimum rate threshold for switching logic transitions

WL		2DEC	.0008888888	# = WL   = 0.4 DEG/SEC		$450
; Nominal rate limit: 0.4 degrees/second
; Center of the rate deadband for nominal attitude hold

# Page 1030
SLOPE2		DEC	.32		# = 0.8 DEG/SEC/DEG		$450/180
; Hysteresis slope: 0.8 (deg/sec)/deg
; Defines the slope of switching lines in the phase plane to prevent
; chattering when the state is near a boundary
;
; ============================================================================
; JETS - MAIN JET SWITCHING LOGIC ENTRY POINT
;
; This is the primary entry point for the RCS jet firing decision logic.
; On each pass through the digital autopilot cycle (nominally every 100 ms),
; this routine examines the current attitude and rate errors for all three
; axes, determines which jets should fire and for how long, and schedules
; those firings via the TJETLAW interrupt routine.
;
; The logic implements a phase-plane control law that minimizes propellant
; consumption while maintaining attitude within acceptable deadbands. During
; Apollo 11's coast phases, this autopilot consumed less than 1% of RCS
; propellant per day, enabling the 8-day mission profile.
; ============================================================================

JETS		CA	ADB		; Load attitude deadband parameter
		AD	FOUR		# AF = FLAT REGION = .044 DEG
		TS	T5TEMP		# ADB+AF
; ADB (attitude deadband) defines the acceptable error zone. AF (flat region)
; adds a hysteresis band to prevent oscillation at the boundary. The combined
; value T5TEMP becomes the outer limit for attitude hold.
;
		CAF	TWO		; Initialize loop counter for 3 axes
; The jet logic processes roll (axis 0), pitch (axis 1), and yaw (axis 2) in
; sequence. CAF TWO loads the starting index for countdown iteration.
;
; ============================================================================
; JLOOP - JET COMMAND CALCULATION FOR ALL AXES
;
; This loop iterates through all three spacecraft axes (roll, pitch, yaw),
; calculating the required jet firing commands for each. The state variables
; (attitude error, rate error) are processed through phase-plane switching
; logic to determine optimal thruster on-time.
; ============================================================================

JLOOP		TS	SPNDX		; Store single-precision axis index
		DOUBLE			; Multiply by 2 for double-precision indexing
		TS	DPNDX		; Store double-precision axis index
; AGC uses separate indices for single-word (SPNDX) and double-word (DPNDX)
; array access. The DOUBLE instruction efficiently creates the double-word
; index from the single-word value.
;
		EXTEND
		INDEX	A		; Use accumulator as index
		DCA	ADOT		; Load rate error (double precision)
		DXCH	EDOT		; Store in EDOT for processing
; ADOT contains the measured angular velocity for the current axis. This rate
; is compared against the desired rate (WBODY during maneuvers, zero during
; hold) to compute rate error EDOT.
;
		CA	HOLDFLAG	# HOLDFLAG = +0 MEANS THAT DAP IS IN
		EXTEND			# ATTITUDE HOLD AND RATE DAMPING IS OVER.
		BZF	INHOLD		# IF THIS IS THE CASE, BYPASS ADDITION
					# OF WBODY AND GO TO INHOLD
; During attitude hold (HOLDFLAG = 0), the desired rate is zero. During active
; maneuvers (HOLDFLAG ≠ 0), the desired rate is WBODY. This branch determines
; whether to subtract the commanded body rate from the measured rate.
;
		EXTEND
		INDEX	DPNDX		; Index into WBODY array
		DCS	WBODY		; Load desired body rate (negated)
		DAS	EDOT		# = ADOT-WBODY
; Compute rate error: measured rate minus desired rate. During maneuvers, this
; tells the autopilot how far the actual rotation differs from the commanded
; rotation profile.
;
; ============================================================================
; INHOLD - ATTITUDE ERROR PROCESSING
;
; With rate error computed, now load the attitude error for the current axis.
; The combination of attitude error and rate error defines a point in the
; phase plane that determines the required jet firing strategy.
; ============================================================================

INHOLD		INDEX	SPNDX		; Index into error array
		CA	ERRORX		; Load attitude error for current axis
		TS	AERR		# AERR = BIAS + AK
; ERRORX contains the angular error (actual attitude minus target attitude)
; plus any maneuver bias. This becomes AERR, the attitude error input to the
; phase-plane switching logic.

; ============================================================================
; PHASE-PLANE REGION DETERMINATION
;
; The autopilot divides the phase plane (attitude error vs. rate error) into
; regions to determine optimal jet firing strategy. This double-precision test
; determines whether the spacecraft is rotating too fast in the positive or
; negative direction, handling the sign and magnitude of EDOT (rate error).
; ============================================================================

		CCS	EDOT		; Test high-order word of rate error
		TCF	POSVEL		; Positive rate error (rotating too fast +)
		TCF	SIGNCK1		; High word zero, check low word for sign
		TCF	NEGVEL		; Negative rate error (rotating too fast -)
SIGNCK1		CCS	EDOT +1		; High word zero: test low-order word
		TCF	POSVEL		; Low word positive -> overall positive
		TCF	POSVEL		; Low word +0 -> treat as positive
		TCF	NEGVEL		; Low word negative -> overall negative
		TCF	NEGVEL		; Low word -0 -> treat as negative
; ============================================================================
; POSITIVE VELOCITY REGION HANDLER
;
; When rate error is positive (spacecraft rotating too fast in + direction),
; load absolute values of errors for phase-plane analysis. This normalizes
; the problem so subsequent logic works with positive quadrants only.
; ============================================================================

POSVEL		EXTEND
		DCA	EDOT		; Load double-precision rate error (positive)
		DXCH	EDOTVEL		; Store as EDOTVEL for phase-plane calculation
		CA	T5TEMP		; Load acceleration + bias term
		TS	ADBVEL		# +(ADB+AF) - positive acceleration bias
		CA	AERR		; Load attitude error
		TS	AERRVEL		; Store as AERRVEL for phase-plane test
		TC	J6.		; Continue to phase-plane boundary checks

; ============================================================================
; NEGATIVE VELOCITY REGION HANDLER
;
; When rate error is negative (spacecraft rotating too fast in - direction),
; negate all error values to normalize to positive quadrant for phase-plane
; analysis. The jet logic will compensate by firing opposite thrusters.
; ============================================================================

NEGVEL		EXTEND
		DCS	EDOT		; Load negated double-precision rate error
		DXCH	EDOTVEL		; Store negated rate as EDOTVEL (now positive)
		CS	T5TEMP		; Negate acceleration + bias term
		TS	ADBVEL		# -(ADB+AF) - reversed acceleration bias
		CS	AERR		; Negate attitude error
		TS	AERRVEL		; Store negated error (now positive magnitude)

; ============================================================================
; PHASE-PLANE BOUNDARY CALCULATION (J6.)
;
; This section performs the geometric tests to determine which region of the
; phase plane the current state occupies. The phase plane is divided by
; switching curves (boundaries) that define optimal jet firing zones. These
; boundaries are computed using rate error (EDOT), attitude error (AERR),
; acceleration bias (ADB), and slope parameters.
; ============================================================================

J6.		EXTEND
# Page 1031
		SU	ADB		; Compute (AERRVEL or ADBVEL) - ADB
		AD	WLH/SLOP	; Add width-of-limit-cycle / slope offset
		EXTEND
		BZMF	J8		; Branch if result negative or zero

; Test if attitude error exceeds acceleration-corrected threshold
		CS	T5TEMP		# -(ADB+AF) complemented acceleration term
		AD	AERRVEL		; Add attitude error: AERRVEL - (ADB+AF)
		EXTEND
		BZMF	+2		; Branch forward if negative (error small)
		TCF	J7		; Error large: go to J7 region

; Compute phase-plane switching curve intercept
; Calculate: -EDOTVEL/SLOPE - AERRVEL + ADB
; This determines if state is above or below the optimal switching boundary
		EXTEND
		DCS	EDOTVEL		; Load negated rate error (double precision)
		EXTEND
		DV	SLOPE		; Divide by slope: -EDOTVEL/SLOPE
		EXTEND
		SU	AERRVEL		; Subtract attitude error
		AD	ADB		; Add acceleration deadband
		EXTEND
		BZMF	J18		; Branch if below switching curve
		TCF	J23		; Above switching curve

; ============================================================================
; REGION J7 - LARGE ATTITUDE ERROR HANDLER
;
; When attitude error is large (exceeded threshold in J6.), perform alternate
; phase-plane test using different switching curve boundary calculation.
; This region typically represents states far from the origin requiring
; aggressive jet firing to return toward the deadband.
; ============================================================================

J7		CS	WL-H/SLP	; Load negative width-limit-high/slope
		EXTEND
		SU	T5TEMP		# Subtract (ADB+AF): compute threshold
		AD	AERRVEL		; Add attitude error for comparison
		EXTEND
		BZMF	J20		; Branch if below alternate switching curve
		TCF	J21		; Above alternate switching curve

; ============================================================================
; REGION J8 - WIDTH-OF-LIMIT-CYCLE BOUNDARY TEST
;
; This section tests if the rate error exceeds the width of the limit cycle.
; The limit cycle is the natural oscillation that occurs with minimum impulse
; control. If rate error is within the limit cycle width, no jets need to fire
; (spacecraft will coast into deadband naturally). If outside limit cycle,
; jets must fire to prevent overshoot.
; ============================================================================

J8		EXTEND
		DCS	WLH		; Load negative width-limit-high (double prec)
		DXCH	WTEMP		; Store in temporary location
		EXTEND
		DCA	EDOTVEL		; Load rate error velocity (normalized positive)
		DAS	WTEMP		; Add: WTEMP = EDOTVEL - WLH
		CCS	WTEMP		; Test high-order word of result
		TCF	J22		; Rate error exceeds limit cycle width (positive)
		TCF	SIGNCK2		; High word zero, check low word
		TCF	NJ22		; Rate error within limit cycle (negative)
SIGNCK2		CCS	WTEMP +1	; Test low-order word when high word is zero
		TCF	J22		; Low word positive -> outside limit cycle
		TCF	J22		; Low word +0 -> treat as outside limit cycle
		TCF	NJ22		; Low word negative -> inside limit cycle

; ============================================================================
; SLOPE BOUNDARY TEST (NJ22)
;
; Rate error is within the limit cycle width. Now test whether the state
; falls on the main switching slope boundary. This divides the phase plane
; into regions where jets should fire versus regions where no action is needed.
; The calculation projects the rate error onto the attitude axis using the
; slope constant, then checks if spacecraft state requires corrective action.
; ============================================================================

NJ22		EXTEND
		DCA	EDOTVEL		; Load normalized rate error (double-precision)
		EXTEND
		DV	SLOPE		; Divide by phase-plane slope (EDOT/SLOPE)
		AD	T5TEMP		# Add (ADB+AF) acceleration+bias term
		AD	AERRVEL		; Add attitude error
# Page 1032
		CCS	A		; Test combined error term
		TCF	J23		; Positive: jets needed for correction
		TCF	J23		; +0: treat as needing correction
		TCF	+2		; Negative: check width boundary
		TCF	J23		; -0: treat as needing correction

; Within slope boundary - now check if state is within width boundary.
; This secondary test ensures spacecraft isn't drifting too far from target.

		EXTEND
		DCS	WLMH		# WL - H (width limit minus half-width)
		DXCH	WTEMP		; Store in WTEMP for comparison
		EXTEND
		DCA	EDOTVEL		; Load rate error again
		DAS	WTEMP		; Add rate error to boundary term
		CCS	WTEMP		; Test if within width boundary
		TCF	J23		; Outside boundary: jets needed
		TCF	SIGNCK3		; High word zero: check low word
		TCF	NJ23		; Within boundary: check further conditions
SIGNCK3		CCS	WTEMP +1	; Test low-order word
		TCF	J23		; Low word positive: jets needed
		TCF	J23		; Low word +0: jets needed
		TCF	NJ23		; Low word negative: within boundary

; ============================================================================
; WIDTH BOUNDARY CHECK (NJ23)
;
; Final boundary test combining attitude error, acceleration/bias terms, and
; width-to-slope ratio. This determines whether spacecraft has drifted far
; enough from target to warrant jet firing, or is close enough to center that
; no corrective action is needed (conserving RCS propellant).
; ============================================================================

NJ23		CA	AERRVEL		; Load normalized attitude error
		AD	T5TEMP		# Add (ADB+AF) acceleration+bias term
		AD	WL-H/SLP	; Add width-to-slope ratio term
		CCS	A		; Test combined boundary term
		TCF	J24		; Positive: check forced firing logic
		TCF	J24		; +0: check forced firing
		TCF	J22		; Negative: within deadband, check rate damping
		TCF	J22		; -0: within deadband

; ============================================================================
; FORCED FIRING RATE SETUP (J18)
;
; Spacecraft requires corrective action based on slope boundary test. Load
; the rate error (EDOT) into KMPAC for jet on-time calculation. This branch
; handles cases where forced firing is needed to bring spacecraft back toward
; the target attitude deadband.
; ============================================================================

J18		EXTEND
		DCS	EDOT		; Load rate error (negated, double-precision)
		DXCH	KMPAC		; Store in KMPAC for JTIME calculation
		TCF	JTIME		; Branch to calculate jet on-time

; ============================================================================
; HYSTERESIS BOUNDARY CALCULATION (J20)
;
; Calculate the hysteresis switching boundary to prevent jet chatter (rapid
; on-off cycling). Hysteresis creates a band around the main switching line
; where jets remain on until well inside the deadband, then stay off until
; significantly outside. This is fundamental to phase-plane autopilot stability.
; ============================================================================

J20		CS	AERR		; Negate attitude error
		AD	ADBVEL		; Add acceleration+bias term
		EXTEND
		MP	SLOPE2		# Multiply by hysteresis slope constant
		DXCH	KMPAC		; Store in KMPAC
		EXTEND
		DCS	EDOT		; Load rate error (negated)
		DAS	KMPAC		; Add to KMPAC (double-add to single)
		TCF	JTIME		; Branch to calculate jet on-time

; ============================================================================
; RATE DIRECTION CHECK (J21)
;
; Determine whether rate error is positive or negative to select appropriate
; jet firing direction. Positive rate requires negative jets (to slow rotation)
; and vice versa. This ensures jets fire in the correct direction to null
; the rate error.
; ============================================================================

J21		CCS	EDOT		; Test rate error sign
		TCF	JP		; Positive rate: fire negative jets
		TCF	SIGNCK4		; +0: check low-order word
		TCF	JN		; Negative rate: fire positive jets
SIGNCK4		CCS	EDOT +1		; Test low-order word
# Page 1033
		TCF	JP		; High word positive: fire negative jets
		TCF	JP		; High word +0: fire negative jets
		TCF	JN		; High word negative: fire positive jets

; ============================================================================
; NEGATIVE JET FIRING COMMAND (JN)
;
; Rate error is negative (spacecraft rotating in negative direction). Fire
; positive jets to add angular momentum and null the rate. Adjust the rate
; error by the limit cycle width (WL) to account for minimum impulse jet
; firing characteristics.
; ============================================================================

JN		EXTEND
		DCS	EDOT		; Load rate error (negated)
		DXCH	KMPAC		; Store in KMPAC
		EXTEND
		DCA	WL		; Load limit cycle width
		DAS	KMPAC		; Add to KMPAC (positive adjustment)
		TCF	JTIME		; Calculate jet on-time

; ============================================================================
; POSITIVE JET FIRING COMMAND (JP)
;
; Rate error is positive (spacecraft rotating in positive direction). Fire
; negative jets to remove angular momentum and null the rate. Adjust the rate
; error by the negative limit cycle width to account for minimum impulse
; jet firing characteristics.
; ============================================================================

JP		EXTEND
		DCS	EDOT		; Load rate error (negated)
		DXCH	KMPAC		; Store in KMPAC
		EXTEND
		DCS	WL		; Load limit cycle width (negated)
		DAS	KMPAC		; Add to KMPAC (negative adjustment)
		TCF	JTIME		; Calculate jet on-time

; ============================================================================
; RATE DIRECTION CHECK AFTER DEADBAND (J22)
;
; Similar to J21, determine whether rate error is positive or negative to
; select appropriate jet firing direction after determining spacecraft is
; within attitude deadband but rate needs damping.
; ============================================================================

J22		CCS	EDOT		; Test rate error sign
		TCF	JN		; Positive rate: fire negative jets
		TCF	SIGNCK5		; +0: check low-order word
		TCF	JP		; Negative rate: fire positive jets
SIGNCK5		CCS	EDOT +1		; Test low-order word
		TCF	JN		; Low word positive: fire negative jets
		TCF	JN		; Low word +0: fire negative jets
		TCF	JP		; Low word negative: fire positive jets
		TCF	JP		; Low word -0: fire positive jets

; ============================================================================
; RATE DAMPING FLAG CLEAR AND FORCED FIRING CHECK (J23)
;
; Spacecraft has reached stable attitude. Clear the axis-specific rate damping
; flag using indexed addressing (BIT13 for roll, BIT12 for pitch, BIT11 for
; yaw based on SPNDX). Then check if forced firing is commanded for this axis.
; If so, execute J18 to generate jet firing command. Otherwise, exit without
; firing jets (conserving RCS propellant).
; ============================================================================

J23		INDEX	SPNDX		; Use SPNDX to select axis-specific bit
		CS	BIT13		# Complement of rate damping flag bit
		MASK	RCSFLAGS	# Clear axis-specific flag from RCSFLAGS
		TS	RCSFLAGS	# Store modified flags
					# BIT13 for roll  (SPNDX = 0)
					# BIT12 for pitch (SPNDX = 1)
					# BIT11 for yaw   (SPNDX = 2)

		INDEX	SPNDX		; Use SPNDX to select axis-specific bit
		CAF	OCT01400	# Load forced firing flag mask
		MASK	RCSFLAGS	# Check if forced firing flag is set
		EXTEND
		BZF	DOJET +2	# Flag clear: no forced firing, exit

		TCF	J18		# Flag set: branch to J18 for forced firing

; ============================================================================
; HYSTERESIS BOUNDARY WITH DEADBAND VELOCITY (J24)
;
; Calculate rate command based on hysteresis slope and attitude error relative
; to the deadband velocity boundary: EDOT = (-AERR - ADBVEL) * SLOPE2.
; This forms the switching line for entering/exiting rate damping mode,
; implementing hysteresis to prevent oscillatory behavior at boundaries.
; The computed rate error (stored in KMPAC) determines jet on-time.
; ============================================================================

J24		CS	AERR		; Load attitude error (complemented)
		EXTEND
		SU	ADBVEL		; Subtract deadband velocity: -AERR - ADBVEL
		EXTEND
		MP	SLOPE2		# Multiply by hysteresis slope
		DXCH	KMPAC		; Store result in KMPAC
		EXTEND
# Page 1034
		DCS	EDOT		; Load rate error (double complemented)
		DAS	KMPAC		; Add to KMPAC: final rate error for jet time

# Page 1035
# 	COMPUTE THE JET ON TIME NECESSARY TO ACCOMPLISH THE DESIRED CHANGE IN RATE, IE
#
#	     T  = J/M(DELTA W)
#	      J
#
#	DELTA W = DESIRED CHANGE IN S/C ANGULAR RATE AS DETERMINED BY THE
#		  SWITCHING LOGIC, AT THIS POINT STORED IN KMPAC.
#
#	    J/M = S/C INERTIA TO TORQUE 9ATIO SCALED BY
#		  	(57.3/450)(B24/1600)(1/.8)
#		  FOR 1 JET OPERATION  (M = 700 FT-LB).
#		  IE  J/M = J(SLUG-FTFT) x 0.00000085601606
#
#	          THE CORRESPONDING COMPUTER VARIABLES ESTABLISHED BY
#		  KEYBOARD ENTRY ARE
#			J/M (ROLL)
#			J/M1 (PITCH)
#			J/M2 (YAW)
#
#	     T  = JET-ON TIME    SCALED 16384/1600 SEC
#	      J
#
#	          THE COMPUTER VARIABLES ARE
#			TAU  (ROLL)
#			TAU1 (PITCH)
#			TAU2 (YAW)

; ============================================================================
; JET ON-TIME CALCULATION (JTIME)
;
; Calculate required jet firing time to null the rate error: T = (J/M) * DELTA_W
; where J/M is spacecraft inertia-to-torque ratio and DELTA_W is the desired
; angular rate change computed by the phase-plane switching logic.
;
; The inertia-to-torque ratio is scaled for single-jet operation (700 ft-lb
; torque). During Apollo 11, these values were uplinked from Mission Control
; based on updated mass properties as propellant was consumed. The computed
; jet on-time (TAU, TAU1, TAU2) is scaled in units of 16384/1600 seconds.
; ============================================================================

JTIME		INDEX	SPNDX		# Select axis-specific inertia/torque ratio
		CA	J/M		# Load J/M (roll), J/M1 (pitch), or J/M2 (yaw)
		TC	SMALLMP		# Multiply by KMPAC (rate error)
		CA	BIT11		; Load scaling factor
		TC	SMALLMP		# Complete multiplication for jet time
		CCS	KMPAC		; Test for overflow in high-order word
		TCF	+4		; Positive overflow: limit to POSMAX
		TCF	TAUNORM		; +0: use computed value
		TCF	+4		; Negative overflow: limit to NEGMAX
		TCF	TAUNORM		; -0: use computed value
		CA	POSMAX		; Load maximum positive jet time
		TCF	DOJET		; Store and continue
		CA	NEGMAX		; Load maximum negative jet time
		TCF	DOJET		; Store and continue

; Store computed jet on-time for current axis and continue to next axis
TAUNORM		CA	KMPAC +1	; Load low-order word (normal range)
DOJET		INDEX	SPNDX		; Use SPNDX to select axis storage
		TS	TAU		; Store jet time: TAU (roll), TAU1 (pitch), TAU2 (yaw)
		CCS	SPNDX		; Decrement axis counter
		TCF	JLOOP		; More axes to process: loop
		TCF	T6PROG		; All axes complete: proceed to T6 program

# Page 1036
; ============================================================================
; ZERO ROTATION COMMANDS (ZEROCMDS)
;
; Clear all jet firing time commands when spacecraft is within deadband and
; no maneuver is required. This entry point prevents unnecessary RCS jet
; firings, conserving propellant during stable attitude hold phases.
; ============================================================================

ZEROCMDS	CAF	ZERO		; Load zero
		TS	TAU		; Clear roll jet time command
		TS	TAU1		; Clear pitch jet time command
		TS	TAU2		; Clear yaw jet time command

; ============================================================================
; T6 PROGRAM SETUP AND RETURN (T6PROG)
;
; Transfer control to the jet selection logic (JETSLECT) for execution of
; computed rotation commands. The T5LOC is reset to point to JETSLECT for
; Phase 3 autopilot execution. After automatic maneuver computation completes,
; control returns to the calling program via RESUME.
; ============================================================================

T6PROG		EXTEND			# Rotation commands (TAU, TAU1, TAU2) ready
		DCA	JETADDR		# Load address of jet selection routine
		DXCH	T5LOC		# Store as next T5 interrupt handler
		TCF	RESUME		; Return to calling program

		EBANK=	KMPAC
JETADDR		2CADR	JETSLECT	; Address of jet selection logic
