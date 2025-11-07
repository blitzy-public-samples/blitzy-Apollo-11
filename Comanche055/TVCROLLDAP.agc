# Copyright:	Public domain.
# Filename:	TVCROLLDAP.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	984-998
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
; FILE: TVCROLLDAP.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth (SPS burns)
;
; TL;DR: Roll axis digital autopilot controlling spacecraft roll attitude during
;        Service Propulsion System (SPS) burns. Since the SPS engine gimbals only
;        in pitch and yaw, RCS thrusters provide roll control. Implements phase
;        plane switching logic to maintain roll within 5-degree deadband while
;        minimizing propellant usage. Critical for Apollo 11 translunar injection,
;        lunar orbit insertion, and transearth injection burn accuracy.
;
; COMMENT-ONLY READERS: This autopilot prevented the spacecraft from spinning
;        during major rocket burns. While the main engine steered in two directions,
;        small thrusters controlled rotation to keep the spacecraft properly oriented.
;
; CODE-ALONG READERS: Study phase plane control implementation with switching
;        parabolas, RCS jet alternation strategy, and coordination with TVC pitch/yaw
;        control loops. Note the limit cycle suppression and propellant conservation
;        through minimum/maximum firing time constraints.
; ============================================================================

; ============================================================================
; FILE: TVCROLLDAP.agc
; MODULE: TVCDAPS Subsystem (Thrust Vector Control and Digital Autopilot)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth (SPS burns)
;
; TL;DR: Roll axis digital autopilot controlling spacecraft roll attitude during
;        Service Propulsion System (SPS) engine burns. Uses RCS thrusters in a
;        phase plane switching control scheme to maintain roll orientation within
;        5-degree deadband while pitch and yaw are controlled by engine gimbal.
;        Critical for Apollo 11 translunar injection, lunar orbit insertion,
;        transearth injection, and midcourse correction burns.
;
; COMMENT-ONLY READERS: During major rocket burns that steered the spacecraft
;        to and from the Moon, the main engine could tilt in two directions but
;        not roll. This autopilot fired small side thrusters to prevent the
;        spacecraft from spinning during these critical maneuvers.
;
; CODE-ALONG READERS: Study phase plane control theory implementation with
;        twelve switching regions, RCS jet pair selection alternation, T6 timer
;        integration, and coordination with TVC pitch/yaw gimbal control loops.
; ============================================================================

# Page 984
# PROGRAM NAME...TVC ROLL AUTOPILOT
# LOG SECTION...TVCROLLDAP			SUBROUTINE...DAPCSM
# MOD BY SCHLUNDT				21 OCTOBER 1968


; ============================================================================
; ROLL AUTOPILOT OPERATIONAL CONTEXT
;
; During Apollo 11's critical burns (translunar injection on July 16, 1969,
; lunar orbit insertion on July 19, and transearth injection on July 21),
; the Service Propulsion System (SPS) main engine fired while gimbaling in
; pitch and yaw for steering. However, the SPS cannot control roll rotation.
; This autopilot uses RCS thrusters to maintain roll attitude while the main
; engine operates, preventing unwanted spacecraft spin that would degrade
; burn accuracy and potentially cause mission failure.
;
; The autopilot coordinates with:
; - TVCDAPS.agc: Pitch and yaw TVC control using SPS gimbal actuators
; - RCS-CSM_DIGITAL_AUTOPILOT.agc: RCS thruster firing logic
; - TVCEXEC: Executive that calls this routine every 0.5 seconds
; ============================================================================

# FUNCTIONAL DESCRIPTION....

#      *AN ADAPTATION OF THE LEM P-AXIS CONTROLLER
#      *MAINTAIN OGA WITHIN 5 DEG DEADBND OF OGAD, WHERE OGAD = OGA AS SEEN
#	BY IGNITION (P40)
#      *MAINTAIN OGA RATE LESS THAN 0.1 DEG/SEC LIMIT CYCLE RATE
#      *SWITCHING LOGIC IN PHASE PLANE.... SEE GSOP CHAPTER 3
#      *USES T6 CLOCK TO TIME JET FIRINGS
#      *MAXIMUM JET FIRING TIME = 2.56 SECONDS, LIMITED TO 2.5 IF GREATER
#      *MINIMUM JET FIRING TIME = 15 MS
#      *JET PAIRS FIRE ALTERNATELY
#      *AT LEAST 1/2 SECOND DELAY BEFORE A NEW JET PAIR IS FIRED
#      *JET FIRINGS MAY NOT BE EXTENDED, ONLY SHORTENED, WHEN RE-EVALUATION
#	OF A JET FIRING TIME IS MADE ON A LATER PASS

; Key Parameters Explained:
; - OGA (Outer Gimbal Angle): Current roll attitude from CDU (Coupling Data Unit)
; - OGAD (Outer Gimbal Angle Desired): Target roll attitude set at ignition
; - OGAERROR: Difference between desired and actual roll (OGAD - OGA)
; - OGARATE: Roll rate in degrees per second
; - Deadband: ±5 degrees tolerance zone where no control action taken
; - Limit cycle: 0.1 deg/sec maximum rate to prevent oscillations
;
; Control Strategy:
; The autopilot maintains roll within ±5 degrees of the target attitude while
; keeping rotation rate below 0.1 deg/sec. This prevents both steady-state
; errors and oscillatory "hunting" behavior that wastes propellant. RCS jets
; fire in carefully timed pulses (15 milliseconds to 2.5 seconds) with pairs
; alternating to balance propellant consumption across thruster quads.

; ============================================================================
; EXECUTION TIMING AND SYSTEM INTEGRATION
;
; The roll autopilot executes as a WAITLIST task triggered every 0.5 seconds
; by TVCEXECUTIVE. A 3-centisecond (30-millisecond) delay prevents conflict
; with other time-critical interrupts like downlink telemetry. During Apollo 11's
; burns, this half-second sample rate provided sufficient control bandwidth while
; leaving AGC cycles for navigation updates and crew display management.
;
; Restart Protection:
; If a restart occurs during an SPS burn (due to power transient or alarm),
; roll DAP computations suspend until the next scheduled execution. The target
; roll attitude (OGAD) set at ignition is preserved, ensuring the spacecraft
; returns to correct orientation after recovery. This restart protection was
; critical - any loss of attitude control during translunar injection would
; have aborted the mission to the Moon.
; ============================================================================

# CALLING SEQUENCE....

#      *ROLLDAP CALL VIA WAITLIST, IN PARTICULAR BY TVCEXEC (EVERY 1/2 SEC)
#	WITH A 3CS DELAY TO ALLOW FREE TIME FOR OTHER RUPTS (DWNRPT, ETC.)

# NORMAL EXIT MODES.... ENDOFJOB

# ALARM OR ABORT EXIT MODES.... NONE

# SUBROUTINES CALLED.....NONE

# OTHER INTERFACES....

#      *TVCEXEC SETS UP ROLLDAP TASK EVERY 1/2 SECOND AND UPDATES 1/CONACC
#	EVERY 10 SECONDS (VIA MASSPROP AND S40.15)
#      *RESTARTS SUSPEND ROLL DAP COMPUTATIONS UNTIL THE NEXT 1/2 SEC
#	SAMPLE PERIOD.  (THE PART OF TVCEXECUTIVE THAT CALLS ROLL DAP IS
#	NOT RESTARTED.)  THE OGAD FROM IGNITION IS MAINTAINED.

# ERASABLE INITIALIZATION REQUIRED....

#      *1/CONACC (S40.15)
#      *OGAD (CDUX AT IGNITION)
#      *OGANOW (CDUX AT TVCINIT4 AND TVCEXECUTIVE)
#      *OGAPAST (OGANOW AT TVCEXECUTIVE)
#      *ROLLFIRE = TEMREG = ROLLWORD = 0 (MRCLEAN LOOP IN TVCDAPON)
#
# OUTPUT....

#      *ROLL JET PAIR FIRINGS

; Input State Variables Explained:
; - 1/CONACC: Inverse of control acceleration (updated every 10 seconds based on
;   current spacecraft mass as propellant is consumed during burn)
; - OGAD: Target roll angle captured at ignition start
; - OGANOW: Current roll angle from IMU (updated by TVCEXECUTIVE)
; - OGAPAST: Previous roll angle (used for rate estimation)
; - ROLLFIRE: Current jet firing status (0 = no jets firing)
; - TEMREG: Temporary register for jet fire timing
; - ROLLWORD: Bit pattern selecting which RCS jet pair to fire
;
; Output Control Actions:
; Commands to RCS roll jet pairs (jets 5, 6, 7, 8 in SM RCS quads). Jet pairs
; fire alternately to balance propellant usage and thermal heating across the
; four RCS quads mounted around the Service Module.

# Page 985
# DEBRIS.... MISCELLANEOUS, SHAREABLE WITH RCS/ENTRY, IN EBANK6 ONLY

; ============================================================================
; OPERATIONAL CONTEXT AND MISSION INTEGRATION
;
; During Apollo 11's journey to the Moon and back, the Service Propulsion
; System (SPS) engine executed several critical burns including translunar
; injection (TLI), lunar orbit insertion (LOI), transearth injection (TEI),
; and midcourse corrections. The SPS engine gimbal could provide pitch and
; yaw control through thrust vector deflection, but roll axis control required
; RCS thruster firings.
;
; This roll autopilot worked in coordination with TVCDAPS.agc (pitch and yaw
; control loops) to maintain three-axis attitude stability. The pitch/yaw DAPs
; commanded the engine gimbal actuators while this roll DAP commanded the RCS
; jets. All three axes used the same 1/2-second sample rate (set by TVCEXEC)
; to ensure coordinated control.
;
; KEY DESIGN PARAMETERS:
; - Deadband: ±5 degrees from desired roll attitude (OGAD)
; - Rate limit: 0.1 deg/sec for limit cycle control
; - Sample period: 0.5 seconds (500 milliseconds)
; - Minimum jet fire time: 14.08 milliseconds (TMINFIRE)
; - Maximum jet fire time: 2.5 seconds (TMAXFIRE)
; - Jet pair alternation: Jets fire alternately to balance propellant usage
; - Firing delay: 0.5 seconds minimum between different jet pair firings
;
; COORDINATION WITH OTHER SYSTEMS:
; - TVCEXEC: Calls ROLLDAP every 1/2 second via WAITLIST task
; - MASSPROP: Updates spacecraft inertia (1/CONACC) every 10 seconds
; - RCS-CSM autopilot: Shares RCS thruster hardware during coast phases
; - RESTART protection: Suspends computations until next sample period
; ============================================================================

# Page 986

; ============================================================================
; TRANSITION: From operational overview to control theory implementation
;
; The roll autopilot uses a sophisticated phase plane switching control law
; adapted from the Lunar Module P-axis controller. Rather than simple on/off
; control, the autopilot divides the phase plane (attitude error vs. rate)
; into twelve distinct regions, each with specific control actions. This
; approach minimizes propellant usage while maintaining tight attitude control.
;
; The following section provides detailed explanation of the phase plane
; geometry and switching logic. Code-along readers should study this theory
; to understand the region testing logic implemented in the ROLLDAP code.
; Comment-only readers can skim to grasp that this is an advanced control
; technique balancing accuracy with fuel efficiency.
; ============================================================================

# SOME NOTES ON THE ROLL AUTOPILOT, AND IN PARTICULAR, ON ITS SWITCHING
# LOGIC.  SEE SECTION THREE OF THE GSOP (SUNDISK/COLOSSUS) FOR DETAILS.

; ============================================================================
; TRANSITION: From System Overview to Phase Plane Control Theory
;
; The roll autopilot uses sophisticated phase plane control logic rather than
; simple proportional control. In the phase plane, the vertical axis represents
; roll rate (OGARATE) and the horizontal axis represents roll error (OGAERROR).
; The spacecraft's current state is a point in this two-dimensional space.
;
; Switching parabolas divide the phase plane into different control regions.
; Depending on which region contains the current state, the autopilot decides
; whether to fire positive roll jets, negative roll jets, or coast without
; thrust. This approach minimizes propellant consumption while ensuring the
; spacecraft reaches the target attitude without overshoot or oscillation.
;
; The following ASCII art diagram shows these control regions. This mathematical
; framework, though complex, was essential for Apollo's success - conventional
; control would have either wasted excessive propellant or failed to maintain
; adequate attitude accuracy during the long translunar coast burns.
; ============================================================================

# SWITCHING LOGIC IN THE PHASE PLANE....

#                              OGARATE
#                                 *
#                                 *
#  * * * * * * * * * * *          *
#                                 *     (REGION 1, SEE TEXT BELOW)
#                            *    *
#                                 *
# * * * * * * *     (COAST)       *     ...PARABOLA (SWITCHING = CONTROL)
#               *                 *    .
#                 *               *   *
#                   *             *                (FIRE NEG ROLL JETS)
#                     *           *      *
#     (-DB,+LMCRATE)....*         *
#                       *         *        *
#                       *         *			         OGAERROR
# ************************************************************************
#                                 *         *                (-AK, OGAERR)
#                        *        *         *      (REGION 6-PRIME)
#                                 *         *      (SEE TEXT BELOW)
#                          *      *           *
#                                 *             *     ...STRAIGHT LINE
#    (FIRE POS ROLL JETS)     *   *               *  .
#                                 *   (COAST)       *
#                                 *                   * * * * * * * * * *
#                                 *                         -MINLIM
#                                 *    *
#                                 *
#                                 *          * * * * * * * * * * * * * * *
#                                 *                        -MAXLIM
#                                 *
#                                 *

# SWITCHING PARABOLAS ARE CONTROL PARABOLAS, THUS REQUIRING KNOWLEDGE OF
#	CONTROL ACCELERATION CONACC, OR ITS RECIPROCAL, 1/CONACC, THE TVC
#	ROLL DAP GAIN (SEE TVCEXECUTIVE VARIABLE GAIN PACKAGE).  JET
#	FIRING TIME IS SIMPLY THAT REQUIRED TO ACHIEVE THE DESIRED OGARATE,
#	SUBJECT TO THE LIMITATIONS DISCUSSED UNDER FUNCTIONAL DESCRIPTION,
#	ABOVE.

# THE THREE CONTROL REGIONS (+, -, AND ZERO TORQUE) ARE COMPRIZED OF
#	TWELVE SUBSET REGIONS ( 1...6, AND THE CORRESPONDING 1-PRIME...
#	6-PRIME ) SEE SECTION 3 OF THE GSOP (SUNDISK OR COLOSSUS)
# Page 987
#
# GIVEN THE OPERATING POINT NOT IN THE COAST REGION, THE DESIRED OGARATE
#	IS AT THE POINT OF PENETRATION OF THE COAST REGION BY THE CONTROL
#	PARABOLA WHICH PASSES THROUGH THE OPERATING POINT.  FOR REGION 3
#	DESIRED OGARATE IS SIMPLY +-MAXLIM.  FOR REGIONS 1 OR 6 THE SOLUTION
#	TO A QUADRATIC IS REQUIRED (THE PENETRATION IS ALONG THE STRAIGHT
#	LINE OR MINLIM BOUNDRY SWITCH LINES).  AN APPROXIMATION IS MADE
#	INSTEAD.  TAKE AN OPERATING POINT IN REGION 6' .  PASS A TANGENT TO
#	THE CONTROL PARABOLA THROUGH THE OPERATING POINT, AND FIND ITS
#	INTERSECTION WITH THE STRAIGHT LINE SECTION OF THE SWITCH CURVE...
#	THE INTERSECTION DEFINES THE DESIRED OGARATE.  IF THE OPERATING POINT IS
#	CLOSE TO THE SWITCH LINE, THE APPROXIMATION IS QUITE GOOD (INDEED
#	THE APPROXIMATE AND QUADRATIC SOLUTIONS CONVERGE IN THE LIMIT AS
#	THE SWITCH LINE IS APPROACHED).  IF THE OPERATING POINT IS NOT CLOSE
#	TO THE SWITCH LINE, THE APPROXIMATE SOLUTION GIVES VALID TREND
#	INFORMATION (DIRECTION OF DESIRED OGARATE) AT LEAST.  THE
#	RE-EVALUATION OF DESIRED OGARATE IN SUBSEQUENT ROLL DAP PASSES (1/2
#	SECOND INTERVALS) WILL BENEFIT FROM THE CONVERGENT NATURE OF THE
#	APPROXIMATION.

# FOR LARGE OGAERROR THE TANGENT INTERSECTS +-MINLIM SWITCH BOUNDARY BEFORE
#	INTERSECTING THE STRAIGHT LINE SWITCH.  HOWEVER THE MINLIM IS
#	IGNORED IN COMPUTING THE FIRING TIME, SO THAT THE EXTENSION (INTO
#	THE COAST REGION) OF THE STRAIGHT LINE SWITCH IS WHAT IS FIRED TO.
#	IF THE ROLL DAP FINDS ITSELF IN THE COAST REGION BEFORE REACHING
#	THE DESIRED INTERSECTION (IE. IN THE REGION BETWEEN THE MINLIM
#	AND THE STRAIGHT LINE SWITCH) IT WILL EXHIBIT NORMAL COAST-REGION
#	BEHAVIOR AND TURN OFF THE JETS.  THE PURPOSE OF THIS FIRING POLICY
#	IS TO MAINTAIN STATIC ROLL STABILITY IN THE EVENT OF A JET
#	FAILED-ON.

# WHEN THE OPERATING POINT IS IN REGION 1 THE SAME APPROXIMATION IS
#	MADE, BUT AT AN ARTIFICIALLY-CREATED OR DUMMY OPERATING POINT,
#	DEFINED BY.. OGAERROR = INTERSECTION OF CONTROL PARABOLA AND
#	OGAERROR AXIS, OGARATE = +-LMCRATE WHERE SIGN IS OPPOSITE THAT OF
#	REAL OPERATING POINT RATE.  WHEN THE OPERATING POINT HAS PASSED
#	FROM REGION 1 TO REGION 6', THE DUMMY POINT IS NO LONGER REQUIRED,
#	AND THE SOLUTION REVERTS TO THAT OF A REGULAR REGION 6' POINT.


# EQUATION FOR SWITCHING PARABOLA (SEE FIGURE ABOVE)....
#				  2
#	SOGAERROR = (DB - (SOGARATE) (1/CONACC)/2) SGN(SOGARATE)


# EQUATION FOR SWITCHING STRAIGHT LINE SEGMENT....

#	SOGARATE = -(-SLOPE)(SOGAERROR) - SGN(SOGARATE) INTERCEP

#		WHERE  INTERCEP = DB(-SLOPE) - LMCRATE
# Page 988
#
# EQUATION FOR INTERSECTION, CONTROL PARABOLA AND STRAIGHT SWITCH LINE....

#	DOGADOT = NUM/DEN, WHERE
#				   2
#	    NUM = (-SLOPE)(OGARATE) (1/CONACC)
#		    +SGN(DELOGA)(-SLOPE)(OGAERROR - SGN(DELOGA)(DB))
#		    +LMCRATE

#	    DEN = (-SLOPE)(LMCRATE)(1/CONACC) - SGN(DELOGA)

#					      2
#	    DELOGA = OGAERROR - (DB - (OGADOT) (1/CONACC)/2) SGN(OGARATE)


# FOR REGIONS 6 AND 6-PRIME USE ACTUAL OPERATING POINT (OGA, OGARATE)
#	FOR OGAERROR AND OGARATE IN THE INTERSECTION EQUATIONS ABOVE.
#	FOR REGIONS 1 AND 1-PRIME USE DUMMY OPERATING POINT FOR OGAERROR
#	AND OGARATE, WHERE THE DUMMY POINT IS GIVEN BY....
#	OGAERROR= DELOGA + DB SGN(OGARATE)
#	OGARATE= -LMCRATE SGN(OGARATE)
#
# NOTE, OGAERROR = OGA - OGAD USES DUMMY REGISTER OGA IN ROLL DAP CODING
#	ALSO, AT POINT WHERE DOGARATE IS COMPUTED, REGISTER DELOGA IS USED
#	AS A DUMMY REGISTER FOR THE OGAERROR IN THE NUM EQUATION ABOVE
# Page 989

# ROLLDAP CODING....

		SETLOC	DAPROLL
		BANK
		EBANK=	OGANOW
		COUNT*	$$/ROLL

; ============================================================================
; ROLLDAP MAIN ENTRY POINT - Roll Control Cycle Execution
;
; Called every 0.5 seconds by TVCEXEC via WAITLIST task. This routine
; performs complete roll axis control evaluation and jet firing commands.
;
; EXECUTION SEQUENCE:
; 1. Estimate roll rate from position change (OGANOW - OGAPAST)
; 2. Check if jets currently firing and evaluate timing constraints
; 3. Compute phase plane position and distance from switching curve
; 4. Determine control region (1 of 12 regions)
; 5. Calculate desired roll rate and required jet fire time
; 6. Select and command appropriate RCS jet pair
; 7. Set up T6 timer for jet shutoff
;
; For comment-only readers: Every half-second during an engine burn, this
; code wakes up, checks how fast the spacecraft is rotating, and decides
; whether to fire small thrusters to correct the rotation.
; ============================================================================

ROLLDAP		CAE	OGANOW		# OGA RATE ESTIMATOR...SIMPLE FIRST-ORDER
		EXTEND			#	DIFFERENCE (SAMPLE TIME = 1/2 SEC)
		MSU	OGAPAST
		EXTEND
		MP	BIT5
		LXCH	A
		TS	OGARATE		# SC.AT B-4 REV/SEC

; Roll rate estimation: Simple first-order difference over 0.5 second sample.
; OGARATE = (OGANOW - OGAPAST) * 2  (scaled at B-4 revolutions/second)
; OGANOW = current roll gimbal angle from CDU (updated by TVCEXEC)
; OGAPAST = roll angle from previous sample (0.5 seconds ago)
; BIT5 = octal 20 = decimal 16, used here to scale the difference by 2
; Result stored in OGARATE represents roll angular velocity

# COMPUTATIONS WHICH FOLLOW USE OGA FOR OGAERR (SAME REGISTER)
# EXAMINE DURATION OF LAST JET FIRING IF JETS ARE NOW ON.

; ============================================================================
; JET FIRING DURATION CHECK
;
; Before computing new control commands, check the status of RCS jets:
; - If jets currently firing: proceed to control logic evaluation
; - If jets just turned off: enforce 0.5-second minimum delay before next fire
; - If delay satisfied: proceed to control logic
;
; This prevents rapid on-off cycling which would waste propellant and wear
; out thruster valves. The 0.5-second "coast" period between firings also
; allows propellant lines to stabilize and provides time for crew monitoring.
;
; For comment-only readers: The autopilot enforces a rest period between
; thruster firings to avoid wasting fuel with rapid pulsing.
; ============================================================================

DURATION	CA	ROLLFIRE	# SAME SGN AS PRESENT TORQ,MAGN=POSMAX
		EXTEND
		BZF	+2		# ROLL JETS ARE NOW OFF.
		TCF	ROLLOGIC	# ENTER LOGIC,JETS NOW ON.

; ROLLFIRE register indicates jet firing status and direction:
; Zero = jets off, Positive = firing one direction, Negative = firing opposite

		CAE	TEMREG		# EXAMINE LAST FIRING INTERVAL
		EXTEND			# IF POSITIVE, DONT FIRE
		BZF	ROLLOGIC	# ENTER LOGIC, JETS NOW OFF.

; TEMREG serves as timer for enforcing 0.5-second delay between firings.
; Positive value means insufficient time elapsed since last jet shutoff.
; Zero means either: (1) jets currently off and delay satisfied, or
; (2) no recent firing. Safe to proceed with control logic.

		CAF	ZERO		# JETS HAVE NOT BEEN OFF FOR 1/2 SEC. WAIT
		TS	TEMREG		# RESET TEMREG
WAIT1/2		TCF	TASKOVER	# EXIT ROLL DAP

; Jets turned off less than 0.5 seconds ago. Skip this control cycle and
; exit via TASKOVER. Will re-evaluate on next ROLLDAP call (0.5 sec later).

# COMPUTE DB-(1/2 CONACC) (OGARATE)SQ  (1/2 IN THE SCALING)

; ============================================================================
; ROLLOGIC - Phase Plane Control Logic Entry
;
; This is the heart of the switching control law. The autopilot divides the
; phase plane (attitude error vs. rate) into twelve regions separated by
; parabolic switching curves. The control action depends on which region the
; spacecraft state occupies.
;
; First step: Compute distance from the switching parabola. This parabola
; divides phase plane into "need to fire jets" vs. "coast" regions.
;
; Formula: TEMREG = DB - (1/2 * CONACC * OGARATE²)
; where:
;   DB = Deadband (5 degrees = 0.01389 revolutions)
;   CONACC = Angular acceleration from one RCS jet pair
;   OGARATE = Current roll rate
;
; Physical interpretation: The parabola represents the boundary where, if
; spacecraft continues at current rate, it will exactly reach the deadband
; edge with zero rate. Inside parabola = need to slow down. Outside = coast.
; ============================================================================

ROLLOGIC	CS	OGARATE		# SCALED AT 2(-4) REV/SEC
		EXTEND
		MP	1/CONACC	# SCALED AT 2(+9) SEC SQ /REV
		EXTEND
		MP	OGARATE
		AD	DB		# SCALED AT 2(+0) REV
		TS	TEMREG		# QUANTITY SCALED AT 2(+0) REV.

; Computation breakdown:
; Step 1: -(OGARATE) * (1/CONACC) → scaled at 2(+5) sec
; Step 2: [result] * OGARATE → scaled at 2(+1) revolutions
; Step 3: Add DB (2(+0) revolutions) → final scaled at 2(+0) revolutions
; TEMREG > 0: Outside parabola (coast or accelerate)
; TEMREG < 0: Inside parabola (decelerate, fire opposing jets)

# GET SIGN OF OGARATE

		CA	OGARATE
		EXTEND
		BZMF	+3		# LET SGN(0) BE NEGATIVE
		CA	BIT1
		TCF	+2
		CS	BIT1
		TS	SGNRT		# + OR -  2(-14)

; Extract rate sign for subsequent region determination.
; SGNRT = +1 if OGARATE > 0 (rolling positive direction)
; SGNRT = -1 if OGARATE ≤ 0 (rolling negative direction or stopped)
; Convention: Zero rate treated as negative for control symmetry.

# Page 990
# CALCULATE DISTANCE FROM SWITCH PARABOLA (DELOGA)

; ============================================================================
; DELOGA CALCULATION - Horizontal Distance from Switching Curve
;
; Now compute signed distance from the control switching parabola in the
; horizontal (attitude error) direction of the phase plane.
;
; Formula: DELOGA = OGA - SGNRT * TEMREG
; where:
;   OGA = Current roll attitude error (same as OGAERR, computed later)
;   SGNRT = Sign of roll rate (+1 or -1)
;   TEMREG = Distance from parabola (computed earlier)
;
; Physical interpretation: DELOGA indicates which of 12 control regions
; the spacecraft occupies. Combined with rate sign and TEMREG, this
; determines the required control action (fire jets, coast, or wait).
; ============================================================================

		EXTEND
		MP	TEMREG		# SGN(OGARATE) TEMREG NOW IN L
		CS	L
		AD	OGA		# SCALED AT 2(+0) REV
DELOGAC		TS	DELOGA		# SC.AT B+0 REV, PLUS TO RIGHT OF C-PARAB

; Computation: SGNRT * TEMREG → L register, then OGA - L → DELOGA
; Positive DELOGA: Right of control parabola in phase plane
; Negative DELOGA: Left of control parabola in phase plane

# EXAMINE SGN(DELOGA) AND CREATE CA OR CS INSTR. DEPENDING UPON SIGN.

; ============================================================================
; PHASE PLANE REGION DETERMINATION - Stage 1
;
; The phase plane is divided into 12 regions by parabolic switching curves.
; Control action depends on region location. This section begins region tests.
;
; First, construct a self-modifying instruction based on DELOGA sign:
; - If DELOGA ≥ 0: Create CA instruction (load value)
; - If DELOGA < 0: Create CS instruction (load complement)
;
; This technique allows one code path to handle symmetric regions by using
; CA or CS dynamically. The I register holds the instruction to execute.
; ============================================================================

		EXTEND
		BZMF	+3
		CAF	PRIO30		# =CA (30000)
		TCF	+2
		CAF	BIT15		# =CS (40000)
		TS	I

; PRIO30 = octal 30000 = CA instruction opcode
; BIT15 = octal 40000 = CS instruction opcode
; Stored in I register for indexed indirect execution

		INDEX	I		# TSET ON I SGN(OGARATE)
		0	SGNRT		# CA OR CS
		COM
		EXTEND
REG1TST		BZMF	ROLLON		# IF REGION 1 (DELOGA OGARATE SAME SIGN)

; Region 1 Test: If (CA or CS of SGNRT) complemented is negative,
; then DELOGA and OGARATE have same sign → Region 1.
; Region 1 action: Turn on jets (ROLLON) to reduce rate toward deadband.

# NO JET FIRE YET.  TEST FOR MAX OGARATE

; ============================================================================
; REGION 3 TEST - Rate Limit Check
;
; Not in Region 1, so check if rate exceeds maximum allowed limit.
; Region 3 occurs when roll rate is too high regardless of position error.
; Must fire jets immediately to prevent rate buildup.
;
; Test: |OGARATE| > MAXLIM ?
; Implementation uses CA or CS trick to get signed rate, then adds limit.
; If result negative, rate exceeds limit → Region 3 (fire rate-limiting jets)
;
; For comment-only readers: If spacecraft spinning too fast, fire thrusters
; immediately to slow it down before it gets out of control.
; ============================================================================

		INDEX	I
		0	OGARATE		# CA OR CS...BOTH MUST BE NEG. HERE
		TS	IOGARATE	# I.E. I OGARATE
		AD	MAXLIM		# SCALED AT 2(-4) REV/SEC
		EXTEND
REG3TST		BZMF	RATELIM		# IF REGION 3 (RATES TOO HIGH, FIRE JETS)

; IOGARATE = indexed OGARATE (CA or CS applied)
; MAXLIM = Maximum acceptable roll rate (0.1 deg/sec limit cycle rate)
; If (IOGARATE + MAXLIM) < 0, rate too high, enter rate limiting mode

# COMPUTATION OF I((-SLOPE)OGA + OGARATE) - INTERCEPT..NOTE THAT STR. LINE
# SWITCH SLOPE IS (SLOPE) DEG/SEC/DEG, A NEG QUANTITY

; ============================================================================
; SWITCHING LOGIC COMPUTATION - Phase Plane Control Law
;
; This computes the parabolic switching curve that determines when to fire
; jets based on both attitude error and rate. The switching curve is defined
; by: Rate = SLOPE * Error + Intercept
;
; Mathematical formulation:
; I((-SLOPE) * OGA + OGARATE) - INTERCEP
;
; Where:
; - OGA = Outer gimbal angle error (roll attitude error)
; - OGARATE = Roll rate (deg/sec)
; - SLOPE = Negative control constant relating error to rate
; - INTERCEP = Parabola intercept defining switching boundary
; - I = Index operator (CA or CS depending on error sign)
;
; Physical interpretation: This parabolic curve in the phase plane (rate vs
; error) defines the boundary between firing jets and coasting. If the
; spacecraft state is above the curve, fire jets; below the curve, coast.
;
; For comment-only readers: This calculates whether the spacecraft needs
; thruster firing based on complex math involving both how far off-target
; it is AND how fast it's rotating.
; ============================================================================

		CA	OGARATE
		EXTEND
		MP	BIT14
		TS	TEMREG
; OGARATE * BIT14 = OGARATE * (1/16) for scaling
; BIT14 = 2^(-4) scaling factor
; TEMREG now holds scaled rate value

		CA	OGA
		EXTEND
		MP	-SLOPE
		DDOUBL
		DDOUBL
		DDOUBL			# (OGA ERROR MUST BE LESS THAN +-225 DEG)
; OGA * (-SLOPE) gives position component
; Three DDOUBL operations multiply by 8 (2^3) for proper scaling
; Constraint: OGA must be < ±225 degrees to prevent overflow
; During Apollo 11 SPS burns, attitude errors were typically < 5 degrees

		AD	TEMREG
; Combine: (-SLOPE * OGA) + OGARATE
; Result is the linear combination defining switching curve value

		INDEX	I
		0	A		# I((-SLOPE)OGA+OGARATE) AT 2(-3)REV/SEC
; INDEX I applies CA or CS depending on error sign
; Result: I((-SLOPE)OGA+OGARATE) scaled at 2^(-3) rev/sec

		COM
# Page 991
		AD	INTERCEP	# SCALED AT 2(-3) REV.
		COM
; Compare switching value to intercept
; COM operations implement signed comparison logic
; INTERCEP defines parabola position in phase plane

		EXTEND
REG2TST		BZMF	NOROLL		# IP REGION 2 (COAST SIDE OF STRT LINE)
; Region 2 test: Are we on the coast side of the switching curve?
; If true, spacecraft is approaching target optimally without thrust
; NOROLL path: Let spacecraft coast, no jet firing needed

# CHECK TO SEE IF OGARATE IS ABOVE MINLIM BOUNDARY

; ============================================================================
; REGION 4 TEST - Minimum Rate Boundary Check
;
; On the fire side of switching curve but need to verify rate is significant.
; Region 4: Rate is small enough that firing jets would be wasteful.
;
; Test: |OGARATE| < MINLIM ?
; IOGARATE is always negative here (absolute value taken)
; If (IOGARATE + MINLIM) < 0, rate below minimum threshold
;
; Physical interpretation: Spacecraft is slightly off target but rotating
; very slowly. Rate is so small that jet firing would waste propellant.
; Better to coast and wait for error to grow before applying control.
;
; For comment-only readers: If spinning very slowly, don't bother firing
; thrusters - it's not worth the fuel for such a tiny correction.
; ============================================================================

		CA	IOGARATE	# ALWAYS NEGATIVE
		AD	MINLIM		# SCALED AT 2(-4) REV/SEC
		EXTEND
REG4TST		BZMF	NOROLL		# IF REGION 4 (COAST SIDE OF MINLIM)
; MINLIM boundary separates regions where thrust is worthwhile
; If rate below MINLIM, enter coast mode (Region 4)

# ALL AREAS CHECKED EXCEPT LAST AREA...NO FIRE IN THIS SMALL SEGMENT

; ============================================================================
; REGION 5 TEST - Deadband Boundary Check
;
; Final coast region check before committing to fire jets. Tests if attitude
; error is within the deadband (DB) around the target.
;
; Test: |OGA| < DB ?
; DB = Deadband, typically 5 degrees for Apollo roll control
;
; Physical interpretation: Spacecraft is very close to desired attitude.
; Within this small zone around the target, don't fire jets - the attitude
; is "good enough" for mission requirements. Firing would cause unnecessary
; oscillation and propellant waste.
;
; For comment-only readers: If you're within 5 degrees of perfect alignment,
; that's close enough - don't waste fuel trying to be even more precise.
; ============================================================================

		INDEX	I
		0	OGA
		COM
		AD	DB
		COM
		EXTEND
REG5TST		BZMF	NOROLL		# IF REGION 5 (COAST SIDE OF DB)
; Deadband test: Within acceptable attitude error tolerance?
; If true, enter coast mode (Region 5) - attitude acceptable

# JETS MUST FIRE NOW.OGARATE IS NEG.(OR VICE VERSA).USE DIRECT STR. LINE.
# DELOGA AND DELOGART ARE USED AS DUMMY VARIABLES IN THE SOLUTION OF A
# STRAIGHT LINE APPROXIMATION TO A QUADRATIC SOLUTION OF THE INTERSECTION
# OF THE CONTROL PARABOLA AND THE STRAIGHT-LINE SWITCH LINE.  THE STRAIGHT
# LINE IS THE TANGENT TO THE CONTROL PARABOLA AT THE OPERATING POINT.  (FOR
# OPERATING POINTS IN REGIONS 6 AND 6')

; ============================================================================
; REGION 6 - Direct Straight Line Firing Logic
;
; Jets must fire. Error and rate have opposite signs (converging toward
; target but rate insufficient to reach switching curve optimally).
;
; Control Strategy: Use direct straight-line approximation to compute
; desired jet firing duration. The straight line is tangent to the control
; parabola at the current operating point.
;
; Mathematical approach:
; - DELOGA = current attitude error (OGA)
; - DELOGART = current rate error (OGARATE)
; - These define the tangent point on the control parabola
;
; Physical interpretation: Spacecraft is moving toward the target but too
; slowly. Apply thrust now to increase rate and reach target optimally.
; The tangent line approximation provides computationally efficient solution
; to the optimal control problem.
;
; For comment-only readers: The spacecraft is heading the right direction but
; too slowly. Fire thrusters to speed up the rotation and hit the target
; at the right time. This uses smart math to figure out how long to fire.
;
; During Apollo 11 SPS burns, this logic maintained roll attitude within the
; 5-degree deadband while pitch/yaw were controlled by TVC gimbal.
; ============================================================================

REGION6		CAE	OGA		# USE ACTUAL OPERATING POINT FOR TANGENT
		TS	DELOGA		# ACTUAL STATE
; Store current attitude error as tangent point for straight-line solution
; DELOGA = operating point error used in firing time calculation

		CA	OGARATE
		TS	DELOGART	# ACTUAL STATE,I.E. DEL OGARATE
; Store current rate as tangent point
; DELOGART = operating point rate (delta OGARATE)

		TCF	ONROLL
; Proceed to jet firing time computation using direct tangent line

# JETS ALSO FIRE FROM HERE EXCEPT OGARATE IS POS (VICE VERSA),USE INDIRECT
# STRAIGHT LINE ESTABLISHED BY TANGENT TO A CONTROL PARABOLA AT    ((DELOGA
# + DB SGN(DELOGA) ), -LMCRATE SGN(DELOGA) )	(THIS IS THE DUMMY
# OPERATING POINT FOR OPERATING POINTS IN REGIONS 1 AND 1' )

; ============================================================================
; ROLLON - Indirect Straight Line Firing Logic (Regions 1 and 1')
;
; Jets must fire, but error and rate have same sign (moving away from target
; or moving toward target too fast). This is more complex than Region 6.
;
; Control Strategy: Use indirect straight-line approximation with a "dummy"
; operating point that's not the actual current state. The dummy point is
; chosen at the limit cycle boundary to provide stable control.
;
; Dummy Operating Point Construction:
; - Error: DELOGA + DB·SGN(DELOGA)  [shift by deadband in error direction]
; - Rate: -LMCRATE·SGN(DELOGA)      [limit cycle rate with correct sign]
;
; Why dummy point? The actual state is far from optimal trajectory. Using
; a dummy point on the limit cycle boundary ensures we fire jets with
; duration that will bring us toward stable oscillation around target.
;
; Physical interpretation: Spacecraft is moving wrong direction or overshooting.
; Can't use current position for control calculation - it would give bad
; answer. Instead, use a reference point that represents where we want the
; motion to stabilize (the limit cycle boundary).
;
; For comment-only readers: The spacecraft is moving the wrong way. Instead of
; calculating based on where it is now, calculate based on where we want it
; to end up - this gives better thruster firing time.
;
; LMCRATE = Limit cycle rate (0.1 deg/sec for Apollo roll control)
; DB = Deadband (5 degrees)
; ============================================================================

ROLLON		INDEX	I
		0	DB
		ADS	DELOGA		# DELOGA WAS DIST. FROM SWITCH PARABOLA
; Construct dummy operating point error component
; DELOGA = (distance from switch parabola) + DB·SGN(DELOGA)
; Shifts error outward by deadband amount in direction of error

		CS	LMCRATE		# LIMIT CYCLE RATE AT 2(-4) REV/SEC
		INDEX	I
		0	A
		TS	DELOGART	# EVALUATE STATE FOR INDIRECT LINE.
; Construct dummy operating point rate component
; DELOGART = -LMCRATE·SGN(DELOGA) = limit cycle rate with appropriate sign
; This represents the rate at the limit cycle boundary

# Page 992
# SOLVE STRAIGHT LINES SIMULTANEOUSLY TO OBTAIN DESIRED OGARATE.

; ============================================================================
; ONROLL - Compute Desired Rate from Straight Line Solution
;
; This section solves for the desired OGARATE by finding where the control
; straight line intersects with the switching parabola. This gives the
; target rate that jets should achieve.
;
; Mathematical Background:
; Two equations in phase plane:
; 1. Switching parabola: OGARATE² = -SLOPE · DELOGA  (control boundary)
; 2. Straight line through operating point (tangent to parabola)
;
; Solving simultaneously yields desired rate to command with jet firing.
;
; The computation uses the quadratic formula solution, with numerator and
; denominator calculated separately for numerical stability.
;
; For comment-only readers: The computer now figures out exactly what
; rotation rate is needed to reach the target perfectly. It's solving
; an equation that balances where you are with how fast you need to spin.
; ============================================================================

ONROLL		EXTEND			# DELOGART IN ACC. ON ARRIVAL
		MP	1/CONACC
		DOUBLE
		EXTEND
		MP	-SLOPE
		TS	TEMREG		# 2(-SLOPE)RATE /CONACC
; Begin denominator calculation
; TEMREG = 2·(-SLOPE)·DELOGART/CONACC
; 1/CONACC = reciprocal of control acceleration (jet angular acceleration)
; -SLOPE = slope of switching parabola (negative value)

		EXTEND
		MP	DELOGART
		TS	DELOGART	# 2(-SLOPE)(RATESQ)/CONACC
; Continue denominator: multiply by rate again
; DELOGART = 2·(-SLOPE)·DELOGART²/CONACC
; This is the quadratic term in the denominator

		CS	BIT11
		INDEX	I
		0	A
RATEDEN		ADS	TEMREG		# DENOMINATOR COMPLETED
; Complete denominator calculation
; TEMREG = 2·(-SLOPE)·DELOGART/CONACC + SGN(DELOGA)
; BIT11 provides the ±1 sign term
; Denominator ready for division

		INDEX	I
		0	DELOGA
		COM
		AD	DB
		COM
		EXTEND
		MP	-SLOPE
		ADS	DELOGART
; Begin numerator calculation
; Compute: -SLOPE · (DB - SGN(DELOGA)·DELOGA)
; This represents the error term contribution to the quadratic solution
; DELOGA indexed by I provides correct sign

		CA	LMCRATE
		EXTEND
		MP	BIT11
RATENUM		AD	DELOGART	# NUMERATOR COMPLETED
; Complete numerator calculation
; Numerator = LMCRATE·SGN(DELOGA) + previous error term
; LMCRATE = limit cycle rate reference
; This completes the quadratic formula numerator

		XCH	L		# PLACE NUMERATOR IN L FOR OVERFL. CHECK
		CA	ZERO
		EXTEND
		DV	TEMREG		# OVERFLOW, IF ANYTHING, NOW APPEARS IN A
		EXTEND
		BZF	DVOK		# NO OVERFLOW....(0,L)/TEMREG = 0,L
; Division overflow check
; Divide (0,L) by TEMREG using double-precision division
; If numerator >> denominator, overflow occurs and A becomes non-zero
; Normal case: A = 0 after division, actual result in L
; This technique detects whether |desired rate| exceeds AGC register capacity

MINLIMAP	CCS	A
		CAF	POSMAX		# 	POSITIVE OVERFLOW
		TCF	ROLLSET
		CS	POSMAX		#	NEGATIVE OVERFLOW
		TCF	ROLLSET
; Handle division overflow
; If overflow detected (A non-zero), saturate desired rate at maximum
; POSMAX = maximum positive rate (or negative if overflow negative)
; This prevents computational instability from extreme control commands
; Physical interpretation: Desired rate is so large that we max out thrust

DVOK		LXCH	A		# PUT NUMERATOR BACK INTO A, 0 INTO L
		EXTEND
		DV	TEMREG		# RESULT OF DIVISION IS DESIRED OGARATE
		TCF	ROLLSET		#	( SCALED AT B-4 REV/SEC )
; Normal division path (no overflow)
; Compute numerator/denominator = desired OGARATE
; Result scaled at 2^-4 revolutions per second
; This is the target rate that jets should achieve to reach switching curve
; optimally and coast into the desired attitude

RATELIM		CS	MAXLIM
		INDEX	I
# Page 993
		0	A		# IF I = CA, DESIRED RATE IS	-MAXLIM
; Rate limiting entry point
; Applied earlier in control flow when desired rate exceeds MAXLIM
; INDEX I applies correct sign: if I=CA, negates; if I=CS, doesn't
; MAXLIM = maximum allowable rate (0.1 deg/sec limit cycle rate)
; Ensures commanded rate stays within safe operating bounds

# COMPUTE JET FIRE TIME, BASED ON DESIRED RATE MINUS PRESENT RATE

; ============================================================================
; ROLLSET - Compute Jet Firing Time from Desired Rate
;
; Now that desired OGARATE is known, calculate how long to fire jets
; to achieve that rate. Firing time proportional to rate change needed.
;
; Formula: T_fire = (OGARATE_desired - OGARATE_current) / angular_acceleration
;
; For comment-only readers: The computer now figures out exactly how long
; to fire the roll thrusters. Too short and you won't spin fast enough,
; too long and you'll spin too fast. It's solving: time = change_needed / thrust_power
; ============================================================================

ROLLSET		TS	TEMREG		# STORE DESIRED OGARATE (SCALED B-4)
		EXTEND
		SU	OGARATE		# RATE DIFF. SCALED AT 2(-4) REV/SEC
		TS	TEMREG		#	OVERFLOW PROTECT
		TCF	+3		#	    "       "
		INDEX	A		#           "       "
		CS	LIMITS		#	    "       "
; Compute rate difference: desired - current
; Result is change in OGARATE needed
; Overflow protection: if |rate difference| > max, saturate at LIMITS
; OGARATE = current roll rate from gyros

		EXTEND
		MP	T6SCALE		# T6SCALE = 8/10.24
		EXTEND
		MP	1/CONACC	# SCALED AT B+9 SECSQ/REV (MAX < .60)
		DDOUBL
		DDOUBL
; Convert rate difference to firing time
; T6SCALE = 8/10.24 converts to T6 counter units (10.24 ms per count)
; 1/CONACC = reciprocal of control acceleration (jet angular accel)
; DDOUBL twice = multiply by 4, adjusts scaling
; Result: firing time needed to achieve desired rate change

		TS	TEMREG		#	OVERFLOW PROTECT
		TCF	+3		#	    "	    "
		INDEX	A		#   	    "	    "
		CS	LIMITS		# 	    " 	    "
		TS	TEMREG		# JET FIRE TIME AT 625 MICROSEC/BIT
		EXTEND			# POS MEANS POSITIVE ROLL TORQUE.
		BZF	NOROLL
; Store firing time with overflow protection
; TEMREG = jet firing time, scaled at 625 microseconds per bit (T6 clock rate)
; Sign indicates torque direction: positive = positive roll, negative = negative roll
; If result is zero, no firing needed → NOROLL

# JET FIRE TIME IS NZ, TEST FOR JETS NOW ON.

; ============================================================================
; Jet Firing State Decision Logic
;
; Determine whether to continue existing firing, start new firing, or
; terminate firing based on current jet state and desired torque direction.
;
; Three cases:
; 1. MOREROLL: Jets already firing in correct direction → extend firing
; 2. NEWROLL: Jets off or need direction reversal → start new firing
; 3. NOROLL: Requested direction opposite to current firing → terminate
;
; For comment-only readers: The computer checks if thrusters are already
; firing. If they're firing the right direction, keep them going. If not,
; it needs to shut them off and fire the opposite direction.
; ============================================================================

		CAE	TEMREG		# DESIRED CHANGE IN OGARATE
		EXTEND
		MP	ROLLFIRE	# (SGN OF TORQUE..ZERO IF JETS NOW OFF)
		CCS	A
		TCF	MOREROLL	# CONTINUE FIRING WITH PRESENT POLARITY
		TCF	NEWROLL		# START NEW FIRING NOW, PLUS
		TCF	NOROLL		# TERMINATE OLD FIRING, NEW SIGN REQUESTED
		TCF	NEWROLL		# START NEW FIRING NOW, MINUS
; Test jet state compatibility
; Multiply desired fire time by ROLLFIRE (current jet state)
; ROLLFIRE = +POSMAX if jets firing positive, -NEGMAX if negative, 0 if off
; Result > 0: same polarity, continue → MOREROLL
; Result = 0 (plus): jets off, new positive firing → NEWROLL
; Result < 0: opposite polarity needed, terminate old → NOROLL
; Result = 0 (minus): jets off, new negative firing → NEWROLL

# CONTINUE PRESENT FIRING

; ============================================================================
; MOREROLL - Extend Existing Jet Firing
;
; Jets already firing in correct direction. Don't restart, just adjust
; the firing duration if needed. This avoids propellant waste from
; premature jet shutdown and reignition.
;
; Physical context: During Apollo missions, frequent jet cycling wasted
; propellant and caused unnecessary disturbances. Extending existing
; firings improved efficiency.
; ============================================================================

MOREROLL	CAF	ZERO
		TS	I		# USE TEMP. AS MOREROLL SWITCH
		TCF	MAXTFIRE
; Set flag for "more roll" mode
; I = 0 indicates continuing existing firing (checked later at FIRELOOK)
; Skip minimum fire time check (already firing)
; Jump to maximum fire time check

# START NEW FIRING BUT CHECK IF GREATER THAN MIN FIRE TIME.

; ============================================================================
; NEWROLL - Minimum Firing Time Check
;
; Before initiating new jet firing, verify requested duration exceeds
; minimum fire time (TMINFIRE = 15 ms). Short firings are inefficient:
; valve response time dominates, producing little useful impulse.
;
; If requested time < 15 ms, skip firing and coast until next cycle.
; This prevents wasteful micro-firings that consume propellant without
; meaningful attitude correction.
;
; For comment-only readers: The computer won't fire thrusters for less
; than 15 milliseconds. Such brief pulses waste fuel without doing much.
; Better to wait for a bigger correction need.
; ============================================================================

NEWROLL		CCS	TEMREG		# CALL THIS T6FIRE
		AD	ONE
		TCF	+2
		AD	ONE
		COM			# -MAG(T6FIRE)
		AD	TMINFIRE	# TMINFIRE-MAG(T6FIRE)
; Compute magnitude of requested firing time
; CCS extracts sign, AD ONE twice gets |TEMREG|
; COM negates to get -|T6FIRE|
; Add TMINFIRE: result = TMINFIRE - |T6FIRE|
# Page 994
		COM
		EXTEND
MINTST		BZMF	NOROLL		# IF NOT GREATER THAN TMINFIRE (NEW FIRE)
; COM again: result = |T6FIRE| - TMINFIRE
; BZMF: if result ≤ 0, then |T6FIRE| ≤ TMINFIRE → too short, don't fire
; TMINFIRE = 15 milliseconds minimum firing duration
; Below this threshold, valve dynamics dominate and impulse is inefficient

# PROCEED WITH NEW FIRING BUT NOT LONGER THAN TMAXFIRE.

; ============================================================================
; MAXTFIRE - Maximum Firing Time Limiter
;
; Cap jet firing duration at TMAXFIRE = 2.5 seconds. Longer continuous
; firings risk:
; - Thermal damage to thruster valves
; - Excessive propellant consumption
; - Buildup of unmonitored disturbances
;
; Algorithm: Multiply TEMREG by (1/TMAXFIRE), producing scaling ratio.
; If ratio > 1, firing exceeds max → clamp to TMAXFIRE
; If ratio ≤ 1, firing within limit → proceed with TEMREG
;
; For comment-only readers: The computer won't fire thrusters longer than
; 2.5 seconds continuously. This prevents overheating and conserves fuel.
; ============================================================================

MAXTFIRE	CA	TEMREG
		EXTEND
		MP	1/TMXFIR	# I.E. 1/TMAXFIRE
		EXTEND
MAXTST		BZF	NOMXFIRE	# IF LESS THAN TMAXFIRE
; Scaling test: TEMREG × (1/TMAXFIRE)
; If result = 0, then TEMREG < TMAXFIRE (within limits)
; If result ≠ 0, then TEMREG ≥ TMAXFIRE (exceeds limit)
; 1/TMXFIR scaled appropriately for T6 timer units

		CCS	A
		CAF	TMAXFIRE	# USE MAXIMUM
		TCF	+2
		CS	TMAXFIRE	# USE MAXIMUM
		TS	TEMREG
; Exceeded maximum: clamp to TMAXFIRE
; CCS branches on sign of overflow ratio
; Positive overflow → use +TMAXFIRE
; Negative overflow → use -TMAXFIRE (preserves torque direction)
; TMAXFIRE = 2.5 seconds = 2500 ms in T6 units

# SET UP SIGN OF REQUIRED TORQUE.

; ============================================================================
; NOMXFIRE - Torque Direction Setup
;
; Configure ROLLFIRE to indicate torque polarity for jet pair selection.
; ROLLFIRE serves dual purpose:
; 1. Sign indicates torque direction (+POSMAX or -NEGMAX)
; 2. Magnitude counts down as T6 interrupt services firing
;
; TEMREG magnitude adjusted to absolute value for consistent time tracking.
;
; Physical interpretation: Positive torque rolls spacecraft clockwise (viewed
; from behind). Negative torque rolls counterclockwise. Sign determines which
; RCS jet pair fires: +X or -X roll jets.
; ============================================================================

NOMXFIRE	CCS	TEMREG		# FOR TORQUE SIGN
		CA	POSMAX		# POSITIVE TORQUE REQUIRED
		TCF	+2
		CA	NEGMAX		# NEGATIVE TORQUE REQUIRED
		TS	ROLLFIRE	# SET ROLLFIRE FOR + OR - TORQUE
; Extract sign of TEMREG (desired firing time with direction)
; If TEMREG > 0: load POSMAX → positive torque
; If TEMREG < 0: load NEGMAX → negative torque
; Store in ROLLFIRE as polarity indicator
; POSMAX/NEGMAX = maximum positive/negative word values

		COM			# COMPLEMENT... POS. FOR NEG. TORQUE
		EXTEND
		BZMF	+3		# POSITIVE TORQUE REQUIRED
		CS	TEMREG
		TS	TEMREG
; Convert TEMREG to absolute magnitude for time countdown
; COM (ROLLFIRE) is positive if negative torque, negative if positive torque
; BZMF: if result ≤ 0, skip (already positive magnitude)
; Otherwise CS TEMREG converts negative to positive magnitude
; Result: TEMREG = |firing time|, ROLLFIRE = ±MAX (polarity)

; ============================================================================
; FIRELOOK / FIREPLUG - Firing Mode Branch
;
; Two paths converge here:
; 1. NEWROLL (I ≠ 0): Starting fresh firing → proceed to JETROLL
; 2. MOREROLL (I = 0): Extending existing firing → check if extension valid
;
; FIREPLUG prevents extending jet firings. Once a firing starts with
; specific duration, subsequent control cycles cannot lengthen it. They can
; only shorten or terminate. This design rule prevents unlimited firing
; extensions that could exceed thermal/mechanical limits.
;
; For comment-only readers: Once thrusters start firing for a set time,
; the computer can't extend that time - only cut it short if needed. This
; prevents accidentally firing too long and overheating valves.
; ============================================================================

FIRELOOK	CA	I		# IS IT MOREROLL
		EXTEND
		BZF	FIREPLUG	# YES
		TCF	JETROLL		# MAG(T6FIRE) NOW IN TEMREG
; Check MOREROLL flag (stored in I)
; I = 0: continuing existing firing → FIREPLUG (check extension rules)
; I ≠ 0: new firing → JETROLL (initiate firing)

FIREPLUG	CAE	TIME6		# CHECK FOR EXTENDED FIRING
		EXTEND
		SU	TEMREG
		EXTEND
EXTENTST	BZMF	TASKOVER	# IF EXTENSION WANTED, DONT, EXIT ROLL DAP
		TCF	JETROLL
; Extension prohibition test
; TIME6 = elapsed time counter since firing began
; TEMREG = newly computed desired fire time
; If TIME6 < TEMREG, new duration longer than already elapsed → extension
; BZMF: if (TIME6 - TEMREG) ≤ 0, attempting extension → exit without change
; Otherwise: new duration shorter or equal → allow (shortening okay)
; This rule from operational requirement: "JET FIRINGS MAY NOT BE EXTENDED"

; ============================================================================
; NOROLL - Coast Mode
;
; No jet firing required this cycle. Set flags to coast state:
; - ROLLFIRE = -0 (negative zero signals coast, no active firing)
; - TEMREG = -0 (no pending fire time)
; - TIME6 inherits negative zero through subsequent processing
;
; Negative zero convention distinguishes "no action" from "zero time action."
; Next control cycle can initiate fresh firing if needed.
; ============================================================================

NOROLL		CS	ZERO		# COAST....(NEG ZERO FOR TIME6)
		TS	ROLLFIRE	# NOTE, JETS CAN FIRE NEXT PASS
		TS	TEMREG
; Clear firing state variables
; CS ZERO produces -0 (negative zero, distinct from +0 in AGC)
; ROLLFIRE = -0 signals coast (no jets active)
; TEMREG = -0 clears pending fire time
; This allows next cycle to start fresh firing if attitude error grows

; ============================================================================
; JETROLL - Jet Firing Initialization
;
; Configure T6 interrupt handler to service roll jet firing:
; 1. Install NOROL1T6 as T6RUPT handler address
; 2. Load TEMREG (firing duration) into TIME6 counter
; 3. Check if MOREROLL: if yes, keep current jets; if no, select new jets
;
; T6 interrupt occurs every 10.24 milliseconds, decrementing TIME6.
; When TIME6 reaches zero, NOROL1T6 handler terminates jet firing.
;
; For comment-only readers: This sets up the timing mechanism. The computer's
; T6 clock will count down the firing duration and automatically shut off
; the thrusters when time expires.
; ============================================================================

JETROLL		EXTEND
		DCA	NOROL1T6
# Page 995
		DXCH	T6LOC
		CA	TEMREG		# ENTER JET FIRING TIME
		TS	TIME6
; Install T6 interrupt handler for roll jet control
; DCA NOROL1T6: load 2-word handler address (double-precision)
; DXCH T6LOC: store in T6 interrupt vector location
; T6LOC points to code executed every T6RUPT (10.24 ms period)
; TIME6 = countdown timer decremented by T6RUPT, terminating at zero

		CA	I		# I=0 IF MOREROLL,KEEP SAME JETS ON
		EXTEND
SAMEJETS	BZF	TASKOVER	# IF JETS ON KEEP SAME JETS.  EXIT ROLL DAP
; Jet selection branch
; I = 0 (MOREROLL): keep current jet pair firing → exit
; I ≠ 0 (NEWROLL): select new jet pair based on torque polarity
; MOREROLL avoids switching jets mid-firing (eliminates transient disturbances)
; New firing requires jet pair alternation per "JET PAIRS FIRE ALTERNATELY" rule

		CCS	ROLLFIRE
		TCF	+TORQUE
		TCF	T6ENABL
		TCF	-TORQUE
		TCF	T6ENABL
; Torque polarity branch
; ROLLFIRE sign determines required torque direction:
; ROLLFIRE > 0: positive torque → +TORQUE (fire +X roll jets)
; ROLLFIRE = +0: coast (no firing) → T6ENABL (enable interrupt, exit)
; ROLLFIRE < 0: negative torque → -TORQUE (fire -X roll jets)
; ROLLFIRE = -0: coast (no firing) → T6ENABL
; CCS four-way branch extracts sign and magnitude in one instruction

# PROCEED WITH + TORQUE

; ============================================================================
; +TORQUE - Positive Roll Torque Jet Selection
;
; Select RCS jet pair for positive (clockwise) roll torque. Two jet pairs
; available for +X roll axis torque:
; - Jets 9 & 11 (first pair, +ROLL1)
; - Jets 13 & 15 (second pair, +ROLL2)
;
; Alternation strategy: Track last-used pair in ROLLWORD bit 1.
; - If bit 1 = 0: last used jets 13-15 → use 9-11 this time
; - If bit 1 = 1: last used jets 9-11 → use 13-15 this time
;
; Alternation balances propellant consumption across all four +X jets,
; distributes thermal loads, and provides redundancy if one pair fails.
;
; Physical spacecraft context: +X jets produce clockwise roll (looking aft).
; Jet numbering per SM RCS quad configuration.
;
; For comment-only readers: The computer alternates between two pairs of
; roll thrusters to spread wear and fuel usage evenly. It remembers which
; pair fired last and picks the other pair this time.
; ============================================================================

+TORQUE		CA	ROLLWORD	# WHAT WAS THE LAST +TORQUE COMBINATION
		MASK	BIT1		# WAS IT NO.9-11
		EXTEND
		BZF	NO.9-11		# NOT 9-11, SO USE IT THIS TIME
; Check ROLLWORD bit 1 to determine last-used +torque jet pair
; MASK BIT1 isolates bit 1 (octal 00002)
; BZF: if bit 1 = 0, then last used was NO.13-15 → fire NO.9-11 now
; Otherwise: bit 1 = 1, last used NO.9-11 → fire NO.13-15 now

NO.13-15	CS	BIT1
		MASK	ROLLWORD
		TS	ROLLWORD	# CHANGE BIT 1 TO ZERO
		CAF	+ROLL2
		EXTEND
		WRITE	CHAN6
		TCF	T6ENABL
; Fire jets 13-15 (+ROLL2)
; Clear bit 1 of ROLLWORD (CS BIT1 produces complement, MASK clears bit)
; Store updated ROLLWORD with bit 1 = 0 (records this pair used)
; Load +ROLL2 jet command pattern
; WRITE CHAN6 outputs to RCS jet driver channel, activating jets 13 & 15
; Proceed to T6ENABL to enable countdown interrupt

NO.9-11		CAF	BIT1		# 1ST + JETS TO FIRE (MRCLEAN OS ROLLWORD)
		ADS	ROLLWORD	# CHANGE BIT 1 TO ONE
		CAF	+ROLL1
		EXTEND
		WRITE	CHAN6
		TCF	T6ENABL
; Fire jets 9-11 (+ROLL1)
; Set bit 1 of ROLLWORD (ADS adds BIT1, setting bit 1 = 1)
; Store updated ROLLWORD (records this pair used)
; Load +ROLL1 jet command pattern
; WRITE CHAN6 activates jets 9 & 11
; MRCLEAN comment: initialization clears ROLLWORD, so first firing uses 9-11
; Proceed to T6ENABL

; ============================================================================
; -TORQUE - Negative Roll Torque Jet Selection
;
; Select RCS jet pair for negative (counter-clockwise) roll torque. Two jet
; pairs available for -X roll axis torque:
; - Jets 10 & 12 (first pair, -ROLL1)
; - Jets 14 & 16 (second pair, -ROLL2)
;
; Alternation strategy: Track last-used pair in ROLLWORD bit 2.
; - If bit 2 = 0: last used jets 16-14 → use 12-10 this time
; - If bit 2 = 1: last used jets 12-10 → use 16-14 this time
;
; Same alternation logic as +TORQUE, but using bit 2 instead of bit 1.
; Independent tracking allows +torque and -torque pairs to alternate
; independently.
;
; Physical spacecraft context: -X jets produce counter-clockwise roll
; (looking aft). These four jets work in opposition to the +X jets,
; providing full bi-directional roll control authority.
;
; For comment-only readers: Just as with clockwise roll, the computer
; alternates between two pairs of counter-clockwise roll thrusters. The
; alternation is tracked independently from the clockwise jets.
; ============================================================================

-TORQUE		CA	ROLLWORD	# WHAT WAS LAST -TORQUE COMBINATION
		MASK	BIT2		# WAS IT NO.12-10
		EXTEND
		BZF	NO.12-10	# NOT 12-10, SO USE IT THIS TIME
; Check ROLLWORD bit 2 to determine last-used -torque jet pair
; MASK BIT2 isolates bit 2 (octal 00004)
; BZF: if bit 2 = 0, then last used was NO.16-14 → fire NO.12-10 now
; Otherwise: bit 2 = 1, last used NO.12-10 → fire NO.16-14 now

NO.16-14	CS	BIT2
		MASK	ROLLWORD
		TS	ROLLWORD	# CHANGE BIT 2 TO ZERO
		CAF	-ROLL2
		EXTEND
		WRITE	CHAN6
		TCF	T6ENABL
; Fire jets 16-14 (-ROLL2)
; Clear bit 2 of ROLLWORD
; Store updated ROLLWORD with bit 2 = 0 (records this pair used)
; Load -ROLL2 jet command pattern
; WRITE CHAN6 activates jets 14 & 16 (counter-clockwise torque)
; Proceed to T6ENABL

NO.12-10	CAF	BIT2		# 1ST -JETS TO FIRE (MRCLEAN OS ROLLWORD)
# Page 996
		ADS	ROLLWORD	# CHANGE BIT 2 TO ONE
		CAF	-ROLL1
		EXTEND
		WRITE	CHAN6
; Fire jets 12-10 (-ROLL1)
; Set bit 2 of ROLLWORD (ADS adds BIT2, setting bit 2 = 1)
; Store updated ROLLWORD (records this pair used)
; Load -ROLL1 jet command pattern
; WRITE CHAN6 activates jets 10 & 12 (counter-clockwise torque)
; MRCLEAN comment: initialization clears ROLLWORD, so first -firing uses 12-10
; Fall through to T6ENABL

; ============================================================================
; T6ENABL - Enable T6 Countdown Interrupt
;
; Enable CHAN13 bit 15 to activate the T6 countdown timer. The T6 interrupt
; will fire after ROLLTIME (1.28 sec clock register) counts down to zero,
; triggering the NOROLL1 task to shut off the jets.
;
; This mechanism provides hardware-timed jet firing control. Once jets are
; commanded on (via WRITE CHAN6 above), they remain on until NOROLL1 executes
; at T6 timeout. The firing duration is controlled by the ROLLTIME value
; loaded earlier (limited 15ms-2.5sec by MINFIRE, MAXTFIRE logic).
;
; For comment-only readers: After commanding the jets to fire, the computer
; sets up a hardware timer that will automatically shut them off after the
; calculated firing time elapses.
; ============================================================================

T6ENABL		CAF	BIT15
		EXTEND
		WOR	CHAN13
; Enable T6 countdown timer
; CAF BIT15 loads bit 15 mask (enable bit for T6 timer)
; WOR CHAN13 performs inclusive-OR write to CHAN13 (sets bit without
; disturbing other channel bits)
; T6 interrupt will fire when ROLLTIME counts down to zero, calling NOROLL1

RDAPEND		TCF	TASKOVER	# EXIT ROLL DAP
; Roll DAP computation complete for this cycle
; TASKOVER returns control to executive scheduler
; Next ROLLDAP invocation in 0.5 seconds via TVCEXEC waitlist call

# Page 997
# THIS T6 TASK SHUTS OFF ALL ROLL JETS

; ============================================================================
; NOROLL1 - T6 Interrupt Task to Shut Off Roll Jets
;
; Called automatically by T6 timer interrupt when ROLLTIME counts down to zero.
; Terminates roll jet firing by writing zero to CHAN6, shutting off all roll
; RCS jets. Also clears ROLLFIRE flag to indicate jets are off.
;
; This task executes at T6 interrupt priority level, preempting lower-priority
; tasks. The LXCH BANKRUPT instruction is the standard interrupt entry sequence
; that saves the interrupted task's return address.
;
; Jet shutoff timing accuracy critical: firing durations computed by phase
; plane logic must be executed precisely to achieve ±5° deadband and <0.1°/sec
; limit cycle rate targets.
;
; For comment-only readers: When the hardware timer expires, this interrupt
; routine immediately shuts off the roll thrusters. The jets have now fired
; for exactly the duration calculated by the control law.
; ============================================================================

NOROLL1		LXCH	BANKRUPT	# SHUT OFF ALL (ROLL) JETS, (A T6 TASK
		CAF	ZERO		#	CALLED BY ..JETROLL..)
		TS	ROLLFIRE	# ZERO INDICATES JETS NOW OFF
; T6 interrupt entry point
; LXCH BANKRUPT saves interrupted task's return bank
; CAF ZERO loads zero into accumulator
; TS ROLLFIRE clears firing flag (indicates no jets currently active)

		EXTEND
KILLJETS	WRITE	CHAN6
		TCF	NOQRSM
; WRITE CHAN6 with zero command shuts off all roll jets
; Jet driver hardware interprets zero as "all jets off"
; TCF NOQRSM exits interrupt, resuming interrupted task
; NOQRSM is standard T6 interrupt exit point

# Page 998
# CONSTANTS FOR ROLL AUTOPILOT....

; ============================================================================
; Roll Autopilot Constants
;
; These constants define the roll autopilot control law parameters. All angle
; values in revolutions (1 rev = 360°), rate values in rev/sec. Scaling
; factors (B+n, B-n) indicate binary point positions for fixed-point arithmetic.
;
; Control law design goals:
; - Maintain roll attitude within ±5° of commanded angle
; - Limit cycle rate ≤ 0.1°/sec for fuel efficiency
; - Minimum rate limit 1°/sec for slew maneuvers
; - Phase plane switching curve optimized for fuel/accuracy tradeoff
; ============================================================================

		EBANK=	BZERO
NOROL1T6	2CADR	NOROLL1
; 2CADR (two-complement address) for NOROLL1 interrupt routine
; Provides both bank number and address for T6 interrupt vector
; EBANK=BZERO specifies erasable memory bank context

DB		DEC	.01388889	# DEAD BAND (5 DEG), SC.AT B+0 REV
; Deadband half-width: ±5° = ±0.01388889 rev
; Scaled at B+0 (unity scale, no binary shift)
; Roll error within ±DB does not trigger jet firing (coast region)
; 5° deadband chosen to balance attitude accuracy vs RCS fuel consumption

-SLOPE		DEC	0.2		# -SWITCHLINE SLOPE(0.2 PER SEC) SC.AT B+0
					#	PER SEC
; Phase plane switching curve slope: -0.2 rad/sec per rad
; Negative slope defines parabolic boundary between coast and fire regions
; Steeper slope = faster response but more fuel usage
; 0.2 selected from control system stability analysis (GSOP section 3)

LMCRATE		DEC	.00027778 B+4	# LIMIT CYCLE RATE (0.1 DEG/SEC) SC.AT
					#	B-4 REV/SEC
; Target limit cycle rate: 0.1°/sec = 0.00027778 rev/sec
; Scaled at B+4 (multiply by 2^4 = 16 for storage)
; Limit cycle = small oscillations around target attitude when in deadband
; 0.1°/sec chosen to minimize propellant waste while maintaining control

INTERCEP	DEC	.0025 B+3	# DB(-SLOPE) - LMCRATE, SC.AT B-3 REV/SC
; Switching curve intercept parameter: DB × |SLOPE| - LMCRATE
; Computed value: 0.01388889 × 0.2 - 0.00027778 = 0.0025
; Scaled at B+3 (multiply by 2^3 = 8)
; Used in phase plane boundary calculation

MINLIM		DEC	.00277778 B+4	# RATELIM,MIN (1DEG/SEC), SC.AT B-4 REV/SC
; Minimum rate limit: 1°/sec = 0.00277778 rev/sec
; Scaled at B+4
; Maximum roll rate commanded during slew maneuvers
; Limits angular acceleration to prevent control saturation

1/MINLIM	DEC	360 B-18	# RECIPROCAL THEREOF, SHIFTED 14 RIGHT
; Reciprocal of MINLIM: 1/0.00277778 = 360 rev·sec
; Scaled at B-18 (divide by 2^18 = 262144)
; Precomputed reciprocal saves division operations in time-critical code
; Shift by 14 right optimizes for AGC's double-precision arithmetic

; ============================================================================
; RATE LIMIT AND TIMING CONSTANTS
; ============================================================================
; These constants define rate limits, firing time constraints, and timer
; scaling for the roll autopilot hardware interface.

MAXLIM		DEC	.01388889 B+4	# RATELIM,MAX (5DEG/SEC), SC.AT B-4 REV/SC
; Maximum angular rate limit: 5°/sec
; Scaled at B-4 revolutions/second
; 0.01388889 rev/sec = 5°/sec (since 360° = 1 revolution)
; Used in phase plane logic to limit commanded rate
; Prevents excessive slewing rates during roll corrections

TMINFIRE	DEC	1.5 B+4		# 15 MS (14MIN), SC.AT 16 BITS/CS
; Minimum jet firing time: 15 milliseconds (actual: 14 ms minimum)
; Scaled at 16 bits/centisecond
; Below 15ms, jet valve response becomes unreliable
; Enforces minimum impulse bit to ensure predictable thrust delivery

TMAXFIRE	DEC	250 B+4		# 2.5 SEC, SC.AT 16 BITS/CS
; Maximum jet firing time: 2.5 seconds
; Scaled at 16 bits/centisecond
; Limits continuous jet operation to conserve RCS propellant
; Longer corrections broken into multiple firing cycles

1/TMXFIR	=	BIT3		# RECIPROCAL THEREOF, SHIFTED 14 RIGHT,
					#	ROUNDS TO OCT00004, SO ALLOWS 2.56
					#	SEC FIRINGS BEFORE APPLYING LIMIT
; Reciprocal of maximum fire time (1/2.5 = 0.4)
; BIT3 = octal 00004
; Shifted 14 bits right for computational efficiency
; Permits 2.56 sec firings before limit enforcement (tolerance margin)

T6SCALE		=	PRIO31		# (B+3) (16 BITS/CS) (100CS/SEC)
; Timer 6 hardware scaling constant
; PRIO31 provides proper bit scaling for T6 countdown register
; Converts centiseconds to T6 timer units (100 cs/sec)
; Used in T6ENABL to load computed fire time into hardware

; ============================================================================
; JET COMMAND BIT PATTERNS (CHANNEL 6 OUTPUT)
; ============================================================================
; These constants define bit patterns written to output channel 6 to command
; specific RCS jet pairs. Roll control uses four jet pairs alternately to
; balance propellant usage and minimize coupling effects.

+ROLL1		= 	FIVE		# ONBITS FOR JETS 9 AND 11
; Positive roll torque, jet pair 1
; FIVE = octal 00005 (binary: bits 0 and 2)
; Commands jets 9 (bit 0) and 11 (bit 2) ON
; First +torque pair used (ROLLWORD bit 1 = 0 initially)

+ROLL2		=	OCT120		# ONBITS FOR JETS 13 AND 15
; Positive roll torque, jet pair 2
; OCT120 = octal 00120 (binary: bits 4 and 6)
; Commands jets 13 (bit 4) and 15 (bit 6) ON
; Second +torque pair, alternates with +ROLL1

-ROLL1		=	TEN		# ONBITS FOR JETS 12 NAD 10
; Negative roll torque, jet pair 1
; TEN = octal 00012 (binary: bits 1 and 3)
; Commands jets 12 (bit 3) and 10 (bit 1) ON
; First -torque pair used (ROLLWORD bit 2 = 0 initially)
; (Note: Original comment typo "NAD" should be "AND")

-ROLL2		OCT	240		# ONBITS FOR JETS 16 AND 14
; Negative roll torque, jet pair 2
; OCT 240 = octal 00240 (binary: bits 5 and 7)
; Commands jets 16 (bit 7) and 14 (bit 5) ON
; Second -torque pair, alternates with -ROLL1
; Balances usage across all four roll jet pairs
