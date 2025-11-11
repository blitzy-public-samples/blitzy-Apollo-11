# Copyright:	Public domain.
# Filename:	Q_R-AXIS_RCS_AUTOPILOT.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1442-1459
# Mod history:	2009-05-27 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-07 RSB	Corrected "DEC 96.0" to "DEC 96", since
#				the former is not compatible with yaYUL.
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
; FILE: Q_R-AXIS_RCS_AUTOPILOT.agc
; MODULE: Digital Autopilot System (DAPS)
; MISSION PHASE: descent/landing/ascent/rendezvous
;
; TL;DR: Implements the Q-axis (yaw) and R-axis (roll) Reaction Control System
;        (RCS) digital autopilot for the Lunar Module. Controls spacecraft yaw
;        and roll attitudes using RCS thrusters during all mission phases from
;        LM separation through ascent rendezvous. Coordinates with P-axis (pitch)
;        autopilot to provide full three-axis attitude control. Executes every
;        100 milliseconds, computing attitude errors, evaluating control laws,
;        and commanding thruster firings to maintain desired spacecraft orientation.
;
; COMMENT-ONLY READERS: This is the autopilot that kept the Lunar Module's
;        yaw and roll attitudes precisely controlled during Armstrong and Aldrin's
;        descent to the lunar surface and their ascent back to rendezvous with
;        Columbia. Read comments to understand how the computer maintained
;        spacecraft orientation automatically.
;
; CODE-ALONG READERS: Study the coupled Q-R control implementation, thruster
;        selection logic, U-V axis transformation to eliminate cross-coupling,
;        PID control loops, and coordination with translation jet commands.
; ============================================================================

# Page 1442
		BANK	17
		SETLOC	DAPS2
		BANK

		EBANK=	CDUXD

		COUNT*	$$/DAPQR

; ============================================================================
; CALLQERR - Q,R AXIS ERROR CALCULATION
;
; The Q,R-axis autopilot begins by determining whether attitude error
; calculations are needed. If the astronauts have switched to manual rate
; command mode using the Attitude Controller Assembly (ACA), the autopilot
; bypasses error computation and allows direct crew control of rotation rates.
; Otherwise, it computes the difference between commanded and actual yaw/roll
; attitudes to drive the automatic control system.
;
; During Apollo 11's descent, this routine executed in automatic mode,
; continuously calculating attitude errors to maintain the LM's proper
; orientation as Armstrong and Aldrin descended toward the Sea of Tranquility.
; ============================================================================

CALLQERR	CA	BIT13		# CALCULATE Q,R ERRORS UNLESS THESE AXES
		EXTEND			# ARE IN MANUAL RATE COMMAND.
		RAND	CHAN31
		CCS	A
		TCF	+5		# IN AUTO COMPUTE Q,R ERRORS
		CS	DAPBOOLS	# IN MANUAL RATE COMMAND?
		MASK	OURRCBIT
		EXTEND
		BZF	Q,RORGTS	# IF SO BYPASS CALCULATION OF ERRORS.
		TC	QERRCALC

Q,RORGTS	CCS	COTROLER	# CHOOSE CONTROL SYSTEM FOR THIS DAP PASS:
		TCF	GOTOGTS		#	GTS (ALTERNATES WITH RCS WHEN DOCKED)
		TCF	TRYGTS		#	GTS IF ALLOWED, OTHERWISE RCS
RCS		CAF	ZERO		#	RCS (TRYGTS MAY BRANCH TO HERE)
		TS	COTROLER

		DXCH	EDOTQ
		TC	ROT-TOUV
		DXCH	OMEGAU

; ============================================================================
; TRANSITION: From Error Calculation to Translation Control
;
; With attitude errors computed (or bypassed for manual mode), the autopilot
; now coordinates rotation commands with translation (X-axis thrust) commands.
; The RCS thrusters serve dual purposes: they provide both attitude control
; torque and linear translation thrust. This section determines the translation
; policy to ensure rotation and translation commands don't conflict.
; ============================================================================

# X - TRANSLATION
#
# INPUT:	BITS 7,8 OF CH31 (TRANSLATION CONTROLLER)
#		ULLAGER
#		APSFLAG, DRIFTBIT
#		ACC40R2X, ACRBTRAN
#
# OUTPUT:	NEXTU, NEXTV	CODES OF TRANSLATION FOR AFTER ROTATION
#		SENSETYP	TELL ROTATION DIRECTION AND DESIRE
#
# X-TRANS POLICIES ARE EITHER 4 JETS OR A DIAGONAL PAIR.  IN 2-JET TRANSLATION THE SYSTEM IS SPECIFIED.  A FAILURE
# WILL OVERRIDE THIS SPECIFICATION.  AN ALARM RESULTS WHEN NO POLICY IS AVAILABLE BECAUSE OF FAILURES.

; ============================================================================
; SENSEGET - TRANSLATION CONTROLLER INPUT PROCESSING
;
; Reads the astronaut's Translation Hand Controller (THC) position from channel
; 31 to determine if forward (+X) or aft (-X) translation thrust is commanded.
; The THC is separate from the ACA (Attitude Controller Assembly) used for
; rotation commands. Translation thrust requires RCS jets that also produce
; rotation torque, so the autopilot must coordinate both functions carefully.
;
; During Apollo 11's descent, translation commands were minimal once powered
; descent began, as the Descent Propulsion System (DPS) provided the primary
; thrust. However, during approach and landing phases, small translation
; corrections using RCS jets helped Armstrong fine-tune the LM's trajectory.
; ============================================================================

SENSEGET	CA	BIT7		# INPUT BITS OVERRIDE THE INTERNAL BITS
		EXTEND			# SENSETYP WILL NOT OPPOSE ANYTRANS
		RAND	CHAN31		# READ THC +X AXIS POSITION (BIT 7)
		EXTEND			# TEST IF BIT 7 IS SET
		BZF	+X0RULGE	# IF SET, CREW COMMANDS +X TRANSLATION

; If neither +X nor -X translation is commanded by the crew, the autopilot
; checks for automatic ullage requirements or continues with null translation.

# Page 1443
		CA	BIT8		# CHECK THC -X AXIS POSITION (BIT 8)
		EXTEND
		RAND	CHAN31		# READ CHANNEL 31 BIT 8
		EXTEND
		BZF	-XTRANS		# IF SET, CREW COMMANDS -X TRANSLATION

; No crew translation command detected. Check for automatic ullage requirement.
; Ullage uses +X RCS jets to settle propellant in tanks before main engine ignition.

		CA	ULLAGER		# CHECK ULLAGE FLAG
		MASK	DAPBOOLS	# TEST IF ULLAGE MODE ACTIVE
		CCS	A		# IS ULLAGER SET?
		TCF	+X0RULGE	# YES, EXECUTE +X TRANSLATION FOR ULLAGE

; No translation commanded (neither crew nor automatic ullage). Set null policy.

		TS	NEXTU		# STORE NULL TRANSLATION POLICIES
		TS	NEXTV		# (A IS ZERO FROM CCS TEST)
		CS	DAPBOOLS	# BURNING OR DRIFTING?
		MASK	DRIFTBIT	# CHECK IF SPACECRAFT IS COASTING
		EXTEND
		BZF	TSENSE		# IF DRIFTING, SET SENSETYP TO ZERO
		CA	FLGWRD10	# DPS (INCLUDING DOCKED) OR APS?
		MASK	APSFLBIT	# CHECK WHICH PROPULSION SYSTEM ACTIVE
		CCS	A		# IS APS FLAG SET?
		CAF	TWO		# FAVOR +X JETS DURING AN APS BURN.
TSENSE		TS	SENSETYP	# STORE TRANSLATION SENSE TYPE
		TCF	QRCONTRL	# PROCEED TO Q,R CONTROL MODE SELECTION

; ============================================================================
; +X TRANSLATION OR ULLAGE PATH
;
; When the crew commands +X translation via THC or automatic ullage is active,
; the autopilot selects appropriate RCS jets to provide forward thrust while
; minimizing unwanted rotation torques. The ROTINDEX value determines which
; jet configuration is used based on current spacecraft attitude.
; ============================================================================

+X0RULGE	CAF	ONE		# +X TRANSLATION INDEX = 1
-XTRANS		AD	FOUR		# -X TRANSLATION INDEX = 5 (1+4)
		TS	ROTINDEX	# STORE ROTATION/TRANSLATION INDEX
		AD	NEG3		# COMPUTE SENSETYP (ROTINDEX - 3)
		TS	SENSETYP	# FAVOR APPROPRIATE JETS DURING TRANS.
		CA	DAPBOOLS	# CHECK TRANSLATION JET CONFIGURATION
		MASK	ACC4OR2X	# 4-JET OR 2-JET X-AXIS TRANSLATION?
		CCS	A		# TEST BIT
		TCF	TRANS4		# USE 4-JET TRANSLATION MODE

; 2-jet translation mode: Use diagonal jet pair (A system or B system).
; This conserves propellant compared to 4-jet mode but provides less thrust.

		CA	DAPBOOLS	# WHICH RCS SYSTEM?
		MASK	AORBTRAN	# A-SYSTEM OR B-SYSTEM FOR 2-JET?
		CCS	A		# TEST SYSTEM SELECTION BIT
		CA	ONE		# THREE FOR B SYSTEM
		AD	TWO		# TWO FOR A SYSTEM 2-JET X TRANS
TSNUMBRT	TS	NUMBERT		# STORE JET CONFIGURATION NUMBER

; Select specific jet policy based on configuration and failure status.
; SELCTSUB routine checks for failed jets and chooses working alternatives.

		TC	SELCTSUB	# CALL JET SELECTION SUBROUTINE

; Check if a valid jet policy was found. If all jets failed, issue alarm.

		CCS	POLYTEMP	# WAS VALID POLICY FOUND?
		TCF	+3		# YES, SKIP ALARM
		TC	ALARM		# NO JETS AVAILABLE
		OCT	02002		# ALARM CODE: TRANSLATION JETS FAILED
		CA	00314OCT
		MASK	POLYTEMP
TSNEXTS		TS	NEXTU
# Page 1444
		CS	00314OCT
		MASK	POLYTEMP
		TS	NEXTV

# Q,R-AXES RCS CONTROL MODE SELECTION
#	SWITCHES	INDICATION WHEN SET
#	BIT13/CHAN31	AUTO, GO TO ATTSTEER
#	PULSES		MINIMUM IMPULSE MODE
#	(OTHERWISE)	RATE COMMAND/ATTITUDE HOLD MODE

QRCONTRL	CA	BIT13		# CHECK MODE SELECT SWITCH.
		EXTEND
		RAND	CHAN31		# BITS INVERTED
		CCS	A
		TCF	ATTSTEER
CHKBIT10	CAF	PULSES		# PULSES = 1 FOR MIN IMP USE OF RHC
		MASK	DAPBOOLS
		EXTEND
		BZF	CHEKSTIK	# IN ATT-HOLD/RATE-COMMAND IF BIT10=0

# MINIMUM IMPULSE MODE

		INHINT
		TC	IBNKCALL
		CADR	ZATTEROR
		CA	ZERO
		TS	QERROR
		TS	RERROR		# FOR DISPLAYS
		RELINT

		EXTEND
		READ	CHAN31
		TS	TEMP31		# IS EQUAL TO DAPTEMP1
		CCS	OLDQRMIN
		TCF	CHECKIN

FIREQR		CA	TEMP31
		MASK	BIT1
		EXTEND
		BZF	+QMIN

		CA	TEMP31
		MASK	BIT2
		EXTEND
		BZF	-QMIN

		CA	TEMP31
		MASK	BIT5
# Page 1445
		EXTEND
		BZF	+RMIN

		CA	TEMP31
		MASK	BIT6
		EXTEND
		BZF	-RMIN

		TCF	XTRANS

CHECKIN		CS	TEMP31
		MASK	OCT63
		TS	OLDQRMIN
		TCF	XTRANS

+QMIN		CA	14MS
		TS	TJU
		CS	14MS
		TCF	MINQR
-QMIN		CS	14MS
		TS	TJU
		CA	14MS
		TCF	MINQR
+RMIN		CA	14MS
		TCF	+2
-RMIN		CS	14MS
		TS	TJU
MINQR		TS	TJV
		CA	MINADR
		TS	RETJADR
		CA	ONE
		TS	OLDQRMIN
MINRTN		TS	AXISCTR
		CA	DAPBOOLS
		MASK	CSMDOCKD
		EXTEND
		BZF	MIMRET
		INDEX	AXISCTR		# IF DOCKED, USE 60MS MINIMUM IMPULSE
		CCS	TJU
		CA	60MS
		TCF	+2
		CS	60MS
		INDEX	AXISCTR
		TS	TJU
MIMRET		CA	DAPBOOLS
		MASK	AORBTRAN
		CCS	A
		CA	ONE
		AD	TWO
		TS	NUMBERT
# Page 1446
		TCF	AFTERTJ

60MS		DEC	96		# RSB 2009 -- was 96.0.
MINADR		GENADR	MINRTN
OCT63		OCT	63
14MS		=	+TJMINT6

; ============================================================================
; TRANS4 - FOUR-JET TRANSLATION MODE
;
; When 4-jet translation mode is selected (ACC4OR2X bit set), all four
; translation jets fire simultaneously to provide maximum thrust along the
; X-axis. This mode uses more propellant than 2-jet mode but delivers higher
; acceleration, which is important during critical maneuvers like ullage
; burns before engine ignition.
;
; The routine sets NUMBERT to FOUR, indicating that a 4-jet translation
; policy should be used. Control then transfers to TSNUMBRT to store this
; configuration and continue jet selection processing.
; ============================================================================

TRANS4		CA	FOUR		# SELECT 4-JET TRANSLATION POLICY
		TCF	TSNUMBRT	# BRANCH TO STORE JET NUMBER

; ============================================================================
; RATE COMMAND MODE
;
; COMMENT-ONLY READERS: This section handles manual spacecraft control when
; the astronaut moves the Rotation Hand Controller (RHC) out of its center
; detent position. The spacecraft responds by rotating at a rate proportional
; to the stick deflection. During Apollo 11's mission, Armstrong and Aldrin
; used this mode extensively for attitude control during LM operations.
;
; CODE-ALONG READERS: Rate command mode implements a direct rate control law
; where commanded angular rate is proportional to stick deflection. The control
; system distinguishes between three states:
;   1. Stick in detent (center position) - damping mode active
;   2. Stick deflected below breakout threshold - pseudo-auto mode
;   3. Stick deflected above breakout threshold - direct rate control
;
; The rate command law uses quadratic sensitivity: WC = A*(B + |D|)*D
; where WC is commanded rate, A is sensitivity factor, B is linear constant,
; and D is stick deflection. This provides fine control near center and
; higher rates at full deflection.
; ============================================================================

# RATE COMMAND MODE:
#
# DESCRIPTION (SAME AS P-AXIS)

CHEKSTIK	TS	INGTS		# NOT IN GTS WHEN IN ATT HOLD
		CS	ONE		# 1/ACCS WILL DO THE NULLING DRIVES
		TS	COTROLER	# COME BACK TO RCS NEXT TIME
		CA	BIT15
		MASK	CH31TEMP
		EXTEND
		BZF	RHCACTIV	# BRANCH IF OUT OF DETENT.
		CA	OURRCBIT	# ***********
		MASK	DAPBOOLS	# *IN DETENT*	CHECK FOR MANUAL CONTROL
		EXTEND			# ***********	LAST TIME.
		BZF	STILLRCS
		CS	BIT9
		MASK	RCSFLAGS
		TS	RCSFLAGS	# BIT 9 IS 0.
		TCF	DAMPING
40CYCL		OCT	50
1/10S		OCT	1
LINRAT		DEC	46

; ============================================================================
; DAMPING MODE - STICK IN DETENT
;
; When the RHC is in its center detent position, the autopilot enters damping
; mode where it actively reduces spacecraft rotation rates to zero using RCS
; thrusters. This provides stable attitude hold without continuous astronaut
; input. The SAVEHAND registers are cleared to indicate no manual commands.
; ============================================================================

# ===========================================================

DAMPING		CA	ZERO		# CLEAR MANUAL RATE COMMAND STORAGE
		TS	SAVEHAND	# Q-AXIS RATE COMMAND = 0
		TS	SAVEHAND +1	# R-AXIS RATE COMMAND = 0

; ============================================================================
; RHCACTIV - ROTATION HAND CONTROLLER ACTIVE (STICK OUT OF DETENT)
;
; This routine implements the quadratic rate command law for manual control.
; The commanded angular rate increases quadratically with stick deflection,
; providing fine control authority near center position and progressively
; higher rates as the stick is deflected further from center.
;
; Rate command equation: WC = A * (B + |D|) * D
; where:
;   WC = Commanded rotational rate (deg/sec)
;   A  = Quadratic sensitivity factor (STIKSENS)
;   B  = Linear rate constant (LINRAT = 46)
;   D  = Stick deflection (from SAVEHAND)
;
; The CCS instruction tests stick deflection sign and magnitude, followed by
; two DOUBLE instructions (multiply by 4) to scale the deflection value.
; Adding LINRAT provides the linear component, then multiplication by STIKSENS
; applies the sensitivity factor to produce the final commanded rate.
; ============================================================================

RHCACTIV	CCS	SAVEHAND	# ******************
		TCF	+3		# Q,R MANUAL CONTROL	WC = A*(B+|D|)*D
		TCF	+2		# ******************
		TCF	+1		# TEST STICK DEFLECTION SIGN/MAGNITUDE
		DOUBLE			# WHERE
		DOUBLE			# MULTIPLY DEFLECTION BY 4 (2 DOUBLE OPS)
		AD	LINRAT		# 	WC  = COMMANDED ROTATIONAL RATE
		EXTEND			#	A   = QUADRATIC SENSITIVITY FACTOR
		MP	SAVEHAND	#	B   = LINEAR/QUADRATIC SENSITIVITY
		CA	L		#	|D| = ABS. VALUE OF DEFLECTION
		EXTEND			#	D   = HAND CONTROLLER DEFLECTION
		MP	STIKSENS	# APPLY SENSITIVITY FACTOR
		XCH	QLAST		# COMMAND Q RATE, SCALED 45 DEG/SEC
		COM			# COMPLEMENT TO GET DELTA
# Page 1447
		AD	QLAST		# COMPUTE CHANGE IN Q COMMAND
		TS	DAPTEMP3	# STORE FOR BREAKOUT CHECK

; R-AXIS (ROLL) RATE COMMAND COMPUTATION
; Same quadratic rate law applied to roll axis using SAVEHAND+1 deflection.

		CCS	SAVEHAND +1	# TEST R-AXIS STICK DEFLECTION
		TCF	+3		# PROCESS R-AXIS RATE COMMAND
		TCF	+2
		TCF	+1
		DOUBLE			# MULTIPLY BY 4
		DOUBLE
		AD	LINRAT		# ADD LINEAR COMPONENT
		EXTEND
		MP	SAVEHAND +1	# MULTIPLY BY DEFLECTION
		CA	L		# GET RESULT FROM L REGISTER
		EXTEND
		MP	STIKSENS	# APPLY SENSITIVITY FACTOR
		XCH	RLAST		# COMMAND R RATE, SCALED 45 DEG/SEC
		COM			# COMPLEMENT TO GET DELTA
		AD	RLAST		# COMPUTE CHANGE IN R COMMAND
		TS	DAPTEMP4	# STORE FOR BREAKOUT CHECK
; ============================================================================
; RATE DIFFERENCE CALCULATION AND COORDINATE TRANSFORMATION
;
; Compute the difference between commanded rates (from stick input) and
; actual spacecraft rates (from IMU). These rate errors drive the RCS
; thruster firing logic. The rates are then transformed from spacecraft
; body axes (Q,R) to jet coordinate axes (U,V) for thruster selection.
; ============================================================================

		CS	QLAST		# COMPUTE Q-AXIS RATE ERROR
		AD	OMEGAQ		# COMMANDED RATE - ACTUAL RATE
		TS	QRATEDIF	# STORE Q RATE DIFFERENCE
		CS	RLAST		# COMPUTE R-AXIS RATE ERROR
		AD	OMEGAR		# COMMANDED RATE - ACTUAL RATE
		TS	RRATEDIF	# STORE R RATE DIFFERENCE

; ============================================================================
; ENTERQR - BREAKOUT THRESHOLD DETECTION
;
; The autopilot implements a "breakout" threshold to distinguish between small
; stick motions (pseudo-auto mode) and large stick deflections (direct rate
; control mode). If the change in commanded rate exceeds the breakout deadband
; (-RATEDB), the system enters direct rate control where stick position
; directly commands spacecraft rotation rate. This provides crisp response to
; astronaut inputs while filtering out unintentional small stick movements.
;
; During Apollo 11, this breakout logic allowed Armstrong to make precise
; attitude adjustments during critical phases like docking and landing site
; selection without the spacecraft being overly sensitive to minor inputs.
; ============================================================================

ENTERQR		DXCH	QRATEDIF	# TRANSFORM RATES FROM Q,R TO U,V AXES
		TC	ROT-TOUV	# CALL ROTATION TRANSFORMATION
		DXCH	URATEDIF	# STORE TRANSFORMED RATES
		CCS	DAPTEMP3	# CHECK IF Q COMMAND CHANGE EXCEEDS
		TC	+3		# BREAKOUT LEVEL.  IF NOT, CHECK R.
		TC	+2
		TC	+1
		AD	-RATEDB		# SUBTRACT BREAKOUT DEADBAND
		EXTEND
		BZMF	+2		# BRANCH IF BELOW THRESHOLD
		TCF	ENTERUV -2	# BREAKOUT LEVEL EXCEEDED.  DIRECT RATE.
		CCS	DAPTEMP4	# R COMMAND BREAKOUT CHECK.
		TC	+3
		TC	+2
		TC	+1
		AD	-RATEDB		# SUBTRACT BREAKOUT DEADBAND
		EXTEND
		BZMF	+2		# BRANCH IF BELOW THRESHOLD
		TCF	ENTERUV -2	# BREAKOUT LEVEL EXCEEDED.  DIRECT RATE.

; ============================================================================
; PSEUDO-AUTO MODE LOGIC
;
; When stick deflection changes are below the breakout threshold, the autopilot
; enters pseudo-auto mode - a hybrid control mode that combines manual rate
; commands with automatic attitude stabilization. This mode provides smooth
; transitions between manual and automatic control, preventing abrupt changes
; in spacecraft behavior when the astronaut makes small stick adjustments.
; ============================================================================

		CA	RCSFLAGS	# BREAKOUT LEVEL NOT EXCEEDED.  CHECK FOR
		MASK	QRBIT		# DIRECT RATE CONTROL LAST TIME (BIT 11)
		EXTEND
		BZF	+2		# IF QRBIT=0, USE PSEUDO-AUTO
		TCF	ENTERUV		# IF QRBIT=1, CONTINUE DIRECT RATE CONTROL
		TCF	STILLRCS	# PSEUDO-AUTO CONTROL.
		CA	40CYCL		# LOAD 4-SECOND TIMER VALUE
# Page 1448
		TS	TCQR		# INITIALIZE DIRECT RATE TIMER

; ============================================================================
; ENTERUV - DIRECT RATE CONTROL MODE
;
; In direct rate control mode, the spacecraft rotates at rates directly
; commanded by stick position. The autopilot zeroes attitude errors to prevent
; interference from the attitude control laws, then checks if rates have
; returned to within target deadband. If rates are nulled and stick is near
; center for 4 seconds, the system transitions back to pseudo-auto mode.
;
; This mode gives the astronaut full manual authority over rotation rates,
; essential during rendezvous, docking, and landing operations where precise
; manual control is required. Armstrong used this mode extensively during the
; final approach to the lunar surface.
; ============================================================================

ENTERUV		INHINT			# DIRECT RATE CONTROL
		TC	IBNKCALL	# CALL CROSS-BANK SUBROUTINE
		FCADR	ZATTEROR	# ZERO ALL ATTITUDE ERRORS
		RELINT			# RE-ENABLE INTERRUPTS

; Zero attitude errors to prevent attitude control laws from interfering
; with direct rate commands. This allows pure rate control.

		CA	ZERO		# ZERO Y-AXIS ATTITUDE ERROR
		TS	DYERROR		# HIGH-ORDER WORD
		TS	DYERROR +1	# LOW-ORDER WORD
		TS	DZERROR		# ZERO Z-AXIS ATTITUDE ERROR
		TS	DZERROR +1	# HIGH-ORDER AND LOW-ORDER

; ============================================================================
; TARGET DEADBAND CHECK - TRANSITION TO PSEUDO-AUTO
;
; After zeroing attitude errors, check if actual rates have returned to within
; the target deadband. If both U and V axis rates are within deadband and
; remain so for 4 seconds (via TCQR timer), the system transitions from direct
; rate mode back to pseudo-auto mode. This provides smooth transition back to
; automatic control when the astronaut releases or centers the stick.
; ============================================================================

		CCS	URATEDIF	# CHECK U-AXIS RATE ERROR MAGNITUDE
		TCF	+3		# PROCESS U RATE DIFFERENCE
		TCF	+2
		TCF	+1
		AD	TARGETDB	# IF TARGET DB IS EXCEEDED, CONTINUE
		EXTEND			# DIRECT RATE CONTROL.
		BZMF	VDB		# U WITHIN DEADBAND, CHECK V
		CCS	VRATEDIF	# U EXCEEDS DEADBAND, CHECK V
		TCF	+3
		TCF	+2
		TCF	+1
		AD	TARGETDB	# CHECK V RATE AGAINST DEADBAND
		EXTEND
		BZMF	+2		# V WITHIN DEADBAND
		TCF	QRTIME		# V EXCEEDS DEADBAND, CONTINUE DIRECT
		CA	ZERO		# U EXCEEDS, V WITHIN - ZERO V RATE
		TS	VRATEDIF	# FOR CONTROL COMPUTATION
		TCF	QRTIME		# CONTINUE DIRECT RATE CONTROL
VDB		CCS	VRATEDIF	# U WITHIN DEADBAND, CHECK V
		TC	+3
		TC	+2
		TC	+1
		AD	TARGETDB	# IF TARGET DB IS EXCEEDED, CONTINUE
		EXTEND			# DIRECT RATE CONTROL.  IF NOT, FIRE AND
		BZMF	TOPSEUDO	# SWITCH TO PSEUDO-AUTO CONTROL ON NEXT
		CA	ZERO		# PASS. V EXCEEDS, U WITHIN - ZERO U
		TS	URATEDIF	# FOR CONTROL COMPUTATION
QRTIME		CA	TCQR		# DIRECT RATE TIME CHECK.
		EXTEND
		BZMF	+5		# BRANCH IF TIME EXCEEDS 4 SEC.
		CS	RCSFLAGS
		MASK	QRBIT
		ADS	RCSFLAGS	# BIT 11 IS 1.
		TC	+4
TOPSEUDO	CS	QRBIT
		MASK	RCSFLAGS
		TS	RCSFLAGS	# BIT 11 IS 0.
		CA	HANDADR
		TS	RETJADR
		CA	ONE

# Page 1449
BACKHAND	TS	AXISCTR

		CA	FOUR
		TS	NUMBERT

		INDEX	AXISCTR
		INDEX	SKIPU
		TCF	+1
		CA	FOUR
		INDEX	AXISCTR
		TS	SKIPU
		TCF	LOOPER

		INDEX	AXISCTR
		CCS	URATEDIF	#	INDEX	AXIS	QUANTITY
		CA	ZERO		#	0	-U	1/JETACC-AOSU
		TCF	+2		#	1	+U	1/JETACC+AOSU
		CA	ONE		#	16	-V	1/JETACC-AOSV
		INDEX	AXISCTR		#	17	+V	1/JETACC+AOSV
		AD	AXISDIFF	# JETACC = 2 JET ACCELERATION (1 FOR FAIL)

		INDEX	A
		CS	1/ANET2 +1
		EXTEND
		INDEX	AXISCTR		# UPRATEDIF IS SCALED AT PI/4 RAD/SEC
		MP	URATEDIF	# JET TIME IN A, SCALED 32 SEC
		TS	Q
		DAS	A
		AD	Q
		TS	A		# OVERFLOW SKIP
		TCF	+2
		CA	Q		# RIGHT SIGN AND BIGGER THAN 150MS
SETTIME		INDEX	AXISCTR
		TS	TJU		# SCALED AT 10.67 WHICH IS CLOSE TO 10.24
		TCF	AFTERTJ

ZEROTJ		CA	ZERO
		TCF	SETTIME

HANDADR		GENADR	BACKHAND

# GTS WILL BE TRIED IF
#	1. USEQRJTS = 0,
#	2. ALLOWGTS POS,
#	3. JETS ARE OFF (Q,R-AXES)

TRYGTS		CAF	USEQRJTS	# IS JET USE MANDATORY.		(AS LONG AS
		MASK	DAPBOOLS	# USEQRJTS BIT IS NOT BIT 15, CCS IS SAFE.)
		CCS	A
		TCF	RCS
		CCS	ALLOWGTS	# NO.  DOES AOSTASK OK CONTROL FOR GTS?
# Page 1450
		TCF	+2
		TCF	RCS
		EXTEND
		READ	CHAN5
		CCS	A
		TCF	CHKINGTS
GOTOGTS		EXTEND
		DCA	GTSCADR
		DTCB

CHKINGTS	CCS	INGTS		# WAS THE TRIM GIMBAL CONTROLLING
		TCF	+2		#	YES.  SET UP A DAMPED NULLING DRIVE.
		TCF	RCS		#	NO.  NULLING WAS SET UP BEFORE.  DO RCS.
		INHINT
		TC	IBNKCALL
		CADR	TIMEGMBL
		RELINT
		CAF	ZERO
		TS	INGTS
		TCF	RCS

		EBANK=	CDUXD
GTSCADR		2CADR	GTS

# Page 1451
# SUBROUTINE TO COMPUTE Q,R-AXES ATTITUDE ERRORS FOR USE IN THE RCS AND GTS CONTROL LAWS AND THE DISPLAYS.

QERRCALC	CAE	CDUY		# Q-ERROR CALCULATION
		EXTEND
		MSU	CDUYD		# CDU ANGLE -- ANGLE DESIRED (Y-AXIS)
		TS	DAPTEMP1	# SAVE FOR RERRCALC
		EXTEND
		MP	M21		# (CDUY-CDUYD)*M21 SCALED AT PI RADIANS
		TS	E
		CAE	CDUZ		# SECOND TERM CALCULATION:
		EXTEND
		MSU	CDUZD		# CDU ANGLE -ANGLE DESIRED (Z-AXIS)
		TS	DAPTEMP2	# SAVE FOR RERRCALC
		EXTEND
		MP	M22		# (CDUZ-CDUZD)*M22 SCALED AT PI RADIANS
		AD	DELQEROR	# KALCMANU INERFACE ERROR
		AD	E
		XCH	QERROR		# SAVE Q-ERROR FOR EIGHT-BALL DISPLAY.

RERRCALC	CAE	DAPTEMP1	# R-ERROR CALCULATION:
		EXTEND			# CDU ANGLE -ANGLE DESIRED (Y-AXIS)
		MP	M31		# (CDUY-CDUYD)*M31 SCALED AT PI RADIANS
		TS	E
		CAE	DAPTEMP2	# SECOND TERM CALCULATION:
		EXTEND			# CDU ANGLE -ANGLE DESIRED (Z-AXIS)
		MP	M32		# (CDUZ-CDUZD)*M32 SCALED AT PI RADIANS
		AD	DELREROR	# KALCMANU INERFACE ERROR
		AD	E
		XCH	RERROR		# SAVE R-ERROR FOR EIGHT-BALL DISPLAY.
		TC	Q

# Page 1452
; ============================================================================
; ATTSTEER - Official Q,R-Axis RCS Attitude Control Entry Point
;
; ATTSTEER is the primary entry point for Q,R-axes (yaw/roll) attitude control
; using the Reaction Control System. It is defined as equivalent to STILLRCS,
; which can be entered either directly or as the RCS fallback exit from TRYGTS
; (the Gimbal Trim System attempt path).
;
; This section transforms the Q,R attitude errors (in spacecraft body axes) into
; U,V errors (aligned with the RCS jet thrust axes) using the rotation matrix.
; The transformation accounts for the physical mounting angles of the RCS jets
; relative to the spacecraft's attitude reference frame.
;
; During Apollo 11's landing, this routine continuously processed yaw and roll
; errors, keeping the Lunar Module properly oriented for radar tracking and
; crew visibility during the historic descent to the Sea of Tranquility.
;
; COMMENT-ONLY READERS: This is where spacecraft yaw/roll errors are converted
; into jet firing commands for attitude stabilization.
;
; CODE-ALONG READERS: Note the coordinate transformation from body-fixed Q,R
; axes to jet-aligned U,V axes via the ROT-TOUV subroutine.
; ============================================================================
# "ATTSTEER" IS THE ENTRY POINT FOR Q,R-AXES (U,V-AXES) ATTITUDE CONTROL USING THE REACTION CONTROL SYSTEM

ATTSTEER	EQUALS	STILLRCS	# "STILLRCS" IS THE RCS EXIT FROM TRYGTS.

; The Lunar Module's yaw and roll errors are now converted from spacecraft
; body axes (Q = yaw, R = roll) into the jet thrust axes (U,V) which are
; rotated relative to the body frame due to the physical mounting of RCS jets.

STILLRCS	CA	RERROR		; Load R-axis (roll) error
		LXCH	A		; Save to L register, prepare for transformation
		CA	QERROR		; Load Q-axis (yaw) error into A
		TC	ROT-TOUV	; Transform Q,R errors to U,V jet axes
		DXCH	UERROR		; Store transformed U,V errors

; ============================================================================
; TJLAW - Torque Jet Law Application for Q,R-Axes
;
; This section prepares for and calls the torque jet control law (TJETLAW)
; for each axis (U and V). It implements skip logic that allows an axis to
; bypass jet firing if a very short pulse (<150ms) is still pending from a
; previous cycle. This prevents excessive jet cycling and conserves propellant.
;
; The routine operates on both U-axis (first iteration, AXISCTR=1) and V-axis
; (second iteration, AXISCTR=0) in sequence. For docked operations, it checks
; whether the Gimbal Trim System can be used instead of RCS jets for control.
;
; COMMENT-ONLY READERS: This loop calculates jet firing times for yaw and roll
; corrections, checking if jets from the previous cycle need to complete first.
;
; CODE-ALONG READERS: AXISCTR counts down from 1 to 0, processing U-axis then
; V-axis. SKIPU/SKIPV flags indicate pending short pulses from prior cycles.
; ============================================================================
# PREPARES CALL TO TJETLAW (OR SPSRCS(DOCKED))
# PREFORMS SKIP LOGIC ON U OR Y AXIS IF NEEDED.

TJLAW		CA	TJLAWADR	; Load return address for TJETLAW
		TS	RETJADR		; Save return address
		CA	ONE		; Initialize axis counter to 1 (U-axis first)
		TS	AXISCTR		; AXISCTR: 1=U-axis, 0=V-axis
		
		; Skip logic: If SKIPU (or SKIPV) is non-zero, a short pulse from
		; the previous cycle is still pending. Skip jet calculation for
		; this axis and proceed to the next axis (LOOPER).
		INDEX	AXISCTR		; Index by axis: SKIPU if 1, SKIPV if 0
		INDEX	SKIPU		; Double index: skip next instruction if non-zero
		TCF	+1		; Skip taken if SKIP flag clear (no pending pulse)
		CA	FOUR		; SKIP flag was set, reset it to 4 (clear state)
		INDEX	AXISCTR		; Index by axis
		TS	SKIPU		; Clear the skip flag (SKIPU or SKIPV)
		TCF	LOOPER		; Jump to process next axis without jet calculation
		
		; No skip needed, proceed with jet law calculation for this axis
		INDEX	AXISCTR		; Index by axis
		CA	UERROR		; Load attitude error (UERROR for U, VERROR for V)
		TS	E		; Store error to E for TJETLAW
		INDEX	AXISCTR		; Index by axis
		CA	OMEGAU		; Load angular rate (OMEGAU for U, OMEGAV for V)
		TS	EDOT		; Store rate to EDOT for TJETLAW
		
		; Check if spacecraft is docked - docked ops may use Gimbal Trim System
		CA	DAPBOOLS	; Load DAP boolean flags
		MASK	CSMDOCKD	; Check docked configuration bit
		CCS	A		; Test docked flag
		TCF	+3		; If docked, branch to GTS check
		TC	TJETLAW		; Not docked: call torque jet law (RCS control)
		TCF	AFTERTJ		; Process TJETLAW results
 +3		CS	DAPBOOLS	# DOCKED.  IF GIMBAL USABLE DO GTS CONTROL
		MASK	USEQRJTS	#	ON THE NEXT PASS.
		CCS	A		# USEQRJTS BIT MUST NOT BE BIT 15.
		TS	COTROLER	# GIMBAL USABLE.  STORE POSITIVE VALUE.
		INHINT
		TC	IBNKCALL
		CADR	SPSRCS		# DETERMINE RCS CONTROL
		RELINT
		CAF	FOUR		# ALWAYS CALL FOR 2-JET CONTROL ABOUT U,V.
		TS	NUMBERT		# FALL THROUGH TO JET SELECTION, ETC.

# Q,R-JET-SELECTION-LOGIC
#
# INPUT:	AXISCTR		0,1 FOR U,V
#		SNUFFBIT	ZERO TJETU,V AND TRANS. ONLY IF SET IN A DPS BURN
# Page 1453
#		TJU,TJV		JET TIME SCALED 10.24 SEC.
#		NUMBERT		INDICATES NUMBER OF JETS AND TYPE OF POLICY
#		RETJADR		WHERE TO RETURN TO
#
# OUTPUT:	NO.U(V)JETS	RATE DERIVATION FEEDBACK
#		CHANNEL 5
#		SKIPU,SKIPV	FOR LESS THAN 150MS FIRING
#
# NOTES:	IN CASE OF FAILURE IN DESIRED ROTATION POLICY, "ALL" UNFAILED
#		JETS OF THE DESIRED POLICY ARE SELECTED.  SINCE THERE ARE ONLY
#		TWO JETS, THIS MEANS THE OTHER ONE OR NONE.  THE ALARM IS SENT
#		IF NONE CAN BE FOUND.
#
#		TIMES LESS THAN 14 MSEC ARE TAKEN TO CALL FOR A SINGLE-JET
#		MINIMUM IMPULSE, WITH THE JET CHOSEN SEMI-RANDOMLY.

AFTERTJ		CA	FLAGWRD5	# IF SNUFFBIT SET DURING A DPS BURN GO TO
		MASK	SNUFFBIT	# XTRANS; THAT IS, INHIBIT CONTROL.
		EXTEND
		BZF	DOROTAT
		CS	FLGWRD10
		MASK	APSFLBIT
		EXTEND
		BZF	DOROTAT
		CA	DAPBOOLS
		MASK	DRIFTBIT
		EXTEND
		BZF	XTRANS

DOROTAT		CAF	TWO
		TS	L
		INDEX	AXISCTR
		CCS	TJU
		TCF	+5
		TCF	NOROTAT
		TCF	+2
		TCF	NOROTAT
		ZL
		AD	ONE
		TS	ABSTJ

		CA	AXISCTR
		AD	L
		TS	ROTINDEX	# 0 1 2 3 = -U -V +U +V

		CA	ABSTJ
		AD	-150MS
		EXTEND
		BZMF	DOSKIP
# Page 1454
		TC	SELCTSUB

		INDEX	AXISCTR
		CA	INDEXES
		TS	L

		CA	POLYTEMP
		INHINT
		INDEX	L
		TC	WRITEP

		RELINT
		TCF	FEEDBACK

NOROTAT		INDEX	AXISCTR
		CA	INDEXES
		INHINT
		INDEX	A
		TC	WRITEP 	-1

		RELINT
LOOPER		CCS	AXISCTR
		TC	RETJADR
		TCF	CLOSEOUT
DOSKIP		CS	ABSTJ
		AD	+TJMINT6	# 14MS
		EXTEND
		BZMF	NOTMIN

		ADS	ABSTJ
		INDEX	AXISCTR
		CCS	TJU
		CA	+TJMINT6
		TCF	+2
		CS	+TJMINT6
		INDEX	AXISCTR
		TS	TJU

		CCS	SENSETYP	# ENSURE MIN-IMPULSE NOT AGAINST TRANS
		TCF	NOTMIN 	-1
		EXTEND
		READ	LOSCALAR
		MASK	ONE
		TS	NUMBERT

NOTMIN		TC	SELCTSUB

		INDEX	AXISCTR
		CA	INDEXES
		INHINT
# Page 1455
		TS	T6FURTHA +1
		CA	POLYTEMP
		INDEX	T6FURTHA +1
		TC	WRITEP

		CA	ABSTJ
		TS	T6FURTHA
		TC	JTLST		# IN QR BANK BY NOW

		RELINT

		CA	ZERO
		INDEX	AXISCTR
		TS	SKIPU

FEEDBACK	CS	THREE
		AD	NUMBERT
		EXTEND
		BZMF	+3

		CA	TWO
		TCF	+2
		CA	ONE
		INDEX	AXISCTR
		TS	NO.UJETS
		TCF	LOOPER

XTRANS		CA	ZERO
		TS	TJU
		TS	TJV
		CA	FOUR
		INHINT
		XCH	SKIPU
		EXTEND
		BZF	+2
		TC	WRITEU 	-1
		CA	FOUR
		XCH	SKIPV
		RELINT

		EXTEND
		BZF	CLOSEOUT
		INHINT
		TC	WRITEV 	-1
		RELINT

		TCF	CLOSEOUT
INDEXES		DEC	4
		DEC	13
+TJMINT6	DEC	22
# Page 1456
-150MS		DEC	-240
BIT8,9		OCT	00600
SCLNORM		OCT	266
TJLAWADR	GENADR	TJLAW 	+3	# RETURN ADDRESS FOR RCS ATTITUDE CONTROL

; ============================================================================
; JET LIST (JTLST) - T6RUPT Interrupt Scheduling for RCS Jet Control
;
; The jet list is a specialized waitlist that schedules T6RUPT interrupts for
; future RCS jet commands. This timing mechanism allows the Digital Autopilot
; to plan ahead for jet firings and turn-offs, enabling smooth attitude control
; and minimizing propellant waste through precise jet firing durations.
;
; The list maintains exactly 3 scheduled events in chronological order:
;   TIME6     - Absolute time of the next (soonest) scheduled jet command
;   T6NEXT    - Delta time from TIME6 to the second scheduled event
;   T6FURTHA  - Delta time from second to third (furthest) scheduled event
;
; Each event also stores an axis index identifying which axis (P, U, or V)
; will be commanded at that time:
;   NXT6ADR     - Axis for TIME6 event (0=P-axis, 4=U-axis, 13=V-axis)
;   T6NEXT+1    - Axis for second event
;   T6FURTHA+1  - Axis for third event
;
; When a new event is scheduled via JTLST, it is inserted as the third (last)
; member of the list. The routine then reorganizes the list to maintain
; chronological order. When TIME6 is reached, a T6RUPT interrupt fires,
; executes the jet command for that axis, and advances the list.
;
; The 150-millisecond limit mentioned in the original comments refers to the
; maximum time span the list can handle - all three events must occur within
; 150ms of TIME6. This constraint, combined with the skip logic (SKIPU/SKIPV),
; prevents excessive jet cycling during rapid maneuvers.
;
; COMMENT-ONLY READERS: Think of this as a calendar scheduling system for the
; RCS jets. The computer plans up to three future jet commands in advance,
; ensuring smooth coordinated control of all three rotation axes without
; conflicts. During Apollo 11's landing, this precise timing kept the LM
; stable while Armstrong searched for a safe landing site.
;
; CODE-ALONG READERS: The JTLST routine implements a priority queue insertion
; algorithm optimized for exactly 3 elements. The use of delta times (rather
; than absolute times for all events) minimizes storage and simplifies the
; interrupt handler's time arithmetic.
; ============================================================================

# THE JET LIST:
# THIS IS A WAITLIST FOR T6RUPTS.
#
# CALLED BY:
#		CA	TJ		# TIME WHEN NEXT JETS WILL BE WRITTEN
#		TS	T6FURTHA
#		CA	INDEX		# AXIS TO BE WRITTEN AT TJ (FROM NOW)
#		TS	T6FURTHA +1
#		TC	JTLST
#
# EXAMPLE -- U-AXIS AUTOPILOT WILL WRITE ITS ROTATION CODE OF
# JETS INTO CHANNEL 5.  IF IT DESIRES TO TURN OFF THIS POLICY WITHIN
# 150MS AND THEN FIRE NEXTU, A CALL TO JTLST IS MADE WITH T6FURTHA
# CONTAINING THE TIME TO TURN OFF THE POLICY, T6FURTHA +1 THE INDEX
# OF THE U-AXIS(4), AND NEXTU WILL CONTAIN THE "U-TRANS" POLICY OR ZERO.
#
# THE LIST IS EXACTLY 3 LONG.  (THIS LEADS UP TO SKIP LOGIC AND 150MS LIMIT)
# THE INPUT IS THE LAST MEMBER OF THE LIST.
#
# RETURNS BY:
#	+	TC	Q
#
# DEFINITIONS:  (OUTPUT)
#	TIME6		TIME OF NEXT RUPT
#	T6NEXT		DELTA TIME TO NEXT RUPT
#	T6FURTHA	DELTA TIME FROM 2ND TO LAST RUPT
#	NXT6ADR		AXIS INDEX	0 -- P-AXIS
#	T6NEXT +1	AXIS INDEX	4 -- U-AXIS
#	T6FURTHA +1	AXIS INDEX	13 -- V-AXIS

; Entry point for JTLST. The new event time and axis index are in T6FURTHA
; and T6FURTHA+1. This routine inserts the new event and reorders the list.
;
; Three cases are handled based on where the new event falls chronologically:
; 1. New event is soonest (earlier than TIME6) - becomes new TIME6
; 2. New event is middle (between TIME6 and TIME6+T6NEXT) - becomes T6NEXT
; 3. New event is last (later than TIME6+T6NEXT) - stays as T6FURTHA

JTLST		CS	T6FURTHA		; Compute TIME6 - T6FURTHA (new event time)
		AD	TIME6			; Result in A register
		EXTEND
		BZMF	MIDORLST		; If TIME6 >= new time, new event is NOT first

; CASE 1: New event occurs BEFORE current TIME6 (new event is soonest).
; Shift entire list forward: TIME6 -> T6NEXT, T6NEXT -> T6FURTHA.
; The new event becomes TIME6.

		LXCH	NXT6ADR			; Save current TIME6 axis index to L
		DXCH	T6NEXT			; Exchange L,A with T6NEXT (shifts data)
		DXCH	T6FURTHA		; Complete the shift
		TS	TIME6			; Store new earliest time in TIME6
		LXCH	NXT6ADR			; Restore axis index to NXT6ADR

; After inserting a new earliest event, enable T6RUPT interrupt to schedule it.

TURNON		CA	BIT15			; T6 interrupt enable bit
		EXTEND
		WOR	CHAN13			; Write OR to channel 13 (enable T6RUPT)
		TC	Q			; Return to caller

# Page 1457
; CASE 2: New event occurs between TIME6 and TIME6+T6NEXT (middle position).
; Only T6NEXT and T6FURTHA need to be adjusted.

MIDORLST	AD	T6NEXT			; Add T6NEXT to comparison (A = TIME6+T6NEXT-new)
		EXTEND
		BZMF	LASTCHG			; If result negative, new event is last

; New event is in middle position. Shift T6NEXT -> T6FURTHA, insert new as T6NEXT.

		LXCH	T6NEXT 	+1		; Save T6NEXT axis index to L
		DXCH	T6FURTHA		; Shift T6NEXT data to T6FURTHA
		EXTEND
		SU	TIME6			; Convert new time to delta from TIME6
		DXCH	T6NEXT			; Store new event as T6NEXT

		TC	Q			; Return to caller

; CASE 3: New event occurs AFTER TIME6+T6NEXT (furthest event).
; The new event is already in T6FURTHA - just adjust delta time format.

LASTCHG		CS	A			; Negate the comparison result
		AD	NEG0			; Adjust for proper delta time format
		TS	T6FURTHA		; Store adjusted delta time

		TC	Q			; Return to caller

; ============================================================================
; ROT-TOUV - Rotation Transformation to U,V Coordinates
;
; This subroutine transforms attitude errors and angular rates from the Q,R
; orthogonal body axis system to the U,V non-orthogonal jet axis system.
; This transformation is essential because RCS jet firings produce torques
; aligned with the U,V axes, not the Q,R axes. In the U,V system, jet
; firings produce NO cross-coupling between axes - a U-axis jet fires pure
; U torque without affecting V, and vice versa.
;
; The transformation equations are:
;   U = COEFFQ * Q + COEFFR * R
;   V = -COEFFQ * Q + COEFFR * R
;
; Where COEFFQ and COEFFR are coefficients determined by the LM's moment of
; inertia ratios and RCS jet geometry. These coefficients account for the
; physical mounting angles of the RCS jets on the Lunar Module.
;
; INPUT:  A register = Q-component (yaw axis quantity)
;         L register = R-component (roll axis quantity)
;
; OUTPUT: A register = U-component (transformed)
;         L register = V-component (transformed)
;
; COMMENT-ONLY READERS: This mathematical transformation converts yaw and roll
; measurements into coordinates aligned with the physical jet mounting angles,
; enabling precise independent control of each axis without interference.
;
; CODE-ALONG READERS: Overflow protection is implemented at two points. If
; either U or V computation overflows, the result is limited to POSMAX/NEGMAX.
; ============================================================================
# ROT-TOUV IS ENTERED WITH THE Q-COMPONENT OF THE QUANTITY TO BE TRANSFORMED IN A AND THE R-COMPONENT IN L.
# ROT-TOUV TRANSFORMS THE QUANTITY INTO THE NON-ORTHOGONAL U-V AXIS SYSTEM.  IN THE U-V SYSTEM NO CROSS-COUPLING IS
# PRODUCED FROM RCS JET FIRINGS.  AT THE COMPLETION OF ROT-TOUV, THE U-COMPONENT OF THE TRANSFORMED QUANTITY IS IN
# A AND THE V-COMPONENT IS IN L.

ROT-TOUV	LXCH	ROTEMP2		# (R) IS PUT INTO ROTEMP2
		EXTEND			; Compute COEFFQ * Q (Q still in A)
		MP	COEFFQ		; Multiply Q by COEFFQ coefficient
		XCH	ROTEMP2		# (R) GOES TO A AND COEFFQ.(Q) TO ROTEMP2
		EXTEND			; Now compute COEFFR * R
		MP	COEFFR		; Multiply R by COEFFR coefficient
		TS	L		# COEFFR.(R) IS PUT INTO L
		AD	ROTEMP2		; Add COEFFQ*Q to form U = COEFFQ*Q + COEFFR*R
		TS	ROTEMP1		# COEFFQ.(Q)+COEFFR.(R) IS PUT IN ROTEMP1
		TCF	+4		; No overflow, continue
		INDEX	A		# COEFFQ.(Q) + COEFFR.(R) HAS OVERFLOWED
		CS	LIMITS		# AND IS LIMITED TO POSMAX OR NEGMAX
		TS	ROTEMP1		; Store limited U value
		CS	ROTEMP2		; Negate COEFFQ*Q for V computation
		AD	L		# -COEFFQ.(Q) + COEFFR.(R) IS NOW IN A
		TS	7		; Temporary storage of V computation
		TCF	+3		; No overflow, continue
		INDEX	A		# -COEFFQ.(Q) + COEFFR.(R) HAS OVERFLOWED
		CS	LIMITS		# AND IS LIMITED TO POSMAX OR NEGMAX
		LXCH	ROTEMP1		# COEFFQ.(Q) + COEFFR.(R) IS PUT INTO L
		TC	Q		; Return: U in A, V in L
; ============================================================================
; SELCTSUB - Jet Selection Subroutine with Failure Recovery
;
; This critical subroutine selects which RCS jets to fire for a commanded
; rotation, while accounting for both the desired translation policy and any
; failed jets. If the primary translation policy cannot provide working jets
; for the rotation, SELCTSUB automatically tries alternate policies until it
; finds a usable combination.
;
; The routine operates in two phases:
; 1. Primary attempt: Tries the current translation policy (indexed by NUMBERT)
; 2. Failure recovery: If primary fails, loops through all 4 translation
;    policies (indices 3,2,1,0) seeking any policy with working jets
;
; If NO policy can provide working jets (all relevant jets failed), the
; routine issues program alarm 02004 and branches to NOROTAT, preventing
; the requested rotation.
;
; JET NUMBERING: The LM has 16 RCS jets total (jets 1,2,5,6,9,10,13,14 are
; the 8 jets used for attitude control). ALLJETS table defines which jets
; produce torque for each axis direction. TYPEPOLY defines which jets are
; available under each translation policy (-X, +X, diagonal A, diagonal B).
;
; COMMENT-ONLY READERS: This routine ensures the Lunar Module can still rotate
; even if some jets have failed, by automatically switching to alternate jet
; configurations. It prevented mission failures when jets malfunctioned.
;
; CODE-ALONG READERS: ROTINDEX (0-3) selects rotation direction from ALLJETS.
; NUMBERT (0-3) selects translation policy from TYPEPOLY. CH5MASK tests jet
; availability (failed jets are masked out). Alarm 02004 = "NO JETS AVAILABLE".
; ============================================================================
SELCTSUB	INDEX	ROTINDEX	; Select jets needed for rotation direction
		CA	ALLJETS		; Load jet pattern: -U, -V, +U, or +V
		INDEX	NUMBERT		; Apply current translation policy
		MASK	TYPEPOLY	; Mask to policy: -X, +X, A, B, or ALL
		TS	POLYTEMP	; Save policy-constrained jet pattern
# Page 1458
		MASK	CH5MASK		; Test if any selected jets are available
		CCS	A		; Check result: zero = all jets failed
		TCF	+2		; Non-zero: at least one jet available, success

		TC	Q		; Success return with jet pattern in POLYTEMP

		; Primary policy failed - begin searching alternate policies
		CA	THREE		; Start search at policy index 3
FAILOOP		TS	NUMBERT		; Set policy index for this iteration
		INDEX	ROTINDEX	; Re-fetch jets for rotation direction
		CA	ALLJETS		; Load jet pattern again
		INDEX	NUMBERT		; Try this policy index
		MASK	TYPEPOLY	; Apply policy mask
		TS	POLYTEMP	; Save result
		MASK	CH5MASK		; Test availability under this policy
		EXTEND			; Prepare for branch
		BZF	FAILOOP -2	; If zero (all failed), try next policy
		; Found working jets under this policy
		CCS	NUMBERT		; Decrement and test policy counter
		TCF	FAILOOP		; More policies to try, continue loop
		; Exhausted all policies with no success - alarm condition
		INDEX	AXISCTR		; Index by axis (U or V)
		TS	TJU		; Store failure indication
		TC	ALARM		; Issue program alarm
		OCT	02004		; Alarm code 02004: NO JETS AVAILABLE
		TCF	NOROTAT		; Branch to no-rotation handler

; ============================================================================
; Jet Pattern Tables
;
; ALLJETS: Defines which jets produce torque for each rotation direction.
; Each entry is an octal bit pattern where each bit represents one RCS jet.
;
; TYPEPOLY: Defines which jets are available under each translation policy.
; Translation policies coordinate attitude control with translation burns.
;   -X policy: Jets producing thrust toward -X (aft)
;   +X policy: Jets producing thrust toward +X (forward)
;   A policy: Diagonal pair A (balanced thrust)
;   B policy: Diagonal pair B (balanced thrust)
;   ALL policy: All jets available (no translation constraint)
; ============================================================================
ALLJETS		OCT	00110		#	-U	6 13
		OCT	00022		#	-V	2 9
		OCT	00204		#	+U	5 14
		OCT	00041		# 	+V	1 10
TYPEPOLY	OCT	00125		#	-X	1 5 9 13
		OCT	00252		#	+X	2 6 10 14
		OCT	00146		#	A	2 5 10 13
		OCT	00231		#	B	1 6 9 14
		OCT	00377		#	ALL	1 2 5 6 9 10 13 14

; ============================================================================
; CLOSEOUT - Interrupt Return Setup
;
; This routine triggers an interrupt return as rapidly as possible to restore
; control to the interrupted job. During DAP execution (JASK), the autopilot
; runs at high priority and can interrupt lower-priority tasks. Once attitude
; control calculations are complete, CLOSEOUT ensures the interrupted job
; resumes execution with minimal delay.
;
; The MAKERUPT instruction forces an immediate interrupt return, causing the
; AGC to restore the interrupted program's context and resume execution.
;
; COMMENT-ONLY READERS: After calculating jet firing commands, the autopilot
; must quickly return control to whatever task it interrupted (navigation,
; displays, etc.). This section handles that handoff.
;
; CODE-ALONG READERS: EDRUPT instruction causes immediate interrupt processing.
; The address ENDJASK is loaded, which will be the interrupt service routine
; that restores all interrupted job registers before actual return.
; ============================================================================
# THE FOLLOWING SETS THE INTERRUPT FLIP-FLOP AS SOON AS POSSIBLE, WHICH PERMITS A RETURN TO THE INTERRUPTED JOB.

CLOSEOUT	CA	ADRRUPT		; Load address of interrupt service routine
		TC	MAKERUPT	; Force interrupt return processing

ADRRUPT		ADRES	ENDJASK		; Address pointer to ENDJASK routine

; ============================================================================
; ENDJASK - End of DAP Job, Register Restoration
;
; This routine completes the Digital Autopilot (DAP) job execution by
; restoring all saved registers from the interrupted job and signaling that
; the JASK (jet selection and control task) is finished. It is the final
; step in every DAP pass through the Q,R-axis autopilot.
;
; The routine performs two critical functions:
; 1. Restores A, L, Q, and Z registers from DAP save areas (DAPARUPT,
;    DAPBQRPT, DAPZRUPT) back to interrupt save areas (ARUPT, BRUPT, ZRUPT)
; 2. Sets DAPZRUPT to NEGMAX (negative flag) signaling DAP completion
;
; After register restoration, control transfers to NOQRSM which performs
; final interrupt return to the interrupted job at its original location.
;
; REGISTER SAVE AREAS:
;   DAPARUPT / ARUPT: Accumulator (A) and Lower Accumulator (L)
;   DAPBQRPT / BRUPT: B register and Q register (return address)
;   DAPZRUPT / ZRUPT: Z register (program counter)
;
; COMMENT-ONLY READERS: This is the final step in the autopilot cycle. After
; commanding the RCS jets, the computer must restore the interrupted program's
; state and resume its original task seamlessly.
;
; CODE-ALONG READERS: The DXCH (double exchange) and XCH (exchange) instructions
; move saved register pairs from DAP temporary storage to interrupt return areas.
; NEGMAX in DAPZRUPT serves as a flag that JASK has completed. NOQRSM performs
; the actual interrupt return using the restored register values.
; ============================================================================
ENDJASK		DXCH	DAPARUPT	; Exchange A,L with DAP saved values
		DXCH	ARUPT		; Move to interrupt return area
		DXCH	DAPBQRPT	; Exchange B,Q with DAP saved values
		XCH	BRUPT		; Move B to interrupt return
		LXCH	Q		; Move Q to interrupt return
		CAF	NEGMAX		# NEGATIVE DAPZRUPT SIGNALS JASK IS OVER.
		DXCH	DAPZRUPT	; Set completion flag and exchange Z
		DXCH	ZRUPT		; Move Z to interrupt return area
		TCF	NOQRSM		; Transfer to final interrupt return
; ============================================================================
; MAKERUPT - Interrupt Return Execution
;
; This routine performs the actual interrupt return operation that transfers
; control back to the interrupted job. It uses the EDRUPT (edit interrupt)
; instruction to trigger immediate interrupt processing.
;
; The EDRUPT instruction is self-referential (EDRUPT MAKERUPT), which causes
; the AGC to immediately begin interrupt return processing. The interrupt
; system will then vector to the address specified in ADRRUPT (which points
; to ENDJASK), which performs register restoration before final return.
;
; This is the mechanism by which the Digital Autopilot, running at high
; priority, returns control to lower-priority jobs that it interrupted
; (such as navigation updates, display refreshes, or guidance computations).
;
; COMMENT-ONLY READERS: This is the actual instruction that tells the computer
; "I'm done with high-priority jet control, return to whatever you were doing."
; The seamless switching between tasks is what allowed the AGC to manage
; multiple critical systems simultaneously during the mission.
;
; CODE-ALONG READERS: EDRUPT is an AGC instruction that forces immediate
; interrupt processing. The self-reference (EDRUPT MAKERUPT) is intentional -
; it sets the interrupt return address to this location, but the interrupt
; system immediately vectors to ENDJASK (via ADRRUPT) for register restoration.
; ============================================================================
# Page 1459
		BLOCK	3
		SETLOC	FFTAG6
		BANK

		COUNT*	$$/DAP

MAKERUPT	EXTEND			; Extended instruction prefix
		EDRUPT	MAKERUPT	; Force interrupt return processing

