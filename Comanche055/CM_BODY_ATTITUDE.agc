# Copyright:    Public domain.
# Filename:     CM_BODY_ATTITUDE.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 883-889
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-12 RSB	Adapted from Colossus249 file of the same
#				name and Comanche 055 page images.
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#       Assemble revision 055 of AGC program Comanche by NASA
#       2021113-051.  April 1, 1969.
#
#       This AGC program shall also be referred to as Colossus 2A
#
#       Prepared by
#                       Massachusetts Institute of Technology
#                       75 Cambridge Parkway
#                       Cambridge, Massachusetts
#
#       under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: CM_BODY_ATTITUDE.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: all-phases
;
; TL;DR: Command Module attitude determination routine converting IMU platform
;        orientation to body frame coordinates. Computes spacecraft attitude
;        relative to reference frames for navigation and control system use,
;        accounting for gimbal angles and coordinate transformations.
;
; COMMENT-ONLY READERS: This program figured out which way the spacecraft was
;        pointing by reading the navigation platform's orientation.
; CODE-ALONG READERS: Study coordinate frame transformation mathematics from
;        inertial platform to body axes, gimbal angle processing.
; ============================================================================

# Page 883
		BANK	35

		SETLOC	BODYATT
		BANK

		COUNT	37/CMBAT

; ============================================================================
; TRANSITION: Command Module Body Attitude Computation
;
; The Command Module's navigation system must constantly determine the
; spacecraft's orientation in space. The Inertial Measurement Unit (IMU)
; provides gimbal angles, but these must be converted to body frame attitude
; for use by guidance and control systems. This routine performs the complex
; coordinate transformations needed to express spacecraft orientation relative
; to both the reference frame and trajectory coordinates.
; ============================================================================

# PDL 12D - 15D SAFE.

# VALUES OF GIMBAL AND BODY ANGLES VALID AT PIP TIME ARE SAVED DURING	READACCS.

		EBANK=	RTINIT		# LET INTERPRETER SET EB

; Entry point CM/POSE computes spacecraft body attitude from IMU gimbal angles.
; This routine determines the orientation of the Command Module in space by
; transforming IMU platform coordinates through the reference coordinate system
; to obtain body frame vectors. The trajectory triad (UXA, UYA, UZA) defines
; the orbital reference frame, while body vectors (UBX, UBY, UBZ) define the
; spacecraft's actual orientation.

CM/POSE		TC	INTPRET		# COME HERE VIA AVEGEXIT.

; ============================================================================
; SECTION: Trajectory Relative Velocity Computation
;
; Compute spacecraft velocity relative to rotating Earth reference frame.
; This accounts for Earth's rotation when determining orbital velocity,
; critical for navigation accuracy during cislunar coast and entry phases.
; ============================================================================

		SETPD	VLOAD
			0
			VN		# KVSCALE = (12800/ .3048) /2VS
		VXSC	PDVL
			-KVSCALE	# KVSCALE = .81491944
			UNITW		# FULL UNIT VECTOR
		VXV	VXSC		# VREL = V - WE*R
			UNITR
			KWE
		VAD	STADR
		STORE	-VREL		# SAVE FOR ENTRY GUIDANCE.	REF COORDS

; Relative velocity -VREL computed as inertial velocity minus Earth rotation
; velocity component (WE × R). Scaled by KVSCALE = 0.81491944 to convert from
; AGC internal units to meters/centisecond. Result stored in reference
; coordinates for use by entry guidance during atmospheric reentry phase.

; ============================================================================
; SECTION: Trajectory Coordinate Triad Construction
;
; Build orthogonal coordinate system aligned with spacecraft trajectory.
; UXA points along velocity vector, UYA perpendicular to orbital plane,
; UZA completes right-handed triad. This trajectory frame provides stable
; reference for attitude computations throughout mission phases.
; ============================================================================

		UNIT	LXA,1
			36D		# ABVAL( -VREL) TO X1
		STORE	UXA/2		# -UVREL			REF COORDS

; UXA/2 is unit vector along negative velocity direction (half-unit scaled).
; Magnitude of relative velocity stored in index register X1 for velocity
; threshold testing during terminal descent phase.

		VXV	VCOMP
			UNITR		# .5 UNIT			REF COORDS
		UNIT	SSP		# THE FOLLOWING IS TO PROVIDE A STABLE
			S1		# UN FOR THE END OF THE TERMINAL PHASE.
SPVQUIT		DEC	.019405		# 1000/ 2 VS
		TIX,1	VLOAD		# IF V-VQUIT POS, BRANCH.
			CM/POSE2	# SAVE UYA IN OLDUYA
			OLDUYA		# OTHERWISE CONTINUE TO USE OLDUYA.

; Velocity threshold test: If velocity exceeds 1000 feet/second, compute new
; UYA perpendicular to orbital plane. Below threshold (terminal phase), use
; previously computed OLDUYA to maintain stable attitude reference when
; velocity becomes very small.

CM/POSE2	STORE	UYA/2		#				REF COORDS

		STORE	OLDUYA		# RESTORE, OR SAVE AS CASE MAY BE.

; UYA/2 is unit vector perpendicular to orbital plane (half-unit scaled),
; computed as unit(UNITR × -UVREL). Provides stable "up" direction reference
; throughout trajectory.

		VXV	VCOMP
			UXA/2		# FINISH OBTAINING TRAJECTORY TRIAD.
		VSL1
		STORE	UZA/2		#				REF COORDS

; UZA/2 completes right-handed orthogonal triad as UYA × UXA. This trajectory
; coordinate system (UXA, UYA, UZA) defines reference frame for expressing
; body attitude angles relative to velocity vector and orbital plane.
# Page 884
; ============================================================================
; SECTION: IMU Gimbal Angle to Body Frame Transformation
;
; The heart of attitude determination: transforming IMU gimbal angles (inner,
; middle, outer) into spacecraft body frame unit vectors. The IMU gimbals are
; measured by Coupling Data Units (CDU) at PIPUP time when accelerometer pulses
; are read. These angles must be converted through trigonometric functions and
; combined to express body orientation in platform coordinates, then rotated
; through REFSMMAT into reference coordinates.
;
; Gimbal angles:
;   AOG = Outer gimbal angle (CDU X)
;   AIG = Inner gimbal angle (CDU Y)  
;   AMG = Middle gimbal angle (CDU Z)
; ============================================================================

		TLOAD			# PICK UP CDUX, CDUY, CDUZ CORRESPONDING
			AOG/PIP		# TO PIPUP TIME IN 2S.C AND SAVE.
CM/TRIO		STODL	24D
			25D		# AIG/PIP

; Load gimbal angles saved during PIPUP time when IMU accelerometer readings
; were taken. These represent spacecraft orientation at precise moment of
; navigation state update. Angles stored in 2's complement format, converted
; to usable form by CDULOGIC routine.

		RTB	PUSH		# TO PDL0
			CDULOGIC
		COS
		STODL	UBX/2		# CI /2
					# AIG/PIP FROM PDL 0
		SIN	DCOMP
		STODL	UBX/2 +4	# -SI /2
			26D		# AMG/PIP

; Compute sine and cosine of inner gimbal angle (AIG). CI = cos(inner),
; SI = sin(inner). These trigonometric values form basis for body frame X-axis
; components. Half-unit scaling (division by 2) maintains precision while
; preventing overflow in subsequent vector operations.

		RTB	PUSH		# TO PDL 0
			CDULOGIC
		SIN	PDDL		# XCH PDL 0.  SAVE SM /2
		COS	PDDL		# CM /2 TO PDL 2
			0		# SM /2
		DCOMP	VXSC
			UBX/2
		VSL1			# NOISE WONT OVFL.
		STODL	UBY/2		# =(-SMCI, NOISE, SMSI)/2
			2		# CM /2 REPLACES NOISE
		STODL	UBY/2 +2	# UBY/2=(-SMCI, CM, SMSI)/2
			24D		# AOG/PIP

; Compute sine and cosine of middle gimbal angle (AMG). SM = sin(middle),
; CM = cos(middle). Combined with inner gimbal trig functions to form body
; frame Y-axis initial components. Vector scaled and stored in UBY/2 with
; placeholder for middle component to be computed next.
		RTB	PUSH		# TO PDL 4
			CDULOGIC
		SIN	PDDL		# XCH PDL 4.  SAVE SO /2
		COS	VXSC		# CO /2
			UBY/2
		STODL	UBY/2		# UBY/2=(-COSMCI, COCM, COSMSI)/4
			4D		# SO /2
		DMP	DCOMP
			UBX/2 +4	# -SI /2
		DAD
			UBY/2		# INCREMENT BY (SOSI /4)
		STODL	UBY/2
					# SO /2 FROM PDL 4
		DMP	DAD
			UBX/2		# CI /2
			UBY/2 +4
		STOVL	UBY/2 +4	# YB/4				PLATFORM COORDS

; Compute sine and cosine of outer gimbal angle (AOG). SO = sin(outer),
; CO = cos(outer). These are combined with previously computed inner and
; middle gimbal trig functions to form complete body frame Y-axis vector.
; Result: YB = (-cos(O)sin(M)cos(I) + sin(O)sin(I), cos(O)cos(M),
;              cos(O)sin(M)sin(I) + sin(O)cos(I))
; This represents spacecraft body Y-axis expressed in stable member
; (platform) coordinates.

				# YB = (-COSMCI + SOSI , COCM , COSMSI + SOCI )

			UBY/2
		VXM	VSL2
			REFSMMAT	# .5 UNIT
		STODL	UBY/2		# YB/2 DONE			REF COORDS

; Transform body Y-axis from platform coordinates to reference coordinates by
; multiplying through Reference Stable Member Matrix (REFSMMAT). This matrix
; rotates from IMU platform frame to basic reference coordinate system,
; accounting for initial IMU alignment. Result scaled to half-unit.
# Page 885
; ============================================================================
; SECTION: Body Frame X-axis and Z-axis Computation
;
; Complete body frame triad by computing X-axis from gimbal angles and
; deriving Z-axis as cross product of X and Y axes. All three body axes
; (UBX, UBY, UBZ) define spacecraft orientation in reference coordinates.
; ============================================================================

					# CM /2 FROM PDL 2
		VXSC	VSL1
			UBX/2
		STODL	UBX/2		# =( CMCI, NOISE, -CMSI)/2
		STADR			# SM /2 FROM PDL 0
		STOVL	UBX/2 +2	# SM /2 REPLACES NOISE
			UBX/2		# XB/2				PLATFORM COORDS

; Body X-axis computed as XB = (cos(M)cos(I), sin(M), -cos(M)sin(I)).
; This represents spacecraft body X-axis (nominally pointing forward through
; crew cabin) expressed in stable member platform coordinates. Middle gimbal
; angle dominates X-axis orientation.

				# XB = ( CMCI , SM , -CMSI )

		VXM	VSL1
			REFSMMAT	# .5 UNIT
		STORE	UBX/2		# XB/2 DONE			REF COORDS

; Transform body X-axis from platform to reference coordinates through
; REFSMMAT. Now UBX/2 expresses spacecraft forward direction in reference
; frame usable by guidance and navigation algorithms.

		VXV	VSL1
			UBY/2
		STOVL	UBZ/2		# ZB/2 DONE			REF COORDS

; Body Z-axis computed as cross product UBX × UBY to complete right-handed
; orthogonal triad. Z-axis points through spacecraft bottom (nominally toward
; Earth during nominal orientation). This approach ensures numerical
; consistency by deriving third axis from first two rather than computing
; independently from gimbal angles.

				# EQUIVALENT TO
				# ZB = ( SOSMCI + COSI , -SOCM , -SOSMSI + COCI)

; If computed independently, body Z-axis would be:
; ZB = (sin(O)sin(M)cos(I) + cos(O)sin(I), -sin(O)cos(M),
;       -sin(O)sin(M)sin(I) + cos(O)cos(I))
; Cross product method used above is computationally more efficient and
; guarantees orthogonality of body frame triad.

; ============================================================================
; SECTION: Attitude Angle Computation from Body Frame Vectors
;
; With body frame triad (UBX, UBY, UBZ) and trajectory triad (UXA, UYA, UZA)
; both defined in reference coordinates, compute attitude angles describing
; spacecraft orientation relative to trajectory:
;   ROLL  - rotation about velocity vector (X-axis of trajectory frame)
;   BETA  - angle between velocity and body Y-axis (yaw/sideslip)
;   ALFA  - angle between velocity and body X-axis (pitch/angle of attack)
;   GAMA  - flight path angle (angle between velocity and local horizontal)
;
; These angles drive entry guidance, controlling lift vector and bank angle
; for atmospheric reentry trajectory control toward desired landing target.
; ============================================================================

			UXA/2		# -UVREL/2 = -UVA/2
		VXV	UNIT		# GET UNIT(-UVREL*UBY)/2 = UL/2
			UBY/2		# YB/2
		PUSH	DOT		# UL/2 TO PDL 0,5
			UZA/2		# UNA/2
		STOVL	COSTH		# COS(ROLL)/4
			0		# UL/2

; Compute ROLL angle: rotation of spacecraft body about velocity vector.
; UL = unit vector perpendicular to velocity in plane of body Y-axis.
; Roll determined by dot products with trajectory normal (UNA/UZA).
; Critical for entry guidance to orient lift vector correctly.

		DOT
			UYA/2
		STCALL	SINTH		# -SIN(ROLL)/4
			ARCTRIG
		STOVL	6D		# -(ROLL/180) /2
			UBY/2
		DOT	SL1		# -UVA.UBY = -SIN(BETA)
			UXA/2		# -UVREL/2
		ARCSIN
		STOVL	7D		# -(BETA/180) /2
			UBX/2		# XB/2

; Compute BETA angle: yaw or sideslip angle between velocity and body Y-axis.
; Dot product of velocity with body Y gives sine of beta directly.
; Beta indicates lateral (sideways) orientation of spacecraft relative to
; flight path. Should be near zero for normal entry trajectory.

		DOT			# UL.UBX = -SIN(ALFA)
			0		# UL/2
		STOVL	SINTH		# -SIN(ALFA)/4
		DOT			# UL/2 FROM PDL 0
			UBZ/2
		STCALL	COSTH		# COS(ALFA)/2
			ARCTRIG
		STOVL	8D		# -(ALFA/180) /2
			UNITR		# UR/2				REF COORDS

; Compute ALFA angle: pitch or angle of attack between velocity and body
; X-axis. Sine and cosine computed from dot products of perpendicular
; vector UL with body X and Z axes. Alpha controls lift magnitude during
; entry - spacecraft flies slightly pitched up to generate lift for range
; control and heating management.

		DOT	SL1
# Page 886
			UZA/2		# MORE ACCURATE AT LARGE ARG.
		ARCCOS
		STORE	10D		# (-GAMA/180)/2

; Compute GAMA angle: flight path angle between velocity vector and local
; horizontal plane. Dot product of position unit vector (up from planet
; center) with trajectory normal gives gamma. Negative gamma means descending
; trajectory into atmosphere. Entry guidance adjusts bank angle to maintain
; desired gamma profile, balancing between excessive deceleration (steep) and
; insufficient deceleration (shallow skip-out).

		TLOAD	EXIT		# ANGLES IN MPAC IN THE ORDER
					# -( (ROLL, BETA, ALFA) /180)/2
			6D		# THESE VALUES CORRECT AT PIPUP TIME.

; All attitude angles now computed and stored, representing spacecraft
; orientation at PIPUP time when IMU data was sampled. Angles scaled as
; negative fractions of 180 degrees divided by 2 for use by guidance.
; Exit interpretive mode to transfer angles to entry guidance routines.

# Page 887
# BASIC SUBROUTINE TO UPDATE ATTITUDE ANGLES

; ============================================================================
; SUBROUTINE: CM/ATUP - Command Module Attitude Update
;
; PURPOSE: Updates stored attitude angles (ROLL/180, BETA/180, ALFA/180) by
; correcting Euler angles for gimbal changes that occurred between PIPUP time
; and current time. This ensures attitude data used by entry guidance remains
; accurate as the IMU platform orientation drifts relative to spacecraft body.
;
; ENTRY POINTS:
;   CM/ATUP - Primary entry, sets EBANK to AOG
;   CMTR1   - Alternative entry for GAMA angle processing
;   CMTR2   - Common return path for BETA angle completion
;
; INPUTS: 
;   6D, 7D, 8D - Newly computed Euler angles (ROLL, BETA, ALFA) from CM/POSE
;   ROLL/PIP, BETA/PIP, ALFA/PIP - Gimbal angles saved at PIPUP time
;   ROLL/180, BETA/180, ALFA/180 - Current stored body angles
;   GAMA - Previous flight path angle
;
; OUTPUTS:
;   ROLL/180, BETA/180, ALFA/180 - Updated body attitude angles
;   GAMA - Updated flight path angle
;   GAMDOT - Rate of change of flight path angle (if significant)
;   VMAGI - Velocity magnitude for display
;
; MISSION CONTEXT: These angles are critical during entry phase when the
; Command Module must maintain precise attitude for lift vector control to
; guide spacecraft to target landing site in Pacific Ocean. Armstrong, Aldrin,
; and Collins relied on these computations during Apollo 11's return to Earth
; on July 24, 1969.
; ============================================================================

		EBANK=	AOG

CM/ATUP		CA	EBAOG
		TS	EBANK
CMTR1		INDEX	FIXLOC
		CS	10D		# (GAMA/180)/2
		XCH	GAMA
		TS	L

; Store newly computed flight path angle GAMA. This angle represents the
; inclination of the velocity vector relative to the local horizontal plane.
; During entry, GAMA transitions from shallow (~0°) at entry interface to
; steep (~40°) at peak deceleration, then back to shallow for final descent.

		INHINT
				# MUST REMAIN INHINTED UNTIL UPDATE OF BODY
				# ANGLES, SO THAT GAMDIFSW IS VALID FIRST PASS
				# INDICATOR.

; Disable interrupts to ensure atomic update of attitude angles. The GAMDIFSW
; flag (BIT11 of CM/FLAGS) acts as first-pass indicator - initially zero to
; skip GAMDOT computation until we have two GAMA values to form a difference.

		CS	CM/FLAGS
		MASK	BIT11		# GAMDIFSW=94D BIT11	INITLY=0
		EXTEND			# DONT CALC GAMA DOT UNTIL HAVE FORMD
					# ONE DIFFERENCE.
		BZF	DOGAMDOT	# IS OK, GO ON.
		ADS	CM/FLAGS	# KNOW BIT IS 0
		TC	NOGAMDOT	# SET GAMDOT = 0

; DOGAMDOT - Compute flight path angle rate of change
; This routine calculates GAMDOT by differencing current and previous GAMA
; values. During entry, flight path angle transitions from shallow at entry
; interface to steep during peak deceleration, back to shallow for landing.

DOGAMDOT	CS	L
		AD	GAMA		# DEL GAMA/360= T GAMDOT/360

; Calculate change in flight path angle: GAMDIF = GAMA(current) - GAMA(previous)
; Scaling: (DEL GAMA/360) represents fractional revolution change.

		EXTEND
		MP	TCDU		# TCDU = .1 SEC, T = 2 SEC.
		TS	GAMDOT		# GAMA DOT TCDU / 180

; Multiply angle difference by time constant TCDU (0.1 seconds) to obtain
; rate of change. Result scaled as (GAMA DOT × TCDU / 180) in degrees per
; update interval. Typical entry values: -2°/sec during initial descent.

		EXTEND			# IGNORE GAMDOT IF LEQ .5 DEG/SEC
		BZMF	+2
		COM
		AD	FIVE
		EXTEND
		BZMF	+3		# SET GAMDOT=+0 AS TAG IF TOO SMALL.

; Threshold test: If |GAMDOT| ≤ 0.5°/sec, set GAMDOT to zero to avoid
; noise amplification. Small angle rates indicate near-constant altitude
; flight or measurement noise rather than significant trajectory changes.

NOGAMDOT	CA	ZERO		# COME HERE INHINTED.
		TS	GAMDOT

; ============================================================================
; ANGLE UPDATE SECTION
;
; The following section updates stored body attitude angles (ROLL, BETA, ALFA)
; by correcting for gimbal angle changes that occurred between PIPUP time (when
; IMU data was last sampled) and current time. This ensures attitude angles
; accurately reflect current spacecraft orientation relative to reference frame.
;
; Update equation for each angle:
;   ANGLE/180 = ANGLE_EULER/180 + ANGLE(NOW) - ANGLE(PIPUP)
;
; Where:
;   ANGLE_EULER = Newly computed Euler angle from CM/POSE coordinate transform
;   ANGLE(NOW)  = Current gimbal angle from CDU registers
;   ANGLE(PIPUP)= Gimbal angle saved at last PIPUP time
;
; OVERFLOW HANDLING: Angles can overflow ±180° boundaries. The CORANGOV
; subroutine corrects for this, ensuring angles remain in valid range.
; ============================================================================

					# FOR NOW LEAVE IN 2S.C
					# UPDATE ANGLES BY CORRECTING EUILER ANG
					# FOR ACCRUED INCREMENT SINCE PIPUP
					# R = R EUIL + R(NOW) -R(PIPUP)

; ROLL ANGLE UPDATE

		CS	MPAC		# GET (R EUL/180) /2
		DOUBLE			# POSSIBLE OVERFLOW

; Load roll Euler angle from MPAC (computed in CM/POSE). Double to scale
; from half-revolutions to full scale. Overflow may occur at ±180° boundary.

		TC	CORANGOV	# CORRECT FOR OVFL IF ANY
		EXTEND
		SU	ROLL/PIP	# GET INCR SINCE PIPUP

; Subtract gimbal angle saved at PIPUP time to isolate the gimbal change that
; occurred during the interval. This increment must be added to stored angle.

		AD	ROLL/180	# ONLY SINGLE OVFL POSSIBLE.
		TC	CORANGOV	# CORRECT FOR OVFL IF ANY

; Add increment to previous stored roll angle. Result may overflow ±180°
; boundary (single overflow maximum), requiring correction by CORANGOV.

# Page 888
		TS	TEMPROLL

; Store updated roll angle temporarily. Final storage occurs after BETA and
; ALFA updates complete, ensuring atomic angle set update.

; ALFA ANGLE UPDATE (Angle of Attack)

		CS	MPAC +2		# GET (ALFA EUL/180) /2
		DOUBLE			# SAME AS FOR ROLL.  NEEDED FOR EXT ATM DAP

; Load angle of attack (ALFA) Euler angle from MPAC+2. ALFA represents the
; angle between velocity vector and spacecraft longitudinal axis. Critical
; during entry: proper ALFA maintains heat shield forward, protecting crew
; from reentry temperatures exceeding 5000°F.

		TC	CORANGOV	# CORRECT FOR OVFL IF ANY
		EXTEND
		SU	ALFA/PIP
		AD	ALFA/180
		TC	CORANGOV	# CORRECT FOR OVFL IF ANY
		TS	TEMPALFA

; Apply same update logic as ROLL: Add gimbal increment since PIPUP to Euler
; angle. Store temporarily in TEMPALFA pending final atomic update.

; BETA ANGLE UPDATE (Sideslip Angle)

		CS	MPAC +1		# GET (BETA EUL/180) /2
CMTR2		DOUBLE

; CMTR2 - Common return entry point for routines updating only BETA angle.
; Load sideslip angle (BETA) Euler angle from MPAC+1. BETA represents lateral
; deviation of velocity vector from spacecraft symmetry plane.

		EXTEND
		SU	BETA/PIP
		AD	BETA/180
		XCH	TEMPBETA	# OVFL NOT EXPECTED.

; Update BETA using gimbal increment. Overflow not expected for BETA as
; sideslip remains small during normal entry (spacecraft flies nearly
; zero-sideslip to minimize asymmetric heating and aerodynamic loads).

; ============================================================================
; RESTART PROTECTION AND FINAL ANGLE STORAGE
;
; The following section sets up restart protection for angle updates, then
; performs atomic storage of all three updated angles (ROLL, BETA, ALFA).
; This ensures that if a restart occurs during angle update, the system
; recovers to a consistent state with all angles updated together.
; ============================================================================

		CA	EBANK3
		TS	EBANK

; Switch to EBANK3 to access phase change registers for restart protection.

		EBANK=	PHSNAME5
		EXTEND
		DCA	REPOSADR	# THIS ASSUMES THAT THE		TC  PHASCHNG
		DXCH	PHSNAME5	# IS NOT CHANGED IN		OCT 10035
					# SERVICER.

; Set up restart protection pointing to REDOPOSE. If restart occurs after
; this point, system will resume at REDOPOSE rather than beginning of CM/ATUP.
; This assumes SERVICER routine does not alter phase change table during
; execution (assumption documented in original NASA code comments).

		CA	EBAOG
		TS	EBANK

; Restore EBANK to AOG (Attitude and Other Guidance variables) for final
; angle storage operations.

		EBANK=	AOG
REDOPOSE	EXTEND			# RE-STARTS COME HERE

; REDOPOSE - Restart recovery entry point. If system restart occurred during
; angle update, execution resumes here with temporary angles ready for storage.

		DCA	TEMPROLL
		DXCH	ROLL/180

; Store updated ROLL angle atomically. Double-precision transfer (DCA/DXCH)
; ensures both ROLL/180 and ALFA/180 (stored consecutively) are updated
; together, maintaining angle set consistency.

		CA	TEMPBETA
		TS	BETA/180

; Store updated BETA angle. All three body attitude angles now reflect current
; spacecraft orientation accounting for gimbal changes since last PIPUP time.

		RELINT

; Re-enable interrupts. Angle update is complete and consistent. System can
; now respond to time-critical events (IMU pulses, displays, navigation).

		TC	INTPRET		# CANT TC DANZIG AFTER PHASCHNG.
CM/POSE3	VLOAD	ABVAL		# RETURN FROM CM/ATUP.	(RESTART)
			VN		# 2(-7) M/CS
		STORE	VMAGI		# FOR DISPLAY ON CALL.

; Compute velocity magnitude for crew display. Entry velocity ranges from
; 36,000 ft/sec at entry interface (400,000 ft altitude) to 300 ft/sec at
; parachute deployment (24,000 ft altitude). Crew monitors VMAGI on DSKY
; to track deceleration progress during Apollo 11's return on July 24, 1969.

		GOTO
			POSEXIT		# ENDEXIT, STARTENT, OR SCALEPOP.

; Exit CM/ATUP via POSEXIT. Destination varies by mission phase:
;   ENDEXIT   - Normal exit after entry guidance cycle
;   STARTENT  - Entry initialization complete, begin guidance
;   SCALEPOP  - Scale variables and return to calling program

; ============================================================================
; SUBROUTINE: CORANGOV - Correct Angle Overflow
;
; PURPOSE: Corrects Euler angle overflow when angle computation crosses the
; ±180° boundary. Adds or subtracts 360° (one full revolution) to bring angle
; back into standard ±180° range.
;
; ENTRY: TC CORANGOV with angle in A register
; EXIT:  Returns via Q with corrected angle in A register, original saved in L
;
; ALGORITHM: If angle overflow detected (sign bit indicates out of range),
; adds appropriate LIMITS constant (+360° or -360°) to correct back into range.
;
; TIMING: Costs 2 machine cycles (MCT) when used. See ANGOVCOR for alternate
; implementation with different timing characteristics.
; ============================================================================

CORANGOV	TS	L
		TC	Q

; Normal case: No overflow detected. Save angle to L register and return
; immediately via Q (return address register).

		INDEX	A
# Page 889
		CA	LIMITS
		ADS	L
		TC	Q		# COSTS 2 MCT TO USE.  SEE ANGOVCOR.

; Overflow case: Use A register as index into LIMITS table to fetch correction
; constant (+360° or -360° depending on overflow direction). Add correction to
; saved angle in L register, then return via Q. This path executes only when
; angle crossed ±180° boundary during arithmetic operations.

; ============================================================================
; CONSTANTS FOR CM_BODY_ATTITUDE COMPUTATIONS
; ============================================================================

-KVSCALE	2DEC	-.81491944	# -12800/(2 VS .3048)

; Velocity scaling constant: -KVSCALE = -0.81491944
; Derivation: -12800 / (2 × VS × 0.3048)
; Where VS = velocity scaling factor, 0.3048 = meters to feet conversion
; Used in CM/POSE to scale velocity vectors from meters/centisecond to
; appropriate units for entry guidance calculations.

TCDU		DEC	.1		# TCDU = .1 SEC.

; Time constant for CDU (Coupling Data Unit) updates: TCDU = 0.1 seconds
; Represents the nominal update interval for gimbal angle measurements.
; Used in GAMDOT computation to convert angle difference to rate of change.

		EBANK=	AOG
REPOSADR	2CADR	REDOPOSE

; Two-complement address (2CADR) of REDOPOSE entry point for restart protection.
; If system restart occurs during angle update, this address directs restart
; logic to resume at REDOPOSE rather than beginning of CM/ATUP subroutine.
