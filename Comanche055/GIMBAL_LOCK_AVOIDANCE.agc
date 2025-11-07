# Copyright:	Public domain.
# Filename:	GIMBAL_LOCK_AVOIDANCE.agc
# Purpose:	Part of the source code for Comanche, build 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 412-413
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Mod history:	05/07/09 OH	Transcription Batch 1 Assignment
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
; FILE: GIMBAL_LOCK_AVOIDANCE.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Gimbal lock detection and avoidance implementing Euler angle singularity
;        monitoring. Detects when middle gimbal approaches 90° causing loss of
;        IMU control authority, triggers crew warning (GIMBAL LOCK light), and
;        recommends avoidance maneuvers preventing platform tumble throughout
;        Apollo 11 operations.
;
; COMMENT-ONLY READERS: This program detected and warned crew when the navigation
;        platform could lose its ability to track orientation accurately.
; CODE-ALONG READERS: Study Euler angle singularity mathematics, gimbal lock
;        detection criteria, warning generation, avoidance maneuver computation.
; ============================================================================

# Page 412
		BANK	15
		SETLOC	KALCMON1
		BANK

		EBANK=	BCDU
; ============================================================================
; GIMBAL LOCK DETECTION AND AVOIDANCE SYSTEM
;
; The Apollo Guidance Computer's Inertial Measurement Unit (IMU) uses a three-
; gimbal platform to maintain orientation reference. When the middle gimbal
; approaches 90 degrees, the platform enters "gimbal lock" - a condition where
; two gimbals become aligned and the system loses one degree of freedom. This
; creates a mathematical singularity in the Euler angle representation.
;
; If gimbal lock occurs, the IMU can no longer accurately track spacecraft
; attitude changes in all three axes. The spacecraft could "tumble" relative
; to the stable platform, requiring time-consuming IMU realignment and
; potentially jeopardizing mission operations.
;
; This routine continuously monitors gimbal angles and computes avoidance
; maneuvers when the middle gimbal threatens to approach the critical 90°
; position. When detected, the GIMBAL LOCK warning light illuminates on the
; DSKY, alerting the crew to maneuver the spacecraft to a safe attitude.
; ============================================================================

# DETECTING GIMBAL LOCK
; ============================================================================
; WCALC - GIMBAL LOCK AVOIDANCE MANEUVER COMPUTATION
;
; This routine calculates the required spacecraft rotation to move away from
; gimbal lock conditions. The computation determines both the direction and
; rate of the avoidance maneuver based on current gimbal angles.
;
; COMMENT-ONLY READERS: When the navigation platform's middle gimbal approached
;        90 degrees, this routine computed a corrective maneuver to prevent
;        loss of orientation reference. The computer would then command the
;        spacecraft to rotate to a safer attitude configuration.
;
; CODE-ALONG READERS: The routine uses interpretive language to compute an
;        incremental rotation matrix representing the avoidance maneuver.
;        Angular rates are selected from a table based on urgency, then
;        transformed through the spacecraft-to-control coordinate system.
; ============================================================================

LOCSKIRT	EQUALS	WCALC
WCALC		LXC,1	DLOAD*
			RATEINDX
			ARATE,1
; Load index for angular rate selection. RATEINDX determines maneuver urgency:
; higher index = faster avoidance rate needed. ARATE table contains four
; rate options: 0.05, 0.2, 0.5, and 2.0 degrees per second.

		SR4	CALL		# COMPUTE THE INCREMENTAL ROTATION MATRIX
			DELCOMP		# DEL CORRESPONDING TO A 1 SEC ROTATION
					# ABOUT COF
; Compute rotation matrix increment for 1-second rotation about the computed
; direction (COF = direction of free rotation). SR4 scales by 2^-4 for proper
; fixed-point representation in subsequent matrix operations.

		DLOAD*	VXSC
			ARATE,1
			COF
; Scale the direction vector (COF) by the selected angular rate to obtain
; the rotation velocity vector. This represents how fast and in what direction
; the spacecraft should rotate to avoid gimbal lock.

		MXV
			QUADROT
; Transform the rotation vector from spacecraft body axes to control system
; axes using QUADROT matrix. The -7.25 degree X-axis rotation accounts for
; mounting offset between spacecraft reference frame and control system frame.

		STODL	BRATE
			AM
; Store computed body rate vector (BRATE) for autopilot use. Load maneuver
; angle magnitude (AM) to calculate time required for complete maneuver.

		DMP	DDV*
			ANGLTIME
			ARATE,1
; Compute maneuver time: TM = (AM * ANGLTIME) / ARATE
; ANGLTIME constant converts angle to time units. Result represents seconds
; needed to complete avoidance maneuver at selected rate.

		SR
			5
		STOVL	TM
			BRATE
; Scale time by 2^-5 for proper units, store as TM (maneuver time).
; Reload body rate vector for bias computation.

		VXSC
			BIASCALE
; Apply bias scaling to prevent autopilot overshoot. BIASCALE factor of
; (450/180)(1/0.6)(1/16384) converts rate to attitude error bias units,
; creating slight offset that dampens oscillation as target attitude nears.

		STORE	BIASTEMP	# ATTITUDE ERROR BIAS TO PREVENT OVERSHOOT
					# IN SYSTEM
; Store computed bias in BIASTEMP for digital autopilot (DAP) to use during
; maneuver execution. Bias creates artificial attitude error that prevents
; spacecraft from overshooting desired gimbal-safe orientation.

		SETGO			# STATE SWITCH CALCMAN2 (43D)
			CALCMAN2	# 0(OFF) = BYPASS STARTING PROCEDURE
			NEWANGL +1	# 1(ON) = START MANEUVER
; Set CALCMAN2 flag to ON state, signaling that avoidance maneuver calculation
; is complete and execution should begin. Transfer control to NEWANGL+1 to
; initiate spacecraft rotation sequence that will move gimbals away from lock.


; ============================================================================
; GIMBAL LOCK AVOIDANCE CONSTANTS
;
; These constants define the angular rates, coordinate transformations, and
; scaling factors used to compute and execute gimbal lock avoidance maneuvers.
; ============================================================================

; ARATE - Angular Rate Table
; Four maneuver rate options selected based on gimbal lock proximity:
; Slower rates for gradual avoidance, faster rates for urgent situations.
; Rates expressed in revolutions per centisecond (AGC time units).

ARATE		2DEC	.0022222222	# = .05 DEG/SEC
; Option 1: Slowest rate (0.05°/sec) for gentle, fuel-efficient avoidance
; when middle gimbal is approaching but not critically near 90°.

		2DEC	.0088888889	# = .2 DEG/SEC
; Option 2: Low rate (0.2°/sec) for gradual correction with minimal
; propellant consumption and smooth crew ride quality.

		2DEC	.0222222222	# = .5 DEG/SEC
; Option 3: Medium rate (0.5°/sec) for moderate urgency situations
; where gimbal angles require timely but not emergency correction.

		2DEC	.0888888889	# = 2 DEG/SEC                $22.5 DEG/SEC
; Option 4: Maximum rate (2.0°/sec) for critical gimbal lock threat
; requiring rapid avoidance maneuver. Used when middle gimbal very
; close to 90° singularity position.

ANGLTIME	2DEC	.000190735	# = 100B - 19
; Maneuver time conversion constant: ANGLTIME = 2^(-19) * 100 (decimal)
; Converts maneuver angle to maneuver time when divided by angular rate.
; Units: (angle in revolutions) * ANGLTIME / (rate in rev/centisec) = time
					# MANEUVER ANGLE TO MANEUVER TIME

; QUADROT - Spacecraft-to-Control Axes Rotation Matrix
; 3x3 rotation matrix transforming vectors from spacecraft body axes to
; control system axes. Accounts for -7.25 degree offset around X-axis
; between spacecraft mechanical mounting frame and guidance control frame.
; Matrix scaled by 0.1 for AGC fixed-point arithmetic precision.

QUADROT		2DEC	.1		# ROTATION MATRIX FROM S/C AXES TO CONTROL
; Matrix element [1,1]: X-component mapping, scaled rotation
; Full matrix represents: Rx(-7.25°) scaled by 0.1

# Page 413
		2DEC	0		# AXES	(X ROT = -7.25 DEG)
; Matrix element [1,2]: Y-component of X-axis mapping (zero for X-rotation)

		2DEC	0
; Matrix element [1,3]: Z-component of X-axis mapping (zero for X-rotation)

		2DEC	0
; Matrix element [2,1]: X-component of Y-axis mapping (zero for X-rotation)

		2DEC	.099200		# =(.1)COS7.25
; Matrix element [2,2]: Y-component of Y-axis mapping
; = 0.1 * cos(7.25°) = 0.1 * 0.99200 = 0.099200
; Y-axis rotates in YZ plane, cosine term represents Y-projection

		2DEC	-.012620	# =-(.1)SIN7.25
; Matrix element [2,3]: Z-component of Y-axis mapping  
; = -0.1 * sin(7.25°) = -0.1 * 0.12620 = -0.012620
; Negative sine term for -7.25° rotation direction

		2DEC	0
; Matrix element [3,1]: X-component of Z-axis mapping (zero for X-rotation)

		2DEC	.012620		# (.1)SIN7.25
; Matrix element [3,2]: Y-component of Z-axis mapping
; = 0.1 * sin(7.25°) = 0.1 * 0.12620 = 0.012620
; Positive sine term from rotation geometry

		2DEC	.099200		# (.1)COS7.25
; Matrix element [3,3]: Z-component of Z-axis mapping
; = 0.1 * cos(7.25°) = 0.1 * 0.99200 = 0.099200
; Z-axis rotates in YZ plane, cosine term represents Z-projection

BIASCALE	2DEC	.0002543132	# = (450/180)(1/0.6)(1/16384)
; Attitude error bias scaling factor for autopilot overshoot prevention.
; Computation: (450/180) converts units, (1/0.6) accounts for DAP gain,
; (1/16384) scales to AGC fixed-point units (2^-14).
; Resulting bias creates slight attitude offset that dampens oscillation
; as spacecraft approaches gimbal-safe orientation, preventing overshoot
; that could reverse into gimbal lock region.
