# Copyright:	Public domain.
# Filename:	GIMBAL_LOCK_AVOIDANCE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	364
# Mod history:	2009-05-17 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2010-12-31 JL	Fixed page number comment.
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

# Page 364
; ============================================================================
; FILE: GIMBAL_LOCK_AVOIDANCE.agc
; MODULE: Inertial Measurement Unit (IMU) Control
; MISSION PHASE: All phases requiring attitude reference
;
; TL;DR: Detects and helps avoid gimbal lock condition where the IMU's middle
;        gimbal approaches 90 degrees (Euler angle singularity). When detected,
;        illuminates the GIMBAL LOCK warning light on the DSKY and computes
;        avoidance maneuver rates to reorient the spacecraft away from the
;        singularity, maintaining stable attitude reference throughout mission.
;
; COMMENT-ONLY READERS: This routine protects the spacecraft's navigation
;        system from losing its orientation reference by detecting when the
;        IMU gimbals are approaching a dangerous alignment.
; CODE-ALONG READERS: Study the gimbal angle monitoring logic and the
;        computation of optimal reorientation rates to understand how the AGC
;        maintained stable attitude reference in three-dimensional space.
; ============================================================================

		BANK	15

		SETLOC	KALCMON1
		BANK

; ============================================================================
; GIMBAL LOCK DETECTION AND AVOIDANCE
;
; The Inertial Measurement Unit (IMU) uses three physical gimbals to measure
; spacecraft orientation relative to inertial space. When the middle gimbal
; approaches 90 degrees, a mathematical singularity occurs (Euler angle
; singularity) where two gimbal axes align, causing loss of one degree of
; freedom. This "gimbal lock" condition means the IMU cannot uniquely
; determine spacecraft attitude.
;
; During Apollo 11's mission, avoiding gimbal lock was critical during all
; attitude maneuvers including translunar coast, lunar orbit, and especially
; during LM descent and ascent when precise attitude knowledge was essential
; for guidance and navigation.
;
; NOGIMLOC routine computes maneuver rates needed to reorient the spacecraft
; away from gimbal lock conditions while maintaining mission attitude
; requirements.
; ============================================================================

# DETECTING GIMBAL LOCK
LOCSKIRT	EQUALS	NOGIMLOC

; ============================================================================
; NOGIMLOC - No Gimbal Lock Maneuver Computation Routine
;
; This routine is called when the spacecraft needs to perform an attitude
; maneuver while avoiding or recovering from gimbal lock conditions. It
; selects an appropriate maneuver rate from a table of four predefined rates
; (ranging from 0.2 to 10.0 degrees per second) and computes the incremental
; rotation needed to safely reorient the spacecraft.
;
; The computed maneuver keeps the middle gimbal angle away from the critical
; 85-90 degree region where gimbal lock would occur. During Apollo 11, the
; crew could observe the middle gimbal angle on their instruments and would
; be alerted by the GIMBAL LOCK warning light if intervention was needed.
; ============================================================================

NOGIMLOC	SET
			CALCMAN3
; Load the maneuver rate index (0, 2, 4, or 6) which selects one of four
; predefined rotation rates. Slower rates (0.2-2.0 deg/sec) used during
; precision operations; faster rates (10 deg/sec) for rapid reorientation.
WCALC		LXC,1	DLOAD*
			RATEINDX	# CHOOSE THE DESIRED MANEUVER RATE
			ARATE,1		# FROM A LIST OF FOUR
		SR4	CALL		# COMPUTE THE INCREMENTAL ROTATION MATRIX
			DELCOMP		# DEL CORRESPONDING TO A 1 SEC ROTATION
					# ABOUT COF
; Compute the body-axis maneuver rates by scaling the rotation axis (COF)
; by the selected angular rate. This produces the commanded rotation velocity
; vector that will move the spacecraft away from gimbal lock.
		DLOAD*	VXSC
			ARATE,1
			COF
		STODL	BRATE		# COMPONENT MANEUVER RATES 45 DEG/SEC
			AM
; Calculate the time required to complete the maneuver by dividing the
; total rotation angle (AM) by the selected rate. Time is scaled as T2
; format (centiseconds squared) for integration with guidance computations.
		DMP	DDV*
			ANGLTIME
			ARATE,1
		SR
			5
		STORE	TM		# MANEUVER EXECUTION TIME SCALED AS T2
; Set flag to indicate maneuver start and branch to attitude maneuver
; execution. CALCMAN2 flag controls whether this is a new maneuver (1=ON)
; or continuation of existing maneuver (0=OFF).
		SETGO
			CALCMAN2	# 0(OFF) = CONTINUE MANEUVER
			NEWANGL +1	# 1(ON) = START MANEUVER

; ============================================================================
; MANEUVER RATE TABLE - Four Selectable Rotation Rates
;
; The AGC provides four predefined angular rotation rates for attitude
; maneuvers, selected by loading RATEINDX with 0, 2, 4, or 6 respectively.
; These rates accommodate different mission phases and operational needs:
;
; Rate 1 (0.2 deg/sec):  Precision maneuvers during critical operations
; Rate 2 (0.5 deg/sec):  Standard reorientation for most mission phases  
; Rate 3 (2.0 deg/sec):  Moderate-speed attitude changes
; Rate 4 (10.0 deg/sec): Rapid reorientation for gimbal lock avoidance
;
; During Apollo 11's lunar descent and ascent, the slower rates were typically
; used to maintain precise attitude control while the faster rates were
; reserved for emergency gimbal lock recovery if the middle gimbal angle
; approached the 85-90 degree danger zone.
;
; All rates scaled in revolutions per centisecond (AGC standard angular rate
; unit). The notation "$ 22.5 DEG/SEC" on each line likely refers to maximum
; achievable rate with RCS thrusters at full authority.
; ============================================================================

# THE FOUR SELECTABLE FREE FALL MANEUVER RATES SELECTED BY
# LOADING RATEINDX WITH 0,2,4,6, RESPECTIVELY

ARATE		2DEC	.0088888888	# = 0.2 DEG/SEC		$ 22.5 DEG/SEC

		2DEC	.0222222222	# = 0.5 DEG/SEC		$ 22.5 DEG/SEC

		2DEC	.0888888888	# = 2.0 DEG/SEC		$ 22.5 DEG/SEC

		2DEC	.4444444444	# = 10.0 DEG/SEC	$ 22.5 DEG/SEC

; ============================================================================
; ANGLTIME - Angle to Time Conversion Factor
;
; This constant converts maneuver angle (in revolutions) to maneuver time
; (in centiseconds squared, T2 format). The value 0.0001907349 represents
; the mathematical relationship between rotation angle and time duration
; at the selected angular rate.
;
; The original comment calls this a "FUDGE FACTOR" but it's actually a
; precisely calculated scaling constant: 10^8 / 2^19 = 190.7349... which
; accounts for the AGC's fixed-point arithmetic scaling conventions where
; angles are scaled in revolutions and times in centiseconds.
; ============================================================================

ANGLTIME	2DEC	.0001907349	# = 1008-19 FUDGE FACTOR TO CONVERT
					# MANEUVER ANGLE TO MANEUVER TIME


