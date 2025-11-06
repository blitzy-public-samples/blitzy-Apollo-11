# Copyright:	Public domain.
# Filename:	IMU_COMPENSATION_PACKAGE.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	297-306
# Mod history:	2009-05-08 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images/
#		2009-05-21 RSB	In IRIGZ, PRIO17 corrected to PRIO21.
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

# Page 297
; ============================================================================
; FILE: IMU_COMPENSATION_PACKAGE.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Inertial Measurement Unit compensation algorithms correcting gyro drift
;        and accelerometer bias errors. Applies calibration parameters accounting
;        for temperature effects and systematic errors, maintaining navigation
;        accuracy throughout Apollo 11 mission duration.
;
; COMMENT-ONLY READERS: This program corrected small errors in the navigation
;        platform's gyroscopes and accelerometers to maintain accuracy.
; CODE-ALONG READERS: Study gyro drift compensation, accelerometer bias correction,
;        temperature effect modeling, systematic error compensation algorithms.
; ============================================================================

		BANK	7
		SETLOC	IMUCOMP
		BANK
		EBANK=	NBDX

		COUNT	06/ICOMP

; ============================================================================
; PIPA COMPENSATION AND BIAS REMOVAL
;
; The Pulsed Integrating Pendulous Accelerometers (PIPAs) measure spacecraft
; acceleration but include systematic errors requiring compensation. This
; section applies scale factor corrections and removes bias drift accumulated
; since the last compensation cycle. Without these corrections, navigation
; errors would accumulate over hours of mission duration.
;
; The compensation process:
; 1. Apply scale factor error (SFE) corrections to PIPA measurements
; 2. Remove bias drift proportional to elapsed time
; 3. Process all three axes (X, Y, Z) sequentially
; ============================================================================

1/PIPA		CAF	LGCOMP		# SAVE EBANK OF CALLING PROGRAM
		XCH	EBANK
		TS	MODE

; Check if gyro compensation is enabled. If GCOMPSW is negative, compensation
; is disabled and the routine bypasses all corrections.
		CCS	GCOMPSW		# BYPASS IF GCOMPSW NEGATIVE
		TCF	+3
		TCF	+2
		TCF	IRIG1		# RETURN

; Disable interrupts to ensure atomic compensation of all three PIPA delta-V
; measurements. This prevents telemetry downlink from reading partially
; compensated values which would appear as erroneous acceleration spikes.
		INHINT			#  ASSURE COMPLETE COMPENSATION OF DELV'S
					# FOR DOWNLINK.

; Process all three PIPA axes in reverse order: Z, Y, X (counter = 4, 2, 0)
1/PIPA1		CAF	FOUR		# PIPAZ, PIPAY, PIPAX
		TS	BUF +2

; ============================================================================
; SCALE FACTOR ERROR COMPENSATION
;
; Each PIPA has a small scale factor error (SFE) calibrated on the ground.
; The scale factor correction PIPASCF is stored in parts-per-million (PPM)
; scaled by 2^-9. Multiply the measured PIPA pulses by the scale factor to
; compute the correction, then add it to the original measurement.
;
; Computation: DELVX_corrected = DELVX + (DELVX * PIPASCF)
; ============================================================================

		INDEX	BUF +2
		CA	PIPASCF		# (P.P.M.) X 2(-9)
		EXTEND
		INDEX	BUF +2
		MP	DELVX		# (PP) X 2(+14) NOW (PIPA PULSES) X 2(+5)
		TS	Q		# SAVE MAJOR PART

; Handle fractional part separately to maintain full precision
		CA	L		# MINOR PART
		EXTEND
		MP	BIT6		# SCALE 2(+9)	SHIFT RIGHT 9
		INDEX 	BUF +2
		TS	DELVX +1	# FRACTIONAL PIPA PULSES SCALED 2(+14)

; Add major part of scale factor correction to PIPA measurement
		CA	Q		# MAJOR PART
		EXTEND
		MP	BIT6		# SCALE 2(+9)	SHIFT RIGHT 9
		INDEX	BUF +2
		DAS	DELVX		# (PIPAI) + (PIPAI)(SFE)

; ============================================================================
; PIPA BIAS REMOVAL
;
; PIPAs exhibit a constant bias drift rate causing accumulated error over time.
; The bias rate (PIPABIAS in pipa pulses per centisecond) is calibrated for
; each axis. Multiply bias by elapsed time (1/PIPADT) since last compensation
; and subtract from the corrected PIPA reading.
;
; During Apollo 11's long coast phases, this correction prevented navigation
; errors from accumulating. Bias drift, though small, would cause position
; errors of hundreds of feet over hours without compensation.
;
; Computation: DELVX_final = DELVX_corrected - (PIPABIAS * DELTAT)
; ============================================================================

		INDEX	BUF +2
		CS	PIPABIAS	# (PIPA PULSES)/(CS) X 2(-8)			*
		EXTEND
		MP	1/PIPADT	# (CS) X 2(+8) NOW (PIPA PULSES) X 2(+0)	*
		EXTEND
		MP	BIT1		# SCALE 2(+14) SHIFT RIGHT 14			*
		INDEX	BUF +2
		DAS	DELVX		# (PIPAI) + (PIPAI)(SFE) - (BIAS)(DELTAT)

; Loop through all three axes: first iteration PIPAZ, then PIPAY, then PIPAX
		CCS	BUF +2		# PIPAZ, PIPAY, PIPAX
# Page 298
		AD	NEG1
		TCF	1/PIPA1 +1
		NOOP			# LESS THAN ZERO IMPOSSIBLE.
		RELINT
# Page 299

; ============================================================================
; GYRO COMPENSATION (IRIG) - CROSS-COUPLING CORRECTIONS
;
; The three gyroscopes (Inertial Rate Integrating Gyros - IRIGs) measure
; spacecraft rotation rates. However, PIPA accelerations create spurious gyro
; outputs through "acceleration-dependent drift" (ADIA). Each gyro responds
; slightly to accelerations measured by the PIPAs.
;
; This section removes the cross-coupling effects by computing correction
; torques based on PIPA readings and ADIA calibration coefficients. The
; corrections maintain the inertial platform's orientation accuracy during
; engine burns and maneuvers when accelerations are high.
;
; Without these corrections, a lunar orbit insertion burn would introduce
; several degrees of platform misalignment, causing navigation errors.
; ============================================================================

IRIGCOMP	TS	GCOMPSW		# INDICATE COMMANDS 2 PULSES OR LESS.
		TS	BUF		# INDEX COUNTER - IRIGX, IRIGY, IRIGZ.

; ============================================================================
; X-AXIS GYRO COMPENSATION (IRIGX)
;
; Compute correction torques for the X-axis gyro based on cross-coupling from
; PIPA measurements. The X-axis gyro responds to both X and Y accelerations
; through calibrated ADIA coefficients.
;
; During Apollo 11's translunar injection burn, these corrections prevented
; the platform from drifting due to the Service Propulsion System's thrust.
; ============================================================================

IRIGX		EXTEND
		DCS	DELVX		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CA	ADIAX		# (GYRO PULSES)/(PIPA PULSE) X 2(-3)		*
		TC	GCOMPSUB	# -(ADIAX)(PIPAX)	(GYRO PULSES) X 2(+14)

; Add Y-axis PIPA contribution to X-axis gyro compensation
		EXTEND			#
		DCS	DELVY		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC		#
		CS	ADSRAX		# (GYRO PULSES)/(PIPA PULSE) X 2(-3)	*
		TC	GCOMPSUB	# +(ADSRAX)(PIPAY)	(GYRO PULSES) X 2(+14)

#		EXTEND			# ***
#		DCS	DELVY		# ***	(PIPA PULSES) X 2(+14)
#		DXCH	MPAC		# ***
#		CA	ADOAX		# ***	(GYRO PULSES)/(PIPA PULSE) X 2(-3)	*
#		TC	GCOMPSUB	# ***	-(ADOAX)(PIPAZ)		(GYRO PULSES) X 2(+14)

; Apply gyro drift bias compensation for X-axis. Each gyro exhibits systematic
; drift over time due to temperature effects and mechanical imperfections. The
; NBDX coefficient represents the X-axis gyro's drift rate in gyro pulses per
; centisecond, accumulated over the time interval DELTAT.

		CS	NBDX		# (GYRO PULSES)/(CS) X 2(-5)
		TC	DRIFTSUB	# -(NBDX)(DELTAT)	(GYRO PULSES) X 2(+14)

; ============================================================================
; Y-AXIS GYRO COMPENSATION (IRIGY)
;
; Compute correction torques for the Y-axis (middle) gyro based on Y and Z
; PIPA cross-coupling. The Y-axis gyro's ADIA coefficients relate how
; accelerations along the Y and Z axes induce false rotation signals.
; The middle gimbal's orientation makes it sensitive to lateral accelerations
; during trajectory correction burns.
; ============================================================================

IRIGY		EXTEND
		DCS	DELVY		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CA	ADIAY		# (GYRO PULSES)/(PIPA PULSE) X 2(-3)		*
		TC	GCOMPSUB	# -(ADIAY)(PIPAY)	(GYRO PULSES) X 2(+14)

; Add Z-axis PIPA contribution to Y-axis gyro compensation. The ADSRAY
; coefficient captures the Y-axis gyro's sensitivity to Z-axis accelerations.

		EXTEND
		DCS	DELVZ		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CS	ADSRAY		# (GYRO PULSES)/(PIPA PULSE) X 2(-3)		*
		TC	GCOMPSUB	# +(ADSRAY)(PIPAZ)	(GYRO PULSES) X 2(+14)

#		EXTEND			# ***
#		DCS	DELVX		# ***	(PIPA PULSES) X 2(+14)
#		DXCH	MPAC		# ***
#		CA	ADOAY		# ***	(GYRO PULSES)/(PIPA PULSE) X 2(-3)	*
#		TC	GCOMPSUB	# ***	-(ADOAY)(PIPAZ)	(GYRO PULSES) X 2(+14)

; Apply gyro drift bias compensation for Y-axis. The middle gimbal gyro
; experiences different thermal conditions than the outer and inner gimbals.

		CS	NBDY		# (GYRO PULSES)/(CS) X 2(-5)
		TC	DRIFTSUB	# -(NBDY)(DELTAT)	(GYRO PULSES) X 2(+14)

; ============================================================================
; Z-AXIS GYRO COMPENSATION (IRIGZ)
;
; Compute correction torques for the Z-axis (inner) gyro based on Y and Z
; PIPA cross-coupling. The inner gimbal gyro measures rotation about the
; spacecraft's roll axis. During powered flight maneuvers, particularly SPS
; burns and course corrections, accelerations induce false rotation signals
; that must be compensated to maintain attitude reference accuracy.
; ============================================================================

IRIGZ		EXTEND
		DCS	DELVY		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CA	ADSRAZ		# (GYRO PULSES)/(PIPA PULSE) X 2(-3)		*
# Page 300
		TC	GCOMPSUB	# -(ADSRAZ)(PIPAY)	(GYRO PULSES) X 2(+14)

; Add Z-axis PIPA contribution to Z-axis gyro compensation. The ADIAZ
; coefficient represents the Z-axis gyro's direct sensitivity to Z-axis
; accelerations, the primary compensation term for the inner gimbal.

		EXTEND
		DCS	DELVZ		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CA	ADIAZ		# (GYRO PULSES)/(PIPA PULSE) X 2(-3)		*
		TC	GCOMPSUB	# -(ADIAZ)(PIPAZ)	(GYRO PULSES) X 2(+14)

#		EXTEND			# ***
#		DCS	DELVX		# ***	(PIPA PULSE) X 2(+14)
#		DXCH	MPAC		# ***
#		CS	ADOAZ		# ***	(GYRO PULSES)/(PIPA PULSE) X 2(-3)	*
#		TC	GCOMPSUB	# ***	+(ADOAZ)(PIPAX)	(GYRO PULSES) X 2(+14)

; Apply gyro drift bias compensation for Z-axis. The inner gimbal gyro's drift
; rate is affected by bearing friction and thermal gradients across the gimbal.

		CA	NBDZ		# (GYRO PULSES)/(CS) X 2(-5)
		TC	DRIFTSUB	# +(NBDZ)(DELTAT)	(GYRO PULSES) X 2(+14)

# Page 301
		CCS	GCOMPSW		# ARE GYRO COMMANDS GREATER THAN 2 PULSES
		TCF	+2		# YES
		TCF	IRIG1		# NO

		CAF	PRIO21		# HIGHER THAN SERVICER-LESS THAN PRELAUNCH
		TC	NOVAC
		EBANK=	NBDX
		2CADR	1/GYRO

		RELINT
IRIG1		CA	MODE		# SET EBANK FOR RETURN
		TS	EBANK
		TCF	SWRETURN

; ============================================================================
; TRANSITION: From main compensation logic to utility subroutines
;
; The following subroutines perform the mathematical operations required for
; IMU compensation. GCOMPSUB applies acceleration-induced gyro drift terms
; (ADIA coefficients). DRIFTSUB applies fixed gyro drift bias corrections
; (NBD terms). These routines are called repeatedly during compensation cycles
; to maintain navigation platform accuracy throughout the mission.
; ============================================================================

; GCOMPSUB - Gyro Compensation Subroutine
; Computes cross-coupling correction terms from accelerometer-induced gyro drift.
; When the spacecraft accelerates, PIPA accelerometers sense the acceleration,
; but this same acceleration creates torques on the gyroscopes through mass
; unbalance and compliance effects. The ADIA (Acceleration-Dependent Input Axis)
; coefficients quantify these cross-coupling effects. This routine multiplies
; ADIA coefficients by PIPA readings to generate corrective gyro torque commands.
;
; Input: A register = ADIA coefficient (gyro pulses/pipa pulse) scaled 2^-3
;        MPAC = PIPA pulse count scaled 2^+14
;        BUF = axis index (0=Z, 2=Y, 4=X)
; Output: GCOMP updated with cross-coupling correction (gyro pulses) scaled 2^+14

GCOMPSUB	XCH	MPAC		# ADIA OR ADSRA COEFFICIENT ARRIVES IN A
		EXTEND			# C(MPAC) = (PIPA PULSES) X 2(+14)
		MP	MPAC		# (GYRO PULSES)/(PIPA PULSE) X 2(-3)		*
		DXCH	VBUF		# NOW = (GYRO PULSES) X 2(+11)			*

		CA	MPAC +1		# MINOR PART OF PIPA PULSES
		EXTEND
		MP	MPAC		# ADIA OR ADSRA
		TS	L
		CAF	ZERO
		DAS	VBUF		# NOW = (GYRO PULSES) X 2(+11)			*

		CA	VBUF		# PARTIAL RESULT - MAJOR
		EXTEND
		MP	BIT12		# SCALE 2(+3)	SHIFT RIGHT 3			*
		INDEX	BUF		# RESULT = (GYRO PULSES) X 2(+14)
		DAS	GCOMP		# HI(ADIA)(PIPAI) OR HI(ADSRA)(PIPAI)

		CA	VBUF +1		# PARTIAL RESULT - MINOR
		EXTEND
		MP	BIT12		# SCALE 2(+3)	SHIFT RIGHT 3			*
		TS	L
		CAF	ZERO
		INDEX	BUF		# RESULT = (GYRO PULSES) X 2(+14)
		DAS	GCOMP		# (ADIA)(PIPAI)  OR  (ADSRA)(PIPAI)

		TC	Q

# Page 302
; DRIFTSUB - Gyro Drift Bias Correction Subroutine
; Applies fixed gyro drift bias corrections to maintain platform alignment.
; Even in the absence of spacecraft motion, the IMU gyroscopes drift due to
; torque disturbances (mass unbalance, magnetic fields, residual gas damping).
; The NBD (Null Bias Drift) coefficients quantify these systematic errors,
; measured during pre-flight calibration. This routine multiplies NBD by elapsed
; time to generate corrective gyro torque commands offsetting the drift.
;
; Input: A register = NBD coefficient (gyro pulses/cs) scaled 2^-5
;        1/PIPADT = elapsed time since last compensation (cs) scaled 2^+8
;        BUF = axis index (0=Z, 2=Y, 4=X)
; Output: GCOMP updated with drift bias correction (gyro pulses) scaled 2^+14

DRIFTSUB	EXTEND
		QXCH	BUF +1

		EXTEND			# C(A) = NBD	(GYRO PULSES)/(CS) X 2(-5)
		MP	1/PIPADT	# (CS) X 2(+8)	NO (GYRO PULSES) X 2(+3)
		LXCH	MPAC +1		# SAVE FOR FRACTIONAL COMPENSATION
		EXTEND
		MP	BIT4		# SCALE 2(+11)	SHIFT RIGHT 11
		INDEX	BUF
		DAS	GCOMP		# HI(NBD)(DELTAT)	(GYRO PULSES) X 2(+14)

		CA	MPAC +1		# NOW MINOR PART
		EXTEND
		MP	BIT4		# SCALE 2(+11)		SHIFT RIGHT 11
		TS	L
		CAF	ZERO
		INDEX	BUF		# ADD IN FRACTIONAL COMPENSATION
		DAS	GCOMP		# (NBD)(DELTAT)		(GYRO PULSES) X 2(+14)

; DRFTSUB2 - Drift Compensation Threshold Check
; After computing gyro compensation commands, verifies if the correction is
; large enough to warrant immediate application. Commands less than 1 gyro pulse
; accumulate for later application to avoid excessive torquing activity.

DRFTSUB2	CAF	TWO		# PIPAX, PIPAY, PIPAZ
		AD	BUF
		XCH	BUF
		INDEX	A
		CCS	GCOMP		# ARE GYRO COMMANDS 1 PULSE OR GREATER
		TCF	+2		# YES
		TC	BUF +1		# NO

		MASK	NEGONE
		CCS	A		# ARE GYRO COMMANDS GREATER THAN 2 PULSES
		TS	GCOMPSW		# YES - SET GCOMPSW POSITIVE
		TC	BUF +1		# NO

# Page 303
; 1/GYRO - Gyro Torque Command Application
; Scales computed compensation commands for gyro torque pulse interface and
; applies them to the IMU. The GCOMP values (gyro pulses scaled 2^+14) must
; be rescaled to the format expected by IMUPULSE routine (2^+7 scaling).
; After rescaling, calls IMUPULSE to generate actual torque pulses driving
; the gyroscope torque motors, then waits for pulse train completion via IMUSTALL.
;
; This is where computed corrections are physically applied to the IMU platform,
; maintaining its alignment throughout the mission. During Apollo 11, these
; compensation pulses fired continuously to counter drift and keep the stable
; member accurately oriented in inertial space.

1/GYRO		CAF	FOUR		# PIPAZ, PIPAY, PIPAX
		TS	BUF

		INDEX	BUF		# SCALE GYRO COMMANDS FOR IMUPULSE
		CA	GCOMP +1	# FRACTIONAL PULSES
		EXTEND
		MP	BIT8		# SHIFT RIGHT 7
		INDEX	BUF
		TS	GCOMP +1	# FRACTIONAL PULSES SCALED

		CAF	ZERO		# SET GCOMP = 0 FOR DAS INSTRUCTION
		INDEX	BUF
		XCH	GCOMP		# GYRO PULSES
		EXTEND
		MP	BIT8		# SHIFT RIGHT 7
		INDEX	BUF
		DAS	GCOMP		# ADD THESE TO FRACTIONAL PULSES ABOVE

		CCS	BUF		# PIPAZ, PIPAY, PIPAX
		AD	NEG1
		TCF	1/GYRO +1
LGCOMP		ECADR	GCOMP		# LESS THAN ZERO IMPOSSIBLE

		CAF	LGCOMP
		TC	BANKCALL
		CADR	IMUPULSE	# CALL GYRO TORQUING ROUTINE
		TC	BANKCALL
		CADR	IMUSTALL	# WAIT FOR PULSES TO GET OUT
		TCF	ENDOFJOB	# TEMPORARY

; GCOMP1 - Gyro Compensation Rescaling (Alternate Path)
; Rescales gyro compensation commands when only minor adjustments needed.
; Unlike 1/GYRO which handles full double-precision scaling, GCOMP1 processes
; only the fractional part, shifting it left 7 bits. Used when compensation
; values are small enough that major part remains zero.
;
; After rescaling, displays compensation status via verb 06 noun 30 before
; ending the compensation job.

GCOMP1		CAF	FOUR		# PIPAZ, PIPAY, PIPAX
		TS	BUF

		INDEX	BUF		# RESCALE
		CA	GCOMP +1
		EXTEND
		MP	BIT8		# SHIFT MINOR PART LEFT 7 - MAJOR PART = 0
		INDEX	BUF
		LXCH	GCOMP +1	# BITS 8-14 OF MINOR PART WERE = 0

		CCS	BUF		# PIPAZ, PIPAY, PIPAX
		AD	NEG1
		TCF	GCOMP1 +1
V06N30S		VN	0630
		TCF	ENDOFJOB

# Page 304
; ============================================================================
; NBDONLY - Null Bias Drift Compensation Only
;
; This routine applies gyro drift bias corrections without PIPA compensation,
; used during coast phases when accelerometer data is less critical. Between
; major maneuvers (translunar coast, lunar orbit), the IMU platform requires
; periodic drift correction to maintain alignment accuracy. NBDONLY computes
; elapsed time since last compensation and applies NBD corrections to all
; three gyro axes, preventing platform drift accumulation.
;
; During Apollo 11's 3-day translunar coast, this routine executed periodically
; to maintain platform alignment within acceptable limits for subsequent
; navigation updates and trajectory correction maneuvers.
; ============================================================================

NBDONLY		CCS	GCOMPSW		# BYPASS IF GCOMPSW NEGATIVE
		TCF	+3
		TCF	+2
		TCF	ENDOFJOB

		INHINT
		CCS	FLAGWRD2	# PREREAD T3RUPT MAY COINCIDE
		TCF	ENDOFJOB
		TCF	ENDOFJOB
		TCF	+1

		CA	TIME1		# (CS) X 2(+14)
		XCH	1/PIPADT	# PREVIOUS TIME
		RELINT
		COM
		AD	1/PIPADT
NBD2		CCS	A		# CALCULATE ELAPSED TIME
		AD	ONE		# NO TIME1 OVERFLOW
		TCF	NBD3		# RESTORE TIME DIFFERENCE AND JUMP
		TCF	+2		# TIME1 OVERFLOW
		TCF	ENDOFJOB	# IF ELAPSED TIME = 0 (DIFFERENCE = -0)

		COM			# CALCULATE ABSOLUTE DIFFERENCE
		AD	POSMAX

NBD3		EXTEND			# C(A) = DELTAT		(CS) X 2(+14)
		MP	BIT10		# SHIFT RIGHT 5
		DXCH	VBUF
		EXTEND
		DCA	VBUF
		DXCH	MPAC		# DELTAT NOW SCALED (CS) X 2(+19)

		CAF	ZERO
		TS	GCOMPSW		# INDICATE COMMANDS 2 PULSES OR LESS
		TS	BUF		# PIPAX, PIPAY, PIPAZ

		CS	NBDX		# (GYRO PULSES)/(CS) X 2(-5)
		TC	FBIASSUB	# -(NBOX)(DELTAT) 	(GYRO PULSES) X 2(+14)

		EXTEND
		DCS	VBUF
		DXCH	MPAC		# DELTAT SCALED (CS) X 2(+19)
		CA	NBDY		# (GYRO PULSES)/(CS) X 2(-5)
		TC	FBIASSUB	# -(NBDY)(DELTAT)	(GYRO PULSES) X 2(+14)

		EXTEND
		DCS	VBUF
		DXCH	MPAC		# DELTAT SCALED (CS) X 2(+19)
		CS	NBDZ		# (GYRO PULSES)/(CS) X 2(-5)
		TC	FBIASSUB	# +(NBDZ)(DELTAT)	(GYRO PULSES) X 2(+14)
# Page 305
		CCS	GCOMPSW		# ARE GYRO COMMANDS GREATER THAN 2 PULSES
		TCF	1/GYRO		# YES
		TCF	ENDOFJOB	# NO

# Page 306
; FBIASSUB - Fixed Bias Compensation Subroutine
; Performs high-precision multiplication of NBD drift coefficients by elapsed
; time for drift-only compensation mode. Similar to DRIFTSUB but uses different
; time scaling (CS x 2^+19 instead of CS x 2^+8) for longer coast periods where
; higher precision is needed to accumulate drift corrections accurately.
;
; Input: A register = NBD coefficient (gyro pulses/cs) scaled 2^-5
;        MPAC = elapsed time DELTAT (cs) scaled 2^+19
;        BUF = axis index (0=Z, 2=Y, 4=X)
; Output: GCOMP updated with drift correction, then magnitude check via DRFTSUB2

FBIASSUB	XCH	Q
		TS	BUF +1

		CA	Q		# NBD SCALED (GYRO PULSES)/(CS) X 2(-5)
		EXTEND
		MP	MPAC		# DELTAT SCALED (CS) X 2(+19)
		INDEX	BUF
		DAS	GCOMP		# HI(NBD)(DELTAT)	(GYRO PULSES) X 2(+14)

		CA	Q		# NO FRACTIONAL PART
		EXTEND
		MP	MPAC +1
		TS	L
		CAF	ZERO
		INDEX	BUF
		DAS	GCOMP		# (NBD)(DELTAT)		(GYRO PULSES) X 2(+14)

		TCF	DRFTSUB2	# CHECK MAGNITUDE OF COMPENSATION

; LASTBIAS - Final Bias Compensation Before IMU Mode Change
; Called before IMU mode switching to apply one last gyro drift compensation
; using accumulated time since last PIPA read. Ensures all drift corrections
; are current before platform realignment or gyrocompassing. Uses PIPUSE to
; service any pending PIPA data, then calculates remaining time interval and
; applies NBD corrections via NBD2 path.
;
; Used during IMU realignments between mission phases (e.g., before translunar
; injection alignment, before lunar orbit insertion, before entry interface).
;
; Input: GCOMPSW = compensation enable flag
;        PIPTIME1 +1 = time of last PIPA reading
; Output: Final drift compensation applied to GCOMP registers

LASTBIAS	TC	BANKCALL
		CADR	PIPUSE

		CCS	GCOMPSW		# BYPASS IF GCOMPSW NEGATIVE
		TCF	+3
		TCF	+2
		TCF	ENDOFJOB

		CAF	PRIO31		# 2 SECONDS SCALED (CS) X 2(+8)
		XCH	1/PIPADT
		COM
		AD	PIPTIME1 +1	# TIME AT PIPA1 =0
		TCF	NBD2

