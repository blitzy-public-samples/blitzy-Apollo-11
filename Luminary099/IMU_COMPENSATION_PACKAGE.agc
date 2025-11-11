# Copyright:	Public domain.
# Filename:	IMU_COMPENSATION_PACKAGE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	326-337
# Mod history:	2009-05-16 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-07 RSB	Corrected a typo.
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
; FILE: IMU_COMPENSATION_PACKAGE.agc
; MODULE: Navigation and Sensors
; MISSION PHASE: descent/landing/ascent/rendezvous
;
; TL;DR: Compensates for systematic errors in the Inertial Measurement Unit
;        (IMU) sensors to maintain accurate navigation during all mission
;        phases. Corrects accelerometer (PIPA) bias and scale factors, gyro
;        drift rates, and acceleration-induced cross-coupling effects between
;        axes using pre-calibrated compensation parameters.
;
; COMMENT-ONLY READERS: This code ensures the spacecraft knows its precise
;        position and velocity by correcting sensor drift and bias errors.
; CODE-ALONG READERS: Study compensation algorithms including fixed-point
;        arithmetic scaling, cross-axis coupling matrices, and drift models.
; ============================================================================

# Page 326
		BANK	7
		SETLOC	IMUCOMP
		BANK

		EBANK=	NBDX

		COUNT*	$$/ICOMP

; ============================================================================
; TRANSITION: From IMU raw measurements to compensated sensor data
;
; The IMU contains three PIPAs (Pulsed Integrating Pendulous Accelerometers)
; measuring acceleration along X, Y, and Z body axes, and three gyroscopes
; measuring angular rates. These sensors have systematic errors including
; bias (constant offset), scale factor errors (sensitivity variations), and
; drift rates. This compensation package applies pre-calibrated corrections
; to produce accurate velocity and attitude measurements for navigation.
; ============================================================================

; PIPA COMPENSATION ROUTINE
; Entry: 1/PIPA - Compensates accelerometer measurements for all three axes.
; The PIPAs measure accelerations by counting pulses proportional to the
; integral of acceleration. This routine corrects for scale factor errors
; (SFE) and bias drift over the measurement interval (DELTAT).
;
1/PIPA		CAF	LGCOMP		# SAVE EBANK OF CALLING PROGRAM
		XCH	EBANK
		TS	MODE

		CCS	GCOMPSW		# BYPASS IF GCOMPSW NEGATIVE
		TCF	+3
		TCF	+2
		TCF	IRIG1		# RETURN

; Process all three PIPA axes: Z, Y, X (indexed by BUF+2 = 4, 2, 0)
; For each axis: Apply scale factor error correction, then subtract bias drift.
;
1/PIPA1		CAF	FOUR		# PIPAZ, PIPAY, PIPAX
		TS	BUF +2

; SCALE FACTOR ERROR CORRECTION
; Scale factor error (SFE) represents PIPA sensitivity deviation from nominal.
; Formula: Corrected = Measured × (1 + SFE)
; Implementation: DELVX = DELVX + (DELVX × PIPASCF)
;
		INDEX	BUF +2
		CA	PIPASCF		# (P.P.M.) X 2(-9)
		EXTEND
		INDEX	BUF +2
		MP	DELVX		# (PP) X 2(+14) NOW (PIPA PULSES) X 2(+5)
		TS	Q		# SAVE MAJOR PART

		CA	L		# MINOR PART
		EXTEND
		MP	BIT6		# SCALE 2(+9)   SHIFT RIGHT 9
		INDEX 	BUF +2
		TS	DELVX +1	# FRACTIONAL PIPA PULSES SCALED 2(+14)

		CA	Q		# MAJOR PART
		EXTEND
		MP	BIT6		# SCALE 2(+9)	SHIFT RIGHT 9
		INDEX	BUF  +2
		DAS	DELVX		# (PIPAI) + (PIPAI)(SFE)

; BIAS CORRECTION
; PIPA bias is a constant acceleration offset that accumulates as spurious
; velocity over time. Subtract bias × time interval from the measurement.
; Formula: Corrected = Measured - (BIAS × DELTAT)
;
		INDEX	BUF +2
		CS	PIPABIAS	# (PIPA PULSES)/(CS) X 2(-5)		 *
		EXTEND
		MP	1/PIPADT	# (CS) X 2(+8)  NOW (PIPA PULSES) X 2(+3)*
		EXTEND
		MP	BIT4		# SCALE 2(+11)  SHIFT RIGHT 11		 *
		INDEX	BUF +2
		DAS	DELVX		# (PIPAI) + (PIPAI)(SFE) - (BIAS)(DELTAT)

		CCS	BUF 	+2	# PIPAZ, PIPAY, PIPAX
		AD	NEG1
		TCF	1/PIPA1	+1
# Page 327
		NOOP			# LESS THAN ZERO IMPOSSIBLE

# Page 328
; ============================================================================
; TRANSITION: From PIPA compensation to gyro drift compensation
;
; Having corrected the accelerometer measurements, the AGC now compensates
; the three gyroscope channels. Gyros measure the spacecraft's rotation rates
; but suffer from constant drift (NBD - Non-Bias Drift) and acceleration-
; sensitive drift (cross-coupling between acceleration and gyro output).
; During lunar descent, these corrections are critical for maintaining precise
; attitude knowledge as the LM descends under engine thrust.
; ============================================================================

; GYRO COMPENSATION ROUTINE
; Entry: IRIGCOMP - Compensates all three gyro axes (X, Y, Z) for drift
; and acceleration-induced errors.
;
IRIGCOMP	TS	GCOMPSW		# INDICATE COMMANDS 2 PULSES OR LESS.
		TS	BUF		# INDEX COUNTER .  IRIGX, IRIGY, IRIGZ.

; X-AXIS GYRO COMPENSATION
; First apply acceleration cross-coupling corrections, then subtract drift.
;
		TC	IRIGX		# COMPENSATE ACCELERATION TERMS

		CS	NBDX		# (GYRO PULSES)/(CS) X 2(-5)
		TC	DRIFTSUB	# -(NBOX)(DELTAT)   (GYRO PULSES) X 2(+14)

; Y-AXIS GYRO COMPENSATION
;
		TC	IRIGY		# COMPENSATE ACCELERATION TERMS

		CS	NBDY		# (GYRO PULSES)/(CS) X 2(-5)
		TC	DRIFTSUB	# -(NBDY)(DELTAT)   (GYRO PULSES) X 2(+14)

; Z-AXIS GYRO COMPENSATION
;
		TC	IRIGZ		# COMPENSATE ACCELERATION TERMS

		CA	NBDZ		# (GYRO PULSES)/(CS) X 2(-5)
		TC	DRIFTSUB	# +(NBDZ)(DELTAT)   (GYRO PULSES) X 2(+14)

		CCS	GCOMPSW		# ARE GYRO COMMANDS GREATER THAN 2 PULSES
		TCF	+2		# YES  SEND OUT GYRO TORQUING COMMANDS.
		TCF	IRIG1		# NO  RETURN

		CA	PRIO21		# PRIO GREATER THAN SERVICER
		TC	NOVAC		# SEND OUT GYRO TORQUING COMMANDS.
		EBANK=	NBDX
		2CADR	1/GYRO

		RELINT
IRIG1		CA	MODE		# RESTORE CALLERS EBANK
		TS	EBANK
		TCF	SWRETURN

# Page 329
IRIGX		EXTEND
		QXCH	MPAC +2	        # SAVE Q
		EXTEND
		DCS	DELVX		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CA	ADIAX		# (GYRO PULSES)/(PIPA PULSE) X 2(-6)     *
		TC	GCOMPSUB	# -(ADIAX)(PIPAX)   (GYRO PULSES) X 2(+14)

		EXTEND			#
		DCS	DELVY		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC		#
		CS	ADSRAX		# (GYRO PULSES)/(PIPA PULSE) X 2(-6)	 *
		TC	GCOMPSUB	# +(ADSRAX)(PIPAY)  (GYRO PULSES) X 2(+14)

#		EXTEND		 # ***
#		DCS	DELVZ	 # ***    (PIPA PULSES) X 2(+14)
#		DXCH	MPAC	 # ***
#		CA	ADOAX	 # ***    (GYRO PULSES)/(PIPA PULSE) X 2(-6)	 *
#		TC	GCOMPSUB # ***    -(ADOAX)(PIPAZ)   (GYRO PULSES) X 2(+14)

		TC	MPAC  +2

IRIGY		EXTEND
		QXCH	MPAC  +2	# SAVE Q
		EXTEND
		DCS	DELVY		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CA	ADIAY		# (GYRO PULSES)/(PIPA PULSE) X 2(-6)     *
		TC	GCOMPSUB	# -(ADIAY)(PIPAY)   (GYRO PULSES) X 2(+14)

		EXTEND
		DCS	DELVZ		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CS	ADSRAY		# (GYRO PULSES)/(PIPA PULSE) X 2(-6)	 *
		TC	GCOMPSUB	# +(ADSRAY)(PIPAZ)  (GYRO PULSES) X 2(+14)

#		EXTEND		 # ***
#		DCS	DELVX	 # ***    (PIPA PULSES) X 2(+14)
#		DXCH	MPAC	 # ***
#		CA	ADOAY	 # ***    (GYRO PULSES)/(PIPA PULSE) X 2(-6)	 *
#		TC	GCOMPSUB # ***    -(ADOAY)(PIPAX)   (GYRO PULSES) X 2(+14)

		TC	MPAC +2

IRIGZ		EXTEND
		QXCH	MPAC +2		# SAVE Q
		EXTEND
		DCS	DELVY		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CA	ADSRAZ		# (GYRO PULSES)/(PIPA PULSE) X 2(-6)	 *
# Page 330
		TC	GCOMPSUB	# -(ADSRAZ)(PIPAY)  (GYRO PULSES) X 2(+14)

		EXTEND
		DCS	DELVZ		# (PIPA PULSES) X 2(+14)
		DXCH	MPAC
		CA	ADIAZ		# (GYRO PULSES)/(PIPA PULSE) X 2(-6)	 *
		TC	GCOMPSUB	# -(ADIAZ)(PIPAZ)   (GYRO PULSES) X 2(+14)

#		EXTEND		 # ***
#		DCS	DELVX	 # ***	  (PIPA PULSE) X 2(+14)
#		DXCH	MPAC	 # ***
#		CS	ADOAZ	 # ***	  (GYRO PULSES)/(PIPA PULSE) X 2(-6)     *
#		TC	GCOMPSUB # ***	  +(ADOAZ)(PIPAX)   (GYRO PULSES) X 2(+14)

		TC	MPAC +2

# Page 331
GCOMPSUB	XCH	MPAC		# ADIA OR ADSRA COEFFICIENT ARRIVES IN A
		EXTEND			# C(MPAC) = (PIPA PULSES) X 2(+14)
		MP	MPAC		# (GYRO PULSES)/(PIPA PULSE) X 2(-6)	 *
		DXCH	VBUF		# NOW = (GYRO PULSES) X 2(+8)		 *

		CA	MPAC +1		# MINOR PART OF PIPA PULSES
		EXTEND
		MP	MPAC		# ADIA OR ADSRA
		TS	L
		CAF	ZERO
		DAS	VBUF		# NOW = (GYRO PULSES) X 2(+8)		 *

		CA	VBUF		# PARTIAL RESULT - MAJOR
		EXTEND
		MP	BIT9		# SCALE 2(+6)	   SHIFT RIGHT           *
		INDEX	BUF		# RESULT = (GYRO PULSES) X 2(+14)
		DAS	GCOMP		# HI(ADIA)(PIPAI)  OR  HI(ADSRA)(PIPAI)

		CA	VBUF +1		# PARTIAL RESULT - MINOR
		EXTEND
		MP	BIT9		# SCALE 2(+6)	SHIFT RIGHT 6		 *
		TS	L
		CAF	ZERO
		INDEX	BUF		# RESULT = (GYRO PULSES) X 2(+14)
		DAS	GCOMP		# (ADIA)(PIPAI) OR (ADSRA)(PIPAI)

		TC	Q

# Page 332
; ============================================================================
; DRIFT SUBTRACTION SUBROUTINE
;
; Compensates for gyro drift bias over the measurement interval. Mechanical
; gyroscopes exhibit constant angular drift due to bearing friction, mass
; imbalances, and temperature gradients. Pre-flight calibration determines
; drift rates (NBDX, NBDY, NBDZ) stored as (gyro pulses/centisecond) × 2^-5.
; This routine multiplies drift rate by time interval (1/PIPADT) to compute
; total drift compensation, maintaining platform orientation accuracy during
; lunar descent, landing, and ascent phases when navigation precision is
; critical for safe spacecraft control.
;
; Entry: A register = drift rate (gyro pulses/CS) × 2^-5
;        Q register saved to BUF +1 for return
; Computation: Drift = NBD × DELTAT (with double-precision for accuracy)
; ============================================================================
;
DRIFTSUB	EXTEND
		QXCH	BUF +1

		EXTEND			# C(A) = NBD	(GYRO PULSES)/(CS) X 2(-5)
		MP	1/PIPADT	# (CS) X 2(+8)	 NOW (GYRO PULSES) X 2(+3)
		LXCH	MPAC +1	        # SAVE FOR FRACTIONAL COMPENSATION
		EXTEND
		MP	BIT4		# SCALE 2(+11)	   SHIFT RIGHT 11
		INDEX	BUF
		DAS	GCOMP		# HI(NBD)(DELTAT)   (GYRO PULSES) X 2(+14)

		CA	MPAC +1	        # NOW MINOR PART
		EXTEND
		MP	BIT4		# SCALE 2(+11)	   SHIFT RIGHT 11
		TS	L
		CAF	ZERO
		INDEX	BUF		# ADD IN FRACTIONAL COMPENSATION
		DAS	GCOMP		# (NBD)(DELTAT)     (GYRO PULSES) X 2(+14)

; DRIFT COMPENSATION VALIDATION
; After computing drift compensation, verify the magnitude is significant
; enough to warrant gyro commands. Commands less than 1 pulse are ignored
; to avoid unnecessary platform perturbations. Commands greater than 2 pulses
; set GCOMPSW positive, indicating substantial drift requiring correction.
; This threshold filtering prevents jitter from fractional pulse commands.
;
DRFTSUB2	CAF	TWO		# PIPAX, PIPAY, PIPAZ
		AD	BUF
		XCH	BUF
		INDEX	A
		CCS	GCOMP		# ARE GYRO COMMANDS 1 PULSE OR GREATER
		TCF	+2		# YES
		TC	BUF +1	        # NO

		MASK	COMPCHK		# DEC -1
		CCS	A		# ARE GYRO COMMANDS GREATER THAN 2 PULSES
		TS	GCOMPSW		# YES - SET GCOMPSW POSITIVE
		TC	BUF +1	        # NO

# Page 333
; ============================================================================
; GYRO TORQUING COMMAND GENERATOR (1/GYRO)
;
; Scales computed gyro compensation commands for physical platform torquing.
; The IMU stabilized platform maintains inertial orientation via gyroscopes
; that require periodic torquing pulses to correct drift and acceleration-
; induced errors. This routine rescales GCOMP values (gyro pulses × 2^14)
; to the format required by IMUPULSE hardware interface routine (shift right
; 7 bits). Commands are issued to all three gyros (PIPAZ, PIPAY, PIPAX) in
; sequence, then the routine waits for pulse delivery via IMUSTALL before
; completing the compensation cycle. This torquing maintains platform accuracy
; throughout lunar descent, landing, ascent, and rendezvous maneuvers where
; precise attitude knowledge is critical for guidance and control.
;
; Processing: For each axis (Z, Y, X):
;   - Extract fractional pulses, scale by BIT8 (shift right 7)
;   - Extract integer pulses, scale by BIT8, combine with fractional
;   - Result: Gyro commands in IMUPULSE-compatible format
; Hardware: Calls IMUPULSE (torque gyros), IMUSTALL (wait for completion)
; ============================================================================
;
1/GYRO		CAF	FOUR		# PIPAZ, PIPAY, PIPAX
		TS	BUF

		INDEX	BUF		# SCALE GYRO COMMANDS FOR IMUPULSE
		CA	GCOMP  +1	# FRACTIONAL PULSES
		EXTEND
		MP	BIT8		# SHIFT RIGHT 7
		INDEX	BUF
		TS	GCOMP  +1	# FRACTIONAL PULSES SCALED

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

; GYRO COMMAND RESCALING (GCOMP1)
;
; Reverses the scaling performed earlier, preparing gyro compensation values
; for subsequent processing cycles or alternative output paths. This routine
; shifts the minor part (fractional pulses) left by 7 bits, restoring the
; original internal representation. Executed for all three gyro axes in
; sequence (Z, Y, X), then exits. This rescaling is necessary when the
; compensation package is invoked without immediate torquing, allowing the
; computed drift corrections to be stored for later use or analysis.
;
; Processing: For each axis, shift GCOMP+1 minor part left 7 bits via BIT8
; Exit: Returns to calling program via ENDOFJOB
;
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
COMPCHK		DEC	-1		# LESS THAN ZERO IMPOSSIBLE
		TCF	ENDOFJOB

# Page 334
; ============================================================================
; DRIFT-ONLY COMPENSATION MODE (NBDONLY)
;
; This routine applies only gyro drift compensation (NBD terms) without
; accelerometer compensation, typically used during coast phases or when
; acceleration effects are negligible. Entry is contingent on GCOMPSW being
; non-negative, indicating previous compensation commands were small enough
; (≤2 pulses) that full acceleration compensation can be skipped this cycle.
;
; COMMENT-ONLY READERS: During lunar orbit coast or when the spacecraft is
; on the lunar surface with engines off, the IMU platform experiences slow
; gyro drift but minimal acceleration. This routine applies time-based drift
; corrections (NBD bias terms) to maintain platform accuracy without the
; computational overhead of full acceleration compensation. On the surface
; (when FLAGWRD8 BIT8 is set), acceleration terms are included to account
; for lunar gravity effects on the horizontal platform.
;
; CODE-ALONG READERS: Interrupt protection via INHINT prevents T3RUPT
; coincidence during time sampling. TIME1 provides elapsed time for drift
; integration (DELTAT). The surface flag check determines whether to call
; IRIGX/IRIGY/IRIGZ for acceleration terms. Drift compensation uses NBDX,
; NBDY, NBDZ bias values scaled (gyro pulses/cs) × 2^-5, integrated over
; DELTAT (cs) × 2^14 to yield gyro pulse commands × 2^14.
; ============================================================================
;
NBDONLY		CCS	GCOMPSW		# BYPASS IF GCOMPSW NEGATIVE
		TCF	+3
		TCF	+2
		TCF	ENDOFJOB

		INHINT
		CCS	FLAGWRD2	# PREREAD T3RUPT MAY COINCIDE
		TCF	ENDOFJOB
		TCF	ENDOFJOB
		TCF	+1

		CA	FLAGWRD8	# IF SURFACE FLAG IS SET, SET TEM1
		MASK	BIT8		# POSITIVE SO THAT THE ACCELERATION TERMS
		TS	TEM1		# WILL BE COMPENSATED.
		EXTEND
		BZF	+3		# ARE WE ON THE SURFACE

		TC	IBNKCALL	# ON THE SURFACE
		CADR	PIPASR +3	# READ PIPAS, BUT DO NOT SCALE THEM

		CA	TIME1		# (CS) X 2(+14)
		XCH	1/PIPADT	# PREVIOUS TIME
		RELINT
		COM
		AD	1/PIPADT	# PRESENT TIME - PREVIOUS TIME
NBD2		AD	HALF		# CORRECT FOR POSSIBLE TIME1 TICK
		AD	HALF
		XCH	L		# IF TIME1 DID NOT TICK, REMOVE RESULTING
		XCH	L		# OVERFLOW.

NBD3		EXTEND			# C(A) = DELTAT    (CS) X 2(+14)
		MP	BIT10		# SHIFT RIGHT 5
		DXCH	VBUF +2

		CA	ZERO
		TS	GCOMPSW		# INDICATE COMMANDS 2 PULSES OR LESS.
		TS	BUF		# INDEX  X, Y, Z.

		CCS	TEM1		# IF SURFACE FLAG IS SET,
		TC	IRIGX		# COMPENSATE ACCELERATION TERMS.

		EXTEND
		DCA	VBUF +2
		DXCH	MPAC		# DELTAT NOW SCALED (CS) X 2(+19)

		CS	NBDX		# (GYRO PULSES)/(CS) X 2(-5)
		TC	FBIASSUB	# -(NBDX)(DELTAT)   (GYRO PULSES) X 2(+14)

		CCS	TEM1		# IF SURFACE FLAG IS SET,
		TC	IRIGY		# COMPENSATE ACCELERATION TERMS.
# Page 335
		EXTEND
		DCS	VBUF +2
		DXCH	MPAC		# DELTAT SCALED (CS) X 2(+19)
		CA	NBDY		# (GYRO PULSES)/(CS) X 2(-5)
		TC	FBIASSUB	# -(NBDY)(DELTAT)   (GYRO PULSES) X 2(+14)

		CCS	TEM1		# IF SURFACE FLAG IS SET.
		TC	IRIGZ		# COMPENSATE ACCELERATION TERMS

		EXTEND
		DCS	VBUF +2
		DXCH	MPAC		# DELTAT SCALED (CS) X 2(+19)
		CS	NBDZ		# (GYRO PULSES)/(CS) X 2(-5)
		TC	FBIASSUB	# +(NBDZ)(DELTAT)   (GYRO PULSES) X 2(+14)

		CCS	GCOMPSW		# ARE GYRO COMMANDS GREATER THAN 2 PULSES
		TCF	1/GYRO		# YES
		TCF	ENDOFJOB	# NO

# Page 336
; FRACTIONAL DRIFT BIAS COMPUTATION SUBROUTINE (FBIASSUB)
;
; Computes high-precision gyro drift compensation by multiplying the NBD
; (gyro bias drift rate) by DELTAT (elapsed time) using double-precision
; arithmetic. This subroutine is called from NBDONLY for each gyro axis,
; accumulating the drift correction into GCOMP arrays. The Q register
; contains the NBD value on entry, and MPAC contains DELTAT. Double-precision
; multiplication handles both integer and fractional parts to maintain
; accuracy over extended coast periods where small drift rates accumulate
; into significant platform errors if not compensated.
;
; Entry: Q = NBD bias (gyro pulses/cs) × 2^-5
;        MPAC = DELTAT (cs) × 2^19
; Exit: GCOMP updated with drift compensation (gyro pulses) × 2^14
;       Continues to DRFTSUB2 for magnitude check
;
FBIASSUB	XCH	Q
		TS	BUF +1

		CA	Q		# NBD SCALED (GYRO PULSES)/(CS) X 2(-5)
		EXTEND
		MP	MPAC		# DELTAT SCALED (CS) X 2(+19)
		INDEX	BUF
		DAS	GCOMP		# HI(NBD)(DELTAT)   (GYRO PULSES) X 2(+14)

		CA	Q		# NOW FRACTIONAL PART
		EXTEND
		MP	MPAC +1
		TS	L
		CAF	ZERO
		INDEX	BUF
		DAS	GCOMP		# (NBD)(DELTAT)     (GYRO PULSES) X 2(+14)

		TCF	DRFTSUB2	# CHECK MAGNITUDE OF COMPENSATION

; ============================================================================
; LAST BIAS COMPENSATION AND SURFACE FLAG HANDLING (LASTBIAS)
;
; Finalizes IMU compensation processing by calling PIPUSE1 to apply computed
; corrections to PIPA readings. Checks compensation switch status and handles
; special case for lunar surface operations where the SURFFBIT flag enables
; acceleration term compensation needed when the LM is stationary on the
; surface (gravity effects on horizontal accelerometers must be compensated).
; Continues to NBD2 to schedule the next compensation cycle with 2-second
; interval for surface operations or standard interval for flight operations.
;
; Historical Context: During Apollo 11's lunar surface stay (21.5 hours),
; this routine continuously compensated IMU drift and accelerometer bias
; while Eagle remained stationary at Tranquility Base, maintaining platform
; alignment accuracy for the ascent phase guidance initialization.
; ============================================================================
;
LASTBIAS	TC	BANKCALL
		CADR	PIPUSE1

		CCS	GCOMPSW
		TCF	+3
		TCF	+2
		TCF	ENDOFJOB

		CA	FLAGWRD8	# IF SURFACE FLAG IS SET, SET TEM1
		MASK	SURFFBIT	# POSITIVE SO THAT THE ACCELERATION TERMS
		TS	TEM1		# WILL BE COMPENSATED.

		CAF	PRIO31		# 2 SECONDS SCALED (CS) X 2(+8)
		XCH	1/PIPADT
		COM
		AD	PIPTIME +1
		TCF	NBD2

; ============================================================================
; GYRO COMPENSATION ARRAY INITIALIZATION (GCOMPZER)
;
; Initializes the GCOMP compensation array to zero before first IMU compensation
; cycle. This routine is called during fresh start or restart sequences to
; establish known initial state for drift compensation accumulators. Zeros
; all six words of GCOMP array (three gyro axes, double-precision values)
; and clears GCOMPSW to enable normal compensation operation. Bank switching
; preserves calling program's EBANK state across initialization.
;
; Entry: Called during system initialization
; Exit: GCOMP[0..5] = 0, GCOMPSW = 0, EBANK restored
;       Returns via IRIG1 path
;
; Technical Note: GCOMP array structure:
;   GCOMP +0,+1 = X-axis gyro compensation (gyro pulses) × 2^14
;   GCOMP +2,+3 = Y-axis gyro compensation (gyro pulses) × 2^14  
;   GCOMP +4,+5 = Z-axis gyro compensation (gyro pulses) × 2^14
; ============================================================================
;
GCOMPZER	CAF	LGCOMP		# ROUTINE TO ZERO GCOMP BEFORE FIRST
		XCH	EBANK		# CALL TO 1/PIPA
		TS	MODE

		CAF	ZERO
		TS	GCOMPSW
		TS	GCOMP
		TS	GCOMP +1
		TS	GCOMP +2
		TS	GCOMP +3
		TS	GCOMP +4
# Page 337
		TS	GCOMP +5

		TCF	IRIG1		# RESTORE EBANK AND RETURN
