# Copyright:	Public domain.
# Filename:	IMU_PERFORMANCE_TESTS_4.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	382-389
# Mod history:	2009-05-17 RSB	Adapted from the corresponding
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
; FILE: IMU_PERFORMANCE_TESTS_4.agc
; MODULE: IMU Performance Testing Suite
; MISSION PHASE: pre-flight/testing/calibration
;
; TL;DR: Implements gyro drift test filter algorithms for the Inertial
;        Measurement Unit (IMU). Processes PIPA (accelerometer) data through
;        digital filters to measure gyroscope drift rates and platform
;        stability. Called from IMU_PERFORMANCE_TEST_2 during ground testing
;        and in-flight calibration verification.
;
; COMMENT-ONLY READERS: This code validates the spacecraft's navigation
;        sensors are operating correctly - essential for accurate guidance.
; CODE-ALONG READERS: Study the digital filtering implementation (E-1973)
;        and coordinate transformation matrices for drift measurement.
; ============================================================================

# Page 382
; ============================================================================
; GYRO DRIFT TEST FILTER IMPLEMENTATION
;
; This digital filter measures the stability and drift characteristics of
; the IMU gyroscopes - critical sensors that maintain spacecraft attitude
; knowledge. The gyros must remain stable within milliarcsecond tolerances
; to ensure accurate navigation during trans-lunar flight and lunar descent.
;
; The filter algorithm (documented in MIT report E-1973) processes PIPA
; (Pulsed Integrating Pendulous Accelerometer) measurements to extract
; long-term drift rates from the combined gyro and accelerometer data.
; ============================================================================

# PROGRAM --	IMU PERFORMANCE TESTS 4
# DATE --	NOV 15, 1966
# BY --		GEORGE SCHMIDT IL7-146 EXT 1126
# MOD NO-ZERO
#
# FUNCTIONAL DESCRIPTION
#
# THIS SECTION CONSISTS OF THE FILTER FOR THE GYRO DRIFT TESTS.  NO COMPASS
# IS DONE IN LEM.  FOR A DESCRIPTION OF THE FILTER SEE E-1973.  THIS
# SECTION IS ENTERED FROM IMU 2.  IT RETURNS THERE AT END OF TEST.
#
# EARTHR,OGC ZERO,ERTHRVSE
#
# NORMAL EXIT
#
# LENGTHOT GOES TO ZERO -- RETURN TO IMU PERF TESTS 2 CONTROL
#
# ALARMS
#
# 1600	OVERFLOW IN DRIFT TEST
# 1601	BAD IMU MODING IN ANY ROUTINE THAT USES IMUSTALL
#	OUTPUT
#
# FLASHING DISPLAY OF RESULTS -- CONTROLLED IN IMU PERF TESTS 2
#
# DEBRIS
#
# ALL CENTRALS -- ALL OF EBANK XSM

# Page 383
		BANK	33
		SETLOC	IMU4
		BANK
		COUNT*	$$/P07

		EBANK=	XSM

; ============================================================================
; ESTIMS - FILTER INITIALIZATION AND TEST SETUP
;
; This routine initializes the gyro drift filter by:
; - Scheduling the main filter loop (ALLOOP) to run every 1 second
; - Zeroing PIPA pulse counters to establish measurement baseline
; - Clearing filter state variables and computation buffers
; - Optionally computing Earth-rate compensation for vertical gyro tests
;
; The test measures how much the gyroscopes drift over time compared to
; true inertial space. For the Apollo missions, gyro drift rates had to
; remain below 0.01 degrees per hour to maintain navigation accuracy.
; ============================================================================

ESTIMS		INHINT
		CAE	1SECXT
		TC	TWIDDLE
		EBANK=	XSM
		ADRES	ALLOOP
		CAF	ZERO		# ZERO THE PIPAS
		TS	PIPAX
		TS	PIPAY
		TS	PIPAZ
		RELINT

; Schedule ALLOOP to execute every 1 second using the WAITLIST timer.
; TWIDDLE sets up recurring task execution for periodic filter updates.
; The 1-second sampling interval provides adequate resolution for measuring
; gyro drift rates without overwhelming the AGC's computational capacity.

; Zero all three PIPA counters (X, Y, Z axes) to establish measurement
; baseline. The PIPAs accumulate acceleration pulses; zeroing them allows
; the filter to measure incremental changes over each 1-second interval.
; Each axis corresponds to a spacecraft body frame direction.

; Clear 77 decimal memory locations starting at ALXXXZ to zero out all
; filter state variables, intermediate computation buffers, and result
; storage locations. This ensures no residual data from previous tests
; contaminates the current measurement session.
		CA	77DECML
		TS	ZERONDX
		CA	ALXXXZ
		TC	ZEROING

; Enter interpretive mode for vector/matrix operations. The interpretive
; language provides efficient double-precision arithmetic for the coordinate
; transformations and filtering computations required by the drift test.
		TC	INTPRET
		SLOAD
			SCHZEROS
		STOVL	GCOMPSW -1
			INTVAL 	+2
		STOVL	ALX1S
			SCHZEROS
		STORE	DELVX
		STORE	GCOMP

; Initialize filter parameters and clear delta-velocity accumulators.
; GCOMPSW controls gravity compensation mode, DELVX stores velocity changes.
; These variables accumulate the integrated PIPA measurements that will be
; processed by the drift filter algorithm.

; Check TORQNDX to determine if vertical gyro test is requested.
; If TORQNDX is negative, skip Earth-rate compensation (ERTHRVSE).
; For horizontal gyro tests, Earth's rotation creates apparent drift that
; must be mathematically compensated to isolate true gyro drift from
; planetary rotation effects (15 degrees per hour).
		SLOAD
			TORQNDX
		DCOMP	BMN
			VERTSKIP
		CALL
			ERTHRVSE
VERTSKIP	EXIT
		TC	SLEEPIE +1

# Page 384
; ============================================================================
; ALLOOP - MAIN FILTER LOOP (EXECUTED EVERY 1 SECOND)
;
; This is the heart of the gyro drift test, executing once per second to:
; - Check for computational overflow conditions
; - Read accumulated PIPA pulses (acceleration measurements)
; - Store delta-velocity values for filter processing
; - Launch the ALFLT job to compute filtered drift estimates
;
; The loop continues until LENGTHOT counts down to zero, indicating the
; test duration has completed and results are ready for crew review.
; ============================================================================

ALLOOP		CA	OVFLOWCK
		EXTEND
		BZF	+2
		TC	TASKOVER

; Check OVFLOWCK flag for arithmetic overflow in previous filter iteration.
; If overflow detected, terminate test immediately (alarm 1600 will trigger).
; Overflow indicates excessive gyro drift or computational instability.

		CCS	ALTIM
		CA	A		# SHOULD NEVER HIT THIS LOCATION
		TS	ALTIMS
		CS	A
		TS	ALTIM

; Toggle ALTIM time marker between iterations for timing verification.
; ALTIMS stores the current state for diagnostic purposes.

; Check if gravity compensation cycle is complete (GEOCOMPS counter).
; If not zero, continue test. When LENGTHOT reaches zero or goes negative,
; test is complete and loop terminates, returning control to IMU PERF TESTS 2.
		CS	ONE
		AD	GEOCOMPS
		EXTEND
		BZF	+4
		CA	LENGTHOT
		EXTEND
		BZMF	+5

; Reschedule ALLOOP for next 1-second iteration via WAITLIST.
; This maintains precise 1-second sampling throughout test duration.
		CAE	1SECXT
		TC	TWIDDLE
		EBANK=	XSM
		ADRES	ALLOOP

; Read and clear PIPA pulse counters for all three axes.
; Each XCH instruction atomically reads the accumulated pulses since last
; sample and resets counter to zero. The pulse counts represent integrated
; acceleration over the 1-second interval, scaled in PIPA-specific units.
		CAF	ZERO
		XCH	PIPAX
		TS	DELVX
		CAF	ZERO
		XCH	PIPAY
		TS	DELVY
		CAF	ZERO
		XCH	PIPAZ
		TS	DELVZ

; ============================================================================
; SPECSTS - LAUNCH FILTER COMPUTATION JOB
;
; Schedule the ALFLT (filter computation) job at priority 20 using FINDVAC.
; This allows the computationally intensive filtering algorithms to run as
; a background job without disrupting the precise 1-second timing loop.
; The filter processes accumulated PIPA data to extract gyro drift rates.
; ============================================================================

SPECSTS		CAF	PRIO20
		TC	FINDVAC
		EBANK=	XSM
		2CADR	ALFLT		# START THE JOB

		TC	TASKOVER

# Page 385
; ============================================================================
; ALFLT - DRIFT FILTER COMPUTATION ROUTINE
;
; This is the core signal processing algorithm that extracts gyro drift
; estimates from PIPA measurements. The filter must separate true inertial
; drift from spacecraft dynamics, gravity gradients, and Earth rotation.
;
; The algorithm performs:
; - Coordinate transformation of delta-velocity vectors to computation frame
; - Integration of measurements over multiple samples
; - Statistical filtering to reduce measurement noise
; - Drift rate extraction using techniques from MIT report E-1973
; ============================================================================

ALFLT		CCS	GEOCOMPS
		TC	+2
		TC	NORMLOP
		TC	BANKCALL
		CADR	1/PIPA

; Check GEOCOMPS flag to determine if this is initial gravity compensation
; phase or normal filter operation. Initial phase calibrates gravity model.
; Call 1/PIPA to convert PIPA pulse counts to standard acceleration units.

NORMLOP		TC	INTPRET

; Enter interpretive mode for vector mathematics and double-precision
; operations required by the filtering algorithm.

		DLOAD
			INTVAL
		STOVL	S1
			DELVX
		VXM	VSL1
			XSM

; Transform delta-velocity vector (DELVX, DELVY, DELVZ) from spacecraft
; body frame to stable member (platform) frame using XSM matrix.
; VSL1 shifts left 1 bit to maintain proper fixed-point scaling.

		DLOAD	DCOMP
			MPAC +3
		STODL	DPIPAY
			MPAC +5
		STORE	DPIPAZ

; Extract Y and Z components from transformed vector and store in DPIPAY,
; DPIPAZ for filter processing. DCOMP negates Y component for proper
; coordinate system alignment.

		SETPD	AXT,1
			0
			8D

; Initialize pushdown stack pointer (PD=0) and index register X1=8.
; The filter algorithm uses the pushdown stack for intermediate vector
; calculations and requires careful stack management.

		SLOAD	DCOMP
			GEOCOMPS
		BMN
			PERFERAS

; Check GEOCOMPS flag (negated). If negative, branch to PERFERAS for
; Earth rate vector subtraction. This distinguishes between initial
; gravity compensation phase and steady-state drift measurement.

ALCGKK		SLOAD	BMN
			ALTIMS
			ALFLT3

; Test ALTIMS timing marker. If negative, skip parameter loading and
; proceed directly to ALFLT3 computation loop. This maintains filter
; state continuity across measurement intervals.

ALKCG		AXT,2	LXA,1		# LOADS SLOPES AND TIME CONSTANTS AT RQST
			12D
			ALX1S

; Initialize index registers for loading filter coefficients.
; X2=12 counts down parameter slots, X1 points to coefficient source.
; Filter "slopes" are gain parameters, "time constants" set response speed.

ALKCG2		DLOAD*	INCR,1
			ALFDK 	+144D,1
		DEC	-2
		STORE	ALDK 	+10D,2
		TIX,2	SXA,1
			ALKCG2
			ALX1S

; Load filter parameters from ALFDK table into working memory ALDK.
; The loop transfers 6 double-precision values (slopes and time constants)
; that define filter frequency response. These were pre-calculated based
; on gyro noise characteristics and mission duration requirements.

; ============================================================================
; ALFLT3 - MAIN FILTER COMPUTATION LOOP
;
; Implements the recursive filtering equations that extract gyro drift from
; noisy PIPA measurements. The filter operates on Y and Z axis data (X axis
; drift cannot be measured in Earth-rate tests due to polar alignment).
; ============================================================================

ALFLT3		AXT,1
			8D

; Initialize X1=8 for dual-axis processing (Y and Z, indexed at +8 offset).

DELMLP		DLOAD*	DMP
			DPIPAY 	+8D,1
			PIPASC
		SLR	BDSU*
			9D
			INTY	+8D,1
		STORE	INTY 	+8D,1

; Scale PIPA measurement by PIPASC conversion factor (pulses to velocity).
; Shift right 9 bits to align with integration state INTY scaling.
; Subtract from integrated state (INTY) - this is the error term that
; drives the filter. Store updated integration state.

		PDDL	DMP*
			VELSC
# Page 386
			VLAUN 	+8D,1
		SL2R
		DSU	STADR
		STORE	DELM 	+8D,1
		STORE	DELM 	+10D,1

; Compute velocity difference: scale launch velocity (VLAUN) by VELSC,
; shift left 2 bits, subtract from stack (measurement residual).
; Result is DELM - the measurement innovation that updates filter state.
; Store in both DELM slots for dual processing paths.

		TIX,1	AXT,2
			DELMLP
			4

; Loop back for second axis (X1 decrements). After both Y and Z axes
; processed, initialize X2=4 for next computation phase.
; Coefficient update loop: multiply ALK gains by ALDK decay factors.
; This implements exponential time decay that gradually adapts filter
; response as measurement history accumulates. Critical for achieving
; optimal convergence between fast initial response and low steady-state noise.

ALILP		DLOAD*	DMPR*
			ALK 	+4,2
			ALDK 	+4,2
		STORE	ALK 	+4,2
		TIX,2	AXT,2
			ALILP
			8D

; Loop through 4 coefficient pairs, decaying each by its time constant.
; After completion, initialize X2=8 for state update loop.
; ============================================================================
; ALKLP - STATE PROPAGATION LOOP
;
; Updates all filter state variables using measurement innovations (DELM).
; This is the heart of the Kalman-like algorithm that recursively refines
; drift estimates as measurements accumulate over time.
; ============================================================================

ALKLP		LXC,1	SXA,1
			CMPX1
			CMPX1

; Load and store CMPX1 comparison index for nested loop control.

		DLOAD*	DMPR*
			ALK 	+1,1
			DELM 	+8D,2
		DAD*
			INTY 	+8D,2
		STORE	INTY 	+8D,2

; First state update: multiply gain ALK by measurement innovation DELM,
; add result to integration state INTY. This propagates the measurement
; correction through the first filter stage.

		DLOAD*	DAD*
			ALK 	+12D,2
			ALDK 	+12D,2
		STORE	ALK 	+12D,2
		DMPR*	DAD*
			DELM 	+8D,2
			INTY 	+16D,2
		STORE	INTY 	+16D,2

; Second stage update: adjust ALK coefficient by adding ALDK time constant,
; then multiply by DELM and add to INTY second stage. This implements the
; cascade filter structure that provides superior noise rejection.

		DLOAD*	DMP*
			ALSK 	+1,1
			DELM 	+8D,2
		SL1R	DAD*
			VLAUN 	+8D,2
		STORE	VLAUN 	+8D,2

; Update velocity estimate: multiply slope ALSK by innovation, shift left
; 1 bit for scaling, add to launch velocity VLAUN. This extracts the
; integrated drift velocity that represents cumulative gyro error.

		TIX,2	AXT,1
			ALKLP
			8D

; Loop through both axes (X2 indexes Y and Z), then initialize X1=8 for
; next phase. Each axis receives complete state update before continuing.

; ============================================================================
; LOOSE - STATE VECTOR TRANSFORMATION
;
; Transforms estimated position, velocity, and acceleration states back
; through coordinate frames to prepare for drift angle computation.
; Applies inverse transformation matrix to align states with platform frame.
; ============================================================================

LOOSE		DLOAD*	PDDL*
			ACCWD 	+8D,1
			VLAUN 	+8D,1
		PDDL*	VDEF
			POSNV 	+8D,1

; Build state vector from acceleration (ACCWD), velocity (VLAUN), and
; position (POSNV) components. Push each double-precision value onto
; stack, then VDEF constructs 3-element vector for transformation.

		MXV	VSL1
			TRANSM1
# Page 387
		DLOAD
			MPAC
		STORE	POSNV	 +8D,1
		DLOAD
			MPAC	 +3
		STORE	VLAUN 	+8D,1
		DLOAD
			MPAC	+5
		STORE	ACCWD 	+8D,1

; Transform state vector by multiplication with TRANSM1 matrix (inverse
; of coordinate transformation applied earlier). Shift left 1 bit to
; maintain precision. Extract X, Y, Z components from result (MPAC)
; and store back into position, velocity, acceleration state variables.

		TIX,1
			LOOSE

; Process both Y and Z axes through transformation loop.

; ============================================================================
; BOOP - SINE AND COSINE EVALUATION
;
; Computes sine and cosine of drift angles for transformation to inertial
; frame. ANGX contains accumulated angle estimates. Results stored in
; arrays at indexed locations for subsequent coordinate transformations.
; ============================================================================

		AXT,2	AXT,1		# EVALUATE SINES AND COSINES
			6
			2

; Initialize index registers: X2=6 (loop counter for three axes),
; X1=2 (starting offset for indexed access). Loop computes sine/cosine
; for all three gyro axes (X, Y, Z).

BOOP		DLOAD*	DMPR
			ANGX 	+2,1
			GEORGEJ

; Load angle from ANGX array (indexed by X1), multiply by GEORGEJ scaling
; constant to convert to proper angular units for trigonometric functions.
; GEORGEJ represents Earth's rotation rate scaling factor.

		SR2R
		PUSH	SIN

; Shift result right 2 bits (SR2R) for proper scaling, push onto stack,
; then compute sine of scaled angle. Sine represents component of drift
; angle perpendicular to reference axis.

		SL3R	XAD,1
			X1
		STORE	16D,2

; Shift sine result left 3 bits to restore scaling, add X1 offset for
; proper positioning, store in array at location 16D indexed by X2.
; This builds sine array for all three axes.

		DLOAD
		COS
		STORE	22D,2		# COSINES

; Reload scaled angle from stack and compute cosine. Store cosine in
; separate array at location 22D indexed by X2. Cosine represents
; component of drift angle parallel to reference axis.

		TIX,2
			BOOP

; Loop through all three axes (decrement X2, branch to BOOP if non-zero).
; Upon completion, sine and cosine tables contain transformation values
; for converting platform drift angles to inertial frame.

; ============================================================================
; PERFERAS - EARTH RATE VECTOR CALCULATION
;
; Exits interpretive mode and switches to erasable bank 7 to execute
; Earth rate calculation routine. This routine computes Earth's rotation
; vector in platform coordinates for drift compensation.
; ============================================================================

PERFERAS	EXIT
		CA	EBANK7
		TS	EBANK
		EBANK=	ATIGINC

; Exit interpreter, load EBANK7 constant, and set EBANK register to
; switch memory bank context. Prepares to call Earth rate calculation
; routine located in erasable bank 7.

		TC	ATIGINC		# GOTO ERASABLE TO CALCULATE ONLY TO RETN

; Transfer control to ATIGINC routine in erasable memory. This routine
; calculates attitude-dependent Earth rate components and returns here
; upon completion. Subroutine performs vector transformations accounting
; for spacecraft attitude relative to Earth-fixed reference frame.

# 			     CAUTION
#
# THE ERASABLE PROGRAM THAT DOES THE CALCULATIONS MUST BE LOADED
# BEFORE ANY ATTEMPT IS MAKE TO RUN THE IMU PERFORMANCE TEST

; CRITICAL: The erasable bank 7 calculation routine must be loaded into
; memory before initiating IMU performance tests. Missing routine will
; cause test failure or incorrect drift measurements.

; ============================================================================
; TEST COMPLETION AND CONTROL LOGIC
;
; Following Earth rate compensation, check if test duration has completed
; (LENGTHOT counter reaches zero). If test continues, evaluate torquing
; index to determine next processing phase.
; ============================================================================

		EBANK=	AZIMUTH
		CCS	LENGTHOT
		TC	SLEEPIE

; Switch to AZIMUTH EBANK context. Check LENGTHOT (test duration counter).
; If positive (test incomplete), transfer to SLEEPIE to suspend job and
; await next measurement cycle. If zero (test complete), fall through to
; finalization logic.

		CCS	TORQNDX
		TCF	+2

; Check TORQNDX (torquing index control). If positive, skip ahead two
; instructions to continue processing. If zero or negative, execute
; next instruction for alternate processing path.

		TC	SETUPER1
		CA	CDUX
		TS	LOSVEC	 +1	# FOR TROUBLESHOOTING VD POSNS 2$4

; If TORQNDX is zero/negative, call SETUPER1 to process drift angles.
; Then load CDUX (X-axis CDU angle) and store in LOSVEC+1 for diagnostic
; troubleshooting purposes (allows verification of platform position).

# Page 388

; ============================================================================
; SETUPER1 - DRIFT ANGLE PROCESSING
;
; Processes accumulated drift angles from all three axes (X, Y, Z) and
; scales them by Earth rate constant (GEORGEJ) to convert from platform
; frame measurements to inertial frame drift rates.
; ============================================================================

SETUPER1	TC	INTPRET
		DLOAD	PDDL		# ANGLES FROM DRIFT TEST ONLY
			ANGZ
			ANGY
		PDDL	VDEF
			ANGX

; Enter interpreter mode. Load drift angles from all three axes: ANGZ
; (Z-axis drift), ANGY (Y-axis drift), ANGX (X-axis drift). Push each
; onto stack (PDDL = Push Double, Double Load) then construct vector
; (VDEF) representing complete three-axis drift measurement.

		VCOMP	VXSC
			GEORGEJ

; Complement the drift angle vector (VCOMP negates all three components),
; then scale by multiplying with GEORGEJ constant (VXSC = Vector times
; Scalar). GEORGEJ converts drift angles from measurement units to
; Earth rate units, producing drift rate vector in radians per second.

		MXV	VSR1
			XSM
		STORE	OGC

; Multiply drift rate vector by XSM transformation matrix (MXV = Matrix
; times Vector) to convert from platform coordinates to stable member
; coordinates. Shift result right 1 bit (VSR1) for proper scaling.
; Store final drift rate vector in OGC (Outer Gimbal Compensation).

		EXIT

; Exit interpreter mode to execute native AGC instructions for IMU
; torquing commands.

		CA	OGCPL
		TC	BANKCALL
		CADR	IMUPULSE
		TC	IMUSLLLG

; Load OGCPL (OGC pulse count), execute cross-bank call to IMUPULSE
; routine which generates torquing pulses to gyros based on computed
; drift rates. Then transfer to IMUSLLLG (IMU Stall Logic) to verify
; IMU responded properly to torquing commands.

; ============================================================================
; GEOSTRT4 - VERTICAL DRIFT TEST HANDLER
;
; Re-entry point following IMU torquing. Checks if vertical drift test
; is active and handles Earth rate vector computation accordingly.
; ============================================================================

GEOSTRT4	CCS	TORQNDX		# ONLY POSITIVE IF IN VERTICAL DRIFT TEST
		TC	VALMIS

; Check TORQNDX to determine if performing vertical drift test (positive
; value indicates vertical test active). If positive, transfer to VALMIS
; routine for specialized vertical drift processing. If zero/negative,
; continue with standard drift test completion.

		TC	INTPRET
		CALL
			ERTHRVSE
		EXIT
		TC	TORQUE

; Enter interpreter mode, call ERTHRVSE (Earth Rate Vector Subroutine in
; Erase) to compute Earth rotation compensation vector. Exit interpreter
; and transfer to TORQUE routine to apply computed torquing corrections
; to IMU gyros.

; ============================================================================
; SLEEPIE - TEST CONTINUATION LOGIC
;
; Decrements test duration counter and determines next processing step.
; Entry from ALLOOP when test cycle completes. If vertical drift test is
; active (TORQNDX positive), computes Earth rate compensation. Otherwise,
; ends current job and awaits next scheduled measurement cycle.
; ============================================================================

SLEEPIE		TS	LENGTHOT	# TEST NOT OVER-DECREMENT LENGTHOT

; Store accumulator (containing decremented value from prior CCS) into
; LENGTHOT, reducing remaining test duration. When LENGTHOT reaches zero,
; test terminates and control returns to IMU_PERFORMANCE_TEST_2 for
; results display.

		CCS	TORQNDX		# ARE WE DOING VERTDRIFT
		TC	EARTHR*

; Check TORQNDX (torquing index). If positive, vertical drift test is
; active requiring Earth rate compensation. Transfer to EARTHR* routine
; which computes Earth rotation vector in platform coordinates. If zero
; or negative, fall through to job termination.

		TC	ENDOFJOB

; Terminate current job and return control to executive scheduler. Job
; will resume at next WAITLIST timer expiration when SPECSTS task is
; dispatched for subsequent measurement cycle (7.68 seconds later).

; ============================================================================
; ERROR HANDLING SECTION
; ============================================================================

; ============================================================================
; SOMEERRR - DRIFT TEST OVERFLOW ERROR HANDLER
;
; Triggered when numerical overflow detected during drift filter
; computations. Sets EBANK context, stops ALLOOP recursion, issues alarm
; 1600, then terminates test returning to IMU_PERFORMANCE_TEST_2 control.
; ============================================================================

SOMEERRR	CA	EBANK5
		TS	EBANK

; Load EBANK5 constant and store to EBANK register, setting erasable memory
; bank context to bank 5. Ensures subsequent memory references access correct
; erasable bank during error recovery sequence.

		CA	ONE
		TS	OVFLOWCK	# STOP ALLOOP FROM CALLING ITSELF

; Load constant ONE and store to OVFLOWCK (overflow check flag). This
; prevents ALLOOP from recursively calling itself after overflow detected,
; breaking the measurement cycle and allowing clean test termination.

		TC	ALARM
		OCT	1600

; Transfer to ALARM routine with alarm code 1600 (overflow in drift test).
; Displays alarm to crew via DSKY, indicating IMU drift measurements exceeded
; computational range. May indicate IMU malfunction or extreme drift rates
; beyond AGC fixed-point precision limits.

		TC	ENDTEST1

; Transfer to ENDTEST1, returning control to IMU_PERFORMANCE_TEST_2 for
; test cleanup and results display. Test terminates abnormally due to
; overflow condition.

; ============================================================================
; SOMERR2 - IMU MODE ERROR HANDLER
;
; Triggered when IMU fails to respond properly during IMUSTALL verification.
; Issues alarm 1601 (bad IMU moding), clears IMUSE flag, and terminates job.
; Prevents continued operation with IMU in uncertain state.
; ============================================================================

SOMERR2		CAF	OCT1601
		TC	VARALARM

; Load OCT1601 (alarm code 1601: bad IMU moding) and transfer to VARALARM
; (variable alarm routine). This alarm indicates IMU did not respond correctly
; to mode commands during gyro torquing or other IMU operations. Suggests
; hardware failure or IMU power/mode switch configuration error.

		TC	DOWNFLAG
		ADRES	IMUSE

; Transfer to DOWNFLAG routine with address of IMUSE flag. Clears IMUSE
; (IMU In Use flag) indicating IMU is no longer being accessed by test
; program. Releases IMU for potential use by other programs or crew
; reconfiguration.

		TC	ENDOFJOB

; Transfer to ENDOFJOB, suspending current job and returning control to
; executive scheduler. Test terminates abnormally. Crew must investigate
; IMU status and potentially recycle IMU power or perform diagnostic
; procedures before retrying performance tests.

; ============================================================================
; CONSTANTS AND SCALING FACTORS SECTION
;
; Mathematical constants, alarm codes, filter gains, and scaling factors
; used throughout IMU performance test computations. Includes Kalman filter
; coefficients (GEORGEJ, GEORGEK), PIPA scaling, velocity compensation gains,
; and erasable memory address references.
; ============================================================================

OCT1601		OCT	01601

; Alarm code 1601 (bad IMU moding in any routine using IMUSTALL). Used by
; SOMERR2 error handler when IMU fails to respond properly to mode commands.

DEC585		OCT	06200		# 3200 B+14 ORDER IS IMPORTANT

; Decimal 585 stored as octal 06200. Represents scaled value 3200 at B+14
; scaling (multiplied by 2^14 = 16384). Used in drift filter computations.
; ORDER IS IMPORTANT comment indicates this constant's position relative to
; surrounding data is significant for table indexing.

SCHZEROS	2DEC	.00000000
# Page 389
		2DEC	.00000000

; SCHZEROS - Double-precision zero vector (two consecutive 2DEC values).
; Used to initialize erasable memory locations to zero state during test
; setup. Two-word structure provides 28-bit precision zero reference.

		OCT	00000
ONEDPP		OCT	00000		# ORDER IS IMPORTANT
		OCT	00001

; ONEDPP - Double-precision positive one (three-word structure). First word
; is padding zero, second and third words form DP value +1.0. ORDER IS
; IMPORTANT indicates position-dependent table access. Used for unity gain
; or normalization operations in filter computations.

INTVAL		OCT	4
		OCT	2
		DEC	144
		DEC	-1

; INTVAL - Integer values table containing: 4 (octal), 2 (octal), 144
; (decimal = 0144 octal = 100 decimal), -1 (decimal). Used for loop counters,
; array indices, and iteration control in drift test algorithms. Value 144
; corresponds to number of measurement samples in certain test configurations.

SOUPLY		2DEC	.93505870	# INITIAL GAINS FOR PIP OUTPUTS

; SOUPLY - Initial gain factor 0.93505870 for PIPA (Pulsed Integrating
; Pendulous Accelerometer) output processing. Applied to raw PIPA pulse
; counts to convert to scaled velocity increments in drift filter. Gain
; value derived from PIPA scale factor calibration and desired filter
; response characteristics.

		2DEC	.26266423	# INITIAL GAINS/4 FOR ERECTION ANGLES

; Initial gain factor 0.26266423 for platform erection angle computations.
; Value is SOUPLY/4 approximately, providing reduced gain for angular error
; correction during gyrocompass alignment. Prevents excessive correction
; rates that could destabilize platform orientation.

77DECML		DEC	77

; Decimal 77 constant used as loop counter or array size. Corresponds to
; number of erasable memory locations to be zeroed during test initialization
; (ZERONDX counter in ESTIMS routine). Value 77 covers complete XSM erasable
; bank variable set.

ALXXXZ		GENADR	ALX1S 	-1

; ALXXXZ - Generated address pointing to (ALX1S - 1), one word before ALX1S
; variable. Used as base address for indexed access to drift test variable
; arrays. GENADR directive generates both bank and offset components for
; cross-bank memory access.

PIPASC		2DEC	.13055869

; PIPASC - PIPA scaling factor 0.13055869 converting PIPA pulse counts to
; physical units. Represents centimeters/second per PIPA pulse based on
; accelerometer pendulum geometry and readout electronics characteristics.
; Critical calibration constant affecting all inertial velocity measurements.

VELSC		2DEC	-.52223476	# 512/980.402

; VELSC - Velocity scaling factor -0.52223476 = 512/980.402. Negative sign
; indicates coordinate frame inversion. Denominator 980.402 is Earth surface
; gravity in cm/sec^2. Factor converts between AGC internal velocity
; representation and physical velocity units accounting for gravity scaling.

ALSK		2DEC	.17329931	# SSWAY VEL GAIN X 980.402/4096

; ALSK - Sway velocity gain 0.17329931 for drift filter. Computed as
; (sway velocity gain × 980.402 / 4096) combining filter gain with gravity
; scaling and AGC word length normalization. Applied to velocity error
; components in Kalman filter state propagation.

		2DEC	-.00835370	# SSWAY ACCEL GAIN X 980.402/4096

; Sway acceleration gain -0.00835370 for drift filter. Computed as
; (sway acceleration gain × 980.402 / 4096). Negative sign indicates
; correction direction. Applied to acceleration error terms in Kalman
; filter covariance update equations.

GEORGEJ		2DEC	.63661977

; GEORGEJ - Kalman filter coefficient 0.63661977 (filter designer: George
; Schmidt). Tuning parameter balancing measurement noise versus process noise
; in optimal state estimation. Name honors George Schmidt of MIT Instrumentation
; Laboratory who derived AGC Kalman filter equations documented in E-1973.

GEORGEK		2DEC	.59737013

; GEORGEK - Kalman filter coefficient 0.59737013 (second George Schmidt
; parameter). Complementary tuning factor to GEORGEJ providing optimal
; estimation gain for gyro drift rate state variables. Together, GEORGEJ and
; GEORGEK define filter bandwidth and noise rejection characteristics critical
; for accurate drift measurement during Apollo 11 IMU performance validation.

