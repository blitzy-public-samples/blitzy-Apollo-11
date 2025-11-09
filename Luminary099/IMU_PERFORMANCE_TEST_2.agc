# Copyright:	Public domain.
# Filename:	IMU_PERFORMANCE_TEST_2.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	373-381
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

# Page 373
# NAME --	IMU PERFORMANCE TESTS 2
#
# DATE --	MARCH 20, 1967
#
# BY --		SYSTEM TEST GROUP 864-6900 EXT. 1274
#
# MODNO. --	ZERO
#
# FUNCTIONAL DESCRIPTION
#
# POSITIONING ROUTINES FOR THE IMU PERFORMANCE TESTS AS WELL AS SOME OF
# THE TESTS THEMSELVES.  FOR A DESCRIPTION OF THESE SUBROUTINES AND THE
# OPERATING PROCEDURES (TYPICALLY) SEE STG MEMO 685.  THEORETICAL REF. E-1973

; ============================================================================
; FILE: IMU_PERFORMANCE_TEST_2.agc
; MODULE: IMU System Tests
; MISSION PHASE: Pre-flight testing, in-flight verification
;
; TL;DR: Implements comprehensive IMU (Inertial Measurement Unit) performance
;        test procedures including gyroscope drift measurement, PIPA (Pulsed
;        Integrating Pendulous Accelerometer) accuracy verification, and
;        platform alignment quality assessment. These tests validate that the
;        IMU can provide accurate navigation data essential for lunar landing.
;
; COMMENT-ONLY READERS: This module contains diagnostic routines that verify
;        the spacecraft's navigation sensors are working correctly. Think of
;        it as the IMU's "health checkup" before and during critical missions.
; CODE-ALONG READERS: Study the integration of positioning algorithms, drift
;        compensation calculations, and PIPA pulse counting mechanisms that
;        ensure navigation accuracy within required tolerances.
; ============================================================================

		BANK	33
		SETLOC	IMU2
		BANK
		EBANK=	POSITON
		COUNT*	$$/P07

; ============================================================================
; IMU TEST INITIALIZATION AND SETUP ROUTINES
;
; The following routines prepare the Inertial Measurement Unit for performance
; testing. The IMU contains three gyroscopes and three accelerometers (PIPAs)
; that must maintain extreme accuracy for successful navigation during lunar
; descent and ascent. These tests verify the IMU operates within specified
; tolerances for drift rate and accelerometer accuracy.
; ============================================================================

REDO		TC	NEWMODEX	; Enter Program 07 (IMU performance test mode)
		MM	07		; Major mode 07 displayed to crew on DSKY

; IMU test entry point - zeros IMU error counters and initializes test state
GEOIMUTT	TC	IMUZERR		; Zero IMU compensation registers (gyro drift accumulators)
IMUBACK		CA	ZERO		; Initialize test control variables
		TS	NDXCTR		; Zero index counter for loop control
		TS	TORQNDX		; Zero torque command index
		TS	TORQNDX +1	; Zero torque command index high word
		TS	OVFLOWCK	; Zero overflow check flag
		
; Initialize navigation base (NB) coordinate frame for test positioning
NBPOSPL		CA	DEC17		; Load count of 17 words to zero
		TS	ZERONDX		; Set zero operation index
		CA	XNBADR		; Load address of XNB vector (navigation base X-axis)
		TC	ZEROING		; Call zeroing routine to clear NB vectors
		CA	HALF		; Load 0.5 (scaled unit value)
		TS	XNB		; Set X navigation base component

; ============================================================================
; LATITUDE AND AZIMUTH VERIFICATION
;
; This section displays current latitude and azimuth settings to the crew
; via DSKY and allows adjustment for IMU alignment testing. Accurate initial
; position and orientation are essential for IMU performance verification.
; ============================================================================

GUESS		TC	INTPRET		; Enter interpretive mode for vector operations
LATAZCHK	DLOAD	SL2		; Load latitude, shift left 2 bits for scaling
			LATITUDE	; Current latitude in revolutions (±0.5 = ±180°)
		STODL	DSPTEM1 +1	; Store for display, load azimuth
			AZIMUTH		; Platform azimuth angle in revolutions
		RTB	EXIT		; Convert single precision to double, exit interpreter
			1STO2S		; 1 word to 2 word conversion
		XCH	MPAC		; Exchange with multi-purpose accumulator
		TS	DSPTEM1		; Store azimuth for display
		CAF	VN0641		; Load verb 06 noun 41 (display lat/long/azimuth)
		TC	BANKCALL	; Cross-bank subroutine call
		CADR	GOFLASH		; Flash display and wait for crew input
		TC	ENDTEST1	; Crew pressed ENTER - end test sequence
		TC	+2		; Crew pressed + (proceed with current values)
		TC	-5		; Crew pressed - (re-display for adjustment)
# Page 374
; Process crew input and compute navigation base orientation vectors
		TC	INTPRET		; Re-enter interpreter mode
		SLOAD	RTB		; Load single precision, convert
			DSPTEM1		; Azimuth value from crew input
			CDULOGIC	; Convert to proper CDU (Coupling Data Unit) format
		STORE	AZIMUTH		; Store updated azimuth angle
		SLOAD	SR2		; Load latitude, shift right 2 bits
			DSPTEM1 +1	; Latitude value from crew input
		STORE	LATITUDE	; Store updated latitude
		
; Compute wander angle components (WANGI, WANGO) for gyrocompass alignment
; Wander angle accounts for Earth rotation and local vertical alignment
		COS	DCOMP		; Cosine of latitude, double complement (negate)
		SL1			; Shift left 1 bit for proper scaling
		STODL	WANGI		; Store inner gimbal wander angle component
			LATITUDE	; Reload latitude for sine computation
		SIN	SL1		; Sine of latitude, shift left 1
		STODL	WANGO		; Store outer gimbal wander angle component
			AZIMUTH		; Load azimuth for NB vector computation
		
; Compute navigation base (NB) coordinate frame unit vectors
; NB frame aligns with local vertical and desired azimuth heading
		PUSH	SIN		; Push to stack, compute sine of azimuth
		STORE	YNB 	+2	; Store Y-axis NB component (East direction)
		STODL	ZNB 	+4	; Store Z-axis NB component, reload azimuth
		COS			; Cosine of azimuth
		STORE	YNB	+4	; Store Y-axis NB Z-component (North direction)
		DCOMP			; Double complement (negate for proper orientation)

; ============================================================================
; GIMBAL POSITIONING ROUTINE
;
; This routine commands the IMU gimbals to the calculated orientation based
; on the navigation base frame. Precise gimbal positioning is critical for
; accurate IMU performance testing and drift measurement.
; ============================================================================

POSGMBL		STCALL	ZNB 	+2	; Store Z-axis NB Y-component, call CALCGA
			CALCGA		; Calculate gimbal angles from NB frame
		EXIT			; Exit interpreter mode
		TC	BANKCALL	; Cross-bank call to IMU coarse align
		CADR	IMUCOARS	; Coarse align routine drives gimbals to position
		
; Check for gimbal lock condition (middle gimbal near 90 degrees)
; Gimbal lock prevents accurate IMU orientation and must be avoided
		CAF	BIT14		# IF BIT14 SET, GIMBAL LOCK
		MASK	FLAGWRD3	; Check gimbal lock flag in flag word 3
		EXTEND			; Extend next instruction
		BZF	+2		; Branch if zero (no gimbal lock)
		INCR	NDXCTR		# +1 IF IN GIMBAL LOCK, OTHERWISE 0
		TC	DOWNFLAG	; Clear gimbal lock failure flag
		ADRES	GLOKFAIL	# RESET GIMBAL LOCK FLAG
		TC	IMUSLLLG	; IMU state logic processing
		
; Determine test sequence based on gimbal lock status
		CCS	NDXCTR		# IF ONE GO AND DO A PIPA TEST ONLY
		TC	PIPACHK		# ALIGN AND MEASURE VERTICAL PIPA RATE
		TC	FINIMUDD	; Finish IMU drift determination (full test)
		
; Insert delay to allow IMU platform suspension system to stabilize
; The IMU is gimbal-mounted with viscous damping that requires settling time
		EXTEND			; Extend next instruction
		DCA	PERFDLAY	; Load performance test delay time (double precision)
		TC	LONGCALL	# DELAY WHILE SUSPENSION STABILIZES
		EBANK=	POSITON	; Set erasable bank for GOESTIMS
		2CADR	GOESTIMS	; Two-word address for long call

; Job sleep/wake mechanism for test synchronization
		CA	ESTICADR	; Load ESTIMS routine address
		TC	JOBSLEEP	; Put this job to sleep
GOESTIMS	CA	ESTICADR	; Wake-up point - reload ESTIMS address
		TC	JOBWAKE		; Wake the ESTIMS job
		TC	TASKOVER	; Terminate this task
ESTICADR	CADR	ESTIMS		; Address of ESTIMS (drift estimation) routine

; ============================================================================
; TORQUE DISPLAY ROUTINE
;
; Displays gyro torque commands to crew on DSKY. Torque values indicate
; the gyro drift corrections being applied during the test.
; ============================================================================

TORQUE		CA	ZERO		; Initialize display data
# Page 375
		TS	DSPTEM2		; Clear display temporary register 2
		CA	DRIFTI		; Load inner gimbal drift rate
		TS	DSPTEM2 +1	; Store for display
		INDEX	POSITON		; Indexed by position counter
		TS	SOUTHDR -1	; Store in drift array (SOUTHDR = South drift rate)
		TC	SHOW		; Display torque values to crew on DSKY

; ============================================================================
; PIPA CHECK ROUTINE
;
; Tests PIPA (Pulsed Integrating Pendulous Accelerometer) accuracy by
; measuring vertical acceleration over time. The vertical PIPA should read
; 1g (Earth gravity) when properly aligned. This test verifies accelerometer
; performance critical for navigation during powered flight.
; ============================================================================

PIPACHK		INDEX	NDXCTR		# PIPA TEST - indexed by gimbal lock status
		TC	+1		; If NDXCTR=0, execute next instruction
		TC	EARTHR*		; If NDXCTR=1, skip to EARTHR* routine
		
; Set up PIPA test parameters
		CA	DEC17		# ALLOW PIP COUNTER TO OVERFLOW 17 TIMES
		TS	DATAPL	+4	# IN THE ALLOTTED TIME INTERVAL
		CA	DEC58		; 58 second test duration (in 1-second units)
		TS	LENGTHOT	; Store test length counter
		CA	ONE		; Single result expected
		TS	RESULTCT	; Set result counter
		CA	ZERO		; Zero PIPA pulse accumulator
		INDEX	PIPINDEX	; Index by PIPA axis (X, Y, or Z)
		TS	PIPAX		; Clear PIPA pulse counter for test axis
		TS	DATAPL		; Clear data collection register
		TC	CHECKG		; Verify G-switch status
		
; Schedule periodic PIPA sampling task
		INHINT			; Inhibit interrupts during task setup
		CAF	TWO		; 2 centisecond delay (20 milliseconds)
		TC	TWIDDLE		; Schedule task on waitlist
		EBANK=	XSM		; Set erasable bank
		ADRES	PIPATASK	; Address of PIPA sampling task
		TC	ENDOFJOB	; End this job, PIPATASK will execute

; PIPA sampling task - executes every 20ms to check test duration
PIPATASK	EXTEND			; Extend next instruction
		DIM	LENGTHOT	; Decrement test timer by 1 (each call = ~1 second)
		CA	LENGTHOT	; Load remaining test time
		EXTEND			; Extend for branch
		BZMF	STARTPIP	; Branch if time expired (Zero, Minus, or Overflow)
		
; Test still running - reschedule next check
		CAF	BIT10		; 1.024 seconds = 2^10 centiseconds
		TC	TWIDDLE		; Schedule next PIPATASK execution
		EBANK=	XSM
		ADRES	PIPATASK
		
; Test duration complete - start data processing job
STARTPIP	CAF	PRIO20		; Priority 20 for processing job
		TC	FINDVAC		; Find vacant core set for new job
		EBANK=	XSM		; Set erasable bank
		2CADR	PIPJOBB		; Two-word address of processing job

		TC	TASKOVER	; Terminate this task

; PIPA data processing job - analyzes accumulated pulse counts
PIPJOBB		INDEX	NDXCTR		; Check gimbal lock status
		TC	+1		; Normal processing if NDXCTR=0
		TC	EARTHR*		; Special handling if in gimbal lock (NDXCTR=1)
		CA	LENGTHOT	; Verify test actually completed
# Page 376
		EXTEND			; Extend for branch
		BZMF	+2		; Continue if timer expired
		TC	ENDOFJOB	; Otherwise, exit (test not yet done)

; Process PIPA pulse data collected during test
		CA	FIVE		; Request 5 data samples for averaging
		TS	RESULTCT	; Set result counter
		TC	CHECKG		; Check G-switch configuration
		
; Handle sign of accumulated pulse count (check for negative)
		CCS	DATAPL 	+1	; Check high-order word of pulse count
		TC	+4		; Positive - skip sign adjustment
		TC	CCSHOLE		; Zero case (CCS hole)
		CS	DATAPL	+4	; Negative - complement low word
		TS	DATAPL	+4	; Store corrected value
		EXTEND			; Extend for double precision
		DCS	DATAPL		; Load pulse count (double precision, complemented)
		DAS	DATAPL 	+4	; Add to accumulator

; Calculate PIPA performance (convert pulses to g units)
		TC	INTPRET		; Enter interpretive mode for floating-point math
		DLOAD	DSU		; Load and subtract
			DATAPL	+6	; Final pulse count
			DATAPL 	+2	; Initial pulse count
		BPL	CALL		; If positive, continue
			AINGOTN		; Branch destination
			OVERFFIX	; Fix overflow condition
AINGOTN		PDDL	DDV		; Push to stack, load, divide
			DATAPL 	+4	; Accumulated pulse data
		DMPR	RTB		; Multiply, return to basic
			DEC585		# DEC585 HAS BEEN REDEFINED FOR LEM
					; Scaling factor converts pulses to g's
			SGNAGREE	; Ensure sign agreement
		STORE	DSPTEM2		; Store result for display
		EXIT			; Exit interpretive mode
		
; Check if platform was in gimbal lock during test
		CCS	NDXCTR		; If NDXCTR > 0, platform is in gimbal lock
		TC	COAALIGN	# TAKE PLATFORM OUT OF GIMBAL LOCK
		TC	SHOW		; Display PIPA test results to crew

; ============================================================================
; VERTICAL DRIFT TEST
;
; Performs extended 1-hour test measuring IMU gyro drift in vertical axis.
; This long-duration test provides high-precision drift rate measurements
; essential for mission navigation accuracy.
; ============================================================================

VERTDRFT	CA	3990DEC		# ABOUT 1 HOUR VERTICAL DRIFT TEST
		TS	LENGTHOT	; Set test duration (~3990 seconds = 66.5 min)
		INDEX	POSITON		; Index by current test position
		CS	SOUTHDR -2	; Load south drift rate (complemented)
		TS	DRIFTT		; Store as drift torque value
		
; Offset platform slightly to avoid PIPA dead zones (sensor non-linearity)
		CCS	PIPINDEX	# OFFSET PLATFORM TO MISS PIP DEAD-ZONES
		TCF	PON4		# Z-UP IN POS 4 - Z-axis pointing up
PON2		CS	BIT5		# X-UP configuration - X-axis pointing up
		ADS	ERCOMP 	+2	; Add offset to error compensation (Y-axis)
		CA	BIT5		; Positive offset
		ADS	ERCOMP 	+4	; Add offset to error compensation (Z-axis)
		TCF	PON		; Continue to drift test
PON4		CS	BIT5		; Z-UP configuration offset
		ADS	ERCOMP	+2	; Adjust Y-axis compensation
		CA	BIT5		; Positive offset
		ADS	ERCOMP		; Adjust X-axis compensation
PON		TC	EARTHR*		; Apply Earth rate compensation
# Page 377
; Configure Earth rate compensation for vertical drift test
; Only south gyro (vertical axis) receives Earth rate compensation
		CA	ZERO		# ALLOW ONLY SOUTH GYRO EARTH RATE COMPENS
		TS	ERVECTOR	; Clear X-axis Earth rate vector
		TS	ERVECTOR +1	; Clear Y-axis Earth rate vector
		
; Initialize drift torque indices for test data collection
GUESS1		CAF	POSMAX		; Maximum positive value
		TS	TORQNDX		; Initialize torque index (high word)
		TS	TORQNDX +1	; Initialize torque index (low word)
		CA	CDUX		; Load inner gimbal angle (X-axis CDU)
		TS	LOSVEC		; Store as line-of-sight vector reference
		TC	ESTIMS		; Estimate drift and collect measurements

; Display measured drift value to crew on DSKY
VALMIS		CA	DRIFTO		; Load computed drift rate (low word)
		TS	DSPTEM2 +1	; Store for display
		CA	ZERO		; High word = 0
		TS	DSPTEM2		; Store high word
		TC	SHOW		; Display vertical drift test results

; ============================================================================
; TEST TERMINATION ROUTINE
;
; ENDTEST1 - Terminates IMU performance test and returns to normal operations
; Called after test completion or crew-initiated abort
; ============================================================================

ENDTEST1	TC	DOWNFLAG	; Clear IMU-in-use flag
		ADRES	IMUSE		; Address of IMUSE flag
		CS	ZERO		; Load negative zero (all ones)
		TC	NEWMODEA	; Request new major mode from crew
		TC	ENDEXT		; Terminate extended verb operation

# Page 378
; ============================================================================
; OVERFLOW CORRECTION ROUTINE
;
; OVERFFIX - Corrects double-precision overflow in interpretive calculations
; Used when accumulated values exceed maximum representable range
; Adjusts by adding maximum positive value plus one to restore correct result
; ============================================================================

OVERFFIX	DAD	DAD		; Double add (interpretive mode)
			DPPOSMAX	; Add double-precision positive maximum
			ONEDPP		; Add one (double-precision)
		RVQ			; Return via Q register

; ============================================================================
; COARSE ALIGNMENT ROUTINE
;
; COAALIGN - Performs coarse IMU alignment to remove gimbal lock condition
; Called when platform has drifted into gimbal lock during extended tests
; Reorients platform to nominal attitude without fine alignment precision
; ============================================================================

COAALIGN	EXTEND			# COARSE ALIGN SUBROUTINE
		QXCH	ZERONDX		; Save return address in ZERONDX
		CA	ZERO		; Clear desired gimbal angles
		TS	THETAD		; Zero desired X-axis gimbal angle
		TS	THETAD +1	; Zero desired Y-axis gimbal angle
		TS	THETAD +2	; Zero desired Z-axis gimbal angle
		TC	BANKCALL	; Call coarse alignment routine
		CADR	IMUCOARS	; IMU coarse align entry point
		
; Wait for alignment to complete
ALIGNCOA	TC	BANKCALL	; Check IMU status
		CADR	IMUSTALL	; IMU stall check routine
		TC	SOMERR2		; Handle alignment error if detected
		TC	ZERONDX		; Return to caller via saved address

; IMU stall check - waits for IMU to complete current operation
IMUSLLLG	EXTEND			; Extend next instruction
		QXCH	ZERONDX		; Save return address
		TC	ALIGNCOA	; Check IMU status and wait if needed

; Fine IMU alignment - performs high-precision platform orientation
FINIMUDD	EXTEND			; Extend next instruction
		QXCH	ZERONDX		; Save return address
		TC	BANKCALL	; Call fine alignment routine
		CADR	IMUFINE		; IMU fine align entry point
		TC	ALIGNCOA	; Wait for alignment completion

; IMU zero routine - zeros IMU error angles and initializes platform
IMUZERR		EXTEND			; Extend next instruction
		QXCH	ZERONDX		; Save return address
		TC	BANKCALL	; Call IMU zero routine
		CADR	IMUZERO		; IMU zeroing entry point
		TC	ALIGNCOA	; Wait for zeroing completion

; ============================================================================
; PIP PULSE DETECTION AND CAPTURE
;
; CHECKG - Sophisticated PIPA pulse detection routine that captures exact timing
; of accelerometer pulses. Uses interrupt masking to ensure atomic reads of
; pulse counters, avoiding race conditions with PIPA hardware interrupts.
; Essential for high-precision drift and performance measurements.
; ============================================================================

CHECKG		EXTEND			# PIP PULSE CATCHING ROUTINE
		QXCH	QPLACE		; Save return address
		TC	+6		; Skip initialization on re-entry

; Wait for PIPA pulse with interrupt handling
CHECKG1		RELINT			; Re-enable interrupts
		CA	NEWJOB		; Check if new job scheduled
		EXTEND			; Extend for branch
		BZMF	+6		; Branch if no new job pending
		TC	CHANG1		; Handle job change
		INHINT			; Inhibit interrupts for atomic operation
		INDEX	PIPINDEX	; Index to selected PIPA axis
		CS	PIPAX		; Read PIPA pulse count (complemented)
		TS	ZERONDX		; Store first reading
		INHINT			; Re-inhibit (safety - ensure no interrupt)
# Page 379
; Verify PIPA value stable (no pulse during read)
		INDEX	PIPINDEX	; Re-read same PIPA axis
		CA	PIPAX		; Load current pulse count
		AD	ZERONDX		; Add complemented first reading
		EXTEND			; Extend for branch
		BZF	CHECKG1		; If zero, no pulse occurred - retry

; PIPA pulse detected - capture data with precise timestamp
		INDEX	PIPINDEX	; Index to selected PIPA axis
		CA	PIPAX		; Load final pulse count
		INDEX	RESULTCT	; Index to result storage location
		TS	DATAPL		; Store pulse count
		TC	FINETIME	; Get precise timestamp (fine time)
		INDEX	RESULTCT	; Index to timestamp storage
		TS	DATAPL +1	; Store timestamp (A register)
		INDEX	RESULTCT	; Index to extended timestamp
		LXCH	DATAPL +2	; Store extended timestamp (L register)
		RELINT			; Re-enable interrupts
ENDCHKG		TC	QPLACE		; Return to caller

; ============================================================================
; MEMORY ZEROING UTILITY
;
; ZEROING - Clears consecutive memory locations to zero
; Two entry points:
;   ZEROING:  Entry with starting address in A, ZERONDX = count-1
;   ZEROING1: Entry with count in A, starting address already in L
;
; Used throughout IMU tests for initializing data arrays and clearing
; accumulated results. During Apollo 11, this routine prepared clean
; storage for each new test iteration.
;
; TECHNICAL DETAILS:
; - Stores zero at consecutive addresses from starting point
; - Uses indexed addressing for efficient array initialization
; - Returns via Q when all locations cleared
; ============================================================================

ZEROING		TS	L		; Save starting address to L
		TCF	+2		; Jump to ZEROING1 entry logic
ZEROING1	TS	ZERONDX		; Save loop counter (count-1)
		CAF	ZERO		; Load zero value to store
		INDEX	L		; Index to address in L register
		TS	0		; Store zero at indexed address
		INCR	L		; Increment address pointer
		CCS	ZERONDX		; Decrement counter, check remaining
		TCF	ZEROING1	; Loop back if more locations to zero
		TC	Q		; Return when complete

# Page 380
; ============================================================================
; EARTH RATE VECTOR INITIALIZATION
;
; ERTHRVSE - Initializes Earth rotation rate vector for drift compensation
; Computes the Earth rotation rate vector components based on vehicle latitude
; during stationary IMU drift tests. This allows measurement of gyro drift
; separate from Earth's rotation.
;
; TECHNICAL COMPUTATION:
; - Uses current latitude to compute rotation vector components
; - Earth rotation rate (OMEG/MS) scaled for platform coordinates
; - Initializes time mark (TMARK) and error compensation vector (ERCOMP)
; - Vector components: (sin(lat), -cos(lat), 0) * OMEG/MS
;
; Used during: IMU alignment verification and drift rate measurement tests
; ============================================================================

ERTHRVSE	DLOAD	PDDL		; Load zeros, push; load latitude
			SCHZEROS	# PD24 = (SIN             -COS     0)(OMEG/MS)
			LATITUDE	; Current vehicle latitude
		COS	DCOMP		; cos(latitude), double complement (-cos)
		PDDL	SIN		; Push -cos; load latitude, compute sin
			LATITUDE	; sin(latitude)
		VDEF	VXSC		; Define vector, scale by Earth rate
			OMEG/MS		; Earth rotation rate (rad/centisec)
		STORE	ERVECTOR	; Store Earth rate vector
		RTB			; Return to basic, call subroutine
			LOADTIME	; Load current mission time
		STOVL	TMARK		; Store time mark; load zero vector
			SCHZEROS	; Zero vector for initial error
		STORE	ERCOMP		; Store error compensation vector
		RVQ			; Return via Q

; ============================================================================
; EARTH RATE COMPENSATION DURING DRIFT TESTS
;
; EARTHR - Compensates for Earth's rotation during stationary IMU drift tests
; Calculates the angular change since last compensation and applies correction
; pulses to the gyros. This removes Earth rotation from measured drift rates.
;
; During Apollo 11 mission, this routine enabled accurate measurement of IMU
; gyro drift characteristics independent of planetary rotation effects.
;
; TECHNICAL OPERATION:
; - Computes elapsed time since last mark (TEMPTIME - TMARK)
; - Scales time interval by Earth rate vector (ERVECTOR)
; - Transforms to platform coordinates (XSM matrix)
; - Accumulates correction in ERCOMP vector
; - Applies compensation pulses via PULSEIMU routine
;
; EARTHR* - Basic routine wrapper for interpreted EARTHR
; Entry point from basic AGC code, calls interpretive version
; ============================================================================

EARTHR		ITA	RTB		; Save return, load current time
			S2		; Return address storage
			LOADTIME	; Get mission elapsed time
		STORE	TEMPTIME	; Save current time
		DSU	BPL		; Subtract mark time, test positive
			TMARK		; Previous time mark
			ERTHR		; Continue if positive delta
		CALL			; Handle time overflow
			OVERFFIX	; Overflow correction routine
ERTHR		SL	VXSC		; Shift left 9, scale by Earth vector
			9D		; Scale factor for time units
			ERVECTOR	; Earth rotation rate vector
		MXV	VAD		; Transform by matrix, add compensation
			XSM		; Stable member to navigation base
			ERCOMP		; Accumulated error compensation
		STODL	ERCOMP		; Store updated compensation; load time
			TEMPTIME	; Current time value
		STORE	TMARK		; Update time mark for next interval
		AXT,1	RTB		; Set index, return to basic
		ECADR	ERCOMP		; Address of compensation vector
			PULSEIMU	; Apply pulses to IMU gyros
		GOTO			; Return to caller
			S2		; Saved return address

EARTHR*		EXTEND			; Extended instruction follows
		QXCH	QPLACES		; Save return address
		TC	INTPRET		; Enter interpretive mode
		CALL			; Call interpretive routine
			EARTHR		; Earth rate compensation
		EXIT			; Return to basic mode
		TC	IMUSLLLG	; IMU sleep logic check
		TC	QPLACES		; Return to caller

; ============================================================================
; TEST POSITION DISPLAY ROUTINE
;
; SHOW - Displays current test position value on DSKY for crew monitoring
; Used throughout IMU performance tests to display test iteration number or
; position counter, allowing crew to track progress through test sequence.
;
; During Apollo 11, this routine provided real-time feedback to the crew on
; which test iteration was executing, enabling manual intervention if needed.
;
; TECHNICAL OPERATION:
; - Loads POSITON counter into display register
; - Uses verb 06 noun 98 to flash display
; - Waits for crew response:
;   V34: Terminate test (branches to ENDTEST1)
;   V33: Proceed (returns to caller)
;   Otherwise: Refresh display (loop to SHOW1)
; ============================================================================

SHOW		EXTEND			; Extended instruction follows
# Page 381
		QXCH	QPLACE		; Save return address
SHOW1		CA	POSITON		; Load current position/iteration counter
		TS	DSPTEM2 +2	; Store in display register
		CA	VB06N98		; Load verb 06, noun 98
		TC	BANKCALL	; Bank call to display routine
		CADR	GOFLASH		; Flash display, wait for crew input
		TC	ENDTEST1	# V34 - Terminate test
		TC	QPLACE		# V33 - Proceed, return to caller
		TCF	SHOW1		; Other key - refresh display

; ============================================================================
; IMU TEST CONSTANTS AND ADDRESSES
;
; Constants and address pointers used throughout IMU performance test routines
; ============================================================================

3990DEC		DEC	3990		; Constant: 3990 decimal
VB06N98		VN	0698		; Verb 06 Noun 98 - Display test data
VN0641		VN	0641		; Verb 06 Noun 41 - Lat/Azimuth display
DEC17		=	ND1		; Constant: 17 decimal (aliased)
DEC58		DEC	58		; Constant: 58 decimal
OGCPL		ECADR	OGC		; ECADR pointer to OGC data
1SECX		=	1SEC		; One second time constant (aliased)
XNBADR		GENADR	XNB		; Generic address of XNB matrix
XSMADR		GENADR	XSM		; Generic address of XSM matrix
		BLOCK	2		; Switch to memory block 2
		COUNT*	$$/P07		; Program counter for P07

; ============================================================================
; HIGH-PRECISION TIME READING
;
; FINETIME - Reads mission elapsed time with high precision and consistency
; Returns with interrupts inhibited to ensure atomic time capture
; 
; TECHNICAL DETAILS:
; - Reads LOSCALAR (low-order time scalar) with verification
; - Detects and handles scalar rollover during read
; - Reads HISCALAR (high-order time scalar) if needed
; - Returns: A = high scalar, L = low scalar, interrupts inhibited
;
; CRITICAL FOR: Precise PIPA pulse timestamping in IMU performance tests
; ============================================================================

FINETIME	INHINT			# RETURNS WITH INTERRUPT INHIBITED
		EXTEND			; Enable extended instruction
		READ	LOSCALAR	; Read low-order time scalar
		TS	L		; Store in L register
		EXTEND			; Enable extended instruction
		RXOR	LOSCALAR	; XOR with current LOSCALAR
		EXTEND			; Enable extended instruction
		BZF	+4		; If zero, no change - skip re-read
		EXTEND			; Enable extended instruction
		READ	LOSCALAR	; Re-read (scalar changed during read)
		TS	L		; Store updated value in L
 +4		CS	POSMAX		; Load -max positive (detects rollover)
		AD	L		; Add low scalar value
		EXTEND			; Enable extended instruction
		BZF	FINETIME +1	; If zero, rollover - retry entire read
		EXTEND			; Enable extended instruction
		READ	HISCALAR	; Read high-order time scalar
		TC Q		; Return with interrupts inhibited

