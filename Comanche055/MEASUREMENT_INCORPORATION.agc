# Copyright:	Public domain.
# Filename:	MEASUREMENT_INCORPORATION.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1252-1261
# Mod history:	2009-05-14 RSB	Adapted from the Colossus249/ file of the
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

# Page 1252
; ============================================================================
; FILE: MEASUREMENT_INCORPORATION.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Navigation state update incorporating sensor measurements via Kalman
;        filtering. Processes optical sightings, radar data, and ground tracking
;        measurements to refine spacecraft position and velocity knowledge,
;        implementing state covariance propagation and measurement weighting
;        throughout Apollo 11 navigation.
;
; COMMENT-ONLY READERS: This program updated the spacecraft's position knowledge
;        using measurements from sensors and ground stations throughout the mission.
; CODE-ALONG READERS: Study Kalman filter navigation state update, measurement
;        incorporation, covariance propagation, optimal estimation implementation.
; ============================================================================
; ============================================================================
; KALMAN FILTER MEASUREMENT UPDATE - INCORP1 SUBROUTINE
;
; Throughout the Apollo 11 mission, the spacecraft's true position was never
; known exactly. The guidance computer maintained an estimated state vector
; (position and velocity) and continuously refined this estimate by incorporating
; measurements from various sensors: optical star sightings, ground tracking
; stations, and radar. This routine implements the measurement update phase of
; a Kalman filter - the optimal statistical method for combining imperfect
; measurements with uncertain state estimates to produce the best possible
; knowledge of where the spacecraft is and where it's going.
; ============================================================================

#  INCORP1--PERFORMS THE SIX DIMENSIONAL STATE VECTOR DEVIATION FOR POSITI
# ON AND VELOCITY OR THE NINE DIMENSIONAL DEVIATION OF POSITION,VELOCITY,A
# ND RADAR OR LANDMARK BIAS.THE OUTPUT OF THE BVECTOR ROUTINE ALONG WITH T
# HE ERROR TRANSITION MATRIX(W) ARE USED AS INPUT TO THE ROUTINE.THE DEVIA
# TION IS OBTAINED BY COMPUTING AN ESTIMATED TRACKING MEASUREMENT FROM THE
# CURRENT STATE VECTOR AND COMPARING IT WITH AN ACTUAL TRACKING MEASUREMEN
# T AND APPLYING A STATISTICAL WEIGHTING VECTOR.
; INPUTS AND OUTPUTS:
; The routine operates in either 6-dimensional mode (position and velocity only)
; or 9-dimensional mode (position, velocity, plus radar or landmark bias terms
; to account for systematic sensor errors). During translunar and transearth
; navigation, 6-dimensional mode was typical. During lunar orbit operations with
; landmark tracking, 9-dimensional mode incorporated bias estimation.
;
# INPUT
#   DMENFLG = 0 6DIMENSIONAL BVECTOR  1= 9DIMENSIONAL
#          W = ERROR TRANSITION MATRIX 6X6 OR 9X9
;              (W matrix represents how state errors propagate over time)
#   VARIANCE = VARIANCE (SCALAR)
;              (Measurement uncertainty - larger values mean less trust in measurement)
#     DELTAQ = MEASURED DEVIATION(SCALAR)
;              (Difference between actual measurement and predicted measurement)
#    BVECTOR = 6 OR 9 DIMENSIONAL BVECTOR
;              (Measurement sensitivity vector - how measurement changes with state)
#
# OUTPUT
#      DELTAX = STATE VECTOR DEVIATIONS 6 OR 9 DIMENSIONAL
;              (Corrections to apply to current state estimate)
#	   ZI = VECTOR USED FOR THE INCORPORATION 6 OR 9 DIMENSIONAL
;              (Intermediate computation: W-transpose times BVECTOR)
#     GAMMA = SCALAR
;              (Kalman gain denominator - normalization factor)
#     OMEGA = OMEGA WEIGHTING VECTOR 6 OR 9 DIMENSIONAL
;              (Kalman gain vector - determines how much to trust this measurement)
#
# CALLING SEQUENCE
#    L  CALL INCORP1
#
# NORMAL EXIT
#    L+1 OF CALLING SEQUENCE

		BANK	37
		SETLOC	MEASINC
		BANK

		COUNT*	$$/INCOR

		EBANK=	W

; ----------------------------------------------------------------------------
; INCORP1 ENTRY POINT
;
; Save return address and initialize loop indices for computing the ZI vectors.
; The routine processes three vector components (Z1, Z2, Z3) corresponding to
; the three rows of the W-transpose times BVECTOR multiplication.
; ----------------------------------------------------------------------------

INCORP1		STQ
			EGRESS		; Save return address in EGRESS
		AXT,1	SSP
			54D		; Index X1 = 54 (points to W matrix row)
			S1
			18D		# IX1 = 54 	S1= 18
		AXT,2	SSP
			18D		; Index X2 = 18 (points to ZI storage)
			S2
			6		# IX2 = 18	S2=6
; Compute ZI = W-transpose times BVECTOR using matrix-vector multiplication.
; This is the heart of the Kalman filter's measurement sensitivity computation.
; For a 6x6 system, BVECTOR has 3 vector components (BVECTOR(0), BVECTOR(1),
; BVECTOR(2)), each multiplied by corresponding rows of W and accumulated.
;
Z123		VLOAD	MXV*
			BVECTOR		# BVECTOR (0) - first vector component
			W +54D,1	; Multiply by W matrix row (indexed)
		STORE	ZI +18D,2	; Store first partial result
		VLOAD
			BVECTOR +6	# BVECTOR (1) - second vector component
# Page 1253
		MXV*	VAD*
			W +108D,1	; Multiply by next W matrix row
			ZI +18D,2	; Add to previous partial result
		STORE	ZI +18D,2
		VLOAD
			BVECTOR +12D	# BVECTOR (2) - third vector component
		MXV*	VAD*
			W +162D,1	; Multiply by final W matrix row
			ZI +18D,2	# B(0)*W+B(1)*(W+54)+B(2)*(W+108)FIRST PAS
		STORE	ZI +18D,2	# ZI THEN Z2 THEN Z3
; Loop three times to compute Z1, Z2, Z3 (the three components of ZI vector).
		TIX,1
			INCOR1		; Decrement X1, loop if not done
INCOR1		TIX,2	BON
			Z123		# LOOP FOR Z1,Z2,Z3
			DMENFLG		; Check dimension flag
			INCOR1A		; Branch to 9-dimensional processing
; For 6-dimensional mode, zero out bias term components.
		VLOAD
			ZEROVECS	; Load zero vector
		STORE	ZI +12D		; Zero third bias component for 6D mode

; ============================================================================
; GAMMA COMPUTATION: MEASUREMENT COVARIANCE
;
; Compute GAMMA = |ZI|^2 + |Z2|^2 + |Z3|^2 + VARIANCE
; This scalar represents the total measurement uncertainty, combining both
; the propagated state uncertainty (ZI magnitude squared) and the inherent
; measurement noise (VARIANCE). During Apollo 11, VARIANCE values were carefully
; chosen based on sensor characteristics: optical sightings had different
; uncertainty than radar measurements or ground tracking data.
; ============================================================================

INCOR1A		SETPD	VLOAD
			0		; Initialize push-down stack pointer
			ZI		; Load first ZI vector
		VSQ	RTB		; Square each component (vector squared)
			TPMODE		; Round to triple precision
		PDVL	VSQ		; Push result, load next vector
			ZI +6		; Second ZI vector
		RTB	TAD		; Round and add to previous result
			TPMODE
		PDVL	VSQ		; Continue with third vector
			ZI +12D
		RTB	TAD		; Round and accumulate
			TPMODE
		TAD	AXT,2		; Add measurement variance
			VARIANCE	; Sensor-specific uncertainty
			0
		STORE	TRIPA		# ZI*2 + Z2*2 + Z3*2 + VARIANCE
; Normalize VARIANCE to prevent overflow in subsequent square root computation.
; The AGC's fixed-point arithmetic requires careful scaling management.
		TLOAD	BOV
			VARIANCE	# CLEAR OVFIND flag from previous operations
			+1
		STORE	TEMPVAR		# TEMP STORAGE FOR VARIANCE
		BZE			; Skip scaling if variance already zero
			INCOR1C

; Iteratively scale VARIANCE down if needed for numerical stability.
INCOR1B		SL2	BOV		; Shift left 2 bits (multiply by 4)
			INCOR1C		; Branch if overflow occurs
		STORE	TEMPVAR
		INCR,2	GOTO		; Increment scale counter
		DEC	1
			INCOR1B		; Loop to continue scaling

; Compute normalized GAMMA = sqrt(TRIPA * TEMPVAR) with proper scaling.
; GAMMA becomes the denominator in Kalman gain: K = (W * BVECTOR) / GAMMA
INCOR1C		TLOAD	ROUND
			TRIPA		; Load ZI magnitude squared sum
# Page 1254
		DMP	SQRT		; Multiply by scaled variance, take square root
			TEMPVAR		; Result is normalized GAMMA
		SL*	TAD		; Scale back and add
			0,2		; Apply accumulated scaling factor
			TRIPA
		NORM	INCR,2		; Normalize result for maximum precision
			X2		; X2 receives normalization exponent
		DEC	-2		; Adjustment constant
		SXA,2	AXT,2
			NORMGAM		# NORMALIZATION COUNT -2 FOR GAMMA
			162D

; Finalize GAMMA computation by dividing by DP1/4TH (0.25 in double precision).
; This scaling adjustment ensures GAMMA has correct units for measurement weighting.
		BDDV	SETPD		; Double precision divide, set pushdown pointer
			DP1/4TH		; Divide by 0.25 (multiply by 4)
			0		; Reset pushdown list pointer to base
		STORE	GAMMA		; Final GAMMA stored for use in state update

; ============================================================================
; MEASUREMENT RESIDUAL COMPUTATION: DELTAQ/A
;
; Compute the normalized measurement residual (DELTAQ/A) where:
; - DELTAQ = measured value minus predicted value (from current state)
; - A = normalized magnitude from TRIPA
; This ratio determines how much the measurement deviates from prediction,
; weighted by the measurement geometry. During Apollo 11 navigation, this
; residual indicated how much to adjust position/velocity estimates based on:
; - Star sighting angles from optical telescope
; - Doppler frequency shifts from ground tracking stations  
; - Landmark position measurements during lunar orbit
; ============================================================================

		TLOAD	NORM		; Load and normalize TRIPA
			TRIPA		; ZI magnitude squared sum
			X1		; Store normalization exponent in X1
		DLOAD	PDDL		# PD 0-1 = NORM (A)
			MPAC		; Load normalized result
			DELTAQ		; Load measurement residual
		NORM			; Normalize DELTAQ
			S1		; Store normalization exponent in S1
		XSU,1	SR1		; Compute exponent difference, shift right 1
			S1		; Subtract normalization factors
		DDV	PUSH		# PD 0-1 = DELTAQ/A (weighted residual)
		GOTO			; Branch to update computation
			NEWZCOMP
   -3		SSP
			S2
			54D

; ============================================================================
; INCOR2: OMEGA VECTOR COMPUTATION
;
; Compute the 3-dimensional OMEGA weighting vectors (OMEGA1, OMEGA2, OMEGA3)
; by multiplying ZI vectors with corresponding rows of error transition matrix W.
; OMEGA represents the Kalman gain vectors that weight how much each measurement
; residual affects each component of the state vector (position, velocity, bias).
;
; During Apollo 11 navigation, these vectors determined:
; - How much a star sighting adjusted position vs velocity estimates
; - How ground tracking Doppler measurements refined velocity knowledge
; - How landmark observations during lunar orbit corrected position drift
;
; The computation: OMEGA_i = ZI_0*W[i,0] + ZI_1*W[i,1] + ZI_2*W[i,2]
; for i = 0,1,2 (corresponding to OMEGA1, OMEGA2, OMEGA3)
; ============================================================================

INCOR2		VLOAD	VXM*		# COMPUT OMEGA1,2,3
			ZI		; Load first ZI vector
			W +162D,2	; Multiply by W matrix row (indexed by S2)
		PUSH	VLOAD		; Push result, load next ZI
			ZI +6		; Second ZI vector
		VXM*	VAD		; Multiply by W matrix row and add
			W +180D,2	; Next row offset
		PUSH	VLOAD		; Push result, load third ZI
			ZI +12D		; Third ZI vector
		VXM*	VAD		; Multiply by W matrix row and add
			W +198D,2	; Final row offset
		PUSH	TIX,2		# PD 2-7=OMEGA1,8-13=OMEGA2,14-19=OMEGA3
			INCOR2		; Loop for all three OMEGA vectors
		VLOAD	STADR		; Load computed OMEGA3 from pushdown
		STORE	OMEGA +12D	; Store OMEGA3
		VLOAD	STADR		; Load OMEGA2
		STORE	OMEGA +6	; Store OMEGA2
		VLOAD	STADR		; Load OMEGA1
		STORE	OMEGA		; Store OMEGA1
# Page 1255
; Check DMENFLG to determine if processing 6-dimensional (position/velocity)
; or 9-dimensional (position/velocity/bias) state vector.
		BON	VLOAD		; Branch on DMENFLG
			DMENFLG		; 0=6-dim, 1=9-dim (includes radar/landmark bias)
			INCOR2AB	; Skip to computation if 9-dimensional
			ZEROVECS	; Load zero vector for 6-dimensional case
		STORE	OMEGA +12D	; Clear unused OMEGA3 for 6-dim navigation

; ============================================================================
; INCOR3: STATE VECTOR DEVIATION COMPUTATION (DELTAX)
;
; Compute final state vector deviations: DELTAX = OMEGA * (DELTAQ/A)
; This is the Kalman filter update equation, scaling the gain vectors OMEGA
; by the normalized measurement residual to produce position, velocity, and
; (if 9-dimensional) bias corrections.
;
; During Apollo 11 mission, these corrections updated:
; - Spacecraft position relative to Moon/Earth (feet, scaled by 2^29)
; - Spacecraft velocity (feet/second, scaled by 2^7)
; - Radar or landmark measurement biases (if applicable)
;
; The VSL* operation applies appropriate scaling based on normalization
; factors accumulated during GAMMA computation.
; ============================================================================

INCOR2AB	AXT,2	SSP		; Initialize loop counters
			18D		; X2 = 18 (decrement by 6 each iteration)
			S2		; S2 = loop control
			6		; Process 3 vectors (or 2 for 6-dim)

INCOR3		VLOAD*			; Load OMEGA vector (indexed)
			OMEGA +18D,2	; OMEGA1, OMEGA2, or OMEGA3
		VXSC	VSL*		; Vector scale by scalar, shift left
			0		# DELTAQ/A (measurement residual ratio)
			0,1		; Apply accumulated normalization scaling
		STORE	DELTAX +18D,2	; Store resulting state deviation
		TIX,2	VLOAD		; Decrement X2, loop if not done
			INCOR3		; Continue for all vectors
			DELTAX +6	; Load middle DELTAX vector for adjustment
		VSL3			; Shift left 3 bits for final scaling
		STORE	DELTAX +6	; Store adjusted DELTAX2
		GOTO			; Return to caller
			EGRESS		; Exit via stored return address

# Page 1256
#  INCORP2 -INCORPORATES THE COMPUTED STATE VECTOR DEVIATIONS INTO THE
# ESTIMATED STATE VECTOR. THE STATE VECTOR UPDATED MAY BE FOR EITHER THE
# LEM OR THE CSM.DETERMINED BY FLAG VEHUPFLG.(ZERO = LEM) (1 = CSM)
# INPUT
#    PERMANENT STATE VECTOR FOR EITHER THE LEM OR CSM
#    VEHUPFLG = UPDATE VEHICLE C=LEM  1=CSM
#    W = 	ERROR TRANSITION MATRIX
#    DELTAX  = 	COMPUTED STATE VECTOR DEVIATIONS
#    DMENFLG = 	SIZE OF W MATRIX (ZERO=6X6) (1=9X9)
#    GAMMA   = 	SCALAR FOR INCORPORATION
#    ZI      = 	VECTOR USED IN INCORPORATION
#    OMEGA   = 	WEIGHTING VECTOR
#
# OUTPUT
#    UPDATED PERMANENT STATE VECTOR
#
# CALLING SEQUENCE
#    L	 CALL INCORP2
#
# NORMAL EXIT
#    L+1 OF CALLING SEQUENCE
#

; ============================================================================
; INCORP2: STATE VECTOR INCORPORATION AND COVARIANCE UPDATE
;
; This subroutine performs the final step of Kalman filter navigation update:
; incorporating the computed state vector deviations (DELTAX) into the current
; spacecraft state vector and updating the error covariance matrix (W).
;
; COMMENT-ONLY READERS: After computing corrections to the spacecraft's position
;        and velocity, this routine applies those adjustments to the navigation
;        system. Throughout Apollo 11's mission, these updates continuously
;        refined knowledge of where the spacecraft was and where it was going,
;        using measurements from stars, radar, and ground tracking stations.
;
; CODE-ALONG READERS: Implements the Kalman filter covariance update equation:
;        W(new) = W(old) - GAMMA * OMEGA * ZI^T
;        Then incorporates DELTAX into the permanent state vector (RCV, VCV).
;        Updates are propagated through coordinate rectification (RECTIFY) and
;        phase change routines (GRP2PC) before storage for downlink telemetry.
;        Handles both CSM and LEM state vectors based on VEHUPFLG.
; ============================================================================

		SETLOC	MEASINC1
		BANK

		COUNT*	$$/INCOR

INCORP2		STQ	CALL		; Save return, initialize integration
			EGRESS		; Return address for final exit
			INTSTALL	; Set up integration parameters

; Phase 1: Compute GAMMA-scaled Kalman gain vectors (OMEGAM = GAMMA * OMEGA)
; These scaled gain vectors will be used to update the error covariance matrix W.
; The GAMMA scalar normalizes the update based on measurement variance and
; innovation magnitude, ensuring numerical stability during matrix updates.

		VLOAD	VXSC		# CALC. GAMMA * OMEGA1,2,3
			OMEGA		; Load first Kalman gain vector
			GAMMA		; Scale by GAMMA normalization factor
		STOVL	OMEGAM1		; Store GAMMA*OMEGA1, load OMEGA2
			OMEGA +6	; Second Kalman gain vector
		VXSC			; Scale by GAMMA
			GAMMA
		STOVL	OMEGAM2		; Store GAMMA*OMEGA2, load OMEGA3
			OMEGA +12D	; Third Kalman gain vector (or zero if 6-dim)
		VXSC			; Scale by GAMMA
			GAMMA
		STORE	OMEGAM3		; Store GAMMA*OMEGA3

; Initialize index registers for W matrix and ZI vector processing.
; The W error transition matrix is 6x6 (36 elements) or 9x9 (81 elements),
; stored as a linear array. Index arithmetic manages the 2D matrix addressing.

		EXIT			; Return to basic AGC instructions
		CAF	54DD		# INITIAL IX 1 SETTING FOR W MATRIX
		TS	WIXA		; W matrix row index (start at row 9, offset 54)
		TS	WIXB		; W matrix column index backup
		CAF	ZERO		; Initialize to zero
		TS	ZIXA		# INITIAL IX 2 SETTING FOR Z COMPONENT
		TS	ZIXB		; ZI vector index (start at ZI component 0)

; ============================================================================
; TRANSITION: From initialization to W matrix update loop
;
; The error covariance matrix W represents uncertainty in the navigation state.
; This loop implements the Kalman filter covariance update equation:
;    W(new) = W(old) - GAMMA * OMEGA * ZI^T
; reducing uncertainty after incorporating a new measurement. The 6x6 or 9x9
; matrix is processed in 3x3 partitions for efficiency and numerical stability.
;
; During Apollo 11, each optical sighting or radar measurement reduced the
; uncertainty ellipsoid around the spacecraft's estimated position, improving
; navigation accuracy for midcourse corrections and lunar orbit insertion.
; ============================================================================

FAZA		TC	PHASCHNG	; Restart protection checkpoint
# Page 1257
		OCT	04022		; Phase change code
		TC	UPFLAG		; Set reintegration flag
		ADRES	REINTFLG	; Indicates state vector reintegration needed

FAZA1		CA	WIXB		# START FIRST PHASE OF INCORP2
		TS	WIXA		#  TO UPDATE 6 OR 9 DIM. W MATRIX IN TEMP
		CA	ZIXB		; Restore ZI vector index
		TS	ZIXA		; Set working index
		TC	INTPRET		; Enter interpretive mode for matrix ops

; Phase 2: Update upper 3x3 partition of W matrix (rows 1-3)
; Computes: W_upper = W_upper - (GAMMA * OMEGA1) * ZI^T
; This updates the position-related covariance terms (how position uncertainty
; correlates with position, velocity, and bias estimates).

		LXA,1	LXA,2		; Load index registers
			WIXA		; X1 = W matrix row offset
			ZIXA		; X2 = ZI vector component index
		SSP	DLOAD*		; Set loop counter, load ZI component
			S1		; S1 = 6 (process 3 double-precision pairs)
			6
			ZI,2		; Load ZI[X2] (indexed ZI component)
		DCOMP	NORM		# CALC UPPER 3X9 PARTITION OF W MATRIX
			S2		; Negate ZI, normalize (shift count -> S2)
		VXSC	XCHX,2		; Scale OMEGA1 by normalized -ZI
			OMEGAM1		; GAMMA*OMEGA1 vector
			S2		; Exchange normalization shift with X2
		LXC,2	XAD,2		; Load complement of X2, add to X2
			X2		; Get current shift accumulation
			NORMGAM		; Add GAMMA normalization factor
		VSL*	XCHX,2		; Shift result by accumulated scaling
			0,2		; Shift count in X2
			S2		; Restore S2, save shift in X2
		VAD*			; Add to existing W matrix partition
			W +54D,1	; W matrix element (indexed by X1)
		STORE	HOLDW		; Store updated partition temporarily

; Phase 3: Update middle 3x3 partition of W matrix (rows 4-6)
; Computes: W_middle = W_middle - (GAMMA * OMEGA2) * ZI^T
; This updates the velocity-related covariance terms (how velocity uncertainty
; correlates with position, velocity, and bias estimates).

		DLOAD*	DCOMP		# CALC MIDDLE 3X9 PARTITION OF W MATRIX
			ZI,2		; Reload ZI[X2] component
		NORM	VXSC		; Negate, normalize, scale OMEGA2
			S2		; Normalization shift count
			OMEGAM2		; GAMMA*OMEGA2 vector (velocity gain)
		XCHX,2	LXC,2		; Exchange S2 with X2, load complement
			S2
			X2		; Get shift accumulation
		XAD,2	VSL*		; Add GAMMA normalization, shift result
			NORMGAM		; GAMMA scaling factor
			0,2		; Accumulated shift count
		XCHX,2	VAD*		; Exchange back, add to W matrix
			S2
			W +108D,1	; W matrix middle partition (offset +108)
		STORE	HOLDW +6	; Store updated middle partition

; Phase 4: Update lower 3x3 partition of W matrix (rows 7-9)
; Only performed for 9-dimensional state (DMENFLG=1), when estimating
; radar or landmark bias in addition to position and velocity.
; Computes: W_lower = W_lower - (GAMMA * OMEGA3) * ZI^T
; This updates the bias-related covariance terms.

		BOFF			; Branch OFF if 6-dimensional
			DMENFLG		# BRANCH IF 6 DIMENSIONAL
			FAZB		; Skip lower partition for 6-dim case
		DLOAD*	DCOMP		# CALC LOWER 3X9 PARTITION OF W MATRIX
			ZI,2		; Load ZI[X2] for 9-dim case
		NORM	VXSC		; Negate ZI, normalize, scale OMEGA3
# Page 1258
			S2		; Normalization shift count
			OMEGAM3		; GAMMA*OMEGA3 vector (bias gain)
		XCHX,2	LXC,2		; Exchange S2 with X2, load complement
			S2
			X2		; Get shift accumulation
		XAD,2	VSL*		; Add GAMMA normalization, shift result
			NORMGAM		; GAMMA scaling factor
			0,2		; Accumulated shift count
		XCHX,2	VAD*		; Exchange back, add to W matrix
			S2
			W +162D,1	; W matrix lower partition (offset +162)
		STORE	HOLDW +12D	; Store updated lower partition (9-dim only)

; Call GRP2PC to determine if more W matrix columns need processing.
; This routine manages the loop through all 3 columns of each 3x3 partition,
; processing the entire 6x6 or 9x9 covariance matrix.

FAZB		CALL			; Call loop control subroutine
			GRP2PC		; Group 2 phase control
		EXIT			; Return to basic instructions

; ============================================================================
; TRANSITION: From W matrix column update to column transfer
;
; After computing updated covariance values in temporary storage (HOLDW),
; this phase transfers the results back to the permanent W matrix locations.
; The index arithmetic advances to the next column, processing all matrix
; elements systematically.
; ============================================================================

FAZB1		CA	WIXA		# START 2ND PHASE OF INCORP2 TO TRANSFER
		AD	6DD		#     TEMP REG TO PERM W MATRIX
		TS	WIXB		; W matrix column index + 6 (next column)
		CA	ZIXA		; Get ZI index
		AD	MINUS2		; Decrement by 2 (next ZI component)
		TS	ZIXB		; Store updated ZI index
		TC	INTPRET		; Enter interpretive mode for matrix transfer

; Transfer updated W matrix partitions from temporary storage (HOLDW) to
; permanent W matrix locations. The covariance matrix W is stored as a linear
; array; index arithmetic (X1) manages the column addressing within each 3x3
; partition. This phase executes once per column for all three rows.

		LXA,1	SSP		; Load index register X1, set S1
			WIXA		; X1 = W matrix column index
			S1		; S1 = loop counter
			6		; Process 6 words (2 columns * 3 words)
		VLOAD			; Load upper partition (3 words)
			HOLDW		; Updated upper 3x3 partition
		STORE	W +54D,1	; Store to W matrix upper section
		VLOAD			; Load middle partition (3 words)
			HOLDW +6	; Updated middle 3x3 partition
		STORE	W +108D,1	; Store to W matrix middle section
		BOFF	VLOAD		; Branch OFF if 6-dimensional
			DMENFLG		; Check dimensionality flag
			FAZB5		; Skip lower partition transfer for 6-dim
			HOLDW +12D	; Load lower 3x3 partition (9-dim only)
		STORE	W +162D,1	; Store to W matrix lower section

; Loop control: Decrement X1 counter, continue or exit W matrix update phase.
; When all columns processed, proceed to state vector incorporation phase.

FAZB2		TIX,1	GOTO		; Test and decrement index X1
			+2		; Branch to next instruction if not done
			FAZC		# DONE WITH W MATRIX. UPDATE STATE VECTOR
		RTB			; Return to basic instructions
			FAZA		; Loop back to process next column

; For 6-dimensional case, check if all columns processed before proceeding.
; ZIXB index tracks ZI components; when ZIXB+12 reaches zero, all done.

FAZB5		SLOAD	DAD		; Load and add
			ZIXB		; Current ZI index
			12DD		; Add 12 to check completion
		BHIZ	GOTO		; Branch if result is zero
			FAZC		; All columns done, update state vector
			FAZB2		; Not done, continue loop
; ============================================================================
; STATE VECTOR INCORPORATION PHASE (INCORP2 Phase 3)
;
; With the W matrix updated to reflect measurement uncertainty, now apply
; the computed state vector corrections (DELTAX) to the spacecraft's actual
; navigation state. This phase updates position, velocity, and (for 9-dim)
; radar/landmark bias parameters with the optimal Kalman filter correction.
;
; COMMENT-ONLY READERS: The computer now applies its calculated corrections
; to the spacecraft's position and velocity knowledge, refining navigation
; accuracy based on sensor measurements.
; CODE-ALONG READERS: State vector incorporation using DELTAX computed from
; Kalman gain. Handles both 6-dim (position/velocity) and 9-dim (includes
; bias terms) state vectors with overflow detection.
; ============================================================================

FAZC		CALL
			GRP2PC		; Phase change for state update
# Page 1259
		VLOAD	VAD		# START 3RD PHASE OF INCORP2
			X789		# 7TH,8TH,9TH,COMPONENT OF STATE VECTOR
			DELTAX +12D	# INCORPORATION FOR X789
		STORE	TX789		; Store 9-dim bias correction temporarily

; Vehicle-specific state vector update: CSM (Command/Service Module) state
; vectors are stored in different memory locations than LM (Lunar Module)
; state vectors. VEHUPFLG determines which vehicle's navigation state to update.

		BON	RTB		; Branch ON if updating CSM
			VEHUPFLG	; Vehicle update flag (ON=CSM, OFF=LM)
			DOCSM		; Update CSM state vector
			MOVEPLEM	; Move LM state vector to working memory

; Apply position and velocity corrections with overflow detection.
; TDELTAV and TNUV are incremental correction accumulators; if adding DELTAX
; causes overflow, corrections are applied directly to RCV/VCV base vectors.
; Scaling differs for Moon orbit (B27/B5) vs Earth orbit (B29/B7) due to
; different distance magnitudes.

FAZAB		BOVB	AXT,2		; Branch on overflow, set X2=0
			TCDANZIG	; Return to Danzig error handler on overflow
			0		; X2=0 for Earth orbit scaling
		BOFF	AXT,2		; Branch OFF if Earth-centered
			MOONTHIS	; Moon sphere of influence flag
			+2		; Skip to Earth orbit processing
			2		; X2=2 for Moon orbit scaling

; Apply position correction (DELTAX) to incremental accumulator (TDELTAV).
; Position scaled B27 for Moon orbit, B29 for Earth orbit. Shift right
; 7 bits for Moon (0-7 via X2=2), 5 bits for Earth (0-5 via X2=0) to
; match TDELTAV scaling. If overflow, apply correction directly to base RCV.

		VLOAD	VSR*		; Load position correction vector
			DELTAX		# B27 IF MOON ORBIT, B29 IF EARTH
			0 -7,2		; Shift right 7 (Moon) or 5 (Earth)
		VAD	BOV		; Add to incremental correction
			TDELTAV		; Accumulated position correction
			FAZAB1		; Overflow: apply directly to RCV
		STOVL	TDELTAV		; Store updated incremental correction

; Apply velocity correction (DELTAX+6) to incremental accumulator (TNUV).
; Velocity scaled B5 for Moon orbit, B7 for Earth orbit. Shift right
; 4 bits for Moon, 2 bits for Earth to match TNUV scaling. If overflow,
; apply correction directly to base VCV.

			DELTAX +6	# B5 IF MOON ORBIT, B7 IF EARTH
		VSR*	VAD		; Shift right and add
			0 -4,2		; Shift right 4 (Moon) or 2 (Earth)
			TNUV		; Accumulated velocity correction
		BOV			; Branch on overflow
			FAZAB2		; Overflow: apply directly to VCV
		STCALL	TNUV		; Store updated velocity correction
			FAZAB3		; Continue to phase change

; Overflow handlers: When incremental correction accumulators (TDELTAV, TNUV)
; overflow, apply the correction directly to the base state vectors (RCV, VCV).
; This ensures navigation state always reflects best estimate even when
; corrections exceed accumulator capacity.

FAZAB1		VLOAD	VAD		; Position overflow handler
			RCV		; Base position vector
			DELTAX		; Position correction
		STORE	RCV		; Update base position directly

FAZAB2		VLOAD	VAD		; Velocity overflow handler
			VCV		; Base velocity vector
			DELTAX +6	; Velocity correction
		STORE	VCV		; Update base velocity directly

; After state vector update, call RECTIFY to ensure navigation state validity.
; RECTIFY verifies conic orbit parameters remain within valid ranges and
; adjusts epoch if necessary. PBODY stores current planetary body index for
; rectification reference frame (Earth=0, Moon=2).

		SXA,2	CALL		; Store X2 index, call subroutine
			PBODY		; Planet body flag for RECTIFY
			RECTIFY		; Validate and adjust state vector

; Finalization: Store updated state vector for downlink telemetry, apply
; 9-dimensional bias correction if needed, and exit INCORP2 routine.

FAZAB3		CALL
			GRP2PC		; Phase change after rectification
		BON	RTB		; Vehicle-specific state vector storage
			VEHUPFLG	; Check which vehicle updated
			DOCSM1		; CSM path: move and store
			MOVEALEM	; LM path: move to permanent storage
		CALL
			SVDWN2		# STORE DOWNLINK STATE VECTOR

FAZAB4		CALL
# Page 1260
			GRP2PC		# PHASE CHANGE

; For 9-dimensional incorporation, apply bias correction (radar or landmark
; bias) stored in TX789 to X789 state vector component.

		BOFF	VLOAD		; Branch OFF if 6-dimensional
			DMENFLG		; Dimensionality flag
			FAZAB5		# 6 DIMENSIONAL - skip bias update
			TX789		# 9 DIMENSIONAL - load bias correction
		STORE	X789		; Apply bias correction to state

; Exit INCORP2: Restore return address (EGRESS) and wake up waiting tasks.
; INTWAKE notifies mission programs that navigation update is complete.

FAZAB5		LXA,1	SXA,1		; Load and store index
			EGRESS		; Restore return address
			QPRET		; Store in Q return register
		EXIT			; Exit interpreter mode
		TC	POSTJUMP	# EXIT
		CADR	INTWAKE		; Wake up tasks waiting on update

; Vehicle-specific state vector handling routines for CSM (Command/Service
; Module). CSM state vectors stored in different memory locations than LM.

DOCSM		RTB	GOTO		; CSM path entry point
			MOVEPCSM	; Move CSM state vector to working area
			FAZAB		; Continue with correction application

DOCSM1		RTB	CALL		; CSM finalization entry point
			MOVEACSM	; Move working state back to CSM storage
			SVDWN1		# STORE DOWNLINK STATE VECTOR
		GOTO
			FAZAB4		; Continue to exit sequence
ZEROD		=	ZEROVECS
54DD		DEC	54
6DD		DEC	-6
12DD		DEC	12
		SETLOC	MEASINC2
		BANK
		COUNT*	$$/INCOR

; ============================================================================
; NEWZCOMP - ZI VECTOR NORMALIZATION SUBROUTINE
;
; This routine finds the largest magnitude component among ZI, ZI+6, and
; ZI+12D vectors and normalizes all three vectors by left-shifting them
; to prevent overflow/underflow during subsequent matrix operations.
; The shift count is applied to NORMGAM to maintain scale consistency
; throughout the incorporation calculation.
;
; Normalization is critical for numerical stability when ZI component
; magnitudes differ by orders of magnitude (e.g., position vs. bias terms).
; During Apollo 11, this ensured accurate navigation updates from optical
; sightings and radar measurements across widely varying measurement scales.
; ============================================================================

NEWZCOMP	VLOAD	ABVAL		; Load ZI first component
			ZI		; Position component (3 words)
		STOVL	NORMZI		; Store magnitude, load next
			ZI +6		; Velocity component (3 words)
		ABVAL	PUSH		; Compute magnitude, push to stack
		DSU	BMN		; Subtract NORMZI, branch if negative
			NORMZI		; Compare |ZI+6| with |ZI|
			+3		; Skip next 3 if |ZI| larger

; If |ZI+6| is larger than |ZI|, update NORMZI with the new maximum.

		DLOAD	STADR		; Load stack value (|ZI+6|)
		STORE	NORMZI		; Store as new maximum magnitude

; Now compare third component |ZI+12D| against current maximum.
; For 6-dimensional case, ZI+12D is zero; for 9-dimensional, contains bias term.

		VLOAD	ABVAL		; Load third component
			ZI +12D		; Bias component (3 words, 9-dim only)
		PUSH	DSU		; Push magnitude, subtract max
			NORMZI		; Compare |ZI+12D| with current max
		BMN	DLOAD		; Branch if |ZI+12D| smaller
			NEWZCMP1	; Continue with current max
		STADR			; Load |ZI+12D| from stack
		STCALL	NORMZI		# LARGEST ABVAL - store and proceed
			NEWZCMP1	; Continue to normalization

		SETLOC	MEASINC3
		BANK
# Page 1261

; ============================================================================
; NEWZCMP1 - VECTOR NORMALIZATION EXECUTION
;
; With maximum magnitude identified in NORMZI, compute the left-shift count
; needed to normalize all ZI vectors. The NORM instruction determines how
; many bits to shift left for optimal precision without overflow. Apply
; this shift to all three ZI components (position, velocity, bias).
; ============================================================================

NEWZCMP1		DLOAD	SXA,1		; Load maximum magnitude
			NORMZI		; Largest |ZI| component
			NORMZI		# SAVE X1 - store index for later
		NORM	INCR,1		; Normalize, compute shift count
			X1		; X1 receives shift count
		DEC	2		; Increment X1 by 2 for adjustment

; Apply computed shift to all three ZI vector components using VSL* 
; (Variable Shift Left, indexed). Shift count in X1 ensures all vectors
; scaled identically, preserving relative magnitudes.

		VLOAD	VSL*		; Load first component
			ZI		; Position component
			0,1		; Shift left by X1 bits
		STOVL	ZI		; Store normalized ZI
			ZI +6		; Load second component
		VSL*			; Shift left by X1 bits
			0,1		; Same shift count
		STOVL	ZI +6		; Store normalized ZI+6
			ZI +12D		; Load third component
		VSL*	SXA,1		; Shift left, store X1
			0,1		; Same shift count
			NORMZI +1	# SAVE SHIFT - preserve for scale adjustment
		STORE	ZI +12D		; Store normalized ZI+12D

; Adjust NORMGAM scale factor to compensate for ZI vector normalization.
; NORMGAM tracks the cumulative scaling applied during GAMMA computation.
; Subtracting the ZI shift count twice maintains mathematical equivalence:
; GAMMA * ZI scaled correctly despite both being independently normalized.
;
; This scale adjustment is crucial for correct state vector deviation
; computation (DELTAX = GAMMA * ZI), ensuring navigation updates have
; proper magnitude regardless of ZI normalization for numerical stability.

		LXA,1	XSU,1		; Load NORMGAM index
			NORMGAM		; Current GAMMA scale factor
			NORMZI +1	; Subtract ZI shift count (first time)
		XSU,1			; Subtract again (twice total)
			NORMZI +1	; Same shift count
		SXA,1	LXC,1		; Store adjusted NORMGAM
			NORMGAM		; Save updated scale factor
			NORMZI +1	; Load shift count into C register
		XAD,1	SETPD		; Add to X1
			NORMZI		; Add original NORMZI value
			2D		; Set pushdown pointer to 2
		GOTO			; Return to continue INCOR2
			INCOR2 -3	; Re-enter at adjusted location

; NORMZI temporary storage location in erasable memory (address 36 octal).

NORMZI		=	36D
