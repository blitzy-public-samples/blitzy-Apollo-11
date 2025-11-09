# Copyright:	Public domain.
# Filename:	MEASUREMENT_INCORPORATION.agc
# Purpose:	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
#
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim DOT lawton AT gmail DOT com>
# Website:	www.ibiblio.org/apollo.
# Pages:	1149-1158
# Mod history:	2009-05-28 JL	Updated from page images.
#		2011-01-06 JL	Fixed pseudo-label indentation.
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
#    Assemble revision 001 of AGC program LMY99 by NASA 2021112-061
#    16:27 JULY 14, 1969

; ============================================================================
; FILE: MEASUREMENT_INCORPORATION.agc
; MODULE: Navigation State Update System
; MISSION PHASE: lunar-orbit/descent/landing/ascent/rendezvous/trans-earth
;
; TL;DR: Implements Kalman filtering to incorporate sensor measurements
;        (landing radar, rendezvous radar, IMU, ground tracking) into the
;        navigation state vector. Computes optimal statistical weighting of
;        measurements, updates position and velocity estimates, and maintains
;        error covariance matrix for navigation accuracy assessment throughout
;        all mission phases requiring precision navigation.
;
; COMMENT-ONLY READERS: This code continuously refines the spacecraft's
;        knowledge of its position and velocity by combining sensor data
;        with predictions, ensuring accurate navigation for lunar operations.
; CODE-ALONG READERS: Study the Kalman filter implementation including
;        measurement residual computation, statistical weighting (BVECTOR),
;        and state/covariance update mathematics across 6 or 9 dimensions.
; ============================================================================

# Page 1149
; ============================================================================
; INCORP1 -- KALMAN FILTER MEASUREMENT UPDATE
;
; This is the heart of the navigation system's measurement incorporation.
; The Lunar Module's position and velocity are continuously estimated by
; integrating the equations of motion. However, sensor measurements (radar
; altimeter, rendezvous radar, ground tracking) provide actual observations
; that differ from predictions due to navigation errors. This routine
; computes the optimal correction to apply to the estimated state.
;
; During lunar descent, this code incorporates landing radar altitude and
; velocity measurements to refine the LM's trajectory knowledge. During
; rendezvous, it processes radar range and range-rate data to track the
; Command Module. The Kalman filter mathematics ensure measurements are
; weighted according to their statistical reliability.
; ============================================================================
# INCORP1 -- PERFORMS THE SIX DIMENSIONAL STATE VECTOR DEVIATION FOR POSITION
# AND VELOCITY OR THE NINE-DIMENSIONAL DEVIATION OF POSITION, VELOCITY, AND
# RADAR OR LANDMARK BIAS. THE OUTPUT OF THE BVECTOR ROUTINE ALONG WITH THE
# ERROR TRANSITION MATRIX (W) ARE USED AS INPU TO THE ROUTINE. THE DEVIATION
# IS OBTAINED BY COMPUTING AN ESTIMATED TRACKING MEASUREMENT FROM THE
# CURRENT STATE VECTOR AND COMPARING IT WITH AN ACTUAL TRACKING MEASUREMENT
# AND APPLYING A STATISTICAL WEIGHTING VECTOR.
#
# INPUT
#	 DMENFLG = 0 (6-DIMENSIONAL BVECTOR), =1 (9-DIMENSIONAL)
#	       W = ERROR TRANSITION MATRIX 6X6 OR 9X9
#	VARIANCE = VARIANCE (SCALAR)
#	  DELTAQ = MEASURED DEVIATION (SCALAR)
#	 BVECTOR = 6 OR 9 DIMENSIONAL BVECTOR
#
# OUTPUT
#	  DELTAX = STATE VECTOR DEVIATIONS 6 OR 9 DIMENSIONAL
#	      ZI = VECTOR USED FOR THE INCORPORATION 6 OR 9 DIMENSIONAL
#	   GAMMA = SCALAR
#	   OMEGA = OMEGA WEIGHTING VECTOR 6 OR 9 DIMENSIONAL
#
# CALLING SEQUENCE
#	L	CALL 	INCORP1
#
# NORMAL EXIT
#	L+1 OF CALLING SEQUENCE

		BANK	37
		SETLOC	MEASINC
		BANK

		COUNT*	$$/INCOR

		EBANK=	W

; The AGC stores the navigation state (position and velocity) and tracks
; how errors in that state might grow over time through the error transition
; matrix W. When a new measurement arrives, INCORP1 combines the measurement
; error characteristics (BVECTOR) with the state error transition matrix to
; compute intermediate vectors ZI (Z1, Z2, Z3) used in the update computation.

INCORP1		STQ
			EGRESS
		AXT,1	SSP
			54D
			S1
			18D		# IX1 = 54 	S1= 18
		AXT,2	SSP
			18D
			S2
			6		# IX2 = 18	S2=6

; Loop to compute Z1, Z2, and Z3 vectors by multiplying BVECTOR components
; with corresponding rows of the error transition matrix W. These vectors
; represent how measurement errors propagate into state estimate corrections.
; For 6-dimensional state: position and velocity corrections
; For 9-dimensional state: adds radar or landmark bias corrections

Z123		VLOAD	MXV*
			BVECTOR		# BVECTOR (0)
			W +54D,1
		STORE	ZI +18D,2
		VLOAD
			BVECTOR +6	# BVECTOR (1)
# Page 1150
		MXV*	VAD*
			W +108D,1
			ZI +18D,2
		STORE	ZI +18D,2
		VLOAD
			BVECTOR +12D	# BVECTOR (2)
		MXV*	VAD*
			W +162D,1
			ZI +18D,2	# B(0)*W+B(1)*(W+54)+B(2)*(W+108) FIRST PASS
		STORE	ZI +18D,2	# ZI THEN Z2 THEN Z3
		TIX,1
			INCOR1
INCOR1		TIX,2	BON
			Z123		# LOOP FOR Z1,Z2,Z3
			DMENFLG
			INCOR1A

; If operating in 6-dimensional mode (position and velocity only), the
; third Z vector (Z3, which would represent radar/landmark bias corrections)
; is zeroed out since bias estimation is not being performed.

		VLOAD
			ZEROVECS
		STORE	ZI +12D

; ============================================================================
; TRANSITION: From Z-vector computation to Kalman gain calculation
;
; With the Z-vectors computed, the algorithm now calculates GAMMA, which is
; a key scalar in the Kalman filter gain computation. GAMMA represents the
; ratio of measurement variance to total variance (measurement plus predicted
; state variance). This ratio determines how much to trust the new measurement
; versus the predicted state. During landing radar updates, a small GAMMA
; means high confidence in the radar data, leading to larger state corrections.
; ============================================================================

; Compute the sum of squared magnitudes of Z1, Z2, Z3 plus measurement
; variance. This total variance represents uncertainty in the measurement
; mapped into state space. The VSQ (vector square) operations compute the
; dot product of each Z vector with itself.

INCOR1A		SETPD	VLOAD
			0
			ZI
		VSQ	RTB
			TPMODE
		PDVL	VSQ
			ZI +6
		RTB	TAD
			TPMODE
		PDVL	VSQ
			ZI +12D
		RTB	TAD
			TPMODE
		TAD	AXT,2
			VARIANCE
			0
		STORE	TRIPA		# ZI*2 + Z2*2 + Z3*2 + VARIANCE

; The following normalization loop ensures proper scaling for the GAMMA
; computation. AGC fixed-point arithmetic requires careful attention to
; prevent overflow or underflow. The variance is scaled up by factors of
; 4 until it reaches appropriate magnitude for subsequent calculations.

		TLOAD	BOV
			VARIANCE	# CLEAR OVFIND
			+1
		STORE	TEMPVAR		# TEMP STORAGE FOR VARIANCE
		BZE
			INCOR1C
INCOR1B		SL2	BOV
			INCOR1C
		STORE	TEMPVAR
		INCR,2	GOTO
		DEC	1
			INCOR1B

; GAMMA calculation: This scalar represents the Kalman gain denominator.
; GAMMA = VARIANCE / (variance + predicted_variance_in_measurement_space)
; A small GAMMA means the measurement is highly reliable relative to the
; state prediction, so the state will be corrected more strongly.
; During lunar landing, radar measurements typically have small GAMMA values,
; indicating high confidence in the altitude and velocity readings.

INCOR1C		TLOAD	ROUND
			TRIPA
# Page 1151
		DMP	SQRT
			TEMPVAR
		SL*	TAD
			0,2
			TRIPA
		NORM	INCR,2
			X2
		DEC	-2
		SXA,2	AXT,2
			NORMGAM		# NORMALIZATION COUNT -2 FOR GAMMA
			162D
		BDDV	SETPD
			DP1/4TH
			0
		STORE	GAMMA

; Compute normalized measurement residual DELTAQ/A where DELTAQ is the
; difference between actual measurement and predicted measurement, and A
; is the normalization factor from the variance computation. This ratio
; scales the measurement innovation appropriately for state correction.

		TLOAD	NORM
			TRIPA
			X1
		DLOAD	PDDL		# PD 0-1 = NORM (A)
			MPAC
			DELTAQ
		NORM
			S1
		XSU,1	SR1
			S1
		DDV	PUSH		# PD 0-1 = DELTAQ/A
		GOTO
			NEWZCOMP
 -3		SSP
			S2
			54D
INCOR2		VLOAD	VXM*		# COMPUT OMEGA1,2,3
			ZI
			W +162D,2
		PUSH	VLOAD
			ZI +6
		VXM*	VAD
			W +180D,2
		PUSH	VLOAD
			ZI +12D
		VXM*	VAD
			W +198D,2
		PUSH	TIX,2		# PD 2-7=OMEGA1, 8-13=OMEGA2, 14-19=OMEGA3
			INCOR2
		VLOAD	STADR
		STORE	OMEGA +12D
		VLOAD	STADR
		STORE	OMEGA +6
		VLOAD	STADR
		STORE	OMEGA
# Page 1152
		BON	VLOAD
			DMENFLG
			INCOR2AB
			ZEROVECS
		STORE	OMEGA +12D
INCOR2AB	AXT,2	SSP
			18D
			S2
			6
INCOR3		VLOAD*
			OMEGA +18D,2
		VXSC	VSL*
			0		# DELTAQ/A
			0,1
		STORE	DELTAX +18D,2
		TIX,2	VLOAD
			INCOR3
			DELTAX +6
		VSL3
		STORE	DELTAX +6
		GOTO
			EGRESS

; ============================================================================
; TRANSITION: From INCORP1 (State Deviation Computation) to INCORP2 (State Update)
;
; INCORP1 has computed the optimal state vector corrections (DELTAX) using
; Kalman filter theory. The spacecraft now knows how much to adjust its
; estimated position and velocity based on the latest sensor measurements.
; INCORP2 applies these corrections to the actual state vector stored in
; memory, updating the spacecraft's knowledge of where it is and how fast
; it's moving. This two-step process separates the mathematical computation
; from the actual state update, allowing for flexibility in how and when
; corrections are applied during mission-critical phases.
; ============================================================================

# Page 1153
# INCORP2 - INCORPORATES THE COMPUTED STATE VECTOR DEVIATIONS INTO THE
# ESTIMATED STATE VECTOR. THE STATE VECTOR UPDATED MAY BE FOR EITHER THE
# LEM OR THE CSM. DETERMINED BY FLAG VEHUPFLG. (ZERO = LEM) (1 = CSM)
#
# INPUT
#	PERMANENT STATE VECTOR FOR EITHER THE LEM OR CSM
#	VEHUPFLG = UPDATE VEHICLE 0=LEM  1=CSM
#	W = ERROR TRANSITION MATRIX
#	DELTAX = COMPUTED STATE VECTOR DEVIATIONS
# 	DMENFLG = SIZE OF W MATRIX (ZERO=6X6) (1=9X9)
#	GAMMA = SCALAR FOR INCORPORATION
# 	ZI = VECTOR USED IN INCORPORATION
#	OMEGA = WEIGHTING VECTOR
#
# OUTPUT
#	UPDATED PERMANENT STATE VECTOR
#
# CALLING SEQUENCE
#	L	CALL	INCORP2
#
# NORMAL EXIT
#	L+1 OF CALLING SEQUENCE
#

		SETLOC	MEASINC1
		BANK

		COUNT*	$$/INCOR

; INCORP2 entry: Save return address and initialize state vector update.
; During lunar descent, this routine executes after landing radar provides
; altitude and velocity measurements, incorporating them into the LM's
; navigation state. During rendezvous, it processes radar measurements of
; relative position to the Command Module.

INCORP2		STQ	CALL
			EGRESS
			INTSTALL

; Scale the OMEGA weighting vectors by GAMMA to produce the final Kalman
; gain vectors. The Kalman gain determines how much to trust the new
; measurement versus the predicted state. OMEGA represents direction of
; correction, GAMMA represents magnitude based on measurement confidence.

		VLOAD	VXSC		# CALC. GAMMA * OMEGA1,2,3
			OMEGA
			GAMMA
		STOVL	OMEGAM1
			OMEGA +6
		VXSC
			GAMMA
		STOVL	OMEGAM2
			OMEGA +12D
		VXSC
			GAMMA
		STORE	OMEGAM3
		EXIT

; Initialize index registers for W matrix update. The W matrix (error
; transition matrix) tracks how uncertainties propagate through time.
; After incorporating a measurement, W must be updated to reflect the
; reduced uncertainty in the state estimate. WIXA/WIXB index through
; the 6x9 or 9x9 W matrix rows. ZIXA/ZIXB index through Z vector components.

		CAF	54DD		# INITIAL IX 1 SETTING FOR W MATRIX
		TS	WIXA
		TS	WIXB
		CAF	ZERO
		TS	ZIXA		# INITIAL IX 2 SETTING FOR Z COMPONENT
		TS	ZIXB

; Begin phased restart protection for W matrix update. If computer restarts
; during this computation (e.g., from alarm or power transient), the phase
; change system allows recovery without corrupting partially-updated state.

FAZA		TC	PHASCHNG
# Page 1154
		OCT	04022
		TC	UPFLAG
		ADRES	REINTFLG

; FAZA1: First phase of INCORP2 computes updated W matrix in temporary storage.
; The W matrix update follows the Kalman filter covariance update equation:
; W_new = (I - K*H) * W_old where K is Kalman gain and H is measurement matrix.
; This reduces the uncertainty (covariance) in the state estimate after
; incorporating new measurement information.

FAZA1		CA	WIXB		# START FIRST PHASE OF INCORP2
		TS	WIXA		# TO UPDATE 6 OR 9 DIM. W MATRIX IN TEMP
		CA	ZIXB
		TS	ZIXA
		TC	INTPRET
		LXA,1	LXA,2
			WIXA
			ZIXA

; Compute upper 3x9 partition of updated W matrix. This partition corresponds
; to position state component uncertainties. The computation uses normalized
; Z vectors and scaled OMEGA vectors to update each 3-element row of W.

		SSP	DLOAD*
			S1
			6
			ZI,2
		DCOMP	NORM		# CALC UPPER 3X9 PARTITION OF W MATRIX
			S2
		VXSC	XCHX,2
			OMEGAM1
			S2
		LXC,2	XAD,2
			X2
			NORMGAM
		VSL*	XCHX,2
			0,2
			S2
		VAD*
			W +54D,1
		STORE	HOLDW

; Compute middle 3x9 partition of updated W matrix. This partition corresponds
; to velocity state component uncertainties. Same computational pattern as
; position partition but uses OMEGAM2 weighting vector.

		DLOAD*	DCOMP		# CALC MIDDLE 3X9 PARTITION OF W MATRIX
			ZI,2
		NORM	VXSC
			S2
			OMEGAM2
		XCHX,2	LXC,2
			S2
			X2
		XAD,2	VSL*
			NORMGAM
			0,2
		XCHX,2	VAD*
			S2
			W +108D,1
		STORE	HOLDW +6
		BOFF
			DMENFLG		# BRANCH IF 6 DIMENSIONAL
			FAZB

; For 9-dimensional state (position, velocity, bias), compute lower 3x9
; partition of W matrix. This partition tracks uncertainty in radar or
; landmark bias parameters. For 6-dimensional state, skip this section.

		DLOAD*	DCOMP		# CALC LOWER 3X9 PARTITION OF W MATRIX
			ZI,2
		NORM	VXSC
# Page 1155
			S2
			OMEGAM3
		XCHX,2	LXC,2
			S2
			X2
		XAD,2	VSL*
			NORMGAM
			0,2
		XCHX,2	VAD*
			S2
			W +162D,1
		STORE	HOLDW +12D

; Complete W matrix computation and prepare for phase change.

FAZB		CALL
			GRP2PC
		EXIT

; FAZB1: Second phase transfers computed W matrix partitions from temporary
; storage (HOLDW) to permanent W matrix memory. The two-phase approach
; (compute in temp, then transfer) allows restart recovery without corrupting
; the operational W matrix if a restart occurs during computation.

FAZB1		CA	WIXA		# START 2ND PHASE OF INCORP2 TO TRANSFER
		AD	6DD		# 	TEMP REG TO PERM W MATRIX
		TS	WIXB
		CA	ZIXA
		AD	MINUS2
		TS	ZIXB
		TC	INTPRET
		LXA,1	SSP
			WIXA
			S1
			6

; Transfer upper partition (position uncertainty).

		VLOAD
			HOLDW
		STORE	W +54D,1

; Transfer middle partition (velocity uncertainty).

		VLOAD
			HOLDW +6
		STORE	W +108D,1

; For 9-dimensional state, transfer lower partition (bias uncertainty).

		BOFF	VLOAD
			DMENFLG
			FAZB5
			HOLDW +12D
		STORE	W +162D,1

; FAZB2: Loop control for W matrix computation. Index register X1 steps through
; matrix rows. When all rows are processed, proceed to FAZC to update state vector.
; Otherwise, return to FAZA to process next row.

FAZB2		TIX,1	GOTO
			+2
			FAZC		# DONE WITH W MATRIX.  UPDATE STATE VECTOR
		RTB
			FAZA

; FAZB5: Check dimensionality and loop index to determine next computation phase.
; For 9-dimensional state, verify all components processed before proceeding.

FAZB5		SLOAD	DAD
			ZIXB
			12DD
		BHIZ	GOTO
			FAZC
			FAZB2

; FAZC: W matrix update complete. Prepare for state vector update phase.
; Phase change provides restart protection during state vector modification.

FAZC		CALL
			GRP2PC

; ============================================================================
; TRANSITION: From W matrix update to state vector update
;
; The error covariance matrix (W) has been updated to reflect reduced
; uncertainty after measurement incorporation. Now apply the computed
; state corrections (DELTAX) to the actual navigation state vector.
; For 9-dimensional state, this includes radar or landmark bias corrections.
; ============================================================================

# Page 1156
		VLOAD	VAD		# START 3RD PHASE OF INCORP2
			X789		# 7TH, 8TH, 9TH COMPONENT OF STATE VECTOR
			DELTAX +12D	# INCORPORATION FOR X789

; Update 9-dimensional state components (X789): radar or landmark bias terms.
; These track systematic measurement errors. During Apollo 11 lunar descent,
; landing radar bias corrections reconciled altitude/velocity readings with
; the AGC's internal navigation state, critical for accurate touchdown.

		STORE	TX789
		BON	RTB
			VEHUPFLG
			DOCSM
			MOVEPLEM

; FAZAB: Update position and velocity components with scaled state corrections.
; Scaling depends on central body (Moon vs Earth) to maintain numerical precision.
; TDELTAV and TNUV are intermediate storage for accumulated velocity/position changes.
; Overflow detection ensures corrections don't exceed AGC fixed-point range.

FAZAB		BOVB	AXT,2
			TCDANZIG
			0
		BOFF	AXT,2
			MOONTHIS
			+2
			2

; Scale position corrections (DELTAX) based on orbital regime:
; Lunar orbit: B27 scaling (meters * 2^27)
; Earth orbit: B29 scaling (meters * 2^29)
; Index register X2 = 0 for Moon, X2 = 2 for Earth (shift amounts differ by 2).

		VLOAD	VSR*
			DELTAX		# B27 IF MOON ORBIT, B29 IF EARTH
			0 -7,2
		VAD	BOV
			TDELTAV
			FAZAB1

; Add scaled position correction to accumulated TDELTAV.
; If overflow occurs (correction too large for fixed-point representation),
; branch to FAZAB1 to apply correction directly to position state vector (RCV).

		STOVL	TDELTAV
			DELTAX +6	# B5 IF MOON ORBIT, B7 IF EARTH
		VSR*	VAD
			0 -4,2
			TNUV
		BOV
			FAZAB2

; Add scaled velocity correction to accumulated TNUV.
; Velocity scaling: B5 (lunar) or B7 (Earth) in meters/centisecond.
; Overflow branches to FAZAB2 for direct velocity state vector (VCV) update.

		STCALL	TNUV
			FAZAB3

; ============================================================================
; OVERFLOW HANDLING: Direct state vector updates when corrections too large
; ============================================================================

; FAZAB1: Position overflow handler. When DELTAX correction causes TDELTAV
; overflow, apply correction directly to position state vector (RCV).
; This prevents loss of precision from accumulated intermediate values.

FAZAB1		VLOAD	VAD
			RCV
			DELTAX
		STORE	RCV

; FAZAB2: Velocity overflow handler. When DELTAX+6 correction causes TNUV
; overflow, apply correction directly to velocity state vector (VCV).
; Ensures velocity updates complete even with large measurement residuals.

FAZAB2		VLOAD	VAD
			VCV
			DELTAX +6
		STORE	VCV
		SXA,2	CALL
			PBODY
			RECTIFY

; Call RECTIFY to maintain coordinate system integrity after state updates.
; RECTIFY performs conic integration rectification, ensuring position and
; velocity vectors remain consistent with orbital mechanics constraints.
; PBODY (central body flag) passed via X2 register for proper integration.
; FAZAB3: Prepare for vehicle-specific state vector storage.
; Phase change ensures restart protection during critical state update completion.

FAZAB3		CALL
			GRP2PC
		BON	RTB
			VEHUPFLG
			DOCSM1
			MOVEALEM

; Store updated navigation state to downlink buffer for telemetry to Mission Control.
; During Apollo 11 descent, these state updates were transmitted to Houston,
; allowing flight controllers to monitor the LM's approach trajectory in real-time.

		CALL
			SVDWN2		# STORE DOWNLINK STATE VECTOR

; FAZAB4: Final phase of state incorporation. For 9-dimensional state (DMENFLG set),
; copy temporary bias storage (TX789) to final state vector component (X789).
; For 6-dimensional state, bypass this step and proceed directly to exit.

FAZAB4		CALL
# Page 1157
			GRP2PC		# PHASE CHANGE
		BOFF	VLOAD
			DMENFLG
			FAZAB5		# 6 DIMENSIONAL
			TX789		# 9 DIMENSIONAL
		STORE	X789

; FAZAB5: Normal exit from INCORP2. Restore return address from EGRESS to QPRET.
; EXIT to basic AGC instructions, then POSTJUMP to INTWAKE to resume interrupted
; task. This completes the measurement incorporation cycle, with navigation state
; now reflecting the latest tracking data (radar, ground station, or landmark).

FAZAB5		LXA,1	SXA,1
			EGRESS
			QPRET
		EXIT
		TC	POSTJUMP	# EXIT
		CADR	INTWAKE

; ============================================================================
; CSM-SPECIFIC STATE UPDATE ROUTINES
; ============================================================================

; DOCSM: Command/Service Module position state update path.
; When VEHUPFLG indicates CSM (not LM), use CSM-specific coordinate transformation
; MOVEPCSM before updating position and velocity in FAZAB.

DOCSM		RTB	GOTO
			MOVEPCSM
			FAZAB

; DOCSM1: CSM velocity state update path.
; Apply CSM-specific coordinate transformation MOVEACSM, store to downlink buffer
; (SVDWN1), then proceed to final phase (FAZAB4). During Apollo 11 mission,
; these routines updated Columbia's orbit while Eagle descended to lunar surface.

DOCSM1		RTB	CALL
			MOVEACSM
			SVDWN1		# STORE DOWNLINK STATE VECTOR
		GOTO
			FAZAB4

; ============================================================================
; CONSTANT DEFINITIONS
; ============================================================================

ZEROD		=	ZEROVECS
54DD		DEC	54		# Matrix dimension offset (6x9 = 54 elements)
6DD		DEC	-6		# Decrement value for 6-dimensional indexing
12DD		DEC	12		# Offset for 9th component (bias terms)

		SETLOC	RENDEZ
		BANK
		COUNT*	$$/INCOR

; ============================================================================
; NEWZCOMP -- NORMALIZE ZI VECTORS FOR NUMERICAL PRECISION
; ============================================================================
;
; Computes the largest magnitude among the three ZI vector components (Z1, Z2, Z3)
; and normalizes all three vectors by left-shifting to maximize precision in
; subsequent matrix computations. This prevents underflow in the Kalman gain
; calculation when measurement sensitivity is small.
;
; COMMENT-ONLY READERS: This routine ensures the guidance computer maintains
; maximum numerical precision when processing tracking measurements. The AGC's
; 16-bit word length requires careful scaling to avoid losing significant digits.
;
; CODE-ALONG READERS: The algorithm finds the maximum absolute value among the
; three ZI vectors, computes the optimal normalization shift count, then applies
; that shift uniformly to all three vectors. The shift count is saved in NORMZI+1
; for later denormalization of computed results.
; ============================================================================

NEWZCOMP	VLOAD	ABVAL
			ZI

; Compute absolute value (magnitude) of first ZI vector component (Z1).
; This represents the sensitivity of first measurement to state vector errors.

		STOVL	NORMZI
			ZI +6
		ABVAL	PUSH
		DSU	BMN
			NORMZI
			+3
		DLOAD	STADR
		STORE	NORMZI

; Compare Z2 magnitude with current maximum (Z1). If Z2 larger, update NORMZI.
; STADR loads address from stack without disturbing pushdown list.

		VLOAD	ABVAL
			ZI +12D
		PUSH	DSU
			NORMZI
		BMN	DLOAD
			+3
		STADR
		STORE	NORMZI		# LARGEST ABVAL

; Compare Z3 magnitude with current maximum. NORMZI now contains the largest
; magnitude among all three ZI vector components. This determines scaling factor.

		DLOAD	SXA,1
			NORMZI
			NORMZI		# SAVE X1
		NORM	INCR,1

; NORM instruction: Find normalization shift count for NORMZI value.
; Result placed in X1 register. This determines how many bits to left-shift
; the ZI vectors to maximize precision without overflow.

# Page 1158
			X1
		DEC	2

; Decrement shift count by 2 to provide headroom for subsequent computations.
; Prevents overflow in matrix multiplication operations following normalization.

		VLOAD	VSL*
			ZI
			0,1
		STOVL	ZI
			ZI +6
		VSL*
			0,1
		STOVL	ZI +6
			ZI +12D
		VSL*	SXA,1
			0,1
			NORMZI +1	# SAVE SHIFT

; Apply computed shift count uniformly to all three ZI vectors (Z1, Z2, Z3).
; VSL* = Variable Shift Left indexed by X1 register.
; Shift count saved in NORMZI+1 for later denormalization of results.
; During Apollo 11 descent, this normalization maintained precision in computing
; the Kalman gain despite the wide range of measurement sensitivities.

		STORE	ZI +12D
		LXA,1	XSU,1
			NORMGAM
			NORMZI +1
		XSU,1
			NORMZI +1
		SXA,1	LXC,1
			NORMGAM
			NORMZI +1

; Adjust NORMGAM (gamma normalization factor) to account for ZI normalization.
; Subtracts twice the shift count from NORMGAM, ensuring gamma scaling remains
; consistent with normalized ZI vectors in subsequent Kalman gain computation.

		XAD,1	SETPD
			NORMZI
			2D
		GOTO
			INCOR2 -3

; Return to INCOR2 main loop with normalized ZI vectors and adjusted scaling.
; The -3 offset re-enters INCOR2 at the point following NEWZCOMP call,
; continuing with Kalman gain and state covariance update computations.

NORMZI		=	36D		# Normalization storage location (erasable memory)


