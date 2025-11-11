# Copyright:	Public domain.
# Filename:	POWERED_FLIGHT_SUBROUTINES.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1365-1372
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#                               from the Colossus249/ file of the same
#                               name, using Comanche055 page images.
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
#    Assemble revision 055 of AGC program Comanche by NASA
#    2021113-051.  10:28 APR. 1, 1969
#
#    This AGC program shall also be referred to as
#            Colossus 2A

; ============================================================================
; FILE: POWERED_FLIGHT_SUBROUTINES.agc
; MODULE: CHIEFTAN Subsystem (Core OS)
; MISSION PHASE: all-phases (burn operations)
;
; TL;DR: Powered flight trajectory utility subroutines supporting thrust
;        integration and navigation during engine burns. Provides computational
;        support for guidance during SPS and RCS powered maneuvers throughout
;        Apollo 11 mission phases (TLI, LOI, TEI burns).
;
; COMMENT-ONLY READERS: This program provided mathematical functions for
;        calculating spacecraft position and orientation during rocket burns.
; CODE-ALONG READERS: Study coordinate transformation algorithms, trigonometric
;        computation for gimbal angles, and navigation state updates during
;        powered flight phases.
; ============================================================================

# Page 1365
		BANK	14		# SAME FBANK AS THE FINDCDUD SUB-PROGRAM
		SETLOC	POWFLITE
		BANK

		EBANK=	DEXDEX
		COUNT*	$$/POWFL

; ============================================================================
; CDUTRIG FAMILY - GIMBAL ANGLE TRIGONOMETRIC COMPUTATION
;
; During powered flight, the spacecraft must continuously track its orientation
; relative to inertial space. The Inertial Measurement Unit (IMU) gimbals
; provide three angles (inner, middle, outer) that define the spacecraft's
; attitude. These routines compute the sines and cosines of these angles,
; which are essential for coordinate transformations between the spacecraft
; body frame and inertial navigation frame during rocket burns.
;
; The Command Module used these calculations during critical engine burns:
; - Translunar Injection (TLI): July 16, 1969, departing Earth orbit
; - Lunar Orbit Insertion (LOI): July 19, 1969, entering Moon orbit  
; - Transearth Injection (TEI): July 21, 1969, departing lunar orbit
;
; Historical Context: During Apollo 11's TLI burn, the Service Propulsion
; System (SPS) engine fired for approximately 5 minutes while these routines
; continuously updated the navigation solution based on changing gimbal angles.
; ============================================================================

# 	CDUTRIG, CDUTRIG1, CDUTRIG2, AND CD*GR*GS ALL COMPUTE THE SINES AND
# COSINES OF THREE 2'S COMPLEMENT ANGLES AND PLACE THE RESULT, DOUBLE
# PRECISION, IN THE SAME ORDER AS THE INPUTS, AT SINCDU AND COSCDU.  AN
# ADDITIONAL OUTPUT IS THE 1'S COMPLEMENT ANGLES AT CDUSPOT.  THESE
# ROUTINES GO OUT OF THEIR WAY TO LEAVE THE MPAC AREA AS THEY FIND IT,
# EXCEPT FOR THE GENERALLY UNIMPORTANT MPAC +2.  THEY DIFFER ONLY IN
# WHERE THEY GET THE ANGLES, AND IN METHOD OF CALLING.

# 	CDUTRIG (AND CDUTRIG1, WHICH CAN BE CALLED IN BASIC) COMPUTE THE
# SINES AND COSINES FROM THE CURRENT CONTENTS OF THE CDU REGISTERS.
# THE CONTENTS OF CDUTEMP, ETC., ARE NOT TOUCHED SO THAT THEY MAY
# CONTINUE TO FORM A CONSISTENT SET WITH THE LATEST PIPA READINGS.

# 	CDUTRIG1 IS LIKE CDUTRIG EXCEPT THAT IT CAN BE CALLED IN BASIC.

# 	CD*TR*GS FINDS CDU VALUES IN CDUSPOT RATHER THAN IN CDUTEMP.  THIS
# ALLOWS USERS TO MAKE TRANSFORMATIONS USING ARBITRARY ANGLES, OR REAL
# ANGLES IN AN ORDER OTHER THAN X Y Z.  A CALL TO THIS ROUTINE IS
# NECESSARY IN PREPARATION FOR A CALL TO AX*SR*T IN EITHER OF ITS TWO
# MODES (SMNB OR NBSM).  SINCE AX*SR*T EXPECTS TO FIND THE SINES AND
# COSINES IN THE ORDER Y Z X THE ANGLES MUST HAVE BEEN PLACED IN CDUSPOT
# IN THIS ORDER.  CD*TR*GS NEED NOT BE REPEATED WHEN AX*SR*T IS CALLED
# MORE THAN ONCE, PROVIDED THE ANGLES HAVE NOT CHANGED.  NOTE THAT SINCE
# IT CLOBBERS BUF2 (IN THE SINE AND COSINE ROUTINES) CD*TR*GS CANNOT BE
# CALLED USING BANKCALL.  SORRY.

# 	CD*TR*G IS LIKE CD*TR*GS EXCEPT THAT IT CAN BE CALLED IN
# INTERPRETIVE.

; CDUTRIG - Main entry point for trigonometric computation of CDU angles.
; Called from interpretive language during powered flight guidance calculations.
; Reads current gimbal angles from CDU hardware registers (CDUX, CDUY, CDUZ)
; representing the spacecraft's orientation relative to the stable member
; (inertial platform). Computes double-precision sine and cosine values needed
; for coordinate transformations during rocket burns.
CDUTRIG		EXIT
		TC	CDUTRIGS
		TC	INTPRET
		RVQ

; CD*TR*G - Interpretive language callable version of CD*TR*GS.
; Used when guidance programs need to compute trigonometric values for
; arbitrary gimbal angles stored in CDUSPOT rather than reading hardware CDUs.
CD*TR*G		EXIT
		TC	CD*TR*GS
		TC	INTPRET
		RVQ

; CDUTRIGS - Copy current CDU hardware register values to CDUSPOT workspace.
; During powered flight, the IMU gimbal angles are continuously updated by
; the autopilot to maintain desired spacecraft attitude. This routine captures
; a consistent snapshot of all three gimbal angles (X, Y, Z) for subsequent
; trigonometric computation. The angles represent rotation about inner, middle,
; and outer gimbal axes respectively.
CDUTRIGS	CA	CDUX
		TS	CDUSPOT +4
		CA	CDUY
		TS	CDUSPOT
# Page 1366
		CA	CDUZ
		TS	CDUSPOT +2

; CD*TR*GS - Core trigonometric computation loop for three gimbal angles.
; Processes gimbal angles in CDUSPOT, computing both sine and cosine values
; in double precision. Results stored at SINCDU and COSCDU arrays. The MPAC
; (Multi-Purpose Accumulator) area is carefully preserved to avoid corrupting
; other concurrent computations. This routine is timing-critical during burns
; when navigation solutions must be updated every guidance cycle.
CD*TR*GS	EXTEND
		QXCH	TEM2
		CAF	FOUR
TR*GL**P	MASK	SIX		# MAKE IT EVEN AND SMALLER
		TS	TEM3
		INDEX	TEM3
		CA	CDUSPOT
		DXCH	MPAC		# STORING 2'S COMP ANGLE, LOADING MPAC
		DXCH	VBUF +4		# STORING MPAC FOR LATER RESTORATION
		TC	USPRCADR
		CADR	CDULOGIC
		EXTEND
		DCA	MPAC
		INDEX	TEM3
		DXCH	CDUSPOT		# STORING 1'S COMPLEMENT ANGLE
		TC	USPRCADR
		CADR	COSINE
		DXCH	MPAC
		INDEX	TEM3
		DXCH	COSCDU		# STORING COSINE
		EXTEND
		INDEX	TEM3
		DCA	CDUSPOT		# LOADING 1'S COMPLEMENT ANGLE
		TC	USPRCADR
		CADR	SINE +1		# SINE +1 EXPECTS ARGUMENT IN A AND L
		DXCH	VBUF +4		# BRINGING UP PRIOR MPAC TO BE RESTORED
		DXCH	MPAC
		INDEX	TEM3
		DXCH	SINCDU
		CCS	TEM3
		TCF	TR*GL**P
		TC	TEM2
# Page 1367
# *******************************************************************************************************

; ============================================================================
; TRANSITION: From full trigonometric computation to fast angle conversion
;
; The previous routines compute complete sine and cosine values for coordinate
; transformations during powered flight. However, during the most time-critical
; portion of the guidance cycle (when the spacecraft is actively thrusting),
; there isn't sufficient time for full trigonometric computation every cycle.
;
; QUICTRIG provides a dramatically faster alternative - executing in just 4.1
; milliseconds compared to CD*TR*GS's 41 milliseconds. This 10X speed improvement
; was crucial during Apollo 11's engine burns when the guidance computer had to
; maintain real-time control while simultaneously updating navigation solutions,
; managing displays, and handling crew inputs.
;
; Historical Context: During the critical Translunar Injection burn on July 16,
; 1969, the Service Propulsion System fired for approximately 5 minutes while
; these fast trigonometric routines enabled the guidance system to maintain
; precise attitude control every guidance cycle.
; ============================================================================

# 	QUICTRIG, INTENDED FOR GUIDANCE CYCLE USE WHERE TIME IS CRITICAL, IS A MUCH FASTER VERSION OF CD*TR*GS.
# QUICTRIG COMPUTES AND STORES THE SINES AND COSINES OF THE 2'S COMPLEMENT ANGLES AT CDUSPOT, CDUSPOT +2,
# AND CDUSPOT +4.  UNLIKE CD*TR*GS, QUICTRIG DOES NOT LEAVE THE 1'S COMPLEMENT VERSIONS OF THE ANGLES IN
# CDUSPOT.  QUICTRIG'S EXECUTION TIME IS 4.1 MS; THIS IS 10 TIMES AS FAST AS CD*TR*GS.  QUICTRIG MAY BE
# CALLED FROM INTERPRETIVE AS AN RTB OP-CODE, OR FROM BASIC VIA BANKCALL OR IBNKCALL.

; QUICTRIG - High-speed gimbal angle trigonometric computation for guidance cycle.
; Optimized for time-critical execution during powered flight when the spacecraft
; is under thrust. Computes sine and cosine of three gimbal angles in just 4.1ms.
; Uses INHINT (inhibit interrupts) to protect temporary storage from concurrent
; DAP (Digital Autopilot) access. Critical for maintaining guidance cycle timing
; during SPS engine burns (TLI, LOI, TEI) when navigation updates must complete
; within strict real-time constraints.
QUICTRIG	INHINT			# INHINT SINCE DAP USES THE SAME TEMPS
		EXTEND
		QXCH	ITEMP1
		CAF	FOUR
	+4	MASK	SIX
		TS	ITEMP2
		INDEX	ITEMP2
		CA	CDUSPOT
		TC	SPSIN
		EXTEND
		MP	BIT14		# SCALE DOWN TO MATCH INTERPRETER OUTPUTS
		INDEX	ITEMP2
		DXCH	SINCDU
		INDEX	ITEMP2
		CA	CDUSPOT
		TC	SPCOS
		EXTEND
		MP	BIT14
		INDEX	ITEMP2
		DXCH	COSCDU
		CCS	ITEMP2
		TCF	QUICTRIG +4
		CA	ITEMP1
		RELINT
		TC	A
# Page 1368
#****************************************************************************


; ============================================================================
; TRANSITION: From fast trigonometric computation to coordinate transformation
;
; Having computed sine and cosine values for gimbal angles, the guidance system
; now requires transformation routines to convert vectors between coordinate
; frames. These transformations are fundamental to powered flight navigation:
;
; - Spacecraft coordinates (Stable Member frame - SM) represent the inertial
;   reference maintained by the IMU gyroscopes
; - Navigation Base coordinates (NB) represent the spacecraft body frame,
;   which changes as the vehicle rotates during thrust vector control
;
; During powered flight phases (TLI, LOI, TEI burns), the guidance computer
; must continuously transform velocity and position vectors between these frames
; to maintain precise trajectory control while the Service Propulsion System
; generates over 20,000 pounds of thrust.
; ============================================================================

# 	THESE INTERFACE ROUTINES MAKE IT POSSIBLE TO CALL AX*SR*T, ETC., IN
# INTERPRETIVE.  LATER, WHERE POSSIBLE, THEY WILL BE ELIMINATED.
#
# 	NBSM WILL BE THE FIRST TO GO.  IT SHOULD NOT BE USED.

; NBSM - Navigation Base to Stable Member transformation (deprecated interface).
; This interface routine allows interpretive programs to call the coordinate
; transformation engine AX*SR*T. The routine expects CDU angles in S1 and the
; vector to transform in 32D. NASA programmers noted this would be eliminated
; in future versions as better interfaces were developed.
NBSM		STQ
			X2
		LXC,1	VLOAD*
			S1		# BASE ADDRESS OF THE CDU ANGLES IS IN S1
			0,1
		STOVL	CDUSPOT
			32D		# VECTOR TO BE TRANSFORMED IS IN 32D
		CALL
			TRG*NBSM
		STCALL	32D		# SINCE THERE'S NO STGOTO
			X2

; ============================================================================
; COORDINATE TRANSFORMATION INTERFACE ROUTINES (Permanent)
;
; These routines provide the standard interface for transforming vectors between
; coordinate frames during powered flight. All routines:
;   - Restore user's erasable bank (EBANK) setting to prevent memory conflicts
;   - Accept and return vectors in MPAC (Math Package Area) at MPAC, MPAC+3, MPAC+5
;   - Use CALL/QPRET for interpretive mode invocation
;
; Two transformation directions are supported:
;   SM→NB (Stable Member to Navigation Base): Convert inertial frame vectors
;         to spacecraft body frame - used when guidance must command specific
;         spacecraft attitudes relative to the inertial trajectory
;   NB→SM (Navigation Base to Stable Member): Convert spacecraft body frame
;         vectors to inertial frame - used when IMU measurements must be
;         integrated into the inertial navigation solution
;
; Two angle input methods:
;   TRG* routines: Use arbitrary angles pre-loaded in CDUSPOT (Y Z X order)
;   CDU* routines: Use current gimbal angles from CDU counters (CDUX, CDUY, CDUZ)
; ============================================================================

# 	THESE INTERFACE ROUTINES ARE PERMANENT.  ALL RESTORE USER'S EBANK
# SETTING.  ALL ARE STRICT INTERPRETIVE SUBROUTINES, CALLED USING "CALL",
# RETURNING VIA QPRET.  ALL EXPECT AND RETURN THE VECTOR TO BE TRANSFOR-
# MED INTERPRETER-STYLE IN MPAC; COMPONENTS AT MPAC, MPAC +3, AND MPAC +5.

# 	TRG*SMNB AND TRG*NBSM BOTH EXPECT TO SEE THE 2'S COMPLEMENT ANGLES
# AT CDUSPOT (ORDER Y Z X, AT CDUSPOT, CDUSPOT +2, AND CDUSPOT +4; ODD
# LOCATIONS NEED NOT BE ZEROED).  TRG*NBSM DOES THE NB TO SM TRANSFOR-
# MATION; TRG*SMNB, VICE VERSA.

# 	CDU*NBSM DOES ITS TRANSFORMATION USING THE PRESENT CONTENTS OF
# THE CDL COUNTERS.  OTHERWISE IT IS LIKE TRG*NBSM.
#
# 	CDU*SMNB IS THE COMPLEMENT OF CDU*NBSM.

; CDU*SMNB - Transform vector from Stable Member to Navigation Base using current CDU angles.
; Reads gimbal angles directly from CDUX, CDUY, CDUZ counters (current spacecraft attitude).
; Used during powered flight when the transformation must reflect the instantaneous
; vehicle orientation. The CDU counters are continuously updated by the IMU as the
; spacecraft rotates under thrust vector control.
CDU*SMNB	EXIT
		TC	CDUTRIGS	; Compute sines/cosines from current CDU angles
		TCF	C*MM*N1		; Branch to common transformation code

; TRG*SMNB - Transform vector from Stable Member to Navigation Base using arbitrary angles.
; Uses pre-loaded angles from CDUSPOT rather than current CDU counters. This allows
; transformations using predicted future attitudes or historical attitudes without
; requiring actual gimbal motion. Critical for guidance computations that simulate
; future trajectory states during burn planning.
TRG*SMNB	EXIT
		TC	CD*TR*GS	; Compute sines/cosines from CDUSPOT angles
C*MM*N1		TC	MPACVBUF	; Copy vector from MPAC to VBUF (AX*SR*T input buffer)
		CS	THREE		; Load -3 signal for SM→NB transformation direction
C*MM*N2		TC	AX*SR*T		; Execute transformation engine
		TC	INTPRET		; Return to interpretive mode
		VLOAD	RVQ		; Load result from VBUF to MPAC and return
			VBUF

; CDU*NBSM - Transform vector from Navigation Base to Stable Member using current CDU angles.
; Inverse transformation of CDU*SMNB. Converts spacecraft body frame vectors to
; inertial frame. Used when processing IMU accelerometer measurements (PIPA pulses)
; that arrive in body coordinates but must be integrated into the inertial
; navigation state vector during powered flight.
CDU*NBSM	EXIT
		TC	CDUTRIGS	; Compute sines/cosines from current CDU angles

# Page 1369

		TCF	C*MM*N3		; Branch to common transformation code

; TRG*NBSM - Transform vector from Navigation Base to Stable Member using arbitrary angles.
; Inverse transformation of TRG*SMNB. Uses pre-loaded angles from CDUSPOT.
; Applied during trajectory prediction when the guidance system must compute
; expected accelerometer readings for future time points to validate the
; navigation solution against predicted thrust profiles.
TRG*NBSM	EXIT
		TC	CD*TR*GS	; Compute sines/cosines from CDUSPOT angles
C*MM*N3		TC	MPACVBUF	; Copy vector from MPAC to VBUF
		CA	THREE		; Load +3 signal for NB→SM transformation direction
		TCF	C*MM*N2		; Branch to common transformation execution

# 	*NBSM* AND *SMNB* EXPECT TO SEE THE SINES AND COSINES (AT SINCDU
# AND COSCDU) RATHER THAN THE ANGLES THEMSELVES.  OTHERWISE THEY ARE
# LIKE TRG*NBSM AND TRG*SMNB.

# 	NOTE THAT JUST AS CD*TR*GS NEED BE CALLED ONLY ONCE FOR EACH SERIES
# OF TRANSFORMATIONS USING THE SAME ANGLES, SO TOO ONLY ONE OF TRG*NBSM
# AND TRG*SMNB NEED BE CALLED FOR EACH SERIES.  FOR SUBSEQUENT TRANFOR-
# MATIONS USE *NBSM* AND *SMNB*.

; *SMNB* - Fast SM→NB transformation when sines/cosines already computed.
; Optimized interface that skips trigonometric computation when multiple vectors
; must be transformed using the same gimbal angles. During rapid trajectory updates
; in powered flight, this saves ~2 milliseconds per transformation - critical when
; the guidance computer must update thrust vector commands every 2 seconds while
; integrating accelerometer data at 100Hz.
*SMNB*		EXIT
		TCF	C*MM*N1		; Branch directly to common transformation path

; *NBSM* - Fast NB→SM transformation when sines/cosines already computed.
; Companion routine to *SMNB* for the inverse transformation. Used when processing
; batches of IMU measurements that must all be converted to inertial frame using
; the same instantaneous attitude snapshot.
*NBSM*		EXIT
		TCF	C*MM*N3		; Branch directly to common transformation path

; ============================================================================
; AX*SR*T - CORE AXIS TRANSFORMATION ENGINE
;
; The heart of powered flight coordinate transformations. This routine performs
; the mathematical transformation of 3-component vectors between coordinate frames
; using the gimbal angles that define the spacecraft's orientation.
;
; MISSION CONTEXT:
; During critical Apollo 11 burn phases, this transformation ran continuously:
; - Translunar Injection (TLI): July 16, 1969 - Converting thrust vectors as
;   the 20,500-lb SPS engine accelerated Columbia and Eagle toward the Moon
; - Lunar Orbit Insertion (LOI): July 19, 1969 - Transforming navigation state
;   as the spacecraft rotated into lunar orbit
; - Transearth Injection (TEI): July 21, 1969 - Computing trajectory corrections
;   for the three-day return journey to Pacific splashdown
;
; TECHNICAL IMPLEMENTATION:
; The transformation uses Euler angle rotation matrices constructed from three
; gimbal angles (Y, Z, X order - inner, middle, outer). The AGC's memory
; constraints (36K ROM, 2K RAM) demand this highly optimized implementation that
; reuses the same memory locations for inputs, intermediate results, and outputs.
;
; Input format: A register contains direction code
;               +3 = NB→SM (Navigation Base to Stable Member - body to inertial)
;               -3 = SM→NB (Stable Member to Navigation Base - inertial to body)
; Input vector: VBUF, VBUF+1, VBUF+2 (three single-precision components)
; Prerequisites: Sines/cosines at SINCDU/COSCDU (Y Z X order from CDULOGIC)
; Output: VBUF, VBUF+1, VBUF+2 (transformed result, overwriting input)
;
; CRITICAL CONSTRAINT: Vector magnitude must be <1.0 (scaled representation)
; Vectors exceeding unity can cause arithmetic overflow during transformation.
; Position and velocity vectors are pre-scaled to satisfy this requirement.
; ============================================================================

# 	AX*SR*T COMBINES THE OLD SMNB AND NBSM.  FOR THE NB TO SM
# TRANSFORMATION, ENTER WITH +3 IN A.  FOR SM TO NB, ENTER WITH -3.
# THE VECTOR TO BE TRANSFORMED ARRIVES, AND IS RETURNED, IN VBUF.
# AX*SR*T EXPECTS TO FIND THE SINES AND COSINES OF THE ANGLES OF ROTATION
# AT SINCDU AND COSCDU, IN THE ORDER Y Z X.  A CALL TO CD*TR*GS, WITH
# THE 2'S COMPLEMENT ANGLES (ORDER Y Z X) AT CDUSPOT, WILL TAKE CARE OF
# THIS.  HERE IS A SAMPLE CALLING SEQUENCE:-

#			TC	CDUTRIGS
#			CS	THREE		("CA THREE" FOR NBSM)
#			TC	AX*SR*T

# THE CALL TO CD*TR*GS NEED NOT BE REPEATED, WHEN AX*SR*T IS CALLED MORE
# THAN ONCE, UNLESS THE ANGLES HAVE CHANGED.

# 	AX*SR*T IS GUARANTEED SAFE ONLY FOR VECTORS OF MAGNITUDE LESS THAN
# UNITY.  A LOOK AT THE CASE IN WHICH A VECTOR OF GREATER MAGNITUDE
# HAPPENS TO LIE ALONG AN AXIS OF THE SYSTEM TO WHICH IT IS TO BE TRANS-
# FORMED CONVINCES ONE THAT THIS IS A RESTRICTION WHICH MUST BE ACCEPTED.

; Core transformation engine entry point. The direction code controls whether we're
; transforming from Navigation Base to Stable Member (body frame to inertial, +3)
; or from Stable Member to Navigation Base (inertial to body frame, -3).

AX*SR*T		TS	DEXDEX		; Store direction code as "index of indexes"		# WHERE IT BECOMES THE INDEX OF INDEXES
		EXTEND
		QXCH	RTNSAVER	; Save return address (Q register) to RTNSAVER

; The transformation applies three successive 2D rotations using gimbal angles Y, Z, X.
; DEXDEX controls iteration: +3,+2,+1 for NB→SM or -3,-2,-1 for SM→NB (reverse order).
; This loop iterates three times, once for each axis rotation.

R*TL**P		CCS	DEXDEX		; Test DEXDEX and decrement:	+3 --> 0	-3 --> 2
		CS	DEXDEX		; Complement and add THREE:	+2 --> 1	-2 --> 1
		AD	THREE		; Result indexes INDEXI table:	+1 --> 2	-1 --> 0
# Page 1370
		EXTEND
		INDEX	A		; Use computed index (0, 1, or 2) into INDEXI
		DCA	INDEXI		; Fetch double word: sine/cosine indices for this rotation
		DXCH	DEXI		; Store to DEXI (DEX1 gets index, DEX2 gets complement)

; Initialize for rotation computation. Each rotation transforms two vector components
; while leaving the third unchanged. BUF indexes which components are transformed.

		CA	ONE		; Start with index 1 (middle component)
		TS	BUF		; BUF will cycle through component positions
		EXTEND
		INDEX	DEX1		; Index using rotation-specific offset
		DCS	VBUF		; Load and complement vector component (prepare for subtraction)
		TCF	LOOP1		; Enter main computation loop		# REALLY BE A SUBTRACT, AND VICE VERSA

; ============================================================================
; MAIN ROTATION COMPUTATION LOOP
;
; This loop performs a 2D rotation transformation for each gimbal angle.
; Uses sin/cos multiplication via DMPSUB subroutine for double-precision accuracy.
; Each iteration: V' = V*cos(θ) ± U*sin(θ) where sign depends on NB→SM or SM→NB.
; ============================================================================

LOOP2		DXCH	BUF		; Swap: load vector component, store loop index	# LOADING VECTOR COMPONENT, STORING INDEX

LOOP1		DXCH	MPAC		; Place vector component in MPAC for multiplication
		CA	SINESLOC	; Base address of sine table (SINCDU array)
		AD	DEX1		; Add axis offset to get correct sine value
		TS	ADDRWD		; ADDRWD now points to sin(gimbal_angle)

		TC	DMPSUB		; Call double-precision multiply: MPAC * sin(θ)	# MULTIPLY BY SIN(CDUANGLE)
		CCS	DEXDEX		; Test sign: positive = NBSM, negative = SMNB
		DXCH	MPAC		; NBSM case: use result as-is (V*sin)	# NBSM CASE
		TCF	+3		; Skip to cosine term
		EXTEND			; SMNB case: negate result (-V*sin)	# SMNB CASE
		DCS	MPAC		; Complement to reverse rotation direction
		DXCH	TERM1TMP	; Store first term: ±V*sin(θ) in TERM1TMP

		CA	SIX		; Offset from SINCDU to COSCDU table	# SINCDU AND COSCDU (EACH 6 WORDS) MUST
		ADS	ADDRWD		; ADDRWD now points to cos(gimbal_angle)	#	BE CONSECUTIVE AND IN THAT ORDER

		EXTEND
		INDEX	BUF		; Double-indexed load: BUF selects component group
		INDEX	DEX1		; DEX1 selects which component within group
		DCA	VBUF		; Load the other vector component for rotation
		DXCH	MPAC		; Place in MPAC for multiplication
		TC	DMPSUB		; Call double-precision multiply: U*cos(θ)	# MULTIPLY BY COS(CDUANGLE)
		DXCH	MPAC		; Retrieve multiplication result
		DAS	TERM1TMP	; Add both terms: V' = ±V*sin(θ) + U*cos(θ)
		DXCH	TERM1TMP	; Load final rotated component
		DDOUBL			; Scale by 2 (compensate for AGC fixed-point format)
		INDEX	BUF		; Double-indexed store back to VBUF
		INDEX	DEX1
		DXCH	VBUF		; Store rotated component in output vector
		DXCH	BUF		; Swap back: load index, store component	# LOADING INDEX, STORING VECTOR COMPONENT

		CCS	A		; Test loop index (now in A register)	# 'CAUSE THAT'S WHERE THE INDEX NOW IS
		TCF	LOOP2		; Positive: rotate second component of pair

		EXTEND
		DIM	DEXDEX		; Decrement iteration counter (magnitude only, preserving ±sign)	# DECREMENT MAGNITUDE PRESERVING SIGN

; Both components of the rotation plane are now updated. Test if all three
; gimbal angle rotations are complete (DEXDEX counts: 3→2→1→0 or -3→-2→-1→0).

# Page 1371
TSTPOINT	CCS	DEXDEX		; Test DEXDEX: zero means all rotations done	# ONLY THE BRANCHING FUNCTION IS USED
		TCF	R*TL**P		; Not zero: continue with next gimbal angle rotation
		TC	RTNSAVER	; Zero: transformation complete, return to caller
		TCF	R*TL**P		; (Negative case: also continue rotation loop)
		TC	RTNSAVER


; Address reference used by AX*SR*T to locate sine/cosine tables
SINESLOC	ADRES	SINCDU		# FOR USE IN SETTING ADDRWD

; ============================================================================
; INDEXI TABLE - Rotation Sequence Index Constants
;
; This table defines which vector components are affected by each gimbal
; rotation. The values (4, 2, 0) correspond to memory offsets into VBUF
; for accessing X, Y, Z components. These constants are carefully ordered
; to implement the correct rotation sequence for spacecraft attitude control.
;
; WARNING: These constants are critical to coordinate transformation logic.
; Modification will break all attitude computations. Hence the emphatic
; "DON'T TOUCH" comments in the original NASA code!
; ============================================================================

INDEXI		DEC	4		# **********   DON'T   ***********
		DEC	2		# **********   TOUCH   ***********
		DEC	0		# **********   THESE   ***********
		DEC	4		# ********** CONSTANTS ***********

# ******************************************************************************
# Page 1372
# THIS SUBROUTINE COMPUTES INCREMENTAL CHANGES IN CDU(GIMBAL) ANGLES FROM INCREMENTAL CHANGES ABOUT SM AXES.  IT
# REQUIRES SM INCREMENTS AS A DP VECTOR SCALED AT ONE REVOLUTION(DTHETASM,+2,+4).  SIN,COS(CDUY,Z,X) ARE IN
# SINCDU,+2,+4 AND COSCDU,+2,+4 RESPECTIVELY,SCALED TO ONE HALF.  CDU INCREENTS ARE PLACED IN DCDU,+2,+4 SCALED TO
# ONE REVOLUTION.

#	*  COS(IGA)SEC(MGA)		0		-SIN(IGA)SEC(MGA) *
#	*								  *
#	* -COS(IGA)TAN(MGA)		1		 SIN(IGA)TAN(MGA) *
#	*								  *
#	*      SIN(IGA)			0		     COS(IGA)     *

		BANK	14
		SETLOC	POWFLIT1
		BANK

; ============================================================================
; SMCDURES - Stable Member to CDU Angle Conversion
;
; MISSION CONTEXT:
; During powered flight maneuvers (SPS burns for translunar injection, lunar
; orbit insertion, transearth injection), the spacecraft attitude changes
; continuously. The IMU stable member platform remains inertially fixed while
; the gimbals (measured by CDUs) rotate. This routine converts desired attitude
; changes in stable member coordinates into required gimbal angle changes.
;
; FUNCTION:
; Computes incremental CDU (gimbal) angle changes from incremental rotations
; about the stable member axes. This inverse transformation is essential for
; attitude control during engine burns when the autopilot must command gimbal
; movements to achieve desired spacecraft orientation changes.
;
; INPUTS:
;   DTHETASM, +2, +4  - Incremental rotations about SM axes (scaled: 1 rev)
;   SINCDU, +2, +4    - Sin of current CDU angles Y, Z, X (scaled: 0.5)
;   COSCDU, +2, +4    - Cos of current CDU angles Y, Z, X (scaled: 0.5)
;
; OUTPUTS:
;   DCDU, +2, +4      - Required CDU angle increments (scaled: 1 revolution)
;
; ALGORITHM:
; Implements the transformation matrix shown above, which accounts for gimbal
; coupling effects. When the middle gimbal is near 90 degrees, the outer and
; inner gimbals become colinear (gimbal lock), requiring secant and tangent
; terms that amplify small attitude errors into large gimbal commands.
; ============================================================================

SMCDURES	DLOAD	DMP		; Load DTHETASM (X-axis increment)
			DTHETASM	; Multiply by cos(CDUY) for first term of DCDU
			COSCDUY

		PDDL	DMP		; Push result, load DTHETASM+4 (Z-axis increment)
			DTHETASM +4	; Multiply by sin(CDUY) for coupling term
			SINCDUY

		BDSU			; Subtract: cos(Y)*dX - sin(Y)*dZ
		DDV			; Divide by cos(CDUZ) [secant(Z) term]
			COSCDUZ
		STORE	DCDU		; Store result: DCDU (outer gimbal increment)

		DMP	SL1		; Multiply by sin(CDUZ), scale left 1	# SCALE
			SINCDUZ		; [tangent(Z) coupling term]
		BDSU			; Subtract from DTHETASM+2 (Y-axis increment)

			DTHETASM +2	; Result: middle gimbal increment
		STODL	DCDU +2		; Store DCDU+2, load DTHETASM again
			DTHETASM

		DMP	PDDL		; Multiply by sin(CDUY), push result
			SINCDUY
			DTHETASM +4	; Load Z-axis increment

		DMP	DAD		; Multiply by cos(CDUY), add to stacked term
			COSCDUY		; Result: sin(Y)*dX + cos(Y)*dZ
		SL1			; Scale left 1 for proper revolution scaling
		STORE	DCDU +4		; Store DCDU+4 (inner gimbal increment)
		RVQ			; Return to caller with gimbal increments computed
