# Copyright:	Public domain.
# Filename:	S-BAND_ANTENNA_FOR_LM.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	486-489
# Mod history:	2009-05-17 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-07 RSB	Corrected a misprint.
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
; FILE: S-BAND_ANTENNA_FOR_LM.agc
; MODULE: LM Communication Systems
; MISSION PHASE: lunar-orbit/descent/landing/ascent/rendezvous
;
; TL;DR: Computes S-band high-gain antenna pointing angles (pitch and yaw) to
;        maintain communications link between Lunar Module and Earth. Called by
;        astronaut during lunar orbit, surface operations, and ascent phases to
;        ensure telemetry, voice communications, and tracking data reach ground
;        stations. Automatically selects Earth or Moon reference frame based on
;        spacecraft position relative to lunar sphere of influence.
;
; COMMENT-ONLY READERS: This routine ensures Eagle can "phone home" to Mission
;        Control. Follow the comments to understand how the computer calculates
;        where to point the dish antenna so Earth is always in view.
; CODE-ALONG READERS: Study the coordinate frame transformations (Earth-fixed
;        vs Moon-fixed), vector geometry for gimbal angle computation, and the
;        sphere of influence test that determines reference frame selection.
; ============================================================================

# Page 486
# SUBROUTINE NAME: R05 - S-BAND ANTENNA FOR LM
#
# MOD0 BY T. JAMES
# MOD1 BY P. SHAKIR
#
# FUNCTIONAL DESCRIPTION
#
#     THE S-BAND ANTENNA ROUTINE, R05, COMPUTES AND DISPLAYS THE PITCH AND
# YAW ANTENNA GIMBAL ANGLES REQUIRED TO POINT THE LM STEERABLE ANTENNA
# TOWARD THE CENTER OF THE EARTH. THIS ROUTINE IS SELECTED BY THE ASTRO-
# NAUT VIA DSKY ENTRY DURING COASTING FLIGHT OR WHEN THE LM IS ON THE MOON
# SURFACE. THE EARTH OR MOON REFERENCE COORDINATE SYSTEM IS USED DEPENDING
# ON WHETHER THE LM IS ABOUT TO ENTER OR HAS ALREADY ENTERED THE MOON
# SPHERE OF INFLUENCE, RESPECTIVELY
#
# TO CALL SUBROUTINE, ASTRONAUT KEYS IN V 64 E
#
# SUBROUTINES CALLED-
#	R02BOTH
#	INTPRET
#	LOADTIME
#	LEMCONIC
#	LUNPOS
#	CDUTRIG
#	*SMNB*
#	BANKCALL
#	B50OFF
#	ENDOFJOB
#	BLANKET
#
# RETURNS WITH
#	PITCH ANGLE IN PITCHANG 	REV. B0
#	YAW ANGLE IN YAWANG		REV. B0
#
# ERASABLES USED
#	PITCHANG
#	YAWANG
#	RLM
#	VAC AREA

; ============================================================================
; S-BAND ANTENNA POINTING COMPUTATION - ROUTINE R05
;
; The Lunar Module's steerable S-band antenna must continuously point toward
; Earth to maintain communications. This routine computes the required antenna
; gimbal angles based on the spacecraft's current position and attitude.
;
; During Apollo 11, this routine was critical during lunar orbit when Eagle
; separated from Columbia, during the descent to Tranquility Base, while
; Armstrong and Aldrin were on the surface, and during ascent back to orbit.
; Without proper antenna pointing, Mission Control would lose telemetry and
; voice communications with the crew.
;
; Astronaut invocation: Verb 64 Enter (V64E) on the DSKY
; ============================================================================

		BANK	41
		SETLOC	SBAND
		BANK

		EBANK=	WHOCARES
		COUNT*	$$/R05

; Entry point for S-band antenna pointing routine.
; First action: Verify the Inertial Measurement Unit (IMU) is powered on
; and properly aligned. Antenna pointing requires accurate attitude data.

SBANDANT	TC	BANKCALL
# Page 487
		CADR	R02BOTH		# CHECK IF IMU IS ON AND ALIGNED

; ============================================================================
; POSITION VECTOR COMPUTATION
;
; To point the antenna at Earth, the computer must first determine the LM's
; current position. This section:
; 1. Loads the current mission time
; 2. Uses conic orbital integration to compute spacecraft position
; 3. Determines whether to use Earth or Moon reference frame
;
; The "sphere of influence" concept: When close to Earth, Earth's gravity
; dominates and Earth-fixed coordinates are used. Near the Moon, the Moon's
; gravity dominates and Moon-fixed coordinates are used. The LEMCONIC routine
; (defined in ORBITAL_INTEGRATION.agc) makes this determination.
; ============================================================================

		TC	INTPRET		; Enter interpretive mode for vector math
		SETPD	RTB		; Set pushdown list pointer, return to basic
			0D		; Initialize pushdown pointer to 0
			LOADTIME	# PICK UP CURRENT TIME
		STCALL	TDEC1		# ADVANCE INTEGRATION TO TIME IN TDEC1
			LEMCONIC	# USING CONIC INTEGRATION

; LEMCONIC returns X2 register indicating sphere of influence:
; X2 = 0: Spacecraft in Earth's sphere of influence (use Earth-centered coords)
; X2 = 2: Spacecraft in Moon's sphere of influence (use Moon-centered coords)

		SLOAD	BHIZ		; Load X2, branch if X2 = 0 (Earth sphere)
			X2		# X2 =0 EARTH SPHERE, X2 =2 MOON SPHERE
			CONV4		; Earth sphere: skip to CONV4

; ============================================================================
; MOON SPHERE OF INFLUENCE CASE (X2 = 2)
;
; When in lunar orbit or on the Moon's surface, the spacecraft position is
; computed relative to the Moon's center. To point the antenna at Earth, we
; need the Earth's position relative to the spacecraft.
;
; Mathematical approach:
; 1. Get spacecraft position relative to Moon (RATT) -> store in RLM
; 2. Get Moon position relative to Earth (LUNPOS provides unit vector)
; 3. Scale Moon position by Earth-Moon distance (REMDIST)
; 4. Vector addition: R_earth_to_spacecraft = R_earth_to_moon + R_moon_to_spacecraft
;
; This was crucial during Eagle's time in lunar orbit and on the surface.
; ============================================================================

		VLOAD			; Moon sphere: load spacecraft position
			RATT		; Position vector relative to Moon
		STODL	RLM		; Store in RLM, load time into accumulator
			TAT		; Time of state vector

CONV3		CALL
			LUNPOS		# UNIT POSITION VECTOR FROM EARTH TO MOON

; LUNPOS (from PLANETARY_INERTIAL_ORIENTATION.agc) returns unit vector from
; Earth to Moon. Must scale by actual Earth-Moon distance for position.

		VLOAD	VXSC		; Load Moon position unit vector, scale
			VMOON		; Unit vector from Earth to Moon
			REMDIST		# MEAN DISTANCE FROM EARTH TO MOON
		VSL1	VAD		; Shift left 1 bit, add vectors
			RLM		; Add spacecraft-to-Moon vector
		GOTO			; Continue to Earth vector computation
			CONV5

; ============================================================================
; EARTH SPHERE OF INFLUENCE CASE (X2 = 0)
;
; When in Earth orbit or cislunar space before entering Moon's sphere of
; influence, spacecraft position is already Earth-centered. Simply use
; the position vector directly.
; ============================================================================

CONV4		VLOAD			; Earth sphere case
			RATT		# UE = -UNIT(RATT)  EARTH SPHERE

; ============================================================================
; COMPUTE UNIT VECTOR TO EARTH
;
; Whether from Earth or Moon sphere calculations, we now have a position
; vector. Convert to unit vector pointing toward Earth (needed for antenna
; pointing geometry). The VCOMP (vector complement) makes it point FROM
; spacecraft TO Earth (negates the vector).
; ============================================================================

CONV5		SETPD	UNIT		# UE = -UNIT((REM)(UEM) + RL)  MOON SPHERE
			0D		# SET PL POINTER TO 0
		VCOMP	CALL		; Complement vector: spacecraft-to-Earth
			CDUTRIG		# COMPUTE SINES AND COSINES OF CDU ANGLES

; CDUTRIG computes sines/cosines of IMU gimbal angles from Coupling Data Units.
; This provides current spacecraft attitude needed for coordinate transformation.

; ============================================================================
; COORDINATE FRAME TRANSFORMATIONS
;
; The Earth direction vector is currently in reference coordinates (inertial
; space). Must transform through multiple coordinate frames:
; 1. Reference -> Stable Member (IMU platform orientation)
; 2. Stable Member -> Navigation Base (spacecraft body frame)
; 3. Navigation Base -> Antenna coordinates (gimbal mounting frame)
;
; REFSMMAT = Reference to Stable Member Matrix
; *SMNB* = Stable Member to Navigation Base transformation
; ============================================================================

		MXV	VSL1		# TRANSFORM REF. COORDINATE SYSTEM TO
			REFSMMAT	# STABLE MEMBER  B-1 X B-1 X B+1 = B-1
		PUSH	DLOAD		# 8D
			HI6ZEROS	; Load zero constant
		STORE	PITCHANG	; Initialize pitch angle to zero
		STOVL	YAWANG		# ZERO OUT ANGLES

; Before computing final gimbal angles, must transform from stable member
; coordinates to spacecraft body (navigation base) coordinates. The *SMNB*
; routine applies the transformation matrix based on current IMU alignment.

		CALL
			*SMNB*		; Stable Member to Navigation Base transform
		STODL	RLM		# PRE-MULTIPLY RLM BY (NBSA) MATRIX(B0)
			RLM 	+2	; Load Z component of vector

; ============================================================================
; ANTENNA MOUNTING GEOMETRY TRANSFORMATION
;
; The S-band antenna is mounted at 45 degrees to the spacecraft axes. This
; rotation must be accounted for in computing gimbal angles. The following
; sequence rotates the Earth direction vector from spacecraft body frame
; to antenna mounting frame using the constant 1/√2 ≈ 0.707.
;
; Mathematical transformation:
; X_antenna = (X_body + Z_body) / √2
; Z_antenna = (Z_body - X_body) / √2
; Y_antenna = Y_body (unchanged)
; ============================================================================

		PUSH	DSU		; Save Z, compute Z - X
			RLM		; Subtract X component
		DMP			; Multiply by 1/√2
			1OVSQRT2	; Constant = 0.707106781...
		STODL	RLM 	+2	; Store new Z component
		DAD	DMP		; Add Z + X
			RLM		; Add X component
			1OVSQRT2	; Multiply by 1/√2
		STOVL	RLM		# R  B-1
			RLM		; Load transformed vector
		UNIT	PDVL		; Normalize to unit vector, push, load
# Page 488
			RLM		; Reload for next operation

; ============================================================================
; PITCH ANGLE COMPUTATION
;
; The pitch angle is the angle of the antenna in the vertical plane. To
; compute this, we:
; 1. Project Earth direction vector onto the spacecraft XZ plane (removing
;    Y component)
; 2. Compute cross products to determine the pitch rotation required
; 3. Use arcsine to convert vector components to angle
;
; Pitch angle = angle between antenna boresight and projection of Earth
; direction onto the XZ plane. Stored in PITCHANG (units: revolutions, B0).
; ============================================================================

		VPROJ	VSL2		# PROJECTION OF R ONTO LM XZ PLANE
			HIUNITY		; Project perpendicular to Y axis
		BVSU	BOV		# CLEAR OVERFLOW INDICATOR IF ON
			RLM		; Subtract from original vector
			COVCNV		; Continue if overflow occurred

COVCNV		UNIT	BOV		# EXIT ON OVERFLOW
			SBANDEX		; Exit routine if normalization overflow

; URP = Unit vector in Reference Plane (XZ plane projection, normalized)
; This represents the Earth direction with Y component removed.

		PUSH	VXV		# URP VECTOR  B-1
			HIUNITZ		; Cross product with Z unit vector
		VSL1	VCOMP		# UZ X URP = -(URP X UZ)
		STORE	RLM		# X VEC  B-1

; The cross product gives a vector perpendicular to both URP and Z axis.
; Its dot product with Y determines the sign of the pitch angle.

		DOT	PDVL		# SGN(X.UY) UNSCALED
			HIUNITY		; Dot with Y unit vector
			RLM		; Reload computed X vector
		ABVAL	SIGN		; Take absolute value, apply sign
		ASIN			# ASIN((SGN(X.UY))ABV(X)) REV B0
		STOVL	PITCHANG	; Store pitch angle, load URP

; ============================================================================
; YAW ANGLE COMPUTATION
;
; The yaw angle is the rotation in the horizontal plane. Computed by examining
; the Earth direction vector's position relative to the Z axis in the XZ plane.
; ============================================================================

		PUSH			; Save URP vector
		DOT	BPL		; Dot product with Z axis, branch if positive
			HIUNITZ		; Z unit vector
			NOADJUST	# YES, -90 TO +90

; If URP.Z is negative, Earth is behind the spacecraft. Must adjust pitch
; angle to account for the antenna pointing backward (add 180 degrees = 0.5 rev).

		DLOAD	DSU		; Load half revolution constant
			HIDPHALF	; 0.5 revolutions = 180 degrees
			PITCHANG	; Subtract current pitch angle
		STORE	PITCHANG	; Store adjusted pitch (180° - pitch)

; ============================================================================
; YAW ANGLE VECTOR COMPUTATION
;
; With pitch angle determined, now compute yaw angle (rotation in horizontal
; plane). The yaw computation requires:
; 1. Creating a reference vector perpendicular to both UR and URP
; 2. Accounting for pitch rotation already computed
; 3. Computing rotation angle in the horizontal plane
; ============================================================================

NOADJUST	VLOAD	VXV		; Load UR, cross with URP
			UR		# Z = (UR X URP)
			URP		; Reference plane unit vector
		VSL1			; Shift left 1 bit (normalize)
		STODL	RLM		# Z VEC  B-1
			PITCHANG	; Load pitch angle for rotation

; Apply pitch rotation to compute yaw reference frame. The following
; constructs a vector rotated by the pitch angle in the vertical plane:
; Rotated_vector = (UX * cos(pitch)) - (UZ * sin(pitch))

		SIN	VXSC		; sin(pitch), scale Z unit vector
			HIUNITZ		; Z unit vector
		PDDL	COS		; Push result, load cos(pitch)
			PITCHANG	; Pitch angle in revolutions
		VXSC	VSU		; Scale X by cos(pitch), subtract
			HIUNITX		# (UX COS ALPHA) - (UZ SIN ALPHA)

; Now compute yaw angle by dotting the rotated vector with the perpendicular
; Z vector computed earlier. This gives the sine of the yaw angle.

		DOT	PDVL		# YAW.Z
			RLM		; Perpendicular Z vector
			RLM		; Reload Z vector
		ABVAL	SIGN		; Absolute value, preserve sign
		ASIN			; Arcsine to get yaw angle (revolutions)
		STORE	YAWANG		; Store yaw angle in YAWANG (rev B0)
; ============================================================================
; EXIT AND DISPLAY ROUTINE
;
; With pitch and yaw angles computed, exit interpretive mode and display
; results to crew via DSKY. The routine provides three options:
; - TERMINATE: End the routine and turn off extended verb activity
; - PROCEED: Accept displayed angles and turn off extended verb
; - RECYCLE: Recompute and update angles continuously
;
; During lunar orbit and surface operations, the antenna angles change as the
; spacecraft moves relative to Earth. Continuous updating ensures the high-gain
; antenna maintains optimal pointing for voice and telemetry communications.
; ============================================================================

SBANDEX		EXIT			; Exit interpretive mode, return to AGC native code
		CA	EXTVBACT	; Load extended verb activity word
		MASK	BIT5		# IS BIT5 STILL ON
		EXTEND			; Extend next instruction
		BZF	ENDEXT		# NO - exit if extended verb not active

; Extended verb is still active, proceed with display update.

		CAF	PRIO5		; Load priority 5 for display job
# Page 489
		TC	PRIOCHNG	; Change job priority
		CAF	V06N51		# DISPLAY ANGLES
		TC	BANKCALL	; Bank call to display routine
		CADR	GOMARKFR	; Address of mark display routine

; Crew response options on DSKY:
; KEY REL (terminate), PRO (proceed), or ENTR (recycle)

		TC	B5OFF		# TERMINATE - turn off bit 5, end routine
		TC	B5OFF		# PROCEED - turn off bit 5, accept angles
		TC	ENDOFJOB	# RECYCLE - end current job, will restart

; If recycling (ENTR pressed), prepare for continuous update mode.

		CAF	BIT3		# IMMEDIATE RETURN
		TC	BLANKET		# BLANK R3 - clear register 3 display
		CAF	PRIO4		; Set priority 4 for recycle job
		TC	PRIOCHNG	; Change priority
		TC	SBANDANT +2	# YES, CONTINUE DISPLAYING ANGLES

; ============================================================================
; CONSTANTS AND DEFINITIONS
; ============================================================================

V06N51		VN	0651		; Verb 06, Noun 51: Display antenna angles
					; Displays PITCHANG in R1, YAWANG in R2
1OVSQRT2	2DEC	.7071067815	# 1/SQRT(2) - constant for 45-degree rotation
					; Used to transform antenna mounting geometry

; ============================================================================
; VECTOR STORAGE LOCATION ASSIGNMENTS
;
; These EQUALS directives map vector names to pushdown list locations.
; The pushdown list (MPAC stack) provides temporary storage during
; interpretive operations.
; ============================================================================

UR		EQUALS	0D		; Unit vector to Earth (location 0D in pushdown)
URP		EQUALS	6D		; Unit Reference Plane vector (location 6D)
		SBANK=	LOWSUPER	; Super bank assignment

# *** END OF LNYAIDE .001 ***

; ============================================================================
; ROUTINE SUMMARY FOR COMMENT-ONLY READERS:
;
; This routine (R05, invoked by astronaut command V 64 E) calculates the
; pitch and yaw angles needed to point the Lunar Module's steerable S-band
; high-gain antenna toward Earth. During Apollo 11's mission, maintaining
; this communication link was critical for voice communications and telemetry
; transmission.
;
; The routine automatically accounts for:
; - Whether the LM is in the Earth or Moon sphere of influence
; - The current position and orientation of the spacecraft
; - The antenna's 45-degree mounting angle on the LM structure
;
; The computed angles are displayed on the DSKY, allowing the crew to:
; - Manually position the antenna using the displayed angles
; - Verify automatic antenna positioning system operation
; - Monitor Earth tracking during lunar orbit and surface operations
;
; During the historic Apollo 11 mission, this antenna pointed from the lunar
; surface back to Earth across 238,855 miles of space, carrying Armstrong's
; famous words: "The Eagle has landed."
; ============================================================================
