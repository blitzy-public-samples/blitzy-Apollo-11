# Copyright:	Public domain.
# Filename:	AOTMARK.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	244-261
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Luminary131/ file of the same
#				name, using Luminary099 page images.
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
; FILE: AOTMARK.agc
; MODULE: Optical Navigation and IMU Alignment
; MISSION PHASE: lunar-orbit/descent/landing/ascent
;
; TL;DR: Processes Alignment Optical Telescope (AOT) mark data for Lunar
;        Module IMU alignment and navigation updates. Handles crew star
;        sightings through the AOT reticle, computes star vectors from crew
;        input, manages both inflight and lunar surface marking modes, and
;        interfaces with alignment programs P51-P53 to maintain accurate
;        spacecraft attitude knowledge throughout lunar operations.
;
; COMMENT-ONLY READERS: This module enables astronauts to sight stars through
;        the LM's optical telescope for navigation. Follow the marking process
;        from crew input through vector computation and alignment integration.
; CODE-ALONG READERS: Study AOT geometry transformations, interrupt-driven
;        mark capture, VAC area management, and dual-mode operation (inflight
;        vs surface) for optical navigation system implementation.
; ============================================================================

# Page 244
		BANK	12
		SETLOC	AOTMARK1
		BANK

		EBANK=	XYMARK
		COUNT*	$$/MARK

; ============================================================================
; AOTMARK - ALIGNMENT OPTICAL TELESCOPE MARK ENTRY POINT
;
; The astronaut initiates an optical mark by sighting a star through the
; AOT reticle and pressing the mark button. This routine coordinates the
; entire marking process, from capturing the initial button press through
; computing the star vector and integrating it with the navigation system.
;
; The LM's AOT is a fixed-position telescope with a rotating reticle that
; allows the crew to sight celestial objects for navigation updates. Unlike
; the Command Module's scanning telescope and sextant, the LM uses this
; simpler optical system optimized for lunar orbit and surface operations.
; ============================================================================

AOTMARK		INHINT
; Mark system resource protection check. The AGC can only process one
; marking operation at a time due to shared computational resources and
; the need for uninterrupted star vector calculations.
		CCS	MARKSTAT	# SEE IF AOTMARK BUSY
		TC	+2		# MARK SYSTEM BUSY -- DO ALARM
		TC	EXTVBCHK
; Alarm 105: AOTMARK system already in use. Crew must complete current
; marking operation before initiating another star sighting.
		TC	POODOO
		OCT	00105

; Extended verb conflict checking. Some extended verbs (crew-initiated
; special functions) are incompatible with marking operations and must
; be blocked during star sightings to prevent data corruption.
EXTVBCHK	CAF	SIX		# SEE IF EXT. VERB WORKING
		MASK	EXTVBACT
		CCS	A
		TCF	MKABORT		# YES -- ABORT

; Mark system has priority - lock out conflicting extended verbs during
; the marking sequence. This prevents crew from inadvertently disrupting
; time-critical optical measurements.
		CAF	BIT2		# NO -- DISALLOW SOME EXTENDED VERB ACTION
		ADS	EXTVBACT	# BIT2 RESET IN ENDMARK

; ============================================================================
; TRANSITION: From mark initiation to computational resource allocation
;
; With conflicts resolved, the AGC now allocates a VAC (Vector Accumulator)
; area for the marking computation. The AGC has five VAC areas that provide
; temporary storage for vector/matrix operations. Finding an available VAC
; is essential - if all are occupied, the mark must be aborted.
; ============================================================================

; Sequential search through all five VAC areas looking for an available
; resource. Each CCS (Count, Compare, Skip) tests if a VAC is in use.
MKVAC		CCS	VAC1USE		# LOOK FOR A VAC AREAD -- DO ABORT IF
		TCF	MKVACFND	# NONE AVAILABLE
		CCS	VAC2USE
		TCF	MKVACFND
		CCS	VAC3USE
		TCF	MKVACFND
		CCS	VAC4USE
		TCF	MKVACFND
		CCS	VAC5USE
		TCF	MKVACFND
; All five VAC areas occupied - computational resources exhausted.
; This is a rare condition indicating heavy AGC workload.
		DXCH	BUF2
		TC	BAILOUT1	# ALL VAC AREAS OCCUPIED -- ABORT.
		OCT	01207

; VAC area found - store its address in MARKSTAT for later reference
; and mark it as occupied. The VAC area will hold intermediate vector
; computations throughout the marking process.
MKVACFND	AD	TWO
		TS	MARKSTAT        # STORE VAC ADR IN LOW 9 OF MARKSTAT

		CAF	ZERO
		INDEX	MARKSTAT
		TS	0 -1		# ZERO IN VACUSE REG TO SHOW VAC OCCUPIED

; Launch background job to process mark data. GETDAT will read crew inputs
; (shaft and trunnion angles from AOT reticle), compute star line-of-sight
; vector, and integrate with navigation state.
		CAF	PRIO15
		TC	FINDVAC		# SET UP JOB FOR GETDAT
		EBANK=	XYMARK
		2CADR	GETDAT

; Re-enable interrupts and return to calling program. The marking process
; continues asynchronously in the GETDAT job.
		RELINT
		TCF	SWRETURN

# Page 245
; Alarm 1211: Extended verb conflict prevents mark operation. Crew must
; wait for current extended verb operation to complete before marking.
MKABORT		DXCH	BUF2
		TC	BAILOUT1	# CONFLICT WITH EXTENDED VERB
		OCT	01211

; Mark sequence termination and resource cleanup. Called when marking
; completes successfully or when crew terminates the operation.
MKRELEAS	CAF	ZERO
		XCH	MARKSTAT	# SET MARKSTAT TO ZERO
		MASK	LOW9		# PICK UP VAC AREA AOR
		CCS	A
		INDEX	A
		TS	0		# SHOW MKVAC AREA AVAILABLE
; Wake up the program that initiated marking (typically P51, P52, or P53).
		CAF	ONE
		TC	IBNKCALL
		CADR	GOODEND		# GO WAKE UP CALLING JOB

# Page 246
; Emergency termination of marking process. Used when crew aborts alignment
; program or when system reset required.
KILLAOT		CAF	ZERO
		TS	EXTVBACT	# TERMINATE AOTMARK -- ALLOW EXT VERB
		TC	GOTOPOOH

; ============================================================================
; GETDAT - RETRIEVE MARK DATA FROM CREW INPUT
;
; This routine reads the astronaut's AOT reticle settings (shaft and trunnion
; angles) from the DSKY and prepares to capture the precise time and attitude
; when the mark button is pressed. During Apollo 11's lunar orbit operations,
; Buzz Aldrin used the AOT to sight navigation stars, carefully rotating the
; reticle to center each star before marking.
; ============================================================================

GETDAT		CS	MARKSTAT	# SET BIT12 TO DISCOURAGE MARKRUPT
		MASK	BIT12		#	BIT12 RESET AT GETMARK
		ADS	MARKSTAT

; Request crew input via DSKY. V01N71 displays the AOT detent position
; (which of the 6 fixed telescope positions is being used) and star code
; from the onboard star catalog.
		CAF	V01N71		# DISPLAY DETENT AND STAR CODE
		TC	BANKCALL
		CADR	GOMARKF

		TCF	KILLAOT		# V34 -- DOES GOTOPOOH
		TCF	DODAT		# V33 -- PROCEED -- USE THIS STAR FOR MARKS
ENTERDAT	TCF	GETDAT		# ENTER -- REDISPLAY STAR CODE

; ============================================================================
; DODAT - PROCESS AOT DETENT CODE AND STAR SELECTION
;
; The LM's AOT has 6 fixed detent positions spaced around the spacecraft,
; each providing a different field of view. The crew selects which detent
; to use and identifies the target star. This routine validates the selection
; and retrieves the optical axis geometry for that detent position.
; ============================================================================

; Validate detent code. The AOT has 6 fixed positions (detents 1-6) plus
; a special code 7 for COAS (Crew Optical Alignment Sight) backup system.
; Invalid codes cause redisplay.
DODAT		CAF	HIGH9		# PICK DETENT CODE FROM BITS7-9 OF AOTCODE
		MASK	AOTCODE		# AND SEE IF CODE 1 TO 6
		EXTEND
		MP	BIT9
		TS	XYMARK		# STORE DETENT

		EXTEND
		BZMF	GETDAT		# COAS CALIBRATION CODE - NO GOOD HERE

		AD	NEG7		# SEE IF DETENT 7 FOR COAS
		EXTEND
		BZF	CODE7

		TCF	CODE1TO6

; CODE7 - COAS (Crew Optical Alignment Sight) backup mode
; The COAS is a simple backup optical device without fixed detents. The crew
; manually provides azimuth and elevation angles. This was critical redundancy
; in case the AOT telescope failed.
CODE7		CAF	V06N87*		# CODE 7, COAS SIGHTING, GET OPTIC AXIS
		TC	BANKCALL	# AZ AND EL OF SIGHTING DEVICE FROM ASTRO
		CADR	GOMARKF

		TCF	KILLAOT		# V34 -- DOES GOTOPOOH
		TCF	+2		# PROCEED
		TCF	CODE7		# ON ENTER, RECYCLE
		EXTEND
		DCA	AZ		# PICK UP AZ AND EL IN SP 25 COMP
		INDEX	FIXLOC
		DXCH	8D		# STORE IN 8D AND 9D OF LOCAL VAC
		CAF	ZERO		# BACKUP SYSTEM TO BE USED
		TCF	COASCODE	# ZERO APPARENT ROTATION

; CODE1TO6 - Normal AOT detent positions (1 through 6)
; Each detent has pre-calibrated azimuth and elevation angles stored in
; AOTAZ and AOTEL tables. Detents on left and right sides of the spacecraft
; have apparent field rotation that must be compensated.
CODE1TO6	INDEX	XYMARK		# INDEX AOT POSITION BY DET CODE
		CA	AOTEL -1
		INDEX	FIXLOC
		TS	9D		# STORE ELEVATION IN VAC+9D

		INDEX	XYMARK		# INDEX DET CODE 1,2 OR 3
# Page 247
		CA	AOTAZ -1
		INDEX	FIXLOC
		TS	8D		# STORE AZIMUTH IN VAC +8D

; Compute field rotation compensation. Left and right detents experience
; apparent rotation of the reticle pattern due to optical geometry.
		CA	AOTAZ +1	# COMPENSATION FOR APPARENT ROTATION OF
		EXTEND			# AOT FIELD OF VIEW IN LEFT AND RIGHT
		INDEX	FIXLOC		# DETENTS IS STORED IN VAC +10D IN SP
		MSU	8D		# PRECISION ONE'S COMPLEMENT
COASCODE	INDEX	FIXLOC
		TS	10D		# ROT ANGLE

		TC	INTPRET		# COMPUTE X AND Y PLANE VECTORS

# Page 248
; ============================================================================
; OPTAXIS - COMPUTE OPTICAL AXIS AND MARK PLANE VECTORS
;
; This subroutine transforms the crew's AOT sighting angles (azimuth and
; elevation) into three-dimensional unit vectors representing the optical
; axis and the X/Y axes of the mark plane. The mark plane is the virtual
; plane perpendicular to the line of sight containing the reticle pattern.
; Field rotation correction accounts for how the reticle appears to rotate
; in left and right detents due to spacecraft geometry.
; ============================================================================
#
# THE OPTAXIS SUBROUTINE COMPUTES THE X AND Y MARK PLANE VECS AND
# ROTATES THEM THRU THE APPARENT FIELD OF VIEW ROTATION UNIQUE TO AOT
# OPTAXIS USES OANB TO COMPUTE THE OPTIC AXIS
#
#	INPUT --	AZIMUTH ANGLE IN SINGLE PREC AT CDU SCALE IN 8D OF JOB VAC
#			ELEVATION ANGLE IN SINGLE PREC AT CDU SCALE IN 9D OF JOB VAC
#			ROTATION ANGLE IN SINGLE PREC IS COMP SCALED BY PI IN 10D OF VAC
#
#	OUTPUT --	OPTIC AXIS VEC IN NG COORDS IN SCAXIS
#			X-MARK PLANE 1/4VEC IN NB COORDS AT 18D OF JOB VAC
#			Y-MARK PLANE 1/4VEC IN NB COORDS AT 12D OF JOB VAC

; First compute the basic optic axis and unrotated mark plane vectors from
; azimuth and elevation angles. OANB returns these in navigation base coords.
OPTAXIS		CALL			# GO COMPUTE OA AN X AND Y PLANE VECS
			OANB
		
; Apply field rotation correction. The reticle pattern appears rotated in
; detents 1, 2, 5, and 6. Rotation transformation uses standard 2D formula:
; X' = cos(θ)X + sin(θ)Y
; Y' = cos(θ)Y - sin(θ)X
		SLOAD	SR1		# LOAD APP ROTATION IN ONES COMP
			10D		# RESCALE BY 2PI
		PUSH	SIN		# 1/2SIN(ROT) 0-1
		PDDL	COS
		PUSH	VXSC		# 1/2COS(ROT) 2-3
			18D
		PDDL	VXSC		# 1/4COS(ROT)UYP 4-9
			0
			24D		# 1/4SIN(ROT)UXP
		BVSU	STADR		# UP 4-9
		STODL	12D		# YPNB=1/4(COS(ROT)UYP-SIN(ROT)UXP)
		VXSC	PDDL		# UP 2-3 UP 0-1 FOR EXCHANGE
			24D		# 1/4COS(ROT)UXP 	PUSH 0-5
		VXSC	VAD		# 1/4SIN(ROT)UYP
			18D		# UP 0-5
		STADR
		STOVL	18D		# XPNB=1/4(COS(ROT)UXP+SIN(ROT)UYP)
		
; Initialize star vector accumulator to zero. Multiple marks will be averaged.
			LO6ZEROS	# INITIALIZE AVE STAR VEC ACCUMULATOR
		STORE	STARAD +6
		EXIT
		TCF	GETMKS

# Page 249
; ============================================================================
; OANB - COMPUTE OPTIC AXIS IN NAVIGATION BASE COORDINATES
;
; This subroutine performs spherical-to-Cartesian coordinate transformation,
; converting the crew's azimuth and elevation angles into a unit vector
; pointing along the line of sight. It also computes perpendicular unit
; vectors defining the mark plane's X and Y axes. These vectors are expressed
; in the spacecraft navigation base coordinate system, which is body-fixed
; to the Lunar Module structure.
; ============================================================================
#
# THE OANB SUBROUTINE COMPUTES THE OPTIC AXIS OF THE SIGHTING INSTRUMENT
# FROM AZIMUTH AND ELEVATION INPUT FROM THE ASTRONAUT.
#
#	INPUT --	AZIMUTH ANGLE IN SINGLE PREC 2'S COMP IN 8D OF JOB VAC
#			ELEVATION ANGLE IN SINGLE PREC 2'S COMP IN 9D OF VAC
#
#	OUTPUT --	OPTIC AXIS IN NB COORDS. IN SCAXIS
#			X-PLANE 1/2VEC IN NB COORDS AT 24D OF VAC
#			Y-PLANE 1/2VEC IN NB COORDS AT 18D OF VAC

		BANK	05
		SETLOC	AOTMARK2
		BANK

		COUNT*	$$/MARK

; Spherical-to-Cartesian transformation. Given elevation and azimuth angles,
; compute the optic axis unit vector. Standard transformation equations:
; OA_x = sin(elevation)
; OA_y = cos(elevation) × sin(azimuth)
; OA_z = cos(elevation) × cos(azimuth)
OANB		SETPD	STQ
			0
			GCTR		# STORE RETURN
		SLOAD	RTB
			9D		# PICK UP SP ELV
			CDULOGIC
		PUSH	COS
		PDDL	SIN		# 1/2COS(ELV)	PD 0-1
		STADR
		STODL	SCAXIS		# OAX=1/2SIN(ELV)
			8D
		RTB
			CDULOGIC
		
; Compute Y-plane vector components. This vector is perpendicular to the
; optic axis and defines one axis of the mark plane coordinate system.
		PUSH	COS
		STORE	20D		# STORE UYP(Y) 	20-21
		PDDL	SIN		# 1/2COS(AZ) 	PD 2-3
		PUSH	DCOMP		# PUSH 1/2S IN (AZ)	4-5
		STODL	22D		# STORE UYP(Z)	22-23
			LO6ZEROS
		STODL	18D		# STORE UYP(X)	18-19
		DMP	SL1
			0
		STODL	SCAXIS +2	# OAY=1/2COS(ELV)SIN(AZ)
		DMP	SL1		# UP	2-3
		STADR			# UP	0-1
		STOVL	SCAXIS +4	# OAZ=1/2COS(ELV)COS(AZ)
		
; Compute X-plane vector using cross product. The X and Y plane vectors
; form an orthonormal basis in the mark plane (perpendicular to optic axis).
			18D		# LOAD UYP VEC
		VXV	UNIT
			SCAXIS		# UXP VEC=UYP X OA
		STORE	24D		# STORE UXP
		GOTO
			GCTR
# Page 250
; ============================================================================
; SURFSTAR - COMPUTE STAR VECTOR FOR LUNAR SURFACE ALIGNMENT
;
; When the Lunar Module is on the surface, this routine transforms crew
; sightings into star vectors for IMU alignment. The AOT reticle includes
; a cursor line and spiral pattern. The crew positions the cursor on the
; star, then reads where the star appears on the spiral scale. This provides
; two-dimensional mark data that, combined with spacecraft attitude (CDUs),
; yields a 3D star vector in stable member coordinates.
; ============================================================================
#
# SURFSTAR COMPUTES A STAR VECTOR IN SM COORDINATES FOR LUNAR
# SURFACE ALIGNMENT AND EXITS TO AVEIT TO AVERAGE STAR VECTORS.
#
#	GIVEN	X-MARK PLANE 1/4 VEC IN NB AT 18D OF LOCAL VAC
#		Y-MARK PLANE 1/4 VEC IN NB AT 12D OF LOCAL VAC
#		CURSOR SP 2COMP AT POSITION 1 OF INDEXED MARKVAC
#		SPIRAL SP 2COMP AT POSITION 3 OF INDEXED MARKVAC
#		CDUY,Z,X AT POSITIONS 0,2,4 OF INDEXED MARKVAC

		BANK	15
		SETLOC	P50S
		BANK
		COUNT*	$$/R59

; Begin surface star vector computation by loading gimbal angles.
SURFSTAR	VLOAD*
			0,1		# PUT X-MARK CDUS IN CDUSPOT FOR TRG*NBSM
		STORE	CDUSPOT
		
; Load cursor position (YROT). This is where the crew positioned the
; horizontal cursor line to intersect the star image.
		SLOAD*	RTB
			1,1		# PICK UP YROT
			CDULOGIC
		STORE	24D		# STORE CURSOR FOR SPIRAL COMP (REVS)
		BZE
			YZCHK		# IF YROT ZERO -- SEE IF SROT ZERO
			
; Apply cursor rotation to the mark plane vectors. This rotates the reference
; frame to account for where the cursor was positioned.
JUSTZY		PUSH	COS
		PDDL	SIN		# 1/2COS(YROT) 	0-1
		VXSC	PDDL		# UP 0-1	1/8SIN(YROT)UXP	0-5
			18D
		VXSC	VSU		# UP 	0-5
			12D		# UYP
		UNIT	VXV
			SCAXIS
		UNIT	PUSH
		
; Now process spiral reading. The spiral scale provides angular separation
; from the cursor position. Crew reads where star appears on spiral (0-360°).
		SLOAD*	RTB
			3,1		# PICK UP SPIRAL
			CDULOGIC
		STORE	26D		# STORE SPIRAL (REVS)
		DSU	DAD
			24D
			ABOUTONE
		DMP
			DP1/12
		STORE	26D		# SEP=(360 + SPIRAL -CURSOR)/12
		
; Rotate by separation angle to get final star direction in mark plane.
		SIN	VXSC		# UP	0-5
		VSL1	PDDL		# 1/2SIN(SEP)(UPP X OA)	0-5
			26D
		COS	VXSC
			SCAXIS
		VSL1	VAD		# UP	0-5
		
; Transform from navigation base to stable member coordinates and average.
JUSTOA		UNIT	CALL
			TRG*NBSM
		STCALL	24D		# STAR VEC IN SM
			AVEIT		# GO AVERAGE
# Page 251
ABOUTONE	2DEC	.99999999

DP1/12		EQUALS	DEG30		# .08333333
		BANK	7
		SETLOC	AOTMARK1
		BANK
		COUNT*	$$/MARK
; Handle special case: cursor rotation is zero. If both cursor and spiral
; are zero, star is aligned with optic axis (looking straight through AOT).
YZCHK		SLOAD*	BZE		# YROT ZERO AND IF SROT ZERO FORCE STAR
			3,1		# ALONG OPTIC AXIS
			YSZERO
		DLOAD	GOTO
			24D
			JUSTZY		# SROT NOT ZERO -- CONTINUE NORMALLY
			
; Both cursor and spiral are zero: star vector equals optic axis direction.
YSZERO		VLOAD	GOTO
			SCAXIS
			JUSTOA

# Page 252
; ============================================================================
; GETMKS - INITIALIZE SIGHTING MARK PROCEDURE
;
; This routine begins the mark-taking sequence. It displays V54 on the DSKY,
; prompting the crew to sight stars through the AOT. The crew can:
; - Press ENTER to take a mark (records current AOT detent and reticle data)
; - Press PROCEED (V33) when finished to compute line-of-sight vectors
; - Press TERMINATE (V34) to abort the marking sequence
; ============================================================================
#
# THE GETMKS ROUTINE INITIALIZES THE SIGHTING MARK PROCEDURE

; Zero out mark tracking registers before starting new mark sequence.
GETMKS		CAF	ZERO		# INITIALIZE MARK ID REGISTER AND MARK CNT
		TS	XYMARK
		TS	MARKCNTR
		CAF	LOW9		# ZERO BITS10 TO 15 RETAINING MKVAC ADR
		MASK	MARKSTAT
		TS	MARKSTAT
		
; Display V54 to crew, prompting for mark input via DSKY.
		CAF	MKVB54*		# DISPLAY VB54 INITIALLY
PASTIT		TC	BANKCALL
		CADR	GOMARK4

; Crew response handling:
		TCF	KILLAOT		# V34 -- DOES GOTOPOOH
		TCF	MARKCHEX	# VB33 -- PROCEED, GOT MARKS, COMPUTE LOS
		TCF	GETDAT		# ENTER -- RECYCLE TO V01N71

; Crew pressed PROCEED (V33): mark sequence complete, compute star vectors.
; Set BIT12 high to prevent MARKRUPT interrupt from interfering with processing.
MARKCHEX	CS	MARKSTAT	# SET BIT12 TO DISCOURAGE MARKRUPT
		MASK	BIT12
		ADS	MARKSTAT
		MASK	LOW9
		TS	XYMARK		# JAM MARK VAC ADR IN XYMARK FOR AVESTAR
		CAF	ZERO
		TS	MKDEX		# SET MKDEX ZERO FOR LOS VEC CNTR
		
; Check if the last mark data is complete before proceeding to averaging.
		CA	MARKSTAT
		MASK	PRIO3		# SEE IF LAST MK PART COMPLETE
		TS	L
		CAF	PRIO3		# BITS10 AND 11
		EXTEND
		RXOR	LCHAN
		EXTEND
		BZF	AVESTAR		# LAST PAIR COMPLETE -- TO COMPUTE LOS
CNTCHK		CCS	MARKCNTR	# NO PAIR SHOWING -- SEE IF PAIR IN HOLD
		TCF	+2		# PAIR BURIED -- DECREMENT COUNTER
		TCF	MKALARM		# NO PAIR -- ALARM
		TS	MARKCNTR	# STORE DECREMENTED COUNTER

; ============================================================================
; AVESTAR - AVERAGE STAR VECTORS
;
; After all marks are taken and individual line-of-sight vectors computed,
; AVESTAR averages them to produce a final observed star direction. This
; reduces random sighting errors and improves IMU alignment accuracy.
;
; For inflight marks: averages multiple sightings of the same star.
; For surface marks: processes the single computed star vector.
; ============================================================================

AVESTAR		CAF	BIT12		# INITIALIZE MKDEX FOR STAR LOS COUNTER
		ADS	MKDEX		# MKDEX WAS INITIALIZED ZERO IN MARKCHEX
		CS	MARKCNTR
		EXTEND
		MP	SIX		# GET C(L) = -6 MARKCNTR
		CS	XYMARK
		AD	L		# ADD -- MARK VAC ADR SET IN MARKCHEX
		INDEX	FIXLOC
		TS	X1		# JAM -- CDU ADR OF X-MARK IN X1

		CA	FIXLOC		# SET PD POINTER TO ZERO
		TS	PUSHLOC

; Enter interpreter mode to perform vector mathematics for star LOS computation.
		TC	INTPRET
# Page 253
; Check if on lunar surface. If so, jump to SURFSTAR computation which handles
; the spiral reticle data differently. Otherwise, proceed with inflight computation.
		BON	VLOAD*
			SURFFLAG	# IF ON SURFACE COMPUTE VEC AT SURFSTAR
			SURFSTAR
			
; INFLIGHT STAR VECTOR COMPUTATION:
; Convert both X-plane and Y-plane marks from navigation base to stable member
; coordinates, then compute their cross product to get star line-of-sight.
			1,1		# PUT Y-MARK CDUS IN CDUSPOT FOR TRG*NBSM
		STOVL	CDUSPOT
			12D		# LOAD Y-PLANE VECTOR IN NG
		CALL
			TRG*NBSM	# CONVERT IT TO STABLE MEMBER
		PUSH	VLOAD*
			0,1		# PUT X-MARK CDUS IN CDUSPOT FOR TRG*NBSM
		STOVL	CDUSPOT
			18D		# LOAD X-PLANE VECTOR IN NB
		CALL
			TRG*NBSM	# CONVERT IT TO STABLE-MEMBER
		VXV	UNIT		# UNIT(XPSM * YPSM)
		STADR
		STORE	24D

; ============================================================================
; AVEIT - INCREMENTAL AVERAGING OF STAR VECTORS
;
; Performs incremental averaging using the formula:
;     NEW_AVG = ((N-1)/N) * OLD_AVG + (1/N) * NEW_VEC
; This allows efficient averaging without storing all individual vectors.
; ============================================================================

AVEIT		SLOAD	PDVL		# N(NUMBER OF VECS) IN 0-1
			MKDEX
			24D		# LOAD CURRENT VECTOR
		VSR3	V/SC		# DIVIDE CURRENT VECTOR BY N
			0
		STODL	24D		# VEC/N
			0
		DSU	DDV		# COMPUTE (N-1)/N SCALING FACTOR
			DP1/8		# (N-1)/N
		VXSC	VAD		# SCALE PREVIOUS AVERAGE AND ADD NEW
			STARAD +6	# ADD VEC TO PREVIOUSLY AVERAGED VECTOR
			24D		# (N-1)/N AVESTVEC + VEC/N
		STORE	STARAD +6	# AVERAGE STAR VECTOR
		STORE	STARSAV2	# SAVE FOR ALIGNMENT ROUTINES
		EXIT
		CCS	MARKCNTR	# SEE IF ANOTHER MARK PAIR IN MKVAC
		TCF	AVESTAR -1	# THERE IS -- GO GET IT -- DECREMENT COUNTER
; ============================================================================
; ENDMARKS - TERMINATE AOT MARK SEQUENCE
;
; Called when all marks have been averaged. Schedules the MKRELEAS job to
; clean up the mark system and return control to the alignment program.
; The averaged star vector in STARAD+6 is ready for IMU alignment processing.
; ============================================================================

ENDMARKS	CAF	FIVE		# NO MORE MARKS -- TERMINATE AOTMARK
		INHINT
		TC	WAITLIST	# SCHEDULE CLEANUP JOB
		EBANK=	XYMARK
		2CADR	MKRELEAS

		TC	ENDMARK

; ============================================================================
; MKALARM - MARK ALARM HANDLER
;
; Issues alarm code 111 when marks are incomplete or missing. After crew
; acknowledges the alarm, returns to GETMKS to restart the mark sequence.
; ============================================================================

MKALARM		TC	ALARM		# NOT A PAIR TO PROCESS -- DO GETMKS
		OCT	111
		TCF	GETMKS

; Display verb/noun code constants for mark operations.
V01N71		VN	171		# V01N71: Display star code
V06N87*		VN	687		# V06N87: Display mark data

# Page 254
# MARKRUPT IS ENTERED FROM INTERRUPT LEAD-INS AND PROCESSES CHANNEL 16
# CAUSED BY X,Y MARK OR MARK REJECT OR BY THE RATE OF DESCENT SWITCH

; ============================================================================
; MARKRUPT - MARK BUTTON INTERRUPT HANDLER
;
; COMMENT-ONLY READERS: When the crew presses a mark button on the DSKY while
; sighting a star through the AOT, this interrupt routine captures the exact
; spacecraft attitude at that instant. The AOT has a reticle pattern with X
; and Y marks allowing precise star sightings. These sightings feed the IMU
; alignment process that keeps the guidance platform accurately oriented.
;
; CODE-ALONG READERS: This is a hardware interrupt triggered by Channel 16
; (NAVKEYIN). It handles three types of inputs: X mark button (BIT3), Y mark
; button (BIT4), and Mark Reject button (BIT5). The routine captures CDU
; angles (CDUX, CDUY, CDUZ) and mission time (TIME2) at interrupt entry,
; validates the mark is wanted, determines surface vs inflight operation via
; MARKTYPE, and stores data through SURFSTOR (surface) or VACSTOR (inflight).
; ============================================================================

MARKRUPT	TS	BANKRUPT	; SAVE ACCUMULATOR IN INTERRUPT REGISTER
		CA	CDUY		; STORE CDUS AND TIME NOW -- THEN SEE IF
		TS	ITEMP3		; WE NEED THEM - SAVE CDUY IMMEDIATELY
		CA	CDUZ		; READ CDUZ (OUTER GIMBAL ANGLE)
		TS	ITEMP4		; SAVE CDUZ FOR LATER STORAGE
		CA	CDUX		; READ CDUX (INNER GIMBAL ANGLE)
		TS	ITEMP5		; SAVE CDUX FOR LATER STORAGE
		EXTEND
		DCA	TIME2		; CAPTURE MISSION TIME (DOUBLE PRECISION)
		DXCH	ITEMP1		; STORE TIME IN ITEMP1/ITEMP2
		XCH	Q		; SAVE RETURN ADDRESS FROM INTERRUPT
		TS	QRUPT		; STORE IN QRUPT FOR LATER RESUME

; Validate this is a mark button interrupt and that marks are being accepted.
		CAF	OCT34		# SEE IF X OR Y MARK OR MKREJECT (BITS 2,3,4,5)
		EXTEND			; EXTENDED INSTRUCTION FOLLOWS
		RAND	NAVKEYIN	# READ CHANNEL 16 (MARK BUTTON INPUTS)
		CCS	A		; CHECK IF ANY MARK BITS SET
		TCF	+2		# ITS A LIVE ONE -- SEE IF ITS WANTED
		TCF	SOMEKEY		# ITS SOME OTHER KEY (DESCENT BITS)

		CAF	BIT12		# ARE WE ASKING FOR A MARK
		MASK	MARKSTAT	; BIT12 SET MEANS "DON'T WANT MARKS NOW"
		CCS	A		; CHECK BIT12 STATE
		TC	RESUME		# DON'T WANT MARK OR MKREJECT -- DO NOTHING

		CCS	MARKSTAT	# ARE MARKS BEING ACCEPTED (MARKSTAT>0)
		TCF	FINDKEY		# THEY ARE -- WHICH ONE IS IT
		TC	ALARM		# MARKS NOT BEING ACCEPTED -- DO ALARM 112
		OCT	112		; ALARM CODE: MARK NOT EXPECTED
		TC	RESUME		; RETURN FROM INTERRUPT

; ============================================================================
; FINDKEY - IDENTIFY WHICH MARK BUTTON WAS PRESSED
;
; COMMENT-ONLY READERS: The crew has pressed one of three buttons: Mark Reject
; to discard a bad sighting, Y Mark when the star aligns with the vertical
; reticle line, or X Mark when the star aligns with the horizontal reticle.
; The computer tests each button in sequence to identify which was pressed.
;
; CODE-ALONG READERS: Tests Channel 16 (NAVKEYIN) in priority order:
; BIT5 (reject) first, then BIT4 (Y mark), then BIT3 (X mark). If none of
; these match, checks for descent rate bits (OCT140 = bits 6,7,8). Issues
; alarm 113 if no recognizable channel 16 input is found.
; ============================================================================

FINDKEY		CAF	BIT5		# SEE IF MARK REJECT (BIT5 = CREW DISCARD)
		EXTEND			; EXTENDED INSTRUCTION FOLLOWS
		RAND	NAVKEYIN	; MASK CHANNEL 16 WITH BIT5
		CCS	A		; TEST IF BIT5 SET
		TCF	MKREJ		# IT'S A MARK REJECT - GO PROCESS

		CAF	BIT4		# SEE IF Y MARK (BIT4 = VERTICAL RETICLE)
		EXTEND			; EXTENDED INSTRUCTION FOLLOWS
		RAND	NAVKEYIN	; MASK CHANNEL 16 WITH BIT4
		CCS	A		; TEST IF BIT4 SET

		TCF	YMKRUPT		# IT'S A Y MARK - GO CAPTURE Y ANGLE

		CAF	BIT3		# SEE IF X MARK (BIT3 = HORIZONTAL RETICLE)
		EXTEND			; EXTENDED INSTRUCTION FOLLOWS
		RAND	NAVKEYIN	; MASK CHANNEL 16 WITH BIT3

# Page 255
		CCS	A		; TEST IF BIT3 SET
		TCF	XMKRUPT		# IT'S A X MARK - GO CAPTURE X ANGLE

; Not a mark button - check for other Channel 16 inputs.
SOMEKEY		CAF	OCT140		# NOT MARK OR MKREJECT -- SEE IF DESCENT BITS
		EXTEND			; EXTENDED INSTRUCTION (BITS 6,7,8 = DESCENT RATE)
		RAND	NAVKEYIN	; MASK CHANNEL 16 WITH OCT140
		EXTEND			; EXTENDED INSTRUCTION FOR BZF
		BZF	+3		# IF NO BITS SET, GO TO ALARM

		TC	POSTJUMP	# IF DESCENT BITS SET, GO TO DESCBITS HANDLER
		CADR	DESCBITS	; ADDRESS OF DESCENT RATE HANDLER

		TC	ALARM		# NO RECOGNIZABLE BITS IN CHANNEL 16
		OCT	113		; ALARM CODE: INVALID CHANNEL 16 INPUT

		TC	RESUME		; RETURN FROM INTERRUPT

; ============================================================================
; XMKRUPT / YMKRUPT - PROCESS X OR Y MARK BUTTON PRESS
;
; COMMENT-ONLY READERS: When the crew aligns a reticle line with a star and
; presses either X Mark (horizontal alignment) or Y Mark (vertical alignment),
; this code captures the spacecraft attitude at that instant. The computer
; stores the angle data and checks if this completes an X/Y mark pair. When
; both X and Y marks are captured for the same star, the complete two-
; dimensional sighting is ready for platform alignment calculations.
;
; CODE-ALONG READERS: These routines use clever fall-through logic. XMKRUPT
; sets RUPTREG1=0 (array index), loads BIT10 (X mark ID), and jumps to
; shared code. YMKRUPT sets RUPTREG1=1, loads BIT11 (Y mark ID). Both store
; the mark type in XYMARK. The code then checks if this is surface vs
; inflight marking, and whether a mark pair has been completed.
; ============================================================================

XMKRUPT		CAF	ZERO		; X MARK USES INDEX 0 (FIRST ARRAY SLOT)
		TS	RUPTREG1	# SET X MARK STORE INDEX TO ZERO
		CAF	BIT10		; BIT10 = X MARK TYPE IDENTIFIER
		TCF	+4		; SKIP YMKRUPT SETUP, GO TO COMMON CODE

YMKRUPT		CAF	ONE		; Y MARK USES INDEX 1 (SECOND ARRAY SLOT)
		TS	RUPTREG1	# SET Y MARK STORE INDEX TO ONE
		CAF	BIT11		; BIT11 = Y MARK TYPE IDENTIFIER
		TS	XYMARK		# SET MARK IDENTIFICATION (COMMON CONVERGENCE)

; Determine if this is a surface mark or inflight mark, then store CDU angles.
		TC	MARKTYPE	# SEE IF SURFACE MARK (ON LUNAR SURFACE)
		TCF	SURFSTOR	# SURFACE MARK -- JUST STORE CDUS

; For inflight marks: check if this completes an X/Y mark pair.
; BIT14 in MARKSTAT indicates one mark of the pair already captured.
		CAF	BIT14		# GOT A MARK -- SEE IF MARK PAIR MADE
		MASK	MARKSTAT	; TEST BIT14 (FIRST MARK ALREADY MADE)
		EXTEND			; EXTENDED INSTRUCTION FOR BZF
		BZF	VERIFYMK	# NOT A PAIR (BIT14=0), NORMAL FIRST MARK
		CS	MARKCNTR	# GOT A PAIR -- SEE IF ANOTHER CAN BE MADE
		AD	FOUR		# ALLOW UP TO 5 PAIRS (COUNTER 0-4)
		EXTEND			; EXTENDED INSTRUCTION FOR BZMF
		BZMF	5MKALARM	# HAVE FIVE MARK PAIRS -- DON'T ALLOW MORE
		INCR	MARKCNTR	# OK FOR ANOTHER PAIR, INCREMENT POINTER
		CS	PRIO23		# CLEAR BITS 10,11,14 FOR NEXT PAIR SETUP
		MASK	MARKSTAT	; PRIO23 = BITS 10,11,14 (REMOVE MARK IDs)
		TS	MARKSTAT	; READY FOR NEXT STAR PAIR

; Verify this mark type (X or Y) is desired based on what was previously captured.
VERIFYMK	CA	XYMARK		; GET CURRENT MARK TYPE (BIT10=X, BIT11=Y)
		MASK	MARKSTAT	; CHECK IF THIS TYPE ALREADY CAPTURED
		CCS	A		; IF BIT ALREADY SET, MARK NOT DESIRED
		TCF	+2		# THIS MARK NOT DESIRED (DUPLICATE)
		TCF	VACSTOR		# MARK DESIRED -- GO STORE CDU ANGLES
		TC	ALARM		; ALARM: WRONG MARK TYPE FOR CURRENT STATE
		OCT	114		; ALARM CODE 114: MARK NOT COMPATIBLE
		TC	RESUME		# RESUME -- DISPLAY UNCHANGED -- WAIT FOR ACTION

# Page 256
; ============================================================================
; 5MKALARM - TOO MANY MARK PAIRS ATTEMPTED
;
; COMMENT-ONLY READERS: The crew can capture up to 5 star sightings (5 X/Y
; mark pairs) for each alignment session. If they try to capture more, the
; computer issues alarm 107 and prevents the extra mark.
;
; CODE-ALONG READERS: MARKCNTR ranges 0-4 (five pairs). When attempting to
; exceed this limit, alarm 107 is issued. Surface marks display V06N79 to
; show mark counter, inflight marks simply resume with no display change.
; ============================================================================

5MKALARM	TC	ALARM		# ATTEMPTING TO MAKE MORE THAN 5 MK PAIRS
		OCT	107		; ALARM CODE 107: TOO MANY MARKS
		TC	MARKTYPE	# SEE IF SURFACE MARK
		TCF	DSPV6N79	# IT IS - DISPLAY MARK COUNT
		TC	RESUME		# INFLIGHT - DON'T CHANGE DISPLAY

# Page 257
; ============================================================================
; MKREJ - MARK REJECT BUTTON PROCESSING
;
; COMMENT-ONLY READERS: If the crew realizes a star sighting was inaccurate
; (perhaps the star was misidentified or the reticle alignment was poor),
; they press the Mark Reject button to discard the bad data. This allows
; them to re-mark the same star without aborting the entire alignment.
;
; CODE-ALONG READERS: Checks MARKSTAT bits 10,11 (PRIO3) to see if any marks
; exist to reject. If none, issues alarm 115. If marks exist, uses BIT13 to
; track rejection state. First reject clears the last mark captured (X or Y
; based on XYMARK). Second consecutive reject clears both X and Y marks
; (PRIO3), allowing complete star re-capture. Updates display via REMARK.
; ============================================================================

MKREJ		TC	MARKTYPE	# SEE IF SURFACE MARK
		TCF	SURFREJ		# SURFACE -- JUST CHECK MARK COUNTER

; Inflight mark rejection: verify marks exist, then clear appropriate bits.
		CAF	PRIO3		# INFLIGHT -- SEE IF MARKS MADE (BITS 10,11)
		MASK	MARKSTAT	; TEST FOR ANY CAPTURED X OR Y MARKS
		CCS	A		; IF ZERO, NO MARKS TO REJECT
		TCF	REJECT		# MARKS MADE -- PROCEED WITH REJECTION
REJALM		TC	ALARM		# NO MARK TO REJECT -- BAD PROCEDURE
		OCT	115		; ALARM CODE 115: NO MARK TO REJECT
		TC	RESUME		# DESIRED ACTION DISPLAYED -- AWAIT CREW

; Process mark rejection based on whether this is first or second reject.
REJECT		CS	PRIO30		# ZERO BIT14 (PAIR FLAG), SET BIT13 (REJECT FLAG)
		MASK	MARKSTAT	# PRIO30 = BITS 13,14 (CLEAR BIT14)
		AD	BIT13		; ADD BIT13 TO SHOW REJECTION ACTIVE
		XCH	MARKSTAT	; UPDATE MARKSTAT, GET OLD VALUE IN A
		MASK	BIT13		; CHECK IF BIT13 WAS ALREADY SET (PRIOR REJECT)
		CCS	A		; IF BIT13 WAS SET, THIS IS SECOND REJECT
		TCF	REJECT2		# ANOTHER REJECT - CLEAR BOTH X AND Y MARKS

; First reject: clear only the most recent mark (X or Y).
		CS	XYMARK		# MARK MADE SINCE REJECT -- REJECT MARK IN 1D
RENEWMK		MASK	MARKSTAT	; CLEAR THE BIT FOR THIS MARK TYPE (X OR Y)
		TS	MARKSTAT	; UPDATE MARKSTAT WITH REJECTED MARK CLEARED
		TCF	REMARK		# GO REQUEST NEW MARK ACTION FROM CREW

; Second consecutive reject: clear both X and Y marks, restart star capture.
REJECT2		CS	PRIO3		# ON SECOND REJECT -- CLEAR BITS 10,11 (BOTH MARKS)
		TCF	RENEWMK		; MERGE WITH COMMON RENEWAL CODE

SURFREJ		CCS	MARKCNTR	# IF MARK DECREMENT COUNTER
		TCF	+2
		TCF	REJALM		# NO MARKS TO REJECT -- ALARM
		TS	MARKCNTR
		TC	RESUME

# Page 258
# MARKTYPE TESTS TO SEE IF LEM ON LUNAR SURFACE.  IF IT IS RETURN TO LOC+1

MARKTYPE	CS	FLAGWRD8	# SURFFLAG ******** TEMPORARY ******
		MASK	BIT8
		CCS	A		; CHECK MARK TYPE
		INCR	Q		# IF SURFACE MARK RETURN TO LOC +1
		TC	Q		# IF INFLIGHT MARK RETURN TO LOC +2

; ============================================================================
; SURFSTOR - STORE SURFACE MARK DATA
;
; COMMENT-ONLY READERS: For lunar surface alignments, the crew uses the AOT
; to sight stars while on the lunar surface. These marks require different
; handling than inflight marks. The computer stores the mark type (surface)
; and prepares to store the gimbal angles in the VAC area.
;
; CODE-ALONG READERS: Sets RUPTREG1 to zero (surface mark index). Sets
; bits 10,11 in MARKSTAT to indicate surface mark type for later processing
; by MARKCHEX. Falls through to VACSTOR to save gimbal angles.
; ============================================================================

SURFSTOR	CAF	ZERO		# FOR SURFACE MARK ZERO MARK KIND INDEX
		TS	RUPTREG1	; RUPTREG1=0 FOR SURFACE MARKS

		CS	MARKSTAT	# SET BITS10,11 TO SHOW SURFACE MARK
		MASK	PRIO3		# FOR MARKCHEX (BITS 10,11 = OCT1400)
		ADS	MARKSTAT	; STORE SURFACE MARK TYPE

; Store mark data in VAC area for later processing.
VACSTOR		CAF	LOW9		; LOW 9 BITS MASK (VAC ADDRESS)
		MASK	MARKSTAT	# STORE MARK VAC ADR IN RUPTREG2
		TS	RUPTREG2	; RUPTREG2 = VAC AREA BASE ADDRESS

		EXTEND			; EXTENDED INSTRUCTION FOLLOWS
		DCA	ITEMP1		# PICK UP MARKTIME (DOUBLE PRECISION)
		DXCH	TSIGHT		# STORE LAST MARK TIME FOR REFERENCE

		CA	MARKCNTR	# 6 X MARKCNTR FOR STORE INDEX
		EXTEND			; EXTENDED INSTRUCTION (MULTIPLY)
		MP	SIX		; EACH MARK USES 6 WORDS IN VAC AREA
		XCH	L		# GET INDEX FROM LOW ORDER PART
		AD	RUPTREG2	# SET CDU STORE INDEX TO MARKVAC BASE
		ADS	RUPTREG1	# INCREMENT VAC PICKUP BY MARK FOR FLIGHT
		TS	MKDEX		# STORE HERE IN CASE OF SURFACE MARK

; Store the three gimbal angles (CDU readings) into VAC area.
; For inflight marks: RUPTREG1 points to proper mark slot (0,6,12,18,24)
; For surface marks: RUPTREG1=0, stores at base of VAC area.
		CA	ITEMP3		; GET CDUY (Y AXIS GIMBAL ANGLE)
		INDEX	RUPTREG1	; INDEXED STORE (VAC BASE + OFFSET)
		TS	0		# STORE CDUY AT VAC+0
		CA	ITEMP4		; GET CDUZ (Z AXIS GIMBAL ANGLE)
		INDEX	RUPTREG1	; INDEXED STORE
		TS	2		# STORE CDUZ AT VAC+2
		CA	ITEMP5		; GET CDUX (X AXIS GIMBAL ANGLE)
		INDEX	RUPTREG1	; INDEXED STORE
		TS	4		# STORE CDUX AT VAC+4

		TC	MARKTYPE	# IF SURFACE MARK -- JUST DO SURFJOB
		TCF	SURFJOB		; SURFACE: GO DISPLAY V06N79

; Inflight mark: Update MARKSTAT with mark ID and check for mark pair completion.
; For star sightings, crew makes both X and Y marks on same star. When both
; marks are made (mark pair complete), set BIT14 to trigger star processing.
		CAF	BIT13		# CLEAR BIT13 TO SHOW MARK MADE
		AD	XYMARK		# SET MARK ID IN MARKSTAT (BIT10 OR BIT11)
		COM			; COMPLEMENT TO CLEAR BIT13, SET MARK ID
		MASK	MARKSTAT	; MASK WITH CURRENT MARKSTAT
		AD	XYMARK		; ADD BACK MARK ID
		TS	MARKSTAT	; UPDATED MARKSTAT WITH MARK RECORDED
		MASK	PRIO3		# SEE IF X, Y MARK MADE (BITS 10,11)
		TS	L		; SAVE MARK STATUS IN L

# Page 259
		CA	PRIO3		; GET BITS 10,11 MASK (OCT1400)
		EXTEND			; EXTENDED INSTRUCTION (EXCLUSIVE OR)
		RXOR	LCHAN		; XOR WITH L: ZERO IF BOTH BITS SET
		CCS	A		; CHECK RESULT
		TCF	REMARK		# NOT PAIR YET, DISPLAY MARK ACTION
		CS	MARKSTAT	# MARK PAIR COMPLETE -- SET BIT14
		MASK	BIT14		; GET BIT14 (MARK PAIR COMPLETE FLAG)
		ADS	MARKSTAT	; SET BIT14 IN MARKSTAT
		TCF	REMARK		# GO DISPLAY V54 (REQUEST NEXT ACTION)

# Page 260
; ============================================================================
; REMARK - DISPLAY MARK ACTION TO CREW
;
; COMMENT-ONLY READERS: After processing each mark button press, the computer
; displays information to the crew on the DSKY. For inflight marks, it shows
; which mark type is needed next (X mark, Y mark, or either). For surface
; marks, it displays the gimbal angles. This routine prepares the display
; verb code and creates a job to update the DSKY.
;
; CODE-ALONG READERS: Extracts mark ID bits (bits 10,11) from MARKSTAT,
; shifts them right by multiplying with BIT6 (divides by 64) to create
; index 0-3 for MKVB table. Creates NOVAC job at PRIO15 to execute CHANGEVB
; which will post the appropriate display verb. Returns via RESUME for marks
; or falls through to SURFJOB for surface processing.
; ============================================================================

REMARK		CAF	PRIO3		# BITS 10 AND 11 (OCT1400 MARK IDS)
		MASK	MARKSTAT	; EXTRACT MARK ID FROM MARKSTAT
		EXTEND			; EXTENDED INSTRUCTION (MULTIPLY)
		MP	BIT6		# SHIFT MARK IDS TO BE 0 TO 3 FOR INDEX
		TS	MKDEX		# STORE VERB INDEX (0=NEITHER, 1=X, 2=Y, 3=BOTH)
SURFJOB		CAF	PRIO15		; PRIORITY 15 FOR DISPLAY UPDATE
		TC	NOVAC		# ENTER JOB TO CHANGE DISPLAY TO
		EBANK=	XYMARK		# REQUEST NEXT ACTION (SET EBANK)
		2CADR	CHANGEVB	; JOB ADDRESS: CHANGEVB ROUTINE

		TC	RESUME		; RETURN FROM INTERRUPT

; ============================================================================
; CHANGEVB - SELECT AND DISPLAY APPROPRIATE MARK REQUEST VERB
;
; COMMENT-ONLY READERS: After each mark, the computer tells the crew what to
; do next. For surface marks (while on the lunar surface), it displays the
; AOT gimbal angles. For inflight marks (during navigation), it tells the
; crew which mark is needed next: X mark only, Y mark only, or either one.
;
; CODE-ALONG READERS: Calls MARKTYPE to test MARKSTAT bit 9 (surface vs
; inflight). Surface marks branch to DSPV6N79 (V06N79). Inflight marks use
; MKDEX (0-3) to index into MKVB table, selecting V54N71 (either mark),
; V53N71 (Y only), or V52N71 (X only). Posts selected verb via PASTIT.
; ============================================================================

CHANGEVB	TC	MARKTYPE	; TEST IF SURFACE OR INFLIGHT MARKING
		TCF	DSPV6N79	# SURFACE -- DISPLAY V 06 N 79 (GIMBAL ANGLES)
		INDEX	MKDEX		# INFLIGHT -- PICK UP MARK VB INDEX (0-3)
		CAF	MKVB54		; INDEXED TABLE ENTRY (BASE + MKDEX)
		TC	PASTIT		# PASTE UP NEXT MK VERB DISPLAY

; ============================================================================
; MARK VERB TABLE - INDEXED BY MKDEX (0-3)
;
; This table cannot be reordered. MKDEX values:
;   0 = Neither mark received yet (V54N71 - make X or Y mark)
;   1 = X mark received (V53N71 - make Y mark)
;   2 = Y mark received (V52N71 - make X mark)
;   3 = Both marks received (V54N71 - make X or Y mark on next star)
; ============================================================================

# THE FOUR MKVBS ARE INDEXED -- THEIR ORDER CANNOT BE CHANGED

MKVB54		VN	5471		# MAKE X OR Y MARK (INDEX 0)
MKVB53		VN	5371		# MAKE Y MARK (INDEX 1 - HAVE X, NEED Y)
MKVB52		VN	5271		# MAKE X MARK (INDEX 2 - HAVE Y, NEED X)
MKVB54*		VN	5471		# MAKE X OR Y MARK (INDEX 3 - NEW STAR)

; ============================================================================
; CONSTANTS AND DISPLAY CODES
; ============================================================================

DP1/8		2DEC	.125		; CONVERSION FACTOR FOR ANGLE SCALING

OCT34		OCT	34		; MARK BUTTON MASK (BITS 2,3,4,5)
V06N71		VN	671		; DISPLAY VERB 06 NOUN 71
V06N79*		VN	679		; DISPLAY VERB 06 NOUN 79 (GIMBAL ANGLES)

# Page 261
# ROUTINE TO REQUEST CURSOR AND SPIRAL MEASUREMENTS

; ============================================================================
; DSPV6N79 - SURFACE MARKING CURSOR AND SPIRAL DISPLAY
;
; COMMENT-ONLY READERS: While the Lunar Module sits on the lunar surface, this
; routine displays V06N79 on the DSKY, asking the crew to sight landmarks
; through the AOT and measure the cursor and spiral angles. These measurements
; help verify the LM's position on the Moon. The crew can take multiple
; measurements (up to 5) for accuracy, or end the marking session when done.
;
; CODE-ALONG READERS: Displays V06N79 (Verb 06 Noun 79) to request cursor and
; spiral angle inputs via GOMARKF. Three return paths: V34 terminates marking
; (KILLAOT), V33 ends marking normally (SURFEND sets BIT14), V32 recycles for
; another mark. ENTER key re-displays V06N79. Stores CURSOR/SPIRAL in VAC area
; indexed by MKDEX. Enforces 5-mark limit via MARKCNTR.
; ============================================================================

		COUNT*	$$/R59

DSPV6N79	CAF	V06N79*		# CURSOR -- SPIRAL DISPLAY (V06N79)
		TC	BANKCALL	; CROSS-BANK CALL TO DISPLAY ROUTINE
		CADR	GOMARKF		; GOMARKF HANDLES DISPLAY AND CREW INPUT

; Three possible crew responses branch to different handlers:
		TCF	KILLAOT		# V34 -- TERMINATE (DOES GOTOPOOH)
		TCF	SURFEND		# V33 -- PROCEED, END MARKING SESSION
		CAF	BIT6		# IF V32 (OCT40) IN MPAC DO RECYCLE
		MASK	MPAC		# OTHERWISE IT IS LOAD VB ENTER SO
		CCS	A		# TEST IF BIT6 SET (V32 RECYCLE)
		TCF	SURFAGAN	# VB32 -- RECYCLE FOR ANOTHER MARK
		TCF	DSPV6N79	# ENTER -- RE-DISPLAY V06N79

; SURFEND - Mark the end of surface marking session.
SURFEND		CS	BIT14		# SET BIT14 TO SHOW MARK END
		MASK	MARKSTAT	; CLEAR BIT14 IN MARKSTAT
		AD	BIT14		# ADD BIT14 BACK (SET IT)
		TS	MARKSTAT	; BIT14=1 MEANS "END OF MARKING"

; SURFAGAN - Store cursor and spiral measurements in VAC area.
SURFAGAN	CA	CURSOR		; LOAD CURSOR ANGLE (CREW INPUT)
		INDEX	MKDEX		# MKDEX HOLDS VAC AREA POINTER FOR SURF MARKING
		TS	1		# STORE CURSOR (SP 2'S COMPLEMENT FORMAT)
		CA	SPIRAL		; LOAD SPIRAL ANGLE (CREW INPUT)
		INDEX	MKDEX		; SAME VAC AREA, OFFSET +2
		TS	3		# STORE SPIRAL ANGLE

; Check if marking session should end or continue with another mark.
		CS	MARKSTAT	# IF BIT 14 SET -- END MARKING
		MASK	BIT14		; EXTRACT BIT14
		EXTEND			; EXTENDED INSTRUCTION (BZF)
		BZF	MARKCHEX	# BIT14 SET: GO TO MARKCHEX TO COMPLETE
		
; This is a recycle (V32) -- check if 5 marks already taken.
		CA	MARKCNTR	# LOAD MARK COUNTER (0-4)
		AD	ONE		; INCREMENT FOR COMPARISON
		COM			; ONE'S COMPLEMENT
		AD	FIVE		; COMPARE TO 5
		EXTEND			; EXTENDED INSTRUCTION (BZMF)
		BZMF	5MKALARM	# CAN'T RECYCLE -- TOO MANY MARKS -- DO ALARM
		INCR	MARKCNTR	# OK FOR RECYCLE -- INCREMENT MARK COUNTER
		TCF	GETMKS +3	# GO DISPLAY MARK VB FOR NEXT SIGHTING

