# Copyright:	Public domain.
# Filename:	R31.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 505-510
# Contact:      Onno Hommes <ohommes@cmu.edu>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-11 OH	Batch 2 Assignment Comanche Transcription
#		2009-05-20 RSB	Corrected INSTALL -> INTSTALL
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#       Assemble revision 055 of AGC program Comanche by NASA
#       2021113-051.  April 1, 1969.
#
#       This AGC program shall also be referred to as Colossus 2A
#
#       Prepared by
#                       Massachusetts Institute of Technology
#                       75 Cambridge Parkway
#                       Cambridge, Massachusetts
#
#       under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: R31.agc
; MODULE: COMEKISS Subsystem (Orbital Navigation)
; MISSION PHASE: all-phases
;
; TL;DR: Display orbital parameters routine presenting range, range rate, and
;        theta angle between Command Module and Lunar Module (or between CSM 
;        and target). Computes relative geometry from state vector integration
;        and formats for DSKY display, enabling crew verification of rendezvous
;        navigation accuracy during proximity operations.
;
; COMMENT-ONLY READERS: This program showed the crew how far apart the two
;        spacecraft were and how fast they were closing together during 
;        rendezvous. Essential for docking after lunar orbit rendezvous.
; CODE-ALONG READERS: Study state vector integration algorithms, relative
;        position/velocity computation, and display formatting for crew
;        situational awareness during rendezvous phases.
; ============================================================================

# Page 505
		BANK	34
		SETLOC	R31
		BANK

		COUNT*	$$/R31

; R31 DISPLAY ORBITAL PARAMETERS ROUTINE
; Entry point for R31 rendezvous navigation display program. During Apollo 11,
; this routine enabled Michael Collins in the Command Module to monitor range
; and closing rate to Eagle during the critical rendezvous after lunar ascent.
; The program continuously updates display showing relative position between
; the two spacecraft.

R31CALL		CAF	PRIO3		; Schedule job at priority 3
		TC	FINDVAC		; Find vacant core set for new job
		EBANK=	SUBEXIT		; Set erasable bank context
		2CADR	V83CALL		; Transfer to V83CALL state vector routine

; DISPLAY DELAY ROUTINE
; Waits for DSKY display system to be ready before showing rendezvous data.
; Ensures crew-requested displays (extended verbs) have completed before
; presenting computed orbital parameters. Polls every second until display
; system indicates availability.

DSPDELAY	CAF	1SEC		; Load 1 second delay constant
		TC	BANKCALL	; Cross-bank call to delay routine
		CADR	DELAYJOB	; Delay this job for 1 second
		CA	EXTVBACT	; Load extended verb activity word
		MASK	BIT12		; Check if display is available
		EXTEND			; Extend next instruction
		BZF	DSPDELAY	; Branch zero: display busy, wait again

; DISPLAY NOUN SELECTION AND FORMATTING
; Selects appropriate DSKY noun for rendezvous parameter display based on
; R31FLAG state. Noun 54 displays range, range rate, and theta angle for R31
; (rendezvous navigation). Displays update continuously enabling crew to
; monitor closure geometry during rendezvous phases.

DISPN5X		CA	FLAGWRD9	# TEST R31FLAG (IN SUNDANCE R31FLAG WILL
		MASK	BIT4		#     ALWAYS BE SET AS R34 DOES NOT EXIST)
		EXTEND			; Extend next instruction
		BZF	+3		; Branch if R31FLAG not set (use N53)
		CAF	V16N54		# R31	USE NOUN 54 (range/rate/theta)
		TC	+2		; Skip next instruction
		CAF	V16N53		# R34	USE NOUN 53 (alternate display)
		TC	BANKCALL	; Cross-bank call to display routine
		CADR	GOMARKF		; GOMARKF: Extended verb mark display
		TC	B5OFF		; Clear bit 5 (first branch return)
		TC	B5OFF		; Clear bit 5 (second branch return)
		TCF	DISPN5X		; Loop: continuous display update

; STATE VECTOR INITIALIZATION LOGIC
; V83: Entry when state vectors already integrated (subsequent display cycles)
; V83CALL: Initial entry requiring state vector extrapolation from current time
; to desired computation time. Branches to appropriate integration path based
; on whether this is first computation or continuous update.

V83		TC	INTPRET		; Transfer to interpreter mode
		GOTO			; Branch to integrated state handler
			HAVEBASE	# INTEG STATE VECTORS (already computed)
V83CALL		TC	INTPRET		; Transfer to interpreter mode
		GOTO			; Branch to extrapolation routine
			STATEXTP	# EXTRAPOLATE STATE VECTORS (first pass)
; ============================================================================
; COMPUTE AND DISPLAY RELATIVE ORBITAL PARAMETERS
;
; Calculates three critical rendezvous parameters displayed to crew:
; 1. RANGE: Distance between vehicles (meters, scaled B-29)
; 2. RRATE: Range rate - closing velocity (meters/centisecond, scaled B-7)
; 3. Theta angle: Line-of-sight angle for relative geometry
;
; During Apollo 11 rendezvous, these parameters enabled Michael Collins to
; verify that Eagle was on correct intercept trajectory after lunar ascent.
; Range decreasing and proper theta angle confirmed successful rendezvous.
; ============================================================================

COMPDISP	VLOAD	VSU		; Load RATT (this vehicle position)
			RATT		; Subtract RONE (other vehicle position)
			RONE
		PUSH	ABVAL		# RATT-RONE TO 0D	PD= 6
		STORE	RANGE		# Store magnitude as range (meters B-29)
		NORM	VLOAD		; Normalize for unit vector computation
			X1		# RATT-RONE		PD= 0
		VSR1			; Shift right 1 for normalization
		VSL*	UNIT		; Scale and compute unit line-of-sight
			0,1		; Variable shift based on X1
		PDVL	VSU		# UNIT(LOS) TO 0D	PD= 6
# Page 506
			VATT		; Load VATT (this vehicle velocity)
			VONE		; Subtract VONE (other vehicle velocity)
		DOT			# Dot product: (VATT-VONE).UNIT(LOS)
		SL1			# Scale left 1 for proper units
		STCALL	RRATE		# Store as range rate (m/cs B-7), call CDU
			CDUTRIG		# Initialize CDU angles for *NBSM* routine
; Compute line-of-sight (LOS) vectors and angular parameters for display.
; R34LOS subroutine transforms relative position into navigation base
; coordinates for theta/phi angle computation.

		CALL
			R34LOS		# NOTE.  PDL MUST = 0 (stack empty)
			
; ANGULAR PARAMETER COMPUTATION
; Computes theta angle (R31) or phi angle (R34) between vehicle position
; vector and spacecraft axis. Theta provides crew with orientation information
; for visual acquisition and docking alignment during final rendezvous phase.

R34ANG		VLOAD	UNIT		; Load RONE, compute unit position vector
			RONE
		PDVL			# UR TO 0D		PD= 6
			THISAXIS	# UNITX FOR CM, UNITZ FOR LM axis
		BON	VLOAD		# Check R31FLAG: ON=R31 theta, OFF=R34 phi
			R31FLAG		; Branch on R31FLAG state
			+2		# R31-THETA computation path
			12D		; R34-PHI alternate path
; Transform spacecraft axis to navigation base coordinates using *NBSM*
; (Navigation Base to Stable Member matrix routine). Compute theta angle
; using vector cross products and dot products to determine orientation.

		CALL
			*NBSM*		; Navigation base coordinate transformation
		VXM	PUSH		# UXORZ TO 6D		PD=12D
			REFSMMAT	; Transform by reference stable member matrix
		VPROJ	VSL2		; Project vector, scale left 2
			0D		; Source vector from 0D pushdown
		BVSU	UNIT		; Subtract from base, compute unit vector
			6D		; Base vector at 6D
		PDVL	VXV		# UP/2 TO 12D		PD=18D
			RONE		; Load position vector RONE
			VONE		; Cross with velocity vector VONE
		UNIT	VXV		; Unit normal, cross with RONE
			RONE		; Position vector
		DOT	PDVL		# SIGN TO 12D, UP/2 TO MPAC	PD=18D
			12D		; Dot product for sign determination
		VSL1	DOT		# UP.UXORZ - scale and dot product
			6D		; With vector at 6D
		SIGN	SL1		; Apply sign, scale left 1
			12D		; Sign from 12D
		ACOS			; Compute arc cosine for theta angle
		STOVL	RTHETA		; Store theta angle, load RONE
			RONE
		DOT	BPL		; Dot product, branch if positive
			6D		; Check sign with vector at 6D
			+5		; Skip adjustment if positive
		DLOAD	BDSU		# IF UXORZ.R NEG, RTHETA = 1 - RTHETA
			RTHETA		; Load computed theta
			DPPOSMAX	; Subtract from 1 revolution (360 degrees)
		STORE	RTHETA		# RTHETA BETWEEN 0 AND 1 REV (normalized)
; ============================================================================
; SECTION: Display Loop Control
;
; Check if crew has responded to display (terminated the extended verb).
; If the extended verb is still active (bit 5 clear), continue the display
; loop. If crew has answered (bit 5 set), terminate. Otherwise, toggle the
; display update flag and restart the computation cycle.
; ============================================================================

		EXIT
		CAF	BIT5		# HAVE WE BEEN ANSWERED
		MASK	EXTVBACT	; Check extended verb activity word
		EXTEND
		BZF	ENDEXT		# YES, DIE - crew terminated display
# Page 507
		CS	EXTVBACT	; No answer yet, toggle display flag
		MASK	BIT12		; Isolate bit 12 (display update flag)
		ADS	EXTVBACT	; Add to EXTVBACT, toggling the flag

		TCF	V83		; Restart computation cycle
V16N54		VN	1654		; Verb 16 Noun 54 code (R31 display format)
V16N53		VN	1653		; Verb 16 Noun 53 code (R34 display format)

# Page 508
# ============================================================================
# SECTION: STATEXTP - Initial State Vector Extrapolation
#
# ORIGINAL NASA COMMENTS (PRESERVED):
# STATEXTP DOES AN INITIAL PRECISION EXTRAPOLATION OF THE
# LEM STATE VECTOR TO PRESENT TIME OR TO PIPTIME IF AV G
# IS ON AND SAVES AS BASE VECTOR. IF AV G IS ON RN + VN
# ARE USED AS THE CM STATE VECTOR AND THE INITIAL R RDOT
# RTHETA ARE COMPUTED WITH NO FURTHER INTEGRATION. IF AV
# G IS OFF A PRECISION EXTRAPOLATION IS MADE OF THE CM
# STATE VECTOR TO PRESENT TIME AND.....
#
#   THE CM + LM STATE VECTORS ARE INTEGRATED TO PRES TIME
#   USING PRECISION OR CONIC AS SURFFLAG IS SET OR CLEAR.
#
#   IF AV G IS ON THEN
#     SUBSEQUENT PASSES WILL PROVIDE
#     USE OF RN + VN AS CM STATE VECTOR AND THE LM STATE
#     VECTOR WILL BE PRECISION INTEGRATED USING LEMPREC
#
#   IF SURFFLAG IS SET.
#     CM STATE VECTOR RONE VONE + LM STATE VECTOR RATT
#     VATT ARE USED IN COMPUTING R RDOT RTHETA.
#
# COMMENT-ONLY READERS: This is the initialization phase. The computer must
# first figure out where both spacecraft are right now by extrapolating their
# positions from the last known location. The "AV G" (Average G) flag indicates
# whether the spacecraft is actively maneuvering or coasting.
#
# CODE-ALONG READERS: STATEXTP performs precision orbital integration to
# establish baseline state vectors (BASEOTP/BASEOTV for LM, BASETHP/BASETHV
# for CM) at a common reference time (BASETIME). Integration method depends
# on V37FLAG (Average G on/off) and SURFFLAG (LM on surface/in orbit).
# ============================================================================
#

STATEXTP	RTB	BOF		# INITIAL INTEGRATION
			LOADTIME	; Load current mission time
			V37FLAG		; Test Average G flag (crew maneuvering?)
			+3		# AV G OFF, USE PRES TIME
		CALL			; AV G ON (maneuvering)
			GETRVN		#      ON,  USE RN VN PIPTIME (IMU data)
		STORE	BASETIME	# PRES TIME OR PIPTIME - reference time
		STCALL	TDEC1		; Set integration target time
			LEMPREC		; Perform precision LM state vector integration
		VLOAD			# BASE VECTOR, LM
			RATT1		; Load integrated LM position from RATT1
		STOVL	BASEOTP		#   POS. - store as LM baseline position
			VATT1		; Load integrated LM velocity from VATT1
		STORE	BASEOTV		#   VEL. - store as LM baseline velocity
		BON	DLOAD		; Branch if Average G on
			V37FLAG
			COMPDISP	# COMPUTE R RDOT RTHETA FROM
					# RONE(RN) VONE(VN) RATT+VATT(LEMPREC)
					; If AV G on, use RN/VN directly, skip CM integration
			TAT		; AV G off: load current time
		STCALL	TDEC1		; Set CM integration target time
			CSMPREC		; Perform precision CM state vector integration
		VLOAD			# BASE VECTOR, CM
			RATT1		; Load integrated CM position from RATT1
		STOVL	BASETHP		#  POS. - store as CM baseline position
			BASETHV		; Load integrated CM velocity from VATT1
		STORE	BASETHV		#  VEL. - store as CM baseline velocity
; ============================================================================
; SECTION: HAVEBASE - Subsequent State Vector Updates
;
; This section handles continuous updates after the initial baseline state
; vectors have been established. On each display cycle, re-integrate both
; CM and LM state vectors to current time, then compute updated relative
; geometry (range, range rate, theta angle).
;
; COMMENT-ONLY READERS: After the first calculation, the computer continuously
; updates the display by recalculating where both spacecraft are. This happens
; several times per second to keep the crew informed of changing conditions
; during rendezvous operations.
;
; CODE-ALONG READERS: HAVEBASE re-integrates stored baseline vectors forward
; to current time. Integration method switches between precision (LEMPREC/
; CSMPREC) and conic depending on SURFFLAG. MOONFLAG determines whether to
; use lunar or Earth gravity model. INTYPFLG selects integration algorithm.
; ============================================================================

HAVEBASE	BON	RTB		# SUBSEQUENT INTEGRATIONS
			V37FLAG		; Check if Average G is on
			GETRVN5		; AV G on: use RN/VN helper routine
			LOADTIME	; AV G off: load current time
		STCALL	TDEC1		# AV G OFF. SET INTEG. OF CM
			INTSTALL	; Initialize integration parameters
		VLOAD	CLEAR		; Load CM baseline position
			BASETHP		; From BASETHP (This Position)
# Page 509
			MOONFLAG	; Clear Moon gravity flag initially
		STOVL	RCV		; Store as current integration position
			BASETHV		; Load CM baseline velocity
		STODL	VCV		; Store as current integration velocity
			BASETIME	; Load baseline reference time
		BOF	SET		# GET APPROPRIATE MOONFLAG SETTING
			MOONTHIS	; Check if "this" vehicle is near Moon
			+2		; Skip if not near Moon
			MOONFLAG	; Set Moon gravity flag if near Moon
		CLEAR			; Clear integration type flag
			INTYPFLG	; (will be set based on SURFFLAG)
		BON	SET		; Check surface flag (LM landed?)
			SURFFLAG
			+2		# PREC. IF LM DOWN (precision integration)
			INTYPFLG	# CONIC IF LM NOT DOWN (faster conic)
		STCALL	TET		; Set integration end time to current
			INTEGRVS	# INTEGRATION --- AT LAST---
		VLOAD			; Load integrated CM state vectors
			RATT		; Position from integration result
		STOVL	RONE		; Store as RONE (CM position)
			VATT		; Velocity from integration result
		STODL	VONE		# GET SET FOR CONIC EXTRAP.,OTHER
			TAT		; Load time of integration
		BON	CALL		; Check surface flag for LM integration
			SURFFLAG
			GETRVN6		# LEMPREC IF LM DOWN (precision)
			INTSTALL	# ..CONIC IF NOT DOWN (faster)
		SET			; Set integration type flag for LM
			INTYPFLG
OTHINT		STORE	TDEC1		# ENTERED IF AV G ON TO INTEG LM
		VLOAD	CLEAR		; Load LM baseline position
			BASEOTP		; From BASEOTP (Other Position)
			MOONFLAG	; Clear Moon flag initially
		STOVL	RCV		; Store as current integration position
			BASEOTV		; Load LM baseline velocity
		STODL	VCV		; Store as current integration velocity
			BASETIME	; Load baseline reference time
		BOF	SET		; Get Moon flag setting for LM
			MOONTHIS	; Check if LM is near Moon
			+2		; Skip if not
			MOONFLAG	; Set Moon gravity flag if near Moon
		STCALL	TET		; Set integration end time
			INTEGRVS	; Perform integration of LM state
		GOTO
			COMPDISP	# COMPUTE R RDOT RTHETA (relative geometry)
; ============================================================================
; Helper Routine: GETRVN5 - Get RN/VN for Average G On, Subsequent Integration
;
; When Average G is on during subsequent updates (HAVEBASE), retrieve the
; current spacecraft position and velocity from the IMU (RN/VN via GETRVN),
; then determine integration approach based on whether LM is on surface.
;
; CODE-ALONG READERS: GETRVN loads current navigation state (RN/VN) and
; PIPTIME. SURFFLAG check determines whether to use LEMPREC (precision
; integration for landed LM) or conic integration. Either path leads to
; OTHINT to integrate the "other" vehicle state.
; ============================================================================

GETRVN5		CALL			# AV G ON
			GETRVN		; Get current RN/VN and PIPTIME
		BON	CALL		; Branch on SURFFLAG (LM on surface?)
			SURFFLAG
			GETRVN6		# LM DOWN, LMPREC (precision integration)
# Page 510
			INTSTALL	; LM not down, conic integration
		CLEAR	GOTO		; Clear integration type flag
			INTYPFLG	; Use conic integration for LM
			OTHINT		; Go integrate "other" vehicle (LM)
; ============================================================================
; Helper Routine: GETRVN6 - Precision Integration Setup for Landed LM
;
; When the LM is on the lunar surface (SURFFLAG set), use high-precision
; integration (LEMPREC) for the CM state vector. After integration, proceed
; directly to compute relative display parameters.
;
; CODE-ALONG READERS: LEMPREC accounts for lunar gravity perturbations with
; higher accuracy than conic integration. Used when LM is stationary on
; surface and only CM state needs continuous propagation.
; ============================================================================

GETRVN6		STCALL	TDEC1		; Store time, call LEM precision integration
			LEMPREC		; High-precision CM integration (LM on surface)
		GOTO
			COMPDISP	# COMPUTE R RDOT RTHETA (compute relative geometry)
; ============================================================================
; Helper Routine: GETRVN - Retrieve Current Navigation State from IMU
;
; When Average G (IMU-based navigation) is active, load the current position
; (RN) and velocity (VN) vectors from the IMU, along with the measurement
; time (PIPTIME). These become the reference state vectors RONE and VONE.
;
; COMMENT-ONLY READERS: This routine reads the spacecraft's current position
; and velocity directly from the navigation platform (the gyroscope and
; accelerometer system that tracks motion).
;
; CODE-ALONG READERS: RN/VN are maintained by integration of PIPA pulses
; (accelerometer measurements). PIPTIME is the timestamp of the navigation
; state. STQ/GOTO pattern implements subroutine return via Q-register.
; ============================================================================

GETRVN		STQ			; Store return address in Q-register
			0D		; At pushdown location 0D
		VLOAD	GOTO		# AV G ON, RONE = RN VONE = VN
			RN		#  AND USE PIPTIME (current position)
			+1		; Continue to next instruction
		STCALL	RONE		; Store RN as RONE (reference position)
			+1		; Continue
		VLOAD	GOTO		; Load velocity vector
			VN		; Current velocity from IMU
			+1		; Continue
		STODL	VONE		; Store VN as VONE, load PIPTIME
			PIPTIME		; Navigation state timestamp
		GOTO			; Return via Q-register
			0D		; Return address stored at 0D
		SETLOC	R34
		BANK

; ============================================================================
; Subroutine: R34LOS - Set Up Navigation Base for R34 Angular Display
;
; R34LOS prepares the coordinate transformation needed for R34's PHI angle
; computation. It loads current IMU gimbal angles (CDUS, CDUT) into the
; pushdown list, then calls SXTNB to compute the navigation base matrix.
; This establishes the reference frame for displaying the line-of-sight
; angle between vehicles.
;
; COMMENT-ONLY READERS: This routine sets up the coordinate system needed
; to compute the angle you'd see if looking from one spacecraft toward the
; other through the optical telescope.
;
; CODE-ALONG READERS: Native AGC code loads CDU angles into indexed pushdown
; locations (9D, 11D), sets up loop counter in X1, then transfers to
; interpreter for SXTNB (sextant navigation base) computation. SXTNB returns
; transformation matrix stored at 12D, then continues to R34ANG for angle
; calculation.
; ============================================================================

R34LOS		EXIT			; Exit interpreter to native AGC code
		CA	CDUS		; Load CDU shaft angle from IMU
		INDEX	FIXLOC		; Index by fixed location pointer
		TS	9D		; Store at pushdown location 9D
		CA	CDUT		; Load CDU trunnion angle from IMU
		INDEX	FIXLOC		; Index by fixed location pointer
		TS	11D		; Store at pushdown location 11D
		CA	FIXLOC		; Load fixed location base address
		AD	SIX		; Add 6 (offset for loop counter setup)
		COM			; Complement (negate and subtract 1)
		INDEX	FIXLOC		; Index by fixed location
		TS	X1		; Store loop counter in X1 register
		TC	INTPRET		; Transfer control to interpreter
		CALL			; Call navigation base subroutine
			SXTNB		; Sextant navigation base transformation
		STCALL	12D		; Store result at 12D, continue
			R34ANG		; Compute R34 angle (PHI)
