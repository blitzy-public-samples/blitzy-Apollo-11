# Copyright:	Public domain.
# Filename:	AGS_INITIALIZATION.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	206-210
# Mod history:	2009-05-19 HG	Transcribed from page images.
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
; FILE: AGS_INITIALIZATION.agc
; MODULE: Abort Guidance System Initialization
; MISSION PHASE: lunar-orbit/descent/landing/ascent
;
; TL;DR: Initializes the backup Abort Guidance System (AGS/AEA) by
;        transferring current spacecraft state vectors from the primary AGC
;        to the AGS computer. Establishes common reference frame by zeroing
;        gimbal angle counters. Critical for abort readiness during lunar
;        descent and ascent when immediate abort capability must be maintained.
;
; COMMENT-ONLY READERS: This routine prepares the LM's backup guidance computer
;        for potential abort scenarios during lunar operations. Read to
;        understand dual-computer redundancy in Apollo spacecraft.
; CODE-ALONG READERS: Study AGC-to-AGS data transfer protocol, coordinate
;        transformation from AGC to AGS frame, and IMU synchronization logic.
; ============================================================================

# Page 206

# PROGRAM NAME:  AGS INITIALIZATION (R47)
#
# WRITTEN BY:  RHODE/KILROY/FOLLETT
#
# MOD NO.:	0
# DATE:		23 MARCH 1967
# MOD BY:	KILROY
#
# MOD NO.:	1
# DATE:		28 OCTOBER 1967
# MOD BY:	FOLLETT
#
# FUNCT. DESC.:	(1) TO PROVIDE THE AGS ABORT ELECTRONICS ASSEMBLY (AEA) WITH THE LEM AND CSM STATE VECTORS
#		(POSITION,VELOCITY,TIME) IN LEM IMU COORDINATES BY MEANS OF THE LGC DIGITAL DOWNLINK.
#
#		(2) TO ZERO THE ICDU, LGC, AND AEA GIMBAL ANGLE COUNTER SIMULTANEOUSLY IN ORDER TO ESTABLISH A
#		COMMON ZERO REFERENCE FOR THE MEASUREMENT OF GIMBAL (EULER) ANGLES WHICH DEFINE LEM ATTITUDE
#		(3) TO ESTABLISH THE GROUND ELAPSED TIME OF AEA CLOCK ZERO.  (IF AN AEA CLOCK ZERO IS
#		REQUESTED DURING THIS PROGRAM
#
# LOG SECTION:	AGS INITIALIZATION
#
# CALLING SEQ:	PROGRAM IS ENTERED WHEN ASTRONAUT KEYS V47E ON DSKY.
#		R47 MAY BE CALLED AT ANY TIME EXCEPT WHEN ANOTHER EXTENDED VERB IS IN PROGRESS
#
# SUBROUTINES
# CALLED:
#
# NORMAL EXIT:	ENDEXT
#
# ALARM/ABORT:	ALARM -- BAD REFSMMAT -- CODE:220
#		OPERATOR ERROR IF V47 SELECTED DURING ANOTHER EXTENDED VERB.
#
# ERASABLES
# USED:		SAMPTIME	(2)	TIME OF :ENTER: KEYSTROKE
#		AGSK		(2)	GROUND ELAPSED TIME OF THE AEA CLOCK :ZERO:
#		AGSBUFF		(14D)	CONTAINS AGS INITIALIZATION DATA (SEE :OUTPUT: BELOW)
#		AGSWORD		(1)	PREVIOUS DOWNLIST SAVED HERE

; ============================================================================
; TRANSITION: AGS Initialization Program Entry
;
; The Lunar Module carries two independent guidance computers for safety:
; the primary AGC and the backup AGS (Abort Guidance System). This program
; (invoked by crew entering V47E on the DSKY) synchronizes the AGS with
; current mission state, ensuring abort readiness during critical phases.
; ============================================================================

		EBANK=	AGSBUFF

		BANK	40
		SETLOC	R47
		BANK

		COUNT*	$$/R47

; AGSINIT: Entry point for AGS initialization routine (V47E)
; Before transferring state vectors to the AGS, verify that the reference
; coordinate system (REFSMMAT) is valid. An invalid REFSMMAT would corrupt
; the AGS with incorrect position and velocity data.

AGSINIT		CAF	REFSMBIT
		MASK	FLAGWRD3			# CHECK REFSMFLG.
		CCS	A
# Page 207
		TC	REDSPTEM			# REFSMMAT IS OK
		TC	ALARM				# REFSMMAT IS BAD
		OCT	220
		TC	ENDEXT

; NEWAGS: Establish new AGS clock zero time
; The AGS maintains its own mission elapsed time clock. This routine captures
; the crew's ENTER keystroke time and uses it to establish the AGS clock zero
; reference point for subsequent time calculations.

NEWAGS		EXTEND
		DCA	SAMPTIME			# TIME OF THE :ENTER: KEYSTROKE
		DXCH	AGSK				# BECOMES NEW AEA CLOCK :ZERO:

; REDSPTEM: Display current AGS clock zero time to crew
; The crew can verify the AGS time reference on the DSKY before proceeding
; with state vector transfer. This ensures both computers share common time.

REDSPTEM	EXTEND
		DCA	AGSK
		DXCH	DSPTEMX
AGSDISPK	CAF	V06N16
		TC	BANKCALL			# R1 = 00XXX. HRS., R2 = 000XX MIN.,
		CADR	GOMARKF				# R3 = 0XX.XX SEC.
		TC	ENDEXT				# TERMINATE RETURN
		TC	AGSVCALC			# PROCEED RETURN
		CS	BIT6				# IS ENTER VIA A V32
		AD	MPAC
		EXTEND
		BZF	NEWAGS				# YES, USE KEYSTROKE TIME FOR NEW AGSK

; Crew can enter AGS clock zero time manually via V25E or automatically
; via V32. This flexibility supports different mission procedures.
		EXTEND					# NO, NEW AGSK LOADED VIA V25
		DCA	DSPTEMX				# LOADED INTO DSPTEMX BY KEYING
		TC	REDSPTEM -1			# V25E FOLLOWED BY HRS.,MINS.,SECS.
							# DISPLAY THE NEW K

; ============================================================================
; TRANSITION: From Time Display to State Vector Calculation
;
; With AGS clock zero established, the computer now calculates current
; position and velocity for both the Lunar Module and Command Module.
; These state vectors must be extrapolated to present time and converted
; to AGS coordinate frame before transmission to the backup computer.
; ============================================================================

; AGSVCALC: Calculate and buffer state vectors for AGS transfer
; The AGS requires both LM and CSM state vectors to maintain abort capability.
; During descent/ascent, if the primary AGC fails, the AGS must know both
; spacecraft positions to compute rendezvous trajectories for abort scenarios.

AGSVCALC	TC	INTPRET
		SET
			NODOFLAG			# DON'T ALLOW V37
		SET	EXIT
			XDSPFLAG

		CAF	V06N16
		TC	BANKCALL
		CADR	EXDSPRET

; Extrapolate spacecraft state vectors to current time
; Both LM and CSM may have maneuvered since last navigation update. The AGC
; uses numerical integration to propagate orbital state to present moment.
		TC	INTPRET				# EXTRAPOLATE LEM AND CSM STATE VECTORS
		RTB					# TO THE PRESENT TIME
			LOADTIME			# LOAD MPAC WITH TIME2,TIME1
		STCALL	TDEC1				# CALCULATE LEM STATE VECTOR
			LEMPREC
		CALL					# CALL ROUTINE TO CONVERT TO SM COORDS AND
			SCALEVEC			# PROVIDE PROPER SCALING
		STODL	AGSBUFF				# (LEMPREC AND CSMPREC LEAVE TDEC1 IN TAT)
			TAT				# TAT = TIME TO WHICH RATT1 AND VATT1 ARE
		STCALL	TDEC1				# COMPUTED (CSEC SINCE CLOCK START B-28).
			CSMPREC				# CALCULATE CSM STATE VECTOR FOR SAME TIME
		CALL
			SCALEVEC
# Page 208
		STODL	AGSBUFF +6
			TAT
		DSU	DDV				# CALCULATE AND STORE THE TIME
			AGSK
			TSCALE
		STORE	AGSBUFF +12D
		EXIT

; Initiate AGS downlink transmission
; The buffered state vectors are transmitted via digital downlink to the
; AGS computer. This 20-second transmission updates the backup system with
; current mission state, maintaining abort readiness.

		CAF	LAGSLIST
		TS	DNLSTCOD

		CAF	20SEC				# DELAY FOR 20 SEC WHILE THE AGS
		TC	BANKCALL			# DOWNLIST IS TRANSMITTED
		CADR	DELAYJOB

; After 20-second downlink transmission completes, restore normal telemetry
; The AGS downlist temporarily interrupts regular telemetry to ground control.
; Once state vector transfer completes, resume previous downlist configuration.
		CA	AGSWORD
		TS	DNLSTCOD			# RETURN TO THE OLD DOWNLIST
		CAF	IMUSEBIT
		MASK	FLAGWRD0			# CHECK IMUSE FLAG.
		CCS	A
; ============================================================================
; TRANSITION: From State Vector Transfer to IMU Synchronization
;
; With state vectors transferred to AGS, the final initialization step
; synchronizes the IMU gimbal angle counters between AGC and AGS. Both
; computers must share common gimbal angle reference to maintain attitude
; knowledge during potential abort scenarios requiring AGS takeover.
; ============================================================================

; IMU zeroing procedure establishes common attitude reference
; The IMU (Inertial Measurement Unit) gimbal angles define spacecraft attitude.
; By simultaneously zeroing both AGC and AGS gimbal counters, both computers
; measure attitude relative to the same reference, critical for abort guidance.
		TC	AGSEND				# IMU IS BEING USED -- DO NOT ZERO
CKSTALL		CCS	IMUCADR				# CHECK FOR IMU USAGE WHICH AVOIDS THE
		TCF	+3				# IMUSE BIT:  I.E., IMU COMPENSATION.
		TCF	+6				# FREE.  GO AHEAD WITH THE IMU ZERO.
		TCF	+1
 +3		CAF	TEN				# WAIT .1 SEC AND TRY AGAIN.
 		TC	BANKCALL
		CADR	DELAYJOB
		TCF	CKSTALL

; Pulse IMU zero discrete for 320 milliseconds
; This hardware signal simultaneously resets AGC, LGC, and AEA gimbal counters
; to zero, establishing synchronized attitude reference across all systems.
 +6		TC	BANKCALL			# IMU IS NOT IN USE
 		CADR	IMUZERO				# SET IMU ZERO DISCRETE FOR 320 MSECS.
		TC	BANKCALL			# WAIT 3 SEC FOR COUNTERS TO INCREMENT
		CADR	IMUSTALL
		TC	AGSEND
AGSEND		TC	DOWNFLAG			# ALLOW V37
		ADRES	NODOFLAG

; Display completion to crew via DSKY
; V50N16 requests crew acknowledgment that AGS initialization completed
; successfully, confirming abort backup system is ready for mission operations.
		CAF	V50N16
		TC	BANKCALL
		CADR	GOMARK3
		TCF	ENDEXT
		TCF	ENDEXT
		TC	ENDEXT

; ============================================================================
; SCALEVEC: Transform and scale state vectors for AGS compatibility
;
; The AGS computer uses different coordinate frame and numerical representation
; than the AGC. This routine transforms AGC vectors from navigation base
; coordinates to stable member coordinates via REFSMMAT, scales for AGS
; precision, and converts from AGC's one's complement to AGS two's complement.
; ============================================================================

SCALEVEC	VLOAD	MXV
			VATT1
			REFSMMAT
		VXSC	VSL2
			VSCALE
# Page 209
; Rounding and complement conversion for AGS compatibility
; AGC uses one's complement arithmetic (negative zero exists: +0 and -0)
; AGS uses two's complement arithmetic (single zero representation)
; This conversion ensures numerical consistency across both computers.
		VAD	VAD				# THIS SECTION ROUNDS THE VECTOR, AND
			AGSRND1				# CORRECTS FOR THE FACT THAT THE AGS
			AGSRND2				# IS A 2 S COMPLEMENT MACHINE WHILE THE
		RTB					# LGC IS A 1 S COMPLEMENT MACHINE.
			VECSGNAG
		STOVL	VATT1
			RATT1
		MXV	VXSC
			REFSMMAT
			RSCALE
		VSL8	VAD				# AGAIN THIS SECTION ROUNDS.  TWO VECTORS
			AGSRND1				# ARE ADDED TO DEFEAT ALSIGNAG IN THE
		VAD	RTB				# CASE OF A HIGH-ORDER ZERO COUPLED WITH
			AGSRND2				# A LOW ORDER NEGATIVE PART.
			VECSGNAG
; Pack transformed vectors into MPAC stack for storage
; The index register manipulation ensures proper alignment in memory buffer
; for transmission to AGS. Vectors must be tightly packed for downlink.
		LXA,1
			VATT1
		SXA,1	LXA,1
			MPAC +1
			VATT1 +2
		SXA,1	LXA,1
			MPAC +4
			VATT1 +4
		SXA,1	RVQ
			MPAC +6

; ============================================================================
; CONSTANTS AND DISPLAY CODES
; ============================================================================

; LAGSLIST: AGS downlist code selector
; Switches telemetry stream to AGS initialization data format during transfer
LAGSLIST	=	ONE

; Display verb/noun codes for crew interface
; These DSKY codes guide crew through AGS initialization procedure
V01N14		VN	0114
V50N00A		VN	5000
V00N25		EQUALS	OCT31
V06N16		VN	0616			# Display time (hrs, min, sec)
V00N34		EQUALS	34DEC
V50N16		VN	5016			# Please perform AGS initialization

; Scaling factors for AGS unit conversion
; AGS uses feet/feet-per-second while AGC uses meters/meters-per-centisecond
TSCALE		2DEC	100 B-10			# CSEC TO SEC SCALE FACTOR
20SEC		DEC	2000				# Downlink transmission duration
RSCALE		2DEC	3.280839 B-3			# METERS TO FEET SCALE FACTOR
VSCALE		2DEC	3.280839 E2 B-9			# METERS/CS TO FEET/SEC SCALE FACTOR

; Rounding vectors for one's complement to two's complement conversion
; These constants ensure proper numerical representation for AGS computer
AGSRND1		2OCT	0000060000
		2OCT	0000060000
		2OCT	0000060000
AGSRND2		2OCT	0000037777
		2OCT	0000037777
# Page 210
		2OCT	0000037777

		SBANK=	LOWSUPER			# FOR SUBSEQUENT LOW 2CADRS.

