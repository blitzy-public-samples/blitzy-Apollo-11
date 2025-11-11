# Copyright:	Public domain.
# Filename:	TVCEXECUTIVE.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	945-950
# Mod history:	2009-05-12 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-20 RSB	Corrections:  CAE -> CAF in one place.
#		2009-05-21 RSB	In 1SHOTCHK, a CAF SEVEN was corrected to
#				CAF SIX.
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

; ============================================================================
; FILE: TVCEXECUTIVE.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth (SPS burns)
;
; TL;DR: TVC executive scheduler managing control loop timing during SPS main
;        engine burns. Coordinates 20-millisecond control cycle execution,
;        integrates guidance commands with autopilot responses, and manages
;        TVC/DAP task scheduling during critical Apollo 11 maneuvers (TLI, LOI, TEI).
;
; COMMENT-ONLY READERS: This program coordinated the precise timing of engine
;        steering calculations 50 times per second during rocket burns.
; CODE-ALONG READERS: Study TVC control loop scheduling, timing constraints,
;        guidance/autopilot integration, and task coordination architecture.
; ============================================================================

# Page 945
# PROGRAM NAME....	TVCEXECUTIVE, CONSISTING OF TVCEXEC, NEEDLEUP, VARGAINS
#			1SHOTCHK, REPCHEK, CG.CORR, COPYCYCLES, ETC.
# LOG SECTION...TVCEXECUTIVE			   SUBROUTINE...DAPCSM
# MODIFIED BY SCHLUNDT				   21 OCTOBER 1968
# FUNCTIONAL DESCRIPTION....
#
#      *A SELF-PERPETUATING WAITLIST TASK AT 1/2 SECOND INTERVALS WHICH:
#	PREPARES THE ROLL DAP WITH OGA (CDUX)
#	PREPARES THE ROLL FDAI NEEDLE (FLY-TO  OGA ERROR)
#	PREPARES THE ROLL PHASE PLANE  OGAERR  (FLY-FROM  OGA ERROR)
#	PREPARES THE TVC ROLLDAP TASK WAITLIST CALL (3 CS DELAY)
#	UPDATES THE NEEDLES DISPLAY
#	UPDATES VEHICLE MASS AND CALLS MASSPROP TO UPDATE INERTIA DATA
#	UPDATES PITCH, YAW, AND ROLL DAP GAINS FROM MASSPROP DATA
#	PERFORMS ONE-SHOT CORRECTION FOR TMC LOOP 0-3 SEC AFTER IGNITION
#	PERFORMS REPETITIVE UPDATES FOR TMC LOOP AFTER THE ONE-SHOT CORR.
#
# CALLING SEQUENCE....
#
#      *TVCEXEC CALLED AS A WAITLIST TASK, IN PARTICULAR BY TVCINIT4 AND BY
#	ITSELF, BOTH AT 1/2 SECOND INTERVALS
#
# NORMAL EXIT MODE.... TASKOVER
#
# ALARM OR ABORT EXIT MODES.... NONE
#
# SUBROUTINES CALLED....NEEDLER, S40.15, MASSPROP, TASKOVER, IBNKCALL
#
# OTHER INTERFACES....
#
#      *TVCRESTART PACKAGE FOR RESTARTS
#      *PITCHDAP, YAWDAP FOR VARIABLE GAINS AND ENGINE TRIM ANGLES
# ERASABLE INITIALIZATION REQUIRED....
#
#      *SEE TVCDAPON....TVCINIT4
#      *VARK AND 1/CONACC (S40.15 OF TVCINITIALIZE)
#      *PAD LOAD EREPFRAC
#      *BITS 15,14 OF FLAGWRD6 (T5 BITS)
#      *TVCEXPHS FOR RESTARTS
#      *ENGINE-ON BIT (11.13) FOR RESTARTS
#      *CDUX, OGAD
#
# OUTPUT....
#
#      *ROLL DAP OGANOW, FDAI NEEDLE (AK), AND PHASE PLANE OGAERR
#      *VARIABLE GAINS FOR PITCH/YAW AND ROLL TVC DAPS
#      *SINGLE-SHOT AND REPETITIVE CORRECTIONS TO ENGINE TRIM ANGLES
#	PACTOFF AND YACTOFF
#
# DEBRIS....	MUCH, BUT SHAREABLE WITH RCS/ENTRY, ALL IN EBANK6

; ============================================================================
; TVCEXECUTIVE OPERATIONAL CONTEXT
;
; During Apollo 11's critical SPS engine burns (translunar injection on July 16,
; lunar orbit insertion on July 19, and transearth injection on July 21), this
; executive scheduler coordinated all thrust vector control calculations. The
; routine ran every half second, updating engine gimbal commands to maintain
; precise spacecraft attitude during rocket firing.
;
; The TVC system steered the 20,500-pound-thrust SPS engine by gimbaling it
; in pitch and yaw axes. This program served as the master coordinator,
; calling the actual control law calculations (TVCDAPS.agc) and managing the
; timing of all TVC-related tasks through the AGC's WAITLIST scheduler.
; ============================================================================

# Page 946
		BANK	16
		SETLOC	DAPROLL
		BANK
		EBANK=	BZERO
		COUNT*	$$/TVCX

; ============================================================================
; TVCEXEC - TVC EXECUTIVE MAIN ENTRY POINT
;
; This waitlist task perpetuates itself every 0.5 seconds during SPS burns.
; It orchestrates roll DAP preparation, FDAI needle updates, variable gain
; computation, mass property tracking, and engine trim corrections.
;
; Called by: TVCINIT4 (initialization), self-perpetuation via WAITLIST
; Timing: Executes every 500 milliseconds during TVC operation
; Termination: When FLAGWRD6 bits 15,14 indicate transition to RCS DAP mode
; ============================================================================

TVCEXEC		CS	FLAGWRD6	# CHECK FOR TERMINATION (BITS 15,14 READ
		MASK	OCT60000	#      10 FROM TVCDAPON TO RCSDAPON)
		EXTEND
		BZMF	TVCEXFIN	# TERMINATE

		CAF	.5SEC		# W.L. CALL TO PERPETUATE TVCEXEC
		TC	WAITLIST
		EBANK=	BZERO
		2CADR	TVCEXEC

; ============================================================================
; ROLLPREP - ROLL AXIS DATA PREPARATION
;
; Prepares roll control data by updating roll angle ladder variables and
; computing FDAI needle error for crew display. The roll axis uses the
; Coupling Data Unit (CDU) mounted on the IMU to measure actual spacecraft
; roll angle, comparing it against commanded roll angle (OGAD) for error.
;
; CDUX = Current roll angle from IMU (in revolutions, scaled B+0)
; OGANOW/OGAPAST = Roll angle history for rate computation
; AK = FDAI needle error (fly-to error shown to crew on attitude indicator)
; OGAERR = Phase plane error (fly-from error used by roll autopilot)
; ============================================================================

ROLLPREP	CAE	CDUX		# UPDATE ROLL LADDERS (NO NEED TO RESTART-
		XCH	OGANOW		#      PROTECT, SINCE ROLL DAPS RE-START)
		XCH	OGAPAST

		CAE	OGAD		# PREPARE ROLL FDAI NEEDLE WITH FLY-TO
		EXTEND			#      ERROR (COMMAND - MEASURED)
		MSU	OGANOW
		TS	AK		# FLY-TO OGA ERROR, SC.AT B-1 REVS

		EXTEND			# PREPARE ROLL DAP PHASE PLANE  OGAERR
		MP	-BIT14
		TS	OGAERR		# PHASE-PLANE (FLY-FROM) OGAERROR,
					#      SC.AT B+0 REVS

		CAF	THREE		# SET UP ROLL DAP TASK (ALLOW SOME TIME)
		TC	WAITLIST
		EBANK=	BZERO
		2CADR	ROLLDAP

; ============================================================================
; NEEDLEUP - CREW DISPLAY UPDATE
;
; Calls the NEEDLER routine to update the FDAI (Flight Director Attitude
; Indicator) needles display visible to the crew. The needles show attitude
; errors in pitch, yaw, and roll, providing real-time visual feedback during
; SPS engine burns. Michael Collins relied on these needle displays during
; Apollo 11's critical maneuvers (TLI, LOI, TEI) to monitor spacecraft
; orientation while the guidance computer controlled the engine gimbals.
;
; The NEEDLER subroutine handles restart protection internally, ensuring
; display updates continue even if a computer restart occurs mid-burn.
; ============================================================================

NEEDLEUP	TC	IBNKCALL	# DO A NEEDLES UPDATE (RETURNS AFTER CADR)
		CADR	NEEDLER		#      (NEEDLES RESTARTS ITSELF)

; ============================================================================
; VARGAINS - VARIABLE GAIN AND MASS PROPERTY UPDATES
;
; Checks the engine-on status bit (bit 13 of channel 11) to determine whether
; to update DAP gains and spacecraft mass properties. These updates are
; crucial during SPS burns as propellant consumption continuously changes the
; spacecraft's mass and center of gravity, requiring corresponding adjustments
; to autopilot control gains for stable engine steering.
;
; If engine is OFF: Bypasses gain/mass updates and jumps to 1SHOTCHK.
; If engine is ON: Proceeds to update vehicle mass, call MASSPROP for inertia
;                  calculations, and update pitch/yaw/roll DAP gains.
;
; The VCNTR (variable gain counter) determines timing of these computationally
; expensive updates, allowing them to occur at longer intervals than the
; 0.5-second TVCEXEC cycle.
; ============================================================================

VARGAINS	CAF	BIT13		# CHECK ENGINE-ON BIT TO INHIBIT VARIABLE
		EXTEND			#      GAINS AND MASS IF ENGINE OFF
		RAND	DSALMOUT	# CHANNEL 11
		CCS	A
		TCF	+4		#     ON , SO OK TO UPDATE GAINS AND MASS
	+5	CAF	TWO		#      OFF, SO BYPASS MASS/GAIN UPDATES,
		TS	TVCEXPHS	#	    ALSO ENTRY FROM CCS BELOW WITH
		TCF	1SHOTCHK	#	    VCNTR = -0 (V97 R40 ENGFAIL)
		CCS	VCNTR		#      TEST FOR GAIN OF UPDATE TIME
		TCF	+4		#	    NOT YET
# Page 947
		TCF	GAINCHNG	#		NOW
		TCF	+0		#		NOT USED
		TCF	VARGAINS +5	#		NO, LOTHRUST (S40.8 R40)

	+4	TS	VCNTRTMP	#	 PROTECT VCNTR AND
		CAE	CSMMASS		#	CSMMASS DURING AN IMPULSIVE BURN
		TS	MASSTMP
		TCF	EXECCOPY

GAINCHNG	TC	IBNKCALL	# UPDATE IXX, IAVG, IAVG/TLX
		CADR	FIXCW		# MASSPROP ENTRY  (ALREADY INITIALIZED)
		TC	IBNKCALL	# UPDATE 1/CONACC, VARK
		CADR	S40.15		#      (S40.15 IS IN TVCINITIALIZE)
		CS	TENMDOT		# UPDATE MASS FOR NEXT 10 SEC. OF BURN
		AD	CSMMASS
		TS	MASSTMP		# KG B+16

		CAF	NINETEEN	# RESET THE VARIABLE-GAIN UPDATE COUNTER
		TS	VCNTRTMP

EXECCOPY	INCR	TVCEXPHS	# RESTART-PROTECT THE COPYCYCLE        (1)

		CAE	MASSTMP		# CSMMASS KG B+16
		TS	CSMMASS

		CAE	VCNTRTMP	# VCNTR
		TS	VCNTR
		TS	V97VCNTR	# FOR ENGFAIL (R41) MASS UPDATES AT SPSOFF

		INCR	TVCEXPHS	# COPYCYCLE OVER                       (2)

; ============================================================================
; 1SHOTCHK - ONE-SHOT ENGINE TRIM CORRECTION TIMING CHECK
;
; Manages timing for the initial engine trim correction that occurs 0-3 seconds
; after SPS ignition. During engine start, the thrust vector may not align
; perfectly with the spacecraft's center of gravity due to propellant slosh,
; thermal effects, or slight misalignments. This one-shot correction measures
; the actual thrust vector offset and applies compensating trim angles to the
; engine gimbals.
;
; CNTR countdown values determine correction timing:
; - Positive: Not yet time for correction, continue countdown
; - Zero: Execute one-shot correction now (jump to 1SHOTOK)
; - Negative non-zero: One-shot complete, check for repetitive corrections (REPCHEK)
; - Minus zero: One-shot only, no repetitive corrections follow
;
; This correction was critical during Apollo 11's TLI burn, ensuring the
; engine thrust remained aligned for the precise velocity change needed to
; depart Earth orbit toward the Moon.
; ============================================================================

1SHOTCHK	CCS	CNTR		# CHECK TIME FOR ONE-SHOT OR REPCORR
		TCF	+4		#      NOT YET
		TCF	1SHOTOK		#      NOW
		TCF	REPCHEK		#      ONE-SHOT OVER, ON TO REPCORR
		TCF	1SHOTOK		#      NOW  (ONE-SHOT ONLY, NO REPCORR)

	+4	TS	CNTRTMP		# COUNT DOWN
		CAF	SIX		# SET UP TVCEXPHS FOR ENTRY AT CNTRCOPY
		TS	TVCEXPHS
		TCF	CNTRCOPY

; ============================================================================
; REPCHEK - REPETITIVE ENGINE TRIM CORRECTION CHECK
;
; After the initial one-shot correction, this routine determines whether
; repetitive trim corrections should continue throughout the burn. Repetitive
; corrections compensate for gradual changes in thrust vector alignment as
; propellant drains from tanks, shifting the center of gravity, and as thermal
; expansion affects engine mounting structure.
;
; REPFRAC (repetitive correction fraction) controls this behavior:
; - Zero or negative: No repetitive corrections, terminate TVC executive (TVCEXFIN)
; - Positive: Enable repetitive corrections at regular intervals, proceed to CORSETUP
;
; The pad-loaded EREPFRAC value (stored in REPFRAC) allows mission planners to
; customize correction strategy for each burn based on expected propellant usage
; and thermal environment. Long burns like TLI used repetitive corrections;
; short burns often used one-shot only.
; ============================================================================

REPCHEK		CAE	REPFRAC		# CHECK FOR REPETITIVE UPDATES
		EXTEND
		BZMF	TVCEXFIN	#      NO, OVER-AND-OUT
		TS	TEMPDAP +1	#      YES, SET UP CORRECTION FRACTION
		CAF	FOUR		# SET UP TVCEXPHS FOR ENTRY AT CORSETUP
		TS	TVCEXPHS
		TCF	CORSETUP
# Page 948
; ============================================================================
; 1SHOTOK - ENGINE-ON VERIFICATION FOR ONE-SHOT CORRECTION
;
; Verifies that the engine remains on before executing the one-shot correction.
; This safety check prevents attempting trim corrections during engine shutdown
; transients when control authority may be compromised.
;
; Reads engine-on bit (bit 13) from channel 11 (DSALMOUT). If engine is off,
; immediately terminates TVCEXEC by branching to TVCEXFIN. If engine is on,
; increments TVCEXPHS phase counter to (3) and proceeds to TEMPSET for
; correction fraction setup.
;
; This check is critical during the 0-3 second window after ignition when
; engine start transients are settling—the one-shot correction compensates
; for initial CG offset but must only execute with stable engine operation.
; ============================================================================

1SHOTOK		CAF	BIT13		# CHECK ENGINE-ON BIT, NOT PERMITTING
		EXTEND			#      ONE-SHOT DURING ENGINE-SHUTDOWN
		RAND	DSALMOUT
		CCS	A
		TCF	+2		#      ONE-SHOT OK
		TCF	TVCEXFIN	#      NO, TERMINATE

		INCR	TVCEXPHS	#					(3)

# RSB 2009.  The following instruction was previously "CAE FCORFRAC", but FCORFRAC
# is not in erasable memory as implied by the use of CAE.  I've accordingly changed
# it to CAF instead to indicate fixed memory.

; ============================================================================
; TEMPSET - CORRECTION FRACTION INITIALIZATION
;
; Loads the fixed correction fraction (FCORFRAC) into temporary storage
; (TEMPDAP+1) for subsequent adjustment based on LEM configuration. FCORFRAC
; is a pad-loaded constant defining the gain/magnitude of trim corrections—
; too small and CG offsets cause attitude errors, too large and corrections
; overshoot causing instability.
;
; This value represents the base correction fraction for LEM-ON configuration
; (combined CSM+LM mass). The following CORSETUP section will double this
; value if LEM is off (CSM-only), reflecting different mass/inertia properties
; and CG locations between docked and undocked configurations.
;
; Note: RSB 2009 transcription correction—instruction changed from CAE to CAF
; to correctly indicate fixed memory rather than erasable memory addressing.
; ============================================================================

TEMPSET		CAF	FCORFRAC	#      SET UP CORRECTION FRACTION
		TS	TEMPDAP +1

		INCR	TVCEXPHS	# ENTRY FROM REPCHECK AT NEXT LOCATION	(4)

; ============================================================================
; CORSETUP - LEM CONFIGURATION CORRECTION SCALING
;
; Adjusts the correction fraction based on whether the Lunar Module is docked
; (LEM-ON) or jettisoned (LEM-OFF). The spacecraft's mass, inertia, and CG
; location differ significantly between these configurations, requiring
; different trim correction gains for stable control.
;
; Reads DAPDATR1 bit 13 (LEM-OFF indicator, note inverted sense):
; - If bit 13 = 0 (BZF branch taken): LEM is ON (docked configuration)
;     Uses TEMPDAP+1 directly (base FCORFRAC value)
; - If bit 13 = 1 (BZF not taken): LEM is OFF (CSM-only configuration)
;     Doubles the correction by loading TEMPDAP+1 twice and adding:
;     TEMPDAP = TEMPDAP+1 + TEMPDAP+1 = 2 * FCORFRAC
;
; The doubling for LEM-OFF reflects the CSM-only spacecraft's different
; response characteristics. With lower total mass but similar engine thrust,
; corrections must be scaled appropriately to avoid under-correcting attitude
; errors during burns.
;
; Final value stored in TEMPDAP is used by CG.CORR section for TMC loop
; calculations, scaling the trim angle corrections to match current vehicle
; configuration.
; ============================================================================

CORSETUP	CAE	DAPDATR1	# CHECK FOR LEM-OFF/ON
		MASK	BIT13		# (NOTE, SHOWS LEM-OFF)
		EXTEND
		BZF	+2		# LEM IS ON,  PICK UP   TEMPDAP+1
		CAE	TEMPDAP +1	# LEM IS OFF, PICK UP 2(TEMPDAP+1)
		AD	TEMPDAP +1
		TS	TEMPDAP		# CG.CORR USES TEMPDAP

		CAF	NEGONE		# SET UP FOR CNTR = -1 (ONE-SHOT DONE)
		TS	CNTRTMP		#      (COPYCYCLE AT  .CNTRCOPY. )

; ============================================================================
; CG.CORR - CENTER-OF-GRAVITY TRIM CORRECTION CALCULATION
;
; Computes engine trim angle corrections for pitch and yaw axes to compensate
; for center-of-gravity (CG) offset from the thrust vector centerline. As
; propellant drains during an SPS burn, the CG shifts, requiring continuous
; gimbal angle adjustments to maintain stable flight without inducing unwanted
; rotation.
;
; The calculation uses the TMC (Thrust-Minus-Correction) loop algorithm:
; For each axis (pitch and yaw):
;   1. Read current offset (PDELOFF/YDELOFF) and commanded rates (DELPBAR/DELYBAR)
;   2. Compute correction = (commanded - 4*current_offset + 4*commanded_rate) * fraction
;   3. Add correction to accumulated trim (PACTTMP/YACTTMP)
;
; Double-precision arithmetic (DCA, DXCH, DAS) maintains accuracy despite
; scaling requirements. DDOUBL instructions multiply by 4 (shift left 2 bits).
; TEMPDAP contains the correction fraction scaled appropriately for LEM-ON
; or LEM-OFF configurations.
;
; These trim angles (PACTOFF, YACTOFF) feed directly to the engine gimbal
; actuators, physically steering the 20,500-pound-thrust SPS engine to maintain
; spacecraft attitude during critical burns like Apollo 11's LOI.
; ============================================================================

CG.CORR		EXTEND			# PITCH TMC LOOP
		DCA	PDELOFF
		DXCH	PACTTMP
		EXTEND
		DCS	PDELOFF
		DDOUBL
		DDOUBL
		DXCH	TTMP1
		EXTEND
		DCA	DELPBAR
		DDOUBL
		DDOUBL
		DAS	TTMP1
		EXTEND
		DCA	TTMP1
		EXTEND
		MP	TEMPDAP
		DAS	PACTTMP

		EXTEND			# YAW TMC LOOP
		DCA	YDELOFF
		DXCH	YACTTMP
		EXTEND
		DCS	YDELOFF
		DDOUBL
# Page 949
		DDOUBL
		DXCH	TTMP1
		EXTEND
		DCA	DELYBAR
		DDOUBL
		DDOUBL
		DAS	TTMP1
		EXTEND
		DCA	TTMP1
		EXTEND
		MP	TEMPDAP
		DAS	YACTTMP

; ============================================================================
; CORCOPY - COPY TRIM CORRECTIONS TO OUTPUT VARIABLES
;
; Transfers the computed trim angle corrections from temporary computation
; registers to the official output variables that drive engine gimbal commands.
; This separation between computation (PACTTMP/YACTTMP) and output (PACTOFF/
; YACTOFF) enables restart protection—if a restart occurs during computation,
; the previous valid trim values remain in the output registers.
;
; Additionally, current offset values are copied to PDELOFF/YDELOFF for use
; in the next correction cycle's calculations, maintaining continuity in the
; TMC loop feedback control.
;
; The TVCEXPHS phase counter increments at entry and exit of this section,
; providing restart protection boundaries. If the AGC restarts during this
; copycycle (unlikely given its short execution time), TVCRESTART logic uses
; TVCEXPHS to determine safe re-entry point.
; ============================================================================

CORCOPY		INCR	TVCEXPHS	# RESTART PROTECT THE COPYCYCLE		(5)

		EXTEND			# TRIM-ESTIMATES, AND
		DCA	PACTTMP
		TS	PACTOFF		#	TRIMS
		DXCH	PDELOFF

		EXTEND
		DCA	YACTTMP
		TS	YACTOFF
		DXCH	YDELOFF

		INCR	TVCEXPHS	# ENTRY FROM 1SHOTCHK AT NEXT LOCATION	(6)

; ============================================================================
; CNTRCOPY - COUNTER UPDATE FOR CORRECTION TIMING
;
; Copies the temporary countdown value (CNTRTMP) to the official counter (CNTR)
; that tracks timing for one-shot and repetitive corrections. This brief
; copycycle is restart-protected by following the CORCOPY copycycle, ensuring
; restart logic can safely resume operations.
;
; The counter decrements on each TVCEXEC cycle (every 0.5 seconds) until
; reaching zero, triggering the appropriate correction phase (one-shot or
; repetitive).
; ============================================================================

CNTRCOPY	CAE	CNTRTMP		# UPDATE CNTR (RESTARTS OK, FOLLOWS CPYCY)
		TS	CNTR

; ============================================================================
; TVCEXFIN - TVC EXECUTIVE TERMINATION
;
; Graceful exit point for TVCEXEC when no further processing is required.
; Reached when:
; - Engine shutdown detected (engine-on bit cleared)
; - Repetitive corrections disabled (REPFRAC zero or negative)
; - Termination flags set (FLAGWRD6 bits 15,14 indicating RCSDAPON takeover)
;
; Resets TVCEXPHS phase counter to zero, clearing restart protection state,
; and returns control via TASKOVER to the executive scheduler's waitlist
; management. The self-perpetuating 0.5-second waitlist calls cease,
; transferring attitude control authority from TVC (thrust vector control)
; to RCS (reaction control system) for coast phases.
;
; During Apollo 11's mission, this transition occurred after each major burn:
; after TLI (departing Earth), after LOI (entering lunar orbit), and after
; TEI (departing lunar orbit for home).
; ============================================================================

TVCEXFIN	CAF	ZERO		# RESET TVCEXPHS
		TS	TVCEXPHS
		TCF	TASKOVER	# TVCEXECUTIVE FINISHED

FCORFRAC	OCT	10000		# ONE-SHOT CORRECTION FRACTION

# Page 950 (page is empty)

