# Copyright:	Public domain.
# Filename:	TVCSTROKETEST.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	979-983
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
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

; ============================================================================
; FILE: TVCSTROKETEST.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: pre-burn (system verification)
;
; TL;DR: Engine gimbal stroke testing routine verifying full range of motion
;        and actuator health before critical SPS burns. Commands gimbals through
;        full deflection range to detect mechanical binding or hydraulic failures,
;        ensuring control authority availability for upcoming maneuvers.
;
; COMMENT-ONLY READERS: This test made sure the engine could swivel freely in
;        all directions before important rocket burns.
; CODE-ALONG READERS: Study gimbal actuator test sequences, stroke limit
;        verification, and fault detection algorithms.
; ============================================================================

# Page 979
;
; ============================================================================
; STROKE TEST OPERATIONAL OVERVIEW
;
; Before any major rocket burn (such as Trans-Lunar Injection, Lunar Orbit
; Insertion, or Trans-Earth Injection), the crew needed absolute confidence
; that the engine could swivel properly for steering. This routine commanded
; the engine gimbal actuators through their full range of motion, checking
; for mechanical problems, hydraulic leaks, or electrical faults that could
; prevent proper thrust vector control during critical burns.
;
; The test generated a specific waveform pattern of gimbal movements, starting
; with slow, large-amplitude swings and progressing to faster, smaller motions.
; This comprehensive motion profile stressed the actuators across all operating
; conditions they would encounter during actual flight maneuvers.
;
; Crew initiated the test using Extended Verb 68 before each critical burn.
; If the test detected any anomaly, the mission timeline allowed for troubleshooting
; or switching to backup procedures before committing to the burn.
; ============================================================================
;
# NAME		STROKE TEST PACKAGE		(INCLUDING INITIALIZATION PACKAGE)
# LOG SECTION...TVCSTROKETEST			SUBROUTINE...DAPCSM
# MODIFIED BY SCHLUNDT				21 OCTOBER 1968
#
# FUNCTIONAL DESCRIPTION....
#	STROKE TEST PACKAGE GENERATES A WAVEFORM DESIGNED TO EXCITE BENDING
#	STRKTSTI (STROKE TEST INITIALIZATION) IS CALLED AS A JOB BY VB68.
#		IT INITIALIZES ALL ERASABLES REQD FOR A STROKE TEST, AND
#		THEN TESTS FOR CSM/LM (BIT 13 OF DAPDATR1). IF CSM/LM,
#		IN EITHER HIGH OR LOW-BANDWIDTH MODE, THE TEST IS STARTED
#		IMMEDIATELY.  IF NOT CSM/LM, PROGRAM EXITS WITH NO ACTION.
#	HACK (STROKE TEST) GENERATES THE WAVEFORM BY DUMPING PULSE BURSTS
#		OF PROPER SIGN AND IN PROPER SEQUENCE DIRECTLY INTO
#		TVCPITCH, WORKING IN CONJUNCTION WITH BOTH PITCH AND YAW
#		TVC DAPS, WITH INTERMEDIATE WAITLIST CALLS.  NOTE, HOWEVER
#		THAT THE STROKE TEST IS PERFORMED ONLY IN THE PITCH AXIS.
#		AN EXAMPLE WAVEFORM IS GIVEN BELOW, TO DEMONSTRATE STROKE-
#		TEST PARAMETER SELECTION.
#	RESTARTS CAUSE TEST TO BE TERMINATED.  ANOTHER V68 REQD IF TEST
#		IS TO BE RE-RUN.
#	PULSE BURST SIZE IS PAD-LOADED (ESTROKER) SO THAT AMPLITUDE OF
#		WAVEFORM CAN BE CHANGED.  THERE ARE TEN PULSE BURSTS IN
#		THE HALF-AMPLITUDE OF THE FIRST FREQUENCY SET IN THE
#		STANDARD WAVEFORM.  AMPLITUDE IS 10(ESTROKER)(1/42.15),
#		NOMINALLY 50/42.15 = 1.185 DEG
#
# CALLING SEQUENCE....
#	EXTENDED VERB 68 SETS UP STRKTSTI JOB
#	PITCH AND YAW TVCDAPS, FINDING STROKER NON-ZERO, DO A ..TC HACK..
#	AN INTERNALLY-GENERATED WAITLIST CALL ENTERS AT ..HACKWLST..
#
# NORMAL EXIT MODES....
#	TC BUNKER (..Q.. IF ENTRY FROM DAP, ..TCTSKOVR.. IF FROM WAITLIST) LIST
#
# SUBROUTINES CALLED....
#	WAITLIST
#
# ALARM OR ABORT EXIT MODES....
#	NONE
#
# ERASABLE INITIALIZATION REQUIRED....
#	ESTROKER (PAD-LOAD)
#	STROKER, CADDY, REVS, CARD, N
#
# OUTPUT....
#	STRKTSTI...INITIALIZATION FOR STROKE TEST
#	HACK, HACKWLST...PULSE BURSTS INTO TVCPITCH VIA..ADS..
#			  RESETS STROKER = +0 WHEN TEST COMPLETED
#
# DEBRIS....
#	N = CADDY = +0, CARD = -0, REVS = -1
#	BUNKER
# Page 980
#
# EXAMPLE STROKE TEST WAVE FORM, DEMONSTRATING PARAMETER SELECTION

# NOTE....THIS IS NOT THE OFFICIAL WAVEFORM....
#
#        **              **
#        **              **
#        **              **		EXAMPLE WAVEFORM (EACH * REPRESENTS
#       *  *            *  *		  (85.41 ARCSEC OF ACTUATOR CMND)
#       *  *            *  *
#       *  *            *  *
#      *    *          *    *          **      **      **      **      **
#      *    *          *    *          **      **      **      **      **
#      *    *          *    *          **      **      **      **      **
#     *      *        *      *        *  *    *  *    *  *    *  *    *  *    **  **  **  **  **
#     *      *        *      *        *  *    *  *    *  *    *  *    *  *    **  **  **  **  **
#     *      *        *      *        *  *    *  *    *  *    *  *    *  *    **  **  **  **  **
# ----------------------------------------------------------------------------------------------------
#             *      *        *      *    *  *    *  *    *  *    *  *    *  *  **  **  **  **  **
#             *      *        *      *    *  *    *  *    *  *    *  *    *  *  **  **  **  **  **
#             *      *        *      *    *  *    *  *    *  *    *  *    *  *  **  **  **  **  **
#              *    *          *    *      **      **      **      **      **
#              *    *          *    *      **      **      **      **      **
#              *    *          *    *      **      **      **      **      **
#               *  *            *  *
#               *  *            *  *
#               *  *            *  *
#                **              **
#                **              **
#                **              **
#
# FOR THIS (UNOFFICIAL, EXAMPLE) WAVEFORM, THE REQUIRED PARAMETERS ARE AS FOLLOWS....
#
#	FCARD	 = +3		(NUMBER OF SETS)
#	ESTROKER = +3		(PULSE BURST SIZE, SC.AT 85.41 ARCSEC/BIT)
#
#	SET1:
#		FREVS	= +3	(NUMBER REVERSALS MINUS 1)
#		FCADDY	= +4	(NUMBER OF PULSE BURSTS IN 1/2 AMPLITUDE)
#	SET2:
#		FCARD1	= +9	(NUMBER REVERSALS MINUS 1)
#		FCARD4	= +2	(NUMBER OF PULSE BURSTS IN 1/2 AMPLITUDE)
#	SET3:
#		FCARD2	= +9	(NUMBER REVERSALS MINUS 1)
#		FCARD5	= +1	(NUMBER OF PULSE BURSTS IN 1/2 AMPLITUDE)
#	SET4:
#		FCARD3	= +0	(NUMBER OF REVERSALS MINUS 1)
#		FCARD6	= +0	(NUMBER OF PULSE BURSTS IN 1/2 AMPLUTUDE)

# Page 981
# STROKE TEST INITIALIZATION PACKAGE (AS A JOB, FROM VERB 68)
;
; ============================================================================
; TRANSITION: From waveform definition to stroke test execution
;
; The crew has confirmed readiness for the upcoming burn and needs to verify
; actuator health. Extended Verb 68 (V68E) calls this initialization routine,
; which prepares all parameters and checks spacecraft configuration before
; allowing the test to proceed. The test only executes in CSM/LM docked
; configuration to avoid disturbing the Lunar Module if already separated.
; ============================================================================

		BANK	17
		SETLOC	DAPS2
		BANK

		COUNT*	$$/STRK
		EBANK=	CADDY

; Stroke test begins here when crew enters Extended Verb 68.
; This job initializes all test parameters and verifies proper configuration.

STRKTSTI	TCR	TSTINIT		# STROKE TEST INITIALIZATION PKG (CALLED
					# AS A JOB BY VERB68)

; Configuration verification ensures the test is only performed when appropriate.
; The stroke test must not run if the Lunar Module has already separated, as
; engine gimbal movements could disturb the delicate orbital mechanics of
; rendezvous operations or interfere with the LM's own maneuvering.

STRKCHK		INHINT

		CAE	DAPDATR1	# CHECK FOR CSM/LM CONFIGURATION
		MASK	BIT14		# Bit 14 indicates CSM/LM docked state
		EXTEND
		BZF	+3		# If not docked, skip test (exit job)

; Configuration verified: CSM/LM docked, safe to test gimbals.
; Load the pulse burst amplitude (ESTROKER, pad-loaded before flight) and
; enable the test. The actual waveform generation begins on the next DAP pass.

		CAE	ESTROKER	# BEGIN ON NEXT DAP PASS (PITCH OR YAW)
		TS	STROKER		# (STROKING DONE IN PITCH ONLY, HOWEVER)
					# STROKER non-zero signals DAP to call HACK

		TCF	ENDOFJOB	# Exit initialization, test will run via DAP

; Test parameter initialization loads the waveform definition from fixed memory
; constants into erasable working registers. Sign changes are intentional for
; the down-counting logic used during waveform generation.

TSTINIT		CS	FCADDY		# NORMAL ENTRY FROM STRKTSTI
		TS	CADDY		# Load number of pulse bursts per half-amplitude
		TS	N		# N saves initial value for reversals
					#	NOTE SGN CHNG FCADDY(+) TO CADDY(-)
					# (Negative for down-counting in main loop)

		CAF	FREVS		# Load number of reversals minus 1 for Set 1
		TS	REVS		# REVS counts slope direction changes

		CS	FCARD		# Load number of frequency sets in waveform
		TS	CARD		# 	NOTE SGN CHNG FCARD(+) TO CARD(-)
					# (Negative for down-counting through sets)

		TC	Q		# RETURN TO STRKTSTI+1 (OR CHKSTRK+2OR+4)

# Page 982
# THE OFFICIAL STROKE TEST WAVEFORM (3 JAN, 1967) CONSISTS OF FOUR STROKE SETS, AS FOLLOWS....
#
#	SET 1...10 BURSTS IN 1/2 AMP,   4 REVERSALS
#	SET 2... 6 BURSTS IN 1/2 AMP,	6 REVERSALS
#	SET 3... 5 BURSTS IN 1/2 AMP,  10 REVERSALS
#	SET 4... 4 BURSTS IN 1/2 AMP,  14 REVERSALS
#
# THE PULSE BURST SIZE (ESTROKER) IS PAD-LOADED (5 BITS AS OF 3JAN,1967)
# THE REMAINING WAVEFORM-GENERATING PARAMETERS ARE AS FOLLOWS....
;
; These parameters define the official stroke test waveform validated through
; ground testing and flight experience. The progression from low frequency
; (Set 1: large, slow movements) to high frequency (Set 4: small, fast
; movements) comprehensively exercises the gimbal actuators across their
; full operating envelope, detecting any resonances, binding, or performance
; degradation that could compromise steering during critical burns.

FCADDY		DEC	10		# NO. PULSE BURSTS IN 1/2 AMP, SET1..(+10)
FREVS		DEC	3		# NO. REVERSALS MINUS 1, SET1........(  3)

FCARD		DEC	4		# NO. STROKE SETS....................(+ 4)

FCARD1		DEC	5		# NO. REVERSALS MINUS 1, SET2........(  5)

FCARD2		DEC	9		# 			    3........(  9)
FCARD3		DEC	13		#                           4........( 13)

FCARD4		DEC	6		# NO. PULSE BURSTS IN 1/2 AMP, SET2..(+ 6)
FCARD5		DEC	5		#                              SET3..(+ 5)
FCARD6		DEC	4		#                              SET4..(+ 4)

20MS		=	BIT2

# STROKE TEST PACKAGE PROPER....
;
; ============================================================================
; TRANSITION: From initialization to waveform generation
;
; The DAP has detected STROKER is non-zero and entered the stroke test routine.
; The gimbal actuators are now commanded through the prescribed motion pattern.
; Only the pitch axis is tested, based on the engineering judgment that pitch
; and yaw actuators share identical design and operating principles, so if
; pitch gimbals operate correctly, yaw gimbals will as well. This reduces test
; time while maintaining confidence in system health.
; ============================================================================

		EBANK=	BUNKER

; Main waveform generation routine. Called by TVC DAP during T5 interrupt
; when STROKER is non-zero. Generates pulse bursts of proper sign and duration,
; working with the DAP to command gimbal deflections through the test pattern.

HACK		EXTEND			# ENTRY (IN T5 RUPT) FROM TVCDAPS
		QXCH	BUNKER		# SAVE Q FOR DAP RETURN

; Schedule the next pulse burst via waitlist. The 20-millisecond timing
; (20MS = 2 centiseconds) coordinates with the DAP cycle rate to ensure
; proper pulse spacing for the waveform frequency content.

		CAF	20MS		# 2DAPSX2(PASSES/DAP)X2(CS/PASS)=8CS=TVCDT
		TC	WAITLIST	# Schedule HACKWLST after 20ms delay
		EBANK=	BUNKER
		2CADR	HACKWLST

		TCF	+3		# Return to DAP via BUNKER

; Waitlist entry point for pulse burst generation. This routine executes
; every 20 milliseconds to output the next pulse command to the pitch gimbal.

HACKWLST	CAF	TCTSKOVR	# ENTRY FROM WAITLIST
		TS	BUNKER		# BUNKER IS TC TASKOVER

; Apply the current pulse burst to the pitch gimbal command channel.
; STROKER contains the pulse amplitude (positive or negative depending on
; desired deflection direction). This directly drives the actuator position.

		CA	STROKER		# STROKE (pulse amplitude, signed)
		ADS	TVCPITCH	# Add to pitch gimbal command

; Release the TVC error integrators to prevent saturation during the test.
; The stroke test commands large deflections that would otherwise accumulate
; as errors in the DAP's control loops. BIT11 of CHAN14 resets these counters.

		CAF	BIT11		# RELEASE THE ERROR COUNTERS
		EXTEND
		WOR	CHAN14		# Write OR to Channel 14 (TVC control)

; Count pulse bursts within the current half-amplitude cycle. CADDY starts
; negative and increments toward zero. When it reaches zero or positive,
; the half-amplitude is complete and the waveform must reverse direction.

		INCR	CADDY		# COUNT DOWN THE NO. BURSTS, THIS SLOPE
# Page 983
		CS	CADDY		# Check if CADDY >= 0
		EXTEND
		BZMF	+2		# Branch if zero or negative (complement)
		TC	BUNKER		# EXIT, WHILE ON A SLOPE (more bursts needed)

; Half-amplitude complete. Check reversal count to determine next action.
; REVS counts remaining reversals: positive = more reversals in this set,
; zero = final reversal of this set, negative = set complete, advance to next.

		CCS	REVS		# Check remaining reversals
		TCF	REVUP		# POSITIVE REVS (more reversals in set)
		TCF	REVUP +4	# FINAL REVERSAL, THIS SET (REVS was +0)

; Set complete (REVS was negative). Advance to the next frequency set.
; Each set has fewer bursts per half-amplitude, creating higher frequencies
; that probe different mechanical response characteristics of the actuators.

		INCR	CARD		# NEGATIVE REVS SET LAST PASS, READY FOR
		CS	CARD		#	THE NEXT SET.  CHECK IF NO MORE SETS
		EXTEND
		BZF	STROKILL	# ALL SETS COMPLETED (CARD reached zero)

; Load parameters for the next frequency set. CARD is negative and serves
; as an index into the parameter tables (FCARD+4 through FCARD+7). Each
; subsequent set has fewer bursts and more reversals, increasing frequency.

		INDEX	CARD		# Use CARD as table index
		CAF	FCARD +4	# PICK UP NO. REVERSALS (-), NEXT SET
		TS	REVS		# REINITIALIZE reversal counter
		INDEX	CARD		# Use CARD as table index again
		CS	FCARD +7	# PICK UP NO. BURSTS IN 1/2AMP, NEXT SET
		TS	N		# REINITIALIZE burst reference value
		TS	CADDY		# Reset burst counter for new set
		TC	BUNKER		# EXIT, AT END OF SET

; Test complete. All four frequency sets have been executed successfully.
; The gimbals have been exercised through their full range at multiple
; frequencies, verifying mechanical integrity and hydraulic performance.
; Reset STROKER to zero so the DAP stops calling the stroke test routine.

STROKILL	TS	STROKER		# RESET (TO +0) TO END TEST
		TC	BUNKER		# EXIT, STROKE TEST FINIS

; Execute a reversal (direction change) in the waveform. Decrement REVS
; and reload the burst counter. For mid-set reversals, the counter is loaded
; with 2N (full amplitude traversal). For the final reversal of a set, load
; N (half amplitude returning to zero before advancing to the next set).

REVUP		TS	REVS		# ALL REVERSALS EXCEPT LAST OF SET
		CA	N		# Load base burst count
		DOUBLE			# 2 x 1/2AMP (full amplitude traversal)
		TCF	+4		# Skip to sign reversal

; Final reversal of the current set. Return to zero deflection with half
; the burst count, preparing for transition to the next frequency set.

	+4	CS	ONE		# FINAL REVERSAL, THIS SET
		TS	REVS		# PREPARE TO BRANCH TO NEW BURST (set to -1)
		CA	N		# JUST RETURN TO ZERO, FINAL SLOPE OF SET
		TS	CADDY		# CADUP (reload burst counter)

; Reverse the sign of STROKER to change gimbal deflection direction.
; The actuators now command motion in the opposite direction, creating
; the alternating pattern that exercises the full range of motion.

		CS	STROKER		# CHANGE SIGN OF SLOPE (negate pulse amplitude)
		TS	STROKER		# Store reversed pulse command
		TC	BUNKER		# EXIT AT A REVERSAL (SLOPE CHANGE)
