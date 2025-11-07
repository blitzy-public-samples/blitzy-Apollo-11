# Copyright:	Public domain.
# Filename:	TVCRESTARTS.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	956-960
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-20 RSB	Corrections:  TCF -> BZF in one place.
#		2009-05-21 RSB	In PHSCHK2, CS TVCPHASE corrected to
#				CCS TVCPHASE and CCS 4 corrected to CCS A.
#				Page 924 corrected to 961.  CORCOPY +2
#				corrected to CORCOPY +1.
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
; FILE: TVCRESTARTS.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth (SPS burns)
;
; TL;DR: TVC restart protection preserving control system state during AGC
;        restart events. Ensures TVC can resume proper operation if restart
;        occurs during critical SPS burn, maintaining engine control authority
;        and preventing loss of mission.
;
; COMMENT-ONLY READERS: This program protected engine steering if the computer
;        had to restart during a rocket burn - a critical safety feature.
; CODE-ALONG READERS: Study restart protection mechanisms for TVC state
;        preservation and recovery procedures.
; ============================================================================

# Page 956
# NAME....TVCRESTART PACKAGE,  CONSISTING OF REDOTVC, ENABL1, 2, CMDSOUT, PHSCHK2, ETC.
# LOG SECTION...TVCRESTARTS			SUBROUTINE....DAPCSM
# MODIFIED BY SCHLUNDT				21 OCTOBER 1968
# FUNCTIONAL DESCRIPTION....
#
#      *RESTART-PROOFS THE TVC DAPS, INCLUDING PITCHDAP, YAWDAP,
#	TVCEXECUTIVE, ROLLDAP, TVCINIT4, TVCDAPON, AND CSM/LM V46 SWTCHOVR.
#      *TVC RESTARTS DESERVE SPECIAL CONSIDERATION IN SEVERAL AREAS.
#	RESTART DOWN-TIME IS IMPORTANT BECAUSE OF THE TRANSIENTS INTRODUCED
#	BY THE THRUST VECTOR RETURN TO THE ACTUATOR MECHANICAL NULLS
#	FOLLOWING TVC- AND OPTICS-ERROR-COUNTER-DISENABLES (CHANNEL 12).
#	TVC    USES A MIXTURE OF WAITLIST, T5, T6, AND JOB CALLS. THERE IS
#	FILTER MEMORY (UP TO 6TH ORDER) TO BE PROTECTED IF WILD TRANSIENTS
#	ARE TO BE AVOIDED. COUNTERS ARE INVOLVED FOR ONE-SHOT
#	CORRECTIONS AND GAIN UPDATES. THE GIMBAL TRIM ESTIMATORS AND THE
#	BODY AXIS ATTITUDE ERROR INTEGRATORS INVOLVE DIGITAL SUMMATION.
#	DIGITAL DIFFERENTIATORS ARE INVOLVED IN THE BODY AXIS RATE ESTIMA-
#	TIONS AND IN THE OUTPUTTING OF ACTUATOR COMMANDS. THERE IS AN
#	OFFSET-TRACKER-FILTER TO PROTECT. ETC., ETC.
#      *THOSE QUANTITIES WHICH MUST BE PROTECTED ARE STORED IN TEMPORARY
#	REGISTERS AS THEY ARE COMPUTED, FOR UPDATING THE REAL REGISTERS
#	DURING COPYCYCLES.
#      *THE SEVERAL COPYCYCLES ARE EACH PROTECTED BY PHASE POINTS AT THEIR
#	BEGINNING AND AT THEIR TERMINATION. THE PHASE POINTS ARE SIMPLY
#	..INCR.. INSTRUCTIONS,   EITHER ..INCR TVCEXPHS.. FOR COPYCYCLES
#	IN THE TVCEXECUTIVE, OR ..INCR TVCPHASE.. FOR THE PITCH AND YAW
#	COPYCYCLES. INDEXING ON EACH OF THESE POINTERS THEN PERMITS A
#	RETURN TO THE APPROPRIATE RESTART POINTS.
#      *IF A RESTART OCCURS DURING EITHER COPYCYCLE, THAT COPYCYCLE IS
#	COMPLETED. THEN THE NORMAL TVCINIT4....DAPINIT....PITCHDAP STARTUP
#	SEQUENCE IS CALLED UPON TO GET THINGS GOING AGAIN.
#      *TVC-ENABLE AND OPTICS-ERROR-COUNTER ENABLE MUST BE SET ASAP
#	(ALLOWING FOR PROCEDURAL DELAYS). THEN THE ENGINES ARE COMMANDED
#	TO THE P,YACTOFF TRIM VALUES. THE DAPS ARE THEN READY TO GO ON THE
#	AIR, WITH THE REGULAR STARTUP SEQUENCE, EITHER AT MRCLEAN FOR A
#	COMPLETE INITIALIZATION OR AT TVCINIT4 FOR A PARTIAL INITIALIZATION
#      *FOR RESTARTS PRIOR TO THE SETTING OF THE T5 BITS AT DOTVCON THE
#	PRE40.6 SECTION OF S40.6 TAKES CARE OF RE-ESTABLISHING TRIMS.
#      *IF A RESTART OCCURS DURING THE TVCEXEC....TVCEXFIN SEQUENCE THE
#	COMPUTATIONS WILL BE COMPLETED, STARTING AT THE APPROPRIATE RESTART
#	POINT, AFTER THE DAPS ARE READY TO GO ON THE AIR.
#      *IF A RESTART OCCURS PRIOR TO TVCINIT4 (TVCPHASE = -1) E.G. DURING
#	THE EARLY DAP INITIALIZATION PHASE, THE DAP STARTUP SEQUENCE IS
#	ENTERED AT MRCLEAN FOR A FULL INITIALIZATION.
#      *FOR RESTARTS DURING CSM/LM V46 SWITCH-OVER, TVCPHASE IS SET TO -2,
#	AND THE RESTART LOGIC GOES BACK TO REDO SWITCH-OVER (AFTER THE
#	NORMAL DAP RESTART SEQUENCE IS FOLLOWED).
#      *RESTARTS ARE NOT CRITICAL TO THE ROLL DAP PERFORMANCES HENCE THE
#	ROLL DAP IS MERELY RESTARTED.
#      *RESTARTS DURING A STROKE TEST (STROKER IS NON-ZERO) WILL CAUSE THE
# Page 957
#	STROKE TEST TO BE TERMINATED. A NEW V68 ENTRY WILL BE REQUIRED
#	TO GET IT GOING AGAIN (NO AUTOMATIC RESTART).
#      *REDOTVC IS REACHED FOLLOWING ANY RESTART WHICH FINDS THE T5 BITS
#	(BITS 15,14 OF FLAGWRD6) SET FOR TVC. DOTVCON SETS TVCPHASE = -1
#	AND TVC EXPHS = 0 JUST BEFORE SETTING THESE BITS, JUST BEFORE
#	MAKING THE T5 CALL TO TVCDAPON. ON A NORMAL SHUTDOWN DOTVCRCS
#	CALLS RCSDAPON, WHICH RESETS THE T5 BIT FOR RCS.
# CALLING SEQUENCE....T5, IN PARTICULAR BY ELRSKIP OF FRESH START/RESTART
#
# NORMAL EXIT MODES....RESUME, NOQRSM, POSTJUMP (TO TVCINIT4 OR MRCLEAN)
#
# ALARM OR ABORT EXIT MODES....NONE
#
# SUBROUTINES CALLED....
#
#      *PCOPY+1, YCOPY+1 (PITCH AND YAW COPYCYCLES)
#      *ENABLE1,2, CMDSOUT (RE-ESTABLISH ACTUATOR TRIMS)
#      *MRCLEAN OR TVCINIT4 (TVCDAP INITIALIZATIONS)
#      *SWICHOVR +5  (CSM/LM V46 SWITCH-OVER)
#      *EXRSTRT AND TVCEXECUTIVE PHASE POINTS 1 THRU 6
#      *WAITLIST, IBNKCALL, POSTJUMP, ISWCALL
#
# OTHER INTERFACES....DOTVCON AND RCSDAPON (T5 BITS), ELRSKIP (CALLS IT)
# ERASABLE INITIALIZATION REQUIRED....
#
#      *T5 BITS (1,0), TVCPHASE (-2,-1,0,1,2,3), TVCEXPHS (1 THRU 6)
#      *TVC DAP VARIABLES
#      *OPERATIONS PERFORMED BY REDOTVC ARE BASED ON THE ASSUMPTION THAT
#	THE TVC DAPS ARE RUNNING NORMALLY
#
# OUTPUT....
#
#      *PITCH AND YAW TVC DAP COPYCYCLES COMPLETED IF INTERUPTED
#      *TVCEXECUTIVE COMPLETED IF INTERUPTED
#      *STROKE TEST TERMINATED IF INTERRUPTED
#      *CSM/LM V46 SWITCH-OVER REPEATED IF INTERRUPTED
#      *ACTUATOR TRIMS RE-ESTABLISHED (ACTUATORS BACK ON THE AIR)
#      *TVC DAP INITIALIZATION AS REQUIRED
#      *ALL TVC DAP OPERATIONS ON THE AIR
#
# DEBRIS....TVC TEMPORARIES IN EBANK6

# Page 958
;
; ============================================================================
; TRANSITION: From Normal TVC Operation to Restart Recovery
;
; When the AGC restarts unexpectedly (due to power transient, memory error, or
; other anomaly), the TVC system must recover gracefully to avoid losing
; control of the spacecraft. This restart package ensures that engine gimbal
; actuators return to proper trim positions and all control loops resume
; stable operation. During critical SPS burns for lunar orbit insertion,
; transearth injection, or other maneuvers, loss of thrust vector control
; could jeopardize the mission. This code protected the crew during those
; moments when the computer needed to restart itself while engines were firing.
; ============================================================================
;
; TVC RESTART PROTECTION STRATEGY
;
; The restart protection system uses "phase points" to track where TVC
; operations were interrupted. Phase points are simply INCR instructions that
; mark progress through critical copycycles (data copying operations that
; update control system memory). If a restart occurs:
;
; 1. Complete any interrupted copycycle to prevent partial state updates
; 2. Re-enable TVC and optics error counter outputs (Channel 12)
; 3. Command actuators back to proper trim positions (P,YACTOFF values)
; 4. Resume normal TVC operation through standard initialization sequence
;
; This approach allows TVC to survive restarts with minimal transients in
; engine gimbal positioning, critical for maintaining vehicle attitude control
; during propulsive maneuvers.
;
; TVCPHASE values indicate restart location:
;   -2 = Restart during CSM/LM switch-over (V46), redo switch-over
;   -1 = Restart before TVCINIT4, perform full DAP initialization at MRCLEAN
;    0 = Normal operation, no restart recovery needed
;  1,2 = Restart during pitch or yaw copycycle, complete copycycle first
;    3 = Restart during roll DAP, just restart roll control
;
; TVCEXPHS values (1-6) track TVC executive phase for restart point selection.
;

		BANK	16
		SETLOC	DAPROLL
		BANK
		EBANK=	TVCPHASE
		COUNT*	$$/RSRT

; TVC RESTART ENTRY POINT
;
; REDOTVC is called by the restart system when a restart occurs while TVC
; is active (T5 bits 15,14 of FLAGWRD6 indicate TVC mode). This routine
; orchestrates the recovery sequence to restore TVC control.

REDOTVC		LXCH	BANKRUPT	# TVC RESTART PACKAGE
		EXTEND
		QXCH	QRUPT		# (  ..TCR..  IN  ..FINCOPY..  )

; Check if TVC executive needs restart recovery. TVCEXPHS is a phase counter
; incremented at various points in the TVC executive processing cycle. If
; non-zero, the executive was interrupted and must be restarted to complete
; its computations before resuming normal DAP operation.

EXECPHS		CCS	TVCEXPHS	# CHECK TVCEXECUTIVE PHASE
		TCF	+2		#	MUST RESTART TVCEXECUTIVE
		TCF	TVCDAPHS	#	NO NEED TO RESTART TVCEXECUTIVE

; Schedule EXRSTRT to restart the TVC executive after a 9 centisecond delay.
; This timing ensures EXRSTRT executes after CMDSOUT (which sends actuator
; commands) but before PITCHDAP (which computes next control cycle). The delay
; allows the system to stabilize before resuming full TVC computations.

		CAF	NINE		# 9CS DELAY TO FORCE EXRSTRT TO OCCUR
		TC	WAITLIST	#	BEFORE PITCHDAP, AFTER CMDSOUT
		EBANK=	TVCEXPHS
		2CADR	EXRSTRT

; TVC DAP PHASE RESTART RECOVERY
;
; Check TVCPHASE to determine which DAP component was active when restart
; occurred. Different phase values indicate different restart requirements:
; copycycle completion, actuator trim restoration, or switch-over retry.

TVCDAPHS	CS	OCT37776	# CHECK BITS 15 AND 1 OF TVCPHASE TO SEE
		MASK	TVCPHASE	#	DAP RESTART LOCATION (-1,1,2,3)
		CCS	A
		TCF	FINCOPY		#	FINISH THE COPYCYCLE FIRST
		TCF	ENABL1		#	JUST PREPARE THE OUTCOUNTERS AND GO

; Check if restart occurred during CSM/LM switch-over (TVCPHASE = -2).
; If so, the switch-over must be redone after normal restart sequence.
; If TVCPHASE = -1, restart was before TVCINIT4, requiring full initialization.

		CS	TVCPHASE	# TEST FOR TVCPHASE = -2
		MASK	BIT2		#	(THIS INDICATES RESTART OCCURRED
		EXTEND			#	 DURING CSM/LM V46 SWITCH-OVER)
		BZF	TRIM/CMD	# NO. TVCPHASE = -1. RSTRT WAS IN TVCINIT

; ACTUATOR OUTPUT ENABLE SEQUENCE (ENABL1 → ENABL2 → CMDSOUT)
;
; After a restart, TVC and optics error counter outputs (Channel 12) must be
; re-enabled with proper timing to avoid transients. The sequence is:
;
; ENABL1: Enable TVC and optics DAC, wait 40ms minimum
; ENABL2: Enable optics error counter, wait 4ms minimum  
; CMDSOUT: Send actuator trim commands, wait 20ms
;
; These delays allow the analog actuator electronics to stabilize before
; receiving control commands, preventing violent gimbal movements that could
; destabilize the spacecraft during critical engine burns.

ENABL1		CAF	BIT8		# TVC ENABLE, FOLLOWED BY 40 MS (MIN) WAIT
		AD	BIT11		# SET BIT FOR OPTICS-DAC-ENABLE ALSO
		EXTEND			# (ENABL1 ENTERED FROM TVCDAPHS / FINCOPY)
		WOR	CHAN12
		CAF	TVCADDR		# WAIT.  CALLING ENABL2  (BBCON THERE)
		TS	T5LOC
		CAF	TVCADDR +4	#	60 MS (TVCEXADR)
		TS	TIME5

		TCF	RESUME

ENABL2		LXCH	BANKRUPT	# CONTINUE PREPARATION OF OUTCOUNTERS

		CAF	BIT2		# OPTICS ERROR CNTR ENABLE. 4MS MIN WAIT
		EXTEND
		WOR	CHAN12
# Page 959
		CAF	TVCADDR +2	# WAIT, CALLING CMDSOUT (BBCON THERE)
		TS	T5LOC
		CAF	OCT37776	#	20MS
		TS	TIME5

		TCF	NOQRSM

; ACTUATOR COMMAND OUTPUT (CMDSOUT)
;
; This routine sends the most recent pitch and yaw actuator commands to the
; engine gimbal actuators. After a restart, the actuators must return to their
; proper positions as commanded by PCMD and YCMD (trim offsets computed by TVC).
;
; The commands are copied to output registers TVCPITCH and TVCYAW, then the
; DAC release bits (11,12 of Channel 14) enable the digital-to-analog converters
; that drive the actuator servos. This completes the restart recovery sequence,
; allowing normal TVC control to resume.

CMDSOUT		LXCH	BANKRUPT	# CONTINUE PREPARATION OF OUTCOUNTERS
		EXTEND
		QXCH	QRUPT

		CS	ZERO		# MOST RECENT ACTUATOR COMMANDS
		AD	PCMD		#	(AVOID +0)
		TS	TVCPITCH
		CS	ZERO
		AD	YCMD
		TS	TVCYAW

		CAF	PRIO6		# RELEASE THE COUNTERS (BITS 11,12)
		EXTEND
		WOR	CHAN14

; RESTART COMPLETION LOGIC (PHSCHK2)
;
; Now that actuators are commanded and outputs are enabled, determine where
; to resume TVC operation based on TVCPHASE value:
;
; TVCPHASE = -2: Restart during CSM/LM switch-over → redo SWICHOVR
; TVCPHASE = -1: Restart before TVCINIT4 → full initialization at MRCLEAN
; TVCPHASE ≥ 0: Restart during normal operation → resume at TVCINIT4
;
; Additionally, if a stroke test was in progress (STROKER non-zero), it must
; be terminated because restart interrupts test sequencing. A new V68 verb
; entry will be required to restart the stroke test.

PHSCHK2		CCS	TVCPHASE	# CHECK TVCPHASE AGAIN
		TCF	CHKSTRK
		TCF	CHKSTRK
		CCS	A		# A CONTAINS THE DIMINISHED ABSOLUTE OF
		TC	+3		# TVCPHASE (-2 BECOMES +1, -1 BECOMES +0)

		TC	POSTJUMP	#	REPEAT TVC INITIALIZATION
		CADR	MRCLEAN		#	(DO NOT RETURN)

	+3	TC	IBNKCALL	#	REPEAT CSM/LM V46 SWITCH-OVER
		CADR	SWICHOVR +5	#	(RETURN TO CHECK FOR STROKE TEST)

CHKSTRK		CCS	STROKER		# CHECK FOR STROKE TEST IN PROGRESS
		TCF	TSTINITJ	# YES, KILL IT
		TCF	+2		# NO, PROCEED
		TCF	TSTINITJ	# YES, KILL IT

	+4	TC	POSTJUMP	#	IF POSITIVE OR ZERO, RESTART AT
		CADR	TVCINIT4	#		TVCINIT4 (ZEROS TVCPHASE, AND
					#		CALLS TVC DAPS VIA DAPINIT)

; COPYCYCLE COMPLETION (FINCOPY)
;
; This routine completes any copycycle that was interrupted by restart.
; Copycycles atomically transfer computed TVC parameters from temporary
; registers to their final memory locations. TVCPHASE serves as an index
; pointing to the appropriate restart resume address.
;
; The copycycle must complete before normal TVC operation resumes, ensuring
; filter states, integrators, and differentiators are properly updated.
; Otherwise, wild transients could result from stale or inconsistent data.

FINCOPY		INDEX	TVCPHASE	# PICK UP THE APPROPRIATE COPYCYCLE
		CAF	TVCCADR
		TCR	ISWCALL		# RE-ENTER THE COPYCYCLE, RETURN AT END
		TCF	ENABL1		# NOW PREPARE THE OUTCOUNTERS

; TRIM COMMAND INITIALIZATION (TRIM/CMD)
;
; If restart occurred before TVCDAPON completed initialization, the pitch and
; yaw command values (PCMD, YCMD) may not yet be set. This section loads them
; from the pitch/yaw actuator offset values (PACTOFF, YACTOFF), which represent
; the mechanical trim position of the engine gimbals.
;
; After setting these trim values, the routine proceeds to ENABL1 to re-enable
; actuator outputs and send the trim commands to the engine gimbals.

TRIM/CMD	EXTEND			# TVCDAPON INITIALIZATION NOT COMPLETED,
# Page 960
		DCA	PACTOFF		#	EG.  P,YCMD MAY NOT BE SET.  SET...
		DXCH	PCMD
		TCF	ENABL1		# NOW PREPARE THE OUTCOUNTERS

; STROKE TEST TERMINATION (TSTINITJ)
;
; If a restart occurs during an engine gimbal stroke test (V68 verb), the test
; must be terminated because the restart interrupts the precise timing sequences
; required for actuator testing. STROKER is set to +0 to indicate termination.
;
; The stroke test cannot be automatically restarted; crew must enter a new V68
; verb to initiate another test after the restart recovery is complete.

TSTINITJ	CAF	ZERO		# DISABLE STROKE TEST (-0 SHOWS PRIOR V68)
		TS	STROKER		# (+0 MEANS NEW V68 REQUIRED FOR STARTUP)

		TCF	CHKSTRK +4

; TVCEXECUTIVE RESTART ENTRY (EXRSTRT)
;
; This is the restart entry point for the TVC Executive scheduler. TVCEXPHS
; contains a phase pointer indicating which copycycle was active when restart
; occurred. The indexed address table (TVCEXADR) contains resume addresses for
; each possible restart phase.
;
; After the copycycle completes, normal TVCEXECUTIVE operation resumes through
; the standard TVCINIT4 initialization sequence.

EXRSTRT		INDEX	TVCEXPHS	# TVCEXECUTIVE RESTARTS....GO TO
		CAF	TVCEXADR	#	APPROPRIATE RESTART POINT
		INDEX	A
		TCF	0

; ============================================================================
; TVC RESTART ADDRESS TABLES
;
; These tables provide indexed restart resume addresses for TVC copycycles.
; The order of entries is critical - each index value of TVCPHASE or TVCEXPHS
; must correspond to the correct resume address.
;
; TVCCADR Table (indexed by TVCPHASE):
;   Index 0: ENABL2 (unused, for T5 call compatibility)
;   Index 1: PCOPY+1 (resume point for pitch copycycle)
;   Index 2: CMDSOUT (unused, for T5 call compatibility)
;   Index 3: YCOPY+1 (resume point for yaw copycycle)
;
; TVCEXADR Table (indexed by TVCEXPHS):
;   Index 0: Unused (60ms T5 filler)
;   Index 1-6: TVCEXECUTIVE copycycle resume points in execution order
;
; During restart, the phase pointer selects the appropriate address, ensuring
; the copycycle completes from where it was interrupted. This protects filter
; memory, integrators, differentiators, and other state variables from wild
; transients that could destabilize spacecraft attitude during engine burns.
; ============================================================================

# TVC RESTART TABLES.... ORDER IS REQUIRED.   HI-ORDER WORDS ONLY, OF 2CADRS, SINCE BBCON IS ALREADY THERE.

TVCADDR		=	TVCCADR		# TABLE OF CADRS, UNUSED LOCS FOR GENADRS
TVCCADR		GENADR	ENABL2		# (FOR T5 CALL, UNUSED TABLE LOC)
	+1	CADR	PCOPY +1	# PITCH COPYCYCLE
	+2	GENADR	CMDSOUT		# (FOR T5 CALL, UNUSED TABLE LOC)
	+3	CADR	YCOPY +1	# YAW COPYCYCLE
TVCEXADR	OCT	37772		# (UNUSED TABLE LOC, FILL WITH 60MS, T5)
	+1	GENADR	EXECCOPY +1	# TVCEXECUTIVE RESTART POINTS (ORDERED)
	+2	GENADR	1SHOTCHK
	+3	GENADR	TEMPSET
	+4	GENADR	CORSETUP
	+5	GENADR	CORCOPY +1
	+6	GENADR	CNTRCOPY


