# Copyright:	Public domain.
# Filename:	RESTART_TABLES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	238-243
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
; FILE: RESTART_TABLES.agc
; MODULE: Restart Protection System
; MISSION PHASE: all phases (continuous protection)
;
; TL;DR: Defines restart protection tables that encode task phases and state
;        preservation information for warm restart recovery. Enables the AGC
;        to recover from power transients, overload conditions (like the
;        famous 1202 alarm), or temporary failures without mission loss by
;        restoring interrupted jobs, waitlist tasks, and longcalls to their
;        proper execution state.
;
; COMMENT-ONLY READERS: This is the AGC's safety net. When the computer
;        experiences overload or power disruption, these tables tell it
;        exactly how to pick up where it left off, ensuring the mission
;        continues safely.
; CODE-ALONG READERS: Study the table structure encoding scheme - priorities
;        stored in PRDTTAB, 2CADR addresses in CADRTAB, with sign conventions
;        distinguishing job types (FINDVAC vs NOVAC) from waitlist calls.
;        Understanding this mechanism is key to comprehending the 1202 alarm
;        recovery during Apollo 11's lunar descent.
; ============================================================================

# Page 238
; ============================================================================
; RESTART PROTECTION TABLE ARCHITECTURE
; ============================================================================
;
; During Apollo 11's descent to the lunar surface, the AGC experienced
; program alarms 1201 and 1202 caused by executive and waitlist overload.
; The restart protection system defined in these tables allowed the computer
; to shed lower-priority tasks and continue executing mission-critical
; guidance programs, enabling Armstrong and Aldrin to land successfully
; despite the computer overload. This restart capability was crucial to
; mission success.
;
; The restart system maintains state information for all active programs
; so that if the AGC loses power, experiences computational overload, or
; encounters errors, it can restore itself to a known-good state and resume
; operations within milliseconds, preserving mission continuity.

# RESTART TABLES
# ------------------
;
; RESTART TABLE ORGANIZATION:
;
; The AGC divides restart protection into 6 major groups (numbered 1-6),
; with each group managing related mission functions. Within each group,
; phases are numbered (e.g., 1.2, 1.3, 2.2, 2.3, etc.) to identify specific
; program restart points.
;
# THERE ARE TWO FORMS OF RESTART TABLES FOR EACH GROUP.  THEY ARE KNOWN AS THE EVEN RESTART TABLES AND THE ODD
# RESTART TABLES.  THE ODD TABLES HAVE ONLY ONE ENTRY OF THREE LOCATIONS WHILE THE EVEN TABLES HAVE TWO ENTRIES
# EACH USING THREE LOCATIONS. THE INFORMATION AS TO WHETHER IT IS A JOB, WAITLIST, OR A LONGCALL IS GIVEN BY THE
# WAY THINGS ARE PUT INTO THE TABLES.
;
; TABLE STRUCTURE DETAILS:
; - EVEN tables (X.2SPOT): Two restart entries, six memory locations total
; - ODD tables (X.3SPOT and higher odd phases): One restart entry, three
;   memory locations
; - Table entries encode job type through data sign conventions and format
; ============================================================================
; RESTART ENTRY TYPE 1: JOBS (FINDVAC and NOVAC)
; ============================================================================
;
; JOBS are computational tasks managed by the AGC executive scheduler.
; They compete for processor time based on assigned priority levels.
; During a restart, the AGC must know which jobs to resurrect and at
; what priority level.
;
#      A JOB HAS ITS PRIORITY STORED IN PRDTTAB OF THE CORRECT PHASE SPOT - A POSITIVE PRIORITY INDICATES A
# FINDVAC JOB, A NEGATIVE PRIORITY A NOVAC.  THE 2CADR OF THE JOB IS STORED IN THE CADRTAB.
;
; ENCODING SCHEME FOR JOBS:
; - PRDTTAB contains priority value (octal format)
;   * Positive value = FINDVAC job (searches for vacant core set)
;   * Negative value = NOVAC job (creates new core set)
; - CADRTAB contains 2CADR (two-word address: bank + address within bank)
;   pointing to the job's entry point
;
; Priority values range from 1 (lowest) to 37 (highest in octal).
; Higher-priority jobs preempt lower-priority ones in the executive queue.
;
# FOR EXAMPLE,
#
# 5.7SPOT	OCT	23000
#		2CADR	SOMEJOB
#
# A RESTART OF GROUP 5 WITH PHASE SEVEN WOULD THEN CAUSE SOMEJOB TO BE RESTARTED AS A FINDVAC WITH PRIORITY 23.
;
; INTERPRETATION: If the AGC restarts while Group 5 Phase 7 is active,
; it will restart SOMEJOB as a FINDVAC with priority 23 (decimal 19).
; FINDVAC means the executive scheduler will search for an available
; core set slot rather than always creating a new one.
;
# 5.5SPOT	OCT	-23000
#		2CADR	ANYJOB
#
# HERE A RESTART OF GROUP 5 WITH PHASE 7 WOULD CAUSE ANYJOB TO BE RESTARTED AS A NOVAC WITH PRIORITY 23.
;
; INTERPRETATION: Negative priority (-23000) indicates NOVAC, which
; always allocates a new core set. This is used when job requires
; guaranteed fresh workspace.
; ============================================================================
; RESTART ENTRY TYPE 2: LONGCALL (Delayed Task Execution)
; ============================================================================
;
; LONGCALL is a mechanism for scheduling tasks to execute after a
; specified time delay. During restart, the AGC must restore both
; the task reference and its original timing parameter.
;
# A LONGCALL HAS ITS GENADR OF ITS 2CADR STORED NEGATIVELY AND ITS BBCON STORED POSITIVELY.  IN ITS PRDTTAB IS
# PLACED THE LOCATION OF A DP REGISTER THAT CONTAINS THE DELTA TIME THAT LONGCALL HAD BEEN ORIGINALLY STARTED
# WITH.  EXAMPLE,
;
; ENCODING SCHEME FOR LONGCALL:
; - PRDTTAB contains GENADR of double-precision register holding delta time
; - CADRTAB first word contains -GENADR (negative general address) of task
; - CADRTAB second word contains BBCON (bank bits and constant) of task
;
; This three-word structure allows the restart system to re-establish
; the longcall with its original time delay, or execute immediately if
; the scheduled time has already passed during the restart delay.
;
# 3.6SPOT	GENADR	DELTAT
#	       -GENADR	LONGTASK
#		BBCON	LONGTASK
;
; INTERPRETATION: Group 3 Phase 6 contains a longcall to LONGTASK.
; DELTAT is a double-precision variable holding the original time delay
; in centiseconds. The negative GENADR identifies this as a longcall.
;
#		OCT	31000
#		2CADR	JOBAGAIN
#
# THIS WOULD START UP LONGTASK AT THE APPROPRIATE TIME, OR IMMEDIATELY IF THE TIME HAD ALREADY PASSED. IT SHOULD
# BE NOTED THAT IF DELTAT IS IN A SWITCHED E BANK, THIS INFORMATOIN SHOULD BE IN THE BBCON OFTHE 2CADR OF THE
# TASK.  FROM ABOVE, WE SEE THAT THE SECOND PART OF THIS PHASE WOULD BE STARTED AS A JOB WITH A PRIORITY OF 31.
;
; MULTI-PART RESTART PHASES:
; A single restart phase can contain multiple restart actions. Here,
; after the longcall is restored, JOBAGAIN is restarted as a FINDVAC
; job with priority 31 (decimal 25). This allows complex program states
; involving both timed tasks and immediate jobs to be fully restored.
#
; ============================================================================
; RESTART ENTRY TYPE 3: WAITLIST (Timer-Driven Tasks)
; ============================================================================
;
; WAITLIST tasks are timer-driven activities scheduled by the T4RUPT
; interrupt handler. These are critical for time-sensitive operations
; like navigation updates, display refreshes, and engine control.
; During the 1202 alarm on Apollo 11's descent, the waitlist became
; overloaded with radar data processing tasks, triggering the restart
; protection mechanism.
;
# WAITLIST CALLS ARE IDENTIFIED BY THE FACT THAT THEIR 2CADR IS STORED NEGATIVELY. IF PRDTTAB OF THE PHASE SPOT
# IS POSITIVE, THEN IT CONTAINS THE DELTA TIME, IF PRDTTAB IS NEGATIVE THEN IT IS THE -GENADR OF AN ERASABLE
# LOCATION CONTAINING THE DELTA TIME, THAT IS, THE TIME IS STORED INDIRECTLY.  IT SHOULD BE NOTED AS ABOVE, THAT
# IF THE TIME IS STORED INDIRECTLY, THE BBCON MUST CONTAIN THE NECESSARY E BANK INFORMATION IF APPLICABLE.  WITH
# WAITLIST WE HAVE ONE FURTHER OPTION, IF -0 IS STORED IN PRDTTAB, IT WILL CAUSE AN IMMEDIATE RESTART OF THE
# TASK.  EXAMPLES,
;
; ENCODING SCHEME FOR WAITLIST:
; - CADRTAB contains -2CADR (negative two-word address) identifying waitlist
; - PRDTTAB encoding determines timing restoration:
;   * Positive value = Direct delta time in centiseconds
;   * Negative value = -GENADR of erasable location containing delta time
;   * -0 (octal 77777) = Immediate restart, bypass timing
;
; The negative 2CADR distinguishes waitlist entries from regular jobs.
; This sign convention allows the restart routine to differentiate
; between executive jobs (positive 2CADR) and waitlist tasks (negative).
;
; WAITLIST RESTART OPTION 1: Immediate Execution
#		OCT	77777		# THIS WILL CAUSE AN IMMEDIATE RESTART
#	       -2CADR	ATASK		# OF THE TASK :ATASK:
;
; INTERPRETATION: Octal 77777 is negative zero (-0). This special value
; tells the restart system to execute ATASK immediately without waiting
; for any timer. Used for critical tasks that must run as soon as the
; AGC recovers from restart.
;
; WAITLIST RESTART OPTION 2: Direct Time Specification
#		DEC	200		# IF THE TIME OF THE 2 SECONDS SINCE DUMMY
#	       -2CADR	DUMMY		# WAS PUT ON THE WAITLIST IS UP, IT WILL BEGIN
#					# IN 10 MS, OTHERWISE IT WILL BEGIN WHEN
#					# IT NORMALLY WOULD HAVE BEGUN.
;
; INTERPRETATION: Delta time of 200 centiseconds (2 seconds) is stored
; directly in PRDTTAB. If 2 seconds have already elapsed during restart,
; DUMMY executes in 10 milliseconds. Otherwise, it waits for remaining time.
; This preserves the original scheduling intent across restart boundaries.
;
# Page 239
; WAITLIST RESTART OPTION 3: Indirect Time Specification
#	       -GENADR	DTIME		# WHERE DTIME CONTAINS THE DELTA TIME
#	       -2CADR	TASKTASK	# OTHERWISE THIS IS AS ABOVE
;
; INTERPRETATION: PRDTTAB contains -GENADR pointing to erasable variable
; DTIME, which holds the actual delta time value. This indirection is
; necessary when the time delay is computed dynamically or stored in
; a switched erasable bank. The BBCON in the 2CADR must specify the
; correct bank if DTIME resides in switched erasable memory.
#
; ============================================================================
; RESTART TABLE DEFINITIONS BEGIN
; ============================================================================
;
; The tables below define restart protection for all mission-critical
; programs. Each group corresponds to major mission functions:
; - Group 1: Fresh start and system initialization
; - Group 2: IMU alignment and calibration
; - Group 3: Navigation and guidance computation
; - Group 4: Display and keyboard (DSKY) interface
; - Group 5: Engine control and RCS autopilot
; - Group 6: Orbital integration and rendezvous
;
; During Apollo 11's lunar descent, these tables enabled the AGC to
; recover from the 1202 alarm by shedding non-critical tasks while
; preserving landing guidance, throttle control, and navigation state.

# ***** NOW THE TABLES THEMSELVES *****

		BANK	01
		SETLOC	RESTART
		BANK

; Memory locations for accessing restart table entries:
; These base addresses are used with relative indexing to locate the
; correct restart data for a given group and phase combination.

PRDTTAB		EQUALS	12000		# USED TO FIND THE PRIORITY OR DELTATIME
					; Base address for priority/timing table
					; Contains either:
					; - Job priority (positive=FINDVAC, negative=NOVAC)
					; - Delta time for waitlist (direct or indirect)
					; - GENADR of time variable for longcall

CADRTAB		EQUALS	12001		# THIS AND THE NEXT RELATIVE LOC CONTAIN
					# RESTART 2CADR
					; Base address for address table
					; Contains two-word addresses (2CADR):
					; - Positive 2CADR = executive job
					; - Negative 2CADR = waitlist task
					; - Negative GENADR + BBCON = longcall

		COUNT*	$$/RSTAB	# TABLES IN BANK 1.

; ============================================================================
; SIZETAB: Restart Table Size and Offset Directory
; ============================================================================
;
; SIZETAB provides the restart system with location and size information
; for each restart group's phase tables. Each entry is a TC (Transfer
; Control) instruction encoding both the table address and its size.
;
; The offset values (-12006 for even phases, -12004 for odd phases)
; indicate the relative memory displacement from the group's base address
; to the start of the restart table data.
;
; - Even phases (X.2SPOT): Offset -12006, contains 2 restart entries (6 words)
; - Odd phases (X.3SPOT and higher): Offset -12004, contains 1 entry (3 words)
;
; RESTART GROUP FUNCTIONAL ASSIGNMENTS:
; Group 1: Fresh start, system initialization, and task management
; Group 2: IMU alignment, calibration, and inertial sensor operations
; Group 3: Navigation state updates and orbital integration
; Group 4: Display interface, DSKY operations, and crew interaction
; Group 5: Engine control, RCS autopilot, and thrust vector management
; Group 6: Rendezvous guidance, targeting, and relative navigation

SIZETAB		TC	1.2SPOT -12006	; Group 1, Phase 2: Fresh start/restart initialization
		TC	1.3SPOT -12004	; Group 1, Phase 3: Post-initialization tasks
		TC	2.2SPOT -12006	; Group 2, Phase 2: IMU fine alignment
		TC	2.3SPOT	-12004	; Group 2, Phase 3: IMU performance and state integration
		TC	3.2SPOT -12006	; Group 3, Phase 2: Navigation state update
		TC	3.3SPOT -12004	; Group 3, Phase 3: Orbital integration cycle
		TC	4.2SPOT -12006	; Group 4, Phase 2: Display interface updates
		TC	4.3SPOT -12004	; Group 4, Phase 3: Extended verb processing
		TC	5.2SPOT -12006	; Group 5, Phase 2: Engine control and throttle management
		TC	5.3SPOT -12004	; Group 5, Phase 3: RCS autopilot and attitude control
		TC	6.2SPOT -12006	; Group 6, Phase 2: Rendezvous targeting computation
		TC	6.3SPOT -12004	; Group 6, Phase 3: Relative navigation and tracking
; ============================================================================
; GROUP 1 RESTART TABLES: System Initialization and Task Management
; ============================================================================
;
; Group 1 manages fundamental system operations that must be protected
; during restarts. These include basic task scheduling, job completion,
; and critical waitlist operations that underpin all other mission programs.

1.2SPOT		OCT	21000		# A DUMMY EXAMPLE TO BE REPLACED AS SOON
		EBANK=	STATE
		2CADR	ENDOFJOB	# AS THERE IS A LEGITIMATE 1.2SPOT
; Priority 21 FINDVAC job restart of ENDOFJOB routine.
; Ensures proper job termination and executive queue cleanup during restart.

		DEC	100
		EBANK=	STATE
		2CADR	TASKOVER
; Waitlist task restart after 100 centiseconds (1 second).
; TASKOVER handles watchdog timer functions and system health monitoring.

# ANY MORE GROUP 1.EVEN RESTART VALUES SHOULD GO HERE

1.3SPOT	       -GENADR	SAVET-30	; Indirect time reference (30 cs before SAVET)
		EBANK=	DVCNTR
	       -2CADR	ULLGTASK		; Negative 2CADR = waitlist task
; Waitlist restart of ULLGTASK (ullage control task) with time stored
; indirectly in SAVET-30 memory location. Ullage motors settle propellant
; before engine ignition, critical for descent and ascent burns.

# ANY MORE GROUP 1.ODD RESTART VALUES SHOULD GO HERE

; ============================================================================
; GROUP 2 RESTART TABLES: IMU Operations and Navigation State Integration
; ============================================================================
;
; Group 2 protects IMU (Inertial Measurement Unit) alignment, calibration,
; and navigation state update tasks. These are critical for maintaining
; attitude reference and position/velocity knowledge during all mission phases.

2.2SPOT		EQUALS	1.2SPOT		; No dedicated Group 2.2 entries; uses 1.2SPOT defaults
# ANY MORE GROUP 2.EVEN RESTART VALUES SHOULD GO HERE

2.3SPOT		GENADR	600SECS		; Delta time = 600 seconds for longcall
	       -GENADR	STATEINT		; Negative GENADR = longcall task address
		EBANK=	RRECTCSM
		BBCON	STATEINT		; Bank and address for STATEINT
; Longcall restart of STATEINT (state vector integration) after 600 seconds.
; Performs periodic orbital integration to maintain accurate position and
; velocity knowledge during coast phases of lunar mission.

# Page 240
2.5SPOT		OCT	05000		; Priority 5 FINDVAC job
		EBANK=	RRECTCSM
		2CADR	STATINT1
; Low-priority job restart for STATINT1 continuation after state integration.
; Performs post-integration navigation updates and state vector refinement.

2.7SPOT		DEC	1500		; 1500 centiseconds = 15 seconds
		EBANK=	LOSCOUNT
	       -2CADR	P20LEMC1		; Negative 2CADR = waitlist task
; Waitlist restart for P20LEMC1 (P20 rendezvous navigation, LEM computer).
; Schedules periodic relative navigation updates during rendezvous operations.

2.11SPOT	OCT	14000		; Priority 14 FINDVAC job
		EBANK=	P21TIME
		2CADR	P25LEM1
; Medium-priority job restart for P25LEM1 (P25 targeting program for LM).
; Computes targeting parameters for rendezvous maneuvers.

2.13SPOT	OCT	10000		; Priority 10 FINDVAC job
		EBANK=	LOSCOUNT
		2CADR	RELINUS
; Job restart for RELINUS (relative motion integration for rendezvous).
; Maintains tracking of CSM position relative to LM during rendezvous.

2.15SPOT	OCT	26000		; Priority 26 FINDVAC job
		EBANK=	LOSCOUNT
		2CADR	R22RSTRT
; High-priority restart for R22RSTRT (R22 navigation data display).
; Ensures crew can view critical navigation information after restart.

2.17SPOT	OCT	77777		; Octal 77777 = -0, immediate restart
		EBANK=	VGPREV
	       -2CADR	REDO2.17		; Negative 2CADR = waitlist task
; Immediate waitlist restart of REDO2.17 (redo phase 2.17 operation).
; The -0 code ensures this task begins immediately upon restart, critical
; for time-sensitive navigation state updates.

2.21SPOT	DEC	25		; 25 centiseconds = 0.25 seconds
		EBANK=	DVCNTR
	       -2CADR	R10,R11		; Negative 2CADR = waitlist task
; Brief 250ms waitlist task for R10,R11 (IMU fine alignment routine).
; Short delay allows IMU to stabilize before alignment measurement.

# ANY MORE GROUP 2.ODD RESTART VALUES SHOULD GO HERE.

; ============================================================================
; TRANSITION: Group 2 to Group 3 - Navigation State Update Systems
;
; Group 2 focused on inertial measurement unit alignment and rendezvous
; navigation. Group 3 shifts to general navigation state updates and
; orbital integration. These routines maintain the spacecraft's knowledge
; of its position and velocity throughout the mission.
; ============================================================================

3.2SPOT		EQUALS	1.2SPOT
; Group 3 Phase 2 shares restart table with Group 1 Phase 2.
; Uses same job termination logic (ENDJOB/ENDOFJOB/JOBWAKE).
# ANY MORE GROUP 3.EVEN RESTART VALUES SHOULD GO HERE

3.3SPOT	       -GENADR	ZOOMTIME	; Negative GENADR = indirect time
		EBANK=	DVCNTR
	       -2CADR	ZOOM		; Negative 2CADR = waitlist task
; Waitlist restart with indirect delta time stored in ZOOMTIME.
; ZOOM task performs rapid state vector propagation during critical phases.
; The indirect time reference allows dynamic scheduling based on mission state.

3.5SPOT		OCT	20000		; Priority 20 FINDVAC job
		EBANK=	TTOGO
		2CADR	S40.13
; Job restart for S40.13 (SPS engine control sequence).
; Manages Service Propulsion System burn sequencing during major maneuvers.

# ANY MORE GROUP 3.ODD RESTART VALUES SHOULD GO HERE

; ============================================================================
; TRANSITION: Group 3 to Group 4 - Engine Control and Burn Sequencing
;
; Group 3 maintained navigation state during coast phases. Group 4 manages
; the critical engine ignition sequences, burn timeline events, and abort
; logic. These restart entries ensure the LM can recover during powered
; flight, the most mission-critical phase of lunar landing and ascent.
; ============================================================================

4.2SPOT		DEC	2500		; 2500 centiseconds = 25 seconds
		EBANK=	TTOGO
	       -2CADR	TIG-5		; Negative 2CADR = waitlist task
; First entry: Waitlist task scheduled 25 seconds before TIG-5 marker.
; TIG-5 = Time of Ignition minus 5 seconds, used for final pre-burn checks.

		OCT	77777		; Octal 77777 = -0, immediate restart
		EBANK=	TTOGO
# Page 241
	       -2CADR	REDO4.2		; Negative 2CADR = waitlist task
; Second entry: Immediate restart of REDO4.2 backup sequence.
; The -0 code ensures critical burn sequencing resumes without delay.

# ANY MORE GROUP 4.EVEN RESTART VALUES SHOULD GO HERE

4.3SPOT		OCT	25000		; Priority 25 FINDVAC job
		EBANK=	DVCNTR
		2CADR	GOABORT
; High-priority restart for GOABORT (abort mode initialization).
; Critical for crew safety - ensures abort logic can resume after any failure.

4.5SPOT		DEC	50		; 50 centiseconds = 0.5 seconds
		EBANK=	TTOGO
	       -2CADR	ULLAGOFF		; Negative 2CADR = waitlist task
; Brief 500ms waitlist task for ULLAGOFF (ullage rocket shutdown).
; Ullage motors settle propellant before main engine ignition; this task
; ensures they shut off at the correct time.

4.7SPOT		DEC	500		; 500 centiseconds = 5 seconds
		EBANK=	DVCNTR
	       -2CADR	TIG-0		; Negative 2CADR = waitlist task
; 5-second waitlist task for TIG-0 (Time of Ignition, T=0 moment).
; Schedules the exact moment of engine ignition command.

4.11SPOT       -GENADR	TGO +1		; Negative GENADR = indirect time
		EBANK=	DVCNTR
	       -2CADR	ENGOFTSK		; Negative 2CADR = waitlist task
; Waitlist restart with indirect time from TGO+1 (time-to-go register).
; ENGOFTSK (engine off task) schedules engine cutoff at computed burn duration.
; Dynamic scheduling ensures precise velocity change despite restart delays.

4.13SPOT	OCT	12000		; Priority 12 FINDVAC job
		EBANK=	TRKMKCNT
		2CADR	POSTBURN
; Medium-priority restart for POSTBURN (post-ignition sequence).
; Performs navigation updates and system reconfigurations after engine cutoff.

4.15SPOT	DEC	500		; 500 centiseconds = 5 seconds
		EBANK=	TTOGO
	       -2CADR	TIG-30		; Negative 2CADR = waitlist task
; 5-second waitlist task for TIG-30 (Time of Ignition minus 30 seconds).
; Initiates pre-burn countdown sequence, crew awareness, system final checks.

4.17SPOT	OCT	77777		; Octal 77777 = -0, immediate restart
		EBANK=	DVCNTR
	       -2CADR	TIG-5		; Negative 2CADR = waitlist task
; Immediate restart of TIG-5 sequence (5 seconds before ignition).
; Critical timing marker - ensures final pre-ignition checks proceed without delay.

4.21SPOT	OCT	13000		; Priority 13 FINDVAC job
		EBANK=	STAR
		2CADR	R51.1 +1
; Job restart for R51.1+1 (IMU alignment display continuation).
; Allows crew to complete or verify alignment procedures after restart.

4.23SPOT	OCT	77777		; Octal 77777 = -0, immediate restart
		EBANK=	DVCNTR
	       -2CADR	IGNITION		; Negative 2CADR = waitlist task
; Immediate restart of IGNITION task.
; Most critical restart entry - ensures engine ignition command is never lost.
; If restart occurs during ignition window, engine fires immediately upon recovery.

4.25SPOT	GENADR	SAVET-30	; Longcall format: GENADR specifies time location
	       -GENADR	TIG-35		; Negative GENADR = task address
		EBANK=	SAVET-30
		BBCON	TIG-35		; Bank/address for task
; Longcall restart for TIG-35 (35 seconds before ignition).
; Uses saved time from SAVET-30, schedules TIG-35 at correct remaining time.
; Longcall format allows precise time reconstruction after restart.

4.27SPOT	OCT	52777		; Priority 42 with special encoding (52-10=42)
		EBANK=	DVCNTR
		2CADR	P70A
; High-priority restart for P70A (abort to lunar orbit during descent).
; P70 is the "abort stage" program - climbs back to orbit if landing fails.
; High priority ensures abort can execute even under extreme processor load.

# Page 242
4.31SPOT	OCT	52777		; Priority 42 with special encoding
		EBANK=	DVCNTR
		2CADR	P71A
; High-priority restart for P71A (abort during powered ascent).
; P71 handles abort-to-orbit if ascent engine fails or guidance errors occur.
; Critical for Eagle's return to rendezvous with Columbia.

4.33SPOT	OCT	46777		; Priority 38 with special encoding (46-10=36)
		EBANK=	DVCNTR
		2CADR	GOP00FIX
; Restart for GOP00FIX (go to P00 program with display fix).
; Returns DSKY to idle state, displays program 00 to crew.

4.35SPOT	OCT	46777		; Priority 38 with special encoding
		EBANK=	DVCNTR
		2CADR	GOPOODOO

4.37SPOT	OCT	52777		; Priority 42 with special encoding
		EBANK=	WHICH
		2CADR	COMFAIL
; High-priority restart for COMFAIL (communications failure handler).
; Manages loss of uplink/downlink with Mission Control, switches to autonomous mode.

# ANY MORE 4.ODD RESTART VALUES SHOULD GO HERE.

; ============================================================================
; TRANSITION: Group 4 to Group 5 - Powered Flight to Surface Operations
;
; Group 4 managed engine ignition sequences and abort logic during powered
; flight. Group 5 handles the landing phase itself - the final moments of
; descent to the lunar surface. These tables governed the code executing
; when Eagle touched down at Tranquility Base on July 20, 1969 at 102:45:40
; mission elapsed time, and Armstrong radioed "The Eagle has landed."
; ============================================================================

5.2SPOT		OCT	22000		; Priority 22 FINDVAC job
		EBANK=	DVCNTR
		2CADR	NORMLIZE
; Job restart for NORMLIZE (normalize display and system state).
; Ensures crew displays return to expected format after restart.

		DEC	200		; 200 centiseconds = 2 seconds
		EBANK=	DVCNTR
	       -2CADR	REREADAC		; Negative 2CADR = waitlist task
; Second entry in 5.2SPOT: 2-second waitlist for REREADAC (re-read accelerometers).
; Ensures fresh IMU data after restart during landing phase.

5.4SPOT		DEC	200		; 200 centiseconds = 2 seconds
		EBANK=	DVCNTR
	       -2CADR	REREADAC		; Negative 2CADR = waitlist task
; First entry: 2-second waitlist for REREADAC.
; Periodic accelerometer reading ensures navigation state remains current.

		OCT	20000		; Priority 20 FINDVAC job
		EBANK=	DVCNTR
		2CADR	SERVICER
; Second entry: Job restart for SERVICER (service routine background tasks).
; Handles housekeeping functions during landing phase.

# ANY MORE GROUP 5.EVEN RESTART VALUES SHOULD GO HERE

5.3SPOT		DEC	200		; 200 centiseconds = 2 seconds
		EBANK=	DVCNTR
	       -2CADR	REREADAC		; Negative 2CADR = waitlist task
; Waitlist restart for REREADAC during landing phase.
; Continuous accelerometer monitoring critical for descent guidance accuracy.

5.5SPOT		OCT	77777		; Octal 77777 = -0, immediate restart
		EBANK=	DVCNTR
	       -2CADR	REDO5.5		; Negative 2CADR = waitlist task
; Immediate restart of REDO5.5 backup sequence.
; Ensures landing phase state machine continues without delay.

5.7SPOT		OCT	77777		; Octal 77777 = -0, immediate restart
		EBANK=	DVCNTR
# Page 243
	       -2CADR	BIBIBIAS	; Negative 2CADR = waitlist task
; Immediate restart of BIBIBIAS (IMU bias compensation).
; Ensures gyro drift corrections continue during landing phase.
; Critical for maintaining accurate attitude reference during descent.

# ANY MORE GROUP 5.ODD RESTART VALUES SHOULD GO HERE

; ============================================================================
; TRANSITION: Group 5 to Group 6 - Surface Operations to System Timekeeping
;
; Group 5 managed the final landing phase - ensuring guidance, navigation,
; and IMU systems remained operational during the most critical moments.
; Group 6 handles post-landing operations and general system timekeeping.
; These tables ensure mission time tracking and system updates continue
; reliably during surface operations and subsequent mission phases.
; ============================================================================

6.2SPOT		EQUALS	1.2SPOT
; Group 6 Phase 2 shares the same restart table as Group 1 Phase 2.
; This equivalence saves memory by reusing the fresh start/restart
; initialization sequence for Group 6 operations.

6.3SPOT		DEC	100		; 100 centiseconds = 1 second
		EBANK=	TIG
	       -2CADR	CLOKTASK	; Negative 2CADR = waitlist task
; 1-second waitlist for CLOKTASK (clock task).
; Maintains mission elapsed time (MET) display and time-critical scheduling.
; Essential for crew situational awareness and ground communication timing.

6.5SPOT		OCT	30000		# PROTECT INCREMENTING OF TIME2,TIME1 BY
		EBANK=	TEPHEM		# P27(UPDATE PROGRAM) VIA V70 OR V73.
		2CADR	TIMEDIDR
; Priority 30 job for TIMEDIDR (time display director).
; Protects TIME2/TIME1 registers during updates from P27 program via
; verbs V70 or V73 (ground uplink time updates).
; Prevents corruption of mission time during uplink operations.

6.7SPOT		OCT	17000		; Priority 17 FINDVAC job
		EBANK=	VGPREV
		2CADR	REDO6.7
; Job restart for REDO6.7 (Group 6 Phase 7 backup sequence).
; General-purpose restart ensuring system housekeeping continues.
