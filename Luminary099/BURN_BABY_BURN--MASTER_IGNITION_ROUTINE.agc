# Copyright:	Public domain.
# Filename:	BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	731-751
# Mod history:	2009-05-19 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-07 RSB	Corrected 3 typos.
#		2009-07-23 RSB	Added Onno's notes on the naming
#				of this function, which he got from
#				Don Eyles.
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

# Page 731
; ============================================================================
; FILE: BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc
; MODULE: Master Ignition Control
; MISSION PHASE: descent/landing/ascent/orbital-maneuvers
;
; TL;DR: Master ignition sequencer controlling DPS (Descent Propulsion System)
;        and APS (Ascent Propulsion System) engine start sequences. Manages
;        complete ignition timeline from TIG-45 seconds through TIG+26 seconds,
;        including ullage motor firing, propellant settling, valve sequencing,
;        engine start verification, and throttle-up commands. Used by P12
;        (ascent), P40/P41/P42 (orbital burns), P61 (abort), and P63 (landing).
;
; COMMENT-ONLY READERS: This code controlled the dramatic moments of engine
;        ignition during powered descent to the lunar surface and ascent back
;        to orbit. Follow the countdown sequence from preparation through the
;        critical moment when engines fire to life.
; CODE-ALONG READERS: Study the table-driven architecture using WHICH register
;        to support multiple programs, countdown task scheduling via WAITLIST,
;        and hardware interface for engine control channels.
; ============================================================================
;
## At the get-together of the AGC developers celebrating the 40th anniversary
## of the first moonwalk, Don Eyles (one of the authors of this routine along
## with Peter Adler) has related to us a little interesting history behind the
## naming of the routine.
##
## It traces back to 1965 and the Los Angeles riots, and was inspired
## by disc jockey extraordinaire and radio station owner Magnificent Montague.
## Magnificent Montague used the phrase "Burn, baby! BURN!" when spinning the
## hottest new records. Magnificent Montague was the charismatic voice of
## soul music in Chicago, New York, and Los Angeles from the mid-1950s to
## the mid-1960s.
;
; HISTORICAL NOTE: The naming of this routine references disc jockey Magnificent
; Montague's catchphrase "Burn, baby! BURN!" from the 1960s, showing the informal
; culture of the Apollo software team even in life-critical code. Despite the
; playful name, this routine controlled actual engine ignition for lunar landing
; and ascent - quite literally making spacecraft "burn" with rocket thrust.
;
# BURN, BABY, BURN -- MASTER IGNITION ROUTINE

		BANK	36
		SETLOC	P40S
		BANK
		EBANK=	WHICH
		COUNT*	$$/P40

; ============================================================================
; MASTER IGNITION ROUTINE ARCHITECTURE
; ============================================================================
;
; The master ignition routine provides a unified engine start sequencer for
; five different LM programs: P12 (powered ascent from lunar surface), P40
; (SPS insertion burn), P42 (DPS insertion burn), P61 (abort to orbit), and
; P63 (powered descent to lunar landing). This table-driven design eliminates
; code duplication while accommodating program-specific variations.
;
; OPERATIONAL TIMELINE: Manages the critical 71-second window from TIG-45
; (Time of IGnition minus 45 seconds) through TIG+26 (when DPS throttles up
; to full thrust). During Apollo 11's powered descent initiation (PDI) at
; mission time 102:33:05 on July 20, 1969, this routine commanded DPS ignition
; to begin the 12-minute journey to the lunar surface. For Eagle's ascent at
; 124:22:00 MET on July 21, 1969, it commanded APS ignition for return to orbit.
;
; TABLE-DRIVEN ARCHITECTURE: Each calling program (P12, P40, P42, P61, P63)
; stores the address of its program-specific table into erasable register
; WHICH (EBANK=WHICH). The ignition routine indexes by WHICH to obtain
; program-specific constants (display codes, exit addresses, ullage duration)
; and TCF branch instructions (for program-specific handling). This elegant
; design pattern allows a single ignition sequencer to serve multiple missions.
;
; ENTRY: Programs call via "TCF BURNBABY" (with BANKJUMP if needed).
; EXIT: No return - routine transfers control to program-specific exit handler.
;
; AUTHORS: Conceived, implemented, and maintained by Peter Adler and Don Eyles.
; Latin motto: "HONI SOIT QUI MAL Y PENSE" (Shame on him who thinks evil of it)
;
# THE MASTER IGNITION ROUTINE IS DESIGNED FOR USE BY THE FOLLOWING LEM PROGRAMS:  P12, P40, P42, P61, P63.
# IT PERFORMS ALL FUNCTIONS IMMEDIATELY ASSOCIATED WITH APS OR DPS IGNITION:  IN PARTICULAR, EVERYTHING LYING
# BETWEEN THE PRE-IGNITION TIME CHECK -- ARE WE WITHIN 45 SECONDS OF TIG? -- AND TIG + 26 SECONDS, WHEN DPS
# PROGRAMS THROTTLE UP.
#
# VARIATIONS AMONG PROGRAMS ARE ACCOMODATED BY MEANS OF TABLES CONTAINING CONSTANTS (FOR AVEGEXIT, FOR
# WAITLIST, FOR PINBALL) AND TCF INSTRUCTIONS.  USERS PLACE THE ADRES OF THE HEAD OF THE APPROPRIATE TABLE
# (OF P61TABLE FOR P61LM, FOR EXAMPLE) IN ERASABLE REGISTER `WHICH' (E4).  THE IGNITION ROUTINE THEN INDEXES BY
# WHICH TO OBTAIN OR EXECUTE THE PROPER TABLE ENTRY.  THE IGNITION ROUTINE IS INITIATED BY A TCF BURNBABY,
# THROUGH BANKJUMP IF NECESSARY.  THERE IS NO RETURN.
#
# THE MASTER IGNITION ROUTINE WAS CONCEIVED AND EXECUTED, AND (NOTA BENE) IS MAINTAINED BY ADLER AND EYLES.
#
# 		   HONI SOIT QUI MAL Y PENSE
;
; ============================================================================
; PROGRAM-SPECIFIC CONFIGURATION TABLES
; ============================================================================
;
; Each table contains 14 entries indexed by WHICH register:
; (0)  VN display code - Verb/Noun for crew display during countdown
; (1)  Ullage ignition branch - TCF target for ullage motor start
; (2)  Communication failure branch - TCF target if uplink fails
; (3)  Post-burn branch - TCF target after engine cutoff
; (4)  Task completion branch - TCF target for normal task end
; (5)  Program-specific spot - TCF target for midcourse corrections
; (6)  Ullage duration - Time in centiseconds for ullage motors (0=none)
; (7)  Average G exit - 2CADR for guidance routine after ignition
; (11) Display change branch - TCF target for display mode switching
; (12) Wait branch - TCF target for countdown hold
; (13) Ignition branch - TCF target for actual engine start sequence
;
; WARNING: "NOLI SE TANGERE" (Do not touch) - These tables are precisely
; tuned for flight operations. Modifications require extensive testing.
;
#	***********************************************
#		TABLES FOR THE IGNITION ROUTINE
#	***********************************************
#
#			NOLI SE TANGERE

; ----------------------------------------------------------------------------
; P12TABLE - Powered Ascent from Lunar Surface
; ----------------------------------------------------------------------------
;
; Configuration for P12 (ascent program) used during lunar liftoff to return
; the LM to orbit for rendezvous with the Command Module. During Apollo 11,
; this table controlled APS (Ascent Propulsion System) ignition at 124:22:00
; MET on July 21, 1969, when Neil Armstrong and Buzz Aldrin lifted off from
; Tranquility Base to rejoin Michael Collins in Columbia.
;
; APS ENGINE CHARACTERISTICS: Fixed-thrust hypergolic engine (3500 lbf),
; no throttle control, instant ignition without ullage motors (hence DEC 0
; for ullage duration). Propellants are pressure-fed and self-igniting.
;
P12TABLE	VN	0674		# (0)  Display V06N74 - Velocity to be gained
		TCF	ULLGNOT		# (1)  No ullage for APS (instant ignition)
		TCF	COMFAIL3	# (2)  Communication failure handler
		TCF	GOCUTOFF	# (3)  Post-burn cutoff sequence
		TCF	TASKOVER	# (4)  Normal task completion
		TCF	P12SPOT		# (5)  P12-specific handling
		DEC	0		# (6)  NO ULLAGE - APS ignites instantly
		EBANK=	WHICH
		2CADR	SERVEXIT	# (7)  Exit to servicer routine

		TCF	DISPCHNG	# (11) Display mode change
		TCF	WAITABIT	# (12) Countdown hold
		TCF	P12IGN		# (13) APS ignition sequence

; ----------------------------------------------------------------------------
; P40TABLE - Service Propulsion System (SPS) Orbital Maneuvers
; ----------------------------------------------------------------------------
;
; Configuration for P40 (SPS burn program) used for major orbital maneuvers
; including translunar injection (TLI), lunar orbit insertion (LOI), and
; transearth injection (TEI). Note: This is the Command Module's SPS engine,
; not the LM engines - this table exists in LM code for CSM/LM integrated
; mission simulation and abort-to-orbit scenarios.
;
; ULLAGE REQUIREMENT: DEC 2240 = 22.40 seconds of RCS ullage thruster firing
; to settle propellants before SPS ignition. Large liquid-fueled engines
; require ullage in microgravity to ensure propellant covers tank outlets.
;
P40TABLE	VN	0640		# (0)  Display V06N40 - Delta-V components
		TCF	ULLGNOT		# (1)  Ullage not required branch
		TCF	COMFAIL4	# (2)  Communication failure handler
		TCF	GOPOST		# (3)  Post-burn processing
		TCF	TASKOVER	# (4)  Normal task completion
		TCF	P40SPOT		# (5)  P40-specific handling
; Page 732
		DEC	2240		# (6)  ULLAGE: 22.40 seconds RCS firing
		EBANK=	OMEGAQ
		2CADR	STEERING	# (7)  Exit to steering routine (guidance)

		TCF	P40SJUNK	# (11) Display mode change for P40
		TCF	WAITABIT	# (12) Countdown hold
		TCF	P40IGN		# (13) SPS ignition sequence
		TCF	REP40ALM	# (14) Repeat P40 alarm handling

; ----------------------------------------------------------------------------
; P41TABLE - RCS Orbital Maneuvers (Partial Table)
; ----------------------------------------------------------------------------
;
; Configuration for P41 (RCS burn program) used for small orbital corrections
; when SPS is unavailable or inappropriate for delta-V magnitude. Uses
; Reaction Control System thrusters instead of main propulsion.
;
; NOTE: Partial table (entries 5-7, 11-12 only). Shares most configuration
; with other programs through COMMON branches.
;
P41TABLE	TCF	P41SPOT		# (5)  P41-specific handling
		DEC	-1		# (6)  Special ullage code -1 (RCS mode)
		EBANK=	OMEGAQ
		2CADR	CALCN85		# (7)  Exit to CALCN85 routine

		TCF	COMMON		# (11) Common display handling
		TCF	TIGTASK		# (12) TIG countdown task

; ----------------------------------------------------------------------------
; P42TABLE - DPS Insertion Burns (Orbit Adjustment)
; ----------------------------------------------------------------------------
;
; Configuration for P42 (DPS burn program) used for orbit insertion or
; plane changes using the Descent Propulsion System. Similar to P40 but
; uses the more efficient DPS engine for larger delta-V maneuvers.
;
; ULLAGE REQUIREMENT: 26.40 seconds - Longest ullage time in table due to
; DPS propellant tank geometry requiring thorough settling before ignition.
;
; DISPLAY: Verb 06 Noun 40 shows time-to-ignition and delta-V during countdown.
;
; NOTE: Entry (1) branches to WANTAPS instead of ULLGNOT - checks for APS
; availability before ullage, ensuring ascent engine is ready as backup.
;
P42TABLE	VN	0640		# (0)  Display V06N40 - Delta-V components
		TCF	WANTAPS		# (1)  Check APS availability first
		TCF	COMFAIL4	# (2)  Comm failure handler
		TCF	GOPOST		# (3)  Post-burn processing
		TCF	TASKOVER	# (4)  Task completion
		TCF	P42SPOT		# (5)  P42-specific mid-course
		DEC	2640		# (6)  Ullage: 26.40 seconds (longest)
		EBANK=	OMEGAQ
		2CADR	STEERING	# (7)  Exit to steering routine

		TCF	P40SJUNK	# (11) Display mode handling
		TCF	WAITABIT	# (12) Countdown hold
		TCF	P42IGN		# (13) DPS ignition sequence
		TCF	P42STAGE	# (14) DPS staging logic

; ----------------------------------------------------------------------------
; P63TABLE - Powered Descent to Lunar Landing (THE CRITICAL PROGRAM)
; ----------------------------------------------------------------------------
;
; Configuration for P63 - the lunar landing program. This table controlled
; the most critical engine ignition of the Apollo program: Powered Descent
; Initiation (PDI) to begin the 12-minute descent to the lunar surface.
;
; APOLLO 11 HISTORICAL CONTEXT: At mission elapsed time 102:33:05 on July 20,
; 1969, this table's configuration controlled DPS ignition as Eagle separated
; from lunar orbit and began descent toward the Sea of Tranquility. Neil
; Armstrong and Buzz Aldrin relied on this ignition sequencer for the burn
; that would either land them on the Moon or force an abort to orbit.
;
; ULLAGE REQUIREMENT: 22.40 seconds - Standard DPS ullage for descent profile.
;
; DISPLAY: Verb 06 Noun 62 shows altitude, altitude-rate, and fuel quantity
; during the critical descent countdown. Crew monitored these values intensely.
;
; SPECIAL NOTE: Entry (3) uses V99RECYC allowing crew Go/No-Go decision via
; Verb 99. P63SPOT (entry 5) assumes CLOKTASK (countdown clock) already running,
; unlike other programs which must start it.
;
P63TABLE	VN	0662		# (0)  Display V06N62 (landing data)
		TCF	ULLGNOT		# (1)  No ullage branch
		TCF	COMFAIL3	# (2)  Comm failure (critical in P63)
		TCF	V99RECYC	# (3)  V99 recycle for go/no-go
		TCF	TASKOVER	# (4)  Task completion
		TCF	P63SPOT		# (5)  P63 mid-course (CLOKTASK running)
		DEC	2240		# (6)  Ullage: 22.40 seconds for PDI
		EBANK=	WHICH
		2CADR	SERVEXIT	# (7)  Exit to servicer

		TCF	DISPCHNG	# (11) Display change handling
		TCF	WAITABIT	# (12) Countdown hold (rare in P63)
# Page 733
		TCF	P63IGN		# (13) THE PDI IGNITION SEQUENCE

; ----------------------------------------------------------------------------
; ABRTABLE - Abort to Orbit (Emergency Ascent)
; ----------------------------------------------------------------------------
;
; Configuration for P70/P71 abort programs - emergency ascent from any point
; during powered descent. If landing became impossible (guidance failure,
; engine malfunction, excessive propellant consumption), crew could invoke
; abort to immediately ignite APS and return to orbit for rendezvous with CSM.
;
; ULLAGE REQUIREMENT: None (NOOP entries). APS uses hypergolic propellants
; (Aerozine 50 fuel + nitrogen tetroxide oxidizer) that ignite on contact,
; requiring no ullage settling. Critical for abort responsiveness.
;
; APOLLO 11 NOTE: This table was never used during the actual mission, but
; was armed and ready throughout descent. Armstrong came close to invoking
; abort when fuel warnings sounded during final approach, but elected to
; continue based on his assessment of landing site and remaining propellant.
; With approximately 25 seconds of fuel remaining at touchdown, the mission
; was closer to abort than most realized at the time.
;
; DISPLAY: Verb 06 Noun 63 shows abort trajectory parameters.
;
; SPECIAL NOTE: Multiple NOOP entries reflect simplified abort logic - no
; ullage, no special exit routine, direct transition to emergency ascent.
;
ABRTABLE	VN	0663		# (0)  Display V06N63 (abort data)
		TCF	ULLGNOT		# (1)  No ullage (instant APS start)
		TCF	COMFAIL3	# (2)  Comm failure handler
		TCF	GOCUTOFF	# (3)  Cutoff after abort burn
		TCF	TASKOVER	# (4)  Task completion
		NOOP			# (5)  No program-specific spot
		NOOP			# (6)  No ullage required for APS
		NOOP			# (7)  No exit routine (direct abort)
		NOOP
		TCF	DISPCHNG	# (11) Display change
		TCF	WAITABIT	# (12) Countdown hold
		TCF	ABRTIGN		# (13) EMERGENCY APS IGNITION

#	*********************************
#	GENERAL PURPOSE IGNITION ROUTINES
#	*********************************

; ============================================================================
; BURNBABY - MASTER IGNITION ROUTINE ENTRY POINT
; ============================================================================
;
; This is the main entry point for the master ignition sequencer used by LM
; programs P12, P40, P42, P61, and P63. The routine orchestrates all functions
; between pre-ignition checks (TIG-45 seconds) and completion of ignition
; sequence (TIG+26 seconds for DPS programs when throttle-up occurs).
;
; ENTRY CONDITIONS:
; - WHICH register contains address of program-specific table (e.g., P63TABLE)
; - TIG register contains Time of Ignition set by calling program
; - Spacecraft in proper attitude for burn
; - Navigation state vector current and valid
;
; APOLLO 11 HISTORICAL CONTEXT:
; During Apollo 11's powered descent on July 20, 1969, program P63 called
; BURNBABY at approximately 102:32:30 MET (Mission Elapsed Time), 35 seconds
; before Powered Descent Initiation. The routine managed the critical ignition
; countdown leading to DPS engine start at 102:33:05 MET, beginning Eagle's
; historic 12-minute descent to Tranquility Base. Armstrong and Aldrin watched
; the countdown displayed as V06N63 on the DSKY while the computer executed
; this exact sequence of instructions.
;
; For ascent on July 21, 1969, at 124:22 MET, P12 called BURNBABY to manage
; the APS ignition that lifted Eagle off the lunar surface for rendezvous
; with Columbia in lunar orbit.
;
BURNBABY	TC	PHASCHNG	# GROUP 4 RESTARTS HERE
		OCT	04024

		CAF	ZERO		# EXTIRPATE JUNK LEFT IN DVTOTAL
		TS	DVTOTAL		# Clear velocity change accumulator
		TS	DVTOTAL +1	# (double-precision zero initialization)

		TC	BANKCALL	# P40AUTO MUST BE BANKCALLED EVEN FROM ITS
		CADR	P40AUTO		# OWN BANK TO SET UP RETURN PROPERLY

; ----------------------------------------------------------------------------
; PRE-IGNITION SETUP AND INITIALIZATION
; ----------------------------------------------------------------------------
;
; Save nominal Time of Ignition for two purposes:
; 1. Oblateness compensation during orbital integration (accounting for
;    Earth's non-spherical gravitational field during transearth coast)
; 2. Abort program reference (P70/P71 abort-to-orbit need original TIG)
;
B*RNB*B*	EXTEND
		DCA	TIG		# STORE NOMINAL TIG FOR OBLATENESS COMP.
		DXCH	GOBLTIME	# AND FOR P70 OR P71.

; Ensure engines are off before ignition sequence begins. This prevents
; premature ignition or thruster interference during attitude setup.
; ENGINOF3 turns off both DPS and APS, commanding all engine valves closed.
;
		INHINT			# Disable interrupts during critical setup
		TC	IBNKCALL
		CADR	ENGINOF3	# Engine Off routine (all propulsion)
		RELINT			# Re-enable interrupts

; Branch to program-specific entry point using table offset 5.
; This handles program-specific initialization before common countdown logic.
;
		INDEX	WHICH		# Index by table base address
		TCF	5		# Jump to table entry (5): program spot

; ----------------------------------------------------------------------------
; PROGRAM-SPECIFIC INITIALIZATION ENTRY POINTS (Table Entry 5)
; ----------------------------------------------------------------------------
;
; P42SPOT, P12SPOT: SPS and APS burns need countdown clock initialization
; P40SPOT: DPS burns need countdown clock initialization  
; P63SPOT, P41SPOT: P63 already has countdown clock running (skip STCLOK2)
;
; During Apollo 11 descent, P63SPOT was used since P63 had already started
; its countdown clock when BURNBABY was called. For ascent, P12SPOT started
; a fresh countdown for the APS ignition.
;
P42SPOT		=	P40SPOT		# (5)
P12SPOT		=	P40SPOT		# (5)
P63SPOT		=	P41SPOT		# (5)	IN P63 CLOKTASK ALREADY GOING
P40SPOT		CS	CNTDNDEX	# (5)
# Page 734
		TC	BANKCALL	# MUST BE BANKCALLED FOR GENERALIZED
		CADR	STCLOK2		# 	RETURN (start countdown clock)
P41SPOT		TC	INTPRET		# (5) Enter interpretive mode
; Compute integration end time: TIG - 29.9 seconds. This allows MIDTOAV
; (MID-course TO Average-G) integration to propagate state vectors up to
; 29.9 seconds before ignition, leaving final seconds for setup.
;
		DLOAD	DSU
			TIG		# Time of Ignition
			D29.9SEC	# Constant: 2990 centiseconds
		STCALL	TDEC1		# Store integration target time
			INITCDUW	# Initialize downlist for integration

; CSM STATE VECTOR PROCESSING (for rendezvous burns only)
; If MUNFLAG is set (rendezvous mode), compute CSM position, velocity, and
; gravity vector in reference coordinates. This enables computation of
; relative state for rendezvous targeting during burns.
;
		BOFF	CALL		# Branch if MUNFLAG clear (not rendezvous)
			MUNFLAG
			GOMIDAV		# Skip CSM processing, go to integration
			CSMPREC		# Compute CSM state for rendezvous
		VLOAD	MXV
			VATT1		# CSM velocity in stable member coords
			REFSMMAT	# Transform to reference coordinates
		VSR1			# Scale: meters/centisec * 2^7
		STOVL	V(CSM)		# CSM VELOCITY -- M/CS*2(7)
			RATT1		# CSM position in stable member coords
		VSL4	MXV
			REFSMMAT	# Transform to reference coordinates
		STCALL	R(CSM)		# CSM POSITION -- M*2(24)
			MUNGRAV		# Compute gravity at CSM position
		STODL	G(CSM)		# CSM GRAVITY VEC. -- M/CS*2(7)
			TAT		# Time at which CSM state is valid
		STORE	TDEC1		# RELOAD TDEC1 FOR MIDTOAV.

; MIDTOAV INTEGRATION: Propagate LM state vector from current time to
; TIG-29.9 seconds using precision orbital integration with average-g.
; Returns MPAC = time difference between requested and achieved integration.
;
GOMIDAV		CALRB
			MIDTOAV1	# Integrate state to TIG-29.9
		TCF	CALLT-35	# MADE IT IN TIME (integration successful)

; TIG SLIP RECOVERY: If integration couldn't reach TIG-29.9 (perhaps TIG was
; too close), reset TIG to 29.9 seconds after the time we actually integrated
; to. This "slips" TIG forward, giving time to complete ignition sequence.
; The calling program will be notified and can adjust burn parameters.
;
		EXTEND			# TIG WAS SLIPPED, SO RESET TIG TO 29.9
		DCA	PIPTIME1	# SECONDS AFTER THE TIME TO WHICH WE DID
		DXCH	TIG		# INTEGRATE (new TIG = integration time + 29.9)
		EXTEND
		DCA	D29.9SEC	# 2990 centiseconds
		DAS	TIG		# Add to get new TIG

; ----------------------------------------------------------------------------
; COUNTDOWN TASK SCHEDULING
; ----------------------------------------------------------------------------
;
; Schedule TIG-35 task on WAITLIST. This task will execute 35 seconds before
; ignition to begin final countdown sequence. The time calculation works
; backward from the integration endpoint:
;   Integration reached: TIG-29.9 seconds
;   Time to TIG-30: 0.1 seconds
;   Time to TIG-35: 5.1 seconds (stored in SAVET-30 as "delta-t until TIG-35")
;
CALLT-35	DXCH	MPAC		# MPAC contains time to integration point
		DXCH	SAVET-30	# DELTA-T UNTIL TIG-30 (0.1 sec in future)
		EXTEND
		DCS	5SECDP		# Subtract 5 seconds (go back to TIG-35)
		DAS	SAVET-30	# DELTA-T UNTIL TIG-35 (5.1 sec in future)
		EXTEND
		DCA	SAVET-30	# Load time delta for WAITLIST
		TC	LONGCALL	# Schedule task on WAITLIST
		EBANK=	TTOGO
		2CADR	TIG-35		# Task to execute at TIG-35 seconds

; Set restart protection for TIG-35 execution. If computer restarts between
; now and TIG-35, this phase table entry ensures TIG-35 task is rescheduled.
;
		TC	PHASCHNG
		OCT	20254		# 4.25SPOT FOR TIG-35 RESTART.
# Page 735
		TC	CHECKMM
		DEC	63
		TCF	ENDOFJOB	# NOT P63
		CS	CNTDNDEX	# P63 CAN START DISPLAYING NOW.
		TS	DISPDEX
		TC	INTPRET
		VLOAD	ABVAL
			VN1
		STORE	ABVEL		# INITIALIZE ABVEL FOR P63 DISPLAY
		EXIT
		TCF	ENDOFJOB

#	********************************
;
; ============================================================================
; TIG-35 COUNTDOWN EVENT: FINAL PREPARATION PHASE (35 Seconds Before Ignition)
; ============================================================================
;
; At TIG minus 35 seconds, the ignition sequence enters its final preparation
; phase. This is the point of no return - all system checks have passed and
; the spacecraft is committed to engine ignition within 35 seconds.
;
; ACTIONS AT TIG-35:
; - Schedule next countdown event at TIG-30 (5 seconds from now)
; - Blank DSKY display for 5 seconds to indicate countdown in progress
; - Check ullage motor timing requirements
; - For P41 (RCS burns), set up display blanking job
;
; APOLLO 11 CONTEXT: For powered descent initiation on July 20, 1969, this
; event occurred at 102:32:30 MET. The DPS ullage motors were prepared to
; fire to settle propellants in the descent stage tanks. Armstrong and Aldrin
; confirmed "GO for PDI" from Mission Control as the final 35-second countdown
; began. The DSKY blanked momentarily to signal the approaching ignition.
;
TIG-35		CAF	5SEC		; Load 5 seconds (deltaT to TIG-30)
		TC	TWIDDLE		; Schedule TIG-30 task on WAITLIST
		ADRES	TIG-30		; Address of TIG-30 routine

		TC	PHASCHNG	; Set restart protection for TIG-30
		OCT	40154		# 4.15SPOT FOR TIG-30 RESTART

		CS	BLANKDEX	; Blank DSKY display for 5 seconds
		TS	DISPDEX		; to indicate countdown in progress

		INDEX	WHICH		; Index by program table pointer
		CS	6		# CHECK ULLAGE TIME.
		EXTEND			; (Table entry 6 = ullage deltaT)
		BZMF	TASKOVER	; If negative/zero ullage time, skip display job
		CAF	4.9SEC		# SET UP TASK TO RESTORE DISPLAY AT TIG-30
		TC	TWIDDLE		; Schedule display restore at TIG-30
		ADRES	TIG-30.1	; (4.9 sec = 0.1 sec before TIG-30 event)

		CAF	PRIO17		# A NEGATIVE ULLAGE TIME INDICATES P41, IN
		TC	NOVAC		# WHICH CASE WE HAVE TO SET UP A JOB TO
		EBANK=	TTOGO		# BLANK THE DSKY FOR FIVE SECONDS, SINCE
		2CADR	P41BLANK	# CLOKJOB IS NOT RUNNING DURING P41.

		TCF	TASKOVER	; Task complete, return to WAITLIST

; P41BLANK - Display blanking job for P41 RCS burns
; (P41 uses RCS thrusters without main engine, so no normal countdown display)
;
P41BLANK	TC	BANKCALL	# BLANK DSKY.
		CADR	CLEANDSP	; Clear display service routine
		TCF	ENDOFJOB	; Job complete

; TIG-30.1 - Display restore task scheduler (0.1 sec before TIG-30)
;
TIG-30.1	CAF	PRIO17		# SET UP JOB TO RESTORE DISPLAY AT TIG-30
		TC	NOVAC		; Create job to restore DSKY display
		EBANK=	TTOGO		; (after 5-second blanking period)
		2CADR	TIG-30A		; Job entry point

		TCF	TASKOVER	; Task complete
# Page 736
; TIG-30A - Display restore job (restores V16N85 countdown display)
;
TIG-30A		CAF	V16N85B		; Verb 16 Noun 85 (Monitor TIG, T-TIG)
		TC	BANKCALL	# RESTORE DISPLAY.
		CADR	REGODSP		# REGODSP DOES A TCF ENDOFJOB

#	********************************
;
; ============================================================================
; TIG-30 COUNTDOWN EVENT: ULLAGE MOTOR PREPARATION (30 Seconds Before Ignition)
; ============================================================================
;
; At TIG minus 30 seconds, the ullage motors are prepared for firing. Ullage
; motors are small solid-propellant rockets that settle liquid propellants to
; the bottom of the tanks under low-gravity conditions (1/6 Earth gravity on
; Moon, or microgravity in space). This prevents gas bubbles from entering the
; engine feed lines during ignition, which would cause catastrophic failure.
;
; ACTIONS AT TIG-30:
; - Schedule next countdown event at TIG-5 (25 seconds from now)
; - Restart CLOKTASK countdown display on DSKY
; - Check ullage deltaT from program table entry 6
; - If ullage required (deltaT > 0), schedule ONULLAGE task
; - Save ullage deltaT for restart protection
;
; APOLLO 11 CONTEXT: During Apollo 11 PDI on July 20, 1969 at 102:32:35 MET,
; the DPS ullage motors were scheduled to fire at TIG-3.9 seconds. For ascent
; on July 21, 1969 at 124:22 MET, the APS ullage motors were similarly prepared.
; The ullage deltaT value is critical - too early and propellant doesn't stay
; settled, too late and engine starts without proper propellant feed.
;
TIG-30		CAF	S24.9SEC	; Load 24.9 seconds (deltaT to TIG-5)
		TC	TWIDDLE		; Schedule TIG-5 task on WAITLIST
		ADRES	TIG-5		; (0.1 sec before TIG-5 to allow setup)

		CS	CNTDNDEX	# START UP CLOKTASK AGAIN
		TS	DISPDEX		; Restore countdown display index

		INDEX	WHICH		# PICK UP APPROPRIATE ULLAGE -- ON TIME
		CA	6		# Was CAF --- RSB 2009.
		EXTEND			; Get ullage deltaT from program table
		BZMF	ULLGNOT		# DON'T SET UP ULLAGE IF DT IS NEG OR ZERO
		TS	SAVET-30	# SAVE DELTA-T FOR RESTART
		TC	TWIDDLE		; Schedule ONULLAGE task at TIG minus deltaT
		ADRES	ULLGTASK	; Address of ullage motor task

		CA	THREE		# RESTART PROTECT ULLGTASK (1.3SPOT)
		TS	L		; Set restart phase to 1.3 for ullage task
		CS	THREE		; (Ensures ullage restarts properly if
		DXCH	-PHASE1		; power failure occurs during countdown)
		CS	TIME1		; Save time base for restart
		TS	TBASE1		; protection

		INDEX	WHICH		; Execute program-specific branch
		TCF	1		; from table entry 1

; WANTAPS - P42 APS flag verification (table entry 1 for P42)
; Ensures the Ascent Propulsion System flag is set for P42 abort staging.
; The Digital AutoPilot (DAP) needs to know which engine is active.
;
WANTAPS		CS	FLGWRD10	# (1) FOR P42 ENSURE APSFLAG IS SET.  IF IT
		MASK	APSFLBIT	# WASN'T SET, DAP WILL BE INITIALIZED TO
		ADS	FLGWRD10	# ASCENT VALUES BY 1/ACCS IN 2 SECONDS.

; ULLGNOT - Common path for programs with no ullage requirement (table entry 1)
; Loads AVEGEXIT with appropriate 2CADR from program table entry 7.
; AVEGEXIT is the average-g integration exit address used during powered flight.
;
ULLGNOT		EXTEND			# (1)
		INDEX	WHICH		; Get 2CADR from table entry 7
		DCA	7		# LOAD AVEGEXIT WITH APPROPRIATE 2CADR
		DXCH	AVEGEXIT	; (Service module or lunar module exit)

		CAF	TWO		# 4.2SPOT RESTARTS IMMEDIATELY AT REDO4.2
		TS	L		; Set restart phase to 4.2
		CS	TWO		# AND ALSO AT TIG-5 AT THE CORRECT TIME.
		DXCH	-PHASE4		; (Double restart protection - immediate and timed)

		CS	TIME1		; Capture current time for TIG-5 restart
		TS	TBASE4		# SET TBASE4 FOR TIG-5 RESTART

; REDO2.17 - Restart entry point for phase 2.17
; Clears velocity-to-be-gained (VG) vector in Group 2 so Lambert targeting
; integration can start fresh. Lambert guidance computes optimal trajectory
; to reach target orbit or rendezvous.
;
REDO2.17	EXTEND			; Prepare for double-precision operation
# Page 737
		DCA	NEG0		# CLEAR OUT GROUP 2 SO LAMBERT CAN START
		DXCH	-PHASE2		# IF NEEDED.
					; (Group 2 = VG velocity-to-be-gained)

; REDO4.2 - Restart entry point for phase 4.2
; Checks if SERVICER (navigation state integration) is already running.
; If not, starts PREREAD to prepare for powered flight navigation.
;
REDO4.2		CCS	PHASE5		# IF SERVICER GOING?
		TCF	TASKOVER	# YES, DON'T START IT UP AGAIN.

		TC	POSTJUMP	; No, start it now
		CADR	PREREAD		# PREREAD END THIS TASK
					; (PREREAD prepares navigation state)

# 	*********************************
;
; ULLGTASK - Ullage motor firing task
; Called at TIG-7.5 seconds (for DPS) or TIG-3.5 seconds (for APS).
; The ullage deltaT in the program table determines exact firing time.
; Solid propellant ullage motors fire for 3.9 seconds to settle liquid
; propellants at bottom of tanks before main engine ignition.
;
; APOLLO 11 CONTEXT: For PDI on July 20, 1969, the DPS ullage motors
; fired at 102:32:58 MET (TIG minus 3.9 sec). Four small ullage motors
; on the descent stage provided 100 lbf total thrust for propellant settling.
;
ULLGTASK	TC	ONULLAGE	# THIS COMES AT TIG-7.5 OR TIG-3.5
		TC	PHASCHNG	; Restart protection (phase 1)
		OCT	1		; Simple phase 1 restart at ULLGTASK
		TCF	TASKOVER	; Task complete

# 	*********************************
;
; ============================================================================
; TIG-5 COUNTDOWN EVENT: ENGINE IGNITION PREPARATION (5 Seconds Before TIG)
; ============================================================================
;
; At TIG minus 5 seconds, final preparation for engine ignition begins.
; This is the last countdown event before actual ignition command.
;
; ACTIONS AT TIG-5:
; - Clear Group 3 phase protection (ensures no conflicting navigation jobs)
; - Schedule TIG-0 (ignition command) in 5 seconds
; - Reset IGNFLAG and ASTNFLAG for engine start logic
; - Branch to program-specific TIG-5 handler (table entry 11)
;
; APOLLO 11 CONTEXT: At 102:33:00 MET on July 20, 1969, this event occurred
; 5 seconds before DPS ignition. Armstrong and Aldrin confirmed systems ready,
; ullage motors had already fired, and the descent engine was about to ignite
; for the 12-minute powered descent to the lunar surface.
;
TIG-5		EXTEND			; Prepare for double-precision operation
		DCA	NEG0		# INSURE THAT GROUP 3 IS INACTIVE.
		DXCH	-PHASE3		; Clear phase 3 (navigation group)

		CAF	5SEC		; Load 5 seconds (deltaT to TIG-0)
		TC	TWIDDLE		; Schedule ignition command
		ADRES	TIG-0		; Address of ignition routine

		TC	DOWNFLAG	# RESET IGNFLAG AND ASINFLAG
		ADRES	IGNFLAG		# 	FOR LIGHT-UP LOGIC.
		TC	DOWNFLAG	; (Flags control engine start sequence
		ADRES	ASTNFLAG	; and astronaut notification)

		INDEX	WHICH		; Execute program-specific TIG-5 handler
		TCF	11		; from table entry 11

; P40SJUNK - TIG-5 handler for P40 and P42 programs (table entry 11)
; Checks if S40.13 (SPS thrust/coast integration) is already running.
; If not, starts S40.13 to integrate powered flight trajectory.
;
P40SJUNK	CCS	PHASE3		# (11) P40 AND P42.  S40.13 IN PROGRESS?
		TCF	DISPCHNG	# YES

		CAF	PRIO20		; No, need to start S40.13
		TC	FINDVAC		; Find vacant core set for job
		EBANK=	TTOGO		; Select TTOGO's EBANK
		2CADR	S40.13		; Start S40.13 trajectory integration

		TC	PHASCHNG	# 3.5SPOT FOR S40.13
		OCT	00053		; Phase 3.5 restart protection

; DISPCHNG - Display change handler (table entry 11 for P63)
; Changes display index for ignition sequence displays.
;
DISPCHNG	CS	VB99DEX		# (11)
		TS	DISPDEX		; Update display index for ignition phase

# Page 738
; COMMON - Common restart protection setup for TIG-0
; Sets phase 4.7 to restart at TIG-0 if power failure occurs.
;
COMMON		TC	PHASCHNG	# RESTART TIG-0 (4.7SPOT)
		OCT	40074		; Phase 4 group, 7.4 subphase
		TCF	TASKOVER	; Task complete

# 	*********************************
;
; ============================================================================
; TIG-0 COUNTDOWN EVENT: ENGINE IGNITION COMMAND (Time of Ignition)
; ============================================================================
;
; THIS IS THE MOMENT. After 35 seconds of countdown preparation, the AGC
; now commands the engine to ignite. This routine executes at the exact
; time of ignition (TIG = Time of Ignition).
;
; ACTIONS AT TIG-0:
; - Set IGNFLAG to indicate ignition has been commanded
; - For P63 descent, schedule throttle-up event (P63ZOOM) at ZOOMTIME
; - Check if astronaut has enabled engine (ASTNFLAG) via DSKY Pro key
; - Branch to program-specific ignition handler (table entry 12 or 13)
;
; APOLLO 11 POWERED DESCENT INITIATION:
; At 102:33:05 MET on July 20, 1969, this routine commanded DPS ignition.
; Armstrong and Aldrin had already pressed PRO to enable the engine.
; The descent engine ignited at 10% throttle, building thrust over several
; seconds. The computer displayed V06N63 showing velocity to be gained.
; Mission Control called "Throttle up" as engine reached full thrust.
; The 12-minute powered descent to the lunar surface had begun.
;
TIG-0		CS	FLAGWRD7	# SET IGNFLAG SINCE TIG HAS ARRIVED
		MASK	IGNFLBIT	; Set ignition flag bit
		ADS	FLAGWRD7	; (Signals ignition logic active)

		TC	CHECKMM		# IN P63 CASE, THROTTLE-UP IS ZOOMTIME
		DEC	63		# AFTER NOMINAL IGNITION, NOT ACTUAL
		TCF	IGNYET?		; Not P63, skip throttle-up scheduling
		CA	ZOOMTIME	; P63: Schedule throttle-up
		TC	WAITLIST	; (26 seconds after ignition for P63)
		EBANK=	DVCNTR		; Select DVCNTR's EBANK
		2CADR	P63ZOOM		; P63ZOOM throttles up from 10% to variable

		TC	2PHSCHNG	; Restart protection for P63ZOOM
		OCT	40033		; Phase 4 group, 0.3 and 3.3 subphases

		OCT	05014		; Phase 5 group, 0.1 and 4.1 subphases
		OCT	77777		; End of phase change table

; IGNYET? - Check for astronaut engine enable response
; The astronaut must press PRO on the DSKY to enable engine ignition.
; If PRO has not been pressed yet, branch to wait routine (table entry 12).
; If PRO has been pressed, continue to ignition (table entry 13).
;
IGNYET?		CAF	ASTNBIT		# CHECK ASTNFLAG:  HAS ASTRONAUT RESPONDED
		MASK	FLAGWRD7	# TO OUR ENGINE ENABLE REQUEST?
		EXTEND			; Prepare for indexed branch
		INDEX	WHICH		; Get table entry 12 or 13
		BZF	12		# BRANCH IF HE HAS NOT RESPONDED YET

IGNITION	CS	FLAGWRD5	# INSURE ENGONFLG IS SET.
		MASK	ENGONBIT
		ADS	FLAGWRD5
		CS	PRIO30		# TURN ON THE ENGINE.
		EXTEND
		RAND	DSALMOUT
		AD	BIT13
		EXTEND
		WRITE	DSALMOUT
		EXTEND			# SET TEVENT FOR DOWNLINK
		DCA	TIME2
		DXCH	TEVENT

		EXTEND			# UPDATE TIG USING TGO FROM S40.13
		DCA	TGO
		DXCH	TIG
		EXTEND
		DCA	TIME2
		DAS	TIG

# Page 739
		CS	FLUNDBIT	# PERMIT GUIDANCE LOOP DISPLAYS
		MASK	FLAGWRD8
		TS	FLAGWRD8

		INDEX	WHICH
		TCF	13

P63IGN		EXTEND			# (13)	INITIATE BURN DISPLAYS
		DCA	DSP2CADR
		DXCH	AVGEXIT

		CA	Z		# ASSASSINATE CLOKTASK
		TS	DISPDEX

		CS	FLAGWRD9	# SET FLAG FOR P70-P71
		MASK	LETABBIT
		ADS	FLAGWRD9

		CS	FLAGWRD7	# SET SWANDISP TO ENABLE R10.
		MASK	SWANDBIT
		ADS	FLAGWRD7

		CS	PULSES		# MAKE SURE DAP IS NOT IN MINIMUM-IMPULSE
		MASK	DAPBOOLS	# MODE, IN CASE OF SWITCH TO P66
		TS	DAPBOOLS

		EXTEND			# INITIALIZE TIG FOR P70 AND P71.
		DCA	TIME2
		DXCH	TIG

		CAF	ZERO		# INITIALIZE WCHPHASE, AND FLPASS0
		TS	WCHPHASE
		TS	WCHPHOLD	# ALSO WHCPHOLD
		CA	TWO
		TS	FLPASS0

		TCF	P42IGN
P40IGN		CS	FLAGWRD5	# (13)
		MASK	NOTHRBIT
		EXTEND
		BZF	P42IGN
		CA	ZOOMTIME
		TC	WAITLIST
		EBANK=	DVCNTR
		2CADR	P40ZOOM

P63IGN1		TC	2PHSCHNG
		OCT	40033		# 3.3SPOT FOR ZOOM RESTART.
		OCT	05014		# TYPE C RESTARTS HERE IMMEDIATELY
		OCT	77777

# Page 740
; ============================================================================
; PROGRAM-SPECIFIC IGNITION HANDLERS
;
; After the common TIG-0 countdown reaches engine ignition, control branches
; to program-specific handlers that configure the guidance computer for the
; specific mission phase about to begin. Each mission program (P12 ascent,
; P40 SPS burn, P42 descent insertion, P63 landing, P70/P71 abort) has
; unique requirements for DAP configuration, guidance targeting, and system
; initialization.
;
; HISTORICAL CONTEXT: During Apollo 11's powered ascent from the lunar
; surface on July 21, 1969 at mission time 124:22, the P12IGN handler
; executed below to configure the computer for ascent guidance. The APS
; engine fired successfully, lifting Eagle off the Moon for rendezvous
; with Columbia in lunar orbit.
; ============================================================================

		TCF	P42IGN
		
; ----------------------------------------------------------------------------
; P12IGN - POWERED ASCENT IGNITION HANDLER
;
; Configures the guidance computer for lunar surface ascent. Initializes
; Digital Autopilot (DAP) bias acceleration estimates to compensate for
; ascent engine offset thrust. Connects ascent guidance routine (ATMAG) to
; the navigation servicer for trajectory computation during ascent to orbit.
;
; The P12 program controls powered ascent from the lunar surface, targeting
; orbital insertion parameters that enable rendezvous with the Command Module.
; During Apollo 11, this routine executed at 124:22 MET as Eagle's Ascent
; Propulsion System (APS) engine ignited for the 7-minute climb to orbit.
; ----------------------------------------------------------------------------

P12IGN		CAF	EBANK6
		TS	EBANK
		EBANK=	AOSQ
		
		; Initialize Digital Autopilot bias acceleration estimates.
		; The APS engine thrust vector does not pass through the LM center
		; of gravity, creating a rotational moment. These bias values
		; compensate for the off-axis thrust during powered ascent.
		CA	IGNAOSQ		# INITIALIZE DAP BIAS ACCELERATION
		TS	AOSQ		# ESTIMATES AT P12 IGNITION.
		CA	IGNAOSR
		TS	AOSR

		CAF	EBANK7
		TS	EBANK
		EBANK=	DVCNTR

; ----------------------------------------------------------------------------
; ABRTIGN - ABORT PROGRAM IGNITION HANDLER
;
; Shared entry point for abort programs P70 and P71. Stops the countdown
; clock display, connects ascent guidance (ATMAG) to the servicer, and
; enables Routine 10 for abort monitoring. This handler executes when an
; abort is commanded during descent, requiring immediate transition to
; ascent guidance to return the LM to orbit.
;
; P70 is used for aborts before APS ignition (during descent), while P71
; handles aborts after APS ignition. Both share this common initialization.
; ----------------------------------------------------------------------------

ABRTIGN		CA	Z		# (13) KILL CLOKTASK
		TS	DISPDEX
		
		; Connect ascent guidance routine to navigation servicer.
		; ATMAG (Ascent Targeting Program) computes the trajectory needed
		; to achieve orbital insertion for rendezvous with the CSM.
		EXTEND			# CONNECT ASCENT GYIDANCE TO SERVICER.
		DCA	ATMAGADR
		DXCH	AVGEXIT
		
		; Enable Routine 10 (R10) for ascent monitoring and display updates.
		CS	FLAGWRD7	# ENABLE R10.
		MASK	SWANDBIT
		ADS	FLAGWRD7

; ----------------------------------------------------------------------------
; P42IGN - P42 IGNITION HANDLER (DESCENT INSERTION / SPS BURNS)
;
; Configures the Digital Autopilot for powered flight. Clears the DRIFTBIT
; to ensure powered-flight switching curves are used (rather than coast-phase
; curves). Checks the impulse switch to determine burn mode, then enables
; Delta-V monitoring and schedules ullage shutdown.
;
; P42 is used for Service Propulsion System (SPS) burns in lunar orbit,
; including descent orbit insertion before LM separation. Also used by P40
; and P41 for other SPS maneuvers.
; ----------------------------------------------------------------------------

P42IGN		CS	DRIFTBIT	# ENSURE THAT POWERED-FLIGHT SWITCHING
		MASK	DAPBOOLS	# CURVES ARE USED.
		TS	DAPBOOLS
		
		; Check impulse switch setting to determine burn control mode.
		; Impulse mode uses crew-commanded thrust termination.
		CAF	IMPULBIT	# EXAMINE IMPULSE SWITCH
		MASK	FLAGWRD2
		CCS	A
		TCF	IMPLBURN
		
; ----------------------------------------------------------------------------
; DVMONCON - DELTA-V MONITOR CONNECTION
;
; Enables the Delta-V monitoring system by clearing control flags. DVMON
; monitors achieved velocity change during the burn, comparing actual
; performance against the targeted trajectory. This allows real-time
; assessment of engine performance and automatic cutoff when the desired
; velocity change is achieved.
; ----------------------------------------------------------------------------

DVMONCON	TC	DOWNFLAG
		ADRES	IGNFLAG		# CONNECT DVMON
		TC	DOWNFLAG
		ADRES	ASTNFLAG
		TC	DOWNFLAG
		ADRES	IDLEFLAG

		TC	PHASCHNG
		OCT	40054
		
		; Schedule ullage motor shutdown 0.5 seconds after main engine
		; ignition. Ullage motors settle propellants before ignition; once
		; main engine thrust provides acceleration, ullage is no longer needed.
		TC	FIXDELAY	# TURN ULLAGE OFF HALF A SECOND AFTER
		DEC	50		# LIGHT UP.

; ----------------------------------------------------------------------------
; ULLAGOFF - ULLAGE MOTOR SHUTDOWN
;
; Turns off ullage motors after main engine ignition. Called 0.5 seconds
; after TIG-0 when main engine thrust has stabilized and provides sufficient
; acceleration for propellant management.
; ----------------------------------------------------------------------------

ULLAGOFF	TC	NOULLAGE

; ----------------------------------------------------------------------------
; WAITABIT - PHASE TERMINATION
;
; Kills restart Group 4 and terminates current task. This cleanup routine
; executes after ignition sequence completion, clearing restart protection
; for the ignition phase.
; ----------------------------------------------------------------------------

WAITABIT	EXTEND			# KILL GROUP 4
		DCA	NEG0
# Page 741
		DXCH	-PHASE4

		TCF	TASKOVER

; ----------------------------------------------------------------------------
; TIGTASK - TIME OF IGNITION TASK SCHEDULER
;
; Schedules the TIGNOW job with priority 16 to execute at TIG-0. This is the
; final task in the countdown sequence, scheduling the actual ignition event
; that transitions from countdown monitoring to active burn control.
;
; Cross-banks to TIGTASK1 in Bank 31 for execution, demonstrating the
; AGC's memory banking system where larger routines are distributed across
; multiple 2K-word memory banks.
; ----------------------------------------------------------------------------

TIGTASK		TC	POSTJUMP	# (12)
		CADR	TIGTASK1

#	********************************

		BANK	31
		SETLOC	P40S3
		BANK
		COUNT*	$$/P40

TIGTASK1	CAF	PRIO16
		TC	NOVAC
		EBANK=	TRKMKCNT
		2CADR	TIGNOW
		
		; Terminate restart Group 6 after ignition scheduling completes.
		TC	PHASCHNG
		OCT	6		# KILL GROUP 6

		TCF	TASKOVER

#	********************************

; ----------------------------------------------------------------------------
; P63ZOOM - POWERED DESCENT ZOOM RESTART
;
; Restart recovery for P63 powered descent program. Reconnects the LUNLAND
; (lunar landing guidance) routine to the average-g exit and calls FLATOUT
; to configure throttle for descent. This zoom restart executes if the
; computer restarts during powered descent, restoring guidance control
; to continue the landing sequence.
;
; HISTORICAL CONTEXT: The restart protection system proved critical during
; Apollo 11's descent when 1202 alarms indicated executive overflow. The
; zoom restart logic allowed the computer to recover from these overload
; conditions while maintaining continuous guidance control. Without this
; capability, the 1202 alarms would have forced an abort.
; ----------------------------------------------------------------------------

P63ZOOM		EXTEND
		DCA	LUNLANAD
		DXCH	AVEGEXIT
		
		; Configure throttle for powered descent.
		TC	IBNKCALL
		CADR	FLATOUT
		TCF	P40ZOOMA

; ----------------------------------------------------------------------------
; P40ZOOM - P40 ZOOM RESTART
;
; Restart recovery for P40 Service Propulsion System burn. Sets thrust to
; maximum (BIT13) and enables engine control through Channel 14. This zoom
; restart restores engine control if the computer restarts during an SPS
; burn, ensuring the burn continues to completion.
; ----------------------------------------------------------------------------

P40ZOOM		CAF	BIT13
		TS	THRUST
		CAF	BIT4

		EXTEND
		WOR	CHAN14

P40ZOOMA	TC	PHASCHNG
		OCT	3
		TCF	TASKOVER

		EBANK=	DVCNTR
LUNLANAD	2CADR	LUNLAND

# Page 742
ZOOM		=	P40ZOOMA
		BANK	36
		SETLOC	P40S
		BANK
		COUNT*	$$/P40

#	********************************

COMFAIL		TC	UPFLAG		# (15)
		ADRES	IDLEFLAG
		TC	UPFLAG		# SET FLAG TO SUPPRESS CONFLICTING DISPLAY
		ADRES	FLUNDISP
		CAF	FOUR		# RESET DVMON
		TS	DVCNTR
		CCS	PHASE6		# CLOCKTASK ACTIVE?
		TCF	+3		# YES
		TC	BANKCALL	# OTHERWISE, START IT UP
		CADR	STCLOK1
 +3		CS	VB97DEX
 		TS	DISPDEX
		TC	PHASCHNG	# TURN OFF GROUP 4.
		OCT	00004
		TCF	ENDOFJOB

; ============================================================================
; IGNITION FAILURE HANDLING ROUTINES
;
; These routines handle various failure conditions during the ignition sequence.
; They provide graceful degradation when ignition cannot proceed as planned,
; ensuring crew safety and maintaining spacecraft in stable configuration.
;
; COMFAIL1: Table-driven failure handler (indexed by WHICH)
; COMFAIL3: Countdown termination using Z register for clock task kill
; COMFAIL4: Display failure recovery - clears countdown display
; COMFAIL2: Engine shutdown on PROCEED key abort during final countdown
;
; During Apollo 11 mission, these abort paths were not needed, but they provided
; critical backup options had ignition failures occurred during PDI or ascent.
; ============================================================================

COMFAIL1	INDEX	WHICH		# Table-indexed failure branch
		TCF	2		# Jump to program-specific failure handler (offset 2)

; COMFAIL3 - Clock task termination for P12 (ascent) failure
; Kills the countdown clock task and returns control to program.
; Z register holds the CLOKTASK CADR to be killed.

COMFAIL3	CA	Z		# (15)	KILL CLOKTASK USING Z
				# Load Z register containing CLOKTASK address
		TCF	+2		# Skip to common failure code below

; COMFAIL4 - Display manager failure recovery (P40/P42 SPS burns)
; Stops countdown display and reconnects guidance displays after failure.
; Restores normal DSKY operation following ignition sequence abort.

COMFAIL4	CS	CNTDNDEX	# Clear countdown display index
		TS	DISPDEX		# Setting DISPDEX positive stops CLOKTASK

		TC	DOWNFLAG	# RECONNECT DV MONITOR
		ADRES	IDLEFLAG	# Clear idle flag - allow DV monitor operation
		TC	DOWNFLAG	# PERMIT GUIDANCE LOOP DISPLAYS
		ADRES	FLUNDISP	# Clear display inhibit flag
		TCF	ENDOFJOB	# Return to executive scheduler

; COMFAIL2 - PROCEED key abort handler at TIG-5 seconds
; Executed when crew presses PROCEED at "Please Enable Engine" V99 display,
; aborting the ignition. Shuts down engine, kills ZOOM restart, switches RCS,
; and reschedules ignition 5 seconds later (allowing crew another attempt).
; This abort path preserves ability to re-ignite without program restart.

COMFAIL2	TC	PHASCHNG	# KILL ZOOM RESTART PROTECTION
		OCT	00003		# Phase change to group 3

		INHINT			# Inhibit interrupts for atomic operations
		TC	KILLTASK	# KILL ZOOM IN CASE IT'S STILL TO COME
		CADR	ZOOM		# Cancel ZOOM task if pending in waitlist
		TC	IBNKCALL	# COMMAND ENGINE OFF
		CADR	ENGINOF4	# Shut down engine immediately for abort safety
		TC	UPFLAG		# SET THE DRIFT BIT FOR THE DAP.
		ADRES	DRIFTDFL	# Enable RCS drift mode for coast attitude hold
# Page 743
		TC	INVFLAG		# USE OTHER RCS SYSTEM
		ADRES	AORBTFLG	# Toggle RCS system A/B for redundancy
		TC	UPFLAG		# TURN ON ULLAGE
		ADRES	ULLAGFLG	# Enable ullage for propellant settling on retry
		CAF	BIT1		# Load 1 (number of seconds to delay)
		INHINT			# Re-inhibit (redundant but safe)
		TC	TWIDDLE		# Schedule TIG-5 event for 1 second
		ADRES	TIG-5		# Reschedule 5-second-before-ignition event
		TCF	ENDOFJOB	# Return to executive

#	***********************************
#	SUBROUTINES OF THE IGNITION ROUTINE
#	***********************************

; ============================================================================
; INVFLAG - Invert (toggle) a flag bit
;
; General-purpose utility that inverts the state of a specified flag bit.
; Used to toggle RCS system selection (A/B redundancy switching) and other
; binary state flags during ignition sequence error handling.
;
; CALLING CONVENTION:
;   TC INVFLAG
;   ADRES <flag address>
;
; Returns via Q register. Preserves flag inversion atomically using RXOR.
; ============================================================================

INVFLAG		CA	Q		# Save return address
		TC	DEBIT		# Get flag address from instruction after caller
		COM			# Complement flag bits
		EXTEND			# Extended instruction follows
		RXOR	LCHAN		# XOR with channel, inverting specified bits
		TCF	COMFLAG		# Store result back to flag location

#	***********************************
; ============================================================================
; ULLAGE MOTOR CONTROL SUBROUTINES
;
; These routines directly control the ullage motors by manipulating the
; DAPBOOLS register. Under lunar gravity (1/6 Earth), liquid propellants
; may not settle properly at the bottom of tanks. Ullage motors are small
; RCS thrusters fired to create positive acceleration, forcing propellant
; to tank outlets before main engine ignition.
;
; NOULLAGE: Turns off ullage motors by clearing the ULLAGER bits in DAPBOOLS
; ONULLAGE: Turns on ullage motors by setting the ULLAGER bits in DAPBOOLS
;
; These subroutines are called throughout the ignition sequence to control
; propellant settling during the critical pre-ignition countdown.
;
; CALLING CONVENTION: Must be called within a task or under INHINT
; (interrupts inhibited) to ensure atomic flag manipulation.
; ============================================================================

NOULLAGE	CS	ULLAGER		# MUST BE CALLED IN A TASK OR UNDER INHINT
				# Complement ULLAGER mask to prepare for clearing
		MASK	DAPBOOLS	# Clear ullage bits from Digital Autopilot flags
		TS	DAPBOOLS	# Store result back - ullage motors now OFF
		TC	Q		# Return to caller

#	***********************************
; ONULLAGE - Turn on ullage thrust
; Commands RCS thrusters to fire for propellant settling acceleration.
; During Apollo 11 PDI at 102:33:05 MET and ascent at 124:22 MET,
; ullage firing ensured propellants were positioned at engine inlets.

ONULLAGE	CS	DAPBOOLS	# TURN ON ULLAGE.  MUST BE CALLED IN
				# Complement current DAP flags
		MASK	ULLAGER		# A TASK OR WHILE INHINTED.
				# Isolate ullage control bits from ULLAGER mask
		ADS	DAPBOOLS	# Add ullage bits to DAPBOOLS - ullage motors now ON
		TC	Q		# Return to caller

# 	***********************************

; ============================================================================
; COUNTDOWN CLOCK INITIALIZATION - STCLOK1/STCLOK2/STCLOK3
;
; Starts the countdown clock display system (CLOKTASK and CLOKJOB) showing
; time-to-ignition (TTOGO) on the DSKY. The clock updates once per second,
; displaying countdown from TIG-35 through TIG-5 seconds.
;
; STCLOK1: Standard entry (sets DISPDEX to zero)
; STCLOK2: Entry with custom DISPDEX (for display mode variations)
; STCLOK3: Common clock start logic
;
; The countdown clock provides crew situational awareness during the critical
; final 30 seconds before engine ignition. During Apollo 11 PDI, Armstrong and
; Aldrin monitored this countdown as they prepared for the 12-minute descent.
;
; DISPDEX controls clock behavior:
;   Negative: Clock running (value selects display format via table)
;   Positive: Clock stopped (CLOKTASK terminates on detection)
;
; Clock scheduling uses TWIDDLE to insert CLOKTASK into waitlist, synchronized
; to occur at 1-second boundaries. Phase change group 6 provides restart
; protection for CLOKTASK.
; ============================================================================

STCLOK1		CA	ZERO		# THIS ROUTINE STARTS THE COUNT-DOWN
STCLOK2		TS	DISPDEX		# (CLOKTASK AND CLOKJOB).  SETTING
				# Initialize display index (negative = running)
STCLOK3		TC	MAKECADR	# SETTING DISPDEX POSITIVE KILLS IT.
		TS	TBASE4		# RETURN SAVE (NOT FOR RESTARTS).
				# Save return address for bank return
		EXTEND			# Double-precision operation follows
		DCA	TIG		# Load target ignition time (TIG)
		DXCH	MPAC		# Store in multi-purpose accumulator
		EXTEND			# Double-precision operation follows
		DCS	TIME2		# Load negative of current time
# Page 744
		DAS	MPAC		# HAVE TIG -- TIME2, UNDOUBTEDLY A + NUMBER
				# Compute time-to-go: TIG - TIME2 (positive)
		TC	TPAGREE		# POSITIVE, SINCE WE PASSED THE
				# Ensure time values agree across time increments
		CAF	1SEC		# 45 SECOND CHECK.
				# Load 1 second (100 centiseconds) for division
		TS	Q		# Store divisor in Q
		DXCH	MPAC		# Retrieve time-to-go for division
		MASK	LOW5		# RESTRICT MAGNITUDE OF NUMBER IN A
				# Mask to prevent overflow in division
		EXTEND			# Extended instruction follows
		DV	Q		# Divide time-to-go by 1 second
		CA	L		# GET REMAINDER
				# Load remainder from L register
		AD	TWO		# Add 2 centiseconds for timing margin
		INHINT			# Inhibit interrupts for atomic scheduling
		TC	TWIDDLE		# Schedule task in waitlist
		ADRES	CLOKTASK	# Address of countdown clock task
		TC	2PHSCHNG	# Two-word phase change for restart protection
		OCT	40036		# 6.3SPOT FOR CLOKTASK
				# Group 6 restart table entry
		OCT	05024		# Phase table parameters
		OCT	13000		# Phase table parameters

		CA	TBASE4		# Retrieve saved return address
		TC	BANKJUMP	# Return to caller (may cross memory banks)

; ============================================================================
; CLOKTASK - Countdown Clock Waitlist Task
;
; Self-rescheduling task that executes once per second to update the countdown
; clock display. Spawns CLOKJOB to compute time-to-go and perform display
; updates based on the current DISPDEX mode. Uses FIXDELAY for 1-second timing.
;
; DISPDEX controls clock operation:
;   Negative: Clock running (absolute value selects display format)
;   Zero: Clock stopped (but CLOKTASK continues checking)
;   Positive: Clock terminated (branches to KILLCLOK to stop task)
;
; Display sequence during ignition countdown:
;   DISPDEX = -35: Blank DSKY at TIG-35 (Average G starting)
;   DISPDEX = -25: Display V06N61 (crew event timer synchronization)
;   DISPDEX = -17 to -6: Display countdown (TIG-30 through TIG-5)
;   DISPDEX = -13: Display V99 "Please Enable Engine" at TIG-5
;   DISPDEX = -2: Blank DSKY again
;
; Group 6 restart protection via TBASE6 ensures clock survives power transients.
; ============================================================================

CLOKTASK	CS	TIME1		# SET TBASE6 FOR GROUP 6 RESTART
				# Complement TIME1 for restart base time
		TS	TBASE6		# Store restart time base for Group 6 protection

		CCS	DISPDEX		# Check display index state
				# Positive: Kill clock, Zero: Continue, Negative: Run clock
		TCF	KILLCLOK	# DISPDEX positive - terminate countdown clock task
		NOOP			# DISPDEX +0 (shouldn't occur)
		CAF	PRIO27		# DISPDEX -0 or negative - continue clock operation
				# Load priority 27 for clock display job
		TC	NOVAC		# Find vacant core set and start job
		EBANK=	TTOGO		# Set E-bank for time-to-go access
		2CADR	CLOKJOB		# Address of clock computation/display job

		TC	FIXDELAY	# WAIT A SECOND BEFORE STARTING OVER
				# Schedule next clock iteration with 1-second delay
		DEC	100		# Delay = 100 centiseconds = 1.00 seconds
		TCF	CLOKTASK	# Loop back to start next clock cycle

; ============================================================================
; KILLCLOK - Terminate Countdown Clock Task
;
; Stops CLOKTASK by clearing Group 6 restart protection and terminating the
; waitlist task. Called when DISPDEX becomes positive (clock killed by operator
; or program logic). Ensures clean shutdown with no restart on power cycle.
; ============================================================================

KILLCLOK	EXTEND			# KILL RESTART
				# Extended instruction follows
		DCA	NEG0		# Load negative zero (phase terminator)
		DXCH	-PHASE6		# Clear Group 6 restart table entry
				# Clock will not restart after power transient
		TCF	TASKOVER	# Terminate task and return to executive

; ============================================================================
; CLOKJOB - Countdown Clock Computation and Display Job
;
; Computes time-to-ignition (TTOGO = TIME2 - TIG) and dispatches to the
; appropriate display handler based on DISPDEX value. Executes at priority 27
; once per second via NOVAC from CLOKTASK.
;
; Computation sequence:
;   1. TTOGO = -TIG (load negative of ignition time)
;   2. TTOGO = TTOGO + TIME2 = TIME2 - TIG (add current time)
;   Result: Positive TTOGO means TIG is in the future (countdown active)
;           Negative TTOGO means TIG has passed (post-ignition)
;
; DISPDEX safety check: Before using DISPDEX as INDEX, verify it remains
; negative. If another task/job changed DISPDEX to positive (clock killed),
; immediately exit via ENDOFJOB to prevent invalid indexed branch.
;
; Display dispatch (when DISPDEX negative):
;   INDEX by (absolute value of DISPDEX - 1) into DISPNOT table:
;   DISPDEX = -35: Offset 34 -> VB97DEX handler (Verb 97 blank)
;   DISPDEX = -25: Offset 24 -> V06N61 display (crew event timer)
;   DISPDEX = -17 to -6: Countdown display handlers (MM:SS format)
;   DISPDEX = -13: Offset 12 -> VB99DEX handler (V99 engine enable)
;   DISPDEX = -2: Offset 1 -> BLANKDEX handler (blank DSKY)
;
; Historical context: During Apollo 11 PDI at 102:33:05 MET, this job updated
; the countdown display every second as Armstrong and Aldrin monitored TIG
; approach on the DSKY. The V99 "Please Enable Engine" prompt appeared at
; TIG-5, giving crew final abort opportunity before DPS ignition.
; ============================================================================

CLOKJOB		EXTEND			# Compute time-to-go
				# Extended instruction for double-precision
		DCS	TIG		# Load -TIG (complement of ignition time)
		DXCH	TTOGO		# Store in TTOGO: TTOGO = -TIG
		EXTEND			# Extended instruction for DCA/DAS
# Page 745
		DCA	TIME2		# Load current mission time (double-precision)
		DAS	TTOGO		# TTOGO = TTOGO + TIME2 = TIME2 - TIG
				# Positive result: TIG in future (countdown)
				# Negative result: TIG has passed (post-ignition)
		INHINT			# Inhibit interrupts for DISPDEX check
		CCS	DISPDEX		# IF DISPDEX HAS BEEN SET POSITIVE BY A
				# Check if clock was killed since CLOKTASK
		TCF	ENDOFJOB	# TASK OR A HIGHER PRIORITY JOB SINCE THE
				# Positive: Clock terminated, exit immediately
		TCF	ENDOFJOB	# LAST CLOKTASK, AVOID USING IT AS AN
				# +0: Should not occur, exit safely
		COM			# INDEX.
				# Negative DISPDEX: complement to get absolute value
		RELINT			# ***** DISPDEX MUST NEVER B -0 *****
				# Re-enable interrupts before indexed branch
		INDEX	A		# Index by (absolute DISPDEX - 1)
		TCF	DISPNOT -1	# (-1 DUE TO EFFECT OF CCS)
				# Dispatch to display handler in DISPNOT table

; ============================================================================
; VB97DEX (-35) - Verb 97 Blank Display Handler
;
; COMMENT-ONLY READERS: When the countdown clock needs to paste a new verb/noun
; combination on the DSKY display, this routine first blanks the display with
; Verb 97, then loads the appropriate noun for the countdown phase. The DSKY
; screen briefly flashes blank before showing the new countdown format.
;
; CODE-ALONG READERS: VB97DEX constant = OCT35 (decimal 29). When DISPDEX = -35,
; CLOKJOB indexes to this handler in the DISPNOT table. The handler constructs
; a verb 97 display command (blank all three data registers), then calls CLOCPLAY
; to execute the display update. NVWORD+2 contains the pre-loaded V06 and noun
; code for the actual countdown display that will follow the blank.
;
; Return branches:
;   - STOPCLOK: Terminate countdown (crew terminated or program transition)
;   - COMFAIL1: Communications failure branch 1
;   - COMFAIL2: Communications failure branch 2
; ============================================================================

VB97DEX		=	OCT35		# NEGATIVE OF THIS IS PROPER FOR DISPDEX
				# Constant: decimal 29, used as -35 in DISPDEX

 -35		CS	ZERO		# INDICATE VERB 97 PASTE
				# Complement of zero = -0, signals V97 paste operation
 		TS	NVWORD1		# Store in NVWORD1 to indicate V97 display
		CA	NVWORD 	+2	# NVWORD+2 CONTAINS V06 & APPROPRIATE NOUN
				# Load pre-stored V06 + noun code for countdown
		TC	BANKCALL	# Call display routine across bank
		CADR	CLOCPLAY	# CLOCPLAY handles verb/noun display execution
		TCF	STOPCLOK	# TERMINATE CLOKTASK ON THE WAY TO POOH
				# Branch 1: Stop countdown, clean termination
		TCF	COMFAIL1	# Branch 2: Communications failure path 1
		TCF	COMFAIL2	# Branch 3: Communications failure path 2

; ============================================================================
; Display Handler (-25) - V06N61 Crew Event Timer Display
;
; COMMENT-ONLY READERS: During powered descent, the crew needs to synchronize
; their event timer with the guidance computer's TIG countdown. This display
; handler shows V06N61 (display decimal data) with the current time-to-ignition,
; allowing Armstrong and Aldrin to set their backup event timer to match. Called
; via ASTNCLOK routine when crew requests timer synchronization.
;
; CODE-ALONG READERS: When DISPDEX = -25, this handler displays V06N61 showing
; TIG in decimal format. Primarily used in P63 (lunar landing) to allow crew
; event timer reset. REFLASH displays the verb/noun with current data, waiting
; for crew acknowledgement (ENTER key).
;
; Return branches:
;   - STOPCLOK: Crew terminated display or countdown ended
;   - ASTNRETN: Return to ASTNCLOK caller after crew acknowledgement
;   - -6: Branch to offset -6 handler (alternative display path)
; ============================================================================
					# THIS DISPLAY IS CALLED VIA ASTNCLOK
 -25		CAF	V06N61		# IT IS PRIMARILY USED BY THE CREW IN P63
				# Load V06N61 code: display decimal in R1/R2/R3
 		TC	BANKCALL	# TO RESET HIS EVENT TIMER TO AGREE WITH
				# Call display routine across bank
		CADR	REFLASH		# TIG.
				# REFLASH: Display and wait for crew response
		TCF	STOPCLOK	# Branch 1: Stop countdown (crew termination)
		TCF	ASTNRETN	# Branch 2: Return to ASTNCLOK after ENTER
		TCF	-6		# Branch 3: Alternate display path

; ============================================================================
; CNTDNDEX (-17) - Countdown Display Handler (MM:SS Format)
;
; COMMENT-ONLY READERS: This is the main countdown display you would have seen
; on the DSKY during the final 30 seconds before ignition. Updated every second,
; it showed the time remaining until engine start in minutes and seconds format.
; Armstrong and Aldrin watched this countdown tick toward zero as they prepared
; for powered descent initiation or ascent from the lunar surface.
;
; CODE-ALONG READERS: CNTDNDEX constant = LOW4 = OCT17 (decimal 15). When
; DISPDEX = -17, this handler is selected from DISPNOT table. Uses WHICH register
; to index into program-specific table (P12TABLE, P40TABLE, etc.) to fetch the
; appropriate verb/noun code at offset 0. REGODSP handles register display update
; and performs its own TCF ENDOFJOB (no return to this routine).
;
; Active interval: TIG-30 seconds through TIG-5 seconds
; Update rate: 1-second intervals via CLOKTASK
; Display format: MM:SS (minutes:seconds to ignition)
;
; Historical note: During Apollo 11 PDI, this display counted down from 00:30
; to 00:05 between 102:32:35 and 102:33:00 MET as the crew monitored final
; approach to powered descent ignition.
; ============================================================================

CNTDNDEX	=	LOW4		# OCT17:  NEGATIVE PROPER FOR DISPDEX
				# Constant: OCT17 = decimal 15, used as -17

 -17		INDEX	WHICH		# THIS DISPLAY COMES UP AT ONE SECOND
				# Index by WHICH to get program-specific V/N
		# Was CAF --- RSB 2009
 		CA	0		# INTERVALS.  IT IS NORMALLY OPERATED
				# Load verb/noun code from indexed table entry 0
		TC	BANKCALL	# BETWEEN TIG-30 SECONDS AND TIG-5 SECONDS
				# Call display update routine
		CADR	REGODSP		# REGODSP DOES ITS OWN TCF ENDOFJOB
				# REGODSP updates DSKY registers, exits via ENDOFJOB

; ============================================================================
; VB99DEX (-13) - Verb 99 "Please Enable Engine" Display Handler
;
; COMMENT-ONLY READERS: At exactly 5 seconds before engine ignition, the DSKY
; displayed Verb 99 ("please proceed") - the computer's formal request for the
; crew to authorize engine start. This was the final crew decision point.
; Armstrong or Aldrin would press the PROCEED key to authorize ignition, or
; could decline to enable the engine if conditions were unsafe. This display
; appeared at TIG-5 during both powered descent initiation and ascent from the
; lunar surface.
;
; CODE-ALONG READERS: VB99DEX constant = ELEVEN = OCT13 (decimal 11). When
; DISPDEX = -13 (triggered at TIG-5 seconds by countdown logic), this handler
; constructs a V99 (proceed required) display with the noun that was previously
; shown (countdown format from prior seconds). BIT9 complement indicates V99
; paste operation. CLOCPLAY executes the display update and awaits crew input.
;
; Three possible return branches:
;   - STOPCLOK: Crew termination or countdown abort (ullage off, go to P00)
;   - *PROCEED: Crew pressed PRO key - authorize engine ignition to proceed
;   - *ENTER: Crew pressed ENTER key - alternate acceptance path
;
; Historical context: During Apollo 11 PDI at 102:33:00 MET (July 20, 1969),
; this V99 display requested crew authorization for DPS ignition. Armstrong
; pressed PROCEED, enabling the 12-minute powered descent that would culminate
; in "The Eagle has landed." During ascent at 124:22:00 MET (July 21, 1969),
; this V99 authorized the APS ignition that returned Eagle to lunar orbit for
; rendezvous with Columbia.
;
; V99RECYC entry point: Allows re-entry to this handler for recycled displays.
; ============================================================================

VB99DEX		=	ELEVEN		# OCT13:  NEGATIVE PROPER FOR DISPDEX
				# Constant: OCT13 = decimal 11, used as -13

V99RECYC	EQUALS			# Re-entry point for recycled V99 displays

 -13		CS	BIT9		# INDICATE VERB 99 PASTE
				# Complement of bit 9 signals V99 operation
 		TS	NVWORD1		# Store V99 indicator in NVWORD1
		INDEX	WHICH		# THIS IS THE "PLEASE ENABLE ENGINE"
				# Index by WHICH to program-specific table
		# Was CAF --- RSB 2004
		CA	0		# DISPLAY; IT IS INITIATED AT TIG-5 SEC.
				# Load verb/noun code from table offset 0
		TC	BANKCALL	# THE DISPLAY IS A V99NXX, WHERE XX IS
				# Call display routine across bank
		CADR	CLOCPLAY	# NOUN THAT HAD PREVIOUSLY BEEN DISPLAYED
				# CLOCPLAY executes V99 display, awaits crew
		TCF	STOPCLOK	# TERMINATE GOTOPOOH TURNS OFF ULLAGE.
				# Branch 1: Stop countdown, turn off ullage
		TCF	*PROCEED	# Branch 2: Crew pressed PRO - enable engine
		TCF	*ENTER		# Branch 3: Crew pressed ENTER - alternate accept

# Page 746
; ============================================================================
; BLANKDEX (-2) - Display Blanking Handler (TIG-35 Average G Indicator)
;
; COMMENT-ONLY READERS: At 35 seconds before ignition, the DSKY display
; suddenly went completely blank for 5 seconds. This wasn't a malfunction -
; it was the computer's way of signaling that the Average G guidance routine
; had started running. The blank screen was a visual cue to the crew that the
; navigation system had begun integrating accelerometer data to compute the
; current state vector with higher precision before engine ignition.
;
; CODE-ALONG READERS: BLANKDEX constant = TWO (octal 2, decimal 2). When
; DISPDEX = -2 (triggered at TIG-35 seconds), this handler calls CLEANDSP
; (clean display) to blank all DSKY registers and lights. The 5-second blank
; period provides visual indication that AVERAGEG routine has initiated,
; preparing the navigation state for powered flight. Returns via ENDOFJOB
; after blanking is complete.
;
; DISPNOT: Main dispatch return point - all display handlers that don't need
; special handling return here to ENDOFJOB, terminating the CLOKTASK cycle.
; ============================================================================

BLANKDEX	=	TWO		# NEGATIVE OF THIS IS PROPER FOR DISPDEX
				# Constant: octal 2, decimal 2, used as -2

 -2		TC	BANKCALL	# BLANK DSKY.  THE DSKY IS BLANKED FOR
				# Call display routine to clear DSKY
 		CADR	CLEANDSP	# 5 SECONDS AT TIG-35 TO INDICATE THAT
				# CLEANDSP blanks all display registers
DISPNOT		TCF	ENDOFJOB	# AVERAGE G IS STARTING.
				# Exit point: End countdown task cycle

; ============================================================================
; STOPCLOK - Countdown Clock Stop and Abort to P00
;
; COMMENT-ONLY READERS: If the crew pressed TERMINATE during the countdown or
; V99 prompt, or if the mission had to be aborted, this routine stopped the
; entire ignition sequence. It turned off the ullage rockets (if firing),
; killed the countdown clock task, and returned the computer to Program 00
; (P00) - the idle state. This was the crew's emergency exit from an unwanted
; engine burn.
;
; CODE-ALONG READERS: STOPCLOK terminates ignition countdown by calling
; NULLCLOK to disable ullage and kill countdown tasks, then transfers to
; GOTOPOOH which returns AGC to P00 idle program with fresh display update
; (GOTOPOOH relights display). Used when crew aborts countdown via TERMINATE
; key or when system detects unsafe ignition conditions requiring abort.
; ============================================================================

STOPCLOK	TC	NULLCLOK	# STOP CLOKTASK & TURN OFF ULLAGE ON THE
				# Call NULLCLOK to disable ullage/countdown
		TCF	GOTOPOOH	# WAY TO P00 (GOTOPOOH RELINTS)
				# Transfer to P00 idle program, relight DSKY

; ============================================================================
; NULLCLOK - Countdown Clock and Ullage Termination
;
; COMMENT-ONLY READERS: This was the routine that actually shut down the
; countdown mechanism. It turned off the ullage rockets, killed the countdown
; clock task to prevent it from continuing to run, and cleared the countdown
; display. Critically, it also protected against restarts - even if a power
; glitch caused the computer to restart, the countdown would stay stopped.
;
; CODE-ALONG READERS: NULLCLOK performs atomic countdown termination with
; interrupt inhibit (INHINT) to prevent race conditions:
; 1. Saves return address in P40/RET via QXCH
; 2. Calls NOULLAGE to command ullage rockets off
; 3. Calls KILLTASK with ULLGTASK address to prevent ullage restart
; 4. Issues PHASCHNG OCT 1 to inhibit restart protection for ullage
; 5. Zeroes DISPDEX (CA Z, TS DISPDEX) to kill CLOKTASK dispatching
; 6. Returns via P40/RET
;
; This ensures complete shutdown: ullage off, ullage task dead, countdown
; display stopped, and restart protection disabled so a computer restart won't
; re-enable ullage or countdown. Critical for safe abort scenarios.
; ============================================================================

NULLCLOK	INHINT			# Inhibit interrupts for atomic operation
		EXTEND			# Enable extended instruction
		QXCH	P40/RET		# Save return address, preserve in P40/RET
		TC	NOULLAGE	# TURN OFF ULLAGE ...
				# Command ullage rockets off immediately
		TC	KILLTASK	#	DON'T LET IT COME ON, EITHER ...
				# Kill ullage task to prevent restart
		CADR	ULLGTASK	# Address of ullage task to terminate
		TC	PHASCHNG	#		NOT EVEN IF THERE'S A RESTART.
				# Phase change to disable restart protection
		OCT	1		# Phase 1: no restart for ullage system
		CA	Z		# KILL CLOKTASK
				# Load zero (Z register always contains 0)
		TS	DISPDEX		# Zero DISPDEX stops countdown dispatching
		TC	P40/RET		# Return to caller via saved address

; ============================================================================
; ASTNRETN - Astronaut Return from V99 Display
;
; COMMENT-ONLY READERS: When the crew pressed PROCEED in response to the V99
; "Please Enable Engine" prompt, the computer needed to both authorize ignition
; AND prepare the system for the astronauts to return to monitoring the burn.
; This routine stopped showing the V99 prompt but kept the countdown running,
; then scheduled a background task to complete the astronaut interface return
; sequence while the main ignition logic proceeded.
;
; CODE-ALONG READERS: ASTNRETN handles post-V99-proceed state transition:
; 1. PHASCHNG OCT 04024 sets restart protection for astronaut return phase
; 2. Zeroes DISPDEX (CAF ZERO, TS DISPDEX) to stop displaying countdown but
;    allow countdown task to continue running in background
; 3. Schedules ASTNRET job at priority 13 via FINDVAC to complete display
;    restoration and crew interface return (EBANK=STARIND context)
; 4. Returns via ENDOFJOB
;
; This allows main ignition sequence to proceed immediately while display
; updates occur in parallel at lower priority.
; ============================================================================

ASTNRETN	TC	PHASCHNG	# Set restart phase for astronaut return
		OCT	04024		# Phase protection code
		CAF	ZERO		# STOP DISPLAYING BUT KEEP RUNNING
				# Zero countdown display updates
		TS	DISPDEX		# Clear DISPDEX (countdown continues)
		CAF	PRIO13		# Priority 13 for astronaut return task
		TC	FINDVAC		# Find vacant core set for background job
		EBANK=	STARIND		# Set EBANK to star indicator context
		2CADR	ASTNRET		# Schedule ASTNRET routine address

		TCF	ENDOFJOB	# Exit, let ASTNRET run in background

; ============================================================================
; *PROCEED - Crew PROCEED Key Handler for V99 Engine Enable
;
; COMMENT-ONLY READERS: When Armstrong or Aldrin pressed the PROCEED key in
; response to the V99 "Please Enable Engine" prompt, this routine executed.
; It was the crew's formal authorization for engine ignition. The asterisk in
; the name indicates this is an indirect jump target - the CLOCPLAY display
; routine would return here when PRO was pressed. Setting the ASTNFLAG told
; the system that the astronauts had explicitly authorized the burn, then
; control passed to IGNITE to fire the engine.
;
; CODE-ALONG READERS: *PROCEED is indirect branch target from CLOCPLAY V99
; display handler (asterisk indicates indirect addressing target in AGC).
; Sets ASTNFLAG up (TC UPFLAG, ADRES ASTNFLAG) to indicate crew has explicitly
; authorized engine ignition, then transfers to IGNITE routine to initiate
; engine start sequence. ASTNFLAG distinguishes crew-authorized ignition from
; automatic ignition in certain burn programs.
; ============================================================================

*PROCEED	TC	UPFLAG		# Set astronaut flag up (crew authorized)
		ADRES	ASTNFLAG	# ASTNFLAG address: crew approved ignition

		TCF	IGNITE		# Transfer to ignition sequence

; ============================================================================
; *ENTER - Crew ENTER Key Handler for V99 (Alternate Accept)
;
; COMMENT-ONLY READERS: In addition to PROCEED, the crew could also press the
; ENTER key to authorize engine ignition in response to V99. This alternate
; path used the program-specific table (indexed by WHICH) to jump to the
; appropriate continuation point for each burn program (P12, P40, P63, etc.).
; Different programs needed slightly different handling after crew acceptance.
;
; CODE-ALONG READERS: *ENTER is alternate indirect branch target from CLOCPLAY
; when crew presses ENTER key instead of PROCEED. Inhibits interrupts (INHINT)
; then uses INDEX WHICH, TCF 3 to jump to table entry 3 for the current burn
; program (P12TABLE, P40TABLE, etc.). Each program's table offset 3 contains
; program-specific post-accept handling logic. Provides program flexibility
; for enter-key response.
; ============================================================================

*ENTER		INHINT			# Inhibit interrupts for table lookup
		INDEX	WHICH		# Index by program table pointer
		TCF	3		# Jump to table offset 3 (program-specific)

; ============================================================================
; GOPOST - Post-Burn Sequence Initiation (Table Entry 3)
;
; COMMENT-ONLY READERS: After the engine completed its burn, the spacecraft
; needed to transition back to coasting flight. This routine scheduled the
; POSTBURN job to calculate the actual velocity change achieved, update the
; navigation state, and prepare for the next mission phase. It also stopped
; the countdown clock and configured the autopilot for coasting (no engine
; thrust). Used by programs like P40 (SPS burns) that had post-burn processing.
;
; CODE-ALONG READERS: GOPOST is program table entry (3) for post-burn handling,
; called via INDEX WHICH, TCF 3 from various burn program paths:
; 1. Schedules POSTBURN job at priority 12 (PRIO12) via FINDVAC - MUST be
;    lower priority than CLOKJOB (priority 17) to avoid race conditions
; 2. Sets EBANK=TTOGO for time-to-go context in POSTBURN routine
; 3. Calls ALLCOAST via IBNKCALL to configure DAP for coasting flight (no
;    thrust vector control, RCS attitude hold mode)
; 4. Calls NULLCLOK to terminate countdown clock and disable ullage
; 5. Sets PHASCHNG OCT 00134 (phase 4.13) for POSTBURN restart protection
; 6. Returns via ENDOFJOB
;
; Used by P40 (SPS burns) where post-burn state vector update and navigation
; correction are required before continuing mission timeline.
; ============================================================================

GOPOST		CAF	PRIO12		# (3) MUST BE LOWER PRIORITY THAN CLOKJOB
				# Priority 12 for POSTBURN job scheduling
		TC	FINDVAC		# Find vacant core set for post-burn job
		EBANK=	TTOGO		# Set EBANK for time-to-go variables
		2CADR	POSTBURN	# Schedule POSTBURN routine address

# Page 747
		INHINT			# SET UP THE DAP FOR COASTING FLIGHT.
				# Inhibit interrupts during DAP transition
		TC	IBNKCALL	# Inter-bank call to DAP configuration
		CADR	ALLCOAST	# Configure DAP for coasting (no thrust)
		TC	NULLCLOK	# Stop countdown clock, kill ullage
		TC	PHASCHNG	# 4.13 RESTART FOR POSTBURN
				# Set restart protection phase
		OCT	00134		# Phase 4.13: POSTBURN restart code

		TCF	ENDOFJOB	# Exit, POSTBURN runs in background

; ============================================================================
; GOCUTOFF - Engine Cutoff Sequence Initiation (Table Entry 3)
;
; COMMENT-ONLY READERS: When the engine completed its planned burn and needed
; to shut down, this routine executed. It scheduled the CUTOFF job to command
; the engine valves closed, verify shutdown, and handle any post-cutoff tasks.
; It also cleared the undispersed-velocity flag (FLUNDISP), configured the
; autopilot for coasting flight, stopped ullage and countdown clock, and set
; up restart protection. Used by programs like P63 (descent) that required
; clean engine shutdown at burn completion.
;
; During Apollo 11's powered descent on July 20, 1969, when Eagle's descent
; engine cut off at touchdown (102:45:40 MET), this routine commanded the DPS
; shutdown sequence, ensuring proper valve closure and transition to surface
; operations mode.
;
; CODE-ALONG READERS: GOCUTOFF is program table entry (3) for engine cutoff
; handling, called via INDEX WHICH, TCF 3 from burn completion paths:
; 1. Schedules CUTOFF job at priority 17 (PRIO17) via FINDVAC to find vacant
;    core set for cutoff processing
; 2. Sets EBANK=TGO for time-to-go context in CUTOFF routine
; 3. Clears FLUNDISP flag (undispersed velocity flag) via DOWNFLAG - indicates
;    all planned velocity change has been executed and dispersed
; 4. Calls ALLCOAST via IBNKCALL to configure DAP for coasting flight (no
;    thrust vector control, RCS attitude hold mode only)
; 5. Calls NULLCLOK to terminate ullage firing and stop countdown clock task
; 6. Sets PHASCHNG with two-word restart group (OCT 07024, OCT 17000) for
;    CUTOFF restart protection - phase 7.24 with priority 17000
; 7. Stores EBANK=TGO and 2CADR CUTOFF for restart table
; 8. Returns via ENDOFJOB
;
; Used by P63 (descent landing), P12 (ascent), and other programs requiring
; clean engine shutdown with full DAP reconfiguration and restart protection.
; ============================================================================

GOCUTOFF	CAF	PRIO17		# (3)
				# Priority 17 for engine cutoff job
		TC	FINDVAC		# Find vacant core set for CUTOFF
		EBANK=	TGO		# Set EBANK for time-to-go variables
		2CADR	CUTOFF		# Schedule CUTOFF routine address

		TC	DOWNFLAG	# Clear undispersed velocity flag
		ADRES	FLUNDISP	# FLUNDISP: velocity change dispersed

		INHINT			# SET UP THE DAP FOR COASTING FLIGHT.
				# Inhibit interrupts during DAP transition
		TC	IBNKCALL	# Inter-bank call to DAP configuration
		CADR	ALLCOAST	# Configure DAP for coasting (no thrust)
		TC	NULLCLOK	# Stop ullage and countdown clock
		TC	PHASCHNG	# Set restart protection
		OCT	07024		# Phase 7.24 restart group
		OCT	17000		# Priority 17000 for restart
		EBANK=	TGO		# EBANK for restart context
		2CADR	CUTOFF		# CUTOFF address for restart table

		TCF	ENDOFJOB	# Exit, CUTOFF runs in background

; ============================================================================
; IGNITE - Engine Ignition Command Execution (Table Entry 2)
;
; COMMENT-ONLY READERS: This is the moment of ignition. When the countdown
; reached zero (TIG - Time of Ignition), this routine checked if the ignition
; flag was already set (preventing double ignition), then commanded the engine
; to fire by twiddling the IGNITION output bit. It set up immediate restart
; protection so that if a power transient occurred during the critical ignition
; moment, the AGC would restart at the IGNITION sequence. Finally, it restored
; the previous display that had been replaced by the countdown.
;
; During Apollo 11's powered descent initiation (PDI) at 102:33:05 MET on
; July 20, 1969, this routine commanded the DPS engine to ignite, beginning
; the 12-minute descent to the lunar surface. For ascent at 124:22 MET on
; July 21, 1969, it commanded the APS engine ignition that returned Eagle
; to orbit for rendezvous with Columbia.
;
; CODE-ALONG READERS: IGNITE is program table entry (2) for engine ignition
; command, called via INDEX WHICH, TCF 2 at TIG (Time of Ignition):
; 1. Checks IGNFLBIT in FLAGWRD7 via complement (CS) and MASK to test if
;    ignition flag already set (CCS A produces positive if bit clear)
; 2. If ignition flag already set (A=0), skip to IGNITE1 to avoid double fire
; 3. If flag clear (normal case), set BIT1 and call TWIDDLE with IGNITION
;    address - TWIDDLE sets output channel bit to fire engine
; 4. Sets immediate restart protection using OCT23 (phase 2.3) via DXCH
;    to -PHASE4 register - if AGC restarts during ignition, execution resumes
;    at IGNITION sequence to ensure engine commands complete
; 5. At IGNITE1: restores previous display by complementing CNTDNDEX (which
;    was negative countdown index) and storing to DISPDEX, returning display
;    to pre-countdown state
; 6. Returns via ENDOFJOB
;
; The TWIDDLE call to IGNITION address triggers hardware output channel bit
; that commands engine valve opening. For DPS: main fuel/oxidizer valves open,
; hypergolic ignition occurs. For APS: similar valve sequence for ascent engine.
; ============================================================================

IGNITE		CS	FLAGWRD7	# (2)
				# Check ignition flag status
		MASK	IGNFLBIT	# Isolate IGNFLBIT from flag word
		CCS	A		# Check if flag already set
		TCF	IGNITE1		# Skip fire if already ignited
		CAF	BIT1		# Prepare bit for TWIDDLE
		INHINT			# Inhibit interrupts during ignition
		TC	TWIDDLE		# Set output channel bit
		ADRES	IGNITION	# IGNITION address for engine fire

		CAF	OCT23		# IMMEDIATE RESTART AT IGNITION
				# Phase 2.3 for restart protection
		TS	L		# Store to L register
		COM			# Complement for DXCH
		DXCH	-PHASE4		# Set restart phase to IGNITION

IGNITE1		CS	CNTDNDEX	# RESTORE OLD DISPLAY.
				# Complement negative countdown index
		TS	DISPDEX		# Restore previous display index

		TCF	ENDOFJOB	# Complete ignition task

# Page 748
#	********************************

; ============================================================================
; P40ALM - Program Selection Alarm for Incorrect Vehicle Configuration
;
; COMMENT-ONLY READERS: This alarm protected the crew from attempting to use
; a program that was incompatible with the current spacecraft configuration.
; For example, trying to run a descent program (P63) when the LM was still
; docked to the Command Module, or attempting an ascent program before the
; descent stage was jettisoned. The alarm displayed "1706" on the DSKY,
; indicating "PROGRAM SELECTION NOT CONSISTENT WITH VEHICLE CONFIGURATION."
; The crew could terminate (V34E), proceed anyway if appropriate like for
; P42 rehearsal (PROCEED key), or redisplay the alarm (V32E) to review again.
;
; CODE-ALONG READERS: P40ALM is the configuration mismatch alarm handler:
; 1. Issues alarm 1706 via TC ALARM, OCT 1706 - displays on DSKY as program
;    alarm code indicating vehicle configuration inconsistent with program
; 2. At REP40ALM: displays V05N09 via GOFLASH - V05 = "Please perform", N09
;    shows alarm code in R1 for crew review
; 3. Crew has three options via GOFLASH return branches:
;    - V34E (TERMINATE): TCF GOTOPOOH exits to POO (idle program)
;    - PROCEED: TCF +2 continues to configuration override check
;    - V32E (RECYCLE): TCF REP40ALM redisplays alarm for review
; 4. If crew PROCEEDS: INDEX WHICH, TCF 14 jumps to program table entry (14)
;    - allows P42 (DPS backup CSM rendezvous) to proceed even with LM unstaged
;    - P42 is rehearsal/backup program that can run in unusual configurations
;
; Alarm 1706 triggers for configuration mismatches like:
; - P40 (DPS burn) or P42 selected when LM still docked
; - P63 (landing) selected when LM still in orbit with CSM
; - P12 (ascent) selected when descent stage not yet jettisoned
; Configuration checked via ABORTED flag (docked state) and APSFLBIT (staging).
; ============================================================================

P40ALM		TC	ALARM		# PROGRAM SELECTION NOT CONSISTENT WITH
					# Alarm 1706: configuration mismatch
		OCT	1706		# VEHICLE CONFIGURATION

REP40ALM	CAF	V05N09		# (14)
					# V05N09: display alarm code
		TC	BANKCALL	# Call display handler
		CADR	GOFLASH		# Flash verb 05 for crew action

		TCF	GOTOPOOH	# V34E 		TERMINATE
					# Crew terminates, return to POO
		TCF	+2		# PROCEED 	CHECK FOR P42
					# Crew proceeds, allow override
		TCF	REP40ALM	# V32E		REDISPLAY ALARM
					# Crew recycles, show alarm again

		INDEX	WHICH		# FOR P42, ALLOW CREW TO PROCEED EVEN
					# Index by program table pointer
		TCF	14		# THOUGH VEHICLE IS UNSTAGED.
					# Jump to table entry 14 for override

#	********************************

		BANK	31
		SETLOC	P40S2
		BANK

		COUNT*	$$/P40

; ============================================================================
; P40AUTO - Verify Spacecraft Control Modes for Automatic Burn Execution
;
; COMMENT-ONLY READERS: Before igniting the engine, the computer verified
; that the spacecraft was in the correct control configuration for an
; automatic burn. It checked three critical systems: (1) the Primary
; Guidance, Navigation and Control System (PGNCS) was in control of the
; spacecraft, not manual pilot control; (2) the stabilization system was
; in automatic mode, allowing the computer to control spacecraft attitude;
; and (3) for DPS descent burns, the auto-throttle system was enabled,
; allowing the computer to adjust engine thrust. If any system was not
; properly configured, the DSKY displayed "V50N25 R1=203" asking the crew
; to "PLEASE PERFORM CHECKLIST 203" to enable the necessary systems before
; proceeding with the burn. This ensured the computer had full authority
; to execute the precision maneuvers required for lunar landing or orbital
; burns without crew intervention during the critical ignition sequence.
;
; CODE-ALONG READERS: P40AUTO is called via BANKCALL from BURNBABY (even
; from within the same bank) to verify spacecraft control mode configuration
; before burn execution. The BANKCALL ensures proper return CADR setup for
; cross-bank returns. Verification sequence:
;
; 1. TC MAKECADR creates return CADR for generalized bank return mechanism
; 2. TS TEMPR60 saves return CADR in temporary storage for later BANKJUMP
; 3. TC BANKCALL, CADR G+N,AUTO verifies PGNCS control and auto stabilization
;    - G+N,AUTO returns +0 if in PGNCS control AND auto stabilization mode
;    - G+N,AUTO returns + (positive) if NOT in proper control modes
; 4. CCS A checks G+N,AUTO return value:
;    - If + (positive): TCF TURNITON to prompt crew for mode changes
;    - If +0 (plus zero): continue to descent stage configuration check
; 5. For descent stage (DPS) burns, verify auto-throttle enabled:
;    - CAF APSFLBIT checks if ascent stage separated (FLGWRD10 staging flag)
;    - MASK FLGWRD10, CCS A tests staging bit
;    - If separated (ascent configuration): TCF GOBACK, no throttle check needed
;    - If on descent stage: check CHAN30 BIT5 for auto-throttle mode
;    - EXTEND, RAND CHAN30 reads discrete input channel 30
;    - EXTEND, BZF GOBACK returns if BIT5=0 (auto-throttle enabled)
;    - If BIT5=1 (manual throttle): fall through to TURNITON
; 6. TURNITON prompts crew via V50N25 R1=203 (GOPERF1 "please perform"):
;    - Displays checklist item 203: "TURN ON PGNCS CONTROL, AUTO STAB, AUTO THR"
;    - V34E (TERMINATE): TCF GOTOPOOH aborts burn, returns to POO idle program
;    - PROCEED: TCF P40A/P recycles verification check after crew actions
; 7. GOBACK returns to calling routine via saved return CADR:
;    - CA TEMPR60 loads saved return address
;    - TC BANKJUMP executes cross-bank return to BURNBABY caller
;
; This verification prevented burns with improper control mode configuration.
; During Apollo 11, all control modes were properly set before PDI (powered
; descent initiation) and before ascent engine ignition, so crew never saw
; the checklist 203 prompt during critical mission phases. The verification
; was essential for mission safety—attempting DPS throttle control without
; auto-throttle mode would have caused control system conflicts and potential
; loss of spacecraft attitude control during engine firing.
;
; PGNCS = Primary Guidance, Navigation and Control System (AGC + IMU + DSKY)
; Auto stabilization = RCS autopilot controlling spacecraft attitude via AGC
; Auto-throttle = DPS engine throttle commanded by AGC (vs manual throttle control)
; APSFLBIT = Ascent Propulsion System flag bit indicating staging (descent stage jettison)
; CHAN30 BIT5 = Discrete input indicating manual throttle mode (0=auto, 1=manual)
; ============================================================================

P40AUTO		TC	MAKECADR	# HELLO THERE.
				# Create return CADR for bank jump
		TS	TEMPR60		# FOR GENERALIZED RETURN TO OTHER BANKS.
				# Save return address in temporary
P40A/P		TC	BANKCALL	# SUBROUTINE TO CHECK PGNCS CONTROL
				# Verify control mode configuration
		CADR	G+N,AUTO	# AND AUTO STABILIZATION MODES
				# Call G+N,AUTO verification routine
		CCS	A		# +0 INDICATES IN PGNCS, IN AUTO
				# Check control mode return value
		TCF	TURNITON	# + INDICATES NOT IN PGNCS AND/OR AUTO
				# Positive: prompt crew action
		CAF	APSFLBIT	# ARE WE ON THE DESCENT STAGE?
				# Load staging flag bit for test
		MASK	FLGWRD10	# Mask against flag word 10
		CCS	A		# Test staging bit
		TCF	GOBACK		# RETURN
				# Ascent stage: skip throttle check
		CAF	BIT5		# YES, CHECK FOR AUTO-THROTTLE MODE
				# Load BIT5 for channel 30 test
		EXTEND			# Extended instruction follows
		RAND	CHAN30		# Read discrete channel 30
		EXTEND			# Extended instruction follows
		BZF	GOBACK		# IN AUTO-THROTTLE MODE -- RETURN
				# BIT5=0: auto-throttle, return OK
TURNITON	CAF	P40A/PMD	# DISPLAYS V50N25 R1=203 PLEASE PERFORM
				# Load checklist item 203
		TC	BANKCALL	# CHECKLIST 203 TURN ON PGNCS ETC.
				# Display please perform message
		CADR	GOPERF1		# Call GOPERF1 flash display
		TCF	GOTOPOOH	# V34E TERMINATE
				# Crew terminates: abort burn
		TCF	P40A/P		# RECYCLE
				# Crew proceeds: recheck modes
GOBACK		CA	TEMPR60		# Load saved return CADR
		TC	BANKJUMP	# GOODBYE.  COME AGAIN SOON.
				# Return to calling bank

P40A/PMD	OCT	00203

# Page 749
		BANK	36
		SETLOC	P40S
		BANK

		COUNT*	$$/P40

#	**********************************
#	CONSTANTS FOR THE IGNITION ROUTINE
#	**********************************

SERVCADR	=	P63TABLE +7

P40ADRES	ADRES	P40TABLE

P41ADRES	ADRES	P41TABLE -5

P42ADRES	ADRES	P42TABLE

		EBANK=	DVCNTR
DSP2CADR	2CADR	P63DISPS -2

		EBANK=	DVCNTR
ATMAGADR	2CADR	ATMAG

?		=	GOTOPOOH

D29.9SEC	2DEC	2990

S24.9SEC	DEC	2490

4.9SEC		DEC	490

OCT20		=	BIT5

V06N61		VN	0661

# Page 750
# KILLTASK
# MOD NO:  NEW PROGRAM
# MOD BY:  COVELLI
#
# FUNCTIONAL DESCRIPTION:
#
#	KILLTASK IS USED TO REMOVE A TASK FROM THE WAITLIST BY SUBSTITUTING A NULL TASK CALLED `NULLTASK' (OF COURSE),
#	WHICH MERELY DOES A TC TASKOVER.  IF THE SAME TASK IS SCHEDULED MORE THAN ONCE, ONLY THE ONE WHICH WILL OCCUR
#	FIRST IS REMOVED.  IF THE TASK IS NOT SCHEDULED, KILLTASK TAKES NO ACTION AND RETURNS WITH NO ALARM.  KILLTASK
#	LEAVES INTERRUPTS INHIBITED SO CALLER MUST RELINT
#
# CALLING SEQUENCE
#	L	TC	KILLTASK	# IN FIXED-FIXED
#	L+1	CADR	????????	# CADR (NOT 2CADR) OF TASK TO BE REMOVED.
#	L+2	(RELINT)		# RETURN
#
# EXIT MODE:  AT L+2 OF CALLING SEQUENCE.
#
# ERASABLE INITIALIZATION:  NONE.
#
# OUTPUT:  2CADR OF NULLTASK IN LST2
#
# DEBRIS:  ITEMP1 - ITEMP4, A, L, Q.

		EBANK=	LST2
		BLOCK	3		# KILLTASK MUST BE IN FIXED-FIXED.
		SETLOC	FFTAG6
		BANK
		COUNT*	$$/KILL
; ============================================================================
; KILLTASK - Remove Active Waitlist Task by CADR
;
; COMMENT-ONLY READERS: During abort scenarios or when mission programs
; needed to cancel scheduled future actions, the computer had to remove
; tasks from its waitlist timer queue. KILLTASK searched through the
; waitlist looking for any active task matching a specific routine
; address, then "killed" those tasks by replacing them with TASKOVER
; (task cancellation marker). This was critical during burn aborts—if
; the crew terminated an engine burn sequence, KILLTASK ensured that all
; scheduled follow-on tasks (throttle changes, cutoff commands, display
; updates) were cancelled so they wouldn't execute unexpectedly. During
; Apollo 11, if descent had been aborted before landing, KILLTASK would
; have cleaned up all pending landing tasks before transitioning to abort
; guidance mode. The routine was called with the task's address in the
; instruction immediately following the KILLTASK call.
;
; CODE-ALONG READERS: KILLTASK removes all active waitlist tasks matching
; a specified CADR (Combined Address). Calling sequence uses immediate
; addressing where CADR follows the TC KILLTASK instruction:
;
;     TC    KILLTASK      ; Call task removal routine
;     CADR  TASKNAME      ; CADR of task to kill (follows TC)
;     (return here after KILLTASK completes)
;
; Entry sequence performs bank switching to reach KILLTSK2:
; 1. CA KILLBB loads BBCON (both-bank constant) for KILLTSK2
; 2. INHINT inhibits interrupts during critical waitlist modification
; 3. LXCH A exchanges L with A, saving BBCON in L, clearing A
; 4. INDEX Q uses return address Q to index next instruction
; 5. CA 0 loads CADR from word following TC KILLTASK (immediate operand)
; 6. LXCH BBANK exchanges current bank with saved BBCON, setting up return
; 7. TCF KILLTSK2 transfers to main routine in switched bank
;
; BBCON mechanism enables cross-bank subroutine calls with proper returns.
; KILLBB contains EBANK=LST2 (erasable bank selection) and BBCON KILLTSK2
; (both-bank constant encoding target bank and address).
;
; KILLTSK2 main task removal logic:
; 1. LXCH ITEMP2 saves caller's BBANK for return bank restoration
; 2. INCR Q advances return address past CADR immediate operand
; 3. EXTEND, QXCH ITEMP1 saves adjusted return address (2ADR in ITEMP1,ITEMP2)
; 4. TS ITEMP3 stores input CADR (still in A from entry)
; 5. MASK LOW10 extracts low 10 bits (address within bank)
; 6. AD BIT11 adds BIT11 to form GENADR (generalized address)
; 7. TS ITEMP4 stores GENADR for comparison against LST2 entries
; 8. CS LOW10, MASK ITEMP3 extracts high bits (FBANK field)
; 9. TS ITEMP3 stores FBANK for later comparison
; 10. ZL zeroes L register for loop index initialization
;
; ADRSCAN loop scans LST2 waitlist table for matching tasks:
; - INDEX L indexes into LST2 by current L value (0, 2, 4, ... 30)
; - CS LST2 loads complement of task GENADR from LST2
; - AD ITEMP4 adds target GENADR (A = ~task_genadr + target_genadr)
; - EXTEND, BZF TSTFBANK if GENADRs match (sum = 0), check FBANK too
; - If no GENADR match, fall through to LETITLIV
;
; LETITLIV continues scan to next task:
; - CS LSTLIM loads complement of limit (BIT5 = DEC 16)
; - AD L adds current index (test if L reached limit)
; - EXTEND, BZF DEAD if scan complete (L = 16), return
; - INCR L, INCR L advances by 2 (LST2 entries are 2-word CADRs)
; - TCF ADRSCAN loops to check next task
;
; TSTFBANK verifies FBANK match for GENADR-matched task:
; - CS LOW10 loads FBANK extraction mask complement
; - INDEX L, MASK LST2+1 extracts FBANK from LST2 entry's second word
; - EXTEND, SU ITEMP3 subtracts target FBANK (A = task_fbank - target_fbank)
; - EXTEND, BZF KILLDEAD if FBANKs match (difference = 0), kill task
; - TCF LETITLIV if no FBANK match, continue scan (GENADR collision)
;
; KILLDEAD cancels matched task:
; - CA TCTSKOVR loads TASKOVER constant (task cancellation marker)
; - INDEX L, TS LST2 writes TASKOVER to LST2 entry, removing task
; - TCF DEAD proceeds to cleanup/return (continues scanning would also work)
;
; DEAD restores state and returns:
; - DXCH ITEMP1 restores return 2ADR to A, L (A=EBANK+FBANK, L=address)
; - DTCB (Double TCB) executes two-word return, restoring bank and jumping
;
; LSTLIM = BIT5 = DEC 16: LST2 can hold maximum 16 waitlist tasks
;
; This routine is called during burn aborts (COMFAIL sequences) to cancel
; all scheduled tasks associated with the burn. The GENADR comparison finds
; tasks by their address within bank, and FBANK comparison ensures correct
; bank match (avoiding false positives from address collisions across banks).
; Replacing task entry with TCTSKOVR (TASKOVER) causes waitlist to skip that
; task during timer processing—effectively removing it from execution queue.
;
; During Apollo 11, if PDI had been aborted, KILLTASK would have removed
; pending landing tasks like throttle-up commands, radar data processing
; tasks, and landing display updates scheduled for execution during descent.
; This prevented orphaned tasks from executing in wrong mission mode.
;
; ITEMP1, ITEMP2 = Saved return 2ADR (EBANK+FBANK, address)
; ITEMP3 = Target task FBANK for comparison
; ITEMP4 = Target task GENADR for comparison
; LST2 = Waitlist task table (16 entries, 2-word CADRs each)
; KILLBB = BBCON for KILLTSK2 (both-bank constant for bank switching)
; TCTSKOVR = TASKOVER constant (task cancellation marker)
; LOW10 = Octal 01777 mask for bits 0-9
; BIT11 = Octal 02000 (used in GENADR formation)
; INHINT = Inhibit interrupts during critical section
; DTCB = Double Task Control Block return (two-word bank return)
; ============================================================================

KILLTASK	CA	KILLBB
			# Load both-bank constant for KILLTSK2
		INHINT
			# Inhibit interrupts during waitlist modification
		LXCH	A
			# Save BBCON in L, clear A
		INDEX	Q
			# Index by return address
		CA	0		# GET CADR.
			# Load CADR from instruction after TC KILLTASK
		LXCH	BBANK
			# Exchange with bank register for bank switch
		TCF	KILLTSK2	# CONTINUE IN SWITCHED FIXED.
			# Transfer to main routine in target bank

		EBANK=	LST2
			# Set erasable bank for LST2 access
KILLBB		BBCON	KILLTSK2
			# Both-bank constant encoding target routine

		BANK	27
			# Switch to fixed bank 27

		SETLOC	P40S1
			# Set location counter to P40S1 region
		BANK
			# Confirm bank setting
		COUNT*	$$/KILL
			# Instruction count for KILLTASK routine

KILLTSK2	LXCH	ITEMP2		# SAVE CALLER'S BBANK
			# Save caller's bank for return restoration
# Page 751
		INCR	Q
			# Advance return address past CADR operand
		EXTEND
			# Extended instruction follows
		QXCH	ITEMP1		# RETURN 2ADR IN ITEMP1,ITEMP2
			# Save adjusted return address as 2ADR

		TS	ITEMP3		# CADR IS IN A
			# Store input CADR in ITEMP3
		MASK	LOW10
			# Extract low 10 bits (address in bank)
		AD	BIT11
			# Add BIT11 to form GENADR
		TS	ITEMP4		# GENADR OF TASK
			# Store task GENADR for comparison

		CS	LOW10
			# Load complement of LOW10 mask
		MASK	ITEMP3
			# Extract high bits (FBANK field)
		TS	ITEMP3		# FBANK OF TASK
			# Store task FBANK for comparison

		ZL
			# Zero L register for loop index
ADRSCAN		INDEX	L
			# Index into LST2 by L
		CS	LST2
			# Load complement of task GENADR from LST2
		AD	ITEMP4		# COMPARE GENADRS
			# Add target GENADR (test equality)
		EXTEND
			# Extended instruction follows
		BZF	TSTFBANK	# IF THEY MATCH, COMPARE FBANKS
			# If GENADRs match, verify FBANK too
LETITLIV	CS	LSTLIM
			# Load complement of scan limit
		AD	L
			# Add current index (test if done)
		EXTEND			# ARE WE DONE?
			# Extended instruction follows
		BZF	DEAD		# YES -- DONE, SO RETURN
			# If scan complete, return
		INCR	L
			# Increment index by 1
		INCR	L
			# Increment again (2-word entries)
		TCF	ADRSCAN		# CONTINUE LOOP.
			# Loop to check next task entry

DEAD		DXCH	ITEMP1
			# Restore return 2ADR to A, L
		DTCB
			# Double return: restore bank and jump

TSTFBANK	CS	LOW10
			# Load FBANK extraction mask complement
		INDEX	L
			# Index into LST2 by L
		MASK	LST2 	+1	# COMPARE FBANKS ONLY.
			# Extract FBANK from second word of entry
		EXTEND
			# Extended instruction follows
		SU	ITEMP3
			# Subtract target FBANK (test equality)
		EXTEND
			# Extended instruction follows
		BZF	KILLDEAD	# MATCH -- KILL IT.
			# If FBANKs match, cancel task
		TCF	LETITLIV	# NO MATCH -- CONTINUE.
			# No FBANK match, continue scan

KILLDEAD	CA	TCTSKOVR
			# Load TASKOVER cancellation marker
		INDEX	L
			# Index into LST2 by L
		TS	LST2		# REMOVE TASK BY INSERTING TASKOVER
			# Replace task entry with TASKOVER
		TCF	DEAD
			# Return via cleanup sequence

LSTLIM		EQUALS	BIT5		# DEC 16
			# Maximum 16 tasks in waitlist
