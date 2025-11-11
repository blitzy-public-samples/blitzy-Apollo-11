# Copyright:	Public domain.
# Filename:	RESTART_TABLES.agc
# Purpose:	Part of the source code for Comanche, build 055. It
#		is part of the source code for the Command Module's
#		(CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 211-221
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	2009-05-16 FB	Transcription Batch 2 Assignment.
#		2009-05-20 RSB	Added a missing comment mark.  Corrected mismarked
#				Page 217 -> 220.
#		2009-05-21 RSB	Fixed value of 5.21SPOT.
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  April 1, 1969.
#
#	This AGC program shall also be referred to as Colossus 2A
#
#	Prepared by
#			Massachusetts Institute of Technology
#			75 Cambridge Parkway
#			Cambridge, Massachusetts
#
#	under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: RESTART_TABLES.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Restart protection table structures encoding task phase and state
;        restoration data. Enables AGC to recover from power transients or
;        computational overloads by restoring interrupted tasks to correct
;        execution points. Critical during Apollo 11's 1202 program alarms
;        when restart protection allowed landing to continue safely.
;
; COMMENT-ONLY READERS: These tables told the computer how to resume interrupted
;        work, essential during the famous 1202 alarms.
; CODE-ALONG READERS: Study restart protection philosophy, table organization,
;        task phase encoding, state restoration mechanisms, 1202 alarm recovery context.
; ============================================================================

# Page 211
# RESTART TABLES
# --------------
;
; ============================================================================
; RESTART PROTECTION PHILOSOPHY
;
; The AGC's restart protection system enables the computer to recover from
; power transients, memory parity errors, or computational overloads without
; losing mission-critical work. When a restart occurs, these tables tell the
; computer which tasks were in progress and how to resume them safely.
;
; During Apollo 11's powered descent on July 20, 1969, the AGC experienced
; multiple 1202 program alarms caused by radar data overloading the executive
; scheduler. The restart protection system preserved the guidance computation
; state, allowing the landing to continue. Flight controller Steve Bales made
; the critical "Go" decision because he knew the restart system could recover.
;
; RESTART GROUPS AND PHASES:
; Tasks are organized into restart groups (numbered 1-6), with each group
; containing multiple phases. A "phase" represents a specific point in a
; task's execution sequence. When a restart occurs, the AGC reads the current
; group and phase numbers, then uses these tables to restore the interrupted
; work to the correct execution point.
; ============================================================================
#
# THERE ARE TWO FORMS OF RESTART TABLES FOR EACH GROUP.  THEY ARE KNOWN AS THE EVEN RESTART TABLES AND THE ODD
# RESTART TABLES.  THE ODD TABLES HAVE ONLY ONE ENTRY OF THREE LOCATIONS WHILE THE EVEN TABLES HAVE TWO ENTRIES
# EACH USING THREE LOCATIONS. THE INFORMATION AS TO WHETHER IT IS A JOB, WAITLIST, OR A LONGCALL IS GIVEN BY THE
# WAY THINGS ARE PUT INTO THE TABLES.
;
; ============================================================================
; TABLE STRUCTURE AND ENTRY FORMATS
;
; Each restart table entry uses three memory locations to encode the task
; type (job, waitlist, or longcall), its priority or timing information, and
; its restart address. The AGC determines task type by examining how values
; are stored in these locations - positive vs negative encoding indicates
; different task types and parameters.
;
; EVEN vs ODD RESTART TABLES:
; - ODD phase numbers use tables with ONE three-location entry
; - EVEN phase numbers use tables with TWO three-location entries
; This structure allows efficient memory usage while providing sufficient
; restart detail for complex mission programs.
; ============================================================================
#
# A JOB HAS ITS PRIORITY STORED IN PRDTTAB OF THE CORRECT PHASE SPOT - A POSITIVE PRIORITY INDICATES A
# FINDVAC JOB, A NEGATIVE PRIORITY A NOVAC.  THE 2CADR OF THE JOB IS STORED IN THE CADRTAB.
# FOR EXAMPLE,
;
; JOB ENTRY FORMAT (FINDVAC - Background Task):
; Jobs are computational tasks that run when the AGC has idle time available.
; FINDVAC jobs search for an available core set (computational slot) and wait
; if none are free. Priority determines scheduling order (higher runs first).
;
#		5.7SPOT		OCT	23000
#				2CADR	SOMEJOB
;
; Positive priority (23000 octal = priority 23 decimal) indicates FINDVAC.
; On restart of group 5 phase 7, SOMEJOB resumes as background task.
;
# A RESTART OF GROUP 5 WITH PHASE SEVEN WOULD THEN CAUSE SOMEJOB TO BE RESTARTED AS A FINDVAC WITH PRIORITY 23.
#
#		5.5SPOT		OCT	-23000
#				2CADR	ANYJOB
;
; JOB ENTRY FORMAT (NOVAC - Immediate Task):
; Negative priority (-23000 octal) indicates NOVAC, which allocates a core
; set immediately without waiting. Used for time-critical restart tasks that
; must execute promptly to maintain mission timeline integrity.
;
# HERE A RESTART OF GROUP 5 WITH PHASE 7 WOULD CAUSE ANYJOB TO BE RESTARTED AS A NOVAC WITH PRIORITY 23.
;
; ============================================================================
; LONGCALL ENTRY FORMAT (Timer-Scheduled Task):
;
; Longcalls schedule tasks to execute after a specified time delay. The delta
; time value indicates how long to wait before execution. If a restart occurs,
; the AGC checks whether the scheduled time has passed - if so, it executes
; immediately; otherwise it reschedules for the remaining time.
; ============================================================================
# A LONGCALL HAS ITS GENADR OF ITS 2CADR STORED NEGATIVELY AND ITS BBCON STORED POSITIVELY.  IN ITS PRDTTAB IS
# PLACED THE LOCATION OF A DP REGISTER THAT CONTAINS THE DELTA TIME THAT LONGCALL HAD BEEN ORIGINALLY STARTED
# WITH.  EXAMPLE,
#
#		3.6SPOT		GENADR	DELTAT
#				-GENADR	LONGTASK
#				BBCON	LONGTASK
;
; DELTAT contains the time delay. Negative GENADR and positive BBCON identify
; this as a longcall entry. Bank/address information in BBCON handles memory
; banking if DELTAT resides in switched erasable memory.
;
#				OCT	31000
#				2CADR	JOBAGAIN
;
; Longcall phases can have multiple parts - after the timed task completes,
; a follow-on job can restart. Here JOBAGAIN restarts as priority 31 job.
;
# THIS WOULD START UP LONGTASK AT THE APPROPRIATE TIME, OR IMMEDIATELY IF THE TIME HAD ALREADY PASSED. IT SHOULD
# BE NOTED THAT IF DELTAT IS IN A SWITCHED E BANK, THIS INFORMATION SHOULD BE IN THE BBCON OF THE 2CADR OF THE
# TASK.  FROM ABOVE, WE SEE THAT THE SECOND PART OF THIS PHASE WOULD BE STARTED AS A JOB WITH A PRIORITY OF 31.
;
; ============================================================================
; WAITLIST ENTRY FORMAT (Timer-Interrupt Task):
;
; Waitlist tasks execute after a time delay, integrated with the AGC's T4RUPT
; timer interrupt system. These are higher-priority than longcalls and execute
; under interrupt context. Critical for time-sensitive operations like guidance
; updates during powered flight.
;
; During Apollo 11's descent, waitlist overload contributed to the 1202 alarms.
; The landing radar was generating waitlist tasks faster than they could complete,
; causing the executive scheduler to overflow. The restart protection preserved
; the guidance state through each restart cycle.
; ============================================================================
#
# WAITLIST CALLS ARE IDENTIFIED BY THE FACT THAT THEIR 2CADR IS STORED NEGATIVELY.  IF PRDTTAB OF THE PHASE SPOT
# IS POSITIVE, THEN IT CONTAINS THE DELTA TIME, IF PRDTTAB IS NEGATIVE THEN IT IS THE -GENADR OF AN ERASABLE
# LOCATION CONTAINING THE DELTA TIME, THAT IS, THE TIME IS STORED INDIRECTLY.  IT SHOULD BE NOTED AS ABOVE, THAT
# IF THE TIME IS STORED INDIRECTLY, THE BBCON MUST CONTAIN THE NECESSARY E BANK INFORMATION IF APPLICABLE.  WITH
# WAITLIST WE HAVE ONE FURTHER OPTION, IF -0 IS STORED IN PRDTTAB, IT WILL CAUSE AN IMMEDIATE RESTART OF THE
# TASK.  EXAMPLES,
;
; WAITLIST EXAMPLE 1 - Immediate Restart:
; OCT 77777 (negative zero) forces immediate task restart, bypassing timing.
; Used when a task must resume instantly after a restart to maintain critical
; mission timeline integrity.
;
#				OCT	77777		# THIS WILL CAUSE AN IMMEDIATE RESTART
#				-2CADR	ATASK		# OF THE TASK :ATASK:
;
; WAITLIST EXAMPLE 2 - Direct Time Specification:
; DEC 200 (200 centiseconds = 2 seconds) specifies delay directly in PRDTTAB.
; If the scheduled time has elapsed during restart, task begins in 10ms.
; Otherwise reschedules for remaining time.
;
#				DEC	200		# IF THE TIME OF THE 2 SECONDS SINCE DUMMY
#				-2CADR	DUMMY		# WAS PUT ON THE WAITLIST IS UP, IT WILL BEGIN
#							# IN 10 MS, OTHERWISE IT WILL BEGIN WHEN
#							# IT NORMALLY WOULD HAVE BEGUN.
# Page 212
;
; WAITLIST EXAMPLE 3 - Indirect Time Specification:
; Negative GENADR in PRDTTAB points to erasable memory location containing delta
; time. This allows dynamic time values that can be computed or updated. The BBCON
; must specify correct bank if DTIME resides in switched erasable memory.
;
#				-GENADR	DTIME		# WHERE DTIME CONTAINS THE DELTA TIME
#				-2CADR	TASKTASK	# OTHERWISE THIS IS AS ABOVE
#
# ***** NOW THE TABLES THEMSELVES *****
;
; ============================================================================
; TRANSITION: From Entry Format Specifications to Actual Restart Tables
;
; The specifications above define how restart table entries are structured.
; Below are the actual tables used by the AGC restart logic. The tables are
; organized into groups (1-6) with even/odd phase spots. SIZETAB provides
; offsets to locate specific restart group entries, while the numbered SPOT
; entries contain the actual restart data (priorities, addresses, delta times).
;
; During a restart, the AGC uses the current restart group number to index
; into these tables, retrieves the appropriate task information, and
; reconstructs the computational state to resume exactly where it left off.
; ============================================================================

		BANK	01
		SETLOC	RESTART
		BANK

		COUNT	01/RSTAB
;
; PRDTTAB: Base address for priority/delta-time table
; CADRTAB: Base address for 2CADR (two-word address) table
; These symbolic addresses anchor the restart table structure in memory.
;
PRDTTAB		EQUALS	12000			# USED TO FIND THE PRIORITY OR DELTATIME
CADRTAB		EQUALS	12001			# THIS AND THE NEXT RELATIVE LOC CONTAIN
						# RESTART 2CADR
;
; ============================================================================
; SIZETAB: Restart Table Index
;
; This table provides offset addresses to locate each restart group's data.
; The AGC uses the current restart group number (1-6) to index into SIZETAB,
; retrieving the address of the corresponding EVEN (x.2SPOT) or ODD (x.3SPOT)
; restart entry. The offset values (-12006, -12004) are relative to PRDTTAB
; base address, enabling efficient address calculation during restart.
;
; Each group has two entries: EVEN phases (x.2, x.4, x.6...) and ODD phases
; (x.3, x.5, x.7...). The restart logic determines which table to use based
; on the phase number's parity.
; ============================================================================
SIZETAB		TC	1.2SPOT -12006
		TC	1.3SPOT -12004
		TC	2.2SPOT -12006
		TC	2.3SPOT	-12004
		TC	3.2SPOT -12006
		TC	3.3SPOT -12004
		TC	4.2SPOT -12006
		TC	4.3SPOT -12004
		TC	5.2SPOT -12006
		TC	5.3SPOT -12004
		TC	6.2SPOT -12006
		TC	6.3SPOT -12004
;
; ============================================================================
; GROUP 1 RESTART ENTRIES
;
; Restart group 1 handles early mission phase tasks and system initialization.
; These entries ensure critical housekeeping tasks resume correctly after any
; restart condition (power transient, program alarm, manual crew restart).
; ============================================================================
;
; GROUP 1.EVEN (1.2SPOT):
; Currently aliased to GROUP 3.EVEN - no unique group 1 even-phase restarts.
;
1.2SPOT		EQUALS	3.2SPOT

# ANY MORE GROUP 1.EVEN RESTART VALUES SHOULD GO HERE
;
; GROUP 1.ODD ENTRIES:
;
; 1.3SPOT - SETJTAG Waitlist Task:
; Restarts the jet firing tag update task after 120 centiseconds (1.2 seconds).
; This task monitors and logs RCS thruster firing history for crew displays and
; telemetry. The JTAGTIME constant (120 cs) ensures periodic updates without
; excessive computational load.
;
1.3SPOT		DEC	120			# THIS NUMBER MUST BE EQUAL C(JTAGTIME)
		EBANK=	AOG
		-2CADR	SETJTAG
;
; 1.5SPOT - REDO40.9 Job (Priority 10):
; Restarts digital autopilot (DAP) reconfiguration job. Used during engine
; burns or major attitude maneuvers when DAP gains and control logic must
; be recalculated for changing spacecraft mass properties.
;
1.5SPOT		OCT	10000
		EBANK=	DAPDATR1
		2CADR	REDO40.9
;
; 1.7SPOT - RELINUS Job (Priority 10):
; Restarts the relative motion integration job used during rendezvous operations.
; Maintains continuous tracking of relative position/velocity between CSM and LM
; even through restart cycles.
;
1.7SPOT		OCT	10000
		EBANK=	ESTROKER
		2CADR	RELINUS
;
; 1.11SPOT - PIKUP20 Job (Priority 10):
; Restarts the P20 rendezvous navigation program pickup routine. Ensures
; rendezvous navigation continues seamlessly after restart, critical during
; time-sensitive orbital rendezvous maneuvers.
;
1.11SPOT	OCT	10000
		EBANK=	ESTROKER
		2CADR	PIKUP20

# ANY MORE GROUP 1.ODD RESTART VALUES SHOULD GO HERE
;
; ============================================================================
; GROUP 2 RESTART ENTRIES
;
; Restart group 2 handles navigation state integration and orbital tracking.
; These entries ensure continuous navigation updates during coast phases and
; rendezvous operations, critical for maintaining accurate position/velocity
; knowledge throughout the mission.
; ============================================================================
;
; GROUP 2.EVEN (2.2SPOT):
; Currently aliased to GROUP 1.EVEN - no unique group 2 even-phase restarts.
;
2.2SPOT		EQUALS	1.2SPOT

# ANY MORE GROUP 2.EVEN RESTART VALUES SHOULD GO HERE
# Page 213
;
; GROUP 2.ODD ENTRIES:
;
; 2.3SPOT - STATEINT Longcall:
; Restarts the state vector integration task after 600 seconds (10 minutes).
; This routine propagates the spacecraft's position and velocity forward in time
; using orbital mechanics equations. The 600-second interval balances accuracy
; with computational load. Longcall format allows resumption at correct time.
;
2.3SPOT		GENADR	600SECS
		-GENADR	STATEINT
		EBANK=	RRECTCSM
		BBCON	STATEINT
;
; 2.5SPOT - STATINT1 Job (Priority 5):
; Restarts the state vector integration initialization job. Sets up coordinate
; frames and initial conditions for orbital integration. Priority 5 ensures
; this completes before time-critical guidance computations.
;
2.5SPOT		OCT	05000
		EBANK=	RRECTCSM
		2CADR	STATINT1
;
; 2.7SPOT - R22 Job (Priority 10):
; Restarts the R22 rendezvous tracking routine. Maintains continuous tracking
; of target spacecraft during rendezvous operations using optical sightings
; and/or radar data.
;
2.7SPOT		OCT	10000
		EBANK=	MRKBUF2
		2CADR	R22
;
; 2.11SPOT - V94ENTER Job (Priority 14):
; Restarts the Verb 94 crew entry routine for manual target identification.
; Allows crew to designate landmarks or targets for optical navigation tracking.
;
2.11SPOT	OCT	14000
		EBANK=	LANDMARK
		2CADR	V94ENTER
;
; 2.13SPOT - REDOR22 Job (Priority 10):
; Restarts the R22 redo/continuation routine. Handles resumption of optical
; tracking computations after interruption, maintaining tracking data continuity.
;
2.13SPOT	OCT	10000
		EBANK=	MRKBUF2
		2CADR	REDOR22

# ANY MORE GROUP 2.ODD RESTART VALUES SHOULD GO HERE
;
; ============================================================================
; GROUP 3 RESTART ENTRIES
;
; Restart group 3 handles powered flight operations including engine burns,
; thrust vector control, and orbital maneuvers. Critical during translunar
; injection (TLI), lunar orbit insertion (LOI), and transearth injection (TEI).
; ============================================================================
;
; GROUP 3.EVEN (3.2SPOT):
; Currently aliased to GROUP 4.EVEN - no unique group 3 even-phase restarts.
;
3.2SPOT		EQUALS	4.2SPOT

# ANY MORE GROUP 3.EVEN RESTART VALUES SHOULD GO HERE
;
; GROUP 3.ODD ENTRIES:
;
; 3.3SPOT - S40.13 Job (Priority 20):
; Restarts the Service Propulsion System (SPS) burn executive routine. Manages
; main engine firing sequences during major orbital maneuvers (TLI, LOI, TEI).
; High priority ensures burn control continues through any restart.
;
3.3SPOT		OCT	20000
		EBANK=	TGO
		2CADR	S40.13
;
; 3.5SPOT - Placeholder Entry:
; Reserved for future group 3 even restart values. Currently contains zero
; padding to maintain table structure alignment.
;
3.5SPOT		DEC	0
		DEC	0
		DEC	0
;
; 3.7SPOT - MATRXJOB Job (Priority 22):
; Restarts the matrix transformation job for coordinate frame conversions.
; Critical during powered flight to maintain correct thrust vector orientation
; relative to inertial reference frame.
;
3.7SPOT		OCT	22000
		EBANK=	TEPHEM
		2CADR	MATRXJOB
;
; 3.11SPOT - REP11 Job (Priority 22):
; Restarts the P11 earth orbit insertion monitoring program. Tracks orbital
; parameters during ascent to verify successful orbit achievement.
;
3.11SPOT	OCT	22000
		EBANK=	TEPHEM
		2CADR	REP11
;
; 3.13SPOT - REP11A Job (Priority 22):
; Restarts the P11 alternate monitoring routine. Provides backup orbital
; parameter tracking with alternate computational methods.
;
3.13SPOT	OCT	22000
		EBANK=	TEPHEM
		2CADR	REP11A
;
; 3.15SPOT - ENGINOFF Waitlist (Indirect Time):
; Restarts the engine cutoff task using time-to-go (TGO+1) as delta time.
; Ensures engine shuts down at correct moment even if restart occurs during
; burn. Indirect time reference allows dynamic cutoff time updates.
;
3.15SPOT	-GENADR	TGO +1
		EBANK=	TGO
		-2CADR	ENGINOFF

# Page 214
# ANY MORE GROUP 3.ODD RESTART VALUES SHOULD GO HERE
;
; ============================================================================
; GROUP 4 RESTART ENTRIES
;
; Restart group 4 handles burn preparation, thrust vector control (TVC), and
; digital autopilot (DAP) during powered flight. Ensures precise attitude
; control and engine gimbal management throughout critical burn sequences.
; ============================================================================
;
; GROUP 4.EVEN ENTRIES:
;
; 4.2SPOT - PRECHECK + P47BODY (Two-Phase Entry):
; Phase 1: PRECHECK Waitlist (Immediate restart, -0 = OCT 77777)
;   Restarts pre-ignition verification checks immediately upon restart.
; Phase 2: P47BODY Job (Priority 30)
;   Restarts the P47 body-axis burn program after precheck completes.
;   Coordinates attitude and engine control during SPS burns.
;
4.2SPOT		OCT	77777
		EBANK=	TIG
		-2CADR	PRECHECK

		OCT	30000
		EBANK=	DELVIMU
		2CADR	P47BODY
;
; 4.4SPOT - PRECHECK + TTG/0 (Two-Phase Entry):
; Phase 1: PRECHECK Waitlist (Immediate restart)
;   Restarts pre-ignition checks immediately.
; Phase 2: TTG/0 Waitlist (2996 centiseconds = ~30 seconds before ignition)
;   Restarts time-to-go display and final countdown sequence. Crew monitors
;   TTG display for burn timing awareness.
;
4.4SPOT		OCT	77777
		EBANK=	TIG
		-2CADR	PRECHECK

		DEC	2996
		EBANK=	DAPDATR1
		-2CADR	TTG/0
;
; 4.6SPOT - PRECHECK + TIG-5 (Two-Phase Entry):
; Phase 1: PRECHECK Waitlist (Immediate restart)
;   Restarts pre-ignition checks immediately.
; Phase 2: TIG-5 Waitlist (2496 centiseconds = ~25 seconds before ignition)
;   Restarts the TIG-5 (time-of-ignition minus 5 seconds) sequence. Final
;   attitude settling and engine arm commands occur at this point.
;
4.6SPOT		OCT	77777
		EBANK=	TIG
		-2CADR	PRECHECK

		DEC	2496
		EBANK=	TIG
		-2CADR	TIG-5

# ANY MORE GROUP 4.EVEN RESTART VALUES SHOULD GO HERE
;
; GROUP 4.ODD ENTRIES:
;
; 4.3SPOT - DOTVCON Waitlist (40 centiseconds = 0.4 seconds):
; Restarts the thrust vector control (TVC) update task. Continuously adjusts
; engine gimbal angles to maintain desired thrust direction through vehicle
; center of gravity. 40cs update rate ensures responsive control.
;
4.3SPOT		DEC	40
		EBANK=	PACTOFF
		-2CADR	DOTVCON
;
; 4.5SPOT - DOSTRULL Waitlist (160 centiseconds = 1.6 seconds):
; Restarts the steering update task for powered flight guidance. Computes
; desired thrust direction based on current trajectory and target orbit.
; 160cs update rate balances guidance accuracy with computational load.
;
4.5SPOT		DEC	160
		EBANK=	PACTOFF
		-2CADR	DOSTRULL
;
; 4.7SPOT - TIG-0 Waitlist (500 centiseconds = 5 seconds):
; Restarts the time-of-ignition (TIG-0) sequence. Final pre-ignition steps
; occur here: engine valve opening, thrust buildup monitoring, commit to burn.
;
4.7SPOT		DEC	500
		EBANK=	PACTOFF
		-2CADR	TIG-0
;
; 4.11SPOT - V97E40.6 Waitlist (250 centiseconds = 2.5 seconds):
; Restarts Verb 97 extended display routine during P40-series burn programs.
; Updates crew displays with burn status information.
;
4.11SPOT	DEC	250
		EBANK=	DAPDATR1
		-2CADR	V97E40.6
;
; 4.13SPOT - R40ENABL Waitlist (200 centiseconds = 2 seconds):
; Restarts the R40 enable task for RCS backup control during SPS burns.
; Ensures RCS jets ready for attitude control if SPS gimbal fails.
;
4.13SPOT	DEC	200
		EBANK=	WHOCARES
		-2CADR	R40ENABL
;
; 4.15SPOT - COMPVER Job (Priority 16):
; Restarts the prelaunch optical verification routine. During countdown,
; verifies optics alignment and star tracker calibration before launch.
;
4.15SPOT	OCT	16000			# PRELAUNCH OPTICAL VERIFICATION
		EBANK=	OGC
# Page 215
		2CADR	COMPVER			# CALLS FOR OPTICS DATA AGAIN (STD LEADIN)
;
; 4.17SPOT - AZMTHCG1 Job (Priority 16):
; Restarts the prelaunch azimuth change routine. Allows platform realignment
; to updated launch azimuth if launch delayed. Ensures IMU aligned to correct
; heading for ascent trajectory.
;
4.17SPOT	OCT	16000			#  PRELAUNCH AZIMUTH CHANGE
		EBANK=	XSM
		2CADR	AZMTHCG1
;
; 4.21SPOT - TIGBLNK Longcall:
; Restarts the time-of-ignition blanking task via longcall. TIGBLNK clears
; certain displays before burn. Uses P40TMP variable for delta time scheduling.
; Protects display state during P40/P41 SPS burn preparation.
;
4.21SPOT	GENADR	P40TMP			# DELTA TIME USED IN SETTING UP
		-GENADR	TIGBLNK			# LONG CALL OF TIGBLNK  BY P40,P41
		EBANK=	P40TMP
		BBCON	TIGBLNK
;
; 4.23SPOT - P40S/SV Job (Priority 12):
; Restarts the P40 state vector computation job. Calculates updated position
; and velocity vectors for burn targeting. Critical for accurate SPS burns.
;
4.23SPOT	OCT	12000			# PROTECT  P40S/SV BY P40 P41
		EBANK=	TIG
		2CADR	P40S/SV
;
; 4.25SPOT - PROG52 Job (Priority 24):
; Restarts Program 52 (IMU fine alignment using star sightings). Used during
; coast phases to maintain precise platform alignment for navigation accuracy.
;
4.25SPOT	OCT	24000
		EBANK=	BESTI
		2CADR	PROG52
;
; 4.27SPOT - DOTVCRCS Waitlist (250 centiseconds = 2.5 seconds):
; Restarts the TVC+RCS coordination task. Manages combined engine gimbal and
; reaction control system operation during powered flight. Ensures optimal
; attitude control using both systems.
;
4.27SPOT	DEC	250
		EBANK=	PACTOFF
		-2CADR	DOTVCRCS
;
; 4.31SPOT - R51 +1 Job (Priority 13):
; Restarts Routine 51 (IMU alignment calibration) at entry point +1. Used
; during alignment procedures to update platform orientation based on
; optical star sightings.
;
4.31SPOT	OCT	13000
		EBANK=	STAR
		2CADR	R51 +1
;
; 4.33SPOT - WAKEP62 Waitlist (2100 centiseconds = 21 seconds):
; Restarts the wake-up task for P62 (final approach program). On Apollo 11,
; P62 was bypassed and P63 (braking phase) went directly to P64 (approach).
; This protects the continuing job to start P63 if needed.
;
4.33SPOT	DEC	2100			# PROTECT CONTINUING JOB TO START P63
		EBANK=	AOG
		-2CADR	WAKEP62
;
; 4.35SPOT - POSTBURN Job (Priority 12):
; Restarts post-burn cleanup and state assessment job. After SPS shutdown,
; computes achieved orbit, evaluates burn success, prepares for next phase.
;
4.35SPOT	OCT	12000
		EBANK=	DAPDATR1
		2CADR	POSTBURN
;
; 4.37SPOT - TIGAVEG Waitlist (500 centiseconds = 5 seconds):
; Restarts time-of-ignition average-g computation. Monitors thrust buildup
; and acceleration during engine start sequence. Verifies proper ignition.
;
4.37SPOT	DEC	500
		EBANK=	TIG
		-2CADR	TIGAVEG
;
; 4.41SPOT - P67.1 Job (Priority 17):
; Restarts Program 67 final phase display job. P67 handles entry monitoring
; during atmospheric reentry. This protects critical display updates showing
; entry corridor and range prediction.
;
4.41SPOT	OCT	17000			# PROTECT DISPLAY JOB IN P67
		EBANK=	AOG
		2CADR	P67.1
;
; 4.43SPOT - S61.1C Waitlist (Indirect time via S61DT):
; Restarts the preread/entry task S61.1C. Uses delta time stored in S61DT
; variable. Prepares for atmospheric entry by initializing entry guidance
; and IMU alignment procedures.
;
4.43SPOT	-GENADR	S61DT			# PROTECT TASK TO START PREREAD,ENTRY
		EBANK=	S61DT			# S61.1C WILL CHANGE EBANK=EB7 FOR PREREAD
		-2CADR	S61.1C
;
; 4.45SPOT - S61.1A -1 Job (Priority 13):
; Restarts the continuing job for S61.1 entry IMU alignment. Ensures platform
; properly oriented for entry phase. Entry attitude critical for proper lift
; vector control during atmospheric deceleration.
;
4.45SPOT	OCT	13000			# PROTECT CONTINUING JOB S61.1
		EBANK=	AOG			# (ENTRY IMU ALIGNMENT)
# Page 216
		2CADR	S61.1A -1
;
; 4.47SPOT - PRE-HUNT Job (Priority 17):
; Restarts the huntest (hunt oscillation test) iteration job. Tests CMC
; autopilot damping characteristics. Ensures stable attitude control without
; excessive oscillation around desired attitude.
;
4.47SPOT	OCT	17000			# PROTECT HUNTEST ITERATION.
		EBANK=	AOG
		2CADR	PRE-HUNT
;
; 4.51SPOT - ATERTASK Waitlist (Immediate restart, -0 = OCT 77777):
; Restarts the FDAI (flight director attitude indicator) error display task
; immediately. During P11 (Earth orbit insertion monitoring), displays attitude
; errors to crew for manual monitoring during critical ascent phase.
;
4.51SPOT	OCT	77777			# PROTECT FDAI ATTITUDE
		EBANK=	BODY3			# ERROR DISPLAY IN P11
		-2CADR	ATERTASK
;
; 4.53SPOT - V97ETASK Waitlist (Immediate restart, DEC -0):
; Restarts Verb 97 engine-on display task immediately upon restart. Updates
; crew displays with engine status, thrust level, and burn progress during
; powered flight.
;
4.53SPOT	DEC	-0
		EBANK=	END-E7			# EBANK7 FOR TIG
		-2CADR	V97ETASK
;
; 4.55SPOT - P65.1 Job (Priority 13):
; Restarts Program 65 responsive display job. P65 (skip targeting for entry)
; continuously updates predicted landing point based on current trajectory.
; Crew monitors to verify entry corridor compliance.
;
4.55SPOT	OCT	13000			# PROTECT  P65 RESPONSIVE DISPLAY.
		EBANK=	RTINIT
		2CADR	P65.1
;
; 4.57SPOT - TIGON Waitlist (Indirect time via P40TMP):
; Restarts the time-of-ignition-on task using delta time from P40TMP variable.
; Triggers final ignition sequence at precisely scheduled moment. Timing
; critical for proper orbital insertion or maneuver execution.
;
4.57SPOT	-GENADR	P40TMP
		EBANK=	P40TMP
		-2CADR	TIGON
;
; 4.61SPOT - IGNITION Waitlist (Immediate restart):
; Restarts the ignition sequence task immediately upon restart. Commands
; engine valve opening and monitors thrust chamber pressure buildup. Critical
; for safe engine start.
;
4.61SPOT	OCT	77777
		EBANK=	PACTOFF
		-2CADR	IGNITION
;
; 4.63SPOT - DOSPSOFF Waitlist (Immediate restart):
; Restarts the SPS (Service Propulsion System) shutdown task immediately.
; Commands engine cutoff, closes valves, secures propulsion system. Executes
; when target velocity achieved or abort commanded.
;
4.63SPOT	OCT	77777
		EBANK=	PACTOFF
		-2CADR	DOSPSOFF
;
; 4.65SPOT - TIG-5 Waitlist (10 centiseconds = 0.1 seconds):
; Restarts TIG-5 sequence with very short delay (100ms). Final attitude
; settling and ullage motor firing occur here before main engine ignition.
;
4.65SPOT	DEC	10
		EBANK=	TIG
		-2CADR	TIG-5
;
; 4.67SPOT - V97TTASK Waitlist (Immediate restart, DEC -0):
; Restarts Verb 97 thrust display task immediately. Shows crew real-time
; thrust magnitude and direction during SPS burns. Critical for monitoring
; burn performance.
;
4.67SPOT	DEC	-0
		EBANK=	CSMMASS
		-2CADR	V97TTASK
;
; 4.71SPOT - V97TRCS Waitlist (250 centiseconds = 2.5 seconds):
; Restarts Verb 97 RCS (Reaction Control System) display task. Updates crew
; displays showing RCS jet firings and propellant usage during attitude control.
;
4.71SPOT	DEC	250
		EBANK=	DAPDATR1		# (FOR RCSDAPON)
		-2CADR	V97TRCS
;
; 4.73SPOT - V97PTASK Waitlist (Immediate restart, DEC -0):
; Restarts Verb 97 performance display task immediately. Shows burn efficiency,
; delta-V achieved, and propellant consumption during powered flight.
;
4.73SPOT	DEC	-0
		EBANK=	V97VCNTR
		-2CADR	V97PTASK
;
; 4.75SPOT - SPSOFF97 Waitlist (Immediate restart, DEC -0):
; Restarts the SPS shutdown display task (Verb 97 variant) immediately.
; Updates crew displays when engine cutoff occurs, showing final achieved
; velocity and burn results.
;
4.75SPOT	DEC	-0
		EBANK=	DAPDATR1
		-2CADR	SPSOFF97

# Page 217
;
; 4.77SPOT - TIG-0 Waitlist (Immediate restart, DEC -0):
; Restarts TIG-0 (time-of-ignition) task immediately. Final pre-ignition
; checks and engine arm commands execute here. Last opportunity for automatic
; abort before thrust buildup begins.
;
4.77SPOT	DEC	-0
		EBANK=	PACTOFF
		-2CADR	TIG-0

# ANY MORE GROUP 4.ODD RESTART VALUES SHOULD GO HERE

; ============================================================================
; TRANSITION: From Burn Operations (Group 4) to Navigation and Alignment (Group 5)
;
; Having protected all burn preparation, execution, and post-burn operations,
; the restart tables now shift focus to Group 5: extended mission programs
; for orbital navigation, IMU alignment sequences (P51/P52/P53), and long-
; duration computational tasks. These programs ran continuously between major
; maneuvers, maintaining accurate navigation state and platform alignment
; throughout Apollo 11's journey to and from the Moon.
; ============================================================================

; ============================================================================
; RESTART GROUP 5: ORBITAL DETERMINATION, IMU ALIGNMENT, AND MISSION PHASE
; ============================================================================
; GROUP 5 PROTECTED: Extended mission programs including IMU alignment
; sequences (P51/P52/P53), orbital integration tasks, navigation updates,
; and specialized mission phase operations. These routines ensured accurate
; navigation state and platform alignment during translunar coast, lunar
; orbit operations, and transearth return phases.
;
; COMMENT-ONLY READERS: This group protected alignment and navigation tasks
;        that kept the spacecraft precisely on course between Earth and Moon.
; CODE-ALONG READERS: Study how restart protection extended to long-running
;        orbital integration, platform alignment convergence, and periodic
;        navigation state updates requiring multi-minute execution times.
; ============================================================================

; Phase 5.2: Navigation normalization job (FINDVAC priority 32)
; Normalizes navigation state vectors after orbit integration or measurement
; incorporation. Ensures position/velocity maintain proper scaling and format.
5.2SPOT		OCT	32000			; FINDVAC job priority 32
		EBANK=	DVCNTR			; Navigation state bank
		2CADR	NORMLIZE		; Normalization routine

; Second entry of Phase 5.2: Reread average G task (200 centiseconds)
; Periodic task to reread and incorporate accelerometer data (average G)
; during coast phases. Updates navigation state with integrated acceleration.
		DEC	200			; Delta time 2 seconds (200 cs)
		EBANK=	AOG			; Average G storage bank
		-2CADR	REREADAC		; Reread accelerometer task

; Phase 5.4: Servicer routine job (FINDVAC priority 20)
; General-purpose servicer for Group 5 housekeeping operations. Handles
; periodic maintenance tasks during extended mission programs.
5.4SPOT		OCT	20000			; FINDVAC job priority 20
		EBANK=	DVCNTR			; Navigation counter bank
		2CADR	SERVICER		; Servicer routine

; Second entry of Phase 5.4: Reread average G task (200 centiseconds)
; Continuation of accelerometer integration during coast navigation.
		DEC	200			; Delta time 2 seconds
		EBANK=	AOG			; Average G bank
		-2CADR	REREADAC		; Reread accelerometer data

# ANY MORE GROUP 5.EVEN RESTART VALUES SHOULD GO HERE

; *** GROUP 5 ODD RESTART VALUES (Single Entry Per Phase) ***

; Phase 5.3: Reread average G task (200 centiseconds)
; Coast phase accelerometer monitoring during navigation updates.
5.3SPOT		DEC	200			; Delta time 2 seconds
		EBANK=	AOG			; Average G bank
		-2CADR	REREADAC		; Reread accelerometer

; Phase 5.5: Redo Phase 5.5 task (Immediate restart)
; Retry mechanism for Phase 5.5 operations. Immediate restart ensures rapid
; recovery from transient conditions during alignment or integration.
5.5SPOT		OCT	77777			; WAITLIST immediate restart
		EBANK=	AOG			; Average G bank
		-2CADR	REDO5.5			; Redo Phase 5.5 task

; Phase 5.7: Restart GTS routine (FINDVAC priority 20)
; Used by prelaunch and mission initialization. Restarts Guidance and
; Targeting System (GTS) initialization sequence.
5.7SPOT		OCT	20000			# USED BY PRELAUNCH
		EBANK=	XSM			; Extended state bank
		2CADR	RSTGTS1			; Restart GTS routine

; Phase 5.11: Alignment loop iteration (Immediate restart)
; IMU alignment convergence loop for P51/P52/P53 programs. Immediate restart
; ensures continuous alignment processing without timing gaps. During Apollo 11,
; platform alignments were performed before major maneuvers (LOI, TEI) to
; ensure navigation accuracy at critical mission phases.
5.11SPOT	OCT	77777			; WAITLIST immediate restart
		EBANK=	XSM			; Extended state bank
		-2CADR	ALLOOP1			; Alignment loop task

; Phase 5.13: Waitlist entry test (FINDVAC priority 20)
; Tests WAITLIST entry mechanisms during system verification and checkout.
5.13SPOT	OCT	20000			; FINDVAC job priority 20
		EBANK=	XSM			; Extended state bank
		2CADR	WTLISTNT		; Waitlist entry test

; Phase 5.15: Restart test routine 1 (FINDVAC priority 20)
; Self-test verification of restart protection mechanisms. Tests Group 5
; restart table entries and phase transitions.
5.15SPOT	OCT	20000			; FINDVAC job priority 20
		EBANK=	XSM			; Extended state bank
		2CADR	RESTEST1		; Restart test 1

; Phase 5.17: Geometry start routine 4 (FINDVAC priority 20)
; Initialization for geometric computations (coordinate transformations,
; attitude calculations) during mission phase transitions.
5.17SPOT	OCT	20000			; FINDVAC job priority 20
		EBANK=	XSM			; Extended state bank
# Page 218
		2CADR	GEOSTRT4		; Geometry start 4

; Phase 5.21: Alignment filter task (FINDVAC priority 22)
; Kalman filter processing for IMU alignment convergence. Higher priority
; (22) ensures alignment computations complete without excessive delay.
; Critical for achieving fine alignment accuracy before major burns.
5.21SPOT	OCT	22000			; FINDVAC job priority 22
		EBANK=	XSM			; Extended state bank
		2CADR	ALFLT1			; Alignment filter task 1

; Phase 5.23: Special status task (Immediate restart)
; Handles special status conditions requiring immediate attention. Used for
; exceptional mission events or mode transitions that demand rapid processing.
5.23SPOT	OCT	77777			; WAITLIST immediate restart
		EBANK=	XSM			; Extended state bank
		-2CADR	SPECSTS			; Special status task

; Phase 5.25: Restart test routine 3 (FINDVAC priority 20)
; Additional restart protection self-test. Validates Group 5 restart recovery
; under various simulated failure conditions.
5.25SPOT	OCT	20000			; FINDVAC job priority 20
		EBANK=	XSM			; Extended state bank
		2CADR	RESTEST3		; Restart test 3

; Phase 5.27: Restart error handler (FINDVAC priority 20)
; Error recovery routine for restart anomalies. Logs restart failures and
; attempts corrective action to maintain mission program continuity.
5.27SPOT	OCT	20000			; FINDVAC job priority 20
		EBANK=	XSM			; Extended state bank
		2CADR	RESTAIER		; Restart error handler

; Phase 5.31: Unused/Reserved entry
; Reserved phase entry for future expansion or mission-specific modifications.
; Zeroed entries indicate no restart action defined for this phase.
5.31SPOT	DEC	0			; No priority (unused)
		DEC	0			; No address (unused)
		DEC	0			; No address (unused)

; Phase 5.33: Restart change task (FINDVAC priority 20)
; Manages restart group/phase transitions. Coordinates handoff between
; different mission program sections during phase changes.
5.33SPOT	OCT	20000			; FINDVAC job priority 20
		EBANK=	XSM			; Extended state bank
		2CADR	RESCHNG			; Restart change routine

; Phase 5.35: Unused/Reserved entry
; Reserved phase entry. Zeroed entries indicate no restart action defined.
5.35SPOT	DEC	0			; No priority (unused)
		2DEC	0			; No address (unused)

; Phase 5.37: Average-G check task (Immediate restart)
; Monitors Average-G navigation mode for validity. Immediate restart (77777)
; ensures rapid verification of Average-G computations during coast phases.
5.37SPOT	OCT	77777			; WAITLIST immediate restart
		EBANK=	AOG			; Average-G bank
		-2CADR	CHEKAVEG		; Check Average-G task

; Phase 5.41: Preread task for ignition timing (Immediate restart)
; Pre-burn state vector reading at TIG-30 seconds and TIG-15 seconds.
; Critical for accurate burn initialization. Immediate restart ensures
; preread executes even if interrupted during burn preparation window.
5.41SPOT	OCT	77777			; WAITLIST immediate restart
		EBANK=	DVCNTR			; Delta-V counter bank
		-2CADR	PREREAD			; Preread task

# ANY MORE GROUP 5.ODD RESTART VALUES SHOULD GO HERE

; ============================================================================
; RESTART GROUP 6: Update Program and Entry DAP
;
; Manages ground uplink commands (P27), time synchronization, and atmospheric
; entry autopilot functions. Protects critical timing updates and entry
; gimbal angle reading during Command Module Earth return.
; ============================================================================

; Phase 6.2: Pre-P40 gimbal repositioning (Immediate restart)
; Used by P40 burn program after gimbal drive test. Repositions engine
; gimbal to proper orientation before TVC DAP engagement. Immediate restart
; ensures gimbal reaches commanded position without interruption.
6.2SPOT		OCT	77777			; WAITLIST immediate restart
		EBANK=	AK			; Acceleration bank
		-2CADR	PRE40.6			; Pre-P40 phase 6

; Clock task for time-to-go monitoring (100 cs delay)
; Periodic task monitoring time remaining to ignition. 100 centisecond
; (1 second) WAITLIST delay provides regular time-to-go updates.
		DEC	100			; WAITLIST 100 cs (1 second)
		EBANK=	TTOGO			; Time-to-go bank
		-2CADR	CLOKTASK		; Clock task

# ANY MORE 6.ODD RESTART VALUES SHOULD GO HERE
# Page 219

; Phase 6.3: Clock task for TIG monitoring (100 cs delay)
; Monitors time-to-ignition (TIG) during burn preparation. Provides crew
; with countdown display on DSKY. 1-second updates ensure timely crew
; awareness of approaching ignition.
6.3SPOT		DEC	100			; WAITLIST 100 cs (1 second)
		EBANK=	TIG			; Time-to-ignition bank
		-2CADR	CLOKTASK		; Clock task

; Phase 6.5: Time update task (FINDVAC priority 30)
; Protects TIME2/TIME1 incrementing by P27 (Update Program). Ground control
; uplinks time synchronization commands; this restart entry ensures time
; updates complete correctly even if interrupted. Priority 30 ensures
; time corrections process without excessive delay.
6.5SPOT		OCT	30000			; FINDVAC job priority 30
		EBANK=	TEPHEM			; Time/ephemeris bank
		2CADR	TIMEDIDR		; Time update routine

; Phase 6.7: Unused/Reserved entry
; Reserved phase entry for future expansion. Zeroed entries indicate no
; restart action defined for this phase.
6.7SPOT		OCT	0			; No priority (unused)
		OCT	0			; No address (unused)
		OCT	0			; No address (unused)

; Phase 6.11: Entry DAP gimbal read task (LONGCALL with indirect time)
; Protects CDU (Coupling Data Unit) reading for Entry DAP during atmospheric
; entry. Reads gimbal angles to determine CM attitude. LONGCALL structure
; with indirect time reference (-GENADR) allows flexible scheduling based
; on entry trajectory state. Critical for safe Earth return guidance.
6.11SPOT	-GENADR	CM/GYMDT		; Indirect time reference
		EBANK=	CM/GYMDT		; CM gimbal data bank
		-2CADR	READGYMB		; Read gimbal task

; Phase 6.13: Unused/Reserved entry
; Reserved phase entry for future expansion. Zeroed entries indicate no
; restart action defined for this phase.
6.13SPOT	DEC	0			; No priority (unused)
		DEC	0			; No address (unused)
		DEC	0			; No address (unused)

# Page 220
# PROGRAM DESCRIPTION: NEWPHASE						DATE:  11 NOV 1966
# MOD: 1								ASSEMBLY:  SUNBURST REV
# MOD BY: COPPS								LOG SECTION: PHASE TABLE MAINTENANCE
# FUNCTIONAL DESCRIPTION:
#
#	NEWPHASE IS THE QUICK WAY TO MAKE A NON VARIABLE PHASE CHANGE. IT INCLUDES THE OPTION OF SETTING
#	TBASE OF THE GROUP.  IF TBASE IS TO BE SET, -C(TIME1) IS STORED IN THE TBASE TABLE AS FOLLOWS:
#
#		(L-1)	TBASE0
#		(L)	TBASE1	(IF GROUP=1)
#		(L+1)
#		(L+2)	TBASE2	(IF GROUP=2)
#		-----
#		(L+6)	TBASE4	(IF GROUP=4)
#		(L+7)
#		(L+8)	TBASE5	(IF GROUP=5)
#
#	IN ANY CASE, THE NEGATIVE OF THE PHASE, FOLLOWED (IN THE NEXT REGISTER) BY THE PHASE, IS STORED IN THE
#	PHASE TABLE AS FOLLOWS:
#
#		(L)	-PHASE1	(IF GROUP=1)
#		(L+1)	PHASE1
#		(L+2)	-PHASE2	(IF GROUP=2)
#		(L+3)	PHASE2
#		-----
#		(L+7)	PHASE4
#		(L+8)	-PHASE5	(IF GROUP=5)
#		(L+9)	PHASE5
#
# CALLING SEQUENCE:
#	EXAMPLE IS FOR PLACING A PHASE OF FIVE INTO GROUP THREE:
#
#	1) IF TBASE IS NOT TO BE SET:
#			L-1	CA	FIVE
#			L	TC	NEWPHASE
#			L+1	OCT	00003
#
#	2) IF TBASE IS TO BE SET:
#			L-1	CS	FIVE
#			L	TC	NEWPHASE
#			L+1	OCT	00003
#
# SUBROUTINES CALLED: NONE
#
# NORMAL EXIT MODE: AT L+2 OF CALLING SEQUENCE
#
# ALARM OR ABORT EXITS: NONE
#
# OUTPUT: PHASE TABLE AND TBASE TABLE UPDATED
#
# ERASABLE INITIALIZATION REQ'D: NONE
# Page 221
# DEBRIS:  A,L,TEMPG

# ***WARNING*** THIS PROGRAM IS TO BE PLACED IN FIXED-FIXED AND UNSWITCHED ERASABLE.

		BLOCK	02
		SETLOC	FFTAG1
		BANK

		COUNT*	$$/PHASE

NEWPHASE	INHINT

		TS	L			# SAVE FOR FURTHER USE
		NDX	Q			# OBTAIN THE GROUP NUMBER
		CA	0
		INCR	Q			# OBTAIN THE RETURN ADDRESS
		DOUBLE				# SAVE THE GROUP IN A FORM USED FOR
		TS	TEMPG			# INDEXING

		CCS	L			# SEE IF WE ARE TO SET TBASE
		TCF	+7			# NO, THE DELTA T WAS POSITIVE
		TCF	+6

NUFAZ+10	INCR	A			# SET TBASE AND STORE PHASE CORRECTLY
		TS	L
		CS	TIME1			# SET TBASE
		NDX	TEMPG
		TS	TBASE1 -2
		CS	L			# NOW PUT THE PHASE IN THE RIGHT TABLE LOC
		NDX	TEMPG
		DXCH	-PHASE1	-2
		RELINT
		TC	Q			# NOW RETURN TO CALLER


