# Copyright:	Public domain.
# Filename:	WAITLIST.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1221-1235
# Mod history:	2009-05-14 RSB	Adapted from the Colossus249/ file of the
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
; FILE: WAITLIST.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Timer-driven task scheduler implementing preemptive scheduling via
;        delta-time queue structure. Manages tasks awaiting future execution,
;        integrated with T4RUPT 10ms interrupt. During Apollo 11 descent at
;        ~102:38:26 MET, WAITLIST overflow contributed to 1202 program alarm
;        when landing radar data created excessive computational load. Restart
;        protection and alarm system enabled safe landing continuation.
;
; COMMENT-ONLY READERS: The task timer that scheduled future work, whose overflow
;        triggered the famous 1202 alarms during lunar landing.
; CODE-ALONG READERS: Study timer-driven preemptive scheduler, delta-time queue
;        structure, WAITLIST task insertion/deletion, T4RUPT integration, 1202
;        alarm overflow context, timing precision mechanisms.
; ============================================================================

# Page 1221
; ============================================================================
; WAITLIST TIMER-DRIVEN SCHEDULER - CORE OPERATING SYSTEM COMPONENT
; ============================================================================
;
; The Apollo Guidance Computer uses two complementary scheduling systems:
; the EXECUTIVE (cooperative multitasking) and the WAITLIST (preemptive
; timer-driven scheduling). WAITLIST allows programs to schedule tasks to
; execute after a specified delay, essential for time-critical operations
; like guidance updates, radar sampling, and display refreshes.
;
; During Apollo 11's lunar descent on July 20, 1969, at approximately
; 102:38:26 mission elapsed time, WAITLIST overflow contributed to the
; famous 1202 program alarm. The landing radar was generating more tasks
; than the scheduler could handle, filling the task queue. The AGC's restart
; protection system allowed the mission to continue safely, and flight
; controller Steve Bales made the critical "Go" decision to proceed with
; the landing despite the alarm.
;
; WAITLIST DELTA-TIME QUEUE ARCHITECTURE:
; Rather than storing absolute execution times, WAITLIST uses a "delta-time"
; queue where each entry stores the time difference from the previous entry.
; This elegant design minimizes the number of memory locations that need
; updating when the timer interrupt fires. Only the first entry decrements;
; all others remain unchanged until they reach the front of the queue.
;
; Maximum 9 tasks can be scheduled simultaneously. Attempting to schedule
; a 10th task triggers alarm code 1203 (WAITLIST overflow), displayed to
; the crew on the DSKY. This limit was reached during Apollo 11 descent when
; radar processing combined with guidance computations exceeded capacity.
;
# PROGRAM DESCRIPTION						DATE - 10 OCTOBER 1966
# MOD NO - 2							LOG SECTION - WAITLIST
# MOD BY - MILLER	(DTMAX INCREASED TO 162.5 SEC)		ASSEMBLY SUNBURST REV 5
# MOD 3 BY KERNAN	(INHINT INSERTED AT WAITLIST) 2/28/68 SKIPPER REV 4
# MOD 4BY KERNAN	(TWIDDLE IN 54) 3/28/68 SKIPPER REV 13.
#
# FUNCTIONAL DESCRIPTION-
#	PART OF A SECTION OF PROGRAMS,-WAITLIST, TASKOVER, T3RUPT, USED TO CALL A PROGRAM, (CALLED A TASK),
#	WHICH IS TO BEGIN IN C(A) CENTISECONDS.  WAITLIST UPDATES TIME3, LST1 AND LST2. THE MEANING OF THESE LISTS
#	FOLLOW.
;
; ============================================================================
; WAITLIST DATA STRUCTURES - DELTA-TIME QUEUE IMPLEMENTATION
; ============================================================================
;
; TIME3 (single word):
; Contains countdown timer for first scheduled task. T4RUPT interrupt
; decrements TIME3 every 10 milliseconds. When TIME3 reaches zero, T3RUPT
; interrupt fires to execute the waiting task. Stored as:
; C(TIME3) = 16384 - (T1-T) centiseconds, where T=present time, T1=task time.
; The 16384 offset (2^14) provides a reference point for the countdown.
;
; LST1 (8-word delta-time array):
; Stores time differences between consecutive tasks in the queue. Each entry
; represents -(next_task_time - this_task_time) + 1 centiseconds.
; The "+1" adjustment compensates for AGC timing conventions.
;
#		C(TIME3) = 16384 -(T1-T) CENTISECONDS, (T=PRESENT TIME, T1-TIME FOR TASK1)
#
#			C(LST1)		=	-(T2-T1)+1
#			C(LST1 +1)	=	-(T3-T2)+1
#			C(LST1 +2)	=	-(T4-T3)+1
#				       ...
#			C(LST1 +6)	=	-(T8-T7)+1
#			C(LST1 +7)	=	-(T9-T8)+1
;
; LST2 (9 double-word task address array):
; Stores 2CADR (two-word code address) for each scheduled task. Each 2CADR
; contains both the fixed memory bank number and the address within that bank,
; allowing tasks to reside in any memory location. Maximum 9 tasks (18 words).
;
#			C(LST2)		=	2CADR OF TASK1
#			C(LST2 +2)	=	2CADR OF TASK2
#				       ...
#			C(LST2 +14)	=	2CADR OF TASK8
#			C(LST2 +16)	=	2CADR OF TASK9
;
; DELTA-TIME QUEUE EFFICIENCY:
; Only TIME3 decrements with each timer tick. When TIME3 expires, the first
; entry of LST1 is loaded into TIME3 for the next task. This design minimizes
; computational overhead during time-critical interrupt processing, crucial
; for meeting the AGC's real-time performance requirements.
#
; ============================================================================
; OPERATIONAL CONSTRAINTS AND TIMING LIMITS
; ============================================================================
;
; WAITLIST has strict operational limits designed around AGC hardware
; constraints and mission timing requirements:
;
; TIME RANGE: Delays between 1 centisecond (10ms) and 16250 centiseconds
; (162.5 seconds). Shorter delays risk execution before scheduling completes;
; longer delays exceed the 16-bit countdown timer range. For delays beyond
; 162.5 seconds, use LONGCALL routine (see below).
;
; CAPACITY LIMIT: Maximum 9 tasks in queue simultaneously. During Apollo 11
; descent, this limit was approached when landing radar, guidance computations,
; display updates, and navigation processing all demanded scheduled execution.
; Exceeding this limit triggers 1203 alarm (WAITLIST overflow).
;
; INTERRUPT PROTECTION: All WAITLIST operations execute with interrupts
; inhibited (INHINT) to prevent queue corruption from concurrent access.
; The calling program must issue RELINT after WAITLIST returns to re-enable
; interrupts and allow scheduled tasks to execute.
;
; TASK COMPLETION: Scheduled tasks MUST end with TC TASKOVER, which performs
; cleanup and returns control to interrupted program. Failure to call TASKOVER
; will corrupt the task execution state and potentially crash the AGC.
;
# WARNINGS-
# --------
#	1)	1 <= C(A) <= 16250D (1 CENTISECOND TO 162.5 SEC)
#	2)	9 TASKS MAXIMUM
#	3)	TASKS CALLED UNDER INTERRUPT INHIBITED
#	4)	TASKS END BY TC TASKOVER
#
; ============================================================================
; WAITLIST CALLING SEQUENCE - SCHEDULING TASKS FOR FUTURE EXECUTION
; ============================================================================
;
; Programs use WAITLIST to schedule a task to execute after a specified delay.
; The delay is specified in centiseconds (1 centisecond = 10 milliseconds).
; WAITLIST inserts the task into the delta-time queue in chronological order,
; ensuring tasks execute at their scheduled times regardless of insertion order.
;
; CALLING SEQUENCE EXPLANATION:
; L-1: Load accumulator (A register) with delay in centiseconds (1-16250)
; L:   Transfer control to WAITLIST routine
; L+1: First word of 2CADR (bank and address of task to execute)
; L+2: Second word of 2CADR (task entry point within bank)
; L+3: RELINT re-enables interrupts; WAITLIST returns here
;
; The 2CADR (two-word code address) format allows tasks to reside in any
; fixed memory bank, essential for the AGC's 36K ROM organized in multiple
; banks of 1K words each.
;
# CALLING SEQUENCE-
#
#	L-1	CA	DELTAT 	(TIME IN CENTISECONDS TO TASK START)
#	L	TC	WAITLIST
#	L+1	2CADR	DESIRED TASK
#	L+2	(MINOR OF 2CADR)
#	L+3	RELINT		(RETURNS HERE)
#
; ============================================================================
; TWIDDLE - OPTIMIZED LOCAL TASK SCHEDULING
; ============================================================================
;
; TWIDDLE is a memory-efficient alternative to WAITLIST for scheduling tasks
; in the same memory bank as the calling program. By eliminating the bank
; switching overhead of 2CADR, TWIDDLE saves one word of precious AGC memory.
;
; USE TWIDDLE WHEN: Task resides in same EBANK (erasable bank) and FBANK
; (fixed bank) as caller. This common situation occurs when a module schedules
; its own internal housekeeping tasks.
;
# TWIDDLE-
# -------
#	TWIDDLE IS FOR USE WHEN THE TASK BEING SET UP IS IN THE SAME EBANK AND FBANK AS THE USER.  IN
#	SUCH CASES, IT IMPROVES UPON WAITLIST BY ELIMINATING THE NEED FOR THE BBCON HALF OF THE 2CADR,
# Page 1222
#	SAVING A WORD.  TWIDDLE IS LIKE WAITLIST IN EVERY RESPECT EXCEPT CALLING SEQUENCE, TO WIT-
;
; TWIDDLE CALLING SEQUENCE:
; L-1: Load accumulator with delay in centiseconds
; L:   Transfer control to TWIDDLE routine  
; L+1: ADRES (single-word address) of task in current bank
; L+2: RELINT re-enables interrupts; TWIDDLE returns here
;
; Memory savings matter critically in the AGC's 2K erasable memory. During
; mission-critical phases like lunar descent, every saved word allows more
; complex guidance algorithms or additional safety margins.
;
#		L-1	CA	DELTAT
#		L	TC	TWIDDLE
#		L+1	ADRES	DESIRED TASK
#		L+2	RELINT		(RETURNS HERE)
#
# NORMAL EXIT MODES-
#
#	AT L+3 OF CALLING SEQUENCE
#
; ============================================================================
; WAITLIST OVERFLOW ALARM - PROGRAM ALARM 1203
; ============================================================================
;
; When attempting to schedule a 10th task (exceeding the 9-task limit),
; WAITLIST generates alarm code 1203. The ALARM system displays "1203" on
; the DSKY, alerting the crew to scheduler saturation.
;
; ALARM 1203 vs 1202: While alarm 1202 (EXECUTIVE overflow) became famous
; during Apollo 11 landing, alarm 1203 (WAITLIST overflow) can also occur
; under high computational load. Both alarms indicate the AGC is processing
; more work than designed capacity, but restart protection allows safe
; continuation if the overflow is temporary.
;
; During Apollo 11 descent, WAITLIST came close to overflow as radar processing,
; guidance updates, throttle commands, and display refreshes all demanded
; scheduled execution within tight timing windows. The system's ability to
; handle this extreme workload validated the AGC's robust design.
;
# ALARM OR ABORT EXIT MODES-
#
#	TC	ABORT
#	OCT	1203	(WAITLIST OVERFLOW - TOO MANY TASKS)
#
# ERASABLE INITIALIZATION REQUIRED-
#
#	ACCOMPLISHED BY FRESH START,--	LST2,..., LST2 +16 =ENDTASK
#					LST1,..., LST1 +7  =NEG1/2
#
# OUTPUT--
#
#	LST1 AND LST2 UPDATED WITH NEW TASK AND ASSOCIATED TIME.
# DEBRIS-
#	CENTRALS- A,Q,L
#	OTHER   - WAITEXIT, WAITADR, WAITTEMP, WAITBANK
# DETAILED ANALYSIS OF TIMING-
#
#	CONTROL WILL NOT BE RETURNED TO THE SPECIFIED ADDRESS (2CADR) IN EXACTLY DELTA T CENTISECONDS.
#	THE APPROXIMATE TIME MAY BE CALCULATED AS FOLLOWS
#
#		LET T0 = THE TIME OF THE TC WAITLIST
#		LET TS = T0 +147U + COUNTER INCREMENTS (SET UP TIME)
#		LET X  = TS -(100TS)/100  (VARIANCE FROM COUNTERS)
#		LET Y  = LENGTH OF TIME OF INHIBIT INTERRUPT AFTER T3RUPT
#		LET Z  = LENGTH OF TIME TO PROCESS TASKS WHICH ARE DUE THIS T3RUPT BUT DISPATCHED EARLIER.
#	(Z=0, USUALLY)
#		LET DELTD = THE ACTUAL TIME TAKEN TO GIVE CONTROL TO 2CADR
#	       THEN DELTD = TS+DELTA T -X +Y +Z +1.05MS* +COUNTERS*
#	*THE TIME TAKEN BY WAITLIST ITSELF AND THE COUNTER TICKING DURING THIS WAITLIST TIME.
#
#	IN SHORT, THE ACTUAL TIME TO RETURN CONTROL TO A 2CADR IS AUGMENTED BY THE TIME TO SET UP THE TASK'S
# 	INTERRUPT, ALL COUNTERS TICKING, THE T3RUPT PROCESSING TIME, THE WAITLIST PROCESSING TIME AND THE POSSIBILITY
#	OF OTHER TASKS INHIBITING THE INTERRUPT.

; ============================================================================
; TRANSITION: From Program Description to Implementation
;
; The Apollo Guidance Computer needs a mechanism to schedule tasks for future
; execution - for example, checking engine status 5 seconds after ignition,
; or updating navigation 30 seconds from now. WAITLIST provides this timer-
; driven scheduling through a delta-time queue where each entry stores the
; time difference to the next task. The T4RUPT interrupt (firing every 10ms)
; drives the system, decrementing counters and dispatching tasks when ready.
;
; During Apollo 11's lunar descent on July 20, 1969 at mission time 102:38:26,
; the landing radar created excessive task scheduling requests, filling the
; WAITLIST queue. This contributed to the famous 1202 program alarm. The
; system's restart protection and priority handling allowed the mission to
; continue safely, and Neil Armstrong landed successfully 7 minutes later.
; ============================================================================

		BLOCK	02
# Page 1223
		EBANK=	LST1		# TASK LISTS IN SWITCHED E BANK.

		COUNT	02/WAIT

; ============================================================================
; TWIDDLE - Simplified Task Scheduling (Same Bank Only)
;
; COMMENT-ONLY READERS: A faster version of WAITLIST when the scheduled task
; is in the same memory bank as the caller, saving one word of memory.
;
; CODE-ALONG READERS: TWIDDLE optimizes WAITLIST for tasks in same EBANK and
; FBANK. By setting Q to overflow (-1), forces INDEX WAITEXIT to use current
; bank's address space. Eliminates need for BBCON half of 2CADR, saving memory.
; Entry: A = delay (centiseconds), Returns to caller +2 after scheduling.
; ============================================================================

TWIDDLE		INHINT
		TS	L		# SAVE DELAY TIME IN L
		CA	POSMAX
		ADS	Q		# CREATING OVERFLOW AND Q-1 IN Q
		CA	BBANK
		EXTEND
		ROR	SUPERBNK
		XCH	L

; ============================================================================
; WAITLIST - Main Entry Point for Task Scheduling
;
; COMMENT-ONLY READERS: This is where programs request the computer to call
; them back after a specified delay. During Apollo 11's descent, the landing
; radar generated so many scheduled tasks that this list became full, causing
; the 1202 program alarm at 102:38:26 mission time.
;
; CODE-ALONG READERS: Primary task scheduler entry. Accepts delay time in A
; register (centiseconds, range 1-16250). Following TC WAITLIST must be 2CADR
; of task to execute. Validates positive delay, extracts 2CADR, then jumps to
; WAIT2 for queue insertion. Maximum 9 tasks; overflow triggers alarm 1203.
; ============================================================================

WAITLIST	INHINT			; Inhibit interrupts for list integrity
		EXTEND			; Extended instruction mode for BZMF
		BZMF	WATLST0-	; Branch if delay <= 0 (invalid)
		XCH	Q		; SAVE DELTA T IN Q AND RETURN IN
		TS	WAITEXIT	; WAITEXIT.
		EXTEND
		INDEX	WAITEXIT	; IF TWIDDLING, THE TS SKIPS TO HERE
		DCA	0		; PICK UP 2CADR OF TASK.
	-1	TS	WAITADR		; BBCON WILL REMAIN IN L

; Entry point from FIXDELAY and VARDELAY with delay already processed.
DLY2		CAF	WAITBB		; ENTRY FROM FIXDELAY AND VARDELAY.
		XCH	BBANK		; Switch to bank containing WAIT2
		TCF	WAIT2		; Jump to queue insertion logic

; Error handler for invalid delay time (zero or negative).
; Zero/negative delays make no sense for scheduled future tasks.
WATLST0-	TC	POODOO		; Invoke alarm system
		OCT	1204		; WAITLIST CALL WITH ZERO OR NEG DT

; ============================================================================
; RETURN TO CALLER AFTER TASK INSERTION
;
; COMMENT-ONLY READERS: After scheduling the task, control returns to the
; program that made the WAITLIST call, skipping over the 2CADR address data.
;
; CODE-ALONG READERS: Exit routine restoring caller's context. WAITEXIT holds
; return address. Add TWO to skip over 2CADR (both words), then DTCB (Double
; Transfer Control to Both banks) returns to caller's bank and address.
; ============================================================================

LVWTLIST	DXCH	WAITEXIT	; Restore return address to A and L
		AD	TWO		; Skip over 2CADR words (+2)
		DTCB			; Return to caller at new address

		EBANK=	LST1
WAITBB		BBCON	WAIT2		; Bank/address constant for WAIT2

; ============================================================================
; FIXDELAY - Delay Specified Amount Then Return
;
; COMMENT-ONLY READERS: Allows a running task to pause itself for a fixed
; time, then resume. Used throughout mission for timed sequences like waiting
; between engine commands or sensor readings.
;
; CODE-ALONG READERS: Task self-scheduling with fixed delay. Caller places
; TC FIXDELAY followed by delay constant. Extracts delay from caller+1,
; increments return address (Q) to skip delay word, then re-schedules this
; task via DLY2. Must be called under WAITLIST control (within active task).
; ============================================================================

FIXDELAY	INDEX	Q		; BOTH ROUTINES MUST BE CALLED UNDER
		CAF	0		; WAITLIST CONTROL AND TERMINATE THE TASK
		INCR	Q		; IN WHICH THEY WERE CALLED.

; ============================================================================
; VARDELAY - Delay Variable Amount Then Return
;
; COMMENT-ONLY READERS: Similar to FIXDELAY but accepts the delay time as
; a parameter in the accumulator, allowing computed delays.
;
; CODE-ALONG READERS: Task self-scheduling with variable delay passed in A.
; Saves return address (Q) as task address, extracts current bank (BBANK),
; adds superbank bits via ROR SUPERBNK to form complete BBCON, sets WAITEXIT
; to return to TASKOVER after delay, then jumps to DLY2 for queue insertion.
; ============================================================================

VARDELAY	XCH	Q		; DT TO Q.  TASK ADRES TO WAITADR.
		TS	WAITADR		; Save task resume address
		CA	BBANK		; BBANK IS SAVED DURING DELAY.
		EXTEND
# Page 1224
		ROR	SUPERBNK	; ADD SBANK TO BBCON.
		TS	L		; Store BBCON in L register
		CAF	DELAYEX		; Get TASKOVER return address
		TS	WAITEXIT	; GO TO TASKOVER AFTER TASK ENTRY.
		TCF	DLY2		; Jump to delay processing

DELAYEX		TCF	TASKOVER -2	; RETURNS TO TASKOVER

# Page 1225
# ENDTASK MUST ENTERED IN FIXED-FIXED SO IT IS DISTINGUISHABLE BY ITS ADRES ALONE.

		EBANK=	LST1
ENDTASK		-2CADR	SVCT3		; End-of-list marker task address

; ============================================================================
; SVCT3 - IMU Drift Compensation Task
;
; COMMENT-ONLY READERS: A special scheduled task that periodically compensates
; for drift in the spacecraft's gyroscopes (IMU = Inertial Measurement Unit).
; The IMU tells the computer which way the spacecraft is pointing - critical
; for navigation and control. Over time, tiny errors accumulate, so this
; task applies corrections every 81.93 seconds throughout the mission.
;
; CODE-ALONG READERS: ENDTASK marker points here as final entry in WAITLIST.
; Checks FLAGWRD2 drift flag; if set, exits. Otherwise, if IMU not busy
; (IMUCADR=0), schedules NBDONLY task at priority 35 to apply NBD (navigation
; base) coefficient compensation. If IMU busy, delays 500 centiseconds (5 sec)
; and retries. Mission-critical for maintaining navigation accuracy.
; ============================================================================

SVCT3		CCS	FLAGWRD2	; Check DRIFT FLAG in FLAGWRD2
		TCF	TASKOVER	; Positive: drift compensation disabled
		TCF	TASKOVER	; Zero: drift compensation disabled
		TCF	+1		; Negative: proceed with compensation

		CCS	IMUCADR		; DON'T DO NBDONLY IF SOMEONE ELSE IS IN
		TCF	SVCT3X		; IMU busy (positive), delay and retry
		TCF	+3		; IMU free (zero), proceed
		TCF	SVCT3X		; IMU busy (negative), delay and retry
		TCF	SVCT3X		; IMU busy (overflow), delay and retry
	+3	CAF	PRIO35		; COMPENSATE FOR NBD COEFFICIENTS ONLY.
		TC	NOVAC		; Schedule job at priority 35
		EBANK=	NBDX		;   ENABLE EVERY 81.93 SECONDS
		2CADR	NBDONLY		; Address of NBD compensation routine

		TCF	TASKOVER	; Done - terminate this task

		SETLOC	FFTAG6		; Set location for following code
		BANK			; Bank directive

SVCT3X		TC	FIXDELAY	; DELAY MAX OF 2 TIMES FOR IMU ZERO
		DEC	500		; Wait 500 centiseconds (5 seconds)
		TC	SVCT3		; CHECK DRIFT FLAG AGAIN

# Page 1226
; ============================================================================
; BEGIN TASK INSERTION - WAIT2 Core Delta-Time Queue Algorithm
;
; COMMENT-ONLY READERS: This is where new tasks get added to the waiting list.
; The computer maintains a queue of tasks sorted by when they should run.
; Each entry stores the time difference to the next task, not absolute times.
; This delta-time approach saves memory and simplifies timer handling. During
; Apollo 11 descent at 102:38:26 MET, excessive radar data processing created
; so many tasks that this queue filled up, triggering the famous 1202 alarm.
;
; CODE-ALONG READERS: WAIT2 implements delta-time queue insertion. TIME3 counts
; down from 16384 (40000 octal) toward zero, with T3RUPT interrupt when it
; reaches zero. Queue entries in LST1 store negative delta-times: -(T2-T1)+1.
; Algorithm handles TIME3 overflow carefully (bit test at 200). Computes where
; new task TD fits relative to T1 (next task time), then either inserts at
; front or scans queue via WTLST5. Critical timing-sensitive code during 1202.
; ============================================================================

		BANK	01
		COUNT	01/WAIT

WAIT2		TS	WAITBANK	; Save BBANK of calling program
		CS	TIME3		; Get complement of current timer
		AD	BIT8		; BIT 8 = OCT 200
		CCS	A		; TEST 200 - C(TIME3).  IF POSITIVE,
					; IT MEANS THAT TIME3 OVERFLOW HAS OCCURRED PRIOR TO CS TIME3 AND THAT
					; C(TIME3) = T - T1, INSTEAD OF 1.0 - (T1 - T).  THE FOLLOWING FOUR
					; ORDERS SET C(A) = TD - T1 + 1 IN EITHER CASE.

		AD	OCT40001	; OVERFLOW HAS OCCURRED.  SET C(A) =
		CS	A		; T - T1 + 1.0 - 201

; NORMAL CASE (C(A) NNZ) YIELDS SAME C(A):  -( -(1.0-(T1-T)) + 200) - 1

		AD	OCT40201	; Complete time computation
		AD	Q		; RESULT = TD - T1 + 1 (Q contains DT)

		CCS	A		; TEST TD - T1 + 1 (new task vs next task)

		AD	LST1		; IF TD - T1 POS, new task comes AFTER next task
		TCF	WTLST5		; Scan queue to find insertion point
					; C(A) = (TD - T1) + C(LST1) = TD-T2+1

		NOOP			; Required delay slot for CCS branch
		CS	Q		; New task comes BEFORE next task

; ============================================================================
; INSERT NEW TASK AT FRONT OF QUEUE
;
; COMMENT-ONLY READERS: The new task needs to run sooner than any other waiting
; task, so it becomes the next task. The hardware timer (TIME3) gets updated
; to count down to this new task's execution time. The old "next task" shifts
; down in the queue and its time gets adjusted relative to the new task.
;
; CODE-ALONG READERS: TD < T1, so new task becomes next. Must update TIME3
; counter to new task's execution time. This section handles TIME3 carefully:
; overflow impossible here since TD-T G/E +1 (see NOTE below). Computes
; 1.0 - (TD-T) as new TIME3 value. Exchanges TIME3 to load new countdown,
; then adjusts old first entry LST1 by computing -(T1-TD)+1. Uses QXCH to
; zero-index Q for LST1/LST2 array traversal in WTLST4.
; ============================================================================

; NOTE THAT THIS PROGRAM SECTION IS NEVER ENTERED WHEN T-T1 G/E -1,
; SINCE TD-T1+1 = (TD-T) + (T-T1+1), AND DELTA T = TD-T G/E +1.  (G/E
; SYMBOL MEANS GREATER THAN OR EQUAL TO).  THUS THERE NEED BE NO CON-
; CERN OVER A PREVIOUS OR IMMINENT OVERFLOW OF TIME3 HERE.

		AD	POS1/2		; WHEN TD IS NEXT, FORM QUANTITY
		AD	POS1/2		; 1.0 - DELTA T = 1.0 - (TD - T)
		XCH	TIME3		; Update timer, get old TIME3 in A
		AD	NEGMAX		; Convert old TIME3 format
		AD	Q		; 1.0 - DELTA T now complete (new LST1 value)
		EXTEND			; ZERO INDEX Q for array access
		QXCH	7		; Zero Q, save A in location 7

# Page 1227
; ============================================================================
; WTLST4 - Shift Queue Entries Down to Make Room at Position Q
;
; COMMENT-ONLY READERS: To insert the new task, the system must shift all
; existing tasks down in the queue to make room. This is like inserting a
; new appointment in a paper calendar - you have to move all the later
; appointments down one line. The system carefully shifts both the time
; differences and the task addresses. If the queue is already full (9 tasks),
; this triggers alarm 1203. During Apollo 11 descent, radar data processing
; created so many tasks that this section executed frequently under heavy load.
;
; CODE-ALONG READERS: Ripple-shift algorithm for both LST1 (delta-times) and
; LST2 (2CADRs). INDEX Q determines insertion point (0-7 for positions 1-8).
; First ripples LST1 entries using cascaded XCH, with A holding value to insert.
; Then uses indexed TCF +1 to position for LST2 ripple (double-precision task
; addresses). Eight XCH operations shift LST1, nine DXCH operations shift LST2.
; Final check: if ENDTASK marker shifted out (not FIXED-FIXED), queue overflow,
; branch WTABORT → alarm 1203. Otherwise LVWTLIST to complete insertion.
; ============================================================================

WTLST4		XCH	LST1		; Shift LST1 delta-time queue down
		XCH	LST1 	+1	; A ripples through array
		XCH	LST1 	+2	; Each entry shifts to next position
		XCH	LST1 	+3	; Starting from insertion point Q
		XCH	LST1 	+4
		XCH	LST1 	+5
		XCH	LST1 	+6
		XCH	LST1 	+7	; Final shift pushes last entry into A

		CA	WAITADR		; Get new task address (minor part in L)
		INDEX	Q		; Use Q to position for LST2 insertion
		TCF	+1		; Indexed jump into ripple sequence

		DXCH	LST2		; Shift LST2 task address queue down
		DXCH	LST2 	+2	; Each 2CADR (double-precision address)
		DXCH	LST2 	+4	; shifts to next position
		DXCH	LST2 	+6	; Nine positions total (18 words)
		DXCH	LST2 	+8D
		DXCH	LST2 	+10D	; AT END, CHECK THAT C(LST2 +10) IS STD
		DXCH	LST2 	+12D	; Checking for queue overflow
		DXCH	LST2 	+14D
		DXCH	LST2 	+16D	; Last shift pushes ENDTASK into A/L
		AD	ENDTASK		; Check if ENDTASK marker was displaced
					; END ITEM, AS CHECK FOR EXCEEDING
					; THE LENGTH OF THE LIST.
		EXTEND			; DUMMY TASK ADRES SHOULD BE IN FIXED-
		BZF	LVWTLIST	; FIXED SO ITS ADRES ALONE DISTINGUISHES
		TCF	WTABORT		; IT. Overflow! → Alarm 1203

# Page 1228
; ============================================================================
; WTLST5 - Sequential Queue Position Search (Positions 2-8)
;
; COMMENT-ONLY READERS: The computer continues searching through the scheduled
; task queue, comparing the new task's delay time against each entry to find
; the right position. If all 9 slots are full, the system triggers alarm 1203.
;
; CODE-ALONG READERS: Series of 7 identical test blocks, each checking if
; remaining time (A register) is less than next delta-time entry. Pattern:
; CCS A tests sign, adds LST1+n to accumulate position, branches to WTLST2
; for insertion if negative (task belongs here), or continues if positive.
; Sequence tests positions 2-8. If all fail, falls through to WTABORT.
; ============================================================================

WTLST5		CCS	A		; TEST TD - T2 + 1 (remaining vs task 2)
		AD	LST1 +1		; Positive: add task 2 delta-time
		TCF	+4		; Continue to next test
		AD	ONE		; Zero/negative: restore for insertion
		TC	WTLST2		; Insert at position 1
		OCT	1		; Position index

	+4	CCS	A		; TEST TD - T3 + 1 (remaining vs task 3)
		AD	LST1 +2		; Positive: add task 3 delta-time
		TCF	+4		; Continue to next test
		AD	ONE		; Zero/negative: restore for insertion
		TC	WTLST2		; Insert at position 2
		OCT	2		; Position index

	+4	CCS	A		; TEST TD - T4 + 1 (remaining vs task 4)
		AD	LST1 +3		; Positive: add task 4 delta-time
		TCF	+4		; Continue to next test
		AD	ONE		; Zero/negative: restore for insertion
		TC	WTLST2		; Insert at position 3
		OCT	3		; Position index

	+4	CCS	A		; TEST TD - T5 + 1 (remaining vs task 5)
		AD	LST1 +4		; Positive: add task 5 delta-time
		TCF	+4		; Continue to next test
		AD	ONE		; Zero/negative: restore for insertion
		TC	WTLST2		; Insert at position 4
		OCT	4		; Position index

	+4	CCS	A		; TEST TD - T6 + 1 (remaining vs task 6)
		AD	LST1 +5		; Positive: add task 6 delta-time
		TCF	+4		; Continue to next test
		AD	ONE		; Zero/negative: restore for insertion
		TC	WTLST2		; Insert at position 5
		OCT	5		; Position index

	+4	CCS	A		; TEST TD - T7 + 1 (remaining vs task 7)
		AD	LST1 +6		; Positive: add task 7 delta-time
		TCF	+4		; Continue to next test
		AD	ONE		; Zero/negative: restore for insertion
		TC	WTLST2		; Insert at position 6
		OCT	6		; Position index

# Page 1229
	+4	CCS	A		; TEST TD - T8 + 1 (remaining vs task 8)
		AD	LST1 +7		; Positive: add task 8 delta-time
		TCF	+4		; Continue to next test
		AD	ONE		; Zero/negative: restore for insertion
		TC	WTLST2		; Insert at position 7
		OCT	7		; Position index

; ============================================================================
; WTABORT - Waitlist Overflow Handler
;
; COMMENT-ONLY READERS: All 9 task slots are full! The computer cannot
; schedule the new task and triggers alarm 1203 "WAITLIST OVERFLOW". During
; Apollo 11 descent, this overflow (along with Executive overflow 1202)
; occurred when landing radar data created excessive scheduling demand at
; mission time ~102:38:26. Flight controller Steve Bales made the critical
; "Go" decision to continue landing despite these alarms.
;
; CODE-ALONG READERS: Falls through when all 8 queue positions occupied
; (9-task limit reached). Calls BAILOUT with alarm code 1203. System design
; allows graceful degradation - alarm recorded, lowest-priority tasks may be
; dropped, but mission-critical operations continue. Restart protection
; preserves state. This overflow mechanism, combined with restart system,
; enabled Apollo 11 landing despite computational overload.
; ============================================================================

	+4	CCS	A		; Final test - any room left?
WTABORT		TC	BAILOUT		; NO ROOM IN THE INN - trigger alarm
		OCT	1203		; Alarm code: WAITLIST OVERFLOW

		AD	ONE		; Task belongs at end of queue (position 8)
		TC	WTLST2		; Insert at final position
		OCT	10		; Position index (octal 10 = decimal 8)

OCT40201	OCT	40201		; Constant: 40201 octal

# Page 1230
# THE ENTRY TO WTLST2 JUST PRECEDING OCT N IS FOR T  LE TD LE T   -1.
#                                                  N           N+1
#
# (LE MEANS LESS THAN OR EQUAL TO).  AT ENTRY, C(A) = -(TD - T   + 1)
#                                                             N+1
#
# THE LST1 ENTRY-(T   -T +1) IS TO BE REPLACED BY -(TD - T  + 1), AND
#                  N+1  N                                 N
#
# THE ENTRY-(T   - TD + 1) IS TO BE INSERTED IMMEDIATELY FOLLOWING.
#             N+1

; ============================================================================
; WTLST2 - Task Insertion at Found Position
;
; COMMENT-ONLY READERS: The computer has found where the new task belongs in
; the queue. Now it updates the delta-time values to insert the task at the
; correct position, splitting the time between the previous and next tasks.
;
; CODE-ALONG READERS: Called with position index in Q, A = -(TD - T[n+1] + 1).
; Saves A to WAITTEMP, extracts position index from return address (Q),
; computes updated delta-times: LST1[n-1] gets -(TD - T[n]) + 1 (time from
; previous task to new task), then branches to WTLST4 with -(T[n+1] - TD + 1)
; (time from new task to next task) to shift remaining entries. Complex
; address arithmetic using INDEX to access correct LST1 entry.
; ============================================================================

WTLST2		TS	WAITTEMP	; Save A = -(TD - T[n+1] + 1)
		INDEX	Q		; Use Q as index
		CAF	0		; Load position index from return address
		TS	Q		; Store index value into Q

		CAF	ONE		; Start computing new delta-time
		AD	WAITTEMP	; A = -(TD - T[n+1]) + 2
		INDEX	Q		; Index by position
		ADS	LST1 -1		; Add to LST1[n-1]: now = -(TD - T[n]) + 1

		CS	WAITTEMP	; A = +(TD - T[n+1] + 1) = -(T[n+1] - TD - 1)
		INDEX	Q		; Index by position
		TCF	WTLST4		; Branch to shift remaining entries down

# Page 1231
# ENTERS HERE ON T3 RUPT TO DISPATCH WAITLISTED TASK.

; ============================================================================
; T3RUPT - Waitlist Task Dispatch Interrupt Handler
;
; COMMENT-ONLY READERS: When a task's scheduled time arrives, this interrupt
; handler dispatches it for execution. The computer carefully saves its
; current state, moves all tasks up in the queue, and then transfers control
; to the waiting task. This is the mechanism that executes scheduled work at
; precise times throughout the mission.
;
; CODE-ALONG READERS: Timer 3 interrupt handler for dispatching waitlisted
; tasks. Entry point preserves current SUPERBANK, BANKRUPT (with E/F bank),
; and QRUPT. Falls through to T3RUPT2 for actual dispatch. Critical timing:
; runs under interrupt inhibit, must complete queue updates before enabling
; interrupts. Part of preemptive scheduler enabling real-time task execution.
; ============================================================================

T3RUPT		EXTEND			; Extend next instruction
		ROR	SUPERBNK	; Read current superbank value
		TS	BANKRUPT	; Save with E and F bank values
		EXTEND			; Extend next instruction
		QXCH	QRUPT		; Save Q, exchange with QRUPT

; ============================================================================
; T3RUPT2 - Waitlist Task Dispatcher Main Logic
;
; COMMENT-ONLY READERS: The computer now executes the scheduled task. It moves
; all tasks up one position in the queue (the first task is about to run, so
; everything else moves forward). It carefully updates the timing registers to
; prevent interference, then dispatches the task for execution.
;
; CODE-ALONG READERS: Core dispatch logic. Algorithm: (1) Shift LST1 delta-
; time queue up by one position, inserting NEG1/2 at bottom (corresponds to
; 81.91 sec ENDTASK interval for position 8). (2) Update TIME3 = 1.0 - T2 - T
; to prevent timer tick during update. (3) Shift LST2 2CADR queue up, moving
; ENDTASK to bottom. (4) Extract BBCON from task 2CADR, write to SUPERBNK.
; (5) DXCH Z to transfer control to task. Queue shifting ensures next task
; ready for dispatch. RUPTAGN overflow detection ensures timing integrity.
; ============================================================================

T3RUPT2		CAF	NEG1/2		; Load -1/2 for bottom queue entry
					; DISPATCH WAITLIST TASK.
		XCH	LST1 +7		; Exchange with LST1+7, shift chain starts
		XCH	LST1 +6		; Continue upward shift
		XCH	LST1 +5		; A ripples through all positions
		XCH	LST1 +4		# 1. MOVE UP LST1 CONTENTS, ENTERING
		XCH	LST1 +3		#    A VALUE OF 1/2 +1 AT THE BOTTOM
		XCH	LST1 +2		#    FOR T6-T5, CORRESPONDING TO THE
		XCH	LST1 +1		#    INTERVAL 81.91 SEC FOR ENDTASK.
		XCH	LST1		; Final exchange, A now = old LST1
		AD	POSMAX		# 2. SET T3 = 1.0 - T2 - T USING LIST 1.
		ADS	TIME3		#    SO T3 WONT TICK DURING UPDATE.
					; Add to TIME3 to compensate for shift
		TS	RUPTAGN		; Store result in RUPTAGN overflow flag
		CS	ZERO		; Load -0 (all ones)
		TS	RUPTAGN		# SETS RUPTAGN TO +1 ON OVERFLOW.
					; Detects if another task became due

		EXTEND			# DISPATCH TASK.
		DCS	ENDTASK		; Load double ENDTASK marker (2CADR)
		DXCH	LST2 +16D	; Start 2CADR queue shift (double precision)
		DXCH	LST2 +14D	; Each DXCH moves 2 words up 2 positions
		DXCH	LST2 +12D	; Continue through all task 2CADRs
		DXCH	LST2 +10D	; Shifting entire queue forward
		DXCH	LST2 +8D	; As first task dispatches
		DXCH	LST2 +6		; Second task becomes first
		DXCH	LST2 +4		; Third becomes second, etc.
		DXCH	LST2 +2		; Next-to-last shift
		DXCH	LST2		; Final shift, A,L = first task 2CADR

		XCH	L		; Move BBCON (bank) to A, CADR stays in L
		EXTEND			; Extend next instruction
		WRITE 	SUPERBNK	# SET SUPERBANK FROM BBCON OF 2CADR
					; Write bank value to SUPERBNK register
		XCH	L		# RESTORE TO L FOR DXCH Z.
					; Put BBCON back in L, get CADR in A
		DTCB			; DXCH Z (transfer control to task 2CADR)
					; Dispatches task, Z (PC) gets A,L values

# Page 1232
# RETURN, AFTER EXECUTION OF T3 OVERFLOW TASK:

		BLOCK	02
		COUNT	02/WAIT

; ============================================================================
; TASKOVER - Waitlist Task Exit Handler
;
; COMMENT-ONLY READERS: When a scheduled task finishes its work, it calls this
; routine. The computer checks if another task became due while the first one
; was running. If so, it immediately dispatches that next task. If not, it
; restores the computer's state and returns to whatever was interrupted.
;
; CODE-ALONG READERS: Exit point for all waitlisted tasks (via TCF TASKOVER).
; Checks RUPTAGN overflow flag: +1 means another task due during previous
; task execution, requires immediate dispatch via T3RUPT2. -0 (negative zero)
; means no overflow, safe to resume interrupted program. Falls through to
; RESUME to restore context and return to mainline. Critical decision point
; ensuring no tasks missed their scheduled execution time.
; ============================================================================

TASKOVER	CCS	RUPTAGN		# IF +1 RETURN TO T3RUPT, IF -0 RESUME.
					; Check RUPTAGN: +1=overflow, -0=no overflow
		CAF	WAITBB		; Overflow occurred, another task due
		TS	BBANK		; Set bank to WAITLIST bank
		TCF	T3RUPT2		# DISPATCH NEXT TASK IF IT WAS DUE.
					; Loop back to dispatch next waiting task

		CA	BANKRUPT	; No overflow, prepare to resume
		EXTEND			; Extend next instruction
		WRITE	SUPERBNK	# RESTORE SUPERBANK BEFORE RESUME IS DONE
					; Restore saved superbank value

; ============================================================================
; RESUME - Interrupt Return Handler
;
; COMMENT-ONLY READERS: The computer now returns to whatever it was doing
; before the task interrupt occurred. It carefully restores all saved values
; (return address, bank, accumulator) and then continues exactly where it left
; off. The interrupt is now complete, and normal program flow resumes.
;
; CODE-ALONG READERS: Standard interrupt return sequence. Restores QRUPT
; (return address in Q), BANKRUPT (bank register with superbank), and ARUPT
; (A register). RELINT re-enables interrupts. Final RESUME instruction
; restores program counter and returns control. NOQRSM and NOQBRSM are
; alternate entry points skipping Q or Q+BBANK restoration when not needed.
; ============================================================================

RESUME		EXTEND			; Extend next instruction
		QXCH	QRUPT		; Restore Q from QRUPT
NOQRSM		CA	BANKRUPT	; Load saved bank value
		XCH	BBANK		; Restore BBANK register
NOQBRSM		DXCH	ARUPT		; Restore A register from ARUPT
		RELINT			; Re-enable interrupts
		RESUME			; Return from interrupt

# Page 1233
# LONGCALL
#
# PROGRAM DESCRIPTION				      DATE- 17 MARCH 1967
# PROGRAM WRITTEN BY W.H. VANDEVER		     LOG SECTION WAITLIST
# MOD BY- R. MELANSON TO ADD DOCUMENTATION	ASSEMBLY SUNDISK REV. 100
#
# FUNCTIONAL DESCRIPTION-
# LONGCALL IS CALLED WITH THE DELTA TIME ARRIVING IN A,L SCALED AS TIME2,TIME1 WITH THE 2CADR OF THE TASK
# IMMEDIATELY FOLLOWING THE TC LONGCALL.  FOR EXAMPLE, IT MIGHT BE DONE AS FOLLOWS WHERE TIMELOC IS THE NAME OF
# A DP REGISTER CONTAINING A DELTA TIME AND WHERE TASKTODO IS THE NAME OF THE LOCATION AT WHICH LONGCALL IS TO
# START
#
# CALLING SEQUENCE-
#		EXTEND
#		DCA	TIMELOC
#		TC	LONGCALL
#		2CADR	TASKTODO
#
# NORMAL EXIT MODE-
#	1). TC		WAITLIST
#	2). DTCB	(TC L+3 OF CALLING ROUTINE 1ST PASS THRU LONGCYCL)
#	3). DTCB	(TO TASKOVER ON SUBSEQUENT PASSES THRU LONGCYCL)
#
# ALARM OR ABORT EXIT MODE-
#	NONE
#
# OUTPUT-
#	LONGTIME AND LONGTIME+1 = DELTA TIME
#	LONGEXIT AND LONGEXIT+1 = RETURN 2CADR
#	LONGCADR AND LONGCADR+1 = TASK 2CADR
#	A = SINGLE PRECISION TIME FOR WAITLIST
#
# ERASABLE INITIALIZATION-
#	A = MOST SIGNIFICANT PART OF DELTA TIME
#	L = LEAST SIGNIFICANT PART OF DELTA TIME
#	Q = ADDRESS OF 2CADR TASK VALUE
#
# DEBRIS-
#	A,Q,L
#	LONGCADR AND LONGCADR+1
#	LONGEXIT AND LONGEXIT+1
#	LONGTIME AND LONGTIME+1
#
# *** THE FOLLOWING IS TO BE IN FIXED-FIXED AND UNSWITCHED ERRASIBLE ***

; ============================================================================
; LONGCALL - Extended Duration Task Scheduler
;
; COMMENT-ONLY READERS: Sometimes the spacecraft needs to schedule a task many
; minutes into the future—beyond the 162.5 second limit of the regular
; waitlist. This routine handles those long delays by breaking them into
; smaller chunks. It schedules intermediate "cycle through" tasks that keep
; checking if it's time yet, and finally executes the real task when the full
; delay has elapsed.
;
; CODE-ALONG READERS: Extends WAITLIST capability beyond 16250 centisecond
; (162.5 sec) maximum by scheduling LONGCYCL intermediate tasks. Takes double
; precision delta time in A,L (TIME2,TIME1 format) and 2CADR of task following
; TC LONGCALL. Saves parameters in LONGTIME, LONGCADR, LONGEXIT. Transfers to
; LNGCALL2 in switched bank for main logic. Critical for mission events
; requiring precise timing over multi-minute intervals (orbital maneuvers,
; IMU alignments, system recalibrations).
; ============================================================================

		BLOCK	02
		EBANK=	LST1
LONGCALL	DXCH	LONGTIME	# OBTAIN THE DELTA TIME
					; Save double precision time to LONGTIME

		EXTEND			# OBTAIN THE 2CADR
# Page 1234
		NDX	Q		; Index by Q (return address + 1)
		DCA	0		; Load 2CADR following TC LONGCALL
		DXCH	LONGCADR	; Save task 2CADR to LONGCADR

		EXTEND			# NOW GO TO THE APPROPRIATE SWITCHED BANK
		DCA	LGCL2CDR	# FOR THE REST OF LONGCALL
					; Load 2CADR of LNGCALL2
		DTCB			; Transfer control to switched bank

		EBANK=	LST1
LGCL2CDR	2CADR	LNGCALL2	; 2CADR pointer to continuation

# *** THE FOLLOWING MAY BE IN A SWITCHED BANK, INCLUDING ITS ERASABLE ***

		BANK	01
		COUNT	01/WAIT

; ============================================================================
; LNGCALL2 - Long Call Setup and Return Address Save
;
; COMMENT-ONLY READERS: The computer now saves where to return after the long
; wait completes. It stores the bank information and calculates the correct
; return address (skipping past the 2CADR task specification in the calling
; routine).
;
; CODE-ALONG READERS: Continuation of LONGCALL in switched BANK 01. Saves
; current L (contains BBCON from caller) to LONGEXIT+1 for proper bank return.
; Adds TWO to Q to skip past 2CADR (which is 2 words), giving return address
; at caller's Q+2 position. Stores in LONGEXIT. Falls through to LONGCYCL
; which performs the time comparison and scheduling logic.
; ============================================================================

LNGCALL2	LXCH	LONGEXIT +1	# SAVE THE CORRECT BB FOR RETURN
					; Save bank (BBCON) for return
		CA	TWO		# OBTAIN THE RETURN ADDRESS
		ADS	Q		; Add 2 to Q (skip 2CADR)
		TS	LONGEXIT	; Save return address

; ============================================================================
; LONGCYCL - Long Wait Cycle Time Reduction (Waitlist Task Entry)
;
; COMMENT-ONLY READERS: This task runs when a long wait is broken into cycles.
; The computer checks: "Can I subtract about 81.92 seconds (1.37 minutes) from
; the remaining wait time?" If yes, it schedules another cycle. If no, the
; remaining time is short enough for a single WAITLIST entry, so it schedules
; the final wait directly. This cycle-by-cycle approach prevents timer overflow
; on waits longer than WAITLIST's maximum ~162.5 second capacity.
;
; CODE-ALONG READERS: Task entry point for long wait cycle management. Algorithm:
; (1) Subtract DPBIT14 (~81.92 sec = 8192 centiseconds) from double-precision
; LONGTIME. (2) Test LONGTIME+1 with CCS: if positive, >81.92 sec remains, go
; to MUCHTIME to schedule another cycle. (3) If LONGTIME+1 <= 0, test LONGTIME
; with CCS: if positive, go MUCHTIME. (4) If both non-positive, fall through to
; LASTTIME. Double-word logic handles unsigned DP values correctly despite lack
; of sign correction in DAS. DPBIT14 constant = 00000 20000 octal (bit 14 set).
; ============================================================================

# *** WAITLIST TASK LONGCYCL ***
LONGCYCL	EXTEND			# CAN WE SUCCESFULLY TAKE ABOUT 1.25
					; MINUTES OFF OF LONGTIME
		DCS	DPBIT14		# Subtract ~81.92 seconds
		DAS	LONGTIME	; From remaining wait time (DP)

		CCS	LONGTIME +1	# THE REASONING BEHIND THIS PART IS
					; Test low-order word
		TCF	MUCHTIME	# Positive: >81.92 sec remains
					# INVOLVED, TAKING INTO ACCOUNT THAT THE
					# WORDS MAY NOT BE SIGNED CORRECTED (DP
					# BASIC INSTRUCTIONS
					# DO NOT SIGN CORRECT) AND THAT WE SUBTRAC-
					# TED BIT14 (1 OVER HALF THE POS. VALUE
					# REPRESENTABLE IN SINGLE WORD)
		NOOP			# CAN'T GET HERE **********
		TCF	+1		; Zero or negative: test high word
		CCS	LONGTIME	; Test high-order word
		TCF	MUCHTIME	; Positive: >81.92 sec remains
					; Fall through to LASTTIME: <81.92 sec
DPBIT14		OCT	00000		; Double-precision constant
		OCT	20000		; Bit 14 = 8192 decimal = ~81.92 sec

; ============================================================================
; LASTTIME - Final Wait Cycle Scheduling
;
; COMMENT-ONLY READERS: The remaining wait time is now short enough for a single
; timer entry. The computer adds back the 81.92 seconds it just subtracted (to
; get the true remaining time), then schedules one final wait. When that wait
; completes, the task you originally requested will execute. This is the end of
; the long-wait cycle chain.
;
; CODE-ALONG READERS: Final wait cycle when LONGTIME < DPBIT14 (~81.92 sec).
; Algorithm: (1) Load BIT14 (restore the amount subtracted) and add to
; LONGTIME+1, yielding actual remaining delta-time. (2) Call WAITLIST with
; that delta-time in A. (3) Specify GETCADR as the task to execute (which will
; retrieve and dispatch the original LONGCADR task). This completes the long
; wait by scheduling the final segment within WAITLIST's capacity. EBANK=LST1
; ensures correct erasable bank for WAITLIST access.
; ============================================================================

					# LONGCALL
LASTTIME	CA	BIT14		# GET BACK THE CORRECT DELTA TFOR WAITLIST
					; Restore subtracted amount
		ADS	LONGTIME +1	; True remaining time now in A
		TC	WAITLIST	; Schedule final wait
		EBANK=	LST1		; Set erasable bank
		2CADR	GETCADR		# THE ENTRY TO OUR LONGCADR
					; Task: retrieve original 2CADR

; ============================================================================
; LONGRTRN - Long Call Return Path Setup
;
; COMMENT-ONLY READERS: The computer now sets up where to return after each
; wait cycle completes. The very first return goes back to your calling program
; (allowing your code to continue). All subsequent cycle completions return to
; TASKOVER (ending the internal cycle task cleanly). This ensures the cyclic
; wait mechanism operates transparently to your program.
;
; CODE-ALONG READERS: Return path configuration for LONGCALL cycles. Algorithm:
; (1) Load TSKOVCDR (GENADR of TASKOVER). (2) DXCH with LONGEXIT, swapping
; TASKOVER address into LONGEXIT and original caller's return address into A/L.
; (3) DTCB transfers control back to caller via A/L. Result: First exit returns
; to original caller (LONGCALL appears atomic). Subsequent LONGCYCL completions
; find TASKOVER in LONGEXIT, ending cycle tasks properly. LONGEXIT modified
; in-place for future cycles.
; ============================================================================

LONGRTRN	CA	TSKOVCDR	# SET IT UP SO THAT ONLY THE FIRST EXIT IS
					; Load TASKOVER address
# Page 1235
		DXCH	LONGEXIT	# TO THE CALLER OF LONGCALL
					; Swap: A/L = caller return, LONGEXIT = TASKOVER
		DTCB			# THE REST ARE TO TASKOVER
					; Transfer control to caller

; ============================================================================
; MUCHTIME - Schedule Another Long Wait Cycle
;
; COMMENT-ONLY READERS: More than 81.92 seconds still remain in the wait. The
; computer schedules another cycle: wait 81.92 seconds, then come back to
; LONGCYCL to test again. This continues until the remaining time is short
; enough for a final direct wait. Each cycle consumes about 1.37 minutes of
; the total wait time.
;
; CODE-ALONG READERS: Path taken when LONGTIME >= DPBIT14 after subtraction
; (meaning original LONGTIME was >= 2*DPBIT14 ≈ 163.84 sec). Algorithm:
; (1) Load BIT14 (~81.92 sec) into A. (2) Call WAITLIST to schedule 81.92-sec
; wait. (3) Specify LONGCYCL as task to execute after wait. (4) TCF to LONGRTRN
; to set up return paths. Result: Another cycle scheduled. LONGCYCL will execute
; in ~81.92 sec, subtract another DPBIT14, test again. Process repeats until
; LONGTIME small enough for LASTTIME path. Enables arbitrarily long waits.
; ============================================================================

MUCHTIME	CA	BIT14		# WE HAVE OVER OUR ABOUT 1.25 MINUTES
					; Load ~81.92 sec for next cycle
		TC	WAITLIST	# SO SET UP FOR ANOTHER CYCLE THROUGH HERE
					; Schedule another cycle wait
		EBANK=	LST1		; Set erasable bank
		2CADR	LONGCYCL	; Task: come back to LONGCYCL
					; (test time remaining again)

		TCF	LONGRTRN	# NOW EXIT PROPERLY
					; Set up return paths

; ============================================================================
; GETCADR - Retrieve and Execute Original Long-Called Task
;
; COMMENT-ONLY READERS: The long wait is finally complete! The computer now
; retrieves the task you originally requested (which has been saved all this
; time) and transfers control to it. Your task executes exactly as if the long
; wait had been a single atomic operation. The cycling mechanism is completely
; transparent to your code.
;
; CODE-ALONG READERS: Final step in LONGCALL chain. Entry as waitlist task
; after LASTTIME wait completes. Algorithm: (1) DXCH LONGCADR loads the
; original caller's 2CADR (saved by LONGCALL at entry) into A/L. (2) DTCB
; performs double-precision transfer control via bank call, switching to
; specified bank and transferring to specified address. Result: Original
; task executes with no knowledge of intermediate cycling. LONGCALL mechanism
; complete. Task ends normally via TASKOVER when finished.
; ============================================================================

# *** WAITLIST TASK GETCADR ***
GETCADR		DXCH	LONGCADR	# GET THE LONGCALL THAT WE WISHED TO START
					; Load original task's 2CADR
		DTCB			# AND TRANSFER CONTROL TO IT
					; Execute original task

; ============================================================================
; TSKOVCDR - TASKOVER Address Constant
;
; CODE-ALONG READERS: GENADR constant providing address of TASKOVER for
; return path setup in LONGRTRN. Used to configure LONGEXIT for subsequent
; cycle task completions.
; ============================================================================

TSKOVCDR	GENADR	TASKOVER	; TASKOVER address for return setup
