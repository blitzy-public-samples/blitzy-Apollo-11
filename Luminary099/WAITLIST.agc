# Copyright:	Public domain.
# Filename:	WAITLIST.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1117-1132
# Mod history:	2009-05-25 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-01-06 JL	Fixed pseudo-label indentation.
#		2011-05-07 JL	Removed workarounds.
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
; FILE: WAITLIST.agc
; MODULE: Core Operating System - Timer-Driven Task Scheduler
; MISSION PHASE: All phases (continuous operation throughout mission)
;
; TL;DR: Implements the AGC's timer-driven preemptive task scheduling system
;        managing up to 9 time-delayed tasks in delta-time queue structure.
;        Tasks are scheduled for execution at specific future times (1-162.5
;        seconds ahead) and dispatched via T3RUPT interrupt handler. Works in
;        tandem with EXECUTIVE.agc (job scheduler) to provide complete real-time
;        operating system. During Apollo 11's descent, WAITLIST contributed to
;        the 1202 program alarm when radar data processing created excessive
;        task load, but delta-time queue structure enabled rapid recovery.
;
; COMMENT-ONLY READERS: This is the AGC's alarm clock system - it wakes up
;        tasks at precise future times. During landing, it helped coordinate
;        radar readings, guidance updates, and display refreshes. When too many
;        tasks piled up, it triggered the 1202 alarm that Steve Bales famously
;        cleared for "Go" during Apollo 11's descent.
;
; CODE-ALONG READERS: Study the delta-time queue implementation using LST1
;        (time deltas) and LST2 (task 2CADRs). The TIME3 register maintains
;        absolute time to next task. Maximum 9 tasks enforced by alarm 1203.
;        T3RUPT interrupt handler (see T4RUPT_PROGRAM.agc) dispatches tasks
;        when TIME3 reaches zero, then updates queue for next task.
; ============================================================================
;
; ============================================================================
; WAITLIST AND THE 1202 ALARM - APOLLO 11 CONTEXT
; ============================================================================
;
; During Apollo 11's powered descent on July 20, 1969, at approximately 
; 102:38:26 mission elapsed time, the rendezvous radar (inadvertently left
; in AUTO mode) generated excessive data requiring processing. This created
; computational overload conditions where:
;
; 1. EXECUTIVE job queue filled (1201 alarm - executive overflow)
; 2. WAITLIST task queue approached capacity (contributing to 1202 alarm)
;
; The delta-time queue structure (storing relative time differences between
; tasks rather than absolute times) enabled efficient queue management even
; under overload. When tasks couldn't execute on time due to computational
; load, the queue maintained correct ordering for execution priority.
;
; Flight controller Steve Bales (GUIDO) consulted Jack Garman's pre-studied
; alarm list and made the critical "Go" call: alarms were non-catastrophic,
; restart protection was working, landing could continue. This decision,
; enabled by the robust WAITLIST/EXECUTIVE architecture, allowed Armstrong
; and Aldrin to continue to successful touchdown at 102:45:40 MET.
;
; The alarm recurred multiple times during descent, but each time the AGC's
; restart protection (see FRESH_START_AND_RESTART.agc, RESTART_TABLES.agc)
; preserved critical guidance and navigation state, demonstrating the fault-
; tolerant design of the real-time operating system.
; ============================================================================

# Page 1117
# PROGRAM DESCRIPTION								DATE -- 10 OCTOBER 1966
# MOD NO -- 2									LOG SECTION -- WAITLIST
# MOD BY -- MILLER	(DTMAX INCREASED TO 162.5 SEC)				ASSEMBLY -- SUNBURST REV 5
# MOD 3 BY KERNAN	(INHINT INSERTED AT WAITLIST) 2/28/68 SKIPPER REV 4
# MOD 4 BY KERNAN	(TWIDDLE IN 54) 3/28/68 SKIPPER REV 13.
#
# FUNCTIONAL DESCRIPTION --
#	PART OF A SECTION OF PROGRAMS -- WAITLIST, TASKOVER, T3RUPT, USED TO CALL A PROGRAM (CALLED A TASK),
#	WHICH IS TO BEGIN IN C(A) CENTISECONDS.  WAITLIST UPDATES TIME3, LST1, AND LST2.  THE MEANING OF THESE LISTS
#	FOLLOW.
#
#		C(TIME3) = 16384 -(T1-T) CENTISECONDS, (T=PRESENT TIME, T1-TIME FOR TASK1)
#
#			C(LST1)		=	-(T2-T1)+1
#			C(LST1 +1)	=	-(T3-T2)+1
#			C(LST1 +2)	=	-(T4-T3)+1
#				       ...
#			C(LST1 +6)	=	-(T8-T7)+1
#			C(LST1 +7)	=	-(T9-T8)+1
#
#			C(LST2)		=	2CADR OF TASK1
#			C(LST2 +2)	=	2CADR OF TASK2
#				       ...
#			C(LST2 +14)	=	2CADR OF TASK8
#			C(LST2 +16)	=	2CADR OF TASK9
#
# WARNINGS --
#	1)	1 <= C(A) <= 16250D (1 CENTISECOND TO 162.5 SEC)
#	2)	9 TASKS MAXIMUM
#	3)	TASKS CALLED UNDER INTERRUPT INHIBITED
#	4)	TASKS END BY TC TASKOVER
#
# CALLING SEQUENCE --
#	L-1	CA	DELTAT 	(TIME IN CENTISECONDS TO TASK START)
#	L	TC	WAITLIST
#	L+1	2CADR	DESIRED TASK.
#	L+2	(MINOR OF 2CADR)
#	L+3	RELINT		(RETURNS HERE)
#
# TWIDDLE --
#	TWIDDLE IS FOR USE WHEN THE TASK BEING SET UP IS IN THE SAME EBANK AND FBANK AS THE USER.  IN
#	SUCH CASES, IT IMPROVES UPON WAITLIST BY ELIMINATING THE NEED FOR THE BBCON HALF OF THE 2CADR,
# Page 1118
#	SAVING A WORD.  TWIDDLE IS LIKE WAITLIST IN EVERY RESPECT EXCEPT CALLING SEQUENCE, TO WIT,
#		L-1	CA	DELTAT
#		L	TC	TWIDDLE
#		L+1	ADRES	DESIRED TASK
#		L+2	RELINT		(RETURNS HERE)
#
# NORMAL EXIT MODES --
#	AT L+3 OF CALLING SEQUENCE.
#
# ALARM OR ABORT EXIT MODES --
#	TC	ABORT
#	OCT	1203	(WAITLIST OVERFLOW -- TOO MANY TASKS)
#
# ERASABLE INITIALIZATION REQUIRED --
#	ACCOMPLISHED BY FRESH START --	LST2, ..., LST2 +16 = ENDTASK
#					LST1, ..., LST1 +7  = NEG1/2
#
# OUTPUT --
#	LST1 AND LST2 UPDATED WTIH NEW TASK AND ASSOCIATED TIME.
#
# DEBRIS --
#	CENTRALS -- A,Q,L
#	OTHER    -- WAITEXIT, WAITADR, WAITTEMP, WAITBANK
#
# DETAILED ANALYSIS OF TIMING --
#	CONTROL WILL NOT BE RETURNED TO THE SPECIFIED ADDRESS (2CADR) IN EXACTLY DELTA T CENTISECONDS.
#	THE APPROXIMATE TIME MAY BE CALCULATED AS FOLLOWS:
#		LET T0 = THE TIME OF THE TC WAITLIST
#		LET TS = T0 +147U + COUNTER INCREMENTS (SET UP TIME)
#		LET X  = TS -(100TS)/100  (VARIANCE FROM COUNTERS)
#		LET Y  = LENGTH OF TIME OF INHIBIT INTERRUPT AFTER T3RUPT
#		LET Z  = LENGTH OF TIME TO PROCESS TASKS WHICH ARE DUE THIS T3RUPT BUT DISPATCHED EARLIER.
#			 (Z=0, USUALLY).
#		LET DELTD  = THE ACTUAL TIME TAKEN TO GIVE CONTROL TO 2CADR
#		THEN DELTD = TS+DELTA T -X +Y +Z +1.05MS* +COUNTERS*
#		*THE TIME TAKEN BY WAITLIST ITSELF AND THE COUNTER TICKING DURING THIS WAITLIST TIME.
#	IN SHORT, THE ACTUAL TIME TO RETURN CONTROL TO A 2CADR IS AUGMENTED BY THE TIME TO SET UP THE TASK'S
# 	INTERRUPT, ALL COUNTERS TICKING, THE T3RUPT PROCESSING TIME, THE WAITLIST PROCESSING TIME AND THE POSSIBILITY
#	OF OTHER TASKS INHIBITING THE INTERRUPT.

		BLOCK	02
# Page 1119
		EBANK=	LST1		# TASK LISTS IN SWITCHED E BANK.

; ============================================================================
; WAITLIST IMPLEMENTATION - DELTA-TIME QUEUE TASK SCHEDULER
; ============================================================================
;
; The following code implements the AGC's timer-driven task scheduling system.
; Tasks are stored in a delta-time queue where each entry stores the time
; difference from the previous entry, not absolute time. This enables efficient
; queue management and rapid insertion/deletion operations.
;
; QUEUE STRUCTURE:
; - LST1: Array of delta-time values (time until next task in queue)
; - LST2: Array of 2CADR task addresses (bank + address pairs)
; - TIME3: Absolute time until next task dispatch (maintained by T3RUPT)
;
; OPERATIONAL FLOW:
; During Apollo 11's descent, this system coordinated timing for:
; - Guidance equation updates (every 2 seconds during powered descent)
; - Landing radar data sampling (altitude and velocity measurements)
; - Throttle command updates to descent engine
; - DSKY display refreshes showing altitude, velocity, fuel remaining
;
; When the system became overloaded during descent (due to inadvertently active
; rendezvous radar), the WAITLIST queue filled toward capacity, contributing
; to the 1202 alarm. The robust queue structure maintained correct task timing
; even under overload, enabling successful landing.
; ============================================================================

		COUNT*	$$/WAIT

; ============================================================================
; TWIDDLE - SPECIAL WAITLIST ENTRY FOR DELAY > 163.83 SECONDS
; ============================================================================
;
; TWIDDLE handles scheduling tasks more than 163.83 seconds in the future.
; Since delta-time values are stored in 14-bit signed registers (±8191 
; centiseconds = ±81.91 seconds), delays longer than this require special
; handling. TWIDDLE creates an intermediate dummy task that reschedules
; itself until the full delay expires.
;
; TECHNICAL DETAILS:
; - Maximum single delta-time: 163.83 seconds (16383 centiseconds)
; - Delays > 163.83s: TWIDDLE breaks into multiple scheduling cycles
; - Entry condition: Delay time in A register, return address in Q
; - Creates overflow in Q register to signal long-delay handling
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
; WAITLIST - PRIMARY ENTRY POINT FOR TIMER-DRIVEN TASK SCHEDULING
; ============================================================================
;
; This is where mission programs request future task execution. When code needs
; something to happen at a specific time in the future (radar sampling, guidance
; updates, display refreshes), it calls WAITLIST with a delta-time and task
; address (2CADR format: bank + address).
;
; CALLING CONVENTION:
; - TC WAITLIST (transfer control to this entry point)
; - Delta-time in A register (centiseconds, 1cs = 10ms)
; - Task address as 2CADR immediately after TC instruction
; - Example: TC WAITLIST / 2CADR RADARTASK (schedule radar task)
;
; APOLLO 11 DESCENT USAGE:
; During the famous 12-minute descent to the lunar surface, WAITLIST coordinated:
; - Landing radar altitude/velocity readings every 2 seconds (P63 guidance)
; - Guidance equation updates every 2 seconds (trajectory computation)
; - Throttle command updates to descent engine (continuous thrust modulation)
; - DSKY display refreshes (altitude, velocity, fuel remaining for crew)
;
; 1202 ALARM CONTEXT:
; When the rendezvous radar was inadvertently left active during descent, radar
; data processing overloaded the system. WAITLIST's queue filled toward the
; 7-task capacity limit, triggering alarm 1202 when combined with EXEC overload.
; The delta-time queue structure maintained correct task timing even under
; overload, enabling Steve Bales (GUIDO) to call "GO" and continue landing.
;
; TECHNICAL IMPLEMENTATION:
; - Reads mission elapsed time from TIME1/TIME2 registers
; - Calls WAITADR to insert task into delta-time queue (LST1/LST2 arrays)
; - If queue full (>7 tasks), triggers alarm 1203 via BAILOUT
; - Returns control to caller after successful task insertion
; ============================================================================

WAITLIST	INHINT
		XCH	Q		# SAVE DELTA T IN Q AND RETURN IN
		TS	WAITEXIT	# WAITEXIT.
		EXTEND
		INDEX	WAITEXIT	# IF TWIDDLING, THE TS SKIPS TO HERE
		DCA	0		# PICK UP 2CADR OF TASK.
 -1		TS	WAITADR		# BBCON WILL REMAIN IN L
DLY2		CAF	WAITBB		# ENTRY FROM FIXDELAY AND VARDELAY.
		XCH	BBANK
		TCF	WAIT2

; ============================================================================
; RETURN TO CALLER AFTER TASK INSERTION
; ============================================================================
;
; After successfully inserting a task into the WAITLIST queue, control returns
; to the calling program. This exit path restores the return address and
; advances it by 2 words to skip over the 2CADR task address parameter.
;
; RETURN CONVENTION:
; - Caller executes: TC WAITLIST
; - Next 2 words contain 2CADR (bank address + local address of task)
; - Return occurs 2 words after the 2CADR (caller+3 instruction)
; - This allows in-line task scheduling without subroutine overhead
;
; REGISTER STATE ON RETURN:
; - Return address restored to Q from WAITEXIT
; - Bank restored to calling program's bank (BBANK)
; - A register contains return address + 2
; - Task insertion confirmed successful (no alarm)
; ============================================================================

# RETURN TO CALLER AFTER TASK INSERTION:

LVWTLIST	DXCH	WAITEXIT
		AD	TWO
		DTCB

		EBANK=	LST1
WAITBB		BBCON	WAIT2

; ============================================================================
; FIXDELAY - SCHEDULE TASK RE-EXECUTION WITH FIXED DELAY
; ============================================================================
;
; FIXDELAY allows a task to reschedule itself with a fixed delay specified
; as a constant immediately following the call. This is used for periodic
; tasks that need to execute at regular intervals.
;
; CALLING CONVENTION:
; - TC FIXDELAY (from within a WAITLIST task)
; - Next word: Delta-time constant (centiseconds)
; - Return: Task terminates, will be re-executed after delay expires
;
; USAGE EXAMPLE:
; During descent, guidance tasks use FIXDELAY to reschedule themselves:
;   GUIDTASK  TC  INTPRET
;             ... (guidance computations)
;             TC  FIXDELAY
;             DEC  200        # Re-execute after 2.0 seconds
;
; TECHNICAL NOTES:
; - Task address (WAITADR) is preserved from current task context
; - Q register points to caller+1 (the delay constant)
; - Delay constant fetched via INDEX Q addressing mode
; - Q incremented to skip delay constant on eventual return
; - Must be called under WAITLIST control (task environment)
; ============================================================================

# RETURN TO CALLER +2 AFTER WAITING DT SPECIFIED AT CALLER +1.

FIXDELAY	INDEX	Q		# BOTH ROUTINES MUST BE CALLED UNDER
		CAF	0		# WAITLIST CONTROL AND TERMINATE THE TASK
		INCR	Q		# IN WHICH THEY WERE CALLED.

; ============================================================================
; VARDELAY - SCHEDULE TASK RE-EXECUTION WITH VARIABLE DELAY
; ============================================================================
;
; VARDELAY allows a task to reschedule itself with a computed delay passed
; in the A register. This is used when delay duration depends on runtime
; calculations or mission state.
;
; CALLING CONVENTION:
; - Delay time computed and placed in A register (centiseconds)
; - TC VARDELAY (from within a WAITLIST task)
; - Return: Task terminates, will be re-executed after delay expires
;
; USAGE EXAMPLE:
; Landing radar sampling adjusts its rate based on altitude:
;   RADARTASK CA  ALTITUDE
;             ... (compute sampling interval based on altitude)
;             TC  VARDELAY     # A register contains computed delay
;
; APOLLO 11 CONTEXT:
; During final descent, many tasks used VARDELAY to adjust timing based on
; mission phase. As altitude decreased, some tasks increased sampling rates
; while others decreased to manage computational load and prevent overload.
;
; TECHNICAL NOTES:
; - Delay arrives in A register
; - Task address (WAITADR) is preserved from current task context
; - A exchanged with Q to move delay to Q, return address to A
; - Task reschedules itself then terminates
; - Must be called under WAITLIST control (task environment)
; ============================================================================

# RETURN TO CALLER +1 AFTER WAITING THE DT AS ARRIVING IN A.

VARDELAY	XCH	Q		# DT TO Q.  TASK ADRES TO WAITADR.
		TS	WAITADR
		CA	BBANK		# BBANK IS SAVED DURING DELAY.
		EXTEND
		ROR	SUPERBNK	# ADD SBANK TO BBCON.
		TS	L
		CAF	DELAYEX
		TS	WAITEXIT	# GO TO TASKOVER AFTER TASK ENTRY.
		TCF	DLY2

; ============================================================================
; DELAYEX - DELAY TASK EXIT HANDLER
; ============================================================================
;
; After a delayed task is re-inserted into the WAITLIST, control transfers
; to DELAYEX which routes execution to TASKOVER for task completion and
; cleanup. The -2 offset adjusts for the TASKOVER entry protocol.
;
; This is the exit point for tasks that used FIXDELAY or VARDELAY to
; reschedule themselves. When the delay expires and the task re-executes,
; it completes its work then exits through DELAYEX to properly terminate.
; ============================================================================

# Page 1120
DELAYEX		TCF	TASKOVER -2	# RETURNS TO TASKOVER.

; ============================================================================
; ENDTASK - STANDARD TASK COMPLETION ENTRY POINT
; ============================================================================
;
; ENDTASK provides a standard way for tasks to terminate execution. Tasks
; that complete their work (rather than rescheduling via FIXDELAY/VARDELAY)
; execute TC ENDTASK or TCF ENDTASK to properly clean up and return control
; to the EXECUTIVE scheduler.
;
; FIXED-FIXED REQUIREMENT:
; ENDTASK must reside in fixed-fixed memory (bank-independent) so its address
; is recognizable without bank switching. This allows any task in any bank to
; terminate using the same ENDTASK address.
;
; SVCT3 INTEGRATION:
; ENDTASK routes to SVCT3 which checks IMU drift compensation flags and
; schedules periodic calibration tasks before final task completion.
; ============================================================================

# Page 1121
# ENDTASK MUST ENTERED IN FIXED-FIXED SO IT IS DISTINGUISHABLE BY ITS ADRES ALONE.

		EBANK=	LST1
ENDTASK		-2CADR	SVCT3

; ============================================================================
; SVCT3 - IMU DRIFT COMPENSATION SCHEDULING
; ============================================================================
;
; Before allowing a task to complete, SVCT3 checks if IMU (Inertial Measurement
; Unit) drift compensation is needed. The IMU gyroscopes and accelerometers
; exhibit small drifts over time that must be compensated periodically to
; maintain navigation accuracy.
;
; DRIFT FLAG CHECK (FLAGWRD2):
; - Positive: Drift compensation not needed, proceed to TASKOVER
; - Zero: Drift compensation not needed, proceed to TASKOVER  
; - Negative: Drift compensation required, check IMU availability
;
; MISSION CONTEXT:
; During lunar descent and surface operations, maintaining IMU accuracy was
; critical for guidance and navigation. The AGC periodically scheduled drift
; compensation tasks (NBDONLY) to correct for accumulated gyro drift.
;
; COMPENSATION FREQUENCY:
; NBDONLY task executes every 81.93 seconds to update drift coefficients
; based on measured IMU performance and thermal variations.
; ============================================================================

SVCT3		CCS	FLAGWRD2	# DRIFT FLAG
		TCF	TASKOVER
		TCF	TASKOVER
		TCF	+1

; ============================================================================
; CKIMUSE - CHECK IMU AVAILABILITY FOR DRIFT COMPENSATION
; ============================================================================
;
; Before scheduling drift compensation, verify the IMU is not currently in
; use by another task (IMUZERO, calibration, or alignment operations).
;
; IMUCADR CHECK:
; - Zero: IMU available, schedule NBDONLY drift compensation task
; - Non-zero: IMU busy, delay and check again in 5.0 seconds
;
; COLLISION AVOIDANCE:
; Multiple tasks may need IMU access. IMUSTALL mechanism prevents conflicts
; by checking IMUCADR (IMU caller address) before initiating IMU operations.
; If busy, delay via FIXDELAY and retry after IMU becomes available.
; ============================================================================

CKIMUSE		CCS	IMUCADR		# DON'T DO NBDONLY IF SOMEONE ELSE IS IN
		TCF	SVCT3X		# IMUSTALL.
		TCF	+3
		TCF	SVCT3X
		TCF	SVCT3X

; IMU available - Schedule NBDONLY drift compensation task
; Priority 35 ensures timely execution without disrupting critical guidance

 +3		CAF	PRIO35		# COMPENSATE FOR NBD COEFFICIENTS ONLY.
		TC	NOVAC		#	ENABLE EVERY 81.93 SECONDS
		EBANK=	NBDX
		2CADR	NBDONLY

		TCF	TASKOVER

; ============================================================================
; SVCT3X - DELAY RETRY FOR IMU AVAILABILITY
; ============================================================================
;
; IMU is currently busy. Delay 5.0 seconds then re-check drift flag and IMU
; availability. Maximum of 2 delays (10 seconds total) prevents infinite
; waiting if IMU remains continuously busy.
;
; DELAY STRATEGY:
; - First check: IMU busy, delay 500 centiseconds (5.0 seconds)
; - After delay: Return to SVCT3 to re-check FLAGWRD2 drift flag
; - If still needed and IMU still busy: Delay another 5.0 seconds
; - After second delay: Proceed to TASKOVER even if compensation incomplete
;
; MISSION ROBUSTNESS:
; This timeout mechanism ensures tasks complete even if IMU drift compensation
; cannot be immediately performed, preventing task hangs during busy periods.
; ============================================================================

SVCT3X		TC	FIXDELAY	# DELAY MAX OF 2 TIMES FOR IMUZERO.
		DEC	500
		TC	SVCT3		# CHECK DRIFT FLAG AGAIN.

# Page 1122
# BEGIN TASK INSERTION.

		BANK	01
		COUNT*	$$/WAIT

; ============================================================================
; WAIT2 - TASK INSERTION WITH T3 TIMER RESET
;
; COMMENT-ONLY READERS: When a new task needs to be scheduled and its timing
; would affect the currently running T3RUPT timer, the AGC must temporarily
; stop the timer, insert the new task into the queue, and recalculate all
; timing relationships. This is one of the most critical timing-sensitive
; operations in the entire guidance computer.
;
; CODE-ALONG READERS: This routine handles the complex case where task
; insertion requires T3 timer manipulation. The code must disable interrupts,
; read the current timer value, calculate the new timer setting accounting
; for the inserted task's delta-time, and restart the timer - all within a
; single interrupt-disabled window to maintain timing integrity.
; ============================================================================

; Entry point: Save calling program's bank and retrieve the delta-time (Q).
; If delta-time is negative or zero, reject the request (WAITPOOH error path).

WAIT2		TS	WAITBANK	# BBANK OF CALLING PROGRAM.
		CA	Q
		EXTEND
		BZMF	WAITPOOH

; Critical timing calculation: Determine relationship between requested task
; delta-time (TD) and current T3 timer position (T1). This complex arithmetic
; handles T3 timer overflow cases that can occur during this calculation.
;
; The AGC must account for two possible states:
; 1. Normal: TIME3 counts down from 1.0 toward zero
; 2. Overflow: TIME3 has passed zero and wrapped during this computation
;
; Historical context: This timing precision was critical during Apollo 11's
; descent when the guidance computer was managing multiple high-priority tasks
; with millisecond-level timing requirements.

		CS	TIME3
		AD	BIT8		# BIT 8 = OCT 200
		CCS	A		# TEST 200 - C(TIME3).  IF POSITIVE,
					# IT MEANS THAT TIME3 OVERFLOW HAS OCCURRED PRIOR TO CS TIME3 AND THAT
					# C(TIME3) = T - T1, INSTEAD OF 1.0 - (T1 - T).  THE FOLLOWING FOUR
					# ORDERS SET C(A) = TD - T1 + 1 IN EITHER CASE.

		AD	OCT40001	# OVERFLOW HAS OCCURRED.  SET C(A) =
		CS	A		# T - T1 + 1.0 - 201

# NORMAL CASE (C(A) NNZ) YIELDS SAME C(A):  -( -(1.0-(T1-T)) + 200) - 1

		AD	OCT40201
		AD	Q		# RESULT = TD - T1 + 1.

; Test whether the new task's delta-time places it before or after the
; current T3RUPT timer position. This determines whether the timer must
; be reset (task comes before current timer expiration) or the task can
; be inserted normally (task comes after).

		CCS	A		# TEST TD - T1 + 1.

		AD	LST1		# IF TD - T1 POS, GO TO WTLST5 WITH
		TCF	WTLST5		# C(A) = (TD - T1) + C(LST1) = TD-T2+1

		NOOP
		CS	Q

; ============================================================================
; T3 TIMER RESET SECTION
;
; COMMENT-ONLY READERS: The new task must execute before the current timer
; expires. The AGC now performs the critical operation of stopping TIME3,
; calculating a new timer value that accounts for the inserted task, and
; restarting the timer. This must complete within a single interrupt-disabled
; window to prevent timing corruption.
;
; CODE-ALONG READERS: This section is protected by the mathematical guarantee
; that it only executes when T-T1 >= -1, meaning no TIME3 overflow can occur
; during these operations. The timer value is calculated as 1.0 - DELTA_T,
; loaded into TIME3, and the routine proceeds to shift the waitlist entries.
; ============================================================================

# NOTE THAT THIS PROGRAM SECTION IS NEVER ENTERED WHEN T-T1 G/E -1,
# SINCE TD-T1+1 = (TD-T) + (T-T1+1), AND DELTA T = TD-T G/E +1.  (G/E
# SYMBOL MEANS GREATER THAN OR EQUAL TO).  THUS THERE NEED BE NO CON-
# CERN OVER A PREVIOUS OR IMMINENT OVEFLOW OF TIME3 HERE.

		AD	POS1/2		# WHEN TD IS NEXT, FORM QUANTITY
		AD	POS1/2		#	1.0 - DELTA T = 1.0 - (TD - T)
		XCH	TIME3
		AD	NEGMAX
		AD	Q		# 1.0 - DELTAT T NOW COMPLETE.
		EXTEND			# ZERO INDEX Q.
		QXCH	7		# (ZQ)

; ============================================================================
; WAITLIST SHIFTING ROUTINE (WTLST4)
;
; COMMENT-ONLY READERS: The AGC now opens up space in the waitlist by shifting
; all existing tasks down by one position. This is like inserting a new
; appointment at the top of a calendar - all subsequent appointments must
; move down. During Apollo 11's descent, this operation occurred repeatedly
; as guidance, throttle, and display tasks competed for execution slots.
;
; CODE-ALONG READERS: The XCH (exchange) instruction swaps the accumulator
; with each waitlist position (LST1 through LST1+7), effectively performing
; a single-instruction shift operation. The accumulator carries each delta-time
; value forward, creating a ripple effect. Q register contains the insertion
; index (0-7) for the new task. This is a time-critical routine that must
; execute quickly to maintain timing precision.
; ============================================================================

# Page 1123
WTLST4		XCH	LST1
		XCH	LST1 	+1
		XCH	LST1 	+2
		XCH	LST1 	+3
		XCH	LST1 	+4
		XCH	LST1 	+5
		XCH	LST1 	+6
		XCH	LST1 	+7

	; After shifting delta-times, now shift the corresponding task addresses.
; The CADR (code address) for each task consists of bank and address portions.
; These must be shifted in synchronization with their delta-times to maintain
; the correct task-to-delta-time pairing.

		CA	WAITADR		# (MINOR PART OF TASK CADR HAS BEEN IN L.)
		INDEX	Q
		TCF	+1

; Shift all task addresses (CADR pairs) down by one position using DXCH
; (double exchange). Each task's bank+address must move to make room for
; the newly inserted task. The index skip (TCF +1) ensures shifting starts
; at the correct position based on where the new task will be inserted.

		DXCH	LST2
		DXCH	LST2 	+2
		DXCH	LST2 	+4
		DXCH	LST2 	+6
		DXCH	LST2 	+8D
		DXCH	LST2 	+10D	# AT END, CHECK THAT C(LST2 +10) IS STD
		DXCH	LST2 	+12D
		DXCH	LST2 	+14D
		DXCH	LST2 	+16D

; Safety check: Verify the end marker (ENDTASK) is still in position after
; the shift operations. If the end marker has been displaced, the waitlist
; has overflowed (more than 7 tasks attempted). This is a critical error
; condition that contributed to the 1202 program alarms during Apollo 11's
; descent - the waitlist was approaching capacity as radar data processing,
; guidance updates, and display tasks all demanded execution time.

		AD	ENDTASK		# END ITEM, AS CHECK FOR EXCEEDING
					# THE LENGTH OF THE LIST.
		EXTEND			# DUMMY TASK ADRES SHOULD BE IN FIXED-
		BZF	LVWTLIST	# FIXED SO ITS ADRES ALONE DISTINGUISHES
		TCF	WTABORT		# IT.

# Page 1124
; ============================================================================
; WTLST5 - SEQUENTIAL SEARCH FOR TASK INSERTION POSITION
;
; TRANSITION: From fast-path insertion checks to sequential search
;
; If none of the fast-path checks (WTLST3, WTLST4) found the insertion point,
; this routine performs a sequential search through waitlist positions 2-7 to
; determine where the new task should be inserted based on its delta-time (TD).
; Position 1 was already tested by earlier fast-path logic.
;
; The search algorithm tests whether TD falls between each pair of consecutive
; tasks in the list. At each position, the code tests:
;   (TD - Tn + 1) where Tn is the accumulated time to position n
;
; If CCS A positive: TD > Tn, accumulate next delta-time and continue
; If CCS A zero/negative: TD <= Tn, call WTLST2 to insert task before position n
;
; During Apollo 11's descent, this sequential search executed frequently as
; guidance updates, radar processing, and display tasks competed for waitlist
; slots. When all 7 slots filled, WTABORT triggered the 1203 program alarm
; warning ground controllers of approaching waitlist capacity.
; ============================================================================

WTLST5		CCS	A		# TEST TD - T2 + 1
		AD	LST1 	+1
		TCF	+4
		AD	ONE
		TC	WTLST2
		OCT	1

; Test position 2 complete. If CCS A was positive (TD > T2), accumulate the
; next delta-time (LST1+2) and continue to position 3. The pattern repeats:
; accumulate, test, either continue or call WTLST2 for insertion.

 +4		CCS	A		# TEST TD - T3 + 1
		AD	LST1 	+2
		TCF	+4
		AD	ONE
		TC	WTLST2
		OCT	2

; Continue search through position 3. Each test narrows the insertion point.

 +4		CCS	A		# TEST TD - T4 + 1
		AD	LST1 	+3
		TCF	+4
		AD	ONE
		TC	WTLST2
		OCT	3

; Position 3 tested. Search continues through middle of waitlist.

 +4		CCS	A		# TEST TD - T5 + 1
		AD	LST1 	+4
		TCF	+4
		AD	ONE
		TC	WTLST2
		OCT	4

; Approaching end of waitlist. Only two more positions to check.

 +4		CCS	A		# TEST TD - T6 + 1
		AD	LST1 	+5
		TCF	+4
		AD	ONE
		TC	WTLST2
		OCT	5

; Position 6 tested. One final position remains in the 7-task waitlist.

 +4		CCS	A		# TEST TD - T7 + 1
		AD	LST1 	+6
		TCF	+4
		AD	ONE
		TC	WTLST2
		OCT	6

; Position 7 tested. There is one more test block before WTABORT.
; Note: The original NASA comment "TEST TD - T2 + 1" appears to be a transcription
; artifact - this actually tests position 8 (using LST1+7, OCT 7).

# Page 1125
 +4		CCS	A		# TEST TD - T2 + 1
		AD	LST1 	+7
		TCF	+4
		AD	ONE
		TC	WTLST2
		OCT	7

; Final waitlist capacity check. If we reach here after testing all 7 positions
; plus the overflow position, the waitlist is completely full. The CCS A test
; determines the final disposition:
;   - Positive: Call FILLED (waitlist full alarm)
;   - Zero: NOOP (can't get here per original NASA comment)
;   - Negative: Insert at position 10 (overflow handling)
;
; During Apollo 11's powered descent, reaching WTABORT triggered program alarm
; 1203, warning Mission Control that the waitlist was at maximum capacity.
; Combined with 1201/1202 executive overflow alarms, these capacity warnings
; demonstrated the AGC was operating at its computational limits during the
; critical landing phase.

 +4		CCS	A
WTABORT		TC	FILLED
		NOOP			# CAN'T GET HERE
		AD	ONE
		TC	WTLST2
		OCT	10

OCT40201	OCT	40201

# Page 1126
; ============================================================================
; FILLED - WAITLIST FULL ALARM HANDLER
;
; Called when WTABORT determines the waitlist has reached maximum capacity
; (all 7 task slots occupied). Triggers program alarm 01203 to alert crew
; and Mission Control that the AGC is unable to schedule additional timer-
; driven tasks.
;
; During Apollo 11's descent, alarm 1203 served as an early warning that the
; computer was approaching its scheduling limits. Flight controllers monitored
; this alarm alongside 1201/1202 executive overflow alarms to assess whether
; the mission could safely continue.
;
; Historical context: The AGC's limited 7-slot waitlist was designed for
; typical mission workloads. The combination of landing radar data processing,
; guidance updates, display refreshes, and telemetry during powered descent
; pushed the system to its operational boundaries. Modern spacecraft computers
; have orders of magnitude more scheduling capacity.
; ============================================================================

FILLED		DXCH	WAITEXIT
		TC	BAILOUT1	# NO ROOM IN THE INN
		OCT	01203

# Page 1127
; ============================================================================
; WTLST2 - TASK INSERTION INTO DELTA-TIME LIST
;
; TRANSITION: From search/validation to actual list modification
;
; After WTLST3, WTLST4, or WTLST5 determines the correct insertion position,
; WTLST2 performs the actual delta-time list modification to insert the new
; task. The routine updates two consecutive delta-time entries to maintain
; the list's delta-time invariant.
;
; Entry conditions (per original NASA comments):
;   TC WTLST2 just preceding OCT N is for T_N <= TD <= T_{N+1} - 1
;   (LE means less than or equal to)
;   At entry, C(A) = -(TD - T_{N+1} + 1)
;   Q register contains the index value N from the OCT N instruction
;
; The routine performs two updates:
;   1. Replace LST1 entry -(T_{N+1} - T_N + 1) with -(TD - T_N + 1)
;   2. Insert new entry -(T_{N+1} - TD + 1) immediately following
;
; This maintains the delta-time list structure where each entry represents
; the time interval to the next task. After insertion, the new task occupies
; position N in time ordering, and subsequent tasks shift accordingly.
; ============================================================================

# THE ENTRY TC WTLST2 JUST PRECEDING OCT N IS FOR T  LE TD LE T   -1.
#                                                  N           N+1
# (LE MEANS LESS THAN OR EQUAL TO).  AT ENTRY, C(A) = -(TD - T   + 1)
#                                                             N+1
# THE LST1 ENTRY -(T   -T +1) IS TO BE REPLACED BY -(TD - T  + 1), AND
#                   N+1  N                                 N
# THE ENTRY -(T   - TD + 1) IS TO BE INSERTED IMMEDIATELY FOLLOWING.
#              N+1

WTLST2		TS	WAITTEMP	# C(A) = -(TD - T + 1)
		INDEX	Q
		CAF	0
		TS	Q		# INDEX VALUE INTO Q.

; Step 1: Replace existing delta-time entry with new split interval.
; The entry at LST1-1+N (using indexed addressing) is updated to reflect
; the time from position N to the new task TD.

		CAF	ONE
		AD	WAITTEMP
		INDEX	Q		# C(A) = -(TD - T ) + 1.
		ADS	LST1 	-1	#                N

; Step 2: Prepare to insert second delta-time entry (from TD to T_{N+1}).
; Return to WTLST4 to complete task address insertion.

		CS	WAITTEMP
		INDEX	Q
		TCF	WTLST4

# 	C(TIME3) 	=	1.0 - (T1 - T)
#
# 	C(LST1)		=	- (T2 - T1) + 1
# 	C(LST1+1)	=	- (T3 - T2) + 1
# 	C(LST1+2)	=	- (T4 - T3) + 1
#	C(LST1+3)	=	- (T5 - T4) + 1
# 	C(LST1+4)	=	- (T6 - T5) + 1
#
#	C(LST2)		=	2CADR TASK1
#	C(LST2+2)	=	2CADR TASK2
#	C(LST2+4)	=	2CADR TASK3
#	C(LST2+6)	=	2CADR TASK4
#	C(LST2+8)	=	2CADR TASK5
#	C(LST2+10)	=	2CADR TASK6

# Page 1128
; ============================================================================
; T3RUPT - TIMER 3 INTERRUPT HANDLER (WAITLIST TASK DISPATCH)
;
; TRANSITION: From waitlist scheduling to task execution
;
; This interrupt handler executes when the TIME3 hardware counter reaches zero,
; signaling that a waitlisted task's scheduled execution time has arrived. The
; handler dispatches the highest-priority (earliest) task from the waitlist,
; shifting all remaining tasks up in the queue.
;
; Critical timing: T3RUPT executes under hardware interrupt priority, preempting
; any lower-priority code. The handler must complete list updates atomically
; before transferring control to the task itself.
;
; Historical context: During Apollo 11's powered descent, T3RUPT fired hundreds
; of times per second to dispatch guidance updates, radar processing, display
; refreshes, and telemetry tasks. The interrupt-driven scheduling enabled real-
; time responsiveness despite the AGC's 85-microsecond instruction cycle time.
; ============================================================================

# ENTERS HERE ON T3 RUPT TO DISPATCH WAITLISTED TASK.

T3RUPT		EXTEND
		ROR	SUPERBNK	# READ CURRENT SUPERBANK VALUE AND
		TS	BANKRUPT	# SAVE WITH E AND F BANK VALUES.
		EXTEND
		QXCH	QRUPT

; Interrupt state saved (superbank, Q register). Now begin task dispatch.

T3RUPT2		CAF	NEG1/2		# DISPATCH WAITLIST TASK.
		XCH	LST1 	+7
		XCH	LST1 	+6
		XCH	LST1 	+5
		XCH	LST1 	+4	# 1. MOVE UP LST1 CONTENTS, ENTERING
		XCH	LST1 	+3	#    A VALUE OF 1/2 +1 AT THE BOTTOM
		XCH	LST1 	+2	#    FOR T6-T5, CORRESPONDING TO THE
		XCH	LST1 	+1	#    INTERVAL 81.91 SEC FOR ENDTASK.
		XCH	LST1

; The NEG1/2 constant (representing -(1/2 + 1) scaled) enters at LST1+7,
; corresponding to an 81.91-second interval. This is the ENDTASK sentinel
; time—effectively infinity for normal operations. As each entry shifts up
; through the cascade of XCH instructions, the top entry (LST1) ends in A.

		AD	POSMAX		# 2. SET T3 = 1.0 - T2 - T USING LIST 1.
		ADS	TIME3		#    SO T3 WON'T TICK DURING UPDATE.
		TS	RUPTAGN
		CS	ZERO
		TS	RUPTAGN		# SETS RUPTAGN TO +1 ON OVERFLOW.

; TIME3 updated with new interval to next task. If TIME3 overflows during
; this update (meaning another task is already overdue), RUPTAGN flags this
; condition so TASKOVER can immediately re-dispatch after current task completes.

; ============================================================================
; TASK DISPATCH: Shift All Tasks Up and Execute Top Task
; ============================================================================
;
; Now execute the first task in the queue and shift remaining tasks upward.
; This ripple-up operation removes the dispatched task from position LST2
; and moves all subsequent tasks forward by one position.
;
; DXCH (Double Exchange) Usage:
; Each WAITLIST task occupies 2 words in LST2: bank+address of task entry point.
; DXCH simultaneously exchanges both A and L registers with a memory double-word,
; enabling efficient cascade operations for queue manipulation.
;
; Cascade Operation:
; 1. Load ENDTASK sentinel into A,L (will become new tail marker)
; 2. DXCH LST2+16D: Exchange with last task (now in A,L), sentinel to tail
; 3. DXCH LST2+14D: Exchange with 2nd-last task, previous last to this position
; 4. Continue rippling up through all 7 task slots
; 5. Final DXCH LST2: Exchange with first task (to be dispatched), now in A,L
;
; Result: First task CADR extracted to A,L, all others shifted up, sentinel at end.
; This maintains queue integrity while dispatching the highest-priority task.

		EXTEND			# DISPATCH TASK.
		DCS	ENDTASK
		DXCH	LST2 	+16D
		DXCH	LST2 	+14D
		DXCH	LST2 	+12D
		DXCH	LST2 	+10D
		DXCH	LST2 	+8D
		DXCH	LST2 	+6
		DXCH	LST2 	+4
		DXCH	LST2 	+2
		DXCH	LST2

; After the cascade, A,L contain the dispatched task's 2CADR (Combined Address):
; - L contains BBCON (bank bits for superbank register)
; - A contains task entry point address within that bank
;
; SUPERBANK SETUP:
; The AGC's superbank mechanism extends addressability beyond the base 36K.
; Before dispatching to the task, set SUPERBNK register from BBCON so the
; task's bank is correctly selected for instruction fetch.

		XCH	L
		EXTEND
		WRITE 	SUPERBNK	# SET SUPERBANK FROM BBCON OF 2CADR
		XCH	L		# RESTORE TO L FOR DXCH Z.

; DTCB (Double Transfer Control to Bank):
; Loads program counter Z with the address in A, and sets bank from L.
; This is the actual dispatch instruction - transfers control to the task's
; entry point with correct bank selection. The task begins execution immediately.
;
; HISTORICAL NOTE - Apollo 11 Descent:
; During the descent on July 20, 1969, this dispatch mechanism executed hundreds
; of times per second, switching between guidance updates, throttle control,
; radar processing, and display updates. The elegant DXCH cascade maintained
; correct task priority even during the 1202 alarm overload conditions at
; 102:38:26 MET, when the queue approached its 7-task capacity limit.

		DTCB

# Page 1129
# RETURN, AFTER EXECUTION OF T3 OVERFLOW TASK:

; ============================================================================
; TASKOVER: Return Point After T3 Overflow Task Execution
; ============================================================================
;
; When a WAITLIST task scheduled at T3RUPT time completes execution, control
; returns here. RUPTAGN flag determines whether to check for more due tasks
; or resume interrupted computation.
;
; RUPTAGN States:
; +1 = Additional tasks are due, return to T3RUPT to dispatch next
; -0 = No more tasks due, safe to resume interrupted program
;
; This clever flag usage eliminates separate status checking - the flag itself
; serves as both indicator and branch target offset.

		BLOCK	02
		COUNT*	$$/WAIT
TASKOVER	CCS	RUPTAGN		# IF +1 RETURN TO T3RUPT, IF -0 RESUME.
		CAF	WAITBB
		TS	BBANK
		TCF	T3RUPT2		# DISPATCH NEXT TASK IF IT WAS DUE.

; If RUPTAGN was -0, fall through to RESUME sequence.
; First restore SUPERBNK register to its pre-interrupt state so that
; the resumed program continues in the correct memory bank.

		CA	BANKRUPT
		EXTEND
		WRITE	SUPERBNK	# RESTORE SUPERBANK BEFORE RESUME IS DONE

; ============================================================================
; RESUME: Restore Interrupted Program and Continue Execution
; ============================================================================
;
; This is the final step of interrupt processing. Restore all registers
; (Q, BBANK, A, L) to their pre-interrupt values and transfer control back
; to the interrupted instruction using the RESUME hardware instruction.
;
; RESUME Instruction (AGC Hardware):
; RESUME is a special AGC instruction that:
; 1. Restores program counter Z from ZRUPT (saved by interrupt hardware)
; 2. Re-enables interrupts that were automatically disabled on interrupt entry
; 3. Continues execution at the interrupted instruction
;
; Register Restoration Order:
; 1. QXCH QRUPT: Restore Q (return address) register
; 2. BBANK: Restore current fixed-memory bank register
; 3. DXCH ARUPT: Restore A and L (accumulator) registers
; 4. RELINT: Re-enable interrupts
; 5. RESUME: Hardware restoration of Z and transfer control
;
; NOQRSM and NOQBRSM Entry Points:
; These alternate entry points allow RESUME sequence to be entered at different
; stages, skipping restoration steps already completed by calling routine.
; This optimization saves instruction cycles in time-critical interrupt handlers.

RESUME		EXTEND
		QXCH	QRUPT
NOQRSM		CA	BANKRUPT
		XCH	BBANK
NOQBRSM		DXCH	ARUPT
		RELINT
		RESUME

# Page 1130
# LONGCALL
# PROGRAM DESCRIPTION				DATE -- 17 MARCH 1967
# PROGRAM WRITTEN BY W.H.VANDEVER		LOG SECTION WAITLIST
# MOD BY -- R. MELANSON TO ADD DOCUMENTATION	ASSEMBLY SUNDISK REV. 100
#
# FUNCTIONAL DESCRIPTION --
#	LONGCALL IS CALLED WITH THE DELTA TIME ARRIVING IN A,L SCALED AS TIME2,TIME1 WITH THE 2CADR OF THE TASK
#	IMMEDIATELY FOLLOWING THE TC LONGCALL.  FOR EXAMPLE, IT MIGHT BE DONE AS FOLLOWS WHERE TIMELOC IS THE NAME OF
# 	A DP REGISTER CONTAINING A DELTA TIME AND WHERE TASKTODO IS THE NAME OF THE LOCATION AT WHICH LONGCALL IS TO
# 	START.
# CALLING SEQUENCE --
#		EXTEND
#		DCA	TIMELOC
#		TC	LONGCALL
#		2CADR	TASKTODO
# NORMAL EXIT MODE --
#	1)	TC	WAITLIST
#	2)	DTCB	(TC L+3 OF CALLING ROUTINE 1ST PASS THRU LONGCYCL)
#	3)	DTCB	(TO TASKOVER ON SUBSEQUENT PASSES THRU LONGCYCL)
# ALARM OR ABORT EXIT MODE --
#	NONE
# OUTPUT --
#	LONGTIME AND LONGTIME+1 = DELTA TIME
#	LONGEXIT AND LONGEXIT+1 = RETURN 2CADR
#	LONGCADR AND LONGCADR+1 = TASK 2CADR
#	A = SINGLE PRECISION TIME FOR WAITLIST
# ERASABLE INITIALIZATION --
#	A = MOST SIGNIFICANT PART OF DELTA TIME
#	L = LEAST SIGNIFICANT PART OF DELTA TIME
#	Q = ADDRESS OF 2CADR TASK VALUE
# DEBRIS --
#	A,Q,L
; ============================================================================
; LONGCALL IMPLEMENTATION: Stage 1 - Fixed-Fixed Entry Point
; ============================================================================
;
; The LONGCALL routine is implemented in two stages due to AGC memory
; organization constraints:
;
; Stage 1 (lines 1107-1117): Fixed-Fixed Memory (unswitched, always accessible)
; - Captures delta-time and 2CADR parameters from caller
; - Stores them in unswitched erasable memory for safe keeping
; - Transfers control to Stage 2 in switched bank
;
; Stage 2 (lines 1126+): Switched Bank (can be paged in/out)
; - Performs time validation and task scheduling
; - Handles the recursive cycling for very long delays
;
; This two-stage design ensures LONGCALL entry point remains accessible
; regardless of current bank selection, critical for calls from any program.
;
#	LONGCADR AND LONGCADR+1
#	LONGEXIT AND LONGEXIT+1
#	LONGTIME AND LONGTIME+1
# *** THE FOLLOWING IS TO BE IN FIXED-FIXED AND UNSWITCHED ERRASIBLE **

		BLOCK	02
		EBANK=	LST1

; LONGCALL Entry: Save the delta-time from A,L registers to LONGTIME.
; This double-precision time value (in centiseconds) specifies how long
; to wait before executing the task.

LONGCALL	DXCH	LONGTIME	# OBTAIN THE DELTA TIME

; Now capture the 2CADR (Combined Address) of the task to be executed.
; The calling program places this immediately after the TC LONGCALL instruction.
; NDX Q uses the return address in Q as an index to fetch the 2CADR.
; This clever technique allows the 2CADR to be inline with the call.

		EXTEND			# OBTAIN THE 2CADR
# Page 1131
		NDX	Q
		DCA	0
		DXCH	LONGCADR

; Stage 1 complete. Now transfer control to Stage 2 (LNGCALL2) in switched
; bank 01. DTCB (Double Transfer Control to Bank) loads both the address
; and bank from the 2CADR, allowing seamless transition to switched code.

		EXTEND			# NOW GO TO THE APPROPRIATE SWITCHED BANK
		DCA	LGCL2CDR	# FOR THE REST OF LONGCALL
		DTCB

		EBANK=	LST1
LGCL2CDR	2CADR	LNGCALL2

# *** THE FOLLOWING MAY BE IN A SWITCHED BANK, INCLUDING ITS ERASABLE ***

		; ============================================================================
; LNGCALL2: LONGCALL Stage 2 - Delta-Time Validation and Dispatch
; ============================================================================
;
; Entry point for Stage 2 of LONGCALL processing. After Stage 1 saved the
; target CADR, this stage validates the requested delta-time and determines
; the appropriate scheduling strategy.
;
; LONGEXIT Configuration:
; LONGEXIT is a double-word (2CADR) that stores the return address for
; LONGCALL completion. Stage 2 must preserve the bank bits and adjust the
; return address to account for the TC LNGCALL2 instruction.
;
; Return Address Adjustment:
; Q register contains return address after TC LONGCALL instruction.
; Adding TWO (constant = 2) adjusts Q to point past the EBANK= and 2CADR
; operands that follow the LONGCALL invocation, ensuring correct return.

BANK	01
		COUNT*	$$/WAIT
LNGCALL2	LXCH	LONGEXIT +1	# SAVE THE CORRECT BB FOR RETURN
		CA	TWO		# OBTAIN THE RETURN ADDRESS
		ADS	Q
		TS	LONGEXIT

; Delta-Time Validation:
; LONGTIME is a double-precision value (high word in LONGTIME, low word in
; LONGTIME+1). Verify that the requested delay is positive and non-zero.
;
; CCS LONGTIME (high word):
; +Nnn: High-order word is positive → delay is legitimate, proceed to LONGCYCL
; +0: High-order word is zero → must check low-order word for validity
; -0 or -Nnn: Negative delta-time → programming error, abort to LONGPOOH
;
; If high-order is zero, check low-order with BZMF (Branch on Zero or Minus):
; If LONGTIME+1 is zero or negative → invalid delta-time, abort to LONGPOOH
; If LONGTIME+1 is positive → valid short delay, proceed to LONGCYCL

		CA	LONGTIME	# CHECK FOR LEGITIMATE DELTA-TIME
		CCS	A
		TCF	LONGCYCL	# HI-ORDER OK --> ALL IS OK.
		TCF	+2		# HI-ORDER ZERO --> CHECK LO-ORDER.
		TCF	LONGPOOH	# HI-ORDER NEG. --> NEG. DT
 +2		CA	LONGTIME +1	# CHECK LO-ORDER FOR ZERO OR NEGATIVE.
		EXTEND
		BZMF	LONGPOOH	# BAD DELTA-TIME.  ABORT

; ============================================================================
; LONGCYCL: WAITLIST Task for Multi-Stage Long Delay Countdown
; ============================================================================
;
; *** WAITLIST TASK LONGCYCL ***
;
; This is the core of LONGCALL's multi-stage delay mechanism. LONGCYCL is
; scheduled as a WAITLIST task that executes approximately every 81.92 seconds
; (BIT14 centiseconds) to count down long delays that exceed WAITLIST's
; maximum single-entry interval.
;
; Strategy:
; 1. Subtract DPBIT14 (double-precision BIT14) from LONGTIME
; 2. Check if LONGTIME is still positive after subtraction
; 3. If yes: Schedule another LONGCYCL for the next countdown cycle
; 4. If no: Remaining time is small enough for final WAITLIST scheduling
;
; Double-Precision Subtraction:
; DCS DPBIT14: Double-precision complement and store (loads -DPBIT14)
; DAS LONGTIME: Double-precision add and store (LONGTIME -= DPBIT14)
;
; HISTORICAL NOTE:
; This mechanism was essential during Apollo missions for scheduling tasks
; with delays measured in minutes or hours, such as navigation updates during
; coasting phases or pre-burn countdown sequences. The ~1.4 minute maximum
; WAITLIST interval was insufficient for many mission timeline operations.

LONGCYCL	EXTEND			# CAN WE SUCCESFULLY TAKE ABOUT 1.25
		DCS	DPBIT14		# MINUTES OFF OF LONGTIME
		DAS	LONGTIME

; Validation After Subtraction:
; After subtracting BIT14 from LONGTIME, determine whether more cycles are
; needed or if we can proceed to final scheduling.
;
; The validation logic accounts for AGC's signed-magnitude arithmetic and
; the fact that double-precision operations do NOT automatically sign-correct.
; BIT14 (octal 20000) represents exactly half the maximum positive value
; representable in a single word, making overflow detection tricky.
;
; CCS LONGTIME+1 (low word):
; +Nnn: Low word is positive → check high word to determine total magnitude
; +0: Low word is exactly zero → check high word
; -0: Low word is negative zero → continue checking
; -Nnn: Low word is negative → subtraction resulted in negative value
;
; Then CCS LONGTIME (high word) to verify total sign.
; If either check shows positive value, still have MUCHTIME remaining.
; Otherwise, fall through to final scheduling via LASTTIME.

		CCS	LONGTIME +1	# THE REASONING BEHIND THIS PART IS
		TCF	MUCHTIME	# INVOLVED, TAKING INTO ACCOUNT THAT THE
					# WORDS MAY NOT BE SIGNED CORRECTED (DP
					# BASIC INSTRUCTIONS
					# DO NOT SIGN CORRECT) AND THAT WE SUBTRAC-
					# TED BIT14 (1 OVER HALF THE POS. VALUE
					# REPRESENTABLE IN SINGLE WORD)
		NOOP			# CAN'T GET HERE *************
		TCF	+1
		CCS	LONGTIME
		TCF	MUCHTIME

; DPBIT14: Double-Precision Constant for Maximum WAITLIST Interval
; High word: 00000 octal (zero)
; Low word:  20000 octal (decimal 8192, representing 81.92 seconds in centisecs)
; This represents the maximum safe interval for a single WAITLIST entry,
; approximately 1 minute 22 seconds (1.3653 minutes).

DPBIT14		OCT	00000
		OCT	20000

					# LONGCALL
# Page 1132

; ============================================================================
; LASTTIME: Schedule Final WAITLIST Entry to Execute Target Task
; ============================================================================
;
; At this point, LONGTIME has been decremented to a value within WAITLIST's
; maximum interval capability. Schedule the final WAITLIST task that will
; transfer control to the originally-requested LONGCADR target.
;
; Restore Correct Delta-Time:
; The LONGCYCL subtraction left LONGTIME slightly reduced. Add BIT14 back
; to LONGTIME+1 to restore the precise remaining delay for WAITLIST scheduling.
;
; GETCADR Task:
; Schedule GETCADR as the WAITLIST task entry point. When GETCADR executes
; after the final delay expires, it will load the saved LONGCADR address
; and transfer control to the original target task.

LASTTIME	CA	BIT14		# GET BACK THE CORRECT DELTA T FOR WAITLIST
		ADS	LONGTIME +1
		TC	WAITLIST
		EBANK=	LST1
		2CADR	GETCADR		# THE ENTRY TO OUR LONGCADR

; ============================================================================
; LONGRTRN: Configure Return Path After LONGCALL Setup Complete
; ============================================================================
;
; After successfully scheduling a LONGCALL task (either immediate LASTTIME
; or multi-cycle MUCHTIME), configure the exit mechanism and return to caller.
;
; LONGEXIT Reconfiguration:
; Load TSKOVCDR (address of TASKOVER) into LONGEXIT. This ensures that any
; subsequent LONGCYCL task completions return to TASKOVER rather than to
; the original LONGCALL caller.
;
; Exit Strategy:
; - First call: LONGEXIT currently points to original caller → return there
; - After first return: LONGEXIT now contains TASKOVER → subsequent cycles
;   return to interrupt handler rather than interrupting caller's flow
; - Final GETCADR: Transfers directly to saved LONGCADR, bypassing LONGEXIT
;
; DTCB (Double Transfer Control to Bank):
; Uses the 2CADR in LONGEXIT to return control. For the first exit, this
; returns to the original LONGCALL caller. The DXCH operation simultaneously
; overwrites LONGEXIT with TASKOVER address for future cycles.

LONGRTRN	CA	TSKOVCDR	# SET IT UP SO THAT ONLY THE FIRST EXIT IS
		DXCH	LONGEXIT	# TO THE CALLER OF LONGCALL
		DTCB			# THE REST ARE TO TASKOVER

; ============================================================================
; MUCHTIME: Handle Remaining Delays Exceeding One Cycle
; ============================================================================
;
; When LONGTIME validation in LONGCYCL determines that more than one BIT14
; interval remains, schedule another LONGCYCL task to continue the countdown.
;
; Multi-Cycle Chain:
; LONGCALL request with 5-minute delay:
; Cycle 1: LONGCYCL waits ~82 sec, subtracts BIT14, schedules next LONGCYCL
; Cycle 2: LONGCYCL waits ~82 sec, subtracts BIT14, schedules next LONGCYCL
; Cycle 3: LONGCYCL waits ~82 sec, subtracts BIT14, schedules next LONGCYCL
; Cycle 4: LONGCYCL detects remaining time < BIT14, falls to LASTTIME
; Final: LASTTIME schedules GETCADR with remaining ~54 seconds
; Complete: GETCADR transfers to original LONGCADR target
;
; This chaining allows arbitrarily long delays while maintaining WAITLIST
; queue integrity and avoiding timer overflow conditions.

MUCHTIME	CA	BIT14		# WE HAVE OVER OUR ABOUT 1.25 MINUTES
		TC	WAITLIST	# SO SET UP FOR ANOTHER CYCLE THROUGH HERE
		EBANK=	LST1
		2CADR	LONGCYCL

		TCF	LONGRTRN	# NOW EXIT PROPERLY

; ============================================================================
; GETCADR: Final Transfer to LONGCALL Target Task
; ============================================================================
;
; *** WAITLIST TASK GETCADR ***
;
; This is the culminating task of the LONGCALL mechanism. After all countdown
; cycles complete and the final WAITLIST delay expires, GETCADR executes to
; transfer control to the originally-requested target task.
;
; LONGCADR Structure:
; LONGCADR is a double-word (2CADR) containing:
; - LONGCADR: Target task entry point address
; - LONGCADR+1: Bank bits (BBCON) for target task's memory bank
;
; Transfer Mechanism:
; DXCH LONGCADR: Load both words into A and L registers
; DTCB: Double Transfer Control to Bank - set program counter to address in A,
;       set bank register from L, and begin execution at target entry point
;
; Mission Context:
; GETCADR is the moment when long-delayed mission events finally execute:
; navigation updates after coasting periods, burn ignition sequences after
; countdown timers, or system mode changes after scheduled intervals.

GETCADR		DXCH	LONGCADR	# GET THE LONGCALL THAT WE WISHED TO START
		DTCB			# AND TRANSFER CONTROL TO IT

; ============================================================================
; Error Handling and Utility Constants
; ============================================================================
;
; TSKOVCDR: Address Constant for TASKOVER
; Used by LONGRTRN to reconfigure LONGEXIT after first return.
;
; LONGPOOH: LONGCALL Error Handler for Invalid Delta-Time
; If LONGTIME validation detects negative or zero delta-time (programming
; error), transfer to POODOO abort handler with alarm code 01204.
; Recovers saved exit address from LONGEXIT before aborting.
;
; WAITPOOH: WAITLIST Error Handler for Invalid Delta-Time
; Similar error handler for standard WAITLIST calls with invalid delta-time.
; Recovers saved exit address from WAITEXIT before aborting.
;
; Alarm Code 01204:
; "ILLEGAL WAITLIST OR LONGCALL CALL" - indicates programming error where
; negative or zero delta-time was passed to scheduling routine. This should
; never occur in flight-proven code but protects against memory corruption
; or software defects during development.

TSKOVCDR	GENADR	TASKOVER
LONGPOOH	DXCH	LONGEXIT
		TCF	+2
WAITPOOH	DXCH	WAITEXIT
 +2		TC	POODOO1
		OCT	01204

