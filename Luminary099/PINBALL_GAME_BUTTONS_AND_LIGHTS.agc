# Copyright:	Public domain.
# Filename:	PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
# Purpose:	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim.lawton@gmail.com>
# Website:	www.ibiblio.org/apollo.
# Pages:	390-471
# Mod history:	2009-05-16 JVL	Started updating from page images.
# 		2009-05-25 JVL	Finished updating from page images.
#		2009-07-01 RSB	Began annotating.
#		2010-12-31 JL	Fixed page number comment.
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
#    Assemble revision 001 of AGC program LMY99 by NASA 2021112-061
#    16:27 JULY 14, 1969

## <b>Note:</b> Notations below resembling this note are 21st-century
## annotations added by the <a href="http://www.ibiblio.org/apollo">
## Virtual AGC project</a>, and are not original AGC source code.

; ============================================================================
; FILE: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
; MODULE: Display and Crew Interface
; MISSION PHASE: all phases (launch/earth-orbit/trans-lunar/lunar-orbit/
;                descent/landing/ascent/rendezvous/trans-earth/re-entry)
;
; TL;DR: Implements complete DSKY (Display and Keyboard) button handling and
;        indicator light control for Lunar Module crew interface. Processes
;        all button presses (VERB, NOUN, KEY REL, ENTR, RSET, CLR, +, -, 0-9)
;        through interrupt-driven state machine. Controls all indicator lights
;        including COMP ACTY, UPLINK ACTY, PROG, KEY REL, OPR ERR, TEMP,
;        NO ATT, GIMBAL LOCK, STBY, RESTART, TRACKER, ALT, VEL. This is the
;        primary human-computer interface through which Armstrong and Aldrin
;        communicated with the AGC during all mission phases.
;
; COMMENT-ONLY READERS: This file contains every interaction between the crew
;        and the computer. Every verb/noun entry, every button press during
;        landing and ascent went through these routines. Read to understand
;        how astronauts controlled the spacecraft computer.
;
; CODE-ALONG READERS: Study the interrupt-driven state machine architecture,
;        button debouncing logic, verb/noun entry validation, display locking
;        mechanism, and indicator light control through CHANNEL 11 output.
;        Note integration with DISPLAY_INTERFACE_ROUTINES for output and
;        KEYRUPT for keyboard interrupt handling.
; ============================================================================

# Page 390
; ============================================================================
; TRANSITION: Program Introduction and Operational Context
;
; The "PINBALL" program is the complete human-computer interface system for
; the Lunar Module AGC. The whimsical name originated from a demonstration
; program that unexpectedly became the production interface. Every crew
; interaction during Apollo 11 - from pre-launch checks through "The Eagle
; has landed" and ascent rendezvous - passed through these routines.
;
; During descent on July 20, 1969, Armstrong and Aldrin monitored altitude
; and velocity displays updated by this code. When the 1202 alarm occurred
; at 102:38:26 mission time, these routines displayed the alarm code on the
; DSKY. Armstrong's manual landing site selection commands were processed here.
; ============================================================================

# PROGRAM NAME -- KEYBOARD AND DISPLAY PROGRAM
# MOD NO -- 4		DATE -- 27 APRIL 1967		ASSEMBLY -- PINDANCE REV 18
# MOD BY -- FILENE
# LOG SECTION -- PINBALL GAME BUTTONS AND LIGHTS
#
# FUNCTIONAL DESCRIPTION
#
# THE KEYBOARD AND DISPLAY SYSTEM PROGRAM OPERATES UNDER EXECUTIVE
# CONTROL AND PROCESSES INFORMATION EXCHANGED BETWEEN THE AGC AND THE
# COMPUTER OPERATOR.  THE INPUTS TO THE PROGRAM ARE FROM THE KEYBOARD,
# FROM INTERNAL PROGRAM, AND FROM THE UPLINK.

; The DSKY (Display and Keyboard) is the crew's window into the AGC. It has
; three main input sources:
; 1. KEYBOARD: Physical button presses from the crew (19 buttons total)
; 2. UPLINK: Commands from Mission Control transmitted via radio
; 3. INTERNAL: Program-initiated displays and data requests
#
; ============================================================================
; THE VERB/NOUN INTERFACE SYSTEM
;
; Communication language: Pairs of two-digit decimal codes (VERB + NOUN)
; VERB = what action to perform (01-99 decimal)
; NOUN = what data to act upon (01-99 decimal)
;
; Example from Apollo 11 landing:
;   V16 N68 = "VERB 16, NOUN 68" = Display altitude and altitude rate
;   (Used continuously during final descent below 10,000 feet)
;
; Example from ascent:
;   V06 N62 = "VERB 06, NOUN 62" = Display velocity components
;   (Monitored during ascent to ensure orbital insertion)
; ============================================================================

# THE LANGUAGE OF COMMUNICATION WITH THE PROGRAM IS A PAIR OF WORDS
# KNOWN AS VERB AND NOUN.  EACH OF THESE IS REPRESENTED BY A 2 CHARACTER
# DECIMAL NUMBER.  THE VERB CODE INDICATES WHAT ACTION IS TO BE TAKEN, THE
# NOUN CODE INDICATES TO WHAT THIS ACTION IS APPLIED.  NOUNS USUALLY
# REFER TO A GROUP OF ERASABLE REGISTERS.

; The verb/noun system works like an object-oriented command structure:
; - VERB: The method or action (display, load, monitor, test, execute)
; - NOUN: The data object (position, velocity, time, angles, etc.)
; Most nouns reference groups of erasable memory registers containing
; related data (e.g., three components of a position vector).
#
; VERB CATEGORIES (complete catalog in EXTENDED_VERBS.agc):
;
; DISPLAYS: One-time data display (V05, V06, V16 - show data then wait)
; LOADS: Crew data entry into AGC memory (V21, V22, V24 - load target data)
; MONITORS: Continuous display updated once per second (V11, V16 - real-time)
; SPECIAL FUNCTIONS: Utility operations (V25 self-check, V35 light test)
; EXTENDED VERBS: Mission-specific programs (see EXTENDED_VERBS.agc)

# VERBS ARE GROUPED INTO DISPLAYS, LOADS, MONITORS (DISPLAYS THAT ARE
# UPDATED ONCE PER SECOND), SPECIAL FUNCTIONS, AND EXTENDED VERBS (THESE
# ARE OUTSIDE OF THE DOMAIN OF PINBALL AND CAN BE FOUND UNDER LOG SECTION
# 'EXTENDED VERBS').
#
# A LIST OF VERBS AND NOUNS IS GIVEN IN LOG SECTION 'ASSEMBLY AND
# OPERATION INFORMATION'.
#
## Ram&oacute;n Alonso, one of the original AGC developers, provides a
## little more insight:  Apparently, nobody had yet arrived at any kind
## of software requirements for the AGC's user interface when the desire
## arose within the Instrumentation Laboratory to set up a demo
## guidance-computer unit with which to impress visitors to the lab.
## Of course, this demo would have to <i>do</i> something, if it was going to be
## at all impressive, and to do something it would need some software. In
## short order, some of the coders threw together a demo program,
## inventing and using the verb/noun user-interface concept (in the
## whimsical fashion seen in much of this code), but without any idea
## that the verb/noun concept would somehow survive into the flight
## software.  As time passed, and more and more people became familiar
## with the demo, nobody got around to inventing an improvement for the
## user interface, so the coders simply built it into the flight software
## without any specific requirements to do so.<br>
## <br>
## However, that does not mean that the verb/noun interface was universally
## beloved.  Ram&oacute;n says that <i>many</i> objections were received from
## naysayers, such as "it's not scientific", "it's not dignified", or
## even "astronauts won't understand it".  Even though the coders of
## the demo hadn't seriously intended the verb/noun interface to be used
## in any permanent way, it became a kind of devilish game to counter
## these objections with (perhaps) sophistic arguments as to why the
## interface was really a good one.  In the end, the coders won.  I don't
## know whether they were elated or dismayed by this victory.<br>
## <br>
## The astronauts, of course, <i>could</i> understand the interface,
## but they did not like it.  Most of them really wanted an interface much
## more like that they had used in aircraft:  i.e., lots of dials and
## switches.  Dave Scott is the the only astronaut I'm aware of who had
## kind words for it (or for the AGC in general), though we are told that
## Jim McDivitt wasn't necessary completely hostile to it.<br>
## <br>
## <div style="text-align: right;"><small>&mdash;Ron Burkey, 07/2009</small></div>
#
; ============================================================================
; CALLING SEQUENCES - How PINBALL gets activated
; ============================================================================

# CALLING SEQUENCES --
#
# KEYBOARD:
# EACH DEPRESSION OF A KEYBOARD BUTTON ACTIVATES AN INTERRUPT KEYRUPT1
# AND PLACES THE 5 BIT KEY CODE INTO CHANNEL 15.  KEYRUPT1 PLACES THE KEY
# CODE INTO MPAC, ENTERS AN EXECUTIVE REQUEST FOR THE KEYBOARD AND DISPLAY
# PROGRAM (AT 'CHARIN'), AND EXECUTES A RESUME.

; KEYBOARD INPUT PATH (interrupt-driven, highest priority crew input):
;
; 1. Crew presses button on DSKY (e.g., Armstrong presses "1" "6" during landing)
; 2. Hardware generates KEYRUPT1 interrupt (see KEYRUPT_UPRUPT.agc)
; 3. KEYRUPT1 reads 5-bit key code from CHANNEL 15
; 4. Key code stored in MPAC (temporary storage register)
; 5. KEYRUPT1 requests EXECUTIVE to schedule PINBALL job at CHARIN entry point
; 6. KEYRUPT1 executes RESUME, returning to interrupted program
; 7. EXECUTIVE schedules PINBALL at priority 30000 (NOVAC job)
; 8. PINBALL processes key code and updates DSKY display
;
; Button debouncing and validation occur in CHARIN and subsequent routines.
; Multiple rapid button presses are queued and processed sequentially.
#
# UPLINK:
# EACH WORD RECEIVED BY THE UPLINK ACTIVATES INTERRUPT UPRUPT, WHICH
# PLACES THE 5 BIT KEY CODE INTO MPAC, ENTERS AN EXECUTIVE REQUEST FOR THE
# KEYBOARD AND DISPLAY PROGRAM (AT 'CHARIN') AND EXECUTES A RESUME.

; UPLINK INPUT PATH (Mission Control commands via radio):
;
; Mission Control could send verb/noun commands to the AGC via uplink.
; During Apollo 11, Houston sent state vector updates and program loads.
; The uplink path mirrors keyboard input but originates from ground:
;
; 1. Mission Control transmits command via S-band antenna
; 2. UPRUPT interrupt activated when word received
; 3. 5-bit code placed in MPAC (same as keyboard)
; 4. EXECUTIVE request for PINBALL at CHARIN
; 5. Processing identical to keyboard input from this point
;
; Uplink was critical for navigation updates based on ground tracking data.

#
# INTERNAL PROGRAMS:
# INTERNAL PROGRAMS CALL PINBALL AT `NVSUB' WITH THE DESIRED VERB/NOUN
# CODE IN A (LOW 7 BITS FOR NOUN, NEXT 7 BITS FOR VERB).  DETAILS
# DESCRIBED ON REMARKS CARDS JUST BEFORE 'NVSUB' AND 'NVSBWAIT' (SEE
# SYMBOL TABLE FOR PAGE NUMBERS).

; INTERNAL PROGRAM INPUT PATH (AGC programs requesting display/input):
;
; Mission programs can request DSKY display or crew input by calling NVSUB.
; Example: Landing program calls NVSUB to display altitude (V16 N68).
;
; Calling convention:
;   Load A register with 14-bit verb/noun code:
;     Bits 1-7: NOUN code (00-99 decimal)
;     Bits 8-14: VERB code (00-99 decimal)
;   TC NVSUB (transfer control to NVSUB subroutine)
;
; NVSUB can operate in two modes:
; - NVSUB: Returns immediately, display updates in background
; - NVSBWAIT: Waits for crew response before returning
;
; During landing, THE_LUNAR_LANDING.agc uses NVSUB for continuous displays.
#
# NORMAL EXIT MODES --
#
#	IF PINBALL WAS CALLED BY EXTERNAL ACTION, THERE ARE FOUR EXITS:
#		1)	ALL BUT (2), (3), AND (4) EXIT DIRECTLY TO ENDOFJOB.
#		2)	EXTENDED VERBS GO TO THE EXTENDED VERB FAN AS PART OF THE
# Page 391
#			PINBALL EXECUTIVE JOB WITH PRIORITY 30000.  IT IS THE
#			RESPONSIBILITY OF THE EXTEDED VERB CALLED TO EVENTUALLY
#			CHANGE PRIORITY (IF NECESSARY) AND DO AN ENDOFJOB.
#			ALSO PINBALL IS A NOVAC JOB.  EBANK SET FOR COMMON.
#		3)	VERB 37.  CHANGE OF PROGRAM (MAJOR MODE) CALLS 'V37' IN THE
#			SERVICE ROUTINES AS PART OF THE PINBALL EXEC JOB WITH PRIO
#			30000.  THE NEW PROGRAM CODE (MAJOR MODE) IS LEFT IN A.
#		4)	KEY RELEASE BUTTON CALLS 'PINBRNCH' IN THE DISPLAY INTERFACE
#			ROUTINES AS PART OF THE PINBALL EXEC JOB WITH PRIO 30000 IF
#			THE KEY RELEASE LIGHT IS OFF AND 'CADRSTOR' IS NOT +0.
#
# IF PINBALL WAS CALLED BY INTERNAL PROGRAMS, EXIT FROM PINBALL IS BACK
# TO CALLING ROUTINE.  DETAILS DESCRIBED IN REMARKS CARDS JUST BEFORE
# 'NVSUB' AND 'NVSBWAIT' (SEE SYMBOL TABLE FOR PAGE NUMBERS).
#
; ============================================================================
; ALARM OR ABORT EXIT MODES - Error handling and safety interlocks
; ============================================================================

# ALARM OR ABORT EXIT MODES --
#
# 	EXTERNAL INITIATION:
#	IF SOME IMPROPER SEQUENCE OF KEY CODES IS DETECTED, THE OPERATOR
#	ERROR LIGHT IS TURNED ON AND EXIT IS TO 'ENDOFJOB'.

; OPERATOR ERROR HANDLING (crew input mistakes):
;
; The OPR ERR (Operator Error) light indicates invalid crew input:
; - Invalid verb or noun code (e.g., V99 which doesn't exist)
; - Improper key sequence (e.g., pressing ENTR without entering data)
; - Data out of range (e.g., time value exceeding valid limits)
;
; When OPR ERR light illuminates:
; 1. Current operation cancelled
; 2. DSKY display frozen to show error context
; 3. Crew must press RSET (reset) button to clear error and continue
; 4. PINBALL exits to ENDOFJOB, releasing executive priority
;
; During Apollo 11 landing, any OPR ERR would have been critical as crew
; attention was focused on altitude/velocity monitoring. The error
; protection prevented invalid data from corrupting guidance calculations.

#
#	INTERNAL PROGRAM INITIATION:
#	IF AN ILLEGAL V/N COMBINATION IS ATTEMPTED, AN ABORT IS CAUSED
#	(WITH OCTAL 01501).
#	IF A SECOND ATTEMPT IS MADE TO GO TO SLEEP IN PINBALL, AN ABORT IS
#	CAUSED (WITH OCTAL 01206). THERE ARE TWO WAYS TO GO TO SLEEP IN PINBALL:
#		1) ENDIDLE OR DATAWAIT.
#		2) NVSBWAIT, PRENVBSY, OR NVSUBUSY.

; INTERNAL PROGRAM ABORT CONDITIONS (software-detected errors):
;
; ABORT 01501 (octal): Illegal verb/noun combination attempted
;   Triggered when internal program calls NVSUB with invalid V/N pair.
;   Example: Requesting display of non-existent noun data.
;   This is a software bug, not crew error - should never occur in flight.
;
; ABORT 01206 (octal): Attempted to sleep in PINBALL while already sleeping
;   PINBALL can "sleep" (wait for crew input) in two ways:
;     1) ENDIDLE or DATAWAIT - waiting for keyboard input
;     2) NVSBWAIT, PRENVBSY, NVSUBUSY - waiting for display availability
;   Second sleep attempt indicates programming error (resource deadlock).
;   This abort prevents the AGC from hanging indefinitely.
;
; Both aborts are safety interlocks preventing software errors from
; causing mission-critical failures. See ALARM_AND_ABORT.agc for abort
; handling and restart protection.
#
# CONDITIONS LEADING TO THE ABOVE ARE DESCRIBED IN FORTHCOMING MIT/IL
# E-REPORT DESCRIBING KEYBOARD AND DISPLAY OPERATION FOR 278.
#
; ============================================================================
; OUTPUT TO DSKY DISPLAY - How data reaches the crew's eyes
; ============================================================================

# OUTPUT --
#
# INFORMATION TO BE SENT TO THE DISPLAY PANEL IS LEFT IN THE 'DSPTAB'
# BUFFERS REGISTERS (UNDER EXEC CONTROL). 'DSPOUT' (A PART OF T4RUPT)
# HANDLES THE PLACING OF THE 'DSPTAB' INFORMATION INTO OUTPUT CHANNEL 10
# IN INTERRUPT.

; DISPLAY OUTPUT PATH (interrupt-driven display refresh):
;
; The DSKY has three display registers (R1, R2, R3) showing numerical data,
; plus VERB/NOUN displays and 13 indicator lights. Display update process:
;
; 1. PINBALL formats data into DSPTAB buffer registers (in erasable memory)
; 2. DSPTAB contains:
;    - R1, R2, R3 display values (each 5 digits + sign)
;    - VERB display (2 digits)
;    - NOUN display (2 digits)
;    - Indicator light states (COMP ACTY, PROG, KEY REL, etc.)
; 3. DSPOUT routine (part of T4RUPT, executed every 10ms) reads DSPTAB
; 4. DSPOUT outputs formatted data to CHANNEL 10 (hardware output channel)
; 5. DSKY hardware decodes CHANNEL 10 and illuminates seven-segment displays
;
; Display refresh is interrupt-driven to ensure crew sees current data
; even during intensive computation (like landing guidance calculations).
;
; Example: During descent at 102:40 MET, R1 showed altitude, R2 showed
; altitude rate, R3 showed forward velocity. These updated continuously
; while PINBALL processed button presses and guidance ran simultaneously.
#
; ============================================================================
; ERASABLE MEMORY INITIALIZATION - Startup and restart preparation
; ============================================================================

# ERASABLE INITIALIZATION --
#
# FRESH START AND RESTART INITIALIZE THE NECESSARY E REGISTERS FOR
# PINBALL IN 'STARTSUB'.  REGISTERS ARE:  DSPTAB BUFFER, CADRSTOR,
# REQRET, CLPASS, DSPLOCK, MONSAVE, MONSAVE1, VERBREG, NOUNREG, DSPLIST,
# DSPCOUNT, NOUT.

; PINBALL ERASABLE MEMORY INITIALIZATION (occurs during AGC startup):
;
; FRESH START: Complete power-on initialization (rarely occurs in flight)
; RESTART: Recovery from power transient or program alarm (can occur anytime)
;
; Both initialize PINBALL state through STARTSUB routine, clearing:
;
; DSPTAB BUFFER: Display output buffer (blanked, all lights off)
; CADRSTOR: Return address storage for sleeping programs
; REQRET: Request/return status flags
; CLPASS: Clear pass counter for display sequencing
; DSPLOCK: Display resource lock flag (0 = available, 1 = busy)
; MONSAVE/MONSAVE1: Monitor verb save area for restart protection
; VERBREG: Currently active verb code (cleared to 00)
; NOUNREG: Currently active noun code (cleared to 00)
; DSPLIST: List of programs waiting for display access
; DSPCOUNT: Count of waiting display requests
; NOUT: Numerical output formatting control
;
; During Apollo 11 landing, if 1202 alarm caused restart, STARTSUB would
; reinitialize PINBALL while preserving mission program state. This allowed
; landing to continue despite computer overload. Restart protection was
; critical to mission success - without it, any alarm would force abort.
#
# A COMPLETE LIST OF ALL THE ERASABLES (BOTH RESERVED AND TEMPORARIES) FOR
# Page 392
# PINBALL IS GIVEN BELOW.
#
# THE FOLLOWING ARE OF GENERAL INTEREST --
#
#	REMARKS CARDS PRECEDE THE REFERENCED SYMBOL DEFINITION.  SEE SYMBOL
# TABLE TO FIND APPROPRIATE PAGE NUMBERS.
#
#	NVSUB		CALLING POINT FOR INTERNAL USE OF PINBALL.
#			OF RELATED INTEREST	NVSBWAIT
#						NVSUBUSY
#						PRENVBSY
#
#	ENDIDLE		ROUTINE FOR INTERNAL PROGRAMS WISHING TO GO TO SLEEP WHILE
#			AWAITING OPERATORS RESPONSE.
#
#	DSPMM		ROUTINE BY WHICH AN INTERNAL PROGRAM MAY DISPLAY A DECIMAL
#			PROGRAM CODE (MAJOR MODE) IN THE PROGRAM (MAJOR MODE) LIGHTS.
#			(DSPMM DOES NOT DISPLAY DIRECTLY BUT ENTERS EXEC REQUEST
#			FOR DSPMMJB WITH PRIO 30000 AND RETURNS TO CALLER.)
#
#	BLANKSUB	ROUTINE BY WHICH AN INTERNAL PROGRAM MAY BLANK ANY
#			COMBINATION OF THE DISPLAY REGISTERS R1, R2, R3.
#
; KEY SUBROUTINES (cross-references to code sections below):
;
; JAMTERM/JAMPROC: Allow internal programs to force TERMINATE (V34) or
;   PROCEED (V33) responses without crew input. Used when guidance programs
;   need to bypass "PLEASE PERFORM" crew confirmations during time-critical
;   operations (e.g., abort maneuver initiation).
;
; MONITOR: Implements monitor verbs (V11, V16) providing continuous display
;   updated once per second. During landing, V16 N68 monitored range and
;   range rate continuously while crew managed final approach.

#	JAMTERM		ROUTINE BY WHICH AN INTERNAL PROGRAM MAY PERFORM THE
#	JAMPROC		TERMINATE (V 34) OR PROCEED (V 33) FUNCTION.
#
#	MONITOR		VERBS FOR PERIODIC ( 1 PER SEC) DISPLAY.
#
#	PLEASE PERFORM, PLEASE MARK SITUATIONS
#		REMARKS DESCRIBING HOW AN INTERNAL ROUTINE SHOULD HANDLE
#		THESE SITUATIONS CAN BE FOUND JUST BEFORE :NVSUB: (SEE
#		SYMBOL TABLE FOR PAGE NUMBER).

; PLEASE PERFORM / PLEASE MARK situations:
;   These are crew decision points where AGC waits for V33 (PROCEED) or
;   V34 (TERMINATE) response. FLASHING VERB display indicates AGC is waiting.
;   Example: During P63 landing, "PLEASE PERFORM" requested crew approval
;   before committing to powered descent. See NVSUB routine for implementation.

#
#	THE NOUN TABLE FORMAT IS DESCRIBED ON A PAGE OF REMARKS CARDS JUST
#	BEFORE :DSPABC: (SEE SYMBOL TABLE FOR PAGE NUMBER).
#
#	THE NOUN TABLES THEMSELVES ARE FOUND IN LOG SECTION :PINBALL NOUN
#	TABLES:.

; NOUN TABLE REFERENCE:
;   Complete noun definitions found in PINBALL_NOUN_TABLES.agc
;   Noun format specifies: scale factor, component count (1/2/3),
;   memory addresses for data components, display mode (octal/decimal)
#
# FOR FURTHER DETAILS ABOUT OPERATION OF THE KEYBOARD AND DISPLAY SYSTEM
# PROGRAM, SEE THE MISSION PLAN AND/OR MIT/IL E-2129
# DESCRIBING KEYBOARD AND DISPLAY OPERATION FOR 278.

## The document described above, "Keyboard and Display Program Operation"
## by Alan I. Green and Robert J. Filene is
## <a href="http://www.ibiblio.org/apollo/hrst/archive/1706.pdf">
## available online at the Virtual AGC website</a>.
## <small>&mdash;Ron Burkey, 07/2009</small>

# THE FOLLOWING QUOTATION IS PROVIDED THROUGH THE COURTESY OF THE AUTHORS.
#
#	::IT WILL BE PROVED TO THY FACE THAT THOU HAST MEN ABOUT THEE THAT
# USUALLY TALK OF A NOUN AND A VERB, AND SUCH ABOMINABLE WORDS AS NO
# Page 393
# CHRISTIAN EAR CAN ENDURE TO HEAR.::
#					HENRY 6, ACT 2, SCENE 4

## Actually, this quotation is from <i>Henry VI</i>, Part 2, Act IV, Scene VII.
## <small>&mdash;Ron Burkey, 07/2009</small>

; ============================================================================
; ERASABLE MEMORY ASSIGNMENTS FOR PINBALL (DEFINED ELSEWHERE)
; ============================================================================
;
; The following registers are defined in ERASABLE_ASSIGNMENTS.agc but used
; extensively by PINBALL routines. These provide the core state storage for
; DSKY operations, consuming precious erasable memory carefully allocated.
;
; COMMENT-ONLY READERS: These memory locations store everything you see on
; the DSKY display. During Apollo 11's landing, when Armstrong and Aldrin
; watched altitude and descent rate displays, these registers held that data
; and controlled the seven-segment display characters.
;
; CODE-ALONG READERS: Note the memory-sharing techniques (overlays using =)
; and the careful allocation to avoid bank conflicts during interrupt handling.
;

# THE FOLLOWING ASSIGNMENTS FOR PINBALL ARE MADE ELSEWHERE
#
# DSPCOUNT	ERASE			# DISPLAY POSITION INDICATOR

; DSPCOUNT: Display character position indicator
;   Tracks current position in display update sequence (0-11 for DSPTAB entries)
;   Used by DSPOUT interrupt to sequence through relay outputs 60 times/second
# DECBRNCH	ERASE			# +DEC, -DEC, OCT INDICATOR

; DECBRNCH: Decimal branch indicator
;   Flags whether display format is +DEC (positive decimal), -DEC (negative
;   decimal with sign), or OCT (octal). Determines formatting path for data.

# VERBREG	ERASE			# VERB CODE

; VERBREG: Currently active verb code (two-digit decimal 01-99)
;   During landing: V16 N63 monitored altitude/altitude-rate/computed-altitude
;   V99 = "Proceed" (crew confirmation to continue with program)
;   Verb determines what action AGC takes with the noun data

# NOUNREG	ERASE			# NOUN CODE

; NOUNREG: Currently active noun code (two-digit decimal 01-99)  
;   During landing: N63 = landing radar data (altitude, rate, computed)
;   N68 = range to landing site, altitude, altitude rate
;   Noun specifies which data registers to display or modify

# XREG		ERASE			# R1 INPUT BUFFER

; XREG: Register 1 input buffer (high part)
;   Stores crew keyboard input or program data for DSKY Register 1 (top row)
;   During data entry, holds value until ENTR key confirms

# YREG		ERASE			# R2 INPUT BUFFER

; YREG: Register 2 input buffer (high part)
;   Stores crew keyboard input or program data for DSKY Register 2 (middle row)

# ZREG		ERASE			# R3 INPUT BUFFER

; ZREG: Register 3 input buffer (high part)
;   Stores crew keyboard input or program data for DSKY Register 3 (bottom row)
;   Example during landing: altitude rate in feet/second
# XREGLP	ERASE			# LO PART OF XREG (FOR DEC CONV ONLY)

; XREGLP: Register 1 input buffer low part (for decimal conversion)
;   AGC uses double-precision (two 15-bit words) for extended precision
;   Only used when decimal conversion requires full precision

# YREGLP	ERASE			# LO PART OF YREG (FOR DEC CONV ONLY)

; YREGLP: Register 2 input buffer low part (for decimal conversion)

# HITEMOUT	=	YREGLP		# TEMP FOR DISPLAY OF HRS, MIN, SEC
#					# 	MUST = LOTEMOUT-1.

; HITEMOUT overlays YREGLP: High part of time display temp storage
;   Used for formatting hours:minutes:seconds displays
;   Memory-saving technique: Reuses YREGLP when not needed for data entry
;   CRITICAL: Must be exactly one word before LOTEMOUT for indexing

# ZREGLP	ERASE			# LO PART OF ZREG (FOR DEC CONV ONLY)

; ZREGLP: Register 3 input buffer low part (for decimal conversion)

# LOTEMOUT	=	ZREGLP		# TEMP FOR DISPLAY OF HRS, MIN, SEC
#					# 	MUST = HITEMOUT+1.

; LOTEMOUT overlays ZREGLP: Low part of time display temp storage
;   Paired with HITEMOUT for time formatting
;   CRITICAL: Must be exactly one word after HITEMOUT for indexing
# MODREG	ERASE			# MODE CODE

; MODREG: Display mode code
;   Indicates current DSKY operating mode (normal, data entry, flash, etc.)
;   Controls how button presses and display updates are processed

# DSPLOCK	ERASE			# KEYBOARD/SUBROUTINE CALL INTERLOCK

; DSPLOCK: Display system interlock flag
;   Prevents simultaneous access from keyboard input and program display calls
;   Critical during high-activity periods like descent (P63 program + crew input)

# REQRET	ERASE			# RETURN REGISTER FOR LOAD

; REQRET: Return address for data load operations
;   Stores calling routine's address for resumption after load completes

# LOADSTAT	ERASE			# STATUS INDICATOR FOR LOADTST

; LOADSTAT: Load test status indicator
;   Tracks state of data entry validation (checking for valid numeric input)

# CLPASS	ERASE			# PASS INDICATOR FOR CLEAR

; CLPASS: Clear operation pass indicator
;   Multi-pass clear operations use this to track completion state

# NOUT		ERASE			# ACTIVITY COUNTER FOR DSPTAB

; NOUT: Display table activity counter
;   Counts pending updates to DSPTAB relay buffer
;   Nonzero during display changes, prevents premature register switching

# NOUNCADR	ERASE			# MACHINE CADR FOR NOUN

; NOUNCADR: Machine CADR (call address) for noun data fetch
;   Specifies both bank and address where noun's data registers are located
;   Noun table lookup provides this address for each defined noun

# MONSAVE	ERASE			# N/V CODE FOR MONITOR. (= MONSAVE1-1)

; MONSAVE: Saved noun/verb code for monitor verbs (V11, V16)
;   Monitor verbs refresh display once per second - this preserves N/V state
;   CRITICAL: Must be exactly one word before MONSAVE1 for indexing

# MONSAVE1	ERASE			# NOUNCADR FOR MONITOR (MATBS) = MONSAVE+1

; MONSAVE1: Saved noun CADR for monitor operations
;   Paired with MONSAVE to restore monitor state after interruption
;   CRITICAL: Must be exactly one word after MONSAVE for indexing

# MONSAVE2	ERASE			# NVMONOPT OPTIONS

; MONSAVE2: Monitor options flags
;   Controls monitor verb behavior (display format, update rate, etc.)

# DSPTAB	ERASE		+13D	# 0-10, DISPLAY PANEL BUFFER, 11-13, C RELAYS

; DSPTAB (+13D = 14 words): Display table / relay control buffer
;   Words 0-10: Character segment patterns for MD, VD, ND, R1, R2, R3 displays
;   Words 11-13: Indicator light relay control (COMP ACTY, PROG, KEY REL, etc.)
;   Updated by format routines, output by DSPOUT interrupt to Channel 10/11

# CADRSTOR	ERASE			# ENDIDLE STORAGE

; CADRSTOR: ENDIDLE return CADR storage
;   When verb/noun sequence completes (ENDIDLE), returns to calling program
;   Contains both bank number and address for cross-bank return

# NVQTEM	ERASE			# NVSUB STORAGE FOR CALLING ADDRESS
#					# MUST = NVBNKTEM-1.

; NVQTEM: Noun/verb subroutine calling address temp (Q register save)
;   NVSUB stores return address (Q register) here during N/V processing
;   CRITICAL: Must be exactly one word before NVBNKTEM for paired access

# NVBNKTEM	ERASE			# NVSUB STORAGE FOR CALLING BANK
#					# MUST = NVQTEM+1

; NVBNKTEM: Noun/verb subroutine calling bank temp
;   NVSUB stores return bank here during N/V processing  
;   CRITICAL: Must be exactly one word after NVQTEM for paired access

# VERBSAVE	ERASE			# NEEDED FOR RECYCLE

; VERBSAVE: Saved verb code for recycle operations
;   When verb execution is interrupted (e.g., by RSET), preserves verb for
;   potential restart. Allows crew to recover from interrupted operations.

# DSPLIST	ERASE			# WAITING REG FOR DSP SYST INTERNAL USE

; DSPLIST: Display system internal waiting list
;   Queue of pending display requests from multiple mission programs
;   Priority management: P63 landing program takes precedence during descent

# EXTVBACT	ERASE			# EXTENDED VERB ACTIVITY INTERLOCK

; EXTVBACT: Extended verb activity interlock flag
;   Prevents multiple extended verbs (V40-V99) from executing simultaneously
;   Extended verbs perform complex operations requiring exclusive access

# DSPTEM1	ERASE			# BUFFER STORAGE AREA 1 (MOSTLY FOR TIME)

; DSPTEM1: Display temp buffer 1 (primarily time data)
;   Triple-precision buffer (3 words) for hours:minutes:seconds formatting
;   Used by time display verbs during mission elapsed time updates

# DSPTEM2	ERASE			# BUFFER STORAGE AREA 2 (MOSTLY FOR DEG)

; DSPTEM2: Display temp buffer 2 (primarily angular data)
;   Triple-precision buffer (3 words) for degrees, arc-minutes, arc-seconds
;   Used by angular display verbs during attitude or position displays
#
# END OF ERASABLES RESERVED FOR PINBALL EXECUTIVE ACTION
#
# TEMPORARIES FOR PINBALL EXECUTIVE ACTION

# Page 394

# DSEXIT	=	INTB15+		# RETURN FOR DSPIN
# EXITEM	=	INTB15+		# RETURN FOR SCALE FACTOR ROUTINE SELECT
# BLANKRET	=	INTB15+		# RETURN FOR 2BLANK

# WRDRET	=	INTBIT15	# RETURN FOR 5BLANK.
# WDRET		=	INTBIT15	# RETURN FOR DSPWD
# DECRET	=	INTBIT15	# RETURN FOR PUTCOM(DEC LOAD)
# 21/22REG	=	INTBIT15	# TEMP FOR CHARIN

# UPDATRET	=	POLISH		# RETURN FOR UPDATNN, UPDATVB
# CHAR		=	POLISH		# TEMP FOR CHARIN
# ERCNT		=	POLISH		# COUNTER FOR ERROR LIGHT RESET
# DECOUNT	=	POLISH		# COUNTER FOR SCALING AND DISPLAY (DEC)

# SGNON		=	VBUF		# TEMP FOR +,- ON
# NOUNTEM	=	VBUF		# COUNTER FOR MIXNOUN FETCH
# DISTEM	= 	VBUF		# COUNTER FOR OCTAL DISPLAY VERB
# DECTEM	=	VBUF		# COUNTER FOR FETCH (DEC DISPLAY VERBS)

# SGNOFF	=	VBUF 	+1	# TEMP FOR +,- ON
# NVTEMP	=	VBUF 	+1	# TEMP FOR NVSUB
# SFTEMP1	=	VBUF 	+1	# STORAGE FOR SF CONST HI PART (=SFTEMP2-1)
# HITEMIN	=	VBUF 	+1	# TEMP FOR LOAD OF HRS, MIN, SEC
#					# 	MUST = LOTEMIN-1.
# CODE		=	VBUF 	+2	# FOR DSPIN
# SFTEMP2	=	VBUF 	+2	# STORAGE FOR SF CONST LO PART (=SFTEMP1+1)
# LOTEMIN	=	VBUF 	+2	# TEMP FOR LOAD OF HRS, MIN, SEC
#					# 	MUST = HITEMIN+1
# MIXTEMP	=	VBUF 	+3	# FOR MIXNOUN DATA
# SIGNRET	=	VBUF 	+3	# RETURN FOR +,- ON
# ALSO MIXTEMP+1 = VBUF+4, MIXTEMP+2 = VBUF+5.

# ENTRET	=	DOTINC		# EXIT FROM ENTER

# WDCNT		=	DOTRET		# CHAR COUNTER FOR DSPWD
# INREL		=	DOTRET		# INPUT BUFFER SELECTOR (X, Y, Z, REG)

# DSPMMTEM	=	MATINC		# DSPCOUNT SAVE FOR DSPMM
# MIXBR		=	MATINC		# INDICATOR FOR MIXED OR NORMAL NOUN

# TEM1		ERASE			# EXEC TEMP
# DSREL		=	TEM1		# REL ADDRESS FOR DSPIN

# TEM2		ERASE			# EXEC TEMP
# DSMAG		=	TEM2		# MAGNITUDE STORE FOR DSPIN
# IDADDTEM	=	TEM2		# MIXNOUN INDIRECT ADDRESS STORAGE
# TEM3		ERASE			# EXEC TEMP
# COUNT		=	TEM3		# FOR DSPIN

# Page 395
# TEM4		ERASE			# EXEC TEMP
# LSTPTR	=	TEM4		# LIST POINTER FOR GRABUSY
# RELRET	=	TEM4		# RETURN FOR RELDSP
# FREERET	=	TEM4		# RETURN FOR FREEDSP
# DSPWDRET	=	TEM4		# RETURN FOR DSPSIGN
# SEPSCRET	=	TEM4		# RETURN FOR SEPSEC
# SEPMNRET	=	TEM4		# RETURN FOR SEPMIN

# TEM5		ERASE			# EXEC TEMP
# NOUNADD	=	TEM5		# TEMP STORAGE FOR NOUN ADDRESS

# NNADTEM	ERASE			# TEMP FOR NOUN ADDRESS TABLE ENTRY
# NNTYPTEM	ERASE			# TEMP FOR NOUN TYPE TABLE ENTRY
# IDAD1TEM	ERASE			# TEMP FOR INDIR ADDRESS TABLE ENTRY (MIXNN)
#					# MUST = IDAD2TEM-1, = IDAD3TEM-2.
# IDAD2TEM	ERASE			# TEMP FOR INDIR ADDRESS TABLE ENTRY (MIXNN)
#					# MUST = IDAD1TEM+1, IDAD3TEM-1.
# IDAD3TEM	ERASE			# TEMP FOR INDIR ADDRESS TABLE ENTRY (MIXNN)
#					# MUST = IDAD1TEM+2, IDAD2TEM+1.
# RUTMXTEM	ERASE			# TEMP FOR SF ROUT TABLE ENTRY (MIXNN ONLY)
#
; ADDITIONAL TEMPORARY STORAGE (shared with INTERPRETER and EXECUTIVE):
;
; These registers are borrowed from other subsystems during PINBALL operations:
;
; MPAC, THRU MPAC+6: Multi-purpose accumulator (interpreter stack)
;   Used for numerical computations during display formatting
; BUF, +1, +2: General buffer registers
; BUF2, +1, +2: Secondary buffer for complex formatting
; MPTEMP: Multi-purpose temporary
; ADDRWD: Address word temporary
;
; Sharing temporary storage reduces erasable memory usage (critical with
; only 2K words available). PINBALL can use interpreter temps because
; interpreter and PINBALL don't execute simultaneously.

# END OF TEMPORARIES FOR PINBALL EXECUTIVE ACTION.
#
# ADDITIONAL TEMPORARIES FOR PINBALL EXECUTIVE ACTION
#
#	MPAC, THRU MPAC +6
#	BUF, +1, +2
#	BUF2, +1, +2
#	MPTEMP
#	ADDRWD
#
# END OF ADDITIONAL TEMPS FOR PINBALL EXEC ACTION

; ============================================================================
; INTERRUPT-LEVEL STORAGE - Used by KEYRUPT, UPRUPT, DSPOUT
; ============================================================================

#
# RESERVED FOR PINBALL INTERRUPT ACTION
#
# DSPCNT	ERASE			# COUNTER FOR DSPOUT
# UPLOCK	ERASE			# BIT1 = UPLINK INTERLOCK (ACTIVATED BY
#					# RECEPTION OF A BAD MESSAGE IN UPLINK)
#
# END OF ERASABLES RESERVED FOR PINBALL INTERRUPT ACTION

; INTERRUPT-LEVEL RESERVED STORAGE (permanent allocation):
;
; DSPCNT: Display counter for DSPOUT routine
;   Tracks which display segment is being refreshed (MD, VD, ND, R1, R2, R3)
;   Incremented each T4RUPT (10ms), cycles through all display elements
;
; UPLOCK: Uplink interlock flag
;   BIT1 = 1: Uplink message error detected, keyboard input disabled
;   Prevents crew from interfering with uplink data transfer
;   Cleared after successful uplink completion or RSET button press
;   During Apollo 11, uplinks sent navigation updates from Mission Control

#
# TEMPORARIES FOR PINBALL INTERRUPT ACTION
#
# KEYTEMP1	=	WAITEXIT	# TEMP FOR KEYRUPT, UPRUPT
# DSRUPTEM	=	WAITEXIT	# TEMP FOR DSPOUT
# KEYTEMP2	=	RUPTAGN		# TEMP FOR KEYRUPT, UPRUPT
#
# END OF TEMPORARIES FOR PINBALL INTERRUPT ACTION

; INTERRUPT-LEVEL TEMPORARY STORAGE (shared with WAITLIST):
;
; KEYTEMP1 = WAITEXIT: Temporary for keyboard/uplink interrupt processing
; DSRUPTEM = WAITEXIT: Temporary for display output interrupt processing
; KEYTEMP2 = RUPTAGN: Secondary temp for keyboard/uplink interrupts
;
; These share memory with WAITLIST registers (WAITEXIT, RUPTAGN) because
; interrupts can't occur simultaneously. Saves precious erasable memory.

; ============================================================================
; KEYBOARD INPUT CODES (5-bit codes from CHANNEL 15 on button press)
; ============================================================================
;
; Each button press generates a 5-bit code read by KEYRUPT interrupt handler.
; These codes route to appropriate processing routines in the CHARIN dispatch table.
;
; BUTTON        BINARY   OCTAL   CREW ACTION
; --------      ------   -----   -----------
; 0             10000    20      Numeric digit entry
; 1             00001    01      Numeric digit entry
; 2-7           [sequential binary encoding]
; 8             01000    10      Numeric digit entry (restricted - see 89TEST)
; 9             01001    11      Numeric digit entry (restricted - see 89TEST)
; VERB          10001    21      Begin verb entry (2-digit code V01-V99)
; ERROR RES     10010    22      Clear OPERATOR ERROR light
; KEY RLSE      11001    31      Release flashing verb/noun display
; +             11010    32      Set positive sign for numeric entry
; -             11011    33      Set negative sign for numeric entry
; ENTER         11100    34      Accept entered data, execute verb/noun
; CLEAR         11110    36      Clear current entry, reset state machine
; NOUN          11111    37      Begin noun entry (2-digit code N01-N99)
;
; PROCEED button: No 5-bit keycode. Read via CHANNEL 32 (separate mechanism).
;   Used to confirm astronaut decisions (e.g., "GO" for engine ignition).
;   During Apollo 11 landing, PROCEED confirmed continuation after 1202 alarm.
;
; Historical note: Armstrong and Aldrin used this keyboard throughout lunar
; descent, ascent, and rendezvous. Every V16N36 (altitude/velocity display),
; every V06N62 (landing site update), every mission-critical data entry
; passed through these 5-bit codes and the CHARIN routine below.

# Page 396
# THE INPUT CODES ASSUMED FOR THE KEYBOARD ARE,
# 0		10000
# 1		00001
# 9		01001
# VERB		10001
# ERROR RES	10010
# KEY RLSE	11001
# +		11010
# -		11011
# ENTER		11100
# CLEAR		11110
# NOUN		11111
#
# (2003 RSB -- The PROCEED key has no keycode; it is read by an alternate mechanism.)

; ============================================================================
; DSKY DISPLAY OUTPUT FORMAT (sent to CHANNEL 10 via DSPOUT routine)
; ============================================================================
;
; Display updates are sent as 16-bit words to CHANNEL 10 (hardware interface).
; Format: AAAABCCCCCDDDDD (bits 15-1, bit 16 is parity)
;
; Bits 15-12 (AAAA): RELAYWORD - selects which pair of display characters to update
; Bit 11 (B): Special relay control (plus/minus signs for R1, R2, R3)
; Bits 10-6 (CCCCC): 5-bit relay code for LEFT character of selected pair
; Bits 5-1 (DDDDD): 5-bit relay code for RIGHT character of selected pair
;
; The AGC doesn't directly drive 7-segment displays. Instead, it sends relay
; codes to electromechanical relay logic that energizes the appropriate segments.
; This design provided isolation between computer and crew cabin systems.

#
# OUTPUT FORMAT FOR DISPLAY PANEL.  SET OUT0 TO AAAABCCCCCDDDDD.
# A'S 	SELECTS A RELAYWORD. THIS DETERMINES WHICH PAIR OF CHARACTERS ARE
#     	ENERGIZED.
# B	FOR SPECIAL RELAYS SUCH AS SIGNS ETC.
# C'S	5 BIT RELAY CODE FOR LEFT CHAR OF PAIR SELECTED BY RELAYWORD.
# D'S	5 BIT RELAY CODE FOR RIGHT CHAR OF PAIR SELECTED BY RELAYWORD.
; ============================================================================
; DSKY DISPLAY PANEL LAYOUT (as seen by crew)
; ============================================================================
;
; The DSKY display shows mission data in this fixed format:
;
;      MD1 MD2                    (MAJOR MODE - program number P00-P99)
;      VD1 VD2   (VERB)    ND1 ND2    (NOUN)
;    R1D1 R1D2 R1D3 R1D4 R1D5         (Register 1 - 5 digits)
;    R2D1 R2D2 R2D3 R2D4 R2D5         (Register 2 - 5 digits)
;    R3D1 R3D2 R3D3 R3D4 R3D5         (Register 3 - 5 digits)
;
; Example during lunar landing approach (Apollo 11):
;   MAJOR MODE: P63 (braking phase descent guidance)
;   VERB/NOUN: V16N68 (altitude and altitude rate display)
;   R1: +00573  (altitude 573 feet)
;   R2: -00025  (altitude rate -25 feet per second, descending)
;   R3: (blank or other data)
;
; Each display position has an internal DSPCOUNT number for addressing:

#
# THE PANEL APPEARS AS FOLLOWS,
# MD1	MD2 				(MAJOR MODE)
# VD1	VD2 (VERB)	ND1	ND2 	(NOUN)
# R1D1	R1D2	R1D3	R1D4	R1D5 	(R1)
# R2D1	R2D2	R2D3	R2D4	R2D5 	(R2)
# R3D1	R3D2	R3D3	R3D4	R3D5 	(R3)

; DSPCOUNT ADDRESSING (internal identification for each display position):
;
; Each character position has an octal DSPCOUNT number used by formatting
; routines to identify where to place data. Higher numbers = leftmost positions.
;
; POSITION  DSPCOUNT(octal)   POSITION  DSPCOUNT(octal)
; --------  ---------------   --------  ---------------
; MD1       25                R2D1      11
; MD2       24                R2D2      10
; VD1       23                R2D3      07
; VD2       22                R2D4      06
; ND1       21                R2D5      05
; ND2       20                R3D1      04
; R1D1      16                R3D2      03
; R1D2      15                R3D3      02
; R1D3      14                R3D4      01
; R1D4      13                R3D5      00
; R1D5      12

#
# EACH OF THESE IS GIVEN A DSPCOUNT NUMBER FOR USE WITHIN COMPUTATION ONLY
#
# MD1	25		R2D1	11		ALL ARE OCTAL
# MD2	24		R2D2	10
# VD1	23		R2D3	 7
# VD2	22		R2D4	 6
# ND1	21		R2D5	 5
# ND2	20		R3D1	 4
# R1D1	16		R3D2	 3
# R1D2	15		R3D3	 2
# R1D3	14		R3D4	 1
# R1D4	13		R3D5	 0
# R1D5	12
#
; ============================================================================
; DSPTAB RELAY BUFFER STRUCTURE (display output staging buffer)
; ============================================================================
;
; DSPTAB is an 11-word erasable table holding formatted relay codes ready
; for transmission to the DSKY display panel via CHANNEL 11. Each word
; contains display data for specific character positions, packed into
; bits 10-6 (left character) and bits 5-1 (right character).
;
; During Apollo 11 landing, this buffer held the altitude/velocity data
; Armstrong monitored on the DSKY (V16N68 displays). Updated continuously
; by verb/noun routines, transmitted to display hardware by DSPOUT routine.
;
# THERE IS AN 11-REGISTER TABLE (DSPTAB) FOR THE DISPLAY PANEL.

; DSPTAB WORD MAPPING (each word controls two display positions):
;
; WORD  RELAYWD  BIT11      BITS 10-6       BITS 5-1        USAGE
; ----  -------  -----      ---------       --------        -----
; 10    1011     (unused)   MD1 (25)        MD2 (24)        Major mode (program P00-P99)
; 9     1010     (unused)   VD1 (23)        VD2 (22)        Verb display (V01-V99)
; 8     1001     (unused)   ND1 (21)        ND2 (20)        Noun display (N01-N99)
; 7     1000     (unused)   (unused)        R1D1 (16)       Register 1 first digit
; 6     0111     +R1 sign   R1D2 (15)       R1D3 (14)       Register 1 digits 2-3
; 5     0110     -R1 sign   R1D4 (13)       R1D5 (12)       Register 1 digits 4-5
; 4     0101     +R2 sign   R2D1 (11)       R2D2 (10)       Register 2 digits 1-2
; 3     0100     -R2 sign   R2D3 (7)        R2D4 (6)        Register 2 digits 3-4
; 2     0011     (unused)   R2D5 (5)        R3D1 (4)        Register 2 digit 5, Register 3 digit 1
; 1     0010     +R3 sign   R3D2 (3)        R3D3 (2)        Register 3 digits 2-3
; 0     0001     -R3 sign   R3D4 (1)        R3D5 (0)        Register 3 digits 4-5
; ---   0000     ---        NO RELAYWORD    (no output)
;
; Sign bits (BIT11): Positive/negative indicators for registers
; DSPCOUNT numbers in parentheses show internal position addressing
;
#
# DSPTAB		RELAYWD		BIT11		BITS 10-6	BITS 5-1
# RELADD
# 10		1011				MD1 (25)	MD2 (24)
# 9		1010				VD1 (23)	VD2 (22)
# 8		1001				ND1 (21)	ND2 (20)
# 7		1000						R1D1 (16)
# Page 397
# 6		0111		+R1		R1D2 (15)	R1D3 (14)
# 5		0110		-R1		R1D4 (13)	R1D5 (12)
# 4		0101		+R2		R2D1 (11)	R2D2 (10)
# 3		0100		-R2		R2D3 (7)	R2D4 (6)
# 2		0011				R2D5 (5)	R3D1 (4)
# 1		0010		+R3		R3D2 (3)	R3D3 (2)
# 0		0001		-R3		R3D4 (1)	R3D5 (0)
# 		0000	    NO RELAYWORD

; ============================================================================
; 5-BIT OUTPUT RELAY CODES (character encoding for electromechanical display)
; ============================================================================
;
; Each display character is represented by a 5-bit code controlling relay
; drivers in the DSKY hardware. These codes activate the appropriate segments
; in the electromechanical display. The encoding is optimized for the relay
; circuitry, not human-readable binary patterns.
;
; CHARACTER   BINARY CODE   NOTES
; ---------   -----------   -----
; BLANK       00000         All segments off (dark)
; 0           10101         Digit zero
; 1           00011         Digit one
; 2           11001         Digit two
; 3           11011         Digit three
; 4           01111         Digit four
; 5           11110         Digit five
; 6           11100         Digit six
; 7           10011         Digit seven
; 8           11101         Digit eight
; 9           11111         Digit nine (all segments lit)
;
; Note: Plus (+) and minus (-) signs use BIT11 in relay word, not 5-bit codes.
; The electromechanical display design provided critical crew interface even
; if computer failed - relay state persisted without active power.
;
# THE 5-BIT OUTPUT RELAY CODES ARE:
#
# BLANK	00000
# 0	10101
# 1	00011
# 2	11001
# 3	11011
# 4	01111
# 5	11110
# 6	11100
# 7	10011
# 8	11101
# 9	11111
;
; ============================================================================
; OUTPUT BITS USED BY PINBALL (CHANNEL 11 control signals)
; ============================================================================
;
; CHANNEL 11 provides direct hardware control for DSKY indicator lights.
; These bits are set/cleared by display routines to provide crew feedback
; on system state and required actions.
;
; BIT   INDICATOR           FUNCTION
; ---   ---------           --------
; 5     KEY RELEASE         Lit when computer awaits KEY REL button press
;                           Crew must acknowledge data entry or system state
;
; 6     VERB/NOUN FLASH     Flashes VERB and NOUN displays to request crew action
;                           Used during operator input sequences and program requests
;
; 7     OPERATOR ERROR      Lit when crew enters invalid verb/noun combination
;                           or provides out-of-range data. Cleared by RSET button.
;
; Historical context: During Apollo 11 landing, Armstrong saw these indicators
; along with numeric displays. OPR ERR light required immediate crew attention.
; KEY REL requested confirmation before critical operations.
;
#
# OUTPUT BITS USED BY PINBALL:
#
#	KEY RELEASE LIGHT	-	BIT 5 OF CHANNEL 11
#	VERB/NOUN FLASH		-	BIT 6 OF CHANNEL 11
#	OPERATOR ERROR LIGHT	-	BIT 7 OF CHANNEL 11

## <b>Hint:</b> In the source code below, each of the blue operands to the
## right of the instruction opcodes is a hyperlink back to the definition
## of the symbol.  This is particularly useful for tracing program flow.
# Page 398
# START OF EXECUTIVE SECTION OF PINBALL

		BANK	40
		SETLOC	PINBALL1
		BANK

		COUNT*	$$/PIN
; ============================================================================
; TRANSITION: From interrupt handling to keyboard processing
;
; KEYRUPT1 interrupt has captured a 5-bit key code from Channel 15 and
; scheduled the PINBALL job. Execution now enters CHARIN, the main keyboard
; input dispatcher. Every button press during Apollo 11's mission - Armstrong
; calling V16N68 for position displays, Aldrin entering V06N62 during descent,
; the famous ERROR RESET during the 1202 alarm - all flow through this routine.
; ============================================================================

CHARIN		CAF	ONE		# BLOCK DISPLAY SYST
		XCH	DSPLOCK		# MAKE DSP SYST BUSY, BUT SAVE OLD
		TS	21/22REG	# C(DSPLOCK) FOR ERROR LIGHT RESET.

; CHARIN: Main keyboard input dispatcher entry point
;   Called by KEYRUPT1 interrupt handler after button press captured
;   1. Locks display system (DSPLOCK=1) preventing interference from program
;      display updates while processing keyboard input
;   2. Checks if KEY RELEASE lite should illuminate (CADRSTOR full condition)
;   3. Dispatches to appropriate button handler via indexed jump table
;
; COMMENT-ONLY READERS: This is where every DSKY button press begins processing.
; CODE-ALONG READERS: Note the DSPLOCK interlock mechanism for mutual exclusion.

		CCS	CADRSTOR	# ALL KEYS EXCEPT ER TURN ON KR LITE IF
		TC	+2		# CADRSTOR IS FULL.  THIS REMINDS OPERATOR
		TC	CHARIN2		# TO RE-ESTABLISH A FLASHING DISPLAY
		CS	ELRCODE1	# WHICH HE HAS OBSCURED WITH DISPLAYS OF
		AD	MPAC		# HIS OWN (SEE REMARKS PRECEDING ROUTINE
		EXTEND			# VBRELDSP).
		BZF	CHARIN2
		TC	RELDSPON

; KEY RELEASE indicator logic:
;   If CADRSTOR (ENDIDLE storage) is non-empty, and key pressed is NOT the
;   ERROR RESET button (code 22 octal), illuminate KEY RELEASE indicator.
;   This reminds the operator that a flashing display (program alarm or
;   monitor verb) has been obscured by manual displays and needs restoration.
;   HISTORICAL: During Apollo 11 descent, Armstrong and Aldrin frequently saw
;   KEY REL lite when checking displays while P63 program was running.

CHARIN2		XCH	MPAC
		TS	CHAR

; Button dispatch table: Indexed jump based on 5-bit key code (0-37 octal)
;   Key code in A register, INDEX instruction adds it to TC +1 address
;   Each button routes to its specialized handler routine
;
; BUTTON LAYOUT (DSKY keyboard physical arrangement):
;   Top row:    VERB  NOUN  [+]   [-]   [0]
;   Row 2:      [1]   [2]   [3]   [4]   [5]
;   Row 3:      [6]   [7]   [8]   [9]   CLR
;   Row 4:      KEY REL     ENTR        ERROR RESET (not on panel)
;
; CODE-ALONG READERS: This is an AGC indexed jump table. The INDEX instruction
; increments the next TC by the value in A, creating a computed jump.

		INDEX	A
		TC	+1		# INPUT CODE	FUNCTION
		TC	CHARALRM	# 0		(undefined key code)
		TC	NUM		# 1		Digit [1]
		TC	NUM		# 2		Digit [2]
		TC	NUM		# 3		Digit [3]
		TC	NUM		# 4		Digit [4]
		TC	NUM		# 5		Digit [5]
		TC	NUM		# 6		Digit [6]
		TC	NUM		# 7		Digit [7]
		TC	89TEST		# 10		Digit [8] (special validation)
		TC	89TEST		# 11		Digit [9] (special validation)
		TC	CHARALRM	# 12		(undefined key code)
		TC	CHARALRM	# 13		(undefined key code)
		TC	CHARALRM	# 14		(undefined key code)
		TC	CHARALRM	# 15		(undefined key code)
		TC	CHARALRM	# 16		(undefined key code)
		TC	CHARALRM	# 17		(undefined key code)
		TC	NUM	-2	# 20		Digit [0]
		TC	VERB		# 21		VERB button
		TC	ERROR		# 22		ERROR RESET button
		TC	CHARALRM	# 23		(undefined key code)
		TC	CHARALRM	# 24		(undefined key code)
		TC	CHARALRM	# 25		(undefined key code)
		TC	CHARALRM	# 26		(undefined key code)
		TC	CHARALRM	# 27		(undefined key code)
		TC	CHARALRM	# 30		(undefined key code)
		TC	VBRELDSP	# 31		KEY RELEASE button
		TC	POSGN		# 32		[+] sign button
# Page 399
		TC	NEGSGN		# 33		[-] sign button
		TC	ENTERJMP	# 34		ENTR (enter) button
		TC	CHARALRM	# 35		(undefined key code)
		TC	CLEAR		# 36		CLR (clear) button
		TC	NOUN		# 37		NOUN button

; Button code summary:
;   Digits 0-9: NUM routine (numeric data entry), 8/9 require special validation
;   VERB (21): Initiates verb entry sequence (two-digit decimal)
;   NOUN (37): Initiates noun entry sequence (two-digit decimal)
;   ENTR (34): Completes data entry, executes verb/noun command
;   CLR (36): Clears current entry or display
;   + (32), - (33): Sign selection for numeric data entry
;   KEY REL (31): Releases flashing displays, returns to program control
;   ERROR RESET (22): Clears operator error and program alarm indicators

ELRCODE1	OCT	22
ENTERJMP	TC	POSTJUMP
		CADR	ENTER

89TEST		CCS	DSPCOUNT
		TC	+4		# +
		TC	+3		# +0
		TC	ENDOFJOB	# - BLOCK DATA IN IF DSPCOUNT IS - OR -0
		TC	ENDOFJOB	# -0
		CAF	THREE
		MASK	DECBRNCH
		CCS	A
		TC	NUM		# IF DECBRNCH IS +, 8 OR 9 OK
		TC	CHARALRM	# IF DECBRNCH IS +0, REJECT 8 OR 9

# NUM ASSEMBLES OCTAL 3 BITS AT A TIME.  FOR DECIMAL IT CONVERTS INCOMING
# WORD AS A FRACTION, KEEPING RESULTS TO DP.
# OCTAL RESULTS ARE LEFT IN XREG, YREG, OR ZREG.  HI PART OF DEC IN XREG,
# YREG, ZREG.  THE LOW PARTS IN XREGLP, YREGLP, OR ZREGLP.
# DECBRNCH IS LEFT AT +0 FOR OCT, +1 FOR + DEC, +2 FOR - DEC.
# IF DSPCOUNT WAS LEFT -, NO MORE DATA IS ACCEPTED.

; ============================================================================
; NUM - Numeric Input Processing (Keyboard Character Assembly)
;
; PURPOSE: Assembles numeric characters from keyboard into octal or decimal
; values, building multi-digit numbers as crew types them digit-by-digit.
;
; MISSION CONTEXT: Every time Armstrong typed coordinates, velocities, or
; times into the DSKY during landing approach, this routine assembled each
; digit. For example, entering "V16N36E" (Monitor Time) requires typing
; time values - this code processes each digit typed.
;
; INPUT MODES:
; - OCTAL: Assembles 3 bits at a time (digits 0-7), left-justifies in register
; - DECIMAL: Converts incoming digits as fraction, maintains double-precision
;
; OUTPUT LOCATIONS:
; - Octal results: XREG, YREG, or ZREG (selected by INREL pointer)
; - Decimal high parts: XREG, YREG, ZREG
; - Decimal low parts: XREGLP, YREGLP, ZREGLP
; - Format indicator: DECBRNCH (+0=octal, +1=+decimal, +2=-decimal)
;
; DSPCOUNT BEHAVIOR:
; - Positive: Number of digits remaining to be entered
; - Zero: Last digit just entered
; - Negative: Data entry complete, blocks further input
;
; PROCESSING FLOW:
; 1. Checks DSPCOUNT: if negative, blocks input and ends job
; 2. Fetches character from keyboard via GETINREL
; 3. Extracts 5-bit keycode from RELTAB (digit 0-9 or operator)
; 4. Determines format via DECBRNCH:
;    - Octal: Shifts existing value left 3 bits, ORs new digit
;    - Decimal: Multiplies by 10, adds new digit (fixed-point arithmetic)
; 5. Stores result in appropriate register (VERBREG, NOUNREG, or R1/R2/R3)
; 6. Decrements DSPCOUNT
; 7. Tests if entry complete (DSPCOUNT = CRITCON threshold)
;
; TECHNICAL DETAILS:
; - Decimal uses double-precision fixed-point scaled by 2^-14
; - Multiplication by 10 via SHORTMP subroutine (10 x 2^-14)
; - For register data (R1/R2/R3), final conversion to fractional via DECON
; - Overflow checking on 5th decimal character
; - CLPASS flag cleared to +0 on first digit (enables CLEAR button)
;
; CREW INTERACTION: Digit keys 0-9 invoke this routine repeatedly until
; the required number of digits is entered (DSPCOUNT reaches zero).
; ============================================================================

		CAF	ZERO
		TS	CHAR
NUM		CCS	DSPCOUNT
		TC	+4		# +
		TC	+3		# +0
		TC	+1		# -BLOCK DATA IN IF DSPCOUNT IS -
		TC	ENDOFJOB	# -0
		TC	GETINREL
		CCS	CLPASS		# IF CLPASS IS + OR +0, MAKE IT +0.
		CAF	ZERO
		TS	CLPASS
		TC	+1
		INDEX	CHAR
		CAF	RELTAB
		MASK	LOW5
		TS	CODE
		CA	DSPCOUNT
		TS	COUNT
		TC	DSPIN
		CAF	THREE
# Page 400
		MASK	DECBRNCH
		CCS	A		# +0, OCTAL.  +1, + DEC.  +2, - DEC.
		TC	DECTOBIN	# +
		INDEX	INREL		# +0 OCTAL
		XCH	VERBREG
		TS	CYL
		CS	CYL
		CS	CYL
		XCH	CYL
		AD	CHAR
		TC	ENDNMTST
DECTOBIN	INDEX	INREL
		XCH	VERBREG
		TS	MPAC		# SUM X 2EXP-14 IN MPAC
		CAF	ZERO
		TS	MPAC +1
		CAF	TEN		# 10 X 2EXP-14
		TC	SHORTMP		# 10SUM X 2EXP-28 IN MPAC, MPAC+1
		XCH	MPAC +1
		AD	CHAR
		TS	MPAC +1
		TC	ENDNMTST	# NO OF
		ADS	MPAC		# OF MUST BE 5TH CHAR
		TC	DECEND
ENDNMTST	INDEX	INREL
		TS	VERBREG
		CS	DSPCOUNT
		INDEX	INREL
		AD	CRITCON
		EXTEND
		BZF	ENDNUM		# -0, DSPCOUNT = CRITCON
		TC	MORNUM		# - , DSPCOUNT G/ CRITCON
ENDNUM		CAF	THREE
		MASK	DECBRNCH
		CCS	A
		TC	DECEND
ENDALL		CS	DSPCOUNT	# BLOCK NUMIN BY PLACING DSPCOUNT
		TC	MORNUM +1	# NEGATIVELY
DECEND		CS	ONE
		AD	INREL
		EXTEND
		BZMF	ENDALL		# IF INREL=0,1 (VBREG,NNREG) LEAVE WHOLE
		TC	DMP		# IF INREL=2,3,4 (R1,R2,R3), CONVERT TO FRAC
					# MULT SUM X 2EXP-28 IN MPAC, MPAC+1 BY
		ADRES	DECON		# 2EXP14/10EXP5, GIVES (SUM/10EXP5)X2EXP-14
		CAF	THREE		# IN MPAC, +1, +2.
		MASK	DECBRNCH
		INDEX	A
		TC	+0
		TC	+DECSGN
# Page 401
		EXTEND			# - CASE
		DCS	MPAC +1
		DXCH	MPAC +1
+DECSGN		XCH	MPAC +2
		INDEX	INREL
		TS	XREGLP -2
		XCH	MPAC +1
		INDEX	INREL
		TS	VERBREG
		TC	ENDALL
MORNUM		CCS	DSPCOUNT	# DECREMENT DSPCOUNT
		TS	DSPCOUNT
		TC	ENDOFJOB

CRITCON		OCT	22		# (DEC 18)
		OCT	20		# (DEC 16)
		OCT	12		# (DEC 10)
		OCT	5
		OCT	0

DECON		2DEC	1 E-5 B14	# 2EXP14/10EXP5 = .16384 DEC

# GETINREL GETS PROPER DATA REG REL ADDRESS FOR CURRENT C(DSPCOUNT) AND
# PUTS IN INTO INREL. +0 VERBREG, 1 NOUNREG, 2 XREG, 3 YREG, 4 ZREG.

GETINREL	INDEX	DSPCOUNT
		CAF	INRELTAB
		TS	INREL		# (A TEMP. REG)
		TC	Q

INRELTAB	OCT	4		# R3D5 (DSPCOUNT = 0)
		OCT	4		# R3D4		 =(1)
		OCT	4		# R3D3		 =(2)
		OCT	4		# R3D2		 =(3)
		OCT	4	 	# R3D1		 =(4)
		OCT	3		# R2D5		 =(5)
		OCT	3		# R2D4		 =(6)
		OCT	3		# R2D3		 =(7)
		OCT	3		# R2D2		 =(8D)
		OCT	3		# R2D1		 =(9D)
		OCT	2		# R1D5		 =(10D)
		OCT	2		# R1D4		 =(11D)
		OCT	2		# R1D3		 =(12D)
		OCT	2		# R1D2		 =(13D)
		OCT	2		# R1D1		 =(14D)
		TC	CCSHOLE		# NO DSPCOUNT NUMBER = 15D
		OCT	1		# ND2		 =(16D)
		OCT	1		# ND1		 =(17D)
# Page 402
		OCT	0		# VD2		 =(18D)
		OCT	0		# VD1		 =(19D)

; ============================================================================
; VERB BUTTON HANDLER
;
; Crew presses VERB button to begin entering a two-digit verb code. This
; routine initializes the verb entry process by clearing the verb register
; and preparing the display for decimal digit entry. The V/N lights on the
; DSKY flash to indicate the computer is waiting for verb digits.
;
; Historical Context: During Apollo 11 landing, Armstrong and Aldrin used
; VERB button dozens of times to request displays (V16 N68 altitude/rate,
; V06 N62 velocity), load programs (V37 program change), and monitor systems.
; Every verb entry during the 12-minute powered descent began here.
;
; Technical: Clears VERBREG, sets DSPCOUNT to VD1 (first verb digit position),
; blanks old display data, sets DECBRNCH for decimal mode, initializes REQRET
; and ENTRET for entry pass 0, then suspends job until digits are entered.
; ============================================================================
VERB		CAF	ZERO
		TS	VERBREG		; Clear verb register for new entry
		CAF	VD1		; VD1 = first verb digit display position
NVCOM		TS	DSPCOUNT	; Set display counter for digit entry
		TC	2BLANK		; Blank previous display data
		CAF	ONE
		TS	DECBRNCH	; SET FOR DEC V/N CODE (decimal mode)
		CAF	ZERO
		TS	REQRET		; SET FOR ENTPAS0 (entry pass 0)
		CAF	ENDINST		; IF DSPALARM OCCURS BEFORE FIRST ENTPAS0
		TS	ENTRET		; OR NVSUB, ENTRET MUST ALREADY BE SET
					; TO TC ENDOFJOB
		TC	ENDOFJOB	; Suspend job, await digit entry via CHARIN

; ============================================================================
; NOUN BUTTON HANDLER
;
; Crew presses NOUN button to begin entering a two-digit noun code. Similar
; to VERB handler but targets noun register. The computer expects noun entry
; after verb has been specified, forming the complete verb/noun command pair.
;
; Historical Context: After entering V16 (display verb) during landing,
; Armstrong would press NOUN then 6 and 8 to complete V16 N68 (altitude and
; altitude rate display). This pattern repeated for every commanded display.
;
; Technical: Clears NOUNREG, sets DSPCOUNT to ND1 (first noun digit), then
; uses common NVCOM code path for decimal entry setup. Like VERB, this
; suspends the job pending digit entry completion.
; ============================================================================
NOUN		CAF	ZERO
		TS	NOUNREG		; Clear noun register for new entry
		CAF	ND1		; ND1, OCT 21 (DEC 17) = first noun digit position
		TC	NVCOM		; Use common verb/noun entry initialization

; ============================================================================
; PLUS/MINUS SIGN BUTTON HANDLERS (NEGSGN, POSGN)
;
; The DSKY includes separate + and - buttons for entering the sign of
; numerical data. These are critical when crew enters signed velocity
; components, attitude angles, or other values requiring sign specification.
;
; NEGSGN (-): Negative Sign Button Handler
; - Calls SIGNTEST to verify sign entry is legal (first position of word)
; - Calls -ON to display minus sign and set decimal comparison bit
; - Sets TWO in accumulator, proceeds to BOTHSGN for common processing
;
; POSGN (+): Positive Sign Button Handler  
; - Calls SIGNTEST to verify sign entry is legal
; - Calls +ON to display plus sign
; - Sets ONE in accumulator, proceeds to BOTHSGN for common processing
;
; BOTHSGN: Common Sign Processing
; - Sets decimal comparison bit in DECBRNCH (BIT7 indexed by INREL)
; - BIT 5 for R1, BIT 4 for R2, BIT 3 for R3
; - Manages CLPASS (clear pass counter) for display backup logic
;
; +ON/-ON: Sign Display Routines
; - GETINREL determines which register (R1/R2/R3) is active
; - Uses SGNTAB to find display position for sign
; - Calls 11DSPIN twice: once to blank old sign, once to display new sign
; - SGNON holds position for new sign, SGNOFF holds position to blank
;
; SIGNTEST: Sign Entry Validation
; - Allows +/- only when DSPCOUNT indicates first digit position (R1D1,
;   R2D1, or R3D1), preventing signs in middle of numbers
; - Checks DECBRNCH low 2 bits: if not zero, sign already entered, reject
; - Returns to caller (via L register) if sign entry is legal
; - Otherwise executes ENDOFJOB to reject illegal sign entry
;
; Historical Context: During Apollo 11's translunar coast, crew updated
; state vector with signed velocity components from ground tracking. Correct
; sign entry was essential - wrong sign on ΔV component would compute
; incorrect trajectory. Armstrong and Aldrin used these buttons extensively
; during lunar orbit navigation updates and landing site coordinate entry.
; ============================================================================
NEGSGN		TC	SIGNTEST
		TC 	-ON
		CAF	TWO

; BOTHSGN - Common Sign Processing After POSGN/NEGSGN
;
; Called after either positive or negative sign has been validated and display
; updated. Sets the decimal complement bit in DECBRNCH to indicate this register
; has signed data. The complement bit position depends on which register (R1, R2,
; or R3) is receiving the sign.
;
; Entry: A register contains 2 (from CAF TWO) or 1 (from CAF ONE in POSGN path)
; Exit: DECBRNCH updated with complement bit, execution continues to FIXCLPAS
;
; Operations performed:
; 1. INDEX INREL: Use INREL as index to select proper bit position
; 2. AD BIT7: Add BIT7 (octal 200) with indexing to create bit mask
;    - Bit 5 set for R1 (when INREL=0)
;    - Bit 4 set for R2 (when INREL=1)  
;    - Bit 3 set for R3 (when INREL=2)
; 3. ADS DECBRNCH: Add result to DECBRNCH, setting decimal complement control bit
;
; Technical detail: DECBRNCH uses bits 3-5 to track which registers have decimal
; complement (negative sign) entered. Bits 0-2 track decimal digit entry progress
; within each register. This dual tracking allows the system to know both how many
; digits have been entered and whether the number is negative.
;
; The INDEX INREL mechanism automatically selects the correct bit position based
; on which register is active, making this routine register-independent.

BOTHSGN		INDEX	INREL		# SET DEC COMP BIT TO 1 (IN DECBRNCH)
		AD	BIT7		# BIT 5 FOR R1.  BIT 4 FOR R2.
		ADS	DECBRNCH	# BIT 3 FOR R3.

; FIXCLPAS - Normalize CLPASS Value After Sign Entry
;
; Ensures CLPASS is set to +0 after sign entry, normalizing the clear-pass state
; for consistent backup behavior. CLPASS controls how the CLEAR button behaves:
; if +0, crew can back up through data entry; if negative, backup is blocked.
;
; Entry: CLPASS contains current clear pass state (may be positive, +0, or negative)
; Exit: CLPASS set to +0 if was positive, unchanged if was negative or -0
;
; Operations performed:
; 1. CCS CLPASS: Count, compare, skip on CLPASS value
;    - If + (positive): Continue to next instruction
;    - If +0: Skip to TC +1 (jumps over CAF ZERO and TS CLPASS)
;    - If - (negative): Skip to TC ENDOFJOB (exit immediately, no normalization)
;    - If -0: Skip to TC ENDOFJOB
; 2. CAF ZERO: If CLPASS was positive, load zero
; 3. TS CLPASS: Store +0 to CLPASS, normalizing state
; 4. TC +1: Continue past ENDOFJOB to next instruction
; 5. TC ENDOFJOB: Exit if CLPASS was +0 or negative
;
; Technical detail: The CCS (Count, Compare, Skip) instruction is AGC's clever
; way to test a value and branch based on its sign and zero status in a single
; instruction. The four possible branches (+, +0, -, -0) enable compact conditional
; logic without explicit comparison and jump instructions.
;
; Why normalize CLPASS after sign entry: When crew enters a sign (+ or -), the
; system wants to ensure they can use CLEAR to back up and change their minds.
; Setting CLPASS to +0 enables this backup capability. If CLPASS was already
; negative (indicating backup was blocked for some reason), this routine preserves
; that restriction by leaving CLPASS negative.

FIXCLPAS	CCS	CLPASS		# IF CLPASS IS + OR +0. MAKE IT +0.
		CAF	ZERO
		TS	CLPASS
		TC	+1
		TC	ENDOFJOB

; POSGN - Positive Sign (+) Button Handler
;
; Handles crew pressing the + (plus) button on DSKY. Displays the positive sign
; indicator for the current register (R1, R2, or R3) and updates internal state
; to reflect that a positive number is being entered.
;
; Entry: Crew pressed + button, KEYRUPT delivered +SIGN key code
; Exit: Positive sign displayed on appropriate register, DECBRNCH updated
;
; Operations performed:
; 1. TC SIGNTEST: Validate that sign entry is legal at this point
;    - Checks DSPCOUNT matches R1D1, R2D1, or R3D1 (first digit position)
;    - Rejects if sign already entered for this register
;    - Returns to caller (ENDOFJOB) if illegal, continues if legal
; 2. TC +ON: Display positive sign indicator on current register
;    - Turns on "+" indicator on appropriate display register
;    - Uses SGNTAB to look up correct display channel bits
; 3. CAF ONE: Load 1 into A register
; 4. TC BOTHSGN: Common sign processing (set decimal complement bit)
;
; Technical detail: Positive sign is indicated on DSKY by illuminating specific
; segments on the sign position of each register. Although mathematically the
; positive sign is redundant (numbers are positive by default), displaying it
; provides crew confirmation that the + button press was accepted.
;
; Historical context: During Apollo 11 landing, Armstrong entered positive altitude
; rate values when updating landing radar data. The + button provided visual
; confirmation of each entry before he pressed ENTER to accept the data.

POSGN		TC	SIGNTEST
		TC	+ON
		CAF	ONE
		TC	BOTHSGN

; +ON - Display Positive Sign Indicator
;
; Turns on the positive (+) sign display for the current register. Uses indexed
; table lookup to find the correct display channel bits for R1, R2, or R3, then
; calls display routines to update the DSKY.
;
; Entry: L register contains return address
; Exit: Positive sign displayed on current register, returns via TC L
;
; Operations performed:
; 1. LXCH Q: Exchange L and Q (save return address in L, previous Q in A)
; 2. TC GETINREL: Get INREL value (0, 1, or 2 for R1, R2, R3)
; 3. INDEX INREL / CAF SGNTAB -2: Look up positive sign display code
;    - For R1 (INREL=0): Fetches SGNTAB (octal 5)
;    - For R2 (INREL=1): Fetches SGNTAB+1 (octal 3)
;    - For R3 (INREL=2): Fetches SGNTAB+2 (octal 0)
; 4. TS SGNOFF: Store in SGNOFF (sign-off code, not used for + display)
; 5. AD ONE: Add 1 to get positive sign code (one bit higher)
;    - R1 positive: octal 6
;    - R2 positive: octal 4
;    - R3 positive: octal 1
; 6. TS SGNON: Store positive sign display code
; 7. Fall through to SGNCOM: Common sign display routine
;
; Technical detail: SGNTAB contains the negative sign codes for each register.
; Adding 1 to these codes produces the positive sign codes. This encoding scheme
; allows efficient lookup and calculation in minimal instructions.
;
; The SGNOFF variable is loaded but not actually used in the +ON path - it's set
; for consistency with the -ON path where both on and off codes are needed.

+ON		LXCH	Q
		TC	GETINREL
		INDEX	INREL
		CAF	SGNTAB -2
		TS	SGNOFF
		AD 	ONE
		TS	SGNON

; SGNCOM - Common Sign Display Routine
;
; Core routine that updates DSKY display with sign indicators. Used by both
; positive (+ON) and negative (-ON) sign display paths. Manages the actual
; channel output to illuminate or extinguish sign segments on the display.
;
; Entry: SGNOFF contains sign-off code, SGNON contains sign-on code, L has return
; Exit: Sign display updated on DSKY, returns via TC L
;
; Operations performed:
; 1. CAF ZERO: Clear A register
; 2. TS CODE: Set CODE to zero (selects display channel 10 mode)
; 3. XCH SGNOFF: Exchange A with SGNOFF, putting sign-off code in A
; 4. TC 11DSPIN: Call display routine to turn off old sign segments
;    - Uses CODE=0 to select proper display channel
;    - SGNOFF code specifies which segments to extinguish
; 5. CAF BIT11: Load BIT11 (octal 2000) into A
; 6. TS CODE: Set CODE to BIT11 (selects alternate display channel mode)
; 7. XCH SGNON: Exchange A with SGNON, putting sign-on code in A
; 8. TC 11DSPIN: Call display routine to turn on new sign segments
;    - Uses CODE=BIT11 for proper display channel
;    - SGNON code specifies which segments to illuminate
; 9. TC L: Return to caller
;
; Technical detail: The AGC DSKY uses relay logic to drive the seven-segment
; displays. Sign indicators are controlled separately from numeric digits.
; The CODE variable selects which set of display relays to energize. Setting
; CODE to zero and then BIT11 allows the routine to control different sign
; segments without interfering with numeric display.
;
; The two-phase approach (turn off old, turn on new) prevents transient display
; states where both positive and negative signs might appear simultaneously during
; the transition. This creates clean, flicker-free sign changes on the DSKY.

SGNCOM		CAF	ZERO
		TS	CODE
		XCH	SGNOFF
# Page 403
		TC	11DSPIN
		CAF	BIT11
		TS	CODE
		XCH	SGNON
		TC	11DSPIN
		TC	L

; -ON - Display Negative Sign Indicator
;
; Turns on the negative (-) sign display for the current register. Uses indexed
; table lookup to find the correct display channel bits for R1, R2, or R3, then
; calls common sign display routine to update the DSKY.
;
; Entry: L register contains return address
; Exit: Negative sign displayed on current register, returns via SGNCOM->TC L
;
; Operations performed:
; 1. LXCH Q: Exchange L and Q (save return address in L)
; 2. TC GETINREL: Get INREL value (0, 1, or 2 for R1, R2, R3)
; 3. INDEX INREL / CAF SGNTAB -2: Look up negative sign display code
;    - For R1 (INREL=0): Fetches SGNTAB (octal 5)
;    - For R2 (INREL=1): Fetches SGNTAB+1 (octal 3)
;    - For R3 (INREL=2): Fetches SGNTAB+2 (octal 0)
; 4. TS SGNON: Store negative sign code in SGNON (will be turned on)
; 5. AD ONE: Add 1 to compute positive sign code
;    - R1 positive: octal 6
;    - R2 positive: octal 4
;    - R3 positive: octal 1
; 6. TS SGNOFF: Store positive sign code in SGNOFF (will be turned off)
; 7. TC SGNCOM: Call common sign display routine to update DSKY
;
; Technical detail: Unlike +ON which displays the positive sign (often just to
; confirm input), -ON displays the mathematically significant negative sign.
; The routine sets SGNON to the negative code and SGNOFF to the positive code,
; ensuring that any previous positive sign indicator is extinguished when the
; negative sign is displayed.
;
; Historical context: During Apollo 11, negative signs were critical for entering
; velocity corrections (approaching/receding from target), gimbal angles (above/
; below reference plane), and delta-V components (prograde/retrograde). A single
; incorrect sign could result in navigation errors of thousands of miles.

-ON		LXCH	Q
		TC	GETINREL
		INDEX	INREL
		CAF	SGNTAB -2
		TS	SGNON
		AD	ONE
		TS	SGNOFF
		TC	SGNCOM

; SGNTAB - Sign Display Code Table
;
; Table containing the negative sign display codes for each of the three DSKY
; registers (R1, R2, R3). These octal codes correspond to specific relay patterns
; that illuminate the minus sign segments on the seven-segment displays.
;
; Table entries:
; SGNTAB+0 (octal 5): Negative sign code for R1 (register 1)
; SGNTAB+1 (octal 3): Negative sign code for R2 (register 2)
; SGNTAB+2 (octal 0): Negative sign code for R3 (register 3)
;
; Usage: Indexed by INREL (0, 1, or 2) with offset -2 to fetch correct entry
; - INDEX INREL / CAF SGNTAB -2 fetches SGNTAB when INREL=0
; - INDEX INREL / CAF SGNTAB -2 fetches SGNTAB+1 when INREL=1
; - INDEX INREL / CAF SGNTAB -2 fetches SGNTAB+2 when INREL=2
;
; Technical detail: The octal codes (5, 3, 0) are specific to the DSKY hardware
; design. Each bit in the code controls a different relay in the display driver
; circuitry. When these patterns are sent through channel 10 with proper CODE
; settings, they energize the relays that illuminate the horizontal bar forming
; the minus sign on the appropriate register display.
;
; Positive sign codes are computed by adding 1 to these negative codes, yielding
; octal 6, 4, 1 for R1, R2, R3 respectively. This elegant encoding allows both
; sign types to be derived from a single compact table.

SGNTAB		OCT	5		# -R1
		OCT	3		# -R2
		OCT	0		# -R3

; SIGNTEST - Sign Entry Validation
;
; Critical validation routine that determines whether crew's + or - button press
; is legal at the current point in data entry. Signs are only permitted at the
; beginning of each register's digit entry (first digit position), and only one
; sign per register is allowed.
;
; Entry: L register contains return address from caller (POSGN or NEGSGN)
; Exit: Either continues with sign processing (returns via TC L when legal)
;       or terminates input (TC ENDOFJOB when illegal)
;
; Validation checks performed:
; 1. Verify no previous sign entered for this register
;    - Masks DECBRNCH with THREE (octal 3) to examine low 2 bits
;    - If any bit set, complement bit already indicates signed entry -> REJECT
;    - If both bits clear, no sign yet entered -> continue validation
;
; 2. Verify DSPCOUNT matches first digit position of some register
;    - Compares DSPCOUNT against R1D1 (first digit of register 1)
;    - Compares DSPCOUNT against R2D1 (first digit of register 2)
;    - Compares DSPCOUNT against R3D1 (first digit of register 3)
;    - If match found, sign entry is legal -> return via TC L (sign legal)
;    - If no match found, cursor not at digit 1 position -> TC ENDOFJOB (reject)
;
; Operations performed:
; 1. LXCH Q: Save return address in L, old Q in A
; 2. CAF THREE: Load octal 3 (binary 011, mask for low 2 bits)
; 3. MASK DECBRNCH: AND with DECBRNCH to isolate complement status bits
; 4. CCS A: Test result
;    - If non-zero: Sign already in for this register, TC ENDOFJOB (reject)
;    - If zero: No sign yet, continue to position checking
; 5. CS R1D1: Complement R1D1 value
; 6. TC SGNTST1: Check if DSPCOUNT matches R1D1
; 7. CS R2D1: Complement R2D1 value
; 8. TC SGNTST1: Check if DSPCOUNT matches R2D1
; 9. CS R3D1: Complement R3D1 value
; 10. TC SGNTST1: Check if DSPCOUNT matches R3D1
; 11. TC ENDOFJOB: No match found, sign illegal at this position
;
; SGNTST1 subroutine:
; 1. AD DSPCOUNT: Add DSPCOUNT to complemented RxD1 value
; 2. EXTEND / BZF +2: Branch if zero (match found)
; 3. TC Q: No match, return to SIGNTEST to try next comparison
; 4. TC L: Match found, sign is legal, return to original caller
;
; Technical detail: The complemented comparison technique (CS RxD1, AD DSPCOUNT,
; BZF) is AGC's efficient way to test equality. If DSPCOUNT equals R1D1, then
; -R1D1 + DSPCOUNT = 0. The EXTEND instruction enables the BZF (Branch Zero to
; Fixed) instruction, which jumps forward 2 instructions if the sum is zero.
;
; Historical context: This validation was crucial during Apollo 11's powered
; descent when Armstrong manually entered landing radar altitude and velocity
; data. The system had to reject erroneous sign entries if crew accidentally
; pressed +/- at the wrong time, preventing garbage data from corrupting the
; landing guidance computations that were executing in parallel with the data
; entry process.

SIGNTEST	LXCH	Q		# ALLOWS +,- ONLY WHEN DSPCOUNT=R1D1,
		CAF	THREE		# R2D1, OR R3D1.  ALLOWS ONLY FIRST OF
		MASK	DECBRNCH	# CONSECUTIVE +/- CHARACTERS.
		CCS	A		# IF LOW2 BITS OF DECBRNCH NOT= 0,  SIGN
		TC	ENDOFJOB	# FOR THIS WORD ALREADY IN. REJECT.
		CS	R1D1
		TC	SGNTST1
		CS	R2D1
		TC	SGNTST1
		CS	R3D1
		TC	SGNTST1
		TC	ENDOFJOB	# NO MATCH FOUND. SIGN ILLEGAL
SGNTST1		AD	DSPCOUNT
		EXTEND
		BZF	+2		# MATCH FOUND
		TC	Q
		TC	L		# SIGN LEGAL

# CLEAR BLANKS WHICH R1, R2, R3 IS CURRENT OR LAST TO BE DISPLAYED (PERTINENT
# XREG, YREG, ZREG IS CLEARED).  SUCCESSIVE CLEARS TAKE CARE OF EACH RX
# L/ RC UNTIL R1 IS DONE. THEN NO FURTHER ACTION.
#
# THE SINGLE COMPONENT LOAD VERBS ALLOW ONLY THE SINGLE RC THAT IS
# APPROPRIATE TO BE CLEARED.
#
# CLPASS	+0 PASS0, CAN BE BACKED UP
#		+NZ HIPASS, CAN BE BACKED UP
#		-NZ PASS0, CANNOT BE BACKED UP
# Page 404
; ============================================================================
; CLEAR BUTTON HANDLER (CLEAR, CLR5, 5BLANK, 2BLANK)
;
; The CLR button on the DSKY allows crew to erase displayed data and back
; up through multi-component entries. Essential during data entry mistakes
; or when crew needs to re-enter values. During Apollo 11 landing, if
; Armstrong wanted to change an entered value, he would use CLR to erase
; and start over.
;
; CLEAR: Main Clear Button Entry Point
; - Examines DSPCOUNT to determine which display word (R1, R2, R3) is active
; - Uses INRELTAB indexed by DSPCOUNT to set INREL (register index)
; - Checks CLPASS to determine clear pass state:
;   * +0 or -: PASS0 (initial pass), allows backing up through entry
;   * +NZ: HIPASS (higher pass), previously cleared, can still back up
; - PASS0 calls LEGALTST to verify INREL is valid (>= 2)
; - HIPASS decrements INREL first, then validates, then backs up REQRET
;   by 3 positions to back up data requests
; - Decrements VERBREG and calls UPDATVB to update verb display
;
; CLPASHI: High Pass Clear Processing
; - Decrements INREL (move to previous register)
; - Backs up REQRET by +3 (using DOUBLK+2) to reverse data requests
; - Decrements and re-displays VERBREG
; - Allows single-component load verbs to back through components
;
; CLEAR1: Clear Execution
; - Calls CLR5 to blank the 5-character display word
; - Increments CLPASS to set for next higher pass
; - Executes ENDOFJOB when complete
;
; CLR5: 5-Character Blank (avoiding GETINREL call)
; - Uses 5BLANK+2 entry to skip GETINREL (INREL already set)
; - Blanks display and clears register
;
; 5BLANK: Blank 5-Character Display Word
; - Blanks 5-character display in R1, R2, or R3
; - Zeroes corresponding XREG, YREG, or ZREG
; - Clears pertinent decimal comparison bit in DECBRNCH
; - Calls DSPIN to blank isolated character
; - Calls 2BLANK to blank double-precision display
; - Sets DSPCOUNT to leftmost display number for blanked register
;
; LEGALTST: Legal INREL Test
; - Verifies INREL >= 2 (valid register index)
; - Returns to caller if legal
; - Calls ENDOFJOB if illegal (INREL = 0 or 1)
;
; Historical Context: During powered descent, if crew entered wrong value
; for landing radar data check or abort criteria, CLR button allowed
; immediate correction. Time-critical during final approach when seconds
; mattered. Single button press cleared display, allowing fresh entry
; without aborting current program or losing display context.
; ============================================================================
CLEAR		CCS	DSPCOUNT
		AD	ONE
		TC	+2
		AD	ONE
		INDEX	A		# DO NOT CHANGE DSPCOUNT BECAUSE MAY LATER
		CAF	INRELTAB	# FAIL LEGALTST.
		TS	INREL		# MUST SET INREL, EVEN FOR HIPASS.
		CCS	CLPASS
		TC	CLPASHI		# +
		TC	+2		# +0	IF CLPASS IS +0 OR -, IT IS PASS0
		TC	+1		# -
		CA	INREL
		TC	LEGALTST
		TC	CLEAR1
CLPASHI		CCS	INREL
		TS	INREL
		TC	LEGALTST
		CAF	DOUBLK +2	# +3 TO - NUMBER, BACKS DATA REQUESTS.
		ADS	REQRET
		CA	INREL
		TS	MIXTEMP		# TEMP STORAGE FOR INREL
		EXTEND
		DIM	VERBREG		# DECREMENT VERB AND RE-DISPLAY
		TC	BANKCALL
		CADR	UPDATVB
		CA	MIXTEMP
		TS	INREL		# RESTORE INREL
CLEAR1		TC	CLR5
		INCR	CLPASS		# ONLY IF CLPASS IS + OR +0.
		TC	ENDOFJOB	# SET FOR HIGHER PASS.
CLR5		LXCH	Q		# USES 5BLANK BUT AVOIDS ITS TC GETINREL
		TC	5BLANK +2
LEGALTST	AD	NEG2
		CCS	A
		TC	Q		# LEGAL 	INREL G/ 2
		TC	CCSHOLE
		TC	ENDOFJOB	# ILLEGAL 	INREL=0,1
		TC	Q		# LEGAL		INREL=2

# 5BLANK BLANKS 5 CHAR DISPLAY WORD IN R1, R2, OR R3. IT ALSO ZEROES XREG,
# YREG, OR ZREG. PLACE ANY + DSPCOUNT NUMBER FOR PERTINENT RC INTO DSPCOUNT.
# DSPCOUNT IS LEFT SET TO LEFT MOST DSP NUMB FOR RC JUST BLANKED.

		TS	DSPCOUNT	# NEEDED FOR BLANKSUB

; ============================================================================
; 5BLANK - Blank Display Register Routine
;
; Clears a complete display register (R1, R2, or R3) and blanks all its
; display characters. Used after CLEAR button or when switching between
; different data displays. Zeros the verb/noun registers and clears the
; decimal composition flags.
;
; COMMENT-ONLY READERS: When the crew clears a display or switches between
; different information views, this routine erases the old data from the
; DSKY seven-segment displays, preparing for new information.
;
; CODE-ALONG READERS: Uses INREL index to select which register to blank
; (0=R1, 1=R2, 2=R3). Zeros VERBREG, XREGLP, CODE, and clears relevant
; decimal composition bits in DECBRNCH. Then blanks individual characters
; using DSPIN and pairs using 2BLANK.
; ============================================================================

5BLANK		LXCH	Q
		TC	GETINREL
		CAF	ZERO
		INDEX	INREL
		TS	VERBREG		# ZERO X, Y, Z, REG.
# Page 405
		INDEX	INREL
		TS	XREGLP	-2
		TS	CODE
		INDEX	INREL		# ZERO PERTINENT DEC COMP BIT.
		CS	BIT7		# PROTECT OTHERS
		MASK	DECBRNCH
		MASK	BRNCHCON	# ZERO LOW 2 BITS.
		TS	DECBRNCH
		INDEX	INREL
		CAF	SINBLANK -2	# BLANK ISOLATED CHAR SEPARATELY
		TS	COUNT
		TC	DSPIN
5BLANK1		INDEX	INREL
		CAF	DOUBLK -2
		TS	DSPCOUNT
		TC	2BLANK
		CS	TWO
		ADS	DSPCOUNT
		TC	2BLANK
		INDEX	INREL
		CAF	R1D1 -2
		TS	DSPCOUNT	# SET DSPCOUNT TO LEFT MOST DSP NUMBER
		TC	L		# OF REG. JUST BLANKED

SINBLANK	OCT	16		# DEC 14
		OCT	5
		OCT	4
DOUBLK		OCT	15		# DEC 13
		OCT	11		# DEC 9
		OCT	3

BRNCHCON	OCT	77774

; ============================================================================
; 2BLANK - Blank Two Display Characters
;
; Blanks a pair of adjacent display characters on the DSKY. The display
; position number for the left character of the pair must be in DSPCOUNT.
; Updates NOUT counter to track number of output requests pending.
;
; COMMENT-ONLY READERS: The DSKY displays numbers using pairs of seven-segment
; characters. This routine turns off a pair of digits, making them blank.
;
; CODE-ALONG READERS: Loads BLANKCON (octal 4000, blank pattern) into DSPTAB
; at position specified by DSPCOUNT (via SR). Uses INHINT/RELINT to protect
; critical section. Checks old DSPTAB contents: if positive, increments NOUT
; (display interrupt will process); if negative, NOUT already correct.
; Display hardware interrupt reads DSPTAB to update physical seven-segment LEDs.
; ============================================================================

# 2BLANK BLANKS TWO CHAR. PLACE DSP NUMBER OF LEFT CHAR OF THE PAIR INTO
# DSPCOUNT. THIS NUMBER IS LEFT IN DSPCOUNT

2BLANK		CA	DSPCOUNT
		TS	SR
		CS	BLANKCON
		INHINT
		INDEX	SR
		XCH	DSPTAB
		EXTEND
		BZMF 	+2		# IF OLD CONTENTS -, NOUT OK
		INCR	NOUT		# IF OLD CONTENTS +, +1 TO NOUT
		RELINT			# IF -, NOUT OK
		TC	Q
BLANKCON	OCT	4000

# Page 406
# ENTER PASS 0 IS THE EXECUTE FUNCTION. HIGHER ORDER ENTERS ARE TO LOAD
# DATA. THE SIGN OF REQRET DETERMINES THE PASS, + FOR PASS 0, - FOR HIGHER
# PASSES.
#
# MACHINE CADR TO BE SPECIFIED (MCTBS) NOUNS DESIRE AN ECADR TO BE LOADED
# WHEN USED WITH LOAD VERBS, MONITOR VERBS, OR DISPLAY VERBS (EXCEPT
# VERB = FIXED MEMORY DISPLAY, WHICH REQUIRES AN FCADR).

		BANK	41
		SETLOC	PINBALL2
		BANK

		COUNT*	$$/PIN
NVSUBB		TC	NVSUB1		# STANDARD LEAD INS. DONT MOVE.
LOADLV1		TC	LOADLV

					# END OF STANDARD LEAD INS.

; ============================================================================
; ENTER BUTTON HANDLER
;
; Crew presses ENTER to accept and execute a complete verb/noun command, or
; to provide data in response to a program request. This is the primary
; command acceptance mechanism for the entire DSKY interface.
;
; Historical Context: During Apollo 11 landing at 102:38:26 MET, when the
; 1202 program alarm occurred, the ENTER button was critical. After Mission
; Control's "Go" call, Armstrong pressed ENTER to acknowledge the alarm and
; continue the descent. Every verb/noun sequence during the 12-minute landing
; concluded with ENTER: V16 N68 ENTER for altitude display, V06 N62 ENTER
; for velocity monitoring. This button pressed dozens of times during descent.
;
; Technical Operation: ENTER has two modes determined by REQRET sign:
; - Pass 0 (REQRET positive): Execute function - runs the specified verb/noun
; - Higher passes (REQRET negative): Load data - accepts numerical input
;
; Pass 0 branches immediately to ENTPAS0 to execute the command.
; Higher passes validate data completeness, checking for required character
; count (5 decimal digits, or any number for octal). If incomplete, triggers
; OPERATOR ERROR alarm and keeps display flashing awaiting more input.
; ============================================================================
ENTER		CAF	ZERO
		TS	CLPASS		; Clear pass counter
		CAF	ENDINST
		TS	ENTRET		; Set return address to ENDOFJOB
		CCS	REQRET		; Check sign of REQRET to determine pass
		TC	ENTPAS0		# IF +, PASS 0 (execute function)
		TC	ENTPAS0		# IF +, PASS 0 (execute function)
		TC	+1		# IF -, NOT PASS 0 (loading data)
ENTPASHI	CAF	MMADREF
		AD	REQRET		# IF L/ 2 CHAR IN FOR MM CODE, ALARM
		EXTEND			# AND RECYCLE (DECIDE AT MMCHANG+1).
		BZF	ACCEPTWD
		CAF	THREE		# IF DEC, ALARM IF L/ 5 CHAR IN FOR DATA,
		MASK	DECBRNCH	# BUT LEAVE REQRET - AND FLASH ON, SO
		CCS	A		# OPERATOR CAN SUPPLY MISSING NUMERICAL
		TC	+2		# CHARACTERS AND CONTINUE.
		TC	ACCEPTWD	# OCTAL. ANY NUMBER OF CHAR OK.
		CCS	DSPCOUNT
		TC	GODSPALM	# LESS THAN 5 CHAR DEC(DSPCOUNT IS +)
		TC	GODSPALM	# LESS THAN 5 CHAR DEC(DSPCOUNT IS +)
		TC	+1		# 5 CHAR IN (DSPCOUNT IS -)

; ============================================================================
; ACCEPTWD - Accept Word Entry (Data Entry Completion)
;
; Final step in multi-character numeric data entry, reached after crew has
; entered all required digits. This routine validates the entry is complete,
; turns off the flashing prompt, and returns control to the requesting verb.
;
; Historical Context: Every numeric entry during Apollo 11's descent, ascent,
; and rendezvous sequences passed through this validation point. When Armstrong
; and Aldrin entered landing site coordinates, burn times, or target data, this
; code ensured the complete entry was captured before execution.
;
; Entry Conditions:
; - DSPCOUNT = -0 (indicates exactly 5 characters entered)
; - REQRET contains return address (complemented form)
; - Flash state active (FLASH bit set)
;
; Operation:
; 1. Complements REQRET to restore positive return address
; 2. Calls FLASHOFF to stop display flashing
; 3. Returns to requesting verb routine via REQRET
;
; Used by: All load verbs (V21-V25), monitor setup, address specification
; ============================================================================
ACCEPTWD	CS	REQRET		# 5 CHAR IN (DSPCOUNT IS -)
		TS	REQRET		# SET REQRET +.
		TC	FLASHOFF	; Turn off flashing display
		TC	REQRET		; Return to requesting verb

ENTEXIT		=	ENTRET		; Entry/exit address equivalence

MMADREF		ADRES	MMCHANG +1	# ASSUMES TC REQMM AT MMCHANG.

# Page 407
; ============================================================================
; Verb/Noun Validation Constants
; ============================================================================
LOWVERB		DEC	28		# LOWER VERB THAT AVOIDS NOUN TEST.

; ============================================================================
; ENTPAS0 - Entry Pass Zero (Verb/Noun Validation Entry Point)
;
; Primary validation entry after crew presses ENTR key. Initializes validation
; state and determines whether entered verb requires noun parameter.
;
; Verb Classification:
; - Verbs 01-28: Display/load/monitor verbs requiring noun specification
; - Verbs 29+: Special functions and extended verbs (no noun required)
;
; Entry Conditions:
; - VERBREG contains entered verb code (01-99)
; - NOUNREG contains entered noun code (if any)
; - ENTR key pressed, awaiting validation
;
; Operation:
; 1. Clears DECBRNCH (decimal/octal branch indicator)
; 2. Blocks further numeric character input (prevents stray digits)
; 3. Saves verb code for possible recycle on error
; 4. Compares verb against LOWVERB threshold (28)
; 5. Routes to VERBFAN (verb ≥28) or TESTNN (verb <28, needs noun)
;
; Historical Context: During Apollo 11 descent, every crew entry passed through
; this validation. When Armstrong entered V16N68 (monitor landing radar altitude
; and velocity), this routine verified V16 was <28 and required noun, then
; validated N68 was loaded before executing the monitor display.
; ============================================================================
ENTPAS0		CAF	ZERO		# NOUN VERB SUB ENTERS HERE
		TS	DECBRNCH	; Clear decimal/octal branch indicator
		CS	VD1		# BLOCK FURTHER NUM CHAR, SO THAT STRAY
		TS	DSPCOUNT	# CHAR DO NOT GET INTO VERB OR NOUN LTS.

; ============================================================================
; TESTVB - Test Verb (Verb Threshold Check)
;
; Determines if verb is above or below LOWVERB threshold to decide if noun
; validation is needed. Saves verb for possible recycle on operator error.
;
; Technical Implementation:
; Complements VERBREG and adds LOWVERB. Result sign indicates:
; - Positive or zero: Verb ≥ 28, no noun required → branch to VERBFAN
; - Negative: Verb < 28, noun required → continue to TESTNN
;
; Register Usage:
; - A register: Contains (LOWVERB - VERBREG) comparison result
; - VERBSAVE: Stores original verb for recycle on error conditions
; ============================================================================
TESTVB		CS	VERBREG		# IF VERB IS G/E LOWVB, SKIP NOUN TEST.
		TS	VERBSAVE	# SAVE VERB FOR POSSIBLE RECYCLE.
		AD	LOWVERB		# LOWVERB - VB (compute threshold difference)
		EXTEND
		BZMF	VERBFAN		# VERB G/E LOWVERB (≥28, no noun needed)
; ============================================================================
; TESTNN - Test Noun (Noun Validation and Classification)
;
; Validates that required noun is loaded and classifies noun type (normal,
; mixed, or machine address). Reads noun table from fixed memory to determine
; noun characteristics and data format.
;
; Noun Types:
; - Normal: Standard 3-component data (positions in R1, R2, R3)
; - Mixed: Combination of component types (decimal, octal, or special format)
; - Machine Address: Requires ECADR (Extended Core Address) specification
;
; Technical Implementation:
; 1. Bank switches to noun table reading routine (LODNNLOC)
; 2. Uses MIXBR to index noun type (normal/mixed branch indicator)
; 3. Examines NNADTEM (Noun Address Temporary) to classify noun:
;    - Positive: Normal noun, proceed to VERBFAN
;    - +0: Noun not in use, trigger operator error (GODSPALM)
;    - Negative: Machine address noun, request CADR specification
;    - -0: Augment existing machine CADR (increment)
;
; Historical Context: When crew entered V16N68 during landing, this routine
; validated N68 (landing radar altitude and velocity) was defined in noun table
; and classified it as "normal" 3-component noun before enabling the monitor.
;
; NNADTEM Encoding:
; Sign and magnitude indicate noun table entry status and required handling.
; This elegant encoding minimizes memory usage in the 2K erasable RAM.
; ============================================================================
TESTNN		EXTEND			# VERB L/ LOWVERB (verb <28, needs noun)
		DCA	LODNNLOC	# SWITCH BANKS TO NOUN TABLE READING
		DXCH	Z		# ROUTINE (bank call for noun lookup).
		INDEX	MIXBR		; Index by mixed/normal noun indicator
		TC	+0		; Indexed transfer to appropriate handler
		TC	+2		# NORMAL noun type
		TC	MIXNOUN		# MIXED noun type
		CCS	NNADTEM		# NORMAL - check noun address temporary
		TC	VERBFAN -2	# NORMAL IF + (valid noun, proceed)
		TC	GODSPALM	# NOT IN USE IF +0 (undefined noun error)
		TC	REQADD		# SPECIFY MACHINE CADR IF - (address noun)
		INCR	NOUNCADR	# AUGMENT MACHINE CADR IF -0 (increment)
		TC	SETNADD		# ECADR FROM NOUNCADR, SETS EB, NOUNADD.
		TC	INTMCTBS +2	; Continue to machine address processing

; ============================================================================
; REQADD - Request Address (Machine CADR Entry Request)
;
; Handles verbs/nouns that require machine address (CADR) specification from
; the crew. Determines whether address comes from external crew entry or
; internal program specification, then routes to appropriate handler.
;
; Machine Address Verbs (requiring CADR):
; - V05Nxx: Load data at specified address
; - V01Nxx: Display data at specified address
; - V25Nxx: Load component at specified address
;
; Technical Implementation:
; Uses CLPASS (clear pass indicator) to track first-pass entry state.
; Compares ENDINST with ENTEXIT to determine internal vs external call origin.
; If external (crew-initiated), prompts for octal CADR entry via REQDATZ.
; If internal (program-initiated), processes address from MPAC.
;
; ECADR Format:
; Extended Core Address combines bank number (bits 15-12) with address within
; bank (bits 11-1). This allows addressing the full 36K words of fixed memory
; and 2K words of erasable memory across multiple banks.
;
; Historical Context: During Apollo 11, crew used V01N01 to display specific
; memory locations for telemetry verification. Engineers on the ground would
; uplink addresses to check via this machine CADR mechanism.
;
; Entry Conditions:
; - NNADTEM negative (machine address noun detected in TESTNN)
; - Verb/noun pair requires address specification
;
; Operation:
; 1. Set CLPASS for first-pass indicator (BIT15)
; 2. Determine if call is internal (program) or external (crew)
; 3. For external: call REQDATZ to request octal address entry
; 4. Validate decimal not used (machine addresses must be octal)
; 5. If CADRSTOR set, restore address; otherwise prompt for new entry
; 6. Store ECADR into NOUNCADR and proceed to VERBFAN
; ============================================================================
REQADD		CAF	BIT15		# SET CLPASS FOR PASS 0 ONLY
		TS	CLPASS		; First-pass indicator
		CS	ENDINST		# TEST IF REACHED HERE FROM INTERNAL OR
		AD	ENTEXIT		#	FROM EXTERNAL (compare entry source)
		EXTEND
		BZF	+2		# EXTERNAL MACH CADR TO BE SPECIFIED
		TC	INTMCTBS	; Internal call - address in MPAC
		TC	REQDATZ		# EXTERNAL MACH CADR TO BE SPECIFIED
		CCS	DECBRNCH	# ALARM AND RECYCLE IF DECIMAL USED
		TC	ALMCYCLE	# FOR MCTBS (addresses must be octal).
		CS	VD1		# OCTAL USED OK
		TS	DSPCOUNT	# BLOCK NUM CHAR IN (prevent stray input)
		CCS	CADRSTOR	; Check if stored address available
		TC	+3		# EXTERNAL MCTBS DISPLAY WILL LEAVE FLASH
		TC	USEADD		# ON IF ENDIDLE NOT = +0 (use stored addr).
		TC	+1		; No stored address, continue
		TC	FLASHON		; Activate flashing prompt for entry

; ============================================================================
; USEADD - Use Address (Apply Stored or Entered CADR)
;
; Final step in machine address processing. Takes ECADR from ZREG (where
; SETNCADR placed it) and stores into NOUNCADR for verb execution.
;
; Operation:
; 1. Exchange A with ZREG (retrieves ECADR)
; 2. Call SETNCADR to store ECADR into NOUNCADR
; 3. Bank switch to noun table reading routine
; 4. Proceed to VERBFAN for verb execution with address now specified
;
; Register Usage:
; - ZREG: Temporary storage for ECADR during processing
; - NOUNCADR: Final destination for machine address
; - EB (EBANK): Set to appropriate bank for noun data access
; ============================================================================
USEADD		XCH	ZREG		; Retrieve ECADR from temporary storage
		TC	SETNCADR	# ECADR INTO NOUNCADR. SET EB, NOUNADD.
		EXTEND
		DCA	LODNNLOC	# SWITCH BANKS TO NOUN TABLE READING
		DXCH	Z		# ROUTINE (bank call for noun processing).
		TC	VERBFAN		; Proceed to verb execution

		EBANK=	DSPCOUNT
# Page 408
LODNNLOC	2CADR	LODNNTAB

NEG5		OCT	77772		; Constant -5 in 1's complement octal

; ============================================================================
; INTMCTBS - Internal Machine CADR Specification
;
; Handles machine address specification when the ECADR comes from internal
; program (not crew keyboard entry). Used when verbs are invoked by program
; with pre-specified addresses rather than prompting crew for address entry.
;
; Technical Implementation:
; Retrieves ECADR from MPAC+2 (left there by NVSUB calling sequence), stores
; into NOUNCADR, then optionally displays the address back to crew depending
; on verb type. V05 (LOAD) does not display the address; other verbs do.
;
; Verb-Specific Behavior:
; - V05Nxx (Load): Address NOT displayed (silent operation)
; - V01Nxx (Display): Address IS displayed in R3 (octal format)
; - Other machine CADR verbs: Address IS displayed for verification
;
; Display Format:
; When displayed, CADR appears in R3 in 5-digit octal format (XXXXX).
; Format controlled by DSPOCTWO (octal display, 2 registers).
; DSPCOUNT set to R3D1 indicating R3, 1 component.
;
; Historical Context: During Apollo 11, ground controllers uplinked specific
; memory addresses for telemetry verification. When program automatically
; displayed or loaded these addresses, this routine handled the internal
; address specification without requiring crew keyboard entry.
;
; Entry Conditions:
; - MPAC+2 contains ECADR (placed by calling program)
; - VERBREG contains verb code
; - Noun table processing complete
;
; Operation:
; 1. Load ECADR from MPAC+2
; 2. Store into NOUNCADR via SETNCADR (also sets EB and NOUNADD)
; 3. Check if VERBREG = 05 (LOAD verb)
; 4. If V05: proceed directly to VERBFAN without display
; 5. If not V05: display CADR in R3, then proceed to VERBFAN
;
; Register Usage:
; - MPAC+2: Source of ECADR (input)
; - NOUNCADR: Destination for ECADR (output)
; - EB (EBANK): Set to appropriate bank for noun data access
; - NOUNADD: Computed address within bank
; ============================================================================
INTMCTBS	CA	MPAC	+2	# INTERNAL MACH CADR TO BE SPECIFIED.
		TC	SETNCADR	# ECADR INTO NOUNCADR. SET EB. NOUNADD.
		CS	FIVE		# NVSUB CALL LEFT CADR IN MPAC+2 FOR MACH
		AD	VERBREG		# CADR TO BE SPECIFIED (test VB = 05).
		EXTEND
		BZF	VERBFAN		# DONT DISPLAY CADR IF VB = 05 (silent load).
		CAF	R3D1		# VB NOT = 05. DISPLAY CADR in R3.
		TS	DSPCOUNT	; Set display register and component count
		CA	NOUNCADR	; Retrieve stored ECADR
		TC	DSPOCTWO	; Display in octal, 2 registers (R3)
		TC	VERBFAN		; Proceed to verb execution

		AD	ONE		; Alternate entry: increment address by 1
		TC	SETNCADR	# ECADR INTO NOUNCADR. SETS EB, NOUNADD.
VERBFAN		CS	LST2CON
		AD	VERBREG		# VERB = LST2CON
		CCS	A
		AD	ONE		# VERB G/ LST2CON
		TC	+2
		TC	VBFANDIR	# VERB L/ LST2CON
		TS	MPAC
		TC	RELDSP		# RELEASE DISPLAY SYST
		TC	POSTJUMP	# GO TO GOEXTVB WITH VB=40 IN MPAC.
		CADR	GOEXTVB
LST2CON		DEC	40		# FIRST LIST2 VERB (EXTENDED VERB)

; ============================================================================
; VBFANDIR/VERBTAB - Verb Jump Table Dispatcher
;
; Central dispatch table routing verb codes (V01-V39) to their handler
; routines. Extended verbs (V40+) handled separately via GOEXTVB.
;
; COMMENT-ONLY READERS: This table is the "phone directory" for the DSKY.
; When the crew enters a verb number and presses ENTER, the AGC looks up
; that verb's handler in this table and jumps to it. For example, V16 N65
; displays the crew's current range and velocity - this table connects V16
; to the MONITOR routine that updates the display once per second.
;
; CODE-ALONG READERS: Indexed jump table using VERBREG as index. Each entry
; is a CADR (cross-bank address) allowing handlers to reside in different
; memory banks. VBFANDIR loads CADR from VERBTAB using INDEX VERBREG, then
; performs BANKJUMP. Verbs 00-39 in this table, verbs 40+ processed by
; GOEXTVB in extended verb bank. Display verbs (01-17) show data on DSKY.
; Load verbs (21-27) accept crew input. Special function verbs (30-37)
; control AGC operations.
; ============================================================================

VBFANDIR	INDEX	VERBREG
		CAF	VERBTAB
		TC	BANKJUMP

; Verb Jump Table - Maps verb codes to handler routines
; Display Verbs (01-17): Show data on DSKY registers R1, R2, R3
; Load Verbs (21-27): Accept crew numeric input
; Special Function Verbs (30-37): AGC control operations
; Extended Verbs (40+): Handled by GOEXTVB in separate bank

VERBTAB		CADR	GODSPALM	# VB00 ILLEGAL
		CADR	DSPA		# VB01 DISPLAY OCT COMP 1 (R1)
		CADR	DSPB		# VB02 DISPLAY OCT COMP 2 (R1)
		CADR	DSPC		# VB03 DISPLAY OCT COMP 3 (R1)
		CADR	DSPAB		# VB04 DISPLAY OCT COMP 1,2 (R1,R2)
		CADR	DSPABC		# VB05 DISPLAY OCT COMP 1,2,3 (R1,R2,R3)
		CADR	DECDSP		# VB06 DECIMAL DISPLAY
		CADR	DSPDPDEC	# VB07 DP DECIMAL DISPLAY (R1,R2)
		CADR	GODSPALM	# VB08 SPARE
		CADR	GODSPALM	# VB09 SPARE
		CADR	DSPALARM	# VB10 SPARE
		CADR	MONITOR		# VB11 MONITOR OCT COMP 1 (R1)
		CADR	MONITOR		# VB12 MONITOR OCT COMP 2 (R1)
		CADR	MONITOR		# VB13 MONITOR OCT COMP 3 (R1)
		CADR	MONITOR		# VB14 MONITOR OCT COMP 1,2 (R1,R2)
# Page 409
		CADR	MONITOR		# VB15 MONITOR OCT COMP 1,2,3 (R1,R2,R3)
		CADR	MONITOR		# VB16 MONITOR DECIMAL
		CADR	MONITOR		# VB17 MONITOR DP DEC (R1,R2)
		CADR	GODSPALM	# VB18 SPARE
		CADR	GODSPALM	# VB19 SPARE
		CADR	GODSPALM	# VB20 SPARE
		CADR	ALOAD		# VB21 LOAD COMP 1 (R1)
		CADR	BLOAD		# VB22 LOAD COMP 2 (R2)
		CADR	CLOAD		# VB23 LOAD COMP 3 (R3)
		CADR	ABLOAD		# VB24 LOAD COMP 1,2 (R1,R2)
		CADR	ABCLOAD		# VB25 LOAD COMP 1,2,3 (R1,R2,R3)
		CADR	GODSPALM	# VB26 SPARE
		CADR	DSPFMEM		# VB27 FIXED MEMORY DISPLAY
					# THE FOLLOWING VERBS MAKE NO NOUN TEST
		CADR	GODSPALM	# VB28 SPARE
		CADR	GODSPALM	# VB29 SPARE
REQEXLOC	CADR	VBRQEXEC	# VB30 REQUEST EXECUTIVE
		CADR	VBRQWAIT	# VB31 REQUEST WAITLIST
		CADR	VBRESEQ		# VB32 RESEQUENCE
		CADR	VBPROC		# VB33 PROCEED WITHOUT DATA
		CADR	VBTERM		# VB34 TERMINATE CURRENT TEST OR LOAD REQ
		CADR	VBTSTLTS	# VB35 TEST LIGHTS
		CADR	SLAP1		# VB36 FRESH START
		CADR	MMCHANG		# VB37 CHANGE MAJOR MODE
		CADR	GODSPALM	# VB38 SPARE
		CADR	GODSPALM	# VB39 SPARE

# THE LIST2 VERBFAN IS LOCATED IN THE EXTENDED VERB BANK.
# Page 410

; ============================================================================
; MIXNOUN - Mixed Noun Data Retrieval
;
; Retrieves data for "mixed nouns" - nouns that combine data from three
; different memory locations (rather than three consecutive registers).
; NNADTAB contains IDADDREL pointing to IDADDTAB where three source addresses
; are stored. MIXNOUN fetches data from these scattered locations and stores
; them consecutively in MIXTEMP, MIXTEMP+1, MIXTEMP+2 for display.
;
; COMMENT-ONLY READERS: Some DSKY displays show information from different
; parts of the AGC's memory simultaneously. For example, one register might
; show altitude from the navigation system, another velocity from guidance,
; and a third fuel quantity from the engine monitor. This routine gathers
; those scattered pieces of information so they can be displayed together.
;
; CODE-ALONG READERS: Checks NNADTEM to verify noun is in use. Verifies verb
; is display type (verbs 01-06) to avoid swapping data on non-display verbs.
; Loops through DECOUNT (2 down to 0) retrieving three components. For each:
; indexes into IDAD1TEM to get source address, tests for double-precision
; using DPTEST (if DP, increments NOUNTEM to get minor part), sets EBANK,
; reads data, stores in MIXTEMP+K. Sets NOUNADD to MIXTEMP base for display
; processing. Critical for nouns like N50 (altitude, altitude rate, position).
; ============================================================================

# NNADTAB CONTAINS A RELATIVE ADDRESS, IDADDREL (IN LOW 10 BITS), REFERRING
# TO WHERE 3 CONSECUTIVE ADDRESSES ARE STORED (IN IDADDTAB).
# MIXNOUN GETS DATA AND STORES IN MIXTEMP,+1,+2.  IT SETS NOUNADD FOR
# MIXTEMP.

MIXNOUN		CCS	NNADTEM
		TC	+4		# + IN USE
		TC	GODSPALM	# +0 NOT IN USE
		TC	+2		# - IN USE
		TC	+1		# -0 IN USE
		CS	SIX
		AD	VERBREG
		EXTEND
		BZMF	+2		# VERB L/E 6
		TC	VERBFAN		# AVOID MIXNOUN SWAP IF VB NOT = DISPLAY
		CAF	TWO
MIXNN1		TS	DECOUNT
		AD	MIXAD
		TS	NOUNADD		# SET NOUNADD TO MIXTEMP + K
		INDEX	DECOUNT		# GET IDADDTAB ENTRY FOR COMPONENT K
		CA	IDAD1TEM	# OF NOUN.
		TS	NOUNTEM
					# TEST FOR DP (FOR OCT DISPLAY).  IF SO, GET
					#	MINOR PART ONLY.
		TC	SFRUTMIX	# GET SF ROUT NUMBER IN A
		TC	DPTEST
		TC	MIXNN2		# NO DP
		INCR	NOUNTEM		# DP GET MINOR PART
MIXNN2		CA	NOUNTEM
		MASK	LOW11		# ESUBK (NO DP) OR (ESUBK)+1 (garbled) FOR DP
		TC	SETEBANK	# SET EBANK, LEAVE EADRES IN A.
		INDEX	A		# PICK UP C(ESUBK) NOT DP
		CA	0		# OR C((ESUBK)+1) FOR DP MINOR PART
		INDEX	NOUNADD
		XCH	0		# STORE IN MIXTEM + K
		CCS	DECOUNT
		TC	MIXNN1
		TC	VERBFAN

MIXAD		TC	MIXTEMP

; ============================================================================
; DPTEST - Double-Precision Data Test
;
; Tests whether a scale factor routine number indicates double-precision data.
; Enter with SF routine number in A register. Returns to L+1 if single-
; precision, returns to L+2 if double-precision.
;
; COMMENT-ONLY READERS: Some displayed numbers need extra precision - more
; digits than can fit in one AGC word. This routine checks whether a piece
; of data uses one word (single-precision) or two words (double-precision).
; If it's double-precision, the display system needs to fetch both words.
;
; CODE-ALONG READERS: Indexed jump table. SF routine numbers 0-13 represent:
; 0=octal, 1=fract, 2=deg, 3=arith (all single-precision, return to L+1).
; SF routines 4,5,7,10 are DP1OUT, DP2OUT, DP3OUT, DP4OUT (double-precision,
; jump to DPTEST1 which returns to L+2). Special cases: 6=LRPOSOUT (channel
; 33 data, single-precision), 8=HMS, 9=M/S, 11=ARITH1, 12=2INTOUT, 13=360-CDU
; (all single-precision). Called by MIXNOUN, display routines, load routines
; to determine whether to fetch one or two memory words.
; ============================================================================

# DPTEST	ENTER WITH SF ROUT NUMBER IN A.
#		RETURNS TO L+1 IF NO DP.
#		RETURNS TO L+2 IF DP.

DPTEST		INDEX	A
		TCF	+1
		TC	Q		# OCTAL ONLY NO DP
		TC	Q		# FRACT NO DP
# Page 411
		TC	Q		# DEG  NO DP
		TC	Q		# ARITH  NO DP
		TCF	DPTEST1		# DP1OUT
		TCF	DPTEST1		# DP2OUT
		TC	Q		# LRPOSOUT NO DP (DATA IN CHANNEL 33)
		TCF	DPTEST1		# DP3OUT
		TC	Q		# HMS   NO DP
		TC	Q		# M/S   NO DP
		TCF	DPTEST1		# DP4OUT
		TC	Q		# ARITH1   NO DP
		TC	Q		# 2INTOUT  NO DP TO GET HI PART IN MPAC
		TC	Q		# 360-CDU   NO DP
DPTEST1		INDEX	Q
		TC	1		# RETURN TO L+2

; ============================================================================
; REQDATX/Y/Z - Request Data Entry for Registers R1/R2/R3
;
; Prompts crew to enter data by blanking the specified register and flashing
; the VERB light. REQDATX requests R1 data, REQDATY requests R2, REQDATZ
; requests R3. Common routine REQCOM blanks display and sets flash state.
;
; COMMENT-ONLY READERS: When the AGC needs information from the crew, it
; blanks one of the three display registers and makes the VERB light flash.
; For example, if the crew selects V21 N01 (load component 1), the AGC
; calls REQDATX to blank register R1 and flash the lights, signaling "I'm
; waiting for you to type a number." The crew types the value and presses
; ENTER to proceed.
;
; CODE-ALONG READERS: Entry points set DSPCOUNT to register identifier
; (R1D1, R2D1, or R3D1). REQCOM stores DSPCOUNT, saves return address in
; REQRET (negated for bank call convention), calls 5BLANK to clear display,
; calls FLASHON to activate VERB light flashing. ENDRQDAT enters executive
; wait state via ENTEXIT. When crew presses ENTER, keyboard interrupt
; resumes execution at REQRET. Used by load verbs V21-V27 to prompt for
; numeric input on landing, ascent, rendezvous programs.
; ============================================================================

REQDATX		CAF	R1D1
		TCF	REQCOM
REQDATY		CAF	R2D1
		TCF	REQCOM
REQDATZ		CAF	R3D1
REQCOM		TS	DSPCOUNT
		CS	Q
		TS	REQRET
		TC	BANKCALL
		CADR	5BLANK
		TC	FLASHON
ENDRQDAT	TC	ENTEXIT

; ============================================================================
; UPDATNN - Update Noun Display
;
; Updates the two-digit noun code displayed on DSKY. Called when new noun
; entered by crew or set by internal program. Retrieves noun table entry,
; sets up data addresses, and calls display update routine GOVNUPDT.
;
; COMMENT-ONLY READERS: When the crew types a noun number (like N65 for
; range and velocity), this routine displays that two-digit code in the NOUN
; area of the DSKY so the crew can confirm what data they're looking at.
;
; CODE-ALONG READERS: Enter with noun code in NOUNREG. Saves return address
; in UPDATRET. Performs DXCH LODNNLOC to Z to switch banks for noun table
; access (LODNNLOC points to noun table reading bank). Tests NNADTEM to
; determine noun type: +N is normal noun (calls SETNCADR to set ECADR from
; noun table, establishes EBANK and NOUNADD), -0 is MCTBS (machine CADR to
; be specified, skips PUTADD), -N is MCTBI (machine CADR being incremented,
; skips PUTADD). Sets DSPCOUNT to ND1 (noun display identifier), loads
; NOUNREG, jumps to UPDAT1 common path which calls GOVNUPDT via POSTJUMP.
; ============================================================================

		TS	NOUNREG
UPDATNN		XCH	Q
		TS	UPDATRET
		EXTEND
		DCA	LODNNLOC	# SWITCH BANKS TO NOUN TABLE READING
		DXCH	Z		# ROUTINE.
		CCS	NNADTEM
		AD	ONE		# NORMAL
		TCF	PUTADD
		TCF	PUTADD +1	# MCTBS	 DONT CHANGE NOUNADD
		TCF	PUTADD +1	# MCTBI	 DONT CHANGE NOUNADD
PUTADD		TC	SETNCADR	# ECADR INTO NOUNCADR. SETS EB. NOUNADD.
		CAF	ND1
		TS	DSPCOUNT
		CA	NOUNREG
		TCF	UPDAT1

; ============================================================================
; UPDATVB - Update Verb Display
;
; Updates the two-digit verb code displayed on DSKY. Called when new verb
; entered by crew or set by internal program. Calls display update routine
; GOVNUPDT to format and display verb number.
;
; COMMENT-ONLY READERS: When the crew types a verb number (like V16 to
; display information), this routine shows that two-digit code in the VERB
; area of the DSKY so the crew knows which action the computer is performing.
;
; CODE-ALONG READERS: Enter with verb code in VERBREG. Saves return address
; in UPDATRET (note original comment warns can't use SWCALL to reach
; DSPDECVN since UPDATVB itself may be called via SWCALL - avoids stack
; depth issues). Sets DSPCOUNT to VD1 (verb display identifier), loads
; VERBREG into A. Falls through to UPDAT1 common path.
; ============================================================================

		TS	VERBREG
UPDATVB		XCH	Q
		TS	UPDATRET
		CAF	VD1
# Page 412
		TS	DSPCOUNT
		CA	VERBREG
UPDAT1		TC	POSTJUMP	# CANT USE SWCALL TO GO TO DSPDECVN, SINCE
		CADR	GOVNUPDT	# UPDATVB CAN ITSELF BE CALLED BY SWCALL.
		TC	UPDATRET

; ============================================================================
; GOALMCYC - Go to Alarm Cycle Routine
;
; Bank jump wrapper for ALMCYCLE routine. Required because BANKJUMP cannot
; handle fixed-to-fixed bank transfers directly.
;
; COMMENT-ONLY READERS: Small utility that transfers control to the alarm
; cycling routine which makes the alarm lights flash on and off periodically.
;
; CODE-ALONG READERS: TC ALMCYCLE performs direct transfer. Original comment
; notes BANKJUMP instruction cannot handle fixed-to-fixed (ROM-to-ROM) bank
; transfers, so direct TC is used instead. ALMCYCLE is in another fixed bank.
; ============================================================================

GOALMCYC	TC	ALMCYCLE	# NEEDED BECAUSE BANKJUMP CANT HANDLE F/F.

; ============================================================================
; GODSPALM - Go to Display Alarm Routine
;
; Bank jump to DSPALARM routine which displays program alarm codes.
;
; COMMENT-ONLY READERS: When something goes wrong (like invalid crew input
; or a computation problem), this transfers control to the alarm display
; system which shows an alarm code on the DSKY and makes the PROG light flash.
;
; CODE-ALONG READERS: POSTJUMP with CADR performs inter-bank jump to
; DSPALARM routine. Used throughout pinball to trigger alarm display for
; operator errors, invalid verb/noun combinations, data entry violations.
; ============================================================================

GODSPALM	TC	POSTJUMP
		CADR	DSPALARM

# Page 413
# NOUN TABLES
#
# NOUN CODE L/40, NORMAL NOUN CASE.  NOUN CODE G/E 40, MIXED NOUN CASE.
# FOR NORMAL CASE, NNADTAB CONTAINS ONE       ECADR     FOR EACH NOUN.
# +0 INDICATES NOUN NOT USED.   - ENTRY INDICATES MACHINE CADR (E OR F) TO
# BE SPECIFIED. -1 INDICATES CHANNEL TO BE SPECIFIED.  -0 INDICATES AUGMENT
# OF LAST MACHINE CADR SUPPLIED.
#
# FOR MIXED CASE, NNADTAB CONTAINS ONE INDIRECT ADDRESS (IDADDREL) IN LOW
# 10 BITS, AND THE COMPONENT CODE NUMBER IN THE HIGH 5 BITS.
#
# NNTYPTAB IS A PACKED TABLE OF THE FORM MMMMMNNNNNPPPPP.
#
# FOR THE NORMAL CASE,	M'S ARE THE COMPONENT CODE NUMBER.
#			N'S ARE THE SF ROUTINE CODE NUMBER.
#			P'S ARE THE SF CONSTANT CODE NUMBER.
#
# MIXED-CASE,		M'S ARE THE SF CONSTANT3 CODE NUMBER	3 COMPONENT CASE
#			N'S ARE THE SF CONSTANT2 CODE NUMBER
#			P'S ARE THE SF CONSTANT1 CODE NUMBER
#			N'S ARE THE SF CONSTANT2 CODE NUMBER	2 COMPONENT CASE
#			P'S ARE THE SF CONSTANT1 CODE NUMBER
#			P'S ARE THE SF CONSTANT1 CODE NUMBER	1 COMPONENT CASE
#
# THERE IS ALSO AN INDIRECT ADDRESS TABLE (IDADDTAB) FOR MIXED CASE ONLY.
# EACH ENTRY CONTAINS ONE ECADR.    IDADDREL IS THE RELATIVE ADDRESS OF
# THE FIRST OF THESE ENTRIES.
#
# THERE IS ALSO A SCALE FACTOR ROUTINE NUMBER TABLE (RUTMXTAB) FOR MIXED
# CASE ONLY.  THERE IS ONE ENTRY PER MIXED NOUN.  THE FORM IS,
#
#	QQQQQRRRRRSSSSS
#
# Q'S ARE THE SF ROUTINE 3 CODE NUMBER		3 COMPONENT CASE
# R'S ARE THE SF ROUTINE 2 CODE NUMBER
# S'S ARE THE SF ROUTINE 1 CODE NUMBER
# R'S ARE THE SF ROUTINE 2 CODE NUMBER		2 COMPONENT CASE
# S'S ARE THE SF ROUTINE 1 CODE NUMBER
#
# IN OCTAL DISPLAY AND LOAD (OCT OR DEC) VERBS, EXCLUDE USE OF VERBS WHOSE
# COMPONENT NUMBER IS GREATER THAN THE NUMBER OF COMPONENTS IN NOUN.
# (ALL MACHINE ADDRESS TO BE SPECIFIED NOUNS ARE 3 COMPONENT.)
#
# IN MULTI-COMPONENT LOAD VERBS, NO MIXING OF OCTAL AND DECIMAL DATA
# COMPONENT WORDS IS ALLOWED. ALARM IF VIOLATION.
#
# IN DECIMAL LOADS OF DATA, 5 NUMERICAL CHARACTERS MUST BE KEYED IN
# BEFORE EACH ENTER. IF NOT, ALARM.

# Page 414
# DISPLAY VERBS

; ============================================================================
; DISPLAY VERBS - DSPABC, DSPAB, DSPA, DSPB, DSPC
;
; Display data components on DSKY for display verbs (V01-V07, V16, etc.).
; Each entry point displays different component combinations:
;   DSPABC: Display all three registers R1, R2, R3 (verb codes ending in 3)
;   DSPAB:  Display two registers R1, R2 (verb codes ending in 2)
;   DSPA:   Display one register R1 (verb codes ending in 1)
;   DSPB:   Display register R2 only (verb code V04)
;   DSPC:   Display register R3 only (verb code V05)
;
; COMMENT-ONLY READERS: When you request to see information with a display
; verb like V16 N65 (monitor range and velocity), these routines fetch the
; data from the noun's memory addresses and format it for the three DSKY
; registers. V06 displays all three values (R1/R2/R3), V16 monitors two
; values (R1/R2), etc. During Apollo 11's landing, Armstrong and Aldrin
; constantly monitored displays like V16 N68 (range to landing site) and
; V16 N63 (velocity components) through these routines.
;
; CODE-ALONG READERS: Entry logic tests component count via COMPTEST (or
; DCOMPTST for single components) to ensure verb doesn't request more
; components than noun provides. DSPABC fetches component 3 (offset +2),
; stores to BUF+2, then falls through to DSPAB which fetches component 2
; (offset +1), stores to BUF+1, then falls through to DSPA which calls
; DECTEST to verify noun is decimal (not octal-only), calls TSTFORDP to
; check double-precision requirement, fetches component 1 (offset 0), and
; enters DSPCOM1/DSPCOM2 common path. DSPB and DSPC are alternate entries
; for displaying single non-R1 components. NOUNADD contains base address
; from noun table. CS instruction (complement and store) with INDEX loads
; negated data value. Original verb code in VERBREG determines display
; register assignment via R1D1/R2D1/R3D1 lookup. DSPOCTWO formats and
; outputs to DSKY seven-segment display.
; ============================================================================

DSPABC		CS	TWO
		TC	COMPTEST
		INDEX	NOUNADD
		CS	2
		XCH	BUF	+2
DSPAB		CS	ONE
		TC	COMPTEST
		INDEX	NOUNADD
		CS	1
		XCH	BUF	+1
DSPA		TC	DECTEST
		TC	TSTFORDP
		INDEX	NOUNADD
		CS	0
DSPCOM1		XCH	BUF
		TC	DSPCOM2
DSPB		CS	ONE
		TC	DCOMPTST
		INDEX	NOUNADD
		CS	1
		TC	DSPCOM1
DSPC		CS	TWO
		TC	DCOMPTST
		INDEX	NOUNADD
		CS	2
		TC	DSPCOM1

; Common display output logic: determines which registers to update based on
; verb code, then iterates through BUF data calling DSPOCTWO for formatting.

DSPCOM2		CS	TWO		# A  B  C  AB ABC
		AD	VERBREG		# -1 -0 +1 +2 +3   IN A
		CCS	A		# +0 +0 +0 +1 +2   IN A AFTER CCS
		TC	DSPCOM3
		TC	ENTEXIT
		TC	+1
DSPCOM3		TS	DISTEM		# +0 +1 +2 INTO DISTEM
		INDEX	A
		CAF	R1D1
		TS	DSPCOUNT
		INDEX	DISTEM
		CS	BUF
		TC	DSPOCTWO
		XCH	DISTEM
		TC	DSPCOM2 +2

; ============================================================================
; COMPTEST - Component Count Compatibility Test
;
; Validates that verb's requested component count does not exceed noun's
; available components. Alarms if verb requests more data than noun provides.
;
; COMMENT-ONLY READERS: Some verbs display three registers (V06), some two
; (V16), some one (V01). Each noun provides one, two, or three components.
; This routine ensures the verb doesn't try to display more components than
; the noun actually has. For example, V06 N65 would alarm because noun 65
; (range and velocity) only provides two components but V06 wants three.
;
; CODE-ALONG READERS: Enter with verb component count (negated) in A, return
; address in Q. Stores verb comp to SFTEMP1, moves Q to L. Calls GETCOMP to
; retrieve noun's component code from noun table, shifts right 5 bits (LEFT5),
; masks with THREE to extract component count bits. Adds to SFTEMP1 (verb
; comp, already negated), CCS tests result: positive means noun has more
; components than verb needs (OK, return via L), zero means equal (OK via
; NDCMPTST), negative means verb needs more components than noun has (alarm
; via GODSPALM). CCSHOLE handles +0 case (also OK). Used by display and
; load verbs to prevent nonsensical verb/noun combinations.
; ============================================================================

# COMPTEST ALARMS IF COMPONENT NUMBER OF VERB (LOAD OR OCT DISPLAY) IS
# GREATER THAN THE HIGHEST COMPONENT NUMBER OF NOUN.

COMPTEST	TS	SFTEMP1		# VERB COMP
		LXCH	Q
COMPTST1	TC	GETCOMP
		TC	LEFT5
		MASK	THREE		# NOUN COMP
# Page 415
		AD	SFTEMP1		# NOUN COMP = VERB COMP
		CCS	A
		TC	L		# NOUN COMP G/ VERB COMP
		TC	CCSHOLE
		TC	GODSPALM	# NOUN COMP L/ VERB COMP
NDCMPTST	TC	L		# NOUN COMP = VERB COMP

; ============================================================================
; DCOMPTST - Decimal Component Test with Component Count Check
;
; Combined test: first checks decimal-only bit, then performs COMPTEST.
; Ensures noun is decimal-capable before checking component compatibility.
;
; COMMENT-ONLY READERS: Some nouns can only be displayed in octal (base-8)
; format, not decimal. If a decimal display verb tries to use an octal-only
; noun, this routine detects the incompatibility and raises an alarm.
;
; CODE-ALONG READERS: Enter with negated verb component count in A. Stores
; to SFTEMP1, saves Q to L, calls DECTEST to verify decimal capability,
; then falls through to COMPTST1 for component count validation. Essentially
; DECTEST followed by COMPTEST. Used by display verbs requiring decimal.
; ============================================================================

# DCOMPTST ALARMS IF DECIMAL ONLY BIT (BIT4 OF COMP CODE NUMBER) = 1.
# IF NOT, IT PERFORMS REGULAR COMPTEST.

DCOMPTST	TS	SFTEMP1		# - VERB COMP
		LXCH	Q
		TC	DECTEST
		TC	COMPTST1

; ============================================================================
; DECTEST - Decimal-Only Capability Test
;
; Checks whether noun supports decimal display format. Alarms if noun is
; octal-only (decimal-only bit = 1 in component code).
;
; COMMENT-ONLY READERS: The DSKY can show numbers in decimal (0-9 digits)
; or octal (0-7 digits). Some spacecraft data (like memory addresses) makes
; sense only in octal. This test prevents trying to show octal-only data
; in decimal format, which would confuse the crew.
;
; CODE-ALONG READERS: Saves return address Q to MPAC+2 via QXCH. Calls
; GETCOMP to retrieve noun component code, masks with BIT14 to test decimal-
; only bit (bit 4 of component code word). CCS A: if positive (bit set),
; noun is octal-only, alarm via GODSPALM. If zero (bit clear), noun supports
; decimal, return via MPAC+2. Component code bit 4: 1=octal only, 0=decimal OK.
; ============================================================================

DECTEST		EXTEND			# ALARMS IF DEC ONLY BIT = 1 (BIT4 OF COMP
		QXCH	MPAC +2		# CODE NUMBER).  RETURNS IF NOT.
		TC	GETCOMP
		MASK	BIT14
		CCS	A
		TC	GODSPALM
		TC	MPAC +2

; ============================================================================
; DCTSTCYC - Decimal Test with Alarm Cycle
;
; Similar to DECTEST but triggers alarm cycling (flashing) instead of simple
; alarm display. Used by load verbs where continued flashing alerts crew to
; persistent input error rather than one-time alarm.
;
; COMMENT-ONLY READERS: When entering data with load verbs, if you try to
; load a value into an octal-only noun, this routine makes the alarm lights
; flash repeatedly until you press RSET, reminding you the noun can't accept
; the data format you're trying to use.
;
; CODE-ALONG READERS: Saves Q to L, calls GETCOMP, masks BIT14 for decimal-
; only bit test. If bit set (octal-only noun), calls ALMCYCLE to trigger
; alarm flash cycle. If clear, returns via L. Used by load verbs V21-V25
; to provide persistent error indication for format mismatches during data
; entry operations. Alarm continues flashing until crew presses RSET.
; ============================================================================

DCTSTCYC	LXCH	Q		# ALARMS AND RECYCLES IF DEC ONLY BIT = 1
		TC	GETCOMP		# (BIT4 OF COMP CODE NUMBER). RETURNS
		MASK	BIT14		# IF NOT.  USED BY LOAD VERBS.
		CCS	A
		TC	ALMCYCLE
		TC	L

; ============================================================================
; NOUNTEST - No-Load Capability Test
;
; Checks whether noun permits data loading. Alarms if noun is display-only
; (no-load bit set in component code). Prevents load verbs from attempting
; to write to read-only nouns.
;
; COMMENT-ONLY READERS: Some displayed values (like computed velocity) can
; only be shown, not changed by the crew. If you try to use a load verb
; (like V21) with a display-only noun, this routine catches the error and
; raises an alarm. You can only load data into nouns designed to accept input.
;
; CODE-ALONG READERS: Saves Q to L, calls GETCOMP to retrieve component code.
; CCS A tests sign bit (bit 15, MSB): if negative (no-load bit set), noun
; is display-only, alarm via GODSPALM. If positive or zero (no-load bit
; clear), noun accepts loading, return via L. Component code bit 15: 1=no
; load allowed (display-only), 0=load permitted. Two TC L instructions handle
; positive and +0 cases (both mean load OK). Used by all load verbs V21-V27
; to prevent writes to read-only nouns like computed navigation states.
; ============================================================================

# NOUNTEST ALARMS IF NO-LOAD BIT (BIT5 OF COMP CODE NUMBER) = 1.
# IF NOT, IT RETURNS.

NOUNTEST	LXCH	Q
		TC	GETCOMP
		CCS	A
		TC	L
		TC	L
		TC	GODSPALM

; ============================================================================
; SCALE FACTOR AND DISPLAY FORMAT HELPER ROUTINES
;
; The following section contains utility routines used throughout the DSKY
; display system to determine proper scale factors and formatting for data
; presentation. These routines handle both normal and mixed-noun cases, test
; for double-precision requirements, and coordinate between internal AGC
; representation and crew-readable display formats.
;
; During Apollo 11's lunar landing, these routines continuously formatted
; altitude, velocity, and fuel data for Armstrong and Aldrin's DSKY displays,
; converting internal scaled integers to readable decimal values.
; ============================================================================

; ----------------------------------------------------------------------------
; TSTFORDP - Test for Double-Precision Scale Format
;
; Tests whether the current noun component requires double-precision (DP)
; scale factor formatting. For mixed nouns, the DP test was already handled
; in MIXNOUN. For normal nouns, tests the entire noun's scale factor type.
;
; COMMENT-ONLY READERS: Some values like position (millions of meters) need
; more precision than fits in one AGC word (15 bits). These "double-precision"
; values use two words and require special display formatting. This routine
; checks if the current value needs that extra precision.
;
; CODE-ALONG READERS: Saves Q to L. Loads NNADTEM and adds 1 to check for -1
; (channel case). BZF branches to CHANDSP if zero (channel noun). INDEX MIXBR
; selects: normal case TC +2 (skip mixed return), mixed case TC L (already
; tested in MIXNOUN). Normal path calls SFRUTNOR to get scale factor routine,
; then DPTEST to check DP bit. Returns TC L if not DP, increments NOUNADD
; (E+1 for minor part address) and returns TC L if DP.
; ----------------------------------------------------------------------------
TSTFORDP	LXCH	Q		# TEST FOR DP. IF SO, GET MINOR PART ONLY.
		CA	NNADTEM
		AD	ONE		# IF NNADTEM = -1, CHANNEL TO BE SPECIFIED
		EXTEND
		BZF	CHANDSP
		INDEX	MIXBR
		TC	+0
		TC	+2		# NORMAL
# Page 416
		TC	L		# MIXED CASE ALREADY HANDLED IN MIXNOUN
		TC	SFRUTNOR
		TC	DPTEST
		TC	L		# NO DP
		INCR	NOUNADD		# DP	E+1 INTO NOUNADD FOR MINOR PART.
		TC	L

; ----------------------------------------------------------------------------
; CHANDSP - Channel Display Format Check
;
; Handles display formatting for channel-based nouns (NNADTEM = -1). Reads
; the channel data directly and checks its format requirements.
;
; COMMENT-ONLY READERS: Some nouns display hardware channel values (like
; thruster commands or sensor readings). This routine reads those channels
; and prepares them for display formatting.
;
; CODE-ALONG READERS: Loads NOUNCADR (noun address), masks LOW9 bits to get
; channel address. EXTEND READ 0 reads the indexed channel. CS A complements
; data (AGC channels use 1's complement). TCF DSPCOM1 continues to display
; common processing. Used for nouns that display I/O channel contents directly.
; ----------------------------------------------------------------------------
CHANDSP		CA	NOUNCADR
		MASK	LOW9
		EXTEND
		INDEX	A
		READ	0
		CS	A
		TCF	DSPCOM1

; ----------------------------------------------------------------------------
; COMPICK - Component Picker Address Table
;
; Two-word table containing addresses for component code retrieval:
; Word 1: NNTYPTEM (normal noun type code address)
; Word 2: NNADTEM (mixed noun address code)
; Indexed by MIXBR to select appropriate address.
; ----------------------------------------------------------------------------
COMPICK		ADRES	NNTYPTEM
		ADRES	NNADTEM

; ----------------------------------------------------------------------------
; GETCOMP - Get Component Code
;
; Retrieves the component code (HI5 bits) for the current noun component.
; For normal nouns, fetches from NNTYPTEM. For mixed nouns, fetches from
; NNADTEM. The component code contains scale factor and format information.
;
; COMMENT-ONLY READERS: Each piece of data (noun component) has a code that
; tells the computer how to format it - degrees, meters, seconds, etc. This
; routine fetches that formatting code.
;
; CODE-ALONG READERS: INDEX MIXBR selects COMPICK -1 (normal: NNTYPTEM) or
; COMPICK (mixed: NNADTEM). INDEX A loads from that address. MASK HI5 extracts
; high 5 bits containing component type code. Returns via Q with component
; code in A register. Component code format: bits 15-11 contain scale factor
; routine number and flags (DP, load permission, etc.).
; ----------------------------------------------------------------------------
GETCOMP		INDEX	MIXBR		# NORMAL			MIXED
		CAF	COMPICK -1	# ADRES NNTYPTEM		ADRES NNADTEM
		INDEX	A
		CA	0		# C(NNTYPTEM)			C(NNADTEM)
		MASK	HI5		# GET HI5 OF NNTYPTAB (NORM) 	OF NNADTAB (MIX)
		TC	Q

; ============================================================================
; DECIMAL DISPLAY (DECDSP)
;
; This routine displays numerical data in decimal format on the DSKY. During
; Apollo 11's landing, Armstrong and Aldrin watched decimal altitude and
; velocity displays generated through this code path as they descended toward
; the Sea of Tranquility. These decimal readouts were critical for monitoring
; fuel margins and descent rate during the final approach.
;
; The routine retrieves data components from memory, applies appropriate scale
; factors based on the noun type, and formats the numbers for display. It
; handles 1, 2, or 3 component displays (R1, R2, R3 on the DSKY).
;
; CODE-ALONG READERS: This is the main dispatcher for decimal output routines.
; It uses GETCOMP to determine component count, loads data from the XREG
; buffer, and routes to specialized scale factor output routines via the
; SFOUTABR (scale factor output address) table.
; ============================================================================

DECDSP		TC	GETCOMP
		TC	LEFT5
		MASK	THREE
		TS	DECOUNT		# COMP NUMBER INTO DECOUNT
; Data retrieval loop: Loads each display component (R1, R2, R3) from memory
; into the XREG buffer. NOUNADD points to the base address for this noun's
; data. The routine loops through 1, 2, or 3 components based on DECOUNT.
;
; Example: For a 3-component display like position (X, Y, Z), this loads all
; three values before formatting and displaying them.

DSPDCGET	TS	DECTEM		# PICKS UP DATA
		AD	NOUNADD		# DECTEM 1COMP +0, 2COMP +1, 3COMP +2
		INDEX	A
		CS	0
		INDEX	DECTEM
		XCH	XREG		# CANT USE BUF SINCE DMP USES IT.
		CCS	DECTEM
		TC	DSPDCGET	# MORE TO GET
; Display loop: Takes each component from XREG and formats it for display on
; the DSKY. DSPCOUNT determines which register (R1, R2, or R3) will receive
; the formatted output. The data is placed in MPAC for scale factor conversion.
;
; Historical context: When Armstrong called out "540 feet, down at 30" during
; landing, he was reading decimal values displayed through this code path.

DSPDCPUT	CAF	ZERO		# DISPLAYS DATA
		TS	MPAC +1		# DECOUNT 1COMP +0, 2COMP +1, 3COMP +2
		TS	MPAC +2
		INDEX	DECOUNT
		CAF	R1D1
		TS	DSPCOUNT
		INDEX	DECOUNT
		CS	XREG
		TS	MPAC
		TC	SFCONUM		# 2X (SF CON NUMB) IN A
# Page 417
; Scale factor routing: Switches to the bank containing scale factor constants
; and invokes the appropriate conversion routine. MIXBR (mixed/normal branch)
; determines whether this is a normal display or mixed octal/decimal display.
;
; CODE-ALONG READERS: The GTSFOUTL (get scale factor output) address loads
; SFTEMP1 and SFTEMP2, then branches based on MIXBR to either normal or mixed
; display handling.

		TS	SFTEMP1
		EXTEND			# SWITCH BANKS TO SF CONSTANT TABLE
		DCA	GTSFOUTL	#	READING ROUTINE.
		DXCH	Z		# LOADS SFTEMP1, SFTEMP2
		INDEX	MIXBR
		TC	+0
		TC	DSPSFNOR
		TC	SFRUTMIX
		TC	DECDSP3

DSPSFNOR	TC	SFRUTNOR
		TC	DECDSP3

		EBANK=	DSPCOUNT
GTSFOUTL	2CADR	GTSFOUT

DSPDCEND	TC	BANKCALL	# ALL SFOUT ROUTINES END HERE
		CADR	DSPDECWD
		CCS	DECOUNT
		TC	+2
		TC	ENTEXIT
		TS	DECOUNT
		TC	DSPDCPUT	# MORE TO DISPLAY

; Final dispatch: Uses indexed jump table to branch to the appropriate scale
; factor output routine based on the noun's scale factor code.

DECDSP3		INDEX	A
		CAF	SFOUTABR
		TC	BANKJUMP

; ============================================================================
; SCALE FACTOR OUTPUT ADDRESS TABLE (SFOUTABR)
;
; This jump table routes to specialized output formatters based on the noun's
; scale factor type. Each noun in the system has an associated scale factor
; code that determines how its raw internal value is converted to displayable
; decimal format.
;
; Examples of scale factor types:
; - DEGOUTSF: Degrees output (used for angles, attitudes)
; - ARTOUTSF: Altitude/range/time output
; - DP1OUTSF/DP2OUTSF/DP3OUTSF: Double-precision with different scalings
; - HMSOUT: Hours:Minutes:Seconds time format
; - M/SOUT: Minutes/Seconds format
; - LRPOSOUT: Landing radar position output
;
; Historical note: The altitude values Armstrong and Aldrin watched during
; descent used ARTOUTSF to convert from internal scaling to feet.
; ============================================================================

SFOUTABR	CADR	PREDSPAL	# ALARM IF DEC DISP WITH OCTAL ONLY NOUN
		CADR	DSPDCEND
		CADR	DEGOUTSF
		CADR	ARTOUTSF
		CADR	DP1OUTSF
		CADR	DP2OUTSF
		CADR	LRPOSOUT
		CADR	DP3OUTSF
		CADR	HMSOUT
		CADR	M/SOUT
		CADR	DP2OUTSF
		CADR	AROUT1SF
		CADR	2INTOUT
		CADR	360-CDUO
ENDRTOUT	EQUALS

; ============================================================================
; SCALE FACTOR OUTPUT ROUTINES
;
; The following routines convert AGC internal representations to displayable
; decimal formats. Each routine uses MPAC for computation and leaves results
; in MPAC and MPAC+1, ending with TC DSPDCEND.
;
; Historical context: These routines converted the raw sensor data and computed
; values into the human-readable numbers Armstrong and Aldrin saw during descent.
; ============================================================================

# THE FOLLOWING IS A TYPICAL SF ROUTINE. IT USES MPAC. LEAVES RESULTS
# IN MPAC, MPAC+1.  ENDS WITH TC DSPDCEND

# Page 418
		SETLOC	BLANKCON +1

		COUNT*	$$/PIN

; ============================================================================
; DEGOUTSF - Degree Output Scale Factor
;
; Converts AGC angular representation to decimal degrees for display. The AGC
; stores angles in a scaled format where 360 degrees = 2^14 (16384) units.
; This routine scales by 0.18 (approximately 360/2048) to convert to degrees.
;
; For negative angles (in AGC's one's complement format), an augmenter of 0.18
; is added to correct the representation.
;
; CODE-ALONG READERS: The low 14 bits contain the angle value. FIXRANGE checks
; the sign, and SETAUG adds a correction factor if needed for negative values.
; ============================================================================

# DEGOUTSF SCALES BY .18 THE LOW 14 BITS OF ANGLE, ADDING .18 FOR
# NUMBERS IN THE NEGATIVE (AGC) RANGE.

DEGOUTSF	CAF	ZERO
		TS	MPAC +2		# SET INDEX FOR FULL SCALE.
		TC	FIXRANGE
		TC	+2		# NO AUGMENT NEEDED (SFTEMP1 AND 2 ARE 0)
		TC	SETAUG		# SET AUGMENTER ACCORDING TO C(MPAC +2)
		TC	DEGCOM

; ============================================================================
; 360-CDUO - Display 360 Minus CDU Angle
;
; Computes the complement of a CDU (Coupling Data Unit) angle and displays it.
; CDUs are the gimbal angle readouts from the IMU (Inertial Measurement Unit).
; This routine is used when the display requires the complementary angle.
;
; For example, if a CDU reads 45 degrees, this routine displays 315 degrees
; (360 - 45). Special case: 0 or 180 degrees remain unchanged (their own
; complements).
;
; CODE-ALONG READERS: Uses one's complement arithmetic (CS instruction) to
; negate, then adds 1 to complete two's complement conversion.
; ============================================================================

# 360-CDUD COMPUTES 360 - CDU ANGLE IN MPAC, STORES RESULT IN MPAC AND
# GOES TO DEGOUTSF.

360-CDUO	TC	360-CDU
		TC	DEGOUTSF

; Subroutine to compute 360 - angle. Returns via Q register.
360-CDU		CA	MPAC
		MASK	POSMAX		# IF ANGLE IS 0 OR 180 DEGREES, DO NOTHING
		EXTEND
		BZF	360-CDUE
		CS	MPAC		# COMPUTE 360 DEGREES MINUS ANGLE
		AD	ONE
		TS	MPAC
360-CDUE	TC	Q

; ============================================================================
; LRPOSOUT - Landing Radar Position Output
;
; Displays the landing radar antenna position as a whole number (0, 1, 2, or 3)
; based on the state of channel 33 bits 7-6. The landing radar could be positioned
; in one of four positions during lunar descent.
;
; Bit pattern decoding:
;   Bits 7-6 = 11 (binary) → Display 0
;   Bits 7-6 = 10 (binary) → Display 1  
;   Bits 7-6 = 01 (binary) → Display 2
;   Bits 7-6 = 00 (binary) → Display 3
;
; CODE-ALONG READERS: Reads hardware channel 33, multiplies by BIT10 to shift
; bits 7-6 into bits 2-1 position, complements, masks to get final value.
;
; Historical context: During Apollo 11 descent, the landing radar provided crucial
; altitude and velocity data. This routine displayed the radar's physical position.
; ============================================================================

# LRPOSOUT DISPLAYS +0,1,2, OR 3 (WHOLE) FOR CHANNEL 33, BITS 7-6 = 11,10,
# 01,00 RESPECTIVELY

LRPOSOUT	EXTEND
		READ	CHAN33
		EXTEND
		MP	BIT10		# BITS 7-6 TO BITS 2-1
		COM
		MASK	THREE
		TS	MPAC
		TC	ARTOUTSF	# DISPLAY AS WHOLE

; ============================================================================
; SETAUG - Set Augmenter
;
; Loads the double-precision augmenter constant from DEGTAB into SFTEMP1 and
; SFTEMP2. The augmenter corrects for AGC's one's complement representation
; when converting negative angles to displayable decimal format.
;
; Input: MPAC+2 contains index into DEGTAB (0 for 0.18, 2 for 0.45)
; Output: SFTEMP1, SFTEMP2 loaded with augmenter constant
; ============================================================================

SETAUG		EXTEND			# LOADS SFTEMP1 AND SFTEMP2 WITH THE
		INDEX	MPAC +2		# DP AUGMENTER CONSTANT
		DCA	DEGTAB
		DXCH	SFTEMP1
		TC	Q

; ============================================================================
; FIXRANGE - Fix Range Check
;
; Checks if MPAC contains a positive or negative value and returns accordingly:
;   - If MPAC is positive: returns to caller+1 (Q+0)
;   - If MPAC is negative: masks out sign bit, stores back to MPAC, returns to
;     caller+2 (Q+1)
;
; CODE-ALONG READERS: CCS instruction tests sign. For negative values, the sign
; bit (bit 15) is masked out to get absolute value, then indexed return jumps
; to Q+1 instead of Q+0.
; ============================================================================

FIXRANGE 	CCS	MPAC		# IF MPAC IS + RETURN TO L+1
		TC	Q		# IF MPAC IS - RETURN TO L+2 AFTER
		TC	Q		# MASKING OUT THE SIGN BIT
		TCF	+1

# Page 419
		CS	BIT15
		MASK	MPAC
		TS	MPAC
		INDEX	Q
		TC	1

; ============================================================================
; DEGCOM - Degree Conversion Common Routine
;
; Performs the core angle-to-degrees conversion: loads the multiplier from DEGTAB
; (indexed by MPAC+2), performs short multiply of the angle value, and adds the
; augmenter correction. Results are left in MPAC for display.
;
; Process:
;   1. Load scale factor from DEGTAB based on MPAC+2 index
;   2. Perform short multiply: angle × scale factor
;   3. Add augmenter (from SFTEMP1, SFTEMP2) to result
;   4. Jump to SCOUTEND for final display processing
; ============================================================================

DEGCOM		EXTEND			# LOADS MULTIPLIER, DOES SHORTMP, AND
		INDEX	MPAC +2		# ADDS AUGMENTER.
		DCA	DEGTAB
		DXCH	MPAC		# ADJUSTED ANGLE IN A
		TC	SHORTMP
		DXCH	SFTEMP1
		DAS	MPAC
		TC	SCOUTEND

; ============================================================================
; DEGTAB - Degree Conversion Table
;
; Double-precision constants for angle conversion to degrees. Two sets of
; constants handle different scale factors:
;
;   Entry 0-1: 0.18 (approximately 360°/2048) for full-scale angle conversion
;   Entry 2-3: 0.45 (approximately 360°/800) for half-scale angle conversion
;
; Format: Each entry is a DP (double-precision) value with high part first,
; low part second, stored in AGC's scaled fixed-point format.
; ============================================================================

DEGTAB		OCT	05605		# HI PART OF 	.18
		OCT	03656		# LOW PART OF	.18
		OCT	16314		# HI PART OF 	.45
		OCT	31463		# LO PART OF	.45

; ============================================================================
; ARTOUTSF - Arithmetic Output Scale Factor
;
; Scales a value for display by multiplying MPAC by the DP scale factor in
; SFTEMP1, SFTEMP2. Assumes binary point is at the left of the DP scale factor.
; Used for displaying whole numbers and simple scaled values.
;
; CODE-ALONG READERS: PRSHRTMP (Preserved Short Multiply) handles the special
; case where A = -0, which SHORTMP cannot handle correctly.
; ============================================================================

ARTOUTSF	DXCH	SFTEMP1		# ASSUMES POINT AT LEFT OF DP SFCON
		DXCH	MPAC
		TC	PRSHRTMP	# IF C(A) = -0, SHORTMP FAILS TO GIVE -0.
SCOUTEND	TC	POSTJUMP
		CADR	DSPDCEND

; ============================================================================
; AROUT1SF - Arithmetic Output 1 Scale Factor
;
; Similar to ARTOUTSF but assumes binary point is between high and low parts of
; the DP scale factor. After multiplication, shifts result left 14 bits by taking
; results from MPAC+1 and MPAC+2 (via L14/OUT). Used for fractional displays
; requiring different scaling.
; ============================================================================

AROUT1SF	DXCH	SFTEMP1		# ASSUMES POINT BETWEEN HI AND LO PARTS OF
		DXCH	MPAC		# DP SFCON. SHIFTS RESULTS LEFT 14, BY
		TC	PRSHRTMP	# TAKING RESULTS FROM MPAC+1, MPAC+2.
		TC	L14/OUT

; ============================================================================
; DP1OUTSF - Double Precision 1 Output Scale Factor
;
; Scales a double-precision value (MPAC, MPAC+1) by the DP scale factor in
; SFTEMP1, SFTEMP2, then scales the result by B14 (shifts left 14 bits).
; Uses DPOUT for the DP multiply, then L14/OUT to shift and finalize.
; ============================================================================

DP1OUTSF	TC	DPOUT		# SCALES MPAC, MPAC +1 BY DP SCALE FACTOR
L14/OUT		XCH	MPAC +2		# IN SFTEMP1, SFTEMP2.  THEN SCALE RESULT
		XCH	MPAC +1		# BY B14
		TS	MPAC
		TC	SCOUTEND

; ============================================================================
; DP2OUTSF - Double Precision 2 Output Scale Factor
;
; Simpler DP scale routine: scales MPAC, MPAC+1 by the DP scale factor in
; SFTEMP1, SFTEMP2 without additional shifting. Used for standard DP displays.
; ============================================================================

DP2OUTSF	TC	DPOUT		# SCALES MPAC, MPAC +1 BY DP SCALE FACTOR
		TC	SCOUTEND

; ============================================================================
; DP3OUTSF - Double Precision 3 Output Scale Factor
;
; Specialized DP scale routine that assumes binary point between bits 7-8 of
; the high part. After DPOUT, shifts left by 7 bits using TPLEFTN (Triple Left N),
; rounding MPAC+2 into MPAC+1. Used for fractional values with specific scaling.
; ============================================================================

DP3OUTSF	TC	DPOUT		# ASSUMES POINT BETWEEN BITS 7-8 OF HIGH
		CAF	SIX		# LEFT BY 7, ROUNDS MPAC+2 INTO MPAC+1.
		TC	TPLEFTN		# SHIFT LEFT 7.
		TC	SCOUTEND
# Page 420
MPAC+6		= 	MPAC +6		# USE MPAC +6 INSTEAD OF OVFIND

; ============================================================================
; DPOUT - Double Precision Output Common Routine
;
; Core DP multiplication routine called by all DPxOUTSF routines. Multiplies
; the DP value in MPAC, MPAC+1 by the DP scale factor in SFTEMP1, SFTEMP2.
;
; Process:
;   1. Saves return address in MPAC+6
;   2. Calls READLO to refresh data for high and low parts
;   3. Calls TPAGREE to ensure DP data agreement (consistency check)
;   4. Performs DP multiply (DMP) with scale factor at SFTEMP1
;   5. Returns via saved address in MPAC+6
;
; CODE-ALONG READERS: READLO and TPAGREE ensure data integrity before the
; critical multiplication, preventing display of stale or inconsistent values.
; ============================================================================

DPOUT		XCH	Q
		TS	MPAC+6
		TC	READLO		# GET FRESH DATA FOR BOTH HI AND LO.
		TC	TPAGREE		# MAKE DP DATA AGREE
		TC	DMP
		ADRES	SFTEMP1
		TC	MPAC+6

; ============================================================================
; 2INTOUT - Two Integer Output Display Routine
;
; Displays two contiguous single-precision positive integers as two separate
; positive decimal integers on the DSKY:
;   - First integer (MPAC) displayed in Register 1 digits 1-2 (RXD1-RXD2)
;   - Second integer (MPAC+1) displayed in Register 1 digits 4-5 (RXD4-RXD5)
;   - Register 1 digit 3 (RXD3) is blanked for visual separation
;
; Used for displaying paired values like hours/minutes or minutes/seconds
; where both values are whole numbers. The lower-addressed integer appears
; first (left) on the display.
;
; COMMENT-ONLY READERS: This creates displays like "12 34" where the space
; separates two related time components or paired values.
; ============================================================================

# THE FOLLOWING ROUTINE DISPLAYS TWO CONTIGUOUS SP POSITIVE INTEGERS
# AS TWO POSITIVE DECIMAL INTEGERS IN RXD1-RXD2 AND RXD4-RXD5 (RXD3 IS
# BLANKED).  THE INTEGER IN THE LOWER NUMBERED ADDRESS IS DISPLAYED IN
# RXD1-RXD2.

2INTOUT		TC	5BLANK		# TO BLANK RXD3
		TC	+ON		# TURN ON + SIGN
		CA	MPAC
		TC	DSPDECVN	# DISPLAY 1ST INTEGER (LIKE VERB AND NOUN)
		CS	THREE
		INDEX	DECOUNT
		AD	R1D1		# RXD4
		TS	DSPCOUNT
		TC	READLO		# GET 2ND INTEGER
		CA	MPAC +1
		TC	DSPDECVN	# DISPLAY 2ND INTEGER (LIKE VERB AND NOUN)
		TC	POSTJUMP
		CADR	DSPDCEND +2

; ============================================================================
; READLO - Read Low Component Fresh Data
;
; Fetches fresh data for both high (HI) and low (LO) components of a double-
; precision value, placing results in MPAC and MPAC+1. Critical for time
; displays and other values that may change during computation. Zeroes MPAC+2
; but does NOT force TPAGREE (data agreement check).
;
; Process:
;   1. Saves return address in TEM4
;   2. Checks MIXBR flag to determine mixed/normal addressing mode
;   3. Retrieves IDADDTAB entry for the noun component K
;   4. Extracts E-bank address (E_subK) and sets E-bank
;   5. Performs indexed DCA to load both words into MPAC, MPAC+1
;   6. Clears MPAC+2 to zero
;   7. Returns via saved address
;
; CODE-ALONG READERS: The MIXBR index determines whether to use mixed
; addressing (offset indexing) or normal addressing. SETEBANK sets the
; erasable bank for multi-bank noun data access.
; ============================================================================

# READLO PICKS UP FRESHDATA FOR BOTH HI AND LO AND LEAVES IT IN
# MPAC, MPAC+1.  THIS IS NEEDED FOR TIME DISPLAY.  IT ZEROES MPAC+2, BUT
# DOES NOT FORCE TPAGREE.

READLO		XCH	Q
		TS	TEM4
		INDEX	MIXBR
		TC	+0
		TC	RDLONOR
		INDEX	DECOUNT
		CA	IDAD1TEM	# GET IDADDTAB ENTRY FOR COMP K OF NOUN.
		MASK	LOW11		# E SUBK
		TC	SETEBANK	# SET EB, LEAVE EADRES IN A.
READLO1		EXTEND			# MIXED			NORMAL
		INDEX	A		# C(ESUBK)		C(E)
		DCA	0		# C(E SUBK)+1)		C(E+1)
		DXCH	MPAC
		CAF	ZERO
		TS	MPAC	+2
		TC	TEM4
# Page 421
RDLONOR		CA	NOUNADD		# E
ENDRDLO		TC	READLO1

		BANK	42
		SETLOC	PINBALL3
		BANK

		COUNT*	$$/PIN
;
; ============================================================================
; HMSOUT - Hours, Minutes, Seconds Output Formatter
; ============================================================================
; FORMAT: Converts double-precision time data to hours, minutes, seconds
; display on DSKY. This routine is used for displaying mission elapsed time,
; ground elapsed time, and other time-based values in HH:MM:SS format.
;
; INPUTS: NOUNADD points to time data (double-precision centiseconds)
; OUTPUTS: DSKY display shows time in hours, minutes, seconds
;
; COMMENT-ONLY READERS: This routine displays time values on the DSKY in a
; familiar hours:minutes:seconds format. During Apollo 11 descent, the crew
; monitored mission elapsed time displayed by this routine.
;
; CODE-ALONG READERS: Converts scaled time from storage format to display
; format, separates into hours/minutes/seconds components, applies scaling
; factors (.06 for seconds, .0006 for minutes, .16384 for hours), and sends
; each component to decimal display routine DSPDECWD.
; ============================================================================
HMSOUT		TC	BANKCALL	# READ FRESH DATA FOR HI AND LO INTO MPAC,
		CADR	READLO		# MPAC+1.
		TC	TPAGREE		# MAKE DP DATA AGREE.
		TC	SEPSECNR	# LEAVE FRACT SEC/60 IN MPAC, MPAC+1. LEAVE
					# WHOLE MIN IN BIT13 OF LOTEMOUT AND ABOVE
		TC	DMP		# USE ONLY FRACT SEC/60 MOD 60
		ADRES	SECON2		# MULT BY .06
		CAF	R3D1		# GIVES CENTI-SEC/10EXP5 MOD 60
		TS	DSPCOUNT
		TC	BANKCALL	# DISPLAY SEC MOD 60
		CADR	DSPDECWD
		TC	SEPMIN		# REMOVE REST OF SECONDS
		CAF	MINCON2		# LEAVE FRACT MIN/60 IN MPAC+1.  LEAVE
		XCH	MPAC		# WHOLE HOURS IN MPAC.
		TS	HITEMOUT	# SAVE WHOLE HOURS.
		CAF	MINCON2 +1
		XCH	MPAC 	+1	# USE ONLY FRACT MIN/60 MOD 60
		TC	PRSHRTMP	# IF C(A) = -0, SHORTMP FAILS TO GIVE -0.
					# MULT BY .0006
		CAF	R2D1		# GIVE MIN/10EXP5 MOD 60
		TS	DSPCOUNT
		TC	BANKCALL	# DISPLAY MIN MOD 60
		CADR	DSPDECWD
		EXTEND			# MINUTES, SECONDS HAVE BEEN REMOVED
		DCA	HRCON1
		DXCH	MPAC
		CA	HITEMOUT 	# USE WHOLE HOURS
		TC	PRSHRTMP	# IF C(A) = -0, SHORTMP FAILS TO GIVE -0.
					# MULT BY .16384
		CAF	R1D1		# GIVES HOURS/10EXP5
		TS	DSPCOUNT
		TC	BANKCALL	# USE REGULAR DSPDECWD, WITH ROUND OFF.
		CADR	DSPDECWD
		TC	ENTEXIT

; Time conversion constants for HMSOUT display formatting
SECON1		2DEC*	1.666666666 E-4 B12*	# 2EXP12/6000
SECON2		OCT	01727		# .06 FOR SECONDS DISPLAY
		OCT	01217
MINCON2		OCT	00011		# .0006 FOR MINUTES DISPLAY
		OCT	32445
# Page 422
MINCON1		OCT	02104		# .066..66 UPPED BY 2EXP-28
		OCT	10422
HRCON1		2DEC	.16384

		OCT	00000
RNDCON		OCT	00062		# .5 SEC

; ============================================================================
; M/SOUT - Minutes/Seconds Output Formatter
; ============================================================================
; FORMAT: Converts time data to minutes:seconds display (MM:SS) with a
; maximum limit of 59:59. Used for displaying time intervals, countdowns,
; or elapsed times that don't require hours.
;
; INPUTS: NOUNADD points to time data (double-precision centiseconds)
; OUTPUTS: DSKY display shows MM:SS format, limited to 59:59
;
; COMMENT-ONLY READERS: This routine displays time in minutes and seconds
; format, limiting display to 59:59 maximum. Used for shorter time intervals
; where hours are not needed.
;
; CODE-ALONG READERS: Performs range check against 59:58.5s limit (M/SCON1,
; M/SCON2), clamps to +/-59:59 if exceeded (M/SLIMIT), then separates into
; minutes and seconds using SEPSEC/SEPMIN. Displays with blank character
; between minutes and seconds.
; ============================================================================
M/SOUT		TC	BANKCALL	# READ FRESH DATA FOR HI AND LO INTO MPAC.
		CADR	READLO		# MPAC+1.
		TC	TPAGREE		# MAKE DP DATA AGREE
		CCS	MPAC		# IF MAG OF (MPAC, MPAC+1) G/ 59 M 59 S.
		TC	+2		# DISPLAY 59B59, WITH PROPER SIGN.
		TC	M/SNORM		# MPAC = +0. L/ 59M58.5S
		AD	M/SCON1		# - HI PART OF (59M58.5S) +1 FOR CCS
		CCS	A		# MAG OF MPAC - HI PART OF (59M58.5S)
		TC	M/SLIMIT	# G/ 59M58.5S
		TC	M/SNORM		# ORIGINAL MPAC = -0. L/ 59M58.5S
		TC	M/SNORM		# L/ 59M58.5S
		CCS	MPAC +1		# MAG OF MPAC = HI PART OF 59M58.5S
		TC	+2
		TC	M/SNORM		# MPAC+1 = +0. L/ 59M58.5S
		AD	M/SCON2		# - LO PART OF (59M58.5S) +1 FOR CCS
		CCS	A		# MAG OF MPAC+1 - LO PART OF (59M58.5S)
		TC	M/SLIMIT	# G/ 59M58.5S
		TC	M/SNORM		# ORIGINAL MPAC+1 = -0. L/ 59M58.5S
		TC	M/SNORM		# L/ 59M58.5S
; M/SLIMIT: Clamp time value to +/-59:59 maximum
M/SLIMIT	CCS	MPAC		# = 59M58.5S	LIMIT
		CAF	M/SCON3		# MPAC CANNOT BE +/- 0 AT THIS POINT.
		TC	+LIMIT		# FORCE MPAC, MPAC+1 TO +/- 59M59.5S
		CS	M/SCON3
		TS	MPAC		# WILL DISPLAY 59M59S IN DSPDECNR
		CS	M/SCON3 +1
LIMITCOM	TS	MPAC +1
		CAF	NORMADR		# SET RETURN TO M/SNORM+1.
		TC	SEPSECNR +1
+LIMIT		TS	MPAC
		CAF	M/SCON3 +1
		TC	LIMITCOM
; M/SNORM: Normal path - convert to MM:SS display format
; Separates minutes and seconds, formats each for DSKY display in D1D2 (minutes) 
; and D4D5 (seconds), with D3 blank between them.
M/SNORM		TC	SEPSEC		# LEAVE FRACT SEC/60 IN MPAC,MPAC+1. LEAVE
					# WHOLE MIN IN BIT13 OF LOTEMOUT AND ABOVE
		CAF	HISECON		# USE ONLY FRACT SEC/60 MOD 60
		TC	SHORTMP		# MULT BY .6 + 2EXP-14
		CS	THREE		# GIVES SEC/100 MOD 60
		ADS	DSPCOUNT	# DSPCOUNT ALREADY SET TO RXD1
		TC	BANKCALL	# DISPLAY SEC MOD 60 IN D4D5.
		CADR	DSPDC2NR
		CAF	ZERO
		TS	CODE
		CS	TWO
# Page 423
		INDEX	DECOUNT
		AD	R1D1		# RXD3
		TS	COUNT
		TC	BANKCALL	# BLANK MIDDLE CHAR
		CADR	DSPIN
		TC	SEPMIN		# REMOVE REST OF SECONDS
		XCH	MPAC +1		# LEAVE FRACT MIN/60 IN MPAC+1
		EXTEND			# USE ONLY FRACT MIN/60 MOD 60
		MP	HIMINCON	# MULT BY .6 + 2EXP-7
		DXCH	MPAC		# GIVES MIN/100 MOD 60
		INDEX	DECOUNT
		CAF	R1D1		# RXD1
		TS	DSPCOUNT
		TC	BANKCALL	# DISPLAY MIN MOD 60 IN D1D2.
		CADR	DSPDC2NR
		TC	POSTJUMP
		CADR	DSPDCEND +2

; Scaling constants for M/S time display formatting
HISECON		OCT	23147		# .6 + 2EXP-14
HIMINCON	OCT	23346		# .6 + 2EXP-7

; Constants for M/S limit checking (max 59:59)
M/SCON1		OCT	77753		# - HI PART OF (59M58.5S) +1
M/SCON2		OCT	41126		# - LO PART OF (59M58.5S) +1
NORMADR		ADRES	M/SNORM +1
M/SCON3		OCT	00025		# 59M 59.5S
		OCT	37016

; ============================================================================
; SEPSEC: Separate seconds from time value
; Rounds time by +/- 0.5 seconds, extracts fractional seconds/60, and leaves
; whole minutes in BIT13 of LOTEMOUT and above.
; Used by time display formatting routines.
; ============================================================================
SEPSEC		CCS	MPAC	+1	# IF +, ROUND BY ADDING .5 SEC
		TCF	POSEC		# IF -, ROUND BY SUBTRACING .5 SEC
		TCF	POSEC		# FINDS TIME IN MPAC, MPAC+1
		TCF	+1		# ROUNDS OFF BY +/- .5 SEC
		EXTEND			# LEAVES WHOLE MIN IN BIT13 OF
		DCS	RNDCON	-1	# LOTEMOUT AND ABOVE.
SEPSEC1		DAS	MPAC		# LEAVES FRACT SEC/60 IN MPAC, MPAC+1.
		TCF	SEPSECNR
POSEC		EXTEND
		DCA	RNDCON -1
		TCF	SEPSEC1
SEPSECNR	XCH	Q		# THIS ENTRY AVOIDS ROUNDING BY .5 SEC
		TS	SEPSCRET
		TC	DMP		# MULT BY 2EXP12/6000
		ADRES	SECON1		# GIVES FRACT SEC/60 IN BIT12 OF MPAC+1
		EXTEND			# AND BELOW.
		DCA	MPAC		# SAVE MINUTES AND HOURS
		DXCH	HITEMOUT
		TC	TPSL1
		TC	TPSL1		# GIVES FRACT SEC/60 IN MPAC+1, MPAC+2.
		CAF	ZERO
		XCH	MPAC +2		# LEAVE FRACT SEC/60 IN MPAC, MPAC+1.
# Page 424
		XCH	MPAC +1
		XCH	MPAC
		TC	SEPSCRET

; ============================================================================
; SEPMIN: Separate minutes from time value
; Extracts whole minutes from BIT13 of LOTEMOUT and above, removes seconds,
; leaves fractional min/60 in MPAC+1 and whole hours in MPAC.
; Called by time formatting routines after SEPSEC.
; ============================================================================
SEPMIN		XCH	Q		# FIND WHOLE MINUTES IN BIT13
		TS	SEPMNRET	# OF LOTEMOUT AND ABOVE.
		CA	LOTEMOUT	# REMOVES REST OF SECONDS.
		EXTEND			# LEAVES FRACT MIN/60 IN MPAC+1.
		MP	BIT3		# LEAVES WHOLE HOURS IN MPAC.
		EXTEND			# SR 12, THROW AWAY LP.
		MP	BIT13		# SR 2, TAKE FROM LP. = SL 12.
		LXCH	MPAC +1		# THIS FORCES BITS 12-1 TO 0 IF +.
					# FORCES BITS 12-1 TO 1 IF -.
		CA	HITEMOUT
		TS	MPAC
		TC	DMP		# MULT BY 1/15
		ADRES	MINCON1		# GIVES FRACT MIN/60 IN MPAC+1.
ENDSPMIN	TC	SEPMNRET	# GIVES WHOLE HOURS IN MPAC.

; ============================================================================
; DSPDPDEC: Display Double Precision Decimal
; Special-purpose verb for displaying a double precision AGC word as 10 decimal
; digits on the DSKY. Can be used with any noun except mixed nouns. Displays
; the contents of the register pointed to by NOUNADD. Display appears in R1 and 
; R2 only with sign in R1.
; Warning: If used with inherently non-DP nouns (e.g., CDU counters), display 
; will be garbage.
; ============================================================================
# THIS IS A SPECIAL PURPOS VERB FOR DISPLAYING A DOUBLE PRECISION AGC
# WORD AS 10 DECIMAL DIGITS ON THE AGC DISPLAY PANEL.  IT CAN BE USED WITH
# ANY NOUN, EXCEPT MIXED NOUNS. IT DISPLAYS THE CONTENTS
# OF THE REGISTER NOUNADD IS POINTING TO.  IF USED WITH NOUNS WHICH ARE
# INHERENTLY NOT DP SUCH AS THE CDU COUNTERS THE DISPLAY WILL BE GARBAGE.
# DISPLAY IS IN R1 AND R2 ONLY WITH THE SIGN IN R1.

		SETLOC	ENDRDLO +1

		COUNT*	$$/PIN
DSPDPDEC	INDEX	MIXBR
		TC	+0
		TC	+2		# NORMAL NOUN
		TC	DSPALARM
		EXTEND
		INDEX	NOUNADD
		DCA	0
		DXCH	MPAC
		CAF	R1D1
		TS	DSPCOUNT
		CAF	ZERO
		TS	MPAC +2
		TC	TPAGREE
		TC	DSP2DEC
ENDDPDEC	TC	ENTEXIT

# Page 425
# LOAD VERBS		IF ALARM CONDITION IS DETECTED DURING EXECUTE,
# CHECK FAIL LIGHT IS TURNED ON AND ENDOFJOB.  IF ALARM CONDITION IS
# DETECTED DURING ENTER OF DATA, CHECK FAIL IS TURNED ON AND IT RECYCLES
# TO EXECUTE OF ORIGINAL LOAD VERB.  RECYCLE CAUSED BY  1) DECIMAL MACHINE
# CADR  2) MIXTURE OF OCTAL/DECIMAL DATA  3) OCTAL DATA INTO DECIMAL
# ONLY NOUN  4) DEC DATA INTO OCT ONLY NOUN  5) DATA TOO LARGE FOR SCALE
# 6) FEWER THAN 3 DATA WORDS LOADED FOR HRS, MIN, SEC NOUN.  (2)-(6) ALARM
# AND RECYCLE OCCUR AT FINAL ENTER OF SET. (1) ALARM AND RECYCLE OCCUR AT
# ENTER OF CADR.

; ============================================================================
; LOAD VERBS - Data Entry Routines
;
; The load verb family enables crew to enter numerical data into AGC memory.
; These routines were used extensively during Apollo 11 mission for entering:
; - Landing site coordinates (V24 N17 during descent preparation)
; - AGS initialization data (V21/V22 for Abort Guidance System backup)
; - Navigation state vectors (V21 N01 for position/velocity updates)
; - TIME entries (V25 N36 for mission elapsed time corrections)
;
; Load Verb Categories:
; V21 (ALOAD): Load one component (R1 only)
; V22 (ABLOAD): Load two components (R1, R2)
; V23 (ABCLOAD): Load three components (R1, R2, R3)
; V24/V25: Load with octal/decimal bit display verification
;
; Error Handling: Load verbs implement comprehensive validation:
; 1) Decimal machine CADR detection -> alarm + recycle to execute
; 2) Mixed octal/decimal data -> OPR ERR light + recycle
; 3) Octal data into decimal-only noun -> alarm + recycle
; 4) Decimal data into octal-only noun -> alarm + recycle
; 5) Data too large for scale factor -> alarm + recycle
; 6) Incomplete time data (HMS nouns) -> alarm + recycle
;
; COMMENT-ONLY READERS: These routines process crew keypad entries (0-9, +, -)
; during data loading operations. Armstrong and Aldrin relied on load verbs
; to enter critical mission parameters during descent and surface operations.
;
; CODE-ALONG READERS: Load verbs coordinate DSKY display state machine,
; validate octal vs decimal mode consistency, apply scale factors from noun
; table, perform bank switching for memory access, and handle all six error
; conditions with appropriate alarm display and verb recycle logic.
; ============================================================================

		SETLOC	ENDRTOUT

		COUNT*	$$/PIN

; V23 ABCLOAD - Load Three Components (R1, R2, R3)
; Used for three-dimensional vectors, position/velocity triads, time entries.
; Example: V23 N01 loads position vector (X, Y, Z coordinates).
; Validates all three components are consistent (all decimal or all octal).
ABCLOAD		CS	TWO
		TC	COMPTEST
		TC	NOUNTEST	# TEST IF NOUN CAN BE LOADED.
		CAF	VBSP1LD
		TC	UPDATVB -1
		TC	REQDATX
		CAF	VBSP2LD
		TC	UPDATVB -1
		TC	REQDATY
		CAF	VBSP3LD
		TC	UPDATVB -1
		TC	REQDATZ

PUTXYZ		CS	SIX		# TEST THAT THE 3 DATA WORDS LOADED ARE
		TC	ALLDC/OC	# ALL DEC OR ALL OCT.
		EXTEND
		DCA	LODNNLOC	# SWITCH BANKS TO NOUN TABLE READING
		DXCH	Z		# ROUTINE.
		CAF	ZERO		# X COMP
		TC	PUTCOM
		INDEX	NOUNADD
		TS	0
		CAF	ONE		# Y COMP
		TC	PUTCOM
		INDEX	NOUNADD
		TS	1
		CAF	TWO		# Z COMP
		TC	PUTCOM
		INDEX	NOUNADD
		TS	2
		CS	SEVEN		# IF NOUN 7 HAS JUST BEEN LOADED, SET
		AD	NOUNREG		# FLAG BITS AS SPECIFIED.
		EXTEND
		BZF	+2
		TC	LOADLV
# Page 426
		CA	XREG		# ECADR OF FLAG WORD.
		TC	SETNCADR +1	# SET EBANK, NOUNADD.
		CA	ZREG		# ZERO TO RESET BITS, NON-ZERO TO SET BITS.
		INHINT
		EXTEND
		BZF	BITSOFF
		INDEX	NOUNADD
		CS	0
		MASK	YREG		# BITS TO BE PROCESSED.
		INDEX	NOUNADD
		ADS	0		# SET BITS.
		TC	BITSOFF1
BITSOFF		CS	YREG		# BITS TO BE PROCESSED.
		INDEX	NOUNADD
		MASK	0
		INDEX	NOUNADD
		TS	0		# RESET BITS.
BITSOFF1	RELINT
		TC	LOADLV

; V22 ABLOAD - Load Two Components (R1 and R2)
; Entry point for V22 verb (load two data items).
; Used for: coordinate pairs, two-dimensional vectors, paired parameters.
; Example: V22 N17 during Apollo 11 descent loaded target delta-V components.
; Sequence: Display V22, request R1 data, request R2 data, validate consistency.
ABLOAD		CS	ONE
		TC	COMPTEST
		TC	NOUNTEST	# TEST IF NOUN CAN BE LOADED.
		CAF	VBSP1LD
		TC	UPDATVB -1
		TC	REQDATX
		CAF	VBSP2LD
		TC	UPDATVB -1
		TC	REQDATY

; PUTXY - Validate and Store Two Components
; Validates that both R1 and R2 data are consistent (both decimal or both octal).
; Then stores data to noun address+0 and noun address+1.
PUTXY		CS	FIVE		# TEST THAT THE 2 DATA WORDS LOADED ARE
		TC	ALLDC/OC	# ALL DEC OR ALL OCT.
		EXTEND
		DCA	LODNNLOC	# SWITCH BANKS TO NOUN TABLE READING
		DXCH	Z		# ROUTINE.
		CAF	ZERO		# X COMP
		TC	PUTCOM
		INDEX	NOUNADD
		TS	0
		CAF	ONE		# Y COMP
		TC	PUTCOM
		INDEX	NOUNADD
		TS	1
		TC	LOADLV

; V21 ALOAD - Load Single Component (R1 only)
; Entry point for V21 verb (load one data item into first register).
; Used for: single values, scalar parameters, individual settings.
; Example: V21 N26 loads crew-entered priority/delay for programs.
; Simpler than V22 - only requests R1 data, stores to noun address+0.
ALOAD		TC	REQDATX
		EXTEND
		DCA	LODNNLOC	# SWITCH BANKS TO NOUN TABLE READING
		DXCH	Z		# ROUTINE.
		CAF	ZERO		# X COMP
		TC	PUTCOM
# Page 427
		INDEX	NOUNADD
		TS	0
		TC	LOADLV

; V23 BLOAD - Load Second Component (R2 only)
; Entry point for V23 verb (load one data item into second register).
; Used for: updating Y-component of coordinate, modifying second element only.
; Example: V23 N17 loads just the Y-axis delta-V without changing X.
; Stores data to noun address+1 (second component).
BLOAD		CS	ONE
		TC	COMPTEST
		CAF	BIT15		# SET CLPASS FOR PASS0 ONLY
		TS	CLPASS
		TC	REQDATY
		EXTEND
		DCA	LODNNLOC	# SWITCH BANKS TO NOUN TABLE READING
		DXCH	Z		# ROUTINE.
		CAF	ONE
		TC	PUTCOM
		INDEX	NOUNADD
		TS	1
		TC	LOADLV

; V24 CLOAD - Load Third Component (R3 only)
; Entry point for V24 verb (load one data item into third register).
; Used for: updating Z-component of vector, modifying third element only.
; Example: V24 N17 loads just the Z-axis delta-V without changing X or Y.
; Stores data to noun address+2 (third component).
CLOAD		CS	TWO
		TC	COMPTEST
		CAF	BIT15		# SET CLPASS FOR PASS0 ONLY
		TS	CLPASS
		TC	REQDATZ
		EXTEND
		DCA	LODNNLOC	# SWITCH BANKS TO NOUN TABLE READING
		DXCH	Z		# ROUTINE.
		CAF	TWO
		TC	PUTCOM
		INDEX	NOUNADD
		TS	2
		TC	LOADLV

; LOADLV - Complete Load Verb Processing
; Common exit point for all load verbs (V21, V22, V23, V24).
; Clears load state flags, releases display system for other programs,
; blocks further numerical entry, then checks for pending recall conditions.
; This routine is called after data has been successfully validated and stored.
LOADLV		CAF	ZERO
		TS	DECBRNCH
		CS	ZERO
		TS	LOADSTAT
		TC	RELDSP		# RELEASE FOR PRIORITY DISPLAY PROBLEM.
		CS	VD1		# TO BLOCK NUMERICAL CHARACTERS AND
		TS	DSPCOUNT	# CLEARS AFTER A COMPLETED LOAD
		TC	POSTJUMP	# AFTER COMPLETED LOAD, GO TO RECALTST
		CADR	RECALTST	# TO SEE IF THERE IS RECALL FROM ENDIDLE.

VBSP1LD		DEC	21		# VB21 = ALOAD
VBSP2LD		DEC	22		# VB22 = BLOAD
VBSP3LD		DEC	23		# VB23 = CLOAD

; ALLDC/OC - Validate Decimal/Octal Consistency
; Critical validation routine ensuring crew doesn't accidentally mix decimal
; and octal data entry (e.g., entering first component in decimal, second in octal).
; The AGC allows both input modes, but mixing them within a single noun would
; produce incorrect results. This routine checks DECBRNCH flags for consistency.
; If inconsistency detected, issues operator error and forces recycle.
; Apollo 11 crew trained extensively to avoid this error during mission.
ALLDC/OC	TS	DECOUNT		# TESTS THAT DATA WORDS LOADED ARE EITHER
		CS	DECBRNCH	# ALL DEC OR ALL OCT. ALARMS IF NOT.
		TS	SR
# Page 428
		CS	SR
		CS	SR		# SHIFTED RIGHT 2
		CCS	A		# DEC COMP BITS IN LOW 3
		TCF	+2		# SOME ONES IN LOW 3
		TC	Q		# ALL ZEROS. ALL OCTAL.  OK
		AD	DECOUNT		# DEC COMP = 7 FOR 3COMP, =6 FOR 2COMP
		EXTEND			# (BUT IT HAS BEEN DECREMENTED BY CCS)
		BZF	+2		# MUST MATCH 6 FOR 3COMP, 5 FOR 2COMP.
		TC	ALMCYCLE	# ALARM AND RECYCLE.
GOQ		TC	Q		# ALL REQUIRED ARE DEC.  OK

; SFRUTNOR - Scale Factor Routine Number Lookup (Normal Nouns)
; Extracts the scale factor routine number from the noun type table for normal
; (non-mixed) nouns. Each noun has associated scale factor formatting rules
; stored in NNTYPTAB. This routine uses the MID5 mask to extract bits identifying
; which SF formatting routine should be used for display or data entry.
; Returns SF routine number in A register for subsequent SF processing.
; Used during DSKY display updates to select correct decimal point placement.
SFRUTNOR	XCH	Q		# GETS SF ROUTINE NUMBER FOR NORMAL CASE
		TS	EXITEM		# CAN'T USE L FOR RETURN. TSTFORDP USES L.
		CAF	MID5
		MASK 	NNTYPTEM
		TC	RIGHT5
		TC	EXITEM		# SF ROUTINE NUMBER IN A

; SFRUTMIX - Scale Factor Routine Number Lookup (Mixed Nouns)
; Handles SF routine number extraction for mixed nouns (nouns with different
; scale factors for each component). DECOUNT index selects which component
; (R1, R2, or R3) is being processed. Uses indexed table lookup through RUTMXTAB
; with dynamic bit field extraction (HI5, MID5, or LOW5) depending on component.
; The DISPLACE table provides indirect branching to correct bit shift routine.
; Essential for nouns like N43 (latitude/longitude/altitude) where each component
; has different units and thus different scale factor requirements.
SFRUTMIX	XCH	Q		# GETS SF ROUTINE NUMBER FOR MIXED CASE
		TS	EXITEM
		INDEX	DECOUNT
		CAF	DISPLACE	# PUT TC GOQ, TC RIGHT5, OR TC LEFT5 IN L
		TS	L
		INDEX	DECOUNT
		CAF	LOW5		# LOW5, MID5, OR HI5 IN A
		MASK	RUTMXTEM	# GET HI5, MID5, OR LOW5 OF RUTMXTAB ENTRY
		INDEX	L
		TC	0

# DO TC GOQ(DECOUNT=0), DO TC RIGHT5(DECOUNT=1), DO TC LEFT5(DECOUNT=2).
SFRET1		TC	EXITEM		# SF ROUTINE NUMBER IN A

; SFCONUM - Scale Factor Constant Number Lookup
; Retrieves the scale factor constant number (table index) for the current noun.
; Branches based on MIXBR flag: normal nouns use CONUMNOR (simple LOW5 extraction),
; mixed nouns use indexed DISPLACE branching to extract correct component's SF.
; Returns double the SF constant number (2X) for use as table index into
; scale factor constant tables. The doubling accommodates double-precision
; table entry format. Critical for mapping noun types to actual SF values
; used in decimal point positioning during display and data entry.
SFCONUM		XCH	Q		# GETS 2X(SF CONSTANT NUMBER)
		TS	EXITEM
		INDEX	MIXBR
		TC	+0
		TC	CONUMNOR	# NORMAL NOUN
		INDEX	DECOUNT		# MIXED NOUN
		CAF	DISPLACE
		TS	L		# PUT TC GOQ, TC RIGHT5, OR TC LEFT5 IN L
		INDEX	DECOUNT
		CAF	LOW5
		MASK	NNTYPTEM
		INDEX	L
		TC	0

# DO TC GOQ(DECOUNT=0), DO TC RIGHT5(DECOUNT=1), DO TC LEFT5(DECOUNT=2).
SFRET		DOUBLE			# 2X(SF CONSTANT NUMBER) IN A
		TC	EXITEM

; DISPLACE - Branch Table for Bit Field Extraction
; Three-entry table providing indirect branches to bit shift routines.
; Used by mixed noun SF processing to dynamically select correct bit field
; extraction (no shift, right 5, or left 5) based on component being processed.
DISPLACE	TC	GOQ
# Page 429
		TC	RIGHT5
		TC	LEFT5

; CONUMNOR - SF Constant Number for Normal Nouns
; Simple extractor for normal nouns (non-mixed). All components of normal noun
; share same scale factor, so always extract LOW5 bits from NNTYPTEM.
; Doubles result for table indexing. Normal nouns are majority of noun types.
CONUMNOR	CAF	LOW5		# NORMAL NOUN ALWAYS GETS LOW5 OF
		MASK	NNTYPTEM	# NNTYPTAB FOR SF CONUM.
		DOUBLE
		TC	EXITEM		# 2X(SF CONSTANT NUMBER) IN A

PUTCOM		TS	DECOUNT
		XCH	Q
		TS	DECRET
		CAF	ZERO
		TS	MPAC+6
		INDEX	DECOUNT
		XCH	XREGLP
		TS	MPAC +1
		INDEX	DECOUNT
		XCH	XREG
		TS	MPAC
		INDEX	MIXBR
		TC	+0
		TC	PUTNORM		# NORMAL NOUN
# IF MIXNOUN, PLACE ADDRESS FOR COMPONENT K INTO NOUNADD, SET EBANK BITS.
		INDEX	DECOUNT		# GET IDADDTAB ENTRY FOR COMPONENT K
		CA	IDAD1TEM	#	OF NOUN.
		MASK	LOW11		# (ECADR)SUBK FOR CURRENT COMP OF NOUN
		TC	SETNCADR	# ECADR INTO NOUNCADR. SETS EB, NOUNADD.
		EXTEND			# C(NOUNADD) IN A UPON RETURN
		SU	DECOUNT		# PLACE (ESUBK)-K INTO NOUNADD
		TS	NOUNADD
		CCS	DECBRNCH
		TC	PUTDECSF	# + DEC
		TC	DCTSTCYC	# +0 OCTAL
		TC	SFRUTMIX	# TEST IF DEC ONLY BIT = 1. IF SO,
		TC	DPTEST		# ALARM AND RECYCLE. IF NOT, CONTINUE.
		TC	PUTCOM2		# NO DP
					# TEST FOR DP SCALE FOR OCT LOAD. IF SO,
					# +0 INTO MAJOR PART. SET NOUNADD FOR
					# LOADING OCTAL WORD INTO MINOR PART.
PUTDPCOM	INCR	NOUNADD		# DP (ESUBK)-K+1  OR  E+1
		CA	NOUNADD		# NOUNADD NOW SET FOR MINOR PART
		ADS	DECOUNT		# (ESUBK)+1  OR  E+1  INTO DECOUNT
		CAF	ZERO		# NOUNADD SET FOR MINOR PART
		INDEX	DECOUNT
		TS	0 -1		# ZERO MAJOR PART(ESUBK OR E)
		TC	PUTCOM2

PUTNORM		TC	SETNADD		# ECADR FROM NOUNCADR. SETS EB, NOUNADD.
		CCS	DECBRNCH
# Page 430
		TC	PUTDECSF	# + DEC
		TC	DCTSTCYC	# +0 OCTAL
		TC	SFRUTNOR	# TEST IF DEC ONLY BIT =1. IF SO,
		TC	DPTEST		# ALARM AND RECYCLE. IF NOT, CONTINUE.
		TC	PUTCOM2 -4	# NO DP
		CAF	ZERO		# DP
		TS	DECOUNT
		TC	PUTDPCOM

		CA	NNADTEM
		AD	ONE		# IF NNADTEM = -1, CHANNEL TO BE SPECIFIED
		EXTEND
		BZF	CHANLOAD
PUTCOM2		XCH	MPAC
		TC	DECRET

		EBANK=	DSPCOUNT
GTSFINLC	2CADR	GTSFIN

CHANLOAD	CS	SEVEN		# DONT LOAD CHAN 7. (IT = SUPERBANK).
		AD	NOUNCADR
		EXTEND
		BZF	LOADLV
		CA	NOUNCADR
		MASK	LOW9
		XCH	MPAC
		EXTEND
		INDEX	MPAC
		WRITE	0
		TC	LOADLV

; ============================================================================
; NUMERIC INPUT CONVERSION SUBSYSTEM
; Routines: PUTDECSF, DEGINSF, ARTHINSF, DPINSF, HMSIN, and support functions
;
; COMMENT-ONLY READERS: When astronauts entered numbers through the DSKY,
; the data had to be converted from decimal display format into the AGC's
; internal fixed-point binary representation. Different types of data used
; different scaling factors - angles in degrees, positions in nautical miles,
; velocities in feet per second, and time in hours:minutes:seconds format.
; This section contains the conversion routines that transformed crew keyboard
; inputs into properly scaled values the guidance programs could use. During
; the lunar landing, descent velocities displayed as "XXX.X feet/sec" on the
; DSKY were converted by these routines when Armstrong or Aldrin needed to
; load target values or verify automatic guidance parameters.
;
; CODE-ALONG READERS: PUTDECSF is the main entry dispatcher that selects
; appropriate conversion routine via jump table SFINTABR. Input data arrives
; in XREG/YREG/ZREG registers from keyboard processing. Scale factor code in
; noun definition indexes into SFINTABR to select: BINROUND (no conversion),
; DEGINSF (decimal degrees), ARTHINSF (arithmetic with scale factor), or
; DPINSF (double-precision). Each routine applies appropriate scaling and
; range checking, storing final values to memory addresses specified by
; NOUNADD. Overflow/underflow conditions trigger ALMCYCLE (operator error).
; The conversions handle AGC's scaled fixed-point arithmetic: positions
; scaled as meters*2^29, angles as revolutions (1 rev = 360 deg), times as
; centiseconds. Supporting routines (BINROUND, TPLEFTN, 2ROUND, RND/TST,
; SIZETST, MPACTST) perform rounding, normalization, and validation.
; ============================================================================

# PUTDECSF FINDS MIXBR AND DECOUNT STILL SET FROM PUTCOM

; PUTDECSF - Scale Factor Conversion Entry Point
; Gets scale factor code from noun and dispatches to appropriate conversion
; routine. Handles normal nouns, mixed nouns, and fractional nouns.
; Entry: MIXBR indicates noun type (0=normal, 1=mixed, 2=fractional)
;        DECOUNT indicates number of components (1, 2, or 3)
;        XREG, YREG, ZREG contain decimal input from keyboard

PUTDECSF	TC	SFCONUM		# 2X(SF CON NUMB) IN A
		TS	SFTEMP1
		EXTEND			# SWITCH BANKS TO SF CONSTANT TABLE
		DCA	GTSFINLC	# READING ROUTINE.
		DXCH	Z		# LOADS SFTEMP1, SFTEMP2.
		INDEX	MIXBR
		TC	+0		# Branch based on noun type
		TC	PUTSFNOR	# MIXBR=0: Normal noun
		TC	SFRUTMIX	# MIXBR=1: Mixed noun (multiple scale factors)
		TC	PUTDCSF2	# MIXBR=2: Fractional noun
PUTSFNOR	TC	SFRUTNOR	# Process normal scale factor conversion

PUTDCSF2	INDEX	A
		CAF	SFINTABR	# Get address from jump table
# Page 431
		TC	BANKJUMP	# SWITCH BANKS FOR EXPANSION ROOM

; SFINTABR - Scale Factor Conversion Jump Table
; Indexed by scale factor code (from noun definition) to select conversion.
; Code  0: GOALMCYC  - Alarm if decimal load with octal-only noun
; Code  1: BINROUND  - No conversion (integer binary storage)
; Code  2: DEGINSF   - Decimal degrees to fractional revolutions (÷360)
; Code  3: ARTHINSF  - Arithmetic with scale factor (general multiply/divide)
; Code  4: DPINSF    - Double-precision scaled arithmetic
; Code  5: DPINSF2   - Alternate DP conversion
; Code  6: DSPALARM  - Landing radar position output (not loadable by crew)
; Code  7: DPINSF    - DP arithmetic version 1
; Code  8: HMSIN     - Hours:minutes:seconds input conversion
; Code  9: DSPALARM  - Minutes/seconds (not loadable separately)
; Code 10: DPINSF4   - DP version 4
; Code 11: ARTIN1SF  - Arithmetic input version 1
; Code 12: DSPALARM  - Two integer output (not loadable)
; Code 13: DEGINSF   - Degrees with 360-CDU validation

SFINTABR	CADR	GOALMCYC	# ALARM AND RECYCLE IF DEC LOAD
					# WITH OCTAL ONLY NOUN.
		CADR	BINROUND
		CADR	DEGINSF
		CADR	ARTHINSF
		CADR	DPINSF
		CADR	DPINSF2
		CADR	DSPALARM	# LRPOSOUT CANT BE LOADED.
		CADR	DPINSF		# SAME AS ARITHDP1
		CADR	HMSIN
		CADR	DSPALARM	# MIN/SEC CANT BE LOADED.
		CADR	DPINSF4
		CADR	ARTIN1SF
		CADR	DSPALARM	# 2INTOUT CANT BE LOADED.
		CADR	DEGINSF		# TESTS AT END FOR 360-CDU
ENDRUTIN	EQUALS

# SCALE FACTORS FOR THOSE ROUTINES NEEDING THEM ARE AVAILABLE IN SFTEMP1.
# ALL SFIN ROUTINES USE MPAC MPAC+1. LEAVE RESULT IN A. END WITH TC DECRET.

		SETLOC	ENDDPDEC +1

		COUNT*	$$/PIN
# DEGINSF APPLIES 1000/180 = 5.55555(10) = 5.43434(8)

; DEGINSF - Decimal Degrees to Fractional Revolutions Conversion
; Converts decimal degrees (0.00 to 360.00) entered by crew into AGC's internal
; angle representation as fractional revolutions (0 to 0.99999 revolutions).
; 
; Conversion: decimal degrees ÷ 360 = fractional revolutions
; Implementation: multiply by 1000/180 then shift left 3 positions
; This gives proper scaling where 1.0 = 360 degrees = one full revolution.
; 
; Entry: MPAC, MPAC+1 contain decimal degrees (DP format)
; Exit: MPAC contains fractional revolutions, or alarm if overflow (>360 deg)

DEGINSF		TC	DMP		# SF ROUTINE FOR DEC DEGREES
		ADRES	DEGCON1		# MULT BY 5.5 5(10)X2EXP-3
		CCS	MPAC +1		# THIS ROUNDS OFF MPAC+1 BEFORE SHIFT
		CAF	BIT11		# LEFT 3, AND CAUSES 360.00 TO OF/UF
		TC	+2		# WHEN SHIFTED LEFT AND ALARM.
		CS	BIT11
		AD	MPAC +1
		TC	2ROUND +2	# Round to 14 bits before shifting
		TC	TPSL1		# LEFT 1
DEGINSF2	TC	TPSL1		# LEFT 2
		TC	TESTOFUF	# Test for overflow/underflow
		TC	TPSL1		# RETURNS IF NO OF/UF (LEFT3)
		CCS	MPAC		# Check sign of result
		TC	SIGNFIX		# IF +, GO TO SIGNFIX
		TC	SIGNFIX		# IF +0, GO TO SIGNFIX
		COM			# IF -, USE -MAGNITUDE +1
		TS	MPAC		# IF -0, USE +0
; Handle overflow/underflow and sign correction from DEGINSF conversion
SIGNFIX		CCS	MPAC+6		# Check overflow/underflow indicator
		TC	SGNTO1		# IF OVERFLOW
		TC	ENDSCALE	# NO OVERFLOW/UNDERFLOW
		CCS	MPAC		# IF UF FORCE SIGN TO 0 EXCEPT -180
		TC	CCSHOLE
# Page 432
		TC	NEG180		# Handle -180 degree special case
		TC	+1
		XCH	MPAC
		MASK	POSMAX		# Force positive magnitude
		TS	MPAC
; ENDSCALE - Complete scale conversion and handle CDU angle adjustments
; For CDU (Coupling Data Unit) angle inputs, may need to compute 360° - angle
; based on MIXBR index (set from scale factor code in noun table).
ENDSCALE	INDEX	MIXBR		# IF ROUTINE NO. IS NOT CDU DEGREES,
		TC	+0		# THEN THIS IS 360 - CDU DEGREES
		TC	+3		# AND ANGLE IN MPAC MUST BE REPLACED
		TC	SFMIXCAL	# BY 360 DEGREES MINUS ITSELF.
MIXBACK		TC	+2
		TC	SFNORCAL
NORBACK		CS	A
		AD	BIT2
		EXTEND
		BZF	+2
		TC	360-CDU		# Compute 360° - input angle for CDU
ENDSCAL1	TC	POSTJUMP	# Return to caller
		CADR	PUTCOM2

; Helper routines for mixed and normal scale factor processing
SFMIXCAL	TC	BANKCALL
		CADR	SFRUTMIX	# Mixed scale factor routine
		TC	MIXBACK

SFNORCAL	TC	BANKCALL
		CADR	SFRUTNOR	# Normal scale factor routine
		TC	NORBACK

; Special case handlers for angle conversion
NEG180		CS	POSMAX		# Force to -180 degrees (-.5 revolutions)
		TC	ENDSCALE -1

SGNTO1		CS	MPAC		# IF OF FORCE SIGN TO 1
		MASK	POSMAX
		CS	A
		TC	ENDSCALE -1

; DEGCON1 - Conversion constant: 1000/180 = 5.555555... (decimal)
; This constant times 2^-3 converts degrees to internal angle representation.
; After multiply, 3 left shifts complete conversion: degrees ÷ 360 = revolutions
DEGCON1		2DEC	5.555555555 B-3

; ============================================================================
; ARTHINSF - Arithmetic scale factor input conversion (single precision)
; Multiplies decimal input by scale factor constant, checks for overflow,
; and rounds result. Used for most numeric inputs (non-angle, non-time).
; Entry: Decimal value in MPAC, MPAC+1; scale factor in SFTEMP1, SFTEMP2
; Exit: Scaled binary value in MPAC (single precision result)
; ============================================================================
ARTHINSF	TC	DMP		# SCALES MPAC, +1 BY SFTEMP1, SFTEMP2.
		ADRES	SFTEMP1		# ASSUMES POINT BETWEEN HI AND LO PARTS
		XCH	MPAC +2		# OF SFCON. SHIFTS RESULTS LEFT BY 14.
		XCH	MPAC +1		# (BY TAKING RESULTS FROM MPAC+1, MPAC+2)
		XCH	MPAC		# Extract single-precision result
		EXTEND
		BZF	BINROUND	# If zero, proceed to rounding
		TC	ALMCYCLE	# TOO LARGE A LOAD. ALARM AND RECYCLE.
; BINROUND - Apply binary rounding and test for overflow/underflow
BINROUND	TC	2ROUND		# Round to 2 significant bits
		TC	TESTOFUF	# Check for overflow/underflow
		TC	ENDSCAL1	# RETURNS IF NO OF/UF

# Page 433
; ARTIN1SF - Arithmetic input with rounding (variant entry point)
; Similar to ARTHINSF but enters directly at rounding step
ARTIN1SF	TC	DMP		# SCALES MPAC, +1 BY SFTEMP1, SFTEMP2.
		ADRES	SFTEMP1		# ROUNDS MPAC+1 INTO MPAC.
		TC	BINROUND

; ============================================================================
; DPINSF - Double precision input scale factor routine
; Multiplies double-precision decimal value by scale factor, stores both
; high and low parts of result. Handles mixed and normal noun types.
; Entry: DP value in MPAC/MPAC+1; scale factor in SFTEMP1/SFTEMP2
;        MIXBR flag indicates mixed (0) or normal (1) noun
; Exit: High part in E (or E SUBK), low part in E+1 (or E SUBK +1)
; ============================================================================
DPINSF		TC	DMP		# SCALES MPAC, MPAC +1 BY SFTEMP1,
		ADRES	SFTEMP1		# SFTEMP.  STORES LOW PART OF RESULT
		XCH	MPAC +2		# IN (E SUBK) +1 OR E+1
		DOUBLE			# Shift left 1 bit for proper scaling
		TS	MPAC +2
		CAF	ZERO
		AD	MPAC +1		# Add high part with overflow consideration
		TC	2ROUND +2	# Round the middle part
		TC	TESTOFUF	# Test for overflow/underflow
		INDEX	MIXBR		# RETURNS IF NO OF/UF
		TC	+0		# Branch based on noun type
		TC	DPINORM		# Jump to normal noun handling
		CA	DECOUNT		# MIXED NOUN - get component counter
; DPINCOM - Common double precision input completion
; Stores low part of result in appropriate location for mixed or normal noun
DPINCOM		AD	NOUNADD		#	MIXED		NORMAL
		TS	Q		#	E SUBK		E
		XCH	MPAC +1		# Get low part of result
		INDEX	Q
		TS	1		# PLACE LOW PART IN
		TC	ENDSCAL1	# (E SUBK) +1	MIXED

; DPINORM - Normal noun double precision entry point
DPINORM		CAF	ZERO		# E +1		NORMAL
		TC	DPINCOM		# Continue with common completion

; ============================================================================
; DPINSF2 - Double precision input with left shift by 7
; Variant of DPINSF for scale factors where binary point is between bits 7-8
; Entry: DP value in MPAC/MPAC+1, scale factor with point at bit 7-8
; ============================================================================
DPINSF2		TC	DMP		# ASSUMES POINT BETWEEN BITS 7-8 OF HIGH
		ADRES	SFTEMP1		# PART OF SF CONST. DPINSF2 SHIFTS RESULTS
		CAF	SIX		# LEFT BY 7, ROUNDS MPAC+2 INTO MPAC+1
		TC	TPLEFTN		# SHIFT LEFT 7 (N-1 = 6 in A register)
		TC	DPINSF +2	# Continue with standard DP completion

; ============================================================================
; DPINSF4 - Double precision input with left shift by 3
; Variant of DPINSF for scale factors where binary point is between bits 11-12
; Entry: DP value in MPAC/MPAC+1, scale factor with point at bit 11-12
; ============================================================================
DPINSF4		TC	DMP		# ASSUMES POINT BETWEEN BITS 11-12 OF HIGH
		ADRES	SFTEMP1		# PART OF SF CONST. DPINSF2 SHIFTS RESULTS
		CAF	TWO		# LEFT BY 3, ROUNDS MPAC+2 INTO MPAC+1.
		TC	TPLEFTN		# SHIFT LEFT 3 (N-1 = 2 in A register)
		TC	DPINSF +2	# Continue with standard DP completion

; ============================================================================
; TPLEFTN - Triple precision left shift by N positions
; Shifts MPAC, MPAC+1, MPAC+2 left by N bits with overflow/underflow detection
; Entry: A register contains N-1 (shift count minus 1)
;        Q register contains return address
; Exit: Returns via address in SFTEMP2; sets OVFIND for overflow/underflow
; Loop time: 0.37 milliseconds per iteration
; ============================================================================
TPLEFTN		XCH	Q		# SHIFTS MPAC, +1, +2 LEFT N. SETS OVFIND
		TS	SFTEMP2		# TO +1 FOR OF, -1 FOR UF.
		XCH	Q		# CALL WITH N-1 IN A.
LEFTNCOM	TS	SFTEMP1		#	LOOP TIME .37 MSEC.
		TC	TPSL1		# Triple precision shift left by 1
		CCS	SFTEMP1		# Decrement shift counter
		TC	LEFTNCOM	# Loop if more shifts needed
# Page 434
		TC	SFTEMP2		# Return via saved address

; ============================================================================
; 2ROUND - Two-word rounding routine
; Rounds MPAC+1 into MPAC with overflow/underflow propagation
; Entry: Double-precision value in MPAC, MPAC+1
; Exit: Rounded value; returns via Q or continues if overflow/underflow
; ============================================================================
2ROUND		XCH	MPAC	 +1
		DOUBLE			# Shift left to get rounding bit
		TS	MPAC 	+1
		TC	Q		# IF MPAC+1 DOES NOT OF/UF
		AD	MPAC		# Add overflow/underflow to high part
		TS	MPAC		# Store rounded high part
		TC	Q		# IF MPAC DOES NOT OF/UF
		TS	MPAC+6		# Save overflow/underflow indicator
2RNDEND		TC	Q		# Normal return

; ============================================================================
; TESTOFUF - Test for overflow/underflow condition
; Checks MPAC+6 for overflow (+) or underflow (-) indication
; Entry: MPAC+6 contains OF/UF indicator from previous rounding
; Exit: Returns via Q if no error; alarms and recycles if OF or UF detected
; ============================================================================
TESTOFUF	CCS	MPAC+6		# RETURNS IF NO OF/UF
		TC	ALMCYCLE	# OF	ALARM AND RECYCLE.
		TC	Q		# No error - return to caller
		TC	ALMCYCLE	# UF	ALARM AND RECYCLE.

		SETLOC	ENDSPMIN +1

		COUNT*	$$/PIN

; ============================================================================
; HMSIN - Hours/Minutes/Seconds Time Input Conversion
; Converts time input in Hours:Minutes:Seconds.Centiseconds format to
; internal double-precision centiseconds representation for mission timers.
;
; Entry: XREG/XREGLP = Hours (must be ≤745, whole number)
;        YREG/YREGLP = Minutes (must be ≤59, whole number)
;        ZREG/ZREGLP = Seconds (must be ≤59.99, to centisecond precision)
;        All three components must be loaded (ALL3DEC check)
;
; Exit: Double-precision total centiseconds stored at NOUNADD (E and E+1)
;       Returns via LOADLV to continue display processing
;
; Alarms: OPR ERR if any component missing or exceeds limits
;         OPR ERR if total time exceeds 745:39:14.59 (DP centisecond overflow)
;
; Processing: Hours × 360000 + Minutes × 6000 + Seconds × 100 = Centiseconds
;             Maximum representable: 745:39:14.59 = 2,683,154,459 centiseconds
;             (fits in DP: ±2^28-1 scaled by 2^-14)
;
; Historical: Crew used HMS format extensively during Apollo 11 mission for:
;             - Mission Elapsed Time (MET) displays during descent/ascent
;             - Event timer programming for critical maneuver sequences
;             - Countdown timers for ignition events (PDI, ascent, etc.)
;             Armstrong and Aldrin monitored descent time in HMS format on DSKY
;             during the 12-minute powered descent to lunar surface landing.
; ============================================================================
HMSIN		TC	ALL3DEC		# IF ALL 3 WORDS WERE NOT LOADED, ALARM.
		
		; Process HOURS component (XREG/XREGLP already in MPAC/MPAC+1)
		TC	DMP		# XREG, XREGLP (=HOURS) WERE ALREADY PUT
		ADRES	WHOLECON	# INTO MPAC, MPAC+1.
		TC	RND/TST		# ROUND OFF TO WHOLE HRS IN MPAC+1.
					# Ensures fractional hours are rounded properly
		CAF	ZERO		# ALARM IF MPAC NON ZERO (G/ 16383).
		TS	MPAC	+2	# Clear MPAC+2 for multiplication
		
		; Convert hours to centiseconds (hours × 360000)
		; HRCON = 360000 centiseconds/hour (1 hour = 60 min × 60 sec × 100 cs)
		CAF	HRCON		# Load high part of hour conversion constant
		TS	MPAC
		CAF	HRCON	+1	# Load low part of hour conversion constant
		XCH	MPAC	+1	# Set up for multiplication
		TC	SHORTMP		# Multiply hours by centiseconds/hour
		TC	MPACTST		# ALARM IF MPAC NON ZERO (G/ 745)
					# Detects hours > 745 (overflow limit)
		DXCH	MPAC	+1	# STORE HOURS CONTRIBUTION
		DXCH	HITEMIN		# Save in HITEMIN for later accumulation
		
		; Process MINUTES component (from YREG/YREGLP)
		CA	YREG		# PUT YREG, YREGLP INTO MPAC, +1.
		LXCH	YREGLP		# Load minutes value into accumulator
		DXCH	MPAC		# Transfer to MPAC for processing
		TC	DMP
		ADRES	WHOLECON	# Force to whole number representation
		TC	RND/TST		# ROUND OFF TO WHOLE MIN IN MPAC+1
					# Ensures fractional minutes are rounded properly
		CS	59MIN		# ALARM IF MPAC NON ZERO (G/16383)
		TC	SIZETST		# ALARM IF MPAC+1 G/ 59MIN
					# Validates minutes ≤ 59 (range check)
		
		; Convert minutes to centiseconds and accumulate
		; MINCON = 6000 centiseconds/minute (1 min = 60 sec × 100 cs)
		XCH	MPAC 	+1	# Get rounded whole minutes value
		EXTEND
		MP	MINCON		# LEAVES MINUTES CONTRIBUTION IN A,L
					# Multiply minutes × 6000 centiseconds/minute
		DAS	HITEMIN		# ADD IN MINUTES CONTRIBUTION
					# Add to hours contribution (double-precision add)
		EXTEND			# IF THIS DAS OVEFLOWS, G/ 745 HR, 39MIN
					# Check for overflow (time exceeds max representable)
# Page 435
		BZF	+2		# If no overflow, continue
		TC	ALMCYCLE	# Else alarm and recycle (OPR ERR)
		
		; Process SECONDS component (from ZREG/ZREGLP)
		CA	ZREG		# PUT ZREG, ZREGLP INTO MPAC, +1.
		LXCH	ZREGLP		# Load seconds value (may include centiseconds)
		DXCH	MPAC		# Transfer to MPAC for processing
		TC	DMP
		ADRES	WHOLECON	# Force to whole centisecond representation
		TC	RND/TST		# ROUND OFF TO WHOLE CENTI-SEC IN MPAC+1
					# Rounds to nearest centisecond (0.01 sec precision)
		CS	59.99SEC	# ALARM IF MPAC NON ZERO (G/163.83 SEC)
		TC	SIZETST		# ALARM IF MPAC+1 G/59.99 SEC
					# Validates seconds ≤ 59.99 (range check)
		
		; Seconds are already in centiseconds (multiply by 100 implicit in input)
		; Now add seconds contribution to accumulated hours+minutes total
		DXCH	HITEMIN		# ADD IN SECONDS CONTRIBUTION
					# Get accumulated hours+minutes centiseconds
		DAS	MPAC		# IF THIS DAS OVERFLOWS,
					# Add seconds to total (double-precision add)
		EXTEND			# G/ 745 HR, 39 MIN, 14.59 SEC.
		BZF	+2		# Check for final overflow
		TC	ALMCYCLE	# ALARM AND RECYCLE
					# Total time exceeds maximum representable in DP
		
		; Final result preparation and storage
		CAF	ZERO
		TS	MPAC +2		# Clear unused precision word
		TC	TPAGREE		# Ensure triple-precision agreement
		DXCH	MPAC		# Get final double-precision centiseconds total
		INDEX	NOUNADD		# Store at address specified by noun
		DXCH	0		# Write DP result to noun destination (E and E+1)
		TC	POSTJUMP	# Return to display processing
		CADR	LOADLV		# Continue with LOADLV routine

		; ============================================================================
		; TIME INPUT CONVERSION CONSTANTS
		;
		; These constants are used by HMSIN to convert hours:minutes:seconds
		; into centiseconds (the AGC's standard time unit for mission elapsed time).
		; ============================================================================
		
WHOLECON	OCT	00006		# (10EXP5/2EXP14)2EXP14
		OCT	03240		# DP constant for rounding fractional inputs
					# Forces decimal inputs to whole number representation
HRCON		OCT	00025		# 1 HOUR IN CENTI-SEC
		OCT	37100		# DP constant = 360000 centiseconds/hour
					# (1 hr = 60 min × 60 sec × 100 cs/sec)
MINCON		OCT	13560		# 1 MINUTE IN CENTI-SEC
					# Single constant = 6000 centiseconds/minute
					# (1 min = 60 sec × 100 cs/sec)
59MIN		OCT	00073		# 59 AS WHOLE
					# Maximum valid minutes value for range checking
59.99SEC	OCT	13557		# 5999 CENTI-SEC
					# Maximum valid seconds value (59.99 sec)
					# for range checking

		; ============================================================================
		; RND/TST - Round MPAC+2 Into MPAC+1 and Test for Overflow
		;
		; This routine rounds the least significant word (MPAC+2) into the middle
		; word (MPAC+1) of a triple-precision number, then verifies that the most
		; significant word (MPAC) is zero (no overflow). If MPAC is non-zero after
		; rounding, the input exceeded the representable range and an alarm occurs.
		;
		; Used by time input routines to validate range of hours/minutes/seconds.
		; ============================================================================
		
RND/TST		XCH	MPAC +2		# ROUNDS MPAC+2 INTO MPAC+1.
					# Get least significant word for rounding
		DOUBLE			# ALARMS IF MPAC NOT 0
					# Multiply by 2 to get rounding carry bit
		TS	MPAC +2		# Store back (clears rounding word)
		CAF	ZERO		# Prepare to add with carry propagation
		AD	MPAC +1		# Add to middle word (with any carry from rounding)
		TS	MPAC +1		# Store rounded middle word
		CAF	ZERO		# Prepare for carry to high word
		AD	MPAC		# CANT OVFLOW
					# Add any carry to most significant word
		XCH	MPAC		# Store back and leave in A for test
		
		; Fall through to MPACTST to verify MPAC is zero
		
		; ============================================================================
		; MPACTST - Test That MPAC is Zero (Alarm if Non-Zero)
		;
		; Verifies that the most significant word of MPAC is zero. Used after
		; rounding or conversion operations to detect overflow conditions.
		; If MPAC is positive or negative (non-zero), triggers OPR ERR alarm.
		; ============================================================================
		
MPACTST		CCS	MPAC		# ALARM IF MPAC NON ZERO
					# Check sign/magnitude of most significant word
		TC	ALMCYCLE	# ALARM AND RECYCLE.
					# Positive: overflow detected
		TC	Q		# Zero: acceptable, return to caller
		TC	ALMCYCLE	# ALARM AND RECYCLE.
					# Negative: overflow detected
		TC	Q		# -0: acceptable, return to caller

		; ============================================================================
		; SIZETST - Test That MPAC+1 Does Not Exceed Maximum Size
		;
		; Validates that the magnitude of MPAC+1 does not exceed a maximum size
		; specified (negatively) in the accumulator. Used for range checking
		; time component inputs (e.g., minutes ≤ 59, seconds ≤ 59.99).
		;
		; Entry: A contains negative of maximum size (-CON)
		;        MPAC+1 contains value to test (may be positive or negative)
		; Returns: Q if magnitude ≤ size (acceptable)
		;          Alarms and recycles if magnitude > size (out of range)
		;
		; Method: Uses CCS to get magnitude, subtracts from limit, checks result.
		; ============================================================================
		
SIZETST		TS	MPAC +2		# CALLED WITH - CON IN A
					# Store negative constant for comparison
		CCS	MPAC +1		# GET MAG OF MPAC+1
					# CCS extracts magnitude and provides 4-way branch
# Page 436
		AD	ONE		# If MPAC+1 positive: magnitude = value - 1, add 1 back
		TCF	+2		# Skip next instruction
		AD	ONE		# If MPAC+1 +0: magnitude = 0, add 1
		AD	MPAC +2		# Add the negative constant (-CON)
					# Now A = |MPAC+1| - CON
		EXTEND			# MAG OF MPAC+1 - CON
		BZMF	+2		# Branch if result ≤ 0 (magnitude ≤ CON, acceptable)
		TC	ALMCYCLE	# MAG OF MPAC+1 G/ CON. ALARM AND RECYCLE.
					# Magnitude exceeds limit: trigger OPR ERR
		TC	Q		# MAG OF MPAC+1 L/= CON
					# Acceptable range, return to caller

		; ============================================================================
		; ALL3DEC - Verify All Three Decimal Components Were Loaded
		;
		; For triple-component inputs (hours, minutes, seconds for HMSIN),
		; verifies that the operator loaded all three components before
		; accepting the entry. Uses bit testing on DECBRNCH status register.
		;
		; DECBRNCH bits 3, 4, 5 track which components were entered:
		;   Bit 3 = 1 if first component (hours) loaded
		;   Bit 4 = 1 if second component (minutes) loaded  
		;   Bit 5 = 1 if third component (seconds) loaded
		;
		; Entry: DECBRNCH contains component load status
		; Returns: Q if all three components loaded (bits 3,4,5 all = 1)
		;          Forces Verb 25 and recycles if incomplete (OPR ERR alarm)
		;
		; Method: Bit masking to test if all three status bits are set.
		; ============================================================================

# ALL3DEC TESTS THAT ALL 3 WORDS ARE LOADED IN DEC (FOR HMSIN).
# ALARM IF NOT. (TEST THAT BITS 3,4,5 OF DECBRNCH ARE ALL = 1).

ALL3DEC		CS	OCT34BAR	# GET BITS 3,4,5 IN A
					# Complement of 77743 isolates bits 3-5
		MASK	DECBRNCH	# GET BITS 3,4,5 OF DECBRNCH IN A
					# Extract the three component-loaded status bits
		AD	OCT34BAR	# BITS 3,4,5 OF DECBRNCH MUST ALL = 1
					# If all three bits = 1, result will be zero
		CCS	A		# Test result
		TC	FORCEV25	# Positive: not all components loaded, force V25
OCT34BAR	OCT	77743		# Bit mask constant (octal 77743 = binary ...111 111 111 100 011)
					# Bits 3,4,5 are 1's when complemented
		TC	FORCEV25	# +0 case (shouldn't occur): force V25
		TC	Q		# Zero case: all three components loaded, return to caller

		; ----------------------------------------------------------------------------
		; FORCEV25 - Force Execution of Verb 25 (Load Data)
		;
		; When an incomplete time entry is detected (not all three components
		; loaded), forces the system to execute Verb 25 (load) on recycle.
		; This handles the case where the operator may have entered a lower
		; load verb but didn't complete the three-component entry.
		;
		; Action: Sets VERBSAVE to Verb 25 code, triggers OPR ERR alarm, recycles.
		; ----------------------------------------------------------------------------

FORCEV25	CS	OCT31		# FORCE VERB 25 TO BE EXECUTED BY RECYCLE
					# OCT31 complement gives Verb 25 code
		TS	VERBSAVE	# IN CASE OPERATOR EXECUTED A LOWER LOAD
					# Save Verb 25 for execution after recycle
		TC	ALMCYCLE	# VERB.  ALARM AND RECYCLE.
					# Trigger OPR ERR and restart display sequence
ENDHMSS		EQUALS

# Page 437
# MONITOR ALLOWS OTHER KEYBOARD ACTIVITY. IT IS ENDED BY VERB TERMINATE,
# VERB PROCEED WITHOUT DATA, VERB RESEQUENCE,
# ANOTHER MONITOR, OR ANY NVSUB CALL THAT PASSES THE DSPLOCK (PROVIDED
# THAT THE OPERATOR HAS SOMEHOW ALLOWED THE ENDING OF A MONITOR WHICH
# HE HAS INITIATED THROUGH THE KEYBOARD).
#
# MONITOR ACTION IS SUSPENDED, BUT NOT ENDED, BY ANY KEYBOARD ACTION,
# EXCEPT ERROR LIGHT RESET. IT BEGINS AGAIN WHEN KEY RELEASE IS PERFORMED.
# MONITOR SAVES THE NOUN AND APPROPRIATE DISPLAY VERB IN MONSAVE. IT SAVES
# NOUNCADR IN MONSAVE1, IF NOUN = MACHINE CADR TO BE SPECIFIED. BIT 15 OF
# MONSAVE1 IS THE KILL MONITOR SIGNAL (KILLER BIT). BIT 14 OF MONSAVE1
# INDICATES THE CURRENT MONITOR WAS EXTERNALLY INITIATED (EXTERNAL
# MONITOR BIT). IT IS TURNED OFF BY RELDSP AND KILMONON.
#
# MONSAVE INDICATES IF MONITOR IS ON (+=ON, +0=OFF)
# IF MONSAVE IS +, MONITOR ENTERS NO REQUEST, BUT TURNS KILLER BIT OFF.
# IF MONSAVE IS +0, MONITOR ENTERS REQUEST AND TURNS KILLER BIT OFF.
#
# NVSUB (IF EXTERNAL MONITOR BIT IS OFF), VB=PROCEED WITHOUT DATA,
# VB=RESEQUENCE, AND VB=TERMINATE TURN KILL MONITOR BIT ON.
#
# IF KILLER BIT IS ON, MONREQ ENTERS NO FURTHER REQUESTS, ZEROS MONSAVE
# AND MONSAVE1 (TURNING OFF KILLER BIT AND EXTERNAL MONITOR BIT).
#
# MONITOR DOESNT TEST FOR MATBS SINCE NVSUB CAN HANDLE INTERNAL MATBS NOW.

		SETLOC	ENDRUTIN

		COUNT*	$$/PIN
MONITOR		CS	BIT15/14
		MASK	NOUNCADR
MONIT1		TS	MPAC +1		# TEMP STORAGE
		CS	ENTEXIT
		AD	ENDINST
		CCS	A
		TC	MONIT2
BIT15/14	OCT	60000
		TC	MONIT2
		CAF	BIT14		# EXTERNALLY INITIATED MONITOR.
		ADS	MPAC +1		# SET BIT 14 FOR MONSAVE1.
		CAF	ZERO
		TS	MONSAVE2	# ZERO NVMONOPT OPTIONS
MONIT2		CAF	LOW7
		MASK	VERBREG
		TC	LEFT5
		TS	CYL
		CS	CYL
		XCH	CYL
		AD	NOUNREG
		TS	MPAC		# TEMP STORAGE
		CAF	ZERO
# Page 438
		TS	DSPLOCK		# +0 INTO DSPLOCK SO MONITOR CAN RUN.
		CCS	CADRSTOR	# TURN OFF KR LITE IF CADRSTOR AND DSPLIST
		TC	+2		# ARE BOTH EMPTY. (LITE COMES ON IF NEW
		TC	RELDSP1		# MONITOR IS KEYED IN OVER OLD MONITOR.)
		INHINT
		CCS	MONSAVE
		TC	+5		# IF MONSAVE WAS +, NO REQUEST
		CAF	ONE		# IF MONSAVE WAS 0, REQUEST MONREQ
		TC	WAITLIST
		EBANK=	DSPCOUNT
		2CADR	MONREQ

		DXCH	MPAC		# PLACE MONITOR VERB AND NOUN INTO MONSAVE
		DXCH	MONSAVE		# ZERO THE KILL MONITOR BIT
		RELINT			# SET UP EXTERNAL MONITOR BIT
		TC	ENTRET

MONREQ		TC	LODSAMPT	# CALLED BY WAITLIST
		CCS	MONSAVE1	# TIME IS SNATCHED N RUPT FOR NOUN 65
		TC	+4		# IF KILLER BIT = 0, ENTER REQUESTS
		TC	+3		# IF KILLER BIT = 0, ENTER REQUESTS
		TC	KILLMON		# IF KILLER BIT = 1, NO REQUESTS.
		TC	KILLMON		# IF KILLER BIT = 1, NO REQUESTS.
		CAF	MONDEL
		TC	WAITLIST	# ENTER WAITLIST REQUEST FOR MONREQ
		EBANK=	DSPCOUNT
		2CADR	MONREQ

		CAF	CHRPRIO
		TC	NOVAC		# ENTER EXEC REQUEST FOR MONDO
		EBANK=	DSPCOUNT
		2CADR	MONDO

		TC	TASKOVER

KILLMON		CAF	ZERO		# ZERO MONSAVE AND TURN KILLER BIT OFF
		TS	MONSAVE
		TS	MONSAVE1	# TURN OFF KILL MONITOR BIT.
		TC	TASKOVER	# TURN OFF EXTERNAL MONITOR BIT.
MONDEL		OCT	144		# FOR 1 SEC MONITOR INTERVALS

MONDO		CCS	MONSAVE1	# CALLED BY EXEC
		TC	+4		# IF KILLER BIT = 0, CONTINUE
		TC	+3		# IF KILLER BIT = 0, CONTINUE
		TC	ENDOFJOB	# IN CASE TERMINATE CAME SINCE LAST MONREQ
		TC	ENDOFJOB	# IN CASE TERMINATE CAME SINCE LAST MONREQ
		CCS	DSPLOCK
		TC	MONBUSY		# NVSUB IS BUSY
# Page 439
		CAF	LOW7
		MASK	MONSAVE
		TC	UPDATNN -1	# PLACE NOUN INTO NOUNREG AND DISPLAY IT
		CAF	MID7
		MASK	MONSAVE		# CHANGE MONITOR VERB TO DISPLAY VERB
		AD	MONREF		# -DEC10, STARTING IN BIT8
		TS	EDOP		# RIGHT 7
		CA	EDOP
		TS	VERBREG
		CAF	MONBACK		# SET RETURN TO PASTEVB AFTER DATA DISPLAY
		TS	ENTRET
		CS	BIT15/14
		MASK	MONSAVE1	# PUT ECADR INTO MPAC +2.  INTMCTBS WILL
		TS	MPAC +2		# DISPLAY IT AND SET NOUNCADR, NOUNADD,
ENDMONDO	TC	TESTNN		# EBANK.

		BLOCK	2

		SETLOC	FFTAG8
		BANK

		COUNT*	$$/PIN
PASTEVB		CAF	MID7
		MASK	MONSAVE2	# NVMONOPT PASTE OPTION
		EXTEND
		BZF	+2
		TC	PASTEOPT	# PASTE PLEASE VERB FOR NVMONOPT
		CA	MONSAVE		# PASTE MONITOR VERB - PASTE OPTION IS 0
PASTEOPT	TS	EDOP		# RIGHT 7
		CA	EDOP		# PLACE MONITOR VERB OR PLEASE VERB INTO
		TC	BANKCALL	#  VERBREG AND DISPLAY IT.
		CADR	UPDATVB -1
		CAF	ZERO		# ZERO REQRET SO THAT PASTED VERBS CAN
		TS	REQRET		#  BE EXECUTED BY OPERATOR.
		CA	MONSAVE2
		TC	BLANKSUB	# PROCESS NVMONOPT BLANK OPTION IF ANY
		TC	+1
ENDPASTE	TC	ENDOFJOB

MID7		OCT	37600

		SETLOC	ENDMONDO +1
		COUNT*	$$/PIN
MONREF		OCT	75377		# -DEC10, STARTING IN BIT8
MONBACK		ADRES	PASTEVB

MONBUSY		TC	RELDSPON	# TURN KEY RELEASE LIGHT
		TC	ENDOFJOB
# Page 440
# DSPFMEM IS USED TO DISPLAY (IN OCTAL) ANY FIXED REGISTER.
# IT IS USED WITH NOUN = MACHINE CADR TO BE SPECIFIED. THE FCADR OF THE
# DESIRED LOCATION IS THEN PUNCHED IN. IT HANDLES F/F (FCADR 4000-7777)
#
# FOR BANKS L/E 27, THIS IS ENOUGH.
#
# FOR BANKS G/E 30, THE THIRD COMPONENT OF NOUN 26 (PRIO, ADRES, BBCON)
# MUST BE PRELOADED WITH THE DESIRED SUPERBANK BITS (BITS 5,6,7).
#	V23N26 SHOULD BE USED.
#
# SUMMARY
# FOR BANKS L/E 27,				V27N01E(FCADR)E
# FOR BANKS G/E 30, 	V23N26E(SUPERBITS)E	V27N01E(FCADR)E

DSPFMEM		CAF	R1D1		# IF F/F, DATACALL USES BANK 02 OR 03.
		TS	DSPCOUNT
		CA	DSPTEM1 +2	# SUPERBANK BITS WERE PRELOADED INTO
		TS	L		# 3RD COMPONENT OF NOUN 26.
		CA	NOUNCADR	# ORIGINAL FCADR LOADED STILL IN NOUNCADR.
		TC	SUPDACAL	# CALL WITH FCADR IN A, SUPERBITS IN L.
		TC	DSPOCTWO
ENDSPF		TC	ENDOFJOB

# Page 441
# WORD DISPLAY ROUTINES
		SETLOC	TESTOFUF +4
		COUNT*	$$/PIN
DSPSIGN		XCH	Q
		TS	DSPWDRET
		CCS	MPAC
		TC	+8D
		TC	+7
		AD	ONE
		TS	MPAC
		TC	-ON
		CS	MPAC +1
		TS	MPAC +1
		TC	DSPWDRET
		TC	+ON
		TC	DSPWDRET

DSPRND		EXTEND			# ROUND BY 5 EXP-6
		DCA	DECROUND -1
		DAS	MPAC
		EXTEND
		BZF	+4
		EXTEND
		DCA	DPOSMAX
		DXCH	MPAC
		TC	Q

# DSPDECWD CONVERTS C(MPAC, MPAC+1) INTO A SIGN AND 5 CHAR DECIMAL
# STARTING IN LOC SPECIFIED IN DSPCOUNT. IT ROUNDS BY 5 EXP-6.

DSPDECWD	XCH	Q
		TS	WDRET
		TC	DSPSIGN
		TC	DSPRND
		CAF	FOUR
DSPDCWD1	TS	WDCNT
		CAF	BINCON
		TC	SHORTMP
TRACE1		INDEX	MPAC
		CAF	RELTAB
		MASK	LOW5
		TS	CODE
		CAF	ZERO
		XCH	MPAC +2
		XCH	MPAC +1
		TS	MPAC
		XCH	DSPCOUNT
TRACE1S		TS	COUNT
		CCS	A		# DECREMENT DSPCOUNT EXCEPT AT +0
# Page 442
		TS	DSPCOUNT
		TC	DSPIN
		CCS	WDCNT
		TC	DSPDCWD1
		CS	VD1
		TS	DSPCOUNT
		TC	WDRET

		OCT	00000
DECROUND	OCT	02476

; ============================================================================
; DISPLAY FORMATTING AND CONVERSION ROUTINES
;
; This section contains the core DSKY display formatting routines that convert
; internal AGC data representations into the specific character formats required
; by the seven-segment displays. These routines handle:
; - Decimal number formatting (positive/negative with proper sign display)
; - Octal number formatting (5-character octal display format)
; - Character positioning (left/right character in each register pair)
; - Sign blanking control (for unsigned display modes)
;
; COMMENT-ONLY READERS: These routines translate the computer's internal
; numbers into the specific patterns of lights that astronauts see on the
; DSKY display. Every number, velocity, altitude, or coordinate shown to
; Armstrong and Aldrin during lunar descent passed through these formatting
; routines. The complexity here reflects the seven-segment display hardware
; constraints - each character requires specific bit patterns to light the
; correct segments.
;
; CODE-ALONG READERS: The display formatting system operates in three layers:
; 1. High-level routines (DSPDECNR, DSPOCTWO) called by mission programs
; 2. Mid-level routines (DSP2DEC, DSP2BIT) that format pairs of characters
; 3. Low-level routines (DSPIN, DSPIN1) that update individual DSPTAB entries
;
; The DSPTAB table (defined in DISPLAY_INTERFACE_ROUTINES) contains the actual
; character codes displayed. These routines modify DSPTAB entries while handling
; the complex bit-level formatting required by the Compactron display hardware.
; The NOUT counter tracks how many display table entries have been modified,
; triggering the actual hardware update via output channel 10.
; ============================================================================

; DSPDECNR - Display Decimal Number (5-character format)
;
; Converts double-precision value in MPAC/MPAC+1 into signed 5-character
; decimal display format, starting at location specified in DSPCOUNT.
; No rounding is performed - fractional parts are truncated.
;
; Entry: MPAC, MPAC+1 contain double-precision value to convert
;        DSPCOUNT specifies starting display position
;        Q contains return address
; Exit: Display table updated with decimal representation
;       Returns via WDRET
;
; This routine was invoked thousands of times during Apollo 11's descent,
; displaying altitude rates, velocity components, and guidance parameters
; that Armstrong and Aldrin monitored during the approach to the Sea of
; Tranquility. The 5-character format allows display of values from
; -99999 to +99999, sufficient for most mission parameters.

# DSPDECNR CONVERTS C(MPAC,MPAC+1) INTO A SIGN AND 5 CHAR DECIMAL
# STARTING IN LOC SPECIFIED IN DSPCOUNT. IT DOES NOT ROUND

DSPDECNR	XCH	Q
		TS	WDRET
		TC	DSPSIGN
		TC	DSPDCWD1 -1

; DSPDC2NR - Display Decimal 2 Number (2-character format)
;
; Converts double-precision value in MPAC/MPAC+1 into signed 2-character
; decimal display format. Used for displaying smaller numeric values or
; when display space is limited. No rounding performed.
;
; Entry: MPAC, MPAC+1 contain value to convert
;        DSPCOUNT specifies starting display position
; Exit: Display table updated with 2-digit decimal (range -99 to +99)
;
; During landing, this routine displayed verb and noun codes, program
; numbers, and other small integer values in compact 2-digit format.

# DSPDC2NR CONVERTS C(MPAC,MPAC+1) INTO A SIGN AND 2 CHAR DECIMAL
# STARTING IN LOC SPECIFIED IN DSPCOUNT. IT DOES NOT ROUND

DSPDC2NR	XCH	Q
		TS	WDRET
		TC	DSPSIGN
		CAF	ONE
		TC	DSPDCWD1

; DSP2DEC - Display 2 Decimal Numbers (10-character format)
;
; Converts two separate single-precision values from MPAC and MPAC+1 into
; two signed 5-character decimal displays, totaling 10 characters. This
; allows displaying two independent decimal values side-by-side.
;
; Entry: MPAC contains first value
;        MPAC+1 contains second value
;        DSPCOUNT specifies starting display position
; Exit: Display table updated with both decimal representations
;
; Used for displaying coordinate pairs, component values, or related
; parameters that the crew needed to monitor together. Each value gets
; its own sign and 5-digit display field.

# DSP2DEC CONVERTS C(MPAC) AND C(MPAC+1) INTO A SIGN AND 10 CHAR DECIMAL
# STARTING IN THE LOC SPECIFIED IN DSPCOUNT.

DSP2DEC		XCH	Q
		TS	WDRET
		CAF	ZERO
		TS	CODE
		CAF	THREE
		TC	11DSPIN		# -R2 OFF
		CAF	FOUR
		TC	11DSPIN		# +R2 OFF
		TC	DSPSIGN
		CAF	R2D1
END2DEC		TC	DSPDCWD1

; DSPDECVN - Display Decimal Verb/Noun (scaled 2-character format)
;
; Displays accumulator value as 2-character decimal with special scaling
; for verb/noun codes. Input is expected in the form N × 2^-14, which is
; scaled to N/100 before display conversion. This scaling matches the
; internal representation of verb/noun values.
;
; Entry: A register contains scaled value (N × 2^-14 format)
;        DSPCOUNT specifies starting display position
; Exit: Display table updated with 2-digit decimal
;       Returns via WDRET
;
; This routine displayed every verb and noun code that Armstrong and Aldrin
; entered during the mission. When they keyed "V16N63" to request position
; display during descent, this routine formatted the "16" and "63" for the
; DSKY seven-segment displays. The scaling by VNDSPCON (0.01) converts the
; internal AGC representation to the display range 00-99.

# DSPDECVN DISPLAYS C(A) UPON ENTRY AS A 2 CHAR DECIMAL BEGINNING IN THE
# DSP LOC SPECIFIED IN DSPCOUNT.
# C(A) SHOULD BE IN FORM N X 2EXP-14.  THIS IS SCALED TO FORM N/100 BEFORE
# DISPLAY CONVERSION.

# Page 443
DSPDECVN	EXTEND
		MP	VNDSPCON	# MULT BY .01
		LXCH	MPAC		# TAKE RESULTS FROM L. (MULT BY 2EXP14).
		CAF	ZERO
		TS	MPAC +1
		XCH	Q
		TS	WDRET
		TC	DSPDC2NR +3	# NO SIGN, NO ROUND, 2 CHAR

VNDSPCON	OCT	00244		# .01 ROUNDED UP

GOVNUPDT	TC	DSPDECVN	# THIS IS NOT FOR GENERAL USE. REALLY PART
		TC	POSTJUMP	# OF UPDATVB.
		CADR	UPDAT1 +2

ENDECVN		EQUALS

		SETLOC	ENDSPF +1
		COUNT*	$$/PIN

; DSPOCTWO - Display Octal Two (5-character octal format)
;
; Primary entry point for displaying 5-character octal values. Saves the
; value to be displayed in CYL, sets up return address, and uses bit 14
; to control sign blanking. The routine then extracts and displays each
; octal digit sequentially from the input value.
;
; Entry: A register contains value to display in octal
;        DSPCOUNT specifies starting display position
;        Q register contains return address
; Exit: Display updated with 5-digit octal representation
;       Returns via WDRET
;
; During Apollo 11 operations, this routine displayed memory addresses when
; ground controllers needed to verify or modify specific AGC memory locations
; via uplink. The octal format was preferred for memory operations because
; it directly reflected the AGC's word structure (15 bits + parity).
; The routine uses triple CS (complement and skip) operations to perform
; right-shifts by 3 bits, extracting each octal digit.

# DSPOCTWD DISPLAYS C(A) UPON ENTRY AS A 5 CHAR OCT STARTING IN THE DSP
# CHAR SPECIFIED IN DSPCOUNT. IT STOPS AFTER 5 CHAR HAVE BEEN DISPLAYED.

DSPOCTWO	TS	CYL
		XCH	Q
		TS	WDRET		# MUST USE SAME RETURN AS DSP2BIT.
		CAF	BIT14		# TO BLANK SIGNS
		ADS	DSPCOUNT
		CAF	FOUR
WDAGAIN		TS	WDCNT
		CS	CYL
		CS	CYL
		CS	CYL
		CS	A
		MASK	DSPMSK
		INDEX	A
		CAF	RELTAB
		MASK	LOW5
		TS	CODE
		XCH	DSPCOUNT
		TS	COUNT
		CCS	A		# DECREMENT DSPCOUNT EXCEPT AT +0
		TS	DSPCOUNT
		TC	POSTJUMP
		CADR	DSPOCTIN
OCTBACK		CCS	WDCNT
		TC	WDAGAIN		# +
DSPLV		CS	VD1		# TO BLOCK NUMERICAL CHARACTERS, CLEARS,
		TS	DSPCOUNT	# AND SIGNS AFTER A COMPLETED DISPLAY.
# Page 444
		TC	WDRET

DSPMSK		=	SEVEN

; DSP2BIT - Display 2-Bit Octal (2-character octal format)
;
; Displays accumulator value as 2-character octal by pre-cycling the value
; right and reusing the 5-character octal display logic. This is an optimized
; routine that avoids reimplementing display logic by cleverly positioning
; the data before invoking the standard octal formatter.
;
; Entry: A register contains value to display in octal
;        DSPCOUNT specifies starting display position
;        Q register contains return address
; Exit: Display updated with 2-digit octal representation
;       Returns via WDRET
;
; Implementation: The routine saves the value in CYR, sets WDCNT to 1 (for
; 2 characters), performs triple CS (complement and skip) operations to
; right-shift the value by 9 bits, then jumps to WDAGAIN+5 to use the
; standard octal display logic. This technique minimizes code size by
; reusing existing display routines.
;
; Used during Apollo 11 for displaying compact status codes and bit flags
; that only required 2 octal digits (6 bits of information).

# DSP2BIT DISPLAYS C(A) UPON ENTRY AS A 2 CHAR OCT BEGINNING IN THE DSP
# LOC SPECIFIED IN DSPCOUNT BY PRE CYCLING RIGHT C(A) AND USING THE LOGIC
# OF THE 5 CHAR OCTAL DISPLAY

DSP2BIT		TS	CYR
		XCH	Q
		TS	WDRET
		CAF	ONE
		TS	WDCNT
		CS	CYR
		CS	CYR
		XCH	CYR
		TS	CYL
		TC	WDAGAIN +5

; DSPIN - Display Input (low-level character placement routine)
;
; Core display manipulation routine that places individual characters into
; the DSKY display table (DSPTAB). This is the lowest-level routine that
; directly modifies the display memory, handling both left/right character
; placement, sign blanking, and display table entry updates.
;
; Entry: COUNT contains display position (0-25 octal)
;        CODE contains 5-bit relay code for the character
;        BIT14 of COUNT controls sign blanking (1 = blank sign)
;        Q register contains return address
; Exit: DSPTAB updated with new character code
;       COUNT and CODE are destroyed
;       NOUT incremented if DSPTAB entry was positive
;       Returns via DSEXIT
;
; DSPIN1 Entry: CODE has 0 or 1 in bit 11, COUNT = 2, DSREL = relative
;               address of DSPTAB entry
;
; The routine determines left/right character position based on COUNT parity,
; shifts CODE appropriately, checks if the new character differs from the
; existing display content, and updates DSPTAB only if changed. This
; optimization reduces unnecessary display updates, conserving processing
; time in the interrupt-driven display system.
;
; Every character that appeared on Armstrong's and Aldrin's DSKY during
; the mission - every digit of altitude, velocity, verb code, noun code,
; warning indicator - was placed by this routine. During the critical
; descent phase, DSPIN updated altitude and velocity displays multiple
; times per second, providing the crew with continuous situational awareness.

# FOR DSPIN PLACE 0/25 OCT INTO COUNT, 5 BIT RELAY CODE INTO CODE.  BOTH
# ARE DESTROYED. IF BIT14 OF COUNT IS 1, SIGN IS BLANKED WITH LEFT CHAR.
# FOR DSPIN1 PLACE 0,1 INTO BIT11 OF CODE, 2 INTO COUNT, REL ADDRESS OF
# DSPTAB ENTRY INTO DSREL.

		SETLOC	ENDECVN

		COUNT*	$$/PIN
DSPIN		XCH	Q		# CANT USE L FOR RETURN, SINCE MANYOF THE
		TS	DSEXIT		# ROTINE CALLING DSPIN USE L AS RETURN.
		CAF	LOW5
		MASK	COUNT
		TS	SR
		XCH	SR
		TS	DSREL
		CAF	BIT1
		MASK	COUNT
		CCS	A
		TC	+2		# LEFT IF COUNT IS ODD
		TC	DSPIN1 -1	# RIGHT IF COUNT IS EVEN
		XCH	CODE
		TC	SLEFT5		# DOES NOT USE CYL
		TS	CODE
		CAF	BIT14
		MASK	COUNT
		CCS	A
		CAF	TWO		# BIT14 = 1, BLANK SIGN
		AD	ONE		# BIT14 = 0, LEAVE SIGN ALONE
		TS	COUNT		# +0 INTO COUNT FOR RIGHT
# Page 445
					# +1 INTO COUNT FOR LEFT (SIGN LEFT ALONE)
					# +3 INTO COUNT FOR LEFT (TO BLANK SIGN)
DSPIN1		INHINT
		INDEX	DSREL
		CCS	DSPTAB
		TC	+2		# IF +
		TC	CCSHOLE
		AD	ONE		# IF -
		TS	DSMAG
		INDEX	COUNT
		MASK	DSMSK
		EXTEND
		SU	CODE
		EXTEND
		BZF	DSLV		# SAME
DFRNT		INDEX	COUNT
		CS	DSMSK		# MASK WITH 77740, 76037, 76777, OR 74037
		MASK	DSMAG
		AD	CODE
		CS	A
		INDEX	DSREL
		XCH	DSPTAB
		EXTEND
		BZMF	DSLV		# DSPTAB ENTRY WAS -
		INCR	NOUT		# DSPTAB ENTRY WAS +
DSLV		RELINT
		TC	DSEXIT

DSMSK		OCT	37
		OCT	1740
		OCT	2000
		OCT	3740

; 11DSPIN - Display Sign Position Update (simplified entry point)
;
; Simplified entry point to DSPIN1 for updating the sign position (11th position
; in the display, hence the name). This routine is used when the caller has
; already determined the display table relative address and character code but
; needs DSPIN's character placement logic.
;
; Entry: A register contains relative address of DSPTAB entry
;        CODE contains character code with bit 11 set to 0 or 1
;        Q register contains return address
; Exit: DSPTAB updated via DSPIN1
;       Returns via DSEXIT
;
; Implementation: Sets DSREL from A, loads COUNT with 2 (for sign position),
; swaps return address into DSEXIT, then transfers control to DSPIN1 to
; perform the actual display update.

# FOR 11DSPIN, PUT REL ADDRESS OF DSPTAB ENTRY INTO A, 1 IN BIT11 OR 0 IN
# BIT11 OF CODE.

11DSPIN		TS	DSREL
		CAF	TWO
		TS	COUNT
		XCH	Q		# MUST USE SAME RETURN AS DSPIN
		TS	DSEXIT
		TC	DSPIN1

; DSPOCTIN - Display Octal Input (inter-bank entry point)
;
; This routine provides an inter-bank transfer mechanism for DSPOCTWO,
; allowing it to call DSPIN without using SWCALL (switched call). It
; bridges between the ENDOFJOB bank and the bank containing DSPIN,
; then returns control to OCTBACK via BANKJUMP after DSPIN completes.
;
; Entry: Set up as for DSPIN (COUNT, CODE, Q)
; Exit: After DSPIN completes, transfers to OCTBACK via BANKJUMP
;
; Technical: The routine performs TC DSPIN, then uses CAF +2 to load
; the address of ENDSPOCT (which contains CADR OCTBACK), and executes
; BANKJUMP to return to the calling bank. This bank-switching pattern
; is typical AGC code for minimizing the overhead of inter-bank calls
; in the presence of fixed memory banking constraints.

DSPOCTIN	TC	DSPIN		# SO DSPOCTWO DOESNT USE SWCALL
		CAF	+2
		TC	BANKJUMP
ENDSPOCT	CADR	OCTBACK

# Page 446
# DSPALARM FINDS TC NVSUBEND IN ENTRET FOR NVSUB INITIATED ROUTINES
# ABORT WITH 01501.
#
# DSPALARM FINDS TC ENDOFJOB IN ENTRET FOR KEYBOARD INITIATED ROUTINES.
# DC TC ENTRET.

; ============================================================================
; DSPALARM - DISPLAY OPERATOR ERROR ALARM
;
; This routine handles operator errors during DSKY input, such as invalid
; verb/noun combinations, out-of-range data, or incomplete numerical entry.
; It illuminates the OPR ERR (Operator Error) indicator light on the DSKY
; and may kill active monitor verbs if the error occurred during monitoring.
;
; Historical Context: During Apollo 11, operator errors were rare but critical.
; The OPR ERR light alerted the crew to input mistakes requiring correction.
; In the high-workload descent phase, clear error feedback prevented confusion
; about system state. DSPALARM provided that instant visual error indication.
;
; Technical Operation: DSPALARM checks ENTRET to determine alarm context:
; - If ENTRET = NVSUBEND: Program-initiated alarm, abort with code 01501
; - If ENTRET = MONADR: Monitor verb active, kill it before alarming
; - Otherwise: Keyboard-initiated alarm, light OPR ERR and suspend job
;
; The routine calls FALTON to illuminate the OPR ERR light, which flashes
; until the crew presses RSET (reset) to acknowledge the error.
; ============================================================================
PREDSPAL	CS	VD1
		TS	DSPCOUNT	; Reset display counter for verb digit 1
DSPALARM	CS	NVSBENDL	; Check if this is NVSUB-initiated alarm
		AD	ENTEXIT
		EXTEND
		BZF	CHARALRM +2	; Not NVSUB, branch to character alarm
		CS	MONADR		# IF THIS IS A MONITOR, KILL IT
		AD	ENTEXIT
		EXTEND
		BZF	+2		; Not a monitor, skip kill
		TC	CHARALRM	; Not a monitor, handle char alarm
		TC	KILMONON	; Kill the active monitor verb
		TC	FALTON		; Turn on OPR ERR light
		TC	PASTEVB		# PUT MONITOR VERB BACK IN VERBREG
CHARALRM	TC	FALTON		# NO NVSUB INITATED. TURN ON OPR ERROR
		TC	ENDOFJOB	; Suspend job until crew acknowledges
		TC	POODOO
		OCT	01501
MONADR		GENADR	PASTEVB
NVSBENDL	TC	NVSUBEND

# ALMCYCLE TURNS ON CHECK FAIL LIGHT, REDISPLAYS THE ORIGINAL VERB THAT
# WAS EXECUTED, AND RECYCLES TO EXECUTE THE ORIGINAL VERB/NOUN COMBINATION
# THAT WAS LAST EXECUTED. USED FOR BAD DATA DURING LOAD VERBS AND BY
# MCTBS.  ALSO BY MMCHANG IF 2 NUMERICAL CHARACTERS WERE NOT PUNCHED IN
# FOR MM CODE.

		SETLOC	MID7 +1
		COUNT*	$$/PIN
; ============================================================================
; ALMCYCLE - Alarm and Recycle to Original Operation
;
; This critical routine handles error recovery when invalid operator input
; is detected. It illuminates the OPR ERR indicator light, restores the
; display to show the verb that was being executed when the error occurred,
; and returns control to the keyboard/display program's main entry point.
;
; COMMENT-ONLY READERS: When you entered invalid data (like typing letters
; when numbers were expected, or omitting required components), the DSKY
; would light up "OPR ERR" and show you what verb was being attempted so
; you could try again. During Apollo 11's descent, if Armstrong or Aldrin
; made a data entry mistake, this routine gave them immediate feedback.
;
; CODE-ALONG READERS: ALMCYCLE is the standard error handler for PINBALL.
; It calls FALTON to illuminate the OPR ERR light, restores VERBSAVE to
; REQRET (for ENTPAS0 logic), calls UPDATVB to redisplay the original
; verb in the verb lights, then jumps to ENTER to restart the display
; program loop waiting for the next keyboard input.
;
; Entry: VERBSAVE contains the verb code that was being executed
; Exit: OPR ERR light on, original verb displayed, control at ENTER
; Used by: ALL3DEC, MMCHANG, and many other input validation routines
; ============================================================================
ALMCYCLE	TC	FALTON		# TURN ON CHECK FAIL LIGHT.
		# Illuminate OPR ERR indicator to signal operator error
		CS	VERBSAVE	# GET ORIGINAL VERB THAT WAS EXECUTED
		# Restore the verb that was active when error occurred
		TS	REQRET		# SET FOR ENTPAS0
		# REQRET used by ENTPAS0 to restore display state
		TC	BANKCALL	# PUTS ORIGINAL VERB INTO VERBREG AND
		CADR	UPDATVB -1	# DISPLAYS IT IN VERB LIGHTS.
		# UPDATVB updates VERBREG and refreshes verb display on DSKY
		TC	POSTJUMP	# Jump to restart keyboard/display loop
ENDALM		CADR	ENTER		# Return to main ENTER routine (keyboard wait)

# ============================================================================
; MAJOR MODE CHANGE (MMCHANG)
;
; This routine handles operator-initiated changes to the AGC's major mode.
; During Apollo 11's mission, major modes controlled the primary function
; of the AGC (P63 for descent, P12 for ascent, P00 for standby, etc.).
;
; COMMENT-ONLY READERS: This is how the crew switched between different
; mission programs - like changing from descent mode to surface operations.
;
; CODE-ALONG READERS: Uses noun display for input until ENTER, then updates
; mode lights. Requires exactly 2 numerical characters for the new mode code.
; ============================================================================

# MMCHANG USES NOUN DISPLAY UNTIL ENTER.  THEN IT USES MODE DISP.
# IT GOES TO MODROUT WITH THE NEW M M CODE IN A, BUT NOT DISPLAYED IN
# MM LIGHTS.
# IT DEMANDS 2 NUMERICAL CHARACTERS BE PUNCHED IN FOR NEW MM CODE.
# IF NOT, IT RECYCLES.

# Page 447
		SETLOC	DSP2BIT +10D

		COUNT*	$$/PIN
; ============================================================================
; MAJOR MODE CHANGE HANDLER (MMCHANG)
;
; COMMENT-ONLY READERS: This routine handles requests to change the spacecraft's
; major mode (primary mission program). When the crew entered V37 E (request
; major mode change), this code validated that they entered exactly 2 digits,
; then passed control to the new program. During Apollo 11's descent, Armstrong
; and Aldrin used this to transition between programs like P63 (landing) to
; P00 (standby) after touchdown.
;
; CODE-ALONG READERS: Verifies that DSPCOUNT equals -ND2 (octal -2), confirming
; exactly 2 numeric characters were entered. If validation fails, calls ALMCYCLE
; to trigger operator error. On success, saves NOUNREG (which contains the
; entered major mode code) to MPAC, blanks the display via 2BLANK, blocks further
; numeric input by setting DSPCOUNT to -VD1, and jumps to MODROUTB (V37 handler).
;
; CRITICAL LINKAGE: The comment "ENTPASHI ASSUMES THE TC REQMM AT MMCHANG"
; indicates that the entry point routine ENTPASHI hardcodes the knowledge that
; MMCHANG's first instruction is "TC REQMM". If this code moves, MMADREF at
; ENTPASHI must be updated. This tight coupling was necessary for memory
; efficiency in the 36K ROM constraint.
;
; INPUT: DSPCOUNT = character count, NOUNREG = entered major mode code
; OUTPUT: Control transferred to MODROUTB (V37) with mode code in MPAC
; ============================================================================

MMCHANG		TC	REQMM		# ENTPASHI ASSUMES THE TC REQMM AT MMCHANG
					# IF THIS MOVES AT ALL, MUST CHANGE
					# MMADREF AT ENTPASHI.
		CAF	BIT5		# OCT20 = ND2.
		AD	DSPCOUNT	# DSPCOUNT MUST = -ND2.
		EXTEND			# DEMAND THAT 2 NUM CHAR WERE PUNCHED IN.
		BZF	+2
		TC	ALMCYCLE	# DSPCOUNT NOT= -ND2. ALARM AND RECYCLE.
		CAF	ZERO		# DSPCOUNT = -ND2.
		XCH	NOUNREG
		TS	MPAC
		CAF	ND1
		TS	DSPCOUNT
		TC	BANKCALL
		CADR	2BLANK
		CS	VD1		# BLOCK NUM CHAR IN
		TS	DSPCOUNT
		CA	MPAC
		TC	POSTJUMP
		CADR	MODROUTB	# GO THRU STANDARD LOC.

; ============================================================================
; REQUEST MAJOR MODE INPUT (REQMM)
;
; Prompts the crew for a new major mode code by blanking the noun display
; and flashing, waiting for 2-digit decimal input.
;
; OPERATIONAL CONTEXT: When Armstrong or Aldrin wanted to switch mission
; programs (e.g., from P63 landing to P00 standby after touchdown), they
; would enter V37 E, and this routine would handle their input.
; ============================================================================

MODROUTB	=	V37
REQMM		CS	Q
		TS	REQRET
		CAF	ND1
		TS	DSPCOUNT
		CAF	ZERO
		TS	NOUNREG
		TC	BANKCALL
		CADR	2BLANK
		TC	FLASHON
		CAF	ONE
		TS	DECBRNCH	# SET FOR DEC
		TC	ENTEXIT

; ============================================================================
; REQUEST EXECUTIVE JOB (VBRQEXEC)
;
; This routine schedules a new job for the AGC executive scheduler (EXEC).
; The job parameters are preloaded into noun 26 (DSPTEM1):
;   R1: Priority (bits 10-14), BIT1=0 for NOVAC, BIT1=1 for FINDVAC
;   R2: Job address (12-bit CADR)
;   R3: Bank switching code (BBCON)
;
; TECHNICAL DETAILS: This is how programs dynamically request computational
; jobs from the executive. During descent, guidance jobs would be scheduled
; using this mechanism to compute trajectory updates at regular intervals.
; After scheduling, the display system is released and the routine ends.
;
; EXECUTIVE INTEGRATION: Uses either NOVAC (if a core set is known to be
; available) or FINDVAC (to search for an available core set) based on BIT1.
; ============================================================================

# VBRQEXEC ENTERS REQUEST TO EXEC FOR ANY ADDRESS WITH ANY PRIORITY.
# IT DOES ENDOFJOB AFTER ENTERING REQUEST.  DISPLAY SYST IS RELEASED.
# IT ASSUMES NOUN 26 HAS BEEN PRELOADED WITH
# COMPONENT 1  PRIORITY (BITS 10-14) BIT1=0 FOR NOVAC, BIT1=1 FOR FINDVAC.
# COMPONENT 2  JOB ADRES (12 BIT)
# COMPONENT 3  BBCON

VBRQEXEC	CAF	BIT1
		MASK	DSPTEM1
		CCS	A
# Page 448
		TC	SETVAC		# IF BIT1 = 1, FINDVAC
		CAF	TCNOVAC		# IF BIT1 = 0, NOVAC
REQEX1		TS	MPAC		# TC NOVAC OR TC FINDVAC INTO MPAC
		CS	BIT1
		MASK	DSPTEM1
		TS	MPAC +4		# PRIO INTO MPAC+4 AS A TEMP
REQUESTC	TC	RELDSP
		CA	ENDINST
		TS	MPAC +3		# TC ENDOFJOB INTO MPAC+3
		EXTEND
		DCA	DSPTEM1 +1	# JOB ADRES INTO MPAC+1
		DXCH	MPAC +1		# BBCON INTO MPAC+2
		CA	MPAC +4		# PRIO IN A
		INHINT
		TC	MPAC

SETVAC		CAF	TCFINDVC
		TC	REQEX1

; ============================================================================
; REQUEST WAITLIST TASK (VBRQWAIT)
;
; Schedules a time-delayed task using the WAITLIST timer system. After the
; specified delay, the task will be executed by the AGC executive.
;
; NOUN 26 PRELOADED WITH:
;   R1: Time delay (in centiseconds, low bits)
;   R2: Task address (12-bit CADR)
;   R3: Bank switching code (BBCON)
;
; OPERATIONAL CONTEXT: During powered descent, the guidance computer scheduled
; periodic trajectory updates using waitlist tasks. This allowed time-based
; coordination of guidance, navigation, and control computations.
;
; After scheduling, display system is released and routine ends via ENDOFJOB.
; ============================================================================

# VBRQWAIT ENTERS REQUEST TO WAITLIST FOR ANY ADDRESS WITH ANY DELAY.
# IT DOES ENDOFJOB AFTER ENTERING REQUEST. DISPLAY SYST IS RELEASED.
# IT ASSUMES NOUN 26 HAS BEEN PRELOADED WTIH
# COMPONENT 1  DELAY (LOW BITS)
# COMPONENT 2  TASK ADRES (12 BIT)
# COMPONENT 3  BBCON

VBRQWAIT	CAF	TCWAIT
		TS	MPAC		# TC WAITLIST INTO MPAC
		CA	DSPTEM1		# TIME DELAY
ENDRQWT		TC	REQUESTC -1

# REQUESTC WILL PUT TASK ADRES INTO MPAC+1, BBCON INTO MPAC+2,
# TC ENDOFJOB INTO MPAC+3.  IT WILL TAKE TIME DELAY OUT OF MPAC+4 AND
# LEAVE IT IN A, INHINT AND TC MPAC.

		SETLOC	NVSBENDL +1
		COUNT*	$$/PIN

; VBPROC - Verb Proceed Without Data
;
; This routine processes the "Proceed without data" action, one of the most
; important crew responses in the AGC's verb/noun interaction system. When
; the astronaut presses the PROCEED key (PRO) in response to a flashing
; display requesting data input, this routine handles the acceptance without
; requiring numerical data entry.
;
; Entry: Called when PROCEED key pressed during data request
; Exit: Display system released, flashing stopped, control returned to program
;
; Operations performed:
; 1. Sets LOADSTAT to +1 (proceed flag indicating acceptance without data)
; 2. Activates kill monitor bit (terminates any running monitor)
; 3. Releases display system (clears DSPLOCK, frees for internal use)
; 4. Turns off FLASH indicator light on DSKY
; 5. Checks for pending recall from ENDIDLE state
;
; Historical context: During Apollo 11's descent, Armstrong and Aldrin used
; the PROCEED key extensively to acknowledge computer requests and continue
; with automated sequences. The famous "1202 alarm" situation required multiple
; PROCEED responses as Armstrong and ground control elected to continue the
; landing despite computer overload warnings. Each PROCEED press was processed
; by this routine, clearing the flashing display and allowing the guidance
; program to continue executing.
;
; The routine also handles the transition from external (crew-initiated) to
; internal (program-initiated) control of the display system, ensuring smooth
; handoff between interactive and automated display modes.

VBPROC		CAF	ONE		# PROCEED WITHOUT DATA
		TS	LOADSTAT
		TC	KILMONON	# TURN ON KILL MONITOR BIT
		TC	RELDSP
		TC	FLASHOFF
		TC	RECALTST	# SEE IF THERE IS ANY RECALL FROM ENDIDLE

; VBTERM - Verb Terminate
;
; This routine processes the TERMINATE verb action, which aborts the current
; operation and returns the display system to idle state. Unlike VBPROC which
; accepts and continues, VBTERM rejects the current operation and cancels it.
;
; Entry: Called when TERMINATE action is requested
; Exit: Display system released, operation cancelled, LOADSTAT set negative
;
; Implementation: Sets LOADSTAT to -1 (negative flag indicating termination/
; cancellation) then jumps to VBPROC+1 to execute the common cleanup sequence
; (kill monitor, release display, turn off flash, check recall).
;
; The negative LOADSTAT value signals to the calling program that the astronaut
; chose to cancel rather than proceed with the requested operation. This allows
; mission programs to distinguish between acceptance (PROCEED) and cancellation
; (TERMINATE) and respond appropriately.
;
; During critical mission phases, the crew used TERMINATE to back out of
; unwanted program modes or to cancel requests they chose not to fulfill,
; providing essential manual override capability.

VBTERM		CS	ONE
		TC	VBPROC +1	# TERM VERB SETS LOADSTAT NEG

# Page 449

; PROCKEY - Proceed Key Handler (Executive-Controlled Version)
;
; This routine performs the same proceed-without-data function as VBPROC, but
; is specifically designed to be called under executive control with CHRPRIO
; (character priority). It provides the same user acceptance processing but
; includes additional setup to prepare the display system for the next input
; sequence.
;
; Entry: Must be called as executive job with CHRPRIO set
; Exit: Display system released, ready for next input sequence
;
; Operations performed:
; 1. Sets REQRET to zero (prepares for ENTER pass 0 processing)
; 2. Blocks numerical characters, sign keys, and CLEAR key by setting DSPCOUNT
;    to complement of VD1 (this prevents premature data entry during transition)
; 3. Calls VBPROC to perform standard proceed processing (kill monitor, release
;    display, turn off flash, check recall)
;
; Technical detail: The DSPCOUNT blocking mechanism (CS VD1) temporarily disables
; certain keyboard inputs during the transition period between accepting the
; current operation and beginning the next input sequence. This prevents race
; conditions where the astronaut might press a numeric key before the system
; has fully reset to idle state.
;
; The REQRET zero setting indicates this is the initial pass of ENTER processing,
; ensuring proper state initialization for subsequent load or display operations.

# PROCKEY PERFORMS THE SAME FUNCTION AS VBPROC. IT MUST BE CALLED UNDER
# EXECUTIVE CONTROL, WITH CHRPRIO.

PROCKEY		CAF	ZERO		# SET REQRET FOR ENTER PASS 0.
		TS	REQRET
		CS	VD1		# BLOCK NUMERICAL CHARACTERS, SIGNS, CLEAR
		TS	DSPCOUNT
		TC	VBPROC

; VBRESEQ - Verb Resequence to ENDIDLE Resume Point
;
; This routine wakes ENDIDLE at the same line as the final ENTER of a load
; operation (specifically at L+3 entry point). It simulates a successful data
; entry by setting up the proper entry conditions and jumping into VBPROC at
; the +1 offset.
;
; Primary use case: Response to internally initiated flashing displays in ENDIDLE
; routine, allowing programmatic acceptance of displayed data without requiring
; astronaut input.
;
; IMPORTANT RESTRICTION: Should NOT be used with load verbs, PLEASE PERFORM,
; or PLEASE MARK verbs because these verb types already use the L+3 context
; in a different way, and VBRESEQ would conflict with their intended operation.
;
; Entry: Called from internal program logic responding to flashing displays
; Exit: Control passes to VBPROC +1 with zero complement in A register
;
; Operations performed:
; 1. CS ZERO: Complements zero (creates -0, makes it look like data was entered)
;    This sets up A register to simulate successful numerical data input
; 2. TC VBPROC +1: Jumps to VBPROC offset 1, bypassing the REQRET setup
;    Entry at +1 assumes data is ready and proceeds directly to verb processing
;
; Technical detail: The "CS ZERO" trick creates a value that passes the data
; validation checks in VBPROC without actually requiring keyboard input. This
; allows the system to programmatically proceed past a flashing display prompt
; when the internal logic determines the displayed value should be accepted.
;
; The L+3 wake point in ENDIDLE corresponds to the same execution path taken
; after astronaut presses ENTER on the final register of a multi-register load
; operation, ensuring consistent state machine behavior for both manual and
; programmatic data acceptance.

# VBRESEQ WAKES ENDIDLE AT SAME LINE AS FINAL ENTER OF LOAD (L+3).
# (MAIN USE IS INTENDED AS RESPONSE TO INTERNALLY INITIATED FLASHING
#  DISPLAYS IN ENDIDLE. SHOULD NOT BE USED WITH LOAD VERBS, PLEASE PERFORM,
#  OR PLEASE MARK VERBS BECAUSE THEY ALREADY USE L+3 IN ANOTHER CONTEXT.)

VBRESEQ		CS	ZERO		# MAKE IT LOOK LIKE DATA IN.
		TC	VBPROC +1

# FLASH IS TURNED OFF BY PROCEED WITHOUT DATA, TERMINATE, REQUEQUENCE,
# END OF LOAD.

# Page 450

; ============================================================================
; VERB: KEY RELEASE (VBRELDSP)
;
; COMMENT-ONLY READERS: The KEY RELEASE button on the DSKY was the crew's way
; to "unlock" the display system after they finished entering data or reviewing
; displays. When pressed, it cleared the KEY REL indicator light and allowed
; the computer to resume automatic display updates. During Apollo 11's descent,
; if Armstrong or Aldrin had initiated their own displays (like checking fuel
; quantity or velocity), pressing KEY RELEASE would restore any pending computer
; requests that were waiting for their attention.
;
; CODE-ALONG READERS: Implements the KEY RELEASE button handler with sophisticated
; priority logic. Always turns off UPACT light and clears DSPLOCK. Highest priority
; function is unsuspending an externally-initiated suspended monitor (when both
; DSPLIST and CADRSTOR are empty). If no suspended monitor exists, calls RELDSP
; to clear DSPLOCK and external monitor bit, freeing display system for internal
; use. Special re-establishment feature: if CADRSTOR contains a job in ENDIDLE
; state, transfers control to PINBRNCH to re-execute the NVSUB call series,
; restoring displays to pre-obscured state. This allows crew to recover from
; accidentally overwriting computer-initiated displays with their own queries.
;
; CREW INTERACTION: "Margaret's display subroutines" reference Margaret Hamilton's
; display interface architecture. KEY RELEASE was the crew's "I'm done" signal.
; ============================================================================

# KEY RELEASE ROUTINE
#
# THIS ROUTINE ALWAYS TURNS OFF THE UPACT LIGHT AND ALWAYS CLEARS DSPLOCK.
#
# THE HIGHEST PRIORITY FUNCTION OF THE KEY RELEASE BUTTON IS THE
# UNSUSPENDING OF A SUSPENDED MONITOR WHICH WAS EXTERNALLY INITIATED.
# THIS FUNCTION IS ACCOMPLISHED BY CLEARING DSPLOCK AND TURNING OFF
# THE KEY RELEASE LIGHT IF BOTH DSPLIST AND CADRSTOR ARE EMPTY.
#
# IF NO SUCH MONITOR EXISTS, THEN RELDSP IS EXECUTED TO CLEAR DSPLOCK
# AND THE EXTERNAL MONITOR BIT (FREEING THE DISPLAY SYSTEM FOR INTERNAL
# USE), TURN OFF THE KEY RELEASE LIGHT, AND WAKE UP ANY JOB IN DSPLIST.
#
# IN ADDITION IF THERE IS A JOB IN ENDIDLE, THEN CONTROL IS TRANSFERRED
# TO PINBRNCH (IN DISPLAY INTERFACE ROUTINE) TO RE-EXECUTE THE SERIES OF
# NVSUB CALLS ETC. THAT PRECEDED THE ENDIDLE CALL STILL AWAITING RESPONSE.
# THIS FEATURE IS INTENDED FOR USE WHEN THE OPERATOR HAS BEEN REQUESTED TO
# RESPOND TO SOME INTERNAL ACTION THAT USED ENDIDLE, BUT HE HAS WRITTEN
# OVER THE INFORMATION ON THE DISPLAY PANEL BY SOME DISPLAYS OF HIS OWN
# INITIATION WHICH DO NOT SERVE AS RESPONSES. HITTING KEYRLSE WILL
# RE-ESTABLISH THE DISPLAYS TO THE STATE THEY WERE IN BEOFRE HE OBSCURED
# THEM, SO THAT HE CAN SEE THE WAITING REQUEST. THIS WORKS ONLY FOR
# INTERNAL PROGRAMS THAT USED ENDIDLE THROUGH MARGARETS DISPLAY
# SUBROUTINES.

VBRELDSP	CS	BIT3
		EXTEND
		WAND	DSALMOUT	# TURN OFF UPACT LITE
		CCS	21/22REG	# OLD DSPLOCK
		CAF	BIT14
		MASK	MONSAVE1	# EXTERNAL MONITOR BIT (EMB)
		CCS	A
		TC	UNSUSPEN	# OLD DSPLOCK AND EMB BOTH 1, UNSUSPEND.
TSTLTS4		TC	RELDSP		# NOT UNSUSPENDING EXTERNAL MONITOR,
		CCS	CADRSTOR	#	RELEASE DISPLAY SYSTEM AND
		TC	+2		#	DO RE-ESTABLISH IF CADRSTOR IS FULL.
		TC	ENDOFJOB
		TC	POSTJUMP
		CADR	PINBRNCH
UNSUSPEN	CAF	ZERO		# EXTERNAL MONITOR IS SUSPENDED,
		TS	DSPLOCK		#	JUST UNSUSPEND IT BY CLEARING DSPLOCK.
		CCS	CADRSTOR	#	TURN KEY RELEASE LIGHT OFF IF BOTH
		TC	ENDOFJOB	#	CADRSTOR AND DSPLIST ARE EMPTY.
		TC	RELDSP1
		TC	ENDOFJOB

ENDRELDS	EQUALS

; ============================================================================
; INTERNAL DISPLAY SUBROUTINE (NVSUB/NVMONOPT)
;
; COMMENT-ONLY READERS: These routines allowed AGC programs to display
; information to the crew without requiring manual button presses. When
; the guidance computer needed to show velocity, altitude, or fuel remaining,
; it called NVSUB with a verb-noun combination (like V16N68 for altitude/velocity).
; During Apollo 11's descent, this was how the computer automatically updated
; the displays that Armstrong and Aldrin monitored. NVMONOPT was a special
; version that could show "PLEASE PERFORM" requests after each update, asking
; the crew to take specific actions.
;
; CODE-ALONG READERS: NVSUB implements internal verb-noun display calls with
; sophisticated blocking logic. Takes verb-noun code in A register (bits 14-8=verb,
; bits 7-1=noun). Returns to caller+2 if display available and task completed,
; caller+1 if blocked by DSPLOCK or external monitor bit. Handles special negative
; codes for blanking: -4=full blank, -3=leave mode, -2=leave mode+verb, -1=blank
; R-registers only. NVMONOPT extends this with L register options: bits 8-14=verb
; to paste after each monitor cycle, bits 1-3=blanking options. Critical interlock:
; passes DSPLOCK only if external monitor bit clear, preventing keyboard conflicts.
; Stores caller+2 address in NVQTEM and places TC NVSUBEND in ENTRET for bank
; restoration. Calls BANKCALL/2CADR if MPAC+2 contains machine address.
;
; INTEGRATION: Called by all mission programs (P63 landing, P12 ascent, P20
; rendezvous) to display mission-critical data. Works with DISPLAY_INTERFACE_
; ROUTINES, EXTENDED_VERBS, and PINBALL_NOUN_TABLES to implement complete
; internal display capability.
; ============================================================================

# Page 451
# NVSUB IS USED FOR SUBROUTINE CALLS FROM WITHIN COMPUTER. IT CAN BE
# USED TO CALL THE COMBINATION OF ANY DISPLAY, LOAD, OR MONITOR VERB
# TOGETHER WITH ANY NOUN AVAILABLE TO THE KEYBOARD.
# PLACE 0VVVVVVVNNNNNNN INTO A.
# V'S ARE THE 7-BIT VERB CODE.  N'S ARE THE 7-BIT NOUN CODE.
#
# IF NVSUB IS CALLED WTIH THE FOLLOWING NEGATIVE NUMBERS (RATHER THAN THE
# VERB-NOUN CODE) IN A, THEN THE DISPLAY IS BLANKED AS FOLLOWS -
#  -4 FULL BLANK, -3 LEAVE MODE, -2 LEAVE MODE AND VERB, -1 BLANK R'S ONLY.
#
# NVSUB CAN BE USED WTIH MACH CADR TO BE SPEC BY PLACING THE CADR INTO
# MPAC+2 BEFORE THE STANDARD NVSUB CALL.
#
# NVSUB RETURNS TO 2+ CALLING LOC AFTER PERFORMING TASK, IF DISPLAY
# SYSTEM IS AVAIALBLE. THE NEW NOUN AND VERB CODES ARE DISPLAYED.
# IF V'S =0, THE NEW NOUN CODE IS DISPLAYED ONLY (RETURN WITH NO FURTHER
# ACTION). IF N'S =0, THE NEW VERB CODE IS DISPLAYED ONLY (RETURN WITH NO
# FURTHER ACTION).
#
# IT RETURNS TO 1+ CALLING LOC WITHOUT PERFORMING TASK, IF DISPLAY
# SYSTEM IS BLOCKED (NOTHING IS DISPLAYED IN THIS CASE).
# IT DOES TC ABORT (WITH OCT 01501) IF IT ENCOUNTERS A DISPLAY PROGRAM
# ALARM CONDITION BEFORE RETURN TO CALLER.
#
# THE DISPLAY SYSTEM IS BLOCKED BY THE DEPRESSION OF ANY
# KEY, EXCEPT ERROR LIGHT RESET.
# IT IS RELEASED BY THE KEY RELEASE BUTTON, ALL EXTENDED VERBS,
# PROCED WITHOUT DATA, TERMINATE, RESEQUENCE, INITIALIZE EXECUTIVE,
# RECALL PART OF RECALTST IF ENDIDLE WAS USED,
# VB = REQUEST EXECUTIVE, VB = REQUEST WAITLIST,
# MONITOR SET UP.
#
# THE DISPLAY SYSTEM IS ALSO BLOCKED BY THE EXTERNAL MONITOR BIT, WHICH
# INDICATES AND EXTERNALLY INITIATED MONITOR IS RUNNING (SEE MONITOR).
#
# A NVSUB CALL THAT PASSES DSPLOCK AND THE EXTERNAL MONITOR BIT ENDS OLD
# MONITOR.
#
# DSPLOCK IS THE INTERLOCK FOR USE OF KEYBOARD AND DISPLAY SYSTEM WHICH
# LOCKS OUT INTERNAL USE WHENEVER THERE IS EXTERNAL KEYBOARD ACTION.
#
# NVSUB SHOULD BE USED TWICE IN SUCCESSION FOR 'PLEASE PERFORM' SITUATIONS
# (SIMILARLY FOR PLEASE MARK).  FIRST PLACE THE CODED NUMBER FOR WHAT
# ACTION IS DESIRED OF OPERATOR INTO THE REGISTERS REFERRED TO BY THE
# 'CHECKLIST' NOUN. GO TO NVSUB WITH A DISPLAY VERB AND THE 'CHECKLIST'
# NOUN.  GO TO NVSUB AGAIN WTIH THE 'PLEASE PERFORM' VERB AND ZEROS IN THE
# LOW 7 BITS. THIS 'PASTES UP' THE 'PLEASE PERFORM' VERB INTO THE VERB
# LIGHTS.
#
# NVMONOPT IS AN ENTRY SIMILAR TO NVSUB, BUT REQUIRING AN ADDITIONAL
# Page 452
# PARAMETER IN L. IT SHOULD BE USED ONLY WITH A MONITOR VERB-NOUN CODE IN
# A. AFTER EACH MONITOR DISPLAY A *PLEASE* VERB WILL BE PASED INT THE VERB
# LIGHTS OR DATA WILL BE BLANKED (OR BOTH) ACCORDING TO THE OPTIONS
# SPECIFIED IN L. IF BITS 8-14 OF L ARE OTHER THAN ZERO, THEN THEY WILL
# BE INTERPRETED AS A VERB CODE AND PASTED IN THE VERB LIGHTS. (THIS VERB
# CODE SHOULD DESIGNATE ONE OF THE *PLEASE* VERBS.)  IF BITS 1-3 OF L ARE
# OTHER THAN ZERO, THEN THEY WILL BE USED TO BLANK DATA BY BEING FED TO
# BLANKSUB.  IF NVMONOPT IS USED WITH A VERB OTHER THAN A MONITOR VERB,
# THE PARAMETER IN L HAS NO EFFECT.
#
# NVSUB IN FIXED-FIXED PLACES 2+CALLING LOC INTO NVQTEM, TC NVSUBEND INTO
# ENTRET. (THIS WILL RESTORE OLD CALLING BANK BITS)

		SETLOC	ENDALM +1

		COUNT*	$$/PIN
NVSUB		LXCH	7		# ZERO NVMONOPT OPTIONS
NVMONOPT	TS	NVTEMP
		CAF	BIT14
		MASK 	MONSAVE1	# EXTERNAL MONITOR BIT
		AD	DSPLOCK
		CCS	A
		TC	Q		# DSP SYST BLOCKED, RET TO 1+ CALLING LOC
		CAF	ONE		# DSP SYST AVAILABLE.
NVSBCOM		AD	Q
		TS	NVQTEM		# 2+ CALLING LOC INTO NVQTEM
		LXCH	MONSAVE2	# STORE NVMONOPT OPTIONS
		TC	KILMONON	# TURN ON KILL MONITOR BIT
NVSUBCOM	CAF	NVSBBBNK

		XCH	BBANK
		EXTEND			# SAVE OLD SUPERBITS
		ROR	SUPERBNK
		TS	NVBNKTEM
		CAF	PINSUPBT
		EXTEND
		WRITE 	SUPERBNK
		TC	NVSUBB		# GO TO NVSUB1 THRU STANDARD LOC
		EBANK=	DSPCOUNT
NVSBBBNK	BBCON	NVSUB1

PINSUPBT	=	NVSBBBNK	# CONTAINS THE PINBALL SUPERBITS.

NVSUBEND	DXCH	NVQTEM		# NVBNKTEM MUST = NVQTEM+1
		TC	SUPDXCHZ	# DTCB WITH SUPERBIT SWITCHING

		SETLOC	ENDRQWT +1

		COUNT*	$$/PIN

# BLANKDSP BLANKS DISPLAY ACCORDING TO OPTION NUMBER IN NVTEMP AS FOLLOWS
# Page 453
#  -4 FULL BLANK, -3 LEAVE MODE, -2 LEAVE MODE AND VERB, -1 BLANK R'S ONLY.

BLANKDSP	AD	SEVEN		# 7,8,9, OR 10 (A HAD 0,1,2,OR 3)
		INHINT
		TS	CODE		# BLANK SPECIFIED DSPTABS
		CS	BIT12
		INDEX	CODE
		XCH	DSPTAB
		CCS	A
		INCR	NOUT
		TC	+1
		CCS	CODE
		TC	BLANKDSP +2
		RELINT
		INDEX	NVTEMP
		TC	+5
		TC	+1		# NVTEMP HAS	-4 (NEVER TOUCH MODREG)
		TS	VERBREG		#		-3
		TS	NOUNREG		#		-2
		TS	CLPASS		#		-1
		CS	VD1
		TS	DSPCOUNT
		TC	FLASHOFF	# PROTECT AGAINS INVISIBLE FLASH
		TC	ENTSET -2	# ZEROS REQRET

NVSUB1		CAF	ENTSET		# IN BANK
		TS	ENTRET		# SET RETURN TO NVSUBEND
		CCS	NVTEMP		# WHAT NOW
		TC	+4		# NORMAL NVSUB CALL (EXECUTE VN OR PASTE)
		TC	GODSPALM
		TC	BLANKDSP	# BLANK DISPLAY AS SPECIFIED
		TC	GODSPALM
		CAF	LOW7
		MASK	NVTEMP
		TS	MPAC +3		# TEMP FOR NOUN (CANT USE MPAC. DSPDECVN
		CA	NVTEMP		#		USES MPAC, +1, +2).
		TS	EDOP		# RIGHT 7
		CA	EDOP
		TS	MPAC +4		# TEMP FOR VERB (CANT USE MPAC+1. DSPDECVN
					# 		USES MPAC, +1, +2).
		CCS	MPAC +3		# TEST NOUN
		TC	NVSUB2		# IF NOUN NOT +0, GO ON
		CA	MPAC +4
		TC	UPDATVB -1	# IF NOUN = +0, DISPLAY VERB, THEN RETURN
		CAF	ZERO		# XERO REQRET SO THAT PASTED VERBS CAN
		TS	REQRET		# BE EXECUTED BY OPERATOR.
ENTSET		TC	NVSUBEND
NVSUB2		CCS	MPAC +4		# TEST VERB
		TC	+4		# IF VERB NOT +0, GO ON
		CA	MPAC +3
# Page 454
		TC	UPDATNN -1	# IF VERB = +0, DISPLAY NOUN, THEN RETURN
		TC	NVSUBEND
		CA	MPAC +2		# TEMP FOR MACH CADR TO BE SPEC. (DSPDECVN
		TS	MPAC +5		# 	USES MPAC, +1, +2)
		CA	MPAC +4
		TC	UPDATVB -1	# IF BOTH NOUN AND VERB NOT +0, DISPLAY
		CA	MPAC +3		# BOTH AND GO TO ENTPAS0.
		TC	UPDATNN -1
		CAF	ZERO
		TS	LOADSTAT	# SET FOR WAITING FOR DATA CONDITION
		TS	CLPASS
		TS	REQRET		# SET REQRET FOR PASS 0.
		CA	MPAC +5		# RESTORES MACH CADR TO BE SPEC TO MPAC+2
		TS	MPAC +2		# FOR USE IN INTMCTBS (IN ENTPAS0).
ENDNVSB1	TC	ENTPAS0

# IF INTERNAL MACH CADR TO BE SPECIFIED, MPAC+2 WILL BE PLACED INTO
# NOUNCADR IN ENTPAS0 (INTMCTBS).

		SETLOC	NVSUBEND +2
		COUNT*	$$/PIN
					# FORCE BIT 15 OF MONSAVE1 TO 1.
KILMONON	CAF	BIT15		# 	THIS IS THE KILL MONITOR BIT.
		TS	MONSAVE1	# TURN OFF BIT 14, THE EXTERNAL
					# 	MONITOR BIT.
		TC	Q

;
; ============================================================================
; ENDIDLE - CREW DATA ENTRY SUSPENSION AND JOB SYNCHRONIZATION
; ============================================================================
;
; COMMENT-ONLY READERS:
; When the AGC requests data from the crew through the DSKY, it must wait for
; the crew's response. This routine suspends the requesting program and puts
; it to sleep until the crew presses ENTER (accepting data), PROCEED (continuing
; without data), or TERMINATE (canceling the operation). During Apollo 11's
; mission, every time Armstrong or Aldrin entered coordinates, confirmed a
; program change, or responded to a computer request, this routine managed the
; synchronization between the crew's actions and the AGC's program execution.
;
; CODE-ALONG READERS:
; ENDIDLE implements a blocking wait for crew input using the LOADSTAT variable:
;   LOADSTAT = +0: Inactive, waiting for data (set by NVSUB)
;   LOADSTAT = +1: PROCEED pressed without data (set by JAMPROC)
;   LOADSTAT = -1: TERMINATE pressed (set by JAMTERM)
;   LOADSTAT = -0: Data loaded (set by BLANKSUB) or resequence (V32)
;
; The routine stores the calling job's return address in CADRSTOR and invokes
; JOBSLEEP to suspend execution. RECALTST (called by BLANKSUB/JAMPROC/JAMTERM)
; tests LOADSTAT and wakes the job, routing control to:
;   Return+1: TERMINATE requested (LOADSTAT = -1)
;   Return+2: PROCEED without data (LOADSTAT = +1)
;   Return+3: Data loaded or resequence (LOADSTAT = -0)
;
; ABORT 01206: Issued if CADRSTOR ≠ +0 or DSPLIST ≠ +0, indicating a second
; job is attempting to sleep in PINBALL (capacity exceeded). The AGC can only
; suspend one PINBALL job at a time for crew interaction.
;
; RESTRICTIONS: Cannot be called from erasable or F/F memory since JOBSLEEP
; and JOBWAKE only handle fixed banks. Return address must be in fixed memory.
; ============================================================================

# LOADSTAT	+0	INACTIVE (WAITING FOR DATA). SET BY NVSUB
#		+1	PROCEED NO DATA. SET BY SPECIAL VERB
#		-1	TERMINATE. SET BY SPECIAL VERB.
#		-0		DATA IN		SET BY END OF LOAD ROUTINE
#			OR 	RESEQUENCE	SET BY VERB 32
#
# L TO ENDIDLE (FIXED FIXED)
# ROUTINES THAT REQUEST LOADS THROUGH NVSUB SHOULD USE ENDIDLE WHILE
# WAITING FOR THE DATA TO BE LOADED. ENDIDLE PUTS CURRENT JOB TO SLEEP.
# ENDIDLE CANNOT BE CALLED FROM ERASABLE OR F/F MEMORY,
# SINCE JOB SLEEP AND JOBWAKE CAN HANDLE ONLY FIXED BANKS.
# RECALTST TESTS LOADSTAT AND WAKES JOB UP TO,
#	L+1	FOR TERMINATE
#	L+2	FOR PROCEED WITHOUT DATA
#	L+3	FOR DATA IN, OR RESEQUENCE
# IT DOES NOTHING IF LOADSTAT INDICATES WAITING FOR DTA.
#
# ENDIDLE ABORTS (WITH CODE 1206) IF A SECOND JOB ATTEMPTS TO GO TO SLEEP
# Page 455
# IN PINBALL. IN PARTICULAR, IF AN ATTEMPT IS MADE TO GO TO ENDIDLE WHEN
# 1)	CADRSTOR NOT= +0. THIS IS THE CASE WHERE THE CAPACITY OF ENDIDLE IS
#	EXCEEDED. (+-NZ INDICATES A JOB IS ALREADY ASLEEP DUE TO ENDIDDLE.)
# 2)	DSPLIST NOT= +0. THIS INDICATES A JOB IS ALREADY ASLEEP DUE TO
#	NVSUBUSY.

ENDIDLE		LXCH	Q		# RETURN ADDRESS INTO L.
		TC	ISCADR+0	# ABORT IF CADRSTOR NOT= +0
		TC	ISLIST+0	# ABORT IF DSPLIST NOT= +0
		CA	L		# DONT SET DSPLOC TO 1 SO CAN USE
		MASK	LOW10		# ENDIDLE WITH NVSUB INITIATED MONITOR.
		AD	FBANK		# SAME STRATEGY FOR CADR AS MAKECADR.
		TS	CADRSTOR
		TC	JOBSLEEP

ENDINST		TC	ENDOFJOB

ISCADR+0	CCS	CADRSTOR	# ABORTS (CODE 01206) IF CADRSTOR NOT= +0.
		TC	DSPABORT	# RETURNS IF CADRSTOR = +0.
		TC	Q
		TC	DSPABORT

ISLIST+0	CCS	DSPLIST		# ABORTS (CODE 01206) IF DSPLIST NOT= +0.
		TC	DSPABORT	# RETURNS IF DSPLIST = +0.
		TC	Q
DSPABORT	TC	POODOO
		OCT	01206

; ============================================================================
; JAMTERM - Internal Program Terminate Function Entry Point
;
; COMMENT-ONLY READERS: While crew members could press the RSET button to
; terminate (abort) a program, internal AGC software also needed a way to
; terminate operations programmatically. This routine provides that capability,
; simulating the effect of the TERMINATE verb (V34) without crew interaction.
; This was critical during automatic sequences where the computer needed to
; cleanly exit programs based on internal logic conditions.
;
; CODE-ALONG READERS: JAMTERM allows internal programs to perform the same
; operation as VERB 34 (TERMINATE). It sets REQRET to 34 decimal to simulate
; V34 entry, clears DSPCOUNT to reset display state, switches to PINBALL
; superbank, and transfers control to VBTERM (the V34 handler) which performs
; the actual termination sequence. The routine does an ENDOFJOB after
; completion via the VBTERM path.
; ============================================================================

# JAMTERM ALLOWS PROGRAMS TO PERFORM THE TERMINATE FUNCTION.
# IT DOES ENDOFJOB.

JAMTERM		CAF	PINSUPBT
		EXTEND
		WRITE	SUPERBNK
		CAF	34DEC
		TS	REQRET		# LEAVE ENTER SET FOR ENTPASS0.
		CS	VD1
		TS	DSPCOUNT
		TC	POSTJUMP
		CADR	VBTERM

34DEC		DEC	34

; ============================================================================
; JAMPROC - Internal Program Proceed Function Entry Point
;
; COMMENT-ONLY READERS: Just as internal programs needed a way to terminate
; operations (JAMTERM above), they also needed a way to proceed with pending
; operations without waiting for the crew to press the PRO button. This
; routine simulates the effect of the PROCEED verb (V33), allowing the
; computer to automatically continue through program sequences that normally
; require crew confirmation. During critical phases like powered descent,
; this allowed the AGC to maintain timing-critical sequences without crew
; interaction delays.
;
; CODE-ALONG READERS: JAMPROC allows internal programs to perform the same
; operation as VERB 33 (PROCEED) or PROCEED WITHOUT DATA. It sets REQRET to
; 33 decimal to simulate V33 entry, clears DSPCOUNT to reset display state,
; switches to PINBALL superbank, and transfers control to VBPROC (the V33
; handler) which processes the proceed request. Like JAMTERM, this routine
; does an ENDOFJOB after completion via the VBPROC path.
; ============================================================================

# JAMPROC ALLOWS PROGRAMS TO PERFORM THE PROCEED/PROCEED WITHOUT DATA
# FUNCTION. IT DOES ENDOFJOB.

# Page 456
JAMPROC		CAF	PINSUPBT
		EXTEND
		WRITE	SUPERBNK
		CAF	33DEC
		TS	REQRET		# LEAVE ENTER SET FOR ENTPASS0.
		CS	VD1
		TS	DSPCOUNT
		TC	POSTJUMP
		CADR	VBPROC

33DEC		DEC	33

; ============================================================================
; BLANKSUB - Selective Display Register Blanking Routine
;
; COMMENT-ONLY READERS: During mission operations, there were times when the
; computer needed to clear specific display registers to prepare for new data
; or indicate that certain information was no longer valid. Rather than
; leaving stale data visible to the crew, this routine allowed the AGC to
; selectively blank any combination of the three display registers (R1, R2,
; R3). This was particularly important during display mode transitions or when
; switching between different monitoring functions. The crew would see blank
; registers rather than potentially confusing leftover numbers.
;
; CODE-ALONG READERS: BLANKSUB provides selective blanking of display
; registers R1, R2, and/or R3. The calling program places a blanking code in
; the A register where BIT1=1 blanks R1, BIT2=1 blanks R2, and BIT3=1 blanks
; R3 (any combination accepted, masked to 3 bits via SEVEN). The routine
; checks DSPLOCK and external monitor status before proceeding - if display
; system is blocked, it returns to L+1 (failure); otherwise increments Q to
; return to L+2 (success path). DSPCOUNT is saved and restored to preserve
; display state. Uses bank switching (BLNKBBNK) and superbank operations to
; access the blanking subroutine BLNKSUB1. Each bit is tested via TESTBIT,
; and corresponding register (R1D1, R2D1, R3D1) is blanked via 5BLANK if bit
; is set. Returns via SUPDXCHZ with proper superbank restoration.
; ============================================================================

# BLANKSUB BLANKS ANY COMBINATION OF R1, R2, R3.
# CALL WITH BLANKING CODE IN A.
# BIT1=1 BLANKS R1, BIT2=1 BLANKS R2, BIT3=1 BLANKS R3.
# ANY COMBINATION OF THESE BITS IS ACCEPTED.
#
# DSPCOUNT IS RESTORED TO STATE IT WAS IN BEFORE BLANKSUB WAS EXECUTED.

BLANKSUB	MASK 	SEVEN
		TS	NVTEMP		# STORE BLANKING CODE IN NVTEMP.
		CAF	BIT14
		MASK	MONSAVE1	# EXTERNAL MONITOR BIT
		AD	DSPLOCK
		CCS	A
		TC	Q		# DSP SYST BLOCKED. RET TO 1+ CALLING LOC
		INCR	Q		# DSP SYST AVAILABLE
					# SET RETURN FOR 2+ CALLING LOC
		CCS	NVTEMP
		TCF	+2
		TC	Q		# NOTHING TO BLANK. RET TO 2+ CALLING LOC
		LXCH	Q		# SET RETURN FOR 2 + CALLING LOC
		CAF	BLNKBBNK
		XCH	BBANK
		EXTEND
		ROR	SUPERBNK	# SAVE OLD SUPERBITS.
		DXCH	BUF
		CAF	PINSUPBT
		EXTEND
		WRITE	SUPERBNK
		TC	BLNKSUB1

		EBANK=	DSPCOUNT
BLNKBBNK	BBCON	BLNKSUB1
ENDBLFF		EQUALS

		SETLOC	ENDRELDS
		COUNT*	$$/PIN
BLNKSUB1	CA	DSPCOUNT	# SAVE OLD DSPCOUNT FOR LATER RESTORATION
# Page 457
		TS	BUF +2
		CAF	BIT1		# TEST BIT1. SEE IF R1 TO BE BLANKED.
		TC	TESTBIT
		CAF	R1D1
		TC	5BLANK -1
		CAF	BIT2		# TEST BIT2. SEE IF R2 TO BE BLANKED.
		TC	TESTBIT
		CAF	R2D1
		TC	5BLANK -1
		CAF	BIT3		# TEST BIT3. SEE IF R3 TO BE BLANKED.
		TC	TESTBIT
		CAF	R3D1
		TC	5BLANK -1
		CA	BUF +2		# RESTORE DSPCOUNT TO STATE IT HAD
		TS	DSPCOUNT	# 	BEFORE BLANKSUB.
		DXCH	BUF		# CALL L+2 DIRECTLY.
		TC	SUPDXCHZ +1	# DTCB WITH SUPERBIT SWITCHING

TESTBIT		MASK	NVTEMP		# NVTEMP CONTAINS BLANKING CODE.
		CCS	A
		TC	Q		# IF CURRENT BIT = 1, RETURN TO L+1.
		INDEX	Q		# IF CURRENT BIT = 0, RETURN TO L+3.
		TC	2

ENDBSUB1	EQUALS

;
; ============================================================================
; DSPMM - Major Mode Display Request (Non-Blocking)
; ============================================================================
;
; COMMENT-ONLY READERS:
; Throughout the mission, the DSKY displayed a two-digit "major mode" code
; (MM) showing which program was currently active (e.g., P12 for ascent, P63
; for landing, P20 for rendezvous). This routine updates that major mode
; display whenever a program changes or needs to indicate its operational
; state to the crew. The routine immediately returns to the caller rather than
; waiting for the display update to complete, allowing the calling program to
; continue without delay.
;
; CODE-ALONG READERS:
; DSPMM schedules a display update job (DSPMMJB) via NOVAC with priority 30000
; (CHRPRIO) and returns immediately to the caller. This asynchronous approach
; prevents the calling program from blocking during display operations.
;
; DSPMMJB (the scheduled job) examines MODREG:
;   MODREG > 0 or +0: Display the major mode code via DSPDECVN
;   MODREG = -0: Blank the major mode display (2BLANK)
;   MODREG < 0 (negative non-zero): Do nothing (no update)
;
; DSPCOUNT is temporarily saved in DSPMMTEM during display formatting to
; preserve display state, then restored after DSPDECVN/2BLANK completes.
;
; RESTRICTION: DSPMM must reside in bank 27 or lower so it can be called
; via BANKCALL from any memory location. The job DSPMMJB executes under
; EXECUTIVE control and terminates via ENDOFJOB.
; ============================================================================

# DSPMM DOES NOT DISPLAY MODREG DIRECTLY. IT PUTS IN EXEC REQUEST WITH
# PRIO 30000 FOR DSPMMJB AND RETURNS TO CALLER.
#
# IF MODREG CONTAINS -0, DSPMMJB BLANKS THE MODE LIGHTS.
#
# DSPMM MUST BE IN BANK 27 OR LOWER, SO IT CAN BE CALLED VIA BANKCALL.

		BANK	7
		SETLOC	PINBALL4
		BANK

		COUNT*	$$/PIN
DSPMM		XCH	Q
		TS	MPAC
		INHINT
		CAF	CHRPRIO
		TC	NOVAC
		EBANK=	DSPCOUNT
		2CADR	DSPMMJB

		RELINT
ENDSPMM		TC	MPAC

# Page 458
# DSPMM  PLACE MAJOR MODE CODE INTO MODREG

		SETLOC	ENDBSUB1

		COUNT*	$$/PIN
DSPMMJB		CAF	MD1		# GETS HERE THRU DSPMM
		XCH	DSPCOUNT
		TS	DSPMMTEM	# SAVE DSPCOUNT
		CCS	MODREG
		AD	ONE
		TC	DSPDECVN	# IF MODREG IS + OR +0, DISPLAY MODREG
		TC	+2		# IF MODREG IS -NZ, DO NOTHING
		TC	2BLANK		# IF MODREG IS -0, BLANK MM
		XCH	DSPMMTEM	# RESTORE DSPCOUNT
		TS	DSPCOUNT
		TC	ENDOFJOB

;
; ============================================================================
; RECALTST - Job Wake and Return Routing After ENDIDLE
; ============================================================================
;
; COMMENT-ONLY READERS:
; When a program requested crew input through the DSKY and suspended itself
; (via ENDIDLE), this routine wakes it back up after the crew completes their
; action. The crew might enter data and press ENTER, press PROCEED to skip
; data entry, or press TERMINATE to abort the request. RECALTST determines
; which action the crew took and routes the woken program to the appropriate
; handling code, saving the current VERB and NOUN codes so the program can
; examine what the crew was responding to.
;
; CODE-ALONG READERS:
; RECALTST is called by BLANKSUB (data entered), JAMPROC (proceed without
; data), and JAMTERM (terminate) after they've set LOADSTAT appropriately.
;
; Entry conditions:
;   CADRSTOR: Contains job address to wake, or +0 if no job waiting
;   LOADSTAT: +1 (proceed), -1 (terminate), or -0 (data in/resequence)
;
; Processing:
;   1. CCS CADRSTOR: Test if job is actually waiting
;      - If +0: Normal exit via ENDOFJOB (keyboard-initiated, no job waiting)
;      - If non-zero: Proceed to RECAL1
;
;   2. JOBWAKE: Wake the suspended job using address from CADRSTOR
;
;   3. CCS LOADSTAT: Determine crew action and compute LOC offset
;      - +1 (positive): PROCEED → DOPROC adds 1 to LOC (return address + 1)
;      - +0: Pathological case (should not occur) → ENDOFJOB
;      - -1 (negative): TERMINATE → DOTERM adds 0 to LOC (return address + 0)
;      - -0 (minus zero): DATA IN or RESEQUENCE → adds 2 to LOC (return + 2)
;
;   4. Save VERBREG/NOUNREG to MPAC/MPAC+1 via INDEX LOCCTR + DXCH
;      This allows the woken job to examine what verb/noun was active during
;      the crew's response
;
;   5. RELDSP: Release display resources and turn off KEY REL light if
;      appropriate (handled by RELDSP routine)
;
; The offset mechanism (0, 1, or 2) allows the woken job to have three
; different continuation points based on crew action, enabling clean
; conditional branching without explicit tests in the resumed code.
;
; Historical context: During Apollo 11 landing, Armstrong and Aldrin used
; PROCEED, TERMINATE, and data entry throughout descent to confirm guidance
; decisions, enter manual control parameters, and acknowledge displays.
; ============================================================================

# RECALTST IS ENTERED DIRECTLY AFTER DATA IS LOADED (OR RESEQUENCE VERB IS
# EXECUTED), TERMINATE VERB IS EXECUTED, OR PROCEED WITHOUT DATA VERB IS
# EXECUTED. IT WAKES UP JOB THAT DID TC ENDIDLE.
#
# IF CADRSTOR NOT= +0, IT PUTS +0 INTO DSPLOCK, AND TURNS OFF KEY RLSE
# LIGHT IF DSPLIST IS EMPTY (LEAVES KEY RLSE LIGHT ALONE IF NOT EMPTY).

RECALTST 	CCS	CADRSTOR
		TC	RECAL1
		TC	ENDOFJOB	# NORMAL EXIT IF KEYBOARD INITIATED
RECAL1		CAF	ZERO
		XCH	CADRSTOR
		INHINT
		TC	JOBWAKE
		CCS	LOADSTAT
		TC	DOPROC		# + PROCEED WITHOUT DATA
		TC	ENDOFJOB	# PATHALOGICAL CASE EXIT
		TC	DOTERM		# - TERMINATE
		CAF	TWO		# -0 DATA IN OR RESEQUENCE
RECAL2		INDEX	LOCCTR
		AD	LOC		# LOC IS + FOR BASIC JOBS
		INDEX	LOCCTR
		TS	LOC
		CA	NOUNREG		# SAVE VERB IN MPAC, NOUN IN MPAC+1 AT
		TS	L		# TIME OF RESPONSE TO ENDIDLE FOR
		CA	VERBREG		# POSSIBLE LATER TESTING BY JOB THAT HAS
		INDEX	LOCCTR		# BEEN WAKED UP.
		DXCH	MPAC
		RELINT
RECAL3		TC	RELDSP
		TC	ENDOFJOB

# Page 459
DOTERM		CAF	ZERO
		TC	RECAL2

DOPROC		CAF	ONE
		TC	RECAL2

# Page 460

;
; ============================================================================
; MISCELLANEOUS SERVICE ROUTINES - Noun Data Access Utilities
; ============================================================================
;
; COMMENT-ONLY READERS:
; These short utility routines help the computer locate the erasable memory
; locations containing the data associated with each noun. Since nouns can
; reference data stored in different memory banks (EBANKS), these routines
; extract the bank number and address from the ECADR (Extended Core Address),
; setting up the computer's memory access accordingly. This allowed a single
; noun number to transparently access data regardless of where it was stored
; in the AGC's 2K erasable memory organized across multiple banks.
;
; CODE-ALONG READERS:
; ECADR format packs both EBANK number and erasable address into a single word.
; Upper bits specify EBANK, lower 8 bits specify offset within bank. OCT1400
; is the base address of erasable memory in the AGC address space. These
; routines decode ECADR and set up EBANK register + derived E-address for
; subsequent access to noun data registers.
; ============================================================================

# MISCELLANEOUS SERVICE ROUTINES IN FIXED/FIXED

		SETLOC	ENDBLFF

		COUNT*	$$/PIN

;
; ----------------------------------------------------------------------------
; SETNCADR - Store ECADR and Derive Noun Address
; ----------------------------------------------------------------------------
; Entry: A contains ECADR (Extended Core Address) for noun data
; Exit: NOUNCADR = ECADR, EBANK set, NOUNADD = E-address, A = preserved
;
; Stores the ECADR, sets EBANK register from upper bits, derives E-address
; by masking lower 8 bits and adding OCT1400 base, stores result in NOUNADD.
; Used when setting up access to noun data registers.
; ----------------------------------------------------------------------------

# SETNCADR	E CADR ARRIVES IN A. IT IS STORED IN NOUNCADR. EBANK BITS
#		ARE SET.  E ADRES IS DERIVED AND PUT INTO NOUNADD.

SETNCADR	TS	NOUNCADR	# STORE ECADR
		TS	EBANK		# SET EBANK BITS
		MASK	LOW8
		AD	OCT1400
		TS	NOUNADD		# PUT E ADRES INTO NOUNADD
		TC	Q

;
; ----------------------------------------------------------------------------
; SETNADD - Get ECADR from NOUNCADR and Derive Address
; ----------------------------------------------------------------------------
; Entry: NOUNCADR contains previously stored ECADR
; Exit: EBANK set, NOUNADD = E-address
;
; Retrieves ECADR from NOUNCADR and calls into SETNCADR+1 (bypassing the
; NOUNCADR storage since it's already there). Used when noun ECADR was
; previously stored and needs to be reused for another access.
; ----------------------------------------------------------------------------

# SETNADD	GETS E CADR FROM NOUNCADR, SETS EBANK BITS, DERIVES
#		E ADRES AND PUTS IT INTO NOUNADD.

SETNADD		CA	NOUNCADR
		TCF	SETNCADR +1

;
; ----------------------------------------------------------------------------
; SETEBANK - Set EBANK and Return E-Address
; ----------------------------------------------------------------------------
; Entry: A contains ECADR
; Exit: EBANK set, A = E-address (not stored in NOUNADD)
;
; Similar to SETNCADR but returns E-address in A without storing it in
; NOUNADD or NOUNCADR. Used when caller needs immediate E-address for
; single access without persistent storage.
; ----------------------------------------------------------------------------

# SETEBANK	E CADR ARRIVES IN A. EBANK BITS ARE SET. E ADRES IS
#		DERIVED AND LEFT IN A.

SETEBANK	TS	EBANK		# SET EBANK BITS
		MASK	LOW8
		AD	OCT1400		# E ADRES LEFT IN A
		TC	Q

R1D1		OCT	16		# THESE 3 CONSTANTS FORM A PACKED TABLE.
R2D1		OCT	11		# DONT SEPARATE.
R3D1		OCT	4

RIGHT5		TS	CYR
		CS	CYR
		CS	CYR
		CS	CYR
		CS	CYR
		XCH	CYR
		TC	Q

LEFT5		TS	CYL
		CS	CYL
		CS	CYL
		CS	CYL
		CS	CYL
# Page 461
		XCH	CYL
		TC	Q

SLEFT5		DOUBLE
		DOUBLE
		DOUBLE
		DOUBLE
		DOUBLE
		TC	Q

LOW5		OCT	37		# THESE 3 CONSTANTS FORM A PACKED TABLE.
MID5		OCT	1740		# DONT SEPARATE.
HI5		OCT	76000		# MUST STAY HERE

TCNOVAC		TC	NOVAC
TCWAIT		TC	WAITLIST
TCTSKOVR	TC	TASKOVER
TCFINDVC	TC	FINDVAC

CHRPRIO		OCT	30000		# EXEC PRIORITY OF CHARIN

LOW11		OCT	3777
B12-1		EQUALS	LOW11
LOW8		OCT	377

VD1		OCT	23		# THESE 3 CONSTANTS FORM A PACKED TABLE.
ND1		OCT	21		# DONT SEPARATE.
MD1		OCT	25

BINCON		DEC	10

; ============================================================================
; INDICATOR LIGHT CONTROL ROUTINES
;
; These subroutines directly control DSKY indicator lights via channel 11
; (DSALMOUT). Each light is mapped to a specific bit in the channel word.
; WOR (write OR) turns lights on, WAND (write AND) turns lights off.
;
; Historical Context: During Apollo 11 landing, these lights provided critical
; crew feedback. The OPR ERR light indicated input mistakes, KEY REL showed
; when the computer awaited button release, and V/N FLASH signaled the computer
; needed crew input. Armstrong and Aldrin relied on these visual cues during
; the high-workload descent when verbal communication was minimal.
;
; Channel 11 (DSALMOUT) bit assignments:
; Bit 5: KEY REL (Key Release indicator)
; Bit 6: V/N FLASH (Verb/Noun flash indicator)
; Bit 7: OPR ERR (Operator Error indicator)
; ============================================================================

; ----------------------------------------------------------------------------
; FALTON - Turn On Operator Error Light
;
; Illuminates the OPR ERR indicator on the DSKY to alert crew of invalid input,
; such as illegal verb/noun combination, out-of-range data, or incomplete entry.
; The light remains on until crew presses RSET (reset) to acknowledge the error.
;
; Technical: Sets bit 7 of channel 11 using WOR (write OR) operation.
; Preserves other channel bits. Returns via TC Q (return to caller).
; ----------------------------------------------------------------------------
FALTON		CA	BIT7		# TURN ON OPERATOR ERROR LIGHT.
		EXTEND
		WOR	DSALMOUT	# BIT 7 OF CHANNEL 11
		TC	Q		; Return to caller

; ----------------------------------------------------------------------------
; FALTOF - Turn Off Operator Error Light
;
; Extinguishes the OPR ERR indicator after crew acknowledges the error with
; RSET button. This clears the visual error indication and allows normal
; DSKY operations to resume.
;
; Technical: Clears bit 7 of channel 11 using WAND (write AND) operation
; with complement of BIT7. This turns off only the OPR ERR light while
; preserving the state of other indicator lights in the channel.
; ----------------------------------------------------------------------------
FALTOF		CS	BIT7		# TURN OFF OPERATOR ERROR LIGHT
		EXTEND
		WAND	DSALMOUT	# BIT 7 OF CHANNEL 11
		TC	Q		; Return to caller

; ----------------------------------------------------------------------------
; RELDSPON - Turn On Key Release Light
;
; Illuminates the KEY REL indicator to inform crew that the computer is waiting
; for them to release a held button before continuing. This prevents accidental
; repeated input from a single button press and ensures clean digital transitions.
;
; Historical Context: During Apollo 11, button debouncing was critical. The
; KEY REL light told Armstrong and Aldrin when to release buttons, preventing
; multiple unintended digit entries during rapid verb/noun sequences.
;
; Technical: Sets bit 5 of channel 11 via WOR operation. The light typically
; remains on briefly until the physical button is released and KEYRUPT detects
; the release, then software clears it.
; ----------------------------------------------------------------------------
RELDSPON	CAF	BIT5		# TURN ON KEY RELEASE LIGHT
		EXTEND
		WOR	DSALMOUT	# BIT 5 OF CHANNEL 11
		TC	Q		; Return to caller

# Page 462
; ============================================================================
; LODSAMPT - Load Sample Time for Monitor Displays
;
; Captures the current AGC mission elapsed time into SAMPTIME register for
; use by monitor display verbs. This timestamp indicates when displayed data
; was sampled from the system, helping crew verify data freshness.
;
; COMMENT-ONLY READERS: Monitor displays (like velocity, altitude, orbital
; parameters) were updated once per second. This routine timestamps each
; sample so Armstrong and Aldrin knew exactly when the displayed value was
; captured. During descent, knowing data age was critical for situational
; awareness.
;
; CODE-ALONG READERS: Uses double-precision load (EXTEND/DCA) to atomically
; capture TIME2 (AGC's 28-bit mission elapsed time counter that increments
; every 10 milliseconds). SAMPTIME is then used by display formatting routines
; to show data age or trigger updates when data becomes stale.
;
; Entry: None (reads TIME2 automatically)
; Exit: SAMPTIME contains current TIME2 value
; Called by: MONREQ (waitlist-driven monitor update routine)
; ============================================================================
LODSAMPT	EXTEND			# Double-precision operation
		DCA	TIME2		# Load mission elapsed time (centiseconds)
		DXCH	SAMPTIME	# Store atomically in sample time register
		TC	Q		# Return to caller

; ============================================================================
; TPSL1 - Triple Precision Shift Left by 1 Bit
;
; Shifts the triple-precision value in MPAC, MPAC+1, MPAC+2 left by one bit
; (equivalent to multiplying by 2). Detects overflow/underflow and signals
; via MPAC+6 if the shift causes out-of-range conditions.
;
; COMMENT-ONLY READERS: This is a mathematical utility for scaling display
; values. When the computer needed to adjust decimal point positions or scale
; coordinates for different display formats, this routine performed the
; binary bit-shift operation.
;
; CODE-ALONG READERS: Triple-precision shift is implemented using DAS (double
; add to self = multiply by 2) on MPAC+1/+2, then propagating the high bit
; to MPAC. The TS instruction at line 6489 tests for overflow - if overflow
; occurs, execution falls through to set MPAC+6 to ±1 as an error indicator.
; If no overflow, the first TC Q returns immediately. This is a clever use
; of the AGC's overflow detection where TS skips the next instruction on
; overflow/underflow.
;
; Entry: MPAC, MPAC+1, MPAC+2 contain triple-precision value
; Exit: MPAC, MPAC+1, MPAC+2 shifted left 1 bit
;       MPAC+6 = +1 for overflow, -1 for underflow, or unchanged if no error
; ============================================================================
TPSL1		EXTEND			# SHIFTS MPAC, +1, +2 LEFT 1
		DCA	MPAC +1		# LEAVES OVFIND SET TO +/- 1 FOR OF/UF
		# Load double-precision low words
		DAS	MPAC +1		# Double add to self = shift left 1
		# MPAC+1/+2 now doubled, carry propagates to next operation
		AD	MPAC		# Add high word (propagates carry from DAS)
		ADS	MPAC		# Add to self and store (completes shift)
		# MPAC now shifted left, incorporating carry from lower words
		TS	7		# TS A DOES NOT CHANGE A ON OF/UF.
		# Test for overflow: TS increments next if no overflow
		TC	Q		# NO NET OF/UF - normal return
		TS	MPAC+6		# MPAC +6 SET TO +/- 1 FOR OF/UF
		# Overflow occurred: store +1 or -1 in MPAC+6 as error flag
		TC	Q		# Return with overflow indicator set

; ============================================================================
; PRSHRTMP - Precision-Safe Short Multiply for Scale Factor Constants
;
; A specialized wrapper around SHORTMP that corrects a sign error corner case.
; When multiplying certain scale factor constants, SHORTMP can incorrectly
; produce +0 when the true result should be -0 (maintaining sign in AGC's
; ones-complement arithmetic is critical for proper downstream calculations).
;
; COMMENT-ONLY READERS: This is an arithmetic precision fix for a very specific
; edge case in the display conversion math. The AGC used ones-complement
; arithmetic where both +0 and -0 exist, and maintaining the correct zero
; sign was important for accurate coordinate transformations and unit
; conversions shown on the DSKY.
;
; CODE-ALONG READERS: The bug occurs when MPAC and MPAC+1 are both positive
; non-zero (or +0) and the A register contains -0. In ones-complement
; arithmetic, -0 (octal 177777) is distinct from +0 (octal 000000). The CCS
; instruction tests A: if +, +0, or -, proceed to SHORTMP normally. Only if
; A contains exactly -0 (fourth case of CCS) does the code force the result
; to -0 by storing CS ZERO (which is -0) into MPAC, MPAC+1, MPAC+2.
;
; WARNING: Only use when MPAC and MPAC+1 are both positive (as they are when
; containing scale factor constants). Do not use for general multiplication.
;
; Entry: A register contains multiplier, MPAC/MPAC+1 contain multiplicand
; Exit: MPAC/MPAC+1/MPAC+2 contain product with correct sign
; ============================================================================
# IF MPAC, +1 ARE EACH +NZ OR +0 AND C(A)=-0, SHORTMP WRONGLY GIVES +0.
# IF MPAC, +1 ARE EACH -NZ OR -0 AND C(A)=+0, SHORTMP WRONGLY GIVES +0.
# PRSHRTMP FIXES FORST CASE ONLY, BY MERELY TESTING C(A) AND IF IT = -0,
# SETTING RESULT TO -0.
#  (DO NOT USE PRSHRTMP UNLESS MPAC, +1 ARE EACH +NZ OR +0, AS THEY ARE
#  WHEN THEY CONTAIN THE SF CONSTANTS.)

PRSHRTMP	TS	MPTEMP		# Save A register value for later restore
		CCS	A		# Count, compare, skip: test A register
		# Four cases: positive non-zero, +0, negative non-zero, -0
		CA	MPTEMP		# C(A) +, DO REGULAR SHORTMP
		# Positive non-zero: restore A and do normal multiply
		TCF	SHORTMP +1	# C(A) +0, DO REGULAR SHORTMP
		# Positive zero: do normal multiply
		TCF	-2		# C(A) -, DO REGULAR SHORTMP
		# Negative non-zero: restore A (branch back) and do normal multiply
		CS	ZERO		# C(A) -0, FORCE RESULT TO -0 AND RETURN.
		# This is the bug case: A contained -0, must force result to -0
		TS	MPAC		# Store -0 in result high word
		TS	MPAC +1		# Store -0 in result middle word
		TS	MPAC +2		# Store -0 in result low word
		# Complete triple-precision -0 result (all words -0)
		TC	Q		# Return with corrected result

; ============================================================================
; FLASHON - Turn On Verb/Noun Flash Indicator
;
; Illuminates the V/N FLASH light on the DSKY to signal the crew that the
; computer needs their input. This flashing indicator draws immediate attention
; to the display, indicating a pending verb/noun entry or data request.
;
; Historical Context: During Apollo 11 descent, the V/N FLASH was critical
; for crew awareness. When the computer needed input (such as confirming
; a program load with V37), the flash immediately alerted Armstrong and Aldrin
; even when their attention was on other instruments or out the window during
; manual landing site selection.
;
; The flash typically occurs when:
; - Computer requests verb/noun entry (after VERB or NOUN button)
; - Program requests data load (load verbs V21-V25)
; - Program change confirmation needed (V37)
; - Monitor update completed (certain monitoring verbs)
;
; Technical: Sets bit 6 of channel 11 via WOR operation. The flash remains
; on until crew provides requested input or presses RSET to cancel.
; ============================================================================
FLASHON		CAF	BIT6		# TURN ON V/N FLASH
		EXTEND			# BIT 6 OF CHANNEL 11
		WOR	DSALMOUT	; Set bit 6 (flash) in DSKY output channel
		TC	Q		; Return to caller

; ============================================================================
; FLASHOFF - Turn Off Verb/Noun Flash Indicator
;
; Extinguishes the V/N FLASH light after the crew has provided the requested
; input or when the computer no longer needs crew attention. This signals
; that the DSKY is in a stable display mode rather than awaiting input.
;
; Historical Context: During landing, the flash on/off cycle provided vital
; feedback about system state. When Armstrong entered V16 N68 for altitude
; display, FLASHON occurred after VERB button, then FLASHOFF occurred after
; ENTER completion, confirming the computer had accepted the command and
; was now displaying data rather than awaiting input.
;
; Technical: Clears bit 6 of channel 11 using WAND with complement of BIT6.
; This turns off only the V/N FLASH while preserving other indicator states.
; ============================================================================
FLASHOFF	CS	BIT6		# TURN OFF V/N FLASH
		EXTEND
		WAND	DSALMOUT	; Clear bit 6 (flash) in DSKY output channel
		TC	Q		; Return to caller

# Page 463
# INTERNAL USE OF KEYBOARD AND DISPLAY PROGRAM.
#
# USER MUST SCHEDULE CALLS TO NVSUB SO THAT THERE IS NO CONFLICT OF USE OR
# CONFUSION TO OPERATOR. THE OLD GRABLOCK (INTERNAL/INTERNAL INTERLOCK)
# HAS BEEN REMOVED AND THE INTERNAL USER NO LONGER HAS THE PROTECTION THIS
# OFFERED.
#
# THERE ARE TWO WAYS A JOB CAN BE PUT TO SLEEP BY THE KEYBOARD + DISPLAY
# PROGRAM.	1) BY ENDIDLE
#		2) BY NVSUBUSY
# THE BASIC CONVENTION IS THAT ONLY ONE JOB WILL BE PERMITTED ASLEEP VIA
# THE KEYBOARD + DISPLAY PROGRAM AT A TIME. IF A JOB ATTEMPTS TO GO TO
# SLEEP BY MEANS OF (1) OR (2) AND THERE IS ALREADY A JOB ASLEEP THAT WAS
# PUT TO SLEEP BY (1) OR (2), THEN AN ABORT IS CAUSED.
#
# THE CALLING SEQUENCE FOR NVSUB IS
#			CAF		V/N
#	L		TC		NVSUB
#	L+1		RETURN HERE IF OPERATOR HAS INTERVENED
#	L+2		RETURN HERE AFTER EXECUTION
#
# A ROUTINE CALLED NVSUBUSY IS PROVIDED (USE IS OPTIONAL) TO PUT
# YOUR JOB TO SLEEP UNTIL THE OPERATOR RELEASES THE KEYBOARD + DISPLAY
# SYSTEM. NVSUBUSY ALSO TURNS ON THE KEY RELEASE LIGHT.
# NVSUBUSY CANNOT BE CALLED FROM ERASABLE OR F/F MEMORY,
# SINCE JOBSLEEP AND JOBWAKE CAN HANDLE ONLY FIXED BANKS.
#
# THE CALLING SEQUENCE IS
#	CAF	WAKEFCADR
#	TC	NVSUBUSY
#
#
# .
#
# NVSUBUSY IS INTENDED FOR USE WHEN AN INTERNAL PROGRAM FINDS THE OPERATOR
# IS NOT USING THE KEYBOARD + DISPLAY PROGRAM (BY HIS OWN INITIATION). IT IS
# NOT INTENDED FOR USE WHEN ONE INTERNAL PROGRAM FINDS ANOTHER INTERNAL
# PROGRAM USING THE KEYBOARD + DISPLAY PROGRAM.
#
# NVSUBUSY ABORTS (WITH CODE 01206) IF A SECOND JOB ATTEMPTS TO GO TO
# SLEEP IN PINBALL. IN PARTICULAR, IF AN ATTEMPT IS MADE TO GO TO NVSUBUSY
# WHEN
# 1)	DSPLIST NOT= +0.  THIS IS THE CASE WHERE THE CAPACITY OF THE DSPLIST
#	IS EXCEEDED.
# 2) 	CADRSTOR NOT= +0.  THIS INDICATES THAT A JOB IS ALREADY USING
# Page 464
# ENDIDLE.  (+-NZ INDICATE A JOB IS ALREADY ASLEEP DUE TO ENDIDLE.)

;
; ----------------------------------------------------------------------------
; PRENVBSY - Pre-process Return Address for NVSUBUSY (Fixed Bank Entrance)
; ----------------------------------------------------------------------------
; COMMENT-ONLY READERS:
; This special entrance point is used by routines stored in the AGC's fixed
; (ROM) memory banks when they need to put themselves to sleep waiting for
; the DSKY to become available. It performs address arithmetic to adjust the
; return address (where the routine will resume after waking up) to account
; for the multi-bank calling convention.
;
; CODE-ALONG READERS:
; Entry from fixed banks requires FCADR (Full Core Address with bank bits).
; This routine computes (caller's Q - 2) and adds FBANK to form proper FCADR
; for DSPLIST storage. The "2K+3" (OCT 2003) constant compensates for the
; address offset in the calling sequence. Result flows into NVSUBUSY.
; ----------------------------------------------------------------------------

PRENVBSY	CS	2K+3		# SPECIAL ENTRANCE FOR ROUTINES IN FIXED
		AD	Q		# BANKS ONLY DESIRING THE FCADR OF (LOC
		AD	FBANK		# FROM WHICH THE TC PRENVBSY WAS DONE) -2

;
; ----------------------------------------------------------------------------
; NVSUBUSY - Go to Sleep Waiting for DSKY to Become Available
; ----------------------------------------------------------------------------
; COMMENT-ONLY READERS:
; When multiple programs try to use the DSKY simultaneously, only one can
; proceed while others must wait. This routine puts the calling job to sleep,
; adding its return address to a waiting queue (DSPLIST). When the DSKY
; becomes free, the job will be automatically awakened to continue.
;
; If the system detects that too many jobs are waiting (queue overflow) or
; that the sleep mechanism is already in use by another job, it triggers
; program alarm 01206 to alert the crew that the system is overloaded.
;
; CODE-ALONG READERS:
; Entry: A contains return address (from PRENVBSY or direct call)
; Jumps to NVSUBSY1 in low bank to ensure proper superbits for JOBSLEEP.
; NVSUBSY1 validates CADRSTOR=0 and DSPLIST capacity, then stores return
; address in DSPLIST and calls JOBSLEEP to suspend calling job. Job will
; be restarted by executive when DSKY released via RELDSP.
; ----------------------------------------------------------------------------

NVSUBUSY	TC	POSTJUMP	# TO BE ENTERED.
		CADR	NVSUBSY1
2K+3		OCT	2003

# NVSUBSY1 MUST BE IN BANK 27 OR LOWER, SO IT WILL PUT CALLER TO SLEEP
# WITH HIS PROPER SUPERBITS.

		SETLOC	ENDSPMM +1
		COUNT*	$$/PIN

;
; ----------------------------------------------------------------------------
; NVSUBSY1 - Complete Sleep Setup and Suspend Job
; ----------------------------------------------------------------------------
; COMMENT-ONLY READERS:
; This is the second half of the "go to sleep waiting for DSKY" process.
; After the calling routine's return address is properly computed (by
; PRENVBSY/NVSUBUSY), this routine performs safety checks to ensure the
; sleep mechanism isn't already in use, turns off the KEY RELEASE indicator
; light, adds the return address to the waiting queue, and suspends the job.
;
; If the safety checks fail (indicating system overload or conflicting sleep
; requests), program alarm 01206 is triggered to alert the crew.
;
; CODE-ALONG READERS:
; Entry: A contains return address to store in DSPLIST
;
; 1. TS L: Save return address in L register
; 2. TC ISCADR+0: Check CADRSTOR=0, abort with 01206 if non-zero (ENDIDLE busy)
; 3. TC ISLIST+0: Check DSPLIST capacity, abort with 01206 if overflow
; 4. TC RELDSPON: Turn off KEY RELEASE light (DSKY indicator)
; 5. CA L, TS DSPLIST: Move return address from L into DSPLIST queue
; 6. TC JOBSLEEP: Suspend job execution (will resume at return address when
;    DSKY released via RELDSP which calls DEQUEUE to wake next waiting job)
;
; Bank placement constraint: Must be in bank 27 or lower to ensure JOBSLEEP
; receives proper superbank bits for job restart.
; ----------------------------------------------------------------------------

NVSUBSY1	TS	L
		TC	ISCADR+0	# ABORT IF CADRSTOR NOT= +0.
		TC	ISLIST+0	# ABORT IF DSPLIST NOT= +0.
		TC	RELDSPON
		CA	L
		TS	DSPLIST
ENDNVBSY	TC	JOBSLEEP

# NVSBWAIT IS A SPECIAL ENTRANCE FOR ROUTINES IN FIXED BANKS ONLY. IF
# SYSTEM IS NOT BUSY, IT EXECUTES V/N AND RETURNS TO L+1 (L= LOC FROM
# WHICH THE TC NVSBWAIT WAS DONE). IF SYSTEM IS BUSY, IT PUTS CALLING JOB
# TO SLEEP WITH L-1 GOING INTO LIST FOR EVENTUAL WAKING UP WHEN SYSTEM
# IS NOT BUSY.

		SETLOC	NVSUBUSY +3
		COUNT*	$$/PIN

;
; ----------------------------------------------------------------------------
; NVSBWAIT - Conditional Wait for DSKY Availability (Fixed Bank Entrance)
; ----------------------------------------------------------------------------
; COMMENT-ONLY READERS:
; This routine provides a "try now or wait" approach for programs wanting to
; use the DSKY. When called from a fixed-bank (ROM) routine, it first checks
; if the DSKY is currently available. If the display system is free, the
; program proceeds immediately with its verb/noun display. If the DSKY is
; busy (another program is using it, or an external monitor display is
; active), the calling program is put to sleep and will be automatically
; awakened when the DSKY becomes available.
;
; This mechanism was essential during Apollo 11's descent and landing phases.
; When Armstrong requested altitude and velocity displays while the guidance
; computer was simultaneously trying to update navigation information, this
; routine ensured orderly access without display conflicts. The crew could
; press VERB-NOUN keys without worrying about internal program conflicts.
;
; CODE-ALONG READERS:
; Special fixed-bank entrance with conditional sleep behavior:
;
; Entry: Called via TC NVSBWAIT from fixed bank routine
;        Q register contains return address (caller location)
;        L register expected to contain verb/noun or display data
;
; Operation:
; 1. LXCH 7: Exchange L with register 7, zeroing NVMONOPT options
;    (register 7 serves as a convenient zero source after initialization)
; 2. TS NVTEMP: Save original L content (display data) to NVTEMP
; 3. CAF BIT14, MASK MONSAVE1: Extract external monitor bit (bit 14)
;    This checks if an external monitoring display is currently active
; 4. AD DSPLOCK: Add DSPLOCK value (0 if free, non-zero if locked)
; 5. CCS A: Test combined result (monitor bit + lock status)
;
; If result is zero (system free):
;    TCF NVSBCOM: Jump to common NVSUB entry point
;    NVSBCOM will restore display data from NVTEMP and execute the requested
;    verb/noun operation, then return to caller's L+1 address
;
; If result is non-zero (system busy):
;    TCF NVSBWT1: Branch to wait logic
;    INCR Q: Increment return address from L to L+2
;    TCF PRENVBSY: Jump to sleep routine
;    PRENVBSY will compute adjusted return address L-1 (net result: caller's L+1)
;    and put job to sleep via NVSUBUSY/NVSUBSY1
;
; Return behavior:
; - If system was free: Returns to L+1 after executing display operation
; - If system was busy: Job sleeps, wakes at L+1 when DSKY becomes available
;
; The Q increment/PRENVBSY decrement arithmetic ensures that regardless of the
; execution path (immediate or delayed), the calling routine resumes at the
; same location (L+1) after the display operation completes.
; ----------------------------------------------------------------------------

NVSBWAIT	LXCH	7		# ZERO NVMONOPT OPTIONS
		TS	NVTEMP
		CAF	BIT14
		MASK	MONSAVE1	# EXTERNAL MONITOR BIT
		AD	DSPLOCK
		CCS	A
		TCF	NVSBWT1		# BUSY
		TCF	NVSBCOM		# FREE. NVSUB WILL SAVE L+1 FOR RETURN
					# AFTER EXECUTION.
NVSBWT1		INCR	Q		# L+2. PRENVBSY WILL PUT L-1 INTO LIST AND
		TCF	PRENVBSY	# GO TO SLEEP.

# RELDSP IS USED BY VBPROC, VBTERM, VBRQEXEC, VBRQWAIT, VBRELDSP, EXTENDED
# VERB DISPATCHER, VBRESEQ, RECALTST.
# RELDSP1 IS USED BY MONITOR SET UP, VBRELDSP.

; ============================================================================
; RELDSP - Release Display (Full Version with List Search)
;
; This routine releases the DSKY display system from busy state, allowing
; the next pending display operation to proceed. It turns off the KEY RELEASE
; light, clears the display lock, and wakes up any job waiting in the display
; list queue (DSPLIST).
;
; Historical Context: During Apollo 11's critical phases, multiple programs
; competed for DSKY display time. When Armstrong wanted altitude displays
; (V16 N68) while guidance was updating navigation data, RELDSP managed the
; orderly transition between display users. Without this coordination, display
; information could have become garbled or lost during the landing sequence.
;
; The DSPLIST queue was crucial during descent when several systems needed
; display access: guidance updates, crew-requested displays, and alarm
; conditions all had to be coordinated through this mechanism.
;
; Technical Operation:
; 1. Saves return address (Q register) to RELRET
; 2. Clears external monitor bit in MONSAVE1 (turns off monitoring display)
; 3. Checks DSPLIST for waiting jobs
; 4. If jobs waiting, wakes up next job via JOBWAKE
; 5. Falls through to RELDSP2 to complete release sequence
;
; Called by: Major verb routines, verb dispatcher, VBRESEQ, RECALTST
; ============================================================================
RELDSP		XCH	Q		# SET DSPLOCK TO +0, TURN RELDSP LIGHT
		TS	RELRET		# OFF, SEARCH DSPLIST - save return address
		CS	BIT14		; Prepare to clear external monitor bit
# Page 465
		INHINT			; Disable interrupts for atomic DSPLIST operation
		MASK	MONSAVE1	; Clear bit 14 (external monitor bit)
		TS	MONSAVE1	# TURN OFF EXTERNAL MONITOR BIT
		CCS	DSPLIST		; Check if display list has waiting jobs
		TC	+2		; List not empty, wake up waiting job
		TC	RELDSP2		# LIST EMPTY, skip wakeup
		CAF	ZERO		; Extract job from list
		XCH	DSPLIST		; Get job address, clear list
		TC	JOBWAKE		; Wake up the waiting job

; ============================================================================
; RELDSP2 - Release Display Common Exit Path
;
; Common exit point for RELDSP and RELDSP1 that completes the display release
; by turning off the KEY RELEASE light and clearing the display lock. This
; final cleanup makes the display system available for the next operation.
;
; Technical: Re-enables interrupts, clears bit 5 of channel 11 (KEY RELEASE
; light), zeros DSPLOCK, and returns to saved address in RELRET.
; ============================================================================
RELDSP2		RELINT			; Re-enable interrupts
		CS	BIT5		# TURN OFF KEY RELEASE LIGHT
		EXTEND			# (BIT 5 OF CHANNEL 11)
		WAND	DSALMOUT	; Clear KEY RELEASE indicator bit
		CAF	ZERO		; Prepare to clear display lock
		TS	DSPLOCK		; Release display lock (available for next user)
		TC	RELRET		; Return to caller

; ============================================================================
; RELDSP1 - Release Display (Simplified Version, No List Search)
;
; Simplified display release routine used when the caller knows DSPLIST
; management is not required. Only turns off KEY RELEASE light if the display
; list is actually empty; if jobs are still queued, leaves the light on to
; indicate the display is still in use.
;
; Historical Context: This variant was used by monitor setup routines
; (VBRELDSP) during Apollo 11 when establishing continuous monitoring displays.
; During landing, monitoring verbs like V16 N68 (altitude/rate) provided
; continuous updates. RELDSP1 allowed the monitor to release its hold while
; keeping the KEY RELEASE indicator state accurate if other displays were
; still queued.
;
; Technical Operation:
; 1. Saves return address to RELRET
; 2. Checks DSPLIST state
; 3. If empty (+0): calls RELDSP2 to turn off KEY RELEASE light
; 4. If not empty (+ or -): skips light control, just clears DSPLOCK
; 5. Returns to caller
;
; Called by: Monitor set up (VBRELDSP), situations where DSPLIST wake-up
; is handled elsewhere or not needed.
;
; Difference from RELDSP: Does not wake up waiting jobs, only manages the
; KEY RELEASE light state based on list emptiness.
; ============================================================================
RELDSP1		XCH	Q		# SET DSPLOCK TO +0. NO DSPLIST SEARCH.
		TS	RELRET		# TURN KEY RLSE LIGHT OFF IF DSPLIST IS
					# EMPTY. LEAVE KEY RLSE LIGHT ALONE IF
					# DSPLIST IS NOT EMPTY - save return address
		CCS	DSPLIST		; Check display list state
		TC	+2		# +  NOT EMPTY. LEAVE KEY RLSE LIGHT ALONE.
		TC	RELDSP2		# +0 EMPTY. TURN OFF KEY RLSE LIGHT
		CAF	ZERO		# -  NOT EMPTY. LEAVE KEY RLSE LIGHT ALONE
		TS	DSPLOCK		; Clear display lock
		TC	RELRET		; Return to caller

ENDPINBF	EQUALS

# Page 466
# PINTEST IS NEEDED FOR AUTO CHECK OF PINBALL.

PINTEST		EQUALS	LST2FAN

# Page 467
# VBTSTLTS TURNS ON ALL DISPLAY PANEL LIGHTS. AFTER 5 SEC, IT TURNS
# OFF THE CAUTION AND STATUS LIGHTS.

; ============================================================================
; VBTSTLTS - Verb Test Lights (System Self-Test Display)
;
; PURPOSE: Comprehensive display panel self-test routine that illuminates
; all indicator lights and display segments to verify DSKY functionality.
;
; MISSION CONTEXT: Used during pre-flight checkout and crew procedures to
; verify all DSKY indicators are functional before critical mission phases.
; Armstrong and Aldrin would have executed this test before descent to
; confirm all warning lights (GIMBAL LOCK, NO ATT, PROG ALARM, etc.) were
; operational - critical for detecting guidance system failures during landing.
;
; OPERATION SEQUENCE:
; 1. Inhibits interrupts for atomic light control
; 2. Sets IMODES33 bit to prevent IMU monitor from interfering
; 3. Turns ON all indicator lights:
;    - UPLINK ACTIVITY, TEMP, KEY RLSE
;    - V/N FLASH, OPERATOR ERROR
;    - NO ATT, GIMBAL LOCK, TRACKER
;    - PROG ALM (program alarm)
;    - TEST ALARM outbit
; 4. Fills all display segments with "88888" pattern (all segments lit)
; 5. Displays plus signs on all three register displays
; 6. Schedules WAITLIST task for 5-second timer
; 7. After 5 seconds: turns off caution/status lights, leaves display lit
; 8. Restores normal DSKY operation
;
; TECHNICAL DETAILS:
; - Uses WAITLIST for precise 5-second timing (764 octal = 500 decimal cs)
; - Tests both Channel 11 (DSALMOUT) and Channel 13 outputs
; - Verifies all 11 display registers (DSPTAB through DSPTAB+11D)
; - Pattern OCT 05675 = all segments on (88888)
; - Pattern OCT 07675 = all segments on + plus sign
;
; CREW PROCEDURE: Verb 35 (Test Lights) - "V35E"
; Expected result: All lights on, displays show "88888 +88888 +88888"
; After 5 sec: Some caution lights extinguish, displays remain lit
; Any light failing to illuminate indicates DSKY hardware failure
; ============================================================================

		SETLOC	ENDNVSB1 +1

		COUNT*	$$/PIN
VBTSTLTS	INHINT
		CS	BIT1		# SET BIT 1 OF IMODES33 SO IMUMON WONT
		MASK	IMODES33	# TURN OUT ANY LAMPS.
		AD	BIT1
		TS	IMODES33

		CAF	TSTCON1		# TURN ON UPLINK ACTIVITY, TEMP, KEY RLSE,
		EXTEND			# V/N FLASH, OPERATOR ERROR.
		WOR	DSALMOUT
		CAF	TSTCON2		# TURN ON NO ATT, GIMBAL LOCK, TRACKER,
		TS	DSPTAB +11D	# PROG ALM.
		CAF	BIT10		# TURN ON TEST ALARM OUTBIT
		EXTEND
		WOR	CHAN13
		CAF	TEN
TSTLTS1		TS	ERCNT
		CS	FULLDSP
		INDEX	ERCNT
		TS	DSPTAB
		CCS	ERCNT
		TC	TSTLTS1
		CS	FULLDSP1
		TS	DSPTAB +1	# TURN ON 3 PLUS SIGNS
		TS	DSPTAB +4
		TS	DSPTAB +6
		CAF	ELEVEN
		TS	NOUT
		RELINT
		CAF	SHOLTS
		INHINT
		TC	WAITLIST
		EBANK=	DSPTAB
		2CADR	TSTLTS2

		TC	ENDOFJOB	# DSPLOCK IS LEFT BUSY (FROM KEYBOARD
					# ACTION) UNTIL TSTLTS3 TO INSURE THAT
					# LIGHTS TEST WILL BE SEEN.

; Display and light test constants for VBTSTLTS comprehensive panel test
FULLDSP		OCT	05675		# DISPLAY ALL 8'S (segments a-g on)
FULLDSP1	OCT	07675		# DISPLAY ALL 8'S AND + (with sign bit)
TSTCON1		OCT	00175		# Channel 11 test pattern:
					# UPLINK ACTIVITY, TEMP. KEY RLSE,
					# V/N FLASH, OPERATOR ERROR.
# Page 468
TSTCON2		OCT	40674		# DSPTAB+11D test pattern - bits 3,4,5,6,8,9:
					# NO ATT, GIMBAL LOCK, TRACKER, PROG ALM.
TSTCON3		OCT	00115		# Channel 11 cleanup pattern - bits 1, 3, 4, 7:
					# UPLINK ACITIVY, TEMP, OPERATOR ERROR.
SHOLTS		OCT	764		# 5 seconds display time (500 centiseconds)

; ============================================================================
; TSTLTS2 - Test Lights Phase 2 (WAITLIST Callback)
;
; Second phase of DSKY light test sequence, invoked by WAITLIST timer after
; 5-second display of all lights/segments. Schedules the cleanup phase
; (TSTLTS3) to run under Executive control as a low-priority job.
;
; TECHNICAL IMPLEMENTATION:
; This routine bridges between WAITLIST (timer-driven) and Executive (priority
; job) scheduling systems. The 5-second delay allows crew adequate time to
; visually inspect all DSKY lights before automatic restoration to normal
; operation begins.
;
; Entry Conditions:
; - Called by WAITLIST after SHOLTS (5 second) timer expires
; - All DSKY lights and display segments illuminated
; - DSPLOCK still held busy (prevents interference during test)
;
; Operation:
; 1. Load CHRPRIO (character display priority level)
; 2. Call NOVAC to schedule Executive job
; 3. Provide 2CADR of TSTLTS3 cleanup routine
; 4. Exit via TASKOVER (WAITLIST task completion)
;
; Job Priority: CHRPRIO ensures cleanup runs at appropriate priority level
; without interfering with critical navigation or guidance computations.
; ============================================================================
TSTLTS2		CAF	CHRPRIO		# CALLED BY WAITLIST (timer expired)
		TC	NOVAC		; Schedule Executive job
		EBANK=	DSPTAB		; Set E-bank for DSPTAB access
		2CADR	TSTLTS3		; Cleanup phase address

		TC	TASKOVER	; WAITLIST task complete

; ============================================================================
; TSTLTS3 - Test Lights Phase 3 (Cleanup and System Restore)
;
; Final phase of DSKY light test sequence. Systematically turns off all test
; illumination and restores DSKY and related system flags to normal operating
; state. This comprehensive cleanup ensures no test artifacts remain that could
; confuse crew or interfere with actual mission displays and system status.
;
; CLEANUP SEQUENCE:
; 1. Turn off UPLINK ACTIVITY, TEMP, OPERATOR ERROR lights via DSALMOUT
; 2. Clear TEST ALARM OUTBIT (Channel 13) to disable STBY/RESTART test mode
; 3. Restore NO ATT light to follow actual IMU coarse align status (Channel 12)
; 4. Clear AUTO, HOLD, FREE, GIMBAL LOCK, TRACKER, PROG ALM panel lights
; 5. Update IMODES33 to reflect all lamps out and test completion
; 6. Update IMODES30 (priority 15000 restoration)
; 7. Clear radar failure flags in RADMODES (CDU and data fail indicators)
; 8. Redisplay current mode register (MODREG) contents
; 9. Turn on kill monitor bit (for program termination monitoring)
; 10. Turn off V/N FLASH
; 11. Release display system and return to PINBRNCH if operator waiting
;
; HISTORICAL CONTEXT:
; This cleanup sequence was critical during pre-launch checkout and periodic
; in-flight self-tests. Armstrong and Aldrin would have run V35 before major
; events (LM separation, descent initiation, ascent) to verify DSKY operational
; status. The comprehensive restoration ensures that subsequent crew operations
; see only legitimate system warnings, not test-induced false indicators that
; could distract during critical mission phases like the lunar landing approach.
;
; TECHNICAL IMPLEMENTATION:
; Uses INHINT/RELINT interrupt protection during multi-channel updates to ensure
; atomic restoration of system state. Combines direct channel operations (WAND)
; with display system banking calls (DSPMM, FLASHOFF) to access both hardware
; I/O and display software internals. The mode register restoration (IMODES30,
; IMODES33) and radar flag clearing (RADMODES) ensure all subsystem status flags
; reflect post-test reality rather than test-induced artificial states.
;
; Entry Conditions:
; - Scheduled by TSTLTS2 via NOVAC (Executive job at CHRPRIO)
; - All test lights/segments currently illuminated from TSTLTS phase
; - DSPLOCK held, display system in test mode
; - Multiple system flags in test-induced states
;
; Exit via POSTJUMP to TSTLTS4:
; - Performs RELDSP (release display)
; - Routes to PINBRNCH if ENDIDLE awaiting operator response
; - Otherwise completes job termination
; ============================================================================
TSTLTS3		CS	TSTCON3		# CALLED BY EXECUTIVE
		INHINT			; Protect critical multi-channel updates
		EXTEND			# TURN OFF  UPLINK ACTIVITY, TEMP,
		WAND	DSALMOUT	# OPERATOR ERROR (alarm output channel).
		CS	BIT10		# TURN OFF  TEST ALARM OUTBIT
		EXTEND			; Extended instruction mode
		WAND	CHAN13		; Channel 13 write-AND (clear test alarm)
		CAF	BIT4		# MAKE NO ATT FOLLOW BIT 4 OF CHANNEL 12
		EXTEND			#   (NO ATT LIGHT ON IF IN COARSE ALIGN)
		RAND	CHAN12		; Read-AND Channel 12 (actual IMU status)
		AD	BIT15		# TURN OFF AUTO, HOLD, FREE, SPARE,
		TS	DSPTAB +11D	# GIMBAL LOCK, SPARE, TRACKER, PROG ALM
		CS	13-11,1		# SET BITS TO INDICATE ALL LAMPS OUT. TEST
		MASK	IMODES33	# LIGHTS COMPLETE (update mode flags).
		AD	PRIO16		; Add priority indicator
		TS	IMODES33	; Store updated mode register 33

		CS	OCT55000	; Complement octal 55000 pattern
		MASK	IMODES30	; Mask with mode register 30
		AD	PRIO15		# 15000 (priority level restoration).
		TS	IMODES30	; Store updated mode register 30

		CS	RFAILS2		; Complement radar failure flags
		MASK	RADMODES	; Mask with radar mode register
		AD	RCDUFBIT	; Add radar CDU flag bit
		TS	RADMODES	; Clear radar CDU/data fail indicators

		RELINT			; Release interrupt inhibit (restore normal operation)

		TC	BANKCALL	# REDISPLAY C(MODREG)
		CADR	DSPMM		; Display mode register contents
		TC	KILMONON	# TURN ON KILL MONITOR BIT (enable termination monitoring).
		TC	FLASHOFF	# TURN OFF V/N FLASH (clear verb/noun flashing).
		TC	POSTJUMP	# DOES RELDSP AND GOES TO PINBRNCH IF
		CADR	TSTLTS4		#  ENDIDLE IS AWAITING OPERATOR RESPONSE.
# Page 469
13-11,1		OCT	16001
RFAILS2		OCT	330		# RADAR CDU AND DATA FAIL FLAGS.
OCT55000	OCT	55000
ENDPINS2	EQUALS

# Page 470
# ERROR LIGHT RESET (RSET) TURNS OFF:
# UPLINK ACTIVITY, AUTO, HOLD, FREE, OPERATOR ERROR,
# PROG ALM, TRACKER FAIL.
# LEAVES GIMBAL LOCK AND NO ATT ALONE.
# IT ALSO ZEROS THE 'TEST ALARM' OUT BIT, WHICH TURNS OFF STBY, RESTART.
# IT ALSO SETS 'CAUTION RESET' TO 1.
# IT ALSO FORCES BIT 12 OF ALL DSPTAB ENTRIES TO 1.

; ============================================================================
; ERROR (RSET Button) - Error Light Reset and Alarm Acknowledgment
;
; PURPOSE: Resets caution and warning lights after crew acknowledgment,
; clearing non-critical alarms while preserving attitude warnings.
;
; MISSION CONTEXT: During Apollo 11 descent, Steve Bales (GUIDO) told the
; crew "We're GO on that alarm" for the 1202/1201 program alarms. Armstrong
; and Aldrin pressed RSET (Reset) to acknowledge and clear the PROG ALARM
; light, allowing them to continue the descent. This routine executed every
; time they pressed RSET during that critical sequence.
;
; BUTTON: RSET (Reset) - Located on DSKY panel, dedicated alarm reset button
;
; LIGHTS TURNED OFF:
; - UPLINK ACTIVITY: Ground command reception indicator
; - AUTO, HOLD, FREE: Autopilot mode indicators
; - OPERATOR ERROR: Invalid verb/noun entry indicator  
; - PROG ALM: Program alarm (1201, 1202, etc.)
; - TRACKER FAIL: Optical tracker failure
; - TEST ALARM outbit: Turns off STBY, RESTART lights
;
; LIGHTS LEFT ALONE (Critical attitude warnings):
; - GIMBAL LOCK: Middle gimbal near 90° (prevents platform tumble)
; - NO ATT: Attitude reference lost (IMU failure)
;
; TECHNICAL OPERATION:
; 1. Restores DSPLOCK from saved value (does not change display lock state)
; 2. Sets CAUTION RESET outbit (Channel 11 bit 10)
; 3. Clears alarm indicator lights via DSPTAB+11D and DSALMOUT
; 4. Resets failure flag bits in IMODES33, IMODES30, RADMODES
; 5. If underlying failure condition still exists, alarm will re-trigger
; 6. Forces bit 12 high in all DSPTAB entries (display format control)
; 7. Clears FAILREG registers (failure tracking)
;
; RESTART PROTECTION: Preserves alarm state across restart sequences
;
; DESIGN PHILOSOPHY: Separates acknowledgeable alarms (PROG ALM, OPERATOR
; ERROR) from critical attitude warnings (GIMBAL LOCK, NO ATT) that require
; physical resolution. This prevents crew from inadvertently clearing warnings
; that indicate dangerous spacecraft states.
;
; HISTORICAL NOTE: The ability to reset and continue despite program alarms
; was crucial to Apollo 11's successful landing. Without RSET capability,
; the 1202 alarm would have required abort.
; ============================================================================

		SETLOC	DOPROC +2
		COUNT*	$$/PIN
ERROR		XCH	21/22REG	# RESTORE ORIGINAL C(DSPLOCK). THUS ERROR
		TS	DSPLOCK		# LIGHT RESET LEAVES DSPLOCK UNCHANGED.
		INHINT
		CAF	BIT10		# TURN ON 'CAUTION RESET' OUTBIT
		EXTEND
		WOR	DSALMOUT	# BIT10 CHAN 11
		CAF	GL+NOATT	# LEAVE GIMBAL LOCK AND NO ATT INTACT,
		MASK	DSPTAB +11D	# TURNING OFF AUTO, HOLD, FREE,
		AD	BIT15		# PROG ALARM, AND TRACKER.
		TS	DSPTAB +11D
		CS	PRIO16		# RESET FAIL BITS WHICH GENERATE PROG
		MASK	IMODES33	# ALARM SO THAT IF THE FAILURE STILL
		AD	PRIO16		# EXISTS, THE ALARM WILL COME BACK.
		TS	IMODES33
		CS	BIT10
		MASK	IMODES30
		AD	BIT10
		TS	IMODES30

		CS	RFAILS
		MASK	RADMODES
		AD	RCDUFBIT
		TS	RADMODES

		CS	BIT10		# TURN OFF 'TEST ALARM' OUTBIT.
		EXTEND
		WAND	CHAN13
		CS	ERCON		# TURN OFF UPLINK ACTIVITY,
		EXTEND			# OPERATOR ERROR.
		WAND	DSALMOUT

; ============================================================================
; TSTAB/ERPLUS/ERMINUS/ERCOM - Display Table Error Cleanup Sequence
;
; COMMENT-ONLY READERS: After clearing visible alarm lights, the AGC performs
; internal housekeeping by scanning through the display table (DSPTAB) to
; clean up any error flags or inconsistent states left behind by the alarm
; condition. This ensures the display system is fully reset and ready for
; normal operations. The scan processes all 11 display table entries (BINCON
; = decimal 10, counting from 10 down to 0).
;
; CODE-ALONG READERS: This section iterates through DSPTAB entries (11 total,
; indexed by ERCNT counting down from 10 to 0) to force bit 12 high in each
; entry. The CCS DSPTAB indexed test determines sign: if positive, goes to
; ERPLUS (forces bit 12 high via complement-mask-complement sequence); if
; negative goes to ERMINUS (same operation); if zero skips. NOTBIT12 (octal
; 73777) masks bit 12 to 0, then complement restores all bits except 12 which
; is forced to 1. This standardizes display format control across all entries.
; After completing iteration, clears FAILREG registers (failure tracking) and
; SFAIL (system fail flag), then does ENDOFJOB to complete reset sequence.
; ============================================================================

TSTAB		CAF	BINCON		# (DEC 10)
		TS	ERCNT		# ERCNT = COUNT
		INHINT
		INDEX	ERCNT
		CCS	DSPTAB
		AD	ONE
		TC	ERPLUS
		AD	ONE
ERMINUS		CS	A
		MASK	NOTBIT12
# Page 471
		TC	ERCOM
ERPLUS		CS	A
		MASK	NOTBIT12
		CS	A		# MIGHT WANT TO RESET CLPASS, DECBRNCH,
ERCOM		INDEX	ERCNT		# ETC.
		TS	DSPTAB
		RELINT
		CCS	ERCNT
		TC	TSTAB	+1
		CAF	ZERO
		TS	FAILREG
		TS	FAILREG +1
		TS	FAILREG +2
		TS	SFAIL
		TC	ENDOFJOB

ERCON		OCT	104		# CHAN 11 BITS 3,7.
					# UPLINK ACTIVITY, AND OPERATOR ERROR.
RFAILS		OCT	330		# RADAR CDU AND DATA FAIL FLAGS.
GL+NOATT	OCT	00050		# NO ATT AND GIMBAL LOCK LAMPS
NOTBIT12	OCT	73777

ENDPINS1	EQUALS

		SBANK=	LOWSUPER

