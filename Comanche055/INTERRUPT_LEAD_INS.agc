# Copyright:	Public domain.
# Filename:	INTERRUPT_LEAD_INS.agc
# Purpose:	Part of the source code for Comanche, build 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 131-132
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	09/05/09 FB	Transcription of Batch FB-1 Assignment.
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
; FILE: INTERRUPT_LEAD_INS.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Interrupt vector table and entry points establishing AGC interrupt
;        priority structure. Defines interrupt levels (T4RUPT, T3RUPT, DSRUPT,
;        KEYRUPT, UPRUPT) with priority ordering, entry point mechanics, and
;        context save/restore requirements fundamental to real-time OS operation
;        throughout Apollo 11.
;
; COMMENT-ONLY READERS: This file defined how the computer handled urgent tasks
;        by interrupting normal operations when needed.
; CODE-ALONG READERS: Study interrupt vector table organization, priority levels,
;        entry point implementation, context save/restore mechanisms, AGC interrupt
;        architecture.
; ============================================================================

# Page 131
		SETLOC	4000

		COUNT	02/RUPTS

; ============================================================================
; INTERRUPT VECTOR TABLE
;
; The AGC interrupt system provides real-time response to hardware events
; throughout the Apollo 11 mission. When a hardware interrupt occurs, the
; AGC automatically saves the program counter in register Z and jumps to
; the corresponding vector location below. Each interrupt handler must save
; the A register (and L register for double-precision operations) before
; processing, then restore context before returning.
;
; INTERRUPT PRIORITY LEVELS (highest to lowest):
;   1. GO (GOPROG) - Restart/Fresh Start (highest priority)
;   2. T6RUPT - Timer 6 interrupt
;   3. T5RUPT - Timer 5 interrupt (0.5 second clock)
;   4. T3RUPT - Timer 3 interrupt
;   5. T4RUPT - Timer 4 interrupt (10ms, drives WAITLIST scheduler)
;   6. KEYRUPT1 - DSKY keyboard interrupt
;   7. KEYRUPT2 - Mark button interrupt
;   8. UPRUPT - Uplink data from ground control
;   9. DOWNRUPT - Downlink telemetry
;  10. RADAR RUPT - Radar data ready
;  11. HAND CONTROL RUPT - Manual control input (lowest priority, not used in CM)
;
; Each entry uses 3 instructions: DXCH ARUPT (save A,L), load bank, transfer
; control to actual interrupt handler in the specified bank.
; ============================================================================

; GO (GOPROG) - Restart/Fresh Start interrupt (highest priority)
; This interrupt vector handles system restart conditions including power-on
; initialization and commanded restarts. INHINT disables further interrupts,
; loads the bank containing GOPROG, and transfers control for restart processing.
		INHINT			# GO
		CAF	GOBB
		XCH	BBANK
		TCF	GOPROG

; T6RUPT - Timer 6 interrupt
; Context save: DXCH ARUPT saves A,L registers to ARUPT,ARUPT+1.
; Uses DTCB (Double Transfer Control via Both banks) for dual-bank operation.
		DXCH	ARUPT		# T6RUPT
		EXTEND
		DCA	T6LOC
		DTCB

; T5RUPT - Timer 5 interrupt (0.5 second interval timer)
; This interrupt occurs every 0.5 seconds throughout the mission for timing
; tasks that require less frequent execution than the 10ms T4RUPT cycle.
; Checks TIME5 counter against 0.5 second threshold before processing.
		DXCH	ARUPT		# T5RUPT
		CS	TIME5
		AD	.5SEC
		TCF	T5RUPT

; T3RUPT - Timer 3 interrupt
; Context save: DXCH ARUPT preserves A,L registers.
; Bank switching: Loads T3RPTBB into BBANK register for memory bank selection,
; then transfers control to T3RUPT handler in the newly selected bank.
		DXCH	ARUPT		# T3RUPT
		CAF	T3RPTBB
		XCH	BBANK
		TCF	T3RUPT

; T4RUPT - Timer 4 interrupt (10 millisecond cycle, HIGHEST FREQUENCY)
; This is the critical timing interrupt that drives the WAITLIST scheduler.
; Occurs every 10ms throughout Apollo 11 mission. During the descent that
; triggered the famous 1202 program alarm, this interrupt was executing
; at maximum capacity while processing landing radar data.
; Priority: After GO/T6/T5/T3 but before keyboard/uplink/downlink.
		DXCH	ARUPT		# T4RUPT
		CAF	T4RPTBB
		XCH	BBANK
		TCF	T4RUPT

; KEYRUPT1 - DSKY keyboard interrupt
; Triggered when astronauts press keys on the Display and Keyboard (DSKY)
; unit during mission operations. This interrupt processes verb/noun entries,
; numerical data input, and control keys (ENTR, RSET, KEY REL, etc.).
; Critical during manual data entry phases throughout Apollo 11 mission.
		DXCH	ARUPT		# KEYRUPT1
		CAF	KEYRPTBB
		XCH	BBANK
		TCF	KEYRUPT1

; KEYRUPT2 - Mark button interrupt (MARKRUPT)
; Triggered when astronauts press the MARK button during optical navigation
; sightings with the sextant or scanning telescope. Records precise timing
; of star or landmark observations for navigation state updates.
		DXCH	ARUPT		# KEYRUPT2
		CAF	MKRUPTBB
		XCH	BBANK
		TCF	MARKRUPT

; UPRUPT - Uplink interrupt (ground control data reception)
; Processes commands and data uplinked from Mission Control in Houston.
; During Apollo 11, ground controllers used uplink to send state vector
; updates, target parameters, and mission timeline adjustments to the AGC.
		DXCH	ARUPT		# UPRUPT
		CAF	UPRPTBB
		XCH	BBANK
		TCF	UPRUPT

; DOWNRUPT - Downlink telemetry interrupt
; Formats and transmits AGC data to Mission Control via telemetry downlink.
; Continuously streams spacecraft state, navigation data, program status,
; and alarm information to ground controllers throughout the mission.
		DXCH	ARUPT		# DOWNRUPT
		CAF	DWNRPTBB
		XCH	BBANK
		TCF	DODOWNTM

; RADAR RUPT - Radar data ready interrupt
; Triggered when VHF ranging system has new data available. Processes
; range and range-rate measurements for rendezvous navigation operations.
		DXCH	ARUPT		# RADAR RUPT
# Page 132
		CAF	RDRPTBB
		XCH	BBANK
		TCF	VHFREAD

; HAND CONTROL RUPT - Manual control input interrupt (LOWEST PRIORITY)
; Designed to process manual attitude control inputs from astronaut hand
; controllers. Not actively used in Command Module (CM) configuration.
; Lowest priority ensures manual inputs do not interfere with critical
; automated guidance, navigation, and control functions.
		DXCH	ARUPT		# HAND CONTROL RUPT
		CAF	HCRUPTBB
		XCH	BBANK
		TCF	RESUME +3	# NOT USED

; ============================================================================
; BANK CONFIGURATION CONSTANTS (BBCON)
;
; The AGC memory architecture uses fixed-memory banks (36K words of core rope
; ROM divided into banks) and erasable-memory banks (2K words RAM divided into
; banks). The BBCON directive defines both the fixed-bank location of an
; interrupt handler and the erasable-bank (EBANK) to be selected when entering
; that handler.
;
; Each interrupt handler resides in a specific fixed-memory bank, and may
; require access to specific erasable-memory locations in a particular erasable
; bank. The BBCON values below encode this bank information, loaded into BBANK
; by the interrupt vector code above via CAF/XCH instructions.
;
; CONTEXT SAVE/RESTORE MECHANISM:
; - DXCH ARUPT: Saves A,L registers to locations ARUPT,ARUPT+1 (preserves
;   computation state across interrupt)
; - CAF xxxBB: Loads bank configuration constant for interrupt handler
; - XCH BBANK: Switches to bank containing interrupt handler code
; - TCF xxxRUPT: Transfers control to handler
; - Handler completes with RESUME instruction, restoring A,L,Q,BBANK,Z
; ============================================================================

; GOPROG bank configuration - References LST1 in erasable bank E0/E3.
; Restart processing requires access to task lists and system state variables.
		EBANK=	LST1		# RESTART USES E0,E3
GOBB		BBCON	GOPROG

; T3RUPT bank configuration - References LST1 in erasable bank.
		EBANK=	LST1
T3RPTBB		BBCON	T3RUPT

; KEYRUPT1 bank configuration - References KEYTEMP1 in erasable bank.
; Keyboard interrupt processing uses KEYTEMP1 for temporary key code storage.
		EBANK=	KEYTEMP1
KEYRPTBB	BBCON	KEYRUPT1

; MARKRUPT bank configuration - References MRKBUF1 in erasable bank.
; Mark button interrupt uses MRKBUF1 to buffer optical sighting timing data.
		EBANK=	MRKBUF1
MKRUPTBB	BBCON	MARKRUPT

; UPRUPT bank configuration - Same as KEYRUPT (shared erasable bank usage).
UPRPTBB		=	KEYRPTBB

; DODOWNTM bank configuration - References DNTMBUFF in erasable bank.
; Downlink telemetry interrupt uses DNTMBUFF for telemetry data buffering.
		EBANK=	DNTMBUFF
DWNRPTBB	BBCON	DODOWNTM

; VHFREAD bank configuration - References DATATEST in erasable bank.
; Radar interrupt processing uses DATATEST for radar data validation.
		EBANK=	DATATEST
RDRPTBB		BBCON	VHFREAD

; HCRUPT bank configuration - References TIME1 in erasable bank (not used in CM).
		EBANK=	TIME1
HCRUPTBB	BBCON	RESUME		# NOT USED

; T4RUPT bank configuration - References DSRUPTSW in erasable bank.
; The critical 10ms timer interrupt requires DSRUPTSW for display interrupt control.
		EBANK=	DSRUPTSW
T4RPTBB		BBCON	T4RUPT

; T5RUPT bank configuration - References TIME1 in erasable bank.
; The 0.5 second timer interrupt uses TIME1 for mission elapsed time tracking.
		EBANK=	TIME1
T5RPTBB		BBCON	T5RUPT

; ============================================================================
; T5RUPT INTERRUPT HANDLER
;
; The 0.5 second timer interrupt handler. After TIME5 threshold check in the
; vector table above, control arrives here for actual T5RUPT processing.
; Uses BZMF (Branch Zero or Minus to Fixed) to conditionally branch based on
; accumulator state, then loads T5LOC and uses DTCB for dual-bank transfer.
; ============================================================================

T5RUPT		EXTEND
		BZMF	NOQBRSM
		EXTEND
		DCA	T5LOC
		DTCB

