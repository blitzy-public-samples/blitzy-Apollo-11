# Copyright:    Public domain.
# Filename:     DOWN-TELEMETRY_PROGRAM.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1093-1102
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-08 RSB	Adapted from Colossus249/ file of same name
#				and page images. Corrected various typos
#				in the transcription of program comments,
#				and these should be back-ported to
#				Colossus249.
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#       Assemble revision 055 of AGC program Comanche by NASA
#       2021113-051.  April 1, 1969.
#
#       This AGC program shall also be referred to as Colossus 2A
#
#       Prepared by
#                       Massachusetts Institute of Technology
#                       75 Cambridge Parkway
#                       Cambridge, Massachusetts
#
#       under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: DOWN-TELEMETRY_PROGRAM.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Downlink telemetry formatting program packaging spacecraft data for
;        ground station transmission. Implements packet structure assembly,
;        data selection priority, transmission scheduling, and MSFN (Manned 
;        Space Flight Network) communication protocol enabling continuous 
;        Mission Control monitoring of Apollo 11 spacecraft status throughout
;        all mission phases from launch through splashdown.
;
; COMMENT-ONLY READERS: This program formatted and transmitted spacecraft data
;        to Mission Control in Houston for continuous monitoring. Every 20
;        milliseconds, the AGC packaged navigation data, system status, and
;        crew displays into telemetry packets sent to Earth via S-band radio.
; CODE-ALONG READERS: Study telemetry formatting algorithms, interrupt-driven
;        transmission scheduling (50 Hz downrupt rate), packet assembly using
;        control lists and sublists, and the erasable memory dump capability
;        accessible via Verb 74 for ground-based troubleshooting.
; ============================================================================

# Page 1093
; ============================================================================
; TRANSITION: Down-Telemetry Program - Core Communication System
;
; The Down-Telemetry Program forms the communication backbone between the
; Apollo spacecraft and Mission Control in Houston. Every 20 milliseconds,
; triggered by a telemetry end pulse, this program packages critical spacecraft
; data into standardized formats and transmits them to Earth via the S-band
; antenna. During Apollo 11's journey, this continuous data stream enabled
; flight controllers to monitor navigation state, system health, crew displays,
; and computer status in near real-time despite the 1.3-second signal delay
; from lunar distance.
; ============================================================================
;
# PROGRAM NAME- DOWN TELEMETRY PROGRAM
# MOD NO.- 0		TO COMPLETELY REWRITE THE DOWN TELEMETRY PROGRAM AND DOWNLINK ERASABLE DUMP PROGRAM FOR THE
#			PURPOSE OF SAVING APPROXIMATELY 150 WORDS OF CORE STORAGE.
#			THIS CHANGE REQUIRES AN ENTIRELY NEW METHOD OF SPECIFYING DOWNLINK LISTS.  REFER TO DOWNLINK
#			LISTS LOG SECTION FOR MORE DETAILS.  HOWEVER THIS CHANGES WILL NOT AFFECT THE GROUND PROCESSING
#			OF DOWN TELEMETRY DATA.
# MOD BY-  KILROY, SMITH, DEWITT
# DATE-	   02OCT67
# AUTHORS- KILROY, SMITH, DWWITT, DEWOLF, FAGIN
# LOG SECTION- DOWN-TELEMETRY PROGRAM
;
; TELEMETRY TRANSMISSION SCHEDULING AND TIMING:
; The spacecraft telemetry system operates on a precise 50 Hz schedule,
; generating a hardware interrupt ("downrupt") every 20 milliseconds. This
; interrupt-driven architecture ensures that telemetry transmission never
; interferes with time-critical guidance and navigation computations. Each
; downrupt loads two AGC words into output channels 34 and 35, which are
; then serialized and transmitted to Earth via the S-band high-gain antenna
; or omni antennas depending on spacecraft attitude and mission phase.
;
# FUNCTIONAL DESCRIPTION- THIS ROUTINE IS INITIATED BY TELEMETRY END
#	PULSE FROM THE DOWNLINK TELEMETRY CONVERTER.  THIS PULSE OCCURS
#	AT 50 TIMES PER SEC(EVERY 20 MS) THEREFORE DODOWNTM IS
#	EXECUTED AT THESE RATES.  THIS ROUTINE SELECTS THE APPROPRIATE
#	AGC DATA TO BE TRANSMITTED DOWNLINK AND LOADS IT INTO OUTPUT
#	CHANNELS 34 AND 35.  THE INFORMATION IS THEN GATED OUT FROM THE
#	LGC IN SERIAL FASHION.
;
; DOWNLINK DATA CAPACITY AND STRUCTURE:
; With 50 downrupts per second and 2 words per downrupt, the system achieves
; a steady downlink rate of 100 words per second or 200 words every 2 seconds.
; This 2-second cycle matches the standard telemetry frame duration, allowing
; ground stations to synchronize data processing and display updates for flight
; controllers monitoring the mission.
;
#	THIS PROGRAM IS CODED FOR A 2 SECOND DOWNLIST.  SINCE DOWNRUPTS
#	OCCUR EVERY 20MS AND 2 AGC COMPUTER WORDS CAN BE PLACED IN
#	CHANNELS 34 AND 35 DURING EACH DOWNRUPT THE PROGRAM IS CAPABLE
# 	OF SENDING 200 AGC WORDS EVERY 2 SECONDS.
# CALLING SEQUENCE- NONE
#	PROGRAM IS ENTERED VIA TCF DODOWNTM WHICH IS EXECUTED AS A
#	RESULT OF A DOWNRUPT.  CONTROL IS RETURNED VIA TCF RESUME WHICH
#	IN EFFECT IS A RESUME.
# SUBROUTINES CALLED- NONE
# NORMAL EXIT MODE- TCF RESUME
# ALARM OR ABORT EXIT MODE- NONE
# RESTART PROTECTION:
#	ON A FRESH START AND RESTART THE 'STARTSUB' SUBROUTINE WILL INITIALIZE THE DOWNLIST POINTER(ACTUALLY
#	DNTMGOTO) TO THE BEGINNING OF THE CURRENT DOWNLIST(I.E., CURRENT CONTENTS OF DNLSTADR).  THIS HAS THE
#	EFFECT OF IGNORING THE REMAINDER OF THE DOWNLIST WHICH THE DOWN-TELEMETRY PROGRAM WAS WORKING ON WHEN
#	THE RESTART(OR FRESH START) OCCURRED AND RESUME DOWN TELEMETRY FROM THE BEGINNING OF THE CURRENT
#	DOWNLIST.
#	ALSO OF INTEREST IS THE FACT THAT ON A RESTART THE AGC WILL ZERO DOWNLINK CHANNELS 13, 34 AND 35.
;
; DOWNLINK LIST SELECTION AND ADAPTATION TO MISSION PHASE:
; The telemetry program automatically adapts its data transmission based on
; current mission operations. Different mission programs require different
; telemetry priorities - powered flight needs engine and guidance data, while
; coasting phases emphasize navigation state. The downlink list pointer
; (DNLSTADR) determines which predefined list structure is active.
;
# DOWNLINK LIST SELECTION:
#	THE APPROPRIATE DOWNLINK LISTS ARE SELECTED BY THE FOLLOWING:
#	1. FRESH START
;	   Fresh start initializes to the nominal CSM standby downlist, providing
;	   basic spacecraft status when no specific program is running.
#
#	2. V37EXXE WHERE XX = THE MAJOR MODE BEING SELECTED.
;	   When the crew selects a major mode (program) via DSKY Verb 37, the
;	   appropriate downlink list for that program automatically loads. For
;	   example, selecting P40 (SPS burn) loads the powered flight downlist.
;
#	3. UPDATE PROGRAM(P27)
;	   Program 27 (update program) loads a specialized downlist optimized
;	   for receiving state vector updates from Mission Control.
;
#	4.	NON-V37 SELECTABLE TYPE PROGRAMS(E.G. AGS INITIALIZATION(SUNDANCE,LUMINARY) AND P61-P62
#		TRANSITION(COLOSSUS) ETC.).
;	   Certain programs change downlists directly without crew V37 selection,
;	   such as entry program transitions or abort mode activations.
# DOWNLINK LIST RULES AND LIMITATIONS:
#	READ SECTION(S) WHICH FOLLOW 'DEBRIS' WRITEUP.
;
; TELEMETRY BANDWIDTH AND GROUND STATION PROCESSING:
; The 200-word per 2-second transmission rate provides Mission Control with
; comprehensive spacecraft monitoring capability. Ground computers at MSFN
; (Manned Space Flight Network) stations receive this data stream, decommutate
; the telemetry format, and distribute values to flight controller consoles.
; During Apollo 11's mission, controllers in Houston monitored these displays
; continuously, watching for anomalies and providing go/no-go decisions at
; critical mission phases.
;
# OUTPUT- EVERY 2 SECONDS 100 DOUBLE PRECISION WORDS(I.E. 200 LGC
#	COMPUTER WORDS) ARE TRANSMITTED VIA DOWNLINK.
# ERASABLE INITIALIZATION REQUIRED- NONE
#	'DNTMGOTO' AND 'DNLSTADR' ARE INITIALIZED BY THE FRESH START PROGRAM.
;
; TELEMETRY BUFFER MANAGEMENT:
; The program uses a working buffer (DNTMBUFF) to assemble telemetry packets
; before transmission. Snapshot sublists employ this buffer to capture a
; time-coherent view of multiple memory locations, ensuring ground processing
; sees consistent data even though individual words transmit over multiple
; 20-millisecond intervals.
;
# DEBRIS(ERASABLE LOCATIONS DESTROYED BY THIS PROGRAM)-
#	LDATALST,DNTMBUFF TO DNTMBUFF +21D,TMINDEX,DNQ.
# Page 1094
# (No source on this page of the original assembly listing.)

# Page 1095
; ============================================================================
; INTERRUPT-DRIVEN EXECUTION MODEL
;
; DODOWNTM executes as a hardware interrupt handler, triggered by a telemetry
; end pulse from the spacecraft's downlink telemetry converter. This interrupt
; occurs precisely every 20 milliseconds (50 Hz), making telemetry transmission
; a predictable, time-deterministic process independent of other AGC software
; activity. The interrupt priority ensures telemetry doesn't block critical
; guidance computations, but does execute frequently enough to maintain
; continuous ground station communications.
; ============================================================================
;
# DODOWNTM IS ENTERED EVERY 20 MS BY AN INTERRUPT TRIGGERED BY THE
# RECEIPT OF AN ENDPULSE FROM THE SPACECRAFT TELEMETRY PROGRAMMER.
#
; DOWNLINK LIST ARCHITECTURE AND MEMORY ORGANIZATION:
; The telemetry system uses a two-level data structure: Control Lists define
; the overall transmission sequence for a mission phase, while Sublists contain
; the actual memory addresses to sample. This modular design allows multiple
; Control Lists to reference common Sublists (e.g., IMU data, state vectors),
; minimizing code storage requirements. The rewrite described in Mod 0 saved
; approximately 150 words of precious fixed memory through this optimization.
;
# NOTES REGARDING DOWNLINK LISTS ASSOCIATED WITH THIS PROGRAM:
# 1. DOWNLISTS.  - DOWNLISTS MUST BE COMPILED IN THE SAME BANK AS THE
#    DOWN-TELEMETRY PROGRAM.  THIS IS DONE FOR EASE OF CODING, FASTER
#    EXECUTION.
;    Memory bank co-location eliminates bank-switching overhead during the
;    interrupt handler, ensuring predictable execution timing within the
;    20-millisecond interrupt window.
;
# 2. EACH DOWNLINK LIST CONSISTS OF A CONTROL LIST AND A NUMBER OF
#    SUBLISTS.
;    Control Lists orchestrate the 2-second transmission cycle, specifying
;    which Sublists to send and in what order. Sublists define memory
;    locations to sample, using special op codes (1DNADR, DNCHAN, DNPTR)
;    interpreted by the telemetry program.
;
# 3. A SUBLIST REFERS TO A SNAPSHOT OR DATA COMMON TO THE SAME OR OTHER
#    DOWNLINK LISTS.  ANY SUBLIST CONTAINING COMMON DATA NEEDS TO BE
#    CODED ONLY ONCE FOR THE APPLICABLE DOWNLINK LISTS.
;    For example, a navigation state vector Sublist might be referenced by
;    the powered flight downlist, orbital coast downlist, and rendezvous
;    downlist - defined once, reused three times.
# 4. SNAPSHOT SUBLISTS REFER SPECIFICALLY TO HOMOGENOUS DATA WHICH MUST BE
#    SAVED IN A BUFFER DURING ONE DOWNRUPT.
# 5. THE 1DNADR FOR THE 1ST WORD OF SNAPSHOT DATA IS FOUND AT THE END
#    OF EACH SNAPSHOT SUBLIST, SINCE THE PROGRAM CODING SENDS THIS DP WORD
#    IMMEDIATELY AFTER STORING THE OTHERS IN THE SNAPSHOT BUFFER.
# 6. ALL LISTS ARE COMBINATIONS OF CODED ERASABLE ADDRESS CONSTANTS
#    CREATED FOR THE DOWNLIST PROGRAM.
#    A. 1DNADR			1-WORD DOWNLIST ADDRESS.
#	SAME AS ECADR, BUT USED WHEN THE WORD ADDRESSED IS THE LEFT
#	HALF OF A DOUBLE-PRECISION WORD FOR DOWN TELEMETRY.
#    B. 2DNADR - 6DNADR		N-WORD DOWNLIST ADDRESS, N = 2 - 6.
#	SAME AS 1DNADR, BUT WITH THE 4 UNUSED BITS OF THE ECADR FORMAT
#	FILLED IN WITH 0001-0101.  USED TO POINT TO A LIST OF N DOUBLE-
#	PRECISION WORDS, STORED CONSECUTIVELY, FOR DOWN TELEMETRY.
#    C. DNCHAN			DOWNLIST CHANNEL ADDRESS.
#	SAME AS 1DNADR, BUT WITH PREFIX BITS 0111.  USED TO POINT TO
#	A PAIR OF CHANNELS FOR DOWN TELEMETRY.
#    D. DNPTR			DOWN TELEMETRY SUBLIST POINTER.
#	SAME AS CAF BUT TAGGES AS A CONSTANT.  USED IN CONTROL LIST TO POINT TO A SUBLIST.
#	CAUTION--- A DNPTR CANNOT BE USED IN A SUBLIST.
# 7. THE WORD ORDER CODE IS SET TO ZERO AT THE BEGINNING OF EACH DOWNLIST (I.E., CONTROL LIST) AND WHEN
#    A '1DNADR TIME2' IS DETECTED IN THE CONTROL LIST(ONLY).
# 8. IN THE SNAPSHOT SUBLIST ONLY, THE DNADR'S CANNOT POINT TO THE FIRST WORD OF ANY EBANK.
#
# DOWNLINK LIST RESTRICTIONS:
# (THE FOLLOWING POINTS MAY BE LISTED ELSEWHERE BUT ARE LISTED HERE SO IT IS CLEAR THAT THESE THINGS CANNOT BE
# DONE)
#
# 1. SNAPSHOT DOWNLIST:
#    (A) CANNOT CONTAIN THE FOLLOWING ECADRS(I.E. 1DNADR'S): 0, 400, 1000, 1400, 2000, 2400, 3000, 3400.
#    (B) CAN CONTAIN ONLY 1DNADR'S
#
# 2. ALL DOWNLINKED DATA(EXCEPT CHANNELS) IS PICKED UP BY A <DCA<SO DOWNLINK LISTS CANNOT CONTAIN THE
#    EQUIVALENT OF THE FOLLOWING ECADRS(I.E. IDNADRS): 377, 777, 1377, 1777, 2377, 2777, 3377, 3777.
#
#    (NOTE: THE TERM EQUIVALENT ' MEANT THAT THE IDNADR TO 6DNADR WILL BE PROCESSED LIKE 1 TO 6 ECADRS)
#
# 3. CONTROL LISTS AND SUBLISTS CANNOT HAVE ENTRIES = OCTAL 00000 OR OCTAL 77777
# Page 1096
# 4. THE '1DNADR TIME2' WHICH WILL CAUSE THE DOWNLINK PROGRAM TO SET THE WORDER CODE TO 3 MUST APPEAR IN THE
#    CONTROL SECTION OF THE DOWNLIST.
#
# 5. 'DNCHAN 0' CANNOT BE USED.
#
# 6. 'DNPTR 0' CANNOT BE USED.
#
# 7. DNPTR CANNOT APPEAR IN A SUBLIST.
#
#
#
#
# EBANK SETTINGS
#	IN THE PROCESS OF SETTING THE EBANK(WHEN PICKING UP DOWNLINK DATA) THE DOWN TELEMETRY PROGRAM PUTS
#	'GARBAGE' INTO BITS15-12 OF EBANK.  HUGH BLAIR-SMITH WARNS US THAT BITS15-12 OF EBANK MAY BECOME
#	SIGNIFICANT SOMEDAY IN THE FUTURE.  IF/WHEN THAT HAPPENS, THE PROGRAM SHOULD INSURE(BY MASKING ETC.)
#	THAT BITS 15-12 OF EBANK ARE ZERO.
# INITIALIZATION REQUIRED- TO INTERRUPT CURRENT LIST AND START A NEW ONE..
#	1. ADRES OF DOWNLINK LIST INTO DNLSTADR
#	2. NEGONE INTO SUBLIST
#	3. NEGONE INTO DNECADR

		BANK	22
		SETLOC	DOWNTELM
		BANK

		EBANK=	DNTMBUFF

		COUNT	05/DPROG

; ============================================================================
; MAIN TELEMETRY INTERRUPT HANDLER (DODOWNTM)
;
; DODOWNTM is the primary entry point for the downlink telemetry interrupt
; service routine. Executed 50 times per second (every 20 ms) by hardware
; interrupt from the spacecraft telemetry programmer, this routine maintains
; the continuous data stream to Mission Control in Houston.
;
; The handler uses a state machine approach where DNTMGOTO contains the
; address of the next action to perform within the 2-second telemetry cycle.
; This allows complex multi-word data structures to be transmitted across
; multiple interrupt invocations without blocking other AGC operations.
;
; During Apollo 11's mission, this telemetry stream provided Mission Control
; with real-time spacecraft health monitoring. The 200-word per 2-second
; bandwidth (100 double-precision words) delivered sufficient fidelity to
; track navigation state, guidance parameters, fuel status, crew displays,
; and system health without overwhelming ground communication links.
; ============================================================================
;
DODOWNTM	TS	BANKRUPT
		EXTEND
		QXCH	QRUPT		# SAVE Q
		CA	BIT7		# SET WORD ORDER CODE TO 1.  EXCEPTION- AT
		EXTEND			# THE BEGINNING OF EACH LIST THE WORD
		WOR	CHAN13		# CODE WILL BE SET BACK TO 0.
		TC	DNTMGOTO	# GO TO APPROPRIATE PHASE OF PROGRAM

; ============================================================================
; TRANSITION: From telemetry interrupt to list processing
;
; At this point, the telemetry interrupt handler branches to one of two
; main processing phases based on the current state stored in DNTMGOTO.
; DNPHASE1 begins a new 2-second telemetry list cycle, while DNPHASE2
; continues transmitting data from the current list position. This dual-phase
; design enables the AGC to send 200 computer words every 2 seconds without
; requiring continuous processor attention.
; ============================================================================
;
; DNPHASE1: Start of new downlink list cycle
; This phase initializes control structures for transmitting a complete
; downlist. Executed once every 2 seconds at the beginning of each new
; telemetry transmission cycle. Sets up pointers and prepares to fetch
; the first data words from the current downlist specification.
;
DNPHASE1	CA	NEGONE		# INITIALIZE ALL CONTROL WORDS
		TS	SUBLIST		# WORDS TO MINUS ONE
		TS	DNECADR
		CA	LDNPHAS2	# SET DNTMGOTO =0 ALL SUBSEQUENT DOWNRUPTS
		TS	DNTMGOTO	# GO TO DNPHASE2
		TCF	NEWLIST
;
; DNPHASE2: Continue transmitting data from current list position
; This phase handles subsequent interrupts (2-99) within a 2-second cycle.
; Each interrupt (every 20 ms) transmits 2 AGC words from the current
; downlist. DNECADR tracks the current position within the data stream.
; If DNECADR is positive, data transmission continues. If negative or zero,
; processing transitions to handling list control words.
;
DNPHASE2	CCS	DNECADR		# SENDING OF DATA IN PROGRESS
DODNADR		TC	FETCH2WD	# YES - THEN FETCH THE NEXT 2 SP WORDS
MINTIME2	-1DNADR	TIME2		# NEGATIVE OF TIME2 1DNADR
		TCF	+1		# (ECADR OF 3776 + 74001 = 77777)

; Data transmission paused - process list control words.
; The downlist structure consists of a control list containing pointers to
; sublists (data snapshots). When no active data transmission is in progress
; (DNECADR negative/zero), the program examines SUBLIST to determine whether
; currently processing a sublist or the main control list.
;
		CCS	SUBLIST		# IS THE SUBLIST IN CONTROL
# Page 1097
		TCF	NEXTINSL	# YES
DNADRDCR	OCT	74001		# DNADR COUNT AND ECADR DECREMENTER

; CHKLIST: Process next entry from main control list
; The control list contains three types of entries:
; 1. Pointers to sublists (data snapshots)
; 2. Channel output directives
; 3. Direct memory address pointers
; A negative entry signals end of the 2-second downlist cycle.
;
CHKLIST		CA	CTLIST
		EXTEND
		BZMF	NEWLIST		# IT WILL BE NEGATIVE AT END OF LIST
		TCF	NEXTINCL
;
; NEWLIST: Initialize new 2-second telemetry list cycle
; Called when CTLIST reaches the end marker (negative value). Reloads the
; control list pointer (CTLIST) from the downlist table indexed by DNLSTCOD.
; DNLSTCOD selects which downlist is active (determined by mission phase,
; major mode, or ground updates). The selected list then governs the next
; 2 seconds of telemetry transmission to Mission Control.
;
NEWLIST		INDEX	DNLSTCOD
		CA	DNTABLE		# INITIALIZE CTLIST WITH
		TS	CTLIST		#	STARTING ADDRESS OF NEW LIST
		CS	DNLSTCOD
		TCF	SENDID +3
NEXTINCL	INDEX	CTLIST
		CA	0
		CCS	A
		INCR	CTLIST		# SET POINTER TO PICK UP NEXT CTLIST WORD
		TCF	+4		# ON NEXT ENTRY TO PROG.  (A SHOULD NOT =0)
		XCH	CTLIST		# SET CTLIST TO NEGATIVE AND PLACE(CODING)
		COM			# UNCOMPLEMENTED DNADR INTO A.    (FOR LA)
		XCH	CTLIST		#                                 (ST IN )
	+4	INCR	A		#                                 (CTLIST)
 		TS	DNECADR		# SAVE DNADR
		AD	MINTIME2	# TEST FOR TIME2 (NEG. OF ECADR)
		CCS	A
		TCF	SETWO +1	# DON'T SET WORD ORDER CODE
MINB1314	OCT	47777		# MINUS BIT 13 AND 14 (CAN'T GET HERE)
		TCF	SETWO +1	# DON'T SET WORD ORDER CODE
SETWO		TC	WOZERO		# GO SET WORD ORDER CODE TO ZERO.
 	+1	CA	DNECADR		# RELOAD A WITH THE DNADR.
 	+2	AD	MINB1314	# IS THIS A REGULAR DNADR?
 		EXTEND
		BZMF	FETCH2WD	# YES.  (A MUST NEVER BE ZERO)
		AD	MINB12		# NO-	IS IT A POINTER (DNPTR) OR A
		EXTEND			# 	CHANNEL(DNCHAN)
		BZMF	DODNPTR		# IT'S A POINTER.  (A MUST NEVER BE ZERO)

; ============================================================================
; DODNCHAN: I/O Channel Reading Handler
;
; Retrieves data directly from AGC I/O channels using indexed READ instructions.
; This enables telemetry of hardware sensor data, switch positions, and system
; status bits without requiring memory buffering. Critical for transmitting
; real-time spacecraft state (RCS jet status, IMU CDU angles, radar data, etc.)
; to Mission Control.
;
; The code uses a clever self-modifying technique where instructions at fixed
; addresses (6) are executed as EXTEND/READ pairs through indexing. This saves
; memory while enabling dynamic channel selection.
; ============================================================================
;
DODNCHAN	TC	6		# (EXECUTED AS EXTEND)  IT S A CHANNEL
		INDEX	DNECADR
		INDEX	0 -4000		# (EXECUTED AS READ)
		TS	L
		TC	6		# (EXECUTED AS EXTEND)
		INDEX	DNECADR
		INDEX	0 -4001		# (EXECUTED AS READ)
		TS	DNECADR		# SET DNECADR
		CA	NEGONE		#	TO MINUS
		XCH	DNECADR		#		WHILE PRESERVING A.
		TCF	DNTMEXIT	# GO SEND CHANNELS

; Set word order code to zero in channel 13 for downlink packet framing.
; The word order bit controls whether the AGC transmits high word first or
; low word first in double-precision transmissions. Consistent word ordering
; ensures ground station decommutation equipment correctly reassembles AGC data.
;
WOZERO		CS	BIT7
		EXTEND
# Page 1098
		WAND	CHAN13		# SET WORD ORDER CODE TO ZERO
		TC	Q		# RETURN TO CALLER

; ============================================================================
; DODNPTR: Sublist Pointer Dispatcher
;
; Examines downlist entry to determine data structure type:
; - Positive value: Regular sublist pointer (redirect to another list structure)
; - Zero/Negative value: Snapshot sublist (atomic multi-word capture)
;
; Regular sublists enable data structure reuse across multiple downlist
; configurations, saving memory. Snapshot sublists trigger optimized atomic
; capture of navigation registers - essential during critical phases like
; powered descent when consistent multi-word state vectors (position, velocity,
; attitude) must be captured without interrupt-driven updates between words.
; ============================================================================
;
DODNPTR		INDEX	DNECADR		# DNECADR CONTAINS ADRES OF SUBLIST
		0	0		# CLEAR AND ADD LIST ENTRY INTO A.
		CCS	A		# IS THIS A SNAPSHOT SUBLIST
		CA	DNECADR		# NO, IT IS A REGULAR SUBLIST.
		TCF	DOSUBLST	# A MUST NOT BE ZERO.

		XCH	DNECADR		# YES, IT IS A SNAPSHOT SUBLIST.
		TS	SUBLIST		# C(DNECADR) INTO SUBLIST
		CAF	ZERO		#	A    INTO     A
		XCH	TMINDEX		# (NOTE.. TMINDEX = DNECADR)

# THE FOLLOWING CODING (FROM SNAPLOOP TO SNAPEND)IS FOR THE PURPOSE OF TAKING A SNAPSHOT OF 12 DP REGISTERS.
# THIS IS DONE BY SAVING 11 DP REGISTERS IN DNTMBUFF AND SENDING THE FIRST DP WORD IMMEDIATELY.
# THE SNAPSHOT PROCESSING IS THE MOST TIME CONSUMING AND THEREFORE THE CODING AND LIST STRUCTURE WERE DESIGNED
# TO MINIMIZE TIME.  THE TIME OPTIMIZATION RESULTS IN RULES UNIQUE TO THE SNAPSHOT PORTION OF THE DOWNLIST.
# THESE RULES ARE......
#	1.	ONLY 1DNADR'S CAN APPEAR IN THE SNAPSHOT SUBLIST
#	2.	THE 1DNADR'S CANNOT REFER TO THE FIRST LOCATION IN ANY BANK.

; ============================================================================
; SNAPLOOP: Atomic Multi-Word Snapshot Capture
;
; High-speed capture of 12 double-precision registers (24 AGC words total)
; into DNTMBUFF buffer memory. This atomic snapshot mechanism is essential
; for navigation state vectors where consistency across multiple words is
; critical - position, velocity, and attitude data must represent a single
; instant in time.
;
; During powered descent landing, navigation updates occur continuously via
; interrupts. A snapshot ensures the transmitted state vector represents a
; coherent spacecraft state rather than a mixture of old and new data that
; could span multiple navigation cycles. Mission Control relies on this
; consistency for trajectory monitoring and abort decision-making.
;
; Time-optimized coding: Processes 11 double-precision words in rapid
; succession, buffering them while immediately transmitting the first word.
; Subsequent DOWNRUPT interrupts (every 20ms) drain the buffer.
; ============================================================================
;
SNAPLOOP	TS	EBANK		# SET EBANK
		MASK	LOW8		# ISOLATE RELATIVE ADDRESS
		EXTEND
		INDEX	A
		EBANK=	1401
		DCA	1401		# PICK UP 2 SNAPSHOT WORDS.
		EBANK=	DNTMBUFF
		INDEX	TMINDEX
		DXCH	DNTMBUFF	# STORE 2 SNAPSHOT WORDS IN BUFFER
		INCR	TMINDEX		# SET BUFFER INDEX FOR NEXT 2 WORDS.
		INCR	TMINDEX
SNAPAGN		INCR	SUBLIST		# SET POINTER TO NEXT 2 WORDS OF SNAPSHOT
		INDEX	SUBLIST
		0	0		# = CA SSSS (SSSS = NEXT ENTRY IN SUBLIST)
		CCS	A		# TEST FOR LAST TWO WORDS OF SNAPSHOT.
		TCF	SNAPLOOP	# NOT LAST TWO.
LDNPHAS2	GENADR	DNPHASE2
		TS	SUBLIST		# YES, LAST.  SAVE A.
		CA	NEGONE		# SET DNECADR AND
		TS	DNECADR		#	SUBLIST POINTERS
		XCH	SUBLIST		#		TO NEGATIVE VALUES.
		TS	EBANK
		MASK	LOW8
		EXTEND
		INDEX	A
		EBANK=	1401
# Page 1099
		DCA	1401		# PICK UP FIRST 2 WORDS OF SNAPSHOT.
		EBANK=	DNTMBUFF
SNAPEND		TCF	DNTMEXIT	# 	NOW GO SEND THEM.

; ============================================================================
; FETCH2WD: Standard Double-Precision Word Retrieval
;
; Fetches two consecutive AGC words (double-precision data) from erasable
; memory using 1DNADR encoding. This is the standard data fetch path for
; most telemetry items - navigation state vectors, guidance parameters,
; attitude quaternions, time values, etc.
;
; The routine handles EBANK switching to access any location within the
; AGC's 2K erasable memory space. The DNADRDCR constant decrements both
; the word count and the encoded address, efficiently advancing through
; consecutive double-precision values in the downlist structure.
;
; Mission Control receives these double-precision values for real-time
; monitoring of spacecraft state during all mission phases. During critical
; events (TLI, LOI, powered descent, ascent), continuous telemetry enables
; ground controllers to verify guidance performance and make go/no-go decisions.
; ============================================================================
;
FETCH2WD	CA	DNECADR
		TS	EBANK		# SET EBANK
		MASK	LOW8		# ISOLATE RELATIVE ADDRESS
		TS	L
		CA	DNADRDCR	# DECREMENT COUNT AND ECADR
		ADS	DNECADR
		EXTEND
		INDEX	L
		EBANK=	1400
		DCA	1400		# PICK UP 2 DATA WORDS
		EBANK=	DNTMBUFF
		TCF	DNTMEXIT	# 	NOW GO SEND THEM.

; ============================================================================
; DOSUBLST: Regular Sublist Processing
;
; Processes regular (non-snapshot) sublists by sequentially extracting data
; addresses from the sublist structure. Regular sublists enable efficient
; data structure reuse - common telemetry items (IMU angles, time, spacecraft
; mode flags) can be defined once and referenced from multiple downlist
; configurations, saving program memory.
;
; Sublist structure: A sequence of 1DNADR entries terminated by a zero word.
; Each 1DNADR encodes both EBANK and relative address for a double-precision
; memory location. The zero terminator signals sublist completion, causing
; the program to return to the parent control list.
;
; This hierarchical list architecture was a key innovation that enabled
; Comanche055 to save approximately 150 words of core storage compared to
; earlier downlink implementations - critical savings given the AGC's 36K
; fixed memory constraint.
; ============================================================================
;
DOSUBLST	TS	SUBLIST		# SET SUBLIST POINTER
NEXTINSL	INDEX	SUBLIST
		0	0		# = CA SSSS (SSSS = NEXT ENTRY IN SUBLIST)
		CCS	A		# IS IT THE END OF THE SUBLIST
		INCR	SUBLIST		# NO-
		TCF	+4
		TS	SUBLIST		# SAVE A.
		CA	NEGONE		# SET SUBLIST TO MINUS
		XCH	SUBLIST		# RETRIEVE A.
	+4	INCR	A
 		TS	DNECADR		# SAVE DNADR
		TCF	SETWO +2	# GO USE COMMON CODING(PROBLEMS WOULD
					# OCCUR IF THE PROGRAM ENCOUNTERED A
					# DNPTR NOW)

; ============================================================================
; DNTMEXIT: Downlink Transmission Output
;
; Final exit path that writes the prepared double-precision telemetry word
; (A and L registers) to output channels 34 and 35. These channels interface
; with the spacecraft's downlink telemetry converter hardware, which serializes
; the parallel AGC data into the telemetry bit stream transmitted to Earth.
;
; The MSFN (Manned Space Flight Network) ground stations receive this data
; at 51.2 kilobits per second during Apollo 11. The data passes through
; tracking stations at Goldstone (California), Madrid (Spain), and Honeysuckle
; Creek (Australia), providing continuous communication coverage as Earth
; rotates beneath the spacecraft's ground track.
;
; Mission Control's Real-Time Computer Complex (RTCC) in Houston decommutates
; this telemetry stream, extracting AGC data for display on flight controller
; consoles. During critical mission phases, dozens of controllers simultaneously
; monitor these values to verify spacecraft health and mission progress.
; ============================================================================
;
DNTMEXIT	EXTEND			# DOWN-TELEMETRY EXIT
		WRITE	DNTM1		# TO SEND A + L TO CHANNELS 34 + 35
		CA	L		# RESPECTIVELY
TMEXITL		EXTEND
		WRITE	DNTM2
TMRESUME	TCF	RESUME		# EXIT TELEMETRY PROGRAM VIA RESUME.

MINB12		EQUALS	-1/8
DNECADR		EQUALS	TMINDEX
CTLIST		EQUALS	LDATALST
SUBLIST		EQUALS 	DNQ

# Page 1100
# SUBROUTINE NAME- DNDUMP
# FUNCTIONAL DESCRIPTION - TO SEND(DUMP) ALL ERASABLE STORAGE 'N' TIMES.(N = 1 TO 4).  BANKS ARE SENT ONE AT A TIME
#	EACH BANK IS PRECEDED BY AN ID WORD, SYNCH BITS, ECADR AND TIME1 FOLLOWED BY THE 256D WORDS OF EACH
#	EBANK.  EBANKS ARE DUMPED IN ORDER(I.E. EBANK 0 FIRST, THEN EBANK1 ETC.)
# CALLING SEQUENCE- THE GROUND OR ASTRONAUT BY KEYING V74E CAN INITIALIZE THE DUMP.
#	AFTER KEYING IN V74E THE CURRENT DOWNLIST WILL BE IMMEDIATELY TERMINATED AND THE DOWNLINK ERASABLE DUMP
#	WILL BEGIN.
#	ONCE INITIATED THE DOWNLINK ERASABLE DUMP CAN BE TERMINATED (AND INTERRUPTED DOWNLIST REINSTATED) ONLY
#	BY THE FOLLOWING:
#	1. A FRESH START
#	2. COMPLETION OF ALL DOWNLINK DUMPS REQUESTED (ACCORDING TO BITS SET IN DUMPCNT).  NOTE THAT DUMPCNT
#	   CAN BE ALTERED BY A V21N01.
#	3. AND INVOLUNTARILY BY A RESTART.
# NORMAL EXIT MODE- TCF DNPHASE1
# ALARM OR ABORT MODE- NONE
# *SUBROUTINES CALLED- NONE.
# ERASABLE INITIALIZATION REQUIRED --
#	DUMPCNT	OCT 20000	IF 4 COMPLETE ERASABLE DUMPS ARE DESIRED
#	DUMPCNT OCT 10000	IF 2 COMPLETE ERASABLE DUMPS ARE DESIRED
#	DUMPCNT	OCT 04000	IF 1 COMPLETE ERASABLE DUMP  IS  DESIRED
# DEBRIS- DUMPLOC, DUMPSW, DNTMGOTO, EBANK AND CENTRAL REGISTERS
# TIMING-	TIME (IN SECS) = ((NO.DUMPS)*(NO.EBANKS)* (WDSPEREBANK + NO.IDWDS)) / NO.WDSPERSEC
#		TIME (IN SECS) =  (   4    )*(    8    )* (    256     +     4   )  /     100
#	   THUS TIME (IN SECS TO SEND DUMP OF ERASABLE 4 TIMES VIA DOWNLINK) = 83.2 SECONDS
#
# STRUCTURE OF ONE EBANK AS IT IS SENT BY DOWNLINK PROGRAM-
#	(REMINDER-THIS ONLY DESCRIBES ONE OF THE 8 EBANKS X 4 (DUMPS) = 32 EBANKS WHICH WILL BE SENT BY DNDUMP)
#	 DOWNLIST				W
#	  WORD	TAKEN FROM CONTENTS OF	EXAMPLE	O	COMMENTS
#
#	    1	ERASID			 0177X	0	DOWNLIST I.D. FOR DOWNLINK ERASABLE DUMP (X=7 CSM, 6 LM)
#	    2	LOWIDCOD		 77340 	1	DOWNLINK SYNCH BITS.(SAME ONE USED IN ALL OTHER DOWNLISTS)
#	    3	DUMPLOC			 13400	1	(SEE NOTES ON DUMPLOC)1= 3RD ERAS DUMP, 3400=ECADR OF 5TH WD
#	    4	TIME1			 14120	1	TIME IN CENTISECONDS
#	    5	FIRST WORD OF EBANK X	 03400	1	IN THIS EXAMPLE THIS WORD = CONTENTS OF E7,1400 (ECADR 3400)
#	    6	2ND   WORD OF EBANK X	 00142	1	IN THIS EXAMPLE THIS WORD = CONTENTS OF E7,1401 (ECADR 3401)
#	    7.  3RD   WORD OF EBANK X	 00142	1	IN THIS EXAMPLE THIS WORD = CONTENTS OF E7,1402 (ECADR 3402)
#	    .					1
#	    .					1
#	    .					1
#	 260D	256TH WORD OF EBANK X	 03777	1	IN THIS EXAMPLE THIS WORD = CONTENTS OF E7,1777 (ECADR 3777)
#
# NOTE-	DUMPLOC CONTAINS THE COUNTER AND ECADR FOR EACH WORD BEING SENT.
#	THE BIT STRUCTURE OF DUMPLOC IS FOLLOWS--
#						X = NOT USED
#		X ABC EEE RRRRRRRR	      ABC = ERASABLE DUMP COUNTER(I.E. ABC = 0,1,2, OR 3 WHICH MEANS THAT
#						    COMPLETE ERASABLE DUMP NUMBER 1,2,3, OR 4 RESPECTIVELY IS IN PROGRESS)
#					      EEE = EBANK BITS
#					 RRRRRRRR = RELATIVE ADDRESS WITHIN AN EBANK.

# Page 1101
; ============================================================================
; DNDUMP: Complete Erasable Memory Dump to Ground
;
; Emergency diagnostic system enabling ground controllers to retrieve the
; complete contents of AGC erasable memory (all 2K words across 8 EBANKs).
; This capability is critical during anomalies or unexpected spacecraft
; behavior, allowing Mission Control flight dynamics and guidance officers
; to analyze the AGC's internal state for troubleshooting.
;
; Dump Structure: Each EBANK dump consists of:
;   - ID word (ERASID 0177X where X=7 for CSM, X=6 for LM)
;   - Synchronization code (LOWIDCOD 77340) for ground decommutation
;   - DUMPLOC word encoding dump counter + EBANK + relative address
;   - TIME1 word providing temporal correlation
;   - 256 consecutive words from the EBANK
;
; The dump can be repeated 1-4 times (controlled by DUMPCNT) to capture
; dynamic changes in erasable variables. Four complete dumps of all 8 EBANKs
; takes 83.2 seconds to transmit at 100 words/second downlink rate.
;
; Initiated by: V74E verb or ground command. Immediately preempts current
; downlist, which resumes after dump completion or fresh start/restart.
;
; During Apollo 11, this dump capability provided critical backup diagnostics,
; though the nominal mission did not require its use. The capability gave
; ground controllers confidence they could diagnose unexpected AGC behavior.
; ============================================================================
;
DNDUMPI		CA	ZERO		# INITIALIZE DOWNLINK
		TS	DUMPLOC		# ERASABLE DUMP
	+2	TC	SENDID		# GO SEND ID AND SYNCH BITS
		CA	LDNDUMP1	# SET DNTMGOTO
		TS	DNTMGOTO	# TO LOCATION FOR NEXT PASS
		CA	TIME1		# PLACE TIME1
		XCH	L		# INTO L
		CA	DUMPLOC		# AND ECADR OF THIS EBANK INTO A
		TCF	DNTMEXIT	# SEND DUMPLOC AND TIME1

LDNDUMP		ADRES	DNDUMP
LDNDUMP1	ADRES	DNDUMP1

; ============================================================================
; DNDUMP Main Loop: Sequential Memory Dump Progression
;
; Advances through erasable memory by incrementing DUMPLOC (which encodes
; dump counter, EBANK number, and relative address) by 2 each pass, moving
; through consecutive double-precision words. The routine handles two critical
; transitions:
;
; 1. EBANK boundary detection: When relative address reaches 1777 (octal) and
;    wraps to 0000, the routine recognizes completion of the 256-word EBANK
;    and checks if all requested dumps are complete.
;
; 2. Dump completion test: Examines DUMPCNT to determine if all requested
;    repetitions (1-4 complete erasable dumps) have finished. If complete,
;    restores the interrupted downlist via DNPHASE1. If incomplete, initializes
;    the next EBANK dump header.
;
; This state machine architecture enables the 20ms interrupt-driven dump to
; span many seconds while maintaining responsiveness to other AGC functions.
; The dump progresses in small increments (2 words per 20ms interrupt), never
; monopolizing processor time.
; ============================================================================
;
DNDUMP		CA	TWO		# INCREMENT ECADR IN DUMPLOC
		ADS	DUMPLOC		# TO NEXT DP WORD TO BE
		MASK	LOW8		# DUMPED AND SAVE IT.
		CCS	A		# IS THIS THE BEGINNING OF A NEW EBANK
		TCF	DNDUMP2		# NO- THEN CONTINUE DUMPING
		CA	DUMPLOC		# YES- IS THIS THE END OF THE
		MASK	DUMPCNT		# N TH(N = 1 TO 4) COMPLETE ERASABLE
		MASK	PRIO34		# DUMP(BIT14 FOR 4, BIT13 FOR 2 OR BIT12
		CCS	A		# FOR 1 COMPLETE ERASABLE DUMP(S)).
		TCF	DNPHASE1	# YES- START SENDING INTERRUPTED DOWNLIST
					# AGAIN
		TCF	DNDUMPI +2	# NO- GO BACK AND INITIALIZE NEXT BANK

; ============================================================================
; DNDUMP1 & DNDUMP2: EBANK Data Extraction
;
; After header transmission (ID codes and DUMPLOC), these routines extract
; the actual memory contents from each EBANK for downlink transmission.
;
; DNDUMP1: Sets DNTMGOTO to LDNDUMP, establishing the return point for
; subsequent word-pairs in the current EBANK (words 3-256 of the 260-word
; dump packet structure). This enables the interrupt-driven dump to resume
; at DNDUMP each 20ms, progressively stepping through memory.
;
; DNDUMP2: Performs the actual memory read using sophisticated addressing:
;   1. Loads DUMPLOC into EBANK register to select the target memory bank
;   2. Masks to extract relative address within EBANK (bits 7-0)
;   3. Uses indexed MASK operations (not CA) to read memory without disturbing
;      editing registers 20-23, which may hold active DSKY display data
;   4. Reads double-precision pair at indices 1400/1401 relative to address
;
; The MASK technique is critical: during DSKY operations, registers 20-23
; contain seven-segment display codes. Reading via CA would trigger DSKY
; hardware, potentially corrupting displays. MASK reads without side effects.
; ============================================================================
;
DNDUMP1		CA	LDNDUMP		# SET DNTMGOTO
		TS	DNTMGOTO	# FOR WORDS 3 TO 256D OF CURRENT EBANK

DNDUMP2		CA	DUMPLOC
		TS	EBANK		# SET EBANK
		MASK	LOW8		# ISOLATE RELATIVE ADDRESS.
		TS	Q		# (NOTE: MASK INSTRUCTION IS USED TO PICK
		CA	NEG0		# UP ERASABLE REGISTERS SO THAT EDITING
		TS	L		# REGISTERS 20-23 WILL NOT BE ALTERED.)
		INDEX	Q
		EBANK=	1400		# PICK UP LOW ORDER REGISTER OF PAIR
		MASK	1401		# OF ERASABLE REGISTERS.
		XCH	L
		INDEX	Q		# PICK UP HIGH ORDER REGISTER OF PAIR
		MASK	1400		# OF ERASABLE REGISTERS.
		EBANK=	DNTMBUFF
		TCF	DNTMEXIT	# GO SEND THEM

; ============================================================================
; SENDID: Downlist Identification Code Transmission
;
; Dual-purpose routine providing synchronization codes for both regular
; downlists and erasable memory dumps. Ground decommutation software uses
; these ID codes to recognize packet boundaries and correctly parse the
; incoming telemetry stream.
;
; Two Entry Points:
;
; 1. SENDID (top): Used by erasable dump initialization (DNDUMPI). The QXCH
;    instruction saves the return address in DNTMGOTO, causing subsequent
;    interrupts to resume at the instruction following the "TC SENDID" call.
;    Loads ERASID (octal 01776 for CSM, 01777 for LM) into A register.
;
; 2. SENDID+3 (TS L): Used by regular downlist processing (NEWLIST). Assumes
;    A already contains the appropriate list ID code from the downlist header.
;
; Output Format:
;   A register: ID code (ERASID for dumps, list ID for regular downlists)
;   L register: LOWIDCOD (octal 077340) synchronization pattern
;
; LOWIDCOD provides a distinctive bit pattern enabling ground software to
; achieve bit/word synchronization even if data stream alignment is initially
; unknown. The pattern is chosen to have low probability of accidental
; occurrence in normal telemetry data.
;
; During Apollo 11, these ID codes enabled Mission Control's RTCC to correctly
; distinguish between the various downlist formats (CM/LM, different mission
; phases) and the emergency dump format, ensuring proper data interpretation.
; ============================================================================
;
SENDID		EXTEND			# **ENTRANCE USED BY ERASABLE DUMP PROG.**
		QXCH	DNTMGOTO	# SET DNTMGOTO SO NEXT TIME PROG WILL GO
		CAF	ERASID		# TO LOCATION FOLLOWING :TC SENDID:

		TS	L		# **ENTRANCE USED BY REGULAR DOWNLINK PG**
# Page 1102
		TC	WOZERO		# GO SET WORD ORDER CODE TO ZERO
		CAF	LOWIDCOD	# PLACE SPECIAL ID CODE INTO L
		XCH	L		# AND ID BACK INTO A
		TCF	DNTMEXIT	# SEND DOWNLIST ID CODE(S).
