# Copyright:	Public domain.
# Filename:	DOWN_TELEMETRY_PROGRAM.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	988-997
# Mod history:	2009-05-24 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
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
; FILE: DOWN_TELEMETRY_PROGRAM.agc
; MODULE: Core Operating System - Telemetry
; MISSION PHASE: All phases (launch/earth-orbit/trans-lunar/lunar-orbit/
;                descent/landing/ascent/rendezvous/trans-earth)
;
; TL;DR: Manages downlink telemetry transmission from the Lunar Module AGC
;        to Mission Control via the Manned Space Flight Network (MSFN).
;        Executes 50 times per second (every 20 milliseconds) triggered by
;        the spacecraft telemetry programmer. Packages AGC data including
;        navigation state, guidance parameters, crew displays, and system
;        status into standardized telemetry frames for ground monitoring.
;        Includes erasable memory dump capability for comprehensive system
;        diagnostics (V74E command).
;
; COMMENT-ONLY READERS: This system was Mission Control's eyes into the
;        Lunar Module computer during Apollo 11's journey. Every 20 
;        milliseconds, this code selected critical data and transmitted it
;        to Earth, enabling flight controllers to monitor the spacecraft's
;        health and navigation state throughout the mission.
;
; CODE-ALONG READERS: Study the interrupt-driven architecture (DOWNRUPT at
;        50 Hz), the control list/sublist data structure for flexible
;        telemetry frame composition, and the snapshot buffering mechanism
;        for time-critical data collection. Note the specialized address
;        types (1DNADR through 6DNADR, DNPTR, DNCHAN) that enable efficient
;        multi-word data packaging.
; ============================================================================

# Page 988
; ============================================================================
; MISSION CONTROL COMMUNICATION SYSTEM
;
; During Apollo 11's lunar landing mission, this telemetry program
; transmitted spacecraft data to Earth 50 times every second. Flight
; controllers in Houston monitored altitude, velocity, fuel levels, and
; system status through this continuous data stream. When Neil Armstrong
; and Buzz Aldrin descended toward the Sea of Tranquility, controllers
; watched telemetry from this program to verify guidance computer health
; and mission trajectory accuracy.
; ============================================================================

# PROGRAM NAME -- DOWN TELEMETRY PROGRAM
# MOD NO. -- 0		TO COMPLETELY REWRITE THE DOWN TELEMETRY PROGRAM AND DOWNLINK ERASABLE DUMP PROGRAM FOR THE
#			PURPOSE OF SAVING APPROXIMATELY 150 WORDS OF CORE STORAGE.
#			THIS CHANGE REQUIRES AN ENTIRELY NEW METHOD OF SPECIFYING DOWNLINK LISTS.  REFER TO DOWNLINK
#			LISTS LOG SECTION FOR MORE DETAILS.  HOWEVER THIS CHANGE WILL NOT AFFECT THE GROUND PROCESSING
#			OF DOWN TELEMETRY DATA.
# MOD BY -- KILROY, SMITH, DEWITT
# DATE -- 02 OCT 67
# AUTHORS -- KILROY, SMITH, DWWITT, DEWOLF, FAGIN
# LOG SECTION -- DOWN-TELEMETRY PROGRAM
#
# FUNCTIONAL DESCRIPTION -- THIS ROUTINE IS INITIATED BY TELEMETRY END
#	PULSE FROM THE DOWNLINK TELEMETRY CONVERTER.  THIS PULSE OCCURS
#	AT 50 TIMES PER SEC (EVERY 20 MS) THEREFORE DODOWNTM IS
#	EXECUTED AT THESE RATES.  THIS ROUTINE SELECTS THE APPROPRIATE
#	AGC DATA TO BE TRANSMITTED DOWNLINK AND LOADS IT INTO OUTPUT
#	CHANNELS 34 AND 35.  THE INFORMATION IS THEN GATED OUT FROM THE
#	LGC IN SERIAL FASHION.
#
#	THIS PROGRAM IS CODED FOR A 2 SECOND DOWNLIST.  SINCE DOWNRUPTS
#	OCCUR EVERY 20 MS AND 2 AGC COMPUTER WORDS CAN BE PLACED IN
#	CHANNELS 34 AND 35 DURING EACH DOWNRUPT THE PROGRAM IS CAPABLE
# 	OF SENDING 200 AGC WORDS EVERY 2 SECONDS.
#
# CALLING SEQUENCE -- NONE
#	PROGRAM IS ENTERED VIA TCF DODOWNTM WHICH IS EXECUTED AS A
#	RESULT OF A DOWNRUPT.  CONTROL IS RETURNED VIA TCF RESUME WHICH
#	IN EFFECT IS A RESUME.
#
# SUBROUTINES CALLED -- NONE
#
# NORMAL EXIT MODE -- TCF RESUME
#
# ALARM OR ABORT EXIT MODE -- NONE
#
# RESTART PROTECTION:
#	ON A FRESH START AND RESTART THE `STARTSUB' SUBROUTINE WILL INITIALIZE THE DOWNLIST POINTER (ACTUALLY
#	DNTMGOTO) TO THE BEGINNING OF THE CURRENT DOWNLIST (I.E., CURRENT CONTENTS OF DNLSTADR).  THIS HAS THE
#	EFFECT OF IGNORING THE REMAINDER OF THE DOWNLIST WHICH THE DOWN-TELEMETRY PROGRAM WAS WORKING ON WHEN
#	THE RESTART (OR FRESH START) OCCURRED AND RESUME DOWN TELEMETRY FROM THE BEGINNING OF THE CURRENT
#	DOWNLIST.
#
#	ALSO OF INTEREST IS THE FACT THAT ON A RESTART THE AGC WILL ZERO DOWNLINK CHANNELS 13, 34 AND 35.
#
# DOWNLINK LIST SELECTION:
#	THE APPROPRIATE DOWNLINK LISTS ARE SELECTED BY THE FOLLOWING:
#	1.	FRESH START
#	2.	V37EXXE WHERE XX = THE MAJOR MODE BEING SELECTED.
#	3.	UPDATE PROGRAM (P27)
#	4.	NON-V37 SELECTABLE TYPE PROGRAMS (E.G., AGS INITIALIZATION (SUNDANCE, LUMINARY) AND P61-P62
#		TRANSITION (COLOSSUS) ETC.).
#
# DOWNLINK LIST RULES AND LIMITATIONS:
#	READ SECTION(S) WHICH FOLLOW `DEBRIS' WRITEUP.
#
# OUTPUT -- EVERY 2 SECONDS 100 DOUBLE PRECISION WORDS (I.E., 200 LGC
#	COMPUTER WORDS) ARE TRANSMITTED VIA DOWNLINK.
#
# ERASABLE INITIALIZATION REQUIRED -- NONE
#	`DNTMGOTO' AND `DNLSTADR' ARE INITIALIZED BY THE FRESH START PROGRAM.
#
# DEBRIS (ERASABLE LOCATIONS DESTROYED BY THIS PROGRAM) --
#	LDATALST, DNTMBUFF TO DNTMBUFF +21D, TMINDEX, DNQ.

; ============================================================================
; TELEMETRY SYSTEM ARCHITECTURE
;
; The downlink telemetry system operates on an interrupt-driven schedule.
; Every 20 milliseconds, the spacecraft's telemetry programmer sends an
; "end pulse" to the AGC, triggering a DOWNRUPT interrupt. This interrupt
; invokes the DODOWNTM routine which packages two AGC words into output
; channels 34 and 35. The data is then serially transmitted to Earth via
; the S-band antenna.
;
; TRANSMISSION CAPACITY: With 50 interrupts per second and 2 words per
; interrupt, the system transmits 100 words/second or 200 words every 2
; seconds. This 2-second cycle is called a "downlist frame."
;
; DATA ORGANIZATION: Telemetry data is organized in "downlists" - structured
; lists defining which AGC memory locations to transmit. Different mission
; phases use different downlists (orbital maneuvers, landing, rendezvous).
; The current downlist address is stored in DNLSTADR and can be changed by:
;   - Fresh start initialization
;   - Major mode changes (V37E command)
;   - Update program (P27)
;   - Special programs (AGS initialization, landing programs)
;
; RESTART PROTECTION: If the AGC restarts during telemetry transmission,
; the system abandons the partially-sent downlist and restarts from the
; beginning. This ensures Mission Control receives complete, coherent data
; frames rather than corrupted partial frames.
; ============================================================================
# Page 989 (empty page)
# Page 990

; ============================================================================
; TRANSITION: From system overview to data structure specification
;
; Now that we understand how the telemetry system operates at 50 Hz, we
; must examine the data structures that control what information is sent.
; The "downlist" format uses special address codes to efficiently package
; different data types. This design allowed the relatively slow AGC
; (instructions execute in ~12 microseconds) to quickly gather and transmit
; diverse data types during the brief 20-millisecond window between
; telemetry interrupts.
; ============================================================================

# DODOWNTM IS ENTERED EVERY 20 MS BY AN INTERRUPT TRIGGERED BY THE
# RECEIPT OF AN ENDPULSE FROM THE SPACECRAFT TELEMETRY PROGRAMMER.
#
# NOTES REGARDING DOWNLINK LISTS ASSOCIATED WITH THIS PROGRAM:
# 1.	DOWNLISTS.  DOWNLISTS MUST BE COMPILED IN THE SAME BANK AS THE
#	DOWN-TELEMETRY PROGRAM.  THIS IS DONE FOR EASE OF CODING, FASTER
#	EXECUTION.
# 2.	EACH DOWNLINK LIST CONSISTS OF A CONTROL LIST AND A NUMBER OF
#	SUBLISTS.
# 3.	A SUBLIST REFERS TO A SNAPSHOT OR DATA COMMON TO THE SAME OR OTHER
#	DOWNLINK LISTS.  ANY SUBLIST CONTAINING COMMON DATA NEEDS TO BE
#	CODED ONLY ONCE FOR THE APPLICABLE DOWNLINK LISTS.
# 4.	SNAPSHOT SUBLISTS REFER SPECIFICALLY TO HOMOGENEOUS DATA WHICH MUST BE
#	SAVED IN A BUFFER DURING ONE DOWNRUPT.
# 5.	THE 1DNADR FOR THE 1ST WORD OF SNAPSHOT DATA IS FOUND AT THE END
#	OF EACH SNAPSHOT SUBLIST, SINCE THE PROGRAM CODING SENDS THIS DP WORD
#	IMMEDIATELY AFTER STORING THE OTHERS IN THE SNAPSHOT BUFFER.
# 6.	ALL LISTS ARE COMBINATIONS OF CODED ERASABLE ADDRESS CONSTANTS
#	CREATED FOR THE DOWNLIST PROGRAM.
#	A.	1DNADR			1-WORD DOWNLIST ADDRESS.
#		SAME AS ECADR, BUT USED WHEN THE WORD ADDRESSED IS THE LEFT
#		HALF OF A DOUBLE-PRECISION WORD FOR DOWN TELEMETRY.
#	B.	2DNADR - 6DNADR		N-WORD DOWNLIST ADDRESS, N = 2 - 6.
#		SAME AS 1DNADR, BUT WITH THE 4 UNUSED BITS OF THE ECADR FORMAT
#		FILLED IN WITH 0001-0101.  USED TO POINT TO A LIST OF N DOUBLE-
#		PRECISION WORDS, STORED CONSECUTIVELY, FOR DOWN TELEMETRY.
#	C.	DNCHAN			DOWNLIST CHANNEL ADDRESS.
#		SAME AS 1DNADR, BUT WITH PREFIX BITS 0111.  USED TO POINT TO
#		A PAIR OF CHANNELS FOR DOWN TELEMETRY.
#	D.	DNPTR			DOWN-TELEMETRY SUBLIST POINTER.
#		SAME AS CAF BUT TAGGED AS A CONSTANT.  USED IN CONTROL LIST TO POINT TO A SUBLIST.
#		CAUTION --- A DNPTR CANNOT BE USED IN A SUBLIST.
# 7.	THE WORD ORDER CODE IS SET TO ZERO AT THE BEGINNING OF EACH DOWNLIST (I.E., CONTROL LIST) AND WHEN
#	A `1DNADR TIME2' IS DETECTED IN THE CONTROL LIST (ONLY).
# 8.	IN THE SNAPSHOT SUBLIST ONLY, THE DNADR'S CANNOT POINT TO THE FIRST WORD OF ANY EBANK.
#
# DOWNLIST LIST RESTRICTIONS:
# (THE FOLLOWING POINTS MAY BE LISTED ELSEWHERE BUT ARE LISTED HERE SO IT IS CLEAR THAT THESE THINGS CANNOT BE
# DONE)
# 1.	SNAPSHOT DOWNLIST:
#	(A) CANNOT CONTAIN THE FOLLOWING ECADRS (I.E., 1DNADR'S): Q, 400, 1000, 1400, 2000, 2400, 3000, 3400.
#	(B) CAN CONTAIN ONLY 1DNADR'S
# 2.	ALL DOWNLINKED DATA (EXCEPT CHANNELS) IS PICKED UP BY A DCA SO DOWNLINK LISTS CANNOT CONTAIN THE
#	EQUIVALENT OF THE FOLLOWING ECADRS (I.E., 1DNADRS): 377, 777, 1377, 1777, 2377, 2777, 3377, 3777.
# 	(NOTE: THE TERM `EQUIVALENT' MEANT THAT THE 1DNADR TO 6DNADR WILL BE PROCESSED LIKE 1 TO 6 ECADRS)
# 3.	CONTROL LISTS AND SUBLISTS CANNOT HAVE ENTRIES = OCTAL 00000 OR OCTAL 77777
# Page 991
# 4.	THE `1DNADR TIME2' WHICH WILL CAUSE THE DOWNLINK PROGRAM TO SET THE WORDER CODE TO 3 MUST APPEAR IN THE
#	CONTROL SECTION OF THE DOWNLIST.
# 5.	`DNCHAN 0' CANNOT BE USED.
# 6.	`DNPTR 0' CANNOT BE USED.
# 7.	DNPTR CANNOT APPEAR IN A SUBLIST.
#
# EBANK SETTINGS
#	IN THE PROCESS OF SETTING THE EBANK (WHEN PICKING UP DOWNLINK DATA) THE DOWN TELEMETRY PROGRAM PUTS
#	`GARBAGE' INTO BITS15-12 OF EBANK.  HUGH BLAIR-SMITH WARNS US THAT BITS15-12 OF EBANK MAY BECOME
#	SIGNIFICANT SOMEDAY IN THE FUTURE.  IF/WHEN THAT HAPPENS, THE PROGRAM SHOULD INSURE (BY MASKING ETC.)
#	THAT BITS 15-12 OF EBANK ARE ZERO.
#
# INITIALIZATION REQUIRED -- TO INTERRUPT CURRENT LIST AND START A NEW ONE.
#	1. ADRES OF DOWNLINK LIST INTO DNLSTADR
#	2. NEGONE INTO SUBLIST
#	3. NEGONE INTO DNECADR

; ============================================================================
; DOWNLINK INTERRUPT ENTRY POINT
;
; Every 20 milliseconds during the Apollo 11 mission, the spacecraft
; telemetry system sent an interrupt pulse to the AGC. This interrupt
; arrived regardless of what program was running - whether Armstrong was
; monitoring landing radar during descent or the computer was calculating
; orbital trajectories. The interrupt mechanism ensured Mission Control
; received continuous telemetry updates throughout all mission phases.
; ============================================================================

		BANK	22
		SETLOC	DOWNTELM
		BANK

		EBANK=	DNTMBUFF

		COUNT*	$$/DPROG

; DODOWNTM - Downlink Telemetry Interrupt Handler
; Entered via DOWNRUPT interrupt every 20 milliseconds (50 Hz rate).
; Saves interrupt context (accumulator to BANKRUPT, return address to
; QRUPT), then checks word order status before dispatching to appropriate
; telemetry processing phase (DNPHASE1 initialization or DNPHASE2 data
; transmission). Critical for maintaining continuous ground monitoring.

DODOWNTM	TS	BANKRUPT
		EXTEND
		QXCH	QRUPT		# SAVE Q
		TCF	WOTEST

; WO1 - Word Order Bit Management
; The AGC transmits telemetry data in 14-bit words. The word order bit
; (bit 7 in channel 13) indicates which word of a pair is being sent.
; This routine sets word order to 1, preparing for the two-word data
; transmission format required by the downlink telemetry system.

WO1		EXTEND			# SET WORD ORDER BIT TO 1 ONLY IF IT
		WOR	CHAN13		# ALREADY ISN'T
		TC	DNTMGOTO	# GOTO APPROPRIATE PHASE OF PROGRAM

; ============================================================================
; DNPHASE1 - Telemetry List Initialization Phase
;
; On the first interrupt after selecting a new downlist (via major mode
; change, fresh start, or crew command), this routine initializes control
; variables. During Apollo 11, different mission phases (launch, descent,
; landing, ascent) used different downlists to prioritize relevant data.
; For example, during lunar descent, the downlist emphasized landing radar,
; altitude, and velocity data that Armstrong and Aldrin needed.
; ============================================================================

DNPHASE1	CA	NEGONE		# INITIALIZE ALL CONTROL WORDS
		TS	SUBLIST		# WORDS TO MINUS ONE
		TS	DNECADR
		CA	LDNPHAS2	# SET DNTMGOTO = 0 ALL SUSEQUENT DOWRUPTS
		TS	DNTMGOTO	# GO TO DNPHASE2
		TCF	NEWLIST

; ============================================================================
; DNPHASE2 - Main Telemetry Data Transmission Phase
;
; After initialization completes, all subsequent 20ms interrupts enter
; here. This routine orchestrates the continuous flow of AGC data to
; Mission Control. During Apollo 11's descent to the lunar surface,
; this code executed 50 times per second, packaging critical navigation,
; guidance, and system status data for Armstrong, Aldrin, and the flight
; controllers monitoring from Houston.
; ============================================================================

DNPHASE2	CCS	DNECADR		# SENDING OF DATA IN PROGRESS
DODNADR		TC	FETCH2WD	# YES -- THEN FETCH THE NEXT 2 SP WORDS
MINTIME2	-1DNADR	TIME2		# NEGATIVE OF TIME2 1DNADR
		TCF	+1		# (ECADR OF 3776 + 74001 = 77777)

		CCS	SUBLIST		# IS THE SUBLIST IN CONTROL
		TCF	NEXTINSL	# YES
# Page 992
DNADRDCR	OCT	74001		# DNADR COUNT AND ECADR DECREMENTER

; CHKLIST - Control List Position Check
; Examines control list pointer (CTLIST) to determine if we've reached
; end of current list. If negative, initiates loading of new list.
; Otherwise proceeds to next control list item.

CHKLIST		CA	CTLIST
		EXTEND
		BZMF	NEWLIST		# IT WILL BE NEGATIVE AT END OF LIST
		TCF	NEXTINCL

; NEWLIST - Initialize New Downlink List
; Loads starting address of new downlist from DNTABLE based on current
; downlist code (DNLSTCOD). Different mission phases used different lists:
; descent emphasized landing radar and altitude, ascent focused on orbital
; parameters for rendezvous with Columbia in lunar orbit.

NEWLIST		INDEX	DNLSTCOD
		CA	DNTABLE		# INITIALIZE CTLIST WITH
		TS	CTLIST		#	STARTING ADDRESS OF NEW LIST
		CS	DNLSTCOD
		TCF	SENDID 	+3

; NEXTINCL - Process Next Control List Item
; Fetches next word from control list. Positive value indicates another
; item follows; increments pointer. Zero or negative values signal special
; conditions requiring different handling (end of list or final item).

NEXTINCL	INDEX	CTLIST
		CA	0
		CCS	A
		INCR	CTLIST		# SET POINTER TO PICK UP NEXT CTLIST WORD
		TCF	+4		# ON NEXT ENTRY TO PROG.  (A SHOULD NOT =0)
		XCH	CTLIST		# SET CTLIST TO NEGATIVE AND PLACE(CODING)
		COM			# UNCOMPLEMENTED DNADR INTO A.    (FOR LA)
		XCH	CTLIST		#                                 (ST IN )
; Decode DNADR Type and Handle Special Cases
; Each control list entry is a DNADR (downlink address) specifying what
; data to transmit. This section decodes the DNADR type by examining
; bits 13-14:
;   Regular DNADR (bits 13-14 clear): Erasable memory address
;   DNPTR (bit 13 set): Pointer to sublist
;   DNCHAN (bit 14 set): I/O channel data
; Special case: TIME2 requires word order bit = 0 for proper timing sync.

 +4		INCR	A		#                                 (CTLIST)
 		TS	DNECADR		# SAVE DNADR
		AD	MINTIME2	# TEST FOR TIME2 (NEG. OF ECADR)
		CCS	A
		TCF	SETWO	+1	# DON'T SET WORD ORDER CODE
MINB1314	OCT	47777		# MINUS BIT 13 AND 14 (CAN'T GET HERE)
		TCF	SETWO 	+1	# DON'T SET WORD ORDER CODE
SETWO		TC	WOZERO		# GO SET WORD ORDER CODE TO ZERO.
 +1		CA	DNECADR		# RELOAD A WITH THE DNADR.
 +2		AD	MINB1314	# IS THIS A REGULAR DNADR?
 		EXTEND
		BZMF	FETCH2WD	# YES.  (A MUST NEVER BE ZERO)
		AD	MINB12		# NO.  IS IT A POINTER (DNPTR) OR A
		EXTEND			#	CHANNEL(DNCHAN)
		BZMF	DODNPTR		# IT'S A POINTER.  (A MUST NEVER BE ZERO)

; ============================================================================
; DODNCHAN - Transmit I/O Channel Data
;
; When DNADR indicates a channel (DNCHAN opcode with bit 14 set), this
; routine reads two consecutive I/O channels and packages them for downlink.
; During Apollo 11's descent, this transmitted critical hardware status:
; IMU gimbal angles, RCS jet commands, throttle settings, and landing radar.
; Mission Control monitored these channels to verify spacecraft health and
; validate guidance computer commands to the LM's control systems.
; ============================================================================

DODNCHAN	TC	6		# (EXECUTED AS EXTEND)  IT'S A CHANNEL
		INDEX	DNECADR
		INDEX	0 	-4000	# (EXECUTED AS READ)
		TS	L
		TC	6		# (EXECUTED AS EXTEND)
		INDEX	DNECADR
		INDEX	0 	-4001	# (EXECUTED AS READ)
		TS	DNECADR		# SET DNECADR
		CA	NEGONE		#	TO MINUS
		XCH	DNECADR		#		WHILE PRESERVING A.
		TCF	DNTMEXIT	# GO SEND CHANNELS

; WOZERO - Clear Word Order Bit
; Sets word order bit (bit 7 in channel 13) to zero. Required before
; transmitting TIME2 to maintain timing synchronization between AGC and
; ground station receivers. Called via TC Q (subroutine return).

WOZERO		CS	BIT7
		EXTEND
		WAND	CHAN13		# SET WORD ORDER CODE TO ZERO
# Page 993
		TC	Q		# RETURN TO CALLER

; ============================================================================
; DODNPTR - Process Downlist Pointer (Sublist Entry)
;
; When DNADR is a DNPTR (bit 13 set), this routine transitions to a sublist.
; Sublists organize related data: navigation state vectors, IMU angles,
; guidance parameters. Two sublist types exist:
;   Regular sublist: Sequential data items
;   Snapshot sublist: Special processing via SNAPLOOP for time-critical data
; ============================================================================

DODNPTR		INDEX	DNECADR		# DNECADR CONTAINS ADRES OF SUBLIST
		0	0		# CLEAR AND ADD LIST ENTRY INTO A.
		CCS	A		# IS THIS A SNAPSHOT SUBLIST
		CA	DNECADR		# NO, IT IS A REGULAR SUBLIST.
		TCF	DOSUBLST	# A MUST NOT BE ZERO.

		XCH	DNECADR		# YES.  IT IS A SNAPSHOT SUBLIST.
		TS	SUBLIST		# C(DNECADR) INTO SUBLIST
		CAF	ZERO		#	A    INTO     A
		XCH	TMINDEX		# (NOTE:  TMINDEX = DNECADR)

# THE FOLLOWING CODING (FROM SNAPLOOP TO SNAPEND) IS FOR THE PURPOSE OF TAKING A SNAPSHOT OF 12 DP REGISTERS.
# THIS IS DONE BY SAVING 11 DP REGISTERS IN DNTMBUFF AND SENDING THE FIRST DP WORD IMMEDIATELY.
# THE SNAPSHOT PROCESSING IS THE MOST TIME CONSUMING AND THEREFORE THE CODING AND LIST STRUCTURE WERE DESIGNED
# TO MINIMIZE TIME.  THE TIME OPTIMIZATION RESULTS IN RULES UNIQUE TO THE SNAPSHOT PORTION OF THE DOWNLIST.
# THESE RULES ARE ......
#	1.	ONLY 1DNADR'S CAN APPEAR IN THE SNAPSHOT SUBLIST
#	2.	THE 1DNADR'S CANNOT REFER TO THE FIRST LOCATION IN ANY BANK.

; ============================================================================
; SNAPLOOP - Snapshot Data Capture Loop
;
; COMMENT-ONLY READERS: This loop captures a "frozen moment" of the guidance
; computer's state. During Apollo 11's descent, Mission Control needed to
; see position, velocity, and attitude all measured at the exact same instant.
; The snapshot mechanism buffers 12 double-precision values (24 words total),
; ensuring ground controllers received synchronized navigation data that
; represented a coherent spacecraft state, not values sampled across multiple
; 20-millisecond cycles.
;
; CODE-ALONG READERS: Implements high-speed buffer capture of erasable memory
; locations specified in snapshot sublist. Processes 1DNADR entries, uses
; indexed EBANK addressing to read DP values from target bank, stores in
; DNTMBUFF for subsequent transmission. Time-optimized coding required due
; to snapshot processing being the most time-consuming downlist operation.
; ============================================================================

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
		XCH	SUBLIST		#		TO NEGATIVE VALUES
		TS	EBANK
		MASK	LOW8
		EXTEND
		INDEX	A
		EBANK=	1401
		DCA	1401		# PICK UP FIRST 2 WORDS OF SNAPSHOT.
# Page 994
		EBANK=	DNTMBUFF
SNAPEND		TCF	DNTMEXIT	# NOW TO SEND THEM.

; ============================================================================
; FETCH2WD - Fetch Two-Word Data Entry
;
; COMMENT-ONLY READERS: After processing multi-word downlist entries, the
; computer retrieves the actual data values from memory. This routine handles
; the standard case: fetching 2 consecutive words (one double-precision value)
; from erasable memory and preparing them for transmission to Earth.
;
; CODE-ALONG READERS: Fetches 2-word (DP) data entry specified by DNECADR.
; DNECADR format: bits 15-8 contain EBANK, bits 7-0 contain relative address.
; Sets EBANK for bank selection, extracts relative address, performs indexed
; DCA from erasable bank 1400 base. Decrements DNECADR via DNADRDCR constant
; for multi-word sequence processing. Returns via DNTMEXIT with data in A,L.
; ============================================================================

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
		TCF	DNTMEXIT	# NOW GO SEND THEM.

; ============================================================================
; DOSUBLST/NEXTINSL - Process Sublist Entries
;
; COMMENT-ONLY READERS: Downlists are organized hierarchically - the main
; list points to multiple sublists, each containing related data items. This
; routine walks through a sublist entry-by-entry, processing each data
; address until reaching the sublist's end marker (zero entry). During the
; Apollo 11 mission, one sublist might contain all IMU data, another all
; navigation state data, enabling organized ground displays.
;
; CODE-ALONG READERS: Processes entries in current sublist pointed to by
; SUBLIST pointer. Each iteration: fetches next sublist entry via indexed CA,
; tests for end (zero = sublist complete, sets SUBLIST negative), increments
; SUBLIST pointer for next entry, increments and stores DNADR for processing.
; Uses CCS A test to detect zero (end marker). Rejoins common SETWO+2 path
; for DNADR decoding. Sublist structure: sequence of 1DNADR/NDNADR codes
; terminated by zero word.
; ============================================================================

DOSUBLST	TS	SUBLIST		# SET SUBLIST POINTER
NEXTINSL	INDEX	SUBLIST
		0	0		# = CA SSSS (SSSS = NEXT ENTRY IN SUBLIST)
		CCS	A		# IS IT THE END OF THE SUBLIST
		INCR	SUBLIST		# NO --
		TCF	+4
		TS	SUBLIST		# SAVE A.
		CA	NEGONE		# SET SUBLIST TO MINUS
		XCH	SUBLIST		# RETRIEVE A.
 +4		INCR	A
 		TS	DNECADR		# SAVE DNADR
		TCF	SETWO +2	# GO USE COMMON CODING (PROLEMS WOULD
					# OCCUR IF THE PROGRAM ENCOUNTERED A
					# DNPTR NOW)

; ============================================================================
; DNTMEXIT - Downlink Telemetry Exit Point
;
; COMMENT-ONLY READERS: This is the final step of each 20-millisecond
; telemetry cycle. The guidance computer has selected two data words and
; placed them in the A and L registers. Now they're written to output
; channels 34 and 35, where the spacecraft's telemetry system converts them
; to radio signals beamed to Earth. Every 20 milliseconds during the entire
; Apollo 11 mission - from launch through splashdown - this routine executed,
; feeding the steady stream of data that kept Mission Control informed of
; the spacecraft's state.
;
; CODE-ALONG READERS: Common exit point for all downlist processing paths.
; Performs EXTEND WRITE DNTM1 to send A register contents to output channel
; 34, then transfers L register to A and performs EXTEND WRITE DNTM2 to send
; to channel 35. Two words (one DP value) transmitted per 20ms interrupt.
; DNTM1/DNTM2 are channel addresses for downlink interface. Returns via
; TCF RESUME to restore interrupted program state. Critical timing: must
; complete within interrupt service time budget.
; ============================================================================

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

# Page 995
# SUBROUTINE NAME -- DNDUMP
#
# FUNCTIONAL DESCRIPTION -- TO SEND (DUMP) ALL ERASABLE STORAGE 'N' TIMES. (N=1 TO 4).  BANKS ARE SENT ONE AT A TIME
#	EACH BANK IS PRECEDED BY AN ID WORD, SYNCH BITS, ECADR AND TIME1 FOLLOWED BY THE 256D WORDS OF EACH
#	EBANK.  EBANKS ARE DUMPED IN ORDER (I.E., EBANK 0 FIRST, THEN EBANK1 ETC.)
#
# CALLING SEQUENCE -- THE GROUND OR ASTRONAUT BY KEYING V74E CAN INITIALIZE THE DUMP.
#	AFTER KEYING IN V74E THE CURRENT DOWNLIST WILL BE IMMEDIATELY TERMINATED AND THE DOWNLINK ERASABLE DUMP
#	WILL BEGIN.
#
#	ONCE INITIATED THE DOWNLINK ERASABLE DUMP CAN BE TERMINATED (AND INTERRUPTED DOWNLIST REINSTATED) ONLY
#	BY THE FOLLOWING:
#
#	1.	A FRESH START
#	2.	COMPLETION OF ALL DOWNLINK DUMPS REQUESTED (ACCORDING TO BITS SET IN DUMPCNT).  NOTE THAT DUMPCNT
#		CAN BE ALTERED BY A V21N01.
#	3.	AND INVOLUNTARILY BY A RESTART.
#
# NORMAL EXIT MODE -- TCF DNPHASE1
#
# ALARM OR ABORT MODE -- NONE
#
# *SUBROUTINES CALLED -- NONE
#
# ERASABLE INITIALIZATION REQUIRED --
#	DUMPCNT		OCT 20000	IF 4 COMPLETE ERASABLE DUMPS ARE DESIRED
#	DUMPCNT		OCT 10000	IF 2 COMPLETE ERASABLE DUMPS ARE DESIRED
#	DUMPCNT		OCT 04000	IF 1 COMPLETE ERASABLE DUMP  IS  DESIRED
#
# DEBRIS -- DUMPLOC, DUMPSW, DNTMGOTO, EBANK, AND CENTRAL REGISTERS
#
# TIMING --	TIME (IN SECS) = ((NO.DUMPS)*(NO.EBANKS)*(WDSPEREBANK + NO.IDWDS)) / NO.WDSPERSEC
#		TIME (IN SECS) =  (   4    )*(    8    )*(    256     +     4   )  /     100
#		THUS TIME (IN SECS TO SEND DUMP OF ERASABLE 4 TIMES VIA DOWNLINK) = 83.2 SECONDS
#
# STRUCTURE OF ONE EBANK AS IT IS SENT BY DOWNLINK PROGRAM --
#	(REMINDER -- THIS ONLY DESCRIBES ONE OF THE 8 EBANKS X 4 (DUMPS) = 32 EBANKS WHICH WILL BE SENT BY DNDUMP)
#
#	DOWNLIST				W
#	  WORD	TAKEN FROM CONTENTS OF	EXAMPLE	O	COMMENTS
#	    1	ERASID			 0177X	0	DOWNLIST I.D. FOR DOWNLINK ERASABLE DUMP (X=7 CSM, 6 LM)
#	    2	LOWIDCOD		 77340 	1	DOWNLINK SYNCH BITS.  (SAME ONE USED IN ALL OTHER DOWNLISTS)
#	    3	DUMPLOC			 13400	1	(SEE NOTES ON DUMPLOC) 1 = 3RD ERAS DUMP, 3400=ECADR OF 5TH WD
#	    4	TIME1			 14120	1	TIME IN CENTISECONDS
#	    5	FIRST WORD OF EBANK X	 03400	1	IN THIS EXAMPLE THIS WORD = CONTENTS OF E7,1400 (ECADR 3400)
#	    6	2ND   WORD OF EBANK X	 00142	1	IN THIS EXAMPLE THIS WORD = CONTENTS OF E7,1401 (ECADR 3401)
#	    7   3RD   WORD OF EBANK X	 00142	1	IN THIS EXAMPLE THIS WORD = CONTENTS OF E7,1402 (ECADR 3402)
#	    .
#	    .
#	    .
#	 260D	256TH WORD OF EBANK X	 03777	1	IN THIS EXAMPLE THIS WORD = CONTENTS OF E7,1777 (ECADR 3777)
#
# NOTE --	DUMPLOC CONTAINS THE COUNTER AND ECADR FOR EACH WORD BEING SENT.
#		THE BIT STRUCTURE OF DUMPLOC IS FOLLOW --
#						X = NOT USED
#		X ABC EEE RRRRRRRR	      ABC = ERASABLE DUMP COUNTER (I.E. ABC = 0,1,2, OR 3 WHICH MEANS THAT
#						    COMPLETE ERASABLE DUMP NUMBER 1,2,3, OR 4 RESPECTIVELY IS IN PROGRESS)
#					      EEE = EBANK BITS
#					 RRRRRRRR = RELATIVE ADDRESS WITHIN AN EBANK

# Page 996
; ============================================================================
; DNDUMP/DNDUMPI - Downlink Erasable Memory Dump Subroutine
;
; COMMENT-ONLY READERS: Imagine Mission Control wanting to see EVERYTHING
; stored in the guidance computer's memory - every navigation calculation,
; every flag setting, every intermediate result. The erasable dump makes this
; possible. When commanded (via V74E from the crew or uplinked from Houston),
; the AGC temporarily abandons its normal telemetry and instead streams its
; entire 2K words of RAM memory to Earth. This takes 83.2 seconds for four
; complete dumps - a comprehensive snapshot used for post-flight analysis and
; real-time troubleshooting.
;
; During Apollo 11, erasable dumps provided engineers with detailed forensic
; data to understand anomalies. If something unexpected happened (like the
; famous 1202 alarms during descent), a subsequent erasable dump could reveal
; exactly what the computer was doing - which programs were running, what
; values were in key variables, what the task queues looked like.
;
; The dump is organized bank-by-bank (8 banks, 256 words each). Each bank
; transmission begins with an ID word, synchronization bits, the memory
; address being sent (DUMPLOC format: dump counter + bank + address), and
; current mission time, followed by all 256 words from that bank. Ground
; systems reassemble this into a complete memory image.
;
; CODE-ALONG READERS: Implements erasable memory dump initiated by V74E
; (verb 74). DUMPCNT controls number of complete dumps (OCT 04000 = 1 dump,
; OCT 10000 = 2 dumps, OCT 20000 = 4 dumps). DUMPLOC tracks progress with
; bit structure: X ABC EEE RRRRRRRR where ABC = dump counter (0-3),
; EEE = EBANK (0-7), RRRRRRRR = relative address (0-377 octal).
;
; DNDUMPI initializes dump by zeroing DUMPLOC, sending ID/synch via SENDID,
; setting up continuation address LDNDUMP1 in DNTMGOTO, then sending DUMPLOC
; and TIME1 as first two words. DNDUMP continues by incrementing DUMPLOC by 2
; (DP word), checking for bank boundaries (LOW8 mask isolates relative addr,
; CCS tests for zero = bank complete), and checking for dump completion
; (DUMPLOC masked with DUMPCNT). Transmits 260 words per bank (4 header + 256
; data) at 100 words/sec downlink rate. Total time: 4 dumps × 8 banks × 260
; words ÷ 100 words/sec = 83.2 seconds.
; ============================================================================

DNDUMPI		CA	ZERO		# INITIALIZE DOWNLINK
		TS	DUMPLOC		# ERASABLE DUMP
 +2		TC	SENDID		# GO SEND ID AND SYNCH BITS
		CA	LDNDUMP1	# SET DNTMGOTO
		TS	DNTMGOTO	# TO LOCATION FOR NEXT PASS
		CA	TIME1		# PLACE TIME1
		XCH	L		# INTO L
		CA	DUMPLOC		# AND ECADR OF THIS EBANK INTO A
		TCF	DNTMEXIT	# SEND DUMPLOC AND TIME1

LDNDUMP		ADRES	DNDUMP
LDNDUMP1	ADRES	DNDUMP1

; Bank boundary check and dump completion test. Increments DUMPLOC by 2,
; checks if we've completed an EBANK (256 words), and tests whether we've
; finished all N complete dumps (N specified in DUMPCNT). If bank complete
; but more dumps remain, reinitializes for next bank. If all dumps complete,
; returns to normal downlist processing via DNPHASE1.

DNDUMP		CA	TWO		# INCREMENT ECADR IN DUMPLOC
		ADS	DUMPLOC		# TO NEXT DP WORD TO BE
		MASK	LOW8		# DUMPED AND SAVE IT.
		CCS	A		# IS THIS THE BEGINNING OF A NEW EBANK
		TCF	DNDUMP2		# NO -- THEN CONTINUE DUMPING
		CA	DUMPLOC		# YES -- IS THIS THE END OF THE
		MASK	DUMPCNT		# N TH (N = 1 TO 4) COMPLETE ERASABLE
		MASK	PRIO34		# DUMP (BIT14 FOR 4, BIT13 FOR 2 OR BIT12
		CCS	A		# FOR 1 COMPLETE ERASABLE DUMP(S)).
		TCF	DNPHASE1	# YES -- START SENDING INTERRUPTED DOWNLIST
					# AGAIN
		TCF	DNDUMPI	+2	# NO -- GO BACK AND INITIALIZE NEXT BANK

; DNDUMP1 - Set up continuation address for sending remaining bank words.
; After sending the 4-word header (ID, synch, DUMPLOC, TIME1), subsequent
; 20ms interrupts execute DNDUMP1 to continue transmitting the 256 data
; words from the current EBANK. Sets DNTMGOTO to point to main DNDUMP logic.

DNDUMP1		CA	LDNDUMP		# SET DNTMGOTO
		TS	DNTMGOTO	# FOR WORDS 3 TO 256D OF CURRENT EBANK

; DNDUMP2 - Fetch DP word from erasable memory at address in DUMPLOC.
; Sets EBANK from high bits of DUMPLOC, isolates relative address (LOW8),
; uses INDEX addressing to retrieve DP word. MASK (not CA) instruction used
; to avoid altering editing registers (20-23). Fetched DP word placed in
; A and L registers for transmission via DNTMEXIT.

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
; SENDID - Send Downlist Identification Code
;
; COMMENT-ONLY READERS: Before transmitting each downlist, the AGC sends a
; special ID code that tells the ground what type of data is about to arrive.
; Think of it like a chapter heading in a book - it helps Mission Control's
; computers know how to interpret the incoming stream. During an erasable
; dump, this ID marks the beginning of each memory bank dump.
;
; CODE-ALONG READERS: Dual-entrance subroutine. Erasable dump uses first
; entrance (EXTEND QXCH DNTMGOTO) to save return address. Regular downlist
; uses second entrance (after setting A to ID code). Calls WOZERO to reset
; word order bit (channel 13 bit 7), then sends ID code and LOWIDCOD via
; DNTMEXIT. ID codes defined in DOWNLINK_LISTS distinguish different list
; types for ground processing.
; ============================================================================

SENDID		EXTEND			# ** ENTRANCE USED BY ERASABLE DUMP PROG. **
		QXCH	DNTMGOTO	# SET DNTMGOTO SO NEXT TIME PROG WILL GO
		CAF	ERASID		# TO LOCATION FOLLOWING `TC SENDID'

		TS	L		# ** ENTRANCE USED BY REGULAR DOWNLINK PG **
# Page 997
		TC	WOZERO		# GO SET WORD ORDER CODE TO ZERO
		CAF	LOWIDCOD	# PLACE SPECIAL ID CODE INTO L
		XCH	L		# AND ID BACK INTO A
		TCF	DNTMEXIT	# SEND DOWNLIST ID CODE(S).

; WOTEST - Word Order Bit Test
; Tests channel 13 bit 7 (word order code). If bit is zero (at beginning of
; new downlist), continues normal processing via DNTMGOTO. If bit is one,
; sets it again and branches to WO1. Word order bit alternates between lists
; to help ground synchronization detect list boundaries.

WOTEST		CA	BIT7		# AT THE BEGINNING OF THE LIST THE WORD
		EXTEND			# ORDER BIT WILL BE SET BACK TO ZERO
		RAND	CHAN13
		CCS	A
		TC	DNTMGOTO
		CA	BIT7
		TCF	WO1

