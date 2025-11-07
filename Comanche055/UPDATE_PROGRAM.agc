# Copyright:    Public domain.
# Filename:     UPDATE_PROGRAM.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1497-1507
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-07 RSB	Adapted from Colossus249/UPDATE_PROGRAM.agc
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
; FILE: UPDATE_PROGRAM.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Uplink command processing handling state vector updates and navigation
;        corrections from ground. Processes Mission Control commands transmitted
;        via MSFN (Manned Space Flight Network), verifies data integrity through
;        triple redundancy checking, and updates spacecraft navigation state
;        ensuring accurate trajectory knowledge throughout Apollo 11 mission.
;
; COMMENT-ONLY READERS: This program received and processed navigation updates
;        sent from Mission Control to correct the spacecraft's position knowledge.
;        Ground controllers could update the spacecraft clock, position/velocity
;        vectors, and critical navigation matrices to maintain accuracy during
;        the quarter-million-mile journey to the Moon and back.
; CODE-ALONG READERS: Study uplink command protocol including triple-character
;        redundancy verification, extended verb processing (V70-V73), data
;        buffering in UPBUFF, verification sequence allowing correction before
;        commitment, and restart protection during critical update operations.
; ============================================================================

# Page 1497
;
; ============================================================================
; UPLINK UPDATE SYSTEM OVERVIEW
;
; During Apollo 11's journey, Mission Control continuously tracked the spacecraft
; using ground-based radar and computed refined navigation solutions. This program
; (P27) enabled controllers to uplink corrections to the spacecraft's onboard
; navigation state, ensuring the AGC's internal knowledge of position, velocity,
; and time remained accurate throughout the mission.
;
; The uplink system used the Manned Space Flight Network (MSFN) to transmit
; commands via S-band radio. Each uplink character was sent with triple redundancy
; (handled by UPRUPT interrupt in KEYRUPT_UPRUPT.agc) to detect transmission
; errors. Ground controllers at Houston could send navigation updates only during
; specific mission phases when crew workload was low and systems were stable.
; ============================================================================
;
# PROGRAM NAME:		P27
# WRITTEN BY:		KILROY/ DE WOLF
#
# MOD NO:		0
# MOD BY:		KILROY
# DATE:			01DEC67
#
# LOG SECTION:		UPDATE PROGRAM.
#
# FUNCT. DESCR.:	P27 (THE UPDATE PROGRAM) PROCESSES COMMANDS AND DATA
#			INSERTIONS REQUESTED BY THE GROUND VIA UPLINK.
#			THE P27 PROGRAM WILL ACCEPT UPDATES
#			ONLY DURING P00 FOR THE LM, AND ONLY DURING P00,
#			P02, AND FRESH START FOR THE CSM.
#
# CALLING SEQ:		PROGRAM IS INITIATED BY UPLINK ENTRY OF VERBS 70, 71, 72, AND 73.
#
# SUBROUTINES:		TESTXACT, NEWMODEX, NEWMODEX +3, GOXDSPF, BANKCALL, FINDVAC, INTPRET, INTSTALL, TPAGREE,
#			INTWAKEU, ENDEXT, POSTJUMP, FALTON, NEWPHASE, PHASCHNG
#
# NORMAL EXIT:		TC ENDEXT
#
# ALARM/ABORT:		TC FALTON FOLLOWED BY TC ENDEXT
#
# RESTARTS:		P27 IS RESTART PROTECTED IN TWO WAYS ...
#			1.	PRIOR TO VERIFLAG INVERSION(WHICH IS CAUSED BY THE GROUND/ASTRONAUT'S VERIFICATION OF UPDATE
#				DATA BY SENDING A V33E WHEN V21N02 IS FLASHING)---
#				NO PROTECTION EXCEPT PRE-P27 MODE IS RESTORED, COAST + ALIGN DOWNLIST IS SELECTED AND UPLINE
#				ACTIVITY LIGHT IS TURNED OFF.(JUST AS IF A V34E WAS SENT DURING P27 DATA LOADS).
#				V70,V71,V72 OR V73 WILL HAVE TO BE COMPLETELY RESENT BY USER.
#			2.	AFTER VERIFLAG INVERSION(WHEN UPDATE OF THE SPECIFIED ERASABLES IS BEING PERFORMED)---
#				PROTECTED AGAINST RESTARTS.
#
# DEBRIS:		UPBUFF	(20D)	TEMP STORAGE FOR ADDRESSES AND CONTENTS.
#			UPVERB	(1)	VERB NUMBER MINUS 70D (E.G. FOR V72, UPVERB = 72D - 70D = 2)
#			UPOLDMOD(1)	FOR MAJOR MODE INTERRUPTED BY P27.
#			COMPNUMB(1)	TOTAL NUMBER OF COMPONENTS TO BE TRANSMITTED.
#			UPCOUNT	(1)	ACTUAL NUMBER OF COMPONENTS RECEIVED.
#			UPTEMP	(1)	SCRATCH, BUT USUALLY CONTAINS COMPONENT NUMBER TO BE CHANGED DURING VERIFY CYCLE
#
# INPUT:
#
#	ENTRY:			DESCRIPTION
#
#	V70EXXXXXEXXXXXE	(LIFTOFF TIME INCREMENT) DOUBLE PRECISION OCTAL TIME INCREMENT, XXXXX XXXXX,
#				IS ADDED TO TEPHEM, SUBTRACTED FROM AGC CLOCK(TIME2,TIME1), SUBTRACTED FROM CSM STATE
#				VECTOR TIME(TETCSM) AND SUBTRACTED FROM LEM STATE VECTOR TIME(TETLEM).
#				THE DP OCTAL TIME INCREMENT IS SCALED AT 2(28).
# Page 1498
#	V71EIIEAAAAE		(CONTIGUOUS BLOCK UPDATE) II-2 OCTAL COMPONENTS,XXXXX,
#	XXXXXE			ARE LOADED INTO ERASABLE STARTING AT ECADR, AAAA.
#	XXXXXE			IT IS .GE. 3 .AND. .LE. 20D.,
#				AND (AAAA + II -3) DOES NOT PRODUCE AN ADDRESS IN THE
#				NEXT BANK.
#	  .			SCALING IS SAME AS INTERNAL REGISTERS.
#
#	V72EIIE			(SCATTER UPDATE) (II-1)/2 OCTAL COMPONENTS,XXXXX, ARE
#	AAAAEXXXXXE		LOADED INTO ERASABLE LOCATIONS, AAAA.
#	AAAAEXXXXXE		II IS .GE. 3 .AND. .LE. 19D, AND MUST BE ODD.
#	  .			SCALING IS SAME AS INTERNAL REGISTERS.
#
#	V73EXXXXXEXXXXXE	(OCTAL CLOCK INCREMENT) DOUBLE PRECISION OCTAL TIME
#				INCREMENT XXXXX XXXXX, IS ADDED TO THE AGC CLOCK, IN
#				CENTISECONDS SCALED AT (2)28.
#				THIS LOAD IS THE OCTAL EQUIVALENT OF V55.
#
# OUTPUT:		IN ADDITION TO THE ABOVE REGISTER LOADS, ALL UPDATES
#			COMPLEMENT BIT3 OF FLAGWORD7.
#
# ADDITIONAL NOTES:	VERB 71, JUST DEFINED ABOVE WILL BE USED TO PERFORM BUT NOT LIMITED TO THE FOLLOWING UPDATES --
#			1.  CSM/LM STATE VECTOR UPDATE
#			2.  REFSMMAT UPDATE
#
#	THE FOLLOWING COMMENTS DELINEATE EACH SPECIAL UPDATE ---
#
#	1.  CSM/LM STATE VECTOR UPDATE(ALL DATA ENTRIES IN OCTAL)
#		ENTRIES:	DATA DEFINITION:				SCALE FACTORS:
#		V71E		CONTIGUOUS BLOCK UPDATE VERB
#		   21E		NUMBER OF COMPONENTS FOR STATE VECTOR UPDATE
#		 AAAAE		ECADR OF 'UPSVFLAG'
#		XXXXXE		STATE VECTOR IDENTIFIER: 00001 FOR CSM, 77776 FOR LEM - EARTH SPHERE OF INFLUENCE SCALING
#							 00002 FOR CSM, 77775 FOR LEM - LUNAR SPHERE OF INFLUENCE SCALING
#		XXXXXEXXXXXE	X POSITION
#		XXXXXEXXXXXE	Y POSITION
#		XXXXXEXXXXXE	Z POSITION
#		XXXXXEXXXXXE	X VELOCITY
#		XXXXXEXXXXXE	Y VELOCITY
#		XXXXXEXXXXXE	Z VELOCITY
#		XXXXXEXXXXXE	TIME FROM AGC CLOCK ZERO
#		V33E		VERB 33 TO SIGNAL THAT THE STATE VECTOR IS READY TO BE STORED.
#
#	2.  REFSMMAT(ALL DATA ENTRIES IN OCTAL)
#		ENTRIES:	DATA DEFINITIONS:				SCALE FACTORS:
# Page 1499
#		V71E		CONTIGUOUS BLOCK UPDATE VERB
#		   24E		NUMBER OF COMPONENTS FOR REFSMMAT UPDATE
#		 AAAAE		ECADR OF 'REFSMMAT'
#		XXXXXEXXXXXE	ROW 1 COLUMN 1					2(-1)
#		XXXXXEXXXXXE	ROW 1 COLUMN 2					2(-1)
#		XXXXXEXXXXXE	ROW 1 COLUMN 3					2(-1)
#		XXXXXEXXXXXE	ROW 2 COLUMN 1					2(-1)
#		XXXXXEXXXXXE	ROW 2 COLUMN 2					2(-1)
#		XXXXXEXXXXXE	ROW 2 COLUMN 3					2(-1)
#		XXXXXEXXXXXE	ROW 3 COLUMN 1					2(-1)
#		XXXXXEXXXXXE	ROW 3 COLUMN 2					2(-1)
#		XXXXXEXXXXXE	ROW 3 COLUMN 3					2(-1)
#		V33E		VERB 33 TO SIGNAL THAT REFSMMAT IS READY TO BE STORED.
;
; ============================================================================
; EXTENDED VERB ENTRY POINTS: V70, V71, V72, V73
;
; Mission Control initiates updates by sending one of four extended verbs:
;
; V70: LIFTOFF TIME INCREMENT - Adjusts mission reference time (TEPHEM) and
;      decrements AGC clock, CSM state vector time (TETCSM), and LM state
;      vector time (TETLEM). Used to synchronize clocks after liftoff or to
;      correct accumulated time drift during mission.
;
; V71: CONTIGUOUS BLOCK UPDATE - Loads 3-20 consecutive memory locations
;      starting at a specified address. Efficient for updating tables or
;      matrices stored in sequential erasable memory locations. Used for
;      state vector and REFSMMAT (reference to stable member matrix) updates.
;
; V72: SCATTER UPDATE - Loads data into non-consecutive memory locations by
;      specifying address/data pairs. Used when updating widely-separated
;      parameters that don't form a contiguous block.
;
; V73: OCTAL CLOCK INCREMENT - Directly adds time increment to AGC clock
;      (TIME2, TIME1) in centiseconds scaled at 2^28. Simpler than V70 as
;      it only affects the clock without adjusting state vector times.
;
; Each entry point loads its verb identifier (0-3) into A register and
; converges at common entry logic (V73UPDAT +1) where verb code is saved,
; display system availability checked, and major mode validated.
; ============================================================================
;

		BANK	07
		SETLOC	EXTVERBS
		BANK

		EBANK=	TEPHEM

		COUNT*	$$/P27
;
; Extended verb entry points. Each loads verb code (0 for V70, 1 for V71,
; 2 for V72, 3 for V73) and branches to common validation sequence.
;
V70UPDAT	CAF	UP70		# COMES HERE ON V70E
		TCF	V73UPDAT +1

V71UPDAT	CAF	UP71		# COMES HERE ON V71E
		TCF	V73UPDAT +1

V72UPDAT	CAF	UP72		# COMES HERE ON V72E
		TCF	V73UPDAT +1

V73UPDAT	CAF	UP73		# COMES HERE ON V73E
;
; Common entry logic for all uplink extended verbs. Saves verb identifier in
; UPVERBSV, checks if extended verb display system is available via TESTXACT,
; and validates that current major mode (MODREG) allows updates.
;
 	+1	TS	UPVERBSV	# SAVE UPVERB UNTIL IT'S OK TO ENTER P27

		TC	TESTXACT	# GRAB DISPLAY IF AVAILABLE, OTHERWISE
					# TURN*OPERATOR ERROR* ON AND TERMINATE JOB

;
; Major mode validation. Updates are only accepted during specific programs
; when crew workload is low and the computer isn't performing time-critical
; calculations. For safety, Mission Control cannot interrupt guidance programs.
;
		CA	MODREG		# CHECK IF UPDATE ALLOWED
		EXTEND			# FIRST CHECK FOR MODREG = +0, -0
		BZF	+2		# (+0 = P00, -0 = FRESH START)
		TC	CKMDMORE	# NOW CHECK FOR PROGRAM WHICH CAN BE
					# INTERRUPTED BY P27.
;
; Update is allowed. Save current major mode for restoration after update
; completes. Initialize update counter and transfer to main update logic in P27.
;
		CAE	MODREG		# UPDATE ALLOWED.
		TS	UPOLDMOD	# SAVE CURRENT MAJOR MODE
# Page 1500
		CAE	UPVERBSV	# SET UPVERB TO INDICDATE TO P27
		TS	UPVERB		# WHICH EXTENDED VERB CALLED IT.

		CAF	ONE
		TS	UPCOUNT		# INITIALIZE UPCOUNT TO 1

		TC	POSTJUMP	# LEAVE EXTENDED VERB BANK AND
		CADR	UPPART2		# GO TO UPDATE PROGRAM(P27) BANK.
;
; Additional mode checking for modes other than P00/fresh start. LM (LGC)
; accepts updates only during P00. CM (AGC) accepts updates during P00 or P02
; (CM/LM checkout). This safety interlock prevents ground updates from
; interfering with critical guidance phases like powered descent or entry.
;
CKMDMORE	CS	FLAGWRD5
		MASK	BIT8		# CHECK IF COMPUTER IS LGC
		CCS	A		# IS COMPUER LGC OR AGC
UPERLEM		TCF	UPERROR		# ERROR- IT'S THE LEM + MODE IS NOT P00.
		CS	TWO
		MASK	MODREG
		CCS	A
UPERCMC		TCF	UPERROR		# ERROR- IT'S THE CMC AND MODE IS NOT
					# P00 OR P02.
		TC	Q		# ALLOW UPDATE TO PROCEED
;
; Mode validation failed. Display operator error and exit without accepting
; update. Crew would need to terminate current program and enter P00 before
; Mission Control could retry the update.
;
UPERROR		TC	POSTJUMP	# TURN ON 'OPERATOR ERROR' LIGHT
		CADR	UPERROUT +2	# GO TO COMMON UPDATE PROGRAM EXIT

		SBANK=	LOWSUPER
UP70		EQUALS	ZERO
UP71		EQUALS	ONE
UP72		EQUALS	TWO
UP73		EQUALS	THREE

		BANK	04
		SETLOC	UPDATE2
		BANK

		COUNT*	$$/P27

; ============================================================================
; MAIN UPDATE PROGRAM ENTRY POINT (UPPART2)
;
; COMMENT-ONLY READERS: After verifying the spacecraft is in a safe mode
; (P00, P02, or Fresh Start), the uplink program begins the main update
; sequence. Mission Control can now send precise navigation corrections,
; time adjustments, or memory updates that keep the spacecraft on course.
;
; CODE-ALONG READERS: UPPART2 is the main P27 entry after mode validation.
; Sets restart protection (Group 6) to restore previous mode if interrupted,
; switches to major mode 27, and branches based on the update verb received.
; ============================================================================

UPPART2		EQUALS			# UPDATE PROGRAM - PART 2

; Establish restart protection for update sequence. If power transient or
; interrupt occurs during update, Group 6 restart will restore pre-P27 mode
; and exit gracefully rather than leaving spacecraft in partial update state.
		TC	PHASCHNG	# SET RESTART GROUP 6 TO RESTORE OLD MODE
		OCT	07026		# AND DOWNLIST AND EXIT IF RESTART OCCURS.
		OCT	30000		# PRIORITY SAME AS CHRPRIO
		EBANK=	UPBUFF
		2CADR	UPOUT +1

; Select Coast + Align downlink telemetry format during update processing
; to provide ground with spacecraft state information for verification.
		CAF	ONE
		TS	DNLSTCOD	# DOWNLIST

; Switch to major mode 27 to indicate update program active. Displayed
; on DSKY as "P27" informing crew that ground uplink is being processed.
		TC	NEWMODEX	# SET MAJOR MODE = 27
# Page 1501
		DEC	27

; ============================================================================
; VERB BRANCHING LOGIC
;
; COMMENT-ONLY READERS: Different types of updates require different amounts
; of data. Time updates (V70/V73) always need two numbers, while memory
; updates (V71/V72) can vary in size. The program branches to handle each type.
;
; CODE-ALONG READERS: Indexed branch on UPVERB (0-3 for V70-V73). Fixed-length
; verbs (V70, V73) immediately set COMPNUMB=2 and proceed. Variable-length
; verbs (V71, V72) jump to OHWELL1 to request component count from ground.
; ============================================================================

		INDEX	UPVERB		# BRANCH DEPENDING ON WHETHER THE UPDATE
		TCF	+1		# VERB REQUIRES A FIXED OR VARIABLE NUMBER
		TCF	+3		# V70 FIXED.		   (OF COMPONENTS.
		TCF	OHWELL1		# V71 VARIABLE - GO GET NO. OF COMPONENTS
		TCF	OHWELL1		# V72 VARIABLE - GO GET NO. OF COMPONENTS
		CA	TWO		# V73 (AND V70) FIXED
		TS	COMPNUMB	# SET NUMBER OF COMPONENTS TO 2.
		TCF	OHWELL2		# GO GET THE TWO UPDATE COMPONENTS

; ============================================================================
; COMPONENT COUNT REQUEST (OHWELL1)
;
; COMMENT-ONLY READERS: For memory updates (V71/V72), ground must first tell
; the AGC how many data words will follow. The DSKY displays V21N01 requesting
; this count. Mission Control enters the number of memory locations to update.
;
; CODE-ALONG READERS: Display flashing V21N01 to request component count (II)
; for variable-length updates. Count must be ≥3 and ≤20 decimal to prevent
; buffer overflow. Handles V32 (recycle) and V34 (terminate) responses.
; ============================================================================

OHWELL1		CAF	ADUPBUFF	# * REQUEST USER TO SEND NUMBER  *
		TS	MPAC +2		# * OF COMPONENTS PARAMETER(II). *
	+2	CAF	UPLOADNV	# (CKV432 RETURNS HERE IF V32 ENCOUNTERED)
		TC	BANKCALL	# DISPLAY A FLASHING V21N01
		CADR	GOXDSPF		# TO REQUEST II.
		TCF	UPOUT4		# V34 TERMINATE UPDATE(P27) RETURN
		TCF	OHWELL1 +2
		TC	CK4V32		# DATA OR V32 RETURN
		CS	BIT2		# VALIDATE COMPONENT COUNT:
		AD	UPBUFF		# IS II(NUMBER OF COMPONENTS PARAMETER)
		EXTEND			# .GE. 3 AND .LE. 20D.
		BZMF	OHWELL1 +2	# TOO SMALL - REQUEST AGAIN
		CS	UPBUFF
		AD	UP21
		EXTEND
		BZMF	OHWELL1 +2	# TOO LARGE - REQUEST AGAIN
		CAE	UPBUFF		# VALID COUNT RECEIVED
		TS	COMPNUMB	# SAVE II IN COMPNUMB

; ============================================================================
; COMPONENT RECEPTION AND BUFFERING (OHWELL2)
;
; COMMENT-ONLY READERS: The program now receives each data word transmitted
; from Mission Control. These might be new coordinates, time corrections, or
; other critical values. Each word is temporarily stored in a buffer while
; awaiting crew verification before being applied to the spacecraft's memory.
;
; CODE-ALONG READERS: Loop receiving COMPNUMB components via V21N02 display.
; Each component stored sequentially in UPBUFF. UPCOUNT tracks components
; received. Buffer address calculated as ADUPBFM1 + UPCOUNT (ECADR format).
; ============================================================================

# UPBUFF LOADING SEQUENCE

		INCR	UPCOUNT		# INCREMENT COUNT OF COMPONENTS RECEIVED.
OHWELL2		CAF	ADUPBFM1	# CALCULATE LOCATION(ECADR) IN UPBUFF
		AD	UPCOUNT		# WHERE NEXT COMPONENT SHOULD BE STORED.
	+2	TS	MPAC +2		# PLACE ECADR INTO R3.
	+3	CAF	UPLOADNV	# (CK4V32 RETURNS HERE IF V32 ENCOUNTERED)
		TC	BANKCALL	# DISPLAY A FLASHING V21N01
		CADR	GOXDSPF		# TO REQUEST DATA.
		TCF	UPOUT4		# V34 TERMINATE UPDATE(P27) RETURN.
		TCF	OHWELL2 +3	# V33 PROCEED RETURN
		TC	CK4V32		# DATA OR V32 RETURN
		CS	UPCOUNT		# HAVE WE FINISHED RECEIVING ALL
		AD	COMPNUMB	# THE DATA WE EXPECTED.
		EXTEND
		BZMF	UPVERIFY	# YES- GO TO VERIFICATION SEQUENCE
		TCF	OHWELL2 -1	# NO- REQUEST ADDITIONAL DATA.

# Page 1502
; ============================================================================
; VERIFICATION SEQUENCE (UPVERIFY)
;
; COMMENT-ONLY READERS: All update data has been received and buffered. Now
; comes a critical safety step: Mission Control reviews the received values
; displayed on DSKY. If any data was corrupted during transmission, ground
; can send corrections. Once verified correct, crew sends V33 (proceed) to
; commit the updates to spacecraft memory. This verification prevented many
; potential disasters during Apollo missions.
;
; CODE-ALONG READERS: Display V21N02 requesting component verification or
; correction. User can enter component index to modify, or V33 to proceed
; with storage. Component index validated against COMPNUMB bounds. UPTEMP
; holds component number to change if correction needed.
; ============================================================================

# VERIFY SEQUENCE
UPVERIFY	CAF	ADUPTEMP	# PLACE ECADR WHERE COMPONENT NO. INDEX
		TS	MPAC +2		# IS TO BE STORED INTO R3.
		CAF	UPVRFYNV	# (CK4V32 RETURNS HERE IF V32 ENCOUNTERED)
		TC	BANKCALL	# DISPLAY A FLASHING V21N02 TO REQUEST
		CADR	GOXDSPF		# DATA CORRECTION OR VERIFICATION.
		TCF	UPOUT4		# V34 TERMINATE UPDATE(P27) RETURN
		TCF	UPSTORE		# V33 DATA SENT IS GOOD.  GO STORE IT.
		TC	CK4V32		# COMPONENT NO. INDEX OR V32 RETURN

; Data correction loop - If ground detects transmission error in buffered data,
; they enter component number to correct. Validates index is positive nonzero
; and within COMPNUMB range before jumping back to component input sequence.
		CA	UPTEMP		# DOES THE COMPONENT NO. INDEX JUST SENT
		EXTEND			# SPECIFY A LEGAL COMPONENT NUMBER?
		BZMF	UPVERIFY	# NO, IT IS NOT POSITIVE NONZERO
		CS	UPTEMP
		AD	COMPNUMB
		AD	BIT1
		EXTEND
		BZMF	UPVERIFY	# NO
		CAF	ADUPBFM1	# YES- BASED ON THE COMPONENT NO. INDEX
		AD	UPTEMP		# CALCULATE THE ECADR OF LOCATION IN
		TCF	OHWELL2 +2	# UPBUFF WHICH USER WANTS TO CHANGE.

UPOUT4		EQUALS	UPOUT +1	# COMES HERE ON V34 TC TERMINATE UPDATE

; ============================================================================
; V32 RECYCLE CHECK SUBROUTINE (CK4V32)
;
; COMMENT-ONLY READERS: When Mission Control presses ENTR after entering data,
; the program must distinguish between actual data and the V32 "recycle" verb.
; If V32 was entered instead of data, the program loops back to re-request.
;
; CODE-ALONG READERS: Tests MPAC bit 6 to detect V32 (octal 40). Called after
; GOXDSPF returns. If V32 detected, indexed return to caller -6 instructions
; (back to display request). If data, normal return via Q register.
; ============================================================================

# CHECK FOR VERB 32 SEQUENCE

CK4V32		CS	MPAC		# ON DATA RETURN FROM 'GOXDSPF'
		MASK	BIT6		# ON DATA RETURN FROM "GOXDSP"& THE CON-
		CCS	A		# TENTS OF MPAC = VERB.  SO TEST FOR V32.
		TC	Q		# IT'S NOT A V32, IT'S DATA.  PROCEED.
		INDEX	Q
		TC	0 -6		# V32 ENCOUNTERED - GO BACK AND GET DATA

; Constants and addresses for update data reception and verification
ADUPTEMP	ADRES	UPTEMP		# ADDRESS OF TEMP STORAGE FOR CORRECTIONS
ADUPBUFF	ADRES	UPBUFF		# ADDRESS OF UPDATE DATA STORAGE BUFFER
UPLOADNV	VN	2101		# VERB 21 NOUN 01
UPVRFYNV	VN	2102		# VERB 21 NOUN 02
UP21		=	MD1		# DEC 21 = MAX NO OF COMPONENTS +1
UPDTPHAS	EQUALS	FIVE

; ============================================================================
; DATA STORAGE PHASE (UPSTORE)
;
; TRANSITION: From verification cycle to data installation
;
; Mission Control has sent V33E verifying the buffered update data. Program
; now inverts VERIFLAG to acknowledge verification, sets restart protection,
; and branches to appropriate storage handler (V73 clock update directly, or
; FINDVAC for V70-V72 requiring state vector integration via INTSTALL).
;
; CODE-ALONG READERS: VERIFLAG inversion (bit 3 of FLAGWRD7) telemetered to
; ground confirming V33 receipt. Restart group 6 protects data installation.
; UPVERB determines branch: 0-2 (V70-V72) → UPFNDVAC, 3 (V73) → UPEND73.
; ============================================================================

# PRE-STORE AND FAN TO APPROPRIATE BRANCH SEQUENCE

UPSTORE		EQUALS			# GROUND HAS VERIFIED UPDATE. STORE DATA.

		INHINT

		CAE	FLAGWRD7	# INVERT VERIFLAG(BIT3 OF FLAGWRD7) TO
		XCH	L		# INDICATE TO THE GROUND(VIA DOWNLINK)
		CAF	BIT3		# THAT THE V33(WHICH THE GROUND SENT TO
# Page 1503
		EXTEND			# VERIFY THE UPDATE) HAS BEEN SUCCESSFULLY
		RXOR	LCHAN		# RECEIVED BY THE UPDATE PROGRAM
		TS	FLAGWRD7

		TC	PHASCHNG	# SET RESTART GROUP 6 TO REDO THE UPDATE
		OCT	04026		# DATA STORE IF A RESTART OCCURS.
		INHINT			# (BECAUSE PHASCHNG DID A RELINT)

		CS	TWO		# GO TO UPFNDVAC IF INSTALL IS REQUIRED,
		AD	UPVERB		# THAT IS, IF IT'S A V70 - V72.
		EXTEND			# GO TO UPEND73 IF IT'S A V73.
		BZMF	UPFNDVAC

; ============================================================================
; V73 CLOCK INCREMENT BRANCH (UPEND73)
;
; COMMENT-ONLY READERS: V73 adds time correction directly to spacecraft clock.
; Used when Mission Control detects clock drift, sending delta-time to adjust
; AGC's time base (TIME2, TIME1) maintaining synchronization with ground time.
;
; CODE-ALONG READERS: Extracts DP time increment from UPBUFF, calls TIMEDIDL
; to add increment to AGC clock. Error return (invalid time) lights operator
; error, normal return proceeds to UPOUT cleanup. Scaling: 2^28 centiseconds.
; ============================================================================

# VERB 73 BRANCH

UPEND73		EXTEND			# V73-PERFORM DP OCTAL AGC CLOCK INCREMENT
		DCA	UPBUFF
		DXCH	UPBUFF +8D
		TC	TIMEDIDL
		TC	FALTON		# ERROR- TURN ON *OPERATOR ERROR* LIGHT
		TC	UPOUT +1	# GO TO COMMON UPDATE PROGRAM EXIT

; ============================================================================
; FINDVAC SEQUENCE FOR V70-V72 (UPFNDVAC)
;
; V70, V71, V72 require integration with navigation state via INTSTALL. These
; updates must execute as VAC area jobs for proper interaction with ORBITAL
; INTEGRATION and state vector updates. Allocates VAC area, schedules UPJOB.
; ============================================================================

UPFNDVAC	CAF	CHRPRIO		# (USE EXTENDED VERB PRIORITY)
		TC	FINDVAC		# GET VAC AREA FOR 'CALL INTSTALL'
		EBANK=	TEPHEM
		2CADR	UPJOB		# (NOTE:  THIS WILL ALSO SET EBANK FOR
		TC	ENDOFJOB	# 'TEPHEM' UPDATE BY V70)

; ============================================================================
; UPDATE JOB INTEGRATION COORDINATION (UPJOB/UPWAKE)
;
; COMMENT-ONLY READERS: Before updating navigation state vectors, program must
; coordinate with ORBITAL INTEGRATION. If orbit calculations in progress, this
; job sleeps until integration completes. Critical for V70 (liftoff time shift
; affecting state vectors) and ensuring navigation data consistency.
;
; CODE-ALONG READERS: INTPRET call to INTSTALL handles integration lock. Job
; sleeps if ORBITAL INTEGRATION active, preventing state vector corruption.
; UPWAKE continues after INTSTALL returns. Restart group 6 protection ensures
; atomic state vector updates. REINTFLG set signaling integration restart needed.
; ============================================================================

UPJOB		TC	INTPRET		# THIS COULD BE A STATE VECTOR UPDATE--SO
		CALL			# WAIT(PUT JOB TO SLEEP) IF ORBIT INT(OI)
			INTSTALL	# IS IN PROGRESS--OR--GRAB OI AND RETURN
					# TO UPWAKE IF OI IS NOT IN PROGRESS.

UPWAKE		EXIT

		TC	PHASCHNG	# RESTART PROTECT(GROUP 6)
		OCT	04026

		TC	UPFLAG		# SET INTEGRATION RESTART BIT
		ADRES	REINTFLG
		INHINT

; ============================================================================
; TRANSITION: From job scheduling to verb-specific update execution
;
; With the orbital integration lock released and update data verified, the
; program now branches to verb-specific handlers. Each verb (V70/V71/V72)
; implements distinct update operations: time adjustments, contiguous memory
; blocks, or scatter writes to multiple addresses. Mission Control selected
; the appropriate verb based on what needed correction in the spacecraft's
; navigation state.
; ============================================================================

UPPART3		EQUALS

; ============================================================================
; VERB-SPECIFIC UPDATE BRANCHING
;
; COMMENT-ONLY READERS: After verification, the program executes the actual
; update operation. Different verbs perform different types of updates:
; - V70: Adjusts all mission timers (clock, ephemeris, state vector times)
; - V71: Updates a continuous block of memory locations
; - V72: Updates scattered individual memory locations
;
; CODE-ALONG READERS: Indexed branch based on UPVERB (0/1/2 for V70/V71/V72).
; Each handler implements its specific update algorithm and provides restart
; protection during the critical update phase. INHINT active during branching.
; ============================================================================

		INDEX	UPVERB		# BRANCH TO THE APPROPRIATE UPDATE VERB
		TCF	+1		# ROUTINE TO ACTUALLY PERFORM THE UPDATE
		TCF	UPEND70		# V70
		TCF	UPEND71		# V71
		TCF	UPEND72		# V72

# Page 1504
# ROUTINE TO INCREMENT CLOCK(TIME2,TIME1) WITH CONTENTS OF DP WORD AT UPBUFF.

; ============================================================================
; TIME INCREMENT/DECREMENT ROUTINE (TIMEDIDL)
;
; COMMENT-ONLY READERS: This routine performs the actual clock arithmetic,
; carefully adding or subtracting time while the spacecraft continues flying.
; If an error occurs during the delicate clock manipulation, the routine
; detects it and aborts the update safely.
;
; CODE-ALONG READERS: Accepts DP time increment in UPBUFF+8D. Saves TIME2/TIME1
; to UPBUFF+18D for rollback. Uses TPAGREE to force sign agreement. Tests for
; overflow via BZF. TC UPTEMP (return address) for normal/error exit. Critical
; restart-protected section with PHASCHNG group 6.
; ============================================================================

; ============================================================================
; TIMEDIDL CLOCK UPDATE IMPLEMENTATION
;
; COMMENT-ONLY READERS: The routine saves the current clock value, then carefully
; adds the time increment from Mission Control. If the math causes an overflow
; (the number becomes too large to represent), it restores the old clock value
; and reports an error. The entire operation is protected against computer
; restarts so a power glitch won't leave the clock corrupted.
;
; CODE-ALONG READERS: Critical restart-protected clock manipulation:
; 1. Save return address (Q) to UPTEMP
; 2. Zero and save TIME2/TIME1 to UPBUFF+18D (rollback buffer)
; 3. Set PHASCHNG restart protection (group 6, phase UPDTPHAS)
; 4. TIMEDIDR restart entry: Add increment from UPBUFF+8D to saved time
; 5. BZF overflow check: If overflow, restore old time and error exit
; 6. DELTATOK: Call TPAGREE for sign agreement, apply to TIME2/TIME1
; 7. Return TC UPTEMP (normal) or TC UPTEMP (error on overflow)
; All operations INHINT to prevent interrupt corruption of clock state.
; ============================================================================

TIMEDIDL	EXTEND
		QXCH	UPTEMP		# SAVE Q FOR RETURN
		CAF	ZERO		# ZERO AND SAVE TIME2,TIME1
		ZL
		DXCH	TIME2
		DXCH	UPBUFF +18D	# STORE IN CASE OF OVERFLOW

		CAF	UPDTPHAS	# DO
		TS	L		# A
		COM			# QUICK
		DXCH	-PHASE6		# PHASCHNG

; Restart entry point: If power fails during clock update, restart returns here
; to re-attempt the operation with saved data in UPBUFF.

TIMEDIDR	INHINT
		CAF	ZERO
		ZL			# PICK UP INCREMENTER(AND ZERO
		TS	MPAC +2		# IT IN CASE OF RESTARTS) AND
		DXCH	UPBUFF +8D	# STORE IT
		DXCH	MPAC		# INTO MPAC FOR TPAGREE.

		EXTEND
		DCA	UPBUFF +18D
		DAS	MPAC		# FORM SUM IN MPAC
		EXTEND
		BZF	DELTATOK	# TEST FOR OVERFLOW
		CAF	ZERO
		DXCH	UPBUFF +18D	# OVERFLOW, RESTORE OLD VALUE OF CLOCK
		DAS	TIME2		# AND TURN ON OPERATOR ERROR

		TC	PHASCHNG	# RESTART PROTECT(GROUP 6)
		OCT	04026

		TC	UPTEMP		# GO TO ERROR EXIT

; Clock increment is valid - apply it to the master clock

DELTATOK	TC	TPAGREE		# FORCE SIGN AGREEMENT
		DXCH	MPAC
		DAS	TIME2		# INCREMENT TIME2,TIME1

		TC	PHASCHNG	# RESTART PROTECT(GROUP 6)
		OCT	04026

		INHINT
		INDEX	UPTEMP		# (CODED THIS WAY FOR RESTART PROTECTION)
		TC	1		# NORMAL RETURN

# VERB 71 BRANCH

; ============================================================================
; VERB 71: CONTIGUOUS BLOCK UPDATE (UPEND71)
;
; COMMENT-ONLY READERS: This update type writes a continuous series of values
; to consecutive memory locations. Mission Control used this when multiple
; related navigation values needed updating together (like all components of
; a position vector). The program checks that the update won't accidentally
; overflow into the next memory bank, preventing corruption.
;
; CODE-ALONG READERS: Implements contiguous block write from UPBUFF+2 onwards
; to erasable memory starting at address in UPBUFF+1. Sets EBANK from high bits
; of ECADR. Validates that (base_address + word_count - 3) doesn't cross bank
; boundary (checks bit 9 of result). Loops STORLP71 decrementing MPAC counter,
; using indexed LXCH to swap L register with target locations. COMPNUMB-3 gives
; word count (minus header words).
; ============================================================================

UPEND71		CAE	UPBUFF +1	# SET EBANK
		TS	EBANK		#	AND
# Page 1505
		MASK	LOW8		# CALCULATE
		TS	UPTEMP		# S-REG VALUE OF RECEIVING AREA

		AD	NEG3		# IN THE PROCESS OF
		AD	COMPNUMB	# PERFORMING
		EXTEND			# THIS UPDATE
		BZF	STORLP71	# WILL WE
		MASK	BIT9		# OVERFLOW
		CCS	A		# INTO THE NEXT EBANK....
		TCF	UPERROUT	# YES

; Bank boundary check passed - proceed with contiguous update loop

		CA	NEG3		# NO- CALCULATE NUMBER OF
		AD	COMPNUMB	# WORDS TO BE STORED MINUS ONE
STORLP71	TS	MPAC		# SAVE NO. OF WORDS REMAINING MINUS ONE
		INDEX	A		# TAKE NEXT UPDATE WORD FROM
		CA	UPBUFF +2	# UPBUFF AND
		TS	L		# SAVE IT IN L
		CA	MPAC		# CALCULATE NEXT
		AD	UPTEMP		# RECEIVING ADDRESS
		INDEX	A
		EBANK=	1400
		LXCH	1400		# UPDATE THE REGISTER  BY CONTENTS OF L
		EBANK=	TEPHEM
		CCS	MPAC		# ARE THERE ANY WORDS LEFT TO BE STORED
		TCF	STORLP71	# YES
		TCF	UPOUT		# NO- THEN EXIT UPDATE PROGRAM
ADUPBFM1	ADRES	UPBUFF -1	# SAME AS ADUPBUFF BUT LESS 1 (DON'T MOVE)
		TCF	UPOUT		# NO- EXIT UPDATE(HERE WHEN COMPNUMB = 3)

# VERB 72 BRANCH

; ============================================================================
; VERB 72: SCATTER UPDATE (UPEND72)
;
; COMMENT-ONLY READERS: This update type writes values to scattered, non-
; consecutive memory locations. Mission Control used this when updating several
; unrelated parameters in a single uplink transmission (more efficient than
; separate commands). Each value has its own address specified. The format
; requires an odd number of words: one count word followed by address/data pairs.
;
; CODE-ALONG READERS: Implements scatter write using alternating ECADR/data
; pairs in UPBUFF. Validates COMPNUMB is odd (BIT1 mask). Loop counter set to
; COMPNUMB-2 (skipping count word, processing pairs). LDLOOP72 interleaves:
; - Pick up data word from UPBUFF+1 indexed, save to L via LXCH
; - Decrement pointer (CCS MPAC)
; - Pick up ECADR from UPBUFF+1 indexed, set EBANK
; - Extract relative address (LOW8 mask)
; - Store data via indexed LXCH 1400 (swaps L with target address)
; Continues until MPAC exhausted. Typo in original: "THORUGH" should be "THROUGH".
; ============================================================================

UPEND72		CAF	BIT1		# HAVE AN ODD NO. OF COMPONENTS
		MASK	COMPNUMB	# BEEN SENT FOR A V72 UPDATE...
		CCS	A
		TCF	+2		# YES
		TCF	UPERROUT	# ERROR- SHOULD BE ODD NO. OF COMPONENTS
		CS	BIT2
		AD	COMPNUMB
LDLOOP72	TS	MPAC		# NOW PERFORM THE UPDATE
		INDEX	A
		CAE	UPBUFF +1	# PICK UP NEXT UPDATE WORD
		LXCH	A
		CCS	MPAC		# SET POINTER TO ECADR(MUST BE CCS)
		TS	MPAC
		INDEX	A
		CAE	UPBUFF +1	# PICK UP NEXT ECADR OF REG TO BE UPDATED
		TS	EBANK		# SET EBANK
		MASK	LOW8		# ISOLATE RELATIVE ADDRESS
		INDEX	A

# Page 1506
		EBANK=	1400
		LXCH	1400		# UPDATE THE REGISTER BY CONTENTS OF L
		EBANK=	TEPHEM
		CCS	MPAC		# ARE WE THORUGH THE V72 UPDATE...
		TCF	LDLOOP72	# NO

# NORMAL FINISH OF P27

; ============================================================================
; NORMAL EXIT: UPOUT
;
; COMMENT-ONLY READERS: When the ground-commanded update completes successfully,
; this routine restores the spacecraft to normal operations. It releases the
; orbital integration computation that was paused during the update, restores
; the previous operating mode (the program that was running before P27 update),
; re-enables the normal telemetry downlink to Mission Control, turns off the
; uplink activity light on the crew's display, and exits the extended verb
; processing. The crew and ground see the update complete smoothly.
;
; CODE-ALONG READERS: Normal completion cleanup sequence executing multiple
; housekeeping operations:
; 1. INTWAKEU releases orbital integration "grab" (navigation resumed)
; 2. UPOLDMOD restored via NEWMODEX+3 (return to pre-P27 major mode)
; 3. DNLSTCOD zeroed to re-enable coast+align downlist
; 4. UPACTOFF called to extinguish uplink activity indicator light
; 5. Group 6 restart protection killed via EXTEND DCA NEG0 to -PHASE6
; 6. ENDEXT performs extended verb exit procedures
; Note: UPOUT uses EQUALS label (entry point synonym for INTWAKEU call).
; ============================================================================

UPOUT		EQUALS
		TC	INTWAKEU	# RELEASE  GRAB  OF ORBITAL INTEGRATION
	+1	CAE	UPOLDMOD	# RESTORE PRIOR P27 MODE
		TC	NEWMODEX +3
		CAF	ZERO
		TS	DNLSTCOD
		TC	UPACTOFF	# TURN OFF 'UPLINK ACTIVITY' LIGHT

		EXTEND			# KILL GROUP 6
		DCA	NEG0
		DXCH	-PHASE6

		TC	ENDEXT		# EXTENDED VERB EXIT

# VERB 70 BRANCH

; ============================================================================
; VERB 70: LIFTOFF TIME INCREMENT (UPEND70)
;
; COMMENT-ONLY READERS: This update adjusts multiple mission clocks by a single
; time increment sent from Mission Control. Used for launch time corrections or
; mission timeline adjustments. The ground sends one time value, and the AGC
; applies it to the onboard clock, both spacecraft state vector times (CSM and
; LM), and the ephemeris time reference. This keeps all mission clocks
; synchronized when timeline changes occur.
;
; CODE-ALONG READERS: Implements comprehensive time adjustment with restart
; protection. Sequence:
; 1. Copy DP time increment from UPBUFF to UPBUFF+8D for TIMEDIDL processing
; 2. TIMEDIDL decrements AGC clock (TIME2,TIME1), checking for underflow
; 3. If TIMEDIDL error, exit via UPERROUT
; 4. Copy time increment to UPBUFF+10D and UPBUFF+12D for restart protection
; 5. PHASCHNG restart protection (Group 6)
; 6. Decrement TETCSM (CSM state vector time) via DAS with zero extension
; 7. Decrement TETLEM (LM state vector time) via DAS with zero extension
; 8. Increment TEPHEM (ephemeris time) via DAS TEPHEM+1 and ADS TEPHEM
; 9. Second PHASCHNG restart protection
; 10. Exit via UPOUT standard cleanup
; Time increment scaled at 2^28 centiseconds (per program comments page 1497).
; ============================================================================

UPEND70		EXTEND			# V70 DOES THE FOLLOWING WITH DP DELTA
		DCS	UPBUFF		# TIME IN UPBUFF
		DXCH	UPBUFF +8D
		TC	TIMEDIDL	# DECREMENT AGC CLOCK

		TC	UPERROUT	# ERROR WHILE DECREMENTING CLOCK -- EXIT

		EBANK=	TEPHEM
		EXTEND
		DCS	UPBUFF		# COPY DECREMENTERS FOR
		DXCH	UPBUFF +10D	# RESTART PROTECTION
		EXTEND
		DCS	UPBUFF
		DXCH	UPBUFF +12D

		TC	PHASCHNG	# RESTART PROTECT(GROUP 6)
		OCT	04026

		CAF	ZERO
		ZL
		DXCH	UPBUFF +10D	# DECREMENT CSM STATE VECTOR TIME
		DAS	TETCSM

		CAF	ZERO
# Page 1507
		ZL
		DXCH	UPBUFF +12D	# DECREMENT LEM STATE VECTOR TIME
		DAS	TETLEM

		CAF	ZERO
		ZL
		DXCH	UPBUFF
		DAS	TEPHEM +1	# INCREMENT TP TEPHEM
		ADS	TEPHEM

		TC	PHASCHNG	# RESTART PROTECT(GROUP 6)
		OCT 	04026

		EBANK=	UPBUFF

		TC	UPOUT		# GO TO STANDARD UPDATE PROGRAM EXIT

# ERROR SEQUENCE

; ============================================================================
; ERROR EXIT: UPERROUT
;
; COMMENT-ONLY READERS: When Mission Control sends an uplink command with
; incorrect data format or invalid parameters, this routine activates the
; operator error light on the crew's display panel to alert them and Houston
; that the uplink failed. The update is aborted and the spacecraft returns
; to normal operations. Astronauts would see the error light, report it to
; Mission Control, and await a corrected uplink transmission.
;
; CODE-ALONG READERS: Error handler with two entry points providing different
; cleanup strategies:
; 1. UPERROUT entry: Standard error exit calling FALTON to illuminate operator
;    error light (displayed to crew), then TCF to UPOUT for full cleanup
;    (restore mode, reset downlink, kill restart protection, exit verb).
; 2. UPERROUT+2 entry: Alternate error exit for cases where restart groups
;    must NOT be reset. Calls FALTON for error light, UPACTOFF to extinguish
;    uplink activity light, then ENDEXT for minimal verb exit. Preserves
;    restart protection state (useful during certain restart recovery scenarios).
; Choice of entry point depends on calling context and restart protection needs.
; ============================================================================

UPERROUT	TC	FALTON		# TURN ON *OPERATOR ERROR* LIGHT
		TCF	UPOUT		# GO TO COMMON UPDATE PROGRAM EXIT

	+2	TC	FALTON		# TURN ON 'OPERATOR ERROR' LIGHT
		TC	UPACTOFF	# TURN OFF'UPLINK ACTIVITY'LIGHT
		TC	ENDEXT		# EXTENDED VERB EXIT
					# (THE PURPOSE OF UPERROUT +2 EXIT IS
					# TO PROVIDE AN ERROR EXIT WHICH DOES NOT
					# RESET ANY RESTART GROUPS)

# :UPACTOFF: IS A ROUTINE TO TURN OFF UPLINK ACTIVITY LIGHT ON ALL EXITS FROM UPDATE PROGRAM(P27).

; ============================================================================
; UTILITY: UPACTOFF - Uplink Activity Light Control
;
; COMMENT-ONLY READERS: This small routine extinguishes the uplink activity
; light on the crew's display panel. When the light is on, astronauts know
; Mission Control is transmitting commands to the spacecraft. When it goes off,
; the uplink transmission is complete (either successfully or with error).
; This provides crew situational awareness of ground communications.
;
; CODE-ALONG READERS: Clears bit 3 of channel 11 (DSALMOUT) to extinguish
; uplink activity indicator light. Implementation:
; - CS BIT3: Complement BIT3 constant (inverts all bits including bit 3)
; - EXTEND: Enables next instruction as extended instruction
; - WAND DSALMOUT: Write AND to channel 11 (clears bit 3, preserves others)
; - TC Q: Return to caller via Q register
; Channel 11 (octal 13) is output channel for discrete alarms and lights.
; Bit 3 specifically controls uplink activity indicator visible to crew.
; Used by all UPDATE_PROGRAM exit paths to signal uplink completion.
; ============================================================================

UPACTOFF	CS	BIT3
		EXTEND			# TURN OFF UPLINK ACTIVITY LIGHT
		WAND	DSALMOUT	# (BIT 3 OF CHANNEL 11)
		TC	Q
