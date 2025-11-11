# Copyright:	Public domain.
# Filename:	UPDATE_PROGRAM.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1386-1396
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Luminary131/ file of the same
#				name, using Luminary099 page images.
#		2009-06-07 RSB	Added an SBANK= to account for incompatibilities
#				between YUL and yaYUL.
#		2011-01-06 JL	Fixed pseudo-label indentation.
#		2011-05-07 JL	Flag SBANK= workaround.

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
; FILE: UPDATE_PROGRAM.agc
; MODULE: Ground Uplink Interface
; MISSION PHASE: All mission phases (launch/earth-orbit/trans-lunar/lunar-orbit/descent/landing/ascent/rendezvous/trans-earth)
;
; TL;DR: Implements P27 UPDATE PROGRAM which processes commands and data
;        insertions transmitted from Mission Control via the ground uplink
;        system. Accepts four types of updates through extended verbs V70-V73:
;        time corrections to mission clocks and state vectors, contiguous and
;        non-contiguous memory updates, and navigation state corrections.
;        Critical for mission flexibility enabling ground controllers to
;        update spacecraft computer with refined trajectory data, timing
;        corrections, and operational parameter changes throughout mission.
;
; COMMENT-ONLY READERS: This program is the communication bridge between
;        Mission Control in Houston and the Lunar Module's guidance computer.
;        Read comments to understand how ground controllers transmitted
;        navigation updates and time corrections to Armstrong and Aldrin.
;
; CODE-ALONG READERS: Study the uplink protocol, data verification sequence,
;        and restart protection mechanisms. Note the two-phase update process:
;        data reception with crew verification, followed by protected storage
;        after V33E confirmation. Examine memory buffer management (UPBUFF)
;        and the different update modes (V70-V73).
; ============================================================================

# Page 1386
# PROGRAM NAME:     P27
# WRITTEN BY:       KILROY/ DE WOLF
#
# MOD NO:           6
# MOD BY:           KILROY
# DATE:             01DEC67
#
# LOG SECTION:      UPDATE PROGRAM.
#
# FUNCT. DESCR:     P27 (THE UPDATE PROGRAM) PROCESSES COMMANDS AND DATA
#                       INSERTIONS REQUESTED BY THE GROUND VIA UPLINK.
#                       THE P27 PROGRAM WILL ACCEPT UPDATES
#                       ONLY DURING P00 FOR THE LM, AND ONLY DURING P00,
#                   P02, AND FRESH START FOR THE CSM
#
# CALLING SEQ:      PROGRAM IS INITIATED BY UPLINK ENTRY OF VERBS 70, 71, 72 AND 73.
#
# SUBROUTINES:      TESTXACT, NEWMODEX, NEWMODEX +3, GOXDSPF, BANKCALL, FINDVAC, INTPRET, INTSTALL, TPAGREE,
#                   INTWAKEU, ENDEXT, POSTJUMP, FALTON, NEWPHASE, PHASCHNG
#
# NORMAL EXIT:      TC ENDEXT
#
# ALARM/ABORT:      TC FALTON FOLLOWED BY TC ENDEXT
#
# RESTARTS:         P27 IS RESTART PROTECTED IN TWO WAYS...
#                   1. PRIOR TO VERIFLAG INVERSION(WHICH IS CAUSED BY THE GROUND/ASTRONAUT'S VERIFICATION OF UPDATE
#                      DATA BY SENDING A V33E WHEN V21N02 IS FLASHING)---
#                      NO PROTECTION EXCEPT PRE-P27 MODE IS RESTORED, COAST + ALIGN DOWNLIST IS SELECTED AND UPLINK
#                      ACTIVITY LIGHT IS TURNED OFF.(JUST AS IF A V34E WAS SENT DURING P27 DATA LOADS).
#                      V70,V71,V72 OR V73 WILL HAVE TO BE COMPLETELY RESENT BY USER.
#                   2. AFTER VERIFLAG INVERSION(WHEN UPDATE OF THE SPECIFIED ERASABLES IS BEING PERFORMED)---
#                      PROTECTED AGAINST RESTARTS.
#
# DEBRIS:           UPBUFF   (20D)  TEMP STORAGE FOR ADDRESSES AND CONTENTS.
#                   UPVERB   (1)    VERB NUMBER MINUS 70D (E.G., FOR V72, UPVERB = 72D - 70D = 2)
#                   UPOLDMOD (1)    FOR MAJOR MODE INTERRUPTED BY P27.
#                   COMPNUMB (1)    TOTAL NUMBER OF COMPONENTS TO BE TRANSMITTED.
#                   UPCOUNT  (1)    ACTUAL NUMBER OF COMPONENTS RECEIVED.
#                   UPTEMP   (1)    SCRATCH, BUT USUALLY CONTAINS COMPONENT NUMBER TO BE CHANGED DURING VERIFY CYCLE
#
# INPUT:
#
#  ENTRY:             DESCRIPTION
#
#  V70EXXXXXEXXXXXE   (LIFTOFF TIME INCREMENT) DOUBLE PRECISION OCTAL TIME INCREMENT, XXXXX XXXXX,
#                     IS ADDED TO TEPHEM, SUBTRACTED FROM AGC CLOCK(TIME2,TIME1), SUBTRACTED FROM CSM STATE
#                     VECTOR TIME(TETCSM) AND SUBTRACTED FROM LEM STATE VECTOR TIME(TETLEM).
#                     THE DP OCTAL TIME INCREMENT IS SCALED AT 2(28).
# Page 1387
#  V71EIIEAAAAE     (CONTIGUOUS BLOCK UPDATE) II-2 OCTAL COMPONENTS,XXXXX,
#  XXXXXE           ARE LOADED INTO ERASABLE STARTING AT ECADR, AAAA.
# XXXXXE            IT IS .GE. 3 .AND. .LE. 200.,
#                   AND (AAAA + II - 3) DOES NOT PRODUCE AN ADDRESS IN THE
# 9 NEXT BANK.
#   .               SCALING IS SAME AS INTERNAL REGISTERS.
#
#  V72EIIE          (SCATTER UPDATE) (II-1)/2 OCTAL COMPONENTS,XXXXX, ARE
#  AAAAEXXXXXE      LOADED INTO ERASABLE LOCATIONS, AAAA.
#  AAAAEXXXXXE      II IS .GE. 3 .AND. .LE. 19D, AND MUST BE ODD.
#   .               SCALING IS SAME AS INTERNAL REGISTERS.
#
#  V73EXXXXXEXXXXXE (OCTAL CLOCK INCREMENT) DOUBLE PRECISION OCTAL TIME
#                   INCREMENT XXXXX XXXXX, IS ADDED TO THE AGC CLOCK, IN
#                   CENTISECONDS SCALED AT (2)28.
#                   THIS LOAD IS THE OCTAL EQUIVALENT OF V55.
#
# OUTPUT:         IN ADDITION TO THE ABOVE REGISTER LOADS, ALL UPDATES
#                 COMPLEMENT BIT3 OF FLAGWORD7.
#
# ADDITIONAL NOTES: VERB 71, JUST DEFINED ABOVE WILL BE USED TO PERFORM BUT NOT LIMITED TO THE FOLLOWING UPDATES--
#
#                 1. CSM/LM STATE VECTOR UPDATE
#                 2. REFSMMAT UPDATE
#
#
#          THE FOLLOWING COMMENTS DELINEATE EACH SPECIAL UPDATE----
#
# 1. CSM/LM STATE VECTOR UPDATE(ALL DATA ENTRIES IN OCTAL)
#
# ENTRIES:        DATA DEFINITION:                                        SCALE FACTORS:
# V71E            CONTIGUOUS BLOCK UPDATE VERB
#    21E          NUMBER OF COMPONENTS FOR STATE VECTOR UPDATE
#  AAAAE          ECADR OF 'UPSVFLAG'
# XXXXXE          STATE VECTOR IDENTIFIER: 00001 FOR CSM, 77776 FOR LEM - EARTH SPHERE OF INFLUENCE SCALING
#                                          00002 FOR CSM, 77775 FOR LEM - LUNAR SPHERE OF INFLUENCE SCALING
#
# XXXXXEXXXXXE    X POSITION
# XXXXXEXXXXXE    Y POSITION
# XXXXXEXXXXXE    Z POSITION
# XXXXXEXXXXXE    X VELOCITY
# XXXXXEXXXXXE    Y VELOCITY
# XXXXXEXXXXXE    Z VELOCITY
# XXXXXEXXXXXE    TIME FROM AGC CLOCK ZERO
# V33E            VERB 33 TO SIGNAL THAT THE STATE VECTOR IS READY TO BE STORED.
#
#
# 2. REFSMMAT(ALL DATA ENTRIES IN OCTAL)
# ENTRIES:        DATA DEFINITITIONS:                                     SCALE FACTORS:
# Page 1388
# V71E            CONTIGUOUS BLOCK UPDATE VERB
#    24E          NUMBER OF COMPONENTS FOR REFSMMAT UPDATE
#  AAAAE          ECADR OF 'REFSMMAT'
# XXXXXEXXXXXE    ROW 1 COLUMN 1                                          2(-1)
# XXXXXEXXXXXE    ROW 1 COLUMN 2                                          2(-1)
# XXXXXEXXXXXE    ROW 1 COLUMN 3                                          2(-1)
# XXXXXEXXXXXE    ROW 2 COLUMN 1                                          2(-1)
# XXXXXEXXXXXE    ROW 2 COLUMN 2                                          2(-1)
# XXXXXEXXXXXE    ROW 2 COLUMN 3                                          2(-1)
# XXXXXEXXXXXE    ROW 3 COLUMN 1                                          2(-1)
# XXXXXEXXXXXE    ROW 3 COLUMN 2                                          2(-1)
# XXXXXEXXXXXE    ROW 3 COLUMN 3                                          2(-1)
# V33E            VERB 33 TO SIGNAL THAT REFSMMAT IS READY TO BE STORED.

; ============================================================================
; TRANSITION: Ground Uplink Extended Verb Entry Points
;
; Mission Control in Houston could transmit four types of updates to the
; Lunar Module's guidance computer during the Apollo 11 mission. Each update
; type was initiated by the crew entering an extended verb (V70, V71, V72,
; or V73) on the DSKY after ground controllers sent the command via uplink.
; This section provides the entry points for all four update modes, routing
; each to the main P27 UPDATE PROGRAM logic after validation checks.
; ============================================================================

		BANK	07
		SETLOC	EXTVERBS
		BANK

		EBANK=	TEPHEM

		COUNT*	$$/P27
; V70: Clock decrement and state vector time adjustment
; This verb adjusts the AGC mission clock backward and updates spacecraft
; state vector times to account for launch delays or mission timeline changes.
V70UPDAT	CAF	UP70		# COMES HERE ON V70E
		TCF	V73UPDAT +1

; V71: Contiguous block memory update
; Used primarily for state vector and REFSMMAT (reference stable member matrix)
; updates where multiple consecutive memory locations need to be loaded.
V71UPDAT	CAF	UP71		# COMES HERE ON V71E
		TCF	V73UPDAT +1

; V72: Non-contiguous (scatter) memory update
; Loads individual address/data pairs to non-consecutive memory locations.
; Useful for updating specific navigation parameters or scattered coefficients.
V72UPDAT	CAF	UP72		# COMES HERE ON V72E
		TCF	V73UPDAT +1

; V73: Clock increment (positive time adjustment)
; Adds a double-precision time increment to the AGC mission clock.
; This is the octal equivalent of V55 (decimal clock update).
V73UPDAT	CAF	UP73		# COMES HERE ON V73E

 +1		TS	UPVERBSV	# SAVE UPVERB UNTIL IT'S OK TO ENTER P27

; Check if display system is available for P27 update program.
; If another program is using the DSKY, this will turn on operator error light.
		TC	TESTXACT	# GRAB DISPLAY IF AVAILABLE, OTHERWISE
					# TURN*OPERATOR ERROR* ON AND TERMINATEJOB

; Ground updates are only permitted during specific mission modes for safety.
; LM accepts updates only during P00 (program 00 - crew standby mode).
; This prevents updates from interfering with critical guidance programs.
		CA	MODREG		# CHECK IF UPDATE ALLOWED
		EXTEND			# FIRST CHECK FOR MODREG = +0, -0
		BZF	+3		# (+0 = P00, -0 = FRESHSTART)
UPERROR		TC	POSTJUMP	# TURN ON 'OPERATOR ERROR' LIGHT
		CADR	UPERROUT +2	# GO TO COMMON UPDATE PROGRAM EXIT

; Mode check passed - updates are permitted.
; Save current major mode so it can be restored after P27 completes.
		CAE	MODREG		# UPDATE ALLOWED.
CKMDMORE	=	UPERROR
		TS	UPOLDMOD	# SAVE CURRENT MAJOR MODE
# Page 1389
		CAE	UPVERBSV	# SET UPVERB TO INDICDATE TO P27
		TS	UPVERB		# WHICH EXTENDED VERB CALLED IT.

		CAF	ONE
		TS	UPCOUNT		# INITIALIZE UPCOUNT TO 1

		TC	POSTJUMP	# LEAVE EXTENDED VERB BANK AND
		CADR	UPPART2		# GO TO UPDATE PROGRAM(P27) BANK.

; Verb identification constants (verb number minus 70)
; Used to branch to appropriate update handling logic.
UP70		EQUALS	ZERO
UP71		EQUALS	ONE
UP72		EQUALS	TWO
UP73		EQUALS	THREE

; ============================================================================
; TRANSITION: Main P27 Update Program Logic
;
; Control now transfers from the extended verb entry points to the main
; update program bank. P27 coordinates the data reception, verification,
; and storage sequence. The program displays V21 N02 to request each data
; component from Mission Control via uplink. After all components are
; received and displayed to the crew for verification, P27 waits for crew
; confirmation (V33E) before permanently storing the updates in memory.
; This two-phase approach protects against corrupted uplink data.
; ============================================================================

		BANK	04
		SETLOC	UPDATE2
		BANK

		COUNT*	$$/P27

UPPART2		EQUALS			# UPDATE PROGRAM - PART 2

; Establish restart protection before beginning update data reception.
; If AGC restarts during P27 (before verification), restore previous mode,
; reset downlist, and exit cleanly - forcing ground to retransmit update.
		TC	PHASCHNG	# SET RESTART GROUP 6 TO RESTORE OLD MODE
		OCT	07026		# AND DOWNLIST AND EXIT IF RESTART OCCURS.
		OCT	30000		# PRIORITY SAME AS CHRPRIO
		EBANK=	UPBUFF
		2CADR	UPOUT +1

; Set downlist code to 1 (Coast and Align downlist) during P27 operation.
; This telemetry configuration provides Mission Control with navigation
; state and alignment data while update is in progress.
		CAF	ONE
		TS	DNLSTCOD	# DOWNLIST

; Display major mode 27 to crew, informing them P27 UPDATE PROGRAM is active.
; Armstrong and Aldrin would see "27" in the major mode display area.
		TC	NEWMODEX	# SET MAJOR MODE = 27
		DEC	27

; Different verbs require different numbers of data components:
; V70/V73: Fixed 2 components (double-precision time)
; V71/V72: Variable component count transmitted as first data item
		INDEX	UPVERB		# BRANCH DEPENDING ON WHETHER THE UPDATE
		TCF	+1		# VERB REQUIRES A FIXED OR VARIABLE NUMBER
		TCF	+3		# V70 FIXED.               (OF COMPONENTS.
		TCF	OHWELL1		# V71 VARIABLE - GO GET NO. OF COMPONENTS
		TCF	OHWELL1		# V72 VARIABLE - GO GET NO. OF COMPONENTS
		CA	TWO		# V73 (AND V70) FIXED
		TS	COMPNUMB	# SET NUMBER OF COMPONENTS TO 2.
		TCF	OHWELL2		# GO GET THE TWO UPDATE COMPONENTS

; Variable component count reception (V71/V72 path)
; For contiguous block (V71) and scatter (V72) updates, the first data
; item transmitted from Mission Control specifies how many components follow.
; Display V21 N01 (flashing) requesting component count from ground uplink.
OHWELL1		CAF	ADUPBUFF	# * REQUEST USER TO SEND NUMBER *
		TS	MPAC +2		# * OF COMPONENTS PARAMETER(II).*
 +2		CAF	UPLOADNV	# (CK4V32 RETURNS HERE IF V32 ENCOUNTERED)
		TC	BANKCALL	# DISPLAY A FLASHING V21N01
# Page 1390
		CADR	GOXDSPF		# TO REQUEST II.
		TCF	UPOUT4		# V34 TERMINATE UPDATE(P27) RETURN
		TCF	OHWELL1 +2
		TC	CK4V32		# DATA OR V32 RETURN

; Validate component count is within acceptable range (3 to 20 decimal).
; Too few components (< 3) or too many (> 20) indicates corrupted uplink.
		CS	BIT2
		AD	UPBUFF		# IS II(NUMBER OF COMPONENTS PARAMETER)
		EXTEND			# .GE. 3 AND .LE. 20D.
		BZMF	OHWELL1 +2
		CS	UPBUFF
		AD	UP21
		EXTEND
		BZMF	OHWELL1 +2
		CAE	UPBUFF
		TS	COMPNUMB	# SAVE II IN COMPNUMB

; ============================================================================
; MAIN DATA COMPONENT RECEPTION LOOP
;
; After determining the required number of components, P27 enters this
; loop to receive each data word from the ground uplink. Mission Control
; transmits state vector components, time increments, or memory addresses
; one at a time. Each transmission triggers V21 N01 display requesting
; the next component. During Apollo 11's mission, this loop processed
; critical navigation updates from Houston's Real-Time Computer Complex.
; ============================================================================

# UPBUFF LOADING SEQUENCE

; Each iteration: increment component counter, calculate buffer address,
; display V21 N01 flashing, receive one data word from ground uplink.
		INCR	UPCOUNT		# INCREMENT COUNT OF COMPONENTS RECEIVED.
OHWELL2		CAF	ADUPBFM1	# CALCULATE LOCATION(ECADR) IN UPBUFF
		AD	UPCOUNT		# WHERE NEXT COMPONENT SHOULD BE STORED.
 +2		TS	MPAC +2		# PLACE ECADR INTO R3.
 +3		CAF	UPLOADNV	# (CK4V32 RETURNS HERE IF V32 ENCOUNTERED)
		TC	BANKCALL	# DISPLAY A FLASHING V21N01
		CADR	GOXDSPF		# TO REQUEST DATA.
		TCF	UPOUT4		# V34 TERMINATE UPDATE(P27) RETURN.
		TCF	OHWELL2 +3	# V33 PROCEED RETURN
		TC	CK4V32		# DATA OR V32 RETURN

; Check if all expected data components have been received.
; When complete (UPCOUNT = COMPNUMB), transition to verification phase
; where crew can review data before final storage into navigation memory.
		CS	UPCOUNT		# HAVE WE FINISHED RECEIVING ALL
		AD	COMPNUMB	# THE DATA WE EXPECTED.
		EXTEND
		BZMF	UPVERIFY	# YES- GO TO VERIFICATION SEQUENCE
		TCF	OHWELL2 -1	# NO- REQUEST ADDITIONAL DATA.

; ============================================================================
; CREW VERIFICATION SEQUENCE
;
; All data components received from ground are now stored in temporary
; buffer UPBUFF. Before committing these updates to navigation memory,
; P27 requires crew verification. This safety check ensures data integrity
; and gives astronauts final authority over navigation state changes.
; Display V21 N02 flashing. Crew presses V33 to accept all data, or sends
; component number to change specific data word before final acceptance.
; ============================================================================

# VERIFY SEQUENCE

UPVERIFY	CAF	ADUPTEMP	# PLACE ECADR WHERE COMPONENT NO. INDEX
		TS	MPAC +2		# IS TO BE STORED INTO R3.
		CAF	UPVRFYNV	# (CK4V32 RETURNS HERE IF V32 ENCOUNTERED)
		TC	BANKCALL	# DISPLAY A FLASHING V21N02 TO REQUEST
		CADR	GOXDSPF		# DATA CORRECTION OR VERIFICATION.
		TCF	UPOUT4		# V34 TERMINATE UPDATE(P27) RETURN
		TCF	UPSTORE		# V33 DATA SENT IS GOOD. GO STORE IT.
		TC	CK4V32		# COMPONENT NO. INDEX OR V32 RETURN

; If crew entered component number instead of V33, validate it's in range.
; Crew can specify which component to change (1 to COMPNUMB).
; Armstrong or Aldrin might change component if ground made transmission error.
		CA	UPTEMP		# DOES THE COMPONENT NO. INDEX JUST SENT
		EXTEND			# SPECIFY A LEGAL COMPONENT NUMBER?
		BZMF	UPVERIFY	# NO, IT IS NOT POSITIVE NONZERO
		CS	UPTEMP
		AD	COMPNUMB
# Page 1391
		AD	BIT1
		EXTEND
		BZMF	UPVERIFY	# NO

; Valid component number - loop back to receive replacement data word
; for that specific component. Ground retransmits corrected value.
		CAF	ADUPBFM1	# YES- BASED ON THE COMPONENT NO. INDEX
		AD	UPTEMP		# CALCULATE THE ECADR OF LOCATION IN
		TCF	OHWELL2 +2	# UPBUFF WHICH USER WANTS TO CHANGE.

UPOUT4		EQUALS	UPOUT +1	# COMES HERE ON V34 TO TERMINATE UPDATE

# CHECK FOR VERB 32 SEQUENCE

CK4V32		CS	MPAC		# ON DATA RETURN FROM 'GOXDSPF'
		MASK	BIT6		# ON DATA RETURN FROM "GOXDSP"& THE CON-
		CCS	A		# TENTS OF MPAC = VERB.  SO TEST FOR V32.
		TC	Q		# IT'S NOT A V32, IT'S DATA.  PROCEED.
		INDEX	Q
		TC	0 -6		# V32 ENCOUNTERED - GO BACK AND GET DATA

ADUPTEMP	ADRES	UPTEMP		# ADDRESS OF TEMP STORAGE FOR CORRECTIONS
ADUPBUFF	ADRES	UPBUFF		# ADDRESS OF UPDATE DATA STORAGE BUFFER
UPLOADNV	VN	2101		# VERB 21 NOUN 01
UPVRFYNV	VN	2102		# VERB 21 NOUN 02
UP21		=	MD1		# DEC 21 = MAX NO OF COMPONENTS +1
UPDTPHAS	EQUALS	FIVE

; ============================================================================
; DATA STORAGE SEQUENCE - COMMIT UPDATES TO MEMORY
;
; TRANSITION: From verification to final storage
;
; Crew has pressed V33 accepting all uplink data. P27 now commits these
; updates to navigation memory. This is the moment ground updates become
; active - state vectors are corrected, clock adjusted, or memory updated.
; Restart protection engaged: if power failure occurs during storage,
; restart routine will complete the update after recovery. This sequence
; executes under INHINT (interrupts inhibited) to prevent corruption.
; ============================================================================

# PRE-STORE AND FAN TO APPROPRIATE BRANCH SEQUENCE

UPSTORE		EQUALS			# GROUND HAS VERIFIED UPDATE.  STORE DATA.

		INHINT

; Signal verification completion to ground via downlink telemetry.
; Mission Control monitors VERIFLAG to confirm crew accepted update.
; Flag inversion generates telemetry change visible at Houston consoles.
		CAE	FLAGWRD7	# INVERT VERIFLAG(BIT3 OF FLAGWRD7) TO
		XCH	L		# INDICATE TO THE GROUND(VIA DOWNLINK)
		CAF	VERIFBIT	# THAT THE V33 (WHICH THE GROUND SENT TO
		EXTEND			# VERIFY THE UPDATE) HAS BEEN SUCCESSFULLY
		RXOR	LCHAN		# RECEIVED BY THE UPDATE PROGRAM
		TS	FLAGWRD7

; Enable restart protection for update storage phase.
; Group 6 restart ensures update completes even if AGC loses power.
; Critical during translunar coast when updates must not be lost.
		TC	PHASCHNG	# SET RESTART GROUP 6 TO REDO THE UPDATE
		OCT	04026		# DATA STORE IF A RESTART OCCURS.
		INHINT			# (BECAUSE PHASCHNG DID A RELINT)

; Branch to appropriate storage routine based on verb number.
; V70-V72 require FINDVAC to schedule interpretive integration tasks.
; V73 (clock increment) executes immediately without task scheduling.
		CS	TWO		# GO TO UPFNDVAC IF INSTALL IS REQUIRED.
		AD	UPVERB		# THAT IS, IF IT'S A V70 - V72.
		EXTEND			# GO TO UPEND73 IF IT'S A V73.
		BZMF	UPFNDVAC

; ============================================================================
; V73 CLOCK INCREMENT BRANCH
;
; Simplest update: adjust AGC mission timer by specified time increment.
; Ground uses V73 to correct minor clock drift or synchronize timing.
; The octal time increment (scaled 2^28 centiseconds) is added directly
; to the AGC clock registers TIME2/TIME1. This is octal equivalent of
; crew-entered V55 decimal time update. Executes immediately without
; requiring VAC area or orbit integration coordination.
; ============================================================================

# VERB 73 BRANCH
# Page 1392
UPEND73		EXTEND			# V73-PERFORM DP OCTAL AGC CLOCK INCREMENT

; Move time increment from reception buffer to working storage.
; TIMEDIDL subroutine adds increment to TIME2/TIME1 double-precision clock.
		DCA	UPBUFF
		DXCH	UPBUFF +8D
		TC	TIMEDIDL
		TC	FALTON		# ERROR- TURN ON *OPERATOR ERROR* LIGHT
		TC	UPOUT +1	# GO TO COMMON UPDATE PROGRAM EXIT

; ============================================================================
; V70/V71/V72 STATE VECTOR UPDATE BRANCH
;
; TRANSITION: From storage initiation to VAC task scheduling
;
; State vector updates (V70 liftoff time, V71 contiguous block, V72 scatter)
; require coordination with orbital integration. Cannot update state vectors
; while orbit integration calculations are in progress - would corrupt
; navigation state. FINDVAC allocates VAC (Vector Accumulator) processing
; area. INTSTALL waits for orbit integration to complete if necessary.
; During Apollo 11's translunar coast, ground transmitted multiple state
; vector corrections as trajectory predictions improved.
; ============================================================================

UPFNDVAC	CAF	CHRPRIO		# (USE EXTENDED VERB PRIORITY)
		TC	FINDVAC		# GET VAC AREA FOR 'CALL INTSTALL'
		EBANK=	TEPHEM
		2CADR	UPJOB		# (NOTE: THIS WILL ALSO SET EBANK FOR

		TC	ENDOFJOB	# 'TEPHEM' UPDATE BY V70)

; Interpretive mode entry for state vector updates.
; INTSTALL checks ORBITAL_INTEGRATION status. If integration in progress,
; this job sleeps until integration completes. If integration idle, job
; proceeds immediately. Prevents corruption of navigation state vectors.
UPJOB		TC	INTPRET		# THIS COULD BE A STATE VECTOR UPDATE--SO
		CALL			# WAIT(PUT JOB TO SLEEP) IF ORBIT INT(OI)
			INTSTALL	# IS IN PROGRESS--OR--GRAB OI AND RETURN
					# TO UPWAKE IF OI IS NOT IN PROGRESS.

; Orbit integration now locked out. Safe to update state vectors.
; Re-enable restart protection for actual data storage phase.
UPWAKE		EXIT

		TC	PHASCHNG	# RESTART PROTECT(GROUP 6)
		OCT	04026

		TC	UPFLAG		# SET INTEGRATION RESTART BIT
		ADRES	REINTFLG
		INHINT
UPPART3		EQUALS

; Branch to specific update routine based on original verb number.
; INDEX instruction uses UPVERB to select correct TCF (transfer control).
; V70: Liftoff time increment update (adjust all ephemeris times)
; V71: Contiguous block update (sequential memory addresses)
; V72: Scatter update (non-contiguous address/data pairs)
		INDEX	UPVERB		# BRANCH TO THE APPROPRIATE UPDATE VERB
		TCF	+1		# ROUTINE TO ACTUALLY PERFORM THE UPDATE
		TCF	UPEND70		# V70
		TCF	UPEND71		# V71
		TCF	UPEND72		# V72

# ROUTINE TO INCREMENT CLOCK(TIME2,TIME1) WITH CONTENTS OF DP WORD AT UPBUFF.

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
# Page 1393
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
; UPEND71 - CONTIGUOUS BLOCK UPDATE (VERB 71)
;
; COMMENT-ONLY READERS: This routine loads consecutive memory locations with
; data from the ground. Mission Control uses V71 when they need to update a
; series of related parameters stored sequentially in memory (like a table of
; constants or a set of guidance coefficients). The ground sends the starting
; address followed by the data values, and the AGC writes them into consecutive
; memory locations automatically. The routine also checks that the block won't
; overflow into the next memory bank, preventing addressing errors.
;
; CODE-ALONG READERS: V71 implements a contiguous block transfer from UPBUFF
; to erasable memory. The routine extracts the ECADR from UPBUFF +1, sets the
; target EBANK, and isolates the relative address. It then verifies that
; (address + COMPNUMB - 3) won't overflow into the next bank (BIT9 check).
; The loop transfers (COMPNUMB - 3) words from UPBUFF +2 onward to consecutive
; addresses, using indexed indirect addressing with EBANK=1400. Each iteration
; calculates the target address, swaps the update word via L register, and
; decrements the counter until all words are stored.
; ============================================================================

UPEND71		CAE	UPBUFF +1	# SET EBANK
		TS	EBANK		#    AND
		MASK	LOW8		# CALCULATE
		TS	UPTEMP		# S-REG VALUE OF RECEIVING AREA

; Verify that the contiguous block update won't overflow into the next EBANK.
; The calculation checks if (starting address + number of words - 3) would
; cross the bank boundary. If BIT9 is set, overflow would occur, triggering
; error exit. This safety check prevents corruption of unintended memory.
		AD	NEG3		# IN THE PROCESS OF
		AD	COMPNUMB	# PERFORMING
		EXTEND			# THIS UPDATE
		BZF	STORLP71	# WILL WE
		MASK	BIT9		# OVERFLOW
		CCS	A		# INTO THE NEXT EBANK....
		TCF	UPERROUT	# YES

; Calculate the loop counter: number of words to store minus one.
; COMPNUMB includes the ECADR itself plus the data words, so subtract 3
; (1 for ECADR, 2 for loop indexing convention) to get the actual count.
		CA	NEG3		# NO- CALCULATE NUMBER OF
		AD	COMPNUMB	# WORDS TO BE STORED MINUS ONE

; Main storage loop for V71. Each iteration fetches the next word from UPBUFF,
; calculates the target address by adding MPAC (loop counter) to UPTEMP (base
; address), then swaps the update word into the target location via L register.
; The LXCH instruction updates the memory location while preserving restart
; capability.
STORLP71	TS	MPAC		# SAVE NO. OF WORDS REMAINING MINUS ONE
# Page 1394
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
; UPEND72 - SCATTER UPDATE (VERB 72)
;
; COMMENT-ONLY READERS: This routine handles non-contiguous (scattered) memory
; updates from the ground. Unlike V71 which updates consecutive addresses, V72
; allows Mission Control to update multiple unrelated memory locations in a
; single uplink. Each address/data pair is sent: the address specifies where
; to store, followed by the data value. This is useful when ground controllers
; need to update several isolated parameters (like a guidance constant and a
; display flag) without sending separate uplink messages. The routine first
; verifies that an odd number of components was sent (verb/noun + pairs = odd).
;
; CODE-ALONG READERS: V72 processes ECADR/data pairs from UPBUFF. COMPNUMB must
; be odd: 2 (verb/noun) + even number of words (pairs) = odd total. After
; validation, the routine calculates the loop counter as COMPNUMB - 2 (subtract
; verb/noun), then iterates through pairs. Each iteration: fetch data word to L,
; decrement counter, fetch ECADR, set EBANK, isolate relative address, and swap
; data into target via indexed LXCH 1400. The loop alternates between data and
; address, processing pairs until counter reaches zero.
; ============================================================================

UPEND72		CAF	BIT1		# HAVE AN ODD NO. OF COMPONENTS
		MASK	COMPNUMB	# BEEN SENT FOR A V72 UPDATE...
		CCS	A
		TCF	+2		# YES
		TCF	UPERROUT	# ERROR- SHOULD BE ODD NO. OF COMPONENTS

; Calculate loop counter: COMPNUMB - 2 (removes verb/noun count).
; The result is the number of data/address words to process.
; Loop will iterate (COMPNUMB - 2) / 2 times for pairs.
		CS	BIT2
		AD	COMPNUMB

; Main scatter update loop. Processes alternating data/ECADR pairs from UPBUFF.
; The loop structure fetches data first, then address, using the counter to
; index through UPBUFF. Each pair update: load data → decrement counter →
; load ECADR → set EBANK → extract relative address → swap data to target.
LDLOOP72	TS	MPAC		# NOW PERFORM THE UPDATE
		INDEX	A
		CAE	UPBUFF +1	# PICK UP NEXT UPDATE WORD
		LXCH	A

; Decrement the loop counter using CCS, which simultaneously tests if more
; pairs remain. The counter value determines the UPBUFF index for the next
; ECADR. This interleaved structure (data, then address) continues until
; all pairs are processed.
		CCS	MPAC		# SET POINTER TO ECADR(MUST BE CCS)
		TS	MPAC
		INDEX	A
		CAE	UPBUFF +1	# PICK UP NEXT ECADR OF REG TO BE UPDATED
		TS	EBANK		# SET EBANK
		MASK	LOW8		# ISOLATE RELATIVE ADDRESS

; Perform the actual memory update. The indexed LXCH uses the relative address
; to access the target location within the selected EBANK, swapping the data
; word (currently in L) with the target location's contents.
		INDEX	A
		EBANK=	1400
		LXCH	1400		# UPDATE THE REGISTER BY CONTENTS OF L
		EBANK=	TEPHEM

; Test if more pairs remain to process. CCS decrements MPAC and branches
; back to LDLOOP72 if positive, or falls through to UPOUT if zero.
		CCS	MPAC		# ARE WE THORUGH THE V72 UPDATE...
		TCF	LDLOOP72	# NO

# NORMAL FINISH OF P27

; ============================================================================
; UPOUT - NORMAL COMPLETION EXIT FOR UPDATE PROGRAM
;
; COMMENT-ONLY READERS: After successfully receiving and storing the uplinked
; data, the AGC completes the update process by releasing the orbital
; integration system (allowing navigation calculations to resume), restoring
; the major mode that was interrupted when P27 started, clearing the downlink
; code, turning off the uplink activity light on the DSKY, and ending the
; extended verb sequence. The crew sees the uplink light extinguish,
; confirming the update completed successfully.
;
; CODE-ALONG READERS: UPOUT is the common successful exit point for all P27
; verbs. INTWAKEU releases the orbital integration inhibit (allowing ORBITAL
; INTEGRATION routines to run). UPOLDMOD (saved at P27 entry) is restored as
; the major mode via NEWMODEX +3. DNLSTCOD is zeroed to reset downlink list
; selection. UPACTOFF turns off DSKY bit 3 (uplink activity indicator). Group
; 6 restart protection is killed by storing NEG0 into -PHASE6. Finally,
; ENDEXT performs the standard extended verb termination sequence.
; ============================================================================

UPOUT		EQUALS
		TC	INTWAKEU	# RELEASE  GRAB  OF ORBITAL INTEGRATION

; Restore the major mode that was active before P27 was invoked by the uplink.
; This returns the AGC to its previous operational state (e.g., P00 for LM,
; or P02 for CSM coast).
 +1		CAE	UPOLDMOD	# RESTORE PRIOR P27 MODE
		TC	NEWMODEX +3
		CAF	ZERO
# Page 1395
		TS	DNLSTCOD

; Turn off the uplink activity light (DSKY indicator bit 3). This visual
; confirmation tells the crew that Mission Control has finished sending data
; and the update is complete.
		TC	UPACTOFF	# TURN OFF 'UPLINK ACTIVITY' LIGHT

; Kill group 6 restart protection by storing negative zero into -PHASE6.
; This clears the P27 restart table entries, indicating the update sequence
; is fully complete and should not be restarted if a subsequent restart occurs.
		EXTEND			# KILL GROUP 6.
		DCA	NEG0
		DXCH	-PHASE6

		TC	ENDEXT		# EXTENDED VERB EXIT

# ============================================================================
; TRANSITION: From data storage to verb-specific update implementation
;
; After the verified data has been stored and restart protection enabled,
; control branches to verb-specific routines that perform the actual updates.
; V70 performs liftoff time adjustments affecting multiple time references,
; V71 updates contiguous memory blocks, and V72 performs scattered updates.
; ============================================================================

# VERB 7O BRANCH

; ============================================================================
; UPEND70 - LIFTOFF TIME DECREMENT (VERB 70)
;
; COMMENT-ONLY READERS: This routine adjusts the spacecraft's time references
; when Mission Control sends a liftoff time correction. The ground calculates
; the time difference and uplinks it as a double-precision value. The AGC then
; decrements the onboard clock and adjusts the state vector times for both the
; Command Module (Columbia) and Lunar Module (Eagle), ensuring all navigation
; calculations remain synchronized after the time correction.
;
; CODE-ALONG READERS: V70 receives a DP time increment in UPBUFF (scaled at
; 2^28 centiseconds). The routine calls TIMEDIDL to decrement the AGC clock
; (TIME2, TIME1), then applies the same decrement to state vector times
; (TETCSM for CM, TETLEM for LM). Finally, it increments TEPHEM (ephemeris
; time) by adding the value. Restart protection (group 6) guards each phase.
; ============================================================================

UPEND70		EXTEND			# V70 DOES THE FOLLOWING WITH DP DELTA
		DCS	UPBUFF		# TIME IN UPBUFF
		DXCH	UPBUFF +8D
		TC	TIMEDIDL	# DECREMENT AGC CLOCK

; If the clock decrement operation fails (e.g., time would go negative),
; exit via error sequence to alert crew with operator error light.
		TC	UPERROUT	# ERROR WHILE DECREMENTING CLOCK -- EXIT

; Copy the time decrement values to multiple buffer locations for restart
; protection. If a restart occurs during the following operations, the
; time adjustments can be safely reapplied.
		EBANK=	TEPHEM
		EXTEND
		DCS	UPBUFF		# COPY DECREMENTERS FOR
		DXCH	UPBUFF +10D	# RESTART PROTECTION
		EXTEND
		DCS	UPBUFF
		DXCH	UPBUFF +12D

		TC	PHASCHNG	# RESTART PROTECT(GROUP 6)
		OCT	04026

; Decrement CSM (Command Module) state vector time TETCSM. This ensures
; that navigation calculations for Columbia's orbit remain correct after
; the liftoff time adjustment. The DP value is zero-extended before the
; double-precision subtraction.
		CAF	ZERO
		ZL
		DXCH	UPBUFF +10D	# DECREMENT CSM STATE VECTOR TIME
		DAS	TETCSM

; Decrement LEM (Lunar Module) state vector time TETLEM. This ensures
; that navigation calculations for Eagle's trajectory (whether docked or
; separated) remain synchronized with the corrected mission timeline.
		CAF	ZERO
		ZL
		DXCH	UPBUFF +12D	# DECREMENT LEM STATE VECTOR TIME
		DAS	TETLEM

; Increment ephemeris time TEPHEM by adding the liftoff time correction.
; TEPHEM tracks astronomical time for lunar and solar position calculations.
; The operation adds to both the fractional part (TEPHEM +1) and whole
; part (TEPHEM) to maintain double-precision accuracy.
		CAF	ZERO
		ZL
		DXCH	UPBUFF
		DAS	TEPHEM +1	# INCREMENT TP TEPHEM
		ADS	TEPHEM

; Second restart protection point after all time adjustments complete.
; This ensures that if a restart occurs now, the update sequence will
; not be repeated (avoiding double-application of time corrections).
		TC	PHASCHNG	# RESTART PROTECT(GROUP 6)
		OCT 	04026

		EBANK=	UPBUFF
# Page 1396
		TC	UPOUT		# GO TO STANDARD UPDATE PROGRAM EXIT


# ERROR SEQUENCE

; ============================================================================
; UPERROUT - ERROR EXIT FOR UPDATE PROGRAM (TWO ENTRY POINTS)
;
; COMMENT-ONLY READERS: When the AGC detects an error in the uplinked data
; (such as an invalid address, wrong number of components, or data format
; problem), this routine alerts Mission Control and the crew. The operator
; error light on the DSKY flashes, indicating the update failed. There are
; two error exits: the standard exit (UPERROUT) which goes through the normal
; completion sequence including restart group cleanup, and an alternate exit
; (UPERROUT +2) which skips restart group reset for certain error conditions.
; Ground controllers must diagnose the problem and resend the uplink correctly.
;
; CODE-ALONG READERS: UPERROUT provides two error handling paths:
; 1. UPERROUT (standard): Calls FALTON to illuminate DSKY operator error
;    light, then branches to UPOUT which performs full cleanup (restart group
;    kill, mode restoration, activity light off, ENDEXT).
; 2. UPERROUT +2 (alternate): Calls FALTON for error indication, then
;    UPACTOFF to clear activity light, then ENDEXT to terminate. This path
;    DOES NOT reset restart groups, preserving restart protection state for
;    errors detected before the VERIFLAG inversion point. Used for validation
;    failures during initial data reception before update commitment.
; ============================================================================

UPERROUT	TC	FALTON		# TURN ON *OPERATOR ERROR* LIGHT
		TCF	UPOUT		# GO TO COMMON UPDATE PROGRAM EXIT

; Alternate error exit that preserves restart groups. Used when an error
; is detected before the update data has been committed (before VERIFLAG
; inversion). This allows restart recovery to restore pre-P27 state without
; partially-applied updates.
 +2		TC	FALTON		# TURN ON 'OPERATOR ERROR' LIGHT
		TC	UPACTOFF	# TURN OFF'UPLINK ACTIVITY'LIGHT
		TC	ENDEXT		# EXTENDED VERB EXIT
					# (THE PURPOSE OF UPERROUT +2 EXIT IS
					# TO PROVIDE AN ERROR EXIT WHICH DOES NOT
					# RESET ANY RESTART GROUPS)


# :UPACTOFF: IS A ROUTINE TO TURN OFF UPLINK ACTIVITY LIGHT ON ALL EXITS FROM UPDATE PROGRAM(P27).

; ============================================================================
; UPACTOFF - TURN OFF UPLINK ACTIVITY INDICATOR
;
; COMMENT-ONLY READERS: This simple utility routine turns off the uplink
; activity light on the DSKY, providing visual confirmation to the crew
; that Mission Control has stopped transmitting data. Whether the update
; succeeded or failed, the uplink activity indicator is cleared when data
; transmission ceases. The crew sees the light extinguish, confirming that
; the ground has finished sending commands.
;
; CODE-ALONG READERS: UPACTOFF clears DSKY bit 3 (uplink activity indicator)
; by performing a logical AND with the complement of BIT3 on DSALMOUT (channel
; 11, DSKY alarm and light output channel). The CS BIT3 instruction forms the
; mask ~00004 (all bits set except bit 3). EXTEND/WAND performs the channel
; write-AND operation, clearing bit 3 while preserving all other bits. Returns
; via Q register to caller (can be called from multiple locations: UPOUT,
; UPERROUT +2, or other P27 exit paths).
; ============================================================================

UPACTOFF	CS	BIT3
		EXTEND			# TURN OFF UPLINK ACTIVITY LIGHT
		WAND	DSALMOUT	# (BIT 3 OF CHANNEL 11)
		TC	Q

