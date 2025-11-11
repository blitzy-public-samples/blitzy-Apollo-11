# Copyright:	Public domain.
# Filename:	SXTMARK.agc
# Purpose:	Part of the source code for Comanche, build 055. It
#		is part of the source code for the Command Module's
#		(CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 222-235
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	16/05/09 FB	Transcription Batch 2 Assignment.
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
; FILE: SXTMARK.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Sextant optical navigation mark processing for star and landmark
;        sighting. Captures crew optical measurements through sextant, processes
;        line-of-sight angles, and integrates with navigation state update
;        routines for IMU alignment verification and position determination.
;
; COMMENT-ONLY READERS: This program processed crew sightings of stars and
;        landmarks through the spacecraft's optical telescope.
; CODE-ALONG READERS: Study optical navigation measurement processing, sextant
;        angle computation, and navigation state update integration.
; ============================================================================

# Page 222
# PROGRAM NAME - SXTMARK					DATE- 5 APRIL 1967
# PROGRAM MODIFIED BY 258/278 PROGRAMMERS		       LOG SECTION SXTMARK
# MOD BY- R. MELANSON TO ADD DOCUMENTATION		 ASSEMBLY SUNDISK REV. 116

; ============================================================================
; SEXTANT MARK SYSTEM OVERVIEW
;
; The Command Module's sextant allowed astronauts to sight stars and lunar
; landmarks for optical navigation. When the astronaut pressed the mark button,
; this system captured the precise spacecraft attitude (via IMU gimbal angles)
; and the sextant's line-of-sight direction at that instant. These optical
; measurements were then processed by navigation routines to update the
; spacecraft's position and velocity estimates, verify IMU alignment, and
; improve trajectory knowledge during translunar coast and lunar orbit.
;
; The sextant mark system operated independently of the main guidance programs,
; allowing navigation updates without interrupting mission-critical routines.
; ============================================================================

# FUNCTIONAL DESCRIPTION-
#	SXTMARK IS CALLED FROM INTERNAL ROUTINES WHICH MAY REQUIRE STAR OR LANDMARK MARKINGS BY THE ASTRONAUT.  IF
#	THE MARK SYSTEM IS NOT IN USE, SXTMARK RESERVES A VAC AREA FOR MARKING AND REQUESTS EXECUTION OF THE MKVB51
#	ROUTINE VIA THE EXECUTIVE JOB PRIORITY LIST.  R21 USES THIS ROUTINE TO DETERMINE IF THE MARK SYSTEM CAN BE
#	USED.  IF YES, SXTMARK RETURNS TO R21 TO PERFORM ITS OWN MARK REQUESTS VIA THE V51 FLASH.

# CALLING SEQUENCE-

#	CAF	(NO. MARK REQUESTS IN BITS 1-3 OF A)
#	TC	BANKCALL
#	CADR	SXTMARK

# NORMAL EXIT MODE-
#	SWRETURN

# ALARM OR ABORT EXIT MODE-
#	ABORT

# OUTPUT-
#	1) MARKSTAT CONTAINS MARK VALUE (BITS 14-12) AND VAC AREA ADDRESS
#	2) QPRET = VAC AREA POINTER VALUE
#	3) 1ST WORD OF RESERVED VAC AREA SET TO +0
#	4) PRIO32 PLACED IN A REGISTER

# ERASABLE INITIALIZATION-
#	1) BITS 1-3 OF A = NO. MARKS REQUESTED
#	2) BITS 2,3 OF EXTVBACT =0
#	3) A VAC AREA MUST BE AVAILABLE (WORD 1 = ADDRESS OF VAC AREA)

# DEBRIS-
#	A,Q,L,RUPTREG1,MARKSTAT,QPRET,BIT2 OF EXTVBACT

		BANK	13
		SETLOC	SXTMARKE
		BANK

		EBANK=	MRKBUF1
		COUNT	07/SXTMK

; ============================================================================
; SXTMARK - MAIN ENTRY POINT FOR OPTICAL NAVIGATION MARKING
;
; This routine is called when a navigation program (such as P20-series
; rendezvous programs, P51-P53 IMU alignment programs, or R21 landmark
; tracking) requires the astronaut to make optical sightings through the
; sextant. The routine checks if the marking system is already in use by
; another program, and if available, reserves it and sets up the marking
; interface.
;
; The astronaut would position the sextant crosshairs on a star or landmark,
; then press the MARK button on the DSKY. This triggered the MARKRUPT
; interrupt handler which captured the sextant shaft and trunnion angles
; along with IMU CDU (Coupling Data Unit) readings, preserving the exact
; line-of-sight direction at the moment of marking.
; ============================================================================

SXTMARK		INHINT
		TS	RUPTREG1		# NUMBER OF MARKS WANTED

; Check if the marking system is currently in use. The EXTVBACT word
; contains status bits indicating whether marking is active (BIT2) or
; an extended verb is in progress (BIT3). If either bit is set, the
; marking system cannot be used by another program, and we abort.
; This prevents conflicts between multiple navigation programs trying
; to use the sextant simultaneously.

		CAF	SIX			# BIT2 = MARKING SYSTEM IN USE
		MASK	EXTVBACT		# BIT3 = EXTENDED VERB IN PROGRESS
		CCS	A
		TC	MKABORT			# SET THEREFORE ABORT
# Page 223

; Marking system is available. Set BIT2 in EXTVBACT to reserve it,
; preventing other programs from attempting to use it until this
; marking sequence completes (cleared in ENDMARK routine).

		CAF	BIT2			# NOT SET
		ADS	EXTVBACT		# SET IT, RESET IN ENDMARK
		TC	MARKOK			# YES, FIND VAC AREA

MKABORT		TC	BAILOUT
		OCT	01211

; ============================================================================
; VAC AREA ALLOCATION
;
; The AGC has five Vector Accumulator (VAC) areas used for temporary storage
; of vector/matrix computations. Each VAC area can be reserved by setting its
; USE word to a non-zero value. The marking system needs one VAC area to store
; the captured mark data (sextant angles, CDU readings, time) until it can be
; processed by navigation routines.
;
; This section searches through VAC1 through VAC5 to find an available area.
; If all five are in use, the system aborts with alarm 01207, indicating
; insufficient computational resources to perform the marking operation.
; ============================================================================

MARKOK		CCS	VAC1USE			# FIND VAC AREA
		TC	MKVACFND
		CCS	VAC2USE
		TC	MKVACFND
		CCS	VAC3USE
		TC	MKVACFND
		CCS	VAC4USE
		TC	MKVACFND
		CCS	VAC5USE
		TC	MKVACFND

; No VAC area available - abort with alarm code 01207

		TC	BAILOUT
		OCT	01207

; ============================================================================
; MKVACFND - VAC AREA SETUP AND MARK REQUEST INITIALIZATION
;
; Once an available VAC area is found, this routine sets it up for storing
; mark data. The MARKSTAT word is configured to contain both the VAC area
; base address and the number of marks requested (stored in bits 12-14).
; This allows the marking system to know where to store captured sextant
; data and how many marks the navigation program expects.
;
; The routine checks if we're in Major Mode 53 or 54 (backup IMU alignment
; programs), which handle marking differently and don't need the standard
; MKVB51 display routine.
;
; After setup, the routine schedules the MKVB51 job via NOVAC, which displays
; Verb 51 on the DSKY. This flashing verb alerts the astronaut that the
; computer is ready to receive sextant marks. The astronaut would then
; position the sextant and press MARK (or REJECT to skip/cancel).
; ============================================================================

MKVACFND	AD	TWO			# ADDRESS OF VAC AREA
		TS	MARKSTAT
		INDEX	A
		TS	QPRET			# STORE NEXT AVAILABLE MARK SLOT

		CAF	ZERO			# STORE VAC AREA OCCUPIED
		INDEX	MARKSTAT
		TS	0	-1

; Check if we're in Major Mode 53 or 54 (backup alignment programs).
; These modes handle marking internally and return immediately without
; scheduling the standard V51 display job.

		TC	CHECKMM			# BACKUP MARK ROUTINE USES SXTMARK
		MM	53
		TCF	+2
		TCF	SWRETURN
		TC	CHECKMM
		MM	54
		TCF	+2
		TCF	SWRETURN

; Not in backup mode. Set up MARKSTAT with requested mark count.
; BIT12 places the count in bits 12-14 of MARKSTAT.

		CAF	BIT12			# DESIRED NUMBER OF MARKS IN 12-14
		EXTEND
		MP	RUPTREG1
		XCH	L
		ADS	MARKSTAT

; Schedule MKVB51 job to display "PLEASE MARK" verb to astronaut.
; Priority 32 allows this display job to run at appropriate priority
; without disrupting higher-priority navigation or guidance computations.

		CAF	PRIO32			# ENTER MARK JOB
		TC	NOVAC
		EBANK=	MARKSTAT
		2CADR	MKVB51

		RELINT
		TCF	SWRETURN		# SAME AS MODEEXIT

# Page 224
# PROGRAM NAME - MKRELEAS					DATE- 5 APRIL 1967
# PROGRAM MODIFIED BY 248/278 PROGRAMMERS		       LOG SECTION SXTMARK
# MOD BY- R. MELANSON TO ADD DOCUMENTATION		 ASSEMBLY SUNDISK REV. 116

# FUNCTIONAL DESCRIPTION-
#	MKRELEAS IS EXECUTED BY INTERNAL ROUTINES TO RELEASE THE MARK SYSTEM TO MAKE IT AVAILABLE TO OTHER INTERNAL
#	SYSTEM ROUTINES.  IT ALSO CLEARS THE COARSE OPTICS FLAG BIT AND DISABLES THE OPTICS ERROR COUNTER.

# CALLING SEQUENCE-

#	TC	BANKCALL
#	CADR	MKRELEAS

# NORMAL EXIT MODE-
#	SWRETURN

# ALARM OR ABORT EXIT MODE- NONE

# OUTPUT-
#	1) BIT9 OPTMODES SET TO 0
#	2) OPTIND SET TO -1
#	3) 1ST WORD OF VAC AREA SET TO VAC ADDRESS TO SIGNIFY AVAILABILITY.
#	4) MARKSTAT CLEARED
#	5) BIT2 CHANNEL 12 SET TO 0

# ERASABLE INITIALIZATION- NONE

# DEBRIS-
#	A,MARKSTAT,BIT9,OPTMODES OPTIND,BIT2 CHANNEL 12

; ============================================================================
; MKRELEAS - RELEASE MARKING SYSTEM AND FREE VAC AREA
;
; When a navigation program completes its marking operations (such as P52
; IMU alignment completing its star sightings), MKRELEAS is called to
; release the marking system for use by other programs. This routine:
;   1. Clears MARKSTAT and frees the VAC area
;   2. Clears the coarse optics flag (BIT9 of OPTMODES)
;   3. Disables the optics error counter (BIT2 of CHAN12)
;
; COMMENT-ONLY READERS: After the astronaut completed sextant markings for
; navigation, this routine released the marking system so other programs
; could use it and reset the optics system to idle state.
;
; CODE-ALONG READERS: This routine performs complete teardown of marking
; system state. The VAC area address is extracted from MARKSTAT (bits 1-9),
; and the area is freed by storing its own address in its first word.
; The coarse optics return flag is cleared, and the optics error counter
; hardware is disabled via channel 12.
; ============================================================================

MKRELEAS	CAF	ZERO			# SHOW MARK SYSTEM NOW AVAILABLE
		XCH	MARKSTAT		# Clear MARKSTAT, get old value
		MASK	LOW9			# Extract VAC area address (bits 1-9)
		CCS	A			# Check if VAC area was allocated
		INDEX	A			# If yes, free it
		TS	0			# Store address in word 0 = available

; Clear coarse optics mode and disable error counter.
; This returns the optics system to idle state.

MKRLEES		INHINT
		CS	BIT9			# COARSE OPTICS RETURN FLAG.
		MASK	OPTMODES		# Clear BIT9 in OPTMODES
		TS	OPTMODES

		CA	NEGONE			# -1 = optics system idle
		TS	OPTIND			# KILL COARSE OPTICS

		CS	BIT2			# DISABLE OPTICS ERROR COUNTER
		EXTEND
		WAND	CHAN12			# Clear BIT2 in channel 12 hardware

		RELINT
		TC	SWRETURN
# Page 225
# PROGRAM NAME - MARKRUPT					DATE- 5 APRIL 1967
# PROGRAM MODIFIED BY 258/278 PROGRAMMERS		       LOG SECTION SXTMARK
# MOD BY- R. MELANSON TO ADD DOCUMENTATION		 ASSEMBLY SUNDISK REV. 116

# FUNCTIONAL DESCRIPTION-
#	MARKRUPT STORES CDUS,OPTICS AND TIME AND TRANSFERS CONTROL TO THE MARKIT,MARK REJECT OR KEYCOM ROUTINES IF
#	BITS IN CHANNEL 16 ARE SET AS REQUIRED.

# CALLING SEQUENCE-
#	ROUTINE ENTERED VIA KEYRUPT2 WHEN MARK,MARK REJECT OR DSKY KEYS DEPRESSED BY THE OPERATOR.

# NORMAL EXIT MODE-
#	MARKIT, MKREJECT, OR POSTJUMP ROUTINES (MARK,MARK REJECT, OR DSKY CODE)

# ALARM OR ABORT EXIT MODE-
#	ALARM AND RESUME

# OUTPUT-
#	RUPTSTOR+5 = CDUT, RUPTSTOR+3 = CDUS, RUPTSTOR+2 = CDUY,
#	RUPTREG3 = CDUZ, RUPTSTOR+6 = CDUX, RUPTSTOR+1 AND SAMPTIME+1 =TIME1,
#	RUPTSTOR AND SAMPTIME = TIME2

# ERASABLE INITIALIZATION-
#	CDUT,CDUS,CDUY,CDUZ,CDUX,TIME2,TIME1,CHANNEL 16 BITS 6,7 OR 1-5

# DEBRIS-
#	A,QRUPT,RUPTREG3,SAMPTIME,SAMPTIME+1,RUPTSTOR TO RUPTSTOR+6 EXCEPT RUPTSTOR+4 (LOCATION 67)

; ============================================================================
; MARKRUPT - MARK BUTTON INTERRUPT HANDLER
;
; This interrupt handler is entered when the astronaut presses the MARK
; button, MARK REJECT button, or any DSKY key. The routine immediately
; captures time-critical data:
;   - CDU angles (CDUX, CDUY, CDUZ from IMU gimbals)
;   - Optics angles (CDUS shaft, CDUT trunnion from sextant)
;   - TIME2/TIME1 (precise moment of button press)
;
; COMMENT-ONLY READERS: When the astronaut pressed MARK while viewing a
; star or landmark through the sextant, this interrupt routine captured
; the exact time and all spacecraft/optics angles at that instant. This
; data would then be processed to determine the line-of-sight direction
; and update the navigation state.
;
; CODE-ALONG READERS: This is an interrupt service routine entered via
; KEYRUPT2. All CDU and optics counter values are read immediately to
; minimize timing errors. The TIME2/TIME1 capture provides microsecond-
; precision timestamp. After data capture, the routine checks NAVKEYIN
; channel 16 to determine if this was MARK (BIT6), MARK REJECT (BIT7),
; or other DSKY key (bits 1-5).
; ============================================================================

MARKRUPT	TS	BANKRUPT		# STORE CDUS AND OPTICS NOW

; Capture all CDU angles immediately for time-critical measurement.
; CDUx = IMU gimbal angles, CDUS/CDUT = sextant optics angles.

		CA	CDUT			# Sextant trunnion angle
		TS	MKCDUT
		CA	CDUS			# Sextant shaft angle
		TS	MKCDUS
		CA	CDUY			# IMU inner gimbal
		TS	MKCDUY
		CA	CDUZ			# IMU middle gimbal
		TS	MKCDUZ
		CA	CDUX			# IMU outer gimbal
		TS	MKCDUX

; Capture precise mission elapsed time at button press moment.
; TIME2/TIME1 = centiseconds since mission start.

		EXTEND
		DCA	TIME2			# GET TIME
		DXCH	MKT2T1
		EXTEND
		DCA	MKT2T1
		DXCH	SAMPTIME		# RUPT TIME FOR NOUN 65.

		XCH	Q
		TS	QRUPT

; Determine what button was pressed by checking channel 16 (NAVKEYIN).
; BIT6 = MARK button, BIT7 = MARK REJECT, bits 1-5 = DSKY keys.

		CAF	BIT6			# SEE IF MARK OR MKREJECT
# Page 226
		EXTEND
		RAND	NAVKEYIN		# Check channel 16 for MARK button
		CCS	A
		TC	MARKIT			# ITS A MARK

; If not MARK button, check for MARK REJECT button.

		CAF	BIT7			# NOT A MARK, SEE IF MKREJECT
		EXTEND
		RAND	NAVKEYIN		# Check channel 16 for MARK REJECT
		CCS	A
		TC	MKREJECT		# ITS A MARK REJECT

; If neither MARK nor MARK REJECT, check for DSKY key press.

KEYCALL		CAF	OCT37			# NOT MARK OR MKREJECT, SEE IF KEYCODE
		EXTEND
		RAND	NAVKEYIN		# Check bits 1-5 for DSKY keys
		EXTEND
		BZF	+3			# IF NO INBITS
		TC	POSTJUMP
		CADR	KEYCOM			# IT,S A KEY CODE, NOT A MARK.

; If no button detected at all, issue alarm 113.

	+3	TC	ALARM			# ALARM IF NO INBITS
		OCT	113
		TC	RESUME

# Page 227
; ============================================================================
; ROUTINE: MARKCONT (Mark Continue)
; 
; Mark data has been captured and validated. This routine stores the mark
; data in the VAC area buffer and determines if all requested marks have
; been completed. If more marks are needed, continues displaying V51 request.
; If all marks completed, displays V50 to indicate termination.
;
; Handles three modes: R21 special marking, special display job, and normal
; star/landmark marking for navigation state updates.
; ============================================================================

# PROGRAM NAME - MARKCONT				DATE- 19 SEPT 1967

# PROGRAM MODIFIED BY 258/278 PROGRAMMERS	       LOG SECTION SXTMARK
# MOD BY- R. MELANSON TO ADD DOCUMENTATION	 ASSEMBLY SUNDISK REV. 116

# FUNCTIONAL DESCRIPTION-
#	MARKCONT IS USED TO PERFORM A SPECIAL MARK FUNCTION FOR R21, TO EXECUTE A SPECIAL DISPLAY OF OPTICS AND TIME OR
#	 TO PERFORM A MARK OF THE STAR OR LAND SIGHTING BASED UPON FLASHING V-N.

# CALLING SEQUENCE-
#	FROM MARKDIF

# NORMAL EXIT MODE-
#	TASKOVER

# ALARM OR ABORT EXIT MODE-
#	ALARM AND TASKOVER

# OUTPUT-
#	1) FOR R21-
#	   EBANK=EBANK7
#	   MRKBUF1 TO MRKBUF1+6 = TIME2,TIME1,CDUY,OPTICX,CDUZ,OPTICSY,CDUX OF CURRENT R21 MARK FUNCTION.
#	   MRKBUF2 TO MRKBUF2+6 CONTAINS PREVIOUS R21 MARK VALUES.
#	2) FOR SPECIAL DISPLAY JOB-
#	   RUPTREG1 AND MRKBUF1 = CDUS,RUPTREG2 AND MRKBUF1 +1 = CDUT.
#	   RUPTREG3 AND MRKBUF1 +2 = TIME2,RUPTREG4 AND MRKBUF1 +3 = TIME1
#	3) FOR NORMAL MARKING-
#	   DECREMENT BITS14-12 OF MARKSTAT BY 1,
#	   BIT10 MARKSTAT SET TO 1,INCREMENT QPRET BY 7,
#	   STORE TIME2,TIME1,CDUY,CDUS,CDUZ,CDUT AND CDUX IN VAC+1 TO VAC+7

# ERASABLE INITIALIZATION-
#	1) FOR R21-
#	   BIT14 OF STATE+2 =1, MRKBUF1 TO MRKBUF1+6, ITEMP1, RUPTREG3,
#	   RUPTSTOR TO RUPTSTOR+6 EXCEPT RUPTSTOR+4
#	2) FOR SPECIAL DISPLAY JOB-
#	   BIT14 OF STATE+2 =0,MARKSTAT =+0,RUPTREG1,RUPTREG2,RUPTREG3
#	   RUPTREG4,RUPTSTOR,RUPTSTOR+1,RUPTSTOR+3,RUPTSTOR+5,
#	   BIT12 OF STATE+5 (V59 FLAG),MRKBUF1 THRU MRKBUF1+3
#	3) FOR NORMAL MARKING-
#	   BIT14 OF STATE+2 =0,MARKSTAE =VAC ADDRESS, A REG, ITEMP1, RUPTREG3,
#	   RUPTSTOR TO RUPTSTOR+6 EXCEPT RUPTSTOR+4.

# DEBRIS-
#	1) FOR R21-
#	   A,ITEMP1,MRKBUF1,MRKBUF2
#	2) FOR SPECIAL DISPLAY JOB-
#	   A,RUPTREG1,RUPTREG2,RUPTREG3,RUPTREG4,MPAC TO MPAC+3
#	3) FOR NORMAL MARKING-
#	   A,MARKSTAT,ITEMP1,QPRET,VAC+1 TO VAC+7 OF VAC AREA IN USE

# Page 228
; Determine which type of mark processing is required: R21 special marking,
; special display job, or normal star/landmark marking.

MARKCONT	CAF	BIT14
		MASK	STATE	+2		# R21 MARK (SPECIAL MARKING FOR R21)
		EXTEND
		BZF	MARKET			# NOT SET THEREFORE REGULAR MARKING

; R21 Special Marking Mode:
; R21 (Rendezvous Navigation) uses this special marking routine to maintain
; a history of two consecutive marks for relative navigation computations.

MARKIT1		CAF	SIX			# SPECIAL FOR R21
		TC	GENTRAN			# TRANSFER MRKBUF1 TO MRKBUF2
		ADRES	MRKBUF1			# Copy previous mark to history buffer
		ADRES	MRKBUF2

		CAF	SIX			# TRANSFER CURRENT MARK DATA TO MARKBUF1
		TC	GENTRAN			# Store new mark in primary buffer
		ADRES	MKT2T1			# Source: captured mark data
		ADRES	MRKBUF1			# Destination: primary mark buffer

		TCF	TASKOVER		# R21 mark complete

; Regular Marking Mode (Non-R21):
; Handle normal star/landmark marking or special display jobs.

MARKET		CCS	MARKSTAT		# SEE IF MARKS CALLED FOR
		TC	MARK2			# COLLECT MARKS

; If MARKSTAT was zero, check if this is a special display job.
; Special display job: Used by P23 (optical calibration) with V59.

		CAF	TWO			# IS MARKING SYSTEM IN USE (BIT2)
		MASK	EXTVBACT
		EXTEND
		BZF	MARKET3			# MARKING NOT CALLED FOR
		CAF	BIT12
		MASK	STATE	+5		# V59FLAG
		EXTEND
		BZF	MARKET3			# IF V59FLAG NOT SET-MARK UNCALLED FOR

; V59 is active (optical calibration display mode for P23).
; Schedule MARKDISP job to display shaft and trunnion angles with time.

		CAF	PRIO5			# CALIBRATION MARK (SET) FOR P23
		TC	NOVAC			# SPECIAL DISPLAY JOB
		EBANK=	MRKBUF1
		2CADR	MARKDISP

		CAF	SIX
		TC	GENTRAN			# TRANSFER MARK DATE TO MARKDOWN
		ADRES	MKT2T1
		ADRES	MARKDOWN
		CAF	SIX
		TC	GENTRAN			# TRANSFER MARK DATA TO MRKBUF1 FOR
		ADRES	MKT2T1			# SPECIAL DISPLAY OF SHAFT AND TRUNNION
		ADRES	MRKBUF1			# IF V59 ACTING
		TCF	TASKOVER

MARKET3		TC	ALARM
		OCT	122			# MARKING NOT CALLED FOR
		TCF	TASKOVER

114ALM		TC	ALARM			# MARK NOT WANTED
		OCT	114
		TCF	TASKOVER

# Page 229
# STORE MARK DATA IN MKVAC AND INCREMENT POINTER

; Normal Mark Processing:
; Decrement mark counter, store mark data in VAC area, advance pointer.
; Mark data consists of 7 words: TIME2, TIME1, CDUY, CDUS (shaft angle),
; CDUZ, CDUT (trunnion angle), and CDUX.

MARK2		AD	74K			# SEE IF MARKS WANTED-REDUCE MARKS WANTED
		EXTEND
		BZMF	114ALM			# MARK NOT WANTED-ALARM
		TS	MARKSTAT		# Store decremented mark count
		COM
		MASK	BIT10			# SET BIT10 TO ENABLE REJECT
		ADS	MARKSTAT		# BIT10=1 allows mark rejection

; Extract VAC area address from MARKSTAT and get mark storage pointer.

		MASK	LOW9			# Extract VAC area number (bits 1-9)
		TS	ITEMP1
		INDEX	A			# Index into VAC area control table
		XCH	QPRET			# PICK UP MARK SLOT-POINTER
		TS	ITEMP2			# SAVE CURRENT POINTER
		AD	SEVEN			# INCREMENT POINTER (7 words per mark)
		INDEX	ITEMP1
		TS	QPRET			# STORE ADVANCED POINTER

; Store captured mark data (7 words) in VAC area at current pointer location.
; Data format: TIME2, TIME1, CDUY, CDUS, CDUZ, CDUT, CDUX

VACSTOR		EXTEND
		DCA	MKT2T1			# TIME2 and TIME1 (double precision)
		INDEX	ITEMP2
		DXCH	0			# Store at VAC+0 and VAC+1
		CA	MKCDUY			# IMU CDU Y-axis angle
		INDEX	ITEMP2
		TS	2			# Store at VAC+2
		CA	MKCDUS			# Sextant shaft angle (OPTICS X-axis)
		INDEX	ITEMP2
		TS	3			# Store at VAC+3
		CA	MKCDUZ			# IMU CDU Z-axis angle
		INDEX	ITEMP2
		TS	4			# Store at VAC+4
		CA	MKCDUT			# Sextant trunnion angle (OPTICS Y-axis)
		INDEX	ITEMP2
		TS	5			# Store at VAC+5
		CA	MKCDUX			# IMU CDU X-axis angle
		INDEX	ITEMP2
		TS	6			# Store at VAC+6 (completes 7-word mark)

; Check if all requested marks have been captured.
; If mark counter in MARKSTAT bits 12-14 is zero, all marks complete.

		CAF	PRIO34			# IF ALL MARKS MADE FLASH VB50
		MASK	MARKSTAT		# Extract mark counter (bits 12-14)
		EXTEND
		BZF	+2			# If zero, all marks complete
		TCF	TASKOVER		# More marks needed, continue V51
		CAF	PRIO32
		TC	NOVAC			# Schedule VB50 termination display
		EBANK=	MARKSTAT
		2CADR	MKVB50			# Flash V50 to indicate completion

		TCF	TASKOVER

# Page 230
; ============================================================================
; ROUTINE: MKREJECT (Mark Reject)
;
; Allows astronaut to reject a mark after capture but before acceptance.
; Called from MARKRUPT when reject button pressed (BIT7 of CHANNEL 16).
; 
; For R21: Sets flag indicating rejected mark (-1 in MRKBUF1).
; For normal marking: Clears reject enable bit, increments mark counter,
; and backs up VAC pointer by 7 words to overwrite rejected mark.
;
; This gives crew ability to correct erroneous optical sightings before
; they are processed by navigation state update routines.
; ============================================================================

# PROGRAM NAME - MKREJECT					DATE- 5 APRIL 1967
# PROGRAM MODIFIED BY 258/276 PROGRAMMERS		       LOG SECTION SXTMARK
# MOD BY- R. MELANSON TO ADD DOCUMENTATION		 ASSEMBLY SUNDISK REV. 116

# FUNCTIONAL DESCRIPTION-
#	ROUTINE ALLOWS OPEATOR TO REJECT MARK MADE PRIOR TO ACCEPTANCE AND ALLOWS A NEW MARK TO BE MADE BY ASTRONAUT

# CALLING SEQUENCE-
#	FROM MARKRUPT IF BIT7 OF CHANNEL 16 IS 1.

# NORMAL EXIT MODE-
#	RESUME

# ALARM OR ABORT EXIT MODE-
#	ALARM AND RESUME

# OUTPUT-
#	1) FOR R21-
#	  MRKRUP1 SET TO -1
#	2) FOR NORMAL MARKING-
#	   BIT10 MARKSTAT =0,INCREMENT NO. MARKS BY 1,DECREMENT QPRET BY 7

# ERASABLE INITIALIZATION-
#	1) FOR R21-
#	   BIT14 OF STATE+2 SET TO 1
#	2) FOR NORMAL MARKING-
#	   BIT14 OF STATE+2 SET TO 0, MARKSTAT,QPRET

# DEBRIS-
#	1) FOR R21-
#	   A,MARKSTAT,EBANK
#	2) FOR NORMAL MARKING-
#	   A,MARKSTAT,ITEMP1,QPRET

; Check if this is R21 special marking or normal marking.

MKREJECT	CAF	BIT14
		MASK	STATE	+2		# R21 MARK (SPECIAL MARKING FOR R21)
		EXTEND
		BZF	MRKREJCT		# NOT SET THEREFORE REGULAR REJECT

; R21 Mark Rejection: Set flag to -1 in mark buffer to signal R22.

		CA	NEGONE			# -1 (FOR R22)
		TS	MRKBUF1			# -0 IN TIME IS FLAG TO R22 SIGNIFYING A
		TC	RESUME			# REJECTED MARK

; Normal (non-R21) Mark Rejection:
; First verify that marking system is active (MARKSTAT positive).

MRKREJCT	CCS	MARKSTAT		# SEE IF MARKS BEING ACCEPTED
		TC	REJECT2			# Positive, marking active
		TC	ALARM			# MARKS NOT BEING ACCEPTED
		OCT	112			# Alarm 00112: reject not valid
		TC	RESUME

; Check if a mark was made since last reject by examining BIT10.
; BIT10=1 means mark was made and can be rejected.
; BIT10=0 means no new mark to reject (already rejected or no mark yet).

REJECT2		CS	BIT10			# SEE IF MARK HAD BEEN MADE SINCE LAST
		MASK	MARKSTAT		# REJECT, AND SET BIT10 TO ZERO TO
		XCH	MARKSTAT		# SHOW MARK REJECT (clear BIT10)
# Page 231
		MASK	BIT10			# Extract old BIT10 value
		CCS	A			# Was BIT10 set (mark available)?
		TC	REJECT3			# Yes, proceed with rejection

		TC	ALARM			# DONT ACCEPT TWO REJECTS TOGETHER
		OCT	110			# Alarm 00110: double reject not allowed
		TC	RESUME

; Valid rejection: Back up VAC pointer by 7 words to overwrite the rejected
; mark data. Increment mark counter to request replacement mark.

REJECT3		CAF	LOW9			# DECREMENT POINTER TO REJECT MARK
		MASK	MARKSTAT		# Extract VAC area number
		TS	ITEMP1
		CS	SEVEN			# -7 words
		INDEX	ITEMP1
		ADS	QPRET			# NEW POINTER (backs up 7 words)

; Increment mark counter in MARKSTAT (bits 12-14).
; If counter becomes non-zero (more marks needed), switch from V50 to V51
; to request another mark from astronaut.

		CAF	BIT12			# INCREMENT MARKS WANTED AND IF FIELD
		AD	MARKSTAT		# IS NOW NON-ZERO, CHANGE TO VB51 TO
		XCH	MARKSTAT		# INDICATE MORE MARKS WANTED
		MASK	PRIO34			# Check mark counter (bits 12-14)
		CCS	A			# Any marks still needed?
		TC	RESUME			# Yes, already displaying V51
		CAF	PRIO32			# No marks needed before, now need 1
		TC	NOVAC			# Schedule V51 display
		EBANK=	MARKSTAT
		2CADR	MKVB51			# Flash V51 to request replacement mark

		TC	RESUME

# Page 232
# PROGRAM DESCRIPTION MKVB51 AND MKVB50

# AUTHOR-BARNERT DATE-2-15-67 MOD-0
# PURPOSE	FLASH V51N70,V51N43, OR V51 TO REQUEST MARKING,
#		AND V50N25 R1=16 TO REQUEST TERMINATE MARKING.

# CALLING SEQUENCE	AS JOB WITHIN SXTMARK

# EXIT TO ENDMARK UPON RECEIPT OF V33, V34 CAUSES GOTOPOOH, ENTER
#		RECYCLES THE DISPLAY

# NOTE- SXTMARK AUTOMATICALLY CHANGES FROM CALLING MKVB51 TO MKVB50 WHEN
#		SUFFICIENT MARKS HAVE BEEN MADE, AND THE REVERSE WHEN A MARK
#		REJECT REDUCES THE NUMBER MADE BELOW THAT REQUIRED

# SUBROUTINES CALLED- BANKCALL, GOMARK2,GOODEND,ENDMARK,WAITLIST

# ALARM OR ABORT MODES - NONE

# ERASABLE USED-VERBREG,MARKSTAT,QPRET,DSPTEM1

# OUTPUT MARKSTAT=VAC ADDRESS

# 	 QPRET=	NO.MARKS

; ============================================================================
; MKVB51/MKVB50: DSKY Display Verb Routines for Marking
;
; MKVB51 displays V51 (Please Perform) to request astronaut to make a mark.
; MKVB50 displays V50 N25 R1=16 (Please Enter) to signal marking complete.
; System automatically switches between V51 and V50 based on mark count.
;
; Astronaut responses:
;   V33 (Proceed) - Confirms marking complete, exits to ENDMARK
;   V34 (Terminate) - Aborts marking, terminates sextant operation
;   ENTR - Recycles to initial mark display
;
; This interface allows the astronaut to control the optical navigation
; marking process through standard DSKY verb/noun displays.
; ============================================================================

; Clear DSKY display and flash V51 to request mark from astronaut.
; GOMARK4 waits for astronaut response and returns to one of three exit points.

MKVB51		TC	BANKCALL		# CLEAR DISPLAY FOR MARK VERB
		CADR	KLEENEX			# Clear DSKY
		CAF	VB51			# DISPLAY MARK VB51 (Please Perform)
		TC	BANKCALL
		CADR	GOMARK4			# Flash display, await response
		TCF	TERMSXT			# VB34-TERMINATE (astronaut aborts)
		TCF	ENTANSWR		# V33-PROCEED-MARKING DONE
		TCF	MKVB5X			# ENTER-RECYCLE TO INITIAL MARK DISPLAY

; V34 (Terminate): Astronaut requests abort of marking operation.
; Clear mark system flags and check major mode to determine proper exit.

TERMSXT		TC	CLEARMRK		# CLEAR MARK ACTIVITY.

		TC	CHECKMM			# Check current major mode
		MM	03			# Is it P03 (Optical Navigation)?
		TCF	+2			# No, use standard termination
		TC	TERMP03			# Yes, set P03 termination flag
		TC	POSTJUMP
		CADR	TERM52			# Jump to standard V52 termination

TERMP03		TC	UPFLAG			# Set flag for P03 termination
		ADRES	TRM03FLG

; V33 (Proceed): Astronaut confirms marking complete.
; Calculate number of marks actually made and store in QPRET.
; Format: QPRET = (final pointer - initial pointer) / 7

ENTANSWR	CAF	LOW9			# PUT VAC ADR IN MARKSTAT AND NO. OF
		MASK	MARKSTAT		# MARKS MADE IN QPRET BEFORE LEAVING
		TS	MARKSTAT		# SXTMARK (strip to VAC address only)
		COM				# Complement to prepare for subtraction
		INDEX	MARKSTAT
		AD	QPRET			# QPRET - VAC_base = pointer offset
# Page 233
		EXTEND
		BZMF	JAMIT			# NO MARKS MADE, SHOW IT IN QPRET, R53
		EXTEND				#	WILL PICK IT UP AND RECYCLE
		MP	BIT12			# THIS PUTS NUMBER MARKS-1 IN A
		AD	ONE			# Convert to actual count

; Store final mark count in QPRET for calling program (R21, R22, P51-P53, etc.)
; Schedule ENDMARKS task to interface with OPTSTALL and perform final cleanup.

JAMIT		INDEX	MARKSTAT		# STORE NO OF MARKS MADE
		TS	QPRET			# Result available to calling program
		INHINT				# SERVICE OPTSTALL INTERFACE WITH
		CAF	FIVE			# 50 millisecond delay
		TC	WAITLIST		# Schedule cleanup task
		EBANK=	MARKSTAT
		2CADR	ENDMARKS		# Delayed cleanup routine

		TC	ENDMARK			# KNOCKS DOWN MARKING FLAG + DOES ENDOFJOB

; ENDMARKS: Delayed cleanup task (50ms after mark completion).
; Calls GOODEND to signal successful completion to optical navigation programs.

ENDMARKS	CAF	ONE
		TC	IBNKCALL
		CADR	GOODEND			# Signal optical navigation complete

; MKVB5X: ENTER key response - Decide which display to show based on marks.
; Examines MARKSTAT to determine if more marks needed (show V51) or complete
; (show V50 N25). This routine handles display recycling after ENTER pressed.

MKVB5X		CAF	PRIO34			# Mask for mark count bits
		MASK	MARKSTAT		# RE-DISPLAY VB51 IF MORE MARKS WANTED
		CCS	A			# AND VB50 IF ALL IN
		TCF	MKVB51			# More marks needed, show V51
		
; MKVB50: Display V50 N25 (Please Enter) when all marks obtained.
; Sets R1=16 (octal) to indicate completion status on DSKY.

MKVB50		CAF	R1D1			# OCT 16 (completion indicator)
		TS	DSPTEM1			# Store in display temporary
		CAF	V50N25			# V50 N25 code
		TCF	MKVB51	+3		# Jump into V51 display logic

; Verb/Noun code constants for marking displays
V50N25		VN	5025			# Please Enter (marking complete)
VB51		VN	5100			# Please Perform (request mark)
OCT37		=	LOW5

# PROGRAM NAME - MARKIT				DATE- 19 SEPT 1967

# CALLING SEQUENCE
#	FROM MARKRUPT IF CHAN 16 BIT 6 = 1

# EXIT
#	RESUME

# INPUT
#	CDUCHKWD.  ALSO ALL INITIALIZATION FOR MARKCONT

# OUTPUT
#	MKT2T1,MKCDUX,MKCDUY,MKCDUZ,MKCDUS,MKCDUT

# ALARM EXIT
#	NONE

; ============================================================================
; MARKIT: Initial Mark Processing from Interrupt
;
; Called from MARKRUPT when astronaut presses MARK button (Channel 16 bit 6).
; Schedules MARKDIF task to verify spacecraft attitude stability before
; accepting the mark. The delay check (CDUCHKWD) determines if vehicle rates
; are low enough for valid optical measurement.
;
; This is the first step in capturing a sextant mark - the mark button press
; is detected, and a delayed verification task is scheduled to check that
; the spacecraft isn't rotating too fast for an accurate sighting.
; ============================================================================

MARKIT		CCS	CDUCHKWD		# Check delay requirement
		TCF	+3			# DELAY OF CDUCHKWD CS IF PNZ
# Page 234
		TCF	+2			# Skip if zero
		CAF	ZERO			# No delay required
		AD	ONE			# 10 MS IF NO CHECK (minimum delay)
		TC	WAITLIST		# Schedule verification task
		EBANK=	MRKBUF1
		2CADR	MARKDIF			# Delayed mark difference check

		TCF	RESUME			# Return from interrupt

		SETLOC	SXTMARK1
		BANK

		COUNT	20/SXTMK

# PROGRAM NAME - MARKDIF			DATE- 19 SEPT 1967

# CALLING SEQUENCE
#	WAITLIST FROM MARKIT

# EXIT
#	TASKOVER TO IBNKCALL TO MARKCONT

# INPUT
# 	OUTPUT FROM MARKIT, INPUT TO MARKCONT, CDUCHKWD

# OUTPUT
#	RUPTSTOR - RUPTSTOR+3,RUPTREG3,RUPTSTOR+5 - RUPTSTOR+6

# ALARM EXIT
#	ALARM AND TASKOVER

; ============================================================================
; MARKDIF: Mark Validation by Attitude Stability Check
;
; Called from WAITLIST after brief delay from MARKIT. Compares CDU angles
; captured at mark (MKCDUX/Y/Z) with current angles (CDUX/Y/Z). If spacecraft
; attitude has changed more than 3 bits (~0.044 degrees) in any axis, mark
; is rejected via alarm 00121 due to excessive vehicle motion.
;
; This ensures optical measurements are only accepted when spacecraft is
; stable. During star sightings or landmark tracking, crew must hold attitude
; steady. Excessive rotation between mark button press and this verification
; indicates invalid measurement due to vehicle drift or active maneuvering.
;
; For Apollo 11: Critical for P51/P52 IMU alignment star sightings and P22/P23
; landmark tracking for navigation updates during lunar orbit and translunar
; coast phases.
; ============================================================================

MARKDIF		CA	CDUCHKWD		# IF DELAY CHECK IS ZERO OR NEG,ACP MARK
		EXTEND				# (if no stability check required)
		BZMF	MKACPT			# Accept mark without checking
		CS	BIT1			# Start with -1 index
		TS	MKNDX			# SET INDEX -1 (for indexed access)
		CA	MKCDUX			# X axis mark angle
		TC	DIFCHK			# SEE IF VEHICLE RATE TOO MUCH AT MARK
		CA	MKCDUY			# Y axis mark angle
		TC	DIFCHK			# Check Y stability
		CA	MKCDUZ			# Z axis mark angle
		TC	DIFCHK			# Check Z stability

; All three axes within limits (difference < 3 bits each). Accept mark.

MKACPT		TC	IBNKCALL		# Mark validated
		CADR	MARKCONT		# MARK DATA OK, WHAT DO WE DO WITH IT

; ============================================================================
; DIFCHK: Difference Check Subroutine
;
; Compares marked IMU angle against current angle. Increments MKNDX (0,1,2)
; to index through CDUX, CDUY, CDUZ for comparison. Computes difference and
; checks if absolute value exceeds 3 bits. If exceeded, issues alarm 00121
; and rejects mark via TASKOVER. If within limit, returns to check next axis.
;
; The 3-bit threshold (0.044 degrees or 2.6 arc-minutes) represents maximum
; allowable spacecraft rotation during the ~10-20 millisecond mark capture
; window. This tight tolerance ensures sextant line-of-sight accuracy for
; navigation star sightings and lunar landmark measurements.
; ============================================================================

DIFCHK		INCR	MKNDX			# INCREMENT INDEX (now 0, 1, or 2)

		EXTEND				# Prepare for indexed operation
		INDEX	MKNDX			# Index selects CDUX, CDUY, or CDUZ
# Page 235
		MSU	CDUX			# GET MARK(ICDU) - CURRENT(ICDU)
		CCS	A			# Check sign and magnitude
		TCF	+4			# Positive, check threshold
		TC	Q			# Zero difference, return (stable)
		TCF	+2			# Negative, make positive
		TC	Q			# -0 case, return (stable)
		AD	NEG2			# SEE IF DIFFERENCE GREATER THAN 3 BITS
		EXTEND				# (subtract 2 from absolute difference)
		BZMF	-3			# NOT GREATER - return via TC Q above

; Difference exceeds 3 bits - spacecraft moved too much. Reject mark.

		TC	ALARM			# COUPLED WITH PROGRAM ALARM
		OCT	00121			# Alarm 00121: mark rejected, attitude unstable

		TCF	TASKOVER		# DO NOT ACCEPT (abort mark processing)
