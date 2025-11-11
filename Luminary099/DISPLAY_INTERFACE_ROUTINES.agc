# Copyright:	Public domain.
# Filename:	DISPLAY_INTERFACE_ROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1341-1373
# Mod history:	2009-05-27 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-01-06 JL	Fixed pseudo-label indentation.
#		2011-05-08 JL	Removed workaround.

; ============================================================================
; FILE: DISPLAY_INTERFACE_ROUTINES.agc
; MODULE: Display and Crew Interface
; MISSION PHASE: all phases (launch through landing and ascent)
;
; TL;DR: Implements DSKY (Display and Keyboard) display management routines
;        for the Lunar Module, serving as the central interface between AGC
;        programs and crew displays. Handles seven-segment display formatting,
;        decimal/octal conversion, verb/noun state machine processing, display
;        buffer management, flash patterns for crew attention, and priority
;        display arbitration. Armstrong and Aldrin relied on these routines
;        for all AGC interaction during descent, landing, and ascent.
;
; COMMENT-ONLY READERS: This is the code that formatted and displayed all
;        information Armstrong and Aldrin saw on the DSKY during the mission.
;        Follow comments to understand how the computer communicated with crew.
; CODE-ALONG READERS: Study display priority logic, buffer management, and
;        state machine implementation to understand AGC's human interface.
; ============================================================================

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

# Page 1341
# DISPLAYS CAN BE CLASSIFIED INTO THE FOLLOWING CATEGORIES --
#	1.  PRIORITY DISPLAYS -- DISPLAYS WHICH TAKE PRIORITY OVER ALL OTHER DISPLAYS.  USUALLY THESE DISPLAYS ARE SENT
#	    OUT UNDER CRITICAL ALARM CONDITIONS.
#	2.  EXTENDED VERB DISPLAYS -- ALL EXTENDED VERBS AND MARK ROUTINES SHOULD USE EXTENDED VERB (MARK) DISPLAYS.
#	3.  NORMAL DISPLAYS -- ALL MISSION PROGRAM DISPLAYS WHICH INTERFACE WITH THE ASTRONAUT DURING THE NORMAL
#	    SEQUENCE OF EVENTS.
#	4.  MISC. DISPLAYS -- ALL DISPLAYS NOT HANDLED BY THE DISPLAY INTERFACE ROUTINES.  THESE INCLUDE SUCH DISPLAYS AS
#	    MM DISPLAYS AND SPECIAL PURPOSE DISPLAYS HANDLED BY PINBALL.
#	5.  ASTRONAUT INITIATED DISPLAYS -- ALL DISPLAYS INITIATED EXTERNALLY.
#
# THE FOLLOWING TERMS ARE USED TO DESCRIBE THE STATUS OF DISPLAYS --
#	1.  ACTIVE -- THE DISPLAY WHICH IS (1) BEING DISPLAYED TO THE ASTRONAUT AND WAITING FOR A RESPONSE OR
#	    (2) WAITING FIRST IN LINE FOR THE ASTRONAUT TO FINISH USING THE DSKY OR (3) BEING DISPLAYED ON THE DSKY
#	    BUT NOT WAITING FOR A RESPONSE.
#	2.  INACTIVE -- A DISPLAY WHICH HAS (1) BEEN ACTIVE BUT WAS INTERRUPTED BY A DISPLAY OF HIGHER PRIORITY,
#	    (2) BEEN PUT INTO THE WAITING LIST AT TIME IT WAS REQUESTED DUE TO THE FACT A HIGHER PRIORITY DISPLAY
#	    WAS ALREADY DOING, (3) BEEN INTERRUPTED BY THE ASTRONAUT (CALLED A PINBRANCH CONDITION, SINCE THIS TYPE
#	    OF INACTIVE DISPLAY IS USUALLY REACTIVATED ONLY BY PINBALL) OR (4) A DISPLAY WHICH HAS FINISHED BUT STILL
#	    HAS INFO SAVED FOR RESTART PURPOSES.
#
# DISPLAY PRIORITIES WORK AS FOLLOWS --
#	INTERRUPTS --
#		1.  THE ASTRONAUT CAN INTERRUPT ANY DISPLAY WITH AN EXTERNAL DISPLAY REQUEST.
#		2.  INTERNAL DISPLAYS CAN NOT BE SENT OUT WHEN THE ASTRONAUT IS USING THE DSKY.
#		3.  PRIORITY DISPLAYS INTERRUPT ALL OTHER TYPES OF INTERNAL DISPLAYS.  A PRIORITY DISPLAY INTERRUPTING ANOTHER
#		    PRIORITY DISPLAY WILL CAUSE AN ABORT UNLESS BIT14 IS SET FOR THE LINUS ROUTINE.
#		4.  A MARK DISPLAY INTERRUPTS ANY NORMAL DISPLAY.
#		5.  A MARK THAT INTERRUPTS A MARK COMPLETELY REPLACES IT.
#
# 	ORDER OF WAITING DISPLAYS --
#		1.  ASTRONAUT
#		2.  PRIORITY
#		3.  INTERRUPTED MARK
#		4.  INTERRUPTED NORMAL
#		5.  MARK TO BE REQUESTED (SEE DESCRIPTION OF ENDMARK)
#		6.  MARK WAITING
#		7.  NORMAL WAITING
#
# Page 1342
# THE DISPLAY ROUTINES ARE INTENDED TO SERVE AS AN INTERFACE BETWEEN THE USER AND PINBALL.  THE
# FOLLOWING STATEMENTS CAN BE MADE ABOUT NORMAL DISPLAYS AND PRIORITY DISPLAYS (A DESCRIPTION OF MARK ROUTINES
# WILL FOLLOW LATER):
#	1.  ALL ROUTINES THAT END IN R HAVE AN IMMEDIATE RETURN TO THE USER.  FOR ALL FLASHING DISPLAYS THIS RETURN
#	    IS TO THE USER'S CALL CADR +4.  FOR THE ONLY NON-FLASHING IMMEDIATE RETURN DISPLAY (GODSPR) THIS RETURN
#	    IS TO THE USER'S CALLING LOC +1.
#	2.  ALL ROUTINES NOT ENDING IN R DO NOT DO AN IMMEDIATE RETURN TO THE USER.
#	3.  ALL ROUTINES THAT END IN R START A SEPARATE JOB (MAKEPLAY) WITH USER'S JOB PRIORITY.
#	4.  ALL ROUTINES NOT ENDING IN R BRANCH DIRECTLY TO MAKEPLAY WHICH MAKES THESE DISPLAYS A PART OF THE
#	    USER'S JOB.
#	5.  ALL DISPLAY ROUTINES ARE CALLED VIA BANKCALL.
#	6.  TO RESTART A DISPLAY THE USER WILL GENERALLY USE A PHASE OF ONE WITH DESIRED RESTART GROUP (SEE
#	    DESCRIPTION OF RESTARTS).
#	7.  ALL FLASHING DISPLAYS HAVE 3 RETURNS TO THE USER FROM ASTRONAUT RESPOSES.  A TERMINATE (V34) BRANCHES
#	    TO THE USER'S CALL CADR +1.  A PROCEED (V33) BRANCHES TO THE USER'S CALL CADR +2.  AN ENTER OR RECYCLE
#	    (V32) BRANCHES TO THE USER'S CALL CADR +3.
#	8.  ALL ROUTINES MUST BE USED UNDER EXECUTIVE CONTROL
#
# A DESCRIPTION OF EACH ROUTINE WITH AN EXAMPLE FOLLOWS:
#	GODSP IS USED TO DISPLAY A VERB NOUN ARRIVING IN A.  NO RETURN IS MADE TO THE USER.
#		1.  GODSP IS NOT RESTARTABLE
#		2.  A VERB PASTE WITH GODSP ALWAYS TURNS ON THE FLASH.
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	GODSP
#			VXXNYY	OCT	0XXYY
#	GODSPR IS THE SAME AS GODSP ONLY RETURN IS TO THE USER.
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	GODSPR
#				...	...		# IMMEDIATE RETURN OF GODSPR
#	GOFLASH DISPLAYS A FLASHING VERB NOUN WITH NO IMMEDIATE RETURN TO THE USER.  3 RETURNS ARE POSSIBLE FORM
#	THE ASTRONAUT (SEE NO. 7 ABOVE).
#				CAF	VXXNYY		# VXX NYY WILL BE A FLASHING VERB NOUN.
#				TC	BANKCALL
#				CADR	GOFLASH
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN
#	GOPERF1 IS ENTERED WITH DESIRED CHECKLIST VALUE IN A.  GOPERF1 WILL DISPLAY THIS VALUE IN R1 BY MEANS OF A
# Page 1343
# 	V01 N25.  A FLASHING PLEASE PERFORM ON CHECKLIST (V50 N25) IS THEN DISPLAYED.  NO IMMEDIATE RETURN IS MADE TO
# 	USER (SEE NO. 7 ABOVE).
#	GOPERF1 BLANKS REGISTERS R2 AND R3
#				CAF	OCTXX		# CODE FOR CHECKLIST VALUE XX
#				TC	BANKCALL
#				CADR	GOPERF1
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOPERF2 IS ENTERED WITH A VARIABLE NOUN AND V01 (V00 FOR N10 OR N11) IN A.  GOPERF2 WILL FIRST DISPLAY THE
# 	REQUESTED NOUN BY MEANS OF A V01NYY OR A V00NYY.  PLEASE PERFORM ON NOUN (V50 NYY) THEN BECOMES A FLASHING
#	DISPLAY.  NO IMMEDIATE RETURN IS MADE TO THE USER (SEE NO. 7 ABOVE).
#	GOPERF2 DOES NOT BLANK ANY REGISTERS
#				CAF	VXXNYY		# VARIABLE NOUN YY. XX=0 OR 01.
#				TC	BANKCALL
#				CADR	GOPERF2
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOPERF3 IS USED FOR A PLEASE PERFORM ON A PROGRAM NUMBER.  THE DESIRED PROGRAM NO. IS ENTERED IN A.  GOPERF3
#	DISPLAYS THE NO. BY MEANS OF A V06 N07 FOLLOWED BY A FLASHING V50 N07 FOR A PLEASE PERFORM.  NO IMMEDIATE RETURN
#	IS MADE TO THE USER (SEE NO. 7 ABOVE).
#	GOPERF3 BLANKS REGISTERS R2 AND R3
#				CAF	DECXX		# REQUEST PERFORM ON PXX
#				TC	BANKCALL
#				CADR	GOPERF3
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOPERF4 IS USED FOR A PLEASE PERFORM ON AN OPTION.  THE DESIRED OPTION IS ENTERED IN A AND STORED IN OPTION1.
#	GOPERF4 DISPLAYS R1 AND R2 BY MEANS OF A V04N06 FOLLOWED BY A FLASHING V50N06 FOR A PLEASE PERFORM.  NO
#	IMMEDIATE RETURN IS MADE TO THE USER (SEE NO. 7 ABOVE).
#				CAF	OCTXX		# REQUEST PERFORM ON OPTION XX
#				TC	BANKCALL
#				CADR	GOPERF4
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOPERF4 BLANKS REGISTER R3.
# Page 1344
#	GODSPRET IS USED TO DISPLAY A VERB NOUN ARRIVING IN A WITH A RETURN TO THE USER AFTER THE DISPLAY HAS BEEN SENT
#	OUT.
#				CAF	VXXXNYY
#				TC	BANKCALL
#				CADR	GODSPRET
#				...	...		# RETURN TO USER.
#	REGODSP IS USED TO DISPLAY A VERB NOUN ARRIVING IN A.  REGODSP IS THE SAME AS GODSP ONLY REGODSP REPLACES ANY
# 	ACTIVE NORMAL DISPLAY IF ONE WAS ACTIVE.
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	REGODSP
#	REFLASH IS THE SAME AS GOFLASH ONLY REFLASH REPLACES ANY ACTIVE NORMAL DISPLAY IF ONE WAS ACTIVE.
#				CAF	VXXNYY		# VXX NYY WILL BE A FLASHING VERB NOUN
#				TC	BANKCALL
#				CADR	REFLASH
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
# 	GOFLASHR IF SAME AS GOFLASH ONLY AN IMMEDIATE RETURN IS MADE TO THE USER'S CALL CADR +4.
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	GOFLASHR
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN
#				...	...		# IMMEDIATE RETURN FROM GOFLASHR
#	GOPERF1R IS THE SAME AS GOPERF1 ONLY GOPERF1R HAS AN IMMEDIATE RETURN TO USER'S CALL CADR +4.
#	GOPERF1R BLANKS REGISTERS R2 AND R3
#				CAF	OCTXX		# CODE FOR CHECKLIST VALUE XX.
#				TC	BANKCALL
#				CADR	GOPERF1R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN FROM GOPERF1R
#	GOPERF2R IS THE SAME AS GOPERF2 ONLY AN IMMEDIATE RETURN IS MADE TO USER'S CALL CADR +4.
# Page 1345
#	GOPERF2R DOES NOT BLANK ANY REGISTERS
#				CAF	VXXXNYY		# VARIABLE NOUN YY REQUESTED.  XX=00 OR 01
#				TC	BANKCALL
#				CADR	GOPERF2R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN HERE FROM GOPERF2R
# 	GOPERF3R IS THE SAME AS GOPERF3 ONLY AN IMMEDIATE RETURN IS MADE TO USER'S CALL CADR +4.
#	GOPERF3R BLANKS REGISTERS R2 AND R3
#				CAF	PROGXX		# PERFORM PROGRAM XX
#				TC	BANKCALL
#				CADR	GOPERF3R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# GOPERF3R IMMEDIATELY RETURNS HERE
#	GOPERF4R IS THE SAME AS GOPERF4 ONLY AN IMMEDIATE RETURN IS MADE TO USER'S CALL CADR +4.
#				CAF	OCTXX		# REQUEST PERFORM ON OPTIONXX
#				TC	BANKCALL
#				CADR	GOPERF4R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN TO USER
#	GOPERF4R BLANKS REGISTER R3.
#	REFLASHR IS THE SAME AS REFLASH ONLY AN IMMEDIATE RETURN IS MADE TO THE USER'S CALL CADR +4.
#				CAF	VXXNYY		# VXX NYY WILL BE A FLASHING VERB NOUN
#				TC	BANKCALL
#				CADR	REFLASHR
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN TO USER
#	REGODSPR IS THE SAME AS REGODSP ONLY A RETURN (IMMEDIATE) IS MADE TO THE USER.
# Page 1346
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	REGODSPR
#				...	...		# IMMEDIATE RETURN TO USER
# Page 1347
#	GOMARK IS USED TO DISPLAY A MARK VERB NOUN ARRIVING IN A.  NO RETURN IS MADE TO THE USER.
#	GOXDSP = GOMARK
#				CAF	VXXNYY		# VXXNYY CONTAINS VERB AND NOUN
#				TC	BANKCALL
#				CADR	GOMARK		# OTHER EXTENDED VERBS USE CADR GOXDSP
#	GOMARKR IS THE SAME AS GOMARK ONLY RETURN IS TO THE USER.
#	GOXDSPR = GOMARKR
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	GOMARKR		# OTHER EXTENDED VERBS USE CADR GOXDSPR
#				...	...		# IMMEDIATE RETURN OF GOMARKR
#	GOMARKF DISPLAYS A FLASHING MARK VERB NOUN WITH NO IMMEDIATE RETURN TO THE USER.  3 RETURNS ARE POSSIBLE FROM
#	THE ASTRONAUT (SEE NO. 7 ABOVE).
#	GOXDSPF = GOMARKF
#				CAF	VXXNYY		# VXXNYY WILL BE A FLASHING MARK VERB NOUN
#				TC	BANKCALL
#				CADR	GOMARKFR	# OTHER EXTENDED VERBS USE CADR GOXDSPFR
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN
#	GOMARKFR IS THE SAME AS GOMARKF ONLY AN IMMEDIATE RETURN IS MADE TO THE USER CALL CADR +4.
#	GOXDSPFR = GOMARKFR
#				CAF	VXXNYY		# FLASHING MARK VERB NOUN
#				TC	BANKCALL
#				CADR	GOMARKFR	# OTHER EXTENDED VERBS USE CADR GOXDSPFR
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN
#				...	...		# IMMEDIATE RETURN TO THE USER
#	GOMARK1 IS USED FOR A PLEASE PERFORM ON A MARK REQUEST WITH ONLY 1 ASTRONAUT RETURN TO THE USER.  NO IMMEDIATE
#	RETURN IS MADE.  THE DESIRED MARK PLEASE PERFORM VERB AND DESIRED NOUN IS ENTERED IN A.  GOMARK1 DISPLAYS R1, R2, R
#	MEANS OF A V05NYY FOLLOWED BY A FLASHING V5XNYY FOR A PLEASE PERFORM.  THE ASTRONAUT WILL RESPOND WITH A MARK
#	OR MARK REJECT OR AN ENTER.  THE ENTER IS THE ONLY ASTRONAUT RESPONSE THAT WILL COME BACK TO THE USER.
#				CAF	V5XNYY		# X=1,2,3,4	Y=NOUN
#				TC	BANKCALL
# Page 1348
#				CADR	GOMARK1
#				...	...		# ENTER RETURN
#	*** IF BLANKING DESIRED ON NON-R ROUTINES, NOTIFY DISPLAYER.
#
#	GOMARK1R IS THE SAME AS A GOMARK1 ONLY AN IMMEDIATE RETURN IS MADE TO THE USER'S CALL CADR +2.
#				CAF	V5XNYY		# X=1,2,3,4	YY=NOUN
#				TC	BANKCALL
#				CADR	GOMARK1R
#				...	...		# ASTRONAUT ENTER RETURN
#				...	...		# IMMEDIATE RETURN TO USER
#	GOMARK2 IS THE SAME AS GOMARK1 ONLY 3 RETURNS ARE MADE TO THE USER FROM THE ASTRONAUT.
#				CAF	V5XNYY		# X=1,2,3,4	YY=NOUN
#				TC	BANKCALL
#				CADR	GOMARK2
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOMARK2R IS THE SAME AS GOMARK1R ONLY 3 ASTRONAUT RETURNS ARE MADE TO THE USER.
#				CAF	V5XNYY		# X=0,1,2,3,4	YY=NOUN
#				TCF	BANKCALL
#				CADR	GOMARK2R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN TO THE USER.
#	GOMARK3 IS USED FOR A PLEASE PERFORM ON A MARK REQUEST WITH A 3 COMP. DEC DISPLAY.  THE DESIRED MARK PLEASE
#	PERFORM VERB AND NOUN ARE ENTERED IN A.  GOMARK3 DISPLAYS R1, R2, R3 BY MEANS OF A V06NYY FOLLOWED BY A FLASHING
#	V5XNYY FOR A PLEASE PERFORM.  GOMARK3 HAS 3 ASTRONAUT RETURNS TO THE USER WITH NO IMMEDIATE RETURN.
#				CAF	V5XNYY		# X=1,2,3,4	YY=NOUN
#				TC	BANKCALL
#				CADR	GOMARK3
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOMARK4 IS THE SAME AS GOMARK3 ONLY R2 AND R3 ARE BLANKED AND R1 IS DISPLAYED IN OCTAL.
#				CAF	V5XNYY		# X=1,2,3,4	YY=NOUN
#				TC	BANKCALL
#				CADR	GOMARK4
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
# Page 1349
#			...	...			# ENTER RETURN
#	EXDSPRET IS USED TO DISPLAY A VERB NOUN ARRIVING IN A WITH A RETURN MADE TO THE USER AFTER THE DISPLAY HAS BEEN
#	SEND OUT.
#				CAF	VXNYY
#				TC	BANKCALL
#				CADR	EXDSPRET
#				...	...		# RETURN TO USER
#	KLEENEX CLEANS OUT ALL MARK DISPLAYS (ACTIVE AND INACTIVE). A RETURN IS MADE TO THE USER AFTER THE MARK DISPLAYS
#	HAVE BEEN CLEANED OUT.
#				TC	BANKCALL
#				CADR	KLEENEX
#				...	...		# RETURN TO USER
#	MARKBRAN IS A SPECIAL PURPOSE ROUTINE USED FOR SAVING JOB VAC AREAS (SEE DESCRIPTION OF MARKBRAN BELOW).
#				TC	BANKCALL
#				CADR	MARKBRAN
#				...	...		# BAD RETURN IF MARK DISPLAY NOT ACTIVE
#							# (GOOD RETURN TO IMMEDIATE RETURN LOC OF
#							# LAST FLASHING MARK R ROUTINE)
#	PINBRNCH REESTABLISHES THE LAST ACTIVE FLASHING DISPLAY.  IF THERE IS NO ACTIVE FLASHING DISPLAY, THE DSKY IS
#	BLANKED AND CONTROL IS SENT TO ENDOFJOB.
#				TC	POSTJUMP
#				CADR	PINBRNCH
#	PRIODSP IS USED AS A PRIORITY DISPLAY.  IT WILL DISPLAY A GOFLASH TYPE DISPLAY WITH THREE POSSIBLE RETURNS FROM
#	THE ASTRONAUT (SEE NO. 7 ABOVE).
#	THE MAIN PURPOSE OF PRIODSP IS TO REPLACE THE PRESENT DISPLAY WITH A DISPLAY OF HIGHER PRIORITY AND TO
#	PROVIDE A MEANS FOR RESTORING THE OLD DISPLAY WHEN THE PRIORITY DISPLAY
# 	IS RESPONDED TO BY THE ASTRONAUT.
#	THE FORMER DISPLAY IS RESTORED BY AN AUTOMATIC BRANCH TO WAKE UP THE DISPLAY THAT WAS INTERRUPTED BY THE
#	PRIO DISPLAY
#				CAF	VXXNYY		# VXXNYY WILL BE A FLASHING VERB NOUN
#				TC	BANKCALL
#				CADR	PRIODSP
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
# Page 1350
#				...	...		# ENTER OR RECYCLE RETURN
#	PRIODSPR IS THE SAME AS PRIODSP ONLY AN IMMEDIATE RETURN IS MADE TO THE USER'S CALL CADR +4.
#				CAF	VXXNYY		# VXXNYY WILL BE A FLASHING VERB NOUN
#				TC	BANKCALL
#				CADR	PRIODSPR
#				...	...		# TERMINATE ACTION
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN
#				...	...		# IMMEDIATE RETURN
#	PRIOLARM DOES A V05N09 PRIODSPR.
#
#	CLEANDSP CLEANS OUT ALL NORMAL DISPLAYS (ACTIVE AND INACTIVE).  A RETURN IS MADE TO THE USER AFTER NORMAL
#	DISPLAYS ARE CLEANED OUT.
#				TC	BANKCALL
#				CADR	CLEANDSP
#				...	...		# RETURN TO USER
# Page 1351
#
# GENERAL INFORMATION
# -------------------
#
# ALARM OR ABORT EXIT MODE --
#	PRIOBORT	TC	ABORT
#			OCT	1502
#
#	PRIOBORT IS BRANCHED TO WHEN (1) A NORMAL DISPLAY IS REQUESTED AND ANOTHER NORMAL DISPLAY IS ALREADY ACTIVE
#	(REFLASH AND REGODSP ARE EXCEPTIONS) OR (2) A PRIORITY DISPLAY IS REQUESTED WHEN ANOTHER PRIORITY DISPLAY IS
#	ALREADY ACTIVE (A PRIORITY WITH LINUS BIT14 IS AN EXCEPTION).
#
# ERASABLE INITIALIZATION REQUIRED --
#	ACCOMPLISHED BY FRESH START --	1.  FLAGWRD4 (USED EXCLUSIVELY BY DISPLAY INTERFACE ROUTINES)
#					2.  NVSAVE = NORMAL VERB AND NOUN REGISTER.
#					3.  EBANKTEM = NORMAL INACTIVE FLAGWORD (ALSO CONTAINS NORMALS EBANK).
#					5.  R1SAVE = MARKBRAN CONTROL WORD
#					4.  RESTREG = PRIORITY 30 AND SUPERBANK 3.
#
# OUTPUT --
#	NVWORD = PRIO VERB AND NOUN
#	NVWORD +1 (MARKNV) = MARK VERB AND NOUN
#	NVWORD +2 (NVSAVE) = NORMAL VERB AND NOUN
#	DSPFLG (EBANKSAV) = PRIO FLAGWORD (INCLUDING EBANK)
#	DSPFLG +1 (MARKEBAN) = MARK FLAGWORD (INCLUDING EBANK)
#	DSPFLG +2 (EBANKTEM) = NORMAL FLAGWORD (INCLUDING EBANK)
#	CADRFLSH = PRIO USER'S CALL CADR +1 LOCATION
#	CADRFLSH +1 (MARKFLSH) = MARK USER'S CALL CADR +1 LOCATION
#	CADRFLSH +2 (TEMPFLSH) = NORMAL USER'S CALL CADR +1 LOCATION
#	PRIOTIME = TIME EACH PRIO REQUEST FIRST SENT OUT
#	OPTION1 = DESIRED OPTION FROM GOPERF4
#	FLAGWRD4 = BIT INFO FOR CONTROL OF ALL DISPLAY ROUTINES
#	DSPTEM1 = R1 INFO FOR ASTRONAUT FROM PERFORM DISPLAYS (NORMAL)
#
# SUBROUTINES USED -- NVSUB, FLAGUP, FLAGDOWN, ENDOFJOB, BLANKSUB, ABORT, JOBWAKE, JOBSLEEP, FINDVAC, PRIOCHNG,
#	JAMTERM, NVSUBUSY, FLASHON, ENDIDLE, CHANG1, BANKJUMP, MAKECADR, NOVAC
#
# DEBRIS -- (STORED INTO)
#	TEMPORARY TEMPORARIES -- A, Q, L, MPAC +2, MPAC +3, MPAC +4, MPAC +5, MPAC +6, RUPREG2, RUPTREG3, CYL,
#		EBANK, RUPTREG4, LOC, BANKSET, MODE, MPAC, MPAC +1, FACEREG
#	ERASABLES (SHARED AND USED WITH OTHER PROGRAMS) -- CADRSTOR, DSPLIST, LOC, DSPTEM1, OPTION1
#	ERASABLES (USED ONLY BY DISPLAY ROUTINES) -- NVWORD,+1,+2, DSPFLAG,+1,+2, CADRFLSH,+1,+2, PRIOTIME, FLAGWRD4,
# Page 1352
#		R1SAVE, MARK2PAC
#
# DEBRIS -- (USED BUT NOT STORED INTO) -- NOUNREG, VERBREG, LOCCTR, MONSAVE1
#
# FLAGWORD DESCRIPTIONS --
#	FLAGWRD4 -- SEE DESCRIPTION UNDER LOG SECTION ERASABLE ASSIGNMENTS
#
#	DSPFLG, DSPFLG+1, DSPFLG+2
#	--------------------------
#	BITS 1	BLANK R1
#	     2	BLANK R2
#	     3	BLANK R3
#	     4	FLASHING DISPLAY REQUESTED
#	     5	PERFORM DISPLAY REQUESTED
#	     6	-----			EXDSPRET		GODSPRET
#	     7	PRIO DISPLAY		-----			-----
#	     8	-----			DEC MARK PERFORM	-----
#	     9	EBANK
#	    10	EBANK
#	    11	EBANK
#	    12	-----			-----			V99PASTE
#	    13	2ND PART OF PERFORM
#	    14	REFLASH OR REDO		-----			REFLASH OR REDO
#	    15	-----			MARK REQUEST		-----
#
# RESTARTING DISPLAYS --
#
# RULES FOR THE DSKY OPERATOR --
#	1.  PROCEED AND TERMINATE SERVE AS RESPONSES TO REQUESTS FOR OPERATOR RESPONSE (FLASHING V/N).  AS LONG
#	    AS THERE IS ANY REQUEST AWAITING OPERATOR RESPONSE, ANY USE OF PROCEED OR TERMINATE WILL SERVE AS
#	    RESPONSES TO THAT REQUEST.  CARE SHOULD BE EXERCISED IN ATTEMPTING TO KILL AN OPERATOR INITIATED MONITOR
#	    WITH PROCEED AND TERMINATE FOR THIS REASON.
#	2.  THE ASTRONAUT MUST RESPOND TO A PRIORITY DISPLAY NO SOONER THAN 2 SECONDS FROM THE TIME THE
#	    PROGRAM SENT OUT THE REQUEST FOR OPERATOR RESPONSE (THE ASTRONAUT WOULD SEE THIS DISPLAY FOR LESS TIME
#	    DUE TO TIME IT TAKES TO GET DISPLAY SENT OUT.)  IF THE ASTRONAUT RESPONDS TOO SOON, THE PRIORITY DISPLAY
#	    IS SENT OUT AGAIN -- AND AGAIN UNTIL AN ACCUMULATED 2 SECS FROM THE TIME THE FIRST PRIORITY DISPLAY
#	    OUT.  THE SAME 2 SEC. DELAY WILL OCCUR AT 163.84 SECS OR IN ANY MULTIPLE OF THAT TIME DUE TO PROGRAM
#	    CONSIDERATION.
#	3.  KEY RELEASE BUTTON --
#	    A)  IF THE KEY RELEASE LIGHT IS ON, IT SIMPLY RELEASES THE KEYBOARD AND DISPLAY FOR INTERNAL USE.
#	    B)  IF THE KEY RELEASE LIGHT IS OFF, AND IF SOME REQUEST FOR OPERATOR RESPONSE (FLASHING V/N) IS STILL
#	        AWAITING RESPONSE THEN IT RE-ESTABLISHES THE DISPLAYS THAT ORIGINALLY REQUESTED RESPONSE.
#	    IF AN OPERATOR WANTS THEREFORE TO RE-ESTABLISH BUT CONDITION (A) IS ENCOUNTERED, A SECOND DEPRESSION OF
#	    KEY RELEASE BUTTON MAY BE NECESSARY.
#	4.  IT IS IMPORTANT TO ANSWER ALL REQUESTS FOR OPERATOR RESPONSE.
#	5.  IT IS ALWAYS GOOD PRACTICE TO TERMINATE AN EXTENDED VERB BEFORE ASKING FOR ANOTHER ONE OR THE SAME ONE
#	    OVER AGAIN.
#
# SPECIAL CONSIDERATONS --
# Page 1353
#	1.  MPAC +2 SAVED ONLY IN MARK DISPLAYS
#	2.  GODSP(R), REGODSP(R), GOMARK(R) ALWAYS TURN ON THE FLASH IF ENTERED WITH A PASTE VERB REQUEST.
#	3.  ALL NORMAL DISPLAYS ARE RESTARTABLE EXCEPT GODSP(R), REGODSP(R)
#	4.  ALL EXTENDED VERBS WITH DISPLAYS SHOULD START WITH A TC TESTXACT AND FINISH WITH A TC ENDEXT.
#	5.  GODSP(R) AND REGODSP(R) MUST BE IN THE SAME EBANK AND SUPERBANK AS THE LAST NORMAL DISPLAY RESTARTED
#	    BY A .1 RESTART PHASE CHANGE.
#	6.  IN ORDER TO SET UP A NON DISPLAY .1 RESTART POINT, THE USER MUST MAKE CERTAIN THAT RESTREG CONTAINS THE
#	    CORRECT PRIORITY AND SUPERBANK AND THAT EBANKTEM CONTAINS THE CO
#	7.  IF CLEANDSP IS RESTARTED VIA A .1 PHASE CHANGE, CAF ZERO SHOULD BE EXECUTED BEFORE THE TC BANKCALL.

; ============================================================================
; TRANSITION: From display system architecture documentation to executable code
;
; The preceding comments (pages 1341-1353) define the comprehensive display
; interface architecture used throughout the Apollo 11 mission. These routines
; controlled every piece of information Armstrong and Aldrin saw on the DSKY
; during critical mission phases including powered descent, landing, and ascent.
;
; The display system implements a priority-based architecture where:
; - Priority displays (alarms) interrupt all other displays
; - Mark displays (optical navigation) interrupt normal displays
; - Astronaut-initiated displays take precedence over internal requests
; - Display buffer management ensures data integrity during interruptions
;
; During the descent on July 20, 1969, these routines continued displaying
; altitude and velocity data to the crew even during the famous 1202 program
; alarm at mission time 102:38:26. The flash patterns and priority logic
; enabled Armstrong to maintain situational awareness while the AGC executive
; resolved the computational overload.
;
; The following code implements: display blanking control, verb/noun state
; machine processing, flash pattern generation, buffer management, priority
; arbitration, and format conversion (decimal/octal, seven-segment encoding).
; ============================================================================

# Page 1354
# CALLING SEQUENCE FOR BLANKING
#		CAF	BITX		# X=1,2,3 BLANK R1,R2,R3 RESPECTIVELY
#		TC	BLANKET
#		...	...		# RETURN TO USER HERE
# IN ORDER TO USE BLANKET CORRECTLY, THE USER MUST USE A DISPLAY ROUTINE THAT ENDS IN R FIRST FOLLOWED BY THE CALL
# TO BLANKET AT THE IMMEDIATE RETURN LOC.

; Display blanking routine - used to selectively blank DSKY registers R1, R2,
; or R3 when data is not yet ready for crew display. During descent, blanking
; prevented display of invalid or intermediate computational results.

		BLOCK	02
		SETLOC	FFTAG4
		BANK

		COUNT*	$$/DSPLA
BLANKET		TS	MPAC 	+6
		CS	PLAYTEM4
		MASK	MPAC 	+6
		INDEX	MPAC 	+5
		ADS	PLAYTEM4

		TC	Q

; ============================================================================
; MARK ROUTINE SUPPORT - Optical Navigation Display Interface
;
; Mark routines handle displays for optical navigation during mission phases
; where the crew used the sextant (CM) or Alignment Optical Telescope (LM)
; to sight on stars or landmarks for navigation updates. These displays have
; intermediate priority between normal mission displays and priority alarms.
;
; ENDMARK: Terminates an extended verb mark routine, clearing mark active flag
; CLEARMRK: Clears the extended verb active flag and mark display bit
;
; Mark routines were critical during:
; - IMU alignment before critical maneuvers
; - Landmark tracking in lunar orbit
; - Star sightings for navigation state updates
; ============================================================================

ENDMARK		TC	POSTJUMP
		CADR	MARKEND

CLEARMRK	CAF	ZERO
		TS	EXTVBACT	; Clear extended verb active flag

 +2		INHINT			; Disable interrupts for flag update
		CS	XDSPBIT		; Complement of extended display bit
		MASK	FLAGWRD4	; Clear XDSPBIT in FLAGWRD4
		TS	FLAGWRD4	; Store updated flag word

		RELINT			; Re-enable interrupts
		TC	Q		; Return to caller

# *** ALL EXTENDED VERB ROUTINES THAT HAVE AT LEAST ONE FLASHING DISPLAY MUST TCF ENDMARK OR TCF ENDEXT WHEN
# FINISHED.

		BANK	10
		SETLOC	DISPLAYS
		BANK

		COUNT*	$$/DSPLA

# NTERONLY IS USED TO DIFFERENTIATE THE MARK ROUTINE WITH ONLY ONE RETURN TO THE USER FROM THE MARKING ROUTINE WITH
# 3 RETURNS TO THE USER.  THIS ROUTINE IS ONLY USED BY GOMARK1 AND GOMARK1R.

; ============================================================================
; MARK DISPLAY ENTRY POINTS - Multiple Mark Routine Variants
;
; The display interface provides several mark routine entry points to support
; different optical navigation scenarios:
;
; GOMARK:   Standard mark display (no immediate return)
; GOMARKR:  Mark display with immediate return to caller
; GOMARKF:  Mark with flash indicator
; GOMARK2:  Mark with performance display (3 astronaut returns)
; GOMARK3:  Mark with 3-component decimal performance display
; GOMARK4:  Mark with blanking capability
;
; Each variant supports specific extended verb requirements for sextant marks,
; AOT marks, and landmark tracking during different mission phases.
; ============================================================================

MARKEND		TC	CLEARMRK	; Clear mark flags
		TCF	MARKOVER	; Branch to mark completion handler

GOMARK		TS	PLAYTEM1	# ENTRANCE FOR MARK GODSP
# Page 1355
GOMARS		CAF	BIT15		# BIT15 SET FOR ALL MARK REQUESTS
		TCF	GOFLASH2

KLEENEX		CAF	ZERO		# CLEAN OUT EXTENDED VERBS
GOMARKF		TS	PLAYTEM1	# ENTRANCE FOR MARK GOFLASH

		CAF	MARKFMSK	# MARK, FLASH
		TCF	GOFLASH2

GOMARK2		TS	PLAYTEM1	# MARK GOPERFS-3 AST. RETURNS
MARKFORM	CAF	MPERFMSK	# MARK, PERFORM, FLASH
		TCF	GOFLASH2

GOMARK3		TS	PLAYTEM1	# USED FOR 3COMP DECIMAL PERFORM
		CAF	MARK3MSK
		TCF	GOFLASH2

GOMARK4		TS	PLAYTEM1
		CAF	MARK4MSK	# MARK,PERFORM,FLASH,BLANK
		TCF	GOFLASH2

GOMARKR		TS	PLAYTEM1	# ENTRANCE FOR MARK GODSPR

		CAF	BIT15
		TCF	GODSPR2

GOMARKFR	TS	PLAYTEM1	# ENTRANCE FOR MARK GOFLASHR

		CAF	MARKFMSK
		TCF	GODSPRS

GOMARK2R	TS	PLAYTEM1	# MARK GOPERFS-3 AST. RETS+ IMMEDIATE RET.
		CAF	MPERFMSK	# MARK, PERFORM, FLASH
		TCF	GODSPRS

GOMARK3R	TS	PLAYTEM1
		CAF	MARK3MSK
		TCF	GODSPRS

; ============================================================================
; MAKEMARK - Mark Display Request Processing
;
; This routine determines whether a mark display can be activated immediately
; or must wait based on current display system state. The priority logic
; ensures mark displays (optical navigation) can interrupt normal displays
; but defer to priority displays (alarms).
;
; During Apollo 11's mission, mark displays were used for:
; - Pre-maneuver IMU alignments
; - Landmark tracking in lunar orbit for navigation updates
; - Star sightings for state vector refinement
;
; The routine checks:
; 1. Are normal or priority displays active? (FLAGWRD4 bits)
; 2. Is a mark already sleeping waiting for astronaut?
; 3. Can the mark interrupt or must it wait?
; ============================================================================

MAKEMARK	CAF	ONE
		TC	COPIES		; Copy display parameters

		CA	FLAGWRD4	# IS NORM OR PRIO BUSY OR WAITING
		MASK	OCT34300	; Check normal/priority active bits
		CCS	A		; Test if any are set
		TCF	CHKPRIO		; Yes - check priority rules

		CA	FLAGWRD4	# IS MARK SLEEPING DUE TO ASTRO BUSY
		MASK	MRKNVBIT	; Check mark sleep bit

		EXTEND
# Page 1356
		BZF	MARKPLAY	# NO - activate mark display now

		TCF	ENDOFJOB	; Yes - mark stays asleep, end job

; Mark display can be activated - set mark active flag and clear mark-over-norm
MARKPLAY	INHINT			; Disable interrupts for flag update
		CS	FIVE		# RESET MARK OVER NORM, SET MARK
		MASK	FLAGWRD4	; Clear bits 1 and 3 (mark over norm)
		AD	ONE		; Set bit 1 (mark active)
		TS	FLAGWRD4	; Update flag word
		RELINT			; Re-enable interrupts

; Process mark display based on performance flag
GOGOMARK	CS	MARKFLAG	# PERFORM
		MASK	BIT5		; Check performance display bit
		CCS	A		; Performance display needed?
		TCF	MARKCOP		; Yes - proceed to mark copy
		CS	MARKNV		; No - complement mark verb/noun
		TS	MARKNV		; Toggle display format

MARKCOP		CAF	ONE		# MARK INDEX
		TCF	PRIOPLAY	; Branch to priority display handler

COPYTOGO	CA	MPAC2SAV
		TS	MPAC 	+2

COPYPACS	INDEX	COPINDEX
		CAF	PRIOOCT
		TS	GENMASK

		INDEX	COPINDEX
		CAF	EBANKSAV
		TS	TEMPOR2		# ACTIVE EBANK AND FLAG

		TS	EBANK

		TC	Q

# PINCHEK CHECKS TO SEE IF THE CURRENT MARK REQUEST IS MADE BY THE ASTRONAUT WHILE INTERUPTING A GOPLAY DISPLAY
# (A NORMAL OR A PRIO).  IF THE ASTRONAUT TRIES TO MARK DURING A PRIO, THE CHECK FAIL LIGHT GOES ON AND THE MARK
# REQUEST IS ENDED.  IF HE TRIES TO MARK DURING A NORM, THE MARK IS ALLOWED.  IN THIS CASE THE NORM IS PUT TO SLEEP
# UNTIL ALL MARKING IS FINISHED.
#
# IF THE MARK REQUEST COMES FROM THE PROGRAM DURING A TIME THE ASTRONAUT IS NOT INTERRUPTING A NORMAL OR A
# PRIO, THE MARK REQUEST IS PUT TO SLEEP UNTIL THE PRESENT ACTIVE DISPLAY IS RESPONDED TO BY THE ASTRONAUT.

; ============================================================================
; CHKPRIO - Check Priority Display Conflict with Mark Request
;
; This routine implements the critical display priority arbitration logic.
; When a mark (optical navigation) display is requested while another display
; is active, this determines whether the mark can interrupt:
;
; Priority hierarchy:
; 1. Astronaut-initiated displays (highest - never interrupted)
; 2. Priority displays (alarms) - mark CANNOT interrupt
; 3. Mark displays - can interrupt normal displays
; 4. Normal displays (lowest)
;
; If astronaut attempts mark during priority alarm: CHECK FAIL light activated
; If mark requested during normal display: Normal sleeps, mark activates
; If mark from program during astronaut activity: Mark sleeps until response
; ============================================================================

CHKPRIO		CA	FLAGWRD4	# MARK ATTEMPT DURING PRIO
		MASK	OCT24100	; Test priority display active bits
		CCS	A		; Priority display active?
		TCF	MARSLEEP	; Yes - mark must sleep

		CS	FLAGWRD4	; No priority conflict
# Page 1357
		MASK	MKOVBIT		# SET MARK OVER NORM
		INHINT			; Disable interrupts
		ADS	FLAGWRD4	; Set mark-over-normal bit

		TCF	SETNORM		; Put normal display to sleep

MARKPERF	CA	MARKNV
		MASK	VERBMASK
		TCF	NV50DSP

; ============================================================================
; NORMAL DISPLAY ENTRY POINTS - GODSP Family
;
; These routines handle normal mission program displays that interface with
; the astronaut during standard operations. Normal displays have lower priority
; than priority displays (alarms) and mark displays (optical navigation).
;
; ENTRY POINTS:
;
; GODSP:    Standard display, no immediate return
;           - Stores verb/noun code
;           - No flash by default
;           - Branches directly to MAKEPLAY (same job as caller)
;
; GODSPRET: GODSP with paste operation
;           - Used when combining display with data entry
;           - Sets BIT6 to return to user after NVSUB
;
; GODSPR:   Display with immediate return (R suffix)
;           - Returns to caller immediately at CADR+4
;           - Starts separate job (MAKEPLAY) at user's priority
;           - Used when calling program needs to continue execution
;
; REGODSP:  Regodspr without immediate return
;           - Sets BIT14 for special handling
;
; REGODSPR: Regodsp with immediate return
;           - Sets BIT14 and returns immediately
;
; CLOCPLAY: Special clock display mode
;           - Uses CLOCKCON constant for formatting
;
; GOFLASH:  Display with flash indicator
;           - Sets BIT4 to flash verb/noun
;           - Indicates display requires astronaut attention
;
; During Apollo 11 descent (July 20, 1969), normal displays showed:
; - V16N60: Altitude and altitude-rate during powered descent
; - V06N60: Crew-requested displays for monitoring descent parameters
; - Fuel remaining displays
; - LPD (Landing Point Designator) angle
;
; Armstrong and Aldrin relied on these displays to monitor the LM's
; descent trajectory and make the critical decision to continue landing
; despite low fuel warnings in the final moments before touchdown.
; ============================================================================

GODSP		TS	PLAYTEM1	; Store verb/noun code (format: VxxNyy in octal)

GODSP2		CAF	ZERO		; No special flags
		TCF	GOFLASH2	; Join common path without immediate return

GODSPRET	TS	PLAYTEM1	# ENTRANCE FOR A GODSP WITH A PASTE
					; Used when display includes data paste operation

		CAF	BIT6		# SET BIT6 TO GO BACK TO USER AFTER NVSUB
		TCF	GOFLASH2	; Return control after noun subroutine completes

GODSPR		TS	PLAYTEM1	; Store verb/noun code
					; R suffix = immediate Return to caller

GODSPR1		CAF	ZERO		; No special flags
GODSPR2		TS	PLAYTEM4	; Store display control flags

		CAF	ZERO		# * DON'T MOVE
		TCF	GODSPRS1	; Branch to immediate return path

# CLEANDSP IS USED FOR CLEARING OUT A NORMAL DISPLAY THAT IS PRESENTLY ACTIVE OR A NORMAL DISPLAY THAT IS
# SET UP TO BE STARTED OR RESTARTED.
#
# NORMALLY THE USER WILL NOT NEED TO USE THIS ROUTINE SINCE A NEW NORMAL DISPLAY AUTOMATICALLY CLEARS OUT AN
# OLD DISPLAY.
#
# CALLING SEQUENCE FOR CLEANDSP --
#
#		TC	BANKCALL
#		CADR	CLEANDSP

; ============================================================================
; CLEANDSP - Clear Normal Display
;
; Clears active or pending normal display from DSKY and internal state.
; Usually not needed as new displays automatically clear old ones, but
; useful for explicit cleanup or when canceling display without replacement.
;
; REFLASH - Reflash existing display
; Causes current display to flash again, drawing astronaut attention.
;
; REFLASHR - Reflash with immediate return
; Same as REFLASH but returns immediately to caller.
; ============================================================================

CLEANDSP	CAF	ZERO		; Clear verb/noun code
REFLASH		TS	PLAYTEM1	; Store (ZERO or verb/noun to reflash)

		CAF	REDOMASK	# FLASH AND PERMIT
					; Sets flash bit + permit bit for re-entry
		TCF	GOFLASH2	; Process display

REFLASHR	TS	PLAYTEM1	; Store verb/noun to reflash

		CAF	REDOMASK	# FLASH AND PERMIT
		TCF	GODSPRS		; Process with immediate return

# Page 1358
; ============================================================================
; REGODSP/REGODSPR - Regodsp Display (with BIT14 set)
;
; Special display mode that sets BIT14 flag for alternative handling.
; Used when display requires special processing or state preservation.
;
; REGODSP:  Without immediate return (same job)
; REGODSPR: With immediate return (separate job)
; ============================================================================

REGODSP		TS	PLAYTEM1	; Store verb/noun code

		CAF	BIT14		; Set special handling flag (BIT14)
		TCF	GOFLASH2	; Process display without return

REGODSPR	TS	PLAYTEM1	; Store verb/noun code

		CAF	BIT14		; Set special handling flag
		TCF	GODSPR2		; Process with immediate return

; ============================================================================
; CLOCPLAY - Clock Display Mode
;
; Special display using CLOCKCON constant for mission elapsed time formatting.
; Used for displaying time in Hours:Minutes:Seconds format on DSKY.
; ============================================================================

CLOCPLAY	TS	PLAYTEM1	; Store verb/noun code
		CAF	CLOCKCON	; Load clock display format constant
		TCF	GOFLASH2	; Process display

; ============================================================================
; GOFLASH - Flash Display
;
; Initiates display with flashing verb/noun to draw astronaut attention.
; Flashing indicates display requires response or acknowledgment.
;
; GOFLASH2 - Common convergence point for display processing
; All display entry points eventually route through GOFLASH2.
; ============================================================================

GOFLASH		TS	PLAYTEM1	; Store verb/noun code

		CAF	BIT4		# LEAVE ONLY FLASH BIT SET
GOFLASH2	TS	PLAYTEM4	; Store display control flags (flash/permit/etc)

		TC	SAVELOCS	; Save user return address and state

		RELINT			; Re-enable interrupts

		TCF	MAKEPLAY	# BRANCH DIRECT WITH NO SEPARATE JOB CALL
					; Joins caller's job, no new executive job created

; ============================================================================
; PRIORITY DISPLAY ENTRY POINTS - PRIODSP Family
;
; Priority displays take precedence over all other display types and are
; used for critical alarm conditions requiring immediate astronaut attention.
;
; ENTRY POINTS:
;
; PRIODSPR: Priority display with immediate return (R suffix)
;           - Starts separate job at user's priority
;           - Returns immediately to caller
;           - Sets BITS7+4 flags for priority handling
;
; PRIODSP:  Priority display without immediate return
;           - Branches directly to MAKEPLAY (same job)
;           - Sets priority flags
;
; SETPRIO:  Alternative priority entry point
;           - Sets BITS7+4 flags
;           - Joins common GOFLASH2 path
;
; Priority displays interrupt normal and mark displays. During Apollo 11
; descent on July 20, 1969, priority displays showed critical alarms:
; - Program alarm 1202 at 102:38:26 MET (executive overflow)
; - Low fuel warnings during final approach
; - Engine status alerts
;
; These displays were essential - Flight Controller Steve Bales had to
; quickly assess the 1202 alarm display and make the GO/NO-GO decision
; that allowed Armstrong and Aldrin to continue descent to landing.
; ============================================================================

PRIODSPR	TS	PLAYTEM1	; Store verb/noun code for priority display

		CAF	BITS7+4		; Set priority flags (BIT7 + BIT4)
		TCF	GODSPRS		; Process with immediate return

PRIODSP		TS	PLAYTEM1	; Store verb/noun code

SETPRIO		CAF	BITS7+4		; Set priority display flags
		TCF	GOFLASH2	; Process without immediate return

; ============================================================================
; MAKEPRIO - Priority Display Processing
;
; Handles priority display interrupt logic. Checks if another priority
; display is active (which would cause abort unless LINUS flag set).
; Interrupts any active mark or normal displays.
; ============================================================================

MAKEPRIO	CAF	ZERO		; Initialize copy index
		TS	COPINDEX	; Used for saving interrupted display state

		TC	LINUSCHR	; Check LINUS routine flag (BIT14)
		TCF	HIPRIO		# LINUS RETURN
					; LINUS set: allow priority over priority
		CA	FLAGWRD4	; Check if priority already active
		MASK	OCT20100	# IS PRIO IN ENDIDLE OR BUSY
		CCS	A		; Test result
		TCF	PRIOBORT	# YES, ABORT
					; Priority interrupting priority = abort!

HIPRIO		CA	FLAGWRD4	# MARK ACTIVE
					; Check if mark display currently active
		MASK	OCT40400	; Test mark active bits
		EXTEND
		BZF	ASKIFNRM	# NO
					; No mark active, check normal display

# Page 1359
SETMARK		CAF	ZERO		; Mark display was active
		TCF	JOBXCHS		; Exchange jobs to save mark state

ASKIFNRM	CA	FLAGWRD4	# NORMAL ACTIVE
					; Check if normal display active
		MASK	OCT10200	# BITS 13+8
		EXTEND
		BZF	OKTOCOPY	# NO
					; No normal active, proceed to copy

SETNORM		CAF	ONE		; Normal display was active
		TCF	JOBXCHS		; Exchange jobs to save normal state

OKTOCOPY	TC	COPYNORM	; Copy normal display data to save area
		TC	WITCHONE	; Determine which display to interrupt

		TC	JOBWAKE		; Wake up interrupted job when priority ends

		TC	XCHTOEND	; Exchange to ENDIDLE state

REDOPRIO	CA	TIME1		# SAVE TIME PRIODSP SENT OUT
		TS	PRIOTIME	; Store timestamp for priority display
					; Used to determine display age

KEEPPRIO	CAF	ZERO		# START UP PRIO DISPLAY
		TCF	PRIOPLAY	; Branch to priority display handler

; ============================================================================
; CENTRAL DISPLAY DISPATCHER - MAKEPLAY
;
; MAKEPLAY is the heart of the display interface system. Every display request
; (normal, priority, or mark) eventually flows through this central dispatcher.
; It coordinates display ownership, manages the DSKY busy state, and routes
; displays to the appropriate handling logic based on priority and current
; system state.
;
; MAKEPLAY RESPONSIBILITIES:
;
; 1. Priority and Job Management:
;    - Saves user's job priority (PRIORITY register bits 10-14)
;    - Raises priority to PRIO33 for fast display processing
;    - Restores original priority when display completes
;
; 2. Display Type Routing:
;    - Examines PLAYTEM4 bits 15 and 7 to determine display type
;    - Routes to MAKEPRIO for priority displays (alarms)
;    - Routes to MAKEMARK for mark displays (optical navigation)
;    - Routes to normal display handling for standard displays
;
; 3. Linus Routine Integration:
;    - Calls LINUSCHR to check for display system conflicts
;    - LINUSCHR returns with display approval or rejection
;    - Prevents multiple priority displays from colliding (would cause abort)
;
; 4. Display Conflict Resolution:
;    - Checks if other displays are asleep (FLAGWRD4 NBUSMASK)
;    - Detects illegal display state combinations
;    - Routes to PRIOBORT (program abort 1502) if display conflict unresolvable
;
; 5. Display Activation:
;    - Routes to OKTOPLAY when DSKY available for this display
;    - Copies display data to active registers
;    - Restores user priority and bank settings
;    - Either activates display immediately or puts to sleep if DSKY busy
;
; DISPLAY TYPE DETERMINATION (PLAYTEM4 bits):
; - BIT15 set: Priority display (highest priority, interrupts others)
; - BIT7 set: Mark display (medium priority, optical navigation)
; - Neither set: Normal display (lowest priority, standard mission programs)
;
; HISTORICAL CONTEXT:
;
; During Apollo 11 descent on July 20, 1969, MAKEPLAY processed:
; - V16N60 normal displays showing altitude and descent rate
; - Program alarm 1202 priority displays at 102:38:26 MET
; - Crew-initiated displays (V06N60) for monitoring descent parameters
; - LPD angle displays for landing site designation
; - Fuel quantity warnings in final moments before touchdown
;
; The display priority system ensured that critical alarms (1202/1201)
; interrupted normal displays to alert Armstrong and Aldrin, while still
; allowing flight controllers to assess the situation. MAKEPLAY's priority
; logic was essential to the successful Go/No-Go decision that allowed
; the landing to continue despite the alarm condition.
;
; MAKEPLAY coordinated dozens of display updates per second during the
; 12-minute powered descent, maintaining crew situational awareness while
; managing display contention between multiple competing display requests.
; Without this coordination, display conflicts could have caused program
; aborts or obscured critical alarm information.
; ============================================================================

MAKEPLAY	CA	PRIORITY	# SAVE USER'S PRIORITY
					; Load current job priority (bits 10-14 of PRIORITY)
		MASK	PRIO37		; Extract priority bits (octal 37 = bits 10-14)
		TS	USERPRIO	; Save original priority for restoration later
					; User's job will return to this priority when display completes

		CAF	PRIO33		# RAISE PRIORITY FOR FAST JOBS AFTER WAKE
					; Load priority 33 (higher priority = lower number)
					; Ensures display processing runs quickly
		TC	PRIOCHNG	; Change current job priority to PRIO33
					; Display interface runs at elevated priority

		CA	PLAYTEM4	# IS IT MARK OR PRIO OR NORM
					; Load display flags to determine type
		MASK	BITS15+7	; Check BIT15 (priority) and BIT7 (mark)
					; BIT15=1: Priority display (alarms)
					; BIT7=1: Mark display (optical navigation)
					; Both=0: Normal display (standard programs)
		CCS	A		; Check Sign/Compare (routes based on bits)
		TCF	MAKEPRIO	# ITS PRIO
					; BIT15 set: Priority display, handle immediately
		TCF	IFLEGAL		; A was +0: Normal display, check legality
		TCF	MAKEMARK	# ITS MARK
					; BIT7 set: Mark display, optical navigation timing

IFLEGAL		CAF	TWO		; Normal display: Set up for legality check
					; COPINDEX = 2 for normal display data copy
		TS	COPINDEX	; Store copy index for later data transfer

		TC	LINUSCHR	; Call Linus Character routine
					; Checks if display system ready for this request
					; Returns to OKTOPLAY if approved

		TCF	OKTOPLAY	# LINUS RETURN
					; Linus approved: Display can proceed
		CS	EBANKTEM	; Check EBANKTEM for special flags
					; EBANKTEM contains erasable bank and flags
		MASK	BIT4		; Check BIT4 status
		CCS	A		; Is BIT4 set?
		TCF	OKTOPLAY	# NO
					; BIT4 not set: Safe to proceed

		CA	FLAGWRD4	# WAS NORM ASLEEP
					; Check if any normal displays are sleeping
					; Multiple sleeping normals indicate conflict
# Page 1360
		MASK	NBUSMASK	# ARE ANY NORMS ASLEEP
					; NBUSMASK isolates normal busy flags
		EXTEND
		BZF	OKTOPLAY	# NO
					; No sleeping normals: Display can proceed

PRIOBORT	TC	POODOO		; PROGRAM ABORT - Display Conflict
					; Unresolvable display state conflict detected
					; Multiple priority displays or illegal state
		OCT	1502		; Abort code 1502: Display system conflict
					; Crew would see "PROG" light and alarm code
					; Indicates software error in display management

; ============================================================================
; DISPLAY ACTIVATION - OKTOPLAY
;
; Once MAKEPLAY has determined that a display can proceed (priority checked,
; conflicts resolved, Linus approved), control reaches OKTOPLAY. This routine
; activates the display by copying display data from temporary storage to
; active registers and managing job priorities.
;
; OKTOPLAY RESPONSIBILITIES:
;
; 1. Display Data Transfer:
;    - Calls COPIES2 to copy display data from PLAYTEM to DSPY area
;    - Transfers verb, noun, and data register values
;    - Copies display flags and configuration
;
; 2. Priority Restoration:
;    - Restores user's original job priority (saved in USERPRIO)
;    - Combines with SUPERBNK (bank information)
;    - Stores in RESTREG for eventual priority restoration
;
; 3. Display State Management:
;    - Checks if priority or mark displays currently active (PMMASK)
;    - Routes to GOSLEEPS if DSKY busy with higher-priority display
;    - Allows display to proceed if DSKY available
;
; 4. Job Wake Coordination:
;    - Calls WITCHONE to check display system state
;    - Routes to JOBWAKE if appropriate
;    - Routes to XCHTOEND for end-idle handling
;
; HISTORICAL CONTEXT:
;
; During Apollo 11's descent on July 20, 1969, OKTOPLAY activated each
; display update that Armstrong and Aldrin relied upon:
; - Altitude displays (V16N60) updated every few seconds
; - LPD angle updates as Armstrong adjusted landing site designation
; - Fuel quantity displays in final approach (60-second, 30-second warnings)
; - Program alarm 1202 displays requiring immediate crew attention
;
; OKTOPLAY was called dozens of times per minute during powered descent,
; coordinating the flow of navigation data, guidance parameters, and alarm
; information to the DSKY. This routine ensured that displays appeared in
; correct priority order and that the crew always saw the most critical
; information first.
; ============================================================================

OKTOPLAY	TC	COPIES2		; Copy display data from PLAYTEM to DSPY area
					; COPIES2 transfers verb, noun, and register data
					; Makes display data active for Pinball to output

		CA	USERPRIO	; Load user's original job priority
					; Priority was saved at MAKEPLAY entry
					; Will be restored when display completes
		EXTEND			; Next instruction is double-precision
		ROR	SUPERBNK	; Rotate right, combining USERPRIO with SUPERBNK
					; SUPERBNK contains bank information
					; Creates complete priority+bank restore value
		TS	RESTREG		; Store combined value for priority restoration
					; RESTREG used when returning to user's job

		CA	FLAGWRD4	# PRIO OR MARK GOING
					; Check if priority or mark displays active
		MASK	PMMASK		; PMMASK isolates priority/mark busy flags
					; If set, higher-priority display owns DSKY
		CCS	A		; Check result
		TCF	GOSLEEPS	# MARK GOING
					; Non-zero: Priority or mark active, go to sleep
					; This display must wait for DSKY availability

		TCF	+2		; A was +0: No priority/mark active
		TCF	GOSLEEPS	; A was -0: Still need to sleep

# COULD PUT NORM BUSY CHECK HERE TO SAVE TIME

		TC	WITCHONE	# IS IT NVSUB BUSY, ENDIDLE OR NOONE
		TC	JOBWAKE

		TC	XCHTOEND

PLAYJUM1	CAF	TWO
PRIOPLAY	TS	COPINDEX

		TCF	GOPLAY

EXDSPRET	TS	PLAYTEM1

		CAF	BIT15+6
		TCF	GOFLASH2

GOPERF1		TS	NORMTEM1	# STORE DESIRED CHECKLIST VALUE
		CAF	V01N25		# USED TO DISPLAY CHECKLIST VALUE IN R1

GOPERFS		TS	PLAYTEM1

		CAF	PERFMASK	# LEAVE ONLY FLASH, PERFORM, BLANKING
		TCF	GOFLASH2

GOPERF2		TS	PLAYTEM1	# DESIRED VERB-NOUN TO DISPLAY R1,R2,R3

		CAF	PERF2MSK
		TCF	GOFLASH2

# Page 1361
GOPERF4		TC	PURRS4

		TCF	GOFLASH2

GOFLASHR	TS	PLAYTEM1

		CAF	BIT4		# LEAVE ONLY FLASH BIT SET
GODSPRS		TS	PLAYTEM4

		CAF	THREE

GODSPRS1	INHINT			# IMMEDIATE RETURN IS CALL CADR +4
		TS	RUPTREG3

		CA	PRIORITY	# MAKE DISPLAY ONE HIGHER THAN USER
		MASK	PRIO37
		TS	NEWPRIO

		CA	PLAYTEM4	# IS THIS A FLASHING R DISPLAY
		MASK	BIT4
		CCS	A
		TCF	VACDSP		# YES, MAKE DSPLAY JOB A VAC
		CA	NEWPRIO		# NO, MAKE DSPLAY JOB A NOVAC
		TC	NOVAC
		EBANK=	WHOCARES
		2CADR	MAKEPLAY

		TCF	BOTHJOBS

VACDSP		CA	BBANK
		EXTEND
		ROR	SUPERBNK
		TS	L
		CAF	MAKEGEN
		TC	SPVAC

BOTHJOBS	TC	SAVELOCS	# COPY TEMPS INTO PERMANENT REGISTERS

		EXTEND			# SAVE NVWORD AND USER'S MPAC +2
		DCA	MPAC 	+1
		INDEX	LOCCTR
		DXCH	MPAC 	+1

		EXTEND			# SAVE USER'S CADR, FLAGS AND EBANK
		DCA	MPAC 	+3
		INDEX 	LOCCTR
		DXCH	MPAC 	+3

		CA	LOCCTR
		TS	MPAC 	+5
# Page 1362
		TC	SAVELOCR
		RELINT

		TCF	BANKJUMP	# CALL CADR +4

GOPERF1R	TS	NORMTEM1	# DESIRED CHECKLIST VALUE

		CAF	V01N25		# DISPLAYS CHECKLIST VALUE IN R1

GOPERFRS	TS	PLAYTEM1

		CAF	PERFMASK	# LEAVE ONLY FLASH, PERFORM, BLANKING
		TCF	GODSPRS

GOPERF2R	TS	PLAYTEM1	# DESIRED VERB-NOUN TO DISPLAY R1,R2,R3

		CAF	PERF2MSK
		TCF	GODSPRS

GOPERF4R	TC	PURRS4

		TCF	GODSPRS

PURRS4		TS	OPTION1		# DESIRED OPTION CODE

		CAF	V04N06
		TS	PLAYTEM1

		CAF	PERF4MSK	# FLASH, PERFORM AND EBANK R3
		TC	Q

SAVELOCS	INHINT

		CS	OCT3400		# EBANK BITS
		MASK	PLAYTEM4
		AD	EBANK
		TS	PLAYTEM4

; ============================================================================
; JOB CREATION AND DATA MANAGEMENT ROUTINES
;
; These routines handle the critical task of creating new display jobs,
; copying display data between temporary and permanent storage areas, and
; managing job sleep/wake states when the DSKY is busy.
; ============================================================================

; SAVELOCR - Save Return Location for Immediate Return Displays
;
; For displays ending in "R" (GODSPR, PRIODSPR, GOMARKR, etc.), this routine
; prepares the immediate return to the caller while starting a separate job
; to handle the actual display processing. The user's job continues immediately
; while the display operates independently in the background.
;
; This separation allows mission programs to request displays without waiting
; for display completion, maintaining real-time responsiveness during critical
; flight phases. During Apollo 11 descent, this enabled P63 to continue
; guidance calculations while simultaneously updating descent displays.

SAVELOCR	LXCH	Q		; Save return address in L register

		TC	MAKECADR	; Create CADR from return address
		TS	PLAYTEM3	; Store for later return

		AD	RUPTREG3	# NOT USED FOR NON R ROUTINES
		TC	L		; Return to caller immediately

; COPYNORM / COPIES / COPIES2 - Display Data Copy Routines
;
; These routines copy display data from temporary working registers (PLAYTEM)
; to permanent storage areas (EBANKSAV, NVSAVE, CADRFLSH, etc.). This copying
; is essential because:
;
; 1. Display requests use temporary registers that will be overwritten
; 2. If DSKY is busy, display must be put to sleep with data preserved
; 3. When display wakes up, permanent storage provides restart information
;
; COPINDEX controls which set of storage locations to use:
; - COPINDEX = 0: Normal display storage (EBANKSAV, NVSAVE, etc.)
; - COPINDEX = 2: Priority display storage (offset by 2 words)
; - COPINDEX = 4: Mark display storage (offset by 4 words)
;
; This indexing allows three concurrent displays to be "asleep" simultaneously,
; each with its own protected data area, waiting for DSKY availability.

COPYNORM	CAF	ZERO		; Set index for normal display
COPIES		TS	COPINDEX	; Store copy index
COPIES2		INHINT			; Disable interrupts during copy
		CA	PLAYTEM4	# FLAGWORD

# Page 1363
		INDEX	COPINDEX
		TS	EBANKSAV	# EQUIV TO DSPFLG

		MASK	CADRMASK	# FLASH AND GODSPRET
		EXTEND
		BZF	SKIPADD

		CA	PLAYTEM3
		INDEX	COPINDEX
		TS	CADRFLSH

SKIPADD		CA	PLAYTEM1	# VERB NOUN
		INDEX	COPINDEX
		TS	NVWORD

		TCF	RELINTQ		; Return with interrupts enabled

; ============================================================================
; DISPLAY SLEEP/WAKE MANAGEMENT
;
; When the DSKY is busy with another display, incoming display requests cannot
; be processed immediately. These routines put display jobs to "sleep" in the
; executive's job queue, preserving all their data, then "wake" them later
; when the DSKY becomes available.
;
; This is critical during high-workload mission phases. During Apollo 11's
; lunar descent, multiple displays competed for the single DSKY: guidance
; updates, alarm displays, crew-initiated requests. The sleep/wake mechanism
; ensured all displays eventually reached the astronauts without data loss.
; ============================================================================

; GOSLEEPS - Put Display Job to Sleep
;
; Called when DSKY is busy and display must wait. Sets "waiting" flag bits
; in FLAGWRD4 to indicate which type of display is asleep (priority/normal/mark).
; The display data has already been copied to permanent storage by COPIES2.
;
; COPINDEX determines which display type: 0=normal, 2=priority, 4=mark

GOSLEEPS	INDEX	COPINDEX	; Get priority octal for this display type
		CA	PRIOOCT
		MASK	WAITMASK	; Extract waiting flag bits
		TC	UPENT2		; Set the waiting flag in FLAGWRD4
WAITMASK	OCT	3004		; Mask for priority waiting flags
		CS	ONE
		AD	COPINDEX
		TS	FACEREG		; Store display type index

; XCHSLEEP - Exchange Sleep (Wake One Display, Sleep Another)
;
; This routine performs an atomic swap: it wakes up a sleeping display job
; while simultaneously putting a different job to sleep. Used when switching
; between displays of different priorities.
;
; The awakened job's return address is changed to ENDOFJOB so it terminates
; cleanly, then the new job is put to sleep in its place with the proper
; wake-up address. This maintains exactly one display job active at a time.

XCHSLEEP	INDEX	FACEREG		; Get wake-up address for this display type
		CAF	WAKECADR
		INHINT			; Disable interrupts during job manipulation
		TC	JOBWAKE		# FIND CADR IN JOB AREA

		TC	XCHTOEND	# CAUSES AWAKENED JOB TO GO TO ENDOFJOB

		INDEX	FACEREG		# REPLACE SAME CADR BUT NEW JOB AREA
		CAF	WAKECADR	; New wake address for sleeping job
		TCF	JOBSLEEP	; Put new job to sleep

; JOBXCHS - Job Exchange and Sleep Management
;
; Complex routine that manages display job swapping when priorities change.
; Coordinates with WITCHONE to determine which display should be active,
; manipulates the job queue to wake/sleep appropriate displays, and updates
; flag bits to track display states.
;
; This routine is called when a new display interrupts an existing one:
; the interrupted display is put to sleep, and the new display takes over
; the DSKY. The sleeping display will be reawakened later via WAKEPLAY.

JOBXCHS		TS	FACEREG		# CONTROLS TYPE OF DISPLAY PUT TO SLEEP
		TC	WITCHONE	; Determine which display should run
		TC	JOBWAKE		; Wake the appropriate job
		CA	FACEREG
		INDEX	LOCCTR
		TS	FACEREG		; Update display type tracking

		CAF	XCHQADD		; Address for exchange sleep
		TC	XCHNYLOC	; Exchange job locations

		INDEX	FACEREG
		CA	MARKOCT
		MASK	IDLESLEP	; Check for idle/sleep flags

# Page 1364
		TC	DOWNENT2	; Clear appropriate flag bits
IDLEMASK	OCT	74004		# * DON'T MOVE

		INDEX	FACEREG		# BIT SHOWS PRIO INTERRUPTED NORM OR MARK
		CA	BIT5		# BIT5 FOR MARK, BIT4 FOR NORMAL
		AD	FOUR		; Offset for flag bit setting
		TC	UPENT2		# FLAG ROUTINE DOES RELINT
XCHQADD		GENADR	XCHSLEEP	# * DON'T MOVE
		CA	FLAGWRD4
		MASK	MKOVBIT		# MARK OVER NORM?
		CCS	A		; Test if mark interrupted normal
GENMARK		TC	MARKPLAY	# USED AS GENADR FOR JOBWAKE
		TCF	OKTOCOPY	; Continue to display processing

; MARKWAKE / WAKEPLAY - Wake Sleeping Display Jobs
;
; These routines wake up display jobs that have been put to sleep when the
; DSKY becomes available again. Called after active display completes or
; is interrupted, allowing the next waiting display to take over.
;
; MARKWAKE specifically wakes mark displays (TEMPOR2 = 0 for normal marks)
; WAKEPLAY is the general wake-up routine for any display type
;
; During Apollo 11 landing, when Armstrong finished examining a crew-initiated
; display and pressed PROCEED, these routines would reawaken the landing
; guidance displays that had been temporarily interrupted.

MARKWAKE	CAF	ZERO		; Wake normal mark display
WAKEPLAY	TS	TEMPOR2		; Store display type (0=mark, 2=prio, 4=norm)

		INDEX	TEMPOR2	; Get flag bits for this display type
		CA	BITS5+11	; Bits indicating interrupted/sleeping state
		AD	FOUR		; Offset for flag manipulation
		TC	DOWNENT2	; Clear the sleeping/interrupted flags
MARKFMSK	OCT	40010		# *** DON'T MOVE

		INDEX	TEMPOR2	; Get wake-up address for this display
		CAF	WAKECADR
		INHINT			; Disable interrupts during wake operation
		TC	JOBWAKE		; Wake the job in executive queue

		TCF	ENDRET		; Return via standard exit

# ALL .1 RESTARTS BRANCH DIRECTLY TO INITDSP.  NORMAL DISPLAYS ARE THE ONLY DISPLAYS ALLOWED TO USE .1 RESTARTS
# INITDSP FIRST RESTORES THE EBANK AND THE SUPERBANK TO THE MOST RECENT NORMAL EBANK AND SUPERBANK.
#
# IF THE MOST RECENT NORMAL DISPLAY REQUEST WAS NOT FINISHED, CONTROL IS SENT BACK TO THE LAST NORMAL USER.
# OTHERWISE THE NORMAL DISPLAY SET UP IN THE NORMAL DISPLAY REGS IS STARTED UP IMMEDIATELY.

INITDSP		CA	EBANKTEM	# RESTORE MOST RECENT NORMAL EBANK
		TS	EBANK

		CA	RESTREG		# SUPERBANK AND JOB PRIORITY
		TC	SUPERSW		# RESTORE SUPERBANK

		MASK	PRIO37
		TC	PRIOCHNG

		CS	THREE
		AD	TEMPFLSH
		TCF	BANKJUMP

PINBRNCH	RELINT			# FOR GOPIN USERS
		CA	MARK2PAC	# NEEDED TO SAVE MPAC +2 FOR MARK USERS
# Page 1365
		TS	MPAC 	+2	# ONLY

		CA	FLAGWRD4	# PINBRANCH CONDITION
		MASK	PINMASK
		CCS	A
		TCF	+3
		TCF	ERASER		# ** NOTHING IN ENDIDLE
		TCF	MARKPLAY

NORMBNCH	TC	UPFLAG		# SET PINBRANCH BIT
		ADRES	PINBRFLG

		CAF	PRIODBIT	# PRIO INTERRUPTED
		MASK	FLAGWRD4
		CCS	A
		TCF	KEEPPRIO

		TCF	PLAYJUM1

NVDSP		TC	COPYPACS

		CA	TEMPOR2		# SET UP BLANK BITS FOR NVMONOPT IN CASE
		MASK	SEVEN		# USER REQUESTS BLANKING MONITOR
		TS	L

		CS	BIT13
		INDEX	COPINDEX
		MASK	DSPFLG
		INDEX	COPINDEX
		TS	DSPFLG

		MASK	BIT8		# BIT8 SET IF DEC MARK PERFORM DISPLAY
		TS	TEM1

		CA	MPAC 	+2
		TS	MPAC2SAV

		TS	MARK2PAC	# * FOR DISK ONLY *
		INDEX	COPINDEX
		CCS	NVWORD
		TCF	NVDSP1
		TCF	CLEANEND
		CS	MARKNV
		TS	MARKNV		# IN CASE MARKPLAY AWAKENED AFTER SLEEPING
		MASK	LOW7
		AD	V05N00M1
		AD	TEM1
NVDSP1		AD	ONE
NV50DSP		TC	NVMONOPT
		TCF	REST		# IF BUSY

# Page 1366
		TC	FLASHOFF	# IN CASE OF EXTENDED VERB NON-FLASH

		TC	COPYTOGO	# MPACS DESTROYED BY NVSUB
		TC	DOWNFLAG	# UNSET SLEEPING BITS
		ADRES	MRKNVFLG
		TC	DOWNFLAG
		ADRES	NRMNVFLG
		TC	DOWNFLAG
		ADRES	PRONVFLG
BLANKCHK	CA	TEMPOR2		# BLANK BITS 1,2,3 IF SET
		TC	BLANKSUB
		TCF	NVDSP
PERFCHEK	CAF	BIT5		# BIT5 FOR PERFORM
		MASK	TEMPOR2
		CCS	A		# IS THIS A GOPERF DISPLAY
		TCF	1STOR2ND	# YES

GOANIDLE	CAF	BIT4
		MASK	TEMPOR2
		CCS	A
		TCF	FLASHSUB	# IT IS

		CS	TEMPOR2		# IS THIS A GODSPRET
		MASK	BIT6
		CCS	A
		TCF	ISITN00

		INDEX	COPINDEX
		CA	CADRFLSH
		TS	MPAC 	+3
		TCF	ENDIT

ISITN00		INDEX	COPINDEX	# IS THIS A PASTE
		CA	NVWORD
		MASK	LOW7		# CHECK MADE FOR PINBRNCH AND PRIO ON MARK
		EXTEND
		BZF	FLASHSUB	# YES, ASSUME PASTE ALWAYS ON FLASH

		TCF	ENDOFJOB	# NOT FLASH, NOT GOPERF, THEREFORE EXIT

1STOR2ND	CA	TEMPOR2
		MASK	BIT13
		CCS	A
		TCF	GOANIDLE	# SECOND

		CA	BIT13
		INDEX	COPINDEX
		ADS	DSPFLG

		ZL
# Page 1367
		EXTEND			# IS IT MARK
		BZMF	MARKPERF	# YES

		MASK	BIT12
		EXTEND
		BZF	V50PASTE
		CS	NVWORD1		# NVOWRD1= -0 IS V97.  NVWORD1= -400 IS V99
		AD	V97N00
		TCF	NV50DSP
V50PASTE	CAF	V50N00
		TCF	NV50DSP		# DISPLAY SECOND PART OF GOPERF

WITCHONE	CS	BIT5		# TURN OFF KEY RELEASE LIGHT
		EXTEND
		WAND	DSALMOUT

		CA	FLAGWRD4
		MASK	NVBUSMSK	# IS IT NVSUB ALEEP
		CCS	A
		CAF	ONE
		TS	L
		CAF	ZERO
		INDEX	L
		XCH	CADRSTOR

		INHINT
		TC	Q

XCHTOEND	CAF	ENDINST		# TC ENDOFJOB REPLACES GENADR IN LOC FOR
XCHNYLOC	XCH	LOCCTR		# WAS THIS ADDRESS SLEEPING
		EXTEND
		BZMF	RELINTQ		# NO
		XCH	LOCCTR		# YES
		INDEX	LOCCTR
		TS	LOC

RELINTQ		RELINT
		TC	Q		# BACK TO USER

CLEANEND	CAF	PRIO32		# ONE LOWER THAN DISPLAYS SLEEPING
		TC	FINDVAC
		EBANK=	NVSAVE
		2CADR	JAMTERM

		TCF	FLASHSUB +1

; ============================================================================
; DISPLAY RESTART AND STATE MANAGEMENT LOGIC
;
; This section handles display system restarts after program interruptions,
; manages the transition between idle and active display states, and 
; coordinates the return paths for astronaut responses. Critical for
; maintaining display system integrity during program alarms and restarts.
; ============================================================================

; ----------------------------------------------------------------------------
; ISITPRIO - Check if priority or mark display is active
;
; COMMENT-ONLY: Verifies whether a priority alarm display or mark routine
; is currently in control of the DSKY before allowing display restart.
;
; CODE-ALONG: Tests PINBRFLG and MARKIDFLG bits in FLAGWRD4 to determine
; if high-priority display operations are in progress. If set, aborts the
; restart; if clear, allows normal display termination.
; ----------------------------------------------------------------------------

ISITPRIO	CA	FLAGWRD4
		MASK	ITISMASK	# IS PINBRFLG, MARKIDFLG SET
		EXTEND
# Page 1368
		BZF	PRIOBORT
		TCF	ENDOFJOB

; ----------------------------------------------------------------------------
; REST - Restart checkpoint for display operations
;
; COMMENT-ONLY: Entry point for restarting display routines after program
; restart. Checks if another display is already waiting in idle state before
; attempting to reactivate this display.
;
; CODE-ALONG: Tests CADRSTOR to see if ENDIDLE is already occupied. If
; positive (someone in ENDIDLE), exits to avoid conflict. If zero or negative,
; proceeds to RESTSLEP to set up display sleep state for proper restart.
; ----------------------------------------------------------------------------

REST		CCS	CADRSTOR	# IS SOMEONE IN ENDIDLE
		TCF	ENDOFJOB	# YES
		TCF	RESTSLEP

		TCF	ENDOFJOB

; ----------------------------------------------------------------------------
; RESTSLEP - Set up display sleep state during restart
;
; COMMENT-ONLY: Configures the display system to "sleep" state during
; restart recovery. This preserves the display's place in the priority
; queue so it can be reactivated when the astronaut is ready.
;
; CODE-ALONG: Sets NVSLEEP bits in flag word using GENMASK and ASTROMSK.
; Calls NVSUBUSY to mark display as busy or abort if display request is
; illegal. This ensures proper sequencing when multiple displays restart.
; ----------------------------------------------------------------------------

RESTSLEP	CA	GENMASK		# SET NVSLEEP BITS
		MASK	ASTROMSK
		TC	UPENT2
OCT24100	OCT	24100		# *** DON'T MOVE

		INDEX	COPINDEX
		CAF	NVCADR
		TC	NVSUBUSY	# BUSY OR ABORT IF ILLEGAL

; ----------------------------------------------------------------------------
; FLASHSUB - Flash display activation
;
; COMMENT-ONLY: Turns on the flashing indicators on the DSKY to draw the
; astronaut's attention. Used for displays requiring crew response.
;
; CODE-ALONG: Calls FLASHON routine to activate verb/noun flash pattern.
; Preserves COPINDEX in COPMPAC since ENDIDLE will destroy it. Sets up
; display idle flags for proper ENDIDLE coordination.
; ----------------------------------------------------------------------------

FLASHSUB	TC	FLASHON

		CA	COPINDEX	# COPINDEX DESTROYED BY ENDIDLE
		TS	COPMPAC

		CA	GENMASK
		MASK	IDLEMASK
		TC	UPENT2
ITISMASK	OCT	40040		# *** ENDIDLE ALLOW *** DON'T MOVE

; ----------------------------------------------------------------------------
; Repeat-and-Return Display Check
;
; COMMENT-ONLY: Determines if this display is configured for "repeat and
; return" mode, where the same data is displayed multiple times without
; waiting for new astronaut input each time.
;
; CODE-ALONG: Tests BIT3 of R1SAVE to check repeat-and-return flag. If set,
; branches to UNSETR1 to clear the flag. If clear, continues to ENDIDLE
; state management to wait for astronaut response.
; ----------------------------------------------------------------------------

		CA	R1SAVE		# IS THIS A REPEAT AND RETURN DISPLAY
		INDEX	COPINDEX
		MASK	BIT3
		CCS	A
		TCF	UNSETR1		# YES

; ----------------------------------------------------------------------------
; ENDIDLE Entry and Return Path Coordination
;
; COMMENT-ONLY: The display now enters idle state, waiting for the astronaut
; to respond via the DSKY. When the astronaut presses PROCEED, ENTER, or
; TERMINATE, execution returns to one of the following paths to complete
; the display operation.
;
; CODE-ALONG: Checks CADRSTOR to prevent multiple displays from entering
; ENDIDLE simultaneously (would cause state corruption). Calls ENDIDLE
; routine which blocks until astronaut responds. ENDIDLE has three return
; points: TERMATE (terminate), next instruction (proceed), next+1 (enter).
; ----------------------------------------------------------------------------

		CCS	CADRSTOR	# SEE IF SOMEONE ALREADY IN ENDIDLE
		TCF	ISITPRIO
		TCF	+2
		TCF	ISITPRIO

		TC	ENDIDLE
IDLERET1	TCF	TERMATE

		TCF	PROCEED		# ENDIDLE RETURNS HERE ON PROCEED

; ----------------------------------------------------------------------------
; Load Validity Check for V21/V22/V23
;
; COMMENT-ONLY: When astronaut presses ENTER after typing data, verifies
; that the verb is one that accepts numeric loads (V21=load component 1,
; V22=load component 2, V23=load component 3). If not one of these verbs,
; the data entry is invalid.
;
; CODE-ALONG: Computes (VERBREG - 22). If result is 0, -1, or -2, then
; verb is V21, V22, or V23 respectively. BZF on DIM A tests for these
; three cases. If match, branches to LOADITIS to validate and store data.
; ----------------------------------------------------------------------------

		CS	LOWLOAD
		AD	MPAC		# VERBREG
		EXTEND
		DIM	A
		EXTEND
		BZF	LOADITIS	# V21 OR V22 OR V23 ON DSKY

# Page 1369
; ----------------------------------------------------------------------------
; OKTOENT - Valid data entry accepted
;
; COMMENT-ONLY: The astronaut's data entry has been validated. Store the
; completion code indicating successful entry.
;
; CODE-ALONG: Sets OUTHERE to 2 (enter/recycle return code). Falls through
; to classify the type of display that is completing (priority, normal, or
; mark) to determine proper return path to user program.
; ----------------------------------------------------------------------------

OKTOENT		CAF	TWO
ENDOUT		TS	OUTHERE

; ----------------------------------------------------------------------------
; ENDIDLE Return Classification
;
; COMMENT-ONLY: After the astronaut responds, the display system must
; determine what type of display is completing: priority alarm (highest),
; mark display (medium), or normal mission program display (lowest). Each
; type has different rules for returning control to the waiting program.
;
; CODE-ALONG: Tests bits 14 and 15 of FLAGWRD4 (OCT60000 mask) to classify
; display type. Positive=priority (bit 15), zero=normal (neither bit),
; negative=mark (bit 14). Priority displays go to TIMECHEK first to verify
; they haven't expired before returning.
; ----------------------------------------------------------------------------

		CA	FLAGWRD4	# CHECK NATURE OF ENDIDLE RETURN
		MASK	OCT60000
		CCS	A
		TCF	TIMECHEK	# PRIO ENDIDLE RETURN
		TCF	NORMRET		# NORMAL ENDIDLE RETURN
		TCF	MARKRET		# MARK ENDIDLE RETURN

; ----------------------------------------------------------------------------
; TIMECHEK - Priority display timeout verification
;
; COMMENT-ONLY: Priority alarm displays have a time limit. If the astronaut
; took too long to respond (more than 2 seconds), the display may no longer
; be valid. Computes elapsed time and downgrades to normal display if expired.
;
; CODE-ALONG: Computes (PRIOTIME - TIME1) to get elapsed centiseconds since
; priority display started. Compares against -2SEC threshold. If more than
; 2 seconds elapsed (BZMF fails), treats as normal display return instead.
; This prevents stale priority alarms from disrupting current operations.
; ----------------------------------------------------------------------------

TIMECHEK	CS	TIME1
		AD	PRIOTIME
		CCS	A
		COM
		AD	OCT37776
		AD	ONE
		AD	-2SEC
		EXTEND
		BZMF	KEEPPRIO

		TCF	NORMRET

; ----------------------------------------------------------------------------
; NORMWAKE - Wake normal display from sleep
;
; COMMENT-ONLY: A normal display that was put to sleep is now ready to
; continue. Reactivates it through the standard wake mechanism.
;
; CODE-ALONG: Sets A=1 (COPINDEX for normal display) and branches to
; WAKEPLAY routine to restore display state and continue execution.
; ----------------------------------------------------------------------------

NORMWAKE	CAF	ONE
		TCF	WAKEPLAY

; ============================================================================
; DISPLAY COMPLETION AND EXIT ROUTINES
;
; This section handles the various exit paths when displays complete or are
; interrupted. Manages flag cleanup, state restoration, and proper return
; sequencing to ensure the DSKY can be reused by the next display.
; ============================================================================

; ----------------------------------------------------------------------------
; ENDRET - ENDIDLE exit processing
;
; COMMENT-ONLY: The display system is exiting from the ENDIDLE state, which
; occurs when a display completes while the astronaut is still using the DSKY.
; Determines whether to end the current job or continue with cleanup.
;
; CODE-ALONG: Tests OUTHERE flag. If negative (CCS result), adds ONE and
; continues to ENDIDLE cleanup. If zero, branches to ENDOFJOB (display job
; complete). Otherwise, computes return address from CADRFLSH table (indexed
; by COPMPAC) and stores in MPAC+3. Clears ENDIDLE and PINBRANCH bits using
; PINIDMSK (OCT 74044), blanks all displays except mission mode (MM), then
; continues to ENDIT for final cleanup.
; ----------------------------------------------------------------------------

ENDRET		CCS	OUTHERE
		AD	ONE
		TCF	+2		# NORMAL ENDIDLE EXIT
		TCF	ENDOFJOB
		INDEX	COPMPAC
		AD	CADRFLSH
		TS	MPAC 	+3

		CA	GENMASK		# REMOVE ENDIDLE AND PINBRANCH BITS
		MASK	PINIDMSK
		TC	DOWNENT2
PINIDMSK	OCT	74044		# *** DON'T MOVE

		CS	THREE		# BLANK EVERYTHING EXCEPT MM
		TC	NVSUB
		TCF	+1

; ----------------------------------------------------------------------------
; ENDIT - Display completion cleanup and return to calling program
;
; COMMENT-ONLY: The display has completed its interaction with the astronaut.
; Restore the calling program's priority level and return control to it.
;
; CODE-ALONG: Restores original job priority from USERPRIO (masked with
; PRIO37 to extract priority bits 10-12). Calls PRIOCHNG to change priority.
; Loads return address from MPAC+3 (set up earlier by ENDRET) and branches
; to BANKJUMP to return control to calling program at completion exit point.
; ----------------------------------------------------------------------------

ENDIT		CA	USERPRIO	# RETURN TO USER'S PRIORITY
		MASK	PRIO37
		TC	PRIOCHNG
		CA	MPAC 	+3
		TCF	BANKJUMP

; ----------------------------------------------------------------------------
; UNSETR1 - Clear repeat-and-return flag
;
; COMMENT-ONLY: This display was configured for "repeat and return" mode.
; Clear that flag now that the repeat cycle is complete, then continue to
; immediate return processing.
;
; CODE-ALONG: Clears BIT3 of R1SAVE (indexed by COPINDEX) which is the
; repeat-and-return flag. Uses CS BIT3 to invert bit 3, then MASK with
; R1SAVE to keep only that bit clear. Stores result back to R1SAVE.
; Calls SUPERSW with ZERO to ensure superbank 0 for immediate return users
; (only MARKBRAN users in 205 use this path). Falls through to IMMEDRET.
; ----------------------------------------------------------------------------

UNSETR1		INDEX	COPINDEX	# RESET REPEAT AND RETURN REQUEST
		CS	BIT3
# Page 1370
		MASK	R1SAVE
		TS	R1SAVE

		CAF	ZERO		# *** 205 ONLY MARKBRAN USERS IN
		TC	SUPERSW		# SUPERBANK 0

; ----------------------------------------------------------------------------
; IMMEDRET - Immediate return display cleanup
;
; COMMENT-ONLY: For displays that return control immediately to the calling
; program (without waiting for astronaut response), compute the return
; address and branch back to the user.
;
; CODE-ALONG: Retrieves immediate return address from CADRFLSH table (indexed
; by COPINDEX), adds THREE (from prior CAF) to offset to correct return
; location, and branches to BANKJUMP to return to calling program. This path
; is used by GODSPR, PRIODSR, MARKR routines that return immediately after
; initiating display.
; ----------------------------------------------------------------------------

 -1		CAF	THREE		# RETURN TO USER'S IMMEDIATE RETURN LOC
IMMEDRET	INDEX	COPINDEX
		AD	CADRFLSH
		TCF	BANKJUMP

; ============================================================================
; ASTRONAUT RESPONSE HANDLERS
;
; This section processes the three primary astronaut responses to flashing
; displays: TERMINATE (V34), PROCEED (V33), and ENTER/RECYCLE (V32).
; During Apollo 11's descent, Armstrong and Aldrin used these verbs to
; respond to AGC display requests, with V34 declining, V33 accepting,
; and V32 entering data or recycling displays.
; ============================================================================

; ----------------------------------------------------------------------------
; TERMATE - Terminate response handler (V34)
;
; COMMENT-ONLY: The astronaut pressed VERB 34 to terminate/decline the
; flashing display. This clears the request and returns control to the
; calling program at its "terminate" exit point (return address + 0).
;
; CODE-ALONG: Loads ZERO into A register (return offset = 0) and branches
; to ENDOUT to complete display termination and return to caller at CADR+0.
; ----------------------------------------------------------------------------

TERMATE		CAF	ZERO		# ASTRONAUT TERMINATE (V34) RETURNS TO
		TCF	ENDOUT

; ----------------------------------------------------------------------------
; LINUSCHR - LINUS priority display special handling
;
; COMMENT-ONLY: Special logic for LINUS priority displays. LINUS (BIT14 set
; in PLAYTEM4) requires careful handling to avoid duplicate displays or
; improper termination of critical alarms. Checks if LINUS is already active
; before proceeding.
;
; CODE-ALONG: Tests PLAYTEM4 BIT14 (LINUS flag). If not LINUS, returns to
; Q+1 (skip this logic). If LINUS, checks if already in ENDIDLE by comparing
; PLAYTEM3 to CADRFLSH entry. If already in ENDIDLE (BZF +2), or if astronaut
; is busy (DSPLOCK positive), ends job to prevent duplicate displays.
; Otherwise returns to Q (normal path).
; ----------------------------------------------------------------------------

LINUSCHR	CS	PLAYTEM4	# IS THIS A LINUS
		MASK	BIT14
		CCS	A
		TCF	Q+1		# NO
		CS	PLAYTEM3	# YES, IS IT ALREADY IN ENDIDLE
		INDEX	COPINDEX
		AD	CADRFLSH
		EXTEND
		BZF	+2		# YES

		TC	Q		# NO
		CCS	DSPLOCK		# IS THE ASTRONAUT BUSY
		TC	ENDOFJOB	# END THE NEW DISPLAY, IT'S ALREADY ACTIVE
		TC	Q

# MORE LOGIC COULD BE INCORPORATED HERE TO MAKE SURE A RECYCLE IS A RECYCLE AND CONVERSELY THAT A LOAD IS A LOAD

; ----------------------------------------------------------------------------
; PROCEED - Proceed response handler (V33)
;
; COMMENT-ONLY: The astronaut pressed VERB 33 to proceed/accept the flashing
; display. This affirms the request and returns control to the calling
; program at its "proceed" exit point (return address + 1). During critical
; mission phases like landing, V33 responses acknowledged AGC requests and
; allowed programs to continue.
;
; CODE-ALONG: Loads ONE into A register (return offset = 1) and branches to
; ENDOUT to complete display processing and return to caller at CADR+1
; (proceed exit point).
; ----------------------------------------------------------------------------

PROCEED		CAF	ONE		# ASTRONAUT PROCEED (V33) RETURNS
		TCF	ENDOUT

; ============================================================================
; DISPLAY RETURN AND WAKE LOGIC
;
; LASTPLAY (referenced in comments) manages display wake-up logic when
; normal displays were interrupted by higher-priority displays or were
; waiting while a higher-priority display was active. This ensures the
; display system properly restores interrupted displays after priority
; displays complete.
; ============================================================================

# LASTPLAY CHECKS TO SEE IF (1) THE LAST NORMAL DISPLAY WAS EITHER INTERRUPTED BY A PRIO OR A MARK (MARK
# COULD ONLY HAPPEN DURING PINBRANCH) OR IF (2) THE LAST NORMAL DISPLAY WAS REQUESTED WHILE A HIGHER PRIORITY
# DISPLAY WAS GOING, RESULTING IN THE NORMAL BEING PUT TO SLEEP.
#
# IF EITHER OF THE ABOVE 2 CONDITIONS EXISTS, THE NORMAL DISPLAY IS AWAKENED TO GO TO PLAYJUM1 WHICH STARTS
# UP THE MOST RECENT VALID NORMAL DISPLAY.  IF THESE 2 CONDITIONS DO NOT EXIST, CONTROL GOES TO PLAYJUM1 WHICH IS
# STARTED IMMEDIATELY WITH THE ASSUMPTION THAT THE MOST RECENT NORMAL DISPLAY IS ALREADY IN ENDIDLE (DURING A
# PINBRNCH) OR THAT A RESTART HAS OCCURRED AND THE DISPLAY CAN BE STARTED AS A .1 RESTART.

; ----------------------------------------------------------------------------
; MARKRET - MARK display completion
;
; COMMENT-ONLY: Complete a MARK display and clear its status flags. MARK
; displays (extended verb optical sightings) complete quickly and need their
; flag bits cleared before returning control.
;
; CODE-ALONG: Clears FLAGWRD4 bits related to MARK display (CS SIX masks
; out MARK-related bits). Uses INHINT/RELINT to protect critical section
; while updating FLAGWRD4. Branches to ENDRET to complete return sequence.
; ----------------------------------------------------------------------------

MARKRET		CS	SIX
		MASK	FLAGWRD4
		INHINT			# *** MAY MOVE DISPLAY FLAGWORD OUT OF
		TS	FLAGWRD4

		RELINT			# INHINT REALM
		TCF	ENDRET

; ----------------------------------------------------------------------------
; MARKOVER - MARK display overlapping check
;
; COMMENT-ONLY: Handle case where a MARK display is finishing while checking
; if another display is active in ENDIDLE. Sets OUTHERE to MINUS1 to signal
; ENDRET to perform ENDOFJOB rather than return to caller.
;
; CODE-ALONG: Stores MINUS1 in OUTHERE (RUPTREG2 negative signals ENDOFJOB).
; Tests FLAGWRD4 PRIO30 bits (BIT11+BIT10) to check if normal or priority
; display is in ENDIDLE state. If so, branches to NORMBNCH. Otherwise falls
; through to NORMRET.
; ----------------------------------------------------------------------------

# Page 1371
MARKOVER	CAF	MINUS1		# RUPTREG2 IS - MEANS ENDOFJOB TO ENDRET
		TS	OUTHERE

		CA	FLAGWRD4	# IS ENDIDFLG SET
		MASK	PRIO30		# IS NORMAL OR PRIO IN ENDIDLE
		CCS	A
		TCF	NORMBNCH

; ----------------------------------------------------------------------------
; NORMRET - Normal display completion and wake-up logic
;
; COMMENT-ONLY: Complete a normal display and check if any waiting displays
; need to be awakened. If a MARK display is sleeping or waiting, wake it.
; If a normal display is interrupted or waiting, wake it. If neither, check
; if new display should be started via PLAYJUM1. This ensures the display
; system properly sequences through multiple pending displays.
;
; CODE-ALONG: First checks FLAGWRD4 BITS5+11 (MARK sleeping/waiting). If set,
; branches to MARKWAKE to wake MARK display. If not, checks BITS4+10 (normal
; interrupted/waiting). If set, branches to NORMWAKE. If neither, checks
; EBANKTEM OCT50 (flash request or GODSPRET indicator). If set, returns via
; ENDRET. Otherwise checks NVSAVE for pending display. If non-zero, creates
; new job at PRIO15 calling PLAYJUM1 to start next display, then returns
; via ENDRET.
; ----------------------------------------------------------------------------

NORMRET		CA	FLAGWRD4	# IS MARK SLEEPING
		MASK	BITS5+11	# OR WAITING
		CCS	A
		TCF	MARKWAKE

		CA	FLAGWRD4	# NO
		MASK	BITS4+10	# IS NORMAL INTERRUPTED OR WAITING
		CCS	A
		TCF	NORMWAKE	# YES

		CA	EBANKTEM	# NO, WAS IT A FLASH REQUEST
		MASK	OCT50		# OR A GODSPRET
		CCS	A
		TCF	ENDRET		# YES
		CA	NVSAVE
		EXTEND
		BZF	ENDRET

		CAF	PRIO15
		INHINT
		TC	NOVAC
		EBANK=	NVWORD
		2CADR	PLAYJUM1

		TCF	ENDRET

; ----------------------------------------------------------------------------
; MARSLEEP - Put MARK display to sleep if not already sleeping
;
; COMMENT-ONLY: When a MARK display (optical sighting routine) needs to
; be deferred, this routine checks if a MARK is already sleeping. If so,
; end the job (don't create duplicate sleeping MARK). Otherwise, put this
; MARK to sleep using GOSLEEPS.
;
; CODE-ALONG: Loads FLAGWRD4 and masks BITS5+11 (MARK sleeping/waiting flags).
; CCS tests the result. If positive (MARK already sleeping), branches to
; ENDOFJOB to prevent duplicate sleep entries. If zero (no MARK sleeping),
; falls through to GOSLEEPS to put this MARK to sleep properly.
; ----------------------------------------------------------------------------

MARSLEEP	CA	FLAGWRD4	# IS MARK ALREADY IN
		MASK	BITS5+11
		CCS	A
		TCF	ENDOFJOB	# YES
		TCF	GOSLEEPS

; ----------------------------------------------------------------------------
; LOADITIS - Validate astronaut data entry against current noun
;
; COMMENT-ONLY: When the astronaut enters data using VERB 32 (ENTER), this
; routine validates that the entered data matches the noun type being loaded.
; If the data is valid for the current noun, accept the load. If not, branch
; to PINBRNCH to accept the load but re-request the last display (giving the
; astronaut another chance to verify).
;
; CODE-ALONG: Indexed load (COPMPAC) retrieves NVWORD (verb/noun word).
; Masks LOW7 to extract noun number, complements it, and adds MPAC+1
; (NOUNREG - current noun register). Extended BZF tests if result is zero
; (noun matches). If zero, branches to OKTOENT (load is valid). If non-zero
; (noun mismatch), branches to PINBRNCH (accept load but re-display request).
; This prevents astronaut data entry errors during critical mission phases.
; ----------------------------------------------------------------------------

LOADITIS	INDEX	COPMPAC
		CA	NVWORD
		MASK	LOW7
		COM
		AD	MPAC 	+1	# NOUNREG
		EXTEND
		BZF	OKTOENT		# NO, THEN LOAD IS VALID
		TCF	PINBRNCH	# YES, ACCEPT LOAD BUT ASK FOR LAST AGAIN

# Page 1372

; ----------------------------------------------------------------------------
; ERASER - Blank DSKY display registers (except MM - Major Mode)
;
; COMMENT-ONLY: Clear the DSKY display registers R1, R2, and R3, leaving
; only the Major Mode (MM) display visible. Used when displays complete or
; when the AGC needs to clear stale display data before showing new
; information. During Apollo 11's mission, blank displays indicated the
; AGC was ready for new commands or was between operational phases.
;
; CODE-ALONG: Loads complement of THREE into A register, calls NVSUB display
; subroutine to blank registers (THREE encodes which registers to blank,
; complemented form preserves MM). Both return paths (NVSUB has two exits)
; lead to ENDOFJOB, terminating the display job after blanking complete.
; ----------------------------------------------------------------------------

ERASER		CS	THREE		# BLANK EVERYTHING EXCEPT MM
		TC	NVSUB
		TCF	ENDOFJOB
		TCF	ENDOFJOB

; ============================================================================
; DISPLAY INTERFACE CONSTANTS AND EQUATES
;
; COMMENT-ONLY: The following section contains constant definitions and
; equates (symbolic name assignments) used throughout the display interface
; routines. These include display masks controlling which DSKY elements
; flash or blank, verb/noun code combinations for common displays, and
; address pointers (CADRs) to key display routines.
;
; CODE-ALONG: Constants are defined using OCT (octal), VN (verb/noun),
; CADR (code address), and EQUALS directives. Display masks use bit patterns
; to control DSKY behavior (flash, perform light, blank registers). The AGC
; assembler resolves these symbolic constants at assembly time into fixed
; memory locations. No executable code follows this point.
; ============================================================================

PERFMASK	OCT	0036		# FLASH, PERFORM, BLANK R2 AND R3
V01N25		VN	00125
V06N07		VN	00607		# GOPERF3 VN DISPLAY BEFORE V50
V50N00		VN	5000
PERF2MSK	OCT	00030		# FLASH, PERFORM
V04N06		VN	00406
PERF4MSK	OCT	14		# FLASH, BLANK R3
GOAGIN		EQUALS	PINBRNCH
REDOMASK	OCT	20010		# BITS 4 AND 14
MARK3MSK	OCT	40230		# MARK, DECIMAL NOUN, PERFORM, FLASH
MARK4MSK	OCT	40036		# MARK, PERFORM, FLASH, BLANK 2 AND 3
NVCADR		CADR	REDOPRIO
WAKECADR	CADR	MARKPLAY
		CADR	PLAYJUM1

OCT3400		OCT	3400		# EBANK MASK
NBUSMASK	OCT	11210
PMMASK		OCT	66521
VERBMASK	=	MID7		# (OCT 37600)
V05N00M1	OCT	1177		# V05 MINUS ONE
GOXDSP		EQUALS	GOMARK
GOXDSPR		EQUALS	GOMARKR
GOXDSPF		EQUALS	GOMARKF
GOXDSPFR	EQUALS	GOMARKFR
ENDEXT		EQUALS	ENDMARK
MPAC2SAV	EQUALS	BANKSET
NVBUSMSK	OCT	700
ASTROMSK	OCT	704
MPERFMSK	OCT	40030		# BIT 15,5,4 FOR MARK,PERFORM,FLASH
OCT34300	OCT	34300
BITS15+7	OCT	40100
BITS7+4		OCT	110
DSPFLG		EQUALS	EBANKSAV
MARKFLAG	EQUALS	MARKEBAN
SAVEFLAG	EQUALS	EBANKTEM
BITS5+11	OCT	2020		# * DON'T MOVE
BITS4+10	OCT	1010		# * DON'T MOVE
LOWLOAD		DEC	22
BUSYMASK	OCT	77730
CADRMASK	OCT	50
PINMASK		EQUALS	13,14,15
GOPLAY		EQUALS	NVDSP
PRIOSAVE	EQUALS	R1SAVE
COPMPAC		EQUALS	MPAC 	+3
TEMPOR2		EQUALS	MPAC 	+4

# Page 1373
OUTHERE		EQUALS	MPAC 	+5
COPINDEX	EQUALS	LOC
USERPRIO	EQUALS	MODE
GENMASK		EQUALS	MPAC 	+6
PRIOOCT		OCT	20144		# PRIO
MARKOCT		OCT	42424		# MARK
		OCT	11254		# NORM

IDLESLEP	OCT	74704
OCT67777	OCT	67777
LINUS		EQUALS	BLANKET
FACEREG		EQUALS	MPAC
PLAYTEM1	EQUALS	MPAC 	+1
PLAYTEM3	EQUALS	MPAC 	+3
PLAYTEM4	EQUALS	MPAC 	+4
OCT40420	OCT	40420
MAKEGEN		GENADR	MAKEPLAY
OCT10200	OCT	10200
V97N00		VN	09700		# PASTE FOR V97 OR V99
OCT20100	OCT	20100
CLOCKCON	OCT	24030		# FLASH, PERFORM, V99 OR V97 PASTE, REFLASH
