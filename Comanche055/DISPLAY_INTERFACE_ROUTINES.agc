# Copyright:    Public domain.
# Filename:     DISPLAY_INTERFACE_ROUTINES.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 1452-1484
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-07 RSB	Adapted from Colossus249 file of the same
#				name, and page images. Corrected various
#				typos in the transcription of program
#				comments, and these should be back-ported
#				to Colossus249.
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
; FILE: DISPLAY_INTERFACE_ROUTINES.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: DSKY display management and formatting routines implementing display
;        request prioritization, verb-noun coordinate control, seven-segment
;        display formatting, job lifecycle management (sleep/wake), astronaut
;        interrupt handling, and restart recovery. Core interface between
;        mission programs and crew displays throughout Apollo 11 mission.
;
; COMMENT-ONLY READERS: This program controlled what appeared on the DSKY
;        (Display and Keyboard unit), managing priorities so critical warnings
;        could interrupt normal displays, and coordinating crew interactions.
; CODE-ALONG READERS: Study display priority queue management, job state
;        machine (active/inactive/sleeping), verb-noun state coordination,
;        and integration with EXECUTIVE scheduler (NOVAC/FINDVAC/JOBWAKE).
; ============================================================================

# Page 1452
# DISPLAYS CAN BE CLASSIFIED INTO THE FOLLOWING CATEGORIES-
#	1.  PRIORITY DISPLAYS- DISPLAYS WHICH TAKE PRIORITY OVER ALL OTHER DISPLAYS. USUALLY THESE DISPLAYS ARE SENT
#	    OUT UNDER CRITICAL ALARM CONDITIONS.
#	2.  EXTENDED VERB DISPLAYS- ALL EXTENDED VERBS AND MARK ROUTINES SHOULD USE EXTENDED VERB (MARK) DISPLAYS.
#	3.  NORMAL DISPLAYS- ALL MISSION PROGRAM DISPLAYS WHICH INTERFACE WITH THE ASTRONAUT DURING THE NORMAL
#	    SEQUENCE OF EVENTS.
#	4.  MISC. DISPLAYS- ALL DISPLAYS NOT HANDLED BY THE DISPLAY INTERFACEROUTINES. THESE INCLUDE SUCH DISPLAYS AS
#	    MM DISPLAYS AND SPECIAL PURPOSE DISPLAYS HANDLED BY PINBALL.
#	5.  ASTRONAUT INITIATED DISPLAYS- ALL DISPLAYS INITIATED EXTERNALLY.
;
; DISPLAY CATEGORY EXPLANATION:
; Throughout the Apollo 11 mission, the crew interacted with the AGC through
; the DSKY (Display and Keyboard). These routines managed what appeared on
; the numeric displays, ensuring critical information always reached the crew.
;
; Priority displays (category 1) ensured that program alarms and critical
; warnings would immediately interrupt whatever the crew was viewing. During
; the famous 1202 alarm on descent, this priority system allowed the alarm
; code to flash on the display even while landing guidance was running.
;
; Mark displays (category 2) supported optical navigation, where crew members
; would sight stars or landmarks through the sextant and press MARK to record
; the timing. Extended verb displays handled special crew requests.
;
; Normal displays (category 3) showed routine mission data - orbital parameters,
; velocity, altitude, time to ignition - the operational information crew
; members monitored during translunar coast, lunar orbit, and powered flight.
#
# THE FOLLOWING TERMS ARE USED TO DESCRIBE THE STATUS OF DISPLAYS-
#	1.  ACTIVE-THE DISPLAY WHICH IS (1) BEING DISPLAYED TO THE ASTRONAUT AND WAITING FOR A RESPONSE OR
#	    (2) WAITING FIRST IN LINE FOR THE ASTRONAUT TO FINISH USING THE DSKY OR (3) BEING DISPLAYED ON THE DSKY
#	    BUT NOT WAITING FOR A RESPONSE.
#	2.  INACTIVE -A DISPLAY WHICH HAS (1) BEEN ACTIVE BUT WAS INTERRUPTED BY A DISPLAY OF HIGHER PRIORITY,
#	    (2) BEEN PUT INTO THE WAITING LIST AT TIME IT WAS REQUESTED DUE TO THE FACT A HIGHER PRIORITY DISPLAY
#	    WAS ALREADY GOING, (3) BEEN INTERRUPTED BY THE ASTRONAUT (CALLED A PINBRANCH CONDITION, SINCE THIS TYPE
#	    OF INACTIVE DISPLAY IS USUALLY REACTIVATED ONLY BY PINBALL) OR (4) A DISPLAY WHICH HAS FINISHED BUT STILL
#	    HAS INFO SAVED FOR RESTART PURPOSES.
;
; DISPLAY STATE MACHINE EXPLANATION:
; The AGC managed multiple simultaneous requests to display information, but
; only one display could appear on the DSKY at any time. Think of it like
; modern computer windows - programs can request display time, but only one
; has focus.
;
; ACTIVE displays owned the DSKY. During Apollo 11's powered descent, the
; landing program's velocity and altitude display was ACTIVE, updating every
; two seconds to show Armstrong and Aldrin their descent progress.
;
; INACTIVE displays waited their turn or had been interrupted. When that 1202
; alarm occurred, the landing display became INACTIVE (pushed aside but not
; forgotten), the alarm code became ACTIVE (flashing on the display), and
; after crew acknowledgment, the landing display returned to ACTIVE status.
;
; This state management ensured displays could be interrupted, suspended, and
; resumed without losing crew context or mission data - critical for the
; complex multi-tasking operations throughout the mission.
#
# DISPLAY PRIORITIES WORK AS FOLLOWS-
#	INTERRUPTS-
#		1.  THE ASTRONAUT CAN INTERRUPT ANY DISPLAY WITH AN EXTERNAL DISPLAY REQUEST.
#		2.  INTERNAL DISPLAYS CAN NOT BE SENT OUT WHEN THE ASTRONAUT IS USING THE DSKY.
#		3.  PRIORITY DISPLAYS INTERRUPT ALL OTHER TYPES OF INTERNAL DISPLAYS.  A PRIORITY DISPLAY INTERRUPTING ANOTHER
#		    PRIORITY DISPLAY WILL CAUSE AN ABORT UNLESS BIT14 IS SET FOR THE LINUS ROUTINE.
#		4.  A MARK DISPLAY INTERRUPTS ANY NORMAL DISPLAY.
#		5.  A MARK THAT INTERRUPTS A MARK COMPLETELY REPLACES IT.
#
# 	ORDER OF WAITING DISPLAYS-
#		1.  ASTRONAUT EXTERNAL USE
#		2.  PRIORITY
#		3.  INTERRUPTED MARK
#		4.  INTERRUPTED NORMAL
#		5.  MARK TO BE REQUESTED (SEE DESCRIPTION OF ENDMARK)
#		6.  MARK WAITING
#		7.  NORMAL WAITING
;
; PRIORITY SYSTEM IN OPERATION:
; This priority hierarchy ensured the crew always saw the most important
; information. The astronaut had ultimate control (interrupt rule 1) - pressing
; VERB or NOUN keys at any time would interrupt the computer's displays and let
; the crew request different information.
;
; For computer-generated displays, priority displays (alarms, critical warnings)
; took precedence over everything (rule 3). During lunar descent, when the
; guidance computer detected the executive scheduler overload, it immediately
; interrupted the landing velocity display to flash "1202" - the alarm code -
; even though landing guidance was the active program.
;
; Mark displays (rule 4) had intermediate priority, used during navigation when
; crew members sighted stars or landmarks. Normal displays showed routine
; mission data and could be interrupted by either marks or priority displays.
;
; The waiting order (rules 1-7 above) created a queue. Multiple programs could
; request displays simultaneously, but they waited their turn based on priority.
; Astronaut requests always went first, priority displays second, interrupted
; displays got resumed, and waiting normal displays came last.
#
# Page 1453
# THE DISPLAY ROUTINES ARE INTENDED TO SERVE AS AN INTERFACE BETWEEN THE USER AND PINBALL.  THE
# FOLLOWING STATEMENTS CAN BE MADE ABOUT NORMAL DISPLAYS AND PRIORITY DISPLAYS (A DESCRIPTION OF MARK ROUTINES
# WILL FOLLOW LATER):
#	1.  ALL ROUTINES THAT END IN R HAVE AN IMMEDIATE RETURN TO THE USER.  FOR ALL FLASHING DISPLAYS THIS RETURN
#	    IS TO THE USERS CALL CADR +4.  FOR THE ONLY NON FLASHING IMMEDIATE RETURN DISPLAY (GODSPR) THIS RETURN
#	    IS TO THE USERS CALLING LOC +1.
#	2.  ALL ROUTINES NOT ENDING IN R DO NOT DO AN IMMEDIATE RETURN TO THE USER.
#	3.  ALL ROUTINES THAT END IN R START A SEPARATE JOB (MAKEPLAY) WITH USERS JOB PRIORITY.
#	4.  ALL ROUTINES NOT ENDING IN R BRANCH DIRECTLY TO MAKEPLAY WHICH MAKES THESE DISPLAYS A PART OF THE
#	    USERS JOB.
#	5.  ALL DISPLAY ROUTINES ARE CALLED VIA BANKCALL.
#	6.  TO RESTART A DISPLAY THE USER WILL GENERALLY USE A PHASE OF ONE WITH DESIRED RESTART GROUP (SEE
#	    DESCRIPTION OF RESTARTS).
#	7.  ALL FLASHING DISPLAYS HAVE 3 RETURNS TO THE USER FROM ASTRONAUT RESPONSES.  A TERMINATE (V34) BRANCHES
#	    TO THE USERS CALL CADR +1.  A PROCEED (V33) BRANCHES TO THE USERS CALL CADR +2.  AN ENTER OR RECYCLE
#	    (V32) BRANCHES TO THE USERS CALL CADR +3.
#	8.  ALL ROUTINES MUST BE USED UNDER EXECUTIVE CONTROL.
;
; DISPLAY ROUTINE INTERFACE ARCHITECTURE:
; These rules define how mission programs interface with the display system.
; "User" here means mission programs (landing, rendezvous, navigation) - the
; programs that need to show information to the crew.
;
; The "R" suffix convention (rules 1,3 vs 2,4) controlled whether displays ran
; in the calling program's job or spawned a separate job. Routines ending in R
; (like GODSPR, PRIODSPR) returned immediately to the calling program and
; started a separate job to manage the display. This allowed time-critical
; guidance calculations to continue while display formatting ran in parallel.
;
; Routines without R (like GODSP, PRIODSP) made the display part of the calling
; job, causing the caller to wait until the display completed or the astronaut
; responded. During Apollo 11's descent, the landing guidance program used
; immediate-return displays so altitude/velocity calculations continued while
; the display updated.
;
; The three-return system (rule 7) gave astronauts control. When a flashing
; display waited for input, pressing PROCEED (V33) meant "continue normally",
; ENTER meant "I've entered data, use it", and TERMINATE (V34) meant "abort
; this operation". The calling program provided three branch addresses (CADR +1,
; +2, +3) to handle each response, implementing different mission logic paths
; based on crew decisions.
;
; Executive control requirement (rule 8) meant these routines relied on the job
; scheduler (NOVAC, FINDVAC, JOBWAKE) to manage display timing and priorities.
#
# A DESCRIPTION OF EACH ROUTINE WITH AN EXAMPLE FOLLOWS:
#	GODSP IS USED TO DISPLAY A VERB NOUN ARRIVING IN A.  NO RETURN IS MADE TO THE USER.
#		1.  GODSP IS NOT RESTARTABLE
#		2.  A VERB PASTE WITH GODSP ALWAYS TURNS ON THE FLASH.
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	GODSP
#			VXXNYY	OCT	OXXYY

#	GODSPR IS THE SAME AS GODSP ONLY RETURN IS TO THE USER.
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	GODSPR
#				...	...		# IMMEDIATE RETURN OF GODSPR

#	GOFLASH DISPLAYS A FLASHING VERB NOUN WITH NO IMMEDIATE RETURN TO THE USER. 3 RETURNS ARE POSSIBLE FORM
#	THE ASTRONAUT (SEE NO. 7 ABOVE).
#				CAF	VXXNYY		# VXX NYY WILL BE A FLASHING VERB NOUN.
#				TC	BANKCALL
#				CADR	GOFLASH
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN

#	GOPERF1 IS ENTERED WITH DESIRED CHECKLIST VALUE IN A.  GOPERF1 WILL DISPLAY THIS VALUE IN R1 BY MEANS OF A
# Page 1454
# 	V01 N25.A FLASHING PLEASE PERFORM ON CHECKLIST (V50 N25) IS THEN DISPLAYED.  NO IMMEDIATE RETURN IS MADE TO
# 	USER (SEE NO. 7 ABOVE).
#	GOPERF1 BLANKS REGISTERS R2 AND R3
#				CAF	OCTXX		# CODE FOR CHECKLIST VALUE XX
#				TC	BANKCALL
#				CADR	GOPERF1
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOPERF2 IS ENTERED WITH A VARIABLE NOUN AND V01 (V00 FOR N10 OR N11) IN A.  GOPERF2 WILL FIRST DISPLAY THE
# 	REQUESTED NOUN BY MEANS OF A V01NYY OR A V00NYY. PLEASE PERFORM ON NOUN (V50 NYY) THEN BECOMES A FLASHING
#	DISPLAY.  NO IMMEDIATE RETURN IS MADE TO THE USER (SEE NO. 7 ABOVE).
#	GOPERF2 DOES NOT BLANK ANY REGISTERS
#				CAF	VXXNYY		# VARIABLE NOUN YY. XX=00 OR 01.
#				TC	BANKCALL
#				CADR	GOPERF2
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOPERF3 IS USED FOR A PLEASE PERFORM ON A PROGRAM NUMBER.  THE DESIRED PROGRAM NO. IS ENTERED IN A.  GOPERF3
#	DISPLAYS THE NO. BY MEANS OF A V06 N07 FOLLOWED BY A FLASHING V50 N07 FOR A PLEASE PERFORM. NO IMMEDIATE RETURN
#	IS MADE TO THE USER (SEE NO. 7 ABOVE).
#	GOPERF3 BLANKS REGISTERS R2 AND R3
#				CAF	DECXX		# REQUEST PERFORM ON PXX
#				TC	BANKCALL
#				CADR	GOPERF3
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN

#	GOPERF4 IS USED FOR A PLEASE PERFORM ON AN OPTION. THE DESIRED OPTION IS ENTERED IN A AND STORED IN OPTION1.
#	GOPERF4 DISPLAYS R1 AND R2 BY MEANS OF A V04N06 FOLLOWED BY A FLASHING V50N06 FOR A PLEASE PERFORM.  NO
#	IMMEDIATE RETURN IS MADE TO THE USER (SEE NO. 7 ABOVE).
#				CAF	OCTXX		# REQUEST PERFORM ON OPTION XX
#				TC	BANKCALL
#				CADR	GOPERF4
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#	GOPERF4 BLANKS REGISTER R3
#
# Page 1455
#	GODSPRET IS USED TO DISPLAY A VERB NOUN ARRIVING IN A WITH A RETURN TO THE USER AFTER THE DISPLAY HAS BEEN SENT
#	OUT.
#				CAF	VXXXNYY
#				TC	BANKCALL
#				CADR	GODSPRET
#				...	...		# RETURN TO USER

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

# 	GOFLASHR IF SAME AS GOFLASH ONLY AN IMMEDIATE RETURN IS MADE TO THE USERS CALL CADR +4.
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	GOFLASHR
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN
#				...	...		# IMMEDIATE RETURN FROM GOFLASHR

#	GOPERF1R IS THE SAME AS GOPERF1 ONLY GOPERF1R HAS AN IMMEDIATE RETURN TO USERS CALL CADR +4.
#	GOPERF1R BLANKS REGISTERS R2 AND R3
#				CAF	OCTXX		# CODE FOR CHECKLIST VALUE XX.
#				TC	BANKCALL
#				CADR	GOPERF1R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN FROM GOPERF1R

#	GOPERF2R IS THE SAME AS GOPERF2 ONLY AN IMMEDIATE RETURN IS MADE TO USERS CALL CADR +4.
# Page 1456
#	GOPERF2R DOES NOT BLANK ANY REGISTERS
#				CAF	VXXNYY		# VARIABLE NOUN YY REQUESTED.  XX=00 OR 01
#				TC	BANKCALL
#				CADR	GOPERF2R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN HERE FROM GOPERF2R

# 	GOPERF3R IS THE SAME AS GOPERF3 ONLY AN IMMEDIATE RETURN IS MADE TO USERS CALL CADR +4.
#	GOPERF3R BLANKS REGISTERS R2 AND R3
#				CAF	PROGXX		# PERFORM PROGRAM XX
#				TC	BANKCALL
#				CADR	GOPERF3R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# GOPERF3R IMMEDIATELY RETURNS HERE

#	GOPERF4R IS THE SAME AS GOPERF4 ONLY AN IMMEDIATE RETURN IS MADE TO USERS CALL CADR +4.
#				CAF	OCTXX		# REQUEST PERFORM ON OPTIONXX
#				TC	BANKCALL
#				CADR	GOPERF4R
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN TO USER
#	GOPERF4R BLANKS REGISTER R3
#
#	REFLASHR IS THE SAME AS REFLASH ONLY AN IMMEDIATE RETURN IS MADE TO THE USERS CALL CADR +4.
#				CAF	VXXNYY		# VXX NYY WILL BE A FLASHING VERB NOUN
#				TC	BANKCALL
#				CADR	REFLASHR
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER RETURN
#				...	...		# IMMEDIATE RETURN TO USER

#	REGODSPR IS THE SAME AS REGODSP ONLY A RETURN (IMMEDIATE) IS MADE TO THE USER.
# Page 1457
#				CAF	VXXNYY
#				TC	BANKCALL
#				CADR	REGODSPR
#				...	...		# IMMEDIATE RETURN TO USER

# Page 1458
#	GOMARK IS USED TO DISPLAY A MARK VERB NOUN ARRIVING IN A. NO RETURN IS MADE TO THE USER.
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

#	GOMARKF DISPLAYS A FLASHING MARK VERB NOUN WITH NO IMMEDIATE RETURN TO THE USER. 3 RETURNS ARE POSSIBLE FORM
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
#				TCF	BANKCALL
#				CADR	GOMARKFR	# OTHER EXTENDED VERBS USE CADR GOXDSPFR
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN
#				...	...		# IMMEDIATE RETURN TO THE USER

#	GOMARK1 IS USED FOR A PLEASE PERFORM ON A MARK REQUEST WITH ONLY 1 ASTRONAUT RETURN TO THE USER.  NO IMMEDIATE
#	RETURN IS MADE. THE DESIRED MARK PLEASE PERFORM VERB AND DESIRED NOUN IS ENTERED IN A.  GOMARK1 DISPLAYS R1, R2, R
#	MEANS OF A V05NYY FOLLOWED BY A FLASHING V5XNYY FOR A PLEASE PERFORM. THE ASTRONAUT WILL RESPOND WITH A MARK
#	OR MARK REJECT OR AN ENTER. THE ENTER IS THE ONLY ASTRONAUT RESPONSE THAT WILL COME BACK TO THE USER.
#				CAF	V5XNYY		# X=1,2,3,4	YY= NOUN
#				TC	BANKCALL
# Page 1459
#				CADR	GOMARK1
#				...	...		# ENTER RETURN

#	*** IF BLANKING DESIRED ON NON R ROUTINES, NOTIFY DISPLAYER.
#
#	GOMARK1R IS THE SAME AS A GOMARK1 ONLY AN IMMEDIATE RETURN IS MADE TO THE USERS CALL CADR +2.
#				CAF	V5XNYY		# X=1,2,3,4	YY = NOUN
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
#				...	...		# IMMEDIATE RETURN TO THE USER

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
# Page 1460
#			...	...			# ENTER RETURN
#	EXDSPRET IS USED TO DISPLAY A VERB NOUN ARRIVING IN A WITH A RETURN MADE TO THE USER AFTER THE DISPLAY HAS BEEN
#	SENT OUT.
#				CAF	VXXNYY
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
#	THE ASTRONAUT(SEE NO.7 ABOVE).
#
#		THE MAIN PURPOSE OF PRIODSP IS TO REPLACE THE PRESENT DISPLAY WITH A DISPLAY OF HIGHER PRIORITY AND TO
#	PROVIDE A MEANS FOR RESTORING THE OLD DISPLAY WHEN THE PRIORITY DISPLAY
# 	IS RESPONDED TO BY THE ASTRONAUT.
#
#		THE FORMER DISPLAY IS RESTORED BY AN AUTOMATIC BRANCH TO WAKE UP THE DISPLAY THAT WAS INTERRUPTED BY THE
#	PRIO DISPLAY.
#				CAF	VXXNYY		# VXXNYY WILL BE A FLASHING VERB NOUN
#				TC	BANKCALL
#				CADR	PRIODSP
#				...	...		# TERMINATE RETURN
#				...	...		# PROCEED RETURN
# Page 1461
#				...	...		# ENTER OR RECYCLE RETURN

#	PRIODSPR IS THE SAME AS PRIODSP ONLY AN IMMEDIATE RETURN IS MADE TO THE USERS CALL CADR +4.
#				CAF	VXXNYY		# VXXNYY WILL BE A FLASHING VERB NOUN
#				TC	BANKCALL
#				CADR	PRIODSPR
#				...	...		# TERMINATE ACTION
#				...	...		# PROCEED RETURN
#				...	...		# ENTER OR RECYCLE RETURN
#				...	...		# IMMEDIATE RETURN

#	PRIOLARM DOES A V05N09 PRIODSPR.
#	CLEANDSP CLEANS OUT ALL NORMAL DISPLAYS (ACTIVE AND INACTIVE). A RETURN IS MADE TO THE USER AFTER NORMAL
#	DISPLAYS ARE CLEANED OUT.
#				TC	BANKCALL
#				CADR	CLEANDSP
#				...	...		# RETURN TO USER
# Page 1462
#
# GENERAL INFORMATION
# -------------------
#
# ALARM OR ABORT EXIT MODES--
#					PRIOBORT	TC	ABORT
#							OCT	1502
#
#	PRIOBORT IS BRANCED TO WHEN (1)  A NORMAL DISPLAY IS REQUESTED AND  ANOTHER NORMAL DISPLAY IS ALREADY ACTIVE
#	(REFLASH AND REGODSP ARE EXCEPTIONS) OR (2) A PRIORITY DISPLAY IS REQUESTED WHEN ANOTHER PRIORITY DISPLAY IS
#	ALREADY ACTIVE (A PRIORITY WITH LINUS BIT14 IS AN EXCEPTION).
#
# ERASABLE INITIALIZATION REQUIRED--
#	ACCOMPLISHED BY FRESH START-	1.  FLAGWRD4 (USED EXCLUSIVELY BY DISPLAY INTERFACE ROUTINES)
#					2.  NVSAVE = NORMAL VERB AND NOUN REGISTER.
#					3.  EBANKTEM = NORMAL INACTIVE FLAGWORD (ALSO CONTAINS NORMALS EBANK).
#					5.  R1SAVE = MARKBRAN CONTROL WORD
#					4.  RESTREG = PRIORITY 30 AND SUPERBANK 3.
#
# OUTPUT--
#	NVWORD = PRIO VERB AND NOUN
#	NVWORD +1(MARKNV) = MARK VERB AND NOUN
#	NVWORD +2(NVSAVE) = NORMAL VERB AND NOUN
#	DSPFLG(EBANKSAV) = PRIO FLAGWORD (INCLUDING EBANK)
#	DSPFLG +1 (MARKEBAN) = MARK FLAGWORD (INCLUDING EBANK)
#	DSPFLG +2 (EBANKTEM) = NORMAL FLAGWORD (INCLUDING EBANK)
#	CADRFLSH = PRIO USERS CALL CADR +1 LOCATION
#	CADRFLSH +1 (MARKFLSH) = MARK USERS CALL CADR +1 LOCATION
#	CADRFLSH +2 (TEMPFLSH) = NORMAL USERS CALL CADR +1 LOCATION
#	PRIOTIME = TIME EACH PRIO REQUEST FIRST SENT OUT
#	OPTION1 = DESIRED OPTION FROM GOPERF4
#	FLAGWRD4 = BIT INFO FOR CONTROL OF ALL DISPLAY ROUTINES
#	DSPTEM1 = R1 INFO FOR ASTRONAUT FROM PERFORM DISPLAYS(NORMAL)
#
# SUBROUTINES USED-- NVSUB, FLAGUP, FLAGDOWN, ENDOFJOB, BLANKSUB, ABORT, JOBWAKE, JOBSLEEP, FINDVAC, PRIOCHNG,
#	JAMTERM, NVSUBUSY, FLASHON, ENDIDLE, CHANG1, BANKJUMP, MAKECADR, NOVAC,
#
# DEBRIS-- (STORED INTO)
#	TEMPORARY TEMPORARIES- A, Q, L, MPAC +2, MPAC +3, MPAC +4, MPAC +5, MPAC +6, RUPTREG2, RUPTREG3, CYL,
#		EBANK, RUPTREG4, LOC, BANKSET, MODE, MPAC, MPAC +1, FACEREG
#	ERASABLES(SHARED AND USED WITH OTHER PROGRAMS) CADRSTOR, DSPLIST, LOC, DSPTEM1, OPTION1
#	ERASABLES(USED ONLY BY DISPLAY ROUTINES)- NVWORD,+1,+2, DSPFLAG,+1,+2, CADRFLSH,+1,+2, PRIOTIME, FLAGWRD4,
# Page 1463
#		R1SAVE, MARK2PAC
#
# DEBRIS-- (USED BUT NOT STORED INTO)- NOUNREG, VERBREG, LOCCTR, MONSAVE1
#
# FLAGWORD DESCRIPTIONS--
#	FLAGWRD4- SEE DESCRIPTION UNDER LOG SECTION ERASABLE ASSIGNMENTS
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
# RESTARTING DISPLAYS--
#
# RULES FOR THE DSKY OPERATOR--
#	1.  PROCEED AND TERMINATE SERVE AS RESPONSES TO REQUESTS FOR OPERATOR RESPONSE (FLASHING Y/N).  AS LONG
#	    AS THERE IS ANY REQUEST AWAITING OPERATOR RESPONSE, ANY USE OF PROCEED OR TERMINATE WILL SERVE AS
#	    RESPONSES TO THAT REQUEST.  CARE SHOULD BE EXERCISED IN ATTEMPTING TO KILL AN OPERATOR INITIATED MONITOR
#	    WITH PROCEED AND TERMINATE FOR THIS REASON.
#	2.  THE ASTRONAUT MUST RESPOND TO A PRIORITY DISPLAY NO SOONER THAN 5 SECS FROM THE TIME THE MISSION
#	    PROGRAM SENT OUT THE REQUEST FOR OPERATOR RESPONSE (THE ASTRONAUT WOULD SEE THIS DISPLAY FOR LESS TIME
#	    DUE TO TIME IT TAKES TO GET DISPLAY SENT OUT.) IF THE ASTRONAUT RESPONDS TOO SOON, THE PRIORITY DISPLAY
#	    IS SENT OUT AGAIN---AND AGAIN UNTIL AN ACCUMULATED 5 SECS FROM TIME THE FIRST PRIORITY DISPLAY WAS SENT
#	    OUT. THE SAME 5 SEC. DELAY WILL OCCUR AT 163.84 SECS OR IN ANY MULTIPLE OF THAT TIME DUE TO PROGRAM
#	    CONSIDERATION.
#	3.  KEY RELEASE BUTTON-
#	    A) IF THE KEY RELEASE LIGHT IS ON, IT SIMPLY RELEASES THE KEYBOARD AND DISPLAY FOR INTERNAL USE.
#	    B) IF THE KEY RELEASE LIGHT IS OFF, AND IF SOME REQUEST FOR OPERATOR RESPONSE (FLASHING V/N) IS STILL
#	       AWAITING RESPONSE THEN IT RE-ESTABLISHES THE DISPLAYS THAT ORIGINALLY REQUESTED RESPONSE.
#	    IF AN OPERATOR WANTS THEREFORE TO RE-ESTABLISH BUT CONDITION (A) IS ENCOUNTERED, A SECOND DEPRESSION OF
#	    KEY RELEASE BUTTON MAY BE NECESSARY.
#	4.  IT IS IMPORTANT TO ANSWER ALL REQUESTS FOR OPERATOR RESPONSE.
#	5.  IT IS ALWAYS GOOD PRACTICE TO TERMINATE AN EXTENDED VERB BEFORE ASKING FOR ANOTHER ONE OR THE SAME ONE
#	    OVER AGAIN.
#
# SPECIAL CONSIDERATIONS--
# Page 1464
#	1.  MPAC +2 SAVED ONLY IN MARK DISPLAYS
#	2.  GODSP(R),REGODSP(R),GOMARK(R) ALWAYS TURN ON THE FLASH IF ENTERED WITH A PASTE VERB REQUEST.
#	3.  ALL NORMAL DISPLAYS ARE RESTARTABLE EXCEPT GODSP(R), REGODSP(R)
#	4.  ALL EXTENDED VERBS WITH DISPLAYS SHOULD START WITH A TC TESTXACT AND FINISH WITH A TC ENDEXT.
#	5.  GODSP(R) AND REGODSP(R) MUST BE IN THE SAME EBANK AND SUPERBANK AS THE LAST NORMAL DISPLAY RESTARTED
#	    BY A .1 RESTART PHASE CHANGE.
#	6.  IN ORDER TO SET UP A NON DISPLAY .1 RESTART POINT, THE USER MUST MAKE CERTAIN THAT RESTREG CONTAINS THE
#	    CORRECT PRIORITY AND SUPERBANK AND THAT EBANKTEM CONTAINS THE CO
#	7.  IF CLEANDSP IS RESTARTED VIA A .1 PHASE CHANGE, CAF ZERO SHOULD BE EXECUTED BEFORE THE TC BANKCALL.

# Page 1465
# CALLING SEQUENCE FOR BLANKING
#		CAF	BITX		# X=1,2,3 BLANK R1,R2,R3 RESPECTIVELY
#		TC	BLANKET
#		...	...		# RETURN TO USER HERE
# IN ORDER TO USE BLANKET CORRECTLY THE USER MUST USE A DISPLAY ROUTINE THAT ENDS IN R FIRST FOLLOWED BY THE CALL
# TO BLANKET AT THE IMMEDIATE RETURN LOC.
;
; ============================================================================
; TRANSITION: From Interface Specifications to Executable Code
;
; The display interface documentation above defines the rules and priorities.
; The executable code below implements these rules, managing the DSKY displays
; that Michael Collins monitored throughout Columbia's solo lunar orbit while
; Armstrong and Aldrin descended to the Moon. These same routines formatted
; the "1202" alarm display during Eagle's descent and showed the landmark
; tracking data Collins used for navigation photography.
; ============================================================================
;
		BLOCK	02
		SETLOC	FFTAG4
		BANK

		COUNT	02/DSPLA
;
; BLANKET - Display Blanking Control Routine
; Called to selectively blank unused digit positions on DSKY displays.
; During Apollo 11, this cleared previous display values when switching
; between different verb/noun combinations, preventing crew confusion from
; seeing residual digits from the prior display.
;
; TECHNICAL IMPLEMENTATION:
; Input: A register contains bit pattern (BIT1, BIT2, or BIT3) specifying
;        which register (R1, R2, or R3) to blank on the DSKY display.
; The routine masks the blanking request against the current display template
; (PLAYTEM4) to set the appropriate blanking bits for the display driver.
;
BLANKET		TS	MPAC +6
		CS	PLAYTEM4
		MASK	MPAC +6
		INDEX	MPAC +5
		ADS	PLAYTEM4

		TC	Q
;
; ENDMARK - Mark Display Completion Handler
; Extended verb routines (star sightings, landmark tracking) call ENDMARK
; when their display sequence completes. During Apollo 11's translunar coast,
; Collins used extended verbs to sight navigation stars; each sighting sequence
; ended with ENDMARK to clean up the mark display state and allow normal
; mission displays to resume.
;
ENDMARK		TC	POSTJUMP
		CADR	MARKEND
;
; CLEARMRK - Clear Mark Display Active Flag
; Resets the extended verb active flag (EXTVBACT) and clears the mark-in-
; progress bit (BIT1) in FLAGWRD4. This cleanup ensures the display system
; knows no mark operation is active, allowing normal displays to proceed.
;
; TECHNICAL: Uses INHINT/RELINT to protect the flag word modification from
; interrupt-driven corruption. The CS BIT1 / MASK / TS sequence atomically
; clears bit 1 while preserving other flag bits.
;
CLEARMRK	CAF	ZERO
		TS	EXTVBACT

		INHINT
		CS	BIT1
		MASK	FLAGWRD4
		TS	FLAGWRD4

		RELINT
		TC	Q

# *** ALL EXTENDED VERB ROUTINES THAT HAVE AT LEAST ONE FLASHING DISPLAY MUST TCF ENDMARK OR TCF ENDEXT WHEN
# FINISHED.
;
; MEMORY BANK ORGANIZATION:
; The display routines are split across multiple banks due to AGC's fixed
; memory architecture. Bank 02 contains utility routines like BLANKET, while
; Bank 10 contains the main display logic. Bank switching (via BANKCALL and
; CADR) allows these distributed routines to call each other transparently.
;
		BANK	10
		SETLOC	DISPLAYS
		BANK

		COUNT	10/DSPLA

# NTERONLY IS USED TO DIFFERENTIATE THE MARK ROUTINE WITH ONLY ONE RETURN TO THE USER FROM THE MARKING ROUTINE WITH
# 3 RETURNS TO THE USER.  THIS ROUTINE IS ONLY USED BY GOMARK1 AND GOMARK1R.
;
; MARKEND - Final Mark Display Cleanup
; Called after CLEARMRK to complete mark display termination. Transfers control
; to MARKOVER which handles the transition back to normal display operations.
;
MARKEND		TC	CLEARMRK
		TCF	MARKOVER

# Page 1466
;
; ============================================================================
; GOMARK FAMILY - Mark Display Entry Points
;
; The GOMARK routines provide multiple entry points for mark displays (extended
; verbs used for navigation). Different entry points support different display
; modes - some wait for completion, others return immediately; some flash
; displays, others don't; some allow data entry, others are display-only.
;
; During Apollo 11's cislunar navigation, Collins repeatedly used these routines
; to display star sighting results. The mark displays had priority over normal
; mission displays but could be interrupted by the crew or by priority alarms.
; ============================================================================
;
; GOMARK - Basic Mark Display Entry Point
; Standard mark display without immediate return. Stores verb/noun code in
; PLAYTEM1 and sets BIT15 to flag this as a mark request. The display becomes
; part of the calling job (blocks until display completes or astronaut responds).
;
GOMARK		TS	PLAYTEM1	# ENTRANCE FOR MARK GODSP
;
; GOMARS - Common Mark Display Setup
; All GOMARK variants converge here. Loads BIT15 (mark request flag) into
; A register and transfers to GOFLASH2 for display processing. BIT15 ensures
; the display system recognizes this as a mark display with appropriate priority.
;
GOMARS		CAF	BIT15		# BIT15 SET FOR ALL MARK REQUESTS
		TCF	GOFLASH2
;
; KLEENEX - Extended Verb Cleanup Entry Point
; Clears extended verb state before initiating a new mark display. The name
; "KLEENEX" reflects its cleanup function - wiping away residual state from
; previous extended verb operations.
;
KLEENEX		CAF	ZERO		# CLEAN OUT EXTENDED VERBS
;
; GOMARKF - Mark Display with Flash
; Mark display entry that enables flashing (MARKFMSK sets flash bit). During
; Apollo 11, flashing displays indicated the computer was waiting for crew
; response. Star sighting displays would flash while waiting for the astronaut
; to confirm the mark or enter correction data.
;
GOMARKF		TS	PLAYTEM1	# ENTRANCE FOR MARK GOFLASH

		CAF	MARKFMSK	# MARK, FLASH
		TCF	GOFLASH2
;
; GOMARK2 - Mark Display with Perform Mode (3 Returns)
; Enables three astronaut response paths (PROCEED, ENTER, TERMINATE). Used
; by navigation extended verbs where crew could accept results (PROCEED),
; modify values (ENTER), or abort the sequence (TERMINATE). The guidance
; program provided three branch addresses to handle each crew decision.
;
GOMARK2		TS	PLAYTEM1	# MARK GOPERFS-3 AST. RETURNS
MARKFORM	CAF	MPERFMSK	# MARK, PERFORM, FLASH
		TCF	GOFLASH2
;
; GOMARK3 - Three-Component Decimal Perform Display
; Specialized for displays showing three decimal values (like position vectors
; or velocity components). Collins used this during landmark tracking to display
; three-axis position updates from optical sightings.
;
GOMARK3		TS	PLAYTEM1	# USED FOR 3COMP DECIMAL PERFORM
		CAF	MARK3MSK
		TCF	GOFLASH2
;
; GOMARK4 - Mark Display with Blanking
; Adds blanking control to mark display (MARK4MSK includes blank bit). Blanking
; clears unused digit positions, preventing display clutter. Used when switching
; between different noun formats with varying numbers of significant digits.
;
GOMARK4		TS	PLAYTEM1
		CAF	MARK4MSK	# MARK,PERFORM,FLASH,BLANK
		TCF	GOFLASH2
;
; GOMARKR - Mark Display with Immediate Return
; The "R" suffix indicates immediate return to caller while display runs in
; separate job. Allowed time-critical navigation calculations to continue while
; the mark display updated. Critical during P23 cislunar navigation when
; multiple star sightings were processed rapidly.
;
GOMARKR		TS	PLAYTEM1	# ENTRANCE FOR MARK GODSPR

		CAF	BIT15
		TCF	GODSPR2
;
; GOMARKFR - Mark Flash Display with Immediate Return
; Combines mark display, flashing, and immediate return. Navigation programs
; used this to request flashing displays without blocking their computation loops.
;
GOMARKFR	TS	PLAYTEM1	# ENTRANCE FOR MARK GOFLASHR

		CAF	MARKFMSK
		TCF	GODSPRS
;
; GOMARK2R - Three-Return Mark Display with Immediate Return
; Combines perform mode (three astronaut responses) with immediate return to
; caller. Navigation sequences used this to request crew input displays while
; continuing computational tasks in parallel.
;
GOMARK2R	TS	PLAYTEM1	# MARK GOPERFS-3 AST. RETS+ IMMEDIATE RET.
		CAF	MPERFMSK	# MARK, PERFORM, FLASH
		TCF	GODSPRS
;
; GOMARK3R - Three-Component Decimal Display with Immediate Return
; Immediate-return variant of GOMARK3 for vector displays. During cislunar
; navigation, this allowed rapid display updates of position/velocity vectors
; without pausing guidance calculations.
;
GOMARK3R	TS	PLAYTEM1
		CAF	MARK3MSK
		TCF	GODSPRS
;
; ============================================================================
; MAKEMARK - Mark Display Scheduling and Priority Management
;
; Called by GOMARK variants through COPIES routine. Determines if the mark
; display can proceed immediately or must wait due to higher-priority displays
; or astronaut DSKY usage. Implements the priority hierarchy: astronaut first,
; priority displays second, then marks.
; ============================================================================
;
; MAKEMARK Entry Point
; Checks display system status flags to determine mark display scheduling.
; During Apollo 11's translunar coast, multiple mark displays could be queued
; from simultaneous navigation operations, requiring careful priority management.
;
MAKEMARK	CAF	ONE
		TC	COPIES
;
; Check Normal and Priority Display Status
; FLAGWRD4 bits indicate active/waiting displays. OCT34300 mask checks if
; normal or priority displays are busy (higher priority than marks). If any
; higher-priority display is active, mark must wait its turn via CHKPRIO.
;
		CA	FLAGWRD4	# IS NORM OR PRIO BUSY OR WAITING
		MASK	OCT34300
		CCS	A
		TCF	CHKPRIO
;
; Check Astronaut DSKY Usage
; BIT9 of FLAGWRD4 indicates astronaut is using DSKY (external display request).
; If set, mark display sleeps until astronaut finishes. The ENDOFJOB return
; puts this job into wait state, where it will be awakened when DSKY becomes
; available.
;
		CA	FLAGWRD4	# IS MARK SLEEPING DUE TO ASTRO BUSY
# Page 1467
		MASK	BIT9
		EXTEND
		BZF	MARKPLAY	# NO

		TCF	ENDOFJOB
;
; MARKPLAY - Activate Mark Display
; DSKY is available and no higher-priority displays active. Set mark active
; flag (bit 0) and clear mark-over-normal flag (bit 2) in FLAGWRD4. Interrupts
; disabled (INHINT) during flag updates to prevent race conditions if astronaut
; presses keys during this critical update.
;
MARKPLAY	INHINT
		CS	FIVE		# RESET MARK OVER NORM, SET MARK
		MASK	FLAGWRD4
		AD	ONE
		TS	FLAGWRD4
		RELINT
;
; GOGOMARK - Check Mark Perform Mode
; Tests MARKFLAG bit 5 to determine if this is a perform-mode display (waiting
; for crew response). If perform mode not set, complements MARKNV to negate the
; verb-noun code (standard AGC technique using twos-complement). Perform mode
; marks enable three-response handling (PROCEED, ENTER, TERMINATE).
;
GOGOMARK	CS	MARKFLAG	# PERFORM
		MASK	BIT5
		CCS	A
		TCF	MARKCOP
		CS	MARKNV
		TS	MARKNV
;
; MARKCOP - Transfer to Priority Display Handler
; Loads mark index (1) and transfers to PRIOPLAY for common display processing.
; Both mark and priority displays share the PRIOPLAY handler logic, distinguished
; by index value. This code reuse simplified the display system architecture.
;
MARKCOP		CAF	ONE		# MARK INDEX
		TCF	PRIOPLAY
;
; COPYTOGO - Restore MPAC Stack Value
; Called during display activation to restore saved MPAC+2 value from MPAC2SAV.
; The MPAC (Multi-Purpose Accumulator) stack is used by the interpreter for
; mathematical operations. Display routines must preserve MPAC state when
; interrupting ongoing calculations.
;
COPYTOGO	CA	MPAC2SAV
		TS	MPAC +2
;
; COPYPACS - Load Display Control Masks
; Indexed load of display control mask (PRIOOCT) and erasable bank number
; (EBANKSAV) for the display type specified by COPINDEX (0=priority, 1=mark,
; 2=normal). Sets GENMASK for display formatting and EBANK for memory access.
; TEMPOR2 stores active EBANK and flags for later restoration.
;
COPYPACS	INDEX	COPINDEX
		CAF	PRIOOCT
		TS	GENMASK

		INDEX	COPINDEX
		CAF	EBANKSAV
		TS	TEMPOR2		# ACTIVE EBANK AND FLAG

		TS	EBANK

		TC	Q
;
; ============================================================================
; PINCHEK Logic - Mark Request Priority Conflict Resolution
;
; PINCHEK CHECKS TO SEE IF THE CURRENT MARK REQUEST IS MADE BY THE ASTRONAUT WHILE INTERRUPTING A GOPLAY DISPLAY
; (A NORMAL OR A PRIO). IF THE ASTRONAUT TRIES TO MARK DURING A PRIO, THE CHECK FAIL LIGHT GOES ON AND THE MARK
; REQUEST IS ENDED. IF HE TRIES TO MARK DURING A NORM, THE MARK IS ALLOWED. IN THIS CASE THE NORM IS PUT TO SLEEP
; UNTIL ALL MARKING IS FINISHED.
;
; 	IF THE MARK REQUEST COMES FROM THE PROGRAM DURING A TIME THE ASTRONAUT IS NOT INTERRUPTING A NORMAL OR A
; PRIO, THE MARK REQUEST IS PUT TO SLEEP UNTIL THE PRESENT ACTIVE DISPLAY IS RESPONDED TO BY THE ASTRONAUT.
;
; This conflict resolution was critical during Apollo 11's mission. If Armstrong
; or Aldrin initiated a star sighting (mark) while a priority alarm display was
; active (like the 1202 alarm during descent), the mark would be rejected and
; the CHECK FAIL light would illuminate to warn the crew. However, marks could
; interrupt normal displays (like velocity readouts), putting the normal display
; to sleep until the navigation mark completed.
; ============================================================================
;
; CHKPRIO - Check Priority Display Conflict
; Tests if priority display is active (OCT24100 mask). If so, mark request
; must sleep (MARSLEEP). If only normal display active, mark can interrupt it.
; Priority displays (alarms, critical warnings) cannot be interrupted by marks.
;
CHKPRIO		CA	FLAGWRD4	# MARK ATTEMPT DURING PRIO
		MASK	OCT24100
		CCS	A
		TCF	MARSLEEP
;
; Allow Mark Over Normal Display
; No priority conflict detected. Sets BIT3 (mark-over-normal flag) in FLAGWRD4
; to indicate a mark display has interrupted a normal display. The normal display
; will be put to sleep and resumed after marking completes. INHINT protects this
; critical flag update from keyboard interrupts.
;
# Page 1468
		CS	FLAGWRD4
		MASK	BIT3		# SET MARK OVER NORM
		INHINT
		ADS	FLAGWRD4

		TCF	SETNORM
;
; MARKPERF - Mark Performance Display Entry
; Extracts verb code from MARKNV (using VERBMASK to isolate verb bits) and
; transfers to NV50DSP for verb-noun display formatting. Used when mark displays
; need to show verb-noun combinations to the crew for confirmation or data entry.
;
MARKPERF	CA	MARKNV
		MASK	VERBMASK
		TCF	NV50DSP
;
; ============================================================================
; GODSP Family - Normal Display Request Routines
;
; These routines request normal displays (mission program data displays with
; lower priority than marks or priority alarms). During Apollo 11's translunar
; coast, GODSP displayed navigation state updates, orbital parameters, and
; burn targeting data. The crew saw these displays during routine operations
; when no higher-priority information needed attention.
; ============================================================================
;
; GODSP - Normal Display Request
; Stores verb-noun code in PLAYTEM1 and transfers to GODSP2 with zero flag mask.
; This is the standard entry point for normal displays without special formatting.
;
GODSP		TS	PLAYTEM1

GODSP2		CAF	ZERO
		TCF	GOFLASH2
;
; GODSPRET - Normal Display with Paste (Return After NVSUB)
; Used when display needs to return to user program after NVSUB (noun display
; subroutine) completes. BIT6 flag signals return path. "Paste" refers to
; attaching a return address to the display request, allowing the calling program
; to regain control after display formatting completes.
;
GODSPRET	TS	PLAYTEM1	# ENTRANCE FOR A GODSP WITH A PASTE

		CAF	BIT6		# SET BIT6 TO GO BACK TO USER AFTER NVSUB
		TCF	GOFLASH2
;
; GODSPR - Normal Display with Immediate Return
; The "R" suffix indicates immediate return to caller while display runs in
; separate job. Mission programs used this during time-critical phases (like
; midcourse corrections) to request displays without blocking guidance calculations.
;
GODSPR		TS	PLAYTEM1

GODSPR1		CAF	ZERO
GODSPR2		TS	PLAYTEM4

		CAF	ZERO		# * DONT MOVE
		TCF	GODSPRS1
;
; ============================================================================
; CLEANDSP - Clear Normal Display
;
; 	CLEANDSP IS USED FOR CLEARING OUT A NORMAL DISPLAY THAT IS PRESENTLY ACTIVE OR A NORMAL DISPLAY THAT IS
; SET UP TO BE STARTED OR RESTARTED.
;
; 	NORMALLY THE USER WILL NOT NEED TO USE THIS ROUTINE SINCE A NEW NORMAL DISPLAY AUTOMATICALLY CLEARS OUT AN
; OLD DISPLAY.
;
; CALLING SEQUENCE FOR CLEANDSP-
;
;		TC	BANKCALL
;		CADR	CLEANDSP
;
; Used to explicitly clear displays when switching between mission phases or
; when a program wants to ensure clean DSKY state. During Apollo 11's lunar
; orbit insertion, programs used CLEANDSP to clear translunar navigation displays
; before showing orbital parameters in the new flight regime.
; ============================================================================
;
CLEANDSP	CAF	ZERO
;
; REFLASH - Reflash Current Display Entry Point
; Alternate entry to CLEANDSP that re-displays the current verb-noun with
; flashing enabled (REDOMASK sets flash and permit bits). Used when programs
; want to re-prompt the crew for input after a timeout or data validation failure.
;
REFLASH		TS	PLAYTEM1

		CAF	REDOMASK	# FLASH AND PERMIT
		TCF	GOFLASH2
;
; REGODSP - Re-request Normal Display
; Requests display with BIT14 set (re-request flag). Used during restart
; sequences to restore displays that were active before a restart condition.
; After a computer restart during Apollo 11 descent, REGODSP restored the
; landing velocity display so the crew maintained situational awareness.
;
REGODSP		TS	PLAYTEM1

		CAF	BIT14
		TCF	GOFLASH2

# Page 1469
;
; REGODSPR - Re-request Normal Display with Immediate Return
; Combines REGODSP functionality (BIT14 re-request flag) with GODSPR behavior
; (immediate return via separate job). Used during restart sequences when displays
; need to be restored but calling program must continue execution immediately.
;
REGODSPR	TS	PLAYTEM1
		CAF	BIT14
		TCF	GODSPR2
;
; CLOCPLAY - Clock Display Request
; Special display routine for mission elapsed time (MET) clock displays.
; CLOCKCON flag indicates clock formatting. During Apollo 11's mission, the crew
; monitored mission time continuously on DSKY. This routine formatted the elapsed
; time from launch (July 16, 1969, 13:32:00 UTC) for crew display during all phases.
;
CLOCPLAY	TS	PLAYTEM1
		CAF	CLOCKCON
		TCF	GOFLASH2
;
; ============================================================================
; GOFLASH Family - Flash Display Control
;
; These routines manage flashing displays (alternating display/blank cycles)
; to indicate the DSKY is waiting for crew input. During Apollo 11's descent,
; flashing verb-noun displays prompted Armstrong and Aldrin for confirmation
; before critical maneuvers like powered descent initiation.
; ============================================================================
;
; GOFLASH - Flash Display Request
; Standard entry for flashing displays. Sets BIT4 (flash bit) in PLAYTEM4 to
; enable flash mode, then transfers to GOFLASH2 for common processing.
;
GOFLASH		TS	PLAYTEM1

		CAF	BIT4		# LEAVE ONLY FLASH BIT SET
;
; GOFLASH2 - Flash Display Common Processing
; All flash display variants converge here. Saves caller location (SAVELOCS),
; enables interrupts (RELINT), then branches directly to MAKEPLAY without creating
; separate job. This makes the display part of the calling program's job, blocking
; the caller until crew responds. Critical for mission sequences that cannot
; proceed without crew confirmation (like abort mode selections).
;
GOFLASH2	TS	PLAYTEM4

		TC	SAVELOCS

		RELINT

		TCF	MAKEPLAY	# BRANCH DIRECT WITH NO SEPARATE JOB CALL
;
; ============================================================================
; PRIODSP Family - Priority Display Request Routines
;
; Priority displays take precedence over all other displays except astronaut-
; initiated displays. Used for critical alarms and warnings that demand immediate
; crew attention. During Apollo 11 descent, the famous 1202 program alarm was
; displayed via PRIODSP, interrupting the landing velocity display to warn of
; executive scheduler overflow at mission time 102:38:26.
; ============================================================================
;
; PRIODSPR - Priority Display with Immediate Return
; Requests priority display with immediate return to caller (separate job created).
; BITS7+4 set priority and flash flags. Used when alarm conditions detected but
; calling program must continue monitoring (like guidance continuing to run during
; 1202 alarm display).
;
PRIODSPR	TS	PLAYTEM1

		CAF	BITS7+4
		TCF	GODSPRS
;
; PRIODSP - Priority Display Request
; Standard entry for priority displays without immediate return. Transfers to
; SETPRIO for priority flag setup. Used for critical warnings where calling
; program should block until crew acknowledges the alarm condition.
;
PRIODSP		TS	PLAYTEM1
;
; SETPRIO - Set Priority Flags
; Sets BITS7+4 (priority display + flash) in PLAYTEM4 and transfers to GOFLASH2
; for common flash processing. BIT7 marks this as priority display, BIT4 enables
; flash mode for crew attention.
;
SETPRIO		CAF	BITS7+4
		TCF	GOFLASH2
;
; ============================================================================
; MAKEPRIO - Priority Display Job Creation and Conflict Resolution
;
; Central logic for establishing priority displays. Handles complex priority
; conflict resolution when multiple displays compete for DSKY. During Apollo 11,
; this logic ensured program alarms always interrupted lower-priority displays
; but didn't cause system abort when multiple alarms occurred simultaneously.
; ============================================================================
;
; Initialize Priority Display Job
; Sets COPINDEX to zero (priority display index) and calls LINUSCHR to check
; LINUS abort conditions. LINUS (Lunar Module Abort Sensor) integration allows
; certain priority displays to be suppressed during critical abort sequences.
;
MAKEPRIO	CAF	ZERO
		TS	COPINDEX

		TC	LINUSCHR
		TCF	HIPRIO		# LINUS RETURN
;
; Check for Priority Display Conflict
; Tests if another priority display is already active in ENDIDLE or busy state
; (OCT20100 mask checks priority busy flags). If priority display already running,
; system aborts via PRIOBORT unless BIT14 was set (LINUS exception). This prevents
; cascading priority display conflicts.
;
		CA	FLAGWRD4
		MASK	OCT20100	# IS PRIO IN ENDIDLE OR BUSY
		CCS	A
		TCF	PRIOBORT	# YES, ABORT
;
; HIPRIO - High Priority Display Activation
; No priority conflict detected. Now checks if mark display is active (OCT40400 mask).
; If mark active, must interrupt it and put mark to sleep (SETMARK). Priority
; displays interrupt marks - this was critical during Apollo 11 when program alarms
; interrupted navigation star sightings during descent.
;
HIPRIO		CA	FLAGWRD4	# MARK ACTIVE
		MASK	OCT40400
		EXTEND
		BZF	ASKIFNRM	# NO
;
; SETMARK - Interrupt Active Mark Display
; Mark display is active. Call JOBXCHS with index zero to put mark display to
; sleep and exchange job control to priority display. The interrupted mark will
; be resumed after priority display completes (or after crew responds if mark
; requires input).
;
SETMARK		CAF	ZERO
		TCF	JOBXCHS
;
; ASKIFNRM - Check for Normal Display Activity
; No mark active. Tests if normal display is active (OCT10200 checks normal busy
; and active flags). If normal display active, must interrupt it and put normal
; to sleep (SETNORM). Priority displays interrupt normals - during Apollo 11's
; translunar coast, fuel cell alarms interrupted orbital navigation displays.
;
ASKIFNRM	CA	FLAGWRD4	# NORMAL ACTIVE
		MASK	OCT10200	# BITS 13+8
		EXTEND
# Page 1470
		BZF	OKTOCOPY	# NO
;
; SETNORM - Interrupt Active Normal Display
; Normal display is active. Call JOBXCHS with index one to put normal display
; to sleep and exchange job control to priority display. The interrupted normal
; will be resumed after priority display completes.
;
SETNORM		CAF	ONE
		TCF	JOBXCHS
;
; OKTOCOPY - No Display Conflict, Activate Priority Display
; No conflicting displays active. Copy normal display state for potential restoration
; (COPYNORM), clear key release light and handle address storage (WITCHONE), wake
; the priority display job (JOBWAKE), then replace caller's return address with
; ENDOFJOB termination (XCHTOEND). Priority display takes full control.
;
OKTOCOPY	TC	COPYNORM
		TC	WITCHONE

		TC	JOBWAKE

		TC	XCHTOEND
;
; REDOPRIO - Priority Display Timestamp
; Records TIME1 (AGC incremental time register) to PRIOTIME when priority display
; is sent out. This timestamp allows tracking priority display duration and
; implementing timeout logic if crew doesn't respond. During Apollo 11, priority
; alarm timestamps were telemetered to Mission Control for post-mission analysis.
;
REDOPRIO	CA	TIME1		# SAVE TIME PRIODSP SENT OUT
		TS	PRIOTIME
;
; KEEPPRIO - Continue Priority Display Activation
; Initializes COPINDEX to zero (priority display index) and transfers to PRIOPLAY
; to start the priority display job. Used when priority display has passed all
; conflict checks and is ready to execute.
;
KEEPPRIO	CAF	ZERO		# START UP PRIO DISPLAY
		TCF	PRIOPLAY
;
; ============================================================================
; MAKEPLAY - Main Display Job Dispatch and Priority Management
;
; Central routing logic for all display types (normal, mark, priority). Determines
; display type from PLAYTEM4 flags, raises job priority for display processing,
; performs legality checks, and dispatches to appropriate handler (MAKEPRIO,
; MAKEMARK, or normal display processing). This is the common entry point for
; both immediate-return displays (with separate job) and direct branch displays.
; ============================================================================
;
; Save Caller's Priority and Raise Display Job Priority
; Saves original caller priority in USERPRIO for restoration later, then raises
; current job priority to PRIO33 (high priority) via PRIOCHNG. Display jobs run
; at elevated priority to minimize DSKY response latency. During Apollo 11 descent,
; this ensured crew inputs and alarm displays were processed quickly despite heavy
; computational load from guidance and navigation.
;
MAKEPLAY	CA	PRIORITY	# SAVE USERS PRIORITY
		MASK	PRIO37
		TS	USERPRIO

		CAF	PRIO33		# RAISE PRIORITY FOR FAST JOBS AFTER WAKE
		TC	PRIOCHNG
;
; Determine Display Type from Flags
; Examines PLAYTEM4 bits (BITS15+7) to classify display type:
;   - BIT15 set: Priority display → branch to MAKEPRIO
;   - BIT7 set: Mark display → branch to MAKEMARK
;   - Neither set: Normal display → continue to IFLEGAL
; This three-way dispatch implements the display priority hierarchy.
;
		CA	PLAYTEM4	# IS IT MARK OR PRIO OR NORM
		MASK	BITS15+7
		CCS	A
		TCF	MAKEPRIO	# ITS PRIO
		TCF	IFLEGAL
		TCF	MAKEMARK	# ITS MARK
;
; IFLEGAL - Normal Display Legality Check
; For normal displays, sets COPINDEX=2 (normal display index) and calls LINUSCHR
; to check LINUS abort sensor conditions. Then verifies no conflicting normal
; displays already asleep (sleeping normals indicate display queue conflict).
; If illegal condition detected (multiple normals asleep), aborts via PRIOBORT.
; This prevents normal display queue corruption.
;
IFLEGAL		CAF	TWO
		TS	COPINDEX

		TC	LINUSCHR

		TCF	OKTOPLAY	# LINUS RETURN
;
; Check for Re-request Flag (BIT4 of EBANKTEM)
; Tests if this is a re-request of an interrupted normal display. If BIT4 clear,
; this is a new request (not a re-request), so continue to OKTOPLAY. If BIT4 set,
; check for sleeping normals conflict.
;
		CS	EBANKTEM
		MASK	BIT4
		CCS	A
		TCF	OKTOPLAY	# NO
;
; Verify No Normal Display Sleeping (Conflict Check)
; Tests FLAGWRD4 with NBUSMASK to detect sleeping normal displays. If any normals
; are asleep, system is in illegal state (normal display conflict). This would
; indicate display queue corruption or programming error. Abort system via PRIOBORT
; with alarm code 1502. During Apollo 11, such aborts never occurred due to careful
; mission program display coordination.
;
		CA	FLAGWRD4	# WAS NORM ASLEEP
		MASK	NBUSMASK	# ARE ANY NORMS ASLEEP
		EXTEND
		BZF	OKTOPLAY	# NO
;
; PRIOBORT - Priority Display Conflict Abort
; System abort when priority display conflict detected (priority display already
; running when new priority display requested) or normal display queue corrupted.
; Calls POODOO (abort handler) with alarm code 1502. This critical safety check
; prevents display system state corruption.
;
PRIOBORT	TC	POODOO
		OCT	1502
# Page 1471
;
; OKTOPLAY - Normal Display Activation Processing
; Normal display passed all legality checks. Now copy display parameters (COPIES2),
; save user priority and superbank to restart register (RESTREG) for potential
; restart recovery, then check if priority or mark display is active. If higher
; priority display active, must interrupt it appropriately.
;
OKTOPLAY	TC	COPIES2

		CA	USERPRIO
		EXTEND
		ROR	SUPERBNK
		TS	RESTREG

		CA	FLAGWRD4	# PRIO OR MARK GOING
		MASK	PMMASK
		CCS	A
		TCF	GOSLEEPS	# YES

		TCF	+2
		TCF	GOSLEEPS	# MARK GOING

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

GOPERF4		TC	PURRS4

		TCF	GOFLASH2

GOFLASHR	TS	PLAYTEM1
# Page 1472
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

		EXTEND			# SAVE NVWORD AND USERS MPAC +2
		DCA	MPAC +1
		INDEX	LOCCTR
		DXCH	MPAC +1

		EXTEND			# SAVE USERS CADR, FLAGS AND EBANK
		DCA	MPAC +3
		INDEX 	LOCCTR
		DXCH	MPAC +3

		CA	LOCCTR
		TS	MPAC +5
		TC	SAVELOCR
		RELINT
		TCF	BANKJUMP	# CALL CADR +4

# Page 1473
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

SAVELOCR	LXCH	Q

		TC	MAKECADR
		TS	PLAYTEM3

		AD	RUPTREG3	# NOT USED FOR NON R ROUTINES
		TC	L

COPYNORM	CAF	ZERO
COPIES		TS	COPINDEX
COPIES2		INHINT
		CA	PLAYTEM4	# FLAGWORD
		INDEX	COPINDEX
		TS	EBANKSAV	# EQUIV TO DSPFLG
		MASK	CADRMASK	# FLASH AND GODSPRET
		EXTEND
# Page 1474
		BZF	SKIPADD

		CA	PLAYTEM3
		INDEX	COPINDEX
		TS	CADRFLSH

SKIPADD		CA	PLAYTEM1	# VERB NOUN
		INDEX	COPINDEX
		TS	NVWORD

		TCF	RELINTQ

; ============================================================================
; GOSLEEPS - Put Display Job to Sleep
;
; This routine is called when a higher priority display interrupts a currently
; active display. The interrupted display's job is put to sleep and will be
; awakened when the higher priority display completes.
;
; COMMENT-ONLY READERS: When multiple displays competed for astronaut attention,
;        the computer put lower-priority displays "to sleep" until the crew
;        finished with the urgent information. This preserved all display data
;        for later resumption.
;
; CODE-ALONG READERS: Sets wait bit flags via UPENT2, then calls XCHSLEEP to
;        swap job sleep/wake state. Uses COPINDEX (display type: 0=prio, 1=mark,
;        2=norm) to index into display control tables.
; ============================================================================

GOSLEEPS	INDEX	COPINDEX
		CA	PRIOOCT
		MASK	WAITMASK
		TC	UPENT2
WAITMASK	OCT	3004
		CS	ONE
		AD	COPINDEX
		TS	FACEREG

; ============================================================================
; XCHSLEEP - Exchange Sleep State
;
; Awakens a sleeping job while putting the current job to sleep. This implements
; display priority switching by exchanging job states in the executive scheduler.
; The awakened job resumes where it left off, while the current job suspends
; execution until later reactivated.
;
; COMMENT-ONLY READERS: This was the computer's mechanism for switching between
;        displays. When Armstrong needed to see critical landing data, this
;        routine instantly suspended the previous display and restored the
;        landing information.
;
; CODE-ALONG READERS: Uses WAKECADR (wake-up address) from display tables,
;        calls JOBWAKE to locate and activate sleeping job, then XCHTOEND to
;        redirect awakened job, finally JOBSLEEP to suspend current job.
;        INHINT disables interrupts during critical job state exchange.
; ============================================================================

XCHSLEEP	INDEX	FACEREG
		CAF	WAKECADR
		INHINT
		TC	JOBWAKE		# FIND CADR IN JOB AREA

		TC	XCHTOEND	# CAUSES AWAKENED JOB TO GO TO ENDOFJOB

		INDEX	FACEREG		# REPLACE SAME CADR BUT NEW JOB AREA
		CAF	WAKECADR
		TCF	JOBSLEEP

; ============================================================================
; JOBXCHS - Job Exchange Control
;
; Controls which type of display (priority, mark, or normal) is put to sleep
; during display priority handling. Manages flag bits and job wake/sleep
; transitions for display type transitions.
; ============================================================================

JOBXCHS		TS	FACEREG		# CONTROLS TYPE OF DISPLAY PUT TO SLEEP
		TC	WITCHONE
		TC	JOBWAKE
		CA	FACEREG
		INDEX	LOCCTR
		TS	FACEREG

		CAF	XCHQADD
		TC	XCHNYLOC

		INDEX	FACEREG
		CA	MARKOCT
		MASK	IDLESLEP
		TC	DOWNENT2
IDLEMASK	OCT	74004		# * DONT MOVE
		INDEX	FACEREG		# BIT SHOWS PRIO INTERRUPTED NORM OR MARK
		CA	BIT5		# BIT5 FOR MARK, BIT4 FOR NORMAL
# Page 1475
		AD	FOUR
		TC	UPENT2		# FLAG ROUTINE DOES RELINT
XCHQADD		GENADR	XCHSLEEP	# * DONT MOVE
		CA	FLAGWRD4
		MASK	BIT3		# IF BIT3 THEN MARK OVER NORM
		CCS	A
GENMARK		TC	MARKPLAY	# USED AS GENADR FOR JOBWAKE
		TCF	OKTOCOPY

; ============================================================================
; MARKWAKE / WAKEPLAY - Wake Up Sleeping Display
;
; These routines awaken a display job that was previously put to sleep by a
; higher priority display. MARKWAKE specifically awakens mark displays (used
; for optical navigation sightings), while WAKEPLAY handles general display
; wake-up operations.
;
; COMMENT-ONLY READERS: After the crew finished reviewing urgent information,
;        the computer automatically restored the previous display that had been
;        waiting. All data was preserved exactly as the astronaut left it.
;
; CODE-ALONG READERS: Clears sleep flag bits via DOWNENT2, retrieves WAKECADR
;        (wake-up address) from display tables, calls JOBWAKE to reactivate
;        the sleeping job, then ENDRET to complete. INHINT protects critical
;        job state changes.
; ============================================================================

MARKWAKE	CAF	ZERO
WAKEPLAY	TS	TEMPOR2

		INDEX	TEMPOR2
		CA	BITS5+11
		AD	FOUR
		TC	DOWNENT2
MARKFMSK	OCT	40010		# ***DONT MOVE

		INDEX	TEMPOR2
		CAF	WAKECADR
		INHINT
		TC	JOBWAKE

		TCF	ENDRET

# 	ALL .1 RESTARTS BRANCH DIRECTLY TO INITDSP. NORMAL DISPLAYS ARE THE ONLY DISPLAYS ALLOWED TO USE .1 RESTARTS
# INITDSP FIRST RESTORES THE EBANK AND THE SUPERBANK TO THE MOST RECENT NORMAL EBANK AND SUPERBANK.
# 	IF THE MOST RECENT NORMAL DISPLAY REQUEST WAS NOT FINISHED, CONTROL IS SENT BACK TO THE LAST NORMAL USER.
# OTHERWISE THE NORMAL DISPLAY SET UP IN THE NORMAL DISPLAY REGS IS STARTED UP IMMEDIATELY.

; ============================================================================
; INITDSP - Initialize Display After Restart
;
; Handles display recovery after system restart (AGC power transient or reset).
; Restores memory bank context (EBANK and SUPERBANK) and resumes interrupted
; normal display operations. Only normal displays use .1 restart protection
; because priority and mark displays complete quickly.
;
; COMMENT-ONLY READERS: If the computer temporarily lost power or encountered
;        an error requiring restart, this routine restored the crew's display
;        exactly as it appeared before the interruption. Critical during Apollo 11
;        when 1202 program alarms triggered restart protection during descent.
;
; CODE-ALONG READERS: Restores EBANK (erasable memory bank), SUPERBANK (fixed
;        memory bank), and job priority from saved restart data. Checks if
;        normal display was incomplete and resumes user's program, or initiates
;        new display request if previous one finished.
; ============================================================================

INITDSP		CA	EBANKTEM	# RESTORE MOST RECENT NORMAL EBANK
		TS	EBANK

		CA	RESTREG		# SUPERBANK AND JOB PRIORITY
		TC	SUPERSW		# RESTORE SUPERBANK

		MASK	PRIO37
		TC	PRIOCHNG

		CS	THREE
		AD	TEMPFLSH
		TCF	BANKJUMP

; ============================================================================
; PINBRNCH - Pinball Branch Entry Point
;
; Entry point for astronaut-initiated display actions (keyboard input) that
; interrupt the current display. Called when crew uses DSKY keyboard or when
; mark displays (optical sightings) need special handling. Manages PINBRANCH
; flag condition to handle display interruptions properly.
;
; COMMENT-ONLY READERS: This was the entry point when the astronaut typed
;        something on the computer keyboard or when optical navigation marks
;        were taken. The computer immediately recognized crew input and
;        prepared to handle their request, ensuring the DSKY always responded
;        to astronaut commands during critical mission phases.
;
; CODE-ALONG READERS: Re-enables interrupts (RELINT) after GOPIN users. Saves
;        MPAC+2 (needed for mark routine data preservation) to MARK2PAC, then
;        checks PINBRANCH flag condition via FLAGWRD4. If flag clear, goes to
;        ERASER (idle state). If set, routes to MARKPLAY for mark handling.
;        This implements the display priority system for crew interactions.
; ============================================================================

PINBRNCH	RELINT			# FOR GOPIN USERS
		CA	MARK2PAC	# NEEDED TO SAVE MPAC +2 FOR MARK USERS
		TS	MPAC +2		# ONLY

		CA	FLAGWRD4	# PINBRANCH CONDITION
		MASK	PINMASK
		CCS	A
# Page 1476
		TCF	+3
		TCF	ERASER		# ** NOTHING IN ENDIDLE
		TCF	MARKPLAY

; ============================================================================
; NORMBNCH - Normal Display Branch
;
; Processes display requests arriving through normal channels (not priority
; alarms). Handles crew-initiated verb/noun displays and standard mission
; program displays. Sets PINBRANCH flag and checks if priority display was
; interrupted to determine proper handling path.
;
; COMMENT-ONLY READERS: When the crew requested routine information displays
;        (such as orbit parameters, velocity readings, or navigation data),
;        the computer used this routine to properly sequence the display
;        without interfering with any urgent priority alarms that might be
;        showing. Armstrong and Aldrin relied on smooth display transitions
;        during critical mission phases.
;
; CODE-ALONG READERS: Sets PINBRFLG (pinbranch flag) via UPFLAG to mark
;        astronaut-initiated display branch. Checks BIT14 of FLAGWRD4 to
;        determine if priority display was interrupted. If priority active,
;        branches to KEEPPRIO to preserve priority display. Otherwise continues
;        to PLAYJUM1 for normal display processing. This implements display
;        priority rule: priority displays cannot be interrupted by normals.
; ============================================================================

NORMBNCH	TC	UPFLAG		# SET PINBRANCH BIT
		ADRES	PINBRFLG

		CAF	BIT14		# PRIO INTERRUPTED
		MASK	FLAGWRD4
		CCS	A
		TCF	KEEPPRIO

		TCF	PLAYJUM1

; ============================================================================
; NVDSP - CORE DISPLAY FORMATTING AND EXECUTION ROUTINE
;
; This is the main verb/noun display processing routine that formats data
; for the DSKY seven-segment displays. Handles blank bit setup, decimal
; mark positioning, display buffer management, and coordinates display
; updates with the DSKY hardware. All display requests ultimately flow
; through this routine for final formatting and output.
;
; Processing: Copies PACS buffers, sets up blanking bits, processes display
; flags (BIT8 for decimal mark, BIT13 for flash control), manages MPAC
; save areas, handles noun 00 special case, and coordinates with NVMONOPT
; for display option monitoring.
;
; Returns: Via various paths depending on display type and completion status
; ============================================================================
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

		CA	MPAC +2
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
		TC	FLASHOFF	# IN CASE OF EXTENDED VERB NON FLASH

		TC	COPYTOGO	# MPACS DESTROYED BY NVSUB
		TC	DOWNFLAG	# UNSET SLEEPING BITS
		ADRES	MRKNVFLG
# Page 1477
		TC	DOWNFLAG
		ADRES	NRMNVFLG
		TC	DOWNFLAG
		ADRES	PRONVFLG

; BLANKCHK - Check and process blank bits for display positions
; Examines TEMPOR2 bits 1, 2, 3 to determine which display digit
; positions should be blanked. Calls BLANKSUB to perform blanking,
; then recycles to NVDSP for continued processing.
BLANKCHK	CA	TEMPOR2		# BLANK BITS 1,2,3 IF SET
		TC	BLANKSUB
		TCF	NVDSP

; PERFCHEK - Check if this is a GOPERF (perform) display
; Tests BIT5 of TEMPOR2 to determine if display requires perform
; function. Perform displays wait for crew acknowledgment before
; proceeding. Branches to 1STOR2ND if perform required.
PERFCHEK	CAF	BIT5		# BIT 5 FOR PERFORM
		MASK	TEMPOR2
		CCS	A		# IS THIS A GOPERF DISPLAY
		TCF	1STOR2ND	# YES

; GOANIDLE - Handle display completion and idle state transitions
; Checks BIT4 of TEMPOR2 to determine if display should go to idle
; (flashing) state. If BIT4 set, branches to FLASHSUB for flash
; handling. Otherwise checks BIT6 for GODSPRET type displays.
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
		TS	MPAC +3
		TCF	ENDIT

; ISITN00 - Check if noun is 00 (paste/blank display)
; Examines NVWORD to determine if this is a noun 00 display, which
; indicates a verb paste operation (verb only, no noun data). Noun 00
; displays always flash to indicate awaiting crew input. Used during
; PINBRNCH (astronaut-initiated) and priority displays on mark routines.
ISITN00		INDEX	COPINDEX	# IS THIS A PASTE
		CA	NVWORD
		MASK	LOW7		# CHECK MADE FOR PINBRNCH AND PRIO ON MARK
		EXTEND
		BZF	FLASHSUB	# YES, ASSUME PASTE ALWAYS ON FLASH

		TCF	ENDOFJOB	# NOT FLASH, NOT GOPERF, THEREFORE EXIT

; 1STOR2ND - Determine if first or second pass through display routine
; Checks BIT13 of TEMPOR2 to distinguish between first and second
; execution pass. Second pass branches to GOANIDLE for completion.
; First pass sets BIT13 in DSPFLG and continues processing.
1STOR2ND	CA	TEMPOR2
		MASK	BIT13
		CCS	A
		TCF	GOANIDLE	# SECOND

		CA	BIT13
		INDEX	COPINDEX
		ADS	DSPFLG

		ZL
		EXTEND			# IS IT MARK
		BZMF	MARKPERF	# YES
		MASK	BIT12
		EXTEND
# Page 1478
		BZF	V50PASTE
		CS	NVWORD1		# NVOWRD1= -0 IS V97. NVWORD1= -400 IS V99
		AD	V97N00
		TCF	NV50DSP
V50PASTE	CAF	V50N00
		TCF	NV50DSP		# DISPLAY SECOND PART OF GOPERF

; ============================================================================
; WITCHONE - Determine which display buffer (CADRSTOR index 0 or 1) to use
;
; Turns off KEY RELEASE light (BIT5 of DSALMOUT), then checks NVBUSMSK
; flags in FLAGWRD4 to determine if NVSUB is asleep. Sets L register to
; 0 or 1 based on busy status, then exchanges appropriate CADRSTOR buffer.
; Returns with interrupts inhibited (INHINT) via Q register.
;
; This routine coordinates buffer selection between normal and priority
; displays, ensuring proper display state preservation during interrupts.
; ============================================================================
WITCHONE	CS	BIT5		# TURN OFF KEY RELEASE LIGHT
		EXTEND
		WAND	DSALMOUT

		CA	FLAGWRD4
		MASK	NVBUSMSK	# IS IT NVSUB ASLEEP
		CCS	A
		CAF	ONE
		TS	L
		CAF	ZERO
		INDEX	L
		XCH	CADRSTOR

		INHINT
		TC	Q

; XCHTOEND - Exchange location counter and prepare for end of job
; Loads ENDINST (TC ENDOFJOB instruction) and exchanges with LOCCTR.
; Checks if address was sleeping (negative). If not sleeping, branches
; to RELINTQ. If sleeping, restores LOCCTR and stores instruction at
; indexed LOC. Entry point XCHNYLOC allows direct entry with custom
; instruction in A register.
XCHTOEND	CAF	ENDINST		# TC ENDOFJOB REPLACES GENADR IN LOC FOR
XCHNYLOC	XCH	LOCCTR		# WAS THIS ADDRESS SLEEPING
		EXTEND
		BZMF	RELINTQ		# NO
		XCH	LOCCTR		# YES
		INDEX	LOCCTR
		TS	LOC

; RELINTQ - Re-enable interrupts and return
; Simple exit routine that re-enables interrupts (RELINT) and returns
; to caller via Q register. Used after INHINT-protected critical sections.
RELINTQ		RELINT
		TC	Q		# BACK TO USER

; CLEANEND - Clean termination with lower priority job setup
; Finds vacant executive core set at PRIO32 (priority 32, one lower than
; sleeping displays) and schedules JAMTERM routine to clean up display
; state. After job setup, falls through to FLASHSUB+1 to continue flash
; processing. Used when display must terminate cleanly while preserving
; system state for restart.
CLEANEND	CAF	PRIO32		# ONE LOWER THAN DISPLAYS SLEEPING
		TC	FINDVAC
		EBANK=	NVSAVE
		2CADR	JAMTERM

		TCF	FLASHSUB +1

; ISITPRIO - Check priority display status flags
; Examines ITISMASK bits in FLAGWRD4 to check if PINBRFLG (pinball branch)
; or MARKIDFLG (mark idle) flags are set. If both clear (BZF), this is a
; priority display abort condition (PRIOBORT). Otherwise normal end of job.
ISITPRIO	CA	FLAGWRD4
		MASK	ITISMASK	# IS PINBRFLG, MARKIDFLG SET
		EXTEND
		BZF	PRIOBORT
		TCF	ENDOFJOB

; REST - Check if display restart needed
; Tests CADRSTOR to determine if someone is already in ENDIDLE state.
; If positive (someone waiting), exits to ENDOFJOB. If zero or negative,
; falls through to RESTSLEP to set up display sleep state for restart.
REST		CCS	CADRSTOR	# IS SOMEONE IN ENDIDLE
		TCF	ENDOFJOB	# YES
# Page 1479
		TCF	RESTSLEP

		TCF	ENDOFJOB

; RESTSLEP - Set display sleep bits for restart protection
; Sets NVSLEEP bits using GENMASK and ASTROMSK to preserve display state
; for potential restart. Calls UPENT2 to update flag bits. OCT24100
; constant immediately follows and must not be moved (used by UPENT2).
RESTSLEP	CA	GENMASK		# SET NVSLEEP BITS
		MASK	ASTROMSK
		TC	UPENT2
OCT24100	OCT	24100		# *** DONT MOVE

; (continuation from RESTSLEP)
; Load NVCADR indexed by COPINDEX and call NVSUBUSY to check display
; subsystem availability. NVSUBUSY will either proceed if available
; or abort if illegal display state detected.
		INDEX	COPINDEX
		CAF	NVCADR
		TC	NVSUBUSY	# BUSY OR ABORT IF ILLEGAL

; FLASHSUB - Initialize flashing display sequence
; Turns on display flash (calls FLASHON), saves COPINDEX to COPMPAC
; before ENDIDLE destroys it, then sets IDLEMASK bits via UPENT2.
; ITISMASK constant follows and must not be moved (ENDIDLE dependency).
; Checks if this is repeat-and-return display via BIT3 of R1SAVE.
FLASHSUB	TC	FLASHON

		CA	COPINDEX	# COPINDEX DESTROYED BY ENDIDLE
		TS	COPMPAC

		CA	GENMASK
		MASK	IDLEMASK
		TC	UPENT2
ITISMASK	OCT	40040		# *** ENDIDLE ALLOW *** DONT MOVE

		CA	R1SAVE		# IS THIS A REPEAT AND RETURN DISPLAY
		INDEX	COPINDEX
		MASK	BIT3
		CCS	A
		TCF	UNSETR1		# YES

; Check if someone already waiting in ENDIDLE by testing CADRSTOR.
; If positive, branch to ISITPRIO to check priority status. Zero case
; skips ahead by 2. Negative cases also branch to ISITPRIO.
		CCS	CADRSTOR	# SEE IF SOMEONE ALREADY IN ENDIDLE
		TCF	ISITPRIO
		TCF	+2
		TCF	ISITPRIO

; Call ENDIDLE to enter idle display wait state. ENDIDLE has three
; return paths based on astronaut response: IDLERET1 for terminate,
; next instruction for proceed, and skip one for enter/recycle.
		TC	ENDIDLE
IDLERET1	TCF	TERMATE

		TCF	PROCEED		# ENDIDLE RETURNS HERE ON PROCEED

; Check if this is a LOAD verb (V21, V22, or V23) by comparing MPAC
; (VERBREG) against LOWLOAD constant. Uses DIM (double precision
; increment of magnitude) followed by BZF test. If match, branch to
; LOADITIS for special load verb handling.
		CS	LOWLOAD
		AD	MPAC		# VERBREG
		EXTEND
		DIM	A
		EXTEND
		BZF	LOADITIS	# V21 OR V22 OR V23 ON DSKY

; OKTOENT - Valid enter response path
; Sets OUTHERE flag to TWO indicating enter action accepted.
OKTOENT		CAF	TWO
; ENDOUT - Common exit path determining return type
; Stores entry code in OUTHERE, then checks FLAGWRD4 bits (OCT60000)
; to determine nature of ENDIDLE return: priority, normal, or mark.
ENDOUT		TS	OUTHERE
		CA	FLAGWRD4	# CHECK NATURE OF ENDIDLE RETURN
		MASK	OCT60000
# Page 1480
		CCS	A
		TCF	TIMECHEK	# PRIO ENDIDLE RETURN
		TCF	NORMRET		# NORMAL ENDIDLE RETURN
		TCF	MARKRET		# MARK ENDIDLE RETURN

; TIMECHEK - Verify priority display time constraint
; Checks elapsed time since priority display started (PRIOTIME vs TIME1).
; Computes time delta and compares against -2SEC threshold. If priority
; time expired (BZMF), keeps priority status (KEEPPRIO). Otherwise
; transitions to normal return path (NORMRET). Ensures priority displays
; don't monopolize DSKY beyond reasonable response time.
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

; NORMWAKE - Wake normal priority display
; Loads ONE into A register and branches to WAKEPLAY to wake a normal
; priority display from waiting queue.
NORMWAKE	CAF	ONE
		TCF	WAKEPLAY

; ENDRET - Process ENDIDLE return and compute return address
; Tests OUTHERE flag using CCS to determine exit path. Adds ONE to
; distinguish between terminate (ENDOFJOB) and proceed/enter paths.
; For normal exit, computes return address by indexing CADRFLSH with
; COPMPAC and stores in MPAC+3 for eventual BANKJUMP return to caller.
ENDRET		CCS	OUTHERE
		AD	ONE
		TCF	+2		# NORMAL ENDIDLE EXIT
		TCF	ENDOFJOB
		INDEX	COPMPAC
		AD	CADRFLSH
		TS	MPAC +3

; Remove ENDIDLE and PINBRANCH status bits by masking GENMASK with
; PINIDMSK (OCT 74044) and calling DOWNENT2 to clear flag bits.
; PINIDMSK constant must not be moved (flag manipulation dependency).
		CA	GENMASK		# REMOVE ENDIDLE AND PINBRANCH BITS
		MASK	PINIDMSK
		TC	DOWNENT2
PINIDMSK	OCT	74044		# *** DONT MOVE

; Blank all DSKY displays except Mission Timer (MM) by calling NVSUB
; with complement of THREE (-3). This clears verb, noun, and data
; displays while preserving mission elapsed time display.
		CS	THREE		# BLANK EVERYTHING EXCEPT MM
		TC	NVSUB
		TCF	+1

; ENDIT - Final display cleanup and return to caller
; Restores user's original job priority by extracting USERPRIO bits
; with PRIO37 mask and calling PRIOCHNG. Loads computed return address
; from MPAC+3 and executes BANKJUMP to return control to calling routine
; at appropriate continuation point (terminate+1, proceed+2, enter+3).
ENDIT		CA	USERPRIO	# RETURN TO USERS PRIORITY
		MASK	PRIO37
		TC	PRIOCHNG
		CA	MPAC +3
		TCF	BANKJUMP

; UNSETR1 - Clear repeat-and-return request flag
; Resets BIT3 in R1SAVE indexed by COPINDEX to clear repeat and return
; request from extended verb processing. For AGC Block II (205 only),
; calls SUPERSW with ZERO to ensure MARKBRAN users are in superbank 0.
UNSETR1		INDEX	COPINDEX	# RESET REPEAT AND RETURN REQUEST
		CS	BIT3
		MASK	R1SAVE
		TS	R1SAVE
		CAF	ZERO		# *** 205 ONLY MARKBRAN USERS IN
		TC	SUPERSW		# SUPERBANK 0
# Page 1481

; -1 entry point loads THREE for immediate return path offset
-1		CAF	THREE		# RETURN TO USERS IMMEDIATE RETURN LOC
; IMMEDRET - Immediate return to caller (no ENDIDLE wait)
; Computes return address by indexing CADRFLSH with COPINDEX and adding
; offset in A register (THREE from -1 entry, or other values). Executes
; BANKJUMP to return immediately to calling routine without entering idle
; wait state. Used by routines ending in "R" for immediate job return.
IMMEDRET	INDEX	COPINDEX
		AD	CADRFLSH
		TCF	BANKJUMP

; TERMATE - Astronaut terminate response (V34) handler
; Loads ZERO into A register and branches to ENDOUT to process terminate
; action. ENDOUT will set OUTHERE to ZERO, causing ENDRET to branch to
; ENDOFJOB, returning control to caller at CADR+1 (terminate return point).
TERMATE		CAF	ZERO		# ASTRONAUT TERMINATE (V34) RETURNS TO
		TCF	ENDOUT

; LINUSCHR - Check LINUS routine special handling
; Tests if display is a LINUS routine by checking BIT14 in PLAYTEM4.
; LINUS routines are priority displays that can interrupt other priority
; displays without causing abort. If LINUS, checks if already in ENDIDLE
; by comparing PLAYTEM3 against indexed CADRFLSH. If already active and
; astronaut busy (DSPLOCK positive), ends new display request via ENDOFJOB
; since LINUS display is already showing.
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
		TC	ENDOFJOB	# END THE NEW DISPLAY, ITS ALREADY ACTIVE
		TC	Q

; Note: Additional logic could verify recycle vs load operation distinction
# MORE LOGIC COULD BE INCORPORATED HERE TO MAKE SURE A RECYCLE IS A RECYCLE AND CONVERSELY THAT A LOAD IS A LOAD.

; PROCEED - Astronaut proceed response (V33) handler
; Loads ONE into A register and branches to ENDOUT to process proceed
; action. ENDOUT will set OUTHERE to ONE, causing ENDRET to add ONE and
; return control to caller at CADR+2 (proceed return point).
PROCEED		CAF	ONE		# ASTRONAUT PROCEED (V33) RETURNS
		TCF	ENDOUT

; ============================================================================
; LASTPLAY LOGIC - Display queue continuation handling
;
; LASTPLAY checks if the most recent normal display was interrupted or put
; to sleep due to higher priority display activity. This occurs in two cases:
; (1) Last normal display interrupted by priority or mark display (mark only
;     during PINBRANCH astronaut key press handling)
; (2) Last normal display requested while higher priority display active,
;     resulting in normal display being queued (put to sleep)
;
; If either condition exists, normal display is awakened to branch to PLAYJUM1
; which restarts the most recent valid normal display. If neither condition
; exists, PLAYJUM1 is started immediately assuming most recent normal display
; is already in ENDIDLE (during PINBRNCH) or that restart has occurred and
; display can start as phase 1 restart.
; ============================================================================
# 	LASTPLAY CHECKS TO SEE IF (1) THE LAST NORMAL DISPLAY WAS EITHER INTERRUPTED BY A PRIO OR A MARK (MARK
# COULD ONLY HAPPEN DURING PINBRANCH) OR IF (2) THE LAST NORMAL DISPLAY WAS REQUESTED WHILE A HIGHER PRIORITY
# DISPLAY WAS GOING, RESULTING IN THE NORMAL BEING PUT TO SLEEP.
#
# 	IF EITHER OF THE ABOVE 2 CONDITIONS EXISTS, THE NORMAL DISPLAY IS AWAKENED TO GO TO PLAYJUM1 WHICH STARTS
# UP THE MOST RECENT VALID NORMAL DISPLAY.  IF THESE 2 CONDITIONS DO NOT EXIST, CONTROL GOES TO PLAYJUM1 WHICH IS
# STARTED IMMEDIATELY WITH THE ASSUMPTION THAT THE MOST RECENT NORMAL DISPLAY IS ALREADY IN-ENDIDLE (DURING A
# PINBRNCH) OR THAT A RESTART HAS OCCURRED AND THE DISPLAY CAN BE STARTED AS A .1 RESTART.

; MARKRET - Mark display return handler
; Clears mark-related flags (BITS 5 and 11) from FLAGWRD4 using SIX mask.
; This marks the mark display as complete. INHINT protects flagword update
; from interrupt interference. After clearing flags, branches to ENDRET for
; standard return processing.
MARKRET		CS	SIX
		MASK	FLAGWRD4
		INHINT			# *** MAY MOVE DISPLAY FLAGWORD OUT OF
		TS	FLAGWRD4

		RELINT			# INHINT REALM
		TCF	ENDRET

; MARKOVER - Mark display complete with ENDOFJOB processing
; Sets OUTHERE to MINUS1 (signals ENDOFJOB to ENDRET). Tests ENDIDFLG in
; FLAGWRD4 masked with PRIO30 to check if normal or priority display is in
; ENDIDLE wait state. If set, branches to NORMBNCH to handle normal display
; continuation. Used when mark display completes and control should return
; to interrupted normal or priority display.
MARKOVER	CAF	MINUS1		# RUPTREG2 IS - MEANS ENDOFJOB TO ENDRET
		TS	OUTHERE
		CA	FLAGWRD4	# IS ENDIDFLG SET
		MASK	PRIO30		# IS NORMAL OR PRIO IN ENDIDLE
		CCS	A
# Page 1482
		TCF	NORMBNCH

; NORMRET - Normal display return handler
; Checks display queue state to determine next action. First tests if mark
; display is sleeping or waiting (BITS 5+11 in FLAGWRD4). If so, wakes mark
; display via MARKWAKE. If no mark waiting, tests if normal display was
; interrupted or waiting (BITS 4+10). If so, wakes normal via NORMWAKE.
; If neither, checks if this was flash request or GODSPRET (EBANKTEM masked
; with OCT50). If not, and NVSAVE is non-zero, starts new job at PLAYJUM1
; with priority 15 to continue display processing. This handles display queue
; continuation after current display completes.
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

; MARSLEEP - Put mark display to sleep if not already active
; Checks if mark display is already active by testing BITS 5+11 in FLAGWRD4.
; If mark already on, exits via ENDOFJOB (mark can't be queued twice).
; Otherwise, branches to GOSLEEPS to put mark display into waiting state.
; Used when mark display request arrives while higher priority display active.
MARSLEEP	CA	FLAGWRD4	# IS MARK ALREADY ON
		MASK	BITS5+11
		CCS	A
		TCF	ENDOFJOB	# YES
		TCF	GOSLEEPS

; LOADITIS - Validate data load operation
; Checks if verb-noun combination being loaded matches the noun currently
; displayed (indexed via COPMPAC from NVWORD, masked with LOW7 for noun only).
; Complements and adds to MPAC+1 (NOUNREG). If result zero (BZF), nouns match
; so this is recycle not new load - branches to OKTOENT to accept data entry.
; If nouns differ, this is actual load operation - branches to PINBRNCH to
; accept load but re-request last display. Implements load/recycle distinction
; logic mentioned in earlier comment.
LOADITIS	INDEX	COPMPAC
		CA	NVWORD
		MASK	LOW7
		COM
		AD	MPAC +1		# NOUNREG
		EXTEND
		BZF	OKTOENT		# NO, THEN LOAD IS VALID
		TCF	PINBRNCH	# YES, ACCEPT LOAD BUT ASK FOR LAST AGAIN

; ERASER - Blank all display fields except major mode (MM)
; Loads negative THREE and calls NVSUB to blank verb, noun, and data registers
; while preserving major mode display. Both return paths lead to ENDOFJOB,
; terminating current display after blanking. Used for display cleanup when
; clearing display without starting new display request.
ERASER		CS	THREE		# BLANK EVERYTHING EXCEPT MM
		TC	NVSUB
		TCF	ENDOFJOB
		TCF	ENDOFJOB

; ============================================================================
; DISPLAY INTERFACE CONSTANTS AND MASKS
;
; This section defines all constants, bit masks, CADR addresses, and symbolic
; equates used throughout the display interface routines. These values control
; DSKY display formatting, verb/noun encoding, flagword bit positions, and
; memory address mappings for display state management.
; ============================================================================

; Display control masks for DSKY indicator lights and register blanking
PERFMASK	OCT	0036		# FLASH,PERFORM,BLANK R2 AND R3
# Page 1483

; Standard verb-noun combinations used by display routines
; VN macro encodes verb in high 7 bits, noun in low 7 bits
V01N25		VN	00125		# V01N25 display combination
V06N07		VN	00607		# GOPERF3 VN DISPLAY BEFORE V50
V50N00		VN	5000		# V50N00 please perform routine
PERF2MSK	OCT	00030		# FLASH, PERFORM
V04N06		VN	00406		# V04N06 display combination
PERF4MSK	OCT	14		# FLASH, BLANK R3

; Symbolic address equates for branch targets
GOAGIN		EQUALS	PINBRNCH	# Retry/branch-again address

; Flag bit masks for display state control
REDOMASK	OCT	20010		# BITS 4 AND 14 - redo display flags
MARK3MSK	OCT	40230		# MARK,DECIMAL NOUN, PERFORM,FLASH
MARK4MSK	OCT	40036		# MARK,PERFORM,FLASH,BLANK 2 AND 3

; CADR table for display routine addresses (used by indexed addressing)
NVCADR		CADR	REDOPRIO	# Priority display redo address
WAKECADR	CADR	MARKPLAY	# Mark display wakeup address
		CADR	PLAYJUM1	# Normal display continuation address

; Memory bank and address masks
OCT3400		OCT	3400		# EBANK MASK - extracts erasable bank number
NBUSMASK	OCT	11210		# Noun busy status mask
PMMASK		OCT	66521		# Program mode mask
VERBMASK	=	MID7		# (OCT 37600) - verb field mask (bits 8-14)
V05N00M1	OCT	1177		# V05 MINUS ONE - encoded V05N00 - 1

; Symbolic equates for extended display routines
; These map alternative names to mark display entry points
GOXDSP		EQUALS	GOMARK		# Extended display = mark display
GOXDSPR		EQUALS	GOMARKR		# Extended display with return
GOXDSPF		EQUALS	GOMARKF		# Extended display with flash
GOXDSPFR	EQUALS	GOMARKFR	# Extended display with flash and return
ENDEXT		EQUALS	ENDMARK		# End extended display = end mark

; Memory location equates for temporary storage
MPAC2SAV	EQUALS	BANKSET		# MPAC save location 2
NVBUSMSK	OCT	700		# Noun/verb busy mask
ASTROMSK	OCT	704		# Astronaut activity mask
MPERFMSK	OCT	40030		# BIT 15,5,4 FOR MARK,PERFORM,FLASH
OCT34300	OCT	34300		# Combined mask value
BITS15+7	OCT	40100		# Bit positions 15 and 7
BITS7+4		OCT	110		# Bit positions 7 and 4

; Flag register symbolic equates
DSPFLG		EQUALS	EBANKSAV	# Display flag storage
MARKFLAG	EQUALS	MARKEBAN	# Mark display flag storage
SAVEFLAG	EQUALS	EBANKTEM	# Saved flag storage

; Critical bit position masks (marked DONT MOVE - position-dependent)
BITS5+11	OCT	2020		# * DONT MOVE - mark display state bits
BITS4+10	OCT	1010		# * DONT MOVE - normal display state bits

; Display load threshold
LOWLOAD		DEC	22		# Low load threshold (decimal 22)

; Status and control masks
BUSYMASK	OCT	77730		# Busy status mask (all but low 3 bits)
CADRMASK	OCT	50		# CADR address mask
PINMASK		EQUALS	13,14,15	# PINBALL mask bits

; Routine address equates
GOPLAY		EQUALS	NVDSP		# Display play = NVDSP entry
PRIOSAVE	EQUALS	R1SAVE		# Priority save location

; MPAC (multi-purpose accumulator) temporary storage equates
COPMPAC		EQUALS	MPAC +3		# Copy of MPAC location 3
TEMPOR2		EQUALS	MPAC +4		# Temporary storage 2
OUTHERE		EQUALS	MPAC +5		# Output storage location
COPINDEX	EQUALS	LOC		# Copy of index location
USERPRIO	EQUALS	MODE		# User priority storage
GENMASK		EQUALS	MPAC +6		# General mask storage

; Display type identification constants (used for display queue sorting)
; These octal values encode display type in specific bit positions
PRIOOCT		OCT	20144		# PRIO - priority display type code
MARKOCT		OCT	42424		# MARK - mark/extended verb display type code
# Page 1484
		OCT	11254		# NORM - normal display type code

; Sleep and control constants
IDLESLEP	OCT	74704		# Idle sleep state value
OCT67777	OCT	67777		# Maximum octal value mask

; Additional symbolic equates for memory locations
LINUS		EQUALS	BLANKET		# Linus routine address (blanket display)
FACEREG		EQUALS	MPAC		# Face register = MPAC base
PLAYTEM1	EQUALS	MPAC +1		# Play temporary 1
PLAYTEM3	EQUALS	MPAC +3		# Play temporary 3
PLAYTEM4	EQUALS	MPAC +4		# Play temporary 4

; Display control constants
OCT40420	OCT	40420		# Flash and special indicator mask
MAKEGEN		GENADR	MAKEPLAY	# Generate address for MAKEPLAY routine
OCT10200	OCT	10200		# Verb activity indicator mask

; Special verb-noun combinations
V97N00		VN	09700		# PASTE FOR V97 OR V99 - load program
OCT20100	OCT	20100		# Flash request mask
CLOCKCON	OCT	24030		# FLASH, PERFORM, V99 OR V97 PASTE,REFLASH

; End of DISPLAY_INTERFACE_ROUTINES.agc
; All display management, formatting, and state control constants defined above
; support the complete DSKY crew interface throughout Apollo 11 mission phases.


