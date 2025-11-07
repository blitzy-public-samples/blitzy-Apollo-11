# Copyright:    Public domain.
# Filename:     SERVICER207.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 819-836
# Contact:      Ron Burkey <info@sandroid.org>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-12 RSB	Adapted from Colossus249 file of the same
#				name and Comanche 055 page images.
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

# Page 819
# SERVICER207

; ============================================================================
; FILE: SERVICER207.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: all-phases
;
; TL;DR: Mission servicer background tasks performing housekeeping functions
;        and periodic updates during powered flight. Executes accelerometer
;        reading (PIPA), thrust monitoring, engine failure detection, and
;        state vector integration every 2 seconds to maintain navigation
;        accuracy during burns. Critical for TLI, LOI, TEI engine firings.
;
; COMMENT-ONLY READERS: This program handled routine background tasks to keep
;        the navigation system accurate during engine burns throughout the
;        Apollo 11 mission's major propulsive maneuvers.
; CODE-ALONG READERS: Study real-time periodic task scheduling, PIPA sensor
;        reading with restart protection, numerical integration of equations
;        of motion, and gravity compensation algorithms.
; ============================================================================

# PROGRAM NAME - PREREAD, READACCS, SERVICER, AVERAGE G.


# MOD NO. 00	BY M.HAMILTON	DEC.12, 1966


# FUNCTIONAL DESCRIPTION

# THE ROUTINES DESCRIBED BELOW ARE USED TO CALCULATE VALUES OF RN, VN, AND GDT/2 DURING ACCELERATED FLIGHT.
# THE SEVERAL ROUTINES COMPRISE A PACKAGE AND ARE NOT MEANT TO BE USED AS SEPARATE SUBROUTINES.

# GENERAL REFERENCES TO SERVICER OR AVERAGE G ARE UNDERSTOOD TO REFER TO THE ENTIRE SET OF ROUTINES INCLUDING
# READACCS, SERVICER, AVERAGE G, INTEREAD, SMOOTHER, AND ANY ADDITIONAL ROUTINES ATTACHED AT AVGEXIT (SEE BELOW).

# PROGRAMS INITIATING SERVICER ARE REQUIRED TO MAKE A WAITLIST CALL FOR PREREAD (OR, IF LIFTOFF, FOR BIBIBIAS)
# AT 2 SECONDS BEFORE THE FIRST AVERAGE G UPDATE IN ORDER TO INITIALIZE THE SEQUENCE, WHICH WILL RECUR EVERY
# 2 SECONDS FROM THAT TIME ON AS LONG AS AVEGFLAG REMAINS SET.

# THE USE OF ERASABLE AVGEXIT ALLOWS VARIOUS ROUTINES TO BE PERFORMED AS PART OF THE NORMAL CYCLE (SEE
# EXPLANATION OF AVGEXIT BELOW).

# DESCRIPTIONS OF INDIVIDUAL ROUTINES FOLLOW.


#	PREREAD

#		PREVIOUSLY EXTRAPOLATED VALUES COPIED FROM RN1, VN1, AND PIPTIME1 INTO RN, VN, AND PIPTIME.
#		LASTBIAS JOB SCHEDULED.
#		PIPS READ AND CLEARED VIA PIPASR SUBROUTINE.
#		AVERAGE G FLAG SET ON.
#		DRIFT FLAG SET OFF.
#		V37 FLAG SET ON.
#		INITIALIZATION OF	1) THRUST MONITOR (DVMON) - DVCNTR SET TO ONE.
#					2) TOTAL ACCUMULATED DELV VALUE (DVTOTAL) - SET TO ZERO.
#					3) AXIS VECTOR (AXIS) - SET TO (.5,0,0).
#		NORMLIZE JOB SCHEDULED.
#		READACCS TASH CALLED IN 2 SECONDS.


#	NORMLIZE
#
#		GDT/2 INITIALIZED VIA CALCGRAV SUBROUTINE.


#	READACCS

#		IF ONMON FLAG SET QUIKREAD ROUTINE IS PERFORMED BEFORE PIPASR ZEROS THE PIPA REGISTERS, AND THE 1/2 SEC
#		ONMONITOR LOOP IS INITIATED TO PROVIDE DOWNLINK INFORMATION DURING ENTRY.
#		PIPS READ AND CLEARED BY PIPASR SUBROUTINE.
#		IF CM/DSTBY IS ON, ENTRY VARIABLES INITIALIZED AND SETJTAG TASK CALLED.
# Page 820
# SERVICER207

#		IF AVERAGEG FLAG ON	READACCS CALLED TO RECYCLE IN 2 SECONDS.
#		IF AVERAGEG FLAG OFF	AVERAGE G EXIT (AVGEXIT) SET TO 2CADR AVGEND FOR FINAL PASS.
#		SERVICER JOB SCHEDULED.


#		TEST CONNECTOR OUTBIT TURNED ON.


# ONMNITOR

#		A SEQUENCE OF THREE PASSES THROUGH QUICREAD FOLLOWING A CALL TO READACCS WITH ONMONFLG SET AT 1/2
#		SEC INTERVALS.  INTERVALS ARE COUNTED OUT BY PIPCTR, INITIALISED AT 3 BY READACCS

#	QUIKREAD

#	      READS CURRENT PIPS INTO X,Y,ZPIPBUF.  READS OLD X,Y,ZPIPBUF INTO X,Y,ZOLDBUF.  VALUES ARE SENT TO
#		DOWNLIST DURING ENTRY.
#	SERVICER
#		DELV VALUES CHECKED TO DETECT RUNAWAY PIP -
#			IF BAD PIP	1) ALARM SENT.
#					2) COMPENSATION, DVTOTAL ACCUMULATION, AND DVMON BYPASSED.  CONTROL
#					   TRANSFERRED TO AVERAGE G.
#		PIPS COMPENSATED VIA 1/PIPA SUBROUTINE.
#		DVTOTAL INCREMENTED BY ABSOLUTE VALUE OF DELV.
#		THRUST MONITOR (DVMON) PERFORMED UNLESS IDLE FLAG IS ON.
#		CONTROL TRANSFERRED TO AVERAGE Q.


#		DVMON

#			THRESHOLD VALUD (PLACED IN DVTHRUSH BY USER) CHECKED AGAINST ABSOLUTE VALUE OF DELV TO CHECK
#			THRUST LEVEL.
#				IF THRUST	1) ULLAGE OFF ROUTINE PERFORMED.
#						2) STEERING FLAG TURNED ON AT FIRST DETECTION OF THRUST.
#						3) CONTROL TRANSFERRED TO AVERAGE G.
#				IF NO THRUST	1) ON FIRST PASS THROUGH MONITOR, CONTROL TRANSFERRED TO AVERAGE G.
#						2) ON SUBSEQUENT PASSES, CONTROL TRANSFERRED TO ENGINE FAIL ROUTINE IF THRUST
#						   HAS FAILED FOR 3 CONSECUTIVE PASSES.


#		ENGINE FAIL

#			ENGFAIL1 TASK CALLED IN 2.5 SECONDS.  THIS WILL RETURN CONTROL TO TIG-5 SO THAT THE IGNITION
#				SEQUENCE MAY BE REPEATED.
#			ENGINOF3 PERFORMED.
#			DAP SET UP FOR RCS.


#	AVERAGE G
# Page 821
#	RN1, VN1, GDT1/2 CALCULATED VIA CALCRVG ROUTINE BY UPDATING RN, VN WITH DELV AND AN AVERAGED VALUE
#	OF GDT/2.
#	RN1, VN1, GDT1/2, PIPTIME1 COPIED INTO RN, VN, GDT/2, PIPTIME FOR RESTART PROTECTION.
#	CONTROL TRANSFERRED TO ADDRESS SPECIFIED BY USER (OR BY READACCS FOR LAST PASS) IN AVGEXIT.
#	LAST PASS (AVGEND)	1) FREE FALL GYRO COMPENSATION SET UP.
#				2) DRIFT FLAG TURNED ON.
#				3) STATE VECTOR TRANSFERRED VIA AVETOMID ROUTINE.
#				4) ONMONITOR FLAG RESET.
#				5) V37 FLAG RESET.
#				6) TEST CONNECTOR OUTBIT RESET.
#				7) CONTROL TRANSFERRED TO CANV37 TO CONTINUE MM CHANGE ROUTINE (R00).


# CALLING SEQUENCE

#	PREREAD ENTERED DIRECTLY FROM TIG-30 VIA POSTJUMP.
#	READACCS CALLED AS WAITLIST TASK.


# SUBROUTINES CALLED

# 	UTILITY ROUTINES - PHASCHNG FLAGUP FLAGDOWN NOVAC FINDVAC WAITLIST ALARM NEWPHASE 2PHSCHNG

#	OTHER - PIPASR 1/PIPA CALCGRAV CALCRVG AVETOMID


# NORMAL EXIT MODES

#	ENDOFJOB	TASKOVER	CANV37

#	AVGEXIT -	THIS IS A DOUBLE PRECISION ERASABLE LOCATION BY WHICH CONTROL IS TRANSFERRED AT THE END
#			OF EACH CYCLE OF AVERAGE G.
#			THE 2CADR OF A ROUTINE TO BE PERFORMED AT THAT TIME (E.G., STEERING EQUATIONS TO BE PERFORMED
#			AT 2 SECOND INTERVALS) MAY BE SET BY THE USER INTO AVGEXIT.
#			ALL SUCH ROUTINES SHOULD RETURN TO SERVEXIT, WHICH IS THE NORMAL EXIT FROM AVERAGE G.

#	SERVEXIT -	DOES A PHASE CHANGE FOR RESTART PROTECTION AND GOES TO ENDOFJOB.
#			THE 2CADR OF SERVEXIT IS SET INTO AVGEXIT BY THE USER IF NO OTHER ROUTINE (SEE ABOVE).
#
#	AVGEND -	LAST PASS OF AVERAGE G EXITS HERE, BYPASSING SPECIAL ROUTINE (SEE ABOVE UNDER READACCS).
#			FINAL EXIT IS TO CANV37.				F AVERAGE G).


# OUTPUT

#	DVTOTAL(2)  PIPTIME(2)  XPIPBUF(2)  YPIPBUF(2)  ZPIPBUF(2)
#	RN(6)		REFERENCE COORD.   SCALED AT 2(+29) M/CS
#	VN(6)		REFERENCE COORD.   SCALED AT 2(+7) M/CS
#	GDT/2(6)	REFERENCE COORD.   SCALED AT 2(+7) M/CS
#	DELV(6)		STABLE MEMB. COORD.SCALED AT 2(+14)*5.85*10(-4)M/CS (KPIP1 USED TO GET DV/2 AT 2(+7))
# Page 822
#	DELVREF(6)	REFERENCE COORD.   SCALED AT 2(+7) M/CS

# INITIALIZATION

#		ONMONITOR FLAG SET BY ENTRY TO SHOW PIPBUF VALUES REQUIRED.
#		IDLE FLAG ON IF DVMON TO BE BYPASSED.
#		DVTHRUSH SET TO APPROPRIATE VALUE FOR DVMON.
#		AVGEXIT SET TO 2CADR OF ROUTINE, IF ANY, TO BE PERFORMED AFTER EACH CYCLE OF AVERAGE G.  IF NO ROUTINE
#			TO BE DONE, AVGEXIT SET TO SERVEXIT.
#		VALUES NEEDED
#			REFSMMAT
#			UNITW - FULL UNIT VECTOR, IN REFERENCE COORD., OF EARTH S ROTATIONAL VECTOR
#			RN1, VN1, PIPTIME1 - IN REFERENCE COORD., CONSISTENT WITH TIME OF EXECUTION OF PREREAD


# DEBRIS

#	CENTRALS	A, L, Q
#	OTHER		INTERNAL -  DVCNTR(1)  PIPAGE(1)  PIPCTR(1)  AVGEXIT(2)
#			EXTERNAL -  ITEMP1(1)  ITEMP2(1)  RUPTREG1(1)  TEMX(1)  TEMY(1)  TEMZ(1)
#			USEFUL DEBRIS
#				RN1(6)  VN1(6)  GDT1/2  PIPTIME1(2)
#					THESE LOCATIONS USED AS BUFFER STORAGE FOR NEWLY CALCULATED VALUES OF RN, VN, GDT/2,
#					AND PIPTIME DURING PERFORMANCE OF SERVICER ROUTINES.
#				UNITR - HALF UNIT VECTOR OF RN, REFERENCE COORD.
#				RMAG SCALED AT 2(+58) IN 36D.
#				RMAGSQ SCALED AT 2(+58) IN 34D.
#				(RE/RMAG)SQ IN 32D.


		BANK	27
		SETLOC	SERVICES
		BANK

		EBANK=	DVCNTR
# ********************* PREREAD ***************************************
#
; ============================================================================
; PREREAD ROUTINE - Initialization for 2-second SERVICER cycle
;
; This routine begins the periodic navigation update sequence used during
; powered flight. Called 2 seconds before first state vector update is needed.
; Schedules LASTBIAS for final gyro compensation in free fall, schedules
; NORMLIZE job to initialize gravity, then schedules READACCS task to begin
; the recurring 2-second cycle of accelerometer readings and navigation updates.
; Used during all major engine burns: TLI, LOI, TEI, and SPS maneuvers.
; ============================================================================

		COUNT	37/SERV

; Schedule LASTBIAS job to perform final gyro drift compensation before burn
PREREAD		CAF	PRIO21		# CALLER MUST PROTECT PREREAD
		TC	NOVAC
		EBANK=	NBDX
		2CADR	LASTBIAS	# DO LAST GYRO COMPENSATION IN FREE FALL

					# CALL-TO AND LASTBIAS ITSELF ARE NOT
					#	PROTECTED. REREADAC SETS 1/PIPADT
					#	TO 2.0 SECS IN CASE LASTBIAS LOST.
					#	(REDUNDANT IF LASTBIAS IS AOK)
# Page 823
; Copy extrapolated state vectors and initialize flags
REDO5.31	TC	PREREAD1

; Schedule NORMLIZE job to calculate initial gravity vector (GDT/2)
		CAF	PRIO32
		TC	FINDVAC		# SET UP NORMLIZE JOB REQUIRED PRIOR TO
		EBANK=	DVCNTR		# FIRST AVERAGE G PASS
		2CADR	NORMLIZE

; Schedule READACCS task to start in 2 seconds - begins recurring cycle
		CAF	2SECS
		TC	WAITLIST
		EBANK=	AOG
		2CADR	READACCS

		CS	TWO
		TC	NEWPHASE
		OCT	5

		TCF	TASKOVER

; PREREAD1 subroutine: Initialize state vectors, flags, and thrust monitoring
PREREAD1	EXTEND
		QXCH	RUPTREG1

; Read and clear PIPA counters one final time before powered flight begins
		TC	PIPASR		# CLEAR + READ PIPS LAST TIME IN FREE FALL

; Set up restart protection indicator
		CAF	ONE		# SET UP PIPAGE FOR REREADAC IN CASE A
		TS	PIPAGE		# 	RESTART OCCURS BEFORE READACCS

; Set AVERAGE G flag ON to enable recurring 2-second navigation updates
		CS	FLAGWRD1	# SET AVEG FLAG
		MASK	BIT1
		ADS	FLAGWRD1

; Clear DRIFT flag since we're entering powered flight (no longer coasting)
		CA	POSMAX
		MASK	FLAGWRD2
		TS	FLAGWRD2	# KNOCK DOWN DRIFT FLAG

; Set V37 flag for telemetry and display updates during burn
		CS	FLAGWRD7	# SET V37 FLAG
		MASK	BIT6
		ADS	FLAGWRD7

; Initialize total delta-V accumulator to zero for thrust monitoring
		CAF	ZERO
		TS	DVTOTAL		# CLEAR DVTOTAL
		TS	DVTOTAL +1

		TC	RUPTREG1

# Page 824
# ********************* READACCS ***************************************
; ============================================================================
; READACCS TASK - Recurring 2-second accelerometer reading and state update
;
; Called every 2 seconds during powered flight (AVERAGE G mode). Reads PIPA
; accelerometer counters, processes accumulated delta-V, schedules SERVICER
; job to update position/velocity state vectors. During entry, also saves
; PIPA readings every 0.5 seconds for telemetry downlink to Mission Control.
; This is the heartbeat of navigation during all major burns.
; ============================================================================
		EBANK=	AOG
; Read and clear PIPA counters (Pulsed Integrating Pendulous Accelerometers)
READACCS	TC	PIPASR

PIPSDONE	CAF	FIVE
		TS	L
		COM
		DXCH	-PHASE5

; Mark that PIPAs have been successfully read this cycle
REDO5.5		CAF	ONE		# SHOW PIPS HAVE BEEN READ
		TS	PIPAGE

; Initialize counter for ONMNITOR entry monitoring (3 additional 0.5-sec reads)
		CA	TWO		# SET PIPCTR FOR ONMINTOR
		TS	PIPCTR		# AFTER ABOVE PHASCHNG

; Check if in CM/DSTBY (Command Module standby) mode for entry phase
		CS	CM/FLAGS
		MASK	BIT2		# CM/DSTBY
		CCS	A
		TC	CHEKAVEG

; Entry phase: Save time base and angular variables for Entry DAP
		CS	PIPTIME1 +1
		TS	TBASE6		# FOR RESTARTS
		EXTEND			# CONTINUE FOR ENTRY DAP
		DCA	AOG
		DXCH	AOG/PIP
		CA	AMG
		XCH	AMG/PIP
		EXTEND
		DCA	ROLL/180
		DXCH	ROLL/PIP
		CA	BETA/180
		XCH	BETA/PIP
; Check if RCS DAP is inactive during entry (CM/DAPARM flag)
		CA	CM/FLAGS
		MASK	BIT12		# CM/DAPARM 93D BIT12
		EXTEND			# DURING ENTRY, WHEN RCS DAP IS INACTIVE,
		BZF	NOSAVPIP	# SAVE PIPAS EACH 0.5 SEC FOR TM.

; Schedule QUIKREAD for high-rate entry data downlink (0.5 sec intervals)
		CA	0.5SEC
		TC	WAITLIST
		EBANK=	XPIPBUF
		2CADR	QUIKREAD

; Save raw PIPA delta-V readings to telemetry buffers for Mission Control
					# NO NEED TO RESTART PROTECT THIS.
		CA	DELVX		# SAVE PIPAS AS READ (BUT NOT COMPENSATED)
		XCH	XPIPBUF
		TS	XOLDBUF

		CA	DELVY
		XCH	YPIPBUF
		TS	YOLDBUF
# Page 825
		CA	DELVZ
		XCH	ZPIPBUF
		TS	ZOLDBUF

; Continue entry processing without high-rate telemetry
NOSAVPIP	CA	FIVE
		TS	CM/GYMDT

; Schedule RCS jet firing monitor for entry control
		CA	JTAGTIME	# ACTIVATE CM/RCS AFTER PIPUP TO GO
					# IN JTAGTIME +5 CS.
		TC	WAITLIST
		EBANK=	AOG
		2CADR	SETJTAG

		CS	THREE		# 1.3SPOT FOR SETJTAG
		TC	NEWPHASE
		OCT	1

		CAF	OCT37
		TS	L
		COM
		DXCH	-PHASE5

; Check if AVERAGE G mode is still active (determines if 2-sec cycle continues)
CHEKAVEG	CS	FLAGWRD1
		MASK	BIT1
		CCS	A		# IF AVEG FLAG DOWN SET FINAL EXIT AVEG
		TC	AVEGOUT

; AVERAGE G active: schedule next READACCS cycle in 2 seconds
		CAF	2SECS
		TC	WAITLIST
		EBANK=	AOG
		2CADR	READACCS

; Schedule SERVICER job to process PIPA data and update state vectors
MAKESERV	CAF	PRIO20		# ESTABLISH SERVICER ROUTINE
		TC	FINDVAC
		EBANK=	DVCNTR
		2CADR	SERVICER

; Set restart protection for SERVICER and READACCS coordination
		CS	FOUR		# RESTART SERVICER AND READACCS
		TC	NEWPHASE
		OCT	5

; Turn on test connector output bit for ground monitoring
		CAF	BIT9
		EXTEND
		WOR	DSALMOUT	# TURN TEST CONNECTOR OUTBIT ON

		TCF	TASKOVER	# END PREVIOUS READACCS WAITLIST TASK

# Page 826
; AVERAGE G mode terminating: set exit handler to AVGEND for final pass
AVEGOUT		EXTEND
		DCA	AVOUTCAD
		DXCH	AVGEXIT
		TCF	MAKESERV

		EBANK=	DVCNTR
AVOUTCAD	2CADR	AVGEND

# Page 827
# ROUTINE NAME:	ONMNITOR
# MOD 04 BY BAIRNSFATHER 30 APR 1968	REDO ONMNITOR TO SAVE PIPS EACH 0.5 SEC FOR TM,ENTRY.
# MOD 03 BY FISHER DECEMBER 1967
# MOD 02 BY RYE SEPT 1967
# MOD 01 BY KOSMALA 23 MAR 1967
# MOD 00 BY KOSMALA 27 FEB 1967

# FUNCTIONAL DESCRIPTION

#	THE PURPOSE OF ONMONITOR IS TO PROVIDE 1/2 SEC.READING OF PIPAS FOR DOWNLIST DURING ENTRY.
#	X,Y,ZPIPBUF CONTAIN PRESET VALUES X,Y,ZOLDBUF CONTAIN VALUES FROM PREVIOUS READING.

# CALLING SEQUENCE

#	CALL AS WAITLIST TASK. TERMINATES ITSELF IN TASKOVER

# INITIALISATION

#	PIPCTR = 2 (FOR DT = 0.5 SEC)
#	X,Y,ZPIPBUF SET TO PREVIOUS PIPAX,Y,Z

# OUTPUT

#	X,Y,ZPIPBUF, X,Y,ZOLDBUF
# DEBRIS

#	X,Y,ZPIPBUF CONTAIN LAST PIPAX,Y,Z VALUES
#		X,Y,ZOLDBUF CONTAIN LAST-BUT-ONE PIPAX,Y,Z VALUES
#	RUPTREG1
#	PIPCTR

; ============================================================================
; ONMNITOR/QUIKREAD - High-rate entry telemetry sampling
;
; During atmospheric entry, Mission Control needs high-rate accelerometer
; data to monitor vehicle dynamics. ONMNITOR schedules 3 additional PIPA
; readings at 0.5-second intervals after each 2-second READACCS cycle.
; This provides 0.5-sec downlink resolution during critical reentry phase.
; ============================================================================
ONMNITOR	TS	PIPCTR

		TC	FIXDELAY	# WAIT
0.5SEC		DEC	50

; QUIKREAD entry point - saves PIPA readings for telemetry during entry
QUIKREAD	CAF	TWO
		TS	RUPTREG1
		INDEX	A
		CA	PIPAX		# SAVE ACTUAL PIPAS FOR TM.
		INDEX	RUPTREG1
		XCH	XPIPBUF		# UPDATE X,Y,ZPIPBUF
		INDEX	RUPTREG1
		TS	XOLDBUF		# AND X,Y,ZOLDBUF
; Loop through all 3 axes (X, Y, Z) to save PIPA values
CHKCTR		CCS	RUPTREG1
		TCF	QUIKREAD +1	# LOOP AGAIN
; Check if more 0.5-second samples needed (PIPCTR counts down from 2)
		CCS	PIPCTR
		TCF	ONMNITOR
		TC	TASKOVER

# Page 828
# ********************* SERVICER ***************************************
#
; ============================================================================
; SERVICER JOB - Main navigation state update during powered flight
;
; Scheduled by READACCS every 2 seconds during burns (TLI, LOI, TEI, descent,
; ascent). Processes accumulated delta-V from PIPA accelerometers, applies
; compensation for IMU bias and scale factors, updates position (RN) and
; velocity (VN) state vectors, and computes gravity (GDT/2) for next cycle.
; This is where the magic happens - raw accelerometer pulses become precise
; orbital state knowledge for guidance and navigation.
; ============================================================================

		EBANK=	DVCNTR

; Check for PIPA saturation before compensation (protect against sensor overflow)
SERVICER	CAF	TWO
		INHINT
; Loop through all 3 axes checking for PIPA saturation
PIPCHECK	TS	RUPTREG1

		DOUBLE
		INDEX	A
		CCS	DELVX
		TC	+2
		TC	PIPLOOP

; Test if accumulated delta-V exceeds maximum expected value (sensor saturation)
		AD	-MAXDELV	# DO PIPA-SATURATION TEST BEFORE
		EXTEND
		BZMF	PIPLOOP		# COMPENSATION.

; Issue alarm 00205 if PIPA saturated (extreme acceleration beyond sensor range)
		TC	ALARM
		OCT	00205		# SATURATED-PIPA ALARM   ***CHANGE LATER
		TC	AVERAGEG

PIPLOOP		CCS	RUPTREG1
		TCF	PIPCHECK

; Set restart protection - if power failure, resume at DVTOTUP after REREADAC
		TC	PHASCHNG	# RESTART REREADAC + SERVICER
		OCT	16035
		OCT	20000
		EBANK=	DVCNTR
		2CADR	DVTOTUP

; Apply PIPA bias and scale factor compensation to raw delta-V measurements
		TC	BANKCALL	# PIPA COMPENSATION CALL
		CADR	1/PIPA

; Accumulate total delta-V magnitude for thrust monitoring (DVTOTAL)
DVTOTUP		TC	INTPRET
		VLOAD	ABVAL		# GET ABS VALUE OF DELV
			DELV
		DMP	EXIT
			KPIP1		# SCALE AT 2(+7)

		EXTEND
		DCA	MPAC
		DAS	DVTOTAL		# ACCUMULATE DVTOTAL
; Main state integration routine - update position, velocity, and gravity
AVERAGEG	TC	PHASCHNG
		OCT	10035

		TC	INTPRET
		CALL
# Page 829
; CALCRVG: Integrates state vector using compensated delta-V and current gravity
			CALCRVG
		EXIT

		TC	PHASCHNG
		OCT	10035

; Copy updated state vectors from integration workspace to current state
		CAF	OCT31		# COPY RN1,VN1,GOT102,GOBL1/2,PIPTIME1
		TC	GENTRAN		# INTO RN ,VN ,GDT/12 ,GOBL/2 ,PIPTIME
		ADRES	RN1
		ADRES	RN
		RELINT			# GENTRAN DOES AN INHINT
		TC	PHASCHNG
		OCT	10035

; Jump to program-specific exit routine (set by AVGEXIT variable)
		EXTEND
		DCA	AVGEXIT
		DXCH	Z		# AVERAGEG EXIT

AVGEND		CA	PIPTIME +1	# FINAL AVERAGE G EXIT
		TS	OLDBT1		# SET UP FREE FALL GYRO COMPENSATION

		TC	UPFLAG		# SET DRIFTFLG
		ADRES	DRIFTFLG	# BIT 15 FLAG 2
		TC	2PHSCHNG
		OCT	5		# GROUP 5 OFF
		OCT	05022		# GROUP 2 ON FOR AVETOMID
		OCT	20000

		TC	INTPRET
		CALL
			AVETOMID	# CONVERT STATE VECTOR TO REFERENCE SCALE.
		EXIT

; Zero optical navigation mark counters for new cycle
		CAF	ZERO		# ZERO MARK COUNTERS.
		TS	VHFCNT
		TS	TRKMKCNT

; Release PIPA registers for next reading cycle
		TC	BANKCALL
		CADR	PIPFREE

; Invalidate mark buffer and disable alarm output
		CS	BIT9
		TS	MRKBUF2		# INVALIDATE MARK BUFFER
		EXTEND
		WAND	DSALMOUT

; Clear CM/Docked Standby flag (end of entry monitoring mode)
		TC	DOWNFLAG
		ADRES	CM/DSTBY

; Clear V37 flag (end of powered flight monitoring mode)
		TC	DOWNFLAG
		ADRES	V37FLAG

# Page 830
; If P20 rendezvous program is active, restore restart protection groups
		CAF	BIT7		# RESTORE GROUP 1 + 2 IF P20 IS RUNNING.
		MASK	FLAGWRD0
		EXTEND
		BZF	+4

		TC	2PHSCHNG
		OCT	111		# 1.11SPOT
		OCT	132		# 2.13SPOT

; Jump to V37 cancellation routine to clean up display interface
		TC	POSTJUMP
		CADR	CANV37

; Exit point for SERVICER job - sets restart protection and terminates
SERVEXIT	TC	PHASCHNG
		OCT	00035		# A, 5.3 = REREADAC (ONLY)

		TCF	ENDOFJOB

DVTHRUSH	EQUALS	ELEVEN		# 15 PERCENT OF 2SEC PIPA ACCUMULATION,
					#	FOR 503-FULL CSM/LEM....DELV SC.AT
					#	5.85 CM/SEC.

-MAXDELV	DEC	-6398		# 3200 PPS FOR 2 SEC CCS TAKES 1

JTAGTIME	DEC	120		# = 1 SEC + T CDU, T CDU = .1 SEC

2.5SEC		DEC	250
MDOTFAIL	DEC	144.0 B-16	# 5 SEC MASS LOSS AT 28.8 KG/SEC
					# SHOULD BE 2-4 SECS FOR NO START
					#	    6-8 SECS FOR FAILURE

# Page 831
# NORMLIZE PERFORMS THE INITIALIZATION REQUIRED PRIOR TO THE FIRST ENTRY TO AVERAGEG, AND SCALES RN SO THAT IT
# HAS 1 LEADING BINARY ZERO.  IN MOST MISSIONS, RN WILL BE SCALED AT 2(+29), BUT IN THE 206 MISSION, RN WILL BE
# SCALED AT 2(+24)M.

; ============================================================================
; NORMLIZE JOB - Initialize gravity computation for powered flight
;
; Scheduled by PREREAD before first AVERAGEG cycle. Copies extrapolated
; state vectors from integration workspace (RN1, VN1) to current state
; (RN, VN), then computes initial gravity vector GDT/2 and oblate correction
; GOBL/2 based on current position. This sets the baseline for subsequent
; 2-second integration cycles during engine burns.
; ============================================================================
NORMLIZE	CAF	THIRTEEN	# SET UP TO COPY 14 REGS- RN1,VN1,PIPTIME1
		TC	GENTRAN		# INTO RN,VN, PIPTIME
		ADRES	RN1		# FROM HERE
		ADRES	RN		# TO HERE

		RELINT
; Compute initial gravity vector based on current position
		TC	INTPRET
		VLOAD	CALL		# LOAD RN FOR CALCGRAV
			RN
			CALCGRAV	# INITIALISE UNITR RMAG GDT1

; Store gravity and oblate Earth correction for integration
		STOVL	GDT/2
			GOBL1/2
		STORE	GOBL/2
		EXIT
		TCF	ENDOFJOB

# Page 832
# *****  PIPA READER *****


# 		MOD NO. 00 BY D. LICKLY DEC.9 1966


# FUNCTIONAL DESCRIPTION

# 	SUBROUTINE TO READ PIPA COUNTERS, TRYING TO BE VERY CAREFUL SO THAT IT WILL BE RESTARTABLE.
# 	PIPA READINGS ARE STORED IN THE VECTOR DELV.  THE HIGH ORDER PART OF EACH COMPONENT CONTAINS THE PIPA READING,
# 	RESTARTS BEGIN AT REREADAC.


# 	AT THE END OF THE PIPA READER THE CDUS ARE READ AND STORED AS A
# VECTOR IN CDUTEMP.  THE HIGH ORDER PART OF EACH COMPONENT CONTAINS
# THE CDU READING IN 2S COMP IN THE ORDER CDUX,Y,Z.  THE THRUST
# VECTOR ESTIMATOR IN FINDCDUD REQUIRES THE CDUS BE READ AT PIPTIME.

# CALLING SEQUENCE AND EXIT

#	CALL VIA TC, ISWCALL, ETC.

#	EXIT IS VIA Q.


# INPUT

#	INPUT IS THROUGH THE COUNTERS PIPAX, PIPAY, PIPAZ, AND TIME2.


# OUTPUT

#	HIGH ORDER COMPONENTS OF THE VECTOR DELV CONTAIN THE PIPA READINGS.
#	PIPTIME CONTAINS TIME OF PIPA READING.


# DEBRIS (ERASABLE LOCATIONS DESTROYED BY THE PROGRAM)

#		LOW ORDER DELV'S ARE ZEROED FOR TM INDICATION.
#		TEMX	TEMY	TEMZ	PIPAGE

; ============================================================================
; PIPASR SUBROUTINE - Read and zero PIPA accelerometer counters
;
; The Pulsed Integrating Pendulous Accelerometers (PIPAs) are hardware pulse
; counters that accumulate acceleration pulses from the IMU. This routine
; atomically reads all three axis counters (PIPAX, PIPAY, PIPAZ), stores the
; readings in DELV vector, and zeros the counters for the next integration
; interval. Extreme care is taken to make this restartable in case of power
; failure or computer restart during the read sequence.
; ============================================================================
; ============================================================================
; Read all three PIPA accelerometer counters and clear them atomically
; ============================================================================
; The PIPA (Pulsed Integrating Pendulous Accelerometer) counters accumulate
; velocity changes from the IMU accelerometers. This routine reads all three
; axes (X, Y, Z) and clears the hardware counters to begin the next integration
; period. The read must be atomic to ensure consistent measurements.
;
; Restart protection: If interrupted mid-read, REREADAC logic below determines
; which PIPs were successfully read and resumes from the correct point.
PIPASR		EXTEND
		DCA	TIME2
		DXCH	PIPTIME1	# CURRENT TIME	POSITIVE VALUE
; Initialize temporary storage to -0 for proper restart behavior
		CS	ZERO		# INITIALIZE THESE AT NEG ZERO.
		TS	TEMX
		TS	TEMY
		TS	TEMZ
# Page 833
; Clear delta-V accumulators to prepare for new PIPA readings
		CA	ZERO
		TS	DELVZ		# OTHER DELVS OK INCLUDING LOW ORDER
		TS	DELVY

		TS	DELVX +1	# LOW ORDER DELV'S ARE ZEROED FOR TM: THUS
		TS	DELVY +1	# IF DNLNK'D LOW ORDER DELVS ARE NZ, THEY
		TS	DELVZ +1	# CONTAIN PROPER COMPENSATION.  IF=0, THEN
					# THE TM VALUES ARE BEFORE COMPENSATION.

; Mark PIPA reading as in-progress (restart protection flag)
		TS	PIPAGE		# SHOW PIPA READING IN PROGRESS

; ============================================================================
; Read X and Y PIPAs simultaneously (double-precision read)
; ============================================================================
REPIP1		EXTEND
		DCS	PIPAX		# X AND Y PIPS READ
		DXCH	TEMX
		DXCH	PIPAX		# PIPAS SET TO NEG ZERO AS READ.
		TS	DELVX
		LXCH	DELVY

; ============================================================================
; Read Z PIPA (single-precision read)
; ============================================================================
REPIP3		CS	PIPAZ		# REPEAT PROCESS FOR Z PIP
		XCH	TEMZ
		XCH	PIPAZ
; Store Z delta-V and return to caller
DODELVZ		TS	DELVZ

		TC	Q

		EBANK=	AOG

; ============================================================================
; RESTART RECOVERY LOGIC FOR PIPASR
; ============================================================================
; If a restart occurs during PIPA reading, this logic determines which PIPs
; were successfully read and resumes from the appropriate point. The AGC uses
; temporary storage (TEMX, TEMY, TEMZ) and delta-V flags to reconstruct state.
;
; Recovery strategy:
; 1. Check if Z delta-V completed -> resume at PIPSDONE (CDU reading)
; 2. Check if Y delta-V completed -> complete Z reading
; 3. Check if X delta-V started -> complete Y and Z readings
; 4. Nothing completed -> restart entire PIPA read from REPIP1
;
; This restart protection was critical during Apollo 11 - any brief power
; transient during descent couldn't corrupt navigation state.
REREADAC	CCS	PHASE5		# LAST PASS CHECK
		TCF	+2
		TCF	TASKOVER

; Restore 1/PIPADT timer in case restart cleared it
		CAF	PRIO31		# RESTART MAY HAVE WIPED OUT LASTBIAS, AN
		TS	1/PIPADT	#	UNPROTECTED NOVAC FROM PREREAD,
					#	WHICH SET(S) UP 1/PIPADT (THUSLY)
					#	FOR NON-COASTING COMPENSATION....BE
					#	SURE 1/PIPADT IS AOK.  (PRIO31 IS
					#	2.0SEC SC.AT B+8CS)

; Check if PIPA reading even started
		CCS	PIPAGE
		TCF	READACCS	# PIP READING NOT STARTED.  GO TO BEGINNING

; Set return address to PIPSDONE for when PIPA reads complete
		CAF	DONEADR		# SET UP RETURN FROM PIPASR
		TS	Q

; Check if Z axis completed (furthest point in read sequence)
		CCS	DELVZ
		TC	Q		# Z DONE, GO DO CDUS
		TCF	+3		# Z NOT DONE, CHECK Y.
		TC	Q
		TC	Q
# Page 834
; Check if Y axis completed (middle of read sequence)
		ZL
		CCS	DELVY
		TCF	+3
		TCF	CHKTEMX		# Y NOT DONE, CHECK X.
		TCF	+1
		LXCH	PIPAZ		# Y DONE, ZERO Z PIP.

; Y was completed, now complete Z axis reading
		CCS	TEMZ
		CS	TEMZ		# TEMZ NOT = -0, CONTAINS -PIPAZ VALUE.
		TCF	DODELVZ
		TCF	-2
		LXCH	DELVZ		# TEMZ = -0, L HAS ZPIP VALUE.
		TC	Q

; Check if X axis reading was started (earliest point in sequence)
CHKTEMX		CCS	TEMX		# HAS THIS CHANGED
		CS	TEMX		# YES
		TCF	+3		# YES
		TCF	-2		# YES
		TCF	REPIP1		# NO - restart entire read from beginning
; X was started, complete Y and Z readings
		TS	DELVX

		CS	TEMY
		TS	DELVY

		CS	ZERO		# ZERO X AND Y PIPS
		DXCH	PIPAX		# L STILL ZERO FROM ABOVE

		TCF	REPIP3

DONEADR		GENADR	PIPSDONE

# Page 835
; ============================================================================
; CALCRVG - EQUATIONS OF MOTION INTEGRATION WITH GRAVITY COMPENSATION
; ============================================================================
# *********************************************************************************************

# 	ROUTINE CALCRVG INTEGRATES THE EQUATIONS OF MOTION BY AVERAGING THE THRUST AND GRAVITATIONAL
# ACCELERATIONS OVER A TIME INTERVAL OF 2 SECONDS.
#
# 	FOR THE EARTH-CENTERED GRAVITATIONAL FIELD, THE PERTURBATION DUE TO OBLATENESS IS COMPUTED TO THE FIRST
# HARMONIC COEFFICIENT J.

# 	ROUTINE CALCRVG REQUIRES...
#		1) THRUST ACCELERATION INCREMENTS IN DELV SCALED SAME AS PIPAX,Y,Z IN STABLE MEMBER COORDS.
#		2) VN SCALED 2(+7)M/CS IN REFERENCE COORDS.
#		3) RN SCALED AT 2(+29) METERS IN REFERENCE COORDS.
#		4) UNITW THE EARTH S UNIT ROTATIONAL VECTOR (SCALED AS A FULL UNIT VECTOR) IN REFERENCE COORDS.

# IT LEAVES RN1 UPDATED (SCALED AT 2(+29)M, VN1 (SCALED AT 2(+7)M/CS), AND GDT1/2 (SCALED AT 2(+7)M/CS). ALSO HALF
# UNIT VECTOR UNITR, RMAG IN 36D SCALED AT 2(+29)M, R MAG SQ. IN 34D SCALED AT 2(+58) M SQ.

; ============================================================================
; CALCGRAV - Calculate gravitational acceleration with oblateness correction
; ============================================================================
; Computes GDT/2 (half the gravitational acceleration over 2-second interval)
; accounting for Earth oblateness (J2 harmonic) or lunar point mass.
;
; For Earth: Includes first-order oblateness correction using J2 coefficient
; For Moon: Simple inverse-square law (point mass approximation)
;
; Entry: RN (position vector) in MPAC, scaled at 2^29 meters
; Exit: GDT1/2 scaled at 2^7 m/cs, UNITR (unit position vector), RMAG
CALCGRAV	UNIT	PUSH		# ENTER WITH RN IN MPAC
; Compute unit position vector UNITR = RN / |RN|
		STORE 	UNITR
; Load index register and check if orbiting Moon or Earth
		LXC,1	SLOAD
			RTX2
			X1
		BMN	VLOAD
			ITISMOON	# Negative X1 indicates lunar orbit
; ============================================================================
; Earth oblateness correction (J2 harmonic term)
; ============================================================================
; Compute oblateness perturbation: J2 term depends on (UNITR·UNITW)^2
; where UNITW is Earth's rotation axis unit vector
		DOT	PUSH		# (UNITR · UNITW)
			UNITW
		DSQ	BDSU		# (UNITR · UNITW)^2 - 1/20
			DP1/20
		PDDL	DDV		# Compute (RE/RN)^2 ratio
			RESQ		# RE^2 (Earth radius squared)
			34D		# (RN)SQ
		STORE	32D		# TEMP FOR (RE/RN)SQ
; J2 oblateness term: 20*J*(RE/RN)^2*[(UNITR·UNITW)^2 - 1/20]*UNITR
		DMP	DMP
			20J		# 20*J2 coefficient
		VXSC	PDDL		# Scale by UNITR
			UNITR
; Second J2 term: 2*J*(RE/RN)^2*UNITW (rotational axis contribution)
		DMP	DMP
			2J
			32D
		VXSC	VAD		# Scale by UNITW and add to first term
			UNITW
		STADR
		STORE	GOBL1/2		# Oblateness correction vector
		VAD	PUSH		# Add to UNITR for total direction
			UNITR
; ============================================================================
; Inverse-square law computation (both Earth and Moon)
; ============================================================================
ITISMOON	DLOAD	NORM		# Load R^2 and normalize for precision
			34D
			X2
		BDDV*	SLR*		# Divide by -μ*Δt (gravitational parameter * time)
# Page 836
			-MUDT(E),1	# Select Earth or Moon μ based on index
			0 -21D,2	# Shift for proper scaling
		VXSC	STADR		# Scale direction vector by magnitude
		STORE	GDT1/2		# SCALED AT 2(+7) M/CS
		RVQ			# Return to caller

; ============================================================================
; CALCRVG - Integrate position and velocity using averaged accelerations
; ============================================================================
; Updates RN, VN by integrating equations of motion over 2-second interval:
;   RN1 = RN + VN*Δt + 0.5*(ΔV + GDT/2)*Δt^2
;   VN1 = VN + 0.5*(ΔV_new + ΔV_old + GDT_new - GDT_old)
;
; This uses trapezoidal integration for improved accuracy.
CALCRVG		VLOAD	VXSC
			DELV		# Thrust acceleration from PIPAs
			KPIP1		# Scale: 1 PIPA pulse = 5.85 cm/sec
; Transform from stable member coordinates to reference inertial coordinates
		VXM	VSL1
			REFSMMAT	# Reference to stable member matrix
		STORE	DELVREF		# DELV IN REF COORDS AT 2(+7)
; Compute average acceleration: (thrust - old_gravity)/2
		VSR1	PUSH		# Divide ΔV by 2
		VAD	PUSH		# (DV-OLDGDT)/2 TO PD SCALED AT 2(+7)M/CS
			GDT/2		# Add previous gravity (already halved)
; Position update: RN1 = RN + VN*Δt + 0.5*avg_accel*Δt^2
		VAD	VXSC		# VN + average_accel
			VN
			2SEC(22)	# Multiply by Δt = 2 seconds
		VAD	STQ		# Add to RN for new position
			RN
			31D		# Save return address
		STCALL	RN1		# TEMP STORAGE OF RN SCALED 2(+29)M
			CALCGRAV	# Compute new gravity at updated position

; Velocity update using trapezoidal rule:
; VN1 = VN + 0.5*(ΔV_new + old_avg) where old_avg already in pushlist
		VAD	VAD		# Add new gravity to ΔV_new
		VAD			# Add (ΔV_old + GDT_old)/2 from pushlist
			VN		# Add to previous velocity
		STCALL	VN1		# TEMP STORAGE OF VN SCALED 2(+7)M/CS.
			31D		# Return to saved address

; ============================================================================
; Physical and scaling constants for gravity and navigation computations
; ============================================================================
KPIP		2DEC	.1024		# SCALES DELV TO 2(+4)

KPIP1		2DEC	0.074880	# 207 DELV SCALING.  1 PULSE = 5.85 CM/SEC.
					# Converts PIPA pulses to m/cs velocity increments

-MUDT(E)	2DEC*	-7.9720645 E+12 B-44*
					# -μ*Δt for Earth (gravitational parameter * 2 sec)
					# μ_Earth = 3.986004×10^14 m^3/s^2

-MUDT(M)	2DEC*	-9.805556 E+10 B-44*
					# -μ*Δt for Moon (gravitational parameter * 2 sec)
					# μ_Moon = 4.903×10^12 m^3/s^2

2SEC(22)	2DEC	200 B-22		# Time interval Δt = 2 seconds scaled at B-22

DP1/20		2DEC	0.05			# Constant 1/20 = 0.05 for J2 calculations

RESQ		2DEC*	40.6809913 E12 B-59*
					# R_Earth^2 (Earth radius squared)
					# R_Earth ≈ 6378.145 km

20J		2DEC*	3.24692010 E-2 B1*
					# 20*J2 where J2 = 1.62346×10^-3
					# J2 is Earth's oblateness coefficient

2J		2DEC*	3.24692010 E-3 B1*
					# 2*J2 coefficient for rotational axis term
