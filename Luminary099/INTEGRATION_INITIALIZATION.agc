# Copyright:	Public domain.
# Filename:	INTEGRATION_INITIALIZATION.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1205-1226
# Mod history:	2009-05-26 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-01-06 JL	Fixed pseudo-label indentation.
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
; FILE: INTEGRATION_INITIALIZATION.agc
; MODULE: Orbital Integration Initialization
; MISSION PHASE: trans-lunar/lunar-orbit/descent/ascent/trans-earth
;
; TL;DR: Initializes numerical integration system for orbit propagation using
;        Encke method with precision perturbations or Kepler conic sections.
;        Sets up coordinate system transformations, initial conditions, state
;        vector scaling, and rectification thresholds. Critical for accurate
;        navigation state updates throughout cislunar flight phases.
;
; COMMENT-ONLY READERS: This module prepares the mathematics that keeps the
;        spacecraft on course between Earth and Moon. Read to understand how
;        the AGC transitions between different integration modes during flight.
; CODE-ALONG READERS: Study Encke method initialization, coordinate frame
;        setup, scaling transformations for Earth/Moon spheres, and the job
;        stalling mechanism for background integration tasks.
; ============================================================================

# Page 1205
# 1.0 INTRODUCTION
# ----------------
#
# FROM A USER'S POINT OF VIEW, ORBITAL INTEGRATION IS ESSENTIALLY THE SAME AS THE 278 INTEGRATION
# PROGRAM.  THE SAME ENTRANCES TO THE PROGRAM WILL BE MAINTAINED, THE SAME STALLING ROUTINE WILL BE USED AND
# OUTPUT WILL STILL BE VIA THE PUSHLIST.  THE PRIMARY DIFFERENCES TO A USER INVOLVE THE ADDED CAPABILITY OF
# TERMINATING INTEGRATION AT A SPECIFIC FINAL RADIUS AND THE DIFFERENCE IN STATE VECTOR SCALING INSIDE AND OUTSIDE
# THE LUNAR SPHERE OF INFLUENCE.
#
# IN ORDER TO MAKE THE CSM(LEM)PREC AND CSM(LEM)CONIC ENTRANCES SIMILAR TO FLIGHT 278, THE INTEGRATION PROGRAM
# WILL ITSELF SET THE FINAL RADIUS (RFINAL) TO 0 SO THAT REACHING THE DESIRED TIME ONLY WILL TERMINATE
# INTEGRATION.  THE DP REGISTER RFINAL MUST BE SET BY USERS OF INTEGRVS AND INTEGRV, AND MUST BE DONE AFTER THE
# CALL TC INTSTALL.
#
# WHEN THE LM IS ON THE LUNAR SURFACE (INDICATED BY LUNAR SURFACE FLAG SET) CALLS TO LEMCONIC, LEMPREC, AND
# INTEGRV WITH VINFLAG = 0 WILL RESULT IN THE USE OF THE PLANETARY INERTIAL ORIENTATION SUBROUTINES TO PROVIDE
# BOTH THE LM'S POSITION AND VELOCITY IN THE REFERENCE COORDINATE SYSTEM.
# THE PROGRAM WILL PROVIDE OUTPUT AS IF INTEGRATION WAS USED.  THAT IS, THE PUSHLIST WILL BE SET AS NOTED BELOW AND
# THE PERMANENT STATE VECTOR UPDATED WHEN SPECIFIED BY AN INTEGRV CALL.
#
# USERS OF INTEGRVS DESIRING INTEGRATION (INTYPFLG = 0) SHOULD NOTE THAT THE OBLATENESS PERTURBATION COMPUTATION
# IN LUNAR ORBIT IS TIME DEPENDENT.  THEREFORE, THE USER SHOULD SUPPLY AN INITIAL STATE VECTOR VALID AT SOME REAL
# TIME AND THE DESIRED TIME (TDEC1) ALSO AT SOME REAL TIME.  FOR CONIC "INTEGRATION" THE USER MAY STILL USE ZERO
# AS THE INITIAL TIME AND DELTA TIME AS THE DESIRED TIME.
#
# 2.0 GENERAL DESCRIPTION
# -----------------------
#
# THE INTEGRATION PROGRAM OPERATES AS A CLOSED INTERPRETIVE SUBROUTINE AND PERFORMS THESE FUNCTIONS --
#	1) INTEGRATES (PRECISION OR CONIC) EITHER CSM OR LM STATE VECTOR
#	2) INTEGRATES THE W-MATRIX
#	3) PERMANENT OR TEMPORARY UPDATE OF THE STATE VECTOR
#
# THERE ARE SIX ENTRANCES TO THE INTEGRATION PROGRAM.  FOUR OF THESE (CSMPREC, LEMPREC, CSMCONIC, LEMCONIC) SET
# ALL THE FLAGS REQUIRED IN THE INTEGRATION PROGRAM ITSELF TO CAUSE THE PRECISION OR CONIC INTEGRATION (KEPLER) OF
# THE LM OR CSM STATE VECTOR, AS THE NAMES SUGGEST.  ONE ENTRANCE (INTEGRVS) PERMITS THE CALLING PROGRAM TO
# PROVIDE A STATE VECTOR TO BE INTEGRATED.  THE CALLING PROGRAM MUST SET THE FLAGS INDICATING (1) PRECISION OR
# CONIC INTEGRATION, (2) IN OR OUT OF LUNAR SPHERE, (3) MIDCOURSE OR NOT, AND THE INTEGRATION PROGRAM COMPLETES
# THE FLAG SETTING TO BYPASS W-MATRIX INTEGRATION.  THE LAST ENTRANCE (INTEGRV, USED IN GENERAL BY THE
# NAVIGATION PROGRAMS) PERMITS THE CALLER TO SET FIVE FLAGS (NOT MOONFLAG OR MIDFLAG) BUT NOT TO INPUT A STATE
# VECTOR.  ANY PROGRAM WHICH CALLS INTEGRVS OR INTEGRV MUST CALL INTSTALL BEFORE IT SETS THE INTEGRATION FLAGS
# AND/OR STATE VECTOR.
#
# THREE SETS OF 42 REGISTERS AND 2 FLAGS ARE USED FOR THE STATE VECTORS.  TWO SETS, WHICH MAY NOT BE OVERLAYED, ARE
# USED FOR THE PERMANENT STATE VECTORS FOR THE CSM AND LM.  THE THIRD SET, WHICH MAY BE OVERLAYED WHEN INTEGRATION
# IS NOT BEING DONE, IS USED IN THE COMPUTATIONS.
#
# THE PERMANENT STATE VECTORS WILL BE PERIODICALLY UPDATED SO THAT THE VECTORS WILL NOT BE OLDER THAN 4 TIMESTEPS.
# THE PERMANENT STATE VECTORS WILL ALSO BE UPDATED WHENEVER THE W-MATRIX IS INTEGRATED OR WHEN A CALLER OF INTEGRV
# SETS STATEFLG (THE NAVIGATION PROGRAMS P20, P22.)
#
# Page 1206
# APPENDIX B OF THE USERS' GUIDE LISTS THE STATE VECTOR QUANTITIES.
#
# 2.1 RESTARTS
#
# PHASE CHANGES WILL BE MADE IN THE INTEGRATION PROGRAM ONLY FOR THE INTEGRV ENTRANCE (I.E., WHEN THE W-MATRIX IS
# INTEGRATED OR PERMANENT STATE VECTOR IS UPDATED.)  THE GROUP NUMBER USED WILL BE THAT FOR THE P20-25 PROGRAMS
# (I.E., GROUP2) WINCE THE INTEGRV ENTRANCE WILL ONLY BE USED BY THESE PROGRAMS.  IF A RESTART OCCURS DURING AN
# INTEGRATION OF THE STATE VECTOR ONLY, THE RECOVERY WILL BE TO THE LAST PHASE IN THE CALLING PROGRAM.  CALLING
# PROGRAMS WHICH USE THE INTEGRV OR INTEGRVS ENTRANCE OF INTEGRATION WHOULD ENSURE THAT IF PHASE CHANGING IS DONE
# THAT IT IS PRIOR TO SETTING THE INTEGRATION INPUTS IN THE PUSHLIST.
# THIS IS BECAUSE THE PUSHLIST IS LOST DURING A RESTART.
#
# 2.2 SCALING
#
# THE INTEGRATION ROUTINE WILL MAINTAIN THE PERMANENT MEMORY STATE VECTORS IN THE SCALING AND UNITS DEFINED IN
# APPENDIX B OF THE USERS GUIDE.  THE SCALING OF THE OUTPUT POSITION VECTOR DEPENDS ON THE ORIGIN OF THE COORDINATE
# SYSTEM AT THE DESIRED INTEGRATION TIME.  THE COORDINATE SYSTEM TRANSFORMATION WILL BE DONE AUTOMATICALLY ON
# MULTIPLE TIMESTEP ENCKE INTEGRATION ONLY.  THUS IT IS POSSIBLE TO HAVE OUTPUT FROM SUCCESSIVE INTEGRATIONS IN
# DIFFERENT SCALING.
# HOWEVER, RATT, VATT WILL ALWAYS BE SCALED THE SAME.
#
# 3.0 INPUT/OUTPUT
# ----------------
#
# PROGRAM INPUTS ARE THE FLAGS DESCRIBED IN APPENDIX A AND THE PERMANENT STATE VECTOR QUANTITIES DESCRIBED IN
# APPENDIX B OF THE USERS GUIDE, PLUS THE DESIRED TIME TO INTEGRATE TO IN TDEC1 (A PUSH LIST LOCATION).
# FOR INTEGRVS, THE RCV,VCV,TET OR THE TEMPORARY STATE VECTOR MUST BE SET, PLUS MOONFLAG AND MIDFLAG
#
# FOR SIMULATION THE FOLLOWING QUANTITIES MUST BE PRESET ---
#										EARTH	MOON
#										 29	 27
#	RRECTCSM(LEM) 		RECTIFIED POSITION VECTOR	METERS		2	2
#
#										 7	 5
#	VRECTCSM(LEM)		RECTIFIED VELOCITY VECTOR	M/CSEC		2	2
#
#										 28	 28
#	TETCSM(LEM)		TIME STATE VECTOR IS VALID	CSEC		2	2
#				CUSTOMARILY 0, BUT NOTE LUNAR
#				ORBIT DEPENDENCE ON REAL TIME.
#
#										 22	 18
#	DELTAVCSM(LEM)		POSITION DEVIATION		METERS		2	2
#				0 IF TCCSM(LEM) = 0
#
#										 3	 -1
#	NUVCSM(LEM)		VELOCITY DEVIATION		M/CSEC		2	2
#				0 IF TCCSM(LEM) = 0
# Page 1207
#										 29	 27
#	RCVSM(LEM)		CONIC POSITION			METERS		2	2
#				EQUALS RRECTCSM(LEM) IF
#				TCCSM(LEM) = 0
#
#										 7	 5
#	VCVCSM(LEM)		CONIC VELOCITY			M/CSEC		2	2
#				EQUALS VRECTCSM(LEM) IF
#				TCCSM(LEM) = 0
#
#										 28	 28
#	TCCSM(LEM)		TIME SINCE RECTIFICATION	CSECS		2	2
#				CUSTOMARILY 0
#
#								 1/2		 17	 16
#	XKEPCSM(LEM)		ROOT OF KEPLER'S EQUATION	M		2	2
#				0 IF TCCSM(LEM) = 0
#
#	CMOONFLG		PERMANENT FLAGS CORRESPONDING			0	0
#	CMIDFLAG		TO MOONFLAG AND MIDFLAG				0,1	0,1
#	LMOONFLG		C = CSM, L = LM					0	0
#	LMIDFLG									0,1	0,1
#
#	SURFFLAG		LUNAR SURFACE FLAG				0,1	0,1
#
# IN ADDITION, IF (L)CMIDFLAG IS SET, THE INITIAL INPUT VALUES FOR LUNAR
# SOLAR EPHEMERIDES SUBROUTINE AND PLANETARY INERTIAL ORIENTATION SUB-
# ROUTINE MUST BE PRESET.
#
# OUTPUT
#	AFTER EVERY CALL TO INTEGRATION
#									EARTH	MOON
#									 29	 29
#	0D	RATT	POSITION			METERS		2	2
#
#									 7	 7
#	6D	VATT	VELOCITY			M/CSEC		2	2
#
#									 28	 28
#	12D	TAT	TIME						2	2
#
#									 29	 27
#	14D	RATT1	POSITION			METERS		2	2
#
#									 7 	 5
#	20D	VATT1	VELOCITY			M/CSEC		2	2
#
#						 	 3   2		 36	 30
#	26D	MU(P)	MU				M /CS		2	2
#
#	X1		MUTABLE ENTRY					-2	-10D
#
#	X2		COORDINT
#	X2		COORDINATE SYSTEM ORIGIN			0	2
#			(THIS, NOT MOONFLAG, SHOULD BE
# Page 1208
#			USED TO DETERMINE ORIGIN.)
#
# IN ADDITION TO THE ABOVE, THE PERMANENT STATE VECTOR IS UPDATED WHENEVER
# STATEFLG WAS SET AND WHENEVER A W-MATRIX IS TO BE INTEGRATED.  THE PUSH
# COUNTER IS SET TO 0 AND OVERFLOW IS CLEARED BEFORE RETURNING TO THE
# CALLING PROGRAM.
#
# 4.0 CALLING SEQUENCES AND SAMPLE CODE
# -------------------------------------
#
#	A) PRECISION ORBITAL INTEGRATION.  CSMPREC, LEMPREC ENTRANCES
#		L-X	STORE TIME TO 96T5791T5 T 95 PUS L9ST (T4531)
#		L	CALL
#		L+1		CSMPREC (OR LEMPREC)
#		L+2	RETURN
#	   INPUT							   28
#		TDEC1 (PD 32D) TIME TO INTEGRATE TO...CENTISECONDS SCALED 2
#	   OUTPUT
#		THE DATA LISTED IN SECTION 3.0 PLUS
#		RQVV	POSITION VECTOR OF VEHICLE WITH RESPECT TO SECONDARY
#		BODY... METERS B-29 ONLY IF MIDFLAG = DIM0FLAG = 1
#	B) CONIC INTEGRATION.  CSMCONIC, LEMCONIC ENTRANCES
#		L-X	STORE TIME IN PUSH LIST (TDEC1)
#		L	CALL
#		L+1		CSMCONIC (OR LEMCONIC)
#	   INPUT/OUTPUT
#		SAME AS PRECISION INTEGRATION, EXCEPT RQVV NOT SET
#	C) INTEGRATE GIVEN STATE VECTOR.  INTEGRVS ENTRANCE
#		CALL
#				INTSTALL
#		VLOAD
#				POSITION VECTOR
#		STOVL		RCV
#				VELOCITY VECTOR
#		STODL		VCV
#				TIME STATE VECTOR VALID
#		STODL		TET
#				FINAL RADIUS
#		STORE		RFINAL
#		SET(CLEAR)	SET(CLEAR)
#				INTYPFLAG
#				MOONFLAG
#		SET(CLEAR)	DLOAD
#				DESIRED TIME
#		STCALL		TDEC1
#				INTEGRVS
#	  INPUT
#		RCV	POSITION VECTOR			METERS
#		VCV	VELOCITY VECTOR			M/CSEC
#		TET	TIME OF STATE VECTOR (MAY = 0)	CSEC B-28
# Page 1209
#		TDEC1	TIME TO INTEGRATE TO		CSEC B-28 (PD 32D)
#			(MAY BE INCREMENT IF TET=0)
#	  OUTPUT
#		SAME AS FOR PRECISION OR CONIC INTEGRATION,
#		DEPENDING ON INTYPFLG.
#	D) INTEGRATE STATE VECTOR.  INTGRV ENTRANCE
#		L-X	STORE TIME IN PUSH LIST (TDEC1) (MAY BE DONE AFTER CALL TO INTSTALL)
#		L-8	CALL
#		L-7
#		L-6	SET(CLEAR)	SET(CLEAR)
#		L-5			VINTFLAG	1=CSM, 0=LM
#		L-4			INTYPFLAG	1=CONIC, 0=PRECISION
#		L-3	SET(CLEAR)	SET(CLEAR)
#		L-2			DIM0FLAG	1=W-MATRIX, 0=NO W-MATRIX
#		L-1			D6OR9FLG	1=9X9, 0=6X6
#		L	SET		DLOAD
#		L+1			STATEFLG	DESIRE PERMANENT UPDATE
#		L+2			FINAL RAD.	OF STATE VECTOR
#		L+3	STCALL		RFINAL
#		L+4			INTEGRV
#		L	CALL				NORMAL USE -- WILL UPDATE STATE
#		L+1			INTEGRV		VECTOR IF DIM0FLAG=1. (STATEFLG IS
#		L+2	RETURN				ALWAYS RESET IN INTEGRATION AFTER
#							IT USED.)
#	  INPUT
#		TDEC1 (PD 32D) TIME TO INTEGRATE TO 	CSEC B-28
#	  OUTPUT
#		SAME AS FOR PRECISION OR CONIC INTEGRATION
#	  THE PROGRAM WILL SET MOONFLAG, MIDFLAG DEPENDING ON
#	  THE PERMANENT STATE VECTOR REPRESENTATION.

		BANK	11
		SETLOC	INTINIT
		BANK
		EBANK=	RRECTCSM
		COUNT*	$$/INTIN
STATEINT	TC	PHASCHNG
		OCT	00052
		CAF	PRIO5
		TC	FINDVAC
		EBANK=	RRECTCSM
		2CADR	STATINT1

		TC	TASKOVER
STATINT1	TC	INTPRET
		BON	RTB
			QUITFLAG	# KILL INTEGRATION UNTIL NEXT P00.
			NOINT
			LOADTIME
		STORE	TDEC1
# Page 1210
		CALL
			INTSTALL
		SET	CALL
			NODOFLAG
			SETIFLGS
		GOTO
			STATEUP
600SECS		2DEC	60000

ENDINT		CLEAR	EXIT
			STATEFLG
		TC	PHASCHNG
		OCT	20032
		EXTEND
		DCA	600SECS
		TC	LONGCALL
		EBANK=	RRECTHIS
		2CADR	STATEINT

		TC	ENDOFJOB
SETIFLGS	SET	CLEAR
			STATEFLG
			INTYPFLG
		CLEAR	CLEAR
			DIM0FLAG
			D6OR9FLG
		RVQ
NOINT		EXIT
		TC	PHASCHNG
		OCT	00002

		TC	DOWNFLAG
		ADRES	QUITFLAG
		TC	ENDOFJOB

# ATOPCSM TRANSFERS RRECT TO RRECT +41 TO RRECTCSM TO RRECTCSM +41
#
# CALLING SEQUENCE
#	L	CALL
#	L+1		ATOPCSM
#
# NORMAL EXIT AT L+2

ATOPCSM		STQ	RTB
			S2
			MOVEACSM
		SET	CALL
			CMOONFLG
			SVDWN1
		BON	CLRGO
# Page 1211
			MOONFLAG
			S2
			CMOONFLG
			S2
MOVEACSM	TC	SETBANK
		TS	DIFEQCNT	# INITIALIZE INDEX
		INDEX	DIFEQCNT
		CA	RRECT
		INDEX	DIFEQCNT
		TS	RRECTCSM
		CCS	DIFEQCNT	# IS TRANSFER COMPLETE
		TCF	MOVEACSM +1	# NO-LOOP
		TC	DANZIG		# COMPLETE -- RETURN

# PTOACSM TRANSFERS RRECTCSM TO RRECTCSM +41 TO RRECT TO RRECT +41
#
# CALLING SEQUENCE
#	L	CALL
#			PTOACSM
#
# NORMAL EXIT AT L+2

PTOACSM		RTB	BON
			MOVEPCSM
			CMOONFLG
			SETMOON
CLRMOON		CLEAR	SSP
			MOONFLAG
			PBODY
			0
		RVQ
SETMOON		SET	SSP
			MOONFLAG
			PBODY
			2
		RVQ
MOVEPCSM	TC	SETBANK
		TS	DIFEQCNT
		INDEX	DIFEQCNT
		CA	RRECTCSM
		INDEX	DIFEQCNT
		TS	RRECT
		CCS	DIFEQCNT
		TCF	MOVEPCSM +1
		TC	DANZIG

# ATOPLEM TRANSFERS RRECT TO RRECT +41 TO RRECTLEM TO RRECTLEM +41
ATOPLEM		STQ	RTB
# Page 1212
			S2
			MOVEALEM
		SET	CALL
			LMOONFLG
			SVDWN2
		BON	CLRGO
			MOONFLAG
			S2
			LMOONFLG
			S2
MOVEALEM	TC	SETBANK
		TS	DIFEQCNT
		INDEX	DIFEQCNT
		CA	RRECT
		INDEX	DIFEQCNT
		TS	RRECTLEM
		CCS	DIFEQCNT
		TCF	MOVEALEM +1
		TC	DANZIG

# PTOALEM TRANSFERS RRECTLEM TO RRECTLEM +41 TO RRECT TO RRECT +41

PTOALEM		BON	RTB
			SURFFLAG
			USEPIOS
			MOVEPLEM
		BON	GOTO
			LMOONFLG
			SETMOON
			CLRMOON
MOVEPLEM	TC	SETBANK
		TS	DIFEQCNT
		INDEX	DIFEQCNT
		CA	RRECTLEM
		INDEX	DIFEQCNT
		TS	RRECT
		CCS	DIFEQCNT
		TCF	MOVEPLEM +1
		TC	DANZIG

USEPIOS		SETPD	VLOAD
			0
			RLS
		PDDL	PUSH
			TDEC1
		STODL	TET
			5/8
		CALL
# Page 1213
			RP-TO-R
		STOVL	RCV
			ZUNIT
		STODL	0D
			TET
		STODL	6D
			5/8
		SET	CALL		# NEEDED FOR SETTING X1 ON EXIT
			MOONFLAG
			RP-TO-R
		VXV	VXSC
			RCV
			OMEGMOON
		STOVL	VCV
			ZEROVEC
		STORE	TDELTAV
		AXT,2	SXA,2
			2
			PBODY
		STCALL	TNUV
			A-PCHK
SETBANK		CAF	INTBANK
		TS	BBANK
		CAF	FORTYONE
		TC	Q
		EBANK=	RRECTCSM
INTBANK		BBCON	INTEGRV

; ============================================================================
; SPECIAL PURPOSE ENTRY POINTS TO ORBITAL INTEGRATION
;
; These four routines provide convenient entry points for different types
; of orbit integration. Each sets appropriate flags before calling the main
; integration logic.
;
; COMMENT-ONLY READERS: The computer can track spacecraft position using
; either high-precision calculations (PREC) that account for the Moon's
; irregular shape, or faster approximate calculations (CONIC) that assume
; perfect spheres. These entry points select the appropriate method.
;
; CODE-ALONG READERS: Study how flag combinations control integration method
; (Encke vs Kepler), vehicle selection (CSM vs LM), and computation mode.
; ============================================================================

# SPECIAL PURPOSE ENTRIES TO ORBITAL INTEGRATION.  THESE ROUTINES PROVIDE ENTRANCES TO INTEGRATION WITH
# APPROPRIATE SWITCHES SET OR CLEARED FOR THE DESIRED INTEGRATION.
#
# CSMPREC AND LEMPREC PERFORM ORBIT INTEGRATION BY THE ENCKE METHOD TO THE TIME INDICATED IN TDEC1.
# ACCELERATIONS DUE TO OBLATENESS ARE INCLUDED.  NO W-MATRIX INT. IS DONE.
# THE PERMANENT STATE VECTOR IS NOT UPDATED.
#
# CSMCONIC AND LEMCONIC PERFORM ORBIT INTEG. BY KEPLER'S METHOD TO THE TIME INDICATED IN TDEC1.
# NO DISTURBING ACCELERATIONS ARE INCLUDED.  IN THE PROGRAM FLOW THE GIVEN
# STATE VECTOR IS RECTIFIED BEFORE SOLUTION OF KEPLER'S EQUATION.
#
# THE ROUTINES ASSUME THAT THE CSM (LEM) STATE VECTOR IN P-MEM IS VALID.
# SWITCHES SET PRIOR TO ENTRY TO THE MAIN INTEG. PROG ARE AS FOLLOWS:
#			CSMPREC		CSMCONIC	LEMPREC		LEMCONIC
#	VINTFLAG	SET		SET		CLEAR		CLEAR
#	INTYPFLG	CLEAR		SET		CLEAR		SET
#	DIM0FLAG	CLEAR		CLEAR		CLEAR		CLEAR
#
# CALLING SEQUENCE
#	L-X	STORE	TDEC1
#	L	CALL			(STCALL TDEC1)
# Page 1214
#	L+1		CSMPREC		(CSMCONIC, LEMPREC, LEMCONIC)
#
# NORMAL EXIT TO L+2
#
# SUBROUTINES CALLED
#	INTEGRV1
#	PRECOUT FOR CSMPREC AND LEMPREC
#	CONICOUT FOR CSMCONIC AND LEMCONIC
#
# OUTPUT -- SEE PAGE 2 OF THIS LOG SECTION
#
# INPUT
#	TDEC1		TIME TO INTEGRATE TO.  CSECS B-28

; CSMPREC - Command/Service Module Precision Integration Entry Point
; Performs high-accuracy Encke method integration of CSM state vector to
; time TDEC1, including oblateness perturbations from Moon's irregular shape.
; Used during translunar and transearth coast for navigation updates.
; Register X1 saved to IRETURN for return linkage.

CSMPREC		STQ	CALL
			X1
			INTSTALL
		SXA,1	SET
			IRETURN
			VINTFLAG

; Flag configuration for precision integration: PRECIFLG set indicates Encke
; method, DIM0FLAG clear for 3-dimensional integration, INTYPFLG clear for
; non-conic (precision) mode. Proceeds to main integration logic INTEGRV1.

IFLAGP		SET	CLEAR
			PRECIFLG
			DIM0FLAG
		CLRGO
			INTYPFLG
			INTEGRV1

; LEMPREC - Lunar Module Precision Integration Entry Point
; Performs Encke method integration of LM state vector with oblateness
; perturbations. Critical during descent orbit, powered descent preparation,
; and ascent rendezvous navigation. VINTFLAG cleared for LM (vs CSM).

LEMPREC		STQ	CALL
			X1
			INTSTALL
		SXA,1	CLRGO
			IRETURN
			VINTFLAG
			IFLAGP

; CSMCONIC - Command/Service Module Conic Integration Entry Point
; Fast Keplerian (two-body) integration assuming spherical gravity field.
; Used for quick trajectory predictions where oblateness effects negligible.
; Faster than CSMPREC but less accurate - acceptable for preliminary planning.

CSMCONIC	STQ	CALL
			X1
			INTSTALL
		SXA,1	SET
			IRETURN
			VINTFLAG

; Flag configuration for conic integration: DIM0FLAG clear, INTYPFLG set
; indicates Kepler's method (conic sections). State vector rectified before
; solving Kepler's equation for fast two-body orbit propagation.

IFLAGC		CLEAR	SETGO
			DIM0FLAG
			INTYPFLG
			INTEGRV1

; LEMCONIC - Lunar Module Conic Integration Entry Point
; Fast Keplerian integration of LM state vector. Used for quick orbit checks
; and trajectory planning when high precision unnecessary. During Apollo 11
; rendezvous, provided rapid what-if calculations for crew and ground.

LEMCONIC	STQ	CALL
			X1
			INTSTALL
		SXA,1	CLRGO
			IRETURN
# Page 1215
			VINTFLAG
			IFLAGC

; ============================================================================
; INTEGRVS - General Purpose Integration with Supplied State Vector
;
; COMMENT-ONLY READERS: Programs can integrate any position and velocity
; (not just permanent CSM/LM vectors) by providing their own state vector.
; Used for "what-if" trajectory calculations and mission planning.
;
; CODE-ALONG READERS: Caller supplies state vector and sets integration type
; flags. PBODY index determines gravitational body (0=Earth, 2=Moon) based
; on MOONFLAG. State vector stored and rectified before integration begins.
; ============================================================================

INTEGRVS	SET	SSP
			PRECIFLG
			PBODY
			0
		BOF	SSP
			MOONFLAG
			+3
			PBODY
			2
		STQ	VLOAD
			IRETURN
			ZEROVEC
		STORE	TDELTAV
		STCALL	TNUV
			RECTIFY
		CLEAR	SET
			DIM0FLAG
			NEWIFLG
		SETGO
			RPQFLAG
			ALOADED

; ============================================================================
; INTEGRV - Navigation Program Integration Entry Point
;
; COMMENT-ONLY READERS: This is the primary entry used by P20-P25 navigation
; programs to integrate spacecraft orbits. The calling program sets flags
; to control whether precision or conic integration is used, which vehicle
; to integrate, and whether to update the permanent state vector.
;
; CODE-ALONG READERS: Multiple entry points (INTEGRV, INTEGRV1, INTEGRV2)
; serve different callers. INTEGRV for navigation programs, INTEGRV1 for
; CSMPREC/LEMPREC, ALOADED from INTEGRVS. Study flag testing logic and
; A-memory setup path selection via VINTFLAG (CSM vs LM).
; ============================================================================

# INTEGRV IS AN ENTRY TO ORBIT INTEGRATION WHICH PERMITS THE CALLER,
# NORMALLY THE NAVIGATION PROGRAM, TO SET THE INTEG. FLAGS.  THE ROUTINE
# IS ENTERED AT INTEGRV1 BY CSMPREC ET AL. AND AT ALOADED BY INTEGRVS.
# THE ROUTINE SETS UP A-MEMORY IF ENTERED AT INTEGRV,1 AND SETS THE INTEG.
# PROGRAM FOR PRECISION OR CONIC.
#
# THE CALLER MUST FIRST CALL INTSTALL TO CHECK IF INTEG. IS IN USE BEFORE
# SETTING ANY FLAGS.
#
# THE FLAGS WHICH SHOULD BE SET OR CLEARED ARE
#	VINTFLAG	(IGNORED WHEN ENTERED FROM INTEGRVS)
#	INTYPFLG
#	DIM0FLAG
#	D6OR9FLG
#
# CALLING SEQUENCE
#	L-X	CALL
#	L-Y		INTSTALL
#	L-1	SET OR CLEAR ALL FOUR FLAGS.  ALSO CAN SET STATEFLG IF DESIRED
#		AND DIM0FLAG IS CLEAR.
#	L	CALL
#	L+1		INTEGRV
#
# INITIALIZATION
#	FLAGS AS ABOVE
#	STORE TIME TO INTEGRATE TO IN TDEC1
#
# OUTPUT
#	RATT	AS
#	VATT	      DEFINED
# Page 1216
#	TAT			BEFORE

INTEGRV		STQ
			IRETURN
INTEGRV1	SET	SET
			RPQFLAG
			NEWIFLG
INTEGRV2	SSP
			QPRET
			ALOADED
		BON	GOTO
			VINTFLAG
			PTOACSM
			PTOALEM

; ALOADED - Common integration path after state vector loaded into A-memory
; Loads desired integration time (TDEC1) and branches to precision (TESTLOOP)
; or conic (RVCON) integration based on INTYPFLG.

ALOADED		DLOAD
			TDEC1
		STORE	TDEC
		BOFF	GOTO
			INTYPFLG
			TESTLOOP
			RVCON
A-PCHK		BOF	EXIT
			STATEFLG
			RECTOUT
		TC	PHASCHNG
		OCT	04022
		TC	UPFLAG		# PHASE CHANGE HAS OCCURRED BETWEEN
		ADRES	REINTFLG	# INTSTALL AND INTWAKE
		TC	INTPRET
		SSP
			QPRET
			PHEXIT
		BON	GOTO
			VINTFLAG
			ATOPCSM
			ATOPLEM
PHEXIT		CALL
			GRP2PC
RECTOUT		SETPD	CALL
			0
			RECTIFY
		VLOAD	VSL*
			RRECT
			0,2
		PDVL	VSL*		# RATT TO PD0
			VRECT
			0,2
		PDDL	PDVL		# VATT TO PD6	TAT TO PD12
			TET
# Page 1217
			RRECT
		PDVL	PDDL*
			VRECT
			MUEARTH,2
		PUSH	AXT,1
		DEC	-10
		BON	AXT,1
			MOONFLAG
			+2
		DEC	-2

; ============================================================================
; INTEXIT - Integration Exit and Cleanup Routine
;
; COMMENT-ONLY READERS: When orbit integration completes (reaching desired time
; or final radius), the computer performs cleanup: updating state vectors,
; clearing operational flags, and preparing for the next navigation task. The
; results are now available for guidance decisions or display to the crew.
;
; CODE-ALONG READERS: Common exit point from integration (precision or conic).
; Reached when QUITFLAG set (time/radius reached) or error conditions detected.
; SETPD 0 resets pushdown stack. BOV +1 handles overflow (skips next instruction).
; Clears AVEMIDSW (allows downlink state vector update), PRECIFLG, and STATEFLG.
; Loads IRETURN address from interpretive stack, exits to basic mode, stores
; return address in QPRET (indexed by FIXLOC for CSM/LM distinction), then
; calls INTWAKE to handle restart logic and release integration stall area.
; Output state vector available in pushlist per calling sequence specification.
; ============================================================================

INTEXIT		SETPD	BOV
			0
			+1
		CLEAR	CLEAR
			AVEMIDSW	# ALLOW UPDATE OF DOWNLINK STATE VECTOR
			PRECIFLG
		CLEAR
			STATEFLG
		SLOAD	EXIT
			IRETURN
		CA	MPAC
		INDEX	FIXLOC
		TS	QPRET
		TC	INTWAKE

; ============================================================================
; RVCON - Conic (Keplerian) Integration Path
;
; COMMENT-ONLY READERS: When high precision is unnecessary, the computer
; uses Kepler's laws to quickly calculate orbital position. This assumes
; perfect spherical gravity with no perturbations from Moon's irregular shape.
; Much faster than precision integration.
;
; CODE-ALONG READERS: Computes time interval TAU = (TDEC - TET), rectifies
; state vector, calls KEPPREP to solve Kepler's equation, updates TET, then
; proceeds to RECTOUT for output formatting. See CONIC_SUBROUTINES.agc for
; Kepler solver details.
; ============================================================================

# RVCON SETS UP ORBIT INTEGRATION TO DO A CONIC SOLUTION FOR POSITION AND
# VELOCITY FOR THE INTERVAL (TET-TDEC)

RVCON		DLOAD	DSU
			TDEC
			TET
		STCALL	TAU.
			RECTIFY
		CALL
			KEPPREP
		DLOAD	DAD
			TC
			TET
		STCALL	TET
			RECTOUT

# Page 1218

; ============================================================================
; TESTLOOP - Precision (Encke Method) Integration Main Loop
;
; COMMENT-ONLY READERS: For accurate trajectories, the computer uses advanced
; Encke method integration accounting for Moon's irregular gravity and other
; perturbing forces. This loop repeatedly advances the orbit in small time
; steps (DT/2), checking for completion or need to update the reference orbit.
;
; CODE-ALONG READERS: Main iteration loop for precision integration. Tests
; QUITFLAG for termination, computes adaptive timestep DT/2 based on orbital
; radius and gravitational parameter, checks RFINAL radius termination
; condition. Each iteration calls POOHCHK which performs actual Encke step.
; See ORBITAL_INTEGRATION.agc for Encke algorithm implementation.
; ============================================================================

TESTLOOP	BOF	CLRGO
			QUITFLAG
			+3
			STATEFLG
			INTEXIT		# STOP INTEGRATION
 +3		SETPD	LXA,2
			10D
			PBODY
		VLOAD	ABVAL
			RCV
		PUSH	CLEAR		# RC TO 10D
			MIDFLAG
		DSU*	BMN		# MIDFLAG=0 IF R G.T. RMP
			RME,2
			+3
		SET
			MIDFLAG
NORFINAL	DLOAD	DMP
			10D
			34D
		SR1R	DDV*
			MUEARTH,2
		SQRT	DMP
			.3D
		SR3	SR4		# DT IS TRUNCATED TO A MULTIPLE
		DLOAD	SL
			MPAC
			15D		#	OF 128 CSECS.
		PUSH	BOV
			MAXDT
		BDSU	BMN
			DT/2MAX
			MAXDT
DT/2COMP	DLOAD	DSU
			TDEC
			TET
		RTB	SL
			SGNAGREE
			8D
		STORE	DT/2		# B-19
		BOV	ABS
			GETMAXDT
		DSU	BMN		# IS TIME TO INTEG. TO GR THAN MAXTIME
			12D
			POOHCHK
USEMAXDT	DLOAD	SIGN
			12D
			DT/2
# Page 1219
		STCALL	DT/2
			POOHCHK
MAXDT		DLOAD	PDDL		# EXCHANGE DT/2MAX WITH COMPUTED MAX.
			DT/2MAX
		GOTO
			DT/2COMP
GETMAXDT	RTB
			SIGNMPAC
		STCALL	DT/2
			USEMAXDT
POOHCHK		DLOAD	ABS
			DT/2
		DSU	BMN
			DT/2MIN
			A-PCHK
		SLOAD	BHIZ
			MODREG
			+3
		GOTO
			TIMESTEP
		BON			# WAS THIS CALL VIA CSM(LEM)PREC
			PRECIFLG
			TIMESTEP	# YES
		DLOAD	DSU
			DT/2
			12D
		BMN	BOFCLR
			A-PCHK
			NEWIFLG
			TIMESTEP
		DLOAD	DSU
			TDEC
			TET
		BMN			# NO BACKWARD INTEGRATION
			INTEXIT
		PDDL	SR4
			DT/2		# IS 4(DT) LS (TDEC - TET)
		SR2R	BDSU
		BMN	GOTO
			INTEXIT
			TIMESTEP
DT/2MIN		2DEC	3 B-20

DT/2MAX		2DEC	4000 E2 B-20

; ============================================================================
; INTSTALL - Integration Stall/Synchronization Routine
;
; COMMENT-ONLY READERS: To prevent multiple integration calculations from
; interfering with each other in shared memory, the computer includes a
; "stall" mechanism. When a program wants to integrate, it must first check
; if the integration area is available. If busy, the program waits its turn.
; This ensures accurate trajectory calculations during critical mission phases.
;
; CODE-ALONG READERS: Entry point for integration synchronization using RASFLAG
; as semaphore. Called via TC INTSTALL from interpretive mode. EXIT instruction
; (line 910) switches to basic mode. Tests INTBITAB bits (octal 20100) in
; RASFLAG to check if integration scratch area is available. If free (BZF),
; proceeds to OKTOGRAB which sets INTFLBIT and returns to interpretive mode.
; If busy, calls JOBSLEEP with WAKESTAL address, putting job on waitlist until
; integration area freed. INTWAKE/INTWAKE0/INTWAKE1 handle restart recovery,
; preserving QPRET return address across restarts. Critical for P20-P25
; navigation programs that may have multiple concurrent integration requests.
; ============================================================================

INTSTALL	EXIT
		CA	RASFLAG
		MASK	INTBITAB	# IS THIS STALL AREA FREE
		EXTEND
		BZF	OKTOGRAB	# YES
# Page 1220
		CAF	WAKESTAL
		TC	JOBSLEEP
INTWAKE0	EXIT
		TCF	INTWAKE1

INTWAKE		CS	RASFLAG		# IS THIS INTSTALLED ROUTINE TO BE
		MASK	REINTBIT	#	RESTARTED
		CCS	A
		TC	INTWAKE1	# NO

		INDEX	FIXLOC
		CA	QPRET
		TS	TBASE2		# YES, DON'T RESTART WITH SOMEONE ELSE'S Q

		TC	PHASCHNG
		OCT	04022

		CA	TBASE2
		INDEX	FIXLOC
		TS	QPRET

		CAF	REINTBIT
		MASK	RASFLAG
		EXTEND
		BZF	GOBAC		# DON'T INTWAKE IF WE CAME HERE VIA RESTART

INTWAKE1	CAF	WAKESTAL
		INHINT
		TC	JOBWAKE
		CCS	LOCCTR
		TCF	INTWAKE1
FORTYONE	DEC	41
		CS	INTBITAB
		MASK	RASFLAG
		TS	RASFLAG		# RELEASE STALL AREA
		RELINT
		TCF	GOBAC
OKTOGRAB	CAF	INTFLBIT
		INHINT
		ADS	RASFLAG
GOBAC		TC	INTPRET
		RVQ
WAKESTAL	CADR	INTSTALL +1
INTBITAB	OCT	20100

# Page 1221

; ============================================================================
; TRANSITION: From Integration Synchronization to State Vector Management
;
; With synchronization mechanisms established, attention shifts to state
; vector transitions between powered and coast flight phases. During Apollo 11's
; mission, these routines managed critical handoffs: after descent engine burns,
; after RCS maneuvers, and during trajectory updates. AVETOMID transfers
; average-g computed positions to the integration state vector, while MIDTOAV1
; and MIDTOAV2 extrapolate states forward for guidance and navigation use.
; ============================================================================

# AVETOMID
#
# THIS ROUTINE PERFORMS THE TRANSITION FROM A THRUSTING PHASE TO THE COAST
# PHASE BY INITIALIZING THIS VEHICLE'S PERMANENT STATE VECTOR WITH THE
# VALUES LEFT BY THE AVERAGEG ROUTINE IN RN,VN,PIPTIME.
#
# BEFORE THIS IS DONE THE W-MATRIX, IF ITS VALID (OR WFLAG OR RENDWFLT IS
# SET) IS INTEGRATED FORWARD TO PIPTIME WITH THE PRE-THRUST STATE VECTOR.
#
# IN ADDITION, THE OTHER VEHICLE IS INTEGRATED (PERMANENT) TO PIPTIME.
#
# FINALLY TRKMKCNT IS ZEROED.

		SETLOC	INTINIT
		BANK

		COUNT*	$$/INTIN
AVETOMID	STQ	BON
			EGRESS
			RENDWFLG
			INT/W		# W-MATRIX VALID, GO INTEGRATE IT
		BON
			ORBWFLAG
			INT/W		# W-MATRIX VALID, GO INTEGRATE IT.

OTHERS		DLOAD	CALL		# GET SET FOR OTHER VEHICLE INTEGRATION
			PIPTIME		# DESIRED TIME
			INTSTALL
		SET	CALL
			VINTFLAG	# CM
			SETIFLGS	# SETS UP NONE W-MAT. PERMANENT INTEG.
		STCALL	TDEC1
			INTEGRV

		AXT,2	CALL		# NOW MOVE PROPERLY SCALE RN,UN AS WELL AS
			2		# PIPTIME TO INTEGRATION ERASABLES.
			INTSTALL
		BON	AXT,2
			MOONTHIS
			+2
			0
		VLOAD	VSR*
			RN
			0,2
		STORE	RRECT
		STODL	RCV
			PIPTIME
		STOVL	TET
			VN
# Page 1222
		VSR*	CALL
			0,2
			MINIRECT	# FINISH SETTING UP STATE VECTOR
		RTB	SSP
			MOVATHIS	# PUT TEMP STATE VECTOR INTO PERMANENT
			TRKMKCNT
			0
		GOTO
			FAZAB5

INT/W		DLOAD	CALL
			PIPTIME		# INTEGRATE W THRU BURN
			INTSTALL
		SET	SET
			DIM0FLAG	# DO W-MATRIX
			AVEMIDSW	# SO WON'T CLOBBER RN,VN,PIPTIME
		SET	CLEAR
			D6OR9FLG	# 9X9 FOR LM
			VINTFLAG	# LM
		STCALL	TDEC1
			INTEGRV
		GOTO
			OTHERS		# NOW GO DO THE OTHER VEHICLE

; ============================================================================
; TRANSITION: From W-Matrix Integration to State Vector Extrapolation
;
; After computing updated navigation states, the AGC must extrapolate position
; and velocity forward to support real-time guidance decisions. MIDTOAV1
; integrates to a specified future time (TDEC1), while MIDTOAV2 integrates to
; the current time plus a small delta. During Apollo 11's descent, these
; routines provided Armstrong and Aldrin with continuously updated position
; estimates, enabling the crew to monitor landing site approach and make the
; critical decision to divert to a boulder-free touchdown zone.
; ============================================================================

# Page 1223
# MIDTOAV1
#
# THIS ROUTINE INTEGRATES (PRECISION) TO THE TIME SPECIFIED IN TDEC1.
# IF, AT THE END OF AN INTEGRATION TIME STEP, CURRENT TIME PLUS A DELTA
# TIME (SEE TIMEDELT.....BASED ON THE COMPUTATION TIME FOR ONE TIME STEP)
# IS GREATER THAN THE DESIRED TIME, ALARM 1703 IS SET AND THE INTEGRATION
# IS DONE TO THE CURRENT TIME.
# RETURN IS IN BASIC TO THE RETURN ADDRESS PLUS ONE.
#
# IF THE INTEGRATION IS FINISHED TO THE DESIRED TIME, RETURN IS IN BASIC
# TO THE RETURN ADDRESS.
#
# IN EITHER CASE, BEFORE RETURNING, THE EXTRAPOLATED STATE VECTOR IS TRANSFERRED
# FROM R,VATT TO R,VN1-PIPTIME1 IS SET TO THE FINISHING INTEGRATION
# TIME AND MPAC IS SET TO THE DELTA TIME --
#			TAT MINUS CURRENT TIME

# MIDTOAV2
#
# THIS ROUTINE INTEGRATES THIS VEHICLE'S STATE VECTOR TO THE CURRENT TIME.
# NO INPUTS ARE REQUIRED OF THE CALLER.  RETURN IS IN BASIC TO THE RETURN
# ADDRESS WITH THE ABOVE TRANSFERS TO R,VN1-PIPTIME1-AND MPAC DONE

		EBANK=	IRETURN1
MIDTOAV2	STQ	CLRGO		# INTEGRATE TO PRESENT TIME PLUS TIMEDELT
			IRETURN1
			MID1FLAG
			ENTMID2

MIDTOAV1	STQ	SET		# INTEGRATE TO TDEC1
			IRETURN1
			MID1FLAG
		RTB	DAD		# INITIAL CHECK, IS TDEC1 IN THE FUTURE
			LOADTIME
			TIMEDELT
		BDSU	BPL
			TDEC1
			ENTMID1		# Y5S
		CALL
			NOTIME		# NO, SET ALARM, SWITCH TO MIDTOAV2

ENTMID2		RTB	DAD
			LOADTIME
			TIMEDELT
		STORE	TDEC1

ENTMID1		CALL
			INTSTALL
		CLEAR	CALL
# Page 1224
			DIM0FLAG	# NO W-MATRIX
			THISVINT
		CLEAR	SET
			INTYPFLG
			MIDAVFLG	# LET INTEG. KNOW THE CALL IS FOR MIDTOAV.
		CALL
			INTEGRV		# GO INTEGRATE
		CLEAR	VLOAD
			MIDAVFLG
			RATT
		STOVL	RN1
			VATT
		STODL	VN1
			TAT
		STORE	PIPTIME1
		SXA,2	SXA,1
			RTX2
			RTX1
		EXIT

		INHINT
		EXTEND
		DCS	TIME2
		DAS	MPAC
		TC	TPAGREE

		CA	IRETURN1
		TC	BANKJUMP
CKMID2		BOF	RTB
			MID1FLAG
			MID2
			LOADTIME
		DAD	BDSU
			TIMEDELT
			TDEC
		BPL	CALL
			TESTLOOP	# YES
			NOTIME

TIMEINC		RTB	DAD
			LOADTIME
			TIMEDELT
		STCALL	TDEC
			TESTLOOP

MID2		DLOAD	DSU
			TDEC
			TET
		ABS	DSU
			3CSECS

# Page 1225
		BMN	GOTO
			A-PCHK
			TIMEINC

NOTIME		CLEAR	EXIT		# TOO LATE
			MID1FLAG
		INCR	IRETURN1	# SET ERROR EXIT (CALLOC +2)
		TC	ALARM		# INSUFFICIENT TIME FOR INTEGRATION --
		OCT	1703		#	TIG WILL BE SLIPPED...
		TC	INTPRET
		RVQ

3CSECS		2DEC	3

TIMEDELT	2DEC	2000

		BANK	27
		SETLOC	UPDATE2
		BANK
		EBANK=	INTWAKUQ

		COUNT*	$$/INTIN

INTWAKUQ	=	INTWAK1Q	# TEMPORARY UNTIL NAME OF INTWAK1Q IS CHNG

; ============================================================================
; TRANSITION: From Time Management to State Vector Update Processing
;
; Having computed forward-integrated trajectories, the AGC must now commit
; these calculations to permanent memory. INTWAKEU handles state vector update
; requests from navigation programs, moving computed positions and velocities
; (RRECT/VRECT) into the reference coordinate vectors (RCV/VCV). During Apollo
; 11's mission, these updates followed optical navigation marks, radar tracking
; measurements, and ground-computed state vectors uplinked from Mission Control.
; The routine distinguishes between Earth and lunar sphere updates, and between
; CSM and LM vehicle states, ensuring the correct coordinate transformations
; and memory locations are used for each update scenario.
; ============================================================================

; State Vector Update Wake-Up Routine
; Called when integration completes or when navigation programs request
; permanent state vector updates. During Apollo 11, this executed after
; optical navigation marks provided position fixes, after radar tracking
; refined relative positions, and when Mission Control uplinked updated
; state vectors based on ground tracking data.

INTWAKEU	RELINT
		EXTEND
		QXCH	INTWAKUQ	# SAVE Q FOR RETURN

		TC	INTPRET

; Check if this wake-up is for a state vector update request.
; UPSVFLAG values: 0 = no update, +1 = CSM Earth, +2 = CSM Moon,
;                  -1 = LM Earth, -2 = LM Moon
; If zero, skip to standard wake-up processing (INTWAKUP).

		SLOAD	BZE		# IS THIS A CSM/LEM STATE VECTOR UPDATE
			UPSVFLAG	# REQUEST.  IF NOT GO TO INTWAKUP.
			INTWAKUP

; Transfer computed rectangular coordinates to reference coordinate system.
; RRECT and VRECT contain the integrated position and velocity vectors.
; RCV and VCV are the permanent reference coordinate vectors used throughout
; navigation computations. This transfer commits the integration results to
; the primary navigation state.

		VLOAD			# MOVE PRECT(6) AND VRECT(6) INTO
			RRECT		#	RCV(6) AND VCV(6) RESPECTIVELY.
		STOVL	RCV
			VRECT		# NOW GO TO `RECTIFY +13D' TO
		CALL			# STORE VRECT INTO VCV AND ZERO OUT
			RECTIFY +13D	# TDELTAV(6),TNUV(6),TC(2), AND XKEP(2)
; Determine gravitational sphere of influence (Earth vs. Moon).
; The AGC must know which central body dominates gravity to select proper
; coordinate transformations and perturbation models. During Apollo 11's
; translunar coast, the spacecraft crossed the lunar sphere boundary at
; approximately 61 hours mission elapsed time, requiring coordinate system
; transformation from Earth-centered to Moon-centered reference frames.
; X2 register is set to 0 for Earth sphere, 2 for lunar sphere.

		SLOAD	ABS		# COMPARE ABSOLUTE VALUE OF `UPSVFLAG'
			UPSVFLAG	# TO `UPDATE MOON STATE VECTOR CODE'
		DSU	BZE		# TO DETERMINE WHETHER THE STATE VECTOR TO
			UPMNSVCD	# BE UPDATED IS IN THE EARTH OR LUNAR
			INTWAKEM	# SPHERE OF INFLUENCE........
		AXT,2	CLRGO		# EARTH SPHERE OF INFLUENCE.
		DEC	0
			MOONFLAG
# Page 1226
			INTWAKEC
INTWAKEM	AXT,2	SET		# LUNAR SPHERE OF INFLUENCE.
		DEC	2
			MOONFLAG
; Determine which vehicle's state vector to update: CSM or LM.
; Sign of UPSVFLAG indicates vehicle: positive = CSM, negative = LM.
; During Apollo 11's mission, both vehicles maintained independent state
; vectors while docked, then diverged after LM separation for descent.
; Armstrong and Aldrin in Eagle tracked separate from Collins in Columbia.

INTWAKEC	SLOAD	BMN		# COMMON CODING AFTER X2 INITIALIZED AND
					# MOONFLAG SET (OR CLEARED).
			UPSVFLAG	# IS THIS A REQUEST FOR A LEM OR CSM
			INTWAKLM	# 	STATE VECTOR UPDATE......
		CALL			# UPDATE CSM STATE VECTOR
			ATOPCSM

		CLEAR	GOTO
			ORBWFLAG
			INTWAKEX

INTWAKLM	CALL			# UPDATE LM STATE VECTOR
			ATOPLEM

; Clear flags and complete state vector update processing.
; RENDWFLG is cleared after rendezvous navigation updates.
; UPSVFLAG is reset to zero to indicate update complete.
; INTWAKE0 releases any integration "grab" allowing other programs to proceed.

INTWAKEX	CLEAR
			RENDWFLG

INTWAKUP	SSP	CALL		# REMOVE `UPDATE STATE VECTOR INDICATOR'
			UPSVFLAG
			0
			INTWAKE0	# RELEASE `GRAB' OF ORBIT INTEG.
		EXIT

; Phase change for restart protection, then return to caller.
; During critical mission phases (descent, rendezvous), phase changes ensure
; that power transients or computer resets can recover navigation state.

		TC	PHASCHNG
		OCT	04026
		TC	INTWAKUQ

UPMNSVCD	OCT	2
		OCT	0

; Group 2 Phase Change Routine
; Saves current Q register (return address) and performs phase change for
; restart protection during Group 2 programs (P20-P25 navigation and
; rendezvous programs). This ensures critical navigation computations can
; be recovered after power transients or resets.

GRP2PC		STQ	EXIT
			GRP2SVQ
		TC	PHASCHNG
		OCT	04022
		TC	INTPRET
		GOTO
			GRP2SVQ

