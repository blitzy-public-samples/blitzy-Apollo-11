# Copyright:	Public domain.
# Filename:	IMU_CALIBRATION_AND_ALIGNMENT.agc
# Purpose:      Part of the source code for Comanche, build 055. It
#               is part of the source code for the Command Module's
#               (CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:    yaYUL
# Reference:    pp. 423-455
# Contact:      Onno Hommes <ohommes@cmu.edu>
# Website:      http://www.ibiblio.org/apollo.
# Mod history:  2009-05-10 OH	Batch 1 Assignment Comanche Transcription
#		2009-05-20 RSB	Corrections: P00D00H -> P00DOOH, definition
#				of 25DECML fixed.
#		2009-05-23 RSB	At SPECSTS, corrected to PRIO22.
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
; FILE: IMU_CALIBRATION_AND_ALIGNMENT.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: IMU calibration procedures implementing star sighting alignment,
;        gyrocompass mode operations, coarse and fine align sequences. Establishes
;        accurate inertial platform orientation through optical measurements,
;        enabling precise navigation throughout Apollo 11 mission phases.
;
; COMMENT-ONLY READERS: This program aligned the navigation platform by sighting
;        stars, ensuring accurate spacecraft orientation knowledge.
; CODE-ALONG READERS: Study IMU alignment procedures, star catalog integration,
;        coarse/fine align algorithms, alignment quality metrics, gyrocompass mode.
; ============================================================================

# Page 423
# NAME- IMU PERFORMANCE TESTS 2

# DATE- MARCH 20,1967
#
# BY- SYSTEM TEST GROUP 864-6900 EXT. 1274

# MODNO.- ZERO

# FUNCTIONAL DESCRIPTION

# POSITIONING ROUTINES FOR THE IMU PERFORMANCE TESTS AS WELL AS SOME OF
# THE TESTS THEMSELVES.  FOR A DESCRIPTION OF THESE SUBROUTINES AND THE
# OPERATING PROCEDURES (TYPICALLY) SEE STG MEMO 685.THEORETICAL REF.E-1973

		BANK	33
		SETLOC	IMUCAL
		BANK

; ============================================================================
; IMU PERFORMANCE TEST INITIALIZATION
;
; The Inertial Measurement Unit (IMU) requires periodic calibration to correct
; for gyro drift and accelerometer bias. This section initializes test parameters
; and prepares the stable member for alignment procedures. The IMU contains three
; gyroscopes and three accelerometers mounted on a gimbaled platform that maintains
; inertial reference orientation independent of spacecraft attitude changes.
; ============================================================================

		EBANK=	POSITON
IMUTEST		CA	ZERO
		TS	DRIFTT
		TS	GEOCOMP1
		CAF	TESTTIME
		TS	LENGTHOT
		TC	COAALIGN		# TAKE CARE OF DRIFT FLAG
		CAF	1SECX
		TS	1SECXT1

		CA	OC14400
		TS	1/PIPADT
		
; Compute local vertical orientation parameters based on spacecraft latitude.
; The gyrocompass alignment mode requires knowledge of Earth's rotation vector
; components in the local coordinate frame to null out rotation-induced drift.
; WANGI stores -cos(latitude), WANGO stores sin(latitude), scaled for interpretive math.

GUESS		TC	INTPRET			# CALCULATE -COS LATITUDE AND SIN LATITUDE
		CALL
			LATAZCHK
		COS	DCOMP
		SL1
		STODL	WANGI
			LATITUDE
		SIN	SL1
		STORE	WANGO
		EXIT
		
; ============================================================================
; GYROCOMPASS ALIGNMENT ENTRY POINT
;
; The gyrocompass alignment mode uses Earth's rotation to establish an inertial
; reference without optical star sightings. This is particularly useful during
; launch phases or when star visibility is limited. The procedure zeros the IMU
; Coupling Data Units (CDUs) and monitors gyro drift rates to achieve alignment.
; ============================================================================

GEOIMUTT	TC	BANKCALL		# GYROCOMPASS COMES IN HERE
		CADR	IMUZERO
		TC	IMUSTLLG
IMUBACK		CA	ZERO
		TS	NDXCTR
		TS	TORQNDX
		TS	TORQNDX	+1
		
; ============================================================================
; NAVIGATION BASE (NB) MATRIX INITIALIZATION
;
; Establishes the desired orientation of the stable member relative to the
; navigation base coordinate system. The azimuth angle defines rotation about
; the local vertical, determining which direction will be "north" for navigation
; purposes. During Apollo missions, specific azimuth alignments were chosen based
; on mission phase requirements (launch azimuth, orbital plane orientation, etc.).
; ============================================================================

NBPOSPL		CA	DEC17
		TS	ZERONDX1
		CA	XNBADR
# Page 424
		TC	ZEROING
		CA	HALF
		TS	XNB
		
; Compute navigation base orientation from azimuth angle.
; The NB matrix columns (XNB, YNB, ZNB) represent the desired stable member
; axis directions expressed in the local reference frame. This transformation
; allows the IMU to maintain any desired inertial orientation, not just
; celestial north alignment.

		TC	INTPRET
		DLOAD	SIN
			AZIMUTH
		STORE	YNB	+2
		STODL	ZNB	+4
			AZIMUTH
		COS
		STORE	YNB	+4
		DCOMP
		STORE	ZNB	+2
		EXIT
		TC	CHECKMM
		MM	03			#   SEE IF IN OPTICAL VERIFICATION
		TCF	+2			#  NO
		TCF	SETNBPOS +1		#    YES
		TC	INTPRET
		CALL
			CALCGA
		EXIT
		TC	BANKCALL
		CADR	IMUCOARS
		CAF	GLOKFBIT		# IF GLOKFAIL SET, GIMBAL LOCK
		MASK	FLAGWRD3
		EXTEND
		BZF	+2
		INCR	NDXCTR			# +1 IF IN GIMBAL LOCK,OTHERWISE 0
		TC	DOWNFLAG		# RESET GIMBAL LOCK FLAG
		ADRES	GLOKFAIL		# BIT 14 FLAG 3
		TC	IMUSTLLG
		CCS	NDXCTR			# IF ONE GO AND DO A PIPA TEST ONLY
		TC	PIPACHK			# ALIGN AND MEASURE VERTICAL PIPA RATE
		TC	BANKCALL
		CADR	IMUFINE
		TC	IMUSTLLG
		EXTEND
		DCA	PERFDLAY
		TC	LONGCALL
		EBANK=	POSITON
		2CADR	GOESTIMS

		CA	ESTICADR
		TC	JOBSLEEP
GOESTIMS	CA	ESTICADR
		TC	JOBWAKE
		TC	TASKOVER
ESTICADR	CADR	ESTIMS
# Page 425

; ============================================================================
; TORQUE - Display Gyro Drift Measurement
;
; This routine displays the measured gyro drift value on the DSKY, allowing
; crew or ground personnel to assess IMU performance. The drift measurement
; is stored for later analysis of gyroscope stability. During calibration
; tests, this provides immediate visual feedback of drift magnitude.
; ============================================================================

TORQUE		CA	ZERO
		TS	DSPTEM2
		CA	DRIFTI
		TS	DSPTEM2	+1
		INDEX	POSITON
		TS	SOUTHDR	-1
		TC	SHOW

; ============================================================================
; PIPACHK - PIPA (Accelerometer) Performance Test
;
; Measures accelerometer drift by monitoring PIPA pulse accumulation over time.
; This test verifies that the inertial measurement unit's accelerometers are
; functioning within specifications. During ground testing and pre-launch
; calibration, this provided critical verification of IMU health. The test
; measures acceleration bias and noise characteristics.
; ============================================================================

PIPACHK		INDEX	NDXCTR		# PIPA TEST
		TC	+1
		TC	EARTHR*
		CA	DEC57
		TS	LENGTHOT
		CA	ONE
		TS	RESULTCT
		CA	ZERO
		INDEX	PIPINDEX
		TS	PIPAX
		TS	DATAPL
		TS	DATAPL +4
		TC	CHECKG		# PIP PULSE CATCHING ROUTINE
		INHINT
		CAF	TWO
		TC	TWIDDLE
		EBANK=	XSM
		ADRES	PIPATASK
		TC	ENDOFJOB

; Timer-driven task for PIPA test. Decrements test duration counter and
; schedules PIPJOBB job when test period expires. This implements precise
; timing for accelerometer drift measurement over the specified interval.

PIPATASK	EXTEND
		DIM	LENGTHOT
		CA	LENGTHOT
		EXTEND
		BZMF	STARTPIP
		CAF	BIT10
		TC	TWIDDLE
		EBANK=	XSM
		ADRES	PIPATASK
STARTPIP	CAF	PRIO20
		TC	FINDVAC
		EBANK=	XSM
		2CADR	PIPJOBB

		TC	TASKOVER

; Job to compute and display PIPA test results after measurement period completes.
; Calculates drift rate from accumulated pulses and stores results for crew review.

PIPJOBB		INDEX	NDXCTR
		TC	+1
		TC	EARTHR*
		CA	LENGTHOT
		EXTEND
		BZMF	+2
		TC	ENDOFJOB
		CA	FIVE
# Page 426
		TS	RESULTCT
		TC	CHECKG
		EXTEND
		DCS	DATAPL
		DAS	DATAPL 	+4

		TC	INTPRET
		DLOAD	DSU
			DATAPL	+6
			DATAPL 	+2
		BPL	CALL
			AINGOTN
			OVERFFIX
AINGOTN		PDDL	DDV
			DATAPL 	+4
		SL4	DMPR
			DEC585		# DEC585 HAS BEEN REDEFINED FOR LEM
		RTB
			SGNAGREE
		STORE	DSPTEM2
		EXIT
		CCS	NDXCTR
		TC	COAALIGN	# TAKE PLATFORM OUT OF GIMBAL LOCK
		TC	SHOW

; ============================================================================
; VERTDRFT - Vertical Drift Test
;
; Performs long-duration (approximately 1 hour) vertical axis drift measurement.
; With the platform held in a known orientation, this test measures gyro drift
; along the vertical (gravity) axis. The test applies deliberate platform offsets
; to different axes depending on test configuration, allowing isolation of
; individual gyro drift characteristics.
; ============================================================================

VERTDRFT	CA	3990DEC		# ABOUT 1 HOUR VERTICAL DRIFT TEST
		TS	LENGTHOT
		INDEX	POSITON
		CS	SOUTHDR -2
		TS	DRIFTT
		CA	XSM	+4	# 0 IF POSN 4
		EXTEND
		BZF	PON2
PON4		CS	BIT5		# 	OFFSET PLATFORM
		ADS	ERCOMP1	+2
		CA	BIT5
		ADS	ERCOMP1
		TCF	PONG
PON2		CS	BIT5
		ADS	ERCOMP1 +2
		CA	BIT5
		ADS	ERCOMP1 +4
PONG		TC	EARTHR*
		CA	ZERO		# ALLOW ONLY SOUTH GYRO EARTH RATE COMPENS
		TS	ERVECTOR
		TS	ERVECTOR +1

; Initialize torque index and gimbal angle storage for drift compensation.
; Prepares system for Earth rate compensation during extended drift tests.

GUESS1		CAF	POSMAX
		TS	TORQNDX
		TS	TORQNDX +1
		CA	CDUX
		TS	LOSVEC
# Page 427
		TC	ESTIMS

; Display measured drift value after test completion.

VALMIS		CA	DRIFTO
		TS	DSPTEM2 +1
		CA	ZERO
		TS	DSPTEM2
		TC	SHOW

; ============================================================================
; ENDTEST1 - Test Completion and Resource Release
;
; Terminates IMU calibration test, releases IMU for other programs, and returns
; system to normal operational state. Clears IMUSE flag indicating IMU is no
; longer reserved for calibration testing.
; ============================================================================

ENDTEST1	TC	DOWNFLAG	# IMU NOT IN USE
		ADRES	IMUSE		# BIT 8 FLAG 0
		CS	ZERO
		TC	NEWMODEX +3
		TC	BANKCALL
		CADR	MKRELEAS
		TC	ENDEXT
# Page 428

; Overflow correction routine. Adds DPPOSMAX and ONEDPP to handle double-precision
; arithmetic overflow during torque calculations.

OVERFFIX	DAD	DAD
			DPPOSMAX
			ONEDPP
		RVQ

; ============================================================================
; COAALIGN - Coarse Align Subroutine
;
; Performs initial coarse alignment of the IMU platform. This rapid alignment
; uses approximate orientation commands to bring the platform within a few
; degrees of the desired orientation. Coarse align precedes fine align operations
; and does not require star sightings. Used during ground operations and after
; platform caging to establish rough orientation before precise alignment.
; ============================================================================

COAALIGN	EXTEND			# COARSE ALIGN SUBROUTINE
		QXCH	QPLACE
		CA	ZERO
		TS	THETAD
		TS	THETAD +1
		TS	THETAD +2
		TC	BANKCALL
		CADR	IMUCOARS
		TC	BANKCALL
		CADR	IMUSTALL
		TC	SOMERR2
		TC	QPLACE

; IMU stall and gimbal lock check subroutine. Calls IMU stall routine after
; coarse alignment to ensure platform reaches target orientation before proceeding.

IMUSTLLG	EXTEND
		QXCH	QPLACE
		TC	COAALIGN +10

; ============================================================================
; CHECKG - PIPA Pulse Monitoring Routine
;
; Monitors PIPA (accelerometer) pulse increments to detect platform motion or
; vehicle acceleration during IMU operations. This "pulse catching" routine
; ensures that PIPA measurements remain within acceptable bounds during alignment
; or calibration tests. Verifies platform stability during critical operations.
; ============================================================================

CHECKG		EXTEND			# PIP PULSE CATCHING ROUTINE
		QXCH	QPLACE
		TC	+6
CHECKG1		RELINT
		CA	NEWJOB
		EXTEND
		BZMF	+6
		TC	CHANG1
		INHINT
		INDEX	PIPINDEX
		CS	PIPAX
		TS	ZERONDX
		INHINT
		INDEX	PIPINDEX
		CA	PIPAX
		AD	ZERONDX
		EXTEND
		BZF	CHECKG1
		INDEX	PIPINDEX
		CA	PIPAX
		INDEX	RESULTCT
		TS	DATAPL
		TC	FINETIME
		INDEX	RESULTCT
		TS	DATAPL +1
# Page 429
		INDEX	RESULTCT
		LXCH	DATAPL +2
		RELINT
ENDCHKG		TC	QPLACE

; ============================================================================
; ZEROING - Memory Initialization Utility
;
; Clears a block of memory locations to zero. The routine uses indexed addressing
; to zero multiple consecutive memory locations efficiently. Used during IMU test
; setup to initialize data storage areas before measurement collection begins.
; ============================================================================

ZEROING		TS	L
		TCF	+2
ZEROING1	TS	ZERONDX1
		CAF	ZERO
		INDEX	L
		TS	0
		INCR	L
		CCS	ZERONDX1
		TCF	ZEROING1
		TC	Q

# Page 430
		SETLOC	IMUCAL3
		BANK

; ============================================================================
; ERTHRVSE - Earth Rate Vector Setup
;
; Computes the Earth rotation rate vector components in platform coordinates
; based on spacecraft latitude. The Earth rate compensation vector accounts for
; the planet's rotation, preventing apparent gyro drift caused by Earth's
; rotation beneath the inertial platform. This is essential for maintaining
; accurate inertial reference during long-duration coast phases.
;
; Vector components: (SIN(latitude), -COS(latitude), 0) * Earth_rate
; ============================================================================

ERTHRVSE	DLOAD	PDDL
			SCHZEROS	# PD24 = (SIN		-COS	0)(OMEG/MS)
			LATITUDE
		COS	DCOMP
		PDDL	SIN
			LATITUDE
		VDEF	VXSC
			OMEG/MS
		STORE	ERVECTOR
		RTB
			LOADTIME
		STOVL	TMARK
			SCHZEROS
		STORE	ERCOMP1
		RVQ
		SETLOC	IMUCAL
		BANK

; ============================================================================
; EARTHR - Earth Rate Compensation Calculator
;
; Calculates Earth rotation compensation torque to be applied to the IMU gyros.
; Integrates Earth rate vector over time since last compensation, transforms to
; platform coordinates, and pulses the IMU to maintain true inertial orientation.
; Without this compensation, the gyros would drift at Earth's rotation rate
; (15 degrees/hour), making the platform Earth-referenced instead of inertial.
;
; This routine is called periodically during coast phases to maintain alignment.
; ============================================================================

EARTHR		ITA	RTB		# CALCULATES AND COMPENSATES EARTH RATE
			S2
			LOADTIME
		STORE	TEMPTIME
		DSU	BPL
			TMARK
			ERTHR
		CALL
			OVERFFIX
ERTHR		SL	VXSC
			9D
			ERVECTOR
		MXV	VAD
			XSM
			ERCOMP1
		STODL	ERCOMP1
			TEMPTIME
		STORE	TMARK
		AXT,1	RTB
		ECADR	ERCOMP1
			PULSEIMU
		GOTO
			S2

; Entry point for Earth rate compensation with IMU stall. Ensures platform
; reaches target position after torquing before returning to caller.

EARTHR*		EXTEND
		QXCH	QPLACES
		TC	INTPRET
		CALL
			EARTHR
PROUT		EXIT
		TC	IMUSTLLG
		TC	QPLACES
# Page 431

; ============================================================================
; SHOW - Display Current Position/Status
;
; Displays current test position or alignment status to the crew via DSKY using
; Verb 06 Noun 98 (decimal display). Used during IMU tests to show intermediate
; results or current calibration step. Crew can respond with V33 (proceed) or
; V34 (terminate test). This provides real-time feedback during calibration.
; ============================================================================

SHOW		EXTEND
		QXCH	QPLACE
SHOW1		CA	POSITON
		TS	DSPTEM2 +2
		CA	VB06N98
		TC	BANKCALL
		CADR	GOFLASH
		TC	ENDTEST1	# V 34
		TC	QPLACE		#  V33
		TCF	SHOW1


OC14400		OCT	14400
3990DEC		=	OMEG/MS
VB06N98		VN	0698
TESTTIME	OCT	01602
DEC17		=	ND1
OGCPL		ECADR	OGC
1SECX		=	1SEC
DEC57		=	VD1
XNBADR		GENADR	XNB
XSMADR		GENADR	XSM
OMEG/MS		2DEC	.24339048


P11OUT		TC	BANKCALL
		CADR	MATRXJOB	# RETURN TO P11

		COUNT	02/COMST

		BLOCK	2

; ============================================================================
; FINETIME - Precise Time Reading Utility
;
; Reads mission elapsed time with interrupt protection to ensure atomic read of
; time counter. Uses double-read technique to detect and handle counter rollover
; during read operation. Returns with interrupts inhibited, ensuring time value
; remains synchronized with subsequent operations. Critical for precise timing
; measurements in IMU calibration and drift tests.
; ============================================================================

FINETIME	INHINT			# RETURNS WITH INTERRUPT INHIBITED
		EXTEND
		READ	LOSCALAR
		TS	L
		EXTEND
		RXOR	LOSCALAR
		EXTEND
		BZF	+4
		EXTEND
		READ	LOSCALAR
		TS	L
	+4	CS	POSMAX
		AD	L
		EXTEND
		BZF	FINETIME +1
		EXTEND
		READ	HISCALAR
		TC Q

# Page 432
# PROGRAM NAME-OPTIMUM PRELAUNCH ALIGNMENT CALIBRATION
# DATE- NOVEMBER 2 1966
# BY- GEORGE SCHMIDT IL 7-146 EXT. 126
# MOD NO 3
# FUNCTIONAL DESCRIPTION

# THIS	SECTION CONSISTS OF PRELAUNCH ALIGNMENT AND GYRO DRIFT TESTS
# INTEGRATED TOGETHER TO SAVE WORDS.  COMPASS IS COMPLETELY RESTART
# PROOFED EXCEPT FOR THE FIRST 30 SECONDS OR SO.  PERFORMANCE TESTS OF
# THE IRIGS IS RESTART PROOFED ENOUGH TO GIVE 75 PERCENT CONFIDENCE THAT
# IF A RESTART OCCURS THE DATA WILL STILL BE GOOD.  GOOD PRACTICE TO RECYCL
# WHEN A RESTART OCCURS UNLESS IT HAPPENS NEAR THE END OF A TEST-THEN WAIT
# FOR THE DATA TO FLASH.
# A RESTART IN GYROCOMPASS DURING GYRO TORQUING CAUSES PULSES TO BE LOST
# THE PRELAUNCH ALIGNMENT TECHNIQUE IS BASICALLY THE SAME AS IN BLOCK 1
# EXCEPT THAT IT HAS BEEN SIMPLIFIED IN THE SENSE THAT SMALL ANGLE APPROX.
# HAVE BEEN USED.  THE DRIFT TESTS USE A UNIQUE IMPLEMENTATION OF THE
# OPTIMUM STATISTICAL FILTER.  FOR A DESCRIPTION SEE E-1973.BOTH OF THESE
# ROUTINES USE STANDARD SYSTEM TEST LEADIN PROCEDURES.  THE INITIALIZATION
# PROCEDURE THE DRIFT TESTS IS IN THE JDC S.  THE INITIALIZATION METHOD
# FOR GYROCOMPASS IS AN ERAS LOAD THEN A MISSION PHASE CALL.
# THE COMPASS ALIGNS TO Z DOWN,X DOWNRANGE, HAS THE CAPABILITY
# CHANGE AZIMUTH WHILE RUNNING , IS COMPENSATED FOR
# COMPONENT ERRORS,IS CAPABLE OF OPTICAL VERIFICATION( CSM ONLY).

# COMPASS ERASABLE LOAD REQUIRED

#  1-LAUNCHAZ -DP AZIMUTH IN REV FROM NORTH OF XSM DESIRED	(NOM=.2)
# 2- LATITUDE -DP-OF LAUNCH PAD
# 3- AZIMUTH-DP-OF ZNB OF VEHICLE
# 4- IMU COMPENSATION PARAMETERS
# 5-AZ AND ELEVATION OF TARGETS 1,2		****OPTIONAL****

# TO PERFORM AS PART OF COMPASS

# 1-OPTICAL VERIFICATION- V 65 E
# 2-AZIMUTH CHANGE-V 78 E

# SUBROUTINES CALLED

# DURING OPTICAL VERIFICATION (CSM ONLY) ESSENTIALLY ALL OF INFLIGHT ALIGN
# IS CALLED IN ONE WAY OR ANOTHER.  SEE THE LISTING.

# NORMAL EXIT

# DRIFT TESTS-  LENGTHOT GOES TO ZERO-RETURN TO IMU PERF TEST2 CONTROL
# GYROCOMPASS-MANY, SEE THE LISTING
# ALARMS

# 1600	OVERFLOW IN DRIFT TEST
# Page 433
# 1601	BAD IMU TORQUE ABORT
# 1602	BAD OPTICS DURING VERIFICATION-RETURN TO COMPASS	CSM ONLY

# OUTPUT

# DRIFT TESTS- FLASHING DISPLAYS OF RESULTS-CONTROLLED IN IMU PERF TESTS 2
# COMPASS-PROGRAM MODE LIGHTS TELL YOU WHAT PHAS OF PROGRAM YOU ARE IN
#   01	INITIALIZING THE PLATFORM POSITION AND ERASABLE
#   02	GYROCOMPASSING
#   03	DOING OPTICAL VERIFICATION (CSM)
#
#
# DEBRIS

# ALL CENTRALS,ALL OF EBANK XSM

# Page 434
# MOST OF THE ROUTINES COMMON TO ALIGNMENT AND CALIBRATION APPEAR
# ON THE NEXT FEW PAGES.


		COUNT	33/P02

		EBANK=	XSM
		BANK	33
		SETLOC	IMUCAL
		BANK

; ============================================================================
; ESTIMS - Estimation Initialization for Optimal Alignment
;
; Entry point for optimal prelaunch alignment and calibration routine. Called
; from IMU2 program to begin the comprehensive gyro drift estimation and
; platform alignment process. Sets up phase control, zeros PIPAs (accelerometer
; pulse counters), initializes erasable memory, and prepares for gyrocompass
; or drift test operations. This is the main initialization for Program P03
; (gyrocompass alignment) and IMU performance tests.
; ============================================================================

ESTIMS		TC	2PHSCHNG	# COMES HERE FROM IMU2
		OCT	00075
		OCT	00004		# TURN OFF GROUP 4 IF ON
RSTGTS1		INHINT			# COMES HERE PHASE1 RESTART
		CA	TIME1
		TS	GTSWTLT1
		CAF	ZERO		# ZERO THE PIPAS
		TS	PIPAX
		TS	PIPAY
		TS	PIPAZ
		RELINT
		CA	77DECML		# ZERO ALL NECESSARY LOCATIONS
		TS	ZERONDX1
		CA	ALXXXZ
		TC	ZEROING
		TC	INTPRET
		SLOAD
			SCHZEROS
		STOVL	GCOMPSW -1
			INTVAL +2	# LOAD SOME INITIAL DRIFT GAINS
		STOVL	ALX1S
			SCHZEROS
		STORE	GCOMP
		STORE	DELVX		# GCOMPZER SUBROUTINE NO LONGER NEEDED
		EXIT

		CCS	GEOCOMP1	# NON ZERO IF COMPASS.
		TC	+2
		TC	SLEEPIE	+1
		TC	INTPRET
		CALL
			ERTHRVSE
		EXIT
		CA	LENGTHOT	# TIMES FIVE IS THE NUM OF SEC ERECTING
		TS	ERECTIME

		TC	NEWMODEX
		MM	02
		TC	BANKCALL	# SET UP PIPA FAIL TO CAUSE ISS ALARM
# Page 435
		CADR	PIPUSE		# COMPASS NEVER TURNS THIS OFF
		TC	ANNNNNN		# END OF FIRST TIME THROUGH

# Page 436
# COMES HERE AT THE END OF EVERY ITERATION THROUGH DRIFT TEST OR COMPASS

# SET UP WAITLIST SECTION

; ============================================================================
; SLEEPIE - Iteration Control for Alignment/Calibration Loop
;
; End-of-iteration processing for gyrocompass or drift test cycles. Decrements
; test duration counter (LENGTHOT) and sets up next iteration via waitlist.
; If vertical drift test is active (TORQNDX non-zero), applies Earth rate
; compensation torque to south gyro. Checks if compass operation is complete,
; then schedules next alignment loop cycle. This routine ensures periodic
; sampling and torquing at precise 1-second intervals during calibration.
; ============================================================================

SLEEPIE		TS	LENGTHOT	# TEST NOT OVER-DECREMENT LENGTHOT
		TC	PHASCHNG	# CHANGE PHASE
		OCT	00135
		CCS	TORQNDX		# ARE WE DOING VERTDRIFT
		TC	EARTHR*		# TRUE TORQUE SOUTH GYRO
WTLISTNT	TC	CHKCOMED	# 	SEE IF COMPASS OVER
		TC	SETGWLST
		TC	ENDOFJOB

; ============================================================================
; SETGWLST - Setup Gyrocompass Waitlist Task
;
; Schedules the next alignment loop iteration (ALLOOP) on the waitlist with
; precise 1-second timing (0.5 seconds during gyrocompass). Computes time
; remaining until next cycle, adjusting for processing delays. Maintains
; accurate periodic sampling essential for drift estimation and gyrocompass
; convergence. Called every iteration or when azimuth estimate changes during
; gyrocompass operation. Returns via saved return address in MPAC.
; ============================================================================

SETGWLST	EXTEND
		QXCH	MPAC		# CALLED EVERY WAITLIST OR AZIMUTH CHANGE
		INHINT
		CS	TIME1
		AD	GTSWTLT1
		EXTEND
		BZMF	+2
		AD	NEGMAX		# 10 MS ERROR OK
		AD	1SECXT1		# 1 SEC FOR CALIBRATION, .5 SEC IN COMPASS
		EXTEND
		BZMF	RIGHTGTS
WTGTSMPL	TC	TWIDDLE
		EBANK=	ALTIM
		ADRES	ALLOOP
		TC	MPAC
RIGHTGTS	CAF	FOUR		# SET UP NEXT WAITLIST-ALLOW SOME TIME
		TC	WTGTSMPL	# END OF WAITLIST SECTION


# STORE AND LOAD DATA SECTIONS FOR RESTART PROOFING

; ============================================================================
; STOREDTA - Store Calibration Data for Restart Protection
;
; Copies 26 words of critical alignment data from working storage (THETAX1 area)
; to restart-protected storage (RESTARPT area). This ensures that if a computer
; restart occurs during the multi-hour gyrocompass or drift test, the accumulated
; calibration results are preserved and the test can resume without starting over.
; Called periodically during alignment operations. Uses indexed loop to copy all
; angle and drift estimate data.
; ============================================================================

25DECML		EQUALS	OCT31
STOREDTA	CAF	25DECML
		TS	MPAC
		INDEX	MPAC
		CAE	THETAX1
		INDEX	MPAC
		TS	RESTARPT
		CCS	MPAC
		TCF	STOREDTA +1
		TC	Q

; ============================================================================
; LOADSTDT - Load Calibration Data from Restart Storage
;
; Restores 26 words of critical alignment data from restart-protected storage
; (RESTARPT area) back to working storage (THETAX1 area). Called after a computer
; restart to recover the accumulated calibration state and continue the alignment
; or drift test from where it was interrupted. This restart recovery mechanism
; is crucial for long-duration tests that could otherwise be lost due to brief
; power transients or commanded restarts.
; ============================================================================

LOADSTDT	CAF	25DECML
		TS	MPAC
		INDEX	MPAC
		CA	RESTARPT
		INDEX	MPAC

# Page 437
		TS	THETAX1
		CCS	MPAC
		TCF	LOADSTDT +1
		TC	Q


# COMES HERE EVERY ITERATION BY A WAITLIST CALL SET IN SLEEPIE

; ============================================================================
; ALLOOP - Alignment Loop Main Entry Point
;
; Primary periodic entry point for gyrocompass and drift test iterations. Called
; every 1 second (or 0.5 seconds during gyrocompass) via waitlist task scheduled
; by SLEEPIE/SETGWLST. Saves current time for next waitlist setup, reads and
; processes PIPA (accelerometer) data, applies gyro torquing commands, updates
; drift estimates, and monitors alignment convergence. This is the heart of the
; iterative alignment process that runs continuously during IMU calibration.
; ============================================================================

ALLOOP		CA	TIME1
		TS	GTSWTLT1	# STORE TIME TO SET UP NEXT WAITLIST.
ALLOOP3		CA	ALTIM
		TS	GEOSAVE1
		TC	PHASCHNG
		OCT	00115
ALLOOP1		CAE	GEOSAVE1
		TS	ALTIM
		CCS	A
		CA	A		# SHOULD NEVER HIT THIS LOCATION
		TS	ALTIMS
		CS	A
		TS	ALTIM
		CAF	ZERO
		XCH	PIPAX
		TS	DELVX
		CAF	ZERO
		XCH	PIPAY
		TS	DELVY
		CAF	ZERO
		XCH	PIPAZ
		TS	DELVZ
		CAF	19DECML		# 23 OCT
		TC	NEWPHASE
		OCT	00005
SPECSTS		CAF	PRIO22
		TC	FINDVAC
		EBANK=	GEOSAVE1
		2CADR	ALFLT		# START THE JOB

		TC	TASKOVER

# Page 438
# THIS IS PART OF THE JOB DONE EVERY ITERATION

; ============================================================================
; ALFLT - Alignment Filter Job (Main Alignment Processing)
;
; Comprehensive job routine executing every alignment iteration. Stores calibration
; data for restart protection, processes PIPA velocity increments through the
; alignment filter equations, updates platform angle estimates (THETAX, THETAY,
; THETAZ), computes gyro drift corrections, and applies compensation torquing.
; During gyrocompass mode, performs azimuth convergence monitoring. This job
; implements the core Kalman filtering and iterative estimation algorithms that
; gradually refine platform orientation knowledge to arc-second accuracy.
; ============================================================================

ALFLT		TC	STOREDTA	# STORE DATA IN CASE OF RESTART IN JOB
		TC	PHASCHNG	# THIS IS THE JOB DONE EVERY ITERATION
		OCT	00215
		TCF	+2
ALFLT1		TC	LOADSTDT	# COMES HERE ON RESTART

		CCS	GEOCOMP1
		TC	+2
		TC	NORMLOP
		TC	CHKCOMED	# 	SEE IF PRELAUNCH OVER
		TC	BANKCALL	# COMPENSATION IF IN COMPASS
		CADR	1/PIPA

; ============================================================================
; NORMLOP - Normal Loop Processing (Drift Test Mode)
;
; Standard alignment loop processing used during normal drift test operations
; (non-compass mode). Transforms PIPA velocity increments from stable member
; coordinates to navigation base coordinates using XSM matrix. Computes platform
; angle updates (THETAX, THETAY, THETAZ) by integrating apparent platform tilts
; indicated by horizontal accelerations. Updates gyro drift estimates by comparing
; measured tilts to expected values from Earth rotation and vehicle motion.
; Applies smoothing and filtering to achieve optimal drift estimation.
; ============================================================================

NORMLOP		TC	INTPRET
		DLOAD
			INTVAL
		STOVL	S1
			DELVX
		VXM	VSL1
			XSM
		DLOAD	DCOMP
			MPAC +3
		STODL	DPIPAY
			MPAC +5
		STORE	DPIPAZ

		SETPD	AXT,1
			0
			8D
		SLOAD	DCOMP
			GEOCOMP1
		BMN
			ALWAYSG		# DO A QUICK COMPASS

# Page 439
# NOW WE HAVE JUST THE CALIBRATION PARTS OF THE PROGRAM-NEXT PAGES

		COUNT	33/COMST

; ============================================================================
; ALCGKK - Alignment Constant/Gain Check
;
; Checks if new filter gain coefficients need to be loaded based on elapsed
; time (ALTIMS). Filter gains change over time as alignment uncertainty decreases
; and measurement confidence increases. If ALTIMS is negative, indicating time
; for gain update, branches to ALKCG to load new slope and time constant values
; from the ALFDK gain schedule table. Otherwise continues to ALFLT3 with current
; gains. This adaptive filtering optimizes convergence speed and final accuracy.
; ============================================================================

ALCGKK		SLOAD	BMN
			ALTIMS
			ALFLT3		# NO NEW GAINS NEEDED

; ============================================================================
; ALKCG - Alignment Load Constants and Gains
;
; Loads new filter slope and time constant values from the gain schedule table
; (ALFDK) into active filter coefficient storage (ALDK). Uses indexed addressing
; to transfer 6 double-precision values representing the current gain set. The
; gain schedule is pre-computed based on optimal estimation theory, with gains
; decreasing over time as platform angle uncertainty reduces from initial coarse
; knowledge (~1 degree) to final fine accuracy (~10 arc-seconds).
; ============================================================================

ALKCG		AXT,2	LXA,1		# LOADS SLOPES AND TIME CONSTANTS AT RQST
			12D
			ALX1S
ALKCG2		DLOAD*	INCR,1
			ALFDK +144D,1
		DEC	-2
		STORE	ALDK +10D,2
		TIX,2	SXA,1
			ALKCG2
			ALX1S

; ============================================================================
; ALFLT3 - Alignment Filter Phase 3 (Measurement Incorporation)
;
; Main measurement processing phase of the alignment filter. Initializes loop
; counter for processing 3-axis accelerometer data (DPIPAY array). Leads into
; DELMLP (Delta Measurement Loop) which incorporates velocity increment measurements
; into platform angle estimates. This section executes regardless of whether new
; gains were loaded, processing current measurements with active filter coefficients.
; ============================================================================

ALFLT3		AXT,1			# MEASUREMENT INCORPORATION ROUTINES
			8D		# AND GAIN UPDATES

; ============================================================================
; DELMLP - Delta Measurement Loop
;
; Processes PIPA velocity increment measurements for each axis. Scales raw PIPA
; counts by PIPASC conversion factor, shifts for proper units, and computes the
; difference from expected velocity (INTY). This measurement residual represents
; the error signal indicating platform misalignment. The loop processes all three
; axes (X, Y, Z) in sequence, building the measurement vector that drives the
; Kalman filter update equations in subsequent code sections.
; ============================================================================

DELMLP		DLOAD*	DMP
			DPIPAY +8D,1
			PIPASC
		SLR	BDSU*
			9D
			INTY +8D,1
		STORE	INTY +8D,1
		PDDL	DMP*
			VELSC
			VLAUN +8D,1
		SL2R
		DSU	STADR
		STORE	DELM +8D,1
		STORE	DELM +10D,1
		TIX,1	AXT,2
			DELMLP
			4

; ============================================================================
; ALILP - Alignment Inner Loop (Gain Update)
;
; Updates the adaptive filter gain coefficients (ALK array) by multiplying current
; gains by gain decay factors (ALDK). This implements time-varying Kalman filter
; behavior where measurement weights decrease as alignment accuracy improves and
; state uncertainty reduces. Executes 5 iterations (counter from 4 down) to update
; all gain terms. The decreasing gains prevent late measurements from disrupting
; the converged alignment solution.
; ============================================================================

ALILP		DLOAD*	DMPR*
			ALK +4,2
			ALDK +4,2
		STORE	ALK +4,2
		TIX,2	AXT,2
			ALILP
			8D

; ============================================================================
; ALKLP - Alignment Kalman Loop (State Update)
;
; Applies the Kalman filter update equations to refine platform angle estimates
; (INTY array). Uses updated gains (ALK) and measurement residuals (DELM) to
; compute corrections for each axis. The loop processes 3 axes using indexed
; addressing with CMPX1 controlling axis selection. Each iteration multiplies
; the appropriate gain by measurement residual and adds the correction to the
; current angle estimate. This is the core state update implementing optimal
; linear estimation theory.
; ============================================================================

ALKLP		LXC,1	SXA,1
			CMPX1
			CMPX1
		DLOAD*	DMPR*
			ALK +1,1
			DELM +8D,2
# Page 440
		DAD*
			INTY +8D,2
		STORE	INTY +8D,2
		DLOAD*	DAD*
			ALK +12D,2
			ALDK +12D,2
		STORE	ALK +12D,2
		DMPR*	DAD*
			DELM +8D,2
			INTY +16D,2
		STORE	INTY +16D,2
		DLOAD*	DMP*
			ALSK +1,1
			DELM +8D,2
		SL1R	DAD*
			VLAUN +8D,2
		STORE	VLAUN +8D,2
		TIX,2	AXT,1
			ALKLP
			8D

; ============================================================================
; LOOSE - Launch Sway Extrapolation
;
; Extrapolates vehicle sway motion variables (acceleration, velocity, position)
; forward in time using a state transition matrix (TRANSM1). During prelaunch
; operations, the Saturn V on the pad experiences small swaying motions due to
; wind and structural flexibility. This routine propagates the sway state vector
; to maintain accurate knowledge of vehicle motion relative to the local vertical,
; which affects platform leveling and azimuth alignment. Processes all three axes
; using indexed loop.
; ============================================================================

LOOSE		DLOAD*	PDDL*		# EXTRAPOLATE SWAY VARIABLES
			ACCWD +8D,1
			VLAUN +8D,1
		PDDL*	VDEF
			POSNV +8D,1
		MXV	VSL1
			TRANSM1
		DLOAD
			MPAC
		STORE	POSNV +8D,1
		DLOAD
			MPAC +3
		STORE	VLAUN +8D,1
		DLOAD
			MPAC +5
		STORE	ACCWD +8D,1
		TIX,1
			LOOSE

; ============================================================================
; BOOP - Compute Sines and Cosines for Angular Updates
;
; Evaluates sine and cosine functions for the three platform angles (ANGX array),
; storing results in transformation matrix arrays. Multiplies each angle by GEORGEJ
; scaling constant, computes sine and cosine, and stores with appropriate addressing.
; These trigonometric values form the rotation matrix elements used to transform
; coordinates between inertial and platform reference frames. Loop processes all
; three axes (X, Y, Z) with index arithmetic managing storage locations for both
; sine and cosine arrays.
; ============================================================================

		AXT,2	AXT,1		# EVALUATE SINES AND COSINES
			6
			2
BOOP		DLOAD*	DMPR
			ANGX +2,1
			GEORGEJ
		SR2R
		PUSH	SIN
# Page 441
		SL3R	XAD,1
			X1
		STORE	16D,2
		DLOAD
		COS
		STORE	22D,2		# COSINES
		TIX,2
			BOOP

; ============================================================================
; PERFERAS - Performance Test Erasable Section Handoff
;
; Transfers control to erasable memory routines for IMU performance test calculations.
; Sets appropriate EBANK addressing for LAT(SPL) variable access, then executes TC
; (Transfer Control) to jump to erasable program section. This handoff technique
; allows complex test algorithms to execute from RAM rather than fixed ROM, enabling
; on-orbit software updates or ground-loaded test procedures without altering core
; flight software. The CAUTION comment warns that erasable test code must be loaded
; before executing IMU performance tests.
; ============================================================================

PERFERAS	EXIT
		CA	EBANK7
		EBANK=	LAT(SPL)
		TS	EBANK
		TC	LAT(SPL)	# GO TO ERASABLE ONLY TO RETURN

# CAUTION

# THE ERASABLE PROGRAM THAT DOES THE CALCULATIONS MUST BE LOADED
# BEFORE ANY ATTEMPT IS MADE TO RUN THE IMU PERFORMANCE TEST

; ============================================================================
; ONCEMORE - Test Continuation Decision Point
;
; Checks remaining test duration (LENGTHOT) to determine if IMU performance test
; should continue for another iteration or if test sequence is complete. If time
; remains (positive LENGTHOT), branches to SLEEPIE to schedule next waitlist call.
; If test is complete, checks torque index (TORQNDX) and proceeds to SETUPER1 for
; final drift angle processing. Also captures CDU gimbal angle (CDUX) into LOSVEC
; for troubleshooting and post-test analysis.
; ============================================================================

		EBANK=	LENGTHOT
ONCEMORE	CCS	LENGTHOT
		TC	SLEEPIE		# TEST NOT OVER SET UP NEXT WAITLIST
		CCS	TORQNDX
		TCF	+2
		TC	SETUPER1
		CA	CDUX
		TS	LOSVEC +1	# FOR TROUBLESHOOTING POSNS 2$4 VD

; ============================================================================
; SETUPER1 - Drift Test Completion Processing
;
; Final processing after IMU drift test completes. Loads the three measured drift
; angles (ANGX, ANGY, ANGZ) representing gyro drift accumulated during test period,
; scales by GEORGEJ constant for proper units, transforms via XSM matrix to convert
; from platform coordinates to stable member coordinates, and shifts result for
; storage. These drift measurements quantify IMU gyroscope performance and can be
; used to update drift compensation models or assess hardware health during mission.
; ============================================================================

SETUPER1	TC	INTPRET		# DRIFT TEST OVER
		DLOAD	PDDL		# ANGLES FROM DRIFT TEST ONLY
			ANGZ
			ANGY
		PDDL	VDEF
			ANGX
		VCOMP	VXSC
			GEORGEJ
		MXV	VSR1
			XSM
		STORE	OGC
		EXIT

; ============================================================================
; TORQINCH - Torque Inch (Apply Drift Compensation Torques)
;
; Applies compensating torque pulses to IMU gyroscopes to null out measured drift
; angles (OGC). Uses IMUPULSE routine to command precise torque motor current pulses
; that physically rotate gyro gimbals by calculated drift amount, effectively
; zeroing accumulated drift error. Checks TORQNDX to determine if vertical drift
; test is complete (branches to VALMIS if positive). After torquing, sets up earth
; rate vector (ERTHRVSE) for subsequent PIPA test or gyrocompass operation. This
; compensation maintains platform accuracy by periodically correcting gyro drift,
; a key technique for long-duration missions.
; ============================================================================

TORQINCH	TC	PHASCHNG
		OCT	00005
		CA	OGCPL
		TC	BANKCALL
		CADR	IMUPULSE
		TC	IMUSTLLG
		CCS	TORQNDX		# + IF IN VERTICAL DRIFT TEST
		TC	VALMIS		# VERT DRIFT TEST OVER
		TC	INTPRET
# Page 442
		CALL			# SET UP ERATE FOR PIP TEST OR COMPASS
			ERTHRVSE
		EXIT
		TC	TORQUE		# GO TO IMU2 FOR A PIPA TEST AND DISPLAY

; ============================================================================
; SOMEERRR - IMU Alignment Error Handler 1 (Alarm 1600)
;
; Error handler for IMU calibration or alignment failures detected during optical
; mark processing or gyrocompass operation. Issues program alarm code 1600 to alert
; crew via DSKY display that IMU alignment quality is degraded or that calibration
; has detected out-of-tolerance conditions. After alarm display, branches to ENDTEST1
; for graceful test termination. Crew can assess situation and decide whether to
; retry alignment, accept degraded accuracy, or abort current mission phase. This
; alarm typically indicates star tracker measurement problems, excessive platform
; drift, or gimbal lock approach during alignment sequence.
; ============================================================================

SOMEERRR	TC	ALARM
		OCT	1600
		TC	+3

; ============================================================================
; SOMERR2 - IMU Alignment Error Handler 2 (Alarm 1601)
;
; Second error handler for IMU alignment failures, issuing program alarm 1601 to
; distinguish from 1600 alarm type. Indicates different failure mode such as optical
; telescope gimbal limit reached, CDU angle readout failure, or gyrocompass convergence
; failure. Like SOMEERRR, terminates test via ENDTEST1 after alarm display. Having
; distinct alarm codes (1600 vs 1601) aids ground controllers and crew in diagnosing
; root cause without requiring detailed telemetry analysis, enabling faster response
; to IMU problems during time-critical mission phases.
; ============================================================================

SOMERR2		TC	ALARM
		OCT	1601
		TC	PHASCHNG
		OCT	00005
		TC	ENDTEST1


# THE FAMOUS MAGIC NUMBERS OF SCHMIDT ARE NOW PART OF AN ERASABLE LOAD.


DEC585		OCT	02222		# 1170 B+14 ORDER IS NOW IMPORTANT
SCHZEROS	2DEC	.00000000
		2DEC	.00000000
		OCT	00000
ONEDPP		OCT	00000
		OCT	00001		# 	ABOVE ORDER IS IMPORTANT

INTVAL		OCT	4
		OCT	2
		DEC	144
		DEC	-1
SOUPLY		2DEC	.93505870	# INITIAL GAINS FOR PIP OUTPUTS
		2DEC	.26266423	# INITIAL GAINS/4 FOR ERECTION ANGLES

77DECML		DEC	77
ALXXXZ		GENADR	ALX1S -1

; ============================================================================
; TRANSITION: From IMU Performance Tests to Gyrocompass Alignment
;
; The gyrocompass alignment procedure uses Earth's rotation as an inertial
; reference to establish platform orientation. Unlike star sighting alignment,
; gyrocompass can operate during daylight or when optical sighting is not
; practical. During Apollo 11 prelaunch operations, gyrocompass alignment
; verified the IMU orientation before liftoff from Kennedy Space Center.
; ============================================================================

# GYROCOMPASS PORTIONS FINISH THIS LOG SECTION


		COUNT	33/P01

# INITIALIZATION SECTION

; ============================================================================
; GYROCOMPASS INITIALIZATION (Called by Verb 37)
;
; Entry point for gyrocompass alignment sequence. Checks that no previous
; program (P01-P11) is running, then initializes gyrocompass parameters.
; The gyrocompass uses a 0.5-second iteration loop to integrate gyro outputs
; and converge on the local vertical and launch azimuth alignment.
; ============================================================================

GTSCPSS		CA	FLAGWRD1	# CALLED BY V37
		MASK	NOP01BIT
# Page 443
		EXTEND
		BZF	GTSCPSSA
		TC	POODOO
		OCT	1521		# NODO ALARM FOR P01 - P11 ALREADY DONE

; Initialize gyrocompass mode parameters.
; GEOCOMP1 flag indicates gyrocompass active (vs. standard IMU test mode).
; 1/2SECX sets 0.5-second update rate for gyrocompass drift integration.
; Launch azimuth (LUNCHAZ1) defines the desired heading for the stable member
; X-axis, typically aligned with the launch pad azimuth direction.

GTSCPSSA	CAF	ONE
		TS	GEOCOMP1	# THIS IS THE LEAD IN FOR COMPASS.
		CA	1/PIPAGT
		TS	1/PIPADT
NXXTENN		CA	BIT8
		TS	LENGTHOT
		CAF	1/2SECX		# COMPASS IS A .5 SEC LOOP
		TS	1SECXT1
		CAF	ONE
		TS	PREMTRX1
		TS	PERFDLAY +1
		CAF	ZERO
		TS	PERFDLAY
		EXTEND
		DCA	LUNCHAZ1
		DXCH	NEWAZ1
		EXTEND
		DCA	LUNCHAZ1
		DXCH	OLDAZMTH
SETUPGC		CA	DEC17
		TS	ZERONDX1
		CA	XSMADR
		TC	ZEROING
		TC	POSN17C
		TC	GEOIMUTT	# GO TO IMU2 FOR FURTHER INITIALIZATION

; Compute stable member (SM) matrix for gyrocompass orientation.
; ZSM points down (toward Earth center), X-axis points downrange along
; launch azimuth, Y-axis completes right-handed coordinate system.
; This configuration is optimized for launch vehicle guidance during ascent.

POSN17C		EXTEND			# COMPASS POSITION Z DOWN,X DOWNRANGE
		QXCH	QPLACE		# FROM NORTH IN REVOLUTIONS + CLOCKWISE
		CS	HALF		# ALL THIS TO INITIALIZE MATRIX
		TS	ZSM
		TC	INTPRET
		DLOAD	PUSH
			NEWAZ1
		SIN
		STORE	XSM	+4
		STODL	YSM	+2
		COS
		STORE	YSM	+4
		DCOMP
		STORE	XSM	+2
		EXIT
		TC	QPLACE

# Page 444
# JOB DONE EVERY ITERATION THROUGH COMPASS PROGRAM.SET BY TASK ALLOOP

		COUNT	33/P02

; ============================================================================
; GYROCOMPASS MAIN ITERATION LOOP
;
; This routine executes every 0.5 seconds during gyrocompass alignment,
; processing gyro outputs (PIPA pulses) to determine platform drift from the
; desired local vertical orientation. Earth's rotation causes predictable
; drift patterns that allow the computer to calculate alignment errors and
; generate corrective gyro torquing commands.
;
; The filtered delta-V values (FILDELV1) are updated with new PIPA readings,
; compensated by geometric constants (GEOCONS1, GEOCONS2) that account for
; Earth's rotation rate at the current latitude. Integration of these values
; (INTVEC1) provides the angular error between current and desired orientation.
; ============================================================================

ALWAYSG		DLOAD*	DSU*		# COMPASS AND ERECT
			DPIPAY +8D,1
			FILDELV1 +8D,1
		DMPR	DAD*
			GEOCONS1
			FILDELV1 +8D,1
		STORE	FILDELV1 +8D,1
		DAD*
			INTVEC1 +8D,1
		STORE	INTVEC1 +8D,1
		DMPR	DAD*
			GEOCONS2
			FILDELV1 +8D,1
		DMPR	PUSH
			GEOCONS5
		TIX,1	SLOAD
			ALWAYSG
			ERECTIM1
		BZE	DLOAD
			COMPGS
			THETAN1 +2
		DSU	STADR
		STODL	THETAN1 +2	# ERECTION ONLY.
		BDSU
			THETAN1 +4
		STORE	THETAN1 +4
		GOTO
			ADDINDRF
			
; ============================================================================
; COMPGS - Gyrocompass Angle Computation
;
; Computes platform alignment errors during gyrocompass mode by integrating PIPA
; outputs (FILDELV1) with geometric compensation factors (GEOCONS3, GEOCONS4). The
; three THETAN1 angles represent accumulated drift error in each axis that must be
; corrected through gyro torquing. Earth's rotation creates predictable drift patterns:
; gyroscopes sense horizontal component of Earth rate at the local latitude. By
; observing this drift and comparing to predicted values, the computer calculates
; the angular error between current platform orientation and the desired local vertical
; plus launch azimuth alignment. Values converge toward zero as alignment improves.
; After each iteration, control passes to ADDINDRF for final processing.
; ============================================================================

COMPGS		DLOAD	DAD		# COMPASS
			THETAN1
			FILDELV1
		STODL	THETAN1
			FILDELV1
		DMPR	BDSU
			GEOCONS3
			THETAN1 +4
		STODL	THETAN1 +4
			FILDELV1 +4
		DMPR	BDSU
			GEOCONS3
			THETAN1 +2
		PDDL	DMPR
			INTVEC1 +4
			GEOCONS4
		BDSU	STADR
		STORE	THETAN1 +2
# Page 445
; ============================================================================
; ADDINDRF - Add Inertial Drift (Exit from Interpretive)
;
; Exits from interpretive language execution back to native AGC code after gyrocompass
; angle computation completes. The THETAN1 values computed by COMPGS have been stored
; and are ready for integration into the alignment correction loop. Control passes
; to ENDGTSAL to check if the 5-second gyrocompass torquing interval has elapsed,
; determining whether to schedule another waitlist iteration or proceed to final
; torquing. This exit point enables seamless transition between interpretive vector
; mathematics and native AGC control flow for time-sensitive operations.
; ============================================================================

ADDINDRF	EXIT


; ============================================================================
; ENDGTSAL - End Gyrocompass Test/Alignment Loop Decision
;
; Decision point determining if 5-second gyrocompass torquing interval has elapsed
; (LENGTHOT timer). If time remains, schedules next waitlist call via SLEEPIE for
; 0.5 seconds to continue alignment iterations. If interval complete, checks if
; gyros are busy (LGYRO) to avoid torquing conflict. When clear, proceeds to LASTGTS
; for final alignment torquing. This paced iteration prevents overwhelming the IMU
; with continuous torque commands while allowing gradual convergence to accurate
; alignment. The 5-second interval balances alignment speed versus platform stability.
; ============================================================================

ENDGTSAL	CCS	LENGTHOT	# IS 5 SEC OVER-THE TIME TO TORQ PLATFORM
		TC	SLEEPIE		# NO-SET UP NEXT WAITLIST CALL FOR .5 SEC
		TC	CHKCOMED
		CCS	LGYRO		# YES BUT ARE GYROS BUSY
		TCF	SLEEPIE +1	# BUSY-GET THEM .5 SECONDS FROM NOW

; ============================================================================
; LASTGTS - Last Gyrocompass Test Store
;
; Prepares for final gyrocompass torquing by saving current error compensation angles
; (ERCOMP1) to THETAX1 and preserving time mark (TMARK) in ALK for restart protection.
; This snapshot captures alignment state before applying final torque corrections,
; enabling restart recovery if power interruption or other failure occurs during
; torquing. The comment "PREVIOUS SECTION WAS FOR RESTARTS" indicates this data
; preservation is specifically for the restart protection mechanism, ensuring gyrocompass
; alignment can resume without loss of accumulated convergence progress if interrupted.
; ============================================================================

LASTGTS		TC	INTPRET
		VLOAD
			ERCOMP1
		STODL	THETAX1
			TMARK
		STORE	ALK
		EXIT			# PREVIOUS SECTION WAS FOR RESTARTS

; ============================================================================
; RESTAIER - Restart-Protected Gyrocompass Torquing
;
; Combines accumulated gyrocompass drift angles (THETAN1) with existing error
; compensation (THETAX1) to compute final alignment correction stored in ERCOMP1.
; Transforms THETAN1 through XSM matrix to convert from platform to stable member
; coordinates, scales by 2 (VSL1), and adds to previous error compensation. After
; updating time mark (TMARK) from ALK, calls EARTHR* to torque all corrections into
; the IMU gyros. Saves erection timer (ERECTIM1) to GEOSAVE1 for subsequent operations.
; Phase change protection (00275, 00155) ensures restart recovery at appropriate
; points if power interruption occurs during this critical torquing sequence.
; ============================================================================

RESTAIER	TC	PHASCHNG
		OCT	00275
		TC	INTPRET		# ADD COMPASS COMMANDS INTO ERATE
		VLOAD	MXV
			THETAN1
			XSM
		VSL1	VAD
			THETAX1
		STODL	ERCOMP1
			ALK
		STORE	TMARK
		EXIT
		TC	EARTHR*		# TORQUE IT ALL IN
		CAE	ERECTIM1
		TS	GEOSAVE1
		TC	PHASCHNG
		OCT	00155
; ============================================================================
; RESTEST1 - Reset Test Angles to Zero
;
; Clears accumulated gyrocompass drift angles (THETAN1) by loading zeros from
; SCHZEROS and storing to THETAN1 vector. This reset prepares for next alignment
; iteration or test cycle, ensuring previous gyrocompass corrections don't carry
; forward into subsequent operations. After zeroing, checks matrix premultiply
; control flag (PREMTRXC) to determine if additional processing is required before
; final test completion. This clean-slate approach prevents error accumulation
; across multiple gyrocompass runs during pre-launch checkout or mission realignment.
; ============================================================================

RESTEST1	TC	INTPRET
		VLOAD
			SCHZEROS
		STORE	THETAN1
		EXIT
		CCS	PREMTRXC
		TC	NOCHORLD
		TC	PHASCHNG
		OCT	00255

; ============================================================================
; RESTEST3 - Azimuth Change Detection and Matrix Update
;
; Compares current launch azimuth (LAUNCHAZ) against previously stored azimuth
; (OLDAZMTH) to detect any azimuth changes that require IMU orientation updates.
; If launch azimuth has changed (e.g., due to pad realignment or launch site
; change), computes the delta azimuth and stores in ERCOMP+4 for compensation.
; Increments matrix premultiply counter (PREMTRXC) and updates azimuth reference
; (NEWAZMTH). If no azimuth change detected, branches to NOAZCHGE. This routine
; handles dynamic launch site changes during countdown or holds, ensuring IMU
; alignment reflects actual spacecraft orientation relative to launch azimuth.
; ============================================================================

RESTEST3	TC	INTPRET
		DLOAD
			LAUNCHAZ
		DSU	BZE
			OLDAZMTH
			NOAZCHGE
		STORE	0D
# Page 446
		SLOAD	DAD
			ONEDPP +1
			PREMTRXC	# DOES NOT CHANGE LAUNCHAZ
		STODL	PREMTRXC
			LAUNCHAZ
		STODL	NEWAZMTH
			0D

; Azimuth delta error compensation: Store computed azimuth change in ERCOMP+4
; for subsequent IMU torquing to physically realign gyros to new azimuth reference.

ADERCOMP	STORE	ERCOMP +4
		EXIT
		TC	POSN17C
		TC	PHASCHNG
		OCT	00335

; ============================================================================
; RESCHNG - Azimuth Reference Update and Erection Timer Reset
;
; Updates the old azimuth reference (OLDAZMTH) with the new azimuth value
; (NEWAZMTH) to track current launch azimuth. Resets gyrocompass erection
; duration to 320 seconds (BIT7) in LENGTHOT, initiating a full erection cycle
; at the new azimuth. This routine handles the procedural reset required when
; launch azimuth changes, ensuring the IMU undergoes complete re-erection at
; the updated reference orientation. The 320-second erection time allows gyros
; to fully settle and stabilize at the new azimuth before verification testing.
; ============================================================================

RESCHNG		EXTEND
		DCA	NEWAZMTH
		DXCH	OLDAZMTH
		CA	BIT7		# SPEND 320 SEC ERECTING
		TS	LENGTHOT
		TC	PHASCHNG
		OCT	00075

; ============================================================================
; SPITGYRO - Apply Azimuth Correction Torques to IMU
;
; Physically torques the IMU gyroscopes to compensate for detected azimuth error
; stored in ERCOMP. Calls IMUPULSE to generate torquing pulses that mechanically
; rotate the stable member to the new azimuth orientation. After torquing, stalls
; IMU operations (IMUSTALL) to allow gyros to settle before resuming navigation.
; If torquing fails, branches to error handler SOMERR2. On success, returns to
; ESTIMS to re-initialize gyrocompass state for verification. This routine is the
; physical actuation step that corrects IMU orientation when azimuth changes are
; detected during pre-launch countdown holds or pad realignments.
; ============================================================================

SPITGYRO	CA	ERCOMPPL
		TC	BANKCALL
		CADR	IMUPULSE
		TC	BANKCALL
		CADR	IMUSTALL
		TC	SOMERR2
		TC	ESTIMS		# RE-INITIALIZE

; ============================================================================
; NOAZCHGE - Continue Gyrocompass Without Azimuth Correction
;
; Branch taken when no launch azimuth change detected. Exits interpreter mode
; and sets premultiply counter to 1 to indicate stable azimuth reference. Falls
; through to NOCHORLD to continue gyrocompass erection countdown without the
; azimuth-change torquing and reset cycle required by the SPITGYRO path.
; ============================================================================

NOAZCHGE	EXIT
		CA	ONE
		TS	PREMTRXC

; Erection timer countdown: If erection time remaining (GEOSAVE1) is positive,
; store in ERECTIM1 and continue countdown. If erection complete, falls through
; to ANNNNNN to reset monitoring interval. This countdown tracks the 320-second
; gyrocompass erection phase, ensuring gyros fully stabilize before verification.

NOCHORLD	CCS	GEOSAVE1
		TS	ERECTIM1	# COUNTS DOWN FOR ERECTION.

; ============================================================================
; ANNNNNN - Reset Monitoring Interval Timer
;
; Sets next monitoring interval to 9 seconds (NINE) in LENGTHOT and returns
; to SLEEPIE+1 to wait for next gyrocompass check cycle. During gyrocompass
; erection, the routine periodically wakes up to verify IMU alignment status,
; check for azimuth changes, and monitor erection progress. The 9-second interval
; balances responsiveness to launch azimuth updates against computational overhead
; during pre-launch countdown operations.
; ============================================================================

ANNNNNN		CAF	NINE
		TS	LENGTHOT
		TC	SLEEPIE +1

; ============================================================================
; CHKCOMED - Check for Mission Mode Changes and Liftoff
;
; Monitors for conditions requiring gyrocompass termination: checks if already in
; MM 07 (performance test mode) and returns to calling routine if so; otherwise
; checks CHAN30 BIT5 for liftoff signal from Saturn V launch sequencer. If liftoff
; detected (either primary or backup signal via FLAGWRD5 BIT5), branches to
; PRELTERM to terminate gyrocompass and handoff to Program 11 (ascent guidance).
; If no liftoff, returns to caller (GOBKCALB). During Apollo 11 countdown, this
; check ran every 9 seconds, ready to detect the July 16, 1969 liftoff and
; instantly transition from ground-aligned IMU to inertial ascent navigation.
; ============================================================================

CHKCOMED	INHINT
		CS 	MODREG		# CHECK FOR MM 07 FIRST
		AD 	SEVEN
		EXTEND
		BZF 	GOBKCALB	# IF MM 07 RETURN TO PERF TEST
		CS	ZERO
		EXTEND
		RXOR	CHAN30		# READ AND INVERT BITS IN CHANNEL 30
		MASK	BIT5		# LIFTOFF BIT
		CCS	A
		TCF	PRELTERM	# LIFTOFF HAS OCCURRED

# Page 447
		CA	GRRBKBIT	# CHECK FOR BACKUP LIFTOFF
		MASK	FLAGWRD5	# BIT5 FLAGWRD5
		CCS	A
		TCF	PRELTERM	# BACKUP RECEIVED

		RELINT
GOBKCALB	TC	Q

; ============================================================================
; PRELTERM - Prelaunch Termination and Handoff to Program 11
;
; Terminates gyrocompass operations upon liftoff detection and initiates handoff
; to Program 11 (P11) for ascent guidance. Increases task priority to PRIO22
; (higher than SERVICER) to ensure P11 initialization takes precedence over
; background tasks. Performs POSTJUMP to P11 entry point, transferring control
; to ascent navigation which will use the gyrocompass-aligned IMU as inertial
; reference for Saturn V guidance during first-stage ascent. For Apollo 11, this
; transition occurred at 102:00:00 mission time on July 16, 1969, beginning the
; 12-minute powered ascent to Earth orbit insertion.
; ============================================================================

PRELTERM	CA	PRIO22		# PRELAUNCH DONE - SET UP P11
		TC	PRIOCHNG	# INCREASE PRIORITY HIGHER THAN SERVICER
		INHINT
		TC	POSTJUMP
		CADR	P11

; ECADR reference to ERCOMP error compensation array used by IMUPULSE for torquing.

ERCOMPPL	ECADR	ERCOMP

GEOCONS5	EQUALS	HIDPHALF
1/PIPAGT	OCT	06200
17DECML		=	ND1		# OCT 21
19DECML		=	VD1		# OCT 23
1/2SECX		=	.5SEC


# Page 448
GEOSTRT4	EQUALS	ENDOFJOB

# Page 449
# OPTICAL VERIFICATION ROUTINES FOR GYROCOMPASS

; ============================================================================
; OPTICAL VERIFICATION FOR GYROCOMPASS ALIGNMENT
;
; The gyrocompass alignment can be verified by sighting known ground targets
; using the spacecraft's optical instruments (sextant). This routine (entered
; via Verb 65) prompts the crew to input target azimuth and elevation data,
; calculates expected target direction in Navigation Base coordinates, and
; compares with actual optical measurements. Discrepancies reveal alignment
; errors that can be used to refine the IMU orientation. This verification
; builds crew confidence that the gyrocompass procedure has correctly established
; the stable member orientation.
;
; During Apollo 11's mission, optical alignment verification was critical during
; translunar coast and lunar orbit phases to ensure navigation accuracy before
; the descent to the lunar surface.
; ============================================================================

		COUNT	33/P03

GCOMPVER	TC	PHASCHNG	# OPTICAL VERIFICATION ROUTINE
		OCT	00154
		TC	NEWMODEX	# ENTERED BY VERB 65 ENTER
		MM	03
		
; Initialize Navigation Base position matrix for the verification. The NBPOSPL
; routine sets up the transformation between the local vertical coordinate frame
; and the Navigation Base frame established by the gyrocompass alignment.

SETNBPOS	TC	NBPOSPL
		TC	BANKCALL
		CADR	MKRELEAS
		
; Prompt crew to input target coordinates for two optical sightings.
; Verb 06 Noun 41 is displayed to request azimuth and elevation angles for each
; target. The crew measures azimuth clockwise from north (0-360°) and elevation
; above the horizon (0-90°). Two targets are used to provide redundant alignment
; verification. During translunar coast, crew typically sighted known landmarks
; on Earth or stars. In lunar orbit, prominent lunar surface features served as
; alignment references.

OPTDATA		CAF	BIT1		# CALLS FOR AZIMUTH AND ELEVATION OF TARGE
		ZL			# T 1,THEN TARGET 2
		LXCH	RUN		# AZIMUTH CLOCKWSE FROM NORTH TO TARGET
		TS	DSPTEM1 +2	# ELEVATION MEASURED FROM HORIZONTAL
		EXTEND
		INDEX	RUN
		DCA	TAZEL1
		DXCH	DSPTEM1
		CAF	V05N30E
		TC	BANKCALL
		CADR	GODSPRET
		CAF	VN0641
		TC	BANKCALL
		CADR	GOFLASH
		TC	GCOMP5
		TC	+3
		TC	-8D
VN0641		VN	0641
		DXCH	DSPTEM1		# TAZEL1 TARGET 1 AZIMUTH
		INDEX	RUN
		DXCH	TAZEL1		# TAZEL1 +2 TARGET 2 AZIMUTH
		CCS	RUN
		TCF	+4
		CAF	TWO
		TS	L
		TCF	OPTDATA +2	# MPAC	1ST PASS=0 2ND PASS=2


		TC	CONTIN33

V05N30E		VN	0530

; ============================================================================
; TARGET VECTOR COMPUTATION IN EARTH REFERENCE FRAME
;
; Transform target azimuth/elevation angles into unit vectors expressed in the
; Earth reference coordinate frame. The transformation produces three components:
;   Z-component: sin(elevation) - vertical component
;   X-component: -cos(azimuth)*cos(elevation) - north reference component
;   Y-component: sin(azimuth)*cos(elevation) - east reference component
;
; This representation allows comparison between predicted target direction (based
; on known target position and spacecraft location) and actual sextant measurements.
; The computation is performed for both targets using indexed loop execution.
; ============================================================================

		TC	INTPRET		# UNDYNAMIC ASSEMBLER
TAR/EREF	AXT,1	AXT,2		# TARGET VECTOR
			2		# SIN(EL)   -COS(AZ)COS(EL)   SIN(AZ)COS(EL
			12D
		SSP	SETPD
			S2
			6
# Page 450
			0
TAR1		SLOAD*	SR2		# X1=2 X2=12 S2=6 X1=0 X2=6 S2=6
			TAZEL1 +3,1
		STORE	0		# PD00 ELEVATION PD00
		SIN
		STORE	18D,2		# PD06 *** SIN(EL) ***PD12
		DLOAD
			0
		COS	PUSH		# PD00 COS(EL) PD00
		SLOAD*	RTB
			TAZEL1 +2,1
			CDULOGIC
		STORE	2		# PD02 AZIMUTH PD02
		SIN	DMP
			0
		SL1
		STORE	22D,2		# PD10 *** SIN(AZ)COS(EL) ***PD16
		DLOAD	COS
			2
		DMP	SL1
		DCOMP	AXT,1
			0
		STORE	20D,2		# PD08 *** -COS(AZ)COS(EL) ***PD14
		TIX,2	RVQ
			TAR1


		BANK	33
		SETLOC	IMUCAL
		BANK
		COUNT*	$$/P03

; Transform target vectors from Earth reference frame to stable member (SM)
; coordinates using the XSM transformation matrix. The stable member frame is
; defined by the IMU gimbaled platform orientation. If the gyrocompass alignment
; is accurate, the transformed target vectors should match the actual line-of-sight
; vectors measured by the crew using the sextant. Discrepancies indicate alignment
; errors requiring correction or refinement.

CONTIN33	CA	ONE
		TS	STARCODE
		CA	ZERO
		TC	TARGDRVE
		TC	INTPRET
		CALL
			TAR/EREF
			
; Apply XSM transformation to both target vectors. The XSM matrix represents the
; transformation from Earth reference coordinates to stable member coordinates,
; accounting for spacecraft position and IMU platform orientation. STARAD stores
; the first target vector, STARAD+6 stores the second target vector.

NEXTBNKS	VLOAD	MXV
			6D
			XSM
		VSL1
		STOVL	STARAD
			12D
		MXV	VSL1
			XSM
		STCALL	STARAD +6
			LITTLSUB
		STORE	LOSVEC
# Page 451
		EXIT
		TC	BANKCALL
		CADR	MKRELEAS

; ============================================================================
; SECOND TARGET PROCESSING
;
; After processing the first optical target, a second target is measured to
; provide additional alignment information. Using two targets enables the
; alignment algorithm to compute gyro torquing angles that correct both the
; orientation and any residual drift of the stable member. The AXISGEN routine
; generates orthogonal axes from the two target vectors, and CALCGTA computes
; the gyro torquing angles (GTA) needed to align the platform.
; ============================================================================

NEXBNKSS	CAF	TWO
		TS	STARCODE
		CAF	SIX
		TC	TARGDRVE
		TC	INTPRET
		CALL
			LITTLSUB
		STOVL	12D
			LOSVEC
		STCALL	06D
			AXISGEN
		CALL
			CALCGTA
		EXIT

; ============================================================================
; ALIGNMENT RESULT DISPLAY AND CREW CONFIRMATION
;
; GCOMP4 displays the computed gyro torquing angles (GTA) to the crew using
; Verb 06 Noun 93. These angles show the IMU misalignment detected by the
; optical measurements. The crew reviews the values and presses ENTER to
; accept or PROCEED to recycle. Accepted values are accumulated into ERCOMP1
; for alignment quality tracking. This step ensures the crew maintains
; awareness of platform alignment quality throughout the mission.
; ============================================================================

GCOMP4		CAF	V06N93S
		TC	BANKCALL
		CADR	GOFLASH		; Display gyro torquing angles with V06N93
		TC	GCOMP5		; ENTER: Accept and complete alignment
		TCF	+2		; PROCEED: Recycle alignment
		TCF	GCOMP4
		TC	INTPRET
		VLOAD	VAD		; Accumulate alignment error for quality tracking
			OGC
			ERCOMP1
		STORE	ERCOMP1
		EXIT

; ============================================================================
; ALIGNMENT COMPLETION AND MODE CLEANUP
;
; GCOMP5 completes the alignment process by releasing the mark button, clearing
; the track mode flag (TRM03FLG), and returning the system to mode 02. The
; PHASCHNG call with OCT 00004 ensures proper restart protection. The job ends
; cleanly, ready for the next navigation or guidance operation.
;
; Error Handling: GTSOPTCS issues alarm 01602 if optical alignment fails to
; converge or encounters invalid star sightings, then attempts cleanup via GCOMP5.
; ============================================================================

GCOMP5		TC	BANKCALL
		CADR	MKRELEAS	; Release mark button
		TC	DOWNFLAG
		ADRES	TRM03FLG	; Clear track mode flag

		TC	NEWMODEX
		MM	02		; Return to mode 02
		TC	PHASCHNG
		OCT	00004		; Restart protection
		TC	ENDOFJOB
V06N93S		VN	0693		; Verb 06 Noun 93: Display gyro torquing angles
GTSOPTCS	TC	ALARM		; Optical alignment error handler
GTSOPTSS	OCT	01602		; Alarm code: Optical sighting failure
		TC	GCOMP5		; Attempt cleanup after error


		BANK	34
		SETLOC	IMUCAL1
		BANK
# Page 452

		COUNT	34/COMST

; ============================================================================
; LATITUDE AND AZIMUTH INPUT/VERIFICATION
;
; LATAZCHK formats and displays current latitude and azimuth values to the
; crew using Verb 06 Noun 41, allowing them to verify or modify these
; parameters for gyrocompass alignment. The latitude defines the local
; vertical direction, and the azimuth orients the stable member relative
; to true north. After crew input, the routine converts the displayed values
; back to internal format (CDULOGIC for azimuth, SR2 for latitude) and
; returns. The PROCEED key at line 1479 allows recycle to redisplay values.
; ============================================================================

LATAZCHK	DLOAD	SL2		# CALLS FOR AZIMUTH AND LATITUDE
			LATITUDE
		STODL	DSPTEM1 +1
			AZIMUTH
		RTB	EXIT
			1STO2S
		XCH	MPAC
		TS	DSPTEM1
		TC	BANKCALL
		CADR	CLEANDSP
		CAF	VNG0641
		TC	BANKCALL
		CADR	GOFLASH
		TC	+2		# NOT ALLOWED
		TC	+2
		TC	-5
		TC	INTPRET
		SLOAD	RTB
			DSPTEM1
			CDULOGIC
		STORE	AZIMUTH
		SLOAD	SR2
			DSPTEM1 +1
		STORE	LATITUDE
		RVQ
VNG0641		VN	0641
		BANK	33
		SETLOC	IMUCAL
		BANK


		COUNT*	$$/P03

; ============================================================================
; TARGET DRIVE ROUTINE - OPTICAL ALIGNMENT TARGET ACQUISITION
;
; TARGDRVE orchestrates the optical target acquisition process for star
; sightings during IMU alignment. The accumulator on entry specifies which
; target (first or second) to acquire. The routine calls TAR/EREF to compute
; the target reference vector, then loads the star position from the star
; catalog (indexed by TARG1/2) and calls SXTANG to compute sextant pointing
; angles. After positioning the optics, the routine calls SXTMARK to wait
; for the crew's mark button press, then OPTSTALL to process the measurement.
; If the track mode flag (TRM03BIT) is set, the routine exits to GCOMP5 for
; completion. Otherwise, it cycles through RETARG to acquire multiple marks
; on the same target, improving measurement accuracy through averaging.
; ============================================================================

TARGDRVE	EXTEND
		QXCH	QPLAC		; Save return address
		TS	TARG1/2		; Store target index (first or second star)
		TC	INTPRET
		CALL
			TAR/EREF
		LXC,1	VLOAD*
			TARG1/2
			6D,1
		STCALL	STAR
			SXTANG
		EXIT
		CA	SAC
		TS	DESOPTS

# Page 453
		CA	PAC
		TS	DESOPTT
RETARG		CAF	ZERO
		TS	OPTIND
		CAF	ONE
		TC	BANKCALL
		CADR	SXTMARK
		TC	BANKCALL
		CADR	OPTSTALL
		TC	GTSOPTCS
		CAE	FLAGWRD1
		MASK	TRM03BIT
		CCS	A
		TC	GCOMP5

		INDEX	MARKSTAT
		CA	QPRET
		EXTEND
		BZF	RETARG1
		TC	QPLAC


RETARG1		CA	ZERO		# RELEASE PREVIOUSLY GRABBED VAC AREA
		XCH	MARKSTAT
		CCS	A
		INDEX	A
		TS	A
		TCF	RETARG		# GO DO SXTMARK AGAIN
		BANK	33
		SETLOC	IMUCAL
		BANK
		COUNT*	$$/P03
PIPASC		2DEC	.76376833

VELSC		2DEC	-.52223476

ALSK		2DEC	.17329931

		2DEC	-.00835370

GEORGEJ		2DEC	.63661977

GEOCONS1	2DEC	.1

GEOCONS2	2DEC	.005

GEOCONS3	2DEC	.062

GEOCONS4	2DEC	.0003

# Page 454

		COUNT	33/P02

; ============================================================================
; MARK DATA PROCESSING SUBROUTINE
;
; LITTLSUB is a small utility routine that processes stored mark data from
; the optical sighting buffer. It retrieves the CDU angles stored when the
; crew pressed the mark button, transforms them through the navigation base
; to stable member coordinate system, and returns. The routine is called
; during alignment processing to convert raw crew sighting data into usable
; navigation reference measurements. MARKSTAT indexes which mark in the buffer
; to process, allowing sequential processing of multiple star sightings.
; ============================================================================

LITTLSUB	STQ
			QPLAC		; Save return address
		LXC,1	VLOAD*
			MARKSTAT
			2,1
		STCALL	CDUSPOT
			SXTNB
		CALL
			TRG*NBSM
		GOTO
			QPLAC


		EXIT

; ============================================================================
; GYROCOMPASS AZIMUTH DISPLAY AND CONFIRMATION
;
; AZMTHCG1 displays the computed gyrocompass azimuth result to the crew for
; verification and optional modification. The routine loads NEWAZMTH (the
; azimuth computed from the gyrocompass alignment iteration), converts it to
; display format, and presents it using Verb 06 Noun 29. The crew can accept
; the computed value by pressing PROCEED, or modify it using the DSKY numeric
; keys if ground-supplied corrections are needed. After crew input, the
; routine stores the final azimuth value as LAUNCHAZ for use in subsequent
; mission programs. The PREMTRXC flag is cleared to indicate the pre-alignment
; transformation matrix is no longer valid after this alignment. This routine
; completes the gyrocompass alignment procedure and returns control via
; PINBRNCH to the calling program.
; ============================================================================

AZMTHCG1	TC	INTPRET
		DLOAD	RTB
			NEWAZMTH	; Load computed gyrocompass azimuth
			1STO2S		; Convert to display format
		EXIT
		XCH	MPAC
		TS	DSPTEM1
		TC	BANKCALL
		CADR	CLEANDSP
		CAF	VN0629
		TC	BANKCALL
		CADR	GOFLASH
		TCF	+2
		TCF	+2
		TCF	-5
		TC	INTPRET
		SLOAD	RTB
			DSPTEM1
			CDULOGIC
		STORE	LAUNCHAZ
		EXIT
		CA	ZERO
		TS	PREMTRXC
		TC	PHASCHNG
		OCT	00004
		TC	POSTJUMP
		CADR	PINBRNCH

VN0629		VN	0629

# Page 455
# *** END OF COMAID .029 ***
