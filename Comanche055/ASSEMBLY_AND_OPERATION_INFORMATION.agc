# Copyright:	Public domain.
# Filename:	ASSEMBLY_AND_OPERATION_INFORMATION.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Mod history:	2009-05-05 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#
# This source code has been transcribed or otherwise adapted from digitized
# images of a hardcopy from the MIT Museum.  The digitization was performed
# by Paul Fjeld, and arranged for by Deborah Douglas of the Museum.  Many
# thanks to both.  The images (with suitable reduction in storage size and
# consequent reduction in image quality as well) are available online at
# www.ibiblio.org/apollo.  If for some reason you find that the images are
# illegible, contact me at info@sandroid.org about getting access to the
# (much) higher-quality images which Paul actually created.
#
# Notations on the hardcopy document read, in part:
#
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  10:28 APR. 1, 1969
#
#	This AGC program shall also be referred to as
#			Colossus 2A

# Page 2

; ============================================================================
; FILE: ASSEMBLY_AND_OPERATION_INFORMATION.agc
; MODULE: INFORMATION Subsystem
; MISSION PHASE: all-phases (assembly configuration)
;
; TL;DR: Master assembly directives and operational notes defining how Comanche
;        055 source files compile into executable flight software. Documents
;        subroutine call structure tables, memory bank organization philosophy,
;        operational modes, and yaYUL assembler targeting requirements that
;        differ from original YUL/GAP assemblers.
;
; COMMENT-ONLY READERS: This file explains how the Apollo software was built
;        from source code into the programs that flew to the Moon.
; CODE-ALONG READERS: Critical reference for understanding AGC memory banking,
;        assembly directives, and build process architecture.
; ============================================================================

# ASSEMBLY AND OPERATIONS INFORMATION
# TAGS FOR RELATIVE SETLOC AND BLANK BANK CARDS
# SUBROUTINE CALLS

; ============================================================================
; SECTION: SUBROUTINE CALL STRUCTURE TABLE
;
; This section provides an index of all major software modules in Comanche 055,
; organized by subsystem. The Command Module software is divided into six major
; subsystems, each containing multiple program modules that handle specific
; mission functions from launch through splashdown.
;
; COMMENT-ONLY READERS: Think of this as the table of contents for the Apollo
;        Command Module flight software. Each subsystem handles different parts
;        of the mission - from navigation and guidance to attitude control and
;        reentry.
;
; CODE-ALONG READERS: This organizational structure reflects the AGC memory
;        banking requirements. With only 36K words of fixed memory available,
;        modules are carefully placed in memory banks to optimize cross-bank
;        call overhead and minimize bank-switching during time-critical operations.
; ============================================================================
#
; --- COMERASE SUBSYSTEM ---
; Erasable (RAM) memory allocation. Defines the 2K words of read-write memory
; available for variables, computation scratch space, and runtime state.
;
#	COMERASE
#		ERASABLE ASSIGNMENTS
;
; --- COMAID SUBSYSTEM ---
; Interrupt handlers, restart logic, crew interface, and navigation support.
; This subsystem handles real-time interrupts, DSKY display/keyboard operations,
; IMU sensor management, and mission program support routines.
;
#	COMAID
#		INTERRUPT LEAD INS
#		T4RUPT PROGRAM
#		DOWNLINK LISTS
#		FRESH START AND RESTART
#		RESTART TABLES
#		SXTMARK
#		EXTENDED VERBS
#		PINBALL NOUN TABLES
#		CSM GEOMETRY
#		IMU COMPENSATION PACKAGE
#		PINBALL GAME  BUTTONS AND LIGHTS
#		R60,R62
#		ANGLFIND
#		GIMBAL LOCK AVOIDANCE
#		KALCMANU STEERING
#		SYSTEM TEST STANDARD LEAD INS
#		IMU CALIBRATION AND ALIGNMENT
;
; --- COMEKISS SUBSYSTEM ---
; Orbital navigation and targeting programs. Handles ground tracking data,
; Lambert targeting for orbital maneuvers, and stable orbit determination.
; Used during translunar coast, lunar orbit, and transearth phases.
;
#	COMEKISS
#		GROUND TRACKING DETERMINATION PROGRAM - P21
#		P34-P35, P74-P75
#		R31
#		P76
#		R30
#		STABLE ORBIT - P38-P39
;
; --- TROUBLE SUBSYSTEM ---
; Major mission programs including rendezvous navigation, service propulsion
; burns, IMU alignment, and atmospheric entry control. Contains the programs
; that execute during critical mission phases including translunar injection,
; lunar orbit insertion, rendezvous operations, and Earth reentry.
;
#	TROUBLE
#		P11
#		TPI SEARCH
#		P20-P25
#		P30,P37
#		P32-P33, P72-P73
#		P40-P47
#		P51-P53
#		LUNAR AND SOLAR EPHEMERIDES SUBROUTINES
#		P61-P67
#		SERVICER207
#		ENTRY LEXICON
#		REENTRY CONTROL
#		CM BODY ATTITUDE
#		P37,P70
#		S-BAND ANTENNA FOR CM
;
; --- TVCDAPS SUBSYSTEM ---
; Thrust Vector Control (TVC) and Digital Autopilot System. Controls engine
; gimbal positioning during SPS burns and manages RCS thruster firing for
; attitude control. Critical for precision orbital maneuvers and maintaining
; spacecraft orientation during coast phases.
;
#	TVCDAPS
#		TVCINITIALIZE
# Page 3
#		TVCEXECUTIVE
#		TVCMASSPROP
#		TVCRESTARTS
#		TVCDAPS
#		TVCSTROKETEST
#		TVCROLLDAP
#		MYSUBS
#		RCS-CSM DIGITAL AUTOPILOT
#		AUTOMATIC MANEUVERS
#		RCS-CSM DAP EXECUTIVE PROGRAMS
#		JET SELECTION LOGIC
#		CM ENTRY DIGITAL AUTOPILOT
;
; --- CHIEFTAN SUBSYSTEM ---
; Core operating system and mathematical foundation. Contains the Executive
; scheduler, Waitlist timer system, Interpreter virtual machine for high-level
; vector/matrix operations, coordinate transformation routines, orbital
; mechanics calculations, and fundamental utility functions. This is the
; foundational layer upon which all mission programs are built.
;
#	CHIEFTAN
#		DOWN-TELEMETRY PROGRAM
#		INTER-BANK COMMUNICATION
#		INTERPRETER
#		FIXED-FIXED CONSTANT POOL
#		INTERPRETIVE CONSTANTS
#		SINGLE PRECISION SUBROUTINES
#		EXECUTIVE
#		WAITLIST
#		LATITUDE LONGITUDE SUBROUTINES
#		PLANETARY INERTIAL ORIENTATION
#		MEASUREMENT INCORPORATION
#		CONIC SUBROUTINES
#		INTEGRATION INITIALIZATION
#		ORBITAL INTEGRATION
#		INFLIGHT ALIGNMENT ROUTINES
#		POWERED FLIGHT SUBROUTINES
#		TIME OF FREE FALL
#		STAR TABLES
#		AGC BLOCK TWO SELF-CHECK
#		PHASE TABLE MAINTENANCE
#		RESTARTS ROUTINE
#		IMU MODE SWITCHING ROUTINES
#		KEYRUPT, UPRUPT
#		DISPLAY INTERFACE ROUTINES
#		SERVICE ROUTINES
#		ALARM AND ABORT
#		UPDATE PROGRAM
#		RTB OP CODES
#
#
#       SYMBOL TABLE LISTING
#       UNREFERENCED SYMBOL LISTING
#       ERASABLE & EQUALS CROSS-REFERENCE TABLE
#       SUMMARY OF SYMBOL TABLE LISTINGS
#       MEMORY TYPE & AVAILABILITY DISPLAY
#       COUNT TABLE
#       PARAGRAPHS GENERATED FOR THIS ASSEMBLY
# Page 4
#       OCTAL LISTING
#       OCCUPIED LOCATIONS TABLE
#       SUBROS CALLED & PROGRAM STATUS

# Page 5

; ============================================================================
; TRANSITION: From Subroutine Organization to Crew Interface Documentation
;
; The following sections document the DSKY (Display and Keyboard) verb/noun
; system that astronauts used to communicate with the AGC. Verbs specify
; operations to perform (display data, load values, execute programs), while
; nouns specify which data to operate on (position, velocity, time, etc.).
;
; During the Apollo 11 mission, Neil Armstrong and Buzz Aldrin used these
; verb/noun combinations hundreds of times - requesting navigation data,
; initiating maneuvers, and monitoring spacecraft systems. The DSKY was their
; primary interface to the guidance computer throughout the journey from
; Earth to the Moon and back.
;
; COMMENT-ONLY READERS: This is like the command language astronauts used to
;        "talk" to the computer. Verb+Noun combinations let them ask questions
;        and give commands (e.g., V16N36 displayed time from ignition).
;
; CODE-ALONG READERS: Understanding the verb/noun architecture is essential
;        for tracing how crew procedures trigger specific software routines.
;        Each verb number maps to executable code in EXTENDED_VERBS.agc or
;        PINBALL_GAME_BUTTONS_AND_LIGHTS.agc.
; ============================================================================

# VERB LIST FOR CSM

; ============================================================================
; SECTION: VERB LIST (Regular and Extended)
;
; Verbs are numerical commands (V01-V99) that instruct the AGC what operation
; to perform. Regular Verbs (01-37) handle common display and data operations.
; Extended Verbs (40-99) control mission programs and specialized functions.
;
; Astronauts entered verbs by pressing the VERB key followed by two digits.
; The AGC would then typically prompt for a noun to specify which data to
; operate on. Some verbs operated standalone (e.g., V37 to change programs).
;
; VERB CATEGORIES:
; - Display verbs (V05-V06, V16): Show data on DSKY in various formats
; - Load verbs (V21-V25): Accept crew input to update navigation or targeting
; - Monitor verbs (V11, V16): Continuously update displays with changing data
; - Program verbs (V37, V70-V75): Terminate or change major mission programs
; - Test verbs (V34-V36): System self-test and hardware checkout functions
; ============================================================================

# REGULAR VERBS

# 00 NOT IN USE
# 01 DISPLAY OCTAL COMP 1 IN R1
# 02 DISPLAY OCTAL COMP 2 IN R1
# 03 DISPLAY OCTAL COMP 3 IN R1
# 04 DISPLAY OCTAL COMP 1,2 IN R1,R2
# 05 DISPLAY OCTAL COMP 1,2,3 IN R1,R2,R3
# 06 DISPLAY DECIMAL IN R1 OR R1,R2 OR R1,R2,R3
# 07 DISPLAY DP DECIMAL IN R1,R2 (TEST ONLY)
# 08
# 09
# 10
# 11 MONITOR OCTAL COMP 1 IN R1
# 12 MONITOR OCTAL COMP 2 IN R1
# 13 MONITOR OCTAL COMP 3 IN R1
# 14 MONITOR OCTAL COMP 1,2, IN R1,R2
# 15 MONITOR OCTAL COMP 1,2,3 IN R1,R2,R3
# 16 MONITOR DECIMAL IN R1 OR R1,R2 OR R1,R2,R3
# 17 MONITOR DP DECIMAL IN R1,R2 (TEST ONLY)
# 18
# 19
# 20
# 21 LOAD COMPONENT 1 INTO R1
# 22 LOAD COMPONENT 2 INTO R2
# 23 LOAD COMPONENT 3 INTO R3
# 24 LOAD COMPONENT 1,2 INTO R1,R2
# 25 LOAD COMPONENT 1,2,3 INTO R1,R2,R3
# 26
# 27 DISPLAY FIXED MEMORY
# 28
# 29
# 30 REQUEST EXECUTIVE
# 31 REQUEST WAITLIST
# 32 RECYCLE PROGRAM
# 33 PROCEED WITHOUT DSKY INPUTS
# 34 TERMINATE FUNCTION
# 35 TEST LIGHTS
# 36 REQUEST FRESH START
# 37 CHANGE PROGRAM (MAJOR MODE)
# 38
# 39

# Page 6

# EXTENDED VERBS

# 40 ZERO CDU-S
# 41 COARSE ALIGN CDU-S
# 42 FINE ALIGN IMU-S
# 43 LOAD IMU ATT ERROR METERS
# 44 SET   SURFACE FLAG
# 45 RESET SURFACE FLAG
# 46 ESTABLISH G+C CONTROL
# 47 MOVE LM STATE VECTOR INTO CM STATE VECTOR.
# 48 REQUEST DAP DATA LOAD ROUTINE (R03)
# 49 REQUEST CREW DEFINED MANEUVER ROUTINE (R62)
# 50 PLEASE PERFORM
# 51 PLEASE MARK
# 52 MARK ON OFFSET LANDING SITE
# 53 PLEASE PERFORM ALTERNATE LOS MARK
# 54 REQUEST RENDEZVOUS BACKUP SIGHTING MARK ROUTINE (R23)
# 55 INCREMENT AGC TIME (DECIMAL)
# 56 TERMINATE TRACKING (P20 + P25)
# 57 REQUEST RENDEZVOUS SIGHTING MARK ROUTINE (R21)
# 58 RESET STICK FLAG
# 59 PLEASE CALIBRATE
# 60 SET ASTRONAUT TOTAL ATTITUDE (N17) TO PRESENT ATTITUDE
# 61 DISPLAY DAP ATTITUDE ERROR
# 62 DISPLAY TOTAL ATTITUDE ERROR (WRT N22 (THETAD))
# 63 DISPLAY TOTAL ASTRONAUT ATTITUDE ERROR (WRT N17 (CPHIX))
# 64 REQUEST S-BAND ANTENNA ROUTINE
# 65 OPTICAL VERIFICATION OF PRELAUNCH ALIGNMENT
# 66 VEHICLES ARE ATTACHED.  MOVE THIS VEHICLE STATE TO OTHER VEHICLE.
# 67
# 68 CSM STROKE TEST ON
# 69 CAUSE RESTART
# 70 UPDATE LIFTOFF TIME
# 71 UNIVERSAL UPDATE-BLOCK  ADR
# 72 UNIVERSAL UPDATE-SINGLE ADR
# 73 UPDATE AGC TIME (OCTAL)
# 74 INITIALIZE ERASABLE DUMP VIA DOWNLINK
# 75 BACKUP LIFTOFF
# 76 SET PREFERRED ATTITUDE FLAG
# 77 RESET PREFERRED ATTITUDE FLAG
# 78 UPDATE PRELAUNCH AZIMUTH
# 79 REQUEST LUNAR LANDMARK SELECTION ROUTINE (R35)
# 80 UPDATE LEM STATE VECTOR
# 81 UPDATE CSM STATE VECTOR
# 82 REQUEST ORBIT PARAM DISPLAY (R30)
# 83 REQUEST REND  PARAM DISPLAY (R31)
# 84 START TARGET DELTA V (R32)
# 85 REQUEST RENDEZVOUS PARAMETER DISPLAY NO. 2 (R34)
# 86 REJECT RENDEZVOUS BACKUP SIGHTING MARK
# 87 SET VHF RANGE FLAG
# Page 7
# 88 RESET VHF RANGE FLAG
# 89 REQUEST RENDEZVOUS FINAL ATTITUDE ROUTINE (R63)
# 90 REQUEST RENDEZVOUS OUT OF PLANE DISPLAY ROUTINE (R36)
# 91 DISPLAY BANK SUM
# 92 OPERATE IMU PERFORMANCE TEST (P07)
# 93 ENABLE W MATRIX INITIALIZATION
# 94 PERFORM CYSLUNAR ATTITUDE MANEUVER (P23)
# 95 NO UPDATE OF EITHER STATE VECTOR (P20 OR P22)
# 96 TERMINATE INTEGRATION AND GO TO P00
# 97 PERFORM ENGINE FAIL PROCEDURE
# 98 ENABLE TRANSLUNAR INJECT
# 99 PLEASE ENABLE ENGINE

# Page 8

; ============================================================================
; SECTION: NOUN LIST (Normal and Mixed Nouns)
;
; Nouns are numerical data identifiers (N01-N99) that specify which information
; to display or modify. Each noun represents 1-3 components of related data
; (e.g., X/Y/Z coordinates, or hours/minutes/seconds of time).
;
; Nouns fall into two categories:
; - NORMAL NOUNS: Fixed data locations in memory (e.g., N36 = time from ignition)
; - MIXED NOUNS: Variable data depending on program context (e.g., N90 adapts
;                to current program's needs)
;
; DATA COMPONENT FORMATS:
; - 1COMP: Single value (altitude, velocity magnitude, angle)
; - 2COMP: Pair of values (latitude/longitude, option codes)
; - 3COMP: Triple of values (position vector XYZ, velocity vector, time HMS)
;
; SCALING AND DISPLAY:
; Each noun component has a specific scaling factor and decimal point position.
; For example, N36 displays time as HHH.HH hours, MMM.MM minutes, SSS.SS seconds.
; Position nouns use scaling appropriate for distances (nautical miles, feet).
;
; LOAD RESTRICTIONS:
; :NO LOAD: - Noun contains components that cannot be crew-loaded (computed values)
; :DEC ONLY: - Only decimal entry allowed (no octal). NO LOAD implies DEC ONLY.
;
; COMMENT-ONLY READERS: Think of nouns as "what" the astronauts wanted to see
;        or change - their position, velocity, fuel remaining, time to ignition.
;        Each number (N01-N99) meant something specific to the mission.
;
; CODE-ALONG READERS: Each noun maps to specific memory registers in erasable
;        memory. The mapping is defined in PINBALL_NOUN_TABLES.agc. Scaling
;        factors convert internal AGC fixed-point representation to human-
;        readable decimal values on the DSKY display.
; ============================================================================

# IN THE FOLLOWING NOUN LIST THE :NO LOAD: RESTRICTION MEANS THE NOUN
# CONTAINS AT LEAST ONE COMPONENT WHICH CANNOT BE LOADED, I.E. OF
# SCALE TYPE L (MIN/SEC) OR PP (2 INTEGERS).
# IN THIS CASE VERBS 24 AND 25 ARE NOT ALLOWED, BUT VERBS 21, 22 OR 23
# MAY BE USED TO LOAD ANY OF THE NOUN:S COMPONENTS WHICH ARE NOT OF THE
# ABOVE SCALE TYPES.
# THE :DEC ONLY: RESTRICTION MEANS ONLY DECIMAL OPERATION IS ALLOWED ON
# EVERY COMPONENT IN THE NOUN. (NOTE THAT :NO LOAD: IMPLIES :DEC ONLY:.)

# NORMAL NOUNS				   COMPONENTS	SCALE AND DECIMAL POINT		RESTRICTIONS

# 00	NOT IN USE
# 01	SPECIFY MACHINE ADDRESS (FRACTIONAL)	3COMP	.XXXXX FOR EACH
# 02	SPECIFY MACHINE ADDRESS (WHOLE)		3COMP	XXXXX. FOR EACH
# 03	SPECIFY MACHINE ADDRESS (DEGREES)	3COMP	XXX.XX DEG FOR EACH
# 04	SPARE
# 05	ANGULAR ERROR/DIFFERENCE		1COMP	XXX.XX DEG
# 06	OPTION CODE				2COMP	OCTAL ONLY FOR EACH
# LOADING NOUN 07 WILL SET OR RESET SELECTED BITS IN ANY ERASABLE REGISTER
# 07	ECADR OF WORD TO BE MODIFIED		3COMP	OCTAL ONLY FOR EACH
#	ONES FOR BITS TO BE MODIFIED
#	1 TO SET OR 0 TO RESET SELECTED BITS
# 08	ALARM DATA				3COMP	OCTAL ONLY FOR EACH
# 09	ALARM CODES				3COMP	OCTAL ONLY FOR EACH
# 10	CHANNEL TO BE SPECIFIED			1COMP	OCTAL ONLY
# 11	TIG OF CSI				3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 12	OPTION CODE				2COMP	OCTAL ONLY FOR EACH
#	 (USED BY EXTENDED VERBS ONLY)
# 13	TIG OF CDH				3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 14	SPARE
# 15	INCREMENT MACHINE ADDRESS		1COMP	OCTAL ONLY
# 16	TIME OF EVENT				3COMP	00XXX. HRS		DEC ONLY
#	 (USED BY EXTENDED VERBS ONLY)			000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 17	ASTRONAUT TOTAL ATTITUDE		3COMP	XXX.XX DEG FOR EACH
# 18	AUTO MANEUVER BALL ANGLES		3COMP	XXX.XX DEG FOR EACH
# 19	BYPASS ATTITUDE TRIM MANEUVER		3COMP	XXX.XX DEG FOR EACH
# 20	ICDU ANGLES				3COMP	XXX.XX DEG FOR EACH
# 21	PIPAS					3COMP	XXXXX. PULSES FOR EACH
# 22	NEW ICDU ANGLES				3COMP	XXX.XX DEG FOR EACH
# 23	SPARE
# 24	DELTA TIME FOR AGC CLOCK		3COMP	00XXX. HRS.		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 25	CHECKLIST				3COMP	XXXXX. FOR EACH
#	 (USED WITH PLEASE PERFORM ONLY)
# Page 9
# 26	PRIORITY/DELAY, ADRES, BBCON		3COMP	OCTAL ONLY FOR EACH
# 27	SELF TEST ON/OFF SWITCH			1COMP	XXXXX.
# 28	SPARE
# 29	XSM LAUNCH AZIMUTH			1COMP	XXX.XX DEG		DEC ONLY
# Page 10
# 30	TARGET CODES				3COMP	XXXXX. FOR EACH
# 31	TIME OF LANDING SITE			3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
# 							0XX.XX SEC
# 32	TIME FROM PERIGEE			3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 33	TIME OF IGNITION			3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 34	TIME OF EVENT				3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 35	TIME FROM EVENT				3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 36	TIME OF AGC CLOCK			3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 37	TIG OF TPI				3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 38	TIME OF STATE VECTOR			3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC
# 39	DELTA TIME FOR TRANSFER			3COMP	00XXX. HRS		DEC ONLY
#							000XX. MIN		MUST LOAD 3 COMPS
#							0XX.XX SEC

# Page 11

# MIXED NOUNS				   COMPONENTS	SCALE AND DECIMAL POINT	RESTRICTIONS
#
# 40	TIME FROM IGNITION/CUTOFF		3COMP	XXBXX  MIN/SEC		NO LOAD, DEC ONLY
#	VG,						XXXX.X FT/SEC
#	DELTA V (ACCUMULATED)				XXXX.X FT/SEC
# 41	TARGET	AZIMUTH,			2COMP	XXX.XX DEG
#		ELEVATION				XX.XXX DEG
# 42	APOGEE,					3COMP	XXXX.X NAUT MI		DEC ONLY
#	PERIGEE,					XXXX.X NAUT MI
#	DELTA V (REQUIRED)				XXXX.X FT/SEC
# 43	LATITUDE,				3COMP	XXX.XX DEG		DEC ONLY
#	LONGITUDE,					XXX.XX DEG
#	ALTITUDE					XXXX.X NAUT MI
# 44	APOGEE,					3COMP	XXXX.X NAUT MI		NO LOAD, DEC ONLY
#	PERIGEE,					XXXX.X NAUT MI
#	TFF						XXBXX  MIN/SEC
# 45	MARKS (VHF - OPTICS)			3COMP	+XXBXX			NO LOAD, DEC ONLY
#	TFI OF NEXT BURN				XXBXX  MIN/SEC
#	MGA						XXX.XX DEG
# 46	AUTOPILOT CONFIGURATION			2COMP	OCTAL ONLY FOR EACH
# 47	THIS VEHICLE WEIGHT			2COMP	XXXXX. LBS		DEC ONLY
#	OTHER VEHICLE WEIGHT				XXXXX. LBS
# 48	PITCH TRIM				2COMP	XXX.XX DEG		DEC ONLY
#	YAW TRIM,					XXX.XX   DEG
# 49	DELTA R					3COMP	XXXX.X NAUT MI		DEC ONLY
#	DELTA V						XXXX.X FT/SEC
#	VHF OR OPTICS CODE				XXXXX.
# 50	SPLASH ERROR,				3COMP	XXXX.X NAUT MI		NO LOAD, DEC ONLY
#	PERIGEE,					XXXX.X NAUT MI
#	TFF						XXBXX  MIN/SEC
# 51	S-BAND ANTENNA ANGLES PITCH		2COMP	XXX.XX DEG		DEC ONLY
#			       YAW			XXX.XX DEG
# 52	CENTRAL ANGLE OF ACTIVE VEHICLE		1COMP	XXX.XX DEG
# 53	RANGE,					3COMP	XXX.XX NAUT MI		DEC ONLY
#	RANGE RATE,					XXXX.X FT/SEC
#	PHI						XXX.XX DEG
# 54	RANGE,					3COMP	XXX.XX NAUT MI		DEC ONLY
#	RANGE RATE,					XXXX.X FT/SEC
#	THETA						XXX.XX DEG
# 55	PERIGEE CODE				3COMP	XXXXX.			DEC ONLY
#	ELEVATION ANGLE					XXX.XX DEG
#	CENTRAL ANGLE OF PASSIVE VEHICLE		XXX.XX DEG
# 56	REENTRY ANGLE,				2COMP	XXX.XX DEG		DEC ONLY
#	DELTA V						XXXXX. FT/SEC
# 57	DELTA R					1COMP	XXXX.X NAUT MI		DEC ONLY
# 58	PERIGEE ALT (POST TPI)			3COMP	XXXX.X NAUT MI		DEC ONLY
#	DELTA V TPI					XXXX.X FT/SEC
#	DELTA V TPF					XXXX.X FT/SEC
# 59	DELTA VELOCITY LOS			3COMP	XXXX.X FT/SEC FOR EA.	DEC ONLY
# 60	GMAX,					3COMP	XXX.XX G		DEC ONLY
# Page 12
#	VPRED,						XXXXX. FT/SEC
#	GAMMA EI					XXX.XX DEG
# 61	IMPACT LATITUDE,			3COMP	XXX.XX DEG		DEC ONLY
#	IMPACT LONGITUDE,				XXX.XX DEG
#	HEADS UP/DOWN					+/- 00001
# 62	INERTIAL VEL MAG (VI),			3COMP	XXXXX. FT/SEC		DEC ONLY
#	ALT RATE CHANGE (HDOT),				XXXXX. FT/SEC
#	ALT ABOVE PAD RADIUS (H)			XXXX.X NAUT MI
# 63	RANGE  297,431 TO SPLASH (RTGO),	3COMP	XXXX.X NAUT MI		NO LOAD, DEC ONLY
#	PREDICTED INERT VEL (VIO),			XXXXX. FT/SEC
#	TIME FROM 297,431 (TFE),			XXBXX  MIN/SEC
# 64	DRAG ACCELERATION,			3COMP	XXX.XX G		DEC ONLY
#	INERTIAL VELOCITY (VI),				XXXXX. FT/SEC
#	RANGE TO SPLASH					XXXX.X NAUT MI
# 65	SAMPLED AGC TIME			3COMP	00XXX. HRS		DEC ONLY
#	 (FETCHED IN INTERRUPT)				000XX. MIN              MUST LOAD 3 COMPS
#							0XX.XX SEC
# 66	COMMAND BANK ANGLE (BETA),		3COMP	XXX.XX DEG		DEC ONLY
#	CROSS RANGE ERROR,				XXXX.X NAUT MI
#	DOWN RANGE ERROR				XXXX.X NAUT MI
# 67	RANGE TO TARGET,			3COMP	XXXX.X NAUT MI		DEC ONLY
#	PRESENT LATITUDE,				XXX.XX DEG
#	PRESENT LONGITUDE				XXX.XX DEG
# 68	COMMAND BANK ANGLE (BETA),		3COMP	XXX.XX DEG		DEC ONLY
#	INERTIAL VELOCITY (VI),				XXXXX. FT/SEC
#      ALT RATE CHANGE (RDOT)				XXXXX. FT/SEC
# 69   BETA					3COMP	XXX.XX DEG		DEC ONLY
#      DL						XXX.XX G
#      VL						XXXXX. FT/SEC
# 70	STAR CODE,				3COMP	OCTAL ONLY
#	LANDMARK DATA,					OCTAL ONLY
#	HORIZON DATA					OCTAL ONLY
# 71	STAR CODE				3COMP	OCTAL ONLY
#	LANDMARK DATA					OCTAL ONLY
#	HORIZON DATA					OCTAL ONLY
# 72	DELT ANG				3COMP	XXX.XX DEG		DEC ONLY
# 73	ALTITUDE				3COMP	XXXXXB. NAUT MI
#	VELOCITY					XXXXX.  FT/SEC
#	FLIGHT PATH ANGLE				XXX.XX  DEG
# 74	COMMAND BANK ANGLE (BETA)		3COMP	XXX.XX DEG
#	INERTIAL VELOCITY (VI)				XXXXX. FT/SEC
#	DRAG ACCELERATION				XXX.XX G
# 75	DELTA ALTITUDE CDH			3COMP	XXXX.X NAUT MI		NO LOAD, DEC ONLY
#	DELTA TIME (CDH-CSI OR TPI-CDH)			XXBXX  MIN/SEC
#	DELTA TIME (TPI-CDH OR TPI-NOMTPI)		XXBXX  MIN/SEC
# 76	SPARE
# 77	SPARE
# 78	SPARE
# 79	SPARE
# 80	TIME FROM IGNITION/CUTOFF		3COMP	XXBXX  MIN/SEC		NO LOAD, DEC ONLY
# Page 13
#	VG						XXXXX. FT/SEC
#	DELTA V (ACCUMULATED)				XXXXX. FT/SEC
# 81	DELTA V (LV)				3COMP	XXXX.X FT/SEC FOR EACH	DEC ONLY
# 82	DELTA V (LV)				3COMP	XXXX.X FT/SEC FOR EACH	DEC ONLY
# 83	DELTA V (BODY)				3COMP	XXXX.X FT/SEC FOR EACH	DEC ONLY
# 84	DELTA V (OTHER VEHICLE)			3COMP	XXXX.X FT/SEC FOR EACH	DEC ONLY
# 85	VG (BODY)				3COMP	XXXX.X FT/SEC FOR EACH	DEC ONLY
# 86	DELTA V(LV)				3COMP	XXXXX. FT/SEC FOR EACH	DEC ONLY
# 87	MARK DATA	SHAFT,			2COMP	XXX.XX DEG
#			TRUNION				XX.XXX DEG
# 88	HALF UNIT SUN OR PLANET VECTOR		3COMP	.XXXXX FOR EACH		DEC ONLY
# 89	LANDMARK	LATITUDE,		3COMP	XX.XXX DEG		DEC ONLY
#			LONGITUDE/2,			XX.XXX DEG
#			ALTITUDE			XXX.XX NAUT MI
# 90	Y					3COMP	XXX.XX NM		DEC ONLY
#	Y DOT						XXXX.X FPS
#	PSI						XXX.XX DEG
# 91	OCDU ANGLES	SHAFT,			2COMP	XXX.XX DEG
#			TRUNION				XX.XXX DEG
# 92	NEW OPTICS ANGLES	SHAFT,		2COMP	XXX.XX DEG
#				TRUNION			XX.XXX DEG
# 93	DELTA GYRO ANGLES			3COMP	XX.XXX DEG FOR EACH
# 94	NEW OPTICS ANGLES	SHAFT		2COMP	XXX.XX DEG
#				TRUNNION		XX.XXX DEG
# 95	PREFERRED ATTITUDE ICDU ANGLES		3COMP	XXX.XX FOR FOR EACH
# 96	+X-AXIS ATTITUDE ICDU ANGLES		3COMP	XXX.XX DEG FOR EACH
# 97	SYSTEM TEST INPUTS			3COMP	XXXXX. FOR EACH
# 98	SYSTEM TEST RESULTS AND INPUTS		3COMP	XXXXX.
#							.XXXXX
#							XXXXX.
# 99	RMS IN POSITION				3COMP	XXXXX.FT	        DEC ONLY
#	RMS IN VELOCITY					XXXX.X FT/SEC
#	RMS OPTION					XXXXX.

# Page 14

; ============================================================================
; SECTION: REGISTERS AND SCALING FOR NORMAL AND MIXED NOUNS
;
; This section maps each noun to its specific erasable memory register
; locations and scaling factors. Understanding this mapping is critical for
; both mission operations and software maintenance.
;
; MEMORY ARCHITECTURE CONTEXT:
; The AGC has only 2K words of erasable (RAM) memory, shared between all
; mission programs, navigation state, guidance parameters, and display buffers.
; Every register location is precious and carefully allocated. The register
; assignments shown here were finalized after extensive analysis to optimize
; memory usage while maintaining real-time performance.
;
; REGISTER NAMING CONVENTIONS:
; - Single letter codes (TSTRT, DSPTEM1, CDUX, etc.) are symbolic names for
;   specific erasable memory addresses
; - Register locations are defined in ERASABLE_ASSIGNMENTS.agc
; - Some registers are shared between multiple nouns depending on program phase
;
; SCALING FACTOR EXPLANATION:
; AGC lacks floating-point hardware, so all numbers use fixed-point arithmetic
; with predefined scaling. Each noun component has a scale factor indicating
; the physical unit per AGC internal count. For example:
; - Position scale "28" means bit 1 of high register = 2^28 centimeters
; - Velocity scale "7" means bit 1 = 2^7 meters/centisecond
; - Time scales use special formats (HMS, decimal hours, etc.)
;
; WHY SCALING MATTERS:
; Scaling choices balance precision versus range. A position scaled at 2^29
; meters can represent cislunar distances while maintaining meter-level
; precision. Velocity scaled at 2^7 m/cs covers orbital speeds while preserving
; sub-meter/second precision. These scaling decisions were fundamental to
; AGC's ability to navigate accurately with 15-bit signed arithmetic.
;
; DOUBLE PRECISION (DP) VALUES:
; Many nouns use two consecutive registers for double precision (30 bits of
; data + sign). The "high register" contains the most significant bits, the
; "low register" contains the least significant bits. This extends range and
; precision beyond single 15-bit words.
;
; COMMENT-ONLY READERS: This section shows where in computer memory each piece
;        of mission data lived - position, velocity, time, fuel, angles. The
;        Apollo computer had very limited memory, so every location was
;        carefully planned and reused when possible.
;
; CODE-ALONG READERS: Cross-reference these register assignments with
;        ERASABLE_ASSIGNMENTS.agc to see the complete memory map. Note how
;        registers are reused (e.g., DSPTEM1/2/3 serve multiple nouns). The
;        scaling factors here must match the scaling used in all computational
;        code that writes to these registers.
; ============================================================================

# REGISTERS AND SCALING  FOR NORMAL NOUNS
#
# NOUN	        REGISTER	SCALE TYPE
#
# 00	NOT IN USE
# 01	SPECIFY ADDRESS		B
# 02	SPECIFY ADDRESS		C
# 03	SPECIFY ADDRESS		D
# 04	SPARE
# 05		DSPTEM1		H
# 06		OPTION1		A
# 07		XREG		A
# 08		ALMCADR		A
# 09		FAILREG		A
# 10	SPECIFY CHANNEL		A
# 11		TCSI		K
# 12		OPTIONX		A
# 13		TCDH		K
# 14	SPARE
# 15	INCREMENT ADDRESS	A
# 16		DSPTEMX		C
# 17		CPHIX		D
# 18		THETAD		D
# 19		THETAD		D
# 20		CDUX		D
# 21		PIPAX		C
# 22		THETAD		D
# 23	SPARE
# 24		DSPTEM2 +1	K
# 25		DSPTEM1		C
# 26		DSPTEM1		A
# 27		SMODE		C
# 28	SPARE
# 29		DSPTEM1		D
# 30		DSPTEM1		C
# 31		DSPTEM1		K
# 32		-TPER		K
# 33		TIG		K
# 34		DSPTEM1		K
# 35		TTOGO		K
# 36		TIME2		K
# 37		TTPI		K
# 38		TET		K
# 39		T3TOT4		K

# Page 15

# REGISTERS AND SCALING FOR MIXED NOUNS
#
# NOUN	COMP	REGISTER	SCALE TYPE
#
# 40	1	TTOGO		L
#	2	VGDISP		S
#	3	DVTOTAL		S
# 41	1	DSPTEM1		D
#	2	DSPTEM1 +1	E
# 42	1	HAPO		Q
#	2	HPER		Q
#	3	VGDISP		S
# 43	1	LAT		H
#	2	LONG		H
#	3	ALT		Q
# 44	1	HAPOX		Q
#	2	HPERX		Q
#	3	TFF		L
# 45	1	VHFCNT		PP
#	2	TTOGO		L
#	3	+MGA		H
# 46	1	DAPDATR1	A
#	2	DAPDATR2	A
# 47	1	CSMMASS		KK
#	2	LEMMASS		KK
# 48	1	PACTOFF		FF
#	2	YACTOFF		FF
# 49	1	N49DISP		Q
#	2	N49DISP +2	S
#	3	N49DISP +4	C
# 50	1	RSP-RREC	LL
#	2	HPERX		Q
#	3	TFF		L
# 51	1	RHOSB		H
#	2	GAMMASB		H
# 52	1	ACTCENT		H
# 53	1	RANGE		JJ
#	2	RRATE		S
# 	3	RTHETA		H
# 54	1	RANGE		JJ
#	2	RRATE		S
# 	3	RTHETA		H
# 55	1	NN1		C
# 	2	ELEV		H
#	3	CENTANG		H
# 56	1	RTEGAM2D	H
#	2	RTEDVD		P
# 57	1	DELTAR		Q
# 58	1	POSTTPI		Q
#	2	DELVTPI		S
# Page 16
#	3	DELVTPF		S
# 59	1	DVLOS		S
#	2	DVLOS +2	S
#	3	DVLOS +4	S
# 60	1	GMAX		T
#	2	VPRED		P
#	3	GAMMAEI		H
# 61	1	LAT(SPL)	H
#	2	LNG(SPL)	H
#	3	HEADSUP		C
# 62	1	VMAGI		P
#	2	HDOT		P
#	3	ALTI		Q
# 63	1	RTGO		LL
#	2	VIO		P
#	3	TTE		L
# 64	1	D		MM
#	2	VMAGI		P
#	3	RTGON64		LL
# 65	1	SAMPTIME	K
#	2	SAMPTIME	K
#	3	SAMPTIME	K
# 66	1	ROLLC		H
#	2	XRNGERR		VV
#	3	DNRNGERR	LL
# 67	1	RTGON67		LL
#	2	LAT		H
#	3	LONG		H
# 68	1	ROLLC		H
#	2	VMAGI		P
#	3	RDOT		UU
# 69	1	ROLLC		H
#	2	Q7		MM
#	3	VL		UU
# 70	1	STARCODE	A
#	2	LANDMARK	A
#	3	HORIZON		A
# 71	1	STARCODE	A
#	2	LANDMARK	A
#	3	HORIZON		A
# 72	1	THETZERO	H
# 73	1	P21ALT		Q (MEMORY/100 TO DISPLAY TENS N.M.)
#	2	P21VEL		P
#	3	P21GAM		H
# 74	1	ROLLC		H
#	2	VMAGI		P
#	3	D		MM
# 75	1	DIFFALT		Q
#	2	T1TOT2		L
#	3	T2TOT3		L
# Page 17
# 76	SPARE
# 77	SPARE
# 78	SPARE
# 79	SPARE
# 80	1	TTOGO		L
#	2	VGDISP		P
#	3	DVTOTAL		P
# 81	1	DELVLVC		S
#	2	DELVLVC +2	S
#	3	DELVLVC +4	S
# 82	1	DELVLVC		S
#	2	DELVLVC +2	S
#	3	DELVLVC +4	S
# 83	1	DELVIMU		S
#	2	DELVIMU +2	S
#	3	DELVIMU +4	S
# 84	1	DELVOV		S
#	2	DELVOV +2	S
#	3	DELVOV +4	S
# 85	1	VGBODY		S
#	2	VGBODY +2	S
#	3	VGBODY +4	S
# 86	1	DELVLVC		P
#	2	DELVLVC +2	P
#	3	DELVLVC +4	P
# 87	1	MRKBUF1 +3	D
#	2	MRKBUF1 +5	J
# 88	1	STARSAV3	ZZ
#	2	STARSAV3 +2	ZZ
#	3	STARSAV3 +4	ZZ
# 89	1	LANDLAT		G
#	2	LANDLONG	G
#	3	LANDALT		JJ
# 90	1	RANGE		JJ
#	2	RRATE		S
#	3	RTHETA		H
# 91	1	CDUS		D
#	2	CDUT		J
# 92	1	SAC		D
#	2	PAC		J
# 93	1	OGC		G
#	2	OGC +2		G
#	3	OGC +4		G
# 94	1	MRKBUF1 +3	D
#	2	MRKBUF1 +5	J
# 95	1	PRAXIS		D
#	2	PRAXIS +1	D
#	3	PRAXIS +2	D
# 96	1	CPHIX		D
#	2	CPHIX +1	D
# Page 18
#	3	CPHIX +2	D
# 97	1	DSPTEM1		C
#	2	DSPTEM1 +1	C
# 	3	DSPTEM1 +2	C
# 98	1	DSPTEM2		C
#	2	DSPTEM2 +1	B
#	3	DSPTEM2 +2	C
# 99	1	WWPOS		XX
#	2	WWVEL		YY
#	3	WWOPT		C

# Page 19

; ============================================================================
; SECTION: NOUN SCALES AND FORMATS - Complete Reference
;
; This critical reference section defines the 40+ scale types (A through YY)
; used throughout the AGC display system. Each scale type specifies:
; - Physical units (degrees, meters, feet, seconds, etc.)
; - Decimal format shown on DSKY (where decimal point appears)
; - AGC internal format (how bits map to physical values)
; - Precision available (what's the smallest displayable increment)
;
; SCALE TYPE CATEGORIES:
; - Generic types (A-C): Octal, fractional, whole numbers
; - Angular types (D-K): Degrees in various formats and precisions
; - Linear types (M-P): Distance in feet or nautical miles
; - Velocity types (Q-T): Speed in feet/second or meters/centisecond
; - Time types (U-L): Hours, minutes, seconds in various combinations
; - Specialized types: Propellant mass, gimbal angles, option codes, etc.
;
; FIXED-POINT REPRESENTATION:
; All scale types use AGC's fixed-point arithmetic. The "BIT 1 = 2^N UNITS"
; notation indicates the scaling factor - the physical value represented by
; the least significant data bit (bit 1). For example:
; - "BIT 1 = 2^-14 UNITS" means bit 1 = 1/16384 of a unit
; - "BIT 1 = 2^28 CENTIMETERS" means bit 1 = 268,435,456 cm (2,684 km)
;
; DECIMAL FORMAT NOTATION:
; - "XXXXX." means whole number with decimal after all digits (12345.)
; - ".XXXXX" means fraction with decimal before all digits (.12345)
; - "XXX.XX" means decimal point in middle (123.45)
; - "±XXXXX" means signed value with explicit plus/minus display
;
; PRECISION AND RANGE TRADEOFF:
; Each scale type represents an engineering decision balancing measurement
; precision against maximum representable value. A 15-bit signed word can
; represent -16384 to +16383 counts. Scaling determines what physical range
; this covers. Fine precision (like scale D: 1 degree = 180 counts) gives
; accurate angle measurement. Coarse precision (like scale 28: position scaled
; at 2^28 cm per count) allows cislunar distances but sacrifices sub-kilometer
; resolution.
;
; WHY THIS MATTERS FOR OPERATIONS:
; During Apollo 11, when Armstrong and Aldrin requested V16N36 (time from
; ignition), the AGC retrieved values from registers, applied scale type L
; formatting (minutes and seconds), and displayed "MM.SS" on the DSKY. When
; they loaded a target address with V21N18, the DSKY converted their decimal
; input to internal fixed-point using scale B. Every number exchange between
; crew and computer flowed through these scale type conversions.
;
; COMMENT-ONLY READERS: This section is like a codebook translating between
;        what astronauts saw on the display (feet, degrees, minutes) and how
;        the computer internally stored those values as binary numbers. Each
;        "scale type" letter code defined a specific conversion rule.
;
; CODE-ALONG READERS: These scale type definitions are referenced throughout
;        PINBALL_NOUN_TABLES.agc, DISPLAY_INTERFACE_ROUTINES.agc, and all code
;        that formats data for DSKY output or accepts DSKY input. The scaling
;        math (multiply, shift, round) happens in display interface routines.
;        Understanding these scales is essential for tracing data flow from
;        computational routines through display formatting to crew visibility.
; ============================================================================

# NOUN SCALES AND FORMATS
#
# -SCALE TYPE-				 PRECISION
# UNITS			DECIMAL FORMAT		--	AGC FORMAT
# ------------		--------------		--	----------
#
# -A-
# OCTAL			XXXXX			SP	OCTAL
#
# -B-								 -14
# FRACTIONAL		.XXXXX			SP	BIT 1 = 2    UNITS
#			(MAX .99996)
#
# -C-
# WHOLE			XXXXX.			SP	BIT 1 = 1 UNIT
#			(MAX 16383.)
#
# -D-								     15
# CDU DEGREES		XXX.XX DEGREES		SP	BIT 1 = 360/2   DEGREES
#			(MAX 359.99)			(USES 15 BITS FOR MAGNI-
#							 TUDE AND 2-S COMP.)
#
# -E-								    14
# ELEVATION DEGREES	XX.XXX DEGREES		SP	BIT 1 = 90/2   DEGREES
#			(MAX 89.999)
#
# -F-								     14
# DEGREES (180)		XXX.XX DEGREES		SP	BIT 1 = 180/2   DEGREES
#			(MAX 179.99)
#
# -G-
# DP DEGREES(90)	XX.XXX DEGREES		DP	BIT 1 OF LOW REGISTER =
#							     28
#							360/2   DEGREES
#
# -H-
# DP DEGREES (360)	XXX.XX DEGREES		DP	BIT 1 OF LOW REGISTER =
#			        			     28
#			(MAX 359.99)			360/2   DEGREES
#
# -J-								    15
# Y OPTICS DEGREES	XX.XXX DEGREES		SP	BIT 1 = 90/2   DEGREES
#			(BIAS OF 19.775			(USES 15 BITS FOR MAGNI-
#			DEGREES ADDED FOR		TUDE AND 2-S COMP.)
#			DISPLAY, SUBTRACTED
#			FOR LOAD.)
#			NOTE: NEGATIVE NUM-
#			BERS CANNOT BE
#			LOADED.
#
# -K-
# Page 20
# TIME (HR, MIN, SEC)	00XXX. HR		DP	BIT 1 OF LOW REGISTER =
#			000XX. MIN			  -2
#			0XX.XX SEC			10   SEC
#			(DECIMAL ONLY.
#			MAX MIN COMP=59
#			MAX SEC COMP=59.99
#			MAX CAPACITY=745 HRS
#				      39 MINS
#				      14.55 SECS.
#			WHEN LOADING, ALL 3
#			COMPONENTS MUST BE
#			SUPPLIED.)
#
# -L-
# TIME (MIN/SEC)	XXBXX MIN/SEC		DP	BIT 1 OF LOW REGISTER =
#			(B IS A BLANK			  -2
#			POSITION, DECIMAL		10   SEC
#			ONLY, DISPLAY OR
#			MONITOR ONLY. CANNOT
#			BE LOADED.
#			MAX MIN COMP=59
#			MAX SEC COMP=59
#			VALUES GREATER THAN
#			59 MIN 59 SEC
#			ARE DISPLAYED AS
#			59 MIN 59 SEC.)
#
# -M-								  -2
# TIME (SEC)		XXX.XX SEC		SP	BIT 1 = 10   SEC
#			(MAX 163.83)
#
# -N-
# TIME(SEC) DP		XXX.XX SEC		DP	BIT 1 OF LOW REGISTER =
#							  -2
#							10   SEC
#
# -P-
# VELOCITY 2		XXXXX. FEET/SEC		DP	BIT 1 OF HIGH REGISTER =
#			(MAX 41994.)			 -7
#							2   METERS/CENTI-SEC
#
# -Q-
# POSITION 4		XXXX.X NAUTICAL MILES	DP	BIT 1 OF LOW REGISTER =
#							2 METERS
#
# -S-
# VELOCITY 3		XXXX.X FT/SEC		DP	BIT 1 OF HIGH REGISTER =
#							 -7
#							2   METERS/CENTI-SEC
# Page 21
# -T-								  -2
# G			XXX.XX G		SP	BIT 1 = 10   G
#			(MAX 163.83)
#
# -FF-
# TRIM DEGREES		XXX.XX DEG.		SP	LOW ORDER BIT = 85.41 SEC
#			(MAX 388.69)			OF ARC
#
# -GG-
# INERTIA		XXXXXBB. SLUG FT SQ	SP	FRACTIONAL PART OF
#			(MAX 07733BB.)			 20     2
#							2   KG M
#
# -II-									    20
# THRUST MOMENT		XXXXXBB. FT LBS		SP	FRACTIONAL PART OF 2
#			(MAX 07733BB.)			NEWTON METER
#
# -JJ-
# POSITION5		XXX.XX NAUT MI		DP	BIT 1 OF LOW REGISTER =
#							2 METERS
#
# -KK-									    16
# WEIGHT2		XXXXX. LBS		SP	FRACTIONAL PART OF 2   KG
#
# -LL-
# POSITION6		XXXX.X NAUT MI		DP	BIT 1 OF LOW REG =
#									    -28
#							(6,373,338)(2(PI))X2
#							-----------------------
#								 1852
#							NAUT. MI.
#
# -MM-
# DRAG ACCELERATION	XXX.XX G		DP	BIT 1 OF LOW REGISTER =
#			MAX (024.99)			    -28
#							25X2    G
#
# -PP-
# 2 INTEGERS		+XXBYY			DP	BIT 1 OF HIGH REGISTER =
#			(B IS A BLANK			 1 UNIT OF XX
#			POSITION.  DECIMAL		BIT 1 OF LOW REGISTER =
#			ONLY, DISPLAY OR		 1 UNIT OF YY
#			MONITOR ONLY. CANNOT		(EACH REGISTER MUST
#			BE LOADED.)                     CONTAIN A POSITIVE INTEGER
#			(MAX 99B99)                      LESS THAN 100)
#
# -UU-
# VELOCITY/2VS		XXXXX. FEET/SEC		DP	FRACTIONAL PART OF
#			(MAX 51532.)			2VS FEET/SEC
#							(VS = 25766.1973)
# Page 22
# -VV-
# POSITION8		XXXX.X NAUT MI		DP	BIT 1 OF LOW REGISTER =
#									 -28
#							4 X 6,373,338 X 2
#							--------------------
#							      1852
#							NAUT MI.
#
# -XX-
# POSITION 9		XXXXX. FEET		DP	BIT 1 OF LOW REGISTER =
#							 -9
#							2   METERS
#
# -YY-
# VELOCITY 4		XXXX.X FEET/SEC		DP	FRACTIONAL PART OF
#			(MAX 328.0)			METERS/CENTI-SEC
#
# -ZZ-
# DP FRACTIONAL		.XXXXX			DP	BIT 1 OF HIGH REGISTER =
#							 -14
#							2    UNITS

# THAT-S ALL ON THE NOUNS.

# Page 23

; ============================================================================
; SECTION: ALARM CODES - Program Fault Detection and Recovery
;
; This section documents the complete catalog of program alarm codes used
; throughout Comanche 055 (Command Module flight software). Program alarms
; are the AGC's fault detection and annunciation system, alerting the crew
; to software-detected anomalies while allowing mission continuation.
;
; ALARM SYSTEM ARCHITECTURE:
; When software detects an error condition (invalid sensor data, computational
; overflow, timing violation, etc.), it invokes the ALARM routine with a
; 5-digit octal code. The ALARM routine:
; 1. Displays the alarm code on the DSKY with flashing PROG light
; 2. Records the alarm in telemetry for ground analysis
; 3. Logs the alarm in erasable memory alarm history
; 4. Continues program execution (non-fatal) or initiates abort logic (fatal)
;
; ALARM CODE STRUCTURE:
; Five-digit octal codes (00110 through 77777) identify specific fault
; conditions. The code assignment reflects:
; - Subsystem origin (navigation, guidance, control, display, etc.)
; - Severity level (informational, warning, critical)
; - Recovery action required (crew intervention, automatic, abort)
;
; HISTORICAL CONTEXT - APOLLO 11 MISSION:
; During Apollo 11's lunar landing on July 20, 1969, program alarms became
; mission-critical. At mission time 102:38:26, the Lunar Module's AGC issued
; alarm 1202 (executive overflow - too many jobs queued). This occurred
; because the rendezvous radar was inadvertently left on, flooding the
; computer with unnecessary tracking data while simultaneously executing
; landing guidance. Flight controller Steve Bales and backroom engineer Jack
; Garman recognized 1202 as non-critical (the AGC's restart system would
; recover), gave a "GO" decision, and the landing continued successfully.
;
; The 1202 alarm repeated four times during descent. Without understanding
; the alarm system's design - that it protected against overload while
; maintaining critical functions - mission control might have aborted. The
; alarm codes documented in this file represent the AGC's fault tolerance
; strategy that enabled the first lunar landing despite computer overload.
;
; ALARM vs ABORT:
; Most alarm codes are informational or recoverable warnings. The AGC
; continues operation after displaying the alarm. Critical alarms (like
; IMU failures or guidance system faults) may trigger abort programs that
; separate the spacecraft from the lunar surface or terminate powered flight.
; The distinction between recoverable alarms and abort conditions was
; carefully engineered into each alarm code's handling logic.
;
; CREW RESPONSE PROCEDURES:
; When an alarm illuminates the DSKY PROG light:
; 1. Crew notes the 5-digit alarm code
; 2. Presses KEY REL to acknowledge and clear the flashing display
; 3. Consults checklist or ground control for alarm meaning
; 4. Executes corrective action if required (switch setting, data entry)
; 5. Continues monitoring for alarm recurrence
;
; During Apollo 11's descent, Armstrong and Aldrin acknowledged each 1202
; alarm with KEY REL, received "GO" from Houston, and continued the landing.
; This interaction between alarm system, crew response, and ground control
; decision-making exemplified human-computer cooperation under pressure.
;
; ALARM CODE TABLE FORMAT:
; Each entry specifies:
; - CODE: 5-digit octal alarm number
; - TYPE: Error category (computational, sensor, timing, etc.)
; - SET BY: Which software module detects and issues this alarm
; - ALARM ROUTINE: How the alarm system processes this code
;
; TELEMETRY AND GROUND MONITORING:
; Every alarm triggers downlink telemetry, allowing Mission Control to track
; AGC health and anomalies in real-time. The telemetry includes alarm code,
; time of occurrence, program executing when alarm occurred, and system state.
; This data was critical for Apollo 11's ground team to diagnose the 1202
; alarms and authorize landing continuation.
;
; RESTART PROTECTION:
; Many alarms coordinate with the AGC's restart system. If an alarm indicates
; computational overload or timing violation, the restart system preserves
; mission-critical state and restarts interrupted programs from protected
; phases. This restart capability, integrated with the alarm system, provided
; the fault tolerance that saved Apollo 11's landing when 1202 alarms occurred.
;
; COMMENT-ONLY READERS: Program alarm codes are like warning lights in a car,
;        but much more sophisticated. When the Apollo computer detected a
;        problem, it displayed a 5-digit code to alert the astronauts. During
;        Apollo 11's lunar landing, alarm code 1202 flashed four times because
;        the computer was overloaded with data. Mission Control understood the
;        alarm meant "I'm busy but still working" rather than "I'm failing,"
;        so they allowed Armstrong to continue the landing. This alarm system
;        was one reason the first Moon landing succeeded despite unexpected
;        computer problems.
;
; CODE-ALONG READERS: Cross-reference these alarm codes with ALARM_AND_ABORT.agc
;        which contains the ALARM display routine and alarm handling logic.
;        EXECUTIVE.agc and WAITLIST.agc contain the code that generates 1201/
;        1202 alarms when job queues or task lists overflow. Each module that
;        issues alarms (SXTMARK, IMU_CALIBRATION, POWERED_FLIGHT_SUBROUTINES,
;        etc.) calls TC ALARM with the appropriate 5-digit code in the A
;        register. Understanding alarm flow requires tracing from fault
;        detection point → ALARM routine → DSKY display → telemetry downlink
;        → crew/ground response.
; ============================================================================

# 		ALARM CODES FOR 504

# 		REPORT DEFICIENCIES TO JOHN SUTHERLAND @ MIT 617-864-6900 X1458

# *9		*18						*60			                *25  COLUMN
#
# CODE       *	TYPE						SET BY			                ALARM ROUTINE
#
# 00110		NO MARK SINCE LAST MARK REJECT			SXTMARK			                ALARM
# 00112		MARK NOT BEING ACCEPTED				SXTMARK			                ALARM
# 00113		NO INBITS					SXTMARK			                ALARM
# 00114		MARK MADE BUT NOT DESIRED			SXTMARK			                ALARM
# 00115		OPTICS TORQUE REQUESTWITH SWITCH NOT AT	        EXT VERB OPTICS CDU	                ALARM
# 		 CGC
# 00116		OPTICS SWITCH ALTERED BEFORE 15 SEC ZERO	T4RUPT			                ALARM
#		 TIME ELAPSED.
# 00117		OPTICS TORQUE REQUEST WITH OPTICS NOT		EXT VERB OPTICS CDU	                ALARM
#		 AVAILABLE (OPTIND=-0)
# 00120		OPTICS TORQUE REQUEST WITH OPTICS		T4RUPT			                ALARM
#		 NOT ZEROED
# 00121		CDUS NO GOOD AT TIME OF MARK			SXTMARK			                ALARM
# 00122		MARKING NOT CALLED FOR				SXTMARK			                ALARM
# 00124		P17 TPI SEARCH - NO SAFE PERICTR HERE.		TPI SEARCH		                ALARM
# 00205		BAD PIPA READING				SERVICER		                ALARM
# 00206		ZERO ENCODE NOT ALLOWED WITH COARSE ALIGN	IMU MODE SWITCHING	                ALARM
# 		 + GIMBAL LOCK
# 00207		ISS TURNON REQUEST NOT PRESENT FOR 90 SEC	T4RUPT			                ALARM
# 00210		IMU NOT OPERATING				IMU MODE SWITCH, IMU-2, R02, P51        ALARM,VARALARM
# 00211		COARSE ALIGN ERROR - DRIVE > 2 DEGREES		IMU MODE SWITCH		                ALARM
# 00212		PIPA FAIL BUT PIPA IS NOT BEING USED		IMU MODE SWITCH,T4RPT	                ALARM
# 00213		IMU NOT OPERATING WITH TURN-ON REQUEST		T4RUPT			                ALARM
# 00214		PROGRAM USING IMU WHEN TURNED OFF		T4RUPT			                ALARM
# 00215		PREFERRED ORIENTATION NOT SPECIFIED		P52,P54			                ALARM
# 00217		BAD RETURN FROM STALL ROUTINES.			CURTAINS		                ALARM2
# 00220		IMU NOT ALIGNED - NO REFSMMAT			R02,P51			                VARALARM
# 00401		DESIRED GIMBAL ANGLES YIELD GIMBAL LOCK		IMF ALIGN, IMU-2	                ALARM
# 00404		TARGET OUT OF VIEW - TRUN ANGLE > 90 DEG	R52			                PRIOLARM
# 00405		TWO STARS NOT AVAILABLE				P52,P54			                ALARM
# 00406		REND NAVIGATION NOT OPERATING			R21,R23			                ALARM
# 00407		AUTO OPTICS REQUEST TRUN ANGLE > 50 DEG.	R52			                ALARM
# 00421		W-MATRIX OVERFLOW				INTEGRV			                VARALARM
# 00430	     *	INTEG. ABORT DUE TO SUBSURFACE S. V.		ALL CALLS TO INTEG	                POODOO
# 00600		IMAGINARY ROOTS ON FIRST ITERATION		P32, P72		                VARALARM
# 00601		PERIGEE ALTITUDE LT PMIN1			P32,P72,		                VARALARM
# 00602		PERIGEE ALTITUDE LT PMIN2			P32,P72,		                VARALARM
# 00603		CSI TO CDH TIME LT PMIN22			P32,P72,P33,P73		                VARALARM
# 00604		CDH TO TPI TIME LT PMIN23			P32,P72			                VARALARM
# 00605		NUMBER OF ITERATIONS EXCEEDS LOOP MAXIMUM	P32,P72,P37		                VARALARM
# 00606		DV EXCEEDS MAXIMUM				P32,P72			                VARALARM
# 00607	     *	NO SOLN FROM TIME-THETA OR TIME-RADIUS		TIMETHET,TIMERAD	                POODOO
# Page 24
# 00610      *	LAMBDA LESS THAN UNITY				P37			                POODOO
# 00611		NO TIG FOR GIVEN ELEV ANGLE			P34,P74			                VARALARM
# 00612		STATE VECTOR IN WRONG SPHERE OF INFLUENCE	P37			                VARALARM
# 00613		REENTRY ANGLE OUT OF LIMITS			P37			                VARALARM
# 00777		PIPA FAIL CAUSED ISS WARNING.			T4RUPT			                VARALARM
# 01102		CMC SELF TEST ERROR							                ALARM2
# 01103      *	UNUSED CCS BRANCH EXECUTED			ABORT			                POODOO
# 01104      *	DELAY ROUTINE BUSY				EXEC			                BAILOUT
# 01105		DOWNLINK TOO FAST				T4RUPT			                ALARM
# 01106		UPLINK TOO FAST					T4RUPT			                ALARM
# 01107		PHASE TABLE FAILURE. ASSUME			RESATRT			                ALARM
#		ERASABLE MEMORY IS DESTROYED
# 01201	     *	EXECUTIVE OVERFLOW-NO VAC AREAS		        EXEC			                BAILOUT
# 01202	     *	EXECUTIVE OVERFLOW-NO CORE SETS		        EXEC			                BAILOUT
# 01203      *	WAITLIST OVERFLOW-TOO MANY TASKS		WAITLIST		                BAILOUT
# 01204      *	NEGATIVE OR ZERO WAITLIST CALL			WAITLIST		                POODOO
# 01206      *	SECOND JOB ATTEMPTS TO GO TO SLEEP		PINBALL			                POODOO
#		VIA KEYBOARD AND DISPLAY PROGRAM
# 01207      *	NO VAC AREA FOR MARKS				SXTMARK			                BAILOUT
# 01210	     *	TWO PROGRAMS USING DEVICE AT SAME TIME		IMU MODE SWITCH		                POODOO
# 01211      *	ILLEGAL INTERRUPT OF EXTENDED VERB		SXTMARK			                BAILOUT
# 01301		ARCSIN-ARCCOS ARGUMENT TOO LARGE		INTERPRETER		                ALARM
# 01302      *	SQRT CALLED WITH NEGATIVE ARGUMENT.ABORT	INTERPRETER		                POODOO
# 01407		VG INCREASING					S40.8			                ALARM
# 01426		IMU UNSATISFACTORY				P61, P62			        ALARM
# 01427		IMU REVERSED					P61, P62			        ALARM
# 01501	     *	KEYBOARD AND DISPLAY ALARM DURING		PINBALL			                POODOO
#		 INTERNAL USE (NVSUB). ABORT.
# 01502	     *	ILLEGAL FLASHING DISPLAY			GOPLAY			                POODOO
# 01520		V37 REQUEST NOT PERMITTED AT THIS TIME		V37			                ALARM
# 01521	     *	P01 ILLEGALLY SELECTED				P01, P07                                POODOO
# 01600		OVERFLOW IN DRIFT TEST				OPT PRE ALIGN CALIB	                ALARM
# 01601      	BAD IMU TORQUE  				OPT PRE ALIGN CALIB	                ALARM
# 01602		BAD OPTICS DURING VERIFICATION			OPTALGN CALIB (CSM)	                ALARM
# 01703		INSUF. TIME FOR INTEG., TIG WAS SLIPPED		R41			                ALARM
# 03777		ICDU FAIL CAUSED THE ISS WARNING		T4RUPT			                VARALARM
# 04777		ICDU , PIPA FAILS CAUSED THE ISS WARNING	T4RUPT			                VARALARM
# 07777		IMU FAIL CAUSED THE ISS WARNING			T4RUPT			                VARALARM
# 10777		IMU , PIPA FAILS CAUSED THE ISS WARNING		T4RUPT			                VARALARM
# 13777		IMU , ICDU FAILS CAUSED THE ISS WARNING		T4RUPT			                VARALARM
# 14777		IMU,ICDU,PIPA FAILS CAUSED THE ISSWNING	        T4RUPT			                VARALARM
# 	     *	INDICATES ABORT TYPE.ALL OTHERS ARE NON-ABORTIVE

# Page 25

; ============================================================================
; SECTION: CHECKLIST CODES - Crew Procedure Request System
;
; This section documents the complete catalog of checklist codes used by
; Comanche 055 (Command Module flight software) to request astronaut actions.
; Checklist codes are the AGC's method of prompting the crew to perform
; specific console switch operations, manual tasks, or data entry procedures
; that cannot be automated by the computer.
;
; CHECKLIST CODE ARCHITECTURE:
; When software requires crew intervention, it displays a checklist code in
; DSKY register R1 (typically via Verb 05 Noun 09 or similar display verb).
; The code format is a 5-digit octal number identifying the specific action
; required. The crew:
; 1. Notes the checklist code displayed in R1
; 2. References their printed checklist or memory for the action
; 3. Performs the requested procedure
; 4. Presses PROCEED to acknowledge completion
; 5. The program continues execution
;
; CREW INTERFACE DESIGN PHILOSOPHY:
; The AGC cannot physically control all spacecraft systems. Many critical
; functions require manual switch throws, optical sightings, or physical
; procedures. Checklist codes provide structured computer-human cooperation,
; where the AGC:
; - Determines when an action is needed based on mission timeline
; - Displays the specific checklist code to identify the action
; - Waits for crew acknowledgment before continuing
; - Maintains mission sequence coordination
;
; CHECKLIST CODE CATEGORIES:
; Codes follow a systematic organization reflecting action type:
; - SWITCH codes: Request crew to change a console switch position
; - PERFORM codes: Request crew to start or complete a manual task
; - KEY IN codes: Request crew to manually enter data via DSKY
;
; Each action type requires different crew interaction. SWITCH operations
; typically involve immediate hardware configuration (CMC AUTO, OPTICS MODE,
; etc.). PERFORM operations require extended procedures (maneuver execution,
; alignment procedures, etc.). KEY IN operations require numerical data entry
; following the checklist code display.
;
; HISTORICAL CONTEXT - APOLLO 11 MISSION:
; Throughout Apollo 11's flight from July 16-24, 1969, checklist codes
; coordinated crew actions with autonomous AGC operations. Critical examples:
;
; - During translunar coast, checklist codes prompted Michael Collins to
;   perform periodic platform realignment using star sightings. The AGC
;   displayed the code, Collins performed optical marks on stars, the AGC
;   computed alignment corrections.
;
; - During preparations for lunar orbit insertion (LOI), checklist codes
;   prompted switch configurations for the Service Propulsion System (SPS)
;   engine burn. The AGC coordinated timing but relied on Collins to enable
;   hardware systems.
;
; - During entry preparations for Earth return, checklist codes guided
;   console switch settings for CM/SM separation, parachute deployment
;   arming, and entry autopilot configuration.
;
; The checklist code system exemplified Apollo's human-computer partnership.
; The AGC provided computational intelligence and mission timeline management,
; while astronauts provided physical manipulation capability and judgment.
;
; CODE FORMAT AND DISPLAY:
; Checklist codes appear as 5-digit octal numbers (e.g., 00014, 00041, 00202)
; displayed in DSKY register R1. The VERB NOUN combination varies by program:
; - V05 N09: Display checklist code and wait for PROCEED
; - V04 N06: Display checklist code with option selection
; - V50 N25: Display checklist code with load request
;
; The specific Verb/Noun determines whether the code is informational (crew
; acknowledges with PROCEED) or interactive (crew enters data in R2/R3).
;
; CHECKLIST REFERENCE DOCUMENTATION:
; Astronauts carried printed checklists keyed to these codes. Each code
; number corresponded to a specific procedure card or checklist page with
; detailed instructions. The codes served as compact references, avoiding
; long text messages on the limited DSKY display. A 5-digit code could
; reference multi-step procedures documented in crew manuals.
;
; PROGRAM COORDINATION:
; Multiple programs use checklist codes to coordinate mission phases:
; - P50 series (IMU alignment programs): Request fine alignment options,
;   star mark termination, optics mode switches
; - P40 series (SPS burn programs): Request gimbal trim, switch to CMC AUTO,
;   automatic maneuver execution
; - Entry programs: Request CM/SM separation switch, AGC power down sequences
;
; The checklist system integrated software state machines with human
; procedures, creating a cooperative control system spanning computer logic
; and astronaut training.
;
; MISSION TIMELINE INTEGRATION:
; Checklist codes aren't arbitrary interruptions—they're precisely timed
; within mission sequences. The AGC's executive scheduler determines when
; a program reaches a point requiring crew action, displays the appropriate
; checklist code, and suspends program execution until crew acknowledgment.
; This ensures mission phases proceed in correct order with proper crew
; coordination.
;
; APOLLO TRAINING INTEGRATION:
; Astronaut training extensively practiced checklist code responses. Simulator
; sessions displayed checklist codes at mission-realistic timing, training
; crews to recognize codes, execute procedures, and maintain mission flow.
; Armstrong, Aldrin, and Collins trained hundreds of hours responding to
; these exact checklist codes before Apollo 11's flight.
;
; COMMENT-ONLY READERS: Checklist codes are how the Apollo computer asked
;        astronauts to flip switches or perform manual tasks. Instead of
;        displaying long instructions on the small DSKY screen, the computer
;        showed a 5-digit code number. The astronauts recognized the code
;        from their training and printed checklists, performed the action
;        (like "switch to automatic mode" or "perform alignment"), then
;        pressed PROCEED to tell the computer they were done. This system
;        let the computer coordinate mission timing while astronauts handled
;        physical operations.
;
; CODE-ALONG READERS: Checklist codes are displayed by calling display
;        routines (DSPLAY, BANKCALL to NVSUBUSY, etc.) with the 5-digit
;        code loaded in appropriate erasable memory locations. The display
;        routine formats the code for DSKY register R1 and typically uses
;        V05 N09 (display and wait for PROCEED) or V04 N06 (display and
;        request option entry). The calling program then executes TC ENDIDLE
;        or similar wait instruction, suspending until crew presses PROCEED.
;        Understanding checklist code flow requires tracing: program decision
;        → checklist code selection → display routine call → DSKY update →
;        crew action → PROCEED detection → program continuation. See
;        PINBALL_GAME_BUTTONS_AND_LIGHTS.agc for PROCEED key handling and
;        DISPLAY_INTERFACE_ROUTINES.agc for register formatting logic.
; ============================================================================

#               CHECKLIST CODES FOR 504

#               PLEASE REPORT ANY DEFICIENCIES IN THIS LIST TO JOHN SUTHERLAND

# *9		*17		*26  COLUMN
#
# R1 CODE	   ACTION TO BE EFFECTED
#
# 00014		KEY IN		FINE ALIGNMENT OPTION
# 00015		PERFORM		CELESTIAL BODY ACQUISITION
# 00016		KEY IN		TERMINATE MARK SEQUENCE
# 00041		SWITCH		CM/SM SEPARATION TO UP
# 00062		SWITCH		AGC POWER DOWN
# 00202		PERFORM		GNCS AUTOMATIC MANEUVER
# 00203		SWITCH		TO CMC-AUTO
# 00204		PERFORM		SPS GIMBAL TRIM
# 00403		SWITCH		OPTICS TO MANUAL OR ZERO
#		                  SWITCH DENOTES CHANGE POSITION OF A CONSOLE SWITCH
#		                  PERFORM DENOTES START OR END OF A TASK
#		                  KEY IN DENOTES KEY IN OF DATA THRU THE DSKY

# Page 26

; ============================================================================
; SECTION: OPTION CODES - Crew Decision Selection System
;
; This section documents the complete catalog of option codes used by
; Comanche 055 (Command Module flight software) to request astronaut
; decision input during mission operations. Option codes differ from
; checklist codes: instead of requesting a predefined action, option codes
; ask the crew to choose between multiple operational alternatives.
;
; OPTION CODE ARCHITECTURE:
; When software reaches a decision point requiring crew judgment, it displays
; an option code in DSKY register R1 (via Verb 04 Noun 06) and requests the
; astronaut to select their preferred option by loading a choice code into
; register R2. The interaction sequence:
; 1. AGC displays option code in R1 (flashing, indicating input required)
; 2. Crew references option code meaning from training or checklist
; 3. Crew evaluates mission context and decides which option to select
; 4. Crew keys in option number to R2 via DSKY numeric keys and ENTER
; 5. AGC validates the option selection
; 6. AGC executes the selected operational branch
; 7. Program continues based on crew's choice
;
; OPTION CODE vs CHECKLIST CODE DISTINCTION:
; - CHECKLIST CODES: Request specific predefined action ("perform alignment")
; - OPTION CODES: Request crew choice between alternatives ("preferred, 
;   nominal, or current orientation?")
;
; Option codes acknowledge that some mission decisions require human judgment
; based on factors the AGC cannot assess: visual observations, system status
; indications, mission priorities, ground controller recommendations, etc.
; The AGC provides the computational framework and presents the decision
; point, but defers the choice to astronaut expertise.
;
; OPTION CODE CATEGORIES BY PURPOSE:
; Options span diverse operational domains:
; - IMU orientation selection (preferred, nominal, REFSMMAT)
; - Navigation state update methods (optical marks, radar data, ground uplink)
; - Burn targeting options (time vs propellant optimization)
; - Display format preferences (inertial, stabilization, local vertical)
; - Program sequence alternatives (continue, skip, repeat)
; - Alignment method selection (auto optics, manual marks, platform)
; - Entry targeting options (primary, backup, manual)
;
; Each option code defines valid input ranges. For example, option code 00001
; (IMU orientation) accepts R2 inputs: 1=PREF, 2=NOM, 3=REFSMMAT. Invalid
; entries trigger operator error (OPER ERR light) and request re-entry.
;
; HISTORICAL CONTEXT - APOLLO 11 MISSION:
; Throughout Apollo 11's mission, option codes enabled crew control over
; operational decisions:
;
; - During translunar navigation, option codes allowed Michael Collins to
;   choose which celestial bodies to mark with the sextant for state vector
;   updates. The AGC presented options, Collins selected based on star
;   visibility and geometric strength.
;
; - During lunar orbit operations, option codes let Collins choose IMU
;   alignment strategies. Based on time availability and mission phase, he
;   could select faster nominal alignments or slower precision alignments.
;
; - During pre-entry preparations, option codes allowed the crew to select
;   entry targeting options. With variable weather at splashdown sites,
;   they could bias the entry corridor for different landing zones.
;
; The option code system embodied Apollo philosophy: automation handles
; routine calculations, but humans retain decision authority for judgment
; calls. This preserved crew agency while leveraging computer precision.
;
; VERB 04 NOUN 06 INTERACTION PROTOCOL:
; Option code displays use V04 N06 (Display Component 1, Monitor Components
; 2 and 3). The display sequence:
; - R1: Option code (flashing to indicate input required)
; - R2: Blank initially, awaits crew numerical entry
; - R3: May display additional context (timing, quantity, etc.)
;
; The flashing R1 signals "this is not just information—I need your input."
; After crew enters their choice in R2 and presses ENTER, R1 stops flashing,
; confirming acceptance. If the entry is invalid, OPER ERR illuminates and
; R1 continues flashing until valid input received.
;
; OPTION INPUT VALIDATION:
; Each option code defines acceptable input ranges. Software validates crew
; entries against these ranges:
; - Numeric range validation (e.g., 1-3 for three-option choices)
; - Semantic validation (e.g., don't select radar option when radar failed)
; - Context validation (e.g., certain options only valid in specific orbits)
;
; Invalid entries don't crash the program or proceed with bad data. The AGC
; displays OPER ERR, clears the invalid entry, and re-flashes R1 requesting
; correct input. This error handling prevented crew data entry mistakes from
; propagating into mission-critical calculations.
;
; TRAINING AND CREW PROFICIENCY:
; Astronauts memorized common option codes and their meanings through
; extensive simulator training. During Apollo 11's mission, Armstrong, Aldrin,
; and Collins responded to option codes reflexively, having practiced each
; decision point hundreds of times. The option codes served as compact
; communication protocol between well-trained crews and software.
;
; Option codes also appeared in crew checklists with explanatory text. For
; less-common options or complex decisions, astronauts could reference
; printed documentation. The 5-digit code linked computer display to detailed
; procedure documentation.
;
; GROUND CONTROL COORDINATION:
; Option code displays were visible to Mission Control via telemetry. When
; an option code flashed on the spacecraft DSKY, ground controllers saw the
; same code in their displays. This enabled:
; - Ground advice on option selection when requested
; - Ground awareness of crew decisions for coordinated planning
; - Post-mission analysis of option choices vs outcomes
;
; During Apollo 11, Houston monitored option code displays and often
; radioed recommendations, especially for navigation and trajectory decisions.
; The crew retained final authority but benefited from ground analysis.
;
; PROGRAM APPLICABILITY:
; Option codes appear throughout mission software:
; - P50 series (IMU alignment): IMU orientation, alignment method, star
;   selection, optics mode
; - P20 series (Navigation): Update method, reference body, mark quantity
; - P30/P40 series (Maneuvers): Burn targeting, gimbal control, propellant
;   budget
; - P60 series (Entry): Target selection, entry mode, lift vector strategy
;
; The same option code may appear in multiple programs if the decision context
; recurs. For example, IMU orientation selection (code 00001) appears in
; P51, P52, P53 alignment programs and various maneuver programs.
;
; DECISION AUTHORITY AND RESPONSIBILITY:
; Option codes formalized decision authority. When an option code appeared,
; the computer explicitly transferred decision-making to the crew. This clear
; authority handoff prevented confusion about who (computer or human) was
; controlling each mission aspect. The AGC never guessed at crew intent—it
; asked explicitly and waited for authoritative input.
;
; COMMENT-ONLY READERS: Option codes are how the Apollo computer asked
;        astronauts to make decisions during the mission. When the computer
;        reached a choice point—like "which way should I orient the spacecraft?"
;        or "which navigation method do you want to use?"—it displayed an
;        option code and waited for the astronaut to pick an option number.
;        Unlike checklist codes that requested specific actions, option codes
;        presented multiple alternatives and let the crew choose based on
;        their judgment of the situation. This system kept humans in control
;        of important decisions while letting the computer handle calculations.
;
; CODE-ALONG READERS: Option codes are displayed using V04 N06 (display R1,
;        load R2/R3). The calling program loads the option code into DSPTEM1
;        (or similar erasable), sets the display flash flag, and calls the
;        display interface routine. The display system formats R1 with the
;        option code (flashing) and blanks R2 awaiting crew entry. After crew
;        keys in their selection and presses ENTER, the KEYBOARD interrupt
;        handler processes the entry, validates it against acceptable range,
;        and either stores the option in program-specific erasable (valid) or
;        triggers OPER ERR (invalid). The calling program polls for completion
;        or gets resumed by display system, reads the selected option from
;        erasable, and branches accordingly (TC indexed by option, computed
;        CADR, or switch table). Understanding option code flow requires
;        tracing: program decision point → V04 N06 display → KEYBOARD input
;        processing → validation → option storage → program branch selection.
;        See DISPLAY_INTERFACE_ROUTINES.agc for V04 N06 formatting and
;        PINBALL_GAME_BUTTONS_AND_LIGHTS.agc for ENTER key processing and
;        validation logic.
; ============================================================================

#          OPTION CODES FOR 504

#          PLEASE REPORT ANY DEFICIENCIES IN THIS LIST TO JOHN SUTHERLAND

# THE SPECIFIED OPTION CODES WILL BE FLASHED IN COMPONENT R1 IN
# CONJUNCTION WITH VERB04NOUN06 TO REQUEST THE ASTRONAUT TO LOAD INTO
# COMPONENT R2 THE OPTION HE DESIRES.

# *9		*17					*52				*11		*25  COLUMN
#
# OPTION
# CODE		PURPOSE					INPUT FOR COMPONENT 2		PROGRAM(S)	APPLICABILITY
#
# 00001		SPECIFY IMU ORIENTATION			1=PREF 2=NOM 3=REFSMMAT		P50'S		ALL
# 00002		SPECIFY VEHICLE				1=THIS 2=OTHER			P21,R30		ALL
# 00003		SPECIFY TRACKING ATTITUDE		1=PREFERRED 2=OTHER		R63		ALL
# 00004		SPECIFY RADAR				1=RR 2=LR			R04		SUNDANCE + LUMINARY
# 00005		SPECIFY SOR PHASE			1=FIRST 2=SECOND		P38		COLOSSUS + LUMINARY
# 00006		SPECIFY RR COARSE ALIGN OPTION		1=LOCKON 2=CONTINUOUS DESIG.	V41N72		SUNDANCE + LUMINARY
# 00007		SPECIFY PROPULSION SYSTEM		1=SPS 2=RCS			P37		COLOSSUS
# 00010		SPECIFY ALIGNMENT MODE			0=ANY TIME 1=REFSMMAT +G	P57		LUMINARY
#							2=TWO BODIES 3=ONE BODY + G
# 00011		SPECIFY SEPARATION MONITOR PHASE	1=DELTAV 2=STATE VECTOR UPDATE	P46		LUMINARY
# 00012		SPECIFY CSM ORBIT OPTION		1=NO ORBIT CHANGE 2=CHANGE	P22		LUMINARY
#							ORBIT TO PASS OVER LM
