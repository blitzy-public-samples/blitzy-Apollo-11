# Copyright:	Public domain.
# Filename:	TAGS_FOR_RELATIVE_SETLOC.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Mod history:	2009-05-05 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-20 RSB	Corrected R32 -> R31.
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

# Page 27
# TAGS FOR RELATIVE SETLOC AND BLANK BANK CARDS

; ============================================================================
; FILE: TAGS_FOR_RELATIVE_SETLOC.agc
; MODULE: INFORMATION Subsystem
; MISSION PHASE: all-phases (memory organization)
;
; TL;DR: Defines SETLOC tags establishing relative memory segment locations
;        across AGC's banked memory architecture. These tags enable assembler
;        to allocate code and data to appropriate banks within 2K erasable
;        and 36K fixed memory spaces, critical for managing the severe memory
;        constraints of 1960s computer hardware.
;
; COMMENT-ONLY READERS: This file organizes how code fits into the computer's
;        limited memory by dividing it into manageable segments.
; CODE-ALONG READERS: Study memory segment allocation strategy and bank
;        assignment rationale for understanding AGC memory architecture.
; ============================================================================

; FIXED MEMORY ORGANIZATION
; The Apollo Guidance Computer has 36K words of fixed (read-only) rope core
; memory organized into banks. This file establishes symbolic tags that allow
; the assembler to place program code into specific memory banks, ensuring
; optimal organization and enabling bank-switching mechanisms during execution.
;
; Memory addresses 120000-167777 (octal) represent the fixed memory space.
; Banks are 1K-word segments, and the AGC uses bank-switching to access the
; full 36K address space within its 15-bit addressing limitations.

FIXED		MEMORY	120000 - 167777
		COUNT	BANKSUM

; ============================================================================
; MODULE 1: BANKS 0 THROUGH 5
;
; Module 1 contains the lowest-numbered fixed memory banks, traditionally
; housing core operating system code, interpreter routines, and fundamental
; navigation/guidance subroutines. Banks 0 and 1 are special-purpose and
; always accessible without bank switching.
; ============================================================================

# MODULE 1 CONTAINS BANKS 0 THROUGH 5

; BLOCK 02 - Core Program Segments
; Multiple program segments (FFTAGx) are assigned to Block 02, including
; fundamental subroutines used throughout mission operations. SETLOC tags
; defined here (EQUALS directives) will be referenced in other source files
; to place code at these locations during assembly.

		BLOCK	02
FFTAG1		EQUALS
FFTAG2		EQUALS
FFTAG3		EQUALS
FFTAG4		EQUALS
FFTAG7		EQUALS
FFTAG8		EQUALS
FFTAG9		EQUALS
FFTAG10		EQUALS
FFTAG12		EQUALS
P30SUBS		EQUALS
STOPRAT		EQUALS
P23S		EQUALS
		BNKSUM	02

; BLOCK 03 - Additional Core Segments
; Block 03 contains additional program segments including Digital Autopilot
; System (DAPS9) routines crucial for spacecraft attitude control.

		BLOCK	03
FFTAG5		EQUALS
FFTAG6		EQUALS
DAPS9		EQUALS
FFTAG13		EQUALS
		BNKSUM	03

; BANK 00 - Unswitched Fixed Memory (Always Accessible)
; Bank 00 is special: it's always accessible without bank switching, containing
; critical interrupt handlers and time-critical routines that must execute
; without the overhead of bank switching. DLAYJOB manages delayed job execution.

		BANK	00
DLAYJOB		EQUALS
		BNKSUM	00

; BANK 01 - Restart and Recovery System
; Bank 01 contains restart protection logic, essential for recovering from
; power transients or computational anomalies. The restart system preserved
; critical mission data during the famous 1202 program alarms of Apollo 11's
; lunar descent, allowing the landing to proceed successfully.

		BANK	01
RESTART		EQUALS
		BNKSUM	01

; BANK 4 - Verb/Noun Processing and Mission Programs
; Bank 4 houses DSKY verb processing (VERB37), orbital mechanics calculations
; (CONICS1), display interface routines (PINBALL4), rendezvous targeting
; (CSI/CDH1), interpreter extensions (INTPRET2), IMU calibration (IMUCAL1),
; and various navigation/guidance programs used throughout the mission.

		BANK	4
VERB37		EQUALS
CONICS1		EQUALS
PINBALL4	EQUALS
CSI/CDH1	EQUALS
INTPRET2	EQUALS
IMUCAL1		EQUALS

# Page 28

STBLEORB	EQUALS
E/PROG		EQUALS
MIDDGIM		EQUALS
		BNKSUM	04

; BANK 5 - Telemetry and Autopilot Mass Properties
; Bank 5 contains downlink telemetry formatting (DOWNTELM), digital autopilot
; mass property calculations (DAPMASS) accounting for fuel consumption and
; center-of-gravity changes, and rendezvous program tags (CDHTAG).

		BANK	5
FRANDRES	EQUALS
DOWNTELM	EQUALS
DAPMASS		EQUALS
CDHTAG		EQUALS
		BNKSUM	05

; ============================================================================
; MODULE 2: BANKS 6 THROUGH 13
;
; Module 2 contains mid-level mission programs including IMU compensation,
; interrupt handlers, display routines, orbital mechanics calculations,
; and various navigation programs. This module supported critical mission
; phases including translunar coast, lunar orbit operations, and rendezvous.
; ============================================================================

# MODULE 2 CONTAINS BANKS 6 THROUGH 13

; BANK 6 - IMU and Interrupt Processing
; Bank 6 houses IMU compensation algorithms (IMUCOMP) correcting for gyro
; drift and accelerometer bias, timer interrupt handler T4RUP executing every
; 10 milliseconds, and rendezvous program segments (CSIPROG).

		BANK	6
IMUCOMP		EQUALS
T4RUP		EQUALS
IMUCAL2		EQUALS
CSIPROG		EQUALS
		BNKSUM	06

; BANK 7 - Optical Navigation and Keyboard Interrupt
; Bank 7 contains sextant optical mark tracking (SXTMARKE) for star/landmark
; sightings, keyboard interrupt handler (KEYRUPT) processing DSKY button
; presses, and mode switching routines enabling transitions between mission
; program phases.

		BANK	7
SXTMARKE	EQUALS
R02		EQUALS
MODESW		EQUALS
XANG		EQUALS
KEYRUPT		EQUALS
CSIPROG6	EQUALS
		BNKSUM	07

; BANK 10 (Octal) - Display Interface and Phase Tables
; Bank 10 houses comprehensive display interface routines (DISPLAYS), mission
; phase state tables (PHASETAB) tracking current program phase for restart
; protection, spacecraft geometry definitions (COMGEOM2), and optical system
; drivers (OPTDRV) for telescope and sextant control.

		BANK	10
DISPLAYS	EQUALS
PHASETAB	EQUALS
COMGEOM2	EQUALS
SXTMARK1	EQUALS
P60S4		EQUALS
OPTDRV		EQUALS
CSIPROG8	EQUALS
		BNKSUM	10

; BANK 11 (Octal) - Orbital Mechanics and Integration
; Bank 11 contains orbital integration routines (ORBITAL) propagating state
; vectors through time, orbital constants, numerical integration initialization
; (INTINIT1), and velocity computation routines (INTVEL). These calculations
; were continuously updated during cislunar coast and lunar orbit phases.

		BANK	11
ORBITAL		EQUALS
ORBITAL1	EQUALS			# CONSTANTS

# Page 29

INTVEL		EQUALS
S52/2		EQUALS
CSIPROG5	EQUALS
INTINIT1	EQUALS
		BNKSUM	11

; BANK 12 (Octal) - Conic Trajectory Calculations
; Bank 12 houses conic subroutines (CONICS) solving two-body orbital mechanics
; problems, rendezvous program segments (CSIPROG2, CSI/CDH2), and mode change
; logic (MODCHG2) enabling transitions between mission programs.

		BANK	12
CONICS		EQUALS
CSIPROG2	EQUALS
CSI/CDH2	EQUALS
MODCHG2		EQUALS
		BNKSUM	12

; BANK 13 (Octal) - Navigation Updates and Program Initialization
; Bank 13 contains navigation state initialization (INTINIT), latitude/
; longitude computation (LATLONG), orbital integration routines (ORBITAL2),
; and various program entry points for rendezvous and navigation functions.

		BANK	13
P76LOC		EQUALS
LATLONG		EQUALS
INTINIT		EQUALS
SR52/1		EQUALS
ORBITAL2	EQUALS
CDHTAGS		EQUALS
E/PROG1		EQUALS
MODCHG3		EQUALS
		BNKSUM	13

# SPACER

; ============================================================================
; MODULE 3: BANKS 14 THROUGH 21
;
; Module 3 contains upper-level mission programs including star tables for
; optical navigation, measurement incorporation algorithms, powered flight
; navigation, time-of-free-fall calculations, and various mission-specific
; programs. This module supported major propulsive maneuvers and navigation
; state updates throughout the mission.
; ============================================================================

# MODULE 3 CONTAINS BANKS 14 THROUGH 21

; BANK 14 (Octal) - Star Tables and Measurement Processing
; Bank 14 houses star catalog (STARTAB) containing positions of navigation
; stars, measurement incorporation routines (MEASINC2) integrating sensor
; data into navigation state, and rendezvous program segments (CSI/CDH3).

		BANK 	14
STARTAB		EQUALS
RT53		EQUALS
P50S1		EQUALS
MEASINC2	EQUALS
CSI/CDH3	EQUALS
		BNKSUM	14

; BANK 15 (Octal) - Alignment Programs and Entry DAP
; Bank 15 contains IMU alignment program routines (P50S), entry digital
; autopilot initialization (ETRYDAP) for atmospheric reentry control, and
; supporting subroutines for platform orientation procedures.

		BANK	15
P50S		EQUALS
ETRYDAP		EQUALS
S52/3		EQUALS
		BNKSUM	15

; BANK 16 (Octal) - SPS Burn Programs and DAP Roll Control
; Bank 16 houses Service Propulsion System burn program segments (P40S1),
; digital autopilot roll axis control (DAPROLL), alignment routines (P50S2),
; and entry program segments (RTE2, P23S1).

		BANK	16
P40S1		EQUALS

# Page 30

DAPROLL		EQUALS
P50S2		EQUALS
P23S1		EQUALS
RTE2		EQUALS
		BNKSUM	16

; BANK 17 (Octal) - Digital Autopilot Segments and Alignment
; Bank 17 contains multiple digital autopilot segments (DAPS4, DAPS5, DAPS7)
; for attitude control during coast phases and propulsive maneuvers, along
; with alignment program routines (P50S3).

		BANK	17
DAPS4		EQUALS
DAPS5		EQUALS
DAPS7		EQUALS
P50S3		EQUALS
		BNKSUM	17

; BANK 20 (Octal) - Autopilot Core and Manual Control
; Bank 20 houses foundational digital autopilot segments (DAPS6, DAPS1, DAPS2)
; managing spacecraft attitude control throughout mission, manual control
; routines (MANUSTUF) enabling crew attitude overrides, and utility subroutines
; (R36CM, VAC5LOC) supporting navigation calculations.

		BANK	20
DAPS6		EQUALS
DAPS1		EQUALS
DAPS2		EQUALS
MANUSTUF	EQUALS
R36CM		EQUALS
VAC5LOC		EQUALS
		BNKSUM	20

; BANK 21 (Octal) - Autopilot and Kalman Monitoring
; Bank 21 contains digital autopilot segment (DAPS3), utility subroutines
; (MYSUBS) for control system calculations, and Kalman filter monitoring
; routines (KALCMON3) for navigation state quality assessment.

		BANK	21
DAPS3		EQUALS
MYSUBS		EQUALS
KALCMON3	EQUALS
		BNKSUM	21

; ============================================================================
; MODULE 4: BANKS 22 THROUGH 27
;
; Module 4 contains interpreter return code handlers, mission programs for
; rendezvous and powered flight, reentry guidance, ephemeris calculations,
; and time-of-free-fall computations. This module supported critical mission
; phases including rendezvous operations, main engine burns, and Earth
; atmospheric reentry.
; ============================================================================

# MODULE 4 CONTAINS BANKS 22 THROUGH 27

; BANK 22 (Octal) - Interpreter Return Codes and Orbital Programs
; Bank 22 houses interpreter RTB (Return To Bank) operation codes (RTBCODES,
; RTBCODE1) enabling bank-switching returns from interpretive routines,
; digital autopilot segment (DAPS8), apogee/perigee calculation (APOPERI),
; and Kalman filter monitoring (KALCMON2, KALCMON1).

		BANK	22
RTBCODES	EQUALS
RTBCODE1	EQUALS
DAPS8		EQUALS
APOPERI		EQUALS
P40S5		EQUALS
KALCMON2	EQUALS
KALCMON1	EQUALS
CSIPROG3	EQUALS
		BNKSUM	22

# Page 31

; BANK 23 (Octal) - Powered Flight and Rendezvous Navigation
; Bank 23 contains rendezvous program segments (P20S2), in-flight alignment
; routines (INFLIGHT), spacecraft geometry (COMGEOM1), powered flight
; navigation (POWFLITE, POWFLIT1, POWFLIT2) monitoring main engine burns,
; rendezvous guidance (RENDGUID), and orbit parameter display (R30LOC).
; These routines executed during SPS burns for lunar orbit insertion,
; transearth injection, and CSM/LM rendezvous operations.

		BANK	23
P20S2		EQUALS
INFLIGHT	EQUALS
COMGEOM1	EQUALS
POWFLITE	EQUALS
POWFLIT1	EQUALS
RENDGUID	EQUALS
POWFLIT2	EQUALS
R30LOC		EQUALS
P11FOUR		EQUALS
CSIPROG4	EQUALS
		BNKSUM	23

; BANK 24 (Octal) - DAP Initialization and Burn Programs
; Bank 24 houses digital autopilot initialization (LOADDAP) configuring
; control gains and spacecraft mass properties before propulsive maneuvers,
; SPS burn program segments (P40S), and rendezvous program continuation
; (CSIPROG7).

		BANK	24
LOADDAP		EQUALS
P40S		EQUALS
CSIPROG7	EQUALS
		BNKSUM	24

; BANK 25 (Octal) - Atmospheric Reentry Guidance
; Bank 25 contains reentry guidance routines (REENTRY) computing lift vector
; commands for range control during atmospheric entry, executed during final
; Apollo 11 mission phase on July 24, 1969, targeting Pacific Ocean splashdown.

		BANK	25
REENTRY		EQUALS
CDHTAG1		EQUALS
		BNKSUM	25

; BANK 26 (Octal) - Entry Programs and Ephemerides
; Bank 26 houses interpreter routines (INTPRET1), reentry program segments
; (REENTRY1), entry initialization programs (P60S series), planetary inertial
; orientation (PLANTIN) for lunar rotation effects, ephemeris calculations
; (EPHEM) for Moon and Sun positions, and P05/P06 program segments.

		BANK	26
INTPRET1	EQUALS
REENTRY1	EQUALS
P60S		EQUALS
P60S1		EQUALS
P60S2		EQUALS
P60S3		EQUALS
PLANTIN		EQUALS			# LUNAR ROT
EPHEM		EQUALS
P05P06		EQUALS
26P50S		EQUALS
		BNKSUM	26

; BANK 27 (Octal) - Time-of-Free-Fall and Maneuver Calculations
; Bank 27 contains time-of-free-fall calculations (TOF-FF, TOF-FF1) predicting
; trajectory arrival times, attitude maneuver routines (MANUVER, MANUVER1)
; for crew-commanded rotations, vector pointing (VECPT), navigation state
; update programs (UPDATE1, UPDATE2), and various program segments supporting
; mission operations.

		BANK	27
TOF-FF		EQUALS
TOF-FF1		EQUALS
MANUVER		EQUALS
MANUVER1	EQUALS

# Page 32

VECPT		EQUALS
UPDATE1		EQUALS
UPDATE2		EQUALS
R22S1		EQUALS
P60S5		EQUALS
P40S2		EQUALS
		BNKSUM	27

; ============================================================================
; MODULE 5 CONTAINS BANKS 30 THROUGH 35
; ============================================================================
; Module 5 provides memory allocation for advanced navigation, integration,
; orbital mechanics, and executive system functions. These banks contain
; critical computational subroutines used throughout all mission phases,
; including conic trajectory calculations, numerical integration routines,
; and restart protection mechanisms that proved essential during Apollo 11's
; 1202 program alarm recovery.
; ============================================================================

; BANK 30 (Octal) - IMU Supervisor and Mission Programs
; Bank 30 contains IMU supervisor routines (IMUSUPER), low-level supervisor
; functions (LOWSUPER), fresh start/restart entry point (FCSTART), low-priority
; computer routines (LOPC), rendezvous navigation segments (P20S1, P20S6),
; Service Propulsion System burn programs (P40S3), and rotational maneuver
; routines (R35A).

		BANK	30
IMUSUPER	EQUALS
LOWSUPER	EQUALS
FCSTART		EQUALS			# STANDARD LOCATION FOR THIS. (FOR EXTV8)
LOPC		EQUALS
P20S1		EQUALS
P20S6		EQUALS
P40S3		EQUALS
R35A		EQUALS
		BNKSUM	30

; BANK 31 (Octal) - Navigation and Targeting Programs
; Bank 31 houses rotational maneuver routines (R35, R34), rendezvous targeting
; programs (RT23, RTE3), external Delta-V programs (P30S1A), coelliptic sequence
; initiation (CSIPROG9), orbit parameter display (R31), landmark tracking (P22S),
; and CDH targeting (CDHTAG2).

		BANK	31
R35		EQUALS
RT23		EQUALS
P30S1A		EQUALS
R34		EQUALS
CDHTAG2		EQUALS
CSIPROG9	EQUALS
R31		EQUALS
P22S		EQUALS
RTE3		EQUALS
		BNKSUM	31

; BANK 32 (Octal) - Display Interface and IMU Calibration
; Bank 32 contains message scanning routines for crew display updates (MSGSCAN1),
; rendezvous targeting calculations (RTE), DSKY display list processing
; (DELRSPL1), and IMU calibration subroutines (IMUCAL3) for gyroscope drift
; compensation.

		BANK	32
MSGSCAN1	EQUALS
RTE		EQUALS
DELRSPL1	EQUALS
IMUCAL3		EQUALS
		BNKSUM	32

; BANK 33 (Octal) - System Test and IMU Calibration
; Bank 33 holds self-test lead-in routines (TESTLEAD) and comprehensive IMU
; calibration procedures (IMUCAL) including gyroscope alignment verification,
; accelerometer bias determination, and platform drift compensation algorithms.

		BANK	33
TESTLEAD	EQUALS

# Page 33

IMUCAL		EQUALS
		BNKSUM	33

; BANK 34 (Octal) - Earth Orbit and Rendezvous Programs
; Bank 34 stores Earth orbit insertion monitoring (P11ONE), rendezvous navigation
; segments (P20S3, P20S4), and rendezvous targeting constants (RTECON) used for
; calculating CSI/CDH/TPI burn parameters during orbital rendezvous sequences.

		BANK	34
P11ONE		EQUALS
P20S3		EQUALS
P20S4		EQUALS
RTECON		EQUALS
		BNKSUM	34

; BANK 35 (Octal) - Targeting and Navigation State Updates
; Bank 35 contains rendezvous targeting constants (RTECON1), CSI/CDH maneuver
; programs, external Delta-V program segments (P30S1, P30S), star acquisition
; routines (P17S1), measurement incorporation for navigation state updates
; (MEASINC3), and numerical integration initialization (INTINIT2).

		BANK	35
RTECON1		EQUALS
CSI/CDH		EQUALS
P30S1		EQUALS
P30S		EQUALS
P17S1		EQUALS
MEASINC3	EQUALS
INTINIT2	EQUALS
		BNKSUM	35

; ============================================================================
; MODULE 6 CONTAINS BANKS 36 THROUGH 43
; ============================================================================
; Module 6 provides memory allocation for navigation state update algorithms,
; orbital integration routines, conic trajectory calculations, and system
; self-check diagnostics. These banks house the mathematical foundation for
; all navigation state propagation and the AGC self-test routines that
; verified computer health throughout the mission.
; ============================================================================

; BANK 36 (Octal) - Measurement Incorporation and Navigation State Updates
; Bank 36 contains measurement incorporation routines (MEASINC, MEASINC1) that
; process sensor data from the IMU, sextant, and radar to update the spacecraft
; navigation state. Also includes star acquisition programs (P17S), rendezvous
; targeting routines (RTE1), and shared location tags (S3435LOC).

		BANK	36
MEASINC		EQUALS
MEASINC1	EQUALS
P17S		EQUALS
RTE1		EQUALS
S3435LOC	EQUALS
		BNKSUM	36

; BANK 37 (Octal) - Rendezvous Navigation and Service Routines
; Bank 37 houses rendezvous navigation programs (P20S for relative state vector
; computation, RENDEZ for rendezvous targeting), body attitude determination
; routines (BODYATT), general service functions (SERVICES), Earth orbit insertion
; monitoring (P11TWO), and CSI/CDH maneuver routines (CDHTAG3).

		BANK	37
P20S		EQUALS
BODYATT		EQUALS
RENDEZ		EQUALS
SERVICES	EQUALS
P11TWO		EQUALS
CDHTAG3		EQUALS
		BNKSUM	37

; BANK 40 (Octal) - Display Interface and Self-Check Supervisor
; Bank 40 contains the DSKY (display and keyboard) supervisor routines (PINSUPER)
; controlling crew interface, AGC self-check supervisor (SELFSUPR) coordinating
; diagnostic tests, display interface routines (PINBALL1), and landmark tracking
; for Command Module (R36CM1).

		BANK	40
PINSUPER	EQUALS

# Page 34

SELFSUPR	EQUALS
PINBALL1	EQUALS
R36CM1		EQUALS
		BNKSUM	40

; BANK 41 (Octal) - Display Interface Continuation
; Bank 41 contains additional display interface routines (PINBALL2) managing
; DSKY verb/noun processing and display formatting, plus landmark tracking
; routines for Lunar Module (R36LM) used during cislunar navigation.

		BANK	41
PINBALL2	EQUALS
R36LM		EQUALS
		BNKSUM	41

; BANK 42 (Octal) - Communication and Extended Display Functions
; Bank 42 contains S-band antenna pointing routines (SBAND) for high-gain
; communication with Earth, additional display interface segments (PINBALL3),
; extended verb routines (EXTVBS) providing specialized crew commands, and
; lunar landmark tracking continuation (R36LM1).

		BANK	42
SBAND		EQUALS
PINBALL3	EQUALS
EXTVBS		EQUALS
R36LM1		EQUALS
		BNKSUM	42

; BANK 43 (Octal) - Self-Check Diagnostics and Extended Verbs
; Bank 43 contains AGC Block II self-check diagnostic routines (SELFCHEC)
; performing instruction validation and memory testing to verify computer
; health, plus additional extended verb implementations (EXTVERBS) providing
; specialized crew interface functions for mission operations.

		BANK	43
SELFCHEC	EQUALS
EXTVERBS	EQUALS
		BNKSUM	43

; ============================================================================
; CONSTANT ASSIGNMENTS FOR HIGH/LOW MEMORY ACCESS
; ============================================================================
; These assignments define standard mathematical constants (zero vectors,
; unit vectors, fractional values) with separate high-memory and low-memory
; aliases. This dual-location strategy ensures constants are accessible
; from any memory bank without requiring bank switching, optimizing
; computation speed for time-critical navigation and guidance calculations.
; ============================================================================

HI6ZEROS	EQUALS	ZEROVECS		# ZERO VECTOR ALWAYS IN HIGH MEMORY
LO6ZEROS	EQUALS	ZEROVEC			# ZERO VECTOR ALWAYS IN LOW MEMORY
HIDPHALF	EQUALS	UNITX
LODPHALF	EQUALS	XUNIT
HIDP1/4		EQUALS	DP1/4TH
LODP1/4		EQUALS	D1/4			# 2DEC .25
HIUNITX		EQUALS	UNITX
HIUNITY		EQUALS	UNITY
HIUNITZ		EQUALS	UNITZ
LOUNITX		EQUALS	XUNIT			# 2DEC .5
LOUNITY		EQUALS	YUNIT			# 2DEC 0
LOUNITZ		EQUALS	ZUNIT			# 2DEC 0
3/4LOWDP	EQUALS	3/4			# 2DEC 3.0 B-2
		SBANK=	LOWSUPER

; ============================================================================
; COMMAND MODULE ROPE-SPECIFIC ASSIGNMENTS
; ============================================================================
; These assignments define CSM-specific aliases for integration and navigation
; routines, eliminating runtime computer flag checks. In the Command Module
; rope (core memory), "THIS" refers to CSM and "OTHER" refers to LM, enabling
; shared code to operate correctly in both spacecraft without conditional logic.
; This compile-time specialization saves precious execution time and memory.
; ============================================================================

# ROPE SPECIFIC ASSIGNS OBVIATING NEED TO CHECK COMPUTER FLAG IN DETVRUZVING INTEGRATION AREA ENTRIES

OTHPREC		EQUALS	LEMPREC
ATOPOTH		EQUALS	ATOPLEM
ATOPTHIS	EQUALS	ATOPCSM
MOONTHIS	EQUALS	CMOONFLG

# Page 35

; Continuation of rope-specific assignments for CSM:
; MOONOTH = LM moon flag, MOVATHIS = CSM moving flag,
; STATEST = state estimate call, THISPREC = CSM precision orbit integration,
; THISAXIS = CSM axis definition, ERASID = erasable memory dump identifier,
; DELAYNUM = timing delay constant

MOONOTH		EQUALS	LMOONFLG
MOVATHIS	EQUALS	MOVEACSM
STATEST		EQUALS	V83CALL			# * TEMPORARY
THISPREC	EQUALS	CSMPREC
THISAXIS	=	UNITX
ERASID		EQUALS	LOW10			# DOWNLINK ERASABLE DUMP ID
DELAYNUM	EQUALS	THREE

; ============================================================================
; ECADRS (EXTENDED-BANK CALL ADDRESSES) FOR EBANK SWITCHING
; ============================================================================
; The following ECADRS facilitate erasable bank (EBANK) switching across the
; AGC's 2K RAM organized into banks. Each ECADR combines a fixed-bank address
; with its associated EBANK, enabling automatic bank switching during cross-
; bank calls. This architecture allows erasable memory reorganization without
; disrupting program logic, as programs reference symbolic ECADRS rather than
; absolute addresses. Critical for restart protection: after power transients
; or program alarms (like Apollo 11's 1202 alarm), the executive uses these
; ECADRS to restore both fixed and erasable bank contexts.
; ============================================================================

#***************************************************************************************************************

#   THE FOLLOWING ECADRS ARE DEFINED TO FACILITATE EBANK SWITCHING.  THEY ALSO MAKE IT EASIER FOR
# ERASABLE CONTROL TO REARRANGE ERASABLE MEMORY WITHOUT DISRUPTING THE PROGRAMS WHICH SET EBANKS.
# PRIOR TO ROPE RELEASE FIXED MEMORY CAN BE SAVED BY SETTING EACH EBXXXX =EBANKX (X=4,5,6,7).EBANKX OF COURSE
# WILL BE THE ERASABLES REFERENCED IN EBXXXX WILL BE STORED.

	; BANK 7 ECADRS - Optical Navigation Mark Buffers
; These ECADRS enable access to sextant mark data (MARKDOWN) and mark buffer
; storage (MRKBUF1) used during star/landmark sightings for navigation updates.

	BANK	7
		EBANK=	MARKDOWN
EBMARKDO	ECADR	MARKDOWN
		EBANK=	MRKBUF1
EBMRKBUF	ECADR	MRKBUF1

	; BANK 24 ECADRS - Burn Program Data
; DVCNTR: Delta-V counter for tracking velocity change accumulation during burns
; P40TMP: Temporary storage for P40-P47 SPS burn programs (TLI, LOI, TEI maneuvers)

	BANK	24
		EBANK=	DVCNTR
EBDVCNTR	ECADR	DVCNTR
		EBANK=	P40TMP
EBP40TMP	ECADR	P40TMP

	; BANK 34 ECADRS - Navigation State and Integration Data
; DVCNTR: Delta-V counter (alternate access for orbital integration routines)
; QPLACES: Quaternion storage for attitude representation during state propagation

	BANK	34
		EBANK=	DVCNTR
EBDVCNT		ECADR	DVCNTR
		EBANK=	QPLACES
EBQPLACE	ECADR	QPLACES

	; BANK 37 ECADRS - State Vector Data
; RN1: Position vector (R) for navigation state vector storage. Used by orbital
; integration and conic subroutines to propagate spacecraft trajectory through
; cislunar space during Apollo 11's journey from Earth to Moon and back.

	BANK	37
		EBANK=	RN1
EBRN1		ECADR	RN1

#***************************************************************************************************************

; ============================================================================
; END OF TAGS FOR RELATIVE SETLOC
; ============================================================================
; This file has established the complete memory organization framework for the
; Command Module AGC program Comanche 055. All fixed memory banks (0-43) have
; been assigned SETLOC tags, and ECADRS have been defined for critical cross-
; bank calls requiring automatic EBANK switching. This memory architecture
; successfully supported Apollo 11's historic lunar landing mission in July 1969.
; ============================================================================

# Page 36

# *** END OF MAIN PROGRAM ***
