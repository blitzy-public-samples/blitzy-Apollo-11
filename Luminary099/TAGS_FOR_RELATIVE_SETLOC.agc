# Copyright:	Public domain.
# Filename:	TAGS_FOR_RELATIVE_SETLOC.agc
# Purpose:	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
#
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim.lawton@gmail.com>
# Website:	www.ibiblio.org/apollo.
# Pages:	028-037
# Mod history:	2009-05-16	JVL	Transcribed from page images.
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
#    Assemble revision 001 of AGC program LMY99 by NASA 2021112-061
#    16:27 JULY 14, 1969

# Page 28
# TAGS FOR RELATIVE SETLOC AND BLANK BANK CARDS

; ============================================================================
; FILE: TAGS_FOR_RELATIVE_SETLOC.agc
; MODULE: Memory Organization Infrastructure
; MISSION PHASE: all
;
; TL;DR: Defines memory segment location tags (SETLOC tags) and bank 
;        assignments for organizing the Lunar Module's 36K words of fixed 
;        (core rope) memory across 6 modules spanning banks 0-43. Establishes
;        the foundation for relative addressing throughout the LM AGC program,
;        enabling modular code organization and efficient memory utilization.
;
; COMMENT-ONLY READERS: This file sets up the memory structure that organizes
;        all the guidance computer code into logical sections. Each "bank" is
;        a segment of memory, and this file maps which programs go where.
; CODE-ALONG READERS: Study this file to understand AGC fixed memory banking
;        architecture. SETLOC tags enable position-independent code assembly,
;        while BNKSUM directives provide bank checksum validation. The 6-module
;        organization reflects functional grouping of mission programs.
; ============================================================================

# 		COUNT	BANKSUM

; ============================================================================
; TRANSITION: Beginning of Memory Bank Organization
;
; The AGC's fixed memory (36K words of core rope ROM) is organized into banks
; of 1024 words each. The yaYUL assembler uses SETLOC tags defined here to
; place code modules at specific memory addresses during assembly. This allows
; programs to reference locations symbolically rather than with hard-coded
; addresses, enabling maintainable and relocatable code.
;
; Each MODULE groups related functional areas. During the Apollo 11 mission,
; when guidance programs called subroutines or accessed data, the AGC hardware
; automatically managed bank switching using these predefined memory layouts.
; ============================================================================

; ============================================================================
; MODULE 1: Core System Functions and Initial Programs
; BANKS: 0-5 (Fixed memory addresses 02000-05777 octal)
;
; This module contains fundamental system operations including restart logic,
; display interface foundations, basic orbital computations, keyboard handling,
; and initial mission programs. Bank 00 holds time-critical interrupt-driven
; code. Bank 01 contains restart recovery routines used throughout the mission.
; ============================================================================

# MODULE 1 CONTAINS BANKS 0 THROUGH 5

		BLOCK	02
; BLOCK 02 location tags for fixed-fixed memory segment.
; RADARFF: Radar lead-in routines for landing and rendezvous radar systems.
; FFTAG1-13: Fixed-fixed memory tags for interpreter routines, mathematical
;            subroutines, and navigation computation modules requiring fast
;            access without bank switching overhead.
RADARFF		EQUALS
FFTAG1		EQUALS
FFTAG2		EQUALS
FFTAG3		EQUALS
FFTAG4		EQUALS
FFTAG7		EQUALS
FFTAG8		EQUALS
FFTAG9		EQUALS
FFTAG10		EQUALS
FFTAG11		EQUALS
FFTAG12		EQUALS
FFTAG13		EQUALS
		BNKSUM	02

		BLOCK	03
; BLOCK 03 location tags for additional fixed-fixed memory segment.
; FFTAG5-6: Mathematical constant pools and single-precision subroutines.
FFTAG5		EQUALS
FFTAG6		EQUALS
		BNKSUM	03

		BANK	00
; BANK 00: Erasable memory and time-critical interrupt handlers.
; DLAYJOB: Delayed job scheduling for background task management.
DLAYJOB		EQUALS
		BNKSUM	00

		BANK	01
; BANK 01: System restart and recovery routines.
; RESTART: Restart protection tables and recovery logic, critical during
;          power transients or the 1202 alarm conditions experienced during
;          Apollo 11's lunar descent at mission time 102:38:26.
; LOADDAP1: Digital autopilot load routines for RCS control initialization.
RESTART		EQUALS
LOADDAP1	EQUALS
		BNKSUM	01

		BANK	04
; BANK 04: Display interface, keyboard handling, and basic orbital programs.
; R02: Display program R02 for orbit parameter monitoring.
; VERB37: Extended verb 37 implementation for program change requests.
; PINBALL4: DSKY (Display and Keyboard) interface routines for crew input.
; CONICS1: Keplerian orbit (conic section) calculation subroutines.
; KEYRUPT: Keyboard interrupt handler processing crew button presses.
; R36LM: Rendezvous navigation display program R36 for LM.
; UPDATE2: State vector update routines for ground uplink data.
; E/PROG: Program change (Enter/Program) logic.
; AOTMARK2: Alignment Optical Telescope mark processing (part 2).
R02		EQUALS
VERB37		EQUALS
PINBALL4	EQUALS
CONICS1		EQUALS
KEYRUPT		EQUALS
R36LM		EQUALS
UPDATE2		EQUALS
E/PROG		EQUALS
AOTMARK2	EQUALS
		BNKSUM	04

# Page 29
		BANK	05
; BANK 05: Restart, telemetry, abort logic, ephemerides, and ascent programs.
; FRANDRES: Fresh start and restart routines for system initialization.
; DOWNTELM: Downlink telemetry program formatting data for ground stations.
; ABORTS1: Abort program logic (P70/P71) for ascent abort scenarios.
; EPHEM1: Lunar and solar ephemeris calculations for navigation.
; ASENT3: Ascent guidance subroutines (part 3) for lunar surface launch.
FRANDRES	EQUALS
DOWNTELM	EQUALS
ABORTS1		EQUALS
EPHEM1		EQUALS
ASENT3		EQUALS
		BNKSUM	05

; ============================================================================
; MODULE 2: Guidance, Navigation, and Control Core
; BANKS: 6-13 (Fixed memory addresses 06000-13777 octal)
;
; This module contains the essential guidance and navigation algorithms that
; controlled the Lunar Module during critical mission phases. IMU compensation,
; gimbal control, ascent trajectories, and attitude management routines reside
; here. During Apollo 11's powered descent and ascent, these programs computed
; the thrust commands and attitude angles that safely landed Eagle and returned
; it to lunar orbit for rendezvous with Columbia.
; ============================================================================

# MODULE 2 CONTAINS BANKS 6 THROUGH 13

		BANK	06
; BANK 06: IMU compensation, timing interrupts, RCS monitoring, gimbal control.
; IMUCOMP: Inertial Measurement Unit compensation for gyro drift and
;          accelerometer bias corrections.
; T4RUP: Timer 4 interrupt (T4RUPT) program executing every 10 milliseconds
;        for WAITLIST task scheduling and IMU counter sampling.
; RCSMONT: Reaction Control System monitoring for thruster performance.
; MIDDGIM: Middle gimbal angle calculations for gimbal lock avoidance.
; EARTHLOC: Earth location computations for antenna pointing.
IMUCOMP		EQUALS
T4RUP		EQUALS
RCSMONT		EQUALS
MIDDGIM		EQUALS
EARTHLOC	EQUALS
		BNKSUM	06

		BANK	07
; BANK 07: Optical navigation, mode switching, and ascent guidance.
; AOTMARK1: Alignment Optical Telescope mark processing (part 1) for
;           star sighting and platform alignment verification.
; MODESW: Mode switching logic for transitioning between mission phases.
; ASENT2: Ascent guidance subroutines (part 2) for lunar launch trajectory.
AOTMARK1	EQUALS
MODESW		EQUALS
ASENT2		EQUALS
		BNKSUM	07

		BANK	10
; BANK 10: Interpreter support, display management, phase tracking, and IMU.
; RTBCODES: Return-to-bank codes for interpreter RTB (Return To Basic) opcodes.
; DISPLAYS: Display interface routines for DSKY numerical output formatting.
; PHASETAB: Phase table maintenance for mission program restart protection.
; FLESHLOC: Flash display location management for crew alerts.
; SLCTMU: Select IMU mode switching routines.
RTBCODES	EQUALS
DISPLAYS	EQUALS
PHASETAB	EQUALS
FLESHLOC	EQUALS
SLCTMU		EQUALS
		BNKSUM	10

		BANK	11
; BANK 11: Orbital mechanics and integration velocity calculations.
; ORBITAL: Orbital integration primary routines for position/velocity propagation.
; F2DPS*11: Descent Propulsion System interface routines (part 11) for thrust
;           vector control during powered flight phases.
; INTVEL: Integration velocity computations for trajectory state updates.
ORBITAL		EQUALS
F2DPS*11	EQUALS
INTVEL		EQUALS
		BNKSUM	11

		BANK	12
; BANK 12: Conic subroutines, orbital mechanics extended, interpreter support.
; CONICS: Conic section calculations for two-body orbital mechanics (Keplerian
;         elements, position/velocity transformations, Lambert targeting).
; ORBITAL1: Orbital integration subroutines (part 1) for perturbed motion.
; INTPRET2: Interpreter support routines (part 2) for virtual machine opcodes.
CONICS		EQUALS
ORBITAL1	EQUALS
# Page 30
INTPRET2	EQUALS
		BNKSUM	12

		BANK	13
; BANK 13: Coordinate transformations, integration initialization, geometry.
; LATLONG: Latitude/longitude subroutines for planetary surface coordinates.
; INTINIT: Integration initialization routines (Encke method setup for precision).
; LEMGEOM: Lunar Module geometry constants (dimensions, thruster locations,
;          center-of-gravity, inertia tensor).
; P76LOC: Program 76 location (target Delta-V computation for external burns).
; ORBITAL2: Orbital integration subroutines (part 2) for trajectory propagation.
; ABTFLGS: Abort flag management for contingency mode tracking.
LATLONG		EQUALS
INTINIT		EQUALS
LEMGEOM		EQUALS
P76LOC		EQUALS
ORBITAL2	EQUALS
ABTFLGS		EQUALS
		BNKSUM	13

# Page 31

; ============================================================================
; MODULE 3: Digital Autopilot and Mission Programs
; BANKS: 14-21 (Fixed memory addresses 14000-21777 octal)
;
; This module contains the Digital AutoPilot (DAP) routines that controlled
; Lunar Module attitude using the Reaction Control System (RCS) thrusters.
; The DAP managed spacecraft rotation, translation, and attitude hold during
; all flight phases. Also includes mission programs P40-P47 (engine burns),
; P50 series (star sightings), ephemeris calculations, and ascent guidance.
; During Apollo 11's final approach, these routines maintained Eagle's precise
; orientation while Armstrong selected the landing site.
; ============================================================================

# MODULE 3 CONTAINS BANKS 14 THROUGH 21

		BANK 	14
; BANK 14: Star navigation programs and ascent guidance.
; P50S1: Program 50 series (part 1) for IMU alignment using star sightings
;        (P51 manual optics, P52 automatic optics, P53 backup alignment).
; STARTAB: Star catalog table containing celestial coordinates and magnitudes
;          for navigation stars used in optical alignment procedures.
; ASENT4: Ascent guidance routines (part 4) for lunar launch to rendezvous.
P50S1		EQUALS
STARTAB		EQUALS
ASENT4		EQUALS
		BNKSUM	14

		BANK	15
; BANK 15: IMU alignment programs and celestial ephemerides.
; P50S: Program 50 series main routines for platform realignment using optical
;       telescope star tracking and gyrocompass techniques.
; EPHEM: Ephemeris subroutines for computing lunar and solar position vectors
;        from polynomial approximations, used in navigation state updates.
P50S		EQUALS
EPHEM		EQUALS
		BNKSUM	15

		BANK	16
; BANK 16: Digital Autopilot primary routines (part 1).
; DAPS1: Digital AutoPilot System primary control logic for RCS thruster firing.
;        Implements attitude hold, rate damping, and minimum impulse mode to
;        conserve propellant. During descent, DAPS maintained LM orientation
;        while throttle control managed vertical velocity.
DAPS1		EQUALS
		BNKSUM	16

		BANK	17
; BANK 17: Digital Autopilot extended routines and engine burn programs.
; DAPS2: Digital AutoPilot System routines (part 2) for advanced attitude
;        control including jet selection logic and propellant balancing.
; P40S3: Program 40 series (part 3) for DPS/APS engine burns including ignition
;        sequencing, thrust monitoring, and cutoff logic (P40 DPS, P42 APS).
DAPS2		EQUALS
P40S3		EQUALS
		BNKSUM	17

		BANK	20
; BANK 20: Digital Autopilot configuration and rate-of-descent monitoring.
; DAPS3: Digital AutoPilot System routines (part 3) for autopilot executive
;        coordination with mission programs.
; LOADDAP: Load DAP configuration parameters (control gains, deadbands, jet
;          selection criteria) appropriate for current mission phase.
; RODTRAP: Rate-Of-Descent trap logic for landing phase monitoring, checking
;          vertical velocity against altitude to ensure safe touchdown approach.
DAPS3		EQUALS
LOADDAP		EQUALS
RODTRAP		EQUALS
		BNKSUM	20

		BANK	21
; BANK 21: Digital Autopilot final routines and rendezvous programs.
; DAPS4: Digital AutoPilot System routines (part 4) completing DAP functionality.
; R10: Routine R10 for IMU attitude error display on DSKY.
; R11: Routine R11 for backup IMU attitude computation using optical sightings.
DAPS4		EQUALS
R10		EQUALS
R11		EQUALS
		BNKSUM	21
# Page 32

; ============================================================================
; MODULE 4: Rendezvous Navigation and Landing Programs
; BANKS: 22-27 (Fixed memory addresses 22000-27777 octal)
;
; This module contains mission-critical rendezvous navigation programs (P20-P25)
; for relative state estimation between LM and CM using radar tracking. Also
; includes powered flight subroutines, measurement incorporation for navigation
; state updates, Kalman filtering for optimal estimation, landing constants,
; and planetary inertial orientation routines. During Apollo 11's ascent and
; rendezvous, these routines tracked Columbia's position and computed the
; precise trajectory for docking in lunar orbit.
; ============================================================================

# MODULE 4 CONTAINS BANKS 22 THROUGH 27

		BANK	22
; BANK 22: Kalman filter monitoring, rendezvous programs, and landing constants.
; KALCMON1: Kalman filter monitor (part 1) for navigation state covariance check.
; KALCMON2: Kalman filter monitor (part 2) for measurement residual validation.
; R30LOC: Routine R30 for orbit parameter display (apogee/perigee/period).
; RENDEZ: Rendezvous programs coordination and sequencing logic.
; SERV2: Service routines (part 2) for utility functions and time conversions.
; LANDCNST: Landing constants defining descent trajectory parameters, site
;           coordinates, and guidance gains for lunar touchdown sequence.
KALCMON1	EQUALS
KALCMON2	EQUALS
R30LOC		EQUALS
RENDEZ		EQUALS
SERV2		EQUALS
LANDCNST	EQUALS
		BNKSUM	22

		BANK	23
; BANK 23: Powered flight navigation, measurement incorporation, extended verbs.
; POWFLITE: Powered flight trajectory integration and state extrapolation during
;           engine burns (descent, ascent, orbital maneuvers).
; POWFLIT1: Powered flight subroutines (part 1) for thrust integration.
; INFLIGHT: In-flight alignment routines for IMU platform realignment.
; APOPERI: Apoapsis/periapsis computation from current state vector.
; R61: Routine R61 for radar test and checkout procedures.
; R62: Routine R62 for rendezvous radar self-test display.
; INTPRET1: Interpreter utility routines (part 1) for virtual machine support.
; MEASINC: Measurement incorporation for navigation state updates with sensor
;          data (radar, optics) using Kalman filtering techniques.
; MEASINC1: Measurement incorporation (part 1) state covariance propagation.
; EXTVB1: Extended verbs (part 1) implementing DSKY verb functions beyond basic set.
; P12A: Program P12 ascent routines for powered launch from lunar surface.
; NORMLIZ: Normalization routines for vector operations and scaling.
; ASENT7: Ascent guidance (part 7) completing lunar liftoff trajectory logic.
POWFLITE	EQUALS
POWFLIT1	EQUALS
INFLIGHT	EQUALS
APOPERI		EQUALS
R61		EQUALS
R62		EQUALS
INTPRET1	EQUALS
MEASINC		EQUALS
MEASINC1	EQUALS
EXTVB1		EQUALS
P12A		EQUALS
NORMLIZ		EQUALS
ASENT7		EQUALS
		BNKSUM	23

		BANK	24
; BANK 24: Planetary inertial orientation and rendezvous navigation primary.
; PLANTIN: Planetary inertial orientation routines for coordinate frame
;          transformations between inertial and rotating reference frames,
;          REFSMMAT usage, and attitude reference computations.
; P20S: Program 20 series (primary) for complete rendezvous navigation suite
;       including relative state estimation, radar tracking, and targeting
;       computation essential for LM-CM rendezvous in lunar orbit.
PLANTIN		EQUALS
P20S		EQUALS
		BNKSUM	24

		BANK	25
; BANK 25: Rendezvous navigation extended, radar processing, coordinate frames.
; P20S1: Program 20 series (part 1) extended rendezvous navigation logic.
; P20S2: Program 20 series (part 2) continuing rendezvous computation routines.
; RADARUPT: Radar data interrupt handler for processing rendezvous radar updates.
; RRLEADIN: Rendezvous Radar lead-in routines for range and range-rate measurements
;           used in tracking Columbia during ascent and rendezvous phases.
; R29S1: Routine R29 (part 1) for rendezvous out-of-plane display.
; PLANTIN3: Planetary inertial orientation (part 3) additional frame transforms.
P20S1		EQUALS
P20S2		EQUALS
RADARUPT	EQUALS
RRLEADIN	EQUALS
R29S1		EQUALS
PLANTIN3	EQUALS
		BNKSUM	25

		BANK	26
# Page 33
; BANK 26: Rendezvous navigation completion, attitude maneuvers, coordinate frames.
; P20S3: Program 20 series (part 3) completing rendezvous navigation suite.
; BAWLANGS: Body Axis Wide Angle Gimbal System routines for attitude computations.
; MANUVER: Automated attitude maneuver routines for crew-commanded rotations and
;          inertial hold modes during coast and rendezvous phases.
; MANUVER1: Maneuver routines (part 1) for maneuver initialization and execution.
; PLANTIN1: Planetary inertial orientation (part 1) core coordinate transforms.
; PLANTIN2: Planetary inertial orientation (part 2) extended transformation logic.
P20S3		EQUALS
BAWLANGS	EQUALS
MANUVER		EQUALS
MANUVER1	EQUALS
PLANTIN1	EQUALS
PLANTIN2	EQUALS
		BNKSUM	26

		BANK	27
; BANK 27: Time of free fall, burn programs, vector operations, ascent, services.
; TOF-FF: Time Of Free Fall trajectory calculations for coast arc prediction and
;         Lambert problem variations in orbital transfer targeting.
; TOF-FF1: Time of free fall routines (part 1) for time-of-flight computations.
; P40S1: Program 40 series (part 1) for engine burn sequences (DPS/APS ignition,
;        ullage, monitoring, cutoff) used during descent and ascent maneuvers.
; VECPT: Vector point operations for coordinate transformations and rotations.
; ASENT1: Ascent guidance (part 1) for lunar surface launch trajectory computations.
; SERV3: Service routines (part 3) for additional utility functions.
TOF-FF		EQUALS
TOF-FF1		EQUALS
P40S1		EQUALS
VECPT		EQUALS
ASENT1		EQUALS
SERV3		EQUALS
		BNKSUM	27

# Page 34

; ============================================================================
; MODULE 5: Powered Ascent, Throttle Control, and Landing Programs
; BANKS: 30-35 (Fixed memory addresses 30000-35777 octal)
;
; This module contains powered ascent programs (P12) for Eagle's launch from
; the lunar surface, descent/ascent throttle control routines managing engine
; commands, landing guidance computations including the fuel-optimal trajectory
; algorithms, lunar landing programs (THE LUNAR LANDING implementing P63 braking
; phase), abort programs (P70-P71), service routines, and analog display formatting.
; The code in this module executed during the most critical 12 minutes of Apollo 11:
; the powered descent to the lunar surface on July 20, 1969, culminating in
; Armstrong's "The Eagle has landed" at 102:45:40 mission elapsed time.
; ============================================================================

# MODULE 5 CONTAINS BANKS 30 THROUGH 35

		BANK	30
; BANK 30: Powered ascent program, guidance interface, and display routines.
; LOWSUPER: Low-level supervisory routines for program coordination and sequencing.
; P12: Program 12 for powered ascent from lunar surface implementing launch trajectory
;      guidance used during Eagle's liftoff on July 21, 1969 for rendezvous with Columbia.
; ASENT: Ascent guidance primary routines for vertical rise, pitchover, and closed-loop
;        guidance to orbital insertion targeting.
; FCDUW: FINDCDUW guidance-DAP interface for thrust vector coordination between guidance
;        commands and digital autopilot thruster/gimbal control.
; FLOGSUB: Flight log subroutines for mission event recording and telemetry.
; VB67A: Verb 67 implementation for W-matrix display and navigation covariance monitoring.
; ASENT5: Ascent guidance (part 5) for trajectory phase management.
LOWSUPER	EQUALS
P12		EQUALS
ASENT		EQUALS
FCDUW		EQUALS
FLOGSUB		EQUALS
VB67A		EQUALS
ASENT5		EQUALS
		BNKSUM	30

		BANK	31
; BANK 31: Throttle control and force-to-DPS interface for descent/ascent engines.
; FTHROT: Fine throttle control routines for precise DPS engine throttle management
;         during powered descent final approach and touchdown phases.
; F2DPS*31: Force-to-DPS interface (part 31) converting guidance thrust commands to
;           engine throttle settings with response lag compensation and smoothing.
; VB67: Verb 67 display routines for W-matrix and navigation state covariance.
FTHROT		EQUALS
F2DPS*31	EQUALS
VB67		EQUALS
		BNKSUM	31

		BANK	32
; BANK 32: Rendezvous navigation, abort programs, landing radar, and services.
; P20S4: Program 20 series (part 4) completing rendezvous navigation computations.
; F2DPS*32: Force-to-DPS interface (part 32) for engine control coordination.
; ABORTS: Abort program logic (P70-P71) for emergency ascent from descent trajectory,
;         providing crew with abort capability throughout powered descent phase.
; LRS22: Landing Radar System routines for altitude and velocity data processing
;        critical during lunar approach and landing operations.
; P66LOC: Program 66 location (P66 rate-of-descent landing program).
; R47: R47 routine for specific navigation or display functions.
; SERV: Service utility routines for general program support operations.
P20S4		EQUALS
F2DPS*32	EQUALS
ABORTS		EQUALS
LRS22		EQUALS
P66LOC		EQUALS
R47		EQUALS
SERV		EQUALS
		BNKSUM	32

		BANK	33
; BANK 33: Service routines and ascent guidance extensions.
; SERVICES: Extended service utility routines for general mission program support,
;           time conversions, and common mathematical operations.
; R29/SERV: R29 routine with service function integration for specialized computations.
; ASENT6: Ascent guidance (part 6) for trajectory phase management and targeting.
SERVICES	EQUALS
R29/SERV	EQUALS
ASENT6		EQUALS
		BNKSUM	33

		BANK	34
; BANK 34: Orbit determination, external ΔV programs, rendezvous targeting, filtering.
; STBLEORB: Stable orbit determination routines for coast phase monitoring and orbit
;           decay prediction during lunar orbital operations.
; P30S1: Program 30 series (part 1) for external ΔV maneuver planning and targeting.
; CSI/CDH1: Coelliptic Sequence Initiation and Constant Delta Height routines (part 1)
;           for rendezvous height adjustment maneuvers (CSI/CDH burns).
; ASCFILT: Ascent filter for navigation state estimation during powered ascent phase.
; R12STUFF: R12 routine support functions for program coordination.
; SERV4: Service routines (part 4) for additional utility operations.
STBLEORB	EQUALS
P30S1		EQUALS
CSI/CDH1	EQUALS
ASCFILT		EQUALS
R12STUFF	EQUALS
SERV4		EQUALS
		BNKSUM	34

# Page 35
		BANK	35
; BANK 35: Rendezvous targeting, external ΔV programs, Lambert guidance, burn programs.
; CSI/CDH: Coelliptic Sequence Initiation and Constant Delta Height primary routines
;          for rendezvous maneuver targeting (CSI and CDH burn computations).
; P30S: Program 30 series routines for external ΔV program suite and burn targeting.
; GLM: General Lambert aimpoint guidance for orbit transfer trajectory computations
;      using Lambert targeting (two-point boundary value problem solutions).
; P40S2: Program 40 series (part 2) for engine burn programs and sequences.
CSI/CDH		EQUALS
P30S		EQUALS
GLM		EQUALS
P40S2		EQUALS
		BNKSUM	35

# Page 36
# MODULE 6 CONTAINS BANKS 36 THROUGH 43
;
; MODULE 6: Mission program execution, IMU operations, display/keyboard interface,
;           self-test diagnostics, and extended verb implementations.
; Memory Banks: 36-43 (8 banks)
; Mission Phase: All phases - core crew interface and system diagnostics
; Key Functions: P40 series burn programs, IMU calibration, DSKY pinball interface,
;                self-check routines, extended verb command processing, S-band antenna
;                control for communications with Mission Control.

		BANK	36
; BANK 36: Engine burn program suite (P40 series primary routines).
; P40S: Program 40 series main routines for DPS (Descent Propulsion System) and
;       APS (Ascent Propulsion System) burn programs including ullage, ignition,
;       monitoring, and cutoff sequences for orbital maneuvers and powered flight.
P40S		EQUALS
		BNKSUM	36

		BANK	37
; BANK 37: IMU operations, navigation programs, and service routines.
; P05P06: Programs 05 and 06 for optical navigation and IMU alignment verification.
; IMU2: Inertial Measurement Unit routines (part 2) for gyro and accelerometer data
;       processing, coordinate transformations, and platform stability monitoring.
; IMU4: IMU routines (part 4) for extended IMU operations and calibration.
; R31: R31 routine for orbital parameter display (apogee/perigee, period, etc.).
; IMUSUPER: IMU supervisor routines coordinating IMU mode transitions and operations.
; SERV1: Service routines (part 1) providing utility functions for program support.
P05P06		EQUALS
IMU2		EQUALS
IMU4		EQUALS
R31		EQUALS
IMUSUPER	EQUALS
SERV1		EQUALS
		BNKSUM	37

		BANK	40
; BANK 40: DSKY pinball interface, self-test supervisor, and display routines.
; PINBALL1: Pinball interface routines (part 1) for DSKY button handling and display
;           update state machine. "Pinball" refers to the blinking display lights and
;           button interactions used for crew communication with the AGC.
; SELFSUPR: Self-test supervisor coordinating AGC self-check diagnostic sequences.
; PINSUPER: Pinball supervisor routines managing DSKY interface state and coordination.
; R31LOC: R31 routine location for orbital parameter display implementation.
PINBALL1	EQUALS
SELFSUPR	EQUALS
PINSUPER	EQUALS
R31LOC		EQUALS
		BNKSUM	40

		BANK	41
; BANK 41: DSKY pinball interface continuation (part 2).
; PINBALL2: Pinball interface routines (part 2) continuing DSKY verb/noun processing,
;           display formatting, flash patterns, and crew input validation logic.
PINBALL2	EQUALS
		BNKSUM	41

		BANK	42
; BANK 42: S-band antenna control and DSKY pinball interface (part 3).
; SBAND: S-band high-gain antenna pointing control for communications with Earth.
;        Manages antenna tracking to maintain communications link with Mission Control
;        during all mission phases including lunar orbit and surface operations.
; PINBALL3: Pinball interface routines (part 3) for additional DSKY display functions.
SBAND		EQUALS
PINBALL3	EQUALS
		BNKSUM	42

		BANK	43
; BANK 43: Extended verb implementations and self-check diagnostics.
; EXTVERBS: Extended DSKY verb implementations (V01-V99) defining verb functions,
;           noun requirements, and crew command procedures for mission operations.
;           Verbs control AGC operations through DSKY keyboard input.
; SELFCHEC: Self-check diagnostic routines for AGC hardware validation, instruction
;           testing, memory verification, and error detection/reporting.
EXTVERBS	EQUALS
SELFCHEC	EQUALS
		BNKSUM	43

# Page 37
;
; ============================================================================
; SECTION: MEMORY ALIASES AND LM-SPECIFIC CONSTANT DEFINITIONS
;
; This final section defines memory location aliases and LM-specific constant
; assignments that enable code portability between Command Module (Comanche)
; and Lunar Module (Luminary) while maintaining computational precision through
; proper memory bank addressing.
;
; Purpose: Establishes high/low memory aliases for commonly-used mathematical
;          constants and vectors, ensuring computational routines access correct
;          memory locations regardless of addressing mode restrictions.
; ============================================================================

; HIGH AND LOW MEMORY VECTOR ALIASES
; AGC memory addressing requires careful bank management. These aliases ensure
; vector/constant access works correctly whether code executes in high-address
; banks (fixed memory) or low-address banks (erasable memory).
;
; Zero vectors: Used for initialization and coordinate frame setup in guidance.
; Unit vectors: Define coordinate axes (X/Y/Z) for spacecraft attitude and position.
; Fractional constants: Provide scaled fixed-point values for mathematical operations.
HI6ZEROS	EQUALS	ZEROVECS		# ZERO VECTOR ALWAYS IN HIGH MEMORY
LO6ZEROS	EQUALS	ZEROVEC			# ZERO VECTOR ALWAYS IN LOW MEMORY
; HIDPHALF/LODPHALF: Double-precision 0.5 constant in high/low memory (scaled value).
HIDPHALF	EQUALS	UNITX
LODPHALF	EQUALS	XUNIT
; HIDP1/4/LODP1/4: Double-precision 0.25 constant (2DEC .25 = fractional value
;                  scaled for fixed-point arithmetic in AGC 16-bit word format).
HIDP1/4		EQUALS	DP1/4TH
LODP1/4		EQUALS	D1/4			# 2DEC .25
; HIUNITX/Y/Z: High-memory unit vectors defining X, Y, Z coordinate axes.
;              Used in coordinate transformations and attitude computations.
HIUNITX		EQUALS	UNITX
HIUNITY		EQUALS	UNITY
HIUNITZ		EQUALS	UNITZ
; LOUNITX/Y/Z: Low-memory unit vectors for erasable-bank computational routines.
;              XUNIT = 0.5, YUNIT = 0, ZUNIT = 0 (scaled fractional representations).
LOUNITX		EQUALS	XUNIT			# 2DEC .5
LOUNITY		EQUALS	YUNIT			# 2DEC 0
LOUNITZ		EQUALS	ZUNIT			# 2DEC 0

; SUBROUTINE RETURN SPLICE ALIAS
; DELRSPL: Return address for COL (Coelliptic) program routines.
;          Also called by R30 (orbit parameter display) in Luminary LM software.
;          Provides common return point for multiple calling programs.
DELRSPL		EQUALS	SPLRET			# COL PGM, ALSO CALLED BY R30 IN LUMINARY.

# ROPE-SPECIFIC ASSIGNS OBVIATING NEED TO CHECK COMPUTER FLAG IN DETERMINING INTEGRATION AREA ENTRIES.
;
; LM-SPECIFIC CONFIGURATION ALIASES
; These assignments differentiate Lunar Module (LM/Luminary) from Command Module
; (CSM/Comanche) without requiring runtime computer flag checks. Core rope memory
; is assembled with LM-specific values, enabling efficient code execution during
; time-critical mission phases like lunar descent and ascent.
;
; "THIS" refers to Lunar Module, "OTH" refers to Command Module.
; Integration routines use these aliases to process state vectors for both
; spacecraft during rendezvous operations after LM ascent from lunar surface.

; ATOPTHIS/ATOPOTH: Atmosphere entry parameters for LM and CSM respectively.
;                   LM does not perform atmospheric entry (remains in space or
;                   impacts lunar/Earth surface), CSM performs Earth reentry.
ATOPTHIS	EQUALS	ATOPLEM
ATOPOTH		EQUALS	ATOPCSM
; OTHPREC: CSM precision integration parameters for orbital computations.
OTHPREC		EQUALS	CSMPREC
; MOONTHIS/MOONOTH: Lunar sphere of influence flags for LM and CSM.
;                   Determines which gravitational body (Earth or Moon) dominates
;                   for integration accuracy during translunar/transearth flight.
MOONTHIS	EQUALS	LMOONFLG
MOONOTH		EQUALS	CMOONFLG
; MOVATHIS: State vector moving average routine specific to LM navigation.
MOVATHIS	EQUALS	MOVEALEM
; RMM/RME: Lunar and Earth radial distance maximum precision limits.
;          RMM = Moon radius maximum, RME = Earth radius maximum.
;          Used for integration rectification and coordinate frame transitions.
RMM		=	LODPMAX
RME		=	LODPMAX1
; THISPREC: LM precision integration parameters defining numerical integration
;           accuracy requirements for LM orbital mechanics computations.
THISPREC	EQUALS	LEMPREC
; THISAXIS: LM body axis definition (Z-axis = thrust axis for descent/ascent engines).
;           Used in attitude control and thrust vector computations.
THISAXIS	=	UNITZ
; NB1NB2: Navigation base axis definition for R31 (display orbit parameters).
NB1NB2		EQUALS	THISAXIS		# FOR R31
; ERASID: Erasable memory downlink dump identification (bits 2-10 pattern).
;         Used in telemetry to identify erasable memory contents in downlink.
ERASID		EQUALS	BITS2-10		# DOWNLINK ERASABLE DUMP ID
; DELAYNUM: Delay count constant for timing loops (value = 2).
DELAYNUM	EQUALS	TWO

