; ============================================================================
; FILE: MAIN.agc
; MODULE: Luminary099 Master Assembly File
; MISSION PHASE: All phases (launch/earth-orbit/trans-lunar/lunar-orbit/descent/landing/ascent/rendezvous/trans-earth)
;
; TL;DR: Top-level assembly control file for the Luminary099 Lunar Module AGC
;        flight software. Includes all 89 source files in carefully ordered
;        sequence to build the complete LM program that flew aboard Eagle
;        during Apollo 11's historic lunar landing on July 20, 1969. Defines
;        overall program structure and memory organization for LM mission operations.
;
; COMMENT-ONLY READERS: This file shows the complete software architecture
;        that controlled the Lunar Module from separation through landing,
;        surface operations, ascent, and rendezvous. Read file names to
;        understand the scope of systems managed by the AGC.
; CODE-ALONG READERS: Study the inclusion order to understand AGC assembly
;        structure: memory setup first, then core OS, then mission programs,
;        then control systems. This ordering reflects dependencies and memory
;        bank allocation strategy.
; ============================================================================
;
; HISTORICAL CONTEXT
;
; This is the Luminary099 assembly - the actual flight software configuration
; that flew aboard the Apollo 11 Lunar Module "Eagle" in July 1969. This code
; successfully guided Neil Armstrong and Buzz Aldrin to humanity's first lunar
; landing at Tranquility Base.
;
; The assembly structure reflects the constraints of the AGC Block II hardware:
; - 2,048 words (2K) of erasable memory (RAM) for variables and computation
; - 36,864 words (36K) of fixed memory (core rope ROM) for program code
; - Memory organized in banks requiring careful placement of code modules
; - Real-time operating system managing guidance, navigation, and control
;
; File inclusion order is critical: erasable memory must be allocated before
; code that references it, core operating system must be placed before mission
; programs that use it, and interrupt handlers must be positioned in specific
; memory locations.
;
; This master file was assembled using the yaYUL assembler (successor to the
; original YUL and GAP assemblers used at MIT Instrumentation Laboratory).
; The resulting binary was loaded into AGC core rope memory - literally woven
; by hand into copper wire arrays at Raytheon Manufacturing.
;
; Total program size: Approximately 1,743 pages of assembly code
; Source: MIT Instrumentation Laboratory Report R-567
; Flight Configuration: Luminary 1A (Revision 099)
; Mission: Apollo 11 Lunar Module
; Spacecraft: LM-5 "Eagle"
;
; ============================================================================
; SECTION 1: ASSEMBLY INFORMATION AND MEMORY STRUCTURE
; ============================================================================
;
; These files establish the foundational structure of the AGC program:
; assembly directives, memory organization tags, and core constants that
; define how the entire system operates.

$ASSEMBLY_AND_OPERATION_INFORMATION.agc		# pp. 1-27
$TAGS_FOR_RELATIVE_SETLOC.agc			# pp. 28-37

; Controlled constants define mission-specific parameters: landing site
; coordinates, trajectory parameters, spacecraft mass properties, and
; operational limits used throughout LM guidance and control algorithms.
$CONTROLLED_CONSTANTS.agc			# pp. 38-53

; I/O channel bit descriptions map AGC memory-mapped I/O to physical
; spacecraft systems: engine controls, RCS thrusters, radar interfaces,
; DSKY displays, IMU channels, and sensor inputs.
$INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc	# pp. 54-60

; Flagword assignments define software state flags controlling program
; modes, system configurations, and operational states throughout mission.
$FLAGWORD_ASSIGNMENTS.agc			# pp. 61-88
						# p.  89 is a GAP-generated table

; ============================================================================
; SECTION 2: ERASABLE MEMORY ALLOCATION
; ============================================================================
;
; Erasable memory (RAM) allocation must be defined before any code that
; references variables. This 2K word memory space holds navigation state,
; guidance parameters, control system variables, display buffers, and
; operating system structures.
;
; Memory organization is critical for the AGC's bank-switching architecture.
; Variables are grouped by function and usage patterns to minimize bank
; switching overhead during time-critical operations.

$ERASABLE_ASSIGNMENTS.agc			# pp. 90-152

; ============================================================================
; SECTION 3: INTERRUPT HANDLERS AND SYSTEM SERVICES
; ============================================================================
;
; Real-time interrupt handlers form the foundation of AGC operations. These
; routines execute at precise intervals and priorities, managing time-critical
; tasks like IMU sampling, display updates, and telemetry. The T4RUPT interrupt
; runs every 10 milliseconds and drives the entire real-time operating system.
;
; During Apollo 11's descent, these interrupt handlers managed the computational
; load that led to the famous 1202 program alarms - the system was handling
; radar data updates, guidance computations, and display updates simultaneously.

$INTERRUPT_LEAD_INS.agc				# pp. 153-154
$T4RUPT_PROGRAM.agc				# pp. 155-189
$RCS_FAILURE_MONITOR.agc			# pp. 190-192
$DOWNLINK_LISTS.agc				# pp. 193-205
$AGS_INITIALIZATION.agc				# pp. 206-210

; Fresh start and restart capabilities enable the AGC to recover from power
; transients, program errors, or crew-commanded resets without losing critical
; mission state. This restart protection was crucial during the 1202 alarms.
$FRESH_START_AND_RESTART.agc			# pp. 211-237
$RESTART_TABLES.agc				# pp. 238-243

; ============================================================================
; SECTION 4: NAVIGATION, SENSORS, AND ATTITUDE CONTROL
; ============================================================================
;
; Navigation and sensor management routines enable the LM to determine its
; position, velocity, and orientation in space. The Inertial Measurement Unit
; (IMU) provides gyroscope and accelerometer data, while the Alignment Optical
; Telescope (AOT) enables star sighting for platform alignment.
;
; These systems worked continuously throughout the mission, from Earth orbit
; separation through lunar landing, providing the navigation data that guided
; Armstrong and Aldrin to Tranquility Base.

$AOTMARK.agc					# pp. 244-261

; Extended verbs and noun tables define the DSKY (Display and Keyboard)
; interface that allowed the crew to monitor systems and command operations.
; Verb/noun combinations like V16N68 (display altitude and altitude rate)
; were critical during descent monitoring.
$EXTENDED_VERBS.agc				# pp. 262-300
$PINBALL_NOUN_TABLES.agc			# pp. 301-319

; LM geometry definitions specify spacecraft physical dimensions, mass
; properties, thruster locations, and IMU mounting orientation - essential
; for accurate guidance and control computations.
$LEM_GEOMETRY.agc				# pp. 320-325

; IMU compensation corrects for gyro drift and accelerometer bias, maintaining
; navigation accuracy throughout the mission.
$IMU_COMPENSATION_PACKAGE.agc			# pp. 326-337

$R63.agc					# pp. 338-341

; Attitude maneuver routines enable the LM to rotate to desired orientations
; using RCS thrusters, with gimbal lock avoidance ensuring the IMU gimbals
; never reach singularity positions.
$ATTITUDE_MANEUVER_ROUTINE.agc			# pp. 342-363
$GIMBAL_LOCK_AVOIDANCE.agc			# p.  364
$KALCMANU_STEERING.agc				# pp. 365-369

; System test routines enable preflight verification and inflight diagnostics
; of critical systems.
$SYSTEM_TEST_STANDARD_LEAD_INS.agc		# pp. 370-372
$IMU_PERFORMANCE_TEST_2.agc			# pp. 373-381
$IMU_PERFORMANCE_TESTS_4.agc			# pp. 382-389

; ============================================================================
; SECTION 5: CREW INTERFACE AND DISPLAY SYSTEMS
; ============================================================================
;
; The DSKY (Display and Keyboard) provided the primary human-computer interface
; during the mission. This "Pinball Game" code managed button presses, display
; updates, and the complex verb/noun state machine that Armstrong and Aldrin
; used to monitor systems and enter commands throughout descent and ascent.

$PINBALL_GAME_BUTTONS_AND_LIGHTS.agc		# pp. 390-471

; Radar self-test and S-band antenna pointing routines maintain communication
; and sensor functionality.
$R60_62.agc					# pp. 472-485
$S-BAND_ANTENNA_FOR_LM.agc			# pp. 486-489

; Landing and rendezvous radar lead-in routines interface with the radar
; systems that provided altitude, velocity, and range data during descent
; and rendezvous operations.
$RADAR_LEADIN_ROUTINES.agc			# pp. 490-491

; ============================================================================
; SECTION 6: RENDEZVOUS AND ORBITAL NAVIGATION PROGRAMS
; ============================================================================
;
; These programs enabled the critical rendezvous between the ascending Lunar
; Module and the orbiting Command Module. After landing and surface operations,
; Eagle used these programs to target, execute, and navigate the ascent and
; rendezvous that reunited Armstrong and Aldrin with Michael Collins in Columbia.
;
; Rendezvous navigation (P20-P25) tracked relative position and velocity,
; while targeting programs (P30-P37, P32-P35) computed the precise burns needed
; for orbital insertion and rendezvous maneuvers.

$P20-P25.agc					# pp. 492-614
$P30_P37.agc					# pp. 615-617
$P32-P35_P72-P75.agc				# pp. 618-650

; Lambert targeting solves the two-point boundary value problem: given current
; position and desired target position, compute the optimal trajectory and
; required velocity change.
$LAMBERT_AIMPOINT_GUIDANCE.agc			# pp. 651-653

$GROUND_TRACKING_DETERMINATION_PROGRAM.agc	# pp. 654-657
$P34-35_P74-75.agc				# pp. 658-702
$R31.agc					# pp. 703-708
$P76.agc					# pp. 709-711
$R30.agc					# pp. 712-722
$STABLE_ORBIT.agc				# pp. 723-730

; ============================================================================
; SECTION 7: LUNAR LANDING SEQUENCE - DESCENT AND ABORT PROGRAMS
; ============================================================================
;
; THIS IS THE HEART OF THE APOLLO 11 MISSION.
;
; These programs executed the powered descent from lunar orbit to the surface
; of the Moon on July 20, 1969. The landing sequence began at 102:33 mission
; elapsed time with Powered Descent Initiation (PDI) and concluded 12 minutes
; later with the historic words "The Eagle has landed."
;
; The Burn Baby Burn ignition routine fired the Descent Propulsion System (DPS),
; P63 braking phase program managed the high-altitude descent, guidance equations
; computed the fuel-optimal trajectory, and throttle control commanded engine
; power from 10% to full thrust and back down to 25% for final touchdown.
;
; During descent, these programs triggered the 1202 program alarms at 102:38:26
; when radar data temporarily overloaded the executive scheduler. Ground control,
; recognizing the alarms as non-critical, gave the "Go" decision to continue.
; At approximately 500 feet altitude, Neil Armstrong transitioned to semi-manual
; control to avoid a boulder field, extending the landing and reducing fuel
; margins to approximately 25 seconds remaining.
;
; This code represents one of humanity's greatest software engineering achievements.

$BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc	# pp. 731-751
$P40-P47.agc					# pp. 752-784
$THE_LUNAR_LANDING.agc				# pp. 785-792
$THROTTLE_CONTROL_ROUTINES.agc			# pp. 793-797
$LUNAR_LANDING_GUIDANCE_EQUATIONS.agc		# pp. 798-828

; Abort programs P70-P71 provide emergency ascent capability if landing must
; be aborted. These programs were never used on Apollo 11, but stood ready to
; save the crew if conditions became unsafe.
$P70-P71.agc					# pp. 829-837

; ============================================================================
; SECTION 8: LUNAR ASCENT AND RENDEZVOUS TARGETING
; ============================================================================
;
; After 21.5 hours on the lunar surface, Armstrong and Aldrin prepared Eagle
; for ascent. The P12 ascent program and ascent guidance routines executed the
; critical launch from the Moon on July 21, 1969, inserting the LM into an
; orbit that enabled rendezvous with Columbia.
;
; Unlike Earth launches with extended countdowns, lunar ascent happened in
; seconds: ascent engine ignition, vertical rise, pitchover, and insertion
; into a rendezvous trajectory - all managed autonomously by these programs.

$P12.agc					# pp. 838-842
$ASCENT_GUIDANCE.agc				# pp. 843-856

; ============================================================================
; SECTION 9: GUIDANCE/DAP INTERFACE AND CREW DISPLAYS
; ============================================================================
;
; Servicer routines, analog displays, and guidance/DAP interface programs
; coordinate between high-level mission programs and low-level control systems.
; Landing analog displays provided Armstrong and Aldrin with altitude rate
; and horizontal velocity information during the final approach.

$SERVICER.agc					# pp. 857-897
$LANDING_ANALOG_DISPLAYS.agc			# pp. 898-907
$FINDCDUW--GUIDAP_INTERFACE.agc			# pp. 908-925

; ============================================================================
; SECTION 10: IMU ALIGNMENT AND CELESTIAL EPHEMERIDES
; ============================================================================
;
; IMU alignment programs (P51-P53) enable the crew to align the inertial
; platform using star sightings through the AOT. Accurate platform alignment
; is essential for navigation - even small errors accumulate over time.
;
; Lunar and solar ephemerides provide Moon and Sun positions for navigation
; computations, trajectory planning, and optical alignment procedures.

$P51-P53.agc					# pp. 926-983
$LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc	# pp. 984-987

; ============================================================================
; SECTION 11: CORE OPERATING SYSTEM AND RUNTIME SUPPORT
; ============================================================================
;
; The AGC's real-time operating system is one of the earliest and most elegant
; examples of embedded spacecraft software. The Executive provides cooperative
; multitasking through a priority job queue, while the Waitlist implements
; preemptive timer-driven scheduling. Together, these routines managed dozens
; of concurrent tasks during mission operations.
;
; The Interpreter is a virtual machine that executes high-level vector and
; matrix operations, enabling complex navigation and guidance computations
; without consuming excessive memory. Approximately 40% of the AGC code uses
; interpretive language instructions rather than native AGC instructions.
;
; During the 1202 alarms, the Executive detected that its job queue was
; temporarily full due to radar data processing load. The restart protection
; routines preserved critical mission state, allowing the landing to continue.

$DOWN_TELEMETRY_PROGRAM.agc			# pp. 988-997

; Inter-bank communication handles subroutine calls across memory banks using
; the BANKCALL mechanism, essential for the AGC's bank-switched architecture.
$INTER-BANK_COMMUNICATION.agc			# pp. 998-1001

; The Interpreter virtual machine implements approximately 70 high-level
; instructions for vector arithmetic, matrix operations, and trigonometric
; functions, with double-precision arithmetic and automatic scaling.
$INTERPRETER.agc				# pp. 1002-1094

; Constant pools store mathematical constants (π, e, conversion factors) and
; physical constants (gravitational parameters, planetary radii) used
; throughout guidance and navigation computations.
$FIXED_FIXED_CONSTANT_POOL.agc			# pp. 1095-1099
$INTERPRETIVE_CONSTANT.agc			# pp. 1100-1101
$SINGLE_PRECISION_SUBROUTINES.agc		# p.  1102

; The Executive scheduler manages job priorities and task switching, implementing
; cooperative multitasking. The job queue holds up to 7 concurrent jobs.
$EXECUTIVE.agc					# pp. 1103-1116

; The Waitlist implements timer-driven preemptive scheduling using a delta-time
; queue. Tasks are scheduled with precise timing and executed by interrupts.
$WAITLIST.agc					# pp. 1117-1132

; ============================================================================
; SECTION 12: NAVIGATION STATE PROPAGATION AND INTEGRATION
; ============================================================================
;
; Navigation state propagation maintains the spacecraft's position and velocity
; estimate over time by integrating accelerometer measurements and accounting
; for gravitational forces. The orbital integration routines use Encke's method
; for numerical precision, propagating perturbations rather than absolute
; position to minimize accumulated errors.
;
; These computations ran continuously throughout the mission, providing the
; navigation state that guided every maneuver from Earth orbit through lunar
; landing and back to Earth.

$LATITUDE_LONGITUDE_SUBROUTINES.agc		# pp. 1133-1139
$PLANETARY_INERTIAL_ORIENTATION.agc		# pp. 1140-1148

; Measurement incorporation updates navigation state using sensor data from
; the IMU, optical sightings, and radar measurements, implementing Kalman
; filtering techniques to optimally blend measurements with predictions.
$MEASUREMENT_INCORPORATION.agc			# pp. 1149-1158

; Conic subroutines solve two-body orbital mechanics problems using Keplerian
; orbit theory, computing trajectories under gravitational influence.
$CONIC_SUBROUTINES.agc				# pp. 1159-1204

; Integration initialization and orbital integration propagate spacecraft
; trajectory through time, accounting for perturbations from non-spherical
; gravity, lunar gravity, solar gravity, and thrust forces.
$INTEGRATION_INITIALIZATION.agc			# pp. 1205-1226
$ORBITAL_INTEGRATION.agc			# pp. 1227-1248

; Inflight alignment, powered flight support, and time-of-free-fall computations
; provide specialized navigation functions for specific mission phases.
$INFLIGHT_ALIGNMENT_ROUTINES.agc		# pp. 1249-1258
$POWERED_FLIGHT_SUBROUTINES.agc			# pp. 1259-1267
$TIME_OF_FREE_FALL.agc				# pp. 1268-1283

; ============================================================================
; SECTION 13: SYSTEM DIAGNOSTICS, STATE MANAGEMENT, AND CREW INTERFACE
; ============================================================================
;
; Self-check diagnostics validate AGC hardware functionality, testing memory,
; instruction execution, and I/O channels. Phase table maintenance tracks
; mission program state for restart protection. The restart routine recovers
; from power transients or computational overloads by restoring task state.

$AGC_BLOCK_TWO_SELF_CHECK.agc			# pp. 1284-1293
$PHASE_TABLE_MAINTENANCE.agc			# pp. 1294-1302
$RESTARTS_ROUTINE.agc				# pp. 1303-1308

; IMU mode switching controls the inertial measurement unit's operational modes:
; coarse align, fine align, and gyrocompass mode for initial platform alignment.
$IMU_MODE_SWITCHING_ROUTINES.agc		# pp. 1309-1337

; Keyboard and uplink interrupt handlers process DSKY button presses and
; ground control commands received via the uplink communications channel.
$KEYRUPT_UPRUPT.agc				# pp. 1338-1340

; Display interface routines manage DSKY display formatting, verb/noun state
; machine processing, and crew input/output for all mission programs.
$DISPLAY_INTERFACE_ROUTINES.agc			# pp. 1341-1373

; Service routines provide utility functions, time conversions, and common
; mathematical operations used throughout the AGC software.
$SERVICE_ROUTINES.agc				# pp. 1374-1380

; Alarm and abort system displays program alarms (including the famous 1202),
; manages alarm history, and implements abort mode logic for mission safety.
$ALARM_AND_ABORT.agc				# pp. 1381-1385

; Update program processes uplink commands from Mission Control, including
; navigation state vector updates and mission parameter changes.
$UPDATE_PROGRAM.agc				# pp. 1386-1396

; RTB (Return To Bank) opcodes implement special interpreter instructions
; that call native AGC code from interpretive programs.
$RTB_OP_CODES.agc				# pp. 1397-1402

; ============================================================================
; SECTION 14: DIGITAL AUTOPILOT AND ATTITUDE CONTROL SYSTEMS
; ============================================================================
;
; The Digital Autopilot (DAP) maintains spacecraft attitude using the Reaction
; Control System (RCS) thrusters and engine gimbal control. The LM DAP manages
; three axes independently: P-axis (pitch), Q-axis (yaw), and R-axis (roll).
;
; During descent, the DAP coordinated with throttle control to maintain the
; commanded attitude while the descent engine provided variable thrust. During
; ascent and rendezvous, the DAP used RCS thrusters to maintain pointing for
; navigation sightings and orbital maneuvers.
;
; The TJET law implements minimum impulse bit firing logic to conserve RCS
; propellant while maintaining attitude deadbands. The Kalman filter estimates
; vehicle angular rates from gimbal angle measurements.

$T6-RUPT_PROGRAMS.agc				# pp. 1403-1405
$DAP_INTERFACE_SUBROUTINES.agc			# pp. 1406-1409
$DAPIDLER_PROGRAM.agc				# pp. 1410-1420

; P-axis (pitch) autopilot controls spacecraft pitch attitude using RCS
; thrusters or engine gimbal, critical during landing for velocity control.
$P-AXIS_RCS_AUTOPILOT.agc			# pp. 1421-1441

; Q-axis (yaw) and R-axis (roll) autopilots control spacecraft yaw and roll,
; maintaining three-axis stabilization throughout all mission phases.
$Q_R-AXIS_RCS_AUTOPILOT.agc			# pp. 1442-1459

; TJET law determines thruster firing patterns for minimum impulse control,
; implementing optimal fuel usage strategies for attitude maintenance.
$TJET_LAW.agc					# pp. 1460-1469
$KALMAN_FILTER.agc				# pp. 1470-1471

; Trim gimbal control adjusts engine gimbal angles to align thrust vector
; with the spacecraft center of mass, compensating for propellant usage.
$TRIM_GIMBAL_CNTROL_SYSTEM.agc			# pp. 1472-1484

; AOS (Acquisition Of Signal) task monitors communication with Earth and
; manages antenna pointing for optimal signal strength.
$AOSTASK_AND_AOSJOB.agc				# pp. 1485-1506

; SPS backup RCS control provides contingency attitude control if the Service
; Propulsion System is unavailable during critical maneuvers.
$SPS_BACK-UP_RCS_CONTROL.agc			# pp. 1507-1510

; ============================================================================
; END OF SOURCE FILE INCLUSIONS
; ============================================================================
;
; Pages 1511-1743 contain GAP-generated cross-reference tables, memory maps,
; and assembly listings produced by the assembler but not part of the source
; code. These tables were essential for debugging and verification at MIT
; Instrumentation Laboratory but are not included in the digital transcription.
;
; ASSEMBLY COMPLETE: Luminary099 (Revision 099)
; Target: Apollo 11 Lunar Module AGC
; Total Source Files: 90 (this file plus 89 included modules)
; Program Memory Usage: ~36K words of fixed memory (core rope)
; Variable Memory Usage: ~2K words of erasable memory
;
; This software successfully guided Eagle to humanity's first lunar landing
; on July 20, 1969, at Tranquility Base. The landing occurred at mission
; elapsed time 102:45:40, with Neil Armstrong reporting: "The Eagle has landed."
;
; ============================================================================
						# pp. 1511-1743: GAP-generated tables.
