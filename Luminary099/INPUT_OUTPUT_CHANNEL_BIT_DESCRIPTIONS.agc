# Copyright:	Public domain.
# Filename:	INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Pages:	0054-0060
# Mod history:	2009-05-14 OH	Transcribed from page images.
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

# ============================================================================
# FILE: INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc
# MODULE: Core Information and Hardware Interface
# MISSION PHASE: All phases (launch through landing through ascent)
#
# TL;DR: Complete reference documentation mapping AGC I/O channels (1-35 octal)
#        to Lunar Module hardware systems. Defines bit-level interface between
#        flight software and spacecraft hardware including: RCS thruster control,
#        descent/ascent engine commands, landing and rendezvous radar data,
#        IMU gimbal control, DSKY display outputs, and crew input channels.
#        Critical for understanding how AGC software directly controls physical
#        spacecraft systems during all mission phases.
#
# COMMENT-ONLY READERS: This reference explains the "nerve connections" between
#        the guidance computer and the lunar module hardware. Read to understand
#        how computer commands translate to thruster firings, engine throttle,
#        display updates, and how crew inputs reach the software.
# CODE-ALONG READERS: Essential hardware interface specification. Each channel
#        description shows exact bit assignments for I/O operations used throughout
#        the codebase. Reference this when analyzing how programs control spacecraft
#        systems via WRITE/READ channel instructions.
# ============================================================================

# Page 54

# ***	CHANNEL DESCRIPTIONSF WORDS ARE ALLOCATED IN ERASABLE ASSIGNMENTS ***

# ============================================================================
# SECTION: FUNDAMENTAL SYSTEM CHANNELS (1-4)
#
# The first four channels provide access to core AGC resources: processor
# registers and the mission elapsed time counter. These channels enable
# software to read system state and precise timing critical for navigation
# and guidance computations during all mission phases.
# ============================================================================

#	CHANNEL 1	IDENTICAL TO COMPUTER REGISTER L (0001)
#			The L register (lower accumulator) holds the least significant word
#			in double-precision arithmetic operations used extensively by the
#			interpreter for navigation and guidance vector/matrix computations.

#	CHANNEL 2	IDENTICAL TO COMPUTER REGISTER Q (0002)
#			The Q register stores subroutine return addresses. Access via this
#			channel enables interrupt handlers and restart protection logic to
#			preserve and restore program execution state.

#	CHANNEL 3	HISCALAR; INPUT CHANNEL; MOST SIGNIFICANT 14 BITS FROM 33 STAGE BINARY COUNTER. SCALE
#			FACTOR IS B23 IN CSEC, SO MAX VALUE ABOUT 23.3 HOURS AND LEAST SIGNIFICANT BIT 5.12 SECS.
#			Mission Elapsed Time (MET) high-order word. Combined with CHANNEL 4, provides the
#			precise timing used to synchronize all mission events from launch through splashdown.

#	CHANNEL 4	LOSCALAR; INPUT CHANNEL; NEXT MOST SIGNIFICANT 14 BITS FROM THE 33 STAGE BINARY COUNTER
#			ASSOCIATED WITH CHANNEL 3. SCALE FACTOR IS B9 IN  CSEC. SO MAX VAL IS 5.12 SEC AND LEAST
#			SIGNIFICANT BIT IS 1/3200 SEC. SCALE FACTOR OF D.P. WORD WITH CHANNEL 3 IS B23 CSEC.
#			Mission Elapsed Time (MET) low-order word. The 1/3200 second precision (0.3125 milliseconds)
#			enables accurate navigation state updates and guidance command timing during critical phases
#			like powered descent and ascent.

# ============================================================================
# SECTION: REACTION CONTROL SYSTEM (RCS) CHANNELS (5-6)
#
# These output channels directly fire the LM's 16 attitude control thrusters.
# The RCS jets control spacecraft orientation during all mission phases: orbital
# maneuvering, descent attitude control, landing site adjustments, and ascent.
# During the Apollo 11 landing, these jets maintained LM stability as Armstrong
# took semi-manual control to avoid the boulder field in the final approach.
# ============================================================================

#	CHANNEL 5	PYJETS; OUTPUT CHANNEL; PITCH RCS JET CONTROL.   (REACTION CONTROL SYSTEM) USES BITS 1-8.
#			Commands pitch axis thruster pairs. The Digital Autopilot (DAP) writes to this channel
#			to maintain pitch attitude or execute pitch maneuvers commanded by guidance programs
#			or crew manual control via the Rotational Hand Controller (RHC).

#	CHANNEL 6	ROLLJETS; OUTPUT CHANNEL; ROLL RCS JET CONTROL.   (REACTION CONTROL SYSTEM) USES BIT 1-8.
#			Commands roll axis thruster pairs. Critical during lunar descent for maintaining proper
#			landing radar antenna orientation and crew visibility of the landing site through the
#			LM windows.

#	CHANNEL 7	SUPERBNK; OUTPUT CHANNEL; NOT RESET BY RESTART;   FIXED EXTENSION BITS USED TO SELECT THE
#			APPROPRIATE FIXED MEMORY BANK IF FBANK IS 30 OCTAL OR MORE. USES BITS 5-7.
#			Memory bank extension register. The AGC's 36K words of fixed memory are organized in
#			banks; this channel selects higher-numbered banks for program execution, enabling access
#			to the complete mission software including landing and ascent guidance programs.

# ============================================================================
# SECTION: DISPLAY AND KEYBOARD (DSKY) INTERFACE CHANNELS (10-11)
#
# The DSKY is the crew's primary interface to the guidance computer. These channels
# control the display's numeric readouts, indicator lamps, and keyboard input.
# During descent, Buzz Aldrin monitored altitude and velocity on the DSKY while
# Armstrong focused on visual landing site selection. The DSKY also displayed the
# famous 1202 program alarm that occurred during Apollo 11's final approach.
# ============================================================================

#	CHANNEL 10	OUT0; OUTPUT CHANNEL; REGISTER USED TO TRANSMIT    LATCHING-RELAY DRIVING INFORMATION FOR
#			THE DISPLAY SYSTEM.  BITS 15-12 ARE SET TO THE ROW NUMBER (1-14 OCTAL) OF THE RELAY TO BE
#			CHANGED AND BITS 11-1 CONTAIN THE REQUIRED SETTINGS FOR THE RELAYS IN THE ROW.
#			DSKY display relay driver. The seven-segment displays showing VERB, NOUN, and numeric
#			data registers are controlled through this latching relay matrix. Programs write to
#			this channel to update the crew's visual display of navigation state, program status,
#			and alarm conditions.

#	CHANNEL 11	DSALMOUT; OUTPUT CHANNEL; REGISTER WHOSE BITS ARE USED FOR ENGINE ON-OFF CONTROL AND TO
#			DRIVE INDIVIDUAL INDICATORS OF THE DISPLAY SYSTEM. BITS 1-7 ARE A RELAYS.
#			CRITICAL CONTROL CHANNEL: Combines DSKY indicator lamp control with engine on/off commands.
#			Bits 13-14 control engine ignition and shutdown for both descent and ascent propulsion.
#			During Apollo 11's landing, the guidance computer used this channel to command descent
#			engine throttle changes and ultimately engine shutdown at "Contact Light" touchdown.
#
#	  BIT 1		ISS WARNING (Inertial Subsystem warning indicator)
#	  BIT 2		LIGHT COMPUTER ACTIVITY LAMP (illuminates during AGC computations)
#	  BIT 3		LIGHT UPLINK ACTIVITY LAMP (indicates ground commands being received)
#	  BIT 4		LIGHT TEMP CAUTION LAMP (temperature limit warning)
#	  BIT 5		LIGHT KEYBOARD RELEASE LAMP (enables crew DSKY input)
#	  BIT 6		FLASH VERB AND NOUN LAMPS (attention-getting flash for crew input requests)
#	  BIT 7		LIGHT OPERATOR ERROR LAMP (invalid crew input indicator)
# Page 55
#	  BIT 8		SPARE
#	  BIT 9		TEST CONNECTOR OUTBIT
#	  BIT 10	CAUTION RESET
#	  BIT 11	SPARE
#	  BIT 12	SPARE
#	  BIT 13	ENGINE ON (commands engine ignition - CRITICAL for powered descent and ascent)
#	  BIT 14	ENGINE OFF (commands engine shutdown - CRITICAL for landing touchdown and abort cutoff)
#	  BIT 15	SPARE

# ============================================================================
# SECTION: NAVIGATION AND SPACECRAFT HARDWARE CONTROL (12)
#
# Channel 12 controls critical navigation sensors and propulsion system hardware.
# IMU (Inertial Measurement Unit) control bits enable gyro alignment and error
# correction. Descent engine gimbal trim bits adjust thrust vector direction
# for attitude control during powered descent. Radar control bits manage both
# landing radar (for altitude/velocity data) and rendezvous radar (for CM tracking).
# ============================================================================

#	CHANNEL 12	CHAN12; OUTPUT CHANNEL; BITS USED TO DRIVE NAVIGATION AND SPAECRAFT HARDWARE
#
#	  BIT 1		ZERO RR CDU; CDU'S GIVE RRADAR INFORMATION FOR LM (Rendezvous Radar angle readout reset)
#	  BIT 2		ENABLE CDU RADAR ERROR COUNTERS (activates radar data error detection)
#	  BIT 3		NOT USED
#	  BIT 4		COARSE ALIGN ENABLE OF IMU (enables initial IMU platform alignment mode)
#	  BIT 5		ZERO IMU CDU'S (resets IMU gimbal angle counters for alignment procedures)
#	  BIT 6		ENABLE IMU ERROR COUNTER, CDU ERROR COUNTER. (activates navigation sensor error monitoring)
#	  BIT 7		SPARE
#	  BIT 8		DISPLAY INERTIAL DATA (selects IMU data for crew display on FDAI attitude indicator)
#	  BIT 9		-PITCH GIMBAL TRIM (BELL MOTION) DESCENT ENGINE (trims engine bell pitch angle negative)
#	  BIT 10	+PITCH GIMBAL TRIM (BELL MOTION) DESCENT ENGINE (trims engine bell pitch angle positive)
#	  BIT 11	-ROLL  GIMBAL TRIM (BELL MOTION) DESCENT ENGINE (trims engine bell roll angle negative)
#	  BIT 12	+ROLL  GIMBAL TRIM (BELL MOTION) DESCENT ENGINE (trims engine bell roll angle positive)
#	  BIT 13	LR POSITION 2 COMMAND (commands landing radar antenna to position 2 for descent phase)
#	  BIT 14	ENABLE RENDESVOUS RADAR LOCK-ON;AUTO ANGLE TRACK'G (enables automatic CM tracking after ascent)
#	  BIT 15	ISS TURN ON DELAY COMPLETE (Inertial Subsystem warm-up completed, ready for navigation)

# Page 56
# ============================================================================
# SECTION: RADAR CONTROL AND COMMUNICATION CHANNELS (13-14)
#
# Channel 13 controls radar parameter selection and ground communication links.
# The A,B,C matrix bits select which radar measurement (range, range-rate, or
# angle data) is transferred to the AGC. During Apollo 11's descent, landing
# radar data through these channels triggered the 1202 alarm when the computer
# became overloaded processing both landing radar and rendezvous radar inputs
# simultaneously. Flight controller Steve Bales made the critical "GO" decision
# to continue despite these alarms.
# ============================================================================

#	CHANNEL 13	CHAN13; OUTPUT CHANNEL.
#
#	  BIT 1		RADAR C		PROPER SETTING OF THE A,B,C MATRIX
#	  BIT 2		RADAR B		SELECTS CERTAIN RADAR
#	  BIT 3		RADAR A		PARAMETERS TO BE READ.
#				(ABC=001 selects range data, ABC=010 selects range-rate, ABC=100 selects angles)
#	  BIT 4		RADAR ACTIVITY (indicates radar is powered and transmitting)
#	  BIT 5		NOT USED (CONNECTS AN ALTERNATE INPUT TO UPLINK)
#	  BIT 6		BLOCK INPUTS TO UPLINK CELL (disables ground command reception for testing)
#	  BIT 7		DOWNLINK TELEMETRY WORD ORDER CODE BIT (controls telemetry frame format)
#	  BIT 8		RHC COUNTER ENABLE (READ HAND CONTROLLER ANGLES - enables reading rotation hand controller position)
#	  BIT 9		START RHC READ INTO COUNTERS IF BIT 8 SET (initiates hand controller angle sampling)
#	  BIT 10	TEST ALARMS, TEST DSKY LIGHTS (activates self-test mode for alarm and display systems)
#	  BIT 11	ENABLE STANDBY (enables AGC standby power mode for reduced power consumption)
#	  BIT 12	RESET TRAP 31-A		ALWAYS APPEAR TO BE SET TO 0 (hardware error trap reset)
#	  BIT 13	RESET TRAP 31-B		ALWAYS APPEAR TO BE SET TO 0 (hardware error trap reset)
#	  BIT 14	RESET TRAP 32		ALWAYS APPEAR TO BE SET TO 0 (hardware error trap reset)
#	  BIT 15	ENABLE T6 RUPT (enables Timer 6 interrupt for DAP and control system timing)

; ============================================================================
; CHANNEL 14: IMU Control and Crew Display Outputs
;
; Controls the Inertial Measurement Unit (IMU) gyro torquing and Coupling Data Unit
; (CDU) positioning. The IMU provides spacecraft attitude reference through three
; gyroscopes and three accelerometers mounted on a stabilized platform. CDUs measure
; gimbal angles. This channel enables the AGC to correct gyro drift and maintain
; precise navigation during all mission phases.
; ============================================================================

#	CHANNEL 14	CHAN14; OUTPUT CHANNEL; USED TO CONTROL COMPUTER  COUNTER CELLS (CDU,GYRO,SPACECRAFT FUNC.
#
#	  BIT 1		OUTLINK ACTIVITY (NOT USED) (reserved for future telemetry expansion)
#	  BIT 2		ALTITUDE RATE OR ALTITIDE SELECTOR (controls landing analog display mode for crew)
#	  BIT 3		ALTITUDE METER ACTIVITY (enables altitude display updates during descent)
#	  BIT 4		THRUST DRIVE ACTIVITY FOR DESCENT ENGINE (enables thrust magnitude display to crew)
#	  BIT 5		SPARE
#	  BIT 6		GYRO ENABLE POWER FOR PULSES (powers IMU gyro torquing motors)
#	  BIT 7		GYRO SELECT B		PAIR OF BITS IDENTIFIES AXIS OF -
#	  BIT 8		GYRO SELECT A		GYRO SYSTEM TO BE TORQUED (00=X, 01=Y, 10=Z axis selection)
#	  BIT 9		GYRO TORQUING COMMAND IN NEGATIVE DIRECTION (when set, applies negative gyro correction)
# Page 57
#	  BIT 10	GYRO ACTIVITY (signals gyro torquing pulse train active - used to correct drift)
#	  BIT 11	DRIVE CDU S (outer gimbal angle counter - CDU measures IMU platform orientation)
#	  BIT 12	DRIVE CDU T (inner gimbal angle counter)
#	  BIT 13	DRIVE CDU Z (middle gimbal angle counter)
#	  BIT 14	DRIVE CDU Y (Y-axis gimbal angle counter)
#	  BIT 15	DRIVE CDU X (X-axis gimbal angle counter)

; ============================================================================
; CHANNELS 15-16: Crew Input Interfaces
;
; These input channels receive signals from the crew through the DSKY keyboard
; and manual flight controls. Channel 15 captures keyboard entries triggering
; interrupt #5. Channel 16 monitors manual descent rate control and optical
; telescope mark buttons, triggering interrupt #6. During Apollo 11's final
; approach, Armstrong used the DESCENT+/- controls on Channel 16 to manually
; adjust the rate of descent when he took semi-manual control below 500 feet.
; ============================================================================

#	CHANNEL 15	MNKEYIN; INPUT CHANNEL;KEY CODE INPUT FROM KEYBOARD OF DSKY, SENSED BY PROGRAM WHEN
#			PROGRAM INTERRUPT #5 IS RECEIVED. USES BITS 5-1 (encoded key value in bits 1-5)

#	CHANNEL 16	NAVKEYIN; INPUT CHANNEL; OPTICS MARK INFORMATION AND NAVIGA ION PANEL DSKY (CM) OR THRUST
#			CONTROL (LM) SENSED BY PROGRAM WHEN PROGRAM INTER-RUPT #6 IS RECEIVED. USES BITS 3-7 ONLY.
#
#	  BIT 1		NOT ASSIGNED
#	  BIT 2		NOT ASSIGNED
#	  BIT 3		OPTICS X-AXIS MARK SIGNAL FOR ALIGN OPTICAL TSCOPE (AOT - Alignment Optical Telescope)
#	  BIT 4		OPTICS Y-AXIS MARK SIGNAL FOR AOT (crew marks star or landmark for navigation)
#	  BIT 5		OPTICS MARK REJECT SIGNAL (crew discards erroneous optical sighting)
#	  BIT 6		DESCENT+ ; CREW DESIRED SLOWING RATE OF DESCENT (manual descent rate decrease)
#	  BIT 7		DESCENT- ; CREW DESIRED SPEEDING UP RATE OF D'CENT (manual descent rate increase)

; ============================================================================
; CHANNELS 30-33: Spacecraft Status Monitoring (Inverted Logic)
;
; CRITICAL: All bits in these channels use inverted logic - a ZERO value means
; the signal IS PRESENT, a ONE means NOT PRESENT. This inverted sensing is a
; hardware design characteristic requiring software compensation throughout the AGC.
;
; These channels monitor critical spacecraft systems: abort switches, engine
; arming status, IMU health, guidance mode selection (AGC vs AGS backup computer),
; crew controllers (rotation and translation hand controllers), and engine
; gimbal positions. Continuous monitoring of these channels enables the AGC to
; respond to crew commands and detect system malfunctions.
; ============================================================================

# NOTE: ALL BITS IN CHANNELS 30-33 ARE INVERTED AS SENSED BY THE  PROGRAM, SO THAT A VALUE OF ZERO MEANS
# THAT THE INDICATED SIGNAL IS PRESENT.

#	CHANNEL 30	INPUT CHANNEL (Abort Controls, Engine Status, IMU Health)
#
#	  BIT 1		ABORT WITH DESCENT STAGE (crew abort button - jettisons ascent stage, fires APS)
#	  BIT 2		   UNUSED
#	  BIT 3		ENGINE ARMED SIGNAL (descent or ascent engine ready for ignition)
#	  BIT 4		ABORT WITH ASCENT ENGINE STAGE (alternate abort mode selection)
#	  BIT 5		AUTO THROTTLE; COMPUTER CONTROL OF DESCENT ENGINE (AGC has throttle authority)
# Page 58
#	  BIT 6		DISPLAY INERTIAL DATA (IMU data valid for crew displays)
#	  BIT 7		RR CDU FAIL (Rendezvous Radar Coupling Data Unit malfunction detected)
#	  BIT 8		SPARE
#	  BIT 9		IMU OPERATE WITH NO MALFUNCTION (IMU operating normally - critical for navigation)
#	  BIT 10	LM COMPUTER (NOT AGS) HAS CONTROL OF LM (AGC selected, not Abort Guidance System)
#	  BIT 11	IMU CAGE COMMAND TO DRIVE IMU GIMBAL ANGLES TO 0 (crew-initiated platform realignment)
#	  BIT 12	IMU CDU FAIL (MALFUNCTION OF IMU CDU,S - gimbal angle measurement failure)
#	  BIT 13	IMU FAIL (MALFUNCTION OF IMU STABILIZATION LOOPS - gyro or accelerometer failure)
#	  BIT 14	ISS TURN ON REQUESTED (Inertial Subsystem power-up command from crew)
#	  BIT 15	TEMPERATURE OF STABLE MEMBER WITHIN DESIGN LIMITS (IMU thermal control adequate)

; ============================================================================
; CHANNEL 31: Manual Flight Control Inputs (RHC and THC)
;
; Monitors the Rotation Hand Controller (RHC) and Translation Hand Controller (THC)
; used by the crew for manual spacecraft attitude and position control. The RHC
; commands rotations in pitch, yaw, and roll. The THC commands translations along
; X, Y, and Z axes. During Apollo 11's final landing approach, Armstrong used these
; controllers extensively below 500 feet altitude to maneuver the LM away from the
; boulder field to a safe landing site. The RCS Digital Autopilot (DAP) interprets
; these inputs to fire appropriate RCS thrusters.
;
; RHC inputs also control the Landing Point Designator (LPD), allowing the crew
; to adjust the targeted landing point by changing elevation and azimuth angles.
; ============================================================================

#	CHANNEL 31	INPUT CHANNEL; BITS ASSOCIATED WITH THE ATTITUDE  CONTROLLER, TRANSLATIONAL CONTROLLER,
#			AND SPACECRAFT ATTITUDE CONTROL; USED BY RCS DAP
#
#	  BIT 1		ROTATION (BY RHC) COMMANDED IN POSITIVE PITCH DIRECTION; MUST BE IN MINIMUM IMPULSE MODE.
#			ALSO POSITIVE ELEVATION CHANGE FOR LANDING POINT  DESIGNATOR (LPD reticle moves up)
#	  BIT 2		AS BIT 1 EXCEPT NEGATIVE PITCH AND ELEVATION (RHC forward - pitch down, LPD reticle down)
#	  BIT 3		ROTATION (BY RHC) COMMANDED IN POSITIVE YAW DIRECTION; MUST BE IN MINIMUM IMPULSE MODE.
#	  BIT 4		AS BIT 3 EXCEPT NEGATIVE YAW (RHC left/right commands yaw rotation)
#	  BIT 5		ROTATION (BY RHC) COMMANDED IN POSITIVE ROLL DIRECTION; MUST BE IN MINIMUM IMPULSE MODE.
#			ALSO POSITIVE AZIMUTH CHANGE FOR LANDING POINT DESIGNATOR (LPD reticle moves right)
#	  BIT 6		AS BIT 5 EXCEPT NEGATIVE ROLL AND AZIMUTH (RHC twist left - roll and LPD left)
#	  BIT 7		TRANSLATION IN +X DIRECTION COMMANDED BY THC (forward thrust - moves LM forward)
#	  BIT 8		TRANSLATION IN -X DIRECTION COMMANDED BY THC (aft thrust - moves LM backward)
#	  BIT 9		TRANSLATION IN +Y DIRECTION COMMANDED BY THC (right thrust - moves LM right)
#	  BIT 10	TRANSLATION IN -Y DIRECTION COMMANDED BY THC (left thrust - moves LM left)
#	  BIT 11	TRANSLATION IN +Z DIRECTION COMMANDED BY THC (up thrust - increases altitude)
#	  BIT 12	TRANSLATION IN -Z DIRECTION COMMANDED BY THC (down thrust - increases descent rate)
# Page 59
#	  BIT 13	ATTITUDE HOLD MODE ON SCS MODE CONTROL SWITCH (Stabilization Control System holds attitude)
#	  BIT 14	AUTO STABILIZATION OF ATTITUDE ON SCS MODE SWITCH (SCS provides automatic damping)
#	  BIT 15	ATTITUDE CONTROL OUT OF DETENT (RHC NOT IN NEUTRAL - crew actively commanding rotation)

; ============================================================================
; CHANNEL 32: Thruster Disable Switches and Engine Status
;
; Monitors crew-controlled thruster disable switches and descent engine gimbal
; status. If a thruster malfunctions (fails on, fails off, or leaks propellant),
; the crew can disable the affected thruster pair using panel switches. The RCS
; Digital Autopilot (DAP) then reconfigures jet selection logic to avoid using
; the disabled thrusters and maintain attitude control with remaining pairs.
;
; The LM has 16 RCS thrusters arranged in 4 clusters (quads) of 4 thrusters each.
; Thrusters are paired for redundancy. If one thruster in a pair fails, the crew
; can disable both to prevent unbalanced torques. The DAP software compensates
; by using alternate thruster combinations.
;
; Descent engine gimbal status indicates whether the crew has disabled gimbal
; control or if the system has detected a gimbal actuator failure. Without
; functional gimbals, thrust vector control is lost and the guidance system
; must rely on RCS thrusters alone for attitude control during powered descent.
; ============================================================================

#	CHANNEL 32	   INPUT CHANNEL.
#
#	  BIT 1		   THRUSTERS 2 & 4 DISABLED BY CREW (crew switch disables this thruster pair)
#	  BIT 2		   THRUSTERS 5 & 8 DISABLED BY CREW (crew switch disables this thruster pair)
#	  BIT 3		   THRUSTERS 1 & 3 DISABLED BY CREW (crew switch disables this thruster pair)
#	  BIT 4		   THRUSTERS 6 & 7 DISABLED BY CREW (crew switch disables this thruster pair)
#	  BIT 5		   THRUSTERS 14 & 16 DISABLED BY CREW (crew switch disables this thruster pair)
#	  BIT 6		   THRUSTERS 13 & 15 DISABLED BY CREW (crew switch disables this thruster pair)
#	  BIT 7		   THRUSTERS 9 & 12 DISABLED BY CREW (crew switch disables this thruster pair)
#	  BIT 8		   THRUSTERS 10 & 11 DISABLED BY CREW (crew switch disables this thruster pair)
#	  BIT 9		   DESCENT ENGINE GIMBALS DISABLED BY CREW (crew has disabled gimbal control)
#	  BIT 10	   APPARENT DESCENT ENGINE GIMBAL FAILURE (gimbal actuator malfunction detected)
#	  BIT 14	INDICATES PROCEED KEY IS DEPRESSED (crew pressed PRO key on DSKY)

; ============================================================================
; CHANNEL 33: Hardware Status and Communication Monitoring
;
; Provides real-time status of critical navigation sensors and communication
; systems. Bits 15-11 are flip-flop bits that latch error conditions until
; reset by software or by the T4RUPT timing loop. These bits capture transient
; failures that might otherwise be missed.
;
; Landing Radar (LR) and Rendezvous Radar (RR) status bits indicate data
; validity and operating modes. During Apollo 11's descent, the landing radar
; provided altitude and velocity data essential for the powered descent guidance.
; The guidance computer monitored these status bits to detect radar failures
; and switch to backup navigation modes if necessary.
;
; PIPA (Pulsed Integrating Pendulous Accelerometer) failure indication is
; critical because the IMU accelerometers provide the primary velocity and
; position updates. A PIPA failure would force an abort of the landing attempt.
;
; Uplink/downlink communication status allows the software to detect ground
; station communication problems. The "UPLINK TOO FAST" and "DOWNLINK TOO FAST"
; bits indicate timing violations in the data stream. The oscillator stop bit
; indicates catastrophic computer failure requiring immediate abort.
; ============================================================================

#	CHANNEL 33	CHAN33; INPUT CHANNEL; FOR HARDWARE STATUS AND COMMAND INFORMATION. BITS 15-11 ARE FLIP-
#			FLOP BITS RESET BY A CHANNEL "WRITE" COMMAND THAT ARE RESET BY A RESTART & BY T4RUPT LOOP.
#
#	  BIT 1		SPARE
#	  BIT 2		RR AUTO-POWER ON (Rendezvous Radar automatically powered and operational)
#	  BIT 3		RR RANGE LOW SCALE (Rendezvous Radar in close-range mode for final approach)
#	  BIT 4		RR DATA GOOD (Rendezvous Radar providing valid range and angle data)
#	  BIT 5		LR RANGE DATA GOOD (Landing Radar providing valid altitude measurement)
#	  BIT 6		LR POS1 (Landing Radar antenna in Position 1 - used during descent)
#	  BIT 7		LR POS2 (Landing Radar antenna in Position 2 - alternate position)
# Page 60
#	  BIT 8		LR VEL DATA GOOD (Landing Radar providing valid velocity measurement)
#	  BIT 9		LR RANGE LOW SCALE (Landing Radar in low-altitude scale for final approach)
#	  BIT 10	BLOCK UPLINK INPUT (ground uplink communication blocked or disabled)
#	  BIT 11	UPLINK TOO FAST (uplink data rate exceeds computer processing capability - FLIP-FLOP)
#	  BIT 12	DOWNLINK TOO FAST (downlink data rate timing violation - FLIP-FLOP)
#	  BIT 13	PIPA FAIL (IMU accelerometer failure detected - CRITICAL abort condition - FLIP-FLOP)
#	  BIT 14	WARNING OF REPEATED ALARMS: RESTART,COUNTER FAIL, VOLTAGE FAIL,AND SCALAR DOUBLE. (FLIP-FLOP)
#	  BIT 15	LGC OSCILLATOR STOPPED (computer clock failure - CATASTROPHIC - requires abort - FLIP-FLOP)

; ============================================================================
; CHANNELS 34-35: Downlink Telemetry Serialization
;
; These output channels serialize telemetry data for transmission to ground
; stations via the S-band communication system. The AGC prepares telemetry
; data in two 15-bit words that are serialized (converted from parallel to
; serial format) for radio transmission.
;
; During Apollo 11's mission, ground controllers in Houston continuously
; monitored telemetry data including computer state, navigation parameters,
; crew inputs, and spacecraft system status. This data stream enabled Mission
; Control to track the LM's descent trajectory, monitor fuel consumption,
; and provide real-time guidance to the crew.
;
; Channel 34 (DNT M1) carries the first word of the two-word telemetry frame.
; Channel 35 (DNT M2) carries the second word. Together they form a complete
; telemetry snapshot transmitted to Earth at regular intervals.
;
; The telemetry system was crucial during the 1202 program alarms - ground
; controllers saw the alarm codes via this downlink and were able to make
; the critical "Go" decision to continue the landing despite the executive
; overflow condition.
; ============================================================================

#	CHANNEL 34	DNT M1; OUTPUT CHANNEL; DOWNLINK 1  FIRST OF TWO WORDS SERIALIZATION.
#	CHANNEL 35	DNT M2; OUTPUT CHANNEL DOWNLINK 2 SOCOND OF TWO   WORDS SERIALIZATION.

