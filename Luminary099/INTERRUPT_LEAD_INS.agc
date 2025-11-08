# Copyright:	Public domain.
# Filename:	INTERRUT_LEAD_INS.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Pages:	0153-0154
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

; ============================================================================
; FILE: INTERRUPT_LEAD_INS.agc
; MODULE: Interrupt System Foundation
; MISSION PHASE: All phases (launch/earth-orbit/trans-lunar/lunar-orbit/descent/landing/ascent/rendezvous/trans-earth/re-entry)
;
; TL;DR: Defines the AGC's interrupt vector table at fixed memory location 4000,
;        establishing the real-time operating system foundation. Specifies interrupt
;        priorities (T4RUPT highest to UPRUPT lowest), entry point mechanics, and
;        context save requirements. Every interrupt that occurred during Apollo 11's
;        mission - from DSKY button presses to the critical radar updates during
;        lunar descent - entered through these vectors.
;
; COMMENT-ONLY READERS: This code establishes how the computer responds to external
;        events like crew button presses, radar updates, and timer ticks. Every 10
;        milliseconds during the lunar landing, T4RUPT interrupted whatever the
;        computer was doing to update critical timing and displays.
;
; CODE-ALONG READERS: Study the interrupt vector table structure at fixed location
;        4000, the DXCH ARUPT pattern for context save, bank switching via BBCON,
;        and the priority implicit in vector ordering. This is the AGC's hardware
;        interrupt entry mechanism.
; ============================================================================

# Page 153
; ============================================================================
; INTERRUPT VECTOR TABLE - FIXED MEMORY LOCATION 4000
;
; The AGC hardware automatically vectors to specific fixed memory addresses
; when interrupts occur. This table at location 4000 (octal) serves as the
; master interrupt dispatch mechanism. During the lunar landing descent,
; interrupts occurred every 10 milliseconds (T4RUPT) while the computer
; simultaneously processed radar data, guidance computations, and crew inputs.
; ============================================================================

		SETLOC	4000

; GO INTERRUPT - System Initialization and Fresh Start
; This is not a hardware interrupt but serves as the initial entry point
; when the AGC powers up or receives a fresh start command from the crew.
; During Apollo 11's mission, fresh starts were avoided during critical
; phases as they would reset the computer state.

		COUNT*	$$/RUPTS	# FIX-FIX LEAD INS
		INHINT			# GO
		CAF	GOBB
		XCH	BBANK
		TCF	GOPROG

; T6RUPT - Digital Autopilot Timer Interrupt (Typically 100ms intervals)
; Controls the LM's attitude through RCS thruster firings during coast phases
; and non-powered flight. During Apollo 11's descent, the autopilot continuously
; adjusted the spacecraft orientation to maintain the correct attitude for landing.
; DXCH ARUPT saves A and L registers to preserve interrupted program state.
; DCA and DTCB mechanism loads address from T6ADR and transfers control with
; both bank and address in a single atomic operation.

		DXCH	ARUPT		# T6RUPT
		EXTEND
		DCA	T6ADR
		DTCB

; T5RUPT - Autopilot Supplementary Interrupt
; Provides additional timing for digital autopilot computations.
; The autopilot calculations for RCS jet selection and attitude error
; correction run at regular intervals to maintain spacecraft pointing.

		DXCH	ARUPT		# T5RUPT - AUTOPILOT
		EXTEND
		DCA	T5ADR
		DTCB

; T3RUPT - Timer 3 Interrupt (Lower priority timing tasks)
; Handles periodic functions that don't require the urgency of T4RUPT.
; Uses bank switching pattern: load bank constant (CAF), exchange with BBANK
; register, then transfer control. This pattern appears throughout the
; interrupt vectors to reach handlers in different memory banks.

		DXCH	ARUPT		# T3RUPT
		CAF	T3RPTBB
		XCH	BBANK
		TCF	T3RUPT

; T4RUPT - HIGHEST PRIORITY TIMER INTERRUPT (10 millisecond intervals)
; The heartbeat of the AGC real-time system. Every 10ms, this interrupt fires
; to update the mission timer, service the WAITLIST (scheduled tasks), sample
; IMU data, and drive the DSKY displays. During Apollo 11's descent at mission
; time 102:38:26, radar data processing under T4RUPT triggered the famous 1202
; program alarm when the executive job queue became overloaded. Flight controller
; Steve Bales' decision to continue despite the alarm became a critical moment
; in the landing. T4RUPT's reliable 10ms cadence was essential - it continued
; executing flawlessly even during the alarm condition.

		DXCH	ARUPT		# T4RUPT
		CAF	T4RPTBB
		XCH	BBANK
		TCF	T4RUPT

; KEYRUPT1 - DSKY Keyboard Interrupt
; Fires when Armstrong or Aldrin press any button on the Display and Keyboard
; unit. During the descent, the crew used the DSKY to monitor Noun 63 (range to
; landing site, velocity), Verb 16 Noun 68 (altitude above landing site), and
; other critical navigation data. Each button press triggers this interrupt to
; capture and process the keystroke. The keyboard interrupt priority is lower
; than T4RUPT, ensuring timing-critical functions always complete first.

		DXCH	ARUPT		# KEYRUPT1
		CAF	KEYRPTBB
		XCH	BBANK
		TCF	KEYRUPT1

; KEYRUPT2 - Mark Button Interrupt (MARKRUPT)
; Triggered when the crew presses the MARK button on the DSKY or uses the
; Alignment Optical Telescope (AOT) mark button. Used for star sightings,
; landmark tracking, and manual navigation marks. During lunar orbit, the
; crew used marks to refine navigation state before beginning descent.

		DXCH	ARUPT		# KEYRUPT2
		CAF	MKRUPTBB
		XCH	BBANK
		TCF	MARKRUPT

; UPRUPT - Uplink Interrupt (Lowest hardware priority)
; Processes commands and data sent from Mission Control in Houston.
; During the landing, ground controllers could uplink navigation updates,
; target parameters, and commands. Steve Bales and the GUIDO team monitored
; telemetry and could have sent abort commands through this channel if needed.
; Lowest priority ensures onboard computations aren't disrupted by ground
; communications.

		DXCH	ARUPT		# UPRUPT
		CAF	UPRPTBB
		XCH	BBANK
		TCF	UPRUPT

; DOWNRUPT - Downlink Telemetry Interrupt
; Sends spacecraft state, navigation data, and system status to ground
; controllers. Mission Control watched this telemetry stream during descent,
; seeing the same data the crew saw plus additional engineering parameters.
; The ground team's ability to call "Go" for landing depended on this
; continuous data stream showing healthy systems.

		DXCH	ARUPT		# DOWNRUPT
		CAF	DWNRPTBB
		XCH	BBANK
		TCF	DODOWNTM

; RADAR RUPT - Landing and Rendezvous Radar Data Interrupt
; Processes altitude, altitude-rate, and velocity measurements from the
; landing radar during descent. During Apollo 11's approach, the landing radar
; continuously updated altitude and velocity data that fed into the guidance
; equations. The radar data processing load contributed to the 1202 program
; alarm when combined with other tasks. Despite the alarm, the radar data
; remained accurate and the landing continued successfully.

		DXCH	ARUPT		# RADAR RUPT
		CAF	RDRPTBB
# Page 154
		XCH	BBANK
		TCF	RADAREAD

; RUPT10 - Landing Guidance Specific Interrupt (PITFALL)
; Used exclusively during the powered descent phase for landing guidance
; computations. This interrupt services the descent guidance equations that
; computed the throttle commands and attitude steering during P63 (braking phase)
; and P64 (approach phase). During the final minutes before "The Eagle has
; landed," this interrupt executed the fuel-optimal guidance law that brought
; the LM safely to the surface with approximately 25 seconds of fuel remaining.

		DXCH	ARUPT		# RUPT10 IS USED ONLY BY LANDING GUIDANCE
		CA	RUPT10BB
		XCH	BBANK
		TCF	PITFALL


; ============================================================================
; INTERRUPT HANDLER BANK CONSTANTS AND ADDRESSES
;
; Each interrupt handler may reside in a different memory bank. The BBCON
; (Both Bank Constant) and 2CADR (Two Complemented Address) directives encode
; both the bank number and address within the bank, enabling the interrupt
; vectors above to switch banks and transfer control atomically.
;
; The EBANK= directive specifies which erasable (RAM) bank each handler uses,
; ensuring proper access to variables and data structures. The AGC's banking
; system allows 36K words of fixed memory and 2K words of erasable memory to
; be addressed despite a 12-bit address space.
; ============================================================================

		EBANK=	LST1		# RESTART USES E0, E3
GOBB		BBCON	GOPROG

		EBANK=	PERROR
T6ADR		2CADR	DOT6RUPT

		EBANK=	LST1
T3RPTBB		BBCON	T3RUPT

		EBANK=	KEYTEMP1
KEYRPTBB	BBCON	KEYRUPT1

; MARKRUPT bank constant - Uses EBANK containing AOTAZ (Alignment Optical
; Telescope azimuth angle). The mark interrupt handler accesses AOT data
; and navigation state variables in this erasable bank.

		EBANK=	AOTAZ
MKRUPTBB	BBCON	MARKRUPT

; UPRUPT shares the same bank as KEYRUPT1 - both keyboard/uplink functions
; use similar data structures in EBANK containing KEYTEMP1.

UPRPTBB		=	KEYRPTBB

; DOWNRUPT bank constant - Uses EBANK containing DNTMBUFF (downlink telemetry
; buffer). The telemetry system formats data for transmission to Mission Control.

		EBANK=	DNTMBUFF
DWNRPTBB	BBCON	DODOWNTM

; RADAR interrupt bank constant - Uses EBANK containing RADMODES (radar mode
; flags and control variables). The radar interrupt handler updates altitude,
; velocity, and range data used by guidance.

		EBANK=	RADMODES
RDRPTBB		BBCON	RADAREAD

; T4RUPT bank constant - Uses EBANK containing M11 (first element of 3x3
; transformation matrix). T4RUPT accesses extensive navigation and guidance
; data structures during its 10ms execution cycle.

		EBANK=	M11
T4RPTBB		BBCON	T4RUPT

; RUPT10 (PITFALL) bank constant - Uses EBANK containing ELVIRA (landing
; guidance variable). This interrupt is active only during powered descent,
; accessing the specific data structures needed for landing guidance equations.

		EBANK=	ELVIRA
RUPT10BB	BBCON	PITFALL

