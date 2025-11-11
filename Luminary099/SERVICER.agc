# Copyright:	Public domain.
# Filename:	SERVICER.agc
# Purpose:	Part of the source code for Luminary, build 099. It
#		is part of the source code for the Lunar Module's
#		(LM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 857-897
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	2009-06-01 FB	Transcription Batch 4 Assignment.
#		2009-06-05 RSB	Fixed a couple of typos, plus a goofy relative
#				label reference from the original source.
#
# The contents of the "Luminary099" files, in general, are transcribed
# from scanned documents.
#
#	Assemble revision 001 of AGC program Luminary099 by NASA
#	2021112-061.  July 14, 1969.
#
#	Prepared by
#			Massachusetts Institute of Technology
#			75 Cambridge Parkway
#			Cambridge, Massachusetts
#
#	under NASA contract NAS 9-4065.
#
# Refer directly to the online document mentioned above for further
# information.  Please report any errors to info@sandroid.org.

; ============================================================================
; FILE: SERVICER.agc
; MODULE: Mission Servicer and Navigation Support
; MISSION PHASE: all phases (launch/earth-orbit/trans-lunar/lunar-orbit/
;                descent/landing/ascent/rendezvous)
;
; TL;DR: Implements critical background tasks that run continuously throughout
;        all mission phases. Reads accelerometer (PIPA) data every 2 seconds,
;        monitors Delta-V changes, manages Landing Radar during descent,
;        calculates gravity effects, updates navigation state vectors, and
;        provides housekeeping functions. During Apollo 11's descent, this
;        code processed landing radar data that enabled Armstrong and Aldrin
;        to determine altitude and velocity for the historic lunar landing.
;
; COMMENT-ONLY READERS: This is the "heartbeat" of the navigation system,
;        running constantly in the background to keep the computer updated
;        on the spacecraft's motion. Read the Landing Radar sections (lines
;        900+) to follow the descent monitoring during the final approach.
; CODE-ALONG READERS: Study the PIPA reading timing synchronization with
;        autopilot (READACCS), the gravity calculation integration (CALCGRAV,
;        MUNRVG), and the sophisticated Landing Radar data validation logic.
; ============================================================================

# Page 857
		BANK	37
		SETLOC	SERV1
		BANK

		EBANK=	DVCNTR

; ============================================================================
; SECTION: PREREAD - Servicer Initialization and Gyro Bias Compensation
;
; This initialization sequence prepares the navigation system for continuous
; operation. The Inertial Measurement Unit (IMU) gyroscopes have small drift
; errors that must be compensated. This code performs the final gyro bias
; adjustment when the spacecraft is in free fall (not under thrust).
;
; During all mission phases, this ensures the platform remains accurately
; aligned to the reference coordinate system, critical for navigation.
; ============================================================================

# ************* PREREAD *******************

		COUNT*	$$/SERV

; PREREAD sets up the servicer background tasks and performs final IMU
; gyro drift compensation. Called during system initialization and after
; certain restarts to ensure navigation accuracy throughout the mission.

PREREAD		CAF	SEVEN		# 5.7 SPOT TO SKIP LASTBIAS AFTER
		TC	GNUFAZE5	# RESTART.
		CAF	PRIO21
		TC	NOVAC		; Schedule job without VAC area (uses EXECUTIVE)
		EBANK=	NBDX
		2CADR	LASTBIAS	# DO LAST GYRO COMPENSATION IN FREE FALL

; BIBIBIAS reads the PIPA accelerometers one final time during free fall
; coast phase to establish the baseline for subsequent motion detection.
; The PIPAs (Pulsed Integrating Pendulous Accelerometers) measure velocity
; changes - critical for dead reckoning navigation between radar updates.

BIBIBIAS	TC	PIPASR +3	# CLEAR + READ PIPS LAST TIME IN FRE5+F133
					# DO NOT DESTROY VALUE OF PIPTIME1

; Set navigation control flags for average-G processing and enable
; verb 37 (change major mode) functionality. These flags coordinate
; between navigation updates and crew display requests on the DSKY.

		CS	FLAGWRD7
		MASK	SUPER011	# SET V37FLAG AND AVEGFLAG (BITS 5 AND 6
		ADS	FLAGWRD7	# 	OF FLAGWRD7)

		CS	DRFTBIT
		MASK	FLAGWRD2	# RESET DRIFTFLAG
		TS	FLAGWRD2

; Initialize Delta-V monitor for tracking velocity changes.
; PIPAGE controls which of four PIPA data buffers is currently active.

		CAF	FOUR		# INITIALIZE DV MONITOR
		TS	PIPAGE

		CAF	ENDJBCAD	# POINT OUTROUTE TO END-OF-JOB.
		TS	OUTROUTE

; Schedule the state vector normalization task. NORMLIZE ensures position
; and velocity vectors maintain proper scaling and precision by periodically
; rectifying the coordinate system representation.

		CAF	PRIO22
		TC	FINDVAC		# TO FIRST ENTRY TO AVERAGE G
		EBANK=	DVCNTR
		2CADR	NORMLIZE

; Set up 2-second delay before starting accelerometer readings.
; This allows the system to stabilize after initialization.

		CA	TWO		# 5.2SPOT FOR REREADAC AND NORMLIZE
GOREADAX	TC	GNUTFAZ5
		CA	2SECS		# WAIT TWO SECONDS FOR READACCS
		TC	VARDELAY

# Page 858
; ============================================================================
; SECTION: READACCS - Accelerometer Reading and Autopilot Synchronization
;
; Every 2 seconds throughout the mission, this routine reads the three PIPA
; accelerometers that measure velocity changes along the X, Y, and Z axes.
; The timing is carefully synchronized with the digital autopilot to avoid
; interrupt conflicts that could cause data loss during telemetry downlink.
;
; This continuous monitoring provided the navigation data that, combined with
; radar during descent, allowed Armstrong and Aldrin to track Eagle's position
; and velocity during the approach to the lunar surface.
; ============================================================================

# ************* READACCS ****************

; READACCS synchronizes PIPA reading with the autopilot interrupt schedule.
; The 70-millisecond offset prevents simultaneous interrupts that could cause
; the computer to lose downlink telemetry ruptures (downrupts) to Earth.
; During the critical descent, maintaining this telemetry link was essential
; so Mission Control could monitor the LM's status.

READACCS	CS	OCT37771	# THIS PIECE OF CODING ATTEMPTS TO
		AD	TIME5		# SYNCHRONIZE READACCS WITH THE DIGITAL
		CCS	A		# AUTOPILOT SO THAT A PAXIS RUPT WILL
		CS	ONE		# OCCUR APPROXIMATELY 70 MILLISECONDS
		TCF	+2		# FOLLOWING THE READACCS RUPT.  THE 70 MS
		CA	ONE		# OFFSET WAS CHOSEN SO THAT THE PAXIS
 +2		ADS	TIME5		# RUPT WOULD NOT OCCUR SIMULTANEOUSLY
 					# WITH ANY OF THE 8 SUBSEQUENT R10,R11
					# INTERRUPTS -- THUS MINIMIZING THE POSS-
					# IBILITY OF LOSING DOWNRUPTS.

; Read all three PIPA accelerometers. Each PIPA accumulates velocity change
; pulses since the last reading. The values are scaled in units of centimeters
; per second, providing the dead-reckoning navigation between radar fixes.

		TC	PIPASR		# READ THE PIPAS.

; After PIPA reading completes, set up the main SERVICER job to process
; the accelerometer data, update gravity calculations, and manage navigation
; state. PIPAGE is set to 1, indicating the first of four rotating buffers.

PIPSDONE	CA	FIVE
		TC	GNUFAZE5
REDO5.5		CAF	ONE
		TS	PIPAGE		; Select PIPA buffer 1

; Schedule the SERVICER job at priority 20 to process sensor data and
; perform navigation updates. SERVICER runs continuously throughout the
; mission, executing every 2 seconds to maintain accurate position knowledge.

		CA	PRIO20
		TC	FINDVAC		; Request VAC area from EXECUTIVE
		EBANK=	DVCNTR
		2CADR	SERVICER	# SET UP SERVICER JOB

; Turn on test connector output bit for ground test equipment monitoring.
; This allows Mission Control to verify servicer is running during pre-flight
; checkout and provides a heartbeat signal during the mission.

		CA	BIT9
		EXTEND
		WOR	DSALMOUT	# TURN ON TEST CONNECTOR OUTBIT

; Check average-G flag. If clear, this is the final servicer pass and
; we should branch to exit sequence. The average-G calculation integrates
; accelerometer readings over time to determine net velocity changes.

		CA	FLAGWRD7
		MASK	AVEGFBIT
		EXTEND
		BZF	AVEGOUT		# AVEGFLAG DOWN -- SET UP FINAL EXIT

; Check MUNFLAG (Moon flag). When set, indicates we're in lunar operations
; and should enable Landing Radar processing and displays. If clear, we're
; in cislunar coast or Earth operations - bypass LR monitoring.

		CA	FLAGWRD6
		MASK	MUNFLBIT
		EXTEND
		BZF	MAKEACCS	# MUNFLAG CLEAR -- BYPASS LR AND DISP.

; Check if radar ranging programs (R10, R11) are already active.
; Avoid scheduling multiple instances which could conflict.

		CCS	PHASE2
		TCF	MAKEACCS	# PHASE 2 ACTIVATED -- AVOID MULTIPLE R10.

; Set up radar update rate at 4 times per second. During lunar descent,
; the Landing Radar provides altitude and velocity measurements that
; are critical for the powered descent guidance. The R10/R11 programs
; process this radar data and update the navigation state.

		CAF	SEVEN		# SET PIPCTR FOR 4X/SEC RATE.
		TS	PIPCTR

; Set time base 50 milliseconds in the past to synchronize radar updates
; with the servicer cycle. Precise timing ensures radar data processing
; doesn't interfere with autopilot or telemetry interrupts.

		CS	TIME1		# SET TBASE2 .05 SECONDS IN THE PAST.
		AD	FIVE
		AD	NEG1/2
		AD	NEG1/2
		XCH	TBASE2
# Page 859
; Set restart phase for radar ranging programs. Phase 2.21 allows proper
; recovery if the computer experiences a restart during radar processing.
; Restart protection was critical - during Apollo 11's descent, the 1202
; program alarms were caused by task overload, but restart protection
; allowed the guidance to continue safely.

		CAF	DEC17		# 2.21SPOT FOR R10,R11
		TS	L
		COM
		DXCH	-PHASE2

; Schedule the R10/R11 radar ranging programs to execute in 200 milliseconds.
; R10 processes Landing Radar altitude data, R11 processes velocity data.
; These programs were essential during Apollo 11's descent - the radar data
; combined with PIPA readings gave Armstrong and Aldrin their altitude and
; descent rate displayed on the DSKY and the landing point designator.

		CAF	OCT24		# FIRST R10,R11 IN .200 SECONDS
		TC	WAITLIST	; Schedule on timer-driven task list
		EBANK=	UNIT/R/
		2CADR	R10,R11

; MAKEACCS: Return to schedule another READACCS cycle in 2 seconds.
; This loop continues throughout the mission, providing the navigation
; heartbeat that keeps position and velocity knowledge current.

MAKEACCS	CA	FOUR
		TCF	GOREADAX	# DO PHASE CHANGE AND RECALL READACCS

; AVEGOUT: Final servicer exit when average-G flag indicates completion.
; Sets up the AVGEND routine to finish navigation integration and prepare
; for the next navigation phase (e.g., transition from descent to surface ops).

AVEGOUT		EXTEND
		DCA	AVOUTCAD	# SET UP FINAL SERVICER EXIT
		DXCH	AVGEXIT

		CA	FOUR		# SET 5.4 SPOT FOR REREADAC AND SERVICER
		TC	GNUTFAZ5	# IF REREADAC IS CALLED, IT WILL EXIT
		TC	TASKOVER	# END TASK WITHOUT CALLING READACCS

GNUTFAZ5	TS	L		# SAVE INPUT IN L
		CS	TIME1
		TS	TBASE5		# SET TBASE5
		TCF	+2

GNUFAZE5	TS	L		# SAVE INPUT IN L
		CS	L		# -PHASE IN A, PHASE IN L.
		DXCH	-PHASE5		# SET -PHASE5,PHASE5
		TC	Q

		EBANK=	DVCNTR
AVOUTCAD	2CADR	AVGEND

ENDJBCAD	CADR	SERVEXIT +2

OCT37771	OCT	37771

		BANK	33
		SETLOC	SERVICES
		BANK

		COUNT*	$$/SERV

# Page 860
; ============================================================================
; SECTION: SERVICER - Main Navigation Update and Data Processing
;
; This is the heart of the navigation system. SERVICER runs every 2 seconds
; throughout all mission phases to process accelerometer data, calculate
; gravity effects, update velocity and position, and prepare displays.
;
; During Apollo 11's descent on July 20, 1969, SERVICER integrated PIPA
; acceleration measurements with Landing Radar data to compute Eagle's
; altitude and descent rate - the numbers Armstrong and Aldrin watched on
; the DSKY during the final approach to Tranquility Base.
; ============================================================================

# ************* SERVICER ****************

; SERVICER main entry point. Set restart protection so if the computer
; experiences a restart (like the 1202 alarms during descent), navigation
; processing can resume properly without losing critical state information.

SERVICER	TC	PHASCHNG	# RESTART REREADAC + SERVICER
		OCT	16035
		OCT	20000
		EBANK=	DVCNTR
		2CADR	GETABVAL

		CAF	PRIO31		# INITIALIZE 1/PIPADT IN CASE RESTART HAS
		TS	1/PIPADT	# CAUSED LASTBIAS TO BE SKIPPED.

		TC	BANKCALL	# PIPA COMPENSATION CALL
		CADR	1/PIPA

GETABVAL	TC	INTPRET
		VLOAD	ABVAL
			DELV
		EXIT
		CA	MPAC
		TS	ABDELV		# ABDELV = CM/SEC*2(-14).
		EXTEND
		MP	KPIP
		DXCH	ABDVCONV	# ABDVCONV = M/CS * 2(-5).
		EXTEND
		DCA	MASS
		DXCH	MASS1		# INITIALIZE MASS1 IN CASE WE SKIP MASSMON
MASSMON		CS	FLAGWRD8	# ARE WE ON THE SURFACE?
		MASK	SURFFBIT
		EXTEND
		BZF	MOONSPOT	# YES:  BYPASS MASS MESS

		CA	FLGWRD10	# NO:  WHICH VEX SHOULD BE USED?
		MASK	APSFLBIT
		CCS	A
		EXTEND			# IF EXTEND IS EXECUTED, APSVEX --> A,
		DCA	APSVEX		# 	OTHERWISE DPSVEX --> A
		TS	Q

		EXTEND
		DCA	ABDVCONV
		EXTEND
OCT10002	DV	Q		# WHERE APPROPRIATE VEX RESIDES
		EXTEND
		MP	MASS
		DAS	MASS1

MOONSPOT	CA	KPIP1		# TP MPAC = ABDELV AT 2(14) CM/SEC
		TC	SHORTMP		# MULTIPLY BY KPIP1 TO GET
# Page 861
		DXCH	MPAC		# ABDELV AT 2(7) M/CS
		DAS	DVTOTAL		# UPDATE DVTOTAL FOR DISPLAY

		TC	TMPTOSPT

		TC	BANKCALL
		CADR	QUICTRIG

		CAF	XNBPIPAD
		TC	BANKCALL
		CADR	FLESHPOT
		TC	INTPRET
AVERAGEG	BON	CALL
			MUNFLAG
			RVBOTH
			CALCRVG
		EXIT
GOSERV		TC	QUIKFAZ5

COPYCYCL	TC	COPYCYC

#		CA	ZERO		# A IS ZERO ON RETURN FROM COPYCYC
		TS	PIPATMPX
		TS	PIPATMPY
		TS	PIPATMPZ

; ============================================================================
; TRANSITION: From PIPA Processing to Delta-V Monitoring
;
; The servicer now enters the critical DVMON (Delta-V Monitor) routine that
; detects engine thrust by monitoring accumulated velocity changes. During
; powered flight phases like lunar descent or ascent, this routine determines
; when thrust has exceeded the detection threshold and switches the Digital
; Autopilot (DAP) from RCS jets to gimbaled engine control for attitude.
; This automatic mode switching was essential during Apollo 11's descent when
; the Descent Propulsion System (DPS) ignited for powered descent initiation.
; ============================================================================

		CS	STEERBIT	# CLEAR STEERSW PRIOR TO DVMON.
		MASK	FLAGWRD2
		TS	FLAGWRD2

; Delta-V monitoring logic begins by checking system state flags to determine
; if thrust monitoring is appropriate. The IDLEFLAG indicates if the vehicle
; is in a quiescent state where no thrust should be expected.

		CAF	IDLEFBIT	# IS THE IDLE FLAG SET?
		MASK	FLAGWRD7
		CCS	A
		TCF	NODVMON1	# IDLEFLAG = 1, HENCE SET AUXFLAG TO 0.

; The AUXFLAG is used for state management during thrust detection cycling.
; When transitioning between thrust and no-thrust states, this auxiliary flag
; helps prevent rapid oscillation between control modes.

		CS	FLAGWRD6
		MASK	AUXFLBIT
		CCS	A
		TCF	NODVMON2	# AUXFLAG = 0, HENCE SET AUXFLAG TO 1.

; ============================================================================
; DVMON - Delta-V Threshold Monitoring
;
; This is the core thrust detection logic. The accumulated delta-V magnitude
; (ABDELV) is compared against the thrust detection threshold (DVTHRUSH).
; When ABDELV exceeds DVTHRUSH, the system recognizes that significant thrust
; is being applied and adjusts control mode accordingly. During Apollo 11's
; lunar descent, this routine detected DPS ignition and enabled gimbal steering.
; ============================================================================

DVMON		CS	DVTHRUSH
		AD	ABDELV		# Compare accumulated ΔV against threshold
		EXTEND			# (ABDELV - DVTHRUSH, scaled in ft/sec)
		BZMF	LOTHRUST	# Branch if below threshold (low thrust)

; High thrust detected! The delta-V exceeds the threshold, indicating engine
; firing. Set the STEERSW flag to enable steering control through gimbals.

		CS	FLAGWRD2	# SET STEERSW.
		MASK	STEERBIT
		ADS	FLAGWRD2

; Thrust has been confirmed. Initialize the DVCNTR counter to allow up to two
; more servicer passes before requiring re-detection. This prevents spurious
; mode changes due to momentary thrust dropouts during engine operation.

DVCNTSET	CAF	ONE		# ALLOW TWO PASSES MAXIMUM NOW THAT
# Page 862
		TS	DVCNTR		# THRUST HAS BEEN DETECTED.

; ============================================================================
; Engine Control Mode Selection: Gimbals vs. RCS Jets
;
; With thrust detected, the system must now decide which control effectors to
; use. The LM has two attitude control options during powered flight:
; 1. Gimbaled engine (DPS or APS) - preferred for fuel efficiency
; 2. RCS jets - used if gimbal system unavailable or for APS backup
;
; This decision is critical during descent. During Apollo 11's landing, the
; gimbaled DPS provided primary attitude control while the RCS provided fine
; adjustments and backup capability.
; ============================================================================

		CA	FLGWRD10	# BRANCH IF APSFLAG IS SET.
		MASK	APSFLBIT	# Check if Ascent Propulsion System is active
		CCS	A
		TCF	USEJETS		# APS has no gimbal, must use RCS jets

; For DPS (Descent Propulsion System), check gimbal system health before
; committing to gimbal steering. The gimbal fail bit in channel 32 indicates
; if the engine gimbal actuators are functioning properly.

		CA	BIT9		# CHECK GIMBAL FAIL BIT
		EXTEND
		RAND	CHAN32		# Read hardware status from I/O channel 32
		EXTEND
		BZF	USEJETS		# Gimbal failed, use RCS jets instead

; Gimbal system is healthy and available. Configure DAP to use gimbaled thrust
; vector control (GTS) for attitude control. Clear the USEQRJTS flag to disable
; RCS jet usage for yaw/roll axes (pitch will use gimbals).

USEGTS		CS	USEQRJTS
		MASK	DAPBOOLS	# Clear jet control flag in DAP boolean word
		TS	DAPBOOLS
		TCF	SERVOUT

; ============================================================================
; NODVMON Paths - Bypass Delta-V Monitoring
;
; These paths are taken when delta-V monitoring should not be active, either
; because the vehicle is in idle mode (IDLEFLAG set) or due to auxiliary flag
; state management. The system reverts to RCS jet control and manages the
; AUXFLAG state to prepare for the next monitoring cycle.
; ============================================================================

NODVMON1	CS	AUXFLBIT	# SET AUXFLAG TO 0.
		MASK	FLAGWRD6	# Clear auxiliary flag (idle state confirmed)
		TS	FLAGWRD6
		TCF	USEJETS		# Use RCS jets for attitude control
NODVMON2	CS	FLAGWRD6	# SET AUXFLAG TO 1.
		MASK	AUXFLBIT	# Set auxiliary flag (prepare for next cycle)
		ADS	FLAGWRD6
		TCF	USEJETS		# Use RCS jets for attitude control

; ============================================================================
; LOTHRUST - Low Thrust Detection Path
;
; When delta-V falls below the detection threshold, the system enters this
; path to manage the transition out of powered flight mode. A counter-based
; hysteresis prevents rapid mode switching due to transient thrust variations.
; If thrust remains low, the system eventually schedules a COMFAIL (Command
; Fail) job to handle the transition to coast phase attitude control.
; ============================================================================

LOTHRUST	TC	QUIKFAZ5	# Quick phase change for state management
		CCS	DVCNTR		# Check pass counter
		TCF	DECCNTR		# Counter > 0, decrement and continue

; Counter has reached zero with sustained low thrust. Check if a COMFAIL job
; is already active before scheduling another. PHASE4 tracks active jobs.

		CCS	PHASE4		# COMFAIL JOB ACTIVE?
		TCF	SERVOUT		# YES:  WON'T NEED ANOTHER.

; No COMFAIL job is running. Schedule one now to handle the transition from
; powered flight to coast phase. This job will reconfigure the DAP and other
; systems for the new flight regime.

		TC	PHASCHNG	# 4.37SPOT FOR COMFAIL.
		OCT	00374		# Phase change code for COMFAIL scheduling

		CAF	PRIO25
		TC	NOVAC
		EBANK=	WHICH
		2CADR	COMFAIL

		TCF	SERVOUT

; Decrement the counter to allow one more pass before declaring sustained low
; thrust. This provides hysteresis to prevent mode oscillation.

DECCNTR		TS	DVCNTR1		# Temporarily save decremented counter
		TC	QUIKFAZ5
		CA	DVCNTR1
		TS	DVCNTR		# Update actual counter
		INHINT
		TC	IBNKCALL	# IF THRUST IS LOW, NO STEERING IS DONE
# Page 863
		CADR	STOPRATE	# AND THE DESIRED RATES ARE SET TO ZERO.

; ============================================================================
; USEJETS - Configure DAP for RCS Jet Attitude Control
;
; This path is taken when gimbal steering is not available or not appropriate.
; The DAP is configured to use RCS jets on all axes (pitch, yaw, and roll) for
; attitude control. This mode is used during coast phases, during APS burns
; (which cannot gimbal), or when DPS gimbal system has failed.
; ============================================================================

USEJETS		CS	DAPBOOLS
		MASK	USEQRJTS	# Set flag to enable yaw/roll jets
		ADS	DAPBOOLS

; All control mode paths (gimbal steering or RCS jets) converge here to exit
; the servicer routine and invoke the selected acceleration integration routine.

SERVOUT		RELINT			# Re-enable interrupts
		TC	BANKCALL
		CADR	1/ACCS		# Scale accelerations by 1/ACCS factor

; Save context for eventual return to servicer. This state preservation allows
; the servicer to resume after the acceleration processing completes.

		CA	PRIORITY
		MASK	LOW9		# Extract priority bits
		TS	PUSHLOC		# Save priority for restoration
		ZL
		DXCH	FIXLOC		# Save FIXLOC and DVFIND for continuation

		TC	QUIKFAZ5
		EXTEND			# EXIT TO SELECTED ROUTINE WHETHER THERE
		DCA	AVGEXIT		# IS THRUST OR NOT.  THE STATE OF STEERSW
		DXCH	Z		# WILL CONVEY THIS INFORMATION.

XNBPIPAD	ECADR	XNBPIP

		BANK	32
		SETLOC	SERV2
		BANK
		COUNT*	$$/SERV

; ============================================================================
; AVGEND - Average-G Termination and Free-Fall Gyro Configuration
;
; This exit point is used when average-G processing completes, typically when
; transitioning from powered flight (descent or ascent) to coast/free-fall
; phases. The IMU gyro compensation is reconfigured for drift compensation
; appropriate to the zero-g environment.
; ============================================================================

AVGEND		CA	PIPTIME +1	# FINAL AVERAGE G EXIT
		TS	1/PIPADT	# SET UP FREE FALL GYRO COMPENSATION.

; Enable drift compensation mode. During powered flight, gyro compensation
; handles acceleration effects. In free fall, only drift compensation is needed.

		TC	UPFLAG		# SET DRIFT FLAG.
		ADRES	DRIFTFLG

		TC	BANKCALL
		CADR	PIPFREE		# Configure PIPAs for free-fall mode

		CS	BIT9
		EXTEND
		WAND	DSALMOUT	# Clear alarm bit

; Phase change management: deactivate servicer group (5) and activate the
; appropriate coast phase group (2) for continued system monitoring.

		TC	2PHSCHNG
		OCT	5		# GROUP 5 OFF (servicer terminating)
		OCT	05022		# GROUP 2 ON (coast phase monitoring)
		OCT	20000

; Configure display and navigation flags for coast phase. R29 (range/range-rate
; display) and R10 (DSKY displays) are shut off. MUNFLAG (Moon reference frame)
; is cleared as the routine completes.

		TC	INTPRET
		SET	CLEAR
			NOR29FLG	# SHUT OFF R29 WHEN SERVICER ENDS.
			SWANDISP	# SHUT OFF R10 WHEN SERVICER ENDS.
		CLEAR	CALL		# RESET MUNFLAG.
			MUNFLAG
# Page 864
			AVETOMID
		CLEAR	EXIT
			V37FLAG

; Final average-G processing complete. Return control to the routine that
; originally invoked the servicer (address stored in OUTROUTE).

AVERTRN		CA	OUTROUTE	# RETURN TO DESIRED POINT.
		TC	BANKJUMP

OUTGOAVE	=	AVERTRN		# Alternate entry label for same function
DVCNTR1		=	MASS1		# Share storage location with MASS1

# Page 865
		SETLOC	SERV3
		BANK
		COUNT*	$$/SERV

; ============================================================================
; SERVIDLE - Servicer Disconnect and Idle Mode
;
; Called when servicer must be disconnected from active guidance operations,
; typically during program termination, aborts, or transitions to idle mode.
; Systematically shuts down servicer integration points, clears phase tables,
; and prepares for restart or program change. Critical for clean abort handling.
; ============================================================================

SERVIDLE	EXTEND			# DISCONNECT SERVICER FROM ALL GUIDANCE
		DCA	SVEXTADR	# Restore original exit address
		DXCH	AVGEXIT		# Disconnect from average-G processing

; Disable the delta-V monitor. In idle mode, there is no thrust to monitor and
; navigation updates are suspended.

		CS	FLAGWRD7	# DISCONNECT THE DELTA-V MONITOR
		MASK	IDLEFBIT	# Set idle flag
		ADS	FLAGWRD7

		CAF	LRBYBIT		# TERMINATE R12 IS RUNNING.
		TS	FLGWRD11	# Disable Landing Radar processing

; Clear restart protection phase tables. Each phase group must be individually
; deactivated. Phase 2 is preserved if MUNFLAG is set (Moon-referenced navigation
; still active).

		EXTEND
		DCA	NEG0
		DXCH	-PHASE1		# Deactivate phase group 1

		CA	FLAGWRD6	# DO NOT TURN OFF PHASE 2 IF MUNFLAG SET.
		MASK	MUNFLBIT	# Check Moon reference flag
		CCS	A
		TCF	+4		# Skip phase 2 deactivation if Moon-referenced

		EXTEND
		DCA	NEG0
		DXCH	-PHASE2		# Deactivate phase group 2

 +4		EXTEND
 		DCA	NEG0
		DXCH	-PHASE3		# Deactivate phase group 3

		EXTEND
		DCA	NEG0
		DXCH	-PHASE6		# Deactivate phase group 6

		CAF	OCT33		# 4.33SPOT FOR GOP00FIX
		TS	L
		COM
		DXCH	-PHASE4		# Special handling for phase group 4

; Complete servicer disconnection by triggering restart logic. This ensures
; clean transition to GOTOPOOH (idle program) while servicer gracefully exits.

		TCF	WHIMPER		# PERFORM A SOFTWARE RESTART AND PROCEED
					# TO GOTOPOOH WHILE SERVICER CONTINUES TO
					# RUN, ALBEIT IN A GROUND STATE WHERE
					# ONLY STATE-VECTOR DEPENDENT FUNCTIONS
					# ARE MAINTAINED.

		EBANK=	DVCNTR
# Page 866
SVEXTADR	2CADR	SERVEXIT

		BANK	32
		SETLOC	SERV
		BANK
		COUNT*	$$/SERV

SERVEXIT	TC	PHASCHNG
		OCT	00035

+2		TCF	ENDOFJOB

		BANK	23
		SETLOC	NORMLIZ
		BANK

		COUNT*	$$/SERV

# Page 867
; ============================================================================
; NORMLIZE - Navigation State Normalization and Update
;
; This is the heart of the SERVICER navigation update cycle. NORMLIZE takes
; the integration results from RN1 and VN1 (position and velocity vectors in
; the reference frame) and processes them into the current navigation state
; vectors R and V used by guidance routines throughout the mission.
;
; During lunar operations, this routine also computes the gravity acceleration
; vector and maintains the hyperbolic unit vector UHYP (pointing from the CSM
; to the LM) used for rendezvous navigation after ascent from the lunar surface.
;
; This routine runs every 2 seconds throughout powered flight and critical
; mission phases, maintaining the spacecraft's knowledge of where it is and
; where it's going—essential for every maneuver from descent initiation through
; final rendezvous with Columbia in lunar orbit.
; ============================================================================

NORMLIZE	TC	INTPRET		# Enter interpretive mode for vector operations
		VLOAD	BOFF		# Load position vector
			RN1		# RN1 = integrated position vector
			MUNFLAG		# Check if on lunar surface (MUNFLAG set)
			NORMLIZ1	# If on Moon, use different gravity calc

		; Lunar operations path: Transform position to stable member coordinates.
		; The VSL6 (vector shift left 6) converts from meters*2^29 scaling to
		; meters*2^23 scaling required by REFSMMAT transformation.

		VSL6	MXV		# Scale and multiply by matrix
			REFSMMAT	# Transform to stable member coordinates
		STCALL	R		# Store as current position R
			MUNGRAV		# Compute lunar gravity (MUNicipal GRAVity)

		; Now process velocity vector through same coordinate transformation.
		; During descent, this velocity drives throttle and attitude commands.

		VLOAD	VSL1		# Load velocity vector (shift left 1)
			VN1		# VN1 = integrated velocity vector
		MXV			# Transform to stable member frame
			REFSMMAT
		STOVL	V		# Store as current velocity V, load CSM velocity
			V(CSM)		# Velocity of Command Module (for rendezvous)

		; Compute hyperbolic unit vector UHYP: direction from CSM to LM.
		; This is critical after ascent begins—used to target the rendezvous.
		; The cross product of velocity vectors gives a reference direction,
		; normalized to unit length for navigation calculations.

		VXV	UNIT		# Cross product with CSM position, normalize
			R(CSM)		# Position of Command Module
		STORE	UHYP		# Unit vector toward CSM (rendezvous direction)

ASCSPOT		EXIT		# Return to native AGC code

		; Clear GROUP 2 phase register to ensure clean job termination.
		; This prevents phase table conflicts during restart scenarios.

		EXTEND			# MAKE SURE GROUP 2 IS OFF
		DCA	NEG0		# Load double-precision negative zero
		DXCH	-PHASE2		# Clear phase 2 register

		TC	POSTJUMP	# Jump to different bank
		CADR	NORMLIZ2	# Continue at NORMLIZ2

		BANK	33
		SETLOC	SERVICES
		BANK
		COUNT*	$$/SERV

; Alternate entry for lunar surface gravity calculation:
NORMLIZ1	CALL
			CALCGRAV	# Standard spherical gravity model
		EXIT

; Final step: copy navigation state to working registers:
NORMLIZ2	CA	EIGHTEEN	# Copy 18 words (9 double-precision values)
		TC	COPYCYC +1	# DO NOT COPY MASS IN NORMLIZE
		TC	ENDOFJOB	# Job complete, return to EXECUTIVE

COPYCYC		CA	OCT24		# DEC 20
 +1		INHINT
 +2		MASK	NEG1		# REDUCE BY 1 IF ODD
 		TS	ITEMP1
		EXTEND
		INDEX	ITEMP1
		DCA	RN1
		INDEX	ITEMP1
# Page 868
		DXCH	RN
		CCS	ITEMP1
		TCF	COPYCYC +2
		TC	Q		# RETURN UNDER INHINT

EIGHTEEN	DEC	18

# Page 869
# ************* PIPA READER *****************
# MOD NO. 00 BY D. LICKLY, DEC. 9 1966
#
# FUNCTIONAL DESCRIPTION
#	SUBROUTINE TO READ PIPA COUNTERS, TRYING TO BE VERY CAREFUL SO THAT WILL BE RESTARTABLE.
#	PIPA READINGS ARE STORED IN THE VECTOR DELV.  THE HIGH ORDER PART OF EACH COMPONENT CONTAINS THE PIPA READING,
# 	RESTARTS BEGIN AT REREADAC.
#
#	AT THE END OF THE PIPA READER THE CDUS ARE READ AND STORED AS A
#	VECTOR IN CDUTEMP.  THE HIGH ORDER PART OF EACH COMPONENT CONTAINS
# 	THE CDU READING IN 25 COMP IN THE ORDER CDUX,Y,Z.  THE THRUST
#	VECTOR ESTIMATOR IN FINDCDUD REQUIRES THE CDUS BE READ AT PIPTIME.
#
# CALLING SEQUENCE AND EXIT
#	CALL VIA TC, ISWCALL, ETC.
#	EXIT IS VIA Q.
#
# INPUT
#	INPUT IS THROUGH THE COUNTERS PIPAX, PIPAY, PIPAZ, AND TIME2.
#
# OUTPUT
#	HIGH ORDER COMPONENTS OF THE VECTOR DELV CONTAIN THE PIPA READINGS.
#	PIPTIME CONTAINS TIME OF PIPA READING.
#
# DEBRIS (ERASABLE LOCATIONS DESTROYED BY PROGRAM)
#	TEMX, TEMY, TEMZ, PIPAGE

		BANK	37
		SETLOC	SERV1
		BANK

		COUNT*	$$/SERV

PIPASR		EXTEND
# Page 870
		DCA	TIME2
		DXCH	PIPTIME1	# CURRENT TIME POSITIVE VALUE
 +3		CS	ZERO		# INITIALIZE THESE AT NEG. ZERO.
 		TS	TEMX
		TS	TEMY
		TS	TEMZ

		CA	ZERO
		TS	DELVZ
		TS	DELVZ +1
		TS	DELVY
		TS	DELVY +1
		TS	DELVX +1
		TS	PIPAGE		# SHOW PIPA READING IN PROGRESS

REPIP1		EXTEND
		DCS	PIPAX		# X AND Y PIPS READ
		DXCH	TEMX
		DXCH	PIPAX		# PIPAS SET TO NEG ZERO AS READ.
		TS	DELVX
		LXCH	DELVY

REPIP3		CS	PIPAZ		# REPEAT PROCESS FOR Z PIP
		XCH	TEMZ
		XCH	PIPAZ
DODELVZ		TS	DELVZ

REPIP4		EXTEND			# COMPUTE GUIDANCE PERIOD
		DCA	PIPTIME1
		DXCH	PGUIDE
		EXTEND
		DCS	PIPTIME
		DAS	PGUIDE

		CA	CDUX		# READ CDUS INTO HIGH ORDER CDUTEMPS
		TS	CDUTEMPX
		CA	CDUY
		TS	CDUTEMPY
		CA	CDUZ
		TS	CDUTEMPZ
		CA	DELVX
		TS	PIPATMPX
		CA	DELVY
		TS	PIPATMPY
		CA	DELVZ
		TS	PIPATMPZ

		TC	Q

# Page 871
REREADAC	CCS	PIPAGE
		TCF	READACCS	# PIP READING NOT STARTED.  GO TO BEGINNING

		CAF	DONEADR		# SET UP RETURN FROM PIPASR
		TS	Q

		CCS	DELVZ
		TCF	REPIP4		# Z DONE, GO DO CDUS
		TCF	+3		# Z NOT DONE, CHECK Y.
		TCF	REPIP4
		TCF	REPIP4

		ZL
		CCS	DELVY
		TCF	+3
		TCF	CHKTEMX		# Y NOT DONE, CHECK X.
		TCF	+1
		LXCH	PIPAZ		# Y DONE, ZERO Z PIP.

		CCS	TEMZ
		CS	TEMZ		# TEMZ NOT = -0, CONTAINS -PIPAZ VALUE.
		TCF	DODELVZ
		TCF	-2
		LXCH	DELVZ		# TEMZ = -0, L HAS ZPIP VALUE.
		TCF	REPIP4

CHKTEMX		CCS	TEMX		# HAS THIS CHANGED
		CS	TEMX		# YES
		TCF	+3		# YES
		TCF	-2		# YES
		TCF	REPIP1		# NO
		TS	DELVX

		CS	TEMY
		TS	DELVY

		CS	ZERO		# ZERO X AND Y PIPS
		DXCH	PIPAX		# L STILL ZERO FROM ABOVE

		TCF	REPIP3

DONEADR		GENADR	PIPSDONE

# Page 872
		BANK	33
		SETLOC	SERVICES
		BANK

		COUNT*	$$/SERV

TMPTOSPT	CA	CDUTEMPY	# THIS SUBROUTINE, CALLED BY AN RTB FROM
		TS	CDUSPOTY	# INTERPRETIVE, LOADS THE CDUS CORRESPON-
		CA	CDUTEMPZ	# DING TO PIPTIME INTO THE CDUSPOT VECTOR.
		TS	CDUSPOTZ
		CA	CDUTEMPX
		TS	CDUSPOTX
		TC 	Q

# LRHTASK IS A WAITLIST TASK SET BY READACCS DURING THE DESCENT BRAKING
# PHASE WHEN THE ALT TO THE LUNAR SURFACE IS LESS THAN 25,000 FT.  THIS
# TASK CLEARS THE ALTITUDE MEASUREMENT MADE DISCRETE AND INITIATES THE
# LANDING RADAR MEASUREMENT JOB (LRHJOB) TO TAKE A ALTITUDE MEASUREMENT
# 50 MS PRIOR TO THE NEXT READACCS TASK.

		BANK	21
		SETLOC	R10
		BANK

		COUNT*	$$/SERV

LRHTASK		CS	FLGWRD11
		MASK	LRBYBIT
		EXTEND
		BZF	GRP2OFF		# LR BYPASS SET -- BYPASS ALL LR READING.

		CA	READLBIT
		MASK	FLGWRD11	# IS READLR FLAG SET?
		EXTEND
		BZF	GRP2OFF		# NO.  BYPASS LR READ.

		CS	FLGWRD11
		MASK	NOLRRBIT	# IS LR READ INHIBITED?
		EXTEND
		BZF	GRP2OFF		# YES.  BYPASS LR READ.

		CA	PRIO32		# LR READ OK.  SET JOB TO DO IT
		TC	NOVAC		# ABOUT 50 MS. PRIOR TO PIPA READ.
		EBANK=	HMEAS
		2CADR	LRHJOB

GRP2OFF		EXTEND
		DCA	NEG0
		DXCH	-PHASE2
		TCF	R10,R11A

		BANK	33
		SETLOC	SERVICES
		BANK
# Page 873
		COUNT*	$$/SERV

# HIGATASK IS ENTERED APPROXIMATELY 6 SECS PRIOR TO HIGATE DURING THE
# DESCENT PHASE.  HIGATASK SETS THE HIGATE FLAG (BIT11) AND THE LR INHIBIT
# FLAG (BIT10) IN LRSTAT.  THE HIGATJOB IS SET UP TO REPOSITION THE LR
# ANTENNA FROM POSITION 1 TO POSITION 2.  IF THE REPOSITIONING IS
# SUCCESSFUL THE ALT BEAM AND VELOCITY BEAMS ARE TRANSFORMED TO THE NEW
# ORIENTATION IN NB COORDINATES AND STORED IN ERASABLE.

HIGATASK	INHINT
		CS	PRIO3		# SET HIGATE AND LR INHIBIT FLAGS
		MASK	FLGWRD11
		AD	PRIO3
		TS	FLGWRD11
		CAF	PRIO32
		TC	FINDVAC		# SET LR POSITIONING JOB (POS2)
		EBANK=	HMEAS
		2CADR	HIGATJOB

		TCF	CONTSERV	# CONTINUE SERVICER

# Page 874
# MUNRETRN IS THE RETURN LOC FROM SPECIAL AVE G ROUTINE (MUNRVG)

MUNRETRN	EXIT

		CS	FLGWRD11
		MASK	LRBYBIT
		EXTEND
		BZF	COPYCYC1	# BYPASS LR LOGIC IF BIT15 IS SET.

		CA	READLBIT	# SEE IF ALT < 35000 FT LAST CYCLE
		MASK	FLGWRD11
		EXTEND
		BZF	35KCHK		# ALT WAS > 35000 FT LAST CYCLE   CHK NOW

		CAF	XORFLBIT	# WERE WE BELOW 30000 FT LAST PASS?
		MASK	FLGWRD11
		EXTEND
		BZF	XORCHK		# NO -- TEST THIS PASS
HITEST		CAF	PSTHIBIT	# CHECK FOR HIGATE
		MASK	FLGWRD11
		EXTEND
		BZF	HIGATCHK	# NOT AT HIGATE LAST CYCLE -- CHK THIS CYCLE

POS2CHK		CAF	BIT7		# VERIFY LR IN POS2
		EXTEND
		RAND	CHAN33
		EXTEND
		BZF	UPDATCHK	# IT IS -- CHECK FOR LR UPDATE
		CAF	BIT13
		EXTEND
		RAND	CHAN12
		EXTEND
		BZF	LRPOSALM	# LR NOT IN POS2 OR REPOSITIONING -- BAD
		TCF	CONTSERV	# LR BEING REPOSITIONED -- CONTINUE SERV

HIGATCHK	CA	TTF/8		# IS TTF > CRITERION?  (TTF IS NEGATIVE)
		AD	RPCRTIME
		EXTEND
		BZMF	POS1CHK		# NO

		CA	EBANK4		# MUST SWITCH EBANKS
		XCH	EBANK
		TS	L		# SAVE IN L

		EBANK=	XNBPIP
		CS	XNBPIP		# UXBXP IN GSOP CH5
		EBANK=	DVCNTR
		LXCH	EBANK		# RESTORE EBANK
		AD	RPCRTQSW	# QSW - UXBXP
# Page 875

		EXTEND
		BZMF	HIGATASK	# IF UXBXP > QSW, THEN REPOSITION

POS1CHK		CAF	BIT6		# HIGATE NOT IN SIGHT -- DO POS1 CHK
		EXTEND
		RAND	33
		EXTEND
		BZF	UPDATCHK	# LR IN POS1 -- CHECK FOR LR UPDATE

LRPOSALM	TC	ALARM		# LR NOT IN PROPER POS-ALARM-BYPASS UPDATE
		OCT	511		# AND CONTINUE SERVICER
CONTSERV	INHINT
		CS	BITS4-7
		MASK	FLGWRD11	# CLEAR LR MEASUREMENT MADE DISCRETES.
		TS	FLGWRD11

		TC	IBNKCALL	# SET LR LITES PROPERLY
		CADR	R12LITES

# Page 876
COPYCYC1	TC	QUIKFAZ5

R29?		CA	FLAGWRD3
		MASK	NR29&RDR
		CCS	A		# IS NOR29FLG OR READRFLG SET?
		TCF	R29NODES	# YES, SO DON'T DESIGNATE.

		CA	RADMODES	# NO, SO R29 IS CALLED FOR.
		MASK	OCT10002	# IS THE RR NOT ZEROING ITS CDUS, AND
		CCS	A		# IS THE RENDEZVOUS RADAR IN AUTO MODE?
		TCF	R29NODES	# NO, SO DON'T DESIGNATE.

		CA	RADMODES
		MASK	PRIO22
		CCS	A		# IS RR REPOSITIONING OR REMODING?
		TCF	NOR29NOW	# YES:  COME BACK IN 2 SECONDS & TRY AGAIN.

		TCF	R29

R29NODES	INHINT			# R29 NOT ALLOWED THIS CYCLE.
		CS	DESIGBIT	# SHOW THAT DESIGNATION IS OFF.
		MASK	RADMODES
		TS	RADMODES

NOR29NOW 	TC	INTPRET		# INTPRET DOES A RELINT.
		VLOAD	ABVAL		# MPAC = ABVAL( NEW SM. POSITION VECTOR )
			R1S
		PUSH	DSU		# 				(2)
			/LAND/
		STORE	HCALC		# NEW HCALC*2(24)M.
		STORE	HCALC1
		DMPR	RTB
			ALTCONV
			SGNAGREE
		STOVL	ALTBITS		# ALTITUDE FOR R10 IN BIT UNITS.
			UNIT/R/
		VXV	UNIT
			UHYP
		STOVL	UHZP		# DOWNRANGE HALF-UNIT VECTOR FOR R10.
			R1S
		VXM	VSR4
			REFSMMAT
		STOVL	RN1		# TEMP. REF. POSITION VECTOR*2(29)M.
			V1S
		VXM	VSL1
			REFSMMAT
		STOVL	VN1		# TEMP. REF. VELOCITY VECTOR 2(7) M/CS.
			UNIT/R/
		VXV	ABVAL
# Page 877
			V1S
		SL1	DSQ
		DDV
		DMPR	RTB
			ARCONV1
			SGNAGREE
COPYCYC2	EXIT			# LEAVE ALTITUDE RATE COMPENSATION IN MPAC
		INHINT
		CA	UNIT/R/		# UPDATE RUNIT FOR R10.
		TS	RUNIT
		CA	UNIT/R/ +2
		TS	RUNIT +1
		CA	UNIT/R/ +4
		TS	RUNIT +2
		CA	MPAC		# LOAD NEW DALTRATE FOR R10.
		TS	DALTRATE

		EXTEND
		DCA	R1S
		DXCH	R
		EXTEND
		DCA	R1S +2
		DXCH	R +2
		EXTEND
		DCA	R1S +4
		DXCH	R +4
		EXTEND
		DCA	V1S
		DXCH	V
		EXTEND
		DCA	V1S +2
		DXCH	V +2
		EXTEND
		DCA	V1S +4
		DXCH	V +4

		TCF	COPYCYCL	# COMPLETE THE COPYCYCL.

# Page 878
# ALTCHK COMPARES CURRENT ALTITUDE (IN HCALC) WITH A SPECIFIED ALTITUDE FROM A TABLE BEGINNING AT ALTCRIT.
# ITS CALLING SEQUENCE IS AS FOLLOWS:-
#
#	L	CAF	N
#	L+1	TC	BANKCALL
#	L+2	CADR	ALTCHK
#	L+3	RETURN HERE IF HCALC STILL > SPECIFIED CRITERION.   C(L) = +0.
#	L+4	RETURN HERE IF HCALC < OR = SPECIFIED CRITERION.   C(A) = C(L) = +0
#
# ALTCHK MUST BE BANKCALLED EVEN FROM ITS OWN BANK.   N IS THE LOCATION, RELATIVE TO THE TAG ALTCRIT,
# OF THE BEGINNING OF THE DP CONSTANT TO BE USED AS A CRITERION.

ALTCHK		EXTEND
		INDEX	A
		DCA	ALTCRIT
		DXCH	MPAC +1
		EXTEND
		DCS	HCALC
		DAS	MPAC +1
		TC	BRANCH +4
		CAF	ZERO		# BETTER THAN A NOOP, PERHAPS
		INCR	BUF2
		TCF	SWRETURN

ALTCRIT		=	25KFT

25KFT		2DEC	7620 B-24  		# (0)

50KFT		2DEC	15240 B-24		# (2)

50FT		2DEC	15.24 B-24		# (4)

30KFT		2DEC	9144 B-24		# (6)

2KFT/SEC	DEC	6.096 B-7		# 2000 FT/SEC AT 2(7) M/CS


# (A remark was likely to be needed here to explain XORCHK) 4/Jun/09,FB

XORCHK		CAF	SIX		# ARE WE BELOW 30000 FT?
		TC	BANKCALL
		CADR	ALTCHK
		TCF	HITEST		# CONTINUE LR UPDATE
		TC	UPFLAG		# YES: INHIBIT X-AXIS OVERRIDE
		ADRES	XOVINFLG
		TC	UPFLAG
		ADRES	XORFLG
		TCF	HITEST		# CONTINUE LR UPDATE

35KCHK		CAF	TWO		# ARE WE BELOW 35000 FT?

# Page 879
		TC	BANKCALL
		CADR	ALTCHK
		TCF	CONTSERV
		TC	UPFLAG
		ADRES	READLR		# SET READLR FLAG TO ENABLE LR READING.
		TCF	CONTSERV

# Page 880
# ***************************************************************

; ============================================================================
; CALCGRAV - General Gravity Field Calculation with Oblateness
;
; This routine calculates the gravitational acceleration vector including
; non-spherical perturbations (Earth or lunar oblateness - the J2 term).
; Used throughout most mission phases when precise gravity modeling is needed.
;
; The routine determines whether to use Earth or lunar gravity parameters based
; on RTX2 flag, then computes the full gravity vector including the oblateness
; correction that accounts for the celestial body's equatorial bulge.
;
; Input: UNIT/R/ = unit position vector, 34D = R·R (position magnitude squared)
; Output: GDT1/2 = gravity acceleration * delta-time / 2, scaled at 2(+7) m/cs
;         UNITGOBL = unit gravity vector including oblateness perturbation
;
; This calculation ran continuously during trans-lunar coast and lunar orbit,
; ensuring navigation accuracy for trajectory corrections and orbit maintenance.
; ============================================================================

CALCGRAV	UNIT	PUSH		# SAVE UNIT/R/ IN PUSHLIST	(18)
		STORE 	UNIT/R/		# Unit position vector for direction

		; Determine whether we're in Earth or lunar gravity field.
		; RTX2 = 0 for Earth orbit, = 2 for lunar operations.
		; This selects the appropriate gravitational parameter (μ) and
		; oblateness coefficient (J2) from indexed constant tables.

		LXC,1	SLOAD		# RTX2 = 0 IF EARTH ORBIT, =2 IF LUNAR.
			RTX2		# Load index for gravity parameter selection
			RTX2
		DCOMP	BMN		# If negative (Earth), skip oblateness calc
			CALCGRV1	# Go to simple spherical gravity

		; Compute oblateness perturbation (J2 term). The Earth and Moon are
		; oblate spheroids (equatorial bulge), which creates an additional
		; gravitational component proportional to (RE/RN)² and dependent on
		; latitude. This term becomes significant in low orbits.
		;
		; The calculation: J2 correction = J2 * (RE/RN)² * f(latitude)
		; where f(latitude) involves cos²(latitude) - 1/20 term.

		VLOAD	DOT		#				(12)
			UNITZ		# Z-axis unit vector (toward pole)
			UNIT/R/		# Dot product = cos(co-latitude)
		SL1	PUSH		# Scale and save		(14)
		DSQ	BDSU		# Square and subtract constant
			DP1/20		# (cos²(θ) - 0.05) for J2 formula
		PDDL	DDV		# Push, load radius of body
			RESQ		# RE² or RM² (body radius squared)
			34D		# (RN)SQ - position magnitude squared
		STORE	32D		# TEMP FOR (RE/RN)SQ

		; First J2 term: radial component correction
		DMP	DMP		# Multiply by J2 coefficient
			20J		# 20*J2 constant
		VXSC	PDDL		# Scale unit vector by result, push, load
			UNIT/R/		# Radial correction component

		; Second J2 term: polar component correction
		DMP	DMP		# Multiply by latitude function
			2J		# 2*J2 constant
			32D		# (RE/RN)² ratio
		VXSC	VSL1		# Scale Z-axis unit vector
			UNITZ		# Polar correction component

		; Combine corrections into unit gravity vector
		VAD	STADR		# Add components and set address
		STORE	UNITGOBL	# Unit gravity with oblateness (global)
		VAD	PUSH		# MPAC = UNIT GRAVITY VECTOR.	(18)

		; Now scale by gravitational parameter μ and distance.
		; Gravity = -μ * UNIT/R / R² scaled appropriately for integration.

CALCGRV1	DLOAD	NORM		# PERFORM A NORMALIZATION ON RMAGSQ IN
			34D		# ORDER TO BE ABLE TO SCALE THE MU FOR
			X2		# MAXIMUM PRECISION.
		BDDV*	SLR*		# Indexed divide by μ*ΔT
			-MUDT,1		# Gravitational parameter * time step
			0 -21D,2	# Shift for scaling
		VXSC	STADR		# Scale unit vector by magnitude
		STORE	GDT1/2		# SCALED AT 2(+7) M/CS
		RVQ			# Return to caller

CALCRVG		VLOAD	VXM
			DELV
			REFSMMAT
		VXSC	VSL1
			KPIP1
		STORE	DELVREF
		VSR1	PUSH
		VAD	PUSH		# (DV-OLDGDT)/2 TO PD SCALED AT 2(+7) M/CS.
# Page 881
			GDT/2
		VAD	PDDL
			VN
			PGUIDE
		SL	VXSC
			6D
		VAD	STQ
			RN
			31D
		STCALL	RN1		# TEMP STORAGE OF RN SCALED 2(+29) M
			CALCGRAV

		VAD	VAD
		VAD
			VN
		STCALL	VN1		# TEMP STORAGE OF VN SCALED 2(+7) M/CS
			31D

DP1/20		2DEC	0.05
SHIFT11		2DEC	1 B-11

# Page 882
#*****************************************************************************
# MUNRVG IS A SPECIAL AVERAGE G INTEGRATION ROUTINE USED BY THRUSTING
# PROGRAMS WHICH FUNCTION IN THE VICINITY OF AN ASSUMED SPHERICAL MOON.
# THE INPUT AND OUTPUT QUANTITIES ARE REFERENCED TO THE STABLE MEMBER
# COORDINATE SYSTEM.

RVBOTH		VLOAD	PUSH
			G(CSM)
		VAD	PDDL
			V(CSM)
			PGUIDE
		DDV	VXSC
			SHIFT11
		VAD
			R(CSM)
		STCALL	R1S
			MUNGRAV
		VAD	VAD
			V(CSM)
		STADR
		STORE	V1S
		EXIT
		TC	QUIKFAZ5
		TC	INTPRET
		VLOAD
			GDT1/2
		STOVL	G(CSM)
			R1S
		STOVL	R(CSM)
			V1S
		STORE	V(CSM)
		EXIT
		TC	QUIKFAZ5
		TC	INTPRET
MUNRVG		VLOAD	VXSC
			DELV
			KPIP2
		PUSH	VAD		# 1ST PUSH:  DELV IN UNITS OF 2(8) M/CS
			GDT/2
		PUSH	VAD		# 2ND PUSH:  (DELV + GDT)/2, UNITS OF 2(7)
			V		#				(12)
		PDDL	DDV
			PGUIDE
			SHIFT11
		VXSC
		VAD
			R
		STCALL	R1S		# STORE R SCALED AT 2(+24) M
			MUNGRAV
# Page 883
		VAD	VAD
		VAD
			V
		STORE	V1S		# STORE V SCALED AT 2(+7) M/CS.
		ABVAL
		STOVL	ABVEL		# STORE SPEED FOR LR AND DISPLAYS.
			UNIT/R/
		DOT	SL1
			V1S
		STOVL	HDOTDISP	# HDOT = V. UNIT(R)*2(7) M/CS.
			R1S
		VXV	VSL2
			WM
		STODL	DELVS		# LUNAR ROTATION CORRECTION TERM*2(5) M/CS.
			36D
		DSU
			/LAND/
		STCALL	HCALC		# FOR NOW, DISPLAY WHETHER POS OR NEG
			MUNRETRN

; ============================================================================
; MUNGRAV - Simplified Lunar Surface Gravity Calculation
;
; "MUNicipal GRAVity" - a simplified spherical gravity model used during lunar
; surface operations (descent, landing, ascent). This routine assumes the Moon
; is a perfect sphere and computes only the radial gravity component without
; oblateness corrections, which is adequate for the short duration and low
; altitude of LM operations near the surface.
;
; This is much faster than CALCGRAV and ran during the critical 12-minute
; powered descent on July 20, 1969, providing the gravity acceleration needed
; to compute trajectory corrections from 50,000 feet altitude to touchdown.
;
; Input: UNIT/R/ = unit position vector toward Moon center
;        34D = R·R (position magnitude squared from lunar center)
; Output: GDT1/2 = gravity acceleration * delta-time / 2, scaled at 2(+7) m/cs
;
; The simplified calculation: g = -μ/R² in the radial direction, where μ is
; the lunar gravitational parameter (mass × G constant).
; ============================================================================

MUNGRAV		UNIT			# AT 36D HAVE ABVAL(R), AT 34D R.R
		STODL	UNIT/R/		# Store unit radial vector (direction)
			34D		# Load R² (position magnitude squared)

		; Compute gravity magnitude: g = μ/R²
		; The lunar gravitational parameter μ = 4.9028 × 10¹² m³/s²
		; scaled and pre-multiplied by the integration time step.

		SL	BDDV		# Scale R² and divide by μ*ΔT
			6D		# Shift left 6 for proper scaling
			-MUDTMUN	# Lunar gravitational parameter * time

		; Scale result and apply to unit vector for final gravity vector.
		; The SHIFT11 constant adjusts the scaling to match the integration
		; routine's expected input format (2^7 m/cs for half delta-T).

		DMP	VXSC		# Multiply by scaling constant
			SHIFT11		# Scaling factor (1 B-11 = 2^-11)
			UNIT/R/		# Apply magnitude to direction
		STORE	GDT1/2		# 1/2GDT SCALED AT 2(7) M/CS.
		RVQ			# Return to caller (NORMLIZE routine)

1.95SECS	DEC	195
7.5		2DEC	.02286 B-6	# 7.5 FT/SEC AT 2(6) M/CS

2SEC(18)	2DEC	200 B-18

2SEC(28)	2OCT	0000000310	# 2SEC AT 2(28)

4SEC(28)	2DEC	400 B-28

BITS4-7		OCT	110

; ============================================================================
; TRANSITION: From Gravity Computation to Landing Radar Data Processing
;
; The servicer has completed the gravity calculation needed for trajectory
; integration. Now attention shifts to processing measurements from the landing
; radar - the critical sensor that tells the guidance computer the LM's altitude
; above the lunar surface and velocity relative to the ground below.
;
; During Apollo 11's descent on July 20, 1969, the landing radar provided the
; altitude and velocity data that enabled Armstrong to monitor the descent rate
; and ultimately select the final landing site manually. Without this radar,
; there would be no safe way to land on the Moon.
; ============================================================================

# Page 884
; ============================================================================
; LANDING RADAR ALTITUDE UPDATE CHECK (UPDATCHK/POSUPDAT)
; ============================================================================
;
; The landing radar has acquired the lunar surface and is returning altitude
; measurements. This section validates those measurements and incorporates them
; into the navigation state vector, keeping the guidance computer's knowledge
; of position accurate as the LM descends.
;
; COMMENT-ONLY READERS: The radar tells the computer how high above the Moon
; the spacecraft is flying. This routine checks that the radar data makes sense
; and then updates the computer's position estimate.
;
; CODE-ALONG READERS: UPDATCHK checks flags to determine if landing radar updates
; are permitted, then POSUPDAT transforms the radar range measurement into a
; position correction along the radar beam direction. The measurement is validated
; against expected values before being incorporated into the state vector.
; ============================================================================

; First check: Is landing radar updating currently inhibited?
; The crew or guidance program can inhibit LR updates during certain phases.
; If NOLRRBIT flag is set in FLGWRD11, skip all LR processing this cycle.

UPDATCHK	CAF	NOLRRBIT	# SEE IF LR UPDATE INHIBITED.
		MASK	FLGWRD11
		CCS	A
		TCF	CONTSERV	# IT IS -- NO LR UPDATE

; No inhibit - check if we have new altitude measurement data this cycle.
; The RNGEDBIT flag in FLGWRD11 indicates fresh altitude data from radar.

		CAF	RNGEDBIT	# NO INHIBIT -- SEE ALT MEAS. THIS CYCLE.
		MASK	FLGWRD11
		EXTEND
		BZF	VMEASCHK	# NO ALT MEAS THIS CYCLE -- CHECK FOR VEL

; ============================================================================
; POSUPDAT - Position Update from Landing Radar Altitude
;
; Transform radar altitude measurement into a position correction along the
; radar beam direction. The radar measures slant range to the surface below,
; which must be converted to a position update in the navigation frame.
;
; The calculation projects the radar beam direction into the stable member
; frame and computes how much the measured range differs from the expected
; range based on current velocity and time since last measurement.
; ============================================================================

POSUPDAT	CA	FIXLOC		# SET PUSHLIST TO ZERO
		TS	PUSHLOC

; Enter interpretive mode for vector/matrix operations.
; Transform radar beam vector from navigation base to stable member coordinates.

		TC	INTPRET
		VLOAD	VXM
			HBEAMNB		; Landing radar beam unit vector (nav base)
			XNBPIP		# HBEAM SM AT 2(2)

; Store transformed beam vector in pushlist, then load current velocity.
; Velocity is scaled at 2^5 meters/centisecond for computation.

		PDVL	VSL2		# STORE HBEAM IN PD 0-5
			V1S		# SCALE V AT 2(5) M/CS

; Add surface velocity correction and compute velocity component along beam.
; This gives the rate of range change we expect from vehicle motion.

		VAD	DOT
			DELVS		# V RELATIVE TO SURFACE AT 2(5) M/CS
			0D		# V ALONG HBEAM AT 2(7) M/CS.

; Scale the velocity component to radar count units for comparison with
; measured range. RADSKAL converts from m/cs to radar counts × 5.

		DMP	EXIT
			RADSKAL		# SCALE TO RADAR COUNTS X 5

; Check the landing radar altitude scale factor.
; The radar has two scale ranges: HIGH SCALE (high altitude) and LOW SCALE
; (low altitude, closer to surface). Different scaling is needed for each mode.

		CS	FLGWRD12	# TEST LR ALTITUDE SCALE FACTOR
		MASK	ALTSCBIT
		EXTEND
		BZF	+3		# BRANCH IF HIGH SCALE

; If low scale mode, apply additional scaling factor.
; Low scale provides finer resolution for final approach.

		CA	SKALSKAL	# RESCALE IF LOW SCALE
		TC	SHORTMP

; Now correct the measured altitude for Doppler effect caused by vehicle motion.
; The Doppler correction accounts for range rate along the radar beam.
; HMEAS contains the raw radar range measurement in radar counts.

 	+3	TC	INTPRET
 		DAD	SL		# CORRECT HMEAS FOR DOPPLER EFFECT
			HMEAS		; Add Doppler correction to measurement
			7D		; Shift left 7 for scaling

; Convert slant range measurement to a position vector along beam direction.
; Multiply scalar range by beam unit vector to get range vector.

		DMP	VXSC		# SLANT RANGE AT 2(21), PUSH UP FOR HBEAM
			HSCAL		# SLANT RANGE VECTOR AT 2(23) M

; Project slant range onto vertical (radial from Moon center) to get altitude.
; Then subtract expected altitude (HCALC) to get altitude error (DELTA H).
; This error will be used to correct the position state vector.

		DOT	DSU
			UNIT/R/		# ALTITUDE AT 2(24) M
			HCALC		# DELTA H AT 2(24) M
		STORE	DELTAH
		EXIT

; ============================================================================
; Altitude Measurement Reasonableness Test
;
; Before accepting the radar altitude measurement, perform a reasonableness
; check to detect gross errors. The test compares the altitude difference
; (DELTAH) against a threshold that depends on expected altitude. This prevents
; bad radar data from corrupting the navigation state.
;
; During Apollo 11's descent, this check protected against occasional radar
; glitches that could have caused the guidance to diverge from reality.
; ============================================================================

; Check if position test inhibit flag is set. If set (before HIGATE), skip
; the reasonableness test and accept the measurement unconditionally.

		CA	FLGWRD11
		MASK	PSTHIBIT
		EXTEND			# DO NOT PERFORM DATA REASONABLENESS TEST
		BZF	NOREASON	# UNTIL AFTER HIGATE

# Page 885
; Perform altitude reasonableness test:
; Check if |DELTAH| > 50 feet + HCALC/8
; This threshold increases with altitude, allowing larger deviations at
; high altitude where radar is less accurate.

		TC	INTPRET
		ABS	DSU
			DELQFIX		# ABS(DELTAH) - DQFIX	50 FT NOM
		SL3	DSU		# SCALE TO 2(21)
			HCALC		# ABS(DELTAH) - (50 + HCALC/8) AT 2(21)
		EXIT

; Increment landing radar reasonableness counter and check result.
; If altitude error is too large (positive result from test above),
; branch to HFAIL to set the altitude fail lamp.

		INCR	LRLCTR
		TC	BRANCH
		TCF	HFAIL		# DELTA H TOO LARGE
		TCF	HFAIL		# DELTA H TOO LARGE

; Altitude passed reasonableness test - turn off altitude fail indicator.

		TC	DOWNFLAG	# TURN OFF ALT FAIL LAMP
		ADRES	HFLSHFLG

; ============================================================================
; Position Update Application with Altitude-Dependent Weighting
;
; Apply the altitude correction to the position state vector using a weight
; that decreases as altitude increases. This weight function (WH) prevents
; large position corrections at high altitude where radar accuracy is lower,
; while allowing full corrections near the surface where radar is most accurate.
; ============================================================================

NOREASON	CS	FLGWRD11
		MASK	LRINHBIT
		CCS	A
		TCF	VMEASCHK	# UPDATE INHIBITED -- TEST VELOCITY ANYWAY

; Altitude measurement passed checks or test was inhibited.
; Now apply position correction with altitude-dependent weighting.

		TC	INTPRET		# DO POSITION UPDATE
		DLOAD	SR4
			HCALC		# RESCALE H TO 2(28)M
		EXIT

; Check if altitude exceeds maximum allowed for update (HMAX).
; If HCALC > HMAX, bypass the position update entirely.

		EXTEND
		DCA	DELTAH		# STORE DELTAH IN MPAC AND
		DXCH	MPAC		# BRING HCALC INTO A,L
		TC	ALSIGNAG
		EXTEND			# IF HIGH PART OF HCALC IS NON-ZERO, THEN
		BZF	+2		# HCALC > HMAX,
		TCF	VMEASCHK	# SO UPDATE IS BYPASSED
		TS	MPAC +2		#	FOR LATER SHORTMP

; Compute altitude-dependent weight: WH(1 - H/HMAX)
; Weight decreases linearly from WH at surface (H=0) to zero at HMAX.
; This provides smooth transition from full weighting to no update.

		CS	L		# -H AT 2(14) M
		AD	LRHMAX		# HMAX - H
		EXTEND
		BZMF	VMEASCHK	# IF H >HMAX, BYPASS UPDATE
		EXTEND
		MP	LRWH		# WH(HMAX - H)
		EXTEND
		DV	LRHMAX		# WH(1 - H/HMAX)
		TS	MPTEMP

; Multiply altitude error by computed weight: DELTAH × WH × (1 - H/HMAX)
; This gives the weighted position correction magnitude.

		TC	SHORTMP2	# DELTAH (WH)(1 - H/HMAX) IN MPAC

; Convert scalar correction to vector along local vertical (UNIT/R/).
; Add weighted correction to current position vector R1S to get new position.
; Store result in GNUR and call MUNGRAV to recompute gravity at new position.

		TC	INTPRET		# MODE IS DP FROM ABOVE
		SL1
		VXSC	VAD
			UNIT/R/		# DELTAR = DH(WH)(1 - H/HMAX) UNIT/R/
			R1S
		STCALL	GNUR
			MUNGRAV
		EXIT

# Page 886
; Position update complete. Set phase for restart protection.

		TC	QUIKFAZ5

; Store updated position into navigation state vector.

		CA	ZERO
RUPDATED	TC	GNURVST

; ============================================================================
; TRANSITION: From Altitude Update to Velocity Processing
;
; With altitude measurement processed and position potentially updated, the
; servicer now checks for available velocity data from the landing radar.
; The radar can measure velocity along three beam axes (X, Y, Z) which are
; combined with position knowledge to correct the spacecraft's velocity vector.
;
; During Apollo 11's landing, velocity data became available at lower altitudes
; and provided critical information for Armstrong and Aldrin to monitor their
; descent rate during the final approach phase.
; ============================================================================

; ============================================================================
; Velocity Measurement Check and Processing (VMEASCHK/VELUPDAT)
;
; These routines process landing radar velocity measurements along the radar
; beam axes. The velocity data is transformed from the radar's coordinate frame
; to the stable member frame, compared against the estimated velocity for
; reasonableness, and then incorporated with altitude-dependent weighting.
; ============================================================================

VMEASCHK	TC	QUIKFAZ5	# RESTART AT NEXT LOCATION

; Check if velocity data is available. The VELDABIT flag indicates whether
; a valid velocity measurement has been received from the radar.

		CS	FLGWRD11
		MASK	VELDABIT	# IS V READING AVAILABLE?
		CCS	A
		TCF	VALTCHK		# NO:  SEE IF V READING TO BE TAKEN

; Velocity data available - process it.
; VSELECT indicates which beam (0=Z, 1=Y, 2=X) provides this measurement.

VELUPDAT	CS	VSELECT		# PROCESS VELOCITY DATA
		TS	L
		ADS	L		# -2 VSELECT IN L
		AD	L
		AD	L		# -6 VSELECT IN A
		INDEX	FIXLOC
		DXCH	X1		# X1 = -6 VSELECT, X2 = -2 VSELECT

; Prepare to transform velocity measurement from radar beam frame (NB) to
; stable member frame (SM). First, load landing radar gimbal angles (CDUs)
; which define the orientation of the radar beams relative to the LM body.

		CA	EBANK4
		TS	EBANK
		EBANK=	LRXCDU

		CA	LRYCDU		# STORE LRCDUS IN CDUSPOTS
		TS	CDUSPOT
		CA	LRZCDU
		TS	CDUSPOT +2
		CA	LRXCDU
		TS	CDUSPOT +4

; Compute sines and cosines of gimbal angles for coordinate transformation.

		TC	BANKCALL
		CADR	QUICTRIG	# GET SINES AND COSINES FOR NBSM

		CA	FIXLOC
		TS	PUSHLOC		# SET PD TO ZERO

; Transform velocity measurement from navigation base (NB) to stable member (SM).
; The radar measures velocity along its beam axis in the NB frame. This must be
; rotated to SM coordinates to compare with the estimated velocity vector.

		TC	INTPRET
		VLOAD*	CALL
			VZBEAMNB,1	# CONVERT VBEAM FROM NB TO SM
			*NBSM*
		PDDL	SL		# STORE IN PD 0-5
			VMEAS		# LOAD VELOCITY MEASUREMENT
			12D
		DMP*	PUSH		# SCALE TO M/CS AT 2(6)
			VZSCAL,2	# AND STORE IN PD 6-7
		EXIT

; ============================================================================
; Velocity Reasonableness Test and Estimated Velocity Computation
;
; Compute the expected velocity (VU) using navigation state and gravity, then
; compare the radar-measured velocity component with this estimate. The test
; accepts measurements within 7.5 + VM/8 m/cs, which allows larger errors at
; higher velocities. This prevents incorporation of spurious radar readings.
; ============================================================================

		CS	ONE
		TS	MODE		# CHANGE STORE MODE TO VECTOR

; Store DELV (accumulated velocity increments from PIPAs) in MPAC for
; subsequent vector operations. DELV represents ΔV since last update.

		CA	PIPTEM		# STORE DELV IN MPAC
# Page 887
		ZL
		DXCH	MPAC

		CA	PIPTEM +1
		ZL
		DXCH	MPAC +3

		CA	PIPTEM +2
		ZL
		DXCH	MPAC +5

; Compute estimated velocity VU using navigation equation:
; VU = V(N-1) + DELVU + G(N-1) × DTU - moon_rotation
; where V(N-1) is previous velocity, DELVU is PIPA-measured ΔV,
; G(N-1) is previous gravity acceleration, and DTU is time interval.

		CA	EBANK7
		TS	EBANK		# RESTORE EBANK 7
		EBANK=	DVCNTR
		TC	INTPRET
		VXSC	PDDL
			KPIP1		# SCALE DELV TO 2(7) M/CS AND PUSH
			LRVTIME		# TIME OF DELV AT 2(28) CS
		DSU	DDV
			PIPTIME		# TU - T(N-1)
			2SEC(28)
		VXSC	VSL1		# G(N-1)(TU - T(N-1))
			GDT/2		# SCALED AT 2(7) M/CS
		VAD	VAD		# PUSH UP FOR DELV
			V		# VU = V(N-1) + DELVU + G(N-1) DTU
		VSL2	VAD		# SCALE TO 2(5) M/CS AND SUBTRACT
			DELVS		#	MOON ROTATION.

; Perform velocity reasonableness test:
; Accept measurement if ABS(VMEAS - VEST) < 7.5 + ABS(VM)/8
; where VMEAS is radar measurement, VEST is computed estimate, and VM is
; measured velocity magnitude. The threshold increases with velocity magnitude
; to account for larger navigation uncertainties at high speeds.

		PUSH	ABVAL		# STORE IN PD
		SR4	DAD		# ABS(VM)/8 + 7.5 AT 2(6)
			7.5
		STOVL	20D		# STORE IN 20D AND PICK UP VM
		DOT	BDSU		# V(EST) AT 2(6)
			0		# DELTAV = VMEAS - V(EST)
		PUSH	ABS
		DSU	EXIT		# ABS(DV) - (7.5 + ABS(VM)/8))
			20D

; Check result of reasonableness test. Increment measurement counter and
; branch based on sign of (ABS(DV) - threshold).
; Negative or zero: measurement reasonable, continue to update
; Positive: measurement unreasonable, reject with alarm

		INCR	LRMCTR
		TC	BRANCH
		TCF	VFAIL		# DELTA V TOO LARGE.	ALARM
		TCF	VFAIL		# DELTA V TOO LARGE.	ALARM

		TC	DOWNFLAG	# TURN OFF VEL FAIL LAMP
		ADRES	VFLSHFLG

		CA	FLGWRD11
		MASK	VXINHBIT
		EXTEND
		BZF	VUPDAT		# IF VX INHIBIT RESET, INCORPORATE DATA.
# Page 888
		TC	DOWNFLAG
		ADRES	VXINH		# RESET VX INHIBIT

		CA	VSELECT
		AD	NEG2		# IF VSELECT = 2 (X AXIS).
		EXTEND			# BYPASS UPDATE
		BZF	ENDVDAT

VUPDAT		CS	FLGWRD11
		MASK	LRINHBIT
		CCS	A
		TCF	VALTCHK		# UPDATE INHIBITED

		TS	MPAC +1

		CA	ABVEL		# STORE E7 ERASABLES NEEDED IN TEMPS
		TS	ABVEL*
		CA	VSELECT
		TS	VSELECT*
		CA	EBANK5
		TS	EBANK		# CHANGE EBANKS

		EBANK=	LRVF
		CS	LRVF
		AD	ABVEL*		# IF V < VF, USE WVF
		EXTEND
		BZMF	USEVF

		CS	ABVEL*
		AD	LRVMAX		# VMAX - V
		EXTEND
		BZMF	WSTOR -1	# IF V > VMAX, W = 0

		EXTEND
		INDEX	VSELECT*
		MP	LRWVZ		# WV(VMAX - V)

		EXTEND
		DV	LRVMAX		# WV( 1 - V/VMAX )
		TCF	WSTOR

USEVF		INDEX	VSELECT*
		CA	LRWVFZ		# USE APPROPRIATE CONSTANT WEIGHT
		TCF	WSTOR

 -1		CA	ZERO
WSTOR		TS	MPAC
		CS	BIT7		# (=64D)
		AD	MODREG
		EXTEND
# Page 889
		BZMF	+3		# IF IN P65,P66,P67, USE ANOTHER CONSTANT

		CA	LRWVFF
		TS	MPAC

 +3		CA	EBANK7
 		TS	EBANK		# CHANGE EBANKS

		EBANK=	ABVEL
		TC	INTPRET
		DMP	VXSC		# W(DELTA V)(VBEAMSM) UP 6-7, 0-5
		VAD
			V1S		# ADD WEIGHTED DELTA V TO VELOCITY
		STORE	GNUV
		EXIT

		TC	QUIKFAZ5	# DO NOT RE-UPDATE

		CA	SIX
VUPDATED	TC	GNURVST		# STORE NEW VELOCITY VECTOR
ENDVDAT		=	VALTCHK

VALTCHK		TC	QUIKFAZ5	# DO NOT REPEAT ABOVE

		CAF	READVBIT	# TEST READVEL TO SEE IF VELOCITY READING
		MASK	FLGWRD11	# IS DESIRED.
		CCS	A
		TCF	READV		# YES -- READ VELOCITY
		CS	ABVEL		# NO -- SEE IF VELOCITY < 2000 FT/SEC
		AD	2KFT/SEC
		EXTEND
		BZMF	CONTSERV	# V > 2000 FT/SEC  DO NOT READ VEL

		TC	UPFLAG		# V < 2000 FT/SEC  SET READVEL AND READ.
		ADRES	READVEL

READV		CAF	PRIO32		# SET UP JOB TO READ VELOCITY BEAMS.
		TC	NOVAC
		EBANK=	HMEAS
		2CADR	LRVJOB

		TCF	CONTSERV	# CONTINUE WITH SERVICER

GNURVST		TS	BUF		# STORE GNUR (=GNUV) IN R1S OR V1S
		EXTEND			# A = 0 FOR R, A = 6 FOR V
		DCA	GNUR
		INDEX	BUF
		DXCH	R1S
		EXTEND
# Page 890
		DCA	GNUR +2
		INDEX	BUF
		DXCH	R1S +2
		EXTEND
		DCA	GNUR +4
		INDEX	BUF
		DXCH	R1S +4
		TC	Q

QUIKFAZ5	CA	EBANK3
		XCH	EBANK		# SET EBANK 3
		DXCH	L		# Q TO A, A TO L
		EBANK=	PHSNAME5
		TS	PHSNAME5
		LXCH	EBANK
		EBANK=	DVCNTR
		TC	A

; ============================================================================
; LANDING RADAR FAILURE HANDLING
;
; HFAIL and VFAIL routines handle horizontal and vertical velocity channel
; failures respectively. These routines track failure statistics and decide
; when to illuminate tracker fail lights on the DSKY. During Apollo 11's
; descent, radar data quality was continuously monitored to ensure guidance
; accuracy. Multiple consecutive bad readings trigger crew warnings.
; ============================================================================

; Horizontal velocity channel failure handler.
; Checks failure counters LRRCTR (R) and LRLCTR (L) to determine if
; persistent failures warrant crew notification via tracker fail light.
; Algorithm: If R=0 or (L-R)<4, no light. Otherwise illuminate fail light.
; This prevents spurious warnings from single bad readings.
HFAIL		CS	LRRCTR		; Load -R (good reading counter)
		EXTEND
		BZF	NORLITE		# IF R = 0, DO NOT TURN ON TRK FAIL
		AD	LRLCTR		; Form L-R (bad vs good reading difference)
		MASK	NEG3		; Test if (L-R) > 3
		EXTEND			# IF L-R LT 4, DO NOT TURN ON TRK FAIL
		BZF	+2		; Less than 4 difference - no alarm yet
		TCF	NORLITE

		TC	UPFLAG		# AND SET BIT TO TURN ON TRACKER FAIL LITE
		ADRES	HFLSHFLG	; Illuminate horizontal velocity fail light

NORLITE		CA	LRLCTR		; Update good counter
		TS	LRRCTR		# SET R = L

		TCF	VMEASCHK	; Continue with velocity measurement check

; Vertical velocity channel failure handler.
; Checks failure counters LRSCTR (S) and LRMCTR (M) to determine if
; persistent vertical velocity failures warrant crew notification.
; Algorithm: If S=0 or (M-S)>3, illuminate fail light to warn crew.
; During descent, vertical velocity is critical for rate-of-descent control.
VFAIL		CS	LRSCTR		# DELTA Q LARGE - load -S (good counter)
		EXTEND			# IF S = 0, DO NOT TURN ON TRACKER FAIL
		BZF	NOLITE		; No good readings yet - skip light
		AD	LRMCTR		# M-S (form bad vs good difference)
		MASK	NEG3		# TEST FOR M-S > 3
		EXTEND			# IF M-S > 3, THEN TWO OR MORE OF THE
		BZF	+2		# 	LAST FOUR V READINGS WERE BAD,
		TCF	NOLITE		#	SO TURN ON VELOCITY FAIL LIGHT

		TC	UPFLAG		# AND SET BIT TO TURN ON TRACKER FAIL LITE
		ADRES	VFLSHFLG	; Illuminate vertical velocity fail light

# Page 891
NOLITE		CA	LRMCTR		# SET S = M
		TS	LRSCTR		; Update good counter from bad counter

		CCS	VSELECT		# TEST FOR Z COMPONENT
		TCF	ENDVDAT		# NOT Z, DO NOT SET VX INHIBIT

; If Z component failed, inhibit X component on next cycle.
; Cross-lobe lock-up on X axis can cause spurious Z failures.
; Setting VXINH flag skips next X reading to break the lock-up condition.
		TC	UPFLAG		# Z COMPONENT - SET FLAG TO SKIP X
		ADRES	VXINH		# COMPONENT, AS ERROR MAY BE DUE TO CROSS
		TCF	ENDVDAT		# LOBE LOCK UP NOT DETECTED ON X AXIS.

# Page 892
; ============================================================================
; LANDING RADAR VELOCITY MEASUREMENT JOB
;
; LRVJOB is initiated when LM descends below 15,000 feet during landing phase.
; This job manages landing radar velocity sampling - critical for guidance
; during final descent. During Apollo 11's landing on July 20, 1969, these
; velocity measurements enabled Armstrong's semi-manual control as Eagle
; descended to Tranquility Base. The job coordinates 5 velocity samples over
; ~500ms while reading IMU gimbal angles mid-sampling for data correlation.
; ============================================================================
# ********************************************************************************
# LRVJOB IS SET WHEN THE LEM IS BELOW 15000 FT DURING THE LANDING PHASE
# THIS JOB INITIALIZES THE LANDING RADAR READ ROUTINE FOR 5 VELOCITY
# SAMPLES AND GOES TO SLEEP WHILE THE SAMPLING IS DONE -- ABOUT 500 MS.
# WITH A GOODEND RETURN THE DATA IS STORED IN VMEAS AND BIT7 OF LRSTAT
# IS SET.  THE GIMBAL ANGLES ARE READ ABOUT MIDWAY IN THE SAMPLINGS.

170MS		EQUALS	ND1		; 170ms delay constant

; Landing radar velocity measurement job entry point.
; Sequences velocity beam selector, initiates 5-sample radar reading cycle,
; and stores validated measurements in VMEAS for guidance use.
LRVJOB		CA	170MS		# SET TASK TO READ CDUS + PIPAS
		TC	WAITLIST	; Schedule RDGIMS task in 170ms
		EBANK=	LRVTIME
		2CADR	RDGIMS		; Read gimbals/PIPAs midway through sampling

; Sequence through velocity beam selector (X, Y, Z components).
; VSELECT cycles 2→1→0→2... to sample all three velocity components.
; Each component requires separate radar beam pointing and sampling cycle.
		CCS	VSELECT		# SEQUENCE LR VEL BEAM SELECTOR
		TCF	+2		; VSELECT was 2 or 1 - will decrement next
		CAF	TWO		# IF ZERO, RESET TO TWO (cycle complete)
		DOUBLE			# 2XVSELECT USED FOR VBEAM INDEX IN LRVEL
		TC	BANKCALL	# GO INITIALIZE LR VEL READ ROUTINE
		CADR	LRVEL		; Start radar velocity sampling sequence
		TC	BANKCALL	# PUT LRVJOB TO SLEEP ABOUT 500 MS
		CADR	RADSTALL	; Wait for 5-sample averaging to complete
		TCF	VBAD		; Bad return - data invalid, retry later
		CCS	STILBADV	# IS DATA GOOD JUST PRESENT?
		TCF	VSTILBAD	# JUST GOOD -- MUST WAIT 4 SECONDS.

; Good velocity data received - store for guidance use.
; VMEAS contains spacecraft velocity components from landing radar.
; This data is critical during final descent - Armstrong monitored these
; velocity values on the DSKY during Apollo 11's landing approach.
		INHINT			; Protect critical data transfer
		EXTEND			# GOOD RETURN -- STOW AWAY VMEAS
		DCA	SAMPLSUM	; Load averaged velocity measurement (DP)
		DXCH	VMEAS		; Store in VMEAS for guidance algorithms
		CA	EBANK4		# FOR DOWNLINK
		TS	EBANK		; Switch to downlink data bank
		EBANK=	LRVTIME

; Store radar measurement time and gimbal angles for telemetry downlink.
; Ground controllers monitored these values to verify radar data quality
; during descent. Time-tagging enables correlation with other sensor data.
		EXTEND
		DCA	LRVTIME		; Radar velocity measurement timestamp (DP)
		DXCH	LRVTIMDL	; Store for telemetry downlink
		EXTEND
		DCA	LRXCDU		; LR X and Y gimbal angles (DP)
		DXCH	LRXCDUDL	; Store for downlink
		CA	LRZCDU		; LR Z gimbal angle
		TS	LRZCDUDL	; Store for downlink
		CA	EBANK7		; Restore servicer EBANK
		TS	EBANK
		EBANK=	VSELECT

		CS	FLGWRD11	# SET BIT TO INDICATE VELOCITY
		MASK	VELDABIT	# MEASUREMENT MADE (set flag for guidance)
# Page 893
		ADS	FLGWRD11	; Set velocity data available flag

; Update velocity beam selector for next measurement cycle.
; Decrement VSELECT to sequence through X→Y→Z velocity components.
; When reaching zero, reset to 2 to restart the 3-axis cycle.
ENDLRV		CCS	VSELECT		# UPDATE VSELECT
		TCF	+2		; Was 2 or 1 - just store decremented value
		CA	TWO		; Was 0 - reset to 2 for new cycle
		TS	VSELECT		; Store updated selector
		TCF	ENDOFJOB	; Velocity measurement job complete

; Bad velocity data handler.
; When radar returns invalid data (noise, signal loss, or out-of-range),
; set STILBADV counter to wait 4 seconds before trusting data again.
; This prevents guidance from using transient bad readings.
VBAD		CAF	TWO		# SET STILBAD TO WAIT 4 SECONDS
VSTILBAD	TS	STILBADV	; Initialize or maintain bad data wait period
		TCF	ENDLRV		; Continue to next measurement cycle

; ============================================================================
; LANDING RADAR ALTITUDE MEASUREMENT JOB
;
; LRHJOB is initiated when LM descends below 25,000 feet during landing phase.
; This job manages landing radar altitude (height) measurements - essential for
; lunar landing. During Apollo 11's descent, this radar provided the altitude
; data that appeared on Armstrong's and Aldrin's displays, enabling them to
; judge their height above the lunar surface during the final approach.
; The job completes altitude sampling in ~95ms and stores validated data in HMEAS.
; ============================================================================
# LRHJOB IS SET BY LRHTASK WHEN LEM IS BELOW 25000 FT.  THIS JOB
# INITIALIZES THE LR READ ROUTINE FOR AN ALT MEASUREMENT AND GOES TO
# SLEEP WHILE THE SAMPLING IS DONE -- ABOUT 95 MS.  WITH A GOODEND RETURN
# THE ALT DATA IS STORED IN HMEAS AND BIT7 OF LRSTAT IS SET.

		BANK	34
		SETLOC	R12STUFF
		BANK

		COUNT*	$$/SERV

; Landing radar altitude measurement job entry point.
; Initiates altitude beam sampling, waits for measurement completion (~95ms),
; and stores validated altitude in HMEAS for guidance and crew displays.
; Altitude resolution: 1.079 ft/bit for precise landing site approach.
LRHJOB		TC	BANKCALL	# INITIATE LR ALT MEASUREMENT
		CADR	LRALT		; Start radar altitude sampling sequence
		TC	BANKCALL	# LRHJOB TO SLEEP ABOUT 95MS
		CADR	RADSTALL	; Wait for altitude measurement completion
		TCF	HBAD		; Bad return - invalid altitude data
		CCS	STILBADH	# IS DATA GOOD JUST PRESENT?
		TCF	HSTILBAD	# JUST GOOD -- MUST WAIT 4 SECONDS.

; Good altitude data received - store for guidance and displays.
; HMEAS contains radar altitude above lunar surface (1.079 ft/bit).
; During Apollo 11 landing, this altitude value appeared on crew displays
; and drove the guidance equations that controlled descent engine throttle.
		INHINT			; Protect critical data storage
		EXTEND
		DCA	SAMPLSUM	# GOOD RETURN -- STORE AWAY LRH DATA
		DXCH	HMEAS		# LRH DATA 1.079 FT/BIT (altitude measurement)
		EXTEND			# FOR DOWNLINK
		DCA	PIPTIME1	; Measurement timestamp for telemetry
		DXCH	MKTIME		; Store time for ground correlation

; Store IMU gimbal angles at time of altitude measurement.
; These angles (AIG, AMG, AOG) enable transformation of radar data
; from antenna-fixed coordinates to inertial navigation frame.
		EXTEND
		DCA	CDUTEMPY	# CDUY,Z = AIG,AMG (inner, middle gimbal)
		DXCH	AIG		; Store for coordinate transformation

		CA	CDUTEMPX	# CDUX = AOG (outer gimbal angle)
		TS	AOG		; Store outer gimbal angle

		CS	FLGWRD11	# SET BIT TO INDICATE RANGE
		MASK	RNGEDBIT	# MEASUREMENT MADE (set flag for guidance)
		ADS	FLGWRD11	; Mark altitude data as valid and current
ENDLRH		TC	ENDOFJOB	# TERMINATE LRHJOB (altitude job complete)

# Page 894
; Bad altitude data handler with scale change detection.
; Landing radar operates at two ranges: low altitude mode and high altitude mode.
; When switching between modes, a transient "bad" return may occur that should
; be ignored (not a real data fault). This logic distinguishes between
; scale-change transients and actual bad altitude data.
HBAD		CA	FLAGWRD5	; Check for scale change condition
		MASK	RNGSCBIT	# IS BAD RETURN DUE TO SCALE CHANGE?
		EXTEND
		BZF	HSTILBAD -1	# NO  RESET HSTILBAD (true bad data)
		TC	DOWNFLAG	# YES  RESET SCALE CHANGE BIT AND IGNORE
		ADRES	RNGSCFLG	; Clear scale-change flag (transient)
		TC	ENDOFJOB	; Ignore bad return, wait for next cycle

; Set STILBADH counter to wait 4 seconds after actual bad altitude data.
; This prevents guidance from using intermittent bad readings.
		CAF	TWO		# SET STILBAD TO WAIT 4 SECONDS
HSTILBAD	TS	STILBADH	; Initialize bad data wait period
		TC	ENDOFJOB	; Altitude job complete (no data stored)

		BANK	34
		SETLOC	SERV4
		BANK

		COUNT*	$$/SERV

# RDGIMS IS A TASK SET UP BY LRVJOB TO PICK UP THE IMU CDUS AND TIME
# AT ABOUT THE MIDPOINT OF THE LR VEL READ ROUTINE WHEN 5 VEL SAMPLES
# ARE SPECIFIED.

; ============================================================================
; READ IMU GIMBALS TASK
;
; RDGIMS samples IMU gimbal angles and accelerometer outputs at the midpoint
; of landing radar velocity measurement when 5 velocity samples are specified.
; This synchronized sampling ensures that inertial data corresponds temporally
; with radar velocity measurements, enabling accurate coordinate frame
; transformations between antenna-fixed radar data and navigation coordinates.
; ============================================================================

		EBANK=	LRVTIME
; Sample inertial navigation data synchronized with radar velocity measurement.
; Captures mission time, gimbal angles (X,Y,Z), and accelerometer outputs
; for precise transformation of radar velocity data into navigation frame.
RDGIMS		EXTEND
		DCA	TIME2		# PICK UP TIME2, TIME1
		DXCH	LRVTIME		# Save timestamp for velocity measurement

		EXTEND
		DCA	CDUX		# PICK UP CDUX AND CDUY (gimbal angles)
		DXCH	LRXCDU		# Save X and Y gimbal angles

		CA	CDUZ		; Read Z gimbal angle
		TS	LRZCDU		# SAVE CDUZ IN LRZCDU (complete gimbal set)

; Capture PIPA (accelerometer) outputs for velocity integration.
; These measurements enable dead-reckoning navigation and cross-checking
; of radar velocity data during descent.
		CA	PIPAX		; Read X-axis accelerometer
		TS	PIPTEM		# SAVE PIPAX IN PIPTEM

		EXTEND
		DCA	PIPAY		# PICK UP PIPAY AND PIPAZ (Y and Z accelerometers)
		DXCH	PIPTEM +1	# Save Y and Z accelerometer readings
		TC	TASKOVER	; IMU sampling task complete

		BANK	33
		SETLOC	SERVICES
		BANK

		COUNT*	$$/SERV

		EBANK=	DVCNTR
# Page 895
# HIGATJOB IS SET APPROXIMATELY 6 SECONDS PRIOR TO HIGH GATE DURING
# THE DESCENT BURN PHASE OF LUNAR LANDING.  THIS JOB INITIATES THE
# LANDING RADAR REPOSITIONING ROUTINE AND GOES TO SLEEP UNTIL THE
# LR ANTENNA MOVES FROM POSITION 1 TO POSITION 2.  IF THE LR ANTENNA
# ACHIEVES POSITION 2 WITHIN 22 SECONDS THE ALTITUDE AND VELOCITY
# BEAM VECTORS ARE RECOMPUTED TO REFLECT THE NEW ORIENTATION WITH
# RESPECT TO THE NB.  BIT10 OF LRSTAT IS CLEARED TO ALLOW LR
# MEASUREMENTS AND THE JOB TERMINATES.

; ============================================================================
; HIGH GATE LANDING RADAR REPOSITIONING JOB
;
; HIGATJOB executes ~6 seconds before High Gate (altitude ~7500 ft) during
; powered descent. At this point, the LM's attitude changes require the
; landing radar antenna to slew from Position 1 (face-down) to Position 2
; (forward-tilted) for continued tracking. This job commands the antenna
; reposition and waits for mechanical completion (maximum 22 seconds).
;
; Upon successful repositioning, beam vectors are recalculated to reflect
; the antenna's new orientation, and radar measurements resume. Failure to
; reposition triggers program alarm 523, requiring crew decision to proceed
; without radar or abort the landing.
; ============================================================================

; Initiate landing radar antenna slew from Position 1 to Position 2.
; During Apollo 11 descent, this reposition occurred at approximately
; 7500 feet altitude to maintain radar tracking as LM pitched forward.
HIGATJOB	TC	BANKCALL	# START LRPOS2 JOB (antenna reposition)
		CADR	LRPOS2		; Command antenna to Position 2
		TC	BANKCALL	# PUT HIGATJOB TO SLEEP UNTIL JOB IS DONE
		CADR	RADSTALL	; Wait for mechanical slew completion
		TCF	POSALARM	# BAD END -- ALARM if antenna stuck

; Antenna successfully repositioned - configure radar for Position 2 operation.
; Recalculate beam vectors to reflect antenna's new orientation relative to
; navigation base (NB) coordinate frame, then enable radar measurements.
POSGOOD		CA	PRIO23		# REDUCE PRIORITY FOR INTERPRETIVE COMPS.
		TC	PRIOCHNG	; Lower priority for math-intensive work

		TC	SETPOS2		# LR IN POS2 -- SET UP TRANSFORMATIONS
					; Recalculate antenna beam geometry

		TC	DOWNFLAG	; Clear radar inhibit flag
		ADRES	NOLRREAD	# RESET NOLRREAD FLAG TO ENABLE LR READING
		TC	ENDOFJOB	; Antenna reposition complete - resume ops

; Antenna repositioning failure handler - displays alarm 523 to crew.
; Crew options: TERMINATE (abort landing), PROCEED (retry mechanism),
; or V32E (terminate R12 radar program and continue descent without radar).
; During Apollo 11, this alarm did not occur - antenna repositioned successfully.
POSALARM	CA	OCT523		; Alarm code 523: LR antenna position failure
		TC	BANKCALL
		CADR	PRIOLARM	# FLASH ALARM CODE on DSKY
		TCF	GOTOPOOH	# TERMINATE (crew abort landing)
		TCF	+3		# PROCEED -- TRY AGAIN (crew retry)
		TCF	ENDOFJOB	# V 32 E    TERMINATE R12 (no radar)
		TC	ENDOFJOB	; Should not reach here

; Crew selected PROCEED - check if antenna reached Position 2 yet.
; If achieved, continue to POSGOOD. If still moving, re-alarm.
 +3		CA	BIT7		# SEE IF IN POS2 YET
		EXTEND		; Read antenna position sensor
		RAND	CHAN33		; Channel 33, Bit 7 = Position 2 sensor
		EXTEND
		BZF	POSGOOD		# POS2 ACHIEVED -- SET UP ANTENNA BEAMS
		TCF	POSALARM	# STILL DIDN'T MAKE IT -- REALARM

OCT523		OCT	00523

; ============================================================================
; LANDING RADAR ANTENNA TRANSFORMATION SETUP ROUTINES
;
; SETPOS1 and SETPOS2 compute coordinate transformations for the landing radar
; antenna in its two positions. The antenna can face either down (Position 1,
; early descent) or forward-tilted (Position 2, final approach). These routines
; convert the antenna beam vectors (velocity, altitude, Z-axis) from the
; antenna's local coordinate frame to the navigation base (NB) coordinate frame,
; enabling the guidance computer to correctly interpret radar measurements
; regardless of antenna orientation.
;
; The transformations use rotation matrices defined by LRALPHA and LRBETA angles
; which specify the antenna's mounting orientation relative to the LM body.
; ============================================================================

; SETPOS1: Configure radar beam vectors for antenna Position 1 (face-down).
; Called during early descent phase when LM attitude is nearly vertical.
; Initializes bad-data counters and antenna geometry parameters before
; computing Position 1 beam transformations.
SETPOS1		TC	MAKECADR	# MUST BE CALLED BY BANKCALL
		TS	LRADRET1	# SAVE RETURN CADR.  SINCE BUP2 CLOBBERED

		CAF	TWO		; Reset radar data quality counters
		TS	STILBADH	# INITIALIZE STILBAD (altitude)
		TS	STILBADV	# INITIALIZE STILBAD (velocity)

		CA	ZERO		# INDEX FOR LRALPHA, LRBETA IN POS 1.
# Page 896
		TS	LRLCTR		# SET L,M,R, AND S TO ZERO
		TS	LRMCTR		; Radar beam counter initialization
		TS	LRRCTR
		TS	LRSCTR
		TS	VSELECT		# INITIALIZE VSELECT (velocity beam selector)

		TC	SETPOS		# CONTINUE WITH COMPUTATIONS.

		CA	LRADRET1	; Retrieve return address
		TC	BANKJUMP	# RETURN TO CALLER

; SETPOS2: Configure radar beam vectors for antenna Position 2 (forward-tilted).
; Called after successful antenna repositioning at High Gate. Loads Position 2
; rotation angles (index TWO) and performs identical transformation computations
; as SETPOS1 but for the antenna's new orientation.
SETPOS2		CA	TWO		# INDEX FOR POS2 (use second set of angles)

; SETPOS: Core transformation computation routine (common code for both positions).
; Converts antenna coordinate frame unit vectors (UNITX, UNITY, HBEAMANT) to
; navigation base coordinate frame using rotation matrices. The index in Q
; selects either Position 1 or Position 2 angle sets (LRALPHA, LRBETA).
;
; Computed vectors:
;   VXBEAMNB - Velocity beam X component in NB frame
;   VYBEAMNB - Velocity beam Y component in NB frame
;   VZBEAMNB - Velocity beam Z component in NB frame (computed as X cross Y)
;   HBEAMNB  - Altitude beam vector in NB frame
SETPOS		XCH	Q		# SAVE INDEX IN Q
		TS	LRADRET		# SAVE RETURN address

		CA	EBANK5		; Switch to erasable bank containing angles
		TS	EBANK
		EBANK=	LRALPHA

		EXTEND
		INDEX	Q		; Use index (0 for Pos1, 2 for Pos2)
		DCA	LRALPHA		# LRALPHA IN A, LRBETA IN L
		TS	CDUSPOT +4	# ROTATION ABOUT X (LRALPHA)
		LXCH	CDUSPOT		# ROTATION ABOUT Y (LRBETA)
		CA	ZERO
		TS	CDUSPOT +2	# ZERO ROTATION ABOUT Z (no yaw offset)

		CA	EBANK7		; Switch to erasable bank for computations
		TS	EBANK
		EBANK=	LRADRET

		TC	INTPRET		; Enter interpretive mode for vector math
		VLOAD	CALL		; Load antenna Y-axis unit vector
			UNITY		# CONVERT UNITY(ANTENNA) TO NB
			TRG*SMNB	; Transform: antenna → stable member → NB
		STOVL	VYBEAMNB	; Store velocity beam Y component in NB
			UNITX		# CONVERT UNITX(ANTENNA) TO NB
		CALL			; Transform antenna X-axis to NB
			*SMNB*
		STORE	VXBEAMNB	; Store velocity beam X component in NB
		VXV	VSL1		; Compute Z = X cross Y (right-hand rule)
			VYBEAMNB
		STOVL	VZBEAMNB	# Z = X * Y (velocity beam Z in NB)
			HBEAMANT	; Load altitude beam vector (antenna frame)
		CALL
			*SMNB*		# CONVERT TO NB
		STORE	HBEAMNB		; Store altitude beam vector in NB frame
		EXIT			; Return to native AGC code

# Page 897
		TC	LRADRET		; Return to caller with beams configured

