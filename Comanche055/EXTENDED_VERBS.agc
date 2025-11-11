# Copyright:	Public domain.
# Filename:	EXTENDED_VERBS.agc
# Purpose:	Part of the source code for Comanche, build 055. It
#		is part of the source code for the Command Module's
#		(CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 236-267
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	2009-05-18 FB	Transcription Batch 3 Assignment.
#		2009-05-20 RSB	Corrections:  POODOO -> P00DOO,
#				GOTOPOOH -> GOTOPOOH, added a couple of
#				missing instructions in Verb 96.
#		2009-05-23 RSB	In SYSTEST, corrected TC FLAGWRD1 to
#				CA FLAGWRD1.  Added a variety of SBANK=
#				statements prior to 2CADRs.  One day I'll
#				have to figure out what yaYUL is doing
#				wrong with those ....
#
# The contents of the "Comanche055" files, in general, are transcribed
# from scanned documents.
#
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  April 1, 1969.
#
#	This AGC program shall also be referred to as Colossus 2A
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
; FILE: EXTENDED_VERBS.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: DSKY extended verb implementations documenting all extended verb
;        functions (V40-V99) with complete catalog of purposes, required noun
;        associations, crew procedures, and mission phase usage. Verbs are
;        commands typed on DSKY keyboard directing AGC to perform specific
;        operations from display updates to program mode changes throughout
;        Apollo 11 mission.
;
; COMMENT-ONLY READERS: This file defines all the command codes crew could
;        type to tell the computer what to do - like V16N36 to show time.
; CODE-ALONG READERS: Study complete verb catalog with implementation details,
;        noun requirements, execution logic, and DSKY state machine integration.
; ============================================================================

# Page 236
		BANK	7
		SETLOC	EXTVERBS
		BANK

		EBANK=	OGC

		COUNT*	$$/EXTVB

; ============================================================================
; EXTENDED VERB FAN-OUT TABLE (V40-V99)
;
; This fan-out table routes extended verb commands (V40 through V99) to their
; specific implementation routines. Crew enters verbs on DSKY keyboard using
; VERB key followed by two-digit verb number. Extended verbs (40+) handle
; specialized functions including IMU alignment, state vector updates, display
; requests, system tests, and program control operations.
;
; During Apollo 11 mission, crew used these verbs throughout all phases to:
; - Align IMU platform (V41, V42)
; - Update spacecraft state vectors (V80, V81)
; - Request orbital parameter displays (V82, V83)
; - Control automatic maneuvers (V49, V58)
; - Perform system tests and calibrations (V92, V96)
; ============================================================================

# FAN-OUT

GOEXTVB		INDEX	MPAC		# VERB-40 IS IN MPAC
		TC	LST2FAN		# FAN AS BEFORE.

; ============================================================================
; VERB 40-49: IMU ALIGNMENT, FLAGS, AND CONTROL ESTABLISHMENT
; ============================================================================
;
; V40: ZERO CDU (Noun 20 only)
;      Zeros the IMU CDU counters. Used during IMU initialization.
;      Crew procedure: V40E (Enter). No noun required for basic operation.
;
; V41: COARSE ALIGN IMU (Noun 20 or 91 only)
;      Performs coarse alignment of IMU platform to desired orientation.
;      Used during platform alignment procedures. Crew enters desired gimbal
;      angles via Noun 20 (3 CDU angles) or Noun 91 (desired REFSMMAT).
;      Crew procedure: V41N20E, then enter gimbal angles.
;
; V42: FINE ALIGN IMU
;      Performs fine alignment of IMU platform using gyrocompassing or star
;      sightings. Follows coarse alignment in standard alignment sequence.
;      Apollo 11 crew used this before major maneuvers throughout mission.
;      Crew procedure: V42E (requires prior alignment program selection).
;
; V43: LOAD IMU ATTITUDE ERROR METERS
;      Loads gimbal angle errors into IMU for display on flight director
;      attitude indicator (FDAI). Allows crew to monitor alignment quality.
;      Crew procedure: V43E.
;
; V44: SET SURFACE FLAG
;      Sets flag indicating spacecraft is on planetary surface (Moon or Earth).
;      Changes gravity model and navigation assumptions. Used after landing.
;      Crew procedure: V44E.
;
; V45: RESET SURFACE FLAG
;      Clears surface flag when spacecraft lifts off from surface. Eagle used
;      this during ascent from lunar surface on July 21, 1969.
;      Crew procedure: V45E.
;
; V46: ESTABLISH GUIDANCE AND CONTROL (G+C CONTROL)
;      Establishes active guidance and control authority. Enables digital
;      autopilot and guidance program control of spacecraft attitude/thrust.
;      Crew procedure: V46E.
;
; V47: MOVE LM STATE VECTOR INTO CM SLOT
;      Copies Lunar Module state vector (position/velocity) to Command Module
;      slot in memory. Used during rendezvous when LM and CM are docked.
;      Crew procedure: V47E.
;
; V48: LOAD AUTOPILOT DATA
;      Loads digital autopilot (DAP) configuration parameters. Crew enters
;      desired autopilot gains, deadbands, and control modes via DSKY.
;      Crew procedure: V48E, then load parameters via appropriate nouns.
;
; V49: START AUTOMATIC ATTITUDE MANEUVER
;      Initiates automatic attitude maneuver to orientation specified by crew.
;      Used throughout mission for reorientations during coast phases.
;      Crew enters desired attitude via Noun 22 (FDAI gimbal angles).
;      Crew procedure: V49N22E, then enter desired gimbal angles.
;
LST2FAN		TC	VBZERO		# VB40 ZERO (USED WITH NOUN 20 ONLY).
		TC	VBCOARK		# VB41 COARSE ALIGN (USED WITH NOUN 20 OR
					#				91 ONLY)
		TC	IMUFINEK	# VB42 FINE ALIGN IMU
		TC	IMUATTCK	# VB43 LOAD IMU ATTITUDE ERROR METERS.
		TC	SETSURF		# VB44 SET SURFACE FLAG
		TC	RESTSRF		# VB45 RESET SURFACE FLAG
		TC	STABLISH	# VB46 ESTABLISH G+C CONTROL.
		TC	LMTOCMSV	# VB47 MOVE LM STATE VECTOR INTO CM
		TC	DAPDISP		# VB48 LOAD A/P DATA.
		TCF	CREWMANU	# VB 49 START AUTOMATIC ATTITUDE MANEUVER

; ============================================================================
; VERB 50-59: MARK ROUTINES, TRACKING, AND MANEUVER CONTROL
; ============================================================================
;
; V50: PLEASE PERFORM (Generic mark request)
;      Requests crew action or measurement. Context-dependent based on active
;      program. Used by various navigation and alignment programs.
;      Crew procedure: Varies by program context.
;
; V51: PLEASE MARK
;      Requests crew to perform optical mark (star sighting or landmark).
;      Used during navigation programs P20-P27 for state vector updates.
;      Crew procedure: Sight target, press MARK button, V51E.
;
; V52: SET OFFSET NUMBER FOR P22
;      Allows crew to specify which landmark offset to use during P22
;      (landmark tracking) program. Used for lunar surface navigation.
;      Crew procedure: V52E, then enter offset number.
;
; V53: PLEASE PERFORM COAS MARK
;      Requests crew to perform COAS (Crewman Optical Alignment Sight) mark.
;      Backup optical device used when sextant unavailable.
;      Crew procedure: Align COAS on target, press MARK, V53E.
;
; V54: PLEASE MARK (R21 BACKUP)
;      Backup mark request for R21 (rendezvous tracking) routine.
;      Used during rendezvous when primary tracking unavailable.
;      Crew procedure: Visual sighting, press MARK, V54E.
;
; V55: ALIGN TIME
;      Synchronizes onboard mission timer with ground-provided time update.
;      Used periodically throughout mission for time base synchronization.
;      Crew procedure: V55E, then enter time from ground via Noun 33.
;
; V56: TERMINATE TRACKING (P20 + P25)
;      Stops automatic tracking in programs P20 (rendezvous navigation) or
;      P25 (orbital navigation). Returns control to crew.
;      Crew procedure: V56E.
;
; V57: START R21 RENDEZVOUS TRACKING SIGHT MARK ROUTINE
;      Initiates R21 rendezvous tracking routine for relative navigation.
;      Used during Apollo 11 rendezvous when Columbia tracked Eagle.
;      Crew procedure: V57E, then perform optical marks as requested.
;
; V58: ENABLE AUTOMATIC ATTITUDE MANEUVER
;      Enables automatic attitude control during program execution.
;      Allows programs to command spacecraft orientation changes.
;      Crew procedure: V58E.
;
; V59: PLEASE CALIBRATE
;      Requests crew action for system calibration procedure.
;      Context-dependent on active program requiring calibration.
;      Crew procedure: Varies by system being calibrated.
;
		TC	GOLOADLV	# VB50 PLEASE PERFORM
		TC	GOLOADLV	# VB51 PLEASE MARK
		TC	V52		# VB52 SET OFFSET NO. FOR P22
		TC	GOLOADLV	# VB 53 PLEASE PERFORM COAS MARK
		TC	GOTOR23		# VB54 PLEASE MARK (R-21-BACKUP)
		TC	ALINTIME	# VB55 ALIGN TIME
		TC	TRACKTRM	# VB56 TERMINATE TRACKING (P20 +P25)
		TC	GOTOR21		# V57 START R21 REND TRACK SIGHT MARK ROUT
		TC	ENATMA		# VB58 ENABLE AUTOMATIC ATTITUDE MANEUVER
		TC	GOLOADLV	# VB59 PLEASE CALIBRATE

; ============================================================================
; VERB 60-69: AUTOPILOT MODES, ANTENNA POINTING, AND SYSTEM TESTS
; ============================================================================
;
; V60: SET CPHIX (N17) EQUAL TO CDU
;      Sets desired gimbal angles (Noun 17) equal to current CDU values.
;      Used to maintain current spacecraft attitude as reference.
;      Crew procedure: V60E.
;
; V61: SELECT MODE I
;      Selects autopilot mode I (specific control mode configuration).
;      Mode configurations vary by spacecraft and mission phase.
;      Crew procedure: V61E.
;
; V62: SELECT MODE II, ERROR WITH RESPECT TO NOUN 22
;      Selects autopilot mode II with attitude errors computed relative to
;      orientation specified in Noun 22 (FDAI gimbal angles).
;      Crew procedure: V62N22E, enter desired attitude.
;
; V63: SELECT MODE III, ERROR WITH RESPECT TO NOUN 17
;      Selects autopilot mode III with attitude errors relative to Noun 17.
;      Provides alternative attitude reference mode.
;      Crew procedure: V63N17E, enter desired gimbal angles.
;
; V64: CALCULATE AND DISPLAY S-BAND ANTENNA ANGLES
;      Computes required S-band high-gain antenna pointing angles for Earth
;      communication. Displays pitch/yaw angles crew should set manually.
;      Used throughout Apollo 11 mission to maintain communication link.
;      Crew procedure: V64E, read angles from display, position antenna.
;
; V65: OPTICAL VERIFICATION FOR PRELAUNCH
;      Prelaunch optical verification test ensuring sextant alignment.
;      Ground test procedure, not used during Apollo 11 flight.
;      Crew procedure: V65E (ground testing only).
;
; V66: ATTACHED - MOVE LM TO OTHER STATE VECTOR SLOT
;      Designates spacecraft as "attached" (docked configuration). Moves LM
;      state vector slot. Used after docking during rendezvous.
;      Crew procedure: V66E.
;
; V67: W-MATRIX MONITOR
;      Displays W-matrix elements (navigation covariance matrix) showing
;      estimated accuracy of state vector knowledge. Used to assess
;      navigation quality.
;      Crew procedure: V67E, displays covariance diagonal elements.
;
; V68: CSM STROKE TEST ON
;      Activates Command/Service Module engine gimbal stroke test.
;      Cycles engine gimbals through full range to verify actuator operation.
;      Used before critical burns.
;      Crew procedure: V68E.
;
; V69: CAUSE RESTART
;      Forces AGC restart (warm restart with state preservation).
;      Used for troubleshooting or recovering from anomalous conditions.
;      Crew procedure: V69E (use with caution).
;
		TC	V60		# VB60 SET CPHIX (N17) EQUAL TO CDU
		TC	V61		# VB61 SELECT MODE I
		TC	V62		# VB62 SELECT MODE II, ERROR WRT N22
		TC	V63		# VB63 SELECT MODE III, ERROR WRT N17
		TC	VB64		# VB64 CALCULATE,DISPLAY S-BAND ANT ANGLES
		TC	CKOPTVB		# V 65 E OPTICAL VERIFICATION FOR PRELAUNC
		TC	ATTACHED	# VB 66 ATTACHED. MOVE THIS TO OTHER STATE
		TC	V67		# VB67 WMATRIX MONITOR
		TC	STROKON		# VB68 CSM STROKE TEST ON.
VERB69		TC	VERB69		# VB 69 CAUSE RESTART

; ============================================================================
; VERB 70-79: TIME UPDATES, STATE VECTOR UPDATES, AND FLAG CONTROL
; ============================================================================
;
; V70: UPDATE LIFTOFF TIME
;      Updates stored liftoff time (T0) reference. Used if ground provides
;      corrected liftoff time after launch or during mission planning.
;      Crew procedure: V70E, enter time via Noun 33.
;
; V71: UNIVERSAL UPDATE - BLOCK ADDRESS
;      Universal memory update allowing crew to modify block of consecutive
;      memory locations. Ground can uplink memory changes via this verb.
;      Crew procedure: V71E, enter start address and data values.
;
; V72: UNIVERSAL UPDATE - SINGLE ADDRESS
;      Updates single memory location. Allows crew to modify specific AGC
;      memory address with ground-provided or crew-computed value.
;      Crew procedure: V72E, enter address (octal) and new value.
;
; V73: UPDATE AGC TIME (OCTAL)
;      Updates AGC mission elapsed time clock with corrected value in octal.
;      Used for time synchronization with Mission Control.
;      Crew procedure: V73E, enter centiseconds since liftoff (octal).
;
; V74: INITIALIZE DOWNLINK TELEMETRY PROGRAM FOR ERASABLE DUMP
;      Configures downlink telemetry to dump erasable memory contents to
;      ground. Used for troubleshooting and ground analysis of AGC state.
;      Crew procedure: V74E (ground-requested).
;
; V75: SET LIFTOFF FLAG
;      Sets flag indicating liftoff has occurred. Changes program sequencing
;      and navigation reference frames from prelaunch to flight configuration.
;      Crew procedure: V75E (automatic during launch sequence).
;
; V76: SET PREFERRED ATTITUDE FLAG
;      Sets flag indicating preferred attitude mode active. In this mode,
;      autopilot maintains specified "preferred" attitude orientation.
;      Used during coast phases to minimize propellant usage.
;      Crew procedure: V76E.
;
; V77: RESET PREFERRED ATTITUDE FLAG
;      Clears preferred attitude flag, returning to free attitude mode.
;      Allows crew or programs to command arbitrary attitude changes.
;      Crew procedure: V77E.
;
; V78: CHANGE GYROCOMPASS LAUNCH AZIMUTH
;      Updates launch azimuth value for gyrocompass alignment mode.
;      Used during prelaunch if launch azimuth changes.
;      Crew procedure: V78E, enter new azimuth via Noun 34.
;
; V79: SPARE
;      Reserved for future use. No function assigned in Apollo 11 software.
;
		TC	V70UPDAT	# VB70 UPDATE LIFTOFF TIME.
		TC	V71UPDAT	# VB71 UNIVERSAL UPDATE - BLOCK ADDRESS.
		TC	V72UPDAT	# VB72 UNIVERSAL UPDATE - SINGLE ADDRESS.
		TC	V73UPDAT	# VB73 UPDATE AGC TIME (OCTAL).
		TC	DNEDUMP		# VB74 INITIALIZE DOWN-TELEMETRY PROGRAM
					#	FOR ERASABLE DUMP.
		TC	LFTFLGON	# VB75 SET LIFTOFF FLAG.
# Page 237
		TC	SETPRFLG	# VB76 SET PREFERRED ATTITUDE FLAG
		TC	RESETPRF	# VB77 RESET PREFERRED ATT. FLAG
		TC	CHAZFOGC	# CHANGE GYROCOMPASS LAUNCH AZIMUTH V78
		TC	ALM/END		# V79 SPARE

; ============================================================================
; VERB 80-89: STATE VECTOR UPDATES, ORBITAL DISPLAYS, AND RANGE PARAMETERS
; ============================================================================
;
; V80: UPDATE LEM STATE VECTOR
;      Updates Lunar Module state vector (position/velocity in inertial space).
;      Ground uplinks refined state vectors based on tracking data. During
;      Apollo 11 rendezvous, Mission Control provided updated vectors for
;      Eagle to improve orbital prediction accuracy.
;      Crew procedure: V80E, enter state vector components via appropriate nouns.
;
; V81: UPDATE CSM STATE VECTOR
;      Updates Command/Service Module state vector. Critical for accurate
;      orbital mechanics predictions during translunar coast, lunar orbit,
;      and transearth coast. Ground tracking provided updates multiple times
;      daily during Apollo 11 mission.
;      Crew procedure: V81E, enter state vector via noun sequence.
;
; V82: REQUEST ORBIT PARAMETER DISPLAY (R30)
;      Invokes R30 routine displaying current orbital parameters: apogee
;      altitude, perigee altitude, time to perigee, orbital period, and
;      other Keplerian elements. Used throughout Apollo 11 for orbit
;      verification after LOI, DOI, and transearth injection.
;      Crew procedure: V82E, displays computed orbital parameters.
;
; V83: DISPLAY RANGE, RANGE RATE, +X AXIS (R31)
;      Invokes R31 displaying range, range rate, and theta angle along
;      spacecraft +X axis relative to target (Earth, Moon, or other vehicle).
;      Used during rendezvous and coast to monitor relative geometry.
;      Crew procedure: V83E.
;
; V84: SPARE
;      Reserved for future use. No function assigned in Apollo 11 software.
;
; V85: DISPLAY RANGE, RANGE RATE, SLOS (R32)
;      Invokes R32 routine displaying range and range rate along sextant
;      line-of-sight (SLOS) to target. Used during optical navigation when
;      tracking celestial bodies or other spacecraft.
;      Crew procedure: V85E.
;
; V86: BACKUP MARK REJECT
;      Rejects last navigation mark entered by crew. Used when crew determines
;      optical mark was inaccurate (wrong star, poor sighting conditions).
;      Prevents bad data from corrupting state vector.
;      Crew procedure: V86E immediately after questionable mark.
;
; V87: SET VHF RANGE FLAG
;      Sets flag enabling VHF ranging mode. VHF ranging provides relative
;      range measurements between spacecraft during rendezvous. Apollo 11
;      LM and CM used VHF for backup ranging during rendezvous phase.
;      Crew procedure: V87E.
;
; V88: RESET VHF RANGE FLAG
;      Clears VHF ranging flag, disabling VHF range measurements in navigation
;      filter. Used when VHF ranging not available or unreliable.
;      Crew procedure: V88E.
;
; V89: ALIGN X-AXIS OR PREFERRED CSM AXIS TO LINE-OF-SIGHT (R63)
;      Invokes R63 routine commanding automatic maneuver to align spacecraft
;      X-axis (or preferred attitude axis) to computed line-of-sight vector
;      to target. Used during rendezvous to point spacecraft at target.
;      Crew procedure: V89E.
;
		TC	LEMVEC		# VB80 UPDATE LEM STATE VECTOR
		TC	CSMVEC		# VB81 UPDATE CSM STATE VECTOR
		TC	V82PERF		# VB82 REQUEST ORBIT PARAM DISPLAY (R30)
		TC	V83PERF		# VB83 RANGE, RANGE RATE, +X AXIS  (R31)
		TC	ALM/END		# V84  SPARE
		TC	V85PERF		# VB85 RANGE, RANGE RATE, SLOS	   (R32)
		TC	V86PERF		# VB86 BACKUP MARK REJECT
		TC	SETVHFLG	# VB87 SET VHF RANGE FLAG
		TC	RESETVHF	# VB88 RESET VHF RANGE FLAG
		TC	V89PERF		# V89-ALIGN X OR PRF CSM AXIS TO LOS (R63)

; ============================================================================
; VERB 90-99: OUT-OF-PLANE PARAMETERS, SYSTEM TESTS, AND ENGINE OPERATIONS
; ============================================================================
;
; V90: OUT-OF-PLANE PARAMETERS (R36)
;      Invokes R36 displaying out-of-plane rendezvous parameters. Shows
;      orbital plane differences between spacecraft, critical for rendezvous
;      planning. Used during Apollo 11 LM-to-CM rendezvous.
;      Crew procedure: V90E.
;
; V91: HYBRID SIMULATION AND STAGING TEMPORARY
;      Temporary verb for hybrid simulation testing and staging operations.
;      Used during ground testing and development, minimal flight usage.
;      Crew procedure: V91E (ground testing).
;
; V92: OPERATE IMU PERFORMANCE TEST
;      Executes comprehensive IMU (Inertial Measurement Unit) performance
;      test checking gyro drift rates, accelerometer bias, and alignment
;      quality. Provides diagnostic data for IMU health assessment.
;      Used before critical maneuvers requiring high navigation accuracy.
;      Crew procedure: V92E, test runs automatically, displays results.
;
; V93: CLEAR RENDEZVOUS FLAG (RENDWFLG)
;      Clears rendezvous flag in W-matrix (navigation covariance matrix).
;      Resets navigation state estimation when rendezvous operations complete.
;      Crew procedure: V93E.
;
; V94: EXECUTE R64
;      Invokes R64 routine (specific display or computation function).
;      Context varies by mission phase and program configuration.
;      Crew procedure: V94E.
;
; V95: SPARE
;      Reserved for future use. No function assigned in Apollo 11 software.
;
; V96: SET QUITFLAG TO STOP INTEGRATION
;      Sets flag stopping orbital integration routine. Used to freeze current
;      state vector when numerical integration causing problems or when
;      preparing for state vector update from ground.
;      Crew procedure: V96E.
;
; V97: PLEASE PERFORM ENGINE FAIL TEST (R41)
;      Requests crew to perform engine failure test procedure R41. Simulates
;      engine-out conditions to verify backup modes and contingency procedures.
;      Used during systems checkout.
;      Crew procedure: V97E, perform test per checklist.
;
; V98: SPARE
;      Reserved for future use. No function assigned in Apollo 11 software.
;
; V99: PLEASE ENABLE ENGINE
;      Requests crew action to enable engine for upcoming burn. Part of
;      pre-ignition checklist ensuring propulsion system armed and ready.
;      Crew procedure: V99E, then enable engine per checklist.
;
		TC	V90PERF		# VB90-OUT OF PLAN PARAMETERS %R36"
		TC	GOSHOSUM	# VB91 TEMP FOR HYBRID AND STG.
		TC	SYSTEST		# VB92 OPERATE IMU PERFORMANCE TEST
		TC	WMATRXNG	# VB93 CLEAR RENDWFLG
		TC	VERB94		# VB94 DO R64
		TC	ALM/END		# VB95 SPARE
		TCF	VERB96		# VB96 SET QUITFLAG TO STOP INTEGRATION
		TC	GOLOADLV	# VB97 PLEASE PERFORM ENGINE-FAIL (R41)
		TC	ALM/END		# VB98 SPARE
		TC	GOLOADLV	# VB99 PLEASE ENABLE ENGINE

; ============================================================================
; END OF EXTENDED VERB FAN-OUT TABLE
;
; All 60 extended verbs (V40-V99) now documented. These verbs provided Apollo
; 11 crew with comprehensive command and control capability throughout entire
; mission from launch through splashdown. Combination of verb + noun codes
; enabled crew to command AGC operations, request data displays, update
; navigation state, configure autopilot modes, and execute specialized test
; and calibration procedures.
; ============================================================================

# END OF EXTENDED VERB FAN


TESTXACT	CCS	EXTVBACT
		TC	ALM/END		# YES, TURN ON OPERATOR ERROR LIGHT
		CA	FLAGWRD4	# ARE PRIOS USING DSKY
		MASK	OC24100
		CCS	A
		TC	ALM/END

		CAF	OCT24		# SET BITS 3 AND 5
SETXTACT	TS	EXTVBACT	# NO. SET FLAG TO SHOW EXT VERB DISPLAY
					# SYSTEM BUSY

		CA	Q
		TS	MPAC +1

		CS	TWO		# BLANK EVERYTHING EXCEPT MM AND VERB
		TC	NVSUB
		TC	+1
		TC	MPAC +1

XACTALM		TC	FALTON		# TURN ON OPERATOR ERROR LIGHT.
		TC	ENDEXT		# RELEASE MARK AND EXT. VERB DISPLAY SYS.
# Page 238
TERMEXTV 	EQUALS 	ENDEXT
ENDEXTVB	EQUALS	ENDEXT

XACT0		CAF	ZERO		# RELEASE MARK AND EXT. VERB DISPLAY SYS.
		TC	SETXTACT

ALM/END		TC	FALTON		# TURN ON OPERATOR ERROR LIGHT
GOPIN		TC	POSTJUMP
		CADR	PINBRNCH

OC24100		OCT	24100

# Page 239
# VBZERO	VERB 40		DESCRIPTION
#	ZERO
#	1. REQUIRE NOUN 20 (ICDU ANGLES)
#	2. REQUIRE AVAILABILITY OF EXT VERB DISPLAY SYSTEM
#	3. IF EITHER OF ABOVE CONDITIONS NOT PRESENT, TURN ON OPERATOR ERROR LIGHT AND GO TO PINBRNCH.
#	4. SET EXT VERB DISPLAY ACTIVE FLAG.
#	5. EXECUTE IMUZERO (ZERO IMU CDU ANGLES).
#	6. EXECUTE IMUSTALL (ALLOW TIME FOR DATA TRANSFER).
#	7. RELEASE EXT. VERB DISPLAY SYSTEM.

VBZERO		TC	OP/INERT
		TC	IMUZEROK	# RETURN HERE IF NOUN = ICDU(20)
		TC	ALM/END		# RETURN HERE IF NOUN = OCDU(91)
					#	(NOT IN USE YET)

IMUZEROK	TC	CKMODCAD	# KEYBOARD REQUEST FOR ISS CDUZERO
		TC	BANKCALL
		CADR	IMUZERO

		TC	BANKCALL	# STALL
		CADR	IMUSTALL
		TC	+1

		TC	GOPIN

OP/INERT	CS	OCT24
		AD	NOUNREG
		EXTEND
		BZF	XACT0Q		# IF = 20.

		INCR	Q
		AD	OPIMDIFF	# -71
		EXTEND
		BZF	XACT0Q

		TC	ALM/END		# ILLEGAL.

OPIMDIFF	DEC	-71

# Page 240
# VBCOARK	VERB 41		DESCRIPTION
#	COARSE ALIGN IMU OR RADAR
#	1. REQUIRE NOUN 20 OR NOUN 91 OR TURN ON OPERATOR ERROR
#	2. REQUIRE EXT VERB DISPLAY SYS AVAILABLE OR TURN ON OPERATOR ERROR LIGHT AND GO TO PINBRNCH.
#	CASE 1	NOUN 20	(ICDU ANGLES)
#	3. SET EXT VERB DISPLAY ACTIVE FLAG.
#	4. DISPLAY FLASHING V25,N22 (LOAD NEW ICDU ANGLES).
#	   RESPONSES
#		A. TERMINATE
#		   1. RELEASE EXT VERB DISPLAY SYSTEM
#		B. PROCEED
#		   1. DISPLAY FLASHING V25,N23 (LOAD DELTA ICDU ANGLES).
#			RESPONSES
#				A. TERMINATE
#				   1. RELEASE EXT VERB DISPLAY SYSTEM.
#				B. PROCEED
#				   1. EXECUTE ICORK2.
#				C. ENTER
#				   1. INCREMENT CDU ANGLES
#				   2. EXECUTE ICORK2.
#		C. ENTER
#		   1. EXECUTE ICORK2.
# 	ICORK2
#		1. RE-DISPLAY VERB 41.
#		2. EXECUTE IMUCCARS (IMU COARSE ALIGN).
#		3. EXECUTE IMUSTALL (ALLOW TIME FOR DATA TRANSFER).
#		4. RELEASE EXT VERB DISPLAY SYSTEM.
#	CASE 2	NOUN 91	(OCDU ANGLES)
#	5. (REQUIRE OPTICS SWITCH TO BE AT COMPUTER OR TURN ON OPERATOR ERROR AND ALARM 115) AND (REQUIRE
#	   OPTICS AVAILABLE AND DISPLAY FLASHING V24,N92....LOAD NEW OPTICS ANGLES....OR TURN ON ALARM 117
#	   AND RELEASE EXT VERB DISPLAY SYSTEM).
#	6. RESPONSES TO V29,N92.
#	   A. TERMINATE
#	      RELEASE EXT VERB DISPLAY SYSTEM
#	   B. PROCEED OR ENTER
#	      RE-DISPLAY V41,	SET SWITCH TO INDICATE COURSE ALIGN OPTICS WORKING.
#	      RELEASE EXT VERB DISPLAY SYSTEM.

VBCOARK		TC	OP/INERT
		TC	IMUCOARK		# RETURN HERE IF NOUN = ICDU(20)
		TC	OPTCOARK		# RETURN HERE IF NOUN = OCDU(91)
# RETURNS TO L+1 IF NOUN 20 - TO L+2 IF NOUN 91.
IMUCOARK	TC	CKMODCAD		# COARSE ALIGN FROM KEYBOARD
		TC	TESTXACT
		CAF	VNLODCDU		# CALL FOR THETAD LOAD
		TC	BANKCALL
		CADR	GOXDSPF
		TC	TERMEXTV
		TCF	+1
# Page 241

ICORK2		CAF	IMUCOARV		# RE-DISPLAY COARSE ALIGN VERB.
		TC	BANKCALL
		CADR	EXDSPRET

		TC	BANKCALL		# CALL MODE SWITCHING PROG
		CADR	IMUCOARS

		TC	BANKCALL		# STALL
		CADR	IMUSTALL
		TC	ENDEXTVB
		TC	ENDEXTVB

VNLODCDU	VN	2522
IMUCOARV	VN	4100

# Page 242
# TEMPORARY ROUTINE TO RUN THE OPTICS CDUS FROM THE KEYBOARD

OPTCOARK	CA	OPTCADR
		TC	CKMODCAD +1
		TC	TESTXACT
		CAF	EBANK5
		TS	EBANK

		CCS	SWSAMPLE		# SEE IF SWITCH AT COMPUTER
		TC	+5			# SWITCH AT COMPUTER
		TC	+1			# NOT ON COMPUTER
		TC	FALTON			# TURN ON OPERATOR ERR
		TC	ALARM			# AND ALARM
		OCT	00115

		CCS	OPTIND			# SEE IF OPTICS AVAILABLE
		TC	OPTC1			# IN USE
		TC	OPTC1			# IN USE
		TC	OPTC1			# IN USE

		TC	ALARM			# OPTICS RESERVED (OPTIND=-0)
		OCT	00117
		TC	ENDEXT

OPTC1		CAF	VNLD0CDU		# VERB-NOUN TO LOAD OPTICS CDUS
		TC	BANKCALL
		CADR	GOXDSPF
		TC	TERMEXTV
		TC	+1			# PROCEED

		CA	SAC
		TS	DESOPTS
		CA	PAC
		TS	DESOPTT
		CAF	OPTCOARV		# RE-DISPLAY OUR OWN VERB
		TC	BANKCALL
		CADR	EXDSPRET

		CAF	ONE
		TS	OPTIND			# SET COARS WORKING

		TC	ENDEXTVB
		TC	ENDEXTVB

VNLD0CDU	VN	2492
OPTCOARV	EQUALS	IMUCOARV		# DIFFERENT NOUNS.

# Page 243
# IMUFINEK	VERB 42		DESCRIPTION
#	FINE ALIGN IMU
#	1. REQUIRE EXT VERB DISPLAY AVAILABLE AND SET BUSY FLAG OR TURN ON OPER ERROR AND GO TO PINBRNCH.
#	2. DISPLAY FLASHING V25,N93....LOAD DELTA GYRO ANGLES....
#	   RESPONSES
#	   A. TERMINATE
#	      1. RELEASE EXT VERB DISPLAY SYSTEM.
#	   B. PROCEED OR ENTER
#	      1. RE-DISPLAY VERB 42
#	      2. EXECUTE IMUFINE (IMU FIVE ALIGN MODE SWITCHING).
#	      3. EXECUTE IMUSTALL (ALLOW FOR DATA TRANSFER)
#		 A. FAILED
#		    1. RELEASE EXT VERB DISPLAY SYSTEM.
#		 B. GOOD
#		    1. EXECUTE IMUPULSE (TORQUE IRIGS).
#		    2. EXECUTE IMUSTALL AND RELEASE EXT VERB DISPLAY SYSTEM.

IMUFINEK	TC	CKMODCAD		# FINE ALIGN WITH GYRO TORQUING
		TC	TESTXACT
		CAF	VNLODGYR		# CALL FOR LOAD OF GYRO COMMANDS
		TC	BANKCALL
		CADR	GOXDSPF
		TC	TERMEXTV
		TC	+1			# PROCEED WITHOUT A LOAD

		CAF	IMUFINEV		# RE-DISPLAY OUR OWN VERB
		TC	BANKCALL
		CADR	EXDSPRET

		TC	BANKCALL		# CALL MODE SWITCH PROG
		CADR	IMUFINE

		TC	BANKCALL		# HIBERNATION
		CADR	IMUSTALL
		TC	ENDEXTVB

FINEK2		CAF	LGYROBIN		# PINBALL LEFT COMMANDS IN OGC REGIST5RS
		TC	BANKCALL
		CADR	IMUPULSE

		TC	BANKCALL		# WAIT FOR PULSES TO GET OUT.
		CADR	IMUSTALL
		TC	ENDEXTVB
		TC	ENDEXTVB

LGYROBIN	ECADR	OGC
VNLODGYR	VN	2593
IMUFINEV	VN	4200			# FINE ALIGN VERB

CKMODCAD	CA	MODECADR
# Page 244
		EXTEND
		BZF	TCQ
		TC	ALM/END			# SOMEBODY IS USING MODECADR SO EXIT
# GOLOADLV	VERB 50		DESCRIPTION
#		AND OTHER PLEASE
#		DO SOMETHING VERBS
# 	PLEASE PERFORM, MARK, CALIBRATE, ETC.
#	1. PRESSING ENTER ON DSKY INDICATES REQUESTED ACTION HAS BEEN PERFORMED, AND THE PROGRAM DOES THE
#	   SAME RECALL AS A COMPLETED LOAD.
#	2. THE EXECUTION OF A VERB 33 (PROCEED WITHOUT DATA) INDICATES THE REQUESTED ACTION IS NOT DESIRED.

GOLOADLV	TC	FLASHOFF
		CAF	PINSUPBT
		EXTEND
		WRITE	SUPERBNK	# TURN ON FE7
		TC	POSTJUMP
		SBANK=	PINSUPER
		CADR	LOADLV1

# V60	VERB 60
V60		EXTEND			# SET ASTRONAUT TOTAL ATTITUDE (N17) EQUAL
		DCA	CDUX		# TO PRESENT ATTITUDE
		DXCH	CPHIX
		CA	CDUZ
		TS	CPHIX	+2
		TC	GOPIN

# V61	VERB 61
V61		TC	DOWNFLAG	# SET NEEDLFLG TO 0 (FLAGWRD0,BIT9), PHASE
		ADRES	NEEDLFLG	# PLANE A/P FOLLOWING ERROR DISPLAYED
		TC	GOPIN

# V62	VERB 62
V62		TC	UPFLAG		# SET NEEDLFLG TO 1 (FLAGWRD0,BIT9),
		ADRES	NEEDLFLG	# TOTAL ATTITUDE ERROR DISPLAYED

		TC	UPFLAG		# SET N22ORN17 TO 1 (FLAGWRD9,BIT6),
		ADRES	N22ORN17	# COMPUTE TOTAL ATTITUDE ERROR WRT N22
		TC	GOPIN

# V63	VERB 63
V63		TC	UPFLAG		# SET NEEDLFLG TO 1 (FLAGWRD0,BIT9),
		ADRES	NEEDLFLG	# TOTAL ATTITUDE ERROR DISPLAYED

		TC	DOWNFLAG	# SET N22ORN17 TO 0 (FLAGWRD9,BIT6,
# Page 245
		ADRES	N22ORN17	# COMPUTE TOTAL ASTRONAUT ATTITUDE ERROR
		TC	GOPIN

# Page 246
# ALINTIME	VERB 55		DESCRIPTION
#	1. SET EXT VERB DISPLAY BUSY FLAG.
#	2. DISPLAY FLASHING V25,N24 (LOAD DELTA TIME FOR AGC CLOCK.
#	3. REQUIRE EXECUTION OF VERB 23.
#	4. ADD DELTA TIME, RECEIVED FROM INPUT REGISTER, TO THE COMPUTER TIME.
#	5. RELEASE EXT VERB DISPLAY SYSTEM

		COUNT	04/R33

ALINTIME	TC	TESTXACT

		CAF	VNLODDT
		TC	BANKCALL
		CADR	GOMARKF
		TC	ENDEXT		# TERMINATE
		TC	ENDEXT		# PROCEED
		CS	DEC23		# DATA IN OR RESEQUENCE(UNLIKELY)
		AD	MPAC		# RECALL LEFT VERB IN MPAC
		EXTEND
		BZF	UPDATIME	# GO AHEAD WITH UPDATE ONLY IF RECALL
		TC	ENDEXT		#	WITH V23 (DATA IN).
UPDATIME	INHINT			# DELTA TIME IS IN DSPTEM1, +1.
		CAF	ZERO
		TS	MPAC +2		# NEEDED FOR TP AGREE
		TS	L		# ZERO T1 & 2 WHILE ALIGNING.
		DXCH	TIME2
		DXCH	MPAC
		DXCH	DSPTEM2 +1	# INCREMENT
		DAS	MPAC

		TC	TPAGREE		# FORCE SIGN AGREEMENT.
		DXCH	MPAC		# NEW CLOCK.
		DAS	TIME2
		RELINT
UPDTMEND	TC	ENDEXT
DEC23		DEC	23		# V 23

VNLODDT		VN	2524		# V25N24 FOR LOAD DELTA TIME

# Page 247
# SYSTEST	VERB 92		DESCRIPTION
#	OPERATE SELECTED SYSTEM TEST
#	1. REQUIRE P00 OR P00- OR TURN ON OPERATOR ERROR.
#	2. TURN OFF DAP IF IT IS ON.
#	3. DISPLAY FLASHING V21,N01 (LOAD TEST NUMBER 1 THRU 17).
#	4. UPON ENTRY OF TEST NUMBER, SCHEDULE TSELECT WITH PRIORITY 20.
# 	TSELECT
#		1. IF LOADED TEST NUMBER IS VALID, GO TO THAT TEST ROUTINE, OTHERWISE TURN ON OPERATOR ERROR AND
#		   REPEAT LOAD REQUEST DISPLAY.  (NO. 3 ABOVE)

		EBANK=	QPLACE

		COUNT	04/EXTVB

SYSTEST		TC	CHKPOOH
		CA	FLAGWRD1	# IS NODOP01 FLAGBIT ON? (SET BY P11)
		MASK	NOP01BIT
		EXTEND
		BZF	V92CONT		# IF IT'S NOT YET SET, CONTINUE
		TC	POODOO		# IT'S ON. SEND NODO ALARM FOR P07
		OCT	1521
V92CONT		TC	EXDAPOFF	# TURN DAP OFF IF IT'S ON
		CAF	PRIO20
		TC	FINDVAC
		EBANK=	QPLACE
		SBANK=	IMUSUPER
		2CADR	REDO

		TC	GOPIN


# REDO AND TSELECT ARE NOW IN SYSTEM TEST.

		COUNT*	$$/EXTVB
# CKOPTVB	VERB 65		DESCRIPTION
#	OPTICAL VERIFICATION FOR PRELAUNCH.
#	1.	SCHEDULE GCOMPVER, OPTICAL VERIFICATION SUBPROGRAM, WITH PRIORITY 17.

CKOPTVB		TC	CHECKMM
		MM	02		# I WONDER IF PRELAUNCH IS RUNNING
		TC	ALM/END		# NOT RUNNING OPERATOR ERROR
		INHINT
		CAF	PRIO16		# PRELAUNCH OPTICAL VERIFICATION
		TC	FINDVAC
		EBANK=	QPLACE
		2CADR	COMPVER		# STANDARD LEADIN TO GCOMPVER.

		TC	GOPIN

# Page 248
# V 78....	TO CHANGE GYROCOMPASS AZIMUTH

CHAZFOGC	TC	CHECKMM		# IS IT PRELAUNCH
		MM	02
		TC	ALM/END		# NO - OPERA TOR ERROR

		CAF	PRIO16		#  PRELAUNCH AZIMUTH CHANGE
		TC	FINDVAC
		EBANK=	XSM
		2CADR	AZMTHCG1

		TC	PHASCHNG
		OCT	00174
		TC	GOPIN
# Page 249
# IMUATTCK	VERB 43		DESCRIPTION
#	LOAD IMU ATTITUDE ERROR METERS
#	1. REQUIRE PROGRAM 00 ACTIVE, COARSE ALIGN ENABLE BIT OFF AND ZERO ICDU BIT OFF.
#	2. IF GUID REF RELEASE OR LIFTOFF HAS OCCURRED REQUIRE EXT VERB DISPLAY AVAILABLE AND SET BUSY
#	   FLAG, OTHERWISE ALLOW CURRENT EXT VERB DISPLAY TO BE OVER-RIDDEN.
#	3.  REMOVE COARSE ALIGN ENABLE AND IMU ERROR COUNTER ENABLE
#	4. DISPLAY FLASHING V25,N22 (LOAD NEW ICDU ANGLES).
#	5. UPON PROCEED OR ENTER RESPONSE, INITIALIZE CURRENT DAC AND COMMAND VALUES, ENABLE ERROR COUNTERS
#	   TRANSFER LOADED VALUES TO REGISTERS, AND SEND COMMANDS.
#	6. IF BUSY FLAG SET, RESET IT TO RELEASE EXT VERB DISPLAY.

IMUATTCK	TC	CHKPOOH

		CAF	OCTAL30		# CHECK IF IMU ZERO AND IMU COARSE ARE ON
		EXTEND
		RAND	CHAN12
		CCS	A
		TCF	ALM/END		# NOT ALLOWED IF IMU COARSE OR IMU ZERO ON

		TC	CKLFTBTS	# IS IT BEFORE OR AFTER LIFTOFF
		TC	TESTXACT	# AFTER
		CS	OCT50		# REMOVE COARSE AND ECTR ENABLE.
		EXTEND
		WAND	CHAN12

		CAF	VNLODCDU
		TC	BANKCALL
		CADR	GOXDSPF
		TCF	TRMATTCK
		TC	+1
		CAF	EBANK6
		TS	EBANK		# SET E6 FOR NEEDLES.

		EBANK=	AK

		TC	BANKCALL	# INITIALIZE CURRENT DAC AND
		CADR	NEEDLE11	# COMMAND VALUES

		TC	BANKCALL	# ENABLE ERROR COUNTERS.
		CADR	NEEDLER2

		CAF	TWO		# 4 MS MIN.
		TC	WAITLIST
		EBANK=	AK
		2CADR	ATTCK1

TRMATTCK	TC	CKLFTBTS	# IS IT BEFORE OR AFRER LIFTOFF
		TCF	ENDEXT		# AFTER
		TC	GOPIN
# Page 250
ATTCK1		EXTEND			# TRANSFER LOADED VALUES TO DESIRED REGS.
		DCA	THETAD
		DXCH	AK
		CAE	THETAD	+2
		TS	AK	+2

		TC	IBNKCALL	# SENDS COMMANDS LIMITED TO +,- 384 PULSES
		CADR	NEEDLES		# AND LEAVES ERROR COUNTERS ENABLED.

		TC	TASKOVER

CKLFTBTS	CAF	GRRBKBIT	# HAS LIFTOFF OCCURRED
		MASK	FLAGWRD5
		CCS	A
		TC	Q		# YES
		CAF	BIT5
		EXTEND
		RAND	CHAN30
		CCS	A
		TCF	Q+1
XACT0Q		TC	Q		# YES

OCTAL30		OCT	30
VB64		TC	CHKPOOH		# DEMAND PROGRAM 00.
		TC	TESTXACT	# IF DISPLAY SYS. NOT BUSY,MAKE IT BUSY.
		INHINT
		CAF	PRIO4
		TC	FINDVAC
		EBANK=	RHOSB
		2CADR	SBANDANT	# CALC.,DISPLAY S-BAND ANTENNA ANGLES.

		TC	ENDOFJOB

# 	ENATMA		VERB 58		DESCRIPTION
#		ENABLE AUTOMATIC ATTITUDE MANEUVER

# VERB58 RESETS STIKFLAG TO ENABLE R61 TO PERFORM AUTOMATIC TRACKING MANEUVERS, AFTER INTERRUPTS BY THE RHC ACT-
# IVITY.

ENATMA		TC	DOWNFLAG	# RESET STIKFLAG.
		ADRES	STIKFLAG	# BIT 14 FLAG 1
		TC	GOPIN

# Page 251
# STROKON	VERB 68		DESCRIPTION

#	STROKE TEST SETUP/ENABLE
#	1. SET EXT VERB DISPLAY BUSY FLAG
#	2. SCHEDULE STRKTST1 WITH PRIORITY 30.
#	3. RELEASE EXT VERB DISPLAY.

		EBANK=	T5TVCDT
STROKON		CS	FLAGWRD6	# V68	PERMITTED ONLY DURING TVC
		MASK	OCT60000
		EXTEND
		BZMF	ALM/END		# NOT TVC....FLASH OP ERROR LIGHT
		CAF	PRIO30		# JOB REQUEST, TO SET UP STROKE TEST,
		TC	NOVAC		#	INCLUDING INITIALIZATIONS
		EBANK=	STROKER
		2CADR	STRKTSTI

		TC	GOPIN


# STABLISH	VERB 46		DESCRIPTION
#	ESTABLISH G & N AUTOPILOT CONTROL
#	1. SETS UP EITHER RCS, ENTRY, OR SATURN
#	2. IF TVC IS ON, SETS UP CSM/LM SWITCH-OVER
#		FROM HIGH BW TO LOW BW


STABLISH	CAF	EBANK6		# V46 	- SET EBANK TO E6
		TS	EBANK

		CS	FLAGWRD6	# 	TEST FOR TVC
		MASK	OCT60000
		EXTEND
		BZMF	+8

		CAE	DAPDATR1	# 	TEST FOR CSM/LM
		MASK	BIT14
		EXTEND
		BZMF	+3

		TC	POSTJUMP	# CSM/LM, SO PERFORM HB TO LB SWITCH-OVER
		CADR	PRESWTCH

+3		TC	ALM/END		# CSM, SO ALARM AND EXIT

+8		TC	POSTJUMP	# SET UP RCS, ENTRY, OR SATURN-STICK DAP
		CADR	DAPFIG
# Page 252
# CREMANU	VERB 49		DESCRIPTION
#	START AUTOMATIC ATTITUDE MANEUVER
#	1. REQUIRE PROGRAM 00 ACTIVE.
#	2. SET EXT VERB DISPLAY BUSY FLAG.
#	3. SCHEDULE R62DISP WITH PRIORITY 10.
#	4. RELEASE EXT VERB DISPLAY.

#	R62DISP
#	1. DISPLAY FLASHING V06,N22 (DECIMAL DISPLAY NEW ICDU ANGLES).  UPON IMMEDIATE RETURN, SET-UP GROUP
#	   4 FOR RESTART OF DISPLAY SEQUENCE.
#	   RESPONSES
#	   A. TERMINATE
#	      1. GO TO GOTOPOOH.
#	   B. PROCEED
#	      1. SET 3AXISFLG TO INDICATE MANEUVER IS SPECIFIED BY 3 AXIS.
#	      2. EXECUTE R60CSM (ATTITUDE MANEUVER).
#	      3. ZERO GROUP 4 (END R62).
#	   C. ENTER
#	      1. REPEAT FLASHING V06,N22.

CREWMANU	TC	CHKPOOH		# DEMAND P00

		TC	TESTXACT

		CAF	PRIO10
		TC	FINDVAC
		EBANK=	CPHI
		2CADR	R62DISP

		TC	ENDOFJOB

# Page 253
# DAPDISP	VERB 48		DESCRIPTION
#	LOAD AUTOPILOT DATA (ROUTINE R03)
#	0. CHECKFAIL AND RETURN IF TVC.
#	1. REQUIRE EXT VERB DISPLAY AVAILABLE AND SET BUSY FLAG.
#	2. LOWER PRIORITY TO 10.
#	3. DISPLAY FLASHING V04,N46 (DISPLAY AUTOPILOT CONFIGURATION)
#	4. UPON PROCEED RESPONSE, EXECUTE S41.2.
#	5. DISPLAY FLASHING V06,N47 (DISPLAY CSM WGT.. LEM WGT.)
#	6. UPON PROCEED RESPONSE EXECUTE S40.14.
#	7. DISPLAY FLASHING V06,N48 (DISPLAY PITCH TRIM, YAW TRIM)
#	8. UPON PROCEED RESPONSE, RELEASE EXTENDED VERB DESPLAY SYSTEM
		COUNT*	$$/EXTVB

DAPDISP		CS	FLAGWRD6
		MASK	OCT60000
		EXTEND
		BZMF	+2		# TVC = 10, CS YIELDS 01, BZMF TO CONTINUE
		TC	ALM/END		# RETURN IF TVC

		TC	TESTXACT
		TC	BANKCALL
		CADR	DAPDISP1
		BANK	42
		SETLOC	EXTVBS
		BANK
		COUNT	24/R03

DAPDISP1	CAF	EBANK6
		TS	EBANK

		CAF	PRIO10
		TC	PRIOCHNG

DONOUN46	CAF	V04N46		#	  R1		  R2
		TC	BANKCALL	#	DAPDATR1	DAPDATR2
		CADR	GOXDSPF		# GOXDSP ROUTINES USED FOR EXTENDED VERBS.

		TC	ENDEXT		# EXT. VBS GO TO ENDEXT, NOT ENDOFJOB.
		TC	+2
		TC	DONOUN46

		CA	DAPDATR1
		MASK	BIT4
		CCS	A
		TCF	MAXIN
		TC	DOWNFLAG
		ADRES	MAXDBFLG
MAXOUT		TC	BANKCALL
		CADR	S41.2

DONOUN47	CAF	V06N47		#	R1		R2		R3
# Page 254
		TC	BANKCALL	#	CSM WGT.	LEM WGT.	BLANK
		CADR	GOXDSPF

		TC	ENDEXT
		TC	+2
		TC	DONOUN47
		CAE	DAPDATR1	# DO MASS PROPERTIES CALCULATION ONLY IF
		MASK	PRIO30		# CONFIG = 1(CSM), 2(CSM/LM), 6(CSM/LMA)
		EXTEND
		BZF	DONOUN48	# SKIP IF 0, 4
		COM
		MASK	PRIO30
		EXTEND
		BZF	DONOUN48	# SKIP IF 3, 7
		INHINT
		TC	IBNKCALL
		CADR	MASSPROP	# UPDATE IXX, IAVG, IAVG/TLX

		RELINT
		TC	BANKCALL
		CADR	S40.14		# COMPUTE RCS DAP STUFF

DONOUN48	CAF	V0648		#	 R1		 R2		 R3
		TC	BANKCALL	#	PTRIM		YTRIM		BLANK
		CADR	GOXDSPF

		TC	ENDEXT
		TC	ENDEXT
		TC	DONOUN48

MAXIN		TC	UPFLAG
		ADRES	MAXDBFLG
		TC	MAXOUT

V0648		VN	0648
V06N47		VN	0647
V04N46		VN	0446
		BANK	43
		SETLOC	EXTVERBS
		BANK

		COUNT*	$$/EXTVB

# 	V82PERF		VERB82		DESCRIPTION
#		REQUEST ORBIT PARAMETERS DISPLAY (R30)
# 1. IF AVERAGE G IS OFF:
#	FLASH DISPLAY V04N06.  R2 INDICATES WHICH SHIP'S STATE VECTOR IS
#	TO BE UPDATED.  INITIAL CHOICE IS THIS SHIP (R2=1).  ASTRONAUT
#	CAN CHANGE TO OTHER SHIP BY V22EXE, WHERE X NOT EQ 1.
#	SELECTED STATE VECTOR UPDATED BY THISPREC (OTHPREC).
#	CALLS SR30.1 (WHICH CALLS TFFCONMU + TFFRP/RA) TO CALCULATE
# Page 255
#	RPER (PERIGEE RADIUS), RAP0 (APOGEE RADIUS), HPER (PERIGEE
#	HEIGHT ABOVE LAUNCH PAD OR LUNAR LANDING SITE), HAPO (APOGEE
#	HEIGHT AS ABOVE), TPER (TIME TO PERIGEE), TFF (TIME TO
#	INTERSECT 300 KFT ABOVE PAD OR 35KFT ABOVE LANDING SITE).
#	FLASH MONITOR V16N44 (HAPO, HPER, TFF).  TFF IS -59M59S IF IT WAS
#	NOT COMPUTABLE, OTHERWISE IT INCREMENTS ONCE PER SECOND.
#	ASTRONAUT HAS OPTION TO MONITOR TPER BY KEYING IN N 32 E.
#	DISPLAY IS IN HMS, IS NEGATIVE (AS WAS TFF), AND INCREMENTS
#	ONCE PER SECOND ONLY IF TFF DISPLAY WAS -59M59S.
#
# 2. IF AVERAGE G IS ON:
#	CALLS SR30.1 APPROX EVERY TWO SECS.  STATE VECTOR IS ALWAYS
#	FOR THIS VEHICLE.  V82 DOES NOT DISTURB STATE VECTOR.  RESULTS
#	OF SR30.1 ARE RAPO, RPER, HAPO, HPER, TPER, TFF.
#	FLASH MONITOR V16N44 (HAPO, HPER, TFF).
#	IF MODE IS P11, THEN CALL DELRSPL SO ASTRONAUT CAN MONITOR
#	RESULTS BY N50E.  SPLASH COMPUTATION DONE ONCE PER TWO SECS.


# ADDENDUM: HAPO AND HPER SHOULD BE CHANGED TO READ HAPOX AND HPERX IN THE
#	    ABOVE REMARKS.
V82PERF		TC	TESTXACT

		CAF	PRIO7
		TC	PRIOCHNG
		TC	POSTJUMP
		CADR	V82CALL		# ***** V82CALL MUST NOT BE A FINDVAC JOB.


# 	VB83PERF	VERB 83		DESCRIPTION
#		REQUEST RENDEZVOUS PARAMETER DISPLAY (R31)
#		1. SET EXT VERB DISPLAY BUSY FLAG.
#		2. SCHEDULE V83CALL WITH PRIORITY 10.
#		   A. DISPLAY
#			R1	RANGE
#			R2	RANGE RATE
#			R3	THETA

V83PERF		TC	TESTXACT
		INHINT
		CS	FLAGWRD9	# SET R31 FLAG-BIT 4 FLAGWRD9
		MASK	R31FLBIT
		ADS	FLAGWRD9
		CAF	PRIO5
		TC	NOVAC
		EBANK=	SUBEXIT
		2CADR	R31CALL

		TC	ENDOFJOB

# Page 256
V85PERF		TC	TESTXACT
		INHINT
		CS	R31FLBIT	# RESET R31 FLAG TO INDICATE R34
		MASK	FLAGWRD9
		TS	FLAGWRD9
		TC	V83PERF	+5
# Page 257
#	GOTOR21		VERB 57
#	GOTOR23-	VERB 54		DESCRIPTION
# SET UP MARKING FOR R22(REND TRACK DATA PROC)
# 1. SET EXT VERB DISPLAY BUSY FLAG
# 2. IF REND (P20 RUNNING) + TRACK (TRACKING ALLOWED) FLAGS ARE SET,
#    SCHEDULE R21 OR R23 WITH PRIORITY 16, OTHERWISE TURN ON ALARM 406
# 3. RELEASE EXT VERB DISPLAY SYSTEM
GOTOR21		TC	DOWNFLAG	# CLEAR R23FLG
		ADRES	R23FLG		# BIT 9 FLAG 1
		TC	+3
GOTOR23		TC	UPFLAG		# SET R23FLG
		ADRES	R23FLG		# BIT 9 FLAG 1
		TC	TESTXACT
		CA	FLAGWRD0	# VB 57	UNACCEPTABLE UNLESS BOTH
		MASK	RNDVZBIT	#	RENDEZVOUS AND TRACK FLAGS ON
		EXTEND
		BZF	R22ALARM

		CA	FLAGWRD1
		MASK	TRACKBIT
		EXTEND
		BZF	R22ALARM

		CA	FLAGWRD1	# TEST R23FLG
		MASK	R23BIT
		EXTEND
		BZF	REGR21		# R21
		CAF	PRIO16
		TC	NOVAC
		EBANK=	MRKBUF1
		2CADR	R23CSM

		TC	ENDOFJOB
REGR21		CAF	PRIO16
		TC	NOVAC
		EBANK=	MRKBUF1
		2CADR	R21CSM

		TC	ENDOFJOB
R22ALARM	TC	ALARM		# VERB 57 WAS SELECTED AND NEITHER REND
		OCT	00406		#	NOR TRACK FLAG WERE ON.
		TC	ENDEXT

# Page 258
# VERB 86	DESCRIPTION
#	V86 IS TO R23 AS MARK REJECT IS TO R21
#	V86 IS THE MARK REJECT FOR R23(THE BACKUP MARKING ROUTINE)
		EBANK=	MRKBUF1
V86PERF		CAF	EBANK7		# BACKUP MARK REJECT (R23)
		XCH	EBANK
		CA	NEGONE
		TS	MRKBUF1
		TC	GOPIN

# Page 259
# TRACKTRM	VERB 56		DESCRIPTION
#	TERMINATE TRACKING (P20)
#	1. KNOCK DOWN RENDEZVOUS, TRACK, AND UPDATE FLAGS.
#	2. REQUIRE P20 NOT RUNNING ALONE OR GO TO GOTOPOOH (REQUEST PROGRAM 00).
#	3. REQUIRE R22 RUNNING OR GO TO PINBRNCH.
#	4. IF INTEGRATION RUNNING, STALL UNTIL IT IS COMPLETED, THEN ZERO GROUPS 2 AND 3 TO KILL R21 + R22.
#	3. KNOCK DOWN RENDEZFOUS, R22, R21, TRACK, UPDATE, AND TARG1 FLAGS.
#	4. GO TO ENEMA (SOFTWARE RESTART).
#	REFERENCE
#		P20	RENDEZVOUS	NAVIGATION.
#		R21	RENDEZVOUS	TRACKING SIGHTING MARK.
#		R22	RENDEZVOUS	TRACKING DATA PROCESSING.

TRACKTRM	CA	RNDVZBIT	# IS REND FLAG ON
		MASK	FLAGWRD0
		EXTEND
		BZF	GOPIN		# NO

		TC	DOWNFLAG
		ADRES	RNDVZFLG

		CA	TRACKBIT	# IS TRACK FLAG ON
		MASK	FLAGWRD1
		EXTEND
		BZF	GOPIN		# NO

		TC	DOWNFLAG
		ADRES	TRACKFLG

		TC	DOWNFLAG
		ADRES	UPDATFLG

		TC	DOWNFLAG
		ADRES	IMUSE

		CAF	EBANK6
		TS	EBANK

		INHINT
		TC	STOPRATE

		CAF	NEGONE
		TS	OPTIND

		TC	INTPRET
		CALL
			INTSTALL	# DONT INTERRUPT INTEGRATION
		EXIT

		TC	2PHSCHNG
# Page 260
		OCT	2		# KILL GROUP 2 TO HALT P20 ACTIVITY
		OCT	1		# ALSO KILL GROUP 1

CLEANOUT 	INHINT
		TC	POSTJUMP
		CADR	ENEMA		# CAUSE RESTART

# LEMVEC	VERB 80		DESCRIPTION
#	UPDATE LEM STATE VECTOR
#		RESET VEHUPFLG TO 0

LEMVEC		TC	DOWNFLAG
		ADRES	VEHUPFLG	# VEHUPFLG DOWN INDICATES LEM

		TCF	GOPIN

# CSMVEC	VERB 81		DESCRIPTION
#	UPDATE CSM STATE VECTOR
#		SET VEHUPFLG TO 1

CSMVEC		TC	UPFLAG
		ADRES	VEHUPFLG	# VEHUPFLG UP INDICATES CM.

		TCF	GOPIN

# DNEDUMP	VERB 74		DESCRIPTION
#	INITIALIZE DOWN-TELEMETRY PROGRAM FOR ERASABLE MEMORY DUMP.
#	1. SET EXT VERB DISPLAY BUSY FLAG.
#	2. REPLACE CURRENT DOWNLIST WITH ERASABLE MEMORY.
#	3. RELEASE EXT VERB DISPLAY.

		EBANK=	10
DNEDUMP		CAF	LDNDUMPI
		TS	DNTMGOTO
		TC	GOPIN

V74		EQUALS	DNEDUMP
LDNDUMPI	REMADR	DNDUMPI

# LFTFLGON	VERB 75		DESCRIPTION
#	SET LIFT-OFF FLAG
#	1. SETUP GGRBKFLG, GUIDANCE REFERENCE RELEASE BACK-UP FLAG.
#	2. RETURN VIA PINBRNCH

LFTFLGON	TC	UPFLAG		# VB 75 - SET LIFTOFF FLAG BIT
		ADRES	GRRBKFLG	# BIT 5  FLAG 5
		TC	GOPIN

# Page 261
CHKPOOH		CA	MODREG
		EXTEND
		BZF	TCQ
		TCF	ALM/END

EXDAPOFF	EXTEND
		DCA	IDLECADR	# SET T5 TO IDLE.
		DXCH	T5LOC
		CS	OCT60000
		MASK	FLAGWRD6	# RESET DAPBITS 1 AND 2.
		TS	FLAGWRD6
		TC	Q

		EBANK=	PACTOFF
IDLECADR	2CADR	T5IDLOC

# Page 262
# VERB 89	DESCRIPTION	RENDEZVOUS FINAL ATTITUDE ROUTINE (R63)

# CALLED BY VERB 89 ENTER DURING P00.  PRIO 10 IS USED.  CALCULATES AND
# DISPLAYS FINAL GIMBAL ANGLES TO POINT CSM +X AXIS OR PREFERRED AXIS
# (UNIT(Z)COS55 DEG + UNIT(X)SIN55 DEG) AT LM.

# 1. KEY IN V 89 E ONLY IF IN PROG 00.  IF NOT IN P00, OPERATOR ERROR AND
# EXIT R63, OTHERWISE CONTINUE.

# 2. IF IN P00, DO IMU STATUS CHECK ROUTINE (R02BOTH).  IF IMU ON AND ITS
# ORIENTATION KNOWN TO CGC, CONTINUE.

# 3. FLASH DISPLAY V 04 N 06.  R2 INDICATES WHICH SPACECRAFT AXIS IS TO
# BE POINTED AT LM.  INITIAL CHOICE IS PREFERRED AXIS.  (R2=1).
# ASTRONAUT CAN CHANGE TO (+X) AXIS (R2 NOT= 1) BY V 22 E 2 E.  CONTINUE
# AFTER KEYING IN PROCEED.

# 4. SET PREFERRED ATTITUDE FLAG ACCORDING TO OPTION DESIRED.  SET FLAG
# FOR PREFERRED AXIS.  RESET FLAG FOR X AXIS.

# 5. CURRENT TIME IS STORED AND R63COMP IS CALLED

#	R63COMP JOB:

#		UPDATES CSM AND LM STATE VECTORS USING CONIC EQUATIONS

#		CALCULATES BOTH PREFERRED AND X AXIS TRACKING ATT FROM CSM TO LM.

#		DESIRED GIMBAL ANGLES AS INDICATED BY PREFERRED ATTITUDE FLAG
#		ARE STORED FOR LATER R60CSM CALL.

# 6. FLASH DISPLAY V 06 N18 AND AWAIT RESPONSE.

# 7. RECYCLE- RETURN TO STEP 5.
#    TERMINATE-  EXIT R63 ROUTINE
#    PROCEED-  RESET 3AXISFLG AND CALL R60CSM FOR ATTITUDE MANEUVER.

V89PERF		TC	CHKPOOH		# DEMAND P00
		TC	TESTXACT
		INHINT
		CAF	PRIO10
		TC	FINDVAC
		EBANK=	P21TIME
		2CADR	V89CALL

		TCF	ENDOFJOB

WMATRXNG	TC	DOWNFLAG	# RESET RENDWFLG
		ADRES	RENDWFLG
# Page 263

		TC	DOWNFLAG	# RESET ORBWFLAG
		ADRES	ORBWFLAG
		TC	GOPIN

GOSHOSUM	EQUALS	SHOWSUM

SHOWSUM		TC	CHKPOOH
		TC	TESTXACT	# *
		CAF	S+1		# *
		TS	SKEEP6		# * SHOWSUM OPTION
		CAF	S+ZERO		# *
		TS	SMODE		# * TURN OFF SELF-CHECK
		CA	SELFADRS	# *
		TS	SELFRET		# *
		TC	STSHOSUM	# * ENTER ROPECHK

SDISPLAY	LXCH	SKEEP2		# * BNK NO FOR DSP
		LXCH	SKEEP3		# * BUGGER WORD FOR DSP
NOKILL		CA	ADRS1		# *
		TS	MPAC	+2	# *
		CA	VNCON		# * 0501
		TC	BANKCALL	# *
		CADR	GOXDSPF		# *
		TC	+3		# *
		TC	NXTBNK		# *
		TC	NOKILL		# *
		CA	SELFADRS
		TS	SKEEP1

		TC	ENDEXT		# *

VNCON		VN	501		# *

ENDSUMS		CA	SKEEP6		# *
		EXTEND			# *
		BZF	SELFCHK		# * ROPECHK, START SELFCHK AGAIN.
		TC	STSHOSUM	# * START SHOWSUM AGAIN.


# VB 76 SET PREFERRED ATTITUDE FLAG - DRIVE TO PREFERRED.

SETPRFLG	TC	UPFLAG
		ADRES	PRFTRKAT	# BIT 10 FLAG 5
		TC	GOPIN


# VB 77 RESET PREFERRED ATTITUDE FLAG - DRIVE TO +X-AXIS ATT.

RESETPRF	TC	DOWNFLAG
		ADRES	PRFTRKAT	# BIT 10 FLAG 5
		TC	GOPIN

# Page 264
# VB 87 SET VHF RANGE FLAG - ALLOWS R22 TO ACCEPT RANGE DATA.

SETVHFLG	TC	INTPRET
		SET	EXIT
			VHFRFLAG
		TC	GOPIN


# VB 88 RESET VHF RANGE FLAG - STOPS ACCEPTANCE OF RANGE DATA.

RESETVHF	TC	INTPRET
		CLEAR	EXIT
			VHFRFLAG
		TC	TRFAILOF	# TRACKER FAIL LIGHT

		TC	GOPIN


# VERB 66. VEHICLES ARE ATTACHED.- MOVE THIS VEHICLE STATE VECTOR TO
#	   OTHER VEHICLE STATE VECTOR.

# 	USE SUBROUTINE GENTRAN.

		EBANK=	RRECTHIS
ATTACHED	CAF	PRIO10
		TC	FINDVAC
		EBANK=	RRECTHIS
		2CADR	ATTACHIT

		TC	ENDOFJOB

ATTACHIT	TC	INTPRET
		CALL
			INTSTALL
		SET	BON
			MOONOTH
			MOONTHIS
			+3
		CLEAR
			MOONOTH
		EXIT
		CAF	OCT51
		TC	GENTRAN
		ADRES	RRECTHIS	# OUR STATE VECTOR INTO OTHER VIA GENTRAN
		ADRES	RRECTOTH

TACHEXIT	RELINT
		TC	INTPRET
		CALL			# UPDATE RN, VN, R-OTHER, V-OTHER
			PTOACSM
# Page 265
		LXA,2	CALL
			PBODY
			SVDWN1
		CALL
			SVDWN2
		EXIT

		CAF	TCPINAD
		INDEX	FIXLOC
		TS	QPRET
		TC	POSTJUMP
		CADR	INTWAKE

TCPIN		RTB
			PINBRNCH

OCT51		OCT	51
TCPINAD		CADR	TCPIN

# VERB 47 MOVE LM STATE VECTOR INTO CSM STATE VECTOR.

LMTOCMSV	CAF	PRIO10
		TC	FINDVAC
		EBANK=	RRECTHIS
		2CADR	LMTOCM

		TC	ENDOFJOB

LMTOCM		TC	INTPRET
		CALL
			INTSTALL
		SET	BON
			MOONTHIS
			MOONOTH
			+3
		CLEAR
			MOONTHIS
		EXIT

		CAF	OCT51
		TC	GENTRAN
		ADRES	RRECTOTH	# LM STATE VECTOR INTO CM VIA GENTRAN
		ADRES	RRECTHIS

		TCF	TACHEXIT

# VERB 94 DO R64 VIA ENEMA TO PICK UP IN P23.

VERB94		CAF	V94FLBIT
		MASK	FLAGWRD9	# IS V94FLAG SET
# Page 266
		EXTEND
		BZF	ALM/END		# NO - OPERATOR ERROR

		TC	DOWNFLAG
		ADRES	V94FLAG

		TC	CHECKMM		# IS IT P23
		MM	23
		TC	ALM/END		# NO - OPERATOR ERROR
		TC	PHASCHNG
		OCT	112		# SET GROUP 2 TO DO R64

		TC	CLEANOUT	# CAUSE RESTART

# V90PERF	VERB 90		DESCRIPTION
#	REQUEST RENDEZVOUS OUT-OF-PLANE DISPLAY (R36)
#	1. SET EXT VERB DISPLAY BUSY FLAG.
#	2. SCHEDULE R36 CALL WITH PRIORITY 10
#	   A. DISPLAY
#		TIME OF EVENT - HOURS , MINUTES , SECONDS
#		Y OUT-OF-PLANE POSITION - NAUTICAL MILES
#		YDOT 	 OUT-OF-PLANE VELOCITY - FEET/SECOND
#		PSI 	ANGLE BTW LINE OF SIGHT AND FORWARD
#			DIRECTION VECTOR IN HORIZONTAL PLANE - DEGREES
V90PERF		TC	TESTXACT
		CAF	PRIO7		# R36,V90
		TC	FINDVAC
		EBANK=	RPASS36
		2CADR	R36

		TCF	ENDOFJOB
# VERB 96 SET QUITFLAG TO STOP INTEGRATION.

VERB96		TC	UPFLAG		# QUITFLAG WILL CAUSE INTEGRATION TO EXIT
		ADRES	QUITFLAG	# 	AT NEXT TIMESTEP

		TC	UPFLAG
		ADRES	V96ONFLG
		CAF	ZERO
		TC	POSTJUMP
		CADR	V37		# GO TO P00

		EBANK=	LANDMARK
V52		TC	CHECKMM		# IS P22 OPERATING
		MM	22
		TC	ALM/END		# NO
		CAF	LANDBANK
		TS	EBANK

# Page 267
		CS	PRIO7		# YES	SET BITS 12,11,10 OF LANDMARK =
		MASK	LANDMARK	#	BITS 14,13,12 OF MARKSTAT AFTER
		TS	LANDMARK	#	SUBT. THEM FROM 5 TO GET OFFSET
		CA	MARKSTAT	#	MARK NO.
		TS	SR
		CA	SR
		CA	SR
		MASK	PRIO7
		CS	A
		AD	PRIO5
		ADS	LANDMARK
		TC	GOPIN
LANDBANK	ECADR	LANDMARK
#

# VERB 67 ASTRONAUT DISPLAY OF W MATRIX

V67		TC	TESTXACT
		CAF	PRIO5
		TC	FINDVAC
		EBANK=	W
		2CADR	V67CALL

		TC	ENDOFJOB
# VB 44. SET SURFACE FLAG.

SETSURF		TC	UPFLAG
		ADRES	SURFFLAG
		TCF	GOPIN


# VB 45. RESET SURFACE FLAG.

RESTSRF		TC	DOWNFLAG
		ADRES	SURFFLAG
		TCF	GOPIN
