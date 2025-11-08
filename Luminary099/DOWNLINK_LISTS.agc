# Copyright:	Public domain.
# Filename:	DOWNLINK_LISTS.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	193-205
# Mod history:	2009-05-19 HG	Transcribed from page images.
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

# Page 193
# ============================================================================
# FILE: DOWNLINK_LISTS.agc
# MODULE: Telemetry System
# MISSION PHASE: all phases (launch/earth-orbit/trans-lunar/lunar-orbit/descent/landing/ascent/rendezvous/trans-earth)
#
# TL;DR: Defines telemetry downlink data structure lists that specify which
#        AGC data is transmitted to ground stations via the MSFN (Manned Space
#        Flight Network). Organizes mission data into phase-specific lists
#        (orbital maneuvers, rendezvous, descent, landing) transmitted every
#        2 seconds at 50Hz downlink rate, enabling Mission Control monitoring.
#
# COMMENT-ONLY READERS: This file configures what spacecraft data Mission
#        Control receives during each phase of the lunar mission. Read to
#        understand how ground controllers monitored the Eagle's systems.
# CODE-ALONG READERS: Study the downlink list structure and special op codes
#        (1DNADR-6DNADR, DNPTR, DNCHAN) that define telemetry packet formats
#        and data selection priorities for real-time ground monitoring.
# ============================================================================

		BANK	22
		SETLOC	DOWNTELM
		BANK

		EBANK=	DNTMBUFF

;
; ============================================================================
; TELEMETRY DOWNLINK SYSTEM OVERVIEW
;
; The Lunar Module transmits spacecraft data to Earth every 20 milliseconds
; (50 times per second) through the MSFN ground station network. Each downlink
; "frame" can contain 2 AGC words, allowing 200 words every 2 seconds. This
; file defines which data Mission Control receives during each mission phase.
;
; During Apollo 11's descent on July 20, 1969, these downlink lists enabled
; flight controllers like Steve Bales (GUIDO) to monitor guidance computer
; state and make critical Go/NoGo decisions including the famous call during
; the 1202 program alarm at 102:38:26 mission time.
; ============================================================================
;
# SPECIAL DOWNLINK OP CODES
;
; The following op codes define telemetry packet formats. Each specifies how
; many AGC memory words to transmit and where to find them. The DOWN_TELEMETRY
; PROGRAM (pages 988-997) interprets these op codes at 50Hz interrupt rate.
;
#	OP CODE		ADDRESS (EXAMPLE)	SENDS...		BIT 15		BITS 14-12	BITS 11-0
#	-------		-----------------	--------		------		----------	---------
#	1DNADR		TIME2			(2 AGC WDS)		0		0		ECADR
#	2DNADR		TEPHEM			(4 AGC WDS)		0		1		ECADR
#	3DNADR		VGBODY			(6 AGC WDS)		0		2		ECADR
#	4DNADR		STATE			(8 AGC WDS)		0		3		ECADR
#	5DNADR		UPBUFF			(10 AGC WDS)		0		4		ECADR
#	6DNADR		DSPTAB			(12 AGC WDS)		0		5		ECADR
#	DNCHAN		30			CHANNELS		0		7		CHANNEL
#													ADDRESS
#	DNPTR		NEXTLIST		POINTS TO NEXT		0		6		ADRES
#						LIST
;
; Technical note: ECADR = Erasable Core Address, specifying location in AGC's
; 2K RAM where data resides. ADRES = Address of next list segment to process.
; CHANNEL ADDRESS = I/O channel number for hardware status monitoring.
;
# DOWNLIST FORMAT DEFINITIONS AND RULES --
# 1. END OF A LIST = -XDNADR (X = 1 TO 6), -DNPTR, OR -DNCHAN.
# 2. SNAPSHOT SUBLIST = LIST WHICH STARTS WITH A -1DNADR.
# 3. SNAPSHOT SUBLIST CAN ONLY CONTAIN 1DNADRS.
# 4. TIME2 1DNADR MUST BE LOCATED IN THE CONTROL LIST OF A DOWNLIST.
# 5. ERASABLE DOWN TELEMETRY WORDS SHOULD BE GROUPED IN SEQUENTIAL
#    LOCATIONS AS MUCH AS POSSIBLE TO SAVE STORAGE USED BY DOWNLINK LISTS.
;
; Snapshot sublists capture time-synchronized data groups (e.g., position and
; velocity at same instant) by collecting into DNTMBUFF buffer before transmit.

;
; Downlink list identifiers and default assignments follow. After Fresh Start
; or completion of Update Program (P27), AGC defaults to coast/align downlist.
;
		COUNT*	$$/DLIST
ERASZERO	EQUALS	7
UNKNOWN		EQUALS	ERASZERO
SPARE		EQUALS	ERASZERO			# USE SPARE TO INDICATE AVAILABLE SPACE
LOWIDCOD	OCT	77340				# LOW ID CODE

NOMDNLST	EQUALS	LMCSTADL			# FRESH START AND POST P27 DOWNLIST

AGSLIST		EQUALS	LMAGSIDL

UPDNLIST	EQUALS	LMAGSIDL			# UPDATE PROGRAM (P27) DOWNLIST

# Page 194
;
; ============================================================================
; TRANSITION: From telemetry op code definitions to mission-phase downlists
;
; The following sections define complete downlink lists for each mission phase.
; Each "Control List" is the master list for a specific mission mode, composed
; of reusable "Sub-Lists" that capture related data groups. During Apollo 11,
; the appropriate list was automatically selected when crew or Mission Control
; initiated major mode changes (e.g., P20 Rendezvous Navigation, P63 Landing).
; ============================================================================
;
# LM ORBITAL MANEUVERS LIST
;
; Active during: Orbital coast, rendezvous planning, CSI/CDH/TPI burns
; Mission phases: Post-landing abort ascent, rendezvous with Command Module
; Used by programs: P20-P25 (Rendezvous Navigation), P30-P37 (Orbit Targeting)
;
; This downlist transmitted Eagle's orbital state, planned maneuver parameters,
; and rendezvous targeting data to Mission Control during Collins' wait in
; Columbia. Critical for monitoring July 21, 1969 ascent and rendezvous.
;
# --------------------- CONTROL LIST -------------------------

LMORBMDL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	LMORBM01			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		1DNADR	DELLT4				# DELLT4,+1
		3DNADR	RTARG				# RTARG,+1...+5
		1DNADR	ELEV				# ELEV,+1
		1DNADR	TEVENT				# TEVENT,+1
		6DNADR	REFSMMAT			# REFSMMAT +0...+11D
		1DNADR	TCSI				# TCSI,+1
		3DNADR	DELVEET1			# DELVEET1 +0...+5
		3DNADR	VGTIG				# VGTIG +0...+5
		1DNADR	DNLRVELZ			# DNLRVELZ,DNLRALT
		1DNADR	TPASS4				# TPASS4,+1
		DNPTR	LMORBM02			# COMMON DATA
		1DNADR	TIME2				# TIME2/1
		DNPTR	LMORBM03			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	LMORBM04			# COMMON DATA
		2DNADR	POSTORKU			# POSTORKU,NEGTORKU,POSTORKV,NEGTORKV
		1DNADR	SPARE
		1DNADR	TCDH				# TCDH,+1
		3DNADR	DELVEET2			# DELVEET2 +0...+5
		1DNADR	TTPI				# TTPI,+1
		3DNADR	DELVEET3			# DELVEET3 +0...+5
		1DNADR	DNRRANGE			# DNRRANGE,DNRRDOT
		2DNADR	DNLRVELX			# DNLRVELX,DNLRVELY,DNLRVELZ,DNLRALT
		1DNADR	DIFFALT				# DIFFALT,+1
		1DNADR	LEMMASS				# LEMMASS,CSMMASS
		1DNADR	IMODES30			# IMODES30,IMODES33
		1DNADR	TIG				# TIG,+1
		DNPTR	LMORBM05			# COMMON DATA
		DNPTR	LMORBM06			# COMMON DATA
		1DNADR	SPARE				# FORMERLY PIF
		-1DNADR	TGO				# TGO,+1

# --------------------- SUB-LISTS ---------------------------
;
; COMMENT-ONLY READERS: The following sections are reusable data collection
; sub-lists. The control list above calls these sub-lists to gather specific
; data groupings. Sub-lists starting with -1DNADR are "snapshots" - they
; capture data at a precise instant to ensure all values are synchronized.
;
; CODE-ALONG READERS: These sub-lists are referenced by DNPTR directives in
; the control list. Each sub-list defines a sequence of memory addresses to
; collect. The downlink program processes these sequentially, reading data
; from AGC memory and formatting it for transmission to Mission Control.

; ============================================================================
; LMORBM01: OTHER VEHICLE STATE SNAPSHOT
;
; Captures position (R-OTHER) and velocity (V-OTHER) state vectors of the
; "other vehicle" (Command Module when viewed from LM, or vice versa) at a
; precise instant. This snapshot ensures all six vector components plus
; timestamp are synchronized, preventing motion blur in ground computations.
;
; During Apollo 11 rendezvous after lunar ascent, this data allowed Mission
; Control to compute relative trajectories between Eagle and Columbia.
; ============================================================================

LMORBM01	-1DNADR	R-OTHER +2			# R-OTHER +2,+3		SNAPSHOT
		1DNADR	R-OTHER +4			# R-OTHER +4,+5
		1DNADR	V-OTHER				# V-OTHER,+1
		1DNADR	V-OTHER +2			# V-OTHER +2,+3
		1DNADR	V-OTHER +4			# V-OTHER +4,+5
		1DNADR	T-OTHER				# T-OTHER,+1
		-1DNADR	R-OTHER				# R-OTHER +0,+1

; ============================================================================
; LMORBM02: COMMON GUIDANCE AND ATTITUDE DATA
;
; Collects guidance system state and spacecraft attitude information shared
; across multiple mission phases. REDOCTR tracks integration cycle count,
; THETAD contains desired gimbal angles, OMEGAP/Q/R are body rates, and
; CDUX/Y/Z are current IMU gimbal angles.
;
; The STATE vector (8 AGC words, 11D in double-precision) contains critical
; flag words indicating program modes and system states. DSPTAB contains the
; complete DSKY display buffer showing what the crew currently sees.
; ============================================================================

LMORBM02	2DNADR	REDOCTR				# REDOCTR,THETAD,+1,+2	COMMON DATA
# Page 195
		1DNADR	RSBBQ				# RSBBQ,+1
		2DNADR	OMEGAP				# OMEGAP,OMEGAQ,OMEGAR,GARBAGE
		2DNADR	CDUXD				# CDUXD,CDUYD,CDUZD,GARBAGE
		2DNADR	CDUX				# CDUX,CDUY,CDUZ,CDUT
		6DNADR	STATE				# STATE +0...+11D (FLAGWORDS)
		-6DNADR	DSPTAB				# DSPTAB TABLES

; ============================================================================
; LMORBM03: LM NAVIGATION STATE SNAPSHOT
;
; Captures the Lunar Module's own position (RN) and velocity (VN) vectors at
; a precise instant, synchronized with PIPTIME (time from IMU accelerometers).
; This is the AGC's best estimate of where the LM is and how fast it's moving.
;
; During orbital maneuvers, Mission Control used this data to verify the
; navigation state matched predictions and to compute any required corrections.
; The snapshot ensures ground computers see a consistent state vector.
; ============================================================================

LMORBM03	-1DNADR	RN +2				# RN +2,+3		SNAPSHOT
		1DNADR	RN +4				# RN +4,+5
		1DNADR	VN				# VN,+1
		1DNADR	VN +2				# VN +2,+3
		1DNADR	VN +4				# VN +4,+5
		1DNADR	PIPTIME				# PIPTIME,+1
		-1DNADR	RN				# RN,+1

; ============================================================================
; LMORBM04: DESIRED RATES, FAILURE REGISTERS, AND MODE FLAGS
;
; Collects commanded body rates (OMEGAPD/QD/RD) that the autopilot is trying
; to achieve. CADRFLSH contains recent failure addresses for self-check
; diagnostics. FAILREG records hardware failures detected by the AGC.
;
; RADMODES indicates radar operating modes (landing radar vs rendezvous radar).
; DAPBOOLS contains Digital Autopilot control flags. Ground controllers
; monitored these to verify the spacecraft was responding correctly to commands
; and to diagnose any anomalies in attitude control or sensor systems.
; ============================================================================

LMORBM04	2DNADR	OMEGAPD				# OMEGAPD,OMEGAQD,OMEGARD,GARBAGE
		3DNADR	CADRFLSH			# CADRFLSH,+1,+2,FAILREG,+1,+2
		-1DNADR	RADMODES			# RADMODES,DAPBOOLS	COMMON DATA

; ============================================================================
; LMORBM05: ATTITUDE STATE AND RCS CHANNEL DATA
;
; Provides current body rates (OMEGAP/Q/R), desired gimbal angles (CDUXD/YD/ZD),
; actual gimbal angles (CDUX/Y/Z), and commanded autopilot torques (ALPHAQ/R,
; POSTORKP/NEGTORKP). Also reads I/O channels 11-14 and 30-33 which contain
; RCS jet firing commands, engine status, and IMU error signals.
;
; During critical maneuvers, ground controllers watched these values to verify
; the Digital Autopilot was commanding correct thruster firings and that the
; spacecraft attitude was tracking desired orientation.
; ============================================================================

LMORBM05	2DNADR	OMEGAP				# OMEGAP,OMEGAQ,OMEGAR,GARBAGE
		2DNADR	CDUXD				# CDUXD,CDUYD,CDUZD,GARBAGE
		2DNADR	CDUX				# CDUX,CDUY,CDUZ,CDUT
		1DNADR	ALPHAQ				# ALPHAQ,ALPHAR		COMMON DATA
		1DNADR	POSTORKP			# POSTORKP,NEGTORKP
		DNCHAN	11				# CHANNELS 11,12
		DNCHAN	13				# CHANNELS 13,14
		DNCHAN	30				# CHANNELS 30,31
		-DNCHAN	32				# CHANNELS 32,33

; ============================================================================
; LMORBM06: IMU VELOCITY INCREMENT DATA
;
; Transmits PIPTIME (IMU accelerometer time tag) and DELV (velocity increment
; measured by Pulsed Integrating Pendulous Accelerometers). The DELV vector
; accumulates velocity changes sensed since the last reset, providing raw
; inertial acceleration data independent of the navigation state estimate.
;
; Ground controllers used this to independently verify navigation accuracy by
; comparing integrated DELV with computed trajectory predictions.
; ============================================================================

LMORBM06	1DNADR	PIPTIME1			# PIPTIME,+1		COMMON DATA
		-3DNADR	DELV				# DELV +0...+5

; ============================================================================
; TRANSITION: From LM Orbital Maneuvers List to Coast and Alignment List
;
; Having defined the data collection lists for orbital operations (rendezvous,
; station-keeping, transfer burns), the downlink format now shifts to coast
; phase operations. The LMCSTADL list supports long-duration coast periods
; and platform alignment activities where different telemetry priorities apply.
; ============================================================================

# --------------------------------------------------------------------

# Page 196
# LM COAST AND ALIGNMENT DOWNLIST
#
# ---------------------- CONTROL LIST --------------------------------
;
; COMMENT-ONLY READERS: This downlink list supports coast phases and platform
; alignment operations. During long periods between maneuvers, the LM transmits
; different telemetry emphasizing alignment sensors (Alignment Optical Telescope),
; star tracker data, and gyro calibration values rather than rendezvous targeting.
;
; CODE-ALONG READERS: LMCSTADL reuses most sub-lists from LMORBMDL via EQUALS
; directives (LMCSTA01 through LMCSTA05 are aliases). It adds unique sub-lists
; LMCSTA06 (IMU compensation data) and LMCSTA07 (optical tracking data). This
; list is activated during P51/P52/P53 alignment programs and extended coast.

LMCSTADL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	LMCSTA01			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
; Coast and alignment telemetry emphasizes IMU platform data. During P51/P52/P53
; alignment programs (manual/automatic star sighting), ground monitors gyro drift,
; star tracker accuracy, and optical alignment quality.
		1DNADR	AGSK				# AGSK,+1
		1DNADR	TALIGN				# TALIGN,+1 - Alignment time reference
		2DNADR	POSTORKU			# POSTORKU,NEGTORKU,POSTORKV,NEGTORKV
		1DNADR	DNRRANGE			# DNRRANGE,DNRRDOT
		1DNADR	TEVENT				# TEVENT,+1
		6DNADR	REFSMMAT			# REFSMMAT +0...+11D - Reference matrix
		1DNADR	AOTCODE				# AOTCODE,GARBAGE - AOT star code
		3DNADR	RLS				# RLS +0...+5 - LM state vector
		2DNADR	DNLRVELX			# DNLRVELX,DNLRVELY,DNLRVELZ,DNLRALT
; LMCSTA06 sub-list provides IMU compensation parameters (gyro and accelerometer
; correction factors). LMCSTA07 provides optical tracking data from AOT sightings.
; These unique sub-lists distinguish coast/alignment mode from maneuver mode.
		DNPTR	LMCSTA06			# COMMON DATA
		DNPTR	LMCSTA02			# COMMON DATA
		1DNADR	TIME2				# TIME2/1
		DNPTR	LMCSTA03			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	LMCSTA04			# COMMON DATA
		DNPTR	LMCSTA07			# COMMON DATA
		2DNADR	DNLRVELX			# DNLRVELX,DNLRVELY,DNLRVELZ,DNLRALT
; IMU gimbal angles (CDUS) and accelerometer pulses (PIPA) enable ground tracking
; of platform drift during coast. LASTYCMD/LASTXCMD show RCS activity. IMODES
; flags indicate IMU operational mode (coarse align, fine align, gyrocompassing).
		2DNADR	CDUS				# CDUS,PIPAX,PIPAY,PIPAZ
		1DNADR	LASTYCMD			# LASTYCMD,LASTXCMD
		1DNADR	LEMMASS				# LEMMASS,CSMMASS
		1DNADR	IMODES30			# IMODES30,IMODES33 - IMU mode flags
		1DNADR	TIG				# TIG,+1
		DNPTR	LMCSTA05			# COMMON DATA
		-6DNADR	DSPTAB				# DSPTAB +0...+11D TABLE

# ---------------------- SUB-LISTS --------------------------
;
; Most sub-lists reused via EQUALS from LMORBMDL (snapshot buffers, common
; spacecraft data). LMCSTA06 and LMCSTA07 are unique to coast/alignment mode.

LMCSTA01	EQUALS	LMORBM01			# COMMON DOWNLIST DATA
LMCSTA02	EQUALS	LMORBM02			# COMMON DOWNLIST DATA
LMCSTA03	EQUALS	LMORBM03			# COMMON DOWNLIST DATA
LMCSTA04	EQUALS	LMORBM04			# COMMON DOWNLIST DATA
LMCSTA05	EQUALS	LMORBM05			# COMMON DOWNLIST DATA

; LMCSTA06: IMU compensation package sub-list
; X789 contains gyro compensation parameters for drift correction. Ground uses
; these values to assess platform calibration quality during extended coast.
LMCSTA06	2DNADR	X789				# X789 +0...+3 - Gyro compensation
		-1DNADR	LASTYCMD			# LASTYCMD,LASTXCMD

; LMCSTA07: Optical tracking sub-list (AOT alignment data)
; During P51/P52/P53, crew sights stars through Alignment Optical Telescope (AOT).
; OGC/IGC/MGC are outer/inner/middle gimbal compensation. STARSAV1/STARSAV2 store
; star sighting vectors enabling ground to verify alignment accuracy. BESTI/BESTJ
; identify which star pair provided best alignment solution.
LMCSTA07	3DNADR	OGC				# OGC,+1,IGC,+1,MGC,+1 - Gimbal comp
		1DNADR	BESTI				# BESTI,BESTJ - Best star indices
		3DNADR	STARSAV1			# STARSAV1 +0...+5 - Star 1 vector
		-3DNADR	STARSAV2			# STARSAV2 +0...+5 - Star 2 vector
# Page 197
# -----------------------------------------------------------

# Page 198
# LM RENDEZVOUS AND PRE-THRUST DOWNLIST
#
# --------------------- CONTROL LIST ------------------------
;
; COMMENT-ONLY READERS: This downlink list supports rendezvous operations and
; pre-burn preparations. During rendezvous with the Command Module (Collins
; in Columbia orbiting above), ground controllers monitor relative range/rate,
; maneuver targeting data, and radar tracking quality. This list is active
; during P20-P25 rendezvous navigation programs and pre-TPI targeting setup.
;
; CODE-ALONG READERS: LMRENDDL emphasizes rendezvous-specific data. The control
; list snapshots AGC clock, radar range/rate (DNRRANGE, DNRRDOT), relative
; position/velocity vectors, CSI/CDH/TPI targeting times and delta-V values,
; and rendezvous radar LOS angles. Unique sub-list LMREND07 provides radar
; antenna pointing data. Most sub-lists reuse LMORBM definitions via EQUALS.

; Rendezvous control list sends two snapshots (LMREND01 and LMREND07) followed
; by rendezvous maneuver parameters. TCSI/TCDH/TTPI are times for Coelliptic
; Sequence Initiation, Constant Delta-Height, and Terminal Phase Initiation burns.
; DELVEET1/2/3 are delta-V vectors for CSI, CDH, and TPI maneuvers respectively.
LMRENDDL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	LMREND01			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	LMREND07			# COLLECT SNAPSHOT (radar angles)
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		1DNADR	DELLT4				# DELLT4,+1
		3DNADR	RTARG				# RTARG +0...+5 - Target position
		3DNADR	DELVSLV				# DELVSLV +0...+5 - Delta-V to solve
		1DNADR	TCSI				# TCSI,+1 - CSI burn time
		3DNADR	DELVEET1			# DELVEET +0...+5 - CSI delta-V
		1DNADR	SPARE
		1DNADR	TPASS4				# TPASS4,+1 - Fourth pass time
		DNPTR	LMREND06			# COMMON DATA
		DNPTR	LMREND02			# COMMON DATA
		1DNADR	TIME2				# TIME2/1
		DNPTR	LMREND03			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	LMREND04			# COMMON DATA
		2DNADR	POSTORKU			# POSTORKU,NEGTORKU,POSTORKV,NEGTORKV
		1DNADR	SPARE
		1DNADR	TCDH				# TCDH,+1 - CDH burn time
		3DNADR	DELVEET2			# DELVEET2 +0...+5 - CDH delta-V
		1DNADR	TTPI				# TTPI,+1 - TPI burn time
		3DNADR	DELVEET3			# DELVEET3 +0...+5 - TPI delta-V
		1DNADR	ELEV				# ELEV,+1 - Elevation angle
		2DNADR	CDUS				# CDUS,PIPAX,PIPAY,PIPAZ - IMU angles/accel
		1DNADR	LASTYCMD			# LASTYCMD,LASTXCMD - Last DAP commands
		1DNADR	LEMMASS				# LEMMASS,CSMMASS - Vehicle masses
		1DNADR	IMODES30			# IMODES30,IMODES33 - IMU mode flags
		1DNADR	TIG				# TIG,+1 - Time of ignition
		DNPTR	LMREND05			# COMMON DATA
		1DNADR	DELTAR				# DELTAR,+1 - Delta radius
		1DNADR	CENTANG				# CENTANG,+1 - Central angle
		1DNADR	NN				# NN,+1 - Braking parameter
		1DNADR	DIFFALT				# DIFFALT,+1 - Altitude difference
		1DNADR	DELVTPF				# DELVTPF,+1 - Delta-V terminal phase final
		-1DNADR	SPARE

# --------------------- SUB-LISTS --------------------------
;
; Most LMREND sub-lists are aliases to LMORBM sub-lists. Sub-list LMREND07
; is unique and provides rendezvous radar antenna pointing data and tracking
; status, critical for ground monitoring of radar lock quality during approach.

LMREND01	EQUALS	LMORBM01			# COMMON DOWNLIST DATA
LMREND02	EQUALS	LMORBM02			# COMMON DOWNLIST DATA
LMREND03	EQUALS	LMORBM03			# COMMON DOWNLIST DATA
# Page 199
LMREND04	EQUALS	LMORBM04			# COMMON DOWNLIST DATA
LMREND05	EQUALS	LMORBM05			# COMMON DOWNLIST DATA
LMREND06	EQUALS	LMCSTA06			# COMMON DOWNLIST DATA

; LMREND07: Rendezvous radar antenna pointing and tracking quality data.
; During Apollo 11 rendezvous (July 21, 1969), this data confirmed radar lock
; on Columbia (CM) as Eagle ascended from lunar surface. Ground controllers
; monitored these angles to verify tracking before commit to rendezvous burns.
LMREND07	-1DNADR	AIG				# AIG,AMG - Antenna inner/middle gimbal angles
		1DNADR	AOG				# AOG,TRKMKCNT - Outer gimbal angle, track mark count
		1DNADR	TANGNB				# TANGNB,+1 - Trunnion angle in navigation base
		1DNADR	MKTIME				# MKTIME,+1 - Mark time
		-1DNADR	RANGRDOT			# DNRRANGE,DNRRDOT - Radar range and range rate

# -----------------------------------------------------------

# Page 200
# LM DESCENT AND ASCENT DOWNLIST
;
; ============================================================================
; TRANSITION: From orbital operations to powered descent/ascent
;
; This downlist supports the most critical mission phases: lunar descent
; (P63/P64 powered descent) and lunar ascent (P12). During Apollo 11's
; historic landing on July 20, 1969, and Eagle's return to lunar orbit on
; July 21, this downlist transmitted guidance data to mission control,
; including the landing radar data that triggered the famous 1202 alarm.
; Ground controllers monitored UNFC/2 (unit thrust vector), VGVECT (velocity),
; and landing radar measurements to confirm safe descent trajectory.
; ============================================================================

# ---------------------- CONTROL LIST ------------------------
;
; LMDSASDL combines specialized landing radar data (LMDSAS07/08 snapshots)
; with guidance parameters. TTF/8 is time-to-go-to-ignition divided by 8.
; RLS is landing site radius vector. LAND is computed landing site position.
; FC is final commanded thrust magnitude. This list monitored Eagle's descent
; from powered descent initiation (102:33 MET) through touchdown (102:45 MET).

LMDSASDL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	LMDSAS07			# COLLECT SNAPSHOT (landing radar data)
		DNPTR	LMDSAS08			# SEND SNAPSHOT
		1DNADR	TEVENT				# TEVENT,+1 - Event time
		3DNADR	UNFC/2				# UNFC/2 +0...+5 - Unit thrust vector / 2
		3DNADR	VGVECT				# VGVECT +0...+5 - Velocity to be gained
		1DNADR	TTF/8				# TTF/8,+1 - Time-to-go to ignition / 8
		1DNADR	DELTAH				# DELTAH,+1 - Altitude difference
		3DNADR	RLS				# RLS +0...+5 - Landing site radius vector
		1DNADR	SPARE
		DNPTR	LMDSAS09			# COMMON DATA (IMU compensation)
		DNPTR	LMDSAS02			# COMMON DATA
		1DNADR	TIME2				# TIME2/1
		DNPTR	LMDSAS03			# COLLECT SNAPSHOT (state vectors)
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	LMDSAS04			# COMMON DATA
		2DNADR	POSTORKU			# POSTORKU,NEGTORKU,POSTORKV,NEGTORKV - Torque limits
		3DNADR	RGU				# RGU +0...+5 - Position vector (updated)
		3DNADR	VGU				# VGU +0...+5 - Velocity vector (updated)
		3DNADR	LAND				# LAND +0...+5 - Computed landing site position
		1DNADR	AT				# AT,+1 - Acceleration magnitude
		1DNADR	TLAND				# TLAND,+1 - Predicted landing time
		1DNADR	FC				# FC,GARBAGE - Final commanded thrust magnitude
		1DNADR	LASTYCMD			# LASTYCMD,LASTXCMD - Last commanded roll/pitch
		1DNADR	LEMMASS				# LEMMASS,CSMMASS - LM and CSM masses
		1DNADR	IMODES30			# IMODES30,IMODES33 - IMU mode status flags
		1DNADR	TIG				# TIG,+1 - Time of ignition
		DNPTR	LMDSAS05			# COMMON DATA (extended verb status)
		DNPTR	LMDSAS06			# COMMON DATA (alarm codes)
		1DNADR	PSEUDO55			# PSEUDO55,GARBAGE - Pseudo-channel 55
		-1DNADR	TTOGO				# TTOGO,+1 - Time to go (END OF LIST)

# ---------------------- SUB-LISTS ------------------------
;
; Most LMDSAS sub-lists are aliases to LMORBM sub-lists. Sub-lists LMDSAS07/08
; are unique and provide landing radar measurements critical for powered descent.
; During Apollo 11's descent (July 20, 1969, 102:33-102:45 MET), this radar
; data stream contributed to the computational load that triggered the 1202
; program alarm. Ground controllers monitored this data to confirm safe descent.

LMDSAS02	EQUALS	LMORBM02			# COMMON DOWNLIST DATA
LMDSAS03	EQUALS	LMORBM03			# COMMON DOWNLIST DATA
LMDSAS04	EQUALS	LMORBM04			# COMMON DOWNLIST DATA
LMDSAS05	EQUALS	LMORBM05			# COMMON DOWNLIST DATA
LMDSAS06	EQUALS	LMORBM06			# COMMON DOWNLIST DATA

; LMDSAS07: Landing radar measurements snapshot collected during powered descent.
; Provides altitude (HMEAS), velocity (VMEAS), range (RM), and antenna pointing
; angles. This data validated guidance computer trajectory computations and
; enabled ground controllers to monitor descent progress in real-time.
LMDSAS07	-1DNADR	LRZCDUDL			# LRZCDUDL,GARBAGE - LR Z-axis CDU (altitude)
		1DNADR	VSELECT				# VSELECT,GARBAGE - Velocity selection flag
		1DNADR	LRVTIMDL			# LRVTIMDL,+1 - LR velocity time downlink
# Page 201
		1DNADR	VMEAS				# VMEAS,+1 - Measured velocity from radar
		1DNADR	MKTIME				# MKTIME,+1 - Mark time (velocity measurement)
		1DNADR	HMEAS				# HMEAS,+1 - Measured altitude from radar
		1DNADR	RM				# RM,+1 - Range measurement
		1DNADR	AIG				# AIG,AMG - Antenna inner/middle gimbal angles
		1DNADR	AOG				# AOG,TRKMKCNT - Antenna outer gimbal, track count
		1DNADR	TANGNB				# TANGNB,+1 - Trunnion angle (navigation base)
		1DNADR	MKTIME				# MKTIME,+1 - Mark time (position measurement)
		-1DNADR	LRXCDUDL			# LRXCDUDL,LRYCDUDL - LR X/Y-axis CDUs (END)

; LMDSAS08: Snapshot buffer send (contains landing radar snapshot from LMDSAS07).
; Transmits two buffer sections totaling 22 AGC words of landing radar data
; to ground stations, providing real-time descent monitoring capability.
LMDSAS08	6DNADR	DNTMBUFF			# SEND SNAPSHOT - Buffer start (12 words)
		-5DNADR	DNTMBUFF +12D			# Continuation (10 words) - END

LMDSAS09	EQUALS	LMCSTA06			# COMMON DOWNLIST DATA

# ---------------------------------------------------------

# Page 202
# LM LUNAR SURFACE ALIGN DOWNLIST
;
; LMLSALDL: Lunar surface alignment downlist used during LM surface operations.
; After landing, the IMU (Inertial Measurement Unit) required realignment using
; optical sightings of stars. This downlist telemeters alignment parameters,
; reference matrices (REFSMMAT), star sightings (YNBSAV, SNBSAV), and gravity
; vector data (GSAV) to ground controllers for verification. During Apollo 11's
; surface stay (July 20-21, 1969), this data supported post-landing IMU checks.

# ---------------------- CONTROL LIST ---------------------

LMLSALDL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	LMLSAL01			# COLLECT SNAPSHOT - Navigation data
		6DNADR	DNTMBUFF			# SEND SNAPSHOT (12 words)
		DNPTR	LMLSAL07			# COLLECT SNAPSHOT - Alignment data
		4DNADR	DNTMBUFF			# SEND SNAPSHOT (8 words)
		1DNADR	TALIGN				# TALIGN,+1 - Alignment time
		6DNADR	REFSMMAT			# REFSMMAT +0...+11D - Reference matrix
		6DNADR	YNBSAV				# YNBSAV +0...+5,SNBSAV +0...+5 - Star vectors
		DNPTR	LMLSAL08			# COMMON DATA - Coast/align modes
		DNPTR	LMLSAL02			# COMMON DATA - State vector
		1DNADR	TIME2				# TIME2/1 - Mission time
		DNPTR	LMLSAL03			# COLLECT SNAPSHOT - IMU data
		6DNADR	DNTMBUFF			# SEND SNAPSHOT (12 words)
		DNPTR	LMLSAL04			# COMMON DATA - Orbital parameters
		DNPTR	LMLSAL09			# COMMON DATA - Gimbal angles
		3DNADR	GSAV				# GSAV +0...+5 - Gravity vector (6 words)
		1DNADR	AGSK				# AGSK,+1 - Gyro acceleration scale factors
		1DNADR	LASTYCMD			# LASTYCMD,LASTXCMD - Last torque commands
		1DNADR	LEMMASS				# LEMMASS,CSMMASS - Vehicle masses
		1DNADR	IMODES30			# IMODES30,IMODES33 - IMU mode indicators
		1DNADR	TIG				# TIG,+1 - Time of ignition (next maneuver)
		DNPTR	LMLSAL05			# COMMON DATA - Targeting
		DNPTR	LMLSAL06			# COMMON DATA - Configuration
		1DNADR	SPARE				# Reserved
		-1DNADR	SPARE				# Reserved - END OF LIST

# ---------------------- SUB-LISTS ----------------------
;
; All LMLSAL sub-lists are aliases to previously defined common sub-lists,
; maximizing code reuse. LMLSAL07 uniquely references LMREND07 for rendezvous
; data, while LMLSAL08/09 reference coast/align mode data from LMCSTA06/07.

LMLSAL01	EQUALS	LMORBM01			# COMMON DOWNLIST DATA
LMLSAL02	EQUALS	LMORBM02			# COMMON DOWNLIST DATA
LMLSAL03	EQUALS	LMORBM03			# COMMON DOWNLIST DATA
LMLSAL04	EQUALS	LMORBM04			# COMMON DOWNLIST DATA
LMLSAL05	EQUALS	LMORBM05			# COMMON DOWNLIST DATA
LMLSAL06	EQUALS	LMORBM06			# COMMON DOWNLIST DATA
LMLSAL07	EQUALS	LMREND07			# COMMON DOWNLIST DATA
LMLSAL08	EQUALS	LMCSTA06			# COMMON DOWNLIST DATA
LMLSAL09	EQUALS	LMCSTA07			# COMMON DOWNLIST DATA

# Page 203
# --------------------------------------------------------

# Page 204
# LM AGS INITIALIZATION AND UPDATE DOWNLIST
;
; LMAGSIDL: Abort Guidance System (AGS) initialization and update downlist.
; The AGS was the LM's backup guidance computer (also known as the Abort
; Electronic Assembly, AEA), independent of the primary AGC. This downlist
; telemeters data being uplinked to initialize or update the AGS, including
; state vectors stored in AGSBUFF and uplink command buffers (UPBUFF).
; During Apollo 11, the AGS remained in standby mode as backup during descent
; and ascent. Used post-P27 (update program) to verify ground uplink reception.

# ---------------------- CONTROL LIST --------------------

LMAGSIDL	EQUALS					# SEND IO BY SPECIAL CODING
		3DNADR	AGSBUFF +0			# AGSBUFF +0...+5 - AGS buffer section 1
		1DNADR	AGSBUFF +12D			# AGSBUFF +12D,GARBAGE - Padding word
		3DNADR	AGSBUFF +1			# AGSBUFF +1...+6 - AGS buffer section 2
		1DNADR	AGSBUFF +13D			# AGSBUFF +13D,GARBAGE - Padding word
		3DNADR	AGSBUFF +6			# AGSBUFF +6...+11 - AGS buffer section 3
		1DNADR	AGSBUFF +12D			# AGSBUFF +12D,GARBAGE - Padding word
		3DNADR	AGSBUFF +7			# AGSBUFF +7...+12D - AGS buffer section 4
		1DNADR	AGSBUFF +13D			# AGSBUFF +13D,GARBAGE - Padding word
		6DNADR	COMPNUMB			# COMPNUMB,UPOLDMOD,UPVERB,UPCOUNT,
							# UPBUFF +0...+7 - Uplink header + data
		6DNADR	UPBUFF +8D			# UPBUFF +8D...+19D - Uplink continuation
		DNPTR	LMAGSI02			# COMMON DATA - State vector
		1DNADR	TIME2				# TIME2/1 - Mission time
		DNPTR	LMAGSI03			# COLLECT SNAPSHOT - IMU data
		6DNADR	DNTMBUFF			# SEND SNAPSHOT (12 words)
		DNPTR	LMAGSI04			# COMMON DATA - Orbital parameters
		2DNADR	POSTORKU			# POSTORKU,NEGTORKU,POSTORKV,NEGTORKV
		1DNADR	SPARE				# Reserved
		1DNADR	SPARE				# Reserved
		1DNADR	AGSK				# AGSK,+1 - Gyro acceleration scale
		6DNADR	UPBUFF				# UPBUFF +0...+11D - Full uplink buffer
		4DNADR	UPBUFF +12D			# UPBUFF +12D...+19D - Buffer continuation
		1DNADR	LEMMASS				# LEMMASS,CSMMASS - Vehicle masses
		1DNADR	IMODES30			# IMODES30,IMODES33 - IMU mode indicators
		1DNADR	SPARE				# Reserved
		DNPTR	LMAGSI05			# COMMON DATA - Targeting
		-6DNADR	DSPTAB				# DSPTAB +0...+11D - Display table - END

# ---------------------- SUB-LISTS ---------------------
;
; All LMAGSI sub-lists are aliases to common LMORBM sub-lists, maximizing
; code reuse. These shared sub-lists provide state vectors, IMU data, orbital
; parameters, and targeting data common across multiple mission phases.

LMAGSI02	EQUALS	LMORBM02			# COMMON DOWNLIST DATA - State vector
LMAGSI03	EQUALS	LMORBM03			# COMMON DOWNLIST DATA - IMU snapshot
LMAGSI04	EQUALS	LMORBM04			# COMMON DOWNLIST DATA - Orbital params
LMAGSI05	EQUALS	LMORBM05			# COMMON DOWNLIST DATA - Targeting

# ------------------------------------------------------
;
; DNTABLE: Master downlink list address table.
; This table provides entry points for the downlink telemetry program
; (DOWN_TELEMETRY_PROGRAM.agc) to access mission-phase-specific downlists.
; The downlink program indexes into this table based on the current mission
; phase (stored in DNLSTADR) to select the appropriate data for ground
; telemetry. Each GENADR (general address) points to a control list that
; orchestrates which AGC data is formatted and transmitted to MSFN ground
; stations during that mission phase.
;
; COMMENT-ONLY READERS: This table routes telemetry data based on mission
; phase, ensuring ground controllers always receive the most relevant data.
;
; CODE-ALONG READERS: Indexed by mission phase code, returns GENADR pointer
; to phase-specific control list structure. GENADR encoding allows addressing
; across memory banks.

DNTABLE		GENADR	LMCSTADL			# Index 0: Coast and IMU alignment
		GENADR	LMAGSIDL			# Index 1: AGS initialization/update
		GENADR	LMRENDDL			# Index 2: Rendezvous and pre-thrust
		GENADR	LMORBMDL			# Index 3: Orbital maneuvers
		GENADR	LMDSASDL			# Index 4: Descent and ascent (CRITICAL)
# Page 205
		GENADR	LMLSALDL			# Index 5: Lunar surface alignment

# ------------------------------------------------------


