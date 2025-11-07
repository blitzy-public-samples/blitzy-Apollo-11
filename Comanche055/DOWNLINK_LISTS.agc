# Copyright:	Public domain.
# Filename:	DOWNLINK_LISTS.agc
# Purpose:	Part of the source code for Comanche, build 055. It
#		is part of the source code for the Command Module's
#		(CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 170-180
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	10/05/09 FB	Transcription of Batch FB-1 Assignment.
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
; FILE: DOWNLINK_LISTS.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: Telemetry downlink data structure definitions organizing spacecraft
;        state information for transmission to ground stations. Formats navigation
;        state, guidance parameters, system health data, and program status into
;        MSFN (Manned Space Flight Network) downlink packets enabling Mission
;        Control monitoring throughout Apollo 11.
;
; COMMENT-ONLY READERS: This file organized spacecraft data into packages for
;        radio transmission to Mission Control in Houston.
; CODE-ALONG READERS: Study telemetry data structure organization, downlink
;        packet formatting, priority scheduling, and MSFN protocol requirements.
; ============================================================================

# Page 170
		BANK	22
		SETLOC	DOWNTELM
		BANK

		EBANK=	DNTMBUFF

; ============================================================================
; DOWNLINK TELEMETRY SYSTEM OVERVIEW
;
; The Command Module continuously transmits spacecraft state data to Mission
; Control Houston via the MSFN (Manned Space Flight Network) ground stations.
; This file defines the structure of telemetry downlink lists - organized
; collections of data transmitted during different mission phases. During
; Apollo 11's journey to the Moon and back, these lists enabled flight
; controllers to monitor navigation accuracy, guidance performance, system
; health, and crew activities.
;
; The telemetry system operates on a priority-based scheduling model where
; time-critical data (navigation state, attitude) is preserved in snapshot
; buffers and transmitted immediately, while less critical data (program
; status, system flags) is sent cyclically. The DOWNRUPT interrupt handler
; processes these lists, extracting data from erasable memory and formatting
; it for S-band radio transmission to Earth.
; ============================================================================

; ============================================================================
; DOWNLINK OPCODE DEFINITIONS
;
; Downlink lists use special opcodes that specify how many AGC words to
; transmit and where to find the data. Each opcode encodes the data length
; and memory address into a single 16-bit word using bit field encoding.
; The downlink interrupt handler decodes these opcodes to extract the
; appropriate data from erasable memory locations.
;
; OPCODE STRUCTURE:
; Bit 15: Sign bit (0 = continue list, 1 = end of list/snapshot marker)
; Bits 14-12: Opcode type field (determines word count)
; Bits 11-0: Memory address (ECADR for erasable, channel number for I/O)
;
; During Apollo 11, these opcodes enabled efficient data transmission
; without requiring separate length and address fields, conserving precious
; downlink bandwidth on the 51.2 kilobit/second S-band telemetry channel.
; ============================================================================

# SPECIAL DOWNLINK OP CODES
#	OP CODE		ADDRESS(EXAMPLE)	SENDS...		BIT 15		BITS 14-12	BITS 11-0
#	-------		----------------	----------		------		----------	---------
#	1DNADR		TIME2			(2 AGC WDS)		0		0		ECADR
#	2DNADR		TEPHEM			(4 AGC WDS)		0		1		ECADR
#	3DNADR		VGBODY			(6 AGC WDS)		0		2		ECADR
#	4DNADR		STATE			(8 AGC WDS)		0		3		ECADR
#	5DNADR		UPBUFF			(10AGC WDS)		0		4		ECADR
#	6DNADR		DSPTAB			(12AGC WDS)		0		5		ECADR
#	DNCHAN		30			CHANNELS		0		7		CHANNEL ADDRESS
#	DNPTR		NEXTLIST		POINTS TO NEXT LIST.	0		6		ADRES
;
; ============================================================================
; DOWNLIST FORMAT RULES AND SNAPSHOT MECHANISM
;
; Downlink lists follow structured formatting rules that enable the downlink
; interrupt handler to process telemetry data efficiently:
;
; LIST TERMINATION: A negative opcode (bit 15 set) marks the end of a list
; or sublist, signaling the handler to move to the next list segment.
;
; SNAPSHOT SUBLISTS: Time-critical data (navigation state during burns,
; attitude during maneuvers) requires immediate preservation to prevent
; updates from changing values mid-transmission. Snapshot sublists copy
; data into DNTMBUFF buffer in a single interrupt cycle, then transmit
; the frozen snapshot over multiple downlink cycles. This preserved Armstrong
; and Aldrin's precise state vectors during critical maneuvers.
;
; TIME STAMPING: Every control list includes a TIME2 reference providing
; Mission Control the exact mission elapsed time when the data was valid.
; This enabled ground controllers to correlate telemetry with trajectory
; predictions and detect navigation drift.
;
; TRANSMISSION PRIORITY: Lists are transmitted front-to-back, with snapshot
; data receiving highest priority. During Apollo 11's translunar coast,
; this ensured navigation updates reached Houston before less critical
; system status information.
; ============================================================================
#
# DOWNLIST FORMAT DEFINITIONS AND RULES -
# 1. END OF A LIST = -XDNADR (X = 1 TO 6), -DNPTR, OR -DNCHAN.
# 2. SNAPSHOT SUBLIST = LIST WHICH STARTS WITH A -1DNADR.
# 3. SNAPSHOT SUBLIST CAN ONLY CONTAIN 1DNADRS.
# 4. TIME2 1DNADR MUST BE LOCATED IN THE CONTROL LIST OF A DOWNLIST.
# 5. ERASABLE DOWN TELEMETRY WORDS SHOULD BE GROUPED IN SEQUENTIAL
#    LOCATIONS AS MUCH AS POSSIBLE TO SAVE STORAGE USED BY DOWNLINK LISTS.
# 6. THE DOWNLINK LISTS (INCLUDING SUBLISTS) ARE ORGANIZED SUCH THAT THE ITEMS LISTED FIRST (IN FRONT OF FBANK) ARE
#    SENT FIRST.  EXCEPTION--- SNAPSHOT SUBLISTS.  IN THE SNAPSHOT SUBLISTS THE DATA REPRESENTED BY THE FIRST
#    11 1DNADRS IS PRESERVED (IN ORDER) IN DNTMBUFF AND SENT BY THE NEXT 11 DOWNRUPTS. THE DATA REPRESENTED BY THE
#    LIST IS SENT IMMEDIATELY.

		COUNT	05/DLIST
ERASZERO	EQUALS	7
SPARE		EQUALS	ERASZERO			# USE SPARE TO INDICATE AVAILABLE SPACE
LOWIDCOD	OCT	77340				# LOW ID CODE

NOMDNLST	EQUALS	CMCSTADL			# FRESH START AND POST P27 DOWNLIST
UPDNLIST	EQUALS	CMENTRDL			# UPDATE PROGRAM (P27) DOWNLIST

# Page 171
; ============================================================================
; TRANSITION: From general telemetry definitions to mission-phase downlists
;
; The following sections define specific downlink lists tailored to different
; phases of Apollo 11's mission. Each mission phase required different
; telemetry priorities: powered flight emphasized thrust vector and guidance
; data, coast phases prioritized navigation accuracy, rendezvous focused on
; relative state vectors, and entry required atmospheric data. By switching
; between these lists, Mission Control received the most relevant information
; for each flight regime.
; ============================================================================

# CSM POWERED FLIGHT DOWNLIST
;
; ============================================================================
; CSM POWERED FLIGHT DOWNLIST (CMPOWEDL)
;
; This downlist transmitted telemetry during all Service Propulsion System
; (SPS) engine burns: translunar injection (TLI), midcourse corrections,
; lunar orbit insertion (LOI), and transearth injection (TEI). During these
; critical burns, Mission Control monitored thrust vector control, guidance
; steering commands, and navigation state updates to verify the spacecraft
; remained on the planned trajectory.
;
; For Apollo 11, this list was active during:
; - TLI burn (July 16, 1969): Earth orbit departure toward Moon
; - LOI burn (July 19, 1969): Lunar orbit insertion
; - TEI burn (July 21, 1969): Return to Earth initiation
;
; The list uses multiple snapshot sublists to preserve time-critical attitude
; and navigation data at precise moments during the burn sequence.
; ============================================================================
#
# --------------------- CONTROL LIST -------------------------

; POWERED FLIGHT CONTROL LIST STRUCTURE:
; This control list coordinates multiple snapshot sublists and direct data
; transmission. Snapshots freeze navigation state and attitude at specific
; moments, preventing corruption from guidance updates during multi-cycle
; telemetry transmission. Direct data items (TIG, DELV, REFSMMAT) are sent
; without buffering since they remain constant during the burn sequence.

CMPOWEDL	EQUALS
		DNPTR	CMPOWE01			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMPOWE02			# COLLECT SECOND SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMPOWE03			# COMMON DATA
		1DNADR	TIG				# TIG,+1 (Time of ignition)
		1DNADR	DELLT4				# DELLT4,+1 (Burn duration error)
		3DNADR	RTARG				# RTARG,+1,+2,...+5 (Target position vector)
		1DNADR	TGO				# TGO,+1 (Time remaining in burn)
		1DNADR	PIPTIME1			# PIPTIME1,+1 (Accelerometer sample time)
		3DNADR	DELV				# DELV,+1,...+4,+5 (Velocity change achieved)
		1DNADR	PACTOFF				# PACTOFF,YACTOFF (Pitch/yaw trim offsets)
		1DNADR	PCMD				# PCMD,YCMD (Pitch/yaw gimbal commands)
		1DNADR	CSTEER				# CSTEER,+1 (Steering command)
		3DNADR	DELVEET1			# CSI DELTA VELOCITY COMPONENTS   (31-33)
		6DNADR	REFSMMAT			# REFSMMAT,+1,...+10,+11 (Reference matrix)
		DNPTR	CMPOWE04			# COMMON DATA
		1DNADR	TIME2				# TIME2,TIME1 (Mission elapsed time stamp)
		DNPTR	CMPOWE05			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMPOWE02			# COLLECT SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMPOWE03			#
		DNPTR	CMPOWE06			# COMMON DATA
		1DNADR	ELEV				# ELEV,+1 (Elevation angle)
		1DNADR	CENTANG				# CENTANG,+1
		1DNADR	DELTAR				# DELTAR,+1
		1DNADR	STATE +10D			# FLAGWRDS 10 AND 11
		1DNADR	TEVENT				# TEVENT,+1
		1DNADR	PCMD				# PCMD,YCMD
		1DNADR	OPTMODES			# OPTMODES,HOLDFLAG
		DNPTR	CMPOWE07			# COMMON DATA
		3DNADR	VGTIG				# VGTIG,+1,...+4,+5
		-3DNADR DELVEET2			# CDH DELTA VELOCITY COMPONENTS   (98-100)
# --------------------- SUB LISTS ---------------------------
;
; REUSABLE DATA SUBLISTS (CMPOWE01 through CMPOWE07)
;
; These sublists contain common telemetry data blocks referenced by multiple
; downlists. By defining them once and reusing via EQUALS statements, the AGC
; saves precious fixed memory storage. Each sublist serves a specific purpose:
;
; COMMENT-ONLY READERS: These are the building blocks that package different
; types of spacecraft data for transmission. Think of them as standardized
; containers that get reused across different mission phases.
;
; CODE-ALONG READERS: Note the efficient memory management - rather than
; duplicating these data structure definitions for each mission phase downlist,
; the code uses EQUALS to create aliases. The snapshot sublists (starting with
; -1DNADR) trigger special DNTMBUFF preservation behavior.
;
; ----------------------------------------------------------------------------
;
; SUBLIST: CMPOWE01 - Primary Navigation State Snapshot
;
; Captures position and velocity vectors at a specific instant in time.
; This snapshot is preserved in DNTMBUFF for transmission over 11 consecutive
; downrupts, ensuring time-coherent state data reaches Mission Control.
;
; Data: RN (position vector), VN (velocity vector), PIPTIME (measurement time)
; All vectors in scaled fixed-point format relative to reference frame.
;
CMPOWE01	-1DNADR	RN	+2			# RN+2,+3			SNAPSHOT DATA
		1DNADR	RN	+4			# RN+4,+5
		1DNADR	VN				# VN,+1
		1DNADR	VN	+2			# VN+2,+3
		1DNADR	VN	+4			# VN+4,+5
		1DNADR	PIPTIME				# PIPTIME,+1
		-1DNADR	RN				# RN,+1

; SUBLIST: CMPOWE02 - IMU Gimbal Angles and Body Rates Snapshot
;
; Captures Inertial Measurement Unit gimbal angles and spacecraft angular rates.
; Critical for attitude determination and control system monitoring.
;
; CDUX, CDUY, CDUZ: IMU gimbal Coupling Data Unit angles
; ADOT/OGARATE/OMEGAB: Body angular rate components
;
CMPOWE02	-1DNADR	CDUZ				# CDUZ,CDUT			SNAPSHOT DATA
# Page 172
		1DNADR	ADOT				# ADOT,+1/OGARATE,+1
		1DNADR	ADOT	+2			# ADOT+2,+3/OMEGAB+2,+3
		1DNADR	ADOT	+4			# ADOT+4,+5/OMEGAB+4,+5
		-1DNADR	CDUX				# CDUX,CDUY

; SUBLIST: CMPOWE03 - Digital Autopilot Parameters (Common Data)
;
; Digital autopilot coefficients and attitude error data. Not a snapshot -
; sent as common data that doesn't require time-synchronization.
;
; AK: Autopilot gain coefficients for control law computation
; RCSFLAGS: Reaction Control System status flags
; THETADX/Y/Z: Attitude error components in body axes
;
CMPOWE03	2DNADR	AK				# AK,AK1,AK2,RCSFLAGS		COMMON DATA
		-2DNADR	THETADX				# THETADX,THETADY,THETADZ,GARBAGE

; SUBLIST: CMPOWE04 - Program State Flags and DSKY Display (Common Data)
;
; Software state flags and Display/Keyboard (DSKY) display table contents.
; Enables Mission Control to monitor which programs are running and what
; the crew sees on their DSKY display unit.
;
; STATE (FLAGWRD0-9): 10 words of program status flags controlling guidance,
;                     navigation, and display modes
; DSPTAB: 12 words containing DSKY seven-segment display data (verbs, nouns,
;         register contents visible to crew)
;
CMPOWE04	5DNADR	STATE				# FLAGWRD0 THRU FLAGWRD9	COMMON DATA
		-6DNADR	DSPTAB				# DISPLAY TABLES

; SUBLIST: CMPOWE05 - Other Vehicle State Snapshot
;
; Position and velocity of the "other vehicle" during rendezvous operations.
; For CSM, this is the LM state; for LM, this is the CSM state.
;
; R-OTHER: Position vector of other vehicle (6 components)
; V-OTHER: Velocity vector of other vehicle (6 components)  
; T-OTHER: Time tag for other vehicle state
;
; This snapshot enables relative navigation state monitoring at Mission Control.
;
CMPOWE05	-1DNADR	R-OTHER	+2			# R-OTHER+2,+3			SNAPSHOT DATA
		1DNADR	R-OTHER	+4			# R-OTHER+4,+5
		1DNADR	V-OTHER				# V-OTHER,+1
		1DNADR	V-OTHER	+2			# V-OTHER+2,+3
		1DNADR	V-OTHER	+4			# V-OTHER+4,+5
		1DNADR	T-OTHER				# T-OTHER,+1
		-1DNADR	R-OTHER				# R-OTHER,+1

; SUBLIST: CMPOWE06 - Star Tracker, Fault Registers, and IMU Data (Common Data)
;
; Monitors star tracker status, system fault registers, and IMU sensor data.
;
; RSBBQ: Star tracker (sextant) shaft and trunnion angles
; CADRFLSH: Flash display code address and associated data
; FAILREG: Fault register containing system failure indicators
; CDUS: Coupling Data Unit shaft angle (IMU gimbal position)
; PIPAX/Y/Z: Pulsed Integrating Pendulous Accelerometer readings (x, y, z axes)
;            Incremental velocity measurements from IMU accelerometers
;
CMPOWE06	1DNADR	RSBBQ				# RSBBQ,+1			COMMON DATA
		3DNADR	CADRFLSH			# CADRFLSH,+1,+2,FAILREG,+1,+2
		-2DNADR	CDUS				# CDUS,PIPAX,PIPAY,PIPAZ

; SUBLIST: CMPOWE07 - DAP Data, Channels, and System Status (Common Data)
;
; Digital Autopilot (DAP) state, control errors, body rates, and hardware I/O
; channel states for comprehensive system monitoring.
;
; LEMMASS/CSMMASS: Vehicle mass estimates for control law computation
; DAPDATR1/2: Digital Autopilot data registers (configuration and mode)
; ERRORX/Y/Z: Control system attitude errors in body axes
; WBODY: Body angular rate vector (or OMEGAC commanded angular rate)
; REDOCTR: RCS jet redundancy counter, attitude command angles (THETAD)
; IMODES30/33: IMU mode status indicators
; CHANNELS 11-14, 30-33: Hardware I/O channels (engine commands, display outputs,
;                        IMU readouts, RCS jet firing commands)
;
; COMMENT-ONLY READERS: This block captures the autopilot's decisions and the
; hardware commands being sent to thrusters and displays.
;
CMPOWE07	1DNADR	LEMMASS				# LEMMASS,CSMMASS		COMMON DATA
		1DNADR	DAPDATR1			# DAPDATR1,DAPDATR2
		2DNADR	ERRORX				# ERRORX,ERRORY,ERRORZ,GARBAGE
		3DNADR	WBODY				# WBODY,...+5/OMEGAC,...+5
		2DNADR	REDOCTR				# REDOCTR,THETAD,+1,+2
		1DNADR	IMODES30			# IMODES30,IMODES33
		DNCHAN	11				# CHANNELS 11,12
		DNCHAN	13				# CHANNELS 13,14
		DNCHAN	30				# CHANNELS 30,31
		-DNCHAN	32				# CHANNELS 32,33

# -----------------------------------------------------------

; ============================================================================
; TRANSITION: From Powered Flight to Coast and Alignment Phase
;
; Following engine cutoff and orbit insertion, the spacecraft transitions from
; powered flight monitoring to coast phase operations. The CMCSTADL (CSM Coast
; and Alignment Downlist) provides telemetry during orbital coast periods and
; IMU alignment procedures. This downlist efficiently reuses all seven CMPOWE
; sublists via memory-saving alias definitions, demonstrating the AGC's careful
; management of its limited 36K ROM capacity.
; ============================================================================

# Page 173
# CSM COAST AND ALIGNMENT DOWNLIST
#
# --------------------- CONTROL LIST ------------------------
;
; This control list sends telemetry during coast phases of the Apollo 11 mission
; (Earth orbit, translunar coast, lunar orbit). During these non-thrusting periods,
; Mission Control monitored navigation state, IMU alignment quality, and spacecraft
; system health. The snapshot mechanism captured time-critical data (navigation state,
; velocity, attitude) at specific moments for accurate ground reconstruction of
; spacecraft trajectory.

CMCSTADL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	CMCSTA01			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMCSTA02			# COLLECT SECOND SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMCSTA03			# COMMON DATA
		1DNADR	TIG				# TIG,+1
		1DNADR	BESTI				# BESTI,BESTJ
		4DNADR	MARKDOWN			# MARKDOWN,+1,...+5,+6,GARBAGE
		4DNADR	MARK2DWN			# MARK2DWN,+1,...+5,+6
		2DNADR	HAPOX				# APOGEE AND PERIGEE FROM R30   (28-29)
		1DNADR	PACTOFF				# PACTOFF, YACTOFF                 (30)
		3DNADR	VGTIG				# VGTIG,...+5
		6DNADR	REFSMMAT			# REFSMMAT,+1,...+10,+11
		DNPTR	CMCSTA04			# COMMON DATA
		1DNADR	TIME2				# TIME2,TIME1
		DNPTR	CMCSTA05			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMCSTA02			# COLLECT SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMCSTA03			# COMMON DATA
		DNPTR	CMCSTA06			# COMMON DATA
		3DNADR	OGC				# OGC,+1,IGC,+1,MGC,+1
		1DNADR	STATE +10D			# FLAGWRDS 10 AND 11
		1DNADR	TEVENT				# TEVENT,+1
		1DNADR	LAUNCHAZ			# LAUNCHAZ,+1
		1DNADR	OPTMODES			# OPTMODES,HOLDFLAG
		DNPTR	CMCSTA07			# COMMON DATA
		-6DNADR	DSPTAB				# DISPLAY TABLES

; --------------------- SUB LISTS ---------------------------
;
; MEMORY EFFICIENCY NOTE: All seven CMCSTA sublists are aliases (EQUALS statements)
; pointing to the corresponding CMPOWE sublists documented above. This design pattern
; saves valuable ROM space by reusing common telemetry data structures across multiple
; mission phases. The AGC's 36K fixed memory constraint drove this elegant approach
; to data organization.

CMCSTA01	EQUALS	CMPOWE01			# COMMON DOWNLIST DATA

CMCSTA02	EQUALS	CMPOWE02			# COMMON DOWNLIST DATA

CMCSTA03	EQUALS	CMPOWE03			# COMMON DOWNLIST DATA

CMCSTA04	EQUALS	CMPOWE04			# COMMON DOWNLIST DATA

CMCSTA05	EQUALS	CMPOWE05			# COMMON DOWNLIST DATA

CMCSTA06	EQUALS	CMPOWE06			# COMMON DOWNLIST DATA

CMCSTA07	EQUALS	CMPOWE07			# COMMON DOWNLIST DATA
# Page 174
# -----------------------------------------------------------

; ============================================================================
; TRANSITION: From Coast Operations to Rendezvous and Prethrust
;
; As the spacecraft prepares for critical maneuvers (rendezvous with the Lunar
; Module, transearth injection burns), telemetry requirements shift to focus on
; burn targeting parameters and trajectory calculations. The CMRENDDL provides
; prethrust checkout data and rendezvous navigation state. During Apollo 11,
; this downlist monitored Columbia's state during the historic rendezvous with
; Eagle after Armstrong and Aldrin's lunar ascent on July 21, 1969.
; ============================================================================

# Page 175
# CSM RENDEZVOUS AND PRETHRUST LIST
#
# --------------------- CONTROL LIST ------------------------
;
; This control list provides telemetry for rendezvous operations (relative navigation,
; trajectory targeting) and prethrust periods before major engine burns. Mission Control
; monitored TIG (Time of Ignition), delta-V components, attitude parameters, and REFSMMAT
; (reference stable member matrix) to verify spacecraft configuration before committing
; to burns. The snapshot sublists captured time-critical state data for precise trajectory
; reconstruction.

CMRENDDL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	CMREND01			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMREND02			# COLLECT SECOND SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMREND03			# COMMON DATA
		1DNADR	TIG				# TIG,+1
		1DNADR	DELLT4				# DELLT4,+1
		3DNADR	RTARG				# RTARG,+1,...+4,+5
		1DNADR	VHFTIME				# VHFTIME,+1
		4DNADR	MARKDOWN			# MARKTIME(DP),YCDU,SCDU,ZCDU,TCDU,XCDU,RM
		1DNADR	VHFCNT				# VHFCNT,+1
		1DNADR	TTPI				# TTPI,+1
		1DNADR	ECSTEER				# ECSTEER,+1
		1DNADR	DELVTPF				# DELVTPF,+1
		2DNADR  TCDH				# CDH AND CSI TIME                      (32-33)
		1DNADR	TPASS4				# TPASS4,+1
		3DNADR	DELVSLV				# DELVSLV,+1...+4,+5
		2DNADR	RANGE				# RANGE,+1,RRATE,+1
		DNPTR	CMREND04			# COMMON DATA
		1DNADR	TIME2				# TIME2,TIME1
		DNPTR	CMREND05			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMREND02			# COLLECT SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMREND03			# COMMON DATA
		DNPTR	CMREND06			# COMMON DATA
		1DNADR	DIFFALT				# CDH DELTA ALTITUDE
		1DNADR	CENTANG				# CENTANG,+1
		1DNADR	DELTAR				# DELTAR,+1
		3DNADR	DELVEET3			# DELVEET3,+1,...+4,+5
		1DNADR	OPTMODES			# OPTMODES,HOLDFLAG
		DNPTR	CMREND07			# COMMON DATA
		1DNADR	RTHETA				# RTHETA,+1
		2DNADR	LAT(SPL)			# LAT(SPL),LNG(SPL),+1
		2DNADR	VPRED				# VPRED,+1,GAMMAEI,+1
		-1DNADR	STATE +10D			# FLAGWRDS 10 AND 11

; --------------------- SUB LISTS ----------------------------
;
; MEMORY EFFICIENCY: Like CMCSTADL above, all seven CMREND sublists are aliases
; to the corresponding CMPOWE sublists. This architecture allows CMRENDDL to
; provide mission-specific control list sequences while reusing the common data
; collection sublists, minimizing ROM usage.

CMREND01	EQUALS	CMPOWE01			# COMMON DOWNLIST DATA

CMREND02	EQUALS	CMPOWE02			# COMMON DOWNLIST DATA

CMREND03	EQUALS	CMPOWE03			# COMMON DOWNLIST DATA

CMREND04	EQUALS	CMPOWE04			# COMMON DOWNLIST DATA
# Page 176
CMREND05	EQUALS	CMPOWE05			# COMMON DOWNLIST DATA

CMREND06	EQUALS	CMPOWE06			# COMMON DOWNLIST DATA

CMREND07	EQUALS	CMPOWE07			# COMMON DOWNLIST DATA

# ------------------------------------------------------------

; ============================================================================
; TRANSITION: To Entry and Update Operations
;
; As Apollo 11 prepared for Earth return and atmospheric entry on July 24, 1969,
; telemetry requirements shifted to entry-specific parameters. The CMENTRDL
; (CSM Entry and Update Downlist) provided data during reentry guidance and
; during update program operations (P27) when Mission Control uploaded state
; vectors and trajectory parameters. This hybrid downlist reuses some CMPOWE
; sublists but includes a unique CMENTR05 sublist for entry-specific data.
; ============================================================================

# Page 177
# CSM ENTRY AND UPDATE DOWNLIST
#
# --------------------- CONTROL LIST -------------------------
;
; This control list serves dual purposes: (1) atmospheric entry monitoring with
; lift/drag ratio (L/D1), roll attitude commands, and splashdown targeting data,
; and (2) update program (P27) operations with uplink buffer contents (UPBUFF)
; for ground-commanded state vector corrections. During entry, Mission Control
; monitored reentry corridor parameters to ensure safe splashdown in the Pacific
; recovery area.

CMENTRDL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	CMENTR01			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMENTR02			# COLLECT SECOND SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMENTR03			# COMMON DATA
		2DNADR	CMDAPMOD			# CMDAPMOD,PREL,QREL,RREL
		1DNADR	L/D1				# L/D1,+1
		6DNADR	UPBUFF				# UPBUFF,+1,...+10,+11
		4DNADR	UPBUFF +12D			# UPBUFF+12,13,...+18,+19D
		2DNADR	COMPNUMB			# COMPNUMB,UPOLDMOD,UPVERB,UPCOUNT
		1DNADR	PAXERR1				# PAXERR1,ROLLTM
		3DNADR	LATANG				# LATANG,+1,RDOT,+1,THETAH,+1
		2DNADR	LAT(SPL)			# LAT(SPL),+1,LNG(SPL),+1
		1DNADR	ALFA/180			# ALFA/180,BETA/180
		DNPTR	CMENTR04			# COMMON DATA
		1DNADR	TIME2				# TIME2,TIME1
		DNPTR	CMENTR05			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMENTR02			# COLLECT SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		2DNADR	AK				# AK,AK1,AK2,RCSFLAGS
		3DNADR	ERRORX				# ERRORX/Y/Z,THETADX/Y/Z
		2DNADR	CMDAPMOD			# CMDAPMOD,PREL,QREL,RREL
		6DNADR	UPBUFF				# UPBUFF+0,+1,...+10,+11D
		4DNADR	UPBUFF +12D			# UPBUFF+12,+13,...+18,+19D
		1DNADR	LEMMASS				# LEMMASS,CSMMASS
		1DNADR	DAPDATR1			# DAPDATR1,DAPDATR2
		1DNADR	ROLLTM				# ROLLTM,ROLLC
		1DNADR	OPTMODES			# OPTMODES,HOLDFLAG
		3DNADR	WBODY				# WBODY,...+5/OMEGAC,...+5
		2DNADR	REDOCTR				# REDOCTR,THETAD+0,+1,+2
		1DNADR	IMODES30			# IMODES30,IMODES33
		DNCHAN	11				# CHANNELS 11,12
		DNCHAN	13				# CHANNELS 13,14
		DNCHAN	30				# CHANNELS 30,31
		DNCHAN	32				# CHANNELS 32,33
		1DNADR	RSBBQ				# RSBBQ,+1
		3DNADR	CADRFLSH			# CADRFLSH,+1,+2,FAILREG,+1,+2
		1DNADR	STATE +10D			# FLAGWRDS 10 AND 11
		-1DNADR	GAMMAEI				# GAMMAEI,+1

; --------------------- SUB LISTS ----------------------------
;
; HYBRID ARCHITECTURE: CMENTRDL follows a hybrid memory optimization strategy.
; Five sublists (CMENTR01/02/03/04/07) are aliases reusing CMPOWE data structures,
; but CMENTR05 is a unique sublist capturing entry-specific snapshot data that
; wasn't required during powered flight. This pattern demonstrates how AGC
; programmers balanced memory efficiency (aliasing common data) with mission-specific
; requirements (unique sublists for specialized telemetry).

CMENTR01	EQUALS	CMPOWE01			# COMMON DOWNLIST DATA
# Page 178
CMENTR02	EQUALS	CMPOWE02			# COMMON DOWNLIST DATA

CMENTR03	EQUALS	CMPOWE03			# COMMON DOWNLIST DATA

CMENTR04	EQUALS	CMPOWE04			# COMMON DOWNLIST DATA

; CMENTR05 - UNIQUE ENTRY SNAPSHOT SUBLIST
;
; This is the only unique sublist in CMENTRDL. It captures entry-specific telemetry
; in snapshot format for atmospheric reentry monitoring. During Apollo 11's return on
; July 24, 1969, this data enabled Mission Control to verify reentry corridor accuracy
; and predict splashdown location.
;
; SNAPSHOT DATA CAPTURED:
; - DELV (6 words): Velocity changes measured by IMU accelerometers during entry
; - TTE (2 words): Time to go until entry interface (400,000 feet altitude)
; - VIO (2 words): Inertial velocity magnitude
; - VPRED (2 words): Predicted velocity at current trajectory point
; - PIPTIME1 (2 words): Time of last accelerometer pulse integration
;
; This telemetry supported the entry guidance algorithm's lift vector control,
; enabling the Command Module to "fly" through the atmosphere to reach the Pacific
; recovery area with precision.

CMENTR05	-1DNADR	DELV				# DELV,+1		SNAPSHOT DATA
		1DNADR	DELV +2				# DELV+2,+3
		1DNADR	DELV +4				# DELV+4,+5
		1DNADR	TTE				# TTE,+1
		1DNADR	VIO				# VIO,+1
		1DNADR	VPRED				# VPRED,+1
		-1DNADR	PIPTIME1			# PIPTIME1,+1

CMENTR07	EQUALS	CMPOWE07			# COMMON DOWNLIST DATA

; ============================================================================
; TRANSITION: From Entry Monitoring to Landmark Navigation Telemetry
;
; Having documented entry guidance telemetry for return to Earth, we now shift
; to optical navigation telemetry used during cislunar and lunar orbital phases.
; Program 22 (P22) enabled the crew to sight lunar landmarks through the sextant,
; providing independent navigation fixes to verify and update the spacecraft's
; position and velocity state vector. During Apollo 11, landmark tracking helped
; confirm trajectory accuracy during lunar orbit and could have provided backup
; navigation if communication with Earth was lost.
; ============================================================================

# -------------------------------------------------------------

# Page 179
# P22 DOWNLISTS - LANDMARK TRACKING TELEMETRY
;
; MISSION CONTEXT: Program 22 (P22) provided optical navigation capability by
; tracking known lunar surface landmarks through the spacecraft's sextant. During
; Apollo 11's lunar orbit, this program enabled the crew to independently verify
; their position by sighting pre-surveyed landmarks and measuring their angular
; position relative to the inertial reference frame.
;
; DOWNLIST PURPOSE: CMPG22DL transmits landmark sighting data to Mission Control,
; enabling ground navigation experts to verify optical measurement quality and
; update trajectory calculations. This data supported both real-time navigation
; during Apollo 11 and post-flight analysis of optical navigation accuracy.
;
; LANDMARK TRACKING OPERATIONS: The Command Module Pilot (Michael Collins during
; Apollo 11) would use the sextant to acquire a pre-designated landmark, track it
; as the spacecraft orbited, and mark the time of closest approach. The AGC
; recorded the landmark's line-of-sight vector and time, which could be processed
; to determine spacecraft position with accuracy comparable to ground-based tracking.
#
# --------------------- CONTROL LIST --------------------------
;
; CMPG22DL STRUCTURE: This downlist follows the standard snapshot + common data
; pattern. It begins with two snapshot collections (via CMPG2201 and CMPG2202),
; then transmits mission-specific landmark tracking data.
;
; KEY LANDMARK TRACKING DATA TRANSMITTED:
;
; SVMRKDAT (18 words total): Landing site mark data stored as three consecutive
; 6-word blocks. Each mark contains the spacecraft state vector at the time of
; landmark sighting, enabling ground controllers to back-calculate landmark
; position or verify spacecraft trajectory. During Apollo 11, multiple landmark
; sightings were downlinked for navigation analysis.
;
; LANDMARK: Identifier code for the specific landmark being tracked (catalogue
; number from pre-surveyed lunar landmark database).
;
; RLS: Position vector of landmark in selenocentric (Moon-centered) coordinates.
;
; OPTMODES/HOLDFLAG: Optical subsystem mode flags indicating sextant/telescope
; configuration and whether automatic landmark tracking (HOLD mode) is active.

CMPG22DL	EQUALS					# SEND ID BY SPECIAL CODING
		DNPTR	CMPG2201			# COLLECT SNAPSHOT
		6DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMPG2202			# COLLECT SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMPG2203			# COMMON DATA
		6DNADR	SVMRKDAT			# LANDING SITE MARK DATA
		6DNADR	SVMRKDAT +12D			# SVMRKDAT+0...+34
		6DNADR	SVMRKDAT +24D			# LANDING SITE MARK DATA
		1DNADR	LANDMARK			# LANDMARK,GARBAGE
		1DNADR	SPARE
		1DNADR	SPARE
		1DNADR	SPARE
		DNPTR	CMPG2204			# COMMON DATA
		1DNADR	TIME2				# TIME2,TIME1
		DNPTR	CMPG2205			# COLLECT SNAPSHOT
		2DNADR	DNTMBUFF			# SEND SNAPSHOT (LONG,ALT,LAT)
		1DNADR	SPARE
		1DNADR	SPARE
		1DNADR	SPARE
		1DNADR	SPARE
		DNPTR	CMPG2202			# COLLECT SNAPSHOT
		4DNADR	DNTMBUFF			# SEND SNAPSHOT
		DNPTR	CMPG2203			# COMMON DATA
		DNPTR	CMPG2206			# COMMON DATA
		1DNADR	8NN				# 8NN,GARBAGE (PROGRAM NUMBER)
		1DNADR	STATE +10D			# FLAGWRDS 10 AND 11
		3DNADR	RLS				# RLS,+1,...+4,+5 (LANDMARK POSITION)
		1DNADR	SPARE
		1DNADR	OPTMODES			# OPTMODES,HOLDFLAG (OPTICS STATUS)
		DNPTR	CMPG2207			# COMMON DATA
		1DNADR	SPARE				# RESERVED FOR FUTURE USE
		1DNADR	SPARE
		1DNADR	SPARE
		1DNADR	SPARE
		1DNADR	SPARE
		-1DNADR	SPARE				# END OF DOWNLIST

; --------------------- SUB LISTS --------------------------
;
; HYBRID ARCHITECTURE: CMPG22DL follows the same memory optimization strategy as
; CMENTRDL. Six sublists (CMPG2201/02/03/04/06/07) are aliases reusing CMPOWE
; common data structures, but CMPG2205 is a unique sublist capturing spacecraft
; geographic position data specifically needed for landmark tracking correlation.
;
; This demonstrates the AGC's memory conservation philosophy: reuse common telemetry
; structures wherever possible, create unique sublists only for mission-specific
; requirements that don't overlap with other programs.

CMPG2201	EQUALS	CMPOWE01			# COMMON DOWNLIST DATA

CMPG2202	EQUALS	CMPOWE02			# COMMON DOWNLIST DATA

CMPG2203	EQUALS	CMPOWE03			# COMMON DOWNLIST DATA
# Page 180
CMPG2204	EQUALS	CMPOWE04			# COMMON DOWNLIST DATA

; CMPG2205 - UNIQUE SPACECRAFT POSITION SNAPSHOT SUBLIST
;
; This sublist captures the spacecraft's geographic position (longitude, altitude,
; latitude) at the time of landmark sighting. This data enables ground controllers
; to correlate the spacecraft's position with the observed landmark, verifying both
; the spacecraft's trajectory and the accuracy of the landmark's catalogued position.
;
; SNAPSHOT DATA CAPTURED:
; - LONG (2 words): Spacecraft longitude in selenographic coordinates (Moon-fixed frame)
; - ALT (2 words): Spacecraft altitude above lunar reference sphere
; - LAT (2 words): Spacecraft latitude in selenographic coordinates
;
; During Apollo 11's lunar orbit, this position data combined with the landmark
; line-of-sight vectors (in SVMRKDAT) provided complete geometric information for
; navigation triangulation. Ground computers could process multiple landmark sightings
; to refine the spacecraft's state vector with accuracy comparable to Earth-based
; radar tracking.

CMPG2205	-1DNADR	LONG				# LONG,+1			SNAPSHOT DATA
		1DNADR	ALT				# ALT,+1
		-1DNADR	LAT				# LAT,+1

CMPG2206	EQUALS	CMPOWE06			# COMMON DOWNLIST DATA

CMPG2207	EQUALS	CMPOWE07			# COMMON DOWNLIST DATA

; ============================================================================
; TRANSITION: From Mission-Specific Downlists to Downlist Directory
;
; Having defined all five Command Module telemetry downlists (CSM Station-Keeping,
; Entry, Rendezvous, Powered Flight, and Landmark Tracking), we now conclude with
; the downlist directory table (DNTABLE) that provides indexed access to each
; downlist's starting address.
; ============================================================================

# -----------------------------------------------------------
# DOWNLIST DIRECTORY TABLE (DNTABLE)
;
; PURPOSE: DNTABLE serves as the master index for all Command Module telemetry
; downlists defined in this file. The downlink telemetry program (DOWN-TELEMETRY_
; PROGRAM.agc) uses this table to select the appropriate downlist based on the
; current mission program or operational mode.
;
; DIRECTORY STRUCTURE: Each entry contains a GENADR (generalized address) pointing
; to a downlist control list. The order of entries in this table corresponds to
; the downlist ID codes recognized by the telemetry system. When Mission Control
; or the crew changes programs (e.g., from P20 Rendezvous Navigation to P22
; Landmark Tracking), the telemetry system automatically switches downlists by
; indexing into this table.
;
; HISTORICAL CONTEXT: During Apollo 11, the downlink telemetry system cycled through
; these lists automatically as the mission progressed through different phases.
; For example:
; - Translunar coast: CMCSTADL (Station-Keeping)
; - Lunar orbit landmark tracking: CMPG22DL (Landmark Navigation)
; - Rendezvous operations: CMRENDDL (Rendezvous)
; - Transearth injection burn: CMPOWEDL (Powered Flight)
; - Entry interface: CMENTRDL (Entry Monitoring)
;
; This automated downlist selection ensured Mission Control always received the
; most relevant telemetry for the current mission phase without requiring manual
; configuration by the crew.

DNTABLE		GENADR	CMCSTADL			# CSM STATION-KEEPING DOWNLIST
		GENADR	CMENTRDL			# CM ENTRY MONITORING DOWNLIST
		GENADR	CMRENDDL			# CSM RENDEZVOUS DOWNLIST
		GENADR	CMPOWEDL			# CSM POWERED FLIGHT DOWNLIST
		GENADR	CMPG22DL			# P22 LANDMARK TRACKING DOWNLIST

; END OF DOWNLINK_LISTS.agc - All Command Module telemetry downlists defined.

# -----------------------------------------------------------

