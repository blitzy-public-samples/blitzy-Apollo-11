# Copyright:	Public domain.
# Filename:	PINBALL_NOUN_TABLES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	301-319
# Mod history:  2009-05-10 SN   (Sergio Navarro).  Started adapting
#				from the Luminary131/ file of the same
#				name, using Luminary099 page images.
#		2009-06-07 RSB	Corrected two typos.
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
; FILE: PINBALL_NOUN_TABLES.agc
; MODULE: Display and Crew Interface
; MISSION PHASE: all-phases
;
; TL;DR: Defines the complete DSKY noun system (N01-N99) used throughout the
;        Apollo 11 mission. Nouns are typed data containers that specify what
;        information the crew can display or enter via the DSKY (Display and
;        Keyboard). Each noun definition includes memory addresses, display
;        formats, and scaling factors. Critical nouns include N63 (LM altitude
;        and altitude rate for landing), N16 (mission time), N68 (landing site
;        coordinates), and N43 (latitude/longitude/altitude).
;
; COMMENT-ONLY READERS: This file is the "data dictionary" that tells the
;        DSKY what each numbered noun means. When you see crew procedures like
;        "V16N63" (verb 16, noun 63), this file defines noun 63 as showing
;        altitude and descent rate during landing.
;
; CODE-ALONG READERS: Study the table structures (NNADTAB for normal nouns,
;        NNTYPTAB for mixed nouns, IDADDTAB for memory addresses, SFINTAB/
;        SFOUTAB for scaling, RUTMXTAB for format routing). Understanding
;        noun definitions is essential for following display interface logic.
; ============================================================================

# Page 301
; ============================================================================
; NOUN TABLE ENCODING SPECIFICATION
;
; The following sections define how noun data types are encoded in the
; NNTYPTAB (Noun Type Table). Each noun has a type code specifying:
; - Component count (1, 2, or 3 register displays, e.g., time has 3 components)
; - Display mode restrictions (decimal-only vs octal/decimal)
; - Load restrictions (whether crew can ENTER new values via DSKY)
; - Scale factor routines (how to format for display/accept from keyboard)
; ============================================================================
#
# THE FOLLOWING REFERS TO THE NOUN TABLES
#
; ----------------------------------------------------------------------------
; COMPONENT CODE NUMBER ENCODING (Bits 1-5 of noun type code)
; ----------------------------------------------------------------------------
; This code specifies how many data components (registers) the noun displays
; and whether crew input is permitted. Examples:
; - N16 (time) has 3 components: hours, minutes, seconds
; - N63 (altitude/rate) has 2 components: altitude, altitude-rate
; - Single-value nouns have 1 component
; ----------------------------------------------------------------------------
#
# COMPONENT CODE NUMBER		INTERPRETATION
#
#	00000			1 COMPONENT (single register, e.g., N37 single time value)
#	00001			2 COMPONENT (register pair, e.g., N63 altitude and rate)
#	00010			3 COMPONENT (register triplet, e.g., N16 hours/min/sec or N20 IMU angles)
#	X1XXX			BIT 4 = 1.  DECIMAL ONLY (cannot display in octal mode)
#	1XXXX			BIT 5 = 1.  NO LOAD (display-only, crew cannot ENTER values)
# END OF COMPONENT CODE NUMBER
;
; ----------------------------------------------------------------------------
; SF ROUTINE CODE NUMBER ENCODING (Scale Factor / Format Codes)
; ----------------------------------------------------------------------------
; These codes specify how raw AGC memory values are converted for DSKY display
; (OUT direction) and how crew keyboard entries are converted to memory format
; (IN direction). Critical for proper data interpretation during mission phases.
;
; Example: During landing, N63 altitude (feet) and altitude-rate (ft/sec) use
; ARITH DP1 scaling. Armstrong and Aldrin monitored V16N63 continuously during
; final approach, with altitude-rate displayed as XXXXX. ft/sec.
; ----------------------------------------------------------------------------
#
# SF ROUTINE CODE NUMBER	INTERPRETATION
#
# 	00000			OCTAL ONLY (raw octal display, no decimal conversion)
#	00001			STRAIGHT FRACTIONAL (pure fractional values, no scaling)
#	00010			CDU DEGREES (XXX.XX) - IMU gimbal angles in degrees with 2 decimal places
#	00011			ARITHMETIC SF (standard arithmetic scaling)
#	00100			ARITH DP1	OUT (MULT BY 2EXP14 AT END)	IN (STRAIGHT)
#					Double-precision type 1: output scaled by 16384, input direct
#	00101			ARITH DP2	OUT (STRAIGHT)			IN (SL 7 AT END)
#					Double-precision type 2: output direct, input shifted left 7 bits
#	00110			LANDING RADAR POSITION (+0000X) - radar data format with sign display
#	00111			ARITH DP3	OUT (SL 7 AT END)		IN (STRAIGHT)
#					Double-precision type 3: output shifted left 7, input direct
#	01000			WHOLE HOURS IN R1, WHOLE MINUTES (MOD 60) IN R2,
#					SECONDS (MOD 60) 0XX.XX IN R3.  *** ALARMS IF USED WITH OCTAL
#					Time format: HH:MM:SS.SS (e.g., N16 mission elapsed time)
#	01001			MINUTES (MOD 60) IN D1D2, D3 BLANK, SECONDS (MOD 60) IN D4D5
#					LIMITS TO 59B59 IF MAG EXCEEDS THIS VALUE.
#					ALARMS IF USED WITH OCTAL ******** IN (ALARM)
#					Countdown format: MM_SS (e.g., N36 time-from-ignition T-minus)
#	01010			ARITH DP4	OUT (STRAIGHT)			IN (SL 3 AT END)
#					Double-precision type 4: output direct, input shifted left 3 bits
#	01011			ARITH1 SF	OUT (MULT BY 2EXP14 AT END)	IN (STRAIGHT)
#					Arithmetic type 1: output scaled by 16384, input direct
#	01100			2 INTEGERS IN D1D2, D4D5, D3 BLANK.
#					ALARMS IF USED WITH OCTAL ******** IN (ALARM)
#					Integer pair display (center digit blank for separation)
#	01101			360-CDU DEGREES (XXX.XX) - full-circle gimbal angles (0-360 degrees)
#
# END OF SF ROUTINE CODE NUMBERS

; ----------------------------------------------------------------------------
; SF CONSTANT CODE NUMBER (Display Format and Units Specification)
; ----------------------------------------------------------------------------
; These codes define physical units, display precision, and value ranges for
; noun data types. Each code maps to specific scaling routines (ARITH, ARITHDP1,
; etc.) that convert between AGC internal representation and human-readable
; display formats on the DSKY seven-segment displays.
;
; MISSION CONTEXT: During Apollo 11 lunar descent, critical nouns included:
; - N63: Altitude (01110) and altitude-rate (10000) - monitored continuously
; - N68: Landing site latitude/longitude - used for targeting Sea of Tranquility
; - N60: Forward velocity (10001) - used during manual site selection phase
; ----------------------------------------------------------------------------
#
# SF CONSTANT CODE NUMBER	INTERPRETATION
#
#	00000			WHOLE				USE ARITH (integer values, no decimals)
#	00000			DP TIME SEC (XXX.XX SEC)	USE ARITHDP1 (time in seconds with centiseconds)
#	00000			LR POSITION (+0000X)		USE LR POSITION (landing radar position format)
#	00001			SPARE (unused code, reserved for future expansion)
#	00010			CDU DEGREES			USE CDU DEGREES (gimbal angles, standard range)
#	00010			360-CDU DEGREES			USE 360-CDU DEGREES (full-circle gimbal range)
#	00011			DP DEGREES (90 XX.XXX DEG	USE ARITHDP3 (high-precision angles)
#	00100			DP DEGREES (360) XXX.XX DEG	USE ARITHDP4 (full-circle with decimals)
#	00101			DEGREES (180) XXX.XX DEG	USE ARITH (half-circle angle range)
#	00101			OPTICAL TRACKER AZIMUTH ANGLE (XXX.XXDEG)
#								USE ARITHDP1 (AOT sighting angles)
#	00110			WEIGHT2 (XXXXX. LBS)		USE ARITH1 (spacecraft mass in pounds)
# Page 302
#	00111			POSITION5 (XXX.XX NAUTICAL MILES)
#								USE ARITHDP3 (position with high precision)
#	01000			POSITION4 (XXXX.X NAUTICAL MILES)
#								USE ARITHDP3 (position moderate precision)
#	01001			VELOCITY2 (XXXXX. FT/SEC)	USE ARITHDP4 (velocity integer ft/sec)
#	01010			VELOCITY3 (XXXX.X FT/SEC)	USE ARITHDP3 (velocity with 1 decimal)
#	01011			ELEVATION DEGREES (89.999 MAX)	USE ARITH (elevation angle, limited range)
#	01100			RENDEZVOUS RADAR RANGE (XXX.XX NAUT MI)
#								USE ARITHDP1 (RR range to CSM in nautical miles)
#	01101			RENDEZVOUS RADAR RANGE RATE (XXXXX.FT/SEC)
#								USE ARITHDP1 (RR closing velocity)
#	01110			LANDING RADAR ALTITUDE (XXXXX.FEET)
#								USE ARITHDP1 (LR altitude above surface)
#								CRITICAL for P63/P64 landing programs
#	01111			INITIAL/FINAL ALTITUDE (XXXXX. FEET)
#								USE ARITHDP1 (altitude targets/displays)
#	10000			ALTITUDE RATE (XXXXX.FT/SEC)	USE ARITH (vertical descent rate)
#								CRITICAL: Armstrong monitored this in N63 during final approach
#	10001			FORWARD/LATERAL VELOCITY (XXXXX.FEET/SEC)
#								USE ARITH (horizontal velocity components)
#								Used during manual landing site selection
#	10010			ROTATIONAL HAND CONTROLLER ANGLE RATES
#					XXXXX.DEG/SEC		USE ARITH (attitude control rates)
#	10011			LANDING RADAR VELX (XXXXX.FEET/SEC)
#								USE ARITHDP1
#	10100			LANDING RADAR VELY (XXXXX.FEET/SEC)
#								USE ARITHDP1
#	10101			LANDING RADAR VELZ (XXXXX.FEET/SEC)
#								USE ARITHDP1
#	10110			POSITION7 (XXXX.X NAUT MI)	USE ARITHDP4
#	10111			TRIM DEGREES2 (XXX.XX DEG)	USE ARITH
#	11000			COMPUTED ALTITUDE (XXXXX. FEET)
#								USE ARITHDP1
#	11001			DP DEGREES (XXXX.X DEG)		USE ARITHDP3
#	11010			POSITION9 (XXXX.X FT)		USE ARITHDP3
#	11011			VELOCITY4 (XXXX.X FT/SEC)	USE ARITHDP2
#	11100			RADIANS (XXX.XXX RADIANS)	USE ARITHDP4
#
# END OF SF CONSTANT CODE NUMBERS

# FOR GREATER THAN SINGLE PRECISION SCALES, PUT ADDRESS OF MAJOR PART INTO
# NOUN TABLES.

# OCTAL LOADS PLACE +0 INTO MAJOR PART, DATA INTO MINOR PART.

# OCTAL DISPLAYS SHOW MINOR PART ONLY.

# TO GET AT BOTH MAJOR AND MINOR PARTS (IN OCTAL), USE NOUN 01.

# A NOUN MAY BE DECLARED :DECIMAL ONLY: BY MAKING BIT4=1 OF ITS COMPONENT
# CODE NUMBER.  IF THIS NOUN IS USED WITH ANY OCTAL DISPLAY VERB, OR IF
# DATA IS LOADED IN OCTAL, IT ALARMS.


# IN LOADING AN :HOURS, MINUTES, SECONDS: NOUN, ALL 3 WORDS MUST BE
# LOADED, OR ALARM.

# Page 303

# ALARM IF AN ATTEMPT IS MADE TO LOAD :SPLIT MINUTES/SECONDS: (MMBSS).
# THIS IS USED FOR DISPLAY ONLY.

# Page 304
# THE FOLLOWING ROUTINES ARE FOR READING THE NOUN TABLES AND THE SF TABLES
# (WHICH ARE IN A SEPARATE BANK FROM THE REST OF PINBALL).  THESE READING
# ROUTINES ARE IN THE SAME BANK AS THE TABLES.  THEY ARE CALLED BY DXCH Z.

# LODNNTAB LOADS NNADTEM WITH THE NNADTAB ENTRY, NNTYPTEM WITH THE
# NNTYPTAB ENTRY.  IF THE NOUN IS MIXED, IDADITEM IS LOADED WITH THE FIRST
# IDADDTAB ENTRY, IDAD2TEM THE SECOND IDADDTAB ENTRY, IDAD3TEM THE THIRD
# IDADDTAB ENTRY, RUTMXTEM WITH THE RUTMXTAB ENTRY.  MIXBR IS SET FOR
# MIXED OR NORMAL NOUN.

;
; ============================================================================
; NOUN TABLE ACCESS ROUTINES
;
; The following subroutines retrieve noun definitions from the tables stored
; in Bank 6. During Apollo 11's descent and landing, these routines were
; called continuously to update DSKY displays showing critical flight data.
;
; LODNNTAB: Master noun table lookup routine
;   - Retrieves noun type and format from NNADTAB/NNTYPTAB
;   - For mixed nouns (N40-N99), also retrieves:
;     * Memory addresses from IDADDTAB (where data is stored)
;     * Scale factor codes from RUTMXTAB (how to convert data)
;   - Sets MIXBR flag indicating normal (nouns 00-39) or mixed (nouns 40-99)
;
; GTSFOUT: Retrieves output scale factor table entries (SFOUTAB)
;   - Determines how to format data for DSKY display
;   - Example: Altitude scaled from internal meters to displayed feet
;
; GTSFIN: Retrieves input scale factor table entries (SFINTAB)
;   - Determines how to convert crew keyboard entries to internal format
;   - Example: Crew enters degrees, stored as scaled binary angle units
;
; Historical Context: During the landing sequence (102:33-102:45 MET),
; Armstrong and Aldrin monitored N63 (altitude/altitude-rate) using V16N63.
; These routines executed every display update cycle, fetching the noun
; definition, retrieving altitude data from landing radar memory locations,
; applying scale factors to convert internal representation to feet and
; feet/second, and formatting for the seven-segment DSKY displays.
; ============================================================================
;

		BANK	6
		SETLOC	PINBALL3
		BANK
		COUNT*	$$/NOUNS
LODNNTAB	DXCH	IDAD2TEM		# SAVE RETURN INFO IN IDAD2TEM, IDAD3TEM.
		INDEX	NOUNREG
		CAF	NNADTAB
		TS	NNADTEM
		INDEX	NOUNREG
		CAF	NNTYPTAB
		TS	NNTYPTEM
		CS	NOUNREG
		AD	MIXCON
		EXTEND
		BZMF	LODMIXNN		# NOUN NUMBER G/E FIRST MIXED NOUN
		CAF	ONE			# NOUN NUMBER L/ FIRST MIXED NOUN
		TS	MIXBR			# NORMAL.  +1 INTO MIXBR
		TC	LODNLV
LODMIXNN	CAF	TWO			# MIXED.  +2 INTO MIXBR.
		TS	MIXBR
		INDEX	NOUNREG
		CAF	RUTMXTAB -40D		# FIRST MIXED NOUN = 40.
		TS	RUTMXTEM
		CAF	LOW10
		MASK	NNADTEM
		TS	Q			# TEMP
		INDEX	A
		CAF	IDADDTAB
		TS	IDAD1TEM		# LOAD IDAD1TEM	WITH FIRST IDADDTAB ENTRY
		EXTEND
		INDEX	Q			# LOAD IDAD2TEM WITH 2ND IDADDTAB ENTRY
		DCA	IDADDTAB +1		# LOAD IDAD3TEM WITH 3RD IDADDTAB ENTRY.
LODNLV		DXCH	IDAD2TEM		# PUT RETURN INFO INTO A, L.
		DXCH	Z

MIXCON		=	OCT50			# (DEC 40)

# GTSFOUT LOADS SFTEMP1, SFTEMP2 WTIH THE DP SFOUTAB ENTRIES.

GTSFOUT		DXCH	SFTEMP1			# 2X(SFCONUM) ARRIVES IN SFTEMP1.
# Page 305
		EXTEND
		INDEX	A
		DCA	SFOUTAB
SFCOM		DXCH	SFTEMP1
		DXCH	Z

# GTSFIN LOADS SFTEMP1, SFTEMP2 WITH THE DP SFINTAB INTRIES.

GTSFIN		DXCH	SFTEMP1			# 2X(SFCONUM) ARIVES IN SFTEMP1.
		EXTEND
		INDEX	A
		DCA	SFINTAB
		TCF	SFCOM

;
; ============================================================================
; NNADTAB - NORMAL NOUN ADDRESS TABLE (Nouns 00-39)
;
; This table defines "normal" nouns where the memory address is directly
; encoded in the table entry. Each entry contains either:
;   - An ECADR (erasable address) pointing to data in erasable memory
;   - An octal code with special meaning (e.g., 77776 = channel I/O)
;   - An octal code for machine-address-specification nouns (40000)
;
; Normal nouns are used for frequently accessed data with fixed memory
; locations. For nouns requiring more complex addressing (different
; scale factors, multiple component types), see NNTYPTAB below.
;
; Historical Note: During Apollo 11's landing, crew primarily used mixed
; nouns (40-99) for flight data, but normal nouns like N01-N03 (machine
; address specification) were used during pre-flight checkout and N16
; (time of event) was displayed throughout the mission.
; ============================================================================
;

						# NN 	NORMAL NOUNS
NNADTAB		OCT	00000			# 00 	NOT IN USE
		OCT	40000			# 01 	SPECIFY MACHINE ADDRESS (FRACTIONAL)
		OCT	40000			# 02 	SPECIFY MACHINE ADDRESS (WHOLE)
		OCT	40000			# 03	SPECIFY MACHINE ADDRESS (DEGREES)
		ECADR	DSPTEM1			# 04	ANGULAR ERROR/DIFFERENCE
		ECADR	DSPTEM1			# 05 	ANGULAR ERROR/DIFFERENCE
		ECADR	OPTION1			# 06	OPTION CODE
		ECADR	XREG			# 07	ECADR OF WORD TO BE MODIFIED
						#	ONES FOR BITS TO BE MODIFIED
						#	1 TO SET OR 0 TO RESET SELECTED BITS
		ECADR	ALMCADR			# 08 	ALARM DATA
		ECADR	FAILREG			# 09	ALARM CODES
		OCT	77776			# 10	CHANNEL TO BE SPECIFIED
		ECADR	TCSI			# 11	TIG OF CSI (HRS,MIN,SEC)
		ECADR	OPTIONX			# 12	OPTION CODE
						#	(USED BY EXTENDED VERBS ONLY)
		ECADR	TCDH			# 13	TIG OF CDH (HRS,MIN,SEC)
		ECADR	DSPTEMX			# 14	CHECKLIST
						#	(USED BY EXTENDED VERBS ONLY)
		OCT	77777			# 15	INCREMENT MACHINE ADDRESS
		ECADR	DSPTEMX			# 16	TIME OF EVENT (HRS,MIN,SEC)
;		^ N16: Displays mission event times in HH:MM:SS format
;		  Used throughout mission for displaying burn times, ignition times
		OCT	00000			# 17	SPARE
		ECADR	FDAIX			# 18	AUTO MANEUVER BALL ANGLES
		OCT	00000			# 19	SPARE
		ECADR	CDUX			# 20	ICDU ANGLES
;		^ N20: Displays IMU gimbal angles (inner, middle, outer CDU angles)
;		  Critical for monitoring IMU orientation during mission
		ECADR	PIPAX			# 21	PIPAS
;		^ N21: Displays PIPA (accelerometer) readings
;		  Pulsed Integrating Pendulous Accelerometers measured vehicle acceleration
		ECADR	THETAD			# 22	NEW ICDU ANGLES
		OCT	00000			# 23	SPARE
		ECADR	DSPTEM2 +1		# 24	DELTA TIME FOR AGC CLOCK (HRS,MIN,SEC)
		ECADR	DSPTEM1			# 25	CHECKLIST
						#	(USED WTIH PLEASE PERFORM ONLY)
		ECADR	DSPTEM1			# 26	PRIO/DELAY, ADRES, BBCON
		ECADR	SMODE			# 27	SELF TEST ON/OFF SWITCH
# Page 306
		OCT	00000			# 28	SPARE
		OCT	00000			# 29	SPARE
		OCT	0			# 30	SPARE
		OCT	0			# 31	SPARE
		ECADR	-TPER			# 32	TIME TO PERIGEE (HRS,MIN,SEC)
		ECADR	TIG			# 33	TIME OF IGNITION (HRS,MIN,SEC)
;		^ N33: TIG - Time of Ignition for upcoming burn
;		  Critical for mission planning and engine start sequencing
		ECADR	DSPTEM1			# 34	TIME OF EVENT (HRS,MIN,SEC)
		ECADR	TTOGO			# 35	TIME TO GO TO EVENT (HRS,MIN,SEC)
		ECADR	TIME2			# 36	TIME OF AGC CLOCK (HRS,MIN,SEC)
;		^ N36: Mission Elapsed Time (MET) display in hours:minutes:seconds
;		  During Apollo 11 descent, MET showed 102:33 at PDI, 102:45 at touchdown
		ECADR	TTPI			# 37	TIG OF TPI (HRS,MIN,SEC)
;		^ N37: Time of ignition for Terminal Phase Initiation (rendezvous maneuver)
		ECADR	TET			# 38	TIME OF STATE BEING INTEGRATED
		OCT	00000			# 39	SPARE

# END OF NNADTAB FOR NORMAL NOUNS

;
; ============================================================================
; NNTYPTAB - MIXED NOUN TYPE TABLE (Nouns 40-99)
;
; Mixed nouns have components with different scale factors or formats.
; Each octal entry encodes:
;   Bits 1-2: Number of components (00=1, 01=2, 10=3)
;   Bits 3-7: Scale factor routine code for component 1
;   Bits 8-12: Scale factor routine code for component 2
;   Bits 13-17: Scale factor routine code for component 3
;
; This table enables complex data displays like N43 (lat/long/alt) where
; each component has different units and scaling, or N63 (altitude/altitude-rate)
; critical during Apollo 11's landing sequence.
;
; Historical Note: During Apollo 11 descent, Armstrong and Aldrin monitored
; V16N63 (altitude and altitude rate) continuously. The famous "1202 alarm"
; occurred while the AGC was processing these displays along with landing
; radar data. V06N62 displayed landing radar velocity data during final
; approach when Armstrong took semi-manual control at ~500 feet altitude.
; ============================================================================
;

						# NN	MIXED NOUNS
		OCT	64000			# 40	TIME TO IGNITION/CUTOFF
						#	VG
						#	DELTA V (ACCUMULATED)
		OCT	02003			# 41	TARGET	AZIMUTH
						#		ELEVATION
		OCT	24006			# 42	APOGEE
						#	PERIGEE
						#	DELTA V (REQUIRED)
		OCT	24011			# 43	LATITUDE
						#	LONGITUDE
						#	ALTITUDE
;		^ N43: Position display - latitude (deg), longitude (deg), altitude (nmi)
;		  Used to monitor spacecraft position relative to Earth or lunar surface
;		  Landing site coordinates: Sea of Tranquility, 0.67°N, 23.5°E
		OCT	64014			# 44	APOGEE
						#	PERIGEE
						#	TFF
		OCT	64017			# 45	MARKS
						#	TTI OF NEXT BURN
						#	MGA
		OCT	00022			# 46	AUTOPILOT CONFIGURATION
		OCT	22025			# 47	LEM WEIGHT
						#	CSM WEIGHT
		OCT	22030			# 48	GIMBAL PITCH TRIM
						#	GIMBAL ROLL TRIM
		OCT	24033			# 49	DELTA R
						#	DELTA V
						#	RADAR DATA SOURCE CODE
		OCT	0			# 50	SPARE
		OCT	22041			# 51	S-BAND ANTENNA	PITCH
						#			YAW
		OCT	00044			# 52	CENTRAL ANGLE OF ACTIVE VEHICLE
		OCT	00000			# 53	SPARE
		OCT	24052			# 54	RANGE
						#	RANGE RATE
						#	THETA
		OCT	24055			# 55	NO. OF APSIDAL CROSSINGS
# Page 307
						#	ELEVATION ANGLE
						#	CENTRAL ANGLE
		OCT	02060			# 56	RR LOS	AZIMUTH
						#		ELEVATION
		OCT	20063			# 57	DELTA R
		OCT	24066			# 58	PERIGEE ALT
						#	DELTA V TPI
						#	DELTA V TPF
		OCT	24071			# 59	DELTA VELOCITY LOS
		OCT	24074			# 60	HORIZONTAL VELOCITY
						#	ALTITUDE RATE
						#	COMPUTED ALTITUDE
		OCT	64077			# 61	TIME TO GO IN BRAKING PHASE
						#	TIME TO IGNITION
						#	CROSS RANGE DISTANCE
		OCT	64102			# 62	ABSOLUTE VALUE OF VELOCITY
						#	TIME TO IGNITION
						#	DELTA V (ACCUMULATED)
;		^ N62: Velocity magnitude display (used during powered flight)
		OCT	24105			# 63	ABSOLUTE VALUE OF VELOCITY
						#	ALTITUDE RATE
						#	COMPUTED ALTITUDE
;		^ N63: CRITICAL LANDING DISPLAY - Velocity, altitude rate, altitude
;		  V16N63 was THE primary display during Apollo 11 descent
;		  Armstrong and Aldrin continuously monitored this during the
;		  final 12 minutes of powered descent from PDI to touchdown.
;		  At 102:45:40 MET, N63 showed near-zero velocity at touchdown.
		OCT	64110			# 64	TIME LEFT FOR REDESIGNATION -- LPD ANGLE
						#	ALTITUDE RATE
						#	COMPUTED ALTITUDE
;		^ N64: Landing Point Designator (LPD) angle - used during approach phase
;		  When Armstrong took semi-manual control at ~500 feet, the LPD angle
;		  helped him visually designate the landing site to avoid boulder field
		OCT	24113			# 65	SAMPLED AGC TIME (HRS,MIN,SEC)
						#	(FETCHED IN INTERRUPT)
		OCT	62116			# 66	LR	RANGE
						#		POSITION
		OCT	04121			# 67	LRVX
						#	LRVY
						#	LRVZ
		OCT	64124			# 68	SLANT RANGE TO LANDING SIGHT
						#	TIME TO GO IN BRAKING PHASE
						#	LR ALTITUDE -- COMPUTED ALTITUDE
;		^ N68: Landing site targeting data - slant range, time-to-go, altitude
;		  Used by guidance equations to compute trajectory to target landing site
;		  Apollo 11 target: Sea of Tranquility, manually adjusted during descent
		OCT	00000			# 69	SPARE
		OCT	04132			# 70	AOT DETENT CODE/STAR CODE
		OCT	04135			# 71	AOT DETENT CODE/STAR CODE
		OCT	02140			# 72	RR	360 -- TRUNNION ANGLE
						#		SHAFT ANGLE
		OCT	02143			# 73	NEW RR	360 -- TRUNNION ANGLE
						#		SHAFT ANGLE
		OCT	64146			# 74	TIME TO IGNITION
						#	YAWAFTER VEHICLE RISE
						#	PITCH AFTER VEHICLE RISE
		OCT	64151			# 75	DELTA ALTITUDE CDH
						#	DELTA TIME (CDH-CSI OR TPI-CDH)
						#	DELTA TIME (TPI-CDH OR TPI-NOMTPI)
		OCT	24154			# 76	DESIRED HORIZONTAL VELOCITY
						#	DESIRED RADIAL VELOCITY
						#	CROSS-RANGE DISTANCE
;		^ N76: Desired velocity components for rendezvous guidance targeting
# Page 308
		OCT	62157			# 77	TIME TO ENGINE CUTOFF
						#	VELOCITY NORMAL TO CSM PLANE
;		^ N77: Engine burn monitoring - time to cutoff, velocity out-of-plane
		OCT	02162			# 78	RR	RANGE
						#		RANGE RATE
;		^ N78: Rendezvous Radar data - range to CSM, closing rate
;		  Used during LM ascent and rendezvous with Columbia in lunar orbit
		OCT	24165			# 79	CURSOR ANGLE
						#	SPIRAL ANGLE
						#	POSITION CODE
		OCT	02170			# 80	DATA INDICATOR
						#	OMEGA
		OCT	24173			# 81	DELTA V (LV)
;		^ N81: Delta-V in Local Vertical frame - burn planning display
		OCT	24176			# 82	DELTA V (LV)
;		^ N82: Delta-V in Local Vertical frame - alternate format
		OCT	24201			# 83	DELTA V (BODY)
;		^ N83: Delta-V in spacecraft body coordinates
		OCT	24204			# 84	DELTA V (OTHER VEHICLE)
;		^ N84: Delta-V relative to target vehicle (CSM during rendezvous)
		OCT	24207			# 85	VG (BODY)
;		^ N85: Velocity to be gained (VG) in body frame - burn monitoring
		OCT	24212			# 86	VG (LV)
;		^ N86: Velocity to be gained in Local Vertical frame
		OCT	02215			# 87	BACKUP OPTICS LOS	AZIMUTH
						#				ELEVATION
		OCT	24220			# 88	HALF UNIT SUN OR PLANET VECTOR
;		^ N88: Unit vector to celestial body - star/planet tracking
		OCT	24223			# 89	LANDMARK	LATITUDE
						#			LONGITUDE/2
						#			ALTITUDE
;		^ N89: Landmark position - ground site tracking and navigation updates
		OCT	24226			# 90	Y
						#	Y DOT
						#	PSI
;		^ N90: AGS (Abort Guidance System) state - out-of-plane position/velocity/angle
		OCT	04231			# 91	ALTITUDE
						#	VELOCITY
						#	FLIGHT PATH ANGLE
;		^ N91: AGS flight profile data - backup guidance system monitoring
		OCT	00000			# 92	SPARE
		OCT	04237			# 93	DELTA GYRO ANGLES
		OCT	00000			# 94	SPARE
		OCT	0			# 95	SPARE
		OCT	0			# 96	SPARE
		OCT	04253			# 97	SYSTEM TEST INPUTS
;		^ N97: System self-test input values for AGC diagnostics
		OCT	04256			# 98	SYSTEM TEST RESULTS
;		^ N98: System self-test output results - hardware verification
		OCT	24261			# 99	RMS IN POSITION
						#	RMS IN VELOCITY
						#	RMS IN BIAS
;		^ N99: Root-Mean-Square navigation errors - accuracy monitoring
;		  RMS values quantify uncertainty in position, velocity, and IMU bias

# END OF NNADTAB FOR MIXED NOUNS

						# NN	NORMAL NOUNS
NNTYPTAB	OCT	00000			# 00	NOT IN USE
		OCT	04040			# 01	3COMP FRACTIONAL
		OCT	04140			# 02	3COMP WHOLE
		OCT	04102			# 03	3COMP CDU DEGREES
		OCT	00504			# 04	1COMP DPDEG(360)
		OCT	00504			# 05	1COMP DPDEG(360)
		OCT	04000			# 06	3COMP OCTAL ONLY
		OCT	04000			# 07 	3COMP OCTAL ONLY
		OCT	04000			# 08	3COMP OCTAL ONLY
# Page 309
		OCT	04000			# 09	3COMP OCTAL ONLY
		OCT	00000			# 10	1COMP OCTAL ONLY
		OCT	24400			# 11	3COMP HMS (DEC ONLY)
		OCT	02000			# 12	2COMP OCTAL ONLY
		OCT	24400			# 13	3COMP HMS (DEC ONLY)
		OCT	04140			# 14	3COMP WHOLE
		OCT	00000			# 15	1COMP OCTAL ONLY
		OCT	24400			# 16	3COMP HMS (DEC ONLY)
		OCT	0			# 17 	SPARE
		OCT	04102			# 18	3COMP CDU DEG
		OCT	00000			# 19	SPARE
		OCT	04102			# 20	3COMP CDU DEGREES
		OCT	04140			# 21	3COMP WHOLE
		OCT	04102			# 22	3COMP CDU DEGREES
		OCT	00000			# 23 	SPARE
		OCT	24400			# 24	3COMP HMS (DEC ONLY)
		OCT	04140			# 25	3COMP WHOLE
		OCT	04000			# 26	3COMP OCTAL ONLY
		OCT	00140			# 27	1COMP WHILE
		OCT	00000			# 28	SPARE
		OCT	00000			# 29	SPARE
		OCT	0			# 30	SPARE
		OCT	0			# 31	SPARE
		OCT	24400			# 32 	3COMP HMS (DEC ONLY)
		OCT	24400			# 33 	3COMP HMS (DEC ONLY)
		OCT	24400			# 34	3COMP HMS (DEC ONLY)
		OCT	24400			# 35 	3COMP HMS (DEC ONLY)
		OCT	24400			# 36 	3COMP HMS (DEC ONLY)
		OCT	24400			# 37 	3COMP HMS (DEC ONLY)
		OCT	24400			# 38 	3COMP HMS (DEC ONLY)
		OCT	00000			# 39	SPARE

# END OF NNTYPTAB FOR NORMAL NOUNS

						# NN	MIXED NOUNS
		OCT	24500			# 40	3COMP	MIN/SEC, VEL3, VEL3
						#		(NO LOAD, DEC ONLY)
		OCT	00542			# 41	2COMP	CDU DEG, ELEV DEG
		OCT	24410			# 42	3COMP	POS4, POS4, VEL3
						#		(DEC ONLY)
		OCT	20204			# 43	3COMP	DPDEG(360), DPDEG(360) POS4
						#		(DEC ONLY)
		OCT	00410			# 44	3COMP	POS4, POS4, MIN/SEC
						#		(NO LOAD, DEC ONLY)
		OCT	10000			# 45	3COMP	WHOLE, MIN/SEC, DPDEG(360)
						#		(NO LOAD, DEC ONLY)
		OCT	00000			# 46	1COMP 	OCTAL ONLY
		OCT	00306			# 47	2COMP	WEIGHT2 FOR EACH
						#		(DEC ONLY)
		OCT	01367			# 48	2COMP	TRIM DEG2 FOR EACH
# Page 310
						#		(DEC ONLY)
		OCT	00510			# 49	3COMP	POS4, VEL3, WHOLE
						#		(DEC ONLY)
		OCT	0			# 50	SPARE
		OCT	00204			# 51	2COMP	DPDEG(360), DPDEG(360)
						#		(DEC ONLY)
		OCT	00004			# 52	1COMP	DPDEG(360)
		OCT	00000			# 53	SPARE
		OCT	10507			# 54	3COMP	POS5, VEL3, DPDEG(360)
						#		(DEC ONLY)
		OCT	10200			# 55	3COMP	WHOLE, DPDEG(360), DPDEG(360)
						#		(DEC ONLY)
		OCT	00204			# 56	2COMP	DPDEG(360), DPDEG(360)
		OCT	00010			# 57	1COMP	POS4
						#		(DEC ONLY)
		OCT	24510			# 58	3COMP	POS4, VEL3, VEL3
						#		(DEC ONLY)
		OCT	24512			# 59	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
		OCT	60512			# 60	3COMP	VEL3, VEL3, COMP ALT
						# 		(DEC ONLY)
		OCT	54000			# 61	3COMP 	MIN/SEC, MIN/SEC, POS7
						#		(NO LOAD, DEC ONLY)
		OCT	24012			# 62	3COMP	VEL3, MIN/SEC, VEL3
						#		(NO LOAD, DEC ONLY)
		OCT	60512			# 63	3COMP	VEL3, VEL3, COMP ALT
						#		(DEC ONLY)
		OCT	60500			# 64	3COMP	2INT, VEL3, COMP ALT
						#		(NO LOAD, DEC ONLY)
		OCT	00000			# 65	3COMP	HMS (DEC ONLY)
		OCT	00016			# 66	2COMP 	LANDING RADAR ALT, POSITION
						#		(NO LOAD, DEC ONLY)
		OCT	53223			# 67	3COMP	LANDING RADAR VELX, Y, Z
		OCT	60026			# 68	3COMP	POS7, MIN/SEC, COMP ALT
						#		(NO LOAD, DEC ONLY)
		OCT	00000			# 69	SPARE
		OCT	0			# 70	3COMP	OCTAL ONLY FOR EACH
		OCT	0			# 71	3COMP	OCTAL ONLY FOR EACH
		OCT	00102			# 72	2COMP	360-CDU DEG, CDU DEG
		OCT	00102			# 73	2COMP	360-CDU DEG, CDU DEG
		OCT	10200			# 74	3COMP	MIN/SEC, DPDEG(360), DPDEG(360)
						#		(NO LOAD, DEC ONLY)
		OCT	00010			# 75	3COMP	POS4, MIN/SEC, MIN/SEC
						#		(NO LOAD, DEC ONLY)

		OCT	20512			# 76	3COMP	VEL3, VEL3, POS4
						#		(DEC ONLY)
		OCT	00500			# 77	2COMP	MIN/SEC, VEL3
						#		(NO LOAD, DEC ONLY)
		OCT	00654			# 78	2 COMP	RR RANGE, RR RANGE RATE
		OCT	00102			# 79	3COMP	CDU DEG, CDU DEG, WHOLE
# Page 311
						#		(DEC ONLY)
		OCT	00200			# 80	2COMP	WHOLE, DPDEG(360)
		OCT	24512			# 81	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
		OCT	24512			# 82	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
		OCT	24512			# 83	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
		OCT	24512			# 84	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
		OCT	24512			# 85	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
		OCT	24512			# 86	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
		OCT	00102			# 87	2COMP	CDU DEG FOR EACH
		OCT	0			# 88	3COMP	FRAC FOR EACH
						#		(DEC ONLY)
		OCT	16143			# 89	3COMP 	DPDEG(90), DPDEG(90), POS5
						#		(DEC ONLY)
		OCT	10507			# 90	3COMP	POS5, VEL3, DEPDEG(360)
						#		(DEC ONLY)
		OCT	10450			# 91	3COMP	POS4, VEL2, DPDEG(360)
		OCT	00000			# 92	SPARE
		OCT	06143			# 93	3COMP	DPDEG(90) FOR EACH
		OCT	00000			# 94	SPARE
		OCT	0			# 95	SPARE
		OCT	0			# 96	SPARE
		OCT	00000			# 97	3COMP	WHOLE FOR EACH
		OCT 	00000			# 98	3COMP	WHOLE, FRAC, WHOLE
		OCT	71572			# 99	3COMP	POS9, VEL4, RADIANS
						#		(DEC ONLY)

# END OF NNTYPTAB FOR MIXED NOUNS

SFINTAB		OCT	00006			# WHOLE, DP TIME (SEC)
		OCT	03240
		OCT	00000			# SPARE
		OCT	00000
		OCT	00000			# CDU DEGREES, 360-CDU DEGREES
		OCT	00000			#	(SFCONS IN DEGINSF)
		OCT	10707			# DP DEGREES (90)
		OCT	03435			#	UPPED BY 1
		OCT	13070			# DP DEGREES (360) (POINT BETWN BITS 11-12)
		OCT	34345			#	UPPED BY 1
		OCT	00005			# DEGREES (180)
		OCT	21616
		OCT	26113			# WEIGHT2
		OCT	31713
		OCT	00070			# POSITION5
		OCT	20460
# Page 312
		OCT	01065			# POSITION4
		OCT	05740
		OCT	11414			# VELOCITY2 	(POINT BETWN BITS 11-12)
		OCT	31463
		OCT	07475			# VELOCITY3
		OCT	16051
		OCT	00001			# ELEVATION DEGREES
		OCT	03434
		OCT	00047			# RENDEZVOUS RADAR RANGE
		OCT	21135
		OCT	77766			# RENDESVOUS RADAR RANGE RATE
		OCT	50711
		2DEC*	.9267840599 E5 B-28*	# LANDING RADAR ALTITUDE

		OCT	00002			# INITIAL/FINAL ALTITUDE
		OCT	23224
		OCT	00014			# ALTITUDE RATE
		OCT	06500
		OCT	00012			# FORWARD/LATERAL VELOCITY
		OCT	36455
		OCT	04256			# ROT HAND CONT ANGLE RATE
		OCT	07071
		2DEC*	-1.552795030 E5 B-28*	# LANDING RADAR VELX

		2DEC*	.8250825087 E5 B-28*	# LANDING RADAR VELY

		2DEC*	1.153668673 E5 B-28*	# LANDING RADAR VELZ

		OCT	04324			# POSITION7
		OCT	27600
		OCT	00036			# TRIM DEGREES2
		OCT	20440
		OCT	00035			# COMPUTED ALTITUDE
		OCT	30400
		OCT	23420			# DP DEGREES
		OCT	00000
		2DEC	30480 B-19		# POSITION 9

		2DEC	30.48 B-7		# VELOCITY4

		2DEC	100 B-8			# RADIANS

						# END OF SFINTAB

SFOUTAB		OCT	05174			# WHOLE, DP TIME (SEC)
		OCT	13261
		OCT	00000			# SPARE
		OCT	00000
		OCT	00000			# CDU DEGREES, 360-CDU DEGREES
# Page 313
		OCT	00000			#	(SFCONS IN DEGOUTSF, 360 CDUO)
		OCT	00714			# DP DEGREES (90) (POINT BETWN BITS 7-8)
		OCT	31463
		OCT	13412			# DP DEGREES (360)
		OCT	07534
		OCT	05605			# DEGREES (180)
		OCT	03656
		OCT	00001			# WEIGHT2
		OCT	16170
		OCT	00441			# POSITION5
		OCT	34306
		OCT	07176			# POSITION4	(POINT BETWN BITS 7-8)
		OCT	21603
		OCT	15340			# VELOCITY2
		OCT	15340
		OCT	01031			# VELOCITY3	(POINT BETWN BITS 7-8)
		OCT	21032
		OCT	34631			# ELEVATION DETREES
		OCT	23146
		OCT	00636			# RENDEZVOUS RADAR RANGE
		OCT	14552
		OCT	74552			# RENDEZVOUS RADAR RANGE RATE
		OCT	70307
		2DEC	1.079 E-5 B14		# LANDING RADAR ALTITUDE

		OCT	14226			# INITIAL/FINAL ALTITUDE
		OCT	31757
		OCT	02476			# ALTITUDE RATE
		OCT	05531
		OCT	02727			# FORWARD/LATERAL VELOCTY
		OCT	16415
		OCT	00007			# ROT HAND CONT ANGLE RATE
		OCT	13734
		2DEC	-.6440 E-5 B14		# LANDING RADAR VELX

		2DEC	1.212 E-5 B14		# LANDING RADAR VELY

		2DEC	.8668 E-5 B14		# LANDING RADAR VELZ

		OCT	34772			# POSITION7
		OCT	07016
		OCT	01030			# TRIM DEGREES2
		OCT	33675
		OCT	01046			# COMPUTED ALTITUDE
		OCT	15700
		OCT	00321			# DP DEGREES
		OCT	26706
		2DEC	17.2010499 B-7		# POSITION 9

		2DEC	.032808399		# VELOCITY4
# Page 314
		2DEC	.32			# RADIANS

						# END OF SFOUTAB

; ============================================================================
; IDADDTAB - NOUN COMPONENT ADDRESS TABLE
;
; This table maps each noun's components to their erasable memory addresses
; (ECADR). When a noun is displayed or updated, the system uses this table
; to locate the actual data in RAM. Each noun has up to 3 components, stored
; as consecutive ECADR entries.
;
; For COMMENT-ONLY READERS: This is the "phone book" connecting DSKY displays
; to actual spacecraft data. When V16N63 displayed altitude and altitude rate
; during landing, this table told the computer where to find ABVEL (velocity)
; and HCALC1 (altitude) in memory.
;
; For CODE-ALONG READERS: ECADR (Erasable memory Core ADdRess) entries point
; to RAM locations defined in ERASABLE_ASSIGNMENTS.agc. Three-component nouns
; have 3 consecutive ECADR entries. "OCT 0" indicates unused/spare components.
; ============================================================================
						# NN 	SF CONSTANT		SF ROUTINE
IDADDTAB	ECADR	TTOGO			# 40	MIN/SEC			M/S
		ECADR	VGDISP			# 40	VEL3			DP3
		ECADR	DVTOTAL			# 40	VEL3			DP3
		ECADR	DSPTEM1			# 41	CDU DEG			CDU
		ECADR	DSPTEM1 +1		# 41	ELEV DEG		ARTH
		OCT	0			# 41	SPARE COMPONENT
		ECADR	HAPO			# 42	POS4			DP3
		ECADR	HPER			# 42	POS4			DP3
		ECADR	VGDISP			# 42 	VEL3			DP3
		ECADR	LAT			# 43	DPDEG(360)		DP4
		ECADR	LONG			# 43	DPDEG(360		DP4
		ECADR	ALT			# 43	POS4			DP3
		ECADR	HAPOX			# 44	POS4			DP3
		ECADR	HPERX			# 44	POS4			DP3
		ECADR	TFF			# 44	MIN/SEC			M/S
		ECADR	TRKMKCNT		# 45	WHOLE			ARTH
		ECADR	TTOGO			# 45	MIN/SEC			M/S
		ECADR	+MGA			# 45	DPDEG(360)		DP4
		ECADR	DAPDATR1		# 46	OCTAL ONLY		OCT
		OCT	0			# 46	SPARE COMPONENT
		OCT	0			# 46	SPARE COMPONENT
		ECADR	LEMMASS			# 47	WEIGHT2			ARTH1
		ECADR	CSMMASS			# 47	WEIGHT2			ARTH1
		OCT	0			# 47	SPARE COMPONENT
		ECADR	PITTIME			# 48	TRIM DEG2		ARTH
		ECADR	ROLLTIME		# 48	TRIM DEG2		ARTH
		OCT	0			# 48	SPARE COMPONENT
		ECADR	R22DISP			# 49	POS4			DP3
		ECADR	R22DISP +2		# 49	VEL3			DP3
		ECADR	WHCHREAD		# 49	WHOLE			ARTH
		OCT	0			# 50	SPARE
		OCT	0			# 50 	SPARE
		OCT	0			# 50	SPARE
		ECADR	ALPHASB			# 51	DPDEG(360)		DP4
		ECADR	BETASB			# 51	DPDEG(360)		DP4
		OCT	0			# 51	SPARE COMPONENT
		ECADR	ACTCENT			# 52	DPDEG(360)		DP4
		OCT	00000			# 52	SPARE COMPONENT
		OCT	00000			# 52 	SPARE COMPONENT
		OCT	00000			# 53	SPARE
		OCT	00000			# 53
		OCT	00000			# 53
		ECADR	RANGE			# 54	POS5			DP1
# Page 315
		ECADR	RRATE			# 54	VEL3			DP3
		ECADR	RTHETA			# 54	DPDEG(360)		DP4
		ECADR	NN			# 55	WHOLE			ARTH
		ECADR	ELEV			# 55	DPDEG(360)		DP4
		ECADR	CENTANG			# 55	DPDEG(360)		DP4
		ECADR	RR-AZ			# 56	DPDEG(360)		DP4
		ECADR	RR-ELEV			# 56	DPDEG(360)		DP4
		OCT	0			# 56	SPARE COMPONENT
		ECADR	DELTAR			# 57	POS4			DP3
		OCT	0			# 57	SPARE COMPONENT
		OCT	0			# 57	SPARE COMPONENT
		ECADR	POSTTPI			# 58	POS4			DP3
		ECADR	DELVTPI			# 58	VEL3			DP3
		ECADR	DELVTPF			# 58	VEL3			DP3
		ECADR	DVLOS			# 59	VEL3			DP3
		ECADR	DVLOS +2		# 59	VEL3			DP3
		ECADR	DVLOS +4		# 59	VEL3			DP3
		ECADR	VHORIZ			# 60	VEL3			DP3
		ECADR	HDOTDISP		# 60	VEL3			DP3
		ECADR	HCALC			# 60	COMP ALT		DP1
		ECADR	TTFDISP			# 61	MIN/SEC			M/S
		ECADR	TTOGO			# 61	MIN/SEC			M/S
		ECADR	OUTOFPLN		# 61	POS7			DP4
		ECADR	ABVEL			# 62	VEL3			DP3
		ECADR	TTOGO			# 62	MIN/SEC			M/S
		ECADR	DVTOTAL			# 62	VEL3			DP3
		ECADR	ABVEL			# 63	VEL3			DP3
		ECADR	HDOTDISP		# 63	VEL3			DP3
		ECADR	HCALC1			# 63	COMP ALT		DP1
		ECADR	FUNNYDSP		# 64 	2INT			2INT
		ECADR	HDOTDISP		# 64	VEL3			DP3
		ECADR	HCALC			# 64	COMP ALT		DP1
		ECADR	SAMPTIME		# 65	HMS (MIXED ONLY TO KEEP CODE 65) HMS
		ECADR	SAMPTIME		# 65	HMS			HMS
		ECADR	SAMPTIME		# 65	HMS			HMS
		ECADR	RSTACK +6		# 66	LANDING RADAR ALT	DP1
		OCT	0			# 66	LR POSITION		LRPOS
		OCT	0			# 66	SPARE COMPONENT
		ECADR	RSTACK			# 67	LANDING RADAR VELX	DP1
		ECADR	RSTACK +2		# 67	LANDING RADAR VELY	DP1
		ECADR	RSTACK +4		# 67	LANDING RADAR VELZ	DP1
		ECADR	RANGEDSP		# 68	POS7			DP4
		ECADR	TTFDISP			# 68	MIN/SEC			M/S
		ECADR	DELTAH			# 68	COMP ALT		DP1
		OCT	00000			# 69	SPARE
		OCT	00000			# 69
		OCT	00000			# 69
		ECADR	AOTCODE			# 70	OCTAL ONLY		OCT
		ECADR	AOTCODE +1		# 70 	OCTAL ONLY		OCT
		ECADR	AOTCODE +2		# 70	OCTAL ONLY		OCT
# Page 316
		ECADR	AOTCODE			# 71	OCTAL ONLY		OCT
		ECADR	AOTCODE +1		# 71	OCTAL ONLY		OCT
		ECADR	AOTCODE +2		# 71 	OCTAL ONLY		OCT
		ECADR	CDUT			# 72	360-CDU DEG		360-CDU
		ECADR	CDUS			# 72	CDU DEG			CDU
		OCT	0			# 72	SPARE COMPONENT
		ECADR	TANG			# 73	360-CDU DEG		360-CDU
		ECADR	TANG +1			# 73	CDU DEG			CDU
		OCT	0			# 73	SPARE COMPONENT
		ECADR	TTOGO			# 74	MIN/SEC			M/S
		ECADR	YAW			# 74	DPDEG(360)		DP4
		ECADR	PITCH			# 74	DPDEG(360)		DP4
		ECADR	DIFFALT			# 75	POS4			DP3
		ECADR	T1TOT2			# 75	MIN/SEC
		ECADR	T2TOT3			# 75	MIN/SEC			M/S
		ECADR	ZDOTD			# 76	VEL3			DP3
		ECADR	RDOTD			# 76	VEL3			DP3
		ECADR	XRANGE			# 76	POS4			DP3
		ECADR	TTOGO			# 77	MIN/SEC			M/S
		ECADR	YDOT			# 77 	VEL3			DP3
		OCT	0			# 77	SPARE COMPONENT
		ECADR	RSTACK			# 78 	RR RANGE		DP1
		ECADR	RSTACK +2		# 78	RR RANGE RATE		DP1
		OCT	00000			# 78	SPARE COMPONENT
		ECADR	CURSOR			# 79	CDU DEG			CDU
		ECADR	SPIRAL			# 79	CDU DEG			CDU
		ECADR	POSCODE			# 79	WHOLE			ARTH
		ECADR	DATAGOOD		# 80	WHOLE			ARTH
		ECADR	OMEGAD			# 80	DPDEG(360)		DP4
		OCT	0			# 80	SPARE COMPONENT
		ECADR	DELVLVC			# 81	VEL3			DP3
		ECADR	DELVLVC +2		# 81	VEL3			DP3
		ECADR	DELVLVC +4		# 81	VEL3			DP3
		ECADR	DELVLVC			# 82	VEL3			DP3
		ECADR	DELVLVC +2		# 82	VEL3			DP3
		ECADR	DELVLVC +4		# 82	VEL3			DP3
		ECADR	DELVIMU			# 83	VEL3			DP3
		ECADR	DELVIMU +2		# 83	VEL3			DP3
		ECADR	DELVIMU +4		# 83	VEL3			DP3
		ECADR	DELVOV			# 84	VEL3			DP3
		ECADR	DELVOV +2		# 84	VEL3			DP3
		ECADR	DELVOV +4		# 84	VEL3			DP3
		ECADR	VGBODY			# 85	VEL3			DP3
		ECADR	VGBODY +2		# 85	VEL3			DP3
		ECADR	VGBODY +4		# 85	VEL3			DP3
		ECADR	DELVLVC			# 86	VEL3			DP3
		ECADR	DELVLVC +2		# 86	VEL3			DP3
		ECADR	DELVLVC +4		# 86	VEL3			DP3
		ECADR	AZ			# 87	CDU DEG			CDU
		ECADR	EL			# 87	CDU DEG			CDU
# Page 317
		OCT	0			# 87	SPARE COMPONENT
		ECADR	STARAD			# 88	FRAC			FRAC
		ECADR	STARAD +2		# 88	FRAC			FRAC
		ECADR	STARAD +4		# 88	FRAC			FRAC
		ECADR	LANDLAT			# 89	DPDEG(90)		DP3
		ECADR	LANDLONG		# 89	DPDEG(90)		DP3
		ECADR	LANDALT			# 89	POS5			DP1
		ECADR	RANGE			# 90	POS5			DP1
		ECADR	RRATE			# 90	VEL3			DP3
		ECADR	RTHETA			# 90	DPDEG(360)		DP4
		ECADR	P21ALT			# 91	POS4			DP3
		ECADR	P21VEL			# 91 	VEL2			DP4
		ECADR	P21GAM			# 91	DPDEG(360)		DP4
		OCT	00000			# 92	SPARE
		OCT	00000			# 92
		OCT	00000			# 92
		ECADR	OGC			# 93	DPDEG(90)		DP3
		ECADR	OGC +2			# 93	DPDEG(90)		DP3
		ECADR	OGC +4			# 93	DPDEG(90)		DP3
		OCT	00000			# 94	SPARE
		OCT	00000			# 94
		OCT	00000			# 94
		OCT	0			# 95	SPARE
		OCT	0			# 95	SPARE
		OCT	0			# 95	SPARE
		OCT	0			# 96	SPARE
		OCT	0			# 96	SPARE
		OCT	0			# 96	SPARE
		ECADR	DSPTEM1			# 97	WHOLE			ARTH
		ECADR	DSPTEM1 +1		# 97	WHOLE			ARTH
		ECADR	DSPTEM1 +2		# 97	WHOLE			ARTH
		ECADR	DSPTEM2			# 98	WHOLE			ARTH
		ECADR	DSPTEM2 +1		# 98	FRAC			FRAC
		ECADR	DSPTEM2 +2		# 98	WHOLE			ARTH
		ECADR	WWPOS			# 99 	POS9			DP3
		ECADR	WWVEL			# 99	VEL4			DP2
		ECADR	WWBIAS			# 99 	RADIANS			DP4

# END OF IDADDTAB

; ============================================================================
; RUTMXTAB - SCALING FACTOR ROUTINE MATRIX TABLE
;
; This table specifies which scaling factor (SF) routines to apply to each
; noun component during display output and keyboard input. Each octal entry
; encodes three SF routine codes (one per component) in a packed format.
;
; For COMMENT-ONLY READERS: Different data needs different formatting. Time
; displays as HH:MM:SS, velocity as XXXXX.FT/SEC, angles as XXX.XX degrees.
; This table tells the DSKY how to convert internal computer units into the
; formats astronauts see on the display.
;
; For CODE-ALONG READERS: Each octal value encodes 3 SF routine codes (5 bits
; each). Bits 1-5 = component 1 routine, bits 6-10 = component 2, bits 11-15
; = component 3. Routine codes defined in SFINTAB/SFOUTAB comments (lines
; 42-62): 00=OCT, 01=FRAC, 02=CDU, 03=ARITH, 04=DP1, 05=DP2, etc.
; ============================================================================
						# NN	SF ROUTINES
RUTMXTAB	OCT	16351			# 40	M/S, DP3, DP3
		OCT	00142			# 41	CDU, ARTH
		OCT	16347			# 42	DP3, DP3, DP3
		OCT	16512			# 43	DP4, DP4, DP3
		OCT	22347			# 44	DP3, DP3, M/S
		OCT	24443			# 45	ARTH, M/S, DP4
		OCT	00000			# 46	OCT
		OCT	00553			# 47	ARITH1, ARITH1
# Page 318
		OCT	00143			# 48	ARTH, ARTH
		OCT	06347			# 49	DP3, DP3, ARTH
		OCT	0			# 50	SPARE
		OCT	00512			# 51	DP4, DP4
		OCT	00012			# 52	DP4
		OCT	00000			# 53 	SPARE
		OCT	24344			# 54	DP1, DP3, DP4
		OCT	24503			# 55	ARTH, DP4, DP4
		OCT	00512			# 56	DP4, DP4
		OCT	00007			# 57	DP3
		OCT	16347			# 58	DP3, DP3, DP3
		OCT	16347			# 59	DP3, DP3, DP3
		OCT	10347			# 60	DP3, DP3, DP1
		OCT	24451			# 61	M/S, M/S, DP4
		OCT	16447			# 62	DP3, M/S, DP3
		OCT	10347			# 63	DP3, DP3, DP1
		OCT	10354			# 64	2INT, DP3, DP1
		OCT	20410			# 65	HMS, HMS, HMS
		OCT	00304			# 66	DP1, LRPOS
		OCT	10204			# 67	DP1, DP1, DP1
		OCT	10452			# 68	DP4, M/S, DP1
		OCT	00000			# 69	SPARE
		OCT	0			# 70	OCT, OCT, OCT
		OCT	0			# 71	OCT, OCT, OCT
		OCT	00115			# 72	360-CDU, CDU
		OCT	00115			# 73	360-CDU, CDU
		OCT	24511			# 74	M/S, DP4, DP4
		OCT	22447			# 75	DP3, M/S, M/S
		OCT	16347			# 76	DP3, DP3, DP3
		OCT	00351			# 77	M/S, DP3
		OCT	00204			# 78	DP1, DP1
		OCT	06102			# 79	CDU, CDU, ARTH
		OCT	00503			# 80	ARTH, DP4
		OCT	16347			# 81	DP3, DP3, DP3
		OCT	16347			# 82	DP3, DP3, DP3
		OCT	16347			# 83	DP3, DP3, DP3
		OCT	16347			# 84	DP3, DP3, DP3
		OCT	16347			# 85	DP3, DP3, DP3
		OCT	16347			# 86	DP3, DP3, DP3
		OCT	00102			# 87	CDU, CDU
		OCT	02041			# 88	FRAC FOR EACH
		OCT	10347			# 89	DP3, DP3, DP1
		OCT	24344			# 90	DP1, DP3, DP4
		OCT	24507			# 91	DP3, DP4, DP4
		OCT	00000			# 92	SPARE
		OCT	16347			# 93	DP3, DP3, DP3
		OCT	00000			# 94	SPARE
		OCT	0			# 95	SPARE
		OCT	0			# 96	SPARE
		OCT	06143			# 97	ARTH, ARTH, ARTH
# Page 319
		OCT	06043			# 98	ARTH, FRAC, ARTH
		OCT	24247			# 99	DP3, DP2, DP4

# END OF RUTMXTAB

		SBANK=	LOWSUPER

