# Copyright:	Public domain.
# Filename:	PINBALL_NOUN_TABLES.agc
# Purpose:	Part of the source code for Comanche, build 055. It
#		is part of the source code for the Command Module's
#		(CM) Apollo Guidance Computer (AGC), Apollo 11.
# Assembler:	yaYUL
# Reference:	pp. 268-284
# Contact:	Ron Burkey <info@sandroid.org>,
#  		Fabrizio Bernardini <fabrizio@spacecraft.it>
# Website:	http://www.ibiblio.org/apollo.
# Mod history:	2009-05-18 FB	Transcription Batch 3 Assignment.
#		2009-05-23 RSB	In NNTYPTAB, corrected former 13 SPARE.
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
; FILE: PINBALL_NOUN_TABLES.agc
; MODULE: COMAID Subsystem (Mission Support)
; MISSION PHASE: all-phases
;
; TL;DR: DSKY noun definition tables documenting all 99 noun data types 
;        (N01-N99) with complete catalog of data formats, scaling factors,
;        display modes, and associated verbs. Nouns specify what information
;        to display or input via DSKY - from time and position to velocity
;        and angles - throughout Apollo 11 mission phases.
;
; COMMENT-ONLY READERS: This file defines all the types of information the
;        computer could show on the DSKY display - like N36 for time or 
;        N17 for crew velocity readings during critical mission phases.
; CODE-ALONG READERS: Study complete noun catalog with data format specs,
;        scaling factors, display modes (octal/decimal), component counts,
;        precision requirements, and verb associations for DSKY interface.
; ============================================================================

# Page 268
# THE FOLLOWING REFERS TO THE NOUN TABLES

; ============================================================================
; NOUN TABLE ARCHITECTURE OVERVIEW
;
; The DSKY (Display and Keyboard) interface uses "nouns" to specify what
; type of data to display or accept as input. Each noun (N01-N99) defines:
;   - How many components (1, 2, or 3 register values)
;   - Display format (octal or decimal)
;   - Scaling factors for unit conversion
;   - Whether data can be loaded by crew (input capability)
;
; During Apollo 11, the crew used verb+noun combinations extensively:
;   V16N36 - Display time remaining in program
;   V06N62 - Display velocity components
;   V16N17 - Display crew velocity for landing monitoring
;
; This file contains five major tables:
;   NNADTAB  - Memory addresses for each noun's data
;   NNTYPTAB - Component count and display format codes
;   SFINTAB  - Scaling factors for input (crew entry)
;   SFOUTAB  - Scaling factors for output (display)
;   IDADDTAB/RUTMXTAB - Special handling for "mixed" nouns (40-99)
; ============================================================================

# COMPONENT CODE NUMBER		INTERPRETATION

# 00000				1 COMPONENT
# 00001				2 COMPONENT
# 00010				3 COMPONENT
# X1XXX				BIT 4 = 1. DECIMAL ONLY
# 1XXXX				BIT 5 = 1. NO LOAD
# END OF COMPONENT CODE NUMBERS

; Component codes define how many data values a noun displays:
;   1 COMPONENT - Single value (e.g., N36 time shows one time value)
;   2 COMPONENT - Two values (e.g., apogee and perigee altitudes)
;   3 COMPONENT - Three values (e.g., X, Y, Z position coordinates)
;
; DECIMAL ONLY flag forces decimal display even if verb requests octal
; NO LOAD flag prevents crew from entering data for this noun (display-only)


# SF ROUTINE CODE NUMBER	INTERPRETATION

; SF (Scale Factor) routines convert between AGC internal representation
; and human-readable display formats. AGC uses fixed-point arithmetic with
; various scaling factors depending on the physical quantity:
;
;   Positions: Scaled by 2^29 meters (centimeters precision)
;   Velocities: Scaled by 2^7 meters/centisecond  
;   Angles: Scaled as fractional revolutions or CDU degrees
;   Time: Scaled in centiseconds
;
; Different SF routines handle unit conversion, decimal point placement,
; and range checking for the five-digit DSKY display (±XXXXX format).

# 00000		OCTAL ONLY
# 00001		STRAIGHT FRACTIONAL
# 00010		CDU DEGREES (XXX.XX)
# 00011		ARITHMETIC SF
# 00100		ARITH DP1	OUT(MULT BY 2EXP14 AT END)	IN(STRAIGHT)
# 00101		ARITH DP2	OUT(STRAIGHT)			IN(SL 7 AT END)
# 00110		Y OPTICS DEGREES (XX.XXX MAX 89.999)
# 00111		ARITH DP3	OUT ( SL 7 AT END)		IN ( STRAIGHT)
# 01000		WHOLE HOURS IN R1, WHOLE MINUTES (MOD 60) IN R2,
#			SECONDS (MOD 60) 0XX.XX IN R3. *** ALARMS IF USED WITH OCTAL
# 01001		MINUTES (MOD 60) IN D1D2, D3 BLANK, SECONDS (MOD 60) IN D4D5
#				LIMITS TO 59B59 IF MAG EXCEEDS THIS VALUE.
#				ALARMS IF USED WITH OCTAL ******** IN (ALARM)
# 01010		ARITH DP4	OUT (STRAIGHT)			IN (SL 3 AT END)
# 01011		ARITH1 SF	OUT(MULT BY 2EXP14 AT END)	IN(STRAIGHT)
# 01100		2 INTEGERS IN D1D2, D4D5, D3 BLANK.
#				ALARMS IF USED WITH OCTAL ******** IN (ALARM)
# 01101	DP STRAIGHT FRACTIONAL
# END OF SF ROUTINE CODE NUMBERS

; Special time formats (01000, 01001) format mission elapsed time for crew:
;   Format 01000: HH:MM:SS.SS (hours, minutes, seconds with centiseconds)
;   Format 01001: MM:SS (compact minutes:seconds for countdowns)
; These were critical during Apollo 11 descent when Armstrong monitored
; remaining time in landing program while fuel margins became critical.


# 	SF CONSTANT CODE NUMBER	INTERPRETATION

; SF constant codes define the physical units and display precision for
; each type of measurement. The AGC displays used both nautical miles
; (for orbital/cislunar navigation) and feet (for landing approach).
;
; Common units displayed during Apollo 11:
;   Position: Nautical miles (orbital), Feet (landing altitude)
;   Velocity: Feet/second (descent rate, horizontal velocity)
;   Angles: Degrees (IMU gimbal angles, spacecraft attitude)
;   Weight: Pounds (propellant remaining, spacecraft mass)
;   Time: Hours:Minutes:Seconds (mission elapsed time)

#	00000			WHOLE				USE ARITH
#	00000			DP TIME SEC (XXX.XX SEC)	USE ARITHDP1
#	00001			SPARE
#	00010			CDU DEGREES		     USE CDU DEGREES
#	00010			Y OPTICS DEGREES	USE Y OPTICS DEGREES
#	00011			DP DEGREES (90) XX.XXX DEG	USE ARITHDP3
#	00100			DP DEGREES (360) XXX.XX DEG	USE ARITHDP4
#	00101			DEGREES (180) XXX.XX DEG	USE ARITH
#	00110			WEIGHT2 (XXXXX. LBS)		USE ARITH1
#	00111			POSITION5 (XXX.XX NAUTICAL MILES)
#								USE ARITHDP3
#	01000			POSITION4 (XXXX.X NAUTICAL MILES)
# Page 269
#							    USE ARITHDP3
#	01001			VELOCITY2 (XXXXX. FT/SEC)   USE ARITHDP4
#	01010			VELOCITY3 (XXXX.X FT/SEC)   USE ARITHDP3
#	01011			ELEVATION DEGREES (89.999MAX) USE ARITH
#	01100			TRIM DEGREES (XXX.XX DEG)    USE ARITH
#	01101			INERTIA (XXXXXBB. SLUG FT FT)  USE ARITH
#	01101			THRUST MOMENT (XXXXXBB.FT LBS) USE ARITH
#	01110			VELOCITY/2VS (XXXXX. FT/SEC)USE ARITHDP4
#	01111			POSITION6 (XXXX.X NAUT MI)  USE ARITHDP3
#	10000			DRAG ACCELERATION (XXX.XX G)USE ARITHDP2
#	10001			POSITION8 (XXXX.X NAUT MI)  USE ARITHDP3
#	10010			POSITION9 (XXXXX. FT)	    USE ARITHDP3
#	10011			VELOCITY4 (XXXX.X FT/SEC)   USE ARITHDP2
# 	END OF SF CONSTANT CODE NUMBERS

; Velocity formats were especially critical during lunar landing:
;   VELOCITY2/VELOCITY4: Displayed descent rate during powered descent
;   Armstrong and Aldrin monitored these values continuously as the LM
;   descended from 50,000 feet to touchdown on July 20, 1969.
;
; Position formats tracked orbital mechanics and landing approach:
;   POSITION5/POSITION6: Orbital altitude in nautical miles
;   POSITION9: Landing radar altitude in feet (critical below 10,000 ft)


# FOR GREATER THAN SINGLE PRECISION SCALES, PUT ADDRESS IN MAJOR PART INTO
# NOUN TABLES.
# OCTAL LOADS PLACE +0 INTO MAJOR PART, DATA INTO MINOR PART.
# OCTAL DISPLAYS SHOW MINOR PART ONLY.
# TO GET AT BOTH MAJOR AND MINOR PARTS(IN OCTAL), USE NOUN 01.

; Double-precision nouns store their address in the "major part" (upper word)
; while single-precision data uses only the "minor part" (lower word).
; When crew loads data in octal mode, the AGC automatically sets major part
; to +0 and places the entered value in minor part.

# A NOUN MAY BE DECLARED :DECIMAL ONLY: BY MAKING BIT4=1 OF ITS COMPONENT
# CODE NUMBER.  IF THIS NOUN IS USED WITH ANY OCTAL DISPLAY VERB, OR IF
# DATA IS LOADED IN OCTAL, IT ALARMS.

; Decimal-only nouns (bit 4 = 1 in component code) prevent accidental octal
; entry/display which could confuse crew during critical mission phases.
; Most navigation and guidance nouns are decimal-only for clarity.

# IN LOADING AN :HOURS, MINUTES, SECONDS: NOUN, ALL 3 WORDS MUST BE
# LOADED, OR ALARM.
# ALARM IF AN ATTEMPT IS MADE TO LOAD :SPLIT MINUTES/SECONDS: (MMBSS).
# THIS IS USED FOR DISPLAY ONLY.

; Time format nouns require complete entry (all components) to maintain
; consistency. The split minutes/seconds format (MMBSS) is display-only
; to prevent partial time updates that could corrupt mission timers.

; ============================================================================
; TRANSITION: From noun table format specifications to noun table routines
;
; The preceding sections defined the encoding schemes for noun tables.
; The following section contains routines that read noun table entries
; from memory. These routines execute in the same memory bank as the tables
; themselves (to avoid bank-switching overhead) and are called via DXCH Z
; from the main display interface code.
;
; When crew presses NOUN followed by two digits on the DSKY, these routines
; retrieve the corresponding noun definition parameters and prepare the
; display system to show or accept the requested data type.
; ============================================================================

# Page 270
# THE FOLLOWING ROUTINES ARE FOR READING THE NOUN TABLES AND THE SF TABLES
# (WHICH ARE IN A SEPARATE BANK FROM THE REST OF PINBALL).  THESE READING
# ROUTINES ARE IN THE SAME BANK AS THE TABLES.  THEY ARE CALLED BY DXCH Z.


# LODNNTAB LOADS NNADTEM WITH THE NNADTAB ENTRY, NNTYPTEM WITH THE
# NNTYPTAB ENTRY.  IF THE NOUN IS MIXED, IDADITEM IS LOADED WITH THE FIRST
# IDADDTAB ENTRY, IDAD2TEM THE SECOND IDADDTAB ENTRY, IDAD3TEM THE THIRD
# IDADDTAB ENTRY, RUTMXTEM WITH THE RUTMXTAB ENTRY.  MIXBR IS SET FOR
# MIXED OR NORMAL NOUN.

; LODNNTAB - Load Noun Table Entry
;
; Called by display interface when crew enters a noun number on DSKY.
; Retrieves noun definition from six parallel tables based on NOUNREG value.
;
; For NORMAL nouns (N01-N39):
;   - Loads NNADTEM with noun address from NNADTAB
;   - Loads NNTYPTEM with component/scale codes from NNTYPTAB
;   - Sets MIXBR = +1 to indicate normal noun
;
; For MIXED nouns (N40-N99):
;   - Loads all normal noun data plus:
;   - RUTMXTEM with routine address from RUTMXTAB
;   - IDAD1TEM, IDAD2TEM, IDAD3TEM with display addresses from IDADDTAB
;   - Sets MIXBR = +2 to indicate mixed noun
;
; Mixed nouns display data from multiple memory locations simultaneously.
; Example: N17 (velocity components) displays VREL (R1), VREL+2 (R2), VREL+4 (R3).
;
; INPUT:  NOUNREG = Noun number (00-99 decimal)
;         Z = Return address (via DXCH)
; OUTPUT: Noun parameter temporary storage loaded with table entries
;
; Historical context: During Apollo 11 descent, this routine executed when
; Armstrong requested N63 (lunar latitude/longitude/altitude display) and
; N16 (velocity components) to monitor descent progress.

		BANK	06
		SETLOC	PINBALL3
		BANK

		COUNT	42/NOUNS

LODNNTAB	DXCH	IDAD2TEM		# SAVE RETURN INFO IN IDAD2TEM, IDAD3TEM.
		INDEX	NOUNREG
		CAF	NNADTAB		; Use NOUNREG as index into NNADTAB
		TS	NNADTEM		; Store noun address in NNADTEM
		INDEX	NOUNREG
		CAF	NNTYPTAB	; Indexed fetch of component/scale code
		TS	NNTYPTEM	; Store in NNTYPTEM for later use
		CS	NOUNREG		; Complement of noun number
		AD	MIXCON		; Add MIXCON (octal 50 = decimal 40)
		EXTEND
		BZMF	LODMIXNN	; Branch if noun >= 40 (mixed noun)
		CAF	ONE		; Noun < 40 (normal noun)
		TS	MIXBR		; Set MIXBR = +1 (normal noun indicator)
		TC	LODNLV		; Skip mixed noun processing
LODMIXNN	CAF	TWO		; Noun >= 40 (mixed noun)
		TS	MIXBR		; Set MIXBR = +2 (mixed noun indicator)
		INDEX	NOUNREG
		CAF	RUTMXTAB -40D	; Indexed fetch from RUTMXTAB (offset by -40)
		TS	RUTMXTEM	; Store mixed noun routine address
		CAF	LOW10		; Mask constant (bits 0-9)
		MASK	NNADTEM		; Extract low 10 bits of noun address
		TS	Q		; Temporary storage in Q register
		INDEX	A		; Use extracted value as index
		CAF	IDADDTAB	; Fetch first component address
		TS	IDAD1TEM	; Store R1 component address
		EXTEND
		INDEX	Q		; Use Q as index for double-precision load
		DCA	IDADDTAB +1	; Fetch R2 and R3 component addresses
LODNLV		DXCH	IDAD2TEM	; Restore return address from IDAD2TEM
		DXCH	Z		; Place into Z register for return

MIXCON		=	OCT50		; Constant: First mixed noun = 40 (decimal)

; ============================================================================
; GTSFOUT - Get Scale Factor Output Routine
;
; Loads SFTEMP1, SFTEMP2 with double-precision SFOUTAB entries for display
; formatting. Output scale factors convert internal AGC scaled values to
; human-readable DSKY display formats (degrees, velocities, positions, etc.).
;
; INPUT:  2X(SFCONUM) in SFTEMP1 (double index into SFOUTAB)
; OUTPUT: SFTEMP1, SFTEMP2 loaded with scale factor routine addresses
; ============================================================================
# Page 271

GTSFOUT		DXCH	SFTEMP1		; Save incoming index temporarily
		EXTEND
		INDEX	A		; Use A as index into SFOUTAB
		DCA	SFOUTAB		; Double-precision load of SF routine addresses
SFCOM		DXCH	SFTEMP1		; Store in SFTEMP1, SFTEMP2
		DXCH	Z		; Restore return address and return

; ============================================================================
; GTSFIN - Get Scale Factor Input Routine
;
; Loads SFTEMP1, SFTEMP2 with double-precision SFINTAB entries for data input
; processing. Input scale factors convert crew DSKY entries to internal AGC
; scaled representations for computational use.
;
; INPUT:  2X(SFCONUM) in SFTEMP1 (double index into SFINTAB)
; OUTPUT: SFTEMP1, SFTEMP2 loaded with scale factor routine addresses
; ============================================================================

GTSFIN		DXCH	SFTEMP1		; Save incoming index temporarily
		EXTEND
		INDEX	A		; Use A as index into SFINTAB
		DCA	SFINTAB		; Double-precision load of SF routine addresses
		TCF	SFCOM		; Transfer to common return path

; ============================================================================
; NNADTAB - NOUN ADDRESS TABLE
; 
; This table contains the memory addresses for all 99 nouns (N01-N99).
; Each entry points to the erasable memory location where the noun's data
; is stored. The DSKY uses these addresses to fetch or store values when
; displaying or accepting crew input for a given noun.
;
; COMMENT-ONLY READERS: Think of this as the computer's index showing where
;        each type of information lives in memory - like an address book
;        telling where to find time, position, velocity, and other data.
;
; CODE-ALONG READERS: Each entry is an ECADR (erasable memory address) or
;        FCADR (fixed memory address) pointing to the first component of
;        the noun's data. Multi-component nouns store successive values in
;        consecutive memory locations.
;
; KEY NOUNS IN APOLLO 11 MISSION:
;   N16 - Time from ignition (used during engine burns)
;   N17 - Velocity display (critical during descent and ascent)
;   N36 - Time display (mission elapsed time, GET)
;   N43 - Latitude, longitude, altitude (landing site coordinates)
;   N63 - Range, range rate, theta (rendezvous radar data)
;   N90 - Range, range rate, theta (landing radar data during descent)
; ============================================================================

						# NN 	 NORMAL NOUNS
NNADTAB		OCT	00000			# 00 	NOT IN USE
		OCT	40000			# 01 	SPECIFY MACHINE ADDRESS (FRACTIONAL)
		OCT	40000			# 02 	SPECIFY MACHINE ADDRESS (WHOLE)
		OCT	40000			# 03	SPECIFY MACHINE ADDRESS (DEGREES)
		OCT	0			# 04	SPARE
		ECADR	DSPTEM1			# 05 	ANGULAR ERROR/DIFFERENCE
		ECADR	OPTION1			# 06	OPTION CODE
		ECADR	XREG			# 07	ECADR OF WORD TO BE MODIFIED
						#	ONES FOR BITS TO BE MODIFIED
						#	1 TO SET OR 0 TO RESET SELECTED BITS
; N08 - Alarm data display. Used to show program alarm codes on DSKY.
; During Apollo 11 descent, this displayed the famous 1202 alarm code.
		ECADR	ALMCADR			# 08 	ALARM DATA
; N09 - Alarm codes register. Tracks system failure conditions and warnings.
		ECADR	FAILREG			# 09	ALARM CODES
		OCT	77776			# 10	CHANNEL TO BE SPECIFIED
		ECADR	TCSI			# 11	TIG OF CSI (HRS,MIN,SEC)
		ECADR	OPTIONX			# 12	OPTION CODE
						#	   (USED BY EXTENDED VERBS ONLY)
		ECADR	TCDH			# 13	TIG OF CDH (HRS,MIN,SEC)
		OCT	0			# 14	SPARE
		OCT	77777			# 15	INCREMENT MACHINE ADDRESS
; N16 - Time from ignition (HRS:MIN:SEC). Critical during engine burns
; for monitoring burn duration and cutoff timing.
		ECADR	DSPTEMX			# 16	TIME OF EVENT (HRS,MIN,SEC)
; N17 - Astronaut total attitude (degrees). Displays spacecraft orientation
; in all three axes. Monitored continuously during maneuvers and attitude control.
		ECADR	CPHIX			# 17	ASTRONAUT TOTAL ATTITUDE
		ECADR	THETAD			# 18	AUTO MANEUVER BALL ANGLES
		ECADR	THETAD			# 19	BYPASS ATTITUDE TRIM MANEUVER
; N20 - IMU CDU angles (CDUX, CDUY, CDUZ in degrees). Shows the gimbal angles
; of the Inertial Measurement Unit. Essential for navigation and alignment.
		ECADR	CDUX			# 20	ICDU ANGLES
; N21 - PIPA readings (acceleration pulses). Raw data from the IMU's
; Pulsed Integrating Pendulous Accelerometers used for velocity calculations.
		ECADR	PIPAX			# 21	PIPAS
		ECADR	THETAD			# 22	NEW ICDU ANGLES
		OCT	00000			# 23	SPARE
		ECADR	DSPTEM2 +1		# 24	DELTA TIME FOR AGC CLOCK(HRS,MIN,SEC)
		ECADR	DSPTEM1			# 25	CHECKLIST
						#	   (USED WITH PLEASE PERFORM ONLY)
		ECADR	DSPTEM1			# 26	PRIO/DELAY, ADRES, BBCON
		ECADR	SMODE			# 27	SELF TEST ON/OFF SWITCH
# Page 272
		OCT	0			# 28	SPARE
		ECADR	DSPTEM1			# 29	XSM LAUNCH AZIMUTH
		ECADR	DSPTEM1			# 30	TARGET CODES
		ECADR	DSPTEM1			# 31	TIME OF LANDING SITE (HRS,MIN,SEC)
		ECADR	-TPER			# 32	TIME TO PERIGEE (HRS,MIN,SEC)
; N33 - Time of ignition TIG (HRS:MIN:SEC). Displays when the next engine
; burn will occur. Crew monitors this for critical maneuvers like TLI, LOI, TEI.
		ECADR	TIG			# 33	TIME OF IGNITION (HRS,MIN,SEC)
		ECADR	DSPTEM1			# 34	TIME OF EVENT (HRS,MIN,SEC)
; N35 - Time to go (HRS:MIN:SEC). Counts down to upcoming events.
; Displayed during coast phases and while waiting for maneuvers.
		ECADR	TTOGO			# 35	TIME TO GO TO EVENT (HRS,MIN,SEC)
; N36 - Mission elapsed time (GET - Ground Elapsed Time) displayed in
; HRS:MIN:SEC format. The master clock for the mission. Apollo 11 landing
; occurred at 102:45:40 GET. This was one of the most frequently displayed nouns.
		ECADR	TIME2			# 36	TIME OF AGC CLOCK (HRS,MIN,SEC)
		ECADR	TTPI			# 37	TIG OF TPI (HRS,MIN,SEC)
		ECADR	TET			# 38	TIME OF STATE VECTOR
		ECADR	T3TOT4			# 39	DELTA TIME TO TRANSFER (HRS,MIN,SEC)
# END OF NNADTAB FOR NORMAL NOUNS

						# NN	 MIXED NOUNS
		OCT	64000			# 40	TIME TO IGNITION/CUTOFF
						#	VG
						#	DELTA V (ACCUMULATED)
		OCT	02003			# 41	TARGET	AZIMUTH
						#		ELEVATION
		OCT	24006			# 42	APOGEE
						#	PERIGEE
						#	DELTA V (REQUIRED)
; N43 - Latitude, longitude, altitude. Displays spacecraft position on or above
; the lunar/Earth surface. Critical during landing - Armstrong used N43 to verify
; Eagle's landing site coordinates in the Sea of Tranquility.
		OCT	24011			# 43	LATITUDE
						#	LONGITUDE
						#	ALTITUDE
		OCT	64014			# 44	APOGEE
						#	PERIGEE
						#	TFF
		OCT	64017			# 45	MARKS (VHF - OPTICS)
						#	TTI OF NEXT BURN
						#	MGA
		OCT	02022			# 46	AUTOPILOT CONFIGURATION
		OCT	22025			# 47	THIS VEHICLE WEIGHT
						#	OTHER VEHICLE WEIGHT
		OCT	22030			# 48	PITCH TRIM
						#	YAW TRIM
		OCT	24033			# 49	DELTA R
						#	DELTA V
						#	VHF OR OPTICS CODE
		OCT	64036			# 50	SPLASH ERROR
						#	PERIGEE
						#	TFF
		OCT	22041			# 51	S-BAND ANTENNA	PITCH
						#			YAW
		OCT	00044			# 52	CENTRAL ANGLE OF ACTIVE VEHICLE
		OCT	24047			# 53	RANGE
						#	RANGE RATE
						#	PHI
# Page 273
		OCT	24052			# 54	RANGE
						#	RANGE RATE
						#	THETA
		OCT	24055			# 55	PERIGEE CODE
						#	ELEVATION ANGLE
						#	CENTRAL ANGLE
		OCT	22060			# 56	REENTRY ANGLE,
						#	DELTA V
		OCT	20063			# 57	DELTA R
		OCT	24066			# 58	PERIGEE ALT
						#	DELTA V TPI
						#	DELTA V TPF
		OCT	24071			# 59	DELTA VELOCITY LOS
; N60 - Reentry parameters: Maximum G-load, predicted velocity, flight path angle
; at entry interface. Crew monitored these for safe reentry corridor constraints.
		OCT	24074			# 60	GMAX
						#	VPRED
						#	GAMMA EI
; N61 - Predicted impact point: Latitude, longitude, and spacecraft orientation
; (heads up/down). For Apollo 11, displayed Pacific Ocean splashdown coordinates.
		OCT	24077			# 61	IMPACT LATITUDE
						#	IMPACT LONGITUDE
						#	HEADS UP/DOWN
; N62 - Landing velocity data: Inertial velocity magnitude, altitude rate (HDOT),
; altitude above surface. Essential for monitoring safe descent and touchdown conditions.
		OCT	24102			# 62	INERTIAL VEL MAG (V1)
						#	ALT RATE CHANGE (HDOT)
						#	ALT ABOVE PAD RADIUS (H)
; N63 - Range-to-go during reentry: Distance to splash, predicted velocity at
; impact, time remaining to entry interface at 400,000 feet altitude.
		OCT	64105			# 63	RANGE 297,431 TO SPLASH (RTGO)
						#	PREDICTED INERT VEL (VIO)
						#	TIME TO GO TO 297,431 (TTE)
		OCT	24110			# 64	DRAG ACCELERATION
						#	INERTIAL VELOCITY (VI)
						#	RANGE TO SPLASH
		OCT	24113			# 65	SAMPLED AGC TIME (HRS,MIN,SEC)
						#	(FETCHED IN INTERRUPT)
		OCT	24116			# 66	COMMAND BANK ANGLE (BETA)
						# 	CROSS RANGE ERROR
						#	DOWN RANGE ERROR
		OCT	24121			# 67	RANGE TO TARGET
						#	PRESENT LATITUDE
						#	PRESENT LONGITUDE
		OCT	24124			# 68	COMMAND BANK ANGLE (BETA)
						#	INERTIAL VELOCITY (VI)
						#	ALT RATE CHANGE (RDOT)
		OCT	24127			# 69	BETA
						#	DL
						#	VL
; N70 - Optical sighting data: Star identification code, landmark position data,
; and horizon angle. Used for celestial navigation and IMU alignment verification.
		OCT	04132			# 70	STAR CODE
						#	LANDMARK DATA
						#	HORIZON DATA
; N71 - Star tracking data: Star code and landmark/horizon measurements for
; navigation updates using sextant or telescope during cislunar coast.
		OCT	04135			# 71	STAR CODE
						#	LANDMARK
						#	HORIZON
		OCT	24140			# 72	DELT ANG
						#	DELT ALT
# Page 274
						# 	SEARCH OPTION
		OCT	04143			# 73	ALTITUDE
						#	VELOCITY
						#	FLIGHT PATH ANGLE
		OCT	04146			# 74	COMMAND BANK ANGLE (BETA)
						# 	INERTIAL VELOCITY (VI)
						# 	DRAG ACCELERATION
		OCT	64151			# 75	DELTA ALTITUDE CDH
						# 	DELTA TIME (CDH-CSI OR TPI-CDH)
						#	DELTA TIME (TPI-CDH OR TPI-NOMTPI)
		OCT	0			# 76	SPARE
		OCT	0			# 77	SPARE
		OCT	0			# 78	SPARE
		OCT	0			# 79	SPARE
; N80 - Burn parameters: Time-to-ignition or cutoff, terminal velocity (VG),
; and accumulated delta-V. Critical for SPS and RCS burn planning and execution.
		OCT	64170			# 80	TIME TO IGNITION/CUTOFF
						#	VG
						#	DELTA V (ACCUMULATED)
; N81 - Delta-V for this vehicle (launch vehicle or CSM/LM). Single component
; velocity change used in trajectory and rendezvous calculations.
		OCT	24173			# 81	DELTA V (LV)
; N82 - Delta-V for this vehicle (alternate format). Used for burn planning
; and post-burn verification of velocity change achieved.
		OCT	24176			# 82	DELTA V (LV)
; N83 - Delta-V in body coordinates. Velocity change components along spacecraft
; body axes (X, Y, Z) for thrust vector control and attitude planning.
		OCT	24201			# 83	DELTA V (BODY)
; N84 - Delta-V for other vehicle (CSM or LM). Target spacecraft velocity change
; for relative trajectory calculations during rendezvous operations.
		OCT	24204			# 84	DELTA V (OTHER VEHICLE)
; N85 - Terminal velocity in body coordinates (VG). Final velocity at burn cutoff
; expressed in spacecraft body reference frame.
		OCT	24207			# 85	VG (BODY)
; N86 - Delta-V for this vehicle (third format). Additional velocity change
; display option for mission planning and post-maneuver analysis.
		OCT	24212			# 86	DELTA V (LV)
; N87 - Mark data: Optical instrument shaft and trunnion angles when crew pressed
; MARK button during star or landmark sighting. Captures target direction for navigation.
		OCT	02215			# 87	MARK DATA	SHAFT
						#			TRUNION
; N88 - Half-unit sun or planet vector. Direction vector to celestial body scaled
; to 0.5 magnitude for navigation computations and solar pressure calculations.
		OCT	24220			# 88	HALF UNIT SUN OR PLANET VECTOR
; N89 - Landmark position: Latitude, scaled longitude (divided by 2), and altitude
; above reference surface. For lunar or Earth surface feature tracking.
		OCT	24223			# 89	LANDMARK	LATITUDE
						#			LONGITUDE/2
						#			ALTITUDE
; N90 - Entry trajectory data: Y-position, Y-velocity (Y DOT), and heading angle
; (PSI). Used during atmospheric entry guidance for Apollo 11's Earth return.
		OCT	24226			# 90	Y
						#	Y DOT
						#	PSI
; N91 - Optical CDU angles (octal format): Shaft and trunnion angles in octal
; representation for raw sensor data display and engineering analysis.
		OCT	02231			# 91	OCDU ANGLES	SHAFT
						#			TRUNION
; N92 - New optics angles: Commanded shaft and trunnion positions for sextant
; or telescope pointing. Updated when crew selects new celestial target.
		OCT	02234			# 92	NEW OPTICS ANGLES SHAFT
						#			  TRUNION
; N93 - Delta gyro angles: Changes in IMU gimbal angles since last measurement.
; Three-component vector for gyro drift monitoring and alignment quality assessment.
		OCT	04237			# 93	DELTA GYRO ANGLES
; N94 - New optics angles (alternate): Commanded optical instrument positions
; for target acquisition. Similar to N92 but different scale or reference frame.
		OCT	02242			# 94	NEW OPTICS ANGLES SHAFT
						#			  TRUNNION
; N95 - Preferred attitude: ICDU (Inner gimbal CDU) angles defining desired
; spacecraft orientation for current mission phase or maneuver.
		OCT	04245			# 95	PREFERRED ATTITUDE ICDU ANGLES
; N96 - X-axis attitude: ICDU angles for spacecraft orientation with +X axis
; pointed toward specific target (Earth, Moon, Sun, or star).
		OCT	04250			# 96	+X-AXIS ATTITUDE ICDU ANGLES
; N97 - System test inputs: Three test values injected for AGC self-check routines
; and sensor validation. Used during pre-flight testing and in-flight diagnostics.
		OCT	04253			# 97	SYSTEM TEST INPUTS
; N98 - System test results: Output values from self-check routines comparing
; computed results against expected values for system health verification.
		OCT	04256			# 98	SYSTEM TEST RESULTS
; N99 - Navigation accuracy: RMS (Root Mean Square) errors in position and velocity,
; plus RMS option code. Quantifies navigation solution quality for crew awareness.
		OCT	24261			# 99	RMS IN POSITION
						#	RMS IN VELOCITY
						#	RMS OPTION
# END OF NNADTAB FOR MIXED NOUNS

; ============================================================================
; NNTYPTAB - NOUN TYPE TABLE
; 
; This table defines the display format and component structure for each noun.
; Each entry specifies:
;   - Number of components (1, 2, or 3)
;   - Display format (octal, decimal, fractional, degrees, HMS time)
;   - Scale factor routine (whole, fractional, CDU degrees, HMS, etc.)
;   - Load permission (some nouns display-only, cannot be loaded by crew)
;
; Format codes (from bits 1-2): 00=1comp, 01=2comp, 10=3comp
; Decimal-only flag (bit 4): 1=decimal display only, no octal option
; No-load flag (bit 5): 1=display-only, crew cannot change value
; ============================================================================

						# NN	NORMAL NOUNS
# Page 275
; N00-N39: Normal nouns with standard component formats
; These nouns use consistent display formats across all components.
NNTYPTAB	OCT	00000			# 00	NOT IN USE
		OCT	04040			# 01	3COMP FRACTIONAL
		OCT	04140			# 02	3COMP WHOLE
		OCT	04102			# 03	3COMP CDU DEGREES
		OCT	0			# 04	SPARE
		OCT	00504			# 05	1COMP DPDEG(360)
; N06-N10: Octal-only nouns for displaying raw memory or register contents
		OCT	02000			# 06	2COMP OCTAL ONLY
		OCT	04000			# 07 	3COMP OCTAL ONLY
		OCT	04000			# 08	3COMP OCTAL ONLY (Alarms)
		OCT	04000			# 09	3COMP OCTAL ONLY
		OCT	00000			# 10	1COMP OCTAL ONLY
; N11-N16: HMS time formats (hours:minutes:seconds) for mission elapsed time
; Decimal-only display (24xxx codes) prevents confusing octal time display
		OCT	24400			# 11	3COMP HMS (DEC ONLY)
		OCT	02000			# 12	2COMP OCTAL ONLY
		OCT	24400			# 13	3COMP HMS (DEC ONLY)
		OCT	0			# 14	SPARE
		OCT	00000			# 15	1COMP OCTAL ONLY
		OCT	24400			# 16	3COMP HMS (DEC ONLY) (Mission time)
; N17-N22: CDU degree formats for gimbal angles and attitude display
; XXX.XX degree format with 0-360 degree range
		OCT	04102			# 17 	3COMP CDU DEG (Velocity)
		OCT	04102			# 18	3COMP CDU DEG
		OCT	04102			# 19	3COMP CDU DEG
		OCT	04102			# 20	3COMP CDU DEGREES (ICDU angles)
		OCT	04140			# 21	3COMP WHOLE (Incremental velocity)
		OCT	04102			# 22	3COMP CDU DEGREES
		OCT	00000			# 23 	SPARE
; N24-N39: Additional time formats for mission planning and event timing
; HMS format critical for TLI, LOI, PDI, TEI burn timing displays
		OCT	24400			# 24	3COMP HMS (DEC ONLY)
		OCT	04140			# 25	3COMP WHOLE
		OCT	04000			# 26	3COMP OCTAL ONLY
		OCT	00140			# 27	1COMP WHOLE
		OCT	00000			# 28	SPARE
		OCT	20102			# 29	1COMP CDU DEG (DEC ONLY)
		OCT	04140			# 30	3COMP WHOLE
		OCT	24400			# 31	3COMP HMS (DEC ONLY)
		OCT	24400			# 32 	3COMP HMS (DEC ONLY)
		OCT	24400			# 33 	3COMP HMS (DEC ONLY) (Event time)
		OCT	24400			# 34	3COMP HMS (DEC ONLY)
		OCT	24400			# 35 	3COMP HMS (DEC ONLY) (AGS time)
		OCT	24400			# 36 	3COMP HMS (DEC ONLY) (GET/mission time)
		OCT	24400			# 37 	3COMP HMS (DEC ONLY)
		OCT	24400			# 38 	3COMP HMS (DEC ONLY)
		OCT	24400			# 39	3COMP HMS (DEC ONLY)
# END OF NNTYPTAB FOR NORMAL NOUNS

; ============================================================================
; SECTION: NNTYPTAB - MIXED NOUNS (N40-N99)
;
; COMMENT-ONLY READERS: Mixed nouns displayed different types of related
; information together - for example, N42 showed both position AND velocity
; on the same display, letting Armstrong and Aldrin monitor multiple critical
; parameters simultaneously during descent and rendezvous operations.
;
; CODE-ALONG READERS: Mixed nouns combine different scale factors and display
; formats within a single noun. Each component can have its own SF constant
; code (position, velocity, angle, time, etc.). The component code and SF
; routine code in bits 1-10 specify the format, while bits 11-15 encode
; which SF constant applies to each of the three display registers.
; ============================================================================

						# NN	MIXED NOUNS

; N40: Time + Velocity Display (MIN/SEC, VEL3, VEL3)
; Used during powered descent to show elapsed time alongside LM descent velocities.
; R1: Minutes/seconds, R2: Horizontal velocity (ft/sec), R3: Vertical velocity (ft/sec)
; Decimal only, no keyboard load permitted - display-only during critical phases.
		OCT	24500			# 40	3COMP	MIN/SEC, VEL3, VEL3
						#		(NO LOAD, DEC ONLY)
; N41: IMU Angle + Elevation (CDU DEG, ELEV DEG)
; R1: IMU gimbal angle in degrees, R2: Elevation angle (max 89.999°)
; Used for alignment procedures and attitude monitoring.
		OCT	00542			# 41	2COMP	CDU DEG, ELEV DEG
; N42: Position + Velocity Vector (POS4, POS4, VEL3)
; Critical rendezvous noun showing relative position and approach rate.
; R1: Range (nautical miles), R2: Range rate, R3: Line-of-sight velocity (ft/sec)
; Displayed during CSM-LM rendezvous after lunar ascent. Decimal only.
		OCT	24410			# 42	3COMP	POS4, POS4, VEL3
						#		(DEC ONLY)
; N43: Angles + Position (DPDEG(360), DPDEG(360), POS4)
; R1: Azimuth angle (0-360°), R2: Elevation angle (0-360°), R3: Range (naut mi)
; Used for tracking and targeting computations. Decimal only.
		OCT	20204			# 43	3COMP	DPDEG(360), DPDEG(360), POS4
# Page 276
						#		(DEC ONLY)
; N44: Position + Time (POS4, POS4, MIN/SEC)
; R1: Downrange position, R2: Crossrange position, R3: Time to event (min:sec)
; Display-only noun for trajectory monitoring. Decimal only, no load.
		OCT	00410			# 44	3COMP	POS4, POS4, MIN/SEC
						#		(NO LOAD, DEC ONLY)
; N45: Mixed Data Display (2INT, MIN/SEC, DPDEG(360))
; R1: Two integer values, R2: Time (min:sec), R3: Angle (0-360°)
; General-purpose mixed display. Decimal only, no load.
		OCT	10000			# 45	3COMP	2INT, MIN/SEC, DPDEG(360)
						#		(NO LOAD, DEC ONLY)
		OCT	00000			# 46	2COMP 	OCTAL ONLY FOR EACH
		OCT	00306			# 47	2COMP	WEIGHT2 FOR EACH
						#		(DEC ONLY)
		OCT	00614			# 48	2COMP	TRIM DEG, TRIM DEG
						#		(DEC ONLY)
; N49: Landing Radar Data (POS4, VEL3, WHOLE)
; R1: Altitude (naut mi), R2: Descent rate (ft/sec), R3: Counter/flag value
; Critical during lunar descent for monitoring radar-derived altitude and velocity.
		OCT	00510			# 49	3COMP	POS4, VEL3, WHOLE
						#		(DEC ONLY)
; N50: Position + Time Countdown (POS6, POS4, MIN/SEC)
; R1: Primary position, R2: Secondary position, R3: Time remaining (min:sec)
; Used for burn monitoring and event sequencing. Display only. Decimal only.
		OCT	00417			# 50	3COMP	POS6, POS4, MIN/SEC
						#		(NO LOAD, DEC ONLY)
		OCT	00204			# 51	2COMP	DPDEG(360), DPDEG(360)
						#		(DEC ONLY)
		OCT	00004			# 52	1COMP	DPDEG(360)
; N53: Integrated Navigation Data (POS5, VEL3, DPDEG(360))
; R1: Position (naut mi), R2: Velocity (ft/sec), R3: Angle (0-360°)
; Combines position, velocity, and directional data for comprehensive state display.
		OCT	10507			# 53	3COMP	POS5, VEL3, DPDEG(360)
						#		(DEC ONLY)
; N54: Alternate Navigation Display (POS5, VEL3, DPDEG(360))
; Same format as N53, used for alternate reference frame or backup navigation data.
		OCT	10507			# 54	3COMP	POS5, VEL3, DPDEG(360)
						#		(DEC ONLY)
		OCT	10200			# 55	3COMP	WHOLE, DPDEG(360), DPDEG(360)
						#		(DEC ONLY)
		OCT	00444			# 56	2COMP	DPDEG(360), VEL2
						#		(DEC ONLY)
		OCT	00010			# 57	1COMP	POS4
						#		(DEC ONLY)
; N58: Descent Position + Velocities (POS4, VEL3, VEL3)
; R1: Altitude/range (naut mi), R2: Horizontal velocity, R3: Vertical velocity (ft/sec)
; Key descent monitoring noun showing position and two-axis velocity components.
		OCT	24510			# 58	3COMP	POS4, VEL3, VEL3
						#		(DEC ONLY)
; N59: Three-Axis Velocity Display (VEL3, VEL3, VEL3)
; R1: X-axis velocity, R2: Y-axis velocity, R3: Z-axis velocity (all in ft/sec)
; Complete velocity vector display in spacecraft reference frame. Decimal only.
		OCT	24512			# 59	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
; N60: Event Counter + Velocity + Angle (WHOLE, VEL2, DPDEG(360))
; R1: Integer count/flag, R2: Velocity (ft/sec), R3: Directional angle (0-360°)
; Multi-purpose display combining discrete state with continuous parameters.
		OCT	10440			# 60	3COMP	WHOLE, VEL2, DPDEG(360)
						# 		(DEC ONLY)
		OCT	00204			# 61	3COMP 	DPDEG(360), DPDEG(360), WHOLE
						#		(DEC ONLY)
		OCT	20451			# 62	3COMP	VEL2, VEL2, POS4
						#		(DEC ONLY)
; N63: Range + Velocity + Time Display (POS6, VEL2, MIN/SEC)
; R1: Range/altitude (naut mi), R2: Velocity (ft/sec), R3: Mission elapsed time
; Display only - combines position, velocity, and timing for mission phase monitoring.
		OCT	00457			# 63	3COMP	POS6, VEL2, MIN/SEC
						#		(NO LOAD, DEC ONLY)
; N64: Entry Dynamics Display (DRAG ACCEL, VEL2, POS6)
; R1: Drag acceleration (G-forces), R2: Entry velocity, R3: Altitude (naut mi)
; Critical during CM atmospheric reentry showing deceleration and descent parameters.
		OCT	36460			# 64	3COMP	DRAG ACCEL, VEL2, POS6
						#		(DEC ONLY)
; N65: Time Display - Hours:Minutes:Seconds (HMS)
; R1: Hours, R2: Minutes (mod 60), R3: Seconds (mod 60) with 0.XX precision
; Standard mission elapsed time display used throughout Apollo 11 mission phases.
		OCT	00000			# 65	3COMP	HMS (DEC ONLY)
		OCT	37044			# 66	3COMP 	DPDEG(360), POS8, POS6
						#		(DEC ONLY)
		OCT	10217			# 67	3COMP	POS6, DPDEG(360), DPDEG(360)
						#		(DEC ONLY)
; N68: Entry Navigation Display (DPDEG(360), VEL2, VEL/2VS)
; R1: Entry angle (0-360°), R2: Inertial velocity, R3: Velocity relative to atmosphere
; Used during transearth coast and entry interface to monitor trajectory parameters.
		OCT	34444			# 68	3COMP	DPDEG(360), VEL2, VEL/2VS
						#		(DEC ONLY)
; N69: Entry Deceleration Monitor (DPDEG(360), DRAG ACCEL, VEL/2VS)
; R1: Flight path angle, R2: Drag acceleration (G-forces), R3: Relative velocity
; Critical CM reentry display combining attitude, deceleration, and velocity for
; safe corridor monitoring during July 24, 1969 Pacific Ocean splashdown approach.
		OCT	35004			# 69	3COMP	DPDEG(360), DRAG ACCEL,VEL/2VS
						#		(DEC ONLY)
# Page 277
		OCT	00000			# 70	3COMP	OCTAL ONLY FOR EACH
		OCT	0			# 71	3COMP	OCTAL ONLY FOR EACH
		OCT	00404			# 72	3COMP	DPDEG(360), POS4, WHOLE
						#		(DEC ONLY)
; N73: Rendezvous Navigation (POS4, VEL2, DPDEG(360))
; R1: Range to target (naut mi), R2: Closing velocity, R3: Phase angle
; Used during Eagle's rendezvous with Columbia after lunar ascent.
		OCT	10450			# 73	3COMP	POS4, VEL2, DPDEG(360)
; N74: Entry Phase Monitor (DPDEG(360), VEL2, DRAG ACCEL)
; R1: Roll angle, R2: Inertial velocity, R3: Sensed deceleration (G-forces)
; Entry corridor monitoring combining attitude control, velocity, and drag forces.
		OCT	40444			# 74	3COMP	DPDEG(360), VEL2, DRAG ACCEL
; N75: Landing/Ascent Timing Display (POS4, MIN/SEC, MIN/SEC)
; R1: Altitude/range (naut mi), R2: Time-to-go, R3: Elapsed time in phase
; Display only - shows position and dual timing for powered flight monitoring.
		OCT	00010			# 75	3COMP	POS4, MIN/SEC, MIN/SEC
#						# 		(NO LOAD, DEC ONLY)
		OCT	0			# 76		SPARE
		OCT	0			# 77		SPARE
		OCT	0			# 78		SPARE
		OCT	0			# 79		SPARE
; N80: Time + Dual Velocity Display (MIN/SEC, VEL2, VEL2)
; R1: Mission elapsed time (min:sec), R2: Primary velocity, R3: Secondary velocity
; Display only - timing combined with two independent velocity measurements.
		OCT	22440			# 80	3COMP	MIN/SEC, VEL2, VEL2
						#		(NO LOAD, DEC ONLY)
; N81: Three-Axis Velocity Vector #1 (VEL3, VEL3, VEL3)
; R1: X velocity, R2: Y velocity, R3: Z velocity (all ft/sec with 0.1 precision)
; Complete spacecraft velocity vector in reference coordinate frame.
		OCT	24512			# 81	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
; N82: Three-Axis Velocity Vector #2 (VEL3, VEL3, VEL3)
; Alternate velocity display format - same as N81, different context/usage.
		OCT	24512			# 82	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
; N83: Three-Axis Velocity Vector #3 (VEL3, VEL3, VEL3)
; Another velocity vector display - multiple nouns allow different velocity
; sources (computed, desired, error) to be monitored independently.
		OCT	24512			# 83	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
; N84: Three-Axis Velocity Vector #4 (VEL3, VEL3, VEL3)
; Fourth velocity display variant for comprehensive multi-source monitoring.
		OCT	24512			# 84	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
; N85: Three-Axis Velocity Vector #5 (VEL3, VEL3, VEL3)
; Fifth independent velocity display - extensive velocity monitoring capability
; supports complex navigation state estimation and guidance error tracking.
		OCT	24512			# 85	3COMP	VEL3 FOR EACH
						#		(DEC ONLY)
; N86: High-Precision Velocity Triple (VEL2, VEL2, VEL2)
; R1: Component 1, R2: Component 2, R3: Component 3 (all XXXXX. ft/sec precision)
; Higher precision velocity display (whole ft/sec vs. 0.1 ft/sec) for fine control.
		OCT	22451			# 86	3COMP	VEL2 FOR EACH
						#		(DEC ONLY)
; N87: Gimbal Angles - IMU and Optics (CDU DEG, Y OPTICS DEG)
; R1: CDU gimbal angle (XXX.XX degrees), R2: Y-axis optics angle (XX.XXX deg, max 89.999)
; Two-component display combining IMU platform orientation with optical telescope pointing.
		OCT	00102			# 87	2COMP	CDU DEG, Y OPTICS DEG
; N88: Three Fractional Values (FRAC, FRAC, FRAC)
; R1: Fraction 1, R2: Fraction 2, R3: Fraction 3 (straight fractional format)
; General-purpose fractional display for normalized values, ratios, or scale factors.
		OCT	0			# 88	3COMP	FRAC FOR EACH
						#		(DEC ONLY)
; N89: Attitude + Range Display (DPDEG(90), DPDEG(90), POS5)
; R1: Pitch angle (±90°), R2: Yaw angle (±90°), R3: Range/altitude (naut mi)
; Combined attitude and position monitoring for navigation cross-checks.
		OCT	16143			# 89	3COMP 	DPDEG(90), DPDEG(90), POS5
						#		(DEC ONLY)
; N90: Navigation State Composite (POS5, VEL3, DPDEG(360))
; R1: Position/range (XXX.XX naut mi), R2: Velocity (XXXX.X ft/sec), R3: Heading angle (0-360°)
; Complete 2D navigation state showing position, speed, and direction in single display.
		OCT	10507			# 90	3COMP	POS5, VEL3, DEPDEG(360)
						#		(DEC ONLY)
; N91: IMU and Optics Gimbal Angles #1 (CDUDEG, YOPTICS DEG)
; R1: IMU CDU gimbal angle (XXX.XX°), R2: Optics Y-axis angle (XX.XXX°)
; Variant of N87 - different memory sources for same format display.
		OCT	00102			# 91	2COMP	CDUDEG, YOPTICS DEG
; N92: IMU and Optics Gimbal Angles #2 (CDUDEG, YOPTICS DEG)
; Another gimbal angle pair display - multiple nouns allow simultaneous monitoring
; of computed, desired, and actual gimbal positions.
		OCT	00102			# 92	2COMP	CDUDEG, YOPTICS DEG
; N93: Three-Axis Attitude Angles (DPDEG(90), DPDEG(90), DPDEG(90))
; R1: Angle 1 (±90°), R2: Angle 2 (±90°), R3: Angle 3 (±90°)
; Triple attitude display - pitch, roll, yaw or other three-angle sets.
		OCT	06143			# 93	3COMP	DPDEG(90) FOR EACH
; N94: IMU and Optics Gimbal Angles #3 (CDUDEG, YOPTICS DEG)
; Third variant of gimbal angle display for comprehensive platform monitoring.
		OCT	00102			# 94	2COMP	CDUDEG, YOPTICS DEG
; N95: IMU Three-Axis Gimbal Angles #1 (CDU DEG, CDU DEG, CDU DEG)
; R1: Inner gimbal, R2: Middle gimbal, R3: Outer gimbal (all XXX.XX degrees)
; Complete IMU platform orientation in gimbal angle representation.
		OCT	04102			# 95	3COMP	CDU DEG FOR EACH
; N96: IMU Three-Axis Gimbal Angles #2 (CDU DEG, CDU DEG, CDU DEG)
; Alternate three-gimbal display - allows comparison of computed vs actual orientation.
		OCT	04102			# 96	3COMP	CDU DEG FOR EACH
; N97: Three Integer Display (WHOLE, WHOLE, WHOLE)
; R1: Integer 1, R2: Integer 2, R3: Integer 3 (all whole numbers)
; General-purpose multi-integer display for counters, flags, or discrete states.
		OCT	00000			# 97	3COMP	WHOLE FOR EACH
; N98: Mixed Integer and Fractional (WHOLE, FRAC, WHOLE)
; R1: Integer value, R2: Fractional value (0.xxxxx), R3: Integer value
; Hybrid display combining discrete and continuous parameters.
		OCT 	00000			# 98	3COMP	WHOLE, FRAC, WHOLE
; N99: State Vector in Feet (POS9, VEL4, WHOLE)
; R1: Position in feet (XXXXX.), R2: Velocity (XXXX.X ft/sec), R3: Integer flag/mode
; High-resolution state display using feet instead of nautical miles. Used for
; close-proximity operations where foot-level precision needed. Decimal only.
		OCT	01162			# 99	3COMP	POS9, VEL4, WHOLE
						#		(DEC ONLY)
; ============================================================================
; END OF NNTYPTAB - MIXED NOUNS SECTION COMPLETE
;
; The NNTYPTAB table defines display format and scaling for all 100 DSKY nouns
; (N00-N99). Nouns specify WHAT information to display, while verbs specify HOW
; to process that information. During Apollo 11 mission, crew used these nouns
; to monitor spacecraft state throughout all phases from Earth orbit through
; lunar landing and return.
; ============================================================================

; ============================================================================
; TRANSITION: From Noun Type Definitions to Scale Factor Constants
;
; Having defined WHAT to display (noun types and formats in NNADTAB/NNTYPTAB),
; the next tables define the precise scaling constants used to convert between
; AGC internal representation and crew-readable DSKY displays. These constants
; enable accurate physical unit conversions throughout the mission.
; ============================================================================

; ============================================================================
; SFINTAB - SCALE FACTOR INPUT CONSTANT TABLE
;
; This table contains the actual scaling constants referenced by SF constant
; codes in NNTYPTAB entries. Each constant is a double-precision (two-word)
; octal value defining the multiplication or division factor for input scaling.
;
; Table Organization:
; - Paired OCT values (two consecutive words per constant)
; - Indexed by SF constant code from NNTYPTAB
; - Constants are fixed-point scaled values optimized for AGC arithmetic
;
; Scale Factor Constants Defined:
; 00000 = WHOLE / DP TIME (SEC): Time in seconds format (XXX.XX sec)
; 00001 = SPARE (reserved for future use)
; 00010 = CDU DEGREES / Y OPTICS DEGREES: Angular measurements from IMU/optics
; 00011 = DP DEGREES (90): Double-precision angles up to 90 degrees (XX.XXX)
; 00100 = DP DEGREES (360): Full circle angles (XXX.XX degrees)
; 00101 = DEGREES (180): Half-circle angles (XXX.XX degrees, ±180° range)
; 00110 = WEIGHT2: Spacecraft mass in pounds (XXXXX. lbs)
; 00111 = POSITION5: Position in nautical miles (XXX.XX n.mi.)
; 01000 = POSITION4: Position in nautical miles (XXXX.X n.mi.)
; 01001 = VELOCITY2: Velocity in feet/second (XXXXX. ft/sec)
; 01010 = VELOCITY3: Velocity in feet/second (XXXX.X ft/sec)
; 01011 = ELEVATION DEGREES: Elevation angles (max 89.999 degrees)
; 01100 = TRIM DEGREES: Gimbal trim angles (XXX.XX degrees)
; 01101 = INERTIA / THRUST MOMENT: Rotational inertia or thrust torque
; 01110 = VELOCITY/2VS: Velocity scaled by 2*circular orbital velocity
; 01111 = POSITION6: Alternate position scaling (XXXX.X n.mi.)
; 10000 = DRAG ACCELERATION: Atmospheric drag in g-units (XXX.XX g)
; 10001 = POSITION8: Position in nautical miles (XXXX.X n.mi.)
; 10010 = POSITION9: Position in feet (XXXXX. ft) for close-range operations
; 10011 = VELOCITY4: Velocity in feet/second (XXXX.X ft/sec)
;
; During Apollo 11, these constants ensured accurate conversion of:
; - Navigation state vectors (position/velocity in proper units)
; - IMU gimbal angles (CDU degrees to display degrees)
; - Spacecraft mass changes due to propellant consumption
; - Velocity increments for maneuver planning (delta-V displays)
; - Time values for countdown and mission elapsed time displays
;
; The precision of these constants (many scaled by powers of 2 for efficient
; AGC fixed-point arithmetic) was critical for accurate guidance during lunar
; descent and ascent, where position errors of even a few feet could affect
; landing site selection or rendezvous trajectory.
; ============================================================================

; SF Constant 00000: WHOLE / DP TIME (SEC)
; Scaling for time display in seconds with centisecond precision (XXX.XX sec)
; Used by nouns displaying mission elapsed time, countdown timers, etc.
SFINTAB		OCT	00006			# WHOLE, DP TIME (SEC)
		OCT	03240
; SF Constant 00001: SPARE (reserved)
		OCT	00000			# SPARE
		OCT	00000
# Page 278
; SF Constant 00010: CDU DEGREES / Y OPTICS DEGREES
; Angular scaling for IMU gimbal angles and optics telescope pointing
; CDU (Coupling Data Unit) reads gimbal shaft positions for attitude determination
		OCT	00000			# CDU DEGREES, Y OPTICS DEGREES
		OCT	00000			#	(SFCONS IN DEGINSF, OPTDEGIN)
; SF Constant 00011: DP DEGREES (90)
; Double-precision angle scaling for ±90 degree range (XX.XXX degrees)
; Used for elevation angles, pitch/roll attitudes with sub-degree precision
		OCT	10707			# DP DEGREES (90)
		OCT	03435			#	UPPED BY 1
; SF Constant 00100: DP DEGREES (360)
; Full circle angle scaling (XXX.XX degrees), binary point between bits 11-12
; Used for heading, azimuth, and full rotational measurements
		OCT	13070			# DP DEGREES (360)(POINT BETWN BITS 11-12)
		OCT	34345			#	UPPED BY 1
; SF Constant 00101: DEGREES (180)
; Half-circle angle scaling (XXX.XX degrees) for ±180° range
; Used for spacecraft attitude displays where ±180° convention applies
		OCT	00005			# DEGREES (180)
		OCT	21616
; SF Constant 00110: WEIGHT2
; Spacecraft mass in pounds (XXXXX. lbs)
; Critical during Apollo 11 for tracking propellant consumption in descent/ascent
; Mass changes affected guidance equations and thrust-to-weight calculations
		OCT	26113			# WEIGHT2
		OCT	31713
; SF Constant 00111: POSITION5
; Position in nautical miles (XXX.XX n.mi.)
; Standard range unit for cislunar navigation displays
		OCT	00070			# POSITION5
		OCT	20460
; SF Constant 01000: POSITION4
; Position in nautical miles (XXXX.X n.mi.)
; Higher magnitude position scaling for translunar/transearth trajectory displays
		OCT	01065			# POSITION4
		OCT	05740
; SF Constant 01001: VELOCITY2
; Velocity in feet/second (XXXXX. ft/sec), binary point between bits 11-12
; High-range velocity scaling for orbital and translunar velocities
; Apollo 11 translunar coast: ~3000 ft/sec relative to Earth
		OCT	11414			# VELOCITY2 	(POINT BETWN BITS 11-12)
		OCT	31463
; SF Constant 01010: VELOCITY3
; Velocity in feet/second (XXXX.X ft/sec)
; Medium-range velocity with decimal precision for maneuver delta-V displays
		OCT	07475			# VELOCITY3
		OCT	16051
; SF Constant 01011: ELEVATION DEGREES
; Elevation angle with maximum 89.999 degrees (avoids singularity at 90°)
; Used for star sighting elevation angles during IMU alignment
		OCT	00001			# ELEVATION DEGREES
		OCT	03434
; SF Constant 01100: TRIM DEGREES
; Engine gimbal trim angles (XXX.XX degrees)
; Displayed during SPS and descent engine burns for thrust vector control
		OCT	00002			# TRIM DEGREES
		OCT	22245
; SF Constant 01101: INERTIA / THRUST MOMENT
; Rotational inertia (XXXXXBB. slug-ft²) or thrust moment (XXXXXBB. ft-lbs)
; Used for spacecraft dynamics calculations and control system displays
		OCT	00014			# INERTIA, THRUST MOMENT
		OCT	35607
; SF Constant 01110: VELOCITY/2VS
; Velocity scaled by 2 times circular orbital velocity (XXXXX. ft/sec)
; Used for rendezvous displays showing relative velocity as fraction of orbital speed
		OCT	07606			# VELOCITY/2VS
		OCT	06300
; SF Constant 01111: POSITION6
; Alternate position scaling in nautical miles (XXXX.X n.mi.)
		OCT	16631			# POSITION 6
		OCT	11307
; SF Constant 10000: DRAG ACCELERATION
; Atmospheric drag acceleration in g-units (XXX.XX g)
; Used during Earth entry phase to monitor deceleration forces
; Binary point between bits 7-8 for proper scaling
		OCT	12000			# DRAG ACCELERATION (POINT BETWN BITS 7-8)
		OCT	00000
; SF Constant 10001: POSITION8
; Position in nautical miles (XXXX.X n.mi.)
		OCT	27176			# POSITION 8
		OCT	14235
; SF Constant 10010: POSITION9
; Position in feet (XXXXX. ft) for close-range operations
; Used during final lunar landing approach when foot-level precision required
; Armstrong used displays with this scaling during final descent below 500 feet
		2DEC	30480 B-19		# POSITION 9

; SF Constant 10011: VELOCITY4
; Velocity in feet/second (XXXX.X ft/sec)
; Close-range velocity scaling for landing and docking operations
		2DEC	30.48 B-7		# VELOCITY4

						# END OF SFINTAB

; ============================================================================
; TRANSITION: From Input Scale Factors to Output Scale Factors
;
; Having defined the scaling constants for converting crew keyboard inputs into
; AGC internal representation (SFINTAB), the next table provides the inverse
; transformations. SFOUTAB contains the scaling constants for converting AGC
; internal values back to human-readable DSKY display formats. These output
; scale factors ensure that computed navigation states, velocities, and mission
; parameters appear correctly formatted with appropriate decimal points and units.
; ============================================================================

; ============================================================================
; SFOUTAB - SCALE FACTOR OUTPUT CONSTANT TABLE
;
; This table contains the output scaling constants that convert AGC internal
; fixed-point values to human-readable DSKY display formats. Where SFINTAB
; handled input conversion (keyboard → computer), SFOUTAB handles output
; conversion (computer → display).
;
; Table Organization:
; - Paired OCT values (two consecutive words per constant)
; - Indexed by SF constant code from NNTYPTAB
; - Constants are inverses or complements of SFINTAB values
; - Optimized for AGC fixed-point multiplication/division
;
; Output Scale Factors Mirror SFINTAB Structure:
; - Same SF constant codes (00000-10011)
; - Inverse scaling relationships for bidirectional conversion
; - Decimal point positioning for proper display formatting
; - Unit conversions matching physical measurement conventions
;
; Critical Mission Usage During Apollo 11:
;
; During Translunar Coast:
; - Position displays showed distance from Earth in nautical miles
; - Velocity relative to Earth updated periodically
; - Mission elapsed time displayed continuously
;
; During Lunar Descent (July 20, 1969):
; - Altitude displays (POSITION9 scaling for feet) updated 10 times/second
; - Velocity displays (VELOCITY3/VELOCITY4) showed descent rate to crew
; - Armstrong and Aldrin monitored these values throughout powered descent
; - DSKY displayed "06 33" for altitude-rate noun during final approach
;
; During Lunar Ascent (July 21, 1969):
; - Velocity displays showed ascent progress toward orbital insertion
; - Time-to-apogee predictions updated continuously
; - Rendezvous radar range/range-rate displayed for Columbia tracking
;
; The precision of these output scalings was essential for crew situational
; awareness. During the famous 1202 alarm at 102:38:26 MET, Armstrong and
; Aldrin relied on correctly formatted altitude/velocity displays to maintain
; confidence in the computer's guidance while Houston analyzed the alarm.
; ============================================================================

; SF Output Constant 00000: WHOLE / DP TIME (SEC)
; Output scaling converts AGC internal time (centiseconds) to display format
; Displays time as XXX.XX seconds on DSKY R1, R2, R3 registers
SFOUTAB		OCT	05174			# WHOLE, DP TIME (SEC)
		OCT	13261
; SF Output Constant 00001: SPARE (reserved)
		OCT	00000			# SPARE
		OCT	00000
; SF Output Constant 00010: CDU DEGREES / Y OPTICS DEGREES
; Converts AGC internal gimbal angles to degrees for DSKY display
; IMU CDU shaft angles displayed for crew platform alignment verification
		OCT	00000			# CDU DEGREES, Y OPTICS DEGREES
		OCT	00000			#	(SFCONS IN DEGOUTSF, OPTDEGOUT)
; SF Output Constant 00011: DP DEGREES (90)
; Converts internal angle to ±90° display format (XX.XXX degrees)
; Binary point between bits 7-8 for proper decimal positioning
; Used for elevation angles during star sightings and optical alignments
		OCT	00714			# DP DEGREES (90) (POINT BETWN BITS 7-8)
		OCT	31463
; SF Output Constant 00100: DP DEGREES (360)
; Converts internal angle to full-circle 360° format (XXX.XX degrees)
; Used for azimuth, heading, and rotational position displays
		OCT	13412			# DP DEGREES (360)
		OCT	07534
; SF Output Constant 00101: DEGREES (180)
; Converts internal angle to ±180° display format (XXX.XX degrees)
; Standard spacecraft attitude display convention
		OCT	05605			# DEGREES (180)
# Page 279
		OCT	03656
; SF Output Constant 00110: WEIGHT2
; Converts internal mass representation to pounds (XXXXX. lbs)
; During Apollo 11 descent, displayed remaining propellant mass
; Critical for fuel management during landing approach
		OCT	00001			# WEIGHT2
		OCT	16170
; SF Output Constant 00111: POSITION5
; Converts internal position to nautical miles (XXX.XX n.mi.)
; Used throughout cislunar coast to display distance from Earth/Moon
		OCT	00441			# POSITION5
		OCT	34306
; SF Output Constant 01000: POSITION4
; Converts internal position to nautical miles (XXXX.X n.mi.)
; Higher-magnitude position scaling for translunar trajectory displays
		OCT	07176			# POSITION4
		OCT	21603
; SF Output Constant 01001: VELOCITY2
; Converts internal velocity to feet/second (XXXXX. ft/sec)
; High-range velocity for orbital and translunar velocities
; Apollo 11 Earth departure velocity: ~35,000 ft/sec
		OCT	15340			# VELOCITY2
		OCT	15340
; SF Output Constant 01010: VELOCITY3
; Converts internal velocity to feet/second (XXXX.X ft/sec)
; Binary point between bits 7-8 for decimal precision
; Used during lunar descent to display descent rate
; Armstrong monitored velocity displays during final approach
		OCT	01031			# VELOCITY3	(POINT BETWN BITS 7-8)
		OCT	21032
; SF Output Constant 01011: ELEVATION DEGREES
; Converts internal elevation angle to degrees (max 89.999°)
; Prevents display singularity at 90° elevation
		OCT	34631			# ELEVATION DEGREES
		OCT	23146
; SF Output Constant 01100: TRIM DEGREES
; Converts internal gimbal position to trim angle degrees (XXX.XX)
; Displayed during engine burns showing thrust vector control gimbal positions
		OCT	14340			# TRIM DEGREES
		OCT	24145
; SF Output Constant 01101: INERTIA / THRUST MOMENT
; Converts internal dynamics values to slug-ft² or ft-lbs (XXXXXBB.)
; Used for spacecraft rotational dynamics and control system displays
		OCT	02363			# INERTIA, THRUST MOMENT
		OCT	03721
; SF Output Constant 01110: VELOCITY/ZVS
; Converts internal velocity to fraction of orbital velocity (XXXXX. ft/sec)
; Rendezvous displays showed relative velocity as multiple of circular speed
		OCT	20373			# VELOCITY/ZVS
		OCT	02122
; SF Output Constant 01111: POSITION6
; Converts internal position to nautical miles (XXXX.X n.mi.)
; Binary point between bits 7-8
		OCT	00424			# POSITION 6	(POINT BETWN BITS 7-8)
		OCT	30446
; SF Output Constant 10000: DRAG ACCELERATION
; Converts internal acceleration to g-units (XXX.XX g)
; Displayed during Earth entry showing deceleration forces on crew
; Peaked at ~6.5g during Apollo 11 reentry on July 24, 1969
		OCT	00631			# DRAG ACCELERATION
		OCT	23146
; SF Output Constant 10001: POSITION8
; Converts internal position to nautical miles (XXXX.X n.mi.)
		OCT	00260			# POSITION 8
		OCT	06213
; SF Output Constant 10010: POSITION9
; Converts internal position to feet (XXXXX. ft)
; Critical during final lunar landing approach below 500 feet
; Armstrong relied on altitude display in feet during manual site selection
; Display updated continuously: "300 feet, 2½ down..." callouts by Aldrin
		2DEC	17.2010499 B-7		# POSITION 9

; SF Output Constant 10011: VELOCITY4
; Converts internal velocity to feet/second (XXXX.X ft/sec)
; Close-range velocity scaling for final landing and docking
; "Forward velocity 2 feet per second" - typical descent rate display
		2DEC	.032808399		# VELOCITY4

						# END OF SFOUTAB

; ============================================================================
; TRANSITION: From Scale Factor Tables to Noun Address Tables
;
; Having defined the scale factor constants for bidirectional data conversion
; (SFINTAB and SFOUTAB), the next table provides the actual memory addresses
; where noun data resides. IDADDTAB (IDentifier ADDress TABle) maps each noun
; number to the specific erasable memory locations containing the noun's
; component values. This indirection allows flexible data routing - multiple
; nouns can display the same underlying data with different formatting, and
; mission programs can update data independently of DSKY display logic.
; ============================================================================

; ============================================================================
; IDADDTAB - NOUN ADDRESS IDENTIFICATION TABLE
;
; This table maps each noun number (40-99) to the erasable memory addresses
; containing the noun's data components. Each noun has up to three component
; addresses, corresponding to the DSKY's three display registers (R1, R2, R3).
;
; Table Organization:
; - Each noun occupies 3 consecutive entries (3 components)
; - Entries use ECADR (Erasable Constant ADdRess) format
; - OCT 0 indicates spare/unused component
; - Nouns 01-39 use NNADTAB (machine address table from earlier)
; - Nouns 40-99 use IDADDTAB (this symbolic address table)
;
; Why Two Noun Address Tables?
; - NNADTAB (Nouns 01-39): Direct machine addresses for simple/fixed nouns
;   These nouns typically display constant or rarely-changed data
; - IDADDTAB (Nouns 40-99): Symbolic addresses (ECADR) for complex nouns
;   These nouns display dynamically-updated mission data that moves in memory
;   or requires flexible addressing across memory banks
;
; Address Format (ECADR):
; - ECADR macro generates bank-independent erasable memory references
; - Allows noun data to reside in any erasable bank (E-bank switching)
; - Runtime resolution provides correct absolute address during execution
;
; Critical Apollo 11 Nouns in This Table:
;
; Noun 40 (TTOGO, VGDISP, DVTOTAL):
; - Time-to-go, velocity-to-go, delta-V total for current maneuver
; - Displayed throughout powered flight phases
; - During TLI burn: showed remaining burn time and target velocity
;
; Noun 42 (HAPO, HPER, VGDISP):
; - Apoapsis altitude, periapsis altitude, velocity
; - Displayed after LOI burn to confirm lunar orbit insertion
; - "HAPO: 169.5 n.mi., HPER: 60.0 n.mi." confirmed successful orbit
;
; Noun 43 (LAT, LONG, ALT):
; - Latitude, longitude, altitude of current position
; - During lunar descent, showed landing site coordinates
; - Armstrong referenced coordinates when selecting final landing spot
;
; Noun 49 (RDOT, RRATE, RTHETA):
; - Range, range-rate, angle for rendezvous radar tracking
; - Used during Eagle's ascent to track Columbia in lunar orbit
; - Range data critical for rendezvous timing calculations
;
; Noun 50 (PIPTIME, DELVX, DELVY, DELVZ):
; - IMU PIPA (accelerometer) data for navigation updates
; - Continuously integrated to update spacecraft state vector
; - Foundation of all inertial navigation computations
;
; Noun 63 (RRANGE, RRDOT):
; - Rendezvous radar range and range-rate
; - Final approach displays during Eagle-Columbia docking
; - Collins monitored from Columbia, Aldrin from Eagle
;
; Noun 68 (TTOGO, DELVLVC):
; - Delta-V monitor for burn cutoff accuracy
; - Displayed during critical engine burns
; - "Shutdown" callouts based on TTOGO reaching zero
;
; Historical Context - 1202 Alarm and Noun Displays:
; During the famous 1202 alarm at 102:38:26 MET, Armstrong and Aldrin continued
; monitoring altitude and velocity nouns (using IDADDTAB addresses) despite the
; executive overflow. The correct display of descent data via these noun address
; mappings maintained crew confidence that guidance remained functional even as
; the alarm light flashed on the DSKY.
;
; Memory Management Note:
; The ECADR format in IDADDTAB allows noun data to move between erasable banks
; during program phase changes without breaking DSKY display logic. This
; flexibility proved essential when memory became constrained during mission
; additions after initial AGC software design.
; ============================================================================

						# NN	 SF CONSTANT		SF ROUTINE

; Noun 40: TIME-TO-GO / VELOCITY-TO-GO / DELTA-V TOTAL
; Component 1: TTOGO - Time remaining in current maneuver (minutes/seconds)
; Component 2: VGDISP - Velocity-to-be-gained in current burn (ft/sec)
; Component 3: DVTOTAL - Total delta-V accumulated in burn (ft/sec)
; Mission Usage: Displayed during all powered flight phases (TLI, LOI, TEI, descent, ascent)
; Format: MIN/SEC, VEL3, VEL3 with M/S, DP3, DP3 routines
IDADDTAB	ECADR	TTOGO			# 40	MIN/SEC			M/S
		ECADR	VGDISP			# 40	VEL3			DP3
		ECADR	DVTOTAL			# 40	VEL3			DP3

; Noun 41: CDU ANGLES / ELEVATION ANGLE
; Component 1: DSPTEM1 - Temporary display storage for CDU gimbal angle (degrees)
; Component 2: DSPTEM1+1 - Elevation angle for optical sighting or antenna pointing
; Component 3: Spare (unused component)
; Mission Usage: IMU gimbal angle displays, optical telescope elevation displays
; Format: CDU DEG, ELEV DEG with CDU, ARITH routines
		ECADR	DSPTEM1			# 41	CDU DEG			CDU
		ECADR	DSPTEM1 +1		# 41	ELEV DEG		ARTH
		OCT	0			# 41	SPARE COMPONENT

; Noun 42: APOAPSIS ALTITUDE / PERIAPSIS ALTITUDE / VELOCITY
; Component 1: HAPO - Apoapsis altitude (highest point in orbit, nautical miles)
; Component 2: HPER - Periapsis altitude (lowest point in orbit, nautical miles)
; Component 3: VGDISP - Current orbital velocity (ft/sec)
; Mission Usage: Critical for confirming LOI success - "169.5 x 60.0 n.mi." orbit
; Apollo 11 Context: After LOI burn July 19, Noun 42 confirmed successful lunar orbit
; Format: POS4, POS4, VEL3 with DP3, DP3, DP3 routines
		ECADR	HAPO			# 42	POS4			DP3
		ECADR	HPER			# 42	POS4			DP3
		ECADR	VGDISP			# 42 	VEL3			DP3

; Noun 43: LATITUDE / LONGITUDE / ALTITUDE
; Component 1: LAT - Latitude of current position (degrees, ±180°)
; Component 2: LONG - Longitude of current position (degrees, 0-360°)
; Component 3: ALT - Altitude above reference surface (nautical miles)
; Mission Usage: Displays spacecraft position over Earth or Moon
; Apollo 11 Context: During descent, showed landing site coordinates in Sea of Tranquility
; Armstrong referenced these coordinates when selecting final landing spot manually
; Format: DPDEG(360), DPDEG(360), POS4 with DP4, DP4, DP3 routines
		ECADR	LAT			# 43	DPDEG(360)		DP4
		ECADR	LONG			# 43	DPDEG(360)		DP4
		ECADR	ALT			# 43	POS4			DP3

; Noun 44: APOAPSIS ALTITUDE / PERIAPSIS ALTITUDE / TIME-FROM-PERIAPSIS
; Component 1: HAPOX - Apoapsis altitude above reference (nautical miles, XXXX.X)
; Component 2: HPERX - Periapsis altitude above reference (nautical miles, XXXX.X)
; Component 3: TFF - Time from periapsis passage (minutes:seconds display)
; Mission Usage: Extended orbit parameters with timing from lowest orbital point
; Used for monitoring orbital evolution and predicting next periapsis passage
; Format: POS4, POS4, MIN/SEC with DP3, DP3, M/S routines
		ECADR	HAPOX			# 44	POS4			DP3
		ECADR	HPERX			# 44	POS4			DP3
		ECADR	TFF			# 44	MIN/SEC			M/S

; Noun 45: VHF RANGE COUNTER / TIME-TO-GO / MIDDLE GIMBAL ANGLE
; Component 1: VHFCNT - VHF ranging counter value (2 integers D1D2, D4D5)
; Component 2: TTOGO - Time remaining to next event (minutes:seconds)
; Component 3: +MGA - Middle gimbal angle for IMU orientation (degrees, 0-360°)
; Mission Usage: Rendezvous navigation with VHF range, timing, and IMU monitoring
; Apollo 11 Context: Used during LM-CSM rendezvous after lunar ascent July 21
; VHF ranging provided relative distance between Eagle and Columbia
; Format: 2INT, MIN/SEC, DPDEG(360) with 2INT, M/S, DP4 routines
		ECADR	VHFCNT			# 45	2INT			2INT
# Page 280
		ECADR	TTOGO			# 45	MIN/SEC			M/S
		ECADR	+MGA			# 45	DPDEG(360)		DP4

; Noun 46: DAP DATA REGISTER 1 / DAP DATA REGISTER 2 / (SPARE)
; Component 1: DAPDATR1 - Digital Autopilot configuration register 1 (octal)
; Component 2: DAPDATR2 - Digital Autopilot configuration register 2 (octal)
; Component 3: (Spare component, not used)
; Mission Usage: Displays DAP control parameters for attitude control system
; Shows thruster configuration, control gains, deadbands in raw octal format
; Used by ground controllers and crew for DAP status monitoring and troubleshooting
; Format: OCTAL ONLY, OCTAL ONLY, (spare) with OCT, OCT, (none) routines
		ECADR	DAPDATR1		# 46	OCTAL ONLY		OCT
		ECADR	DAPDATR2		# 46	OCATAL ONLY		OCT
		OCT	0			# 46	SPARE COMPONENT

; Noun 47: CSM MASS / LM MASS / (SPARE)
; Component 1: CSMMASS - Command/Service Module mass (pounds, XXXXX. lbs)
; Component 2: LEMMASS - Lunar Module mass (pounds, XXXXX. lbs)
; Component 3: (Spare component, not used)
; Mission Usage: Displays spacecraft masses for guidance and control calculations
; Mass values updated as propellant consumed during burns
; Critical for accurate trajectory calculations and engine performance
; Apollo 11 Context: CSM Columbia ~63,500 lbs, LM Eagle ~33,500 lbs at separation
; Format: WEIGHT2, WEIGHT2, (spare) with ARTH1, ARTH1, (none) routines
		ECADR	CSMMASS			# 47	WEIGHT2			ARTH1
		ECADR	LEMMASS			# 47	WEIGHT2			ARTH1
		OCT	00000			# 47	SPARE COMPONENT

; Noun 48: PITCH ACTUATOR TRIM / YAW ACTUATOR TRIM / (SPARE)
; Component 1: PACTOFF - Pitch actuator trim offset (degrees, XXX.XX)
; Component 2: YACTOFF - Yaw actuator trim offset (degrees, XXX.XX)
; Component 3: (Spare component, not used)
; Mission Usage: Displays gimbal trim settings for engine thrust vector control
; Trim offsets compensate for center-of-gravity shifts as propellant consumed
; Ensures thrust vector properly aligned through spacecraft mass center
; Used during SPS and DPS burns for optimal control
; Format: TRIM DEG, TRIM DEG, (spare) with ARTH, ARTH, (none) routines
		ECADR	PACTOFF			# 48	TRIM DEG		ARTH
		ECADR	YACTOFF			# 48	TRIM DEG		ARTH
		OCT	00000			# 48	SPARE COMPONENT

; Noun 49: DELTA-R / DELTA-V / BURN-TIME
; Component 1: N49DISP - Position change magnitude (nautical miles, XXXX.X)
; Component 2: N49DISP+2 - Velocity change magnitude (ft/sec, XXXX.X)
; Component 3: N49DISP+4 - Required burn duration (seconds, whole number)
; Mission Usage: Displays computed maneuver parameters for upcoming engine burn
; Shows how much position change, velocity change, and burn time required
; Critical display during burn planning and execution monitoring
; Apollo 11 Context: Used for TLI, LOI, TEI burn parameter displays
; Format: POS4, VEL3, WHOLE with DP3, DP3, ARTH routines
		ECADR	N49DISP			# 49	POS4			DP3
		ECADR	N49DISP +2		# 49	VEL3			DP3
		ECADR	N49DISP +4		# 49	WHOLE			ARTH

; Noun 50: RANGE-TO-PERIGEE / PERIAPSIS ALTITUDE / TIME-FROM-PERIAPSIS
; Component 1: RSP-RREC - Range distance to perigee point (nautical miles, XXXX.X)
; Component 2: HPERX - Periapsis altitude above reference (nautical miles, XXXX.X)
; Component 3: TFF - Time from periapsis passage (minutes:seconds display)
; Mission Usage: Combined orbit display showing perigee parameters with timing
; Shows distance to lowest point, altitude at that point, and time since passing it
; Used for precise orbital monitoring and maneuver timing
; Format: POS6, POS4, MIN/SEC with DP3, DP3, M/S routines
		ECADR	RSP-RREC		# 50	POS6			DP3
		ECADR	HPERX			# 50 	POS4			DP3
		ECADR	TFF			# 50	MIN/SEC			M/S

; Noun 51: RHO ANGLE / GAMMA ANGLE / (SPARE)
; Component 1: RHOSB - Rho angle for S-band antenna pointing (degrees, 0-360°)
; Component 2: GAMMASB - Gamma angle for S-band antenna pointing (degrees, 0-360°)
; Component 3: (Spare component, not used)
; Mission Usage: Displays computed S-band high-gain antenna pointing angles
; Rho and gamma define antenna orientation to maintain Earth communications
; Critical for maintaining downlink telemetry and uplink command capability
; Apollo 11 Context: Antenna automatically tracked Earth during translunar/transearth coast
; Format: DPDEG(360), DPDEG(360), (spare) with (none), DP4, (none) routines
		ECADR	RHOSB			# 51	DPDEG(360)
		ECADR	GAMMASB			# 51	DPDEG(360)		DP4
		OCT	0			# 51	SPARE COMPONENT

; Noun 52: ACTUAL CENTROID ANGLE / (SPARE) / (SPARE)
; Component 1: ACTCENT - Actual centroid angle from optical tracker (degrees, 0-360°)
; Component 2: (Spare component, not used)
; Component 3: (Spare component, not used)
; Mission Usage: Displays computed centroid angle from star tracker or landmark tracker
; Used for optical navigation mark processing and alignment verification
; Centroid represents center of tracked target in optical field of view
; Format: DPDEG(360), (spare), (spare) with DP4, (none), (none) routines
		ECADR	ACTCENT			# 52	DPDEG(360)		DP4
		OCT	00000			# 52	SPARE COMPONENT
		OCT	00000			# 52 	SPARE COMPONENT

; Noun 53: RANGE / RANGE RATE / RANGE ANGLE (THETA)
; Component 1: RANGE - Distance to target (nautical miles, XXX.XX)
; Component 2: RRATE - Range rate (closing velocity, feet/second, XXXX.X)
; Component 3: RTHETA - Range angle theta in target coordinate system (degrees, 0-360°)
; Mission Usage: Complete rendezvous navigation state display
; Shows how far, how fast closing, and angular position relative to target
; Apollo 11 Context: Critical during Eagle's rendezvous with Columbia after lunar ascent
; Used by crew and ground controllers to monitor approach trajectory
; Format: POS5, VEL3, DPDEG(360) with DP1, DP3, DP4 routines
		ECADR	RANGE			# 53	POS5			DP1
		ECADR	RRATE			# 53 	VEL3			DP3
		ECADR	RTHETA			# 53 	DPDEG(360)		DP4

; Noun 54: RANGE / RANGE RATE / RANGE ANGLE (THETA) - ALTERNATE
; Component 1: RANGE - Distance to target (nautical miles, XXX.XX)
; Component 2: RRATE - Range rate (closing velocity, feet/second, XXXX.X)
; Component 3: RTHETA - Range angle theta in target coordinate system (degrees, 0-360°)
; Mission Usage: Duplicate of Noun 53 for alternate display or program usage
; Provides identical rendezvous navigation state information
; May be used by different mission programs accessing same underlying data
; Format: POS5, VEL3, DPDEG(360) with DP1, DP3, DP4 routines
		ECADR	RANGE			# 54	POS5			DP1
		ECADR	RRATE			# 54	VEL3			DP3
		ECADR	RTHETA			# 54	DPDEG(360)		DP4

; Noun 55: NN1 / ELEVATION ANGLE / CENTROID ANGLE
; Component 1: NN1 - Integer counter or identifier (whole number)
; Component 2: ELEV - Elevation angle (degrees, 0-360°, XXX.XX)
; Component 3: CENTANG - Centroid angle from optical tracker (degrees, 0-360°, XXX.XX)
; Mission Usage: Displays optical tracking data with counter and angular measurements
; Used during star sighting or landmark tracking for navigation fixes
; Elevation and centroid angles define target position in optical field
; Format: WHOLE, DPDEG(360), DPDEG(360) with ARTH, DP4, DP4 routines
		ECADR	NN1			# 55	WHOLE			ARTH
		ECADR	ELEV			# 55	DPDEG(360)		DP4
		ECADR	CENTANG			# 55	DPDEG(360)		DP4

; Noun 56: RTE GAMMA 2D / RTE DELTA-V / (SPARE)
; Component 1: RTEGAM2D - Return-to-Earth flight path angle gamma (degrees, 0-360°, XXX.XX)
; Component 2: RTEDVD - Return-to-Earth required delta-velocity (feet/second, XXXXX.)
; Component 3: (Spare component, not used)
; Mission Usage: Displays transearth injection (TEI) targeting parameters
; Flight path angle gamma defines re-entry corridor trajectory angle
; Delta-V shows required velocity change for TEI burn targeting
; Apollo 11 Context: Used during P37 return-to-Earth targeting calculations
; Format: DPDEG(360), VEL2, (spare) with DP4, DP4, (none) routines
		ECADR	RTEGAM2D		# 56	DPDEG(360)		DP4
		ECADR	RTEDVD			# 56	VEL2			DP4
		OCT	0			# 56	SPARE COMPONENT

; Noun 57: DELTA RANGE / (SPARE) / (SPARE)
; Component 1: DELTAR - Delta range change or range correction (nautical miles, XXXX.X)
; Component 2: (Spare component, not used)
; Component 3: (Spare component, not used)
; Mission Usage: Displays range change or correction value
; Used for trajectory targeting or navigation state updates
; Delta range represents change in distance between current and target position
; Format: POS4, (spare), (spare) with DP3, (none), (none) routines
		ECADR	DELTAR			# 57	POS4			DP3
		OCT	0			# 57	SPARE COMPONENT
		OCT	0			# 57	SPARE COMPONENT

; Noun 58: POSITION AT TPI / DELTA-V TPI / DELTA-V TPF
; Component 1: POSTTPI - Position at Terminal Phase Initiation (nautical miles, XXXX.X)
; Component 2: DELVTPI - Delta-velocity required for TPI burn (feet/second, XXXX.X)
; Component 3: DELVTPF - Delta-velocity required for TPF (Terminal Phase Final) (ft/sec, XXXX.X)
; Mission Usage: Displays complete Terminal Phase Initiation rendezvous planning data
; TPI is critical burn initiating final approach to target vehicle
; Shows where TPI occurs and required velocity changes for TPI and TPF burns
; Apollo 11 Context: Used during rendezvous sequence planning (Eagle to Columbia)
; Format: POS4, VEL3, VEL3 with DP3, DP3, DP3 routines
		ECADR	POSTTPI			# 58	POS4			DP3
		ECADR	DELVTPI			# 58	VEL3			DP3
		ECADR	DELVTPF			# 58	VEL3			DP3

; Noun 59: DELTA-V LOS (LINE OF SIGHT) - X, Y, Z COMPONENTS
; Component 1: DVLOS - Delta-velocity line-of-sight X component (feet/second, XXXX.X)
; Component 2: DVLOS+2 - Delta-velocity line-of-sight Y component (feet/second, XXXX.X)
; Component 3: DVLOS+4 - Delta-velocity line-of-sight Z component (feet/second, XXXX.X)
; Mission Usage: Displays required delta-V vector in line-of-sight coordinate frame
; LOS coordinates align with direction to target vehicle during rendezvous
; Three components define complete velocity change vector for intercept
; Used for manual targeting or verification of computed rendezvous solutions
; Format: VEL3, VEL3, VEL3 with DP3, DP3, DP3 routines
		ECADR	DVLOS			# 59	VEL3			DP3
		ECADR	DVLOS +2		# 59	VEL3			DP3
		ECADR	DVLOS +4		# 59	VEL3			DP3

; Noun 60: GMAX / PREDICTED VELOCITY / GAMMA (ENTRY INTERFACE)
; Component 1: GMAX - Maximum sensed acceleration during entry (whole G units)
; Component 2: VPRED - Predicted inertial velocity (feet/second, XXXXX.)
; Component 3: GAMMAEI - Flight path angle gamma at entry interface (degrees, 0-360°, XXX.XX)
; Mission Usage: Displays critical entry parameters during atmospheric re-entry
; GMAX shows peak deceleration forces experienced (Apollo 11: ~6.5 G on re-entry)
; VPRED shows expected velocity magnitude at current point in trajectory
; GAMMAEI defines entry corridor angle (shallow = skip out, steep = overheat)
; Apollo 11 Context: Critical monitoring during July 24, 1969 Pacific splashdown approach
; Format: WHOLE, VEL2, DPDEG(360) with ARTH, DP4, DP4 routines
		ECADR	GMAX			# 60	WHOLE			ARTH
		ECADR	VPRED			# 60	VEL2			DP4
		ECADR	GAMMAEI			# 60	DPDEG(360)		DP4

; Noun 61: SPLASH LATITUDE / SPLASH LONGITUDE / HEADS-UP/DOWN FLAG
; Component 1: LAT(SPL) - Predicted splashdown latitude (degrees, 0-360°, XXX.XX)
; Component 2: LNG(SPL) - Predicted splashdown longitude (degrees, 0-360°, XXX.XX)
; Component 3: HEADSUP - Crew orientation flag (whole number: 1=heads-up, 0=heads-down)
; Mission Usage: Displays predicted landing coordinates and entry attitude mode
; Latitude/longitude show where spacecraft will land based on current trajectory
; Heads-up/heads-down determines crew orientation during entry for lift control
; Apollo 11 Context: Splashdown coordinates monitored during P67 final entry phase
; Actual Apollo 11 splashdown: 13°19'N 169°9'W in Pacific Ocean, July 24, 1969
; Format: DPDEG(360), DPDEG(360), WHOLE with DP4, DP4, ARTH routines
		ECADR	LAT(SPL)		# 61	DPDEG(360)		DP4
		ECADR	LNG(SPL)		# 61	DPDEG(360)		DP4
		ECADR	HEADSUP			# 61	WHOLE			ARTH

; Noun 62: INERTIAL VELOCITY MAGNITUDE / ALTITUDE RATE / ALTITUDE
; Component 1: VMAGI - Inertial velocity magnitude (feet/second, XXXXX.)
; Component 2: HDOT - Altitude rate (vertical velocity) (feet/second, XXXXX.)
; Component 3: ALTI - Current altitude above reference (nautical miles, XXXX.X)
; Mission Usage: Displays complete trajectory state during powered flight or entry
; VMAGI shows total velocity magnitude in inertial reference frame
; HDOT indicates rate of altitude change (positive=climbing, negative=descending)
; ALTI shows height above Earth or lunar surface depending on mission phase
; Used during ascent, orbit maneuvers, descent, and entry for trajectory monitoring
; Format: VEL2, VEL2, POS4 with DP4, DP4, DP3 routines
# Page 281
		ECADR	VMAGI			# 62	VEL2			DP4
		ECADR	HDOT			# 62	VEL2			DP4
		ECADR	ALTI			# 62	POS4			DP3

; Noun 63: RANGE-TO-GO / INERTIAL VELOCITY / TIME-TO-END
; Component 1: RTGO - Range-to-go to target (nautical miles, XXXX.X)
; Component 2: VIO - Inertial velocity magnitude (feet/second, XXXXX.)
; Component 3: TTE - Time-to-end or time remaining (minutes:seconds, MM:SS)
; Mission Usage: Displays complete trajectory status during entry phase
; RTGO shows remaining distance to splashdown point (decreases during descent)
; VIO shows current velocity magnitude for energy management monitoring
; TTE indicates time remaining until splashdown (critical for crew preparation)
; Apollo 11 Context: Monitored during P67 final entry phase before Pacific splashdown
; Format: POS6, VEL2, MIN/SEC with DP3, DP4, M/S routines
		ECADR	RTGO			# 63	POS6			DP3
		ECADR	VIO			# 63	VEL2			DP4
		ECADR	TTE			# 63	MIN/SEC			M/S

; Noun 64: DRAG ACCELERATION / INERTIAL VELOCITY / RANGE-TO-GO
; Component 1: D - Drag acceleration (G units, XXX.XX)
; Component 2: VMAGI - Inertial velocity magnitude (feet/second, XXXXX.)
; Component 3: RTGON64 - Range-to-go to target (nautical miles, XXXX.X)
; Mission Usage: Comprehensive entry monitoring combining deceleration, velocity, and range
; Drag acceleration shows atmospheric braking forces (increases as atmosphere thickens)
; Inertial velocity decreases as drag slows spacecraft during entry corridor
; Range-to-go tracks remaining distance to splashdown point
; Apollo 11 Context: Critical parameters during July 24, 1969 Pacific Ocean entry
; Format: DRAG ACCEL, VEL2, POS6 with DP2, DP4, DP3 routines
		ECADR	D			# 64 	DRAG ACCEL		DP2
		ECADR	VMAGI			# 64	VEL2			DP4
		ECADR	RTGON64			# 64	POS6			DP3

; Noun 65: SAMPLE TIME (HOURS:MINUTES:SECONDS - ALL THREE COMPONENTS IDENTICAL)
; Component 1: SAMPTIME - Sample time (hours:minutes:seconds format, HH:MM:SS)
; Component 2: SAMPTIME - Sample time (hours:minutes:seconds format, HH:MM:SS)
; Component 3: SAMPTIME - Sample time (hours:minutes:seconds format, HH:MM:SS)
; Mission Usage: Displays sample acquisition timestamp (used for navigation mark logging)
; All three components point to same SAMPTIME variable (display format requirement)
; Used when crew takes optical sightings (star or landmark marks) for navigation updates
; Timestamp allows ground controllers to correlate marks with trajectory solution
; Mixed format code 65 keeps hours/minutes/seconds display (HMS format)
; Format: HMS, HMS, HMS (all three components display same time value)
		ECADR	SAMPTIME		# 65	HMS (MIXED ONLY TO KEEP CODE 65) HMS
		ECADR	SAMPTIME		# 65	HMS			HMS
		ECADR	SAMPTIME		# 65	HMS			HMS

; Noun 66: ROLL COMMAND / CROSSRANGE ERROR / DOWNRANGE ERROR
; Component 1: ROLLC - Roll command angle (degrees, 0-360°, XXX.XX)
; Component 2: XRNGERR - Crossrange error from target (nautical miles, XXXX.X)
; Component 3: DNRNGERR - Downrange error from target (nautical miles, XXXX.X)
; Mission Usage: Entry guidance error monitoring - tracks trajectory deviations from target
; Roll command controls lift vector direction for lateral maneuvering during entry
; Crossrange error shows lateral (left/right) deviation from planned entry corridor
; Downrange error shows along-track (early/late) deviation from splashdown point
; CM modulates roll angle to correct both crossrange and downrange errors
; Apollo 11 Context: Used during P67 entry phase to guide toward Pacific splashdown point
; Format: DPDEG(360), POS8, POS6 with DP4, DP3, DP3 routines
		ECADR	ROLLC			# 66	DPDEG(360)		DP4
		ECADR	XRNGERR			# 66	POS8			DP3
		ECADR	DNRNGERR		# 66	POS6			DP3

; Noun 67: RANGE-TO-GO / LATITUDE / LONGITUDE
; Component 1: RTGON67 - Range-to-go to target (nautical miles, XXXX.X)
; Component 2: LAT - Latitude of current or target position (degrees, 0-360°, XXX.XX)
; Component 3: LONG - Longitude of current or target position (degrees, 0-360°, XXX.XX)
; Mission Usage: Displays geographic position and range during entry or orbital operations
; Range-to-go tracks distance remaining to splashdown or landing point
; Latitude and longitude specify Earth surface position in degrees
; Used during entry to verify trajectory toward Pacific recovery zone
; Apollo 11 Context: Monitored during transearth coast and entry phase
; Final splashdown at 13°19'N, 169°9'W in Pacific Ocean, July 24, 1969
; Format: POS6, DPDEG(360), DPDEG(360) with DP3, DP4, DP4 routines
		ECADR	RTGON67			# 67	POS6			DP3
		ECADR	LAT			# 67	DPDEG(360)		DP4
		ECADR	LONG			# 67	DPDEG(360)		DP4

; Noun 68: ROLL COMMAND / INERTIAL VELOCITY / RADIAL VELOCITY
; Component 1: ROLLC - Roll command angle (degrees, 0-360°, XXX.XX)
; Component 2: VMAGI - Inertial velocity magnitude (feet/second, XXXXX.)
; Component 3: RDOT - Radial velocity component (feet/second, XXXXX.)
; Mission Usage: Entry flight dynamics display combining roll control and velocity vectors
; Roll command shows commanded bank angle for lift vector modulation
; Inertial velocity shows total spacecraft velocity magnitude
; Radial velocity (RDOT) indicates rate of altitude change (positive = ascending)
; Critical for entry monitoring: RDOT negative during descent, approaches zero at splashdown
; Apollo 11 Context: Monitored during entry corridor to verify descent rate
; Format: DPDEG(360), VEL2, VEL/2VS with DP4, DP4, DP4 routines
		ECADR	ROLLC			# 68	DPDEG(360)		DP4
		ECADR	VMAGI			# 68	VEL2			DP4
		ECADR	RDOT			# 68	VEL/2VS			DP4

; Noun 69: ROLL COMMAND / DRAG ACCELERATION / LATERAL VELOCITY
; Component 1: ROLLC - Roll command angle (degrees, 0-360°, XXX.XX)
; Component 2: Q7 - Drag acceleration (G units, XXX.XX)
; Component 3: VL - Lateral velocity component (feet/second, XXXXX.)
; Mission Usage: Entry dynamics monitoring combining control, deceleration, and lateral motion
; Roll command shows commanded bank angle for trajectory corrections
; Q7 shows atmospheric drag forces acting on spacecraft during entry
; Lateral velocity (VL) indicates crossrange velocity component
; High drag acceleration indicates deep atmospheric penetration (peak ~6 G for Apollo)
; Apollo 11 Context: Monitored during entry to verify deceleration profile
; Format: DPDEG(360), DRAG ACCEL, VEL/2VS with DP4, DP2, DP4 routines
		ECADR	ROLLC			# 69	DPDEG(360)		DP4
		ECADR	Q7			# 69	DRAG ACCEL		DP2
		ECADR	VL			# 69	VEL/2VS			DP4

; Noun 70: STAR CODE / LANDMARK CODE / HORIZON CODE (OPTICAL NAVIGATION)
; Component 1: STARCODE - Star catalog identification number (octal display only)
; Component 2: LANDMARK - Lunar or Earth landmark identification code (octal display only)
; Component 3: HORIZON - Horizon identification or selection code (octal display only)
; Mission Usage: Optical navigation target identification for IMU alignment or state vector updates
; STARCODE identifies which star from catalog being sighted (e.g., Antares, Sirius, Vega)
; LANDMARK identifies lunar or terrestrial surface features for navigation marks
; HORIZON specifies horizon for elevation angle measurements (Earth or lunar limb)
; Used with sextant (CM) or Alignment Optical Telescope (LM) for navigation
; Apollo 11 Context: Used during IMU alignments and landmark tracking passes
; Format: OCTAL ONLY, OCTAL ONLY, OCTAL ONLY (all integer identification codes)
		ECADR	STARCODE		# 70 	OCTAL ONLY		OCT
		ECADR	LANDMARK		# 70	OCTAL ONLY		OCT
		ECADR	HORIZON			# 70	OCTAL ONLY		OCT

; Noun 71: STAR CODE / LANDMARK CODE / HORIZON CODE (ALTERNATE DISPLAY)
; Component 1: STARCODE - Star catalog identification number (octal display only)
; Component 2: LANDMARK - Lunar or Earth landmark identification code (octal display only)
; Component 3: HORIZON - Horizon identification or selection code (octal display only)
; Mission Usage: Identical to Noun 70 - alternate noun for same optical navigation codes
; Provides duplicate noun for different program contexts or display requirements
; Allows multiple programs to display optical target codes without conflict
; Useful when different mission phases need simultaneous optical navigation displays
; Apollo 11 Context: Used for P51/P52/P53 IMU alignment program displays
; Format: OCTAL ONLY, OCTAL ONLY, OCTAL ONLY (all integer identification codes)
		ECADR	STARCODE		# 71	OCTAL ONLY		OCT
		ECADR	LANDMARK		# 71	OCTAL ONLY		OCT
		ECADR	HORIZON			# 71 	OCTAL ONLY		OCT

; Noun 72: INITIAL AZIMUTH / DELTA HEIGHT / OPTION CODE
; Component 1: THETZERO - Initial azimuth angle (degrees, 0-360°, XXX.XX)
; Component 2: DELHITE - Delta height parameter (nautical miles, XXXX.X)
; Component 3: OPTION2 - Option code selection (whole number, integer)
; Mission Usage: Entry or rendezvous targeting parameters
; THETZERO specifies initial azimuth angle for trajectory or alignment
; DELHITE represents height differential between current and target altitude
; OPTION2 selects program mode or computational option variant
; Used in targeting calculations where azimuth and height are critical parameters
; Apollo 11 Context: May be used in rendezvous or entry targeting computations
; Format: DPDEG(360), POS4, WHOLE with DP4, DP3, ARTH routines
		ECADR	THETZERO		# 72 	DPDEG(360)		DP4
		ECADR	DELHITE			# 72	POS4			DP3
		ECADR	OPTION2			# 72	WHOLE			ARTH

; Noun 73: P21 ALTITUDE / VELOCITY / FLIGHT PATH ANGLE
; Component 1: P21ALT - Program 21 altitude (nautical miles, XXXX.X)
; Component 2: P21VEL - Program 21 velocity magnitude (feet/second, XXXXX.)
; Component 3: P21GAM - Program 21 flight path angle (degrees, 0-360°, XXX.XX)
; Mission Usage: Ground tracking determination program (P21) state vector display
; P21 processes MSFN (Manned Space Flight Network) ground tracking data
; Displays altitude, velocity, and flight path angle from ground tracking solution
; Flight path angle (gamma) shows trajectory angle relative to local horizontal
; Positive gamma = ascending trajectory, negative gamma = descending trajectory
; Apollo 11 Context: Used during translunar and transearth coast for navigation updates
; Format: POS4, VEL2, DPDEG(360) with DP3, DP4, DP4 routines
		ECADR	P21ALT			# 73	POS4			DP3
		ECADR	P21VEL			# 73	VEL2			DP4
		ECADR	P21GAM			# 73	DPDEG(360)		DP4

; Noun 74: ROLL COMMAND / INERTIAL VELOCITY / DRAG ACCELERATION
; Component 1: ROLLC - Roll command angle (degrees, 0-360°, XXX.XX)
; Component 2: VMAGI - Inertial velocity magnitude (feet/second, XXXXX.)
; Component 3: D - Drag acceleration (G units, XXX.XX)
; Mission Usage: Entry guidance display combining control, velocity, and deceleration
; Roll command shows commanded bank angle for lift vector control
; Inertial velocity magnitude tracks total spacecraft speed (decreases during entry)
; Drag acceleration (D) shows atmospheric deceleration forces on spacecraft
; Peak drag occurs at maximum dynamic pressure (max-q) during atmospheric entry
; Apollo 11 Context: Critical entry monitoring noun during P67 entry guidance
; Format: DPDEG(360), VEL2, DRAG ACCEL with DP4, DP4, DP2 routines
		ECADR	ROLLC			# 74	DPDEG(360)		DP4
		ECADR	VMAGI			# 74	VEL2			DP4
		ECADR	D			# 74	DRAG ACCEL		DP2

; Noun 75: DIFFERENTIAL ALTITUDE / TIME INTERVAL 1 / TIME INTERVAL 2
; Component 1: DIFFALT - Differential altitude (nautical miles, XXXX.X)
; Component 2: T1TOT2 - Time interval from T1 to T2 (minutes:seconds, MM:SS)
; Component 3: T2TOT3 - Time interval from T2 to T3 (minutes:seconds, MM:SS)
; Mission Usage: Timeline and altitude difference display for mission phase sequencing
; DIFFALT shows altitude difference between two orbital conditions or targets
; T1TOT2 displays elapsed time between first and second timeline events
; T2TOT3 displays elapsed time between second and third timeline events
; Used for maneuver planning where altitude changes and timing are critical
; Apollo 11 Context: Rendezvous or orbital maneuver timeline displays
; Format: POS4, MIN/SEC, MIN/SEC with DP3, M/S, M/S routines
		ECADR	DIFFALT			# 75	POS4			DP3
		ECADR	T1TOT2			# 75	MIN/SEC			M/S
		ECADR	T2TOT3			# 75	MIN/SEC			M/S

; Nouns 76, 77, 78, 79: SPARE (RESERVED FOR FUTURE USE)
; These noun numbers are reserved but not assigned to any variables
; Spare nouns provide expansion capacity for additional display/input requirements
; Can be allocated for mission-specific needs or software updates
; Apollo 11 Context: Unused noun slots available for future mission enhancements
		OCT	0			# 76	SPARE
		OCT	0			# 76	SPARE
		OCT	0			# 76	SPARE
		OCT	0			# 77	SPARE
		OCT	0			# 77	SPARE
		OCT	0			# 77	SPARE
		OCT	0			# 78	SPARE
		OCT	0			# 78	SPARE
# Page 282
		OCT	0			# 78	SPARE
		OCT	0			# 79	SPARE
		OCT	0			# 79	SPARE
		OCT	0			# 79	SPARE

; Noun 80: TIME-TO-GO / INERTIAL VELOCITY / TOTAL DELTA-V
; Component 1: TTOGO - Time remaining until event (minutes:seconds, MM:SS)
; Component 2: VGDISP - Inertial velocity for display (feet/second, XXXXX.)
; Component 3: DVTOTAL - Total delta-V accumulated (feet/second, XXXXX.)
; Mission Usage: Burn monitoring display for powered flight phases
; TTOGO counts down remaining burn time (critical for precise cutoff timing)
; VGDISP shows current inertial velocity magnitude during powered flight
; DVTOTAL accumulates total velocity change delivered by engine firing
; Used during translunar injection, lunar orbit insertion, and all major burns
; Apollo 11 Context: Primary burn monitoring noun for TLI, LOI, and TEI maneuvers
; Crew monitored TTOGO for abort decision timing and burn completion verification
; Format: MIN/SEC, VEL2, VEL2 with M/S, DP4, DP4 routines
		ECADR	TTOGO			# 80	MIN/SEC			M/S
		ECADR	VGDISP			# 80	VEL2			DP4
		ECADR	DVTOTAL			# 80	VEL2			DP4

; Noun 81: DELTA-V (LOCAL VERTICAL COORDINATES)
; Component 1: DELVLVC (X) - Delta-V X component in local vertical frame (ft/sec, XXXX.X)
; Component 2: DELVLVC+2 (Y) - Delta-V Y component in local vertical frame (ft/sec, XXXX.X)
; Component 3: DELVLVC+4 (Z) - Delta-V Z component in local vertical frame (ft/sec, XXXX.X)
; Mission Usage: Velocity change display in local vertical coordinate system
; Local vertical frame: X=downrange, Y=crossrange, Z=altitude (radial)
; Used for targeting displays showing required velocity change components
; Critical for rendezvous maneuvers where relative position to target matters
; Apollo 11 Context: CSI (Coelliptic Sequence Initiation) and CDH targeting displays
; Format: VEL3, VEL3, VEL3 with DP3, DP3, DP3 routines
		ECADR	DELVLVC			# 81	VEL3			DP3
		ECADR	DELVLVC +2		# 81	VEL3			DP3
		ECADR	DELVLVC +4		# 81	VEL3			DP3

; Noun 82: DELTA-V (LOCAL VERTICAL COORDINATES) - ALTERNATE DISPLAY
; Component 1: DELVLVC (X) - Delta-V X component in local vertical frame (ft/sec, XXXX.X)
; Component 2: DELVLVC+2 (Y) - Delta-V Y component in local vertical frame (ft/sec, XXXX.X)
; Component 3: DELVLVC+4 (Z) - Delta-V Z component in local vertical frame (ft/sec, XXXX.X)
; Mission Usage: Duplicate noun number accessing same data as N81
; Provides alternate noun number for same local vertical delta-V display
; May be used by different programs or verb sequences requiring same data
; Apollo 11 Context: Alternate access to CSI/CDH targeting information
; Format: VEL3, VEL3, VEL3 with DP3, DP3, DP3 routines
		ECADR	DELVLVC			# 82	VEL3			DP3
		ECADR	DELVLVC +2		# 82	VEL3			DP3
		ECADR	DELVLVC +4		# 82 	VEL3			DP3

; Noun 83: DELTA-V (IMU COORDINATES)
; Component 1: DELVIMU (X) - Delta-V X component in IMU stable member frame (ft/sec, XXXX.X)
; Component 2: DELVIMU+2 (Y) - Delta-V Y component in IMU stable member frame (ft/sec, XXXX.X)
; Component 3: DELVIMU+4 (Z) - Delta-V Z component in IMU stable member frame (ft/sec, XXXX.X)
; Mission Usage: Velocity change display in IMU stable member coordinate system
; IMU frame is inertial reference maintained by gyroscopes (not rotating with spacecraft)
; Used for displays requiring inertial delta-V components independent of spacecraft attitude
; Critical for trajectory planning where inertial frame alignment is maintained
; Apollo 11 Context: Midcourse corrections and navigation state updates
; Format: VEL3, VEL3, VEL3 with DP3, DP3, DP3 routines
		ECADR	DELVIMU			# 83	VEL3			DP3
		ECADR	DELVIMU +2		# 83	VEL3			DP3
		ECADR	DELVIMU +4		# 83	VEL3			DP3

; Noun 84: DELTA-V (OTHER VEHICLE)
; Component 1: DELVOV (X) - Delta-V X component for other vehicle (ft/sec, XXXX.X)
; Component 2: DELVOV+2 (Y) - Delta-V Y component for other vehicle (ft/sec, XXXX.X)
; Component 3: DELVOV+4 (Z) - Delta-V Z component for other vehicle (ft/sec, XXXX.X)
; Mission Usage: Velocity change display for target/other vehicle (LM or CSM)
; Used during rendezvous operations to display relative delta-V of target spacecraft
; Critical for CSM-LM rendezvous planning and maneuver coordination
; Apollo 11 Context: Columbia (CSM) tracking Eagle (LM) during rendezvous (July 21)
; Format: VEL3, VEL3, VEL3 with DP3, DP3, DP3 routines
		ECADR	DELVOV			# 84	VEL3			DP3
		ECADR	DELVOV +2		# 84	VEL3			DP3
		ECADR	DELVOV +4		# 84	VEL3			DP3

; Noun 85: VELOCITY (BODY COORDINATES)
; Component 1: VGBODY (X) - Velocity X component in spacecraft body frame (ft/sec, XXXX.X)
; Component 2: VGBODY+2 (Y) - Velocity Y component in spacecraft body frame (ft/sec, XXXX.X)
; Component 3: VGBODY+4 (Z) - Velocity Z component in spacecraft body frame (ft/sec, XXXX.X)
; Mission Usage: Inertial velocity resolved into spacecraft body coordinate system
; Body frame rotates with spacecraft attitude (X=forward, Y=right, Z=down typically)
; Used during powered flight to show velocity components along spacecraft axes
; Apollo 11 Context: SPS burns for trajectory corrections, TLI, LOI, TEI maneuvers
; Format: VEL3, VEL3, VEL3 with DP3, DP3, DP3 routines
		ECADR	VGBODY			# 85	VEL3			DP3
		ECADR	VGBODY +2		# 85	VEL3			DP3
		ECADR	VGBODY +4		# 85	VEL3			DP3

; Noun 86: DELTA-V (LOCAL VERTICAL) - HIGH PRECISION
; Component 1: DELVLVC (X) - Delta-V X component in local vertical frame (ft/sec, XXXXX.)
; Component 2: DELVLVC+2 (Y) - Delta-V Y component in local vertical frame (ft/sec, XXXXX.)
; Component 3: DELVLVC+4 (Z) - Delta-V Z component in local vertical frame (ft/sec, XXXXX.)
; Mission Usage: High-precision local vertical delta-V display (VEL2 format vs VEL3)
; Same data as N81/N82 but displayed with 5-digit whole number precision
; Used when higher velocity magnitudes require less fractional precision
; Apollo 11 Context: Large maneuvers where velocities exceed 9999.9 ft/sec capability
; Format: VEL2, VEL2, VEL2 with DP4, DP4, DP4 routines (whole number display)
		ECADR	DELVLVC			# 86	VEL2			DP4
		ECADR	DELVLVC +2		# 86	VEL2			DP4
		ECADR	DELVLVC +4		# 86	VEL2			DP4

; Noun 87: SEXTANT MARK DATA (BUFFER 1)
; Component 1: MRKBUF1+3 (Shaft) - Sextant shaft angle in CDU degrees (XXX.XX deg)
; Component 2: MRKBUF1+5 (Trunnion) - Sextant trunnion angle in Y optics (XX.XXX deg, max 89.999)
; Component 3: Spare (no third component for this noun)
; Mission Usage: Displays sextant optical angles from manual star/landmark sightings
; Shaft angle rotates sextant around spacecraft Z-axis (0-360 degrees)
; Trunnion angle elevates sextant line-of-sight (limited to ±90 degrees)
; Used with P51/P52/P53 alignment programs and landmark tracking
; Apollo 11 Context: IMU alignment star sightings, transearth navigation marks
; Format: CDU DEG, Y OPTICS DEG, SPARE with CDU, YOPT, (none) routines
		ECADR	MRKBUF1 +3		# 87	CDU DEG			CDU
		ECADR	MRKBUF1 +5		# 87	Y OPTICS DEG		YOPT
		OCT	0			# 87	SPARE COMPONENT

; Noun 88: STAR CATALOG DATA (SAVED STAR 3)
; Component 1: STARSAV3 - Star catalog unit vector X component (double-precision fractional)
; Component 2: STARSAV3+2 - Star catalog unit vector Y component (double-precision fractional)
; Component 3: STARSAV3+4 - Star catalog unit vector Z component (double-precision fractional)
; Mission Usage: Displays saved star catalog position vector (unit vector in IMU frame)
; Star catalog contains precise inertial positions of 37 navigation stars
; Used during IMU alignment programs to compare sighted star against catalog position
; Apollo 11 Context: P51/P52 alignment verification with third star sighting
; Format: DPFRAC, DPFRAC, DPFRAC with DPFRAC, DPFRAC, DPFRAC routines (straight fractional)
		ECADR	STARSAV3		# 88	DPFRAC		      DPFRAC
		ECADR	STARSAV3 +2		# 88	DPFRAC		      DPFRAC
		ECADR	STARSAV3 +4		# 88	DPFRAC		      DPFRAC

; Noun 89: LANDMARK DATA (LATITUDE, LONGITUDE, ALTITUDE)
; Component 1: LANDLAT - Landmark latitude in degrees (XX.XXX deg, range ±90 degrees)
; Component 2: LANDLONG - Landmark longitude in degrees (XX.XXX deg, range ±90 degrees)
; Component 3: LANDALT - Landmark altitude (XXX.XX nautical miles above reference)
; Mission Usage: Displays tracked lunar or Earth surface landmark coordinates
; Used in optical navigation for position determination from landmark sightings
; Crew marks landmark through optics, computer calculates spacecraft position
; Apollo 11 Context: Lunar landmark tracking during orbits, Earth nav during coast
; Format: DPDEG(90), DPDEG(90), POS5 with DP3, DP3, DP1 routines
		ECADR	LANDLAT			# 89	DPDEG(90)		DP3
		ECADR	LANDLONG		# 89	DPDEG(90)		DP3
		ECADR	LANDALT			# 89	POS5			DP1

; Noun 90: RENDEZVOUS RADAR DATA (RANGE, RANGE RATE, THETA)
; Component 1: RANGE - Slant range to target (XXX.XX nautical miles)
; Component 2: RRATE - Range rate/closing velocity (XXXX.X feet per second)
; Component 3: RTHETA - Radar antenna angle theta (XXX.XX degrees, 0-360)
; Mission Usage: Displays rendezvous radar measurements to target spacecraft
; Range is line-of-sight distance, rate is rate of change (positive = closing)
; Theta is antenna pointing angle around spacecraft axis
; Apollo 11 Context: LM tracking CM during rendezvous phase after lunar ascent
; Format: POS5, VEL3, DPDEG(360) with DP1, DP3, DP4 routines
		ECADR	RANGE			# 90	POS5			DP1
		ECADR	RRATE			# 90	VEL3			DP3
		ECADR	RTHETA			# 90	DPDEG(360)		DP4

; Noun 91: IMU CDU ANGLES (SHAFT AND TRUNNION)
; Component 1: CDUS - IMU shaft angle from coupling data unit (XXX.XX deg)
; Component 2: CDUT - IMU trunnion angle from coupling data unit (XX.XXX deg, max 89.999)
; Component 3: Spare (no third component for this noun)
; Mission Usage: Displays IMU gimbal angles measured by coupling data units (CDUs)
; CDUs are synchros that measure actual gimbal positions of stable platform
; Shaft angle is outer gimbal rotation, trunnion is middle gimbal elevation
; Used for gimbal lock avoidance monitoring and platform alignment verification
; Apollo 11 Context: Gimbal angle monitoring throughout mission for lock avoidance
; Format: CDU DEG, Y OPTICS DEG, SPARE with CDU, YOPT, (none) routines
		ECADR	CDUS			# 91	CDU DEG			CDU
		ECADR	CDUT			# 91	Y OPTICS DEG		YOPT
		OCT	0			# 91	SPARE COMPONENT

; Noun 92: OPTICS COUPLING DATA UNIT ANGLES (SHAFT AND TRUNNION)
; Component 1: SAC - Optics shaft angle coupling (XXX.XX deg)
; Component 2: PAC - Optics trunnion angle coupling (XX.XXX deg, max 89.999)
; Component 3: Spare (no third component for this noun)
; Mission Usage: Displays optics telescope gimbal angles from CDUs
; SAC (shaft angle coupling) measures telescope azimuth rotation
; PAC (trunnion angle coupling) measures telescope elevation angle
; Used during manual star sightings and landmark tracking operations
; Apollo 11 Context: Sextant/telescope pointing angles during P51/P52 alignment
; Format: CDU DEG, Y OPTICS DEG, SPARE with CDU, YOPT, (none) routines
		ECADR	SAC			# 92	CDU DEG			CDU
		ECADR	PAC			# 92	Y OPTICS DEG		YOPT
		OCT	0			# 92	SPARE COMPONENT

; Noun 93: OUTER GIMBAL ANGLES (OGC - OUTER GIMBAL COMMAND/CALCULATED)
; Component 1: OGC - Outer gimbal angle X component (XX.XXX deg, range ±90 degrees)
; Component 2: OGC+2 - Outer gimbal angle Y component (XX.XXX deg, range ±90 degrees)
; Component 3: OGC+4 - Outer gimbal angle Z component (XX.XXX deg, range ±90 degrees)
; Mission Usage: Displays commanded or calculated outer gimbal angles
; OGC represents desired IMU platform orientation angles
; Used in gimbal drive control system for platform reorientation
; Three Euler angles define platform attitude relative to spacecraft body
; Apollo 11 Context: Platform alignment verification during course corrections
; Format: DPDEG(90), DPDEG(90), DPDEG(90) with DP3, DP3, DP3 routines
		ECADR	OGC			# 93	DPDEG(90)		DP3
		ECADR	OGC +2			# 93	DPDEG(90)		DP3
		ECADR	OGC +4			# 93	DPDEG(90)		DP3

; Noun 94: MARK BUFFER OPTICS ANGLES (STORED SIGHTING DATA)
; Component 1: MRKBUF1+3 - Marked shaft angle (XXX.XX deg)
; Component 2: MRKBUF1+5 - Marked trunnion angle (XX.XXX deg, max 89.999)
; Component 3: Spare (no third component for this noun)
; Mission Usage: Displays stored optics angles from previous crew mark/sighting
; MRKBUF1 buffer stores sextant/telescope pointing angles when crew marks target
; Used to recall and verify star or landmark sighting angles
; Allows review of marked position before accepting measurement for navigation
; Apollo 11 Context: Verification of star sightings during P51/P52 alignment checks
; Format: CDU DEG, Y OPTICS DEG, SPARE with CDU, YOPT, (none) routines
		ECADR	MRKBUF1 +3		# 94	CDU DEG			CDU
		ECADR	MRKBUF1 +5		# 94 	Y OPTICS DEG		YOPT
		OCT	00000			# 94 	SPARE

; Noun 95: PREFERRED TRACKING ATTITUDE (PRAXIS)
; Component 1: PRAXIS - Preferred tracking axis 1 gimbal angle (XXX.XX deg)
; Component 2: PRAXIS+1 - Preferred tracking axis 2 gimbal angle (XXX.XX deg)
; Component 3: PRAXIS+2 - Preferred tracking axis 3 gimbal angle (XXX.XX deg)
; Mission Usage: Displays computed preferred spacecraft attitude for star tracking
; PRAXIS defines optimal attitude to keep optics pointed at navigation star
; Minimizes gimbal motion and avoids gimbal lock during star tracking operations
; Computer calculates best attitude considering current IMU gimbal positions
; Apollo 11 Context: Optimal attitude computation for extended star tracking periods
; Format: CDU DEG, CDU DEG, CDU DEG with CDU, CDU, CDU routines
		ECADR	PRAXIS			# 95	CDU DEG			CDU
# Page 283
		ECADR	PRAXIS +1		# 95	CDU DEG			CDU
		ECADR	PRAXIS +2		# 95	CDU DEG			CDU

; Noun 96: COMMANDED PLATFORM ATTITUDE (CPHIX - CHI ANGLES)
; Component 1: CPHIX - Commanded platform attitude angle 1 (XXX.XX deg)
; Component 2: CPHIX+1 - Commanded platform attitude angle 2 (XXX.XX deg)
; Component 3: CPHIX+2 - Commanded platform attitude angle 3 (XXX.XX deg)
; Mission Usage: Displays commanded IMU platform orientation angles
; CPHIX represents desired platform gimbal angles (chi angles) for alignment
; Used during platform reorientation maneuvers and course correction burns
; Three Euler angles defining commanded inertial attitude of stable platform
; Apollo 11 Context: Platform alignment commands during SPS burns and maneuvers
; Format: CDU DEG, CDU DEG, CDU DEG with CDU, CDU, CDU routines

		ECADR	CPHIX			# 96	CDU DEG			CDU
		ECADR	CPHIX +1		# 96	CDU DEG			CDU
		ECADR	CPHIX +2		# 96	CDU DEG			CDU

; Noun 97: DISPLAY TEMPORARY REGISTER 1 (DSPTEM1 - MULTI-PURPOSE BUFFER)
; Component 1: DSPTEM1 - Display temporary storage 1 (whole number)
; Component 2: DSPTEM1+1 - Display temporary storage 2 (whole number)
; Component 3: DSPTEM1+2 - Display temporary storage 3 (whole number)
; Mission Usage: General-purpose temporary storage for display calculations
; DSPTEM1 serves as working buffer for intermediate display formatting results
; Used by multiple programs to prepare data before final DSKY presentation
; Allows programs to stage multi-component displays before committing to screen
; Apollo 11 Context: Used throughout mission for temporary display computations
; Format: WHOLE, WHOLE, WHOLE with ARTH, ARTH, ARTH routines

		ECADR	DSPTEM1			# 97	WHOLE			ARTH
		ECADR	DSPTEM1 +1		# 97	WHOLE			ARTH
		ECADR	DSPTEM1 +2		# 97	WHOLE			ARTH

; Noun 98: DISPLAY TEMPORARY REGISTER 2 (DSPTEM2 - MIXED FORMAT BUFFER)
; Component 1: DSPTEM2 - Display temporary storage 1 (whole number)
; Component 2: DSPTEM2+1 - Display temporary storage 2 (fractional value)
; Component 3: DSPTEM2+2 - Display temporary storage 3 (whole number)
; Mission Usage: Alternative temporary storage with mixed whole/fractional format
; DSPTEM2 provides second buffer allowing simultaneous display staging
; Mixed format (whole, fractional, whole) suits specific display patterns
; Fractional middle component useful for ratio or percentage displays
; Apollo 11 Context: Used for special display formatting throughout mission
; Format: WHOLE, FRAC, WHOLE with ARTH, FRAC, ARTH routines

		ECADR	DSPTEM2			# 98	WHOLE			ARTH
		ECADR	DSPTEM2 +1		# 98	FRAC			FRAC
		ECADR	DSPTEM2 +2		# 98	WHOLE			ARTH

; Noun 99: WIND/WEATHER VECTOR (REENTRY ATMOSPHERIC CONDITIONS)
; Component 1: WWPOS - Wind/weather position/altitude (XXXXX. feet)
; Component 2: WWVEL - Wind velocity magnitude (XXXX.X feet per second)
; Component 3: WWOPT - Wind/weather option or direction indicator (whole number)
; Mission Usage: Atmospheric wind data for reentry guidance computations
; WWPOS indicates altitude band for wind model, WWVEL gives wind speed
; WWOPT may specify wind direction or atmospheric model selection
; Used during P67 final reentry phase for trajectory adjustments
; Apollo 11 Context: Wind compensation during July 24 reentry to Pacific Ocean
; Format: POS9, VEL4, WHOLE with DP3, DP2, ARTH routines

		ECADR	WWPOS			# 99 	POS9			DP3
		ECADR	WWVEL			# 99	VEL4			DP2
		ECADR	WWOPT			# 99 	WHOLE			ARTH
# END OF IDADDTAB

; ============================================================================
; TRANSITION: From IDADDTAB (Address Table) to RUTMXTAB (Routine Mixture Table)
;
; Having defined the memory addresses for each noun component, the AGC now
; specifies which scale factor (SF) routines process each noun's data for
; display and input. RUTMXTAB contains the routine mixture codes that determine
; how values are formatted (decimal, octal, degrees, time, etc.) for DSKY
; presentation. Each octal code encodes up to three SF routine selections,
; one for each component of the noun's triple-register display format.
; ============================================================================

; ----------------------------------------------------------------------------
; RUTMXTAB: ROUTINE MIXTURE TABLE (SF ROUTINE SELECTION FOR NOUNS 40-99)
; ----------------------------------------------------------------------------
; Purpose: Specifies Scale Factor (SF) routines for formatting noun displays
; 
; Structure: Each entry is an octal value encoding the SF routine codes for
;            a noun's three components (or fewer for 1 or 2-component nouns)
; 
; Octal Encoding Format:
;   Bits 0-4:   SF routine for component 3 (rightmost in octal value)
;   Bits 5-9:   SF routine for component 2
;   Bits 10-14: SF routine for component 1 (leftmost in octal value)
; 
; SF Routine Codes (from earlier definition section):
;   00000 = OCT (Octal only display)
;   00001 = FRAC (Straight fractional)
;   00010 = CDU (CDU degrees XXX.XX)
;   00011 = ARTH (Arithmetic scale factor)
;   00100 = DP1 (Arith DP1: OUT mult by 2^14, IN straight)
;   00101 = DP2 (Arith DP2: OUT straight, IN shift left 7)
;   00110 = YOPT (Y Optics degrees XX.XXX max 89.999)
;   00111 = DP3 (Arith DP3: OUT shift left 7, IN straight)
;   01000 = HMS (Hours:Minutes:Seconds format)
;   01001 = M/S (Minutes:Seconds format)
;   01010 = DP4 (Arith DP4: OUT straight, IN shift left 3)
;   01011 = ARITH1 (Arith1 SF: OUT mult by 2^14, IN straight)
;   01100 = 2INT (Two integers format)
;   01101 = DPFRAC (Double precision fractional)
; 
; Mission Context: RUTMXTAB defines how every noun value is scaled and formatted
; throughout Apollo 11 mission. Whether displaying altitude during descent,
; velocity during rendezvous, or time to ignition, RUTMXTAB determines the
; presentation format Armstrong and Aldrin saw on the DSKY.
; 
; Usage: Display Interface Routines (DISPLAY_INTERFACE_ROUTINES.agc) use
;        RUTMXTAB to select appropriate formatting for each noun component
;        based on whether display is input (IN) or output (OUT) mode.
; ----------------------------------------------------------------------------


						# NN	SF ROUTINES

; Noun 40: TIME OF IGNITION (TIG) - M/S + position + velocity formats
; Used during maneuver planning to display time-to-ignition with position/velocity
RUTMXTAB	OCT	16351			# 40	M/S, DP3, DP3

; Noun 41: GIMBAL ANGLES - CDU degrees and arithmetic formats
; Displays IMU gimbal angles for alignment monitoring
		OCT	00142			# 41	CDU, ARTH

; Noun 42: APOGEE/PERIGEE/TIME - Altitude parameters with DP3 scaling
; Orbital mechanics display: apogee altitude, perigee altitude, time
		OCT	16347			# 42	DP3, DP3, DP3

; Noun 43: ORBITAL PARAMETERS - Mixed DP4/DP3 for velocity and position
; Used during orbital calculations with velocity components
		OCT	16512			# 43	DP4, DP4, DP3

; Noun 44: APSIS DATA - Position components with minutes/seconds time
; Displays apsis (apogee/perigee) information with time component
		OCT	22347			# 44	DP3, DP3, M/S

; Noun 45: MARK DATA - Integer marks with time and velocity
; Used for landmark tracking: two integer IDs with time and position
		OCT	24454			# 45	2INT, M/S, DP4

; Noun 46: COMMAND MODULE DAC ACTIVITY - Octal display format
; Digital-to-Analog Converter status displayed in pure octal
		OCT	00000			# 46	OCT, OCT

; Noun 47: THRUST MONITOR - Arithmetic scale factors for thrust values
; Monitors engine thrust levels with high-precision arithmetic formatting
		OCT	00553			# 47	ARITH1, ARITH1

; Noun 48: DELTA VELOCITY - Standard arithmetic format
; Displays velocity change components for maneuver planning
		OCT	00143			# 48	ARTH, ARTH

; Noun 49: DELTA VELOCITY TO CUTOFF - Mixed DP3 and arithmetic
; Remaining velocity change until engine cutoff
		OCT	06347			# 49	DP3, DP3, ARTH

; Noun 50: SURFACE POSITION - Latitude/longitude with time
; Landing site or surface coordinates with minutes/seconds timestamp
		OCT	22347			# 50	DP3, DP3, M/S
; Noun 51: DELTA ALTITUDE/DELTA V - Velocity components DP4 scaling
; Altitude rate and velocity change with high-precision DP4 format
		OCT	00512			# 51	DP4, DP4

; Noun 52: SINGLE VELOCITY - Single component DP4 format
; Single velocity value with DP4 precision scaling
		OCT	00012			# 52	DP4

; Noun 53: ANGLE SET - Mixed precision angles
; Three angles with DP1, DP3, and DP4 mixed scaling
		OCT	24344			# 53 	DP1, DP3, DP4

; Noun 54: ANGLE SET (ALTERNATE) - Mixed precision angles
; Similar to N53 with DP1, DP3, DP4 scaling
		OCT	24344			# 54	DP1, DP3, DP4

; Noun 55: THRUST ACCELERATION - Arithmetic + velocity DP4
; Thrust vector components: arithmetic and DP4 velocity formats
		OCT	24503			# 55	ARTH, DP4 , DP4

; Noun 56: VELOCITY COMPONENTS - Dual DP4 format
; Two velocity components with DP4 high-precision scaling
		OCT	00512			# 56	DP4, DP4

; Noun 57: SINGLE POSITION - DP3 format
; Single position/altitude value with DP3 precision
		OCT	00007			# 57	DP3

; Noun 58: POSITION TRIPLET - Three DP3 components
; Three position coordinates all using DP3 scaling
		OCT	16347			# 58	DP3, DP3, DP3

; Noun 59: POSITION TRIPLET (ALTERNATE) - Three DP3 components
; Additional position vector display with DP3 scaling
		OCT	16347			# 59	DP3, DP3, DP3

; Noun 60: VELOCITY/THRUST DATA - Mixed arithmetic and DP4
; Combines arithmetic thrust value with two DP4 velocity components
		OCT	24503			# 60	ARTH, DP4, DP4
; Noun 61: VELOCITY WITH ANGLE - Two DP4 velocities + arithmetic angle
; Velocity components with angular component in arithmetic format
		OCT	06512			# 61	DP4, DP4, ARTH

; Noun 62: VELOCITY/POSITION MIX - Two DP4 velocities + DP3 position
; Mixed velocity and position data for trajectory calculations
		OCT	16512			# 62	DP4, DP4, DP3

; Noun 63: MANEUVER DATA - Position + velocity + time
; Complete maneuver specification: DP3 position, DP4 velocity, M/S time
		OCT	22507			# 63	DP3, DP4, M/S

; Noun 64: MIXED DYNAMICS - DP2 drag + DP4 velocity + DP3 position
; Entry dynamics: drag (DP2), velocity (DP4), position (DP3)
		OCT	16505			# 64	DP2, DP4, DP3

; Noun 65: TIME TRIPLET - Three HMS time displays
; Three separate time values in Hours:Minutes:Seconds format
		OCT	20410			# 65	HMS, HMS, HMS

; Noun 66: MIXED PRECISION VELOCITY/POSITION - DP4 + two DP3
; High precision velocity (DP4) with two DP3 position components
		OCT	16352			# 66	DP4, DP3, DP3

; Noun 67: MIXED PRECISION DATA - DP3 + two DP4
; Combines DP3 position with two DP4 velocity/acceleration values
		OCT	24507			# 67	DP3, DP4, DP4

; Noun 68: TRIPLE VELOCITY - Three DP4 components
; Three velocity components all with high-precision DP4 scaling
		OCT	24512			# 68	DP4, DP4, DP4

; Noun 69: MIXED DP VELOCITY - Three different DP scales
; Complex mixed precision: DP4, DP2, DP4 for specialized calculations
		OCT	24252			# 69	DP4, DP2, DP4

; Noun 70: OCTAL TRIPLET - Three octal values
; Three raw octal values for system diagnostics or bit patterns
		OCT	00000			# 70	OCT, OCT, OCT
# Page 284

; Noun 71: OCTAL TRIPLET (ZERO) - Placeholder noun
; Zero value indicating unused noun or default octal display
		OCT	0			# 71	OCT, OCT, OCT

; Noun 72: MIXED DYNAMICS DISPLAY - Velocity + position + angle
; Two precision levels: DP4 velocity, DP3 position, arithmetic angle
		OCT	06352			# 72	DP4, DP3, ARTH

; Noun 73: DOUBLE PRECISION + VELOCITY - DPR (DP fractional) + two DP4
; High-precision fractional value with two DP4 velocity components
		OCT	24507			# 73	DPR, DP4, DP4

; Noun 74: MIXED VELOCITY/DRAG - Two DP4 velocities + DP2 drag
; Velocity components (DP4) with drag acceleration (DP2)
		OCT	12512			# 74	DP4, DP4, DP2

; Noun 75: POSITION/TIME DATA - DP3 position + two M/S times
; Position value with two minutes:seconds time components
		OCT	22447			# 75	DP3, M/S, M/S

; Noun 76: SPARE - Reserved noun
; Unused noun slot, available for future mission requirements
		OCT	0			# 76	SPARE

; Noun 77: SPARE - Reserved noun
; Unused noun slot, available for future mission requirements
		OCT	0			# 77	SPARE

; Noun 78: SPARE - Reserved noun
; Unused noun slot, available for future mission requirements
		OCT	0			# 78	SPARE

; Noun 79: SPARE - Reserved noun
; Unused noun slot, available for future mission requirements
		OCT	0			# 79	SPARE

; Noun 80: TIME/VELOCITY DATA - M/S time + two DP4 velocities
; Minutes:seconds time with two high-precision velocity components
		OCT	24511			# 80	M/S, DP4, DP4

; Noun 81: POSITION TRIPLET (DP3) - Three DP3 position/distance components
; Standard triple DP3 format for three-axis position or distance display
; Used throughout mission for position vector displays (X, Y, Z coordinates)
		OCT	16347			# 81	DP3, DP3, DP3

; Noun 82: POSITION TRIPLET (DP3) - Three DP3 position/distance components
; Identical to N81, standard triple DP3 format for position vectors
		OCT	16347			# 82	DP3, DP3, DP3

; Noun 83: POSITION TRIPLET (DP3) - Three DP3 position/distance components
; Identical to N81-82, standard triple DP3 format for position vectors
		OCT	16347			# 83	DP3, DP3, DP3

; Noun 84: POSITION TRIPLET (DP3) - Three DP3 position/distance components
; Identical to N81-83, standard triple DP3 format for position vectors
		OCT	16347			# 84	DP3, DP3, DP3

; Noun 85: POSITION TRIPLET (DP3) - Three DP3 position/distance components
; Identical to N81-84, standard triple DP3 format for position vectors
		OCT	16347			# 85	DP3, DP3, DP3

; Noun 86: VELOCITY TRIPLET (DP4) - Three DP4 velocity components
; High-precision triple DP4 format for three-axis velocity vectors (VX, VY, VZ)
; DP4 provides higher precision than DP3 for accurate velocity displays
		OCT	24512			# 86	DP4, DP4, DP4

; Noun 87: GIMBAL ANGLES - CDU degrees + Y optics degrees
; Two-component display: CDU gimbal angle (XXX.XX°) + Y optics angle (XX.XXX°)
; Used for IMU gimbal position and optical telescope alignment displays
		OCT	00302			# 87	CDU, YOPT

; Noun 88: DOUBLE PRECISION FRACTIONAL TRIPLET - Three DPFRAC components
; Each component displays as double-precision fractional value
; Used for high-precision fractional data displays (ratios, coefficients)
		OCT	32655			# 88	DPFRAC FOR EACH

; Noun 89: MIXED PRECISION TRIPLET - Two DP3 + one DP1 component
; DP3, DP3, DP1 format for mixed-precision display of related parameters
		OCT	10347			# 89	DP3, DP3, DP1

; Noun 90: MIXED PRECISION TRIPLET - DP1 + DP3 + DP4 components
; Three different precision levels for displaying heterogeneous data set
; DP1 (high multiplier), DP3 (medium), DP4 (straight) precisions
		OCT	24344			# 90	DP1, DP3, DP4

; Noun 91: GIMBAL ANGLES - CDU degrees + Y optics degrees
; Two-component display: CDU gimbal angle + Y optics angle
; Identical format to N87, used for alternate gimbal/optics display
		OCT	00302			# 91	CDU, YOPT

; Noun 92: GIMBAL ANGLES - CDU degrees + Y optics degrees
; Two-component display: CDU gimbal angle + Y optics angle
; Identical format to N87, N91, for additional gimbal/optics display option
		OCT	00302			# 92	CDU, YOPT

; Noun 93: POSITION TRIPLET (DP3) - Three DP3 position/distance components
; Standard triple DP3 format, identical to N81-85
; Additional noun slot for position vector display
		OCT	16347			# 93	DP3, DP3, DP3

; Noun 94: GIMBAL ANGLES - CDU degrees + Y optics degrees
; Two-component display: CDU gimbal angle + Y optics angle
; Identical format to N87, N91-92, for alternate gimbal/optics display
		OCT	00302			# 94	CDU, YOPT

; Noun 95: CDU ANGLE TRIPLET - Three CDU degree components
; Three CDU gimbal angles (XXX.XX° format each)
; Used for displaying complete IMU gimbal set (inner, middle, outer)
; Critical during gimbal lock avoidance procedures throughout mission
		OCT	04102			# 95	CDU, CDU, CDU

; Noun 96: CDU ANGLE TRIPLET - Three CDU degree components
; Three CDU gimbal angles, identical format to N95
; Alternate noun for complete gimbal angle set display
		OCT	04102			# 96	CDU, CDU, CDU

; Noun 97: ARITHMETIC TRIPLET - Three arithmetic scale factor components
; Triple arithmetic format for general-purpose integer/scaled displays
; Used for counts, indices, or other integer-type mission data
		OCT	06143			# 97	ARTH, ARTH, ARTH

; Noun 98: MIXED ARITHMETIC/FRACTIONAL - ARTH + FRAC + ARTH components
; Arithmetic, fractional, arithmetic format for mixed data display
; Center component displays as pure fractional, outer two as arithmetic
		OCT	06043			# 98	ARTH, FRAC, ARTH

; Noun 99: MIXED PRECISION TRIPLET - DP3 + DP2 + ARTH components
; Three different format types: DP3 position, DP2 value, arithmetic integer
; Maximum noun number, used for specialized multi-type data display
		OCT	06247			# 99	DP3, DP2, ARTH

; ============================================================================
; END OF RUTMXTAB - Scale Factor Routine Mixture Table Complete
;
; All 60 nouns (40-99) now have defined SF routine mixtures. The DSKY Display
; Interface Routines use RUTMXTAB to format every noun value displayed to the
; crew during Apollo 11 mission. From launch through splashdown, this table
; determined how Armstrong, Aldrin, and Collins saw critical mission data:
; altitude during descent, velocity during rendezvous, gimbal angles during
; platform alignment, and time to ignition for every propulsive maneuver.
; ============================================================================
# END OF RUTMXTAB


		SBANK=	LOWSUPER
