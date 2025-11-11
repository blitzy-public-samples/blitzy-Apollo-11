# Copyright:	Public domain.
# Filename:	FLAGWORD_ASSIGNMENTS.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Pages:	0061-0089
# Mod history:	2009-05-15 OH	Transcribed from page images.
#		2009-05-17 RSB	Extended to (blank) p. 89.
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
; FILE: FLAGWORD_ASSIGNMENTS.agc
; MODULE: Core Information and Memory
; MISSION PHASE: all phases
;
; TL;DR: Defines all software flag bit assignments used throughout the Lunar
;        Module guidance software for state control, mode selection, and
;        inter-program communication. Organizes 14 flagwords (210 individual
;        bits) controlling navigation modes, landing sequences, rendezvous
;        operations, display states, and system configurations. Includes the
;        famous FLAGORGY flags that manage lunar descent operational modes.
;
; COMMENT-ONLY READERS: This is a technical reference file. You may skip it
;        initially and return when you encounter flag references in mission
;        programs like THE_LUNAR_LANDING.agc.
; CODE-ALONG READERS: Study this flag organization to understand the LM's
;        software state machine architecture. Flags control program flow,
;        enable/disable features, and coordinate between subsystems.
; ============================================================================


# Page 61
; 
; SOFTWARE FLAG ARCHITECTURE OVERVIEW
;
; The AGC uses 14 flagwords (FLAGWRD0 through FLGWRD13) to maintain program
; state and coordinate operations between subsystems. Each flagword contains
; 15 flag bits, providing 210 individual binary state indicators.
;
; FLAG ORGANIZATION BY MISSION FUNCTION:
;
; LANDING OPERATIONS FLAGS (used during powered descent):
;   NOTHROTL (BIT 15 FLAG 4) - Throttle control enable/disable
;   MUNFLAG  (BIT 14 FLAG 5) - Descent guidance operational mode
;   REDFLAG  (BIT 12 FLAG 5) - Landing site redesignation by crew
;   LRBYPASS (BIT 14 FLAG 7) - Landing radar bypass for altitude data
;   These flags were set/cleared during Apollo 11's descent on July 20, 1969
;
; RENDEZVOUS OPERATIONS FLAGS (used during ascent and docking):
;   P25FLAG  (BIT 9 FLAG 0) - P25 rendezvous tracking program active
;   RNDVZFLG (BIT 7 FLAG 0) - P20 rendezvous navigation radar in use
;
; NAVIGATION MODE FLAGS:
;   Control which celestial body is sphere of influence (MOONFLAG)
;   Select integration method for trajectory propagation (MIDFLAG)
;   Enable/disable sensor data incorporation (UPDATFLG, AVEGFLAG)
;
; DISPLAY AND CREW INTERFACE FLAGS:
;   Control DSKY display modes, keyboard input processing, and crew alerts
;   Enable verb/noun operations and display formatting
;
; SYSTEM STATE FLAGS:
;   Track IMU status, engine states, autopilot modes, abort conditions
;   Coordinate between guidance, navigation, and control subsystems
;
; FLAG MANIPULATION:
;   - Interpreter: UP-FLAG and DOWN-FLAG instructions (FLAGWRDS 0-11)
;   - Native code: BIT manipulation instructions on STATE memory locations
;   - Downlink: FLAGWRDS 0-13 telemetered to Mission Control for monitoring
;
# FLAGWORDS 0-11	ARE DOWNLINKED AND CAN BE SET AND CLEARED BY UP-FLAG AND DOWN-FLAG INSTRUCTIONS IN THE
#			INTERPRETER.  THESE WERE PREVIOUSLY LISTED UNDER "INTERPRETIVE SWITCH BIT ASSIGNMENTS" IN
#			THE ERASABLE LOG SECTION.  FLAGWORDS 12 & 13 WERE PREVIOUSLY RADMODES AND DAPBOOLS AND
#			ARE STILL DOWNLINKED UNDER THOSE NAMES.

# 		ALPHABETICAL LIST OF FLAGWORDS
;
; This alphabetical index provides quick reference to all 210 flag bits.
; Each entry shows: flag name, decimal bit number (0-209), bit position
; within flagword (BIT 1-15), flagword number (FLAG 0-13), and bit name.
; 
; Detailed descriptions with SET/RESET states appear in FLAGWRD sections below.
; Cross-reference this list when programs reference flags by name.
;
#
# FLAGWORD	DEC. NUMBER	BIT AND FLAG		BIT NAME

# ACCOKFLG	207		BIT  3 FLAG 13		ACCSOKAY
# ACC4-2FL	199		BIT 11 FLAG 13		ACC4OR2X
# ACMODFLG	032		BIT 13 FLAG  2		ACMODBIT
# ALTSCALE	186		BIT  9 FLAG 12		ALTSCBIT
# ANTENFLG	183		BIT 12 FLAG 12		ANTENBIT
# AORBSFLG	205		BIT  5 FLAG 13		AORBSYST
# AORBTFLG	200		BIT 10 FLAG 13		AORBTRAN
# APSESW	130		BIT  5 FLAG  8		APSESBIT
# APSFLAG	152		BIT 13 FLAG 10		APSFLBIT
# ASTNFLAG	108		BIT 12 FLAG  7		ASTNBIT
# ATTFLAG	104		BIT  1 FLAG  6		ATTFLBIT
# AUTOMODE	193		BIT  2 FLAG 12		AUTOMBIT
# AUTR1FLG	209		BIT  1 FLAG 13		AUTRATE1
# AUTR2FLG	208		BIT  2 FLAG 13		AUTRATE2
# AUXFLAG	103		BIT  2 FLAG  6		AUXFLBIT
# AVEGFLAG	115		BIT  5 FLAG  7		AVEGFBIT
# AVEMIDSW	149		BIT  1 FLAG  9		AVEMDBIT
# AVFLAG	040		BIT  5 FLAG  2		AVFLBIT
# CALCMAN2	043		BIT  2 FLAG  2		CALC2BIT
# CALCMAN3	042		BIT  3 FLAG  2		CALC3BIT
# CDESFLAG	180		BIT 15 FLAG 12		CDESBIT
# CMOONFLG	123		BIT 12 FLAG  8		CMOONBIT
# COGAFLAG	131		BIT  4 FLAG  8		COGAFBIT
# CSMDKFLG	197		BIT 13 FLAG 13		CSMDOCKD
# CULTFLAG	053		BIT  7 FLAG  3		CULTBIT
# DAPBOOLS			FLGWRD13
# DBSELFLG	206		BIT  4 FLAG 13		DBSELECT
# DESIGFLG	185		BIT 10 FLAG 12		DESIGBIT
# DIDFLAG	016		BIT 14 FLAG		DIDFLBIT
# DIMOFLAG	059		BIT  1 FLAG  3		DIMOBIT
# DMENFLG	081		BIT  9 FLAG  5		DMENFBIT
# DRIFTDFL	202		BIT  8 FLAG 13		DRIFTBIT
# DRIFTFLG	030		BIT 15 FLAG  2		DRFTBIT
# DSKYFLAG	075		BIT 15 FLAG  5		DSKYFBIT
# Page 62
# D6OR9FLG	058		BIT  2 FLAG  3		D6OR9BIT
# ENGONFLG	083		BIT  7 FLAG  5		ENGONBIT
# ERADFLAG	017		BIT 13 FLAG  1		ERADFBIT
# ETPIFLAG	038		BIT  7 FLAG  2		ETPIBIT		EQUIVALENT FLAG NAME: DPTNSW
# FINALFLG	039		BIT  6 FLAG  2		FINALBIT
# FLAGWRD0	(000-014)	(STATE +0)
# FLAGWRD1	(015-029)	(STATE +1)
# FLAGWRD2	(030-044)	(STATE +2)
# FLAGWRD3	(045-059)	(STATE +3)
# FLAGWRD4	(060-074)	(STATE +4)
# FLAGWRD5	(075-089)	(STATE +5)
# FLAGWRD6	(090-104)	(STATE +6)
# FLAGWRD7	(105-119)	(STATE +7)
# FLAGWRD8	(120-134)	(STATE +8D)
# FLAGWRD9	(135-149)	(STATE +9D)
# FLAP		142		BIT  8 FLAG  9		FLAPBIT
# FLGWRD10	(150-164)	(STATE +10D)
# FLGWRD11	(165-179)	(STATE +11D)
# FLGWRD12	(180-194)	(STATE +12D)
# FLGWRD13	(195-209)	(STATE +13D)
# FLPC		138		BIT 12 FLAG  9		FLPCBIT
# FLPI		139		BIT 11 FLAG  9		FLPIBIT
# FLRCS		149		BIT 10 FLAG  9		FLRCSBIT
# FLUNDISP	125		BIT 10 FLAG  8		FLUNDBIT
# FLVR		136		BIT 14 FLAG  9		FLVRBIT
# FREEFLAG	012		BIT  3 FLAG  0		FREEFBIT
# FSPASFLG	005		BIT 10 FLAG  0		FSPASBIT
# GLOKFAIL	046		BIT 14 FLAG  3		GLOKFBIT
# GMBDRVSW	095		BIT 10 FLAG  6		GMBDRBIT
# GUESSW	028		BIT  2 FLAG  1		GUESSBIT
# HFLSHFLG	179		BIT  1 FLAG 11		HFLSHBIT
# IDLEFLAG	113		BIT  7 FLAG  7		IDLEFBIT
# IGNFLAG	107		BIT 13 FLAG  7		IGNFLBIT
# IMPULSW	036		BIT  9 FLAG  2		IMPULBIT
# IMUSE		007		BIT  8 FLAG  0		IMUSEBIT
# INFINFLG	128		BIT  7 FLAG  8		INFINBIT
# INITALGN	133		BIT  2 FLAG  8		INITABIT
# INTFLAG	151		BIT 14 FLAG 10		INTFLBIT
# INTYPFLG	056		BIT  4 FLAG  3		INTYPBIT
# ITSWICH	105		BIT 15 FLAG  7		ITSWBIT
# JSWITCH	001		BIT 14 FLAG  0		JSWCHBIT
# LETABORT	141		BIT  9 FLAG  9		LETABBIT
# LMOONFLG	124		BIT 11 FLAG  8		LMOONBIT
# LOKONSW	010		BIT  5 FLAG  0		LOKONBIT
# LOSCMFLG	033		BIT 12 FLAG  2		LOSCMBIT
# LRALTFLG	190		BIT  5 FLAG 12		LRALTBIT
# LRBYPASS	165		BIT 15 FLAG 11		LRBYBIT
;				FLAGORGY FLAG: Bypass landing radar altitude data during descent.
;				SET=use alternate altitude source, RESET=use landing radar.
;				Referenced in THE_LUNAR_LANDING.agc P63 braking phase.
# LRINH		172		BIT  8 FLAG 11		LRINHBIT
# LRPOSFLG	189		BIT  6 FLAG 12		LRPOSBIT
# LRVELFLG	187		BIT  8 FLAG 12		LRVELBIT
# Page63
# LUNAFLAG	048		BIT 12 FLAG  3		LUNABIT
# MANUFLAG	106		BIT 14 FLAG  7		MANUFBIT
# MGLVFLAG	088		BIT  2 FLAG  5		MGLVFBIT
# MIDAVFLG	148		BIT  2 FLAG  9		MIDAVBIT
# MIDFLAG	002		BIT 13 FLAG  0		MIDFLBIT
# MID1FLAG	147		BIT  3 FLAG  9		MID1BIT
# MKOVFLAG	072		BIT  3 FLAG  4		MKOVBIT
# MOONFLAG	003		BIT 12 FLAG  0		MOONBIT
# MRKIDFLG	060		BIT 15 FLAG  4		MRKIDBIT
# MRKNVFLG	066		BIT  9 FLAG  4		MRKNVBIT
# MRUPTFLG	070		BIT  5 FLAG  4		MRUPTBIT
# MUNFLAG	097		BIT  8 FLAG  6		MUNFLBIT
;				FLAGORGY FLAG: Descent guidance operational mode indicator.
;				SET=manual throttle mode, RESET=automatic guidance control.
;				Critical during Apollo 11 landing when Armstrong took semi-manual
;				control. Referenced in THE_LUNAR_LANDING.agc.
# MWAITFLG	064		BIT 11 FLAG  4		MWAITBIT
# NEEDLFLG	011		BIT  4 FLAG  0		NEEDLBIT
# NEWIFLG	122		BIT 13 FLAG  8		NEWIBIT
# NJETSFLG	015		BIT 15 FLAG		NJETSBIT
# NODOFLAG	044		BIT  1 FLAG  2		NODOBIT
# NOLRREAD	170		BIT 10 FLAG 11		NOLRRBIT
# NORMSW	110		BIT 10 FLAG  7		NORMSBIT
# NORRMON	086		BIT  4 FLAG  5		NORRMBIT
# NOR29FLG	049		BIT 11 FLAG  3		NR29FBIT
# NOTHROTL	078		BIT 12 FLAG  5		NOTHRBIT
;				FLAGORGY FLAG: Throttle control enable/disable for descent engine.
;				SET=throttle disabled (no engine commands), RESET=throttle active.
;				Used during powered descent throttle management. Referenced in
;				THE_LUNAR_LANDING.agc and THROTTLE_CONTROL_ROUTINES.agc.
# NOUPFLAG	024		BIT  6 FLAG  1		NOUPFBIT
# NRMNVFLG	067		BIT  8 FLAG  4		NRMNVBIT
# NRMIDFLG	062		BIT 13 FLAG  4		NRMIDBIT
# NRUPTFLG	071		BIT  4 FLAG  4		NRUPTBIT
# NTARGFLG	102		BIT  3 FLAG  6		NTARGBIT
# NWAITFLG	065		BIT 10 FLAG  4		NWAITBIT
# OLDESFLG	014		BIT  1 FLAG  0		OLDESBIT
# OPTNSW	038		BIT  7 FLAG  2		OPTNBIT		EQUIVALENT FLAG NAME: ETPIFLAG
# ORBWFLAG	054		BIT  6 FLAG  3		ORBWFBIT
# ORDERSW	129		BIT  6 FLAG  8		ORDERBIT
# OURRCFLG	198		BIT 12 FLAG 13		OURRCBIT
# PDSPFLAG	063		BIT 12 FLAG  4		PDSPFBIT
# PFRATFLG	041		BIT  4 FLAG  2		PFRATBIT
# PINBRFLG	069		BIT  6 FLAG  4		PINBRBIT
# PRECIFLG	052		BIT  8 FLAG  3		PRECIBIT
# PRIODFLG	061		BIT 14 FLAG  1		PRIODBIT
# PRONVFLG	068		BIT  7 FLAG  4		PRONVBIT
# PSTHIGAT	169		BIT 11 FLAG 11		PSTHIBIT
# PULSEFLG	195		BIT 15 FLAG 13		PULSES
# P21FLAG	004		BIT 11 FLAG  0		P21FLBIT
# P25FLAG	006		BIT  9 FLAG  0		P25FLBIT
;				FLAGORGY FLAG: P25 rendezvous tracking program active indicator.
;				SET=P25 running (post-ascent rendezvous operations), RESET=inactive.
;				Used during lunar orbit rendezvous between LM and CM after ascent.
;				Referenced in THE_LUNAR_LANDING.agc and P20-P25.agc.
# P39/79SW	126		BIT  9 FLAG  8		P39SWBIT
# QUITFLAG	145		BIT 5 FLAG 9		QUITBIT
# RADMODES			FLGWRD12
# RASFLAG			FLGWRD10
# RCDUFAIL	188		BIT  7 FLAG 12		RCDUFBIT
# RCDU0FLG	182		BIT 13 FLAG 12		RCDU0BIT
# READLR	174		BIT  6 FLAG 11		READLBIT
# Page 64
# READRFLG	051		BIT  9 FLAG  3		READRBIT	EQUIVALENT FLAG NAME FOR R04FLAG
# READVEL	175		BIT  5 FLAG 11		READVBIT
# REDFLAG	099		BIT  6 FLAG  6		REDFLBIT
;				FLAGORGY FLAG: Landing site redesignation by crew.
;				SET=crew has manually selected new landing target, RESET=nominal.
;				Armstrong used manual redesignation at ~500 feet to avoid boulder
;				field during Apollo 11 landing. Referenced in THE_LUNAR_LANDING.agc
;				and LUNAR_LANDING_GUIDANCE_EQUATIONS.agc.
# REFSMFLG	047		BIT 13 FLAG  3		REFSMBIT
# REINTFLG	158		BIT  7 FLAG 10		REINTBIT
# REMODFLG	181		BIT 14 FLAG 12		REMODBIT
# RENDWFLG	089		BIT  1 FLAG  5		RENDWBIT
# REPOSMON	184		BIT 11 FLAG 12		REPOSBIT
# RHCSCFLG	203		BIT  7 FLAG 13		RHCSCALE
# RNDVZFLG	008		BIT  7 FLAG  0		RNDVZBIT
;				FLAGORGY FLAG: P20 rendezvous navigation radar in use.
;				SET=rendezvous radar tracking active, RESET=radar not in use.
;				Used during post-ascent approach to CM for docking. Referenced in
;				THE_LUNAR_LANDING.agc and P20-P25.agc rendezvous programs.
# RNGEDATA	176		BIT  4 FLAG 11		RNGEDBIT
# RNGSCFLG	080		BIT 10 FLAG  5		RNGSCBIT
# RODFLAG	018		BIT 12 FLAG  1		RODFLBIT
# ROTFLAG	144		BIT  6 FLAG  9		ROTFLBIT
# RPQFLAG	120		BIT 15 FLAG  8		RPQFLBIT
# RRDATAFL	191		BIT  4 FLAG 12		RRDATABT
# RRNBSW	009		BIT  6 FLAG  0		RRNBBIT
# RRRSFLAG	192		BIT  3 FLAG 12		RRRSBIT
# RVSW		111		BIT  9 FLAG  7		RVSWBIT
# R04FLAG	051		BIT  9 FLAG  3		R04FLBIT	EQUIVALENT FLAG NAME:  READRFLG
# R10FLAG	013		BIT  2 FLAG  0		R10FLBIT
# R61FLAG	020		BIT 10 FLAG  1		R61FLBIT
# R77FLAG	079		BIT 11 FLAG  5		R77FLBIT
# SCALBAD	177		BIT  3 FLAG 11		SCABBIT
# SLOPESW	027		BIT  3 FLAG  1		SLOPEBIT
# SNUFFER	077		BIT 13 FLAG  5		SNUFFBIT
# SOLNSW	087		BIT  3 FLAG  5		SOLNSBIT
# SRCHOPTN	031		BIT 14 FLAG  2		SRCHOBIT
# STATEFLG	055		BIT  5 FLAG  3		STATEBIT
# STEERSW	034		BIT 11 FLAG  2		STEERBIT
# SURFFLAG	127		BIT  8 FLAG  8		SURFFBIT
# SWANDISP	109		BIT 11 FLAG  7		SWANDBIT
# S32.1F1	090		BIT 15 FLAG  6		S32BIT1
# S32.1F2	091		BIT 14 FLAG  6		S32BIT2
# S32.1F3A	092		BIT 13 FLAG  6		S32BIT3A
# S32.1F3B	093		BIT 12 FLAG  6		S32BIT3B
# TFFSW		119		BIT  1 FLAG  7		TFFSWBIT
# TRACKFLG	025		BIT  5 FLAG  1		TRACKBIT
# TURNONFL	194		BIT  1 FLAG 12		TURNONBT
# ULLAGFLG	204		BIT  6 FLAG 13		ULLAGER
# UPDATFLG	023		BIT  7 FLAG  1		UPDATBIT
# UPLOCKFL	116		BIT  4 FLAG 7		UPLOCBIT
# USEQRFLG	196		BIT 14 FLAG 13		USEQRJTS
# VEHUPFLG	022		BIT  8 FLAG  1		VEHUPBIT
# VELDATA	173		BIT  7 FLAG 11		VELDABIT
# VERIFLAG	117		BIT  3 FLAG  7		VERIFBIT
# VFLAG		050		BIT 10 FLAG  3		VFLAGBIT
# VFLSHFLG	178		BIT  2 FLAG 11		VFLSHBIT
# VINTFLAG	057		BIT  3 FLAG  3		VINTFBIT
# VXINH		168		BIT 12 FLAG 11		VXINHBIT
# Page 65
# V37FLAG	114		BIT  6 FLAG  7		V37FLBIT
# V67FLAG	112		BIT  8 FLAG  7		V67FLBIT
# V82EMFLG	118		BIT  2 FLAG  7		V82EMBIT
# XDELVFLG	037		BIT  8 FLAG  2		XDELVBIT
# XDSPFLAG	074		BIT  1 FLAG  4		XDSPBIT
# XORFLG	171		BIT  9 FLAG 11		XORFLBIT
# XOVINFLG	201		BIT  9 FLAG 13		XOVINHIB
# 3AXISFLG	084		BIT  6 FLAG  5		3AXISBIT
# 360SW		134		BIT  1 FLAG  8		360SWBIT

# ASSIGNMENT AND DESCRIPTION OF FLAGWORDS

; ============================================================================
; FLAGWORD 0: NAVIGATION, RADAR, AND RENDEZVOUS FLAGS
;
; This is the primary flagword controlling core navigation and rendezvous
; operations. It manages critical mission functions including:
;   - Sphere of influence transitions (Earth/Moon)
;   - Integration method selection for trajectory propagation
;   - Rendezvous program status (P20-P25 suite)
;   - Radar operation modes and lock-on control
;   - IMU usage coordination
;   - Display mode selection
;
; MISSION CONTEXT: During Apollo 11's rendezvous phase on July 21, 1969,
; after Eagle's ascent from the lunar surface, flags in this word coordinated
; the rendezvous radar operation and P20-P25 navigation programs that guided
; the LM back to Columbia in lunar orbit.
;
; FLAGORGY FLAGS IN THIS WORD:
;   - P25FLAG (BIT 9): Rendezvous program P25 operating status
;   - RNDVZFLG (BIT 7): P20 program running with radar in use
;
; These flags were set/cleared in THE_LUNAR_LANDING.agc's FLAGORGY routine
; to configure the navigation system for different mission phases.
; ============================================================================

FLAGWRD0	=	STATE +0		# (000-014)

						#  	(SET)			(RESET)

# BIT 15 FLAG 0	(S)
		=	000D
		=	BIT15

# BIT 14 FLAG 0	(S)
JSWITCH		=	001D			# 	INTEGRATION OF W	INTEGRATION OF STATE
JSWCHBIT	=	BIT14			#	MATRIX			VECTOR

# BIT 13 FLAG 0	(S)
MIDFLAG		=	002D			# 	INTEGRATION WITH	INTEGRATION WITHOUT
						# 	SECONDARY BODY AND	SOLAR PERTURBATIONS
MIDFLBIT	=	BIT13			# 	SOLAR PERTURBATIONS

# BIT 12 FLAG 0	(L)
MOONFLAG	=	003D			# 	MOON IS SPHERE OF	EARTH IS SPHERE OF
MOONBIT		=	BIT12			#	INFLUENCE		INFLUENCE

# BIT 11 FLAG 0
P21FLAG		=	004D			#	USE BASE VECTORS	1ST PASS -- CALC-
P21FLBIT	=	BIT11			#	ALREADY CALCULATED	ULATE BASE VECTORS

# BIT 10 FLAG 0
FSPASFLG	=	005D			#	FIRST PASS THROUGH	NOT FIRST PASS THRU
FSPASBIT	=	BIT10			#	REPOSITION ROUTINE	REPOSITION ROUTINE

# Page 66
; ----------------------------------------------------------------------------
; P25FLAG - Rendezvous Program P25 Status (Part of FLAGORGY flags)
;
; This flag indicates whether program P25 (Auto Optics Positioning for
; Rendezvous Navigation) is currently operating. P25 is part of the P20-P25
; rendezvous navigation suite that provides automated optical tracking of
; the Command Module during rendezvous operations.
;
; MISSION CONTEXT: During Apollo 11's rendezvous on July 21, 1969, this flag
; coordinated the automatic optics positioning system. When SET, it indicated
; P25 was actively controlling the Alignment Optical Telescope (AOT) to track
; Columbia for relative navigation state updates.
;
; PROGRAM COORDINATION:
;   SET: P25 is running - automatic optical tracking active
;   RESET: P25 not running - manual optics or radar navigation
;
; CROSS-REFERENCES:
;   - THE_LUNAR_LANDING.agc: Part of FLAGORGY initialization
;   - P20-P25.agc: Primary program setting/clearing this flag
;   - Rendezvous navigation suite coordination
;
; NOTE: This flag is part of the critical FLAGORGY set that manages
; operational mode transitions during powered flight phases.
; ----------------------------------------------------------------------------
# BIT 9 FLAG 0	(S)
P25FLAG		=	006D			# 	P25 OPERATING		P25 NOT OPERATING
P25FLBIT	=	BIT9

# BIT 8 FLAG 0	(S)
IMUSE		=	007D			# 	IMU IN USE		IMU NOT IN USE
IMUSEBIT	=	BIT8

; ----------------------------------------------------------------------------
; RNDVZFLG - Rendezvous Radar Active Status (Part of FLAGORGY flags)
;
; This flag indicates whether program P20 (Rendezvous Navigation) is actively
; running with rendezvous radar in use. P20 provides real-time relative
; navigation state updates during the rendezvous phase using radar tracking
; of the Command Module.
;
; MISSION CONTEXT: After Eagle's ascent from the lunar surface on July 21,
; 1969, this flag was SET to activate P20's radar-based rendezvous navigation.
; The rendezvous radar measured range, range rate, and angles to Columbia,
; providing the navigation data needed for Aldrin and Armstrong to guide
; Eagle back to dock with Collins in lunar orbit.
;
; PROGRAM COORDINATION:
;   SET: P20 running with rendezvous radar active and tracking CSM
;   RESET: P20 not running or operating without radar
;
; CROSS-REFERENCES:
;   - THE_LUNAR_LANDING.agc: Part of FLAGORGY initialization for mode control
;   - P20-P25.agc: Primary rendezvous navigation program using this flag
;   - RADAR_LEADIN_ROUTINES.agc: Radar interface respects this flag
;   - Ascent and rendezvous sequence programs
;
; HISTORICAL SIGNIFICANCE: This flag coordinated one of the most critical
; phases of Apollo 11 - the rendezvous that reunited the landing crew with
; the Command Module for the return journey to Earth.
; ----------------------------------------------------------------------------
# BIT 7 FLAG 0	(S)
RNDVZFLG	=	008D			#	P20 RUNNING (RADAR	P20 NOT RUNNING
RNDVZBIT	=	BIT7			# 	IN USE)

# BIT 6 FLAG 0	(S)
RRNBSW		=	009D			#	RADAR TARGET IN		RADAR TARGET IN
RRNBBIT		=	BIT6			# 	NB COORDINATES		SM COORDINATES

# BIT 5 FLAG 0	(S)
LOKONSW		=	010D			# 	RADAR LOCK-ON		RADAR LOCK-ON NOT
LOKONBIT	=	BIT5			# 	DESIRED			DESIRED

# BIT 4 FLAG 0	(S)
NEEDLFLG	=	011D			# 	TOTAL ATTITUDE		A/P FOLLOWING
NEEDLBIT	=	BIT4			# 	ERROR DISPLAYED		ERROR DISPLAYED

# BIT 3 FLAG 0
FREEFLAG	=	012D			# (USED BY P51-53 TEMP IN MANY DIFFERENT
						# ROUTINES & BY LUNAR + SOLAR EPHEMERIDES)
FREEFBIT	=	BIT3

# BIT 2 FLAG 0
R10FLAG		=	013D			# 	R10 OUTPUTS DATA TO 	BESIDES OUTPUT WHEN
R10FLBIT	=	BIT2			# 	ALTITUDE & ALTITUDE 	SET, R10 ALSO OUTPUT
						# 	RATE METERS ONLY	TO FORWARD & LATERAL
						#				VELOCITY CROSSPOINTER

# BIT 1 FLAG 0	(L)
OLDESFLG	=	014D			# 	R29 GYRO CMD LOOP	R29 GYRO CMD LOOP
OLDESBIT	=	BIT1			# 	REQUESTED		NOT REQUESTED

; ============================================================================
; FLAGWORD 1: RCS CONFIGURATION, DISPLAY INITIALIZATION, AND STATE UPDATES
;
; This flagword manages Reaction Control System (RCS) jet configuration,
; display system initialization sequences, navigation state vector update
; permissions, landing program operational modes, and numerical iteration
; control parameters.
;
; RCS CONFIGURATION:
;   - NJETSFLG: Selects two-jet vs four-jet RCS burn mode
;     Two-jet mode conserves propellant during low-thrust maneuvers
;     Four-jet mode provides higher control authority during critical phases
;
; DISPLAY SYSTEM INITIALIZATION:
;   - DIDFLAG: Indicates whether inertial data is available for display
;     SET: Data available, normal display operations proceed
;     RESET: Perform display data initialization functions first
;     Critical for preventing display of invalid navigation data
;
; NAVIGATION STATE UPDATE CONTROL:
;   Three-flag system manages update permissions during mission operations:
;   - VEHUPFLG: Designates which vehicle state vector is being updated
;     SET: CSM (Command Module) state vector update in progress
;     RESET: LM (Lunar Module) state vector update in progress
;   - UPDATFLG: Controls whether optical mark updates are permitted
;     Used by P20-P25 rendezvous navigation and P51-P53 alignment
;   - NOUPFLAG: Master enable/disable for all state vector updates
;     SET: Neither CSM nor LM state vector may be updated (freeze state)
;     RESET: Updates allowed (controlled by VEHUPFLG and UPDATFLG)
;   - TRACKFLG: Controls whether optical tracking operations are allowed
;
; LANDING PROGRAM MODE (P66):
;   - RODFLAG: Controls P66 (LM final approach rate-of-descent) behavior
;     SET: Normal P66 operation continues through restart
;     RESET: P66 reinitialization performed (restart clears flag)
;     P66 allowed Armstrong manual rate-of-descent control during final
;     approach, providing capability to slow descent if needed
;
; NUMERICAL ITERATION CONTROL:
;   Lambert targeting and orbital mechanics routines use iterative solvers:
;   - SLOPESW: Selects iteration algorithm
;     SET: Use bias method in iterator
;     RESET: Use regular falsi (false position) method
;   - GUESSW: Indicates whether starting value exists for iteration
;     SET: No starting value available, algorithm must generate initial guess
;     RESET: Starting value for iteration exists, convergence faster
;
; EARTH RADIUS COMPUTATION:
;   - ERADFLAG: Selects Earth radius calculation method
;     SET: Compute REARTH using Fischer ellipsoid model (latitude-dependent)
;     RESET: Use constant REARTH from pad-loaded value (faster, less accurate)
;     Affects entry targeting accuracy and transearth navigation precision
;
; DISPLAY/LANDING PROGRAM COORDINATION:
;   - R61FLAG: Selects landing radar data display routine
;     SET: Run R61 (standard LEM landing radar display)
;     RESET: Run R65 (alternate LEM display mode)
;
; CROSS-REFERENCES:
;   - RCS-CSM_DIGITAL_AUTOPILOT.agc: Uses NJETSFLG for jet selection
;   - DISPLAY_INTERFACE_ROUTINES.agc: Checks DIDFLAG before display updates
;   - P20-P25.agc: Controls UPDATFLG and TRACKFLG during navigation
;   - P66.agc: Uses RODFLAG for landing mode control (if present in codebase)
;   - INTEGRATION_INITIALIZATION.agc: Uses numerical iteration flags
; ============================================================================

FLAGWRD1	=	STATE +1		# (015-029)

# Page 67
						#	(SET)			(RESET)

# BIT 15 FLAG 1	(S)
NJETSFLG	=	015D			#	TWO JET RCS BURN	FOUR JET RCS BURN
NJETSBIT	=	BIT15

# BIT 14 FLAG 1	(L)
DIDFLAG		= 	016D			#	INERTIAL DATA IS	PERFORM DATA DISPLAY
DIDFLBIT	=	BIT14			#	AVAILABLE		INITIALIZATION FUNCS

# BIT 13 FLAG 1	(S)
ERADFLAG	=	017D			#	COMPUTE REARTH		USE CONSTANT REARTH
ERADFBIT	=	BIT13			#	FISCHER ELLIPSOID	PAD RADIUS

# BIT 12 FLAG 1
RODFLAG		=	018D			#	IF IN P66, NORMAL	IF IN P66, RE-INIT-
RODFLBIT	=	BIT12			#	OPERATION CONTINUES.	IALIZATION IS PER-
						#	RESTART CLEARS FLAG	FORMED AND FLAG IS

# BIT 11 FLAG 1
		=	019D
		=	BIT11

# BIT 10 FLAG 1	(L)
R61FLAG		=	020D			#	RUN R61 LEM		RUN R65 LEM
R61FLBIT	=	BIT10

# BIT 9 FLAG 1
		=	021D
		=	BIT9

# BIT 8 FLAG 1	(S)
VEHUPFLG	=	022D			#	CSM STATE-VECTOR	LEM STATE VECTOR
VEHUPBIT	=	BIT8			#	BEING UPDATED		BEING UPDATED

# BIT 7 FLAG 1	(S)
UPDATFLG	=	023D			#	UPDATING BY MARKS	UPDATING BY MARKS
UPDATBIT	=	BIT7			#	ALLOWED			NOT ALLOWED

# BIT 6 FLAG 1	(S)
NOUPFLAG	=	024D			#	NEITHER CSM		EITHER STATE
						#	NOR LM STATE VECTOR	VECTOR MAY BE
NOUPFBIT	=	BIT6			#	MAY BE UPDATED		UPDATED

# Page 68
# BIT 5 FLAG 1	(S)
TRACKFLG	=	025D			#	TRACKING ALLOWED	TRACKING NOT ALLOWED
TRACKBIT	=	BIT5

# BIT 4 FLAG 1
		=	026D
		=	BIT4

# BIT 3 FLAG 1	(S)
SLOPESW		=	027D			#	ITERATE WITH BIAS	ITERATE WITH REGULAR
						#	METHOD IN ITERATOR	FALSI METHOD IN
SLOPEBIT	=	BIT3			#				ITERATOR

# BIT 2 FLAG 1	(S)
GUESSW		=	028D			#	NO STARTING VALUE	STARTING VALUE FOR
GUESSBIT	=	BIT2			#	FOR ITERATION		ITERATION EXISTS

# BIT 1 FLAG 1
		=	029D
		=	BIT1			# OH 2009-05-15 Scan does not have this line

; ============================================================================
; FLAGWORD 2: RENDEZVOUS AND NAVIGATION CONTROL FLAGS
;
; This flagword manages orbital rendezvous operations, navigation mode
; selection, and guidance computation options used primarily by programs
; P20-P25 (rendezvous navigation) and P30-P37 (maneuver targeting).
;
; RENDEZVOUS FLAGS:
;   - Radar search and acquisition modes
;   - Line of sight computation control
;   - Active vehicle designation (LM vs CSM)
;   - Final pass vs interim computation selection
;
; GUIDANCE AND STEERING FLAGS:
;   - Thrust availability monitoring
;   - Minimum impulse vs continuous burn modes
;   - External delta-V vs Lambert targeting selection
;   - Preferred attitude computation status
;
; MISSION CONTEXT:
; During Apollo 11's rendezvous on July 21, 1969, these flags coordinated the
; LM's ascent targeting and subsequent rendezvous with Columbia. They enabled
; the guidance computer to select appropriate computation modes and configure
; the rendezvous radar for tracking the Command Module during approach.
;
; CROSS-REFERENCES:
;   - P20-P25.agc: Rendezvous navigation using these mode flags
;   - P30-P37.agc: External delta-V programs using guidance flags
;   - RADAR_LEADIN_ROUTINES.agc: Responds to radar mode flag settings
; ============================================================================

FLAGWRD2	=	STATE +2		# (030-044)

						#	(SET)			(RESET)

# BIT 15 FLAG 2	(S)
DRIFTFLG	=	030D			#	T3RUPT CALLS GYRO	T3RUPT DOES NO GYRO
DRFTBIT		=	BIT15			#	COMPENSATION		COMPENSATION

# BIT 14 FLAG 2	(S)
SRCHOPTN	=	031D			#	RADAR IN AUTOMATIC	RADAR NOT IN AUTO-
SRCHOBIT	=	BIT14			#	SEARCH OPTION (R24)	MATIC SEARCH OPTION

# BIT 13 FLAG 2	(S)
ACMODFLG	=	032D			#	MANUAL ACQUISITION	AUTO ACQUISITION
ACMODBIT	=	BIT13			#	BY RENDEZVOUS RADAR	BY RENDEZVOUS RADAR

# BIT 12 FLAG 2 (S)
LOSCMFLG	=	033D			#	LINE OF SIGHT BEING	LINE OF SIGHT NOT
						#	COMPUTED (R21)		BEING COMPUTED
LOSCMBIT	=	BIT12

# Page 69
# BIT 11 FLAG 2 (S)
STEERSW		=	034D			#	SUFFICIENT THRUST	INSUFFICIENT THRUST
STEERBIT	=	BIT11			#	IS PRESENT		IS PRESENT

# BIT 10 FLAG 2 (S)
		=	035D			# OH 2009-05-15 These two line don't appear in scan
		=	BIT10

# BIT 9 FLAG 2 (S)
IMPULSW		=	036D			#	MINIMUM IMPULSE		STEERING BURN (NO
						#	BURN (CUTOFF TIME	CUTOFF TIME YET
IMPULBIT	=	BIT9			#	SPECIFIED)		AVAILABLE)

# BIT 8 FLAG 2 (S)
XDELVFLG	=	037D			#	EXTERNAL DELTAV VG	LAMBERT (AIMPOINT)
XDELVBIT	=	BIT8			#	COMPUTATION		VG COMPUTATION

# BIT 7 FLAG 2 (S)
ETPIFLAG	=	038D			#	ELEVATION ANGLE		TPI TIME SUPPLIED
						#	SUPPLIED FOR		FOR P34,74 TO COMPUTE
ETPIBIT		=	BIT7			#	P34,74			ELEVATION

# BIT 7 FLAG 2 (L)
OPTNSW		=	ETPIFLAG		#	SOI PHASE OF P38/78	SOR PHASE OF P38/78
OPTNBIT		=	BIT7

# BIT 6 FLAG 2 (S)
FINALFLG	=	039D			#	LAST PASS THROUGH	INTERIM PASS THROUGH
						#	RENDEZVOUS PROGRAM	RENDEZVOUS PROGRAM
FINALBIT	=	BIT6			#	COMPUTATIONS		COMPUTATIONS

# BIT 5 FLAG 2 (S)
AVFLAG		=	040D			#	LEM IS ACTIVE		CSM IS ACTIVE
AVFLBIT		=	BIT5			#	VEHICLE			VEHICLE

# BIT 4 FLAG 2 (S)
PFRATFLG	=	041D			#	PREFERRED ATTITUDE	PREFERRED ATTITUDE
PFRATBIT	=	BIT4			#	COMPUTED		NOT COMPUTED

# BIT 3 FLAG 2 (S)

# Page 70
CALCMAN3	=	042D			#	NO FINAL ROLL		FINAL ROLL IS
CALC3BIT	=	BIT3			#				NECESSARY

# BIT 2 FLAG 2 (S)
CALCMAN2	=	043D			#	PERFORM MANEUVER	BYPASS STARTING
CALC2BIT	=	BIT2			#	STARTING PROCEDURE	PROCEDURE

# BIT 1 FLAG 2 (S)
NODOFLAG	=	044D			#	V37 NOT PERMITTED	V37 PERMITTED
NODOBIT		=	BIT1

; ----------------------------------------------------------------------------
; FLAGWRD3 (Bits 045-059): NAVIGATION MODES AND COORDINATE SYSTEMS
;
; This flagword controls navigation state vector management, coordinate frame
; selection, and orbital integration parameters.
;
; COORDINATE SYSTEM FLAGS (REFSMFLG, LUNAFLAG):
; - REFSMFLG: Reference Stable Member Matrix validity (*** PROTECTED FROM
;   FRESH START *** - survives system resets)
; - LUNAFLAG: Selects lunar vs. Earth latitude-longitude coordinate systems
; - Used throughout PLANETARY_INERTIAL_ORIENTATION.agc for frame transforms
;
; INTEGRATION MODE FLAGS (INTYPFLG, VINTFLAG, D6OR9FLG, DIM0FLAG, PRECIFLG):
; - Control Encke vs. conic integration methods in ORBITAL_INTEGRATION.agc
; - Select CSM vs. LM state vector for propagation
; - Control W-matrix dimensionality (6x6 vs. 9x9) for covariance propagation
; - Engage precision integration modes when required
;
; STAR TRACKING FLAGS (VFLAG, CULTFLAG):
; - Track number of stars in optical field of view for alignment
; - Detect star occultation conditions during navigation sightings
; - Interface with IMU alignment routines (P51-P53.agc)
;
; NAVIGATION STATE FLAGS (ORBWFLAG, STATEFLG):
; - Indicate validity of W-matrix for orbital navigation
; - Track permanent state vector update status
; - Coordinate MEASUREMENT_INCORPORATION.agc with navigation updates
;
; GIMBAL LOCK PROTECTION (GLOKFAIL):
; - Indicates gimbal lock has occurred (middle gimbal near ±90°)
; - Triggers crew warning on DSKY (GIMBAL LOCK light)
; - Prevents IMU operations during lock condition
;
; RENDEZVOUS RADAR FLAGS (NOR29FLG, READRFLG):
; - Control R29 rendezvous radar data reading during powered flight
; - Coordinate radar tracking with P20-P25.agc rendezvous programs
; ----------------------------------------------------------------------------

FLAGWRD3	=	STATE +3		# (045-059)

						#	(SET)			(RESET)

# BIT 15 FLAG 3
		=	045D			#
		=	BIT15			# OH 2009-05-15 This line is not in scans

# BIT 14 FLAG 3	(S)
GLOKFAIL	=	046D			#	GIMBAL LOCK HAS		NOT IN GIMBAL LOCK
GLOKFBIT	=	BIT14			#	OCCURRED

# BIT 13 FLAG 3	*** PROTECTED FROM FRESH START ***
REFSMFLG	=	047D			#	REFSMMAT GOOD		REFSMMAT NO GOOD
REFSMBIT	=	BIT13

# BIT 12 FLAG 3	(S)
LUNAFLAG	=	048D			#	LUNAR LAT-LONG		EARTH LAT-LONG
LUNABIT		=	BIT12

# BIT 11 FLAG 3	(L)
NOR29FLG	=	049D			#	R29 NOT ALLOWED		R29 ALLOWED (RR DES-
NR29FBIT	=	BIT11			#				IGNATED POWERED FLT)

# BIT 10 FLAG 3	(S)
VFLAG		=	050D			#	LESS THAN TWO STARS	TWO STARS IN FIELD
VFLAGBIT	=	BIT10			#	IN FIELD OF VIEW	OF VIEW

# BIT 9 FLAG 3	(S)
R04FLAG		=	051D			#	ALARM 521		ALARM 521 ALLOWED
						#	SUPPRESSED
# Page 71
R04FLBIT	=	BIT9

# BIT 9 FLAG 3	(L)
READRFLG	=	R04FLAG			#	READING RR DATA		NOT READING RR DATA
READRBIT	=	BIT9			#	PURSUANT TO R29		PURSUANT TO R29

# BIT 8 FLAG 3	(S)
PRECIFLG	=	052D			#	NORMAL INTEGRATION	ENGAGES 4-TIME STEP
						#	IN P00			(P00) LOGIC IN INTE-
PRECIBIT	=	BIT8			#				GRATION

# BIT 7 FLAG 3	(S)
CULTFLAG	=	053D			#	STAR OCCULTED		STAR NOT OCCULTED
CULTBIT		=	BIT7

# BIT 6 FLAG 3	(S)
ORBWFLAG	=	054D			#	W MATRIX VALID FOR	W MATRIX INVALID FOR
ORBWFBIT	=	BIT6			#	ORBITAL NAVIGATION	ORBITAL NAVIGATION

# BIT 5 FLAG 3	(S)
STATEFLG	=	055D			#	PERMANENT STATE		PERMANENT STATE
STATEBIT	=	BIT5			#	VECTOR UPDATED		VECTOR NOT UPDATED

# BIT 4 FLAG 3	(S)
INTYPFLG	=	056D			#	CONIC INTEGRATION	ENCKE INTEGRATION
INTYPBIT	=	BIT4

# BIT 3 FLAG 3	(S)
VINTFLAG	=	057D			#	CSM STATE VECTOR	LEM STATE VECTOR
VINTFBIT	=	BIT3			#	BEING INTEGRATED	BEING INTEGRATED

# BIT 2 FLAG 3 (S)
D6OR9FLG	= 	058D			#	DIMENSION OF W IS 9	DIMENSION OF W IS 6
D6OR9BIT	=	BIT2			#	FOR INTEGRATION		FOR INTEGRATION

# BIT 1 FLAG 3	(S)
DIM0FLAG	=	059D			#	W MATRIX IS TO BE	W MATRIX IS NOT TO
DIM0BIT		=	BIT1			#	USED			USED

; ----------------------------------------------------------------------------
; FLAGWRD4 (Bits 060-074): DSKY DISPLAY MANAGEMENT AND KEYBOARD ARBITRATION
;
; This flagword implements the display priority system managing the single
; DSKY (Display and Keyboard) unit shared by all LM programs. Controls three
; display priority levels and keyboard access arbitration.
;
; DISPLAY PRIORITY LEVELS:
; Three concurrent display types compete for the single DSKY screen:
;
; 1. PRIORITY DISPLAYS (highest):
;    - Critical mission data requiring immediate crew attention
;    - Can interrupt mark and normal displays
;    - PRIODFLG: Priority display active in ENDIDLE loop
;    - Examples: Program alarms, critical navigation updates
;
; 2. MARK DISPLAYS (medium):
;    - Optical navigation mark data during star/landmark sightings
;    - Can interrupt normal displays but not priority displays
;    - MRKIDFLG: Mark display active in ENDIDLE loop
;    - Used during IMU alignment (P51-P53.agc) and optical tracking
;
; 3. NORMAL DISPLAYS (lowest):
;    - Standard program data displays
;    - Can be interrupted by priority or mark displays
;    - NRMIDFLG: Normal display active in ENDIDLE loop
;    - Most mission programs display data at this level
;
; DISPLAY INTERRUPT COORDINATION:
; Manages what happens when higher-priority display preempts current display:
; - MRUPTFLG: Mark display interrupted by priority display
; - NRUPTFLG: Normal display interrupted by priority or mark
; - MKOVFLAG: Mark display currently over normal display
; - Enables display restoration after higher-priority display completes
;
; KEYBOARD ARBITRATION FLAGS:
; Controls crew keyboard access when multiple programs request DSKY input:
; - MRKNVFLG: Astronaut using keyboard during mark display initiation
; - NRMNVFLG: Astronaut using keyboard during normal display initiation
; - PRONVFLG: Astronaut using keyboard during priority display initiation
; - PINBRFLG: Astronaut has interfered with existing display by keying input
; - Prevents display corruption from simultaneous keyboard operations
;
; SPECIAL DISPLAY CONTROL FLAGS:
; - PDSPFLAG: P20 (rendezvous nav) converts normal display to priority in R60
; - MWAITFLG/NWAITFLG: Higher-priority display operating when mark/normal
;   display initiated (defers lower-priority display until completion)
; - XDSPFLAG: Mark display protected from interruption (special mark info mode)
;
; ARCHITECTURE NOTE:
; This priority system prevents display "thrashing" on the single DSKY unit
; while ensuring critical information reaches crew during mission-critical
; phases. The ENDIDLE loop (DISPLAY_INTERFACE_ROUTINES.agc) implements the
; arbitration logic using these flags.
; ----------------------------------------------------------------------------

FLAGWRD4	=	STATE +4		# (060-074)

# Page 72
						#	(SET)			(RESET)

# BIT 15 FLAG 4	(S)
MRKIDFLG	=	060D			#	MARK DISPLAY IN 	NO MARK DISPLAY IN
MRKIDBIT	=	BIT15			#	ENDIDLE			ENDIDLE

# BIT 14 FLAG 4	(S)
PRIODFLG	=	061D			#	PRIORITY DISPLAY IN	NO PRIORITY DISPLAY
PRIODBIT	=	BIT14			#	ENDIDLE			IN ENDIDLE

# BIT 13 FLAG 4	(S)
NRMIDFLG	=	062D			#	NORMAL DISPLAY IN	NO NORMAL DISPLAY
NRMIDBIT	=	BIT13			#	ENDIDLE			IN ENDIDLE

# BIT 12 FLAG 4 (S)
PDSPFLAG	=	063D			#	P20 SETS SO AS TO	LEAVE AS NORMAL DISP
						#	TURN A NORMAL DIS-
PDSPFBIT	=	BIT12			#	PLAY INTO A PRIORITY
						#	DISPLAY IN R60

# BIT 11 FLAG 4 (S)
MWAITFLG	=	064D			#	HIGHER PRIORITY		NO HIGHER PRIORITY
						#	DISPLAY OPERATING	DISPLAY OPERATING
MWAITBIT	=	BIT11			#	WHEN MARK		WHEN MARK DISPLAY
						#	DISPLAY INITIATED	INITIATED

# BIT 10 FLAG 4 (S)
NWAITFLG	=	065D			#	HIGHER PRIORITY		NO HIGHER PRIORITY
						#	DISPLAY OPERATING	DISPLAY OPERATING
NWAITBIT	=	BIT10			#	WHEN NORMAL		WHEN NORMAL DISPLAY
						#	DISPLAY INITIATED	INITIATED

# BIT 9 FLAG 4	(S)
MRKNVFLG	=	066D			#	ASTRONAUT USING		ASTRONAUT NOT USING
						#	KEYBOARD WHEN MARK	KEYBOARD WHEN MARK
MRKNVBIT	=	BIT9			#	DISPLAY INITIATED	DISPLAY INITIATED

# BIT 8 FLAG 4	(S)
NRMNVFLG	=	067D			#	ASTRONAUT USING		ASTRONAUT NOT USING
						#	KEYBOARD WHEN		KEYBOARD WHEN
NRMNVBIT	=	BIT8			#	NORMAL DISPLAY		NORMAL DISPLAY
						#	INITIATED		INITIATED

# BIT 7 FLAG 4	(S)
PRONVFLG	=	068D			#	ASTRONAUT USING		ASTRONAUT NOT USING

# Page 73
						#	KEYBOARD WHEN		KEYBOARD WHEN
PRONVBIT	=	BIT7			#	PRIORITY DISPLAY	PRIORITY DISPLAY
						#	INITIATED		INITIATED

# BIT 6 FLAG 4	(S)
PINBRFLG	=	069D			#	ASTRONAUT HAS		ASTRONAUT HAS NOT
						#	INTERFERED WITH		INTERFERED WITH
PINBRBIT	=	BIT6			#	EXISTING DISPLAY	EXISTING DISPLAY

# BIT 5 FLAG 4	(S)
MRUPTFLG	=	070D			#	MARK DISPLAY		MARK DISPLAY NOT
						#	INTERRUPTED BY		INTERRUPTED BY
MRUPTBIT	=	BIT5			#	PRIORITY DISPLAY	PRIORITY DISPLAY

# BIT 4 FLAG 4	(S)
NRUPTFLG	=	071D			#	NORMAL DISPLAY		NORMAL DISPLAY NOT
						#	INTERRUPTED BY		INTERRUPTED BY
NRUPTBIT	=	BIT4			#	PRIORITY OR MARK	PRIORITY OR MARK
						#	DISPLAY			DISPLAY

# BIT 3 FLAG 4	(S)
MKOVFLAG	=	072D			#	MARK DISPLAY OVER	NO MARK DISPLAY OVER
MKOVBIT		=	BIT3			#	NORMAL			NORMAL

# BIT 2 FLAG 4
		=	073D
		=	BIT2			# OH 2009-05-15 Not in scan.


# BIT 1 FLAG 4	(S)
XDSPFLAG	=	074D			#	MARK DISPLAY NOT	NO SPECIAL MARK
XDSPBIT		=	BIT1			#	TO BE INTERRUPTED	INFORMATION

; ============================================================================
; FLAGWORD 5: ENGINE CONTROL AND MANEUVER CONFIGURATION FLAGS
;
; This flagword manages engine throttle control, jet selection, attitude
; maneuver configuration, and radar monitoring modes. Several flags directly
; control safety-critical operations during powered flight phases.
;
; ENGINE AND THROTTLE CONTROL FLAGS:
;   - NOTHROTL: Inhibits full throttle on descent engine (prevents overburn)
;   - ENGONFLG: Tracks engine on/off state
;   - SNUFFER: Disables U,V jets during descent propulsion system (DPS) burns
;
; MANEUVER CONFIGURATION FLAGS:
;   - 3AXISFLG: Selects three-axis vs single-axis attitude maneuvers
;   - MGLVFLAG: Controls coordinate system selection (local vertical vs gimbal)
;
; RADAR MONITORING FLAGS:
;   - R77FLAG: Suppresses radar alarms during R77 diagnostic routine
;   - NORRMON: Bypasses rendezvous radar gimbal monitoring
;   - RNGSCFLG: Tracks scale changes during RR readings
;
; NAVIGATION AND GUIDANCE FLAGS:
;   - RENDWFLG: Indicates W matrix validity for rendezvous navigation
;   - SOLNSW: Lambert targeting convergence status
;   - DMENFLG: Controls measurement vector dimension (6 or 9 elements)
;
; DISPLAY FLAGS:
;   - DSKYFLAG: Controls DSKY display output
;
; MISSION CONTEXT:
; During Apollo 11's descent on July 20, 1969, NOTHROTL prevented the descent
; engine from achieving full throttle during critical low-altitude phases where
; controlled deceleration was required. This flag was part of the throttle
; management system that enabled Armstrong's manual landing site selection.
;
; CROSS-REFERENCES:
;   - THROTTLE_CONTROL_ROUTINES.agc: Uses NOTHROTL flag
;   - THE_LUNAR_LANDING.agc: Sets NOTHROTL during descent phases
;   - TJET_LAW.agc: Responds to SNUFFER for jet selection
;   - ATTITUDE_MANEUVER_ROUTINE.agc: Uses 3AXISFLG configuration
; ============================================================================

FLAGWRD5	=	STATE +5		# (075-089)

						#	(SET)			(RESET)

# BIT 15 FLAG 5	(S)
DSKYFLAG	=	075D			#	DISPLAYS SENT TO	NO DISPLAYS TO DSKY
DSKYFBIT	=	BIT15			#	DSKY

# BIT 14 FLAG 5
		=	076D
		=	BIT14

# Page 74
# BIT 13 FLAG 5	(S,L)
SNUFFER		=	077D			#	U,V JETS DISABLED	U,V JETS ENABLED
						#	DURING DPS		DURING DPS
SNUFFBIT	=	BIT13			#	BURNS (V65)		BURNS (V75)

# BIT 12 FLAG 5	(S)
NOTHROTL	=	078D			#	INHIBIT FULL		PERMIT FULL THROTTLE
NOTHRBIT	=	BIT12			#	THROTTLE

# BIT 11 FLAG 5	(S,L)
R77FLAG		=	079D			#	R77 IS ON,		R77 IS NOT ON.
						#	SUPPRESS ALL RADAR
						#	ALARMS AND TRACKER
R77FLBIT	=	BIT11			#	FAILS

# BIT 10 FLAG 5	(S)
RNGSCFLG	=	080D			#	SCALE CHANGE HAS	NO SCALE CHANGE HAS
						#	OCCURRED DURING		OCCURRED DURING
RNGSCBIT	=	BIT10			#	RR READING		RR READING

# BIT 9 FLAG 5	(S)
DMENFLG		=	081D			#	DIMENSION OF W IS 9	DIMENSION OF W IS 6
DMENFBIT	=	BIT9			#	FOR INCORPORATION	FOR INCORPORATION

# BIT 8 FLAG 5	(S)
		=	082D
		=	BIT8

# BIT 7 FLAG 5	(S)
ENGONFLG	=	083D			#	ENGINE TURNED ON	ENGINE TURNED OFF
ENGONBIT	=	BIT7			#

# BIT 6 FLAG 5	(S)
3AXISFLG	=	084D			#	MANEUVER SPECIFIED	MANEUVER SPECIFIED
						#	BY THREE AXES		BY ONE AXIS; R60
3AXISBIT	=	BIT6			#				CALLS VECPOINT.

# BIT 5 FLAG 5
		=	085D
		=	BIT5			# OH 2009-05-15 Not in scan

# BIT 4 FLAG 5	(S)

# Page 75
NORRMON		=	086D			#	BYPASS RR GIMBAL	PERFORM
NORRMBIT	=	BIT4			#	MONITOR			RR GIMBAL MONITOR

# BIT 3 FLAG 5	(S)
SOLNSW		=	087D			#	LAMBERT DOES NOT	LAMBERT CONVERGES OR
						#	CONVERGE, OR TIME-RAD	TIME-RADIUS NON-
SOLNSBIT	=	BIT3			#	NEARLY CIRCULAR		CIRCULAR

# BIT 2 FLAG 5	(S)
MGLVFLAG	=	088D			#	LOCAL VERTICAL		MIDDLE GIMBAL ANGLE
						#	COORDINATES		COMPUTED
MGLVFBIT	=	BIT2			#	COMPUTED

# BIT 1 FLAG 5	(S)
RENDWFLG	=	089D			#	W MATRIX VALID		W MATRIX INVALID
						#	FOR RENDEZVOUS		FOR RENDEZVOUS
RENDWBIT	=	BIT1			#	NAVIGATION		NAVIGATION


; ============================================================================
; FLAGWORD 6: LANDING REDESIGNATION AND TARGETING FLAGS
;
; This flagword contains critical flags controlling lunar landing site
; redesignation, targeting operations, and coordinate system states. Several
; flags here were part of the FLAGORGY subroutine in THE_LUNAR_LANDING.agc
; that configured the landing sequence.
;
; LANDING MODE FLAGS (FLAGORGY group):
;   - MUNFLAG: Selects whether servicer calls MUNRVG (set) or CALCRVG (reset)
;              for navigation state vector computation during descent
;   - REDFLAG: Enables/inhibits landing site redesignation by crew
;              When SET, astronaut can use manual redesignation controls
;              When RESET, landing site is locked to computed target
;
; TARGETING AND MANEUVER FLAGS:
;   - NTARGFLG: Indicates astronaut manually overwrote delta-velocity at
;               Terminal Phase Initiation (TPI) or Midcourse (TPM) in P34/P35
;   - S32.1F1: Delta-V at Coelliptic Sequence Initiation (CSI) time exceeds
;              maximum allowable value (requires retargeting)
;   - S32.1F2/S32.1F3A/S32.1F3B: Track Newton iteration stages in S32.1
;              targeting computations (ordered pair logic for convergence)
;
; GIMBAL CONTROL FLAGS:
;   - GMBDRVSW: Indicates TRIMGIMB (trim gimbal routine) has completed
;
; COORDINATE SYSTEM FLAGS:
;   - ATTFLAG: Indicates LM attitude is available in Moon-fixed coordinates
;              Essential for landing radar data processing and touchdown
;
; SERVICER FLAGS:
;   - AUXFLAG: Controls DVMON (delta-velocity monitor) execution in servicer
;              background tasks based on IDLEFLAG state
;
; MISSION CONTEXT:
; During Apollo 11's descent, REDFLAG was SET to permit Armstrong to manually
; redesignate the landing site when he observed the computer was guiding Eagle
; toward a boulder field. At approximately 500 feet altitude, Armstrong used
; the Attitude Controller Assembly (ACA) to slew the landing point downrange,
; extending the powered descent time and reducing fuel margins to ~25 seconds
; at touchdown. This manual redesignation capability, controlled by REDFLAG,
; was essential to mission success.
;
; MUNFLAG controlled the navigation computation method during descent, selecting
; between the MUNRVG (Munakata algorithm) and CALCRVG (calculated R and V)
; routines for state vector propagation.
;
; CROSS-REFERENCES:
;   - THE_LUNAR_LANDING.agc: Sets MUNFLAG and REDFLAG in FLAGORGY subroutine
;   - LUNAR_LANDING_GUIDANCE_EQUATIONS.agc: Uses REDFLAG for redesignation
;   - SERVICER.agc: Uses MUNFLAG to select navigation computation method
;   - ATTITUDE_MANEUVER_ROUTINE.agc: Uses ATTFLAG for coordinate validation
;   - P32-P35_P72-P75.agc: Uses S32 iteration flags for targeting convergence
;   - P34-35_P74-75.agc: Uses NTARGFLG for manual override tracking
; ============================================================================

FLAGWRD6	=	STATE +6		# (090-104)

						#	(SET)			(RESET)

# BIT 15 FLAG 6	(S)
S32.1F1		=	090D			#	DELTA V AT CSI TIME	DVT1 LESS THAN MAX
S32BIT1		=	BIT15			#	ONE EXCEEDS MAX

# BIT 14 FLAG 6	(S)
S32.1F2		=	091D			#	FIRST PASS OF		REITERATION OF
S32BIT2		=	BIT14			#	NEWTON ITERATION	NEWTON

# BIT 13 FLAG 6	(S)
S32.1F3A	=	092D			# BIT 13 AND BIT 12 FUNCTION AS AN ORDERED
S32BIT3A	=	BIT13			# PAIR (13,12) INDICATING THE POSSIBLE OC-
						# CURRENCE OF 2 NEWTON ITERATIONS FOR S32.1
						# IN THE PROGRAM IN THE FOLLOWING ORDER:
# BIT 12 FLAG 6	(S)				# (0,1) (I.E. BIT 13 RESET, BIT 12 SET)
S32.1F3B	=	093D			#      = FIRST NEWTON ITERATION BEING DONE
S32BIT3B	=	BIT12			# (0,0)= FIRST PASS OF SECOND NEWTON ITERATION
						# (1,1)= 50 FT/SEC STAGE OF SECOND NEWTON ITERATION
						# (1,0)= REMAINDER OF SECOND NEWTON ITERATION
# BIT 11 FLAG 6	(S)
		=	094D			#
		=	BIT11			#
# Page 76
# BIT 10 FLAG 6	(S)
GMBDRVSW	=	095D			#	TRIMGIMB OVER		TRIMGIMB NOT OVER
GMBDRBIT	=	BIT10			#

# BIT 9 FLAG 6
		=	096D			#
		=	BIT9			#

# BIT 8 FLAG 6	(S)
MUNFLAG		=	097D			#	SERVICER CALLS		SERVICER CALLS
MUNFLBIT	=	BIT8			#	MUNRVG			CALCRVG

# BIT 7 FLAG 6	(L)
		=	098D			#
		=	BIT7			#

# BIT 6 FLAG 6	(L)
REDFLAG		=	099D			#	LANDING SITE		LANDING SITE
						#	REDESIGNATION		REDESIGNATION NOT
REDFLBIT	=	BIT6			#	PERMITTED		PERMITTED

# BIT 5 FLAG 6
		=	100D			#
		=	BIT5			# OH 2009-05-15 Not in scan

# BIT 4 FLAG 6
		=	101D			#
		=	BIT4			# OH 2009-05-15 Not in scan

# BIT 3 FLAG 6	(S)
NTARGFLG	=	102D			#	ASTRONAUT DID		ASTRONAUT DID NOT
						#	OVERWRITE DELTA		OVERWRITE DELTA
NTARGBIT	=	BIT3			#	VELOCITY AT TPI		VELOCITY
						#	OR TPM (P34,35)

# BIT 2 FLAG 6
AUXFLAG		=	103D			#	PROVIDING IDLEFLAG	SERVICER WILL SKIP
AUXFLBIT	=	BIT2			#	IS NOT SET, SERV-	DVMON ON ITS NEXT
						#	ICER WILL EXERCISE	PASS EVEN IF THE
						#	DVMON ON ITS NEXT	IDLEFLAG IS NOT SET.
						#	PASS.			IT WILL THEN SET
						#				AUXFLAG.

# BIT 1 FLAG 6	(L)
ATTFLAG		=	104D			#	LEM ATTITUDE EXISTS	NO LEM ATTITUDE
						#	IN MOON-FIXED		AVAILABLE IN MOON-

# Page 77
ATTFLBIT	=	BIT1			#	COORDINATES		FIXED COORDINATES

; ============================================================================
; FLAGWORD 7: IGNITION CONTROL, DISPLAYS, AND SERVICER FLAGS
;
; This flagword contains critical flags controlling powered flight ignition
; sequencing, crew displays during landing, servicer background task control,
; and targeting computation modes. Several flags coordinate between mission
; programs and the servicer routine for navigation state monitoring.
;
; IGNITION AND BURN CONTROL FLAGS:
;   - IGNFLAG: Indicates Time of Ignition (TIG) has arrived for powered burn
;              Controls transition from coast to powered flight in burn programs
;   - ASTNFLAG: Indicates astronaut has manually approved engine ignition
;               Safety interlock requiring crew GO before automatic ignition
;   - ITSWICH: Controls whether R34 should compute TPI (Terminal Phase Initiation)
;              time or if TPI has already been computed and stored
;
; LANDING DISPLAY FLAGS:
;   - SWANDISP: Enables/disables landing analog displays (altitude rate,
;               horizontal velocity indicators on crew panel during descent)
;               SET during P63/P64 landing programs to provide crew situational
;               awareness. Critical for Armstrong's monitoring during Apollo 11.
;
; SERVICER AND MONITOR FLAGS:
;   - IDLEFLAG: Controls DV (delta-velocity) monitor execution in servicer
;               When RESET, servicer runs DVMON to track accumulated velocity
;               changes from RCS firings and verify against planned maneuvers
;   - AVEGFLAG: Indicates AVERAGEG (average acceleration) computation desired
;               Used by servicer to integrate IMU acceleration data
;   - V37FLAG: Indicates AVERAGEG computation currently running in servicer
;              State tracking flag for servicer background task execution
;
; TARGETING COMPUTATION FLAGS:
;   - NORMSW: Controls whether Lambert targeting routine computes its own
;             unit normal vector or uses externally-provided normal as input
;   - RVSW: Selects final state vector computation mode in time-theta targeting
;           Determines whether to compute state at time-delta or time-theta
;   - MANUFLAG: Indicates attitude maneuver in progress during Rendezvous Radar
;               (RR) search operations. Coordinates autopilot with radar tracking.
;
; W-MATRIX AND NAVIGATION FLAGS:
;   - V67FLAG: Indicates astronaut manually overwrote W-matrix initial values
;              W-matrix contains navigation state covariance for Kalman filtering
;   - UPLOCKFL: Indicates K-KBAR-K update computation failed (uplink lock failure)
;               Prevents bad navigation updates from corrupting state vector
;
; PROGRAM CONTROL FLAGS:
;   - VERIFLAG: Changed when V33E (proceed with external delta-V) occurs at
;               end of P27 (update program), tracking crew input acceptance
;   - V82EMFLG: Indicates spacecraft vicinity (Moon vs Earth) for coordinate
;               frame selection and ephemeris computation
;   - TFFSW: Selects between calculating time of free fall (TFF) or time to
;            perigee (TPERIGEE) in trajectory computations
;
; MISSION CONTEXT:
; During Apollo 11's descent, SWANDISP was SET to enable the landing analog
; displays that showed altitude rate and horizontal velocity. Armstrong and
; Aldrin monitored these displays continuously during the final approach,
; with Armstrong calling out "Down two and a half" (feet per second descent
; rate) repeatedly during the last moments before touchdown.
;
; ASTNFLAG provided the crew safety interlock for engine ignition. Before
; any powered burn (descent engine, ascent engine), this flag had to be SET
; by astronaut action (typically PRO key on DSKY) to permit ignition. This
; prevented uncommanded engine starts due to software or hardware faults.
;
; IDLEFLAG and AVEGFLAG controlled the servicer routine's monitoring of
; spacecraft acceleration and velocity changes, essential for verifying that
; RCS thruster firings achieved intended velocity corrections and detecting
; anomalous accelerations that might indicate system failures.
;
; CROSS-REFERENCES:
;   - THE_LUNAR_LANDING.agc: Sets SWANDISP to enable descent displays
;   - LANDING_ANALOG_DISPLAYS.agc: Uses SWANDISP to control display updates
;   - P40-P47.agc: Sets IGNFLAG and ASTNFLAG for SPS/DPS burn sequencing
;   - SERVICER.agc: Uses IDLEFLAG, AVEGFLAG, V37FLAG for background tasks
;   - P32-P35_P72-P75.agc: Uses NORMSW and RVSW for Lambert targeting modes
;   - P34-35_P74-75.agc: Sets ITSWICH for TPI computation control
;   - UPDATE_PROGRAM.agc: Uses VERIFLAG to track P27 crew acceptance
;   - MEASUREMENT_INCORPORATION.agc: Uses UPLOCKFL to validate nav updates
; ============================================================================

FLAGWRD7	=	STATE +7		# (105-119)

						#	(SET)			(RESET)

# BIT 15 FLAG 7	(S)
ITSWICH		=	105D			#	R34;TPI TIME TO BE	TPI HAS BEEN
ITSWBIT		=	BIT15			#	COMPUTED		COMPUTED

# BIT 14 FLAG 7	(S)
MANUFLAG	=	106D			#	ATTITUDE MANEUVER	NO ATTITUDE MANEUVER
						#	GOING DURING RR		DURING RR SEARCH
MANUFBIT	=	BIT14			#	SEARCH

# BIT 13 FLAG 7	(S)
IGNFLAG		=	107D			#	TIG HAS ARRIVED		TIG HAS NOT ARRIVED
IGNFLBIT	=	BIT13			#

# BIT 12 FLAG 7	(S)
ASTNFLAG	=	108D			#	ASTRONAUT HAS		ASTRONAUT HAS NOT
ASTNBIT		=	BIT12			#	OKAYED IGNITION		OKAYED IGNITION

# BIT 11 FLAG 7	(L)
SWANDISP	=	109D			#	LANDING ANALOG		LANDING ANALOG
SWANDBIT	=	BIT11			#	DISPLAYS ENABLED	DISPLAYS SUPPRESSED

# BIT 10 FLAG 7	(S)
NORMSW		=	110D			#	UNIT NORMAL INPUT	LAMBERT COMPUTES ITS
NORMSBIT	=	BIT10			#	TO LAMBERT		OWN UNIT NORMAL

# BIT 9 FLAG 7	(S)
RVSW		=	111D			#	DO NOT COMPUTE		COMPUTE FINAL STATE
						#	FINAL STATE VECTOR	VECTOR IN TIME-THETA
RVSWBIT		=	BIT9			#	IN TIME-DELTA

# BIT 8 FLAG 7	(S)
V67FLAG		=	112D			#	ASTRONAUT OVERWRITE	ASTRONAUT DOES NOT
						#	W-MATRIX INITIAL	OVERWRITE W-MATRIX
V67FLBIT	=	BIT8			#	VALUES			INITIAL VALUES

# Page 78
# BIT 7 FLAG 7	(S)
IDLEFLAG	=	113D			#	NO DV MONITOR		CONNECT DV MONITOR
IDLEFBIT	=	BIT7			#

# BIT 6 FLAG 7	(S)
V37FLAG		=	114D			#	AVERAGEG (SERVICER)	AVERAGEG (SERVICER)
V37FLBIT	=	BIT6			#	RUNNING			OFF

# BIT 5 FLAG 7	(S)
AVEGFLAG	=	115D			#	AVERAGEG (SERVICER)	AVERAGEG (SERVICER)
AVEGFBIT	=	BIT5			#	DESIRED			NOT DESIRED

# BIT 4 FLAG 7	(S)
UPLOCKFL	=	116D			#	K-KBAR-K FAIL		NO K-KBAR-K FAIL
UPLOCBIT	=	BIT4			#

# BIT 3 FLAG 7	(S)
VERIFLAG	=	117D			# CHANGED WHEN V33E OCCURS AT END OF P27
VERIFBIT	=	BIT3			#

# BIT 2 FLAG 7	(L,C)
V82EMFLG	=	118D			#	MOON VICINITY		EARTH VICINITY
V82EMBIT	=	BIT2			#

# BIT 1 FLAG 7	(S)
TFFSW		=	119D			#	CALCULATE TPERIGEE	CALCULATE TFF
TFFSWBIT	=	BIT1			#


; ============================================================================
; FLAGWORD 8: SPHERE OF INFLUENCE, SURFACE STATUS, AND TRAJECTORY FLAGS
;
; This flagword contains critical flags for coordinate system management,
; spacecraft location tracking, trajectory computation modes, and conic
; solution handling. Several flags are PROTECTED FROM FRESH START to preserve
; essential state information across computer restarts.
;
; SPHERE OF INFLUENCE FLAGS (PROTECTED FROM FRESH START):
;   - CMOONFLG: Indicates Command Module permanent state in lunar sphere
;               (SET) or Earth sphere (RESET) of gravitational influence
;               Determines primary gravitational body for trajectory computation
;   - LMOONFLG: Indicates Lunar Module permanent state in lunar sphere
;               (SET) or Earth sphere (RESET) of gravitational influence
;               Essential for proper ephemeris and coordinate frame selection
;
; SURFACE OPERATION FLAG (PROTECTED FROM FRESH START):
;   - SURFFLAG: Indicates LM is on lunar surface (SET) or in flight (RESET)
;               This flag is SET at touchdown and remains SET during surface
;               operations (21.5 hours for Apollo 11). Critical for:
;               * Disabling descent programs that require flight state
;               * Enabling surface navigation and alignment modes
;               * Coordinating ascent preparation and ignition sequencing
;               * Preventing inadvertent guidance mode changes on surface
;               PROTECTED to preserve surface state across any restart events
;               during critical surface operations or ascent countdown.
;
; CONIC TRAJECTORY SOLUTION FLAGS:
;   - INFINFLG: Indicates no conic solution exists, requiring closure through
;               infinity (SET) vs normal conic solution exists (RESET)
;               Occurs when trajectory hyperbolic escape or insufficient data
;   - COGAFLAG: Indicates COGA (Conic Orbital Guidance) routine overflow due
;               to near-rectilinear trajectory (SET) vs valid conic (RESET)
;   - APSESW: Indicates RDESIRED (desired radius) is outside (SET) or inside
;             (RESET) the pericenter-apocenter range in time-radius iteration
;   - ORDERSW: Controls whether Lambert iterator uses second-order minimum
;              mode (SET) or first-order standard mode (RESET) for convergence
;
; INTEGRATION FLAGS:
;   - NEWIFLG: Indicates first pass through integration routine (SET) vs
;              succeeding iteration of integration (RESET)
;              Used by ORBITAL_INTEGRATION.agc for initialization control
;   - RPQ FLAG: Indicates RPQ vector (between secondary body and primary body)
;               not yet computed (SET) vs already computed (RESET)
;
; DISPLAY FLAGS:
;   - FLUNDISP: Controls whether current guidance displays are inhibited (SET)
;               or permitted (RESET). Used during mode transitions to prevent
;               confusing or invalid display data from appearing on DSKY.
;
; PROGRAM SELECTION FLAGS:
;   - P39/79SW: Indicates P39 or P79 programs operating (SET) vs P38 or P78
;               programs operating (RESET). Selects between rendezvous program
;               variants for different mission phases.
;
; MISSION CONTEXT:
; SURFFLAG was one of the most critical flags in the Apollo 11 mission. At
; the moment of touchdown (102:45:40 MET, July 20, 1969), when the contact
; probes touched the lunar surface and Armstrong announced "Contact light",
; SURFFLAG was SET. This single bit transition:
; - Terminated the descent guidance programs
; - Disabled throttle control (engine shutdown followed immediately)
; - Enabled surface navigation modes for star alignment
; - Protected against inadvertent guidance mode activation
; - Persisted through the 21.5-hour surface stay
; - Coordinated with ascent program initialization
;
; The PROTECTED status of SURFFLAG, CMOONFLG, and LMOONFLG was essential.
; If a restart occurred during surface operations (none did on Apollo 11,
; but hardware transients were possible), these flags had to survive to
; maintain correct operational context. Losing SURFFLAG during surface stay
; could have caused the computer to think it was still in powered descent,
; with potentially dangerous consequences for ascent preparation.
;
; The sphere of influence flags (CMOONFLG/LMOONFLG) managed the transition
; between Earth-centered and Moon-centered coordinate systems during the
; translunar and transearth coast phases. Proper flag state ensured the
; correct gravitational models and ephemeris calculations were used.
;
; CROSS-REFERENCES:
;   - THE_LUNAR_LANDING.agc: Sets SURFFLAG at touchdown detection
;   - ASCENT_GUIDANCE.agc: Checks SURFFLAG before ascent ignition
;   - FRESH_START_AND_RESTART.agc: Protects SURFFLAG/CMOONFLG/LMOONFLG
;   - ORBITAL_INTEGRATION.agc: Uses NEWIFLG, CMOONFLG, LMOONFLG for setup
;   - CONIC_SUBROUTINES.agc: Uses INFINFLG, COGAFLAG, APSESW, ORDERSW
;   - P39_P79.agc: Uses P39/79SW for program variant selection
;   - DISPLAY_INTERFACE_ROUTINES.agc: Uses FLUNDISP to control updates
;   - PLANETARY_INERTIAL_ORIENTATION.agc: Uses sphere flags for frame selection
; ============================================================================

FLAGWRD8	=	STATE +8D		# (120-134)

						#	(SET)			(RESET)

# BIT 15 FLAG 8	(S)
RPQFLAG		=	120D			#	RPQ NOT COMPUTED	RPQ COMPUTED
						#	(RPQ = VECTOR BE-
RPQFLBIT	=	BIT15			#	TWEEN SECONDARY BODY
						#	AND PRIMARY BODY)

# BIT 14 FLAG 8
		=	121D			#
		=	BIT14			#

# Page 79
# BIT 13 FLAG 8	(S)
NEWIFLG		=	122D			#	FIRST PASS THROUGH	SUCCEEDING ITERATION
NEWIBIT		=	BIT13			#	INTEGRATION		OF INTEGRATION

# BIT 12 FLAG 8	*** PROTECTED FROM FRESH START ***
CMOONFLG	=	123D			#	PERMANENT CSM STATE	PERMANENT CSM STATE
CMOONBIT	=	BIT12			#	IN LUNAR SPHERE		IN EARTH SPHERE

# BIT 11 FLAG 8	*** PROTECTED FROM FRESH START ***
LMOONFLG	=	124D			#	PERMANENT LM STATE	PERMANENT LM STATE
LMOONBIT	=	BIT11			#	IN LUNAR SPHERE		IN EARTH SPHERE

# BIT 10 FLAG 8	(L)
FLUNDISP	=	125D			#	CURRENT GUIDANCE	CURRENT GUIDANCE
FLUNDBIT	=	BIT10			#	DISPLAYS INHIBITED	DISPLAYS PERMITTED

# BIT 9 FLAG 8	(L)
P39/79SW	=	126D			#	P39/79 OPERATING	P38/78 OPERATING
P39SWBIT	=	BIT9			#

# BIT 8 FLAG 8	*** PROTECTED FROM FRESH START ***
SURFFLAG	=	127D			#	LM ON LUNAR SURFACE	LM NOT ON LUNAR
SURFFBIT	=	BIT8			#				SURFACE

# BIT 7 FLAG 8	(S)
INFINFLG	=	128D			#	NO CONIC SOLUTION	CONIC SOLUTION
						#	(CLOSURE THROUGH	EXISTS
INFINBIT	=	BIT7			#	INFINITY REQUIRED)

# BIT 6 FLAG 8	(S)
ORDERSW		=	129D			#	ITERATOR USES 2ND	ITERATOR USES 1ST
ORDERBIT	=	BIT6			#	ORDER MINIMUM MODE	ORDER STANDARD MODE

# BIT 5 FLAG 8	(S)
APSESW		=	130D			#	RDESIRED OUTSIDE	RDESIRED INSIDE
						#	PERICENTER-APOCENTER	PERICENTER-APOCENTER
APSESBIT	=	BIT5			#	RANGE IN TIME-RADIUS	RANGE IN TIME-RADIUS

# BIT 4 FLAG 8	(S)
COGAFLAG	=	131D			#	NO CONIC SOLUTION --	CONIC SOLUTION
						#	TOO CLOSE TO RECTI-	EXISTS (COGA DOES NOT

# Page 80
COGAFBIT	=	BIT4			#	LINEAR (COGA OVERFLWS)	OVERFLOW)

# BIT 3 FLAG 8	(S)
		=	132D			#
		=	BIT3			# OH 2009-05-15 Line not in scan

# BIT 2 FLAG 8	(L)
INITALGN	=	133D			#	INITIAL PASS THRU	SECOND PASS THRU P57
INITABIT	=	BIT2			#	P57			(CHECK RESET-MILLARD)

# BIT 1 FLAG 8	(S)
360SW		=	134D			#	TRANSFER ANGLE NEAR	TRANSFER ANGLE NOT
360SWBIT	=	BIT1			#	360 DEGREES		NEAR 360 DEGREES

; ----------------------------------------------------------------------------
; FLAGWRD9 (Bits 135-149): ASCENT GUIDANCE AND INTEGRATION CONTROL
;
; This flagword manages ascent guidance modes, abort control, and numerical
; integration parameters used during powered flight from the lunar surface.
;
; ASCENT GUIDANCE FLAGS (FLVR, FLPC, FLPI, FLAP, FLRCS):
; - Control guidance law selection during lunar ascent
; - Enable/disable specific guidance components (velocity, position, attitude)
; - Coordinate thrust vector control and RCS jet selection
; - Used heavily in ASCENT_GUIDANCE.agc and P12.agc during Eagle's return
;
; ABORT FLAGS (LETABORT):
; - Enable abort mode capability
; - Interfaces with P70-P71.agc abort programs
; - Critical safety feature for crew emergency return
;
; INTEGRATION CONTROL FLAGS (AVEMIDSW, MIDFLAG, MID1FLAG-MID3FLAG):
; - Control orbital integration sequencing
; - Manage Encke method rectification intervals
; - Coordinate state vector propagation in ORBITAL_INTEGRATION.agc
;
; MISSION CONTEXT: These flags were active during Eagle's ascent from
; Tranquility Base on July 21, 1969, managing the 7-minute powered climb
; to rendezvous orbit with Columbia.
; ----------------------------------------------------------------------------

FLAGWRD9	=	STATE +9D		# (135-149)

						#	(SET)			(RESET)

# BIT 15 FLAG 9
		=	135D			#
		=	BIT15			#

# BIT 14 FLAG 9	(L)
FLVR		=	136D			#	VERTICAL RISE		NON-VERTICAL RISE
FLVRBIT		=	BIT14			#	(ASCENT GUIDANCE)

# BIT 13 FLAG 9
		=	137D			#
		=	BIT13			# OH 2009-05-15 Line not in scan

# BIT 12 FLAG 9	(L)
FLPC		=	138D			#	NO POSITION CONTROL	POSITION CONTROL
FLPCBIT		=	BIT12			#	(ASCENT GUIDANCE)

# BIT 11 FLAG 9	(L)
FLPI		=	139D			#	PRE-IGNITION PHASE	REGULAR GUIDANCE
FLPIBIT		=	BIT11			#	(ASCENT GUIDANCE)

# BIT 10 FLAG 9	(L)
FLRCS		=	140D			#	RCS INJECTION MODE	MAIN ENGINE MODE
FLRCSBIT	=	BIT10			#	(ASCENT GUIDANCE)

# BIT 9 FLAG 9	(L)

# Page 81
LETABORT	=	141D			#	ABORT PROGRAMS		ABORT PROGRAMS
LETABBIT	=	BIT9			#	ARE ENABLED		ARE NOT ENABLED

# BIT 8 FLAG 9	(L)
FLAP		=	142D			#	APS CONTINUED ABORT	APS ABORT IS NOT A
						#	AFTER DPS STAGING	CONTINUATION
FLAPBIT		=	BIT8			#	(ASCENT GUIDANCE)

# BIT 7 FLAG 9	(L)
		=	143D
		=	BIT7			# OH 2009-05-15 Line not in scan


# BIT 6 FLAG 9	(L)
ROTFLAG		=	144D			#	P70 AND P71 WILL	P70 AND P71 WILL NOT
ROTFLBIT	=	BIT6			#	FORCE VEHICLE		FORCE VEHICLE
						#	ROTATION IN THE		ROTATION IN THE
						#	PREFERRED DIRECTION	PREFERRED DIRECTION

# BIT 5 FLAG 9	(S)
QUITFLAG	=	145D			#	DISCONTINUE INTEGR.	CONTINUE INTEGRATION
QUITBIT		=	BIT5			#

# BIT 4 FLAG 9
		=	146D			#
		=	BIT4			#

# BIT 3 FLAG 9	(L)
MID1FLAG	=	147D			#	INTEGRAT TO TDEC	INTEGRATE TO THE
MID1FBIT	=	BIT3			#				THEN-PRESENT TIME

# BIT 2 FLAG 9	(L)
MIDAVFLG	=	148D			#	INTEGRATION ENTERED	INTEGRATION WAS
						#	FROM ONE OF MIDTOAV	NOT ENTERED VIA
MIDAVBIT	=	BIT2			#	PORTALS			MIDTOAV

# BIT 1 FLAG 9	(S)
AVEMIDSW	=	149D			#	AVETOMID CALLING	NO AVETOMID W INTEGR
						#	FOR W.MATRIX INTEGR	ALLOW SET UP RM, VN
AVEMDBIT	=	BIT1			#	DON'T WRITE OVER RN, 	PIPTIME
						#	VN,PIPTIME


RASFLAG		EQUALS	FLGWRD10		# WAS ONLY AN INSTALL-ERASTALL FLAG

; ----------------------------------------------------------------------------
; FLGWRD10 (Bits 150-164): INTEGRATION STATUS AND STAGE CONFIGURATION
;
; This flagword manages orbital integration state tracking and the critical
; ascent/descent stage configuration flag.
;
; INTEGRATION FLAGS (INTFLAG, REINTFLG):
; - Track whether orbital integration computation is currently active
; - Control restart behavior for integration routines
; - Interface with ORBITAL_INTEGRATION.agc state vector propagation
;
; STAGE CONFIGURATION FLAG (APSFLAG):
; - **CRITICAL**: Indicates Ascent Propulsion System (APS) vs Descent (DPS)
; - SET = Ascent stage active, RESET = Descent stage active
; - *** PROTECTED FROM FRESH START *** (survives system resets)
; - This flag changed state when Eagle jettisoned its descent stage after
;   liftoff from the lunar surface, permanently reconfiguring the LM from
;   landing to rendezvous configuration
; - Used throughout guidance and control software to select stage-specific
;   parameters (mass properties, thrust levels, RCS configurations)
;
; MISSION CONTEXT: APSFLAG was SET at approximately 124:22:00 MET when
; Eagle's ascent engine ignited for the return to orbit, and remained SET
; throughout rendezvous with Columbia.
; ----------------------------------------------------------------------------

# Page 82
FLGWRD10	=	STATE +10D		# (150-164)

						#	(SET)			(RESET)

# BIT 15 FLAG 10 (S)
		=	150D			#
		=	BIT15			# OH 2009-05-15 Line not in scan

# BIT 14 FLAG 10 (L,C)
INTFLAG		=	151D			#	INTEGRATION IN		INTEGRATION NOT IN
INTFLBIT	=	BIT14			#	PROGRESS		PROGRESS

# BIT 13 FLAG 10 (S,L)
APSFLAG		=	152D			#	ASCENT STAGE		DESCENT STAGE
APSFLBIT	=	BIT13			#	 *** PROTECTED FROM FRESH START ***

# BIT 12 FLAG 10
		=	153D			#
		=	BIT12			# OH 2009-05-15 Line not in scan

# BIT 11 FLAG 10
		=	154D			#
		=	BIT11			# OH 2009-05-15 Line not in scan

# BIT 10 FLAG 10
		=	155D			#
		=	BIT10			# OH 2009-05-15 Line not in scan

# BIT 9 FLAG 10
		=	156D			#
		=	BIT9			# OH 2009-05-15 Line not in scan

# BIT 8 FLAG 10
		=	157D			#
		=	BIT8			# OH 2009-05-15 Line not in scan

# BIT 7 FLAG 10 (L,C)
REINTFLG	=	158D			#	INTEGRATION ROUTINE	INTEGRATION ROUTINE
REINTBIT	=	BIT7			#	TO BE RESTARTED		NOT TO BE RESTARTED

# BIT 6 FLAG 10
		=	159D			#
		=	BIT6			# OH 2009-05-15 Line not in scan

# BIT 5 FLAG 10
		=	160D			#
		=	BIT5			# OH 2009-05-15 Line not in scan

# Page 83
# BIT 4 FLAG 10
		=	161D			#
		=	BIT4			# OH 2009-05-15 Line not in scan

# BIT 3 FLAG 10
		=	162D			#
		=	BIT3			# OH 2009-05-15 Line not in scan

# BIT 2 FLAG 10
		=	163D			#
		=	BIT2			# OH 2009-05-15 Line not in scan

# BIT 1 FLAG 10
		=	164D			#
		=	BIT1			# OH 2009-05-15 Line not in scan



; ============================================================================
; FLAGWORD 11: LANDING RADAR CONTROL FLAGS
;
; This flagword is dedicated entirely to landing radar (LR) management during
; powered descent. The landing radar provides critical altitude and velocity
; measurements during the final 50,000 feet of descent to the lunar surface.
;
; During Apollo 11's descent on July 20, 1969, these flags controlled:
; - Whether to accept or bypass radar altitude measurements
; - Whether to accept or bypass radar velocity measurements
; - Status of radar data validity and quality
; - Radar repositioning and lock-on states
; - Crew control over radar data incorporation
;
; The landing radar was essential for Armstrong and Aldrin's safe touchdown,
; providing the altitude and descent rate information displayed on the DSKY
; and fed to the guidance equations in LUNAR_LANDING_GUIDANCE_EQUATIONS.agc.
; ============================================================================

FLGWRD11	=	STATE +11D		# (165-179)

						#	(SET)			(RESET)

; ----------------------------------------------------------------------------
; LRBYPASS - Landing Radar Bypass Control (Part of FLAGORGY flags)
;
; This flag enables complete bypass of landing radar altitude and velocity
; data updates to the navigation state. When SET, all LR measurements are
; ignored regardless of their validity or quality.
;
; MISSION CONTEXT: During nominal Apollo 11 descent, this flag remained RESET,
; allowing radar data to update the guidance system. It could be SET by crew
; if radar malfunctioned or provided erroneous data. The flag provided abort
; protection - if radar failed, crew could land using inertial data alone.
;
; CROSS-REFERENCES:
;   - THE_LUNAR_LANDING.agc: Part of FLAGORGY initialization
;   - LUNAR_LANDING_GUIDANCE_EQUATIONS.agc: Checks this flag before using LR
;   - RADAR_LEADIN_ROUTINES.agc: Radar interface respects this bypass flag
; ----------------------------------------------------------------------------
# BIT 15 FLAG 11 (L)(R12)
LRBYPASS	=	165D			#	BYPASS ALL LANDING	DO NOT BYPASS LR
LRBYBIT		=	BIT15			#	RADAR UPDATES		UPDATES

# BIT 14 FLAG 11
		=	166D			#
		=	BIT14			#

# BIT 13 FLAG 11
		=	167D			#
		=	BIT13			#

# BIT 12 FLAG 11 (L)(R12)
VXINH		=	168D			#	IF Z VELOCITY DATA	UPDATE X AXIS
						#	UNREASONABLE,		VELOCITY
VXINHBIT	=	BIT12			#	BYPASS X VELOCITY
						#	UPDATE ON NEXT PASS

# BIT 11 FLAG 11 (L)(R12)
PSTHIGAT	=	169D			#	PAST HIGATE		PREHIGATE
PSTHIBIT	=	BIT11			#

# BIT 10 FLAG 11 (L)(R12)

# Page 84
NOLRREAD	=	170D			#	LANDING RADAR		LR NOT REPOSITIONING
						#	REPOSITIONING;
NOLRRBIT	=	BIT10			#	BYPASS UPDATE

# BIT 9 FLAG 11 (L)(R12)
XORFLG		=	171D			#	BELOW LIMIT		ABOVE LIMIT DO
						#	INHIBIT X AXIS		NOT INHIBIT
XORFLBIT	=	BIT9			#	OVERRIDE

# BIT 8 FLAG 11
LRINH		=	172D			#	LANDING RADAR UP-	LR UPDATES INHIBITED
LRINHBIT	=	BIT8			#	DATES PERMITTED		BY ASTRONAUT
						#	BY ASTRONAUT

# BIT 7 FLAG 11	(L)(R12)
VELDATA		=	173D			#	LR VELOCITY		LR VELOCITY MEASURE
VELDABIT	=	BIT7			#	MEASUREMENT MADE	NOT MADE

# BIT 6 FLAG 11	(L)(R12)
READLR		=	174D			#	OK TO READ LR		DO NOT READ LR RANGE
READLBIT	=	BIT6			#	RANGE DATA		DATA

# BIT 5 FLAG 11	(L)(R12)
READVEL		=	175D			#	OK TO READ LR		DO NOT READ LR
READVBIT	=	BIT5			#	VELOCITY DATA		VELOCITY DATA

# BIT 4 FLAG 11	(L)(R12)
RNGEDATA	=	176D			#	LR ALTITUDE		LR ALTITUDE MEASURE
RNGEDBIT	=	BIT4			#	MEASUREMENT MADE	NOT MADE

# BIT 3 FLAG 11
SCALBAD		=	177D			#	LR LOW SCALE DISP-	LS SCALE DISCRETE
SCABBIT		=	BIT3			#	CRETE NOT PRESENT	APPEARS OK
						# 	WHEN IT SHOULD

# BIT 2 FLAG 11	(L)(R12)
VFLSHFLG	=	178D			#	LR VELOCITY FAIL	LR VEL FAIL LAMP
						#	LAMP SHOULD BE		SHOULDN'T FLASH
VFLSHBIT	=	BIT2			#	FLASHING

# BIT 1 FLAG 11	(L)(R12)
# Page 85
HFLSHFLG	=	179D			#	LR ALTITUDE FAIL	LR ALTITUDE FAIL
HFLSHBIT	=	BIT1			#	LAMP SHOULD BE		LAMP SHOULD NOT BE
						#	FLASHING		FLASHING

RADMODES	EQUALS	FLGWRD12		# RADAR FLAG WORD

; ============================================================================
; FLAGWORD 12: RADAR CONTROL AND STATUS FLAGS (formerly RADMODES)
;
; This flagword manages all radar system operations for both the Landing Radar
; (LR) and Rendezvous Radar (RR). It controls radar modes, monitors data
; validity, tracks positioning sequences, and coordinates radar measurements
; with the navigation state.
;
; LANDING RADAR (LR) FLAGS:
;   - Altitude scale selection (high/low range)
;   - Position selection (antenna position 1 or 2)
;   - Data validity (altitude fail, velocity fail flags)
;   - Measurement quality monitoring
;
; RENDEZVOUS RADAR (RR) FLAGS:
;   - Antenna mode selection (mode 1 or 2)
;   - Designate and reposition status
;   - CDU zeroing operations
;   - Auto mode control
;   - Data failure detection
;   - Range scale selection (high/low)
;
; MISSION CONTEXT:
; During Apollo 11's descent on July 20, 1969, these flags managed the landing
; radar that provided altitude and velocity data crucial for the final approach.
; During ascent and rendezvous on July 21, these flags coordinated rendezvous
; radar tracking of Columbia for relative navigation.
;
; CROSS-REFERENCES:
;   - RADAR_LEADIN_ROUTINES.agc: Primary radar interface routines
;   - LUNAR_LANDING_GUIDANCE_EQUATIONS.agc: Uses LR data quality flags
;   - P20-P25.agc: Uses RR status and control flags
;   - THE_LUNAR_LANDING.agc: Monitors radar status during descent
; ============================================================================

FLGWRD12	=	STATE +12D		# (180-194)		WAS RADMODES

						#	(SET)			(RESET)

# BIT 15 FLAG 12
CDESFLAG	=	180D			#	CONTINUOUS DESIG-	LGC CHECKS FOR LOCK-
CDESBIT		=	BIT15			#	NATE, LGC COMMANDS	ON WHEN ANTENNA
						#	RR REGARDLESS OF	BEING DESIGNATED
						#	LOCK-ON

# BIT 14 FLAG 12
REMODFLG	=	181D			#	CHANGE IN ANTENNA	NO REMODE REQUESTED
REMODBIT	=	BIT14			#	MODE BEEN REQUESTED	OR OCCURRING
						#	I.E., REMODE

# BIT 13 FLAG 12
RCDU0FLG	=	182D			#	RR CDU'S BEING		RR CDU'S NOT BEING
RCDU0BIT	=	BIT13			#	ZEROED			ZEROED

# BIT 12 FLAG 12
ANTENFLG	=	183D			#	RR ANTENNA MODE IS	RR ANTENNA IN MODE 1
ANTENBIT	=	BIT12			#	MODE 2

# BIT 11 FLAG 12
REPOSMON	=	184D			#	REPOSITION MONITOR.	NO REPOSITION TAKING
REPOSBIT	=	BIT11			#	RR REPOSITION IS	PLACE
						#	TAKING PLACE

# BIT 10 FLAG 12
DESIGFLG	=	185D			#	RR DESIGNATE		RR DESIGNATE NOT
DESIGBIT	=	BIT10			#	REQUESTED OR IN		REQUESTED OR IN
						#	PROGRESS		PROGRESS

# BIT 9 FLAG 12
ALTSCALE	=	186D			#	LR ALTITUDE READING	LR ALTITUDE READING
ALTSCBIT	=	BIT9			#	IS ON HIGH SCALE	IS ON LOW SCALE

# Page 86
# BIT 8 FLAG 12
LRVELFLG	=	187D			#	LR VELOCITY DATA	NO LR VELOCITY DATA
LRVELBIT	=	BIT8			#	FAIL			FAIL

# BIT 7 FLAG 12
RCDUFAIL	=	188D			#	RR CDU FAIL HAS		RR CDU FAIL OCCURRED
RCDUFBIT	=	BIT7			#	NOT OCCURRED

# BIT 6 FLAG 12
LRPOSFLG	=	189D			#	LANDING RADAR		LR POSITION 1
LRPOSBIT	=	BIT6			#	POSITION 2

# BIT 5 FLAG 12
LRALTFLG	=	190D			#	LR ALTITUDE DATA	NO LR ALTITUDE DATA
LRALTBIT	=	BIT5			#	FAIL.  COULD NOT BE	FAIL
						#	READ SUCCESSFULLY.

# BIT 4 FLAG 12
RRDATAFL	=	191D			#	RR DATA FAIL.		NO RR DATA FAIL
RRDATABT	=	BIT4			#	DATA COULD NOT BE
						# 	READ SUCCESSFULLY

# BIT 3 FLAG 12
RRRSFLAG	=	192D			#	RR RANGE READING	RR RANGE READING ON
RRRSBIT		=	BIT3			#	ON THE HIGH SCALE	THE LOW SCALE

# BIT 2 FLAG 12
AUTOMODE	=	193D			#	RR NOT IN AUTO MODE.	RR IN AUTO MODE
AUTOMBIT	=	BIT2			#	AUTO MODE DISCRETE
						#	IS NOT PRESENT

# BIT 1 FLAG 12
TURNONFL	=	194D			#	RR TURN-ON SEQUENCE	NO RR TURN-ON
TURNONBT	=	BIT1			#	IN PROGRESS.  (ZERO	SEQUENCE IN PROGRESS
						#	CDU'S, FIX ANTENNA
						#	MODE)

DAPBOOLS	EQUALS	FLGWRD13		# DIGITAL AUTOPILOT FLAGWORD

; ============================================================================
; FLAGWORD 13: DIGITAL AUTOPILOT CONTROL FLAGS (formerly DAPBOOLS)
;
; This flagword manages the Digital Autopilot (DAP), which controls spacecraft
; attitude and translation using the Reaction Control System (RCS) thrusters.
; The DAP is the primary flight control system for the Lunar Module during all
; mission phases when the main engines are not firing.
;
; DAP CONFIGURATION FLAGS:
;   - RCS jet selection (4-jet, 2-jet translation modes)
;   - Automatic vs manual attitude control
;   - Minimum impulse mode (fuel conservation)
;   - Ullage control during engine burns
;
; DOCKING AND RENDEZVOUS FLAGS:
;   - CSM docked status (changes mass properties and RCS configuration)
;   - Orbit rate (affects attitude control gains)
;   - Docking axis selection
;
; MANEUVER CONTROL FLAGS:
;   - Automatic rotation rate selection
;   - Attitude hold modes
;   - Translation enable/disable
;   - Gimbal drive test modes
;
; MISSION CONTEXT:
; The DAP maintained LM attitude throughout all mission phases: during lunar
; orbit, the descent to the surface, Neil Armstrong's manual landing site
; selection, the ascent from Tranquility Base, and rendezvous with Columbia.
; These flags configured the DAP for each mission phase, optimizing thruster
; usage to conserve precious RCS propellant.
;
; CROSS-REFERENCES:
;   - P-AXIS_RCS_AUTOPILOT.agc: Pitch axis control using these flags
;   - Q_R-AXIS_RCS_AUTOPILOT.agc: Yaw/roll axis control using these flags
;   - TJET_LAW.agc: Jet firing logic responds to these configuration flags
;   - DAPIDLER_PROGRAM.agc: DAP background processing and flag monitoring
; ============================================================================

# Page 87
FLGWRD13	=	STATE +13D		# (195-209)	WAS DAPBOOLS

						#	(SET)			(RESET)

# BIT 15 FLAG 13
PULSEFLG	=	195D			#	MINIMUM IMPUSE		NOT IN MINIMUM
PULSES		=	BIT15			#	COMMAND MODE IN		IMPULSE COMMAND MODE
						#	"ATT HOLD" (V76)	(V77)

# BIT 14 FLAG 13
USEQRFLG	=	196D			#	GIMBAL UNUSABLE.	TRIM GIMBAL MAY BE
USEQRJTS	=	BIT14			#	USE JETS ONLY.		USED.

# BIT 13 FLAG 13
CSMDKFLG	=	197D			#	CSM DOCKED.  USE	CSM NOT DOCKED TO LM
CSMDOCKD	=	BIT13			#	BACKUP DAP

# BIT 12 FLAG 13
OURRCFLG	=	198D			#	CURRENT DAP PASS	CURRENT DAP PASS IS
OURRCBIT	=	BIT12			#	IS RATE COMMAND		NOT RATE COMMAND

# BIT 11 FLAG 13
ACC4-2FL	=	199D			#	4 JET X-AXIS TRANS-	2 JET X-AXIS TRANS-
ACC4OR2X	=	BIT11			#	LATION REQUESTED	LATION REQUESTED

# BIT 10 FLAG 13
AORBTFLG	=	200D			#	B SYSTEM FOR X-		A SYSTEM FOR X-
AORBTRAN	=	BIT10			#	TRANSLATION		TRANSLATION PREFER'D

# BIT 9 FLAG 13
XOVINFLG	=	201D			#	X-AXIS OVERRIDE		X-AXIS OVERRIDE OKAY
XOVINHIB	=	BIT9			#	LOCKED OUT

# BIT 8 FLAG 13
DRIFTDFL	=	202D			#	ASSUME 0 OFFSET		USE OFFSET ACCELERA-
DRIFTBIT	=	BIT8			#	DRIFTING FLIGHT		ION ESTIMATE

# BIT 7 FLAG 13
RHCSCFLG	=	203D			#	NORMAL RHC SCALING	FINE RHC SCALING
RHCSCALE	=	BIT7			#	REQUESTED		REQUESTED

# Page 88
# BIT 6 FLAG 13
ULLAGFLG	=	204D			#	ULLAGE REQUEST BY	NO INTERNAL ULLAGE
ULLAGER		=	BIT6			#	MISSION PROGRAM		REQUEST

# BIT 5 FLAG 13
AORBSFLG	=	205D			#	P-AXIS COUPLES 7.15	P-AXIS COUPLES 4.12
AORBSYST	=	BIT5			#	AND 8.16 PREFERRED	AND 3.11 PREFERRED

# BIT 4 FLAG 13
DBSELFLG	=	206D			#	MAX DB SELECTED		MIN DB SELECTED BY
DBSELECT	=	BIT4			#	BY CREW	(5 DEG)		CREW (0.3 DEG)

# BIT 3 FLAG 13
ACCOKFLG	=	207D			#	CONTROL AUTHORITY	RESTART OR FRESH ST.
ACCSOKAY	=	BIT3			#	VALUES FROM 1/ACCS	SINCE LAST 1/ACCS;
						#	USABLE			OUTPUTS SUSPECT.

# BIT 2 FLAG 13
AUTR2FLG	=	208D			# THESE FLAGS ARE USED TOGETHER TO INDICATE
AUTRATE2	=	BIT2			# ASTRONAUT-CHOSEN KALCMANU MANEUVER RATES
						# (0,0)=(BIT2,BIT1)=	0.2 DEG/SEC
# BIT 1 FLAG 13					# (0,1)= 		0.5 DEG/SEC
AUTR1FLG	=	209D			# (1,0)=		2.0 DEG/SEC
AUTRATE1	=	BIT1			# (1,1)=	       10.0 DEG/SEC

# Page 89 (nothing on this page)

